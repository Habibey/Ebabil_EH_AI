"""
İKİNCİ PlutoSDR'ı RX olarak kullanıp 144-148 MHz, 430-440 MHz, 863-870 MHz,
2.4-2.483 GHz ve 5.725-5.875 GHz bantlarını hoplayarak tarayan ED (Elektronik
Destek) süreci -- streamer.py'nin RTL-SDR (144/433/868 MHz) tarafına ek, KTR'de
tanımlı ama şu ana kadar kodda hiç var olmayan üst-bant (2.4G/5.8G) tarama
zincirini gerçekleştirir (alt bantlar sahada test amaçlı ayrıca burada da
taranıyor, bkz. BANDS altındaki not).

Neden AYRI süreç ve AYRI Pluto: et_control.py'deki Pluto TX (ET/karıştırma)
ile bu Pluto RX (ED/tarama) AYNI ANDA çalışmalı -- aynı Pluto'da hem TX hem RX
yapmak (daha önce konuştuğumuz self-desense riski) burada söz konusu değil
çünkü fiziksel olarak FARKLI cihazlar. streamer.py ile aynı ARAMA/İZLEME
mimarisini kullanır (bkz. streamer.py docstring'i), sadece:
  - Donanım: RtlSdr yerine PlutoSDR RX
  - Bant listesi: tek sürekli aralık yerine BİRDEN FAZLA ayrı bant (aradaki
    boşluklar taranmaz -- 868-870 bitince direkt 2400-2483'e atlar)
  - Portlar: streamer.py'nin 5555/5556'sıyla ÇAKIŞMASIN diye 5560 (SYS/SPEC)

ÖNEMLİ -- İKİNCİ PLUTO IP ÇAKIŞMASI: Fabrika ayarıyla HER Pluto 192.168.2.1
kullanır. et_control.py'nin TX Pluto'su o adreste kalmalı; bu scriptin
kullandığı RX Pluto'nun IP'sini (Pluto'yu USB diski olarak açıp config.txt
içindeki local_ip satırını değiştirerek) FARKLI bir adrese (örn. 192.168.3.1)
almanız gerekiyor -- yoksa ikisi de aynı adrese denk gelir, ikisine de düzgün
erişilemez. Aşağıdaki EBABIL_PLUTO_ED_IP ile hangi adresi kullanacağını
belirtebilirsin.

Sınıflandırma (AI): streamer.py'deki AYNI predict.py modelini kullanır (tek
kaynak, iki backend de aynı şekilde çıkarım yapar). Tek fark: predict.py'nin
özellik çıkarımı RTL-SDR'ın 250 kSPS'lik verisiyle fine-tune edildi, Pluto ise
o hızda değil CLASSIFY_CAPTURE_RATE'te (kanıtlanmış çalışan, DINLEME_SAMPLE_RATE
ile aynı) yakalıyor -- sınıflandırmadan önce 250 kSPS'e yeniden örnekleniyor
(bkz. _resample_to_classify_rate). AI paketleri streamer.py'nin 5556'sıyla
ÇAKIŞMASIN diye AYRI bir portta (5561) yayınlanır -- ikisi de aynı anda
sınıflandırma yapabilsin diye (bkz. SYS_SPEC_PORT'taki aynı gerekçe).

Kullanım:
  python src/pluto_ed_scanner.py
  EBABIL_PLUTO_ED_DRY_RUN=1 python src/pluto_ed_scanner.py   # ikinci Pluto yokken mantığı test et
"""
import os
import queue
import sys
import threading
import time
import wave

from fractions import Fraction

import numpy as np
import zmq
from scipy.ndimage import gaussian_filter1d
from scipy.signal import resample_poly

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..")
sys.path.insert(0, _THIS_DIR)
if sys.platform == "win32":
    # libiio.dll'i bulmak için -- Linux/Jetson'da libiio sistem paketinden
    # (apt: libiio0, pip: pylibiio/pyadi-iio) geldiği için buna gerek yok
    # (bkz. streamer.py'deki RTL-SDR için aynı desen).
    os.environ["PATH"] = os.path.join(_REPO_ROOT, "tools", "libiio") + os.pathsep + os.environ.get("PATH", "")

try:
    import sounddevice as sd
except Exception:
    sd = None  # Kurulu değilse dinleme sesi sadece .wav'a yazılır, canlı çalınmaz.

import sdr_common
import demod
from predict import load_model_and_scalers, classify_iq_gated
from konum_istemcisi import KonumIstemcisi, UavKonumDinleyici, konum_guncelle_ve_gonder
from tespit_kaydedici import TespitKaydedici
from sayisal_cozucu import SayisalCozucu

DRY_RUN = os.environ.get("EBABIL_PLUTO_ED_DRY_RUN", "0") == "1"
PLUTO_ED_IP = os.environ.get("EBABIL_PLUTO_ED_IP", "ip:192.168.3.1")

# Bu script sınıflandırma (AI) yapmadığı için DİNLE hangi demodülatörü
# kullanacağını otomatik bilemiyor -- streamer.py'de bu son AI sonucundan
# geliyordu. Burada sabit/ortam değişkeniyle seçiliyor (WBFM en yaygın analog
# ses test senaryosu, GNU Radio'nun standart "Audio Source -> WBFM Transmit"
# şablonuyla eşleşir). Farklı bir modülasyon deniyorsan "AM" olarak ayarla.
DINLEME_VARSAYILAN_MOD = os.environ.get("EBABIL_PLUTO_DINLEME_MOD", "WBFM")

# --- Taranacak bantlar -- KTR'nin güncellenmiş anten/bant tablosuyla birebir ---
# 430-440 sahada test amaçlı eklendi (streamer.py/RTL-SDR'ın taradığı bantla
# aynı) -- Pluto donanımsal olarak zaten 70MHz-6GHz'i destekliyor, RTL-SDR'a
# dönmeye gerek yok. step_mhz=0.5 (varsayılan 2.0 MHz'in aksine) -- 433.0 MHz
# gibi bir sinyal, varsayılan 2MHz'lik adımlarla tam iki adımın (432/434)
# SINIRINA denk gelip her iki pencerede de kenar zayıflamasına uğrayıp
# CFAR'ı kaçırabiliyordu (sahada doğrulandı: dogrudan 433.0'a kilitlenince
# SNR ~70dB, ama tarama hiç yakalamıyordu). 0.5 MHz adım, ardışık 2MHz'lik
# pencerelerin %75 örtüşmesini sağlıyor -- hiçbir frekans artık iki pencere
# arasında "kenarda" kalmıyor.
BANDS = [
    # 143-145/868-870 -> 144-148/863-870 (resmi bant tablosuyla birebir --
    # önceki aralıklar 145-148 MHz ve 863-868 MHz'i hiç taramıyordu, kapsam
    # boşluğu). 5725-5875 (ISM 5.8G) eklendi -- Pluto (Rev.C, firmware
    # unlock ile 6GHz'e kadar) fiziksel olarak buraya çıkabiliyor,
    # ebabil_sdr/main.cpp'deki pluto_bantlar tablosunda zaten taranıyor,
    # burada eksikti.
    {"name": "144-148", "start_mhz": 144.0, "stop_mhz": 148.0, "step_mhz": 0.5},
    {"name": "430-440", "start_mhz": 430.0, "stop_mhz": 440.0, "step_mhz": 0.5},
    {"name": "863-870", "start_mhz": 863.0, "stop_mhz": 870.0, "step_mhz": 1.0},
    {"name": "2400-2483", "start_mhz": 2400.0, "stop_mhz": 2483.0, "step_mhz": 1.0},
    {"name": "5725-5875", "start_mhz": 5725.0, "stop_mhz": 5875.0, "step_mhz": 2.0},
]

SEARCH_SAMPLE_RATE = 2_000_000  # arama modu -- Pluto'nun genis RX bandini kullanip az adimda tara
SEARCH_STEP_MHZ = SEARCH_SAMPLE_RATE / 1e6  # bantlarda step_mhz belirtilmemişse varsayılan

DWELL_SAMPLE_RATE = 4_000_000  # izleme modu -- daha genis, stabil waterfall
DWELL_DURATION_S = 4.0
DWELL_SNAP_MHZ = 0.1

THROWAWAY_SAMPLES = 1024
SYS_SPEC_PORT = 5560  # streamer.py'nin 5555/5556'siyla CAKISMASIN diye ayri
CMD_PORT = 5557  # komut kanali streamer.py ile PAYLASILIYOR (ayni PUB/SUB fan-out)
AI_PORT = int(os.environ.get("EBABIL_PLUTO_AI_PORT", "5561"))  # streamer.py'nin 5556'siyla CAKISMASIN diye ayri

# --- Sınıflandırma (AI) -- predict.py ile ORTAK model, streamer.py ile aynı mantık ---
# predict.py'nin özellik çıkarımı (Inst_Freq vb.) RTL-SDR'ın 250 kSPS'lik
# verisiyle fine-tune edildi -- Pluto/AD9361'de bu hızın güvenilir çalışıp
# çalışmadığı test edilmedi. Bunun yerine Pluto için KANITLANMIŞ çalışan bir
# hızda (DINLEME_SAMPLE_RATE ile aynı varsayılan) yakalayıp, sınıflandırmadan
# hemen önce 250 kSPS'e yeniden örnekliyoruz (bkz. _resample_to_classify_rate)
# -- resample_poly zaten anti-alias filtreli olduğu için fiziksel frekans
# içeriği korunur, sadece model neyle eğitildiyse o hıza getirilir.
CLASSIFY_CAPTURE_RATE = int(os.environ.get("EBABIL_PLUTO_CLASSIFY_HZ", "1000000"))
CLASSIFY_TARGET_RATE = 250_000
CLASSIFY_WINDOW = 128  # modelin beklediği pencere uzunluğu (bkz. streamer.py'deki aynı sabit)
CLASSICAL_CHECK_WINDOW = 5000  # klasik periyodiklik çapraz kontrolü için (bkz. predict.classify_iq_gated)
# CLASSICAL_CHECK_WINDOW kadar örneği 250 kSPS'te elde etmek için
# CLASSIFY_CAPTURE_RATE'te kaç ham örnek yakalanması gerektiği.
_CLASSIFY_RAW_SAMPLES = int(CLASSICAL_CHECK_WINDOW * CLASSIFY_CAPTURE_RATE / CLASSIFY_TARGET_RATE)


def build_multi_band_scan_freqs():
    freqs = []
    for band in BANDS:
        step = band.get("step_mhz", SEARCH_STEP_MHZ)
        freqs.extend(sdr_common.build_scan_freqs(band["start_mhz"], band["stop_mhz"], step))
    return freqs


class PlutoRX:
    def __init__(self):
        self.pluto = None
        # Son ayarlanan (rx_lo, sample_rate, buffer_size) -- aynı değerlerle
        # tekrar tekrar retune etmemek için (bkz. _tune yorumu).
        self._last_tune = None

    def _tune(self, center_mhz, sample_rate, buffer_size):
        """rx_lo/sample_rate/rx_rf_bandwidth/rx_buffer_size'ı SADECE
        gerçekten değiştiyse yazar. Bu değerleri her çağrıda (değişmese bile)
        yeniden yazmak AD9363'ü her seferinde donanımsal olarak yeniden
        ayarlıyor (senkronizör/filtre -- onlarca-yüzlerce ms sürebilir).
        DİNLE gibi aynı frekansta arka arkaya çok sayıda kısa yakalama
        yapılan yerlerde bu retune maliyeti, üretimin tüketimden (ses
        oynatma hızından) yavaş kalmasına yol açıyordu -- kuyruk/thread
        bunu çözemez, kaynağında önlemek gerekiyordu."""
        tune = (int(center_mhz * 1e6), int(sample_rate), buffer_size)
        if tune == self._last_tune:
            return
        self.pluto.rx_lo = tune[0]
        self.pluto.sample_rate = tune[1]
        self.pluto.rx_rf_bandwidth = tune[1]
        self.pluto.rx_buffer_size = tune[2]
        # rx_destroy_buffer() SADECE burada, gerçekten değişimde -- her
        # capture() çağrısında (değişmese bile) çağırmak Pluto'nun USB/IIOD
        # arayüzünde her seferinde tam bir arabellek söküm+kurulum maliyeti
        # yaratıyordu (~100-170ms/parça, ölçüldü). DİNLE gibi aynı frekansta
        # arka arkaya çok sayıda kısa yakalama yapılan yerlerde bu, üretimin
        # sürekli tüketimden geride kalmasına (kesik/pıt sesi) yol açıyordu.
        self.pluto.rx_destroy_buffer()
        self._last_tune = tune

    def connect(self):
        if DRY_RUN:
            print(f"[*] EBABIL_PLUTO_ED_DRY_RUN=1 -- {PLUTO_ED_IP} donanımına bağlanılmıyor, sahte veriyle test.")
            return
        import adi
        print(f"[*] İkinci PlutoSDR'a (RX) bağlanılıyor ({PLUTO_ED_IP})...")
        self.pluto = adi.Pluto(PLUTO_ED_IP)
        self.pluto.rx_enabled_channels = [0]
        # DENENDİ VE GERİ ALINDI: sabit 70dB manuel kazanç, alıcının kendi
        # gürültü tabanını da orantılı yükseltip sinyali neredeyse her yerde
        # gürültüye gömüyordu (canlı CFAR verisiyle doğrulandı -- 430-440 MHz
        # taramasında 8 pencereden 7'sinde yerel gürültü tabanı sinyal
        # seviyesine o kadar yakındı ki hiçbir yerde eşiği geçemiyordu,
        # sadece tesadüfen ÇOK güçlü olan tek bir pencere sıyrılabildi).
        # AGC ("slow_attack") bu dengeyi kendisi kuruyor -- RTL-SDR'daki
        # gain="auto" eşdeğeri, aralıklı tespit sorununun asıl kaynağı
        # muhtemelen kazanç değil, antenin kendisiydi.
        self.pluto.gain_control_mode_chan0 = "slow_attack"
        print("[+] Pluto RX hazır (AGC: slow_attack).")

    def capture(self, center_mhz, sample_rate):
        if DRY_RUN:
            # Donanım yok -- gürültü tabanı + ~%8 ihtimalle sahte bir tespit
            # üret, ana döngünün/CFAR'ın gerçek koşullarda test edilebilmesi
            # için. Tek bir saf ton DEĞİL (o kadar dar ki 64 bin'e biriktirmede
            # kayboluyor) -- bant-sınırlı bir gürültü patlaması (yaklaşık 80 ham
            # FFT bin genişliğinde, et_control.py'deki generate_barrage_noise
            # ile aynı teknik) kullanılıyor ki gerçek bir sinyal gibi 64-bin
            # ortalamasından sonra da görünür kalsın.
            n = sdr_common.FFT_SIZE
            samples = (np.random.randn(n) + 1j * np.random.randn(n)).astype(np.complex64) * 0.01
            if np.random.rand() < 0.08:
                i = gaussian_filter1d(np.random.randn(n), sigma=n / (2 * np.pi * 40), mode="wrap")
                q = gaussian_filter1d(np.random.randn(n), sigma=n / (2 * np.pi * 40), mode="wrap")
                burst = (i + 1j * q)
                burst = burst / (np.max(np.abs(burst)) + 1e-9) * 0.4
                samples = samples + burst.astype(np.complex64)
            return sdr_common.compute_power_spectrum(samples, center_mhz, sample_rate)

        self._tune(center_mhz, sample_rate, sdr_common.FFT_SIZE + THROWAWAY_SAMPLES)
        samples = self.pluto.rx()
        if isinstance(samples, (list, tuple)):
            samples = samples[0]
        samples = np.asarray(samples)[THROWAWAY_SAMPLES:]
        return sdr_common.compute_power_spectrum(samples, center_mhz, sample_rate)

    def capture_raw(self, center_mhz, sample_rate, n_samples):
        """Dinleme (DİNLE) için ham I/Q örnekleri -- capture()'ın aksine
        FFT'ye indirgemeden döner, demod.py doğrudan bunu işler."""
        if DRY_RUN:
            n = max(n_samples, sdr_common.FFT_SIZE)
            return (np.random.randn(n) + 1j * np.random.randn(n)).astype(np.complex64) * 0.05

        self._tune(center_mhz, sample_rate, n_samples + THROWAWAY_SAMPLES)
        samples = self.pluto.rx()
        if isinstance(samples, (list, tuple)):
            samples = samples[0]
        return np.asarray(samples)[THROWAWAY_SAMPLES:]


# --- Sinyal İzleme/Dinleme (KTR 4.3) -- streamer.py'deki DinlemeOturumu ile
# aynı, sınıflandırma (AI) burada olmadığı için demod.demod_for_modulation
# her zaman modulasyon_turu=None ile çağrılır (AM zarf algılamaya düşer --
# konuşma/enerji dinlenebilir, ama otomatik WBFM seçimi yok).
DINLEME_BLOK_SURESI_S = 0.25
DINLEME_KAYIT_DIZINI = os.path.join(_REPO_ROOT, "data", "dinleme_kayitlari")
IQ_OVERLAP_SAMPLES = 8192  # ~4ms @ DINLEME_SAMPLE_RATE -- parça sınırı sürekliliği için (bkz. DinlemeOturumu.demodle)
# DİNLE'de SEARCH_SAMPLE_RATE (2 MSPS, tarama için geniş bant) kullanmaya
# gerek yok -- ses/WBFM için gereğinden çok veri demek, her 0.25sn'lik parça
# için donanım+işleme süresini gerçek zamanlıdan yavaş bırakıp kesintiye
# ("pıt" sesleri) yol açıyordu. 1 MSPS bile WBFM kanalı (~200kHz) için bol
# bol yeterli, işlem yükünü yarıya indirip gerçek zamanlı kalmak için pay açar.
DINLEME_SAMPLE_RATE = int(os.environ.get("EBABIL_PLUTO_DINLEME_HZ", "1000000"))


class DinlemeOturumu:
    """Ses çalma artık sd.OutputStream'in CALLBACK modunda -- PortAudio kendi
    ayrı gerçek-zamanlı thread'inde çalışıp sabit hızda veri istiyor. RF
    yakalama (handle_dinleme_capture, ana döngüde) düzensiz aralıklarla
    tamamlanabiliyor -- isle() artık bloklayan bir write() YAPMIYOR, üretilen
    parçayı sadece bir kuyruğa atıyor (hızlı, hiç beklemiyor). Callback bu
    kuyruktan çeker; kuyruk yetişemezse (RF yakalama gecikirse) kesik/bozuk
    ses yerine kısa bir sessizlikle devam eder -- çok daha pürüzsüz."""

    def __init__(self):
        self.hedef_id = None
        self.wav = None
        self.stream = None
        self._audio_queue = None
        self._leftover = np.zeros(0, dtype=np.float32)
        # Her DİNLEME_BLOK_SURESI_S'lik parça bağımsız demodüle edilince
        # (faz farkı + yeniden-örnekleme filtresi her seferinde sıfırdan
        # başlayınca) her parça sınırında küçük bir "tık" oluşuyordu --
        # bir önceki parçanın son birkaç bin ham örneğini burada tutup bir
        # sonraki parçanın başına ekliyoruz (bkz. demodle()), süreklilik
        # sağlanıyor.
        self._iq_overlap = np.zeros(0, dtype=np.complex64)
        # Sayısal (dijital) telsiz decode -- madde 5.1.3'ün opsiyonel kısmı
        # (bkz. sayisal_cozucu.py, streamer.py'deki AYNI entegrasyon).
        self.sayisal_cozucu = SayisalCozucu(giris_sample_rate=demod.AUDIO_SAMPLE_RATE)

    def _audio_callback(self, outdata, frames, time_info, status):
        buf = self._leftover
        while len(buf) < frames:
            try:
                chunk = self._audio_queue.get_nowait()
            except queue.Empty:
                break
            buf = np.concatenate([buf, chunk])
        if len(buf) >= frames:
            outdata[:, 0] = buf[:frames]
            self._leftover = buf[frames:]
        else:
            outdata[:len(buf), 0] = buf
            outdata[len(buf):, 0] = 0.0  # RF yakalama yetişemedi -- kesinti yerine sessizlik
            self._leftover = np.zeros(0, dtype=np.float32)

    def baslat(self, hedef_id, freq_mhz):
        self.durdur()
        os.makedirs(DINLEME_KAYIT_DIZINI, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(DINLEME_KAYIT_DIZINI, f"{hedef_id}_{freq_mhz:.3f}MHz_{timestamp}.wav")
        self.wav = wave.open(path, "wb")
        self.wav.setnchannels(1)
        self.wav.setsampwidth(2)
        self.wav.setframerate(demod.AUDIO_SAMPLE_RATE)
        self.hedef_id = hedef_id
        self._audio_queue = queue.Queue()
        self._leftover = np.zeros(0, dtype=np.float32)
        self._iq_overlap = np.zeros(0, dtype=np.complex64)
        if sd is not None:
            try:
                self.stream = sd.OutputStream(
                    samplerate=demod.AUDIO_SAMPLE_RATE, channels=1, dtype="float32",
                    callback=self._audio_callback)
                self.stream.start()
            except Exception as e:
                print(f"[!] Canlı ses çıkışı açılamadı (sadece .wav'a yazılacak): {e}")
                self.stream = None
        print(f"[*] Dinleme başladı: {hedef_id} -> {path}")
        return path

    def isle(self, audio):
        if self.wav is not None:
            pcm16 = np.clip(audio * 32767, -32768, 32767).astype(np.int16)
            self.wav.writeframes(pcm16.tobytes())
        if self._audio_queue is not None:
            self._audio_queue.put(audio.astype(np.float32))
        self.sayisal_cozucu.besle(audio.astype(np.float32))

    def demodle(self, raw_samples, fs_in, modulasyon_turu):
        """Yeni yakalanan ham örnekleri, bir önceki parçanın kuyruğuyla
        (self._iq_overlap) birleştirip demodüle eder -- FM faz farkı ve
        yeniden-örnekleme filtresi böylece parça sınırında sıfırdan
        başlamıyor. Örtüşmeye karşılık gelen ses kısmı (zaten bir önceki
        parçada üretilip çalınmıştı) çıktıdan kırpılıp atılır."""
        overlap_n = len(self._iq_overlap)
        combined = np.concatenate([self._iq_overlap, raw_samples]) if overlap_n else raw_samples
        audio = demod.demod_for_modulation(combined, fs_in, modulasyon_turu)

        if overlap_n:
            drop_n = int(round(overlap_n / fs_in * demod.AUDIO_SAMPLE_RATE))
            audio = audio[drop_n:]

        overlap_len = min(IQ_OVERLAP_SAMPLES, len(raw_samples))
        self._iq_overlap = raw_samples[-overlap_len:].copy()
        return audio

    def durdur(self):
        if self.wav is not None:
            self.wav.close()
            self.wav = None
        if self.stream is not None:
            try:
                self.stream.stop()
                self.stream.close()
            except Exception:
                pass
            self.stream = None
        self._audio_queue = None
        self.hedef_id = None


def _resample_to_classify_rate(iq, fs_in):
    """iq (complex) örneklerini fs_in'den CLASSIFY_TARGET_RATE'e yeniden
    örnekler -- bkz. CLASSIFY_CAPTURE_RATE yorumundaki gerekçe."""
    ratio = Fraction(CLASSIFY_TARGET_RATE, int(fs_in)).limit_denominator(1000)
    return resample_poly(iq, ratio.numerator, ratio.denominator)


def handle_classify_request(rx, pub_ai, tracker, model, feature_mean, feature_std, selected_id, son_siniflandirma):
    """streamer.py'deki handle_classify_request ile aynı mantık, sadece
    yakalama donanımı ve hızı farklı (bkz. CLASSIFY_CAPTURE_RATE). Sonuç
    AI_PORT'tan streamer.py ile BİREBİR AYNI formatta yayınlanır:
    AI,id,analogSayisal,modulasyonTuru -- GUI id'ye göre eşleştirdiği için
    (bkz. mainwindow.cpp hedefYapayZekaGuncelle) PHEDEF-N kartları da bu
    paketle güncellenir, arayüzde ayrıca bir değişiklik gerekmez."""
    tid = sdr_common.pick_target(tracker, selected_id)
    if tid is None:
        print("[!] Henüz tespit edilmiş hedef yok, sınıflandırma isteği atlandı.")
        return

    freq_mhz = tracker.known[tid]["freq_mhz"]
    raw_samples = rx.capture_raw(freq_mhz, CLASSIFY_CAPTURE_RATE, _CLASSIFY_RAW_SAMPLES)
    resampled = _resample_to_classify_rate(raw_samples, CLASSIFY_CAPTURE_RATE)

    # Modelin gördüğü pencere (ilk CLASSIFY_WINDOW örnek) DEĞİŞMİYOR -- eğitimde
    # kullanılanla birebir aynı kalsın diye. Geri kalanı SADECE klasik periyodiklik
    # çapraz kontrolü için (bkz. predict.classify_iq_gated).
    window_full = resampled[:CLASSICAL_CHECK_WINDOW]
    window = window_full[:CLASSIFY_WINDOW]

    I = np.real(window).astype(np.float32)
    Q = np.imag(window).astype(np.float32)
    classical_I = np.real(window_full).astype(np.float32)
    classical_Q = np.imag(window_full).astype(np.float32)
    analog_sayisal, mod, confidence = classify_iq_gated(
        model, feature_mean, feature_std, I, Q, classical_I, classical_Q)

    pub_ai.send_string(f"AI,{tid},{analog_sayisal},{mod}")
    print(f"[>] {tid} ({freq_mhz:.3f} MHz) sınıflandırıldı: {mod} ({analog_sayisal}) - güven %{confidence:.1f}")

    if son_siniflandirma is not None:
        son_siniflandirma[tid] = (analog_sayisal, mod)


def handle_dinleme_capture(rx, dinleme, hedef_id, freq_mhz, pub):
    t0 = time.time()
    n_samples = int(DINLEME_BLOK_SURESI_S * DINLEME_SAMPLE_RATE)
    samples = rx.capture_raw(freq_mhz, DINLEME_SAMPLE_RATE, n_samples)
    audio = dinleme.demodle(samples, DINLEME_SAMPLE_RATE, DINLEME_VARSAYILAN_MOD)
    dinleme.isle(audio)
    for metin in dinleme.sayisal_cozucu.oku():
        pub.send_string(f"SAYISAL,{hedef_id},{metin}")
    elapsed = time.time() - t0
    if elapsed > DINLEME_BLOK_SURESI_S * 2.0:
        # Sadece GERÇEKTEN kötü durumlarda logla (rx_destroy_buffer() düzeltmesi
        # öncesi ~350-420ms/parça sistematikti -- artık öyle olmamalı, bunu
        # görürsek hâlâ bir sorun var demektir).
        print(f"[!] Dinleme parçası gecikti: {elapsed*1000:.0f}ms (bütçe {DINLEME_BLOK_SURESI_S*1000:.0f}ms)")
    return sdr_common.compute_power_spectrum(samples[:sdr_common.FFT_SIZE], freq_mhz, DINLEME_SAMPLE_RATE)


def stdin_command_reader(command_queue):
    for line in sys.stdin:
        line = line.strip()
        if line:
            command_queue.put(line)


def main():
    context = zmq.Context()

    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://127.0.0.1:{SYS_SPEC_PORT}")

    pub_ai = context.socket(zmq.PUB)
    pub_ai.bind(f"tcp://127.0.0.1:{AI_PORT}")

    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect(f"tcp://127.0.0.1:{CMD_PORT}")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    print("[*] Model yükleniyor...")
    model, feature_mean, feature_std = load_model_and_scalers()
    print("[+] Model hazır.")

    rx = PlutoRX()
    rx.connect()

    tracker = sdr_common.TargetTracker(id_prefix="PHEDEF")  # streamer.py'nin HEDEF-N'iyle karışmasın

    # Yön bulma + konum kestirimi -- streamer.py'deki AYNI mekanizma (bkz. o
    # dosyadaki yorum ve konum_istemcisi.py). konum_servisi PHEDEF-N ve
    # HEDEF-N'i ayrı hedefler olarak (farklı string id) takip ettiği için
    # iki tarayıcının aynı anda çalışması bir sorun teşkil etmiyor.
    uav_konum = UavKonumDinleyici()
    konum_istemcisi = KonumIstemcisi()
    tespit_kaydedici = TespitKaydedici(dosya_onek="pluto")
    varsayilan_scan_freqs = build_multi_band_scan_freqs()  # "bant varsayilan" ile buna geri dönülür
    scan_freqs = varsayilan_scan_freqs
    scan_idx = 0

    dwelling = False
    dwell_center_mhz = None
    dwell_target_id = None  # otomatik modda pick_target()'ın histerezisi için "şu an kilitli hedef"
    dwell_started_at = 0.0
    dwell_locked = False
    selected_target_id = None

    dinleme = DinlemeOturumu()
    dinleme_hedef_id = None

    # GUI'deki "TARAMAYI DURDUR" düğmesiyle -- True iken ARAMA/İZLEME tamamen
    # durur (Pluto'ya hiç dokunulmaz, SPEC/SYS yayınlanmaz), DİNLE etkilenmez.
    tarama_duraklatildi = False
    son_duraklatma_heartbeat = 0.0  # bkz. aşağıdaki "DURAKLATILDI" dalı

    # tid -> (analogSayisal, modulasyonTuru) -- handle_classify_request
    # günceller. DİNLE bunu şu an kullanmıyor (bkz. DINLEME_VARSAYILAN_MOD),
    # sadece AI kartı alanlarını doldurmak için tutuluyor.
    son_siniflandirma = {}

    command_queue = queue.Queue()
    threading.Thread(target=stdin_command_reader, args=(command_queue,), daemon=True).start()

    band_names = ", ".join(f"{b['name']} MHz" for b in BANDS)
    print(f"[*] Taranacak bantlar: {band_names} ({len(scan_freqs)} adım toplam)")
    print(f"[*] Port {SYS_SPEC_PORT}: SYS/SPEC | Port {AI_PORT}: AI | Port {CMD_PORT}: komut dinleniyor (paylaşımlı)")
    print("[*] Belirli bir hedefe kilitlenmek için: hedef <id>  |  hedef oto\n")

    try:
        while True:
            try:
                try:
                    msg = sub_cmd.recv_string(flags=zmq.NOBLOCK)
                    if msg.startswith("PLUTO_ED_HEDEF_SEC|"):
                        _, target_id = msg.split("|")
                        command_queue.put(f"hedef {target_id}")
                    elif msg == "SDR_VERISI_ISTEK":
                        handle_classify_request(rx, pub_ai, tracker, model, feature_mean, feature_std,
                                                 selected_target_id, son_siniflandirma)
                    elif msg.startswith("DINLE_BASLAT|"):
                        try:
                            _, hedef_id = msg.split("|")
                        except ValueError:
                            print(f"[!] Geçersiz DINLE_BASLAT komutu: {msg}")
                        else:
                            if hedef_id not in tracker.known:
                                print(f"[!] {hedef_id} bilinmiyor, dinleme başlatılamadı.")
                            else:
                                freq_mhz = tracker.known[hedef_id]["freq_mhz"]
                                dinleme.baslat(hedef_id, freq_mhz)
                                dinleme_hedef_id = hedef_id
                                pub.send_string(f"DURUM,DINLEME_AKTIF,{freq_mhz:.3f}")
                    elif msg == "DINLE_DURDUR":
                        dinleme.durdur()
                        dinleme_hedef_id = None
                        pub.send_string("DURUM,TARIYOR")
                    elif msg.startswith("BANT_AYARLA|"):
                        # BANT_AYARLA|<başlangıç_mhz>|<bitiş_mhz> -- arayüzden
                        # gelebilecek eşdeğeri, bkz. aşağıdaki "bant" komutu.
                        try:
                            _, start_s, stop_s = msg.split("|")
                            command_queue.put(f"bant {start_s} {stop_s}")
                        except ValueError:
                            print(f"[!] Geçersiz BANT_AYARLA komutu: {msg}")
                    elif msg == "BANT_VARSAYILAN":
                        command_queue.put("bant varsayilan")
                    elif msg == "TARAMA_DURDUR":
                        tarama_duraklatildi = True
                        pub.send_string("DURUM,DURAKLATILDI")
                        print("[*] Tarama duraklatıldı.")
                    elif msg == "TARAMA_DEVAM":
                        tarama_duraklatildi = False
                        print("[*] Tarama devam ediyor.")
                except zmq.Again:
                    pass

                try:
                    line = command_queue.get_nowait()
                    lower = line.lower()
                    if lower.startswith("bant"):
                        # Operatör arayüzden ya da elle yeni bir tarama bandı
                        # verdi -- kod değiştirip süreci yeniden başlatmaya
                        # gerek kalmadan BANDS'i geçici olarak TEK bu bandla
                        # değiştirir. step_mhz=0.5 sabit -- 433.0 MHz'de
                        # öğrendik: varsayılan 2MHz adımda sinyal iki adımın
                        # sınırına denk gelip CFAR'ı kaçırabiliyordu.
                        parts = line.split()
                        if len(parts) == 2 and parts[1].lower() in ("varsayilan", "varsayılan", "oto", "otomatik"):
                            scan_freqs = varsayilan_scan_freqs
                            scan_idx = 0
                            dwelling = False
                            dwell_locked = False
                            band_names = ", ".join(f"{b['name']} MHz" for b in BANDS)
                            print(f"[*] Tarama bandı varsayılana döndürüldü: {band_names} "
                                  f"({len(scan_freqs)} adım)")
                        elif len(parts) != 3:
                            print("[!] Kullanım: bant <başlangıç_mhz> <bitiş_mhz>  (örn: bant 143 145)  |  bant varsayilan")
                        else:
                            try:
                                new_start, new_stop = float(parts[1]), float(parts[2])
                                if new_start >= new_stop:
                                    print(f"[!] Başlangıç bitişten küçük olmalı: {new_start} >= {new_stop}")
                                else:
                                    scan_freqs = sdr_common.build_scan_freqs(new_start, new_stop, 0.5)
                                    scan_idx = 0
                                    dwelling = False
                                    dwell_locked = False
                                    print(f"[*] Tarama bandı güncellendi: {new_start}-{new_stop} MHz "
                                          f"({len(scan_freqs)} adım)")
                            except ValueError:
                                print(f"[!] Geçersiz sayı: {line!r}")

                    elif lower.startswith("hedef"):
                        parts = line.split()
                        if len(parts) != 2:
                            print("[!] Kullanım: hedef <id>  |  hedef oto")
                        elif parts[1].upper() in ("OTO", "OTOMATIK"):
                            selected_target_id = None
                            dwell_target_id = None
                            dwelling = False
                            dwell_locked = False
                            scan_idx = 0
                            print("[*] Hedef seçimi temizlendi.")
                        else:
                            target_id = parts[1].upper()
                            if target_id not in tracker.known:
                                print(f"[!] {target_id} bilinmiyor (bilinenler: {', '.join(tracker.known) or '(yok)'})")
                            else:
                                selected_target_id = target_id
                                dwell_target_id = target_id
                                raw_freq = tracker.known[target_id]["freq_mhz"]
                                dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                                dwelling = True
                                # Kasıtlı olarak dwell_locked=False -- operatörün karta
                                # tıklaması (sadece görüntülemek/DİNLE için) taramayı
                                # SONSUZA KADAR durdurmamalı. DWELL_DURATION_S sonra
                                # otomatik tam-bant turuna döner, tur bitince
                                # selected_target_id'ye tekrar döner -- yani bu hedef
                                # ÖNCELİKLİ ama taramayı ENGELLEMİYOR. Tam kilit sadece
                                # hakemin gerçek frekansı açıkladığı "frekans" komutunda.
                                dwell_locked = False
                                dwell_started_at = time.time()
                                print(f"[*] {target_id} seçildi, {dwell_center_mhz:.3f} MHz'e odaklanıldı "
                                      f"(tam bant taraması devam edecek).")
                except queue.Empty:
                    pass

                if dwelling and not dwell_locked and (time.time() - dwell_started_at > DWELL_DURATION_S):
                    dwelling = False

                if dinleme_hedef_id is not None:
                    # --- DİNLE: gerçek AM demodülasyonu -- taramaya ara verir,
                    # sadece dinlenen hedefin frekansına kilitli kalır (bkz.
                    # streamer.py'deki aynı isimli mantık). CFAR/SYS burada
                    # ÇALIŞMAZ, sadece SPEC akışı sürer (waterfall donmasın).
                    if dinleme_hedef_id not in tracker.known:
                        print(f"[!] {dinleme_hedef_id} artık bilinmiyor, dinleme durduruluyor.")
                        dinleme.durdur()
                        dinleme_hedef_id = None
                        pub.send_string("DURUM,TARIYOR")
                    else:
                        dinleme_freq_mhz = tracker.known[dinleme_hedef_id]["freq_mhz"]
                        binned_db, bin_freqs_mhz, fs_mhz = handle_dinleme_capture(
                            rx, dinleme, dinleme_hedef_id, dinleme_freq_mhz, pub)
                        spec_fields = ["SPEC", f"{dinleme_freq_mhz:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                        pub.send_string(",".join(spec_fields))

                elif tarama_duraklatildi:
                    # --- DURAKLATILDI: operatör "TARAMAYI DURDUR" dedi --
                    # Pluto'ya hiç dokunulmuyor, SPEC/SYS yayınlanmıyor. GUI'nin
                    # bağlantı-canlılık kontrolü duraklatmayı kopma sanmasın
                    # diye hafif bir "hâlâ buradayım" mesajı gönderiyoruz.
                    if time.time() - son_duraklatma_heartbeat > 1.0:
                        pub.send_string("DURUM,DURAKLATILDI")
                        son_duraklatma_heartbeat = time.time()
                    time.sleep(0.2)

                else:
                    if dwelling:
                        binned_db, bin_freqs_mhz, fs_mhz = rx.capture(dwell_center_mhz, DWELL_SAMPLE_RATE)
                    else:
                        center_mhz = scan_freqs[scan_idx]
                        scan_idx += 1
                        binned_db, bin_freqs_mhz, fs_mhz = rx.capture(center_mhz, SEARCH_SAMPLE_RATE)

                    spec_center = dwell_center_mhz if dwelling else center_mhz
                    spec_fields = ["SPEC", f"{spec_center:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                    pub.send_string(",".join(spec_fields))

                    peak = sdr_common.detect_peak(binned_db, bin_freqs_mhz)
                    if peak is not None:
                        freq_mhz, power_db, bandwidth_khz, noise_floor_db = peak
                        snr_db = power_db - noise_floor_db
                        sapma_mhz = freq_mhz - spec_center
                        tid = tracker.update(freq_mhz, power_db, bandwidth_khz, snr_db, sapma_mhz)
                        info = tracker.known[tid]
                        sureklilik = tracker.sureklilik_durumu(tid)
                        sys_fields = [
                            "SYS", tid, "1", "nan", "nan", "0",
                            f"{info['freq_mhz']:.3f}", f"{info['power_db']:.2f}", f"{info['bandwidth_khz']:.1f}",
                            f"{sapma_mhz:.4f}", f"{power_db - snr_db:.2f}", f"{snr_db:.2f}", sureklilik,
                        ]
                        pub.send_string(",".join(sys_fields))
                        konum_guncelle_ve_gonder(pub, konum_istemcisi, uav_konum, tid, freq_mhz, power_db)
                        tespit_kaydedici.kaydet(tracker, tid)

                    if not dwelling and scan_idx >= len(scan_freqs):
                        scan_idx = 0
                        # current_id=dwell_target_id ile histerezis -- streamer.py'deki
                        # aynı mantık (bkz. sdr_common.pick_target / CLAUDE.md).
                        lock_target = sdr_common.pick_target(tracker, selected_target_id, dwell_target_id)
                        if lock_target is not None:
                            dwell_target_id = lock_target
                            raw_freq = tracker.known[lock_target]["freq_mhz"]
                            dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                            dwelling = True
                            dwell_started_at = time.time()

                if DRY_RUN:
                    time.sleep(0.05)  # sahte modda CPU'yu bogmasin

            except Exception as e:
                import traceback; traceback.print_exc()
                print(f"[!] Tarama sırasında hata (devam ediliyor): {e}")
                time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        dinleme.durdur()
        if not DRY_RUN and rx.pluto is not None:
            rx.pluto.rx_destroy_buffer()
        print("\n[*] pluto_ed_scanner kapatıldı.")


if __name__ == "__main__":
    main()
