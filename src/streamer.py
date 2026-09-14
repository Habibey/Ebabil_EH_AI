"""
Gerçek RTL-SDR ile bant taraması + hedef tespiti + (istendiğinde) AI
sınıflandırması yapan tek süreç.

Neden tek süreç: RTL-SDR tek bir USB cihazı, aynı anda sadece TEK bir process
tarafından açılabilir. Bu yüzden artık streamer.py hem SYS/SPEC (tarama +
tespit, port 5555) hem de AI (sınıflandırma, port 5556) paketlerini kendisi
üretiyor -- predict.py'nin bağımsız süreci (start_ai_node) hâlâ duruyor ama
o SADECE donanımsız/sahte-IQ demo modu için (bkz. predict.py). Gerçek
donanımla artık SADECE bu dosyayı çalıştırman yeterli, predict.py'yi ayrıca
başlatmana gerek yok.

Model/özellik çıkarma kodu predict.py'den import ediliyor (tek kaynak, aynı
kod hem gerçek hem sahte modda kullanılıyor).

--- ARAMA / İZLEME (dwell) MODLARI ---
İlk tasarım tek bir ince (250kHz) taramayla tüm bandı sürekli dolaşıyordu --
ama arayüzün waterfall'ı, ardışık SPEC paketlerinin merkez frekansı
%15'ten fazla kayarsa görünümü SIFIRLAYIP yeniden merkezliyor (bkz.
mainwindow.cpp odaklanGerekirse). 250kHz'lik adımlarla sürekli sıçramak bu
sıfırlamayı HER paketten tetikliyor, yani waterfall zaman içinde biriken
anlamlı bir görüntü değil, sadece o anki tek adımın anlık görüntüsü oluyordu.

Çözüm -- iki mod:
  ARAMA: tüm bandı (SCAN_START/STOP) ince adımlarla tarar, SYS/SPEC basar.
  İZLEME (dwell): bir tam arama turu bir hedef bulduysa, o hedefin
    frekansına kilitlenip DAHA GENİŞ örnekleme hızıyla (DWELL_SAMPLE_RATE)
    AYNI merkez/genişliği tekrar tekrar gönderir -- görünüm sabit kalır,
    gerçek bir spektrum analizördeki gibi zaman içinde birikip akar.
    DWELL_DURATION_S sonra kısa bir arama turuna dönüp yeni hedef arar.
"""
import os
import queue
import sys
import threading
import time
import wave

import zmq
import numpy as np

try:
    import sounddevice as sd
except Exception:
    sd = None  # Kurulu değilse dinleme sesi sadece .wav'a yazılır, canlı çalınmaz.

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..")
sys.path.insert(0, _THIS_DIR)
if sys.platform == "win32":
    # librtlsdr.dll'i bulmak için -- Linux/Jetson'da paket zaten sistem
    # kütüphanesini (librtlsdr-dev) kullanıyor, buna gerek yok.
    os.add_dll_directory(os.path.join(_REPO_ROOT, "tools", "rtlsdr"))

from rtlsdr import RtlSdr

from predict import load_model_and_scalers, classify_iq_gated, CLASSES
import sdr_common
import demod
from konum_istemcisi import KonumIstemcisi, UavKonumDinleyici, konum_guncelle_ve_gonder

# --- Tarama ayarları ---
SEARCH_SAMPLE_RATE = 250000  # arama modu -- ince çözünürlük (RTL-SDR'ın düşük geçerli aralığı: 225k-300k)
SEARCH_STEP_MHZ = SEARCH_SAMPLE_RATE / 1e6  # örtüşmesiz, kanal genişliği kadar adım

# --- Taranacak bantlar -- KTR'nin bant tablosuyla birebir (bkz. pluto_ed_scanner.py'deki
# AYNI liste) -- yarışmada operatör hedefin hangi bantta olduğunu ÖNCEDEN bilmiyor, bu
# yüzden varsayılan mod TEK bir aralık değil, ilgili TÜM bantları sırayla tarar. Hakem
# BANT açıklarsa "bant <başlangıç> <bitiş>" komutu bunu geçici olarak TEK bir aralıkla
# değiştirir (bkz. aşağıdaki "bant" komutu), "bant varsayilan" bu listeye geri döner.
# 2400-2483 MHz burada YOK -- RTL-SDR (R828D tuner) donanımsal olarak ~1.7GHz'in
# üzerine çıkamıyor, o bant sadece Pluto/pluto_ed_scanner.py'de taranabiliyor.
BANDS = [
    # Resmi bant tablosuyla (KTR) birebir: 144-148 (VHF amatör), 863-870
    # (ISM 868) -- önceki 143-145/868-870 aralıkları 145-148 MHz ve 863-868
    # MHz'i hiç taramıyordu (kapsam boşluğu, hakem sinyali tam oraya
    # koyarsa tespit edilemezdi). 430-440 zaten 432.82-435.02'yi kapsayacak
    # kadar geniş, öyle bırakıldı.
    #
    # rf_port: RTL-SDR girişindeki kamçı(144/433)/868 SP2T RF anahtarının bu
    # bant için hangi porta alınması gerektiği (bkz. RtlRfAnahtari altta,
    # yonKonum1905/parametreCikarimi/ebabil_sdr/main.cpp'deki C++ eşdeğeriyle
    # AYNI kablolama kararı: 0=kamçı, 1=868 anten).
    {"name": "144-148", "start_mhz": 144.0, "stop_mhz": 148.0, "rf_port": 0},
    {"name": "430-440", "start_mhz": 430.0, "stop_mhz": 440.0, "rf_port": 0},
    {"name": "863-870", "start_mhz": 863.0, "stop_mhz": 870.0, "rf_port": 1},
]

DWELL_SAMPLE_RATE = 1024000  # izleme modu -- geniş anlık bant, kararlı/akan waterfall için
DWELL_SPAN_MHZ = DWELL_SAMPLE_RATE / 1e6
DWELL_DURATION_S = 4.0  # bu süre boyunca hedefe kilitlenip kal, sonra kısa bir arama turu yap
DWELL_SNAP_MHZ = 0.05  # merkezi bu hassasiyete yuvarla -- küçük titreşim görünümü resetlemesin

FFT_SIZE = sdr_common.FFT_SIZE
FREQ_BINS = sdr_common.FREQ_BINS
THROWAWAY_SAMPLES = 1024  # her retune/hız değişimi sonrası kararsız örnekleri at

TARGET_MATCH_TOLERANCE_MHZ = 0.5  # bu aralıktaki tekrar tespitler AYNI hedef kabul edilir
# NOT: Geniş/gürültülü sinyaller (ör. ~200kHz FM yayını, ya da 868 MHz gibi
# bantlarda görülen sıçramalı tepe tahminleri) komşu tarama adımları arasında
# bölününce tepe frekansı tahmini yüzlerce kHz kayabiliyor -- tolerans bunu
# tolere edecek kadar geniş olmalı (0.05 ve 0.15 denendi, aynı yayına art
# arda ikinci/üçüncü/... bir HEDEF-N açıyordu, sahada 868 MHz testinde TEK
# bir yayın 8 ayrı hedef gibi göründü). Çok dar bantlı/birbirine yakın
# GERÇEKTEN FARKLI hedeflerle çalışırken bu değeri düşürmek gerekebilir --
# 0.5 MHz'den yakın iki ayrı verici varsa artık aynı hedef sayılırlar.

CLASSIFY_WINDOW = 128  # modelin beklediği pencere uzunluğu -- SEARCH_SAMPLE_RATE'te toplanmalı (fine-tuning verisiyle tutarlı)
# Klasik periyodiklik çapraz kontrolü (predict.classical_analog_sayisal) 128
# örnekte güvenilir değil -- otokorelasyonun anlamlı olması için çok daha
# geniş bir pencere gerekiyor (11 sınıfla doğrulanan test 5000 örnek kullandı).
CLASSICAL_CHECK_WINDOW = 5000


class RtlRfAnahtari:
    """RTL-SDR RX girişindeki kamçı(144/433)/868 SP2T RF anahtarı -- TEK GPIO
    hattı, ikili değer (bkz. yonKonum1905/parametreCikarimi/ebabil_sdr/main.cpp
    içindeki GpiodRfAnahtari, C++ tarafındaki BİREBİR aynı mantık; bu proje
    artık RTL-SDR taramasını C++ (ebabil_sdr) değil BU dosyayı çalıştırarak
    yapıyor, o yüzden GERÇEK GPIO sürücüsü burada olmalı).

    Gerçek GPIO chip/hat değeri FABRİKE VERİLMEZ -- yanlış bir pin gerçek
    donanımda istenmeyen bir hattı tetikleyebilir (bkz. proje notları, aynı
    karar etSunucu ve ebabil_sdr'de de alındı). EBABIL_RTL_RF_SWITCH_CHIP /
    EBABIL_RTL_RF_SWITCH_HAT açıkça verilmezse anahtar YAPILANDIRILMAZ, anten
    sabit kalır, net bir uyarı basılır -- kod çalışmaya devam eder."""

    def __init__(self):
        self._line = None
        self._son_port = None

        chip_adi = os.environ.get("EBABIL_RTL_RF_SWITCH_CHIP", "")
        hat_metin = os.environ.get("EBABIL_RTL_RF_SWITCH_HAT", "")
        if not chip_adi or not hat_metin:
            print("[RTL RF ANAHTARI] YAPILANDIRILMADI (EBABIL_RTL_RF_SWITCH_CHIP/HAT verilmedi) -- "
                  "anten sabit kalacak. Gerçek GPIO chip/hat numarasını switch modülünün "
                  "kablolamasından doğrulayıp EBABIL_RTL_RF_SWITCH_CHIP (ör. gpiochip0) ve "
                  "EBABIL_RTL_RF_SWITCH_HAT (ör. 17, BCM pin no) ile verin.")
            return

        try:
            import gpiod  # python3-libgpiod (apt) -- C++ tarafıyla aynı libgpiod v1 API
            chip = gpiod.Chip(chip_adi)
            self._line = chip.get_line(int(hat_metin))
            self._line.request(consumer="ebabil_rtl_rf_anahtari", type=gpiod.LINE_REQ_DIR_OUT,
                                default_vals=[0])
            print(f"[RTL RF ANAHTARI] Hazır (chip={chip_adi} hat={hat_metin}).")
        except Exception as e:
            print(f"[RTL RF ANAHTARI] AÇILAMADI (chip={chip_adi} hat={hat_metin}): {e} -- "
                  "GPIO donanımı yok/hazır değil ya da hat geçersiz, anten sabit kalacak.")
            self._line = None

    def porta_gec(self, center_mhz):
        """center_mhz'in BANDS'teki hangi banda düştüğünü bulup o bandın
        rf_port'una geçer -- main.cpp'deki rtlAntenPortuBul ile aynı mantık.
        Hiçbir banda denk gelmezse (olağan akışta olmamalı, tüm retune'lar
        BANDS'ten üretilir) mevcut pozisyon KORUNUR."""
        if self._line is None:
            return
        port = None
        for band in BANDS:
            if band["start_mhz"] <= center_mhz <= band["stop_mhz"]:
                port = band.get("rf_port", 0)
                break
        if port is None or port == self._son_port:
            return
        try:
            self._line.set_value(port)
            self._son_port = port
        except Exception as e:
            print(f"[RTL RF ANAHTARI] port {port}'a geçiş başarısız: {e}")


class RtlAntenliSdr(sdr_common.TunedSdr):
    """TunedSdr'i (bkz. sdr_common.py) RF anten anahtarlamayla genişletir --
    SADECE streamer.py'de kullanılır (RTL-SDR'a özel bir donanım detayı);
    sdr_common pluto_ed_scanner.py ile ORTAK olduğu için bu detayı
    TAŞIMAMALI, o yüzden ayrı bir alt sınıf olarak burada tutuluyor."""

    def __init__(self, device, throwaway_samples, rf_anahtari):
        super().__init__(device, throwaway_samples)
        self._rf_anahtari = rf_anahtari

    def tune(self, center_mhz, sample_rate):
        self._rf_anahtari.porta_gec(center_mhz)
        super().tune(center_mhz, sample_rate)


def capture_power_spectrum(sdr, center_mhz, sample_rate):
    """center_mhz'e kilitlenip FFT_SIZE örnek alır, FREQ_BINS'e indirgenmiş
    güç spektrumunu (dB), her bin'in gerçek frekansını (MHz) ve kullanılan
    örnekleme hızını (MHz) döndürür. sdr, sdr_common.TunedSdr olduğu için
    center_mhz/sample_rate zaten aynıysa retune atlanır -- çağıran taraf
    hız/mod geçişiyle uğraşmaz. FFT/binleme matematiği sdr_common'da --
    pluto_ed_scanner.py ile ortak."""
    sdr.tune(center_mhz, sample_rate)
    samples = sdr.read_samples(FFT_SIZE)
    return sdr_common.compute_power_spectrum(samples, center_mhz, sample_rate)


# detect_peak artık sdr_common'da GERÇEK OS-CFAR ile uygulanıyor (eskiden
# medyan + sabit eşikti -- KTR Tablo 7'de OS-CFAR yazıyordu ama kod basit bir
# eşiklemeydi, gerçek adaptif persentil tabanlı tespite geçildi).
detect_peak = sdr_common.detect_peak
TargetTracker = sdr_common.TargetTracker


def stdin_command_reader(command_queue):
    """Ayrı thread'de stdin'den satır satır okur, kuyruğa koyar -- ana döngü
    bunu NOBLOCK şekilde kontrol eder (ZMQ komut kontrolüyle aynı desen)."""
    for line in sys.stdin:
        line = line.strip()
        if line:
            command_queue.put(line)


def handle_save_command(sdr, tracker, args, selected_id=None):
    """'kaydet ETIKET [süre_sn]' komutunu işler -- şu an seçili/en son
    görülen hedefin frekansına kilitlenip gerçek I/Q kaydeder,
    data/real_captures/<ETIKET>/<zaman damgası>.npz olarak kaydeder.
    Format, build_training_set.py'nin beklediğiyle birebir aynı (bkz.
    tools/dataset/capture_and_label.py) -- ayrıca bir dönüştürme gerekmez.
    Kayıt her zaman SEARCH_SAMPLE_RATE'te yapılır (fine-tuning verisiyle
    tutarlı olsun diye), dwell modunda olsak bile."""
    parts = args.split()
    if not parts:
        print("[!] Kullanım: kaydet <ETİKET> [süre_sn]  (örn: kaydet WBFM 10)")
        return

    label = parts[0].upper()
    if label not in CLASSES:
        print(f"[!] Geçersiz etiket: {label}. Geçerli etiketler: {', '.join(CLASSES)}")
        return

    duration = float(parts[1]) if len(parts) > 1 else 10.0

    tid = pick_target(tracker, selected_id)
    if tid is None:
        print("[!] Henüz tespit edilmiş hedef yok, kayıt alınamadı.")
        return

    freq_mhz = tracker.known[tid]["freq_mhz"]
    print(f"[*] Kayıt başlıyor: {tid} ({freq_mhz:.3f} MHz), etiket={label}, süre={duration:.1f}s...")

    sdr.tune(freq_mhz, SEARCH_SAMPLE_RATE)

    n_samples = int(duration * SEARCH_SAMPLE_RATE)
    chunk = 262144  # tek seferde büyük blok istemek USB zaman aşımına yol açıyor
    parts_list = []
    remaining = n_samples
    while remaining > 0:
        n = min(chunk, remaining)
        parts_list.append(sdr.read_samples(n))
        remaining -= n
    samples = np.concatenate(parts_list).astype(np.complex64)

    out_dir = os.path.join(_REPO_ROOT, "data", "real_captures", label)
    os.makedirs(out_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"{timestamp}.npz")
    np.savez_compressed(
        out_path, samples=samples, label=label, freq_mhz=freq_mhz,
        sample_rate=SEARCH_SAMPLE_RATE, captured_at=timestamp,
    )
    print(f"[+] Kaydedildi: {out_path} ({len(samples)} örnek, {len(samples) // 128} pencere üretilebilir)")


def handle_classify_request(sdr, pub_ai, tracker, model, feature_mean, feature_std, selected_id=None,
                             son_siniflandirma=None):
    """Sınıflandırma için her zaman SEARCH_SAMPLE_RATE'e döner -- model,
    fine-tuning verisi bu hızda toplandığı için buna göre eğitildi. Dwell
    modundaysak bile geçici olarak hıza döner, sonraki dwell/arama adımı
    kendi hızını main() döngüsünde yeniden ayarlar."""
    tid = pick_target(tracker, selected_id)
    if tid is None:
        print("[!] Henüz tespit edilmiş hedef yok, sınıflandırma isteği atlandı.")
        return

    freq_mhz = tracker.known[tid]["freq_mhz"]
    sdr.tune(freq_mhz, SEARCH_SAMPLE_RATE)
    # Modelin gördüğü pencere (ilk CLASSIFY_WINDOW örnek) DEĞİŞMİYOR -- eğitimde
    # kullanılanla birebir aynı kalsın diye. Geri kalanı SADECE klasik periyodiklik
    # çapraz kontrolü için (bkz. predict.classify_iq_gated).
    window_full = sdr.read_samples(CLASSICAL_CHECK_WINDOW)
    window = window_full[:CLASSIFY_WINDOW]

    I = np.real(window).astype(np.float32)
    Q = np.imag(window).astype(np.float32)
    classical_I = np.real(window_full).astype(np.float32)
    classical_Q = np.imag(window_full).astype(np.float32)
    analog_sayisal, mod, confidence = classify_iq_gated(
        model, feature_mean, feature_std, I, Q, classical_I, classical_Q)

    pub_ai.send_string(f"AI,{tid},{analog_sayisal},{mod}")
    print(f"[>] {tid} ({freq_mhz:.3f} MHz) sınıflandırıldı: {mod} ({analog_sayisal}) - güven %{confidence:.1f}")

    # Dinleme modu (bkz. handle_dinleme_capture) hangi demodülatörü
    # kullanacağını bilsin diye son sınıflandırmayı hatırlıyoruz.
    if son_siniflandirma is not None:
        son_siniflandirma[tid] = (analog_sayisal, mod)


def build_scan_freqs(start_mhz, stop_mhz):
    return sdr_common.build_scan_freqs(start_mhz, stop_mhz, SEARCH_STEP_MHZ)


def build_multi_band_scan_freqs():
    """BANDS listesindeki tüm bantları arka arkaya tarayan frekans listesini
    üretir -- varsayılan mod budur (bkz. pluto_ed_scanner.py'deki aynı isimli
    fonksiyon). Bantlar arasındaki boşluklar taranmaz, bir bitince direkt
    sıradakine atlanır."""
    freqs = []
    for band in BANDS:
        freqs.extend(build_scan_freqs(band["start_mhz"], band["stop_mhz"]))
    return freqs


# --- Sinyal İzleme/Dinleme (KTR 4.3) -- gerçek AM/FM demodülasyonu ---
DINLEME_BLOK_SURESI_S = 0.25  # her demodülasyon parçası bu kadar sürer
DINLEME_KAYIT_DIZINI = os.path.join(_REPO_ROOT, "data", "dinleme_kayitlari")
IQ_OVERLAP_SAMPLES = 1024  # ~4ms @ 250 kSPS -- parça sınırı sürekliliği için (bkz. DinlemeOturumu.demodle)
# Boşsa (varsayılan) AI sınıflandırmasına güvenilir; "WBFM" gibi bir değer
# verilirse sınıflandırma yok sayılıp DİNLE hep o tiple demodüle eder --
# sınıflandırma kararsızsa (bkz. handle_dinleme_capture) sahada hızlı çözüm.
DINLEME_MOD_ZORUNLU = os.environ.get("EBABIL_DINLEME_MOD", "")


class DinlemeOturumu:
    """Bir hedefi dinlerken açık kalan .wav dosyasını ve (varsa) canlı ses
    çıkışını yönetir. Süreç boyunca en fazla bir oturum aktif olabilir --
    yeni bir hedef dinlenmeye başlanınca öncekini kapatır.

    Ses çalma sd.OutputStream'in CALLBACK modunda -- PortAudio kendi ayrı
    gerçek-zamanlı thread'inde çalışıp sabit hızda veri istiyor. RF yakalama
    (handle_dinleme_capture, ana döngüde) düzensiz aralıklarla tamamlanabiliyor
    -- isle() artık bloklayan bir write() YAPMIYOR, üretilen parçayı sadece
    bir kuyruğa atıyor (hızlı, hiç beklemiyor). Callback bu kuyruktan çeker;
    kuyruk yetişemezse (RF yakalama gecikirse) kesik/bozuk ses yerine kısa
    bir sessizlikle devam eder -- çok daha pürüzsüz."""

    def __init__(self):
        self.hedef_id = None
        self.wav = None
        self.stream = None
        self._audio_queue = None
        self._leftover = np.zeros(0, dtype=np.float32)
        # Her parça bağımsız demodüle edilince (faz farkı + yeniden-örnekleme
        # filtresi her seferinde sıfırdan başlayınca) parça sınırında küçük
        # bir "tık" oluşuyordu -- bir önceki parçanın son birkaç bin ham
        # örneğini burada tutup bir sonrakinin başına ekliyoruz (bkz.
        # demodle()), süreklilik sağlanıyor.
        self._iq_overlap = np.zeros(0, dtype=np.complex64)

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
        self.wav.setsampwidth(2)  # 16-bit PCM
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


def handle_dinleme_capture(sdr, dinleme, hedef_id, freq_mhz, son_siniflandirma):
    """Bir dinleme döngüsü adımı: freq_mhz'e kilitlenip DINLEME_BLOK_SURESI_S
    kadar örnek alır, hedefin son bilinen sınıflandırmasına göre demodüle
    eder, aktif DinlemeOturumu'na yazar/çalar. Ayrıca aynı örneklerden bir
    güç spektrumu döndürür ki İZLEME modundaki gibi waterfall akmaya devam
    etsin. sdr bir sdr_common.TunedSdr olduğu için freq_mhz bloklar arasında
    DEĞİŞMEDİYSE (asıl senaryo -- aynı hedef dinlenirken) gereksiz retune
    atlanır; önceden HER 250ms'lik blokta yeniden retune ediliyordu, bu da
    üretimin gerçek zamanlıdan geride kalıp kesik/cızırtılı sese yol açan
    ana etkendi."""
    sdr.tune(freq_mhz, SEARCH_SAMPLE_RATE)
    n_samples = int(DINLEME_BLOK_SURESI_S * SEARCH_SAMPLE_RATE)
    samples = sdr.read_samples(n_samples)

    _, mod = son_siniflandirma.get(hedef_id, (None, None))
    if DINLEME_MOD_ZORUNLU:
        # AI sınıflandırması güvenilmezse (donanım kararsızlığı vb. nedenle
        # "Belirsiz" ile gidip geliyorsa) yanlış demodülatöre (AM zarf) düşüp
        # sesin bozuk/anlaşılmaz çıkmasına yol açabiliyordu -- bu ortam
        # değişkeniyle sabit bir tipe zorlanabilir (pluto_ed_scanner.py'deki
        # aynı çözümle tutarlı).
        mod = DINLEME_MOD_ZORUNLU
    audio = dinleme.demodle(samples, SEARCH_SAMPLE_RATE, mod)
    dinleme.isle(audio)

    return sdr_common.compute_power_spectrum(samples[:FFT_SIZE], freq_mhz, SEARCH_SAMPLE_RATE)


def build_sys_fields(tracker, tid):
    """tracker.known[tid]'den (zaten update() ile dolduruldu) tam SYS satırını
    üretir -- klasik parametre çıkarımının hepsini taşır (KTR Tablo 8):
    SYS,id,tespit,lat,lon,alt,freq,güç,bant_gen,frekans_sapması,gürültü_tabanı,SNR,süreklilik.
    ARAMA ve İZLEME modlarının ikisi de aynı formatı kullansın diye ortak."""
    info = tracker.known[tid]
    snr_db = info.get("snr_db")
    sapma_mhz = info.get("freq_sapmasi_mhz")
    gurultu_db = info["power_db"] - snr_db if snr_db is not None else float("nan")
    sureklilik = tracker.sureklilik_durumu(tid)
    return [
        "SYS", tid, "1", "nan", "nan", "0",
        f"{info['freq_mhz']:.3f}", f"{info['power_db']:.2f}", f"{info['bandwidth_khz']:.1f}",
        f"{sapma_mhz:.4f}" if sapma_mhz is not None else "nan",
        f"{gurultu_db:.2f}" if snr_db is not None else "nan",
        f"{snr_db:.2f}" if snr_db is not None else "nan",
        sureklilik,
    ]


# pick_target artık sdr_common'da -- pluto_ed_scanner.py ile ORTAK (aynı
# "seçili hedef yoksa en güçlüye düş" davranışı iki dosyada da isteniyor).
pick_target = sdr_common.pick_target
# konum_guncelle_ve_gonder de aynı şekilde konum_istemcisi'nde -- pluto_ed_scanner.py
# ile ORTAK (bkz. o dosyadaki import).


def main():
    # Varsayılan olarak tek makine (loopback) mimarisi. Bu betik başka bir
    # makinede (örn. Jetson) çalışıp GUI ayrı bir makinede (örn. Windows PC)
    # ise EBABIL_ZMQ_BIND_HOST=0.0.0.0 (PUB'ların dışarıdan erişilebilmesi
    # için) ve EBABIL_GUI_HOST=<GUI'nin LAN IP'si> (GUI'nin bind ettiği komut
    # kanalına ulaşmak için) ayarla.
    zmq_bind_host = os.environ.get("EBABIL_ZMQ_BIND_HOST", "127.0.0.1")
    gui_host = os.environ.get("EBABIL_GUI_HOST", "127.0.0.1")

    context = zmq.Context()

    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://{zmq_bind_host}:5555")

    pub_ai = context.socket(zmq.PUB)
    pub_ai.bind(f"tcp://{zmq_bind_host}:5556")

    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect(f"tcp://{gui_host}:5557")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    print("[*] Model yükleniyor...")
    model, feature_mean, feature_std = load_model_and_scalers()
    print("[+] Model hazır.")

    # TunedSdr, aynı frekans/hıza tekrar tekrar kilitlenmek istendiğinde
    # (DİNLE, sınıflandırma, dwell) gereksiz retune'u kendi içinde önbellekle
    # atlıyor -- bkz. sdr_common.TunedSdr. Çağıran kod (aşağıdaki main döngüsü
    # ve capture_power_spectrum/handle_dinleme_capture/handle_classify_request/
    # handle_save_command) artık "hangi hızdayız" diye ayrıca takip etmiyor,
    # sadece sdr.tune(freq, rate) der. RtlAntenliSdr bunu genişletip HER
    # tune() çağrısında RF anahtarını da (varsa) doğru porta alıyor -- tüm
    # çağıranlar tek bir yerden (burada) otomatik doğru anteni kullanır.
    rtl_rf_anahtari = RtlRfAnahtari()
    sdr = RtlAntenliSdr(RtlSdr(), throwaway_samples=THROWAWAY_SAMPLES, rf_anahtari=rtl_rf_anahtari)
    sdr.gain = "auto"

    tracker = TargetTracker(match_tolerance_mhz=TARGET_MATCH_TOLERANCE_MHZ)

    # Yön bulma + konum kestirimi (madde 5.1.4/5.1.5) -- anten çifti donanımı
    # YOK (tek anten, doğrulandı), bu yüzden "menzil-only": gerçek RSSI +
    # gerçek İHA konumu (mavlink_bridge.py'den) konum_servisi'ne (C++ PF/EKF)
    # gönderiliyor, o da hedef konumu + türetilmiş açıyı döndürüyor (bkz.
    # konum_istemcisi.py). İkisi de arka planda/best-effort -- mavlink_bridge
    # ya da konum_servisi henüz çalışmıyorsa DF satırı basitçe gönderilmez,
    # SYS/SPEC akışı etkilenmez.
    uav_konum = UavKonumDinleyici()
    konum_istemcisi = KonumIstemcisi()

    # Bant aralığı artık çalışırken değiştirilebilir (bkz. aşağıdaki "bant" ve
    # "frekans" komutları) -- şartname madde 5.1.1: hakemler önce hiçbir şey
    # söylemez, hiçbir takım bulamazsa önce BANT sonra FREKANS açıklayabilir.
    # Bu durumda süreci yeniden başlatmadan taramayı daraltabilmemiz gerekiyor.
    # Varsayılan: TEK bir aralık değil, BANDS'teki TÜM bantlar sırayla --
    # operatör hedefin hangi bantta olduğunu önceden bilmiyor.
    varsayilan_scan_freqs = build_multi_band_scan_freqs()  # "bant varsayilan" ile buna geri dönülür
    scan_freqs = varsayilan_scan_freqs
    scan_idx = 0

    dwelling = False
    dwell_center_mhz = None
    dwell_target_id = None  # otomatik modda pick_target()'ın histerezisi için "şu an kilitli hedef"
    dwell_started_at = 0.0
    dwell_locked = False  # "frekans" komutuyla kilitlendiyse DWELL_DURATION_S sonra arama moduna dönmez

    # Operatör GUI'de belirli bir HEDEF-N kartını seçtiğinde ("hedef <id>"
    # komutu) buraya yazılır -- artık "en son bulunan" değil, operatörün
    # SEÇTİĞİ hedefe kilitleniriz (bkz. pick_target). "hedef oto" veya
    # "hedef <id>" ile tekrar otomatik moda dönülebilir.
    selected_target_id = None

    # --- Sinyal İzleme/Dinleme (KTR 4.3) durumu ---
    # tid -> (analogSayisal, modulasyonTuru) -- handle_classify_request her
    # sınıflandırmada günceller, handle_dinleme_capture hangi demodülatörü
    # kullanacağını buradan öğrenir.
    son_siniflandirma = {}
    dinleme = DinlemeOturumu()
    dinleme_hedef_id = None  # None = dinleme kapalı, aksi halde dinlenen hedefin id'si

    # GUI'deki "TARAMAYI DURDUR" düğmesiyle -- True iken ARAMA/İZLEME tamamen
    # durur (SDR'a hiç dokunulmaz, SPEC/SYS yayınlanmaz), DİNLE etkilenmez.
    tarama_duraklatildi = False
    son_duraklatma_heartbeat = 0.0  # bkz. aşağıdaki "DURAKLATILDI" dalı

    command_queue = queue.Queue()
    threading.Thread(target=stdin_command_reader, args=(command_queue,), daemon=True).start()

    band_names = ", ".join(f"{b['name']} MHz" for b in BANDS)
    print(f"[*] Taranacak bantlar: {band_names} ({len(scan_freqs)} adım toplam, "
          f"adım genişliği {SEARCH_STEP_MHZ:.3f} MHz)")
    print("[*] Port 5555: SYS/SPEC | Port 5556: AI | Port 5557: komut dinleniyor")
    print("[*] Gerçek veri kaydetmek için: kaydet <ETİKET> [süre_sn]  (örn: kaydet WBFM 10)")
    print("[*] Hakem bant açıklarsa: bant <başlangıç_mhz> <bitiş_mhz>  (örn: bant 433.0 435.0)")
    print("[*] Hakem tam frekans açıklarsa: frekans <mhz>  (örn: frekans 433.92)\n")

    try:
        while True:
            try:
                # --- ZMQ KOMUTU (arayüzden/GUI'den) ---
                try:
                    msg = sub_cmd.recv_string(flags=zmq.NOBLOCK)
                    if msg == "SDR_VERISI_ISTEK":
                        handle_classify_request(sdr, pub_ai, tracker, model, feature_mean, feature_std,
                                                 selected_target_id, son_siniflandirma)
                    elif msg.startswith("ET,BASLAT,") or msg.startswith("ET,DURDUR,"):
                        # "ET,BASLAT,<görev_kodu>,<frekans_mhz>" / "ET,DURDUR,<görev_kodu>".
                        # ZMQ PUB/SUB fan-out olduğu için bu SATIR SADECE görünürlük/log
                        # amaçlı -- gerçek Pluto TX (karıştırma/aldatma) et_control.py'nin
                        # AYRI süreci tarafından yapılıyor, o da aynı 5557 komut kanalına
                        # bağımsız SUB olarak dinliyor (burada tekrar TX tetiklenmiyor).
                        print(f"[*] ET komutu alındı (SYS/SPEC tarafı sadece logluyor, TX et_control.py'de): {msg}")
                    elif msg.startswith("SET_POWER "):
                        print(f"[*] Çıkış gücü ayarı alındı (SYS/SPEC tarafı sadece logluyor, TX et_control.py'de): {msg}")
                    elif msg == "BANT_VARSAYILAN":
                        command_queue.put("bant varsayilan")
                    elif msg.startswith("BANT_AYARLA|"):
                        # BANT_AYARLA|<başlangıç_mhz>|<bitiş_mhz> -- arayüzden gelebilecek
                        # eşdeğeri, bkz. aşağıdaki "bant" klavye komutu.
                        try:
                            _, start_s, stop_s = msg.split("|")
                            command_queue.put(f"bant {start_s} {stop_s}")
                        except ValueError:
                            print(f"[!] Geçersiz BANT_AYARLA komutu: {msg}")
                    elif msg.startswith("FREKANS_KILITLE|"):
                        try:
                            _, freq_s = msg.split("|")
                            command_queue.put(f"frekans {freq_s}")
                        except ValueError:
                            print(f"[!] Geçersiz FREKANS_KILITLE komutu: {msg}")
                    elif msg.startswith("HEDEF_SEC|"):
                        # HEDEF_SEC|<hedef_id> -- GUI'de bir HEDEF-N kartına
                        # tıklandığında; "HEDEF_SEC|OTOMATIK" seçimi temizler.
                        try:
                            _, target_id = msg.split("|")
                            command_queue.put(f"hedef {target_id}")
                        except ValueError:
                            print(f"[!] Geçersiz HEDEF_SEC komutu: {msg}")
                    elif msg.startswith("DINLE_BASLAT|"):
                        # DINLE_BASLAT|<hedef_id> -- gerçek AM/FM demodülasyonu
                        # başlat (bkz. DinlemeOturumu). Hedef bilinmiyorsa yok sayılır.
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
                    elif msg == "TARAMA_DURDUR":
                        # Sürekli tarama (her adımda retune+capture+SPEC yayını)
                        # zayıf sistemlerde GUI'yi zorluyor -- operatör izlemeye
                        # ara vermek isterse burada tamamen durur (DİNLE aktifse
                        # etkilenmez, bkz. aşağıdaki mod seçimi).
                        tarama_duraklatildi = True
                        pub.send_string("DURUM,DURAKLATILDI")
                        print("[*] Tarama duraklatıldı.")
                    elif msg == "TARAMA_DEVAM":
                        tarama_duraklatildi = False
                        print("[*] Tarama devam ediyor.")
                except zmq.Again:
                    pass

                # --- KLAVYE KOMUTU ---
                try:
                    line = command_queue.get_nowait()
                    lower = line.lower()
                    if lower.startswith("kaydet"):
                        handle_save_command(sdr, tracker, line[len("kaydet"):].strip(), selected_target_id)

                    elif lower.startswith("bant"):
                        # Şartname madde 5.1.1: hiçbir takım sinyali bulamazsa
                        # hakemler önce BANDI açıklayabilir -- taramayı o
                        # aralığa daraltıp süreci yeniden başlatmadan devam
                        # ederiz. Mevcut hedefler/izleme durumu korunur,
                        # sadece arama aralığı ve o anki tur sıfırlanır.
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
                            print("[!] Kullanım: bant <başlangıç_mhz> <bitiş_mhz>  (örn: bant 433.0 435.0)  |  bant varsayilan")
                        else:
                            try:
                                new_start, new_stop = float(parts[1]), float(parts[2])
                                if new_start >= new_stop:
                                    print(f"[!] Başlangıç bitişten küçük olmalı: {new_start} >= {new_stop}")
                                else:
                                    scan_freqs = build_scan_freqs(new_start, new_stop)
                                    scan_idx = 0
                                    dwelling = False
                                    dwell_locked = False
                                    print(f"[*] Tarama aralığı güncellendi: {new_start}-{new_stop} MHz "
                                          f"({len(scan_freqs)} adım)")
                            except ValueError:
                                print(f"[!] Geçersiz sayı: {line!r}")

                    elif lower.startswith("frekans"):
                        # Şartname madde 5.1.1: hakemler bandın ardından TAM
                        # frekansı da açıklayabilir -- artık aramaya hiç gerek
                        # yok, doğrudan o frekansa kilitlenip izleme moduna geçiyoruz.
                        parts = line.split()
                        if len(parts) != 2:
                            print("[!] Kullanım: frekans <mhz>  (örn: frekans 433.92)")
                        else:
                            try:
                                exact_freq = float(parts[1])
                                dwell_center_mhz = round(exact_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                                dwelling = True
                                dwell_locked = True
                                dwell_started_at = time.time()
                                print(f"[*] {dwell_center_mhz:.3f} MHz'e kilitlendi, izleme modunda.")
                            except ValueError:
                                print(f"[!] Geçersiz sayı: {line!r}")

                    elif lower.startswith("hedef"):
                        # "hedef <id>": belirtilen HEDEF-N'e kilitlen (en son
                        # bulunana değil) -- kaydet/sınıflandırma/dwell hep bunu
                        # kullanır. "hedef oto": seçimi temizle, eski otomatik
                        # ("en son bulunan hedef") davranışına dön.
                        parts = line.split()
                        if len(parts) != 2:
                            print("[!] Kullanım: hedef <id>  (örn: hedef HEDEF-2)  |  hedef oto")
                        elif parts[1].upper() in ("OTO", "OTOMATIK"):
                            selected_target_id = None
                            dwell_target_id = None
                            dwelling = False
                            dwell_locked = False
                            scan_idx = 0
                            print("[*] Hedef seçimi temizlendi -- otomatik moda (en son bulunan) dönüldü.")
                        else:
                            target_id = parts[1].upper()
                            if target_id not in tracker.known:
                                print(f"[!] {target_id} henüz bilinmiyor -- önce tespit edilmiş olmalı "
                                      f"(bilinen hedefler: {', '.join(tracker.known) or '(yok)'})")
                            else:
                                selected_target_id = target_id
                                dwell_target_id = target_id
                                raw_freq = tracker.known[target_id]["freq_mhz"]
                                dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                                dwelling = True
                                # Kasıtlı olarak dwell_locked=False -- operatörün karta
                                # tıklaması (görüntülemek/DİNLE için) taramayı SONSUZA
                                # KADAR durdurmamalı. DWELL_DURATION_S sonra otomatik
                                # tam-bant turuna döner, tur bitince selected_target_id'ye
                                # tekrar döner -- bu hedef ÖNCELİKLİ ama taramayı
                                # ENGELLEMİYOR. Tam kilit sadece hakemin gerçek frekansı
                                # açıkladığı "frekans" komutunda (yukarıda, satır ~496).
                                dwell_locked = False
                                dwell_started_at = time.time()
                                print(f"[*] {target_id} seçildi, {dwell_center_mhz:.3f} MHz'e odaklanıldı "
                                      f"(tam bant taraması devam edecek).")

                    else:
                        print(f"[!] Bilinmeyen komut: {line!r} "
                              f"(kullanım: kaydet <ETİKET> [süre_sn] | bant <başlangıç> <bitiş> | "
                              f"frekans <mhz> | hedef <id>|oto)")
                except queue.Empty:
                    pass

                # --- MOD SEÇİMİ ---
                # dwell_locked: "frekans" komutuyla operatör bilerek kilitlendiyse
                # (hakem tam frekansı açıkladıysa) süre dolunca arama moduna
                # dönmüyoruz -- artık aranacak bir şey yok, o frekansta kalınır.
                if dwelling and not dwell_locked and (time.time() - dwell_started_at > DWELL_DURATION_S):
                    dwelling = False  # süre doldu, kısa bir arama turuna dön

                if dinleme_hedef_id is not None:
                    # --- DİNLEME: gerçek AM/FM demodülasyonu -- taramaya ara verir,
                    # sadece dinlenen hedefin frekansına kilitli kalır (bkz.
                    # handle_dinleme_capture / DinlemeOturumu).
                    if dinleme_hedef_id not in tracker.known:
                        print(f"[!] {dinleme_hedef_id} artık bilinmiyor, dinleme durduruluyor.")
                        dinleme.durdur()
                        dinleme_hedef_id = None
                        pub.send_string("DURUM,TARIYOR")
                    else:
                        dinleme_freq_mhz = tracker.known[dinleme_hedef_id]["freq_mhz"]
                        binned_db, bin_freqs_mhz, fs_mhz = handle_dinleme_capture(
                            sdr, dinleme, dinleme_hedef_id, dinleme_freq_mhz, son_siniflandirma)
                        spec_fields = ["SPEC", f"{dinleme_freq_mhz:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                        pub.send_string(",".join(spec_fields))

                elif tarama_duraklatildi:
                    # --- DURAKLATILDI: operatör "TARAMAYI DURDUR" dedi --
                    # SDR'a hiç dokunulmuyor, SPEC/SYS yayınlanmıyor (waterfall
                    # olduğu yerde donuk kalır), ama backend CPU/USB yükü
                    # gerçekten düşer. Yine de GUI'nin bağlantı-canlılık
                    # kontrolü (bkz. mainwindow.cpp baglantiTazelikKontrolu --
                    # birkaç saniyedir HİÇBİR paket gelmezse "BAĞLI DEĞİL"
                    # sayıyor) duraklatmayı kopma sanmasın diye hafif bir
                    # "hâlâ buradayım" mesajı göndermeye devam ediyoruz.
                    if time.time() - son_duraklatma_heartbeat > 1.0:
                        pub.send_string("DURUM,DURAKLATILDI")
                        son_duraklatma_heartbeat = time.time()
                    time.sleep(0.2)

                elif dwelling:
                    # --- İZLEME (dwell): hedefe kilitli, geniş bant, sabit merkez ---
                    binned_db, bin_freqs_mhz, fs_mhz = capture_power_spectrum(sdr, dwell_center_mhz, DWELL_SAMPLE_RATE)

                    spec_fields = ["SPEC", f"{dwell_center_mhz:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                    pub.send_string(",".join(spec_fields))

                    peak = detect_peak(binned_db, bin_freqs_mhz)
                    if peak is not None:
                        freq_mhz, power_db, bandwidth_khz, noise_floor_db = peak
                        snr_db = power_db - noise_floor_db
                        sapma_mhz = freq_mhz - dwell_center_mhz
                        tid = tracker.update(freq_mhz, power_db, bandwidth_khz, snr_db, sapma_mhz)
                        pub.send_string(",".join(build_sys_fields(tracker, tid)))
                        konum_guncelle_ve_gonder(pub, konum_istemcisi, uav_konum, tid, freq_mhz, power_db)

                else:
                    # --- ARAMA: tüm bandı ince adımlarla dolaş ---
                    center_mhz = scan_freqs[scan_idx]
                    scan_idx += 1

                    binned_db, bin_freqs_mhz, fs_mhz = capture_power_spectrum(sdr, center_mhz, SEARCH_SAMPLE_RATE)

                    spec_fields = ["SPEC", f"{center_mhz:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                    pub.send_string(",".join(spec_fields))

                    peak = detect_peak(binned_db, bin_freqs_mhz)
                    if peak is not None:
                        freq_mhz, power_db, bandwidth_khz, noise_floor_db = peak
                        snr_db = power_db - noise_floor_db
                        sapma_mhz = freq_mhz - center_mhz
                        tid = tracker.update(freq_mhz, power_db, bandwidth_khz, snr_db, sapma_mhz)
                        pub.send_string(",".join(build_sys_fields(tracker, tid)))
                        konum_guncelle_ve_gonder(pub, konum_istemcisi, uav_konum, tid, freq_mhz, power_db)

                    if scan_idx >= len(scan_freqs):
                        # Bir tam tur bitti -- operatör bir hedef SEÇTİYSE onun
                        # üzerine, seçmediyse en güçlü hedefin üzerine kilitlen.
                        # current_id=dwell_target_id ile histerezis uygulanır --
                        # birden fazla güçlü hedef varken (ör. yarışma alanında
                        # başka takımların yayınları) küçük güç farklarıyla her
                        # turda hedef değiştirip gereksiz retune yapılmasın diye.
                        scan_idx = 0
                        lock_target = pick_target(tracker, selected_target_id, dwell_target_id)
                        if lock_target is not None:
                            dwell_target_id = lock_target
                            raw_freq = tracker.known[lock_target]["freq_mhz"]
                            dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                            dwelling = True
                            dwell_started_at = time.time()

            except Exception as e:
                # RTL-SDR ile uzun süre çalışırken ara sıra geçici USB
                # hataları (ör. LIBUSB_ERROR_PIPE) görülebiliyor. Tek bir
                # hatada tüm süreç ölmesin -- logla, kısa bir bekleme sonrası
                # taramaya devam et. Sorun kalıcıysa (cihaz gerçekten koptu)
                # bu döngü sürekli hata basar, en azından süreç ayakta kalır
                # ve loglardan fark edilir.
                print(f"[!] Tarama/sınıflandırma sırasında hata (devam ediliyor): {e}")
                sdr.invalidate()  # donanımın gerçek durumu şüpheli, bir sonraki tune() zorla yeniden ayarlasın
                time.sleep(0.5)

    except KeyboardInterrupt:
        # et_control.py ve pluto_ed_scanner.py ile aynı davranış -- Ctrl+C
        # temiz çıksın (traceback basmadan), donanım yine de finally'de kapatılır.
        pass
    finally:
        dinleme.durdur()
        sdr.close()
        print("\n[*] streamer kapatıldı.")


if __name__ == "__main__":
    main()
