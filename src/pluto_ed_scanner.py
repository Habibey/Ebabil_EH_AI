"""
İKİNCİ PlutoSDR'ı RX olarak kullanıp 868-870 MHz ve 2.4-2.483 GHz bantlarını
hoplayarak tarayan ED (Elektronik Destek) süreci -- streamer.py'nin RTL-SDR
(144/433 MHz) tarafına ek, KTR'de tanımlı ama şu ana kadar kodda hiç var
olmayan üst-bant tarama zincirini gerçekleştirir.

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

Sınıflandırma (AI) YOK -- bu scriptin işi sadece tespit (OS-CFAR ile "burada
bir şey var mı"). Modülasyon sınıflandırması istenirse streamer.py'deki gibi
ayrıca eklenir; şimdilik kapsam dışı (KTR'deki YOLO/FHSS-örüntü tanıma da
büyük ayrı bir iş, bu script onu içermiyor).

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

import numpy as np
import zmq
from scipy.ndimage import gaussian_filter1d

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..")
sys.path.insert(0, _THIS_DIR)
os.environ["PATH"] = os.path.join(_REPO_ROOT, "tools", "libiio") + os.pathsep + os.environ.get("PATH", "")

try:
    import sounddevice as sd
except Exception:
    sd = None  # Kurulu değilse dinleme sesi sadece .wav'a yazılır, canlı çalınmaz.

import sdr_common
import demod

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
    {"name": "143-145", "start_mhz": 143.0, "stop_mhz": 145.0, "step_mhz": 0.5},
    {"name": "430-440", "start_mhz": 430.0, "stop_mhz": 440.0, "step_mhz": 0.5},
    {"name": "868-870", "start_mhz": 868.0, "stop_mhz": 870.0, "step_mhz": 1.0},
    {"name": "2400-2483", "start_mhz": 2400.0, "stop_mhz": 2483.0, "step_mhz": 1.0},
]

SEARCH_SAMPLE_RATE = 2_000_000  # arama modu -- Pluto'nun genis RX bandini kullanip az adimda tara
SEARCH_STEP_MHZ = SEARCH_SAMPLE_RATE / 1e6  # bantlarda step_mhz belirtilmemişse varsayılan

DWELL_SAMPLE_RATE = 4_000_000  # izleme modu -- daha genis, stabil waterfall
DWELL_DURATION_S = 4.0
DWELL_SNAP_MHZ = 0.1

THROWAWAY_SAMPLES = 1024
SYS_SPEC_PORT = 5560  # streamer.py'nin 5555/5556'siyla CAKISMASIN diye ayri
CMD_PORT = 5557  # komut kanali streamer.py ile PAYLASILIYOR (ayni PUB/SUB fan-out)


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
        self.pluto.gain_control_mode_chan0 = "slow_attack"  # otomatik kazanç -- RTL-SDR'daki gain='auto' eşdeğeri
        print("[+] Pluto RX hazır.")

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


def handle_dinleme_capture(rx, dinleme, hedef_id, freq_mhz):
    t0 = time.time()
    n_samples = int(DINLEME_BLOK_SURESI_S * DINLEME_SAMPLE_RATE)
    samples = rx.capture_raw(freq_mhz, DINLEME_SAMPLE_RATE, n_samples)
    audio = dinleme.demodle(samples, DINLEME_SAMPLE_RATE, DINLEME_VARSAYILAN_MOD)
    dinleme.isle(audio)
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

    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect(f"tcp://127.0.0.1:{CMD_PORT}")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    rx = PlutoRX()
    rx.connect()

    tracker = sdr_common.TargetTracker(id_prefix="PHEDEF")  # streamer.py'nin HEDEF-N'iyle karışmasın
    varsayilan_scan_freqs = build_multi_band_scan_freqs()  # "bant varsayilan" ile buna geri dönülür
    scan_freqs = varsayilan_scan_freqs
    scan_idx = 0

    dwelling = False
    dwell_center_mhz = None
    dwell_started_at = 0.0
    dwell_locked = False
    selected_target_id = None

    dinleme = DinlemeOturumu()
    dinleme_hedef_id = None

    command_queue = queue.Queue()
    threading.Thread(target=stdin_command_reader, args=(command_queue,), daemon=True).start()

    band_names = ", ".join(f"{b['name']} MHz" for b in BANDS)
    print(f"[*] Taranacak bantlar: {band_names} ({len(scan_freqs)} adım toplam)")
    print(f"[*] Port {SYS_SPEC_PORT}: SYS/SPEC | Port {CMD_PORT}: komut dinleniyor (paylaşımlı)")
    print("[*] Belirli bir hedefe kilitlenmek için: hedef <id>  |  hedef oto\n")

    try:
        while True:
            try:
                try:
                    msg = sub_cmd.recv_string(flags=zmq.NOBLOCK)
                    if msg.startswith("PLUTO_ED_HEDEF_SEC|"):
                        _, target_id = msg.split("|")
                        command_queue.put(f"hedef {target_id}")
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
                            rx, dinleme, dinleme_hedef_id, dinleme_freq_mhz)
                        spec_fields = ["SPEC", f"{dinleme_freq_mhz:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                        pub.send_string(",".join(spec_fields))

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

                    if not dwelling and scan_idx >= len(scan_freqs):
                        scan_idx = 0
                        lock_target = None
                        if selected_target_id is not None and selected_target_id in tracker.known:
                            lock_target = selected_target_id
                        else:
                            # "En son görülen" değil "en güçlü" -- zayıf/aralıklı
                            # gürültü kırıntıları arada bir görülüp otomatik
                            # modu ele geçirmesin (bkz. sdr_common.most_powerful).
                            lock_target = tracker.most_powerful()
                        if lock_target is not None:
                            raw_freq = tracker.known[lock_target]["freq_mhz"]
                            dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                            dwelling = True
                            dwell_started_at = time.time()

                if DRY_RUN:
                    time.sleep(0.05)  # sahte modda CPU'yu bogmasin

            except Exception as e:
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
