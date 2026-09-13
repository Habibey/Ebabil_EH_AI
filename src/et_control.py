"""
Kendi et_kontrol'ümüz: GUI'nin zaten konuştuğu ET protokolünü (ET,BASLAT/DURDUR,
SET_POWER -- bkz. mainwindow.cpp) dinleyip PlutoSDR TX üzerinden GERÇEKTEN RF
yayınlayan bağımsız süreç. streamer.py bu komutları sadece LOGLUYORDU; bu dosya
onun yanında/yerine çalışıp fiilen vericiyi tetikler.

Neden ayrı süreç: streamer.py RTL-SDR'ı (arama/tespit) yönetiyor, bu dosya ise
PlutoSDR'ı (TX) yönetiyor -- farklı donanım, farklı süreç (mavlink_bridge.py ile
aynı prensip). Aynı komut kanalını (port 5557, PUB/SUB fan-out) streamer.py ile
birlikte dinleyebilir; ikisi de bağımsız SUB'dır, çakışmazlar.

Kritik Tasarım Raporu'ndaki (Bölüm 5) mimariyle hizalı ama KISITLI kapsam:
  - 5.1 Sürekli Karıştırma: DESTEKLENIYOR, KTR'nin 3 profili de var:
      TEKLI  -- dar bant (KARISTIRMA_BW_HZ), tek hedef.
      BARAJ  -- geniş bant (BARAJ_BW_HZ), tek hedef -- belirsiz/geniş kapsama.
      COKLU  -- aynı anda ≤3 hedef: her biri için ayrı dar bant gürültü
        üretilip, ortak bir LO'ya göre frekans ofsetine kaydırılıp TEK bir
        kompozit dalga formunda toplanır (bkz. generate_multi_target_noise).
        Hedefler Pluto'nun anlık örnekleme hızına (SAMPLE_RATE) sığacak kadar
        yakın olmalı -- aksi halde en uzak hedeflerde etkinlik düşer, sadece
        bir uyarı basılır, iş durmaz.
    Protokol artık "ET,BASLAT,<görev>,<f1[;f2;f3]>,<tip>" -- tip TEKLI/COKLU/
    BARAJ, frekans alanı COKLU'da noktalı virgülle ayrılmış liste.
  - 5.3 Analog Telsiz Aldatma: DESTEKLENIYOR, operatör GUI'den kaynak seçebilir
    (bkz. ALDATMA_KAYNAK| komutu): KAYIT_TEKRAR (data/aldatma_sesleri/'den
    seçilen bir .wav) veya PIPER_TTS (yerel/çevrimdışı Türkçe TTS, girilen
    metni sentezler -- models/piper_voices/tr_TR-dfki-medium/). Hiçbiri
    seçilmemişse veya başarısız olursa sentetik bir uyarı tonuna düşer --
    hiçbir zaman sessiz kalmaz.
  - 5.2 Arabakışlı Karıştırma: DESTEKLENIYOR -- AYNI Pluto'yu zaman paylaşımlı
    TX/RX olarak kullanır: ARABAKISLI_TX_BURST_S kadar karıştırır, TX'i
    durdurup ARABAKISLI_RX_WINDOW_S kadar dinler, hedef hâlâ eşik üstündeyse
    devam eder -- ard arda ARABAKISLI_MAX_MISSES kez hedefi kaybederse görevi
    kendiliğinden sonlandırır (KTR 5.2: "eşik altındaysa hedefin sonlandığı
    kabul edilir"). ASLA aynı anda TX+RX yapmaz -- ayrı bir thread'de
    (ana ZMQ komut döngüsünü bloklamaması için) sırayla TX/RX arasında geçer.
  - 5.4 GNSS Aldatma: HENÜZ YOK -- GUI tarafında da komut yok.

GÜVENLİK: Varsayılan TX kazancı düşük tutulur (bkz. DEFAULT_TX_GAIN_DB).
Bu takımın Pluto'su yazılımsal yamayla 70MHz-6GHz'e genişletildi (bkz.
PLUTO_TX_MIN_HZ/MAX_HZ) -- standart Pluto'nun (~325MHz-3.8GHz) dışında kalan
bantlar (ör. 144 MHz) artık TX edilebiliyor, ama bu "resmi olmayan" uçlarda
kalibrasyon verisi yok, çıkış gücü daha az öngörülebilir olabilir.

Kullanım:
  python src/et_control.py
  EBABIL_ET_DRY_RUN=1 python src/et_control.py   # Pluto takılı değilken mantığı test et
"""
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
import wave

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..")
sys.path.insert(0, os.path.join(_REPO_ROOT, "tools", "dataset"))
if sys.platform == "win32":
    # libiio.dll'i bulmak için -- Linux/Jetson'da libiio sistem paketinden
    # (apt: libiio0, pip: pylibiio/pyadi-iio) geldiği için buna gerek yok.
    os.environ["PATH"] = os.path.join(_REPO_ROOT, "tools", "libiio") + os.pathsep + os.environ.get("PATH", "")

import numpy as np
import zmq
from scipy.ndimage import gaussian_filter1d

import modulators  # tools/dataset/modulators.py -- PLUTO_TX_SAMPLE_RATE ve üretim stili

DRY_RUN = os.environ.get("EBABIL_ET_DRY_RUN", "0") == "1"

SAMPLE_RATE = modulators.PLUTO_TX_SAMPLE_RATE  # 1 MSPS
TOTAL_SAMPLES = modulators.TOTAL_SAMPLES  # ~60ms döngüsel arabellek

DEFAULT_TX_GAIN_DB = float(os.environ.get("EBABIL_ET_TX_GAIN_DB", -40.0))  # Pluto: 0=tam güç, negatif=zayıflatma
MIN_TX_GAIN_DB = -89.75
MAX_TX_GAIN_DB = 0.0

# Sürekli karıştırma gürültü bant genişliği -- TEKLİ profili.
KARISTIRMA_BW_HZ = float(os.environ.get("EBABIL_KARISTIRMA_BW_HZ", 60000.0))
# BARAJ profili -- belirsiz/geniş frekans davranışı için çok daha geniş bant.
# SAMPLE_RATE'in (1 MHz) önemli bir kısmını kaplayacak şekilde seçildi.
BARAJ_BW_HZ = float(os.environ.get("EBABIL_BARAJ_BW_HZ", 400000.0))

# Arabakışlı karıştırma zaman paylaşımı -- streamer.py'nin PEAK_THRESHOLD_DB'siyle
# tutarlı bir eşik kullanıyoruz (aynı "gürültü tabanının X dB üstü = sinyal var" mantığı).
ARABAKISLI_TX_BURST_S = float(os.environ.get("EBABIL_ARABAKISLI_TX_S", 2.0))
ARABAKISLI_RX_WINDOW_S = float(os.environ.get("EBABIL_ARABAKISLI_RX_S", 0.3))
ARABAKISLI_RX_NSAMPLES = 4096
ARABAKISLI_THRESHOLD_DB = 10.0
ARABAKISLI_MAX_MISSES = 3  # ard arda bu kadar "hedef yok" ölçümünden sonra görev kendiliğinden durur

# Standart Pluto'nun (AD9363) fabrika TX aralığı ~325MHz-3.8GHz'dir, ama bu
# takımın Pluto'su yazılımsal bir yamayla 70MHz-6GHz'e genişletildi (AD9363
# çipinin kendisi bunu destekliyor, Analog Devices'ın resmi/kalibre ettiği
# aralık daha darmış). Bu sayede 144 MHz gibi KTR'de sorun olan bantlar da
# artık TX edilebilir. UYARI: 70-325MHz ve 3.8-6GHz gibi "resmi olmayan"
# uçlarda kalibrasyon verisi yok -- çıkış gücü/doğruluk merkez banda göre
# daha az güvenilir olabilir, sahada doğrulanmalı.
PLUTO_TX_MIN_HZ = float(os.environ.get("EBABIL_PLUTO_TX_MIN_HZ", 70e6))
PLUTO_TX_MAX_HZ = float(os.environ.get("EBABIL_PLUTO_TX_MAX_HZ", 6e9))

ALDATMA_SES_DIZINI = os.path.join(_REPO_ROOT, "data", "aldatma_sesleri")

# Piper TTS -- yerel/cevrimdisi, Turkce ses modeli. pip install piper-tts +
# models/piper_voices/tr_TR-dfki-medium/ altina indirilmis .onnx/.onnx.json.
PIPER_MODEL = os.path.join(_REPO_ROOT, "models", "piper_voices", "tr_TR-dfki-medium", "tr_TR-dfki-medium.onnx")
PIPER_CONFIG = PIPER_MODEL + ".json"

# Operatörün GUI'den seçtiği aldatma mesajı kaynağı -- ALDATMA_KAYNAK|
# komutuyla güncellenir (bkz. main()), handle_baslat çağrıldığında okunur.
# tur: "OTOMATIK" (eski davranış: ilk .wav varsa o, yoksa sentetik ton) |
# "KAYIT_TEKRAR" (deger=dosya adı) | "PIPER_TTS" (deger=söylenecek metin).
_aldatma_kaynak = {"tur": "OTOMATIK", "deger": None}


def set_aldatma_kaynak(tur, deger):
    _aldatma_kaynak["tur"] = tur
    _aldatma_kaynak["deger"] = deger
    print(f"[*] Aldatma kaynağı ayarlandı: {tur} ({deger!r})")


def generate_barrage_noise(bw_hz, n=TOTAL_SAMPLES, fs=SAMPLE_RATE):
    """Karıştırma için bant-sınırlı kompleks gürültü. Gaussian alçak geçiren
    filtre ile I/Q ayrı ayrı yumuşatılarak yaklaşık bw_hz'lik bir güç
    yoğunluğu bandı elde edilir (kesin/keskin bir filtre değil, amaç RF
    zincirini doyuracak kaba bir gürültü tabanı üretmek)."""
    i = np.random.randn(n)
    q = np.random.randn(n)
    sigma = fs / (2 * np.pi * max(bw_hz, 1000.0))
    if sigma > 1.0:
        i = gaussian_filter1d(i, sigma=sigma, mode="wrap")
        q = gaussian_filter1d(q, sigma=sigma, mode="wrap")
    sig = (i + 1j * q).astype(np.complex64)
    sig /= np.max(np.abs(sig)) + 1e-9
    return sig


def generate_multi_target_noise(freqs_mhz, lo_mhz, bw_hz_each=KARISTIRMA_BW_HZ, n=TOTAL_SAMPLES, fs=SAMPLE_RATE):
    """ÇOKLU karıştırma: her hedef için ayrı dar-bant gürültü üretip, o
    hedefin ortak LO'ya (lo_mhz) göre frekans farkı kadar kaydırıp hepsini
    tek bir kompozit taban bantta toplar. Pluto tek bir TX çıkışı olduğu için
    "aynı anda 3 farklı frekans" fiziksel olarak böyle mümkün -- LO'dan uzak
    hedefler SAMPLE_RATE'in yarısını aşarsa (Nyquist) baseband'e doğru
    şekilde yerleşemez, bu durum çağıran tarafta ayrıca uyarılır."""
    composite = np.zeros(n, dtype=np.complex128)
    t = np.arange(n)
    for f_mhz in freqs_mhz:
        offset_hz = (f_mhz - lo_mhz) * 1e6
        shift = np.exp(1j * 2 * np.pi * offset_hz * t / fs)
        burst = generate_barrage_noise(bw_hz_each, n=n, fs=fs)
        composite += burst.astype(np.complex128) * shift
    composite /= (np.max(np.abs(composite)) + 1e-9)
    return composite.astype(np.complex64)


def _load_wav_mono(path, target_fs):
    with wave.open(path, "rb") as wf:
        n_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        src_fs = wf.getframerate()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)
    if sampwidth != 2:
        raise ValueError(f"Sadece 16-bit PCM WAV destekleniyor ({path} sampwidth={sampwidth})")
    audio = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
    if n_channels > 1:
        audio = audio.reshape(-1, n_channels).mean(axis=1)
    if src_fs != target_fs:
        # Basit doğrusal enterpolasyonla yeniden örnekleme -- ses kalitesi
        # kritik değil (test/aldatma amaçlı), harici bağımlılık eklemeyelim.
        src_t = np.arange(len(audio)) / src_fs
        dst_n = int(len(audio) * target_fs / src_fs)
        dst_t = np.arange(dst_n) / target_fs
        audio = np.interp(dst_t, src_t, audio)
    return audio.astype(np.float32)


def _synthetic_alert_tone(duration_s, fs):
    """Kayıtlı ses yoksa yerine geçecek basit, tanınabilir bir uyarı tonu
    dizisi (iki frekans arasında geçiş yapan siren benzeri sinyal)."""
    t = np.arange(int(duration_s * fs)) / fs
    period = 1.0
    frac = (t % period) / period
    freq = np.where(frac < 0.5, 800.0, 1200.0)
    phase = 2 * np.pi * np.cumsum(freq) / fs
    return (0.5 * np.sin(phase)).astype(np.float32)


def load_aldatma_audio(fs, min_duration_s=2.0):
    """Eski/varsayılan ("OTOMATIK") davranış -- operatör özel bir kaynak
    seçmediyse: data/aldatma_sesleri/ altında ilk .wav varsa onu, yoksa
    sentetik uyarı tonunu kullanır."""
    if os.path.isdir(ALDATMA_SES_DIZINI):
        wavs = sorted(f for f in os.listdir(ALDATMA_SES_DIZINI) if f.lower().endswith(".wav"))
        if wavs:
            path = os.path.join(ALDATMA_SES_DIZINI, wavs[0])
            print(f"[*] Aldatma sesi yükleniyor: {path}")
            return _load_wav_mono(path, fs)
    print(f"[*] '{ALDATMA_SES_DIZINI}' altında .wav bulunamadı -- sentetik uyarı tonu kullanılacak "
          f"(gerçek bir mesaj için oraya 16-bit mono/stereo bir .wav koy).")
    return _synthetic_alert_tone(min_duration_s, fs)


def synthesize_piper_tts(text, target_fs):
    """Piper TTS ile (yerel/çevrimdışı, Türkçe model) metinden ses üretir.
    Başarısız olursa (model yok, piper kurulu değil, vb.) None döner --
    çağıran taraf sentetik tona düşmeli."""
    if not text or not text.strip():
        print("[-] Piper TTS için metin boş.")
        return None
    if not (os.path.exists(PIPER_MODEL) and os.path.exists(PIPER_CONFIG)):
        print(f"[-] Piper ses modeli bulunamadı ({PIPER_MODEL}) -- indirilmemiş olabilir.")
        return None

    out_path = os.path.join(tempfile.gettempdir(), f"ebabil_piper_{int(time.time() * 1000)}.wav")
    try:
        subprocess.run(
            [sys.executable, "-m", "piper", "-m", PIPER_MODEL, "-c", PIPER_CONFIG, "-f", out_path],
            input=text.encode("utf-8"), check=True, capture_output=True, timeout=30,
        )
        audio = _load_wav_mono(out_path, target_fs)
        print(f"[*] Piper TTS sesi üretildi: {text!r} ({len(audio)/target_fs:.1f}s)")
        return audio
    except Exception as e:
        print(f"[-] Piper TTS üretimi başarısız: {e}")
        return None
    finally:
        try:
            os.remove(out_path)
        except OSError:
            pass


def get_aldatma_audio(fs):
    """Operatörün seçtiği kaynağa göre (bkz. _aldatma_kaynak/set_aldatma_kaynak)
    aldatma sesini döndürür. Her durumda başarısızlıkta sentetik tona düşer --
    hiçbir seçim ses üretimini tamamen durdurmaz."""
    tur = _aldatma_kaynak["tur"]
    deger = _aldatma_kaynak["deger"]

    if tur == "PIPER_TTS" and deger:
        audio = synthesize_piper_tts(deger, fs)
        if audio is not None:
            return audio
        print("[!] Piper TTS başarısız oldu, sentetik uyarı tonuna düşülüyor.")
        return _synthetic_alert_tone(2.0, fs)

    if tur == "KAYIT_TEKRAR" and deger:
        path = os.path.join(ALDATMA_SES_DIZINI, deger)
        if os.path.exists(path):
            print(f"[*] Aldatma sesi (kayıt-tekrar) yükleniyor: {path}")
            return _load_wav_mono(path, fs)
        print(f"[!] {path} bulunamadı, sentetik uyarı tonuna düşülüyor.")
        return _synthetic_alert_tone(2.0, fs)

    return load_aldatma_audio(fs)


def generate_nbfm_deception(audio, fs=SAMPLE_RATE, deviation_hz=5000.0):
    """Ses -> dar bant FM taban bant. modulators.gen_wbfm ile aynı prensip,
    sadece sapma (deviation) çok daha dar -- analog telsiz bandına uygun."""
    if len(audio) == 0:
        audio = _synthetic_alert_tone(2.0, fs)
    phase = 2 * np.pi * deviation_hz * np.cumsum(audio) / fs
    sig = np.exp(1j * phase).astype(np.complex64)
    return sig


class PlutoTX:
    def __init__(self):
        self.pluto = None
        self.active_gorev = None
        self.active_freq_mhz = None
        self.gain_db = DEFAULT_TX_GAIN_DB
        self._arabakisli_stop_event = None
        self._arabakisli_thread = None

    def connect(self):
        if DRY_RUN:
            print("[*] EBABIL_ET_DRY_RUN=1 -- Pluto donanımına bağlanılmıyor, komutlar sadece loglanacak.")
            return
        import adi
        print("[*] PlutoSDR TX'e bağlanılıyor (ip:192.168.2.1)...")
        self.pluto = adi.Pluto("ip:192.168.2.1")
        self.pluto.sample_rate = SAMPLE_RATE
        self.pluto.tx_hardwaregain_chan0 = self.gain_db
        self.pluto.tx_cyclic_buffer = True
        print(f"[+] Pluto TX hazır (başlangıç kazancı: {self.gain_db} dB -- 0 dB tam güç, negatif=zayıflatma).")

    def set_gain(self, gain_db):
        self.gain_db = max(MIN_TX_GAIN_DB, min(MAX_TX_GAIN_DB, gain_db))
        print(f"[*] TX kazancı ayarlandı: {self.gain_db:.1f} dB")
        if self.pluto is not None:
            self.pluto.tx_hardwaregain_chan0 = self.gain_db

    def start(self, gorev_kodu, freq_mhz, waveform):
        freq_hz = freq_mhz * 1e6
        if not (PLUTO_TX_MIN_HZ <= freq_hz <= PLUTO_TX_MAX_HZ):
            print(f"[-] {freq_mhz} MHz, Pluto TX aralığının ({PLUTO_TX_MIN_HZ/1e6:.0f}-{PLUTO_TX_MAX_HZ/1e6:.0f} MHz) "
                  f"DIŞINDA -- bu script atlıyor.")
            return
        self.stop(silent=True)
        waveform_scaled = waveform / (np.max(np.abs(waveform)) + 1e-9) * 0.7 * (2 ** 14)
        print(f"[*] BAŞLATILIYOR: {gorev_kodu} @ {freq_mhz:.6f} MHz (kazanç {self.gain_db:.1f} dB)")
        if not DRY_RUN:
            self.pluto.tx_lo = int(freq_hz)
            self.pluto.tx_hardwaregain_chan0 = self.gain_db
            self.pluto.tx_destroy_buffer()
            self.pluto.tx(waveform_scaled)
        self.active_gorev = gorev_kodu
        self.active_freq_mhz = freq_mhz

    def stop(self, gorev_kodu=None, silent=False):
        if self.active_gorev is None:
            return
        if gorev_kodu is not None and gorev_kodu != self.active_gorev:
            return
        if not silent:
            print(f"[*] DURDURULUYOR: {self.active_gorev} @ {self.active_freq_mhz} MHz")
        if self._arabakisli_stop_event is not None:
            self._arabakisli_stop_event.set()
            if self._arabakisli_thread is not None:
                self._arabakisli_thread.join(timeout=5.0)
            self._arabakisli_stop_event = None
            self._arabakisli_thread = None
        if not DRY_RUN and self.pluto is not None:
            self.pluto.tx_destroy_buffer()
        self.active_gorev = None
        self.active_freq_mhz = None

    def start_arabakisli(self, gorev_kodu, freqs_mhz, bw_hz=KARISTIRMA_BW_HZ):
        if not freqs_mhz:
            return
        for f in freqs_mhz:
            freq_hz = f * 1e6
            if not (PLUTO_TX_MIN_HZ <= freq_hz <= PLUTO_TX_MAX_HZ):
                print(f"[-] {f} MHz, Pluto TX/RX aralığının ({PLUTO_TX_MIN_HZ/1e6:.0f}-{PLUTO_TX_MAX_HZ/1e6:.0f} MHz) "
                      f"DIŞINDA -- atlanıyor.")
                return
        self.stop(silent=True)
        freq_desc = ";".join(f"{f:.6f}" for f in freqs_mhz)
        print(f"[*] BAŞLATILIYOR (arabakışlı): {gorev_kodu} @ {freq_desc} MHz "
              f"(TX {ARABAKISLI_TX_BURST_S:.1f}s / RX-dinleme {ARABAKISLI_RX_WINDOW_S:.1f}s döngüsü)")
        self.active_gorev = gorev_kodu
        self.active_freq_mhz = freqs_mhz[0]  # durum/log amaçlı birincil hedef
        self._arabakisli_stop_event = threading.Event()
        self._arabakisli_thread = threading.Thread(
            target=self._arabakisli_loop, args=(freqs_mhz, bw_hz, self._arabakisli_stop_event), daemon=True,
        )
        self._arabakisli_thread.start()

    def _check_target_present(self, freq_hz):
        """Kısa bir RX penceresi alıp streamer.py'nin detect_peak'iyle aynı
        mantıkla (medyan gürültü tabanı + eşik) hedefin hâlâ orada olup
        olmadığına bakar. DRY_RUN'da donanım yok, her zaman 'var' varsayılır."""
        if DRY_RUN:
            return True
        self.pluto.tx_destroy_buffer()
        self.pluto.rx_lo = int(freq_hz)
        self.pluto.rx_buffer_size = ARABAKISLI_RX_NSAMPLES
        self.pluto.rx_destroy_buffer()
        samples = self.pluto.rx()
        if isinstance(samples, (list, tuple)):
            samples = samples[0]
        spectrum = np.fft.fftshift(np.fft.fft(samples * np.hanning(len(samples))))
        power_db = 20 * np.log10(np.abs(spectrum) + 1e-9)
        floor = float(np.median(power_db))
        peak = float(np.max(power_db))
        return (peak - floor) >= ARABAKISLI_THRESHOLD_DB

    def _arabakisli_loop(self, freqs_mhz, bw_hz, stop_event):
        lo_mhz = sum(freqs_mhz) / len(freqs_mhz)
        if len(freqs_mhz) > 1:
            span_hz = (max(freqs_mhz) - min(freqs_mhz)) * 1e6
            if span_hz > SAMPLE_RATE * 0.8:
                print(f"[!] Çoklu hedefler ({span_hz/1e3:.0f} kHz aralık), Pluto'nun anlık bant genişliğine "
                      f"({SAMPLE_RATE/1e3:.0f} kHz) sığmıyor olabilir -- en uzak hedeflerde etkinlik azalabilir.")
            waveform = generate_multi_target_noise(freqs_mhz, lo_mhz, bw_hz)
        else:
            waveform = generate_barrage_noise(bw_hz)
        waveform_scaled = waveform / (np.max(np.abs(waveform)) + 1e-9) * 0.7 * (2 ** 14)
        consecutive_misses = 0

        while not stop_event.is_set():
            if not DRY_RUN:
                self.pluto.tx_lo = int(lo_mhz * 1e6)
                self.pluto.tx_hardwaregain_chan0 = self.gain_db
                self.pluto.tx_destroy_buffer()
                self.pluto.tx(waveform_scaled)
            if stop_event.wait(ARABAKISLI_TX_BURST_S):
                break

            # Her hedefi ayrı ayrı dinle -- HERHANGİ biri hâlâ aktifse görev sürer
            # (çoklu hedefte hepsinin AYNI ANDA kaybolması beklenmez).
            any_present = any(self._check_target_present(f * 1e6) for f in freqs_mhz)
            if stop_event.wait(ARABAKISLI_RX_WINDOW_S):
                break

            if any_present:
                consecutive_misses = 0
            else:
                consecutive_misses += 1
                print(f"[!] Arabakışlı: {lo_mhz:.3f} MHz civarında hiçbir hedef görünmüyor "
                      f"({consecutive_misses}/{ARABAKISLI_MAX_MISSES})")
                if consecutive_misses >= ARABAKISLI_MAX_MISSES:
                    print(f"[!] Hedef(ler) muhtemelen sonlandı/taşındı -- arabakışlı görev kendiliğinden durduruluyor "
                          f"(ED modülüne dönülmeli, KTR 5.2).")
                    break

        if not DRY_RUN:
            self.pluto.tx_destroy_buffer()
        self.active_gorev = None
        self.active_freq_mhz = None
        print("[*] Arabakışlı görev sona erdi.")


def handle_baslat(tx, gorev_kodu, freqs_mhz, tip):
    if not freqs_mhz:
        print("[-] En az bir frekans gerekli.")
        return

    if gorev_kodu == "SUREKLI_KARISTIRMA":
        if tip == "COKLU" and len(freqs_mhz) > 1:
            lo_mhz = sum(freqs_mhz) / len(freqs_mhz)
            span_hz = (max(freqs_mhz) - min(freqs_mhz)) * 1e6
            if span_hz > SAMPLE_RATE * 0.8:
                print(f"[!] Çoklu hedefler ({span_hz/1e3:.0f} kHz aralık), Pluto'nun anlık bant genişliğine "
                      f"({SAMPLE_RATE/1e3:.0f} kHz) sığmıyor olabilir -- en uzak hedeflerde etkinlik azalabilir.")
            waveform = generate_multi_target_noise(freqs_mhz, lo_mhz, KARISTIRMA_BW_HZ)
            tx.start(gorev_kodu, lo_mhz, waveform)
        else:
            bw = BARAJ_BW_HZ if tip == "BARAJ" else KARISTIRMA_BW_HZ
            waveform = generate_barrage_noise(bw)
            tx.start(gorev_kodu, freqs_mhz[0], waveform)

    elif gorev_kodu == "ARA_BAKISLI_KARISTIRMA":
        bw = BARAJ_BW_HZ if tip == "BARAJ" else KARISTIRMA_BW_HZ
        tx.start_arabakisli(gorev_kodu, freqs_mhz, bw)

    elif gorev_kodu == "ANALOG_TELSIZ_ALDATMA":
        audio = get_aldatma_audio(SAMPLE_RATE)
        waveform = generate_nbfm_deception(audio)
        tx.start(gorev_kodu, freqs_mhz[0], waveform)

    else:
        print(f"[-] Bilinmeyen görev kodu: {gorev_kodu} -- yok sayılıyor.")


def main():
    tx = PlutoTX()
    tx.connect()

    # GUI ayrı bir makinedeyse (örn. bu betik Jetson'da, GUI Windows PC'de)
    # EBABIL_GUI_HOST'u GUI'nin LAN IP'sine ayarla -- bkz. streamer.py'deki
    # aynı isimli değişken.
    gui_host = os.environ.get("EBABIL_GUI_HOST", "127.0.0.1")

    context = zmq.Context()
    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect(f"tcp://{gui_host}:5557")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    print("[*] et_control hazır -- port 5557'de ET,BASLAT / ET,DURDUR / SET_POWER komutları dinleniyor.\n")

    try:
        while True:
            try:
                msg = sub_cmd.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.05)
                continue

            try:
                if msg.startswith("ET,BASLAT,"):
                    # ET,BASLAT,<görev_kodu>,<f1[;f2;f3]>,<tip> -- tip TEKLI/COKLU/BARAJ
                    parts = msg.split(",")
                    if len(parts) != 5:
                        print(f"[-] Beklenmeyen ET,BASLAT formatı: {msg}")
                        continue
                    _, _, gorev_kodu, freq_field, tip = parts
                    try:
                        freqs_mhz = [float(f) for f in freq_field.split(";") if f]
                    except ValueError:
                        print(f"[-] Geçersiz frekans alanı: {freq_field}")
                        continue
                    handle_baslat(tx, gorev_kodu, freqs_mhz, tip)

                elif msg.startswith("ET,DURDUR,"):
                    gorev_kodu = msg.split(",", 2)[2]
                    tx.stop(gorev_kodu)

                elif msg.startswith("ALDATMA_KAYNAK|"):
                    # ALDATMA_KAYNAK|<tur>|<deger> -- tur: KAYIT_TEKRAR (deger=dosya
                    # adı, data/aldatma_sesleri/ içinde) | PIPER_TTS (deger=metin).
                    # maxsplit=2 ile metnin içindeki olası "|" karakterleri korunur.
                    parts = msg.split("|", 2)
                    if len(parts) != 3:
                        print(f"[-] Geçersiz ALDATMA_KAYNAK komutu: {msg}")
                        continue
                    _, kaynak_tur, kaynak_deger = parts
                    set_aldatma_kaynak(kaynak_tur, kaynak_deger)

                elif msg.startswith("SET_POWER "):
                    try:
                        dbm = float(msg.split(" ", 1)[1])
                    except ValueError:
                        print(f"[-] Geçersiz SET_POWER değeri: {msg}")
                        continue
                    tx.set_gain(dbm)

                # SDR_VERISI_ISTEK, BANT_AYARLA|, FREKANS_KILITLE| -- bunlar streamer.py'nin
                # işi, burada görmezden geliniyor.

            except Exception as e:
                # Pluto'ya giden bir IIO/ağ komutu ara sıra bağlantı hatası
                # (ör. ConnectionResetError [Errno 10054]) verebiliyor --
                # streamer.py/pluto_ed_scanner.py'deki gibi burada da tek bir
                # hata tüm süreci öldürmesin, loglayıp devam edilsin. Sahada
                # defalarca gözlemlendi: bu olmadan operatör her seferinde
                # süreci elle yeniden başlatmak zorunda kalıyordu.
                print(f"[!] Komut işlenirken hata (devam ediliyor): {e}")
                time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        tx.stop(silent=True)
        print("\n[*] et_control kapatıldı.")


if __name__ == "__main__":
    main()
