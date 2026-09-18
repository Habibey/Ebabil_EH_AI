"""
streamer.py (RTL-SDR) ve pluto_ed_scanner.py (PlutoSDR RX) arasında paylaşılan,
DONANIMDAN BAĞIMSIZ sinyal işleme kodu -- FFT/binleme, OS-CFAR tespiti ve hedef
takibi. Bilerek RTL-SDR/TensorFlow importu YOK burada (predict.py/streamer.py
gibi ağır bağımlılıklar), sadece numpy -- iki ayrı süreç de hafif kalabilsin.
"""
import time

import numpy as np

# predict.classical_analog_sayisal ile BİREBİR AYNI matematik -- KASITLI olarak
# buraya kopyalandı, predict.py'yi import ETMEDİK çünkü o dosya (koşullu da
# olsa) tensorflow import ediyor -- AI'nin Jetson'a taşınmasının asıl sebebi
# TensorFlow'u RPi'nin sürecine hiç yüklememekti, predict.py'yi import etmek
# bunu geri getirirdi. Modelden bağımsız, saf numpy -- hiçbir öğrenilmiş
# parametre kullanmaz, sadece otokorelasyon/periyodiklik.
CLASSICAL_PERIODICITY_THRESHOLD = 0.14


def classical_analog_sayisal(I, Q):
    """Ham I/Q'dan (mümkünse geniş bir pencere -- 128 örnek periyodiklik
    ölçümü için kısa kalabilir) anlık frekansın otokorelasyonuna bakarak
    periyodiklik skoru çıkarır, kaba bir Analog/Sayısal tahmini döndürür."""
    phase = np.unwrap(np.arctan2(Q, I))
    inst_freq = np.diff(phase, prepend=phase[:1])
    f = inst_freq - np.mean(inst_freq)
    ac = np.correlate(f, f, mode="full")
    ac = ac[len(ac) // 2:]
    ac = ac / (ac[0] + 1e-12)
    search = ac[2:len(ac) // 2]
    periodicity = float(np.max(search)) if len(search) else 0.0
    tur = "Analog" if periodicity >= CLASSICAL_PERIODICITY_THRESHOLD else "Sayısal"
    return tur, periodicity

try:
    from numpy.lib.stride_tricks import sliding_window_view
except ImportError:
    # numpy < 1.20 (ör. Jetson'daki Python 3.6 için erişilebilir en yeni
    # numpy 1.19.x) bu fonksiyonu içermiyor -- as_strided ile aynısını
    # elle kuruyoruz, sliding_window_view zaten içeride bunu yapıyor.
    def sliding_window_view(arr, window_shape):
        window = window_shape[0] if isinstance(window_shape, tuple) else window_shape
        shape = (arr.shape[0] - window + 1, window)
        strides = (arr.strides[0], arr.strides[0])
        return np.lib.stride_tricks.as_strided(arr, shape=shape, strides=strides)

FFT_SIZE = 4096
FREQ_BINS = 64  # arayüzün SPEC formatıyla eşleşmeli (mainwindow.h FREQ_BINS)

# OS-CFAR parametreleri -- FREQ_BINS=64 ölçeğine göre ayarlı. GUARD_CELLS,
# hedef sinyalin kendisinin gürültü tahminine sızmasını önler; REFERENCE_CELLS
# gürültü tabanının hesaplandığı komşu hücre sayısıdır. PERCENTILE, CA-CFAR'daki
# ortalama yerine sıralı istatistik kullanır (KTR Tablo 7: "çoklu yayın ve
# değişken gürültü koşullarına karşı daha dayanıklı" -- civarda başka güçlü bir
# sinyal olsa bile eşiği şişirmez).
CFAR_GUARD_CELLS = 2
CFAR_REFERENCE_CELLS = 8
CFAR_PERCENTILE = 75.0
CFAR_THRESHOLD_OFFSET_DB = 10.0  # eski sabit-eşik sürümüyle aynı varsayılan değer

# pick_target() histerezisi: birden fazla güçlü hedef aynı anda varken (ör.
# yarışma alanında başka takımların yayınları) "en güçlü" tur tur küçük gürültü
# farklarıyla el değiştirebiliyor -- her el değiştirme ARAMA<->İZLEME arası
# gerçek bir donanım retune'u (rtlsdr_set_sample_rate/PlutoRX örnekleme hızı
# değişimi) demek, ve bunun ÇOK sık tekrarlanması RTL-SDR/libusb'de gerçek bir
# segfault'a yol açtığı gözlemlendi (bkz. CLAUDE.md). Bu yüzden mevcut kilitli
# hedeften yeni aday bu kadar dB daha güçlü değilse hedef DEĞİŞTİRİLMEZ.
TARGET_LOCK_HYSTERESIS_DB = 3.0


def compute_power_spectrum(samples, center_mhz, sample_rate, fft_size=FFT_SIZE, freq_bins=FREQ_BINS):
    """Ham I/Q örneklerinden (zaten center_mhz'e ayarlanmış SDR'dan okunmuş)
    FREQ_BINS'e indirgenmiş güç spektrumunu (dB), her bin'in gerçek frekansını
    (MHz) ve kullanılan örnekleme hızını (MHz) döndürür."""
    windowed = samples[:fft_size] * np.hanning(min(len(samples), fft_size))
    spectrum = np.fft.fftshift(np.fft.fft(windowed, n=fft_size))
    power_db = 20 * np.log10(np.abs(spectrum) + 1e-9)

    group = fft_size // freq_bins
    binned_db = power_db[: group * freq_bins].reshape(freq_bins, group).mean(axis=1)

    fs_mhz = sample_rate / 1e6
    start_mhz = center_mhz - fs_mhz / 2
    bin_freqs_mhz = start_mhz + (np.arange(freq_bins) + 0.5) / freq_bins * fs_mhz

    return binned_db, bin_freqs_mhz, fs_mhz


def os_cfar_detect(power_db, guard_cells=CFAR_GUARD_CELLS, reference_cells=CFAR_REFERENCE_CELLS,
                    percentile=CFAR_PERCENTILE, threshold_offset_db=CFAR_THRESHOLD_OFFSET_DB):
    """Ordered-Statistics CFAR: her hücre için etrafındaki referans hücrelerin
    (guard hücreleri hariç) verilen persentilini gürültü tabanı olarak alır,
    bunun threshold_offset_db üstünü eşik kabul eder. Kenarlara yakın hücreler
    için pencere 'edge' modunda doldurularak simetrik tutulur.
    Döndürür: (detected_mask, noise_floor, threshold) -- hepsi power_db ile aynı boyda."""
    n = len(power_db)
    half_window = guard_cells + reference_cells
    padded = np.pad(power_db, half_window, mode="edge")
    windows = sliding_window_view(padded, 2 * half_window + 1)  # (n, 2*half_window+1)

    mid = half_window
    ref_mask = np.ones(2 * half_window + 1, dtype=bool)
    ref_mask[mid - guard_cells: mid + guard_cells + 1] = False
    ref_windows = windows[:, ref_mask]  # (n, 2*reference_cells)

    noise_floor = np.percentile(ref_windows, percentile, axis=1)
    threshold = noise_floor + threshold_offset_db
    detected = power_db > threshold
    return detected, noise_floor, threshold


DC_EXCLUDE_BINS = 1  # merkez bin'in her iki yanında da hariç tutulacak bin sayısı

# Bilinen sabit "birdie" (donanım kaynaklı, gerçek sinyal OLMAYAN spur) --
# gqrx ile aynı bantta gerçekten yayın OLMADIĞI doğrulanmış haldeyken bu RTL-SDR
# biriminde tekrar tekrar 147.6-147.9 MHz civarında "hedef" görüldü (farklı
# tarama adımlarında FARKLI merkeze göre farklı ofsetlerde çıktı -- yani
# DC_EXCLUDE_BINS'in yakaladığı "merkezde LO sızıntısı" ile AYNI ŞEY DEĞİL,
# hangi adımın penceresine denk gelirse o adımda görünen SABİT MUTLAK
# frekanslı bir iç spur/birdie, muhtemelen RTL2832U/kristal harmoniği).
# Bu yüzden bin-pozisyonuna göre değil, MUTLAK frekansa göre dar bir aralık
# dışlanıyor -- pencere kenarlarını genel olarak dışlamak (denendi, geri
# alındı) tarama adımları bitişik/örtüşmesiz olduğu için taranan bandın
# ~%24'ünü kalıcı kör nokta yapıyordu, bu çok daha riskli bir taviz.
# NOT: bu aralık BU SPESİFİK RTL-SDR birimine özel olabilir -- donanım
# değişirse (farklı bir RTL-SDR takılırsa) geçerliliğini yitirebilir, o
# zaman yeniden (antensiz/gqrx ile çapraz doğrulanarak) belirlenmeli.
KNOWN_BIRDIE_RANGES_MHZ = [(147.5, 148.0)]
# 2026-09-16 sabah saha testinde 147.955 MHz'de (önceki 147.55-147.95
# sınırının hemen 5 kHz DIŞINDA, Bant Gen. tam 1 bin genişliği -- yine aynı
# artefakt) tekrar görüldü, aralık 500kHz'e genişletildi.

def detect_peak(binned_db, bin_freqs_mhz):
    """OS-CFAR ile tespit yapar, en güçlü tespit edilen hücreyi ve onu içeren
    bitişik tespit bölgesinin genişliğini döndürür: (frekans_mhz, güç_db,
    bant_genişliği_khz, gürültü_tabanı_db). Tespit yoksa None.
    Gürültü tabanı, CFAR'ın zaten hesapladığı ama eskiden dışarı hiç
    aktarılmayan değer -- SNR ve KTR Tablo 8'deki "Gürültü Tabanı" parametresi
    için gerekli (bkz. streamer.py'deki SNR hesaplaması).

    Merkez bin (DC) HARİÇ TUTULUYOR -- RTL-SDR/zero-IF SDR'ların donanımsal
    bir kusuru: ayarlandığı merkez frekansta LO sızıntısı/DC bileşeni gerçek
    sinyal yokken bile görünür, anten hiç bağlı olmasa dahi OS-CFAR bunu
    gerçek bir hedef sanabiliyor (sahada anten takılı olmayan bir bantta
    "hedef" görülmesiyle keşfedildi, bkz. proje notları). Ayrıca bilinen sabit
    birdie aralıkları (bkz. KNOWN_BIRDIE_RANGES_MHZ) MUTLAK frekansa göre
    hariç tutuluyor."""
    detected, noise_floor, threshold = os_cfar_detect(binned_db)
    dc_bin = len(binned_db) // 2
    lo = max(0, dc_bin - DC_EXCLUDE_BINS)
    hi = min(len(detected), dc_bin + DC_EXCLUDE_BINS + 1)
    detected[lo:hi] = False
    for birdie_lo, birdie_hi in KNOWN_BIRDIE_RANGES_MHZ:
        detected &= ~((bin_freqs_mhz >= birdie_lo) & (bin_freqs_mhz <= birdie_hi))
    if not detected.any():
        return None

    masked = np.where(detected, binned_db, -np.inf)
    peak_idx = int(np.argmax(masked))
    peak_power = float(binned_db[peak_idx])
    peak_noise_floor = float(noise_floor[peak_idx])

    left = peak_idx
    while left > 0 and detected[left - 1]:
        left -= 1
    right = peak_idx
    while right < len(detected) - 1 and detected[right + 1]:
        right += 1

    bin_width_khz = float(bin_freqs_mhz[1] - bin_freqs_mhz[0]) * 1000.0
    bandwidth_khz = (right - left + 1) * bin_width_khz

    return float(bin_freqs_mhz[peak_idx]), peak_power, bandwidth_khz, peak_noise_floor


# Sinyal Süreklilik Durumu (KTR Tablo 8) sınıflandırma eşikleri.
SUREKLILIK_KAYIP_ESIK_S = 8.0  # bu süredir görülmediyse "Kaybolmuş" -- GUI'nin HEDEF_PASIF_ESIK_MS'iyle tutarlı
SUREKLILIK_SUREKLI_GAP_ESIK_S = 1.5  # ardışık tespitler arası ortalama bu kadar kısaysa "Sürekli"
SUREKLILIK_MIN_GUNCELLEME = 3  # bu kadar tekrar tespit edilmeden "Sürekli" denemez


class TargetTracker:
    """Tespit edilen frekansları kalıcı HEDEF-N kimliklerine eşler -- aynı
    frekansta tekrar tespit AYNI id'yi kullanır (yeni kart açılmaz)."""

    def __init__(self, match_tolerance_mhz=0.5, id_prefix="HEDEF"):
        self.known = {}  # id -> {"freq_mhz", "power_db", "bandwidth_khz", "snr_db",
                          #        "freq_sapmasi_mhz", "last_seen", "first_seen",
                          #        "update_count", "gaps"}
        self._next_id = 1
        self.match_tolerance_mhz = match_tolerance_mhz
        self.id_prefix = id_prefix

    def update(self, freq_mhz, power_db, bandwidth_khz, snr_db=None, freq_sapmasi_mhz=None):
        now = time.time()
        for tid, info in self.known.items():
            if abs(info["freq_mhz"] - freq_mhz) <= self.match_tolerance_mhz:
                gap = now - info["last_seen"]
                gaps = info.setdefault("gaps", [])
                gaps.append(gap)
                if len(gaps) > 5:
                    gaps.pop(0)
                info.update(
                    freq_mhz=freq_mhz, power_db=power_db, bandwidth_khz=bandwidth_khz,
                    snr_db=snr_db, freq_sapmasi_mhz=freq_sapmasi_mhz,
                    last_seen=now, update_count=info.get("update_count", 0) + 1,
                )
                return tid

        tid = f"{self.id_prefix}-{self._next_id}"
        self._next_id += 1
        self.known[tid] = {
            "freq_mhz": freq_mhz, "power_db": power_db, "bandwidth_khz": bandwidth_khz,
            "snr_db": snr_db, "freq_sapmasi_mhz": freq_sapmasi_mhz,
            "last_seen": now, "first_seen": now, "update_count": 1, "gaps": [],
        }
        return tid

    def most_recent(self):
        if not self.known:
            return None
        return max(self.known, key=lambda tid: self.known[tid]["last_seen"])

    def most_powerful(self, max_age_s=None):
        """Son max_age_s içinde görülenler arasında güç (power_db) en yüksek
        olanı döndürür -- most_recent()'ın aksine "en son görülen" değil
        "en baskın sinyal" seçer. Otomatik hedef kilitleme (kimse manuel
        seçim yapmadıysa) için: zayıf/aralıklı gürültü kırıntıları arasında
        gerçek/güçlü hedefi önceliklendirir (bkz. sahada gözlemlenen sorun --
        342 tespitlik güçlü bir hedef varken sistem 3-5 tespitlik gürültü
        kırıntılarına da eşit öncelik veriyordu)."""
        if not self.known:
            return None
        if max_age_s is None:
            max_age_s = SUREKLILIK_KAYIP_ESIK_S
        now = time.time()
        adaylar = {tid: info for tid, info in self.known.items() if now - info["last_seen"] <= max_age_s}
        if not adaylar:
            return None
        return max(adaylar, key=lambda tid: adaylar[tid]["power_db"])

    def sureklilik_durumu(self, tid):
        """KTR Tablo 8 "Sinyal Süreklilik Durumu" -- sürekli/aralıklı/kaybolmuş
        (paketli, tek taramadan ayırt edilemediği için ayrı bir kategori değil,
        bkz. sınırlama notu modülün başında). Zamanlama tabanlı kaba bir
        sezgisel -- gerçek bir burst/zamanlama analizinden farklı."""
        info = self.known.get(tid)
        if info is None:
            return "Bilinmiyor"
        now = time.time()
        if now - info["last_seen"] > SUREKLILIK_KAYIP_ESIK_S:
            return "Kaybolmuş"
        gaps = info.get("gaps", [])
        if info.get("update_count", 0) >= SUREKLILIK_MIN_GUNCELLEME and gaps \
                and (sum(gaps) / len(gaps)) < SUREKLILIK_SUREKLI_GAP_ESIK_S:
            return "Sürekli"
        return "Aralıklı"


def pick_target(tracker, selected_id, current_id=None, hysteresis_db=TARGET_LOCK_HYSTERESIS_DB):
    """Operatör belirli bir hedef seçtiyse (ve o hedef hâlâ tracker'da
    biliniyorsa) onu döndürür; aksi halde EN GÜÇLÜ hedefe düşer -- "en son
    görülen" değil, çünkü zayıf/aralıklı gürültü kırıntıları da arada bir
    görülüp most_recent()'ı ele geçirebiliyordu (sahada gözlemlendi: gerçek
    bir hedef 20sn'de 337 kez, gürültü kırıntıları sadece 3-5 kez tespit
    edilirken otomatik mod aralarında gidip geliyordu).

    current_id: operatör seçimi YOKSA şu an kilitli olunan hedefin id'si
    (çağıran taraf bir önceki pick_target() sonucunu geçirir). Yeni en güçlü
    aday, current_id'den hysteresis_db kadar daha güçlü DEĞİLSE mevcut
    hedefte kalınır -- birden fazla güçlü hedef güç bakımından birbirine
    yakınken (ör. yarışma alanında başka takımların yayınları) her turda
    farklı birine geçip gereksiz sık ARAMA<->İZLEME donanım retune'u
    tetiklenmesin diye (bkz. TARGET_LOCK_HYSTERESIS_DB).

    streamer.py (RTL-SDR) ve pluto_ed_scanner.py (PlutoSDR) ile ORTAK
    kullanılan hedef seçim mantığı -- ikisi de aynı davranışı istediği için
    burada tek yerde: davranış değişirse iki dosyada ayrı ayrı değil,
    sadece burada değişir."""
    if selected_id is not None and selected_id in tracker.known:
        return selected_id

    best_id = tracker.most_powerful()
    if best_id is None:
        return None
    if current_id is not None and current_id in tracker.known and current_id != best_id:
        current_power = tracker.known[current_id]["power_db"]
        best_power = tracker.known[best_id]["power_db"]
        if best_power - current_power < hysteresis_db:
            return current_id
    return best_id


def build_scan_freqs(start_mhz, stop_mhz, step_mhz):
    freqs = []
    f = start_mhz
    while f <= stop_mhz:
        freqs.append(f)
        f += step_mhz
    return freqs


class TunedSdr:
    """Bir SDR donanımını (yalnızca .center_freq, .sample_rate, .read_samples(n)
    ve .close() gerekiyor -- pyrtlsdr'ın arayüzüyle uyumlu her şey) sarmalar;
    center_freq/sample_rate GERÇEKTEN değişmediyse retune ve throwaway-örnek
    okumayı atlar.

    Neden gerekli: DİNLE ve periyodik AI sınıflandırması gibi aynı frekansta
    arka arkaya çok sayıda kısa yakalama yapılan yerlerde, her çağrıda
    (değişmese bile) retune etmek donanımsal PLL kilitlenme gecikmesi
    ekliyor -- üretim (RF yakalama) tüketimden (ses çalma hızından) geride
    kalıp kesik/cızırtılı sese yol açıyordu. Aynı prensip
    pluto_ed_scanner.py'deki PlutoRX._tune() ile ortak.

    Retune KARARI tek yerde (burada) toplandığı için, bunu çağıran her yer
    (arama/izleme/dinleme/sınıflandırma/kayıt) sadece "şu frekansta/hızda
    olmak istiyorum" der -- ne zaman gerçekten donanıma dokunulacağını bilmek
    zorunda değil. İleride bu davranış değişirse (ör. throwaway sayısı) tek
    bir yeri güncellemek yeterli, her çağıran yeri tek tek değiştirmeye
    gerek kalmaz."""

    def __init__(self, device, throwaway_samples):
        self._device = device
        self._throwaway_samples = throwaway_samples
        self._last_freq_hz = None
        self._last_rate = None

    def tune(self, center_mhz, sample_rate):
        """center_mhz'e (MHz) ve sample_rate'e (Hz) kilitlenir -- ikisi de
        son çağrıyla AYNIYSA donanıma hiç dokunmaz."""
        freq_hz = int(round(center_mhz * 1e6))
        rate = int(sample_rate)
        if freq_hz == self._last_freq_hz and rate == self._last_rate:
            return
        if rate != self._last_rate:
            self._device.sample_rate = rate
        self._device.center_freq = freq_hz
        self._device.read_samples(self._throwaway_samples)
        self._last_freq_hz = freq_hz
        self._last_rate = rate

    def invalidate(self):
        """Önbelleği temizler -- bir sonraki tune() çağrısı, önceki
        değerlerle AYNI istense bile donanımı zorla yeniden ayarlar. Bir
        USB/IIO hatasından sonra donanımın gerçek durumu şüpheliyken kullan."""
        self._last_freq_hz = None
        self._last_rate = None

    def read_samples(self, n_samples):
        return self._device.read_samples(n_samples)

    def close(self):
        self._device.close()

    def __getattr__(self, name):
        # Burada tanımlanmayan her şey (ör. .gain okuma) doğrudan cihaza gider.
        return getattr(self._device, name)

    def __setattr__(self, name, value):
        # Sarmalayıcının kendi durumu (_device, _last_freq_hz, ...) hariç her
        # atama (ör. sdr.gain = "auto") doğrudan cihaza gider.
        if name.startswith("_"):
            object.__setattr__(self, name, value)
        else:
            setattr(self._device, name, value)
