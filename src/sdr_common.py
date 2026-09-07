"""
streamer.py (RTL-SDR) ve pluto_ed_scanner.py (PlutoSDR RX) arasında paylaşılan,
DONANIMDAN BAĞIMSIZ sinyal işleme kodu -- FFT/binleme, OS-CFAR tespiti ve hedef
takibi. Bilerek RTL-SDR/TensorFlow importu YOK burada (predict.py/streamer.py
gibi ağır bağımlılıklar), sadece numpy -- iki ayrı süreç de hafif kalabilsin.
"""
import time

import numpy as np

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


def detect_peak(binned_db, bin_freqs_mhz):
    """OS-CFAR ile tespit yapar, en güçlü tespit edilen hücreyi ve onu içeren
    bitişik tespit bölgesinin genişliğini döndürür: (frekans_mhz, güç_db,
    bant_genişliği_khz, gürültü_tabanı_db). Tespit yoksa None.
    Gürültü tabanı, CFAR'ın zaten hesapladığı ama eskiden dışarı hiç
    aktarılmayan değer -- SNR ve KTR Tablo 8'deki "Gürültü Tabanı" parametresi
    için gerekli (bkz. streamer.py'deki SNR hesaplaması)."""
    detected, noise_floor, threshold = os_cfar_detect(binned_db)
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

    def __init__(self, match_tolerance_mhz=0.15, id_prefix="HEDEF"):
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


def build_scan_freqs(start_mhz, stop_mhz, step_mhz):
    freqs = []
    f = start_mhz
    while f <= stop_mhz:
        freqs.append(f)
        f += step_mhz
    return freqs
