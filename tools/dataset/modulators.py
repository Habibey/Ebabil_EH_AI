"""
PlutoSDR TX için bilinen modülasyon türlerinde sentetik temel bant (baseband)
dalga formu üreten fonksiyonlar. Amaç: RadioML tarzı "temiz" sentetik veri
değil, gerçek bir TX/RX RF zincirinden (DAC, anten, hava, RTL-SDR ADC'si)
geçmiş, GERÇEK donanım kusurlarını taşıyan örnekler üretmek.

Sayısal sınıflar (BPSK/QPSK/8PSK/QAM16/QAM64/PAM4) artık kök-yükseltilmiş
kosinüs (RRC) darbe şekillendirmesi kullanıyor -- eskiden dikdörtgen darbe
(her sembolü sps kez tekrarla) vardı, bu da RadioML'in eğitim verisindeki
sinyallerden çok daha geniş spektral yan lobler üretiyordu. Model gerçek
kayıtlarda sayısal sınıfları neredeyse hiç tanımayıp analog sınıflara
("AM-SSB"/"WBFM") kaçıyordu -- bunun bir nedeni muhtemelen buydu: modelin
RadioML'de öğrendiği spektral "şekil" ile bizim ürettiğimiz gerçek RF'nin
şekli uyuşmuyordu. RRC bunu düzeltmeye çalışıyor (kesin garanti değil --
tek kayıt oturumundan gelen düşük çeşitlilik de ayrı bir sınırlama).
"""
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import upfirdn

PLUTO_TX_SAMPLE_RATE = 1_000_000  # 1 MSPS
SAMPLES_PER_SYMBOL = 20  # -> 50 ksym/s sayısal simge hızı
N_SYMBOLS = 3000
TOTAL_SAMPLES = N_SYMBOLS * SAMPLES_PER_SYMBOL  # 60_000 örnek (~60ms, cyclic TX için)

RRC_ROLLOFF = 0.35  # tipik pratik değer (0=brick-wall, 1=en yumuşak/en geniş bant)
RRC_SPAN_SYMBOLS = 8  # filtre uzunluğu, sembol cinsinden


def _upsample(symbols, sps=SAMPLES_PER_SYMBOL):
    """Dikdörtgen darbe -- artık sadece CPFSK/GFSK gibi zaten faz-sürekli
    (dolayısıyla doğal olarak yumuşak) sınıflar için kullanılıyor."""
    return np.repeat(symbols, sps)


def _rrc_taps(beta=RRC_ROLLOFF, sps=SAMPLES_PER_SYMBOL, span_symbols=RRC_SPAN_SYMBOLS):
    n_taps = span_symbols * sps + 1
    t = (np.arange(n_taps) - n_taps // 2) / sps  # sembol periyodu cinsinden, ortalanmış
    taps = np.zeros(n_taps)
    for i, ti in enumerate(t):
        if np.isclose(ti, 0.0):
            taps[i] = 1.0 - beta + 4 * beta / np.pi
        elif beta != 0 and np.isclose(abs(ti), 1.0 / (4 * beta)):
            taps[i] = (beta / np.sqrt(2)) * (
                (1 + 2 / np.pi) * np.sin(np.pi / (4 * beta)) + (1 - 2 / np.pi) * np.cos(np.pi / (4 * beta))
            )
        else:
            num = np.sin(np.pi * ti * (1 - beta)) + 4 * beta * ti * np.cos(np.pi * ti * (1 + beta))
            den = np.pi * ti * (1 - (4 * beta * ti) ** 2)
            taps[i] = num / den
    return taps / np.sqrt(np.sum(taps ** 2))  # birim enerjiye normalize


_RRC_TAPS = _rrc_taps()


def _upsample_shaped(symbols, sps=SAMPLES_PER_SYMBOL):
    """RRC darbe şekillendirmeli yükseltme -- sıfır-doldurma + FIR filtre.
    Çıktı tam TOTAL_SAMPLES uzunluğunda olacak şekilde filtrenin başlangıç
    geçişi (group delay) atlanarak ortadan kesilir."""
    shaped = upfirdn(_RRC_TAPS, symbols, up=sps)
    start = len(_RRC_TAPS) // 2
    return shaped[start:start + TOTAL_SAMPLES].astype(np.complex64)


def gen_bpsk():
    bits = np.random.randint(0, 2, N_SYMBOLS)
    symbols = np.where(bits == 0, -1.0, 1.0).astype(np.complex64)
    return _upsample_shaped(symbols)


def gen_qpsk():
    idx = np.random.randint(0, 4, N_SYMBOLS)
    consts = (np.array([1 + 1j, 1 - 1j, -1 + 1j, -1 - 1j]) / np.sqrt(2)).astype(np.complex64)
    return _upsample_shaped(consts[idx])


def gen_8psk():
    idx = np.random.randint(0, 8, N_SYMBOLS)
    consts = np.exp(1j * 2 * np.pi * np.arange(8) / 8).astype(np.complex64)
    return _upsample_shaped(consts[idx])


def gen_qam16():
    levels = np.array([-3, -1, 1, 3])
    i = np.random.randint(0, 4, N_SYMBOLS)
    q = np.random.randint(0, 4, N_SYMBOLS)
    symbols = ((levels[i] + 1j * levels[q]) / (3 * np.sqrt(2))).astype(np.complex64)
    return _upsample_shaped(symbols)


def gen_qam64():
    levels = np.array([-7, -5, -3, -1, 1, 3, 5, 7])
    i = np.random.randint(0, 8, N_SYMBOLS)
    q = np.random.randint(0, 8, N_SYMBOLS)
    symbols = ((levels[i] + 1j * levels[q]) / (7 * np.sqrt(2))).astype(np.complex64)
    return _upsample_shaped(symbols)


def gen_pam4():
    levels = np.array([-3, -1, 1, 3]) / 3.0
    idx = np.random.randint(0, 4, N_SYMBOLS)
    symbols = levels[idx].astype(np.complex64)  # Q=0 (gerçek değerli)
    return _upsample_shaped(symbols)


def gen_cpfsk(mod_index=0.5):
    bits = np.random.randint(0, 2, N_SYMBOLS)
    freq = np.where(bits == 0, -1.0, 1.0).astype(np.float64)
    freq_upsampled = _upsample(freq).astype(np.float64)
    phase = np.cumsum(freq_upsampled) * (np.pi * mod_index / SAMPLES_PER_SYMBOL)
    return np.exp(1j * phase).astype(np.complex64)


def gen_gfsk(mod_index=0.5, bt=0.3):
    bits = np.random.randint(0, 2, N_SYMBOLS)
    freq = np.where(bits == 0, -1.0, 1.0).astype(np.float64)
    freq_upsampled = _upsample(freq).astype(np.float64)
    # CPFSK'den farkı: frekans darbesi Gauss filtreyle yumuşatılıyor (BT çarpımı yaklaşık).
    freq_smoothed = gaussian_filter1d(freq_upsampled, sigma=SAMPLES_PER_SYMBOL * bt)
    phase = np.cumsum(freq_smoothed) * (np.pi * mod_index / SAMPLES_PER_SYMBOL)
    return np.exp(1j * phase).astype(np.complex64)


def _seamless_audio_freq(target_hz, fs=PLUTO_TX_SAMPLE_RATE, n=TOTAL_SAMPLES):
    """Döngüsel TX buffer'ında sıçrama olmaması için ses frekansını, buffer
    süresine tam sayıda periyot sığacak şekilde hedefe en yakın değere yuvarlar."""
    buffer_s = n / fs
    cycles = max(1, round(target_hz * buffer_s))
    return cycles / buffer_s


def gen_am_dsb(audio_freq_hz=1000):
    fs = PLUTO_TX_SAMPLE_RATE
    f = _seamless_audio_freq(audio_freq_hz, fs)
    t = np.arange(TOTAL_SAMPLES) / fs
    message = 0.6 * np.sin(2 * np.pi * f * t)
    signal = (1.0 + message).astype(np.complex64)  # Q=0, gerçek değerli DSB-AM
    return signal / np.max(np.abs(signal))


def gen_am_ssb(audio_freq_hz=1000):
    fs = PLUTO_TX_SAMPLE_RATE
    f = _seamless_audio_freq(audio_freq_hz, fs)
    t = np.arange(TOTAL_SAMPLES) / fs
    # Tek bir ton için analitik sinyal = tek yönlü kompleks üstel -- doğal SSB.
    return np.exp(1j * 2 * np.pi * f * t).astype(np.complex64)


def gen_wbfm(audio_freq_hz=1000, deviation_hz=60000):
    fs = PLUTO_TX_SAMPLE_RATE
    f = _seamless_audio_freq(audio_freq_hz, fs)
    t = np.arange(TOTAL_SAMPLES) / fs
    message = np.sin(2 * np.pi * f * t)
    phase = 2 * np.pi * deviation_hz * np.cumsum(message) / fs
    return np.exp(1j * phase).astype(np.complex64)


GENERATORS = {
    "BPSK": gen_bpsk,
    "QPSK": gen_qpsk,
    "8PSK": gen_8psk,
    "QAM16": gen_qam16,
    "QAM64": gen_qam64,
    "PAM4": gen_pam4,
    "CPFSK": gen_cpfsk,
    "GFSK": gen_gfsk,
    "AM-DSB": gen_am_dsb,
    "AM-SSB": gen_am_ssb,
    "WBFM": gen_wbfm,
}
