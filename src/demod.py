"""
Gerçek AM/FM demodülasyonu -- KTR 4.3 (Sinyal İzleme/Dinleme): "Analog
yayınlarda ... AM/FM demodülasyon, filtreleme ve ses formatlama ile dinleme
veya kayıt çıktısı üretilebilir." Şimdiye kadar bu SADECE arayüz durumu
değiştiren bir düğmeydi (DİNLE), gerçek ses üretimi yoktu -- bu modül onu
sağlıyor.

Donanımdan/ZMQ'dan bağımsız, saf sinyal işleme -- streamer.py bunu I/Q
örnekleriyle çağırır.
"""
from fractions import Fraction

import numpy as np
from scipy.signal import resample_poly

AUDIO_SAMPLE_RATE = 48000


def _to_audio_rate(signal, fs_in, fs_audio=AUDIO_SAMPLE_RATE):
    """fs_in'den fs_audio'ya yeniden örnekler -- resample_poly zaten
    anti-alias alçak geçiren filtreyi uyguluyor, ayrı bir filtre tasarımına
    gerek yok."""
    ratio = Fraction(fs_audio, int(fs_in)).limit_denominator(1000)
    return resample_poly(signal, ratio.numerator, ratio.denominator)


def demod_wbfm(iq, fs_in, deviation_hz=75000.0, fs_audio=AUDIO_SAMPLE_RATE):
    """Geniş bant FM (yayın FM'i) -- anlık faz farkından (frekans ayrımcısı)
    ses çıkarır. deviation_hz, ses genliğini normalize etmek için kaba bir
    ölçek (standart yayın FM sapması ~75kHz)."""
    phase = np.unwrap(np.angle(iq))
    inst_freq_hz = np.diff(phase, prepend=phase[:1]) * fs_in / (2 * np.pi)
    audio = inst_freq_hz / deviation_hz
    audio = _to_audio_rate(audio, fs_in, fs_audio)
    return np.clip(audio, -1.0, 1.0).astype(np.float32)


def demod_am(iq, fs_in, fs_audio=AUDIO_SAMPLE_RATE):
    """AM-DSB (ve kaba bir yaklaşımla AM-SSB) -- zarf (envelope) algılama.
    Gerçek SSB için normalde bir BFO/ürün detektörü gerekir; zarf algılama
    SSB'de bozulmalı ama anlaşılabilir bir sonuç verir -- ayrı bir SSB
    demodülatörü şimdilik kapsam dışı."""
    envelope = np.abs(iq)
    envelope = envelope - np.mean(envelope)
    peak = np.max(np.abs(envelope)) + 1e-9
    audio = envelope / peak
    audio = _to_audio_rate(audio, fs_in, fs_audio)
    return np.clip(audio, -1.0, 1.0).astype(np.float32)


def demod_for_modulation(iq, fs_in, modulasyon_turu, fs_audio=AUDIO_SAMPLE_RATE):
    """modulasyon_turu (predict.CLASSES'ten biri veya None/"Belirsiz") göre
    doğru demodülatörü seçer. Sayısal veya bilinmeyen sınıflar için de bir
    çıktı üretir (zarf algılama) -- konuşma/anlamlı ses beklenmez ama en
    azından enerji/etkinlik dinlenebilir."""
    if modulasyon_turu == "WBFM":
        return demod_wbfm(iq, fs_in, fs_audio=fs_audio)
    return demod_am(iq, fs_in, fs_audio=fs_audio)
