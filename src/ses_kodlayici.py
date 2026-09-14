"""
DİNLE (sinyal dinleme) sesini Codec2 ile sıkıştırıp 915MHz radyo hattından
gönderilebilir hale getiren modül.

Neden Codec2: 915MHz hattının bant genişliği (~57.6kbps, bkz. proje notları/
CLAUDE.md'deki bant genişliği hesabı) ham ya da hafif sıkıştırılmış ses için
bile yetersiz. Codec2'nin 3200bps modu -- amatör telsiz dijital ses için
özel tasarlanmış, çok düşük bit hızlı bir codec -- bu bütçeye rahatça sığıyor
(diğer trafikle -- SYS/SPEC/DF/IQ -- paylaşılsa bile).

pycodec2 kurulu değilse (pip install pycodec2 -- sistemde libcodec2 paylaşımlı
kütüphanesi gerektirir, Ubuntu'da apt ile zaten geliyor) bu modül devre dışı
kalır, net bir uyarı basar -- DİNLE modu yerel hoparlörden/.wav'a çalmaya
DEVAM EDER, sadece radyoya ses satırı gitmez (bkz. sayisal_cozucu.py'deki
AYNI "opsiyonel araç" deseni).
"""
import base64
from fractions import Fraction

import numpy as np
from scipy.signal import resample_poly

try:
    import pycodec2
    _PYCODEC2_VAR = True
except Exception:
    pycodec2 = None
    _PYCODEC2_VAR = False

CODEC2_SAMPLE_RATE = 8000  # Codec2'nin sabit çalışma hızı (telefon kalitesi)


class SesKodlayici:
    """Demodüle-ses akışını (float32, herhangi bir giriş hızında) 8kHz'e
    indirip Codec2 ile kodlar, kodlanmış baytları biriktirir. .al() ile
    yeterince (min_bayt) birikmiş veri base64 metin olarak alınabilir --
    her 20ms'lik Codec2 çerçevesini (8 bayt) ayrı bir radyo satırı yapmak
    yerine (aşırı satır/çerçeveleme yükü) birkaç çerçeveyi (~200ms'lik ses)
    biriktirip TEK satırda gönderiyoruz."""

    def __init__(self, giris_sample_rate, mod_bps=3200):
        self._giris_sr = giris_sample_rate
        self._aktif = False
        self._codec = None
        self._birikmis_pcm = np.zeros(0, dtype=np.int16)
        self._cerceve_arabellegi = bytearray()

        if not _PYCODEC2_VAR:
            print("[SES KODLAYICI] pycodec2 KURULU DEĞİL (kurulum: pip install pycodec2) -- "
                  "DİNLE sesi radyoya gönderilmeyecek, yerel çalma/.wav kaydı ETKİLENMEZ.")
            return
        try:
            self._codec = pycodec2.Codec2(mod_bps)
            self._samples_per_frame = self._codec.samples_per_frame()
            self._aktif = True
            print(f"[SES KODLAYICI] Hazır (Codec2 {mod_bps}bps, "
                  f"{self._samples_per_frame} örnek/çerçeve @ {CODEC2_SAMPLE_RATE}Hz).")
        except Exception as e:
            print(f"[SES KODLAYICI] BAŞLATILAMADI: {e} -- DİNLE sesi radyoya gönderilmeyecek.")

    def besle(self, audio_float32):
        """DinlemeOturumu.isle()'ın zaten elinde olan float32 ses bloğunu
        (-1..1) besler."""
        if not self._aktif:
            return
        try:
            oran = Fraction(CODEC2_SAMPLE_RATE, int(self._giris_sr)).limit_denominator(1000)
            yeniden_orneklenmis = resample_poly(audio_float32, oran.numerator, oran.denominator)
            pcm16 = np.clip(yeniden_orneklenmis * 32767, -32768, 32767).astype(np.int16)
            self._birikmis_pcm = np.concatenate([self._birikmis_pcm, pcm16])

            while len(self._birikmis_pcm) >= self._samples_per_frame:
                cerceve = self._birikmis_pcm[:self._samples_per_frame]
                self._birikmis_pcm = self._birikmis_pcm[self._samples_per_frame:]
                kodlanmis = self._codec.encode(cerceve)
                self._cerceve_arabellegi.extend(kodlanmis)
        except Exception:
            pass  # bir bloğun kaybolması kritik değil, akış devam etsin

    def al(self, min_bayt=80):
        """Yeterince (min_bayt) Codec2 verisi biriktiyse base64 metin
        döndürür, yoksa None."""
        if not self._aktif or len(self._cerceve_arabellegi) < min_bayt:
            return None
        veri = bytes(self._cerceve_arabellegi)
        self._cerceve_arabellegi.clear()
        return base64.b64encode(veri).decode("ascii")
