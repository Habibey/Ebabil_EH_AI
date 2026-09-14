"""
Sayısal (dijital) telsiz decode -- madde 5.1.3'ün opsiyonel/ek puan kısmı.

AFSK/AX.25/APRS demodülasyonunu SIFIRDAN YAZMAK yerine (bit senkronizasyonu,
HDLC çerçeveleme, bit-stuffing, FCS doğrulama -- hataya açık, uzun sürede
doğru oturtulan bir iş) `multimon-ng` (apt paketi) alt süreç olarak
çalıştırılıp DinlemeOturumu'nun zaten ürettiği demodüle sese uygulanıyor.
C++ tarafındaki SayisalCozucu.h'nin yorumunda da AYNI araç ("multimon-ng
benzeri") referans alınmış -- burada gerçekten O aracın kendisi kullanılıyor.

multimon-ng kurulu değilse (`sudo apt install multimon-ng`) bu modül devre
dışı kalır, net bir uyarı basar -- streamer.py/pluto_ed_scanner.py'nin geri
kalanı ETKİLENMEZ (bkz. RtlRfAnahtari/PlutoEtRfAnahtari'daki AYNI "opsiyonel
donanım/araç" deseni).

NOT -- DOĞRULANMADI: Bu makinede multimon-ng kurulu değildi (sudo şifresi
gerektiriyor, interaktif terminal yoktu) -- kod multimon-ng'nin bilinen
CLI davranışına göre yazıldı ama GERÇEK bir sinyalle uçtan uca TEST
EDİLEMEDİ. Kurulumdan sonra mutlaka gerçek bir APRS/AFSK1200 sinyaliyle
doğrulanmalı.
"""
import queue
import shutil
import subprocess
import threading
from fractions import Fraction

import numpy as np
from scipy.signal import resample_poly

# multimon-ng'nin "-t raw" (stdin'den ham PCM) modu SABİT 22050 Hz, 16-bit
# signed, mono bekliyor (çalışma zamanında değiştirilebilen bir CLI seçeneği
# yok) -- demod.py'nin çıktısı (demod.AUDIO_SAMPLE_RATE=48000) buraya
# yeniden örneklenmeli (bkz. besle()).
MULTIMON_NG_SAMPLE_RATE = 22050

_MULTIMON_NG_YOLU = shutil.which("multimon-ng")


class SayisalCozucu:
    """Demodüle-ses akışını (float32, herhangi bir giriş hızında) multimon-ng'ye
    besleyip çözülen metin satırlarını okur. Alt süreç sürekli çalışır --
    her ses bloğu .besle() ile stdin'ine yazılır, stdout'u ayrı bir thread'de
    okunup bir kuyruğa birikir (.oku() bloklamadan, biriken her şeyi alır)."""

    def __init__(self, giris_sample_rate, modlar=("AFSK1200",)):
        self._giris_sr = giris_sample_rate
        self._aktif = False
        self._proc = None
        self._kuyruk = queue.Queue()

        if _MULTIMON_NG_YOLU is None:
            print("[SAYISAL ÇÖZÜCÜ] multimon-ng KURULU DEĞİL (kurulum: "
                  "sudo apt install multimon-ng) -- sayısal decode devre dışı, "
                  "geri kalan her şey normal çalışmaya devam ediyor.")
            return

        try:
            komut = [_MULTIMON_NG_YOLU, "-t", "raw", "-a"] + list(modlar) + ["-q", "-"]
            self._proc = subprocess.Popen(
                komut, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            )
            self._aktif = True
            threading.Thread(target=self._oku_dongusu, daemon=True).start()
            print(f"[SAYISAL ÇÖZÜCÜ] Hazır (multimon-ng, modlar={modlar}).")
        except Exception as e:
            print(f"[SAYISAL ÇÖZÜCÜ] BAŞLATILAMADI: {e} -- sayısal decode devre dışı.")

    def _oku_dongusu(self):
        for satir in self._proc.stdout:
            metin = satir.decode("utf-8", errors="replace").strip()
            if metin:
                self._kuyruk.put(metin)

    def besle(self, audio_float32):
        """DinlemeOturumu.isle()'ın ZATEN elinde olan float32 ses bloğunu
        (-1..1 aralığında) besler -- 22050 Hz'e yeniden örnekleyip 16-bit
        PCM'e çevirir, multimon-ng'nin stdin'ine yazar."""
        if not self._aktif:
            return
        try:
            oran = Fraction(MULTIMON_NG_SAMPLE_RATE, int(self._giris_sr)).limit_denominator(1000)
            yeniden_orneklenmis = resample_poly(audio_float32, oran.numerator, oran.denominator)
            pcm16 = np.clip(yeniden_orneklenmis * 32767, -32768, 32767).astype(np.int16)
            self._proc.stdin.write(pcm16.tobytes())
            self._proc.stdin.flush()
        except Exception:
            pass  # bir bloğun kaybolması kritik değil, akış devam etsin

    def oku(self):
        """Bekleyen tüm çözülmüş metin satırlarını (varsa) döndürür, yoksa
        boş liste -- ASLA bloklamaz."""
        satirlar = []
        while True:
            try:
                satirlar.append(self._kuyruk.get_nowait())
            except queue.Empty:
                break
        return satirlar

    def kapat(self):
        if self._proc is not None:
            try:
                self._proc.terminate()
            except Exception:
                pass
