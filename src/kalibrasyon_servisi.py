"""
GUI'den (Qt arayuz'ün "SAHA KALİBRASYONU" paneli) KAL_BASLAT/KAL_DURDUR
komutlarıyla tetiklenen saha kalibrasyonu denetleyicisi.

TASARIM NOTU (2026-09-18): kalibrasyon_kaydedici.py'nin kendi dosya başı
yorumu bunu GUI'den kasıtlı ayrı tutmuştu ("arayüz bu aracın çalışıp
çalışmadığından tamamen habersizdir -- istenen buydu"). Bu dosya o kararı
BİLEREK geziyor -- GUI'den tetikleme istendi. kalibrasyon_kaydedici.py'nin
kendisi DEĞİŞMEDİ (mantığı hâlâ elle/terminalden de çalıştırılabilir) --
bu, onu subprocess olarak yöneten AYRI, opsiyonel bir katman.

Nasıl çalışır (streamer_watchdog.py'deki komut-dinleme deseniyle AYNI --
GUI'nin sub_cmd fan-out'una ayrı bir SUB olarak bağlanır):
  1. Komut kanalından (port 5557) "KAL_BASLAT|<tx_lat>|<tx_lon>|<freq_mhz>|<sure_s>"
     gelince kalibrasyon_kaydedici.py'yi subprocess olarak başlatır
     (--sure-s 0 ise KAL_DURDUR gelene kadar sınırsız).
  2. "KAL_DURDUR" gelince subprocess'e SIGINT gönderir -- script'in kendi
     Ctrl+C işleyicisi devreye girip KAL_HESAPLA'yı çalıştırıp normal çıkış
     yapar (bkz. kalibrasyon_kaydedici.py'deki signal.signal(SIGINT, dur)).
  3. Subprocess bitince --sonuc-dosya'ya yazdığı ham sonucu ("OK,P0,n,...",
     "YETERSIZ,<sayı>" ya da "HATA,...") okuyup "KALSONUC,<freq_mhz>,<sonuc>"
     satırı olarak YEREL bir PUB porttan (varsayılan 5562) yayınlar --
     seri_telemetri_koprusu.py'nin KAYNAK_PORTLARI listesine bu port
     eklendiği için (2026-09-18) otomatik olarak radyo üzerinden Jetson'a,
     oradan da GUI'ye ulaşır (bkz. arayuz/mainwindow.cpp parseLine, "KALSONUC").

Kullanım (diğer RPi süreçleriyle BİRLİKTE, bkz. scripts/baslat_iha_rpi.sh):
  EBABIL_GUI_HOST=127.0.0.1 python3 src/kalibrasyon_servisi.py
"""
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import zmq

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
KAYDEDICI_PATH = os.path.join(_THIS_DIR, "kalibrasyon_kaydedici.py")

GUI_HOST = os.environ.get("EBABIL_GUI_HOST", "127.0.0.1")
CMD_PORT = int(os.environ.get("EBABIL_WATCHDOG_CMD_PORT", "5557"))
SONUC_PORT = int(os.environ.get("EBABIL_KAL_SONUC_PORT", "5562"))

DURDURMA_ZAMAN_ASIMI_S = 15.0


def _relay_output(pipe, etiket):
    for line in iter(pipe.readline, ""):
        print(f"[{etiket}] {line}", end="", flush=True)
    pipe.close()


class KalibrasyonDenetleyicisi:
    def __init__(self, sonuc_pub):
        self._sonuc_pub = sonuc_pub
        self._proc = None
        self._sonuc_dosya = None
        self._freq_mhz = None
        self._lock = threading.Lock()

    def baslat(self, tx_lat, tx_lon, freq_mhz, sure_s):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                print(f"[KAL-SVC] Zaten çalışan bir kalibrasyon var ({self._freq_mhz} MHz) -- yeni istek yok sayıldı.")
                return

            fd, sonuc_dosya = tempfile.mkstemp(prefix="ebabil_kal_sonuc_", suffix=".txt")
            os.close(fd)

            args = [
                sys.executable, "-u", KAYDEDICI_PATH,
                "--tx-lat", str(tx_lat),
                "--tx-lon", str(tx_lon),
                "--freq-mhz", str(freq_mhz),
                "--sure-s", str(sure_s),
                "--sonuc-dosya", sonuc_dosya,
            ]
            print(f"[KAL-SVC] Kalibrasyon başlatılıyor: TX={tx_lat},{tx_lon} @ {freq_mhz} MHz")
            self._proc = subprocess.Popen(
                args, cwd=_THIS_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
            self._sonuc_dosya = sonuc_dosya
            self._freq_mhz = freq_mhz
            threading.Thread(target=_relay_output, args=(self._proc.stdout, "KAL"), daemon=True).start()

            if sure_s > 0:
                threading.Thread(target=self._sureli_bekle, args=(self._proc, sure_s), daemon=True).start()

    def _sureli_bekle(self, proc, sure_s):
        try:
            proc.wait(timeout=sure_s + DURDURMA_ZAMAN_ASIMI_S)
        except subprocess.TimeoutExpired:
            pass
        self._sonucu_isle(proc)

    def durdur(self):
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                print("[KAL-SVC] Durdurulacak aktif kalibrasyon yok.")
                return
            print("[KAL-SVC] Kalibrasyon durduruluyor (SIGINT) -- KAL_HESAPLA bekleniyor...")
            proc.send_signal(signal.SIGINT)

        threading.Thread(target=self._durdurmayi_bekle, args=(proc,), daemon=True).start()

    def _durdurmayi_bekle(self, proc):
        try:
            proc.wait(timeout=DURDURMA_ZAMAN_ASIMI_S)
        except subprocess.TimeoutExpired:
            print(f"[KAL-SVC] Süreç {DURDURMA_ZAMAN_ASIMI_S:.0f}sn içinde kapanmadı, zorla sonlandırılıyor.")
            proc.kill()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        self._sonucu_isle(proc)

    def _sonucu_isle(self, proc):
        with self._lock:
            if proc is not self._proc:
                return  # zaten yeni bir kalibrasyon başlamış, eski sonucu yayınlama
            freq_mhz = self._freq_mhz
            sonuc_dosya = self._sonuc_dosya
            self._proc = None

        ham = None
        if sonuc_dosya and os.path.exists(sonuc_dosya):
            try:
                with open(sonuc_dosya) as f:
                    ham = f.read().strip()
            except OSError:
                pass
            finally:
                try:
                    os.remove(sonuc_dosya)
                except OSError:
                    pass

        if not ham:
            ham = "HATA,sonuc_dosyasi_bulunamadi"

        satir = f"KALSONUC,{freq_mhz},{ham}"
        print(f"[KAL-SVC] Sonuç yayınlanıyor: {satir}")
        self._sonuc_pub.send_string(satir)


def main():
    ctx = zmq.Context()

    sonuc_pub = ctx.socket(zmq.PUB)
    sonuc_pub.bind(f"tcp://127.0.0.1:{SONUC_PORT}")
    # PUB soketinin ilk aboneye bağlanması (seri_telemetri_koprusu.py'nin
    # SUB'ı) birkaç yüz ms sürebilir ("slow joiner") -- servis başlar
    # başlamaz bir kalibrasyon biter biterse ilk mesaj kaybolabilir, bu kısa
    # bekleme pratikte yeterli.
    time.sleep(0.3)

    sub_cmd = ctx.socket(zmq.SUB)
    sub_cmd.connect(f"tcp://{GUI_HOST}:{CMD_PORT}")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")
    sub_cmd.setsockopt(zmq.RCVTIMEO, 500)

    denetleyici = KalibrasyonDenetleyicisi(sonuc_pub)

    print(f"[KAL-SVC] Hazır -- komut kanalı tcp://{GUI_HOST}:{CMD_PORT}, sonuç yayını port {SONUC_PORT}.")
    print("[KAL-SVC] Bekleniyor: KAL_BASLAT|<tx_lat>|<tx_lon>|<freq_mhz>|<sure_s>  /  KAL_DURDUR")

    while True:
        try:
            msg = sub_cmd.recv_string()
        except zmq.Again:
            continue
        except KeyboardInterrupt:
            break

        if msg.startswith("KAL_BASLAT|"):
            parcalar = msg.split("|")
            if len(parcalar) != 5:
                print(f"[KAL-SVC] Geçersiz KAL_BASLAT komutu: {msg!r}")
                continue
            try:
                tx_lat = float(parcalar[1])
                tx_lon = float(parcalar[2])
                freq_mhz = float(parcalar[3])
                sure_s = float(parcalar[4])
            except ValueError:
                print(f"[KAL-SVC] Geçersiz KAL_BASLAT parametreleri: {msg!r}")
                continue
            denetleyici.baslat(tx_lat, tx_lon, freq_mhz, sure_s)

        elif msg == "KAL_DURDUR":
            denetleyici.durdur()

        # Diğer komutlar (BANT_AYARLA, ET,... vb.) bizi ilgilendirmiyor,
        # streamer.py/et_control.py'nin kendi dinleyicileri zaten var --
        # sessizce yok sayılıyor (streamer_watchdog.py'deki AYNI ilke).


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
