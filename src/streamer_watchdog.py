"""
streamer.py'yi ayrı bir alt süreç olarak başlatıp izleyen gözcü (watchdog).

Neden gerekli: RTL-SDR/pyrtlsdr'da bilinen bir USB/libusb kararsızlığı var --
sdr.read_samples() çağrısı bazen SONSUZA KADAR bloke oluyor (ne bir exception
fırlatıyor ne süreç ölüyor, sadece veri üretimi duruyor). Bu blokaj Python
kodunun DIŞINDA, libusb'nin C katmanında olduğu için streamer.py'nin kendi
try/except'i bunu YAKALAYAMAZ -- döngü orada donup kalır. Tek çözüm dıştan
izleyip süreci zorla (TerminateProcess) öldürüp yeniden başlatmak.

Nasıl çalışır: streamer.py'yi alt süreç olarak başlatır, kendi ZMQ SUB'ıyla
port 5555'i (SYS/SPEC) dinler. WARMUP_S kadar (model yükleme + ilk tarama
turu için) sessizliğe göz yumar, ondan sonra SILENCE_TIMEOUT_S'den uzun süre
hiç paket gelmezse alt süreci öldürüp yeniden başlatır. Alt sürecin
stdout/stderr'i olduğu gibi bu sürecin konsoluna yansıtılır -- streamer.py'nin
kendi logları görünmeye devam eder.

Kullanım -- streamer.py'yi DOĞRUDAN DEĞİL, bunu çalıştır (aynı ortam
değişkenleriyle, ör. EBABIL_DINLEME_MOD, EBABIL_SCAN_START_MHZ):
  python src/streamer_watchdog.py
  EBABIL_DINLEME_MOD=WBFM python src/streamer_watchdog.py

Elle/GUI'den ZORLA yeniden başlatma:
  streamer.py çökmese bile (ör. operatör garip bir durum fark ettiğinde,
  WARMUP/SILENCE_TIMEOUT dolmasını beklemeden) GUI'nin zaten bağlandığı komut
  kanalından (port 5557, bkz. streamer.py'nin sub_cmd'si) "GOZCU,YENIDEN_BASLAT"
  metnini yayınlamak yeterli -- gözcü bunu ayrı bir SUB ile dinler, görür
  görmez alt süreci öldürüp anında yeniden başlatır. streamer.py da aynı
  mesajı görür ama tanımadığı için zararsızca "[!] Bilinmeyen komut" loglar.
"""
import os
import subprocess
import sys
import threading
import time

import zmq

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
STREAMER_PATH = os.path.join(_THIS_DIR, "streamer.py")

SYS_PORT = int(os.environ.get("EBABIL_WATCHDOG_PORT", "5555"))
# GUI'nin komut yayınladığı kanalla AYNI (bkz. streamer.py'deki sub_cmd) --
# EBABIL_GUI_HOST de aynı isimle streamer.py ile tutarlı, uzak/Jetson
# senaryosunda ikisi de aynı ortam değişkenine bakar.
GUI_HOST = os.environ.get("EBABIL_GUI_HOST", "127.0.0.1")
CMD_PORT = int(os.environ.get("EBABIL_WATCHDOG_CMD_PORT", "5557"))
FORCE_RESTART_CMD = "GOZCU,YENIDEN_BASLAT"
# Model yükleme + ilk tarama turu genelde ~10-15sn sürüyor -- bu süre boyunca
# hiç paket gelmemesi normal, gözcü henüz müdahale etmemeli.
WARMUP_S = float(os.environ.get("EBABIL_WATCHDOG_WARMUP_S", "25"))
# Isınma sonrası normal akışta paketler saniyede birkaç kez geliyor -- bu
# kadar uzun bir sessizlik donma anlamına gelir (dwell/dinleme geçişleri gibi
# meşru duraklamalar bile bu kadar sürmez).
SILENCE_TIMEOUT_S = float(os.environ.get("EBABIL_WATCHDOG_TIMEOUT_S", "10"))
CHECK_INTERVAL_S = 2.0
RESTART_DELAY_S = 2.0  # portların (5555/5556) serbest kalması için kısa bekleme


class HeartbeatMonitor:
    """streamer.py'nin PUB soketine (port 5555) ayrı bir thread'de SUB olarak
    bağlanıp her mesajda son görülme zamanını günceller. Ana thread bunu
    periyodik olarak sorup sessizlik süresine karar verir."""

    def __init__(self, port):
        self._port = port
        self._last_seen = time.time()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        with self._lock:
            self._last_seen = time.time()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def seconds_since_last_seen(self):
        with self._lock:
            return time.time() - self._last_seen

    def _run(self):
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://127.0.0.1:{self._port}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        sub.setsockopt(zmq.RCVTIMEO, 500)
        while not self._stop.is_set():
            try:
                sub.recv_string()
            except zmq.Again:
                continue
            with self._lock:
                self._last_seen = time.time()
        sub.close()
        ctx.term()


class RestartCommandListener:
    """GUI'nin (veya operatörün elle) komut kanalından (port 5557) FORCE_RESTART_CMD
    gelip gelmediğini ayrı bir thread'de dinler. streamer.py'nin kendi sub_cmd'siyle
    AYNI adrese ayrı bir SUB olarak bağlanır -- ZMQ PUB-SUB bire-çok yayın olduğu
    için ikisi de aynı mesajları görür, streamer.py tanımadığı komutu zaten
    zararsızca loglayıp yok sayıyor (bkz. streamer.py'deki "else" dalı)."""

    def __init__(self, gui_host, port):
        self._gui_host = gui_host
        self._port = port
        self._requested = threading.Event()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def consume_request(self):
        """Bekleyen bir istek varsa True döner ve bayrağı temizler (tek seferlik)."""
        if self._requested.is_set():
            self._requested.clear()
            return True
        return False

    def _run(self):
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://{self._gui_host}:{self._port}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        sub.setsockopt(zmq.RCVTIMEO, 500)
        while not self._stop.is_set():
            try:
                msg = sub.recv_string()
            except zmq.Again:
                continue
            if msg == FORCE_RESTART_CMD:
                self._requested.set()
        sub.close()
        ctx.term()


def _relay_output(pipe):
    for line in iter(pipe.readline, ""):
        print(line, end="", flush=True)
    pipe.close()


def main():
    args = [sys.executable, "-u", STREAMER_PATH]
    monitor = HeartbeatMonitor(SYS_PORT)
    # Komut kanalını (GUI'nin "YENİLE" düğmesi buraya "GOZCU,YENIDEN_BASLAT"
    # yayınlayacak) süreç yeniden başlasa da başlamasa da SÜREKLİ dinlemek
    # yeterli -- alt sürecin kimliğinden bağımsız, bu yüzden bir kere
    # başlatılıp dış döngü boyunca hep açık kalıyor.
    restart_listener = RestartCommandListener(GUI_HOST, CMD_PORT)
    restart_listener.start()

    while True:
        print(f"[gözcü] streamer.py başlatılıyor...")
        proc = subprocess.Popen(
            args, cwd=_THIS_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        threading.Thread(target=_relay_output, args=(proc.stdout,), daemon=True).start()

        monitor.start()
        started_at = time.time()

        while True:
            exit_code = proc.poll()
            if exit_code is not None:
                print(f"[gözcü] streamer.py kendiliğinden sonlandı (kod {exit_code}) -- yeniden başlatılıyor.")
                break

            if restart_listener.consume_request():
                print(f"[gözcü] Operatörden zorla yeniden başlatma komutu alındı ({FORCE_RESTART_CMD}) -- "
                      f"öldürülüp yeniden başlatılıyor.")
                proc.kill()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    print("[gözcü] Süreç 10sn içinde kapanmadı, devam ediliyor.")
                break

            warmed_up = (time.time() - started_at) > WARMUP_S
            if warmed_up and monitor.seconds_since_last_seen() > SILENCE_TIMEOUT_S:
                print(f"[gözcü] {SILENCE_TIMEOUT_S:.0f}s'den uzun süredir veri yok -- "
                      f"streamer.py donmuş olabilir, öldürülüp yeniden başlatılıyor.")
                proc.kill()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    print("[gözcü] Süreç 10sn içinde kapanmadı, devam ediliyor.")
                break

            time.sleep(CHECK_INTERVAL_S)

        monitor.stop()
        time.sleep(RESTART_DELAY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
