"""
İHA'daki (RPi) tarafta 915MHz telemetri radyosunun TEK sahibi.

streamer.py (5555 SYS/SPEC, 5556 AI), pluto_ed_scanner.py (5560/5561, varsa)
ve mavlink_bridge.py'nin (5559 UAV) YEREL ZMQ PUB'larına SUB olup gördüğü HER
satırı -- format hiç ayrıştırılmadan, opak metin olarak -- radyoya yazar.
Yer istasyonundaki yer_istasyonu_koprusu (C++, ebabil-eh-backend reposu) bunu
okuyup GUI'nin ZMQ portlarına (5555/5556/5559) geri basıyor; format ikisinde
de AYNI olduğu için GUI_QtCreator tarafında hiçbir değişiklik gerekmiyor.

Ters yön -- radyodan gelen komut satırları (GUI'den "DINLE_BASLAT|...",
"BANT_AYARLA|...", "ET,BASLAT,..." vb., format burada da yorumlanmıyor):
yerel bir ZMQ PUB'dan (port 5557) yayınlanır. streamer.py/pluto_ed_scanner.py/
et_control.py'nin ZATEN VAR OLAN komut dinleyicisi (sub_cmd) bunu otomatik
alır -- YETER Kİ o süreçler EBABIL_GUI_HOST=127.0.0.1 ile başlatılsın (yani
"GUI" olarak bu köprüye baksınlar). Hiçbir backend kodu DEĞİŞMEDİ.

NEDEN AI için DATA96/ikili IQ paketleme YOK (ebabil_sdr/yer_istasyonu_koprusu
ikilisinin aksine): streamer.py zaten predict.py'yi AYNI süreç içinde
doğrudan çağırıyor (bkz. handle_classify_request) -- sınıflandırma RPi'de
lokal olarak yapılıp sonucu ("AI,...") diğer satırlar gibi düz metin olarak
buradan geçiyor. Ayrı bir ikili protokole/Jetson'a ait bir yapay zeka
servisine gerek yok. TEK KOŞUL: RPi'de tflite_runtime'ın (TensorFlow yoksa)
gerçek zamanlı sınıflandırma için yeterince hızlı olduğu doğrulanmalı --
yavaş çıkarsa, ai_servisi.py + Jetson tabanlı alternatif hâlâ mevcut.

NEDEN FC'nin ham MAVLink çerçeveleri Mission Planner'a AKTARILMIYOR: kapsam
bilinçli olarak daraltıldı. mavlink_bridge.py zaten FC'den okuyup temiz
"UAV,..." metin satırı üretiyor (port 5559) -- bu köprü onu OLDUĞU GİBİ
aktarıyor, GUI için yeterli. Mission Planner'ın kendi ham MAVLink akışını
görmesi ayrıca gerekiyorsa (rota izleme vb.) bu sonradan eklenecek ayrı bir
iş -- bkz. proje notları.

Kullanım (RPi'de, diğer süreçlerle BİRLİKTE, ayrı terminallerde/servislerde):
  python src/mavlink_bridge.py &
  EBABIL_GUI_HOST=127.0.0.1 python src/streamer.py &
  python src/seri_telemetri_koprusu.py --port /dev/ttyUSB0

Yerel test (donanımsız/tek makinede, gerçek radyo yerine socat sanal port
çifti -- bkz. ebabil-eh-backend/ebabil_baslat.sh'deki AYNI fikir):
  socat -d -d pty,raw,echo=0,link=/tmp/ebabil_radyo_iha pty,raw,echo=0,link=/tmp/ebabil_radyo_yer &
  python src/seri_telemetri_koprusu.py --port /tmp/ebabil_radyo_iha
"""
import argparse
import os
import time

import serial
import zmq

DEFAULT_RADIO_PORT = os.environ.get("EBABIL_TELEMETRI_PORT", "/dev/ttyUSB0")
DEFAULT_BAUD = int(os.environ.get("EBABIL_TELEMETRI_BAUD", "57600"))

# Bu RPi'deki yerel kaynaklar -- hepsi ZATEN çalışan bağımsız süreçler
# (streamer.py, pluto_ed_scanner.py, mavlink_bridge.py), bu köprü sadece
# dinleyip radyoya aktarır, hiçbirini yorumlamaz.
KAYNAK_PORTLARI = [5555, 5556, 5559, 5560, 5561]

KOMUT_PUB_PORT = 5557


class SeriPort:
    """Radyo seri portuna TEK yazıcı/okuyucu -- satır satır, kısmi yazmaları
    tekrar deneyen sağlam bir write + parçalı gelen veriyi biriktiren bir
    okuma tamponu (bkz. C++ tarafındaki TelemetriGonderici'nin AYNI ilkesi:
    "bir read() = bir tam satır" varsayımı YAPMAZ)."""

    def __init__(self, yol, baud):
        self._ser = serial.Serial(yol, baudrate=baud, timeout=0)
        self._okuma_tamponu = b""

    def satir_yaz(self, satir):
        veri = (satir + "\n").encode("utf-8", errors="replace")
        kalan = veri
        while kalan:
            n = self._ser.write(kalan)
            if n is None:
                n = 0
            kalan = kalan[n:]

    def satirlari_oku(self):
        """Bekleyen tüm tam satırları (varsa) döndürür, yoksa boş liste --
        NON-BLOCKING (Serial timeout=0 ile açıldı)."""
        parca = self._ser.read(4096)
        if parca:
            self._okuma_tamponu += parca
        satirlar = []
        while b"\n" in self._okuma_tamponu:
            satir, self._okuma_tamponu = self._okuma_tamponu.split(b"\n", 1)
            satir = satir.strip()
            if satir:
                satirlar.append(satir.decode("utf-8", errors="replace"))
        return satirlar


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_RADIO_PORT,
                         help=f"915MHz radyonun bağlı olduğu seri port (varsayılan {DEFAULT_RADIO_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    print(f"[SERİ KÖPRÜ] {args.port} açılıyor ({args.baud} baud)...")
    seri = SeriPort(args.port, args.baud)
    print("[SERİ KÖPRÜ] Radyo hazır.")

    ctx = zmq.Context()
    poller = zmq.Poller()
    subs = []
    for port in KAYNAK_PORTLARI:
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://127.0.0.1:{port}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        poller.register(sub, zmq.POLLIN)
        subs.append(sub)
    print(f"[SERİ KÖPRÜ] Yerel kaynaklar dinleniyor: {KAYNAK_PORTLARI} -> radyo "
          "(çalışmayan portlar sessizce atlanır, bağlanınca otomatik devreye girer).")

    komut_pub = ctx.socket(zmq.PUB)
    komut_pub.bind(f"tcp://127.0.0.1:{KOMUT_PUB_PORT}")
    print(f"[SERİ KÖPRÜ] Radyodan gelen komutlar yerel port {KOMUT_PUB_PORT}'den yayınlanacak "
          "-- streamer.py/pluto_ed_scanner.py/et_control.py EBABIL_GUI_HOST=127.0.0.1 ile "
          "başlatılmalı ki bunu 'GUI' sanıp dinlesinler.")
    print("[SERİ KÖPRÜ] Hazır.\n")

    satir_sayaci = 0
    try:
        while True:
            olaylar = dict(poller.poll(timeout=20))
            for sub in subs:
                if sub not in olaylar:
                    continue
                try:
                    satir = sub.recv_string(flags=zmq.NOBLOCK)
                except zmq.Again:
                    continue
                seri.satir_yaz(satir)
                satir_sayaci += 1

            for komut_satiri in seri.satirlari_oku():
                komut_pub.send_string(komut_satiri)
                print(f"[SERİ KÖPRÜ] Radyodan komut alındı, yerele iletildi: {komut_satiri!r}")

            if satir_sayaci and satir_sayaci % 200 == 0:
                print(f"[SERİ KÖPRÜ] Şu ana kadar radyoya yazılan satır: {satir_sayaci}")
                satir_sayaci += 1  # aynı mesajı her satırda tekrar basma

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
