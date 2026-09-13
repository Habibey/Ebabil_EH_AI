"""
Matek uçuş kontrolcüsünden (ArduPilot, MAVLink) gerçek İHA telemetrisini okuyup
arayüzün beklediği "UAV,lat,lon,alt,speed,heading,pitch,roll,battery" satırı
olarak ZeroMQ'dan yayınlar.

Neden ayrı port (5555 değil, 5559): streamer.py zaten 5555'i (SYS/SPEC için)
kendi açıyor -- aynı porta iki süreç birden bind edemez. EHARPP'in
ZmqSubscriber'ı hem 5555/5556'yı HEM bu portu (5559) dinleyecek şekilde
güncellendi (bkz. mainwindow.cpp setupZmqConnections).

Kullanım (Windows):
  python src/mavlink_bridge.py                  # varsayılan COM15, 115200 baud
  python src/mavlink_bridge.py --port COM16
  EBABIL_MAVLINK_PORT=COM15 python src/mavlink_bridge.py

Kullanım (Linux/Jetson -- port adları farklı, /dev/ttyACM0 gibi):
  python src/mavlink_bridge.py --port /dev/ttyACM0
  EBABIL_MAVLINK_PORT=/dev/ttyACM0 python src/mavlink_bridge.py
"""
import argparse
import math
import os
import sys
import time

import zmq
from pymavlink import mavutil

UAV_PUB_PORT = 5559
PUBLISH_INTERVAL_S = 0.5  # arayüz için yeterli, MAVLink akışını da bogmaz
# Matek'in varsayılan port adı işletim sistemine göre değişir -- Windows'ta
# COM<N>, Linux/Jetson'da genelde /dev/ttyACM<N> (native USB CDC seri port).
_DEFAULT_PORT = "COM15" if sys.platform == "win32" else "/dev/ttyACM0"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=os.environ.get("EBABIL_MAVLINK_PORT", _DEFAULT_PORT),
                         help=f"Matek'in bağlı olduğu port (varsayılan {_DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://127.0.0.1:{UAV_PUB_PORT}")

    print(f"[*] {args.port} bağlanılıyor (MAVLink, {args.baud} baud)...")
    conn = mavutil.mavlink_connection(args.port, baud=args.baud)

    print("[*] HEARTBEAT bekleniyor...")
    conn.wait_heartbeat(timeout=15)
    print(f"[+] Bağlandı: system={conn.target_system} component={conn.target_component}")
    print(f"[*] Port {UAV_PUB_PORT}: UAV paketleri yayınlanıyor\n")

    # En son bilinen değerler -- her mesaj tipi farklı sıklıkta geldiği için
    # (GPS ~5Hz, tutum ~10Hz, batarya ~1Hz gibi) her birini ayrı ayrı
    # güncelleyip belirli aralıklarla BİRLEŞTİRİP tek UAV satırı basıyoruz.
    state = {
        "lat": float("nan"), "lon": float("nan"), "alt": float("nan"),
        "speed": 0.0, "heading": 0.0, "pitch": 0.0, "roll": 0.0, "battery": 0.0,
    }

    last_publish = 0.0
    try:
        while True:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == "GLOBAL_POSITION_INT":
                    state["lat"] = msg.lat / 1e7
                    state["lon"] = msg.lon / 1e7
                    state["alt"] = msg.relative_alt / 1000.0
                elif msg_type == "VFR_HUD":
                    state["speed"] = msg.groundspeed
                    state["heading"] = msg.heading
                elif msg_type == "ATTITUDE":
                    state["pitch"] = math.degrees(msg.pitch)
                    state["roll"] = math.degrees(msg.roll)
                elif msg_type == "SYS_STATUS":
                    if msg.battery_remaining >= 0:
                        state["battery"] = msg.battery_remaining

            now = time.time()
            if now - last_publish >= PUBLISH_INTERVAL_S:
                last_publish = now
                fields = ["UAV", f"{state['lat']:.7f}", f"{state['lon']:.7f}", f"{state['alt']:.2f}",
                          f"{state['speed']:.2f}", f"{state['heading']:.1f}",
                          f"{state['pitch']:.1f}", f"{state['roll']:.1f}", f"{state['battery']:.0f}"]
                pub.send_string(",".join(fields))

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
