"""
konum_servisi icin DF referans noktasini (EBABIL_DF_REF_LAT/LON) ELLE
tahmin etmek yerine, Iha'nin KENDI GERCEK GPS fix'inden otomatik alir --
yarisma alaninin tam koordinati onceden kesin bilinmedigi icin (bkz.
proje notlari, "Diyarbakir/Mezopotamya civari ama kesin degil") bu,
sahada dogru yere gidip yanlis/yer tutucu bir referansla ucmaktan
(PF/EKF matematigi ENU duzlemini YANLIS yerde kurar, DF sonuclari anlamsiz
cikar) COK daha guvenilir.

mavlink_bridge.py'nin ZATEN yayinladigi port 5559'daki "UAV,lat,lon,..."
satirlarini dinler, ilk GECERLI (NaN olmayan) fix'i bulunca "<lat> <lon>"
olarak stdout'a basip cikar. GPS fix hic gelmezse (ic mekanda, GPS
antenine ragmen uydu gormuyorsa) EBABIL_DF_REF_WAIT_S (varsayilan 30sn)
sonra sessizce hata koduyla cikar -- caller (baslat_iha_rpi.sh) o zaman
varsayilan/yer tutucu degere doner, hic DF alamamaktansa yanlis ama
calisir bir sistemle devam eder.

Kullanim: python3 df_referans_al.py
  (mavlink_bridge.py'nin AYNI makinede, 5559'da ZATEN calisiyor olmasi sart)
"""
import math
import os
import sys
import time

import zmq

UAV_PORT = 5559
WAIT_S = float(os.environ.get("EBABIL_DF_REF_WAIT_S", "30"))


def main():
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://127.0.0.1:{UAV_PORT}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "UAV,")

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    deadline = time.time() + WAIT_S
    while time.time() < deadline:
        kalan_ms = max(0, int((deadline - time.time()) * 1000))
        olaylar = dict(poller.poll(timeout=kalan_ms))
        if sub not in olaylar:
            continue
        satir = sub.recv_string()
        parcalar = satir.split(",")
        if len(parcalar) < 3:
            continue
        try:
            lat, lon = float(parcalar[1]), float(parcalar[2])
        except ValueError:
            continue
        if math.isnan(lat) or math.isnan(lon):
            continue
        print(f"{lat:.7f} {lon:.7f}")
        return 0

    print("GPS fix alinamadi (sure doldu)", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
