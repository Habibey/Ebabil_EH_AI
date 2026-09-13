"""
streamer.py ve pluto_ed_scanner.py'nin ORTAK kullandığı iki yardımcı:

1) UavKonumDinleyici -- mavlink_bridge.py'nin yayınladığı "UAV,..." satırlarını
   (port 5559) arka planda dinleyip en son İHA konumunu/irtifasını önbellekte
   tutar. mavlink_bridge.py AYRI bir süreç olduğu için (aynı RTL-SDR/Pluto
   donanımını streamer.py/pluto_ed_scanner.py'nin process'i zaten kullanıyor,
   MAVLink bağlantısını da onlara eklemek yerine ayrı tutuldu) buraya ZMQ ile
   bağlanıyoruz.

2) KonumIstemcisi -- konum_servisi'ne (C++, yonKonum1905/konum_servisi) ZMQ
   REQ/REP ile bağlanan istemci. Yön bulma + Parçacık Filtresi/EKF matematiği
   ORADA (zaten yazılıp test edilmiş C++ kodu) -- burada SADECE girdi
   (gerçek RSSI + o anki İHA konumu) gönderip çıktıyı (hedef enlem/boylam +
   türetilmiş açı) okuyoruz. Neden Python'da yeniden YAZILMADI: aynı
   matematiği ikinci kez (ve test tarihinde farklı bir hatayla) üretme riski
   -- bkz. proje notları.

Anten çifti donanımı YOK (tek anten, doğrulandı) -- bu yüzden yön bulma
"menzil-only" yöntemle çalışıyor: gerçek RSSI + gerçek İHA konumu Parçacık
Filtresi/EKF'ye besleniyor, açı ise PF/EKF'nin ürettiği hedef konumundan
SONRADAN (atan2 ile) türetiliyor -- doğrudan ölçülmüyor.
"""
import os
import threading
import time

import zmq


class UavKonumDinleyici:
    """Arka planda mavlink_bridge.py'nin 5559 numaralı portundaki "UAV,..."
    satırlarını dinler, en son lat/lon/irtifa'yı thread-safe şekilde tutar.
    mavlink_bridge.py henüz çalışmıyorsa (ya da GPS henüz kilitlenmediyse)
    son_konum() None döner -- çağıran taraf bunu "henüz gerçek konum yok"
    olarak ele almalı, sahte/varsayılan bir konum ÜRETMEMELİ."""

    def __init__(self, host=None, port=5559):
        self._host = host or os.environ.get("EBABIL_MAVLINK_HOST", "127.0.0.1")
        self._port = port
        self._kilit = threading.Lock()
        self._lat = None
        self._lon = None
        self._irtifa = None
        self._calisiyor = True
        self._thread = threading.Thread(target=self._dinle, daemon=True)
        self._thread.start()

    def _dinle(self):
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://{self._host}:{self._port}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        sub.setsockopt(zmq.RCVTIMEO, 500)
        while self._calisiyor:
            try:
                satir = sub.recv_string()
            except zmq.Again:
                continue
            except Exception:
                continue
            parts = satir.split(",")
            if len(parts) != 9 or parts[0] != "UAV":
                continue
            try:
                lat, lon, irtifa = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            if lat != lat or lon != lon:  # NaN kontrolü -- GPS henuz kilitlenmedi
                continue
            with self._kilit:
                self._lat, self._lon, self._irtifa = lat, lon, irtifa

    def son_konum(self):
        """(lat, lon, irtifa) ya da (None, None, None) -- henüz gerçek GPS
        konumu gelmediyse."""
        with self._kilit:
            return self._lat, self._lon, self._irtifa

    def durdur(self):
        self._calisiyor = False


class KonumIstemcisi:
    """konum_servisi'ne (bkz. yonKonum1905/konum_servisi) ZMQ REQ/REP ile
    bağlanan, zaman aşımında soketi güvenli şekilde yenileyen istemci
    (bkz. "lazy pirate" deseni -- REQ soketleri cevap gelmeden ikinci bir
    istek atılırsa kilitlenir, bu yüzden zaman aşımında soket KAPATILIP
    YENİDEN açılıyor)."""

    def __init__(self, host=None, port=None, zaman_asimi_ms=200):
        self._host = host or os.environ.get("EBABIL_KONUM_SERVISI_HOST", "127.0.0.1")
        self._port = port or int(os.environ.get("EBABIL_KONUM_SERVISI_PORT", "5570"))
        self._zaman_asimi_ms = zaman_asimi_ms
        self._ctx = zmq.Context()
        self._sock = None
        self._baglan()

    def _baglan(self):
        if self._sock is not None:
            self._sock.close(linger=0)
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.RCVTIMEO, self._zaman_asimi_ms)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(f"tcp://{self._host}:{self._port}")

    def guncelle(self, tid, band_hz, rssi_dbm, uav_lat, uav_lon, uav_irtifa_m):
        """Basarili olursa (hedef_lat, hedef_lon, aci_deg, rms_derece)
        dondurur; PF/EKF henuz gecerli bir konum uretmediyse ya da servis
        yanit vermiyorsa None doner (cagiran taraf DF satiri GONDERMEMELI)."""
        istek = f"GUNCELLE,{tid},{band_hz},{rssi_dbm},{uav_lat},{uav_lon},{uav_irtifa_m}"
        try:
            self._sock.send_string(istek)
            cevap = self._sock.recv_string()
        except zmq.Again:
            self._baglan()  # zaman asimi -- soket bozuldu, yenile
            return None
        except Exception:
            self._baglan()
            return None

        parcalar = cevap.split(",")
        if parcalar[0] != "OK" or len(parcalar) != 5:
            return None
        try:
            return float(parcalar[1]), float(parcalar[2]), float(parcalar[3]), float(parcalar[4])
        except ValueError:
            return None

    def sifirla(self, tid):
        try:
            self._sock.send_string(f"SIFIRLA,{tid}")
            self._sock.recv_string()
        except Exception:
            self._baglan()


def konum_guncelle_ve_gonder(pub, konum_istemcisi, uav_konum, tid, freq_mhz, power_db):
    """streamer.py ve pluto_ed_scanner.py'nin ORTAK kullandığı yardımcı --
    her SYS güncellemesinden sonra çağrılır. Gerçek İHA konumu VE
    konum_servisi ikisi de hazırsa "DF,..." satırını GUI'nin beklediği
    formatta (bkz. GUI_QtCreator mainwindow.cpp parseLine, "DF" tipi)
    üretip aynı SYS/SPEC portundan (pub) yayınlar. İkisinden biri hazır
    değilse (GPS henüz kilitlenmedi, konum_servisi henüz ayakta değil vb.)
    SESSİZCE atlanır -- SYS/SPEC akışını hiç etkilemez, sahte bir DF
    satırı ASLA gönderilmez."""
    uav_lat, uav_lon, uav_irtifa = uav_konum.son_konum()
    if uav_lat is None:
        return
    band_hz = freq_mhz * 1e6
    sonuc = konum_istemcisi.guncelle(tid, band_hz, power_db, uav_lat, uav_lon, uav_irtifa)
    if sonuc is None:
        return
    hedef_lat, hedef_lon, aci_deg, rms_derece = sonuc
    pub.send_string(f"DF,{tid},IHA_MENZIL,{aci_deg:.2f},{rms_derece:.2f},{hedef_lat:.7f},{hedef_lon:.7f}")
