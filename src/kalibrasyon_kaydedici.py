"""
Saha (uçuş) RF path-loss kalibrasyon kaydedici -- yonKonum1905/yonKonum/testler/
path_loss_kalibrasyon_araci.cpp'nin (elle mesafe girilen, düz 0-6m yolda kullanılan
masaüstü aracı) GERÇEK/uçan hali.

Fark: orada mesafe/açı elle klavyeden giriliyordu; burada bilinen (sabit) bir GPS
konumundaki bir verici referans alınıp, o anki İHA GPS konumundan mesafe HER SYS
paketinde OTOMATIK hesaplanır. Kullanıcı hiçbir şey girmez, İHA vericinin üstünden
uçtukça (ya da yakınından geçtikçe) örnekler kendiliğinden birikir.

Bilerek GUI'ye BAĞLANMAZ / GUI'de görünmez: streamer.py'nin/pluto_ed_scanner.py'nin
SYS yayınına sadece SUB olarak dinler (5555/5560), GUI'nin okuduğu hiçbir porta
(5556 AI, 5557 komut) yazmaz, kendi PUB soketi de yoktur -- yani arayüz bu aracın
çalışıp çalışmadığından tamamen habersizdir (istenen buydu).

Kullanım (örnek -- bilinen bir 433.92 MHz vericinin GPS konumu biliniyorsa):
    python3 kalibrasyon_kaydedici.py \
        --tx-lat 39.925123 --tx-lon 32.836789 \
        --freq-mhz 433.92 --freq-tol-mhz 0.05

Ctrl+C ile durdurulduğunda (ya da --sure-s dolduğunda) konum_servisi'ne
KAL_HESAPLA gönderir ve sonucu (P0/n/sigma) ekrana basar -- BUNDAN SONRA
konum_servisi'nde açılacak TÜM hedefler bu bant için bu gerçek kalibrasyonu
kullanır (main.cpp'deki demo yer tutucunun yerine geçer).

Ham örnekler ayrıca yerel bir CSV dosyasına da yazılır ("her konumdan aldığı
veriyi kaydetsin") -- konum_servisi'nin bellek-içi örnek listesi süreç
yeniden başlatılınca kaybolur, CSV kalıcı kayıt/denetim izi sağlar.
"""
import argparse
import csv
import math
import os
import signal
import sys
import time

import zmq

DUNYA_YARICAP_M = 6378137.0


def mesafe_hesapla_m(lat1, lon1, lat2, lon2):
    """main.cpp / konum_servisi'ndeki enlemBoylamdanYerele ile AYNI düz-dünya
    (flat-earth) yaklaşımı -- iki tarafın da aynı yaklaşıklığı kullanması
    için kasıtlı olarak birebir aynı formül (referans (lat1,lon1))."""
    lat0_rad = math.radians(lat1)
    y_kuzey = math.radians(lat2 - lat1) * DUNYA_YARICAP_M
    x_dogu = math.radians(lon2 - lon1) * DUNYA_YARICAP_M * math.cos(lat0_rad)
    return math.hypot(x_dogu, y_kuzey)


class KonumServisiKalibrasyon:
    """konum_servisi'nin KAL_EKLE/KAL_HESAPLA/KAL_SIFIRLA uçlarına REQ/REP
    ile bağlanan küçük istemci -- konum_istemcisi.py'deki KonumIstemcisi ile
    aynı "lazy pirate" (zaman aşımında soketi yenile) deseni."""

    def __init__(self, host=None, port=None, zaman_asimi_ms=1000):
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

    def _istek(self, metin):
        try:
            self._sock.send_string(metin)
            return self._sock.recv_string()
        except Exception:
            self._baglan()
            return None

    def ekle(self, band_hz, mesafe_m, rssi_dbm):
        return self._istek(f"KAL_EKLE,{band_hz},{mesafe_m:.2f},{rssi_dbm:.2f}")

    def hesapla(self, band_hz):
        return self._istek(f"KAL_HESAPLA,{band_hz}")

    def sifirla(self, band_hz):
        return self._istek(f"KAL_SIFIRLA,{band_hz}")


def sys_satirini_parcala(satir):
    """streamer.py/pluto_ed_scanner.py'deki build_sys_fields ile AYNI format:
    SYS,id,tespit,lat,lon,alt,freq_mhz,power_db,bant_khz,sapma_mhz,gurultu_db,snr_db,sureklilik
    Sadece bu kalibrasyon aracının kullandığı freq_mhz + power_db (RSSI) alınır."""
    p = satir.split(",")
    if len(p) != 13 or p[0] != "SYS":
        return None
    try:
        freq_mhz = float(p[6])
        power_db = float(p[7])
    except ValueError:
        return None
    if freq_mhz != freq_mhz or power_db != power_db:  # NaN
        return None
    return freq_mhz, power_db


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tx-lat", type=float, required=True, help="Bilinen vericinin GPS enlemi")
    ap.add_argument("--tx-lon", type=float, required=True, help="Bilinen vericinin GPS boylamı")
    ap.add_argument("--freq-mhz", type=float, required=True, help="Kalibrasyon vericisinin frekansı (MHz)")
    ap.add_argument("--freq-tol-mhz", type=float, default=0.05, help="Frekans eşleşme toleransı (MHz, varsayılan 0.05)")
    ap.add_argument("--min-mesafe-m", type=float, default=3.0,
                     help="Bu mesafenin altındaki örnekler atlanır (yakın-alan/anten etkisi -- varsayılan 3m)")
    ap.add_argument("--min-ornek-araligi-m", type=float, default=5.0,
                     help="Bir önceki kaydedilen örnekten en az bu kadar uzaklaşmadan yeni örnek alınmaz (varsayılan 5m, İHA hover halindeyken aynı noktadan yüzlerce özdeş örnek birikmesin diye)")
    ap.add_argument("--min-ornek-araligi-s", type=float, default=1.0,
                     help="İki örnek arası minimum süre (saniye, varsayılan 1.0)")
    ap.add_argument("--sure-s", type=float, default=0.0, help="Kaç saniye sonra otomatik durup KAL_HESAPLA çalıştırılsın (0=sınırsız, Ctrl+C ile durdurulana kadar)")
    ap.add_argument("--sys-host", default=os.environ.get("EBABIL_ZMQ_BIND_HOST", "127.0.0.1"))
    ap.add_argument("--sys-portlar", default="5555,5560", help="SYS yayını dinlenecek portlar, virgülle (varsayılan: streamer.py 5555 + pluto_ed_scanner.py 5560)")
    ap.add_argument("--mavlink-host", default=None, help="mavlink_bridge.py'nin UAV konumu yayınladığı host (varsayılan: EBABIL_MAVLINK_HOST ya da 127.0.0.1)")
    ap.add_argument("--mavlink-port", type=int, default=5559)
    ap.add_argument("--csv", default=None, help="Ham örneklerin yazılacağı CSV dosyası (varsayılan: kalibrasyon_<freq>MHz_<tarih-saat>.csv)")
    args = ap.parse_args()

    # konum_istemcisi.py'deki UavKonumDinleyici -- gerçek İHA GPS konumu icin.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from konum_istemcisi import UavKonumDinleyici

    band_hz = args.freq_mhz * 1e6
    uav_konum = UavKonumDinleyici(host=args.mavlink_host, port=args.mavlink_port)
    kalibrasyon = KonumServisiKalibrasyon()

    ctx = zmq.Context()
    subler = []
    for port_str in args.sys_portlar.split(","):
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://{args.sys_host}:{port_str.strip()}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        sub.setsockopt(zmq.RCVTIMEO, 500)
        subler.append(sub)

    csv_yolu = args.csv or f"kalibrasyon_{args.freq_mhz:.3f}MHz_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    csv_dosya = open(csv_yolu, "w", newline="")
    csv_yazici = csv.writer(csv_dosya)
    csv_yazici.writerow(["zaman_unix", "uav_lat", "uav_lon", "uav_irtifa_m", "tx_lat", "tx_lon", "mesafe_m", "freq_mhz", "rssi_dbm"])
    csv_dosya.flush()

    print(f"[KAL] Vericinin bilinen konumu: {args.tx_lat:.7f},{args.tx_lon:.7f} @ {args.freq_mhz:.3f} MHz")
    print(f"[KAL] SYS dinleniyor: {[f'{args.sys_host}:{p.strip()}' for p in args.sys_portlar.split(',')]}")
    print(f"[KAL] Ham örnek CSV: {csv_yolu}")
    print("[KAL] GPS kilitlenmesi ve İHA hedefe/vericiye yaklaşması bekleniyor... (Ctrl+C ile bitir)")

    calisiyor = True

    def dur(_sig, _frame):
        nonlocal calisiyor
        calisiyor = False

    signal.signal(signal.SIGINT, dur)
    signal.signal(signal.SIGTERM, dur)

    ornek_sayisi = 0
    son_ornek_zamani = 0.0
    son_ornek_lat = None
    son_ornek_lon = None
    baslangic = time.monotonic()

    while calisiyor:
        if args.sure_s > 0 and (time.monotonic() - baslangic) >= args.sure_s:
            break

        satir = None
        for sub in subler:
            try:
                satir = sub.recv_string()
            except zmq.Again:
                continue
            except Exception:
                continue
            break
        if satir is None:
            continue

        ayristirilmis = sys_satirini_parcala(satir)
        if ayristirilmis is None:
            continue
        freq_mhz, rssi_dbm = ayristirilmis
        if abs(freq_mhz - args.freq_mhz) > args.freq_tol_mhz:
            continue  # kalibrasyon vericisinin bandı değil, baska bir tespit

        uav_lat, uav_lon, uav_irtifa = uav_konum.son_konum()
        if uav_lat is None:
            continue  # GPS henuz kilitlenmedi -- sahte konum ASLA uretilmez

        simdi = time.monotonic()
        if (simdi - son_ornek_zamani) < args.min_ornek_araligi_s:
            continue
        if son_ornek_lat is not None:
            hareket_m = mesafe_hesapla_m(son_ornek_lat, son_ornek_lon, uav_lat, uav_lon)
            if hareket_m < args.min_ornek_araligi_m:
                continue

        mesafe_m = mesafe_hesapla_m(args.tx_lat, args.tx_lon, uav_lat, uav_lon)
        if mesafe_m < args.min_mesafe_m:
            continue

        cevap = kalibrasyon.ekle(band_hz, mesafe_m, rssi_dbm)
        if cevap is None or not cevap.startswith("OK"):
            print(f"[KAL] UYARI: konum_servisi'ne KAL_EKLE gönderilemedi (cevap: {cevap})")
            continue

        son_ornek_zamani = simdi
        son_ornek_lat, son_ornek_lon = uav_lat, uav_lon
        ornek_sayisi += 1

        csv_yazici.writerow([f"{time.time():.3f}", f"{uav_lat:.7f}", f"{uav_lon:.7f}", f"{uav_irtifa:.1f}",
                              f"{args.tx_lat:.7f}", f"{args.tx_lon:.7f}", f"{mesafe_m:.2f}",
                              f"{freq_mhz:.3f}", f"{rssi_dbm:.2f}"])
        csv_dosya.flush()

        print(f"[KAL] örnek #{ornek_sayisi}: mesafe={mesafe_m:6.1f}m rssi={rssi_dbm:6.2f}dBm irtifa={uav_irtifa:5.1f}m ({cevap})")

    csv_dosya.close()
    uav_konum.durdur()

    print(f"\n[KAL] Toplam {ornek_sayisi} örnek toplandı. konum_servisi'nden KAL_HESAPLA isteniyor...")
    if ornek_sayisi < 3:
        print(f"[KAL] YETERSİZ ÖRNEK ({ornek_sayisi}/3) -- kalibrasyon hesaplanamadı. Uçuşu tekrarlayıp daha fazla örnek toplayın.")
        return

    sonuc = kalibrasyon.hesapla(band_hz)
    if sonuc is None:
        print("[KAL] HATA: konum_servisi'ne ulaşılamadı.")
        return
    if sonuc.startswith("HATA"):
        print(f"[KAL] Kalibrasyon başarısız: {sonuc}")
        return

    parcalar = sonuc.split(",")
    # OK,P0_dbm,n,rssi_residual_std,d0_m,ornek_sayisi
    print(f"[KAL] BAŞARILI -- P0={parcalar[1]} dBm, n={parcalar[2]}, rssi_residual_std={parcalar[3]} dB, "
          f"d0={parcalar[4]} m, örnek={parcalar[5]}")
    print(f"[KAL] Bu kalibrasyon konum_servisi'nin çalıştığı süre boyunca {args.freq_mhz:.3f} MHz "
          f"(±{args.freq_tol_mhz} MHz) bandındaki TÜM yeni hedeflerde otomatik kullanılacak.")
    print(f"[KAL] Ham örnekler kalıcı olarak kaydedildi: {csv_yolu}")


if __name__ == "__main__":
    main()
