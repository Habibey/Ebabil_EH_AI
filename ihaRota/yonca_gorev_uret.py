#!/usr/bin/env python3
"""
Spiral arama deseni uretici.
1km x 1km alan icin ArduPlane (sabit kanat) uyumlu QGC WPL110 (.waypoints)
gorev dosyasi uretir -- Mission Planner ve QGroundControl ile dogrudan uyumlu.

GECMIS (bu script'in 3. surumu -- onceki iki tasarim, testler/rota_degerlendirici.cpp
ile olculup elenmis durumda, bkz. yonKonum1905/testler/):

1) 4 yaprakli gul egrisi (N/E/S/W, r=R*cos(2*theta), R=500m) -- kare alanin
   KOSELERINE hic ulasmiyordu (tam 45 derecede r=0'a iniyor, R buyutmek de
   cozmuyor -- gercek ucus koordinatlariyla en yakin nokta 392.7m olcüldu).

2) 8 yaprakli gul egrisi (4 kenar N/E/S/W R=500m + 4 kose NE/SE/SW/NW
   R=707m) -- fiziksel kapsamayi duzeltti (koseler ~0m'ye indi) AMA
   testler/rota_degerlendirici.cpp ile 1km x 1km'in TAMAMI taranarak
   olculunce: (a) DF dogrulugu spiral'den daha kotu cikti (ort. hata 4.27m
   vs spiral 3.80m, en kotu durum 20.40m vs spiral 18.14m) VE (b) yaprak
   gecislerinde ~172-180 derecelik SERT DONUSLER icerdigi bulundu (ardisik
   iki bacak arasindaki aci hesaplanarak dogrulandi) -- sabit-kanat icin
   ciddi bir sorun, neredeyse tam bir U-donusu demek.

SONUC: Spiral (Arsimet tipi, r oranla dogrusal artan) her ikisinde de
kazandi -- en iyi DF dogrulugu VE en yumusak donusler (en keskin donus
sadece ~18 derece, 8 yaprağin 172 derecesine karsi). Toplam mesafe de
8 yapraktan (~11.7km) kisa (~9.1km, R=707m ile). Detay/sayisal kiyaslama:
yonKonum1905/testler/rota_degerlendirici.cpp cikti loglari.

ONEMLI NOT (ayni analizde bulundu, script kapsaminda COZULMEDI -- hangi
rota secilirse secilsin gecerli): Sadece ucus yolunun geometrik olarak
koseye yakin gecmesi, DF/konum kestirim sisteminin (EKF+PF) o bolgede
dogru sonuc verecegi anlamina gelmiyor. Saf EKF (parcacik filtresi
olmadan) test edilince, merkeze uzak/kose hedeflerde DOGRUSALLASTIRMA
TUZAGINA dusup ~450-527m hatali sonuc verdigi gozlemlendi -- bu rota
SEKLINDEN BAGIMSIZ bir EKF zaafiyeti. Asil koruma, yazilim tarafindaki
PF+EKF birlikte calismasindan geliyor (bkz.
yonKonum1905/konum_kestirimi/orkestrasyon/ ve
yonKonum1905/testler/pf_ekf_kurtarma_test.cpp). Yani bu script sadece
FIZIKSEL kapsamayi/rota kalitesini duzenliyor -- DF yazilimi tarafinin
PF+EKF ile calistigindan ayrica emin olun, sadece EKF ile calisirsa
kose/uzak hedeflerde ciddi hataya acik kalir.

Egri: Arsimet spirali -- r(oran) = oran * R, bearing(oran) = oran * tur_sayisi
* 360 derece (oran: 0'dan 1'e, merkezden disariya). Ilk nokta merkeze yakin
ve Kuzey yonunde baslar; TUR_SAYISI tam sayi oldugu icin son nokta da tam
Kuzeyde (R kadar uzaklikta) bitiyor -- boylece kalkis bacagi (Kuzey) ile
uyumlu, donussuz bir devamlilik saglaniyor.

SABIT KANAT NOTU: NAV_VTOL_TAKEOFF/LAND yerine NAV_TAKEOFF (22) ve
NAV_LAND (21) kullaniliyor. Pist/katapult kalkisi varsayiliyor.
Inis yaklasma yonu LAND_APPROACH_BEARING_DEG ile ayarlanir.
"""
import math
import os

# ---- PARAMETRELER (yarisma alani belli olunca guncellenecek) ----
HOME_LAT = 39.9250000   # TODO: gercek yarisma alani merkez enlemi
HOME_LON = 32.8369960   # TODO: gercek yarisma alani merkez boylami
CRUISE_SPEED_MS = 12.0  # sabit-kanat seyir hizi

# Irtifa artik SABIT degil, spiralin o andaki yaricapina gore degisiyor:
# spiral MERKEZDE (kucuk yaricap, "siki" donus) DUSUK irtifada baslar,
# disari dogru YUKSELEREK (yaricap buyudukce, donus yumusadikca) taban/
# hedef irtifaya (ALTITUDE_BASE_M = 30m) ULASIR -- yani ucak spiral
# boyunca surekli TIRMANIYOR, inmiyor.
# ONEMLI: 30m sabit kanat icin COK DUSUK bir AGL -- yarisma alanindaki
# gercek engel/arazi durumunu ve yerel irtifa siniflandirmalarini ayrica
# kontrol edin, bu sadece istenen degeri uyguluyor, guvenligini DOGRULAMIYOR.
# NOT: TAKEOFF_ALT_M (asagida, 50m) bu 15m'lik baslangictan YUKSEK --
# kalkis sonrasi spiralin ilk noktasina gecerken kisa bir inis olacak.
# Bu ayrica istenmedikce (soylenmedi) TAKEOFF_ALT_M'ye dokunulmadi.
ALTITUDE_BASE_M = 30.0              # disa dogru (yumusak donuste) ULASILAN taban/hedef irtifa (relative, AGL)
ALTITUDE_MERKEZ_DUSUS_M = 15.0      # en siki donuste (SPIRAL_MIN_YARICAP_M) taban irtifadan ne kadar DUSUK baslanacak
SPIRAL_YARICAP_M = 707.0    # spiralin son yaricapi -- alanin yarim kosegeni (500*sqrt(2)), koseleri kapsar
SPIRAL_TUR_SAYISI = 5        # spiralin kac tam tur atacagi
SPIRAL_NOKTA_PER_TUR = 40    # her tur icin waypoint sayisi (egri cozunurlugu)
SPIRAL_MIN_YARICAP_M = 100.0 # baslangic yaricapi -- sabit kanat min donus yaricapinin altina inmemek icin

# Kalkis parametreleri (NAV_TAKEOFF, komut 22)
TAKEOFF_ALT_M = 50.0         # kalkista hedef irtifa (piste gore relative, AGL)
TAKEOFF_PITCH_DEG = 15.0     # kalkis klapeto acisi (param1, derece)

# Inis parametreleri (NAV_LAND, komut 21)
# Inis noktasi home'dan LAND_APPROACH_DIST_M kadar LAND_APPROACH_BEARING_DEG
# yonunde -- uçak bu noktadan home'a dogru (karsi yonunde) yaklasarak iner.
LAND_APPROACH_DIST_M = 300.0
LAND_APPROACH_BEARING_DEG = 0.0  # 0=Kuzey: spiral son noktasiyla (Kuzey) tutarli

# Ardisik spiral noktalari birbirine asiri yakin dusebiliyor -- esikten yakin
# olanlar tek noktaya indirgeniyor (bkz. noktalari_temizle).
CAKISMA_ESIK_M = 2.0

R_EARTH = 6378137.0  # WGS84 semi-major axis (m)

def local_to_latlon(x_east_m, y_north_m, lat0, lon0):
    """Home merkezli ENU offseti (metre) -> lat/lon derece."""
    dlat = (y_north_m / R_EARTH) * (180.0 / math.pi)
    dlon = (x_east_m / (R_EARTH * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + dlat, lon0 + dlon

def spiral_noktalari(yaricap, tur_sayisi, nokta_per_tur, min_yaricap=0.0):
    """Arsimet spirali uzerinde (x_east, y_north, r) nokta listesi uretir --
    r de donduruluyor ki cagiran taraf (main()) o noktadaki donus "sikiligina"
    gore irtifa hesaplayabilsin (bkz. irtifa_hesapla).
    min_yaricap: baslangic yaricapi (sabit kanat icin min donus yaricapi)."""
    toplam_nokta = tur_sayisi * nokta_per_tur
    pts = []
    for i in range(toplam_nokta + 1):
        oran = i / toplam_nokta
        # r: min_yaricap'tan yaricap'a lineer
        r = min_yaricap + oran * (yaricap - min_yaricap)
        bearing_deg = oran * tur_sayisi * 360.0
        bearing_rad = math.radians(bearing_deg)
        x_east = r * math.sin(bearing_rad)
        y_north = r * math.cos(bearing_rad)
        pts.append((x_east, y_north, r))
    return noktalari_temizle(pts, CAKISMA_ESIK_M)

def noktalari_temizle(pts, esik_m):
    """Art arda gelen, birbirine esik_m'den yakin noktalari tek noktaya indirger
    (r'yi de -- ilk noktanin r'si korunur, aradaki kucuk fark irtifa gecisini
    etkilemeyecek kadar onemsiz)."""
    if not pts:
        return pts
    temiz = [pts[0]]
    for (x, y, r) in pts[1:]:
        onceki_x, onceki_y, _ = temiz[-1]
        if math.hypot(x - onceki_x, y - onceki_y) >= esik_m:
            temiz.append((x, y, r))
    return temiz

def irtifa_hesapla(r, r_min, r_max):
    """Yaricapa gore irtifa: r kucukken (merkez, siki donus) taban irtifadan
    dusuk baslar; r r_max'a yaklastikca (disari dogru, yumusak donus)
    LINEER OLARAK YUKSELIR, r_max'ta taban/hedef irtifaya (ALTITUDE_BASE_M)
    ulasir -- ucak spiral boyunca surekli tirmaniyor, inmiyor."""
    if r_max <= r_min:
        return ALTITUDE_BASE_M
    oran = (r - r_min) / (r_max - r_min)  # 0 (merkez) -> 1 (dis kenar)
    oran = max(0.0, min(1.0, oran))
    return ALTITUDE_BASE_M - (1.0 - oran) * ALTITUDE_MERKEZ_DUSUS_M

def qgc_wpl_satiri(seq, current, frame, command, p1, p2, p3, p4, lat, lon, alt, autocontinue=1):
    return f"{seq}\t{current}\t{frame}\t{command}\t{p1}\t{p2}\t{p3}\t{p4}\t{lat:.7f}\t{lon:.7f}\t{alt:.2f}\t{autocontinue}\n"

def main():
    lines = ["QGC WPL 110\n"]
    seq = 0

    # seq 0: home placeholder
    lines.append(qgc_wpl_satiri(seq, 1, 0, 16, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0.0))
    seq += 1

    # DO_CHANGE_SPEED (178): airspeed
    lines.append(qgc_wpl_satiri(seq, 0, 3, 178, 0, CRUISE_SPEED_MS, -1, 0, 0, 0, 0))
    seq += 1

    # NAV_TAKEOFF (22): sabit kanat kalkisi
    # param1=minimum pitch, param4=yaw (NaN=mevcut yaw), lat/lon=hedef konum
    lines.append(qgc_wpl_satiri(seq, 0, 3, 22, TAKEOFF_PITCH_DEG, 0, 0, float('nan'),
                                HOME_LAT, HOME_LON, TAKEOFF_ALT_M))
    seq += 1

    # Spiral deseni: NAV_WAYPOINT (16) dizisi -- irtifa her noktada o
    # noktadaki yaricapa (donus sikiligina) gore ayri ayri hesaplaniyor.
    for (x_e, y_n, r) in spiral_noktalari(SPIRAL_YARICAP_M, SPIRAL_TUR_SAYISI, SPIRAL_NOKTA_PER_TUR, SPIRAL_MIN_YARICAP_M):
        lat, lon = local_to_latlon(x_e, y_n, HOME_LAT, HOME_LON)
        irtifa = irtifa_hesapla(r, SPIRAL_MIN_YARICAP_M, SPIRAL_YARICAP_M)
        lines.append(qgc_wpl_satiri(seq, 0, 3, 16, 0, 0, 0, 0, lat, lon, irtifa))
        seq += 1

    # Inis yaklasma noktasi: home'dan LAND_APPROACH_DIST_M kadar uzakta
    # Ucak bu noktadan home'a dogru duz bir yaklasma bacagi ucar.
    yaklasma_bearing_rad = math.radians(LAND_APPROACH_BEARING_DEG)
    yaklasma_x = LAND_APPROACH_DIST_M * math.sin(yaklasma_bearing_rad)
    yaklasma_y = LAND_APPROACH_DIST_M * math.cos(yaklasma_bearing_rad)
    yaklasma_lat, yaklasma_lon = local_to_latlon(yaklasma_x, yaklasma_y, HOME_LAT, HOME_LON)
    lines.append(qgc_wpl_satiri(seq, 0, 3, 16, 0, 0, 0, 0,
                                yaklasma_lat, yaklasma_lon, ALTITUDE_BASE_M))
    seq += 1

    # NAV_LAND (21): sabit kanat inisi -- home noktasinda
    # param1=abort alt, param4=yaw (NaN=mevcut)
    lines.append(qgc_wpl_satiri(seq, 0, 3, 21, 0, 0, 0, float('nan'),
                                HOME_LAT, HOME_LON, 0.0))
    seq += 1

    script_dizini = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(script_dizini, "yonca_gorevi.waypoints")
    with open(out_path, "w") as f:
        f.writelines(lines)

    print(f"Yazildi: {out_path}")
    print(f"Toplam waypoint satiri (home haric): {seq - 1}")
    print(f"Spiral yaricapi: {SPIRAL_YARICAP_M} m, tur sayisi: {SPIRAL_TUR_SAYISI}")
    print(f"Irtifa: {ALTITUDE_BASE_M - ALTITUDE_MERKEZ_DUSUS_M}-{ALTITUDE_BASE_M} m "
          f"(merkez/siki donus -> dis/yumusak donus, tirmanarak), hiz: {CRUISE_SPEED_MS} m/s")
    print(f"Kalkis irtifasi: {TAKEOFF_ALT_M} m, inis yaklasma: {LAND_APPROACH_DIST_M} m")

if __name__ == "__main__":
    main()
