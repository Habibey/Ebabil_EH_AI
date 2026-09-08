"""GUI'nin ET hedef-onayı zorunluluğunu atlayip Pluto'yu dogrudan tetikleyen
tek seferlik test scripti -- RTL-SDR + anten dogrulamasi icin (ED hic hedef
bulamiyorken, bilinen bir sinyali biz kendimiz uretip RTL'in yakalayip
yakalamadigina bakiyoruz). et_control.py CALISMIYORKEN calistir (ayni
Pluto'ya iki surec bagli olmasin).

Kullanim: python3 test_pluto_tx_manual.py <frekans_mhz> <sure_s> [kazanc_db]
  (ornek: python3 test_pluto_tx_manual.py 433.5 30 -10)
  kazanc_db verilmezse varsayilan -40 dB (PlutoTX'in baslangic degeri) kullanilir --
  0'a ne kadar yakinsa o kadar guclu (dikkat: cok guclu yayin, dinleyicileri
  bozabilir/mevzuat disi olabilir, sadece kisa sureli kontrollu test icin kullan).
"""
import sys
import time

sys.path.insert(0, ".")
from et_control import PlutoTX, handle_baslat

if __name__ == "__main__":
    freq_mhz = float(sys.argv[1]) if len(sys.argv) > 1 else 433.5
    duration_s = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    gain_db = float(sys.argv[3]) if len(sys.argv) > 3 else None

    tx = PlutoTX()
    tx.connect()
    if gain_db is not None:
        tx.set_gain(gain_db)
    handle_baslat(tx, "SUREKLI_KARISTIRMA", [freq_mhz], "TEKLI")
    print(f"[+] {freq_mhz} MHz'de {duration_s}s yayin yapiliyor... (RTL-SDR/ED ekranini izle)")
    try:
        time.sleep(duration_s)
    finally:
        tx.stop("SUREKLI_KARISTIRMA")
        print("[+] Yayin durduruldu.")
