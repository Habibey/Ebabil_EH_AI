"""
HMC241 (SP4T RF switch) test scripti -- et_control.py'den BAĞIMSIZ, sadece
A/B GPIO pinlerinin doğru role/porta gittiğini elle doğrulamak için.

Kullanım:
    python3 test_rf_switch.py <gpiochip_adi> <A_hat_no> <B_hat_no>
    örn:  python3 test_rf_switch.py gpiochip0 5 6

Çalıştırınca 1-4 girip Enter'a bas, script A/B pinlerini o porta göre
(HMC241 doğruluk tablosu: A=port&1, B=(port>>1)&1) ayarlar, hangi port/hangi
A-B değeri seçtiğini ekrana basar. Sen de multimetre/süreklilik testiyle ya
da anten ucuna RF kaynağı verip spektrum analizör/alıcı ile RFC'nin gerçekten
o RF çıkışına gittiğini doğrularsın. 'q' ile çık.
"""
import sys

import gpiod

PORT_ISIMLERI = {1: "RF1 (144-433)", 2: "RF2 (868-915)", 3: "RF3 (GNSS-1.5G)", 4: "RF4 (2400-2483)"}


def main():
    if len(sys.argv) != 4:
        print(f"Kullanım: python3 {sys.argv[0]} <gpiochip_adi> <A_hat_no> <B_hat_no>")
        sys.exit(1)

    chip_adi, a_no, b_no = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

    chip = gpiod.Chip(chip_adi)
    a_hatti = chip.get_line(a_no)
    b_hatti = chip.get_line(b_no)
    a_hatti.request(consumer="rf_switch_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    b_hatti.request(consumer="rf_switch_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    print(f"[+] Hazır -- chip={chip_adi} A=hat{a_no} B=hat{b_no}")

    try:
        while True:
            secim = input("\nPort seç (1-4, q=çık): ").strip().lower()
            if secim == "q":
                break
            if secim not in ("1", "2", "3", "4"):
                print("Geçersiz, 1-4 arası bir sayı ya da q gir.")
                continue
            port = int(secim) - 1  # 0-3'e çevir (ET_ANTEN_BANDLARI ile aynı)
            a_val = port & 1
            b_val = (port >> 1) & 1
            a_hatti.set_value(a_val)
            b_hatti.set_value(b_val)
            print(f"[*] {PORT_ISIMLERI[port + 1]} seçildi -- A={a_val} B={b_val}")
    finally:
        a_hatti.set_value(0)
        b_hatti.set_value(0)
        print("[*] Çıkıldı, A/B sıfırlandı.")


if __name__ == "__main__":
    main()
