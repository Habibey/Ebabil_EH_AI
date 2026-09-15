"""
HMC241 (SP4T RF switch) test scripti -- et_control.py'den BAĞIMSIZ, sadece
A/B GPIO pinlerinin doğru role/porta gittiğini elle doğrulamak için.

Kullanım:
    python3 test_rf_switch.py <gpiochip_yolu> <A_hat_no> <B_hat_no>
    örn:  python3 test_rf_switch.py /dev/gpiochip0 17 27

Çalıştırınca 1-4 girip Enter'a bas, script A/B pinlerini o porta göre
(HMC241 doğruluk tablosu: A=port&1, B=(port>>1)&1) ayarlar, hangi port/hangi
A-B değeri seçtiğini ekrana basar. Sen de multimetre/süreklilik testiyle ya
da anten ucuna RF kaynağı verip spektrum analizör/alıcı ile RFC'nin gerçekten
o RF çıkışına gittiğini doğrularsın. 'q' ile çık.

NOT -- libgpiod v1/v2 API farkı: Debian/Raspberry Pi OS sürümüne göre kurulu
Python gpiod paketi ya eski (v1, Chip.get_line()/Line.request()) ya da yeni
(v2, gpiod.request_lines()) API'yi sağlıyor -- ikisi birbirine hiç benzemiyor.
Bu script v2'yi (gpiod>=1.6/2.x, `python3 -c "import gpiod; print(gpiod.__version__)"`
ile kontrol edilebilir) hedefliyor çünkü ET RPi'de bu kurulu çıktı.
"""
import sys

import gpiod
from gpiod.line import Direction, Value

PORT_ISIMLERI = {1: "RF1 (144-433)", 2: "RF2 (868-915)", 3: "RF3 (GNSS-1.5G)", 4: "RF4 (2400-2483)"}


def main():
    if len(sys.argv) != 4:
        print(f"Kullanım: python3 {sys.argv[0]} <gpiochip_yolu> <A_hat_no> <B_hat_no>")
        print(f"örn:      python3 {sys.argv[0]} /dev/gpiochip0 17 27")
        sys.exit(1)

    chip_yolu, a_no, b_no = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

    req = gpiod.request_lines(
        chip_yolu,
        consumer="rf_switch_test",
        config={
            a_no: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
            b_no: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
        },
    )
    print(f"[+] Hazır -- chip={chip_yolu} A=hat{a_no} B=hat{b_no}")

    try:
        while True:
            secim = input("\nPort seç (1-4, q=çık): ").strip().lower()
            if secim == "q":
                break
            if secim not in ("1", "2", "3", "4"):
                print("Geçersiz, 1-4 arası bir sayı ya da q gir.")
                continue
            port = int(secim) - 1  # 0-3'e çevir (ET_ANTEN_BANDLARI ile aynı)
            a_bit, b_bit = port & 1, (port >> 1) & 1
            req.set_values({
                a_no: Value.ACTIVE if a_bit else Value.INACTIVE,
                b_no: Value.ACTIVE if b_bit else Value.INACTIVE,
            })
            print(f"[*] {PORT_ISIMLERI[port + 1]} seçildi -- A={a_bit} B={b_bit}")
    finally:
        req.set_values({a_no: Value.INACTIVE, b_no: Value.INACTIVE})
        req.release()
        print("[*] Çıkıldı, A/B sıfırlandı.")


if __name__ == "__main__":
    main()
