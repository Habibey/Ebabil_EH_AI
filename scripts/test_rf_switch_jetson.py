"""
HMC241 (SP4T RF switch) test scripti -- Jetson.GPIO BOARD modu ile, ham
libgpiod chip/line yerine doğrudan fiziksel header pin numarası kullanır
(bkz. scripts/test_rf_switch.py -- RPi/libgpiod içindi, bu Jetson içindir).

NEDEN AYRI: Jetson'ın taşıyıcı kartı NVIDIA'nın resmi Developer Kit'i
DEĞİL (Jetson.GPIO import edilince "Carrier board is not from a Jetson
Developer Kit" uyarısı basıyor) -- bu yüzden ham gpiod chip/line numarasını
tahmin etmek yerine, doğrudan bilinen fiziksel pin numaralarını (A=pin11,
B=pin13, 2026-09-18'de bu Jetson'da multimetreyle doğrulandı) Jetson.GPIO'ya
verip test ettik.

Kullanım:
    python3 test_rf_switch_jetson.py [A_board_pin] [B_board_pin]
    örn:  python3 test_rf_switch_jetson.py 11 13   (varsayılan zaten bu)

1-4 girip Enter'a bas, script A/B pinlerini o porta göre (HMC241 doğruluk
tablosu: A=port&1, B=(port>>1)&1) ayarlar. Multimetre ile pin üzerindeki
gerilimi (GND=pin6'ya göre, 0V=LOW/3.3V=HIGH) ya da RF çıkışını (Pluto TX +
alıcı ile) doğrula. 'q' ile çık.
"""
import sys

import Jetson.GPIO as GPIO

PORT_ISIMLERI = {
    "1": "RF1 (144-433)",
    "2": "RF2 (868-915)",
    "3": "RF3 (GNSS-1.5G)",
    "4": "RF4 (2400-2483)",
}
PORT_AB = {"1": (0, 0), "2": (1, 0), "3": (0, 1), "4": (1, 1)}


def main():
    a_pin = int(sys.argv[1]) if len(sys.argv) > 1 else 11
    b_pin = int(sys.argv[2]) if len(sys.argv) > 2 else 13

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(a_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(b_pin, GPIO.OUT, initial=GPIO.LOW)
    print(f"[*] Hazır -- A=pin{a_pin}, B=pin{b_pin}")

    try:
        while True:
            secim = input("Port sec (1-4, q=cik): ").strip()
            if secim == "q":
                break
            if secim not in PORT_AB:
                continue
            a, b = PORT_AB[secim]
            GPIO.output(a_pin, GPIO.HIGH if a else GPIO.LOW)
            GPIO.output(b_pin, GPIO.HIGH if b else GPIO.LOW)
            print(f"-> {PORT_ISIMLERI[secim]}  (A=pin{a_pin}={a}  B=pin{b_pin}={b})")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
