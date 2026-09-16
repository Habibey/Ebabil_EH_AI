#!/bin/bash
# Iha RPi'deki TUM sureçleri tek komutla, dogru sirada baslatir:
#   mavlink_bridge.py -> konum_servisi -> streamer_watchdog.py (->streamer.py)
#   -> seri_telemetri_koprusu.py
#
# NEDEN BU DORDU BIRLIKTE: konum_servisi ve streamer_watchdog.py'nin ikisi de
# bu gece (2026-09-16) elle baslatilmadigi icin eksik kaldi -- konum_servisi
# calismadan Yon Bulma (madde 5.1.4/5.1.5) hic veri uretmiyor (best-effort,
# hata da vermiyor, sessizce bos kaliyor), streamer.py da watchdog OLMADAN
# dogrudan calistirilirsa RTL-SDR/libusb segfault'unda (bkz. CLAUDE.md
# "Bilinen tuhafliklar") bir daha kendiliginden ayaga kalkmiyor. Bu script
# ikisini de unutmamak icin var.
#
# Kullanim: ./scripts/baslat_iha_rpi.sh
# Ctrl+C ile hepsi birden (trap ile) duzgunce kapatilir.
#
# Fiziksel port/adres varsayimlari -- KENDI DONANIMINA GORE DEGISTIR (asagidaki
# export satirlarini duzenle, ya da calistirmadan once ortam degiskeni olarak
# elle ver):
#   EBABIL_MAVLINK_PORT   -- Matek'in bagli oldugu port (bu saha testinde
#                            /dev/serial0 -- UART pinleri, USB DEGIL)
#   EBABIL_TELEMETRI_PORT -- 915MHz radyonun bagli oldugu port (varsayilan
#                            seri_telemetri_koprusu.py'de /dev/ttyUSB0)
#   EBABIL_DF_REF_LAT/LON -- yarisma alaninin GERCEK referans noktasi.
#                            ELLE VERMENE GEREK YOK -- asagida mavlink_bridge
#                            baslar baslamaz Iha'nin kendi GERCEK GPS fix'i
#                            beklenip (df_referans_al.py) otomatik alinir.
#                            Yarisma yeri onceden kesin bilinmiyor (bkz. proje
#                            notlari) -- bu yuzden sabit bir deger yerine
#                            sahada gercek GPS'ten okumak cok daha guvenilir.
#                            GPS fix hic gelmezse (EBABIL_DF_REF_WAIT_S,
#                            varsayilan 30sn) asagidaki yer tutucuya doner.
set -e
_BURASI="$(cd "$(dirname "$0")/.." && pwd)"
cd "$_BURASI"

export EBABIL_MAVLINK_PORT="${EBABIL_MAVLINK_PORT:-/dev/serial0}"
export EBABIL_TELEMETRI_PORT="${EBABIL_TELEMETRI_PORT:-/dev/ttyUSB0}"
export EBABIL_GUI_HOST="127.0.0.1"  # streamer.py bu makinedeki seri_telemetri_koprusu'nu "GUI" sanacak
export EBABIL_DF_REF_LAT="${EBABIL_DF_REF_LAT:-39.9250000}"
export EBABIL_DF_REF_LON="${EBABIL_DF_REF_LON:-32.8369960}"

KONUM_SERVISI_BIN="$_BURASI/konum_servisi/build/konum_servisi"

PIDLER=()
trap 'echo; echo "[*] Kapatiliyor..."; kill "${PIDLER[@]}" 2>/dev/null; wait 2>/dev/null' INT TERM EXIT

echo "[1/4] mavlink_bridge.py baslatiliyor (port=$EBABIL_MAVLINK_PORT)..."
python3 src/mavlink_bridge.py &
PIDLER+=("$!")
sleep 1

if [ -x "$KONUM_SERVISI_BIN" ]; then
    echo "[*] DF referansi icin gercek GPS fix'i bekleniyor (en fazla ${EBABIL_DF_REF_WAIT_S:-30}sn)..."
    GERCEK_REF="$(python3 "$_BURASI/scripts/df_referans_al.py" 2>/dev/null || true)"
    if [ -n "$GERCEK_REF" ]; then
        export EBABIL_DF_REF_LAT="${GERCEK_REF%% *}"
        export EBABIL_DF_REF_LON="${GERCEK_REF##* }"
        echo "[*] DF referansi GERCEK GPS'ten alindi: $EBABIL_DF_REF_LAT, $EBABIL_DF_REF_LON"
    else
        echo "[!] GPS fix alinamadi (ic mekan/uydu yok?) -- YER TUTUCU referans kullanilacak: $EBABIL_DF_REF_LAT, $EBABIL_DF_REF_LON (YANLIS OLABILIR, DF sonuclari anlamsiz cikabilir)"
    fi

    echo "[2/4] konum_servisi baslatiliyor (DF ref: $EBABIL_DF_REF_LAT,$EBABIL_DF_REF_LON)..."
    "$KONUM_SERVISI_BIN" &
    PIDLER+=("$!")
    sleep 1
else
    echo "[2/4] UYARI: konum_servisi derlenmemis bulunamadi ($KONUM_SERVISI_BIN) -- Yon Bulma (DF) bu oturumda CALISMAYACAK."
    echo "       Derlemek icin: cd konum_servisi && mkdir -p build && cd build && cmake .. && make"
fi

echo "[3/4] streamer_watchdog.py baslatiliyor (RTL-SDR, gozculu)..."
python3 src/streamer_watchdog.py &
PIDLER+=("$!")
sleep 1

echo "[4/4] seri_telemetri_koprusu.py baslatiliyor (radyo=$EBABIL_TELEMETRI_PORT)..."
python3 src/seri_telemetri_koprusu.py --port "$EBABIL_TELEMETRI_PORT" &
PIDLER+=("$!")

echo
echo "[*] Hepsi calisiyor (PID: ${PIDLER[*]}). Cikmak icin Ctrl+C."
wait
