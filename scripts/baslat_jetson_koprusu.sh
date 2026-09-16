#!/bin/bash
# Jetson'da yer_istasyonu_koprusu'nu YARIŞMA GÜNÜ sabit IP planıyla başlatır --
# bkz. CLAUDE.md. PC/Jetson/ET RPi kendi özel (internetsiz) Ethernet switch'ine
# şu sabit IP'lerle bağlı olmalı: PC=192.168.50.1, Jetson=192.168.50.2,
# ET RPi=192.168.50.3. DHCP/WiFi/.local'e bağımlı DEĞİL -- venue ağı ne olursa
# olsun bu üçü birbirini hep aynı adresten bulur.
#
# Kullanım: ./scripts/baslat_jetson_koprusu.sh
# Farklı bir ağda (ör. bugünkü gibi ev/ofis testinde) çalıştırman gerekirse
# bu scripti KULLANMA, EBABIL_ARAYUZ_HOST=<pc_ip> ./yer_istasyonu_koprusu ile
# elle çalıştır (bkz. Ebabil_EH_AI/CLAUDE.md).
#
# ai_servisi.py'yi de BURADA başlatıyoruz (2026-09-16 öncesi unutulmuştu) --
# o çalışmadan yer_istasyonu_koprusu'nun IQ isteklerine cevap veren kimse
# olmuyor, GUI'deki "Modülasyon"/"Analog-Sayısal" alanları sessizce hep "-"
# kalıyor (hata da vermiyor, best-effort). models/ klasörü .gitignore'da --
# bu Jetson'da manuel taşınmış olmalı, yoksa ai_servisi.py model yüklerken
# hata verip çıkar (aşağıdaki arka plan sürecinin logunu kontrol et).
set -e
_BURASI="$(cd "$(dirname "$0")/.." && pwd)"

# Bu gece defalarca yaşandı: script durdurulup hemen yeniden başlatılınca
# eski bir yer_istasyonu_koprusu/ai_servisi.py arka planda kalmış olabiliyor,
# "Address already in use" ile başlamıyordu -- başlamadan önce kendimiz
# zorla temizliyoruz.
port_temizle() {
    local port="$1"
    local pid
    pid="$(sudo ss -ltnp 2>/dev/null | awk -v p=":$port\$" '$4 ~ p {print $0}' | grep -oP 'pid=\K[0-9]+' | head -1)"
    if [ -n "$pid" ]; then
        echo "[*] Port $port zaten kullanımda (PID $pid), temizleniyor..."
        sudo kill -9 "$pid" 2>/dev/null || true
        sleep 0.3
    fi
}
for p in 5555 5556 5559 5580; do
    port_temizle "$p"
done

echo "[*] ai_servisi.py başlatılıyor (arka planda, port 5580)..."
(cd "$_BURASI/src" && python3 ai_servisi.py) &
AI_PID=$!
trap 'echo "[*] ai_servisi.py kapatılıyor..."; kill "$AI_PID" 2>/dev/null' INT TERM EXIT
sleep 1

cd "$_BURASI/yer_istasyonu_koprusu/build"
EBABIL_ARAYUZ_HOST=192.168.50.1 ./yer_istasyonu_koprusu
