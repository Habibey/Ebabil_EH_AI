#!/bin/bash
# Jetson'da yer_istasyonu_koprusu'nu YARIŞMA GÜNÜ sabit IP planıyla başlatır --
# bkz. CLAUDE.md. PC/Jetson kendi özel (internetsiz) Ethernet switch'ine şu
# sabit IP'lerle bağlı olmalı: PC=192.168.50.1, Jetson=192.168.50.2 (ET RPi
# kaldırıldı, eskiden .3'tü -- artık switch'te sadece bu iki makine var).
# DHCP/WiFi/.local'e bağımlı DEĞİL -- venue ağı ne olursa olsun ikisi
# birbirini hep aynı adresten bulur.
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
#
# et_control.py DE BURADA başlatılıyor (2026-09-16 donanım degisikligi --
# yerde ayrı bir ET RPi YOK artık, Pluto TX doğrudan Jetson'a USB ile
# bağlı). ET RF anahtarının (HMC241) GPIO pinleri RPi'deki gibi
# EBABIL_ET_RF_SWITCH_CHIP/HATLAR ile veriliyor -- ama Jetson'ın kendi GPIO
# numaralandırması RPi'ninkiyle AYNI DEĞİL, pinler yeniden fiziksel olarak
# doğrulanmalı (bkz. scripts/test_rf_switch.py, aynı yöntemle Jetson'da da
# kullanılabilir). Vermezsen kod zaten "YAPILANDIRILMADI, anten sabit
# kalacak" diyip zararsızca devam ediyor.
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

pkill -9 -f "python3 et_control.py" 2>/dev/null || true

echo "[*] ai_servisi.py başlatılıyor (arka planda, port 5580)..."
(cd "$_BURASI/src" && python3 ai_servisi.py) &
AI_PID=$!

# TensorFlow modelinin yüklenmesi Jetson Nano'da 1sn'den çok daha uzun
# sürebiliyor (2026-09-18 saha testinde görüldü: sabit "sleep 1" yetmeyip
# yer_istasyonu_koprusu daha model hazır olmadan istek göndermeye başlıyor,
# "AI servisine ulasilamadi/zaman asimi" ile sonuçlanıyordu). Sabit bekleme
# yerine port 5580'in gerçekten açılmasını (en fazla 30sn) bekliyoruz.
echo "[*] AI modelinin yüklenmesi bekleniyor (port 5580, en fazla 30sn)..."
AI_HAZIR=0
for i in $(seq 1 60); do
    if ss -ltn 2>/dev/null | grep -q ':5580 '; then
        AI_HAZIR=1
        break
    fi
    sleep 0.5
done
if [ "$AI_HAZIR" = "1" ]; then
    echo "[*] ai_servisi.py hazır."
else
    echo "[!] ai_servisi.py 30sn içinde hazır olmadı -- AI sınıflandırma bu oturumda çalışmayabilir, devam ediliyor."
fi

echo "[*] et_control.py başlatılıyor (arka planda, Pluto TX)..."
# ET RF anahtarı (HMC241) GPIO pinleri -- 2026-09-18'de bu Jetson'da (ÖZEL/
# ÜÇÜNCÜ PARTİ taşıyıcı kart, resmi devkit DEĞİL) Jetson.GPIO BOARD modu +
# multimetre ile fiziksel olarak DOĞRULANDI: 5V=pin2, GND=pin6, A=pin11,
# B=pin13. Ham gpiod chip/line YERİNE Jetson.GPIO BOARD pin numarası
# kullanılıyor (bkz. et_control.py'deki PlutoEtRfAnahtari._init_jetson_gpio_board
# docstring'i) -- bu taşıyıcı kartta chip/line tahmini güvenilir değildi.
export EBABIL_ET_RF_SWITCH_BOARD_PINS=11,13
(cd "$_BURASI/src" && EBABIL_GUI_HOST=192.168.50.1 python3 et_control.py) &
ET_PID=$!

trap 'echo "[*] ai_servisi.py/et_control.py kapatılıyor..."; kill "$AI_PID" "$ET_PID" 2>/dev/null' INT TERM EXIT
sleep 1

cd "$_BURASI/yer_istasyonu_koprusu/build"
EBABIL_ARAYUZ_HOST=192.168.50.1 ./yer_istasyonu_koprusu
