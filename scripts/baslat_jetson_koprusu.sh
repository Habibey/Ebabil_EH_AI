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
set -e
cd "$(dirname "$0")/../yer_istasyonu_koprusu/build"
EBABIL_ARAYUZ_HOST=192.168.50.1 ./yer_istasyonu_koprusu
