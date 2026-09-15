#!/bin/bash
# ET RPi'de et_control.py'yi YARIŞMA GÜNÜ sabit IP planıyla başlatır -- bkz.
# CLAUDE.md. PC/Jetson/ET RPi kendi özel (internetsiz) Ethernet switch'ine şu
# sabit IP'lerle bağlı olmalı: PC=192.168.50.1, Jetson=192.168.50.2,
# ET RPi=192.168.50.3. DHCP/WiFi/.local'e bağımlı DEĞİL -- venue ağı ne olursa
# olsun bu üçü birbirini hep aynı adresten bulur.
#
# Kullanım: ./scripts/baslat_et_rpi.sh
# Farklı bir ağda (ör. bugünkü gibi ev/ofis testinde) çalıştırman gerekirse
# bu scripti KULLANMA, EBABIL_GUI_HOST=<pc_ip> python3 et_control.py ile elle
# çalıştır (bkz. Ebabil_EH_AI/CLAUDE.md).
set -e
cd "$(dirname "$0")/../src"
EBABIL_GUI_HOST=192.168.50.1 python3 et_control.py
