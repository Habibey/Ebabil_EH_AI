# Ebabil EH AI — Proje Bağlamı

Bu, TEKNOFEST 2026 Elektronik Harp (EH) Yarışması için geliştirilen sistemin
**backend (Python)** deposu. Şartname: Elektronik Destek (ED — sinyal tespiti,
parametre çıkarımı, dinleme, yön bulma) + Elektronik Taarruz (ET — karıştırma,
aldatma). Frekanslar yarışmada ÖNCEDEN bilinmiyor, sistem kendisi bulmalı.

## Mimari

- `src/streamer.py` — RTL-SDR ile ED: çoklu-bant otomatik tarama (KTR bant
  tablosu: 143-145/430-440/868-870 MHz), OS-CFAR tespiti, hedef takibi
  (`sdr_common.TargetTracker`), AI sınıflandırma (`predict.py`), DİNLE
  (gerçek AM/FM demodülasyon, `demod.py`). Port 5555 (SYS/SPEC), 5556 (AI).
- `src/pluto_ed_scanner.py` — İKİNCİ bir PlutoSDR'ı RX olarak kullanıp AYNI
  mimariyle ED yapar (RTL-SDR'ın ulaşamadığı 2400-2483 MHz dahil). Port 5560
  (SYS/SPEC), 5561 (AI). Hedef id'leri `PHEDEF-N` (streamer.py'nin
  `HEDEF-N`'iyle karışmasın diye).
- `src/et_control.py` — Pluto TX ile ET: sürekli/arabakışlı karıştırma
  (`ARA_BAKISLI_KARISTIRMA` aynı Pluto'yu TX/RX zaman paylaşımlı kullanır),
  analog telsiz aldatma (KAYIT_TEKRAR / PIPER_TTS), GNSS aldatma.
- `src/mavlink_bridge.py` — Matek/ArduPilot telemetrisini ZMQ'ya köprüler
  (port 5559).
- `src/sdr_common.py` — İKİ backend'in de PAYLAŞTIĞI, donanımdan bağımsız
  kod: FFT/binleme, OS-CFAR, `TargetTracker`, `pick_target()`, `TunedSdr`
  (gereksiz retune'u önbellekleyen sarmalayıcı).
- `src/predict.py` — TEKNOFEST modeli (`models/teknofest_model_v5_finetuned.keras`
  ya da TFLite fallback) ile modülasyon sınıflandırma. `models/`, `data/`
  klasörleri `.gitignore`'da -- GİT'TE YOK, manuel taşınması gerekiyor.
- `src/streamer_watchdog.py` — streamer.py donarsa/çökerse (RTL-SDR/libusb
  segfault -- bkz. "Bilinen tuhaflıklar") otomatik yeniden başlatan gözcü.
  GUI'den "GOZCU,YENIDEN_BASLAT" (port 5557) ile de anında tetiklenebilir.
- `src/konum_istemcisi.py` — yön bulma/konum kestirimi (madde 5.1.4/5.1.5)
  için `konum_servisi`'ne (C++, aşağıda) ZMQ REQ ile bağlanan istemci.
  `streamer.py`/`pluto_ed_scanner.py` ORTAK kullanıyor.
- `src/tespit_kaydedici.py` — her SYS güncellemesini zaman damgasıyla
  `data/tespit_gunlugu/*.csv`'ye ekleyen sürekli loglama.
- `src/sayisal_cozucu.py` — DİNLE sırasında sayısal (dijital) telsiz decode
  (madde 5.1.3, opsiyonel) -- `multimon-ng`'yi alt süreç olarak kullanır,
  kurulu değilse devre dışı kalır (kod etkilenmez).
- `src/ai_servisi.py` — CRNN modelini (predict.py) ZMQ REQ/REP ile
  Jetson'daki C++ köprüsüne (`yer_istasyonu_koprusu`) açan servis.
- `src/seri_telemetri_koprusu.py` — İHA'daki (RPi) tarafta 915MHz radyonun
  TEK sahibi: yerel ZMQ'daki (streamer.py/pluto_ed_scanner.py/mavlink_bridge.py)
  satırları radyoya yazar, radyodan gelen komutları yerel 5557'den geri
  yayınlar.
- `konum_servisi/` (C++, vendor edildi) — Parçacık Filtresi + EKF matematiği,
  `src/konum_istemcisi.py`'ye ZMQ REQ/REP (port 5570) ile açık.
- `yer_istasyonu_koprusu/` (C++, vendor edildi) — Jetson'da çalışan, radyo
  ile GUI'nin ZMQ portları (5555/5556/5559) arasındaki köprü. Format-agnostik
  (satırları hiç yorumlamadan aktarır) -- RPi tarafının Python olması bunu
  ETKİLEMEZ, C++'ta kalmaya devam ediyor.

## GUI AYRI BİR REPO'DA

Arayüz (Qt6/C++) bu repoda DEĞİL: **https://github.com/Habibey/GUI_QtCreator**
ZMQ ile bu backend'lere bağlanır (portlar: 5555/5556/5557/5559/5560/5561).
`gui/` klasörü buradaki repoda varsa bile ESKİ/TERK EDİLMİŞ bir kopyadır,
`.gitignore`'da, dikkate alma.

## TAMAMLANDI: Windows'tan Ubuntu'ya taşıma

Proje Windows'ta (RTL-SDR + 2x PlutoSDR ile) geliştirildi ve test edildi,
sonra **Ubuntu'ya (bedirhan-EXCALIBUR-G870) taşındı ve orada uçtan uca
DOĞRULANDI** (backend + GUI, gerçek RTL-SDR Blog V4 + 1x PlutoSDR + Matek
telemetri kartıyla).

Yapılanlar:
- Bu repo ve GUI_QtCreator Ubuntu'ya `git clone` ile çekildi.
- RTL-SDR/Pluto için udev/grup izni: `sudo usermod -aG plugdev,dialout $USER`
  yapıldı (çıkış-giriş sonrası aktif oldu, `groups` ile doğrulandı).
- **Python sürümü sorunu**: bu Ubuntu'da tek kurulu Python 3.14 idi,
  TensorFlow'un pip'te henüz 3.14 wheel'i yok. Çözüm: deadsnakes PPA ile
  `python3.12` kuruldu, venv ondan oluşturuldu (`python3.12 -m venv venv`).
  Jetson/RPi'de muhtemelen bu sorun YOK (onlarda zaten eski Python geliyor,
  bkz. aşağıdaki Jetson/RPi bölümü).
- venv'e bağımlılıklar kuruldu: pyzmq, numpy, scipy, tensorflow, pyrtlsdr,
  pyadi-iio, sounddevice, pymavlink, **+ pyserial** (CLAUDE.md'nin önceki
  sürümünde unutulmuştu, `mavlink_bridge.py`/pymavlink seri port için
  gerekiyor).
- `models/` ve `data/aldatma_sesleri/` (gitignore'da oldukları için) Google
  Drive üzerinden zip'lenip manuel taşındı.
- **pyrtlsdr / Ubuntu librtlsdr uyumsuzluğu (ÖNEMLİ, kalıcı DEĞİL)**: Ubuntu
  apt'teki `librtlsdr0` (osmocom mainline 2.0.2) `rtlsdr_set_dithering` ve
  gpio fonksiyonlarını içermiyor (sadece rtlsdrblog fork'unda var), ama
  pyrtlsdr 0.5.0 bunları hem import anında bağlamaya hem de HER `RtlSdr()`
  açılışında çağırmaya çalışıyor -- ikisi de `AttributeError`/donanımla
  gerçek çalıştırmada patlıyordu. Çözüm: venv içindeki
  `site-packages/rtlsdr/librtlsdr.py` (sembol bağlama) ve `rtlsdr.py`
  (`open()` içindeki `rtlsdr_set_dithering` çağrısı) sembol yoksa sessizce
  atlayacak şekilde YAMALANDI -- proje bu fonksiyonları zaten hiç
  kullanmıyor. **BU YAMA venv İÇİNDE, GİT'E COMMIT'LENMEDİ** -- venv silinip
  yeniden kurulursa (`pip install --force-reinstall pyrtlsdr` dahil) tekrar
  uygulanması gerekir, yoksa gerçek RTL-SDR açılışı yine patlar. Kalıcı/temiz
  çözüm rtlsdrblog'un librtlsdr fork'unu kaynaktan derleyip kurmak olur ama
  bu denenmedi (sudo + build gerektirir).
- GUI_QtCreator'daki `CMakeLists.txt`, Linux'ta derlenebilsin diye düzeltildi:
  vcpkg/CONFIG tabanlı ZeroMQ bulma SADECE Windows'ta (`if(WIN32)`), Linux'ta
  `pkg_check_modules` ile `libzmq3-dev` (apt) üzerinden buluyor. cppzmq
  (`zmq.hpp`) Ubuntu'da paket olarak bulunamadı, GitHub'dan (v4.11.0)
  `/usr/local/include`'a manuel indirildi. GUI Ubuntu'da BAŞARIYLA DERLENDİ.
- **Uçtan uca doğrulama YAPILDI**: `streamer.py` (RTL-SDR Blog V4) model
  yükleyip tarama başlattı, port 5555/5556'dan gerçek SPEC verisi aktı;
  `et_control.py` PlutoSDR TX'e (192.168.2.1) bağlandı; `mavlink_bridge.py`
  Matek karttan (/dev/ttyACM0) heartbeat alıp port 5559'da yayına başladı;
  `~/GUI_QtCreator/build/EHARPP` hepsine ZMQ üzerinden bağlandı ("BAĞLI"
  yeşil, ED modunda hedef tespiti + waterfall, ET modunda karıştırma/aldatma
  paneli ekran görüntüsüyle doğrulandı). `pluto_ed_scanner.py` test
  EDİLEMEDİ -- ikinci bir PlutoSDR (192.168.3.1 bekliyor) henüz yok, elde
  tek Pluto var.
- Windows'ta yaşanan RTL-SDR/libusb donma sorunu bu testte GÖRÜLMEDİ ama
  kısa süreli testti, uzun süreli kararlılık henüz kanıtlanmadı.

## KARAR (2026-09-14, yarışmaya 3 gün kala): Python KAZANDI, RPi'de C++ DEĞİL

Ekibin ayrı bir C++ reposu vardı (`ebabil-eh-backend`, GitHub'da
`billgatoss/ebabil-eh-backend` -- İHA/RPi tarafında `parametreCikarimi/ebabil_sdr`
adıyla streamer.py'nin C++ portu). Karşılaştırıldı, **Python'da devam
kararı alındı**:
- Python (bu repo) UÇTAN UCA test edilmişti (gerçek donanım), C++ tarafı
  değildi.
- RPi performans endişesi (C++'ın asıl gerekçesiydi) sanıldığı kadar güçlü
  değil -- en ağır matematik (PF/EKF) zaten C++'ta (`konum_servisi`),
  OS-CFAR/FFT zaten numpy (C hızında) vektörize.
- RTL-SDR segfault riski (aşağıda) `librtlsdr`'da, Python'a özgü değil --
  C++ tarafı da aynı riski taşırdı.
- 3 günde en hızlı iterasyon Python'da.

**SONUÇ**: `ebabil_sdr` (C++, RPi portu) ve `arayuz` (GUI_QtCreator'ın eski
çatalı) ARTIK KULLANILMIYOR/arşivde. Sadece `konum_servisi` (PF/EKF) ve
`yer_istasyonu_koprusu` (radyo<->ZMQ köprüsü) -- ikisi de format-agnostik/
donanımdan bağımsız C++ altyapı, RPi'nin dili değişse de aynı kalıyorlar --
bu repoya vendor edildi (`konum_servisi/`, `yer_istasyonu_koprusu/`).

## GERÇEK MİMARİ: İHA (RPi) <-915MHz radyo-> Jetson <-Ethernet-> PC

```
İHA üzerinde (Raspberry Pi 4/5):
  RTL-SDR (144-148/430-440/863-870, RF anahtarlı) ─┐
  PlutoSDR (2.4G/5.8G, tek anten) ──────────────────┼─> streamer.py + pluto_ed_scanner.py
  Matek FC + Here3 GPS ──────────────────────────────> mavlink_bridge.py
                                                            │ (yerel ZMQ: 5555/5556/5559/5560/5561)
                                                            v
                                              seri_telemetri_koprusu.py
                                                            │ (915MHz seri radyo, TEK hat)
                                                            v
Yerde (Jetson Nano):                        yer_istasyonu_koprusu (C++)
                                                            │ (ZMQ 5555/5556/5559, Ethernet)
                                                            v
PC/Laptop:                                          GUI_QtCreator

ET (bağımsız, ayrı RPi + Pluto TX):
  PC ──ZMQ 5557──> et_control.py ──> Pluto TX ──RF anahtarı──> 144-433/868-915/GNSS-1.5G Yagi+PA
```

`ai_servisi.py`, Jetson'da `yer_istasyonu_koprusu`'nun yanında AYRI bir
Python süreci olarak çalışması PLANLANMIŞTI (REQ/REP, port 5580) -- ama
streamer.py zaten `predict.py`'yi DOĞRUDAN (aynı süreçte, RPi'de) çağırıyor,
bu yüzden **AI sonucu artık düz metin olarak diğer satırlarla (SYS/SPEC/DF)
birlikte seri_telemetri_koprusu üzerinden geçiyor** -- `ai_servisi.py`/
DATA96 ikili IQ protokolü şu an KULLANILMIYOR (kod hazır, gerekirse -- ör.
RPi'de tflite gerçek zamanlı yetişmezse -- devreye alınabilir).

**DOĞRULANDI (socat sanal seri port ile)**: seri_telemetri_koprusu.py <->
yer_istasyonu_koprusu iki yönde de (SYS satırı İHA->yer, komut yer->İHA)
test edildi. **DOĞRULANMADI**: gerçek 915MHz radyo ile, gerçek RPi donanımı
ile, `ai_servisi.py`/Jetson AI hattı (kullanılmıyor demiştik ama RPi'de
tflite hızı hiç ölçülmedi).

## SIRADAKİ ADIMLAR (buradan devam, donanım takılınca)

1. RTL-SDR/Pluto/Matek takılınca `streamer.py` + `mavlink_bridge.py` +
   `seri_telemetri_koprusu.py` (gerçek /dev/ttyUSB0 ile) uçtan uca test et.
2. `multimon-ng` kur (`sudo apt install multimon-ng`) ve `sayisal_cozucu.py`'yi
   gerçek bir APRS/AFSK1200 sinyaliyle doğrula -- HENÜZ HİÇ TEST EDİLMEDİ.
3. GUI'de GNSS ALDATMAYI BAŞLAT düğmesinin gerçekten `et_control.py`'ye
   komut gönderdiğini tıklayarak teyit et (kod/protokol doğrulandı, buton
   tıklaması doğrulanmadı).
4. RPi'de `tflite_runtime` ile gerçek zamanlı sınıflandırma hızını ölç --
   yeterince hızlıysa mimari basit kalır (yukarıdaki gibi), yavaşsa
   `ai_servisi.py`/DATA96 IQ pencereleme hattı devreye alınmalı.
5. `EBABIL_RTL_RF_SWITCH_CHIP/HAT` -- RTL-SDR tarafının GPIO pin numarası
   HENÜZ sahada doğrulanmadı, YAPILANDIRILMAMIŞ (anten sabit kalıyor). ET
   tarafı (`EBABIL_ET_RF_SWITCH_CHIP/HATLAR`) 2026-09-15'te DOĞRULANDI, bkz.
   "ET RF ANAHTARI (HMC241) doğrulandı" bölümü.
6. İkinci bir PlutoSDR edinilince `pluto_ed_scanner.py`'yi (2400-2483/5725-5875
   MHz ED) gerçek donanımla test et.
7. Yarışma/saha koşullarında test (antenler arası mesafe, gerçek karışma
   senaryoları, gerçek 915MHz menzil).

## SAHA DONANIM TESTLERİ (2026-09-15)

Üç makine (İHA RPi, ET RPi/"yasin@ebabil", Jetson/"admim-desktop") gerçek
donanımla (RTL-SDR, 2x PlutoSDR, Matek+GPS, 915MHz radyo, HMC241 RF switch)
ilk kez birlikte test edildi.

- **İHA RPi'de pyrtlsdr/librtlsdr yaması GEREKTİ** (Ubuntu dev makinesindeki
  aynı sorun -- bkz. yukarıdaki "pyrtlsdr / Ubuntu librtlsdr uyumsuzluğu"
  notu, RPi OS'ta da AYNI mainline `librtlsdr0` (2.0.2) kurulu, RTL-SDR Blog
  V4 kullanıyoruz). Yama BURADA DA venv/user-site içinde, GİT'E GİRMEDİ --
  RPi'nin SD kartı değişirse/yeniden kurulursa tekrar uygulanmalı (rtlsdr_set_dithering
  + 4 GPIO fonksiyonunu try/except ile sarmak, hem `librtlsdr.py`'deki sembol
  bağlamada hem `rtlsdr.py`'nin `open()` içindeki çağrıda).
- **İHA RPi + RTL-SDR ile `streamer.py` DOĞRULANDI** -- gerçek tarama, gerçek
  hedef tespiti (GNU Radio'dan 433MHz test sinyaliyle).
- **`mavlink_bridge.py` Matek ile DOĞRULANDI** -- Matek `uart0`'a (`/dev/serial0`
  -> `/dev/ttyAMA0`, USB DEĞİL) bağlı, 115200 baud, `EBABIL_MAVLINK_PORT=/dev/serial0`.
- **`konum_servisi` İHA RPi'de (ARM) DERLENDİ ve ÇALIŞTIRILDI** (`cmake .. && make`,
  sadece `libzmq3-dev` gerekiyor) -- port 5570'te hazır. DF/konum kestirimi
  ucundan uca (gerçek uçuşla) henüz test edilmedi, sadece servisin ayakta
  olduğu doğrulandı.
- **ET RPi + Pluto TX ile `et_control.py` DOĞRULANDI** -- SÜREKLİ/ARABAKIŞLI
  karıştırma, GNSS Aldatma (tekli+çoklu servis) hepsi çalışıyor.
- **Pluto TX "Device or resource busy" (EBUSY) hatası bulundu ve DÜZELTİLDİ**
  (`et_control.py`, commit 9effbc3) -- `tx_destroy_buffer()`+`tx()` art arda
  çağrıldığında Pluto'nun FPGA/DMA tarafı Python çağrısıyla tam senkron
  serbest bırakmıyor, EBUSY ile patlıyordu. `_tx_write()` yardımcı metodu
  (5 deneme, 0.2s bekleme) eklendi, thread'ler artık hata durumunda çökmeden
  temiz sonlanıyor. **Süreç yeniden başlatmak YETMEDİ, Pluto'nun fiziksel
  power-cycle'ı (USB çıkar-tak) gerekti** -- kernel/FPGA tarafındaki kilitli
  DMA durumu process restart ile temizlenmiyor.
- **ET RF ANAHTARI (HMC241) doğrulandı** -- gerçek donanım 3/4 ayrı röle
  hattı DEĞİL, IC içine gömülü 2:4 kod çözücü (2 mantıksal pin: A, B).
  `PlutoEtRfAnahtari` bu gerçek donanıma göre yeniden yazıldı (commit
  3190cc8): `EBABIL_ET_RF_SWITCH_HATLAR` artık tam 2 hat (A,B) bekliyor,
  port->A/B: `A=port&1, B=(port>>1)&1`. **ET RPi'nin gerçek pinleri: A=GPIO17
  (fiziksel pin 11), B=GPIO27 (fiziksel pin 13), chip=`/dev/gpiochip0`.**
  4 anten (RF1=144-433, RF2=868-915, RF3=GNSS-1.5G, RF4=2400-2483 ISM) gerçek
  TX + RTL-SDR ile TEK TEK doğrulandı (`tools/dataset/test_pluto_tx_manual.py`
  ile 433.5/900/1200/2450 MHz, 0dB tam güç) -- hepsi doğru antenden çıktı.
- **libgpiod Python API SÜRÜM FARKI ÖNEMLİ**: ET RPi'de `gpiod` paketi v2
  (2.2.0) kurulu çıktı -- v1'in `Chip.get_line()`/`Line.request()` API'si
  yerine tamamen farklı `gpiod.request_lines(chip_yolu, config={...})`/
  `gpiod.line.Direction`/`Value` API'si kullanıyor. `PlutoEtRfAnahtari` ve
  `scripts/test_rf_switch.py` v2'ye göre yazıldı (commit 347563e) -- RTL RF
  anahtarı (`streamer.py`'deki `RtlRfAnahtari`) test edilirken de AYNI sürüm
  farkına dikkat edilmeli, hangi RPi'de hangi `gpiod` sürümü kurulu önceden
  kontrol edilmeli (`python3 -c "import gpiod; print(gpiod.__version__)"`).
  Ayrıca v2 chip yolu tam yol (`/dev/gpiochip0`) istiyor, bare isim
  (`gpiochip0`) DEĞİL -- kodda otomatik `/dev/` öneki ekleniyor ama elle
  test ederken (`gpioinfo` gibi) bu farka dikkat.
- **`scripts/test_rf_switch.py` eklendi** (commit 47894d4/7dffeab) --
  `et_control.py`'den bağımsız, interaktif 1-4 port seçip A/B GPIO'larını
  elle test etmek için.
- **Ağ/IP dersleri**: PC/Jetson/ET RPi farklı ağlardaysa (ör. biri WiFi
  10.70.x.x, diğerleri 192.168.1.x) ZMQ hiç bağlanamıyor -- hepsi aynı
  L2 segmentte olmalı. `.local` (mDNS/Avahi) isimleri güvenilmez olabiliyor
  (client isolation vb.) -- IP ile bağlanmak daha sağlam. **Yarışma günü
  planı**: PC/Jetson/ET RPi kendi özel (internetsiz) unmanaged Ethernet
  switch'ine SABİT IP'lerle bağlanacak (PC=192.168.10.1... GÜNCEL:
  `scripts/baslat_*.sh`'daki plan `192.168.50.1/2/3` -- PC/Jetson/ET RPi).
  İHA RPi bu ağa HİÇ girmiyor (915MHz radyo üzerinden, IP değil).
  Tek-komut başlatma scriptleri eklendi: `scripts/baslat_jetson_koprusu.sh`,
  `scripts/baslat_et_rpi.sh` (bu repo), `baslat_gui.sh` (GUI_QtCreator
  reposu) -- hepsi bu sabit IP'leri gömülü kullanıyor, env var yazmaya
  gerek yok. Henüz gerçek bir switch ile UÇTAN UCA TEST EDİLMEDİ (switch
  henüz temin edilmedi).
- **`et_control.py`: 4. anten portu eklendi** (2400-2483 ISM, commit 1394962)
  -- `ET_ANTEN_BANDLARI` artık 4 bant (144-433/868-915/GNSS-1.5G/2400-2483),
  Pluto TX zaten 6GHz'e kadar çıkabildiği için (`PLUTO_TX_MAX_HZ`) ek bir
  donanım kısıtı yok.

## YÖN BULMA + KONUM KESTİRİMİ eklendi (2026-09-14)

Bu repoda eksik olan tek şey (yön bulma/konum kestirimi, madde 5.1.4/5.1.5)
artık entegre edildi -- Python'da YENİDEN YAZILMADI, ekibin ayrı bir C++
reposundaki (yonKonum1905/yon_bulma + konum_kestirimi, zaten yazılıp test
edilmiş Parçacık Filtresi + EKF) kod, küçük bir ZMQ servisi
(`yonKonum1905/konum_servisi`, port 5570, REQ/REP) olarak dışarı açıldı.

- `src/konum_istemcisi.py` (YENİ) -- `UavKonumDinleyici` (mavlink_bridge.py'nin
  5559'unu dinler, en son gerçek İHA konumunu tutar) ve `KonumIstemcisi`
  (konum_servisi'ne ZMQ REQ ile bağlanır) sınıfları + `konum_guncelle_ve_gonder()`
  yardımcısı. `streamer.py` ve `pluto_ed_scanner.py` ORTAK kullanıyor.
- `streamer.py`/`pluto_ed_scanner.py`: her SYS güncellemesinden sonra
  `konum_guncelle_ve_gonder()` çağrılıyor -- gerçek RSSI + gerçek İHA
  konumu (mavlink_bridge üzerinden) konum_servisi'ne gönderilip, PF/EKF
  geçerli bir konum ürettiyse "DF,<tid>,IHA_MENZIL,<açı>,<rms>,<lat>,<lon>"
  satırı aynı SYS/SPEC portundan (5555/5560) yayınlanıyor.
- **NEDEN "menzil-only" (açı doğrudan ÖLÇÜLMÜYOR):** Anten çifti donanımı
  YOK (tek anten, doğrulandı) -- bu yüzden sağ-sol RSSI karşılaştırmasıyla
  açı ölçümü fiziksel olarak mümkün değil. Bunun yerine: gerçek RSSI +
  İHA'nın (spiral rota ile) farklı noktalardan geçtiği gerçek GPS konumları
  Parçacık Filtresi'ne "bearing güveni sıfır" ile besleniyor, açı PF/EKF'nin
  bulduğu hedef konumundan SONRADAN (atan2 ile) türetiliyor.
- **ÇALIŞTIRMAK İÇİN EK ADIM:** `streamer.py`/`pluto_ed_scanner.py`'den önce
  (ya da yanında) `yonKonum1905/konum_servisi/build/konum_servisi` da
  çalışıyor olmalı (aynı makinede, port 5570) -- çalışmıyorsa DF satırı
  sessizce hiç gönderilmez, SYS/SPEC akışı etkilenmez (best-effort).
  `EBABIL_DF_REF_LAT`/`EBABIL_DF_REF_LON` (varsayılan: 39.9250000/32.8369960)
  yarışma alanının gerçek referans noktasıyla güncellenmeli.
- `ihaRota/` (YENİ) -- İHA'nın uçacağı gerçek arama deseni: Arşimet spiral
  (`yonca_gorev_uret.py`, üretilmiş `yonca_gorevi.waypoints`). Konum
  kestiriminin (yukarıdaki PF) iyi yakınsaması için İHA'nın FARKLI
  noktalardan geçmesi şart -- bu rota tam bunu sağlamak için tasarlandı
  (4/8 yapraklı gül eğrisi denemeleri ölçülüp elenmiş, script içindeki
  yorumlarda gerekçesi var).

## Bilinen tuhaflıklar / geçmişten notlar

- RTL-SDR Windows'ta sık sık donup USB'den kayboluyordu (libusb kararsızlığı,
  replug gerektiriyordu) -- `streamer_watchdog.py` bunun için yazıldı.
  Ubuntu'da bu SORUN OLMAYABİLİR (Linux'un RTL-SDR desteği genelde daha
  stabil), ama garantisi yok.
- Anten yakınlığı (verici/alıcı birbirine çok yakın) RF aşırı yüklenmesine
  (saturation) yol açıp OS-CFAR'ın hem sahte çoklu hedef üretmesine hem de
  gerçek hedefi hiç bulamamasına sebep olabiliyor -- antenler arası makul
  mesafe (birkaç metre) önemli.
- Pluto'nun AGC'sini ("slow_attack") manuel yüksek kazançla değiştirmek
  DENENDİ ve İŞE YARAMADI (kendi gürültü tabanını da yükseltiyor) -- geri
  alındı, AGC'de kalındı. Aralıklı tespit sorunu muhtemelen antenden
  kaynaklanıyor, kazançtan değil.
- `.vscode/settings.json`'da `python.defaultInterpreterPath` Windows'a özel
  bir yol -- Ubuntu'da bu ayar geçersiz olacak, oradaki Python yorumlayıcısı
  (venv) VSCode'da ayrıca seçilmeli ("Python: Select Interpreter").
