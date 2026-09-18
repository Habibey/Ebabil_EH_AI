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
  PlutoSDR RX (2.4G/5.8G, tek anten) ────────────────┼─> streamer.py + pluto_ed_scanner.py
  Pixhawk FC + CUAV Neo 3 Pro GPS ────────────────────> mavlink_bridge.py
                                                            │ (yerel ZMQ: 5555/5556/5559/5560/5561)
                                                            v
                                              seri_telemetri_koprusu.py
                                                            │ (915MHz seri radyo, TEK hat)
                                                            v
Yerde (Jetson Nano):  yer_istasyonu_koprusu (C++) + ai_servisi.py + et_control.py
                      (Pluto TX de BURADA, USB ile Jetson'a DİREKT bağlı --
                      ayrı bir ET RPi ARTIK YOK, bkz. "ET RPi kaldırıldı" notu altta)
                                                            │ (ZMQ 5555/5556/5559/5580, Ethernet)
                                                            v
PC/Laptop:            GUI_QtCreator ──ZMQ 5557──> (Jetson'daki et_control.py'ye komut, Ethernet üzerinden)

ET RF çıkışı: Pluto TX ──RF anahtarı (HMC241, A/B pinleri Jetson GPIO'sunda)──> 144-433/868-915/GNSS-1.5G/2400-2483 Yagi+PA
```

### DONANIM DEĞİŞİKLİĞİ (2026-09-16/17): ET RPi kaldırıldı, Pluto TX + et_control.py Jetson'a taşındı

2026-09-16 saha kazası sonrası donanım listesi değişti (Matek → Pixhawk, USB/`/dev/ttyACM0`;
GPS → CUAV Neo 3 Pro, Pixhawk'a bağlı, yazılıma şeffaf -- `mavlink_bridge.py` zaten
marka-bağımsız MAVLink konuştuğu için kod değişikliği gerekmedi). Aynı zamanda **yerde
ayrı duran ET RPi tamamen kaldırıldı** (commit 708ffb4) -- gerekçe: Pluto TX'in RF
çıkışı zaten RF anahtarına (HMC241) bağlanmak zorunda, ve o anahtarın A/B kontrol
pinleri artık Jetson'a kablolu (2026-09-17'de fiziksel olarak yapıldı) -- yani Pluto
zaten Jetson'ın yanında duracak. Pluto'nun USB'sini de PC yerine Jetson'a takmak hem
kablo mesafesini kısaltıyor hem de PC↔Jetson arası IP forwarding/routing gibi ekstra
ağ karmaşıklığından kaçınıyor (Pluto'nun `ip:192.168.2.1` USB-ethernet arayüzüne sadece
doğrudan-bağlı makine erişebilir).

- `scripts/baslat_jetson_koprusu.sh` artık `ai_servisi.py` + `et_control.py`'yi de
  `yer_istasyonu_koprusu` ile birlikte başlatıyor (aynı makine, aynı script).
- `scripts/baslat_et_rpi.sh` KALDIRILDI (artık gereksiz/dead code) -- et_control.py'nin
  başlatılma yeri artık `baslat_jetson_koprusu.sh`.
- **ET RF anahtarının GPIO pin numaraları Jetson'da YENİDEN DOĞRULANDI
  (2026-09-18)** -- eski ET RPi değerleri (A=GPIO17/B=GPIO27, `/dev/gpiochip0`)
  GEÇERSİZDİ, Jetson'ınkiyle aynı değildi. Bu Jetson'ın taşıyıcı kartı NVIDIA'nın
  resmi Developer Kit'i DEĞİL (Jetson.GPIO import edilince "Carrier board is not
  from a Jetson Developer Kit" uyarısı basıyor) -- bu yüzden ham gpiod chip/line
  numarası tahmin etmek yerine, **Jetson.GPIO BOARD modu** ile doğrudan bilinen
  fiziksel header pinleri kullanıldı: **5V=pin2, GND=pin6, A=pin11, B=pin13**.
  Multimetre ile her 4 port seçiminde pin 11/13'teki gerilim (0V/3.3V) beklenen
  HMC241 doğruluk tablosuyla (`A=port&1, B=(port>>1)&1`) eşleşti, DOĞRULANDI.
  `PlutoEtRfAnahtari` bu yöntemi destekleyecek şekilde güncellendi (bkz.
  `_init_jetson_gpio_board`, et_control.py) -- artık `EBABIL_ET_RF_SWITCH_BOARD_PINS=11,13`
  ortam değişkeni kullanılıyor (eski `EBABIL_ET_RF_SWITCH_CHIP/HATLAR` hâlâ
  destekleniyor ama BOARD_PINS ayarlıysa öncelikli). `scripts/baslat_jetson_koprusu.sh`
  bu değeri zaten export ediyor. Test scripti: `scripts/test_rf_switch_jetson.py`
  (Jetson.GPIO BOARD modu, `scripts/test_rf_switch.py`'nin ham-gpiod eşdeğeri).
  **HENÜZ YAPILMADI**: gerçek RF/anten testi (multimetre sadece dijital mantığı
  doğruladı, sinyalin GERÇEKTEN doğru antenden çıktığı Pluto TX + alıcı ile
  henüz doğrulanmadı -- bkz. "SIRADAKİ ADIMLAR" listesi).
- **Ağ planı güncellendi**: sabit-IP switch'inde artık sadece PC (`.1`) ve Jetson
  (`.2`) var -- ET RPi (`.3`) rolü Jetson'a katıldığı için ayrı bir IP'ye gerek
  kalmadı (aşağıdaki "Ağ/IP dersleri" notundaki ET RPi satırları artık tarihsel).

**GÜNCELLEME (2026-09-18, bkz. "SAHA TESTİ" bölümü altta) -- BU PARAGRAF
ARTIK GEÇERSİZ, KOD GERÇEKTE ŞÖYLE ÇALIŞIYOR**: AI sınıflandırması RPi'de
DEĞİL, **Jetson'daki `ai_servisi.py`'de** yapılıyor. `predict.py`'nin modeli
ARTIK RPi'de HİÇ yüklenmiyor (bkz. `streamer.py`'deki `handle_classify_request`
docstring'i, "KARAR: AI Jetson'a taşındı" notu) -- RPi sadece 128 örneklik
küçük bir IQ penceresini `"IQ,<hedef_id>,<b64>"` metin satırı olarak radyoya
gönderiyor, `yer_istasyonu_koprusu` bunu çözüp `ai_servisi.py`'ye (aynı
Jetson'da, ZMQ REQ/REP, `tcp://127.0.0.1:5580`) soruyor, cevabı
`"AI,<id>,<analogSayisal>,<mod>"` olarak KENDİSİ yayınlıyor. **KRİTİK**: bu
istek GUI'de SADECE DİNLE aktifken (5 saniyede bir, `mAiRequestTimer`)
gönderiliyor -- GUI'de AI sınıflandırmasını tetikleyen BAŞKA HİÇBİR yol yok,
yani AI'nin çalışması tamamen DİNLE'nin ayakta kalmasına bağlı.

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
4. ~~RPi'de `tflite_runtime` ile hız ölç~~ -- ARTIK GEÇERSİZ, AI zaten
   Jetson'a taşındı (bkz. "SAHA TESTİ (2026-09-18)" bölümü). **YENİ ÖNCELİK
   (KRİTİK)**: RTL-SDR'ın DİNLE sırasında segfault ile çökmesi (`LIBUSB_ERROR_OVERFLOW`
   → kod -11) çözülmeli -- hem ses hem AI bu yüzden hiç çalışmıyor.
   `usbfs_memory_mb` denemesi yapılıp sonucu doğrulanmalı (bkz. aynı bölüm).
5. `EBABIL_RTL_RF_SWITCH_CHIP/HAT` -- RTL-SDR tarafının GPIO pin numarası
   HENÜZ sahada doğrulanmadı, YAPILANDIRILMAMIŞ (anten sabit kalıyor). ET
   tarafı (`EBABIL_ET_RF_SWITCH_BOARD_PINS=11,13`) Jetson'da 2026-09-18'de
   Jetson.GPIO BOARD modu + multimetre ile YENİDEN DOĞRULANDI (bkz. "ET RF
   anahtarının GPIO pin numaraları Jetson'da YENİDEN DOĞRULANDI" notu) --
   SIRADA gerçek RF/anten testi var (madde 7 ile birlikte yapılabilir).
6. İkinci bir PlutoSDR edinilince `pluto_ed_scanner.py`'yi (2400-2483/5725-5875
   MHz ED) gerçek donanımla test et.
7. Yarışma/saha koşullarında test (antenler arası mesafe, gerçek karışma
   senaryoları, gerçek 915MHz menzil).

## SAHA DONANIM TESTLERİ (2026-09-15)

**NOT (2026-09-16/17 itibarıyla TARİHSEL)**: Bu bölüm o tarihteki donanım
düzenini (ayrı bir ET RPi ile) anlatıyor -- ET RPi sonradan kaldırıldı,
Pluto TX + et_control.py Jetson'a taşındı (bkz. yukarıdaki "DONANIM
DEĞİŞİKLİĞİ" notu). Aşağıdaki GPIO pin numaraları ve "ET RPi" makine
referansları ARTIK GEÇERLİ DEĞİL, sadece o zamanki doğrulamanın kaydı
olarak tutuluyor.

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
  `baslat_gui.sh` (GUI_QtCreator reposu) -- hepsi bu sabit IP'leri gömülü
  kullanıyor, env var yazmaya gerek yok. Henüz gerçek bir switch ile UÇTAN
  UCA TEST EDİLMEDİ (switch henüz temin edilmedi). (`scripts/baslat_et_rpi.sh`
  ET RPi kaldırılınca silindi -- bkz. yukarıdaki "DONANIM DEĞİŞİKLİĞİ" notu.)
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

## SAHA TESTİ (2026-09-18): ED uçtan uca hata ayıklama -- RTL-SDR çökmesi ana sorun

İlk kez gerçek donanımla (İHA RPi + RTL-SDR + Pluto RX + Pixhawk, Jetson,
PC/GUI) tam ED uçtan uca test edildi. Sinyal tespiti/parametre çıkarımı
çalışıyor, ama DİNLE (ses) ve AI (modülasyon sınıflandırma) hiç çalışmadı --
kök sebep bulundu, kalıcı çözüm henüz DOĞRULANMADI (bkz. son madde).

### Ortam/kurulum sorunları (çözüldü)
- **İHA RPi'de leftover systemd servisleri** (`ebabil_sdr.service` -- eski
  ARŞİVLENMİŞ C++ portu `ebabil_test`, VE `konum_servisi.service`) arka
  planda `auto-restart` ile sürekli çalışıp port 5555/5570'i işgal ediyordu,
  `baslat_iha_rpi.sh` ile çakışıyordu. `sudo systemctl stop/disable
  ebabil_sdr.service konum_servisi.service` ile durduruldu/kapatıldı. **RPi
  SD kart değişirse/yeniden imajlanırsa bu servisler muhtemelen dönmez** ama
  aynı RPi'de kalırsa bir daha kontrol edilmeli (`systemctl list-units --all
  | grep -i ebabil`).
- **İHA RPi'nin telemetri radyosu USB DEĞİL, jumper/UART ile bağlı**:
  `/dev/ttyAMA4` (`dtoverlay=uart4` açık, `/dev/serial0`→Pixhawk'tan AYRI).
  `EBABIL_TELEMETRI_PORT=/dev/ttyAMA4` ile veriliyor.
- **Pluto ED (RX)'nin IP'si hiç değiştirilmemiş, fabrika varsayılanı
  `192.168.2.1`'de kalmış** (dosyanın kendi varsayılanı `192.168.3.1`'i
  bekliyor) -- `EBABIL_PLUTO_ED_IP=ip:192.168.2.1 python3
  src/pluto_ed_scanner.py` ile RPi'de AYRI bir terminalde başlatılmalı
  (`baslat_iha_rpi.sh`'a dahil DEĞİL, elle başlatılıyor).
- **RTL-SDR'ı ÇALIŞIRKEN USB'den söküp takmak Jetson'da/RPi'de tüm USB
  ağacını (xHCI host controller) çökertti** ("HC died; cleaning up",
  dmesg'de görüldü) -- Pluto ve Pixhawk dahil TÜM USB cihazları aynı anda
  kayboldu, sadece `sudo reboot` düzeltti. **DERS: sistem ÇALIŞIRKEN hiçbir
  SDR/USB cihazını çıkarıp takmayın**, antenini test etmek istiyorsanız
  sadece anten konektörünü (SMA/MCX) sökün, USB'ye dokunmayın.
- **Jetson'un taşıyıcı kartı üç SDR/companion cihazı (RTL-SDR+Pluto+Pixhawk)
  çıplak USB portlarından besliyor olabilir** -- güç bütçesi sınırlı,
  yukarıdaki çökmenin bir nedeni bu olabilir. Sahada mümkünse **güçlü
  (powered) bir USB hub** kullanılması öneriliyor, henüz uygulanmadı.
- **`baslat_jetson_koprusu.sh`'da yarış durumu (race condition)**: sabit
  `sleep 1`, Jetson Nano'da TensorFlow modelinin yüklenme süresine (birkaç
  saniye) yetmiyordu -- `yer_istasyonu_koprusu` daha `ai_servisi.py` hazır
  olmadan istek gönderip "AI servisine ulasilamadi/zaman asimi" basıyordu.
  DÜZELTİLDİ: sabit sleep yerine port 5580'in gerçekten açılmasını (en fazla
  30sn) bekleyen bir döngü eklendi.

### Bant genişliği hatası bulundu ve düzeltildi (SPEC flooding)
GUI'nin debug konsolunda ("Geçersiz SYS/SPEC paketi") sürekli bozuk/birleşmiş
paket akışı görüldü. Kök sebep: **İZLEME (dwell) ve DİNLE modlarında SPEC
satırı HİÇ kısıtlanmadan (her döngüde) gönderiliyordu** -- ARAMA modunda
zaten var olan `ARAMA_SPEC_ATLAMA` kısıtlaması dwell/dinle'ye hiç
uygulanmamıştı (kasıtlı bir tasarım kararıymış: "operatör tam o an izliyor"
ama sahada 57600 baud hattı tıkayıp SYS/SES paketlerinin bozulmasına yol
açtığı görüldü). `pluto_ed_scanner.py`'de durum DAHA KÖTÜYDÜ (ne ARAMA ne
DWELL'de hiç kısıtlama yoktu). DÜZELTİLDİ:
- `streamer.py`: `EBABIL_DWELL_SPEC_ATLAMA` (varsayılan 3) -- hem dwell hem
  DİNLE dalındaki SPEC gönderimini kısıtlıyor.
- `pluto_ed_scanner.py`: `EBABIL_PLUTO_ARAMA_SPEC_ATLAMA` (varsayılan 5) +
  `EBABIL_PLUTO_DWELL_SPEC_ATLAMA` (varsayılan 3).
Bu değişiklikler `scp` ile RPi'ye taşınıp test edildi (sahadaki asıl çökme
sorunundan -- aşağıya bkz. -- dolayı ses/AI'nin gerçekten düzelip
düzelmediği HENÜZ doğrulanamadı, ama "Geçersiz paketi" sıklığı azaldı).

### GUI tarafında bulunan, kod DEĞİŞMEDEN bilinmesi gereken davranışlar
- **`sescozucu.cpp`**: `streamer.py` yeniden başladığında (çökme/watchdog)
  hedef ID numaralandırması HEDEF-1'den SIFIRLANIYOR -- GUI'nin hafızasındaki
  eski seçim (`edSeciliHedefId`) yeni ID'lerle uyuşmazsa SES paketleri
  SESSİZCE atılıyor (kodun kendi 2026-09-16 tarihli yorumu bunu zaten
  öngörmüş). **Operatör kuralı: her backend yeniden başlatmasından/çökmesinden
  sonra DİNLE'ye basmadan ÖNCE mutlaka GÜNCEL bir hedef kartına yeniden
  tıklayın.**
- `streamer.py`'de de AYNI mantık `DINLE_BASLAT` komutunda var (`if hedef_id
  not in tracker.known: ... dinleme başlatılamadı`) -- bu durumda
  `DURUM,DINLEME_AKTIF` onayı HİÇ gönderilmiyor, GUI'de "Demodülasyon:
  başlatılıyor..." sonsuza kadar takılı kalıyor (backend'in RPi konsolundaki
  `[!] ... bilinmiyor, dinleme başlatılamadı` satırı dışında hiçbir belirti
  yok -- GUI ekranından anlaşılmıyor).

### ASIL/KALICI SORUN -- HENÜZ ÇÖZÜLMEDİ: RTL-SDR, DİNLE sırasında segfault ile çöküyor
DİNLE'ye basılır basılmaz RTL-SDR birkaç saniye içinde çöküyor:
```
rtlsdr_demod_write_reg failed with -9
[!] ...hata (devam ediliyor): <LIBUSB_ERROR_OVERFLOW (-8)> "Could not read 125000 bytes"
[gözcü] streamer.py kendiliğinden sonlandı (kod -11) -- yeniden başlatılıyor.
```
(Kod -11 = SIGSEGV, Python'un `try/except`'i YAKALAYAMIYOR -- native
`librtlsdr`/libusb seviyesinde bir bellek sorunu.) Aynı çökme normal
ARAMA modunda da bir kez görüldü (`LIBUSB_ERROR_IO`), yani DİNLE'ye özel
olmayabilir ama DİNLE'nin sürekli/büyük okuması onu çok daha güvenilir
şekilde tetikliyor. **Bu, hem DİNLE sesinin hem AI sınıflandırmasının hiç
çalışmamasının GERÇEK kök nedeni** -- ikisi de aynı süreç ayakta kalamadığı
için hiç tamamlanamıyor (AI'nin kendi payload'u küçük/hafif, telemetri
bant genişliğiyle İLGİSİ YOK, sadece süreç DİNLE başlar başlamaz çöktüğü
için hiç fırsat bulamıyor).

Gözcü (`streamer_watchdog.py`) doğru şekilde yeniden başlatıyor ama her
çökme `tracker.known`'ı sıfırlıyor (yukarıdaki hedef ID senkron sorununu
katlıyor).

**DENENECEK/ÖNERİLEN ÇÖZÜM (uygulanıp SONUÇ henüz raporlanmadı)**: RTL-SDR
"overflow" hatası Linux'ta çok bilinen bir sorun, genelde çekirdeğin usbfs
bellek limiti düşük olduğu için oluşuyor:
```bash
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
# kalıcı yapmak için:
echo 'options usbcore usbfs_memory_mb=1000' | sudo tee /etc/modprobe.d/usbfs-rtlsdr.conf
```
Bu denenip DİNLE tekrar test edilmeli -- eğer çökme devam ederse kök sebep
daha derin bir libusb/RPi kernel uyumsuzluğu olabilir, o zaman farklı bir
RTL-SDR birimi/kablo/USB portu denenmeli.

### Diğer bulgular
- **146-147 MHz civarında sürekli/güçlü bir sinyal test ortamında (ev/ofis)
  gerçekten var** -- antensiz testle DOĞRULANDI (anten çıkınca kayboldu,
  takınca geri geldi) -- bu bir RTL-SDR birdie'si DEĞİL, gerçek bir dış
  kaynak (yakında bir röle/PMR/IoT cihazı olabilir). Koda dışlama
  EKLENMEDİ (yarışma alanındaki gerçek bir hedefi maskeleme riski
  olduğu için) -- sahada muhtemelen görünmeyecek, görünürse operatör
  bunun donanım arızası olmadığını bilmeli.
- **PC'nin dahili ses donanımı (Realtek ALC256, kulaklık jakı dahil) fiziksel
  olarak arızalı** görünüyor (yazılım/mixer/PipeWire tarafı tamamen sağlıklı
  test edildi, ama hoparlörden VE kulaklıktan hiç ses çıkmadı, kulaklık jakı
  takılınca algılanmadı bile) -- **JBL Tune 510BT Bluetooth kulaklıkla
  atlatıldı** (eşleştirilip "trust" edildi, otomatik bağlanmalı). Yarışma
  günü operatör istasyonunda Bluetooth kulaklık/hoparlör bulundurulmalı.
- **Doğru çalıştırma sırası/komutları (2026-09-18 itibarıyla doğrulandı)**:
  ```
  İHA RPi (terminal 1): EBABIL_TELEMETRI_PORT=/dev/ttyAMA4 ./scripts/baslat_iha_rpi.sh
  İHA RPi (terminal 2): EBABIL_PLUTO_ED_IP=ip:192.168.2.1 python3 src/pluto_ed_scanner.py
  Jetson:                ./scripts/baslat_jetson_koprusu.sh
  PC:                    ~/GUI_QtCreator/baslat_gui.sh
  ```
  **`pluto_ed_scanner.py` RPi'de çalışır, Jetson'da DEĞİL** (Pluto RX
  fiziksel olarak RPi'ye takılı) -- bu karıştırılmamalı.
- Telemetri protokolü baştan sona **düz metin, satır tabanlı** (virgülle
  ayrılmış ASCII) -- ikili veri (IQ/ses) bile base64 ile metne çevrilip
  aynı satır formatında taşınıyor, ayrı bir ikili protokol yok.

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
