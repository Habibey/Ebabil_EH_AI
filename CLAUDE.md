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
- `src/streamer_watchdog.py` — streamer.py donarsa (RTL-SDR/libusb
  kararsızlığı) otomatik yeniden başlatan gözcü.

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

## SIRADAKİ ADIMLAR (buradan devam)

1. İkinci bir PlutoSDR edinilince `pluto_ed_scanner.py`'yi (2400-2483 MHz ED)
   gerçek donanımla test et.
2. Uzun süreli (saatler) çalıştırıp Ubuntu'da da RTL-SDR/libusb donması
   yaşanıyor mu gözlemle -- `streamer_watchdog.py`'nin hâlâ gerekip
   gerekmediğini bunun sonucuna göre değerlendir.
3. Yarışma/saha koşullarında test (antenler arası mesafe, gerçek karışma
   senaryoları).
4. Sıradaki büyük hedef: Jetson Nano / Raspberry Pi'ye taşıma -- aşağıdaki
   bölüme bak.

## GELECEK HEDEF: Jetson Nano ve Raspberry Pi (Ubuntu'dan sonra)

Ubuntu laptop kurulumu bitince sıradaki hedef bunları **gömülü/headless
sensör kutusu** olarak kurmak: KARAR VERİLDİ -- GUI bu kartlarda ÇALIŞMAYACAK,
sadece backend (streamer.py/pluto_ed_scanner.py/et_control.py) orada çalışıp
RF donanımına takılı kalacak; operatör GUI'yi kendi laptopunda
(`EBABIL_JETSON_IP=<kartın IP'si>` ortam değişkeniyle, bkz. GUI_QtCreator
CLAUDE.md) uzaktan izleyecek. Yani bu kartlara Qt6/CMake/GUI derleme İŞİ
HİÇ YOK -- sadece Python + SDR sürücüleri.

Farklılıklar/dikkat edilecekler:
- **Jetson Nano (JetPack)**: genelde eski Ubuntu (18.04/20.04) + eski Python
  (3.6 olabilir). `predict.py` zaten TensorFlow yoksa `tflite_runtime`'a
  düşüyor (`_HAS_TF`) -- Jetson'da NVIDIA'nın CUDA'lı TF wheel'ini kurmaya
  UĞRAŞMA, doğrudan `pip install tflite_runtime` yeterli, `models/*.tflite`
  zaten hazır. `sdr_common.py`'de numpy<1.20 için `sliding_window_view`
  uyumluluk shim'i de zaten var (eski Python 3.6 ihtimaline karşı).
- **Raspberry Pi**: ARM mimarisi (Jetson'la ortak nokta) -- TensorFlow yine
  muhtemelen çalışmaz/gereksiz, aynı şekilde `tflite_runtime` kullan.
- İkisinde de: RTL-SDR/Pluto için udev/`plugdev`,`dialout` grup izni Ubuntu'da
  yaptığımızın aynısı gerekiyor. ARM için bazı pip paketleri (numpy/scipy)
  önceden derlenmiş wheel bulamayabilir, `pip install`'ın kaynak koddan
  derlemesi normalden uzun sürebilir (`python3-dev`, `build-essential`
  kurulu olsun).
- Backend'i başlatırken: `EBABIL_ZMQ_BIND_HOST=0.0.0.0` (kartta) +
  `EBABIL_JETSON_IP=<kartın-IP'si>` (GUI'nin çalıştığı laptopta) -- ikisi
  birlikte "backend uzakta, GUI ayrı makinede" senaryosunu aktif eder (kod
  zaten buna göre yazılmıştı, hiç değişiklik gerekmez).

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
