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

## ŞU AN NE YAPILIYOR: Windows'tan Ubuntu'ya taşıma

Proje şimdiye kadar Windows'ta (RTL-SDR + 2x PlutoSDR ile) geliştirildi ve
test edildi. Şu an kullanıcı **Ubuntu'ya (bedirhan-EXCALIBUR-G870) taşıyor**.

Yapılanlar:
- Bu repo ve GUI_QtCreator Ubuntu'ya `git clone` ile çekildi.
- Python venv kuruldu, bağımlılıklar (pyzmq, numpy, scipy, tensorflow,
  pyrtlsdr, pyadi-iio, sounddevice, pymavlink) pip ile kuruldu.
- `models/` ve `data/aldatma_sesleri/` (gitignore'da oldukları için) Google
  Drive üzerinden zip'lenip manuel taşındı.
- RTL-SDR/Pluto için udev/grup izni: `sudo usermod -aG plugdev,dialout $USER`
  yapıldı (çıkış-giriş sonrası aktif olur).
- GUI_QtCreator'daki `CMakeLists.txt`, Linux'ta derlenebilsin diye düzeltildi:
  vcpkg/CONFIG tabanlı ZeroMQ bulma SADECE Windows'ta (`if(WIN32)`), Linux'ta
  `pkg_check_modules` ile `libzmq3-dev` (apt) üzerinden buluyor. cppzmq
  (`zmq.hpp`) Ubuntu'da paket olarak bulunamadı, GitHub'dan (v4.11.0)
  `/usr/local/include`'a manuel indirildi. GUI Ubuntu'da BAŞARIYLA DERLENDİ.

## SIRADAKİ ADIMLAR (buradan devam)

1. Kullanıcı çıkış-giriş yapıp `groups` ile `plugdev`/`dialout`'u doğrulayacak.
2. RTL-SDR/Pluto donanımını Ubuntu'ya takıp `python src/streamer.py` /
   `python src/et_control.py` ile backend'i ilk kez Ubuntu'da test edecek.
3. `~/GUI_QtCreator/build/EHARPP` ile GUI'yi açıp "BAĞLI" durumunu ve hedef
   tespitini doğrulayacak.
4. Bu, Ubuntu'da İLK gerçek uçtan uca test olacak -- Windows'ta yaşanan
   RTL-SDR/libusb kararsızlığının (sık don ma, replug gerektirmesi) Linux'ta
   muhtemelen DAHA AZ görüleceği bekleniyor ama doğrulanmadı.

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
