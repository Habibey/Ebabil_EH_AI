// Yer istasyonu (Jetson Nano) köprüsü.
//
// 915 MHz telemetri radyosunun yer istasyonu ucundaki TEK sahibi bu program
// -- radyo, İHA'daki ebabil_test'in TelemetriGonderici ile yazdığı ÜÇ tür
// veriyi taşıyor:
//   1) ASCII satırlar (SYS/SPEC/DF/SAYISAL/DURUM) -- formatları hiç
//      ayrıştırılmadan, opak string olarak ZMQ PUB (tcp://*:5555) üzerinden
//      arayuz'e aktarılır (bkz. yonKonum1905/arayuz/mainwindow.cpp parseLine
//      -- format orada zaten çözülüyor, burada tekrarlanmıyor).
//   2) MAVLink DATA96 çerçeveleri (bkz. IQGondirici.h/.cpp) -- CRNN
//      modülasyon sınıflandırması için 128 örneklik ham IQ pencereleri VE
//      (ayrı bir metadata çerçevesiyle) hangi banda ait olduğu. Burada
//      çözülüp tamamlanan pencereler, AYNI Jetson'da çalışan ai_servisi.py'ye
//      (Ebabil_EH_AI/src -- CRNN modelini ZMQ REQ/REP ile dışarı açar) REQ
//      isteğiyle gönderilir; sonuç "AI,<bant_adi>,<analogSayisal>,
//      <modulasyonTuru>" satırı olarak tcp://*:5556'dan yayınlanır (bkz.
//      AiServisiIstemcisi). bant_adi, arayüzün SYS paketindeki "id" alanıyla
//      (bant adı) BİREBİR aynı olmalı ki AI sonucu doğru hedefin kartına
//      yazılsın.
//   3) Uçuş kontrolcüsünün (MATEK WingV3) MAVLink çerçeveleri -- RPi
//      tarafında main.cpp'nin fcKoprusuAc() ile AYRI bir UART'tan okuyup
//      AYNI radyoya bindirdiği tam MAVLink akışı (bkz. TelemetriGonderici.h
//      "MAVLink köprüsü" notu). Burada SYS_STATUS/ATTITUDE/
//      GLOBAL_POSITION_INT/VFR_HUD mesajları ayrıştırılıp arayuzun beklediği
//      "UAV,..." satırına (bkz. mainwindow.cpp parseLine, "UAV" tipi)
//      dönüştürülür ve ayrı bir ZMQ PUB soketinden (tcp://*:5559) yayınlanır
//      -- alan sıralamaları pymavlink'in gerçek unpack format'ıyla
//      (native_format) çapraz doğrulandı, hafızadan tahmin EDİLMEDİ.
//      Bu dört mesaj DIŞINDAKİ MAVLink çerçeveleri (ör. HEARTBEAT,
//      PARAM_VALUE) şimdilik yok sayılıyor -- ne Mission Planner'a ne
//      arayuze taşınmıyor (bkz. yer_istasyonu_port_paylastirici.sh, tam
//      MAVLink akışını Mission Planner'a AYRICA kopyalamak için ayrı bir
//      araç; bu ikisi birbirini tamamlıyor, aynı işi yapmıyor).
//
// Ters yönde: arayuz'ün 5557 numaralı PUB portuna (KOMUT,DINLE/DURDUR,
// HEDEF_SEC, ET,..., SET_POWER, SDR_VERISI_ISTEK vb.) SUB olunur, gelen HER
// satır olduğu gibi (filtre yok) radyoya yazılır. Bizi ilgilendirmeyen
// satırlar (ET,../SET_POWER -- bunlar etSunucu'nun kendi SUB bağlantısı için)
// İHA tarafındaki main.cpp'de zaten sessizce yok sayılıyor (main.cpp yalnızca
// "KOMUT,DINLE,"/"KOMUT,DURDUR" önekli satırları tanıyor), bu yüzden
// filtresiz iletmek güvenli.
#include "mavlink_cerceve.hpp"

#include <zmq.hpp>

#include <array>
#include <cerrno>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

using ebabil::mavlinkCerceveUzunlugu;

namespace {

constexpr uint8_t MAVLINK_MSG_ID_DATA96 = 172;
constexpr int PENCERE_BOYUTU = 128;   // IQGondirici.h PENCERE_BOYUTU ile aynı olmalı
constexpr int PARCA_BASI_VERI = 94;   // IQGondirici.h PARCA_BASI_VERI ile aynı olmalı
constexpr int MAKS_PARCA = 8;         // gerçek değer 3 (256/94) -- güvenlik payı
// IQGondirici.h METADATA_SENTINEL ile AYNI olmali -- bant adi cercevesini
// IQ parcalarindan (0..MAKS_PARCA-1) ayirt etmek icin kullanilan deger.
constexpr uint8_t METADATA_SENTINEL = 0xFF;

// Uçuş kontrolcüsünden (MATEK WingV3, ArduPilot) beklenen mesajlar --
// id'ler MAVLink common.xml'de sabit, degismez. Alan sıralamaları
// (byte offset'leri) pymavlink'in native_format unpack string'iyle
// dogrulandi:
//   SYS_STATUS (id=1):           '<IIIHHhHHHHHHb' -- battery_remaining
//                                 (int8, %) son bayt, offset 30.
//   ATTITUDE (id=30):            '<Iffffff' -- roll(off4)/pitch(off8) float rad.
//   GLOBAL_POSITION_INT (id=33): '<Iiiiihhhh' -- lat(off4,degE7)/lon(off8,degE7)/
//                                 relative_alt(off16, mm).
//   VFR_HUD (id=74):             '<ffffhh' -- groundspeed(off4)/heading(off16,deg).
constexpr uint32_t MAVLINK_MSG_ID_SYS_STATUS = 1;
constexpr uint32_t MAVLINK_MSG_ID_ATTITUDE = 30;
constexpr uint32_t MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33;
constexpr uint32_t MAVLINK_MSG_ID_VFR_HUD = 74;

int32_t oku_i32le(const uint8_t* p) { int32_t v; std::memcpy(&v, p, 4); return v; }
uint16_t oku_u16le(const uint8_t* p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
int16_t oku_i16le(const uint8_t* p) { int16_t v; std::memcpy(&v, p, 2); return v; }
float oku_f32le(const uint8_t* p) { float v; std::memcpy(&v, p, 4); return v; }

// FC'den parca parca gelen mesajlardan (SYS_STATUS/ATTITUDE/
// GLOBAL_POSITION_INT/VFR_HUD) biriktirilen, arayuzun tek bir "UAV,..."
// satirinda beklediği alanlar. Dördü de gelene kadar (hazir=false) hicbir
// satir gonderilmez -- yoksa henuz gelmemis alanlar icin 0/varsayilan
// deger, gercek veriymis gibi arayuzde gosterilir.
struct UavDurumu {
    double lat = 0.0, lon = 0.0, alt_m = 0.0;
    double hiz_ms = 0.0, yon_deg = 0.0;
    double pitch_deg = 0.0, roll_deg = 0.0;
    double batarya_yuzde = 0.0;
    bool konum_geldi = false, tutum_geldi = false, hiz_geldi = false, batarya_geldi = false;
    bool hazir() const { return konum_geldi && tutum_geldi && hiz_geldi && batarya_geldi; }
};

constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

// payload: MAVLink cercevesindeki (v1 ya da v2) mesaj govdesi, msgid'e gore
// yorumlanir. Bilinmeyen/ilgisiz msgid'ler sessizce yok sayilir (cagiran
// taraf zaten sadece bu 4 id icin cagirir).
void ucusKontrolcusuMesajiIsle(uint32_t msgid, const uint8_t* payload, size_t payload_len, UavDurumu& durum) {
    if (msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && payload_len >= 18) {
        durum.lat = oku_i32le(payload + 4) / 1e7;
        durum.lon = oku_i32le(payload + 8) / 1e7;
        durum.alt_m = oku_i32le(payload + 16) / 1000.0;  // relative_alt, mm -> m
        durum.konum_geldi = true;
    } else if (msgid == MAVLINK_MSG_ID_ATTITUDE && payload_len >= 12) {
        durum.roll_deg = oku_f32le(payload + 4) * RAD_TO_DEG;
        durum.pitch_deg = oku_f32le(payload + 8) * RAD_TO_DEG;
        durum.tutum_geldi = true;
    } else if (msgid == MAVLINK_MSG_ID_VFR_HUD && payload_len >= 18) {
        durum.hiz_ms = oku_f32le(payload + 4);       // groundspeed
        durum.yon_deg = oku_i16le(payload + 16);     // heading
        durum.hiz_geldi = true;
    } else if (msgid == MAVLINK_MSG_ID_SYS_STATUS && payload_len >= 31) {
        // battery_remaining: FC bilmiyorsa -1 gonderir (MAVLink standardi) --
        // bu deger OLDUGU GIBI iletilir, 0'a CEVRILMEZ ("%0 pil" gercek bir
        // bos batarya OLCUMU gibi gorunup operatoru yanlis bilgilendirirdi;
        // -1 en azindan "veri yok" oldugunu acikca gosterir).
        durum.batarya_yuzde = static_cast<int8_t>(payload[30]);
        durum.batarya_geldi = true;
    }
}

std::string ortamMetin(const char* ad, const std::string& varsayilan) {
    const char* deger = std::getenv(ad);
    return (deger && *deger != '\0') ? std::string(deger) : varsayilan;
}

double ortamOndalik(const char* ad, double varsayilan) {
    const char* deger = std::getenv(ad);
    if (!deger || *deger == '\0') return varsayilan;
    char* son = nullptr;
    double sonuc = std::strtod(deger, &son);
    return (son == deger || *son != '\0') ? varsayilan : sonuc;
}

speed_t baudSabitineCevir(int baud) {
    // TelemetriGonderici.cpp'deki aynı isimli fonksiyonun küçük bir
    // kopyası -- iki taraf da bağımsız süreç/derleme birimi olduğu için
    // (ayrı executable, ortak kütüphane hedefi yok) burada tekrar edildi.
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B57600;
    }
}

// Seri portu TelemetriGonderici ile aynı ayarlarla (8N1, ham mod, O_NONBLOCK)
// açar -- radyonun iki ucu simetrik olmalı.
int seriPortAc(const std::string& yol, int baud) {
    int fd = open(yol.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[KOPRU] Seri port acilamadi: " << yol << " -> " << std::strerror(errno) << "\n";
        return -1;
    }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }
    cfsetospeed(&tty, baudSabitineCevir(baud));
    cfsetispeed(&tty, baudSabitineCevir(baud));
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);
    cfmakeraw(&tty);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

// EAGAIN'de poll() ile bekleyip tekrar deneyen sağlam yazma yardımcısı --
// TelemetriGonderici::tamYazFd ile aynı desen (fd O_NONBLOCK acildigi icin
// write() kismi/short donebilir).
void tamYaz(int fd, const char* veri, size_t uzunluk) {
    size_t gonderilen = 0;
    while (gonderilen < uzunluk) {
        ssize_t n = write(fd, veri + gonderilen, uzunluk - gonderilen);
        if (n > 0) {
            gonderilen += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd{fd, POLLOUT, 0};
            poll(&pfd, 1, 1000);
            continue;
        }
        break;
    }
}

// Tek bir IQ penceresinin parçalarını (data96Gonder'ın ters işlemi) toplar.
struct PencereTamponu {
    std::array<uint8_t, PENCERE_BOYUTU * 2> veri{};
    std::array<bool, MAKS_PARCA> parca_alindi{};
    uint8_t beklenen_toplam_parca = 0;
    int alinan_parca_sayisi = 0;
};

// pencere_id -> o an biriktirilmekte olan pencere (birden fazla kaynak --
// Pluto/RTL thread'leri -- ayni anda pencere gonderebildigi icin, farkli
// pencere_id'lerin parcalari birbirine INTERLEAVE olabilir; bu yuzden tek
// bir "aktif pencere" degil, kimlige gore bir harita tutuluyor).
std::unordered_map<uint8_t, PencereTamponu> pencereler;

// pencere_id -> bant adi (IQGondirici::bantAdiMetadataGonder ile IQ
// parcalarindan ONCE gelir, bkz. IQGondirici.h). Pencere tamamlaninca bu
// haritadan okunup arayuzun SYS paketindeki "id" (bant adi) ile eslesecek
// sekilde AI sonucuna eklenir -- YOKSA (metadata cercevesi kaybolduysa,
// olmamasi beklenir ama seri hat gurultusuyle teorik olarak mumkun) pencere
// hedef_id'siz kalir, cagiran taraf bunu YAYINLAMAMALI (uydurma bir id
// ATANMAZ).
std::unordered_map<uint8_t, std::string> bant_adlari;

// DATA96 payload'ini (98 bayt: type+len+data[96]) cozup, ilgili pencereye
// ekler. Pencere tamamlaninca 128 orneklik std::complex<float> vektorunu VE
// (varsa) bant adini donduren callback cagrilir.
void data96PencereEkle(
    const uint8_t* payload, size_t payload_len,
    const std::function<void(uint8_t, const std::optional<std::string>&, const std::vector<std::complex<float>>&)>&
        pencereHazir) {
    if (payload_len < 4) return;

    const uint8_t pencere_id = payload[0];
    const uint8_t len = payload[1];        // = 2 (alt-header) + bu parcadaki IQ/bant-adi bayt sayisi
    const uint8_t parca_index = payload[2];
    const uint8_t toplam_parca = payload[3];

    if (len < 2) return;  // bozuk/beklenmeyen cerceve -- sessizce at

    if (parca_index == METADATA_SENTINEL) {
        // IQ parcasi degil, bant adi metadata cercevesi (bkz.
        // IQGondirici::bantAdiMetadataGonder) -- bu pencere_id icin IQ
        // parcalari tamamlaninca kullanilmak uzere sakla.
        const size_t ad_uzunluk = std::min<size_t>(static_cast<size_t>(len) - 2, payload_len - 4);
        bant_adlari[pencere_id] = std::string(reinterpret_cast<const char*>(payload + 4), ad_uzunluk);
        return;
    }

    if (parca_index >= MAKS_PARCA || toplam_parca == 0 || toplam_parca > MAKS_PARCA) {
        return;  // bozuk/beklenmeyen cerceve -- sessizce at
    }
    const size_t chunk_len = std::min<size_t>(static_cast<size_t>(len) - 2, payload_len - 4);

    PencereTamponu& pt = pencereler[pencere_id];
    pt.beklenen_toplam_parca = toplam_parca;

    const size_t ofset = static_cast<size_t>(parca_index) * PARCA_BASI_VERI;
    if (ofset + chunk_len <= pt.veri.size()) {
        std::memcpy(pt.veri.data() + ofset, payload + 4, chunk_len);
    }

    if (!pt.parca_alindi[parca_index]) {
        pt.parca_alindi[parca_index] = true;
        ++pt.alinan_parca_sayisi;
    }

    if (pt.alinan_parca_sayisi >= pt.beklenen_toplam_parca) {
        std::vector<std::complex<float>> ornekler;
        ornekler.reserve(PENCERE_BOYUTU);
        for (int i = 0; i < PENCERE_BOYUTU; ++i) {
            const auto i_int = static_cast<int8_t>(pt.veri[2 * i]);
            const auto q_int = static_cast<int8_t>(pt.veri[2 * i + 1]);
            ornekler.emplace_back(i_int / 127.0f, q_int / 127.0f);
        }

        std::optional<std::string> bant_adi;
        auto bant_it = bant_adlari.find(pencere_id);
        if (bant_it != bant_adlari.end()) {
            bant_adi = bant_it->second;
            bant_adlari.erase(bant_it);
        }

        pencereHazir(pencere_id, bant_adi, ornekler);
        pencereler.erase(pencere_id);
    }
}

std::vector<std::string> parcala(const std::string& s, char ayrac) {
    std::vector<std::string> parcalar;
    size_t basla = 0;
    while (true) {
        size_t bul = s.find(ayrac, basla);
        parcalar.push_back(s.substr(basla, bul - basla));
        if (bul == std::string::npos) break;
        basla = bul + 1;
    }
    return parcalar;
}

// "IQ,<hedef_id>,<base64>" satirlarini cozmek icin -- streamer.py/
// pluto_ed_scanner.py artik RPi'de siniflandirma yapmiyor (bkz. Ebabil_EH_AI
// CLAUDE.md "AI Jetson'a tasindi"), 128 orneklik IQ penceresini base64
// metin olarak buraya gonderiyor. Harici bir kutuphaneye bagimli olmamak
// icin standart base64 cozme burada elle yazildi.
std::vector<uint8_t> base64Coz(const std::string& metin) {
    static const std::string tablo =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    static std::vector<int> ters = [] {
        std::vector<int> t(256, -1);
        for (int i = 0; i < 64; ++i) t[static_cast<unsigned char>(tablo[i])] = i;
        return t;
    }();

    std::vector<uint8_t> cikti;
    int deger = 0, bit_sayisi = -8;
    for (unsigned char c : metin) {
        if (c == '=') break;
        if (ters[c] == -1) continue;  // gecersiz/bosluk karakteri, atla
        deger = (deger << 6) + ters[c];
        bit_sayisi += 6;
        if (bit_sayisi >= 0) {
            cikti.push_back(static_cast<uint8_t>((deger >> bit_sayisi) & 0xFF));
            bit_sayisi -= 8;
        }
    }
    return cikti;
}

// ai_servisi.py'ye (Ebabil_EH_AI/src, Jetson'da ayni makinede calisir) ZMQ
// REQ/REP ile baglanan istemci -- konum_servisi'nin Python istemcisiyle
// (konum_istemcisi.py, KonumIstemcisi) AYNI "lazy pirate" deseni (REQ
// soketleri zaman asiminda kilitlenebildigi icin, zaman asiminda soket
// KAPATILIP YENIDEN aciliyor).
//
// NEDEN AYRI BIR SERVIS (C++'a TensorFlow gomulu yerine): CRNN modeli zaten
// Python/Keras'ta egitilip test edildi (Ebabil_EH_AI/src/predict.py). Ayni
// deseni TERSTEN kullaniyoruz -- konum_servisi C++ matematigini Python'a
// aciyordu, burada Python AI modelini C++'a aciyoruz (bkz. ai_servisi.py
// basindaki ayni gerekce).
class AiServisiIstemcisi {
public:
    AiServisiIstemcisi(zmq::context_t& ctx_, std::string adres_, int zaman_asimi_ms_ = 500)
        : ctx(ctx_), adres(std::move(adres_)), zaman_asimi_ms(zaman_asimi_ms_) {
        baglan();
    }

    // Basarili olursa (analog_sayisal, mod, guven) doldurur ve true doner;
    // zaman asimi/hata/servis kapaliysa false doner -- cagiran taraf BU
    // DURUMDA HICBIR SEY YAYINLAMAMALI (sahte/varsayilan bir AI sonucu asla
    // uretilmez).
    bool siniflandir(const std::vector<std::complex<float>>& ornekler, std::string& analog_sayisal_out,
                      std::string& mod_out, std::string& guven_out) {
        zmq::message_t istek(ornekler.data(), ornekler.size() * sizeof(std::complex<float>));
        try {
            if (!sock->send(istek, zmq::send_flags::none)) {
                baglan();
                return false;
            }
            zmq::message_t cevap;
            auto sonuc = sock->recv(cevap, zmq::recv_flags::none);
            if (!sonuc) {
                baglan();  // zaman asimi -- soket bozuldu, yenile
                return false;
            }
            const std::string metin(static_cast<char*>(cevap.data()), cevap.size());
            const auto parcalar = parcala(metin, ',');
            if (parcalar.empty() || parcalar[0] != "OK" || parcalar.size() != 4) {
                return false;
            }
            analog_sayisal_out = parcalar[1];
            mod_out = parcalar[2];
            guven_out = parcalar[3];
            return true;
        } catch (const std::exception&) {
            baglan();
            return false;
        }
    }

private:
    void baglan() {
        sock = std::make_unique<zmq::socket_t>(ctx, zmq::socket_type::req);
        sock->set(zmq::sockopt::rcvtimeo, zaman_asimi_ms);
        sock->set(zmq::sockopt::linger, 0);
        sock->connect(adres);
    }

    zmq::context_t& ctx;
    std::string adres;
    int zaman_asimi_ms;
    std::unique_ptr<zmq::socket_t> sock;
};

}  // namespace

int main() {
    std::cout.setf(std::ios::unitbuf);

    const std::string seriYol = ortamMetin("EBABIL_TELEMETRI_PORT", "/dev/ttyUSB0");
    const int baud = static_cast<int>(ortamOndalik("EBABIL_TELEMETRI_BAUD", 57600));
    const std::string arayuzHost = ortamMetin("EBABIL_ARAYUZ_HOST", "127.0.0.1");

    int fd = seriPortAc(seriYol, baud);
    if (fd < 0) {
        std::cerr << "[KOPRU] Telemetri portu olmadan devam edilemez, cikiliyor.\n";
        return 1;
    }

    zmq::context_t ctx(1);

    // Arayuz mainwindow.cpp setupZmqConnections()'daki 5555 (SYS/SPEC) ucuna
    // eslenir -- format satiri hic parse etmeden aktarildigi icin
    // arayuz tarafinda hicbir degisiklik gerekmiyor.
    zmq::socket_t pub(ctx, zmq::socket_type::pub);
    pub.bind("tcp://*:5555");

    // Arayuzun setupZmqConnections()'da SUB oldugu ikinci uc nokta -- sadece
    // FC'den cozulen "UAV,..." satirlari buradan gider (bkz. dosya basindaki
    // aciklama, madde 3).
    zmq::socket_t pub_uav(ctx, zmq::socket_type::pub);
    pub_uav.bind("tcp://*:5559");

    // Arayuzun ikinci SUB ucu (streamer.py'nin AI portuyla AYNI numara,
    // GUI tarafinda hicbir fark yok) -- CRNN siniflandirma sonuclari
    // ("AI,<id>,<analogSayisal>,<modulasyonTuru>") buradan gider.
    zmq::socket_t pub_ai(ctx, zmq::socket_type::pub);
    pub_ai.bind("tcp://*:5556");

    // ai_servisi.py (Ebabil_EH_AI/src) -- CRNN modelini bu koprude REQ/REP
    // ile disari acan Python servisi, AYNI Jetson'da varsayilan olarak
    // localhost'ta calisir.
    const std::string aiHost = ortamMetin("EBABIL_AI_SERVISI_HOST", "127.0.0.1");
    const std::string aiPort = ortamMetin("EBABIL_AI_SERVISI_PORT", "5580");
    AiServisiIstemcisi ai_istemcisi(ctx, "tcp://" + aiHost + ":" + aiPort);

    // Arayuzun kendi PUB bind ettigi 5557 komut kanalina SUB oluyoruz (bkz.
    // mainwindow.cpp setupZmqConnections yorumu).
    zmq::socket_t sub(ctx, zmq::socket_type::sub);
    sub.connect("tcp://" + arayuzHost + ":5557");
    sub.set(zmq::sockopt::subscribe, "");

    std::cout << "[KOPRU] Hazir -- seri port: " << seriYol << " (" << baud << " baud), "
              << "PUB tcp://*:5555, PUB tcp://*:5559, SUB tcp://" << arayuzHost << ":5557\n";

    uint32_t pencere_sayaci = 0;
    std::string okuma_tamponu;
    UavDurumu uav_durumu;

    while (true) {
        // 1) Radyodan gelen veriyi oku (kisa poll -- ne CPU yakar ne de
        // komut yonunu ac gecikmeyle ihmal eder).
        pollfd pfd{fd, POLLIN, 0};
        poll(&pfd, 1, 20);

        char buf[512];
        ssize_t n;
        while ((n = read(fd, buf, sizeof(buf))) > 0) {
            okuma_tamponu.append(buf, static_cast<size_t>(n));
        }

        while (!okuma_tamponu.empty()) {
            const uint8_t ilk_bayt = static_cast<uint8_t>(okuma_tamponu[0]);

            if (ilk_bayt == 0xFE || ilk_bayt == 0xFD) {
                std::optional<size_t> uzunluk = mavlinkCerceveUzunlugu(okuma_tamponu);
                if (!uzunluk || okuma_tamponu.size() < *uzunluk) {
                    break;  // henuz tam cerceve gelmedi, sonraki dongude devam
                }

                // v2 (0xFD): MSGID 3 bayt (offset 7-9), payload offset 10.
                // v1 (0xFE): MSGID 1 bayt (offset 5), payload offset 6.
                // IQGondirici hep v2 yaziyor (DATA96 sadece orada aranir);
                // FC (ArduPilot) v1 ya da v2 gonderebilir, ikisi de kabul edilir.
                const auto* bayt = reinterpret_cast<const uint8_t*>(okuma_tamponu.data());
                const bool v2 = (ilk_bayt == 0xFD);
                const uint32_t msgid = v2 ? (bayt[7] | (static_cast<uint32_t>(bayt[8]) << 8) |
                                              (static_cast<uint32_t>(bayt[9]) << 16))
                                           : bayt[5];
                const uint8_t* payload = bayt + (v2 ? 10 : 6);
                const size_t payload_len = *uzunluk - (v2 ? 12 : 8);  // -CRC(2) -header

                if (v2 && msgid == MAVLINK_MSG_ID_DATA96 && payload_len >= 98) {
                    data96PencereEkle(payload, 98,
                                      [&](uint8_t pid, const std::optional<std::string>& bant_adi,
                                          const std::vector<std::complex<float>>& ornekler) {
                        ++pencere_sayaci;
                        std::cout << "[KOPRU] IQ penceresi tamamlandi (id=" << static_cast<int>(pid)
                                  << ", #" << pencere_sayaci << ", " << ornekler.size() << " ornek"
                                  << (bant_adi ? (", bant=" + *bant_adi) : ", bant=YOK") << ")\n";

                        if (!bant_adi) {
                            // Metadata cercevesi bu pencere_id icin hic gelmedi
                            // (bkz. IQGondirici::bantAdiMetadataGonder) -- arayuzun
                            // hangi hedefe yazacagini bilemeyiz, UYDURMA bir id
                            // ATAMADAN atla.
                            std::cerr << "[KOPRU] UYARI: pencere " << static_cast<int>(pid)
                                      << " icin bant adi gelmedi, AI sonucu atlaniyor.\n";
                            return;
                        }

                        std::string analog_sayisal, mod, guven;
                        if (!ai_istemcisi.siniflandir(ornekler, analog_sayisal, mod, guven)) {
                            std::cerr << "[KOPRU] AI servisine ulasilamadi/zaman asimi, "
                                         "bu pencere icin siniflandirma atlaniyor.\n";
                            return;
                        }

                        const std::string satir = "AI," + *bant_adi + "," + analog_sayisal + "," + mod;
                        zmq::message_t msg(satir.data(), satir.size());
                        pub_ai.send(msg, zmq::send_flags::dontwait);
                        std::cout << "[KOPRU] " << satir << " (guven %" << guven << ")\n";
                    });
                } else if (msgid == MAVLINK_MSG_ID_SYS_STATUS || msgid == MAVLINK_MSG_ID_ATTITUDE ||
                           msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT || msgid == MAVLINK_MSG_ID_VFR_HUD) {
                    ucusKontrolcusuMesajiIsle(msgid, payload, payload_len, uav_durumu);
                    if (uav_durumu.hazir()) {
                        char satir[256];
                        int n2 = std::snprintf(satir, sizeof(satir), "UAV,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f\n",
                                                uav_durumu.lat, uav_durumu.lon, uav_durumu.alt_m, uav_durumu.hiz_ms,
                                                uav_durumu.yon_deg, uav_durumu.pitch_deg, uav_durumu.roll_deg,
                                                uav_durumu.batarya_yuzde);
                        if (n2 > 0) {
                            zmq::message_t msg(satir, static_cast<size_t>(n2));
                            pub_uav.send(msg, zmq::send_flags::dontwait);
                        }
                    }
                }

                okuma_tamponu.erase(0, *uzunluk);
            } else {
                const size_t yeni_satir = okuma_tamponu.find('\n');
                if (yeni_satir == std::string::npos) {
                    break;  // henuz tam satir gelmedi
                }
                std::string satir = okuma_tamponu.substr(0, yeni_satir);
                okuma_tamponu.erase(0, yeni_satir + 1);
                if (satir.empty()) {
                    // hicbir sey yapma, sadece asagidaki genel yayina dusme
                } else if (satir.rfind("IQ,", 0) == 0) {
                    // "IQ,<hedef_id>,<base64>" -- RPi artik siniflandirma
                    // yapmiyor, burada ai_servisi.py'ye sorup sonucu KENDIMIZ
                    // yayinliyoruz (bkz. base64Coz yorumu).
                    const size_t virgul2 = satir.find(',', 3);
                    if (virgul2 == std::string::npos) {
                        std::cerr << "[KOPRU] Gecersiz IQ satiri (ikinci virgul yok): " << satir << "\n";
                    } else {
                        const std::string hedef_id = satir.substr(3, virgul2 - 3);
                        const std::vector<uint8_t> ham = base64Coz(satir.substr(virgul2 + 1));
                        if (ham.size() != PENCERE_BOYUTU * sizeof(std::complex<float>)) {
                            std::cerr << "[KOPRU] IQ penceresi beklenmeyen boyutta (" << ham.size()
                                      << " bayt) -- atlaniyor.\n";
                        } else {
                            std::vector<std::complex<float>> ornekler(PENCERE_BOYUTU);
                            std::memcpy(ornekler.data(), ham.data(), ham.size());
                            std::string analog_sayisal, mod, guven;
                            if (ai_istemcisi.siniflandir(ornekler, analog_sayisal, mod, guven)) {
                                const std::string ai_satiri = "AI," + hedef_id + "," + analog_sayisal + "," + mod + "," + guven;
                                zmq::message_t ai_msg(ai_satiri.data(), ai_satiri.size());
                                pub_ai.send(ai_msg, zmq::send_flags::dontwait);
                                std::cout << "[KOPRU] " << ai_satiri << " (guven %" << guven << ")\n";
                            } else {
                                std::cerr << "[KOPRU] AI servisine ulasilamadi/zaman asimi -- "
                                             "bu pencere icin siniflandirma atlaniyor.\n";
                            }
                        }
                    }
                } else {
                    zmq::message_t msg(satir.data(), satir.size());
                    pub.send(msg, zmq::send_flags::dontwait);
                }
            }
        }

        // 2) Arayuzden gelen komut satirlarini radyoya ilet.
        zmq::message_t cmsg;
        while (sub.recv(cmsg, zmq::recv_flags::dontwait)) {
            std::string satir(static_cast<char*>(cmsg.data()), cmsg.size());
            satir.push_back('\n');
            tamYaz(fd, satir.data(), satir.size());
        }
    }
}
