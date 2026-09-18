// konum_servisi -- mevcut, test edilmis C++ yon bulma/konum kestirimi
// kodunu (yon_bulma/konum_kestirimi) Python backend'in (streamer.py,
// pluto_ed_scanner.py -- ayni repo) kullanabilmesi icin kucuk bir ZMQ
// REQ/REP servisi olarak disari acar.
//
// KOKEN: bu kod asilinda ayri bir repoda (yonKonum1905/konum_servisi,
// github.com/billgatoss/EbabilElektronikHarp) gelistirildi/test edildi; RPi
// uzerinde artik SADECE bu repo (Ebabil_EH_AI) calistigi icin (RTL-SDR/Pluto
// taramasini streamer.py/pluto_ed_scanner.py yapiyor, eski C++ ebabil_sdr
// artik devrede degil) yon-konum kismi BURAYA vendor edildi -- yon_bulma/ ve
// konum_kestirimi/ alt klasorleri o repodaki AYNI kaynak, kopyalandi.
// Matematikte bir degisiklik/duzeltme gerekirse iki repoda da (asil kaynak +
// bu vendor kopyasi) yapilmasi/senkron tutulmasi gerekir.
//
// NEDEN AYRI BIR SERVIS (Python'da yeniden yazmak yerine): PF/EKF matematigi
// zaten C++'ta yazilip test edildi (koordinat donusumu + aci turetme izole
// test edildi, gercek pymavlink verisiyle dogrulandi -- bkz. proje notlari).
// Python'da bastan yazmak ayni matematigi tekrar (ve test tarihinde farkli
// bir hatayla) uretme riski tasirdi -- bunun yerine Python sadece girdi
// (RSSI + o anki UAV konumu) gonderiyor, TUM hesap (ENU donusumu, PF/EKF,
// aci turetme) burada, ayni kodla yapiliyor.
//
// Protokol (duz metin, ZMQ REQ/REP -- istemci Python, sunucu bu):
//   Istek:  "GUNCELLE,<tid>,<band_hz>,<rssi_dbm>,<uav_lat>,<uav_lon>,<uav_irtifa_m>"
//   Cevap:  "OK,<hedef_lat>,<hedef_lon>,<aci_deg>,<rms_derece>"  (PF/EKF bir konum urettiyse)
//           "BEKLIYOR"                                           (henuz yeterli olcum yok)
//           "HATA,<sebep>"
//   Istek:  "SIFIRLA,<tid>"  -- bir hedefin takibini sifirlar (ör. yeni tarama turu)
//   Cevap:  "OK"
//
//   -- SAHA KALIBRASYONU (bkz. yonKonum1905/yonKonum/testler/
//   path_loss_kalibrasyon_araci.cpp'nin GERCEK/ucan versiyonu -- orada mesafe
//   elle giriliyordu, burada bilinen sabit bir vericinin GPS konumundan
//   ve o anki IHA GPS konumundan OTOMATIK hesaplaniyor, bkz. Python
//   tarafindaki kalibrasyon_kaydedici.py):
//   Istek:  "KAL_EKLE,<band_hz>,<mesafe_m>,<rssi_dbm>" -- bir ornek ekler
//   Cevap:  "OK,<o ana kadar birikmis ornek sayisi>"
//   Istek:  "KAL_HESAPLA,<band_hz>" -- birikmis orneklerden rfKalibrasyonHesapla()
//           ile GERCEK P0/n hesaplar, gecerliyse bu bant icin BUNDAN SONRA
//           olusturulacak TUM hedeflerin kalibrasyonu olarak kaydeder (demo
//           yer tutucunun yerine gecer).
//   Cevap:  "OK,<P0_dbm>,<n>,<rssi_residual_std>,<d0_m>,<ornek_sayisi>"
//           "HATA,yetersiz_ornek,<sayi>"  (en az 3 ornek gerekir)
//           "HATA,gecersiz_regresyon"
//   Istek:  "KAL_SIFIRLA,<band_hz>" -- o bant icin birikmis ornekleri atar
//           (kotu bir kalibrasyon turunu tekrarlamak icin)
//   Cevap:  "OK"
//
// Anten cifti donanimi YOK (tek anten, dogrulandi) -- bu yuzden "menzil-only"
// yontem kullaniliyor: rssi_dbm tek bir gercek olcum, bearing/aci OLCULMUYOR
// (guven=0 ile PF/EKF'ye "sadece RSSI'nin etkili olmasi" saglaniyor), aci
// PF/EKF'nin urettigi hedef konumundan SONRADAN atan2 ile turetiliyor --
// main.cpp'deki (ebabil_sdr) ayni yaklasimla birebir tutarli.
#include "konum_kestirimi/orkestrasyon/konum_orkestrasyonu.hpp"
#include "konum_kestirimi/ortak/rf_kalibrasyon.hpp"
#include "yon_bulma/ortak/yon_sonucu.hpp"
#include "yon_bulma/ortak/aci_yardimci.hpp"

#include <zmq.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr double DUNYA_YARICAP_M = 6378137.0;
constexpr double PI = 3.14159265358979323846;

double ortamOndalik(const char* ad, double varsayilan) {
    const char* deger = std::getenv(ad);
    if (!deger || *deger == '\0') return varsayilan;
    char* son = nullptr;
    double sonuc = std::strtod(deger, &son);
    return (son == deger) ? varsayilan : sonuc;
}

std::string ortamMetin(const char* ad, const std::string& varsayilan) {
    const char* deger = std::getenv(ad);
    return (deger && *deger != '\0') ? std::string(deger) : varsayilan;
}

std::vector<std::string> parcala(const std::string& s, char ayrac) {
    std::vector<std::string> parcalar;
    std::stringstream ss(s);
    std::string parca;
    while (std::getline(ss, parca, ayrac)) parcalar.push_back(parca);
    return parcalar;
}

// band_hz'i 100kHz'e yuvarlayip bir tamsayi anahtara cevirir -- ayni
// "bandin" farkli taramalarda birkac kHz kayan tepe frekanslariyla
// gelmesi (gurultu/binleme) tam float esitligiyle kalibrasyon
// kaydini/aramasini bozmasin diye.
long long bandAnahtari(double band_hz) {
    return static_cast<long long>(std::llround(band_hz / 100000.0));
}

// main.cpp (ebabil_sdr) ile AYNI donusum -- yerelden_enlem_boylama'nin tersi.
void enlemBoylamdanYerele(double lat_deg, double lon_deg, double lat0_deg, double lon0_deg,
                          double& x_out, double& y_out) {
    y_out = (lat_deg - lat0_deg) * (PI / 180.0) * DUNYA_YARICAP_M;
    x_out = (lon_deg - lon0_deg) * (PI / 180.0) * DUNYA_YARICAP_M * std::cos(lat0_deg * PI / 180.0);
}

// DISKTEN KALIBRASYON YUKLEME (2026-09-18 eklendi): testler/
// path_loss_kalibrasyon_araci.cpp'nin urettigi <bant>mhz_ozet.txt
// dosyalarini (basit "anahtar=deger" satirlari, "#" ile baslayan yorum
// kismi yoksayilir) servis ACILISINDA okur ve gercek_kalibrasyonlar'a
// onceden doldurur. Bu olmadan, RPi her yeniden baslatildiginda tum
// bantlar "KAL_HESAPLA" o OTURUMDA calistirilana kadar demo yer tutucuya
// (P0=-40dBm, n=2.5) dusuyordu -- sahada onceden toplanmis gercek
// kalibrasyon, servis restart edilirse kayboluyordu.
std::unordered_map<long long, RfKalibrasyonSonucu> kalibrasyonlariDisktenYukle(const std::string& dizin) {
    std::unordered_map<long long, RfKalibrasyonSonucu> sonuc;

    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::exists(dizin, ec) || ec) {
        std::cout << "[KONUM] Kalibrasyon dizini bulunamadi (" << dizin << ") - hepsi demo yer tutucuyla baslayacak.\n";
        return sonuc;
    }

    for (const auto& girdi : fs::directory_iterator(dizin, ec)) {
        if (ec) break;
        if (girdi.path().extension() != ".txt") continue;
        if (girdi.path().filename().string().find("_ozet") == std::string::npos) continue;

        std::ifstream dosya(girdi.path());
        if (!dosya) continue;

        RfKalibrasyonSonucu kal;
        double band_hz = -1.0;
        std::string satir;

        while (std::getline(dosya, satir)) {
            const auto esit = satir.find('=');
            if (esit == std::string::npos) continue;

            const std::string anahtar = satir.substr(0, esit);
            std::string deger_str = satir.substr(esit + 1);

            // "  # aciklama" kuyruklarini at.
            const auto yorum = deger_str.find('#');
            if (yorum != std::string::npos) deger_str = deger_str.substr(0, yorum);

            double deger = 0.0;
            try {
                deger = std::stod(deger_str);
            } catch (...) {
                continue;
            }

            if (anahtar == "band_hz") band_hz = deger;
            else if (anahtar == "P0_dbm") kal.P0_dbm = deger;
            else if (anahtar == "n") kal.n = deger;
            else if (anahtar == "sigma2_rssi") kal.sigma2_rssi = deger;
            else if (anahtar == "rssi_residual_std") kal.rssi_residual_std = deger;
            else if (anahtar == "sigma_mesafe_katsayisi") kal.sigma_mesafe_katsayisi = deger;
            else if (anahtar == "govde_ek_varyans_db2") kal.govde_ek_varyans_db2 = deger;
            else if (anahtar == "elektronik_taban_gurultu_dbm") kal.elektronik_taban_gurultu_dbm = deger;
        }

        if (band_hz <= 0.0) {
            std::cout << "[KONUM] UYARI: " << girdi.path() << " icinde gecerli band_hz yok, atlandi.\n";
            continue;
        }

        kal.valid = true;
        sonuc[bandAnahtari(band_hz)] = kal;
        std::cout << "[KONUM] Diskten kalibrasyon yuklendi: " << (band_hz / 1e6) << " MHz"
                  << " (P0=" << kal.P0_dbm << "dBm, n=" << kal.n << ") <- " << girdi.path() << "\n";
    }

    return sonuc;
}

void yerelden_enlem_boylama(double x_dogu_m, double y_kuzey_m, double lat0_deg, double lon0_deg,
                             double& lat_out, double& lon_out) {
    lat_out = lat0_deg + (y_kuzey_m / DUNYA_YARICAP_M) * (180.0 / PI);
    lon_out = lon0_deg + (x_dogu_m / (DUNYA_YARICAP_M * std::cos(lat0_deg * PI / 180.0))) * (180.0 / PI);
}

}  // namespace

int main() {
    std::cout.setf(std::ios::unitbuf);

    // TODO: yarisma alaninin GERCEK referans noktasi -- ebabil_sdr/main.cpp'deki
    // DF_REF_LAT/DF_REF_LON ile AYNI tutulmali (ikisi ayri sureclerdeyse bile,
    // ayni fiziksel yeri temsil ediyor olmalilar).
    const double ref_lat = ortamOndalik("EBABIL_DF_REF_LAT", 39.9250000);
    const double ref_lon = ortamOndalik("EBABIL_DF_REF_LON", 32.8369960);
    const double bearing_std_deg = ortamOndalik("EBABIL_DF_BEARING_STD_DEG", 10.0);
    const std::string bind_adres = ortamMetin("EBABIL_KONUM_SERVISI_BIND", "tcp://127.0.0.1:5570");

    // ikinci alan: bu hedefin kalibrasyonu GERCEK saha verisinden mi (disk/
    // KAL_HESAPLA) yoksa DEMO yer tutucudan mi geldi -- GUNCELLE cevabinda
    // her seferinde bildiriliyor (bkz. asagidaki "kal_durum" alani) ki
    // Python/GUI tarafi bunu SESSIZCE gercekmis gibi GOSTERMESIN.
    std::unordered_map<std::string, std::pair<std::unique_ptr<KonumOrkestrasyonu>, bool>> takip;

    // Saha kalibrasyonu -- bkz. dosya basindaki KAL_EKLE/KAL_HESAPLA notu.
    // gercek_kalibrasyonlar doluysa (KAL_HESAPLA basariyla calistiysa) yeni
    // hedefler artik demo yer tutucu yerine BUNU kullanir.
    std::unordered_map<long long, std::vector<KalibrasyonOrnegi>> kalibrasyon_ornekleri;
    std::unordered_map<long long, RfKalibrasyonSonucu> gercek_kalibrasyonlar =
        kalibrasyonlariDisktenYukle(ortamMetin("EBABIL_KAL_CIKTI_DIZINI", "kalibrasyon/rssi_mesafe_kalibrasyonu"));

    zmq::context_t ctx(1);
    zmq::socket_t rep(ctx, zmq::socket_type::rep);
    rep.bind(bind_adres);

    std::cout << "[KONUM] Hazir -- " << bind_adres << " (ref: " << ref_lat << "," << ref_lon << ")\n";

    while (true) {
        zmq::message_t istek_msg;
        auto sonuc = rep.recv(istek_msg, zmq::recv_flags::none);
        if (!sonuc) continue;

        const std::string istek(static_cast<char*>(istek_msg.data()), istek_msg.size());
        const auto p = parcala(istek, ',');
        std::string cevap = "HATA,gecersiz_istek";

        if (!p.empty() && p[0] == "GUNCELLE" && p.size() == 7) {
            try {
                const std::string tid = p[1];
                const double band_hz = std::stod(p[2]);
                const double rssi_dbm = std::stod(p[3]);
                const double uav_lat = std::stod(p[4]);
                const double uav_lon = std::stod(p[5]);
                const double uav_irtifa = std::stod(p[6]);

                auto it = takip.find(tid);
                if (it == takip.end()) {
                    auto orks = std::make_unique<KonumOrkestrasyonu>(/*d0_m=*/1.0, /*alan_boyutu_m=*/1000.0);
                    orks->takibi_baslat(/*baslangic_x=*/500.0, /*baslangic_y=*/500.0,
                                        /*baslangic_belirsizlik_m=*/300.0);

                    // Bu bant icin GERCEK saha kalibrasyonu (KAL_HESAPLA ile)
                    // yapildiysa onu kullan; yapilmadiysa demo/yer tutucuya
                    // dus (bkz. proje notlari -- gercek kalibrasyon yapilana
                    // kadar gecerli bir yaklasik deger).
                    const auto kal_it = gercek_kalibrasyonlar.find(bandAnahtari(band_hz));
                    RfKalibrasyonSonucu kalibrasyon;
                    bool kalibrasyon_gercek = false;
                    if (kal_it != gercek_kalibrasyonlar.end()) {
                        kalibrasyon = kal_it->second;
                        kalibrasyon_gercek = true;
                    } else {
                        kalibrasyon.P0_dbm = -40.0;
                        kalibrasyon.n = 2.5;
                        kalibrasyon.sigma2_rssi = 9.0;
                        kalibrasyon.bearing_bias_deg = 0.0;
                        kalibrasyon.bearing_std_deg = bearing_std_deg;
                        kalibrasyon.valid = true;
                    }
                    orks->kalibrasyon_kaydet(band_hz, kalibrasyon);

                    it = takip.emplace(tid, std::make_pair(std::move(orks), kalibrasyon_gercek)).first;
                }

                double uav_x = 0.0, uav_y = 0.0;
                enlemBoylamdanYerele(uav_lat, uav_lon, ref_lat, ref_lon, uav_x, uav_y);

                YonSonucu yonSonucu;
                yonSonucu.band_hz = band_hz;
                yonSonucu.platform = PlatformTipi::IHA;
                yonSonucu.rssi_ana = rssi_dbm;
                yonSonucu.guven = 0.0;             // bearing olcumu yok, sadece RSSI etkili olsun
                yonSonucu.kuresel_yon_acisi = 0.0; // kullanilmiyor -- asagida konumdan turetiliyor
                yonSonucu.gecerli = true;

                GozlemciKonumu gozlemci;
                gozlemci.x = uav_x;
                gozlemci.y = uav_y;
                gozlemci.irtifa = uav_irtifa;

                EkfTahminSonucu tahmin = it->second.first->yon_sonucuyla_guncelle(yonSonucu, gozlemci);

                if (tahmin.valid) {
                    double hedef_lat = 0.0, hedef_lon = 0.0;
                    yerelden_enlem_boylama(tahmin.hedef_x, tahmin.hedef_y, ref_lat, ref_lon, hedef_lat, hedef_lon);

                    const double dx_dogu = tahmin.hedef_x - uav_x;
                    const double dy_kuzey = tahmin.hedef_y - uav_y;
                    const double aci_deg =
                        AciYardimci::aci_normalize_et(std::atan2(dx_dogu, dy_kuzey) * 180.0 / PI);

                    // kal_durum: GUI/Python tarafinin demo yer tutucuyu
                    // gercek kalibrasyonmus gibi SESSIZCE gostermemesi icin
                    // -- bkz. konum_istemcisi.py'deki kullanim.
                    const char* kal_durum = it->second.second ? "GERCEK" : "DEMO";

                    std::ostringstream os;
                    os << "OK," << std::fixed << std::setprecision(7) << hedef_lat << "," << hedef_lon
                       << "," << std::setprecision(2) << aci_deg << "," << bearing_std_deg
                       << "," << kal_durum;
                    cevap = os.str();
                } else {
                    cevap = "BEKLIYOR";
                }
            } catch (const std::exception& e) {
                cevap = std::string("HATA,") + e.what();
            }
        } else if (p.size() == 2 && p[0] == "SIFIRLA") {
            takip.erase(p[1]);
            cevap = "OK";

        } else if (p.size() == 4 && p[0] == "KAL_EKLE") {
            try {
                const double band_hz = std::stod(p[1]);
                const double mesafe_m = std::stod(p[2]);
                const double rssi_dbm = std::stod(p[3]);
                if (mesafe_m <= 0.0) {
                    cevap = "HATA,mesafe_pozitif_olmali";
                } else {
                    KalibrasyonOrnegi ornek;
                    ornek.mesafe_m = mesafe_m;
                    ornek.rssi_dbm = rssi_dbm;
                    ornek.raw_bearing_deg = 0.0;
                    ornek.true_bearing_deg = 0.0;
                    auto& liste = kalibrasyon_ornekleri[bandAnahtari(band_hz)];
                    liste.push_back(ornek);
                    cevap = "OK," + std::to_string(liste.size());
                }
            } catch (const std::exception& e) {
                cevap = std::string("HATA,") + e.what();
            }

        } else if (p.size() == 2 && p[0] == "KAL_HESAPLA") {
            try {
                const double band_hz = std::stod(p[1]);
                const auto anahtar = bandAnahtari(band_hz);
                const auto orn_it = kalibrasyon_ornekleri.find(anahtar);
                const size_t sayi = (orn_it != kalibrasyon_ornekleri.end()) ? orn_it->second.size() : 0;
                if (sayi < 3) {
                    cevap = "HATA,yetersiz_ornek," + std::to_string(sayi);
                } else {
                    const auto& ornekler = orn_it->second;
                    double d0_m = ornekler.front().mesafe_m;
                    for (const auto& o : ornekler) d0_m = std::min(d0_m, o.mesafe_m);

                    const RfKalibrasyonSonucu hesap = rfKalibrasyonHesapla(ornekler, d0_m);
                    if (!hesap.valid) {
                        cevap = "HATA,gecersiz_regresyon";
                    } else {
                        gercek_kalibrasyonlar[anahtar] = hesap;
                        std::ostringstream os;
                        os << "OK," << std::fixed << std::setprecision(2) << hesap.P0_dbm << ","
                           << std::setprecision(3) << hesap.n << "," << std::setprecision(2)
                           << hesap.rssi_residual_std << "," << d0_m << "," << sayi;
                        cevap = os.str();
                        std::cout << "[KONUM] Kalibrasyon guncellendi (bant~" << band_hz / 1e6 << " MHz): "
                                  << cevap << "\n";
                    }
                }
            } catch (const std::exception& e) {
                cevap = std::string("HATA,") + e.what();
            }

        } else if (p.size() == 2 && p[0] == "KAL_SIFIRLA") {
            try {
                kalibrasyon_ornekleri.erase(bandAnahtari(std::stod(p[1])));
                cevap = "OK";
            } catch (const std::exception& e) {
                cevap = std::string("HATA,") + e.what();
            }
        }

        zmq::message_t cevap_msg(cevap.data(), cevap.size());
        rep.send(cevap_msg, zmq::send_flags::none);
    }
}
