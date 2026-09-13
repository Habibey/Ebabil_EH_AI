#ifndef RF_KALIBRASYON_HPP
#define RF_KALIBRASYON_HPP

#include <vector>

struct KalibrasyonOrnegi {
    double mesafe_m;          // İHA - bilinen kalibrasyon vericisi 3B mesafe
    double rssi_dbm;          // ölçülen RSSI
    double raw_bearing_deg;   // ölçülen bearing
    double true_bearing_deg;  // bilinen vericiye göre gerçek bearing
};

struct RfKalibrasyonSonucu {
    double P0_dbm = 0.0;              // d0 mesafesindeki alınan güç
    double n = 2.0;                   // yol kaybı üssü
    double sigma2_rssi = 25.0;        // RSSI residual varyansı [dB^2]

    double rssi_residual_mean = 0.0;
    double rssi_residual_std = 5.0;

    double bearing_bias_deg = 0.0;
    double bearing_std_deg = 10.0;

    // ASAMA 6 EKLENTISI (2026-09-01): sigma_rssi'yi (dB) log-distance
    // modelinin turevi (Friis) uzerinden mesafe belirsizligine cevirir.
    // rfKalibrasyonHesapla() bunu otomatik dolduruyor - bkz. sigmaMesafeAdaptif().
    // sigma_distance(d) ~= sigma_mesafe_katsayisi * d (UHUK2018'deki gibi,
    // mesafeyle orantili adaptif sigma - RSSI hatasi dB uzayinda sabitse
    // bu oran log-distance modelin dogal bir sonucudur).
    double sigma_mesafe_katsayisi = 0.0;

    // ASAMA 4-5 HOOK'U (2026-09-01, henuz DOLDURULMADI - saha verisi yok):
    // govde/elektronik-gurultu kalibrasyon kampanyalari (protokol Asama 4-5)
    // yapildiginda buraya yazilacak ek sistematik hata terimleri. Su an
    // hepsi 0 - EKF/PF'nin R matrisine hicbir ek katki yapmiyorlar. Sifir
    // degilse (biri kalibre ettiginde), sigma2_rssi'ye eklenmelidir (bkz.
    // sigmaToplamHesapla()).
    double govde_ek_varyans_db2 = 0.0;       // Asama 4: govde golgeleme/yansima
    double elektronik_taban_gurultu_dbm = 0.0; // Asama 5: motor/telemetri kaynakli taban gurultu (henuz kullanilmiyor, sadece kayit icin)

    bool valid = false;
};

double aciSar180(double aci_deg);

double acisalOrtalamaDeg(const std::vector<double>& acilar_deg);

double ucBoyutluMesafe(
    double iha_x,
    double iha_y,
    double iha_alt,
    double hedef_x,
    double hedef_y
);

double logDistanceRssi(
    double mesafe_m,
    double P0_dbm,
    double n,
    double d0_m
);

RfKalibrasyonSonucu rfKalibrasyonHesapla(
    const std::vector<KalibrasyonOrnegi>& ornekler,
    double d0_m
);

// ASAMA 6: log-distance modelinin (RSSI = P0 - 10*n*log10(d/d0)) turevini
// alip sabit bir RSSI hatasini (dB) mesafe hatasina (m) cevirir (Friis'in
// dogal bir sonucu - RSSI dB uzayinda sabit varyansliysa, mesafe uzayindaki
// karsiligi mesafeyle DOGRU ORANTILIDIR):
//
//   sigma_distance(d) = sigma_rssi_db * d * ln(10) / (10*n)
//
// n kucukse (sinyal mesafeyle az degisiyorsa) sonuc BUYUK cikar - bu
// beklenen bir durum, o banddaki RSSI'nin mesafe kestirimi icin dogal
// olarak daha az bilgi tasidigini gosterir.
double sigmaMesafeFriisDen(double sigma_rssi_db, double mesafe_m, double n);

// kalibrasyon.sigma_mesafe_katsayisi * mesafe_m kisayolu - runtime'da
// "su an tahmin edilen mesafede beklenen mesafe belirsizligi ne kadar"
// sorusuna hizli cevap icin (tanı/loglama amacli, EKF/PF'nin kendi ic
// agirliklandirmasini DEGISTIRMEZ - onlar zaten dB uzayinda calisip bu
// etkiyi dogrusal olmayan h(x) uzerinden kendiliginden yansitiyor).
double sigmaMesafeAdaptif(double mesafe_m, const RfKalibrasyonSonucu& kalibrasyon);

// ASAMA 4-5 HOOK'U: sigma2_rssi'ye govde_ek_varyans_db2'yi (dolu ise)
// ekleyip EKF/PF'nin R matrisine gidecek TOPLAM RSSI varyansini doner.
// Asama 4-5 verisi olmadigi surece govde_ek_varyans_db2=0 oldugundan bu,
// mevcut sigma2_rssi'yi degistirmeden doner.
double sigmaToplamHesapla(const RfKalibrasyonSonucu& kalibrasyon);

#endif