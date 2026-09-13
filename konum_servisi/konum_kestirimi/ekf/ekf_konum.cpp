#include "ekf_konum.hpp"

#include <cmath>
#include <algorithm>
#include <array>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// CEP tabanlı hata elipsi hesabı (bkz. eh_guven_skoru_kilavuzu.pdf).
// P matrisi kovaryansından özdeğer ayrışımı yaparak elips yarı-eksenlerini
// ve yönünü döndürür. conf_level: 0.50/0.90/0.95 gibi bir değer.
static HataElipsi calculateErrorEllipse(
    double var_x, double var_y, double cov_xy, double conf_level = 0.90
) {
    HataElipsi elips;
    elips.conf_level = conf_level;

    const double trace    = var_x + var_y;
    const double diff     = var_x - var_y;
    const double root     = std::sqrt((diff * diff / 4.0) + (cov_xy * cov_xy));
    const double lambda1  = (trace / 2.0) + root;
    const double lambda2  = (trace / 2.0) - root;

    const double a_1sig   = std::sqrt(std::max(0.0, lambda1));
    const double b_1sig   = std::sqrt(std::max(0.0, lambda2));

    // k: chi-square 2-DOF eşdeğeri, k = sqrt(-2 * ln(1 - p))
    const double k        = std::sqrt(-2.0 * std::log(1.0 - conf_level));

    elips.major_m   = a_1sig * k;
    elips.minor_m   = b_1sig * k;
    elips.angle_rad = 0.5 * std::atan2(2.0 * cov_xy, diff);

    return elips;
}

static std::array<double, 2> olcumModeliBearingRssi(
    double hedef_x,
    double hedef_y,
    double iha_x,
    double iha_y,
    double iha_alt,
    const RfKalibrasyonSonucu& kalibrasyon,
    double d0_m
) {
    const double dx = hedef_x - iha_x;
    const double dy = hedef_y - iha_y;

    double bearing_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    bearing_deg = aciSar180(bearing_deg);

    const double d3 = ucBoyutluMesafe(iha_x, iha_y, iha_alt, hedef_x, hedef_y);
    const double rssi_dbm = logDistanceRssi(
        d3,
        kalibrasyon.P0_dbm,
        kalibrasyon.n,
        d0_m
    );

    return {bearing_deg, rssi_dbm};
}

static void sayisalJacobian2D(
    double H[2][2],
    double hedef_x,
    double hedef_y,
    double iha_x,
    double iha_y,
    double iha_alt,
    const RfKalibrasyonSonucu& kalibrasyon,
    double d0_m
) {
    const double eps = 0.5; // [m]

    const auto hx_plus = olcumModeliBearingRssi(
        hedef_x + eps, hedef_y,
        iha_x, iha_y, iha_alt,
        kalibrasyon, d0_m
    );

    const auto hx_minus = olcumModeliBearingRssi(
        hedef_x - eps, hedef_y,
        iha_x, iha_y, iha_alt,
        kalibrasyon, d0_m
    );

    const auto hy_plus = olcumModeliBearingRssi(
        hedef_x, hedef_y + eps,
        iha_x, iha_y, iha_alt,
        kalibrasyon, d0_m
    );

    const auto hy_minus = olcumModeliBearingRssi(
        hedef_x, hedef_y - eps,
        iha_x, iha_y, iha_alt,
        kalibrasyon, d0_m
    );

    double dbearing_dx = aciSar180(hx_plus[0] - hx_minus[0]) / (2.0 * eps);
    double drssi_dx = (hx_plus[1] - hx_minus[1]) / (2.0 * eps);

    double dbearing_dy = aciSar180(hy_plus[0] - hy_minus[0]) / (2.0 * eps);
    double drssi_dy = (hy_plus[1] - hy_minus[1]) / (2.0 * eps);

    H[0][0] = dbearing_dx;
    H[0][1] = dbearing_dy;
    H[1][0] = drssi_dx;
    H[1][1] = drssi_dy;
}

void ekfBaslat(
    EkfKonumDurumu& ekf,
    double baslangic_x,
    double baslangic_y,
    double baslangic_belirsizlik_m
) {
    ekf.x = baslangic_x;
    ekf.y = baslangic_y;

    const double var = baslangic_belirsizlik_m * baslangic_belirsizlik_m;

    ekf.P[0][0] = var;
    ekf.P[0][1] = 0.0;
    ekf.P[1][0] = 0.0;
    ekf.P[1][1] = var;

    ekf.initialized = true;
}

void ekfKovaryansAyarla(
    EkfKonumDurumu& ekf,
    const RfKalibrasyonSonucu& kalibrasyon
) {
    double bearing_std = kalibrasyon.bearing_std_deg;
    // sigma2_rssi yerine sigmaToplamHesapla(): Asama 4-5 govde/elektronik
    // kalibrasyonu yapilip govde_ek_varyans_db2 doldurulunca otomatik
    // devreye girsin diye (bkz. rf_kalibrasyon.hpp). Su an o alan 0
    // oldugundan davranis degismiyor.
    double rssi_var = sigmaToplamHesapla(kalibrasyon);

    if (bearing_std < 3.0) {
        bearing_std = 3.0;
    }

    if (rssi_var < 4.0) {
        rssi_var = 4.0;
    }

    ekf.R[0][0] = bearing_std * bearing_std;
    ekf.R[0][1] = 0.0;
    ekf.R[1][0] = 0.0;
    ekf.R[1][1] = rssi_var;
}

EkfTahminSonucu ekfGuncelleBearingRssi(
    EkfKonumDurumu& ekf,
    const EkfOlcum& olcum,
    const RfKalibrasyonSonucu& kalibrasyon,
    double d0_m,
    double area_size_m
) {
    EkfTahminSonucu sonuc;

    if (!ekf.initialized || !kalibrasyon.valid) {
        sonuc.valid = false;
        return sonuc;
    }

    // Prediction: hedef sabit kabul edildiği için x değişmez, sadece P = P + Q
    ekf.P[0][0] += ekf.Q[0][0];
    ekf.P[0][1] += ekf.Q[0][1];
    ekf.P[1][0] += ekf.Q[1][0];
    ekf.P[1][1] += ekf.Q[1][1];

    const auto h = olcumModeliBearingRssi(
        ekf.x,
        ekf.y,
        olcum.iha_x,
        olcum.iha_y,
        olcum.iha_alt,
        kalibrasyon,
        d0_m
    );

    double H[2][2];
    sayisalJacobian2D(
        H,
        ekf.x,
        ekf.y,
        olcum.iha_x,
        olcum.iha_y,
        olcum.iha_alt,
        kalibrasyon,
        d0_m
    );

    // Innovation y = z - h(x)
    // bearing_bias_deg, rf_kalibrasyon.cpp'de raw-true ortalaması olarak
    // tanımlı (raw ~ true + bias); olcumden cikararak duzeltiyoruz.
    double y[2];
    y[0] = aciSar180((olcum.bearing_deg - kalibrasyon.bearing_bias_deg) - h[0]);
    y[1] = olcum.rssi_dbm - h[1];

    // Aşırı sapmış bearing ölçümü EKF'yi bozmasın
    if (std::abs(y[0]) > 85.0) {
        sonuc.hedef_x = ekf.x;
        sonuc.hedef_y = ekf.y;
        sonuc.belirsizlik = std::sqrt(ekf.P[0][0] + ekf.P[1][1]);
        sonuc.elips = calculateErrorEllipse(ekf.P[0][0], ekf.P[1][1], ekf.P[0][1]);
        sonuc.valid = true;
        return sonuc;
    }

    // RSSI ani zıplarsa o ölçüme güveni azalt
    double Rlocal[2][2] = {
    {ekf.R[0][0], ekf.R[0][1]},
    {ekf.R[1][0], ekf.R[1][1]}
    };

    // AoA confidence düşükse bearing ölçümüne daha az güven.
    // confidence = 1.0 ise değişmez.
    // confidence = 0.5 ise bearing varyansı yaklaşık 4 kat büyür.
    // confidence çok düşükse maksimum etki sınırlandırılır.
    double conf = std::min(std::max(olcum.bearing_confidence, 0.15), 1.0);
    double bearing_var_scale = 1.0 / (conf * conf);
    Rlocal[0][0] *= bearing_var_scale;

    // RSSI ani zıplarsa, o anda RSSI etkisini zayıflat.
    if (std::abs(y[1]) > 22.0) {
        Rlocal[1][1] *= 16.0;
    }

    // S = HPH' + R
    double HP[2][2];
    HP[0][0] = H[0][0] * ekf.P[0][0] + H[0][1] * ekf.P[1][0];
    HP[0][1] = H[0][0] * ekf.P[0][1] + H[0][1] * ekf.P[1][1];
    HP[1][0] = H[1][0] * ekf.P[0][0] + H[1][1] * ekf.P[1][0];
    HP[1][1] = H[1][0] * ekf.P[0][1] + H[1][1] * ekf.P[1][1];

    double S[2][2];
    S[0][0] = HP[0][0] * H[0][0] + HP[0][1] * H[0][1] + Rlocal[0][0];
    S[0][1] = HP[0][0] * H[1][0] + HP[0][1] * H[1][1] + Rlocal[0][1];
    S[1][0] = HP[1][0] * H[0][0] + HP[1][1] * H[0][1] + Rlocal[1][0];
    S[1][1] = HP[1][0] * H[1][0] + HP[1][1] * H[1][1] + Rlocal[1][1];

    const double detS = S[0][0] * S[1][1] - S[0][1] * S[1][0];

    if (std::abs(detS) < 1e-12) {
        sonuc.hedef_x = ekf.x;
        sonuc.hedef_y = ekf.y;
        sonuc.belirsizlik = std::sqrt(ekf.P[0][0] + ekf.P[1][1]);
        sonuc.elips = calculateErrorEllipse(ekf.P[0][0], ekf.P[1][1], ekf.P[0][1]);
        sonuc.valid = false;
        return sonuc;
    }

    double invS[2][2];
    invS[0][0] =  S[1][1] / detS;
    invS[0][1] = -S[0][1] / detS;
    invS[1][0] = -S[1][0] / detS;
    invS[1][1] =  S[0][0] / detS;

    // PH'
    double PHt[2][2];
    PHt[0][0] = ekf.P[0][0] * H[0][0] + ekf.P[0][1] * H[0][1];
    PHt[0][1] = ekf.P[0][0] * H[1][0] + ekf.P[0][1] * H[1][1];
    PHt[1][0] = ekf.P[1][0] * H[0][0] + ekf.P[1][1] * H[0][1];
    PHt[1][1] = ekf.P[1][0] * H[1][0] + ekf.P[1][1] * H[1][1];

    // K = PH' inv(S)
    double K[2][2];
    K[0][0] = PHt[0][0] * invS[0][0] + PHt[0][1] * invS[1][0];
    K[0][1] = PHt[0][0] * invS[0][1] + PHt[0][1] * invS[1][1];
    K[1][0] = PHt[1][0] * invS[0][0] + PHt[1][1] * invS[1][0];
    K[1][1] = PHt[1][0] * invS[0][1] + PHt[1][1] * invS[1][1];

    // x = x + K*y
    ekf.x += K[0][0] * y[0] + K[0][1] * y[1];
    ekf.y += K[1][0] * y[0] + K[1][1] * y[1];

    // Saha sınırları
    // DÜZELTME (2026-08-31): [0, area_size_m] yerine merkez-orijinli
    // [-area_size_m/2, +area_size_m/2] kullanılıyor - saha koordinatları
    // (İHA/istasyon konumu, hedef) bu projede tutarlı olarak "home" (0,0)
    // merkezli negatif/pozitif değerlerle kullanılıyor (bkz. main.cpp,
    // testler/). Eski [0, area_size_m] sınırı, negatif koordinatlı her
    // hedefi zorla sıfıra bastırıyordu - gerçek bir hedefe rağmen ~1000m+
    // hataya yol açtığı testler/rota_degerlendirici.cpp ile doğrulandı.
    const double sinir = area_size_m / 2.0;
    ekf.x = std::min(std::max(ekf.x, -sinir), sinir);
    ekf.y = std::min(std::max(ekf.y, -sinir), sinir);

    // P = (I - K H) P
    double KH[2][2];
    KH[0][0] = K[0][0] * H[0][0] + K[0][1] * H[1][0];
    KH[0][1] = K[0][0] * H[0][1] + K[0][1] * H[1][1];
    KH[1][0] = K[1][0] * H[0][0] + K[1][1] * H[1][0];
    KH[1][1] = K[1][0] * H[0][1] + K[1][1] * H[1][1];

    double IminusKH[2][2];
    IminusKH[0][0] = 1.0 - KH[0][0];
    IminusKH[0][1] = 0.0 - KH[0][1];
    IminusKH[1][0] = 0.0 - KH[1][0];
    IminusKH[1][1] = 1.0 - KH[1][1];

    double newP[2][2];
    newP[0][0] = IminusKH[0][0] * ekf.P[0][0] + IminusKH[0][1] * ekf.P[1][0];
    newP[0][1] = IminusKH[0][0] * ekf.P[0][1] + IminusKH[0][1] * ekf.P[1][1];
    newP[1][0] = IminusKH[1][0] * ekf.P[0][0] + IminusKH[1][1] * ekf.P[1][0];
    newP[1][1] = IminusKH[1][0] * ekf.P[0][1] + IminusKH[1][1] * ekf.P[1][1];

    ekf.P[0][0] = newP[0][0];
    ekf.P[0][1] = newP[0][1];
    ekf.P[1][0] = newP[1][0];
    ekf.P[1][1] = newP[1][1];

    // Simetriyi koru
    const double off = 0.5 * (ekf.P[0][1] + ekf.P[1][0]);
    ekf.P[0][1] = off;
    ekf.P[1][0] = off;

    sonuc.hedef_x = ekf.x;
    sonuc.hedef_y = ekf.y;
    sonuc.belirsizlik = std::sqrt(std::max(0.0, ekf.P[0][0] + ekf.P[1][1]));
    sonuc.elips = calculateErrorEllipse(ekf.P[0][0], ekf.P[1][1], ekf.P[0][1]);
    sonuc.valid = true;

    return sonuc;
}