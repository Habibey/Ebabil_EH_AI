#include "rf_kalibrasyon.hpp"

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double aciSar180(double aci_deg) {
    while (aci_deg >= 180.0) {
        aci_deg -= 360.0;
    }

    while (aci_deg < -180.0) {
        aci_deg += 360.0;
    }

    return aci_deg;
}

double acisalOrtalamaDeg(const std::vector<double>& acilar_deg) {
    if (acilar_deg.empty()) {
        return 0.0;
    }

    double s = 0.0;
    double c = 0.0;

    for (double aci : acilar_deg) {
        double rad = aci * M_PI / 180.0;
        s += std::sin(rad);
        c += std::cos(rad);
    }

    s /= static_cast<double>(acilar_deg.size());
    c /= static_cast<double>(acilar_deg.size());

    double ortalama = std::atan2(s, c) * 180.0 / M_PI;
    return aciSar180(ortalama);
}

double ucBoyutluMesafe(
    double iha_x,
    double iha_y,
    double iha_alt,
    double hedef_x,
    double hedef_y
) {
    const double dx = hedef_x - iha_x;
    const double dy = hedef_y - iha_y;
    const double dz = -iha_alt;

    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    return std::max(d, 1.0);
}

double logDistanceRssi(
    double mesafe_m,
    double P0_dbm,
    double n,
    double d0_m
) {
    const double d = std::max(mesafe_m, d0_m);
    return P0_dbm - 10.0 * n * std::log10(d / d0_m);
}

RfKalibrasyonSonucu rfKalibrasyonHesapla(
    const std::vector<KalibrasyonOrnegi>& ornekler,
    double d0_m
) {
    RfKalibrasyonSonucu sonuc;

    if (ornekler.size() < 3 || d0_m <= 0.0) {
        sonuc.valid = false;
        return sonuc;
    }

    /*
        Log-distance model:

        RSSI = P0 - 10*n*log10(d/d0)

        Bunu lineer regresyona çeviriyoruz:

        y = a*x + b

        y = RSSI
        x = log10(d/d0)
        b = P0
        a = -10*n

        n = -a / 10
    */

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;

    int N = 0;

    for (const auto& o : ornekler) {
        if (o.mesafe_m <= 0.0) {
            continue;
        }

        const double x = std::log10(std::max(o.mesafe_m, d0_m) / d0_m);
        const double y = o.rssi_dbm;

        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        N++;
    }

    if (N < 3) {
        sonuc.valid = false;
        return sonuc;
    }

    const double denom = static_cast<double>(N) * sum_xx - sum_x * sum_x;

    if (std::abs(denom) < 1e-12) {
        sonuc.valid = false;
        return sonuc;
    }

    const double slope = (static_cast<double>(N) * sum_xy - sum_x * sum_y) / denom;
    const double intercept = (sum_y - slope * sum_x) / static_cast<double>(N);

    sonuc.P0_dbm = intercept;
    sonuc.n = -slope / 10.0;

    if (sonuc.n < 0.1) {
        sonuc.n = 0.1;
    }

    // RSSI residual hesapları
    std::vector<double> residuals;
    residuals.reserve(N);

    for (const auto& o : ornekler) {
        if (o.mesafe_m <= 0.0) {
            continue;
        }

        const double tahmin = logDistanceRssi(o.mesafe_m, sonuc.P0_dbm, sonuc.n, d0_m);
        residuals.push_back(o.rssi_dbm - tahmin);
    }

    double res_sum = 0.0;
    for (double r : residuals) {
        res_sum += r;
    }

    sonuc.rssi_residual_mean = res_sum / static_cast<double>(residuals.size());

    double var_sum = 0.0;
    for (double r : residuals) {
        const double e = r - sonuc.rssi_residual_mean;
        var_sum += e * e;
    }

    sonuc.sigma2_rssi = var_sum / static_cast<double>(residuals.size());
    sonuc.rssi_residual_std = std::sqrt(sonuc.sigma2_rssi);

    // Çok küçük çıkarsa EKF aşırı güvenmesin diye alt sınır
    if (sonuc.sigma2_rssi < 4.0) {
        sonuc.sigma2_rssi = 4.0;
        sonuc.rssi_residual_std = 2.0;
    }

    // Bearing bias / std hesapları
    std::vector<double> bearing_errors;
    bearing_errors.reserve(ornekler.size());

    for (const auto& o : ornekler) {
        double e = aciSar180(o.raw_bearing_deg - o.true_bearing_deg);
        bearing_errors.push_back(e);
    }

    sonuc.bearing_bias_deg = acisalOrtalamaDeg(bearing_errors);

    double b_var_sum = 0.0;
    for (double e : bearing_errors) {
        const double centered = aciSar180(e - sonuc.bearing_bias_deg);
        b_var_sum += centered * centered;
    }

    const double bearing_var = b_var_sum / static_cast<double>(bearing_errors.size());
    sonuc.bearing_std_deg = std::sqrt(bearing_var);

    // Çok küçük çıkarsa EKF aşırı güvenmesin diye alt sınır
    if (sonuc.bearing_std_deg < 3.0) {
        sonuc.bearing_std_deg = 3.0;
    }

    // ASAMA 6: RSSI residual std'sini (dB, mesafeden bagimsiz varsayilan)
    // Friis turevi uzerinden mesafeyle-orantili bir katsayiya ceviriyoruz.
    // n cok kucukse (dumduz path-loss egrisi) katsayi patlayabilir - ust
    // sinir koyuyoruz ki EKF/PF tanı ciktisi anlamsiz buyumesin.
    sonuc.sigma_mesafe_katsayisi = std::min(
        sigmaMesafeFriisDen(sonuc.rssi_residual_std, 1.0, sonuc.n),
        10.0
    );

    sonuc.valid = true;
    return sonuc;
}

double sigmaMesafeFriisDen(double sigma_rssi_db, double mesafe_m, double n) {
    const double n_guvenli = std::max(n, 0.1);
    const double d_guvenli = std::max(mesafe_m, 0.0);
    return std::abs(sigma_rssi_db) * d_guvenli * std::log(10.0) / (10.0 * n_guvenli);
}

double sigmaMesafeAdaptif(double mesafe_m, const RfKalibrasyonSonucu& kalibrasyon) {
    if (!kalibrasyon.valid) {
        return 0.0;
    }

    return kalibrasyon.sigma_mesafe_katsayisi * std::max(mesafe_m, 0.0);
}

double sigmaToplamHesapla(const RfKalibrasyonSonucu& kalibrasyon) {
    return kalibrasyon.sigma2_rssi + kalibrasyon.govde_ek_varyans_db2;
}