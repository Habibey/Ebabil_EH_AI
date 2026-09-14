#include "particle_filter.hpp"

#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double gaussianLikelihood(double error, double sigma) {
    sigma = std::max(sigma, 1e-6);
    const double z = error / sigma;
    return std::exp(-0.5 * z * z);
}

static double bearingFromToDeg(
    double from_x,
    double from_y,
    double to_x,
    double to_y
) {
    double angle = std::atan2(to_y - from_y, to_x - from_x) * 180.0 / M_PI;
    return aciSar180(angle);
}

static void normalizeWeights(std::vector<Particle>& particles) {
    double sum_w = 0.0;

    for (const auto& p : particles) {
        sum_w += p.weight;
    }

    if (sum_w <= 1e-300) {
        const double uniform_w = 1.0 / static_cast<double>(particles.size());
        for (auto& p : particles) {
            p.weight = uniform_w;
        }
        return;
    }

    for (auto& p : particles) {
        p.weight /= sum_w;
    }
}

static void systematicResample(std::vector<Particle>& particles, std::mt19937& rng) {
    const int N = static_cast<int>(particles.size());

    if (N <= 0) {
        return;
    }

    std::vector<Particle> new_particles;
    new_particles.reserve(N);

    std::uniform_real_distribution<double> dist(0.0, 1.0 / static_cast<double>(N));

    const double r = dist(rng);
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < N; ++m) {
        const double u = r + static_cast<double>(m) / static_cast<double>(N);

        while (u > c && i < N - 1) {
            i++;
            c += particles[i].weight;
        }

        Particle selected = particles[i];
        selected.weight = 1.0 / static_cast<double>(N);
        new_particles.push_back(selected);
    }

    particles = new_particles;
}

static void addSmallJitter(
    std::vector<Particle>& particles,
    double area_size_m,
    std::mt19937& rng
) {
    std::normal_distribution<double> noise(0.0, 3.0);

    // DUZELTME (2026-08-31): [0, area_size_m] yerine merkez-orijinli sinir -
    // bkz. ekf_konum.cpp'deki ayni duzeltmenin notu.
    const double sinir = area_size_m / 2.0;

    for (auto& p : particles) {
        p.x += noise(rng);
        p.y += noise(rng);

        p.x = std::min(std::max(p.x, -sinir), sinir);
        p.y = std::min(std::max(p.y, -sinir), sinir);
    }
}

void particleFilterBaslat(
    ParticleFilterState& pf,
    int particle_count,
    double area_size_m,
    unsigned int seed
) {
    if (particle_count <= 0) {
        particle_count = 1000;
    }

    pf.area_size_m = area_size_m;
    pf.particles.clear();
    pf.particles.reserve(particle_count);

    // DUZELTME (2026-08-31): [0, area_size_m] yerine merkez-orijinli baslangic
    // dagilimi - bkz. ekf_konum.cpp'deki ayni duzeltmenin notu.
    const double sinir = area_size_m / 2.0;

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-sinir, sinir);

    const double uniform_w = 1.0 / static_cast<double>(particle_count);

    for (int i = 0; i < particle_count; ++i) {
        Particle p;
        p.x = dist(rng);
        p.y = dist(rng);
        p.weight = uniform_w;
        pf.particles.push_back(p);
    }

    // Resample/jitter RNG'leri seed'den turetilip bu PF ornegine ozel
    // (paylasilmayan) hale getiriliyor - bkz. ParticleFilterState'teki not.
    pf.resample_rng.seed(seed + 1u);
    pf.jitter_rng.seed(seed + 2u);

    pf.initialized = true;
}

ParticleFilterResult particleFilterTahminAl(
    const ParticleFilterState& pf
) {
    ParticleFilterResult result;

    if (!pf.initialized || pf.particles.empty()) {
        result.valid = false;
        return result;
    }

    double x_sum = 0.0;
    double y_sum = 0.0;
    double w_sum = 0.0;

    for (const auto& p : pf.particles) {
        x_sum += p.x * p.weight;
        y_sum += p.y * p.weight;
        w_sum += p.weight;
    }

    if (w_sum <= 1e-300) {
        result.valid = false;
        return result;
    }

    result.target_x = x_sum / w_sum;
    result.target_y = y_sum / w_sum;

    // Guven, parcacik AGIRLIGINDAN degil, bulutun UZAMSAL YAYILIMINDAN
    // turetiliyor: resample sonrasi agirliklar hep ~1/N'e esitlendigi icin
    // max_w tabanli bir guven, PF ne kadar yakinsarsa yakinsasin sabit
    // kalirdi (dogrulandi). Parcaciklar tahmin etrafinda ne kadar siki
    // kumelenmisse confidence o kadar yuksek.
    double var_toplam = 0.0;

    for (const auto& p : pf.particles) {
        const double dx = p.x - result.target_x;
        const double dy = p.y - result.target_y;
        var_toplam += dx * dx + dy * dy;
    }

    const double yayilim = std::sqrt(var_toplam / static_cast<double>(pf.particles.size()));

    // YER TUTUCU: referans yayilim (alanin %25'i) - saha verisiyle ayarlanmali.
    const double referans_yayilim = 0.25 * pf.area_size_m;

    result.confidence = std::max(0.0, std::min(1.0, 1.0 - yayilim / referans_yayilim));
    result.yayilim_m = yayilim;
    result.valid = true;

    return result;
}

ParticleFilterResult particleFilterGuncelle(
    ParticleFilterState& pf,
    const ParticleFilterMeasurement& measurement,
    const RfKalibrasyonSonucu& calibration,
    double d0_m
) {
    ParticleFilterResult result;

    if (!pf.initialized || pf.particles.empty() || !calibration.valid) {
        result.valid = false;
        return result;
    }

    double bearing_sigma = std::max(calibration.bearing_std_deg, 3.0);
    // sigma2_rssi yerine sigmaToplamHesapla() - bkz. ekf_konum.cpp'deki
    // ayni degisikligin notu (Asama 4-5 govde/elektronik varyans hook'u).
    double rssi_sigma = std::sqrt(std::max(sigmaToplamHesapla(calibration), 4.0));

    const double conf = std::min(std::max(measurement.bearing_confidence, 0.15), 1.0);

    // Confidence düşükse bearing sigma büyür; yani bearing ölçümüne daha az güvenilir.
    bearing_sigma /= conf;

    // Ölçüm kapısı: mevcut PF tahminine göre yeni bearing çok tersse,
    // bearing etkisini yumuşat.
    ParticleFilterResult onceki_tahmin = particleFilterTahminAl(pf);

    if (onceki_tahmin.valid) {
        double beklenen_bearing = bearingFromToDeg(
            measurement.uav_x,
            measurement.uav_y,
            onceki_tahmin.target_x,
            onceki_tahmin.target_y
        );

        double gate_error = std::abs(
            aciSar180(measurement.bearing_deg - beklenen_bearing)
        );

        if (gate_error > 45.0) {
            bearing_sigma *= 3.0;
        }

        if (gate_error > 75.0) {
            bearing_sigma *= 6.0;
        }
    }

    for (auto& p : pf.particles) {
        const double expected_bearing = bearingFromToDeg(
            measurement.uav_x,
            measurement.uav_y,
            p.x,
            p.y
        );

        const double bearing_error = aciSar180(measurement.bearing_deg - expected_bearing);

        const double d3 = ucBoyutluMesafe(
            measurement.uav_x,
            measurement.uav_y,
            measurement.uav_alt,
            p.x,
            p.y
        );

        const double expected_rssi = logDistanceRssi(
            d3,
            calibration.P0_dbm,
            calibration.n,
            d0_m
        );

        const double rssi_error = measurement.rssi_dbm - expected_rssi;

        const double bearing_likelihood = gaussianLikelihood(bearing_error, bearing_sigma);
        const double rssi_likelihood = gaussianLikelihood(rssi_error, rssi_sigma);

        // Bearing ana yön bilgisini verir, RSSI mesafe/olasılık desteği sağlar.
        p.weight *= bearing_likelihood * rssi_likelihood;
    }

    normalizeWeights(pf.particles);
    systematicResample(pf.particles, pf.resample_rng);
    addSmallJitter(pf.particles, pf.area_size_m, pf.jitter_rng);
    normalizeWeights(pf.particles);

    return particleFilterTahminAl(pf);
}