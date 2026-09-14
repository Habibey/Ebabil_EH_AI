#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <random>
#include <vector>
#include "../ortak/rf_kalibrasyon.hpp"

struct Particle {
    double x = 0.0;
    double y = 0.0;
    double weight = 1.0;
};

struct ParticleFilterMeasurement {
    double uav_x = 0.0;
    double uav_y = 0.0;
    double uav_alt = 0.0;

    double bearing_deg = 0.0;
    double rssi_dbm = 0.0;
    double bearing_confidence = 1.0;
};

struct ParticleFilterState {
    std::vector<Particle> particles;

    double area_size_m = 1200.0;
    bool initialized = false;

    // DUZELTME (2026-08-31): resample/jitter RNG'leri onceden fonksiyon-ici
    // static'ti - yani AYNI PROCESS icinde olusturulan TUM ParticleFilterState
    // ornekleri bu RNG akisini PAYLASIYORDU (bkz. particleFilterBaslat'taki
    // seed parametresi bu yuzden resample/jitter'i etkilemiyordu). Coklu
    // hedef takibi ya da testler/rota_degerlendirici.cpp gibi ayni process
    // icinde art arda cok sayida PF calistiran senaryolarda, her PF'in
    // sonucu kendinden ONCEKI PF'lerin RNG durumuna bagli hale geliyordu -
    // bagimsiz/tekrarlanabilir olmuyordu. Artik her ornegin kendi RNG'si var,
    // particleFilterBaslat() icinde seed'den turetiliyor.
    std::mt19937 resample_rng;
    std::mt19937 jitter_rng;
};

struct ParticleFilterResult {
    double target_x = 0.0;
    double target_y = 0.0;

    // Parcacik bulutunun uzamsal yayiliminden turetilir (agirliktan DEGIL -
    // resample sonrasi agirliklar hep ~1/N'e esitlendigi icin agirlik
    // tabanli bir guven anlamsizlasirdi). Parcaciklar ne kadar siki
    // kumelenmisse confidence o kadar yuksek.
    double confidence = 0.0;

    // Parcacik bulutunun ham uzamsal yayilimi (metre, konum standart
    // sapmasi). confidence'in normalize edilmemis hali - EKF'nin kendi
    // belirsizligiyle DOGRUDAN (metre biriminde) karsilastirmak icin.
    double yayilim_m = 0.0;

    bool valid = false;
};

void particleFilterBaslat(
    ParticleFilterState& pf,
    int particle_count,
    double area_size_m,
    unsigned int seed = 42
);

ParticleFilterResult particleFilterGuncelle(
    ParticleFilterState& pf,
    const ParticleFilterMeasurement& measurement,
    const RfKalibrasyonSonucu& calibration,
    double d0_m
);

ParticleFilterResult particleFilterTahminAl(
    const ParticleFilterState& pf
);

#endif