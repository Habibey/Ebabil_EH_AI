#include "rf_kalibrasyon_deposu.hpp"

#include <cmath>

namespace {

long long bandAnahtari(double band_hz) {
    return static_cast<long long>(std::llround(band_hz));
}

}

void RfKalibrasyonDeposu::kalibrasyonKaydet(
    double band_hz,
    const RfKalibrasyonSonucu& kalibrasyon
) {
    depo[bandAnahtari(band_hz)] = kalibrasyon;
}

const RfKalibrasyonSonucu& RfKalibrasyonDeposu::kalibrasyonGetir(double band_hz) const {
    auto it = depo.find(bandAnahtari(band_hz));

    if (it == depo.end()) {
        return gecersiz_kalibrasyon;
    }

    return it->second;
}

bool RfKalibrasyonDeposu::bantKayitliMi(double band_hz) const {
    return depo.find(bandAnahtari(band_hz)) != depo.end();
}
