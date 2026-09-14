#ifndef RF_KALIBRASYON_DEPOSU_HPP
#define RF_KALIBRASYON_DEPOSU_HPP

#include <unordered_map>
#include "rf_kalibrasyon.hpp"

// band_hz'e gore ayri RfKalibrasyonSonucu (P0, n, sigma, bearing bias/std)
// tutan depo. EKF ve parcacik filtresi band-agnostik calisir; hangi bandin
// olcumu isleniyorsa o banda ait kalibrasyonu bu depodan alip cagirana
// (orkestrasyon katmanina) tasimak gerekir.
class RfKalibrasyonDeposu {
public:
    void kalibrasyonKaydet(double band_hz, const RfKalibrasyonSonucu& kalibrasyon);

    const RfKalibrasyonSonucu& kalibrasyonGetir(double band_hz) const;

    bool bantKayitliMi(double band_hz) const;

private:
    std::unordered_map<long long, RfKalibrasyonSonucu> depo;
    RfKalibrasyonSonucu gecersiz_kalibrasyon;
};

#endif
