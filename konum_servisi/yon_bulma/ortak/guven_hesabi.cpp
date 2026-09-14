#include "guven_hesabi.hpp"
#include <cmath>
#include <algorithm>

namespace GuvenHesabi {

double sinirla_0_1(double deger) {
    return std::max(0.0, std::min(1.0, deger));
}

double iha_guven_hesapla(double rssi_sol,
                         double rssi_sag,
                         double en_dusuk_rssi,
                         double anlamli_fark_db) {
    double rssi_ortalama = (rssi_sol + rssi_sag) / 2.0;
    double rssi_farki = std::abs(rssi_sag - rssi_sol);

    double seviye_guveni = (rssi_ortalama - en_dusuk_rssi) / 30.0;
    seviye_guveni = sinirla_0_1(seviye_guveni);

    double fark_guveni = rssi_farki / anlamli_fark_db;
    fark_guveni = sinirla_0_1(fark_guveni);

    return sinirla_0_1(0.6 * seviye_guveni + 0.4 * fark_guveni);
}

}