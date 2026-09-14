
#include "aci_yardimci.hpp"
#include <cmath>

namespace AciYardimci {

double aci_normalize_et(double aci_deg) {
    double sonuc = std::fmod(aci_deg, 360.0);

    if (sonuc < 0.0) {
        sonuc += 360.0;
    }

    return sonuc;
}

double derece_to_radyan(double derece) {
    return derece * M_PI / 180.0;
}

double radyan_to_derece(double radyan) {
    return radyan * 180.0 / M_PI;
}

double aci_farki_hesapla(double aci1_deg, double aci2_deg) {
    double fark = aci_normalize_et(aci1_deg - aci2_deg);

    if (fark > 180.0) {
        fark -= 360.0;
    }

    return fark;
}

double kuresel_yon_hesapla(double yerel_yon_acisi,
                           double iha_bas_acisi,
                           double anten_sapma_duzeltmesi) {
    return aci_normalize_et(iha_bas_acisi + yerel_yon_acisi + anten_sapma_duzeltmesi);
}

}