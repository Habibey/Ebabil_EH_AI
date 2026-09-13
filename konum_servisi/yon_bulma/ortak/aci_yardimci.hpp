// aci_yardimci.hpp
//
// Bu dosya, yön bulma modüllerinde kullanılan ortak açı işlemlerini tanımlar.
//
// Amaç:
// - Açıları 0-360 derece aralığında tutmak
// - Derece/radyan dönüşümleri yapmak
// - İki açı arasındaki en kısa farkı hesaplamak
// - İHA heading bilgisi ile yerel yön açısını küresel yön açısına çevirmek
//
// Bu dosya hem İHA yön bulma hem de yer istasyonu yön bulma modülleri tarafından kullanılır.
#pragma once

namespace AciYardimci {

double aci_normalize_et(double aci_deg);

double derece_to_radyan(double derece);

double radyan_to_derece(double radyan);

double aci_farki_hesapla(double aci1_deg, double aci2_deg);

double kuresel_yon_hesapla(double yerel_yon_acisi,
                           double iha_bas_acisi,
                           double anten_sapma_duzeltmesi = 0.0);

}