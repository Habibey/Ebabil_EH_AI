// yon_sonucu.hpp
//
// Bu dosya, yön bulma modüllerinin ortak çıktı yapısını tanımlar.
// İHA üzerindeki genlik tabanlı yön bulma modülü bu formatı kullanır;
// menzil-only (bearing yok) ölçümler de (bkz. KamciMenzilOkuyucu) aynı
// yapıya, sadece kuresel_yon_acisi doldurulmadan (düşük guven ile) konur.
//
// Amaç:
// - Her bant için yön sonucunu standartlaştırmak
// - Konum kestirimi modülüne temiz veri aktarmak
// - Arayüzde gösterilecek yön bilgisini düzenli tutmak
#pragma once

#include <string>

enum class PlatformTipi {
    IHA,
    YER
};

struct YonSonucu {
    double band_hz = 0.0;
    PlatformTipi platform = PlatformTipi::IHA;

    double yerel_yon_acisi = 0.0;
    double kuresel_yon_acisi = 0.0;

    double rssi_ana = 0.0;
    double rssi_sol = 0.0;
    double rssi_sag = 0.0;
    double rssi_farki = 0.0;

    double guven = 0.0;
    bool gecerli = false;

    std::string aciklama;
};