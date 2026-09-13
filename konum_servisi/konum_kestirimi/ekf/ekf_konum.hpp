#ifndef EKF_KONUM_HPP
#define EKF_KONUM_HPP

#include "../ortak/rf_kalibrasyon.hpp"

struct EkfKonumDurumu {
    double x = 0.0;   // hedef x tahmini [m]
    double y = 0.0;   // hedef y tahmini [m]

    double P[2][2] = {
        {450.0 * 450.0, 0.0},
        {0.0, 450.0 * 450.0}
    };

    double Q[2][2] = {
        {0.05 * 0.05, 0.0},
        {0.0, 0.05 * 0.05}
    };

    double R[2][2] = {
        {10.0 * 10.0, 0.0},   // bearing variance [deg^2]
        {0.0, 25.0}           // RSSI variance [dB^2]
    };

    bool initialized = false;
};

struct EkfOlcum {
    double iha_x = 0.0;
    double iha_y = 0.0;
    double iha_alt = 0.0;

    double bearing_deg = 0.0;
    double rssi_dbm = 0.0;

    // AoA/bearing ölçüm güveni: 0.0 düşük güven, 1.0 yüksek güven
    double bearing_confidence = 1.0;
};

// CEP tabanlı hata elipsi (PDF: eh_guven_skoru_kilavuzu, bkz. calculateErrorEllipse).
// major_m / minor_m: belirtilen güven seviyesindeki elips yarı-eksenleri [m].
// angle_rad: büyük eksenin +x'e göre açısı [rad], atan2(2*cov_xy, var_x-var_y)/2.
// conf_level: 0.50 = CEP50, 0.90 = CEP90, 0.95 = CEP95.
struct HataElipsi {
    double major_m   = 0.0;
    double minor_m   = 0.0;
    double angle_rad = 0.0;
    double conf_level = 0.90;
};

struct EkfTahminSonucu {
    double hedef_x = 0.0;
    double hedef_y = 0.0;
    double belirsizlik = 0.0;   // sqrt(trace(P)) — geriye dönük uyumluluk
    HataElipsi elips;           // CEP90 hata elipsi
    bool valid = false;
};

void ekfBaslat(
    EkfKonumDurumu& ekf,
    double baslangic_x,
    double baslangic_y,
    double baslangic_belirsizlik_m
);

void ekfKovaryansAyarla(
    EkfKonumDurumu& ekf,
    const RfKalibrasyonSonucu& kalibrasyon
);

// UYARI (2026-08-27, gercek testte dogrulandi): Bu fonksiyonu TEK BASINA
// (parcacik filtresi olmadan) production/gercek veri hattinda cagirmayin.
// Baslangic tahmininden uzak (orn. arama alaninin kosesindeki) bir hedefte
// dogrusallastirma tuzagina dusup ~450m hataya kadar YANLIS AMA KENDINDEN
// EMIN (dusuk raporlanan belirsizlikle) bir sonuc verebiliyor. Gercek/
// production kullanimda DAIMA KonumOrkestrasyonu::yon_sonucuyla_guncelle()
// uzerinden cagirin (o, once ParcacikFiltresi ile kaba/kuresel bir tahmin
// yapip EKF'yi gerektiginde oraya sifirliyor). Bu dosyanin dogrudan
// cagrilmasi sadece birim test/karsilastirma amaçlidir (bkz. testler/).
EkfTahminSonucu ekfGuncelleBearingRssi(
    EkfKonumDurumu& ekf,
    const EkfOlcum& olcum,
    const RfKalibrasyonSonucu& kalibrasyon,
    double d0_m,
    double area_size_m
);

#endif