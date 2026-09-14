// konum_orkestrasyonu.hpp
//
// Bu dosya, yon_bulma modülünün ürettiği YonSonucu çıktısını, band_hz'e göre
// doğru RF kalibrasyonuyla birlikte önce parçacık filtresine (PF, kaba
// kestirim), sonra EKF'ye (ince ayar) taşıyan orkestrasyon katmanıdır.
//
// Amaç:
// - band_hz'e göre doğru kalibrasyonu otomatik seçmek (RfKalibrasyonDeposu üzerinden)
// - Aynı İHA'nın farklı bantlardan (genlik-tabanlı bearing + menzil-only)
//   gelen YonSonucu çıktılarını aynı hedef takibine beslemek
// - Çağıran tarafın EkfOlcum/ParticleFilterMeasurement'ı elle doldurmasını gereksiz kılmak
// - PF'yi kaba/birincil kestirim, EKF'yi PF çıktısını inceltip yumuşatan
//   ikinci aşama olarak çalıştırmak: her güncellemede önce PF çalışır, PF
//   güveni belirli bir eşiği geçerse EKF o noktaya yeniden başlatılır, sonra
//   EKF güncel ölçümle ince ayar yapar.
#ifndef KONUM_ORKESTRASYONU_HPP
#define KONUM_ORKESTRASYONU_HPP

#include "../ortak/rf_kalibrasyon_deposu.hpp"
#include "../ekf/ekf_konum.hpp"
#include "../parcacik_filtresi/particle_filter.hpp"
#include "../../yon_bulma/ortak/yon_sonucu.hpp"

// Ölçümü yapan platformun (İHA ya da yer istasyonu) sabit/anlık konumu.
struct GozlemciKonumu {
    double x = 0.0;
    double y = 0.0;
    double irtifa = 0.0;
};

class KonumOrkestrasyonu {
public:
    // pf_parcacik_sayisi: YER TUTUCU (400) - gercek saha performansina gore
    // ayarlanmali (fazla parcacik = daha dogru ama daha yavas).
    KonumOrkestrasyonu(double d0_m, double alan_boyutu_m, int pf_parcacik_sayisi = 400);

    void kalibrasyon_kaydet(double band_hz, const RfKalibrasyonSonucu& kalibrasyon);

    bool kalibrasyon_var_mi(double band_hz) const;

    void takibi_baslat(double baslangic_x, double baslangic_y, double baslangic_belirsizlik_m);

    bool takip_hazir_mi() const;

    // yon_sonucu.gecerli == false ise ya da yon_sonucu.band_hz için kayıtlı
    // kalibrasyon yoksa, ne PF ne EKF güncellenir, valid=false dönülür.
    EkfTahminSonucu yon_sonucuyla_guncelle(
        const YonSonucu& yon_sonucu,
        const GozlemciKonumu& gozlemci
    );

private:
    RfKalibrasyonDeposu kalibrasyon_deposu;
    EkfKonumDurumu ekf;
    ParticleFilterState pf;
    double d0_m;
    double alan_boyutu_m;
    int pf_parcacik_sayisi;
};

#endif
