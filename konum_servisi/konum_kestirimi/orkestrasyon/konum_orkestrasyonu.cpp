#include "konum_orkestrasyonu.hpp"
#include "../../yon_bulma/ortak/guven_hesabi.hpp"

#include <cmath>

namespace {

// YER TUTUCU: PF güveni bu eşiği geçerse EKF, PF'nin tahminine yeniden
// başlatılır. Gerçek kalibrasyon/saha verisiyle ayarlanmalı.
constexpr double PF_GUVEN_ESIGI = 0.3;

// PF ile yeniden başlatma sonrası EKF başlangıç belirsizliği artık SABİT
// DEĞİL — PF'nin o anki gerçek yayılımı (pf_sonuc.yayilim_m) kullanılıyor.
// Sabit bir değer (örn. 50m) kullanmak, sıfırlama+tek-güncelleme sonrası
// belirsizliğin hep o sabitin biraz üstünde kalmasına ve HER adımda tekrar
// sıfırlanmasına yol açtığı doğrulandı (60 iterasyon boyunca ~56m'de tıkalı
// kaldı, sıfırlama hiç durmadı). PF'nin kendi yayılımını hedef almak, PF
// yakınsadıkça hedefi de küçülterek bu tuzağı çözüyor.
//
// YER TUTUCU: PF yayılımı çok küçülürse (parçacık dejenerasyonu vb.) EKF'yi
// aşırı güvenli yapmamak için alt sınır. Gerçek saha verisiyle ayarlanmalı.
constexpr double PF_SIFIRLAMA_ALT_SINIR_M = 5.0;

// YER TUTUCU: C_kal (kalibrasyon kalitesi) türetme sınırları - bearing_std_deg
// bu değerden küçükse çok iyi kalibrasyon (C_kal=1), büyükse kötü (C_kal=0),
// arası lineer. Gerçek saha kalibrasyon verisiyle ayarlanmalı.
constexpr double BEARING_STD_IYI_ESIK_DEG = 3.0;
constexpr double BEARING_STD_KOTU_ESIK_DEG = 20.0;

// YER TUTUCU: yon_bulma'nın ham güveni (C_RSSI+C_ΔR) ile kalibrasyon
// kalitesi (C_kal) arasındaki karışım oranı.
constexpr double HAM_GUVEN_AGIRLIGI = 0.7;
constexpr double C_KAL_AGIRLIGI = 0.3;

double kalibrasyon_guveni_hesapla(const RfKalibrasyonSonucu& kalibrasyon) {
    const double oran =
        (kalibrasyon.bearing_std_deg - BEARING_STD_IYI_ESIK_DEG) /
        (BEARING_STD_KOTU_ESIK_DEG - BEARING_STD_IYI_ESIK_DEG);

    return GuvenHesabi::sinirla_0_1(1.0 - oran);
}

}

KonumOrkestrasyonu::KonumOrkestrasyonu(
    double d0_m_,
    double alan_boyutu_m_,
    int pf_parcacik_sayisi_
) : d0_m(d0_m_), alan_boyutu_m(alan_boyutu_m_), pf_parcacik_sayisi(pf_parcacik_sayisi_) {}

void KonumOrkestrasyonu::kalibrasyon_kaydet(
    double band_hz,
    const RfKalibrasyonSonucu& kalibrasyon
) {
    kalibrasyon_deposu.kalibrasyonKaydet(band_hz, kalibrasyon);
}

bool KonumOrkestrasyonu::kalibrasyon_var_mi(double band_hz) const {
    return kalibrasyon_deposu.bantKayitliMi(band_hz);
}

void KonumOrkestrasyonu::takibi_baslat(
    double baslangic_x,
    double baslangic_y,
    double baslangic_belirsizlik_m
) {
    ekfBaslat(ekf, baslangic_x, baslangic_y, baslangic_belirsizlik_m);
    particleFilterBaslat(pf, pf_parcacik_sayisi, alan_boyutu_m);
}

bool KonumOrkestrasyonu::takip_hazir_mi() const {
    return ekf.initialized;
}

EkfTahminSonucu KonumOrkestrasyonu::yon_sonucuyla_guncelle(
    const YonSonucu& yon_sonucu,
    const GozlemciKonumu& gozlemci
) {
    EkfTahminSonucu sonuc;

    if (!yon_sonucu.gecerli || !ekf.initialized) {
        sonuc.valid = false;
        return sonuc;
    }

    if (!kalibrasyon_var_mi(yon_sonucu.band_hz)) {
        sonuc.valid = false;
        return sonuc;
    }

    const RfKalibrasyonSonucu& kalibrasyon = kalibrasyon_deposu.kalibrasyonGetir(yon_sonucu.band_hz);

    // yon_bulma sadece RSSI-tabanli "ham" guveni (C_RSSI+C_ΔR) uretir; burada
    // kalibrasyon kalitesine bagli C_kal terimi eklenerek nihai guven
    // hesaplaniyor (bkz. guven_hesabi.hpp basindaki not).
    const double c_kal = kalibrasyon_guveni_hesapla(kalibrasyon);
    const double nihai_guven = GuvenHesabi::sinirla_0_1(
        HAM_GUVEN_AGIRLIGI * yon_sonucu.guven + C_KAL_AGIRLIGI * c_kal
    );

    // 1) Once parcacik filtresi (PF) kaba/birincil tahmini gunceller.
    ParticleFilterMeasurement pf_olcum;
    pf_olcum.uav_x = gozlemci.x;
    pf_olcum.uav_y = gozlemci.y;
    pf_olcum.uav_alt = gozlemci.irtifa;
    pf_olcum.bearing_deg = yon_sonucu.kuresel_yon_acisi;
    pf_olcum.rssi_dbm = yon_sonucu.rssi_ana;
    pf_olcum.bearing_confidence = nihai_guven;

    ParticleFilterResult pf_sonuc = particleFilterGuncelle(pf, pf_olcum, kalibrasyon, d0_m);

    // 2) PF yeterince guvenliyse, EKF'yi PF'nin tahminine yeniden baslatir -
    //    kaba kestirimden ince ayara gecis burada oluyor. Ama SADECE EKF'nin
    //    guncel belirsizligi PF'nin O ANKI yayiliminDAN DAHA KOTUYSE - yoksa
    //    EKF zaten daha iyi bir tahmine yakinsamisken PF onu geriye goturur.
    if (pf_sonuc.valid && pf_sonuc.confidence > PF_GUVEN_ESIGI) {
        const double ekf_guncel_belirsizlik = std::sqrt(ekf.P[0][0] + ekf.P[1][1]);
        const double pf_sifirlama_hedefi = std::max(pf_sonuc.yayilim_m, PF_SIFIRLAMA_ALT_SINIR_M);

        if (ekf_guncel_belirsizlik > pf_sifirlama_hedefi) {
            ekfBaslat(ekf, pf_sonuc.target_x, pf_sonuc.target_y, pf_sifirlama_hedefi);
        }
    }

    // 3) EKF, guncel olcumle ince ayar yapar.
    EkfOlcum olcum;
    olcum.iha_x = gozlemci.x;
    olcum.iha_y = gozlemci.y;
    olcum.iha_alt = gozlemci.irtifa;
    olcum.bearing_deg = yon_sonucu.kuresel_yon_acisi;
    olcum.rssi_dbm = yon_sonucu.rssi_ana;
    olcum.bearing_confidence = nihai_guven;

    ekfKovaryansAyarla(ekf, kalibrasyon);

    return ekfGuncelleBearingRssi(ekf, olcum, kalibrasyon, d0_m, alan_boyutu_m);
}
