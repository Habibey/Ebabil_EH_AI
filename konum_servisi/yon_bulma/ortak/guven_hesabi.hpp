// guven_hesabi.hpp
//
// Bu dosya, yön bulma sonucunun güven değerini hesaplamak için kullanılır.
//
// Amaç:
// - RSSI seviyesi yeterli mi kontrol etmek
// - İki anten arasındaki RSSI farkı anlamlı mı kontrol etmek
// - Sonuç güvenini 0 ile 1 arasında standart bir değere dönüştürmek
//
// DUZELTME (2026-09-01): Yer istasyonunun motorlu Yagi taramasi (bearing
// bulma) kaldirildi - PF+EKF artik sadece Iha'nin genlik-tabanli (anten
// cifti) olcumleriyle, farkli ucus noktalarindan hedefin konumunu buluyor;
// yer istasyonundan hedefe olan yon, konum bulunduktan sonra basit bir
// geometri hesabiyla (atan2) cikarilabiliyor, ayrica fiziksel bir taramaya
// gerek yok. Bu yuzden yer_guven_hesapla() de kaldirildi (sadece o modul
// kullaniyordu).
namespace GuvenHesabi {

double sinirla_0_1(double deger);

// Not: kalibrasyon kalitesine bağlı 3. güven terimi (C_kal) burada değil,
// konum_kestirimi/orkestrasyon/konum_orkestrasyonu.cpp'de ekleniyor - çünkü
// kalibrasyon verisi (RfKalibrasyonSonucu) konum_kestirimi katmanında
// yaşıyor, yon_bulma bu katmana bağımlı olmamalı (bkz. o dosyadaki not).
double iha_guven_hesapla(double rssi_sol,
                         double rssi_sag,
                         double en_dusuk_rssi = -95.0,
                         double anlamli_fark_db = 2.0);

}