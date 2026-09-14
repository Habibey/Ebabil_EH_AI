#ifndef MAVLINK_CERCEVE_HPP
#define MAVLINK_CERCEVE_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

// TelemetriGonderici (RPi/İHA tarafı, radyoya yazan/okuyan) ve
// yer_istasyonu_koprusu (Jetson/yer istasyonu tarafı, radyodan okuyan)
// AYNI 915MHz hattındaki MAVLink çerçevelerini tanımak için bu saf
// fonksiyonu paylaşır -- iki tarafta da ayrı ayrı yazılıp senkron
// kalması riski taşıyan bir kopya OLMAMASI için buraya çıkarıldı.
namespace ebabil {

// Bir MAVLink çerçevesinin (v1 ya da v2) TAM bayt uzunluğunu hesaplar.
// tampon[0]'ın 0xFE (v1) ya da 0xFD (v2) olduğu ZATEN kontrol edilmiş
// olmalı -- bu fonksiyon sadece uzunluğu hesaplar, magic byte kontrolü
// yapmaz. Henüz uzunluğu belirlemek için yeterli bayt yoksa (v1 için 2,
// v2 için 3 bayt gerekiyor -- imzalı/imzasız ayrımı için) std::nullopt
// döner; çağıran taraf daha fazla veri gelene kadar beklemeli.
//
// v1: STX(1) LEN(1) SEQ(1) SYSID(1) COMPID(1) MSGID(1) PAYLOAD(LEN) CRC(2)
//     -> toplam = 6 + LEN + 2
// v2: STX(1) LEN(1) INCOMPAT_FLAGS(1) COMPAT_FLAGS(1) SEQ(1) SYSID(1)
//     COMPID(1) MSGID(3) PAYLOAD(LEN) CRC(2) [SIGNATURE(13) -- INCOMPAT_FLAGS
//     bit0 (0x01) set ise]
//     -> toplam = 10 + LEN + 2 (+13 imzalıysa)
inline std::optional<size_t> mavlinkCerceveUzunlugu(const std::string& tampon) {
    if (tampon.empty()) {
        return std::nullopt;
    }
    const uint8_t magic = static_cast<uint8_t>(tampon[0]);
    if (magic == 0xFE) {
        if (tampon.size() < 2) {
            return std::nullopt;
        }
        const uint8_t payload_len = static_cast<uint8_t>(tampon[1]);
        return static_cast<size_t>(6 + payload_len + 2);
    }
    if (magic == 0xFD) {
        if (tampon.size() < 3) {
            return std::nullopt;
        }
        const uint8_t payload_len = static_cast<uint8_t>(tampon[1]);
        const uint8_t incompat_flags = static_cast<uint8_t>(tampon[2]);
        const bool imzali = (incompat_flags & 0x01) != 0;
        return static_cast<size_t>(10 + payload_len + 2 + (imzali ? 13 : 0));
    }
    return std::nullopt;  // MAVLink degil -- ASCII satir olarak islenecek
}

}  // namespace ebabil

#endif
