"""
Sürekli tespit loglama -- her SYS güncellemesini zaman damgasıyla bir CSV
dosyasına ekler (C++ tarafındaki TespitKaydedici'nin Python karşılığı,
bkz. ebabil-eh-backend/parametreCikarimi/ebabil_sdr/TespitKaydedici.h).
streamer.py ve pluto_ed_scanner.py ORTAK kullanıyor -- konum_istemcisi.py
ile AYNI "paylaşılan yardımcı modül" deseni.

Neden CSV (JSON değil): saha sonrası Excel/pandas ile kolayca açılabilir,
append-only (her satır bağımsız -- süreç ortasında çökse/kesilse bile önceki
satırlar sağlam kalır; tek bir JSON dizisi yarım kalan bir yazmada TÜM
dosyayı bozabilirdi).

Kayıt konumu: data/tespit_gunlugu/<YYYYMMDD_HHMMSS>[_onek].csv -- her süreç
başlangıcında YENİ bir dosya (eskilerin üzerine yazılmaz), dosya adı hangi
çalıştırmaya ait olduğunu zaman damgasıyla gösterir. data/ zaten
.gitignore'da -- bu loglar commit'lenmez, sadece yerel/sahada kalır.
"""
import csv
import os
import time

_REPO_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
_LOG_DIR = os.path.join(_REPO_ROOT, "data", "tespit_gunlugu")

_ALAN_ADLARI = [
    "zaman_unix", "zaman_okunur", "hedef_id", "freq_mhz", "power_dbm",
    "bant_genisligi_khz", "sapma_mhz", "gurultu_tabani_db", "snr_db", "sureklilik",
]


class TespitKaydedici:
    """Her .kaydet(tracker, tid) çağrısında tracker.known[tid]'deki (SYS ile
    AYNI kaynak, bkz. build_sys_fields) güncel bilgiyi bir CSV satırına
    ekler. Süreç çökse bile son satırlar kaybolmasın diye HER satırda
    flush() yapılır -- CPU açısından ucuz (saniyede birkaç kez çağrılıyor,
    disk yazma darboğaz değil)."""

    def __init__(self, dosya_onek=""):
        os.makedirs(_LOG_DIR, exist_ok=True)
        zaman_damgasi = time.strftime("%Y%m%d_%H%M%S")
        dosya_adi = f"{zaman_damgasi}{('_' + dosya_onek) if dosya_onek else ''}.csv"
        self._yol = os.path.join(_LOG_DIR, dosya_adi)
        self._dosya = open(self._yol, "w", newline="", encoding="utf-8")
        self._yazici = csv.writer(self._dosya)
        self._yazici.writerow(_ALAN_ADLARI)
        self._dosya.flush()
        print(f"[TESPİT LOG] Kaydediliyor: {self._yol}")

    def kaydet(self, tracker, tid):
        if tid not in tracker.known:
            return
        info = tracker.known[tid]
        snr_db = info.get("snr_db")
        sapma_mhz = info.get("freq_sapmasi_mhz")
        gurultu_db = info["power_db"] - snr_db if snr_db is not None else None
        simdi = time.time()
        satir = [
            f"{simdi:.3f}",
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(simdi)),
            tid,
            f"{info['freq_mhz']:.3f}",
            f"{info['power_db']:.2f}",
            f"{info['bandwidth_khz']:.1f}",
            f"{sapma_mhz:.4f}" if sapma_mhz is not None else "",
            f"{gurultu_db:.2f}" if gurultu_db is not None else "",
            f"{snr_db:.2f}" if snr_db is not None else "",
            tracker.sureklilik_durumu(tid),
        ]
        self._yazici.writerow(satir)
        self._dosya.flush()

    def kapat(self):
        self._dosya.close()
