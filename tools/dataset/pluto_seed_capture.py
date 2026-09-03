"""
PlutoSDR'dan sırayla bilinen modülasyonlar yayınlayıp, aynı anda RTL-SDR ile
gerçek zamanlı kaydeden "tohum veri" toplama scripti.

ÖNEMLİ: streamer.py ÇALIŞMIYOR olmalı -- RTL-SDR tek donanım, aynı anda iki
süreç birden açamaz. Bu script RTL-SDR'ı KENDİSİ açıp kapatıyor.

Kullanım:
  python tools/dataset/pluto_seed_capture.py
  python tools/dataset/pluto_seed_capture.py --classes BPSK,QPSK,QAM16 --duration 8

Çıktı, capture_and_label.py ile TAM AYNI formatta: data/real_captures/<ETİKET>/
-- build_training_set.py ayrıca bir değişiklik gerekmeden bunları da işler.
"""
import argparse
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
sys.path.insert(0, os.path.join(_REPO_ROOT, "src"))
sys.path.insert(0, _THIS_DIR)
os.environ["PATH"] = os.path.join(_REPO_ROOT, "tools", "libiio") + os.pathsep + os.environ["PATH"]
os.add_dll_directory(os.path.join(_REPO_ROOT, "tools", "rtlsdr"))

import numpy as np
import adi
from rtlsdr import RtlSdr

from predict import CLASSES
import modulators

TX_FREQ_MHZ = float(os.environ.get("EBABIL_TX_FREQ_MHZ", 433.0))
RX_SAMPLE_RATE = 250000
# dB cinsinden zayıflatma (0 = tam güç, negatif = daha az güç). RTL-SDR'ı
# doyurmamak için düşük tutuluyor -- gerekirse ortam değişkeniyle ayarla.
TX_GAIN_DB = float(os.environ.get("EBABIL_TX_GAIN_DB", -40.0))
THROWAWAY_SAMPLES = 4096
CHUNK = 262144  # tek seferde büyük blok istemek USB zaman aşımına yol açıyor


def capture_rx(rtl, duration_s):
    n_samples = int(duration_s * RX_SAMPLE_RATE)
    parts = []
    remaining = n_samples
    while remaining > 0:
        n = min(CHUNK, remaining)
        parts.append(rtl.read_samples(n))
        remaining -= n
    return np.concatenate(parts).astype(np.complex64)


def save_capture(samples, label, freq_mhz):
    out_dir = os.path.join(_REPO_ROOT, "data", "real_captures", label)
    os.makedirs(out_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"{timestamp}.npz")
    np.savez_compressed(
        out_path, samples=samples, label=label, freq_mhz=freq_mhz,
        sample_rate=RX_SAMPLE_RATE, captured_at=timestamp,
    )
    print(f"    [+] Kaydedildi: {out_path} ({len(samples)} örnek, {len(samples) // 128} pencere üretilebilir)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--classes", default=",".join(CLASSES),
                         help="Virgülle ayrılmış sınıf listesi (varsayılan: hepsi)")
    parser.add_argument("--duration", type=float, default=10.0, help="Sınıf başına kayıt süresi (sn), varsayılan 10")
    args = parser.parse_args()
    classes = [c.strip().upper() for c in args.classes.split(",")]

    unknown = [c for c in classes if c not in modulators.GENERATORS]
    if unknown:
        print(f"[-] Bilinmeyen sınıf(lar): {unknown}. Geçerli: {list(modulators.GENERATORS)}")
        return

    print(f"[*] PlutoSDR TX ve RTL-SDR RX açılıyor (frekans: {TX_FREQ_MHZ} MHz, TX kazanç: {TX_GAIN_DB} dB)...")
    pluto = adi.Pluto("ip:192.168.2.1")
    pluto.tx_lo = int(TX_FREQ_MHZ * 1e6)
    pluto.sample_rate = modulators.PLUTO_TX_SAMPLE_RATE
    pluto.tx_hardwaregain_chan0 = TX_GAIN_DB
    pluto.tx_cyclic_buffer = True

    rtl = RtlSdr()
    rtl.sample_rate = RX_SAMPLE_RATE
    rtl.gain = "auto"
    rtl.center_freq = TX_FREQ_MHZ * 1e6

    print(f"[*] {len(classes)} sınıf için kayıt alınacak, sınıf başına {args.duration:.1f}s\n")

    try:
        for label in classes:
            print(f"[*] {label} yayınlanıyor...")
            waveform = modulators.GENERATORS[label]()
            # Pluto DAC'ının beklediği ölçeğe getir (fazla yüksek olursa kırpılır/bozulur).
            waveform = waveform / np.max(np.abs(waveform)) * 0.7 * (2 ** 14)

            pluto.tx_destroy_buffer()
            pluto.tx(waveform)

            time.sleep(0.3)  # TX'in oturması için kısa bekleme
            rtl.read_samples(THROWAWAY_SAMPLES)

            samples = capture_rx(rtl, args.duration)
            power_db = 10 * np.log10(np.mean(np.abs(samples) ** 2) + 1e-12)
            print(f"    Yakalanan ortalama güç: {power_db:.1f} dB (göreceli, dBm değil -- "
                  f"çok düşükse TX kazancını artır, çok yüksek/kırpılmış görünüyorsa azalt)")

            save_capture(samples, label, TX_FREQ_MHZ)

            pluto.tx_destroy_buffer()
            time.sleep(0.2)

        print("\n[+] Tüm sınıflar tamamlandı. Sırada: python tools/dataset/build_training_set.py")

    finally:
        try:
            pluto.tx_destroy_buffer()
        except Exception:
            pass
        rtl.close()


if __name__ == "__main__":
    main()
