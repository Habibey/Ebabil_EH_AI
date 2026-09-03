"""
Etiketli gerçek RF kaydı almak için CLI aracı.

Kullanım (RTL-SDR ile, örn. FM yayını için):
  python tools/dataset/capture_and_label.py --label WBFM --freq 104.6 --duration 20

--freq verilmezse (WBFM için) FM bandını tarayıp en güçlü istasyonu otomatik bulur
(test_fm_classify.py'deki tarama mantığının aynısı).

Sayısal sınıflar (BPSK, QAM16 vb.) için PlutoSDR TX + RTL-SDR RX loopback
kurulunca bu script değişmeden kullanılabilir -- sadece --freq PlutoSDR'ın
yayın yaptığı frekans olacak.

Çıktı: data/real_captures/<label>/<timestamp>.npz
  - samples: complex64 ham I/Q dizisi
  - label, freq_mhz, sample_rate, captured_at: metadata
"""
import argparse
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
sys.path.insert(0, os.path.join(_REPO_ROOT, "src"))
os.add_dll_directory(os.path.join(_REPO_ROOT, "tools", "rtlsdr"))

import numpy as np
from rtlsdr import RtlSdr

from predict import CLASSES

DEFAULT_SAMPLE_RATE = 250000
SCAN_START_MHZ = 88.0
SCAN_STOP_MHZ = 108.0
SCAN_STEP_MHZ = 0.2


def scan_for_strongest_fm(sdr: RtlSdr) -> float:
    print(f"[*] {SCAN_START_MHZ}-{SCAN_STOP_MHZ} MHz FM bandı taranıyor...")
    best_freq, best_power = SCAN_START_MHZ, -np.inf
    freq = SCAN_START_MHZ
    while freq <= SCAN_STOP_MHZ:
        sdr.center_freq = freq * 1e6
        power = np.mean(np.abs(sdr.read_samples(8192)) ** 2)
        if power > best_power:
            best_power, best_freq = power, freq
        freq += SCAN_STEP_MHZ
    print(f"[+] En güçlü frekans: {best_freq:.1f} MHz (güç={best_power:.4f})")
    return best_freq


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--label", required=True, choices=CLASSES, help="Bu kaydın gerçek etiketi (model sınıflarından biri)")
    parser.add_argument("--freq", type=float, default=None, help="Merkez frekans (MHz). WBFM için verilmezse otomatik taranır.")
    parser.add_argument("--duration", type=float, default=15.0, help="Kayıt süresi (saniye), varsayılan 15")
    parser.add_argument("--sample-rate", type=float, default=DEFAULT_SAMPLE_RATE, help="Örnekleme hızı (Hz)")
    parser.add_argument("--gain", default="auto", help="Kazanç ('auto' veya sayısal dB değeri)")
    args = parser.parse_args()

    if args.freq is None:
        if args.label != "WBFM":
            parser.error("--freq zorunlu (sadece WBFM için otomatik FM taraması var)")

    sdr = RtlSdr()
    sdr.sample_rate = args.sample_rate
    try:
        sdr.gain = float(args.gain)
    except ValueError:
        sdr.gain = args.gain  # "auto"

    try:
        freq_mhz = args.freq
        if freq_mhz is None:
            freq_mhz = scan_for_strongest_fm(sdr)

        sdr.center_freq = freq_mhz * 1e6
        n_samples = int(args.duration * args.sample_rate)
        print(f"[*] {freq_mhz:.3f} MHz'de {args.duration:.1f}s ({n_samples} örnek) kayıt alınıyor -- etiket: {args.label}")

        sdr.read_samples(8192)  # retune sonrası geçici/kararsız örnekleri at

        # Tek seferde çok büyük blok istemek USB okuma zaman aşımına yol açıyor
        # -- küçük parçalar (256k örnek) halinde okuyup birleştiriyoruz.
        CHUNK = 262144
        chunks = []
        remaining = n_samples
        while remaining > 0:
            n = min(CHUNK, remaining)
            chunks.append(sdr.read_samples(n))
            remaining -= n
        samples = np.concatenate(chunks).astype(np.complex64)

        out_dir = os.path.join(_REPO_ROOT, "data", "real_captures", args.label)
        os.makedirs(out_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(out_dir, f"{timestamp}.npz")
        np.savez_compressed(
            out_path,
            samples=samples,
            label=args.label,
            freq_mhz=freq_mhz,
            sample_rate=args.sample_rate,
            captured_at=timestamp,
        )
        print(f"[+] Kaydedildi: {out_path} ({len(samples)} örnek, {len(samples) // 128} pencere üretilebilir)")

    finally:
        sdr.close()


if __name__ == "__main__":
    main()
