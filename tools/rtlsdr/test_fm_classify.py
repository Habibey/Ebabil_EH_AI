"""
RTL-SDR ile gerçek bir FM radyo yayını yakalayip modelin bunu WBFM olarak
tanip tanimadigina bakan tek seferlik dogrulama scripti.

Adimlar:
  1. 88-108 MHz FM bandini 0.2 MHz adimlarla tarar, her adimda kisa bir
     ornek alip ortalama gucu olcer.
  2. En guclu frekansi kilitler (gercek bir yayin oldugunu varsayarak).
  3. O frekansta uzunca bir I/Q kaydi alir, 128 orneklik pencerelere boler.
  4. Her pencereyi predict.py'deki AYNI ozellik cikarma + normalizasyon +
     model pipeline'indan gecirir, sonuclari sayar.

Beklenti: WBFM en sik cikan sinif olmali (FM yayini oldugu icin).
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_THIS_DIR, "..", "..", "src"))
os.add_dll_directory(_THIS_DIR)

import numpy as np
import tensorflow as tf
from rtlsdr import RtlSdr
from collections import Counter

from predict import CLASSES, ANALOG_CLASSES, iq_to_v5_features

REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
MODEL_PATH = os.path.join(REPO_ROOT, "models", "teknofest_model_v5_best.keras")
MEAN_PATH = os.path.join(REPO_ROOT, "scalers", "v5_feature_mean.npy")
STD_PATH = os.path.join(REPO_ROOT, "scalers", "v5_feature_std.npy")

SCAN_START_MHZ = 88.0
SCAN_STOP_MHZ = 108.0
SCAN_STEP_MHZ = 0.2
SAMPLE_RATE = 250000  # RTL-SDR'ın desteklediği en düşük aralık (225k-300k), RadioML'in 200kHz'ine en yakın
CAPTURE_WINDOWS = 200  # kaç adet 128 örneklik pencere test edilecek


def scan_for_strongest_fm(sdr: RtlSdr) -> float:
    print(f"[*] {SCAN_START_MHZ}-{SCAN_STOP_MHZ} MHz FM bandı taranıyor...")
    best_freq = SCAN_START_MHZ
    best_power = -np.inf
    freq = SCAN_START_MHZ
    while freq <= SCAN_STOP_MHZ:
        sdr.center_freq = freq * 1e6
        samples = sdr.read_samples(8192)
        power = np.mean(np.abs(samples) ** 2)
        if power > best_power:
            best_power = power
            best_freq = freq
        freq += SCAN_STEP_MHZ
    print(f"[+] En güçlü frekans: {best_freq:.1f} MHz (güç={best_power:.4f})")
    return best_freq


def main():
    print("[*] Model yükleniyor...")
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
    model = tf.keras.models.load_model(MODEL_PATH)
    feature_mean = np.load(MEAN_PATH)
    feature_std = np.load(STD_PATH)

    sdr = RtlSdr()
    sdr.sample_rate = SAMPLE_RATE
    sdr.gain = "auto"

    try:
        target_freq = scan_for_strongest_fm(sdr)
        sdr.center_freq = target_freq * 1e6

        print(f"[*] {target_freq:.1f} MHz'de {CAPTURE_WINDOWS} pencerelik kayıt alınıyor...")
        total_samples = CAPTURE_WINDOWS * 128
        raw = sdr.read_samples(total_samples + 4096)[4096:4096 + total_samples]  # ilk örnekler genelde gürültülü/geçici

        results = Counter()
        for i in range(CAPTURE_WINDOWS):
            window = raw[i * 128:(i + 1) * 128]
            I = np.real(window).astype(np.float32)
            Q = np.imag(window).astype(np.float32)

            features = iq_to_v5_features(I, Q)
            processed = np.expand_dims(features, axis=0).astype(np.float32)
            processed = (processed - feature_mean) / feature_std

            pred = model.predict(processed, verbose=0)
            idx = np.argmax(pred)
            mod = CLASSES[idx]
            conf = float(np.max(pred)) * 100
            results[mod] += 1
            if i < 10 or mod == "WBFM":
                print(f"  [{i:03d}] {mod:8s} (%{conf:5.1f})  {'Analog' if mod in ANALOG_CLASSES else 'Sayısal'}")

        print("\n=== SONUÇ (sınıf dağılımı) ===")
        for mod, count in results.most_common():
            print(f"  {mod:8s}: {count:3d}/{CAPTURE_WINDOWS}  (%{100 * count / CAPTURE_WINDOWS:.1f})")

        top_mod, top_count = results.most_common(1)[0]
        if top_mod == "WBFM":
            print(f"\n[BAŞARILI] En sık çıkan sınıf WBFM ({100 * top_count / CAPTURE_WINDOWS:.0f}%) -- model gerçek FM yayınını doğru yakalıyor.")
        else:
            print(f"\n[DİKKAT] En sık çıkan sınıf WBFM değil, {top_mod} -- model gerçek veride beklenen gibi çalışmıyor olabilir.")

    finally:
        sdr.close()


if __name__ == "__main__":
    main()
