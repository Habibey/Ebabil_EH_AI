"""
data/real_captures/<label>/*.npz altındaki tüm ham kayıtları, model_v5'in
eğitim formatına (X: (N,128,7) özellik, Y: (N,11) one-hot) dönüştürür.

Bu sürüm ESKİ sürümden İKİ önemli konuda farklı:

1) TREN/DOĞRULAMA SIZINTISI (leakage) DÜZELTİLDİ. Eskiden tüm pencereler tek
   bir listeye atılıp finetune.py RASTGELE %80/%20 bölüyordu. Ardışık
   pencereler aynı ~10sn'lik kayıttan geldiği için (ve PlutoSDR TX döngüsel
   arabellek kullandığından neredeyse aynı bit örüntüsünü tekrarladığı için)
   komşu pencereler birbirine çok benziyor -- rastgele bölme, val setine
   train'dekiyle neredeyse birebir aynı örnekleri sızdırıyor, yani raporlanan
   val_accuracy olduğundan iyimser çıkıyordu. Bunun yerine HER dosya kendi
   içinde ZAMAN SIRASINA göre bölünüyor: dosyanın ilk %(1-VAL_FRACTION)'i
   train, son %VAL_FRACTION'ı val -- val, train'in hiç görmediği bir zaman
   dilimini temsil ediyor (sızıntı tam sıfırlanmaz ama çok azalır).

2) GÜRÜLTÜ AUGMENTASYONU eklendi. Elimizdeki her sınıf için tek bir kayıt
   oturumu var (aynı SNR/ortam) -- bu da modelin öğrenebileceği gerçek
   çeşitliliği sınırlıyor. Train pencerelerine (SADECE train, val'a DOKUNULMAZ)
   birkaç farklı SNR seviyesinde sentetik AWGN eklenerek varyantlar üretiliyor;
   böylece model gerçek RF karakteristiğini koruyarak farklı gürültü
   seviyelerine karşı biraz daha dayanıklı hale gelir.

Çıktı: data/real_dataset/X_real_train.npy, Y_real_train.npy,
       data/real_dataset/X_real_val.npy, Y_real_val.npy
(Eski tek-parça X_real.npy/Y_real.npy artık ÜRETİLMİYOR -- finetune.py da
 buna göre güncellendi.)
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
sys.path.insert(0, os.path.join(_REPO_ROOT, "src"))

import numpy as np

from predict import CLASSES, iq_to_v5_features

WINDOW = 128
STRIDE = 128  # örtüşmesiz -- bkz. önceki MemoryError notu

VAL_FRACTION = 0.2  # her dosyanın SON %20'si -- zaman bazlı, sızıntıyı azaltmak için

# Train pencereleri için augmentasyon: her pencereden 1 temiz (orijinal, gerçek
# kanal gürültüsüyle) + bu kadar gürültü-enjekte edilmiş varyant üretilir.
# 0 yapıldı (09.09.2026) -- artık pluto_seed_capture.py ile 4 farklı GERÇEK SNR
# seviyesinde (5 oturum/sınıf) veri topladık, yapay AWGN'e eskisi kadar ihtiyaç
# yok; ayrıca 5 kat veri + augment ikiye katlayınca bellek yetmiyordu
# (~5.74GB'lık tek dizi MemoryError verdi, 16GB RAM'in sadece 7.6GB'ı boştu).
AUGMENT_NOISY_COPIES = 0
AUGMENT_SNR_DB_CHOICES = [-15, -10, -5, 0, 5, 10, 15, 20]

CAPTURES_DIR = os.path.join(_REPO_ROOT, "data", "real_captures")
OUT_DIR = os.path.join(_REPO_ROOT, "data", "real_dataset")

_rng = np.random.default_rng(42)


def windows_from_samples(samples: np.ndarray):
    n = len(samples)
    for start in range(0, n - WINDOW + 1, STRIDE):
        yield samples[start:start + WINDOW]


def add_awgn(window: np.ndarray, snr_db: float) -> np.ndarray:
    sig_power = np.mean(np.abs(window) ** 2)
    if sig_power <= 0:
        return window
    noise_power = sig_power / (10 ** (snr_db / 10))
    noise = np.sqrt(noise_power / 2) * (_rng.standard_normal(len(window)) + 1j * _rng.standard_normal(len(window)))
    return (window + noise).astype(np.complex64)


def features_for_window(window: np.ndarray):
    I = np.real(window).astype(np.float32)
    Q = np.imag(window).astype(np.float32)
    return iq_to_v5_features(I, Q)


def main():
    if not os.path.isdir(CAPTURES_DIR):
        print(f"[-] {CAPTURES_DIR} bulunamadı -- önce capture_and_label.py ile kayıt al.")
        return

    X_train, Y_train = [], []
    X_val, Y_val = [], []
    train_count = {c: 0 for c in CLASSES}
    val_count = {c: 0 for c in CLASSES}

    for label in sorted(os.listdir(CAPTURES_DIR)):
        label_dir = os.path.join(CAPTURES_DIR, label)
        if not os.path.isdir(label_dir) or label not in CLASSES:
            continue
        label_idx = CLASSES.index(label)
        y = np.zeros(len(CLASSES), dtype=np.float32)
        y[label_idx] = 1.0

        for fname in sorted(os.listdir(label_dir)):
            if not fname.endswith(".npz"):
                continue
            fpath = os.path.join(label_dir, fname)
            samples = np.load(fpath)["samples"]

            file_windows = list(windows_from_samples(samples))
            if not file_windows:
                continue
            n_val = max(1, int(len(file_windows) * VAL_FRACTION)) if len(file_windows) > 1 else 0
            split_at = len(file_windows) - n_val

            for window in file_windows[:split_at]:
                X_train.append(features_for_window(window))
                Y_train.append(y)
                train_count[label] += 1
                for _ in range(AUGMENT_NOISY_COPIES):
                    snr_db = _rng.choice(AUGMENT_SNR_DB_CHOICES)
                    X_train.append(features_for_window(add_awgn(window, snr_db)))
                    Y_train.append(y)
                    train_count[label] += 1

            for window in file_windows[split_at:]:
                X_val.append(features_for_window(window))
                Y_val.append(y)
                val_count[label] += 1

    if not X_train:
        print("[-] Hiç pencere üretilemedi -- data/real_captures boş veya dosyalar bozuk.")
        return

    feature_mean = np.load(os.path.join(_REPO_ROOT, "scalers", "v5_feature_mean.npy"))
    feature_std = np.load(os.path.join(_REPO_ROOT, "scalers", "v5_feature_std.npy"))

    def to_arrays(X_list, Y_list):
        X = np.stack(X_list).astype(np.float32)
        Y = np.stack(Y_list).astype(np.float32)
        X -= feature_mean
        X /= feature_std
        return X, Y

    X_train_arr, Y_train_arr = to_arrays(X_train, Y_train)
    X_val_arr, Y_val_arr = to_arrays(X_val, Y_val)

    os.makedirs(OUT_DIR, exist_ok=True)
    np.save(os.path.join(OUT_DIR, "X_real_train.npy"), X_train_arr)
    np.save(os.path.join(OUT_DIR, "Y_real_train.npy"), Y_train_arr)
    np.save(os.path.join(OUT_DIR, "X_real_val.npy"), X_val_arr)
    np.save(os.path.join(OUT_DIR, "Y_real_val.npy"), Y_val_arr)

    print(f"[+] Train: {len(X_train_arr)} pencere (augment dahil), Val: {len(X_val_arr)} pencere (temiz, sızıntısız)")
    print(f"    -> {OUT_DIR}/X_real_train.npy, Y_real_train.npy, X_real_val.npy, Y_real_val.npy")
    print("\n--- Sınıf başına pencere sayısı (train / val) ---")
    for label in CLASSES:
        marker = "" if train_count[label] > 0 else "  (VERİ YOK)"
        print(f"  {label:8s}: train={train_count[label]:6d}  val={val_count[label]:5d}{marker}")


if __name__ == "__main__":
    main()
