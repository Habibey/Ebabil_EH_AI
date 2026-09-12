"""
Ayrı, ikili (Analog/Sayısal) bir sınıflandırıcı eğitir -- 11-sınıflı ana modelden
TAMAMEN BAĞIMSIZ. Amaç: classify_iq_gated'de güven eşiği altına düşen (ve bu
yüzden Analog/Sayısal de "Belirsiz"e dönen) durumlarda, çok daha kolay olan bu
2-sınıflı problemde ayrı bir ikinci görüş sağlamak.

Veri: data/real_dataset/X_real_train.npy + Y_real_train.npy (bkz.
build_training_set.py) -- zaten (128,7) özellik çıkarılmış ve normalize edilmiş,
11-sınıflı one-hot etiketli. Burada SADECE etiketler predict.ANALOG_CLASSES'e
göre ikili (Analog=1/Sayısal=0) olarak yeniden gruplanıyor, öznitelikler aynen
kullanılıyor -- ayrı bir veri toplama/işleme adımına gerek yok.

Mimari kasıtlı olarak LSTM DEĞİL (ana modeldeki gibi) -- sadece Conv1D + GAP.
Sebep: (1) ikili ayrım için LSTM'in dizisel hafızasına ihtiyaç yok, zarf/
periyodiklik istatistikleri conv katmanlarıyla zaten yakalanıyor, (2) LSTM'in
TFLite dönüşümünde yarattığı sorunları (bkz. convert_to_tflite.py'deki
unroll=True çözümü) bu model hiç yaşamayacak, Jetson'a taşımak da kolay olur.

Mevcut 11-sınıflı model (teknofest_model_v5_finetuned.keras) HİÇ değiştirilmiyor
-- çıktı tamamen ayrı bir dosyaya (models/analog_sayisal_binary.keras) yazılır.
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
sys.path.insert(0, os.path.join(_REPO_ROOT, "src"))

import numpy as np
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau
from tensorflow.keras.optimizers import Adam

from predict import CLASSES, ANALOG_CLASSES

DATASET_DIR = os.path.join(_REPO_ROOT, "data", "real_dataset")
MODEL_OUT = os.path.join(_REPO_ROOT, "models", "analog_sayisal_binary.keras")

LEARNING_RATE = 1e-3  # sıfırdan eğitim -- fine-tuning değil, orijinal Adam varsayılanı
EPOCHS = 40
BATCH_SIZE = 64
USE_AUGMENTED_DATA = False  # bkz. main() -- ilk denemede augment tıkanmaya sebep olmuş gibi duruyor

_ANALOG_IDX = np.array([1.0 if c in ANALOG_CLASSES else 0.0 for c in CLASSES], dtype=np.float32)


def to_binary_labels(Y_onehot: np.ndarray) -> np.ndarray:
    """(N,11) one-hot -> (N,1) ikili etiket (1=Analog, 0=Sayısal)."""
    return (Y_onehot @ _ANALOG_IDX).reshape(-1, 1).astype(np.float32)


def build_model(input_shape):
    inputs = layers.Input(shape=input_shape)
    x = layers.Conv1D(32, 7, padding="same", activation="relu")(inputs)
    x = layers.BatchNormalization()(x)
    x = layers.Conv1D(64, 5, padding="same", activation="relu")(x)
    x = layers.BatchNormalization()(x)
    x = layers.MaxPooling1D(2)(x)
    x = layers.Conv1D(64, 3, padding="same", activation="relu")(x)
    x = layers.BatchNormalization()(x)
    x = layers.GlobalAveragePooling1D()(x)
    x = layers.Dense(32, activation="relu")(x)
    x = layers.Dropout(0.3)(x)
    outputs = layers.Dense(1, activation="sigmoid")(x)
    return tf.keras.Model(inputs, outputs)


def main():
    x_train_path = os.path.join(DATASET_DIR, "X_real_train.npy")
    y_train_path = os.path.join(DATASET_DIR, "Y_real_train.npy")
    x_val_path = os.path.join(DATASET_DIR, "X_real_val.npy")
    y_val_path = os.path.join(DATASET_DIR, "Y_real_val.npy")
    if not all(os.path.exists(p) for p in (x_train_path, y_train_path, x_val_path, y_val_path)):
        print(f"[-] {DATASET_DIR} altında X/Y_real_train/val.npy bulunamadı -- önce build_training_set.py çalıştır.")
        return

    X_train = np.load(x_train_path)
    Y_train = to_binary_labels(np.load(y_train_path))
    X_val = np.load(x_val_path)
    Y_val = to_binary_labels(np.load(y_val_path))

    if not USE_AUGMENTED_DATA:
        # build_training_set.py her pencereyi (temiz, gurultulu) sirasiyla
        # ekliyor (AUGMENT_NOISY_COPIES=1) -- yani X_train[0::2] temiz,
        # X_train[1::2] -15..+20dB yapay gurultu eklenmis kopyalar. Ilk
        # denemede train accuracy %74'te tikanip val'in altinda kalmisti --
        # asiri gurultulu (-15dB gibi) kopyalarin sinifi gercekten
        # ayirt edilemez hale getirip modeli yanlis yone cektiginden
        # supheleniyoruz. Sadece temiz verilerle deneyelim.
        X_train = X_train[0::2]
        Y_train = Y_train[0::2]
        print(f"[*] USE_AUGMENTED_DATA=False -- sadece temiz pencereler kullaniliyor ({len(X_train)} ornek).")

    n_analog = int(Y_train.sum())
    n_digital = len(Y_train) - n_analog
    print(f"[*] Train: X={X_train.shape}  Analog={n_analog}  Sayısal={n_digital}")
    print(f"[*] Val:   X={X_val.shape}  Analog={int(Y_val.sum())}  Sayısal={len(Y_val) - int(Y_val.sum())}")

    # 3 analog / 8 sayısal sınıf var -- dengesiz, class_weight ile telafi.
    class_weight = {
        0: len(Y_train) / (2 * n_digital),
        1: len(Y_train) / (2 * n_analog),
    }
    print(f"[*] class_weight: {class_weight}")

    model = build_model(X_train.shape[1:])
    model.compile(loss="binary_crossentropy", optimizer=Adam(learning_rate=LEARNING_RATE), metrics=["accuracy"])
    model.summary()

    callbacks = [
        EarlyStopping(monitor="val_loss", patience=6, restore_best_weights=True),
        ReduceLROnPlateau(monitor="val_loss", factor=0.5, patience=3, min_lr=1e-6),
    ]

    print("[*] Eğitim başlıyor...")
    model.fit(
        X_train, Y_train,
        validation_data=(X_val, Y_val),
        batch_size=BATCH_SIZE,
        epochs=EPOCHS,
        callbacks=callbacks,
        class_weight=class_weight,
        verbose=1,
    )

    print("\n[*] Değerlendirme (val seti, zaman-bazlı ayrık, sızıntısız):")
    loss, acc = model.evaluate(X_val, Y_val, verbose=0)
    print(f"    val_loss={loss:.4f}  val_accuracy={acc:.4f}")

    preds = (model.predict(X_val, verbose=0) >= 0.5).astype(np.float32)
    tp = int(((preds == 1) & (Y_val == 1)).sum())
    tn = int(((preds == 0) & (Y_val == 0)).sum())
    fp = int(((preds == 1) & (Y_val == 0)).sum())
    fn = int(((preds == 0) & (Y_val == 1)).sum())
    print(f"    Analog->Analog (doğru)  : {tp}/{tp+fn}")
    print(f"    Analog->Sayısal (yanlış): {fn}/{tp+fn}")
    print(f"    Sayısal->Sayısal (doğru): {tn}/{tn+fp}")
    print(f"    Sayısal->Analog (yanlış): {fp}/{tn+fp}")

    model.save(MODEL_OUT)
    print(f"\n[+] Kaydedildi: {MODEL_OUT}")
    print("[!] Mevcut 11-sınıflı model (teknofest_model_v5_finetuned.keras) DEĞİŞMEDİ.")


if __name__ == "__main__":
    main()
