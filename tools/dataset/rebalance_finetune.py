"""
finetune.py'nin standart (örnek-sayısına-göre) class_weight'i, WBFM/AM-SSB/
AM-DSB dengeleyince modelin sayısal aile içinde (BPSK/8PSK/QAM64) gerilemesine
yol açtı -- karışıklık matrisi bunların PAM4/QAM16'ya kaçtığını gösterdi,
analog sınıflara değil. Bu script, ÖRNEK SAYISI yerine GÜNCEL RECALL'E göre
ağırlıklandırma yapıp (zayıf sınıfa daha çok ağırlık) MEVCUT finetuned
modelden (sıfırdan değil) düşük öğrenme oranıyla kısa bir ek tur atıyor --
amaç WBFM/AM-SSB/AM-DSB'nin kazanımını yıkmadan BPSK/8PSK/QAM64/CPFSK/GFSK'yi
kurtarmak.

Çıktı ayrı bir dosyaya (teknofest_model_v5_finetuned_v5.keras) yazılır --
mevcut v4 dosyası DEĞİŞMEZ, karşılaştırıp elle karar veririz.
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
sys.path.insert(0, os.path.join(_REPO_ROOT, "src"))

import numpy as np
import tensorflow as tf
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau
from tensorflow.keras.optimizers import Adam

from predict import CLASSES

MODEL_IN = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_finetuned.keras")  # v4 -- SIFIRDAN DEĞİL
MODEL_OUT = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_finetuned_v5.keras")
DATASET_DIR = os.path.join(_REPO_ROOT, "data", "real_dataset")

# v4'ün ölçülen recall'ine göre elle belirlendi -- zayıf sınıf yüksek ağırlık,
# zaten çok iyi olan (ve PAM4 gibi "sığınılan") sınıflar düşük ağırlık.
MANUEL_AGIRLIKLAR = {
    "WBFM": 0.3, "AM-SSB": 0.3, "AM-DSB": 0.6, "PAM4": 0.5,
    "QPSK": 1.0, "QAM16": 1.3,
    "GFSK": 1.8, "CPFSK": 2.0,
    "BPSK": 2.8, "8PSK": 3.2, "QAM64": 3.2,
}

LEARNING_RATE = 3e-5  # v4'ten devam ettiğimiz için finetune.py'den bile düşük -- ince ayar
EPOCHS = 15


def main():
    x_train_path = os.path.join(DATASET_DIR, "X_real_train.npy")
    y_train_path = os.path.join(DATASET_DIR, "Y_real_train.npy")
    x_val_path = os.path.join(DATASET_DIR, "X_real_val.npy")
    y_val_path = os.path.join(DATASET_DIR, "Y_real_val.npy")
    if not all(os.path.exists(p) for p in (x_train_path, y_train_path, x_val_path, y_val_path)):
        print(f"[-] {DATASET_DIR} altında X/Y_real_train/val.npy bulunamadı.")
        return
    if not os.path.exists(MODEL_IN):
        print(f"[-] {MODEL_IN} bulunamadı -- önce finetune.py çalışmış olmalı.")
        return

    X_train = np.load(x_train_path)
    Y_train = np.load(y_train_path)
    X_val = np.load(x_val_path)
    Y_val = np.load(y_val_path)
    print(f"[*] Train: X={X_train.shape}  |  Val: X={X_val.shape}")

    class_weight = {CLASSES.index(c): w for c, w in MANUEL_AGIRLIKLAR.items()}
    print("[*] Manuel (recall-tabanlı) class_weight:")
    for c in CLASSES:
        print(f"    {c:8s}: weight={class_weight[CLASSES.index(c)]:.2f}")

    print(f"[*] Mevcut model yükleniyor ({MODEL_IN})...")
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
    model = tf.keras.models.load_model(MODEL_IN)

    print("[*] ÖNCESİ (v4) genel performans:")
    loss_before, acc_before = model.evaluate(X_val, Y_val, verbose=0)
    print(f"    val_loss={loss_before:.4f}  val_accuracy={acc_before:.4f}")

    model.compile(loss="categorical_crossentropy", optimizer=Adam(learning_rate=LEARNING_RATE), metrics=["accuracy"])

    callbacks = [
        EarlyStopping(monitor="val_loss", patience=4, restore_best_weights=True),
        ReduceLROnPlateau(monitor="val_loss", factor=0.5, patience=2, min_lr=1e-6),
    ]

    print("[*] Yeniden dengeleme turu başlıyor...")
    model.fit(
        X_train, Y_train,
        validation_data=(X_val, Y_val),
        batch_size=32,
        epochs=EPOCHS,
        callbacks=callbacks,
        class_weight=class_weight,
        verbose=1,
    )

    print("\n[*] SONRASI genel performans:")
    loss_after, acc_after = model.evaluate(X_val, Y_val, verbose=0)
    print(f"    val_loss={loss_after:.4f}  val_accuracy={acc_after:.4f}  (öncesi: {acc_before:.4f})")

    preds = model.predict(X_val, verbose=0, batch_size=512)
    pred_idx = np.argmax(preds, axis=1)
    true_idx = np.argmax(Y_val, axis=1)
    print("\n--- Sınıf başına recall (SONRASI) ---")
    for i, c in enumerate(CLASSES):
        mask = true_idx == i
        if mask.sum() == 0:
            continue
        recall = (pred_idx[mask] == i).mean()
        print(f"    {c:8s}: recall={recall*100:5.1f}%")

    model.save(MODEL_OUT)
    print(f"\n[+] Kaydedildi: {MODEL_OUT}")
    print("[!] v4 (teknofest_model_v5_finetuned.keras) DEĞİŞMEDİ -- karşılaştırıp elle karar ver.")


if __name__ == "__main__":
    main()
