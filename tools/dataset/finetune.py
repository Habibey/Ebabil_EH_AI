"""
data/real_dataset/X_real.npy + Y_real.npy (bkz. build_training_set.py) ile
mevcut modeli fine-tune eder.

ÖNEMLİ SINIRLAMA: Elimizde orijinal RadioML sentetik eğitim verisi yok (sadece
kayıtlı normalizasyon istatistikleri var), bu yüzden fine-tuning SADECE yeni
gerçek veriyle yapılıyor -- düşük öğrenme oranı + az epoch ile, modelin
sentetik veride öğrendiklerini büyük ölçüde unutmasını önlemeye çalışıyoruz
(tam garanti değil, "catastrophic forgetting" riski var). Sadece gerçek
veride veri toplanan sınıflar güncellenir; hiç gerçek örneği olmayan sınıflar
bu fine-tuning turunda hiç görülmez.

Orijinal model DEĞİŞTİRİLMEZ (models/teknofest_model_v5_best.keras korunur)
-- çıktı ayrı bir dosyaya (models/teknofest_model_v5_finetuned.keras) yazılır,
istersen predict.py'de MODEL_PATH'i değiştirip geçersin, istersen eskisinde kal.
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

MODEL_IN = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_best.keras")
MODEL_OUT = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_finetuned.keras")
DATASET_DIR = os.path.join(_REPO_ROOT, "data", "real_dataset")

LEARNING_RATE = 1e-4  # orijinal eğitimdeki adam varsayılanından (1e-3) kasıtlı düşük -- ince ayar, yıkıcı unutma riskini azaltır
EPOCHS = 30
BATCH_SIZE = 32  # gerçek veri seti küçük olacağı için 1024 (orijinal) yerine küçük batch


def main():
    x_train_path = os.path.join(DATASET_DIR, "X_real_train.npy")
    y_train_path = os.path.join(DATASET_DIR, "Y_real_train.npy")
    x_val_path = os.path.join(DATASET_DIR, "X_real_val.npy")
    y_val_path = os.path.join(DATASET_DIR, "Y_real_val.npy")
    if not all(os.path.exists(p) for p in (x_train_path, y_train_path, x_val_path, y_val_path)):
        print(f"[-] {DATASET_DIR} altında X/Y_real_train/val.npy bulunamadı -- önce build_training_set.py çalıştır.")
        return

    X_train = np.load(x_train_path)
    Y_train = np.load(y_train_path)
    X_val = np.load(x_val_path)
    Y_val = np.load(y_val_path)
    print(f"[*] Train: X={X_train.shape}, Y={Y_train.shape}  |  Val (zaman-bazlı ayrık, augment YOK): "
          f"X={X_val.shape}, Y={Y_val.shape}")

    present_classes = [CLASSES[i] for i in np.unique(np.argmax(Y_train, axis=1))]
    missing_classes = [c for c in CLASSES if c not in present_classes]
    print(f"[*] Veride bulunan sınıflar: {present_classes}")
    if missing_classes:
        print(f"[!] Veride HİÇ örneği olmayan sınıflar (bu turda güncellenmeyecek): {missing_classes}")

    if len(present_classes) < 2:
        print(
            f"\n[DURDURULDU] Veri setinde sadece {len(present_classes)} sınıf var ({present_classes}). "
            "Tek sınıfla fine-tuning modele 'her şey bu sınıf' demeyi öğretir ve diğer TÜM sınıfları "
            "kırar -- mevcut modelden daha kötü sonuç verir. En az 2-3 farklı sınıftan gerçek veri "
            "toplayana kadar bu scripti çalıştırma."
        )
        return

    # Sınıf başına örnek sayısı dengesiz (örn. WBFM diğerlerinden ~3 kat fazla
    # kayıt içeriyor) -- class_weight ile azınlık sınıfların loss'a katkısı
    # örnek sayısı kadar bastırılmasın diye ağırlıklandırılıyor.
    train_labels = np.argmax(Y_train, axis=1)
    class_counts = np.bincount(train_labels, minlength=len(CLASSES))
    n_total = len(train_labels)
    class_weight = {
        i: (n_total / (len(CLASSES) * count)) if count > 0 else 0.0
        for i, count in enumerate(class_counts)
    }
    print("[*] class_weight (dengesizliği telafi etmek için):")
    for i, c in enumerate(CLASSES):
        if class_counts[i] > 0:
            print(f"    {c:8s}: n={class_counts[i]:6d}  weight={class_weight[i]:.2f}")

    print("[*] Mevcut model yükleniyor...")
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
    model = tf.keras.models.load_model(MODEL_IN)

    print("[*] Fine-tuning ÖNCESİ gerçek veri üzerindeki performans:")
    loss_before, acc_before = model.evaluate(X_val, Y_val, verbose=0)
    print(f"    val_loss={loss_before:.4f}  val_accuracy={acc_before:.4f}")

    model.compile(loss="categorical_crossentropy", optimizer=Adam(learning_rate=LEARNING_RATE), metrics=["accuracy"])

    callbacks = [
        EarlyStopping(monitor="val_loss", patience=5, restore_best_weights=True),
        ReduceLROnPlateau(monitor="val_loss", factor=0.5, patience=2, min_lr=1e-6),
    ]

    print("[*] Fine-tuning başlıyor...")
    model.fit(
        X_train, Y_train,
        validation_data=(X_val, Y_val),
        batch_size=BATCH_SIZE,
        epochs=EPOCHS,
        callbacks=callbacks,
        class_weight=class_weight,
        verbose=1,
    )

    print("\n[*] Fine-tuning SONRASI gerçek veri üzerindeki performans:")
    loss_after, acc_after = model.evaluate(X_val, Y_val, verbose=0)
    print(f"    val_loss={loss_after:.4f}  val_accuracy={acc_after:.4f}  (öncesi: {acc_before:.4f})")

    model.save(MODEL_OUT)
    print(f"\n[+] Kaydedildi: {MODEL_OUT}")
    print("[!] Orijinal model (teknofest_model_v5_best.keras) DEĞİŞMEDİ.")
    print("[!] Kullanmak istersen src/predict.py'deki model_path'i bu yeni dosyaya çevir.")


if __name__ == "__main__":
    main()
