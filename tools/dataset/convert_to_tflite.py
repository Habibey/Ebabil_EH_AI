"""Eğitilmiş .keras modelini .tflite'a çevirir -- Jetson gibi tam TensorFlow'un
(CUDA/cuDNN) kurulamadığı zayıf donanımlarda tflite_runtime ile inference
yapabilmek için. Windows PC'de (tam TensorFlow kurulu olan yerde) çalıştır,
çıkan .tflite dosyasını Jetson'a scp ile taşı (bkz. src/predict.py:_TFLiteModel).
"""
import os

import tensorflow as tf

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")

MODEL_IN = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_finetuned.keras")
MODEL_OUT = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_finetuned.tflite")

if __name__ == "__main__":
    model = tf.keras.models.load_model(MODEL_IN)

    # Model bir LSTM katmanı içeriyor -- Keras'ın varsayılan dinamik unroll'u
    # (TensorList/WHILE op'ları) TFLite'ta ya dönüştürme aşamasında ya da
    # çalışma zamanında ("variable != nullptr") hataya yol açıyor. Aynı
    # ağırlıklarla, LSTM'i unroll=True (128 adımı sabit/statik olarak açan)
    # bir kopyasını kurup onu dönüştürmek bu sınıf hataların hepsini ortadan
    # kaldırıyor -- sekans uzunluğumuz zaten hep sabit 128.
    config = model.get_config()
    for layer_cfg in config["layers"]:
        if layer_cfg["class_name"] == "LSTM":
            layer_cfg["config"]["unroll"] = True
    static_model = type(model).from_config(config)
    static_model.set_weights(model.get_weights())

    converter = tf.lite.TFLiteConverter.from_keras_model(static_model)
    tflite_model = converter.convert()

    with open(MODEL_OUT, "wb") as f:
        f.write(tflite_model)

    print(f"[+] TFLite modeli yazıldı: {MODEL_OUT} ({len(tflite_model) / 1024:.1f} KB)")
