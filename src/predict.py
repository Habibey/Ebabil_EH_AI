import zmq
import numpy as np
import os
import random

# Jetson gibi tam TensorFlow'un (CUDA/cuDNN/Docker) kurulamadığı zayıf
# donanımlarda tensorflow paketi hiç yok -- bu durumda tflite_runtime ile
# devam ediyoruz (bkz. _TFLiteModel), sadece inference yapıyoruz zaten,
# eğitim gerekmiyor.
try:
    import tensorflow as tf
    _HAS_TF = True
except ImportError:
    tf = None
    _HAS_TF = False

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..")

# Modülasyon Sınıfları
CLASSES = ['8PSK', 'AM-DSB', 'AM-SSB', 'BPSK', 'CPFSK', 'GFSK', 'PAM4', 'QAM16', 'QAM64', 'QPSK', 'WBFM']
ANALOG_CLASSES = {'AM-DSB', 'AM-SSB', 'WBFM'}  # Bu sınıfların dışındaki her şey "Sayısal" kabul edilir.
TARGET_ID = "HEDEF-1"


def iq_to_v5_features(I, Q):
    # model_v5, ham I/Q'yu değil notebook'taki (128,7) özellik setini bekliyor:
    # I, Q, Amplitude, sin(Phase), cos(Phase), Inst_Freq, Amp_Diff (bkz. model_egitimi.ipynb).
    amplitude = np.sqrt(I ** 2 + Q ** 2)
    phase = np.arctan2(Q, I)
    phase_sin = np.sin(phase)
    phase_cos = np.cos(phase)

    phase_unwrapped = np.unwrap(phase)
    inst_freq = np.diff(phase_unwrapped, prepend=phase_unwrapped[:1])
    amp_diff = np.diff(amplitude, prepend=amplitude[:1])

    return np.stack([I, Q, amplitude, phase_sin, phase_cos, inst_freq, amp_diff], axis=1)  # (128, 7)


class _TFLiteModel:
    """tensorflow paketi yoksa tflite_runtime ile aynı model.predict(x, verbose=0)
    arayüzünü sağlar -- classify_iq bu farkı hiç bilmeden çalışır. tools/dataset/
    convert_to_tflite.py ile üretilen .tflite dosyasını bekler (bkz. o dosya)."""

    def __init__(self, tflite_path):
        if not os.path.exists(tflite_path):
            raise FileNotFoundError(
                f"TFLite model bulunamadı: {tflite_path} -- önce Windows PC'de "
                "tools/dataset/convert_to_tflite.py çalıştırıp .tflite dosyasını buraya taşı."
            )
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            from tensorflow.lite.python.interpreter import Interpreter
        self._interpreter = Interpreter(model_path=tflite_path)
        self._interpreter.allocate_tensors()
        self._input_index = self._interpreter.get_input_details()[0]["index"]
        self._output_index = self._interpreter.get_output_details()[0]["index"]

    def predict(self, x, verbose=0):
        self._interpreter.set_tensor(self._input_index, x.astype(np.float32))
        self._interpreter.invoke()
        return self._interpreter.get_tensor(self._output_index)


def load_model_and_scalers():
    """Modeli ve eğitimde kullanılan normalizasyon istatistiklerini yükler.
    streamer.py (gerçek SDR ile) ve bu dosyanın kendi sahte-IQ modu tarafından
    ortak kullanılır -- tek model her ikisinde de aynı şekilde çıkarım yapar."""
    # Gerçek PlutoSDR/RTL-SDR verisiyle fine-tune edilmiş model (bkz.
    # tools/dataset/finetune.py) -- gerçek veri doğruluğunu %7.1 -> %35.7'ye
    # çıkardı. Orijinal sentetik-veri modeli (teknofest_model_v5_best.keras)
    # hâlâ diskte duruyor, gerekirse buradaki dosya adını ona geri çevirebilirsin.
    feature_mean = np.load(os.path.join(_REPO_ROOT, "scalers", "v5_feature_mean.npy"))
    feature_std = np.load(os.path.join(_REPO_ROOT, "scalers", "v5_feature_std.npy"))

    if _HAS_TF:
        model_path = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_finetuned.keras")
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model bulunamadı: {model_path}")
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
        model = tf.keras.models.load_model(model_path)
    else:
        tflite_path = os.path.join(_REPO_ROOT, "models", "teknofest_model_v5_finetuned.tflite")
        model = _TFLiteModel(tflite_path)

    return model, feature_mean, feature_std


def classify_iq(model, feature_mean, feature_std, I, Q):
    """128 örneklik I/Q penceresini sınıflandırır -> (analogSayisal, modulasyonTuru, güven%)."""
    features = iq_to_v5_features(I, Q)  # (128, 7)
    processed_data = np.expand_dims(features, axis=0).astype(np.float32)  # (1, 128, 7)
    processed_data = (processed_data - feature_mean) / feature_std

    prediction = model.predict(processed_data, verbose=0)
    idx = np.argmax(prediction)
    confidence = float(np.max(prediction)) * 100
    detected_mod = CLASSES[idx]
    analog_sayisal = "Analog" if detected_mod in ANALOG_CLASSES else "Sayısal"
    return analog_sayisal, detected_mod, confidence


# Bu eşiğin altındaki tahminler "Belirsiz" olarak bildirilir. Neden gerekli:
# model eğitim dağılımının çok dışındaki (ör. -20dB SNR'den daha gürültülü)
# girdilerde YÜKSEK güvenle YANLIŞ tahmin edebiliyor (sinir ağlarında bilinen
# bir davranış -- "bilmiyorum" diyemiyor, 11 sınıftan birini seçmek zorunda).
# Bu eşik, öyle bir durumda arayüze yanıltıcı kesin bir sınıf adı basmak
# yerine "Belirsiz" göstererek operatörü uyarır.
CONFIDENCE_THRESHOLD = 25.0  # yüzde -- 2026-09-18: 128 orneklik pencereyle
# sahada model guveni hep %20-40 araliginda kaliyordu (11 sinifli bir
# problemde rastgele sansin -- ~%9 -- uzerinde ama 50'ye hic ulasamiyordu),
# sonuc HEP "Belirsiz" cikiyordu. Modulasyon turu zorunlu olmadigi icin (bkz.
# proje notlari) daha dusuk bir esikle -- yanlis olsa bile -- bir tahmin
# vermek, hic cevap vermemekten daha degerli kabul edildi.

# Modelden TAMAMEN bağımsız, klasik bir çapraz kontrol: sayısal sinyaller
# (PSK/QAM/FSK) simge hızında belirgin bir periyodiklik gösterir (her simge
# geçişinde anlık frekansta düzenli bir sıçrama), analog (AM/FM) sinyallerde
# bu yok. Gerçek Pluto-loopback verisiyle, yüksek SNR'de (TX kazancı -20dB)
# test edildi: analog sınıflar 0.16-0.38, sayısal sınıflar 0.07-0.11 aralığında
# çıktı -- 0.14 civarı bir eşik 11 sınıfın hepsini doğru ayırdı.
# ÖNEMLİ SINIRLAMA: düşük SNR'de (ilk denemede -40dB TX kazancıyla) bu ayrım
# TAMAMEN BOZULDU -- yani bu yöntem TEK BAŞINA güvenilir bir birincil karar
# mekanizması DEĞİL. Bu yüzden birincil karar hâlâ modelde; bu fonksiyon
# sadece modelin kararıyla ÇELİŞTİĞİNDE "Belirsiz" tetiklemek için kullanılır
# (aşağıdaki classify_iq_gated'deki classical_I/classical_Q parametreleri).
CLASSICAL_PERIODICITY_THRESHOLD = 0.14


def classical_analog_sayisal(I, Q):
    """Ham I/Q'dan (mümkünse geniş bir pencere -- 128 örnek periyodiklik
    ölçümü için kısa kalabilir, bkz. streamer.py'deki CLASSICAL_CHECK_WINDOW)
    anlık frekansın otokorelasyonuna bakarak periyodiklik skoru çıkarır ve
    kaba bir Analog/Sayısal tahmini döndürür. Modelden bağımsız -- hiçbir
    öğrenilmiş parametre kullanmaz."""
    phase = np.unwrap(np.arctan2(Q, I))
    inst_freq = np.diff(phase, prepend=phase[:1])
    f = inst_freq - np.mean(inst_freq)
    ac = np.correlate(f, f, mode="full")
    ac = ac[len(ac) // 2:]
    ac = ac / (ac[0] + 1e-12)
    search = ac[2:len(ac) // 2]
    periodicity = float(np.max(search)) if len(search) else 0.0
    tur = "Analog" if periodicity >= CLASSICAL_PERIODICITY_THRESHOLD else "Sayısal"
    return tur, periodicity


def classify_iq_gated(model, feature_mean, feature_std, I, Q, classical_I=None, classical_Q=None):
    """classify_iq'yu çağırır, güven CONFIDENCE_THRESHOLD altındaysa sonucu
    ("Belirsiz", "Belirsiz", güven) ile değiştirir. Gerçek zamanlı akışta
    (streamer.py) ve sahte-IQ demo modunda (aşağıdaki start_ai_node) kullanılan
    ORTAK giriş noktası budur -- ham classify_iq, model çıktısını doğrudan
    değerlendirmek isteyen araçlar (ör. tools/ altındaki test scriptleri) için
    hâlâ ayrı kullanılabilir.

    classical_I/classical_Q verilirse (streamer.py daha geniş bir pencere
    yakalayıp gönderiyor -- periyodiklik ölçümü 128 örneğe göre çok daha
    güvenilir), modelin Analog/Sayısal kararıyla klasik yöntemin kararı
    ÇELİŞİRSE sonuç yine "Belirsiz"e döner -- iki bağımsız yöntem aynı anda
    aynı yanlışı yapma ihtimali, tek başına modele güvenmekten düşüktür."""
    analog_sayisal, detected_mod, confidence = classify_iq(model, feature_mean, feature_std, I, Q)
    if confidence < CONFIDENCE_THRESHOLD:
        return "Belirsiz", "Belirsiz", confidence

    if classical_I is not None and classical_Q is not None:
        classical_tur, _ = classical_analog_sayisal(classical_I, classical_Q)
        if classical_tur != analog_sayisal:
            return "Belirsiz", "Belirsiz", confidence

    return analog_sayisal, detected_mod, confidence


def start_ai_node():
    """Bağımsız demo/test modu: GERÇEK SDR YOK, sahte I/Q üretir. Gerçek
    donanımla çalışan asıl akış artık streamer.py'de (RTL-SDR sahibi tek
    süreç) -- bu fonksiyon donanımsız hızlı gösterim/deneme için hâlâ duruyor."""
    print("[*] Model yükleniyor, lütfen bekleyin...")
    model, feature_mean, feature_std = load_model_and_scalers()
    print("[+] Model ve Squelch (Enerji Eşiği) filtresi aktif! (SAHTE I/Q MODU)")

    # --- ZEROMQ AĞI ---
    context = zmq.Context()
    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect("tcp://127.0.0.1:5557")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    pub_result = context.socket(zmq.PUB)
    pub_result.bind("tcp://127.0.0.1:5556")

    while True:
        msg = sub_cmd.recv_string()

        if msg != "SDR_VERISI_ISTEK":
            continue

        # --- 1. SDR SİMÜLASYONU (Boş Kanal vs Dolu Kanal) ---
        is_signal_present = random.choice([True, False])

        if is_signal_present:
            # İçinde gerçek bir sinyal olan güçlü veri simülasyonu
            raw_iq = np.random.randn(2, 128) * 5.0
        else:
            # Sadece zayıf termal gürültü (Boş hava) simülasyonu
            raw_iq = np.random.randn(2, 128) * 0.1

        # --- 2. SQUELCH (ENERJİ EŞİĞİ) HESAPLAMASI ---
        signal_power = np.mean(np.square(raw_iq))
        squelch_threshold = 1.0  # Bu eşiği sahada ortama göre ayarlayacaksın

        if signal_power < squelch_threshold:
            # Sinyal çok zayıf, arayüzdeki AI alanlarını "-" ile temizle.
            pub_result.send_string(f"AI,{TARGET_ID},-,-")
            print(f"[!] Squelch Devrede: Sinyal gücü ({signal_power:.2f}) yetersiz. AI pas geçildi.")
            continue

        # --- 3. YAPAY ZEKA TAHMİNİ (Sadece güçlü sinyaller buraya ulaşır) ---
        print(f"[*] Güçlü sinyal yakalandı (Güç: {signal_power:.2f}). AI analizi yapılıyor...")
        analog_sayisal, detected_mod, confidence = classify_iq_gated(model, feature_mean, feature_std, raw_iq[0], raw_iq[1])

        # AI,id,analogSayisal,modulasyonTuru
        pub_result.send_string(f"AI,{TARGET_ID},{analog_sayisal},{detected_mod}")
        print(f"[>] Tespit: {detected_mod} ({analog_sayisal}) - Güven: %{confidence:.2f}")

if __name__ == "__main__":
    start_ai_node()
