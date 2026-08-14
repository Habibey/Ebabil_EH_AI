import zmq
import numpy as np
import tensorflow as tf
import os
import random

# Modülasyon Sınıfları
CLASSES = ['8PSK', 'AM-DSB', 'AM-SSB', 'BPSK', 'CPFSK', 'GFSK', 'PAM4', 'QAM16', 'QAM64', 'QPSK', 'WBFM']

def start_ai_node():
    # --- MODELİ YÜKLE ---
    model_path = "models/teknofest_model_v5_best.keras"
    if not os.path.exists(model_path):
        print(f"[-] HATA: Model bulunamadı!")
        return

    print("[*] Model yükleniyor, lütfen bekleyin...")
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
    model = tf.keras.models.load_model(model_path)
    print("[+] Model ve Squelch (Enerji Eşiği) filtresi aktif!")

    # --- ZEROMQ AĞI ---
    context = zmq.Context()
    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect("tcp://127.0.0.1:5557")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")
    
    pub_result = context.socket(zmq.PUB)
    pub_result.bind("tcp://127.0.0.1:5556")

    while True:
        msg = sub_cmd.recv_string()
        
        if msg == "SDR_VERISI_ISTEK":
            # --- 1. SDR SİMÜLASYONU (Boş Kanal vs Dolu Kanal) ---
            is_signal_present = random.choice([True, False])
            
            if is_signal_present:
                # İçinde gerçek bir sinyal olan güçlü veri simülasyonu
                raw_iq = np.random.randn(2, 128) * 5.0 
            else:
                # Sadece zayıf termal gürültü (Boş hava) simülasyonu
                raw_iq = np.random.randn(2, 128) * 0.1

            # --- 2. SQUELCH (ENERJİ EŞİĞİ) HESAPLAMASI ---
            # Sinyalin ortalama gücünü hesaplıyoruz
            signal_power = np.mean(np.square(raw_iq))
            squelch_threshold = 1.0 # Bu eşiği sahada ortama göre ayarlayacaksın
            
            if signal_power < squelch_threshold:
                # Sinyal çok zayıf, yapay zekayı hiç uyandırma!
                result_str = ">> HEDEF TESPİTİ: [KANAL BOŞ - GÜRÜLTÜ]"
                pub_result.send_string(result_str)
                print(f"[!] Squelch Devrede: Sinyal gücü ({signal_power:.2f}) yetersiz. AI pas geçildi.")
                continue # Döngünün başına dön, alttaki AI kodlarını çalıştırma
            
            # --- 3. YAPAY ZEKA TAHMİNİ (Sadece güçlü sinyaller buraya ulaşır) ---
            print(f"[*] Güçlü sinyal yakalandı (Güç: {signal_power:.2f}). AI analizi yapılıyor...")
            processed_data = np.expand_dims(raw_iq, axis=0)
            processed_data = np.expand_dims(processed_data, axis=1)

            prediction = model.predict(processed_data, verbose=0)
            idx = np.argmax(prediction)
            confidence = np.max(prediction) * 100
            detected_mod = CLASSES[idx]

            result_str = f">> HEDEF TESPİTİ: {detected_mod} | GÜVEN: %{confidence:.2f}"
            pub_result.send_string(result_str)
            print(f"[>] Tespit: {detected_mod} (%{confidence:.2f})")

if __name__ == "__main__":
    start_ai_node()