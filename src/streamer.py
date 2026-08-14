import zmq
import time
import numpy as np

def start_radar_and_jammer():
    context = zmq.Context()
    
    # 1. Radar Verisini Arayüze Gönderme Hattı (PUB - 5555)
    pub_radar = context.socket(zmq.PUB)
    pub_radar.bind("tcp://127.0.0.1:5555")

    # 2. Arayüzden Karıştırma Emirlerini Dinleme Hattı (SUB - 5557)
    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect("tcp://127.0.0.1:5557")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    is_jamming = False
    print("\n--- EBABİL RADAR VE JAMMER SİMÜLATÖRÜ AKTİF ---")
    print("[*] Port 5555: Veri basılıyor...")
    print("[*] Port 5557: Komuta merkezinden taarruz emri bekleniyor...\n")

    is_jamming = False
    target_freq = 2450.0

    while True:
        # --- KOMUT KONTROLÜ (Nokta Karıştırma Emirleri) ---
        try:
            msg = sub_cmd.recv_string(flags=zmq.NOBLOCK)
            if msg.startswith("JAM_START"):
                is_jamming = True
                # Gelen mesajı parçala (Örn: "JAM_START|2450 MHz")
                target_str = msg.split("|")[1].replace(" MHz", "")
                target_freq = float(target_str)
                print(f"\n[!!!] DİKKAT: {target_freq} MHz'e Nokta Karıştırma (Spot Jamming) Başladı!")
            elif msg == "JAM_STOP":
                is_jamming = False
                print("\n[*] TAARRUZ DURDURULDU: Normal dinlemeye dönüldü.")
        except zmq.Again:
            pass 

        # --- SİNYAL ÜRETİMİ ---
        data = np.random.uniform(-100, -80, 101) # Arka plan gürültüsü
        
        if not is_jamming:
            # NORMAL DİNLEME: Hedef 2450 MHz'de yayın yapıyor
            data[50] = -30 + np.random.uniform(-5, 5) 
            data[49] = -50
            data[51] = -50
        else:
            # SPOT JAMMING (NOKTA KARIŞTIRMA): Sadece hedef frekansı (-10 dBm ile) boğ!
            # Frekansı indekse çeviriyoruz (Örn: 2450 -> İndeks 50)
            idx = int(target_freq - 2400)
            if 0 <= idx <= 100:
                data[idx] = np.random.uniform(-10, 0) # Devasa tepe noktası
                if idx > 0: data[idx-1] = np.random.uniform(-20, -10) # Yan bantlar
                if idx < 100: data[idx+1] = np.random.uniform(-20, -10)

        # Veriyi C++'a yolla
        data_str = ",".join(f"{v:.2f}" for v in data)
        pub_radar.send_string(data_str)
        time.sleep(0.05)
if __name__ == "__main__":
    start_radar_and_jammer()