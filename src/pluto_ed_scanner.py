"""
İKİNCİ PlutoSDR'ı RX olarak kullanıp 868-870 MHz ve 2.4-2.483 GHz bantlarını
hoplayarak tarayan ED (Elektronik Destek) süreci -- streamer.py'nin RTL-SDR
(144/433 MHz) tarafına ek, KTR'de tanımlı ama şu ana kadar kodda hiç var
olmayan üst-bant tarama zincirini gerçekleştirir.

Neden AYRI süreç ve AYRI Pluto: et_control.py'deki Pluto TX (ET/karıştırma)
ile bu Pluto RX (ED/tarama) AYNI ANDA çalışmalı -- aynı Pluto'da hem TX hem RX
yapmak (daha önce konuştuğumuz self-desense riski) burada söz konusu değil
çünkü fiziksel olarak FARKLI cihazlar. streamer.py ile aynı ARAMA/İZLEME
mimarisini kullanır (bkz. streamer.py docstring'i), sadece:
  - Donanım: RtlSdr yerine PlutoSDR RX
  - Bant listesi: tek sürekli aralık yerine BİRDEN FAZLA ayrı bant (aradaki
    boşluklar taranmaz -- 868-870 bitince direkt 2400-2483'e atlar)
  - Portlar: streamer.py'nin 5555/5556'sıyla ÇAKIŞMASIN diye 5560 (SYS/SPEC)

ÖNEMLİ -- İKİNCİ PLUTO IP ÇAKIŞMASI: Fabrika ayarıyla HER Pluto 192.168.2.1
kullanır. et_control.py'nin TX Pluto'su o adreste kalmalı; bu scriptin
kullandığı RX Pluto'nun IP'sini (Pluto'yu USB diski olarak açıp config.txt
içindeki local_ip satırını değiştirerek) FARKLI bir adrese (örn. 192.168.3.1)
almanız gerekiyor -- yoksa ikisi de aynı adrese denk gelir, ikisine de düzgün
erişilemez. Aşağıdaki EBABIL_PLUTO_ED_IP ile hangi adresi kullanacağını
belirtebilirsin.

Sınıflandırma (AI) YOK -- bu scriptin işi sadece tespit (OS-CFAR ile "burada
bir şey var mı"). Modülasyon sınıflandırması istenirse streamer.py'deki gibi
ayrıca eklenir; şimdilik kapsam dışı (KTR'deki YOLO/FHSS-örüntü tanıma da
büyük ayrı bir iş, bu script onu içermiyor).

Kullanım:
  python src/pluto_ed_scanner.py
  EBABIL_PLUTO_ED_DRY_RUN=1 python src/pluto_ed_scanner.py   # ikinci Pluto yokken mantığı test et
"""
import os
import queue
import sys
import threading
import time

import numpy as np
import zmq
from scipy.ndimage import gaussian_filter1d

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..")
sys.path.insert(0, _THIS_DIR)
os.environ["PATH"] = os.path.join(_REPO_ROOT, "tools", "libiio") + os.pathsep + os.environ.get("PATH", "")

import sdr_common

DRY_RUN = os.environ.get("EBABIL_PLUTO_ED_DRY_RUN", "0") == "1"
PLUTO_ED_IP = os.environ.get("EBABIL_PLUTO_ED_IP", "ip:192.168.3.1")

# --- Taranacak bantlar -- KTR'nin güncellenmiş anten/bant tablosuyla birebir ---
BANDS = [
    {"name": "868-870", "start_mhz": 868.0, "stop_mhz": 870.0},
    {"name": "2400-2483", "start_mhz": 2400.0, "stop_mhz": 2483.0},
]

SEARCH_SAMPLE_RATE = 2_000_000  # arama modu -- Pluto'nun genis RX bandini kullanip az adimda tara
SEARCH_STEP_MHZ = SEARCH_SAMPLE_RATE / 1e6

DWELL_SAMPLE_RATE = 4_000_000  # izleme modu -- daha genis, stabil waterfall
DWELL_DURATION_S = 4.0
DWELL_SNAP_MHZ = 0.1

THROWAWAY_SAMPLES = 1024
SYS_SPEC_PORT = 5560  # streamer.py'nin 5555/5556'siyla CAKISMASIN diye ayri
CMD_PORT = 5557  # komut kanali streamer.py ile PAYLASILIYOR (ayni PUB/SUB fan-out)


def build_multi_band_scan_freqs():
    freqs = []
    for band in BANDS:
        freqs.extend(sdr_common.build_scan_freqs(band["start_mhz"], band["stop_mhz"], SEARCH_STEP_MHZ))
    return freqs


class PlutoRX:
    def __init__(self):
        self.pluto = None

    def connect(self):
        if DRY_RUN:
            print(f"[*] EBABIL_PLUTO_ED_DRY_RUN=1 -- {PLUTO_ED_IP} donanımına bağlanılmıyor, sahte veriyle test.")
            return
        import adi
        print(f"[*] İkinci PlutoSDR'a (RX) bağlanılıyor ({PLUTO_ED_IP})...")
        self.pluto = adi.Pluto(PLUTO_ED_IP)
        self.pluto.rx_enabled_channels = [0]
        self.pluto.gain_control_mode_chan0 = "slow_attack"  # otomatik kazanç -- RTL-SDR'daki gain='auto' eşdeğeri
        print("[+] Pluto RX hazır.")

    def capture(self, center_mhz, sample_rate):
        if DRY_RUN:
            # Donanım yok -- gürültü tabanı + ~%8 ihtimalle sahte bir tespit
            # üret, ana döngünün/CFAR'ın gerçek koşullarda test edilebilmesi
            # için. Tek bir saf ton DEĞİL (o kadar dar ki 64 bin'e biriktirmede
            # kayboluyor) -- bant-sınırlı bir gürültü patlaması (yaklaşık 80 ham
            # FFT bin genişliğinde, et_control.py'deki generate_barrage_noise
            # ile aynı teknik) kullanılıyor ki gerçek bir sinyal gibi 64-bin
            # ortalamasından sonra da görünür kalsın.
            n = sdr_common.FFT_SIZE
            samples = (np.random.randn(n) + 1j * np.random.randn(n)).astype(np.complex64) * 0.01
            if np.random.rand() < 0.08:
                i = gaussian_filter1d(np.random.randn(n), sigma=n / (2 * np.pi * 40), mode="wrap")
                q = gaussian_filter1d(np.random.randn(n), sigma=n / (2 * np.pi * 40), mode="wrap")
                burst = (i + 1j * q)
                burst = burst / (np.max(np.abs(burst)) + 1e-9) * 0.4
                samples = samples + burst.astype(np.complex64)
            return sdr_common.compute_power_spectrum(samples, center_mhz, sample_rate)

        self.pluto.rx_lo = int(center_mhz * 1e6)
        self.pluto.sample_rate = int(sample_rate)
        self.pluto.rx_rf_bandwidth = int(sample_rate)
        self.pluto.rx_buffer_size = sdr_common.FFT_SIZE + THROWAWAY_SAMPLES
        self.pluto.rx_destroy_buffer()
        samples = self.pluto.rx()
        if isinstance(samples, (list, tuple)):
            samples = samples[0]
        samples = np.asarray(samples)[THROWAWAY_SAMPLES:]
        return sdr_common.compute_power_spectrum(samples, center_mhz, sample_rate)


def stdin_command_reader(command_queue):
    for line in sys.stdin:
        line = line.strip()
        if line:
            command_queue.put(line)


def main():
    context = zmq.Context()

    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://127.0.0.1:{SYS_SPEC_PORT}")

    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect(f"tcp://127.0.0.1:{CMD_PORT}")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    rx = PlutoRX()
    rx.connect()

    tracker = sdr_common.TargetTracker(id_prefix="PHEDEF")  # streamer.py'nin HEDEF-N'iyle karışmasın
    scan_freqs = build_multi_band_scan_freqs()
    scan_idx = 0

    dwelling = False
    dwell_center_mhz = None
    dwell_started_at = 0.0
    dwell_locked = False
    selected_target_id = None

    command_queue = queue.Queue()
    threading.Thread(target=stdin_command_reader, args=(command_queue,), daemon=True).start()

    band_names = ", ".join(f"{b['name']} MHz" for b in BANDS)
    print(f"[*] Taranacak bantlar: {band_names} ({len(scan_freqs)} adım toplam)")
    print(f"[*] Port {SYS_SPEC_PORT}: SYS/SPEC | Port {CMD_PORT}: komut dinleniyor (paylaşımlı)")
    print("[*] Belirli bir hedefe kilitlenmek için: hedef <id>  |  hedef oto\n")

    try:
        while True:
            try:
                try:
                    msg = sub_cmd.recv_string(flags=zmq.NOBLOCK)
                    if msg.startswith("PLUTO_ED_HEDEF_SEC|"):
                        _, target_id = msg.split("|")
                        command_queue.put(f"hedef {target_id}")
                except zmq.Again:
                    pass

                try:
                    line = command_queue.get_nowait()
                    lower = line.lower()
                    if lower.startswith("hedef"):
                        parts = line.split()
                        if len(parts) != 2:
                            print("[!] Kullanım: hedef <id>  |  hedef oto")
                        elif parts[1].upper() in ("OTO", "OTOMATIK"):
                            selected_target_id = None
                            dwelling = False
                            dwell_locked = False
                            scan_idx = 0
                            print("[*] Hedef seçimi temizlendi.")
                        else:
                            target_id = parts[1].upper()
                            if target_id not in tracker.known:
                                print(f"[!] {target_id} bilinmiyor (bilinenler: {', '.join(tracker.known) or '(yok)'})")
                            else:
                                selected_target_id = target_id
                                raw_freq = tracker.known[target_id]["freq_mhz"]
                                dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                                dwelling = True
                                dwell_locked = True
                                dwell_started_at = time.time()
                                print(f"[*] {target_id} seçildi, {dwell_center_mhz:.3f} MHz'e kilitlendi.")
                except queue.Empty:
                    pass

                if dwelling and not dwell_locked and (time.time() - dwell_started_at > DWELL_DURATION_S):
                    dwelling = False

                if dwelling:
                    binned_db, bin_freqs_mhz, fs_mhz = rx.capture(dwell_center_mhz, DWELL_SAMPLE_RATE)
                else:
                    center_mhz = scan_freqs[scan_idx]
                    scan_idx += 1
                    binned_db, bin_freqs_mhz, fs_mhz = rx.capture(center_mhz, SEARCH_SAMPLE_RATE)

                spec_center = dwell_center_mhz if dwelling else center_mhz
                spec_fields = ["SPEC", f"{spec_center:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                pub.send_string(",".join(spec_fields))

                peak = sdr_common.detect_peak(binned_db, bin_freqs_mhz)
                if peak is not None:
                    freq_mhz, power_db, bandwidth_khz, noise_floor_db = peak
                    snr_db = power_db - noise_floor_db
                    sapma_mhz = freq_mhz - spec_center
                    tid = tracker.update(freq_mhz, power_db, bandwidth_khz, snr_db, sapma_mhz)
                    info = tracker.known[tid]
                    sureklilik = tracker.sureklilik_durumu(tid)
                    sys_fields = [
                        "SYS", tid, "1", "nan", "nan", "0",
                        f"{info['freq_mhz']:.3f}", f"{info['power_db']:.2f}", f"{info['bandwidth_khz']:.1f}",
                        f"{sapma_mhz:.4f}", f"{power_db - snr_db:.2f}", f"{snr_db:.2f}", sureklilik,
                    ]
                    pub.send_string(",".join(sys_fields))

                if not dwelling and scan_idx >= len(scan_freqs):
                    scan_idx = 0
                    lock_target = None
                    if selected_target_id is not None and selected_target_id in tracker.known:
                        lock_target = selected_target_id
                    else:
                        lock_target = tracker.most_recent()
                    if lock_target is not None:
                        raw_freq = tracker.known[lock_target]["freq_mhz"]
                        dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                        dwelling = True
                        dwell_started_at = time.time()

                if DRY_RUN:
                    time.sleep(0.05)  # sahte modda CPU'yu bogmasin

            except Exception as e:
                print(f"[!] Tarama sırasında hata (devam ediliyor): {e}")
                time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        if not DRY_RUN and rx.pluto is not None:
            rx.pluto.rx_destroy_buffer()
        print("\n[*] pluto_ed_scanner kapatıldı.")


if __name__ == "__main__":
    main()
