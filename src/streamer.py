"""
Gerçek RTL-SDR ile bant taraması + hedef tespiti + (istendiğinde) AI
sınıflandırması yapan tek süreç.

Neden tek süreç: RTL-SDR tek bir USB cihazı, aynı anda sadece TEK bir process
tarafından açılabilir. Bu yüzden artık streamer.py hem SYS/SPEC (tarama +
tespit, port 5555) hem de AI (sınıflandırma, port 5556) paketlerini kendisi
üretiyor -- predict.py'nin bağımsız süreci (start_ai_node) hâlâ duruyor ama
o SADECE donanımsız/sahte-IQ demo modu için (bkz. predict.py). Gerçek
donanımla artık SADECE bu dosyayı çalıştırman yeterli, predict.py'yi ayrıca
başlatmana gerek yok.

Model/özellik çıkarma kodu predict.py'den import ediliyor (tek kaynak, aynı
kod hem gerçek hem sahte modda kullanılıyor).

--- ARAMA / İZLEME (dwell) MODLARI ---
İlk tasarım tek bir ince (250kHz) taramayla tüm bandı sürekli dolaşıyordu --
ama arayüzün waterfall'ı, ardışık SPEC paketlerinin merkez frekansı
%15'ten fazla kayarsa görünümü SIFIRLAYIP yeniden merkezliyor (bkz.
mainwindow.cpp odaklanGerekirse). 250kHz'lik adımlarla sürekli sıçramak bu
sıfırlamayı HER paketten tetikliyor, yani waterfall zaman içinde biriken
anlamlı bir görüntü değil, sadece o anki tek adımın anlık görüntüsü oluyordu.

Çözüm -- iki mod:
  ARAMA: tüm bandı (SCAN_START/STOP) ince adımlarla tarar, SYS/SPEC basar.
  İZLEME (dwell): bir tam arama turu bir hedef bulduysa, o hedefin
    frekansına kilitlenip DAHA GENİŞ örnekleme hızıyla (DWELL_SAMPLE_RATE)
    AYNI merkez/genişliği tekrar tekrar gönderir -- görünüm sabit kalır,
    gerçek bir spektrum analizördeki gibi zaman içinde birikip akar.
    DWELL_DURATION_S sonra kısa bir arama turuna dönüp yeni hedef arar.
"""
import os
import queue
import sys
import threading
import time

import zmq
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.join(_THIS_DIR, "..")
sys.path.insert(0, _THIS_DIR)
os.add_dll_directory(os.path.join(_REPO_ROOT, "tools", "rtlsdr"))

from rtlsdr import RtlSdr

from predict import load_model_and_scalers, classify_iq_gated, CLASSES
import sdr_common

# --- Tarama ayarları (ortam değişkeniyle değiştirilebilir) ---
SEARCH_SAMPLE_RATE = 250000  # arama modu -- ince çözünürlük (RTL-SDR'ın düşük geçerli aralığı: 225k-300k)
SCAN_START_MHZ = float(os.environ.get("EBABIL_SCAN_START_MHZ", 430.0))
SCAN_STOP_MHZ = float(os.environ.get("EBABIL_SCAN_STOP_MHZ", 440.0))
SEARCH_STEP_MHZ = SEARCH_SAMPLE_RATE / 1e6  # örtüşmesiz, kanal genişliği kadar adım

DWELL_SAMPLE_RATE = 1024000  # izleme modu -- geniş anlık bant, kararlı/akan waterfall için
DWELL_SPAN_MHZ = DWELL_SAMPLE_RATE / 1e6
DWELL_DURATION_S = 4.0  # bu süre boyunca hedefe kilitlenip kal, sonra kısa bir arama turu yap
DWELL_SNAP_MHZ = 0.05  # merkezi bu hassasiyete yuvarla -- küçük titreşim görünümü resetlemesin

FFT_SIZE = sdr_common.FFT_SIZE
FREQ_BINS = sdr_common.FREQ_BINS
THROWAWAY_SAMPLES = 1024  # her retune/hız değişimi sonrası kararsız örnekleri at

TARGET_MATCH_TOLERANCE_MHZ = 0.15  # bu aralıktaki tekrar tespitler AYNI hedef kabul edilir
# NOT: Geniş sinyaller (ör. ~200kHz FM yayını) komşu tarama adımları arasında
# bölününce tepe frekansı tahmini birkaç 10 kHz kayabiliyor -- tolerans bunu
# tolere edecek kadar geniş olmalı (0.05 denendi, aynı istasyona ikinci bir
# HEDEF-N açıyordu). Çok dar bantlı/birbirine yakın gerçek hedeflerle
# çalışırken bu değeri düşürmek gerekebilir.

CLASSIFY_WINDOW = 128  # modelin beklediği pencere uzunluğu -- SEARCH_SAMPLE_RATE'te toplanmalı (fine-tuning verisiyle tutarlı)
# Klasik periyodiklik çapraz kontrolü (predict.classical_analog_sayisal) 128
# örnekte güvenilir değil -- otokorelasyonun anlamlı olması için çok daha
# geniş bir pencere gerekiyor (11 sınıfla doğrulanan test 5000 örnek kullandı).
CLASSICAL_CHECK_WINDOW = 5000


def capture_power_spectrum(sdr, center_mhz, sample_rate):
    """center_mhz'e kilitlenip FFT_SIZE örnek alır, FREQ_BINS'e indirgenmiş
    güç spektrumunu (dB), her bin'in gerçek frekansını (MHz) ve kullanılan
    örnekleme hızını (MHz) döndürür. sdr.sample_rate zaten bu değere
    ayarlanmış olmalı (main() içindeki mod geçişleri bunu yönetiyor).
    FFT/binleme matematiği sdr_common'da -- pluto_ed_scanner.py ile ortak."""
    sdr.center_freq = center_mhz * 1e6
    sdr.read_samples(THROWAWAY_SAMPLES)
    samples = sdr.read_samples(FFT_SIZE)
    return sdr_common.compute_power_spectrum(samples, center_mhz, sample_rate)


# detect_peak artık sdr_common'da GERÇEK OS-CFAR ile uygulanıyor (eskiden
# medyan + sabit eşikti -- KTR Tablo 7'de OS-CFAR yazıyordu ama kod basit bir
# eşiklemeydi, gerçek adaptif persentil tabanlı tespite geçildi).
detect_peak = sdr_common.detect_peak
TargetTracker = sdr_common.TargetTracker


def stdin_command_reader(command_queue):
    """Ayrı thread'de stdin'den satır satır okur, kuyruğa koyar -- ana döngü
    bunu NOBLOCK şekilde kontrol eder (ZMQ komut kontrolüyle aynı desen)."""
    for line in sys.stdin:
        line = line.strip()
        if line:
            command_queue.put(line)


def handle_save_command(sdr, tracker, args, selected_id=None):
    """'kaydet ETIKET [süre_sn]' komutunu işler -- şu an seçili/en son
    görülen hedefin frekansına kilitlenip gerçek I/Q kaydeder,
    data/real_captures/<ETIKET>/<zaman damgası>.npz olarak kaydeder.
    Format, build_training_set.py'nin beklediğiyle birebir aynı (bkz.
    tools/dataset/capture_and_label.py) -- ayrıca bir dönüştürme gerekmez.
    Kayıt her zaman SEARCH_SAMPLE_RATE'te yapılır (fine-tuning verisiyle
    tutarlı olsun diye), dwell modunda olsak bile."""
    parts = args.split()
    if not parts:
        print("[!] Kullanım: kaydet <ETİKET> [süre_sn]  (örn: kaydet WBFM 10)")
        return

    label = parts[0].upper()
    if label not in CLASSES:
        print(f"[!] Geçersiz etiket: {label}. Geçerli etiketler: {', '.join(CLASSES)}")
        return

    duration = float(parts[1]) if len(parts) > 1 else 10.0

    tid = pick_target(tracker, selected_id)
    if tid is None:
        print("[!] Henüz tespit edilmiş hedef yok, kayıt alınamadı.")
        return

    freq_mhz = tracker.known[tid]["freq_mhz"]
    print(f"[*] Kayıt başlıyor: {tid} ({freq_mhz:.3f} MHz), etiket={label}, süre={duration:.1f}s...")

    sdr.sample_rate = SEARCH_SAMPLE_RATE
    sdr.center_freq = freq_mhz * 1e6
    sdr.read_samples(THROWAWAY_SAMPLES)

    n_samples = int(duration * SEARCH_SAMPLE_RATE)
    chunk = 262144  # tek seferde büyük blok istemek USB zaman aşımına yol açıyor
    parts_list = []
    remaining = n_samples
    while remaining > 0:
        n = min(chunk, remaining)
        parts_list.append(sdr.read_samples(n))
        remaining -= n
    samples = np.concatenate(parts_list).astype(np.complex64)

    out_dir = os.path.join(_REPO_ROOT, "data", "real_captures", label)
    os.makedirs(out_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"{timestamp}.npz")
    np.savez_compressed(
        out_path, samples=samples, label=label, freq_mhz=freq_mhz,
        sample_rate=SEARCH_SAMPLE_RATE, captured_at=timestamp,
    )
    print(f"[+] Kaydedildi: {out_path} ({len(samples)} örnek, {len(samples) // 128} pencere üretilebilir)")


def handle_classify_request(sdr, pub_ai, tracker, model, feature_mean, feature_std, selected_id=None):
    """Sınıflandırma için her zaman SEARCH_SAMPLE_RATE'e döner -- model,
    fine-tuning verisi bu hızda toplandığı için buna göre eğitildi. Dwell
    modundaysak bile geçici olarak hıza döner, sonraki dwell/arama adımı
    kendi hızını main() döngüsünde yeniden ayarlar."""
    tid = pick_target(tracker, selected_id)
    if tid is None:
        print("[!] Henüz tespit edilmiş hedef yok, sınıflandırma isteği atlandı.")
        return

    freq_mhz = tracker.known[tid]["freq_mhz"]
    sdr.sample_rate = SEARCH_SAMPLE_RATE
    sdr.center_freq = freq_mhz * 1e6
    sdr.read_samples(THROWAWAY_SAMPLES)
    # Modelin gördüğü pencere (ilk CLASSIFY_WINDOW örnek) DEĞİŞMİYOR -- eğitimde
    # kullanılanla birebir aynı kalsın diye. Geri kalanı SADECE klasik periyodiklik
    # çapraz kontrolü için (bkz. predict.classify_iq_gated).
    window_full = sdr.read_samples(CLASSICAL_CHECK_WINDOW)
    window = window_full[:CLASSIFY_WINDOW]

    I = np.real(window).astype(np.float32)
    Q = np.imag(window).astype(np.float32)
    classical_I = np.real(window_full).astype(np.float32)
    classical_Q = np.imag(window_full).astype(np.float32)
    analog_sayisal, mod, confidence = classify_iq_gated(
        model, feature_mean, feature_std, I, Q, classical_I, classical_Q)

    pub_ai.send_string(f"AI,{tid},{analog_sayisal},{mod}")
    print(f"[>] {tid} ({freq_mhz:.3f} MHz) sınıflandırıldı: {mod} ({analog_sayisal}) - güven %{confidence:.1f}")


def build_scan_freqs(start_mhz, stop_mhz):
    return sdr_common.build_scan_freqs(start_mhz, stop_mhz, SEARCH_STEP_MHZ)


def pick_target(tracker, selected_id):
    """Operatör 'hedef <id>' komutuyla belirli bir hedef seçtiyse ve o hedef
    hâlâ tracker'da biliniyorsa onu döndürür; aksi halde (seçim yoksa veya
    seçilen hedef artık bilinmiyorsa) en son görülen hedefe düşer -- eski
    varsayılan davranış."""
    if selected_id is not None and selected_id in tracker.known:
        return selected_id
    return tracker.most_recent()


def main():
    context = zmq.Context()

    pub = context.socket(zmq.PUB)
    pub.bind("tcp://127.0.0.1:5555")

    pub_ai = context.socket(zmq.PUB)
    pub_ai.bind("tcp://127.0.0.1:5556")

    sub_cmd = context.socket(zmq.SUB)
    sub_cmd.connect("tcp://127.0.0.1:5557")
    sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

    print("[*] Model yükleniyor...")
    model, feature_mean, feature_std = load_model_and_scalers()
    print("[+] Model hazır.")

    sdr = RtlSdr()
    sdr.sample_rate = SEARCH_SAMPLE_RATE
    sdr.gain = "auto"
    current_rate = SEARCH_SAMPLE_RATE

    tracker = TargetTracker(match_tolerance_mhz=TARGET_MATCH_TOLERANCE_MHZ)

    # Bant aralığı artık çalışırken değiştirilebilir (bkz. aşağıdaki "bant" ve
    # "frekans" komutları) -- şartname madde 5.1.1: hakemler önce hiçbir şey
    # söylemez, hiçbir takım bulamazsa önce BANT sonra FREKANS açıklayabilir.
    # Bu durumda süreci yeniden başlatmadan taramayı daraltabilmemiz gerekiyor.
    scan_start_mhz = SCAN_START_MHZ
    scan_stop_mhz = SCAN_STOP_MHZ
    scan_freqs = build_scan_freqs(scan_start_mhz, scan_stop_mhz)
    scan_idx = 0

    dwelling = False
    dwell_center_mhz = None
    dwell_started_at = 0.0
    dwell_locked = False  # "frekans" komutuyla kilitlendiyse DWELL_DURATION_S sonra arama moduna dönmez

    # Operatör GUI'de belirli bir HEDEF-N kartını seçtiğinde ("hedef <id>"
    # komutu) buraya yazılır -- artık "en son bulunan" değil, operatörün
    # SEÇTİĞİ hedefe kilitleniriz (bkz. pick_target). "hedef oto" veya
    # "hedef <id>" ile tekrar otomatik moda dönülebilir.
    selected_target_id = None

    command_queue = queue.Queue()
    threading.Thread(target=stdin_command_reader, args=(command_queue,), daemon=True).start()

    print(f"[*] {scan_start_mhz}-{scan_stop_mhz} MHz aralığı taranıyor "
          f"({len(scan_freqs)} adım, adım genişliği {SEARCH_STEP_MHZ:.3f} MHz)")
    print("[*] Port 5555: SYS/SPEC | Port 5556: AI | Port 5557: komut dinleniyor")
    print("[*] Gerçek veri kaydetmek için: kaydet <ETİKET> [süre_sn]  (örn: kaydet WBFM 10)")
    print("[*] Hakem bant açıklarsa: bant <başlangıç_mhz> <bitiş_mhz>  (örn: bant 433.0 435.0)")
    print("[*] Hakem tam frekans açıklarsa: frekans <mhz>  (örn: frekans 433.92)\n")

    try:
        while True:
            try:
                # --- ZMQ KOMUTU (arayüzden/GUI'den) ---
                try:
                    msg = sub_cmd.recv_string(flags=zmq.NOBLOCK)
                    if msg == "SDR_VERISI_ISTEK":
                        handle_classify_request(sdr, pub_ai, tracker, model, feature_mean, feature_std, selected_target_id)
                        current_rate = None  # handle_classify_request hızı değiştirdi, döngü yeniden ayarlasın
                    elif msg.startswith("ET,BASLAT,") or msg.startswith("ET,DURDUR,"):
                        # Takım arkadaşımızın et_kontrol (Desktop/ET/) protokolüyle
                        # aynı format: "ET,BASLAT,<görev_kodu>,<frekans_mhz>" /
                        # "ET,DURDUR,<görev_kodu>". Henüz gerçek bir verici/et_kontrol
                        # bu tarafta çalışmıyor, sadece logluyoruz.
                        print(f"[*] ET komutu alındı (henüz vericiye bağlı değil): {msg}")
                    elif msg.startswith("SET_POWER "):
                        print(f"[*] Çıkış gücü ayarı alındı (henüz vericiye bağlı değil): {msg}")
                    elif msg.startswith("BANT_AYARLA|"):
                        # BANT_AYARLA|<başlangıç_mhz>|<bitiş_mhz> -- arayüzden gelebilecek
                        # eşdeğeri, bkz. aşağıdaki "bant" klavye komutu.
                        try:
                            _, start_s, stop_s = msg.split("|")
                            command_queue.put(f"bant {start_s} {stop_s}")
                        except ValueError:
                            print(f"[!] Geçersiz BANT_AYARLA komutu: {msg}")
                    elif msg.startswith("FREKANS_KILITLE|"):
                        try:
                            _, freq_s = msg.split("|")
                            command_queue.put(f"frekans {freq_s}")
                        except ValueError:
                            print(f"[!] Geçersiz FREKANS_KILITLE komutu: {msg}")
                    elif msg.startswith("HEDEF_SEC|"):
                        # HEDEF_SEC|<hedef_id> -- GUI'de bir HEDEF-N kartına
                        # tıklandığında; "HEDEF_SEC|OTOMATIK" seçimi temizler.
                        try:
                            _, target_id = msg.split("|")
                            command_queue.put(f"hedef {target_id}")
                        except ValueError:
                            print(f"[!] Geçersiz HEDEF_SEC komutu: {msg}")
                except zmq.Again:
                    pass

                # --- KLAVYE KOMUTU ---
                try:
                    line = command_queue.get_nowait()
                    lower = line.lower()
                    if lower.startswith("kaydet"):
                        handle_save_command(sdr, tracker, line[len("kaydet"):].strip(), selected_target_id)
                        current_rate = None

                    elif lower.startswith("bant"):
                        # Şartname madde 5.1.1: hiçbir takım sinyali bulamazsa
                        # hakemler önce BANDI açıklayabilir -- taramayı o
                        # aralığa daraltıp süreci yeniden başlatmadan devam
                        # ederiz. Mevcut hedefler/izleme durumu korunur,
                        # sadece arama aralığı ve o anki tur sıfırlanır.
                        parts = line.split()
                        if len(parts) != 3:
                            print("[!] Kullanım: bant <başlangıç_mhz> <bitiş_mhz>  (örn: bant 433.0 435.0)")
                        else:
                            try:
                                new_start, new_stop = float(parts[1]), float(parts[2])
                                if new_start >= new_stop:
                                    print(f"[!] Başlangıç bitişten küçük olmalı: {new_start} >= {new_stop}")
                                else:
                                    scan_start_mhz, scan_stop_mhz = new_start, new_stop
                                    scan_freqs = build_scan_freqs(scan_start_mhz, scan_stop_mhz)
                                    scan_idx = 0
                                    dwelling = False
                                    dwell_locked = False
                                    print(f"[*] Tarama aralığı güncellendi: {scan_start_mhz}-{scan_stop_mhz} MHz "
                                          f"({len(scan_freqs)} adım)")
                            except ValueError:
                                print(f"[!] Geçersiz sayı: {line!r}")

                    elif lower.startswith("frekans"):
                        # Şartname madde 5.1.1: hakemler bandın ardından TAM
                        # frekansı da açıklayabilir -- artık aramaya hiç gerek
                        # yok, doğrudan o frekansa kilitlenip izleme moduna geçiyoruz.
                        parts = line.split()
                        if len(parts) != 2:
                            print("[!] Kullanım: frekans <mhz>  (örn: frekans 433.92)")
                        else:
                            try:
                                exact_freq = float(parts[1])
                                dwell_center_mhz = round(exact_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                                dwelling = True
                                dwell_locked = True
                                dwell_started_at = time.time()
                                print(f"[*] {dwell_center_mhz:.3f} MHz'e kilitlendi, izleme modunda.")
                            except ValueError:
                                print(f"[!] Geçersiz sayı: {line!r}")

                    elif lower.startswith("hedef"):
                        # "hedef <id>": belirtilen HEDEF-N'e kilitlen (en son
                        # bulunana değil) -- kaydet/sınıflandırma/dwell hep bunu
                        # kullanır. "hedef oto": seçimi temizle, eski otomatik
                        # ("en son bulunan hedef") davranışına dön.
                        parts = line.split()
                        if len(parts) != 2:
                            print("[!] Kullanım: hedef <id>  (örn: hedef HEDEF-2)  |  hedef oto")
                        elif parts[1].upper() in ("OTO", "OTOMATIK"):
                            selected_target_id = None
                            dwelling = False
                            dwell_locked = False
                            scan_idx = 0
                            print("[*] Hedef seçimi temizlendi -- otomatik moda (en son bulunan) dönüldü.")
                        else:
                            target_id = parts[1].upper()
                            if target_id not in tracker.known:
                                print(f"[!] {target_id} henüz bilinmiyor -- önce tespit edilmiş olmalı "
                                      f"(bilinen hedefler: {', '.join(tracker.known) or '(yok)'})")
                            else:
                                selected_target_id = target_id
                                raw_freq = tracker.known[target_id]["freq_mhz"]
                                dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                                dwelling = True
                                dwell_locked = True
                                dwell_started_at = time.time()
                                print(f"[*] {target_id} seçildi, {dwell_center_mhz:.3f} MHz'e kilitlendi.")

                    else:
                        print(f"[!] Bilinmeyen komut: {line!r} "
                              f"(kullanım: kaydet <ETİKET> [süre_sn] | bant <başlangıç> <bitiş> | "
                              f"frekans <mhz> | hedef <id>|oto)")
                except queue.Empty:
                    pass

                # --- MOD SEÇİMİ ---
                # dwell_locked: "frekans" komutuyla operatör bilerek kilitlendiyse
                # (hakem tam frekansı açıkladıysa) süre dolunca arama moduna
                # dönmüyoruz -- artık aranacak bir şey yok, o frekansta kalınır.
                if dwelling and not dwell_locked and (time.time() - dwell_started_at > DWELL_DURATION_S):
                    dwelling = False  # süre doldu, kısa bir arama turuna dön

                if dwelling:
                    # --- İZLEME (dwell): hedefe kilitli, geniş bant, sabit merkez ---
                    if current_rate != DWELL_SAMPLE_RATE:
                        sdr.sample_rate = DWELL_SAMPLE_RATE
                        current_rate = DWELL_SAMPLE_RATE

                    binned_db, bin_freqs_mhz, fs_mhz = capture_power_spectrum(sdr, dwell_center_mhz, DWELL_SAMPLE_RATE)

                    spec_fields = ["SPEC", f"{dwell_center_mhz:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                    pub.send_string(",".join(spec_fields))

                    peak = detect_peak(binned_db, bin_freqs_mhz)
                    if peak is not None:
                        freq_mhz, power_db, bandwidth_khz = peak
                        tid = tracker.update(freq_mhz, power_db, bandwidth_khz)
                        sys_fields = ["SYS", tid, "1", "nan", "nan", "0",
                                      f"{freq_mhz:.3f}", f"{power_db:.2f}", f"{bandwidth_khz:.1f}"]
                        pub.send_string(",".join(sys_fields))

                else:
                    # --- ARAMA: tüm bandı ince adımlarla dolaş ---
                    if current_rate != SEARCH_SAMPLE_RATE:
                        sdr.sample_rate = SEARCH_SAMPLE_RATE
                        current_rate = SEARCH_SAMPLE_RATE

                    center_mhz = scan_freqs[scan_idx]
                    scan_idx += 1

                    binned_db, bin_freqs_mhz, fs_mhz = capture_power_spectrum(sdr, center_mhz, SEARCH_SAMPLE_RATE)

                    spec_fields = ["SPEC", f"{center_mhz:.3f}", f"{fs_mhz:.3f}"] + [f"{v:.2f}" for v in binned_db]
                    pub.send_string(",".join(spec_fields))

                    peak = detect_peak(binned_db, bin_freqs_mhz)
                    if peak is not None:
                        freq_mhz, power_db, bandwidth_khz = peak
                        tid = tracker.update(freq_mhz, power_db, bandwidth_khz)
                        sys_fields = ["SYS", tid, "1", "nan", "nan", "0",
                                      f"{freq_mhz:.3f}", f"{power_db:.2f}", f"{bandwidth_khz:.1f}"]
                        pub.send_string(",".join(sys_fields))

                    if scan_idx >= len(scan_freqs):
                        # Bir tam tur bitti -- operatör bir hedef SEÇTİYSE onun
                        # üzerine, seçmediyse en son bulunan hedefin üzerine kilitlen.
                        scan_idx = 0
                        lock_target = pick_target(tracker, selected_target_id)
                        if lock_target is not None:
                            raw_freq = tracker.known[lock_target]["freq_mhz"]
                            dwell_center_mhz = round(raw_freq / DWELL_SNAP_MHZ) * DWELL_SNAP_MHZ
                            dwelling = True
                            dwell_started_at = time.time()

            except Exception as e:
                # RTL-SDR ile uzun süre çalışırken ara sıra geçici USB
                # hataları (ör. LIBUSB_ERROR_PIPE) görülebiliyor. Tek bir
                # hatada tüm süreç ölmesin -- logla, kısa bir bekleme sonrası
                # taramaya devam et. Sorun kalıcıysa (cihaz gerçekten koptu)
                # bu döngü sürekli hata basar, en azından süreç ayakta kalır
                # ve loglardan fark edilir.
                print(f"[!] Tarama/sınıflandırma sırasında hata (devam ediliyor): {e}")
                current_rate = None  # hata sonrası hız durumu şüpheli, bir sonraki adımda zorla yeniden ayarla
                time.sleep(0.5)

    finally:
        sdr.close()


if __name__ == "__main__":
    main()
