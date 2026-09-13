"""
Yer istasyonu (Jetson Nano) icin AI siniflandirma servisi -- yer_istasyonu_koprusu
(C++, yonKonum1905) tarafindan ZMQ REQ/REP ile cagrilir.

NEDEN AYRI BIR SERVIS (C++'a gomulu TensorFlow yerine): CRNN modeli zaten
Python/Keras'ta egitilip test edildi (predict.py -- classify_iq_gated,
load_model_and_scalers, iq_to_v5_features). C++ tarafinda TensorFlow C++ API'sini
kurup ayni ozellik-cikarma/on-isleme mantigini tekrar yazmak hem agir (Jetson'da
bazel ile TF C++ derlemek pratik degil) hem de ayni matematigi ikinci kez (ve
farkli bir hatayla) uretme riski tasir -- bu yuzden konum_servisi'nde
(C++ matematigini Python'a acan servis) kullanilan AYNI desenin TERSİ: burada
Python/AI modeli C++'a aciliyor.

Protokol (ZMQ REP, istemci C++ -- yer_istasyonu_koprusu):
  Istek:  ham bytes, TAM 1024 bayt = 128 ornek * 2 (I,Q) * 4 bayt (float32,
          kucuk-endian, IEEE 754) -- IQGondirici'nin gonderdigi 128 orneklik
          pencereyle BIREBIR ayni boyut/sira (I0,Q0,I1,Q1,...,I127,Q127).
  Cevap:  "OK,<analogSayisal>,<modulasyonTuru>,<guven_yuzde>"
          "HATA,<sebep>"

NOT -- classical_I/classical_Q cross-check YOK: streamer.py'nin yerel modu
(dogrudan RTL-SDR'a bagli) periyodiklik capraz kontrolu icin 5000 orneklik
DAHA GENIS bir pencere de yakalayip gonderebiliyor (bkz. predict.py
classify_iq_gated). Burada elimizde SADECE 128 ornek var (915MHz telemetri
hattinin bant genisligi kisiti - bkz. IQGondirici.h -- daha genis bir pencere
bu hatla tasinamaz), bu yuzden classical_I/Q verilmiyor; classify_iq_gated bu
durumu zaten destekliyor (opsiyonel parametre), sadece capraz kontrol
ATLANIYOR, model kendi guven esigiyle tek basina karar veriyor.

Hedef-id eslemesi HENUZ YOK (bilinen eksik, bkz. yer_istasyonu_koprusu/main.cpp
TODO): DATA96 alt-protokolu (pencere_id, 0-255 donguisel) ile SYS paketindeki
gercek hedef_id (ör. "HEDEF-1") arasinda bir korelasyon TASIMIYOR -- RPi
tarafinda IQGondirici bant/hedef bilgisini pencereye hic gömmüyor. Bu servis
SADECE siniflandirma yapar, hedef_id eslemesi cagiran tarafin (C++ koprusu)
sorumlulugunda ve o taraf da su an icin ham pencere_id'yi kullaniyor (bkz.
main.cpp yorumu) -- GERCEK bir hedef_id eslemesi degil, bunu burada UYDURMUYORUZ.
"""
import os
import struct

import numpy as np
import zmq

from predict import load_model_and_scalers, classify_iq_gated

PENCERE_BOYUTU = 128
BEKLENEN_BAYT = PENCERE_BOYUTU * 2 * 4  # 128 ornek * (I,Q) * float32


def main():
    bind_adres = os.environ.get("EBABIL_AI_SERVISI_BIND", "tcp://127.0.0.1:5580")

    print("[AI SERVISI] Model yukleniyor...")
    model, feature_mean, feature_std = load_model_and_scalers()
    print("[AI SERVISI] Model hazir.")

    ctx = zmq.Context()
    rep = ctx.socket(zmq.REP)
    rep.bind(bind_adres)
    print(f"[AI SERVISI] Hazir -- {bind_adres}")

    while True:
        istek = rep.recv()

        if len(istek) != BEKLENEN_BAYT:
            rep.send_string(f"HATA,beklenmeyen_boyut_{len(istek)}")
            continue

        try:
            degerler = struct.unpack(f"<{PENCERE_BOYUTU * 2}f", istek)
            I = np.array(degerler[0::2], dtype=np.float32)
            Q = np.array(degerler[1::2], dtype=np.float32)

            analog_sayisal, mod, confidence = classify_iq_gated(model, feature_mean, feature_std, I, Q)
            rep.send_string(f"OK,{analog_sayisal},{mod},{confidence:.1f}")
        except Exception as e:
            rep.send_string(f"HATA,{e}")


if __name__ == "__main__":
    main()
