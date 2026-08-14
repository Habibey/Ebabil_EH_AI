# Ebabil Takımı - Elektronik Harp AI Modeli 🦅

Bu depo, Teknofest Elektronik Harp yarışması için geliştirilen yapay zeka tabanlı sinyal tespit ve sınıflandırma sisteminin (SDR) arka plan kodlarını içerir. 

Sistem, PlutoSDR veya RTL-SDR üzerinden gelen 2x128 boyutundaki IQ verilerini işleyerek sinyal tespiti yapar ve ZeroMQ mimarisi üzerinden Qt arayüzü ile haberleşir.

## 🚀 Özellikler
* **Yüksek Hız, Düşük Boyut:** 2.32 MB boyutunda, milisaniyeler içinde tepki veren optimize edilmiş model mimarisi.
* **Haberleşme:** Qt arayüzü ile asenkron entegrasyon için ZeroMQ (Port 5556/5557) altyapısı.
* **Squelch Filtresi:** Boş kanalları filtrelemek için dinamik enerji eşiği.

## 🛠️ Kurulum Adımları
Sistemi bilgisayarınızda çalıştırmak için aşağıdaki adımları sırasıyla uygulayın:

**1. Repoyu Klonlayın**
```bash
git clone [https://github.com/Habibey/Ebabil_EH_AI.git](https://github.com/Habibey/Ebabil_EH_AI.git)
cd Ebabil_EH_AI
2. Model Dosyasını İndirin (ÖNEMLİ)
GitHub 100 MB sınırından dolayı model dosyası kodların içinde yer almamaktadır.

Sayfanın sağ tarafındaki Releases (Sürümler) sekmesine tıklayın.

v1.0.0 sürümü altındaki teknofest_model_v5_best.keras dosyasını bilgisayarınıza indirin.

İndirdiğiniz bu dosyayı klonladığınız projenin içindeki models/ klasörünün içine kopyalayın.

3. Gerekli Kütüphaneleri Kurun
Sanal ortamınızın (Conda vb.) aktif olduğundan emin olduktan sonra gerekli paketleri yükleyin:
pip install tensorflow pyzmq numpy
python src/predict.py
