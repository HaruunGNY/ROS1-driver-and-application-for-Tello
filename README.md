#  Tello ROS Sürücüsü ve Kontrol Sistemi

Bu proje, DJI Tello drone'larını **ROS (Robot Operating System)** üzerinde tam yetkiyle kontrol etmek, kamera görüntüsünü işlemek ve telemetri verilerini takip etmek için geliştirilmiş bir altyapıdır. Proje, `TelloPy` kütüphanesini temel alarak ROS ekosistemine entegre eder.

---

##  Öne Çıkan Özellikler

*   ** Gerçek Zamanlı Video:** Drone kamerasından alınan görüntüyü h264 veya raw formatında ROS topic'i üzerinden yayınlama.
*   ** Gamepad Desteği:** PS3/PS4 veya benzeri kumandalarla manuel kontrol imkanı.
*   ** Telemetri Takibi:** Batarya durumu, yükseklik, hız, IMU ve Odometri verilerinin anlık izlenmesi.
*   ** Gelişmiş Komutlar:** Kalkış (Takeoff), İniş (Land), Avuca İniş (Palm Land), Takla (Flip) ve Acil Durum Stop desteği.
*   ** Dinamik Yapılandırma:** Yazılım çalışırken uçuş limitlerini ve kamera ayarlarını değiştirebilme.

---

##  Kurulum

### 1. Bağımlılıkların Yüklenmesi
Proje için gerekli olan kütüphaneleri yükleyin:

```bash
# Video işleme için PyAV
pip3 install av --user

# ROS Image Transport paketleri
sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge
```

### 2. Workspace Kurulumu
Eğer bir workspace'iniz yoksa oluşturun ve repoyu içine klonlayın:

```bash
mkdir -p ~/tello_ws/src
cd ~/tello_ws/src
# Repoyu klonlayın (klasör yapısına dikkat edin)
git clone https://github.com/HaruunGNY/ROS1-driver-and-application-for-Tello.git
*
catkin_make
source devel/setup.bash
```

---

##  Kullanım

### Drone Bağlantısı
1.  Tello drone'unu açın.
2.  Bilgisayarınızdan drone'un WiFi ağına (`TELLO_XXXXXX`) bağlanın.

### Çalıştırma
Ana sürücüyü başlatmak için:
```bash
roslaunch tello_driver tello_node.launch
```

Gamepad ile kontrol etmek isterseniz (yeni terminalde):
```bash
roslaunch tello_driver joy_teleop.launch
```

---

##  Node Yapısı

### 1. `tello_driver_node`
Drone ile ROS arasındaki köprüdür.
*   **Yayınlanan Topic'ler:**
    *   `/tello/image_raw`: Kamera görüntüsü.
    *   `/tello/status`: Batarya, uçuş süresi vb. bilgiler.
    *   `/tello/odom` & `/tello/imu`: Konum ve sensör verileri.
*   **Abone Olunan Topic'ler:**
    *   `/tello/cmd_vel`: Hareket komutları.
    *   `/tello/takeoff` & `/tello/land`: Kalkış ve iniş.

### 2. `gamepad_teleop_node`
Joystick girişlerini drone komutlarına dönüştürür.
*   **Kontroller:**
    *   **L1:** Kalkış
    *   **R1:** İniş
    *   **R2:** Acil Durum Stop
    *   **Analog Stickler:** Hareket ve Dönüş

---

##  Notlar
*   **Bağlantı Sorunu:** Video gelmiyorsa drone bağlantısını ve `tello_ip` parametresini kontrol edin (Varsayılan: `192.168.10.1`).
*   **Kamera Kalibrasyonu:** `cfg` klasörü altındaki `.yaml` dosyaları ile kamera kalibrasyonu yapılabilir.

---




##  Proje Uygulama ve Görselleştirme

Bu bölümde sistemin uçuş performansı ve ROS üzerinden işlenen kamera verileri yer almaktadır.

###  Tello FPV & Telemetri Verisi (Drone Kamerası)
Aşağıdaki video, Tello'nun onboard kamerasından alınan görüntünün `/tello/image_raw` topic'i üzerinden ROS ekosistemine aktarılmış halidir.




https://github.com/user-attachments/assets/327ea071-0ca7-4016-be3c-961d400ad3d1




###  Uçuş Operasyonu (Dış Çekim)
Sistemin fiziksel ortamdaki kararlılığı, kalkış/iniş manevraları ve gamepad ile olan tepki süresi aşağıdaki videoda görülmektedir:





https://github.com/user-attachments/assets/b386914a-c32d-4e58-81cb-5b1a752b93e8



---
*Görüntü aktarımında gecikmeyi (latency) minimize etmek için h264 decoding optimize edilmiştir.*
