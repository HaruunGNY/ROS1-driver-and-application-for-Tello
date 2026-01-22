#!/usr/bin/env python3
# -- coding: utf-8 --

import cv2
import numpy as np  # NumPy kütüphanesini ekleyin
import os
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from djitellopy import Tello
from sensor_msgs.msg import Image

class TelloObjectDetection:
    def __init__(self):
        rospy.init_node('tello_siyah_tespit', anonymous=True)

        self.image_pub = rospy.Publisher("/tello/image_raw", Image, queue_size=10)
        self.detection_pub = rospy.Publisher("/tello/siyah_cisim_tespit", String, queue_size=10)
        self.bridge = CvBridge()

        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()

        rospy.sleep(2)  # Bağlantının oturması için bekleme süresi
        print("Kamera Aktif")

        # Resimlerin kaydedileceği klasör
        self.save_folder = "/home/harun/tello_images"  # Klasör yolu
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)  # Eğer klasör yoksa oluşturulur

    def takeoff(self):
        """Drone'u havalandırır ve hafif aşağı hareket ettirir."""
        self.drone.takeoff()
        self.drone.move_down(20)
        print(f"Battery: {self.drone.get_battery()}%")

    def detect_black_objects(self, frame):
        """Görüntüdeki siyah cisimleri tespit eder."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Siyah renk için HSV aralığı
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Konturları bulalım
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        black_objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Eğer kontur yeterince büyükse
                x, y, w, h = cv2.boundingRect(contour)
                black_objects.append((x, y, w, h))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return frame, black_objects

    def save_picture(self, frame, filename):
        """Drone'dan alınan görüntüyü kaydeder."""
        file_path = os.path.join(self.save_folder, filename)
        cv2.imwrite(file_path, frame)
        print(f"Fotoğraf kaydedildi: {file_path}")

    def move_and_detect(self):
        """Drone'u hareket ettirir ve siyah cisimleri tespit eder."""
        while not rospy.is_shutdown():
            # 20 cm ile 100 cm arasında yükseklik değişimi
            for altitude in range(20, 101, 20):  # 20, 40, 60, 80, 100 cm
                self.drone.move_up(20)
                print(f"Yükseklik: {altitude} cm")
                
                # Siyah cisimleri tespit et
                frame = self.drone.get_frame_read().frame
                if frame is None:
                    rospy.logerr("Kamera görüntüsü alınamadı!")
                    continue

                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(ros_image)

                frame, black_objects = self.detect_black_objects(frame)

                if black_objects:
                    rospy.loginfo(f"Siyah cisim tespit edildi! {len(black_objects)} adet cisim")
                    self.detection_pub.publish("Siyah cisim tespit edildi!")  # Kullanıcıya bildirim
                else:
                    rospy.loginfo("Siyah cisim tespit edilmedi.")
                    self.detection_pub.publish("Siyah cisim tespit edilmedi.")  # Kullanıcıya bildirim
                
                # Görüntüyü göster
                cv2.imshow("Kamera", frame)

                # Fotoğraf kaydetmek için tuşa basma kontrolü
                if cv2.waitKey(1) & 0xFF == ord('s'):  # 's' tuşuna basarak fotoğrafı kaydedin
                    filename = f"tello_picture_{rospy.get_time()}.jpg"
                    self.save_picture(frame, filename)

                if cv2.waitKey(1) & 0xFF == ord('a'):
                    rospy.loginfo("Acil iniş yapılıyor!")
                    self.drone.land()  # 'a' tuşuna basıldığında drone yere iner
                    break

            # 20 cm sağa hareket et
            self.drone.move_right(25)
            rospy.loginfo("Drone sağa hareket ediyor.")
            
            # Tekrar iniyor
            self.drone.move_down(100)
            rospy.loginfo("Drone yere iniyor.")
            
                        # 20 cm sağa hareket et
            self.drone.move_right(25)
            rospy.loginfo("Drone sağa hareket ediyor.")
            

            # Yine aynı hareketi yapacak
            rospy.loginfo("Yeni hareket başlıyor...")

        cv2.destroyAllWindows()

# Çalıştırma
iha = TelloObjectDetection()
iha.takeoff()
iha.move_and_detect()

