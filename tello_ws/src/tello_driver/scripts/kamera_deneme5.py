#!/usr/bin/env python3
# -- coding: utf-8 --

import cv2
import numpy as np 
import rospy
from cv_bridge import CvBridge
from djitellopy import Tello
from sensor_msgs.msg import Image

class TelloTracker:
    def __init__(self):
        rospy.init_node('tello_tracker', anonymous=True)
        
        self.image_pub = rospy.Publisher("/tello/image_raw", Image, queue_size=10)
        
        self.bridge = CvBridge()
        
        self.drone = Tello()
        self.drone.connect()
        
        self.drone.streamon()
        rospy.sleep(2)  # ROS düğümlerinin başlatılması için bekleme süresi
        print("Kamera Aktif")

    def takeoff(self):
        self.drone.takeoff()
        rospy.sleep(2)  # Havalanma için bekleme süresi
        self.drone.move_down(20)
        print(f"Battery: {self.drone.get_battery()}%")

    def publish_camera_feed(self):
        cv2.startWindowThread()  # OpenCV'nin pencereyi düzgün açması için
        cv2.namedWindow("Kamera", cv2.WINDOW_NORMAL)  # Pencereyi manuel oluştur
        
        while not rospy.is_shutdown():
            frame = self.drone.get_frame_read().frame
            if frame is None:
                continue
            
            # ROS için görüntüyü dönüştür ve yayınla
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)

            # Sarı rengi algıla
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])         
            upper_yellow = np.array([40, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Görüntü ortasına bir kırmızı nokta koy
            h, w, _ = frame.shape
            cv2.circle(frame, (int(w/2), int(h/2)), 5, (0, 0, 255), -1)

            # Sarı nesnenin merkezini bul
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)  # Sarı nesnenin merkezini işaretle
                
                deviation = int((cx - w/2) / 10)  # Görüntü merkezine göre sapma
                self.drone.send_rc_control(0, 30, 0, deviation)  # İleri hareket ve yönlendirme
            else:
                self.drone.send_rc_control(0, 0, 0, 0)  # Nesne yoksa dur

            # Görüntüyü ekranda göster
            cv2.imshow("Kamera", frame)
            cv2.imshow("Maske", mask)

            # 'q' tuşuna basılırsa iniş yap ve çık
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.drone.land()
                break

        cv2.destroyAllWindows()  # Pencereleri yalnızca döngüden çıktıktan sonra kapat

if __name__ == "__main__":
    try:
        iha = TelloTracker()
        iha.takeoff()
        iha.publish_camera_feed()
    except rospy.ROSInterruptException:
    
        pass
