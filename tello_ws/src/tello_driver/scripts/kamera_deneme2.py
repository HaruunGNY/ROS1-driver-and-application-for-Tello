#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TelloTracking:
    def __init__(self):
        rospy.init_node('tello_tracking', anonymous=False)

        self.bridge = CvBridge()
        rospy.Subscriber("/tello/image_raw", Image, self.image_callback)

        rospy.spin()

    def image_callback(self, msg):
        try:
            # OpenCV formatına çevir
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Görüntüyü HSV formatına çevir
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # **Sadece Tam Siyah Renk İçin Alt ve Üst Sınır**
            lower_black = np.array([0, 0, 0])      # Siyahın en alt tonu
            upper_black = np.array([180, 255, 20])  # En fazla koyu siyah

            # Maske oluştur (sadece tam siyah bölgeler)
            mask = cv2.inRange(hsv, lower_black, upper_black)

            # Kontur bul (siyah alanları takip et)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                rospy.loginfo("Siyah renk tespit edildi!")  # Terminalde çıktı ver
                print("Siyah renk tespit edildi!")  # Konsola yazdır

                # En büyük konturu bul
                largest_contour = max(contours, key=cv2.contourArea)

                # Konturun merkezini hesapla
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Merkeze bir nokta koy
                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)

            # Görüntüyü göster
            cv2.imshow("Tello Kamera", frame)
            cv2.imshow("Maske (Siyah Alanlar)", mask)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Hata: %s", str(e))


if __name__ == '__main__':
    TelloTracking()

