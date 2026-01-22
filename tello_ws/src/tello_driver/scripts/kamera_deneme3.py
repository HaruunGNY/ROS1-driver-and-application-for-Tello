#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty  # Takeoff ve Land komutları için

class TelloTracking:
    def __init__(self):
        rospy.init_node('tello_tracking', anonymous=False)

        self.bridge = CvBridge()
        rospy.Subscriber("/tello/image_raw", Image, self.image_callback)

        # Hareket komutları için publisher
        self.cmd_pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=10)

        # Takeoff ve Land komutları için publisher
        self.takeoff_pub = rospy.Publisher("/tello/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("/tello/land", Empty, queue_size=1)

        rospy.sleep(2)  # ROS düğümünün başlatılmasını bekle

        self.takeoff()  # Otomatik kalkış yap
        rospy.sleep(5)  # Kalkışı tamamlaması için biraz bekle

        rospy.spin()

    def takeoff(self):
        """Drone'u havalandırır"""
        rospy.loginfo("🚀 Tello Kalkış Yapıyor...")
        self.takeoff_pub.publish(Empty())

    def land(self):
        """Drone'u indirir"""
        rospy.loginfo("🛬 Tello İniş Yapıyor...")
        self.land_pub.publish(Empty())

    def image_callback(self, msg):
        try:
            # OpenCV formatına çevir
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (640, 480))

            # Görüntüyü HSV formatına çevir
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Sarı renk için alt ve üst HSV değerleri
            lower_yellow = np.array([20, 100, 100])  
            upper_yellow = np.array([30, 255, 255])

            # Maske oluştur
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Maske üzerinde erozyon ve genişleme (gürültüyü azaltmak için)
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.erode(mask, None, iterations=1)

            # Kontur bul
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # En büyük konturu bul
                largest_contour = max(contours, key=cv2.contourArea)

                # Konturun merkezini hesapla
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Merkeze bir nokta koy
                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)

                    # Çizginin merkezine göre yönlendirme yap
                    self.follow_line(cx, frame.shape[1])

            # Görüntüyü göster
            cv2.imshow("Tello Kamera", frame)
            cv2.imshow("Maske", mask)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Hata: %s", str(e))

    def follow_line(self, cx, frame_width):
        """Çizgiyi takip etmek için drone'u yönlendirir"""
        offset = cx - (frame_width // 2)
        twist = Twist()

        # Hassasiyet eşiği
        threshold = 50  

        if abs(offset) > threshold:
            if offset > 0:
                twist.angular.z = -0.3  # Sağa dön
            else:
                twist.angular.z = 0.3   # Sola dön
        else:
            twist.linear.x = 0.2  # İleri git

        # Komutu yayınla
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        tello = TelloTracking()
        rospy.sleep(30)  # 30 saniye çizgi takibi yap
        tello.land()  # Sonrasında iniş yap
    except rospy.ROSInterruptException:
        pass

