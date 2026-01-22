#!/usr/bin/env python3


import cv2
import numpy as np
from djitellopy import Tello
import time

class TelloControl():
    def __init__(self):
        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()

    def takeoff_and_move(self):
        # Drone'u havalandır
        self.drone.takeoff()
        print("Drone havalandı!")
        time.sleep(2)  # 2 saniye bekle

        # 1 metre yüksel
        self.drone.move_up(50)
        print("1 metre yukarı çıkıldı")
        time.sleep(2)

        # 45 cm sola kay
        self.drone.move_left(5)
        print("45 cm sola kayıldı")
        time.sleep(2)

        # 1 metre aşağı in
        self.drone.move_down(500)
        print("1 metre aşağı inildi")
        time.sleep(2)

    def detect_black_object(self):
        bridge = CvBridge()

        while True:
            frame = self.drone.get_frame_read().frame
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Siyah renk aralığını tanımla (HSV formatında)
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 50])

            # Siyah renk için maske oluştur
            mask = cv2.inRange(hsv, lower_black, upper_black)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Eğer çok küçükse nesneyi dikkate alma
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    print("Siyah nesne tespit edildi!")

                    # Kullanıcıya bildirim
                    self.send_notification()

            cv2.imshow("Kamera Görüntüsü", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.drone.land()
        cv2.destroyAllWindows()

    def send_notification(self):
        print("Kullanıcıya bildirim gönderildi: Siyah nesne duvarda tespit edildi.")

# Ana program
tello_control = TelloControl()
tello_control.takeoff_and_move()
tello_control.detect_black_object()
