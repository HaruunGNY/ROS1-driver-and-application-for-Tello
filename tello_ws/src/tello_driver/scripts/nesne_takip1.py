#!/usr/bin/env python3
# -- coding: utf-8 --

import cv2
import numpy as np
from djitellopy import Tello

# Tello drone bağlantısını başlat
drone = Tello()
drone.connect()
drone.streamon()

def track_black_object(frame):
    """Siyah nesneyi algılar ve merkezini döndürür"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])
    mask = cv2.inRange(hsv, lower_black, upper_black)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        center_x, center_y = x + w // 2, y + h // 2
        return center_x, center_y, w * h  # Nesnenin merkez koordinatı ve alanı
    return None, None, 0

def move_drone(x, y, area, frame_width, frame_height):
    """Drone'u nesnenin konumuna göre hareket ettirir"""
    center_x, center_y = frame_width // 2, frame_height // 2
    move_x, move_y, move_z = 0, 0, 0

    if x is not None:
        if x < center_x - 50:
            move_x = -20  # Sola git
        elif x > center_x + 50:
            move_x = 20   # Sağa git

        if y < center_y - 50:
            move_y = 20   # Yukarı git
        elif y > center_y + 50:
            move_y = -20  # Aşağı git

        if area < 3000:
            move_z = 20   # İleri git
        elif area > 10000:
            move_z = -20  # Geri git

        drone.send_rc_control(move_x, move_z, move_y, 0)

# Drone havalanıyor
drone.takeoff()

while True:
    frame = drone.get_frame_read().frame
    height, width, _ = frame.shape
    x, y, area = track_black_object(frame)
    move_drone(x, y, area, width, height)

    cv2.imshow("Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

drone.land()
cv2.destroyAllWindows()
