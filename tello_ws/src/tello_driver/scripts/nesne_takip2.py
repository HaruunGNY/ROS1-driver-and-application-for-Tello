#!/usr/bin/env python3
# -- coding: utf-8 --


import cv2
import numpy as np
import os
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from djitellopy import Tello
from sensor_msgs.msg import Image

class TelloObjectTracking:
    def __init__(self):
        rospy.init_node('tello_black_tracking', anonymous=True)

        self.image_pub = rospy.Publisher("/tello/image_raw", Image, queue_size=10)
        self.detection_pub = rospy.Publisher("/tello/black_object_tracking", String, queue_size=10)
        self.bridge = CvBridge()

        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()
        rospy.sleep(2)  # Bağlantının oturması için bekleme süresi
        print("Kamera Aktif")

    def takeoff(self):
        self.drone.takeoff()
        self.drone.move_down(20)
        print(f"Battery: {self.drone.get_battery()}%")

    def detect_black_object(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_contour = None
        max_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500 and area > max_area:
                max_area = area
                largest_contour = contour

        center = None
        if largest_contour is not None:
            x, y, w, h = cv2.boundingRect(largest_contour)
            center = (x + w // 2, y + h // 2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)  # Nesnenin ortasına kırmızı nokta koy
        
        return frame, center

    def track_black_object(self):
        while not rospy.is_shutdown():
            frame = self.drone.get_frame_read().frame
            if frame is None:
                rospy.logerr("Kamera görüntüsü alınamadı!")
                continue
            
            frame, center = self.detect_black_object(frame)
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)

            if center:
                frame_width = frame.shape[1]
                frame_height = frame.shape[0]
                center_x, center_y = center

                if center_x < frame_width // 3:
                    self.drone.move_left(20)
                elif center_x > 2 * frame_width // 3:
                    self.drone.move_right(20)

                if center_y < frame_height // 3:
                    self.drone.move_forward(20)
                elif center_y > 2 * frame_height // 3:
                    self.drone.move_back(20)

                rospy.loginfo(f"Siyah cisim bulundu: {center}")
                self.detection_pub.publish("Siyah cisim bulundu!")
            else:
                rospy.loginfo("Siyah cisim tespit edilmedi.")
                self.detection_pub.publish("Siyah cisim tespit edilmedi.")

            cv2.imshow("Siyah Nesne Takibi", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        self.drone.land()

if __name__ == "__main__":
    drone = TelloObjectTracking()
    drone.takeoff()
    drone.track_black_object()

