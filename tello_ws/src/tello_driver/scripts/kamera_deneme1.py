#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image

class TelloCamera:
    def __init__(self):
        rospy.init_node("tello_camera_display", anonymous=True)
        rospy.Subscriber("/tello/image_raw", Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        rospy.loginfo("Görüntü alındı, gösteriliyor...")

        # Gelen ham veriyi numpy array'e çevir
        img_array = np.frombuffer(msg.data, dtype=np.uint8)

        # OpenCV'nin anlayacağı formata sok
        img = img_array.reshape((msg.height, msg.width, 3))

        # Görüntüyü göster
        cv2.imshow("Tello Kamera", img)
        cv2.waitKey(1)

TelloCamera()

