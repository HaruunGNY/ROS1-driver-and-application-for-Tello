#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
import time

def tello_takeoff_land():
    rospy.init_node('tello_takeoff_land', anonymous=True)

    takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size=10)

    rospy.sleep(1)  # ROS'un başlatılmasını bekleyelim

    rospy.loginfo("Drone kalkıyor...")
    takeoff_pub.publish(Empty())  # Kalkış komutu
    rospy.sleep(5)  # 5 saniye havada bekle

    rospy.loginfo("Drone iniş yapıyor...")
    land_pub.publish(Empty())  # İniş komutu

if __name__ == '__main__':
    try:
        tello_takeoff_land()
    except rospy.ROSInterruptException:
        pass
