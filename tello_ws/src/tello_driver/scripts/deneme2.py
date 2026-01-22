#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

def tello_move():
    rospy.init_node('tello_move', anonymous=True)

    takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size=10)
    vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)

    rospy.sleep(1)  # ROS'un başlatılmasını bekleyelim

    rospy.loginfo("Drone kalkıyor...")
    takeoff_pub.publish(Empty())  # Kalkış komutu
    rospy.sleep(3)  # Drone'un stabil hale gelmesi için bekleme süresi

    rospy.loginfo("Drone 10 cm ileri gidiyor...")
    move_cmd = Twist()
    move_cmd.linear.y = -0.3  # Y ekseninde ileri hareket
    vel_pub.publish(move_cmd)
    rospy.sleep(1)  # Yaklaşık 10 cm hareket eder

    rospy.loginfo("Drone duruyor...")
    move_cmd.linear.y = 0.0  # Hızı sıfırla
    vel_pub.publish(move_cmd)
    rospy.sleep(1)  # Stabilizasyon için bekle

    rospy.loginfo("Drone 90 derece sola dönüyor...")
    move_cmd.angular.z = -1.0  # Sola dönüş (negatif z açısal hız)
    vel_pub.publish(move_cmd)
    rospy.sleep(1.5)  # 90 derece dönmesi için süre

    rospy.loginfo("Drone duruyor...")
    move_cmd.angular.z = 0.0  # Dönüşü durdur
    vel_pub.publish(move_cmd)
    rospy.sleep(1)

    rospy.loginfo("Drone iniş yapıyor...")
    land_pub.publish(Empty())  # İniş komutu

if __name__ == '__main__':
    try:
        tello_move()
    except rospy.ROSInterruptException:
        pass

