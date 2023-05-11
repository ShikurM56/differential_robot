#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math
import time

odom_th = 0
target_right = 0

def odom_callback(data):
    global odom_th
    odom_th = data.data
    print ("I'm hearing: ", odom_th)

def publish_commands():
    pub_right = rospy.Publisher('target_right', Float64, queue_size=10)
    pub_left = rospy.Publisher('target_left', Float64, queue_size=10)
    rospy.Subscriber("th_odom_rad", Float64, odom_callback)
    rospy.init_node('main_roboclaw', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        target_right = 0
        target_left = 0
        print ("Target Speed Right [rad/s]: ", target_right)
        print ("Target Speed Left  [rad/s]: ", target_left)


        pub_right.publish(target_right)
        pub_left.publish(target_left)


        rate.sleep()

if __name__ == '__main__':
    try:
        publish_commands()
    except rospy.ROSInterruptException:
        pass
