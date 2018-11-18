#!/usr/bin/env python

import rospy
from pixy2_msgs.msg import PixyData, Servo

def pan_tilt():
    pub = rospy.Publisher("servo_cmd", Servo, queue_size=10)

    def block_data_callback(data):
        #Do stuff
        print("whee")

    rospy.init_node("pixy2_pantilt_demo")
    rospy.Subscriber("block_data", PixyData, block_data_callback)

    rospy.spin()

if __name__ == "__main__":
    pan_tilt()