#!/usr/bin/env python

import rospy
from common.pidloop import PIDLoop
from pixy2_msgs.msg import PixyData, Servo, PixyResolution

panLoop = PIDLoop(400, 0, 400, True)
tiltLoop = PIDLoop(500, 0, 500, True)

def pan_tilt():
    pub = rospy.Publisher("servo_cmd", Servo, queue_size=10)

    frameWidth = -1
    frameHeight = -1

    def resolution_callback(data):
        global frameWidth
        global frameHeight
        frameWidth = data.width
        frameHeight = data.height

    def block_data_callback(data):
        global frameHeight
        global frameWidth
        if (frameHeight == -1):
            return

        if (len(data.blocks) > 0):
            block = data.blocks[0]

            panOffset = (frameWidth // 2) - int(data.blocks[0].roi.x_offset)
            tiltOffset = int(data.blocks[0].roi.y_offset) - (frameHeight // 2)

            panLoop.update(panOffset)
            tiltLoop.update(tiltOffset)

            # Publish a servo message
            pub.publish(Servo(0, panLoop.command))
            pub.publish(Servo(1, tiltLoop.command))
        else:
            # No object detected, go into reset state
            panLoop.reset()
            tiltLoop.reset()
            pub.publish(Servo(0, panLoop.command))
            pub.publish(Servo(1, tiltLoop.command))

    rospy.init_node("pixy2_pantilt_demo")
    rospy.Subscriber("block_data", PixyData, block_data_callback)
    rospy.Subscriber("pixy2_resolution", PixyResolution, resolution_callback)

    rospy.spin()

if __name__ == "__main__":
    pan_tilt()
