#!/usr/bin/env python

import rospy
from common.pidloop import PIDLoop
from pixy2_msgs.msg import PixyData, PixyResolution, Servo
from geometry_msgs.msg import Twist

PIXY_RCS_MIN_POS = 20 # was 0
PIXY_RCS_MAX_POS = 980 # was 1000
PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2)

panLoop = PIDLoop(400, 0, 400, True)
tiltLoop = PIDLoop(500, 0, 500, True)
rotateLoop = PIDLoop(300, 600, 300, False)
translateLoop = PIDLoop(400, 800, 300, False)

#rotateLoop = PIDLoop(5.25, 10.5, 5.25, False) #0.8m/s max, 7 rad/s max
#translateLoop = PIDLoop(0.8, 1.6, 0.6, False)

MAX_TRANSLATE_VELOCITY = 250 # we convert this to 0.8 later
#MAX_TRANSLATE_VELOCITY = 0.8


trackedIndex = -1
trackedBlock = None
frameWidth = -1
frameHeight = -1

def chase():
    servoPub = rospy.Publisher("servo_cmd", Servo, queue_size=10)
    cmdvelPub = rospy.Publisher("/move_base/cmd_vel", Twist, queue_size=10)

    def acquireBlock(blocks):
        if len(blocks) > 0 and blocks[0].age > 30:
            return blocks[0].index
        return -1

    def trackBlock(blocks, index):
        i = 0
        for candidate in blocks:
            if candidate.index == index:
                return candidate

        return None

    def resolution_callback(data):
        global frameWidth
        global frameHeight
        frameWidth = data.width
        frameHeight = data.height

    def block_data_callback(data):
        global frameWidth, frameHeight, trackedIndex, trackedBlock
        if frameHeight == -1:
            return

        if trackedIndex == -1: #search
            trackedIndex = acquireBlock(data.blocks)
            if trackedIndex >= 0:
                print("Found block!")

        if trackedIndex >= 0: # Found a block, now track it
            trackedBlock = trackBlock(data.blocks, trackedIndex)

        if trackedBlock != None:
            # we have a block!
            panOffset = (frameWidth // 2) - int(trackedBlock.roi.x_offset)
            tiltOffset = int(trackedBlock.roi.y_offset) - (frameHeight // 2)

            panLoop.update(panOffset)
            tiltLoop.update(tiltOffset)

            # move the servos
            servoPub.publish(Servo(0, panLoop.command))
            servoPub.publish(Servo(1, tiltLoop.command))

            # calculate translate and rotate errors
            panOffset = panOffset + panLoop.command - PIXY_RCS_CENTER_POS
            tiltOffset = tiltOffset + tiltLoop.command - PIXY_RCS_CENTER_POS - (PIXY_RCS_CENTER_POS/2) + PIXY_RCS_CENTER_POS/8

            rotateLoop.update(panOffset)
            translateLoop.update(-tiltOffset)

            print("rotate: %d, translate: %d" % (rotateLoop.command, translateLoop.command))
            # keep translation velocity below maximum
            if translateLoop.command > MAX_TRANSLATE_VELOCITY:
                translateLoop.command = MAX_TRANSLATE_VELOCITY

            twistMsg = Twist()
            twistMsg.linear.z = translateLoop.command
            twistMsg.angular.z = rotateLoop.command

            cmdvelPub.publish(twistMsg)
        else:
            # no object detected. Stop motor, go into search state
            rotateLoop.reset()
            translateLoop.reset()
            cmdvelPub.publish(Twist())
            trackedIndex = -1
            trackedBlock = None

    rospy.init_node("pixy2_chase_demo")

    rospy.Subscriber("block_data", PixyData, block_data_callback)
    rospy.Subscriber("pixy2_resolution", PixyResolution, resolution_callback)

    rospy.spin()

if __name__ == "__main__":
    chase()


