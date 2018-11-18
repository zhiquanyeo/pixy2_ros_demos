#!/usr/bin/env python

import rospy
from pixy2_msgs.msg import PixyData, Servo, PixyResolution

PIXY_RCS_MIN_POS = 0
PIXY_RCS_MAX_POS = 1000
PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2)

PID_MAX_INTEGRAL = 2000

class PIDLoop:
    def __init__(self, pgain, igain, dgain, servo):
        self.pgain = pgain
        self.igain = igain
        self.dgain = dgain
        self.isServo = servo

        self.command = 0
        self.prevError = 0
        self.integral = 0

        self.reset()

    def reset(self):
        if (self.isServo):
            self.command = PIXY_RCS_CENTER_POS
        else:
            self.command = 0

        self.integral = 0
        self.prevError = 0x80000000

    def update(self, error):
        pid = 0

        if (self.prevError != 0x80000000):
            # integrate integral
            self.integral += error

            # bound the integral
            if (self.integral > PID_MAX_INTEGRAL):
                self.integral = PID_MAX_INTEGRAL
            elif (self.integral < -PID_MAX_INTEGRAL):
                self.integral = -PID_MAX_INTEGRAL

            # calculate PID term
            pid = (error * self.pgain + ((self.integral * self.igain) >> 4) + (error - self.prevError) * self.dgain) >> 10

            if (self.isServo):
                self.command += pid
                if (self.command > PIXY_RCS_MAX_POS):
                    self.command = PIXY_RCS_MAX_POS
                elif (self.command < PIXY_RCS_MIN_POS):
                    self.command = PIXY_RCS_MIN_POS
            else:
                self.command = pid

        self.prevError = error

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
        if (frameHeight == -1):
            return

        if (len(data.blocks) > 0):
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
