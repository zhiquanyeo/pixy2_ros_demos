PIXY_RCS_MIN_POS = 20 # was 0
PIXY_RCS_MAX_POS = 980 # was 1000
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
