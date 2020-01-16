import numpy as np

#Network Communication Constants
ENCODING = "utf-8"
DEFAULT_IP = '127.0.0.1'


CONTROL_SAMPLING_TIME = 0.1
STEERING_ANGLE_MIN = -30 #DEGREES
STEERING_ANGLE_MAX = 30 #DEGREES
VELOCITY_MIN = 0 #cm/s
VELOCITY_MAX = 255 #cm/s (currently pwm)

#Arduino Communication Constants
VELOCITY_REGISTER = 1
STEERING_ANGLE_REGISTER = 2


