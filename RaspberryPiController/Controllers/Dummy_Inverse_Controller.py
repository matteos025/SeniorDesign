from CONSTANTS import *
from Controllers.ControllerABC import CommunicatingControllerABC
import logging

class InverseController(CommunicatingControllerABC):
    def __init__(
            self,
            steering_angle=0,
            velocity=0,
            communicate_to_arduino=True):
        super().__init__(steering_angle=steering_angle, velocity=velocity,
                         communicate_to_arduino=communicate_to_arduino)
        self.master_steering_angle = 0

    @property
    def master_steering_angle(self):
        return self._master_steering_angle

    @master_steering_angle.setter
    def master_steering_angle(self, val):
        self._master_steering_angle = val
        self.ego_steering_angle = -val

    #do nothing because this is a slave
    def set_steering_angle(self):
        logging.debug("Steering Angle is {}".format(self.ego_steering_angle))

    #do nothing because this is a slave
    def set_velocity(self):
        pass

    def receive_message(self, message):
        self.master_steering_angle = float(message["steering_angle"])



