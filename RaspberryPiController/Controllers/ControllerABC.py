from abc import ABC, abstractmethod
from ArduinoCommunicator import ArduinoCommunicator
from CONSTANTS import *
import time
import logging

class Controller(ABC):
    def __init__(
            self,
            steering_angle=0,
            velocity=0,
            communicate_to_arduino=True):
        self.communicate_to_arduino = communicate_to_arduino
        self.arduino_communicator = ArduinoCommunicator() if self.communicate_to_arduino else None

        self.ego_velocity = velocity
        self.ego_steering_angle = steering_angle


    @property
    def ego_velocity(self):
        return self._ego_velocity

    @ego_velocity.setter
    def ego_velocity(self, val):
        self._ego_velocity = val
        if self.communicate_to_arduino:
            self.arduino_communicator.write_float_to_register(
                self.ego_velocity, VELOCITY_REGISTER)

    @property
    def ego_steering_angle(self):
        return self._ego_steering_angle

    @ego_steering_angle.setter
    def ego_steering_angle(self, val):
        self._ego_steering_angle = val
        if self.communicate_to_arduino:
            self.arduino_communicator.write_float_to_register(
                self.ego_steering_angle, STEERING_ANGLE_REGISTER)

    @abstractmethod
    def set_velocity(self):
        pass

    @abstractmethod
    def set_steering_angle(self):
        pass

    def run(self):
        ctr = 0
        while True:
            self.set_velocity()
            self.set_steering_angle()
            if ctr % 10 == 0:
                # logging.debug("Velocity: {} \n Steering Angle: {}\n".format(self.ego_velocity, self.ego_steering_angle))
                ctr = 0
            ctr += 1
            time.sleep(0.1)


class PlatoonControllerABC(Controller, ABC):
    def __init__(self, steering_angle=0,
            velocity=0,
            communicate_to_arduino=True, zero_vel_offset = 0.5, vel_distance_coeff = 0.1):
        super().__init__(steering_angle=steering_angle, velocity=velocity, communicate_to_arduino=communicate_to_arduino)
        self.zero_velocity_offset = zero_vel_offset
        self.velocity_distance_coefficient = vel_distance_coeff

    @property
    def leader_velocity(self):
        return self._leader_velocity

    @leader_velocity.setter
    def leader_velocity(self, val):
        self._leader_velocity = val

    @property
    def distance_error(self):
        #TODO
        return 0

    @property
    def velocity_error(self):
        return self.leader_velocity-self.ego_velocity

    @property
    @abstractmethod
    def state(self):
        pass

