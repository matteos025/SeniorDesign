import numpy as np
import numpy.linalg as npl
from abc import ABC, abstractmethod

class Vehicle(ABC):
    # @abstractmethod
    # @property
    # def pos(self):
    #     pass

    @abstractmethod
    def propogate(self, dt):
        pass


class Car(Vehicle):
    def __init__(
            self,
            controller,
            back_car,
            front_car,
            initial_position=np.zeros(2),
            initial_velocity=np.zeros(2),
            initial_acceleration=np.zeros(2)):
        self.controller = controller
        self.pos = initial_position
        self.v = initial_velocity
        self.a = initial_acceleration
        self.steering_angle = 0
        self.back_car = back_car
        self.front_car = front_car

        # angle of the velocity of the center of mass (inertial heading)
        self.psi = 0

    @property
    def mass(self):
        """ TODO: implement in class """
        return 10

    @property
    def lr(self):
        """ distance from rear axis to center of mass"""
        return 1  # m

    @property
    def lf(self):
        """distance from center of mass to front axis"""
        return 1  # m

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, val):
        self._pos = val

    @property
    def v(self):
        return self._v

    @v.setter
    def v(self, val):
        self._v = val

    @property
    def speed(self):
        return npl.norm(self.v)

    @property
    def a(self):
        return self._a

    @a.setter
    def a(self, val):
        self._a = val

    @property
    def psi(self):
        return self._psi

    @psi.setter
    def psi(self, val):
        self._psi = val

    @property
    def beta(self):
        return np.arctan2(
            self.lr *
            np.tan(
                self.steering_angle),
            self.lf +
            self.lr)

    @property
    def back_car(self):
        return self._back_car

    @back_car.setter
    def back_car(self, car):
        self._back_car = car

    @property
    def front_car(self):
        return self._front_car

    @front_car.setter
    def front_car(self, car):
        self._front_car = car

    @property
    def steering_angle(self):
        return self._steering_angle

    @steering_angle.setter
    def steering_angle(self, val):
        self._steering_angle = val

    @property
    def d_pos(self):
        """
        :return: local derivative of position
        """
        return self.speed * \
            np.array([np.cos(self.psi + self.beta), np.sin(self.psi + self.beta)])

    @property
    def d_psi(self):
        """
        :return: return local derivative of heading
        """
        return self.speed / self.lr * np.sin(self.beta)

    @property
    def ego_state(self):
        """
        :return: return state of this car
        """
        return np.concatenate((self.pos, self.v, self.a, np.array([self.steering_angle])))

    @property
    def chain_state(self):
        """
        :return: Return the state of the local chain as a 3 x len(state) matrix
        """
        return np.array((self.back_car.state, self.ego_state, self.front_car.state))

    def propogate(self, dt = 0.1):
        """
        Propogate the car forward in time. Uses linearization of derivatives so keep dt < 0.1
        :param dt: time to propogate forward
        """
        self.pos += dt*self.d_pos
        self.psi += dt*self.d_psi
        self.v += dt*self.a

    def input_control(self):
        """
        Get an input from the controller
        """
        (self.a, self.steering_angle) = self.controller.get_action(self.ego_state)

    def calc_vel(self):
        back_vel = self._back_car.v
        back_acc = self._back_car.a
        front_vel = self._front_car.v
        front_acc = self._front_car.a
        new_vel = self.controller.vel_calc(
            self._v, back_vel, front_vel, self._a, back_acc, front_acc)
        self.v(new_vel)


