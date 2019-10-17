import numpy as np
import numpy.linalg as npl
from collections import OrderedDict
from numbers import Number
from abc import ABC, abstractmethod


class State_Space(object):
    def __init__(self, variable_dictionary):
        # make sure that the state dictionary is an ordered dict
        self.state_dictionary = variable_dictionary
        self.state_size = {}
        self.total_state_size = 0
        for k in self.state_dictionary.keys():
            if isinstance(self.state_dictionary[k], Number):
                shape = 1
            else:
                shape = len(self.state_dictionary[k])
            self.state_size[k] = shape
            self.total_state_size += shape

    def as_vector(self):
        vector_state = np.zeros(self.total_state_size)
        running_idx = 0
        for k in self.state_dictionary.keys():
            size_state = self.state_size[k]
            vector_state[running_idx:running_idx+size_state] = self.state_dictionary[k]
            running_idx = running_idx+size_state
        return vector_state

    def __getitem__(self, item):
        return self.state_dictionary[item]

    def __setitem__(self, key, value):
        self.state_dictionary[key] = value




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
            initial_velocity=0,
            initial_acceleration=0,
            initial_heading = 0):
        self.controller = controller
        states = OrderedDict()
        states["pos"] = initial_position
        states["v"] = initial_velocity
        states["a"] = initial_acceleration
        states["steering_angle"] = 0
        states["heading"] = initial_heading
        self.states = State_Space(states)


        self.pos = initial_position
        self.v = initial_velocity
        self.a = initial_acceleration
        self.steering_angle = 0
        self.back_car = back_car
        self.front_car = front_car

        # angle of the velocity of the center of mass (inertial heading)
        self.psi = initial_heading

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
        return self.states['pos']

    @pos.setter
    def pos(self, val):
        self.states['pos'] = val

    @property
    def v(self):
        return self.states['v']

    @v.setter
    def v(self, val):
        self.states['v'] = val

    @property
    def speed(self):
        return npl.norm(self.v)

    @property
    def a(self):
        return self.states['a']

    @a.setter
    def a(self, val):
        self.states['a'] = val


    @property
    def psi(self):
        #initertial heading
        return self.states['heading']

    @psi.setter
    def psi(self, val):
        self.states['heading'] = val

    @property
    def heading(self):
        return self.states['heading']

    @heading.setter
    def heading(self, val):
        self.states['heading'] = val

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
        return self.states['steering_angle']

    @steering_angle.setter
    def steering_angle(self, val):
        self.states['steering_angle'] = val

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
        return self.states


    def propogate(self, dt = 0.1):
        """
        Propogate the car forward in time. Uses linearization of derivatives so keep dt < 0.1
        :param dt: time to propogate forward
        """
        self.input_control()
        self.pos += dt*self.d_pos
        if np.isnan(self.pos[0]):
            print()
        self.psi += dt*self.d_psi
        self.v += dt*self.a


    def input_control(self):
        """
        Get an input from the controller
        """
        (self.a, self.steering_angle) = self.controller.get_action(self.states)

    # def calc_vel(self):
    #     back_vel = self._back_car.v
    #     back_acc = self._back_car.a
    #     front_vel = self._front_car.v
    #     front_acc = self._front_car.a
    #     new_vel = self.controller.vel_calc(
    #         self._v, back_vel, front_vel, self._a, back_acc, front_acc)
    #     self.v(new_vel)


