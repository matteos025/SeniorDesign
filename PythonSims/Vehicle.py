import numpy as np
import numpy.linalg as npl


class Car(object):
    def __init__(self, controller, back_car, front_car, initial_position=np.zeros(2), initial_velocity=np.zeros(2), initial_acceleration=np.zeros(2)):
        self.controller = controller
        self.pos = initial_position
        self.v = initial_velocity
        self.a = initial_acceleration
        self.back_car = back_car
        self.front_car = front_car

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
    def a(self):
        return self._a

    @a.setter
    def a(self, val):
        self._a = val

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

    def calc_vel(self):
        back_vel = self._back_car.v
        back_acc = self._back_car.a
        front_vel = self._front_car.v
        front_acc = self._front_car.a
        new_vel = self._controller.vel_calc(self._v, back_vel, front_vel, self._a, back_acc, front_acc)
        self.v(new_vel)


class Controller(object):
    def __int__(self):
        pass

    # stub method to calculate the car velocity given the cars around it
    def vel_calc(self, my_vel, back_vel, front_vel, my_acc, back_acc, front_acc):
        return my_vel * back_vel - front_vel
