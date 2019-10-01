import numpy as np
import numpy.linalg as npl


class Car(object):
    def __init__(self, initial_position=np.zeros(2), initial_velocity=np.zeros(2), initial_acceleration=np.zeros(2)):
        self.pos = initial_position
        self.v = initial_velocity
        self.a = initial_acceleration

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
