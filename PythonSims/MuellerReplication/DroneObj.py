import numpy as np
from abc import ABC, abstractmethod, abstractclassmethod
from sympy import Symbol


class Drone(ABC):
    def __init__(
            self,
            initial_position=None,
            initial_velocity=None,
            initial_normal=None):
        # TOTAL MOMENTS
        self._IxxT = None  # kg * m^2
        self._IzzT = None  # kg * m^2
        self._IzzP = None  # kg * m^2
        self._IxxP = None  # kg * m^2

        # MASS
        self._mass = None  # kg

        # DRONE ARM LENGTH
        self._l = None  # m

        # thrust constant
        self._kappa_f = None  # N*s^2/rad^2
        # reaction torque constant
        self._kappa_tau = None  # N*m/N
        # drag constant
        self._gamma = None

        # normal vector
        self.n = initial_normal

        # position vector
        self.position = initial_position
        # velocity vector
        self.velocity = initial_velocity
        # acceleration vector
        self.acceleration = np.array([0, 0, -9.8])

        # vehicle angular velocities
        self.p = None
        self.q = None
        self.r = None

        # propeller angular velocities
        self.omega1 = None
        self.omega2 = None
        self.omega3 = None
        self.omega4 = None

    @property
    def IxxT(self):
        return self._IxxT

    @property
    def IzzT(self):
        return self._IzzT

    @property
    def IzzP(self):
        return self._IzzP

    @property
    def IxxP(self):
        return self._IxxP

    @property
    def mass(self):
        return self._mass

    @property
    def l(self):
        return self._l

    @property
    def kappa_f(self):
        return self._kappa_f

    @property
    def kappa_tau(self):
        return self._kappa_tau

    @property
    def gamma(self):
        return self._gamma

    @property
    def omegaB(self):
        return np.array([self.p, self.q, self.r])

    @property
    def x(self):
        return self.position[0]

    @property
    def y(self):
        return self.position[1]

    @property
    def z(self):
        return self.position[2]

    @property
    def d(self):
        return self.position

    @property
    def nx(self):
        return self.n[0]

    @property
    def ny(self):
        return self.n[1]

    @property
    def nz(self):
        return self.n[2]

    @property
    def position_state_space(self):
        return np.concatenate(
            (self.position, self.velocity, self.acceleration))

    @property
    def omega1(self):
        return self._omega1

    @omega1.setter
    def omega1(self, val):
        self._omega1 = val

    @property
    def omega2(self):
        return self._omega2

    @omega2.setter
    def omega2(self, val):
        self._omega2 = val

    @property
    def omega3(self):
        return self._omega3

    @omega3.setter
    def omega3(self, val):
        self._omega3 = val

    @property
    def omega4(self):
        return self._omega4

    @omega4.setter
    def omega4(self, val):
        self._omega4 = val

    @property
    def f1(self):
        return self.kappa_f * self.omega1**2

    @property
    def f2(self):
        return self.kappa_f * self.omega2**2

    @property
    def f3(self):
        return self.kappa_f * self.omega3**2

    @property
    def f4(self):
        return self.kappa_f * self.omega4**2\

    @property
    def tau1(self):
        return self.kappa_tau * self.f1

    @property
    def tau2(self):
        return -self.kappa_tau * self.f2

    @property
    def tau3(self):
        return self.kappa_tau * self.f3

    @property
    def tau4(self):
        return -self.kappa_tau * self.f4

    @property
    def tau_drag(self):
        return -self.gamma * self.r

    @property
    def tau_drag_vect(self):
        return np.array([0, 0, self.tau_drag])

    def propogate_position_state_space(self, dt):
        """
        Propogate position forward under constant velocity assumption
        Propogate acceleration forward under constant acceleration assumption
        :param dt: time interval to propogate forward. Keep this small
        """
        A = np.array([[1, 0, 0, dt, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, dt, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, dt, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0, dt, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, dt, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1]])
        self.position = A @ self.position_state_space


class MuellerDrone(Drone):
    def __init__(self):
        super().__init__()
        # TOTAL MOMENTS
        self._IxxT = 3.2e-3  # kg * m^2
        self._IzzT = 5.5e-3  # kg * m^2
        self._IzzP = 5.5e-3  # kg * m^2
        self._IxxP = 0  # kg * m^2

        # MASS
        self._mass = 0.5  # kg

        # DRONE ARM LENGTH
        self._l = 0.17  # m

        # thrust constant
        self._kappa_f = 6.41e-6  # N*s^2/rad^2
        # reaction torque constant
        self._kappa_tau = 1.69e-2  # N*m/N
        # drag constant
        self._gamma = 2.75e-3

        self.omega4 = 0

    @property
    def rho(self):
        return self.f2 / self.f1


class MuellerDrone3(MuellerDrone):
    def __init__(self):
        super().__init__()

    @property
    def omega4(self):
        return self.omega4

    @omega4.setter
    def omega4(self, val):
        raise ValueError("Fourth rotor is broken on MuellerDrone")


drone = MuellerDrone()

print(drone.x)
