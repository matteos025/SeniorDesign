import numpy as np
import numpy.linalg as npl
from abc import ABC, abstractmethod
import json


class Drone(ABC):
    def __init__(
            self,
            initial_position=np.zeros(3),
            initial_velocity=np.zeros(3),
            initial_normal=np.array([1, 0, 0])):
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
        self.p = 0
        self.q = 0
        self.r = 0

        # propeller angular velocities
        self.omega1 = 0
        self.omega2 = 0
        self.omega3 = 0
        self.omega4 = 0

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


class ReducedAttitudeDrone(Drone):
    @property
    @abstractmethod
    def p_bar(self):
        pass

    @property
    @abstractmethod
    def q_bar(self):
        pass

    @property
    def r_bar(self):
        """
        This one is independent of the rotor cases
        :return:
        """
        scale_fact = self.kappa_tau * self.kappa_f / self.gamma
        summed_spin = self.omega1_bar ** 2 - self.omega2_bar  ** 2 + \
            self.omega3_bar  ** 2 - self.omega4_bar  ** 2
        return scale_fact * summed_spin

    @property
    def nx_bar(self):
        return self.eps * self.p_bar

    @property
    def ny_bar(self):
        return self.eps * self.q_bar

    @property
    def nz_bar(self):
        return self.eps * self.r_bar

    @property
    def eps(self):
        return 1 / npl.norm(self.omegaB_bar)

    @property
    def omegaB_bar(self):
        return np.array([self.p_bar, self.q_bar, self.r_bar])

    @property
    @abstractmethod
    def omega1_bar(self):
        pass

    @property
    @abstractmethod
    def omega2_bar(self):
        pass

    @property
    @abstractmethod
    def omega3_bar(self):
        pass

    @property
    @abstractmethod
    def omega4_bar(self):
        pass

    @property
    def Tps(self):
        return 2 * np.pi / npl.norm(self.omegaB_bar)

    @property
    def Rps_bar(self):
        return np.sqrt(1 - self.nz_bar**2) / self.nz_bar * \
            9.8 / npl.norm(self.omegaB_bar)**2

    @property
    def f1_bar(self):
        return self.kappa_f * self.omega1_bar**2

    @property
    def f2_bar(self):
        return self.kappa_f * self.omega2_bar**2

    @property
    def f3_bar(self):
        return self.kappa_f * self.omega3_bar**2

    @property
    def f4_bar(self):
        return self.kappa_f * self.omega4_bar**2


class MuellerDrone(Drone):
    def __init__(self, initial_position=np.zeros(3),
                 initial_velocity=np.zeros(3),
                 initial_normal=np.array([1, 0, 0])):
        super().__init__(initial_position,
                         initial_velocity,
                         initial_normal)
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


class MuellerDrone3(MuellerDrone, ReducedAttitudeDrone):
    def __init__(self):
        super().__init__()
        # This is a tuning parameter
        self._rho = 0.5


        self._p_bar = 0

        # in general p_bar can be obtained using this
        # factor = self.r_bar * (self.IzzT - self.IxxT) + self.IzzP * (
        #         self.omega1_bar + self.omega2_bar + self.omega3_bar + self.omega4_bar)
        #self._p_bar = -self.kappa_f * (self.omega3_bar ** 2 - self.omega1_bar ** 2) * self.l / factor
        factor = self.r_bar * (self.IzzT - self.IxxT) + self.IzzP * (
            self.omega1_bar + self.omega2_bar + self.omega3_bar + self.omega4_bar)
        self._q_bar = self.kappa_f * \
            (self.omega2_bar ** 2 - self.omega4_bar ** 2) * self.l / factor

        # in general p_bar can be obtained using this
        # factor = self.r_bar * (self.IzzT - self.IxxT) + self.IzzP * (
        #     self.omega1_bar + self.omega2_bar + self.omega3_bar + self.omega4_bar)
        # self._q_bar = self.kappa_f * \
        #     (self.omega2_bar ** 2 - self.omega4_bar ** 2) * self.l / factor



    @property
    def omega4(self):
        return 0

    @omega4.setter
    def omega4(self, val):
        if val != 0:
            raise Warning("Attempting to set rotor 4 angular speed to {}. This rotor is broken so this value is coerced to 0".format(val))

    @property
    def p_bar(self):
        return self._p_bar

    @property
    def q_bar(self):
        return self._q_bar

    @property
    def rho(self):
        return self._rho

    @property
    def f1_bar(self):
        return self.mass * 9.8 / ((2 + self.rho) * self.nz_bar)

    @property
    def omega1_bar(self):
        return np.sqrt(self.f1_bar / self.kappa_tau)

    @property
    def f2_bar(self):
        return self.rho * self.f1_bar

    @property
    def omega2_bar(self):
        return np.sqrt(self.f2_bar / self.kappa_tau)

    @property
    def omega3_bar(self):
        return self.omega1_bar

    @property
    def omega4_bar(self):
        return 0


if __name__ == "__main__":
    drone = MuellerDrone3()
    attrs = vars(drone)
    print(json.dumps(attrs, indent=4))
