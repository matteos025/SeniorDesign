from MuellerReplication.DroneObj import MuellerDrone
import numpy as np
import numpy.linalg as npl


def p_dot(drone, q, r):
    term1 = drone.kappa_f * (np.square(drone.omega2) -
                             np.square(drone.omega4)) * drone.l
    term2 = - (drone.IzzT - drone.IxxT) * q * r
    term3 = - drone.IzzP * q * \
            (drone.omega1 + drone.omega2 + drone.omega3 + drone.omega4)
    return term1 + term2 + term3

def solve_q(drone, r):
    term1 = drone.kappa_f * (np.square(drone.omega2) -
                             np.square(drone.omega4)) * drone.l
    term2 = (drone.IzzT - drone.IxxT) * r
    term3 =  drone.IzzP * \
            (drone.omega1 + drone.omega2 + drone.omega3 + drone.omega4)
    return term1 / (term2 + term3)

def solve_r(drone):
    return (drone.kappa_f * drone.kappa_tau * \
            (drone.omega1**2 - drone.omega2**2 + drone.omega3**2 - drone.omega4**2)) / drone.gamma


drone = MuellerDrone()
rho = 0.5

drone.omega1 = np.sqrt(2.05/drone.kappa_f)
drone.omega2 = np.sqrt(1.02/drone.kappa_f)# np.sqrt(rho)*drone.omega1 #
drone.omega3 = drone.omega1
drone.omega4 = 0

drone.r = solve_r(drone)
drone.q = solve_q(drone,drone.r)
drone.p = 0
nbar = drone.omegaB/npl.norm(drone.omegaB)

print("SOLUTION STARTING WITH GIVEN FORCE VALUES")
print("f1 = {}".format(drone.f1))
print("f2 = {}".format(drone.f2))
print("f3 = {}".format(drone.f3))
print("f4 = {}".format(drone.f4))

print("w1 = {}".format(drone.omega1))
print("w2 = {}".format(drone.omega2))
print("w3 = {}".format(drone.omega3))
print("w4 = {}".format(drone.omega4))

print("p_dot = {}".format(drone.p_dot))
print("q_dot = {}".format(drone.q_dot))
print("r_dot = {}".format(drone.r_dot))

print("omegaB = {}".format(drone.omegaB))
print("n_bar = {}".format(nbar))
print("Difference with Gravity = {}".format(nbar[2]*(drone.f1+drone.f2+drone.f3+drone.f4)-drone.mass*9.8))
print("nz bar sol = {}".format(drone.mass*9.8/(drone.f1+drone.f2+drone.f3+drone.f4)))

# drone.r = 18.89
# drone.q = 5.69
# drone.p = 0
#
# print("SOLUTIONS FROM MUELLER PAPER")
# print("f1 = {}".format(drone.f1))
# print("f2 = {}".format(drone.f2))
# print("f3 = {}".format(drone.f3))
# print("f4 = {}".format(drone.f4))
#
# print("p_dot = {}".format(drone.p_dot))
# print("q_dot = {}".format(drone.q_dot))
# print("r_dot = {}".format(drone.r_dot))
#
# print("omegaB = {}".format(drone.omegaB))
# print("n_bar = {}".format(drone.omegaB/npl.norm(drone.omegaB)))




