from abc import ABC, abstractmethod
import numpy as np
import numpy.linalg as npl

class Controller(ABC):
    @abstractmethod
    def get_action(self, system_state):
        """
        Controllers must produce actions
        :param system_state: state of system which controller is controlling
        :return: optimal action
        """
        pass

class PlatoonController(Controller):
    def __init__(self, desired_speed, desired_distance, next_car = None):
        # TODO give this equations to d
        self.desired_speed = desired_speed
        self.desired_distance = desired_distance
        self.next_car = next_car

    def get_action(self, system_state):
        """ TODO platoon action given system state"""
        if self.next_car is None:
            return system_state['a'], system_state['steering_angle']

        next_pos = self.next_car.pos
        ego_pos = system_state["pos"]
        ego_acc = system_state['a']

        true_distance = np.abs(next_pos-ego_pos)
        dspeed = self.next_car.speed - npl.norm(system_state['v'])
        innov_pos = true_distance - self.desired_distance
        innov_speed = dspeed - self.desired_speed

        unit_v = system_state['v']/npl.norm(system_state['v']+1e-17)
        Kp = 0.05
        Kv = 0

        new_a = ego_acc + (Kp*innov_pos + Kv*innov_speed)*unit_v

        steering_angle = system_state['steering_angle']
        return new_a, steering_angle





class KeyBoardController(Controller):
    def __init__(self):
        self.cur_action = '0'

    def get_action(self, system_state):
        return system_state['a'], system_state['steering_angle']
        if self.cur_action == 'left':
            return (0, system_state[6] + 0.1)
        elif self.cur_action == 'right':
            return (0, system_state[6] - 0.1)
        elif self.cur_action == 'forward':
            return (system_state[4:6]+0.1, system_state[6])
        elif self.cur_action == 'backward':
            return (system_state[4:6]-0.1, system_state[6])
        else:
            return (np.array([0,0]), system_state[6])


from abc import ABC, abstractmethod
import numpy as np
import numpy.linalg as npl

class Controller(ABC):
    @abstractmethod
    def get_action(self, system_state):
        """
        Controllers must produce actions
        :param system_state: state of system which controller is controlling
        :return: optimal action
        """
        pass

class CircularPlatoonController(Controller):
    def __init__(self, desired_speed, desired_distance, center, radius, next_car = None):
        # TODO give this equations to d
        self.desired_speed = desired_speed
        self.desired_distance = desired_distance
        self.center = center
        self.radius = radius
        self.next_car = next_car

    def get_action(self, system_state):
        """ TODO platoon action given system state"""
        if self.next_car is None:
            return system_state['a'], system_state['steering_angle']

        next_pos = self.next_car.pos
        ego_pos = system_state["pos"]
        ego_acc = system_state['a']
        line_dist = npl.norm(next_pos-ego_pos)

        angle = np.arccos(1-line_dist**2/(2*self.radius**2))
        dist = self.radius*angle

        innov_pos = dist - self.desired_distance

        unit_v = system_state['v']/npl.norm(system_state['v']+1e-17)
        Kp = 0.25

        new_a = ego_acc + (Kp*innov_pos )*np.array(np.cos(system_state['heading']), np.sin(system_state['heading']))

        steering_angle = system_state['steering_angle']
        return new_a, steering_angle


