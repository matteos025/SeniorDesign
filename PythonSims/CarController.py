from abc import ABC, abstractmethod
import numpy as np

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
    def __int__(self):
        # TODO give this equations to d
        pass

    def get_action(self, system_state):
        """ TODO platoon action given system state"""
        pass

    def vel_calc(
            self,
            my_vel,
            back_vel,
            front_vel,
            my_acc,
            back_acc,
            front_acc):
        """ TODO dynamics function """
        return my_vel * back_vel - front_vel


class KeyBoardController(Controller):
    def __init__(self):
        self.cur_action = '0'

    def get_action(self, system_state):
        if self.cur_action == 'left':
            return (0, system_state[6] + 0.1)
        elif self.cur_action == 'right':
            return (0, system_state[6] - 0.1)
        elif self.cur_action == 'forward':
            return (system_state[4, 5]+0.1, system_state[6])
        elif self.cur_action == 'backward':
            return (system_state[4, 5]-0.1, system_state[6])
        else:
            return (np.array([0,0]), system_state[6])
