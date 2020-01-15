from Controllers.ControllerABC import PlatoonControllerABC
import control
from CONSTANTS import *
import scipy.linalg as spl


class PlatoonController(PlatoonControllerABC):
    def __init__(
            self,
            steering_angle=0,
            velocity=0,
            communicate_to_arduino=True,
            zero_vel_offset=0.5,
            vel_distance_coeff=0.1):
        super().__init__(
            steering_angle=steering_angle,
            velocity=velocity,
            communicate_to_arduino=communicate_to_arduino,
            zero_vel_offset=zero_vel_offset,
            vel_distance_coeff=vel_distance_coeff)
        self.A = np.array([[1, 0, 0],
                           [1, 0, -1],
                           [0, 0, 1]])
        self.B = np.array([[0], [0], [1]])
        self.C = np.eye(3)
        self.D = 0
        self.sampling_time = DEFAULT_IP
        self.state_space = control.StateSpace(self.A, self.B, self.C, self.D)

        dummy = 1
        self.state_cost = np.array([dummy, 0, dummy,
                                    0, dummy, dummy,
                                    dummy, dummy, dummy])
        self.input_cost_one_step = dummy
        self.num_backward_step = 1
        self.input_cost = np.kron(self.input_cost_one_step, np.eye(self.num_backward_step))

        self.cost = spl.block_diag(self.state_cost, self.input_cost)

        self.prediction_horizon = 4

        #TODO compute optimal inputs and store them
        self.optimal_inputs_table = None
        self.current_optimal_velocity= 0
        self.current_optimal_steering_angle = 0

    @property
    def state(self):
        return np.array([self.leader_velocity, self.distance_error, self.ego_velocity, self.current_optimal_velocity])

    def set_velocity(self):
        self.ego_velocity = self.current_optimal_velocity

    def set_steering_angle(self):
        self.ego_steering_angle = self.current_optimal_steering_angle

    def compute_optimal_input(self, look_up = True):
        """
        Compute the optimal input at the given state. If lookup is true get the optimal input
        from the stored lookup table otherwise compute from scratch
        :return: set optimal input variable
        """
        self.current_optimal_velocity = 0
        self.current_optimal_steering_angle = 0

    def _finite_horizon_optimal_input(self):
        #TODO
        pass

    def compute_mpt(self, file_to_save):
        """
        Compute the entire control law and store the table in file to save
        :param file_to_save:
        :return:
        """
        pass

    def load_control_table(self, file_to_read):
        """
        Load the lookup table from file to read
        :param file_to_read:
        :return:
        """
        pass