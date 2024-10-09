from abc import ABC, abstractmethod
import numpy as np
class Controller(ABC):
    @abstractmethod
    def compute_control(self, state: np.ndarray, setpoint: dict, t:float) -> np.ndarray:
        pass

    @abstractmethod
    def set_trajectory(self, *args, **kwargs):
        pass

    @abstractmethod
    def train_GP_controllers(self, *args, **kwargs):
        pass
    
    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def set_parameters(self, parameters: dict):
        """
        Set controller parameters. 
        Args:
            parameters(dict): Parameters
        """
        pass


class Base_MPCC_Controller(ABC):

    @abstractmethod
    def compute_control(self, state: np.ndarray, setpoint: dict, t:float) -> np.ndarray:
        pass

    @abstractmethod
    def set_trajectory(self, *args, **kwargs):
        pass

    @abstractmethod
    def train_GP_controllers(self, *args, **kwargs):
        pass
    
    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def set_parameters(self, parameters: dict):
        """
        Set controller parameters. 
        Args:
            parameters(dict): Parameters
        """
        pass

    @abstractmethod
    def set_trajectory(self, pos_tck, evol_tck, generate_solver = True):
        """
        Evaluete the reference spline from the given spline tck, and convert it into a Spline2D instance.
        Args:
            pos_tck(np.array): position spline
            evol_tck(np.array): reference progress spline
            generate_solver(bool): generate ocp solver
        """
        pass

    @abstractmethod
    def init_controller(self, x0):
        """
        Initialize the Acados SQP solver by solving the optimization problem with Casadi IPOPT
        Args:
            x0(np.array): Initial position of CoM
        """

        pass

    @abstractmethod
    def generate_solver(self, pos_tck = None, evol_tck = None, x0 = None):
        """Create ocp solver
        Args:
            pos_tck(np.array): *optional* position spline
            evol_tck(np.array): *optional* reference progress spline
            x0(np.array): *optinal* starting state
        """
        pass


def normalize(angle):
    """
    Normalizes the given angle into the [-pi/2, pi/2] range

    Arguments:
        - angle(float): The angle to normalize, in radian
    """
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi

    return angle

def clamp(value, bound):
    if isinstance(bound, int) or isinstance(bound, float):
        if value < -bound:
            return -bound
        elif value > bound:
            return bound
        return value
    elif isinstance(bound, tuple) or isinstance(bound, list) or isinstance(bound, np.ndarray):
        if value < bound[0]:
            return bound[0]
        elif value > bound[1]:
            return bound[1]
        return value
    