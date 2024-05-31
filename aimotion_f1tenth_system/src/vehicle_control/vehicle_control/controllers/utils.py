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
    