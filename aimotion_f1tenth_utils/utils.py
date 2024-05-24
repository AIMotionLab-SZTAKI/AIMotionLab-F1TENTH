from enum import Enum
from abc import ABC, abstractmethod
import numpy as np

class CONTROLLER_MODE(Enum):
    RUNNING = 1
    IDLE = 2
    STOP = 3
    PROCESSING = 4
    MANUAL = 5


class TrajectoryBase(ABC):
    @abstractmethod
    def evaluate(self, state: np.ndarray, t: float) -> dict:
        pass

    @abstractmethod
    def reset_trajectory(self):
        pass

    @abstractmethod
    def set_trajectory(self, pos_tck: tuple, evol_tck: tuple, reversed: bool):
        pass


