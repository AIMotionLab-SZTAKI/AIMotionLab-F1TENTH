from enum import Enum

class ControllerState(Enum):
    NOMINAL = 0
    ADAPTIVE = 1


class GPState:
    OFFLINE = 0
    ONLINE = 1


class LoggerState(Enum):
    ON = 1
    OFF =0

class Logger:
    def __init__(self, root) -> None:
        self.root = root
        self.file = open(self.root + "temp_log.csv", 'w')


    def log_state(self, state):
        pass

    def retrieve_states(self):
        pass

    def retrieve_time(self):
        pass

    def retrieve_inputs(self):
        pass

    def retrieve_curvature(self):
        pass
