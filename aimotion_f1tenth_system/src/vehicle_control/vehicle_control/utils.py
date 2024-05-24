from abc import ABC, abstractmethod
from enum import Enum
from .controllers.ModularGPLPVLQR import ModularGPLPVLQR
from .controllers.utils import Controller, normalize, clamp
import numpy as np
from scipy.interpolate import splev
from dataclasses import dataclass


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


class CONTROLLER_MODE(Enum):
    RUNNING = 1
    IDLE = 2
    STOP = 3
    PROCESSING = 4
    MANUAL = 5


def init_GP_LPV_LQR(vehicle_params: dict, GP_LPV_LQR_params: dict) -> Controller:
    LPV_gains = {
        "lat_gains": GP_LPV_LQR_params["lat_gains"],
        "long_gains": GP_LPV_LQR_params["long_gains"]
    }
    GP_params = {
        "GP_type": GP_LPV_LQR_params["GP_type"],
        "num_of_inducing": GP_LPV_LQR_params["num_of_inducing"],
        "forgetting_factor": GP_LPV_LQR_params["forgetting_factor"],
        "confidence_level": GP_LPV_LQR_params["confidence_level"],
        "batch_size": GP_LPV_LQR_params["batch_size"],
        "retrain_iter": GP_LPV_LQR_params["retrain_iter"]
    }
    controller = ModularGPLPVLQR(model = vehicle_params,
                                 LPV_gains = LPV_gains,
                                 GP_params = GP_params,
                                 control_step = 1 / GP_LPV_LQR_params["frequency"])
    
    return controller
    


class Trajectory(TrajectoryBase):
    def __init__(self) -> None:
        self.pos_tck = None
        self.evol_tck = None
        self.reversed = None
        self.length = None
        self.t_end = None
        self.output = {}

    def set_trajectory(self, pos_tck, evol_tck, reversed):
        self.pos_tck = pos_tck
        self.evol_tck = evol_tck
        self.reversed = reversed
        self.length = pos_tck[0][-1]
        self.t_end = evol_tck[0][-1]

    def evaluate(self, state, t):
        """Evaluates the trajectory based on the vehicle state & time
        
        :param state: Vehicle state
        :type state: dict
        :param i: Iterator valiable only used by the simulator
        :type i: int
        :param time: Current time
        :type time: float
        :param control_step: he step of the controller

        """

        if self.pos_tck is None or self.evol_tck is None: # check if data has already been provided
            raise ValueError("Trajectory must be defined before evaluation")
        
        pos = np.array([state[0], state[1]])

        # get the path parameter (position along the path)
        s_ref = splev(t, self.evol_tck) # estimate the path parameter based on the time
        
        try:
            s = self._project_to_closest(pos = pos, param_estimate = s_ref, projetion_window = 5, projection_step = 0.005) # the projection parameters cound be refined/not hardcoded
        except: # projection might fail at the end of the trejectory
            self.output["running"] = False
            return self.output
        
        # check if the retrievd is satisfies the boundary constraints & the path is not completed
        if t >= self.t_end + 30 or (self.length - s) < 0.05:
            self.output["running"] = False # stop the controller
            return self.output
        else:
            self.output["running"] = True # the goal has not been reached, evaluate the trajectory

        # get path data at the parameter s
        (x, y) = splev(s, self.pos_tck)
        (x_, y_) = splev(s, self.pos_tck, der=1)
        (x__,y__) = splev(s, self.pos_tck,der=2)
        
        # calculate base vectors of the moving coordinate frame
        s0 = np.array(
            [x_ / np.sqrt(x_**2 + y_**2), y_ / np.sqrt(x_**2 + y_**2)]
        )
        z0 = np.array(
            [-y_ / np.sqrt(x_**2 + y_**2), x_ / np.sqrt(x_**2 + y_**2)]
        )

        # calculate path curvature
        c = -(x__ * y_ - x_ * y__) / ((x_**2 + y_**2)**(3/2))


        # get speed reference
        v_ref = splev(t, self.evol_tck, der = 1)

        self.output["ref_pos"] = np.array([x,y])
        self.output["s0"] = s0
        self.output["z0"] = z0
        self.output["c"] = c

        self.output["s"] = s
        self.output["s_ref"] = s_ref
        self.output["v_ref"] = v_ref
        self.output["reversed"] = self.reversed

        return self.output

    def reset_trajectory(self):
        self.pos_tck = None
        self.evol_tck = None
        self.reversed = None
        self.length = None
        self.t_end = None

    def _project_to_closest(self, pos: np.ndarray, param_estimate: float, projetion_window: float, projection_step: float) -> float:
        """Projects the vehicle position onto the ginven path and returns the path parameter.
           The path parameter is the curvilinear abscissa along the path as the Bspline that represents the path is arc length parameterized

           
        :param pos: Vehicle x,y position
        :type pos: np.ndarray
        :param param_estimate: Estimated value of the path parameter
        :type param_estimate: float
        :param projetion_window: Length of the projection window along the path
        :type projetion_window: float
        :param projection_step: Precision of the projection in the window
        :type projection_step: float
        :return: The path parameter
        :rtype: float
        
        """

        
        # calulate the endpoints of the projection window
        floored = clamp(param_estimate - projetion_window / 2, [0, self.length])
        ceiled = clamp(param_estimate + projetion_window/2, [0, self.length])

        # create a grid on the window with the given precision
        window = np.linspace(floored, ceiled, round((ceiled - floored) / projection_step))

        # evaluate the path at each instance
        path_points = np.array(splev(window, self.pos_tck)).T

        # find & return the closest points
        deltas = path_points - pos
        indx = np.argmin(np.einsum("ij,ij->i", deltas, deltas))
        return floored + indx * projection_step
        
        #closest_index = np.argmin(np.linalg.norm(deltas))
        #return floored + closest_index * projection_step

class StateLogger:
    def __init__(self) -> None:
        self.data: list[LogState] = []


    def log_state(self,t:float, state: np.ndarray, setpoint: dict, errors, u: np.ndarray):
        self.data.append(LogState(t=t,
                                  x=state[0],
                                  y=state[1],
                                  phi=state[2],
                                  v_xi=state[3],
                                  v_eta=state[4],
                                  omega=state[5],
                                  c=setpoint["c"],
                                  s_err=errors[2],
                                  v_err=errors[3],
                                  e=errors[0],
                                  theta_e=errors[1],
                                  d=u[0],
                                  delta=u[1]))

        
    def reset(self):
        self.data = []

    def get_states(self):
        """Get the elogged states as a numpy array"""
        state_arr = np.array([[state.x, 
                               state.y,
                               state.phi,
                               state.v_xi,
                               state.v_eta,
                               state.omega] for state in self.data])
        
        return state_arr

    def get_errors(self):
        """Get the logged errors as a numpy array"""
        error_arr = np.array([[state.e,
                               state.theta_e,
                               state.s_err,
                               state.v_err] for state in self.data])
        
        return error_arr

    def get_inputs(self):
        """Get the logged inputs as a numpy array"""
        input_arr = np.array([[state.d, state.delta] for state in self.data])
        return input_arr
    
    def get_c(self):
        """Get the logged curvature as a numpy array"""
        c_arr = np.array([state.c for state in self.data])
        return c_arr
    
    def get_latest_inputs(self):
        """Get the latest logged inputs as a numpy array"""
        input_arr_latest = np.array([self.data[-1].d, self.data[-1].delta])
        return input_arr_latest



@dataclass
class LogState:
    t: float
    x: float
    y: float
    phi: float
    v_xi: float
    v_eta: float
    omega: float
    c: float
    s_err: float
    v_err: float
    e: float
    theta_e: float
    d: float
    delta: float





def state2array(state) -> np.ndarray:
    """
    Converts the state dictionary into a numpy array

    Arguments:
        - state(dict): The state dictionary

    Returns:
        - np.ndarray: The state array
    """

    return np.array([state.position_x,
                     state.position_y,
                     normalize(state.heading_angle),
                     state.velocity_x,
                     state.velocity_y, 
                     state.omega])