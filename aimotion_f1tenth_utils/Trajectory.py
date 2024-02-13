import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev, splrep
from scipy.integrate import quad
from typing import Union
import matplotlib as mpl

mpl.rcParams["text.usetex"] = True


class Trajectory:
    def __init__(self, trajectory_ID) -> None:
        """Class implementation of BSpline-based trajectories for autonomous ground vehicles
        """
        self.trajectory_ID = trajectory_ID 
        self.output = {}
        self.pos_tck = None
        self.evol_tck = None
        self.t_end = None
        self.length = None
        self.reversed = None

    def build_from_points_const_speed(self, path_points: np.ndarray, path_smoothing: float, path_degree: int, const_speed: float):
        """Object responsible for storing the reference trajectory data.

        Args:
            path_points (numpy.ndarray): Reference points of the trajectory
            smoothing (float): Smoothing factor used for the spline interpolation
            degree (int): Degree of the fitted Spline
        """
        if const_speed < 0:
            const_speed = abs(const_speed)
            self.reversed = True
        else:
            self.reversed = False

        x_points = path_points[:, 0].tolist()
        y_points = path_points[:, 1].tolist()

        # fit spline and evaluate
        tck, u, *_ = splprep([x_points, y_points], k=path_degree, s=path_smoothing)
        XY=splev(u, tck)

        # calculate arc length
        def integrand(x):
            dx, dy = splev(x, tck, der=1)
            return np.sqrt(dx**2 + dy**2)
        self.length, _ = quad(integrand, u[0], u[-1])

        # build spline for the path parameter
        self.pos_tck, _, *_ = splprep(XY, k=path_degree, s=0, u=np.linspace(0, self.length, len(XY[0])))

        # build constant speed profile
        self.t_end= self.length / const_speed

        t_evol = np.linspace(0, self.t_end, 101)
        s_evol = np.linspace(0, self.length, 101)

        self.evol_tck = splrep(t_evol,s_evol, k=1, s=0) # NOTE: This is completely overkill for constant veloctities but can be easily adjusted for more complex speed profiles


    def export_to_time_dependent(self):
        """Exports the trajectory to a time dependent representation
        """
        t_eval=np.linspace(0, self.t_end, 100)
        
        s=splev(t_eval, self.evol_tck)
        (x,y)=splev(s, self.pos_tck)

        tck, t, *_ = splprep([x, y], k=5, s=0, u=t_eval)

        return tck
        

    def to_send(self):
        return self.pos_tck, self.evol_tck

    def _project_to_closest(self, pos: np.ndarray, param_estimate: float, projetion_window: float, projection_step: float) -> float:
        """Projects the vehicle position onto the ginven path and returns the path parameter.
           The path parameter is the curvilinear abscissa along the path as the Bspline that represents the path is arc length parameterized

        Args:
            pos (np.ndarray): Vehicle x,y position
            param_esimate: Estimated value of the path parameter
            projetion_window (float): Length of the projection window along the path
            projection_step (float): Precision of the projection in the window

        Returns:
            float: The path parameter
        """

        
        # calulate the endpoints of the projection window
        floored = self._clamp(param_estimate - projetion_window / 2, self.s_bounds)
        ceiled = self._clamp(param_estimate + projetion_window/2, self.s_bounds)


        # create a grid on the window with the given precision
        window = np.linspace(floored, ceiled, round((ceiled - floored) / projection_step))


        # evaluate the path at each instance
        path_points = np.array(splev(window, self.pos_tck)).T

        # find & return the closest points
        deltas = path_points - pos
        indx = np.argmin(np.einsum("ij,ij->i", deltas, deltas))
        return floored + indx * projection_step
    



    def evaluate(self, state, i, time, control_step) -> dict:
        """Evaluates the trajectory based on the vehicle state & time"""

        if self.pos_tck is None or self.evol_tck is None: # check if data has already been provided
            raise ValueError("Trajectory must be defined before evaluation")
        
        pos=np.array([state["pos_x"],state["pos_y"]])

        # get the path parameter (position along the path)
        s_est=self.s+float(state["long_vel"])*control_step
        self.s=self._project_to_closest(pos=pos, param_estimate=s_est, projetion_window=0.5, projection_step=0.005) # the projection parameters cound be refined/not hardcoded

        # check if the retrievd is satisfies the boundary constraints & the path is not completed
        if time>=self.t_end: # substraction is required because of the projection step
            self.output["running"]=False # stop the controller
        else:
            self.output["running"]= True # the goal has not been reached, evaluate the trajectory

        # get path data at the parameter s
        (x, y) = splev(self.s, self.pos_tck)
        (x_, y_) = splev(self.s, self.pos_tck, der=1)
        (x__,y__) = splev(self.s, self.pos_tck,der=2)
        
        # calculate base vectors of the moving coordinate frame
        s0 = np.array(
            [x_ / np.sqrt(x_**2 + y_**2), y_ / np.sqrt(x_**2 + y_**2)]
        )
        z0 = np.array(
            [-y_ / np.sqrt(x_**2 + y_**2), x_ / np.sqrt(x_**2 + y_**2)]
        )

        # calculate path curvature
        c=abs(x__*y_-x_*y__)/((x_**2+y_**2)**(3/2))

        # get speed reference
        self.s_ref = splev(time, self.evol_tck)
        v_ref = splev(time, self.evol_tck, der=1)

        self.output["ref_pos"]=np.array([x,y])
        self.output["s0"] = s0
        self.output["z0"] = z0
        self.output["c"] = c

        self.output["s"] = self.s
        self.output["s_ref"]=self.s_ref # might be more effective if output["s"] & self.s are combined
        self.output["v_ref"] = v_ref

        return self.output
    


    @staticmethod
    def _clamp(value: Union[float,int], bound: Union[int,float,list,tuple,np.ndarray]) -> float:
        """Helper function that clamps the given value with the specified bounds

        Args:
            value (float | int): The value to clamp
            bound (list | tuple | np.ndarray): If int | float the function constrains the value into [-bound,bound]
                                               If tuple| list | np.ndarray the value is constained into the range of [bound[0],bound[1]]

        Returns:
            float: The clamped value
        """
        if isinstance(bound, int) or isinstance(bound, float):
            if value < -bound:
                return float(-bound)
            elif value > bound:
                return float(bound)
            return float(value)
        elif isinstance(bound, tuple) or isinstance(bound,list) or isinstance(bound, np.ndarray):
            if value < bound[0]:
                return float(bound[0])
            elif value > bound[1]:
                return float(bound[1])
            return float(value)
    

    @staticmethod
    def _normalize(angle: float) -> float:
        """Normalizes the given angle into the [-pi/2, pi/2] range

        Args:
            angle (float): Input angle

        Returns:
            float: Normalized angle
        """
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi

        return angle
    
    def plot_trajectory(self, block=True) -> None:
        """ Plots, the defined path of the trajectory in the X-Y plane. Nota, that this function interrupts the main thread and the simulator!
        """

        if self.pos_tck is None or self.evol_tck is None: # check if data has already been provided
            raise ValueError("No Spline trajectory is specified!")
        
        # evaluate the path between the bounds and plot
        t_eval=np.linspace(0, self.t_end, 100)
        
        s=splev(t_eval, self.evol_tck)
        (x,y)=splev(s, self.pos_tck)
        
        fig, axs = plt.subplot_mosaic("AB;CC;DD")

        axs["A"].plot(t_eval,x)
        axs["A"].set_xlabel("t [s]")
        axs["A"].set_ylabel("x [m]")
        axs["A"].set_title("X coordinate")

        axs["B"].plot(t_eval,y)
        axs["B"].set_xlabel("t [s]")
        axs["B"].set_ylabel("y [m]")
        axs["B"].set_title("Y coordinate")

        axs["C"].plot(x,y)
        axs["C"].set_xlabel("x [m]")
        axs["C"].set_ylabel("y [m]")
        axs["C"].set_title("X-Y trajectory")
        axs["C"].axis("equal")

        axs["D"].plot(t_eval, s)
        axs["D"].set_xlabel("t [s]")
        axs["D"].set_ylabel("s [m]")
        axs["D"].set_title("Path parameter")

        plt.tight_layout()
        plt.show(block=block)




if __name__ == "__main__":
    """Simple test for the trajectory class
    """
    traj=Trajectory()

    theta=np.linspace(0,2*np.pi,100)
    path_points = np.array([5*np.cos(theta), 5*np.sin(theta)]).T

    traj.build_from_points_const_speed(path_points=path_points, path_smoothing=0.01, path_degree=5, const_speed=1)
    traj.plot_trajectory()