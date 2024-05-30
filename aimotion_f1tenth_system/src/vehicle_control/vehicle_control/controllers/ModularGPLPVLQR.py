import numpy as np
import pynumdiff
import matplotlib.pyplot as plt
import warnings

from .utils import clamp, normalize, Controller
from ..GP.SGPModel import SGPModel
from ..GP.GRADSGPModel import GRADSGPModel
from ..GP.RLSSGPModel import RLSSGPModel
from ..GP.gp_base import GP



class ModularGPLPVLQR(Controller):
    def __init__(self, model, LPV_gains: dict, GP_params, control_step):
        """Modular GP-LPV-LQR controller for trajectory-tracking with autonomous ground vehicles
        
        :param model: vehicle model parameters
        :type model: dict
        :param LPV_gains: LPV gains for the controller stored in a predefined dictionary format
        :type LPV_gains: dict
        :param GP_params: GP model parameters stored in a predefined dictionary format
        :type GP_params: dict
        :param control_step: control step size
        :type control_step: float

        """
        # init the parent class
        super().__init__()
        
        # load model
        self.model = model

        # initialize the LPV gains
        # longitudinal
        self._init_LQ_controllers(LPV_gains)

        # initialize GP parameters
        self._init_GP_models(GP_params)

        # control step size
        self.dt = control_step

        self.q = 0




    def _init_LQ_controllers(self, LPV_gains):
        """Initialize the LPV-LQR controllers for lateral and longitudinal control augmentation
        
        :param LPV_gains: LPV gains for the controller stored in a predefined dictionary format
        :type LPV_gains: dict
        """
        
        # feedback gain polynomials
        self.k_lat1=np.poly1d(LPV_gains["lat_gains"]["k1"])
        self.k_lat2=np.poly1d(LPV_gains["lat_gains"]["k2"])
        self.k_lat3=np.poly1d(LPV_gains["lat_gains"]["k3"])
        
        self.k_lat1_r=np.poly1d(LPV_gains["lat_gains"]["k1_r"])
        self.k_lat2_r=np.poly1d(LPV_gains["lat_gains"]["k2_r"])


        self.k_long1=np.poly1d(LPV_gains["long_gains"]["k1"])
        self.k_long2=np.poly1d(LPV_gains["long_gains"]["k2"])
        
        


    def _init_GP_models(self, GP_params):
        """Initialize the GP models for lateral and longitudinal control augmentation
        
        :param GP_params: GP model parameters stored in a predefined dictionary format
        :type GP_params: dict
        """

        self.GP_type = GP_params["GP_type"]
        self.mode = "offline"
        self.num_of_inducing_lat = GP_params["num_of_inducing"]
        self.num_of_inducing_long = GP_params["num_of_inducing"]
        
        # GRAD_SGP parameters
        self.batch_size = GP_params["batch_size"]
        self.retrain_iter = GP_params["retrain_iter"]
        
        # RLS parameters
        self.forgetting_factor = GP_params["forgetting_factor"]
        self.confidence_level = GP_params["confidence_level"]


        #if self.GP_type == "SGP":
            # none of the above attributes are needes
        #    self.batch_size = None
        #    self.retrain_iter = None
        #    self.forgetting_factor = None
        #    self.confidence_level = None

        #elif self.GP_type == "GRAD_SGP":
        #    self.batch_size = GP_params["batch_size"]
        #    self.retrain_iter = GP_params["retrain_iter"]
        #    self.forgetting_factor = None
        #    self.confidence_level = None

        #elif self.GP_type == "RLS_SGP":
        #    self.batch_size = GP_params["batch_size"]
        #    self.retrain_iter = None
        #    self.forgetting_factor = GP_params["forgetting_factor"]
        #    self.confidence_level = GP_params["confidence_level"]

        self.GP_long = None
        self.GP_lat = None

        self.long_var = None
        self.lat_var = None



    def reset(self, GP_reset=False):
        """Reset the controller
        
        :param GP: If True, the GP models are reset. Defaults to False.
        :type GP: bool, optional
        """
        self.q = 0

        if GP_reset:
            self.GP_long: GP = None
            self.GP_lat: GP = None



    def compute_control(self, state, setpoint)->np.ndarray:
        """Method for calculating the control input, based on the current state and setpoints

        :param state: current state of the vehicle
        :type state: dict
        :param setpoint: current setpoint
        :type setpoint: dict
        :return: control input
        :rtype: np.ndarray
        """
        # retrieve setpoint & state data
        s0=setpoint["s0"]
        z0=setpoint["z0"]
        ref_pos=setpoint["ref_pos"]
        c=setpoint["c"]
        s=setpoint["s"]
        s_ref=setpoint["s_ref"]
        v_ref=setpoint["v_ref"]

        pos = state[:2]
        phi = state[2]
        v_xi =state[3]
        v_eta =state[4]
        omega =state[5]

        theta_p = np.arctan2(s0[1], s0[0])

        # lateral error
        z1=np.dot(pos-ref_pos, z0)

        # for reversing motion invert the heading
        if setpoint["reversed"]:
            phi+=np.pi
            v_ref = -v_ref

        # heading error
        theta_e = normalize(phi-theta_p)
        
        # invert z1 for lateral dynamics:
        e =- z1
        self.q += e
        self.q = clamp(self.q,.1)

        
        beta=np.arctan2(v_eta,abs(v_xi)) # abs() needed for reversing 
        p=abs(np.cos(theta_e+beta)/np.cos(beta)/(1-c*z1))


        ## estimate error derivative
        try:
            self.edot=0.3*((e-self.ep)/self.dt-self.edot)+self.edot # calculate \dot{e} by finite difference
            self.ep=e                                               # 0.5 coeff if used for smoothing
        except AttributeError: # if no previous error value exist assume 0 & store the current value
            self.edot=0
            self.ep=e


        # compute the nominal control inputs: feedback and feedforward
        if v_ref>0:
            delta = -theta_e + self.k_lat1(v_xi) * self.q + self.k_lat2(v_xi) * e + self.k_lat3(v_xi) * self.edot \
                   - self.model["m"]/self.model["C_f"]*((self.model["l_r"]*self.model["C_r"]-self.model["l_f"]*self.model["C_f"])/self.model["m"]-1)*c

            # compute control inputs
            if self.GP_lat is not None: # GP augmentation control, offline mode
            
                lat_mean, self.lat_var = self.GP_lat.predict(np.array([[v_xi, v_eta, omega]]))
                delta -= self.model["m"]/self.model["C_f"]*lat_mean.item()
                #print(self.model["m"]/self.model["C_f"]*np.asscalar(self.GP_lat.eval_gp(np.array([[v_xi, v_eta, omega]]), "numpy")[0]), self.GP_lat.eval_gp(np.array([[v_xi, v_eta, omega]]), "numpy")[1])
        
        else:
            delta=1.5*self.k_lat1_r(v_xi)*z1+self.k_lat2*theta_e#+0.33/((abs(v_xi)+0.01)*c*np.cos(theta_e)/(1-c*z1)) 
            

        # clamp inputs
        delta=clamp(delta, (-.5,.5))

        #### Longitudinal control input ####    
        d=(self.model["C_m2"]*v_ref+self.model["C_m3"]*np.sign(v_ref))/self.model["C_m1"]-self.k_long1(p)*(s-s_ref)-self.k_long2(p)*(v_xi-v_ref)


        if self.GP_long is not None and v_ref > 0: # GP augmentation control, ONLY FORWARD MOTION
            long_mean, self.long_var = self.GP_long.predict(np.array([[v_xi, v_eta, omega]]))
            d-= self.model["m"]/self.model["C_m1"]/(1+np.cos(delta))*long_mean.item()


        # clamp control inputs into the feasible range
        d=clamp(d,(-0.2, 0.25)) # currently only forward motion, TODO: reversing control

        self.u=np.array([d, delta])


        if self.mode == "online":
            self.recursive_GP_update(state, self.u, c)


        # save the errors
        self.errors=np.array([z1, theta_e, s-s_ref, v_xi-v_ref, self.q])
        self.prev_state = state

        return self.u, self.errors

    def _train_SGP_long(self, states, inputs, plot_training_data=False):
        """Train the sparse GP model for longitudinal control augmentation
        
        :param states: Array containing the state variables
        :type states: np.ndarray
        :param inputs: Array containing the control inputs
        :type inputs: np.ndarray
        :param plot_training_data: If True, the training data is plotted. Defaults to False.
        :type plot_training_data: bool, optional
        """
        train_x=np.vstack((states[:,3],states[:,4],states[:,5])) # v_xi, v_eta, omega

        _, d_v_xi_measured=pynumdiff.finite_difference.first_order(states[:,3], self.dt, [50], options={'iterate': True})
        #d_v_xi_measured = np.diff(states[:,3]) / self.dt
        d_v_xi_nominal= - self.model["C_m2"]*(1+np.cos(inputs[:,1]))/self.model["m"]*states[:,3] \
                        + self.model["C_m1"]*(1+np.cos(inputs[:,1]))/self.model["m"]*inputs[:,0] \
                        - self.model["C_m3"]*(1+np.cos(inputs[:,1]))/self.model["m"]*np.sign(states[:,3])
        
        train_y=d_v_xi_measured-d_v_xi_nominal
        
        train_x=train_x.T
        start=10 # skip the first 10 samples
        end=train_x.shape[0]-10
        #step_size = max(1, (end-start) // 20)
        
        train_x=train_x[start:end,:]
        #train_x_init=train_x[::step_size,:]

        train_y=train_y[start:end]

        #step_size = max(1, (end-start) // 5000)
        #train_x=train_x[::step_size,:]
        #train_y=train_y[::step_size]#+0.01*np.random.randn(train_y[::step_size].shape[0])


        if plot_training_data:
            
            fig, axs = plt.subplots(4, 1, figsize=(8, 10))  # 4 rows, 1 column

            # Plot data on each subplot
            axs[0].plot(states[:,3])
            axs[0].set_title(r'$$v_\xi$$')

            axs[1].plot(states[:,4])
            axs[1].set_title(r'$$v_\eta$$')

            axs[2].plot(states[:,5])
            axs[2].set_title(r'$$\omega$$')

            axs[3].plot(train_y)
            axs[3].set_title(r'$$\mathrm{train}_y$$')

            plt.suptitle("Longitudinal GP training data")

            # Adjust layout
            plt.tight_layout()

            # Show plot
            plt.show(block=True)


        #self.GP_long=SGPModel(train_x=train_x, train_y=train_y, num_of_inducing=None, training_iter=200)
        if self.GP_type == "SGP":
            self.GP_long=SGPModel(data_x=train_x, data_y=train_y, num_of_inducing=self.num_of_inducing_long, training_iter=200, lr=.1)
        elif self.GP_type == "GRAD_SGP":
            self.GP_long=GRADSGPModel(data_x=train_x, data_y=train_y, num_of_inducing=self.num_of_inducing_long, training_iter=200, lr=.1)
        elif self.GP_type == "RLS_SGP":
            self.GP_long=RLSSGPModel(data_x=train_x, data_y=train_y, num_of_inducing=self.num_of_inducing_long, training_iter=200, lr=.1)

        loss = self.GP_long.train_model()
        print(f"[SGPController]: Longitudinal augmentation Sparse GP trained, with final loss {loss}, switching to evaluation mode!")
        return (train_x, train_y)

    def _train_SGP_lat(self, states, inputs, c, plot_training_data=False):
        """Train the sparse GP model for lateral control augmentation
        
        :param states: Array containing the state variables
        :type states: np.ndarray
        :param inputs: Array containing the control inputs
        :type inputs: np.ndarray
        :param c: curvature
        :type c: float
        :param plot_training_data: If True, the training data is plotted. Defaults to False.
        :type plot_training_data: bool, optional
        """
        v_xi=states[:,3]
        v_eta=states[:,4]
        omega=states[:,5]
        delta=inputs[:,1]

        _, d_v_eta=pynumdiff.finite_difference.first_order(v_eta, self.dt, [50], options={'iterate': True})  
        #_, d_e_eta=pynumdiff.finite_difference.first_opip irder(e_eta, self.dt, [50], options={'iterate': True})   


        train_x=np.vstack((v_xi, v_eta, omega))#, c, theta_e))

        A=-(self.model["C_f"]+self.model["C_r"])/self.model["m"]/(v_xi+1e-6)
        B=self.model["C_f"]/self.model["m"]
        B_c=((self.model["l_r"]*self.model["C_r"]-self.model["l_f"]*self.model["C_f"])/self.model["m"]-1)

        #train_y=d_v_eta-A*v_eta-B*delta
        train_y=d_v_eta-A*v_eta+B*delta-B_c*c

                
        train_x=train_x.T
        start=10
        end=train_x.shape[0]-10
        #step_size = max(1, (end-start) // 20)
        
        # training data + initial training points for evaluation
        train_x=train_x[start:end,:]
        #train_x_init=train_x[::step_size,:]

        train_y=train_y[start:end]
        #train_y=train_y[::step_size]

        #step_size = max(1, (end-start) // 5000)
        #train_x=train_x[::step_size,:]
        #train_y=train_y[::step_size]#+0.01*np.random.randn(train_y[::step_size].shape[0])


        if plot_training_data:
            
            fig, axs = plt.subplots(4, 1, figsize=(8, 10))  # 4 rows, 1 column

            # Plot data on each subplot
            axs[0].plot(v_xi)
            axs[0].set_title(r'$$v_\xi$$')

            axs[1].plot(v_eta)
            axs[1].set_title(r'$$v_\eta$$')

            axs[2].plot(omega)
            axs[2].set_title(r'$$\omega$$')

            axs[3].plot(train_y)
            axs[3].set_title(r'$$\mathrm{train}_y$$')

            plt.suptitle("Lateral GP training data")

            # Adjust layout
            plt.tight_layout()

            # Show plot
            plt.show(block=True)


       #self.GP_lat=SGPModel(train_x=train_x, train_y=train_y, num_of_inducing=None, training_iter=200)
        if self.GP_type == "SGP":
            self.GP_lat=SGPModel(data_x=train_x, data_y=train_y, num_of_inducing=self.num_of_inducing_lat, training_iter=200, lr=.1)
        elif self.GP_type == "GRAD_SGP":
            self.GP_lat=GRADSGPModel(data_x=train_x, data_y=train_y, num_of_inducing=self.num_of_inducing_lat, training_iter=200, lr=.1)
        elif self.GP_type == "RLS_SGP":
            self.GP_lat=RLSSGPModel(data_x=train_x, data_y=train_y, num_of_inducing=self.num_of_inducing_lat, training_iter=200, lr=.1)

        loss = self.GP_lat.train_model()
        print(f"[SGPController]: Lateral controller trained with final loss {loss}, switching to evaluation mode!")
        
        return (train_x, train_y)

    def _calc_lat_train_data(self, states, inputs, c):
        """Calculate the training data for recursive update of lateral control augmentation GP"""
        v_xi=states[3]
        v_eta=states[4]
        omega=states[5]

        delta=inputs[1]

        #v_eta_old = self.prev_state[4]
        #d_v_eta = (v_eta - v_eta_old) / self.dt # TODO: might be noisy
        try:
            self.d_v_eta=.1*((v_eta-self.v_eta_old)/self.dt-self.d_v_eta)+self.d_v_eta
            self.v_eta_old=v_eta
        except AttributeError: # if no previous error value exist assume 0 & store the current value
            self.d_v_eta = 0
            self.v_eta_old=self.prev_state[4]

        A=-(self.model["C_f"]+self.model["C_r"])/self.model["m"]/(v_xi+1e-6)
        B=self.model["C_f"]/self.model["m"]
        B_c=((self.model["l_r"]*self.model["C_r"]-self.model["l_f"]*self.model["C_f"])/self.model["m"]-1)

        
        train_x = np.array([v_xi, v_eta, omega])
        train_y=self.d_v_eta-A*v_eta-B*delta-B_c*c
        
        return train_x, train_y
    

    def _calc_long_training_data(self, states, inputs):
        """Calculate the training data for recursive update of longitudinal control augmentation GP"""
        v_xi=states[3]
        v_eta=states[4]
        omega=states[5]

        d=inputs[0]
        delta=inputs[1]

        d_v_xi_nominal = - self.model["C_m2"]*(1+np.cos(delta))/self.model["m"]*v_xi \
                         + self.model["C_m1"]*(1+np.cos(delta))/self.model["m"]*d \
                         - self.model["C_m3"]*(1+np.cos(delta))/self.model["m"]*np.sign(v_xi)
        
        #v_xi_old = self.prev_state[3]

        #self.d_v_xi_measured = (v_xi - v_xi_old) / self.dt # TODO: might be noisy

        # computationally efficient low-pass filter
        try:
            self.d_v_xi_measured=.1*((v_xi-self.v_xi_old)/self.dt-self.d_v_xi_measured)+self.d_v_xi_measured
            self.v_xi_old=v_xi
        except AttributeError: # if no previous error value exist assume 0 & store the current value
            self.d_v_xi_measured = 0
            self.v_xi_old=self.prev_state[3]


        train_y = self.d_v_xi_measured-d_v_xi_nominal

        
        train_x = np.array([v_xi, v_eta, omega])

        return train_x, train_y
        

    def train_GP_controllers(self, states, inputs, errors, c, plot_training_data=False):
        """Train the sparse GP controllers for offline augmentation"""
        
        with warnings.catch_warnings():
            warnings.simplefilter("ignore") # ignore warning from linear operator
            data_lat = self._train_SGP_lat(states, inputs, c, plot_training_data=plot_training_data)
            data_long = self._train_SGP_long(states, inputs, plot_training_data=plot_training_data)

        if plot_training_data:
            plt.show(block=True)

        # the last state if saved as a previous state for online tuning
        self.prev_state = states[-1,:]

        return (data_lat, data_long)


    def to_online(self):
        """Switches the GP model to recursive mode to allow continuous update of the model with new data"""
        
        if self.GP_long is None or self.GP_lat is None:
            raise ValueError("The GP models have not been initialized yet!")
        
        if self.GP_type == "SGP":
            raise ValueError("The SGP model cannot be switched to inline mode!")
        
        if self.GP_type == "GRAD_SGP":
            self.GP_long.configure_update_method(self.batch_size, self.retrain_iter)
            self.GP_lat.configure_update_method(self.batch_size, self.retrain_iter)

        elif self.GP_type == "RLS_SGP":
            self.GP_long.configure_update_method(self.batch_size, self.confidence_level, self.forgetting_factor)
            self.GP_lat.configure_update_method(self.batch_size, self.confidence_level, self.forgetting_factor)

        self.mode = "online"

    def recursive_GP_update(self, states, inputs, c):
        """Update the GP models with the latest training data"""
        train_x_long, train_y_long = self._calc_long_training_data(states, inputs)
        train_x_lat, train_y_lat = self._calc_lat_train_data(states, inputs, c)

        self.GP_long.add_to_batch(train_x_long, train_y_long)
        self.GP_lat.add_to_batch(train_x_lat, train_y_lat)


    def __str__(self) -> str:
        return self.GP_type