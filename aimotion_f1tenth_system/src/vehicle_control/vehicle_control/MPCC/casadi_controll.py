import casadi as cs
import numpy as np
from scipy.integrate import ode
import yaml
import os
from .trajectory import Spline_2D
"""
Controller using vector product for error calculation
VIRTUAL INPUT: Containing the d and the delta-> basically the inputs that will be sent to the car
"""
r = 1


class Casadi_MPCC:
    def __init__(self,MPCC_params:dict,input_0, vehicle_param:dict, dt: float, q_c: float, q_l: float, q_t: float,q_delta:float, q_d: float, N: int, x_0, theta_0:float, trajectory: Spline_2D):
        """
        Args:
            q_c (float): Contouring error weight
            q_l (float): Lag error weight
            q_t (float): Progress weight
            N (int): Control horizon
            dt (float): Sampling time
        """
        self.model = Model(vehicle_params=vehicle_param, MPPC_params=MPCC_params)
        self.nx = self.model.nx # = 6
        self.nu = self.model.nu # = 2
        
        self.dt = dt
        self.q_c = q_c
        self.q_l = q_l
        self.q_t = q_t
        self.q_delta = q_delta #The coeficience of the smoothness of the control
        self.q_d =q_d
        self.N = N
        self.trajectory = trajectory

        # Variable horizons for initial guess
        x_0 = np.reshape(x_0, (-1, 1))
        self.X_init = np.repeat(x_0, N, axis=1) #np.repeat(x_0, N, axis=1)  # np.zeros((6, N))
        self.X_init[0, :] = np.reshape(self.trajectory.spl_sx(np.linspace(theta_0, theta_0 + self.N*self.dt*2, self.N)), (-1))
        self.X_init[1, :] = np.reshape(self.trajectory.spl_sy(np.linspace(theta_0, theta_0 + self.N * self.dt * 2, self.N)), (-1))
        
        #Virtual input initital guess

        temp = np.reshape(input_0,(-1,1))

        self.v_U_init = np.repeat(temp, N-1, axis= 1)

        temp = np.zeros((2,1))
        
        
        
        temp = np.reshape(temp,(-1,1)) # I use this to create the initial guess for U (U_init)

        temp = np.zeros((2,1))

        self.U_init = np.repeat(temp, N-2, axis= 1) ##I assume that there is no steering input
        

        
        self.theta_init = np.ones(N)* theta_0
        self.theta_init[0] = theta_0
        self.start_theta = theta_0

        self.v_t_init = np.zeros(N-1) #Store the initial guess for v_theta

        self.lam_g_init = 0 #What does this do?

        # Time measurement variables
        self.time_proc = np.array([])
        self.time_wall = np.array([])
        self.iter_count = np.array([])

        # Initialise optimizer
        self.__init_optimizer()

    def opti_step(self, x_0):
        """
        Method that calculates the optimization solution for given initial values
        :param x_0: initial state
        :return: optimal input, path parameter
        """

        # Set new values for the opti initial parameters
        self.opti.set_value(self.X_0, x_0) #Setting the value of the opti variable
        #self.X_0 is used to as an initial constraint of the solution: self.X[:,0] == self.X_0


        self.opti.set_value(self.v_t_0, self.v_t_init[0])

        self.opti.set_value(self.U_0, self.U_init[:,0])
        self.opti.set_value(self.v_U_0, self.v_U_init[:,0])

        self.opti.set_value(self.theta_0, self.theta_init[0]) 
      
      
        # Giving initial guesses
        self.opti.set_initial(self.X[:, 1:], self.X_init)
        self.opti.set_initial(self.X[:, 1:-1], self.X_init[:, 1:])
        self.opti.set_initial(self.X[:, -1], self.X_init[:, -1])
        self.opti.set_initial(self.U[:, 1:], self.U_init)
        self.opti.set_initial(self.theta[1:], self.theta_init)
        self.opti.set_initial(self.v_t[1:], self.v_t_init)
        
        
        
        self.opti.set_initial(self.v_U[:,1:], self.v_U_init)
        self.opti.set_initial(self.U[:,1:], self.U_init)
        
        self.opti.set_initial(self.X[:, 0], x_0)
        self.opti.set_initial(self.theta[0], self.theta_init[0])
        self.opti.set_initial(self.v_t[0], self.v_t_init[0])

        self.opti.set_initial(self.opti.lam_g, self.lam_g_init)

        # Solve problem
        sol = self.opti.solve()

        # Measurement of IPOPT CPU time
        self.time_proc = np.append(self.time_proc, sol.stats()['t_proc_total'])
        self.time_wall = np.append(self.time_wall, sol.stats()['t_wall_total'])
        self.iter_count = np.append(self.iter_count, sol.stats()['iter_count'])

        # Saving solutions as initial guess for next step
        self.X_init = sol.value(self.X[:, 1:])
        self.U_init = sol.value(self.U[:, 1:])
        self.theta_init = sol.value(self.theta[1:])
        self.v_t_init = sol.value(self.v_t[1:])

        #Not sure wether these are right: (needs testing)
        self.v_U_init = sol.value(self.v_U[:,1:])
        self.U_init = sol.value(self.U[:,1:])


        self.lam_g_init = sol.value(self.opti.lam_g)
        return sol.value(self.X), sol.value(self.v_U),sol.value(self.U), sol.value(self.theta[:]), sol.value(self.v_t)
    

    def __init_optimizer(self):
        """
        Private method that initializes the optimizer
        """

        self.opti = cs.Opti()

        # Declare decision variables
        self.X = self.opti.variable(self.nx, self.N+1)  # state trajectory
        self.U = self.opti.variable(self.nu, self.N-1)  # input trajectory, containing dd and ddelta
        self.theta = self.opti.variable(self.N+1)  # virtual progress state trajectory
        self.v_t = self.opti.variable(self.N)  # virtual speed input trajectory
        self.v_U = self.opti.variable(self.nu, self.N)#virtual input trajectory containing d and delta

        # Parameters for initial conditions
        self.X_0 = self.opti.parameter(6, 1) #The value is set at every call of the optistep(set to the current x[k])
        self.theta_0 = self.opti.parameter(1) #The value is set at every call of optistep  to theta_init[0]
        self.v_t_0 = self.opti.parameter(1) # -||- to v_t_init[0]

        self.U_0 = self.opti.parameter(2,1)
        self.v_U_0 = self.opti.parameter(2,1)

        
        # Set constraints for initial values#

        self.opti.subject_to(self.X[:, 0] == self.X_0)  # initial state

        self.opti.subject_to(self.v_t[0] == self.v_t_0)  # initial path speed

        self.opti.subject_to(self.theta[0] == self.theta_0 + self.dt * self.v_t_0)  # init path parameter

        #self.opti.subject_to(self.U[:,0] == self.U_0) ##Initial input constraint: dd and ddelta

        self.opti.subject_to(self.v_U[:,0] == self.v_U_0+ self.dt*self.U_0) #init virtual input (d & delta)


        

        # Dynamic constraint #

        x_pred = self.model.predict(states=self.X[:,:-1], inputs=self.v_U, dt=self.dt)[0] 
        # x[k] = model.predict(Self.X[k-1], self.v_U[k-1]) -> arrays are sent as arguments-> array is returned
        #Size of the arguments: 1) self.X[:,:-1]: 6xN 2) self.v_U: 2xN-> returns 6xN new states (note that they are shifted by 1 timespamp)


        self.opti.subject_to(self.X[:, 1:] == x_pred) #Settings the constraint


        theta_next = self.theta[:-1] + self.dt * self.v_t #Theta[k+1] = Theta[k]+dt*v_t
        self.opti.subject_to(self.theta[1:] == theta_next) #Settings constraint


        v_U_next = self.v_U[:,:-1]+self.dt*self.U #v_U[k+1] = ... (basically the same)
        #v_U: is a 2xN array & U is a 2x(N-1) array

        self.opti.subject_to(self.v_U[:,1:] == v_U_next) #Constraint :)

        self.opti.subject_to((self.theta[-1]) <= self.trajectory.L)


        
        # State constraints
        #self.opti.subject_to((0 < cs.vec(self.X[3, 1:]), cs.vec(self.X[3, 1:]) <= 5.5))  # Xi speed component constraint
        #self.opti.subject_to((-3 <= cs.vec(self.X[4, 1:]), cs.vec(self.X[4, 1:]) <= 3))  # Eta speed component constraint
        #self.opti.subject_to((-5 <= cs.vec(self.X[5, 1:]), cs.vec(self.X[5, 1:]) <= 5))  # Angular speed constraint
        
        

        # Virtual Input constraints: d and delta
        #self.opti.subject_to((0.0 < cs.vec(self.v_U[0, 1:]), cs.vec(self.v_U[0, 1:]) <= 1))  # motor reference constraints
        self.opti.subject_to((0.0 < cs.vec(self.v_U[0, 1:]), cs.vec(self.v_U[0, 1:]) <= self.model.parameters["d_max"]))  # motor reference constraints

        self.opti.subject_to((-np.pi/4 <= cs.vec(self.v_U[1, 1:]), cs.vec(self.v_U[1, 1:]) <= np.pi/4 ))  # steering angle constraints

         
        # Input constraints: derivate of d and delta
        self.opti.subject_to((-10 <= cs.vec(self.U[0, :]), cs.vec(self.U[0, :]) <= 10))  # motor reference constraints
        self.opti.subject_to((-1000 <= cs.vec(self.U[1, :]), cs.vec(self.U[1, :]) <= 1000))  # steering angle constraints
        
        """
        self.opti.subject_to((self.model.parameters["d_min"] < cs.vec(self.v_U[0, 1:]), cs.vec(self.v_U[0, 1:]) <= self.model.parameters["d_max"]))  # motor reference constraints
        self.opti.subject_to((-self.model.parameters["delta_max"]<= cs.vec(self.v_U[1, 1:]), cs.vec(self.v_U[1, 1:]) <= self.model.parameters["delta_max"] ))  # steering angle constraints

         
        # Input constraints: derivate of d and delta
        self.opti.subject_to((-self.model.parameters["ddot_max"] <= cs.vec(self.U[0, :]), cs.vec(self.U[0, :]) <= self.model.parameters["ddot_max"]))  # motor reference constraints
        self.opti.subject_to((-self.model.parameters["deltadot_max"]<= cs.vec(self.U[1, :]), cs.vec(self.U[1, :]) <= self.model.parameters["deltadot_max"]))  # steering angle constraints
        """

        #Error constraints:

        #e_l = self.e_l(cs.vcat((self.X[0, :], self.X[1, :])), self.theta)
        #e_c = self.e_c(cs.vcat((self.X[0, :], self.X[1, :])), self.theta)
        #e_smooth = self.e_smooth()

        #self.opti.subject_to(cs.mmax(e_c) < 0.05)
        
        #self.opti.subject_to(cs.mmax(e_l) < 0.05)
        #self.opti.subject_to(e_c < 50)


        #self.opti.subject_to(e_c.T @ e_c < 0.1)
        #self.opti.subject_to([e_smooth[0] < 5]) #d smoothness
        #self.opti.subject_to(e_smooth[1] < 500) #delta smoothness
        

        # Path speed constraints
        self.opti.subject_to((0.0 < cs.vec(self.v_t[1:]), cs.vec(self.v_t[:]) <= 3))

        # Set objective function
        self.opti.minimize(self.cost())

        # Solver setup
        p_opt = {'expand': False}
        s_opts = {'max_iter': 2400, 'print_level': 0}
        self.opti.solver('ipopt', p_opt, s_opts)
   


    def cost(self):
        """
        Method which returns the cost
        :return: cost
        """
        e_l = self.e_l(cs.vcat((self.X[0, :], self.X[1, :])), self.theta)
        e_c = self.e_c(cs.vcat((self.X[0, :], self.X[1, :])), self.theta)
        
        e_smooth = self.e_smooth()[0]*self.q_d + self.e_smooth()[1]*self.q_delta
        
        cost = self.q_l * e_l.T @ e_l + self.q_c * e_c.T @ e_c- self.q_t * (self.v_t.T @ self.v_t) + e_smooth 
        #cost = self.q_l * e_l + self.q_c * e_c- self.q_t * (self.trajectory.L-self.theta[-1]) + self.q_smooth  * e_smooth

        return cost

    def e_smooth(self):
        """
        This function puts the to the rows of the input (dd and ddelta) above each other in a vector and returns it
        """

        d = cs.vec(self.U[0,:])

        delta = cs.vec(self.U[1,:])


        e_smooth = np.array([d.T @ d, delta.T @ delta])  #Basically I calculate the lenght^2 of the the d and the delta vectors and add them together
        #This way e_smoot is always >= 0

        return e_smooth

    def e_c(self, point, theta):
        """
        Contouring error function
        :param point: array containing x and y coordinates
        :param theta: path parameter(s)
        :return: contouring error
        """
        point_r, v = self.est_ref_pos(theta)
        n = cs.hcat((v[:, 1], -v[:, 0]))
        e_c = (point_r-point)*n.T
        e_c = cs.vec(e_c[0, :] + e_c[1, :])
        #e_c = e_c.T @ e_c
        return e_c

    def e_l(self, point, theta):
        """
        Lag error function
        :param point: array containing x and y coordinates
        :param theta: path parameter(s)
        :return: lag error
        """
        point_r, v = self.est_ref_pos(theta)
        e_l = (point_r-point)*v.T
        e_l = cs.vec(e_l[0, :]+e_l[1, :])
        #e_l = e_l.T @ e_l
        return e_l

    def est_ref_pos(self, theta):
        if self.trajectory is None:
            # trial circle arc length parametrisation
            x_r = r-r*cs.cos(theta/r) + cs.sin(theta/r) * (theta-self.theta_0)
            y_r = r*cs.sin(theta/r) + cs.cos(theta/r) * (theta-self.theta_0)
            point = cs.vcat((x_r.T, y_r.T))
            v = cs.hcat((cs.cos(np.pi/2-theta/r), cs.sin(np.pi/2-theta/r)))
            return point, v
        else:
            return self.trajectory.get_path_parameters(theta, self.theta_0)
        

class Model:
    def __init__(self, vehicle_params: dict ,MPPC_params: dict):
        '''Class implementation of the dynamic model of a small scale ground vehicle
        Args:
            param_file (str): Yaml file name of the configuration file
        '''
        # load model parameters



        self.parameters = MPPC_params
        # assign model parameters
        self.m =vehicle_params['m']
        self.I_z = vehicle_params['I_z']
        self.l_f = vehicle_params['l_f']
        self.l_r = vehicle_params['l_r']

        self.C_m1 = vehicle_params['C_m1']
        self.C_m2 = vehicle_params['C_m2']
        self.C_m3 = vehicle_params['C_m3']

        self.C_r = vehicle_params['C_r']
        self.C_f = vehicle_params['C_f']

        self.nx = 6
        self.nu = 2

        # constraints


    def nonlin_dynamics(self, t, states, inputs):
        '''
        Class method calculating the state derivatives using nonlinear model equations
        :param t: Current time
        :param states: State vector
        :param inputs: Input vector
        :return: Derivative of states
        '''
        if type(states) == cs.MX:
            x = states[0, :]
            y = states[1, :]
            phi = states[2, :]
            v_xi = states[3, :]
            v_eta = states[4, :]
            omega = states[5, :]
            d = inputs[0, :]
            delta = inputs[1, :]
        else:
            x = states[0]
            y = states[1]
            phi = states[2]
            v_xi = states[3]
            v_eta = states[4]
            omega = states[5]
            d = inputs[0]
            delta = inputs[1]

        # slip angles
        alpha_r = cs.arctan((-v_eta + self.l_r*omega)/(v_xi+0.001))
        alpha_f = delta - cs.arctan((v_eta + self.l_f * omega)/(v_xi+0.001))

        # tire forces
        F_xi = self.C_m1*d - self.C_m2*v_xi - self.C_m3*cs.sign(v_xi)
        F_reta = self.C_r*alpha_r
        F_feta = self.C_f*alpha_f

        # nonlinear state equations
        dx = v_xi * cs.cos(phi) - v_eta * cs.sin(phi)
        dy = v_xi * cs.sin(phi) + v_eta * cs.cos(phi)
        dphi = omega

        dv_xi = 1 / self.m * (F_xi + F_xi * cs.cos(delta) - F_feta * cs.sin(delta) + self.m * v_eta * omega)
        dv_eta = 1 / self.m * (F_reta + F_xi * cs.sin(delta) + F_feta * cs.cos(delta) - self.m * v_xi * omega)
        domega = 1 / self.I_z * (F_feta * self.l_f * cs.cos(delta) + F_xi * self.l_f * cs.sin(delta) - F_reta * self.l_r)
        d_states = cs.vertcat(dx, dy, dphi, dv_xi, dv_eta, domega)
        return d_states

    def predict(self, states, inputs, dt, t=0, method='RK4'):
        ''' Class method predicting the next state of the model from previous state and given input

        :param states: State vector
        :param inputs: Input vector
        :param t: Current time
        :param dt: Sampling time
        :param method: Numerical method of solving the ODEs
        :return: Predicted state vector, predicted time
        '''
        if method == 'RK4':
            k1 = self.nonlin_dynamics(t, states, inputs)
            k2 = self.nonlin_dynamics(t + dt / 2, states + dt / 2 * k1, inputs)
            k3 = self.nonlin_dynamics(t + dt / 2, states + dt / 2 * k2, inputs)
            k4 = self.nonlin_dynamics(t + dt, states + dt * k3, inputs)
            states_next = states + dt/6 * (k1 + 2.0*k2 + 2.0*k3 + k4)
            t_next = t + dt

        elif method == 'FE':
            states_next = states + dt*self.nonlin_dynamics(t, states, inputs)
            t_next = t + dt
        return states_next, t_next

    def simulate(self, states, inputs, dt, t=0):
        sim = ode(self.nonlin_dynamics).set_integrator('lsoda')
        sim.set_initial_value(states, t).set_f_params(inputs)

        return np.reshape(sim.integrate(dt), (-1, 1)), sim.t+dt