from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import casadi as cs
import yaml
from ..MPCC.trajectory import Spline_2D
import numpy as np
from scipy.interpolate import splev
from scipy.integrate import ode
import os
from ..MPCC.casadi_controll import Casadi_MPCC

class MPCC_Controller:
    def __init__(self, vehicle_params: dict, MPCC_params: dict):
        """
        Init controller parameters
        :param vehicle_params: dict
        :param MPCC_params: dict
        """
        self.vehicle_params = vehicle_params
        self.MPCC_params = MPCC_params
        self.trajectory = None

        self.theta = 0.0
        self.s_start = 0.0
        self.x0 = np.zeros((1,6))

        self.input = np.array([0.5,0])
       
        self.ocp_solver = None #acados solver to compute control

        self.casadi_solver = None #casadi_solver for initial guess

     
    def compute_control(self, x0, setpoint = None):
        """
        Calculating the optimal inputs
        :param x0 (1xNx array)
        :setpoint = None
        :return u_opt (optimal input vector)
        :return errors (contouring longitudinal errors, phi, theta)
        """
        
        x0 = np.concatenate((x0, np.array([self.theta]), self.input))

        self.ocp_solver.set(0, 'lbx', x0)
        self.ocp_solver.set(0, 'ubx', x0)
        self.ocp_solver.set(0, 'x', x0)
       

        self.ocp_solver.solve()
        #u_opt = np.reshape(self.ocp_solver.get(0, "u"),(-1,1))

        x_opt = np.reshape(self.ocp_solver.get(1, "x"),(-1,1)) #Full predictied optimal state vector (x,y,phi, vxi, veta, omega, thetahat, d, delta)
        self.theta = x_opt[6,0]
        self.input = x_opt[7:, 0]
        u_opt = np.reshape(self.ocp_solver.get(0, "x"),(-1,1))[7:,0] 

        u_opt = np.reshape(u_opt, (-1,1))

        
        for i in range(self.parameters.N-1):
            self.ocp_solver.set(i, "x", self.ocp_solver.get(i+1, "x"))

        for i in range(self.parameters.N-2):
            self.ocp_solver.set(i, "u", self.ocp_solver.get(i+1, "u"))
        print(self.trajectory.e_c(x_opt[:2,0], self.theta)[0][0])
        errors = np.array((self.trajectory.e_c(x_opt[:2,0], self.theta)[0][0], #Contouring error
                            x_opt[2,0], #Heading error
                            self.trajectory.e_l(x_opt[:2,0],self.theta)[0], self.theta, #Long error
                            np.nan, #v_error
        ))
        
        return u_opt, errors
    

    def controller_init(self):
        """
        Calculate intial guess for the state horizon.
        """
        X, v_U, theta = self.casadi_solver.opti_step(self.x0) #Call casadi solver for optimal initial guess


        x_0 = np.concatenate((self.x0, np.array([self.theta]), np.array([0.5,0])))

        self.ocp_solver.set(0, "x", x_0)
        for i in range(self.parameters.N-1):
            temp = np.concatenate((X[:,i+1],np.array([theta[i+1]]), v_U[:,i+1]))
            temp = np.reshape(temp, (-1,1))
            self.ocp_solver.set(i+1, "x", temp[:,0])
        

    def _generate_model(self):
        """
        Class method for creating the AcadosModel. 
        Sets self.parameters used by the casadi solver.
        """
        

        self.parameters = cs.types.SimpleNamespace()
        self.sim_par = cs.types.SimpleNamespace()

        m = float(self.vehicle_params["m"])
        l_f = float(self.vehicle_params["l_f"])
        l_r = float(self.vehicle_params["l_r"])
        I_z = float(self.vehicle_params["I_z"])

        C_m1 = float(self.vehicle_params["C_m1"])
        C_m2 = float(self.vehicle_params["C_m2"])
        C_m3 = float(self.vehicle_params["C_m3"])

        C_f = float(self.vehicle_params["C_f"])
        C_r = float(self.vehicle_params["C_r"])

        

        self.parameters.m = m
        self.parameters.l_f = l_f
        self.parameters.l_r = l_r
        self.parameters.I_z = I_z
        self.parameters.C_m1 = C_m1
        self.parameters.C_m2 = C_m2
        self.parameters.C_m3 = C_m3
        self.parameters.C_f = C_f
        self.parameters.C_r = C_r

        #incremental input constraints:

        self.parameters.ddot_max= float(self.MPCC_params["ddot_max"])
        self.parameters.deltadot_max = float(self.MPCC_params["deltadot_max"])
        self.parameters.thetahatdot_min = float(self.MPCC_params["thetahatdot_min"])
        self.parameters.thetahatdot_max = float(self.MPCC_params["thetahatdot_max"])

        #input constraints:
        delta_max = float(self.MPCC_params["delta_max"])
        d_max = float(self.MPCC_params["d_max"])
        d_min = float(self.MPCC_params["d_min"])
        #ocp parameters:
        self.parameters.N = int(self.MPCC_params["N"])
        self.parameters.Tf = float(self.MPCC_params["Tf"])

        #cost weights:
        self.parameters.delta_max = delta_max
        self.parameters.d_max = d_max
        self.parameters.d_min = d_min
        self.parameters.q_con = float(self.MPCC_params["q_con"])
        self.parameters.q_lat = float(self.MPCC_params["q_long"])
        self.parameters.q_theta = float(self.MPCC_params["q_theta"])
        self.parameters.q_d = float(self.MPCC_params["q_d"])
        self.parameters.q_delta = float(self.MPCC_params["q_delta"])

        model = AcadosModel()

        model.name = "f1tenth_bicycle_model"

        """ Creating the state vector: [x,y,vx,vy,thetahat, thetahat, d, delta]' """
        x = cs.MX.sym('x')
        y = cs.MX.sym('y')
        phi = cs.MX.sym("phi")
        vxi = cs.MX.sym("vxi")
        veta = cs.MX.sym("veta")
        omega = cs.MX.sym("omega")
        thetahat = cs.MX.sym("thetahat")
        d = cs.MX.sym("d")
        delta = cs.MX.sym("delta")


        model.x = cs.vertcat(x,y,phi,vxi, veta, omega,thetahat, d, delta)

        #Defining the slip angles
        alpha_r = cs.arctan2((-veta+l_r*omega),(vxi+0.0001)) #In the documentation arctan2 is used but vxi can't be < 0
        alpha_f = delta- cs.arctan2((veta+l_f*omega),(vxi+0.0001))

        #Wheel forces

        Fxi = C_m1*d-C_m2*vxi-vxi*C_m3

        Freta = C_r * alpha_r
        Ffeta = C_f*alpha_f
        
        #State derivates:
        xdot = cs.MX.sym("xdot")
        ydot =  cs.MX.sym("ydot")
        phidot =  cs.MX.sym("phidot")
        vxidot =  cs.MX.sym("vxidot")
        vetadot =  cs.MX.sym("vetadot")
        omegadot =  cs.MX.sym("omegadot")
        thetahatdot =  cs.MX.sym("thetadot")
        ddot =  cs.MX.sym("ddot")
        deltadot =  cs.MX.sym("deltadot")

        model.xdot = cs.vertcat(xdot,ydot, phidot, vxidot, vetadot,omegadot, thetahatdot, ddot, deltadot)

        """Input vector: [ddot, deltadot, thetahatdot]' """
        model.u = cs.vertcat(thetahatdot, ddot, deltadot)


        """Explicit expression:"""

        model.f_expl_expr = cs.vertcat(
            vxi*cs.cos(phi)-veta*cs.sin(phi), #xdot
            vxi*cs.sin(phi)+veta*cs.cos(phi), #ydot
            omega, #phidot
            (1/m)*(Fxi+Fxi*cs.cos(delta)-Ffeta*cs.sin(delta)+m*veta*omega), #vxidot
            (1/m)*(Freta+Fxi*cs.sin(delta)+Ffeta*cs.cos(delta)-m*vxi*omega), #vetadot
            (1/I_z)*(Ffeta*l_f*cs.cos(delta)+Fxi*l_f*cs.sin(delta)-Freta*l_r), #omegadot
            thetahatdot,
            ddot,
            deltadot,
        )

        #Current position
        point = cs.vertcat(x,y) 

        model.cost_expr_ext_cost = self._cost_expr(point, theta=thetahat,thetahatdot=thetahatdot, ddot = ddot, deltadot = deltadot)

        return model


    def _cost_expr(self, point,theta,thetahatdot, ddot, deltadot):
        """
        Definition of the cost expression
        :param point: array containing x and y coordinates
        :param theta: path parameter
        :return: cost value (scalar)
        """

        e_c = self._cost_e_c(point,theta)
        e_l = self._cost_e_l(point,theta)
        cost = e_c**2*self.parameters.q_con+e_l**2*self.parameters.q_lat-thetahatdot*self.parameters.q_theta+self.parameters.q_d*ddot**2+self.parameters.q_delta*deltadot**2
        return cost


    def _cost_e_c(self, point ,theta):
        """
        Contouring error function
        :param point: array containing x and y coordinates
        :param theta: path parameter(s)
        :return: contouring error
        """

        point_r, v =  self.trajectory.get_path_parameters(theta, self.s_start) #point: vertical, v: horizontal
        n = cs.hcat((v[:, 1], -v[:, 0])) #Creating a perpendicular vector
        e_c = cs.dot(n.T,(point_r-point))
        return e_c


    def _cost_e_l(self, point, theta):
        """
        Lag error function
        :param point: array containing x and y coordinates
        :param theta: path parameter(s)
        :return: lag error
        """
        point_r, v = self.trajectory.get_path_parameters(theta, self.s_start)
        e_l = cs.dot(v.T,(point_r-point))
        return e_l
    
    def reset(self):
        pass

    def train_GP_controllers(self, *args, **kwargs):
        raise NotImplementedError

    def _generate_ocp_solver(self, model: AcadosModel):
        """
        Creates the acados ocp solver
        :param model: AcadosModel, generated by the class method
        :return ocp_solver: AcadosOcpSolver
        """
        ocp = AcadosOcp()
        ocp.model = model

        ocp.dims.N = self.parameters.N

        ocp.solver_options.tf = self.parameters.Tf

        ocp.cost.cost_type = "EXTERNAL"

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'#'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI
        ocp.solver_options.nlp_solver_max_iter = 10000
        ocp.solver_options.nlp_solver_tol_stat = 1e-6
        ocp.solver_options.levenberg_marquardt = 10.0
        ocp.solver_options.print_level = 0
        ocp.solver_options.qp_solver_iter_max = 10000
        ocp.code_export_directory = 'c_generated_code'
        ocp.solver_options.hessian_approx = 'EXACT'

        lbx = np.array((self.parameters.d_min, -self.parameters.delta_max))
        ubx = np.array((self.parameters.d_max, self.parameters.delta_max))

        ocp.constraints.lbx = lbx
        ocp.constraints.ubx = ubx
        ocp.constraints.idxbx = np.array((7,8)) #d and delta

        lbu = np.array((self.parameters.thetahatdot_min,
                        -self.parameters.ddot_max, 
                        -self.parameters.deltadot_max
                        ))
        
        ubu = np.array((self.parameters.thetahatdot_max,
                        self.parameters.ddot_max, 
                        self.parameters.deltadot_max))

        ocp.constraints.ubu = ubu
        ocp.constraints.lbu = lbu
        ocp.constraints.idxbu = np.arange(3)
        phi0 = float(self.trajectory.get_path_parameters_ang(self.s_start)[2])

        #x0 = np.array((float(self.trajectory.spl_sx(self.s_start)), #x
        #            float(self.trajectory.spl_sy(self.s_start)), #y
        #            phi0,#phi
        #            0.001, #vxi
        #            0, #veta
        #            0, # omega
        #            self.s_start+0.05,
        #            0.02, #d
        #            0, #delta
        #))
        x0 = np.concatenate((self.x0, np.array([self.theta]), self.input))

        ocp.constraints.x0 = x0 #Set in the set_trajectory function
        ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')
        return ocp_solver
    

    def set_trajectory(self, pos_tck, evol_tck, x0, theta_start):
        """
        Evaluetes the reference spline from the given spline tck, and converts it into a Spline2D instance
        :param pos_tck: array
        :param evol_tck: array, not used
        :param x0: initial state, used for initialising the controller
        :param thetastart: float, starting arc lenght of the trajectory
        """
        self.theta = theta_start
        self.s_start = theta_start
        
        self.x0 = x0 #The current position must be the initial condition

        self.x0[3] = 0.01 #Give a small forward speed to make the problem feasable

        self.input = np.array([0.5,0])

        s_max = evol_tck[-1]

        s = np.linspace(0, s_max, 100)

        (x,y) = splev(s, pos_tck)

        points_list = []

        for i in range(len(x)):
            points_list.append([i, x[i], y[i]])
        print(f"initial state: {self.x0}")
        #print(f"starting point: {points_list}")
        self.trajectory = Spline_2D(np.array(points_list))

        self.ocp_solver = self._generate_ocp_solver(self._generate_model())
        self.casadi_solver = Casadi_MPCC(MPCC_params=self.MPCC_params,
                                         vehicle_param=self.vehicle_params,
                                         dt = self.parameters.Tf/self.parameters.N,
                                         q_c = self.parameters.q_con,
                                         q_l= self.parameters.q_lat,
                                         q_t = self.parameters.q_theta,
                                         q_delta=self.parameters.q_delta,
                                         q_d = self.parameters.q_d,
                                         theta_0=self.theta,
                                         trajectory=self.trajectory,
                                         N = self.parameters.N,
                                         x_0 = self.x0)
        self.controller_init()


    def nonlin_dynamics(self,t, states, inputs):
        '''
        Class method calculating the state derivatives using nonlinear model equations
        :param t: Current time
        :param states: State vector
        :param inputs: Input vector
        :return: Derivative of states
        '''
        states = np.reshape(states, (1,-1))
        inputs = np.reshape(inputs, (1,-1))
        states = states[0, :]
        inputs = inputs[0,:]
        x = states[0]
        y = states[1]
        phi = states[2]
        v_xi = states[3]
        v_eta = states[4]
        omega = states[5]

        d = inputs[0]
        delta = inputs[1]

        # slip angles
        alpha_r = cs.arctan((-v_eta + self.sim_par.l_r*omega)/(v_xi+0.001))
        alpha_f = delta - cs.arctan((v_eta + self.sim_par.l_f * omega)/(v_xi+0.001))

        # tire forces
        F_xi = self.sim_par.C_m1*d - self.sim_par.C_m2*v_xi - self.sim_par.C_m3*cs.sign(v_xi)
        F_reta = self.sim_par.C_r*alpha_r
        F_feta = self.sim_par.C_f*alpha_f

        # nonlinear state equations
        dx = v_xi * cs.cos(phi) - v_eta * cs.sin(phi)
        dy = v_xi * cs.sin(phi) + v_eta * cs.cos(phi)
        dphi = omega

        dv_xi = 1 / self.sim_par.m * (F_xi + F_xi * cs.cos(delta) - F_feta * cs.sin(delta) + self.sim_par.m * v_eta * omega)
        dv_eta = 1 / self.sim_par.m * (F_reta + F_xi * cs.sin(delta) + F_feta * cs.cos(delta) - self.sim_par.m * v_xi * omega)
        domega = 1 / self.sim_par.I_z * (F_feta * self.sim_par.l_f * cs.cos(delta) + F_xi * self.sim_par.l_f * cs.sin(delta) - F_reta * self.sim_par.l_r)
        d_states = cs.vertcat(dx, dy, dphi, dv_xi, dv_eta, domega)
        return d_states


    def predict(self, states, inputs, dt, t=0, method = "RK4"):
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
        """
        Class method for simulating the f1tenth vehicle
        :param states: State vector
        :param inputs: Input vector
        :param dt: integration time
        :param t: t_0
        :return states: State vector at t+dt time step
        :return t: t_0+dt
        """
        sim = ode(self.nonlin_dynamics).set_integrator('lsoda')
        sim.set_initial_value(states, t).set_f_params(inputs)

        return np.reshape(sim.integrate(dt), (-1, 1)), sim.t+dt

        