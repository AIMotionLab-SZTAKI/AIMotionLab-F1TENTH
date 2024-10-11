import casadi as cs
import yaml
import numpy as np
from scipy.interpolate import splev
from scipy.integrate import ode
import os
import time
import matplotlib.pyplot as plt

class F1TENTH_sim:
    def __init__(self, vehicle_params: dict):
        """
        Init simulator
        :param vehicle_params: dict
        """
        self.vehicle_params = vehicle_params
        self.load_parameters()


    def load_parameters(self):
        """
        Load self parameters from the dict-s
        """
        self.parameters = cs.types.SimpleNamespace()

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
        
    def nonlin_dynamics(self,t, states, inputs):
        '''
        Class method calculating the state derivatives using nonlinear model equations
        :param t: Current time
        :param states: State vector
        :param inputs: Input vector
        :return: Derivative of states
        '''
        #states = np.reshape(states, (1,-1))
        #inputs = np.reshape(inputs, (1,-1))
        #states = states[0, :]
        #inputs = inputs[0,:]
        x = states[0]
        y = states[1]
        phi = states[2]
        v_xi = states[3]
        v_eta = states[4]
        omega = states[5]

        d = inputs[0]
        delta = inputs[1]

        # slip angles
        alpha_r = cs.arctan((-v_eta + self.parameters.l_r*omega)/(v_xi+0.001))
        alpha_f = delta - cs.arctan((v_eta + self.parameters.l_f * omega)/(v_xi+0.001))

        if v_xi < 0:
            alpha_r = cs.arctan((v_eta - self.parameters.l_r*omega)/(v_xi-0.001))
            alpha_f = -delta + cs.arctan((v_eta + self.parameters.l_f * omega)/(v_xi-0.001))

        # tire forces
        F_xi = self.parameters.C_m1*d - self.parameters.C_m2*v_xi - self.parameters.C_m3*cs.sign(v_xi)
        F_reta = self.parameters.C_r*alpha_r
        F_feta = self.parameters.C_f*alpha_f

        # nonlinear state equations
        dx = v_xi * cs.cos(phi) - v_eta * cs.sin(phi)
        dy = v_xi * cs.sin(phi) + v_eta * cs.cos(phi)
        dphi = omega

        dv_xi = 1 / self.parameters.m * (F_xi + F_xi * cs.cos(delta) - F_feta * cs.sin(delta) + self.parameters.m * v_eta * omega)
        dv_eta = 1 / self.parameters.m * (F_reta + F_xi * cs.sin(delta) + F_feta * cs.cos(delta) - self.parameters.m * v_xi * omega)
        domega = 1 / self.parameters.I_z * (F_feta * self.parameters.l_f * cs.cos(delta) + F_xi * self.parameters.l_f * cs.sin(delta) - F_reta * self.parameters.l_r)
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

        return sim.integrate(dt), sim.t+dt
