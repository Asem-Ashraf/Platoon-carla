#!/usr/bin/env python3.7

import numpy as np
import casadi as ca
from MPC.Dynamics.equations import VehicleModel


class MPC():

    def __init__(self, Horizon, T):
        self.N = Horizon

        self.dynamics = VehicleModel()
        self.n_controls = self.dynamics.n_controls
        self.n_states = self.dynamics.n_states

        self.total_states = Horizon
        self.total_controls = self.total_states - 1

        self.control_values = np.zeros(self.total_controls * self.n_controls)
        self.states_values = np.zeros(self.total_states * self.n_states)

        self.U = ca.SX.sym('u', self.n_controls, self.total_controls)

        self.X = ca.SX.sym('x', self.n_states, self.total_states)
        self.X_ref = ca.SX.sym('x_ref', self.n_states, self.total_states)

        self.UL = ca.SX.sym('ul', self.n_controls)

        obj, g = self.get_costfunction(T, self.dynamics.changeInStates)

        self.lbx, self.ubx, self.lbg, self.ubg, opt_params, opt_variables = self.arrange(
        )

        nlp_prob = {
            'f': obj,
            'x': opt_variables,
            'p': opt_params,
            'g': ca.vertcat(*g)
        }
        opts_setting = {
            'ipopt.max_iter': 50,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    def arrange(self):
        self.a_max = 60
        self.delta_max = np.deg2rad(150)
        upper_bounds_states = {
            'x': np.inf,
            'y': np.inf,
            'psi': np.inf,
            'vx': 30,  # the limit below which carla max acceleration = 6
            'vy': np.inf,
            'psi_dot': np.inf,
        }
        lower_bounds_states = {
            'x': -np.inf,
            'y': -np.inf,
            'psi': -np.inf,
            'vx': 0,  # can't go backwards
            'vy': -np.inf,
            'psi_dot': -np.inf,
        }
        upper_bounds_controls = {
            'acc': self.a_max,
            'delta': self.delta_max,
        }
        lower_bounds_controls = {
            'acc': -self.
            a_max,  # This negative value is not for going backward (braking).
            'delta': -self.delta_max,
        }
        lbs = list(lower_bounds_states.values())
        ubs = list(upper_bounds_states.values())
        lbc = list(lower_bounds_controls.values())
        ubc = list(upper_bounds_controls.values())

        lbx = []
        ubx = []
        lbg = 0.0
        ubg = 0.0

        for _ in range(self.total_controls):
            lbx = np.concatenate((lbx, lbc))
            ubx = np.concatenate((ubx, ubc))

        for _ in range(self.total_states):
            lbx = np.concatenate((lbx, lbs))
            ubx = np.concatenate((ubx, ubs))

        opt_variables = ca.vertcat(ca.reshape(self.U, -1, 1),
                                   ca.reshape(self.X, -1, 1))
        opt_params = ca.vertcat(self.UL, ca.reshape(self.X_ref, -1, 1))

        return lbx, ubx, lbg, ubg, opt_params, opt_variables

    def get_costfunction(self, t, changeinstates):
        g = []
        obj = 0

        # Minimize
        Q = np.diag([
            400,  # X
            400,  # Y
            400,  # psi
            10,  # Vx
            1,  # Vy
            1  # omega
        ])

        # Minimize change in acceeration and steering
        Rchange = np.diag([0.01, 0.1])
        # Minimize acceeration
        # R = np.diag([0, 0.0])

        g.append(self.X[:, 0] - self.X_ref[:, 0])
        for i in range(self.n_states - 1):
            nextstate = self.X[:, i] + t * changeinstates(
                self.X[:, i], self.U[:, i])
            g.append(self.X[:, i + 1] - nextstate)

        # minimize error in references for better following of the trajectory
        for i in range(self.n_states - 1):
            referror = self.X[:, i] - self.X_ref[:, i]
            obj = obj + ca.mtimes([referror.T, Q, referror])
        # prioritize minimize error in the last references point for better following of the trajectory
        referror = self.X[:, -1] - self.X_ref[:, -1]
        obj = obj + ca.mtimes([referror.T, 2 * Q, referror])

        # minimize controls for more fuel efficiency(if the acc was reduced)
        # for i in range(self.n_controls):
        #     cont = self.U[:, i]
        #     obj = obj + ca.mtimes([cont.T, R, cont])

        # given the control that was applied last time, minimize the difference between the last control and the first control
        changeincont = self.U[:, 0] - self.UL
        obj = obj + ca.mtimes([changeincont.T, Rchange, changeincont])
        # Then minimize control changes across the whole trajectory for more vehicle stability
        for i in range(self.n_controls - 1):
            changeincont = self.U[:, i] - self.U[:, i + 1]
            obj = obj + ca.mtimes(
                [changeincont.T, 0.1 * Rchange, changeincont])

        return obj, g

    def get_control(self, references):
        N = self.N
        all_states_init_val = np.array(references[:N]).reshape(-1)
        parameters = np.concatenate(
            (self.control_values[:2], all_states_init_val))
        init_values = np.concatenate(
            (self.control_values, all_states_init_val))
        solution = self.solver(x0=init_values,
                               p=parameters,
                               lbx=self.lbx,
                               ubx=self.ubx,
                               lbg=self.lbg,
                               ubg=self.ubg)

        estimated_values = solution['x'].full()
        self.control_values = estimated_values[:self.n_controls *
                                               (self.total_controls)].reshape(
                                                   -1)
        self.states_values = estimated_values[(self.total_controls) *
                                              self.n_controls:].reshape(-1)
        acc = estimated_values[0][0] / self.a_max
        steer = estimated_values[1][0] / self.delta_max

        x1, y1 = references[0][0], references[0][1]
        x2, y2 = references[-1][0], references[-1][1]
        dis = np.sqrt(((x1 - x2)**2) + ((y1 - y2)**2))
        SAFE_DISTANCE = 12
        if dis < SAFE_DISTANCE:
            return -1, steer

        return acc, steer
