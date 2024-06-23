#!/usr/bin/env python3.7

import numpy as np
import casadi as ca


class VehicleModel():
    def __init__(self, instance):
        self.instance = instance
    # def __init__(self):
    #     # self.instance = instance

        # smallest possible positive float value
        spv = np.finfo( np.float64).eps

        m, Iz, lf, lr, caf, car = self.getConstants()
        # m, Iz, lf, lr, caf, car = 1500,150,1.5,1.6,20,20

        x = ca.SX.sym('X')
        y = ca.SX.sym('Y')
        psi = ca.SX.sym('psi')
        vx = ca.SX.sym('vx')
        vy = ca.SX.sym('vy')
        omega = ca.SX.sym('psi_dot')

        currentStates = ca.vertcat(x, y, psi, vx, vy, omega)
        self.n_states = currentStates.size()[0]

        u_acc = ca.SX.sym('u_acc')
        u_delta = ca.SX.sym('u_delta')

        controls = ca.vertcat(u_acc, u_delta)
        self.n_controls = controls.size()[0]



        #############################################################
        # TYRE MODEL                                                #
        #############################################################
        af = ca.arctan(
            (-vx * ca.sin(u_delta) + (vy + omega * lf) * ca.cos(u_delta)) /
            (spv + vx * ca.cos(u_delta) + (vy + omega * lf) * ca.sin(u_delta)))
        ar = ca.arctan((vy - omega * lr) /
                       (vx + spv))
        # af = (u_delta - ca.atan((vy + psi_dot*lr)/ca.fmax(spv,vx)))
        # ar = ca.atan((psi_dot*lr-vy)/ca.fmax(spv,vx))
        ff = -caf * af
        fr = -car * ar
        #############################################################

        #############################################################
        # EQUATIONS OF MOTION                                       #
        #############################################################
        Dx = vx * ca.cos(psi) - vy * ca.sin(psi)                    #
        Dy = vx * ca.sin(psi) + vy * ca.cos(psi)                    #
        Dpsi = omega                                                #
        # dpsi = vx*np.tan(u_delta)/(lf+lr)                         #
        DVx = omega * vy - ff * ca.sin(u_delta) / m + u_acc         #
        DVy = -omega * vx + (ff * ca.cos(u_delta) + fr) / m         #
        Domega = (lf * ff * ca.cos(u_delta) - lr * fr) / (Iz)       #
        #############################################################

        changeInStatesDynamics = ca.vertcat(Dx, Dy, Dpsi, DVx, DVy, Domega)
        self.changeInStates = ca.Function('f', [currentStates, controls],
                                          [changeInStatesDynamics],
                                          ['st', 'con'], ['chgs'])

    def getConstants(self):
        pc = self.instance.get_physics_control()

        # wheels
        wfl = pc.wheels[0]
        wfr = pc.wheels[1]
        wrl = pc.wheels[2]
        wrr = pc.wheels[3]
        center_of_mass = pc.center_of_mass + self.instance.get_transform().location
        wflpos = wfl.position / 100
        wfrpos = wfr.position / 100
        wrlpos = wrl.position / 100
        wrrpos = wrr.position / 100
        wd = wflpos.distance_2d(wfrpos)

        # cafr = wfr.lat_stiff_value
        # cafl = wfl.lat_stiff_value
        # carr = wrr.lat_stiff_value
        # carl = wrl.lat_stiff_value

        lf = center_of_mass.distance_2d((wflpos + wfrpos) / 2)
        lr = center_of_mass.distance_2d((wrrpos + wrlpos) / 2)
        m = pc.mass
        Iz = 150
        caf = 20
        car = 20
        # print(f"lf {lf}, lr {lr}, w {wd}, mass {m}, Iz {Iz}")

        return m, Iz, lf, lr, caf, car

    def test(self,
             st=np.array([0, 0, 0, 0, 0, 0]),
             con=np.array([6, np.deg2rad(70)]),
             n=5):
        for i in range(n):
            change = self.changeInStates(st, con)
            new = st + change
            print(
                "X     %10.2f                   =dX     %10.2f  |X     %10.2f"
                % (     st[0],                       change[0],        new[0]))
            print(
                "Y     %10.2f                   =dY     %10.2f  |Y     %10.2f"
                % (     st[1],                       change[1],        new[1]))
            print(
                "psi   %10.2f + acc %5.2f       =dpsi   %10.2f  |psi   %10.2f"
                % (     st[2],     con[0],           change[2],        new[2]))
            print(
                "vx    %10.2f + del %5.2f       =dvx    %10.2f  |vx    %10.2f"
                % (     st[3],     con[1],           change[3],        new[3]))
            print(
                "vy    %10.2f                   =dvy    %10.2f  |vy    %10.2f"
                % (     st[4],                       change[4],        new[4]))
            print(
                "omega %10.2f                   =domega %10.2f  |omega %10.2f"
                % (     st[5],                       change[5],        new[5]))
            print()
            st = new
