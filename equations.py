import casadi as ca


class VehicleModel():
    def __init__(self):
        # smallest possible positive float value
        spv = 2.2250738585072014e-308

        m, Iz, lf, lr, caf, car = 1500,150,1.5,1.6,20,20

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
