# -*- coding: utf-8 -*-
#
# Copyright (c) 2021 by Mikael Johansson, Erik Berglund, Pedro Roque
#
# Simple functionality for invariant sets and control invariant sets based on the Polytope library

import polytope as pc
import numpy as np
import matplotlib.pyplot as plt
from control.matlab import dare


class SetOperations(object):

    def __init__(self, A, B, Q, R, xlb, xub):
        """
        Control sets operation class.

        :param A: system state transition matrix
        :type A: np.ndarray
        :param B: system control matrix
        :type B: np.ndarray
        :param Q: states weight matrix
        :type Q: np.ndarray
        :param R: control weight matrix
        :type R: np.ndarray
        :param xub: upper bound on set constraints set
        :type xub: np.ndarray
        :param xlb: lower bound on set constraints set
        :type xlb: np.ndarray
        """

        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        Cub = np.eye(len(xub))
        Clb = -1 * np.eye(len(xlb))
        Cbu = xub
        Cbl = -1 * xlb
        Cb = np.concatenate((Cbu, Cbl), axis=0)
        C = np.concatenate((Cub, Clb), axis=0)
        self.Cx = pc.Polytope(C, Cb)
        pass

    def preset(self, A, Cx):
        """
        Preset of set Cx with the transfer matrix A

        :param A: transfer matrix A
        :type A: np.ndarray
        :param Cx: polytope set
        :type Cx: Polytope
        :return: pre-set of Cx
        :rtype: Polytope
        """
        CxA = Cx.A
        Cxb = Cx.b
        return pc.Polytope(CxA @ A, Cxb)

    def oneStepControllableSet(self, Cx, Cu):
        """
        Calculate the one-step controllable set to Cx given control constraints in Cu

        :param Cx: target set
        :type Cx: Polytope
        :param Cu: control constraints set
        :type Cu: Polytope
        :return: one-step controllable set
        :rtype: Polytope
        """
        CxA = Cx.A
        Cxb = Cx.b
        CuA = Cu.A
        Cub = Cu.b
        C = np.block([[CxA @ self.A, CxA @ self.B], [np.zeros((Cub.shape[0], self.A.shape[1])), CuA]])
        c = np.concatenate((Cxb, Cub))
        xdims = [i + 1 for i in range(self.A.shape[1])]
        P = pc.projection(pc.Polytope(C, c), xdims, solver="fm")
        return P

    def invSet(self, A, Cx):
        """
        Calculate the invariant set for the system x_t+1 = A x_t
        with state constraints given by Cx.

        :param A: state transition matrix
        :type A: np.ndarray
        :param Cx: state constraints set
        :type Cx: Polytope
        :return: invariant set for the dynamics of x_t+1 = A x_t
        :rtype: Polytope
        """

        converged = False
        S = Cx
        dIdx = 0
        while not converged:
            Snew = S.intersect(self.preset(A, S))
            if Snew == S:
                converged = True
            else:
                dIdx += 1
            S = Snew
        print(f'Converged with determinedness index {dIdx}.\n')
        return S

    def ctrlInvSet(self, UlimSet):
        """
        Calculate the control invariant set for the system under analysis
        considering the control constraints in UlimSet

        :param UlimSet: control constraints polytope
        :type UlimSet: Polytope
        :return: controllable invariant set
        :rtype: Polytope
        """
        converged = False
        S = self.Cx
        Cu = UlimSet
        dIdx = 0
        while not converged:
            Snew = S.intersect(self.oneStepControllableSet(self.A, self.B, S, Cu))
            if Snew == S:
                converged = True
            else:
                dIdx += 1
            S = Snew
        print(f'Converged with determinedness index {dIdx}.\n')
        return S

    def LQRSet(self, UlimSet):
        """
        Calculate the LQR terminal set for the system under analysis

        :param UlimSet: control constraint set
        :type UlimSet: Polytope
        :return: LQR invariant set
        :rtype: Polytope
        """
        (P, E, L) = dare(self.A, self.B, self.Q, self.R)
        L = np.array(L)
        CxA = self.Cx.A
        Cxb = self.Cx.b
        CuA = UlimSet.A
        Cub = UlimSet.b
        C = np.block([[CxA], [CuA @ (-L)]])
        c = np.concatenate((Cxb, Cub)).reshape(-1, 1)
        xCstrSetClosedLoop = pc.Polytope(C, c)
        AClosedLoop = self.A - self.B @ L
        return self.invSet(AClosedLoop, xCstrSetClosedLoop)

    def zeroSet(self):
        """
        Get the zero-set.
        """
        eye_mat = np.eye(self.A.shape[1])
        Cb = np.ones((self.A.shape[0] * 2, 1)) * 1e-6
        Cx = np.concatenate((eye_mat, -1 * eye_mat), axis=0)
        zeroSet = pc.Polytope(Cx, Cb)
        return zeroSet

    def getNstepControllableSet(self, uub, ulb, N, inv_set_type="LQR"):
        """
        Get the N-step controllable set for the system in this class.

        :param uub: control upper bounds
        :type uub: np.ndarray
        :param ulb: control lower bounds
        :type ulb: np.ndarray
        :param N: receding horizon steps
        :type N: integer
        :param inv_set_type: type of final invariant set, defaults to "LQR"
        :type inv_set_type: str, optional
        :return: N-step invariant set, intermediate invariant sets
        :rtype: Polytope, dict with Polytopes
        """

        # Create Polytope with uub and ulb
        Cub = np.eye(len(uub))
        Clb = -1 * np.eye(len(ulb))
        Cbu = uub
        Cbl = -1 * ulb
        Cb = np.concatenate((Cbu, Cbl), axis=0)
        C = np.concatenate((Cub, Clb), axis=0)
        Cu = pc.Polytope(C, Cb)

        # Check which invariant set type we want
        if inv_set_type == "zero":
            # Create a zero invariant set (epsilon small for stability)
            Xf = self.zeroSet()
        elif inv_set_type == "LQR":
            # Create the LQR invariant set
            Xf = self.LQRSet(Cu)
        else:
            print("Wrong argument 'inv_set_type'. Available methods are: 'zero', 'LQR'")
            exit()

        # Get the backwards reachable set to the designed terminal set
        temp_reach_set = {str(N): Xf}
        for i in range(N):
            backwards_step = self.oneStepControllableSet(temp_reach_set[str(N - i)], Cu)
            backwards_step_Cx = backwards_step.intersect(self.Cx)
            temp_reach_set[str(N - i - 1)] = backwards_step_Cx

        # Debug
        kns = temp_reach_set['0']

        return temp_reach_set['0'], temp_reach_set, Xf

    def plotNsets(self, set_dict={}, plotU=False, plot_type="translation"):
        """
        Plot the dictionary sets with the provided keys

        :param set_dict: [description], defaults to {}
        :type set_dict: dict, optional
        """
        fig, ax = plt.subplots(1)
        ax.set_title("Invariant Sets and Control Bounds")
        dict_keys = list(set_dict.keys())
        dict_keys.reverse()
        for key in dict_keys:
            Kni = set_dict[key]
            Kni.project([1, 2]).plot(ax=ax, alpha=1, linestyle="-", linewidth=0.3)
        if not plotU:
            dict_keys.reverse()
            ax.set_title("N-step Invariant Sets")
            dict_keys = ['N = ' + key for key in dict_keys]  # Appending N = to keys
            dict_keys = ['Xf' if key == "N = 0" else key for key in dict_keys]  # Replacing N = 0 with Xf
        ax.legend(dict_keys)
        if plot_type == "translation":
            ax.set_xlabel("Position X [m]")
            ax.set_ylabel("Position Y [m]")
            ax.set_xlim(-1.5, 1.5)
            ax.set_ylim(-.15, .15)
        elif plot_type == "attitude":
            ax.set_xlabel("Roll [rad]")
            ax.set_ylabel("Pitch [rad]")
            ax.set_xlim(-.25, .25)
            ax.set_ylim(-.25, .25)
        plt.show()