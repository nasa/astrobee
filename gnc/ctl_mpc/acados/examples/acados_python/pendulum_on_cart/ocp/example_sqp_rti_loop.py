#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

import sys
sys.path.insert(0, '../common')

from acados_template import AcadosOcp, AcadosOcpSolver
from pendulum_model import export_pendulum_ode_model
import numpy as np
import scipy.linalg
from utils import plot_pendulum

def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_pendulum_ode_model()
    ocp.model = model

    Tf = 1.0
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx
    N = 20

    # set dimensions
    ocp.dims.N = N

    # set cost
    Q = 2*np.diag([1e3, 1e3, 1e-2, 1e-2])
    R = 2*np.diag([1e-2])

    ocp.cost.W_e = Q
    ocp.cost.W = scipy.linalg.block_diag(Q, R)

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx,:nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[4,0] = 1.0
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref  = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((ny_e, ))

    # set constraints

    # bound on u
    Fmax = 80
    ocp.constraints.lbu = np.array([-Fmax])
    ocp.constraints.ubu = np.array([+Fmax])
    ocp.constraints.idxbu = np.array([0])

    # duplicated bound on u as a linear constraint
    ocp.constraints.C = np.zeros((1, nx))
    ocp.constraints.D = np.zeros((1, nu))
    ocp.constraints.D[0, 0] = 1.0
    ocp.constraints.lg = np.array([-Fmax])
    ocp.constraints.ug = np.array([+Fmax])

    ocp.constraints.x0 = np.array([0.0, np.pi, 0.0, 0.0])

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = Tf

    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))


    # call SQP_RTI solver in the loop:
    tol = 1e-6

    for i in range(20):
        status = ocp_solver.solve()
        # ocp_solver.custom_update(np.array([]))
        ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")
        residuals = ocp_solver.get_residuals()
        print("residuals after ", i, "SQP_RTI iterations:\n", residuals)
        if max(residuals) < tol:
            break


    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N):
        simX[i,:] = ocp_solver.get(i, "x")
        simU[i,:] = ocp_solver.get(i, "u")
    simX[N,:] = ocp_solver.get(N, "x")

    ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    #
    cost = ocp_solver.get_cost()
    print("cost function value of solution = ", cost)

    PRINT_QP = True

    range_without_terminal = [0, 1, N-2, N-1]
    range_with_terminal = [0, 1, N-1, N]
    if PRINT_QP:
        # dynamics
        for i in range_without_terminal:
            A_qp = ocp_solver.get_from_qp_in(i, "A")
            print(f"qp: A at stage {i}: {A_qp}")

        for i in range_without_terminal:
            B_qp = ocp_solver.get_from_qp_in(i, "B")
            print(f"qp: B at stage {i}: {B_qp}")

        for i in range_without_terminal:
            b_qp = ocp_solver.get_from_qp_in(i, "b")
            print(f"qp: b at stage {i}: {b_qp}")

        # cost
        for i in range_with_terminal:
            Q_qp = ocp_solver.get_from_qp_in(i, "Q")
            print(f"qp: Q at stage {i}: {Q_qp}")

        for i in range_with_terminal:
            R_qp = ocp_solver.get_from_qp_in(i, "R")
            print(f"qp: R at stage {i}: {R_qp}")

        for i in range_with_terminal:
            S_qp = ocp_solver.get_from_qp_in(i, "S")
            print(f"qp: S at stage {i}: {S_qp}")

        for i in range_with_terminal:
            r_qp = ocp_solver.get_from_qp_in(i, "r")
            print(f"qp: r at stage {i}: {r_qp}")

        for i in range_with_terminal:
            q_qp = ocp_solver.get_from_qp_in(i, "q")
            print(f"qp: q at stage {i}: {q_qp}")

        # constraints
        for i in range_with_terminal:
            C_qp = ocp_solver.get_from_qp_in(i, "C")
            print(f"qp: C at stage {i}: {C_qp}")
        for i in range_with_terminal:
            D_qp = ocp_solver.get_from_qp_in(i, "D")
            print(f"qp: D at stage {i}: {D_qp}")

    # plot
    ocp_solver.dump_last_qp_to_json(overwrite=True)
    plot_pendulum(np.linspace(0, Tf, N+1), Fmax, simU, simX, latexify=False)


if __name__ == "__main__":
    main()