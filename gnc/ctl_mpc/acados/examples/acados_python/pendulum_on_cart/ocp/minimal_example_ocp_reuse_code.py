# -*- coding: future_fstrings -*-
# Minimal example showing how to reuse the exported c-code with
# different time-steps.
#
# There are two use-cases demonstrated here. One use-case is to change
# the length of the time-stamp vector (this results in a different
# N). Another use-case is to change the final time but keep the number
# of shooting nodes identical. Reusing the exported code with variing
# N can be useful especially in a c-only application where the process
# of code-generation should only be done once.
#
# This example is an extension of the 'minimal_example_ocp.py' example.

#
# Copyright 2021 Markus Schwienbacher, Gianluca Frison, Dimitris Kouzoupis,
# Robin Verschueren, Andrea Zanelli, Niels van Duijkeren, Jonathan Frey,
# Tommaso Sartor, Branimir Novoselnik, Rien Quirynen, Rezart Qelibari,
# Dang Doan, Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf,
# Moritz Diehl
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
import os
import sys

sys.path.insert(0, '../common')

from acados_template import AcadosOcp, AcadosOcpSolver
from pendulum_model import export_pendulum_ode_model
import numpy as np
import scipy.linalg
from utils import plot_pendulum

print('This example demonstrates 2 use-cases for reuse of the code export.')

# create ocp object to formulate the OCP
ocp = AcadosOcp()

# set model
model = export_pendulum_ode_model()
ocp.model = model

nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

# define the different options for the use-case demonstration
N0 = 20  # original number of shooting nodes
N12 = 15  # change the number of shooting nodes for use-cases 1 and 2
condN12 = max(1, round(N12/1)) # change the number of cond_N for use-cases 1 and 2 (for PARTIAL_* solvers only)
Tf_01 = 1.0  # original final time and for use-case 1
Tf_2 = Tf_01 * 0.7  # change final time for use-case 2 (but keep N identical)

# set dimensions
ocp.dims.N = N0

# set cost
Q = 2 * np.diag([1e3, 1e3, 1e-2, 1e-2])
R = 2 * np.diag([1e-2])

ocp.cost.W_e = Q
ocp.cost.W = scipy.linalg.block_diag(Q, R)

ocp.cost.cost_type = 'LINEAR_LS'
ocp.cost.cost_type_e = 'LINEAR_LS'

ocp.cost.Vx = np.zeros((ny, nx))
ocp.cost.Vx[:nx, :nx] = np.eye(nx)

Vu = np.zeros((ny, nu))
Vu[4, 0] = 1.0
ocp.cost.Vu = Vu

ocp.cost.Vx_e = np.eye(nx)

ocp.cost.yref = np.zeros((ny,))
ocp.cost.yref_e = np.zeros((ny_e,))

# set constraints
Fmax = 80
ocp.constraints.lbu = np.array([-Fmax])
ocp.constraints.ubu = np.array([+Fmax])
ocp.constraints.idxbu = np.array([0])

ocp.constraints.x0 = np.array([0.0, np.pi, 0.0, 0.0])

# set options
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # FULL_CONDENSING_QPOASES
# PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
# PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
# ocp.solver_options.print_level = 1
ocp.solver_options.nlp_solver_type = 'SQP'  # SQP_RTI, SQP

# set prediction horizon
ocp.solver_options.tf = Tf_01

print(80*'-')
print('generate code and compile...')
ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

# --------------------------------------------------------------------------------
# 0) solve the problem defined here (original from code export), analog to 'minimal_example_ocp.py'

simX0 = np.ndarray((N0 + 1, nx))
simU0 = np.ndarray((N0, nu))

print(80*'-')
print(f'solve original code with N = {N0} and Tf = {Tf_01} s:')
status = ocp_solver.solve()

if status != 0:
    ocp_solver.print_statistics()  # encapsulates: stat = ocp_solver.get_stats("statistics")
    raise Exception(f'acados returned status {status}.')

# get solution
for i in range(N0):
    simX0[i, :] = ocp_solver.get(i, "x")
    simU0[i, :] = ocp_solver.get(i, "u")
simX0[N0, :] = ocp_solver.get(N0, "x")

ocp_solver.print_statistics()  # encapsulates: stat = ocp_solver.get_stats("statistics")

# plot but don't halt
plot_pendulum(np.linspace(0, Tf_01, N0 + 1), Fmax, simU0, simX0, latexify=False, plt_show=False, X_true_label=f'original: N={N0}, Tf={Tf_01}')

# --------------------------------------------------------------------------------
# 1) now reuse the code but set a new time-steps vector, with a new number of elements
dt1 = Tf_01 / N12

new_time_steps1 = np.tile(dt1, (N12,))  # Matlab's equivalent to repmat
time1 = np.hstack([0, np.cumsum(new_time_steps1)])

simX1 = np.ndarray((N12 + 1, nx))
simU1 = np.ndarray((N12, nu))

ocp_solver.set_new_time_steps(new_time_steps1)
print(80*'-')
if ocp.solver_options.qp_solver.startswith('PARTIAL'):
    ocp_solver.update_qp_solver_cond_N(condN12)
    print(f'solve use-case 2 with N = {N12}, cond_N = {condN12} and Tf = {Tf_01} s (instead of {Tf_01} s):')
    X_true_label = f'use-case 1: N={N12}, N_cond = {condN12}'
else:
    print(f'solve use-case 2 with N = {N12} and Tf = {Tf_01} s (instead of {Tf_01} s):')
    X_true_label = f'use-case 1: N={N12}'

status = ocp_solver.solve()

if status != 0:
    ocp_solver.print_statistics()  # encapsulates: stat = ocp_solver.get_stats("statistics")
    raise Exception(f'acados returned status {status}.')

# get solution
for i in range(N12):
    simX1[i, :] = ocp_solver.get(i, "x")
    simU1[i, :] = ocp_solver.get(i, "u")
simX1[N12, :] = ocp_solver.get(N12, "x")

ocp_solver.print_statistics()  # encapsulates: stat = ocp_solver.get_stats("statistics")



plot_pendulum(time1, Fmax, simU1, simX1, latexify=False, plt_show=False, X_true_label=X_true_label)

# --------------------------------------------------------------------------------
# 2) reuse the code again, set a new time-steps vector, only with a different final time
dt2 = Tf_2 / N12

new_time_steps2 = np.tile(dt2, (N12,))  # Matlab's equivalent to repmat
time2 = np.hstack([0, np.cumsum(new_time_steps2)])

simX2 = np.ndarray((N12 + 1, nx))
simU2 = np.ndarray((N12, nu))

ocp_solver.set_new_time_steps(new_time_steps2)
print(80*'-')
if ocp.solver_options.qp_solver.startswith('PARTIAL'):
    ocp_solver.update_qp_solver_cond_N(condN12)
    print(f'solve use-case 2 with N = {N12}, cond_N = {condN12} and Tf = {Tf_2} s (instead of {Tf_01} s):')
else:
    print(f'solve use-case 2 with N = {N12} and Tf = {Tf_2} s (instead of {Tf_01} s):')
status = ocp_solver.solve()

if status != 0:
    ocp_solver.print_statistics()  # encapsulates: stat = ocp_solver.get_stats("statistics")
    raise Exception(f'acados returned status {status}.')

# get solution
for i in range(N12):
    simX2[i, :] = ocp_solver.get(i, "x")
    simU2[i, :] = ocp_solver.get(i, "u")
simX2[N12, :] = ocp_solver.get(N12, "x")

ocp_solver.print_statistics()  # encapsulates: stat = ocp_solver.get_stats("statistics")

plot_pendulum(time2, Fmax, simU2, simX2, latexify=False, plt_show=True, X_true_label=f'use-case 2: Tf={Tf_2} s')
