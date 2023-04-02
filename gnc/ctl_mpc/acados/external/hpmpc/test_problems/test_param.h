/**************************************************************************************************
*                                                                                                 *
* This file is part of HPMPC.                                                                     *
*                                                                                                 *
* HPMPC -- Library for High-Performance implementation of solvers for MPC.                        *
* Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                *
*                                                                                                 *
* HPMPC is free software; you can redistribute it and/or                                          *
* modify it under the terms of the GNU Lesser General Public                                      *
* License as published by the Free Software Foundation; either                                    *
* version 2.1 of the License, or (at your option) any later version.                              *
*                                                                                                 *
* HPMPC is distributed in the hope that it will be useful,                                        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  *
*                                                                                                 *
* Author: Gianluca Frison, giaf (at) dtu.dk                                                       *
*                                                                                                 *
**************************************************************************************************/

// processor maximum frequency (for result printing purposes only)
//#define GHZ_MAX 3.6 // 2.3 2.9 3.4 3.6

// problem size (states, inputs, horizon)
//#define NX 30
//#define NU 14
//#define NN 10

// number of inequality constraints
//#define NL 0
//#define NB NU+NX

// number of repetition for timing
//#define NREP 1000

// print results
//#define PRINTRES 1

// print statistics
//#define PRINTSTAT 1

// free initial state: 0 mpc, 1 mhe
//#define FREE_X0 0

// ip method: 1 primal-dual, 2 predictor-corrector primal-dual
//#define IP 2

// compute lagrangian multipliers
//#define COMPUTE_MULT 1

// compute lagrangian multipliers
//#define COMPUTE_MULT_ADMM 0

// warm-start with user-provided solution (otherwise initialize x and u with 0 or something feasible)
//#define WARM_START 0

// double/single/mixed ('d'/'s'/'m') precision
//#define PREC 'd'

// number of iterations of IP method
//#define K_MAX 50

// number of iterations of ADMM method
//#define K_MAX_ADMM 2000

// tolerance in the duality measure
//#define MU_TOL 1e-6

// minimum accepted step length
//#define ALPHA_MIN 1e-8

// threshold in the duality measure to switch from single to double precision
//#define SP_THR 1e5

// duality measure
//#define TOL 1e-6


//// solver for LQ control problem: 1 kernel_single, 2 kernel_multiple, 3 BLAS
//#define LQS 1

//// compute lagrangian multiplier pi TODO not implemented, set to 0 for now
//#define PI 0

//// force double precision (exact IP)
//#define DP 1

//// choose mixed precision (instead of double) in the inexact IP
//#define MP 1

//// number of iterative refinement steps in mixed precision
//#define IT_REF 1




