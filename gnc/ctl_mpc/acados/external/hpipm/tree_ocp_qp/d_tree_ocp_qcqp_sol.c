/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include <stdlib.h>
#include <stdio.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>

#include <hpipm_tree.h>
#include <hpipm_scenario_tree.h>
#include <hpipm_d_tree_ocp_qcqp.h>
#include <hpipm_d_tree_ocp_qcqp_sol.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>



#define CREATE_STRVEC blasfeo_create_dvec
#define UNPACK_VEC blasfeo_unpack_dvec
#define REAL double
#define SIZE_STRVEC blasfeo_memsize_dvec
#define STRVEC blasfeo_dvec
#define TREE_OCP_QCQP d_tree_ocp_qcqp
#define TREE_OCP_QCQP_DIM d_tree_ocp_qcqp_dim
#define TREE_OCP_QCQP_SOL d_tree_ocp_qcqp_sol

#define TREE_OCP_QCQP_SOL_MEMSIZE d_tree_ocp_qcqp_sol_memsize
#define TREE_OCP_QCQP_SOL_CREATE d_tree_ocp_qcqp_sol_create
#define TREE_OCP_QCQP_SOL_GET_ALL d_tree_ocp_qcqp_sol_get_all
#define TREE_OCP_QCQP_SOL_GET d_tree_ocp_qcqp_sol_get
#define TREE_OCP_QCQP_SOL_GET_U d_tree_ocp_qcqp_sol_get_u
#define TREE_OCP_QCQP_SOL_GET_X d_tree_ocp_qcqp_sol_get_x
#define TREE_OCP_QCQP_SOL_GET_SL d_tree_ocp_qcqp_sol_get_sl
#define TREE_OCP_QCQP_SOL_GET_SU d_tree_ocp_qcqp_sol_get_su
#define TREE_OCP_QCQP_SOL_GET_PI d_tree_ocp_qcqp_sol_get_pi
#define TREE_OCP_QCQP_SOL_GET_LAM_LB d_tree_ocp_qcqp_sol_get_lam_lb
#define TREE_OCP_QCQP_SOL_GET_LAM_UB d_tree_ocp_qcqp_sol_get_lam_ub
#define TREE_OCP_QCQP_SOL_GET_LAM_LG d_tree_ocp_qcqp_sol_get_lam_lg
#define TREE_OCP_QCQP_SOL_GET_LAM_UG d_tree_ocp_qcqp_sol_get_lam_ug



#include "x_tree_ocp_qcqp_sol.c"

