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
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>

#include "../include/hpipm_d_ocp_qp_dim.h"
#include "../include/hpipm_d_ocp_qp.h"
#include "../include/hpipm_d_ocp_qp_sol.h"
#include "../include/hpipm_d_dense_qp.h"
#include "../include/hpipm_d_dense_qp_sol.h"
#include "../include/hpipm_d_cond.h"
#include "../include/hpipm_d_part_cond.h"
#include "../include/hpipm_d_cond_aux.h"



#define COND_B d_cond_b
#define COND_BABT d_cond_BAbt
#define COND_BAT d_cond_BAt
#define COND_D d_cond_d
#define COND_DCT d_cond_DCt
#define COND_DCTD d_cond_DCtd
#define COND_RQ d_cond_rq
#define COND_RSQ d_cond_RSQ
#define COND_RSQRQ d_cond_RSQrq
#define COND_QP_ARG d_cond_qp_arg
#define COND_QP_ARG_CREATE d_cond_qp_arg_create
#define COND_QP_ARG_MEMSIZE d_cond_qp_arg_memsize
#define COND_QP_ARG_SET_COMP_DUAL_SOL_EQ d_cond_qp_arg_set_comp_dual_sol_eq
#define COND_QP_ARG_SET_COMP_DUAL_SOL_INEQ d_cond_qp_arg_set_comp_dual_sol_ineq
#define COND_QP_ARG_SET_COMP_PRIM_SOL d_cond_qp_arg_set_comp_prim_sol
#define COND_QP_ARG_SET_COND_LAST_STAGE d_cond_qp_arg_set_cond_last_stage
#define COND_QP_ARG_SET_DEFAULT d_cond_qp_arg_set_default
#define COND_QP_ARG_SET_RIC_ALG d_cond_qp_arg_set_ric_alg
#define COND_QP_ARG_WS d_cond_qp_ws
#define COND_QP_WS_CREATE d_cond_qp_ws_create
#define COND_QP_WS_MEMSIZE d_cond_qp_ws_memsize
#define CREATE_STRVEC blasfeo_create_dvec
#define DENSE_QP d_dense_qp
#define DENSE_QP_SOL d_dense_qp_sol
#define EXPAND_SOL d_expand_sol
#define GECP_LIBSTR blasfeo_dgecp
#define OCP_QP d_ocp_qp
#define OCP_QP_DIM d_ocp_qp_dim
#define OCP_QP_SOL d_ocp_qp_sol
#define PART_COND_QP_ARG d_part_cond_qp_arg
#define PART_COND_QP_WS d_part_cond_qp_ws
#define STRVEC blasfeo_dvec
#define UPDATE_COND_BABT d_update_cond_BAbt
#define UPDATE_COND_DCTD d_update_cond_DCtd
#define UPDATE_COND_RSQRQ_N2NX3 d_update_cond_RSQrq_N2nx3
#define VECCP_LIBSTR blasfeo_dveccp

#define PART_COND_QP_COMPUTE_BLOCK_SIZE d_part_cond_qp_compute_block_size
#define PART_COND_QP_COMPUTE_DIM d_part_cond_qp_compute_dim
#define PART_COND_QP_ARG_MEMSIZE d_part_cond_qp_arg_memsize
#define PART_COND_QP_ARG_CREATE d_part_cond_qp_arg_create
#define PART_COND_QP_ARG_SET_DEFAULT d_part_cond_qp_arg_set_default
#define PART_COND_QP_ARG_SET_RIC_ALG d_part_cond_qp_arg_set_ric_alg
#define PART_COND_QP_ARG_SET_COMP_PRIM_SOL d_part_cond_qp_arg_set_comp_prim_sol
#define PART_COND_QP_ARG_SET_COMP_DUAL_SOL_EQ d_part_cond_qp_arg_set_comp_dual_sol_eq
#define PART_COND_QP_ARG_SET_COMP_DUAL_SOL_INEQ d_part_cond_qp_arg_set_comp_dual_sol_ineq
#define PART_COND_QP_WS_MEMSIZE d_part_cond_qp_ws_memsize
#define PART_COND_QP_WS_CREATE d_part_cond_qp_ws_create
#define PART_COND_QP_COND d_part_cond_qp_cond
#define PART_COND_QP_COND_LHS d_part_cond_qp_cond_lhs
#define PART_COND_QP_COND_RHS d_part_cond_qp_cond_rhs
#define PART_COND_QP_EXPAND_SOL d_part_cond_qp_expand_sol
#define PART_COND_QP_UPDATE d_part_cond_qp_update



#include "x_part_cond.c"
