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
#include <blasfeo_s_blas.h>
#include <blasfeo_s_aux.h>

#include <hpipm_s_ocp_qp_dim.h>
#include <hpipm_s_ocp_qp.h>
#include <hpipm_s_ocp_qp_sol.h>
#include <hpipm_s_dense_qp.h>
#include <hpipm_s_dense_qp_sol.h>
#include <hpipm_s_cond.h>
#include <hpipm_s_cond_aux.h>
#include <hpipm_aux_mem.h>



#define COND_DCT s_cond_DCt
#define COND_DCTD s_cond_DCtd
#define COND_D s_cond_d
#define COND_B s_cond_b
#define COND_BABT s_cond_BAbt
#define COND_BAT s_cond_BAt
#define COND_RQ s_cond_rq
#define COND_RSQ s_cond_RSQ
#define COND_RSQRQ s_cond_RSQrq
#define COND_QP_ARG s_cond_qp_arg
#define COND_QP_WS s_cond_qp_ws
#define CREATE_STRMAT blasfeo_create_smat
#define CREATE_STRVEC blasfeo_create_svec
#define DENSE_QP s_dense_qp
#define DENSE_QP_DIM s_dense_qp_dim
#define DENSE_QP_SOL s_dense_qp_sol
#define EXPAND_SOL s_expand_sol
#define EXPAND_PRIMAL_SOL s_expand_primal_sol
#define OCP_QP s_ocp_qp
#define OCP_QP_DIM s_ocp_qp_dim
#define OCP_QP_SOL s_ocp_qp_sol
#define REAL float
#define SIZE_STRMAT blasfeo_memsize_smat
#define SIZE_STRVEC blasfeo_memsize_svec
#define STRMAT blasfeo_smat
#define STRVEC blasfeo_svec
#define UPDATE_COND_DCTD s_update_cond_DCtd
#define UPDATE_COND_BABT s_update_cond_BAbt
#define UPDATE_COND_RSQRQ_N2NX3 s_update_cond_RSQrq_N2nx3

#define COND_QP_COMPUTE_DIM s_cond_qp_compute_dim
#define COND_QP_ARG_MEMSIZE s_cond_qp_arg_memsize
#define COND_QP_ARG_CREATE s_cond_qp_arg_create
#define COND_QP_ARG_SET_DEFAULT s_cond_qp_arg_set_default
#define COND_QP_ARG_SET_COND_ALG s_cond_qp_arg_set_cond_alg
#define COND_QP_ARG_SET_RIC_ALG s_cond_qp_arg_set_ric_alg
#define COND_QP_ARG_SET_COND_LAST_STAGE s_cond_qp_arg_set_cond_last_stage
#define COND_QP_ARG_SET_COMP_PRIM_SOL s_cond_qp_arg_set_comp_prim_sol
#define COND_QP_ARG_SET_COMP_DUAL_SOL_EQ s_cond_qp_arg_set_comp_dual_sol_eq
#define COND_QP_ARG_SET_COMP_DUAL_SOL_INEQ s_cond_qp_arg_set_comp_dual_sol_ineq
#define COND_QP_WS_MEMSIZE s_cond_qp_ws_memsize
#define COND_QP_WS_CREATE s_cond_qp_ws_create
#define COND_QP_COND s_cond_qp_cond
#define COND_QP_COND_LHS s_cond_qp_cond_lhs
#define COND_QP_COND_RHS s_cond_qp_cond_rhs
#define COND_QP_EXPAND_SOL s_cond_qp_expand_sol
#define COND_QP_EXPAND_PRIMAL_SOL s_cond_qp_expand_primal_sol
#define COND_QP_UPDATE s_cond_qp_update



#include "x_cond.c"

