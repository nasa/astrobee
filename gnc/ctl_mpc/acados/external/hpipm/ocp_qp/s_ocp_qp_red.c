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



#include <stdio.h>
#include <stdlib.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_s_blas.h>

#include <hpipm_s_ocp_qp_dim.h>
#include <hpipm_s_ocp_qp.h>
#include <hpipm_s_ocp_qp_sol.h>
#include <hpipm_s_ocp_qp_red.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>



#define SINGLE_PRECISION



#define AXPY blasfeo_saxpy
#define CREATE_STRVEC blasfeo_create_svec
#define GECP blasfeo_sgecp
#define GEMV_N blasfeo_sgemv_n
#define GEMV_T blasfeo_sgemv_t
#define OCP_QP s_ocp_qp
#define OCP_QP_SOL s_ocp_qp_sol
#define OCP_QP_DIM s_ocp_qp_dim
#define OCP_QP_REDUCE_EQ_DOF_ARG s_ocp_qp_reduce_eq_dof_arg
#define OCP_QP_REDUCE_EQ_DOF_WS s_ocp_qp_reduce_eq_dof_ws
#define MATEL BLASFEO_SMATEL
#define REAL float
#define SIZE_STRVEC blasfeo_memsize_svec
#define STRVEC blasfeo_svec
#define SYMV_L blasfeo_ssymv_l
#define VECAD_SP blasfeo_svecad_sp
#define VECCP blasfeo_sveccp
#define VECEL BLASFEO_SVECEL
#define VECSE blasfeo_svecse

#define OCP_QP_DIM_REDUCE_EQ_DOF s_ocp_qp_dim_reduce_eq_dof
#define OCP_QP_REDUCE_EQ_DOF_ARG_MEMSIZE s_ocp_qp_reduce_eq_dof_arg_memsize
#define OCP_QP_REDUCE_EQ_DOF_ARG_CREATE s_ocp_qp_reduce_eq_dof_arg_create
#define OCP_QP_REDUCE_EQ_DOF_ARG_SET_DEFAULT s_ocp_qp_reduce_eq_dof_arg_set_default
#define OCP_QP_REDUCE_EQ_DOF_ARG_SET_ALIAS_UNCHANGED s_ocp_qp_reduce_eq_dof_arg_set_alias_unchanged
#define OCP_QP_REDUCE_EQ_DOF_ARG_SET_COMP_PRIM_SOL s_ocp_qp_reduce_eq_dof_arg_set_comp_prim_sol
#define OCP_QP_REDUCE_EQ_DOF_ARG_SET_COMP_DUAL_SOL_EQ s_ocp_qp_reduce_eq_dof_arg_set_comp_dual_sol_eq
#define OCP_QP_REDUCE_EQ_DOF_ARG_SET_COMP_DUAL_SOL_INEQ s_ocp_qp_reduce_eq_dof_arg_set_comp_dual_sol_ineq
#define OCP_QP_REDUCE_EQ_DOF_WS_MEMSIZE s_ocp_qp_reduce_eq_dof_ws_memsize
#define OCP_QP_REDUCE_EQ_DOF_WS_CREATE s_ocp_qp_reduce_eq_dof_ws_create
#define OCP_QP_REDUCE_EQ_DOF s_ocp_qp_reduce_eq_dof
#define OCP_QP_REDUCE_EQ_DOF_LHS s_ocp_qp_reduce_eq_dof_lhs
#define OCP_QP_REDUCE_EQ_DOF_RHS s_ocp_qp_reduce_eq_dof_rhs
#define OCP_QP_RESTORE_EQ_DOF s_ocp_qp_restore_eq_dof




#include "x_ocp_qp_red.c"


