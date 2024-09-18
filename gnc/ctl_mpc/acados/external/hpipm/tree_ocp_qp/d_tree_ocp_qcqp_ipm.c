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
#ifdef USE_C99_MATH
#include <math.h>
#endif

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include <hpipm_tree.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>
#include <hpipm_d_core_qp_ipm.h>
#include <hpipm_d_core_qp_ipm_aux.h>
#include <hpipm_d_tree_ocp_qp_dim.h>
#include <hpipm_d_tree_ocp_qp.h>
#include <hpipm_d_tree_ocp_qp_sol.h>
#include <hpipm_d_tree_ocp_qp_res.h>
#include <hpipm_d_tree_ocp_qp_ipm.h>
#include <hpipm_d_tree_ocp_qp_kkt.h>
//#include <hpipm_d_tree_ocp_qp_utils.h>
#include <hpipm_d_tree_ocp_qcqp_dim.h>
#include <hpipm_d_tree_ocp_qcqp.h>
#include <hpipm_d_tree_ocp_qcqp_sol.h>
#include <hpipm_d_tree_ocp_qcqp_res.h>
#include <hpipm_d_tree_ocp_qcqp_ipm.h>
//#include <hpipm_d_tree_ocp_qcqp_utils.h>



#define DOUBLE_PRECISION
#define BLASFEO_VECEL BLASFEO_DVECEL
#define BLASFEO_MATEL BLASFEO_DMATEL



#define AXPY blasfeo_daxpy
#define AXPBY blasfeo_daxpby
#define BACKUP_RES_M d_backup_res_m
#define COLEX blasfeo_dcolex
#define COMPUTE_ALPHA_QCQP d_compute_alpha_qcqp
#define COMPUTE_CENTERING_CORRECTION_QCQP d_compute_centering_correction_qcqp
#define COMPUTE_CENTERING_QCQP d_compute_centering_qcqp
#define COMPUTE_MU_AFF_QCQP d_compute_mu_aff_qcqp
#define COLIN blasfeo_dcolin
#define CORE_QP_IPM_WORKSPACE d_core_qp_ipm_workspace
//#define CREATE_CORE_QP_IPM d_create_core_qp_ipm
#define CREATE_STRMAT blasfeo_create_dmat
#define CREATE_STRVEC blasfeo_create_dvec
#define DOT blasfeo_ddot
#define TREE_OCP_QP_FACT_LQ_SOLVE_KKT_STEP d_tree_ocp_qp_fact_lq_solve_kkt_step
#define TREE_OCP_QP_FACT_SOLVE_LU_KKT_STEP d_tree_ocp_qp_fact_solve_lu_kkt_step
#define TREE_OCP_QP_FACT_SOLVE_KKT_STEP d_tree_ocp_qp_fact_solve_kkt_step
#define TREE_OCP_QP_FACT_SOLVE_KKT_UNCONSTR d_tree_ocp_qp_fact_solve_kkt_unconstr
#define GEAD blasfeo_dgead
#define GECP blasfeo_dgecp
#define GELQF_WORKSIZE blasfeo_dgelqf_worksize
#define GEMV_T blasfeo_dgemv_t
#define HPIPM_MODE hpipm_mode
#define INIT_VAR_OCP_QCQP d_init_var_ocp_qcqp
//#define MEMSIZE_CORE_QP_IPM d_memsize_core_qp_ipm
#define TREE_OCP_QCQP d_tree_ocp_qcqp
#define TREE_OCP_QCQP_IPM_ARG d_tree_ocp_qcqp_ipm_arg
#define TREE_OCP_QCQP_IPM_WS d_tree_ocp_qcqp_ipm_ws
#define TREE_OCP_QCQP_DIM d_tree_ocp_qcqp_dim
#define TREE_OCP_QCQP_RES d_tree_ocp_qcqp_res
#define TREE_OCP_QCQP_RES_COMPUTE d_tree_ocp_qcqp_res_compute
#define TREE_OCP_QCQP_RES_COMPUTE_INF_NORM d_tree_ocp_qcqp_res_compute_inf_norm
#define TREE_OCP_QCQP_RES_COMPUTE_LIN d_tree_ocp_qcqp_res_compute_lin
#define TREE_OCP_QCQP_RES_CREATE d_tree_ocp_qcqp_res_create
#define TREE_OCP_QCQP_RES_MEMSIZE d_tree_ocp_qcqp_res_memsize
#define TREE_OCP_QCQP_RES_WS d_tree_ocp_qcqp_res_ws
#define TREE_OCP_QCQP_RES_WS_CREATE d_tree_ocp_qcqp_res_ws_create
#define TREE_OCP_QCQP_RES_WS_MEMSIZE d_tree_ocp_qcqp_res_ws_memsize
#define TREE_OCP_QCQP_SOL d_tree_ocp_qcqp_sol
#define TREE_OCP_QCQP_SOL_CREATE d_tree_ocp_qcqp_sol_create
#define TREE_OCP_QCQP_SOL_MEMSIZE d_tree_ocp_qcqp_sol_memsize
#define TREE_OCP_QP d_tree_ocp_qp
#define TREE_OCP_QP_CREATE d_tree_ocp_qp_create
#define TREE_OCP_QP_MEMSIZE d_tree_ocp_qp_memsize
#define TREE_OCP_QP_IPM_ARG d_tree_ocp_qp_ipm_arg
#define TREE_OCP_QP_IPM_ARG_CREATE d_tree_ocp_qp_ipm_arg_create
#define TREE_OCP_QP_IPM_ARG_MEMSIZE d_tree_ocp_qp_ipm_arg_memsize
#define TREE_OCP_QP_IPM_ARG_SET_DEFAULT d_tree_ocp_qp_ipm_arg_set_default
#define TREE_OCP_QP_IPM_ARG_SET d_tree_ocp_qp_ipm_arg_set
#define TREE_OCP_QP_IPM_ARG_SET_ITER_MAX d_tree_ocp_qp_ipm_arg_set_iter_max
#define TREE_OCP_QP_IPM_ARG_SET_ALPHA_MIN d_tree_ocp_qp_ipm_arg_set_alpha_min
#define TREE_OCP_QP_IPM_ARG_SET_MU0 d_tree_ocp_qp_ipm_arg_set_mu0
#define TREE_OCP_QP_IPM_ARG_SET_TOL_STAT d_tree_ocp_qp_ipm_arg_set_tol_stat
#define TREE_OCP_QP_IPM_ARG_SET_TOL_EQ d_tree_ocp_qp_ipm_arg_set_tol_eq
#define TREE_OCP_QP_IPM_ARG_SET_TOL_INEQ d_tree_ocp_qp_ipm_arg_set_tol_ineq
#define TREE_OCP_QP_IPM_ARG_SET_TOL_COMP d_tree_ocp_qp_ipm_arg_set_tol_comp
#define TREE_OCP_QP_IPM_ARG_SET_REG_PRIM d_tree_ocp_qp_ipm_arg_set_reg_prim
#define TREE_OCP_QP_IPM_ARG_SET_REG_DUAL d_tree_ocp_qp_ipm_arg_set_reg_dual
#define TREE_OCP_QP_IPM_ARG_SET_WARM_START d_tree_ocp_qp_ipm_arg_set_warm_start
#define TREE_OCP_QP_IPM_ARG_SET_PRED_CORR d_tree_ocp_qp_ipm_arg_set_pred_corr
#define TREE_OCP_QP_IPM_ARG_SET_COND_PRED_CORR d_tree_ocp_qp_ipm_arg_set_cond_pred_corr
#define TREE_OCP_QP_IPM_ARG_SET_COMP_RES_PRED d_tree_ocp_qp_ipm_arg_set_comp_res_pred
#define TREE_OCP_QP_IPM_ARG_SET_COMP_RES_EXIT d_tree_ocp_qp_ipm_arg_set_comp_res_exit
#define TREE_OCP_QP_IPM_ARG_SET_RIC_ALG d_tree_ocp_qp_ipm_arg_set_ric_alg
#define TREE_OCP_QP_IPM_ARG_SET_LAM_MIN d_tree_ocp_qp_ipm_arg_set_lam_min
#define TREE_OCP_QP_IPM_ARG_SET_T_MIN d_tree_ocp_qp_ipm_arg_set_t_min
#define TREE_OCP_QP_IPM_ARG_SET_SPLIT_STEP d_tree_ocp_qp_ipm_arg_set_split_step
#define TREE_OCP_QP_IPM_ARG_SET_T_LAM_MIN d_tree_ocp_qp_ipm_arg_set_t_lam_min
#define TREE_OCP_QP_IPM_ABS_STEP d_tree_ocp_qp_ipm_abs_step
#define TREE_OCP_QP_IPM_DELTA_STEP d_tree_ocp_qp_ipm_delta_step
#define TREE_OCP_QP_IPM_GET_STAT d_tree_ocp_qp_ipm_get_stat
#define TREE_OCP_QP_IPM_GET_STAT_M d_tree_ocp_qp_ipm_get_stat_m
#define TREE_OCP_QP_IPM_WS d_tree_ocp_qp_ipm_ws
#define TREE_OCP_QP_IPM_WS_CREATE d_tree_ocp_qp_ipm_ws_create
#define TREE_OCP_QP_IPM_WS_MEMSIZE d_tree_ocp_qp_ipm_ws_memsize
#define TREE_OCP_QP_RES d_tree_ocp_qp_res
#define TREE_OCP_QP_SOL d_tree_ocp_qp_sol
#define TREE_OCP_QP_SOL_CREATE d_tree_ocp_qp_sol_create
#define TREE_OCP_QP_SOL_MEMSIZE d_tree_ocp_qp_sol_memsize
#define REAL double
#define SIZE_STRMAT blasfeo_memsize_dmat
#define SIZE_STRVEC blasfeo_memsize_dvec
#define SOLVE_KKT_STEP_OCP_QCQP d_solve_kkt_step_ocp_qcqp
#define STRMAT blasfeo_dmat
#define STRVEC blasfeo_dvec
#define SYMV_L blasfeo_dsymv_l
#define UPDATE_VAR_QCQP d_update_var_qcqp
#define VECCP blasfeo_dveccp
#define VECMUL blasfeo_dvecmul
#define VECMULDOT blasfeo_dvecmuldot
#define VECNRM_INF blasfeo_dvecnrm_inf
#define VECSC blasfeo_dvecsc
#define VECSE blasfeo_dvecse



// arg
#define TREE_OCP_QCQP_IPM_ARG_STRSIZE d_tree_ocp_qcqp_ipm_arg_strsize
#define TREE_OCP_QCQP_IPM_ARG_MEMSIZE d_tree_ocp_qcqp_ipm_arg_memsize
#define TREE_OCP_QCQP_IPM_ARG_CREATE d_tree_ocp_qcqp_ipm_arg_create
#define TREE_OCP_QCQP_IPM_ARG_SET_DEFAULT d_tree_ocp_qcqp_ipm_arg_set_default
#define TREE_OCP_QCQP_IPM_ARG_SET d_tree_ocp_qcqp_ipm_arg_set
#define TREE_OCP_QCQP_IPM_ARG_SET_ITER_MAX d_tree_ocp_qcqp_ipm_arg_set_iter_max
#define TREE_OCP_QCQP_IPM_ARG_SET_ALPHA_MIN d_tree_ocp_qcqp_ipm_arg_set_alpha_min
#define TREE_OCP_QCQP_IPM_ARG_SET_MU0 d_tree_ocp_qcqp_ipm_arg_set_mu0
#define TREE_OCP_QCQP_IPM_ARG_SET_TOL_STAT d_tree_ocp_qcqp_ipm_arg_set_tol_stat
#define TREE_OCP_QCQP_IPM_ARG_SET_TOL_EQ d_tree_ocp_qcqp_ipm_arg_set_tol_eq
#define TREE_OCP_QCQP_IPM_ARG_SET_TOL_INEQ d_tree_ocp_qcqp_ipm_arg_set_tol_ineq
#define TREE_OCP_QCQP_IPM_ARG_SET_TOL_COMP d_tree_ocp_qcqp_ipm_arg_set_tol_comp
#define TREE_OCP_QCQP_IPM_ARG_SET_REG_PRIM d_tree_ocp_qcqp_ipm_arg_set_reg_prim
#define TREE_OCP_QCQP_IPM_ARG_SET_WARM_START d_tree_ocp_qcqp_ipm_arg_set_warm_start
#define TREE_OCP_QCQP_IPM_ARG_SET_PRED_CORR d_tree_ocp_qcqp_ipm_arg_set_pred_corr
#define TREE_OCP_QCQP_IPM_ARG_SET_COND_PRED_CORR d_tree_ocp_qcqp_ipm_arg_set_cond_pred_corr
#define TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_PRED d_tree_ocp_qcqp_ipm_arg_set_comp_res_pred
#define TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_EXIT d_tree_ocp_qcqp_ipm_arg_set_comp_res_exit
#define TREE_OCP_QCQP_IPM_ARG_SET_RIC_ALG d_tree_ocp_qcqp_ipm_arg_set_ric_alg
#define TREE_OCP_QCQP_IPM_ARG_SET_LAM_MIN d_tree_ocp_qcqp_ipm_arg_set_lam_min
#define TREE_OCP_QCQP_IPM_ARG_SET_T_MIN d_tree_ocp_qcqp_ipm_arg_set_t_min
#define TREE_OCP_QCQP_IPM_ARG_SET_SPLIT_STEP d_tree_ocp_qcqp_ipm_arg_set_split_step
#define TREE_OCP_QCQP_IPM_ARG_SET_T_LAM_MIN d_tree_ocp_qcqp_ipm_arg_set_t_lam_min
// ipm
#define TREE_OCP_QCQP_IPM_WS_STRSIZE d_tree_ocp_qcqp_ipm_ws_strsize
#define TREE_OCP_QCQP_IPM_WS_MEMSIZE d_tree_ocp_qcqp_ipm_ws_memsize
#define TREE_OCP_QCQP_IPM_WS_CREATE d_tree_ocp_qcqp_ipm_ws_create
#define TREE_OCP_QCQP_IPM_GET d_tree_ocp_qcqp_ipm_get
#define TREE_OCP_QCQP_IPM_GET_STATUS d_tree_ocp_qcqp_ipm_get_status
#define TREE_OCP_QCQP_IPM_GET_ITER d_tree_ocp_qcqp_ipm_get_iter
#define TREE_OCP_QCQP_IPM_GET_MAX_RES_STAT d_tree_ocp_qcqp_ipm_get_max_res_stat
#define TREE_OCP_QCQP_IPM_GET_MAX_RES_EQ d_tree_ocp_qcqp_ipm_get_max_res_eq
#define TREE_OCP_QCQP_IPM_GET_MAX_RES_INEQ d_tree_ocp_qcqp_ipm_get_max_res_ineq
#define TREE_OCP_QCQP_IPM_GET_MAX_RES_COMP d_tree_ocp_qcqp_ipm_get_max_res_comp
#define TREE_OCP_QCQP_IPM_GET_STAT d_tree_ocp_qcqp_ipm_get_stat
#define TREE_OCP_QCQP_IPM_GET_STAT_M d_tree_ocp_qcqp_ipm_get_stat_m
#define TREE_OCP_QCQP_INIT_VAR d_tree_ocp_qcqp_init_var
#define TREE_OCP_QCQP_APPROX_QP d_tree_ocp_qcqp_approx_qp
#define TREE_OCP_QCQP_UPDATE_QP d_tree_ocp_qcqp_update_qp
#define TREE_OCP_QCQP_UPDATE_QP_ABS_STEP d_tree_ocp_qcqp_update_qp_abs_step
#define TREE_OCP_QCQP_SOL_CONV_QP_SOL d_tree_ocp_qcqp_sol_conv_qp_sol
#define TREE_OCP_QP_SOL_CONV_QCQP_SOL d_tree_ocp_qp_sol_conv_qcqp_sol
#define TREE_OCP_QCQP_RES_CONV_QP_RES d_tree_ocp_qcqp_res_conv_qp_res
#define TREE_OCP_QCQP_IPM_SOLVE d_tree_ocp_qcqp_ipm_solve
#define TREE_OCP_QCQP_IPM_PREDICT d_tree_ocp_qcqp_ipm_predict
#define TREE_OCP_QCQP_IPM_SENS d_tree_ocp_qcqp_ipm_sens

#include "x_tree_ocp_qcqp_ipm.c"



