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
#include <math.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include <hpipm_tree.h>
#include <hpipm_d_tree_ocp_qp.h>
#include <hpipm_d_tree_ocp_qp_sol.h>
#include <hpipm_d_tree_ocp_qp_res.h>
#include <hpipm_d_tree_ocp_qp_ipm.h>
#include <hpipm_d_core_qp_ipm.h>
#include <hpipm_d_core_qp_ipm_aux.h>



#define DOUBLE_PRECISION
#define BLASFEO_MATEL BLASFEO_DMATEL
#define BLASFEO_VECEL BLASFEO_DVECEL



#define AXPY blasfeo_daxpy
#define COMPUTE_LAM_T_QP d_compute_lam_t_qp
#define COMPUTE_GAMMA_GAMMA_QP d_compute_Gamma_gamma_qp
#define COMPUTE_GAMMA_QP d_compute_gamma_qp
#define CORE_QP_IPM_WORKSPACE d_core_qp_ipm_workspace
#define COLSC blasfeo_dcolsc
#define DIAAD_SP blasfeo_ddiaad_sp
#define DIARE blasfeo_ddiare
#define GEAD blasfeo_dgead
#define GECP blasfeo_dgecp
#define GELQF blasfeo_dgelqf
#define GELQF_PD blasfeo_dgelqf_pd
#define GELQF_PD_LA blasfeo_dgelqf_pd_la
#define GELQF_PD_LLA blasfeo_dgelqf_pd_lla
#define GEMM_R_DIAG blasfeo_dgemm_nd
#define GEMV_DIAG blasfeo_dgemv_d
#define GEMV_N blasfeo_dgemv_n
#define GEMV_NT blasfeo_dgemv_nt
#define GEMV_T blasfeo_dgemv_t
#define GESE blasfeo_dgese
#define POTRF_L blasfeo_dpotrf_l
#define POTRF_L_MN blasfeo_dpotrf_l_mn
#define REAL double
#define ROWAD_SP blasfeo_drowad_sp
#define ROWIN blasfeo_drowin
#define ROWEX blasfeo_drowex
#define STRMAT blasfeo_dmat
#define STRVEC blasfeo_dvec
#define SYMV_L blasfeo_dsymv_l
#define SYRK_LN blasfeo_dsyrk_ln
#define SYRK_LN_MN blasfeo_dsyrk_ln_mn
#define SYRK_POTRF_LN_MN blasfeo_dsyrk_dpotrf_ln_mn
#define TRCP_L blasfeo_dtrcp_l
#define TREE_OCP_QP d_tree_ocp_qp
#define TREE_OCP_QP_IPM_ARG d_tree_ocp_qp_ipm_arg
#define TREE_OCP_QP_IPM_WS d_tree_ocp_qp_ipm_ws
#define TREE_OCP_QP_RES d_tree_ocp_qp_res
#define TREE_OCP_QP_RES_WS d_tree_ocp_qp_res_ws
#define TREE_OCP_QP_SOL d_tree_ocp_qp_sol
#define TRMM_RLNN blasfeo_dtrmm_rlnn
#define TRMV_LNN blasfeo_dtrmv_lnn
#define TRMV_LTN blasfeo_dtrmv_ltn
#define TRSV_LNN blasfeo_dtrsv_lnn
#define TRSV_LNN_MN blasfeo_dtrsv_lnn_mn
#define TRSV_LTN blasfeo_dtrsv_ltn
#define TRSV_LTN_MN blasfeo_dtrsv_ltn_mn
#define VECAD_SP blasfeo_dvecad_sp
#define VECCP blasfeo_dveccp
#define VECEX_SP blasfeo_dvecex_sp
#define VECMULACC blasfeo_dvecmulacc
#define VECMULDOT blasfeo_dvecmuldot
#define VECSC blasfeo_dvecsc

#define TREE_OCP_QP_COMPUTE_LIN_RES d_tree_ocp_qp_compute_lin_res
#define TREE_OCP_QP_COMPUTE_RES d_tree_ocp_qp_compute_res
#define TREE_OCP_QP_FACT_SOLVE_KKT_UNCONSTR d_tree_ocp_qp_fact_solve_kkt_unconstr
#define TREE_OCP_QP_FACT_SOLVE_KKT_STEP d_tree_ocp_qp_fact_solve_kkt_step
#define TREE_OCP_QP_FACT_LQ_SOLVE_KKT_STEP d_tree_ocp_qp_fact_lq_solve_kkt_step
#define TREE_OCP_QP_SOLVE_KKT_STEP d_tree_ocp_qp_solve_kkt_step



#include "x_tree_ocp_qp_kkt.c"
