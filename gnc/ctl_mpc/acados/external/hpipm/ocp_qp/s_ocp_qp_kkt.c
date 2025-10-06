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
#include <blasfeo_s_aux.h>
#include <blasfeo_s_blas.h>

#include <hpipm_s_ocp_qp_dim.h>
#include <hpipm_s_ocp_qp.h>
#include <hpipm_s_ocp_qp_sol.h>
#include <hpipm_s_ocp_qp_ipm.h>
#include <hpipm_s_core_qp_ipm.h>
#include <hpipm_s_core_qp_ipm_aux.h>



#define SINGLE_PRECISION
#define BLASFEO_MATEL BLASFEO_SMATEL
#define BLASFEO_VECEL BLASFEO_SVECEL



#define AXPY blasfeo_saxpy
#define COLSC blasfeo_scolsc
#define COMPUTE_LAM_T_QP s_compute_lam_t_qp
#define COMPUTE_GAMMA_GAMMA_QP s_compute_Gamma_gamma_qp
#define COMPUTE_GAMMA_QP s_compute_gamma_qp
#define CORE_QP_IPM_WORKSPACE s_core_qp_ipm_workspace
#define DIAAD_SP blasfeo_sdiaad_sp
#define DIARE blasfeo_sdiare
#define GEAD blasfeo_sgead
#define GECP blasfeo_sgecp
#define GELQF blasfeo_sgelqf
#define GELQF_PD blasfeo_sgelqf_pd
#define GELQF_PD_LA blasfeo_sgelqf_pd_la
#define GELQF_PD_LLA blasfeo_sgelqf_pd_lla
#define GEMM_NT blasfeo_sgemm_nt
#define GEMM_R_DIAG blasfeo_sgemm_nd
#define GEMV_N blasfeo_sgemv_n
#define GEMV_T blasfeo_sgemv_t
#define GESE blasfeo_sgese
#define OCP_QP s_ocp_qp
#define OCP_QP_IPM_ARG s_ocp_qp_ipm_arg
#define OCP_QP_IPM_WS s_ocp_qp_ipm_ws
#define OCP_QP_DIM s_ocp_qp_dim
#define OCP_QP_SOL s_ocp_qp_sol
#define POTRF_L blasfeo_spotrf_l
#define POTRF_L_MN blasfeo_spotrf_l_mn
#define PRINT_E_MAT s_print_exp_mat
#define PRINT_E_STRVEC blasfeo_print_exp_svec
#define PRINT_E_TRAN_STRVEC blasfeo_print_exp_tran_svec
#define PRINT_STRMAT blasfeo_print_smat
#define PRINT_STRVEC blasfeo_print_svec
#define PRINT_TRAN_STRVEC blasfeo_print_tran_svec
#define REAL float
#define ROWAD_SP blasfeo_srowad_sp
#define ROWEX blasfeo_srowex
#define ROWIN blasfeo_srowin
#define STRMAT blasfeo_smat
#define STRVEC blasfeo_svec
#define SYRK_LN blasfeo_ssyrk_ln
#define SYRK_LN_MN blasfeo_ssyrk_ln_mn
#define SYRK_POTRF_LN_MN blasfeo_ssyrk_spotrf_ln_mn
#define TRCP_L blasfeo_strcp_l
#define TRMM_RLNN blasfeo_strmm_rlnn
#define TRMV_LNN blasfeo_strmv_lnn
#define TRMV_LTN blasfeo_strmv_ltn
#define TRSV_LNN blasfeo_strsv_lnn
#define TRSV_LTN blasfeo_strsv_ltn
#define TRSV_LNN_MN blasfeo_strsv_lnn_mn
#define TRSV_LTN_MN blasfeo_strsv_ltn_mn
#define TRTR_L blasfeo_strtr_l
#define VECAD_SP blasfeo_svecad_sp
#define VECCP blasfeo_sveccp
#define VECCPSC blasfeo_sveccpsc
#define VECEX_SP blasfeo_svecex_sp
#define VECSC blasfeo_svecsc



#define OCP_QP_FACT_SOLVE_KKT_UNCONSTR s_ocp_qp_fact_solve_kkt_unconstr
#define OCP_QP_FACT_SOLVE_KKT_STEP s_ocp_qp_fact_solve_kkt_step
#define OCP_QP_FACT_LQ_SOLVE_KKT_STEP s_ocp_qp_fact_lq_solve_kkt_step
#define OCP_QP_SOLVE_KKT_STEP s_ocp_qp_solve_kkt_step



#include "x_ocp_qp_kkt.c"

