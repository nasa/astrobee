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

#include <blasfeo_d_aux_ext_dep.h>

#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_d_dense_qp_res.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_core_qp_ipm.h>
#include <hpipm_d_core_qp_ipm_aux.h>



#define AXPY blasfeo_daxpy
#define COLIN blasfeo_dcolin
#define COLPE blasfeo_dcolpe
#define COLPEI blasfeo_dcolpei
#define COLSC blasfeo_dcolsc
#define COMPUTE_LAM_T_QP d_compute_lam_t_qp
#define COMPUTE_GAMMA_GAMMA_QP d_compute_Gamma_gamma_qp
#define COMPUTE_GAMMA_QP d_compute_gamma_qp
#define CORE_QP_IPM_WORKSPACE d_core_qp_ipm_workspace
#define DENSE_QP d_dense_qp
#define DENSE_QP_IPM_ARG d_dense_qp_ipm_arg
#define DENSE_QP_IPM_WS d_dense_qp_ipm_ws
#define DENSE_QP_RES d_dense_qp_res
#define DENSE_QP_RES_WORKSPACE d_dense_qp_res_workspace
#define DENSE_QP_SOL d_dense_qp_sol
#define DIAAD_SP blasfeo_ddiaad_sp
#define DIAEX blasfeo_ddiaex
#define DIARE blasfeo_ddiare
#define DOT blasfeo_ddot
#define GEAD blasfeo_dgead
#define GECP blasfeo_dgecp
#define GELQF blasfeo_dgelqf
#define GELQF_PD_LA blasfeo_dgelqf_pd_la
#define GELQF_PD_LLA blasfeo_dgelqf_pd_lla
#define GELQF_PD blasfeo_dgelqf_pd
#define GELQF_WORKSIZE blasfeo_dgelqf_worksize
#define GEMM_L_DIAG blasfeo_dgemm_dn
#define GEMM_NT blasfeo_dgemm_nt
#define GEMM_R_DIAG blasfeo_dgemm_nd
#define GEMV_DIAG blasfeo_dgemv_d
#define GEMV_N blasfeo_dgemv_n
#define GEMV_T blasfeo_dgemv_t
#define GESE blasfeo_dgese
#define GETR blasfeo_dgetr
#define GETRF_RP blasfeo_dgetrf_rp
#define ORGLQ blasfeo_dorglq
#define POTRF_L blasfeo_dpotrf_l
#define POTRF_L_MN blasfeo_dpotrf_l_mn
#define PSTRF_L dpstrf_l_libstr
#define REAL double
#define ROWAD_SP blasfeo_drowad_sp
#define ROWEX blasfeo_drowex
#define ROWIN blasfeo_drowin
#define ROWPE blasfeo_drowpe
#define ROWPEI blasfeo_drowpei
#define ROWSW blasfeo_drowsw
#define STRMAT blasfeo_dmat
#define STRVEC blasfeo_dvec
#define SYMV_L blasfeo_dsymv_l
#define SYRK_LN blasfeo_dsyrk_ln
#define SYRK_LN_MN blasfeo_dsyrk_ln_mn
#define SYRK_POTRF_LN blasfeo_dsyrk_dpotrf_ln
#define SYRK_POTRF_LN_MN blasfeo_dsyrk_dpotrf_ln_mn
#define TRCP_L blasfeo_dtrcp_l
#define TRSM_RLTN blasfeo_dtrsm_rltn
#define TRSM_RLTU blasfeo_dtrsm_rltu
#define TRSM_RUNN blasfeo_dtrsm_runn
#define TRSV_LNN blasfeo_dtrsv_lnn
#define TRSV_LNU blasfeo_dtrsv_lnu
#define TRSV_LTN blasfeo_dtrsv_ltn
#define TRSV_UNN blasfeo_dtrsv_unn
#define TRTR_L blasfeo_dtrtr_l
#define TRTR_U blasfeo_dtrtr_u
#define VECAD_SP blasfeo_dvecad_sp
#define VECCP blasfeo_dveccp
#define VECEX_SP blasfeo_dvecex_sp
#define VECMUL blasfeo_dvecmul
#define VECSC blasfeo_dvecsc
#define VECSE blasfeo_dvecse
#define VECCPSC blasfeo_dveccpsc
#define VECPE blasfeo_dvecpe
#define VECPEI blasfeo_dvecpei

#define FACT_SOLVE_KKT_UNCONSTR_DENSE_QP d_fact_solve_kkt_unconstr_dense_qp
#define FACT_LQ_SOLVE_KKT_STEP_DENSE_QP d_fact_lq_solve_kkt_step_dense_qp
#define FACT_SOLVE_KKT_STEP_DENSE_QP d_fact_solve_kkt_step_dense_qp
#define SOLVE_KKT_STEP_DENSE_QP d_solve_kkt_step_dense_qp
#define DENSE_QP_REMOVE_LIN_DEP_EQ d_dense_qp_remove_lin_dep_eq
#define DENSE_QP_RESTORE_LIN_DEP_EQ d_dense_qp_restore_lin_dep_eq
#define DENSE_QP_COMPUTE_OBJ d_dense_qp_compute_obj



#include "x_dense_qp_kkt.c"
