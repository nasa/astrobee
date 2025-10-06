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

#include <blasfeo_s_aux_ext_dep.h>

#include <hpipm_s_dense_qp_dim.h>
#include <hpipm_s_dense_qp.h>
#include <hpipm_s_dense_qp_sol.h>
#include <hpipm_s_dense_qp_res.h>
#include <hpipm_s_dense_qp_ipm.h>
#include <hpipm_s_core_qp_ipm.h>
#include <hpipm_s_core_qp_ipm_aux.h>



#define AXPY blasfeo_saxpy
#define COLIN blasfeo_scolin
#define COLPE blasfeo_scolpe
#define COLPEI blasfeo_scolpei
#define COLSC blasfeo_scolsc
#define COMPUTE_LAM_T_QP s_compute_lam_t_qp
#define COMPUTE_GAMMA_GAMMA_QP s_compute_Gamma_gamma_qp
#define COMPUTE_GAMMA_QP s_compute_gamma_qp
#define CORE_QP_IPM_WORKSPACE s_core_qp_ipm_workspace
#define DENSE_QP_RES s_dense_qp_res
#define DENSE_QP_RES_WORKSPACE s_dense_qp_res_workspace
#define DENSE_QP s_dense_qp
#define DENSE_QP_IPM_ARG s_dense_qp_ipm_arg
#define DENSE_QP_IPM_WS s_dense_qp_ipm_ws
#define DENSE_QP_SOL s_dense_qp_sol
#define DIAAD_SP blasfeo_sdiaad_sp
#define DIAEX blasfeo_sdiaex
#define DIARE blasfeo_sdiare
#define DOT blasfeo_sdot
#define GEAD blasfeo_sgead
#define GECP blasfeo_sgecp
#define GELQF blasfeo_sgelqf
#define GELQF_PD_LA blasfeo_sgelqf_pd_la
#define GELQF_PD_LLA blasfeo_sgelqf_pd_lla
#define GELQF_PD blasfeo_sgelqf_pd
#define GELQF_WORKSIZE blasfeo_sgelqf_worksize
#define GEMM_L_DIAG blasfeo_sgemm_dn
#define GEMM_NT blasfeo_sgemm_nt
#define GEMM_R_DIAG blasfeo_sgemm_nd
#define GEMV_DIAG blasfeo_sgemv_d
#define GEMV_N blasfeo_sgemv_n
#define GEMV_T blasfeo_sgemv_t
#define GESE blasfeo_sgese
#define GETR blasfeo_sgetr
#define GETRF_RP blasfeo_sgetrf_rp
#define ORGLQ blasfeo_sorglq
#define POTRF_L blasfeo_spotrf_l
#define POTRF_L_MN blasfeo_spotrf_l_mn
#define PSTRF_L spstrf_l_libstr
#define REAL float
#define ROWAD_SP blasfeo_srowad_sp
#define ROWEX blasfeo_srowex
#define ROWIN blasfeo_srowin
#define ROWPE blasfeo_srowpe
#define ROWPEI blasfeo_srowpei
#define ROWSW blasfeo_srowsw
#define STRMAT blasfeo_smat
#define STRVEC blasfeo_svec
#define SYMV_L blasfeo_ssymv_l
#define SYRK_LN blasfeo_ssyrk_ln
#define SYRK_LN_MN blasfeo_ssyrk_ln_mn
#define SYRK_POTRF_LN blasfeo_ssyrk_spotrf_ln
#define SYRK_POTRF_LN_MN blasfeo_ssyrk_spotrf_ln_mn
#define TRCP_L blasfeo_strcp_l
#define TRSM_RLTN blasfeo_strsm_rltn
#define TRSM_RLTU blasfeo_strsm_rltu
#define TRSM_RUNN blasfeo_strsm_runn
#define TRSV_LNN blasfeo_strsv_lnn
#define TRSV_LNU blasfeo_strsv_lnu
#define TRSV_LTN blasfeo_strsv_ltn
#define TRSV_UNN blasfeo_strsv_unn
#define TRTR_L blasfeo_strtr_l
#define TRTR_U blasfeo_strtr_u
#define VECAD_SP blasfeo_svecad_sp
#define VECCP blasfeo_sveccp
#define VECEX_SP blasfeo_svecex_sp
#define VECMUL blasfeo_svecmul
#define VECSC blasfeo_svecsc
#define VECSE blasfeo_svecse
#define VECCPSC blasfeo_sveccpsc
#define VECPE blasfeo_svecpe
#define VECPEI blasfeo_svecpei

#define FACT_SOLVE_KKT_UNCONSTR_DENSE_QP s_fact_solve_kkt_unconstr_dense_qp
#define FACT_LQ_SOLVE_KKT_STEP_DENSE_QP s_fact_lq_solve_kkt_step_dense_qp
#define FACT_SOLVE_KKT_STEP_DENSE_QP s_fact_solve_kkt_step_dense_qp
#define SOLVE_KKT_STEP_DENSE_QP s_solve_kkt_step_dense_qp
#define DENSE_QP_REMOVE_LIN_DEP_EQ s_dense_qp_remove_lin_dep_eq
#define DENSE_QP_RESTORE_LIN_DEP_EQ s_dense_qp_restore_lin_dep_eq
#define DENSE_QP_COMPUTE_OBJ s_dense_qp_compute_obj



#include "x_dense_qp_kkt.c"

