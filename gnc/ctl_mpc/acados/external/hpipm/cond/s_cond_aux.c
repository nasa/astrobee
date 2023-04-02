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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_s_blas.h>
#include <blasfeo_s_aux.h>

#include "../include/hpipm_s_ocp_qp.h"
#include "../include/hpipm_s_ocp_qp_sol.h"
#include "../include/hpipm_s_dense_qp.h"
#include "../include/hpipm_s_dense_qp_sol.h"
#include "../include/hpipm_s_cond.h"



#define SINGLE_PRECISION



#define AXPY blasfeo_saxpy
#define COND_QP_ARG s_cond_qp_arg
#define COND_QP_WS s_cond_qp_ws
#define DENSE_QP_SOL s_dense_qp_sol
#define DIAEX blasfeo_sdiaex
#define GEAD blasfeo_sgead
#define GECP blasfeo_sgecp
#define GEEX1 blasfeo_sgeex1
#define GESE blasfeo_sgese
#define GEMM_ND blasfeo_sgemm_nd
#define GEMM_NN blasfeo_sgemm_nn
#define GEMM_NT blasfeo_sgemm_nt
#define GEMV_D blasfeo_sgemv_d
#define GEMV_N blasfeo_sgemv_n
#define GEMV_T blasfeo_sgemv_t
#define OCP_QP s_ocp_qp
#define OCP_QP_SOL s_ocp_qp_sol
#define POTRF_L_MN blasfeo_spotrf_l_mn
#define PRINT_MAT blasfeo_print_smat
#define REAL float
#define ROWAD blasfeo_srowad
#define ROWEX blasfeo_srowex
#define ROWIN blasfeo_srowin
#define STRMAT blasfeo_smat
#define STRVEC blasfeo_svec
#define SYMV_L blasfeo_ssymv_l
#define SYRK_LN_MN blasfeo_ssyrk_ln_mn
#define TRCP_L blasfeo_strcp_l
#define TRTR_L blasfeo_strtr_l
#define TRMM_RLNN blasfeo_strmm_rlnn
#define VECAD_SP blasfeo_svecad_sp
#define VECCP blasfeo_sveccp
#define VECSE blasfeo_svecse

#define COND_BABT s_cond_BAbt
#define COND_BAT s_cond_BAt
#define COND_B s_cond_b
#define COND_RSQRQ s_cond_RSQrq
#define COND_RSQ s_cond_RSQ
#define COND_RQ s_cond_rq
#define COND_DCTD s_cond_DCtd
#define COND_DCT s_cond_DCt
#define COND_D s_cond_d
#define EXPAND_SOL s_expand_sol
#define EXPAND_PRIMAL_SOL s_expand_primal_sol
#define UPDATE_COND_BABT s_update_cond_BAbt
#define UPDATE_COND_RSQRQ_N2NX3 s_update_cond_RSQrq_N2nx3
#define UPDATE_COND_DCTD s_update_cond_DCtd



#include "x_cond_aux.c"

