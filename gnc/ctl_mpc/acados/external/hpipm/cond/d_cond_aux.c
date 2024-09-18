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
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>

#include "../include/hpipm_d_ocp_qp.h"
#include "../include/hpipm_d_ocp_qp_sol.h"
#include "../include/hpipm_d_dense_qp.h"
#include "../include/hpipm_d_dense_qp_sol.h"
#include "../include/hpipm_d_cond.h"



#define DOUBLE_PRECISION



#define AXPY blasfeo_daxpy
#define COND_QP_ARG d_cond_qp_arg
#define COND_QP_WS d_cond_qp_ws
#define DENSE_QP_SOL d_dense_qp_sol
#define DIAEX blasfeo_ddiaex
#define GEAD blasfeo_dgead
#define GECP blasfeo_dgecp
#define GEEX1 blasfeo_dgeex1
#define GESE blasfeo_dgese
#define GEMM_ND blasfeo_dgemm_nd
#define GEMM_NN blasfeo_dgemm_nn
#define GEMM_NT blasfeo_dgemm_nt
#define GEMV_D blasfeo_dgemv_d
#define GEMV_N blasfeo_dgemv_n
#define GEMV_T blasfeo_dgemv_t
#define OCP_QP d_ocp_qp
#define OCP_QP_SOL d_ocp_qp_sol
#define POTRF_L_MN blasfeo_dpotrf_l_mn
#define PRINT_MAT blasfeo_print_dmat
#define REAL double
#define ROWAD blasfeo_drowad
#define ROWEX blasfeo_drowex
#define ROWIN blasfeo_drowin
#define STRMAT blasfeo_dmat
#define STRVEC blasfeo_dvec
#define SYMV_L blasfeo_dsymv_l
#define SYRK_LN_MN blasfeo_dsyrk_ln_mn
#define TRCP_L blasfeo_dtrcp_l
#define TRTR_L blasfeo_dtrtr_l
#define TRMM_RLNN blasfeo_dtrmm_rlnn
#define VECAD_SP blasfeo_dvecad_sp
#define VECCP blasfeo_dveccp
#define VECSE blasfeo_dvecse

#define COND_BABT d_cond_BAbt
#define COND_BAT d_cond_BAt
#define COND_B d_cond_b
#define COND_RSQRQ d_cond_RSQrq
#define COND_RSQ d_cond_RSQ
#define COND_RQ d_cond_rq
#define COND_DCTD d_cond_DCtd
#define COND_DCT d_cond_DCt
#define COND_D d_cond_d
#define EXPAND_SOL d_expand_sol
#define EXPAND_PRIMAL_SOL d_expand_primal_sol
#define UPDATE_COND_BABT d_update_cond_BAbt
#define UPDATE_COND_RSQRQ_N2NX3 d_update_cond_RSQrq_N2nx3
#define UPDATE_COND_DCTD d_update_cond_DCtd



#include "x_cond_aux.c"
