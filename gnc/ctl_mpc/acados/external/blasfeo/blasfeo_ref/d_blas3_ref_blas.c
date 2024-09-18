/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS for embedded optimization.                                                      *
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

#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>



#define REF_BLAS



#define XMATEL_A(X, Y) pA[(X)+lda*(Y)]
#define XMATEL_B(X, Y) pB[(X)+ldb*(Y)]
#define XMATEL_C(X, Y) pC[(X)+ldc*(Y)]
#define XMATEL_D(X, Y) pD[(X)+ldd*(Y)]



#define REAL double
#define XMAT blasfeo_cm_dmat
#define XMATEL BLASFEO_CM_DMATEL
#define XVEC blasfeo_dvec
#define XVECEL BLASFEO_DVECEL



// gemm
#define REF_GEMM_NN blasfeo_hp_cm_dgemm_nn
#define REF_GEMM_NT blasfeo_hp_cm_dgemm_nt
#define REF_GEMM_TN blasfeo_hp_cm_dgemm_tn
#define REF_GEMM_TT blasfeo_hp_cm_dgemm_tt
// syrk
#define REF_SYRK_LN blasfeo_hp_cm_dsyrk_ln
#define REF_SYRK_LN_MN blasfeo_hp_cm_dsyrk_ln_mn
#define REF_SYRK_LT blasfeo_hp_cm_dsyrk_lt
#define REF_SYRK_UN blasfeo_hp_cm_dsyrk_un
#define REF_SYRK_UT blasfeo_hp_cm_dsyrk_ut
// trmm
#define REF_TRMM_LLNN blasfeo_hp_cm_dtrmm_llnn
#define REF_TRMM_LLNU blasfeo_hp_cm_dtrmm_llnu
#define REF_TRMM_LLTN blasfeo_hp_cm_dtrmm_lltn
#define REF_TRMM_LLTU blasfeo_hp_cm_dtrmm_lltu
#define REF_TRMM_LUNN blasfeo_hp_cm_dtrmm_lunn
#define REF_TRMM_LUNU blasfeo_hp_cm_dtrmm_lunu
#define REF_TRMM_LUTN blasfeo_hp_cm_dtrmm_lutn
#define REF_TRMM_LUTU blasfeo_hp_cm_dtrmm_lutu
#define REF_TRMM_RLNN blasfeo_hp_cm_dtrmm_rlnn
#define REF_TRMM_RLNU blasfeo_hp_cm_dtrmm_rlnu
#define REF_TRMM_RLTN blasfeo_hp_cm_dtrmm_rltn
#define REF_TRMM_RLTU blasfeo_hp_cm_dtrmm_rltu
#define REF_TRMM_RUNN blasfeo_hp_cm_dtrmm_runn
#define REF_TRMM_RUNU blasfeo_hp_cm_dtrmm_runu
#define REF_TRMM_RUTN blasfeo_hp_cm_dtrmm_rutn
#define REF_TRMM_RUTU blasfeo_hp_cm_dtrmm_rutu
// trsm
#define REF_TRSM_LLNN blasfeo_hp_cm_dtrsm_llnn
#define REF_TRSM_LLNU blasfeo_hp_cm_dtrsm_llnu
#define REF_TRSM_LLTN blasfeo_hp_cm_dtrsm_lltn
#define REF_TRSM_LLTU blasfeo_hp_cm_dtrsm_lltu
#define REF_TRSM_LUNN blasfeo_hp_cm_dtrsm_lunn
#define REF_TRSM_LUNU blasfeo_hp_cm_dtrsm_lunu
#define REF_TRSM_LUTN blasfeo_hp_cm_dtrsm_lutn
#define REF_TRSM_LUTU blasfeo_hp_cm_dtrsm_lutu
#define REF_TRSM_RLNN blasfeo_hp_cm_dtrsm_rlnn
#define REF_TRSM_RLNU blasfeo_hp_cm_dtrsm_rlnu
#define REF_TRSM_RLTN blasfeo_hp_cm_dtrsm_rltn
#define REF_TRSM_RLTU blasfeo_hp_cm_dtrsm_rltu
#define REF_TRSM_RUNN blasfeo_hp_cm_dtrsm_runn
#define REF_TRSM_RUNU blasfeo_hp_cm_dtrsm_runu
#define REF_TRSM_RUTN blasfeo_hp_cm_dtrsm_rutn
#define REF_TRSM_RUTU blasfeo_hp_cm_dtrsm_rutu
// syr2k
#define REF_SYR2K_LN blasfeo_hp_cm_dsyr2k_ln
#define REF_SYR2K_LT blasfeo_hp_cm_dsyr2k_lt
#define REF_SYR2K_UN blasfeo_hp_cm_dsyr2k_un
#define REF_SYR2K_UT blasfeo_hp_cm_dsyr2k_ut


// gemm
#define GEMM_NN blasfeo_cm_dgemm_nn
#define GEMM_NT blasfeo_cm_dgemm_nt
#define GEMM_TN blasfeo_cm_dgemm_tn
#define GEMM_TT blasfeo_cm_dgemm_tt
// syrk
#define SYRK_LN blasfeo_cm_dsyrk_ln
#define SYRK_LN_MN blasfeo_cm_dsyrk_ln_mn
#define SYRK_LT blasfeo_cm_dsyrk_lt
#define SYRK_UN blasfeo_cm_dsyrk_un
#define SYRK_UT blasfeo_cm_dsyrk_ut
// trmm
#define TRMM_LLNN blasfeo_cm_dtrmm_llnn
#define TRMM_LLNU blasfeo_cm_dtrmm_llnu
#define TRMM_LLTN blasfeo_cm_dtrmm_lltn
#define TRMM_LLTU blasfeo_cm_dtrmm_lltu
#define TRMM_LUNN blasfeo_cm_dtrmm_lunn
#define TRMM_LUNU blasfeo_cm_dtrmm_lunu
#define TRMM_LUTN blasfeo_cm_dtrmm_lutn
#define TRMM_LUTU blasfeo_cm_dtrmm_lutu
#define TRMM_RLNN blasfeo_cm_dtrmm_rlnn
#define TRMM_RLNU blasfeo_cm_dtrmm_rlnu
#define TRMM_RLTN blasfeo_cm_dtrmm_rltn
#define TRMM_RLTU blasfeo_cm_dtrmm_rltu
#define TRMM_RUNN blasfeo_cm_dtrmm_runn
#define TRMM_RUNU blasfeo_cm_dtrmm_runu
#define TRMM_RUTN blasfeo_cm_dtrmm_rutn
#define TRMM_RUTU blasfeo_cm_dtrmm_rutu
// trsm
#define TRSM_LLNN blasfeo_cm_dtrsm_llnn
#define TRSM_LLNU blasfeo_cm_dtrsm_llnu
#define TRSM_LLTN blasfeo_cm_dtrsm_lltn
#define TRSM_LLTU blasfeo_cm_dtrsm_lltu
#define TRSM_LUNN blasfeo_cm_dtrsm_lunn
#define TRSM_LUNU blasfeo_cm_dtrsm_lunu
#define TRSM_LUTN blasfeo_cm_dtrsm_lutn
#define TRSM_LUTU blasfeo_cm_dtrsm_lutu
#define TRSM_RLNN blasfeo_cm_dtrsm_rlnn
#define TRSM_RLNU blasfeo_cm_dtrsm_rlnu
#define TRSM_RLTN blasfeo_cm_dtrsm_rltn
#define TRSM_RLTU blasfeo_cm_dtrsm_rltu
#define TRSM_RUNN blasfeo_cm_dtrsm_runn
#define TRSM_RUNU blasfeo_cm_dtrsm_runu
#define TRSM_RUTN blasfeo_cm_dtrsm_rutn
#define TRSM_RUTU blasfeo_cm_dtrsm_rutu
// syr2k
#define SYR2K_LN blasfeo_cm_dsyr2k_ln
#define SYR2K_LT blasfeo_cm_dsyr2k_lt
#define SYR2K_UN blasfeo_cm_dsyr2k_un
#define SYR2K_UT blasfeo_cm_dsyr2k_ut



#include "x_blas3_ref.c"

