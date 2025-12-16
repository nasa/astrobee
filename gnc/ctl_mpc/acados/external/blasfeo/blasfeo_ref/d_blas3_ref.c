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



#if defined(MF_COLMAJ)
	#define XMATEL_A(X, Y) pA[(X)+lda*(Y)]
	#define XMATEL_B(X, Y) pB[(X)+ldb*(Y)]
	#define XMATEL_C(X, Y) pC[(X)+ldc*(Y)]
	#define XMATEL_D(X, Y) pD[(X)+ldd*(Y)]
#else // MF_PANELMAJ
	#define XMATEL_A(X, Y) XMATEL(sA, X, Y)
	#define XMATEL_B(X, Y) XMATEL(sB, X, Y)
	#define XMATEL_C(X, Y) XMATEL(sC, X, Y)
	#define XMATEL_D(X, Y) XMATEL(sD, X, Y)
#endif



#define REF
#define DP



#define REAL double
#define XMAT blasfeo_dmat
#define XMATEL BLASFEO_DMATEL
#define XVEC blasfeo_dvec
#define XVECEL BLASFEO_DVECEL



// gemm
#define REF_GEMM_NN blasfeo_ref_dgemm_nn
#define REF_GEMM_NT blasfeo_ref_dgemm_nt
#define REF_GEMM_TN blasfeo_ref_dgemm_tn
#define REF_GEMM_TT blasfeo_ref_dgemm_tt
// syrk
#define REF_SYRK_LN blasfeo_ref_dsyrk_ln
#define REF_SYRK_LN_MN blasfeo_ref_dsyrk_ln_mn
#define REF_SYRK_LT blasfeo_ref_dsyrk_lt
#define REF_SYRK_UN blasfeo_ref_dsyrk_un
#define REF_SYRK_UT blasfeo_ref_dsyrk_ut
// trmm
#define REF_TRMM_LLNN blasfeo_ref_dtrmm_llnn
#define REF_TRMM_LLNU blasfeo_ref_dtrmm_llnu
#define REF_TRMM_LLTN blasfeo_ref_dtrmm_lltn
#define REF_TRMM_LLTU blasfeo_ref_dtrmm_lltu
#define REF_TRMM_LUNN blasfeo_ref_dtrmm_lunn
#define REF_TRMM_LUNU blasfeo_ref_dtrmm_lunu
#define REF_TRMM_LUTN blasfeo_ref_dtrmm_lutn
#define REF_TRMM_LUTU blasfeo_ref_dtrmm_lutu
#define REF_TRMM_RLNN blasfeo_ref_dtrmm_rlnn
#define REF_TRMM_RLNU blasfeo_ref_dtrmm_rlnu
#define REF_TRMM_RLTN blasfeo_ref_dtrmm_rltn
#define REF_TRMM_RLTU blasfeo_ref_dtrmm_rltu
#define REF_TRMM_RUNN blasfeo_ref_dtrmm_runn
#define REF_TRMM_RUNU blasfeo_ref_dtrmm_runu
#define REF_TRMM_RUTN blasfeo_ref_dtrmm_rutn
#define REF_TRMM_RUTU blasfeo_ref_dtrmm_rutu
// trsm
#define REF_TRSM_LLNN blasfeo_ref_dtrsm_llnn
#define REF_TRSM_LLNU blasfeo_ref_dtrsm_llnu
#define REF_TRSM_LLTN blasfeo_ref_dtrsm_lltn
#define REF_TRSM_LLTU blasfeo_ref_dtrsm_lltu
#define REF_TRSM_LUNN blasfeo_ref_dtrsm_lunn
#define REF_TRSM_LUNU blasfeo_ref_dtrsm_lunu
#define REF_TRSM_LUTN blasfeo_ref_dtrsm_lutn
#define REF_TRSM_LUTU blasfeo_ref_dtrsm_lutu
#define REF_TRSM_RLNN blasfeo_ref_dtrsm_rlnn
#define REF_TRSM_RLNU blasfeo_ref_dtrsm_rlnu
#define REF_TRSM_RLTN blasfeo_ref_dtrsm_rltn
#define REF_TRSM_RLTU blasfeo_ref_dtrsm_rltu
#define REF_TRSM_RUNN blasfeo_ref_dtrsm_runn
#define REF_TRSM_RUNU blasfeo_ref_dtrsm_runu
#define REF_TRSM_RUTN blasfeo_ref_dtrsm_rutn
#define REF_TRSM_RUTU blasfeo_ref_dtrsm_rutu
// syrk
#define REF_SYR2K_LN blasfeo_ref_dsyr2k_ln
#define REF_SYR2K_LT blasfeo_ref_dsyr2k_lt
#define REF_SYR2K_UN blasfeo_ref_dsyr2k_un
#define REF_SYR2K_UT blasfeo_ref_dsyr2k_ut

// gemm
#define GEMM_NN blasfeo_dgemm_nn
#define GEMM_NT blasfeo_dgemm_nt
#define GEMM_TN blasfeo_dgemm_tn
#define GEMM_TT blasfeo_dgemm_tt
// syrk
#define SYRK_LN blasfeo_dsyrk_ln
#define SYRK_LN_MN blasfeo_dsyrk_ln_mn
#define SYRK_LT blasfeo_dsyrk_lt
#define SYRK_UN blasfeo_dsyrk_un
#define SYRK_UT blasfeo_dsyrk_ut
// trmm
#define TRMM_LLNN blasfeo_dtrmm_llnn
#define TRMM_LLNU blasfeo_dtrmm_llnu
#define TRMM_LLTN blasfeo_dtrmm_lltn
#define TRMM_LLTU blasfeo_dtrmm_lltu
#define TRMM_LUNN blasfeo_dtrmm_lunn
#define TRMM_LUNU blasfeo_dtrmm_lunu
#define TRMM_LUTN blasfeo_dtrmm_lutn
#define TRMM_LUTU blasfeo_dtrmm_lutu
#define TRMM_RLNN blasfeo_dtrmm_rlnn
#define TRMM_RLNU blasfeo_dtrmm_rlnu
#define TRMM_RLTN blasfeo_dtrmm_rltn
#define TRMM_RLTU blasfeo_dtrmm_rltu
#define TRMM_RUNN blasfeo_dtrmm_runn
#define TRMM_RUNU blasfeo_dtrmm_runu
#define TRMM_RUTN blasfeo_dtrmm_rutn
#define TRMM_RUTU blasfeo_dtrmm_rutu
// trsm
#define TRSM_LLNN blasfeo_dtrsm_llnn
#define TRSM_LLNU blasfeo_dtrsm_llnu
#define TRSM_LLTN blasfeo_dtrsm_lltn
#define TRSM_LLTU blasfeo_dtrsm_lltu
#define TRSM_LUNN blasfeo_dtrsm_lunn
#define TRSM_LUNU blasfeo_dtrsm_lunu
#define TRSM_LUTN blasfeo_dtrsm_lutn
#define TRSM_LUTU blasfeo_dtrsm_lutu
#define TRSM_RLNN blasfeo_dtrsm_rlnn
#define TRSM_RLNU blasfeo_dtrsm_rlnu
#define TRSM_RLTN blasfeo_dtrsm_rltn
#define TRSM_RLTU blasfeo_dtrsm_rltu
#define TRSM_RUNN blasfeo_dtrsm_runn
#define TRSM_RUNU blasfeo_dtrsm_runu
#define TRSM_RUTN blasfeo_dtrsm_rutn
#define TRSM_RUTU blasfeo_dtrsm_rutu
// syr2k
#define SYR2K_LN blasfeo_dsyr2k_ln
#define SYR2K_LT blasfeo_dsyr2k_lt
#define SYR2K_UN blasfeo_dsyr2k_un
#define SYR2K_UT blasfeo_dsyr2k_ut



#include "x_blas3_ref.c"
