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

#if defined(LA_EXTERNAL_BLAS_WRAPPER)
#if defined(EXTERNAL_BLAS_BLIS)
#include <blis.h>
#elif defined(EXTERNAL_BLAS_MKL)
#include <mkl.h>
#elif defined(EXTERNAL_BLAS_ARMPL)
#include "armpl.h"
#else
#include "../include/d_blas.h"
#endif
#endif

#include <blasfeo_common.h>



#define REAL double
#define XMAT blasfeo_dmat
#define XVEC blasfeo_dvec



#define GEMM_NN blasfeo_dgemm_nn
#define GEMM_NT blasfeo_dgemm_nt
#define GEMM_TN blasfeo_dgemm_tn
#define GEMM_TT blasfeo_dgemm_tt
#define SYRK_LN blasfeo_dsyrk_ln
#define SYRK_LN_MN blasfeo_dsyrk_ln_mn
#define SYRK_LT blasfeo_dsyrk_lt
#define SYRK_UN blasfeo_dsyrk_un
#define SYRK_UT blasfeo_dsyrk_ut
#define TRMM_RLNN blasfeo_dtrmm_rlnn
#define TRMM_RUTN blasfeo_dtrmm_rutn
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
#define SYR2K_LN blasfeo_dsyr2k_ln
#define SYR2K_LT blasfeo_dsyr2k_lt
#define SYR2K_UN blasfeo_dsyr2k_un
#define SYR2K_UT blasfeo_dsyr2k_ut

#define COPY dcopy_
#define GEMM dgemm_
#define SYRK dsyrk_
#define TRMM dtrmm_
#define TRSM dtrsm_
#define SYR2K dsyr2k_



#include "x_blas3_lib.c"
