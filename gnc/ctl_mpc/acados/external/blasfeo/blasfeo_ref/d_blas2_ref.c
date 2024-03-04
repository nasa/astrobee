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



#if defined(MF_COLMAJ)
	#define XMATEL_A(X, Y) pA[(X)+lda*(Y)]
	#define XMATEL_C(X, Y) pC[(X)+ldc*(Y)]
	#define XMATEL_D(X, Y) pD[(X)+ldd*(Y)]
#else // MF_PANELMAJ
	#define XMATEL_A(X, Y) XMATEL(sA, X, Y)
	#define XMATEL_C(X, Y) XMATEL(sC, X, Y)
	#define XMATEL_D(X, Y) XMATEL(sD, X, Y)
#endif



#define REF



#define REAL double
#define XMAT blasfeo_dmat
#define XMATEL BLASFEO_DMATEL
#define XVEC blasfeo_dvec
#define XVECEL BLASFEO_DVECEL



#define REF_GEMV_N blasfeo_ref_dgemv_n
#define REF_GEMV_NT blasfeo_ref_dgemv_nt
#define REF_GEMV_T blasfeo_ref_dgemv_t
#define REF_SYMV_L blasfeo_ref_dsymv_l
#define REF_SYMV_L_MN blasfeo_ref_dsymv_l_mn
#define REF_SYMV_U blasfeo_ref_dsymv_u
#define REF_TRMV_LNN blasfeo_ref_dtrmv_lnn
#define REF_TRMV_LTN blasfeo_ref_dtrmv_ltn
#define REF_TRMV_UNN blasfeo_ref_dtrmv_unn
#define REF_TRMV_UTN blasfeo_ref_dtrmv_utn
#define REF_TRSV_LNN blasfeo_ref_dtrsv_lnn
#define REF_TRSV_LNN_MN blasfeo_ref_dtrsv_lnn_mn
#define REF_TRSV_LNU blasfeo_ref_dtrsv_lnu
#define REF_TRSV_LTN blasfeo_ref_dtrsv_ltn
#define REF_TRSV_LTN_MN blasfeo_ref_dtrsv_ltn_mn
#define REF_TRSV_LTU blasfeo_ref_dtrsv_ltu
#define REF_TRSV_UNN blasfeo_ref_dtrsv_unn
#define REF_TRSV_UTN blasfeo_ref_dtrsv_utn
#define REF_GER blasfeo_ref_dger

#define GEMV_N blasfeo_dgemv_n
#define GEMV_NT blasfeo_dgemv_nt
#define GEMV_T blasfeo_dgemv_t
#define SYMV_L blasfeo_dsymv_l
#define SYMV_L_MN blasfeo_dsymv_l_mn
#define SYMV_U blasfeo_dsymv_u
#define TRMV_LNN blasfeo_dtrmv_lnn
#define TRMV_LTN blasfeo_dtrmv_ltn
#define TRMV_UNN blasfeo_dtrmv_unn
#define TRMV_UTN blasfeo_dtrmv_utn
#define TRSV_LNN blasfeo_dtrsv_lnn
#define TRSV_LNN_MN blasfeo_dtrsv_lnn_mn
#define TRSV_LNU blasfeo_dtrsv_lnu
#define TRSV_LTN blasfeo_dtrsv_ltn
#define TRSV_LTN_MN blasfeo_dtrsv_ltn_mn
#define TRSV_LTU blasfeo_dtrsv_ltu
#define TRSV_UNN blasfeo_dtrsv_unn
#define TRSV_UTN blasfeo_dtrsv_utn
#define GER blasfeo_dger



#include "x_blas2_ref.c"

