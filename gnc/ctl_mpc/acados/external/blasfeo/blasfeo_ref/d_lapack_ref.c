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
#include <math.h>

#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>



#define REF



#if defined(MF_COLMAJ)
	#define XMATEL_A(X, Y) pA[(X)+lda*(Y)]
	#define XMATEL_B(X, Y) pB[(X)+ldb*(Y)]
	#define XMATEL_C(X, Y) pC[(X)+ldc*(Y)]
	#define XMATEL_D(X, Y) pD[(X)+ldd*(Y)]
	#define XMATEL_L(X, Y) pL[(X)+ldl*(Y)]
#else // MF_PANELMAJ
	#define XMATEL_A(X, Y) XMATEL(sA, X, Y)
	#define XMATEL_B(X, Y) XMATEL(sB, X, Y)
	#define XMATEL_C(X, Y) XMATEL(sC, X, Y)
	#define XMATEL_D(X, Y) XMATEL(sD, X, Y)
	#define XMATEL_L(X, Y) XMATEL(sL, X, Y)
#endif



#define SQRT sqrt
#define REAL double
#define XMAT blasfeo_dmat
#define XMATEL BLASFEO_DMATEL
#define XVEC blasfeo_dvec
#define XVECEL BLASFEO_DVECEL



#define SYRK_LN blasfeo_dsyrk_ln
#define SYRK_LN_MN blasfeo_dsyrk_ln_mn

#define REF_GELQF_WORK_SIZE blasfeo_ref_dgelqf_worksize
#define REF_GELQF blasfeo_ref_dgelqf
#define REF_ORGLQ_WORK_SIZE blasfeo_ref_dorglq_worksize
#define REF_ORGLQ blasfeo_ref_dorglq
#define REF_GELQF_PD blasfeo_ref_dgelqf_pd
#define REF_GELQF_PD_DA blasfeo_ref_dgelqf_pd_da
#define REF_GELQF_PD_LA blasfeo_ref_dgelqf_pd_la
#define REF_GELQF_PD_LLA blasfeo_ref_dgelqf_pd_lla
#define REF_GEQRF blasfeo_ref_dgeqrf
#define REF_GEQRF_WORK_SIZE blasfeo_ref_dgeqrf_worksize
#define REF_GETRF_NOPIVOT blasfeo_ref_dgetrf_np
#define REF_GETRF_ROWPIVOT blasfeo_ref_dgetrf_rp
#define REF_POTRF_L blasfeo_ref_dpotrf_l
#define REF_POTRF_L_MN blasfeo_ref_dpotrf_l_mn
#define REF_POTRF_U blasfeo_ref_dpotrf_u
#define REF_PSTRF_L blasfeo_ref_dpstrf_l
#define REF_SYRK_POTRF_LN blasfeo_ref_dsyrk_dpotrf_ln
#define REF_SYRK_POTRF_LN_MN blasfeo_ref_dsyrk_dpotrf_ln_mn

#define GELQF_WORK_SIZE blasfeo_dgelqf_worksize
#define GELQF blasfeo_dgelqf
#define ORGLQ_WORK_SIZE blasfeo_dorglq_worksize
#define ORGLQ blasfeo_dorglq
#define GELQF_PD blasfeo_dgelqf_pd
#define GELQF_PD_DA blasfeo_dgelqf_pd_da
#define GELQF_PD_LA blasfeo_dgelqf_pd_la
#define GELQF_PD_LLA blasfeo_dgelqf_pd_lla
#define GEQRF blasfeo_dgeqrf
#define GEQRF_WORK_SIZE blasfeo_dgeqrf_worksize
#define GETRF_NOPIVOT blasfeo_dgetrf_np
#define GETRF_ROWPIVOT blasfeo_dgetrf_rp
#define POTRF_L blasfeo_dpotrf_l
#define POTRF_L_MN blasfeo_dpotrf_l_mn
#define POTRF_U blasfeo_dpotrf_u
#define PSTRF_L blasfeo_dpstrf_l
#define SYRK_POTRF_LN blasfeo_dsyrk_dpotrf_ln
#define SYRK_POTRF_LN_MN blasfeo_dsyrk_dpotrf_ln_mn



#include "x_lapack_ref.c"

