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
#include <blasfeo_s_blas.h>



#define HP_CM



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



#define SQRT sqrtf
#define REAL float
#define XMAT blasfeo_smat
#define XMATEL BLASFEO_SMATEL
#define XVEC blasfeo_svec
#define XVECEL BLASFEO_SVECEL



#define SYRK_LN blasfeo_ssyrk_ln
#define SYRK_LN_MN blasfeo_ssyrk_ln_mn

#define REF_GELQF_WORK_SIZE blasfeo_hp_sgelqf_worksize
#define REF_GELQF blasfeo_hp_sgelqf
#define REF_ORGLQ_WORK_SIZE blasfeo_hp_sorglq_worksize
#define REF_ORGLQ blasfeo_hp_sorglq
#define REF_GELQF_PD blasfeo_hp_sgelqf_pd
#define REF_GELQF_PD_DA blasfeo_hp_sgelqf_pd_da
#define REF_GELQF_PD_LA blasfeo_hp_sgelqf_pd_la
#define REF_GELQF_PD_LLA blasfeo_hp_sgelqf_pd_lla
#define REF_GEQRF blasfeo_hp_sgeqrf
#define REF_GEQRF_WORK_SIZE blasfeo_hp_sgeqrf_worksize
#define REF_GETRF_NOPIVOT blasfeo_hp_sgetrf_np
#define REF_GETRF_ROWPIVOT blasfeo_hp_sgetrf_rp
#define REF_POTRF_L blasfeo_hp_spotrf_l
#define REF_POTRF_L_MN blasfeo_hp_spotrf_l_mn
#define REF_POTRF_U blasfeo_hp_spotrf_u
#define REF_PSTRF_L blasfeo_hp_spstrf_l
#define REF_SYRK_POTRF_LN blasfeo_hp_ssyrk_spotrf_ln
#define REF_SYRK_POTRF_LN_MN blasfeo_hp_ssyrk_spotrf_ln_mn

#define GELQF_WORK_SIZE blasfeo_sgelqf_worksize
#define GELQF blasfeo_sgelqf
#define ORGLQ_WORK_SIZE blasfeo_sorglq_worksize
#define ORGLQ blasfeo_sorglq
#define GELQF_PD blasfeo_sgelqf_pd
#define GELQF_PD_DA blasfeo_sgelqf_pd_da
#define GELQF_PD_LA blasfeo_sgelqf_pd_la
#define GELQF_PD_LLA blasfeo_sgelqf_pd_lla
#define GEQRF blasfeo_sgeqrf
#define GEQRF_WORK_SIZE blasfeo_sgeqrf_worksize
#define GETRF_NOPIVOT blasfeo_sgetrf_np
#define GETRF_ROWPIVOT blasfeo_sgetrf_rp
#define POTRF_L blasfeo_spotrf_l
#define POTRF_L_MN blasfeo_spotrf_l_mn
#define POTRF_U blasfeo_spotrf_u
#define PSTRF_L blasfeo_spstrf_l
#define SYRK_POTRF_LN blasfeo_ssyrk_spotrf_ln
#define SYRK_POTRF_LN_MN blasfeo_ssyrk_spotrf_ln_mn



#include "x_lapack_ref.c"



