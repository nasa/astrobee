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

#if defined(LA_EXTERNAL_BLAS_WRAPPER)
#if defined(EXTERNAL_BLAS_BLIS)
#include <blis.h>
#elif defined(EXTERNAL_BLAS_MKL)
//#include <mkl.h>
#elif defined(EXTERNAL_BLAS_ARMPL)
#include "armpl.h"
#else
#include "../include/s_blas.h"
#endif
#endif

#include <blasfeo_common.h>



#define REAL float
#define XMAT blasfeo_smat
#define XMATEL BLASFEO_SMATEL
#define XVEC blasfeo_svec
#define XMATEL BLASFEO_SMATEL



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
//#define GETF2_NOPIVOT sgetf2_nopivot
#define GETRF_NOPIVOT blasfeo_sgetrf_np
#define GETRF_ROWPIVOT blasfeo_sgetrf_rp
#define POTRF_L blasfeo_spotrf_l
#define POTRF_L_MN blasfeo_spotrf_l_mn
#define POTRF_U blasfeo_spotrf_u
#define PSTRF_L spstrf_l_libstr
#define SYRK_POTRF_LN blasfeo_ssyrk_spotrf_ln
#define SYRK_POTRF_LN_MN blasfeo_ssyrk_spotrf_ln_mn

#define COPY scopy_
#define GELQF_ sgelqf_
#define ORGLQ_ sorglq_
#define GEMM sgemm_
#define GER sger_
#define GEQRF_ sgeqrf_
#define GEQR2 sgeqr2_
#define GETRF sgetrf_
#define POTRF spotrf_
#define SCAL sscal_
#define SYRK ssyrk_
#define TRSM strsm_


#include "x_lapack_lib.c"

