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
#include <blasfeo_s_blasfeo_api.h>



//#if ( defined(BLAS_API) & defined(LA_HIGH_PERFORMANCE) )
//#define HP_BLAS
//#define HP_BLAS_SP
//#define HP_GEMM_NN blas_hp_sgemm_nn
//#define HP_GEMM_NT blas_hp_sgemm_nt
//#define HP_GEMM_TN blas_hp_sgemm_tn
//#define HP_GEMM_TT blas_hp_sgemm_tt
//#endif



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define GEMM_NN blasfeo_cm_sgemm_nn
#define GEMM_NT blasfeo_cm_sgemm_nt
#define GEMM_TN blasfeo_cm_sgemm_tn
#define GEMM_TT blasfeo_cm_sgemm_tt
#define MAT blasfeo_cm_smat
#else
#define GEMM_NN blasfeo_sgemm_nn
#define GEMM_NT blasfeo_sgemm_nt
#define GEMM_TN blasfeo_sgemm_tn
#define GEMM_TT blasfeo_sgemm_tt
#define MAT blasfeo_smat
#endif
#define REAL float



#if defined(FORTRAN_BLAS_API)
#define GEMM sgemm_
#else
#define GEMM blasfeo_blas_sgemm
#endif



//#ifdef HP_BLAS
//#include "../blasfeo_hp_cm/sgemm.c"
//#endif

#include "xgemm_ref.c"
