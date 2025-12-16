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
#include <blasfeo_d_blasfeo_api.h>
#include <blasfeo_d_kernel.h>



#define DOUBLE_PRECISION



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
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
#define MAT blasfeo_cm_dmat
#else
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
#define MAT blasfeo_dmat
#endif
#define REAL double



#if defined(FORTRAN_BLAS_API)
#define TRMM dtrmm_
#else
#define TRMM blasfeo_blas_dtrmm
#endif




#include "xtrmm_ref.c"


