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

#define FABS fabs
#define SQRT sqrt

#if defined(LA_EXTERNAL_BLAS_WRAPPER)
#if defined(EXTERNAL_BLAS_BLIS)
#include "blis.h"
#elif defined(EXTERNAL_BLAS_MKL)
#include "mkl.h"
#else
#include "../include/s_blas.h"
#endif
#endif

#include <blasfeo_common.h>



#define HP_CM



#define REAL float
#define XMAT blasfeo_smat
#define XVEC blasfeo_svec



#define REF_AXPY blasfeo_hp_saxpy
#define REF_AXPBY blasfeo_hp_saxpby
#define REF_VECMUL blasfeo_hp_svecmul
#define REF_VECMULACC blasfeo_hp_svecmulacc
#define REF_VECMULDOT blasfeo_hp_svecmuldot
#define REF_DOT blasfeo_hp_sdot
#define REF_ROTG blasfeo_hp_srotg
#define REF_COLROT blasfeo_hp_scolrot
#define REF_ROWROT blasfeo_hp_srowrot

#define AXPY blasfeo_saxpy
#define AXPBY blasfeo_saxpby
#define VECMUL blasfeo_svecmul
#define VECMULACC blasfeo_svecmulacc
#define VECMULDOT blasfeo_svecmuldot
#define DOT blasfeo_sdot
#define ROTG blasfeo_srotg
#define COLROT blasfeo_scolrot
#define ROWROT blasfeo_srowrot



#include "x_blas1_ref.c"



