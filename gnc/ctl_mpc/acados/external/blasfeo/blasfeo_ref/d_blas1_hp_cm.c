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
#include "../include/d_blas.h"
#endif
#endif

#include <blasfeo_common.h>



#define HP_CM



#define REAL double
#define XMAT blasfeo_dmat
#define XVEC blasfeo_dvec



#define REF_AXPY blasfeo_hp_daxpy
#define REF_AXPBY blasfeo_hp_daxpby
#define REF_VECMUL blasfeo_hp_dvecmul
#define REF_VECMULACC blasfeo_hp_dvecmulacc
#define REF_VECMULDOT blasfeo_hp_dvecmuldot
#define REF_DOT blasfeo_hp_ddot
#define REF_ROTG blasfeo_hp_drotg
#define REF_COLROT blasfeo_hp_dcolrot
#define REF_ROWROT blasfeo_hp_drowrot

#define AXPY blasfeo_daxpy
#define AXPBY blasfeo_daxpby
#define VECMUL blasfeo_dvecmul
#define VECMULACC blasfeo_dvecmulacc
#define VECMULDOT blasfeo_dvecmuldot
#define DOT blasfeo_ddot
#define ROTG blasfeo_drotg
#define COLROT blasfeo_dcolrot
#define ROWROT blasfeo_drowrot



#include "x_blas1_ref.c"


