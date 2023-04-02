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

#include <blasfeo_target.h>
#include <blasfeo_block_size.h>
#include <blasfeo_common.h>
#include <blasfeo_stdlib.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#include <blasfeo_memory.h>

#include <blasfeo_timing.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_dvec blasfeo_cm_dvec
#define blasfeo_hp_dger blasfeo_hp_cm_dger
#define blasfeo_dger blasfeo_cm_dger
#endif



void blasfeo_hp_dger(int m, int n, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dger (cm) %d %d %f %p %d %p %d %p %d %d %p %d %d\n", m, n, alpha, sx, xi, sy, yi, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int ldc = sC->m;
	int ldd = sD->m;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;

	int ii;

	// main loop
	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_dger_4_libc(m, &alpha, x, y+ii, C+ii*ldc, ldc, D+ii*ldd, ldd);
		}
	// clean up at the end
	if(ii<n)
		{
		kernel_dger_4_vs_libc(m, &alpha, x, y+ii, C+ii*ldc, ldc, D+ii*ldd, ldd, n-ii);
		}
	
	return;
	}




#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dger(int m, int n, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)

	{
	blasfeo_hp_dger(m, n, alpha, sx, xi, sy, yi, sC, ci, cj, sD, di, dj);
	}



#endif
