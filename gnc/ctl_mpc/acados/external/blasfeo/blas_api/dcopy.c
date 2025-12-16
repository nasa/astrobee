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

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#endif

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
//#include <blasfeo_d_kernel.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_blas_dcopy dcopy_
#endif



void blasfeo_blas_dcopy(int *pn, double *x, int *pincx, double *y, int *pincy)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_blas_dcopy %d %p %d %p %d\n", *pn, x, *pincx, y, *pincy);
#endif

	int n = *pn;
	int incx = *pincx;
	int incy = *pincy;

	int ix, iy;
	int ii;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256d
		tmp;
#endif

	if(incx==1 & incy==1)
		{
		ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for(; ii<n-15; ii+=16)
			{
			tmp = _mm256_loadu_pd( &x[0] );
			_mm256_storeu_pd( &y[0], tmp );
			tmp = _mm256_loadu_pd( &x[4] );
			_mm256_storeu_pd( &y[4], tmp );
			tmp = _mm256_loadu_pd( &x[8] );
			_mm256_storeu_pd( &y[8], tmp );
			tmp = _mm256_loadu_pd( &x[12] );
			_mm256_storeu_pd( &y[12], tmp );
			x += 16;
			y += 16;
			}
		for(; ii<n-3; ii+=4)
			{
			tmp = _mm256_loadu_pd( &x[0] );
			_mm256_storeu_pd( &y[0], tmp );
			x += 4;
			y += 4;
			}
#else
		for(; ii<n-3; ii+=4)
			{
			y[0] = x[0];
			y[1] = x[1];
			y[2] = x[2];
			y[3] = x[3];
			x += 4;
			y += 4;
			}
#endif
		for(; ii<n; ii++)
			{
			y[0] = x[0];
			x += 1;
			y += 1;
			}
		}
	else
		{
		if(incx<0)
			{
			ix = - (n-1) * incx;
			}
		else
			{
			ix = 0;
			}
		if(incy<0)
			{
			iy = - (n-1) * incy;
			}
		else
			{
			iy = 0;
			}
		ii = 0;
		for(; ii<n; ii++)
			{
			y[iy] = x[ix];
			ix += incx;
			iy += incy;
			}
		}

	return;

	}
