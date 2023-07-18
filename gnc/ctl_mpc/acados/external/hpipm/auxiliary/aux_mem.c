/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
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

#ifdef TARGET_AVX
#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#endif

#include "hpipm_common.h"

void hpipm_zero_memset(hpipm_size_t memsize, void *mem)
	{
	hpipm_size_t ii;
	hpipm_size_t memsize_m8 = memsize/8; // sizeof(double) is 8
	hpipm_size_t memsize_r8 = memsize%8;
	double *double_ptr = mem;
#ifdef TARGET_AVX
	__m256d
		y_zeros;
	
	y_zeros = _mm256_setzero_pd();
	if(memsize_m8>7)
		for(ii=0; ii<memsize_m8-7; ii+=8)
			{
			_mm256_storeu_pd( double_ptr+ii+0, y_zeros );
			_mm256_storeu_pd( double_ptr+ii+4, y_zeros );
			}
#else
	if(memsize_m8>7)
		for(ii=0; ii<memsize_m8-7; ii+=8)
			{
			double_ptr[ii+0] = 0.0;
			double_ptr[ii+1] = 0.0;
			double_ptr[ii+2] = 0.0;
			double_ptr[ii+3] = 0.0;
			double_ptr[ii+4] = 0.0;
			double_ptr[ii+5] = 0.0;
			double_ptr[ii+6] = 0.0;
			double_ptr[ii+7] = 0.0;
			}
#endif
	for(; ii<memsize_m8; ii++)
		{
		double_ptr[ii] = 0.0;
		}
	char *char_ptr = (char *) (&double_ptr[ii]);
	for(ii=0; ii<memsize_r8; ii++)
		{
		char_ptr[ii] = 0;
		}
	return;
	}

