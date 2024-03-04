/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
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



#include <blasfeo_d_kernel.h>



// full non-transposed
void kernel_dpack_buffer_fn(int m, int n, double *A, int lda, double *pA, int sda)
	{

	const int ps = 4;

	int ii;

	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(m, A+ii*lda, lda, pA+ii*ps, sda);
		}
	if(ii<n)
		{
		kernel_dpack_tt_4_vs_lib4(m, A+ii*lda, lda, pA+ii*ps, sda, n-ii);
		}

	return;

	}



// full transposed
void kernel_dpack_buffer_ft(int m, int n, double *A, int lda, double *pA, int sda)
	{

	const int ps = 4;

	int ii;

	for(ii=0; ii<n-3; ii+=4)
		{
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dpack_tn_4_p0_lib4(m, A+ii*lda, lda, pA+ii*sda);
#else
		kernel_dpack_tn_4_lib4(m, A+ii*lda, lda, pA+ii*sda);
#endif
		}
	if(ii<n)
		{
		kernel_dpack_tn_4_vs_lib4(m, A+ii*lda, lda, pA+ii*sda, n-ii);
		}

	return;

	}



// lower non-transposed
void kernel_dpack_buffer_ln(int m, double *A, int lda, double *pA, int sda)
	{

	const int ps = 4;

	int ii;

	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(m-ii, A+ii+ii*lda, lda, pA+ii*ps+ii*sda, sda);
		}
	if(ii<m)
		{
		kernel_dpack_tt_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pA+ii*ps+ii*sda, sda, m-ii);
		}

	return;

	}



// lower transposed
void kernel_dpack_buffer_lt(int m, double *A, int lda, double *pA, int sda)
	{

	const int ps = 4;

	int ii;

	for(ii=0; ii<m-3; ii+=4)
		{
//#if defined(TARGET_X64_INTEL_HASWELL)
//		kernel_dpack_tn_4_p0_lib4(m-ii, A+ii+ii*lda, lda, pA+ii*ps+ii*sda);
//#else
		kernel_dpack_tn_4_lib4(m-ii, A+ii+ii*lda, lda, pA+ii*ps+ii*sda);
//#endif
		}
	if(ii<m)
		{
		kernel_dpack_tn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pA+ii*ps+ii*sda, m-ii);
		}

	return;

	}



// upper transposed
void kernel_dpack_buffer_ut(int m, double *A, int lda, double *pA, int sda)
	{

	const int ps = 4;

	int ii;

	for(ii=0; ii<m-3; ii+=4)
		{
//#if defined(TARGET_X64_INTEL_HASWELL)
//		kernel_dpack_tn_4_p0_lib4(ii+4, A+ii*lda, lda, pA+ii*sda);
//#else
		kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pA+ii*sda);
//#endif
		}
	if(ii<m)
		{
		kernel_dpack_tn_4_vs_lib4(m, A+ii*lda, lda, pA+ii*sda, m-ii);
		}

	return;

	}

