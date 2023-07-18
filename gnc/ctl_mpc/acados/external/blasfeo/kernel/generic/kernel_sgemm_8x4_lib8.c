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

#include <math.h>

#include <blasfeo_common.h>
#include <blasfeo_s_kernel.h>



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
void kernel_strmm_nt_ru_8x4_lib8(int kmax, float *alpha, float *A, float *B, float *D)
	{

	const int bs = 8;

	float
		a_0, a_1, a_2, a_3,
		a_4, a_5, a_6, a_7,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[32] = {0};
#else
	ALIGNED( float CC[32], 64 ) = {0};
#endif

	int k;

	k = 0;

	// k = 0
	if(kmax>0)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		a_4 = A[4];
		a_5 = A[5];
		a_6 = A[6];
		a_7 = A[7];

		b_0 = B[0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;
		CC[4+bs*0] += a_4 * b_0;
		CC[5+bs*0] += a_5 * b_0;
		CC[6+bs*0] += a_6 * b_0;
		CC[7+bs*0] += a_7 * b_0;

		A += bs;
		B += bs;
		k++;
		}

	// k = 1
	if(kmax>1)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		a_4 = A[4];
		a_5 = A[5];
		a_6 = A[6];
		a_7 = A[7];

		b_0 = B[0];
		b_1 = B[1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;
		CC[4+bs*0] += a_4 * b_0;
		CC[5+bs*0] += a_5 * b_0;
		CC[6+bs*0] += a_6 * b_0;
		CC[7+bs*0] += a_7 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;
		CC[4+bs*1] += a_4 * b_1;
		CC[5+bs*1] += a_5 * b_1;
		CC[6+bs*1] += a_6 * b_1;
		CC[7+bs*1] += a_7 * b_1;

		A += bs;
		B += bs;
		k++;
		}

	// k = 2
	if(kmax>2)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		a_4 = A[4];
		a_5 = A[5];
		a_6 = A[6];
		a_7 = A[7];

		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;
		CC[4+bs*0] += a_4 * b_0;
		CC[5+bs*0] += a_5 * b_0;
		CC[6+bs*0] += a_6 * b_0;
		CC[7+bs*0] += a_7 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;
		CC[4+bs*1] += a_4 * b_1;
		CC[5+bs*1] += a_5 * b_1;
		CC[6+bs*1] += a_6 * b_1;
		CC[7+bs*1] += a_7 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;
		CC[4+bs*2] += a_4 * b_2;
		CC[5+bs*2] += a_5 * b_2;
		CC[6+bs*2] += a_6 * b_2;
		CC[7+bs*2] += a_7 * b_2;

		A += bs;
		B += bs;
		k++;
		}
	
	kernel_sgemm_nt_8x4_lib8(kmax-k, alpha, A, B, alpha, CC, D);

	return;

	}
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
void kernel_strmm_nt_ru_8x4_vs_lib8(int kmax, float *alpha, float *A, float *B, float *D, int km, int kn)
	{

	const int bs = 8;

	float
		a_0, a_1, a_2, a_3,
		a_4, a_5, a_6, a_7,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[32] = {0};
#else
	ALIGNED( float CC[32], 64 ) = {0};
#endif

	int k;

	k = 0;

	// k = 0
	if(kmax>0)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		a_4 = A[4];
		a_5 = A[5];
		a_6 = A[6];
		a_7 = A[7];

		b_0 = B[0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;
		CC[4+bs*0] += a_4 * b_0;
		CC[5+bs*0] += a_5 * b_0;
		CC[6+bs*0] += a_6 * b_0;
		CC[7+bs*0] += a_7 * b_0;

		A += bs;
		B += bs;
		k++;
		}

	// k = 1
	if(kmax>1)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		a_4 = A[4];
		a_5 = A[5];
		a_6 = A[6];
		a_7 = A[7];

		b_0 = B[0];
		b_1 = B[1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;
		CC[4+bs*0] += a_4 * b_0;
		CC[5+bs*0] += a_5 * b_0;
		CC[6+bs*0] += a_6 * b_0;
		CC[7+bs*0] += a_7 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;
		CC[4+bs*1] += a_4 * b_1;
		CC[5+bs*1] += a_5 * b_1;
		CC[6+bs*1] += a_6 * b_1;
		CC[7+bs*1] += a_7 * b_1;

		A += bs;
		B += bs;
		k++;
		}

	// k = 2
	if(kmax>2)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		a_4 = A[4];
		a_5 = A[5];
		a_6 = A[6];
		a_7 = A[7];

		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;
		CC[4+bs*0] += a_4 * b_0;
		CC[5+bs*0] += a_5 * b_0;
		CC[6+bs*0] += a_6 * b_0;
		CC[7+bs*0] += a_7 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;
		CC[4+bs*1] += a_4 * b_1;
		CC[5+bs*1] += a_5 * b_1;
		CC[6+bs*1] += a_6 * b_1;
		CC[7+bs*1] += a_7 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;
		CC[4+bs*2] += a_4 * b_2;
		CC[5+bs*2] += a_5 * b_2;
		CC[6+bs*2] += a_6 * b_2;
		CC[7+bs*2] += a_7 * b_2;

		A += bs;
		B += bs;
		k++;
		}
	
	kernel_sgemm_nt_8x4_lib8(kmax-k, alpha, A, B, alpha, CC, CC);

	if(km>=8)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];
		D[4+bs*0] = CC[4+bs*0];
		D[5+bs*0] = CC[5+bs*0];
		D[6+bs*0] = CC[6+bs*0];
		D[7+bs*0] = CC[7+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];
		D[4+bs*1] = CC[4+bs*1];
		D[5+bs*1] = CC[5+bs*1];
		D[6+bs*1] = CC[6+bs*1];
		D[7+bs*1] = CC[7+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];
		D[4+bs*2] = CC[4+bs*2];
		D[5+bs*2] = CC[5+bs*2];
		D[6+bs*2] = CC[6+bs*2];
		D[7+bs*2] = CC[7+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		D[3+bs*3] = CC[3+bs*3];
		D[4+bs*3] = CC[4+bs*3];
		D[5+bs*3] = CC[5+bs*3];
		D[6+bs*3] = CC[6+bs*3];
		D[7+bs*3] = CC[7+bs*3];
		}
	else if(km>=7)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];
		D[4+bs*0] = CC[4+bs*0];
		D[5+bs*0] = CC[5+bs*0];
		D[6+bs*0] = CC[6+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];
		D[4+bs*1] = CC[4+bs*1];
		D[5+bs*1] = CC[5+bs*1];
		D[6+bs*1] = CC[6+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];
		D[4+bs*2] = CC[4+bs*2];
		D[5+bs*2] = CC[5+bs*2];
		D[6+bs*2] = CC[6+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		D[3+bs*3] = CC[3+bs*3];
		D[4+bs*3] = CC[4+bs*3];
		D[5+bs*3] = CC[5+bs*3];
		D[6+bs*3] = CC[6+bs*3];
		}
	else if(km>=6)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];
		D[4+bs*0] = CC[4+bs*0];
		D[5+bs*0] = CC[5+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];
		D[4+bs*1] = CC[4+bs*1];
		D[5+bs*1] = CC[5+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];
		D[4+bs*2] = CC[4+bs*2];
		D[5+bs*2] = CC[5+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		D[3+bs*3] = CC[3+bs*3];
		D[4+bs*3] = CC[4+bs*3];
		D[5+bs*3] = CC[5+bs*3];
		}
	else if(km>=5)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];
		D[4+bs*0] = CC[4+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];
		D[4+bs*1] = CC[4+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];
		D[4+bs*2] = CC[4+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		D[3+bs*3] = CC[3+bs*3];
		D[4+bs*3] = CC[4+bs*3];
		}
	else if(km>=4)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		D[3+bs*3] = CC[3+bs*3];
		}
	else if(km>=3)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		}
	else if(km>=2)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		}
	else //if(km>=1)
		{
		D[0+bs*0] = CC[0+bs*0];

		if(kn==1)
			return;

		D[0+bs*1] = CC[0+bs*1];

		if(kn==2)
			return;

		D[0+bs*2] = CC[0+bs*2];

		if(kn==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		}

	return;

	}
#endif

