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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_AMD_BULLDOZER)
void kernel_sgemm_nt_4x4_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];
		b_3 = B[3];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;


		// k = 1

		a_0 = A[4];
		a_1 = A[5];
		a_2 = A[6];
		a_3 = A[7];

		b_0 = B[4];
		b_1 = B[5];
		b_2 = B[6];
		b_3 = B[7];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;


		// k = 2

		a_0 = A[8];
		a_1 = A[9];
		a_2 = A[10];
		a_3 = A[11];

		b_0 = B[8];
		b_1 = B[9];
		b_2 = B[10];
		b_3 = B[11];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;


		// k = 3

		a_0 = A[12];
		a_1 = A[13];
		a_2 = A[14];
		a_3 = A[15];

		b_0 = B[12];
		b_1 = B[13];
		b_2 = B[14];
		b_3 = B[15];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 16;
		B += 16;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];
		b_3 = B[3];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4;

		}
	
	D[0+bs*0] = beta[0]*C[0+bs*0] + alpha[0]*CC[0+bs*0];
	D[1+bs*0] = beta[0]*C[1+bs*0] + alpha[0]*CC[1+bs*0];
	D[2+bs*0] = beta[0]*C[2+bs*0] + alpha[0]*CC[2+bs*0];
	D[3+bs*0] = beta[0]*C[3+bs*0] + alpha[0]*CC[3+bs*0];

	D[0+bs*1] = beta[0]*C[0+bs*1] + alpha[0]*CC[0+bs*1];
	D[1+bs*1] = beta[0]*C[1+bs*1] + alpha[0]*CC[1+bs*1];
	D[2+bs*1] = beta[0]*C[2+bs*1] + alpha[0]*CC[2+bs*1];
	D[3+bs*1] = beta[0]*C[3+bs*1] + alpha[0]*CC[3+bs*1];

	D[0+bs*2] = beta[0]*C[0+bs*2] + alpha[0]*CC[0+bs*2];
	D[1+bs*2] = beta[0]*C[1+bs*2] + alpha[0]*CC[1+bs*2];
	D[2+bs*2] = beta[0]*C[2+bs*2] + alpha[0]*CC[2+bs*2];
	D[3+bs*2] = beta[0]*C[3+bs*2] + alpha[0]*CC[3+bs*2];

	D[0+bs*3] = beta[0]*C[0+bs*3] + alpha[0]*CC[0+bs*3];
	D[1+bs*3] = beta[0]*C[1+bs*3] + alpha[0]*CC[1+bs*3];
	D[2+bs*3] = beta[0]*C[2+bs*3] + alpha[0]*CC[2+bs*3];
	D[3+bs*3] = beta[0]*C[3+bs*3] + alpha[0]*CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_sgemm_nt_4x4_vs_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int km, int kn)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	kernel_sgemm_nt_4x4_lib4(kmax, alpha, A, B, beta, C, CC);

	if(km>=4)
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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_nt_4x4_gen_lib4(int kmax, float *alpha, float *A, float *B, float *beta, int offsetC, float *C0, int sdc, int offsetD, float *D0, int sdd, int m0, int m1, int n0, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float
		*C1, *D1;
	
	int k;

	if(offsetC==0)
		{
		CC[0+bs*0] = beta[0]*C0[0+bs*0];
		CC[1+bs*0] = beta[0]*C0[1+bs*0];
		CC[2+bs*0] = beta[0]*C0[2+bs*0];
		CC[3+bs*0] = beta[0]*C0[3+bs*0];

		CC[0+bs*1] = beta[0]*C0[0+bs*1];
		CC[1+bs*1] = beta[0]*C0[1+bs*1];
		CC[2+bs*1] = beta[0]*C0[2+bs*1];
		CC[3+bs*1] = beta[0]*C0[3+bs*1];

		CC[0+bs*2] = beta[0]*C0[0+bs*2];
		CC[1+bs*2] = beta[0]*C0[1+bs*2];
		CC[2+bs*2] = beta[0]*C0[2+bs*2];
		CC[3+bs*2] = beta[0]*C0[3+bs*2];

		CC[0+bs*3] = beta[0]*C0[0+bs*3];
		CC[1+bs*3] = beta[0]*C0[1+bs*3];
		CC[2+bs*3] = beta[0]*C0[2+bs*3];
		CC[3+bs*3] = beta[0]*C0[3+bs*3];
		}
	else if(offsetC==1)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[1+bs*0];
		CC[1+bs*0] = beta[0]*C0[2+bs*0];
		CC[2+bs*0] = beta[0]*C0[3+bs*0];
		CC[3+bs*0] = beta[0]*C1[0+bs*0];

		CC[0+bs*1] = beta[0]*C0[1+bs*1];
		CC[1+bs*1] = beta[0]*C0[2+bs*1];
		CC[2+bs*1] = beta[0]*C0[3+bs*1];
		CC[3+bs*1] = beta[0]*C1[0+bs*1];

		CC[0+bs*2] = beta[0]*C0[1+bs*2];
		CC[1+bs*2] = beta[0]*C0[2+bs*2];
		CC[2+bs*2] = beta[0]*C0[3+bs*2];
		CC[3+bs*2] = beta[0]*C1[0+bs*2];

		CC[0+bs*3] = beta[0]*C0[1+bs*3];
		CC[1+bs*3] = beta[0]*C0[2+bs*3];
		CC[2+bs*3] = beta[0]*C0[3+bs*3];
		CC[3+bs*3] = beta[0]*C1[0+bs*3];
		}
	else if(offsetC==2)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[2+bs*0];
		CC[1+bs*0] = beta[0]*C0[3+bs*0];
		CC[2+bs*0] = beta[0]*C1[0+bs*0];
		CC[3+bs*0] = beta[0]*C1[1+bs*0];

		CC[0+bs*1] = beta[0]*C0[2+bs*1];
		CC[1+bs*1] = beta[0]*C0[3+bs*1];
		CC[2+bs*1] = beta[0]*C1[0+bs*1];
		CC[3+bs*1] = beta[0]*C1[1+bs*1];

		CC[0+bs*2] = beta[0]*C0[2+bs*2];
		CC[1+bs*2] = beta[0]*C0[3+bs*2];
		CC[2+bs*2] = beta[0]*C1[0+bs*2];
		CC[3+bs*2] = beta[0]*C1[1+bs*2];

		CC[0+bs*3] = beta[0]*C0[2+bs*3];
		CC[1+bs*3] = beta[0]*C0[3+bs*3];
		CC[2+bs*3] = beta[0]*C1[0+bs*3];
		CC[3+bs*3] = beta[0]*C1[1+bs*3];
		}
	else //if(offsetC==3)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[3+bs*0];
		CC[1+bs*0] = beta[0]*C1[0+bs*0];
		CC[2+bs*0] = beta[0]*C1[1+bs*0];
		CC[3+bs*0] = beta[0]*C1[2+bs*0];

		CC[0+bs*1] = beta[0]*C0[3+bs*1];
		CC[1+bs*1] = beta[0]*C1[0+bs*1];
		CC[2+bs*1] = beta[0]*C1[1+bs*1];
		CC[3+bs*1] = beta[0]*C1[2+bs*1];

		CC[0+bs*2] = beta[0]*C0[3+bs*2];
		CC[1+bs*2] = beta[0]*C1[0+bs*2];
		CC[2+bs*2] = beta[0]*C1[1+bs*2];
		CC[3+bs*2] = beta[0]*C1[2+bs*2];

		CC[0+bs*3] = beta[0]*C0[3+bs*3];
		CC[1+bs*3] = beta[0]*C1[0+bs*3];
		CC[2+bs*3] = beta[0]*C1[1+bs*3];
		CC[3+bs*3] = beta[0]*C1[2+bs*3];
		}
	
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, alpha, A, B, &beta1, CC, CC);

	// shift sol for cols
	if(n0>0)
		{
		if(n0==1)
			{
			CC[0+bs*0] = CC[0+bs*1];
			CC[1+bs*0] = CC[1+bs*1];
			CC[2+bs*0] = CC[2+bs*1];
			CC[3+bs*0] = CC[3+bs*1];

			CC[0+bs*1] = CC[0+bs*2];
			CC[1+bs*1] = CC[1+bs*2];
			CC[2+bs*1] = CC[2+bs*2];
			CC[3+bs*1] = CC[3+bs*2];

			CC[0+bs*2] = CC[0+bs*3];
			CC[1+bs*2] = CC[1+bs*3];
			CC[2+bs*2] = CC[2+bs*3];
			CC[3+bs*2] = CC[3+bs*3];

			D0 += 1*bs;
			}
		else if(n0==2)
			{
			CC[0+bs*0] = CC[0+bs*2];
			CC[1+bs*0] = CC[1+bs*2];
			CC[2+bs*0] = CC[2+bs*2];
			CC[3+bs*0] = CC[3+bs*2];

			CC[0+bs*1] = CC[0+bs*3];
			CC[1+bs*1] = CC[1+bs*3];
			CC[2+bs*1] = CC[2+bs*3];
			CC[3+bs*1] = CC[3+bs*3];

			D0 += 2*bs;
			}
		else //if(n0==3)
			{
			CC[0+bs*0] = CC[0+bs*3];
			CC[1+bs*0] = CC[1+bs*3];
			CC[2+bs*0] = CC[2+bs*3];
			CC[3+bs*0] = CC[3+bs*3];

			D0 += 3*bs;
			}
		}

	n1 = 4<n1 ? 4 : n1;
	int kn = n1 - n0;

	if(offsetD==0)
		{
		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[0+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[1+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D0[2+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D0[3+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[0+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[1+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D0[2+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D0[3+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[0+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[1+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D0[2+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D0[3+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[0+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[1+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D0[2+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D0[3+bs*3] = CC[3+bs*3];
		}
	else if(offsetD==1)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[1+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[2+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D0[3+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[0+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[1+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[2+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D0[3+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[0+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[1+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[2+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D0[3+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[0+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[1+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[2+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D0[3+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[0+bs*3] = CC[3+bs*3];
		}
	else if(offsetD==2)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[2+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[3+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D1[0+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[1+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[2+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[3+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D1[0+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[1+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[2+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[3+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D1[0+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[1+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[2+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[3+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D1[0+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[1+bs*3] = CC[3+bs*3];
		}
	else //if(offsetD==3)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[3+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D1[0+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D1[1+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[2+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[3+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D1[0+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D1[1+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[2+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[3+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D1[0+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D1[1+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[2+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[3+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D1[0+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D1[1+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[2+bs*3] = CC[3+bs*3];
		}

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X64_AMD_BULLDOZER)
void kernel_sgemm_nn_4x4_lib4(int kmax, float *alpha, float *A, int offsetB, float *B, int sdb, float *beta, float *C, float *D)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float
		*C1, *D1;
	
	int k;

	k = 0;
	if(offsetB!=0)
		{
		if(offsetB==1)
			{

			B += 1;

			a_0 = A[0];
			a_1 = A[1];
			a_2 = A[2];
			a_3 = A[3];

			b_0 = B[0];
			b_1 = B[4];
			b_2 = B[8];
			b_3 = B[12];

			CC[0+bs*0] += a_0 * b_0;
			CC[1+bs*0] += a_1 * b_0;
			CC[2+bs*0] += a_2 * b_0;
			CC[3+bs*0] += a_3 * b_0;

			CC[0+bs*1] += a_0 * b_1;
			CC[1+bs*1] += a_1 * b_1;
			CC[2+bs*1] += a_2 * b_1;
			CC[3+bs*1] += a_3 * b_1;

			CC[0+bs*2] += a_0 * b_2;
			CC[1+bs*2] += a_1 * b_2;
			CC[2+bs*2] += a_2 * b_2;
			CC[3+bs*2] += a_3 * b_2;

			CC[0+bs*3] += a_0 * b_3;
			CC[1+bs*3] += a_1 * b_3;
			CC[2+bs*3] += a_2 * b_3;
			CC[3+bs*3] += a_3 * b_3;

			A += 4;
			B += 1;
			k += 1;

			if(k>=kmax)
				goto scale;

			a_0 = A[0];
			a_1 = A[1];
			a_2 = A[2];
			a_3 = A[3];

			b_0 = B[0];
			b_1 = B[4];
			b_2 = B[8];
			b_3 = B[12];

			CC[0+bs*0] += a_0 * b_0;
			CC[1+bs*0] += a_1 * b_0;
			CC[2+bs*0] += a_2 * b_0;
			CC[3+bs*0] += a_3 * b_0;

			CC[0+bs*1] += a_0 * b_1;
			CC[1+bs*1] += a_1 * b_1;
			CC[2+bs*1] += a_2 * b_1;
			CC[3+bs*1] += a_3 * b_1;

			CC[0+bs*2] += a_0 * b_2;
			CC[1+bs*2] += a_1 * b_2;
			CC[2+bs*2] += a_2 * b_2;
			CC[3+bs*2] += a_3 * b_2;

			CC[0+bs*3] += a_0 * b_3;
			CC[1+bs*3] += a_1 * b_3;
			CC[2+bs*3] += a_2 * b_3;
			CC[3+bs*3] += a_3 * b_3;

			A += 4;
			B += 1;
			k += 1;

			if(k>=kmax)
				goto scale;

			a_0 = A[0];
			a_1 = A[1];
			a_2 = A[2];
			a_3 = A[3];

			b_0 = B[0];
			b_1 = B[4];
			b_2 = B[8];
			b_3 = B[12];

			CC[0+bs*0] += a_0 * b_0;
			CC[1+bs*0] += a_1 * b_0;
			CC[2+bs*0] += a_2 * b_0;
			CC[3+bs*0] += a_3 * b_0;

			CC[0+bs*1] += a_0 * b_1;
			CC[1+bs*1] += a_1 * b_1;
			CC[2+bs*1] += a_2 * b_1;
			CC[3+bs*1] += a_3 * b_1;

			CC[0+bs*2] += a_0 * b_2;
			CC[1+bs*2] += a_1 * b_2;
			CC[2+bs*2] += a_2 * b_2;
			CC[3+bs*2] += a_3 * b_2;

			CC[0+bs*3] += a_0 * b_3;
			CC[1+bs*3] += a_1 * b_3;
			CC[2+bs*3] += a_2 * b_3;
			CC[3+bs*3] += a_3 * b_3;

			A += 4;
			B += 1;
			B += bs*(sdb-1);
			k += 1;

			}
		else if(offsetB==2)
			{

			B += 2;

			a_0 = A[0];
			a_1 = A[1];
			a_2 = A[2];
			a_3 = A[3];

			b_0 = B[0];
			b_1 = B[4];
			b_2 = B[8];
			b_3 = B[12];

			CC[0+bs*0] += a_0 * b_0;
			CC[1+bs*0] += a_1 * b_0;
			CC[2+bs*0] += a_2 * b_0;
			CC[3+bs*0] += a_3 * b_0;

			CC[0+bs*1] += a_0 * b_1;
			CC[1+bs*1] += a_1 * b_1;
			CC[2+bs*1] += a_2 * b_1;
			CC[3+bs*1] += a_3 * b_1;

			CC[0+bs*2] += a_0 * b_2;
			CC[1+bs*2] += a_1 * b_2;
			CC[2+bs*2] += a_2 * b_2;
			CC[3+bs*2] += a_3 * b_2;

			CC[0+bs*3] += a_0 * b_3;
			CC[1+bs*3] += a_1 * b_3;
			CC[2+bs*3] += a_2 * b_3;
			CC[3+bs*3] += a_3 * b_3;

			A += 4;
			B += 1;
			k += 1;

			if(k>=kmax)
				goto scale;

			a_0 = A[0];
			a_1 = A[1];
			a_2 = A[2];
			a_3 = A[3];

			b_0 = B[0];
			b_1 = B[4];
			b_2 = B[8];
			b_3 = B[12];

			CC[0+bs*0] += a_0 * b_0;
			CC[1+bs*0] += a_1 * b_0;
			CC[2+bs*0] += a_2 * b_0;
			CC[3+bs*0] += a_3 * b_0;

			CC[0+bs*1] += a_0 * b_1;
			CC[1+bs*1] += a_1 * b_1;
			CC[2+bs*1] += a_2 * b_1;
			CC[3+bs*1] += a_3 * b_1;

			CC[0+bs*2] += a_0 * b_2;
			CC[1+bs*2] += a_1 * b_2;
			CC[2+bs*2] += a_2 * b_2;
			CC[3+bs*2] += a_3 * b_2;

			CC[0+bs*3] += a_0 * b_3;
			CC[1+bs*3] += a_1 * b_3;
			CC[2+bs*3] += a_2 * b_3;
			CC[3+bs*3] += a_3 * b_3;

			A += 4;
			B += 1;
			B += bs*(sdb-1);
			k += 1;

			}
		else // if(offsetB==3)
			{

			B += 3;

			a_0 = A[0];
			a_1 = A[1];
			a_2 = A[2];
			a_3 = A[3];

			b_0 = B[0];
			b_1 = B[4];
			b_2 = B[8];
			b_3 = B[12];

			CC[0+bs*0] += a_0 * b_0;
			CC[1+bs*0] += a_1 * b_0;
			CC[2+bs*0] += a_2 * b_0;
			CC[3+bs*0] += a_3 * b_0;

			CC[0+bs*1] += a_0 * b_1;
			CC[1+bs*1] += a_1 * b_1;
			CC[2+bs*1] += a_2 * b_1;
			CC[3+bs*1] += a_3 * b_1;

			CC[0+bs*2] += a_0 * b_2;
			CC[1+bs*2] += a_1 * b_2;
			CC[2+bs*2] += a_2 * b_2;
			CC[3+bs*2] += a_3 * b_2;

			CC[0+bs*3] += a_0 * b_3;
			CC[1+bs*3] += a_1 * b_3;
			CC[2+bs*3] += a_2 * b_3;
			CC[3+bs*3] += a_3 * b_3;

			A += 4;
			B += 1;
			B += bs*(sdb-1);
			k += 1;

			}
		}
	for(; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[4];
		b_2 = B[8];
		b_3 = B[12];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;


		// k = 1

		a_0 = A[4];
		a_1 = A[5];
		a_2 = A[6];
		a_3 = A[7];

		b_0 = B[1];
		b_1 = B[5];
		b_2 = B[9];
		b_3 = B[13];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;


		// k = 2

		a_0 = A[8];
		a_1 = A[9];
		a_2 = A[10];
		a_3 = A[11];

		b_0 = B[2];
		b_1 = B[6];
		b_2 = B[10];
		b_3 = B[14];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;


		// k = 3

		a_0 = A[12];
		a_1 = A[13];
		a_2 = A[14];
		a_3 = A[15];

		b_0 = B[3];
		b_1 = B[7];
		b_2 = B[11];
		b_3 = B[15];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[4];
		b_2 = B[8];
		b_3 = B[12];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;

		}	
	
	scale:

	D[0+bs*0] = beta[0]*C[0+bs*0] + alpha[0]*CC[0+bs*0];
	D[1+bs*0] = beta[0]*C[1+bs*0] + alpha[0]*CC[1+bs*0];
	D[2+bs*0] = beta[0]*C[2+bs*0] + alpha[0]*CC[2+bs*0];
	D[3+bs*0] = beta[0]*C[3+bs*0] + alpha[0]*CC[3+bs*0];

	D[0+bs*1] = beta[0]*C[0+bs*1] + alpha[0]*CC[0+bs*1];
	D[1+bs*1] = beta[0]*C[1+bs*1] + alpha[0]*CC[1+bs*1];
	D[2+bs*1] = beta[0]*C[2+bs*1] + alpha[0]*CC[2+bs*1];
	D[3+bs*1] = beta[0]*C[3+bs*1] + alpha[0]*CC[3+bs*1];

	D[0+bs*2] = beta[0]*C[0+bs*2] + alpha[0]*CC[0+bs*2];
	D[1+bs*2] = beta[0]*C[1+bs*2] + alpha[0]*CC[1+bs*2];
	D[2+bs*2] = beta[0]*C[2+bs*2] + alpha[0]*CC[2+bs*2];
	D[3+bs*2] = beta[0]*C[3+bs*2] + alpha[0]*CC[3+bs*2];

	D[0+bs*3] = beta[0]*C[0+bs*3] + alpha[0]*CC[0+bs*3];
	D[1+bs*3] = beta[0]*C[1+bs*3] + alpha[0]*CC[1+bs*3];
	D[2+bs*3] = beta[0]*C[2+bs*3] + alpha[0]*CC[2+bs*3];
	D[3+bs*3] = beta[0]*C[3+bs*3] + alpha[0]*CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_sgemm_nn_4x4_vs_lib4(int kmax, float *alpha, float *A, int offsetB, float *B, int sdb, float *beta, float *C, float *D, int km, int kn)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	kernel_sgemm_nn_4x4_lib4(kmax, alpha, A, offsetB, B, sdb, beta, C, CC);
	
	if(km>=4)
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



#if defined(TARGET_GENERIC) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_nn_4x4_gen_lib4(int kmax, float *alpha, float *A, int offsetB, float *B, int sdb, float *beta, int offsetC, float *C0, int sdc, int offsetD, float *D0, int sdd, int m0, int m1, int n0, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float
		*C1, *D1;
	
	if(offsetC==0)
		{
		CC[0+bs*0] = beta[0]*C0[0+bs*0];
		CC[1+bs*0] = beta[0]*C0[1+bs*0];
		CC[2+bs*0] = beta[0]*C0[2+bs*0];
		CC[3+bs*0] = beta[0]*C0[3+bs*0];

		CC[0+bs*1] = beta[0]*C0[0+bs*1];
		CC[1+bs*1] = beta[0]*C0[1+bs*1];
		CC[2+bs*1] = beta[0]*C0[2+bs*1];
		CC[3+bs*1] = beta[0]*C0[3+bs*1];

		CC[0+bs*2] = beta[0]*C0[0+bs*2];
		CC[1+bs*2] = beta[0]*C0[1+bs*2];
		CC[2+bs*2] = beta[0]*C0[2+bs*2];
		CC[3+bs*2] = beta[0]*C0[3+bs*2];

		CC[0+bs*3] = beta[0]*C0[0+bs*3];
		CC[1+bs*3] = beta[0]*C0[1+bs*3];
		CC[2+bs*3] = beta[0]*C0[2+bs*3];
		CC[3+bs*3] = beta[0]*C0[3+bs*3];
		}
	else if(offsetC==1)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[1+bs*0];
		CC[1+bs*0] = beta[0]*C0[2+bs*0];
		CC[2+bs*0] = beta[0]*C0[3+bs*0];
		CC[3+bs*0] = beta[0]*C1[0+bs*0];

		CC[0+bs*1] = beta[0]*C0[1+bs*1];
		CC[1+bs*1] = beta[0]*C0[2+bs*1];
		CC[2+bs*1] = beta[0]*C0[3+bs*1];
		CC[3+bs*1] = beta[0]*C1[0+bs*1];

		CC[0+bs*2] = beta[0]*C0[1+bs*2];
		CC[1+bs*2] = beta[0]*C0[2+bs*2];
		CC[2+bs*2] = beta[0]*C0[3+bs*2];
		CC[3+bs*2] = beta[0]*C1[0+bs*2];

		CC[0+bs*3] = beta[0]*C0[1+bs*3];
		CC[1+bs*3] = beta[0]*C0[2+bs*3];
		CC[2+bs*3] = beta[0]*C0[3+bs*3];
		CC[3+bs*3] = beta[0]*C1[0+bs*3];
		}
	else if(offsetC==2)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[2+bs*0];
		CC[1+bs*0] = beta[0]*C0[3+bs*0];
		CC[2+bs*0] = beta[0]*C1[0+bs*0];
		CC[3+bs*0] = beta[0]*C1[1+bs*0];

		CC[0+bs*1] = beta[0]*C0[2+bs*1];
		CC[1+bs*1] = beta[0]*C0[3+bs*1];
		CC[2+bs*1] = beta[0]*C1[0+bs*1];
		CC[3+bs*1] = beta[0]*C1[1+bs*1];

		CC[0+bs*2] = beta[0]*C0[2+bs*2];
		CC[1+bs*2] = beta[0]*C0[3+bs*2];
		CC[2+bs*2] = beta[0]*C1[0+bs*2];
		CC[3+bs*2] = beta[0]*C1[1+bs*2];

		CC[0+bs*3] = beta[0]*C0[2+bs*3];
		CC[1+bs*3] = beta[0]*C0[3+bs*3];
		CC[2+bs*3] = beta[0]*C1[0+bs*3];
		CC[3+bs*3] = beta[0]*C1[1+bs*3];
		}
	else //if(offsetC==3)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[3+bs*0];
		CC[1+bs*0] = beta[0]*C1[0+bs*0];
		CC[2+bs*0] = beta[0]*C1[1+bs*0];
		CC[3+bs*0] = beta[0]*C1[2+bs*0];

		CC[0+bs*1] = beta[0]*C0[3+bs*1];
		CC[1+bs*1] = beta[0]*C1[0+bs*1];
		CC[2+bs*1] = beta[0]*C1[1+bs*1];
		CC[3+bs*1] = beta[0]*C1[2+bs*1];

		CC[0+bs*2] = beta[0]*C0[3+bs*2];
		CC[1+bs*2] = beta[0]*C1[0+bs*2];
		CC[2+bs*2] = beta[0]*C1[1+bs*2];
		CC[3+bs*2] = beta[0]*C1[2+bs*2];

		CC[0+bs*3] = beta[0]*C0[3+bs*3];
		CC[1+bs*3] = beta[0]*C1[0+bs*3];
		CC[2+bs*3] = beta[0]*C1[1+bs*3];
		CC[3+bs*3] = beta[0]*C1[2+bs*3];
		}
	
	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, alpha, A, offsetB, B, sdb, &beta1, CC, CC);

	// shift sol for cols
	if(n0>0)
		{
		if(n0==1)
			{
			CC[0+bs*0] = CC[0+bs*1];
			CC[1+bs*0] = CC[1+bs*1];
			CC[2+bs*0] = CC[2+bs*1];
			CC[3+bs*0] = CC[3+bs*1];

			CC[0+bs*1] = CC[0+bs*2];
			CC[1+bs*1] = CC[1+bs*2];
			CC[2+bs*1] = CC[2+bs*2];
			CC[3+bs*1] = CC[3+bs*2];

			CC[0+bs*2] = CC[0+bs*3];
			CC[1+bs*2] = CC[1+bs*3];
			CC[2+bs*2] = CC[2+bs*3];
			CC[3+bs*2] = CC[3+bs*3];

			D0 += 1*bs;
			}
		else if(n0==2)
			{
			CC[0+bs*0] = CC[0+bs*2];
			CC[1+bs*0] = CC[1+bs*2];
			CC[2+bs*0] = CC[2+bs*2];
			CC[3+bs*0] = CC[3+bs*2];

			CC[0+bs*1] = CC[0+bs*3];
			CC[1+bs*1] = CC[1+bs*3];
			CC[2+bs*1] = CC[2+bs*3];
			CC[3+bs*1] = CC[3+bs*3];

			D0 += 2*bs;
			}
		else //if(n0==3)
			{
			CC[0+bs*0] = CC[0+bs*3];
			CC[1+bs*0] = CC[1+bs*3];
			CC[2+bs*0] = CC[2+bs*3];
			CC[3+bs*0] = CC[3+bs*3];

			D0 += 3*bs;
			}
		}

	n1 = 4<n1 ? 4 : n1;
	int kn = n1 - n0;

	if(offsetD==0)
		{
		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[0+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[1+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D0[2+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D0[3+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[0+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[1+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D0[2+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D0[3+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[0+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[1+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D0[2+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D0[3+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[0+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[1+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D0[2+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D0[3+bs*3] = CC[3+bs*3];
		}
	else if(offsetD==1)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[1+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[2+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D0[3+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[0+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[1+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[2+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D0[3+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[0+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[1+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[2+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D0[3+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[0+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[1+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[2+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D0[3+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[0+bs*3] = CC[3+bs*3];
		}
	else if(offsetD==2)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[2+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[3+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D1[0+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[1+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[2+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[3+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D1[0+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[1+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[2+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[3+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D1[0+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[1+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[2+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[3+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D1[0+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[1+bs*3] = CC[3+bs*3];
		}
	else //if(offsetD==3)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[3+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D1[0+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D1[1+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[2+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[3+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D1[0+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D1[1+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[2+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[3+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D1[0+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D1[1+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[2+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[3+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D1[0+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D1[1+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[2+bs*3] = CC[3+bs*3];
		}

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_ssyrk_nt_l_4x4_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	kernel_sgemm_nt_4x4_lib4(kmax, alpha, A, B, beta, C, CC);
	
	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_ssyrk_nt_l_4x4_vs_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int km, int kn)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	kernel_sgemm_nt_4x4_lib4(kmax, alpha, A, B, beta, C, CC);
	
	if(km>=4)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];

		if(kn==1)
			return;

		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];

		if(kn==2)
			return;

		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];

		if(kn==3)
			return;

		D[3+bs*3] = CC[3+bs*3];
		}
	else if(km>=3)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];

		if(kn==1)
			return;

		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];

		if(kn==2)
			return;

		D[2+bs*2] = CC[2+bs*2];
		}
	else if(km>=2)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];

		if(kn==1)
			return;

		D[1+bs*1] = CC[1+bs*1];
		}
	else //if(km>=1)
		{
		D[0+bs*0] = CC[0+bs*0];
		}
	
	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_ssyrk_nt_l_4x4_gen_lib4(int kmax, float *alpha, float *A, float *B, float *beta, int offsetC, float *C0, int sdc, int offsetD, float *D0, int sdd, int m0, int m1, int n0, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float
		*C1, *D1;

	if(offsetC==0)
		{
		CC[0+bs*0] = beta[0]*C0[0+bs*0];
		CC[1+bs*0] = beta[0]*C0[1+bs*0];
		CC[2+bs*0] = beta[0]*C0[2+bs*0];
		CC[3+bs*0] = beta[0]*C0[3+bs*0];

		CC[1+bs*1] = beta[0]*C0[1+bs*1];
		CC[2+bs*1] = beta[0]*C0[2+bs*1];
		CC[3+bs*1] = beta[0]*C0[3+bs*1];

		CC[2+bs*2] = beta[0]*C0[2+bs*2];
		CC[3+bs*2] = beta[0]*C0[3+bs*2];

		CC[3+bs*3] = beta[0]*C0[3+bs*3];
		}
	else if(offsetC==1)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[1+bs*0];
		CC[1+bs*0] = beta[0]*C0[2+bs*0];
		CC[2+bs*0] = beta[0]*C0[3+bs*0];
		CC[3+bs*0] = beta[0]*C1[0+bs*0];

		CC[1+bs*1] = beta[0]*C0[2+bs*1];
		CC[2+bs*1] = beta[0]*C0[3+bs*1];
		CC[3+bs*1] = beta[0]*C1[0+bs*1];

		CC[2+bs*2] = beta[0]*C0[3+bs*2];
		CC[3+bs*2] = beta[0]*C1[0+bs*2];

		CC[3+bs*3] = beta[0]*C1[0+bs*3];
		}
	else if(offsetC==2)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[2+bs*0];
		CC[1+bs*0] = beta[0]*C0[3+bs*0];
		CC[2+bs*0] = beta[0]*C1[0+bs*0];
		CC[3+bs*0] = beta[0]*C1[1+bs*0];

		CC[1+bs*1] = beta[0]*C0[3+bs*1];
		CC[2+bs*1] = beta[0]*C1[0+bs*1];
		CC[3+bs*1] = beta[0]*C1[1+bs*1];

		CC[2+bs*2] = beta[0]*C1[0+bs*2];
		CC[3+bs*2] = beta[0]*C1[1+bs*2];

		CC[3+bs*3] = beta[0]*C1[1+bs*3];
		}
	else //if(offsetC==3)
		{
		C1 = C0 + sdc*bs;

		CC[0+bs*0] = beta[0]*C0[3+bs*0];
		CC[1+bs*0] = beta[0]*C1[0+bs*0];
		CC[2+bs*0] = beta[0]*C1[1+bs*0];
		CC[3+bs*0] = beta[0]*C1[2+bs*0];

		CC[1+bs*1] = beta[0]*C1[0+bs*1];
		CC[2+bs*1] = beta[0]*C1[1+bs*1];
		CC[3+bs*1] = beta[0]*C1[2+bs*1];

		CC[2+bs*2] = beta[0]*C1[1+bs*2];
		CC[3+bs*2] = beta[0]*C1[2+bs*2];

		CC[3+bs*3] = beta[0]*C1[2+bs*3];
		}
	
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, alpha, A, B, &beta1, CC, CC);

	// shift sol for cols
	if(n0>0)
		{
		if(n0==1)
			{
			CC[0+bs*0] = CC[0+bs*1];
			CC[1+bs*0] = CC[1+bs*1];
			CC[2+bs*0] = CC[2+bs*1];
			CC[3+bs*0] = CC[3+bs*1];

			CC[0+bs*1] = CC[0+bs*2];
			CC[1+bs*1] = CC[1+bs*2];
			CC[2+bs*1] = CC[2+bs*2];
			CC[3+bs*1] = CC[3+bs*2];

			CC[0+bs*2] = CC[0+bs*3];
			CC[1+bs*2] = CC[1+bs*3];
			CC[2+bs*2] = CC[2+bs*3];
			CC[3+bs*2] = CC[3+bs*3];

			D0 += 1*bs;
			}
		else if(n0==2)
			{
			CC[0+bs*0] = CC[0+bs*2];
			CC[1+bs*0] = CC[1+bs*2];
			CC[2+bs*0] = CC[2+bs*2];
			CC[3+bs*0] = CC[3+bs*2];

			CC[0+bs*1] = CC[0+bs*3];
			CC[1+bs*1] = CC[1+bs*3];
			CC[2+bs*1] = CC[2+bs*3];
			CC[3+bs*1] = CC[3+bs*3];

			D0 += 2*bs;
			}
		else //if(n0==3)
			{
			CC[0+bs*0] = CC[0+bs*3];
			CC[1+bs*0] = CC[1+bs*3];
			CC[2+bs*0] = CC[2+bs*3];
			CC[3+bs*0] = CC[3+bs*3];

			D0 += 3*bs;
			}
		}

	n1 = 4<n1 ? 4 : n1;
	int kn = n1 - n0;

	if(offsetD==0)
		{
		if(m0<=0)
			{
			if(kn<=0)
				return;

			if(m1>0) D0[0+bs*0] = CC[0+bs*0];
			if(m1>1) D0[1+bs*0] = CC[1+bs*0];
			if(m1>2) D0[2+bs*0] = CC[2+bs*0];
			if(m1>3) D0[3+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>1) D0[1+bs*1] = CC[1+bs*1];
			if(m1>2) D0[2+bs*1] = CC[2+bs*1];
			if(m1>3) D0[3+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>2) D0[2+bs*2] = CC[2+bs*2];
			if(m1>3) D0[3+bs*2] = CC[3+bs*2];

			if(kn<=3)
				return;

			if(m1>3) D0[3+bs*3] = CC[3+bs*3];
			}
		else if(m0<=1)
			{
			if(kn<=0)
				return;

			if(m1>1) D0[1+bs*0] = CC[1+bs*0];
			if(m1>2) D0[2+bs*0] = CC[2+bs*0];
			if(m1>3) D0[3+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>2) D0[2+bs*1] = CC[2+bs*1];
			if(m1>3) D0[3+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>3) D0[3+bs*2] = CC[3+bs*2];
			}
		else if(m0<=2)
			{
			if(kn<=0)
				return;

			if(m1>2) D0[2+bs*0] = CC[2+bs*0];
			if(m1>3) D0[3+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>3) D0[3+bs*1] = CC[3+bs*1];
			}
		else if(m0<=3)
			{
			if(kn<=0)
				return;

			if(m1>3) D0[3+bs*0] = CC[3+bs*0];
			}
		}
	else if(offsetD==1)
		{
		D1 = D0 + sdd*bs;
		if(m0<=0)
			{
			if(kn<=0)
				return;

			if(m1>0) D0[1+bs*0] = CC[0+bs*0];
			if(m1>1) D0[2+bs*0] = CC[1+bs*0];
			if(m1>2) D0[3+bs*0] = CC[2+bs*0];
			if(m1>3) D1[0+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>1) D0[2+bs*1] = CC[1+bs*1];
			if(m1>2) D0[3+bs*1] = CC[2+bs*1];
			if(m1>3) D1[0+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>2) D0[3+bs*2] = CC[2+bs*2];
			if(m1>3) D1[0+bs*2] = CC[3+bs*2];

			if(kn<=3)
				return;

			if(m1>3) D1[0+bs*3] = CC[3+bs*3];
			}
		else if(m0<=1)
			{
			if(kn<=0)
				return;

			if(m1>1) D0[2+bs*0] = CC[1+bs*0];
			if(m1>2) D0[3+bs*0] = CC[2+bs*0];
			if(m1>3) D1[0+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>2) D0[3+bs*1] = CC[2+bs*1];
			if(m1>3) D1[0+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>3) D1[0+bs*2] = CC[3+bs*2];
			}
		else if(m0<=2)
			{
			if(kn<=0)
				return;

			if(m1>2) D0[3+bs*0] = CC[2+bs*0];
			if(m1>3) D1[0+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>3) D1[0+bs*1] = CC[3+bs*1];
			}
		else if(m0<=3)
			{
			if(kn<=0)
				return;

			if(m1>3) D1[0+bs*0] = CC[3+bs*0];
			}
		}
	else if(offsetD==2)
		{
		D1 = D0 + sdd*bs;
		if(m0<=0)
			{
			if(kn<=0)
				return;

			if(m1>0) D0[2+bs*0] = CC[0+bs*0];
			if(m1>1) D0[3+bs*0] = CC[1+bs*0];
			if(m1>2) D1[0+bs*0] = CC[2+bs*0];
			if(m1>3) D1[1+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>1) D0[3+bs*1] = CC[1+bs*1];
			if(m1>2) D1[0+bs*1] = CC[2+bs*1];
			if(m1>3) D1[1+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>2) D1[0+bs*2] = CC[2+bs*2];
			if(m1>3) D1[1+bs*2] = CC[3+bs*2];

			if(kn<=3)
				return;

			if(m1>3) D1[1+bs*3] = CC[3+bs*3];
			}
		else if(m0<=1)
			{
			if(kn<=0)
				return;

			if(m1>1) D0[3+bs*0] = CC[1+bs*0];
			if(m1>2) D1[0+bs*0] = CC[2+bs*0];
			if(m1>3) D1[1+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>2) D1[0+bs*1] = CC[2+bs*1];
			if(m1>3) D1[1+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>3) D1[1+bs*2] = CC[3+bs*2];
			}
		else if(m0<=2)
			{
			if(kn<=0)
				return;

			if(m1>2) D1[0+bs*0] = CC[2+bs*0];
			if(m1>3) D1[1+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>3) D1[1+bs*1] = CC[3+bs*1];
			}
		else if(m0<=3)
			{
			if(kn<=0)
				return;

			if(m1>3) D1[1+bs*0] = CC[3+bs*0];
			}
		}
	else //if(offsetD==3)
		{
		D1 = D0 + sdd*bs;
		if(m0<=0)
			{
			if(kn<=0)
				return;

			if(m1>0) D0[3+bs*0] = CC[0+bs*0];
			if(m1>1) D1[0+bs*0] = CC[1+bs*0];
			if(m1>2) D1[1+bs*0] = CC[2+bs*0];
			if(m1>3) D1[2+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>1) D1[0+bs*1] = CC[1+bs*1];
			if(m1>2) D1[1+bs*1] = CC[2+bs*1];
			if(m1>3) D1[2+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>2) D1[1+bs*2] = CC[2+bs*2];
			if(m1>3) D1[2+bs*2] = CC[3+bs*2];

			if(kn<=3)
				return;

			if(m1>3) D1[2+bs*3] = CC[3+bs*3];
			}
		else if(m0<=1)
			{
			if(kn<=0)
				return;

			if(m1>1) D1[0+bs*0] = CC[1+bs*0];
			if(m1>2) D1[1+bs*0] = CC[2+bs*0];
			if(m1>3) D1[2+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>2) D1[1+bs*1] = CC[2+bs*1];
			if(m1>3) D1[2+bs*1] = CC[3+bs*1];

			if(kn<=2)
				return;

			if(m1>3) D1[2+bs*2] = CC[3+bs*2];
			}
		else if(m0<=2)
			{
			if(kn<=0)
				return;

			if(m1>2) D1[1+bs*0] = CC[2+bs*0];
			if(m1>3) D1[2+bs*0] = CC[3+bs*0];

			if(kn<=1)
				return;

			if(m1>3) D1[2+bs*1] = CC[3+bs*1];
			}
		else if(m0<=3)
			{
			if(kn<=0)
				return;

			if(m1>3) D1[2+bs*0] = CC[3+bs*0];
			}
		}

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_lib4(int kmax, float *alpha, float *A, float *B, float *D)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
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

		b_0 = B[0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 4;
		k++;
		}

	// k = 1
	if(kmax>1)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 4;
		k++;
		}

	// k = 2
	if(kmax>2)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 4;
		k++;
		}
	
	kernel_sgemm_nt_4x4_lib4(kmax-k, alpha, A, B, alpha, CC, D);

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_vs_lib4(int kmax, float *alpha, float *A, float *B, float *D, int km, int kn)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
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

		b_0 = B[0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 4;
		k++;
		}

	// k = 1
	if(kmax>1)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 4;
		k++;
		}

	// k = 2
	if(kmax>2)
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 4;
		k++;
		}
	
	kernel_sgemm_nt_4x4_lib4(kmax-k, alpha, A, B, alpha, CC, CC);

	if(km>=4)
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




#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_4x4_lib4(int kmax, float *alpha, float *A, int offsetB, float *B, int sdb, float *D)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float *D1;
	
	int k;

	B += offsetB;

	k = 0;

	if(offsetB==0)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else if(offsetB==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else if(offsetB==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 4

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 5

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else // if(offetB==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 4

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	
	store:
	
	CC[0+bs*0] = alpha[0]*CC[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3];

	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax-k, alpha, A, 0, B, sdb, &beta1, CC, D);

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR)  || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_4x4_vs_lib4(int kmax, float *alpha, float *A, int offsetB, float *B, int sdb, float *D, int m1, int n1)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float *D1;
	
	int k;

	B += offsetB;

	k = 0;

	if(offsetB==0)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else if(offsetB==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else if(offsetB==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 4

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 5

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else // if(offetB==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 4

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	
	store:
	
	CC[0+bs*0] = alpha[0]*CC[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3];

	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax-k, alpha, A, 0, B, sdb, &beta1, CC, CC);

	if(m1>=4)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		D[3+bs*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];
		D[2+bs*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		D[2+bs*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+bs*1] = CC[0+bs*1];
		D[1+bs*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+bs*2] = CC[0+bs*2];
		D[1+bs*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		D[1+bs*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+bs*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+bs*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+bs*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+bs*3] = CC[0+bs*3];
		}

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_4x4_gen_lib4(int kmax, float *alpha, float *A, int offsetB, float *B, int sdb, int offsetD, float *D0, int sdd, int m0, int m1, int n0, int n1)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float *D1;
	
	int k;

	B += offsetB;

	k = 0;

	if(offsetB==0)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else if(offsetB==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else if(offsetB==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 4

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 5

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	else // if(offetB==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 1;
		k += 1;

		if(k>=kmax)
			goto store;

		// k = 4

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_1 = B[4];
		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		b_2 = B[8];
		CC[0+bs*2] += a_0 * b_2;
		CC[1+bs*2] += a_1 * b_2;
		CC[2+bs*2] += a_2 * b_2;
		CC[3+bs*2] += a_3 * b_2;

		b_3 = B[12];
		CC[0+bs*3] += a_0 * b_3;
		CC[1+bs*3] += a_1 * b_3;
		CC[2+bs*3] += a_2 * b_3;
		CC[3+bs*3] += a_3 * b_3;

		A += 4;
		B += 4*sdb-3;
		k += 1;

		}
	
	store:
	
	CC[0+bs*0] = alpha[0]*CC[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3];

	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax-k, alpha, A, 0, B, sdb, &beta1, CC, CC);

	// shift sol for cols
	if(n0>0)
		{
		if(n0==1)
			{
			CC[0+bs*0] = CC[0+bs*1];
			CC[1+bs*0] = CC[1+bs*1];
			CC[2+bs*0] = CC[2+bs*1];
			CC[3+bs*0] = CC[3+bs*1];

			CC[0+bs*1] = CC[0+bs*2];
			CC[1+bs*1] = CC[1+bs*2];
			CC[2+bs*1] = CC[2+bs*2];
			CC[3+bs*1] = CC[3+bs*2];

			CC[0+bs*2] = CC[0+bs*3];
			CC[1+bs*2] = CC[1+bs*3];
			CC[2+bs*2] = CC[2+bs*3];
			CC[3+bs*2] = CC[3+bs*3];

			D0 += 1*bs;
			}
		else if(n0==2)
			{
			CC[0+bs*0] = CC[0+bs*2];
			CC[1+bs*0] = CC[1+bs*2];
			CC[2+bs*0] = CC[2+bs*2];
			CC[3+bs*0] = CC[3+bs*2];

			CC[0+bs*1] = CC[0+bs*3];
			CC[1+bs*1] = CC[1+bs*3];
			CC[2+bs*1] = CC[2+bs*3];
			CC[3+bs*1] = CC[3+bs*3];

			D0 += 2*bs;
			}
		else //if(n0==3)
			{
			CC[0+bs*0] = CC[0+bs*3];
			CC[1+bs*0] = CC[1+bs*3];
			CC[2+bs*0] = CC[2+bs*3];
			CC[3+bs*0] = CC[3+bs*3];

			D0 += 3*bs;
			}
		}

	n1 = 4<n1 ? 4 : n1;
	int kn = n1 - n0;

	if(offsetD==0)
		{
		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[0+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[1+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D0[2+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D0[3+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[0+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[1+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D0[2+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D0[3+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[0+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[1+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D0[2+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D0[3+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[0+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[1+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D0[2+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D0[3+bs*3] = CC[3+bs*3];
		}
	else if(offsetD==1)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[1+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[2+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D0[3+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[0+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[1+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[2+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D0[3+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[0+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[1+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[2+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D0[3+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[0+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[1+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[2+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D0[3+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[0+bs*3] = CC[3+bs*3];
		}
	else if(offsetD==2)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[2+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D0[3+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D1[0+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[1+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[2+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D0[3+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D1[0+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[1+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[2+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D0[3+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D1[0+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[1+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[2+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D0[3+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D1[0+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[1+bs*3] = CC[3+bs*3];
		}
	else //if(offsetD==3)
		{
		D1 = D0 + sdd*bs;

		if(kn<=0)
			return;

		if(m0<=0 & m1>0) D0[3+bs*0] = CC[0+bs*0];
		if(m0<=1 & m1>1) D1[0+bs*0] = CC[1+bs*0];
		if(m0<=2 & m1>2) D1[1+bs*0] = CC[2+bs*0];
		if(m0<=3 & m1>3) D1[2+bs*0] = CC[3+bs*0];

		if(kn<=1)
			return;

		if(m0<=0 & m1>0) D0[3+bs*1] = CC[0+bs*1];
		if(m0<=1 & m1>1) D1[0+bs*1] = CC[1+bs*1];
		if(m0<=2 & m1>2) D1[1+bs*1] = CC[2+bs*1];
		if(m0<=3 & m1>3) D1[2+bs*1] = CC[3+bs*1];

		if(kn<=2)
			return;

		if(m0<=0 & m1>0) D0[3+bs*2] = CC[0+bs*2];
		if(m0<=1 & m1>1) D1[0+bs*2] = CC[1+bs*2];
		if(m0<=2 & m1>2) D1[1+bs*2] = CC[2+bs*2];
		if(m0<=3 & m1>3) D1[2+bs*2] = CC[3+bs*2];

		if(kn<=3)
			return;

		if(m0<=0 & m1>0) D0[3+bs*3] = CC[0+bs*3];
		if(m0<=1 & m1>1) D1[0+bs*3] = CC[1+bs*3];
		if(m0<=2 & m1>2) D1[1+bs*3] = CC[2+bs*3];
		if(m0<=3 & m1>3) D1[2+bs*3] = CC[3+bs*3];
		}
	
	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER)
void kernel_spotrf_nt_l_4x4_lib4(int kmax, float *A, float *B, float *C, float *D, float *inv_diag_D)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, C, CC);

	if(CC[0+bs*0]>0)
		{
		CC[0+bs*0] = sqrtf(CC[0+bs*0]);
		tmp = 1.0/CC[0+bs*0];
		}
	else
		{
		CC[0+bs*0] = 0.0;
		tmp = 0.0;
		}
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;
	inv_diag_D[0] = tmp;

	CC[1+bs*1] -= CC[1+bs*0] * CC[1+bs*0];
	CC[2+bs*1] -= CC[2+bs*0] * CC[1+bs*0];
	CC[3+bs*1] -= CC[3+bs*0] * CC[1+bs*0];
	if(CC[1+bs*1]>0)
		{
		CC[1+bs*1] = sqrtf(CC[1+bs*1]);
		tmp = 1.0/CC[1+bs*1];
		}
	else
		{
		CC[1+bs*1] = 0.0;
		tmp = 0.0;
		}
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	inv_diag_D[1] = tmp;

	CC[2+bs*2] -= CC[2+bs*0] * CC[2+bs*0];
	CC[3+bs*2] -= CC[3+bs*0] * CC[2+bs*0];
	CC[2+bs*2] -= CC[2+bs*1] * CC[2+bs*1];
	CC[3+bs*2] -= CC[3+bs*1] * CC[2+bs*1];
	if(CC[2+bs*2]>0)
		{
		CC[2+bs*2] = sqrtf(CC[2+bs*2]);
		tmp = 1.0/CC[2+bs*2];
		}
	else
		{
		CC[2+bs*2] = 0.0;
		tmp = 0.0;
		}
	CC[3+bs*2] *= tmp;
	inv_diag_D[2] = tmp;

	CC[3+bs*3] -= CC[3+bs*0] * CC[3+bs*0];
	CC[3+bs*3] -= CC[3+bs*1] * CC[3+bs*1];
	CC[3+bs*3] -= CC[3+bs*2] * CC[3+bs*2];
	if(CC[3+bs*3]>0)
		{
		CC[3+bs*3] = sqrtf(CC[3+bs*3]);
		tmp = 1.0/CC[3+bs*3];
		}
	else
		{
		CC[3+bs*3] = 0.0;
		tmp = 0.0;
		}
	inv_diag_D[3] = tmp;

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_spotrf_nt_l_4x4_vs_lib4(int kmax, float *A, float *B, float *C, float *D, float *inv_diag_D, int km, int kn)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, C, CC);

	if(CC[0+bs*0]>0)
		{
		CC[0+bs*0] = sqrtf(CC[0+bs*0]);
		tmp = 1.0/CC[0+bs*0];
		}
	else
		{
		CC[0+bs*0] = 0.0;
		tmp = 0.0;
		}
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;
	inv_diag_D[0] = tmp;

	if(kn==1)
		goto store;
	
	CC[1+bs*1] -= CC[1+bs*0] * CC[1+bs*0];
	CC[2+bs*1] -= CC[2+bs*0] * CC[1+bs*0];
	CC[3+bs*1] -= CC[3+bs*0] * CC[1+bs*0];
	if(CC[1+bs*1]>0)
		{
		CC[1+bs*1] = sqrtf(CC[1+bs*1]);
		tmp = 1.0/CC[1+bs*1];
		}
	else
		{
		CC[1+bs*1] = 0.0;
		tmp = 0.0;
		}
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	inv_diag_D[1] = tmp;

	if(kn==2)
		goto store;
	
	CC[2+bs*2] -= CC[2+bs*0] * CC[2+bs*0];
	CC[3+bs*2] -= CC[3+bs*0] * CC[2+bs*0];
	CC[2+bs*2] -= CC[2+bs*1] * CC[2+bs*1];
	CC[3+bs*2] -= CC[3+bs*1] * CC[2+bs*1];
	if(CC[2+bs*2]>0)
		{
		CC[2+bs*2] = sqrtf(CC[2+bs*2]);
		tmp = 1.0/CC[2+bs*2];
		}
	else
		{
		CC[2+bs*2] = 0.0;
		tmp = 0.0;
		}
	CC[3+bs*2] *= tmp;
	inv_diag_D[2] = tmp;

	if(kn==3)
		goto store;
	
	CC[3+bs*3] -= CC[3+bs*0] * CC[3+bs*0];
	CC[3+bs*3] -= CC[3+bs*1] * CC[3+bs*1];
	CC[3+bs*3] -= CC[3+bs*2] * CC[3+bs*2];
	if(CC[3+bs*3]>0)
		{
		CC[3+bs*3] = sqrtf(CC[3+bs*3]);
		tmp = 1.0/CC[3+bs*3];
		}
	else
		{
		CC[3+bs*3] = 0.0;
		tmp = 0.0;
		}
	inv_diag_D[3] = tmp;


	store:

	if(km>=4)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];
		D[3+bs*0] = CC[3+bs*0];

		if(kn==1)
			return;

		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];
		D[3+bs*1] = CC[3+bs*1];

		if(kn==2)
			return;

		D[2+bs*2] = CC[2+bs*2];
		D[3+bs*2] = CC[3+bs*2];

		if(kn==3)
			return;

		D[3+bs*3] = CC[3+bs*3];
		}
	else if(km>=3)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];
		D[2+bs*0] = CC[2+bs*0];

		if(kn==1)
			return;

		D[1+bs*1] = CC[1+bs*1];
		D[2+bs*1] = CC[2+bs*1];

		if(kn==2)
			return;

		D[2+bs*2] = CC[2+bs*2];
		}
	else if(km>=2)
		{
		D[0+bs*0] = CC[0+bs*0];
		D[1+bs*0] = CC[1+bs*0];

		if(kn==1)
			return;

		D[1+bs*1] = CC[1+bs*1];
		}
	else //if(km>=1)
		{
		D[0+bs*0] = CC[0+bs*0];
		}
	
	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_ssyrk_spotrf_nt_l_4x4_lib4(int kp, float *Ap, float *Bp, int km_, float *Am, float *Bm, float *C, float *D, float *inv_diag_D)
	{
	float alpha = 1.0;
	float beta = 1.0;
	kernel_ssyrk_nt_l_4x4_lib4(kp, &alpha, Ap, Bp, &beta, C, D);
	kernel_spotrf_nt_l_4x4_lib4(km_, Am, Bm, D, D, inv_diag_D);
	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_ssyrk_spotrf_nt_l_4x4_vs_lib4(int kp, float *Ap, float *Bp, int km_, float *Am, float *Bm, float *C, float *D, float *inv_diag_D, int km, int kn)
	{
	float alpha = 1.0;
	float beta = 1.0;
	kernel_ssyrk_nt_l_4x4_vs_lib4(kp, &alpha, Ap, Bp, &beta, C, D, km, kn);
	kernel_spotrf_nt_l_4x4_vs_lib4(km_, Am, Bm, D, D, inv_diag_D, km, kn);
	}
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER)
void kernel_strsm_nt_rl_inv_4x4_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	tmp = E[1+bs*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	tmp = E[2+bs*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+bs*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	tmp = E[3+bs*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+bs*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+bs*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;
	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_inv_4x4_vs_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E, float *inv_diag_E, int km, int kn)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	if(kn==1)
		goto store;
	
	tmp = E[1+bs*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	if(kn==2)
		goto store;
	
	tmp = E[2+bs*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+bs*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	if(kn==3)
		goto store;
	
	tmp = E[3+bs*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+bs*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+bs*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;
	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;

	store:

	if(km>=4)
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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_strsm_nt_rl_inv_4x4_lib4(int kp, float *Ap, float *Bp, int km_, float *Am, float *Bm, float *C, float *D, float *E, float *inv_diag_E)
	{
	float alpha = 1.0;
	float beta  = 1.0;
	kernel_sgemm_nt_4x4_lib4(kp, &alpha, Ap, Bp, &beta, C, D);
	kernel_strsm_nt_rl_inv_4x4_lib4(km_, Am, Bm, &beta, D, D, E, inv_diag_E);
	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_strsm_nt_rl_inv_4x4_vs_lib4(int kp, float *Ap, float *Bp, int km_, float *Am, float *Bm, float *C, float *D, float *E, float *inv_diag_E, int km, int kn)
	{
	float alpha = 1.0;
	float beta  = 1.0;
	kernel_sgemm_nt_4x4_vs_lib4(kp, &alpha, Ap, Bp, &beta, C, D, km, kn);
	kernel_strsm_nt_rl_inv_4x4_vs_lib4(km_, Am, Bm, &beta, D, D, E, inv_diag_E, km, kn);
	}
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_one_4x4_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	tmp = E[1+bs*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	tmp = E[2+bs*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+bs*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	tmp = E[3+bs*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+bs*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+bs*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_one_4x4_vs_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E, int km, int kn)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	if(kn==1)
		goto store;
	
	tmp = E[1+bs*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	if(kn==2)
		goto store;
	
	tmp = E[2+bs*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+bs*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	if(kn==3)
		goto store;
	
	tmp = E[3+bs*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+bs*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+bs*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;

	store:

	if(km>=4)
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



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[2+bs*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;
	tmp = E[1+bs*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[0+bs*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[1+bs*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;
	tmp = E[0+bs*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[0+bs*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_vs_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E, float *inv_diag_E, int km, int kn)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	if(kn>3)
		{
		tmp = inv_diag_E[3];
		CC[0+bs*3] *= tmp;
		CC[1+bs*3] *= tmp;
		CC[2+bs*3] *= tmp;
		CC[3+bs*3] *= tmp;
		tmp = E[2+bs*3];
		CC[0+bs*2] -= CC[0+bs*3] * tmp;
		CC[1+bs*2] -= CC[1+bs*3] * tmp;
		CC[2+bs*2] -= CC[2+bs*3] * tmp;
		CC[3+bs*2] -= CC[3+bs*3] * tmp;
		tmp = E[1+bs*3];
		CC[0+bs*1] -= CC[0+bs*3] * tmp;
		CC[1+bs*1] -= CC[1+bs*3] * tmp;
		CC[2+bs*1] -= CC[2+bs*3] * tmp;
		CC[3+bs*1] -= CC[3+bs*3] * tmp;
		tmp = E[0+bs*3];
		CC[0+bs*0] -= CC[0+bs*3] * tmp;
		CC[1+bs*0] -= CC[1+bs*3] * tmp;
		CC[2+bs*0] -= CC[2+bs*3] * tmp;
		CC[3+bs*0] -= CC[3+bs*3] * tmp;
		}

	if(kn>2)
		{
		tmp = inv_diag_E[2];
		CC[0+bs*2] *= tmp;
		CC[1+bs*2] *= tmp;
		CC[2+bs*2] *= tmp;
		CC[3+bs*2] *= tmp;
		tmp = E[1+bs*2];
		CC[0+bs*1] -= CC[0+bs*2] * tmp;
		CC[1+bs*1] -= CC[1+bs*2] * tmp;
		CC[2+bs*1] -= CC[2+bs*2] * tmp;
		CC[3+bs*1] -= CC[3+bs*2] * tmp;
		tmp = E[0+bs*2];
		CC[0+bs*0] -= CC[0+bs*2] * tmp;
		CC[1+bs*0] -= CC[1+bs*2] * tmp;
		CC[2+bs*0] -= CC[2+bs*2] * tmp;
		CC[3+bs*0] -= CC[3+bs*2] * tmp;
		}

	if(kn>1)
		{
		tmp = inv_diag_E[1];
		CC[0+bs*1] *= tmp;
		CC[1+bs*1] *= tmp;
		CC[2+bs*1] *= tmp;
		CC[3+bs*1] *= tmp;
		tmp = E[0+bs*1];
		CC[0+bs*0] -= CC[0+bs*1] * tmp;
		CC[1+bs*0] -= CC[1+bs*1] * tmp;
		CC[2+bs*0] -= CC[2+bs*1] * tmp;
		CC[3+bs*0] -= CC[3+bs*1] * tmp;
		}

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;


	store:

	if(km>=4)
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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgetrf_nn_4x4_lib4(int kmax, float *A, float *B, int sdb, float *C, float *D, float *inv_diag_D)
	{

	const int bs = 4;

	int k;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, &beta1, C, CC);

	// factorization

	// first column
	tmp = 1.0 / CC[0+bs*0];
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	inv_diag_D[0] = tmp;

	// second column
	CC[1+bs*1] -= CC[1+bs*0] * CC[0+bs*1];
	CC[2+bs*1] -= CC[2+bs*0] * CC[0+bs*1];
	CC[3+bs*1] -= CC[3+bs*0] * CC[0+bs*1];

	tmp = 1.0 / CC[1+bs*1];
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	
	inv_diag_D[1] = tmp;

	// third column
	CC[1+bs*2] -= CC[1+bs*0] * CC[0+bs*2];
	CC[2+bs*2] -= CC[2+bs*0] * CC[0+bs*2];
	CC[3+bs*2] -= CC[3+bs*0] * CC[0+bs*2];

	CC[2+bs*2] -= CC[2+bs*1] * CC[1+bs*2];
	CC[3+bs*2] -= CC[3+bs*1] * CC[1+bs*2];

	tmp = 1.0 / CC[2+bs*2];
	CC[3+bs*2] *= tmp;

	inv_diag_D[2] = tmp;

	// fourth column
	CC[1+bs*3] -= CC[1+bs*0] * CC[0+bs*3];
	CC[2+bs*3] -= CC[2+bs*0] * CC[0+bs*3];
	CC[3+bs*3] -= CC[3+bs*0] * CC[0+bs*3];

	CC[2+bs*3] -= CC[2+bs*1] * CC[1+bs*3];
	CC[3+bs*3] -= CC[3+bs*1] * CC[1+bs*3];

	CC[3+bs*3] -= CC[3+bs*2] * CC[2+bs*3];

	tmp = 1.0 / CC[3+bs*3];

	inv_diag_D[3] = tmp;

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgetrf_nn_4x4_vs_lib4(int kmax, float *A, float *B, int sdb, float *C, float *D, float *inv_diag_D, int km, int kn)
	{

	const int bs = 4;

	int k;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, &beta1, C, CC);

	// factorization

	// first column
	tmp = 1.0 / CC[0+bs*0];
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	inv_diag_D[0] = tmp;

	if(kn==1)
		goto store;

	// second column
	CC[1+bs*1] -= CC[1+bs*0] * CC[0+bs*1];
	CC[2+bs*1] -= CC[2+bs*0] * CC[0+bs*1];
	CC[3+bs*1] -= CC[3+bs*0] * CC[0+bs*1];

	tmp = 1.0 / CC[1+bs*1];
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	
	inv_diag_D[1] = tmp;

	if(kn==2)
		goto store;

	// third column
	CC[1+bs*2] -= CC[1+bs*0] * CC[0+bs*2];
	CC[2+bs*2] -= CC[2+bs*0] * CC[0+bs*2];
	CC[3+bs*2] -= CC[3+bs*0] * CC[0+bs*2];

	CC[2+bs*2] -= CC[2+bs*1] * CC[1+bs*2];
	CC[3+bs*2] -= CC[3+bs*1] * CC[1+bs*2];

	tmp = 1.0 / CC[2+bs*2];
	CC[3+bs*2] *= tmp;

	inv_diag_D[2] = tmp;

	if(kn==3)
		goto store;

	// fourth column
	CC[1+bs*3] -= CC[1+bs*0] * CC[0+bs*3];
	CC[2+bs*3] -= CC[2+bs*0] * CC[0+bs*3];
	CC[3+bs*3] -= CC[3+bs*0] * CC[0+bs*3];

	CC[2+bs*3] -= CC[2+bs*1] * CC[1+bs*3];
	CC[3+bs*3] -= CC[3+bs*1] * CC[1+bs*3];

	CC[3+bs*3] -= CC[3+bs*2] * CC[2+bs*3];

	tmp = 1.0 / CC[3+bs*3];

	inv_diag_D[3] = tmp;

	store:

	if(km>=4)
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



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	tmp = E[2+bs*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;
	tmp = E[1+bs*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[0+bs*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;

	tmp = E[1+bs*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;
	tmp = E[0+bs*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;

	tmp = E[0+bs*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;


	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_vs_lib4(int kmax, float *A, float *B, float *beta, float *C, float *D, float *E, int km, int kn)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, C, CC);

	if(kn>3)
		{
		tmp = E[2+bs*3];
		CC[0+bs*2] -= CC[0+bs*3] * tmp;
		CC[1+bs*2] -= CC[1+bs*3] * tmp;
		CC[2+bs*2] -= CC[2+bs*3] * tmp;
		CC[3+bs*2] -= CC[3+bs*3] * tmp;
		tmp = E[1+bs*3];
		CC[0+bs*1] -= CC[0+bs*3] * tmp;
		CC[1+bs*1] -= CC[1+bs*3] * tmp;
		CC[2+bs*1] -= CC[2+bs*3] * tmp;
		CC[3+bs*1] -= CC[3+bs*3] * tmp;
		tmp = E[0+bs*3];
		CC[0+bs*0] -= CC[0+bs*3] * tmp;
		CC[1+bs*0] -= CC[1+bs*3] * tmp;
		CC[2+bs*0] -= CC[2+bs*3] * tmp;
		CC[3+bs*0] -= CC[3+bs*3] * tmp;
		}

	if(kn>2)
		{
		tmp = E[1+bs*2];
		CC[0+bs*1] -= CC[0+bs*2] * tmp;
		CC[1+bs*1] -= CC[1+bs*2] * tmp;
		CC[2+bs*1] -= CC[2+bs*2] * tmp;
		CC[3+bs*1] -= CC[3+bs*2] * tmp;
		tmp = E[0+bs*2];
		CC[0+bs*0] -= CC[0+bs*2] * tmp;
		CC[1+bs*0] -= CC[1+bs*2] * tmp;
		CC[2+bs*0] -= CC[2+bs*2] * tmp;
		CC[3+bs*0] -= CC[3+bs*2] * tmp;
		}

	if(kn>1)
		{
		tmp = E[0+bs*1];
		CC[0+bs*0] -= CC[0+bs*1] * tmp;
		CC[1+bs*0] -= CC[1+bs*1] * tmp;
		CC[2+bs*0] -= CC[2+bs*1] * tmp;
		CC[3+bs*0] -= CC[3+bs*1] * tmp;
		}


	store:

	if(km>=4)
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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ll_one_4x4_lib4(int kmax, float *A, float *B, int sdb, float *C, float *D, float *E)
	{

	const int bs = 4;

	int k;

	float
		tmp,
		e_1, e_2, e_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, &beta1, C, CC);

	// solution

	e_1 = E[1+bs*0];
	e_2 = E[2+bs*0];
	e_3 = E[3+bs*0];
	CC[1+bs*0] -= e_1 * CC[0+bs*0];
	CC[2+bs*0] -= e_2 * CC[0+bs*0];
	CC[3+bs*0] -= e_3 * CC[0+bs*0];
	CC[1+bs*1] -= e_1 * CC[0+bs*1];
	CC[2+bs*1] -= e_2 * CC[0+bs*1];
	CC[3+bs*1] -= e_3 * CC[0+bs*1];
	CC[1+bs*2] -= e_1 * CC[0+bs*2];
	CC[2+bs*2] -= e_2 * CC[0+bs*2];
	CC[3+bs*2] -= e_3 * CC[0+bs*2];
	CC[1+bs*3] -= e_1 * CC[0+bs*3];
	CC[2+bs*3] -= e_2 * CC[0+bs*3];
	CC[3+bs*3] -= e_3 * CC[0+bs*3];

	e_2 = E[2+bs*1];
	e_3 = E[3+bs*1];
	CC[2+bs*0] -= e_2 * CC[1+bs*0];
	CC[3+bs*0] -= e_3 * CC[1+bs*0];
	CC[2+bs*1] -= e_2 * CC[1+bs*1];
	CC[3+bs*1] -= e_3 * CC[1+bs*1];
	CC[2+bs*2] -= e_2 * CC[1+bs*2];
	CC[3+bs*2] -= e_3 * CC[1+bs*2];
	CC[2+bs*3] -= e_2 * CC[1+bs*3];
	CC[3+bs*3] -= e_3 * CC[1+bs*3];

	e_3 = E[3+bs*2];
	CC[3+bs*0] -= e_3 * CC[2+bs*0];
	CC[3+bs*1] -= e_3 * CC[2+bs*1];
	CC[3+bs*2] -= e_3 * CC[2+bs*2];
	CC[3+bs*3] -= e_3 * CC[2+bs*3];

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ll_one_4x4_vs_lib4(int kmax, float *A, float *B, int sdb, float *C, float *D, float *E, int km, int kn)
	{

	const int bs = 4;

	int k;

	float
		tmp,
		e_1, e_2, e_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, &beta1, C, CC);

	// solution

	if(km==1)
		goto store;
	
	e_1 = E[1+bs*0];
	e_2 = E[2+bs*0];
	e_3 = E[3+bs*0];
	CC[1+bs*0] -= e_1 * CC[0+bs*0];
	CC[2+bs*0] -= e_2 * CC[0+bs*0];
	CC[3+bs*0] -= e_3 * CC[0+bs*0];
	CC[1+bs*1] -= e_1 * CC[0+bs*1];
	CC[2+bs*1] -= e_2 * CC[0+bs*1];
	CC[3+bs*1] -= e_3 * CC[0+bs*1];
	CC[1+bs*2] -= e_1 * CC[0+bs*2];
	CC[2+bs*2] -= e_2 * CC[0+bs*2];
	CC[3+bs*2] -= e_3 * CC[0+bs*2];
	CC[1+bs*3] -= e_1 * CC[0+bs*3];
	CC[2+bs*3] -= e_2 * CC[0+bs*3];
	CC[3+bs*3] -= e_3 * CC[0+bs*3];

	if(km==2)
		goto store;
	
	e_2 = E[2+bs*1];
	e_3 = E[3+bs*1];
	CC[2+bs*0] -= e_2 * CC[1+bs*0];
	CC[3+bs*0] -= e_3 * CC[1+bs*0];
	CC[2+bs*1] -= e_2 * CC[1+bs*1];
	CC[3+bs*1] -= e_3 * CC[1+bs*1];
	CC[2+bs*2] -= e_2 * CC[1+bs*2];
	CC[3+bs*2] -= e_3 * CC[1+bs*2];
	CC[2+bs*3] -= e_2 * CC[1+bs*3];
	CC[3+bs*3] -= e_3 * CC[1+bs*3];

	if(km==3)
		goto store;
	
	e_3 = E[3+bs*2];
	CC[3+bs*0] -= e_3 * CC[2+bs*0];
	CC[3+bs*1] -= e_3 * CC[2+bs*1];
	CC[3+bs*2] -= e_3 * CC[2+bs*2];
	CC[3+bs*3] -= e_3 * CC[2+bs*3];

	store:

	if(km>=4)
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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_inv_4x4_lib4(int kmax, float *A, float *B, int sdb, float *beta, float *C, float *D, float *E, float *inv_diag_E)
	{

	const int bs = 4;

	int k;

	float
		tmp,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, beta, C, CC);

	// solve

	e_00 = inv_diag_E[0];
	CC[0+bs*0] *= e_00;
	CC[1+bs*0] *= e_00;
	CC[2+bs*0] *= e_00;
	CC[3+bs*0] *= e_00;

	e_01 = E[0+bs*1];
	e_11 = inv_diag_E[1];
	CC[0+bs*1] -= CC[0+bs*0] * e_01;
	CC[1+bs*1] -= CC[1+bs*0] * e_01;
	CC[2+bs*1] -= CC[2+bs*0] * e_01;
	CC[3+bs*1] -= CC[3+bs*0] * e_01;
	CC[0+bs*1] *= e_11;
	CC[1+bs*1] *= e_11;
	CC[2+bs*1] *= e_11;
	CC[3+bs*1] *= e_11;

	e_02 = E[0+bs*2];
	e_12 = E[1+bs*2];
	e_22 = inv_diag_E[2];
	CC[0+bs*2] -= CC[0+bs*0] * e_02;
	CC[1+bs*2] -= CC[1+bs*0] * e_02;
	CC[2+bs*2] -= CC[2+bs*0] * e_02;
	CC[3+bs*2] -= CC[3+bs*0] * e_02;
	CC[0+bs*2] -= CC[0+bs*1] * e_12;
	CC[1+bs*2] -= CC[1+bs*1] * e_12;
	CC[2+bs*2] -= CC[2+bs*1] * e_12;
	CC[3+bs*2] -= CC[3+bs*1] * e_12;
	CC[0+bs*2] *= e_22;
	CC[1+bs*2] *= e_22;
	CC[2+bs*2] *= e_22;
	CC[3+bs*2] *= e_22;

	e_03 = E[0+bs*3];
	e_13 = E[1+bs*3];
	e_23 = E[2+bs*3];
	e_33 = inv_diag_E[3];
	CC[0+bs*3] -= CC[0+bs*0] * e_03;
	CC[1+bs*3] -= CC[1+bs*0] * e_03;
	CC[2+bs*3] -= CC[2+bs*0] * e_03;
	CC[3+bs*3] -= CC[3+bs*0] * e_03;
	CC[0+bs*3] -= CC[0+bs*1] * e_13;
	CC[1+bs*3] -= CC[1+bs*1] * e_13;
	CC[2+bs*3] -= CC[2+bs*1] * e_13;
	CC[3+bs*3] -= CC[3+bs*1] * e_13;
	CC[0+bs*3] -= CC[0+bs*2] * e_23;
	CC[1+bs*3] -= CC[1+bs*2] * e_23;
	CC[2+bs*3] -= CC[2+bs*2] * e_23;
	CC[3+bs*3] -= CC[3+bs*2] * e_23;
	CC[0+bs*3] *= e_33;
	CC[1+bs*3] *= e_33;
	CC[2+bs*3] *= e_33;
	CC[3+bs*3] *= e_33;

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_inv_4x4_vs_lib4(int kmax, float *A, float *B, int sdb, float *beta, float *C, float *D, float *E, float *inv_diag_E, int km, int kn)
	{

	const int bs = 4;

	int k;

	float
		tmp,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, beta, C, CC);

	// solve

	e_00 = inv_diag_E[0];
	CC[0+bs*0] *= e_00;
	CC[1+bs*0] *= e_00;
	CC[2+bs*0] *= e_00;
	CC[3+bs*0] *= e_00;

	if(kn==1)
		goto store;
	
	e_01 = E[0+bs*1];
	e_11 = inv_diag_E[1];
	CC[0+bs*1] -= CC[0+bs*0] * e_01;
	CC[1+bs*1] -= CC[1+bs*0] * e_01;
	CC[2+bs*1] -= CC[2+bs*0] * e_01;
	CC[3+bs*1] -= CC[3+bs*0] * e_01;
	CC[0+bs*1] *= e_11;
	CC[1+bs*1] *= e_11;
	CC[2+bs*1] *= e_11;
	CC[3+bs*1] *= e_11;

	if(kn==2)
		goto store;
	
	e_02 = E[0+bs*2];
	e_12 = E[1+bs*2];
	e_22 = inv_diag_E[2];
	CC[0+bs*2] -= CC[0+bs*0] * e_02;
	CC[1+bs*2] -= CC[1+bs*0] * e_02;
	CC[2+bs*2] -= CC[2+bs*0] * e_02;
	CC[3+bs*2] -= CC[3+bs*0] * e_02;
	CC[0+bs*2] -= CC[0+bs*1] * e_12;
	CC[1+bs*2] -= CC[1+bs*1] * e_12;
	CC[2+bs*2] -= CC[2+bs*1] * e_12;
	CC[3+bs*2] -= CC[3+bs*1] * e_12;
	CC[0+bs*2] *= e_22;
	CC[1+bs*2] *= e_22;
	CC[2+bs*2] *= e_22;
	CC[3+bs*2] *= e_22;

	if(kn==3)
		goto store;
	
	e_03 = E[0+bs*3];
	e_13 = E[1+bs*3];
	e_23 = E[2+bs*3];
	e_33 = inv_diag_E[3];
	CC[0+bs*3] -= CC[0+bs*0] * e_03;
	CC[1+bs*3] -= CC[1+bs*0] * e_03;
	CC[2+bs*3] -= CC[2+bs*0] * e_03;
	CC[3+bs*3] -= CC[3+bs*0] * e_03;
	CC[0+bs*3] -= CC[0+bs*1] * e_13;
	CC[1+bs*3] -= CC[1+bs*1] * e_13;
	CC[2+bs*3] -= CC[2+bs*1] * e_13;
	CC[3+bs*3] -= CC[3+bs*1] * e_13;
	CC[0+bs*3] -= CC[0+bs*2] * e_23;
	CC[1+bs*3] -= CC[1+bs*2] * e_23;
	CC[2+bs*3] -= CC[2+bs*2] * e_23;
	CC[3+bs*3] -= CC[3+bs*2] * e_23;
	CC[0+bs*3] *= e_33;
	CC[1+bs*3] *= e_33;
	CC[2+bs*3] *= e_33;
	CC[3+bs*3] *= e_33;

	store:

	if(km>=4)
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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_lu_inv_4x4_lib4(int kmax, float *A, float *B, int sdb, float *C, float *D, float *E, float *inv_diag_E)
	{

	const int bs = 4;

	int k;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, &beta1, C, CC);

	// solve

	e_03 = E[0+bs*3];
	e_13 = E[1+bs*3];
	e_23 = E[2+bs*3];
	e_33 = inv_diag_E[3];
	CC[3+bs*0] *= e_33;
	CC[3+bs*1] *= e_33;
	CC[3+bs*2] *= e_33;
	CC[3+bs*3] *= e_33;
	CC[0+bs*0] -= e_03 * CC[3+bs*0];
	CC[0+bs*1] -= e_03 * CC[3+bs*1];
	CC[0+bs*2] -= e_03 * CC[3+bs*2];
	CC[0+bs*3] -= e_03 * CC[3+bs*3];
	CC[1+bs*0] -= e_13 * CC[3+bs*0];
	CC[1+bs*1] -= e_13 * CC[3+bs*1];
	CC[1+bs*2] -= e_13 * CC[3+bs*2];
	CC[1+bs*3] -= e_13 * CC[3+bs*3];
	CC[2+bs*0] -= e_23 * CC[3+bs*0];
	CC[2+bs*1] -= e_23 * CC[3+bs*1];
	CC[2+bs*2] -= e_23 * CC[3+bs*2];
	CC[2+bs*3] -= e_23 * CC[3+bs*3];

	e_02 = E[0+bs*2];
	e_12 = E[1+bs*2];
	e_22 = inv_diag_E[2];
	CC[2+bs*0] *= e_22;
	CC[2+bs*1] *= e_22;
	CC[2+bs*2] *= e_22;
	CC[2+bs*3] *= e_22;
	CC[0+bs*0] -= e_02 * CC[2+bs*0];
	CC[0+bs*1] -= e_02 * CC[2+bs*1];
	CC[0+bs*2] -= e_02 * CC[2+bs*2];
	CC[0+bs*3] -= e_02 * CC[2+bs*3];
	CC[1+bs*0] -= e_12 * CC[2+bs*0];
	CC[1+bs*1] -= e_12 * CC[2+bs*1];
	CC[1+bs*2] -= e_12 * CC[2+bs*2];
	CC[1+bs*3] -= e_12 * CC[2+bs*3];

	e_01 = E[0+bs*1];
	e_11 = inv_diag_E[1];
	CC[1+bs*0] *= e_11;
	CC[1+bs*1] *= e_11;
	CC[1+bs*2] *= e_11;
	CC[1+bs*3] *= e_11;
	CC[0+bs*0] -= e_01 * CC[1+bs*0];
	CC[0+bs*1] -= e_01 * CC[1+bs*1];
	CC[0+bs*2] -= e_01 * CC[1+bs*2];
	CC[0+bs*3] -= e_01 * CC[1+bs*3];

	e_00 = inv_diag_E[0];
	CC[0+bs*0] *= e_00;
	CC[0+bs*1] *= e_00;
	CC[0+bs*2] *= e_00;
	CC[0+bs*3] *= e_00;

	D[0+bs*0] = CC[0+bs*0];
	D[1+bs*0] = CC[1+bs*0];
	D[2+bs*0] = CC[2+bs*0];
	D[3+bs*0] = CC[3+bs*0];

	D[0+bs*1] = CC[0+bs*1];
	D[1+bs*1] = CC[1+bs*1];
	D[2+bs*1] = CC[2+bs*1];
	D[3+bs*1] = CC[3+bs*1];

	D[0+bs*2] = CC[0+bs*2];
	D[1+bs*2] = CC[1+bs*2];
	D[2+bs*2] = CC[2+bs*2];
	D[3+bs*2] = CC[3+bs*2];

	D[0+bs*3] = CC[0+bs*3];
	D[1+bs*3] = CC[1+bs*3];
	D[2+bs*3] = CC[2+bs*3];
	D[3+bs*3] = CC[3+bs*3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_lu_inv_4x4_vs_lib4(int kmax, float *A, float *B, int sdb, float *C, float *D, float *E, float *inv_diag_E, int km, int kn)
	{

	const int bs = 4;

	int k;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif
	
	float alpha1 = -1.0;
	float beta1  = 1.0;

	kernel_sgemm_nn_4x4_lib4(kmax, &alpha1, A, 0, B, sdb, &beta1, C, CC);

	// solve

	if(km>3)
		{
		e_03 = E[0+bs*3];
		e_13 = E[1+bs*3];
		e_23 = E[2+bs*3];
		e_33 = inv_diag_E[3];
		CC[3+bs*0] *= e_33;
		CC[3+bs*1] *= e_33;
		CC[3+bs*2] *= e_33;
		CC[3+bs*3] *= e_33;
		CC[0+bs*0] -= e_03 * CC[3+bs*0];
		CC[0+bs*1] -= e_03 * CC[3+bs*1];
		CC[0+bs*2] -= e_03 * CC[3+bs*2];
		CC[0+bs*3] -= e_03 * CC[3+bs*3];
		CC[1+bs*0] -= e_13 * CC[3+bs*0];
		CC[1+bs*1] -= e_13 * CC[3+bs*1];
		CC[1+bs*2] -= e_13 * CC[3+bs*2];
		CC[1+bs*3] -= e_13 * CC[3+bs*3];
		CC[2+bs*0] -= e_23 * CC[3+bs*0];
		CC[2+bs*1] -= e_23 * CC[3+bs*1];
		CC[2+bs*2] -= e_23 * CC[3+bs*2];
		CC[2+bs*3] -= e_23 * CC[3+bs*3];
		}
	
	if(km>2)
		{
		e_02 = E[0+bs*2];
		e_12 = E[1+bs*2];
		e_22 = inv_diag_E[2];
		CC[2+bs*0] *= e_22;
		CC[2+bs*1] *= e_22;
		CC[2+bs*2] *= e_22;
		CC[2+bs*3] *= e_22;
		CC[0+bs*0] -= e_02 * CC[2+bs*0];
		CC[0+bs*1] -= e_02 * CC[2+bs*1];
		CC[0+bs*2] -= e_02 * CC[2+bs*2];
		CC[0+bs*3] -= e_02 * CC[2+bs*3];
		CC[1+bs*0] -= e_12 * CC[2+bs*0];
		CC[1+bs*1] -= e_12 * CC[2+bs*1];
		CC[1+bs*2] -= e_12 * CC[2+bs*2];
		CC[1+bs*3] -= e_12 * CC[2+bs*3];
		}
	
	if(km>1)
		{
		e_01 = E[0+bs*1];
		e_11 = inv_diag_E[1];
		CC[1+bs*0] *= e_11;
		CC[1+bs*1] *= e_11;
		CC[1+bs*2] *= e_11;
		CC[1+bs*3] *= e_11;
		CC[0+bs*0] -= e_01 * CC[1+bs*0];
		CC[0+bs*1] -= e_01 * CC[1+bs*1];
		CC[0+bs*2] -= e_01 * CC[1+bs*2];
		CC[0+bs*3] -= e_01 * CC[1+bs*3];
		}
	
	e_00 = inv_diag_E[0];
	CC[0+bs*0] *= e_00;
	CC[0+bs*1] *= e_00;
	CC[0+bs*2] *= e_00;
	CC[0+bs*3] *= e_00;

	store:

	if(km>=4)
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





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_sgemm_4x4_lib.c"

#endif

