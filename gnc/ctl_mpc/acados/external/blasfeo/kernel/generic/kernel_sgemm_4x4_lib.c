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


//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_nt_4x4_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];
		b_3 = B[3+ldb*0];

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

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[0+ldb*1];
		b_1 = B[1+ldb*1];
		b_2 = B[2+ldb*1];
		b_3 = B[3+ldb*1];

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

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[0+ldb*2];
		b_1 = B[1+ldb*2];
		b_2 = B[2+ldb*2];
		b_3 = B[3+ldb*2];

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

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[0+ldb*3];
		b_1 = B[1+ldb*3];
		b_2 = B[2+ldb*3];
		b_3 = B[3+ldb*3];

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

		A += 4*lda;
		B += 4*ldb;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];
		b_3 = B[3+ldb*0];

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

		A += 1*lda;
		B += 1*ldb;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



static void kernel_sgemm_nt_4x3_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];

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


		// k = 1

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[0+ldb*1];
		b_1 = B[1+ldb*1];
		b_2 = B[2+ldb*1];

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


		// k = 2

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[0+ldb*2];
		b_1 = B[1+ldb*2];
		b_2 = B[2+ldb*2];

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


		// k = 3

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[0+ldb*3];
		b_1 = B[1+ldb*3];
		b_2 = B[2+ldb*3];

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

		A += 4*lda;
		B += 4*ldb;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];

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

		A += 1*lda;
		B += 1*ldb;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	return;

	}



static void kernel_sgemm_nt_4x2_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;


		// k = 1

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[0+ldb*1];
		b_1 = B[1+ldb*1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;


		// k = 2

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[0+ldb*2];
		b_1 = B[1+ldb*2];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;


		// k = 3

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[0+ldb*3];
		b_1 = B[1+ldb*3];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4*lda;
		B += 4*ldb;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 1*lda;
		B += 1*ldb;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	return;

	}



static void kernel_sgemm_nt_4x1_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;


		// k = 1

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[0+ldb*1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;


		// k = 2

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[0+ldb*2];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;


		// k = 3

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[0+ldb*3];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4*lda;
		B += 4*ldb;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 1*lda;
		B += 1*ldb;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	return;

	}



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_nt_4x4_vs_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	if(n1<=1)
		{
		kernel_sgemm_nt_4x1_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}
	else if(n1==2)
		{
		kernel_sgemm_nt_4x2_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}
	else if(n1==3)
		{
		kernel_sgemm_nt_4x3_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}
	else //if(n1==1)
		{
		kernel_sgemm_nt_4x4_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}

	if(m1>=4)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
		D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
		D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
		D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nt_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	kernel_sgemm_nt_4x4_libcccc(kmax, alpha, A, 4, B, ldb, beta, C, ldc, D, ldd);
	
	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nt_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	kernel_sgemm_nt_4x4_vs_libcccc(kmax, alpha, A, 4, B, ldb, beta, C, ldc, D, ldd, m1, n1);

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nt_4x4_libc4cc(int kmax, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd)
	{

	kernel_sgemm_nt_4x4_libcccc(kmax, alpha, A, lda, B, 4, beta, C, ldc, D, ldd);

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nt_4x4_vs_libc4cc(int kmax, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	kernel_sgemm_nt_4x4_vs_libcccc(kmax, alpha, A, lda, B, 4, beta, C, ldc, D, ldd, m1, n1);

	return;

	}
#endif



//#if defined(TARGET_GENERIC) | defined(TARGET_X86_AMD_BARCELONA)
#if ! (defined(TARGET_X64_INTEL_CORE) | defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)  | defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nt_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nt_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	if(m1>=4)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
		D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
		D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
		D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		}

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_nn_4x4_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[0+ldb*1];
		b_2 = B[0+ldb*2];
		b_3 = B[0+ldb*3];

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

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[1+ldb*0];
		b_1 = B[1+ldb*1];
		b_2 = B[1+ldb*2];
		b_3 = B[1+ldb*3];

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

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[2+ldb*0];
		b_1 = B[2+ldb*1];
		b_2 = B[2+ldb*2];
		b_3 = B[2+ldb*3];

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

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[3+ldb*0];
		b_1 = B[3+ldb*1];
		b_2 = B[3+ldb*2];
		b_3 = B[3+ldb*3];

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

		A += 4*lda;
		B += 4;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[0+ldb*1];
		b_2 = B[0+ldb*2];
		b_3 = B[0+ldb*3];

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

		A += 1*lda;
		B += 1;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



static void kernel_sgemm_nn_4x3_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[0+ldb*1];
		b_2 = B[0+ldb*2];

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


		// k = 1

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[1+ldb*0];
		b_1 = B[1+ldb*1];
		b_2 = B[1+ldb*2];

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


		// k = 2

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[2+ldb*0];
		b_1 = B[2+ldb*1];
		b_2 = B[2+ldb*2];

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


		// k = 3

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[3+ldb*0];
		b_1 = B[3+ldb*1];
		b_2 = B[3+ldb*2];

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

		A += 4*lda;
		B += 4;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[0+ldb*1];
		b_2 = B[0+ldb*2];

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

		A += 1*lda;
		B += 1;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	return;

	}



static void kernel_sgemm_nn_4x2_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[0+ldb*1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;


		// k = 1

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[1+ldb*0];
		b_1 = B[1+ldb*1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;


		// k = 2

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[2+ldb*0];
		b_1 = B[2+ldb*1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;


		// k = 3

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[3+ldb*0];
		b_1 = B[3+ldb*1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 4*lda;
		B += 4;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];
		b_1 = B[0+ldb*1];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		CC[0+bs*1] += a_0 * b_1;
		CC[1+bs*1] += a_1 * b_1;
		CC[2+bs*1] += a_2 * b_1;
		CC[3+bs*1] += a_3 * b_1;

		A += 1*lda;
		B += 1;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	return;

	}



static void kernel_sgemm_nn_4x1_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

	float
		a_0, a_1, a_2, a_3,
		b_0;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;


		// k = 1

		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];

		b_0 = B[1+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;


		// k = 2

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];

		b_0 = B[2+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;


		// k = 3

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];

		b_0 = B[3+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 4*lda;
		B += 4;

		}
	
	for(; k<kmax; k++)
		{

		// k = 0

		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];

		b_0 = B[0+ldb*0];

		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += 1*lda;
		B += 1;

		}
	
	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	return;

	}



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_nn_4x4_vs_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	if(n1<=1)
		{
		kernel_sgemm_nn_4x1_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}
	else if(n1==2)
		{
		kernel_sgemm_nn_4x2_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}
	else if(n1==3)
		{
		kernel_sgemm_nn_4x3_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}
	else //if(n1==4)
		{
		kernel_sgemm_nn_4x4_libcccc(kmax, &alpha1, A, lda, B, ldb, &beta1, CC, bs, CC, bs);
		}

	if(m1>=4)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
		D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
		D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
		D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		}

	return;

	}
//#endif


//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nn_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	kernel_sgemm_nn_4x4_libcccc(kmax, alpha, A, 4, B, ldb, beta, C, ldc, D, ldd);

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_nn_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	kernel_sgemm_nn_4x4_vs_libcccc(kmax, alpha, A, 4, B, ldb, beta, C, ldc, D, ldd, m1, n1);

	return;

	}
#endif


//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_tt_4x4_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nn_4x4_libcccc(kmax, &alpha1, B, ldb, A, lda, &beta1, CC, bs, CC, bs);

	float tmp;
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;
	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;
	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_tt_4x4_vs_libcccc(int kmax, float *alpha, float *A, int lda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nn_4x4_libcccc(kmax, &alpha1, B, ldb, A, lda, &beta1, CC, bs, CC, bs);

	float tmp;
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;
	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;
	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	if(m1>=4)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
		D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
		D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
		D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_tt_4x4_libc4cc(int kmax, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, B, A, lda, &beta1, CC, bs, CC, bs);

	float tmp;
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;
	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;
	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_sgemm_tt_4x4_vs_libc4cc(int kmax, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, B, A, lda, &beta1, CC, bs, CC, bs);

	float tmp;
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;
	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;
	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	if(m1>=4)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
		D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
		D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
		D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		}

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_ssyrk_nt_l_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER)  || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_ssyrk_nt_l_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	if(m1>=4)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
		D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

		if(n1==1)
			return;

		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
		D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

		if(n1==2)
			return;

		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
		D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

		if(n1==3)
			return;

		D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];

		if(n1==1)
			return;

		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];

		if(n1==2)
			return;

		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];

		if(n1==1)
			return;

		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_ssyrk_nt_u_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
//	D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
//	D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
//	D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
//	D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
//	D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
//	D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_ssyrk_nt_u_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1 = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	if(m1>=4)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
//		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
//		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];
//		D[3+ldd*0] = beta[0]*C[3+ldc*0] + alpha[0]*CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
//		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];
//		D[3+ldd*1] = beta[0]*C[3+ldc*1] + alpha[0]*CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];
//		D[3+ldd*2] = beta[0]*C[3+ldc*2] + alpha[0]*CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		D[3+ldd*3] = beta[0]*C[3+ldc*3] + alpha[0]*CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
//		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];
//		D[2+ldd*0] = beta[0]*C[2+ldc*0] + alpha[0]*CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];
//		D[2+ldd*1] = beta[0]*C[2+ldc*1] + alpha[0]*CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];
		D[2+ldd*2] = beta[0]*C[2+ldc*2] + alpha[0]*CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		D[2+ldd*3] = beta[0]*C[2+ldc*3] + alpha[0]*CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];
//		D[1+ldd*0] = beta[0]*C[1+ldc*0] + alpha[0]*CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];
		D[1+ldd*1] = beta[0]*C[1+ldc*1] + alpha[0]*CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];
		D[1+ldd*2] = beta[0]*C[1+ldc*2] + alpha[0]*CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		D[1+ldd*3] = beta[0]*C[1+ldc*3] + alpha[0]*CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = beta[0]*C[0+ldc*0] + alpha[0]*CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = beta[0]*C[0+ldc*1] + alpha[0]*CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = beta[0]*C[0+ldc*2] + alpha[0]*CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = beta[0]*C[0+ldc*3] + alpha[0]*CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += 1;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, D, ldd);

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += 1;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, CC, bs);

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += 1;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += 1;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_one_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += 1;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, D, ldd);

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_one_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += 1;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, CC, bs);

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_one_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += 1;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_rl_one_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += 1;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += 1;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_0 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	store:

	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	float alpha1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax;

	if(n1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(n1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(n1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(n1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
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

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_0 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax;

	// XXX m1 and n1 are swapped !!!!!
	if(m1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(m1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(m1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(m1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}

	store:

	// scale
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

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_one_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	b_0 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += 1;
//	k += 1;

	store:

	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_one_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	float alpha1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax;

	if(n1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*3] += a_0;
		CC[1+bs*3] += a_1;
		CC[2+bs*3] += a_2;
		CC[3+bs*3] += a_3;

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(n1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(n1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(n1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_one_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
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

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	b_0 = B[0+1*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	b_0 = B[0+2*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	b_0 = B[0+3*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += 1;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += 1;
//	k += 1;

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nn_ru_one_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax;

	// XXX m1 and n1 are swapped !!!!!
	if(m1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		b_0 = B[0+3*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*3] += a_0;
		CC[1+bs*3] += a_1;
		CC[2+bs*3] += a_2;
		CC[3+bs*3] += a_3;

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(m1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[0+2*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(m1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[0+1*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}
	else if(m1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		A += bs;
		B += 1;
//		k += 1;

		// k = 1

		A += bs;
		B += 1;
//		k += 1;

		// k = 2

		A += bs;
		B += 1;
//		k += 1;

		// k = 3

		A += bs;
		B += 1;
//		k += 1;
		
		}

	store:

	// scale
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

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)
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

	float alpha1 = 1.0;
	float beta1  = 0.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_0 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	store:

	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	D[1+ldd*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	D[2+ldd*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	D[3+ldd*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	D[0+ldd*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	D[2+ldd*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	D[3+ldd*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	D[0+ldd*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	D[1+ldd*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	D[3+ldd*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	D[0+ldd*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	D[1+ldd*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	D[2+ldd*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	float alpha1 = 1.0;
	float beta1  = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	if(n1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(n1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(n1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(n1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_tran_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1  = 0.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_0 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	store:

	// scale & transpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_tran_vs_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1  = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	// XXX m1 and n1 are swapped !!!!!
	if(m1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(m1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(m1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(m1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*bs];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)
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

	float alpha1 = 1.0;
	float beta1  = 0.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	b_0 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += bs;
//	k += 1;

	store:

	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	D[1+ldd*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	D[2+ldd*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	D[3+ldd*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	D[0+ldd*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	D[2+ldd*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	D[3+ldd*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	D[0+ldd*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	D[1+ldd*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	D[3+ldd*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	D[0+ldd*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	D[1+ldd*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	D[2+ldd*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	float alpha1 = 1.0;
	float beta1  = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	if(n1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*3] += a_0;
		CC[1+bs*3] += a_1;
		CC[2+bs*3] += a_2;
		CC[3+bs*3] += a_3;

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(n1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(n1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(n1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_tran_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1  = 0.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	b_0 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	b_0 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	b_0 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += bs;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += bs;
//	k += 1;

	store:

	// scale & transpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_tran_vs_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;
	float beta1  = 0.0;

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, &beta1, CC, CC);

	A += kmax*bs;
	B += kmax*bs;

	// XXX m1 and n1 are swapped !!!!!
	if(m1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		b_0 = B[3+0*bs];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*3] += a_0;
		CC[1+bs*3] += a_1;
		CC[2+bs*3] += a_2;
		CC[3+bs*3] += a_3;

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(m1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*bs];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(m1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*bs];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}
	else if(m1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		A += bs;
		B += bs;
//		k += 1;

		// k = 1

		A += bs;
		B += bs;
//		k += 1;

		// k = 2

		A += bs;
		B += bs;
//		k += 1;

		// k = 3

		A += bs;
		B += bs;
//		k += 1;
		
		}

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_0 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	store:

	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	float alpha1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	if(n1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(n1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(n1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(n1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
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

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_0 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	// XXX m1 and n1 are swapped !!!!!
	if(m1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(m1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(m1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(m1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		b_0 = B[0+0*ldb];
		CC[0+bs*0] += a_0 * b_0;
		CC[1+bs*0] += a_1 * b_0;
		CC[2+bs*0] += a_2 * b_0;
		CC[3+bs*0] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}

	store:

	// scale
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

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	b_0 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += ldb;
//	k += 1;

	store:

	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[1+ldd*0] = alpha[0]*CC[1+bs*0];
	D[2+ldd*0] = alpha[0]*CC[2+bs*0];
	D[3+ldd*0] = alpha[0]*CC[3+bs*0];

	D[0+ldd*1] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[2+ldd*1] = alpha[0]*CC[2+bs*1];
	D[3+ldd*1] = alpha[0]*CC[3+bs*1];

	D[0+ldd*2] = alpha[0]*CC[0+bs*2];
	D[1+ldd*2] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[3+ldd*2] = alpha[0]*CC[3+bs*2];

	D[0+ldd*3] = alpha[0]*CC[0+bs*3];
	D[1+ldd*3] = alpha[0]*CC[1+bs*3];
	D[2+ldd*3] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	float alpha1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	if(n1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*3] += a_0;
		CC[1+bs*3] += a_1;
		CC[2+bs*3] += a_2;
		CC[3+bs*3] += a_3;

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(n1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(n1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(n1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;

	// assume always kmax>=4 !!!

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	// k = 0

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	b_0 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_0;
	CC[1+bs*1] += a_1 * b_0;
	CC[2+bs*1] += a_2 * b_0;
	CC[3+bs*1] += a_3 * b_0;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	b_0 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_0;
	CC[1+bs*2] += a_1 * b_0;
	CC[2+bs*2] += a_2 * b_0;
	CC[3+bs*2] += a_3 * b_0;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	b_0 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_0;
	CC[1+bs*3] += a_1 * b_0;
	CC[2+bs*3] += a_2 * b_0;
	CC[3+bs*3] += a_3 * b_0;

	A += bs;
	B += ldb;
//	k += 1;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += ldb;
//	k += 1;

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_rl_one_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	A += kmax*bs;
	B += kmax*ldb;

	// XXX m1 and n1 are swapped !!!!!
	if(m1>=4)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		b_0 = B[3+0*ldb];
		CC[0+bs*3] += a_0 * b_0;
		CC[1+bs*3] += a_1 * b_0;
		CC[2+bs*3] += a_2 * b_0;
		CC[3+bs*3] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*3] += a_0;
		CC[1+bs*3] += a_1;
		CC[2+bs*3] += a_2;
		CC[3+bs*3] += a_3;

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(m1==3)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		b_0 = B[2+0*ldb];
		CC[0+bs*2] += a_0 * b_0;
		CC[1+bs*2] += a_1 * b_0;
		CC[2+bs*2] += a_2 * b_0;
		CC[3+bs*2] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*2] += a_0;
		CC[1+bs*2] += a_1;
		CC[2+bs*2] += a_2;
		CC[3+bs*2] += a_3;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(m1==2)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		b_0 = B[1+0*ldb];
		CC[0+bs*1] += a_0 * b_0;
		CC[1+bs*1] += a_1 * b_0;
		CC[2+bs*1] += a_2 * b_0;
		CC[3+bs*1] += a_3 * b_0;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*1] += a_0;
		CC[1+bs*1] += a_1;
		CC[2+bs*1] += a_2;
		CC[3+bs*1] += a_3;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}
	else if(m1==1)
		{

		// k = 0

		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];

		CC[0+bs*0] += a_0;
		CC[1+bs*0] += a_1;
		CC[2+bs*0] += a_2;
		CC[3+bs*0] += a_3;

		A += bs;
		B += ldb;
//		k += 1;

		// k = 1

		A += bs;
		B += ldb;
//		k += 1;

		// k = 2

		A += bs;
		B += ldb;
//		k += 1;

		// k = 3

		A += bs;
		B += ldb;
//		k += 1;
		
		}

	store:

	// scale
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

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += bs;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, alpha, A, B, &beta1, CC, CC);

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += bs;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, alpha, A, B, &beta1, CC, CC);

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_tran_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += bs;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, &alpha1, A, B, &beta1, CC, CC);

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_tran_vs_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*bs];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += bs;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, &alpha1, A, B, &beta1, CC, CC);

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += ldb;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, D, ldd);

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += ldb;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, CC, bs);

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += ldb;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	b_3 = B[3+0*ldb];
	CC[0+bs*3] += a_0 * b_3;
	CC[1+bs*3] += a_1 * b_3;
	CC[2+bs*3] += a_2 * b_3;
	CC[3+bs*3] += a_3 * b_3;

	A += bs;
	B += ldb;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += bs;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, alpha, A, B, &beta1, CC, CC);

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += bs;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, alpha, A, B, &beta1, CC, CC);

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_tran_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += bs;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, &alpha1, A, B, &beta1, CC, CC);

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_tran_vs_lib444c(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += bs;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*bs];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*bs];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*bs];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += bs;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4(kmax-k, &alpha1, A, B, &beta1, CC, CC);

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, D, ldd);

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_vs_lib4ccc(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	store:

	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+ldc*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+ldc*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+ldc*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+ldc*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+ldc*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+ldc*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+ldc*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+ldc*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+ldc*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+ldc*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+ldc*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+ldc*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+ldc*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+ldc*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+ldc*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+ldc*3];

	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, alpha, A, B, ldb, &beta1, CC, bs, CC, bs);

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_tran_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd)
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale & tranpose & store
	D[0+ldd*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	D[0+ldd*1] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	D[0+ldd*2] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	D[0+ldd*3] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	D[1+ldd*0] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	D[1+ldd*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	D[1+ldd*2] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	D[1+ldd*3] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	D[2+ldd*0] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	D[2+ldd*1] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	D[2+ldd*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	D[2+ldd*3] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	D[3+ldd*0] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	D[3+ldd*1] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	D[3+ldd*2] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	D[3+ldd*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strmm_nt_ru_one_4x4_tran_vs_lib4c4c(int kmax, float *alpha, float *A, float *B, int ldb, float *beta, float *C, float *D, int ldd, int m1, int n1)
	{

	const int bs = 4;

	float
		tmp,
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

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	CC[0+bs*0] += a_0;
	CC[1+bs*0] += a_1;
	CC[2+bs*0] += a_2;
	CC[3+bs*0] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 1

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	CC[0+bs*1] += a_0;
	CC[1+bs*1] += a_1;
	CC[2+bs*1] += a_2;
	CC[3+bs*1] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 2

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	CC[0+bs*2] += a_0;
	CC[1+bs*2] += a_1;
	CC[2+bs*2] += a_2;
	CC[3+bs*2] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	if(k>=kmax)
		goto store;

	// k = 3

	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];
	a_3 = A[3];

	b_0 = B[0+0*ldb];
	CC[0+bs*0] += a_0 * b_0;
	CC[1+bs*0] += a_1 * b_0;
	CC[2+bs*0] += a_2 * b_0;
	CC[3+bs*0] += a_3 * b_0;

	b_1 = B[1+0*ldb];
	CC[0+bs*1] += a_0 * b_1;
	CC[1+bs*1] += a_1 * b_1;
	CC[2+bs*1] += a_2 * b_1;
	CC[3+bs*1] += a_3 * b_1;

	b_2 = B[2+0*ldb];
	CC[0+bs*2] += a_0 * b_2;
	CC[1+bs*2] += a_1 * b_2;
	CC[2+bs*2] += a_2 * b_2;
	CC[3+bs*2] += a_3 * b_2;

	CC[0+bs*3] += a_0;
	CC[1+bs*3] += a_1;
	CC[2+bs*3] += a_2;
	CC[3+bs*3] += a_3;

	A += bs;
	B += ldb;
	k += 1;

	float alpha1 = 1.0;
	float beta1 = 1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax-k, &alpha1, A, B, ldb, &beta1, CC, bs, CC, bs);

	store:

	// scale
	CC[0+bs*0] = alpha[0]*CC[0+bs*0] + beta[0]*C[0+bs*0];
	CC[1+bs*0] = alpha[0]*CC[1+bs*0] + beta[0]*C[1+bs*0];
	CC[2+bs*0] = alpha[0]*CC[2+bs*0] + beta[0]*C[2+bs*0];
	CC[3+bs*0] = alpha[0]*CC[3+bs*0] + beta[0]*C[3+bs*0];

	CC[0+bs*1] = alpha[0]*CC[0+bs*1] + beta[0]*C[0+bs*1];
	CC[1+bs*1] = alpha[0]*CC[1+bs*1] + beta[0]*C[1+bs*1];
	CC[2+bs*1] = alpha[0]*CC[2+bs*1] + beta[0]*C[2+bs*1];
	CC[3+bs*1] = alpha[0]*CC[3+bs*1] + beta[0]*C[3+bs*1];

	CC[0+bs*2] = alpha[0]*CC[0+bs*2] + beta[0]*C[0+bs*2];
	CC[1+bs*2] = alpha[0]*CC[1+bs*2] + beta[0]*C[1+bs*2];
	CC[2+bs*2] = alpha[0]*CC[2+bs*2] + beta[0]*C[2+bs*2];
	CC[3+bs*2] = alpha[0]*CC[3+bs*2] + beta[0]*C[3+bs*2];

	CC[0+bs*3] = alpha[0]*CC[0+bs*3] + beta[0]*C[0+bs*3];
	CC[1+bs*3] = alpha[0]*CC[1+bs*3] + beta[0]*C[1+bs*3];
	CC[2+bs*3] = alpha[0]*CC[2+bs*3] + beta[0]*C[2+bs*3];
	CC[3+bs*3] = alpha[0]*CC[3+bs*3] + beta[0]*C[3+bs*3];

	// transpose
	tmp = CC[1+bs*0]; CC[1+bs*0] = CC[0+bs*1]; CC[0+bs*1] = tmp;
	tmp = CC[2+bs*0]; CC[2+bs*0] = CC[0+bs*2]; CC[0+bs*2] = tmp;
	tmp = CC[3+bs*0]; CC[3+bs*0] = CC[0+bs*3]; CC[0+bs*3] = tmp;

	tmp = CC[2+bs*1]; CC[2+bs*1] = CC[1+bs*2]; CC[1+bs*2] = tmp;
	tmp = CC[3+bs*1]; CC[3+bs*1] = CC[1+bs*3]; CC[1+bs*3] = tmp;

	tmp = CC[3+bs*2]; CC[3+bs*2] = CC[2+bs*3]; CC[2+bs*3] = tmp;

	// store
	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_spotrf_nt_l_4x4_lib44cc(int kmax, float *A, float *B, float *C, int ldc, float *D, int ldd, float *inv_diag_D)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[3+bs*3] = C[3+ldc*3];

	kernel_spotrf_nt_l_4x4_lib4(kmax, A, B, CC, CC, inv_diag_D);

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_spotrf_nt_l_4x4_vs_lib44cc(int kmax, float *A, float *B, float *C, int ldc, float *D, int ldd, float *inv_diag_D, int m1, int n1)
	{

	const int bs = 4;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	if(m1>=4)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];
		CC[3+bs*0] = C[3+ldc*0];

		if(n1==1)
			goto kernel;

		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];
		CC[3+bs*1] = C[3+ldc*1];

		if(n1==2)
			goto kernel;

		CC[2+bs*2] = C[2+ldc*2];
		CC[3+bs*2] = C[3+ldc*2];

		if(n1==3)
			goto kernel;

		CC[3+bs*3] = C[3+ldc*3];
		}
	else if(m1>=3)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];

		if(n1==1)
			goto kernel;

		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];

		if(n1==2)
			goto kernel;

		CC[2+bs*2] = C[2+ldc*2];
		}
	else if(m1>=2)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];

		if(n1==1)
			goto kernel;

		CC[1+bs*1] = C[1+ldc*1];
		}
	else //if(m1>=1)
		{
		CC[0+bs*0] = C[0+ldc*0];
		}

kernel:
	kernel_spotrf_nt_l_4x4_vs_lib4(kmax, A, B, CC, CC, inv_diag_D, m1, n1);

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			goto end;

		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			goto end;

		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			goto end;

		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			goto end;

		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			goto end;

		D[2+ldd*2] = CC[2+bs*2];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			goto end;

		D[1+ldd*1] = CC[1+bs*1];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];
		}

end:
	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_inv_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[1+lde*0];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_inv_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[1+lde*0];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	store:

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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_inv_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[1+lde*0];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_inv_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[1+lde*0];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_one_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = E[1+lde*0];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_one_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = E[1+lde*0];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	store:

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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_one_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = E[1+lde*0];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_rl_one_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = E[3+lde*0];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = E[2+lde*0];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = E[1+lde*0];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_inv_4x4_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[0+bs*1] = C[0+ldc*1];
	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[0+bs*2] = C[0+ldc*2];
	CC[1+bs*2] = C[1+ldc*2];
	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[0+bs*3] = C[0+ldc*3];
	CC[1+bs*3] = C[1+ldc*3];
	CC[2+bs*3] = C[2+ldc*3];
	CC[3+bs*3] = C[3+ldc*3];

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

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

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_inv_4x4_vs_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	if(m1>=4)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];
		CC[3+bs*0] = C[3+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];
		CC[3+bs*1] = C[3+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];
		CC[2+bs*2] = C[2+ldc*2];
		CC[3+bs*2] = C[3+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		CC[2+bs*3] = C[2+ldc*3];
		CC[3+bs*3] = C[3+ldc*3];
		}
	else if(m1>=3)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];
		CC[2+bs*2] = C[2+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		CC[2+bs*3] = C[2+ldc*3];
		}
	else if(m1>=2)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		}
	else //if(m1>=1)
		{
		CC[0+bs*0] = C[0+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		}

kernel:
	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	if(n1==1)
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

	if(n1==2)
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

	if(n1==3)
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_strsm_nt_rl_inv_4x4_lib44ccc(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[0+bs*1] = C[0+ldc*1];
	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[0+bs*2] = C[0+ldc*2];
	CC[1+bs*2] = C[1+ldc*2];
	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[0+bs*3] = C[0+ldc*3];
	CC[1+bs*3] = C[1+ldc*3];
	CC[2+bs*3] = C[2+ldc*3];
	CC[3+bs*3] = C[3+ldc*3];

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;
	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_strsm_nt_rl_inv_4x4_vs_lib44ccc(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	if(m1>=4)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];
		CC[3+bs*0] = C[3+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];
		CC[3+bs*1] = C[3+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];
		CC[2+bs*2] = C[2+ldc*2];
		CC[3+bs*2] = C[3+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		CC[2+bs*3] = C[2+ldc*3];
		CC[3+bs*3] = C[3+ldc*3];
		}
	else if(m1>=3)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];
		CC[2+bs*2] = C[2+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		CC[2+bs*3] = C[2+ldc*3];
		}
	else if(m1>=2)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		}
	else //if(m1>=1)
		{
		CC[0+bs*0] = C[0+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		}

kernel:
	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	if(n1==1)
		goto store;
	
	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	if(n1==2)
		goto store;
	
	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	if(n1==3)
		goto store;
	
	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_inv_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_inv_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	if(n1==1)
		goto store;
	
	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	if(n1==2)
		goto store;
	
	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	if(n1==3)
		goto store;
	
	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
#if ! (defined(TARGET_ARMV8A_ARM_CORTEX_A57)  | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_strsm_nt_rl_inv_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;
	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_inv_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	if(n1==1)
		goto store;
	
	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	if(n1==2)
		goto store;
	
	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	if(n1==3)
		goto store;
	
	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_strsm_nt_rl_one_4x4_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[0+bs*1] = C[0+ldc*1];
	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[0+bs*2] = C[0+ldc*2];
	CC[1+bs*2] = C[1+ldc*2];
	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[0+bs*3] = C[0+ldc*3];
	CC[1+bs*3] = C[1+ldc*3];
	CC[2+bs*3] = C[2+ldc*3];
	CC[3+bs*3] = C[3+ldc*3];

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

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

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_strsm_nt_rl_one_4x4_vs_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	if(m1>=4)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];
		CC[3+bs*0] = C[3+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];
		CC[3+bs*1] = C[3+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];
		CC[2+bs*2] = C[2+ldc*2];
		CC[3+bs*2] = C[3+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		CC[2+bs*3] = C[2+ldc*3];
		CC[3+bs*3] = C[3+ldc*3];
		}
	else if(m1>=3)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];
		CC[2+bs*0] = C[2+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];
		CC[2+bs*1] = C[2+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];
		CC[2+bs*2] = C[2+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		CC[2+bs*3] = C[2+ldc*3];
		}
	else if(m1>=2)
		{
		CC[0+bs*0] = C[0+ldc*0];
		CC[1+bs*0] = C[1+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];
		CC[1+bs*1] = C[1+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];
		CC[1+bs*2] = C[1+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		CC[1+bs*3] = C[1+ldc*3];
		}
	else //if(m1>=1)
		{
		CC[0+bs*0] = C[0+ldc*0];

		if(n1==1)
			goto kernel;

		CC[0+bs*1] = C[0+ldc*1];

		if(n1==2)
			goto kernel;

		CC[0+bs*2] = C[0+ldc*2];

		if(n1==3)
			goto kernel;

		CC[0+bs*3] = C[0+ldc*3];
		}

kernel:
	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	if(n1==1)
		goto store;
	
	tmp = E[1+bs*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	if(n1==2)
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

	if(n1==3)
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_one_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_one_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	if(n1==1)
		goto store;
	
	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	if(n1==2)
		goto store;
	
	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	if(n1==3)
		goto store;
	
	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;

	store:

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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_one_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;

	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_rl_one_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	if(n1==1)
		goto store;
	
	tmp = E[1+lde*0];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	if(n1==2)
		goto store;
	
	tmp = E[2+lde*0];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[2+lde*1];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	if(n1==3)
		goto store;
	
	tmp = E[3+lde*0];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[3+lde*1];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[3+lde*2];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_inv_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_inv_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	if(n1==1)
		goto store;
	
	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	if(n1==2)
		goto store;
	
	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	if(n1==3)
		goto store;
	
	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_inv_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;
	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_inv_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	if(n1==1)
		goto store;
	
	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;
	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;

	if(n1==2)
		goto store;
	
	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;
	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;

	if(n1==3)
		goto store;
	
	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
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

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_one_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_one_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	if(n1==1)
		goto store;
	
	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	if(n1==2)
		goto store;
	
	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	if(n1==3)
		goto store;
	
	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;

	store:

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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_one_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nn_ru_one_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	if(n1==1)
		goto store;
	
	tmp = E[0+lde*1];
	CC[0+bs*1] -= CC[0+bs*0] * tmp;
	CC[1+bs*1] -= CC[1+bs*0] * tmp;
	CC[2+bs*1] -= CC[2+bs*0] * tmp;
	CC[3+bs*1] -= CC[3+bs*0] * tmp;

	if(n1==2)
		goto store;
	
	tmp = E[0+lde*2];
	CC[0+bs*2] -= CC[0+bs*0] * tmp;
	CC[1+bs*2] -= CC[1+bs*0] * tmp;
	CC[2+bs*2] -= CC[2+bs*0] * tmp;
	CC[3+bs*2] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*2] -= CC[0+bs*1] * tmp;
	CC[1+bs*2] -= CC[1+bs*1] * tmp;
	CC[2+bs*2] -= CC[2+bs*1] * tmp;
	CC[3+bs*2] -= CC[3+bs*1] * tmp;

	if(n1==3)
		goto store;
	
	tmp = E[0+lde*3];
	CC[0+bs*3] -= CC[0+bs*0] * tmp;
	CC[1+bs*3] -= CC[1+bs*0] * tmp;
	CC[2+bs*3] -= CC[2+bs*0] * tmp;
	CC[3+bs*3] -= CC[3+bs*0] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*3] -= CC[0+bs*1] * tmp;
	CC[1+bs*3] -= CC[1+bs*1] * tmp;
	CC[2+bs*3] -= CC[2+bs*1] * tmp;
	CC[3+bs*3] -= CC[3+bs*1] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*3] -= CC[0+bs*2] * tmp;
	CC[1+bs*3] -= CC[1+bs*2] * tmp;
	CC[2+bs*3] -= CC[2+bs*2] * tmp;
	CC[3+bs*3] -= CC[3+bs*2] * tmp;

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[0+bs*1] = C[0+ldc*1];
	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[0+bs*2] = C[0+ldc*2];
	CC[1+bs*2] = C[1+ldc*2];
	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[0+bs*3] = C[0+ldc*3];
	CC[1+bs*3] = C[1+ldc*3];
	CC[2+bs*3] = C[2+ldc*3];
	CC[3+bs*3] = C[3+ldc*3];

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[0+bs*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+bs*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+bs*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[0+bs*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+bs*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

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


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_vs_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[0+bs*1] = C[0+ldc*1];
	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[0+bs*2] = C[0+ldc*2];
	CC[1+bs*2] = C[1+ldc*2];
	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[0+bs*3] = C[0+ldc*3];
	CC[1+bs*3] = C[1+ldc*3];
	CC[2+bs*3] = C[2+ldc*3];
	CC[3+bs*3] = C[3+ldc*3];

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	if(n1<=3)
		goto n3;

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[0+bs*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+bs*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+bs*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[0+bs*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+bs*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

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

n1:

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[0+lde*1];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[0+lde*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	store:

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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[0+lde*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_inv_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = inv_diag_E[3];
	CC[0+bs*3] *= tmp;
	CC[1+bs*3] *= tmp;
	CC[2+bs*3] *= tmp;
	CC[3+bs*3] *= tmp;
	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = inv_diag_E[2];
	CC[0+bs*2] *= tmp;
	CC[1+bs*2] *= tmp;
	CC[2+bs*2] *= tmp;
	CC[3+bs*2] *= tmp;
	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = inv_diag_E[1];
	CC[0+bs*1] *= tmp;
	CC[1+bs*1] *= tmp;
	CC[2+bs*1] *= tmp;
	CC[3+bs*1] *= tmp;
	tmp = E[0+lde*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	tmp = inv_diag_E[0];
	CC[0+bs*0] *= tmp;
	CC[1+bs*0] *= tmp;
	CC[2+bs*0] *= tmp;
	CC[3+bs*0] *= tmp;

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[0+bs*1] = C[0+ldc*1];
	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[0+bs*2] = C[0+ldc*2];
	CC[1+bs*2] = C[1+ldc*2];
	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[0+bs*3] = C[0+ldc*3];
	CC[1+bs*3] = C[1+ldc*3];
	CC[2+bs*3] = C[2+ldc*3];
	CC[3+bs*3] = C[3+ldc*3];

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	tmp = E[0+bs*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+bs*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+bs*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = E[0+bs*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+bs*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = E[0+bs*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_vs_lib44cc4(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	CC[0+bs*0] = C[0+ldc*0];
	CC[1+bs*0] = C[1+ldc*0];
	CC[2+bs*0] = C[2+ldc*0];
	CC[3+bs*0] = C[3+ldc*0];

	CC[0+bs*1] = C[0+ldc*1];
	CC[1+bs*1] = C[1+ldc*1];
	CC[2+bs*1] = C[2+ldc*1];
	CC[3+bs*1] = C[3+ldc*1];

	CC[0+bs*2] = C[0+ldc*2];
	CC[1+bs*2] = C[1+ldc*2];
	CC[2+bs*2] = C[2+ldc*2];
	CC[3+bs*2] = C[3+ldc*2];

	CC[0+bs*3] = C[0+ldc*3];
	CC[1+bs*3] = C[1+ldc*3];
	CC[2+bs*3] = C[2+ldc*3];
	CC[3+bs*3] = C[3+ldc*3];

	kernel_sgemm_nt_4x4_lib4(kmax, &alpha1, A, B, beta, CC, CC);

	if(n1<=3)
		goto n3;

	tmp = E[0+bs*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+bs*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+bs*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = E[0+bs*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+bs*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = E[0+bs*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = E[0+lde*1];
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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_vs_lib4c44c(int kmax, float *A, float *B, int ldb, float *beta, float *C, float *D, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, bs, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = E[0+lde*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	store:

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
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

	tmp = E[0+lde*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_strsm_nt_ru_one_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nt_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	if(n1<=3)
		goto n3;

	tmp = E[0+lde*3];
	CC[0+bs*0] -= CC[0+bs*3] * tmp;
	CC[1+bs*0] -= CC[1+bs*3] * tmp;
	CC[2+bs*0] -= CC[2+bs*3] * tmp;
	CC[3+bs*0] -= CC[3+bs*3] * tmp;
	tmp = E[1+lde*3];
	CC[0+bs*1] -= CC[0+bs*3] * tmp;
	CC[1+bs*1] -= CC[1+bs*3] * tmp;
	CC[2+bs*1] -= CC[2+bs*3] * tmp;
	CC[3+bs*1] -= CC[3+bs*3] * tmp;
	tmp = E[2+lde*3];
	CC[0+bs*2] -= CC[0+bs*3] * tmp;
	CC[1+bs*2] -= CC[1+bs*3] * tmp;
	CC[2+bs*2] -= CC[2+bs*3] * tmp;
	CC[3+bs*2] -= CC[3+bs*3] * tmp;

n3:
	if(n1<=2)
		goto n2;

	tmp = E[0+lde*2];
	CC[0+bs*0] -= CC[0+bs*2] * tmp;
	CC[1+bs*0] -= CC[1+bs*2] * tmp;
	CC[2+bs*0] -= CC[2+bs*2] * tmp;
	CC[3+bs*0] -= CC[3+bs*2] * tmp;
	tmp = E[1+lde*2];
	CC[0+bs*1] -= CC[0+bs*2] * tmp;
	CC[1+bs*1] -= CC[1+bs*2] * tmp;
	CC[2+bs*1] -= CC[2+bs*2] * tmp;
	CC[3+bs*1] -= CC[3+bs*2] * tmp;

n2:
	if(n1<=1)
		goto n1;

	tmp = E[0+lde*1];
	CC[0+bs*0] -= CC[0+bs*1] * tmp;
	CC[1+bs*0] -= CC[1+bs*1] * tmp;
	CC[2+bs*0] -= CC[2+bs*1] * tmp;
	CC[3+bs*0] -= CC[3+bs*1] * tmp;

n1:

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_strsm_nn_ll_one_4x4_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde)
	{

	const int bs = 4;

	float tmp,
		e_1, e_2, e_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	e_1 = E[1+lde*0];
	e_2 = E[2+lde*0];
	e_3 = E[3+lde*0];
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

	e_2 = E[2+lde*1];
	e_3 = E[3+lde*1];
	CC[2+bs*0] -= e_2 * CC[1+bs*0];
	CC[3+bs*0] -= e_3 * CC[1+bs*0];
	CC[2+bs*1] -= e_2 * CC[1+bs*1];
	CC[3+bs*1] -= e_3 * CC[1+bs*1];
	CC[2+bs*2] -= e_2 * CC[1+bs*2];
	CC[3+bs*2] -= e_3 * CC[1+bs*2];
	CC[2+bs*3] -= e_2 * CC[1+bs*3];
	CC[3+bs*3] -= e_3 * CC[1+bs*3];

	e_3 = E[3+lde*2];
	CC[3+bs*0] -= e_3 * CC[2+bs*0];
	CC[3+bs*1] -= e_3 * CC[2+bs*1];
	CC[3+bs*2] -= e_3 * CC[2+bs*2];
	CC[3+bs*3] -= e_3 * CC[2+bs*3];


	D[0+ldd*0] = CC[0+bs*0];
	D[1+ldd*0] = CC[1+bs*0];
	D[2+ldd*0] = CC[2+bs*0];
	D[3+ldd*0] = CC[3+bs*0];

	D[0+ldd*1] = CC[0+bs*1];
	D[1+ldd*1] = CC[1+bs*1];
	D[2+ldd*1] = CC[2+bs*1];
	D[3+ldd*1] = CC[3+bs*1];

	D[0+ldd*2] = CC[0+bs*2];
	D[1+ldd*2] = CC[1+bs*2];
	D[2+ldd*2] = CC[2+bs*2];
	D[3+ldd*2] = CC[3+bs*2];

	D[0+ldd*3] = CC[0+bs*3];
	D[1+ldd*3] = CC[1+bs*3];
	D[2+ldd*3] = CC[2+bs*3];
	D[3+ldd*3] = CC[3+bs*3];

	return;

	}
//#endif



//#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
void kernel_strsm_nn_ll_one_4x4_vs_lib4cccc(int kmax, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, int m1, int n1)
	{

	const int bs = 4;

	float tmp,
		e_1, e_2, e_3;

#if defined(TARGET_GENERIC)
	float CC[16] = {0};
#else
	ALIGNED( float CC[16], 64 ) = {0};
#endif

	float alpha1 = -1.0;

	kernel_sgemm_nn_4x4_lib4ccc(kmax, &alpha1, A, B, ldb, beta, C, ldc, CC, bs);

	e_1 = E[1+lde*0];
	e_2 = E[2+lde*0];
	e_3 = E[3+lde*0];
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

	e_2 = E[2+lde*1];
	e_3 = E[3+lde*1];
	CC[2+bs*0] -= e_2 * CC[1+bs*0];
	CC[3+bs*0] -= e_3 * CC[1+bs*0];
	CC[2+bs*1] -= e_2 * CC[1+bs*1];
	CC[3+bs*1] -= e_3 * CC[1+bs*1];
	CC[2+bs*2] -= e_2 * CC[1+bs*2];
	CC[3+bs*2] -= e_3 * CC[1+bs*2];
	CC[2+bs*3] -= e_2 * CC[1+bs*3];
	CC[3+bs*3] -= e_3 * CC[1+bs*3];

	e_3 = E[3+lde*2];
	CC[3+bs*0] -= e_3 * CC[2+bs*0];
	CC[3+bs*1] -= e_3 * CC[2+bs*1];
	CC[3+bs*2] -= e_3 * CC[2+bs*2];
	CC[3+bs*3] -= e_3 * CC[2+bs*3];

	store:

	if(m1>=4)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];
		D[3+ldd*0] = CC[3+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];
		D[3+ldd*1] = CC[3+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];
		D[3+ldd*2] = CC[3+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		D[3+ldd*3] = CC[3+bs*3];
		}
	else if(m1>=3)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];
		D[2+ldd*0] = CC[2+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];
		D[2+ldd*1] = CC[2+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];
		D[2+ldd*2] = CC[2+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		D[2+ldd*3] = CC[2+bs*3];
		}
	else if(m1>=2)
		{
		D[0+ldd*0] = CC[0+bs*0];
		D[1+ldd*0] = CC[1+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];
		D[1+ldd*1] = CC[1+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];
		D[1+ldd*2] = CC[1+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		D[1+ldd*3] = CC[1+bs*3];
		}
	else //if(m1>=1)
		{
		D[0+ldd*0] = CC[0+bs*0];

		if(n1==1)
			return;

		D[0+ldd*1] = CC[0+bs*1];

		if(n1==2)
			return;

		D[0+ldd*2] = CC[0+bs*2];

		if(n1==3)
			return;

		D[0+ldd*3] = CC[0+bs*3];
		}

	return;

	}
//#endif





