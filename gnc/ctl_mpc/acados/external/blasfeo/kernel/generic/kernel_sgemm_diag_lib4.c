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



// B is the diagonal of a matrix, case beta=0.0
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_right_4_a0_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
	
	alpha0 = alpha[0];
		
	b_0 = alpha0 * B[0];
	b_1 = alpha0 * B[1];
	b_2 = alpha0 * B[2];
	b_3 = alpha0 * B[3];
	
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		c_0 = a_0 * b_0;
		c_1 = a_1 * b_0;
		c_2 = a_2 * b_0;
		c_3 = a_3 * b_0;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
		

		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		c_0 = a_0 * b_1;
		c_1 = a_1 * b_1;
		c_2 = a_2 * b_1;
		c_3 = a_3 * b_1;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;
		

		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		c_0 = a_0 * b_2;
		c_1 = a_1 * b_2;
		c_2 = a_2 * b_2;
		c_3 = a_3 * b_2;

		D[0+bs*2] = c_0;
		D[1+bs*2] = c_1;
		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;
		

		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		c_0 = a_0 * b_3;
		c_1 = a_1 * b_3;
		c_2 = a_2 * b_3;
		c_3 = a_3 * b_3;

		D[0+bs*3] = c_0;
		D[1+bs*3] = c_1;
		D[2+bs*3] = c_2;
		D[3+bs*3] = c_3;

		A += 4*sda;
		D += 4*sdd;
		
		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		
		c_0 = a_0 * b_0;

		D[0+bs*0] = c_0;
		

		a_0 = A[0+bs*1];
		
		c_0 = a_0 * b_1;

		D[0+bs*1] = c_0;
		

		a_0 = A[0+bs*2];
		
		c_0 = a_0 * b_2;

		D[0+bs*2] = c_0;
		

		a_0 = A[0+bs*3];
		
		c_0 = a_0 * b_3;

		D[0+bs*3] = c_0;


		A += 1;
		D += 1;
		
		}
	
	}
#endif



// B is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_right_4_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
	
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	b_0 = alpha0 * B[0];
	b_1 = alpha0 * B[1];
	b_2 = alpha0 * B[2];
	b_3 = alpha0 * B[3];
	
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_0;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_0;
		c_3 = beta0 * C[3+bs*0] + a_3 * b_0;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
		

		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_1;
		c_1 = beta0 * C[1+bs*1] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*1] + a_2 * b_1;
		c_3 = beta0 * C[3+bs*1] + a_3 * b_1;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;
		

		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_2;
		c_1 = beta0 * C[1+bs*2] + a_1 * b_2;
		c_2 = beta0 * C[2+bs*2] + a_2 * b_2;
		c_3 = beta0 * C[3+bs*2] + a_3 * b_2;

		D[0+bs*2] = c_0;
		D[1+bs*2] = c_1;
		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;
		

		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		c_0 = beta0 * C[0+bs*3] + a_0 * b_3;
		c_1 = beta0 * C[1+bs*3] + a_1 * b_3;
		c_2 = beta0 * C[2+bs*3] + a_2 * b_3;
		c_3 = beta0 * C[3+bs*3] + a_3 * b_3;

		D[0+bs*3] = c_0;
		D[1+bs*3] = c_1;
		D[2+bs*3] = c_2;
		D[3+bs*3] = c_3;

		A += 4*sda;
		C += 4*sdc;
		D += 4*sdd;
		
		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
		

		a_0 = A[0+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_1;

		D[0+bs*1] = c_0;
		

		a_0 = A[0+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_2;

		D[0+bs*2] = c_0;
		

		a_0 = A[0+bs*3];
		
		c_0 = beta0 * C[0+bs*3] + a_0 * b_3;

		D[0+bs*3] = c_0;


		A += 1;
		C += 1;
		D += 1;
		
		}
	
	}
#endif



// B is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_right_3_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2,
		c_0, c_1, c_2, c_3;
		
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	b_0 = alpha0 * B[0];
	b_1 = alpha0 * B[1];
	b_2 = alpha0 * B[2];
	
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_0;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_0;
		c_3 = beta0 * C[3+bs*0] + a_3 * b_0;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
		

		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_1;
		c_1 = beta0 * C[1+bs*1] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*1] + a_2 * b_1;
		c_3 = beta0 * C[3+bs*1] + a_3 * b_1;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;
		

		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_2;
		c_1 = beta0 * C[1+bs*2] + a_1 * b_2;
		c_2 = beta0 * C[2+bs*2] + a_2 * b_2;
		c_3 = beta0 * C[3+bs*2] + a_3 * b_2;

		D[0+bs*2] = c_0;
		D[1+bs*2] = c_1;
		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;
		

		A += 4*sda;
		C += 4*sdc;
		D += 4*sdd;
		
		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
		

		a_0 = A[0+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_1;

		D[0+bs*1] = c_0;
		

		a_0 = A[0+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_2;

		D[0+bs*2] = c_0;
		

		A += 1;
		C += 1;
		D += 1;
		
		}
	
	}
#endif



// B is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_right_2_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_0, c_1, c_2, c_3;
		
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	b_0 = alpha0 * B[0];
	b_1 = alpha0 * B[1];

	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_0;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_0;
		c_3 = beta0 * C[3+bs*0] + a_3 * b_0;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
		

		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_1;
		c_1 = beta0 * C[1+bs*1] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*1] + a_2 * b_1;
		c_3 = beta0 * C[3+bs*1] + a_3 * b_1;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;
		

		A += 4*sda;
		C += 4*sdc;
		D += 4*sdd;
		
		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
		

		a_0 = A[0+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_1;

		D[0+bs*1] = c_0;
		

		A += 1;
		C += 1;
		D += 1;
		
		}
	
	}
#endif



// B is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_right_1_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0, a_1, a_2, a_3,
		b_0,
		c_0, c_1, c_2, c_3;
		
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	b_0 = alpha0 * B[0];
	
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_0;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_0;
		c_3 = beta0 * C[3+bs*0] + a_3 * b_0;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
		

		A += 4*sda;
		C += 4*sdc;
		D += 4*sdd;
		
		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
		

		A += 1;
		C += 1;
		D += 1;
		
		}
	
	}
#endif



// A is the diagonal of a matrix, case beta=0.0
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_left_4_a0_lib4(int kmax, float *alpha, float *A, float *B, float *D)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	alpha0 = alpha[0];
		
	a_0 = alpha0 * A[0];
	a_1 = alpha0 * A[1];
	a_2 = alpha0 * A[2];
	a_3 = alpha0 * A[3];
	
	for(k=0; k<kmax-3; k+=4)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = a_0 * b_0;
		c_1 = a_1 * b_1;
		c_2 = a_2 * b_2;
		c_3 = a_3 * b_3;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
		

		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_0 = a_0 * b_0;
		c_1 = a_1 * b_1;
		c_2 = a_2 * b_2;
		c_3 = a_3 * b_3;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;
		

		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_0 = a_0 * b_0;
		c_1 = a_1 * b_1;
		c_2 = a_2 * b_2;
		c_3 = a_3 * b_3;

		D[0+bs*2] = c_0;
		D[1+bs*2] = c_1;
		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;
		

		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		b_3 = B[3+bs*3];
		
		c_0 = a_0 * b_0;
		c_1 = a_1 * b_1;
		c_2 = a_2 * b_2;
		c_3 = a_3 * b_3;

		D[0+bs*3] = c_0;
		D[1+bs*3] = c_1;
		D[2+bs*3] = c_2;
		D[3+bs*3] = c_3;

		B += 16;
		D += 16;
		
		}
	for(; k<kmax; k++)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = a_0 * b_0;
		c_1 = a_1 * b_1;
		c_2 = a_2 * b_2;
		c_3 = a_3 * b_3;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
	
		B += 4;
		D += 4;
		
		}
	
	}
#endif



// A is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_left_4_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	a_0 = alpha0 * A[0];
	a_1 = alpha0 * A[1];
	a_2 = alpha0 * A[2];
	a_3 = alpha0 * A[3];
	
	for(k=0; k<kmax-3; k+=4)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_2;
		c_3 = beta0 * C[3+bs*0] + a_3 * b_3;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
		

		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*1] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*1] + a_2 * b_2;
		c_3 = beta0 * C[3+bs*1] + a_3 * b_3;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;
		

		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*2] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*2] + a_2 * b_2;
		c_3 = beta0 * C[3+bs*2] + a_3 * b_3;

		D[0+bs*2] = c_0;
		D[1+bs*2] = c_1;
		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;
		

		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		b_3 = B[3+bs*3];
		
		c_0 = beta0 * C[0+bs*3] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*3] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*3] + a_2 * b_2;
		c_3 = beta0 * C[3+bs*3] + a_3 * b_3;

		D[0+bs*3] = c_0;
		D[1+bs*3] = c_1;
		D[2+bs*3] = c_2;
		D[3+bs*3] = c_3;

		B += 16;
		C += 16;
		D += 16;
		
		}
	for(; k<kmax; k++)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_2;
		c_3 = beta0 * C[3+bs*0] + a_3 * b_3;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;
	
		B += 4;
		C += 4;
		D += 4;
		
		}
	
	}
#endif



// A is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_left_3_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0, a_1, a_2,
		b_0, b_1, b_2,
		c_0, c_1, c_2;
		
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	a_0 = alpha0 * A[0];
	a_1 = alpha0 * A[1];
	a_2 = alpha0 * A[2];

	for(k=0; k<kmax-3; k+=4)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_2;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		

		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*1] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*1] + a_2 * b_2;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		

		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*2] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*2] + a_2 * b_2;

		D[0+bs*2] = c_0;
		D[1+bs*2] = c_1;
		D[2+bs*2] = c_2;
		

		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		
		c_0 = beta0 * C[0+bs*3] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*3] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*3] + a_2 * b_2;

		D[0+bs*3] = c_0;
		D[1+bs*3] = c_1;
		D[2+bs*3] = c_2;

		B += 16;
		C += 16;
		D += 16;
		
		}
	for(; k<kmax; k++)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_1;
		c_2 = beta0 * C[2+bs*0] + a_2 * b_2;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
	
		B += 4;
		C += 4;
		D += 4;
		
		}
	
	}
#endif



// A is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_left_2_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0, a_1,
		b_0, b_1,
		c_0, c_1;
		
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	a_0 = alpha0 * A[0];
	a_1 = alpha0 * A[1];

	for(k=0; k<kmax-3; k+=4)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_1;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		

		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*1] + a_1 * b_1;

		D[0+bs*1] = c_0;
		D[1+bs*1] = c_1;
		

		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*2] + a_1 * b_1;

		D[0+bs*2] = c_0;
		D[1+bs*2] = c_1;
		

		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_0 = beta0 * C[0+bs*3] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*3] + a_1 * b_1;

		D[0+bs*3] = c_0;
		D[1+bs*3] = c_1;

		B += 16;
		C += 16;
		D += 16;
		
		}
	for(; k<kmax; k++)
		{
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;
		c_1 = beta0 * C[1+bs*0] + a_1 * b_1;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
	
		B += 4;
		C += 4;
		D += 4;
		
		}
	
	}
#endif



// A is the diagonal of a matrix
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgemm_diag_left_1_lib4(int kmax, float *alpha, float *A, float *B, float *beta, float *C, float *D)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	float
		alpha0, beta0,
		a_0,
		b_0,
		c_0;
		
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	a_0 = alpha0 * A[0];
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		b_0 = B[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
		

		b_0 = B[0+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_0;

		D[0+bs*1] = c_0;
		

		b_0 = B[0+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_0;

		D[0+bs*2] = c_0;
		

		b_0 = B[0+bs*3];
		
		c_0 = beta0 * C[0+bs*3] + a_0 * b_0;

		D[0+bs*3] = c_0;

		B += 16;
		C += 16;
		D += 16;
		
		}
	for(; k<kmax; k++)
		{
		
		b_0 = B[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
	
		B += 4;
		C += 4;
		D += 4;
		
		}
		
	}
#endif



