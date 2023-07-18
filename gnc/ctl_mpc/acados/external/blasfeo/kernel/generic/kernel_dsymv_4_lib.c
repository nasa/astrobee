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



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dsymv_l_4_libc(int kmax, double *alpha, double *A, int lda, double *x_n, double *z_n)
	{

	if(kmax<=0) 
		return;
	
	double *x_t = x_n;
	double *z_t = z_n;

	int k;

	double
		a_00, a_01, a_02, a_03,
		x_n_0, x_n_1, x_n_2, x_n_3, y_n_0,
		x_t_0, y_t_0, y_t_1, y_t_2, y_t_3;
	
	x_n_0 = alpha[0]*x_n[0];
	x_n_1 = alpha[0]*x_n[1];
	x_n_2 = alpha[0]*x_n[2];
	x_n_3 = alpha[0]*x_n[3];

	y_t_0 = 0;
	y_t_1 = 0;
	y_t_2 = 0;
	y_t_3 = 0;

	k = 0;
	if(kmax<4)
		{
		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_t_0 += a_00 * x_t_0;

		if(kmax==1)
			goto store_t;

		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[1] = y_n_0;

		if(kmax==2)
			goto store_t;

		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_t_2 += a_02 * x_t_0;

		z_n[2] = y_n_0;

		goto store_t;
		}
	else
		{

		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_t_0 += a_00 * x_t_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_t_2 += a_02 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		a_02 = A[3+lda*2];
		a_03 = A[3+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_t_3 += a_03 * x_t_0;

		z_n[3] = y_n_0;

		k += 4;
		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		a_03 = A[0+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		a_02 = A[1+lda*2];
		a_03 = A[1+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		a_03 = A[2+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		a_02 = A[3+lda*2];
		a_03 = A[3+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		a_03 = A[0+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}
	
	store_t:
	z_t[0] += alpha[0]*y_t_0;
	z_t[1] += alpha[0]*y_t_1;
	z_t[2] += alpha[0]*y_t_2;
	z_t[3] += alpha[0]*y_t_3;

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dsymv_l_3_libc(int kmax, double *alpha, double *A, int lda, double *x_n, double *z_n)
	{

	if(kmax<=0) 
		return;
	
	double *x_t = x_n;
	double *z_t = z_n;

	int k;

	double
		a_00, a_01, a_02,
		x_n_0, x_n_1, x_n_2, y_n_0,
		x_t_0, y_t_0, y_t_1, y_t_2;
	
	x_n_0 = alpha[0]*x_n[0];
	x_n_1 = alpha[0]*x_n[1];
	x_n_2 = alpha[0]*x_n[2];

	y_t_0 = 0;
	y_t_1 = 0;
	y_t_2 = 0;

	k = 0;
	if(kmax<3)
		{
		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_t_0 += a_00 * x_t_0;

		if(kmax==1)
			goto store_t;

		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[1] = y_n_0;

		goto store_t;
		}
	else
		{

		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_t_0 += a_00 * x_t_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_t_2 += a_02 * x_t_0;

		z_n[2] = y_n_0;


		k += 3;
		A += 3;
		z_n += 3;
		x_t += 3;

		}
	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		a_02 = A[1+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		a_02 = A[3+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}
	
	store_t:
	z_t[0] += alpha[0]*y_t_0;
	z_t[1] += alpha[0]*y_t_1;
	z_t[2] += alpha[0]*y_t_2;

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dsymv_l_2_libc(int kmax, double *alpha, double *A, int lda, double *x_n, double *z_n)
	{

	if(kmax<=0) 
		return;
	
	double *x_t = x_n;
	double *z_t = z_n;

	int k;

	double
		a_00, a_01,
		x_n_0, x_n_1, y_n_0,
		x_t_0, y_t_0, y_t_1;
	
	x_n_0 = alpha[0]*x_n[0];
	x_n_1 = alpha[0]*x_n[1];

	y_t_0 = 0;
	y_t_1 = 0;

	k = 0;
	if(kmax<2)
		{
		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_t_0 += a_00 * x_t_0;

		goto store_t;
		}
	else
		{

		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_t_0 += a_00 * x_t_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[1] = y_n_0;


		k += 2;
		A += 2;
		z_n += 2;
		x_t += 2;

		}
	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}
	
	store_t:
	z_t[0] += alpha[0]*y_t_0;
	z_t[1] += alpha[0]*y_t_1;

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dsymv_l_1_libc(int kmax, double *alpha, double *A, int lda, double *x_n, double *z_n)
	{

	if(kmax<=0) 
		return;
	
	double *x_t = x_n;
	double *z_t = z_n;

	int k;

	double
		a_00,
		x_n_0, y_n_0,
		x_t_0, y_t_0;
	
	x_n_0 = alpha[0]*x_n[0];

	y_t_0 = 0;

	k = 0;

	// 0

	x_t_0 = x_t[0];

	a_00 = A[0+lda*0];
	
	y_t_0 += a_00 * x_t_0;


	k += 1;
	A += 1;
	z_n += 1;
	x_t += 1;

	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}
	
	store_t:
	z_t[0] += alpha[0]*y_t_0;

	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
// XXX copy and scale y_n into z_n outside the kernel !!!!!
void kernel_dsymv_l_4_vs_libc(int kmax, double *alpha, double *A, int lda, double *x_n, double *z_n, int km)
	{

	if(km<=0)
		return;

	if(km==1)
		{
		kernel_dsymv_l_1_libc(kmax, alpha, A, lda, x_n, z_n);
		}
	else if(km==2)
		{
		kernel_dsymv_l_2_libc(kmax, alpha, A, lda, x_n, z_n);
		}
	else if(km==3)
		{
		kernel_dsymv_l_3_libc(kmax, alpha, A, lda, x_n, z_n);
		}
	else
		{
		kernel_dsymv_l_4_libc(kmax, alpha, A, lda, x_n, z_n);
		}

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dsymv_u_4_libc(int kmax, double *alpha, double *A, int lda, double *x_t, double *z_n)
	{

	double *x_n = x_t + kmax;
	double *z_t = z_n + kmax;

	int k;

	double
		a_00, a_01, a_02, a_03,
		x_n_0, x_n_1, x_n_2, x_n_3, y_n_0,
		x_t_0, y_t_0, y_t_1, y_t_2, y_t_3;
	
	x_n_0 = alpha[0]*x_n[0];
	x_n_1 = alpha[0]*x_n[1];
	x_n_2 = alpha[0]*x_n[2];
	x_n_3 = alpha[0]*x_n[3];

	y_t_0 = 0;
	y_t_1 = 0;
	y_t_2 = 0;
	y_t_3 = 0;

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		a_03 = A[0+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		a_02 = A[1+lda*2];
		a_03 = A[1+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		a_03 = A[2+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		a_02 = A[3+lda*2];
		a_03 = A[3+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		a_03 = A[0+lda*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}

	// (full 4x4) final triangle

	// 0

	y_n_0 = z_n[0]; 
	x_t_0 = x_t[0];

	a_00 = A[0+lda*0];
	a_01 = A[0+lda*1];
	a_02 = A[0+lda*2];
	a_03 = A[0+lda*3];
	
//	y_n_0 += a_00 * x_n_0;
	y_t_0 += a_00 * x_t_0;
	y_n_0 += a_01 * x_n_1;
	y_t_1 += a_01 * x_t_0;
	y_n_0 += a_02 * x_n_2;
	y_t_2 += a_02 * x_t_0;
	y_n_0 += a_03 * x_n_3;
	y_t_3 += a_03 * x_t_0;

	z_n[0] = y_n_0;


	// 1

	y_n_0 = z_n[1]; 
	x_t_0 = x_t[1];

	a_01 = A[1+lda*1];
	a_02 = A[1+lda*2];
	a_03 = A[1+lda*3];
	
//	y_n_0 += a_01 * x_n_1;
	y_t_1 += a_01 * x_t_0;
	y_n_0 += a_02 * x_n_2;
	y_t_2 += a_02 * x_t_0;
	y_n_0 += a_03 * x_n_3;
	y_t_3 += a_03 * x_t_0;

	z_n[1] = y_n_0;


	// 2

	y_n_0 = z_n[2]; 
	x_t_0 = x_t[2];

	a_02 = A[2+lda*2];
	a_03 = A[2+lda*3];
	
//	y_n_0 += a_02 * x_n_2;
	y_t_2 += a_02 * x_t_0;
	y_n_0 += a_03 * x_n_3;
	y_t_3 += a_03 * x_t_0;

	z_n[2] = y_n_0;


	// 3

	y_n_0 = z_n[3]; 
	x_t_0 = x_t[3];

	a_03 = A[3+lda*3];
	
//	y_n_0 += a_03 * x_n_3;
	y_t_3 += a_03 * x_t_0;

	z_n[3] = y_n_0;


	A += 4;
	z_n += 4;
	x_t += 4;

	
	store_t:
	z_t[0] += alpha[0]*y_t_0;
	z_t[1] += alpha[0]*y_t_1;
	z_t[2] += alpha[0]*y_t_2;
	z_t[3] += alpha[0]*y_t_3;

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dsymv_u_3_libc(int kmax, double *alpha, double *A, int lda, double *x_t, double *z_n)
	{

	double *x_n = x_t + kmax;
	double *z_t = z_n + kmax;

	int k;

	double
		a_00, a_01, a_02,
		x_n_0, x_n_1, x_n_2, y_n_0,
		x_t_0, y_t_0, y_t_1, y_t_2;
	
	x_n_0 = alpha[0]*x_n[0];
	x_n_1 = alpha[0]*x_n[1];
	x_n_2 = alpha[0]*x_n[2];

	y_t_0 = 0;
	y_t_1 = 0;
	y_t_2 = 0;

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		a_02 = A[1+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		a_02 = A[3+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}

	// (full 4x4) final triangle

	// 0

	y_n_0 = z_n[0]; 
	x_t_0 = x_t[0];

	a_00 = A[0+lda*0];
	a_01 = A[0+lda*1];
	a_02 = A[0+lda*2];
	
//	y_n_0 += a_00 * x_n_0;
	y_t_0 += a_00 * x_t_0;
	y_n_0 += a_01 * x_n_1;
	y_t_1 += a_01 * x_t_0;
	y_n_0 += a_02 * x_n_2;
	y_t_2 += a_02 * x_t_0;

	z_n[0] = y_n_0;


	// 1

	y_n_0 = z_n[1]; 
	x_t_0 = x_t[1];

	a_01 = A[1+lda*1];
	a_02 = A[1+lda*2];
	
//	y_n_0 += a_01 * x_n_1;
	y_t_1 += a_01 * x_t_0;
	y_n_0 += a_02 * x_n_2;
	y_t_2 += a_02 * x_t_0;

	z_n[1] = y_n_0;


	// 2

	y_n_0 = z_n[2]; 
	x_t_0 = x_t[2];

	a_02 = A[2+lda*2];
	
//	y_n_0 += a_02 * x_n_2;
	y_t_2 += a_02 * x_t_0;

	z_n[2] = y_n_0;


	A += 3;
	z_n += 3;
	x_t += 3;

	
	store_t:
	z_t[0] += alpha[0]*y_t_0;
	z_t[1] += alpha[0]*y_t_1;
	z_t[2] += alpha[0]*y_t_2;

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dsymv_u_2_libc(int kmax, double *alpha, double *A, int lda, double *x_t, double *z_n)
	{

	double *x_n = x_t + kmax;
	double *z_t = z_n + kmax;

	int k;

	double
		a_00, a_01,
		x_n_0, x_n_1, y_n_0,
		x_t_0, y_t_0, y_t_1;
	
	x_n_0 = alpha[0]*x_n[0];
	x_n_1 = alpha[0]*x_n[1];

	y_t_0 = 0;
	y_t_1 = 0;

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}

	// (full 4x4) final triangle

	// 0

	y_n_0 = z_n[0]; 
	x_t_0 = x_t[0];

	a_00 = A[0+lda*0];
	a_01 = A[0+lda*1];
	
//	y_n_0 += a_00 * x_n_0;
	y_t_0 += a_00 * x_t_0;
	y_n_0 += a_01 * x_n_1;
	y_t_1 += a_01 * x_t_0;

	z_n[0] = y_n_0;


	// 1

	y_n_0 = z_n[1]; 
	x_t_0 = x_t[1];

	a_01 = A[1+lda*1];
	
//	y_n_0 += a_01 * x_n_1;
	y_t_1 += a_01 * x_t_0;

	z_n[1] = y_n_0;


	A += 2;
	z_n += 2;
	x_t += 2;

	
	store_t:
	z_t[0] += alpha[0]*y_t_0;
	z_t[1] += alpha[0]*y_t_1;

	return;

	}
#endif




// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dsymv_u_1_libc(int kmax, double *alpha, double *A, int lda, double *x_t, double *z_n)
	{

	double *x_n = x_t + kmax;
	double *z_t = z_n + kmax;

	int k;

	double
		a_00,
		x_n_0, y_n_0,
		x_t_0, y_t_0;
	
	x_n_0 = alpha[0]*x_n[0];

	y_t_0 = 0;

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[0] = y_n_0;


		// 1

		y_n_0 = z_n[1]; 
		x_t_0 = x_t[1];

		a_00 = A[1+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[1] = y_n_0;


		// 2

		y_n_0 = z_n[2]; 
		x_t_0 = x_t[2];

		a_00 = A[2+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[2] = y_n_0;


		// 3

		y_n_0 = z_n[3]; 
		x_t_0 = x_t[3];

		a_00 = A[3+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[3] = y_n_0;


		A += 4;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+lda*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		}

	// (full 4x4) final triangle

	// 0

	y_n_0 = z_n[0]; 
	x_t_0 = x_t[0];

	a_00 = A[0+lda*0];
	
//	y_n_0 += a_00 * x_n_0;
	y_t_0 += a_00 * x_t_0;

	z_n[0] = y_n_0;


	A += 1;
	z_n += 1;
	x_t += 1;

	
	store_t:
	z_t[0] += alpha[0]*y_t_0;

	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
// XXX copy and scale y_n into z_n outside the kernel !!!!!
void kernel_dsymv_u_4_vs_libc(int kmax, double *alpha, double *A, int lda, double *x_n, double *z_n, int km)
	{

	if(km<=0)
		return;

	if(km==1)
		{
		kernel_dsymv_u_1_libc(kmax, alpha, A, lda, x_n, z_n);
		}
	else if(km==2)
		{
		kernel_dsymv_u_2_libc(kmax, alpha, A, lda, x_n, z_n);
		}
	else if(km==3)
		{
		kernel_dsymv_u_3_libc(kmax, alpha, A, lda, x_n, z_n);
		}
	else
		{
		kernel_dsymv_u_4_libc(kmax, alpha, A, lda, x_n, z_n);
		}

	return;

	}
#endif




