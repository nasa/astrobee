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
void kernel_dgemv_n_4_libc(int kmax, double *alpha, double *A, int lda, double *x, double *z)
	{

	if(kmax<=0) 
		return;
	
	int k;

	double
		a_00, a_01, a_02, a_03,
		x_0, x_1, x_2, x_3, y_0;
	
	x_0 = alpha[0]*x[0];
	x_1 = alpha[0]*x[1];
	x_2 = alpha[0]*x[2];
	x_3 = alpha[0]*x[3];

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		a_03 = A[0+lda*3];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;
		y_0 += a_03 * x_3;

		z[0] = y_0;


		// 1
		y_0 = z[1]; 

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		a_02 = A[1+lda*2];
		a_03 = A[1+lda*3];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;
		y_0 += a_03 * x_3;

		z[1] = y_0;


		// 2
		y_0 = z[2]; 

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		a_03 = A[2+lda*3];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;
		y_0 += a_03 * x_3;

		z[2] = y_0;


		// 3
		y_0 = z[3]; 

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		a_02 = A[3+lda*2];
		a_03 = A[3+lda*3];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;
		y_0 += a_03 * x_3;

		z[3] = y_0;


		A += 4;
		z += 4;

		}
	for(; k<kmax; k++)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		a_03 = A[0+lda*3];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;
		y_0 += a_03 * x_3;

		z[0] = y_0;

		A += 1;
		z += 1;

		}
	
	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dgemv_n_3_libc(int kmax, double *alpha, double *A, int lda, double *x, double *z)
	{

	if(kmax<=0) 
		return;
	
	int k;

	double
		a_00, a_01, a_02,
		x_0, x_1, x_2, y_0;
	
	x_0 = alpha[0]*x[0];
	x_1 = alpha[0]*x[1];
	x_2 = alpha[0]*x[2];

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;

		z[0] = y_0;


		// 1
		y_0 = z[1]; 

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		a_02 = A[1+lda*2];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;

		z[1] = y_0;


		// 2
		y_0 = z[2]; 

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		a_02 = A[2+lda*2];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;

		z[2] = y_0;


		// 3
		y_0 = z[3]; 

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		a_02 = A[3+lda*2];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;

		z[3] = y_0;


		A += 4;
		z += 4;

		}
	for(; k<kmax; k++)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		a_02 = A[0+lda*2];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;
		y_0 += a_02 * x_2;

		z[0] = y_0;

		A += 1;
		z += 1;

		}
	
	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dgemv_n_2_libc(int kmax, double *alpha, double *A, int lda, double *x, double *z)
	{

	if(kmax<=0) 
		return;
	
	int k;

	double
		a_00, a_01,
		x_0, x_1, y_0;
	
	x_0 = alpha[0]*x[0];
	x_1 = alpha[0]*x[1];

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;

		z[0] = y_0;


		// 1
		y_0 = z[1]; 

		a_00 = A[1+lda*0];
		a_01 = A[1+lda*1];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;

		z[1] = y_0;


		// 2
		y_0 = z[2]; 

		a_00 = A[2+lda*0];
		a_01 = A[2+lda*1];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;

		z[2] = y_0;


		// 3
		y_0 = z[3]; 

		a_00 = A[3+lda*0];
		a_01 = A[3+lda*1];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;

		z[3] = y_0;


		A += 4;
		z += 4;

		}
	for(; k<kmax; k++)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		a_01 = A[0+lda*1];
		
		y_0 += a_00 * x_0;
		y_0 += a_01 * x_1;

		z[0] = y_0;

		A += 1;
		z += 1;

		}
	
	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dgemv_n_1_libc(int kmax, double *alpha, double *A, int lda, double *x, double *z)
	{

	if(kmax<=0) 
		return;
	
	int k;

	double
		a_00,
		x_0, y_0;
	
	x_0 = alpha[0]*x[0];

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		
		y_0 += a_00 * x_0;

		z[0] = y_0;


		// 1
		y_0 = z[1]; 

		a_00 = A[1+lda*0];
		
		y_0 += a_00 * x_0;

		z[1] = y_0;


		// 2
		y_0 = z[2]; 

		a_00 = A[2+lda*0];
		
		y_0 += a_00 * x_0;

		z[2] = y_0;


		// 3
		y_0 = z[3]; 

		a_00 = A[3+lda*0];
		
		y_0 += a_00 * x_0;

		z[3] = y_0;


		A += 4;
		z += 4;

		}
	for(; k<kmax; k++)
		{

		// 0
		y_0 = z[0]; 

		a_00 = A[0+lda*0];
		
		y_0 += a_00 * x_0;

		z[0] = y_0;

		A += 1;
		z += 1;

		}
	
	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
// XXX copy and scale y_n into z_n outside the kernel !!!!!
void kernel_dgemv_n_4_vs_libc(int kmax, double *alpha, double *A, int lda, double *x, double *z, int km)
	{

	if(km<=0)
		return;

	if(km==1)
		{
		kernel_dgemv_n_1_libc(kmax, alpha, A, lda, x, z);
		}
	else if(km==2)
		{
		kernel_dgemv_n_2_libc(kmax, alpha, A, lda, x, z);
		}
	else if(km==3)
		{
		kernel_dgemv_n_3_libc(kmax, alpha, A, lda, x, z);
		}
	else
		{
		kernel_dgemv_n_4_libc(kmax, alpha, A, lda, x, z);
		}

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dgemv_t_4_libc(int kmax, double *alpha, double *A, int lda, double *x, double *beta, double *y, double *z)
	{

	int k, kend;
	
	double
		x_0, x_1, x_2, x_3;
	
	double yy[4] = {0.0, 0.0, 0.0, 0.0};
	
	k=0;
	for(; k<kmax-3; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		yy[0] += A[0+lda*0] * x_0;
		yy[1] += A[0+lda*1] * x_0;
		yy[2] += A[0+lda*2] * x_0;
		yy[3] += A[0+lda*3] * x_0;

		yy[0] += A[1+lda*0] * x_1;
		yy[1] += A[1+lda*1] * x_1;
		yy[2] += A[1+lda*2] * x_1;
		yy[3] += A[1+lda*3] * x_1;
		
		yy[0] += A[2+lda*0] * x_2;
		yy[1] += A[2+lda*1] * x_2;
		yy[2] += A[2+lda*2] * x_2;
		yy[3] += A[2+lda*3] * x_2;

		yy[0] += A[3+lda*0] * x_3;
		yy[1] += A[3+lda*1] * x_3;
		yy[2] += A[3+lda*2] * x_3;
		yy[3] += A[3+lda*3] * x_3;
		
		A += 4;
		x += 4;

		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
	
		yy[0] += A[0+lda*0] * x_0;
		yy[1] += A[0+lda*1] * x_0;
		yy[2] += A[0+lda*2] * x_0;
		yy[3] += A[0+lda*3] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(beta[0]==0.0)
		{
		z[0] = alpha[0]*yy[0];
		z[1] = alpha[0]*yy[1];
		z[2] = alpha[0]*yy[2];
		z[3] = alpha[0]*yy[3];
		}
	else
		{
		z[0] = alpha[0]*yy[0] + beta[0]*y[0];
		z[1] = alpha[0]*yy[1] + beta[0]*y[1];
		z[2] = alpha[0]*yy[2] + beta[0]*y[2];
		z[3] = alpha[0]*yy[3] + beta[0]*y[3];
		}

	return;

	}
#endif



static void kernel_dgemv_t_3_libc(int kmax, double *alpha, double *A, int lda, double *x, double *beta, double *y, double *z)
	{

	int k, kend;
	
	double
		x_0, x_1, x_2, x_3;
	
	double yy[3] = {0.0, 0.0, 0.0};
	
	k=0;
	for(; k<kmax-3; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		yy[0] += A[0+lda*0] * x_0;
		yy[1] += A[0+lda*1] * x_0;
		yy[2] += A[0+lda*2] * x_0;

		yy[0] += A[1+lda*0] * x_1;
		yy[1] += A[1+lda*1] * x_1;
		yy[2] += A[1+lda*2] * x_1;
		
		yy[0] += A[2+lda*0] * x_2;
		yy[1] += A[2+lda*1] * x_2;
		yy[2] += A[2+lda*2] * x_2;

		yy[0] += A[3+lda*0] * x_3;
		yy[1] += A[3+lda*1] * x_3;
		yy[2] += A[3+lda*2] * x_3;
		
		A += 4;
		x += 4;

		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
	
		yy[0] += A[0+lda*0] * x_0;
		yy[1] += A[0+lda*1] * x_0;
		yy[2] += A[0+lda*2] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(beta[0]==0.0)
		{
		z[0] = alpha[0]*yy[0];
		z[1] = alpha[0]*yy[1];
		z[2] = alpha[0]*yy[2];
		}
	else
		{
		z[0] = alpha[0]*yy[0] + beta[0]*y[0];
		z[1] = alpha[0]*yy[1] + beta[0]*y[1];
		z[2] = alpha[0]*yy[2] + beta[0]*y[2];
		}

	return;

	}



static void kernel_dgemv_t_2_libc(int kmax, double *alpha, double *A, int lda, double *x, double *beta, double *y, double *z)
	{

	int k, kend;
	
	double
		x_0, x_1, x_2, x_3;
	
	double yy[2] = {0.0, 0.0};
	
	k=0;
	for(; k<kmax-3; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		yy[0] += A[0+lda*0] * x_0;
		yy[1] += A[0+lda*1] * x_0;

		yy[0] += A[1+lda*0] * x_1;
		yy[1] += A[1+lda*1] * x_1;
		
		yy[0] += A[2+lda*0] * x_2;
		yy[1] += A[2+lda*1] * x_2;

		yy[0] += A[3+lda*0] * x_3;
		yy[1] += A[3+lda*1] * x_3;
		
		A += 4;
		x += 4;

		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
	
		yy[0] += A[0+lda*0] * x_0;
		yy[1] += A[0+lda*1] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(beta[0]==0.0)
		{
		z[0] = alpha[0]*yy[0];
		z[1] = alpha[0]*yy[1];
		}
	else
		{
		z[0] = alpha[0]*yy[0] + beta[0]*y[0];
		z[1] = alpha[0]*yy[1] + beta[0]*y[1];
		}

	return;

	}



static void kernel_dgemv_t_1_libc(int kmax, double *alpha, double *A, int lda, double *x, double *beta, double *y, double *z)
	{

	int k, kend;
	
	double
		x_0, x_1, x_2, x_3;
	
	double yy[1] = {0.0};
	
	k=0;
	for(; k<kmax-3; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		yy[0] += A[0+lda*0] * x_0;

		yy[0] += A[1+lda*0] * x_1;
		
		yy[0] += A[2+lda*0] * x_2;

		yy[0] += A[3+lda*0] * x_3;
		
		A += 4;
		x += 4;

		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
	
		yy[0] += A[0+lda*0] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(beta[0]==0.0)
		{
		z[0] = alpha[0]*yy[0];
		}
	else
		{
		z[0] = alpha[0]*yy[0] + beta[0]*y[0];
		}

	return;

	}



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
// XXX copy and scale y_n into z_n outside the kernel !!!!!
void kernel_dgemv_t_4_vs_libc(int kmax, double *alpha, double *A, int lda, double *x, double *beta, double *y, double *z, int km)
	{

	if(km<=0)
		return;

	if(km==1)
		{
		kernel_dgemv_t_1_libc(kmax, alpha, A, lda, x, beta, y, z);
		}
	else if(km==2)
		{
		kernel_dgemv_t_2_libc(kmax, alpha, A, lda, x, beta, y, z);
		}
	else if(km==3)
		{
		kernel_dgemv_t_3_libc(kmax, alpha, A, lda, x, beta, y, z);
		}
	else
		{
		kernel_dgemv_t_4_libc(kmax, alpha, A, lda, x, beta, y, z);
		}

	return;

	}
#endif



