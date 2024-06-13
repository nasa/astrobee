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



#if defined(TARGET_GENERIC) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dgemv_n_4_lib4(int kmax, double *alpha, double *A, double *x, double *beta, double *y, double *z)
	{

	const int bs = 4;

	int k;

	double x_0;
	
	double yy[4] = {0.0, 0.0, 0.0, 0.0};
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = x[0];

		yy[0] += A[0+bs*0] * x_0;
		yy[1] += A[1+bs*0] * x_0;
		yy[2] += A[2+bs*0] * x_0;
		yy[3] += A[3+bs*0] * x_0;
		
		x_0 = x[1];

		yy[0] += A[0+bs*1] * x_0;
		yy[1] += A[1+bs*1] * x_0;
		yy[2] += A[2+bs*1] * x_0;
		yy[3] += A[3+bs*1] * x_0;
		
		x_0 = x[2];

		yy[0] += A[0+bs*2] * x_0;
		yy[1] += A[1+bs*2] * x_0;
		yy[2] += A[2+bs*2] * x_0;
		yy[3] += A[3+bs*2] * x_0;
		
		x_0 = x[3];

		yy[0] += A[0+bs*3] * x_0;
		yy[1] += A[1+bs*3] * x_0;
		yy[2] += A[2+bs*3] * x_0;
		yy[3] += A[3+bs*3] * x_0;
		
		A += 4*bs;
		x += 4;

		}

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		yy[0] += A[0+bs*0] * x_0;
		yy[1] += A[1+bs*0] * x_0;
		yy[2] += A[2+bs*0] * x_0;
		yy[3] += A[3+bs*0] * x_0;
		
		A += 1*bs;
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



#if defined(TARGET_GENERIC) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dgemv_n_4_vs_lib4(int kmax, double *alpha, double *A, double *x, double *beta, double *y, double *z, int m1)
	{

	const int bs = 4;

	double yy[4] = {0.0, 0.0, 0.0, 0.0};

	kernel_dgemv_n_4_lib4(kmax, alpha, A, x, beta, y, yy);
	
	z[0] = yy[0];
	if(m1<2) return;
	z[1] = yy[1];
	if(m1<3) return;
	z[2] = yy[2];
	if(m1<4) return;
	z[3] = yy[3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dgemv_n_4_gen_lib4(int kmax, double *alpha, double *A, double *x, double *beta, double *y, double *z, int m0, int m1)
	{

	const int bs = 4;

	double yy[4] = {0.0, 0.0, 0.0, 0.0};

	kernel_dgemv_n_4_lib4(kmax, alpha, A, x, beta, y, yy);
	
	if(m0<=0 & m1>0) z[0] = yy[0];
	if(m0<=1 & m1>1) z[1] = yy[1];
	if(m0<=2 & m1>2) z[2] = yy[2];
	if(m0<=3 & m1>3) z[3] = yy[3];
	
	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dgemv_t_4_lib4(int kmax, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z)
	{

	const int bs  = 4;
	
	int k, kend;
	
	double
		x_0, x_1, x_2, x_3;
	
	double yy[4] = {0.0, 0.0, 0.0, 0.0};
	
	k=0;
	if(offA!=0) // 1, 2, 3
		{
		kend = 4-offA<kmax ? 4-offA : kmax;
		for(; k<kend; k++)
			{
			
			x_0 = x[0];
		
			yy[0] += A[0+bs*0] * x_0;
			yy[1] += A[0+bs*1] * x_0;
			yy[2] += A[0+bs*2] * x_0;
			yy[3] += A[0+bs*3] * x_0;
		
			A += 1;
			x += 1;
			
			}
		A += bs*(sda-1);
		}
	for(; k<kmax-bs+1; k+=bs)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		yy[0] += A[0+bs*0] * x_0;
		yy[1] += A[0+bs*1] * x_0;
		yy[2] += A[0+bs*2] * x_0;
		yy[3] += A[0+bs*3] * x_0;

		yy[0] += A[1+bs*0] * x_1;
		yy[1] += A[1+bs*1] * x_1;
		yy[2] += A[1+bs*2] * x_1;
		yy[3] += A[1+bs*3] * x_1;
		
		yy[0] += A[2+bs*0] * x_2;
		yy[1] += A[2+bs*1] * x_2;
		yy[2] += A[2+bs*2] * x_2;
		yy[3] += A[2+bs*3] * x_2;

		yy[0] += A[3+bs*0] * x_3;
		yy[1] += A[3+bs*1] * x_3;
		yy[2] += A[3+bs*2] * x_3;
		yy[3] += A[3+bs*3] * x_3;
		
		A += sda*bs;
		x += 4;

		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
	
		yy[0] += A[0+bs*0] * x_0;
		yy[1] += A[0+bs*1] * x_0;
		yy[2] += A[0+bs*2] * x_0;
		yy[3] += A[0+bs*3] * x_0;
	
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




#if defined(TARGET_GENERIC) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dgemv_t_4_vs_lib4(int kmax, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z, int m1)
	{

	const int bs = 4;

	double yy[4] = {0.0, 0.0, 0.0, 0.0};

	kernel_dgemv_t_4_lib4(kmax, alpha, offA, A, sda, x, beta, y, yy);
	
	z[0] = yy[0];
	if(m1<2) return;
	z[1] = yy[1];
	if(m1<3) return;
	z[2] = yy[2];
	if(m1<4) return;
	z[3] = yy[3];

	return;


	}
#endif




#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_ln_inv_4_vs_lib4(int kmax, double *A, double *inv_diag_A, double *x, double *y, double *z, int m1, int n1)
	{

	const int bs = 4;
	
	double yy[4] = {0.0, 0.0, 0.0, 0.0};

	double alpha1 = -1.0;
	double beta1  = 1.0;

	int k1 = kmax/bs*bs;

	kernel_dgemv_n_4_lib4(k1, &alpha1, A, x, &beta1, y, yy);

	A += k1*bs;

	double
		a_00, a_10, a_20, a_30,
		a_11, a_21, a_31;
	
	// a_00
	a_00 = inv_diag_A[0];
	a_10 = A[1+bs*0];
	a_20 = A[2+bs*0];
	a_30 = A[3+bs*0];
	yy[0] *= a_00;
	z[0] = yy[0];
	yy[1] -= a_10 * yy[0];
	yy[2] -= a_20 * yy[0];
	yy[3] -= a_30 * yy[0];

	if(n1==1)
		{
		if(m1==1)
			return;
		z[1] = yy[1];
		if(m1==2)
			return;
		z[2] = yy[2];
		if(m1==3)
			return;
		z[3] = yy[3];
		return;
		}

	// a_11
	a_11 = inv_diag_A[1];
	a_21 = A[2+bs*1];
	a_31 = A[3+bs*1];
	yy[1] *= a_11;	
	z[1] = yy[1];
	yy[2] -= a_21 * yy[1];
	yy[3] -= a_31 * yy[1];

	if(n1==2)
		{
		if(m1==2)
			return;
		z[2] = yy[2];
		if(m1==3)
			return;
		z[3] = yy[3];
		return;
		}

	// a_22
	a_00 = inv_diag_A[2];
	a_10 = A[3+bs*2];
	yy[2] *= a_00;
	z[2] = yy[2];
	yy[3] -= a_10 * yy[2];

	if(n1==3)
		{
		if(m1==3)
			return;
		z[3] = yy[3];

		return;
		}

	// a_33
	a_11 = inv_diag_A[3];
	yy[3] *= a_11;	
	z[3] = yy[3];

	return;

	}
#endif
	

	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_ln_inv_4_lib4(int kmax, double *A, double *inv_diag_A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	double yy[4] = {0.0, 0.0, 0.0, 0.0};

	double alpha1 = -1.0;
	double beta1  = 1.0;

	int k1 = kmax/bs*bs;

	kernel_dgemv_n_4_lib4(k1, &alpha1, A, x, &beta1, y, yy);

	A += k1*bs;

	double
		a_00, a_10, a_20, a_30,
		a_11, a_21, a_31;
	
	// a_00
	a_00 = inv_diag_A[0];
	a_10 = A[1+bs*0];
	a_20 = A[2+bs*0];
	a_30 = A[3+bs*0];
	yy[0] *= a_00;
	z[0] = yy[0];
	yy[1] -= a_10 * yy[0];
	yy[2] -= a_20 * yy[0];
	yy[3] -= a_30 * yy[0];

	// a_11
	a_11 = inv_diag_A[1];
	a_21 = A[2+bs*1];
	a_31 = A[3+bs*1];
	yy[1] *= a_11;	
	z[1] = yy[1];
	yy[2] -= a_21 * yy[1];
	yy[3] -= a_31 * yy[1];

	// a_22
	a_00 = inv_diag_A[2];
	a_10 = A[3+bs*2];
	yy[2] *= a_00;
	z[2] = yy[2];
	yy[3] -= a_10 * yy[2];

	// a_33
	a_11 = inv_diag_A[3];
	yy[3] *= a_11;	
	z[3] = yy[3];

	return;

	}
#endif
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_ln_one_4_vs_lib4(int kmax, double *A, double *x, double *y, double *z, int m1, int n1)
	{

	const int bs = 4;
	
	double yy[4] = {0.0, 0.0, 0.0, 0.0};

	double alpha1 = -1.0;
	double beta1  = 1.0;

	int k1 = kmax/bs*bs;

	kernel_dgemv_n_4_lib4(k1, &alpha1, A, x, &beta1, y, yy);

	A += k1*bs;

	double
		a_00, a_10, a_20, a_30,
		a_11, a_21, a_31;
	
	// a_00
//	a_00 = 1.0;
	a_10 = A[1+bs*0];
	a_20 = A[2+bs*0];
	a_30 = A[3+bs*0];
//	yy[0] *= a_00;
	z[0] = yy[0];
	yy[1] -= a_10 * yy[0];
	yy[2] -= a_20 * yy[0];
	yy[3] -= a_30 * yy[0];

	if(n1==1)
		{
		if(m1==1)
			return;
		z[1] = yy[1];
		if(m1==2)
			return;
		z[2] = yy[2];
		if(m1==3)
			return;
		z[3] = yy[3];
		return;
		}

	// a_11
//	a_11 = 1.0;
	a_21 = A[2+bs*1];
	a_31 = A[3+bs*1];
//	yy[1] *= a_11;	
	z[1] = yy[1];
	yy[2] -= a_21 * yy[1];
	yy[3] -= a_31 * yy[1];

	if(n1==2)
		{
		if(m1==2)
			return;
		z[2] = yy[2];
		if(m1==3)
			return;
		z[3] = yy[3];
		return;
		}

	// a_22
//	a_00 = 1.0;
	a_10 = A[3+bs*2];
//	yy[2] *= a_00;
	z[2] = yy[2];
	yy[3] -= a_10 * yy[2];

	if(n1==3)
		{
		if(m1==3)
			return;
		z[3] = yy[3];

		return;
		}

	// a_33
//	a_11 = 1.0;
//	yy[3] *= a_11;	
	z[3] = yy[3];

	return;

	}
#endif
	

	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_ln_one_4_lib4(int kmax, double *A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	double yy[4] = {0.0, 0.0, 0.0, 0.0};

	double alpha1 = -1.0;
	double beta1  = 1.0;

	int k1 = kmax/bs*bs;

	kernel_dgemv_n_4_lib4(k1, &alpha1, A, x, &beta1, y, yy);

	A += k1*bs;

	double
		a_00, a_10, a_20, a_30,
		a_11, a_21, a_31;
	
	// a_00
//	a_00 = 1.0;
	a_10 = A[1+bs*0];
	a_20 = A[2+bs*0];
	a_30 = A[3+bs*0];
//	yy[0] *= a_00;
	z[0] = yy[0];
	yy[1] -= a_10 * yy[0];
	yy[2] -= a_20 * yy[0];
	yy[3] -= a_30 * yy[0];

	// a_11
//	a_11 = 1.0;
	a_21 = A[2+bs*1];
	a_31 = A[3+bs*1];
//	yy[1] *= a_11;	
	z[1] = yy[1];
	yy[2] -= a_21 * yy[1];
	yy[3] -= a_31 * yy[1];

	// a_22
//	a_00 = 1.0;
	a_10 = A[3+bs*2];
//	yy[2] *= a_00;
	z[2] = yy[2];
	yy[3] -= a_10 * yy[2];

	// a_33
//	a_11 = 1.0;
//	yy[3] *= a_11;	
	z[3] = yy[3];

	return;

	}
#endif
	
	
		
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_inv_4_lib4(int kmax, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	double yy[4] = {0, 0, 0, 0};
	
	double alpha = -1.0;
	double beta = 1.0;
	kernel_dgemv_t_4_lib4(kmax-4, &alpha, 0, A+4+(sda-1)*bs, sda, x+4, &beta, y, yy);

	// bottom trinagle
	yy[3] *= inv_diag_A[3];
	z[3] = yy[3];

	yy[2] -= A[3+bs*2] * yy[3];
	yy[2] *= inv_diag_A[2];
	z[2] = yy[2];

	// square
	yy[0] -= A[2+bs*0]*yy[2] + A[3+bs*0]*yy[3];
	yy[1] -= A[2+bs*1]*yy[2] + A[3+bs*1]*yy[3];
		
	// top trinagle
	yy[1] *= inv_diag_A[1];
	z[1] = yy[1];

	yy[0] -= A[1+bs*0] * yy[1];
	yy[0] *= inv_diag_A[0];
	z[0] = yy[0];

	return;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_inv_3_lib4(int kmax, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	int
		k;
	
	double *tA, *tx;
	tA = A;
	tx = x;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0;
	
	k = 3;
	if(kmax>4)
		{
		// clean up at the beginning
		x_3 = x[3];

		y_0 -= A[3+bs*0] * x_3;
		y_1 -= A[3+bs*1] * x_3;
		y_2 -= A[3+bs*2] * x_3;

		k=4;
		A += 4 + (sda-1)*bs;
		x += 4;
		for(; k<kmax-3; k+=4)
			{
			
			x_0 = x[0];
			x_1 = x[1];
			x_2 = x[2];
			x_3 = x[3];
			
			y_0 -= A[0+bs*0] * x_0;
			y_1 -= A[0+bs*1] * x_0;
			y_2 -= A[0+bs*2] * x_0;

			y_0 -= A[1+bs*0] * x_1;
			y_1 -= A[1+bs*1] * x_1;
			y_2 -= A[1+bs*2] * x_1;
			
			y_0 -= A[2+bs*0] * x_2;
			y_1 -= A[2+bs*1] * x_2;
			y_2 -= A[2+bs*2] * x_2;

			y_0 -= A[3+bs*0] * x_3;
			y_1 -= A[3+bs*1] * x_3;
			y_2 -= A[3+bs*2] * x_3;
			
			A += sda*bs;
			x += 4;

			}
		}
	else
		{
		A += 3;
		x += 1;
		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
		
		y_0 -= A[0+bs*0] * x_0;
		y_1 -= A[0+bs*1] * x_0;
		y_2 -= A[0+bs*2] * x_0;
		
		A += 1;//sda*bs;
		x += 1;

		}

	y_0 = y[0] + y_0;
	y_1 = y[1] + y_1;
	y_2 = y[2] + y_2;

	A = tA;
	x = tx;

	// bottom trinagle
	y_2 *= inv_diag_A[2];
	z[2] = y_2;

	// square
	y_0 -= A[2+bs*0]*y_2;
	y_1 -= A[2+bs*1]*y_2;
		
	// top trinagle
	y_1 *= inv_diag_A[1];
	z[1] = y_1;

	y_0 -= A[1+bs*0] * y_1;
	y_0 *= inv_diag_A[0];
	z[0] = y_0;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_inv_2_lib4(int kmax, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	int
		k;
	
	double *tA, *tx;
	tA = A;
	tx = x;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0;
	
	k = 2;
	if(kmax>4)
		{
		// clean up at the beginning
		x_2 = x[2];
		x_3 = x[3];

		y_0 -= A[2+bs*0] * x_2;
		y_1 -= A[2+bs*1] * x_2;

		y_0 -= A[3+bs*0] * x_3;
		y_1 -= A[3+bs*1] * x_3;

		k=4;
		A += 4 + (sda-1)*bs;
		x += 4;
		for(; k<kmax-3; k+=4)
			{
			
			x_0 = x[0];
			x_1 = x[1];
			x_2 = x[2];
			x_3 = x[3];
			
			y_0 -= A[0+bs*0] * x_0;
			y_1 -= A[0+bs*1] * x_0;

			y_0 -= A[1+bs*0] * x_1;
			y_1 -= A[1+bs*1] * x_1;
			
			y_0 -= A[2+bs*0] * x_2;
			y_1 -= A[2+bs*1] * x_2;

			y_0 -= A[3+bs*0] * x_3;
			y_1 -= A[3+bs*1] * x_3;
			
			A += sda*bs;
			x += 4;

			}
		}
	else
		{
		A += 2;
		x += 2;
		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
		
		y_0 -= A[0+bs*0] * x_0;
		y_1 -= A[0+bs*1] * x_0;
		
		A += 1;//sda*bs;
		x += 1;

		}

	y_0 = y[0] + y_0;
	y_1 = y[1] + y_1;

	A = tA;
	x = tx;

	// top trinagle
	y_1 *= inv_diag_A[1];
	z[1] = y_1;

	y_0 -= A[1+bs*0] * y_1;
	y_0 *= inv_diag_A[0];
	z[0] = y_0;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_inv_1_lib4(int kmax, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	int
		k;
	
	double *tA, *tx;
	tA = A;
	tx = x;

	double
		x_0, x_1, x_2, x_3,
		y_0=0;
	
	k = 1;
	if(kmax>4)
		{
		// clean up at the beginning
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 -= A[1+bs*0] * x_1;
		y_0 -= A[2+bs*0] * x_2;
		y_0 -= A[3+bs*0] * x_3;

		k=4;
		A += 4 + (sda-1)*bs;
		x += 4;
		for(; k<kmax-3; k+=4)
			{
			
			x_0 = x[0];
			x_1 = x[1];
			x_2 = x[2];
			x_3 = x[3];
			
			y_0 -= A[0+bs*0] * x_0;
			y_0 -= A[1+bs*0] * x_1;
			y_0 -= A[2+bs*0] * x_2;
			y_0 -= A[3+bs*0] * x_3;
			
			A += sda*bs;
			x += 4;

			}
		}
	else
		{
		A += 1;
		x += 1;
		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
		
		y_0 -= A[0+bs*0] * x_0;
		
		A += 1;//sda*bs;
		x += 1;

		}

	y_0 = y[0] + y_0;

	A = tA;
	x = tx;

	// top trinagle
	y_0 *= inv_diag_A[0];
	z[0] = y_0;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_one_4_lib4(int kmax, double *A, int sda, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	double yy[4] = {0, 0, 0, 0};
	
	double alpha = -1.0;
	double beta = 1.0;

	kernel_dgemv_t_4_lib4(kmax-4, &alpha, 0, A+4+(sda-1)*bs, sda, x+4, &beta, y, yy);

	// bottom trinagle
	z[3] = yy[3];

	yy[2] -= A[3+bs*2] * yy[3];
	z[2] = yy[2];

	// square
	yy[0] -= A[2+bs*0]*yy[2] + A[3+bs*0]*yy[3];
	yy[1] -= A[2+bs*1]*yy[2] + A[3+bs*1]*yy[3];
		
	// top trinagle
	z[1] = yy[1];

	yy[0] -= A[1+bs*0] * yy[1];
	z[0] = yy[0];

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_one_3_lib4(int kmax, double *A, int sda, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	int
		k;
	
	double *tA, *tx;
	tA = A;
	tx = x;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0;
	
	k = 3;
	if(kmax>4)
		{
		// clean up at the beginning
		x_3 = x[3];

		y_0 -= A[3+bs*0] * x_3;
		y_1 -= A[3+bs*1] * x_3;
		y_2 -= A[3+bs*2] * x_3;

		k=4;
		A += 4 + (sda-1)*bs;
		x += 4;
		for(; k<kmax-3; k+=4)
			{
			
			x_0 = x[0];
			x_1 = x[1];
			x_2 = x[2];
			x_3 = x[3];
			
			y_0 -= A[0+bs*0] * x_0;
			y_1 -= A[0+bs*1] * x_0;
			y_2 -= A[0+bs*2] * x_0;

			y_0 -= A[1+bs*0] * x_1;
			y_1 -= A[1+bs*1] * x_1;
			y_2 -= A[1+bs*2] * x_1;
			
			y_0 -= A[2+bs*0] * x_2;
			y_1 -= A[2+bs*1] * x_2;
			y_2 -= A[2+bs*2] * x_2;

			y_0 -= A[3+bs*0] * x_3;
			y_1 -= A[3+bs*1] * x_3;
			y_2 -= A[3+bs*2] * x_3;
			
			A += sda*bs;
			x += 4;

			}
		}
	else
		{
		A += 3;
		x += 1;
		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
		
		y_0 -= A[0+bs*0] * x_0;
		y_1 -= A[0+bs*1] * x_0;
		y_2 -= A[0+bs*2] * x_0;
		
		A += 1;//sda*bs;
		x += 1;

		}

	y_0 = y[0] + y_0;
	y_1 = y[1] + y_1;
	y_2 = y[2] + y_2;

	A = tA;
	x = tx;

	// bottom trinagle
	z[2] = y_2;

	// square
	y_0 -= A[2+bs*0]*y_2;
	y_1 -= A[2+bs*1]*y_2;
		
	// top trinagle
	z[1] = y_1;

	y_0 -= A[1+bs*0] * y_1;
	z[0] = y_0;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_one_2_lib4(int kmax, double *A, int sda, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	int
		k;
	
	double *tA, *tx;
	tA = A;
	tx = x;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0;
	
	k = 2;
	if(kmax>4)
		{
		// clean up at the beginning
		x_2 = x[2];
		x_3 = x[3];

		y_0 -= A[2+bs*0] * x_2;
		y_1 -= A[2+bs*1] * x_2;

		y_0 -= A[3+bs*0] * x_3;
		y_1 -= A[3+bs*1] * x_3;

		k=4;
		A += 4 + (sda-1)*bs;
		x += 4;
		for(; k<kmax-3; k+=4)
			{
			
			x_0 = x[0];
			x_1 = x[1];
			x_2 = x[2];
			x_3 = x[3];
			
			y_0 -= A[0+bs*0] * x_0;
			y_1 -= A[0+bs*1] * x_0;

			y_0 -= A[1+bs*0] * x_1;
			y_1 -= A[1+bs*1] * x_1;
			
			y_0 -= A[2+bs*0] * x_2;
			y_1 -= A[2+bs*1] * x_2;

			y_0 -= A[3+bs*0] * x_3;
			y_1 -= A[3+bs*1] * x_3;
			
			A += sda*bs;
			x += 4;

			}
		}
	else
		{
		A += 2;
		x += 2;
		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
		
		y_0 -= A[0+bs*0] * x_0;
		y_1 -= A[0+bs*1] * x_0;
		
		A += 1;//sda*bs;
		x += 1;

		}

	y_0 = y[0] + y_0;
	y_1 = y[1] + y_1;

	A = tA;
	x = tx;

	// top trinagle
	z[1] = y_1;

	y_0 -= A[1+bs*0] * y_1;
	z[0] = y_0;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_lt_one_1_lib4(int kmax, double *A, int sda, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	int
		k;
	
	double *tA, *tx;
	tA = A;
	tx = x;

	double
		x_0, x_1, x_2, x_3,
		y_0=0;
	
	k = 1;
	if(kmax>4)
		{
		// clean up at the beginning
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 -= A[1+bs*0] * x_1;
		y_0 -= A[2+bs*0] * x_2;
		y_0 -= A[3+bs*0] * x_3;

		k=4;
		A += 4 + (sda-1)*bs;
		x += 4;
		for(; k<kmax-3; k+=4)
			{
			
			x_0 = x[0];
			x_1 = x[1];
			x_2 = x[2];
			x_3 = x[3];
			
			y_0 -= A[0+bs*0] * x_0;
			y_0 -= A[1+bs*0] * x_1;
			y_0 -= A[2+bs*0] * x_2;
			y_0 -= A[3+bs*0] * x_3;
			
			A += sda*bs;
			x += 4;

			}
		}
	else
		{
		A += 1;
		x += 1;
		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
		
		y_0 -= A[0+bs*0] * x_0;
		
		A += 1;//sda*bs;
		x += 1;

		}

	y_0 = y[0] + y_0;

	A = tA;
	x = tx;

	// top trinagle
	z[0] = y_0;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_un_inv_4_lib4(int kmax, double *A, double *inv_diag_A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	double yy[4] = {0, 0, 0, 0};
	
	double alpha = -1.0;
	double beta = 1.0;

	kernel_dgemv_n_4_lib4(kmax-4, &alpha, A+4*bs, x+4, &beta, y, yy);

	// bottom trinagle
	yy[3] *= inv_diag_A[3];
	z[3] = yy[3];

	yy[2] -= A[2+bs*3] * yy[3];
	yy[2] *= inv_diag_A[2];
	z[2] = yy[2];

	// square
	yy[0] -= A[0+bs*2]*yy[2] + A[0+bs*3]*yy[3];
	yy[1] -= A[1+bs*2]*yy[2] + A[1+bs*3]*yy[3];
		
	// top trinagle
	yy[1] *= inv_diag_A[1];
	z[1] = yy[1];

	yy[0] -= A[0+bs*1] * yy[1];
	yy[0] *= inv_diag_A[0];
	z[0] = yy[0];

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_ut_inv_4_vs_lib4(int kmax, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z, int m1, int n1)
	{

	const int bs = 4;
	
	double yy[4] = {0, 0, 0, 0};
	
	int k1 = kmax/bs*bs;
	double alpha = -1.0;
	double beta = 1.0;

	kernel_dgemv_t_4_lib4(k1, &alpha, 0, A, sda, x, &beta, y, yy);

	A += sda*k1;

	double
		a_00, a_10, a_20, a_30,
		a_11, a_21, a_31;
	
	// a_00
	a_00 = inv_diag_A[0];
	a_10 = A[0+bs*1];
	a_20 = A[0+bs*2];
	a_30 = A[0+bs*3];
	yy[0] *= a_00;
	z[0] = yy[0];
	yy[1] -= a_10 * yy[0];
	yy[2] -= a_20 * yy[0];
	yy[3] -= a_30 * yy[0];

	if(n1==1)
		{
		if(m1==1)
			return;
		z[1] = yy[1];
		if(m1==2)
			return;
		z[2] = yy[2];
		if(m1==3)
			return;
		z[3] = yy[3];
		return;
		}

	// a_11
	a_11 = inv_diag_A[1];
	a_21 = A[1+bs*2];
	a_31 = A[1+bs*3];
	yy[1] *= a_11;	
	z[1] = yy[1];
	yy[2] -= a_21 * yy[1];
	yy[3] -= a_31 * yy[1];

	if(n1==2)
		{
		if(m1==2)
			return;
		z[2] = yy[2];
		if(m1==3)
			return;
		z[3] = yy[3];
		return;
		}

	// a_22
	a_00 = inv_diag_A[2];
	a_10 = A[2+bs*3];
	yy[2] *= a_00;
	z[2] = yy[2];
	yy[3] -= a_10 * yy[2];

	if(n1==3)
		{
		if(m1==3)
			return;
		z[3] = yy[3];

		return;
		}

	// a_33
	a_11 = inv_diag_A[3];
	yy[3] *= a_11;	
	z[3] = yy[3];

	return;

	}
#endif
	

	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrsv_ut_inv_4_lib4(int kmax, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z)
	{

	const int bs = 4;
	
	double yy[4] = {0, 0, 0, 0};
	
	int k1 = kmax/bs*bs;
	double alpha = -1.0;
	double beta = 1.0;

	kernel_dgemv_t_4_lib4(k1, &alpha, 0, A, sda, x, &beta, y, yy);

	A += sda*k1;

	double
		a_00, a_10, a_20, a_30,
		a_11, a_21, a_31;
	
	// a_00
	a_00 = inv_diag_A[0];
	a_10 = A[0+bs*1];
	a_20 = A[0+bs*2];
	a_30 = A[0+bs*3];
	yy[0] *= a_00;
	z[0] = yy[0];
	yy[1] -= a_10 * yy[0];
	yy[2] -= a_20 * yy[0];
	yy[3] -= a_30 * yy[0];

	// a_11
	a_11 = inv_diag_A[1];
	a_21 = A[1+bs*2];
	a_31 = A[1+bs*3];
	yy[1] *= a_11;	
	z[1] = yy[1];
	yy[2] -= a_21 * yy[1];
	yy[3] -= a_31 * yy[1];

	// a_22
	a_00 = inv_diag_A[2];
	a_10 = A[2+bs*3];
	yy[2] *= a_00;
	z[2] = yy[2];
	yy[3] -= a_10 * yy[2];

	// a_33
	a_11 = inv_diag_A[3];
	yy[3] *= a_11;	
	z[3] = yy[3];

	return;

	}
#endif



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrmv_un_4_lib4(int kmax, double *A, double *x, double *z)
	{

	const int bs = 4;
	
	double yy[4] = {0, 0, 0, 0};
	
	double x_0, x_1, x_2, x_3;
	
	x_0 = x[0];
	x_1 = x[1];
	x_2 = x[2];
	x_3 = x[3];

	yy[0] += A[0+bs*0] * x_0;
/*	yy[1] += A[1+bs*0] * x_0;*/
/*	yy[2] += A[2+bs*0] * x_0;*/
/*	yy[3] += A[3+bs*0] * x_0;*/

	yy[0] += A[0+bs*1] * x_1;
	yy[1] += A[1+bs*1] * x_1;
/*	yy[2] += A[2+bs*1] * x_1;*/
/*	yy[3] += A[3+bs*1] * x_1;*/

	yy[0] += A[0+bs*2] * x_2;
	yy[1] += A[1+bs*2] * x_2;
	yy[2] += A[2+bs*2] * x_2;
/*	yy[3] += A[3+bs*2] * x_2;*/

	yy[0] += A[0+bs*3] * x_3;
	yy[1] += A[1+bs*3] * x_3;
	yy[2] += A[2+bs*3] * x_3;
	yy[3] += A[3+bs*3] * x_3;
	
	double alpha1 = 1.0;
	double beta1  = 1.0;

	kernel_dgemv_n_4_lib4(kmax-4, &alpha1, A+4*bs, x+4, &beta1, yy, z);

	return;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrmv_ut_4_vs_lib4(int kmax, double *A, int sda, double *x, double *z, int m1)
	{

	const int bs  = 4;

	double yy[4] = {0, 0, 0, 0};
	
	double x_0, x_1, x_2, x_3;
	
	int k1 = kmax/bs*bs;
	double alpha1 = 1.0;
	double beta1  = 1.0;

	kernel_dgemv_t_4_lib4(k1, &alpha1, 0, A, sda, x, &beta1, yy, yy);

	A += k1*sda;
	x += k1;
	
	x_0 = x[0];
	x_1 = x[1];
	x_2 = x[2];
	x_3 = x[3];

	yy[0] += A[0+bs*0] * x_0;
	yy[1] += A[0+bs*1] * x_0;
	yy[2] += A[0+bs*2] * x_0;
	yy[3] += A[0+bs*3] * x_0;

/*	yy[0] += A[1+bs*0] * x_1;*/
	yy[1] += A[1+bs*1] * x_1;
	yy[2] += A[1+bs*2] * x_1;
	yy[3] += A[1+bs*3] * x_1;
	
/*	yy[0] += A[2+bs*0] * x_2;*/
/*	yy[1] += A[2+bs*1] * x_2;*/
	yy[2] += A[2+bs*2] * x_2;
	yy[3] += A[2+bs*3] * x_2;

/*	yy[0] += A[3+bs*0] * x_3;*/
/*	yy[1] += A[3+bs*1] * x_3;*/
/*	yy[2] += A[3+bs*2] * x_3;*/
	yy[3] += A[3+bs*3] * x_3;
	
//	A += sda*bs;
//	x += 4;

	// store_vs
	if(m1>=4)
		{
		z[0] = yy[0];
		z[1] = yy[1];
		z[2] = yy[2];
		z[3] = yy[3];
		}
	else
		{
		z[0] = yy[0];
		if(m1>=2)
			{
			z[1] = yy[1];
			if(m1>2)
				{
				z[2] = yy[2];
				}
			}
		}
	
	return;

	}
#endif
	
	
	
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dtrmv_ut_4_lib4(int kmax, double *A, int sda, double *x, double *z)
	{
	
	const int bs  = 4;

	double yy[4] = {0, 0, 0, 0};
	
	double x_0, x_1, x_2, x_3;
	
	int k1 = kmax/bs*bs;
	double alpha1 = 1.0;
	double beta1  = 1.0;

	kernel_dgemv_t_4_lib4(k1, &alpha1, 0, A, sda, x, &beta1, yy, yy);

	A += k1*sda;
	x += k1;
	
	x_0 = x[0];
	x_1 = x[1];
	x_2 = x[2];
	x_3 = x[3];

	yy[0] += A[0+bs*0] * x_0;
	yy[1] += A[0+bs*1] * x_0;
	yy[2] += A[0+bs*2] * x_0;
	yy[3] += A[0+bs*3] * x_0;

/*	yy[0] += A[1+bs*0] * x_1;*/
	yy[1] += A[1+bs*1] * x_1;
	yy[2] += A[1+bs*2] * x_1;
	yy[3] += A[1+bs*3] * x_1;
	
/*	yy[0] += A[2+bs*0] * x_2;*/
/*	yy[1] += A[2+bs*1] * x_2;*/
	yy[2] += A[2+bs*2] * x_2;
	yy[3] += A[2+bs*3] * x_2;

/*	yy[0] += A[3+bs*0] * x_3;*/
/*	yy[1] += A[3+bs*1] * x_3;*/
/*	yy[2] += A[3+bs*2] * x_3;*/
	yy[3] += A[3+bs*3] * x_3;
	
//	A += sda*bs;
//	x += 4;

	z[0] = yy[0];
	z[1] = yy[1];
	z[2] = yy[2];
	z[3] = yy[3];
	
	return;

	}
#endif



//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemv_4_lib.c"

#endif

