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



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A53) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
void kernel_sgemv_nt_4_vs_lib4(int kmax, float *alpha_n, float *alpha_t, float *A, int sda, float *x_n, float *x_t, float *beta_t, float *y_t, float *z_n, float *z_t, int km)
	{

	if(kmax<=0) 
		return;
	
	const int bs = 4;

	int k;

	float
		a_00, a_01, a_02, a_03,
		x_n_0, x_n_1, x_n_2, x_n_3, y_n_0,
		x_t_0, y_t_0, y_t_1, y_t_2, y_t_3;
	
	x_n_0 = 0;
	x_n_1 = 0;
	x_n_2 = 0;
	x_n_3 = 0;

	x_n_0 = alpha_n[0]*x_n[0];
	if(km>1)
		{
		x_n_1 = alpha_n[0]*x_n[1];
		if(km>2)
			{
			x_n_2 = alpha_n[0]*x_n[2];
			if(km>3)
				{
				x_n_3 = alpha_n[0]*x_n[3];
				}
			}
		}

	y_t_0 = 0;
	y_t_1 = 0;
	y_t_2 = 0;
	y_t_3 = 0;

	k = 0;
	for(; k<kmax-3; k+=bs)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		a_00 = A[1+bs*0];
		a_01 = A[1+bs*1];
		a_02 = A[1+bs*2];
		a_03 = A[1+bs*3];
		
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

		a_00 = A[2+bs*0];
		a_01 = A[2+bs*1];
		a_02 = A[2+bs*2];
		a_03 = A[2+bs*3];
		
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

		a_00 = A[3+bs*0];
		a_01 = A[3+bs*1];
		a_02 = A[3+bs*2];
		a_03 = A[3+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[3] = y_n_0;


		A += sda*bs;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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
	
	// store t
	if(beta_t[0]==0.0)
		{
		z_t[0] = alpha_t[0]*y_t_0;
		if(km>1)
			{
			z_t[1] = alpha_t[0]*y_t_1;
			if(km>2)
				{
				z_t[2] = alpha_t[0]*y_t_2;
				if(km>3)
					{
					z_t[3] = alpha_t[0]*y_t_3;
					}
				}
			}
		}
	else
		{
		z_t[0] = alpha_t[0]*y_t_0 + beta_t[0]*y_t[0];
		if(km>1)
			{
			z_t[1] = alpha_t[0]*y_t_1 + beta_t[0]*y_t[1];
			if(km>2)
				{
				z_t[2] = alpha_t[0]*y_t_2 + beta_t[0]*y_t[2];
				if(km>3)
					{
					z_t[3] = alpha_t[0]*y_t_3 + beta_t[0]*y_t[3];
					}
				}
			}
		}

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A53) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
void kernel_sgemv_nt_4_lib4(int kmax, float *alpha_n, float *alpha_t, float *A, int sda, float *x_n, float *x_t, float *beta_t, float *y_t, float *z_n, float *z_t)
	{

	kernel_sgemv_nt_4_vs_lib4(kmax, alpha_n, alpha_t, A, sda, x_n, x_t, beta_t, y_t, z_n, z_t, 4);

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A53) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
void kernel_ssymv_l_4_gen_lib4(int kmax, float *alpha, int offA, float *A, int sda, float *x_n, float *z_n, int km)
	{

	if(kmax<=0) 
		return;
	
	float *x_t = x_n;
	float *z_t = z_n;

	const int bs = 4;

	int k;

	float
		a_00, a_01, a_02, a_03,
		x_n_0, x_n_1, x_n_2, x_n_3, y_n_0,
		x_t_0, y_t_0, y_t_1, y_t_2, y_t_3;
	
	x_n_0 = 0;
	x_n_1 = 0;
	x_n_2 = 0;
	x_n_3 = 0;

	x_n_0 = alpha[0]*x_n[0];
	if(km>1)
		{
		x_n_1 = alpha[0]*x_n[1];
		if(km>2)
			{
			x_n_2 = alpha[0]*x_n[2];
			if(km>3)
				{
				x_n_3 = alpha[0]*x_n[3];
				}
			}
		}

	y_t_0 = 0;
	y_t_1 = 0;
	y_t_2 = 0;
	y_t_3 = 0;

	k = 0;
	if(offA==0)
		{
		if(kmax<4)
			{
			// 0

			x_t_0 = x_t[0];

			a_00 = A[0+bs*0];
			
			y_t_0 += a_00 * x_t_0;

			if(kmax==1)
				goto store_t;

			// 1

			y_n_0 = z_n[1]; 
			x_t_0 = x_t[1];

			a_00 = A[1+bs*0];
			a_01 = A[1+bs*1];
			
			y_n_0 += a_00 * x_n_0;
			y_t_0 += a_00 * x_t_0;
			y_t_1 += a_01 * x_t_0;

			z_n[1] = y_n_0;

			if(kmax==2)
				goto store_t;

			// 2

			y_n_0 = z_n[2]; 
			x_t_0 = x_t[2];

			a_00 = A[2+bs*0];
			a_01 = A[2+bs*1];
			a_02 = A[2+bs*2];
			
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

			a_00 = A[0+bs*0];
			
			y_t_0 += a_00 * x_t_0;


			// 1

			y_n_0 = z_n[1]; 
			x_t_0 = x_t[1];

			a_00 = A[1+bs*0];
			a_01 = A[1+bs*1];
			
			y_n_0 += a_00 * x_n_0;
			y_t_0 += a_00 * x_t_0;
			y_t_1 += a_01 * x_t_0;

			z_n[1] = y_n_0;


			// 2

			y_n_0 = z_n[2]; 
			x_t_0 = x_t[2];

			a_00 = A[2+bs*0];
			a_01 = A[2+bs*1];
			a_02 = A[2+bs*2];
			
			y_n_0 += a_00 * x_n_0;
			y_t_0 += a_00 * x_t_0;
			y_n_0 += a_01 * x_n_1;
			y_t_1 += a_01 * x_t_0;
			y_t_2 += a_02 * x_t_0;

			z_n[2] = y_n_0;


			// 3

			y_n_0 = z_n[3]; 
			x_t_0 = x_t[3];

			a_00 = A[3+bs*0];
			a_01 = A[3+bs*1];
			a_02 = A[3+bs*2];
			a_03 = A[3+bs*3];
			
			y_n_0 += a_00 * x_n_0;
			y_t_0 += a_00 * x_t_0;
			y_n_0 += a_01 * x_n_1;
			y_t_1 += a_01 * x_t_0;
			y_n_0 += a_02 * x_n_2;
			y_t_2 += a_02 * x_t_0;
			y_t_3 += a_03 * x_t_0;

			z_n[3] = y_n_0;

			k += 4;
			A += sda*bs;
			z_n += 4;
			x_t += 4;

			}
		}
	else if(offA==1)
		{

		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		
		y_t_0 += a_00 * x_t_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==1)
			goto store_t;

		// 1

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==2)
			goto store_t;

		// 2

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_t_2 += a_02 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		A += (sda-1)*bs; // new panel

		if(kmax==3)
			goto store_t;

		// 3

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_t_3 += a_03 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==4)
			goto store_t;

		// 4

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		if(kmax==5)
			goto store_t;

		// 5

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		if(kmax==6)
			goto store_t;

		// 6

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		A += (sda-1)*bs; // new panel

		if(kmax==7)
			goto store_t;

		k += 7;

		}
	else if(offA==2)
		{

		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		
		y_t_0 += a_00 * x_t_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==1)
			goto store_t;

		// 1

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		A += (sda-1)*bs; // new panel

		if(kmax==2)
			goto store_t;

		// 2

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_t_2 += a_02 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==3)
			goto store_t;

		// 3

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_t_3 += a_03 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==4)
			goto store_t;

		// 4

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		if(kmax==5)
			goto store_t;

		// 5

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		A += (sda-1)*bs; // new panel

		if(kmax==6)
			goto store_t;

		k += 6;

		}
	else // if(offA==3)
		{

		// 0

		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		
		y_t_0 += a_00 * x_t_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		A += (sda-1)*bs; // new panel

		if(kmax==1)
			goto store_t;

		// 1

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_t_1 += a_01 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==2)
			goto store_t;

		// 2

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_t_2 += a_02 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==3)
			goto store_t;

		// 3

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_t_3 += a_03 * x_t_0;

		z_n[0] = y_n_0;

		A += 1;
		z_n += 1;
		x_t += 1;

		if(kmax==4)
			goto store_t;

		// 4

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		A += (sda-1)*bs; // new panel

		if(kmax==5)
			goto store_t;

		k += 5;

		}
	for(; k<kmax-3; k+=bs)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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

		a_00 = A[1+bs*0];
		a_01 = A[1+bs*1];
		a_02 = A[1+bs*2];
		a_03 = A[1+bs*3];
		
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

		a_00 = A[2+bs*0];
		a_01 = A[2+bs*1];
		a_02 = A[2+bs*2];
		a_03 = A[2+bs*3];
		
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

		a_00 = A[3+bs*0];
		a_01 = A[3+bs*1];
		a_02 = A[3+bs*2];
		a_03 = A[3+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;

		z_n[3] = y_n_0;


		A += sda*bs;
		z_n += 4;
		x_t += 4;

		}
	for(; k<kmax; k++)
		{

		// 0

		y_n_0 = z_n[0]; 
		x_t_0 = x_t[0];

		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
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
	if(km>1)
		{
		z_t[1] += alpha[0]*y_t_1;
		if(km>2)
			{
			z_t[2] += alpha[0]*y_t_2;
			if(km>3)
				{
				z_t[3] += alpha[0]*y_t_3;
				}
			}
		}

	return;

	}
#endif



// XXX copy and scale y_n into z_n outside the kernel !!!!!
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A53) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
void kernel_ssymv_l_4_lib4(int kmax, float *alpha, float *A, int sda, float *x_n, float *z_n)
	{

	kernel_ssymv_l_4_gen_lib4(kmax, alpha, 0, A, sda, x_n, z_n, 4);

	return;

	}
#endif





