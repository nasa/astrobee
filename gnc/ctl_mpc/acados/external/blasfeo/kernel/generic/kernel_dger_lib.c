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



#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dger_4_libc(int kmax, double *alpha, double *x, double *y, double *C, int ldc, double *D, int ldd)
	{

	if(kmax<=0)
		return;

	int ii;

	double
		x_0,
		y_0, y_1, y_2, y_3;
	
	y_0 = *alpha * y[0];
	y_1 = *alpha * y[1];
	y_2 = *alpha * y[2];
	y_3 = *alpha * y[3];

	ii = 0;
	for(; ii<kmax-3; ii+=4)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;
		D[0+ldd*1] = C[0+ldc*1] + x_0 * y_1;
		D[0+ldd*2] = C[0+ldc*2] + x_0 * y_2;
		D[0+ldd*3] = C[0+ldc*3] + x_0 * y_3;

		x_0 = x[1];
		D[1+ldd*0] = C[1+ldc*0] + x_0 * y_0;
		D[1+ldd*1] = C[1+ldc*1] + x_0 * y_1;
		D[1+ldd*2] = C[1+ldc*2] + x_0 * y_2;
		D[1+ldd*3] = C[1+ldc*3] + x_0 * y_3;

		x_0 = x[2];
		D[2+ldd*0] = C[2+ldc*0] + x_0 * y_0;
		D[2+ldd*1] = C[2+ldc*1] + x_0 * y_1;
		D[2+ldd*2] = C[2+ldc*2] + x_0 * y_2;
		D[2+ldd*3] = C[2+ldc*3] + x_0 * y_3;

		x_0 = x[3];
		D[3+ldd*0] = C[3+ldc*0] + x_0 * y_0;
		D[3+ldd*1] = C[3+ldc*1] + x_0 * y_1;
		D[3+ldd*2] = C[3+ldc*2] + x_0 * y_2;
		D[3+ldd*3] = C[3+ldc*3] + x_0 * y_3;

		C += 4;
		D += 4;
		x += 4;

		}
	for(; ii<kmax; ii++)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;
		D[0+ldd*1] = C[0+ldc*1] + x_0 * y_1;
		D[0+ldd*2] = C[0+ldc*2] + x_0 * y_2;
		D[0+ldd*3] = C[0+ldc*3] + x_0 * y_3;

		C += 1;
		D += 1;
		x += 1;

		}

	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dger_3_libc(int kmax, double *alpha, double *x, double *y, double *C, int ldc, double *D, int ldd)
	{

	if(kmax<=0)
		return;

	int ii;

	double
		x_0,
		y_0, y_1, y_2;
	
	y_0 = *alpha * y[0];
	y_1 = *alpha * y[1];
	y_2 = *alpha * y[2];

	ii = 0;
	for(; ii<kmax-3; ii+=4)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;
		D[0+ldd*1] = C[0+ldc*1] + x_0 * y_1;
		D[0+ldd*2] = C[0+ldc*2] + x_0 * y_2;

		x_0 = x[1];
		D[1+ldd*0] = C[1+ldc*0] + x_0 * y_0;
		D[1+ldd*1] = C[1+ldc*1] + x_0 * y_1;
		D[1+ldd*2] = C[1+ldc*2] + x_0 * y_2;

		x_0 = x[2];
		D[2+ldd*0] = C[2+ldc*0] + x_0 * y_0;
		D[2+ldd*1] = C[2+ldc*1] + x_0 * y_1;
		D[2+ldd*2] = C[2+ldc*2] + x_0 * y_2;

		x_0 = x[3];
		D[3+ldd*0] = C[3+ldc*0] + x_0 * y_0;
		D[3+ldd*1] = C[3+ldc*1] + x_0 * y_1;
		D[3+ldd*2] = C[3+ldc*2] + x_0 * y_2;

		C += 4;
		D += 4;
		x += 4;

		}
	for(; ii<kmax; ii++)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;
		D[0+ldd*1] = C[0+ldc*1] + x_0 * y_1;
		D[0+ldd*2] = C[0+ldc*2] + x_0 * y_2;

		C += 1;
		D += 1;
		x += 1;

		}

	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dger_2_libc(int kmax, double *alpha, double *x, double *y, double *C, int ldc, double *D, int ldd)
	{

	if(kmax<=0)
		return;

	int ii;

	double
		x_0,
		y_0, y_1;
	
	y_0 = *alpha * y[0];
	y_1 = *alpha * y[1];

	ii = 0;
	for(; ii<kmax-3; ii+=4)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;
		D[0+ldd*1] = C[0+ldc*1] + x_0 * y_1;

		x_0 = x[1];
		D[1+ldd*0] = C[1+ldc*0] + x_0 * y_0;
		D[1+ldd*1] = C[1+ldc*1] + x_0 * y_1;

		x_0 = x[2];
		D[2+ldd*0] = C[2+ldc*0] + x_0 * y_0;
		D[2+ldd*1] = C[2+ldc*1] + x_0 * y_1;

		x_0 = x[3];
		D[3+ldd*0] = C[3+ldc*0] + x_0 * y_0;
		D[3+ldd*1] = C[3+ldc*1] + x_0 * y_1;

		C += 4;
		D += 4;
		x += 4;

		}
	for(; ii<kmax; ii++)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;
		D[0+ldd*1] = C[0+ldc*1] + x_0 * y_1;

		C += 1;
		D += 1;
		x += 1;

		}

	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
static void kernel_dger_1_libc(int kmax, double *alpha, double *x, double *y, double *C, int ldc, double *D, int ldd)
	{

	if(kmax<=0)
		return;

	int ii;

	double
		x_0,
		y_0;
	
	y_0 = *alpha * y[0];

	ii = 0;
	for(; ii<kmax-3; ii+=4)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;

		x_0 = x[1];
		D[1+ldd*0] = C[1+ldc*0] + x_0 * y_0;

		x_0 = x[2];
		D[2+ldd*0] = C[2+ldc*0] + x_0 * y_0;

		x_0 = x[3];
		D[3+ldd*0] = C[3+ldc*0] + x_0 * y_0;

		C += 4;
		D += 4;
		x += 4;

		}
	for(; ii<kmax; ii++)
		{

		x_0 = x[0];
		D[0+ldd*0] = C[0+ldc*0] + x_0 * y_0;

		C += 1;
		D += 1;
		x += 1;

		}

	return;

	}
#endif



#if 1 //defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dger_4_vs_libc(int kmax, double *alpha, double *x, double *y, double *C, int ldc, double *D, int ldd, int km)
	{

	if(kmax<=0)
		return;
	
	if(km==1)
		{
		kernel_dger_1_libc(kmax, alpha, x, y, C, ldc, D, ldd);
		}
	else if(km==2)
		{
		kernel_dger_2_libc(kmax, alpha, x, y, C, ldc, D, ldd);
		}
	else if(km==3)
		{
		kernel_dger_3_libc(kmax, alpha, x, y, C, ldc, D, ldd);
		}
	else
		{
		kernel_dger_4_libc(kmax, alpha, x, y, C, ldc, D, ldd);
		}

	return;

	}
#endif

