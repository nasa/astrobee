/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS for embedded optimization.                                                      *
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

#include <stdlib.h>
#include <stdio.h>

//#include <sys/mman.h>

#include <blasfeo_target.h>
#include <blasfeo_block_size.h>
#include <blasfeo_common.h>
#include <blasfeo_stdlib.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_s_kernel.h>

#include <blasfeo_memory.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_hp_sgemm_nn blasfeo_hp_cm_sgemm_nn
#define blasfeo_hp_sgemm_nt blasfeo_hp_cm_sgemm_nt
#define blasfeo_hp_sgemm_tn blasfeo_hp_cm_sgemm_tn
#define blasfeo_hp_sgemm_tt blasfeo_hp_cm_sgemm_tt
#define blasfeo_sgemm_nn blasfeo_cm_sgemm_nn
#define blasfeo_sgemm_nt blasfeo_cm_sgemm_nt
#define blasfeo_sgemm_tn blasfeo_cm_sgemm_tn
#define blasfeo_sgemm_tt blasfeo_cm_sgemm_tt
#endif



#define CACHE_LINE_EL S_CACHE_LINE_EL
#define L1_CACHE_EL S_L1_CACHE_EL
#define L2_CACHE_EL S_L2_CACHE_EL
#define LLC_CACHE_EL S_LLC_CACHE_EL
#define PS S_PS
#define M_KERNEL S_M_KERNEL
#define KC S_KC
#define NC S_NC
#define MC S_MC



static void blasfeo_hp_sgemm_nt_m2(int m, int n, int k, float alpha, float *pA, int sda, float *pB, int sdb, float beta, float *C, int ldc, float *D, int ldd)
	{

	int ii, jj;

	float *pA_p, *pB_p;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
#if defined(TARGET_X64_INTEL_HASWELL)
		jj = 0;
		if(n>0)
			{
			pA_p = m-ii<=24 ? pA : pA+(ii+24)*sda;
			pB_p = pB;
			kernel_sgemm_nt_24xn_p0_lib88cc(n, k, &alpha, pA+ii*sda, sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
			jj += n;
			}
#else
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
#endif
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_m2_left_8;
			}
		else if(m-ii<=16)
			{
			goto nt_m2_left_16;
			}
		else
			{
			goto nt_m2_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_m2_left_8;
			}
		else
			{
			goto nt_m2_left_16;
			}
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		jj = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda+4*sda, pB+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_m2_left_4;
			}
		else
			{
			goto nt_m2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, pA+ii*sda+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m2_left_4;
		}
#endif
	goto nt_m2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nt_m2_left_24:
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_m2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_m2_left_16:
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_m2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_m2_left_8:
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_m2_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda+4*sda, pB+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
		}
	goto nt_m2_return;
#endif

nt_m2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;

nt_m2_return:
	return;

	return;

	}



static void blasfeo_hp_sgemm_nn_m1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_nn_24_lib8(k, A+ii, lda, pU, sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_24x4_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nn_m1_left_8;
			}
		if(m-ii<=16)
			{
			goto nn_m1_left_16;
			}
		else
			{
			goto nn_m1_left_24;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_nn_16_lib8(k, A+ii, lda, pU, sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_16x4_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nn_m1_left_8;
			}
		else
			{
			goto nn_m1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib8(k, A+ii, lda, pU);
		jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nn_8x8_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_8x4_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_8x4_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nn_m1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii, lda, pU, sdu);
		jj = 0;
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nn_8x8_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_m1_left_4;
			}
		else
			{
			goto nn_m1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_4x4_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nn_m1_left_4;
		}
#endif
	goto nn_m1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nn_m1_left_24:
	kernel_spack_nn_24_vs_lib8(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_m1_left_16:
	kernel_spack_nn_16_vs_lib8(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_m1_left_8:
	kernel_spack_nn_8_vs_lib8(k, A+ii, lda, pU, m-ii);
	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-4; jj+=8)
		{
		kernel_sgemm_nn_8x8_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	for(; jj<n; jj+=4)
		{
		kernel_sgemm_nn_8x4_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_m1_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif

nn_m1_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;

nn_m1_return:
	return;

	}



static void blasfeo_hp_sgemm_nn_n1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-23; jj+=24)
		{
		kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		kernel_spack_tn_8_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x24_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto nn_n1_left_8;
			}
		else if(n-jj<=16)
			{
			goto nn_n1_left_16;
			}
		else
			{
			goto nn_n1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-15; jj+=16)
		{
		kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x16_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto nn_n1_left_8;
			}
		else
			{
			goto nn_n1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_nt_8x8_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x8_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nn_n1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		ii = 0;
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_nt_8x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nn_n1_left_4;
			}
		else
			{
			goto nn_n1_left_8;
			}
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x4_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nn_n1_left_4;
		}
#endif
	goto nn_n1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nn_n1_left_24:
	kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu);
	kernel_spack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
	kernel_spack_tn_8_vs_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu, n-jj-16);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_n1_left_16:
	kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu);
	kernel_spack_tn_8_vs_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_n1_left_8:
	kernel_spack_tn_8_vs_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_n1_left_8:
	kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu);
	kernel_spack_tn_4_vs_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu, n-jj-4);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#endif

nn_n1_left_4:
	kernel_spack_tn_4_vs_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;

nn_n1_return:
	return;

	}



static void blasfeo_hp_sgemm_nt_m1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_nn_24_lib8(k, A+ii, lda, pU, sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_24x4_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_m1_left_8;
			}
		else if(m-ii<=16)
			{
			goto nt_m1_left_16;
			}
		else
			{
			goto nt_m1_left_24;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_nn_16_lib8(k, A+ii, lda, pU, sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_16x4_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_m1_left_8;
			}
		else
			{
			goto nt_m1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib8(k, A+ii, lda, pU);
		jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_8x4_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii, lda, pU, sdu);
		jj = 0;
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_m1_left_4;
			}
		else
			{
			goto nt_m1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m1_left_4;
		}
#endif
	goto nt_m1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nt_m1_left_24:
	kernel_spack_nn_24_vs_lib8(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_m1_left_16:
	kernel_spack_nn_16_vs_lib8(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_m1_left_8:
	kernel_spack_nn_8_vs_lib8(k, A+ii, lda, pU, m-ii);
	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	for(; jj<n; jj+=4)
		{
		kernel_sgemm_nt_8x4_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m1_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif

nt_m1_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;

nt_m1_return:
	return;

	}



static void blasfeo_hp_sgemm_nt_n1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-23; jj+=24)
		{
		kernel_spack_nn_24_lib8(k, B+jj, ldb, pU, sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x24_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto nt_n1_left_8;
			}
		else if(n-jj<=16)
			{
			goto nt_n1_left_16;
			}
		else
			{
			goto nt_n1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-15; jj+=16)
		{
		kernel_spack_nn_16_lib8(k, B+jj, ldb, pU, sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x16_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto nt_n1_left_8;
			}
		else
			{
			goto nt_n1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_nn_8_lib8(k, B+jj, ldb, pU);
		ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_nt_8x8_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x8_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nt_n1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_nn_8_lib4(k, B+jj, ldb, pU, sdu);
		ii = 0;
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_nt_8x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nt_n1_left_4;
			}
		else
			{
			goto nt_n1_left_8;
			}
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_spack_nn_4_lib4(k, B+jj, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x4_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nt_n1_left_4;
		}
#endif
	goto nt_n1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nt_n1_left_24:
	kernel_spack_nn_24_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_n1_left_16:
	kernel_spack_nn_16_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_n1_left_8:
	kernel_spack_nn_8_vs_lib8(k, B+jj, ldb, pU, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_n1_left_8:
	kernel_spack_nn_8_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#endif

nt_n1_left_4:
	kernel_spack_nn_4_vs_lib4(k, B+jj, ldb, pU, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;

nt_n1_return:
	return;

	}



static void blasfeo_hp_sgemm_tn_m1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		kernel_spack_tn_8_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_24x4_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tn_m1_left_8;
			}
		if(m-ii<=16)
			{
			goto tn_m1_left_16;
			}
		else
			{
			goto tn_m1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_16x4_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tn_m1_left_8;
			}
		else
			{
			goto tn_m1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nn_8x8_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_8x4_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_8x4_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tn_m1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		jj = 0;
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nn_8x8_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_m1_left_4;
			}
		else
			{
			goto tn_m1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_4x4_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tn_m1_left_4;
		}
#endif
	goto tn_m1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tn_m1_left_24:
	kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu, m-ii-16);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_m1_left_16:
	kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_m1_left_8:
	kernel_spack_tn_8_vs_lib8(k, A+(ii+0)*lda, lda, pU, m-ii);
	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-4; jj+=8)
		{
		kernel_sgemm_nn_8x8_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	for(; jj<n; jj+=4)
		{
		kernel_sgemm_nn_8x4_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_m1_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif

tn_m1_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;

tn_m1_return:
	return;

	}



static void blasfeo_hp_sgemm_tn_n1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-23; jj+=24)
		{
		kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		kernel_spack_tn_8_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x24_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto tn_n1_left_8;
			}
		else if(n-jj<=16)
			{
			goto tn_n1_left_16;
			}
		else
			{
			goto tn_n1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-15; jj+=16)
		{
		kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x16_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto tn_n1_left_8;
			}
		else
			{
			goto tn_n1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_tt_8x8_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x8_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tn_n1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		ii = 0;
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_tt_8x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tn_n1_left_4;
			}
		else
			{
			goto tn_n1_left_8;
			}
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x4_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tn_n1_left_4;
		}
#endif
	goto tn_n1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tn_n1_left_24:
	kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu);
	kernel_spack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
	kernel_spack_tn_8_vs_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu, n-jj-16);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_n1_left_16:
	kernel_spack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu);
	kernel_spack_tn_8_vs_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_n1_left_8:
	kernel_spack_tn_8_vs_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_n1_left_8:
	kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu);
	kernel_spack_tn_4_vs_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu, n-jj-4);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#endif

tn_n1_left_4:
	kernel_spack_tn_4_vs_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;

tn_n1_return:
	return;

	}



static void blasfeo_hp_sgemm_tt_m1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		kernel_spack_tn_8_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_24x4_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tt_m1_left_8;
			}
		else if(m-ii<=16)
			{
			goto tt_m1_left_16;
			}
		else
			{
			goto tt_m1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_16x4_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tt_m1_left_8;
			}
		else
			{
			goto tt_m1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_8x4_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_m1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		jj = 0;
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_m1_left_4;
			}
		else
			{
			goto tt_m1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_m1_left_4;
		}
#endif
	goto tt_m1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tt_m1_left_24:
	kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu, m-ii-16);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_24x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_m1_left_16:
	kernel_spack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_16x4_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_m1_left_8:
	kernel_spack_tn_8_vs_lib8(k, A+(ii+0)*lda, lda, pU, m-ii);
	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	for(; jj<n; jj+=4)
		{
		kernel_sgemm_nt_8x4_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_m1_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif

tt_m1_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;

tt_m1_return:
	return;

	}



static void blasfeo_hp_sgemm_tt_n1(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd, float *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-23; jj+=24)
		{
		kernel_spack_nn_24_lib8(k, B+jj, ldb, pU, sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x24_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto tt_n1_left_8;
			}
		else if(n-jj<=16)
			{
			goto tt_n1_left_16;
			}
		else
			{
			goto tt_n1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-15; jj+=16)
		{
		kernel_spack_nn_16_lib8(k, B+jj, ldb, pU, sdu);
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x16_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto tt_n1_left_8;
			}
		else
			{
			goto tt_n1_left_16;
			}
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_nn_8_lib8(k, B+jj, ldb, pU);
		ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_tt_8x8_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x8_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tt_n1_left_8;
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_nn_8_lib4(k, B+jj, ldb, pU, sdu);
		ii = 0;
		for(; ii<m-7; ii+=8)
			{
			kernel_sgemm_tt_8x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		for(; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tt_n1_left_4;
			}
		else
			{
			goto tt_n1_left_8;
			}
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_spack_nn_4_lib4(k, B+jj, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x4_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tt_n1_left_4;
		}
#endif
	goto tt_n1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tt_n1_left_24:
	kernel_spack_nn_24_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_n1_left_16:
	kernel_spack_nn_16_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_n1_left_8:
	kernel_spack_nn_8_vs_lib8(k, B+jj, ldb, pU, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_n1_left_8:
	kernel_spack_nn_8_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#endif

tt_n1_left_4:
	kernel_spack_nn_4_vs_lib4(k, B+jj, ldb, pU, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;

tt_n1_return:
	return;

	}



//#ifdef HP_BLAS
//
//static void blas_hp_sgemm_nn(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_sgemm_nn %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	float *D = C;
//
//#else

void blasfeo_hp_sgemm_nn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_sgemm_nn (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *C = sC->pA + ci + cj*ldc;
	float *D = sD->pA + di + dj*ldd;

//#endif

//	printf("\n%p %d %p %d %p %d %p %d\n", A, lda, B, ldb, C, ldc, D, ldd);

	int ii, jj, ll, kk;
	int iii;
	int idx;
	int mc, nc, kc, oc;
	int mleft, nleft, kleft, oleft;
	int mc0, nc0, kc0, oc0;
	int ldc1;
	float beta1;
	float *pA, *pB, *C1;


// no global bs, to be able to mix them in different algorithms !!!
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	const int bs = 8;
//#else
//	const int bs = 4;
//#endif
	const int ps = S_PS;
//	const int ps_4 = 4;
//	const int ps_8 = 8;

#if defined(TARGET_GENERIC)
	float pU[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU[M_KERNEL*K_MAX_STACK], 64 );
#endif
	int sdu = (k+3)/4*4;
	sdu = sdu<K_MAX_STACK ? sdu : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	int error;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_HASWELL)
	const int l2_cache_el = L2_CACHE_EL;
#endif
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const int llc_cache_el = LLC_CACHE_EL;
#endif
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	int m_a = m==lda ? m : m_cache;
	int m_a_kernel = m<=m_kernel ? m_a : m_kernel_cache;
	int k_b = k==ldb ? k : k_cache;
	int m_c = m==ldc ? m : m_cache;
	int m_d = m==ldd ? m : m_cache;

//#if defined(PACKING_ALG_0)
//	goto nn_0; // no pack
//#endif
#if defined(PACKING_ALG_M1)
	goto nn_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto nn_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto nn_2; // pack A and B
#endif

// TODO remove when other kernels are implemented !!!!!
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
//	goto nn_2; // pack A and B
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( (m<=m_kernel & n<=m_kernel) | (m_a_kernel*k + k_b*n <= l1_cache_el) )
		{
//		printf("\nalg 2\n");
//		goto nn_0; // small matrix: no pack
		goto nn_m1; // small matrix: pack A
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=48 & n<=48 & k<=K_MAX_STACK )
		{
		goto nn_m1; // small matrix: pack A
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( (m<=m_kernel & n<=m_kernel) )
		{
//		printf("\nalg 2\n");
//		goto nn_0; // small matrix: no pack
		goto nn_m1; // small matrix: pack A
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=m_kernel & n<=m_kernel & k<160) )
		{
//		printf("\nalg 2\n");
//		goto nn_0; // small matrix: no pack
		goto nn_m1; // small matrix: pack A
		}
#else
	if( m<=8 & n<=8 )
		{
//		goto nn_0; // small matrix: no pack
		goto nn_m1; // small matrix: pack A
		}
#endif
#if defined(TARGET_X64_INTEL_HASWELL)
	if( m<n*4 )
		{
//		if( m<=2*m_kernel | m_a*k + k_b*n <= llc_cache_el )
		if( m<=2*m_kernel | ( k<=KC & (m_a*k + k_b*n + m_c*n + m_d*n <= llc_cache_el) ) )
//		if( n<=2*m_kernel | m_a*k <= l2_cache_el )
			{
//			printf("\nalg m0\n");
			goto nn_m1; // long matrix: pack A
			}
		}
	else
		{
		if( n<=2*m_kernel | m_a*k <= l2_cache_el )
			{
//			printf("\nalg n0\n");
			goto nn_n1; // tall matrix: pack B
			}
		}
#else
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=2*m_kernel | n<=2*m_kernel | k<56 )
#elif defined(TARGET_X64_INTEL_CORE)
	if( m<=1*m_kernel | n<=1*m_kernel | k<8 )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( m<=2*m_kernel | n<=2*m_kernel | m_a*k + k_b*n <= llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=2*m_kernel | n<=2*m_kernel) & (k<160) )
#else
	if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
		{
		if( m<=n*4 )
			{
//			printf("\nalg m0\n");
			goto nn_m1; // long matrix: pack A
			}
		else
			{
//			printf("\nalg n0\n");
			goto nn_n1; // tall matrix: pack B
			}
		}
#endif
//	printf("\nalg 1\n");
	goto nn_2; // big matrix: pack A and B

	// never to get here
	return;


nn_m1:

	if(K_MAX_STACK<=0)
		goto nn_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_nn_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_nn_m1(m, n, kleft, alpha, A+ll*lda, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



nn_n1:

	if(K_MAX_STACK<=0)
		goto nn_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_nn_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_nn_n1(m, n, kleft, alpha, A+ll*lda, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



nn_2:


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
	kc = k<kc0 ? k : kc0;

//	tA_size = blasfeo_pm_memsize_dmat(ps, mc, kc);
//	tB_size = blasfeo_pm_memsize_dmat(ps, nc, kc);
	tA_size = blasfeo_pm_memsize_smat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_smat(ps, nc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
#if 1
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+2*4096);
		}
	else
		{
//		printf("\nbuffer\n");
		mem = blasfeo_get_buffer();
		}
#else
//	error = posix_memalign( &mem, 4*1024, tA_size+tB_size+2*4096 );
	error = posix_memalign( &mem, 2*1024*1024, tA_size+tB_size+2*4096 );
	error = madvise( mem, tA_size+tB_size+2*4096, MADV_HUGEPAGE );
#endif
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

//	blasfeo_pm_create_smat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_smat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_smat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

//	double time_pack_A = 0.0;
//	double time_pack_B = 0.0;
//	double time_kernel = 0.0;

//	blasfeo_timer timer;

	pA = tA.pA;
	pB = tB.pA;

//	for(ii=0; ii<m; ii+=mleft)
//		{
//
//		mleft = m-ii<mc ? m-ii : mc;

	for(ll=0; ll<k; ll+=kleft)
		{

#if 1
		if(k-ll<2*kc0)
			{
			if(k-ll<=kc0) // last
				{
				kleft = k-ll;
				}
			else // second last
				{
				kleft = (k-ll+1)/2;
				kleft = (kleft+4-1)/4*4;
				}
			}
		else
			{
			kleft = kc;
			}
#else
		kleft = k-ll<kc ? k-ll : kc;
#endif

		sda = (kleft+4-1)/4*4;
		sdb = (kleft+4-1)/4*4;

		beta1 = ll==0 ? beta : 1.0;
		C1 = ll==0 ? C : D;
		ldc1 = ll==0 ? ldc : ldd;

		for(ii=0; ii<m; ii+=mleft)
			{

			mleft = m-ii<mc ? m-ii : mc;

			// pack A
	//		blasfeo_tic(&timer);
#if 1
			// TODO prefetch
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			for(iii=0; iii<kleft-7; iii+=8)
				{
				kernel_spack_tt_8_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda);
				}
			if(iii<kleft)
				{
				kernel_spack_tt_8_vs_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda, kleft-iii);
				}
#else
			for(iii=0; iii<kleft-3; iii+=4)
				{
				kernel_spack_tt_4_lib4(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda);
				}
			if(iii<kleft)
				{
				kernel_spack_tt_4_vs_lib4(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda, kleft-iii);
				}
#endif
#endif
	//		time_pack_A += blasfeo_toc(&timer);

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack and tran B
	//			blasfeo_tic(&timer);
#if 1
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				// TODO prefetch
				for(iii=0; iii<nleft-7; iii+=8)
					{
					kernel_spack_tn_8_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_spack_tn_8_vs_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb, nleft-iii);
					}
#else
				for(iii=0; iii<nleft-3; iii+=4)
					{
					kernel_spack_tn_4_lib4(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_spack_tn_4_vs_lib4(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb, nleft-iii);
					}
#endif
#endif
	//			time_pack_B += blasfeo_toc(&timer);

	//			blasfeo_tic(&timer);
				blasfeo_hp_sgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
	//			time_kernel += blasfeo_toc(&timer);

				}
			
			}

		}

//	printf("\ntime: pack_A %e, pack_B %e, kernel %e, kernel2 %e, kernel3 %e\n", time_pack_A, time_pack_B, time_kernel, time_kernel2, time_kernel3); 
	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;


#else // no cache blocking


	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_smat(ps, n1, k1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

	pack_B = 1;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_nn_24_lib8(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nn_2_left_8;
			}
		else if(m-ii<=16)
			{
			goto nn_2_left_16;
			}
		else
			{
			goto nn_2_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_nn_16_lib8(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nn_2_left_8;
			}
		else
			{
			goto nn_2_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib8(k, A+ii, lda, tA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x8_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				t
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		goto nn_2_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii, lda, tA.pA, sda);
		jj = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; jj<n-7; jj+=8)
			{
			if(pack_B)
				{
				kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, tB.pA+(jj+0)*sdb);
				kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, tB.pA+(jj+4)*sdb);
				}
			kernel_sgemm_nt_8x8_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_2_left_4;
			}
		else
			{
			goto nn_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_nn_4_lib4(k, A+ii, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		goto nn_2_left_4;
		}
#endif
	goto nn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nn_2_left_24:
	kernel_spack_nn_24_vs_lib8(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_2_left_16:
	kernel_spack_nn_16_vs_lib8(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_2_left_8:
	kernel_spack_nn_8_vs_lib8(k, A+ii, lda, tA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nn_2_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_2_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
		}
	goto nn_2_return;
#endif

nn_2_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_2_return;

nn_2_return:
	free(mem);
	return;


#endif



nn_0:

	ii = 0;
#if 1
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_4x4_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nn_0_left_4;
		}
#endif
	goto nn_0_return;

nn_0_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_0_return;

nn_0_return:
	return;



	// never to get here
	return;

	}



//#ifdef HP_BLAS
//
//static void blas_hp_sgemm_nt(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_sgemm_nt %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	float *D = C;
//
//#else

void blasfeo_hp_sgemm_nt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_sgemm_nt (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *C = sC->pA + ci + cj*ldc;
	float *D = sD->pA + di + dj*ldd;

//#endif

//	printf("\n%p %d %p %d %p %d %p %d\n", A, lda, B, ldb, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	float beta1;
	float *pA, *pB, *C1;

// no global bs, to be able to mix them in different algorithms !!!
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	const int bs = 8;
//#else
//	const int bs = 4;
//#endif
	const int ps = S_PS;
//	const int ps_4 = 4;
//	const int ps_8 = 8;

#if defined(TARGET_GENERIC)
	float pU[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU[M_KERNEL*K_MAX_STACK], 64 );
#endif
	int sdu = (k+3)/4*4;
	sdu = sdu<K_MAX_STACK ? sdu : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_HASWELL)
	const int l2_cache_el = L2_CACHE_EL;
#endif
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const int llc_cache_el = LLC_CACHE_EL;
#endif
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	int m_a = m==lda ? m : m_cache;
	int m_a_kernel = m<=m_kernel ? m_a : m_kernel_cache;
	int n_b = n==ldb ? n : n_cache;

//#if defined(PACKING_ALG_0)
//	goto nt_0; // no pack
//#endif
#if defined(PACKING_ALG_M1)
	goto nt_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto nt_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto nt_2; // pack A and B
#endif

// TODO remove when other kernels are implemented !!!!!
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
//	goto nt_2; // pack A and B
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57) 
	if( (m<=m_kernel & n<=m_kernel) | (m_a_kernel*k + n_b*k <= l1_cache_el) )
		{
//		printf("\nalg 2\n");
//		goto nt_0; // small matrix: no pack
		goto nt_m1; // small matrix: no pack
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=48 & n<=48 & k<=K_MAX_STACK )
		{
		goto nt_m1; // small matrix: pack A
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( (m<=m_kernel & n<=m_kernel) )
		{
//		printf("\nalg 2\n");
//		goto nt_0; // small matrix: no pack
		goto nt_m1; // small matrix: no pack
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=m_kernel & n<=m_kernel & k<160) )
		{
//		printf("\nalg 2\n");
//		goto nt_0; // small matrix: no pack
		goto nt_m1; // small matrix: no pack
		}
#else
	if( m<=8 & n<=8 )
		{
//		goto nt_0; // small matrix: no pack
		goto nt_m1; // small matrix: no pack
		}
#endif
#if defined(TARGET_X64_INTEL_HASWELL)
	if( m<=n )
		{
		if( m<=2*m_kernel | 2 * n_b*k <= l2_cache_el )
			{
//			printf("\nalg m0\n");
			goto nt_m1; // long matrix: pack A
			}
		}
	else
		{
		if( n<=2*m_kernel | 2 * m_a*k <= l2_cache_el )
			{
//			printf("\nalg n0\n");
			goto nt_n1; // tall matrix: pack B
			}
		}
#else
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=2*m_kernel | n<=2*m_kernel | k<56 )
#elif defined(TARGET_X64_INTEL_CORE)
	if( m<=1*m_kernel | n<=1*m_kernel | k<8 )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( m<=2*m_kernel | n<=2*m_kernel | m_a*k + n_b*k <= llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=2*m_kernel | n<=2*m_kernel) & (k<160) )
#else
	if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
		{
		if( m<=n )
			{
//			printf("\nalg m0\n");
			goto nt_m1; // long matrix: pack A
			}
		else
			{
//			printf("\nalg n0\n");
			goto nt_n1; // tall matrix: pack B
			}
		}
#endif
//	printf("\nalg 1\n");
	goto nt_2; // big matrix: pack A and B

	// never to get here
	return;


nt_m1:

	if(K_MAX_STACK<=0)
		goto nt_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_nt_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_nt_m1(m, n, kleft, alpha, A+ll*lda, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



nt_n1:

	if(K_MAX_STACK<=0)
		goto nt_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_nt_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_nt_n1(m, n, kleft, alpha, A+ll*lda, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



nt_2:


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
	kc = k<kc0 ? k : kc0;

//	tA_size = blasfeo_pm_memsize_dmat(ps, mc, kc);
//	tB_size = blasfeo_pm_memsize_dmat(ps, nc, kc);
	tA_size = blasfeo_pm_memsize_smat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_smat(ps, nc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
#if 1
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+2*4096);
		}
	else
		{
//		printf("\nbuffer\n");
		mem = blasfeo_get_buffer();
		}
#else
//	error = posix_memalign( &mem, 4*1024, tA_size+tB_size+2*4096 );
	error = posix_memalign( &mem, 2*1024*1024, tA_size+tB_size+2*4096 );
	error = madvise( mem, tA_size+tB_size+2*4096, MADV_HUGEPAGE );
#endif
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

//	blasfeo_pm_create_smat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_smat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_smat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

//	double time_pack_A = 0.0;
//	double time_pack_B = 0.0;
//	double time_kernel = 0.0;

//	blasfeo_timer timer;

	pA = tA.pA;
	pB = tB.pA;

//	for(ii=0; ii<m; ii+=mleft)
//		{
//
//		mleft = m-ii<mc ? m-ii : mc;

	for(ll=0; ll<k; ll+=kleft)
		{

#if 1
		if(k-ll<2*kc0)
			{
			if(k-ll<=kc0) // last
				{
				kleft = k-ll;
				}
			else // second last
				{
				kleft = (k-ll+1)/2;
				kleft = (kleft+4-1)/4*4;
				}
			}
		else
			{
			kleft = kc;
			}
#else
		kleft = k-ll<kc ? k-ll : kc;
#endif

		sda = (kleft+4-1)/4*4;
		sdb = (kleft+4-1)/4*4;

		beta1 = ll==0 ? beta : 1.0;
		C1 = ll==0 ? C : D;
		ldc1 = ll==0 ? ldc : ldd;

		for(ii=0; ii<m; ii+=mleft)
			{

			mleft = m-ii<mc ? m-ii : mc;

			// pack A
	//		blasfeo_tic(&timer);
#if 1
			// TODO prefetch
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			for(iii=0; iii<kleft-7; iii+=8)
				{
				kernel_spack_tt_8_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda);
				}
			if(iii<kleft)
				{
				kernel_spack_tt_8_vs_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda, kleft-iii);
				}
#else
			for(iii=0; iii<kleft-3; iii+=4)
				{
				kernel_spack_tt_4_lib4(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda);
				}
			if(iii<kleft)
				{
				kernel_spack_tt_4_vs_lib4(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda, kleft-iii);
				}
#endif
#endif
	//		time_pack_A += blasfeo_toc(&timer);

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack and tran B
	//			blasfeo_tic(&timer);
#if 1
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				// TODO prefetch
				for(iii=0; iii<kleft-7; iii+=8)
					{
					kernel_spack_tt_8_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_spack_tt_8_vs_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb, kleft-iii);
					}
#else
				for(iii=0; iii<kleft-3; iii+=4)
					{
					kernel_spack_tt_4_lib4(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_spack_tt_4_vs_lib4(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb, kleft-iii);
					}
#endif
#endif
	//			time_pack_B += blasfeo_toc(&timer);

	//			blasfeo_tic(&timer);
				blasfeo_hp_sgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
	//			time_kernel += blasfeo_toc(&timer);

				}
			
			}

		}

//	printf("\ntime: pack_A %e, pack_B %e, kernel %e, kernel2 %e, kernel3 %e\n", time_pack_A, time_pack_B, time_kernel, time_kernel2, time_kernel3); 
	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;


#else // no cache blocking


	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_smat(ps, n1, k1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(ii=0; ii<k-7; ii+=8)
		{
		kernel_spack_tt_8_lib8(n, B+ii*ldb, ldb, tB.pA+ii*ps, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_8_vs_lib8(n, B+ii*ldb, ldb, tB.pA+ii*ps, sdb, k-ii);
		}
#else
	for(ii=0; ii<k-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(n, B+ii*ldb, ldb, tB.pA+ii*ps, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_4_vs_lib4(n, B+ii*ldb, ldb, tB.pA+ii*ps, sdb, k-ii);
		}
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_nn_24_lib8(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_2_left_8;
			}
		else if(m-ii<=16)
			{
			goto nt_2_left_16;
			}
		else
			{
			goto nt_2_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_nn_16_lib8(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_2_left_8;
			}
		else
			{
			goto nt_2_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib8(k, A+ii, lda, tA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		goto nt_2_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii, lda, tA.pA, sda);
		jj = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_2_left_4;
			}
		else
			{
			goto nt_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_nn_4_lib4(k, A+ii, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_2_left_4;
		}
#endif
	goto nt_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nt_2_left_24:
	kernel_spack_nn_24_vs_lib8(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_2_left_16:
	kernel_spack_nn_16_vs_lib8(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_2_left_8:
	kernel_spack_nn_8_vs_lib8(k, A+ii, lda, tA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_2_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_2_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
		}
	goto nt_2_return;
#endif

nt_2_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_2_return;

nt_2_return:
	free(mem);
	return;



#endif



nt_0:
	ii = 0;
#if 1
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_0_left_4;
		}
#endif
	goto nt_0_return;

nt_0_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_0_return;

nt_0_return:
	return;



	// never to get here
	return;

	}



//#ifdef HP_BLAS
//
//static void blas_hp_sgemm_tn(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_sgemm_tn %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	float *D = C;
//
//#else

void blasfeo_hp_sgemm_tn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_sgemm_tn (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *C = sC->pA + ci + cj*ldc;
	float *D = sD->pA + di + dj*ldd;

//#endif

//	printf("\n%p %d %p %d %p %d %p %d\n", A, lda, B, ldb, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	float beta1;
	float *pA, *pB, *C1;

// no global bs, to be able to mix them in different algorithms !!!
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	const int bs = 8;
//#else
//	const int bs = 4;
//#endif
	const int ps = S_PS;
//	const int ps_4 = 4;
//	const int ps_8 = 8;

#if defined(TARGET_GENERIC)
	float pU[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU[M_KERNEL*K_MAX_STACK], 64 );
#endif
	int sdu = (k+3)/4*4;
	sdu = sdu<K_MAX_STACK ? sdu : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const int llc_cache_el = LLC_CACHE_EL;
#endif
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	int k_a = k==lda ? k : k_cache;
	int k_b = k==ldb ? k : k_cache;
	int m_c = m==ldc ? m : m_cache;
	int m_d = m==ldd ? m : m_cache;

#if defined(PACKING_ALG_M1)
	goto tn_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto tn_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto tn_2; // pack A and B
#endif

// TODO remove when other kernels are implemented !!!!!
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
//	goto tn_2; // pack A and B
#endif

	// no algorithm for small matrix
#if defined(TARGET_X64_INTEL_HASWELL)
	if( m<=2*m_kernel | n<=2*m_kernel | ( k<=KC & (k_a*m + k_b*n + m_c*n + m_d*n <= llc_cache_el) ) )
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=2*m_kernel | n<=2*m_kernel | k<56 )
#elif defined(TARGET_X64_INTEL_CORE)
	if( m<=1*m_kernel | n<=1*m_kernel | k<8 )
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( m<=2*m_kernel | n<=2*m_kernel | k_a*m + k_b*n <= llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=2*m_kernel | n<=2*m_kernel) & (k<160) )
#else
	if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
		{
		if( m<=n )
			{
//			printf("\nalg m0\n");
			goto tn_m1; // long matrix: pack A
			}
		else
			{
//			printf("\nalg n0\n");
			goto tn_n1; // tall matrix: pack B
			}
		}
//	printf("\nalg 1\n");
	goto tn_2; // big matrix: pack A and B

	// never to get here
	return;


tn_m1:

	if(K_MAX_STACK<=0)
		goto tn_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_tn_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_tn_m1(m, n, kleft, alpha, A+ll, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



tn_n1:

	if(K_MAX_STACK<=0)
		goto tn_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_tn_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_tn_n1(m, n, kleft, alpha, A+ll, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



tn_2:


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
	kc = k<kc0 ? k : kc0;

//	tA_size = blasfeo_pm_memsize_dmat(ps, mc, kc);
//	tB_size = blasfeo_pm_memsize_dmat(ps, nc, kc);
	tA_size = blasfeo_pm_memsize_smat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_smat(ps, nc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
#if 1
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+2*4096);
		}
	else
		{
//		printf("\nbuffer\n");
		mem = blasfeo_get_buffer();
		}
#else
//	error = posix_memalign( &mem, 4*1024, tA_size+tB_size+2*4096 );
	error = posix_memalign( &mem, 2*1024*1024, tA_size+tB_size+2*4096 );
	error = madvise( mem, tA_size+tB_size+2*4096, MADV_HUGEPAGE );
#endif
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

//	blasfeo_pm_create_smat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_smat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_smat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

//	double time_pack_A = 0.0;
//	double time_pack_B = 0.0;
//	double time_kernel = 0.0;

//	blasfeo_timer timer;

	pA = tA.pA;
	pB = tB.pA;

//	for(ii=0; ii<m; ii+=mleft)
//		{
//
//		mleft = m-ii<mc ? m-ii : mc;

	for(ll=0; ll<k; ll+=kleft)
		{

#if 1
		if(k-ll<2*kc0)
			{
			if(k-ll<=kc0) // last
				{
				kleft = k-ll;
				}
			else // second last
				{
				kleft = (k-ll+1)/2;
				kleft = (kleft+4-1)/4*4;
				}
			}
		else
			{
			kleft = kc;
			}
#else
		kleft = k-ll<kc ? k-ll : kc;
#endif

		sda = (kleft+4-1)/4*4;
		sdb = (kleft+4-1)/4*4;

		beta1 = ll==0 ? beta : 1.0;
		C1 = ll==0 ? C : D;
		ldc1 = ll==0 ? ldc : ldd;

		for(ii=0; ii<m; ii+=mleft)
			{

			mleft = m-ii<mc ? m-ii : mc;

			// pack A
	//		blasfeo_tic(&timer);
#if 1
			// TODO prefetch
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			for(iii=0; iii<mleft-7; iii+=8)
				{
				kernel_spack_tn_8_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda);
				}
			if(iii<mleft)
				{
				kernel_spack_tn_8_vs_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda, mleft-iii);
				}
#else
			for(iii=0; iii<mleft-3; iii+=4)
				{
				kernel_spack_tn_4_lib4(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda);
				}
			if(iii<mleft)
				{
				kernel_spack_tn_4_vs_lib4(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda, mleft-iii);
				}
#endif
#endif
	//		time_pack_A += blasfeo_toc(&timer);

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack and tran B
	//			blasfeo_tic(&timer);
#if 1
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				// TODO prefetch
				for(iii=0; iii<nleft-7; iii+=8)
					{
					kernel_spack_tn_8_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_spack_tn_8_vs_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb, nleft-iii);
					}
#else
				for(iii=0; iii<nleft-3; iii+=4)
					{
					kernel_spack_tn_4_lib4(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_spack_tn_4_vs_lib4(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb, nleft-iii);
					}
#endif
#endif
	//			time_pack_B += blasfeo_toc(&timer);

	//			blasfeo_tic(&timer);
				blasfeo_hp_sgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
	//			time_kernel += blasfeo_toc(&timer);

				}
			
			}

		}

//	printf("\ntime: pack_A %e, pack_B %e, kernel %e, kernel2 %e, kernel3 %e\n", time_pack_A, time_pack_B, time_kernel, time_kernel2, time_kernel3); 
	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;


#else // no cache blocking


	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_smat(ps, n1, k1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

	pack_B = 1;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps);
		kernel_spack_tn_8_lib8(k, A+(ii+16)*lda, lda, tA.pA+2*sda*ps);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tn_2_left_8;
			}
		else if(m-ii<=16)
			{
			goto tn_2_left_16;
			}
		else
			{
			goto tn_2_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tn_2_left_8;
			}
		else
			{
			goto tn_2_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		goto tn_2_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
		jj = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; jj<n-7; jj+=8)
			{
			if(pack_B)
				{
				kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, tB.pA+(jj+0)*sdb);
				kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, tB.pA+(jj+4)*sdb);
				}
			kernel_sgemm_nt_8x8_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_2_left_4;
			}
		else
			{
			goto tn_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
		if(ii<m)
		{
		goto tn_2_left_4;
		}
#endif
	goto tn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tn_2_left_24:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
	kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, tA.pA+2*sda*ps, m-ii-16);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_2_left_16:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps, m-ii-8);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_2_left_8:
	kernel_spack_tn_8_vs_lib8(k, A+ii*lda, lda, tA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tn_2_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_2_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
		}
	goto tn_2_return;
#endif

tn_2_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+(ii+0)*lda, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_2_return;

tn_2_return:
free(mem);
	return;



#endif



	// never to get here
	return;

	}



//#ifdef HP_BLAS
//
//static void blas_hp_sgemm_tt(int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_sgemm_tt %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	float *D = C;
//
//#else

void blasfeo_hp_sgemm_tt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_sgemm_tt (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *C = sC->pA + ci + cj*ldc;
	float *D = sD->pA + di + dj*ldd;

//#endif

//	printf("\n%p %d %p %d %p %d %p %d\n", A, lda, B, ldb, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	float beta1;
	float *pA, *pB, *C1;

// no global bs, to be able to mix them in different algorithms !!!
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	const int bs = 8;
//#else
//	const int bs = 4;
//#endif
	const int ps = S_PS;
//	const int ps_4 = 4;
//	const int ps_8 = 8;

#if defined(TARGET_GENERIC)
	float pU[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU[M_KERNEL*K_MAX_STACK], 64 );
#endif
	int sdu = (k+3)/4*4;
	sdu = sdu<K_MAX_STACK ? sdu : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_HASWELL)
	const int l2_cache_el = L2_CACHE_EL;
#endif
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const int llc_cache_el = LLC_CACHE_EL;
#endif
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	int k_a = k==lda ? k : k_cache;
	int n_b = n==ldb ? n : n_cache;
	int n_b_kernel = n<=m_kernel ? n_b : m_kernel_cache;
	int m_c = m==ldc ? m : m_cache;
	int m_d = m==ldd ? m : m_cache;

//#if defined(PACKING_ALG_0)
//	goto tt_0; // no pack
//#endif
#if defined(PACKING_ALG_M1)
	goto tt_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto tt_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto tt_2; // pack A and B
#endif

// TODO remove when other kernels are implemented !!!!!
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
//	goto tt_2; // pack A and B
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( (m<=m_kernel & n<=m_kernel) | (k_a*m + n_b_kernel*k <= l1_cache_el) )
		{
//		goto tt_0; // small matrix: no pack
		goto tt_m1; // small matrix: no pack
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=48 & n<=48 & k<=K_MAX_STACK )
		{
		goto tt_m1; // small matrix: pack A
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( (m<=m_kernel & n<=m_kernel) )
		{
//		printf("\nalg 2\n");
//		goto tt_0; // small matrix: no pack
		goto tt_m1; // small matrix: no pack
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=m_kernel & n<=m_kernel & k<160) )
		{
//		printf("\nalg 2\n");
//		goto tt_0; // small matrix: no pack
		goto tt_m1; // small matrix: no pack
		}
#else
	if( m<=8 & n<=8 )
		{
		goto tt_m1; // small matrix: no pack
		}
#endif
#if defined(TARGET_X64_INTEL_HASWELL)
	if( m*4<=n | k<=4 ) // XXX k too !!!
		{
		if( m<=2*m_kernel | 2 * n_b*k <= l2_cache_el )
			{
//				printf("\nalg m0\n");
			goto tt_m1; // long matrix: pack A
			}
		}
	else
		{
//		if( n<=2*m_kernel | ( k<=KC & (k_a*m + n_b*k + m_c*n + m_d*n <= llc_cache_el) ) )
		if( m<=2*m_kernel | 2 * n_b*k <= l2_cache_el )
			{
//				printf("\nalg n0\n");
			goto tt_n1; // tall matrix: pack B
			}
		}
#else
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=2*m_kernel | n<=2*m_kernel | k<56 )
#elif defined(TARGET_X64_INTEL_CORE)
	if( m<=1*m_kernel | n<=1*m_kernel | k<8 )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( m<=2*m_kernel | n<=2*m_kernel | k_a*m + n_b*k <= llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=2*m_kernel | n<=2*m_kernel) & (k<160) )
#else
	if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
		{
		if( m*4<=n | k<=4 ) // XXX k too !!!
			{
			goto tt_m1; // long matrix: pack A
			}
		else
			{
			goto tt_n1; // tall matrix: pack B
			}
		}
#endif
	goto tt_2; // big matrix: pack A and B

	// never to get here
	return;


tt_m1:

	if(K_MAX_STACK<=0)
		goto tt_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_tt_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_tt_m1(m, n, kleft, alpha, A+ll, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



tt_n1:

	if(K_MAX_STACK<=0)
		goto tt_2;

	// k-blocking alg

	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;

	if(k<kc)
		{
		blasfeo_hp_sgemm_tt_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
			kleft = k-ll<kc ? k-ll : kc;

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_sgemm_tt_n1(m, n, kleft, alpha, A+ll, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	return;



tt_2:


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
	kc = k<kc0 ? k : kc0;

//	tA_size = blasfeo_pm_memsize_dmat(ps, mc, kc);
//	tB_size = blasfeo_pm_memsize_dmat(ps, nc, kc);
	tA_size = blasfeo_pm_memsize_smat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_smat(ps, nc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
#if 1
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+2*4096);
		}
	else
		{
//		printf("\nbuffer\n");
		mem = blasfeo_get_buffer();
		}
#else
//	error = posix_memalign( &mem, 4*1024, tA_size+tB_size+2*4096 );
	error = posix_memalign( &mem, 2*1024*1024, tA_size+tB_size+2*4096 );
	error = madvise( mem, tA_size+tB_size+2*4096, MADV_HUGEPAGE );
#endif
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

//	blasfeo_pm_create_smat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_smat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_smat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

//	double time_pack_A = 0.0;
//	double time_pack_B = 0.0;
//	double time_kernel = 0.0;

//	blasfeo_timer timer;

	pA = tA.pA;
	pB = tB.pA;

//	for(ii=0; ii<m; ii+=mleft)
//		{
//
//		mleft = m-ii<mc ? m-ii : mc;

	for(ll=0; ll<k; ll+=kleft)
		{

#if 1
		if(k-ll<2*kc0)
			{
			if(k-ll<=kc0) // last
				{
				kleft = k-ll;
				}
			else // second last
				{
				kleft = (k-ll+1)/2;
				kleft = (kleft+4-1)/4*4;
				}
			}
		else
			{
			kleft = kc;
			}
#else
		kleft = k-ll<kc ? k-ll : kc;
#endif

		sda = (kleft+4-1)/4*4;
		sdb = (kleft+4-1)/4*4;

		beta1 = ll==0 ? beta : 1.0;
		C1 = ll==0 ? C : D;
		ldc1 = ll==0 ? ldc : ldd;

		for(ii=0; ii<m; ii+=mleft)
			{

			mleft = m-ii<mc ? m-ii : mc;

			// pack A
	//		blasfeo_tic(&timer);
#if 1
			// TODO prefetch
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			for(iii=0; iii<mleft-7; iii+=8)
				{
				kernel_spack_tn_8_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda);
				}
			if(iii<mleft)
				{
				kernel_spack_tn_8_vs_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda, mleft-iii);
				}
#else
			for(iii=0; iii<mleft-3; iii+=4)
				{
				kernel_spack_tn_4_lib4(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda);
				}
			if(iii<mleft)
				{
				kernel_spack_tn_4_vs_lib4(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda, mleft-iii);
				}
#endif
#endif
	//		time_pack_A += blasfeo_toc(&timer);

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack and tran B
	//			blasfeo_tic(&timer);
#if 1
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				// TODO prefetch
				for(iii=0; iii<kleft-7; iii+=8)
					{
					kernel_spack_tt_8_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_spack_tt_8_vs_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb, kleft-iii);
					}
#else
				for(iii=0; iii<kleft-3; iii+=4)
					{
					kernel_spack_tt_4_lib4(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_spack_tt_4_vs_lib4(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb, kleft-iii);
					}
#endif
#endif
	//			time_pack_B += blasfeo_toc(&timer);

	//			blasfeo_tic(&timer);
				blasfeo_hp_sgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
	//			time_kernel += blasfeo_toc(&timer);

				}
			
			}

		}

//	printf("\ntime: pack_A %e, pack_B %e, kernel %e, kernel2 %e, kernel3 %e\n", time_pack_A, time_pack_B, time_kernel, time_kernel2, time_kernel3); 
	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;


#else // no cache blocking


	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_smat(ps, n1, k1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(ii=0; ii<k-7; ii+=8)
		{
		kernel_spack_tt_8_lib8(n, B+ii*ldb, ldb, tB.pA+ii*ps, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_8_vs_lib8(n, B+ii*ldb, ldb, tB.pA+ii*ps, sdb, k-ii);
		}
#else
	for(ii=0; ii<k-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(n, B+ii*ldb, ldb, tB.pA+ii*4, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_4_vs_lib4(n, B+ii*ldb, ldb, tB.pA+ii*4, sdb, k-ii);
		}
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps);
		kernel_spack_tn_8_lib8(k, A+(ii+16)*lda, lda, tA.pA+2*sda*ps);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tt_2_left_8;
			}
		else if(m-ii<=16)
			{
			goto tt_2_left_16;
			}
		else
			{
			goto tt_2_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tt_2_left_8;
			}
		else
			{
			goto tt_2_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		goto tt_2_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
		jj = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
#endif
		for(; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_2_left_4;
			}
		else
			{
			goto tt_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_2_left_4;
		}
#endif
	goto tt_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tt_2_left_24:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
	kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, tA.pA+2*sda*ps, m-ii-16);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tt_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_2_left_16:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, tA.pA);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, tA.pA+sda*ps, m-ii-8);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tt_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_2_left_8:
	kernel_spack_tn_8_vs_lib8(k, A+ii*lda, lda, tA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, tA.pA, tB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tt_2_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_2_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA+4*sda, tB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
#endif
		}
	goto tt_2_return;
#endif

tt_2_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+(ii+0)*lda, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_2_return;

tt_2_return:
	free(mem);
	return;



#endif



tt_0:

	jj = 0;
#if 1
	for(; jj<n-3; jj+=4)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x4_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tt_0_left_4;
		}
#endif
	goto tt_0_return;

tt_0_left_4:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_0_return;

tt_0_return:
	return;



	// never to get here
	return;

	}



#if defined(LA_HIGH_PERFORMANCE)
//#ifndef HP_BLAS



void blasfeo_sgemm_nn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_nn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgemm_nt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_nt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgemm_tn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_tn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgemm_tt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_tt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



//#endif
#endif

