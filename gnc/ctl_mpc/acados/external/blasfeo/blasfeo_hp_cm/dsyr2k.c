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

//#define PRINT_NAME

#include <blasfeo_target.h>
#include <blasfeo_block_size.h>
#include <blasfeo_common.h>
#include <blasfeo_stdlib.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#include <blasfeo_memory.h>
#include <blasfeo_d_blasfeo_api.h>

#include <blasfeo_timing.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_hp_dsyr2k_ln blasfeo_hp_cm_dsyr2k_ln
#define blasfeo_hp_dsyr2k_lt blasfeo_hp_cm_dsyr2k_lt
#define blasfeo_hp_dsyr2k_un blasfeo_hp_cm_dsyr2k_un
#define blasfeo_hp_dsyr2k_ut blasfeo_hp_cm_dsyr2k_ut
#define blasfeo_dsyr2k_ln blasfeo_cm_dsyr2k_ln
#define blasfeo_dsyr2k_lt blasfeo_cm_dsyr2k_lt
#define blasfeo_dsyr2k_un blasfeo_cm_dsyr2k_un
#define blasfeo_dsyr2k_ut blasfeo_cm_dsyr2k_ut

#define blasfeo_dsyrk_ln blasfeo_cm_dsyrk_ln
#define blasfeo_dsyrk_lt blasfeo_cm_dsyrk_lt
#define blasfeo_dsyrk_un blasfeo_cm_dsyrk_un
#define blasfeo_dsyrk_ut blasfeo_cm_dsyrk_ut

void blasfeo_cm_dsyrk_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj);
void blasfeo_cm_dsyrk_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj);
void blasfeo_cm_dsyrk_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj);
void blasfeo_cm_dsyrk_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj);
#endif



#define CACHE_LINE_EL D_CACHE_LINE_EL
#define L1_CACHE_EL D_L1_CACHE_EL
#define L2_CACHE_EL D_L2_CACHE_EL
#define LLC_CACHE_EL D_LLC_CACHE_EL
#define PS D_PS
#define M_KERNEL D_M_KERNEL
#define KC D_KC
#define NC D_NC
#define MC D_MC



static void blasfeo_hp_dger2k_nt_m2(int m, int n, int k, double alpha, double *pA0, int sda0, double *pB0, int sdb0, double *pA1, int sda1, double *pB1, int sdb1, double beta, double *C, int ldc, double *D, int ldd)
	{

	int ii, jj;

	// TODO SKYLAKE_X target since it uses ps=8 !!!!!

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dger2k_nt_12x4_lib44cc(k, &alpha, pA0+ii*sda0, sda0, pB0+jj*sdb0, pA1+ii*sda1, sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dger2k_nt_12x4_vs_lib44cc(k, &alpha, pA0+ii*sda0, sda0, pB0+jj*sdb0, pA1+ii*sda1, sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_m2_left_4;
			}
		if(m-ii<=8)
			{
			goto nt_m2_left_8;
			}
		else
			{
			goto nt_m2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dger2k_nt_8x4_lib44cc(k, &alpha, pA0+ii*sda0, sda0, pB0+jj*sdb0, pA1+ii*sda1, sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dger2k_nt_8x4_vs_lib44cc(k, &alpha, pA0+ii*sda0, sda0, pB0+jj*sdb0, pA1+ii*sda1, sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
			kernel_dger2k_nt_4x4_lib44cc(k, &alpha, pA0+ii*sda0, pB0+jj*sdb0, pA1+ii*sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dger2k_nt_4x4_vs_lib44cc(k, &alpha, pA0+ii*sda0, pB0+jj*sdb0, pA1+ii*sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m2_left_4;
		}
#endif
	goto nt_m2_return;

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dger2k_nt_12x4_vs_lib44cc(k, &alpha, pA0+ii*sda0, sda0, pB0+jj*sdb0, pA1+ii*sda1, sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dger2k_nt_8x4_vs_lib44cc(k, &alpha, pA0+ii*sda0, sda0, pB0+jj*sdb0, pA1+ii*sda1, sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;
#endif

nt_m2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dger2k_nt_4x4_vs_lib44cc(k, &alpha, pA0+ii*sda0, pB0+jj*sdb0, pA1+ii*sda1, pB1+jj*sdb1, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;

nt_m2_return:
	return;

	return;

	}



static void blasfeo_hp_dsyr2k_ln_m2(int m, int k, double alpha, double *pA, int sda, double *pB, int sdb, double beta, double *C, int ldc, double *D, int ldd)
	{

	int ii, jj;

	// TODO SKYLAKE_X target since it uses ps=8 !!!!!

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dger2k_nt_12x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyr2k_nt_l_12x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyr2k_nt_l_8x8_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pB+(jj+4)*sdb, sdb, pB+(ii+4)*sdb, sdb, pA+(jj+4)*sda, sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
#else
		kernel_dsyr2k_nt_l_8x4_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pB+(jj+4)*sdb, pB+(ii+4)*sdb, sdb, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		kernel_dsyr2k_nt_l_4x4_lib44cc(k, &alpha, pA+(ii+8)*sda, pB+(jj+8)*sdb, pB+(ii+8)*sdb, pA+(jj+8)*sda, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
#endif
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ln_m2_left_4;
			}
		if(m-ii<=8)
			{
			goto ln_m2_left_8;
			}
		else
			{
			goto ln_m2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dger2k_nt_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyr2k_nt_l_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyr2k_nt_l_4x4_lib44cc(k, &alpha, pA+(ii+4)*sda, pB+(jj+4)*sdb, pB+(ii+4)*sdb, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ln_m2_left_4;
			}
		else
			{
			goto ln_m2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dger2k_nt_4x4_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, pB+ii*sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyr2k_nt_l_4x4_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, pB+ii*sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		}
	if(ii<m)
		{
		goto ln_m2_left_4;
		}
#endif
	goto ln_m2_return;

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_m2_left_12:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dger2k_nt_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyr2k_nt_l_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyr2k_nt_l_8x8_vs_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pB+(jj+4)*sdb, sdb, pB+(ii+4)*sdb, sdb, pA+(jj+4)*sda, sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#else
	kernel_dsyr2k_nt_l_8x4_vs_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pB+(jj+4)*sdb, pB+(ii+4)*sdb, sdb, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	kernel_dsyr2k_nt_l_4x4_vs_lib44cc(k, &alpha, pA+(ii+8)*sda, pB+(jj+8)*sdb, pB+(ii+8)*sdb, pA+(jj+8)*sda, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
#endif
	goto ln_m2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_m2_left_8:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dger2k_nt_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyr2k_nt_l_8x8_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, sdb, pB+ii*sdb, sdb, pA+jj*sda, sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyr2k_nt_l_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, pB+ii*sdb, sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyr2k_nt_l_4x4_vs_lib44cc(k, &alpha, pA+(ii+4)*sda, pB+(jj+4)*sdb, pB+(ii+4)*sdb, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto ln_m2_return;
#endif

ln_m2_left_4:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dger2k_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, pB+ii*sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyr2k_nt_l_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, pB+ii*sdb, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto ln_m2_return;

ln_m2_return:
	return;

	}



static void blasfeo_hp_dsyr2k_ln_m1(int m, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, pU, sdu);
		kernel_dpack_nn_12_lib4(k, B+ii, ldb, pU+k*ps, sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dger2k_nt_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, pU+k*ps, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyr2k_nt_l_12x4_lib44cc(k, &alpha, pU, sdu, pU+k*ps, pU+k*ps, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyr2k_nt_l_8x8_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu+k*ps, sdu, pU+4*sdu+k*ps, sdu, pU+4*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
//		kernel_dsyr2k_nt_l_8x4_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu+k*ps, pU+4*sdu+k*ps, sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
//		kernel_dsyr2k_nt_l_4x4_lib44cc(k, &alpha, pU+8*sdu, pU+8*sdu+k*ps, pU+8*sdu+k*ps, pU+8*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ln_1_left_4;
			}
		if(m-ii<=8)
			{
			goto ln_1_left_8;
			}
		else
			{
			goto ln_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, pU, sdu);
		kernel_dpack_nn_8_lib4(k, B+ii, ldb, pU+k*ps, sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dger2k_nt_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, pU+k*ps, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyr2k_nt_l_8x4_lib44cc(k, &alpha, pU, sdu, pU+k*ps, pU+k*ps, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyr2k_nt_l_4x4_lib44cc(k, &alpha, pU+4*sdu, pU+4*sdu+k*ps, pU+4*sdu+k*ps, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ln_1_left_4;
			}
		else
			{
			goto ln_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		kernel_dpack_nn_4_lib4(k, B+ii, ldb, pU+k*ps);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dger2k_nt_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, pU+k*ps, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyr2k_nt_l_4x4_lib44cc(k, &alpha, pU, pU+k*ps, pU+k*ps, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		}
	if(ii<m)
		{
		goto ln_1_left_4;
		}
#endif
	goto ln_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
ln_1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	kernel_dpack_nn_12_vs_lib4(k, B+ii, ldb, pU+k*ps, sdu, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dger2k_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, pU+k*ps, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyr2k_nt_l_12x4_vs_lib44cc(k, &alpha, pU, sdu, pU+k*ps, pU+k*ps, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyr2k_nt_l_8x8_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu+k*ps, sdu, pU+4*sdu+k*ps, sdu, pU+4*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
//	kernel_dsyr2k_nt_l_8x4_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu+k*ps, pU+4*sdu+k*ps, sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
//	kernel_dsyr2k_nt_l_4x4_vs_lib44cc(k, &alpha, pU+8*sdu, pU+8*sdu+k*ps, pU+8*sdu+k*ps, pU+8*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
	goto ln_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	kernel_dpack_nn_8_vs_lib4(k, B+ii, ldb, pU+k*ps, sdu, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dger2k_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, pU+k*ps, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyr2k_nt_l_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU+k*ps, sdu, pU+k*ps, sdu, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyr2k_nt_l_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+k*ps, pU+k*ps, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyr2k_nt_l_4x4_vs_lib44cc(k, &alpha, pU+4*sdu, pU+4*sdu+k*ps, pU+4*sdu+k*ps, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto ln_1_return;
#endif

ln_1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	kernel_dpack_nn_4_vs_lib4(k, B+ii, ldb, pU+k*ps, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dger2k_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, pU+k*ps, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyr2k_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pU+k*ps, pU+k*ps, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto ln_1_return;

ln_1_return:
	return;

	}



// dsyr2k lower not-transposed
void blasfeo_hp_dsyr2k_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

//	double dd_1 = 1.0;
//	blasfeo_dsyrk_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
//	blasfeo_dsyrk_ln(m, k, alpha, sB, bi, bj, sA, ai, aj, dd_1, sD, di, dj, sD, di, dj);
//	return;

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dsyr2k_ln (cm) %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc, kcd2;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *C1;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

#if defined(TARGET_GENERIC)
	double pU_stack[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU_stack[M_KERNEL*K_MAX_STACK], 64 );
#endif
	int sdu_stack = K_MAX_STACK;
	int k4 = (k+3)/4*4;

	double *pU;
	int sdu;
	int pU_size;

	struct blasfeo_pm_dmat tA, tB;
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
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const int llc_cache_el = LLC_CACHE_EL;
#endif
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (2*k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	int m_a = m==lda ? m : m_cache;
//	int n_b = n==ldb ? n : n_cache;
	int n_b = m_a; // syrk: B=A
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = 2*k<=k_block ? 2*k : k_block; // m1 and n1 alg are blocked !!!

	double d_1 = 1.0;


//	goto ln_1;
//	goto ln_2;
#if defined(TARGET_X64_INTEL_HASWELL)
//	if(m<200 & k<200)
	if( m<=2*m_kernel | n_b*k_block <= l2_cache_el )
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & 2*k<64)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
//	if(m<32 & 2*k<32)
	if( m<=2*m_kernel | 2*m*k_block < llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<16 & 2*k<16)
#else
	if(m<12 & 2*k<12)
#endif
		{
//		printf("\nalg 1 %d\n", m);
		goto ln_1;
		}
	else
		{
//		printf("\nalg 2 %d\n", m);
		goto ln_2;
		}

	// never to get here
	return;



ln_1:
	// k-blocking alg

	if(2*k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		pU_size = M_KERNEL*KC*sizeof(double);
		blasfeo_malloc(&mem, pU_size+64);
		blasfeo_align_64_byte(mem, (void **) &mem_align);
		pU = (double *) mem_align;
		sdu = KC;
		}
	else
		{
		pU = pU_stack;
		sdu = sdu_stack;
		}
	
	k4 = (2*k+3)/4*4;
	sdu = k4<sdu ? k4 : sdu;

//	kc = 4;
	kc = KC;

	kcd2 = kc/2;

	if(k<kcd2)
		{
		blasfeo_hp_dsyr2k_ln_m1(m, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
#if 0
			if(k-ll<kc)
				{
				if(k-ll<=kc/2) // last
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
				kleft = kcd2;
				}
#else
			kleft = k-ll<kcd2 ? k-ll : kcd2;
#endif

			sdu = (2*kleft+4-1)/4*4; // XXX 2*

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_dsyr2k_ln_m1(m, kleft, alpha, A+ll*lda, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}

		}

	if(2*k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



ln_2:
#if ! defined(TARGET_X64_INTEL_SKYLAKE_X)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

#if defined(DIM_CHECK)
	if(kc0%2!=0)
		{
		printf("\nblasfeo_dsyr2k_ln: kc%2!=0\n");
		}
#endif

	mc = m<mc0 ? m : mc0;
	nc = m<nc0 ? m : nc0; // XXX just one buffer if m small enough ???
//	kc = k<kc0 ? k : kc0;
	kc = 2*k<kc0 ? 2*k : kc0;

	kcd2 = kc/2;

	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+2*4096);
		}
	else
		{
		mem = blasfeo_get_buffer();
		}
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	pA = tA.pA;
	pB = tB.pA;

	for(ll=0; ll<k; ll+=kleft)
		{

#if 1
		if(k-ll<kc0)
			{
			if(k-ll<=kc0/2) // last
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
			kleft = kcd2;
			}
#else
		kleft = k-ll<kcd2 ? k-ll : kcd2;
#endif

		sda = (2*kleft+4-1)/4*4; // XXX 2*
		sdb = (2*kleft+4-1)/4*4; // XXX 2*

		beta1 = ll==0 ? beta : 1.0;
		C1 = ll==0 ? C : D;
		ldc1 = ll==0 ? ldc : ldd;

		for(ii=0; ii<m; ii+=mleft)
			{

			mleft = m-ii<mc ? m-ii : mc;

			// pack A
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
			for(iii=0; iii<kleft-3; iii+=4)
				{
				kernel_dpack_tt_4_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda);
				kernel_dpack_tt_4_lib8(mleft, B+ii+(ll+iii)*ldb, ldb, pA+iii*ps+kleft*ps, sda);
				}
			if(iii<kleft)
				{
				kernel_dpack_tt_4_vs_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda, kleft-iii);
				kernel_dpack_tt_4_vs_lib8(mleft, B+ii+(ll+iii)*ldb, ldb, pA+iii*ps+kleft*ps, sda, kleft-iii);
				}
#else
			kernel_dpack_buffer_fn(mleft, kleft, A+ii+ll*lda, lda, pA, sda);
			kernel_dpack_buffer_fn(mleft, kleft, B+ii+ll*ldb, ldb, pA+kleft*ps, sda);
#endif

			// dgemm
			for(jj=0; jj<ii; jj+=nleft)
				{

				nleft = ii-jj<nc ? ii-jj : nc;

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<kleft-3; iii+=4)
					{
					kernel_dpack_tt_4_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb);
					kernel_dpack_tt_4_lib8(nleft, A+jj+(ll+iii)*lda, lda, pB+iii*ps+kleft*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_dpack_tt_4_vs_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb, kleft-iii);
					kernel_dpack_tt_4_vs_lib8(nleft, A+jj+(ll+iii)*lda, lda, pB+iii*ps+kleft*ps, sdb, kleft-iii);
					}
#else
				kernel_dpack_buffer_fn(nleft, kleft, B+jj+ll*ldb, ldb, pB, sdb);
				kernel_dpack_buffer_fn(nleft, kleft, A+jj+ll*lda, lda, pB+kleft*ps, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, 2*kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);

				}

			// dsyrk
#if 1
			for(iii=0; iii<mleft; iii+=nleft)
				{
				nleft = mleft-iii<nc ? mleft-iii : nc;
//				blasfeo_pm_print_dmat(nleft, 2*kleft, &tA, 0, 0);
				blasfeo_hp_dsyr2k_ln_m2(nleft, kleft, alpha, pA+iii*sda, sda, pA+iii*sda+kleft*ps, sda, beta1, C1+(ii+iii)+(ii+iii)*ldc1, ldc1, D+(ii+iii)+(ii+iii)*ldd, ldd);
//				blasfeo_hp_dgemm_nt_m2(mleft-iii-nleft, nleft, kleft, alpha, pA+(iii+nleft)*sda, sda, pA+iii*sda+kleft*ps, sda, beta1, C1+(ii+iii+nleft)+(ii+iii)*ldc1, ldc1, D+(ii+iii+nleft)+(ii+iii)*ldd, ldd);
//				blasfeo_hp_dgemm_nt_m2(mleft-iii-nleft, nleft, kleft, alpha, pA+(iii+nleft)*sda+kleft*ps, sda, pA+iii*sda, sda, d_1, D+(ii+iii+nleft)+(ii+iii)*ldd, ldd, D+(ii+iii+nleft)+(ii+iii)*ldd, ldd);
				blasfeo_hp_dger2k_nt_m2(mleft-iii-nleft, nleft, kleft, alpha, pA+(iii+nleft)*sda, sda, pA+iii*sda+kleft*ps, sda, pA+(iii+nleft)*sda+kleft*ps, sda, pA+iii*sda, sda, beta1, C1+(ii+iii+nleft)+(ii+iii)*ldc1, ldc1, D+(ii+iii+nleft)+(ii+iii)*ldd, ldd);
				}
#else
			blasfeo_hp_dsyr2k_ln_m2(mleft, kleft, alpha, pA, sda, pA+kleft*ps, sda, beta1, C1+ii+ii*ldc1, ldc1, D+ii+ii*ldd, ldd);
#endif

			}

		}

	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;

#else

	k1 = (2*k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m1, k1);
	mem = malloc(tA_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m, 2*k, &tA, (void *) mem_align);

	pU = tA.pA;
	sdu = tA.cn;

	// pack A
	kernel_dpack_buffer_fn(m, k, A, lda, pU, sdu);
	kernel_dpack_buffer_fn(m, k, B, ldb, pU+k*ps, sdu);

//	blasfeo_print_dmat(m, k, &tA, 0, 0);

	blasfeo_hp_dsyr2k_ln_m2(m, k, alpha, pU, sdu, pU+k*ps, sdu, beta, C, ldc, D, ldd);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}






// dsyr2k lower transposed
void blasfeo_hp_dsyr2k_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	double d_1 = 1.0;
	blasfeo_dsyrk_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_dsyrk_lt(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



// dsyr2k upper not-transposed
void blasfeo_hp_dsyr2k_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	double d_1 = 1.0;
	blasfeo_dsyrk_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_dsyrk_un(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



// dsyr2k upper transposed
void blasfeo_hp_dsyr2k_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	double d_1 = 1.0;
	blasfeo_dsyrk_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_dsyrk_ut(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dsyr2k_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyr2k_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyr2k_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyr2k_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



#endif

