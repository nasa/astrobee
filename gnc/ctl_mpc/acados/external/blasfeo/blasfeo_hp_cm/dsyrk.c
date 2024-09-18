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

#include <blasfeo_timing.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_hp_dsyrk3_ln blasfeo_hp_cm_dsyrk3_ln
#define blasfeo_hp_dsyrk3_lt blasfeo_hp_cm_dsyrk3_lt
#define blasfeo_hp_dsyrk3_un blasfeo_hp_cm_dsyrk3_un
#define blasfeo_hp_dsyrk3_ut blasfeo_hp_cm_dsyrk3_ut
#define blasfeo_hp_dsyrk_ln blasfeo_hp_cm_dsyrk_ln
#define blasfeo_hp_dsyrk_ln_mn blasfeo_hp_cm_dsyrk_ln_mn
#define blasfeo_hp_dsyrk_lt blasfeo_hp_cm_dsyrk_lt
#define blasfeo_hp_dsyrk_un blasfeo_hp_cm_dsyrk_un
#define blasfeo_hp_dsyrk_ut blasfeo_hp_cm_dsyrk_ut
#define blasfeo_dsyrk3_ln blasfeo_cm_dsyrk3_ln
#define blasfeo_dsyrk3_lt blasfeo_cm_dsyrk3_lt
#define blasfeo_dsyrk3_un blasfeo_cm_dsyrk3_un
#define blasfeo_dsyrk3_ut blasfeo_cm_dsyrk3_ut
#define blasfeo_dsyrk_ln blasfeo_cm_dsyrk_ln
#define blasfeo_dsyrk_ln_mn blasfeo_cm_dsyrk_ln_mn
#define blasfeo_dsyrk_lt blasfeo_cm_dsyrk_lt
#define blasfeo_dsyrk_un blasfeo_cm_dsyrk_un
#define blasfeo_dsyrk_ut blasfeo_cm_dsyrk_ut
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



static void blasfeo_hp_dsyrk3_ln_m2(int m, int k, double alpha, double *pA, int sda, double beta, double *C, int ldc, double *D, int ldd)
	{

	int ii, jj;

	// TODO SKYLAKE_X target since it uses ps=8 !!!!!

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_12x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_l_8x8_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pA+(jj+4)*sda, sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
#else
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pA+(ii+8)*sda, pA+(jj+8)*sda, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pA+(ii+4)*sda, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
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
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pA+ii*sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pA+ii*sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		}
	if(ii<m)
		{
		goto ln_m2_left_4;
		}
#endif
	goto ln_m2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_m2_left_12:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pA+(jj+4)*sda, sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pA+(ii+4)*sda, sda, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pA+(ii+8)*sda, pA+(jj+8)*sda, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
#endif
	goto ln_m2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_m2_left_8:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pA+(ii+4)*sda, pA+(jj+4)*sda, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto ln_m2_return;
#endif

ln_m2_left_4:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto ln_m2_return;

ln_m2_return:
	return;

	}



static void blasfeo_hp_dsyrk3_un_m2(int m, int k, double alpha, double *pA, int sda, double beta, double *C, int ldc, double *D, int ldd)
	{

	int ii, jj;

	// TODO SKYLAKE_X target since it uses ps=8 !!!!!

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_u_8x8_lib44cc(k, &alpha, pA+ii*sda, sda, pA+ii*sda, sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
#else
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pA+ii*sda, pA+ii*sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+(ii+4)*sda, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
#endif
		kernel_dsyrk_nt_u_12x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+(ii+8)*sda, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd);
		for(jj=ii+12; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto un_2_left_4;
			}
		if(m-ii<=8)
			{
			goto un_2_left_8;
			}
		else
			{
			goto un_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pA+ii*sda, pA+ii*sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+(ii+4)*sda, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
		for(jj=ii+8; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto un_2_left_4;
			}
		else
			{
			goto un_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pA+ii*sda, pA+ii*sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		for(jj=ii+4; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pA+ii*sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pA+jj*sda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		goto un_2_left_4;
		}
#endif
	goto un_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
un_2_left_12:
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+ii*sda, sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pA+ii*sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+(ii+4)*sda, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	kernel_dsyrk_nt_u_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+(ii+8)*sda, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd, m-ii, m-(ii+8));
	goto un_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
un_2_left_8:
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+ii*sda, sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pA+ii*sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pA+(ii+4)*sda, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	goto un_2_return;
#endif

un_2_left_4:
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pA+ii*sda, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	goto un_2_return;

un_2_return:
	return;

	}



static void blasfeo_hp_dsyrk3_ln_m1(int m, int k, double alpha, double *A, int lda, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, pU, sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_12x4_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyrk_nt_l_8x8_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
//		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
//		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+8*sdu, pU+8*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
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
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+4*sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
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
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, &alpha, pU, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
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
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
//	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
//	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+8*sdu, pU+8*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
	goto ln_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+4*sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto ln_1_return;
#endif

ln_1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto ln_1_return;

ln_1_return:
	return;

	}



static void blasfeo_hp_dsyrk3_lt_m1(int m, int k, double alpha, double *A, int lda, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nn_12x4_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_12x4_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_l_8x8_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
#else
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+8*sdu, pU+8*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
#endif
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lt_1_left_4;
			}
		if(m-ii<=8)
			{
			goto lt_1_left_8;
			}
		else
			{
			goto lt_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nn_8x4_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+4*sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lt_1_left_4;
			}
		else
			{
			goto lt_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nn_4x4_lib4ccc(k, &alpha, pU, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		}
	if(ii<m)
		{
		goto lt_1_left_4;
		}
#endif
	goto lt_1_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lt_1_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-(ii+8));
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+8*sdu, pU+8*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
#endif
	goto lt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lt_1_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-(ii+4));
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+4*sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto lt_1_return;
#endif

lt_1_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto lt_1_return;

lt_1_return:
	return;

	}



static void blasfeo_hp_dsyrk3_un_m1(int m, int k, double alpha, double *A, int lda, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, pU, sdu);
		kernel_dsyrk_nt_u_8x8_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
//		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
//		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
		kernel_dsyrk_nt_u_12x4_lib44cc(k, &alpha, pU, sdu, pU+8*sdu, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd);
		for(jj=ii+12; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto un_1_left_4;
			}
		if(m-ii<=8)
			{
			goto un_1_left_8;
			}
		else
			{
			goto un_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, pU, sdu);
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
		for(jj=ii+8; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto un_1_left_4;
			}
		else
			{
			goto un_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		for(jj=ii+4; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, &alpha, pU, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		goto un_1_left_4;
		}
#endif
	goto un_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
un_1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
//	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
//	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
	kernel_dsyrk_nt_u_12x4_vs_lib44cc(k, &alpha, pU, sdu, pU+8*sdu, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd, m-ii, m-(ii+8));
	goto un_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
un_1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	goto un_1_return;
#endif

un_1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	goto un_1_return;

un_1_return:
	return;

	}



static void blasfeo_hp_dsyrk3_ut_m1(int m, int k, double alpha, double *A, int lda, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_u_8x8_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
#else
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
#endif
		kernel_dsyrk_nt_u_12x4_lib44cc(k, &alpha, pU, sdu, pU+8*sdu, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd);
		for(jj=ii+12; jj<m-3; jj+=4)
			{
			kernel_dgemm_nn_12x4_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ut_1_left_4;
			}
		if(m-ii<=8)
			{
			goto ut_1_left_8;
			}
		else
			{
			goto ut_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
		for(jj=ii+8; jj<m-3; jj+=4)
			{
			kernel_dgemm_nn_8x4_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ut_1_left_4;
			}
		else
			{
			goto ut_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		for(jj=ii+4; jj<m-3; jj+=4)
			{
			kernel_dgemm_nn_4x4_lib4ccc(k, &alpha, pU, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		goto ut_1_left_4;
		}
#endif
	goto ut_1_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ut_1_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-(ii+8));
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	kernel_dsyrk_nt_u_12x4_vs_lib44cc(k, &alpha, pU, sdu, pU+8*sdu, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd, m-ii, m-(ii+8));
	goto ut_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ut_1_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-(ii+4));
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	goto ut_1_return;
#endif

ut_1_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	goto ut_1_return;

ut_1_return:
	return;

	}



void blasfeo_hp_dsyrk3_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dsyrk3_ln (cm) %d %d %f %p %d %d %f %p %d %d %p %d %d\n", m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
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
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	int m_a = m==lda ? m : m_cache;
//	int n_b = n==ldb ? n : n_cache;
	int n_b = m_a; // syrk: B=A
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!


//	goto ln_1;
//	goto ln_2;
#if defined(TARGET_X64_INTEL_HASWELL)
//	if(m<200 & k<200)
	if( m<=2*m_kernel | n_b*k_block <= l2_cache_el )
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & k<64)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
//	if(m<32 & k<32)
	if( m<=2*m_kernel | 2*m*k_block < llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<16 & k<16)
#else
	if(m<12 & k<12)
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

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
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
	
	sdu = k4<sdu ? k4 : sdu;

//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dsyrk3_ln_m1(m, k, alpha, A, lda, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
#if 0
			if(k-ll<2*kc)
				{
				if(k-ll<=kc) // last
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

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_dsyrk3_ln_m1(m, kleft, alpha, A+ll*lda, lda, beta1, C1, ldc1, D, ldd, pU, sdu);
			}

		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
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

	mc = m<mc0 ? m : mc0;
	nc = m<nc0 ? m : nc0; // XXX just one buffer if m small enough ???
	kc = k<kc0 ? k : kc0;

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
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
			for(iii=0; iii<kleft-3; iii+=4)
				{
				kernel_dpack_tt_4_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda);
				}
			if(iii<kleft)
				{
				kernel_dpack_tt_4_vs_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda, kleft-iii);
				}
#else
			kernel_dpack_buffer_fn(mleft, kleft, A+ii+ll*lda, lda, pA, sda);
#endif

			// dgemm
			for(jj=0; jj<ii; jj+=nleft)
				{

				nleft = ii-jj<nc ? ii-jj : nc;

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<kleft-3; iii+=4)
					{
					kernel_dpack_tt_4_lib8(nleft, A+jj+(ll+iii)*lda, lda, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_dpack_tt_4_vs_lib8(nleft, A+jj+(ll+iii)*lda, lda, pB+iii*ps, sdb, kleft-iii);
					}
#else
				kernel_dpack_buffer_fn(nleft, kleft, A+jj+ll*lda, lda, pB, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);

				}

			// dsyrk
#if 1
			for(iii=0; iii<mleft; iii+=nleft)
				{
				nleft = mleft-iii<nc ? mleft-iii : nc;
				blasfeo_hp_dsyrk3_ln_m2(nleft, kleft, alpha, pA+iii*sda, sda, beta1, C1+(ii+iii)+(ii+iii)*ldc1, ldc1, D+(ii+iii)+(ii+iii)*ldd, ldd);
				blasfeo_hp_dgemm_nt_m2(mleft-iii-nleft, nleft, kleft, alpha, pA+(iii+nleft)*sda, sda, pA+iii*sda, sda, beta1, C1+(ii+iii+nleft)+(ii+iii)*ldc1, ldc1, D+(ii+iii+nleft)+(ii+iii)*ldd, ldd);
				}
#else
			blasfeo_hp_dsyrk3_ln_m2(mleft, kleft, alpha, pA, sda, beta1, C1+ii+ii*ldc1, ldc1, D+ii+ii*ldd, ldd);
#endif

			}

		}

	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;

#else

	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m1, k1);
	mem = malloc(tA_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m, k, &tA, (void *) mem_align);

	pU = tA.pA;
	sdu = tA.cn;

	// pack A
	kernel_dpack_buffer_fn(m, k, A, lda, pU, sdu);

//	blasfeo_print_dmat(m, k, &tA, 0, 0);

	blasfeo_hp_dsyrk3_ln_m2(m, k, alpha, pU, sdu, beta, C, ldc, D, ldd);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}



void blasfeo_hp_dsyrk3_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dsyrk3_lt (cm) %d %d %f %p %d %d %f %p %d %d %p %d %d\n", m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
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
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const int llc_cache_el = LLC_CACHE_EL;
#endif
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!

	int k_a = k==lda ? k : k_cache;
	int m_c = m==ldc ? m : m_cache;
	int m_d = m==ldd ? m : m_cache;


//	goto lt_1;
//	goto lt_2;
#if defined(TARGET_X64_INTEL_HASWELL)
//	if(m<=300 & k<=300)
	if( m<=2*m_kernel | (2*k_a*m + m_c*m + m_d*m <= llc_cache_el) )
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & k<64)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
//	if(m<32 & k<32)
	if( m<=2*m_kernel | 2*m*k_block < llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<16 & k<16)
#else
	if(m<12 & k<12)
#endif
		{
		goto lt_1;
		}
	else
		{
		goto lt_2;
		}

	// never to get here
	return;



lt_1:
	// k-blocking alg

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
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
	
	sdu = k4<sdu ? k4 : sdu;

//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dsyrk3_lt_m1(m, k, alpha, A, lda, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
#if 0
			if(k-ll<2*kc)
				{
				if(k-ll<=kc) // last
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

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_dsyrk3_lt_m1(m, kleft, alpha, A+ll, lda, beta1, C1, ldc1, D, ldd, pU, sdu);
			}

		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



lt_2:
#if ! defined(TARGET_X64_INTEL_SKYLAKE_X)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

	mc = m<mc0 ? m : mc0;
	nc = m<nc0 ? m : nc0; // XXX just one buffer if m small enough ???
	kc = k<kc0 ? k : kc0;

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

			// pack and tran A
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
			for(iii=0; iii<mleft-7; iii+=8)
				{
				kernel_dpack_tn_8_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda);
				}
			if(iii<mleft)
				{
				kernel_dpack_tn_8_vs_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda, mleft-iii);
				}
#else
			kernel_dpack_buffer_ft(kleft, mleft, A+ll+ii*lda, lda, pA, sda);
#endif

			// dgemm
			for(jj=0; jj<ii; jj+=nleft)
				{

				nleft = ii-jj<nc ? ii-jj : nc;

				// pack B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<nleft-7; iii+=8)
					{
					kernel_dpack_tn_8_lib8(kleft, A+ll+(jj+iii)*lda, lda, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_dpack_tn_8_vs_lib8(kleft, A+ll+(jj+iii)*lda, lda, pB+iii*sdb, nleft-iii);
					}
#else
				kernel_dpack_buffer_ft(kleft, nleft, A+ll+jj*lda, lda, pB, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);

				}

			// dsyrk
#if 1
			for(iii=0; iii<mleft; iii+=nleft)
				{
				nleft = mleft-iii<nc ? mleft-iii : nc;
				blasfeo_hp_dsyrk3_ln_m2(nleft, kleft, alpha, pA+iii*sda, sda, beta1, C1+(ii+iii)+(ii+iii)*ldc1, ldc1, D+(ii+iii)+(ii+iii)*ldd, ldd);
				blasfeo_hp_dgemm_nt_m2(mleft-iii-nleft, nleft, kleft, alpha, pA+(iii+nleft)*sda, sda, pA+iii*sda, sda, beta1, C1+(ii+iii+nleft)+(ii+iii)*ldc1, ldc1, D+(ii+iii+nleft)+(ii+iii)*ldd, ldd);
				}
#else
			blasfeo_hp_dsyrk3_ln_m2(mleft, kleft, alpha, pA, sda, beta1, C1+ii+ii*ldc1, ldc1, D+ii+ii*ldd, ldd);
#endif

			}

		}

	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;

#else

	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m1, k1);
	mem = malloc(tA_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m, k, &tA, (void *) mem_align);

	pU = tA.pA;
	sdu = tA.cn;

//	if(ta=='n' | ta=='N')
//		blasfeo_pack_dmat(m, k, A, lda, &tA, 0, 0);
//	else
//		blasfeo_pack_tran_dmat(k, m, A, lda, &tA, 0, 0);

	kernel_dpack_buffer_ft(k, m, A, lda, pU, sdu);


//	blasfeo_print_dmat(m, k, &tA, 0, 0);

	blasfeo_hp_dsyrk3_ln_m2(m, k, alpha, pU, sdu, beta, C, ldc, D, ldd);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}



void blasfeo_hp_dsyrk3_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dsyrk3_un (cm) %d %d %f %p %d %d %f %p %d %d %p %d %d\n", m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
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
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	int m_a = m==lda ? m : m_cache;
//	int n_b = n==ldb ? n : n_cache;
	int n_b = m_a; // syrk: B=A
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!


//	goto un_1;
//	goto un_2;
#if defined(TARGET_X64_INTEL_HASWELL)
//	if(m<200 & k<200)
	if( m<=2*m_kernel | n_b*k_block <= l2_cache_el )
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & k<64)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
//	if(m<32 & k<32)
	if( m<=2*m_kernel | 2*m*k_block < llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<16 & k<16)
#else
	if(m<12 & k<12)
#endif
		{
		goto un_1;
		}
	else
		{
		goto un_2;
		}

	// never to get here
	return;



un_1:
	// k-blocking alg

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
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
	
	sdu = k4<sdu ? k4 : sdu;

//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dsyrk3_un_m1(m, k, alpha, A, lda, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
#if 0
			if(k-ll<2*kc)
				{
				if(k-ll<=kc) // last
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

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_dsyrk3_un_m1(m, kleft, alpha, A+ll*lda, lda, beta1, C1, ldc1, D, ldd, pU, sdu);
			}

		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



un_2:
#if ! defined(TARGET_X64_INTEL_SKYLAKE_X)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

	mc = m<mc0 ? m : mc0;
	nc = m<nc0 ? m : nc0; // XXX just one buffer if m small enough ???
	kc = k<kc0 ? k : kc0;

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
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
			for(iii=0; iii<kleft-3; iii+=4)
				{
				kernel_dpack_tt_4_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda);
				}
			if(iii<kleft)
				{
				kernel_dpack_tt_4_vs_lib8(mleft, A+ii+(ll+iii)*lda, lda, pA+iii*ps, sda, kleft-iii);
				}
#else
			kernel_dpack_buffer_fn(mleft, kleft, A+ii+ll*lda, lda, pA, sda);
#endif

			// dsyrk
#if 1
			for(iii=0; iii<mleft; iii+=nleft)
				{
				nleft = mleft-iii<nc ? mleft-iii : nc;
				blasfeo_hp_dgemm_nt_m2(iii, nleft, kleft, alpha, pA, sda, pA+iii*sda, sda, beta1, C1+ii+(ii+iii)*ldc1, ldc1, D+ii+(ii+iii)*ldd, ldd);
				blasfeo_hp_dsyrk3_un_m2(nleft, kleft, alpha, pA+iii*sda, sda, beta1, C1+(ii+iii)+(ii+iii)*ldc1, ldc1, D+(ii+iii)+(ii+iii)*ldd, ldd);
				}
#else
			blasfeo_hp_dsyrk3_un_m2(mleft, kleft, alpha, pA, sda, beta1, C1+ii+ii*ldc1, ldc1, D+ii+ii*ldd, ldd);
#endif

			// dgemm
			for(jj=ii+mleft; jj<m; jj+=nleft)
				{

				nleft = m-jj<nc ? m-jj : nc;

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<kleft-3; iii+=4)
					{
					kernel_dpack_tt_4_lib8(nleft, A+jj+(ll+iii)*lda, lda, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_dpack_tt_4_vs_lib8(nleft, A+jj+(ll+iii)*lda, lda, pB+iii*ps, sdb, kleft-iii);
					}
#else
				kernel_dpack_buffer_fn(nleft, kleft, A+jj+ll*lda, lda, pB, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);

				}

			}

		}

	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;

#else

	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m1, k1);
	mem = malloc(tA_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m, k, &tA, (void *) mem_align);

	pU = tA.pA;
	sdu = tA.cn;

//	if(ta=='n' | ta=='N')
//		blasfeo_pack_dmat(m, k, A, lda, &tA, 0, 0);
//	else
//		blasfeo_pack_tran_dmat(k, m, A, lda, &tA, 0, 0);
	kernel_dpack_buffer_fn(m, k, A, lda, pU, sdu);

//	blasfeo_print_dmat(m, k, &tA, 0, 0);

	blasfeo_hp_dsyrk3_un_m2(m, k, alpha, pU, sdu, beta, C, ldc, D, ldd);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}



void blasfeo_hp_dsyrk3_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dsyrk3_ut (cm) %d %d %f %p %d %d %f %p %d %d %p %d %d\n", m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
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
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const int llc_cache_el = LLC_CACHE_EL;
#endif
	int pack_B;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!

	int k_a = k==lda ? k : k_cache;
	int m_c = m==ldc ? m : m_cache;
	int m_d = m==ldd ? m : m_cache;


//	goto ut_1;
//	goto ut_2;
#if defined(TARGET_X64_INTEL_HASWELL)
//	if(m<=300 & k<=300)
	if( m<=2*m_kernel | (2*k_a*m + m_c*m + m_d*m <= llc_cache_el) )
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & k<64)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
//	if(m<32 & k<32)
	if( m<=2*m_kernel | 2*m*k_block < llc_cache_el )
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<16 & k<16)
#else
	if(m<12 & k<12)
#endif
		{
		goto ut_1;
		}
	else
		{
		goto ut_2;
		}

ut_1:
	// k-blocking alg

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
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
	
	sdu = k4<sdu ? k4 : sdu;

//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dsyrk3_ut_m1(m, k, alpha, A, lda, beta, C, ldc, D, ldd, pU, sdu);
		}
	else
		{
		for(ll=0; ll<k; ll+=kleft)
			{
#if 0
			if(k-ll<2*kc)
				{
				if(k-ll<=kc) // last
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

			sdu = (kleft+4-1)/4*4;

			beta1 = ll==0 ? beta : 1.0;
			C1 = ll==0 ? C : D;
			ldc1 = ll==0 ? ldc : ldd;

			blasfeo_hp_dsyrk3_ut_m1(m, kleft, alpha, A+ll, lda, beta1, C1, ldc1, D, ldd, pU, sdu);
			}

		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



ut_2:
#if ! defined(TARGET_X64_INTEL_SKYLAKE_X)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;

	mc = m<mc0 ? m : mc0;
	nc = m<nc0 ? m : nc0; // XXX just one buffer if m small enough ???
	kc = k<kc0 ? k : kc0;

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

			// pack and tran A
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
			for(iii=0; iii<mleft-7; iii+=8)
				{
				kernel_dpack_tn_8_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda);
				}
			if(iii<mleft)
				{
				kernel_dpack_tn_8_vs_lib8(kleft, A+ll+(ii+iii)*lda, lda, pA+iii*sda, mleft-iii);
				}
#else
			kernel_dpack_buffer_ft(kleft, mleft, A+ll+ii*lda, lda, pA, sda);
#endif

			// dsyrk
#if 1
			for(iii=0; iii<mleft; iii+=nleft)
				{
				nleft = mleft-iii<nc ? mleft-iii : nc;
				blasfeo_hp_dgemm_nt_m2(iii, nleft, kleft, alpha, pA, sda, pA+iii*sda, sda, beta1, C1+ii+(ii+iii)*ldc1, ldc1, D+ii+(ii+iii)*ldd, ldd);
				blasfeo_hp_dsyrk3_un_m2(nleft, kleft, alpha, pA+iii*sda, sda, beta1, C1+(ii+iii)+(ii+iii)*ldc1, ldc1, D+(ii+iii)+(ii+iii)*ldd, ldd);
				}
#else
			blasfeo_hp_dsyrk3_un_m2(mleft, kleft, alpha, pA, sda, beta1, C1+ii+ii*ldc1, ldc1, D+ii+ii*ldd, ldd);
#endif

			// dgemm
			for(jj=ii+mleft; jj<m; jj+=nleft)
				{

				nleft = m-jj<nc ? m-jj : nc;

				// pack B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<nleft-7; iii+=8)
					{
					kernel_dpack_tn_8_lib8(kleft, A+ll+(jj+iii)*lda, lda, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_dpack_tn_8_vs_lib8(kleft, A+ll+(jj+iii)*lda, lda, pB+iii*sdb, nleft-iii);
					}
#else
				kernel_dpack_buffer_ft(kleft, nleft, A+ll+jj*lda, lda, pB, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);

				}

			}

		}

	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}

	return;

#else

	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m1, k1);
	mem = malloc(tA_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m, k, &tA, (void *) mem_align);

	pU = tA.pA;
	sdu = tA.cn;

//	if(ta=='n' | ta=='N')
//		blasfeo_pack_dmat(m, k, A, lda, &tA, 0, 0);
//	else
//		blasfeo_pack_tran_dmat(k, m, A, lda, &tA, 0, 0);

	kernel_dpack_buffer_ft(k, m, A, lda, pU, sdu);

//	blasfeo_print_dmat(m, k, &tA, 0, 0);

	blasfeo_hp_dsyrk3_un_m2(m, k, alpha, pU, sdu, beta, C, ldc, D, ldd);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}



void blasfeo_hp_dsyrk_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dsyrk_ln (cm) %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*lda;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;

	// call routine exploiting A==B
	if(A==B & lda==ldb)
		{
		blasfeo_hp_dsyrk3_ln(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
		return;
		}

//	printf("\n%p %d %p %d %p %d\n", A, lda, C, ldc, D, ldd);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif
	int sdu0 = (k+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	double *pU, *pB;
	int sdu;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;



//	goto ln_1;
//	goto lx_2;
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | k>=200 | k>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | k>=64 | k>K_MAX_STACK)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if(m>=32 | k>=32 | k>K_MAX_STACK)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m>16 | k>16 | k>K_MAX_STACK)
#else
	if(m>=12 | k>=12 | k>K_MAX_STACK)
#endif
		{
		goto lx_2;
		}
	else
		{
		goto ln_1;
		}

	// never to get here
	return;



ln_1:
	pU = pU0;
	sdu = sdu0;
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, pU, sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//		kernel_dsyrk_nt_l_8x8_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		kernel_dsyrk_nt_l_8x4_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib4ccc(k, &alpha, pU+8*sdu, B+jj+8, ldb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
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
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib4ccc(k, &alpha, pU+4*sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
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
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
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
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
//	kernel_dsyrk_nt_l_8x8_vs_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	kernel_dsyrk_nt_l_8x4_vs_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU+8*sdu, B+jj+8, ldb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
	goto ln_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyrk_nt_l_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU+4*sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto ln_1_return;
#endif

ln_1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto ln_1_return;

ln_1_return:
	return;



lx_2:
	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, k1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, k, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;

//	if(ta=='n' | ta=='N')
//		blasfeo_pack_dmat(m, k, A, lda, &tA, 0, 0);
//	else
//		blasfeo_pack_tran_dmat(k, m, A, lda, &tA, 0, 0);
	kernel_dpack_buffer_fn(m, k, B, ldb, pB, sdb);

//	blasfeo_print_dmat(m, k, &tA, 0, 0);

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, pU, sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_12x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_l_8x8_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
#else
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+8*sdu, pB+(jj+8)*sdb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
#endif
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lx_2_left_4;
			}
		if(m-ii<=8)
			{
			goto lx_2_left_8;
			}
		else
			{
			goto lx_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, pU, sdu);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+4*sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lx_2_left_4;
			}
		else
			{
			goto lx_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		}
	if(ii<m)
		{
		goto lx_2_left_4;
		}
#endif
	goto lx_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lx_2_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+8*sdu, pB+(jj+8)*sdb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
#endif
	goto lx_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lx_2_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+4*sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto lx_2_return;
#endif

lx_2_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto lx_2_return;

lx_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_dsyrk_ln_mn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dsyrk_ln_mn (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*lda;
	double *C = sC->pA + ci + cj*ldc;
	double *D = sD->pA + di + dj*ldd;

	// call routine exploiting A==B
//	if(A==B & lda==ldb)
//		{
//		blasfeo_hp_dsyrk3_ln(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
//		return;
//		}

//	printf("\n%p %d %p %d %p %d\n", A, lda, C, ldc, D, ldd);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif
	int sdu0 = (k+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	double *pU, *pB;
	int sdu;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;



//	goto ln_1;
//	goto lx_2;
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | k>=200 | k>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | k>=64 | k>K_MAX_STACK)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if(m>=32 | k>=32 | k>K_MAX_STACK)
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m>16 | k>16 | k>K_MAX_STACK)
#else
	if(m>=12 | k>=12 | k>K_MAX_STACK)
#endif
		{
		goto lx_2;
		}
	else
		{
		goto ln_1;
		}

	// never to get here
	return;



ln_1:
	pU = pU0;
	sdu = sdu0;
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, pU, sdu);
		for(jj=0; jj<ii & jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(jj<ii) // dgemm
				{
				kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
				}
			else // dsyrk
				{
				if(jj<n-11)
					{
					kernel_dsyrk_nt_l_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//					kernel_dsyrk_nt_l_8x8_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
					kernel_dsyrk_nt_l_8x4_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
					kernel_dsyrk_nt_l_4x4_lib4ccc(k, &alpha, pU+8*sdu, B+jj+8, ldb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
//#endif
					}
				else
					{
					kernel_dsyrk_nt_l_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
					if(jj<n-4)
						{
						kernel_dsyrk_nt_l_8x4_vs_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd, m-ii-4, n-jj-4);
						if(jj<n-8)
							{
							kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU+8*sdu, B+jj+8, ldb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd, m-ii-8, n-jj-8);
							}
						}
					}
				}
			}
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
		for(jj=0; jj<ii & jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(jj<ii) // dgemm
				{
				kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
				}
			else // dsyrk
				{
				if(jj<n-7)
					{
					kernel_dsyrk_nt_l_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
					kernel_dsyrk_nt_l_4x4_lib4ccc(k, &alpha, pU+4*sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
					}
				else
					{
					kernel_dsyrk_nt_l_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
					if(jj<n-4)
						{
						kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU+4*sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd, m-ii-4, n-jj-4);
						}
					}
				}
			}
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
		for(jj=0; jj<ii & jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(jj<ii) // dgemm
				{
				kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
				}
			else // dsyrk
				{
				if(jj<n-3)
					{
					kernel_dsyrk_nt_l_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
					}
				else
					{
					kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
					}
				}
			}
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
	for(jj=0; jj<ii & jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
//	kernel_dsyrk_nt_l_8x8_vs_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	if(jj<n)
		{
		kernel_dsyrk_nt_l_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, n-jj);
		if(jj<n-4)
			{
			kernel_dsyrk_nt_l_8x4_vs_lib4ccc(k, &alpha, pU+4*sdu, sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), n-(jj+4));
			if(jj<n-8)
				{
				kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU+8*sdu, B+jj+8, ldb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), n-(jj+8));
				}
			}
		}
	goto ln_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii & jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
//	kernel_dsyrk_nt_l_8x8_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	if(jj<n)
		{
		kernel_dsyrk_nt_l_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, n-jj);
		if(jj<n-4)
			{
			kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU+4*sdu, B+jj+4, ldb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), n-(jj+4));
			}
		}
	goto ln_1_return;
#endif

ln_1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<ii & jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dsyrk_nt_l_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto ln_1_return;

ln_1_return:
	return;



lx_2:
	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, k1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, k, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;

//	if(ta=='n' | ta=='N')
//		blasfeo_pack_dmat(m, k, A, lda, &tA, 0, 0);
//	else
//		blasfeo_pack_tran_dmat(k, m, A, lda, &tA, 0, 0);
	kernel_dpack_buffer_fn(m, k, B, ldb, pB, sdb);

//	blasfeo_print_dmat(m, k, &tA, 0, 0);

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, pU, sdu);
		for(jj=0; jj<ii & jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(jj<ii) // dgemm
				{
				kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
				}
			else // dsyrk
				{
				if(jj<n-11)
					{
					kernel_dsyrk_nt_l_12x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//					kernel_dsyrk_nt_l_8x8_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
					kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
					kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+8*sdu, pB+(jj+8)*sdb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
					}
				else
					{
					kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
					if(jj<n-4)
						{
						kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd, m-ii-4, n-jj-4);
						if(jj<n-8)
							{
							kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+8*sdu, pB+(jj+8)*sdb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd, m-ii-8, n-jj-8);
							}
						}
					}
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lx_2_left_4;
			}
		if(m-ii<=8)
			{
			goto lx_2_left_8;
			}
		else
			{
			goto lx_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, pU, sdu);
		for(jj=0; jj<ii & jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(jj<ii) // dgemm
				{
				kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
				}
			else // dsyrk
				{
				if(jj<n-7)
					{
					kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
					kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+4*sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
					}
				else
					{
					kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
					if(jj<n-4)
						{
						kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+4*sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd, m-ii-4, n-jj-4);
						}
					}
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lx_2_left_4;
			}
		else
			{
			goto lx_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<ii & jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(jj<ii) // dgemm
				{
				kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
				}
			else // dsyrk
				{
				if(jj<n-3)
					{
					kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
					}
				else
					{
					kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
					}
				}
			}
		}
	if(ii<m)
		{
		goto lx_2_left_4;
		}
#endif
	goto lx_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lx_2_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii & jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
//	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	if(jj<n)
		{
		kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, n-jj);
		if(jj<n-4)
			{
			kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), n-(jj+4));
			if(jj<n-8)
				{
				kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+8*sdu, pB+(jj+8)*sdb, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), n-(jj+8));
				}
			}
		}
	goto lx_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lx_2_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii & jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
//	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	if(jj<n)
		{
		kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU, sdu, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, n-jj);
		if(jj<n-4)
			{
			kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+4*sdu, pB+(jj+4)*sdb, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), n-(jj+4));
			}
		}
	goto lx_2_return;
#endif

lx_2_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<ii & jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto lx_2_return;

lx_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



// dsyrk_lower transposed
void blasfeo_hp_dsyrk_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	double
		c_00, c_01,
		c_10, c_11;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *pA = sA->pA + ai + aj*lda;
	double *pB = sB->pA + bi + bj*ldb;
	double *pC = sC->pA + ci + cj*ldc;
	double *pD = sD->pA + di + dj*ldd;

	// call routine exploiting A==B
	if(pA==pB & lda==ldb)
		{
		blasfeo_hp_dsyrk3_lt(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
		return;
		}

	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// diagonal
		c_00 = 0.0;
		c_10 = 0.0;
		c_11 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += pA[kk+(jj+0)*lda] * pB[kk+(jj+0)*ldb];
			c_10 += pA[kk+(jj+1)*lda] * pB[kk+(jj+0)*ldb];
			c_11 += pA[kk+(jj+1)*lda] * pB[kk+(jj+1)*ldb];
			}
		pD[(jj+0)+(jj+0)*ldd] = beta * pC[(jj+0)+(jj+0)*ldc] + alpha * c_00;
		pD[(jj+1)+(jj+0)*ldd] = beta * pC[(jj+1)+(jj+0)*ldc] + alpha * c_10;
		pD[(jj+1)+(jj+1)*ldd] = beta * pC[(jj+1)+(jj+1)*ldc] + alpha * c_11;
		// lower
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += pA[kk+(ii+0)*lda] * pB[kk+(jj+0)*ldb];
				c_10 += pA[kk+(ii+1)*lda] * pB[kk+(jj+0)*ldb];
				c_01 += pA[kk+(ii+0)*lda] * pB[kk+(jj+1)*ldb];
				c_11 += pA[kk+(ii+1)*lda] * pB[kk+(jj+1)*ldb];
				}
			pD[(ii+0)+(jj+0)*ldd] = beta * pC[(ii+0)+(jj+0)*ldc] + alpha * c_00;
			pD[(ii+1)+(jj+0)*ldd] = beta * pC[(ii+1)+(jj+0)*ldc] + alpha * c_10;
			pD[(ii+0)+(jj+1)*ldd] = beta * pC[(ii+0)+(jj+1)*ldc] + alpha * c_01;
			pD[(jj+1)+(jj+1)*ldd] = beta * pC[(ii+1)+(jj+1)*ldc] + alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += pA[kk+(ii+0)*lda] * pB[kk+(jj+0)*ldb];
				c_01 += pA[kk+(ii+0)*lda] * pB[kk+(jj+1)*ldb];
				}
			pD[(ii+0)+(jj+0)*ldd] = beta * pC[(ii+0)+(jj+0)*ldc] + alpha * c_00;
			pD[(ii+0)+(jj+1)*ldd] = beta * pC[(ii+0)+(jj+1)*ldc] + alpha * c_01;
			}
		}
	if(jj<m)
		{
		// diagonal
		c_00 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += pA[kk+(jj+0)*lda] * pB[kk+(jj+0)*ldb];
			}
		pD[(jj+0)+(jj+0)*ldd] = beta * pC[(jj+0)+(jj+0)*ldc] + alpha * c_00;
		}
	return;
	}



// dsyrk_upper not-transposed
void blasfeo_hp_dsyrk_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	double
		c_00, c_01,
		c_10, c_11;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *pA = sA->pA + ai + aj*lda;
	double *pB = sB->pA + bi + bj*ldb;
	double *pC = sC->pA + ci + cj*ldc;
	double *pD = sD->pA + di + dj*ldd;

	// call routine exploiting A==B
	if(pA==pB & lda==ldb)
		{
		blasfeo_hp_dsyrk3_un(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
		return;
		}

	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += pA[(ii+0)+kk*lda] * pB[(jj+0)+kk*ldb];
				c_10 += pA[(ii+1)+kk*lda] * pB[(jj+0)+kk*ldb];
				c_01 += pA[(ii+0)+kk*lda] * pB[(jj+1)+kk*ldb];
				c_11 += pA[(ii+1)+kk*lda] * pB[(jj+1)+kk*ldb];
				}
			pD[(ii+0)+(jj+0)*ldd] = beta * pC[(ii+0)+(jj+0)*ldc] + alpha * c_00;
			pD[(ii+1)+(jj+0)*ldd] = beta * pC[(ii+1)+(jj+0)*ldc] + alpha * c_10;
			pD[(ii+0)+(jj+1)*ldd] = beta * pC[(ii+0)+(jj+1)*ldc] + alpha * c_01;
			pD[(jj+1)+(jj+1)*ldd] = beta * pC[(ii+1)+(jj+1)*ldc] + alpha * c_11;
			}
		// diagonal
		c_00 = 0.0;
		c_01 = 0.0;
		c_11 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += pA[(jj+0)+kk*lda] * pB[(jj+0)+kk*ldb];
			c_01 += pA[(jj+0)+kk*lda] * pB[(jj+1)+kk*ldb];
			c_11 += pA[(jj+1)+kk*lda] * pB[(jj+1)+kk*ldb];
			}
		pD[(jj+0)+(jj+0)*ldd] = beta * pC[(jj+0)+(jj+0)*ldc] + alpha * c_00;
		pD[(jj+0)+(jj+1)*ldd] = beta * pC[(jj+0)+(jj+1)*ldc] + alpha * c_01;
		pD[(jj+1)+(jj+1)*ldd] = beta * pC[(jj+1)+(jj+1)*ldc] + alpha * c_11;
		}
	if(jj<m)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += pA[(ii+0)+kk*lda] * pB[(jj+0)+kk*ldb];
				c_10 += pA[(ii+1)+kk*lda] * pB[(jj+0)+kk*ldb];
				}
			pD[(ii+0)+(jj+0)*ldd] = beta * pC[(ii+0)+(jj+0)*ldc] + alpha * c_00;
			pD[(ii+1)+(jj+0)*ldd] = beta * pC[(ii+1)+(jj+0)*ldc] + alpha * c_10;
			}
		// diagonal
		c_00 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += pA[(jj+0)+kk*lda] * pB[(jj+0)+kk*ldb];
			}
		pD[(jj+0)+(jj+0)*ldd] = beta * pC[(jj+0)+(jj+0)*ldc] + alpha * c_00;
		}
	return;
	}



// dsyrk_upper transposed
void blasfeo_hp_dsyrk_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	double
		c_00, c_01,
		c_10, c_11;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	double *pA = sA->pA + ai + aj*lda;
	double *pB = sB->pA + bi + bj*ldb;
	double *pC = sC->pA + ci + cj*ldc;
	double *pD = sD->pA + di + dj*ldd;

	// call routine exploiting A==B
	if(pA==pB & lda==ldb)
		{
		blasfeo_hp_dsyrk3_ut(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
		return;
		}

	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += pA[kk+(ii+0)*lda] * pB[kk+(jj+0)*ldb];
				c_10 += pA[kk+(ii+1)*lda] * pB[kk+(jj+0)*ldb];
				c_01 += pA[kk+(ii+0)*lda] * pB[kk+(jj+1)*ldb];
				c_11 += pA[kk+(ii+1)*lda] * pB[kk+(jj+1)*ldb];
				}
			pD[(ii+0)+(jj+0)*ldd] = beta * pC[(ii+0)+(jj+0)*ldc] + alpha * c_00;
			pD[(ii+1)+(jj+0)*ldd] = beta * pC[(ii+1)+(jj+0)*ldc] + alpha * c_10;
			pD[(ii+0)+(jj+1)*ldd] = beta * pC[(ii+0)+(jj+1)*ldc] + alpha * c_01;
			pD[(jj+1)+(jj+1)*ldd] = beta * pC[(ii+1)+(jj+1)*ldc] + alpha * c_11;
			}
		// diagonal
		c_00 = 0.0;
		c_01 = 0.0;
		c_11 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += pA[kk+(jj+0)*lda] * pB[kk+(jj+0)*ldb];
			c_01 += pA[kk+(jj+0)*lda] * pB[kk+(jj+1)*ldb];
			c_11 += pA[kk+(jj+1)*lda] * pB[kk+(jj+1)*ldb];
			}
		pD[(jj+0)+(jj+0)*ldd] = beta * pC[(jj+0)+(jj+0)*ldc] + alpha * c_00;
		pD[(jj+0)+(jj+1)*ldd] = beta * pC[(jj+0)+(jj+1)*ldc] + alpha * c_01;
		pD[(jj+1)+(jj+1)*ldd] = beta * pC[(jj+1)+(jj+1)*ldc] + alpha * c_11;
		}
	if(jj<m)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += pA[kk+(ii+0)*lda] * pB[kk+(jj+0)*ldb];
				c_10 += pA[kk+(ii+1)*lda] * pB[kk+(jj+0)*ldb];
				}
			pD[(ii+0)+(jj+0)*ldd] = beta * pC[(ii+0)+(jj+0)*ldc] + alpha * c_00;
			pD[(ii+1)+(jj+0)*ldd] = beta * pC[(ii+1)+(jj+0)*ldc] + alpha * c_10;
			}
		// diagonal
		c_00 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += pA[kk+(jj+0)*lda] * pB[kk+(jj+0)*ldb];
			}
		pD[(jj+0)+(jj+0)*ldd] = beta * pC[(jj+0)+(jj+0)*ldc] + alpha * c_00;
		}
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dsyrk3_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk3_ln(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk3_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk3_lt(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk3_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk3_un(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk3_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk3_ut(m, k, alpha, sA, ai, aj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk_ln_mn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk_ln_mn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



#endif

