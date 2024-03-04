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
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#include <blasfeo_stdlib.h>
#include <blasfeo_memory.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_hp_dtrsm_llnn blasfeo_hp_cm_dtrsm_llnn
#define blasfeo_hp_dtrsm_llnu blasfeo_hp_cm_dtrsm_llnu
#define blasfeo_hp_dtrsm_lltn blasfeo_hp_cm_dtrsm_lltn
#define blasfeo_hp_dtrsm_lltu blasfeo_hp_cm_dtrsm_lltu
#define blasfeo_hp_dtrsm_lunn blasfeo_hp_cm_dtrsm_lunn
#define blasfeo_hp_dtrsm_lunu blasfeo_hp_cm_dtrsm_lunu
#define blasfeo_hp_dtrsm_lutn blasfeo_hp_cm_dtrsm_lutn
#define blasfeo_hp_dtrsm_lutu blasfeo_hp_cm_dtrsm_lutu
#define blasfeo_hp_dtrsm_rlnn blasfeo_hp_cm_dtrsm_rlnn
#define blasfeo_hp_dtrsm_rlnu blasfeo_hp_cm_dtrsm_rlnu
#define blasfeo_hp_dtrsm_rltn blasfeo_hp_cm_dtrsm_rltn
#define blasfeo_hp_dtrsm_rltu blasfeo_hp_cm_dtrsm_rltu
#define blasfeo_hp_dtrsm_runn blasfeo_hp_cm_dtrsm_runn
#define blasfeo_hp_dtrsm_runu blasfeo_hp_cm_dtrsm_runu
#define blasfeo_hp_dtrsm_rutn blasfeo_hp_cm_dtrsm_rutn
#define blasfeo_hp_dtrsm_rutu blasfeo_hp_cm_dtrsm_rutu
#define blasfeo_dtrsm_llnn blasfeo_cm_dtrsm_llnn
#define blasfeo_dtrsm_llnu blasfeo_cm_dtrsm_llnu
#define blasfeo_dtrsm_lltn blasfeo_cm_dtrsm_lltn
#define blasfeo_dtrsm_lltu blasfeo_cm_dtrsm_lltu
#define blasfeo_dtrsm_lunn blasfeo_cm_dtrsm_lunn
#define blasfeo_dtrsm_lunu blasfeo_cm_dtrsm_lunu
#define blasfeo_dtrsm_lutn blasfeo_cm_dtrsm_lutn
#define blasfeo_dtrsm_lutu blasfeo_cm_dtrsm_lutu
#define blasfeo_dtrsm_rlnn blasfeo_cm_dtrsm_rlnn
#define blasfeo_dtrsm_rlnu blasfeo_cm_dtrsm_rlnu
#define blasfeo_dtrsm_rltn blasfeo_cm_dtrsm_rltn
#define blasfeo_dtrsm_rltu blasfeo_cm_dtrsm_rltu
#define blasfeo_dtrsm_runn blasfeo_cm_dtrsm_runn
#define blasfeo_dtrsm_runu blasfeo_cm_dtrsm_runu
#define blasfeo_dtrsm_rutn blasfeo_cm_dtrsm_rutn
#define blasfeo_dtrsm_rutu blasfeo_cm_dtrsm_rutu
#endif



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_hp_dgemm_tn blasfeo_hp_cm_dgemm_tn
#endif
#include <blasfeo_d_blasfeo_hp_api.h>



#define CACHE_LINE_EL D_CACHE_LINE_EL
#define L1_CACHE_EL D_L1_CACHE_EL
#define L2_CACHE_EL D_L2_CACHE_EL
#define LLC_CACHE_EL D_LLC_CACHE_EL
#define PS D_PS
#define M_KERNEL D_M_KERNEL
#define KC D_KC
#define NC D_NC
#define MC D_MC



#if 0
static void blasfeo_hp_dtrsm_lutn_m1(int m, int n, double alpha, double *A, int lda, double *B, int ldb, double *D, int ldd, double *pU, int sdu, double *dA)
	{

	int ii, jj;

	int ps = 4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(ii+4, A+(ii+0)*lda, lda, pU+0*sdu);
		kernel_dpack_tn_4_lib4(ii+8, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dpack_tn_4_lib4(ii+12, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_inv_12x4_lib4ccc4(ii, pU, sdu, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, sdu, dA+ii);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_inv_12x4_vs_lib4ccc4(ii, pU, sdu, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, sdu, dA+ii, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lutn_m1_left_4;
			}
		else if(m-ii<=8)
			{
			goto lutn_m1_left_8;
			}
		else
			{
			goto lutn_m1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(ii+4, A+(ii+0)*lda, lda, pU+0*sdu);
		kernel_dpack_tn_4_lib4(ii+8, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_inv_8x4_lib4ccc4(ii, pU, sdu, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, sdu, dA+ii);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_inv_8x4_vs_lib4ccc4(ii, pU, sdu, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, sdu, dA+ii, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lutn_m1_left_4;
			}
		else
			{
			goto lutn_m1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_inv_4x4_lib4ccc4(ii, pU, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, dA+ii);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_inv_4x4_vs_lib4ccc4(ii, pU, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, dA+ii, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto lutn_m1_left_4;
		}
#endif
	goto lutn_m1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lutn_m1_left_12:
	kernel_dpack_tn_4_lib4(ii+4, A+(ii+0)*lda, lda, pU+0*sdu);
	kernel_dpack_tn_4_lib4(ii+8, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ll_inv_12x4_vs_lib4ccc4(ii, pU, sdu, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, sdu, dA+ii, m-ii, n-jj);
		}
goto lutn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lutn_m1_left_8:
	kernel_dpack_tn_4_lib4(ii+4, A+(ii+0)*lda, lda, pU+0*sdu);
	kernel_dpack_tn_4_vs_lib4(m, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ll_inv_8x4_vs_lib4ccc4(ii, pU, sdu, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, sdu, dA+ii, m-ii, n-jj);
		}
goto lutn_m1_return;
#endif

lutn_m1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, A+ii*lda, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ll_inv_4x4_vs_lib4ccc4(ii, pU, D+jj*ldd, ldd, &alpha, B+jj*ldb+ii, ldb, D+jj*ldd+ii, ldd, pU+ii*ps, dA+ii, m-ii, n-jj);
		}
goto lutn_m1_return;

lutn_m1_return:
	return;

	return;

	}
#endif



static void blasfeo_hp_dtrsm_llnn_m2(int m, int n, double alpha, double *pA0, int sda0, double *dA0, double *B, int ldb, double *D, int ldd, double *pB0, int sdb0)
	{

#if 0

	double *pA = pA0;
	int sda = sda0;
	double *dA = dA0;
	double *pB = pB0;
	int sdb = sdb0;

	int ii, jj;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_ll_inv_12x4_lib44cc4(ii, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, sda, dA+ii);
			kernel_dpack_tn_4_lib4(12, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_ll_inv_12x4_vs_lib44cc4(ii, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, sda, dA+ii, m-ii, n-jj);
			kernel_dpack_tn_4_vs_lib4(12, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto llnn_2_left_4;
			}
		else if(m-ii<=8)
			{
			goto llnn_2_left_8;
			}
		else
			{
			goto llnn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_ll_inv_8x4_lib44cc4(ii, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, sda, dA+ii);
			kernel_dpack_tn_4_lib4(8, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_ll_inv_8x4_vs_lib44cc4(ii, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, sda, dA+ii, m-ii, n-jj);
			kernel_dpack_tn_4_vs_lib4(8, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto llnn_2_left_4;
			}
		else
			{
			goto llnn_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_ll_inv_4x4_lib44cc4(ii, pA+ii*sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, dA+ii);
			kernel_dpack_tn_4_lib4(4, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_ll_inv_4x4_vs_lib44cc4(ii, pA+ii*sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, dA+ii, m-ii, n-jj);
			kernel_dpack_tn_4_vs_lib4(4, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps, n-jj);
			}
		}
	if(ii<m)
		{
		goto llnn_2_left_4;
		}
#endif
	goto llnn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnn_2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_ll_inv_12x4_vs_lib44cc4(ii, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, sda, dA+ii, m-ii, n-jj);
		kernel_dpack_tn_4_vs_lib4(m-ii, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps, n-jj);
		}
goto llnn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnn_2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_ll_inv_8x4_vs_lib44cc4(ii, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, sda, dA+ii, m-ii, n-jj);
		kernel_dpack_tn_4_vs_lib4(m-ii, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps, n-jj);
		}
goto llnn_2_return;
#endif

llnn_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_ll_inv_4x4_vs_lib44cc4(ii, pA+ii*sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pA+ii*sda+ii*ps, dA+ii, m-ii, n-jj);
		kernel_dpack_tn_4_vs_lib4(m-ii, D+ii+jj*ldd, ldd, pB+jj*sdb+ii*ps, n-jj);
		}
goto llnn_2_return;

llnn_2_return:
	return;

#else

	double *pA = pB0;
	int sda = sdb0;
	double *pB = pA0;
	int sdb = sda0;
	double *dB = dA0;

	int ii, jj;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pA+ii*sda);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pA+(ii+4)*sda);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pA+(ii+8)*sda);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, sda, pA+ii*sda+jj*ps, sda, pB+jj*ps+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, sda, pA+ii*sda+jj*ps, sda, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pA+ii*sda, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pA+(ii+4)*sda, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pA+(ii+8)*sda, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_2_left_4;
			}
		if(n-ii<=8)
			{
			goto llnn_2_left_8;
			}
		else
			{
			goto llnn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pA+ii*sda);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pA+(ii+4)*sda);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, sda, pA+ii*sda+jj*ps, sda, pB+jj*ps+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, sda, pA+ii*sda+jj*ps, sda, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pA+ii*sda, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pA+(ii+4)*sda, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_2_left_4;
			}
		else
			{
			goto llnn_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pA+ii*sda);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib4(jj, pA+ii*sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, pA+ii*sda+jj*ps, pB+jj*ps+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(jj, pA+ii*sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, pA+ii*sda+jj*ps, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pA+ii*sda, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnn_2_left_4;
		}
#endif
	goto llnn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnn_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pA+ii*sda);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pA+(ii+4)*sda);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pA+(ii+8)*sda, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, sda, pA+ii*sda+jj*ps, sda, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pA+ii*sda, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pA+(ii+4)*sda, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pA+(ii+8)*sda, D+(ii+8)*ldd, ldd, n-ii-8);
goto llnn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnn_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pA+ii*sda);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pA+(ii+4)*sda, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, sda, pA+ii*sda+jj*ps, sda, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pA+ii*sda, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pA+(ii+4)*sda, D+(ii+4)*ldd, ldd, n-ii-4);
goto llnn_2_return;
#endif

llnn_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pA+ii*sda, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(jj, pA+ii*sda, pB+jj*sdb, &alpha, pA+ii*sda+jj*ps, pA+ii*sda+jj*ps, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pA+ii*sda, D+ii*ldd, ldd, n-ii);
goto llnn_2_return;

llnn_2_return:
	return;

#endif

	}



static void blasfeo_hp_dtrsm_rltn_m2(int m, int n, double alpha, double *pA0, int sda0, double *dA0, double *B, int ldb, double *D, int ldd, double *pA, int sda)
	{

	double *pB = pA0;
	double *dB = dA0;
	int sdb = sda0;

	int ii, jj;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(ii=0; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib44cc4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps, sda);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_2_left_4;
			}
		if(m-ii<=8)
			{
			goto rltn_2_left_8;
			}
		else
			{
			goto rltn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(ii=0; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib44cc4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps, sda);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_2_left_4;
			}
		else
			{
			goto rltn_2_left_8;
			}
		}
#else
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib44cc4(jj, pA+ii*sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pA+ii*sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps);
			}
		}
	if(ii<m)
		{
		goto rltn_2_left_4;
		}
#endif
	goto rltn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltn_2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps, sda, m-ii);
		}
goto rltn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltn_2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4(jj, pA+ii*sda, sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps, sda, m-ii);
		}
goto rltn_2_return;
#endif

rltn_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pA+ii*sda, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pA+ii*sda+jj*ps, m-ii);
		}
goto rltn_2_return;

rltn_2_return:
	return;

	}



static void blasfeo_hp_dtrsm_rutn_m2(int m, int n, double alpha, double *pA0, int sda0, double *dA0, double *B, int ldb, double *D, int ldd, double *pA, int sda)
	{

	double *pB = pA0;
	double *dB = dA0;
	int sdb = sda0;

	int ii, jj;

	int n4, nn4, idx;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4(0, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_12x4_lib44cc4(jj+nn4, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_2_left_4;
			}
		else if(m-ii<=8)
			{
			goto rutn_2_left_8;
			}
		else
			{
			goto rutn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4(0, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_8x4_lib44cc4(jj+nn4, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_2_left_4;
			}
		else
			{
			goto rutn_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4(0, pA+ii*sda+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_4x4_lib44cc4(jj+nn4, pA+ii*sda+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rutn_2_left_4;
		}
#endif
	goto rutn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutn_2_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4(0, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4(jj+nn4, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda, m-ii);
		}
	goto rutn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rutn_2_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4(0, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4(jj+nn4, pA+ii*sda+(idx+4)*ps, sda, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, sda, m-ii);
		}
	goto rutn_2_return;
#endif

rutn_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4(0, pA+ii*sda+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4(jj+nn4, pA+ii*sda+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pA+ii*sda+idx*ps, m-ii);
		}
	goto rutn_2_return;

rutn_2_return:
	return;

	}



void blasfeo_hp_dtrsm_llnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_llnn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;



#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto llnn_2;
		}
	else
		{
		goto llnn_1;
		}

	// never to get here
	return;



llnn_1:
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_1_left_4;
			}
		if(n-ii<=8)
			{
			goto llnn_1_left_8;
			}
		else
			{
			goto llnn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_1_left_4;
			}
		else
			{
			goto llnn_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnn_1_left_4;
		}
#endif
	goto llnn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
llnn_1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
goto llnn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
llnn_1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
goto llnn_1_return;
#endif

llnn_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnn_1_return;

llnn_1_return:
	return;



llnn_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to lower
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, sdb);
		}
	if(ii<m)
		{
		kernel_dpack_tt_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_2_left_4;
			}
		if(n-ii<=8)
			{
			goto llnn_2_left_8;
			}
		else
			{
			goto llnn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_2_left_4;
			}
		else
			{
			goto llnn_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnn_2_left_4;
		}
#endif
	goto llnn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnn_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-ii-8);
goto llnn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnn_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd, n-ii-4);
goto llnn_2_return;
#endif

llnn_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnn_2_return;

llnn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_llnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_llnu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;



#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto llnu_2;
		}
	else
		{
		goto llnu_1;
		}
	return;

	// never to get here
	return;



llnu_1:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_1_left_4;
			}
		if(n-ii<=8)
			{
			goto llnu_1_left_8;
			}
		else
			{
			goto llnu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_1_left_4;
			}
		else
			{
			goto llnu_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnu_1_left_4;
		}
#endif
	goto llnu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
llnu_1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
goto llnu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
llnu_1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_vs_lib4c44c(jj, pU, sdu, A+jj, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
goto llnu_1_return;
#endif

llnu_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnu_1_return;

llnu_1_return:
	return;



llnu_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to lower
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, sdb);
		}
	if(ii<m)
		{
		kernel_dpack_tt_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, sdb, m-ii);
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_2_left_4;
			}
		if(n-ii<=8)
			{
			goto llnu_2_left_8;
			}
		else
			{
			goto llnu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_2_left_4;
			}
		else
			{
			goto llnu_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnu_2_left_4;
		}
#endif
	goto llnu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnu_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-ii-8);
goto llnu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnu_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd, n-ii-4);
goto llnu_2_return;
#endif

llnu_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnu_2_return;

llnu_2_return:
	free(mem);
	return;


	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_lltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_lltn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu, idx, m4, mn4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;



#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto lunn_2;
		}
	else
		{
		goto lltn_1;
		}
	return;

	// never to get here
	return;



lltn_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nn_rl_inv_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nn_rl_inv_12x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltn_1_left_4;
			}
		else if(n-ii<=8)
			{
			goto lltn_1_left_8;
			}
		else
			{
			goto lltn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nn_rl_inv_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nn_rl_inv_8x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltn_1_left_4;
			}
		else
			{
			goto lltn_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nn_rl_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nn_rl_inv_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lltn_1_left_4;
		}
#endif
	goto lltn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lltn_1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nn_rl_inv_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nn_rl_inv_12x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
	goto lltn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lltn_1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nn_rl_inv_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nn_rl_inv_8x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
	goto lltn_1_return;
#endif

lltn_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nn_rl_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nn_rl_inv_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lltn_1_return;

lltn_1_return:
	return;



lunn_2:
	// XXX limits of ii and jj swapped !!!
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
		}
	if(ii<m)
		{
		kernel_dpack_tn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_12x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_2_left_4;
			}
		if(n-ii<=8)
			{
			goto lunn_2_left_8;
			}
		else
			{
			goto lunn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_8x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_2_left_4;
			}
		else
			{
			goto lunn_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_4x4_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunn_2_left_4;
		}
#endif
	goto lunn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunn_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-ii-8);
	goto lunn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunn_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd, n-ii-4);
	goto lunn_2_return;
#endif

lunn_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunn_2_return;

lunn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_lltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_lltu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu, idx, m4, mn4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto lunu_2;
		}
	else
		{
		goto lltu_1;
		}
	return;

	// never to get here
	return;


lltu_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nn_rl_one_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nn_rl_one_12x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltu_1_left_4;
			}
		else if(n-ii<=8)
			{
			goto lltu_1_left_8;
			}
		else
			{
			goto lltu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nn_rl_one_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nn_rl_one_8x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltu_1_left_4;
			}
		else
			{
			goto lltu_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nn_rl_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nn_rl_one_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lltu_1_left_4;
		}
#endif
	goto lltu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lltu_1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nn_rl_one_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nn_rl_one_12x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
	goto lltu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lltu_1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nn_rl_one_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nn_rl_one_8x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
	goto lltu_1_return;
#endif

lltu_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nn_rl_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nn_rl_one_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lltu_1_return;

lltu_1_return:
	return;


lunu_2:
	// XXX limits of ii and jj swapped !!!
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
		}
	if(ii<m)
		{
		kernel_dpack_tn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, m-ii);
		}

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_12x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_2_left_4;
			}
		if(n-ii<=8)
			{
			goto lunu_2_left_8;
			}
		else
			{
			goto lunu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_8x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_2_left_4;
			}
		else
			{
			goto lunu_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_4x4_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunu_2_left_4;
		}
#endif
	goto lunu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunu_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-ii-8);
	goto lunu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunu_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd, n-ii-4);
	goto lunu_2_return;
#endif

lunu_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunu_2_return;

lunu_2_return:
	free(mem);
	return;


	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_lunn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_lunn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu, idx, m4, mn4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


lunn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto lunn_2;
		}
	else
		{
		goto lunn_1;
		}

	// never to get here
	return;

lunn_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_12x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_1_left_4;
			}
		if(n-ii<=8)
			{
			goto lunn_1_left_8;
			}
		else
			{
			goto lunn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_8x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_1_left_4;
			}
		else
			{
			goto lunn_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunn_1_left_4;
		}
#endif
	goto lunn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunn_1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
	goto lunn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunn_1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
	goto lunn_1_return;
#endif

lunn_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunn_1_return;

lunn_1_return:
	return;


lunn_2:
	// XXX limits of ii and jj swapped !!!
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
		}
	if(ii<m)
		{
		kernel_dpack_tt_4_vs_lib4(m, A+ii*lda, lda, pB+ii*ps, sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_12x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_2_left_4;
			}
		if(n-ii<=8)
			{
			goto lunn_2_left_8;
			}
		else
			{
			goto lunn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_8x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_2_left_4;
			}
		else
			{
			goto lunn_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_inv_4x4_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunn_2_left_4;
		}
#endif
	goto lunn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunn_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-ii-8);
	goto lunn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunn_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd, n-ii-4);
	goto lunn_2_return;
#endif

lunn_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunn_2_return;

lunn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_lunu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_lunu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu, idx, m4, mn4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


lunu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto lunu_2;
		}
	else
		{
		goto lunu_1;
		}

lunu_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_12x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_1_left_4;
			}
		if(n-ii<=8)
			{
			goto lunu_1_left_8;
			}
		else
			{
			goto lunu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_8x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_1_left_4;
			}
		else
			{
			goto lunu_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunu_1_left_4;
		}
#endif
	goto lunu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunu_1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
	goto lunu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunu_1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4c44c(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
	goto lunu_1_return;
#endif

lunu_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps, pU+idx*ps, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunu_1_return;

lunu_1_return:
	return;


lunu_2:
	// XXX limits of ii and jj swapped !!!
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
		}
	if(ii<m)
		{
		kernel_dpack_tt_4_vs_lib4(m, A+ii*lda, lda, pB+ii*ps, sdb, m-ii);
		}

	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_12x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_2_left_4;
			}
		if(n-ii<=8)
			{
			goto lunu_2_left_8;
			}
		else
			{
			goto lunu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_8x4_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_2_left_4;
			}
		else
			{
			goto lunu_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_dtrsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_dtrsm_nt_ru_one_4x4_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunu_2_left_4;
		}
#endif
	goto lunu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunu_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-ii-8);
	goto lunu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunu_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4(0, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, sdu, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, sdu, pU+idx*ps, sdu, pB+idx*ps+idx*sdb, n-ii, 4);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd, n-ii-4);
	goto lunu_2_return;
#endif

lunu_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps, pB+(idx+4)*ps+idx*sdb, &alpha, pU+idx*ps, pU+idx*ps, pB+idx*ps+idx*sdb, n-ii, 4);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunu_2_return;

lunu_2_return:
	free(mem);
	return;


	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_lutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_lutn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj, ll;
	int ii0, ll0, jj0, iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *pT, *C1, *dA, *dB, *dT;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

#if defined(TARGET_GENERIC)
	double pU_stack[M_KERNEL*K_MAX_STACK];
	double pd_stack[K_MAX_STACK];
#else
	ALIGNED( double pU_stack[M_KERNEL*K_MAX_STACK], 64 );
	ALIGNED( double pd_stack[K_MAX_STACK], 64 );
#endif
	int sdu_stack = K_MAX_STACK;
	int k4 = (k0+3)/4*4;

	struct blasfeo_pm_dmat tA, tB, tT;
	int sda, sdb, sdt;
	int tA_size, tB_size, tT_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU;
	int sdu;
	int pU_size;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	double d_1 = 1.0;
	double d_minvalpha = -1.0/alpha;

	double *C;
	int ldc;
	struct blasfeo_dmat *sC;


lutn:
//	goto lutn_m1;
//	goto llnn_2;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<300 & n<300 & k0<=K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & n<64 & k0<=K_MAX_STACK)
#else
	if(m<12 & n<12 & k0<=K_MAX_STACK)
#endif
		{
		goto lutn_n1;
		}
	else
		{
		goto llnn_2;
		}

	// never to get here
	return;

#if 0
lutn_m1:

	pU = pU_stack;
	sdu = sdu_stack;
	dA = pd_stack;

	nc = 256;

	// no block over m for now
	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	// block over n
	for(jj=0; jj<n; jj+=nleft)
		{
		nleft = n-jj<nc ? n-jj : nc;

		blasfeo_hp_dtrsm_lutn_m1(m, nleft, alpha, A, lda, B+jj*ldb, ldb, D+jj*ldd, ldd, pU, sdu, dA);
		}

	return;
#endif


lutn_n1:
	// XXX limits of ii and jj swapped !!!
	pU = pU_stack;
	sdu = sdu_stack;
	dA = pd_stack;

	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_12x4_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nn_ru_inv_12x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutn_n1_left_4;
			}
		if(n-ii<=8)
			{
			goto lutn_n1_left_8;
			}
		else
			{
			goto lutn_n1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_8x4_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nn_ru_inv_8x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutn_n1_left_4;
			}
		else
			{
			goto lutn_n1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_4x4_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_dtrsm_nn_ru_inv_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lutn_n1_left_4;
		}
#endif
	goto lutn_n1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lutn_n1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_12x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
goto lutn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lutn_n1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_8x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
goto lutn_n1_return;
#endif

lutn_n1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto lutn_n1_return;

lutn_n1_return:
	return;


llnn_2:
#if ! defined(TARGET_X64_INTEL_SKYLAKE_X)

#if 1

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

	// these must all be multiple of ps !!!
//	mc0 = 12;
//	nc0 = 4;
//	kc0 = 8;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
//	kc = k<kc0 ? k : kc0;
	kc = m<kc0 ? m : kc0;

	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
	tT_size = blasfeo_pm_memsize_dmat(ps, kc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
	tT_size = (tT_size + 4096 - 1) / 4096 * 4096;
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+tT_size+2*4096);
		}
	else
		{
		mem = blasfeo_get_buffer();
		}
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	blasfeo_pm_create_dmat(ps, kc0, kc0, &tT, (void *) mem_align);
	mem_align += tT_size;

	mem_align += 4096-4*128;
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	pA = tA.pA;
	pB = tB.pA;
	pT = tT.pA;
	dT = tT.dA;

	for(ii=0; ii<m; ii+=mleft)
		{

		mleft = m-ii<mc ? m-ii : mc;

		C = B;
		ldc = ldb;
		sC = sB;

#if 0

		if(ii>0)
			{

			blasfeo_hp_dgemm_tn(mleft, n, ii, d_minvalpha, sA, 0, ii, sD, 0, 0, d_1, sC, ii, 0, sD, ii, 0);

#else
		for(ll=0; ll<ii; ll+=kleft)
			{

			kleft = ii-ll<kc ? ii-ll : kc;

			sda = (kleft+4-1)/4*4; // XXX
			sdb = (kleft+4-1)/4*4; // XXX

			// pack and tran A
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
			kernel_dpack_buffer_ft(kleft, mleft, A+ll+ii*lda, lda, pA, sda);
#endif

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
				kernel_dpack_buffer_ft(kleft, nleft, D+ll+jj*ldd, ldd, pB, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, d_minvalpha, pA, sda, pB, sdb, d_1, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);

				}

#endif
			C = D;
			ldc = ldd;
			sC = sD;

			}

		for(ll=0; ll<mleft; ll+=kleft)
			{

			kleft = mleft-ll<kc ? mleft-ll : kc;

			sda = (kleft+4-1)/4*4; // XXX
			sdb = (kleft+4-1)/4*4; // XXX
			sdt = (kleft+4-1)/4*4; // XXX

			// pack and tran A
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
			kernel_dpack_buffer_ut(kleft, A+ii+ll+(ii+ll)*lda, lda, pT, sdt);
			kernel_dpack_buffer_ft(kleft, mleft-ll-kleft, A+ii+ll+(ii+ll+kleft)*lda, lda, pA, sda);
#endif
			// diag A
			for(iii=0; iii<kleft; iii++)
				dT[iii] = 1.0/A[ii+ll+iii+(ii+ll+iii)*lda];

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				blasfeo_hp_dtrsm_llnn_m2(kleft, nleft, alpha, pT, sdt, dT, C+ii+ll+jj*ldc, ldc, D+ii+ll+jj*ldd, ldd, pB, sdb);
				blasfeo_hp_dgemm_nt_m2(mleft-ll-kleft, nleft, kleft, d_minvalpha, pA, sda, pB, sdb, d_1, C+ii+ll+kleft+jj*ldc, ldc, D+ii+ll+kleft+jj*ldd, ldd);

				}

			C = D;
			ldc = ldd;
			sC = sD;

			}

		}
	
	free(mem);
	return;

#else

	// cache blocking alg

	mc0 = NC; // XXX
	nc0 = MC; // XXX
	kc0 = KC;

	// these must all be multiple of ps !!!
//	mc0 = 4;
//	nc0 = 12;
//	kc0 = 8;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
//	kc = k<kc0 ? k : kc0;
	kc = m<kc0 ? m : kc0;

	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
	tT_size = blasfeo_pm_memsize_dmat(ps, kc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
	tT_size = (tT_size + 4096 - 1) / 4096 * 4096;
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+tT_size+2*4096);
		}
	else
		{
		mem = blasfeo_get_buffer();
		}
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	blasfeo_pm_create_dmat(ps, kc0, kc0, &tT, (void *) mem_align);
	mem_align += tT_size;

	mem_align += 4096-4*128;
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	pA = tA.pA;
	pB = tB.pA;
	pT = tT.pA;
	dT = tT.dA;

	for(jj=0; jj<n; jj+=nleft)
		{

		nleft = n-jj<nc ? n-jj : nc;

		C = B;
		ldc = ldb;

		for(ll=0; ll<m; ll+=kleft)
			{

			kleft = m-ll<kc ? m-ll : kc;

			sda = (kleft+4-1)/4*4; // XXX
			sdb = (kleft+4-1)/4*4; // XXX
			sdt = (kleft+4-1)/4*4; // XXX

			for(ii=0; ii<kleft; ii+=mleft)
				{

				mleft = kleft-ii<mc ? kleft-ii : mc;

				// pack and tran A
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
				kernel_dpack_buffer_ut(mleft, A+ll+ii+(ll+ii)*lda, lda, pT, sdt);
				kernel_dpack_buffer_ft(ii, mleft, A+ll+(ll+ii)*lda, lda, pA, sda);
#endif
				// diag A
				for(iii=0; iii<mleft; iii++)
					dT[iii] = 1.0/A[ll+ii+iii+(ll+ii+iii)*lda];

				blasfeo_hp_dgemm_nt_n2(mleft, nleft, ii, d_minvalpha, pA, sda, pB, sdb, d_1, C+ll+ii+jj*ldc, ldc, D+ll+ii+jj*ldd, ldd);
				blasfeo_hp_dtrsm_llnn_m2(mleft, nleft, alpha, pT, sdt, dT, D+ll+ii+jj*ldd, ldd, D+ll+ii+jj*ldd, ldd, pB+ii*ps, sdb);

				}

			for(ii=ll+kleft; ii<m; ii+=mleft)
				{

				mleft = m-ii<mc ? m-ii : mc;

				// pack and tran A
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
				kernel_dpack_buffer_ft(kleft, mleft, A+ll+ii*lda, lda, pA, sda);
#endif

				blasfeo_hp_dgemm_nt_n2(mleft, nleft, kleft, d_minvalpha, pA, sda, pB, sdb, d_1, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);

				}

			C = D;
			ldc = ldd;

			}

		}

	free(mem);
	return;

#endif

#else

	m1 = (m+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, n, &tB, (void *) (mem_align+tA_size));

	pA = tA.pA;
	sda = tA.cn;
	dA = tA.dA;
	pB = tB.pA;
	sdb = tB.cn;

	// pack A
	// upper to lower
	kernel_dpack_buffer_ut(m, A, lda, pA, sda);
	// diag A
	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	blasfeo_hp_dtrsm_llnn_m2(m, n, alpha, pA, sda, dA, B, ldb, D, ldd, pB, sdb);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_lutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_lutu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


lutu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto llnu_2;
		}
	else
		{
		goto lutu_1;
		}

	// never to get here
	return;


lutu_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_one_12x4_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_dtrsm_nn_ru_one_12x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutu_1_left_4;
			}
		if(n-ii<=8)
			{
			goto lutu_1_left_8;
			}
		else
			{
			goto lutu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_one_8x4_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_dtrsm_nn_ru_one_8x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutu_1_left_4;
			}
		else
			{
			goto lutu_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_one_4x4_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_dtrsm_nn_ru_one_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lutu_1_left_4;
		}
#endif
	goto lutu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lutu_1_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nn_ru_one_12x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-(ii+8));
goto lutu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lutu_1_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nn_ru_one_8x4_vs_lib4c44c(jj, pU, sdu, A+jj*lda, lda, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+ps*sdu, D+(ii+4)*ldd, ldd, n-(ii+4));
goto lutu_1_return;
#endif

lutu_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nn_ru_one_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps, pU+jj*ps, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto lutu_1_return;

lutu_1_return:
	return;


llnu_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to lower
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
		}
	if(ii<m)
		{
		kernel_dpack_tn_4_vs_lib4(m, A+ii*lda, lda, pB+ii*sdb, m-ii);
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_2_left_4;
			}
		if(n-ii<=8)
			{
			goto llnu_2_left_8;
			}
		else
			{
			goto llnu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_2_left_4;
			}
		else
			{
			goto llnu_2_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb);
			}
		if(jj<m)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb, n-ii, m-jj);
			}
		kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnu_2_left_4;
		}
#endif
	goto llnu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnu_2_left_12:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-ii-8);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+8*sdu, D+(ii+8)*ldd, ldd, n-ii-8);
goto llnu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
llnu_2_left_8:
	kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-ii-4);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
	kernel_dunpack_nt_4_vs_lib4(m, pU+4*sdu, D+(ii+4)*ldd, ldd, n-ii-4);
goto llnu_2_return;
#endif

llnu_2_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib4(jj, pU, pB+jj*sdb, &alpha, pU+jj*ps, pU+jj*ps, pB+jj*ps+jj*sdb, n-ii, m-jj);
		}
	kernel_dunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnu_2_return;

llnu_2_return:
	free(mem);
	return;


	// never to get here
	return;

	}



// TODO optimize for cortex A57 !!!!!
void blasfeo_hp_dtrsm_rlnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_rlnn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj, ll;
	int ii0, ll0, jj0, iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *pT, *C1, *dA, *dB, *dT;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

#if defined(TARGET_GENERIC)
	double pU_stack[M_KERNEL*K_MAX_STACK];
	double pd_stack[K_MAX_STACK];
#else
	ALIGNED( double pU_stack[M_KERNEL*K_MAX_STACK], 64 );
	ALIGNED( double pd_stack[K_MAX_STACK], 64 );
#endif
	int sdu_stack = K_MAX_STACK;
	int k4 = (k0+3)/4*4;

	struct blasfeo_pm_dmat tA, tB, tT;
	int sda, sdb, sdt;
	int tA_size, tB_size, tT_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU;
	int sdu;
	int pU_size;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	double d_1 = 1.0;
	double d_minvalpha = -1.0/alpha;

	double *C;
	int ldc;


//	goto rlnn_1;
//	goto rutn_2;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<=300 & n<=300 & k0<=K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & n<64 & k0<=K_MAX_STACK)
#else
	if(m<12 & n<12 & k0<=K_MAX_STACK)
#endif
		{
		goto rlnn_1;
		}
	else
		{
		goto rutn_2;
		}

	// never to get here
	return;


rlnn_1:
	pU = pU_stack;
	sdu = sdu_stack;
	dA = pd_stack;

	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nn_rl_inv_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nn_rl_inv_12x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnn_1_left_4;
			}
		else if(m-ii<=8)
			{
			goto rlnn_1_left_8;
			}
		else
			{
			goto rlnn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nn_rl_inv_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nn_rl_inv_8x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnn_1_left_4;
			}
		else
			{
			goto rlnn_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nn_rl_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nn_rl_inv_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rlnn_1_left_4;
		}
#endif
	goto rlnn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rlnn_1_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nn_rl_inv_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nn_rl_inv_12x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rlnn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rlnn_1_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nn_rl_inv_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nn_rl_inv_8x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rlnn_1_return;
#endif

rlnn_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nn_rl_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nn_rl_inv_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	goto rlnn_1_return;

rlnn_1_return:
	return;


rutn_2:
#if ! defined(TARGET_X64_INTEL_SKYLAKE_X)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

	// these must all be multiple of ps !!!
//	mc0 = 20;
//	nc0 = 4;
//	kc0 = 8;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
//	kc = k<kc0 ? k : kc0;
	kc = n<kc0 ? n : kc0;

	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
	tT_size = blasfeo_pm_memsize_dmat(ps, nc0, nc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
	tT_size = (tT_size + 4096 - 1) / 4096 * 4096;
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+tT_size+2*4096);
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

	blasfeo_pm_create_dmat(ps, nc0, nc0, &tT, (void *) mem_align);
	mem_align += tT_size;

	pA = tA.pA;
	pB = tB.pA;
	pT = tT.pA;
	dT = tT.dA;

	for(ii=0; ii<m; ii+=mleft)
		{

		mleft = m-ii<mc ? m-ii : mc;

		C = B;
		ldc = ldb;

		for(ll=0; ll<n; ll+=kleft)
			{

			kleft = n-ll<kc ? n-ll : kc;
			ll0 = n-ll-kleft;

			sda = (kleft+4-1)/4*4; // XXX
			sdb = (kleft+4-1)/4*4; // XXX

			for(jj=0; jj<kleft; jj+=nleft)
				{

				nleft = kleft-jj<nc ? kleft-jj : nc;
				jj0 = kleft-jj-nleft;

				sdt = (nleft+4-1)/4*4; // XXX

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
				kernel_dpack_buffer_lt(nleft, A+ll0+jj0+(ll0+jj0)*lda, lda, pT, sdt);
				kernel_dpack_buffer_ft(jj, nleft, A+ll0+jj0+nleft+(ll0+jj0)*lda, lda, pB, sdb);
#endif
				// diag B
				for(iii=0; iii<nleft; iii++)
					dT[iii] = 1.0/A[ll0+jj0+iii+(ll0+jj0+iii)*lda];

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, jj, d_minvalpha, pA+(jj0+nleft)*ps, sda, pB, sdb, d_1, C+ii+(ll0+jj0)*ldc, ldc, D+ii+(ll0+jj0)*ldd, ldd);
				blasfeo_hp_dtrsm_rutn_m2(mleft, nleft, alpha, pT, sdt, dT, D+ii+(ll0+jj0)*ldd, ldd, D+ii+(ll0+jj0)*ldd, ldd, pA+jj0*ps, sda);

				}

			for(jj=0; jj<ll0; jj+=nleft)
				{

				nleft = ll0-jj<nc ? ll0-jj : nc;
				jj0 = ll0-jj-nleft;

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
				kernel_dpack_buffer_ft(kleft, nleft, A+ll0+jj0*lda, lda, pB, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, d_minvalpha, pA, sda, pB, sdb, d_1, C+ii+jj0*ldc, ldc, D+ii+jj0*ldd, ldd);

				}

			C = D;
			ldc = ldd;

			}

		}
	
	free(mem);
	return;

#else

	m1 = (m+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, n, &tB, (void *) (mem_align+tA_size));

	pA = tA.pA;
	sda = tA.cn;
	dA = tA.dA;
	pB = tB.pA;
	sdb = tB.cn;

	// pack A
	// lower to upper
	kernel_dpack_buffer_lt(n, A, lda, pA, sda);
	// diag A
	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	blasfeo_hp_dtrsm_rutn_m2(m, n, alpha, pA, sda, dA, B, ldb, D, ldd, pB, sdb);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_rlnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_rlnu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


rlnu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>300 | n>300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
//		pack_tran = 1;
		goto rutu_2;
		}
	else
		{
		goto rlnu_1;
		}

	// never to get here
	return;


rlnu_1:
	pU = pU0;
	sdu = sdu0;

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nn_rl_one_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nn_rl_one_12x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnu_1_left_4;
			}
		else if(m-ii<=8)
			{
			goto rlnu_1_left_8;
			}
		else
			{
			goto rlnu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nn_rl_one_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nn_rl_one_8x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnu_1_left_4;
			}
		else
			{
			goto rlnu_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nn_rl_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nn_rl_one_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rlnu_1_left_4;
		}
#endif
	goto rlnu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rlnu_1_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nn_rl_one_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nn_rl_one_12x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rlnu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rlnu_1_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nn_rl_one_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nn_rl_one_8x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rlnu_1_return;
#endif

rlnu_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nn_rl_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nn_rl_one_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	goto rlnu_1_return;

rlnu_1_return:
	return;


rutu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
		}
	if(ii<n)
		{
		kernel_dpack_tn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, n-ii);
		}

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_12x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_2_left_4;
			}
		else if(m-ii<=8)
			{
			goto rutu_2_left_8;
			}
		else
			{
			goto rutu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_8x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_8x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_2_left_4;
			}
		else
			{
			goto rutu_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_4x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rutu_2_left_4;
		}
#endif
	goto rutu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutu_2_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutu_2_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutu_2_return;
#endif

rutu_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	goto rutu_2_return;

rutu_2_return:
	free(mem);
	return;


	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_rltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_rltn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj, ll;
	int ii0, ll0, jj0, iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *pT, *C1, *dA, *dB, *dT;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const int ps = 4; // XXX TODO fix once implemented missing kernels !!!
#else
	const int ps = PS;
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

#if defined(TARGET_GENERIC)
	double pU_stack[M_KERNEL*K_MAX_STACK];
	double pd_stack[K_MAX_STACK];
#else
	ALIGNED( double pU_stack[M_KERNEL*K_MAX_STACK], 64 );
	ALIGNED( double pd_stack[K_MAX_STACK], 64 );
#endif
	int sdu_stack = K_MAX_STACK;
	int k4 = (k0+3)/4*4;

	struct blasfeo_pm_dmat tA, tB, tT;
	int sda, sdb, sdt;
	int tA_size, tB_size, tT_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU;
	int sdu;
	int pU_size;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	double d_1 = 1.0;
	double d_minvalpha = -1.0/alpha;

	double *C;
	int ldc;


//	goto rltn_1;
//	goto rltn_2;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<200 & n<200 & k0<=K_MAX_STACK)
//	if(m<256 & n<256 & k0<=K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<64 & n<64 & k0<=K_MAX_STACK)
#else
	if(m<12 & n<12 & k0<=K_MAX_STACK)
#endif
		{
		goto rltn_1;
		}
	else
		{
		goto rltn_2;
		}

	// never to get here
	return;


rltn_1:
	pU = pU_stack;
	sdu = sdu_stack;
	dA = pd_stack;

	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_1_left_4;
			}
		if(m-ii<=8)
			{
			goto rltn_1_left_8;
			}
		else
			{
			goto rltn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(ii=0; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_1_left_4;
			}
		else
			{
			goto rltn_1_left_8;
			}
		}
#else
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		goto rltn_1_left_4;
		}
#endif
	goto rltn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rltn_1_left_12:
		for(jj=0; jj<n; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
			kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
			}
	goto rltn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltn_1_left_8:
		for(jj=0; jj<n; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
			kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
			}
	goto rltn_1_return;
#endif

rltn_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, m-ii);
		}
goto rltn_1_return;

rltn_1_return:
	return;



rltn_2:
#if ! defined(TARGET_X64_INTEL_SKYLAKE_X)

	// cache blocking alg

	mc0 = MC;
	nc0 = NC;
	kc0 = KC;

	// these must all be multiple of ps !!!
//	mc0 = 12;
//	nc0 = 4;
//	kc0 = 8;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
//	kc = k<kc0 ? k : kc0;
	kc = n<kc0 ? n : kc0;

	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
	tT_size = blasfeo_pm_memsize_dmat(ps, nc0, nc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
	tT_size = (tT_size + 4096 - 1) / 4096 * 4096;
	if(blasfeo_is_init()==0)
		{
		blasfeo_malloc(&mem, tA_size+tB_size+tT_size+2*4096);
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

	blasfeo_pm_create_dmat(ps, nc0, nc0, &tT, (void *) mem_align);
	mem_align += tT_size;

	pA = tA.pA;
	pB = tB.pA;
	pT = tT.pA;
	dT = tT.dA;

	for(ii=0; ii<m; ii+=mleft)
		{

		mleft = m-ii<mc ? m-ii : mc;

		C = B;
		ldc = ldb;

		for(ll=0; ll<n; ll+=kleft)
			{

			kleft = n-ll<kc ? n-ll : kc;

			sda = (kleft+4-1)/4*4; // XXX
			sdb = (kleft+4-1)/4*4; // XXX

			for(jj=0; jj<kleft; jj+=nleft)
				{

				nleft = kleft-jj<nc ? kleft-jj : nc;

				sdt = (nleft+4-1)/4*4; // XXX

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
				kernel_dpack_buffer_fn(nleft, jj, A+ll+jj+ll*lda, lda, pB, sdb);
				kernel_dpack_buffer_ln(nleft, A+ll+jj+(ll+jj)*lda, lda, pT, sdt);
#endif
				// diag B
				for(iii=0; iii<nleft; iii++)
					dT[iii] = 1.0/A[ll+jj+iii+(ll+jj+iii)*lda];

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, jj, d_minvalpha, pA, sda, pB, sdb, d_1, C+ii+(ll+jj)*ldc, ldc, D+ii+(ll+jj)*ldd, ldd);
				blasfeo_hp_dtrsm_rltn_m2(mleft, nleft, alpha, pT, sdt, dT, D+ii+(ll+jj)*ldd, ldd, D+ii+(ll+jj)*ldd, ldd, pA+jj*ps, sda);

				}

			for(jj=ll+kleft; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack and tran B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
#else
				kernel_dpack_buffer_fn(nleft, kleft, A+jj+ll*lda, lda, pB, sdb);
#endif

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, d_minvalpha, pA, sda, pB, sdb, d_1, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);

				}

			C = D;
			ldc = ldd;

			}

		}
	
	free(mem);
	return;

#else

	m1 = (m+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, m, n, &tB, (void *) (mem_align+tA_size));

	pA = tA.pA;
	sda = tA.cn;
	dA = tA.dA;
	pB = tB.pA;
	sdb = tB.cn;

	// pack A
	// lower to lower
	kernel_dpack_buffer_ln(n, A, lda, pA, sda);
	// diag A
	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	blasfeo_hp_dtrsm_rltn_m2(m, n, alpha, pA, sda, dA, B, ldb, D, ldd, pB, sdb);

	free(mem);
	return;

#endif

	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_rltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_rltu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


rltu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
//		pack_tran = 0;
		goto rltu_2;
		}
	else
		{
		goto rltu_1;
		}

	// never to get here
	return;


rltu_1:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_1_left_4;
			}
		if(m-ii<=8)
			{
			goto rltu_1_left_8;
			}
		else
			{
			goto rltu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(ii=0; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_1_left_4;
			}
		else
			{
			goto rltu_1_left_8;
			}
		}
#else
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		goto rltu_1_left_4;
		}
#endif
	goto rltu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rltu_1_left_12:
		for(jj=0; jj<n; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
			kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
			}
	goto rltu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rltu_1_left_8:
		for(jj=0; jj<n; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4cccc(jj, pU, sdu, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
			kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
			}
	goto rltu_1_return;
#endif

rltu_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, m-ii);
		}
goto rltu_1_return;

rltu_1_return:
	return;



rltu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

//	if(pack_tran) // upper to lower
//		{
//		for(ii=0; ii<n-3; ii+=4)
//			{
//			kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
//			}
//		if(ii<n)
//			{
//			kernel_dpack_tn_4_vs_lib4(n, A+ii*lda, lda, pB+ii*sdb, n-ii);
//			}
//		}
//	else // lower to lower
//		{
//		for(ii=0; ii<n-3; ii+=4)
//			{
//			kernel_dpack_nn_4_lib4(ii+4, A+ii, lda, pB+ii*sdb);
//			}
//		if(ii<n)
//			{
//			kernel_dpack_nn_4_vs_lib4(n, A+ii, lda, pB+ii*sdb, n-ii);
//			}
//		}
	// lower to lower
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb);
		}
	if(ii<n)
		{
		kernel_dpack_tt_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb, n-ii);
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
#if 0
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+ii+jj*ps, sdu, pB+jj*ps+jj*sdb);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+ii+jj*ps, sdu, pU+ii+jj*ps, sdu, pB+jj*ps+jj*sdb, m-ii, n-jj);
			}
		kernel_dunpack_nn_12_lib4(n, pU, sdu, D+ii, ldd);
#else
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
#endif
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_2_left_4;
			}
		if(m-ii<=8)
			{
			goto rltu_2_left_8;
			}
		else
			{
			goto rltu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_2_left_4;
			}
		else
			{
			goto rltu_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		goto rltu_2_left_4;
		}
#endif
	goto rltu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltu_2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
		kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto rltu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltu_2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
		kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto rltu_2_return;
#endif

rltu_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, m-ii);
		}
goto rltu_2_return;

rltu_2_return:
	free(mem);
	return;

	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_runn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_runn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


runn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
//		pack_tran = 1;
		goto rltn_2;
		}
	else
		{
		goto runn_1;
		}

	// never to get here
	return;


runn_1:
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_12x4_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_dpack_nn_12_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runn_1_left_4;
			}
		else if(m-ii<=8)
			{
			goto runn_1_left_8;
			}
		else
			{
			goto runn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(ii=0; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_8x4_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ru_inv_8x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_dpack_nn_8_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runn_1_left_4;
			}
		else
			{
			goto runn_1_left_8;
			}
		}
#else
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_4x4_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		goto runn_1_left_4;
		}
#endif
	goto runn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
runn_1_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
		kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto runn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
runn_1_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_8x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
		kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto runn_1_return;
#endif

runn_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, m-ii);
		}
goto runn_1_return;

runn_1_return:
	return;


rltn_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to lower
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
		}
	if(ii<n)
		{
		kernel_dpack_tn_4_vs_lib4(n, A+ii*lda, lda, pB+ii*sdb, n-ii);
		}

	for(ii=0; ii<n; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(ii=0; ii<m-11; ii+=12)
		{
#if 0
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+ii+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+ii+jj*ps, sdu, pU+ii+jj*ps, sdu, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
			}
		kernel_dunpack_nn_12_lib4(n, pU, sdu, D+ii, ldd);
#else
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
#endif
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_2_left_4;
			}
		if(m-ii<=8)
			{
			goto rltn_2_left_8;
			}
		else
			{
			goto rltn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(ii=0; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_2_left_4;
			}
		else
			{
			goto rltn_2_left_8;
			}
		}
#else
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		goto rltn_2_left_4;
		}
#endif
	goto rltn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltn_2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto rltn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltn_2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto rltn_2_return;
#endif

rltn_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, m-ii);
		}
goto rltn_2_return;

rltn_2_return:
	free(mem);
	return;


	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_runu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_runu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


runu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
//		pack_tran = 1;
		goto rltu_2;
		}
	else
		{
		goto runu_1;
		}

	// never to get here
	return;

runu_1:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_one_12x4_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ru_one_12x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_dpack_nn_12_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runu_1_left_4;
			}
		else if(m-ii<=8)
			{
			goto runu_1_left_8;
			}
		else
			{
			goto runu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(ii=0; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_one_8x4_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ru_one_8x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_dpack_nn_8_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runu_1_left_4;
			}
		else
			{
			goto runu_1_left_8;
			}
		}
#else
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_one_4x4_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ru_one_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		goto runu_1_left_4;
		}
#endif
	goto runu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
runu_1_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ru_one_12x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
		kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto runu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
runu_1_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ru_one_8x4_vs_lib4cccc(jj, pU, sdu, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
		kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto runu_1_return;
#endif

runu_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ru_one_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, m-ii);
		}
goto runu_1_return;

runu_1_return:
	return;


rltu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
		}
	if(ii<n)
		{
		kernel_dpack_tn_4_vs_lib4(n, A+ii*lda, lda, pB+ii*sdb, n-ii);
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
#if 0
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+jj*ps, sdu, pU+ii+jj*ps, sdu, pB+jj*ps+jj*sdb);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4(jj, pU, sdu, pB+jj*sdb, &alpha, pU+ii+jj*ps, sdu, pU+ii+jj*ps, sdu, pB+jj*ps+jj*sdb, m-ii, n-jj);
			}
		kernel_dunpack_nn_12_lib4(n, pU, sdu, D+ii, ldd);
#else
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb);
			kernel_dpack_nn_12_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
#endif
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_2_left_4;
			}
		if(m-ii<=8)
			{
			goto rltu_2_left_8;
			}
		else
			{
			goto rltu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb);
			kernel_dpack_nn_8_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps, sdu);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_2_left_4;
			}
		else
			{
			goto rltu_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb);
			kernel_dpack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		if(jj<n)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
//			kernel_dpack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps);
			}
		}
	if(ii<m)
		{
		goto rltu_2_left_4;
		}
#endif
	goto rltu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltu_2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
		kernel_dpack_nn_12_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto rltu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
rltu_2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4(jj, pU, sdu, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
		kernel_dpack_nn_8_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, sdu, m-ii);
		}
goto rltu_2_return;
#endif

rltu_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps+jj*sdb, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps, m-ii);
		}
goto rltu_2_return;

rltu_2_return:
	free(mem);
	return;

	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_rutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_rutn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


rutn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		goto rutn_2;
		}
	else
		{
		goto rutn_1;
		}

	// never to get here
	return;

rutn_1:
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_12x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_1_left_4;
			}
		else if(m-ii<=8)
			{
			goto rutn_1_left_8;
			}
		else
			{
			goto rutn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_8x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_1_left_4;
			}
		else
			{
			goto rutn_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rutn_1_left_4;
		}
#endif
	goto rutn_1_return;

rutn_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	goto rutn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutn_1_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutn_1_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutn_1_return;
#endif

rutn_1_return:
	return;


rutn_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to upper
#if 1
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
		}
	if(ii<n)
		{
		kernel_dpack_tt_4_vs_lib4(n, A+ii*lda, lda, pB+ii*ps, sdb, n-ii);
		}
#else
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
		}
	if(ii<n)
		{
		kernel_dpack_nn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, n-ii);
		}
#endif

	for(ii=0; ii<n; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_12x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_2_left_4;
			}
		else if(m-ii<=8)
			{
			goto rutn_2_left_8;
			}
		else
			{
			goto rutn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_8x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_2_left_4;
			}
		else
			{
			goto rutn_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4(0, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_inv_4x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rutn_2_left_4;
		}
#endif
	goto rutn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutn_2_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutn_2_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutn_2_return;
#endif

rutn_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4(0, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, dB+idx, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	goto rutn_2_return;

rutn_2_return:
	free(mem);
	return;

	// never to get here
	return;

	}



void blasfeo_hp_dtrsm_rutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dtrsm_rutu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	double *A = sA->pA + ai + aj*lda;
	double *B = sB->pA + bi + bj*ldb;
	double *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *dA, *pB, *dB;
	int sdu;

	int idx, nn4, n4;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


rutu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
//		pack_tran = 0;
		goto rutu_2;
		}
	else
		{
		goto rutu_1;
		}

	// never to get here
	return;


rutu_1:
	pU = pU0;
	sdu = sdu0;

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_12x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_1_left_4;
			}
		else if(m-ii<=8)
			{
			goto rutu_1_left_8;
			}
		else
			{
			goto rutu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_8x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_1_left_4;
			}
		else
			{
			goto rutu_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rutu_1_left_4;
		}
#endif
	goto rutu_1_return;

rutu_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	goto rutu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutu_1_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutu_1_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4cccc(0, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps, sdu, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutu_1_return;
#endif

rutu_1_return:
	return;


rutu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

#if 1
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
		}
	if(ii<n)
		{
		kernel_dpack_tt_4_vs_lib4(n, A+ii*lda, lda, pB+ii*ps, sdb, n-ii);
		}
#else
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
		}
	if(ii<n)
		{
		kernel_dpack_nn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, n-ii);
		}
#endif

	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
			kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_12x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps);
			kernel_dpack_nn_12_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_2_left_4;
			}
		else if(m-ii<=8)
			{
			goto rutu_2_left_8;
			}
		else
			{
			goto rutu_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_8x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
			kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_8x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps);
			kernel_dpack_nn_8_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_2_left_4;
			}
		else
			{
			goto rutu_2_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_dtrsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
			kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_dtrsm_nt_ru_one_4x4_lib44cc4(jj+nn4, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps);
			kernel_dpack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps);
			}
		}
	if(ii<m)
		{
		goto rutu_2_left_4;
		}
#endif
	goto rutu_2_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutu_2_left_12:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
		kernel_dpack_nn_12_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, 4);
		kernel_dpack_nn_12_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutu_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutu_2_left_8:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib44cc4(0, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
		kernel_dpack_nn_8_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_8x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, sdu, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, 4);
		kernel_dpack_nn_8_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, sdu, m-ii);
		}
	goto rutu_2_return;
#endif

rutu_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, nn4);
		kernel_dpack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_dtrsm_nt_ru_one_4x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps, pB+idx*sdb+(idx+4)*ps, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps, m-ii, 4);
		kernel_dpack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps, m-ii);
		}
	goto rutu_2_return;

rutu_2_return:
	free(mem);
	return;

	// never to get here
	return;

	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dtrsm_llnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_llnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_llnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_llnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_lltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_lltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_lunn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lunn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_lunu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lunu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_lutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_lutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_rlnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rlnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_rlnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rlnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_rltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_rltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_runn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_runn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_runu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_runu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_rutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_dtrsm_rutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



#endif
