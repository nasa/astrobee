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

//#if defined(OS_LINUX)
//#include <sys/mman.h>
//#endif

//#include "../utils/page-info.h"
//#include "../utils/page-info.c"

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
#include <xmmintrin.h>
#endif

#include <blasfeo_target.h>
#include <blasfeo_block_size.h>
#include <blasfeo_common.h>
#include <blasfeo_stdlib.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>

#include <blasfeo_timing.h>

#include <blasfeo_memory.h>

//void *blas_memory_alloc(int);
//void blas_memory_free(void *);
//void *blas_memory_alloc_nolock(int);
//void blas_memory_free_nolock(void *);



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_hp_dgemm_nn blasfeo_hp_cm_dgemm_nn
#define blasfeo_hp_dgemm_nt blasfeo_hp_cm_dgemm_nt
#define blasfeo_hp_dgemm_tn blasfeo_hp_cm_dgemm_tn
#define blasfeo_hp_dgemm_tt blasfeo_hp_cm_dgemm_tt
#define blasfeo_dgemm_nn blasfeo_cm_dgemm_nn
#define blasfeo_dgemm_nt blasfeo_cm_dgemm_nt
#define blasfeo_dgemm_tn blasfeo_cm_dgemm_tn
#define blasfeo_dgemm_tt blasfeo_cm_dgemm_tt
#endif


//#ifdef HP_BLAS // i.e. when compiled from blas_api/dgemm_ref.c
//#define blasfeo_hp_dgemm_nt_m2 blasfeo_hp_cm_dgemm_nt_m2
//#define blasfeo_hp_dgemm_nt_n2 blasfeo_hp_cm_dgemm_nt_n2
//#endif


#define CACHE_LINE_EL D_CACHE_LINE_EL
#define L1_CACHE_EL D_L1_CACHE_EL
#define L2_CACHE_EL D_L2_CACHE_EL
#define LLC_CACHE_EL D_LLC_CACHE_EL
#define PS D_PS
#define M_KERNEL D_M_KERNEL
#define KC D_KC
#define NC D_NC
#define MC D_MC



void blasfeo_hp_dgemm_nt_m2(int m, int n, int k, double alpha, double *pA, int sda, double *pB, int sdb, double beta, double *C, int ldc, double *D, int ldd)
	{

	int ii, jj;

	double *pA_p, *pB_p;

#if defined(TARGET_X64_INTEL_HASWELL)
	_mm_prefetch(pA+0, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+0, _MM_HINT_T0);
	_mm_prefetch(pA+8*sda+0, _MM_HINT_T0);
	_mm_prefetch(pB+0, _MM_HINT_T0);

	_mm_prefetch(pA+8, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+8, _MM_HINT_T0);
	_mm_prefetch(pA+8*sda+8, _MM_HINT_T0);
	_mm_prefetch(pB+8, _MM_HINT_T0);

	_mm_prefetch(pA+16, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+16, _MM_HINT_T0);
	_mm_prefetch(pA+8*sda+16, _MM_HINT_T0);
	_mm_prefetch(pB+16, _MM_HINT_T0);

	_mm_prefetch(pA+24, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+24, _MM_HINT_T0);
	_mm_prefetch(pA+8*sda+24, _MM_HINT_T0);
	_mm_prefetch(pB+24, _MM_HINT_T0);
#endif

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	_mm_prefetch(pA+0, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+0, _MM_HINT_T0);
	_mm_prefetch(pB+0, _MM_HINT_T0);

	_mm_prefetch(pA+8, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+8, _MM_HINT_T0);
	_mm_prefetch(pB+8, _MM_HINT_T0);

	_mm_prefetch(pA+16, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+16, _MM_HINT_T0);
	_mm_prefetch(pB+16, _MM_HINT_T0);

	_mm_prefetch(pA+24, _MM_HINT_T0);
	_mm_prefetch(pA+4*sda+24, _MM_HINT_T0);
	_mm_prefetch(pB+24, _MM_HINT_T0);
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	// TODO
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
#if defined(TARGET_X64_INTEL_HASWELL)
		jj = 0;
		if(n>0)
			{
			pA_p = m-ii<=12 ? pA : pA+(ii+12)*sda;
			pB_p = pB;
			kernel_dgemm_nt_12xn_p0_lib44cc(n, k, &alpha, pA+ii*sda, sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
//			kernel_dgemm_nt_12xn_pl_lib44cc(n, k, &alpha, pA+ii*sda, sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
			jj += n;
			}
#else
		for(jj=0; jj<n-3; jj+=4)
			{
#if defined(TARGET_X64_INTEL_HASWELL)
			pA_p = pA; //(jj+4<n) ? pA : (pA+(jj+12)*sda);
			pB_p = pB+(jj+4)*sdb; //(jj+4<n) ? (pB+(jj+4)*sdb) : pB;
			kernel_dgemm_nt_12x4_p0_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
#else
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#endif
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
#endif
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
#if 1 //defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		jj = 0;
		if(n>0)
			{
//			pA_p = m-ii<=8 ? pA : pA+(ii+8)*sda;
			pA_p = m-ii<=8 ? pA : pA+(ii+8)*sda;
			pB_p = pB;
			kernel_dgemm_nt_8xn_p0_lib44cc(n, k, &alpha, pA+ii*sda, sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
			jj += n;
//			kernel_dgemm_nt_8x4_p_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p); //, pB_p);
//			jj += 4;
			}
#else
		for(jj=0; jj<n-3; jj+=4)
			{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57)
#if 0
			pA_p = (jj+4<n) ? pA : (pA+(jj+8)*sda);
			pB_p = (jj+4<n) ? (pB+(jj+4)*sdb) : pB;
//			kernel_dgemm_nt_8x4_p0_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#else
			if(n-jj>4)
				{
//				pA_p = (n-jj>4) ? pA : (pA+(jj+8)*sda);
//				pB_p = (n-jj>4) ? (pB+(jj+4)*sdb) : pB;
				pA_p = pA;
				pB_p = pB+(jj+4)*sdb;
//				kernel_dgemm_nt_8x4_p0_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
				kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
				}
			else
				{
				pA_p = m-ii<=8 ? pA : pA+(ii+8)*sda;
				pB_p = pB; // TODO
				kernel_dgemm_nt_8x4_p_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p); //, pB_p);
				}
#endif
#else
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#endif
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
#endif
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-23; ii+=24)
		{
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_24x8_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_24x8_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-15; ii+=16)
		{
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_16x8_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_16x8_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_8x8_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x8_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m2_left_8;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m2_left_4;
		}
#endif
	goto nt_m2_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_m2_left_24:
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_24x8_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_m2_left_16:
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_16x8_vs_lib88cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pA+ii*sda, sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_m2_left_8:
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_8x8_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;
#endif

nt_m2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m2_return;

nt_m2_return:
	return;

	return;

	}



void blasfeo_hp_dgemm_nt_n2(int m, int n, int k, double alpha, double *pA, int sda, double *pB, int sdb, double beta, double *C, int ldc, double *D, int ldd)
	{

	int ii, jj;

	double *pA_p, *pB_p;

#if defined(TARGET_X64_INTEL_HASWELL)
	_mm_prefetch(pB+0, _MM_HINT_T0);
	_mm_prefetch(pB+4*sdb+0, _MM_HINT_T0);
	_mm_prefetch(pB+8*sdb+0, _MM_HINT_T0);
	_mm_prefetch(pA+0, _MM_HINT_T0);

	_mm_prefetch(pB+8, _MM_HINT_T0);
	_mm_prefetch(pB+4*sdb+8, _MM_HINT_T0);
	_mm_prefetch(pB+8*sdb+8, _MM_HINT_T0);
	_mm_prefetch(pA+8, _MM_HINT_T0);

	_mm_prefetch(pB+16, _MM_HINT_T0);
	_mm_prefetch(pB+4*sdb+16, _MM_HINT_T0);
	_mm_prefetch(pB+8*sdb+16, _MM_HINT_T0);
	_mm_prefetch(pA+16, _MM_HINT_T0);

	_mm_prefetch(pB+24, _MM_HINT_T0);
	_mm_prefetch(pB+4*sdb+24, _MM_HINT_T0);
	_mm_prefetch(pB+8*sdb+24, _MM_HINT_T0);
	_mm_prefetch(pA+24, _MM_HINT_T0);
#endif

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-11; jj+=12)
		{
#if defined(TARGET_X64_INTEL_HASWELL)
		ii = 0;
		if(m>0)
			{
			pA_p = pA;
			pB_p = n-jj<=12 ? pB : pB+(jj+12)*sdb;
			kernel_dgemm_nt_mx12_p0_lib44cc(m, k, &alpha, pA+ii*sda, sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
			ii += m;
			}
#else
		for(ii=0; ii<m-3; ii+=4)
			{
#if defined(TARGET_X64_INTEL_HASWELL)
			pA_p = pA+(ii+4)*sda;
			pB_p = pB;
			kernel_dgemm_nt_4x12_p0_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, pA_p, pB_p);
#else
			kernel_dgemm_nt_4x12_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#endif
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x12_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
#endif
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nt_n2_left_4;
			}
		if(n-jj<=8)
			{
			goto nt_n2_left_8;
			}
		else
			{
			goto nt_n2_left_12;
			}
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-7; jj+=8)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x8_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pA+ii*sda, pB+(jj+0)*sdb, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pA+ii*sda, pB+(jj+4)*sdb, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x8_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nt_n2_left_4;
			}
		else
			{
			goto nt_n2_left_8;
			}
		}
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-7; jj+=8)
		{
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_nt_8x8_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_8x8_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nt_n2_left_8;
		}
#else
	for(; jj<n-3; jj+=4)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nt_n2_left_4;
		}
#endif
	goto nt_n2_return;

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_n2_left_12:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x12_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_n2_left_8:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x8_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n2_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_n2_left_8:
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_nt_8x8_vs_lib88cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n2_return;
#endif

nt_n2_left_4:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pA+ii*sda, pB+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n2_return;

nt_n2_return:
	return;

	return;

	}



static void blasfeo_hp_dgemm_nn_m1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_m1_left_4;
			}
		if(m-ii<=8)
			{
			goto nn_m1_left_8;
			}
		else
			{
			goto nn_m1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-23; ii+=24)
		{
		kernel_dpack_nn_24_lib8(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nn_24x8_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-15; ii+=16)
		{
		kernel_dpack_nn_16_lib8(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nn_16x8_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib8(k, A+ii, lda, pU);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nn_8x8_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x8_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nn_m1_left_8;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_4x4_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nn_m1_left_4;
		}
#endif
	goto nn_m1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nn_m1_left_24:
	kernel_dpack_nn_24_vs_lib8(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nn_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nn_m1_left_16:
	kernel_dpack_nn_16_vs_lib8(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nn_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_m1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_m1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nn_m1_left_8:
	kernel_dpack_nn_8_vs_lib8(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nn_8x8_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m1_return;
#endif

nn_m1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_dgemm_nn_4x12_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_dgemm_nn_4x8_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_dgemm_nn_4x8_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nn_m1_return;

nn_m1_return:
	return;

	}



static void blasfeo_hp_dgemm_nn_n1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x12_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nn_n1_left_4;
			}
		else if(n-jj<=8)
			{
			goto nn_n1_left_8;
			}
		else
			{
			goto nn_n1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
//#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
//			kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//			kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-23; jj+=24)
		{
		kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		kernel_dpack_tn_8_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_nt_8x24_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_8x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto nn_n1_left_8;
			}
		if(n-jj<=16)
			{
			goto nn_n1_left_16;
			}
		else
			{
			goto nn_n1_left_24;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-15; jj+=16)
		{
		kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_nt_8x16_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_8x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_nt_8x8_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_8x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nn_n1_left_8;
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x4_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nn_n1_left_4;
		}
#endif
	goto nn_n1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nn_n1_left_24:
	kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
	kernel_dpack_tn_8_vs_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu, n-jj-16);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_nt_8x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nn_n1_left_16:
	kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_8_vs_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_nt_8x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
nn_n1_left_12:
	kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#endif


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_n1_left_8:
	kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu, n-jj-4);
	for(ii=0; ii<m; ii+=4)
		{
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_dgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
//		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
		}
	goto nn_n1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nn_n1_left_8:
	kernel_dpack_tn_8_vs_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_nt_8x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n1_return;
#endif

nn_n1_left_4:
	kernel_dpack_tn_4_vs_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_dgemm_nt_12x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_dgemm_nt_8x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(ii=0; ii<m-4; ii+=8)
		{
		kernel_dgemm_nt_8x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m)
		{
		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nn_n1_return;

nn_n1_return:
	return;


	}



static void blasfeo_hp_dgemm_nt_m1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_m1_left_4;
			}
		if(m-ii<=8)
			{
			goto nt_m1_left_8;
			}
		else
			{
			goto nt_m1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-23; ii+=24)
		{
		kernel_dpack_nn_24_lib8(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_24x8_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_m1_left_8;
			}
		if(m-ii<=16)
			{
			goto nt_m1_left_16;
			}
		else
			{
			goto nt_m1_left_24;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-15; ii+=16)
		{
		kernel_dpack_nn_16_lib8(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_16x8_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib8(k, A+ii, lda, pU);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_8x8_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x8_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m1_left_8;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_m1_left_4;
		}
#endif
	goto nt_m1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_m1_left_24:
	kernel_dpack_nn_24_vs_lib8(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_m1_left_16:
	kernel_dpack_nn_16_vs_lib8(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
nt_m1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_m1_left_8:
	kernel_dpack_nn_8_vs_lib8(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_8x8_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m1_return;
#endif

nt_m1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_dgemm_nt_4x8_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_dgemm_nt_4x8_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nt_m1_return;

nt_m1_return:
	return;

	}



static void blasfeo_hp_dgemm_nt_n1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_dpack_nn_12_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x12_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nt_n1_left_4;
			}
		else if(n-jj<=8)
			{
			goto nt_n1_left_8;
			}
		else
			{
			goto nt_n1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_nn_8_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
//#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
//			kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//			kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-23; jj+=24)
		{
		kernel_dpack_nn_24_lib8(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_nt_8x24_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_8x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto nt_n1_left_8;
			}
		if(n-jj<=16)
			{
			goto nt_n1_left_16;
			}
		else
			{
			goto nt_n1_left_24;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-15; jj+=16)
		{
		kernel_dpack_nn_16_lib8(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_nt_8x16_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_8x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_nn_8_lib8(k, B+jj, ldb, pU);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_nt_8x8_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_8x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nt_n1_left_8;
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_dpack_nn_4_lib4(k, B+jj, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x4_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto nt_n1_left_4;
		}
#endif
	goto nt_n1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_n1_left_24:
	kernel_dpack_nn_24_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_nt_8x24_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_n1_left_16:
	kernel_dpack_nn_16_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_nt_8x16_vs_libc8cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
nt_n1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#endif


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_n1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_dgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
//		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
		}
	goto nt_n1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
nt_n1_left_8:
	kernel_dpack_nn_8_vs_lib8(k, B+jj, ldb, pU, n-jj);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_nt_8x8_vs_libc8cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n1_return;
#endif

nt_n1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, B+jj, ldb, pU, n-jj);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_dgemm_nt_12x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_dgemm_nt_8x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(ii=0; ii<m-4; ii+=8)
		{
		kernel_dgemm_nt_8x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m)
		{
		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nt_n1_return;

nt_n1_return:
	return;

	}



static void blasfeo_hp_dgemm_tn_m1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_m1_left_4;
			}
		if(m-ii<=8)
			{
			goto tn_m1_left_8;
			}
		else
			{
			goto tn_m1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-23; ii+=24)
		{
		kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		kernel_dpack_tn_8_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nn_24x8_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-15; ii+=16)
		{
		kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nn_16x8_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_8_lib8(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nn_8x8_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x8_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tn_m1_left_8;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_4x4_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tn_m1_left_4;
		}
#endif
	goto tn_m1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tn_m1_left_24:
	kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
	kernel_dpack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu, m-ii-16);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nn_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tn_m1_left_16:
	kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nn_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_m1_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_m1_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tn_m1_left_8:
	kernel_dpack_tn_8_vs_lib8(k, A+ii*lda, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nn_8x8_vs_lib8ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m1_return;
#endif

tn_m1_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_dgemm_nn_4x12_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_dgemm_nn_4x8_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_dgemm_nn_4x8_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tn_m1_return;

tn_m1_return:
	return;

	}



static void blasfeo_hp_dgemm_tn_n1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x12_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tn_n1_left_4;
			}
		else if(n-jj<=8)
			{
			goto tn_n1_left_8;
			}
		else
			{
			goto tn_n1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
//#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
//			kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//			kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-23; jj+=24)
		{
		kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		kernel_dpack_tn_8_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_tt_8x24_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_8x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto tn_n1_left_8;
			}
		if(n-jj<=16)
			{
			goto tn_n1_left_16;
			}
		else
			{
			goto tn_n1_left_24;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-15; jj+=16)
		{
		kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		kernel_dpack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_tt_8x16_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_8x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_tt_8x8_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_8x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tn_n1_left_8;
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x4_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tn_n1_left_4;
		}
#endif
	goto tn_n1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tn_n1_left_24:
	kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_8_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
	kernel_dpack_tn_8_vs_lib8(k, B+(jj+16)*ldb, ldb, pU+16*sdu, n-jj-16);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_tt_8x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tn_n1_left_16:
	kernel_dpack_tn_8_lib8(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_8_vs_lib8(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_tt_8x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
tn_n1_left_12:
	kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#endif


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_n1_left_8:
	kernel_dpack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_dpack_tn_4_vs_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu, n-jj-4);
	for(ii=0; ii<m; ii+=4)
		{
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_dgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
//		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
		}
	goto tn_n1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tn_n1_left_8:
	kernel_dpack_tn_8_vs_lib8(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_tt_8x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n1_return;
#endif

tn_n1_left_4:
	kernel_dpack_tn_4_vs_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_dgemm_tt_12x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_dgemm_tt_8x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(ii=0; ii<m-4; ii+=8)
		{
		kernel_dgemm_tt_8x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tn_n1_return;

tn_n1_return:
	return;

	}



static void blasfeo_hp_dgemm_tt_m1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_m1_left_4;
			}
		if(m-ii<=8)
			{
			goto tt_m1_left_8;
			}
		else
			{
			goto tt_m1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-23; ii+=24)
		{
		kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		kernel_dpack_tn_8_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_24x8_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-15; ii+=16)
		{
		kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_16x8_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_8_lib8(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_dgemm_nt_8x8_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x8_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_m1_left_8;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_m1_left_4;
		}
#endif
	goto tt_m1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tt_m1_left_24:
	kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_8_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu);
	kernel_dpack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, pU+16*sdu, m-ii-16);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_24x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tt_m1_left_16:
	kernel_dpack_tn_8_lib8(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_16x8_vs_lib8ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
tt_m1_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_m1_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tt_m1_left_8:
	kernel_dpack_tn_8_vs_lib8(k, A+ii*lda, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=8)
		{
		kernel_dgemm_nt_8x8_vs_lib8ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m1_return;
#endif

tt_m1_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_dgemm_nt_4x8_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_dgemm_nt_4x8_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tt_m1_return;

tt_m1_return:
	return;

	}



static void blasfeo_hp_dgemm_tt_n1(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc, double *D, int ldd, double *pU, int sdu)
	{

	int ii, jj;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_dpack_nn_12_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x12_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tt_n1_left_4;
			}
		else if(n-jj<=8)
			{
			goto tt_n1_left_8;
			}
		else
			{
			goto tt_n1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_nn_8_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
//#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
			kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//			kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
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
#elif defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-23; jj+=24)
		{
		kernel_dpack_nn_24_lib8(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_tt_8x24_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_8x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=8)
			{
			goto tt_n1_left_8;
			}
		if(n-jj<=16)
			{
			goto tt_n1_left_16;
			}
		else
			{
			goto tt_n1_left_24;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-15; jj+=16)
		{
		kernel_dpack_nn_16_lib8(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_tt_8x16_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_8x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#elif 0 //defined(TARGET_X64_INTEL_SKYLAKE_X)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpack_nn_8_lib8(k, B+jj, ldb, pU);
		for(ii=0; ii<m-7; ii+=8)
			{
			kernel_dgemm_tt_8x8_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_8x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tt_n1_left_8;
		}
#else
	for(; jj<n-3; jj+=4)
		{
		kernel_dpack_nn_4_lib4(k, B+jj, ldb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x4_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tt_n1_left_4;
		}
#endif
	goto tt_n1_return;

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tt_n1_left_24:
	kernel_dpack_nn_24_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_tt_8x24_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tt_n1_left_16:
	kernel_dpack_nn_16_vs_lib8(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_tt_8x16_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
tt_n1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#endif


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_n1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_dgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//#else
//		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU+4*sdu, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
//#endif
		}
	goto tt_n1_return;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
tt_n1_left_8:
	kernel_dpack_nn_8_vs_lib8(k, B+jj, ldb, pU, n-jj);
	for(ii=0; ii<m; ii+=8)
		{
		kernel_dgemm_tt_8x8_vs_libc8cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n1_return;
#endif

tt_n1_left_4:
	kernel_dpack_nn_4_vs_lib4(k, B+jj, ldb, pU, n-jj);
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_dgemm_tt_12x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_dgemm_tt_8x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(ii=0; ii<m-4; ii+=8)
		{
		kernel_dgemm_tt_8x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tt_n1_return;

tt_n1_return:
	return;

	}




//#ifdef HP_BLAS
//
//static void blas_hp_dgemm_nn(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_dgemm_nn %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	double *D = C;
//
//#else

void blasfeo_hp_dgemm_nn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgemm_nn (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
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

//#endif

//	printf("\n%p %d %p %d %p %d %p %d\n", A, lda, B, ldb, C, ldc, D, ldd);

	int ii, jj, ll, kk;
	int iii;
	int idx;
	int mc, nc, kc, oc;
	int mleft, nleft, kleft, oleft;
	int mc0, nc0, kc0, oc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *C1;

	const int ps = PS;

#if defined(TARGET_GENERIC)
	double pU_stack[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( double pU_stack[M_KERNEL*K_MAX_STACK], 64 );
//	ALIGNED( double pU_stack[M_KERNEL*K_MAX_STACK], 4096 );
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

	int error;
	int mem_size;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL)
	const int l2_cache_el = L2_CACHE_EL;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
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
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!

#if defined(PACKING_ALG_0)
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	goto nn_m1; // pack A
#else
	goto nn_0; // no pack
#endif
#endif
#if defined(PACKING_ALG_M1)
	goto nn_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto nn_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto nn_2; // pack A and B
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	if( (m<=m_kernel & n<=m_kernel) | (m_a_kernel*k + k_b*n <= l1_cache_el) )
		{
//		printf("\nalg 2\n");
//		goto nn_0; // small matrix: no pack TODO
		goto nn_m1; // small matrix: pack A
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( (m<=m_kernel & n<=m_kernel) | (m_a_kernel*k + k_b*n <= l1_cache_el) )
		{
//		printf("\nalg 2\n");
		goto nn_0; // small matrix: no pack
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=48 & n<=48 & k<=K_MAX_STACK )
		{
		goto nn_m1; // small matrix: pack A
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=m_kernel & n<=m_kernel & k<160) )
		{
//		printf("\nalg 2\n");
		goto nn_0; // small matrix: no pack
		}
#else
	if( m<=8 & n<=8 )
		{
		goto nn_0; // small matrix: no pack
		}
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL)
	if( m<n*4 )
		{
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
//		if( n<=2*m_kernel | k_b*n <= l2_cache_el ) // TODO k_block
		if( n<=2*m_kernel | k_block*n <= l2_cache_el )
#else
//		if( m<=2*m_kernel | m_a*k + k_b*n <= llc_cache_el )
		if( m<=2*m_kernel | ( k<=KC & (m_a*k + k_b*n + m_c*n + m_d*n <= llc_cache_el) ) )
#endif
			{
//			printf("\nalg m0\n");
			goto nn_m1; // long matrix: pack A
			}
		}
	else
		{
		if( n<=2*m_kernel | m_a*k_block <= l2_cache_el )
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
	
//	if(K_MAX_STACK<=0)
//		goto nn_2;

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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_nn_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_nn_m1(m, n, kleft, alpha, A+ll*lda, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}

		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



nn_n1:

//	if(K_MAX_STACK<=0)
//		goto nn_2;

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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_nn_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_nn_n1(m, n, kleft, alpha, A+ll*lda, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



nn_2:

#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

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
	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
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
//	mem = blas_memory_alloc(0);
//	mem = blas_memory_alloc_nolock(0);
	blasfeo_align_4096_byte(mem, (void **) &mem_align);
//	error = madvise( mem_align, tA_size+tB_size+4096, MADV_HUGEPAGE );
#else
	mem_size = tA_size+tB_size+4096+2*2*1024*1024;
	mem_size = (mem_size + 2*1024*1024 - 1) / (2*1024*1024) * (2*1024*1024);
//	error = posix_memalign( &mem, 4*1024, tA_size+tB_size+2*4096 );
//	error = posix_memalign( &mem, 2*1024*1024, mem_size );
//	error = madvise( mem, tA_size+tB_size+2*4096, MADV_HUGEPAGE );
	mem = malloc(mem_size);
	blasfeo_align_2MB(mem, (void **) &mem_align);
//	blasfeo_align_4096_byte(mem, (void **) &mem_align);
	error = madvise( mem_align, tA_size+tB_size+4096+2*1024*1024, MADV_HUGEPAGE );
#endif

#if 0
//page_info_array pinfo = get_info_for_range(mem, mem + tA_size+tB_size+2*4096);
//page_info_array pinfo = get_info_for_range(mem, mem + tA_size+tB_size+4096+2*1024*1024);
page_info_array pinfo = get_info_for_range(mem_align, mem_align + tA_size+tB_size+4096);
flag_count thp_count = get_flag_count(pinfo, KPF_THP);
flag_count huge_count = get_flag_count(pinfo, KPF_HUGE);
if (thp_count.pages_available) {
    printf("Source pages allocated with transparent hugepages: %4.1f%%; huge pages %4.1f%% (%lu or %lu of %lu total pages, %4.1f%% flagged)\n",
            100.0 * thp_count.pages_set / thp_count.pages_total,
            100.0 * huge_count.pages_set / huge_count.pages_total,
            thp_count.pages_set, huge_count.pages_set, thp_count.pages_total,
            100.0 * thp_count.pages_available / thp_count.pages_total);
} else {
    printf("Couldn't determine hugepage info (you are probably not running as root)\n");
}
#endif

//	blasfeo_pm_create_dmat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_dmat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
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
#endif
	//		time_pack_A += blasfeo_toc(&timer);

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack and tran B
	//			blasfeo_tic(&timer);
#if 1
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<nleft-7; iii+=8)
					{
					kernel_dpack_tn_8_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_dpack_tn_8_vs_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb, nleft-iii);
					}
#else
				kernel_dpack_buffer_ft(kleft, nleft, B+ll+jj*ldb, ldb, pB, sdb);
#endif
#endif
	//			time_pack_B += blasfeo_toc(&timer);

				// prefetch A
//				for(iii=0; iii<kleft; iii+=2)
//					{
//					__builtin_prefetch(pA+4*iii);
//					__builtin_prefetch(pA+4*sda+4*iii);
//					}
				// prefetch B
//				for(iii=0; iii<nleft*kleft; iii+=8)
//					{
//					__builtin_prefetch(pB+iii, 0, 2);
//					__builtin_prefetch(pB+iii);
//					_mm_prefetch(pB+iii, _MM_HINT_T1);
//					}

	//			blasfeo_tic(&timer);
				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
	//			time_kernel += blasfeo_toc(&timer);

				}
			
			}

		}

//	printf("\ntime: pack_A %e, pack_B %e, kernel %e, kernel2 %e, kernel3 %e\n", time_pack_A, time_pack_B, time_kernel, time_kernel2, time_kernel3); 
	if(blasfeo_is_init()==0)
		{
		blasfeo_free(mem);
		}
//	blas_memory_free(mem);
//	blas_memory_free_nolock(mem);

	return;



#elif 1 //defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// cache blocking alg

	mc0 = NC; //MC;
	nc0 = MC; //NC;
	kc0 = KC;

#if 0
	mc0 = 8;//12;
	nc0 = 8;
	kc0 = 4;
#endif

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
	kc = k<kc0 ? k : kc0;

	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, kc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
	if(blasfeo_is_init()==0)
		{
		mem = malloc(tA_size+tB_size+2*4096);
		}
	else
		{
		mem = blasfeo_get_buffer();
		}
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	mem_align += 4096-4*128;
	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

//	double time_pack_A = 0.0;
//	double time_pack_B = 0.0;
//	double time_kernel = 0.0;

//	blasfeo_timer timer;

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

		for(jj=0; jj<n; jj+=nleft)
			{

			nleft = n-jj<nc ? n-jj : nc;

			// pack and tran B
//			blasfeo_tic(&timer);
#if 1
			kernel_dpack_buffer_ft(kleft, nleft, B+ll+jj*ldb, ldb, pB, sdb);
#endif
//			time_pack_B += blasfeo_toc(&timer);

			for(ii=0; ii<m; ii+=mleft)
				{

				mleft = m-ii<mc ? m-ii : mc;

				// pack A
		//		blasfeo_tic(&timer);
#if 1
				kernel_dpack_buffer_fn(mleft, kleft, A+ii+ll*lda, lda, pA, sda);
#endif
		//		time_pack_A += blasfeo_toc(&timer);

	//			blasfeo_tic(&timer);
				blasfeo_hp_dgemm_nt_n2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
//				dgemm_kernel(mleft, nleft, kleft, alpha, pA, pB, D+ii+jj*ldd, ldd);
//				dgemm_kernel(nleft, mleft, kleft, alpha, pB, pA, D+jj+ii*ldd, ldd);
//				d_print_mat(mleft, nleft, D+ii+jj*ldd, ldd);
//				exit(1);
	//			time_kernel += blasfeo_toc(&timer);

				}
			
			}

		}

//	printf("\ntime: pack_A %e, pack_B %e, kernel %e, kernel2 %e, kernel3 %e\n", time_pack_A, time_pack_B, time_kernel, time_kernel2, time_kernel3); 
	free(mem);

	return;



#elif 0



	// cache blocking alg

	mc0 = 768; //MC;
	nc0 = 72; //NC;
	kc0 = 256; //KC;
	oc0 = 512; //OC;

//	mc0 = 12;
//	nc0 = 8;
//	kc0 = 4;
//	oc0 = 8;

	mc = m<mc0 ? m : mc0;
	nc = n<nc0 ? n : nc0;
	kc = k<kc0 ? k : kc0;
	oc = k<oc0 ? k : oc0;

	tA_size = blasfeo_pm_memsize_dmat(ps, mc0, oc0);
	tB_size = blasfeo_pm_memsize_dmat(ps, nc0, kc0);
	tA_size = (tA_size + 4096 - 1) / 4096 * 4096;
	tB_size = (tB_size + 4096 - 1) / 4096 * 4096;
	mem = malloc(tA_size+tB_size+2*4096);
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

	blasfeo_pm_create_dmat(ps, mc0, oc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	pA = tA.pA;
	pB = tB.pA;

	for(ll=0; ll<k; ll+=oleft)
		{

		oleft = k-ll<oc ? k-ll : oc;

		for(ii=0; ii<m; ii+=mleft)
			{

			mleft = m-ii<mc ? m-ii : mc;

			for(kk=0, idx=0; kk<oleft; kk+=kleft, idx++)
				{

				kleft = oleft-kk<kc ? oleft-kk : kc;

				sda = (kleft+4-1)/4*4;

#if 1
				// pack A
				kernel_dpack_buffer_fn(mleft, kleft, A+ii+(ll+kk)*lda, lda, pA+idx*mleft*sda, sda);
#endif
//				printf("\nA\n");
//				tA.cn = sda;
//				blasfeo_pm_print_dmat(mleft, kleft, &tA, idx*mleft, 0);

				}

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;


				for(kk=0, idx=0; kk<oleft; kk+=kleft, idx++)
					{

					kleft = oleft-kk<kc ? oleft-kk : kc;

					sda = (kleft+4-1)/4*4;
					sdb = (kleft+4-1)/4*4;

					beta1 = (ll+kk)==0 ? beta : 1.0;
					C1 = (ll+kk)==0 ? C : D;
					ldc1 = (ll+kk)==0 ? ldc : ldd;

					// pack and tran B
#if 1
					kernel_dpack_buffer_ft(kleft, nleft, B+ll+kk+jj*ldb, ldb, pB, sdb);
#endif
//					printf("\nA\n");
//					tA.cn = sda;
//					blasfeo_pm_print_dmat(mleft, kleft, &tA, idx*mleft, 0);

//					printf("\nB\n");
//					tB.cn = sdb;
//					blasfeo_pm_print_dmat(nleft, kleft, &tB, 0, 0);

//					printf("\n%d %d %d\n", mleft, nleft, kleft);
					blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA+idx*mleft*sda, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
//					printf("\nD\n");
//					d_print_mat(mleft, kleft, D+ii+jj*ldd, ldd);
//					exit(1);

					}

				}

			}

		}

//	printf("\ntime: pack_A %e, pack_B %e, kernel %e, kernel2 %e, kernel3 %e\n", time_pack_A, time_pack_B, time_kernel, time_kernel2, time_kernel3); 
	free(mem);

	return;



#else



	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, k1);
//	mem = malloc(tA_size+tB_size+64);
//	blasfeo_align_64_byte(mem, (void **) &mem_align);
	mem = malloc(tA_size+tB_size+4096);
	blasfeo_align_4096_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

//	blasfeo_pack_tran_dmat(k, n, B, ldb, &tB, 0, 0);
	pack_B = 1;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_dpack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_2_left_4;
			}
		if(m-ii<=8)
			{
			goto nn_2_left_8;
			}
		else
			{
			goto nn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_dpack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
		kernel_dpack_nn_4_lib4(k, A+ii, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_dpack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		goto nn_2_left_4;
		}
#endif
	goto nn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_2_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_2_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_2_return;
#endif

nn_2_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_2_return;

nn_2_return:
	free(mem);
	return;

#endif


nn_0:
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_12x4_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_0_left_4;
			}
		if(m-ii<=8)
			{
			goto nn_0_left_8;
			}
		else
			{
			goto nn_0_left_12;
			}
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_8x4_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_0_left_4;
			}
		else
			{
			goto nn_0_left_8;
			}
		}
#elif ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_4x4_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nn_0_left_4;
		}
#endif
	goto nn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nn_0_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
nn_0_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_8x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_0_return;
#endif

#if ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_0_left_4:
#if defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_dgemm_nn_4x12_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_dgemm_nn_4x8_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_dgemm_nn_4x8_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nn_0_return;
#endif

nn_0_return:
	return;


	// never to get here
	return;

	}



//#ifdef HP_BLAS
//
//static void blas_hp_dgemm_nt(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_dgemm_nt %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	double *D = C;
//
//#else

void blasfeo_hp_dgemm_nt(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgemm_nt (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
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

//#endif

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *C1;

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

	const int ps = PS;
	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL)
	const int l2_cache_el = L2_CACHE_EL;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
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
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!

#if defined(PACKING_ALG_0)
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	goto nt_m1; // pack A
#else
	goto nt_0; // no pack
#endif
#endif
#if defined(PACKING_ALG_M1)
	goto nt_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto nt_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto nt_2; // pack A and B
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	if( (m<=m_kernel & n<=m_kernel) | (m_a_kernel*k + n_b*k <= l1_cache_el) )
		{
//		printf("\nalg 2\n");
//		goto nt_0; // small matrix: no pack TODO
		goto nt_m1; // small matrix: pack A
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) 
	if( (m<=m_kernel & n<=m_kernel) | (m_a_kernel*k + n_b*k <= l1_cache_el) )
		{
//		printf("\nalg 2\n");
		goto nt_0; // small matrix: no pack
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=48 & n<=48 & k<=K_MAX_STACK )
		{
		goto nt_m1; // small matrix: pack A
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=m_kernel & n<=m_kernel & k<160) )
		{
//		printf("\nalg 2\n");
		goto nt_0; // small matrix: no pack
		}
#else
	if( m<=8 & n<=8 )
		{
		goto nt_0; // small matrix: no pack
		}
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL)
	if( m<=n )
		{
		if( m<=2*m_kernel | n_b*k_block <= l2_cache_el )
			{
//			printf("\nalg m0\n");
			goto nt_m1; // long matrix: pack A
			}
		}
	else
		{
		if( n<=2*m_kernel | m_a*k_block <= l2_cache_el )
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

//	if(K_MAX_STACK<=0)
//		goto nt_2;

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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_nt_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_nt_m1(m, n, kleft, alpha, A+ll*lda, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



nt_n1:

//	if(K_MAX_STACK<=0)
//		goto nt_2;

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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_nt_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_nt_n1(m, n, kleft, alpha, A+ll*lda, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;




nt_2:

//#if 0
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

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
//		printf("\nbuffer\n");
		mem = blasfeo_get_buffer();
		}
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

//	blasfeo_pm_create_dmat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_dmat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	pA = tA.pA;
	pB = tB.pA;

	for(ll=0; ll<k; ll+=kleft)
		{

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

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack B
	//			printf("\nhere\n");
	//			d_print_mat(nleft, kleft, B+jj+ll*ldb, ldb);
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<kleft-3; iii+=4)
					{
					kernel_dpack_tt_4_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_dpack_tt_4_vs_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb, kleft-iii);
					}
#else
				kernel_dpack_buffer_fn(nleft, kleft, B+jj+ll*ldb, ldb, pB, sdb);
#endif
	//			d_print_mat(4, kleft, pB, 4);
	//			d_print_mat(4, kleft, pB+4*sdb, 4);

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
	//			d_print_mat(m, nleft, D+jj*ldd, ldd);

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
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, k1);
//	mem = malloc(tA_size+tB_size+64);
//	blasfeo_align_64_byte(mem, (void **) &mem_align);
	mem = malloc(tA_size+tB_size+4096);
	blasfeo_align_4096_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

//	blasfeo_pack_dmat(n, k, B, ldb, &tB, 0, 0);
	kernel_dpack_buffer_fn(n, k, B, ldb, tB.pA, sdb);

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_2_left_4;
			}
		if(m-ii<=8)
			{
			goto nt_2_left_8;
			}
		else
			{
			goto nt_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, tA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
		kernel_dpack_nn_4_lib4(k, A+ii, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_2_left_4;
		}
#endif
	goto nt_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_2_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_2_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, tA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_2_return;
#endif

nt_2_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_2_return;

nt_2_return:
	free(mem);
	return;

#endif



nt_0:
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_0_left_4;
			}
		if(m-ii<=8)
			{
			goto nt_0_left_8;
			}
		else
			{
			goto nt_0_left_12;
			}
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_0_left_4;
			}
		else
			{
			goto nt_0_left_8;
			}
		}
#elif ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_0_left_4;
		}
#endif
	goto nt_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nt_0_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
nt_0_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_0_return;
#endif

#if ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_0_left_4:
#if defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_dgemm_nt_4x12_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_dgemm_nt_4x8_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_dgemm_nt_4x8_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nt_0_return;
#endif

nt_0_return:
	return;


	// never to get here
	return;

	}



//#ifdef HP_BLAS
//
//static void blas_hp_dgemm_tn(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_dgemm_tn %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	double *D = C;
//
//#else

void blasfeo_hp_dgemm_tn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgemm_tn (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
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

//#endif

//	printf("\n%p %d %p %d %p %d %p %d\n", A, lda, B, ldb, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *C1;

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

	const int ps = PS;
	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL)
	const int l2_cache_el = L2_CACHE_EL;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
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
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!

#if defined(PACKING_ALG_M1)
	goto tn_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto tn_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto tn_2; // pack A and B
#endif

	// no algorithm for small matrix
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	if( m<=n )
		{
		if( n<=2*m_kernel | k_block*n <= l2_cache_el )
			{
//			printf("\nalg m0\n");
			goto tn_m1; // long matrix: pack A
			}
		}
	else
		{
		if( n<=2*m_kernel | k_block*m <= l2_cache_el )
			{
//			printf("\nalg n0\n");
			goto tn_n1; // tall matrix: pack B
			}
		}
#else
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
#endif
//	printf("\nalg 1\n");
	goto tn_2; // big matrix: pack A and B

	// never to get here
	return;


tn_m1:

//	if(K_MAX_STACK<=0)
//		goto tn_2;

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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_tn_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_tn_m1(m, n, kleft, alpha, A+ll, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}

		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



tn_n1:

//	if(K_MAX_STACK<=0)
//		goto tn_2;

	// k-blocking alg

//	if(k>KC)
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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_tn_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_tn_n1(m, n, kleft, alpha, A+ll, lda, B+ll, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

//	if(k>KC)
	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



tn_2:

//#if 0
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

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
//		printf("\nbuffer\n");
		mem = blasfeo_get_buffer();
		}
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

//	blasfeo_pm_create_dmat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_dmat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	pA = tA.pA;
	pB = tB.pA;

	for(ll=0; ll<k; ll+=kleft)
		{

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

			for(jj=0; jj<n; jj+=nleft)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack B
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<nleft-7; iii+=8)
					{
					kernel_dpack_tn_8_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb);
					}
				if(iii<nleft)
					{
					kernel_dpack_tn_8_vs_lib8(kleft, B+ll+(jj+iii)*ldb, ldb, pB+iii*sdb, nleft-iii);
					}
#else
				kernel_dpack_buffer_ft(kleft, nleft, B+ll+jj*ldb, ldb, pB, sdb);
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
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, k1);
//	mem = malloc(tA_size+tB_size+64);
//	blasfeo_align_64_byte(mem, (void **) &mem_align);
	mem = malloc(tA_size+tB_size+4096);
	blasfeo_align_4096_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

//	blasfeo_pack_tran_dmat(k, n, B, ldb, &tB, 0, 0);
	pack_B = 1;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, tA.pA+8*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_dpack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_2_left_4;
			}
		if(m-ii<=8)
			{
			goto tn_2_left_8;
			}
		else
			{
			goto tn_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_dpack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_dpack_tn_4_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb);
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
		if(ii<m)
		{
		goto tn_2_left_4;
		}
#endif
	goto tn_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_2_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, tA.pA+8*sda, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_2_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_2_return;
#endif

tn_2_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+0)*lda, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_dpack_tn_4_vs_lib4(k, B+jj*ldb, ldb, tB.pA+jj*sdb, n-jj);
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
//static void blas_hp_dgemm_tt(int m, int n, int k, double alpha, double *A, int lda, double *B, int ldb, double beta, double *C, int ldc)
//	{
//
//#if defined(PRINT_NAME)
//	printf("\nblas_hp_dgemm_tt %d %d %d %f %p %d %p %d %f %p %d\n", m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
//#endif
//
//	if(m<=0 | n<=0)
//		return;
//
//	int ldd = ldc;
//	double *D = C;
//
//#else

void blasfeo_hp_dgemm_tt(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgemm_tt (cm) %d %d %d %f %p %d %d %p %d %d %f %p %d %d %p %d %d\n", m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
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

//#endif

//	printf("\n%p %d %p %d %p %d %p %d\n", A, lda, B, ldb, C, ldc, D, ldd);

	int ii, jj, ll;
	int iii;
	int mc, nc, kc;
	int mleft, nleft, kleft;
	int mc0, nc0, kc0;
	int ldc1;
	double beta1;
	double *pA, *pB, *C1;

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

	const int ps = PS;
	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL)
	const int l2_cache_el = L2_CACHE_EL;
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
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
	int k_block = K_MAX_STACK<KC ? K_MAX_STACK : KC;
	k_block = k<=k_block ? k : k_block; // m1 and n1 alg are blocked !!!

#if defined(PACKING_ALG_0)
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	goto tt_m1; // pack A
#else
	goto tt_0; // no pack
#endif
#endif
#if defined(PACKING_ALG_M1)
	goto tt_m1; // pack A
#endif
#if defined(PACKING_ALG_N1)
	goto tt_n1; // pack B
#endif
#if defined(PACKING_ALG_2)
	goto tt_2; // pack A and B
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	if( (m<=m_kernel & n<=m_kernel) | (k_a*m + n_b_kernel*k <= l1_cache_el) )
		{
		goto tt_m1; // small matrix: pack A
//		goto tt_0; // small matrix: no pack TODO
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if( (m<=m_kernel & n<=m_kernel) | (k_a*m + n_b_kernel*k <= l1_cache_el) )
		{
		goto tt_0; // small matrix: no pack
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if( m<=48 & n<=48 & k<=K_MAX_STACK )
		{
		goto tt_m1; // small matrix: pack A
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if( (m<=m_kernel & n<=m_kernel & k<160) )
		{
//		printf("\nalg 2\n");
		goto tt_0; // small matrix: no pack
		}
#else
	if( m<=8 & n<=8 )
		{
		goto tt_0; // small matrix: no pack
		}
#endif
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL)
	if( m*4<=n | k<=4 ) // XXX k too !!!
		{
		if( m<=2*m_kernel | n_b*k_block <= l2_cache_el )
			{
//			printf("\nalg m0\n");
			goto tt_m1; // long matrix: pack A
			}
		}
	else
		{
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
		if( n<=2*m_kernel | k_block*m <= l2_cache_el )
#elif defined(TARGET_X64_INTEL_HASWELL)
		if( n<=2*m_kernel | ( k<=KC & (k_a*m + n_b*k + m_c*n + m_d*n <= llc_cache_el) ) )
#endif
			{
//			printf("\nalg n0\n");
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

//	if(K_MAX_STACK<=0)
//		goto tt_2;

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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_tt_m1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_tt_m1(m, n, kleft, alpha, A+ll, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



tt_n1:

//	if(K_MAX_STACK<=0)
//		goto tt_2;

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

//	kc = K_MAX_STACK<KC ? K_MAX_STACK : KC;
//	kc = 4;
	kc = KC;

	if(k<kc)
		{
		blasfeo_hp_dgemm_tt_n1(m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd, pU, sdu);
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

			blasfeo_hp_dgemm_tt_n1(m, n, kleft, alpha, A+ll, lda, B+ll*ldb, ldb, beta1, C1, ldc1, D, ldd, pU, sdu);
			}
		}

	if(k>K_MAX_STACK && KC>K_MAX_STACK)
		{
		blasfeo_free(mem);
		}

	return;



tt_2:

//#if 0
#if defined(TARGET_X64_INTEL_SKYLAKE_X) | defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)

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
//		printf("\nbuffer\n");
		mem = blasfeo_get_buffer();
		}
	blasfeo_align_4096_byte(mem, (void **) &mem_align);

//	blasfeo_pm_create_dmat(ps, mc, kc, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, mc0, kc0, &tA, (void *) mem_align);
	mem_align += tA_size;

	mem_align += 4096-4*128;
//	blasfeo_pm_create_dmat(ps, nc, kc, &tB, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, nc0, kc0, &tB, (void *) mem_align);
	mem_align += tB_size;

	pA = tA.pA;
	pB = tB.pA;

	for(ll=0; ll<k; ll+=kleft)
		{

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

			for(jj=0; jj<n; jj+=nc)
				{

				nleft = n-jj<nc ? n-jj : nc;

				// pack B
	//			printf("\nhere\n");
	//			d_print_mat(nleft, kleft, B+jj+ll*ldb, ldb);
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
				for(iii=0; iii<kleft-3; iii+=4)
					{
					kernel_dpack_tt_4_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb);
					}
				if(iii<kleft)
					{
					kernel_dpack_tt_4_vs_lib8(nleft, B+jj+(ll+iii)*ldb, ldb, pB+iii*ps, sdb, kleft-iii);
					}
#else
				kernel_dpack_buffer_fn(nleft, kleft, B+jj+ll*ldb, ldb, pB, sdb);
#endif
	//			d_print_mat(4, kleft, pB, 4);
	//			d_print_mat(4, kleft, pB+4*sdb, 4);

				blasfeo_hp_dgemm_nt_m2(mleft, nleft, kleft, alpha, pA, sda, pB, sdb, beta1, C1+ii+jj*ldc1, ldc1, D+ii+jj*ldd, ldd);
	//			d_print_mat(m, nleft, D+jj*ldd, ldd);

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
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, k1);
	tB_size = blasfeo_pm_memsize_dmat(ps, n1, k1);
//	mem = malloc(tA_size+tB_size+64);
//	blasfeo_align_64_byte(mem, (void **) &mem_align);
	mem = malloc(tA_size+tB_size+4096);
	blasfeo_align_4096_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_dmat(ps, m_kernel, k, &tA, (void *) mem_align);
	blasfeo_pm_create_dmat(ps, n, k, &tB, (void *) (mem_align+tA_size));

	sda = tA.cn;
	sdb = tB.cn;

//	blasfeo_pack_dmat(n, k, B, ldb, &tB, 0, 0);
	kernel_dpack_buffer_fn(n, k, B, ldb, tB.pA, sdb);

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, tA.pA+8*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_2_left_4;
			}
		if(m-ii<=8)
			{
			goto tt_2_left_8;
			}
		else
			{
			goto tt_2_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_2_left_4;
		}
#endif
	goto tt_2_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_2_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, tA.pA+8*sda, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_2_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_2_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, tA.pA);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, tA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, tA.pA, sda, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_2_return;
#endif

tt_2_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+0)*lda, lda, tA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, tA.pA, tB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_2_return;

tt_2_return:
	free(mem);
	return;

#endif



tt_0:

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x12_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x12_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tt_0_left_4;
			}
		else if(n-jj<=8)
			{
			goto tt_0_left_8;
			}
		else
			{
			goto tt_0_left_12;
			}
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; jj<n-7; jj+=8)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x8_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x8_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tt_0_left_4;
			}
		else
			{
			goto tt_0_left_8;
			}
		}
#elif ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-3; jj+=4)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x4_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto tt_0_left_4;
		}
#endif
	goto tt_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tt_0_left_12:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x12_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_0_return;
#endif


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
tt_0_left_8:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x8_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_0_return;
#endif

#if ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_0_left_4:
#if defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_dgemm_tt_12x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_dgemm_tt_8x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(ii=0; ii<m-4; ii+=8)
		{
		kernel_dgemm_tt_8x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tt_0_return;
#endif

tt_0_return:
	return;

	// never to get here
	return;

	}



#if defined(LA_HIGH_PERFORMANCE)
//#ifndef HP_BLAS



void blasfeo_dgemm_nn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dgemm_nn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dgemm_nt(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dgemm_nt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dgemm_tn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dgemm_tn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dgemm_tt(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dgemm_tt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



//#endif
#endif

