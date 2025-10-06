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

#include <blasfeo_target.h>
#include <blasfeo_block_size.h>
#include <blasfeo_common.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_s_kernel.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_smat blasfeo_cm_smat
#define blasfeo_hp_strsm_llnn blasfeo_hp_cm_strsm_llnn
#define blasfeo_hp_strsm_llnu blasfeo_hp_cm_strsm_llnu
#define blasfeo_hp_strsm_lltn blasfeo_hp_cm_strsm_lltn
#define blasfeo_hp_strsm_lltu blasfeo_hp_cm_strsm_lltu
#define blasfeo_hp_strsm_lunn blasfeo_hp_cm_strsm_lunn
#define blasfeo_hp_strsm_lunu blasfeo_hp_cm_strsm_lunu
#define blasfeo_hp_strsm_lutn blasfeo_hp_cm_strsm_lutn
#define blasfeo_hp_strsm_lutu blasfeo_hp_cm_strsm_lutu
#define blasfeo_hp_strsm_rlnn blasfeo_hp_cm_strsm_rlnn
#define blasfeo_hp_strsm_rlnu blasfeo_hp_cm_strsm_rlnu
#define blasfeo_hp_strsm_rltn blasfeo_hp_cm_strsm_rltn
#define blasfeo_hp_strsm_rltu blasfeo_hp_cm_strsm_rltu
#define blasfeo_hp_strsm_runn blasfeo_hp_cm_strsm_runn
#define blasfeo_hp_strsm_runu blasfeo_hp_cm_strsm_runu
#define blasfeo_hp_strsm_rutn blasfeo_hp_cm_strsm_rutn
#define blasfeo_hp_strsm_rutu blasfeo_hp_cm_strsm_rutu
#define blasfeo_strsm_llnn blasfeo_cm_strsm_llnn
#define blasfeo_strsm_llnu blasfeo_cm_strsm_llnu
#define blasfeo_strsm_lltn blasfeo_cm_strsm_lltn
#define blasfeo_strsm_lltu blasfeo_cm_strsm_lltu
#define blasfeo_strsm_lunn blasfeo_cm_strsm_lunn
#define blasfeo_strsm_lunu blasfeo_cm_strsm_lunu
#define blasfeo_strsm_lutn blasfeo_cm_strsm_lutn
#define blasfeo_strsm_lutu blasfeo_cm_strsm_lutu
#define blasfeo_strsm_rlnn blasfeo_cm_strsm_rlnn
#define blasfeo_strsm_rlnu blasfeo_cm_strsm_rlnu
#define blasfeo_strsm_rltn blasfeo_cm_strsm_rltn
#define blasfeo_strsm_rltu blasfeo_cm_strsm_rltu
#define blasfeo_strsm_runn blasfeo_cm_strsm_runn
#define blasfeo_strsm_runu blasfeo_cm_strsm_runu
#define blasfeo_strsm_rutn blasfeo_cm_strsm_rutn
#define blasfeo_strsm_rutu blasfeo_cm_strsm_rutu
#endif



// TODO move to a header file to reuse across routines
#define EL_SIZE 4 // single precision

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
#define M_KERNEL 12 // max kernel: 12x4
#define L1_CACHE_EL (32*1024/EL_SIZE) // L1 data cache size: 32 kB
#define CACHE_LINE_EL (64/EL_SIZE) // data cache size: 64 bytes

#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
#define M_KERNEL 8 // max kernel: 8x4
#define L1_CACHE_EL (32*1024/EL_SIZE) // L1 data cache size: 32 kB
#define CACHE_LINE_EL (64/EL_SIZE) // data cache size: 64 bytes

#else // assume generic target
#define M_KERNEL 4 // max kernel: 4x4
#define L1_CACHE_EL (32*1024/EL_SIZE) // L1 data cache size: 32 kB
#define CACHE_LINE_EL (64/EL_SIZE) // data cache size: 64 bytes // TODO 32-bytes for cortex A9
#endif



void blasfeo_hp_strsm_llnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_llnn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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



llnn:
	if(m>=12 | n>=12 | m>K_MAX_STACK)
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
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnn_1_left_4;
		}
	goto llnn_1_return;

llnn_1_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnn_1_return;

llnn_1_return:
	return;



llnn_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to lower
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, sdb);
		}
	if(ii<m)
		{
		kernel_spack_tt_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnn_2_left_4;
		}
	goto llnn_2_return;

llnn_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnn_2_return;

llnn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_llnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_llnu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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



llnu:
	if(m>=12 | n>=12 | m>K_MAX_STACK)
		{
		goto llnu_2;
		}
	else
		{
		goto llnu_1;
		}

	// never to get here
	return;

llnu_1:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nt_rl_one_4x4_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_strsm_nt_rl_one_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnu_1_left_4;
		}
	goto llnu_1_return;

llnu_1_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nt_rl_one_4x4_vs_lib4c44c(jj, pU, A+jj, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnu_1_return;

llnu_1_return:
	return;



llnu_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to lower
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, sdb);
		}
	if(ii<m)
		{
		kernel_spack_tt_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, sdb, m-ii);
		}

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nt_rl_one_4x4_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb);
			}
		if(jj<m)
			{
			kernel_strsm_nt_rl_one_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnu_2_left_4;
		}
	goto llnu_2_return;

llnu_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nt_rl_one_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnu_2_return;

llnu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_lltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_lltn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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



lltn:
	if(m>=12 | n>=12 | m>K_MAX_STACK)
		{
		goto lunn_2;
		}
	else
		{
		goto lltn_1;
		}

	// never to get here
	return;

lltn_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nn_rl_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nn_rl_inv_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lltn_1_left_4;
		}
	goto lltn_1_return;

lltn_1_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nn_rl_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nn_rl_inv_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lltn_1_return;

lltn_1_return:
	return;



lunn_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb);
		}
	if(ii<m)
		{
		kernel_spack_tn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nt_ru_inv_4x4_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunn_2_left_4;
		}
	goto lunn_2_return;

lunn_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunn_2_return;

lunn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_lltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_lltu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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



lltu:
	if(m>=12 | n>=12 | m>K_MAX_STACK)
		{
		goto lunu_2;
		}
	else
		{
		goto lltu_1;
		}

	// never to get here
	return;

lltu_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nn_rl_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nn_rl_one_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lltu_1_left_4;
		}
	goto lltu_1_return;

lltu_1_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nn_rl_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nn_rl_one_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+4+idx*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lltu_1_return;

lltu_1_return:
	return;



lunu_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb);
		}
	if(ii<m)
		{
		kernel_spack_tn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, m-ii);
		}

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nt_ru_one_4x4_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunu_2_left_4;
		}
	goto lunu_2_return;

lunu_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunu_2_return;

lunu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_lunn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_lunn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | m>K_MAX_STACK)
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

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nt_ru_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nt_ru_inv_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunn_1_left_4;
		}
	goto lunn_1_return;

lunn_1_left_4:
	kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, dA+idx, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunn_1_return;

lunn_1_return:
	return;



lunn_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps4, sdb);
		}
	if(ii<m)
		{
		kernel_spack_tt_4_vs_lib4(m, A+ii*lda, lda, pB+ii*ps4, sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nt_ru_inv_4x4_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunn_2_left_4;
		}
	goto lunn_2_return;

lunn_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, dB+idx, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunn_2_return;

lunn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_lunu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_lunu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | m>K_MAX_STACK)
		{
		goto lunu_2;
		}
	else
		{
		goto lunu_1;
		}

	// never to get here
	return;

lunu_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nt_ru_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nt_ru_one_4x4_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunu_1_left_4;
		}
	goto lunu_1_return;

lunu_1_left_4:
	kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4c44c(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4c44c(jj+mn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, pU+idx*ps4, pU+idx*ps4, A+idx+idx*lda, lda, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunu_1_return;

lunu_1_return:
	return;



lunu_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to upper
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps4, sdb);
		}
	if(ii<m)
		{
		kernel_spack_tt_4_vs_lib4(m, A+ii*lda, lda, pB+ii*ps4, sdb, m-ii);
		}

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	mn4 = m%4;
	m4 = m - mn4;

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		if(mn4!=0)
			{
			idx = m4;
			kernel_strsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, n-ii, mn4);
			}
		for(jj=0; jj<m4-3; jj+=4)
			{
			idx = m4-jj-4;
			kernel_strsm_nt_ru_one_4x4_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lunu_2_left_4;
		}
	goto lunu_2_return;

lunu_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	if(mn4!=0)
		{
		idx = m4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4(0, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, n-ii, mn4);
		}
	for(jj=0; jj<m4-3; jj+=4)
		{
		idx = m4-jj-4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4(jj+mn4, pU+(idx+4)*ps4, pB+(idx+4)*ps4+idx*sdb, &alpha, pU+idx*ps4, pU+idx*ps4, pB+idx*ps4+idx*sdb, n-ii, 4);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
	goto lunu_2_return;

lunu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_lutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_lutn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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



lutn:
	if(m>=12 | n>=12 | m>K_MAX_STACK)
		{
		goto llnn_2;
		}
	else
		{
		goto lutn_1;
		}

	// never to get here
	return;

lutn_1:
	// XXX limits of ii and jj swapped !!!
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<m; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nn_ru_inv_4x4_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, dA+jj);
			}
		if(jj<m)
			{
			kernel_strsm_nn_ru_inv_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lutn_1_left_4;
		}
	goto lutn_1_return;

lutn_1_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nn_ru_inv_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, dA+jj, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto lutn_1_return;

lutn_1_return:
	return;



llnn_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to lower
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
		}
	if(ii<m)
		{
		kernel_spack_tn_4_vs_lib4(m, A+ii*lda, lda, pB+ii*sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, dB+jj);
			}
		if(jj<m)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, dB+jj, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnn_2_left_4;
		}
	goto llnn_2_return;

llnn_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, dB+jj, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnn_2_return;

llnn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_lutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_lutu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldb;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;

#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
		k0 = m;
//	else
//		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | m>K_MAX_STACK)
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
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nn_ru_one_4x4_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda);
			}
		if(jj<m)
			{
			kernel_strsm_nn_ru_one_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto lutu_1_left_4;
		}
	goto lutu_1_return;

lutu_1_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nn_ru_one_4x4_vs_lib4c44c(jj, pU, A+jj*lda, lda, &alpha, pU+jj*ps4, pU+jj*ps4, A+jj+jj*lda, lda, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto lutu_1_return;

lutu_1_return:
	return;



llnu_2:
	m1 = (m+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_smat(ps0, m1, m1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, m, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, m, m, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to lower
	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
		}
	if(ii<m)
		{
		kernel_spack_tn_4_vs_lib4(m, A+ii*lda, lda, pB+ii*sdb, m-ii);
		}

	for(ii=0; ii<m; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_strsm_nt_rl_one_4x4_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb);
			}
		if(jj<m)
			{
			kernel_strsm_nt_rl_one_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, n-ii, m-jj);
			}
		kernel_sunpack_nt_4_lib4(m, pU, D+ii*ldd, ldd);
		}
	if(ii<n)
		{
		goto llnu_2_left_4;
		}
	goto llnu_2_return;

llnu_2_left_4:
	kernel_spack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_strsm_nt_rl_one_4x4_vs_lib4(jj, pU, tB.pA+jj*sdb, &alpha, pU+jj*ps4, pU+jj*ps4, tB.pA+jj*ps4+jj*sdb, n-ii, m-jj);
		}
	kernel_sunpack_nt_4_vs_lib4(m, pU, D+ii*ldd, ldd, n-ii);
goto llnu_2_return;

llnu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_rlnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_rlnn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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


rlnn:
	if(m>=12 | n>=12 | n>K_MAX_STACK)
		{
		goto rutn_2;
		}
	else
		{
		goto rlnn_1;
		}

	// never to get here
	return;

rlnn_1:
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nn_rl_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nn_rl_inv_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rlnn_1_left_4;
		}
	goto rlnn_1_return;

rlnn_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nn_rl_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nn_rl_inv_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rlnn_1_return;

rlnn_1_return:
	return;



rutn_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb);
		}
	if(ii<n)
		{
		kernel_spack_tn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, n-ii);
		}

	for(ii=0; ii<n; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nt_ru_inv_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nt_ru_inv_4x4_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rutn_2_left_4;
		}
	goto rutn_2_return;

rutn_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rutn_2_return;

rutn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_rlnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_rlnu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | n>K_MAX_STACK)
		{
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

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nn_rl_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nn_rl_one_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rlnu_1_left_4;
		}
	goto rlnu_1_return;

rlnu_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nn_rl_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nn_rl_one_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+(idx+4)+idx*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rlnu_1_return;

rlnu_1_return:
	return;



rutu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb);
		}
	if(ii<n)
		{
		kernel_spack_tn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps4+ii*sdb, n-ii);
		}

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nt_ru_one_4x4_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rutu_2_left_4;
		}
	goto rutu_2_return;

rutu_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nt_ru_one_4x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rutu_2_return;

rutu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_rltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_rltn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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


rltn:
	if(m>=12 | n>=12 | n>K_MAX_STACK)
		{
		goto rltn_2;
		}
	else
		{
		goto rltn_1;
		}

	// never to get here
	return;

rltn_1:
	pU = pU0;
	sdu = sdu0;
	dA = pd0;

	for(ii=0; ii<n; ii++)
		dA[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto rltn_1_left_4;
		}
	goto rltn_1_return;

rltn_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto rltn_1_return;

rltn_1_return:
	return;



rltn_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps4, sdb);
		}
	if(ii<n)
		{
		kernel_spack_tt_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps4, sdb, n-ii);
		}

	for(ii=0; ii<n; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, dB+jj);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto rltn_2_left_4;
		}
	goto rltn_2_return;

rltn_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto rltn_2_return;

rltn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_rltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_rltu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | n>K_MAX_STACK)
		{
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
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nt_rl_one_4x4_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nt_rl_one_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto rltu_1_left_4;
		}
	goto rltu_1_return;

rltu_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nt_rl_one_4x4_vs_lib4cccc(jj, pU, A+jj, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto rltu_1_return;

rltu_1_return:
	return;



rltu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// lower to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps4, sdb);
		}
	if(ii<n)
		{
		kernel_spack_tt_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps4, sdb, n-ii);
		}

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nt_rl_one_4x4_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto rltu_2_left_4;
		}
	goto rltu_2_return;

rltu_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto rltu_2_return;

rltu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_runn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_runn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | n>K_MAX_STACK)
		{
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
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nn_ru_inv_4x4_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto runn_1_left_4;
		}
	goto runn_1_return;

runn_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, dA+jj, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto runn_1_return;

runn_1_return:
	return;



rltn_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to lower
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
		}
	if(ii<n)
		{
		kernel_spack_tn_4_vs_lib4(n, A+ii*lda, lda, pB+ii*sdb, n-ii);
		}

	for(ii=0; ii<n; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	ii = 0;
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, dB+jj);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, dB+jj, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto rltn_2_left_4;
		}
	goto rltn_2_return;

rltn_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, dB+jj, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto rltn_2_return;

rltn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_runu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_runu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | n>K_MAX_STACK)
		{
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
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nn_ru_one_4x4_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nn_ru_one_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto runu_1_left_4;
		}
	goto runu_1_return;

runu_1_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nn_ru_one_4x4_vs_lib4cccc(jj, pU, A+jj*lda, lda, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, A+jj+jj*lda, lda, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto runu_1_return;

runu_1_return:
	return;



rltu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to lower
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
		}
	if(ii<n)
		{
		kernel_spack_tn_4_vs_lib4(n, A+ii*lda, lda, pB+ii*sdb, n-ii);
		}

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_strsm_nt_rl_one_4x4_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb);
			kernel_spack_nn_4_lib4(4, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		if(jj<n)
			{
			kernel_strsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, m-ii, n-jj);
//			kernel_spack_nn_4_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4);
			}
		}
	if(ii<m)
		{
		goto rltu_2_left_4;
		}
	goto rltu_2_return;

rltu_2_left_4:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_strsm_nt_rl_one_4x4_vs_lib44cc4(jj, pU, pB+jj*sdb, &alpha, B+ii+jj*ldb, ldb, D+ii+jj*ldd, ldd, pB+jj*ps4+jj*sdb, m-ii, n-jj);
		kernel_spack_nn_4_vs_lib4(n-jj, D+ii+jj*ldd, ldd, pU+jj*ps4, m-ii);
		}
goto rltu_2_return;

rltu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_rutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_rutn (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | n>K_MAX_STACK)
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

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nt_ru_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nt_ru_inv_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rutn_1_left_4;
		}
	goto rutn_1_return;

rutn_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, dA+idx, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rutn_1_return;

rutn_1_return:
	return;



rutn_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps4, sdb);
		}
	if(ii<n)
		{
		kernel_spack_tt_4_vs_lib4(n, A+ii*lda, lda, pB+ii*ps4, sdb, n-ii);
		}

	for(ii=0; ii<n; ii++)
		dB[ii] = 1.0/A[ii+ii*lda];

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nt_ru_inv_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nt_ru_inv_4x4_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rutn_2_left_4;
		}
	goto rutn_2_return;

rutn_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, dB+idx, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rutn_2_return;

rutn_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



void blasfeo_hp_strsm_rutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_strsm_rutu (cm) %d %d %f %p %d %d %p %d %d %p %d %d\n", m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	float *A = sA->pA + ai + aj*lda;
	float *B = sB->pA + bi + bj*ldb;
	float *D = sD->pA + di + dj*ldd;

//	printf("\n%p %d %p %d %p %d\n", A, lda, B, ldb);

	int ii, jj;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
	int ps0 = 8;
#else
	int ps0 = 4;
#endif
	int ps4 = 4;
	int ps8 = 8;


#if defined(TARGET_GENERIC)
	float pd0[K_MAX_STACK];
#else
	ALIGNED( float pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_GENERIC)
	float pU0[M_KERNEL*K_MAX_STACK];
#else
	ALIGNED( float pU0[M_KERNEL*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
//	if(*side=='l' | *side=='L')
//		k0 = m;
//	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_smat tA, tB;
	int sda, sdb;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	float *pU, *dA, *pB, *dB;
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
	if(m>=12 | n>=12 | n>K_MAX_STACK)
		{
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

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nt_ru_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nt_ru_one_4x4_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rutu_1_left_4;
		}
	goto rutu_1_return;

rutu_1_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4cccc(0, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nt_ru_one_4x4_vs_lib4cccc(jj+nn4, pU+(idx+4)*ps4, A+idx+(idx+4)*lda, lda, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, A+idx+idx*lda, lda, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rutu_1_return;

rutu_1_return:
	return;



rutu_2:
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_smat(ps0, m_kernel, n1);
	tB_size = blasfeo_pm_memsize_smat(ps0, n1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_pm_create_smat(ps0, m_kernel, n, &tA, (void *) mem_align);
	blasfeo_pm_create_smat(ps0, n, n, &tB, (void *) (mem_align+tA_size));

	pU = tA.pA;
	sdu = tA.cn;
	pB = tB.pA;
	sdb = tB.cn;
	dB = tB.dA;

	// upper to upper
	for(ii=0; ii<n-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps4, sdb);
		}
	if(ii<n)
		{
		kernel_spack_tt_4_vs_lib4(n, A+ii*lda, lda, pB+ii*ps4, sdb, n-ii);
		}

	// XXX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nn4 = n%4;
	n4 = n - nn4;

	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		if(nn4!=0)
			{
			idx = n4;
			kernel_strsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, m-ii, nn4);
			kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
			}
		for(jj=0; jj<n4-3; jj+=4)
			{
			idx = n4-jj-4;
			kernel_strsm_nt_ru_one_4x4_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4);
			kernel_spack_nn_4_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4);
			}
		}
	if(ii<m)
		{
		goto rutu_2_left_4;
		}
	goto rutu_2_return;

rutu_2_left_4:
	if(nn4!=0)
		{
		idx = n4;
		kernel_strsm_nt_ru_one_4x4_vs_lib44cc4(0, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, m-ii, nn4);
		kernel_spack_nn_4_vs_lib4(nn4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	for(jj=0; jj<n4-3; jj+=4)
		{
		idx = n4-jj-4;
		kernel_strsm_nt_ru_one_4x4_vs_lib44cc4(jj+nn4, pU+(idx+4)*ps4, pB+idx*sdb+(idx+4)*ps4, &alpha, B+ii+idx*ldb, ldb, D+ii+idx*ldd, ldd, pB+idx*sdb+idx*ps4, m-ii, 4);
		kernel_spack_nn_4_vs_lib4(4, D+ii+idx*ldd, ldd, pU+idx*ps4, m-ii);
		}
	goto rutu_2_return;

rutu_2_return:
	free(mem);
	return;



	// never to get here
	return;

	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_strsm_llnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_llnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_llnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_llnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_lltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_lltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_lunn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lunn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_lunu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lunu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_lutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_lutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_rlnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rlnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_rlnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rlnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_rltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_rltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_runn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_runn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_runu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_runu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_rutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



void blasfeo_strsm_rutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
	}



#endif
