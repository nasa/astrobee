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
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_d_blasfeo_ref_api.h>
#endif



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_hp_dgetrf_rp blasfeo_hp_cm_dgetrf_rp
#define blasfeo_hp_dgetrf_np blasfeo_hp_cm_dgetrf_np
#define blasfeo_dgetrf_rp blasfeo_cm_dgetrf_rp
#define blasfeo_dgetrf_np blasfeo_cm_dgetrf_np
#endif



// TODO move to a header file to reuse across routines
#define EL_SIZE 8 // double precision

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
#define M_KERNEL 12 // max kernel: 12x4
#define N_KERNEL 8 // max kernel: 8x8
#define L1_CACHE_EL (32*1024/EL_SIZE) // L1 data cache size: 32 kB
#define CACHE_LINE_EL (64/EL_SIZE) // data cache size: 64 bytes

#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
#define M_KERNEL 8 // max kernel: 8x4
#define N_KERNEL 4 // max kernel: 8x4
#define L1_CACHE_EL (32*1024/EL_SIZE) // L1 data cache size: 32 kB
#define CACHE_LINE_EL (64/EL_SIZE) // data cache size: 64 bytes

#else // assume generic target
#define M_KERNEL 4 // max kernel: 4x4
#define N_KERNEL 4 // max kernel: 4x4
#define L1_CACHE_EL (32*1024/EL_SIZE) // L1 data cache size: 32 kB
#define CACHE_LINE_EL (64/EL_SIZE) // data cache size: 64 bytes // TODO 32-bytes for cortex A9
#endif



void blasfeo_hp_dgetrf_rp(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, int *ipiv)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgetrf_rp (cm) %d %d %p %d %d %p %d %d %p\n", m, n, sC, ci, cj, sD, di, dj, ipiv);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int ldc0 = sC->m;
	int ldc = sD->m;
	double *C0 = sC->pA + ci + cj*ldc0;
	double *C = sD->pA + di + dj*ldc;

//	printf("\n%p %d %p %d\n", C, ldc, D, ldd);

	int ii, jj;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif
	int sdu0 = (m+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	double *dA;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *pC, *pd;
	int sdu, sdc;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


	int p = m<n ? m : n;

	int m_max, n_max;

	double d1 = 1.0;
	double dm1 = -1.0;

	double *dummy = NULL;



	if( C!=C0 )
		{
		for(jj=0; jj<n; jj++)
			{
			// TODO use copy_ instead !!!!!!!!!
			for(ii=0; ii<m; ii++)
				{
				C[ii+ldc*jj] = C0[ii+ldc0*jj];
				}
			}
		}



//		goto alg1;
//		goto alg2;
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>300 | n>300 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>240 | n>240 | m>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
		goto alg2;
		}
	else
		{
		goto alg1;
		}



alg1:

	pU = pU0;
	sdu = sdu0;
	pd = pd0;

	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m<=12)
		{
		if(m<=4)
			{
			goto edge_m_4_1;
			}
		else if(m<=8)
			{
			goto edge_m_8_1;
			}
		else //if(m<=12)
			{
			goto edge_m_12_1;
			}
		}
	for(; jj<n-11; jj+=12)
		{

		m_max = m<jj ? m : jj;
		n_max = p-jj<12 ? p-jj : 12;

		// pack
		kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
		kernel_dpack_tn_4_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m_max, C+(jj+8)*ldc, ldc, pU+8*sdu);

		// solve upper
		for(ii=0; ii<m_max-3; ii+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4c44c(ii, pU, sdu, C+ii, ldc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, C+ii+ii*ldc, ldc);
			}
		if(ii<m_max)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4c44c(ii, pU, sdu, C+ii, ldc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, C+ii+ii*ldc, ldc, 12, m_max-ii);
			}

		// unpack
		kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
		kernel_dunpack_nt_4_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc);
		kernel_dunpack_nt_4_lib4(m_max, pU+8*sdu, C+(jj+8)*ldc, ldc);

		// correct
		ii = jj;
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x12_libc4cc(jj, &dm1, C+ii, ldc, pU, sdu, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nt_4x12_vs_libc4cc(jj, &dm1, C+ii, ldc, pU, sdu, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}

		// pivot & factorize & solve
		if(m-jj>=12)
			{
			kernel_dgetrf_pivot_12_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj);
			}
		else
			{
			kernel_dgetrf_pivot_12_vs_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj, n-jj);
			}
		for(ii=0; ii<n_max; ii++)
			{
			ipiv[jj+ii] += jj;
			}

		// pivot
		for(ii=0; ii<n_max; ii++)
			{
			if(ipiv[jj+ii]!=jj+ii)
				{
				kernel_drowsw_lib(jj, C+jj+ii, ldc, C+ipiv[jj+ii], ldc);
				kernel_drowsw_lib(n-jj-12, C+jj+ii+(jj+12)*ldc, ldc, C+ipiv[jj+ii]+(jj+12)*ldc, ldc);
				}
			}

		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto left_4_1;
			}
		if(n-jj<=8)
			{
			goto left_8_1;
			}
		else
			{
			goto left_12_1;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m<=8)
		{
		if(m<=4)
			{
			goto edge_m_4_1;
			}
		else //if(m<=8)
			{
			goto edge_m_8_1;
			}
		}
	for(; jj<n-7; jj+=8)
		{

		m_max = m<jj ? m : jj;
		n_max = p-jj<8 ? p-jj : 8;

		// pack
		kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
		kernel_dpack_tn_4_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu);

		// solve upper
		for(ii=0; ii<m_max-3; ii+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib4c44c(ii, pU, sdu, C+ii, ldc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, C+ii+ii*ldc, ldc);
			}
		if(ii<m_max)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4c44c(ii, pU, sdu, C+ii, ldc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, C+ii+ii*ldc, ldc, 8, m_max-ii);
			}

		// unpack
		kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
		kernel_dunpack_nt_4_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc);

		// correct
		ii = jj;
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x8_libc4cc(jj, &dm1, C+ii, ldc, pU, sdu, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nt_4x8_vs_libc4cc(jj, &dm1, C+ii, ldc, pU, sdu, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}

		// pivot & factorize & solve
		if(m-jj>=8)
			{
			kernel_dgetrf_pivot_8_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj);
			}
		else
			{
			kernel_dgetrf_pivot_8_vs_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj, n-jj);
			}
		for(ii=0; ii<n_max; ii++)
			{
			ipiv[jj+ii] += jj;
			}

		// pivot
		for(ii=0; ii<n_max; ii++)
			{
			if(ipiv[jj+ii]!=jj+ii)
				{
				kernel_drowsw_lib(jj, C+jj+ii, ldc, C+ipiv[jj+ii], ldc);
				kernel_drowsw_lib(n-jj-8, C+jj+ii+(jj+8)*ldc, ldc, C+ipiv[jj+ii]+(jj+8)*ldc, ldc);
				}
			}

		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto left_4_1;
			}
		else
			{
			goto left_8_1;
			}
		}
#else
	if(m<=4)
		{
		goto edge_m_4_1;
		}
	for(; jj<n-3; jj+=4)
		{

		m_max = m<jj ? m : jj;
		n_max = p-jj<4 ? p-jj : 4;

		// pack
		kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);

		// solve upper
		for(ii=0; ii<m_max-3; ii+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib4c44c(ii, pU, C+ii, ldc, &d1, pU+ii*ps, pU+ii*ps, C+ii+ii*ldc, ldc);
			}
		if(ii<m_max)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib4c44c(ii, pU, C+ii, ldc, &d1, pU+ii*ps, pU+ii*ps, C+ii+ii*ldc, ldc, 4, m_max-ii);
			}

		// unpack
		kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);

		// correct
		ii = jj;
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x4_libc4cc(jj, &dm1, C+ii, ldc, pU, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nt_4x4_vs_libc4cc(jj, &dm1, C+ii, ldc, pU, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}

		// pivot & factorize & solve
		if(m-jj>=4)
			{
			kernel_dgetrf_pivot_4_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj);
			}
		else
			{
			kernel_dgetrf_pivot_4_vs_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj, n-jj);
			}
		for(ii=0; ii<n_max; ii++)
			{
			ipiv[jj+ii] += jj;
			}

		// apply pivot
		for(ii=0; ii<n_max; ii++)
			{
			if(ipiv[jj+ii]!=jj+ii)
				{
				kernel_drowsw_lib(jj, C+jj+ii, ldc, C+ipiv[jj+ii], ldc);
				kernel_drowsw_lib(n-jj-4, C+jj+ii+(jj+4)*ldc, ldc, C+ipiv[jj+ii]+(jj+4)*ldc, ldc);
				}
			}

		}
	if(jj<n)
		{
		goto left_4_1;
		}
#endif
	goto end_1;



#if defined(TARGET_X64_INTEL_HASWELL)
left_12_1:

	m_max = m<jj ? m : jj;
	n_max = p-jj<12 ? p-jj : 12;

	// pack
	kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
	kernel_dpack_tn_4_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m_max, C+(jj+8)*ldc, ldc, pU+8*sdu, n-jj-8);

	// solve upper
	for(ii=0; ii<m_max; ii+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_lib4c44c(ii, pU, sdu, C+ii, ldc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, C+ii+ii*ldc, ldc);
		}

	// unpack
	kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
	kernel_dunpack_nt_4_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc);
	kernel_dunpack_nt_4_vs_lib4(m_max, pU+8*sdu, C+(jj+8)*ldc, ldc, n-jj-8);

	// correct
	ii = jj;
	for( ; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x12_vs_libc4cc(jj, &dm1, C+ii, ldc, pU, sdu, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}

	// pivot & factorize & solve
	kernel_dgetrf_pivot_12_vs_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj, n-jj);
	for(ii=0; ii<n_max; ii++)
		{
		ipiv[jj+ii] += jj;
		}

	// pivot
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[jj+ii]!=jj+ii)
			{
			kernel_drowsw_lib(jj, C+jj+ii, ldc, C+ipiv[jj+ii], ldc);
//			kernel_drowsw_lib(n-jj-12, C+jj+ii+(jj+12)*ldc, ldc, C+ipiv[jj+ii]+(jj+12)*ldc, ldc);
			}
		}

	goto end_1;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
left_8_1:

	m_max = m<jj ? m : jj;
	n_max = p-jj<8 ? p-jj : 8;

	// pack
	kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
	kernel_dpack_tn_4_vs_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu, n-jj-4);

	// solve upper
	for(ii=0; ii<m_max; ii+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_lib4c44c(ii, pU, sdu, C+ii, ldc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, C+ii+ii*ldc, ldc);
		}

	// unpack
	kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
	kernel_dunpack_nt_4_vs_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc, n-jj-4);

	// correct
	ii = jj;
	for( ; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x8_vs_libc4cc(jj, &dm1, C+ii, ldc, pU, sdu, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}

	// pivot & factorize & solve
	kernel_dgetrf_pivot_8_vs_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj, n-jj);
	for(ii=0; ii<n_max; ii++)
		{
		ipiv[jj+ii] += jj;
		}

	// pivot
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[jj+ii]!=jj+ii)
			{
			kernel_drowsw_lib(jj, C+jj+ii, ldc, C+ipiv[jj+ii], ldc);
//			kernel_drowsw_lib(n-jj-8, C+jj+ii+(jj+8)*ldc, ldc, C+ipiv[jj+ii]+(jj+8)*ldc, ldc);
			}
		}


	goto end_1;
#endif



left_4_1:

	m_max = m<jj ? m : jj;
	n_max = p-jj<4 ? p-jj : 4;

	// pack
	kernel_dpack_tn_4_vs_lib4(m_max, C+jj*ldc, ldc, pU, n-jj);

	// solve upper
	for(ii=0; ii<m_max; ii+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib4c44c(ii, pU, C+ii, ldc, &d1, pU+ii*ps, pU+ii*ps, C+ii+ii*ldc, ldc, n-jj, m_max-ii);
		}

	// unpack
	kernel_dunpack_nt_4_vs_lib4(m_max, pU, C+jj*ldc, ldc, n-jj);

	// correct
	ii = jj;
	for( ; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x4_vs_libc4cc(jj, &dm1, C+ii, ldc, pU, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}

	// pivot & factorize & solve
	kernel_dgetrf_pivot_4_vs_lib(m-jj, C+jj+jj*ldc, ldc, pd+jj, ipiv+jj, n-jj);
	for(ii=0; ii<n_max; ii++)
		{
		ipiv[jj+ii] += jj;
		}

	// apply pivot
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[jj+ii]!=jj+ii)
			{
			kernel_drowsw_lib(jj, C+jj+ii, ldc, C+ipiv[jj+ii], ldc);
//			kernel_drowsw_lib(n-jj-4, C+jj+ii+(jj+4)*ldc, ldc, C+ipiv[jj+ii]+(jj+4)*ldc, ldc);
			}
		}

	goto end_1;



#if defined(TARGET_X64_INTEL_HASWELL)
	// handle cases m={9,10,11,12}
edge_m_12_1:
	kernel_dgetrf_pivot_12_vs_lib(m, C, ldc, pd, ipiv, n); // it must handle also n={1,2,3,4} !!!!!
	for(ii=0; ii<p; ii++)
		{
		kernel_drowsw_lib(n-12, C+ii+12*ldc, ldc, C+ipiv[ii]+12*ldc, ldc);
		}
	for(ii=12; ii<n; ii+=4)
		{
		kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(0, dummy, 0, dummy, 0, &d1, C+ii*ldc, ldc, C+ii*ldc, ldc, C, ldc, m, n-ii);
		}
	goto end_1;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	// handle cases m={5,6,7,8}
edge_m_8_1:
	kernel_dgetrf_pivot_8_vs_lib(m, C, ldc, pd, ipiv, n); // it must handle also n={1,2,3,4} !!!!!
	for(ii=0; ii<p; ii++)
		{
		kernel_drowsw_lib(n-8, C+ii+8*ldc, ldc, C+ipiv[ii]+8*ldc, ldc);
		}
	for(ii=8; ii<n; ii+=4)
		{
		kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc(0, dummy, 0, dummy, 0, &d1, C+ii*ldc, ldc, C+ii*ldc, ldc, C, ldc, m, n-ii);
		}
	goto end_1;
#endif



	// handle cases m={1,2,3,4}
edge_m_4_1:
	kernel_dgetrf_pivot_4_vs_lib(m, C, ldc, pd, ipiv, n);
	for(ii=0; ii<p; ii++)
		{
		kernel_drowsw_lib(n-4, C+ii+4*ldc, ldc, C+ipiv[ii]+4*ldc, ldc);
		}
	for(ii=4; ii<n; ii+=4)
		{
		kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc(0, dummy, dummy, 0, &d1, C+ii*ldc, ldc, C+ii*ldc, ldc, C, ldc, m, n-ii);
		}
	goto end_1;



end_1:
	// from 0-index to 1-index
	// TODO move to BLAS_API !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	for(ii=0; ii<p; ii++)
//		ipiv[ii] += 1;
//	int_print_mat(1, p, ipiv, 1);
//	d_print_mat(m, n, C, ldc);
	return;



alg2:

	m1 = (m+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	tA_size = blasfeo_pm_memsize_dmat(ps, m_kernel, m1);
	tB_size = blasfeo_pm_memsize_dmat(ps, m1, n1);
	mem = malloc(tA_size+tB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);

	blasfeo_pm_create_dmat(ps, m_kernel, m, &tA, mem_align);
	pU = tA.pA;
	sdu = tA.cn;

	blasfeo_pm_create_dmat(ps, m, n, &tB, mem_align+tA_size);
	pC = tB.pA;
	sdc = tB.cn;
	pd = tB.dA;


	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m<=12)
		{
		if(m<=4)
			{
			goto edge_m_4_2;
			}
		else if(m<=8)
			{
			goto edge_m_8_2;
			}
		else //if(m<=12)
			{
			goto edge_m_12_2;
			}
		}
	for(; jj<n-11; jj+=12)
		{
		
		m_max = m<jj ? m : jj;
		n_max = p-jj<12 ? p-jj : 12;

		// pack
		kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
		kernel_dpack_tn_4_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m_max, C+(jj+8)*ldc, ldc, pU+8*sdu);

		// solve upper
		for(ii=0; ii<m_max-3; ii+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4(ii, pU, sdu, pC+ii*sdc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, pC+ii*sdc+ii*ps);
			}
		if(ii<m_max)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4(ii, pU, sdu, pC+ii*sdc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, pC+ii*sdc+ii*ps, 12, m_max-ii);
			}

		// unpack
		kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
		kernel_dunpack_nt_4_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc);
		kernel_dunpack_nt_4_lib4(m_max, pU+8*sdu, C+(jj+8)*ldc, ldc);

		// pack
		kernel_dpack_tt_12_lib4(m-jj, C+jj+jj*ldc, ldc, pC+jj*sdc+jj*ps, sdc);

		// correct
		ii = jj;
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x12_lib4(jj, &dm1, pC+ii*sdc, pU, sdu, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nt_4x12_vs_lib4(jj, &dm1, pC+ii*sdc, pU, sdu, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc, m-ii, n-jj);
			}

		// pivot & factorize & solve
		if(m-jj>=12)
			{
			kernel_dgetrf_pivot_12_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj);
			}
		else
			{
			kernel_dgetrf_pivot_12_vs_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj, n-jj);
			}
		for(ii=0; ii<n_max; ii++)
			{
			ipiv[jj+ii] += jj;
			}
		// unpack
		if(m-jj<=4)
			{
			kernel_dunpack_nn_4_vs_lib4(12, pC+jj*sdc+jj*ps, C+jj+jj*ldc, ldc, m-jj);
			}
		else if(m-jj<=8)
			{
			kernel_dunpack_nn_8_vs_lib4(12, pC+jj*sdc+jj*ps, sdc, C+jj+jj*ldc, ldc, m-jj);
			}
		else
			{
			kernel_dunpack_nn_12_vs_lib4(12, pC+jj*sdc+jj*ps, sdc, C+jj+jj*ldc, ldc, m-jj);
			}

		// pivot
		for(ii=0; ii<n_max; ii++)
			{
			if(ipiv[jj+ii]!=jj+ii)
				{
				// TODO use kernel
//				blasfeo_drowsw(jj, &sC, jj+ii, 0, &sC, ipiv[jj+ii], 0);
				kernel_drowsw_lib4(jj, pC+(jj+ii)/ps*ps*sdc+(jj+ii)%ps, pC+ipiv[jj+ii]/ps*ps*sdc+ipiv[jj+ii]%ps);
				kernel_drowsw_lib(n-jj-12, C+jj+ii+(jj+12)*ldc, ldc, C+ipiv[jj+ii]+(jj+12)*ldc, ldc);
				}
			}

		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto left_4_2;
			}
		else if(n-jj<=8)
			{
			goto left_8_2;
			}
		else
			{
			goto left_12_2;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if(m<=8)
		{
		if(m<=4)
			{
			goto edge_m_4_2;
			}
		else //if(m<=8)
			{
			goto edge_m_8_2;
			}
		}
	for(; jj<n-7; jj+=8)
		{
		
		m_max = m<jj ? m : jj;
		n_max = p-jj<8 ? p-jj : 8;

		// pack
		kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
		kernel_dpack_tn_4_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu);

		// solve upper
		for(ii=0; ii<m_max-3; ii+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib4(ii, pU, sdu, pC+ii*sdc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, pC+ii*sdc+ii*ps);
			}
		if(ii<m_max)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4(ii, pU, sdu, pC+ii*sdc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, pC+ii*sdc+ii*ps, 8, m_max-ii);
			}

		// unpack
		kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
		kernel_dunpack_nt_4_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc);

		// pack
		kernel_dpack_tt_8_lib4(m-jj, C+jj+jj*ldc, ldc, pC+jj*sdc+jj*ps, sdc);

		// correct
		ii = jj;
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x8_lib4(jj, &dm1, pC+ii*sdc, pU, sdu, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nt_4x8_vs_lib4(jj, &dm1, pC+ii*sdc, pU, sdu, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc, m-ii, n-jj);
			}

		// pivot & factorize & solve
		if(m-jj>=8)
			{
			kernel_dgetrf_pivot_8_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj);
			}
		else
			{
			kernel_dgetrf_pivot_8_vs_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj, n-jj);
			}
		for(ii=0; ii<n_max; ii++)
			{
			ipiv[jj+ii] += jj;
			}
		// unpack
		if(m-jj<=4)
			{
			kernel_dunpack_nn_4_vs_lib4(8, pC+jj*sdc+jj*ps, C+jj+jj*ldc, ldc, m-jj);
			}
		else
			{
			kernel_dunpack_nn_8_vs_lib4(8, pC+jj*sdc+jj*ps, sdc, C+jj+jj*ldc, ldc, m-jj);
			}

		// pivot
		for(ii=0; ii<n_max; ii++)
			{
			if(ipiv[jj+ii]!=jj+ii)
				{
				// TODO use kernel
//				blasfeo_drowsw(jj, &sC, jj+ii, 0, &sC, ipiv[jj+ii], 0);
				kernel_drowsw_lib4(jj, pC+(jj+ii)/ps*ps*sdc+(jj+ii)%ps, pC+ipiv[jj+ii]/ps*ps*sdc+ipiv[jj+ii]%ps);
				kernel_drowsw_lib(n-jj-8, C+jj+ii+(jj+8)*ldc, ldc, C+ipiv[jj+ii]+(jj+8)*ldc, ldc);
				}
			}

		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto left_4_2;
			}
		else //if(n-jj<=8)
			{
			goto left_8_2;
			}
		}
#else
	if(m<=4)
		{
		goto edge_m_4_2;
		}
	for(; jj<n-3; jj+=4)
		{

		m_max = m<jj ? m : jj;
		n_max = p-jj<4 ? p-jj : 4;

		// pack
		kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);

		// solve upper
		for(ii=0; ii<m_max-3; ii+=4)
			{
			kernel_dtrsm_nt_rl_one_4x4_lib4(ii, pU, pC+ii*sdc, &d1, pU+ii*ps, pU+ii*ps, pC+ii*sdc+ii*ps);
			}
		if(ii<m_max)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib4(ii, pU, pC+ii*sdc, &d1, pU+ii*ps, pU+ii*ps, pC+ii*sdc+ii*ps, 4, m_max-ii);
			}

		// unpack
		kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);

		// pack
		kernel_dpack_tt_4_lib4(m-jj, C+jj+jj*ldc, ldc, pC+jj*sdc+jj*ps, sdc);

		// correct
		ii = jj;
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgemm_nt_4x4_lib4(jj, &dm1, pC+ii*sdc, pU, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nt_4x4_vs_lib4(jj, &dm1, pC+ii*sdc, pU, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc, m-ii, n-jj);
			}

		// pivot & factorize & solve
		if(m-jj>=4)
			{
			kernel_dgetrf_pivot_4_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj);
			}
		else
			{
			kernel_dgetrf_pivot_4_vs_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj, n-jj);
			}
		for(ii=0; ii<n_max; ii++)
			{
			ipiv[jj+ii] += jj;
			}
		// unpack
		kernel_dunpack_nn_4_vs_lib4(4, pC+jj*sdc+jj*ps, C+jj+jj*ldc, ldc, m-jj);

		// pivot
		for(ii=0; ii<n_max; ii++)
			{
			if(ipiv[jj+ii]!=jj+ii)
				{
				// TODO use kernel
//				blasfeo_drowsw(jj, &sC, jj+ii, 0, &sC, ipiv[jj+ii], 0);
				kernel_drowsw_lib4(jj, pC+(jj+ii)/ps*ps*sdc+(jj+ii)%ps, pC+ipiv[jj+ii]/ps*ps*sdc+ipiv[jj+ii]%ps);
				kernel_drowsw_lib(n-jj-4, C+jj+ii+(jj+4)*ldc, ldc, C+ipiv[jj+ii]+(jj+4)*ldc, ldc);
				}
			}

		}
	if(jj<n)
		{
		goto left_4_2;
		}
#endif
	goto end_2;



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_12_2:

	m_max = m<jj ? m : jj;
	n_max = p-jj<12 ? p-jj : 12;

	// pack
	kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
	kernel_dpack_tn_4_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(m_max, C+(jj+8)*ldc, ldc, pU+8*sdu, n-jj-8);

	// solve upper
	for(ii=0; ii<m_max; ii+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_vs_lib4(ii, pU, sdu, pC+ii*sdc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, pC+ii*sdc+ii*ps, n-jj, m_max-ii);
		}

	// unpack
	kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
	kernel_dunpack_nt_4_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc);
	kernel_dunpack_nt_4_vs_lib4(m_max, pU+8*sdu, C+(jj+8)*ldc, ldc, n-jj-8);

	// pack
	kernel_dpack_tt_8_lib4(m-jj, C+jj+jj*ldc, ldc, pC+jj*sdc+jj*ps, sdc);
	kernel_dpack_tt_4_vs_lib4(m-jj, C+jj+(jj+8)*ldc, ldc, pC+jj*sdc+(jj+8)*ps, sdc, n-jj-8);

	// correct
	ii = jj;
	for( ; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x12_vs_lib4(jj, &dm1, pC+ii*sdc, pU, sdu, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc, m-ii, n-jj);
		}

	// pivot & factorize & solve
	kernel_dgetrf_pivot_12_vs_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj, n-jj);
	for(ii=0; ii<n_max; ii++)
		{
		ipiv[jj+ii] += jj;
		}
	// unpack
	if(m-jj<=4)
		{
		kernel_dunpack_nn_4_vs_lib4(n-jj, pC+jj*sdc+jj*ps, C+jj+jj*ldc, ldc, m-jj);
		}
	else if(m-jj<=8)
		{
		kernel_dunpack_nn_8_vs_lib4(n-jj, pC+jj*sdc+jj*ps, sdc, C+jj+jj*ldc, ldc, m-jj);
		}
	else
		{
		kernel_dunpack_nn_12_vs_lib4(n-jj, pC+jj*sdc+jj*ps, sdc, C+jj+jj*ldc, ldc, m-jj);
		}

	// pivot
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[jj+ii]!=jj+ii)
			{
			// TODO use kernel instead
//			blasfeo_drowsw(jj, &sC, jj+ii, 0, &sC, ipiv[jj+ii], 0);
			kernel_drowsw_lib4(jj, pC+(jj+ii)/ps*ps*sdc+(jj+ii)%ps, pC+ipiv[jj+ii]/ps*ps*sdc+ipiv[jj+ii]%ps);
//			kernel_drowsw_lib(n-jj-12, C+jj+ii+(jj+12)*ldc, ldc, C+ipiv[jj+ii]+(jj+12)*ldc, ldc);
			}
		}

	goto end_2;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_8_2:

	m_max = m<jj ? m : jj;
	n_max = p-jj<8 ? p-jj : 8;

	// pack
	kernel_dpack_tn_4_lib4(m_max, C+jj*ldc, ldc, pU);
	kernel_dpack_tn_4_vs_lib4(m_max, C+(jj+4)*ldc, ldc, pU+4*sdu, n-jj-4);

	// solve upper
	for(ii=0; ii<m_max; ii+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_vs_lib4(ii, pU, sdu, pC+ii*sdc, &d1, pU+ii*ps, sdu, pU+ii*ps, sdu, pC+ii*sdc+ii*ps, n-jj, m_max-ii);
		}

	// unpack
	kernel_dunpack_nt_4_lib4(m_max, pU, C+jj*ldc, ldc);
	kernel_dunpack_nt_4_vs_lib4(m_max, pU+4*sdu, C+(jj+4)*ldc, ldc, n-jj-4);

	// pack
	kernel_dpack_tt_4_lib4(m-jj, C+jj+jj*ldc, ldc, pC+jj*sdc+jj*ps, sdc);
	kernel_dpack_tt_4_vs_lib4(m-jj, C+jj+(jj+4)*ldc, ldc, pC+jj*sdc+(jj+4)*ps, sdc, n-jj-4);

	// correct
	ii = jj;
	for( ; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x8_vs_lib4(jj, &dm1, pC+ii*sdc, pU, sdu, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc, m-ii, n-jj);
		}

	// pivot & factorize & solve
	kernel_dgetrf_pivot_8_vs_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj, n-jj);
	for(ii=0; ii<n_max; ii++)
		{
		ipiv[jj+ii] += jj;
		}

	// unpack
	if(m-jj<=4)
		{
		kernel_dunpack_nn_4_vs_lib4(n-jj, pC+jj*sdc+jj*ps, C+jj+jj*ldc, ldc, m-jj);
		}
	else
		{
		kernel_dunpack_nn_8_vs_lib4(n-jj, pC+jj*sdc+jj*ps, sdc, C+jj+jj*ldc, ldc, m-jj);
		}

	// pivot
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[jj+ii]!=jj+ii)
			{
			// TODO use kernel instead
//			blasfeo_drowsw(jj, &sC, jj+ii, 0, &sC, ipiv[jj+ii], 0);
			kernel_drowsw_lib4(jj, pC+(jj+ii)/ps*ps*sdc+(jj+ii)%ps, pC+ipiv[jj+ii]/ps*ps*sdc+ipiv[jj+ii]%ps);
//			kernel_drowsw_lib(n-jj-8, C+jj+ii+(jj+8)*ldc, ldc, C+ipiv[jj+ii]+(jj+8)*ldc, ldc);
			}
		}

	goto end_2;
#endif



left_4_2:

	m_max = m<jj ? m : jj;
	n_max = p-jj<4 ? p-jj : 4;

	// pack
	kernel_dpack_tn_4_vs_lib4(m_max, C+jj*ldc, ldc, pU, n-jj);

	// solve upper
	for(ii=0; ii<m_max; ii+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib4(ii, pU, pC+ii*sdc, &d1, pU+ii*ps, pU+ii*ps, pC+ii*sdc+ii*ps, n-jj, m_max-ii);
		}

	// unpack
	kernel_dunpack_nt_4_vs_lib4(m_max, pU, C+jj*ldc, ldc, n-jj);

	// pack
	kernel_dpack_tt_4_vs_lib4(m-jj, C+jj+jj*ldc, ldc, pC+jj*sdc+jj*ps, sdc, n-jj);

	// correct
	ii = jj;
	for( ; ii<m; ii+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4(jj, &dm1, pC+ii*sdc, pU, &d1, pC+jj*ps+ii*sdc, pC+jj*ps+ii*sdc, m-ii, n-jj);
		}

	// pivot & factorize & solve
	kernel_dgetrf_pivot_4_vs_lib4(m-jj, pC+jj*sdc+jj*ps, sdc, pd+jj, ipiv+jj, n-jj);
	for(ii=0; ii<n_max; ii++)
		{
		ipiv[jj+ii] += jj;
		}
	// unpack
	kernel_dunpack_nn_4_vs_lib4(n-jj, pC+jj*sdc+jj*ps, C+jj+jj*ldc, ldc, m-jj);

	// pivot
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[jj+ii]!=jj+ii)
			{
			// TODO use kernel instead
//			blasfeo_drowsw(jj, &sC, jj+ii, 0, &sC, ipiv[jj+ii], 0);
			kernel_drowsw_lib4(jj, pC+(jj+ii)/ps*ps*sdc+(jj+ii)%ps, pC+ipiv[jj+ii]/ps*ps*sdc+ipiv[jj+ii]%ps);
//			kernel_drowsw_lib(n-jj-4, C+jj+ii+(jj+4)*ldc, ldc, C+ipiv[jj+ii]+(jj+4)*ldc, ldc);
			}
		}

	goto end_2;



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	// handle cases m={9,10,11,12}
edge_m_12_2:
	kernel_dgetrf_pivot_12_vs_lib(m, C, ldc, pd, ipiv, n); // it must handle also n={1,2,3,4} !!!!!
	for(ii=0; ii<p; ii++)
		{
		kernel_drowsw_lib(n-12, C+ii+12*ldc, ldc, C+ipiv[ii]+12*ldc, ldc);
		}
	for(ii=12; ii<n; ii+=4)
		{
		kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(0, dummy, 0, dummy, 0, &d1, C+ii*ldc, ldc, C+ii*ldc, ldc, C, ldc, m, n-ii);
		}
	goto end_m_2;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	// handle cases m={5,6,7,8}
edge_m_8_2:
	kernel_dgetrf_pivot_8_vs_lib(m, C, ldc, pd, ipiv, n); // it must handle also n={1,2,3,4} !!!!!
	for(ii=0; ii<p; ii++)
		{
		kernel_drowsw_lib(n-8, C+ii+8*ldc, ldc, C+ipiv[ii]+8*ldc, ldc);
		}
	for(ii=8; ii<n; ii+=4)
		{
		kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc(0, dummy, 0, dummy, 0, &d1, C+ii*ldc, ldc, C+ii*ldc, ldc, C, ldc, m, n-ii);
		}
	goto end_m_2;
#endif



	// handle cases m={1,2,3,4}
edge_m_4_2:
	kernel_dgetrf_pivot_4_vs_lib(m, C, ldc, pd, ipiv, n);
	for(ii=0; ii<p; ii++)
		{
		kernel_drowsw_lib(n-4, C+ii+4*ldc, ldc, C+ipiv[ii]+4*ldc, ldc);
		}
	for(ii=4; ii<n; ii+=4)
		{
		kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc(0, dummy, dummy, 0, &d1, C+ii*ldc, ldc, C+ii*ldc, ldc, C, ldc, m, n-ii);
		}
	goto end_m_2;



end_2:
	// unpack
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
#if 1
	for(; ii<m-11 & ii<n-11; ii+=12)
		{
		kernel_dunpack_nn_12_lib4(ii, pC+ii*sdc, sdc, C+ii, ldc);
		}
	for(; ii<m-11; ii+=12)
		{
		kernel_dunpack_nn_12_lib4(n, pC+ii*sdc, sdc, C+ii, ldc);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			kernel_dunpack_nn_4_vs_lib4(n, pC+ii*sdc, C+ii, ldc, m-ii);
			}
		else if(m-ii<=8)
			{
			kernel_dunpack_nn_8_vs_lib4(n, pC+ii*sdc, sdc, C+ii, ldc, m-ii);
			}
		else //if(m-ii<=12)
			{
			kernel_dunpack_nn_12_vs_lib4(n, pC+ii*sdc, sdc, C+ii, ldc, m-ii);
			}
		}
#else
	for(; ii<n-11; ii+=12)
		{
		kernel_dunpack_tt_4_lib4(m-ii-12, pC+(ii+12)*sdc+ii*ps, sdc, C+(ii+12)+ii*ldc, ldc);
		kernel_dunpack_tt_4_lib4(m-ii-12, pC+(ii+12)*sdc+(ii+4)*ps, sdc, C+(ii+12)+(ii+4)*ldc, ldc);
		kernel_dunpack_tt_4_lib4(m-ii-12, pC+(ii+12)*sdc+(ii+8)*ps, sdc, C+(ii+12)+(ii+8)*ldc, ldc);
		}
	// TODO
#endif
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7 & ii<n-7; ii+=8)
		{
		kernel_dunpack_nn_8_lib4(ii, pC+ii*sdc, sdc, C+ii, ldc);
		}
	for(; ii<m-7; ii+=8)
		{
		kernel_dunpack_nn_8_lib4(n, pC+ii*sdc, sdc, C+ii, ldc);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			kernel_dunpack_nn_4_vs_lib4(n, pC+ii*sdc, C+ii, ldc, m-ii);
			}
		else //if(m-ii<=8)
			{
			kernel_dunpack_nn_8_vs_lib4(n, pC+ii*sdc, sdc, C+ii, ldc, m-ii);
			}
		}
#else
	for(; ii<m-3 && ii<n-3; ii+=4)
		{
		kernel_dunpack_nn_4_lib4(ii, pC+ii*sdc, C+ii, ldc);
		}
	for(; ii<m-3; ii+=4)
		{
		kernel_dunpack_nn_4_lib4(n, pC+ii*sdc, C+ii, ldc);
		}
	if(ii<m)
		{
		kernel_dunpack_nn_4_vs_lib4(n, pC+ii*sdc, C+ii, ldc, m-ii);
		}
#endif
	// TODO clean loops

end_m_2:
	free(mem);
	// from 0-index to 1-index
	// TODO move to BLAS_API !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	for(ii=0; ii<p; ii++)
//		ipiv[ii] += 1;
	return;


	// never to get here
	return;

	}



void blasfeo_hp_dgetrf_np(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgetrf_np (cm) %d %d %p %d %d %p %d %d\n", m, n, sC, ci, cj, sD, di, dj);
#endif

	if(m<=0 | n<=0)
		return;

	// extract pointer to column-major matrices from structures
	int ldc0 = sC->m;
	int ldc = sD->m;
	double *C0 = sC->pA + ci + cj*ldc0;
	double *C = sD->pA + di + dj*ldc;

//	printf("\n%p %d %p %d\n", C, ldc, D, ldd);

	int ii, jj, ie;

	const int ps = 4; //D_PS;

#if defined(TARGET_GENERIC)
	double pU0[M_KERNEL*K_MAX_STACK];
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pU0[M_KERNEL*K_MAX_STACK], 64 );
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif
	int sdu0 = (m+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_pm_dmat tA, tB;
	int sda, sdb;
	double *dA;
	int tA_size, tB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	double *pU, *pC, *pd;
	int sdu, sdc;

	const int m_kernel = M_KERNEL;
	const int l1_cache_el = L1_CACHE_EL;
	const int reals_per_cache_line = CACHE_LINE_EL;

	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
//	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;


	int p = m<n ? m : n;

	int m_max, n_max;

	double d1 = 1.0;
	double dm1 = -1.0;

	double *dummy = NULL;



	// TODO
	if(m>K_MAX_STACK)
#if defined(TARGET_X64_INTEL_HASWELL)
//	if(m>300 | n>300 | m>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	if(m>240 | n>240 | m>K_MAX_STACK)
#else
//	if(m>=12 | n>=12 | m>K_MAX_STACK)
#endif
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dgetrf_np(m, n, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblas_dgetrf_np: not implemented yet for m>K_MAX_STACK\n");
		exit(1);
//		goto alg2;
#endif
		}
	else
		{
		goto alg1;
		}



alg1:

	pU = pU0;
	sdu = sdu0;
	pd = pd0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for( ; ii<m-11; ii+=12)
		{
		jj = 0;

		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_12x4_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_12_lib4(4, C+ii+jj*ldc, ldc, pU+jj*ps, sdu);
			}
		if(jj<ie)
			{
			kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_12_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			jj+=4;
			}

		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_nn_l_12x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_l_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n-3)
			{
			kernel_dgetrf_nn_m_12x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_m_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n-3)
			{
			kernel_dgetrf_nn_r_12x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_r_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_12x4_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}

		}
	if(m>ii)
		{
		if(m-ii<=4)
			{
			goto left_4_1;
			}
		if(m-ii<=8)
			{
			goto left_8_1;
			}
		else
			{
			goto left_12_1;
			}
		}
#elif 0
	for( ; ii<m-7; ii+=8)
		{
		jj = 0;

		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_8x4_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_8_lib4(4, C+ii+jj*ldc, ldc, pU+jj*ps, sdu);
			}
		if(jj<ie)
			{
			kernel_dtrsm_nn_ru_inv_8x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_8_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			jj+=4;
			}

		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_nn_l_8x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_l_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n-3)
			{
			kernel_dgetrf_nn_r_8x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_r_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_8x4_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}

		}
	if(m>ii)
		{
		if(m-ii<=4)
			{
			goto left_4_1;
			}
		else
			{
			goto left_8_1;
			}
		}
#else
	for( ; ii<m-3; ii+=4)
		{
		jj = 0;

		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_4x4_lib4cccc(jj, pU, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_4_lib4(4, C+ii+jj*ldc, ldc, pU+jj*ps);
			}
		if(jj<ie)
			{
			kernel_dtrsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_4_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, m-ii);
			jj+=4;
			}

		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_nn_4x4_lib4ccc(jj, pU, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_4x4_vs_lib4ccc(jj, pU, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_4x4_lib4cccc(ii, pU, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc(ii, pU, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}

		}
	if(m>ii)
		{
		goto left_4_1;
		}
#endif
	goto end_1;



#if defined(TARGET_X64_INTEL_HASWELL)
left_12_1:
		jj = 0;

		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_12_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			}

		// factorize
		if(jj<n)
			{
			kernel_dgetrf_nn_l_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n)
			{
			kernel_dgetrf_nn_m_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n)
			{
			kernel_dgetrf_nn_r_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}
	goto end_1;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_8_1:
		jj = 0;

		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_8x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_8_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			}

		// factorize
		if(jj<n)
			{
			kernel_dgetrf_nn_l_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n)
			{
			kernel_dgetrf_nn_r_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}
	goto end_1;
#endif



left_4_1:
	jj = 0;

	// solve lower
	ie = n<ii ? n : ii; // ie is multiple of 4
	for( ; jj<ie; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, m-ii);
		}

	// factorize
	if(jj<n)
		{
		kernel_dgetrf_nn_4x4_vs_lib4ccc(jj, pU, C+jj*ldc, ldc, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
		jj+=4;
		}

	// solve upper
	for( ; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc(ii, pU, C+jj*ldc, ldc, &d1, C0+ii+jj*ldc0, ldc0, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
		}
	goto end_1;



end_1:
	return;

	// never to get here
	return;

	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dgetrf_rp(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, int *ipiv)
	{
	blasfeo_hp_dgetrf_rp(m, n, sC, ci, cj, sD, di, dj, ipiv);
	}



void blasfeo_dgetrf_np(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dgetrf_np(m, n, sC, ci, cj, sD, di, dj);
	}



#endif

