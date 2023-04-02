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

#include "../include/blasfeo_target.h"
#include "../include/blasfeo_common.h"
#include "../include/blasfeo_s_aux.h"
#include "../include/blasfeo_s_kernel.h"



#if defined(FORTRAN_BLAS_API)
#define blas_sgemm sgemm_
#endif



static void blasfeo_sgemm(char ta, char tb, int m, int n, int k, float alpha, float *A, int lda, float *B, int ldb, float beta, float *C, int ldc, float *D, int ldd)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_sgemm %c %c %d %d %d %f %p %d %p %d %f %p %d %p %d\n", ta, tb, m, n, k, alpha, A, lda, B, ldb, beta, C, ldc, D, ldd);
#endif

	if(m<=0 | n<=0)
		return;

	int ii, jj;

// no global bs, to be able to mix them in different algorithms !!!
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	const int bs = 8;
//#else
//	const int bs = 4;
//#endif
	const int ps_4 = 4;
	const int ps_8 = 8;

// TODO visual studio alignment
#if defined(TARGET_X64_INTEL_HASWELL)
	ALIGNED( float pU[3*8*K_MAX_STACK], 64 );
	int sdu = (k+ps_8-1)/ps_8*ps_8;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	ALIGNED( float pU[2*8*K_MAX_STACK], 64 );
	int sdu = (k+ps_8-1)/ps_8*ps_8;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	ALIGNED( float pU[2*4*K_MAX_STACK], 64 );
	int sdu = (k+ps_4-1)/ps_4*ps_4;
#elif defined(TARGET_GENERIC)
	float pU[1*4*K_MAX_STACK];
	int sdu = (k+ps_4-1)/ps_4*ps_4;
#else
	ALIGNED( float pU[1*4*K_MAX_STACK], 64 );
	int sdu = (k+ps_4-1)/ps_4*ps_4;
#endif
	sdu = sdu<K_MAX_STACK ? sdu : K_MAX_STACK;

	struct blasfeo_smat sA, sB;
	int sda, sdb;
	int sA_size, sB_size;
	void *mem;
	char *mem_align;
	int m1, n1, k1;
	int pack_B;

	// TODO move to a header file to reuse across routines
#if defined(TARGET_X64_INTEL_HASWELL)
	const int m_kernel = 24;
	// L1 data cache size: 32 kB
	const int l1_cache_el = 32*1024/4;
	// data cache size: 64 bytes
	const int reals_per_cache_line = 16;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	const int m_kernel = 16;
	// L1 data cache size: 32 kB
	const int l1_cache_el = 32*1024/4;
	// data cache size: 64 bytes
	const int reals_per_cache_line = 16;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	const int m_kernel = 8;
	// L1 data cache size: 32 kB
	const int l1_cache_el = 32*1024/4;
	// data cache size: 64 bytes
	const int reals_per_cache_line = 16;
	// TODO 32-bytes for cortex A9
#else // assume generic target
	const int m_kernel = 4;
	// L1 data cache size: 32 kB
	const int l1_cache_el = 32*1024/4;
	// data cache size: 64 bytes
	const int reals_per_cache_line = 16;
#endif
	const int m_cache = (m+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int n_cache = (n+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int k_cache = (k+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	const int m_kernel_cache = (m_kernel+reals_per_cache_line-1)/reals_per_cache_line*reals_per_cache_line;
	int m_min = m_cache<m_kernel_cache ? m_cache : m_kernel_cache;
//	int n_min = n_cache<m_kernel_cache ? n_cache : m_kernel_cache;

	if(ta=='n' | ta=='N')
		{
		if(tb=='n' | tb=='N')
			{
//			goto nn_2; // no pack
//			goto nn_m0; // pack A
//			goto nn_n0; // pack B
//			goto nn_1; // pack A and B
			if( k<=K_MAX_STACK )
				{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				goto nn_1; // XXX all matrices for now !!!!!
//				if( m<=48 & n<=48 )
//				if( (m<=12 & n<=12) | (m_min*k_cache + n_cache*k_cache <= l1_cache_el) )
				if( (m<=m_kernel & n<=m_kernel) | (m_kernel_cache*k_cache + n_cache*k_cache <= l1_cache_el) | (m<m_kernel & (m_cache*k_cache + m_kernel_cache*k_cache <= l1_cache_el) ) )
					{
					goto nn_2; // small matrix: no pack
					}
#else
				if( m<=8 & n<=8 )
					{
					goto nn_2; // small matrix: no pack
					}
#endif
#if defined(TARGET_X64_INTEL_HASWELL)
				if( m<=2*m_kernel | n<=2*m_kernel | k<448 )
#else
				if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
					{
					if( m<=n*4 )
						{
						goto nn_m0; // long matrix: pack A
						}
					else
						{
						goto nn_n0; // tall matrix: pack B
						}
					}
				}
			goto nn_1; // big matrix: pack A and B
			}
		else if(tb=='t' | tb=='T' | tb=='c' | tb=='C')
			{
//			goto nt_2; // no pack
//			goto nt_m0; // pack A
//			goto nt_n0; // pack B
//			goto nt_1; // pack A and B
			if( k<=K_MAX_STACK )
				{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				goto nt_1; // XXX all matrices for now !!!!!
//				if( m<=48 & n<=48 )
				if( (m<=m_kernel & n<=m_kernel) | (m_kernel_cache*k_cache + n_cache*k_cache <= l1_cache_el) | (m<m_kernel & (m_cache*k_cache + m_kernel_cache*k_cache <= l1_cache_el) ) )
					{
//					printf("%d %d %d\n", m, n, k);
					goto nt_2; // small matrix: no pack
					}
#else
				if( m<=8 & n<=8 )
					{
					goto nt_2; // small matrix: no pack
					}
#endif
#if defined(TARGET_X64_INTEL_HASWELL)
				if( m<=2*m_kernel | n<=2*m_kernel | k<200 )
#else
				if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
					{
					if( m<=n )
						{
						goto nt_m0; // long matrix: pack A
						}
					else
						{
						goto nt_n0; // tall matrix: pack B
						}
					}
				}
			goto nt_1; // big matrix: pack A and B
			}
		else
			{
			printf("\nBLASFEO: sgemm: wrong value fortb\n");
			return;
			}
		}
	else if(ta=='t' | ta=='T' | ta=='c' | ta=='C')
		{
		if(tb=='n' | tb=='N')
			{
//			goto tn_m0; // pack A
//			goto tn_n0; // pack B
//			goto tn_1; // pack A and B
			if( k<=K_MAX_STACK )
				{
				// no algorithm for small matrix
//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//				if( m<=48 & n<=48 )
//#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
//				if( m<=24 & n<=24 )
//#else
//				if( m<=8 & n<=8 )
//#endif
//					{
//					goto tn_m0; // small matrix: pack A
//					}
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				goto tn_1; // XXX all matrices for now !!!!!
				if( m<=2*m_kernel | n<=2*m_kernel | k<448 )
#else
				if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
					{
					if( m<=n )
						{
						goto tn_m0; // long matrix: pack A
						}
					else
						{
						goto tn_n0; // tall matrix: pack B
						}
					}
				}
			goto tn_1; // big matrix: pack A and B
			}
		else if(tb=='t' | tb=='T' | tb=='c' | tb=='C')
			{
//			goto tt_2; // no pack
//			goto tt_m0; // pack A
//			goto tt_n0; // pack B
//			goto tt_1; // pack A and B
			if( k<=K_MAX_STACK )
				{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
				goto tt_1; // XXX all matrices for now !!!!!
//				if( m<=48 & n<=48 )
				if( (m<=m_kernel & n<=m_kernel) | (m_cache*k_cache + m_kernel_cache*k_cache <= l1_cache_el) | (n<m_kernel & (m_kernel_cache*k_cache + n_cache*k_cache <= l1_cache_el) ) )
					{
					goto tt_2; // small matrix: no pack
					}
#else
				if( m<=8 & n<=8 )
					{
					goto tt_2; // small matrix: no pack
					}
#endif
#if defined(TARGET_X64_INTEL_HASWELL)
				if( m<=2*m_kernel | n<=2*m_kernel | k<448 )
#else
				if( m<=1*m_kernel | n<=1*m_kernel | k<12 )
#endif
					{
					if( m*4<=n | k<=4 ) // XXX k too !!!
						{
						goto tt_m0; // long matrix: pack A
						}
					else
						{
						goto tt_n0; // tall matrix: pack B
						}
					}
				}
			goto tt_1; // big matrix: pack A and B
			}
		}
	else
		{
		printf("\nBLASFEO: sgemm: wrong value for ta\n");
		return;
		}

	return;
	


nn_m0:
	
	ii = 0;
#if 0 //defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_spack_nn_12_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_m0_left_4;
			}
		if(m-ii<=8)
			{
			goto nn_m0_left_8;
			}
		else
			{
			goto nn_m0_left_12;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
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
			goto nn_m0_left_4;
			}
		else
			{
			goto nn_m0_left_8;
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
		goto nn_m0_left_4;
		}
#endif
	goto nn_m0_return;

#if 0 //defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_m0_left_12:
	kernel_spack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m0_return;
#endif

#if 0 //defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_m0_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_m0_return;
#endif

nn_m0_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
#if 0 //defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_sgemm_nn_4x12_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_sgemm_nn_4x8_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nn_m0_return;

nn_m0_return:
	return;



nn_n0:

	jj = 0;
#if 0 //defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		kernel_spack_tn_4_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x12_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nn_n0_left_4;
			}
		else if(n-jj<=8)
			{
			goto nn_n0_left_8;
			}
		else
			{
			goto nn_n0_left_12;
			}
		}
#elif 0 //defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
			kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nn_n0_left_4;
			}
		else
			{
			goto nn_n0_left_8;
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
		goto nn_n0_left_4;
		}
#endif
	goto nn_n0_return;

#if 0 //defined(TARGET_X64_INTEL_HASWELL)
nn_n0_left_12:
	kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
	kernel_spack_tn_4_vs_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_n0_return;
#endif


#if 0 //defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_n0_left_8:
	kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_spack_tn_4_vs_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu, n-jj-4);
	for(ii=0; ii<m; ii+=4)
		{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
		}
	goto nn_n0_return;
#endif

nn_n0_left_4:
	kernel_spack_tn_4_vs_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
#if 0 //defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_sgemm_nt_12x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_sgemm_nt_8x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nn_n0_return;

nn_n0_return:
	return;



nn_1:

	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_smat(m_kernel, k1);
	sB_size = blasfeo_memsize_smat(n1, k1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_smat(m_kernel, k, &sA, (void *) mem_align);
	blasfeo_create_smat(n, k, &sB, (void *) (mem_align+sA_size));

	sda = sA.cn;
	sdb = sB.cn;

	pack_B = 1;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_nn_24_lib8(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nn_1_left_8;
			}
		else if(m-ii<=16)
			{
			goto nn_1_left_16;
			}
		else
			{
			goto nn_1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_nn_16_lib8(k, A+ii, lda, sA.pA, sda);
//		kernel_spack_nn_8_lib8(k, A+ii, lda, sA.pA); // TODO 16
//		kernel_spack_nn_8_lib8(k, A+ii+8, lda, sA.pA+sda*ps_8); // TODO 16
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
//			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
//				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x8_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nn_1_left_8;
			}
		else
			{
			goto nn_1_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib8(k, A+ii, lda, sA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x8_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		goto nn_1_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
//			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_1_left_4;
			}
		else
			{
			goto nn_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_nn_4_lib4(k, A+ii, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		goto nn_1_left_4;
		}
#endif
	goto nn_1_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nn_1_left_12:
	kernel_spack_nn_12_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_12x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
nn_1_left_24:
	kernel_spack_nn_24_vs_lib8(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_1_left_16:
	kernel_spack_nn_16_vs_lib8(k, A+ii, lda, sA.pA, sda, m-ii);
//	kernel_spack_nn_8_lib8(k, A+ii, lda, sA.pA); // TODO 16
//	kernel_spack_nn_8_vs_lib8(k, A+ii+8, lda, sA.pA+sda*ps_8, m-ii-8); // TODO 16
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
//		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_1_left_8:
	kernel_spack_nn_8_vs_lib8(k, A+ii, lda, sA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nn_1_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
nn_1_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
		}
	goto nn_1_return;
#endif

nn_1_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, sA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_1_return;

nn_1_return:
	free(mem);
	return;



nn_2:
	ii = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_12x4_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
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
#else //elif ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
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
		goto nn_2_left_4;
		}
#endif
	goto nn_2_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
nn_2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_2_return;
#endif

#if 0//defined(TARGET_X64_INTEL_HASWELL)
nn_2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_8x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nn_2_return;
#endif

//#if ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_2_left_4:
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_sgemm_nn_4x12_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_sgemm_nn_4x8_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_sgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nn_2_return;
//#endif

nn_2_return:
	return;



nt_m0:

	ii = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_spack_nn_12_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_m0_left_4;
			}
		if(m-ii<=8)
			{
			goto nt_m0_left_8;
			}
		else
			{
			goto nt_m0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
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
			goto nt_m0_left_4;
			}
		else
			{
			goto nt_m0_left_8;
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
		goto nt_m0_left_4;
		}
#endif
	goto nt_m0_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
nt_m0_left_12:
	kernel_spack_nn_12_vs_lib4(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m0_return;
#endif

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_m0_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_m0_return;
#endif

nt_m0_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_sgemm_nt_4x12_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_sgemm_nt_4x8_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nt_m0_return;

nt_m0_return:
	return;



nt_n0:

	jj = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_spack_nn_12_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x12_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nt_n0_left_4;
			}
		else if(n-jj<=8)
			{
			goto nt_n0_left_8;
			}
		else
			{
			goto nt_n0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_nn_8_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_nt_4x8_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
			kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto nt_n0_left_4;
			}
		else
			{
			goto nt_n0_left_8;
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
		goto nt_n0_left_4;
		}
#endif
	goto nt_n0_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
nt_n0_left_12:
	kernel_spack_nn_12_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x12_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_n0_return;
#endif


#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_n0_left_8:
	kernel_spack_nn_8_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_sgemm_nt_4x8_vs_libc4cc(k, &alpha, A+ii, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
		}
	goto nt_n0_return;
#endif

nt_n0_left_4:
	kernel_spack_nn_4_vs_lib4(k, B+jj, ldb, pU, n-jj);
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_sgemm_nt_12x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_sgemm_nt_8x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_nt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nt_n0_return;

nt_n0_return:
	return;



nt_1:

	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_smat(m_kernel, k1);
	sB_size = blasfeo_memsize_smat(n1, k1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_smat(m_kernel, k, &sA, (void *) mem_align);
	blasfeo_create_smat(n, k, &sB, (void *) (mem_align+sA_size));

	sda = sA.cn;
	sdb = sB.cn;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(ii=0; ii<k-7; ii+=8)
		{
		kernel_spack_tt_8_lib8(n, B+ii*ldb, ldb, sB.pA+ii*ps_8, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_8_vs_lib8(n, B+ii*ldb, ldb, sB.pA+ii*ps_8, sdb, k-ii);
		}
#else
	for(ii=0; ii<k-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(n, B+ii*ldb, ldb, sB.pA+ii*ps_4, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_4_vs_lib4(n, B+ii*ldb, ldb, sB.pA+ii*ps_4, sdb, k-ii);
		}
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_nn_24_lib8(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_1_left_8;
			}
		else if(m-ii<=16)
			{
			goto nt_1_left_16;
			}
		else
			{
			goto nt_1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_nn_16_lib8(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto nt_1_left_8;
			}
		else
			{
			goto nt_1_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib8(k, A+ii, lda, sA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		goto nt_1_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_nn_8_lib4(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
//			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_1_left_4;
			}
		else
			{
			goto nt_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_nn_4_lib4(k, A+ii, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_1_left_4;
		}
#endif
	goto nt_1_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
nt_1_left_12:
	kernel_spack_nn_12_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_12x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
nt_1_left_24:
	kernel_spack_nn_24_vs_lib8(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_1_left_16:
	kernel_spack_nn_16_vs_lib8(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_1_left_8:
	kernel_spack_nn_8_vs_lib8(k, A+ii, lda, sA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto nt_1_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
nt_1_left_8:
	kernel_spack_nn_8_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
//		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
		}
	goto nt_1_return;
#endif

nt_1_left_4:
	kernel_spack_nn_4_vs_lib4(k, A+ii, lda, sA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_1_return;

nt_1_return:
	free(mem);
	return;



nt_2:
	ii = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_12x4_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
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
#else //elif ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
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
		goto nt_2_left_4;
		}
#endif
	goto nt_2_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
nt_2_left_12:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_12x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_2_return;
#endif

#if 0//defined(TARGET_X64_INTEL_HASWELL)
nt_2_left_8:
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_8x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto nt_2_return;
#endif

//#if ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_2_left_4:
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_sgemm_nt_4x12_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_sgemm_nt_4x8_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_sgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_libcccc(k, &alpha, A+ii, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto nt_2_return;
//#endif

nt_2_return:
	return;



tn_m0:

	ii = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_spack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nn_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_m0_left_4;
			}
		if(m-ii<=8)
			{
			goto tn_m0_left_8;
			}
		else
			{
			goto tn_m0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
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
			goto tn_m0_left_4;
			}
		else
			{
			goto tn_m0_left_8;
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
		goto tn_m0_left_4;
		}
#endif
	goto tn_m0_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_m0_left_12:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m0_return;
#endif

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_m0_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_m0_return;
#endif

tn_m0_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_sgemm_nn_4x12_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_sgemm_nn_4x8_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, B+jj*ldb, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tn_m0_return;

tn_m0_return:
	return;



tn_n0:

	jj = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		kernel_spack_tn_4_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x12_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tn_n0_left_4;
			}
		else if(n-jj<=8)
			{
			goto tn_n0_left_8;
			}
		else
			{
			goto tn_n0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
		kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
			kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tn_n0_left_4;
			}
		else
			{
			goto tn_n0_left_8;
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
		goto tn_n0_left_4;
		}
#endif
	goto tn_n0_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
tn_n0_left_12:
	kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_spack_tn_4_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu);
	kernel_spack_tn_4_vs_lib4(k, B+(jj+8)*ldb, ldb, pU+8*sdu, n-jj-8);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_n0_return;
#endif


#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_n0_left_8:
	kernel_spack_tn_4_lib4(k, B+(jj+0)*ldb, ldb, pU);
	kernel_spack_tn_4_vs_lib4(k, B+(jj+4)*ldb, ldb, pU+4*sdu, n-jj-4);
	for(ii=0; ii<m; ii+=4)
		{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
		}
	goto tn_n0_return;
#endif

tn_n0_left_4:
	kernel_spack_tn_4_vs_lib4(k, B+(jj+0)*ldb, ldb, pU+0*sdu, n-jj-0);
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_sgemm_tt_12x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_sgemm_tt_8x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tn_n0_return;

tn_n0_return:
	return;



tn_1:

	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_smat(m_kernel, k1);
	sB_size = blasfeo_memsize_smat(n1, k1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_smat(m_kernel, k, &sA, (void *) mem_align);
	blasfeo_create_smat(n, k, &sB, (void *) (mem_align+sA_size));

	sda = sA.cn;
	sdb = sB.cn;

	pack_B = 1;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8);
		kernel_spack_tn_8_lib8(k, A+(ii+16)*lda, lda, sA.pA+2*sda*ps_8);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tn_1_left_8;
			}
		else if(m-ii<=16)
			{
			goto tn_1_left_16;
			}
		else
			{
			goto tn_1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tn_1_left_8;
			}
		else
			{
			goto tn_1_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			if(pack_B)
				kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		goto tn_1_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
//			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
			}
		pack_B = 0;
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_1_left_4;
			}
		else
			{
			goto tn_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			if(pack_B)
				kernel_spack_tn_4_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			if(pack_B)
				kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		pack_B = 0;
		}
		if(ii<m)
		{
		goto tn_1_left_4;
		}
#endif
	goto tn_1_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tn_1_left_12:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, sA.pA+8*sda, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_12x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
tn_1_left_24:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
	kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, sA.pA+2*sda*ps_8, m-ii-16);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_1_left_16:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8, m-ii-8);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_1_left_8:
	kernel_spack_tn_8_vs_lib8(k, A+ii*lda, lda, sA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		if(pack_B)
			kernel_spack_tn_8_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb);
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(pack_B)
			kernel_spack_tn_8_vs_lib8(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tn_1_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
tn_1_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
		}
	goto tn_1_return;
#endif

tn_1_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+(ii+0)*lda, lda, sA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		if(pack_B)
			kernel_spack_tn_4_vs_lib4(k, B+jj*ldb, ldb, sB.pA+jj*sdb, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tn_1_return;

tn_1_return:
free(mem);
	return;



tt_m0:

	ii = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_spack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_12x4_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_m0_left_4;
			}
		if(m-ii<=8)
			{
			goto tt_m0_left_8;
			}
		else
			{
			goto tt_m0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
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
			goto tt_m0_left_4;
			}
		else
			{
			goto tt_m0_left_8;
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
		goto tt_m0_left_4;
		}
#endif
	goto tt_m0_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
tt_m0_left_12:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m0_return;
#endif

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_m0_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_8x4_vs_lib4ccc(k, &alpha, pU, sdu, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_m0_return;
#endif

tt_m0_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_sgemm_nt_4x12_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_sgemm_nt_4x8_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tt_m0_return;

tt_m0_return:
	return;



tt_n0:

	jj = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_spack_nn_12_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x12_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tt_n0_left_4;
			}
		else if(n-jj<=8)
			{
			goto tt_n0_left_8;
			}
		else
			{
			goto tt_n0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; jj<n-7; jj+=8)
		{
		kernel_spack_nn_8_lib4(k, B+jj, ldb, pU, sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x8_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
			kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
			kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tt_n0_left_4;
			}
		else
			{
			goto tt_n0_left_8;
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
		goto tt_n0_left_4;
		}
#endif
	goto tt_n0_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
tt_n0_left_12:
	kernel_spack_nn_12_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x12_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_n0_return;
#endif


#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_n0_left_8:
	kernel_spack_nn_8_vs_lib4(k, B+jj, ldb, pU, sdu, n-jj);
	for(ii=0; ii<m; ii+=4)
		{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_sgemm_tt_4x8_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
#else
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii, lda, pU, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
#endif
		}
	goto tt_n0_return;
#endif

tt_n0_left_4:
	kernel_spack_nn_4_vs_lib4(k, B+jj, ldb, pU, n-jj);
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_sgemm_tt_12x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_sgemm_tt_8x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x4_vs_libc4cc(k, &alpha, A+ii*lda, lda, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tt_n0_return;

tt_n0_return:
	return;



tt_1:

	k1 = (k+128-1)/128*128;
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_smat(m_kernel, k1);
	sB_size = blasfeo_memsize_smat(n1, k1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_smat(m_kernel, k, &sA, (void *) mem_align);
	blasfeo_create_smat(n, k, &sB, (void *) (mem_align+sA_size));

	sda = sA.cn;
	sdb = sB.cn;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(ii=0; ii<k-7; ii+=8)
		{
		kernel_spack_tt_8_lib8(n, B+ii*ldb, ldb, sB.pA+ii*ps_8, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_8_vs_lib8(n, B+ii*ldb, ldb, sB.pA+ii*ps_8, sdb, k-ii);
		}
#else
	for(ii=0; ii<k-3; ii+=4)
		{
		kernel_spack_tt_4_lib4(n, B+ii*ldb, ldb, sB.pA+ii*4, sdb);
		}
	if(ii<k)
		{
		kernel_spack_tt_4_vs_lib4(n, B+ii*ldb, ldb, sB.pA+ii*4, sdb, k-ii);
		}
#endif

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-23; ii+=24)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8);
		kernel_spack_tn_8_lib8(k, A+(ii+16)*lda, lda, sA.pA+2*sda*ps_8);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_24x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tt_1_left_8;
			}
		else if(m-ii<=16)
			{
			goto tt_1_left_16;
			}
		else
			{
			goto tt_1_left_24;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-15; ii+=16)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
		kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
			kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_16x4_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		if(m-ii<=8)
			{
			goto tt_1_left_8;
			}
		else
			{
			goto tt_1_left_16;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
		for(jj=0; jj<n-7; jj+=8)
			{
			kernel_sgemm_nt_8x8_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//			kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd);
			}
		if(jj<n)
			{
			if(n-jj>4)
				{
				kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//				kernel_sgemm_nt_8x4_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd);
//				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
				}
			else
				{
				kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-jj);
				}
			}
		}
	if(ii<m)
		{
		goto tt_1_left_8;
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	for(; ii<m-7; ii+=8)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
//			kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_1_left_4;
			}
		else
			{
			goto tt_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<n)
			{
			kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_1_left_4;
		}
#endif
	goto tt_1_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
tt_1_left_12:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_spack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, sA.pA+8*sda, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_12x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
tt_1_left_24:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
	kernel_spack_tn_8_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+16)*lda, lda, sA.pA+2*sda*ps_8, m-ii-16);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_24x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_1_left_16:
	kernel_spack_tn_8_lib8(k, A+ii*lda, lda, sA.pA);
	kernel_spack_tn_8_vs_lib8(k, A+(ii+8)*lda, lda, sA.pA+sda*ps_8, m-ii-8);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_16x4_vs_lib88cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_1_left_8:
	kernel_spack_tn_8_vs_lib8(k, A+ii*lda, lda, sA.pA, m-ii);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
//		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+4, &beta, C+ii+(jj+4)*ldc, ldc, D+ii+(jj+4)*ldd, ldd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		kernel_sgemm_nt_8x4_vs_lib88cc(k, &alpha, sA.pA, sB.pA+jj*sdb+0, &beta, C+ii+(jj+0)*ldc, ldc, D+ii+(jj+0)*ldd, ldd, m-ii, n-(jj+0));
		}
	goto tt_1_return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
tt_1_left_8:
	kernel_spack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_spack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
//		kernel_sgemm_nt_8x4_vs_lib44cc(k, &alpha, sA.pA, sda, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA+4*sda, sB.pA+jj*sdb, &beta, C+(ii+4)+jj*ldc, ldc, D+(ii+4)+jj*ldd, ldd, m-(ii+4), n-jj);
		}
	goto tt_1_return;
#endif

tt_1_left_4:
	kernel_spack_tn_4_vs_lib4(k, A+(ii+0)*lda, lda, sA.pA, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib44cc(k, &alpha, sA.pA, sB.pA+jj*sdb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_1_return;

tt_1_return:
	free(mem);
	return;



tt_2:

	jj = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_sgemm_tt_4x12_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(ii<m)
			{
			kernel_sgemm_tt_4x12_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto tt_2_left_4;
			}
		else if(n-jj<=8)
			{
			goto tt_2_left_8;
			}
		else
			{
			goto tt_2_left_12;
			}
		}
#else //elif ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
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
		goto tt_2_left_4;
		}
#endif
	goto tt_2_return;

#if 0//defined(TARGET_X64_INTEL_HASWELL)
tt_2_left_12:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x12_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_2_return;
#endif


#if 0//defined(TARGET_X64_INTEL_HASWELL)
tt_2_left_8:
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x8_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	goto tt_2_return;
#endif

//#if ! defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_2_left_4:
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_sgemm_tt_12x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_sgemm_tt_8x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_sgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#else
	for(ii=0; ii<m; ii+=4)
		{
		kernel_sgemm_tt_4x4_vs_libcccc(k, &alpha, A+ii*lda, lda, B+jj, ldb, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, n-jj);
		}
#endif
	goto tt_2_return;
//#endif

tt_2_return:
	return;



	}




void blas_sgemm(char *ta, char *tb, int *m, int *n, int *k, float *alpha, float *A, int *lda, float *B, int *ldb, float *beta, float *C, int *ldc)
	{
	blasfeo_sgemm(*ta, *tb, *m, *n, *k, *alpha, A, *lda, B, *ldb, *beta, C, *ldc, C, *ldc);
	}




