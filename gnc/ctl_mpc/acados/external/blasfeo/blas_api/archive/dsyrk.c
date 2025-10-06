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
#include "../include/blasfeo_d_aux.h"
#include "../include/blasfeo_d_kernel.h"
#include "../include/blasfeo_d_blas.h"



#if defined(FORTRAN_BLAS_API)
#define blas_dsyrk dsyrk_
#endif



static void blasfeo_dsyrk3(char uplo, char ta, int m, int k, double alpha, double *A, int lda, double beta, double *C, int ldc, double *D, int ldd)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_dsyrk3 %c %c %d %d %f %p %d %f %p %d %p %d\n", uplo, ta, m, k, alpha, A, lda, beta, C, ldc, D, ldd);
#endif

	if(m<=0)
		return;

	int ii, jj;

	int bs = 4;


	void *mem;
	char *mem_align;
	double *pU;
	int sdu;
	struct blasfeo_dmat sA;
	int sA_size;
	int m1, k1;


	// stack memory allocation
// TODO visual studio alignment
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
//	ALIGNED( double pD0[4*16], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
//	ALIGNED( double pD0[2*16], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
//	double pD0[1*16];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
//	ALIGNED( double pD0[1*16], 64 );
#endif
	int sdu0 = (k+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;


	// select algorithm
	if(uplo=='l' | uplo=='L')
		{
		if(ta=='n' | ta=='N')
			{
			goto ln;
			}
		else if(ta=='t' | ta=='T' | ta=='c' | ta=='C')
			{
			goto lt;
			}
		else
			{
			printf("\nBLASFEO: dsyrk: wrong value for ta\n");
			return;
			}
		}
	else if(uplo=='u' | uplo=='U')
		{
		if(ta=='n' | ta=='N')
			{
			goto un;
			}
		else if(ta=='t' | ta=='T' | ta=='c' | ta=='C')
			{
			goto ut;
			}
		else
			{
			printf("\nBLASFEO: dsyrk: wrong value for ta\n");
			return;
			}
		}
	else
		{
		printf("\nBLASFEO: dsyrk: wrong value for uplo\n");
		return;
		}



/************************************************
* ln
************************************************/
ln:
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
		goto lx_1;
		}
	else
		{
		goto ln_0;
		}

ln_0:
	pU = pU0;
	sdu = sdu0;
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
			goto ln_0_left_4;
			}
		if(m-ii<=8)
			{
			goto ln_0_left_8;
			}
		else
			{
			goto ln_0_left_12;
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
			goto ln_0_left_4;
			}
		else
			{
			goto ln_0_left_8;
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
		goto ln_0_left_4;
		}
#endif
	goto ln_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
ln_0_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, &alpha, pU, sdu, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pU, sdu, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
//	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+4*sdu, sdu, pU+4*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
//	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+8*sdu, pU+8*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
	goto ln_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ln_0_left_8:
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
	goto ln_0_return;
#endif

ln_0_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, &alpha, pU, A+jj, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto ln_0_return;

ln_0_return:
	return;



/************************************************
* lt
************************************************/
lt:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>300 | k>300 | k>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | k>=64 | k>K_MAX_STACK)
#elif  defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	if(m>=32 | k>=32 | k>K_MAX_STACK)
#elif  defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	if(m>16 | k>16 | k>K_MAX_STACK)
#else
	if(m>=12 | k>=12 | k>K_MAX_STACK)
#endif
		{
		goto lx_1;
		}
	else
		{
		goto lt_0;
		}

lt_0:
	pU = pU0;
	sdu = sdu0;
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
			goto lt_0_left_4;
			}
		if(m-ii<=8)
			{
			goto lt_0_left_8;
			}
		else
			{
			goto lt_0_left_12;
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
			goto lt_0_left_4;
			}
		else
			{
			goto lt_0_left_8;
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
		goto lt_0_left_4;
		}
#endif
	goto lt_0_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lt_0_left_12:
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
	goto lt_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lt_0_left_8:
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
	goto lt_0_return;
#endif

lt_0_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, &alpha, pU, A+jj*lda, lda, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto lt_0_return;

lt_0_return:
	return;


/************************************************
* un
************************************************/
un:
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
		goto ux_1;
		}
	else
		{
		goto un_0;
		}

un_0:
	pU = pU0;
	sdu = sdu0;
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
			goto un_0_left_4;
			}
		if(m-ii<=8)
			{
			goto un_0_left_8;
			}
		else
			{
			goto un_0_left_12;
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
			goto un_0_left_4;
			}
		else
			{
			goto un_0_left_8;
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
		goto un_0_left_4;
		}
#endif
	goto un_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
un_0_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
//	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
//	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
	kernel_dsyrk_nt_u_12x4_vs_lib44cc(k, &alpha, pU, sdu, pU+8*sdu, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd, m-ii, m-(ii+8));
	goto un_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
un_0_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	goto un_0_return;
#endif

un_0_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	goto un_0_return;

un_0_return:
	return;



/************************************************
* ut
************************************************/
ut:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>300 | k>300 | k>K_MAX_STACK)
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
		goto ux_1;
		}
	else
		{
		goto ut_0;
		}

ut_0:
	pU = pU0;
	sdu = sdu0;
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
			goto ut_0_left_4;
			}
		if(m-ii<=8)
			{
			goto ut_0_left_8;
			}
		else
			{
			goto ut_0_left_12;
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
			goto ut_0_left_4;
			}
		else
			{
			goto ut_0_left_8;
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
		goto ut_0_left_4;
		}
#endif
	goto ut_0_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ut_0_left_12:
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
	goto ut_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ut_0_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU+0*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-(ii+4));
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU, sdu, pU, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU, sdu, pU+4*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	goto ut_0_return;
#endif

ut_0_left_4:
	kernel_dpack_tn_4_vs_lib4(k, A+ii*lda, lda, pU, m-ii);
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU, pU, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	goto ut_0_return;

ut_0_return:
	return;



lx_1:
	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(m1, k1);
	mem = malloc(sA_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(m, k, &sA, (void *) mem_align);

	if(ta=='n' | ta=='N')
		blasfeo_pack_dmat(m, k, A, lda, &sA, 0, 0);
	else
		blasfeo_pack_tran_dmat(k, m, A, lda, &sA, 0, 0);
	pU = sA.pA;
	sdu = sA.cn;
//	blasfeo_print_dmat(m, k, &sA, 0, 0);

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_12x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_l_8x8_lib44cc(k, &alpha, pU+(ii+4)*sdu, sdu, pU+(jj+4)*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
#else
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU+(ii+4)*sdu, sdu, pU+(jj+4)*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+(ii+8)*sdu, pU+(jj+8)*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd);
#endif
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lx_1_left_4;
			}
		if(m-ii<=8)
			{
			goto lx_1_left_8;
			}
		else
			{
			goto lx_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_8x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+(ii+4)*sdu, pU+(jj+4)*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto lx_1_left_4;
			}
		else
			{
			goto lx_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pU+ii*sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		kernel_dsyrk_nt_l_4x4_lib44cc(k, &alpha, pU+ii*sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
		}
	if(ii<m)
		{
		goto lx_1_left_4;
		}
#endif
	goto lx_1_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lx_1_left_12:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU+(ii+4)*sdu, sdu, pU+(jj+4)*sdu, sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+(ii+4)*sdu, sdu, pU+(jj+4)*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+(ii+8)*sdu, pU+(jj+8)*sdu, &beta, C+(ii+8)+(jj+8)*ldc, ldc, D+(ii+8)+(jj+8)*ldd, ldd,  m-(ii+8), m-(jj+8));
#endif
	goto lx_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
lx_1_left_8:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
#else
	kernel_dsyrk_nt_l_8x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd,  m-ii, m-jj);
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+(ii+4)*sdu, pU+(jj+4)*sdu, &beta, C+(ii+4)+(jj+4)*ldc, ldc, D+(ii+4)+(jj+4)*ldd, ldd,  m-(ii+4), m-(jj+4));
#endif
	goto lx_1_return;
#endif

lx_1_left_4:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pU+ii*sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib44cc(k, &alpha, pU+ii*sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
	goto lx_1_return;

lx_1_return:
	free(mem);
	return;


ux_1:
	k1 = (k+128-1)/128*128;
	m1 = (m+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(m1, k1);
	mem = malloc(sA_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(m, k, &sA, (void *) mem_align);

	if(ta=='n' | ta=='N')
		blasfeo_pack_dmat(m, k, A, lda, &sA, 0, 0);
	else
		blasfeo_pack_tran_dmat(k, m, A, lda, &sA, 0, 0);
	pU = sA.pA;
	sdu = sA.cn;
//	blasfeo_print_dmat(m, k, &sA, 0, 0);

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_u_8x8_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+ii*sdu, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
#else
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU+ii*sdu, pU+ii*sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+(ii+4)*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
#endif
		kernel_dsyrk_nt_u_12x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+(ii+8)*sdu, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd);
		for(jj=ii+12; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ux_1_left_4;
			}
		if(m-ii<=8)
			{
			goto ux_1_left_8;
			}
		else
			{
			goto ux_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU+ii*sdu, pU+ii*sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		kernel_dsyrk_nt_u_8x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+(ii+4)*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd);
		for(jj=ii+8; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto ux_1_left_4;
			}
		else
			{
			goto ux_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dsyrk_nt_u_4x4_lib44cc(k, &alpha, pU+ii*sdu, pU+ii*sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd);
		for(jj=ii+4; jj<m-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, &alpha, pU+ii*sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd);
			}
		if(jj<m)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, &alpha, pU+ii*sdu, pU+jj*sdu, &beta, C+ii+jj*ldc, ldc, D+ii+jj*ldd, ldd, m-ii, m-jj);
			}
		}
	if(ii<m)
		{
		goto ux_1_left_4;
		}
#endif
	goto ux_1_return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ux_1_left_12:
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+ii*sdu, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU+ii*sdu, pU+ii*sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+(ii+4)*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	kernel_dsyrk_nt_u_12x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+(ii+8)*sdu, &beta, C+ii+(ii+8)*ldc, ldc, D+ii+(ii+8)*ldd, ldd, m-ii, m-(ii+8));
	goto ux_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
ux_1_left_8:
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_u_8x8_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+ii*sdu, sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
#else
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU+ii*sdu, pU+ii*sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	kernel_dsyrk_nt_u_8x4_vs_lib44cc(k, &alpha, pU+ii*sdu, sdu, pU+(ii+4)*sdu, &beta, C+ii+(ii+4)*ldc, ldc, D+ii+(ii+4)*ldd, ldd, m-ii, m-(ii+4));
#endif
	goto ux_1_return;
#endif

ux_1_left_4:
	kernel_dsyrk_nt_u_4x4_vs_lib44cc(k, &alpha, pU+ii*sdu, pU+ii*sdu, &beta, C+ii+ii*ldc, ldc, D+ii+ii*ldd, ldd, m-ii, m-ii);
	goto ux_1_return;

ux_1_return:
	free(mem);
	return;

	}




void blas_dsyrk(char *uplo, char *ta, int *m, int *k, double *alpha, double *A, int *lda, double *beta, double *C, int *ldc)
	{
	blasfeo_dsyrk3(*uplo, *ta, *m, *k, *alpha, A, *lda, *beta, C, *ldc, C, *ldc);
	}
