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

#include <stdlib.h>
#include <stdio.h>

#include "../include/blasfeo_d_kernel.h"

#include "../../../../include/blasfeo_target.h"
#include "../../../../include/blasfeo_common.h"
#include "../../../../include/blasfeo_d_aux.h"
#include "../../../../include/blasfeo_d_kernel.h"



// XXX implementation for x64 intel haswell only
void blasfeo_dgemm(char *ta, char *tb, int *pm, int *pn, int *pk, double *alpha, double *A, int *plda, double *B, int *pldb, double *beta, double *C, int *pldc)
	{

	int m = *pm;
	int n = *pn;
	int k = *pk;
	int lda = *plda;
	int ldb = *pldb;
	int ldc = *pldc;

	int ii, jj;

	int bs = 4;

	ALIGNED( double pU[12*256], 64 );
	int sdu = 256;

	struct blasfeo_dmat sA, sB;
	int sda, sdb;
	int sA_size, sB_size;
	void *smat_mem, *smat_mem_align;

	if(*ta=='n')
		{
		if(*tb=='n')
			{
			if(m>=256 | n>=256 | k>=256)
				{
				goto nn_1;
				}
			else
				{
				goto nn_0;
				}
			}
		else // tb==t
			{
			if(m>=96 | n>=96 | k>=96)
				{
				goto nt_1;
				}
			else
				{
				goto nt_0;
				}
			}
		}
	else // ta==t
		{
		if(*tb=='n')
			{
			if(m>=256 | n>=256 | k>=256)
				{
				goto tn_1;
				}
			else
				{
				goto tn_0;
				}
			}
		else // tb==t
			{
			if(m>=96 | n>=96 | k>=96)
				{
				goto tt_1;
				}
			else
				{
				goto tt_0;
				}
			}
		}

	return;
	


nn_0:
	
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_12x4_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_8x4_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_4x4_lib4ccc(k, alpha, pU, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_4x4_vs_lib4ccc(k, alpha, pU, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_0_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nn_0_return;
#endif

nn_0_left_4:
	kernel_dpack_nn_4_vs_lib4(k, A+ii, lda, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, alpha, pU, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nn_0_return;

nn_0_return:
	return;



nn_1:

	sA_size = blasfeo_memsize_dmat(12, k);
	sB_size = blasfeo_memsize_dmat(n, k);
	smat_mem = malloc(sA_size+sB_size+63);
	smat_mem_align = (void *) ( ( ( (unsigned long long) smat_mem ) + 63) / 64 * 64 );
	blasfeo_create_dmat(12, k, &sA, smat_mem_align);
	blasfeo_create_dmat(n, k, &sB, smat_mem_align+sA_size);

	blasfeo_pack_tran_dmat(k, n, B, k, &sB, 0, 0);

	sda = sA.cn;
	sdb = sB.cn;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nn_1_left_4;
			}
		if(m-ii<=8)
			{
			goto nn_1_left_8;
			}
		else
			{
			goto nn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
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
		kernel_dpack_nn_4_lib4(k, A+ii, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nn_1_left_4;
		}
#endif
	goto nn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nn_1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nn_1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nn_1_return;
#endif

nn_1_left_4:
	kernel_dpack_nn_4_lib4(k, A+ii, lda, sA.pA);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nn_1_return;

nn_1_return:
	free(smat_mem);
	return;



nt_0:

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii+0, lda, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, alpha, pU, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib4ccc(k, alpha, pU, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
	kernel_dpack_nn_12_vs_lib4(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nt_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_0_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii+0, lda, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nt_0_return;
#endif

nt_0_left_4:
	kernel_dpack_nn_4_lib4(k, A+ii, lda, pU);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, alpha, pU, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nt_0_return;

nt_0_return:
	return;



nt_1:

	sA_size = blasfeo_memsize_dmat(12, k);
	sB_size = blasfeo_memsize_dmat(k, n);
	smat_mem = malloc(sA_size+sB_size+63);
	smat_mem_align = (void *) ( ( ( (unsigned long long) smat_mem ) + 63) / 64 * 64 );
	blasfeo_create_dmat(12, k, &sA, smat_mem_align);
	blasfeo_create_dmat(k, n, &sB, smat_mem_align+sA_size);

	blasfeo_pack_dmat(k, n, B, k, &sB, 0, 0);

	sda = sA.cn;
	sdb = sB.cn;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto nt_1_left_4;
			}
		if(m-ii<=8)
			{
			goto nt_1_left_8;
			}
		else
			{
			goto nt_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(k, A+ii, lda, sA.pA, sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
		kernel_dpack_nn_4_lib4(k, A+ii, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto nt_1_left_4;
		}
#endif
	goto nt_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
nt_1_left_12:
	kernel_dpack_nn_12_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
nt_1_left_8:
	kernel_dpack_nn_8_vs_lib4(k, A+ii, lda, sA.pA, sda, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nt_1_return;
#endif

nt_1_left_4:
	kernel_dpack_nn_4_lib4(k, A+ii, lda, sA.pA);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto nt_1_return;

nt_1_return:
	free(smat_mem);
	return;



tn_0:

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_12x4_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_0_left_4;
			}
		if(m-ii<=8)
			{
			goto tn_0_left_8;
			}
		else
			{
			goto tn_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_8x4_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_0_left_4;
			}
		else
			{
			goto tn_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_4x4_lib4ccc(k, alpha, pU, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_4x4_vs_lib4ccc(k, alpha, pU, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tn_0_left_4;
		}
#endif
	goto tn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tn_0_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_0_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tn_0_return;
#endif

tn_0_left_4:
	kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4ccc(k, alpha, pU, B+jj*ldb, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tn_0_return;

tn_0_return:
	return;



tn_1:

	sA_size = blasfeo_memsize_dmat(12, k);
	sB_size = blasfeo_memsize_dmat(n, k);
	smat_mem = malloc(sA_size+sB_size+63);
	smat_mem_align = (void *) ( ( ( (unsigned long long) smat_mem ) + 63) / 64 * 64 );
	blasfeo_create_dmat(12, k, &sA, smat_mem_align);
	blasfeo_create_dmat(n, k, &sB, smat_mem_align+sA_size);

	blasfeo_pack_tran_dmat(k, n, B, k, &sB, 0, 0);

	sda = sA.cn;
	sdb = sB.cn;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, sA.pA+8*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tn_1_left_4;
			}
		if(m-ii<=8)
			{
			goto tn_1_left_8;
			}
		else
			{
			goto tn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
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
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
		if(ii<m)
		{
		goto tn_1_left_4;
		}
#endif
	goto tn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tn_1_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, sA.pA+8*sda, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tn_1_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tn_1_return;
#endif

tn_1_left_4:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tn_1_return;

tn_1_return:
free(smat_mem);
	return;



tt_0:

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_0_left_4;
			}
		if(m-ii<=8)
			{
			goto tt_0_left_8;
			}
		else
			{
			goto tt_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_0_left_4;
			}
		else
			{
			goto tt_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib4ccc(k, alpha, pU, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib4ccc(k, alpha, pU, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_0_left_4;
		}
#endif
	goto tt_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tt_0_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, pU+8*sdu, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tt_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_0_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, pU);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, pU+4*sdu, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4ccc(k, alpha, pU, sdu, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tt_0_return;
#endif

tt_0_left_4:
	kernel_dpack_tn_4_lib4(k, A+ii*lda, lda, pU);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4ccc(k, alpha, pU, B+jj, ldb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tt_0_return;

tt_0_return:
	return;



tt_1:

	sA_size = blasfeo_memsize_dmat(12, k);
	sB_size = blasfeo_memsize_dmat(k, n);
	smat_mem = malloc(sA_size+sB_size+63);
	smat_mem_align = (void *) ( ( ( (unsigned long long) smat_mem ) + 63) / 64 * 64 );
	blasfeo_create_dmat(12, k, &sA, smat_mem_align);
	blasfeo_create_dmat(k, n, &sB, smat_mem_align+sA_size);

	blasfeo_pack_dmat(k, n, B, k, &sB, 0, 0);

	sda = sA.cn;
	sdb = sB.cn;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
		kernel_dpack_tn_4_lib4(k, A+(ii+8)*lda, lda, sA.pA+8*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_12x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto tt_1_left_4;
			}
		if(m-ii<=8)
			{
			goto tt_1_left_8;
			}
		else
			{
			goto tt_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_8x4_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
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
		kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nt_4x4_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto tt_1_left_4;
		}
#endif
	goto tt_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
tt_1_left_12:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_dpack_tn_4_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+8)*lda, lda, sA.pA+8*sda, m-ii-8);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tt_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
tt_1_left_8:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	kernel_dpack_tn_4_vs_lib4(k, A+(ii+4)*lda, lda, sA.pA+4*sda, m-ii-4);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib44cc(k, alpha, sA.pA, sda, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tt_1_return;
#endif

tt_1_left_4:
	kernel_dpack_tn_4_lib4(k, A+(ii+0)*lda, lda, sA.pA);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib44cc(k, alpha, sA.pA, sB.pA+jj*sdb, beta, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, m-ii, n-jj);
		}
	goto tt_1_return;

tt_1_return:
	free(smat_mem);
	return;

	}

