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
#define blas_dgetrf_np dgetrf_np_
#endif



void blas_dgetrf_np(int *pm, int *pn, double *C, int *pldc, int *info)
	{

#if defined(PRINT_NAME)
	printf("\nblas_dgetrf_np %d %d %p %d %d\n", *pm, *pn, C, *pldc, *info);
#endif

//#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//#else
//	printf("\nblas_dgetrf_np: not implemented yet\n");
//	exit(1);
//#endif

	int m = *pm;
	int n = *pn;
	int ldc = *pldc;

//	d_print_mat(m, n, C, ldc);
//	printf("\nm %d n %d ldc %d\n", m, n, ldc);

	// XXX nothing checked for now
	*info = 0;

	if(m<=0 | n<=0)
		return;

	int ps = 4;

// TODO visual studio alignment
#if defined(TARGET_GENERIC)
	double pd0[K_MAX_STACK];
#else
	ALIGNED( double pd0[K_MAX_STACK], 64 );
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
#endif
	int sdu0 = (m+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;


	int sdu, sdc;
	double *pU, *pC, *pd;
	int sC_size, stot_size;
	void *mem;
	char *mem_align;
	int m1, n1;

//	int n4 = n<4 ? n : 4;

	int p = m<n ? m : n;

	int m_max;


	int i1 = 1;
	double d1 = 1.0;
	double dm1 = -1.0;

	int ii, jj, ie;

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
		printf("\nblas_dgetrf_np: not implemented yet for m>K_MAX_STACK\n");
		exit(1);
//		goto alg1;
		}
	else
		{
		goto alg0;
		}



alg0:

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
			kernel_dtrsm_nn_ru_inv_12x4_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_12_lib4(4, C+ii+jj*ldc, ldc, pU+jj*ps, sdu);
			}
		if(jj<ie)
			{
			kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_12_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			jj+=4;
			}

		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_nn_l_12x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_l_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n-3)
			{
			kernel_dgetrf_nn_m_12x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_m_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n-3)
			{
			kernel_dgetrf_nn_r_12x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_r_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_12x4_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}

		}
	if(m>ii)
		{
		if(m-ii<=4)
			{
			goto left_4_0;
			}
		if(m-ii<=8)
			{
			goto left_8_0;
			}
		else
			{
			goto left_12_0;
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
			kernel_dtrsm_nn_ru_inv_8x4_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_8_lib4(4, C+ii+jj*ldc, ldc, pU+jj*ps, sdu);
			}
		if(jj<ie)
			{
			kernel_dtrsm_nn_ru_inv_8x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_8_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			jj+=4;
			}

		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_nn_l_8x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_l_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n-3)
			{
			kernel_dgetrf_nn_r_8x4_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_r_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_8x4_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}

		}
	if(m>ii)
		{
		if(m-ii<=4)
			{
			goto left_4_0;
			}
		else
			{
			goto left_8_0;
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
			kernel_dtrsm_nn_ru_inv_4x4_lib4cccc(jj, pU, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_4_lib4(4, C+ii+jj*ldc, ldc, pU+jj*ps);
			}
		if(jj<ie)
			{
			kernel_dtrsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_4_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, m-ii);
			jj+=4;
			}

		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_nn_4x4_lib4ccc(jj, pU, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_dgetrf_nn_4x4_vs_lib4ccc(jj, pU, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_4x4_lib4cccc(ii, pU, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc);
			}
		if(jj<n)
			{
			kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc(ii, pU, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}

		}
	if(m>ii)
		{
		goto left_4_0;
		}
#endif
	goto end_0;



#if defined(TARGET_X64_INTEL_HASWELL)
left_12_0:
		jj = 0;

		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_12_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			}

		// factorize
		if(jj<n)
			{
			kernel_dgetrf_nn_l_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n)
			{
			kernel_dgetrf_nn_m_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n)
			{
			kernel_dgetrf_nn_r_12x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}
	goto end_0;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_8_0:
		jj = 0;

		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie; jj+=4)
			{
			kernel_dtrsm_nn_ru_inv_8x4_vs_lib4cccc(jj, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			kernel_dpack_nn_8_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, sdu, m-ii);
			}

		// factorize
		if(jj<n)
			{
			kernel_dgetrf_nn_l_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}
		if(jj<n)
			{
			kernel_dgetrf_nn_r_8x4_vs_lib4ccc(jj, pU, sdu, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
			jj+=4;
			}

		// solve upper
		for( ; jj<n; jj+=4)
			{
			kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc(ii, pU, sdu, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
			}
	goto end_0;
#endif



left_4_0:
	jj = 0;

	// solve lower
	ie = n<ii ? n : ii; // ie is multiple of 4
	for( ; jj<ie; jj+=4)
		{
		kernel_dtrsm_nn_ru_inv_4x4_vs_lib4cccc(jj, pU, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, n-jj);
		kernel_dpack_nn_4_vs_lib4(ie-jj, C+ii+jj*ldc, ldc, pU+jj*ps, m-ii);
		}

	// factorize
	if(jj<n)
		{
		kernel_dgetrf_nn_4x4_vs_lib4ccc(jj, pU, C+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, n-jj);
		jj+=4;
		}

	// solve upper
	for( ; jj<n; jj+=4)
		{
		kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc(ii, pU, C+jj*ldc, ldc, &d1, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+ii+ii*ldc, ldc, m-ii, n-jj);
		}
	goto end_0;



end_0:
	return;

	}
