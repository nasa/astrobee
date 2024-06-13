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

#include <blasfeo_common.h>
#include <blasfeo_s_kernel.h>
#include <blasfeo_s_aux.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_s_blasfeo_ref_api.h>
#endif



// sgemm nt
void blasfeo_hp_sgemm_nt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	int air = ai & (ps-1);
	int bir = bi & (ps-1);
	float *pA = sA->pA + aj*ps + (ai-air)*sda;
	float *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;

	int ci0 = ci-air;
	int di0 = di-air;
	int offsetC;
	int offsetD;
	if(ci0>=0)
		{
		pC += ci0/ps*ps*sdc;
		offsetC = ci0%ps;
		}
	else
		{
		pC += -4*sdc;
		offsetC = ps+ci0;
		}
	if(di0>=0)
		{
		pD += di0/ps*ps*sdd;
		offsetD = di0%ps;
		}
	else
		{
		pD += -4*sdd;
		offsetD = ps+di0;
		}

	int i, j;

	int idxB;




	// algorithm scheme
	if(air!=0)
		{
		goto clear_air;
		// TODO instaed use buffer to align A !!!
		}
select_loop:
	if(offsetC==0 & offsetD==0)
		{
		goto loop_00;
		}
	else
		{
		goto loop_CD;
		}
	// should never get here
	return;



	// clean up at the beginning
clear_air:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[0], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps]-bir*ps, sdc, offsetD, &pD[j*ps]-bir*ps, sdd, air, air+m, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[0], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
		}
	m -= ps-air;
	pA += ps*sda;
	pC += ps*sdc;
	pD += ps*sdd;
	goto select_loop;



	// main loop aligned
loop_00:
	i = 0;
#if 0//defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-15; i+=16)
		{
		j = 0;
		// TODO bir != 0
		for(; j<n-3; j+=4)
			{
			kernel_sgemm_nt_16x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+0)*sda], &pB[j*sdb], &beta, &pC[j*ps+(i+0)*sdc], &pD[j*ps+(i+0)*sdd], m-(i+0), n-j);
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[j*sdb], &beta, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], m-(i+4), n-j);
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+8)*sda], &pB[j*sdb], &beta, &pC[j*ps+(i+8)*sdc], &pD[j*ps+(i+8)*sdd], m-(i+8), n-j);
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+12)*sda], &pB[j*sdb], &beta, &pC[j*ps+(i+12)*sdc], &pD[j*ps+(i+12)*sdd], m-(i+12), n-j);
			}
		}
#endif
#if defined(TARGET_ARMV7A_ARM_CORTEX_A15) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+8)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+8)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+8)*sdd]-bir*ps, sdd, 0, m-(i+8), bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_sgemm_nt_12x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+0)*sdc], &pD[j*ps+(i+0)*sdd], m-(i+0), n-j);
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], m-(i+4), n-j);
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+8)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+8)*sdc], &pD[j*ps+(i+8)*sdd], m-(i+8), n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		if(m-i<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_12;
			}
		}
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-7; i+=8)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-7; j+=8, idxB+=8)
			{
			kernel_sgemm_nt_8x8_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
//		for(; j<n-3; j+=4, idxB+=4)
//			{
//			kernel_sgemm_nt_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
//			}
		if(j<n)
			{
			if(n-j<=4)
				{
				kernel_sgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
				}
			else
				{
				kernel_sgemm_nt_8x8_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
				}
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else
			{
			goto left_8;
			}
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) //|| defined(TARGET_ARMV7A_ARM_CORTEX_A15)
	for(; i<m-7; i+=8)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_sgemm_nt_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+0)*sdc], &pD[j*ps+(i+0)*sdd], m-(i+0), n-j);
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], m-(i+4), n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else
			{
			goto left_8;
			}
		}
#else
	for(; i<m-3; i+=4)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_sgemm_nt_4x4_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif
	// common return if i==m
	return;



	// main loop C, D not aligned
loop_CD:
	i = 0;
	for(; i<m; i+=4)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n; j+=4, idxB+=4)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	// common return if i==m
	return;



	// clean up loops definitions

#if defined(TARGET_ARMV7A_ARM_CORTEX_A15) //|| defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+8)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+8)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+8)*sdd]-bir*ps, sdd, 0, m-(i+8), bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+0)*sdc], &pD[j*ps+(i+0)*sdd], m-(i+0), n-j);
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], m-(i+4), n-j);
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+8)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+8)*sdc], &pD[j*ps+(i+8)*sdd], m-(i+8), n-j);
		}
	return;
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n-4; j+=8, idxB+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(j<n)
		{
		kernel_sgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	left_8:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+0)*sdc], &pD[j*ps+(i+0)*sdd], m-(i+0), n-j);
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], m-(i+4), n-j);
		}
	return;
#endif

	left_4:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;

#if 0
	left_4_g:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;
#endif

	}



// sgemm nn
void blasfeo_hp_sgemm_nn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;

	int air = ai & (ps-1);
	int bir = bi & (ps-1);

	// pA, pB point to panels edges
	float *pA = sA->pA + aj*ps + (ai-air)*sda;
	float *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;

	int offsetB = bir;

	int ci0 = ci-air;
	int di0 = di-air;
	int offsetC;
	int offsetD;
	if(ci0>=0)
		{
		pC += ci0/ps*ps*sdc;
		offsetC = ci0%ps;
		}
	else
		{
		pC += -ps*sdc;
		offsetC = ps+ci0;
		}

	if(di0>=0)
		{
		pD += di0/ps*ps*sdd;
		offsetD = di0%ps;
		}
	else
		{
		pD += -ps*sdd;
		offsetD = ps+di0;
		}

	int i, j, l;



	// algorithm scheme
	if(air!=0)
		{
		goto clear_air;
		}
select_loop:
	if(offsetC==0 & offsetD==0)
		{
		goto loop_00;
		}
	else
		{
		goto loop_CD;
		}
	// should never get here
	return;



	// clean up at the beginning
clear_air:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_sgemm_nn_4x4_gen_lib4(k, &alpha, &pA[0], offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
		}
	m -= 1*ps-air;
	pA += 1*ps*sda;
	pC += 1*ps*sdc;
	pD += 1*ps*sdd;
	goto select_loop;



	// main loop aligned
loop_00:
	i = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nn_8x8_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
//		for(; j<n-3; j+=4)
//			{
//			kernel_sgemm_nn_8x4_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
//			}
		if(j<n)
			{
			if(n-j<=4)
				{
				kernel_sgemm_nn_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
				}
			else
				{
				kernel_sgemm_nn_8x8_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
				}

			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else
			{
			goto left_8;
			}
		}
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_sgemm_nn_8x4_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_sgemm_nn_4x4_vs_lib4(k, &alpha, &pA[(i+0)*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+(i+0)*sdc], &pD[j*ps+(i+0)*sdd], m-(i+0), n-j);
			kernel_sgemm_nn_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], m-(i+4), n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else
			{
			goto left_8;
			}
		}
#else
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_sgemm_nn_4x4_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			kernel_sgemm_nn_4x4_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif
	// common return if i==m
	return;



	// main loop C, D not aligned
loop_CD:
	i = 0;
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_sgemm_nn_4x4_gen_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	// common return if i==m
	return;



	// clean up loops definitions

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nn_8x8_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(j<n)
		{
		kernel_sgemm_nn_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	left_8:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_sgemm_nn_4x4_vs_lib4(k, &alpha, &pA[(i+0)*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+(i+0)*sdc], &pD[j*ps+(i+0)*sdd], m-(i+0), n-j);
		kernel_sgemm_nn_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], m-(i+4), n-j);
		}
	return;
#endif

	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_sgemm_nn_4x4_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;

	}



// sgemm_tn
void blasfeo_hp_sgemm_tn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgemm_tn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_sgemm_tn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// sgemm_tt
void blasfeo_hp_sgemm_tt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgemm_tt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_sgemm_tt: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_llnn
void blasfeo_hp_strsm_llnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_llnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_llnn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_llnu
void blasfeo_hp_strsm_llnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	if(m<=0 || n<=0)
		return;
	
	if(ai!=0 | bi!=0 | di!=0 | alpha!=1.0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsm_llnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_strsm_llnu: feature not implemented yet: ai=%d, bi=%d, di=%d, alpha=%f\n", ai, bi, di, alpha);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;
	// TODO alpha
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pD = sD->pA + dj*ps;

	int i, j;
	
	i = 0;

	for( ; i<m-3; i+=4)
		{
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_strsm_nn_ll_one_4x4_lib4(i, pA+i*sda, pD+j*ps, sdd, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps);
			}
		if(j<n)
			{
			kernel_strsm_nn_ll_one_4x4_vs_lib4(i, pA+i*sda, pD+j*ps, sdd, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps, m-i, n-j);
			}
		}
	if(i<m)
		{
		goto left_4;
		}

	// common return
	return;

	left_4:
	j = 0;
	for( ; j<n; j+=4)
		{
		kernel_strsm_nn_ll_one_4x4_vs_lib4(i, pA+i*sda, pD+j*ps, sdd, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps, m-i, n-j);
		}
	return;

	}



// strsm_lltn
void blasfeo_hp_strsm_lltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_lltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_lltn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_lltu
void blasfeo_hp_strsm_lltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_lltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_lltu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_lunn
void blasfeo_hp_strsm_lunn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	if(ai!=0 | bi!=0 | di!=0 | alpha!=1.0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsm_lunn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_strsm_lunn: feature not implemented yet: ai=%d, bi=%d, di=%d, alpha=%f\n", ai, bi, di, alpha);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;
	// TODO alpha
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pD = sD->pA + dj*ps;
	float *dA = sA->dA;
	int ii;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			sdiaex_lib(n, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		sdiaex_lib(n, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}

	int i, j, idx;
	float *dummy;
	
	i = 0;
	int rm = m%4;
	if(rm>0)
		{
		// TODO code expliticly the final case
		idx = m-rm; // position of the part to do
		j = 0;
		for( ; j<n; j+=4)
			{
			// XXX pA & pD are dummy and should not be used internally !!!
			kernel_strsm_nn_lu_inv_4x4_vs_lib4(0, pD, pA, 0, pB+idx*sdb+j*ps, pD+idx*sdd+j*ps, pA+idx*sda+idx*ps, dA+idx, rm, n-j);
			}
		// TODO
		i += rm;
		}
//	int em = m-rm;
	for( ; i<m; i+=4)
		{
		idx = m-i; // position of already done part
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_strsm_nn_lu_inv_4x4_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, dA+(idx-4));
			}
		if(j<n)
			{
			kernel_strsm_nn_lu_inv_4x4_vs_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, dA+(idx-4), 4, n-j);
			}
		}

	// common return
	return;

	}



// strsm_lunu
void blasfeo_hp_strsm_lunu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_lunu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_lunu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_lutn
void blasfeo_hp_strsm_lutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_lutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_lutn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_lutu
void blasfeo_hp_strsm_lutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_lutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_lutu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_rlnn
void blasfeo_hp_strsm_rlnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_rlnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_rlnn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_rlnu
void blasfeo_hp_strsm_rlnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_rlnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_rlnu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_right_lower_transposed_notunit
void blasfeo_hp_strsm_rltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsm_rltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_strsm_rltn: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pD = sD->pA + dj*ps;
	float *dA = sA->dA;

	int i, j;
	
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			sdiaex_lib(n, 1.0, ai, pA, sda, dA);
			for(i=0; i<n; i++)
				dA[i] = 1.0 / dA[i];
			sA->use_dA = 1;
			}
		}
	else
		{
		sdiaex_lib(n, 1.0, ai, pA, sda, dA);
		for(i=0; i<n; i++)
			dA[i] = 1.0 / dA[i];
		sA->use_dA = 0;
		}

	if(m<=0 || n<=0)
		return;
	
	i = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_16x4_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j]);
			}
		if(j<n)
			{
			kernel_strsm_nt_rl_inv_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else if(m-i<=8)
			{
			goto left_8;
			}
		else if(m-i<=12)
			{
			goto left_12;
			}
		else
			{
			goto left_16;
			}
		}
#elif 0 //defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_12x4_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j]);
			}
		if(j<n)
			{
			kernel_strsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else if(m-i<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_12;
			}
		}
#elif 0 //defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_8x4_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j]);
			}
		if(j<n)
			{
			kernel_strsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else
			{
			goto left_8;
			}
		}
#else
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j]);
			}
		if(j<n)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif

	// common return if i==m
	return;

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_16:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_strsm_nt_rl_inv_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_strsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_strsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		}
	return;
#endif

	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		}
	return;

	}



// strsm_right_lower_transposed_unit
void blasfeo_hp_strsm_rltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

		if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsm_rltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_strsm_rltu: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;
	// TODO alpha
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pD = sD->pA + dj*ps;

	int i, j;
	
	i = 0;

	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_one_4x4_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda]);
			}
		if(j<n)
			{
			kernel_strsm_nt_rl_one_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}

	// common return if i==m
	return;

	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_strsm_nt_rl_one_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], m-i, n-j);
		}
	return;

	}



// strsm_runn
void blasfeo_hp_strsm_runn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_runn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_runn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_runu
void blasfeo_hp_strsm_runu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_runu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_runu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// strsm_rutn
void blasfeo_hp_strsm_rutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_strsm_rutn: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;
	// TODO alpha
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pD = sD->pA + dj*ps;
	float *dA = sA->dA;
	int ii;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			sdiaex_lib(n, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		sdiaex_lib(n, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}

	int i, j, idx;

	int rn = n%4;

	// float *dummy;

	i = 0;

	for(; i<m-3; i+=4)
		{
		j = 0;
		// clean at the end
		if(rn>0)
			{
			idx = n-rn;
			// XXX pA & pD are dummy and should not be used internally !!!
			kernel_strsm_nt_ru_inv_4x4_vs_lib4(0, pD, pA, &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
			j += rn;
			}
		for(; j<n; j+=4)
			{
			idx = n-j-4;
			kernel_strsm_nt_ru_inv_4x4_lib4(j, &pD[i*sdd+(idx+4)*ps], &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx]);
			}
		}
	if(m>i)
		{
		goto left_4;
		}

	// common return if i==m
	return;

	left_4:
	j = 0;
	// TODO
	// clean at the end
	if(rn>0)
		{
		idx = n-rn;
		// XXX pA & pD are dummy and should not be used internally !!!
		kernel_strsm_nt_ru_inv_4x4_vs_lib4(0, pD, pA, &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
		j += rn;
		}
	for(; j<n; j+=4)
		{
		idx = n-j-4;
		kernel_strsm_nt_ru_inv_4x4_vs_lib4(j, &pD[i*sdd+(idx+4)*ps], &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx], m-i, 4);
		}
	return;

	}



// strsm_rutu
void blasfeo_hp_strsm_rutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_rutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_rutu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrmm_right_upper_transposed_notunit (B, i.e. the first matrix, is triangular !!!)
void blasfeo_hp_strmm_rutn(int m, int n, float alpha, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sD, int di, int dj)
	{
	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strmm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_strmm_rutn: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	if(m<=0 || n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pD = sD->pA + dj*ps;

	int i, j;
	
//	strmm_nt_ru_lib(m, n, alpha, pA, sda, pB, sdb, 0.0, pD, sdd, pD, sdd); 
	i = 0;
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strmm_nt_ru_4x4_lib4(n-j, &alpha, &pA[j*ps+i*sda], &pB[j*ps+j*sdb], &pD[j*ps+i*sdd]);
			}
		if(j<n) // TODO specialized edge routine
			{
			kernel_strmm_nt_ru_4x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(i<m)
		{
		goto left_4;
		}
	
	// common return
	return;

	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_strmm_nt_ru_4x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], m-i, n-j);
		}
	// TODO specialized edge routine

	return;

	}



// dtrmm_right_lower_nottransposed_notunit (B, i.e. the first matrix, is triangular !!!)
void blasfeo_hp_strmm_rlnn(int m, int n, float alpha, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pD = sD->pA + dj*ps;

	pA += ai/ps*ps*sda;
	pB += bi/ps*ps*sdb;
	int offsetB = bi%ps;
	int di0 = di-ai%ps;
	int offsetD;
	if(di0>=0)
		{
		pD += di0/ps*ps*sdd;
		offsetD = di0%ps;
		}
	else
		{
		pD += -4*sdd;
		offsetD = ps+di0;
		}
	
	int ii, jj;

	ii = 0;
	if(ai%ps!=0)
		{
		jj = 0;
		for(; jj<n; jj+=4)
			{
			kernel_strmm_nn_rl_4x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[ii*sdd+jj*ps], sdd, ai%ps, m-ii, 0, n-jj);
			}
		m -= ps-ai%ps;
		pA += ps*sda;
		pD += ps*sdd;
		}
	if(offsetD==0)
		{
		for(; ii<m-3; ii+=4)
			{
			jj = 0;
			for(; jj<n-5; jj+=4)
				{
				kernel_strmm_nn_rl_4x4_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps]);
				}
			for(; jj<n; jj+=4)
				{
				kernel_strmm_nn_rl_4x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, 0, &pD[ii*sdd+jj*ps], sdd, 0, 4, 0, n-jj);
				}
			}
		if(ii<m)
			{
			goto left_4;
			}
		}
	else
		{
		for(; ii<m; ii+=4)
			{
			jj = 0;
			for(; jj<n; jj+=4)
				{
				kernel_strmm_nn_rl_4x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[ii*sdd+jj*ps], sdd, 0, m-ii, 0, n-jj);
				}
			}
		}

	// common return if i==m
	return;

	// clean up loops definitions

	left_4:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_strmm_nn_rl_4x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[ii*sdd+jj*ps], sdd, 0, m-ii, 0, n-jj);
		}
	return;

	}



void blasfeo_hp_ssyrk_ln(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	// fast return
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	int air = ai & (ps-1);
	int bir = bi & (ps-1);
	float *pA = sA->pA + aj*ps + (ai-air)*sda;
	float *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;

	int ci0 = ci;//-air;
	int di0 = di;//-air;
	int offsetC;
	int offsetD;
	if(ci0>=0)
		{
		pC += ci0/ps*ps*sdc;
		offsetC = ci0%ps;
		}
	else
		{
		pC += -4*sdc;
		offsetC = ps+ci0;
		}
	if(di0>=0)
		{
		pD += di0/ps*ps*sdd;
		offsetD = di0%ps;
		}
	else
		{
		pD += -4*sdd;
		offsetD = ps+di0;
		}

	void *mem;
	float *pU, *pA2;
	int sdu, sda2;

// TODO visual studio alignment
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( float pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	float pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( float pU0[1*4*K_MAX_STACK], 64 );
#endif
	int sdu0 = (k+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	// allocate memory
	if(k>K_MAX_STACK)
		{
		sdu = (k+ps-1)/ps*ps;
		mem = malloc(8*sdu*sizeof(float)+63); // TODO update when bigger kernels are used !!!!!!!!!!!!!!!!
		blasfeo_align_64_byte(mem, (void **) &pU);
		}
	else
		{
		pU = pU0;
		sdu = sdu0;
		}
	

	int i, j, n1;

	int idxB;



	// algorithm scheme
	if(offsetC==0 & offsetD==0)
		{
		if(bir==0)
			{
//	printf("\n000\n");
			goto loop_000;
			}
		else
			{
//	printf("\nB00\n");
			goto loop_B00;
			}
		}
	else
		{
		if(bir==0)
			{
//	printf("\n0CD\n");
			goto loop_0CD;
			}
		else
			{
//	printf("\nBCD\n");
			goto loop_BCD;
			}
		}
	// should never get here
	goto end;



	// main loop aligned
loop_000:
	i = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-7; i+=8)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
//			kernel_spacp_nn_8_lib4(k, air, pA+i*sda, sda, pU, sdu);
			kernel_spacp_nn_4_lib4(k, air, pA+(i+0)*sda, sda, pU+0*sdu);
			kernel_spacp_nn_4_lib4(k, air, pA+(i+4)*sda, sda, pU+4*sdu);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
//		for(; j<i; j+=4)
//			{
//			kernel_sgemm_nt_8x4_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
//			}
		for(; j<i; j+=8)
			{
			kernel_sgemm_nt_8x8_lib4(k, &alpha, pA2, sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
//		if(j<i) // XXX not needed !!!
//			{
//			kernel_sgemm_nt_8x4_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
//			j += 4;
//			}
		kernel_ssyrk_nt_l_8x4_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
		kernel_ssyrk_nt_l_4x4_lib4(k, &alpha, pA2+4*sda2, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd]);
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4;
			}
		else
			{
			goto left_8;
			}
		}
#else
	for(; i<m-3; i+=4)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_spacp_nn_4_lib4(k, air, pA+i*sda, sda, pU);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_sgemm_nt_4x4_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		kernel_ssyrk_nt_l_4x4_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
		}
	if(m>i)
		{
		goto left_4;
		}
#endif
	// common return if i==m
	goto end;



	// main loop aligned
loop_B00:
	i = 0;
	for(; i<m-3; i+=4)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_spacp_nn_4_lib4(k, air, pA+i*sda, sda, pU);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		idxB = 0;
		if(j<i)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, 0, &pC[j*ps+i*sdc]-bir*ps, sdc, 0, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, 4);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_sgemm_nt_4x4_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
				}
			kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, bir);
			j += bir;
			}
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, 0, &pC[j*ps+i*sdc]-bir*ps, sdc, 0, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, 4);
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[(j+4)*sdb], &beta, 0, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, 0, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, 4);
		}
	if(m>i)
		{
		goto left_4_g;
		}
	// common return if i==m
	goto end;



	// main loop C, D not aligned
loop_0CD:
	i = 0;
	for(; i<m; i+=4)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_spacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
		}
	// common return if i==m
	goto end;



	// main loop aligned
loop_BCD:
	i = 0;
	for(; i<m; i+=4)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_spacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		idxB = 0;
		if(j<i)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, m-j);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
				}
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, bir); // XXX n1
			j += bir;
			}
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+m-j);
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[(j+4)*sdb], &beta, offsetC, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, m-j);
		}
	// common return if i==m
	goto end;



	// clean up loops definitions

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
//		kernel_spacp_nn_8_lib4(k, air, pA+i*sda, sda, pU, sdu);
		kernel_spacp_nn_4_lib4(k, air, pA+(i+0)*sda, sda, pU+0*sdu);
		kernel_spacp_nn_4_lib4(k, air, pA+(i+4)*sda, sda, pU+4*sdu);
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
//	for(; j<i; j+=4)
//		{
//		kernel_sgemm_nt_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
//		}
	for(; j<i; j+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		}
//	if(j<i) // XXX not needed !!!
//		{
//		kernel_sgemm_nt_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
//		j += 4;
//		}
	kernel_ssyrk_nt_l_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
	kernel_ssyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2+4*sda2, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, m-j-4);
	goto end;
#endif



	left_4:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_spacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i; j+=4)
		{
		// TODO 4x8 ???
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		}
	kernel_ssyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
	goto end;



	left_4_g:
	j = 0;
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_spacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
		pA2 = pU;
		sda2 = sdu;
		}
	if(bir!=0)
		{
		idxB = 0;
		if(j<i)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, m-j);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
				}
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, bir); // XXX n1
			j += bir;
			}
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+m-j);
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[(j+4)*sdb], &beta, offsetC, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, m-j);
		}
	else
		{
		// main loop
		for(; j<i; j+=4)
			{
			kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
		}
	goto end;



end:
	if(k>K_MAX_STACK)
		{
		free(mem);
		}
	return;



	}



void blasfeo_hp_ssyrk_ln_mn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 | n<=0)
		return;

	if(ai!=0 | bi!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_ssyrk_ln_mn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_ssyrk_ln_mn: feature not implemented yet: ai=%d, bi=%d\n", ai, bi);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int i, j;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pC = sC->pA + cj*ps + (ci-(ci&(ps-1)))*sdc;
	float *pD = sD->pA + dj*ps + (di-(di&(ps-1)))*sdd;

	// TODO ai and bi
	int offsetC;
	int offsetD;
	offsetC = ci&(ps-1);
	offsetD = di&(ps-1);

	// main loop
	i = 0;
	if(offsetC==0 & offsetD==0)
		{
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; i<m-7; i+=8)
			{
			j = 0;
//			for(; j<i & j<n-3; j+=4)
//				{
//				kernel_sgemm_nt_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
//				}
			for(; j<i & j<n-7; j+=8)
				{
				kernel_sgemm_nt_8x8_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
				}
			if(j<i & j<n-3)
				{
				kernel_sgemm_nt_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
				j += 4;
				}
			if(j<n)
				{
				if(j<i) // dgemm
					{
					kernel_sgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
					}
				else // dsyrk
					{
					if(j<n-7)
						{
						kernel_ssyrk_nt_l_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
						kernel_ssyrk_nt_l_4x4_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd]);
						}
					else
						{
						kernel_ssyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
						if(j<n-4)
							{
							kernel_ssyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, n-j-4);
							}
						}
					}
				}
			}
		if(m>i)
			{
			if(m-i<=4)
				{
				goto left_4;
				}
			else
				{
				goto left_8;
				}
			}
#else
		for(; i<m-3; i+=4)
			{
			j = 0;
			for(; j<i & j<n-3; j+=4)
				{
				kernel_sgemm_nt_4x4_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
				}
			if(j<n)
				{
				if(j<i) // dgemm
					{
					kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
					}
				else // dsyrk
					{
					if(j<n-3)
						{
						kernel_ssyrk_nt_l_4x4_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
						}
					else
						{
						kernel_ssyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
						}
					}
				}
			}
		if(m>i)
			{
			goto left_4;
			}
#endif
		}
	else
		{
		for(; i<m; i+=4)
			{
			j = 0;
			for(; j<i & j<n; j+=4)
				{
				kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
				}
			if(j<n)
				{
				kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
				}
			}
		}

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
//	for(; j<i & j<n; j+=4)
//		{
//		kernel_sgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
//		}
	for(; j<i-4 & j<n-4; j+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(j<i & j<n)
		{
		kernel_sgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		j += 4;
		}
	if(j<n)
		{
		kernel_ssyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		if(j<n-4)
			{
			kernel_ssyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, n-j-4);
			}
		}
	return;
#endif

	left_4:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_sgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		kernel_ssyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;

	left_4_gen:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_sgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	if(j<n)
		{
		kernel_ssyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;

	}



void blasfeo_hp_ssyrk_lt(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssyrk_lt(m,  k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_ssyrk_lt: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



void blasfeo_hp_ssyrk_un(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssyrk_un(m,  k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_ssyrk_un: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



void blasfeo_hp_ssyrk_ut(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssyrk_ut(m,  k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_ssyrk_ut: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



void blasfeo_hp_ssyr2k_ln(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	float d_1 = 1.0;
	blasfeo_hp_ssyrk_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_ssyrk_ln(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



void blasfeo_hp_ssyr2k_lt(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	float d_1 = 1.0;
	blasfeo_hp_ssyrk_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_ssyrk_lt(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



void blasfeo_hp_ssyr2k_un(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	float d_1 = 1.0;
	blasfeo_hp_ssyrk_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_ssyrk_un(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



void blasfeo_hp_ssyr2k_ut(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	float d_1 = 1.0;
	blasfeo_hp_ssyrk_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_ssyrk_ut(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_sgemm_nn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_nn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgemm_nt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_nt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgemm_tn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_tn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgemm_tt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_tt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_strsm_llnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_llnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_llnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_llnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_lltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_lltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_lunn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lunn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_lunu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lunu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_lutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_lutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_lutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_rlnn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rlnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_rlnu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rlnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_rltn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_rltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_runn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_runn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_runu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_runu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_rutn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strsm_rutu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strsm_rutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_strmm_rutn(int m, int n, float alpha, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strmm_rutn(m, n, alpha, sB, bi, bj, sA, ai, aj, sD, di, dj);
	}



void blasfeo_strmm_rlnn(int m, int n, float alpha, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_strmm_rlnn(m, n, alpha, sB, bi, bj, sA, ai, aj, sD, di, dj);
	}



void blasfeo_ssyrk_ln(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyrk_ln_mn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_ln_mn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyrk_lt(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyrk_un(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyrk_ut(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyr2k_ln(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyr2k_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyr2k_lt(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyr2k_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyr2k_un(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyr2k_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyr2k_ut(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyr2k_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



#endif



