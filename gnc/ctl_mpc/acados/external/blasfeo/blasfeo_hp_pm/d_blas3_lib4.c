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
#include <blasfeo_d_kernel.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blasfeo_api.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_d_blasfeo_ref_api.h>
#endif



/****************************
* old interface
****************************/

#if 0
void dlauum_blk_nt_l_lib(int m, int n, int nv, int *rv, int *cv, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd)
	{

	if(m<=0 || n<=0)
		return;

	// TODO remove
	double alpha, beta;
	if(alg==0)
		{
		alpha = 1.0;
		beta = 0.0;
		}
	else if(alg==1)
		{
		alpha = 1.0;
		beta = 1.0;
		}
	else
		{
		alpha = -1.0;
		beta = 1.0;
		}

	// TODO remove
	int k = cv[nv-1];

	const int ps = 4;

	int i, j, l;
	int ii, iii, jj, kii, kiii, kjj, k0, k1;

	i = 0;
	ii = 0;
	iii = 0;

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-7; i+=8)
		{

		while(ii<nv && rv[ii]<i+8)
			ii++;
		if(ii<nv)
			kii = cv[ii];
		else
			kii = cv[ii-1];

		j = 0;
		jj = 0;
		for(; j<i && j<n-3; j+=4)
			{

			while(jj<nv && rv[jj]<j+4)
				jj++;
			if(jj<nv)
				kjj = cv[jj];
			else
				kjj = cv[jj-1];
			k0 = kii<kjj ? kii : kjj;

			kernel_dgemm_nt_8x4_lib4(k0, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{

			while(jj<nv && rv[jj]<j+4)
				jj++;
			if(jj<nv)
				kjj = cv[jj];
			else
				kjj = cv[jj-1];
			k0 = kii<kjj ? kii : kjj;

			if(j<i) // dgemm
				{
				kernel_dgemm_nt_8x4_vs_lib4(k0, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, 8, n-j);
				}
			else // dsyrk
				{
				kernel_dsyrk_nt_l_8x4_vs_lib4(k0, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, 8, n-j);
				if(j<n-4)
					{
					kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], 4, n-j-4); // TODO
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

		while(ii<nv && rv[ii]<i+4)
			ii++;
		if(ii<nv)
			kii = cv[ii];
		else
			kii = cv[ii-1];
//		k0 = kii;
//		printf("\nii %d %d %d %d %d\n", i, ii, rv[ii], cv[ii], kii);

		j = 0;
		jj = 0;
		for(; j<i && j<n-3; j+=4)
			{

			while(jj<nv && rv[jj]<j+4)
				jj++;
			if(jj<nv)
				kjj = cv[jj];
			else
				kjj = cv[jj-1];
			k0 = kii<kjj ? kii : kjj;
//			printf("\njj %d %d %d %d %d\n", j, jj, rv[jj], cv[jj], kjj);

			kernel_dgemm_nt_4x4_lib4(k0, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{

			while(jj<nv && rv[jj]<j+4)
				jj++;
			if(jj<nv)
				kjj = cv[jj];
			else
				kjj = cv[jj-1];
			k0 = kii<kjj ? kii : kjj;
//			printf("\njj %d %d %d %d %d\n", j, jj, rv[jj], cv[jj], kjj);

			if(j<i) // dgemm
				{
				kernel_dgemm_nt_4x4_vs_lib4(k0, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], 4, n-j);
				}
			else // dsyrk
				{
				kernel_dsyrk_nt_l_4x4_vs_lib4(k0, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], 4, n-j);
				}
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	left_8:

	kii = cv[nv-1];

	j = 0;
	jj = 0;
	for(; j<i && j<n-3; j+=4)
		{

		while(jj<nv && rv[jj]<j+4)
			jj++;
		if(jj<nv)
			kjj = cv[jj];
		else
			kjj = cv[jj-1];
		k0 = kii<kjj ? kii : kjj;

		kernel_dgemm_nt_8x4_vs_lib4(k0, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(j<n)
		{

		while(jj<nv && rv[jj]<j+4)
			jj++;
		if(jj<nv)
			kjj = cv[jj];
		else
			kjj = cv[jj-1];
		k0 = kii<kjj ? kii : kjj;

		if(j<i) // dgemm
			{
			kernel_dgemm_nt_8x4_vs_lib4(k0, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
			}
		else // dsyrk
			{
			kernel_dsyrk_nt_l_8x4_vs_lib4(k0, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
			if(j<n-4)
				{
				kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, n-j-4); // TODO
				}
			}
		}
	return;
#endif

	left_4:

	kii = cv[nv-1];

	j = 0;
	jj = 0;
	for(; j<i && j<n-3; j+=4)
		{

		while(jj<nv && rv[jj]<j+4)
			jj++;
		if(jj<nv)
			kjj = cv[jj];
		else
			kjj = cv[jj-1];
		k0 = kii<kjj ? kii : kjj;

		kernel_dgemm_nt_4x4_vs_lib4(k0, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<n)
		{

		while(jj<nv && rv[jj]<j+4)
			jj++;
		if(jj<nv)
			kjj = cv[jj];
		else
			kjj = cv[jj-1];
		k0 = kii<kjj ? kii : kjj;

		if(j<i) // dgemm
			{
			kernel_dgemm_nt_4x4_vs_lib4(k0, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		else // dsyrk
			{
			kernel_dsyrk_nt_l_4x4_vs_lib4(k0, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	return;

	}
#endif



/****************************
* new interface
****************************/



// dgemm nn
void blasfeo_hp_dgemm_nn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
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
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pC = sC->pA + cj*ps;
	double *pD = sD->pA + dj*ps;

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
#if defined(TARGET_X64_INTEL_HASWELL)
	if(air+m>8)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nn_12x4_gen_lib4(k, &alpha, &pA[0], sda, offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
			}
		m -= 3*ps-air;
		pA += 3*ps*sda;
		pC += 3*ps*sdc;
		pD += 3*ps*sdd;
		}
	else // air+m<=8
#endif
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	if(air+m>4) // (m>5)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nn_8x4_gen_lib4(k, &alpha, &pA[0], sda, offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
			}
		m -= 2*ps-air;
		pA += 2*ps*sda;
		pC += 2*ps*sdc;
		pD += 2*ps*sdd;
		}
	else // air+m<=4 // m-i<=4
		{
#endif
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nn_4x4_gen_lib4(k, &alpha, &pA[0], offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
			}
		m -= 1*ps-air;
		pA += 1*ps*sda;
		pC += 1*ps*sdc;
		pD += 1*ps*sdd;
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		// nothing more to do
		}
#endif
	goto select_loop;



	// main loop aligned
loop_00:
	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dgemm_nn_12x4_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dgemm_nn_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-12 | i==m-8; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dgemm_nn_8x4_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
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
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dgemm_nn_8x4_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
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
#elif defined(TARGET_X86_AMD_BARCELONA)
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-1; j+=2)
			{
			kernel_dgemm_nn_4x2_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			kernel_dgemm_nn_4x2_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#else // all others
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dgemm_nn_4x4_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
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
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-8; i+=12)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nn_12x4_gen_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4_g;
			}
		else
			{
			goto left_8_g;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nn_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	if(m>i)
		{
		goto left_4_g;
		}
#else
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nn_4x4_gen_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
#endif
	// common return if i==m
	return;



	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12_g:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dgemm_nn_12x4_gen_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dgemm_nn_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_X64_INTEL_HASWELL)
	left_8_g:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dgemm_nn_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dgemm_nn_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#endif



	left_4_g:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dgemm_nn_4x4_gen_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;



#if defined(TARGET_X64_INTEL_HASWELL)
	left_4:
	j = 0;
	for(; j<n-8; j+=12)
		{
		kernel_dgemm_nn_4x12_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<n-4)
		{
		kernel_dgemm_nn_4x8_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	else if(j<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_4:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_dgemm_nn_4x8_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#elif defined(TARGET_X86_AMD_BARCELONA)
	left_4:
	j = 0;
	for(; j<n; j+=2)
		{
		kernel_dgemm_nn_4x2_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#else // all others
	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#endif

	return;

	}



// dgemm nt
void blasfeo_hp_dgemm_nt(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
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
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pC = sC->pA + cj*ps;
	double *pD = sD->pA + dj*ps;

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
#if defined(TARGET_X64_INTEL_HASWELL)
	if(air+m>8)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, &pA[0], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps]-bir*ps, sdc, offsetD, &pD[j*ps]-bir*ps, sdd, air, air+m, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, &pA[0], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
			}
		m -= 3*ps-air;
		pA += 3*ps*sda;
		pC += 3*ps*sdc;
		pD += 3*ps*sdd;
		}
	else // air+m<=8
#endif
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(air+m>4) // (m>5)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[0], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps]-bir*ps, sdc, offsetD, &pD[j*ps]-bir*ps, sdd, air, air+m, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[0], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
			}
		m -= 2*ps-air;
		pA += 2*ps*sda;
		pC += 2*ps*sdc;
		pD += 2*ps*sdd;
		}
	else // m<=4
		{
#endif
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[0], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps]-bir*ps, sdc, offsetD, &pD[j*ps]-bir*ps, sdd, air, air+m, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[0], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps], sdc, offsetD, &pD[j*ps], sdd, air, air+m, 0, n-j);
			}
		m -= ps-air;
		pA += ps*sda;
		pC += ps*sdc;
		pD += ps*sdd;
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		// nothing more to do
		}
#endif
	goto select_loop;



	// main loop aligned
loop_00:
	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-11; i+=12)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, 0, &pC[j*ps+i*sdc]-bir*ps, sdc, 0, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
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
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+8)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+8)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+8)*sdd]-bir*ps, sdd, 0, m-(i+8), bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; i<m-7; i+=8)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
#else
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, 0, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, 0, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
#endif
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
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
#elif defined(TARGET_X86_AMD_BARCELONA)
	for(; i<m-3; i+=4)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x2_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb+0], &beta, &pC[(j+0)*ps+i*sdc], &pD[(j+0)*ps+i*sdd]);
			kernel_dgemm_nt_4x2_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd]);
			}
		if(j<n-2)
			{
			kernel_dgemm_nt_4x2_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb+0], &beta, &pC[(j+0)*ps+i*sdc], &pD[(j+0)*ps+i*sdd]);
			kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd], m-i, n-j-2);
			}
		else if(j<n)
			{
			kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#else
	for(; i<m-3; i+=4)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n-3; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x4_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
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
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-8; i+=12)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4_g;
			}
		else
			{
			goto left_8_g;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-4; i+=8)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	if(m>i)
		{
		goto left_4_g;
		}
#else
	for(; i<m; i+=4)
		{
		j = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
			j += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; j<n; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
#endif
	// common return if i==m
	return;



	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+8)*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+(i+8)*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+(i+8)*sdd]-bir*ps, sdd, 0, m-(i+8), bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
	left_8:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n-8; j+=12, idxB+=12)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		kernel_dgemm_nt_8x8u_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[(idxB+4)*sdb], sdb, &beta, &pC[(j+4)*ps+i*sdc], sdc, &pD[(j+4)*ps+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<n)
		{
		if(n-j<=4)
			{
			kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
			}
		else
			{
			kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[(idxB+4)*sdb], &beta, &pC[(j+4)*ps+i*sdc], &pD[(j+4)*ps+i*sdd], m-i, n-(j+4));
			}
		}
	return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_8:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+0)*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+(i+0)*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+(i+0)*sdd]-bir*ps, sdd, 0, m-(i+0), bir, bir+n-j);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+(i+4)*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+(i+4)*sdd]-bir*ps, sdd, 0, m-(i+4), bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_8_g:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_4:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n-8; j+=12, idxB+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		if(n-j<=4)
			{
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		else
			{
			kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	left_4:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n-4; j+=8, idxB+=8)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#elif defined(TARGET_X86_AMD_BARCELONA)
	left_4:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n-2; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb+0], &beta, &pC[(j+0)*ps+i*sdc], &pD[(j+0)*ps+i*sdd], m-i, n-j-0);
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd], m-i, n-j-2);
		}
	if(j<n)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#else
	left_4:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#endif



	left_4_g:
	j = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+n-j);
		j += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; j<n; j+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;

	}



// dgemm_tn
void blasfeo_hp_dgemm_tn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
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
	int cir = ci & (ps-1);
	int dir = di & (ps-1);

	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pC = sC->pA + cj*ps + (ci-cir)*sdc;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;

	int offsetA = air;
	int offsetB = bir;
	int offsetC = cir;
	int offsetD = dir;

// TODO visual studio alignment
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
#endif
	int sdu0 = (k+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_dmat sAt;
	int sAt_size;
	void *mem;
	char *mem_align;

	double *pU;
	int sdu;

	int ii, jj;

	if(k>K_MAX_STACK)
		{
		sAt_size = blasfeo_memsize_dmat(12, k);
		mem = malloc(sAt_size+64);
		blasfeo_align_64_byte(mem, (void **) &mem_align);
		blasfeo_create_dmat(12, k, &sAt, (void *) mem_align);
		pU = sAt.pA;
		sdu = sAt.cn;
		}
	else
		{
		pU = pU0;
		sdu = sdu0;
		}


	// algorithm scheme
	if(offsetC==0 & offsetD==0)
		{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_X64_INTEL_CORE) | defined(TARGET_GENERIC)
		if(m<=n)
			{
			goto loop_00_m0; // transpose A
			}
		else
			{
			goto loop_00_n0; // transpose B
			}
#else
		goto loop_00_m0; // transpose A
#endif
		}
	else
		{
		goto loop_CD_m0;
		}
	// should never get here
	return;



loop_00_m0:
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_12x4_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_12x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto left_4_m0;
			}
		if(m-ii<=8)
			{
			goto left_8_m0;
			}
		else
			{
			goto left_12_m0;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_8x4_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto left_4_m0;
			}
		else
			{
			goto left_8_m0;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dgemm_nn_4x4_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps);
			}
		if(jj<n)
			{
			kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4_m0;
		}
#endif
	goto tn_return;



	// non-malloc algorith, C, D not aligned
loop_CD_m0:
	ii = 0;
	// clean up loops definitions
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-8; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
		for(jj=0; jj<n; jj+=4)
			{
			kernel_dgemm_nn_12x4_gen_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto left_4_m0_g;
			}
		else
			{
			goto left_8_m0_g;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-4; ii+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		for(jj=0; jj<n; jj+=4)
			{
			kernel_dgemm_nn_8x4_gen_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4_m0_g;
		}
#else
	for(; ii<m; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
		for(jj=0; jj<n; jj+=4)
			{
			kernel_dgemm_nn_4x4_gen_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
#endif
	// common return if i==m
	goto tn_return;



loop_00_n0:
	jj = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<n-11; jj+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+0)*ps, sdb, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+4)*ps, sdb, pU+4*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+8)*ps, sdb, pU+8*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x12_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, sdu, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x12_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, sdu, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto left_4_n0;
			}
		if(n-jj<=8)
			{
			goto left_8_n0;
			}
		else
			{
			goto left_12_n0;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; jj<n-7; jj+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+0)*ps, sdb, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+4)*ps, sdb, pU+4*sdu);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x8_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, sdu, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x8_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, sdu, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			goto left_4_n0;
			}
		else
			{
			goto left_8_n0;
			}
		}
#elif defined(TARGET_GENERIC) | defined(TARGET_X64_INTEL_CORE)
	for(; jj<n-3; jj+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetB, pB+jj*ps, sdb, pU);
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dgemm_tt_4x4_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps);
			}
		if(ii<m)
			{
			kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(jj<n)
		{
		goto left_4_n0;
		}
#endif
	// common return if n==n
	goto tn_return;



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_12_m0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_12x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tn_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_12_n0:
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+0)*ps, sdb, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+4)*ps, sdb, pU+4*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+8)*ps, sdb, pU+8*sdu);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x12_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, sdu, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_8_m0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_8x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tn_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
left_8_m0_g:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_8x4_gen_lib4(k, &alpha, pU, sdu, offsetB, pB+jj*ps, sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
		}
	goto tn_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
left_8_n0:
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+0)*ps, sdb, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+(jj+4)*ps, sdb, pU+4*sdu);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x8_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, sdu, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_4_m0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	for(jj=0; jj<n-8; jj+=12)
		{
		kernel_dgemm_nn_4x12_vs_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	if(jj<n-4)
		{
		kernel_dgemm_nn_4x8_vs_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	else if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
left_4_m0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	for(jj=0; jj<n-4; jj+=8)
		{
		kernel_dgemm_nn_4x8_vs_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#else // all others
left_4_m0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#endif



left_4_m0_g:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dgemm_nn_4x4_gen_lib4(k, &alpha, pU, offsetB, pB+jj*ps, sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
		}
	goto tn_return;



#if defined(TARGET_X64_INTEL_HASWELL)
left_4_n0:
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+jj*ps, sdb, pU);
	for(ii=0; ii<m-8; ii+=12)
		{
		kernel_dgemm_tt_12x4_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	if(ii<m-4)
		{
		kernel_dgemm_tt_8x4_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	else if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
left_4_n0:
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+jj*ps, sdb, pU);
	for(ii=0; ii<m-4; ii+=8)
		{
		kernel_dgemm_tt_8x4_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	if(ii<m)
		{
		kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#elif defined(TARGET_GENERIC) | defined(TARGET_X64_INTEL_CORE)
left_4_n0:
	kernel_dpacp_tn_4_lib4(k, offsetB, pB+jj*ps, sdb, pU);
	for(ii=0; ii<m; ii+=4)
		{
		kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, pA+ii*ps, sda, pU, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tn_return;
#endif



tn_return:
	if(k>K_MAX_STACK)
		{
		free(mem);
		}
	return;

	}



// dgemm_tt
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_X64_INTEL_CORE) | defined(TARGET_GENERIC)
void blasfeo_hp_dgemm_tt(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
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
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pC = sC->pA + (cj-bir)*ps;
	double *pD = sD->pA + (dj-bir)*ps;

	int offsetA = air;

	int ci0 = ci; //-bir;
	int di0 = di; //-bir;
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
	if(bir!=0)
		{
		goto clear_bir;
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
clear_bir:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(bir+n>8) // (m>9)
		{
		i = 0;
		for(; i<m; i+=4)
			{
			kernel_dgemm_tt_4x12_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[0], sdb, &beta, offsetC, &pC[i*sdc], sdc, offsetD, &pD[i*sdd], sdd, 0, m-i, bir, bir+n);
			}
		n -= 3*ps-bir;
		pB += 3*ps*sdb;
		pC += 3*4*ps;
		pD += 3*4*ps;
		}
	else // bir+n<=8
#endif
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(bir+n>4) // (m>5)
		{
		i = 0;
		for(; i<m; i+=4)
			{
			kernel_dgemm_tt_4x8_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[0], sdb, &beta, offsetC, &pC[i*sdc], sdc, offsetD, &pD[i*sdd], sdd, 0, m-i, bir, bir+n);
			}
		n -= 2*ps-bir;
		pB += 2*ps*sdb;
		pC += 2*4*ps;
		pD += 2*4*ps;
		}
	else // air+m<=4 // m-i<=4
		{
#endif
		i = 0;
		for(; i<m; i+=4)
			{
			kernel_dgemm_tt_4x4_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[0], &beta, offsetC, &pC[i*sdc], sdc, offsetD, &pD[i*sdd], sdd, 0, m-i, bir, bir+n);
			}
		n -= 1*ps-bir;
		pB += 1*ps*sdb;
		pC += 1*4*ps;
		pD += 1*4*ps;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		// nothing more to do
		}
#endif
	goto select_loop;



	// main loop aligned
loop_00:
	j = 0;
#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; j<n-11; j+=12)
		{
		i = 0;
		for(; i<m-3; i+=4)
			{
			kernel_dgemm_tt_4x12_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(i<m)
			{
			kernel_dgemm_tt_4x12_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(n>j)
		{
		if(n-j<=4)
			{
			goto left_4;
			}
		else if(n-j<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-12 | i==m-8; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dgemm_nn_8x4_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dgemm_nn_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; j<n-7; j+=8)
		{
		i = 0;
		for(; i<m-3; i+=4)
			{
			kernel_dgemm_tt_4x8_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(i<m)
			{
			kernel_dgemm_tt_4x8_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(n>j)
		{
		if(n-j<=4)
			{
			goto left_4;
			}
		else
			{
			goto left_8;
			}
		}
#elif 0//defined(TARGET_X86_AMD_BARCELONA)
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-1; j+=2)
			{
			kernel_dgemm_nn_4x2_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			kernel_dgemm_nn_4x2_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#else // all others
	for(; j<n-3; j+=4)
		{
		i = 0;
		for(; i<m-3; i+=4)
			{
			kernel_dgemm_tt_4x4_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(i<m)
			{
			kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(n>j)
		{
		goto left_4;
		}
#endif
	// common return if i==m
	return;



	// main loop C, D not aligned
loop_CD:
	j = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; j<n-8; j+=12)
		{
		i = 0;
		for(; i<m; i+=4)
			{
			kernel_dgemm_tt_4x12_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	if(n>j)
		{
		if(n-j<=4)
			{
			goto left_4_g;
			}
		else
			{
			goto left_8_g;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; j<n-4; j+=8)
		{
		i = 0;
		for(; i<m; i+=4)
			{
			kernel_dgemm_tt_4x8_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
	if(n>j)
		{
		goto left_4_g;
		}
#else
	for(; j<n; j+=4)
		{
		i = 0;
		for(; i<m; i+=4)
			{
			kernel_dgemm_tt_4x4_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
#endif
	// common return if i==m
	return;



	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12_g:
	i = 0;
	for(; i<m; i+=4)
		{
		kernel_dgemm_tt_4x12_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	i = 0;
	for(; i<m; i+=4)
		{
		kernel_dgemm_tt_4x12_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_8_g:
	i = 0;
	for(; i<m; i+=4)
		{
		kernel_dgemm_tt_4x8_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	i = 0;
	for(; i<m; i+=4)
		{
		kernel_dgemm_tt_4x8_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#endif



	left_4_g:
	i = 0;
	for(; i<m; i+=4)
		{
		kernel_dgemm_tt_4x4_gen_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	return;



#if defined(TARGET_X64_INTEL_HASWELL)
	left_4:
	i = 0;
	for(; i<m-8; i+=12)
		{
		kernel_dgemm_tt_12x4_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sda], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(i<m-4)
		{
		kernel_dgemm_tt_8x4_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	else if(i<m)
		{
		kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sda], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_4:
	i = 0;
	for(; i<m-4; i+=8)
		{
		kernel_dgemm_tt_8x4_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sda], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(i<m)
		{
		kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sda], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#elif 0//defined(TARGET_X86_AMD_BARCELONA)
	left_4:
	j = 0;
	for(; j<n; j+=2)
		{
		kernel_dgemm_nn_4x2_vs_lib4(k, &alpha, &pA[i*sda], offsetB, &pB[j*ps], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#else // all others
	left_4:
	i = 0;
	for(; i<m; i+=4)
		{
		kernel_dgemm_tt_4x4_vs_lib4(k, &alpha, offsetA, &pA[i*ps], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;
#endif

	return;

	}

#else
void blasfeo_hp_dgemm_tt(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
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
	int cir = ci & (ps-1);
	int dir = di & (ps-1);

	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pC = sC->pA + cj*ps + (ci-cir)*sdc;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;

	int offsetA = air;
	int offsetB = bir;
	int offsetC = cir;
	int offsetD = dir;

// TODO visual studio alignment
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU[1*4*K_MAX_STACK], 64 );
#endif
	int sdu = (k+3)/4*4;
	sdu = sdu<K_MAX_STACK ? sdu : K_MAX_STACK;

	struct blasfeo_dmat sAt;
	int sdat;
	int sAt_size;
	void *mem;
	char *mem_align;
	double *pAt;

	int ii, jj;

	int idxB;



	// algorithm scheme
	if(offsetC==0 & offsetD==0)
		{
		if(k>K_MAX_STACK)
			{
			goto loop_00_1;
			}
		else
			{
			goto loop_00_0;
			}
		}
	else
		{
		if(k>K_MAX_STACK)
			{
			goto loop_CD_1;
			}
		else
			{
			goto loop_CD_0;
			}
		}
	// should never get here
	return;



	// main loop aligned
loop_00_0:
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
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
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+0*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+4*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+8*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+8)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+8)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+8), bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
#else
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+0*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+4*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
#endif
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
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
#elif defined(TARGET_X86_AMD_BARCELONA)
	for(; ii<m-3; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x2_lib4(k, &alpha, pU, pB+idxB*sdb+0, &beta, pC+ii*sdc+(jj+0)*ps, pD+ii*sdd+(jj+0)*ps);
			kernel_dgemm_nt_4x2_lib4(k, &alpha, pU, pB+idxB*sdb+2, &beta, pC+ii*sdc+(jj+2)*ps, pD+ii*sdd+(jj+2)*ps);
			}
		if(jj<n-2)
			{
			kernel_dgemm_nt_4x2_lib4(k, &alpha, pU, pB+idxB*sdb+0, &beta, pC+ii*sdc+(jj+0)*ps, pD+ii*sdd+(jj+0)*ps);
			kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pU, pB+idxB*sdb+2, &beta, pC+ii*sdc+(jj+2)*ps, pD+ii*sdd+(jj+2)*ps, m-ii, n-(jj+2));
			}
		else if(jj<n)
			{
			kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4_0;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x4_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4_0;
		}
#endif
	goto tt_0_return;



	// main loop C, D not aligned
loop_CD_0:
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-8; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
	if(m>ii)
		{
		if(m-ii<=4)
			{
			goto left_4_0_g;
			}
		else
			{
			goto left_8_0_g;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-4; ii+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
	if(m>ii)
		{
		goto left_4_0_g;
		}
#else
	for(; ii<m; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
#endif
	// common return if i==m
	goto tt_0_return;



#if defined(TARGET_X64_INTEL_HASWELL)
left_12_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tt_0_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_12_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pU+8*sdu);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+0*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+4*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+8*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+8)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+8)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+8), bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tt_0_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_8_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-8; jj+=12, idxB+=12)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, pU, sdu, pB+(idxB+0)*sdb, sdb, &beta, pC+ii*sdc+(jj+0)*ps, sdc, pD+ii*sdd+(jj+0)*ps, sdd, m-ii, n-(jj+0));
		kernel_dgemm_nt_8x8u_vs_lib4(k, &alpha, pU, sdu, pB+(idxB+4)*sdb, sdb, &beta, pC+ii*sdc+(jj+4)*ps, sdc, pD+ii*sdd+(jj+4)*ps, sdd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		else
			{
			kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, pU, sdu, pB+(idxB+0)*sdb, sdb, &beta, pC+ii*sdc+(jj+0)*ps, sdc, pD+ii*sdd+(jj+0)*ps, sdd, m-ii, n-(jj+0));
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pU, pB+(idxB+4)*sdb, &beta, pC+ii*sdc+(jj+4)*ps, pD+ii*sdd+(jj+4)*ps, m-ii, n-(jj+4));
			}
		}
	goto tt_0_return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_8_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
#else
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+0*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU+4*sdu, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
#endif
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tt_0_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_8_0_g:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pU+0*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pU+4*sdu);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pU, sdu, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
		}
	goto tt_0_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_4_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-8; jj+=12, idxB+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4(k, &alpha, pU, pB+idxB*sdb, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		else
			{
			kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, pU, pB+idxB*sdb, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	goto tt_0_return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
left_4_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-4; jj+=8, idxB+=8)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, pU, pB+idxB*sdb, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tt_0_return;
#elif defined(TARGET_X86_AMD_BARCELONA)
left_4_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-2; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pU, pB+idxB*sdb+0, &beta, pC+ii*sdc+(jj+0)*ps, pD+ii*sdd+(jj+0)*ps, m-ii, n-(jj+0));
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pU, pB+idxB*sdb+2, &beta, pC+ii*sdc+(jj+2)*ps, pD+ii*sdd+(jj+2)*ps, m-ii, n-(jj+2));
		}
	if(jj<n)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tt_0_return;
#else
left_4_0:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tt_0_return;
#endif



left_4_0_g:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pU);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pU, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
		}
	goto tt_0_return;



tt_0_return:
	return;



loop_00_1:
	sAt_size = blasfeo_memsize_dmat(12, k);
	mem = malloc(sAt_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, k, &sAt, (void *) mem_align);
	pAt = sAt.pA;
	sdat = sAt.cn;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pAt+8*sdat);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
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
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pAt+8*sdat);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+0*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+4*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+8*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+8)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+8)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+8), bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
#else
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+0*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+4*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
#endif
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		}
	if(ii<m)
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
#elif defined(TARGET_X86_AMD_BARCELONA)
	for(; ii<m-3; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x2_lib4(k, &alpha, pAt, pB+idxB*sdb+0, &beta, pC+ii*sdc+(jj+0)*ps, pD+ii*sdd+(jj+0)*ps);
			kernel_dgemm_nt_4x2_lib4(k, &alpha, pAt, pB+idxB*sdb+2, &beta, pC+ii*sdc+(jj+2)*ps, pD+ii*sdd+(jj+2)*ps);
			}
		if(jj<n-2)
			{
			kernel_dgemm_nt_4x2_lib4(k, &alpha, pAt, pB+idxB*sdb+0, &beta, pC+ii*sdc+(jj+0)*ps, pD+ii*sdd+(jj+0)*ps);
			kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pAt, pB+idxB*sdb+2, &beta, pC+ii*sdc+(jj+2)*ps, pD+ii*sdd+(jj+2)*ps, m-ii, n-(jj+2));
			}
		else if(jj<n)
			{
			kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4_1;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n-3; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x4_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps);
			}
		if(jj<n)
			{
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4_1;
		}
#endif
	goto tt_1_return;



	// main loop C, D not aligned
loop_CD_1:
	sAt_size = blasfeo_memsize_dmat(12, k);
	mem = malloc(sAt_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, k, &sAt, (void *) mem_align);
	pAt = sAt.pA;
	sdat = sAt.cn;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-8; ii+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pAt+8*sdat);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
	if(m>ii)
		{
		if(m-ii<=4)
			{
			goto left_4_1_g;
			}
		else
			{
			goto left_8_1_g;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-4; ii+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
	if(m>ii)
		{
		goto left_4_1_g;
		}
#else
	for(; ii<m; ii+=4)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
		jj = 0;
		idxB = 0;
		// clean up at the beginning
		if(bir!=0)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
			jj += ps-bir;
			idxB += 4;
			}
		// main loop
		for(; jj<n; jj+=4, idxB+=4)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
			}
		}
#endif
	// common return if i==m
	goto tt_1_return;



#if defined(TARGET_X64_INTEL_HASWELL)
left_12_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pAt+8*sdat);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tt_1_return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_12_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+8)*ps, sda, pAt+8*sdat);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+0*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+4*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+8*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+8)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+8)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+8), bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tt_1_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_8_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-8; jj+=12, idxB+=12)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, pAt, sdat, pB+(idxB+0)*sdb, sdb, &beta, pC+ii*sdc+(jj+0)*ps, sdc, pD+ii*sdd+(jj+0)*ps, sdd, m-ii, n-(jj+0));
		kernel_dgemm_nt_8x8u_vs_lib4(k, &alpha, pAt, sdat, pB+(idxB+4)*sdb, sdb, &beta, pC+ii*sdc+(jj+4)*ps, sdc, pD+ii*sdd+(jj+4)*ps, sdd, m-ii, n-(jj+4));
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
			}
		else
			{
			kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, pAt, sdat, pB+(idxB+0)*sdb, sdb, &beta, pC+ii*sdc+(jj+0)*ps, sdc, pD+ii*sdd+(jj+0)*ps, sdd, m-ii, n-(jj+0));
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pAt, pB+(idxB+4)*sdb, &beta, pC+ii*sdc+(jj+4)*ps, pD+ii*sdd+(jj+4)*ps, m-ii, n-(jj+4));
			}
		}
	goto tt_1_return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_8_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
#else
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+0*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+0)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+0)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+0), bir, bir+n-jj);
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt+4*sdat, pB+idxB*sdb, &beta, 0, pC+(ii+4)*sdc+jj*ps-bir*ps, sdc, 0, pD+(ii+4)*sdd+jj*ps-bir*ps, sdd, 0, m-(ii+4), bir, bir+n-jj);
#endif
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, sdc, pD+ii*sdd+jj*ps, sdd, m-ii, n-jj);
		}
	goto tt_1_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
left_8_1_g:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+0)*ps, sda, pAt+0*sdat);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(ii+4)*ps, sda, pAt+4*sdat);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pAt, sdat, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
		}
	goto tt_1_return;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_4_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-8; jj+=12, idxB+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	if(jj<n)
		{
		if(n-jj<=4)
			{
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		else
			{
			kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
			}
		}
	goto tt_1_return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
left_4_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-4; jj+=8, idxB+=8)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	if(jj<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tt_1_return;
#elif defined(TARGET_X86_AMD_BARCELONA)
left_4_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n-2; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pAt, pB+idxB*sdb+0, &beta, pC+ii*sdc+(jj+0)*ps, pD+ii*sdd+(jj+0)*ps, m-ii, n-(jj+0));
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pAt, pB+idxB*sdb+2, &beta, pC+ii*sdc+(jj+2)*ps, pD+ii*sdd+(jj+2)*ps, m-ii, n-(jj+2));
		}
	if(jj<n)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tt_1_return;
#else
left_4_1:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, 0, pC+ii*sdc+jj*ps-bir*ps, sdc, 0, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, pC+ii*sdc+jj*ps, pD+ii*sdd+jj*ps, m-ii, n-jj);
		}
	goto tt_1_return;
#endif



left_4_1_g:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+ii*ps, sda, pAt);
	jj = 0;
	idxB = 0;
	// clean up at the beginning
	if(bir!=0)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps-bir*ps, sdc, offsetD, pD+ii*sdd+jj*ps-bir*ps, sdd, 0, m-ii, bir, bir+n-jj);
		jj += ps-bir;
		idxB += 4;
		}
	// main loop
	for(; jj<n; jj+=4, idxB+=4)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pAt, pB+idxB*sdb, &beta, offsetC, pC+ii*sdc+jj*ps, sdc, offsetD, pD+ii*sdd+jj*ps, sdd, 0, m-ii, 0, n-jj);
		}
	goto tt_1_return;



tt_1_return:
	free(mem);
	return;

	}
#endif



// dtrsm_llnn
void blasfeo_hp_dtrsm_llnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsm_llnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrsm_llnn: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	const int ps = 4;

	// TODO alpha
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pD = sD->pA + dj*ps;
	double *dA = sA->dA;

	if(m<=0 || n<=0)
		return;

	int i, j;

	if(ai==0 & aj==0)
		{
		// recompute diagonal if size of operation grows
		if(sA->use_dA<m)
			{
			ddiaex_lib(m, 1.0, ai, pA, sda, dA);
			for(i=0; i<m; i++)
				dA[i] = 1.0 / dA[i];
			sA->use_dA = m;
			}
		}
	// if submatrix recompute diagonal
	else
		{
		ddiaex_lib(m, 1.0, ai, pA, sda, dA);
		for(i=0; i<m; i++)
			dA[i] = 1.0 / dA[i];
		sA->use_dA = 0;
		}

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for( ; i<m-11; i+=12)
		{
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_ll_inv_12x4_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, dA+i);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_ll_inv_12x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, dA+i, m-i, n-j);
			}
		}
	if(i<m)
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for( ; i<m-7; i+=8)
		{
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_ll_inv_8x4_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, dA+i);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_ll_inv_8x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, dA+i, m-i, n-j);
			}
		}
	if(i<m)
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
	for( ; i<m-3; i+=4)
		{
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_ll_inv_4x4_lib4(i, pA+i*sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps, dA+i);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_ll_inv_4x4_vs_lib4(i, pA+i*sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps, dA+i, m-i, n-j);
			}
		}
	if(i<m)
		{
		goto left_4;
		}
#endif
	// common return
	return;

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12:
	j = 0;
	for( ; j<n; j+=4)
		{
		kernel_dtrsm_nn_ll_inv_12x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, dA+i, m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	left_8:
	j = 0;
	for( ; j<n; j+=4)
		{
		kernel_dtrsm_nn_ll_inv_8x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, dA+i, m-i, n-j);
		}
	return;
#endif

	left_4:
	j = 0;
	for( ; j<n; j+=4)
		{
		kernel_dtrsm_nn_ll_inv_4x4_vs_lib4(i, pA+i*sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps, dA+i, m-i, n-j);
		}
	return;

	}



// dtrsm_llnu
void blasfeo_hp_dtrsm_llnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsm_llnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrsm_llnu: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	const int ps = 4;

	// TODO alpha
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pD = sD->pA + dj*ps;

	if(m<=0 || n<=0)
		return;

	int i, j;

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for( ; i<m-11; i+=12)
		{
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_ll_one_12x4_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_ll_one_12x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, m-i, n-j);
			}
		}
	if(i<m)
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for( ; i<m-7; i+=8)
		{
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_ll_one_8x4_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_ll_one_8x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, m-i, n-j);
			}
		}
	if(i<m)
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
	for( ; i<m-3; i+=4)
		{
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_ll_one_4x4_lib4(i, pA+i*sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_ll_one_4x4_vs_lib4(i, pA+i*sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps, m-i, n-j);
			}
		}
	if(i<m)
		{
		goto left_4;
		}
#endif
	// common return
	return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for( ; j<n; j+=4)
		{
		kernel_dtrsm_nn_ll_one_12x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for( ; j<n; j+=4)
		{
		kernel_dtrsm_nn_ll_one_8x4_vs_lib4(i, pA+i*sda, sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, sdb, pD+i*sdd+j*ps, sdd, pA+i*sda+i*ps, sda, m-i, n-j);
		}
	return;
#endif

	left_4:
	j = 0;
	for( ; j<n; j+=4)
		{
		kernel_dtrsm_nn_ll_one_4x4_vs_lib4(i, pA+i*sda, pD+j*ps, sdd, &alpha, pB+i*sdb+j*ps, pD+i*sdd+j*ps, pA+i*sda+i*ps, m-i, n-j);
		}
	return;

	}



// dtrsm_lltn
void blasfeo_hp_dtrsm_lltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_lltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_lltn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_lltu
void blasfeo_hp_dtrsm_lltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_lltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_lltu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_lunn
void blasfeo_hp_dtrsm_lunn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	if(ai!=0 | bi!=0 | di!=0 | alpha!=1.0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsm_lunn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrsm_lunn: feature not implemented yet: ai=%d, bi=%d, di=%d, alpha=%f\n", ai, bi, di, alpha);
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
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pD = sD->pA + dj*ps;
	double *dA = sA->dA;
	int ii;

	int i, j, idx;
//	double *dummy;

	if(ai==0 & aj==0)
		{
		// recompute diagonal if size of operation grows
		if(sA->use_dA<m)
			{
			ddiaex_lib(m, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = m;
			}
		}
	// if submatrix recompute diagonal
	else
		{
		ddiaex_lib(m, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}

	i = 0;
	int rm = m%4;
	if(rm>0)
		{
		// TODO code expliticly the final case
		idx = m-rm; // position of the part to do
		j = 0;
		for( ; j<n; j+=4)
			{
//			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(0, dummy, dummy, 0, pB+idx*sdb+j*ps, pD+idx*sdd+j*ps, pA+idx*sda+idx*ps, dA+idx, rm, n-j);
			// XXX pA & pD are dummy and should not be used internally !!!
			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(0, pA, pD, sdd, pB+idx*sdb+j*ps, pD+idx*sdd+j*ps, pA+idx*sda+idx*ps, dA+idx, rm, n-j);
			}
		// TODO
		i += rm;
		}
//	int em = m-rm;
#if defined(TARGET_X64_INTEL_HASWELL)
	for( ; i<m-8; i+=12)
		{
		idx = m-i; // position of already done part
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_lu_inv_12x4_lib4(i, pA+(idx-12)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-12)*sdb+j*ps, sdb, pD+(idx-12)*sdd+j*ps, sdd, pA+(idx-12)*sda+(idx-12)*ps, sda, dA+(idx-12));
			}
		if(j<n)
			{
			kernel_dtrsm_nn_lu_inv_12x4_vs_lib4(i, pA+(idx-12)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-12)*sdb+j*ps, sdb, pD+(idx-12)*sdd+j*ps, sdd, pA+(idx-12)*sda+(idx-12)*ps, sda, dA+(idx-12), 12, n-j);
//			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, dA+(idx-4), 4, n-j);
//			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(i+4, pA+(idx-8)*sda+(idx-4)*ps, pD+(idx-4)*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, pD+(idx-8)*sdd+j*ps, pA+(idx-8)*sda+(idx-8)*ps, dA+(idx-8), 4, n-j);
//			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(i+8, pA+(idx-12)*sda+(idx-8)*ps, pD+(idx-8)*sdd+j*ps, sdd, pB+(idx-12)*sdb+j*ps, pD+(idx-12)*sdd+j*ps, pA+(idx-12)*sda+(idx-12)*ps, dA+(idx-12), 4, n-j);
			}
		}
#endif
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	for( ; i<m-4; i+=8)
		{
		idx = m-i; // position of already done part
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_lu_inv_8x4_lib4(i, pA+(idx-8)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, sdb, pD+(idx-8)*sdd+j*ps, sdd, pA+(idx-8)*sda+(idx-8)*ps, sda, dA+(idx-8));
			}
		if(j<n)
			{
			kernel_dtrsm_nn_lu_inv_8x4_vs_lib4(i, pA+(idx-8)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, sdb, pD+(idx-8)*sdd+j*ps, sdd, pA+(idx-8)*sda+(idx-8)*ps, sda, dA+(idx-8), 8, n-j);
//			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, dA+(idx-4), 4, n-j);
//			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(i+4, pA+(idx-8)*sda+(idx-4)*ps, pD+(idx-4)*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, pD+(idx-8)*sdd+j*ps, pA+(idx-8)*sda+(idx-8)*ps, dA+(idx-8), 4, n-j);
			}
		}
#endif
	for( ; i<m; i+=4)
		{
		idx = m-i; // position of already done part
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_lu_inv_4x4_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, dA+(idx-4));
			}
		if(j<n)
			{
			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, dA+(idx-4), 4, n-j);
			}
		}

	// common return
	return;

	}



// dtrsm_lunu
void blasfeo_hp_dtrsm_lunu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	if(ai!=0 | bi!=0 | di!=0 | alpha!=1.0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsm_lunu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrsm_lunu: feature not implemented yet: ai=%d, bi=%d, di=%d, alpha=%f\n", ai, bi, di, alpha);
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
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pD = sD->pA + dj*ps;
	int ii;

	int i, j, idx;
//	double *dummy;

	i = 0;
	int rm = m%4;
	if(rm>0)
		{
		// TODO code expliticly the final case
		idx = m-rm; // position of the part to do
		j = 0;
		for( ; j<n; j+=4)
			{
//			kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(0, dummy, dummy, 0, pB+idx*sdb+j*ps, pD+idx*sdd+j*ps, pA+idx*sda+idx*ps, dA+idx, rm, n-j);
			// XXX pA & pD are dummy and should not be used internally !!!
			kernel_dtrsm_nn_lu_one_4x4_vs_lib4(0, pA, pD, sdd, pB+idx*sdb+j*ps, pD+idx*sdd+j*ps, pA+idx*sda+idx*ps, rm, n-j);
			}
		// TODO
		i += rm;
		}
//	int em = m-rm;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for( ; i<m-8; i+=12)
		{
		idx = m-i; // position of already done part
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_lu_one_12x4_lib4(i, pA+(idx-12)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-12)*sdb+j*ps, sdb, pD+(idx-12)*sdd+j*ps, sdd, pA+(idx-12)*sda+(idx-12)*ps, sda);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_lu_one_12x4_vs_lib4(i, pA+(idx-12)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-12)*sdb+j*ps, sdb, pD+(idx-12)*sdd+j*ps, sdd, pA+(idx-12)*sda+(idx-12)*ps, sda, 12, n-j);
//			kernel_dtrsm_nn_lu_one_4x4_vs_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, 4, n-j);
//			kernel_dtrsm_nn_lu_one_4x4_vs_lib4(i+4, pA+(idx-8)*sda+(idx-4)*ps, pD+(idx-4)*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, pD+(idx-8)*sdd+j*ps, pA+(idx-8)*sda+(idx-8)*ps, 4, n-j);
//			kernel_dtrsm_nn_lu_one_4x4_vs_lib4(i+8, pA+(idx-12)*sda+(idx-8)*ps, pD+(idx-8)*sdd+j*ps, sdd, pB+(idx-12)*sdb+j*ps, pD+(idx-12)*sdd+j*ps, pA+(idx-12)*sda+(idx-12)*ps, 4, n-j);
			}
		}
#endif
#if 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	for( ; i<m-4; i+=8)
		{
		idx = m-i; // position of already done part
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_lu_one_8x4_lib4(i, pA+(idx-8)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, sdb, pD+(idx-8)*sdd+j*ps, sdd, pA+(idx-8)*sda+(idx-8)*ps, sda));
			}
		if(j<n)
			{
			kernel_dtrsm_nn_lu_one_8x4_vs_lib4(i, pA+(idx-8)*sda+idx*ps, sda, pD+idx*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, sdb, pD+(idx-8)*sdd+j*ps, sdd, pA+(idx-8)*sda+(idx-8)*ps, sda, 8, n-j);
//			kernel_dtrsm_nn_lu_one_4x4_vs_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, 4, n-j);
//			kernel_dtrsm_nn_lu_one_4x4_vs_lib4(i+4, pA+(idx-8)*sda+(idx-4)*ps, pD+(idx-4)*sdd+j*ps, sdd, pB+(idx-8)*sdb+j*ps, pD+(idx-8)*sdd+j*ps, pA+(idx-8)*sda+(idx-8)*ps, 4, n-j);
			}
		}
#endif
	for( ; i<m; i+=4)
		{
		idx = m-i; // position of already done part
		j = 0;
		for( ; j<n-3; j+=4)
			{
			kernel_dtrsm_nn_lu_one_4x4_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps);
			}
		if(j<n)
			{
			kernel_dtrsm_nn_lu_one_4x4_vs_lib4(i, pA+(idx-4)*sda+idx*ps, pD+idx*sdd+j*ps, sdd, pB+(idx-4)*sdb+j*ps, pD+(idx-4)*sdd+j*ps, pA+(idx-4)*sda+(idx-4)*ps, 4, n-j);
			}
		}

	// common return
	return;

	}



// dtrsm_lutn
void blasfeo_hp_dtrsm_lutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_lutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_lutn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_lutu
void blasfeo_hp_dtrsm_lutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_lutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_lutu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_rlnn
void blasfeo_hp_dtrsm_rlnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_rlnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_rlnn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_rlnu
void blasfeo_hp_dtrsm_rlnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_rlnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_rlnu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_right_lower_transposed_notunit
void blasfeo_hp_dtrsm_rltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	const int ps = 4;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	// TODO alpha !!!!!

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	int bir = bi & (ps-1);
	int dir = di & (ps-1);
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;
	double *dA = sA->dA;

	if(ai!=0 | bir!=0 | dir!=0 | alpha!=1.0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsm_rltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrsm_rltn: feature not implemented yet: ai=%d, bi=%d, di=%d, alpha=%f\n", ai, bi, di, alpha);
		exit(1);
#endif
		}

	int i, j;

	// TODO to avoid touching A, better temporarely use sD.dA ?????
	if(ai==0 & aj==0)
		{
		if(sA->use_dA<n)
			{
			ddiaex_lib(n, 1.0, ai, pA, sda, dA);
			for(i=0; i<n; i++)
				dA[i] = 1.0 / dA[i];
			sA->use_dA = n;
			}
		}
	else
		{
		ddiaex_lib(n, 1.0, ai, pA, sda, dA);
		for(i=0; i<n; i++)
			dA[i] = 1.0 / dA[i];
		sA->use_dA = 0;
		}

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j]);
			}
		if(j<n)
			{
			kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j]);
			}
		if(j<n)
			{
			kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
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
#elif defined(TARGET_X86_AMD_BARCELONA)
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x2_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j]);
			kernel_dtrsm_nt_rl_inv_4x2_lib4(j+2, &pD[i*sdd], &pA[j*sda+2], &alpha, &pB[(j+2)*ps+i*sdb], &pD[(j+2)*ps+i*sdd], &pA[(j+2)*ps+j*sda+2], &dA[j+2]);
			}
		if(j<n)
			{
			kernel_dtrsm_nt_rl_inv_4x2_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j], m-i, n-j);
			if(j<n-2)
				kernel_dtrsm_nt_rl_inv_4x2_vs_lib4(j+2, &pD[i*sdd], &pA[j*sda+2], &alpha, &pB[(j+2)*ps+i*sdb], &pD[(j+2)*ps+i*sdd], &pA[(j+2)*ps+j*sda+2], &dA[j+2], m-i, n-(j+2));
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#else
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j]);
			}
		if(j<n)
			{
			kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif

	// common return if i==m
	return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
	left_8:
	j = 0;
	for(; j<n-8; j+=12)
		{
		kernel_dtrsm_nt_rl_inv_8x8l_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], sda, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], sda, &dA[j], m-i, n-j);
		kernel_dtrsm_nt_rl_inv_8x8u_vs_lib4((j+4), &pD[i*sdd], sdd, &pA[(j+4)*sda], sda, &pB[(j+4)*ps+i*sdb], sdb, &pD[(j+4)*ps+i*sdd], sdd, &pA[(j+4)*ps+(j+4)*sda], sda, &dA[(j+4)], m-i, n-(j+4));
		}
	if(j<n-4)
		{
		kernel_dtrsm_nt_rl_inv_8x8l_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], sda, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], sda, &dA[j], m-i, n-j);
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4((j+4), &pD[i*sdd], &pA[(j+4)*sda], &alpha, &pB[(j+4)*ps+i*sdb], &pD[(j+4)*ps+i*sdd], &pA[(j+4)*ps+(j+4)*sda], &dA[(j+4)], m-i, n-(j+4));
		j += 8;
		}
	else if(j<n)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		j += 4;
		}
	return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
	left_4:
	j = 0;
	for(; j<n-8; j+=12)
		{
		kernel_dtrsm_nt_rl_inv_4x12_vs_lib4(j, &pD[i*sdd], &pA[j*sda], sda, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], sda, &dA[j], m-i, n-j);
		}
	if(j<n-4)
		{
		kernel_dtrsm_nt_rl_inv_4x8_vs_lib4(j, &pD[i*sdd], &pA[j*sda], sda, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], sda, &dA[j], m-i, n-j);
		j += 8;
		}
	else if(j<n)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		j += 4;
		}
	return;
#elif defined(TARGET_X86_AMD_BARCELONA)
	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x2_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		if(j<n-2)
		{
			kernel_dtrsm_nt_rl_inv_4x2_vs_lib4(j+2, &pD[i*sdd], &pA[j*sda+2], &alpha, &pB[(j+2)*ps+i*sdb], &pD[(j+2)*ps+i*sdd], &pA[(j+2)*ps+j*sda+2], &dA[j+2], m-i, n-(j+2));
			}
		}
	return;
#else
	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], &dA[j], m-i, n-j);
		}
	return;
#endif
	}



// dtrsm_right_lower_transposed_unit
void blasfeo_hp_dtrsm_rltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsm_rltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrsm_rltu: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pD = sD->pA + dj*ps;

	int i, j;

	i = 0;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrsm_nt_rl_one_12x4_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda]);
			}
		if(j<n)
			{
			kernel_dtrsm_nt_rl_one_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], m-i, n-j);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrsm_nt_rl_one_8x4_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda]);
			}
		if(j<n)
			{
			kernel_dtrsm_nt_rl_one_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], m-i, n-j);
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
			kernel_dtrsm_nt_rl_one_4x4_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda]);
			}
		if(j<n)
			{
			kernel_dtrsm_nt_rl_one_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], m-i, n-j);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif

	// common return if i==m
	return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrsm_nt_rl_one_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrsm_nt_rl_one_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pA[j*sda], &alpha, &pB[j*ps+i*sdb], sdb, &pD[j*ps+i*sdd], sdd, &pA[j*ps+j*sda], m-i, n-j);
		}
	return;
#endif

	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrsm_nt_rl_one_4x4_vs_lib4(j, &pD[i*sdd], &pA[j*sda], &alpha, &pB[j*ps+i*sdb], &pD[j*ps+i*sdd], &pA[j*ps+j*sda], m-i, n-j);
		}

	return;

	}



// dtrsm_runn
void blasfeo_hp_dtrsm_runn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_runn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_runn: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_runu
void blasfeo_hp_dtrsm_runu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_runu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_runu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrsm_right_upper_transposed_notunit
void blasfeo_hp_dtrsm_rutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrsm_rutn: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pD = sD->pA + dj*ps;
	double *dA = sA->dA;

	int ii;

	if(ai==0 & aj==0)
		{
		if(sA->use_dA<n)
			{
			ddiaex_lib(n, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = n;
			}
		}
	else
		{
		ddiaex_lib(n, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}
//	dtrsm_nt_ru_inv_lib(m, n, pA, sda, dA, pB, sdb, pD, sdd);

	if(m<=0 || n<=0)
		return;

	int i, j, idx;

	int rn = n%4;

	double *dummy = NULL;

	i = 0;

#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-11; i+=12)
		{
		j = 0;
		// clean at the end
		if(rn>0)
			{
			idx = n-rn;
			kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(0, dummy, 0, dummy, &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
			j += rn;
			}
		for(; j<n; j+=4)
			{
			idx = n-j-4;
			kernel_dtrsm_nt_ru_inv_12x4_lib4(j, &pD[i*sdd+(idx+4)*ps], sdd, &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx]);
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-7; i+=8)
		{
		j = 0;
		// clean at the end
		if(rn>0)
			{
			idx = n-rn;
			kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(0, dummy, 0, dummy, &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
			j += rn;
			}
		for(; j<n; j+=4)
			{
			idx = n-j-4;
			kernel_dtrsm_nt_ru_inv_8x4_lib4(j, &pD[i*sdd+(idx+4)*ps], sdd, &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx]);
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
		// clean at the end
		if(rn>0)
			{
			idx = n-rn;
			kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(0, dummy, dummy, &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
			j += rn;
			}
		for(; j<n; j+=4)
			{
			idx = n-j-4;
			kernel_dtrsm_nt_ru_inv_4x4_lib4(j, &pD[i*sdd+(idx+4)*ps], &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx]);
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif
	// common return if i==m
	return;

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12:
	j = 0;
	// TODO
	// clean at the end
	if(rn>0)
		{
		idx = n-rn;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(0, dummy, 0, dummy, &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
		j += rn;
		}
	for(; j<n; j+=4)
		{
		idx = n-j-4;
		kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(j, &pD[i*sdd+(idx+4)*ps], sdd, &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx], m-i, 4);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	left_8:
	j = 0;
	// TODO
	// clean at the end
	if(rn>0)
		{
		idx = n-rn;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(0, dummy, 0, dummy, &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
		j += rn;
		}
	for(; j<n; j+=4)
		{
		idx = n-j-4;
		kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(j, &pD[i*sdd+(idx+4)*ps], sdd, &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], sdb, &pD[i*sdd+idx*ps], sdd, &pA[idx*sda+idx*ps], &dA[idx], m-i, 4);
		}
	return;
#endif

	left_4:
	j = 0;
	// TODO
	// clean at the end
	if(rn>0)
		{
		idx = n-rn;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(0, dummy, dummy, &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx], m-i, rn);
		j += rn;
		}
	for(; j<n; j+=4)
		{
		idx = n-j-4;
		kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(j, &pD[i*sdd+(idx+4)*ps], &pA[idx*sda+(idx+4)*ps], &alpha, &pB[i*sdb+idx*ps], &pD[i*sdd+idx*ps], &pA[idx*sda+idx*ps], &dA[idx], m-i, 4);
		}
	return;
	}



// dtrsm_rutu
void blasfeo_hp_dtrsm_rutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsm_rutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dtrsm_rutu: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// dtrmm_right_upper_transposed_notunit (B, i.e. the first matrix, is triangular !!!)
void blasfeo_hp_dtrmm_rutn(int m, int n, double alpha, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	if(ai!=0 | bi!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrmm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dtrmm_rutn: feature not implemented yet: ai=%d, bi=%d, di=%d\n", ai, bi, di);
		exit(1);
#endif
		}

	if(m<=0 || n<=0)
		return;

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pD = sD->pA + dj*ps;

	int i, j;

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
// XXX there is a bug here !!!!!!
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrmm_nt_ru_12x4_lib4(n-j, &alpha, &pA[j*ps+i*sda], sda, &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dtrmm_nt_ru_12x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], sda, &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], sdd, m-i, n-j);
			}
		}
	if(i<m)
		{
		if(m-i<5)
			{
			goto left_4;
			}
		if(m-i<9)
			{
			goto left_8;
			}
		else
			{
			goto left_12;
			}
		}

#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrmm_nt_ru_8x4_lib4(n-j, &alpha, &pA[j*ps+i*sda], sda, &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			kernel_dtrmm_nt_ru_8x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], sda, &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], sdd, m-i, n-j);
			}
		}
	if(i<m)
		{
		if(m-i<5)
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
			kernel_dtrmm_nt_ru_4x4_lib4(n-j, &alpha, &pA[j*ps+i*sda], &pB[j*ps+j*sdb], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			kernel_dtrmm_nt_ru_4x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], m-i, n-j);
			}
		}
	if(i<m)
		{
		goto left_4;
		}
#endif

	// common return
	return;

#if defined(TARGET_X64_INTEL_HASWELL)
	// clean up
	left_12:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrmm_nt_ru_12x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], sda, &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	// clean up
	left_8:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrmm_nt_ru_8x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], sda, &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	return;
#endif

	left_4:
	j = 0;
	for(; j<n; j+=4)
		{
		kernel_dtrmm_nt_ru_4x4_vs_lib4(n-j, &alpha, &pA[j*ps+i*sda], &pB[j*ps+j*sdb], &pD[j*ps+i*sdd], m-i, n-j);
		}
	return;

	}



// dtrmm_right_lower_nottransposed_notunit (B, i.e. the first matrix, is triangular !!!)
void blasfeo_hp_dtrmm_rlnn(int m, int n, double alpha, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sD, int di, int dj)
	{

	const int ps = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	int air = ai & (ps-1);
	int bir = bi & (ps-1);
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pD = sD->pA + dj*ps;

	int offsetB = bir;

	int di0 = di-air;
	int offsetD;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

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

	int ii, jj;

	// algorithm scheme
	if(air!=0)
		{
		goto clear_air;
		}
select_loop:
	if(offsetD==0)
		{
		goto loop_0;
		}
	else
		{
		goto loop_D;
		}
	// should never get here
	return;



clear_air:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x4_gen_lib4(n-jj, &alpha, &pA[jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[jj*ps], sdd, air, air+m, 0, n-jj);
		}
	m -= ps-air;
	pA += ps*sda;
	pD += ps*sdd;
	goto select_loop;



loop_0:
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		jj = 0;
		for(; jj<n-5; jj+=4)
			{
			kernel_dtrmm_nn_rl_12x4_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], sdd); // n-j>=6 !!!!!
			}
		for(; jj<n; jj+=4)
			{
			kernel_dtrmm_nn_rl_12x4_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], sdd, 12, n-jj);
			}
		}
	if(ii<m)
		{
		if(ii<m-8)
			goto left_12;
		else if(ii<m-4)
			goto left_8;
		else
			goto left_4;
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		jj = 0;
		for(; jj<n-5; jj+=4)
			{
			kernel_dtrmm_nn_rl_8x4_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], sdd);
			}
		for(; jj<n; jj+=4)
			{
			kernel_dtrmm_nn_rl_8x4_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], sdd, 8, n-jj);
			}
		}
	if(ii<m)
		{
		if(ii<m-4)
			goto left_8;
		else
			goto left_4;
		}
#elif defined(TARGET_X86_AMD_BARCELONA)
	for(; ii<m-3; ii+=4)
		{
		jj = 0;
		for(; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x2_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps]);
			if(offsetB+2<4)
				kernel_dtrmm_nn_rl_4x2_lib4(n-(jj+2), &alpha, &pA[ii*sda+(jj+2)*ps], offsetB+2, &pB[jj*sdb+(jj+2)*ps], sdb, &pD[ii*sdd+(jj+2)*ps]);
			else
				kernel_dtrmm_nn_rl_4x2_lib4(n-(jj+2), &alpha, &pA[ii*sda+(jj+2)*ps], offsetB+2-ps, &pB[(jj+ps)*sdb+(jj+2)*ps], sdb, &pD[ii*sdd+(jj+2)*ps]);
			}
		for(; jj<n; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x2_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], 4, n-jj);
			if(jj<n-2)
				{
				if(offsetB+2<4)
					kernel_dtrmm_nn_rl_4x2_vs_lib4(n-(jj+2), &alpha, &pA[ii*sda+(jj+2)*ps], offsetB+2, &pB[jj*sdb+(jj+2)*ps], sdb, &pD[ii*sdd+(jj+2)*ps], 4, n-(jj+2));
				else
					kernel_dtrmm_nn_rl_4x2_vs_lib4(n-(jj+2), &alpha, &pA[ii*sda+(jj+2)*ps], offsetB+2-ps, &pB[(jj+ps)*sdb+(jj+2)*ps], sdb, &pD[ii*sdd+(jj+2)*ps], 4, n-(jj+2));
				}
			}
		}
	if(ii<m)
		{
		goto left_4;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		jj = 0;
		for(; jj<n-5; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x4_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps]);
			}
		for(; jj<n; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x4_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], 4, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4;
		}
#endif
	// common return if i==m
	return;



	// main loop C, D not aligned
loop_D:
	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-4; ii+=8)
		{
		jj = 0;
		for(; jj<n; jj+=4)
			{
			kernel_dtrmm_nn_rl_8x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[ii*sdd+jj*ps], sdd, 0, m-ii, 0, n-jj);
			}
		}
	if(ii<m)
		{
		goto left_4_gen;
		}
#else
	for(; ii<m; ii+=4)
		{
		jj = 0;
		for(; jj<n; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[ii*sdd+jj*ps], sdd, 0, m-ii, 0, n-jj);
			}
		}
#endif
	// common return if i==m
	return;



	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_12x4_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], sdd, m-ii, n-jj);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_8:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_8x4_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], sdd, m-ii, n-jj);
		}
	return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_8_gen:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_8x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], sda, offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[ii*sdd+jj*ps], sdd, 0, m-ii, 0, n-jj);
		}
	return;
#endif

#if defined(TARGET_X86_AMD_BARCELONA)
	left_4:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x2_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], m-ii, n-jj);
		if(jj<n-2)
			{
			if(offsetB+2<4)
				kernel_dtrmm_nn_rl_4x2_vs_lib4(n-(jj+2), &alpha, &pA[ii*sda+(jj+2)*ps], offsetB+2, &pB[jj*sdb+(jj+2)*ps], sdb, &pD[ii*sdd+(jj+2)*ps], m-ii, n-(jj+2));
			else
				kernel_dtrmm_nn_rl_4x2_vs_lib4(n-(jj+2), &alpha, &pA[ii*sda+(jj+2)*ps], offsetB+2-ps, &pB[(jj+ps)*sdb+(jj+2)*ps], sdb, &pD[ii*sdd+(jj+2)*ps], m-ii, n-(jj+2));
			}
		}
	return;
#else
	left_4:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x4_vs_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, &pD[ii*sdd+jj*ps], m-ii, n-jj);
		}
	return;
#endif

	left_4_gen:
	jj = 0;
	for(; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x4_gen_lib4(n-jj, &alpha, &pA[ii*sda+jj*ps], offsetB, &pB[jj*sdb+jj*ps], sdb, offsetD, &pD[ii*sdd+jj*ps], sdd, 0, m-ii, 0, n-jj);
		}
	return;
	}



void blasfeo_hp_dsyrk_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
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
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pC = sC->pA + cj*ps;
	double *pD = sD->pA + dj*ps;

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

	struct blasfeo_dmat sAt;
	int sAt_size;
	void *mem;
	char *mem_align;

	double *pU, *pA2;
	int sdu, sda2;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
#endif
	int sdu0 = (k+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	// allocate memory
	if(k>K_MAX_STACK)
		{
		sAt_size = blasfeo_memsize_dmat(12, k);
		mem = malloc(sAt_size+64);
		blasfeo_align_64_byte(mem, (void **) &mem_align);
		blasfeo_create_dmat(12, k, &sAt, (void *) mem_align);
		pU = sAt.pA;
		sdu = sAt.cn;
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
			goto loop_000;
			}
		else
			{
			goto loop_B00;
			}
		}
	else
		{
		if(bir==0)
			{
			goto loop_0CD;
			}
		else
			{
			goto loop_BCD;
			}
		}
	// should never get here
	goto end;



	// main loop aligned
loop_000:
	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
#if defined(TARGET_X64_INTEL_HASWELL)
			kernel_dpacp_nn_12_lib4(k, air, pA+i*sda, sda, pU, sdu);
#else
			kernel_dpacp_nn_4_lib4(k, air, pA+(i+0)*sda, sda, pU+0*sdu);
			kernel_dpacp_nn_4_lib4(k, air, pA+(i+4)*sda, sda, pU+4*sdu);
			kernel_dpacp_nn_4_lib4(k, air, pA+(i+8)*sda, sda, pU+8*sdu);
#endif
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		kernel_dsyrk_nt_l_12x4_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nt_l_8x8_lib4(k, &alpha, pA2+4*sda2, sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd);
#else
		kernel_dsyrk_nt_l_8x4_lib4(k, &alpha, pA2+4*sda2, sda, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd);
		kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, pA2+8*sda2, &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd]);
#endif
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; i<m-7; i+=8)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
			kernel_dpacp_nn_8_lib4(k, air, pA+i*sda, sda, pU, sdu);
#else
			kernel_dpacp_nn_4_lib4(k, air, pA+(i+0)*sda, sda, pU+0*sdu);
			kernel_dpacp_nn_4_lib4(k, air, pA+(i+4)*sda, sda, pU+4*sdu);
#endif
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_dgemm_nt_8x4_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		kernel_dsyrk_nt_l_8x4_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
		kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, pA2+4*sda2, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd]);
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
			kernel_dpacp_nn_4_lib4(k, air, pA+i*sda, sda, pU);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_dgemm_nt_4x4_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
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
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-7; i+=8)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_dpacp_nn_8_vs_lib4(k, air, pA+i*sda, sda, pU, sdu, m-i);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		idxB = 0;
		if(j<i)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, 0, &pC[j*ps+i*sdc]-bir*ps, sdc, 0, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, m-j);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_dgemm_nt_8x4_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
				}
			kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, bir);
			j += bir;
			}
		kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[j*sdb], &beta, 0, &pC[j*ps+i*sdc]-bir*ps, sdc, 0, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+m-j);
		kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[(j+4)*sdb], &beta, 0, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, 0, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, m-j);
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2+4*sda2, &pB[(j+8)*sdb], &beta, 0, &pC[(j+4)*ps+(i+4)*sdc]+(ps-bir)*ps, sdc, 0, &pD[(j+4)*ps+(i+4)*sdd]+(ps-bir)*ps, sdd, ps-bir, m-(i+4), 0, m-(j+4));
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4_g;
			}
		else
			{
			goto left_8_g;
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
			kernel_dpacp_nn_4_lib4(k, air, pA+i*sda, sda, pU);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		idxB = 0;
		if(j<i)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, 0, &pC[j*ps+i*sdc]-bir*ps, sdc, 0, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, 4);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_dgemm_nt_4x4_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
				}
			kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, bir);
			j += bir;
			}
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, 0, &pC[j*ps+i*sdc]-bir*ps, sdc, 0, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, 4);
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[(j+4)*sdb], &beta, 0, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, 0, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, 4);
		}
	if(m>i)
		{
		goto left_4_g;
		}
#endif
	// common return if i==m
	goto end;



	// main loop C, D not aligned
loop_0CD:
	i = 0;
#if 0//defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-8; i+=12)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_dpacp_nn_12_lib4(k, air, pA+i*sda, sda, pU, sdu); // vs ???
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_dgemm_nt_12x4_gen_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
		kernel_dsyrk_nt_l_12x4_gen_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
		kernel_dsyrk_nt_l_8x8_gen_lib4(k, &alpha, pA2+4*sda2, sda, &pB[(j+4)*sdb], sdb, &beta, offsetC, &pC[(j+4)*ps+(i+4)*sdc], sdc, offsetD, &pD[(j+4)*ps+(i+4)*sdd], sdd, 0, m-i-4, 0, m-j-4);
		}
	if(m>i)
		{
		if(m-i<=4)
			{
			goto left_4_g;
			}
		else
			{
			goto left_8_g;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-4; i+=8)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_dpacp_nn_8_lib4(k, air, pA+i*sda, sda, pU, sdu); // vs ???
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
		kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2+4*sda2, &pB[(j+4)*sdb], &beta, offsetC, &pC[(j+4)*ps+(i+4)*sdc], sdc, offsetD, &pD[(j+4)*ps+(i+4)*sdd], sdd, 0, m-i-4, 0, m-j-4);
		}
	if(m>i)
		{
		goto left_4_g;
		}
#else
	for(; i<m; i+=4)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_dpacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		// main loop
		for(; j<i; j+=4)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
		}
#endif
	// common return if i==m
	goto end;



	// main loop aligned
loop_BCD:
	i = 0;
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-4; i+=8)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_dpacp_nn_8_vs_lib4(k, air, pA+i*sda, sda, pU, sdu, m-i);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		idxB = 0;
		if(j<i)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, m-j);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
				}
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, bir); // XXX n1
			j += bir;
			}
		kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+m-j);
		kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[(j+4)*sdb], &beta, offsetC, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, m-j);
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2+4*sda2, &pB[(j+8)*sdb], &beta, offsetC, &pC[(j+4)*ps+(i+4)*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[(j+4)*ps+(i+4)*sdd]+(ps-bir)*ps, sdd, ps-bir, m-(i+4), 0, m-(j+4));
		}
	if(m>i)
		{
		goto left_4_g;
		}
#else
	for(; i<m; i+=4)
		{
		if(air==0)
			{
			pA2 = pA+i*sda;
			sda2 = sda;
			}
		else
			{
			kernel_dpacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
			pA2 = pU;
			sda2 = sdu;
			}
		j = 0;
		idxB = 0;
		if(j<i)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, m-j);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
				}
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, bir); // XXX n1
			j += bir;
			}
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+m-j);
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[(j+4)*sdb], &beta, offsetC, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, m-j);
		}
#endif
	// common return if i==m
	goto end;



	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dpacp_nn_12_lib4(k, air, pA+i*sda, sda, pU, sdu);
#else
		kernel_dpacp_nn_4_lib4(k, air, pA+(i+0)*sda, sda, pU+0*sdu);
		kernel_dpacp_nn_4_lib4(k, air, pA+(i+4)*sda, sda, pU+4*sdu);
		kernel_dpacp_nn_4_lib4(k, air, pA+(i+8)*sda, sda, pU+8*sdu);
#endif
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nt_l_8x8_vs_lib4(k, &alpha, pA2+4*sda2, sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, m-i-4, m-j-4);
#else
	kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, pA2+4*sda2, sda, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, m-i-4, m-j-4);
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2+8*sda2, &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], m-i-8, m-j-8);
#endif
	goto end;
#endif



#if defined(TARGET_X64_INTEL_HASWELL)
	left_8:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_8_lib4(k, air, pA+i*sda, sda, pU, sdu); // vs ???
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i-8; j+=12)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		kernel_dgemm_nt_8x8u_vs_lib4(k, &alpha, pA2, sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+i*sdc], sdc, &pD[(j+4)*ps+i*sdd], sdd, m-i, m-(j+4));
		}
	if(j<i-4)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pA2, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+i*sdc], &pD[(j+4)*ps+i*sdd], m-i, m-(j+4));
		j += 8;
		}
	else if(j<i)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		j += 4;
		}
	kernel_dsyrk_nt_l_8x8_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
	goto end;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_8:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_8_lib4(k, air, pA+i*sda, sda, pU, sdu); // vs ???
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		}
	kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2+4*sda2, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, m-j-4);
	goto end;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_4_lib4(k, air, pA+(i+0)*sda, sda, pU+0*sdu); // vs ???
		kernel_dpacp_nn_4_lib4(k, air, pA+(i+4)*sda, sda, pU+4*sdu); // vs ???
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		}
	kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, pA2, sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2+4*sda2, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, m-j-4);
	goto end;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_8_g:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_8_vs_lib4(k, air, pA+i*sda, sda, pU, sdu, m-i);
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	idxB = 0;
	if(j<i)
		{
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, m-j);
		j += ps-bir;
		idxB += 4;
		// main loop
		for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
		kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, bir); // XXX n1
		j += bir;
		}
	kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+m-j);
	kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, pA2, sda2, &pB[(j+4)*sdb], &beta, offsetC, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, m-j);
	kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2+4*sda2, &pB[(j+8)*sdb], &beta, offsetC, &pC[(j+4)*ps+(i+4)*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[(j+4)*ps+(i+4)*sdd]+(ps-bir)*ps, sdd, ps-bir, m-(i+4), 0, m-(j+4));
	goto end;
#endif



#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_4:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i-8; j+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4(k, &alpha, pA2, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		}
	if(j<i-4)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, pA2, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		j += 8;
		}
	else if(j<i)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		j += 4;
		}
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
	goto end;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	left_4:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i-4; j+=8)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, pA2, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		}
	if(j<i)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		j+=4;
		}
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
	goto end;
#else
	left_4:
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
		pA2 = pU;
		sda2 = sdu;
		}
	j = 0;
	// main loop
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
	goto end;
#endif



	left_4_g:
	j = 0;
	if(air==0)
		{
		pA2 = pA+i*sda;
		sda2 = sda;
		}
	else
		{
		kernel_dpacp_nn_4_vs_lib4(k, air, pA+i*sda, sda, pU, m-i);
		pA2 = pU;
		sda2 = sdu;
		}
	if(bir!=0)
		{
		idxB = 0;
		if(j<i)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, m-j);
			j += ps-bir;
			idxB += 4;
			// main loop
			for(; j<i+(ps-bir)-ps; j+=4, idxB+=4)
				{
				kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
				}
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[idxB*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, bir); // XXX n1
			j += bir;
			}
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc]-bir*ps, sdc, offsetD, &pD[j*ps+i*sdd]-bir*ps, sdd, 0, m-i, bir, bir+m-j);
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[(j+4)*sdb], &beta, offsetC, &pC[j*ps+i*sdc]+(ps-bir)*ps, sdc, offsetD, &pD[j*ps+i*sdd]+(ps-bir)*ps, sdd, ps-bir, m-i, 0, m-j);
		}
	else
		{
		// main loop
		for(; j<i; j+=4)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, pA2, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
		}
	goto end;



end:
	if(k>K_MAX_STACK)
		{
		free(mem);
		}
	return;



#if 0
	// main loop
	i = 0;
	if(offsetC==0 & offsetD==0)
		{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; i<m-11; i+=12)
			{
			j = 0;
			for(; j<i; j+=4)
				{
				kernel_dgemm_nt_12x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
				}
			kernel_dsyrk_nt_l_12x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
#if defined(TARGET_X64_INTEL_HASWELL)
			kernel_dsyrk_nt_l_8x8_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd);
#else
			kernel_dsyrk_nt_l_8x4_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd);
			kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd]);
#endif
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
		for(; i<m-7; i+=8)
			{
			j = 0;
			for(; j<i; j+=4)
				{
				kernel_dgemm_nt_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
				}
			kernel_dsyrk_nt_l_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd]);
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
#elif defined(TARGET_X86_AMD_BARCELONA)
		for(; i<m-3; i+=4)
			{
			j = 0;
			for(; j<i; j+=4)
				{
				kernel_dgemm_nt_4x2_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
				kernel_dgemm_nt_4x2_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd]);
				}
			kernel_dsyrk_nt_l_4x2_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			kernel_dsyrk_nt_l_2x2_lib4(k, &alpha, &pA[i*sda+2], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc+2], &pD[(j+2)*ps+i*sdd+2]);
			}
		if(m>i)
			{
			goto left_4;
			}
#else
		for(; i<m-3; i+=4)
			{
			j = 0;
			for(; j<i; j+=4)
				{
				kernel_dgemm_nt_4x4_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
				}
			kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(m>i)
			{
			goto left_4;
			}
#endif
		}
	else
		{
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for(; i<m-4; i+=8)
			{
			j = 0;
			for(; j<i; j+=4)
				{
				kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
				}
			kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, offsetC, &pC[(j+4)*ps+(i+4)*sdc], sdc, offsetD, &pD[(j+4)*ps+(i+4)*sdd], sdd, 0, m-i-4, 0, m-j-4);
			}
		if(m>i)
			{
			goto left_4_gen;
			}
#else
		for(; i<m; i+=4)
			{
			j = 0;
			for(; j<i; j+=4)
				{
				kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
				}
			kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
			}
#endif
		}

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12:
	j = 0;
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
	kernel_dsyrk_nt_l_8x8_vs_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, m-i-4, m-j-4);
//	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], m-i-8, n-j-8);
	return;
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		}
	kernel_dsyrk_nt_l_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
	kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, m-i-4, m-j-4);
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], m-i-8, m-j-8);
	return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
	left_8:
	j = 0;
	for(; j<i-8; j+=12)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		kernel_dgemm_nt_8x8u_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+i*sdc], sdc, &pD[(j+4)*ps+i*sdd], sdd, m-i, m-(j+4));
		}
	if(j<i-4)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+i*sdc], &pD[(j+4)*ps+i*sdd], m-i, m-(j+4));
		j += 8;
		}
	else if(j<i)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		j += 4;
		}
	kernel_dsyrk_nt_l_8x8_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
//	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, n-j-4);
	return;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
		}
	kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, m-j);
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, m-j-4);
	return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
	left_4:
	j = 0;
	for(; j<i-8; j+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		}
	if(j<i-4)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		j += 8;
		}
	else if(j<i)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		j += 4;
		}
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
	return;
#elif defined(TARGET_X86_AMD_BARCELONA)
	left_4:
	j = 0;
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd], m-i, m-(j+2));
		}
	kernel_dsyrk_nt_l_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
	if(j<m-2)
		kernel_dsyrk_nt_l_2x2_vs_lib4(k, &alpha, &pA[i*sda+2], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc+2], &pD[(j+2)*ps+i*sdd+2], m-(i+2), m-(j+2));
	return;
#else
	left_4:
	j = 0;
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
		}
	kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, m-j);
	return;
#endif

	left_4_gen:
	j = 0;
	for(; j<i; j+=4)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
		}
	kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, m-j);
	return;
#endif

	}



void blasfeo_hp_dsyrk_ln_mn(int m, int n, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	if(ai!=0 | bi!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dsyrk_ln_mn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dsyrk_ln_mn: feature not implemented yet: ai=%d, bi=%d\n", ai, bi);
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
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pC = sC->pA + cj*ps + (ci-(ci&(ps-1)))*sdc;
	double *pD = sD->pA + dj*ps + (di-(di&(ps-1)))*sdd;

	// TODO ai and bi
	int offsetC;
	int offsetD;
	offsetC = ci&(ps-1);
	offsetD = di&(ps-1);



	// algorithm scheme
	if(offsetC==0 & offsetD==0)
		{
		goto loop_000;
		}
	else
		{
		goto loop_0CD;
		}
	// should never get here
	goto end;



	// main loop aligned
loop_000:
	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i & j<n-3; j+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			if(j<i) // dgemm
				{
				kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
				}
			else // dsyrk
				{
				if(j<n-11)
					{
					kernel_dsyrk_nt_l_12x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
#if defined(TARGET_X64_INTEL_HASWELL)
					kernel_dsyrk_nt_l_8x8_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd);
#else
					kernel_dsyrk_nt_l_8x4_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd);
					kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd]);
#endif
					}
				else
					{
					kernel_dsyrk_nt_l_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
					if(j<n-4)
						{
						kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, m-i-4, n-j-4);
						if(j<n-8)
							{
							kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], m-i-8, n-j-8);
							}
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
		else if(m-i<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i & j<n-3; j+=4)
			{
			kernel_dgemm_nt_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
			}
		if(j<n)
			{
			if(j<i) // dgemm
				{
				kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
				}
			else // dsyrk
				{
				if(j<n-7)
					{
					kernel_dsyrk_nt_l_8x4_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd);
					kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd]);
					}
				else
					{
					kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
					if(j<n-4)
						{
						kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, n-j-4);
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
#elif defined(TARGET_X86_AMD_BARCELONA)
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i & j<n-3; j+=4)
			{
			kernel_dgemm_nt_4x2_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			kernel_dgemm_nt_4x2_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd]);
			}
		if(j<n)
			{
			if(j<i) // dgemm
				{
				kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
				if(j<n-2)
					kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd], m-i, n-(j+2));
				}
			else // dsyrk
				{
				if(j<n-3)
					{
					kernel_dsyrk_nt_l_4x2_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
					kernel_dsyrk_nt_l_2x2_lib4(k, &alpha, &pA[i*sda+2], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc+2], &pD[(j+2)*ps+i*sdd+2]);
					}
				else
					{
					kernel_dsyrk_nt_l_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
					if(j<n-2)
						kernel_dsyrk_nt_l_2x2_vs_lib4(k, &alpha, &pA[i*sda+2], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc+2], &pD[(j+2)*ps+i*sdd+2], m-(i+2), n-(j+2));
					}
				}
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#else
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i & j<n-3; j+=4)
			{
			kernel_dgemm_nt_4x4_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
			}
		if(j<n)
			{
			if(j<i) // dgemm
				{
				kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
				}
			else // dsyrk
				{
				if(j<n-3)
					{
					kernel_dsyrk_nt_l_4x4_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd]);
					}
				else
					{
					kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
					}
				}
			}
		}
	if(m>i)
		{
		goto left_4;
		}
#endif
	// common return if i==m
	goto end;



	// main loop C, D not aligned
loop_0CD:
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<i & j<n; j+=4)
			{
			kernel_dgemm_nt_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		if(j<n)
			{
			kernel_dsyrk_nt_l_8x4_gen_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			if(j<n-4)
				{
				kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, offsetC, &pC[(j+4)*ps+(i+4)*sdc], sdc, offsetD, &pD[(j+4)*ps+(i+4)*sdd], sdd, 0, m-i-4, 0, n-j-4);
				}
			}
		}
	if(m>i)
		{
		goto left_4_gen;
		}
#else
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<i & j<n; j+=4)
			{
			kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		if(j<n)
			{
			kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
			}
		}
#endif
	// common return if i==m
	goto end;

	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_dgemm_nt_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_12x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		if(j<n-4)
			{
			kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, m-i-4, n-j-4);
			if(j<n-8)
				{
				kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], m-i-8, n-j-8);
				}
			}
		}
	goto end;
#endif

#if defined(TARGET_X64_INTEL_HASWELL)
	left_8:
	j = 0;
	for(; j<i-8 & j<n-8; j+=12)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		kernel_dgemm_nt_8x8u_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[(j+4)*sdb], sdb, &beta, &pC[(j+4)*ps+i*sdc], sdc, &pD[(j+4)*ps+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<i-4 & j<n-4)
		{
		kernel_dgemm_nt_8x8l_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+i*sdc], &pD[(j+4)*ps+i*sdd], m-i, n-(j+4));
		j += 8;
		}
	if(j<i & j<n)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		j += 4;
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		if(j<n-4)
			{
			kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, n-j-4);
			}
		}
	goto end;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_dgemm_nt_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_8x4_vs_lib4(k, &alpha, &pA[i*sda], sda, &pB[j*sdb], &beta, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, m-i, n-j);
		if(j<n-4)
			{
			kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[(i+4)*sda], &pB[(j+4)*sdb], &beta, &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], m-i-4, n-j-4);
			}
		}
	goto end;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_4:
	j = 0;
	for(; j<i-8 & j<n-8; j+=12)
		{
		kernel_dgemm_nt_4x12_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<i-4 & j<n-4)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		j += 8;
		}
	else if(j<i & j<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		j += 4;
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	goto end;
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	left_4:
	j = 0;
	for(; j<i-4 & j<n-4; j+=8)
		{
		kernel_dgemm_nt_4x8_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], sdb, &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<i & j<n)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		j += 4;
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	goto end;
#elif defined(TARGET_X86_AMD_BARCELONA)
	left_4:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		if(j<n-2)
			kernel_dgemm_nt_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc], &pD[(j+2)*ps+i*sdd], m-i, n-(j+2));
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_4x2_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		if(j<n-2)
			kernel_dsyrk_nt_l_2x2_vs_lib4(k, &alpha, &pA[i*sda+2], &pB[j*sdb+2], &beta, &pC[(j+2)*ps+i*sdc+2], &pD[(j+2)*ps+i*sdd+2], m-(i+2), n-(j+2));
		}
	goto end;
#else
	left_4:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_dgemm_nt_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_4x4_vs_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], m-i, n-j);
		}
	goto end;
#endif

	left_4_gen:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_dgemm_nt_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	if(j<n)
		{
		kernel_dsyrk_nt_l_4x4_gen_lib4(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, offsetC, &pC[j*ps+i*sdc], sdc, offsetD, &pD[j*ps+i*sdd], sdd, 0, m-i, 0, n-j);
		}
	goto end;

end:
	return;

	}



void blasfeo_hp_dsyrk_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dsyrk_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dsyrk_lt: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



void blasfeo_hp_dsyrk_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dsyrk_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dsyrk_un: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



void blasfeo_hp_dsyrk_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
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
	int cir = ci & (ps-1);
	int dir = di & (ps-1);

	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;
	double *pC = sC->pA + cj*ps + (ci-cir)*sdc;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;

	int offsetA = air;
	int offsetB = bir;
	int offsetC = cir;
	int offsetD = dir;

	void *mem;
	double *pU, *pA2;
	int sdu, sda2;

// TODO visual studio alignment
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
#endif
	int sdu0 = (k+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	// allocate memory
	if(k>K_MAX_STACK)
		{
		sdu = (k+ps-1)/ps*ps;
		mem = malloc(12*sdu*sizeof(double)+63);
		blasfeo_align_64_byte(mem, (void **) &pU);
		}
	else
		{
		pU = pU0;
		sdu = sdu0;
		}
	

	int i, j, n1;

	int idxB;


	if(ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dsyrk_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dsyrk_ut: feature not implemented yet: ci!=0 | di!=0\n");
		exit(1);
#endif
		}

	// algorithm scheme
	goto loop_00;
#if 0
	if(offsetC==0 & offsetD==0)
		{
//	printf("\n00\n");
		goto loop_00;
		}
	else
		{
//	printf("\nCD\n");
		goto loop_CD;
		}
#endif
	// should never get here
	goto end;



	// main loop aligned
loop_00:
	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-11; i+=12)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+i*ps, sda, pU);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(i+4)*ps, sda, pU+4*sdu);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(i+8)*ps, sda, pU+8*sdu);
#if defined(TARGET_X64_INTEL_HASWELL)
		kernel_dsyrk_nn_u_8x8_lib4(k, &alpha, pU, sdu, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, sdc, pD+i*sdd+i*ps, sdd);
#else
		kernel_dsyrk_nn_u_4x4_lib4(k, &alpha, pU, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, pD+i*sdd+i*ps);
		kernel_dsyrk_nn_u_8x4_lib4(k, &alpha, pU, sdu, offsetB, pB+(i+4)*ps, sdb, &beta, pC+i*sdc+(i+4)*ps, sdc, pD+i*sdd+(i+4)*ps, sdd);
#endif
		kernel_dsyrk_nn_u_12x4_lib4(k, &alpha, pU, sdu, offsetB, pB+(i+8)*ps, sdb, &beta, pC+i*sdc+(i+8)*ps, sdc, pD+i*sdd+(i+8)*ps, sdd);
		for(j=i+12; j<m-3; j+=4)
			{
			kernel_dgemm_nn_12x4_lib4(k, &alpha, pU, sdu, offsetB, pB+j*ps, sdb, &beta, pC+i*sdc+j*ps, sdc, pD+i*sdd+j*ps, sdc);
			}
		if(j<m)
			{
			kernel_dgemm_nn_12x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+j*ps, sdb, &beta, pC+i*sdc+j*ps, sdc, pD+i*sdd+j*ps, sdc, m-i, m-j);
			}
		}
	if(i<m)
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
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	for(; i<m-7; i+=8)
		{
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+i*ps, sda, pU);
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+(i+4)*ps, sda, pU+4*sdu);
		kernel_dsyrk_nn_u_4x4_lib4(k, &alpha, pU, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, pD+i*sdd+i*ps);
		kernel_dsyrk_nn_u_8x4_lib4(k, &alpha, pU, sdu, offsetB, pB+(i+4)*ps, sdb, &beta, pC+i*sdc+(i+4)*ps, sdc, pD+i*sdd+(i+4)*ps, sdd);
		for(j=i+8; j<m-3; j+=4)
			{
			kernel_dgemm_nn_8x4_lib4(k, &alpha, pU, sdu, offsetB, pB+j*ps, sdb, &beta, pC+i*sdc+j*ps, sdc, pD+i*sdd+j*ps, sdc);
			}
		if(j<m)
			{
			kernel_dgemm_nn_8x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+j*ps, sdb, &beta, pC+i*sdc+j*ps, sdc, pD+i*sdd+j*ps, sdc, m-i, m-j);
			}
		}
	if(i<m)
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
		kernel_dpacp_tn_4_lib4(k, offsetA, pA+i*ps, sda, pU);
		kernel_dsyrk_nn_u_4x4_lib4(k, &alpha, pU, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, pD+i*sdd+i*ps);
		for(j=i+4; j<m-3; j+=4)
			{
			kernel_dgemm_nn_4x4_lib4(k, &alpha, pU, offsetB, pB+j*ps, sdb, &beta, pC+i*sdc+j*ps, pD+i*sdd+j*ps);
			}
		if(j<m)
			{
			kernel_dgemm_nn_4x4_vs_lib4(k, &alpha, pU, offsetB, pB+j*ps, sdb, &beta, pC+i*sdc+j*ps, pD+i*sdd+j*ps, m-i, m-j);
			}
		}
	if(i<m)
		{
		goto left_4;
		}
#endif
	goto end;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_12:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+i*ps, sda, pU);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(i+4)*ps, sda, pU+4*sdu);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(i+8)*ps, sda, pU+8*sdu);
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dsyrk_nn_u_8x8_lib4(k, &alpha, pU, sdu, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, sdc, pD+i*sdd+i*ps, sdd);
#else
	kernel_dsyrk_nn_u_4x4_lib4(k, &alpha, pU, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, pD+i*sdd+i*ps);
	kernel_dsyrk_nn_u_8x4_lib4(k, &alpha, pU, sdu, offsetB, pB+(i+4)*ps, sdb, &beta, pC+i*sdc+(i+4)*ps, sdc, pD+i*sdd+(i+4)*ps, sdd);
#endif
	kernel_dsyrk_nn_u_12x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+(i+8)*ps, sdb, &beta, pC+i*sdc+(i+8)*ps, sdc, pD+i*sdd+(i+8)*ps, sdd, m-i, m-i-8);
	goto end;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
left_8:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+i*ps, sda, pU);
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+(i+4)*ps, sda, pU+4*sdu);
	kernel_dsyrk_nn_u_4x4_lib4(k, &alpha, pU, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, pD+i*sdd+i*ps);
	kernel_dsyrk_nn_u_8x4_vs_lib4(k, &alpha, pU, sdu, offsetB, pB+(i+4)*ps, sdb, &beta, pC+i*sdc+(i+4)*ps, sdc, pD+i*sdd+(i+4)*ps, sdd, m-i, m-i-4);
	goto end;
#endif

left_4:
	kernel_dpacp_tn_4_lib4(k, offsetA, pA+i*ps, sda, pU);
	kernel_dsyrk_nn_u_4x4_vs_lib4(k, &alpha, pU, offsetB, pB+i*ps, sdb, &beta, pC+i*sdc+i*ps, pD+i*sdd+i*ps, m-i, m-i);
	goto end;

end:
	if(k>K_MAX_STACK)
		{
		free(mem);
		}
	return;

	}



void blasfeo_hp_dsyr2k_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	double d_1 = 1.0;
	blasfeo_hp_dsyrk_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_dsyrk_ln(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



void blasfeo_hp_dsyr2k_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	double d_1 = 1.0;
	blasfeo_hp_dsyrk_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_dsyrk_lt(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



void blasfeo_hp_dsyr2k_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	double d_1 = 1.0;
	blasfeo_hp_dsyrk_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_dsyrk_un(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



void blasfeo_hp_dsyr2k_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	double d_1 = 1.0;
	blasfeo_hp_dsyrk_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	blasfeo_hp_dsyrk_ut(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



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



void blasfeo_dtrsm_llnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_llnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_llnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_llnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_lltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_lltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_lunn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lunn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_lunu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lunu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_lutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_lutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_lutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_rlnn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rlnn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_rlnu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rlnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_rltn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_rltu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_runn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_runn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_runu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_runu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_rutn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrsm_rutu(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrsm_rutu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}



void blasfeo_dtrmm_rutn(int m, int n, double alpha, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrmm_rutn(m, n, alpha, sB, bi, bj, sA, ai, aj, sD, di, dj);
	}



void blasfeo_dtrmm_rlnn(int m, int n, double alpha, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dtrmm_rlnn(m, n, alpha, sB, bi, bj, sA, ai, aj, sD, di, dj);
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



void blasfeo_dsyr2k_ln(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_ln(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyr2k_lt(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_lt(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyr2k_un(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_un(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyr2k_ut(int m, int k, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, double beta, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyr2k_ut(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



#endif



