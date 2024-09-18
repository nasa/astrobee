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
#include <math.h>

#include <blasfeo_common.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_s_kernel.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_s_blasfeo_ref_api.h>
#endif



// dpotrf
void blasfeo_hp_spotrf_l(int m, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0)
		return;

	if(ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_spotrf_l(m, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_spotrf_l: feature not implemented yet: ci=%d, di=%d\n", ci, di);
		exit(1);
#endif
		}

	const int ps = 4;

	float d_1 = 1.0;

	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;
	float *dD = sD->dA;
	if(di==0 && dj==0) // XXX what to do if di and dj are not zero
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
#if 1
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_16x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_16x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_12x4_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4]);
		kernel_spotrf_nt_l_8x4_lib4(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(j+8)*sdc], sdc, &pD[(j+8)*ps+(j+8)*sdd], sdd, &dD[j+8]);
		kernel_spotrf_nt_l_4x4_lib4(j+12, &pD[(i+12)*sdd], &pD[(j+12)*sdd], &pC[(j+12)*ps+(j+12)*sdc], &pD[(j+12)*ps+(j+12)*sdd], &dD[j+12]);
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
#elif 0
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_8x4_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4]);
		kernel_spotrf_nt_l_4x4_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(j+8)*sdc], &pD[(j+8)*ps+(j+8)*sdd], &dD[j+8]);
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
#else
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_4x4_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], &pD[(j+4)*ps+(j+4)*sdd], &dD[j+4]);
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
#endif
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_8x4_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4]);
		kernel_spotrf_nt_l_4x4_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(j+8)*sdc], &pD[(j+8)*ps+(j+8)*sdd], &dD[j+8]);
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
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_4x4_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], &pD[(j+4)*ps+(j+4)*sdd], &dD[j+4]);
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
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_4x4_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
		}
	if(m>i)
		{
		goto left_4;
		}
#endif

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_16:
	j = 0;
	for(; j<i; j+=4)
		{
		kernel_strsm_nt_rl_inv_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
	kernel_spotrf_nt_l_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
	kernel_spotrf_nt_l_12x4_vs_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4], m-i-4, m-j-4);
	kernel_spotrf_nt_l_8x4_vs_lib4(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(j+8)*sdc], sdc, &pD[(j+8)*ps+(j+8)*sdd], sdd, &dD[j+8], m-i-8, m-j-8);
	kernel_spotrf_nt_l_4x4_vs_lib4(j+12, &pD[(i+12)*sdd], &pD[(j+12)*sdd], &pC[(j+12)*ps+(i+12)*sdc], &pD[(j+12)*ps+(i+12)*sdd], &dD[j+12], m-i-12, m-j-12);
	return;
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	if(m-i==12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_8x4_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4]);
		kernel_spotrf_nt_l_4x4_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8]);
		}
	else
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			}
		kernel_spotrf_nt_l_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		kernel_spotrf_nt_l_8x4_vs_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4], m-i-4, m-j-4);
		kernel_spotrf_nt_l_4x4_vs_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, m-j-8);
		}
	return;
#endif
#if defined(TARGET_ARMV7A_ARM_CORTEX_A15)
	left_12:
	if(m-i==12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_8x4_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4]);
		kernel_spotrf_nt_l_4x4_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8]);
		}
	else
		{
		j = 0;
		for(; j<i; j+=4)
			{
	//		kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[(i+4)*sdd], &pD[j*sdd], &d_1, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], &pD[j*ps+j*sdd], &dD[j], m-i-4, m-j);
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[(i+8)*sdd], &pD[j*sdd], &d_1, &pC[j*ps+(i+8)*sdc], &pD[j*ps+(i+8)*sdd], &pD[j*ps+j*sdd], &dD[j], m-i-8, m-j);
			}
	//	kernel_spotrf_nt_l_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		kernel_spotrf_nt_l_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[(i+4)*sdd], &pD[j*sdd], &d_1, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], &pD[j*ps+j*sdd], &dD[j], m-i-4, m-j);
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[(i+8)*sdd], &pD[j*sdd], &d_1, &pC[j*ps+(i+8)*sdc], &pD[j*ps+(i+8)*sdd], &pD[j*ps+j*sdd], &dD[j], m-i-8, m-j);
//		kernel_spotrf_nt_l_8x4_vs_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4], m-i-4, m-j-4);
		kernel_spotrf_nt_l_4x4_vs_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], &pD[(j+4)*ps+(j+4)*sdd], &dD[j+4], m-i-4, m-j-4);
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], &d_1, &pC[(j+4)*ps+(i+8)*sdc], &pD[(j+4)*ps+(i+8)*sdd], &pD[(j+4)*ps+(j+4)*sdd], &dD[j+4], m-i-8, m-j-4);
		kernel_spotrf_nt_l_4x4_vs_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, m-j-8);
		}
	return;
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	if(m-i==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_4x4_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], &dD[j+4]);
		}
	else
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			}
		kernel_spotrf_nt_l_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		kernel_spotrf_nt_l_4x4_vs_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], &dD[j+4], m-i-4, m-j-4);
		}
	return;
#endif
#if defined(TARGET_ARMV7A_ARM_CORTEX_A15) | defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	left_8:
	if(m-i==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_spotrf_nt_l_4x4_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], &dD[j+4]);
		}
	else
		{
		j = 0;
		for(; j<i; j+=4)
			{
	//		kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[(i+4)*sdd], &pD[j*sdd], &d_1, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], &pD[j*ps+j*sdd], &dD[j], m-i-4, m-j);
			}
	//	kernel_spotrf_nt_l_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		kernel_spotrf_nt_l_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[(i+4)*sdd], &pD[j*sdd], &d_1, &pC[j*ps+(i+4)*sdc], &pD[j*ps+(i+4)*sdd], &pD[j*ps+j*sdd], &dD[j], m-i-4, m-j);
		kernel_spotrf_nt_l_4x4_vs_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], &dD[j+4], m-i-4, m-j-4);
		}
	return;
#endif

	left_4:
	if(m-i==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_spotrf_nt_l_4x4_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
		}
	else
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			}
		kernel_spotrf_nt_l_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
	return;

	}



// dpotrf
void blasfeo_hp_spotrf_l_mn(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	if(ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_spotrf_l_mn(m, n, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_spotrf_l: feature not implemented yet: ci=%d, di=%d\n", ci, di);
		exit(1);
#endif
		}

	const int ps = 4;

	float d_1 = 1.0;

	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;
	float *dD = sD->dA;
	if(di==0 && dj==0) // XXX what to do if di and dj are not zero
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i & j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_16x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dtrsm
				{
				kernel_strsm_nt_rl_inv_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dpotrf
				{
				if(j<n-15)
					{
					kernel_spotrf_nt_l_16x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
					kernel_spotrf_nt_l_12x4_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4]);
//					kernel_spotrf_nt_l_8x8_lib4(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], sdd, &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
					kernel_spotrf_nt_l_8x4_lib4(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
					kernel_spotrf_nt_l_4x4_lib4(j+12, &pD[(i+12)*sdd], &pD[(j+12)*sdd], &pC[(j+12)*ps+(i+12)*sdc], &pD[(j+12)*ps+(i+12)*sdd], &dD[j+12]);
					}
				else
					{
					kernel_spotrf_nt_l_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
					if(j<n-4)
						{
						kernel_spotrf_nt_l_12x4_vs_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(j+8)*sdc], sdc, &pD[(j+8)*ps+(j+8)*sdd], sdd, &dD[j+4], m-i-4, n-j-4);
						if(j<n-8)
							{
							kernel_spotrf_nt_l_8x4_vs_lib4(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, n-j-8);
							if(j<n-12)
								{
								kernel_spotrf_nt_l_4x4_vs_lib4(j+12, &pD[(i+12)*sdd], &pD[(j+12)*sdd], &pC[(j+12)*ps+(i+12)*sdc], &pD[(j+12)*ps+(i+12)*sdd], &dD[j+12], m-i-12, n-j-12);
								}
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
		for(; j<i & j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dtrsm
				{
				kernel_strsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dpotrf
				{
				if(j<n-11)
					{
					kernel_spotrf_nt_l_12x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
//					kernel_spotrf_nt_l_8x8_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, &dD[j+4]);
					kernel_spotrf_nt_l_8x4_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, &dD[j+4]);
					kernel_spotrf_nt_l_4x4_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8]);
					}
				else
					{
					kernel_spotrf_nt_l_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
					if(j<n-4)
						{
						kernel_spotrf_nt_l_8x4_vs_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, &dD[j+4], m-i-4, n-j-4);
						if(j<n-8)
							{
							kernel_spotrf_nt_l_4x4_vs_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, n-j-8);
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
#elif 0 //defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i & j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dtrsm
				{
				kernel_strsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dpotrf
				{
				if(j<n-7)
//				if(0)
					{
					kernel_spotrf_nt_l_8x4_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
					kernel_spotrf_nt_l_4x4_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], &dD[j+4]);
					}
				else
					{
					kernel_spotrf_nt_l_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
					if(j<n-4)
						{
						kernel_spotrf_nt_l_4x4_vs_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], &dD[j+4], m-i-4, n-j-4);
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
		for(; j<i && j<n-3; j+=4)
			{
			kernel_strsm_nt_rl_inv_4x4_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dtrsm
				{
				kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dpotrf
				{
				if(j<n-3)
					{
					kernel_spotrf_nt_l_4x4_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
					}
				else
					{
					kernel_spotrf_nt_l_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
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
	return;

	// clean up loops definitions

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_16:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_strsm_nt_rl_inv_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_spotrf_nt_l_16x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		if(j<n-4)
			{
			kernel_spotrf_nt_l_12x4_vs_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(j+4)*sdc], sdc, &pD[(j+4)*ps+(j+4)*sdd], sdd, &dD[j+4], m-i-4, n-j-4);
			if(j<n-8)
				{
				kernel_spotrf_nt_l_8x4_vs_lib4(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, n-j-8);
				if(j<n-12)
					{
					kernel_spotrf_nt_l_4x4_vs_lib4(j+12, &pD[(i+12)*sdd], &pD[(j+12)*sdd], &pC[(j+12)*ps+(i+12)*sdc], &pD[(j+12)*ps+(i+12)*sdd], &dD[j+12], m-i-12, n-j-12);
					}
				}
			}
		}
	return;
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_12:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_strsm_nt_rl_inv_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_spotrf_nt_l_12x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		if(j<n-4)
			{
			kernel_spotrf_nt_l_8x4_vs_lib4(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], sdc, &pD[(j+4)*ps+(i+4)*sdd], sdd, &dD[j+4], m-i-4, n-j-4);
			if(j<n-8)
				{
				kernel_spotrf_nt_l_4x4_vs_lib4(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, n-j-8);
				}
			}
		}
	return;
#endif

#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	left_8:
	j = 0;
	for(; j<i & j<n; j+=4)
		{
		kernel_strsm_nt_rl_inv_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_spotrf_nt_l_8x4_vs_lib4(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		if(j<n-4)
			{
			kernel_spotrf_nt_l_4x4_vs_lib4(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], &pC[(j+4)*ps+(i+4)*sdc], &pD[(j+4)*ps+(i+4)*sdd], &dD[j+4], m-i-4, n-j-4);
			}
		}
	return;
#endif

	left_4:
	j = 0;
	for(; j<i && j<n-3; j+=4)
		{
		kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		if(j<i) // dtrsm
			{
			kernel_strsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &d_1, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
			}
		else // dpotrf
			{
			kernel_spotrf_nt_l_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
			}
		}
	return;

	}



// spotrf
void blasfeo_hp_spotrf_u(int m, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0)
		return;

#if defined(BLASFEO_REF_API)
	blasfeo_ref_spotrf_u(m, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_spotrf_u: feature not implemented yet\n");
	exit(1);
#endif
	
	}



// dsyrk dpotrf
void blasfeo_hp_ssyrk_spotrf_ln(int m, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	if(ai!=0 | bi!=0 | ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_ssyrk_spotrf_ln(m, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_ssyrk_spotrf_ln: feature not implemented yet: ai=%d, bi=%d, ci=%d, di=%d\n", ai, bi, ci, di);
		exit(1);
#endif
		}
	const int ps = 4;
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;
	float *inv_diag_D = sD->dA; // XXX what to do if di and dj are not zero

	if(m<=0 || k<=0)
		return;

	if(di==0 && dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;

	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i && j<k-3; j+=4)
			{
			kernel_sgemm_strsm_nt_rl_inv_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j]);
			}
		if(j<k)
			{
			if(j<i) // dgemm
				{
				kernel_sgemm_strsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
				}
			else // dsyrk
				{
				if(j<k-3)
					{
					kernel_ssyrk_spotrf_nt_l_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &inv_diag_D[j]);
					}
				else
					{
					kernel_ssyrk_spotrf_nt_l_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
					}
				}
			}
		}
	if(m>i)
		{
		goto left_4;
		}

	// common return if i==m
	return;

	// clean up loops definitions

	left_4:
	j = 0;
	for(; j<i && j<k-3; j+=4)
		{
		kernel_sgemm_strsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
		}
	if(j<k)
		{
		if(j<i) // dgemm
			{
			kernel_sgemm_strsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
			}
		else // dsyrk
			{
			kernel_ssyrk_spotrf_nt_l_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
			}
		}

	return;
	}



void blasfeo_hp_ssyrk_spotrf_ln_mn(int m, int n, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	if(ai!=0 | bi!=0 | ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_ssyrk_spotrf_ln_mn(m, n, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_ssyrk_spotrf_ln: feature not implemented yet: ai=%d, bi=%d, ci=%d, di=%d\n", ai, bi, ci, di);
		exit(1);
#endif
		}
	const int ps = 4;
	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*ps;
	float *pB = sB->pA + bj*ps;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;
	float *inv_diag_D = sD->dA; // XXX what to do if di and dj are not zero

	if(m<=0 || k<=0)
		return;

	if(di==0 && dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;

	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i && j<k-3; j+=4)
			{
			kernel_sgemm_strsm_nt_rl_inv_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j]);
			}
		if(j<k)
			{
			if(j<i) // dgemm
				{
				kernel_sgemm_strsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
				}
			else // dsyrk
				{
				if(j<k-3)
					{
					kernel_ssyrk_spotrf_nt_l_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &inv_diag_D[j]);
					}
				else
					{
					kernel_ssyrk_spotrf_nt_l_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
					}
				}
			}
		}
	if(m>i)
		{
		goto left_4;
		}

	// common return if i==m
	return;

	// clean up loops definitions

	left_4:
	j = 0;
	for(; j<i && j<k-3; j+=4)
		{
		kernel_sgemm_strsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
		}
	if(j<k)
		{
		if(j<i) // dgemm
			{
			kernel_sgemm_strsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
			}
		else // dsyrk
			{
			kernel_ssyrk_spotrf_nt_l_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &inv_diag_D[j], m-i, k-j);
			}
		}

	return;
	}



// dgetrf no pivoting
void blasfeo_hp_sgetrf_np(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	if(ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_sgetrf_np(m, n, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_sgetf_np: feature not implemented yet: ci=%d, di=%d\n", ci, di);
		exit(1);
#endif
		}
	const int ps = 4;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;
	float *inv_diag_D = sD->dA; // XXX what to do if di and dj are not zero

	if(m<=0 || n<=0)
		return;
	
	if(di==0 && dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int ii, jj, ie;

	float d_1 = 1.0;

	// main loop
	ii = 0;
	for( ; ii<m-3; ii+=4)
		{
		jj = 0;
		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie-3; jj+=4)
			{
			kernel_strsm_nn_ru_inv_4x4_lib4(jj, &pD[ii*sdd], &pD[jj*ps], sdd, &d_1, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &pD[jj*ps+jj*sdd], &inv_diag_D[jj]);
			}
		if(jj<ie)
			{
			kernel_strsm_nn_ru_inv_4x4_vs_lib4(jj, &pD[ii*sdd], &pD[jj*ps], sdd, &d_1, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &pD[jj*ps+jj*sdd], &inv_diag_D[jj], m-ii, ie-jj);
			jj+=4;
			}
		// factorize
		if(jj<n-3)
			{
			kernel_sgetrf_nn_4x4_lib4(jj, &pD[ii*sdd], &pD[jj*ps], sdd, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &inv_diag_D[jj]);
			jj+=4;
			}
		else if(jj<n)
			{
			kernel_sgetrf_nn_4x4_vs_lib4(jj, &pD[ii*sdd], &pD[jj*ps], sdd, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &inv_diag_D[jj], m-ii, n-jj);
			jj+=4;
			}
		// solve upper 
		for( ; jj<n-3; jj+=4)
			{
			kernel_strsm_nn_ll_one_4x4_lib4(ii, &pD[ii*sdd], &pD[jj*ps], sdd, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &pD[ii*ps+ii*sdd]);
			}
		if(jj<n)
			{
			kernel_strsm_nn_ll_one_4x4_vs_lib4(ii, &pD[ii*sdd], &pD[jj*ps], sdd, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &pD[ii*ps+ii*sdd], m-ii, n-jj);
			}
		}
	if(m>ii)
		{
		goto left_4;
		}

	// common return if i==m
	return;

	left_4:
	jj = 0;
	// solve lower
	ie = n<ii ? n : ii; // ie is multiple of 4
	for( ; jj<ie; jj+=4)
		{
		kernel_strsm_nn_ru_inv_4x4_vs_lib4(jj, &pD[ii*sdd], &pD[jj*ps], sdd, &d_1, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &pD[jj*ps+jj*sdd], &inv_diag_D[jj], m-ii, ie-jj);
		}
	// factorize
	if(jj<n)
		{
		kernel_sgetrf_nn_4x4_vs_lib4(jj, &pD[ii*sdd], &pD[jj*ps], sdd, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &inv_diag_D[jj], m-ii, n-jj);
		jj+=4;
		}
	// solve upper 
	for( ; jj<n; jj+=4)
		{
		kernel_strsm_nn_ll_one_4x4_vs_lib4(ii, &pD[ii*sdd], &pD[jj*ps], sdd, &pC[jj*ps+ii*sdc], &pD[jj*ps+ii*sdd], &pD[ii*ps+ii*sdd], m-ii, n-jj);
		}
	return;

	}




// dgetrf row pivoting
void blasfeo_hp_sgetrf_rp(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, int *ipiv)
	{
	if(ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_sgetrf_rp(m, n, sC, ci, cj, sD, di, dj, ipiv);
		return;
#else
		printf("\nblasfeo_sgetrf_rp: feature not implemented yet: ci=%d, di=%d\n", ci, di);
		exit(1);
#endif
		}
	const int ps = 4;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pC = sC->pA + cj*ps;
	float *pD = sD->pA + dj*ps;
	float *inv_diag_D = sD->dA; // XXX what to do if di and dj are not zero

	// needs to perform row-excanges on the yet-to-be-factorized matrix too
	if(pC!=pD)
		blasfeo_sgecp(m, n, sC, ci, cj, sD, di, dj);

	if(m<=0)
		return;
	
	if(di==0 && dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int ii, jj, i0, i1, j0, ll, p;

	float d1 = 1.0;
	float dm1 = -1.0;

//	// needs to perform row-excanges on the yet-to-be-factorized matrix too
//	if(pC!=pD)
//		sgecp_lib(m, n, 1.0, 0, pC, sdc, 0, pD, sdd);

	// minimum matrix size
	p = n<m ? n : m; // XXX

	// main loop
	// 4 columns at a time
	jj = 0;
	for(; jj<p-3; jj+=4) // XXX
		{
		// pivot & factorize & solve lower
		ii = jj;
		i0 = ii;
		for( ; ii<m-3; ii+=4)
			{
			kernel_sgemm_nn_4x4_lib4(jj, &dm1, &pD[ii*sdd], 0, &pD[jj*ps], sdd, &d1, &pD[jj*ps+ii*sdd], &pD[jj*ps+ii*sdd]);
			}
		if(m-ii>0)
			{
			kernel_sgemm_nn_4x4_vs_lib4(jj, &dm1, &pD[ii*sdd], 0, &pD[jj*ps], sdd, &d1, &pD[jj*ps+ii*sdd], &pD[jj*ps+ii*sdd], m-ii, 4);
			}
		kernel_sgetrf_pivot_4_lib4(m-i0, &pD[jj*ps+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
		ipiv[i0+0] += i0;
		if(ipiv[i0+0]!=i0+0)
			{
			srowsw_lib(jj, pD+(i0+0)/ps*ps*sdd+(i0+0)%ps, pD+(ipiv[i0+0])/ps*ps*sdd+(ipiv[i0+0])%ps);
			srowsw_lib(n-jj-4, pD+(i0+0)/ps*ps*sdd+(i0+0)%ps+(jj+4)*ps, pD+(ipiv[i0+0])/ps*ps*sdd+(ipiv[i0+0])%ps+(jj+4)*ps);
			}
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			srowsw_lib(jj, pD+(i0+1)/ps*ps*sdd+(i0+1)%ps, pD+(ipiv[i0+1])/ps*ps*sdd+(ipiv[i0+1])%ps);
			srowsw_lib(n-jj-4, pD+(i0+1)/ps*ps*sdd+(i0+1)%ps+(jj+4)*ps, pD+(ipiv[i0+1])/ps*ps*sdd+(ipiv[i0+1])%ps+(jj+4)*ps);
			}
		ipiv[i0+2] += i0;
		if(ipiv[i0+2]!=i0+2)
			{
			srowsw_lib(jj, pD+(i0+2)/ps*ps*sdd+(i0+2)%ps, pD+(ipiv[i0+2])/ps*ps*sdd+(ipiv[i0+2])%ps);
			srowsw_lib(n-jj-4, pD+(i0+2)/ps*ps*sdd+(i0+2)%ps+(jj+4)*ps, pD+(ipiv[i0+2])/ps*ps*sdd+(ipiv[i0+2])%ps+(jj+4)*ps);
			}
		ipiv[i0+3] += i0;
		if(ipiv[i0+3]!=i0+3)
			{
			srowsw_lib(jj, pD+(i0+3)/ps*ps*sdd+(i0+3)%ps, pD+(ipiv[i0+3])/ps*ps*sdd+(ipiv[i0+3])%ps);
			srowsw_lib(n-jj-4, pD+(i0+3)/ps*ps*sdd+(i0+3)%ps+(jj+4)*ps, pD+(ipiv[i0+3])/ps*ps*sdd+(ipiv[i0+3])%ps+(jj+4)*ps);
			}

		// solve upper
		ll = jj+4;
		for( ; ll<n-3; ll+=4)
			{
			kernel_strsm_nn_ll_one_4x4_lib4(i0, &pD[i0*sdd], &pD[ll*ps], sdd, &pD[ll*ps+i0*sdd], &pD[ll*ps+i0*sdd], &pD[i0*ps+i0*sdd]);
			}
		if(n-ll>0)
			{
			kernel_strsm_nn_ll_one_4x4_vs_lib4(i0, &pD[i0*sdd], &pD[ll*ps], sdd, &pD[ll*ps+i0*sdd], &pD[ll*ps+i0*sdd], &pD[i0*ps+i0*sdd], 4, n-ll);
			}
		}
	if(m>=n)
		{
		if(n-jj>0)
			{
			goto left_n_4;
			}
		}
	else
		{
		if(m-jj>0)
			{
			goto left_m_4;
			}
		}

	// common return if jj==n
	return;

	// clean up

	left_n_4:
	// 1-4 columns at a time
	// pivot & factorize & solve lower
	ii = jj;
	i0 = ii;
	for( ; ii<m; ii+=4)
		{
		kernel_sgemm_nn_4x4_vs_lib4(jj, &dm1, &pD[ii*sdd], 0, &pD[jj*ps], sdd, &d1, &pD[jj*ps+ii*sdd], &pD[jj*ps+ii*sdd], m-ii, n-jj);
		}
	kernel_sgetrf_pivot_4_vs_lib4(m-i0, n-jj, &pD[jj*ps+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		srowsw_lib(jj, pD+(i0+0)/ps*ps*sdd+(i0+0)%ps, pD+(ipiv[i0+0])/ps*ps*sdd+(ipiv[i0+0])%ps);
		srowsw_lib(n-jj-4, pD+(i0+0)/ps*ps*sdd+(i0+0)%ps+(jj+4)*ps, pD+(ipiv[i0+0])/ps*ps*sdd+(ipiv[i0+0])%ps+(jj+4)*ps);
		}
	if(n-jj>1)
		{
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			srowsw_lib(jj, pD+(i0+1)/ps*ps*sdd+(i0+1)%ps, pD+(ipiv[i0+1])/ps*ps*sdd+(ipiv[i0+1])%ps);
			srowsw_lib(n-jj-4, pD+(i0+1)/ps*ps*sdd+(i0+1)%ps+(jj+4)*ps, pD+(ipiv[i0+1])/ps*ps*sdd+(ipiv[i0+1])%ps+(jj+4)*ps);
			}
		if(n-jj>2)
			{
			ipiv[i0+2] += i0;
			if(ipiv[i0+2]!=i0+2)
				{
				srowsw_lib(jj, pD+(i0+2)/ps*ps*sdd+(i0+2)%ps, pD+(ipiv[i0+2])/ps*ps*sdd+(ipiv[i0+2])%ps);
				srowsw_lib(n-jj-4, pD+(i0+2)/ps*ps*sdd+(i0+2)%ps+(jj+4)*ps, pD+(ipiv[i0+2])/ps*ps*sdd+(ipiv[i0+2])%ps+(jj+4)*ps);
				}
			if(n-jj>3)
				{
				ipiv[i0+3] += i0;
				if(ipiv[i0+3]!=i0+3)
					{
					srowsw_lib(jj, pD+(i0+3)/ps*ps*sdd+(i0+3)%ps, pD+(ipiv[i0+3])/ps*ps*sdd+(ipiv[i0+3])%ps);
					srowsw_lib(n-jj-4, pD+(i0+3)/ps*ps*sdd+(i0+3)%ps+(jj+4)*ps, pD+(ipiv[i0+3])/ps*ps*sdd+(ipiv[i0+3])%ps+(jj+4)*ps);
					}
				}
			}
		}

	// solve upper
	if(0) // there is no upper
		{
		ll = jj+4;
		for( ; ll<n; ll+=4)
			{
			kernel_strsm_nn_ll_one_4x4_vs_lib4(i0, &pD[i0*sdd], &pD[ll*ps], sdd, &pD[ll*ps+i0*sdd], &pD[ll*ps+i0*sdd], &pD[i0*ps+i0*sdd], m-i0, n-ll);
			}
		}
	return;


	left_m_4:
	// 1-4 rows at a time
	// pivot & factorize & solve lower
	ii = jj;
	i0 = ii;
	kernel_sgemm_nn_4x4_vs_lib4(jj, &dm1, &pD[ii*sdd], 0, &pD[jj*ps], sdd, &d1, &pD[jj*ps+ii*sdd], &pD[jj*ps+ii*sdd], m-ii, n-jj);
	kernel_sgetrf_pivot_4_vs_lib4(m-i0, n-jj, &pD[jj*ps+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		srowsw_lib(jj, pD+(i0+0)/ps*ps*sdd+(i0+0)%ps, pD+(ipiv[i0+0])/ps*ps*sdd+(ipiv[i0+0])%ps);
		srowsw_lib(n-jj-4, pD+(i0+0)/ps*ps*sdd+(i0+0)%ps+(jj+4)*ps, pD+(ipiv[i0+0])/ps*ps*sdd+(ipiv[i0+0])%ps+(jj+4)*ps);
		}
	if(m-i0>1)
		{
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			srowsw_lib(jj, pD+(i0+1)/ps*ps*sdd+(i0+1)%ps, pD+(ipiv[i0+1])/ps*ps*sdd+(ipiv[i0+1])%ps);
			srowsw_lib(n-jj-4, pD+(i0+1)/ps*ps*sdd+(i0+1)%ps+(jj+4)*ps, pD+(ipiv[i0+1])/ps*ps*sdd+(ipiv[i0+1])%ps+(jj+4)*ps);
			}
		if(m-i0>2)
			{
			ipiv[i0+2] += i0;
			if(ipiv[i0+2]!=i0+2)
				{
				srowsw_lib(jj, pD+(i0+2)/ps*ps*sdd+(i0+2)%ps, pD+(ipiv[i0+2])/ps*ps*sdd+(ipiv[i0+2])%ps);
				srowsw_lib(n-jj-4, pD+(i0+2)/ps*ps*sdd+(i0+2)%ps+(jj+4)*ps, pD+(ipiv[i0+2])/ps*ps*sdd+(ipiv[i0+2])%ps+(jj+4)*ps);
				}
			if(m-i0>3)
				{
				ipiv[i0+3] += i0;
				if(ipiv[i0+3]!=i0+3)
					{
					srowsw_lib(jj, pD+(i0+3)/ps*ps*sdd+(i0+3)%ps, pD+(ipiv[i0+3])/ps*ps*sdd+(ipiv[i0+3])%ps);
					srowsw_lib(n-jj-4, pD+(i0+3)/ps*ps*sdd+(i0+3)%ps+(jj+4)*ps, pD+(ipiv[i0+3])/ps*ps*sdd+(ipiv[i0+3])%ps+(jj+4)*ps);
					}
				}
			}
		}

	// solve upper
	ll = jj+4;
	for( ; ll<n; ll+=4)
		{
		kernel_strsm_nn_ll_one_4x4_vs_lib4(i0, &pD[i0*sdd], &pD[ll*ps], sdd, &pD[ll*ps+i0*sdd], &pD[ll*ps+i0*sdd], &pD[i0*ps+i0*sdd], m-i0, n-ll);
		}
	return;


	return;
	}



int blasfeo_hp_sgeqrf_worksize(int m, int n)
	{
	return 0;
	}



void blasfeo_hp_sgeqrf(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgeqrf(m, n, sC, ci, cj, sD, di, dj, work);
	return;
#else
	printf("\nblasfeo_sgeqrf: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



int blasfeo_hp_sgelqf_worksize(int m, int n)
	{
	return 0;
	}



void blasfeo_hp_sgelqf(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf(m, n, sC, ci, cj, sD, di, dj, work);
	return;
#else
	printf("\nblasfeo_sgelqf: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



int blasfeo_hp_sorglq_worksize(int m, int n, int k)
	{
	return 0;
	}



void blasfeo_hp_sorglq(int m, int n, int k, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sorglq(m, n, k, sC, ci, cj, sD, di, dj, work);
	return;
#else
	printf("\nblasfeo_sorglq: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// LQ factorization with positive diagonal elements
void blasfeo_hp_sgelqf_pd(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf_pd(m, n, sC, ci, cj, sD, di, dj, work);
	return;
#else
	printf("\nblasfeo_sgelqf_pd: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, A] <= lq( [L. A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void blasfeo_hp_sgelqf_pd_la(int m, int n1, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_smat *sA, int ai, int aj, void *work)
	{
	if(m<=0)
		return;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf_pd_la(m, n1, sD, di, dj, sA, ai, aj, work);
	return;
#else
	printf("\nblasfeo_sgelqf_pd_la: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, L, A] <= lq( [L. L, A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void blasfeo_hp_sgelqf_pd_lla(int m, int n1, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_smat *sL, int li, int lj, struct blasfeo_smat *sA, int ai, int aj, void *work)
	{
	if(m<=0)
		return;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf_pd_lla(m, n1, sD, di, dj, sL, li, lj, sA, ai, aj, work);
	return;
#else
	printf("\nblasfeo_sgelqf_pd_lla: feature not implemented yet\n");
	exit(1);
#endif
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_spotrf_l(int m, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_spotrf_l(m, sC, ci, cj, sD, di, dj);
	}



void blasfeo_spotrf_l_mn(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_spotrf_l_mn(m, n, sC, ci, cj, sD, di, dj);
	}



void blasfeo_spotrf_u(int m, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_spotrf_u(m, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyrk_spotrf_ln_mn(int m, int n, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_spotrf_ln_mn(m, n, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
	}



void blasfeo_ssyrk_spotrf_ln(int m, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_spotrf_ln(m, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgetrf_np(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgetrf_np(m, n, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgetrf_rp(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, int *ipiv)
	{
	blasfeo_hp_sgetrf_rp(m, n, sC, ci, cj, sD, di, dj, ipiv);
	}



int blasfeo_sgeqrf_worksize(int m, int n)
	{
	return blasfeo_hp_sgeqrf_worksize(m, n);
	}



void blasfeo_sgeqrf(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *v_work)
	{
	blasfeo_hp_sgeqrf(m, n, sC, ci, cj, sD, di, dj, v_work);
	}



int blasfeo_sgelqf_worksize(int m, int n)
	{
	return blasfeo_hp_sgelqf_worksize(m, n);
	}



void blasfeo_sgelqf(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
	blasfeo_hp_sgelqf(m, n, sC, ci, cj, sD, di, dj, work);
	}



int blasfeo_sorglq_worksize(int m, int n, int k)
	{
	return blasfeo_hp_sorglq_worksize(m, n, k);
	}



void blasfeo_sorglq(int m, int n, int k, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
	blasfeo_hp_sorglq(m, n, k, sC, ci, cj, sD, di, dj, work);
	}



void blasfeo_sgelqf_pd(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
	blasfeo_hp_sgelqf_pd(m, n, sC, ci, cj, sD, di, cj, work);
	}



void blasfeo_sgelqf_pd_la(int m, int n1, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_smat *sA, int ai, int aj, void *work)
	{
	blasfeo_hp_sgelqf_pd_la(m, n1, sD, di, dj, sA, ai, aj, work);
	}



void blasfeo_sgelqf_pd_lla(int m, int n1, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_smat *sL, int li, int lj, struct blasfeo_smat *sA, int ai, int aj, void *work)
	{
	blasfeo_hp_sgelqf_pd_lla(m, n1, sD, di, dj, sL, li, lj, sA, ai, aj, work);
	}



#endif
