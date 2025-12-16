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
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#include <blasfeo_d_blasfeo_api.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_d_blasfeo_ref_api.h>
#endif



// dpotrf
void blasfeo_hp_dpotrf_l(int m, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

	if(m<=0)
		return;

	if(ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dpotrf_l(m, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dpotrf_l: feature not implemented yet: ci=%d, di=%d\n", ci, di);
		exit(1);
#endif
		}

	const int ps = 8;

	double alpha = 1.0;

	int sdc = sC->cn;
	int sdd = sD->cn;
	double *pC = sC->pA + cj*ps;
	double *pD = sD->pA + dj*ps;
	double *dD = sD->dA;

	if(di==0 & dj==0) // XXX what to do if di and dj are not zero
		sD->use_dA = m;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;
#if 1
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_24x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dpotrf_nt_l_24x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
#if 0 // TODO
		kernel_dpotrf_nt_l_16x16_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], sdd, &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
#else
		kernel_dpotrf_nt_l_16x8_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(j+8)*sdc], sdc, &pD[(j+8)*ps+(j+8)*sdd], sdd, &dD[j+8]);
		kernel_dpotrf_nt_l_8x8_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16]);
#endif
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else if(m-i<=16)
			{
			goto left_16;
			}
		else
			{
			goto left_24;
			}
		}
#elif 1
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_16x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dpotrf_nt_l_16x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_dpotrf_nt_l_8x8_lib8(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8]);
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_16;
			}
		}
#else
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_8x8_lib8(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dpotrf_nt_l_8x8_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
		}
	if(m>i)
		{
		goto left_8;
		}
#endif

	// common return if i==m
	return;

	// clean up loops definitions

	left_24:
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_dtrsm_nt_rl_inv_24x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
	kernel_dpotrf_nt_l_24x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
#if 0 // TODO
	kernel_dpotrf_nt_l_16x16_vs_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], sdd, &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, m-j-8);
#else
	kernel_dpotrf_nt_l_16x8_vs_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, m-j-8);
	kernel_dpotrf_nt_l_8x8_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16], m-i-16, m-j-16);
#endif
	return;

	left_16:
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_dtrsm_nt_rl_inv_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
#if 0 // TODO
	kernel_dpotrf_nt_l_16x16_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
#else
	kernel_dpotrf_nt_l_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
	kernel_dpotrf_nt_l_8x8_vs_lib8(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, m-j-8);
#endif
	return;

#if 0
	left_4:
	j = 0;
	for(; j<i-8; j+=12)
		{
		kernel_dtrsm_nt_rl_inv_4x12_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		}
	if(j<i-4)
		{
		kernel_dtrsm_nt_rl_inv_4x8_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		j += 8;
		}
	else if(j<i)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		j += 4;
		}
	kernel_dpotrf_nt_l_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
	return;
#else
	left_8:
	j = 0;
	if(m-i==8)
		{
		for(; j<i; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_8x8_lib8(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dpotrf_nt_l_8x8_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
		}
	else
		{
		for(; j<i; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			}
		kernel_dpotrf_nt_l_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
	return;
#endif

	}



// dpotrf
void blasfeo_hp_dpotrf_l_mn(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	if(m<=0 || n<=0)
		return;

	if(ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dpotrf_l_mn(m, n, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dpotrf_l_mn: feature not implemented yet: ci=%d, di=%d\n", ci, di);
		exit(1);
#endif
		}

	const int ps = 8;

	double alpha = 1.0;

	int sdc = sC->cn;
	int sdd = sD->cn;
	double *pC = sC->pA + cj*ps;
	double *pD = sD->pA + dj*ps;
	double *dD = sD->dA;

	if(di==0 & dj==0) // XXX what to do if di and dj are not zero
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;
#if 1
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_24x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dtrsm
				{
				kernel_dtrsm_nt_rl_inv_24x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
//				kernel_dtrsm_nt_rl_inv_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
//				kernel_dtrsm_nt_rl_inv_8x8_vs_lib8(j, &pD[(i+16)*sdd], &pD[j*sdd], &alpha, &pC[j*ps+(i+16)*sdc], &pD[j*ps+(i+16)*sdd], &pD[j*ps+j*sdd], &dD[j], m-(i+16), n-j);
				}
			else // dpotrf
				{
				if(j<n-23)
					{
					kernel_dpotrf_nt_l_24x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
//					kernel_dpotrf_nt_l_16x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
//					kernel_dtrsm_nt_rl_inv_8x8_lib8(j, &pD[(i+16)*sdd], &pD[j*sdd], &alpha, &pC[j*ps+(i+16)*sdc], &pD[j*ps+(i+16)*sdd], &pD[j*ps+j*sdd], &dD[j]);
#if 0 // TODO ???
					kernel_dpotrf_nt_l_16x16_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], sdd, &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
#else
					kernel_dpotrf_nt_l_16x8_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
					kernel_dpotrf_nt_l_8x8_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16]);
#endif
					}
				else
					{
					kernel_dpotrf_nt_l_24x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
//					kernel_dpotrf_nt_l_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
//					kernel_dtrsm_nt_rl_inv_8x8_vs_lib8(j, &pD[(i+16)*sdd], &pD[j*sdd], &alpha, &pC[j*ps+(i+16)*sdc], &pD[j*ps+(i+16)*sdd], &pD[j*ps+j*sdd], &dD[j], m-i-16, n-j);
					if(j<n-8)
						{
						kernel_dpotrf_nt_l_16x8_vs_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, n-j-8);
						if(j<n-16)
							{
							kernel_dpotrf_nt_l_8x8_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16], m-i-16, n-j-16);
							}
						}
					}
				}
			}
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else if(m-i<=16)
			{
			goto left_16;
			}
		else
			{
			goto left_24;
			}
		}
#elif 1
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_16x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dtrsm
				{
				kernel_dtrsm_nt_rl_inv_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dpotrf
				{
				if(j<n-15)
//				if(0)
					{
					kernel_dpotrf_nt_l_16x8_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
					kernel_dpotrf_nt_l_8x8_lib8(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8]);
					}
				else
					{
					kernel_dpotrf_nt_l_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
					if(j<n-8)
						{
						kernel_dpotrf_nt_l_8x8_vs_lib8(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, n-j-8);
						}
					}
				}
			}
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_16;
			}
		}
#else
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_dtrsm_nt_rl_inv_8x8_lib8(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dtrsm
				{
				kernel_dtrsm_nt_rl_inv_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dpotrf
				{
				if(j<n-7)
					{
					kernel_dpotrf_nt_l_8x8_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
					}
				else
					{
					kernel_dpotrf_nt_l_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
					}
				}
			}
		}
	if(m>i)
		{
		goto left_8;
		}
#endif

	// common return if i==m
	return;

	// clean up loops definitions

	left_24:
	j = 0;
	for(; j<i & j<n; j+=8)
		{
		kernel_dtrsm_nt_rl_inv_24x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dpotrf_nt_l_24x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		if(j<n-8)
			{
			kernel_dpotrf_nt_l_16x8_vs_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, n-j-8);
			if(j<n-16)
				{
				kernel_dpotrf_nt_l_8x8_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16], m-i-16, n-j-16);
				}
			}
		}
	return;

	left_16:
	j = 0;
	for(; j<i & j<n; j+=8)
		{
		kernel_dtrsm_nt_rl_inv_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dpotrf_nt_l_16x8_vs_lib8(j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		if(j<n-8)
			{
			kernel_dpotrf_nt_l_8x8_vs_lib8(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, n-j-8);
			}
		}
	return;

#if 0
	left_4:
	j = 0;
	for(; j<i-8 & j<n-8; j+=12)
		{
		kernel_dtrsm_nt_rl_inv_4x12_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		}
	if(j<i-4 & j<n-4)
		{
		kernel_dtrsm_nt_rl_inv_4x8_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		j += 8;
		}
	else if(j<i & j<n)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		j += 4;
		}
	if(j<n)
		{
		kernel_dpotrf_nt_l_4x4_vs_lib4(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	return;
#else
	left_8:
	j = 0;
	for(; j<i & j<n; j+=8)
		{
		kernel_dtrsm_nt_rl_inv_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &alpha, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dpotrf_nt_l_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	return;
#endif

	}



// dpotrf
void blasfeo_hp_dpotrf_u(int m, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dpotrf_u(m, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_dpotrf_u: feature not implemented yet\n");
	exit(1);
#endif
	
	}



void blasfeo_hp_dsyrk_dpotrf_ln(int m, int k, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	if(m<=0)
		return;

	if(ai!=0 | bi!=0 | ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dsyrk_dpotrf_ln(m, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dsyrk_dpotrf_ln: feature not implemented yet: ai=%d, bi=%d, ci=%d, di=%d\n", ai, bi, ci, di);
		exit(1);
#endif
		}

	const int ps = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pC = sC->pA + cj*ps;
	double *pD = sD->pA + dj*ps;
	double *dD = sD->dA; // XXX what to do if di and dj are not zero

	if(di==0 & dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;

#if 1
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_24x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dsyrk_dpotrf_nt_l_24x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
#if 0
		kernel_dsyrk_dpotrf_nt_l_16x16_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], sdb, j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], sdd, &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
#else
		kernel_dsyrk_dpotrf_nt_l_16x8_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
		kernel_dsyrk_dpotrf_nt_l_8x8_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16]);
#endif
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else if(m-i<=16)
			{
			goto left_16;
			}
		else
			{
			goto left_24;
			}
		}
#elif 0
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_16x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dsyrk_dpotrf_nt_l_16x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
		kernel_dsyrk_dpotrf_nt_l_8x8_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8]);
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_16;
			}
		}
#else
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_8x8_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dsyrk_dpotrf_nt_l_8x8_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
		}
	if(m>i)
		{
		goto left_8;
		}
#endif

	// common return if i==m
	return;

	// clean up loops definitions

	left_24:
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_24x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
	kernel_dsyrk_dpotrf_nt_l_24x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
#if 0
	kernel_dsyrk_dpotrf_nt_l_16x16_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], sdb, j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], sdd, &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, m-j-8);
#else
	kernel_dsyrk_dpotrf_nt_l_16x8_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, m-j-8);
	kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16], m-i-16, m-j-16);
#endif
	return;

	left_16:
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_16x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
	kernel_dsyrk_dpotrf_nt_l_16x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
	kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, m-j-8);
	return;

#if 0
	left_4:
	j = 0;
	for(; j<i-8; j+=12)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_4x12_vs_lib4(k, &pA[i*sda], &pB[j*sdb], sdb, j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		}
	if(j<i-4)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_4x8_vs_lib4(k, &pA[i*sda], &pB[j*sdb], sdb, j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, m-j);
		j += 8;
		}
	else if(j<i)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		j += 4;
		}
	kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
#else
	left_8:
	j = 0;
	if(m-i==8)
		{
		for(; j<i; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_8x8_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		kernel_dsyrk_dpotrf_nt_l_8x8_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
		}
	else
		{
		for(; j<i; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
			}
		kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, m-j);
		}
#endif

	return;

	}



// dsyrk dpotrf
void blasfeo_hp_dsyrk_dpotrf_ln_mn(int m, int n, int k, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{

	if(m<=0 || n<=0)
		return;

	if(ai!=0 | bi!=0 | ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dsyrk_dpotrf_ln_mn(m, n, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_dsyrk_dpotrf_ln_mn: feature not implemented yet: ai=%d, bi=%d, ci=%d, di=%d\n", ai, bi, ci, di);
		exit(1);
#endif
		}

	const int ps = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	double *pA = sA->pA + aj*ps;
	double *pB = sB->pA + bj*ps;
	double *pC = sC->pA + cj*ps;
	double *pD = sD->pA + dj*ps;
	double *dD = sD->dA; // XXX what to do if di and dj are not zero

	if(di==0 & dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	int i, j, l;

	i = 0;

#if 1
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_24x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dgemm
				{
				kernel_dgemm_dtrsm_nt_rl_inv_24x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dsyrk
				{
				if(j<n-23)
					{
					kernel_dsyrk_dpotrf_nt_l_24x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
#if 0
					kernel_dsyrk_dpotrf_nt_l_16x16_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], sdb, j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], sdd, &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
#else
					kernel_dsyrk_dpotrf_nt_l_16x8_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8]);
					kernel_dsyrk_dpotrf_nt_l_8x8_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16]);
#endif
					}
				else
					{
					kernel_dsyrk_dpotrf_nt_l_24x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
					if(j<n-8)
						{
						kernel_dsyrk_dpotrf_nt_l_16x8_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, n-j-8);
						if(j<n-16)
							{
							kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16], m-i-16, n-j-16);
							}
						}
					}
				}
			}
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else if(m-i<=16)
			{
			goto left_16;
			}
		else
			{
			goto left_24;
			}
		}
#elif 0
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_16x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dgemm
				{
				kernel_dgemm_dtrsm_nt_rl_inv_16x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dsyrk
				{
				if(j<n-15)
//				if(0)
					{
					kernel_dsyrk_dpotrf_nt_l_16x8_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j]);
					kernel_dsyrk_dpotrf_nt_l_8x8_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8]);
					}
				else
					{
					kernel_dsyrk_dpotrf_nt_l_16x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
					if(j<n-8)
						{
						kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, n-j-8);
						}
					}
				}
			}
		}
	if(m>i)
		{
		if(m-i<=8)
			{
			goto left_8;
			}
		else
			{
			goto left_16;
			}
		}
#else
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_dgemm_dtrsm_nt_rl_inv_8x8_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j]);
			}
		if(j<n)
			{
			if(j<i) // dgemm
				{
				kernel_dgemm_dtrsm_nt_rl_inv_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
				}
			else // dsyrk
				{
				if(j<n-7)
					{
					kernel_dsyrk_dpotrf_nt_l_8x8_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j]);
					}
				else
					{
					kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
					}
				}
			}
		}
	if(m>i)
		{
		goto left_8;
		}
#endif

	// common return if i==m
	return;

	// clean up loops definitions

	left_24:
	j = 0;
	for(; j<i & j<n; j+=8)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_24x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dsyrk_dpotrf_nt_l_24x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		if(j<n-8)
			{
			kernel_dsyrk_dpotrf_nt_l_16x8_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], sdc, &pD[(j+8)*ps+(i+8)*sdd], sdd, &dD[j+8], m-i-8, n-j-8);
			if(j<n-16)
				{
				kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*ps+(i+16)*sdc], &pD[(j+16)*ps+(i+16)*sdd], &dD[j+16], m-i-16, n-j-16);
				}
			}
		}
	return;

	left_16:
	j = 0;
	for(; j<i & j<n; j+=8)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_16x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+i*sdc], sdc, &pD[j*ps+i*sdd], sdd, &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dsyrk_dpotrf_nt_l_16x8_vs_lib8(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], &pC[j*ps+j*sdc], sdc, &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		if(j<n-8)
			{
			kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*ps+(i+8)*sdc], &pD[(j+8)*ps+(i+8)*sdd], &dD[j+8], m-i-8, n-j-8);
			}
		}
	return;

#if 0
	left_4:
	j = 0;
	for(; j<i-8 & j<n-8; j+=12)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_4x12_vs_lib4(k, &pA[i*sda], &pB[j*sdb], sdb, j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		}
	if(j<i-4 & j<n-4)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_4x8_vs_lib4(k, &pA[i*sda], &pB[j*sdb], sdb, j, &pD[i*sdd], &pD[j*sdd], sdd, &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], sdd, &dD[j], m-i, n-j);
		j += 8;
		}
	else if(j<i & j<n)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		j += 4;
		}
	if(j<n)
		{
		kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
#else
	left_8:
	j = 0;
	for(; j<i & j<n; j+=8)
		{
		kernel_dgemm_dtrsm_nt_rl_inv_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+i*sdc], &pD[j*ps+i*sdd], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*ps+j*sdc], &pD[j*ps+j*sdd], &dD[j], m-i, n-j);
		}
#endif

	return;

	}



// dgetrf no pivoting
void blasfeo_hp_dgetrf_np(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dgetrf_np(m, n, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_dgetf_np: feature not implemented yet\n");
	exit(1);
#endif
	}



// dgetrf row pivoting
void blasfeo_hp_dgetrf_rp(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, int *ipiv)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dgetrf_rp(m, n, sC, ci, cj, sD, di, dj, ipiv);
#else
	printf("\nblasfeo_dgetrf_rp: feature not implemented yet\n");
	exit(1);
#endif
	}



int blasfeo_hp_dgeqrf_worksize(int m, int n)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dgeqrf_worksize(m, n);
#else
	printf("\nblasfeo_dgeqrf_worksize: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_dgeqrf(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dgeqrf(m, n, sC, ci, cj, sD, di, dj, work);
#else
	printf("\nblasfeo_dgeqrf: feature not implemented yet\n");
	exit(1);
#endif
	}



int blasfeo_hp_dgelqf_worksize(int m, int n)
	{
	return 0;
	}



// LQ factorization
void blasfeo_hp_dgelqf(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *work)
	{
//	blasfeo_ref_dgelqf(m, n, sC, ci, cj, sD, di, dj, work);
//	return;

	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 8;

	// extract dimensions
	int sdc = sC->cn;
	int sdd = sD->cn;

	int cir = ci & (ps-1);
	int dir = di & (ps-1);

	// go to submatrix
	double *pC = sC->pA + cj*ps + (ci-cir)*sdc;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;

	double *dD = sD->dA + di;
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	ALIGNED( double pT[64], 64 ) = {0}; // XXX assuming rank-8 update
//	ALIGNED( double pK[64], 64 ) = {0};
#else
	double pT[144] = {0}; // XXX smaller ?
//	double pK[96] = {0}; // XXX smaller ?
#endif

	if(pC!=pD)
		// copy strmat submatrix
		blasfeo_dgecp(m, n, sC, ci, cj, sD, di, dj);

	int ii, jj, ll, imax0;
	int imax = m<n ? m : n;
#if 0
	kernel_dgelqf_vs_lib8(m, n, imax, dir, pD, sdd, dD);
#else
	if(dir>0)
		{
		imax0 = (ps-dir)&(ps-1);
		imax0 = imax<imax0 ? imax : imax0;
		kernel_dgelqf_vs_lib8(m, n, imax0, dir, pD, sdd, dD);
		pD += imax0-ps+ps*sdd+imax0*ps;
		dD += imax0;
		m -= imax0;
		n -= imax0;
		imax -= imax0;
		}
	ii = 0;
	// rank 8 update
	for(; ii<imax-8; ii+=8)
		{
//		kernel_dgelqf_vs_lib8(8, n-ii, 8, 0, pD+ii*sdd+ii*ps, sdd, dD+ii);
//		kernel_dgelqf_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
//		kernel_dlarft_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii, pT);
		kernel_dgelqf_dlarft8_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii, pT);
//d_print_mat(1, 8, dD+ii, 1);
//d_print_mat(8, 8, pT, 8);
		jj = ii+8;
#if 1
		for(; jj<m-23; jj+=24)
			{
			kernel_dlarfb8_rn_24_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps, sdd);
			}
#endif
#if 1
		for(; jj<m-15; jj+=16)
			{
			kernel_dlarfb8_rn_16_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps, sdd);
			}
#endif
		for(; jj<m-7; jj+=8)
			{
			kernel_dlarfb8_rn_8_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps);
			}
		if(jj<m)
			{
			kernel_dlarfb8_rn_8_vs_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps, m-jj);
			}
		}
	if(ii<imax)
		{
		if(ii==imax-8)
			{
			kernel_dgelqf_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
			}
		else
			{
			kernel_dgelqf_vs_lib8(m-ii, n-ii, imax-ii, ii&(ps-1), pD+ii*sdd+ii*ps, sdd, dD+ii);
			}
		}
#endif

	return;
	}



int blasfeo_hp_dorglq_worksize(int m, int n, int k)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dorglq_worksize(m, n, k);
#else
	printf("\nblasfeo_dorglq_worksize: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_dorglq(int m, int n, int k, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dorglq(m, n, k, sC, ci, cj, sD, di, dj, work);
#else
	printf("\nblasfeo_dorglq: feature not implemented yet\n");
	exit(1);
#endif
	}



// LQ factorization with positive diagonal elements
void blasfeo_hp_dgelqf_pd(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *work)
	{
//	blasfeo_ref_dgelqf_pd(m, n, sC, ci, cj, sD, di, dj, work);
//	return;

	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int ps = 8;

	// extract dimensions
	int sdc = sC->cn;
	int sdd = sD->cn;

	int cir = ci & (ps-1);
	int dir = di & (ps-1);

	// go to submatrix
	double *pC = sC->pA + cj*ps + (ci-cir)*sdc;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;

	double *dD = sD->dA + di;
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	ALIGNED( double pT[64], 64 ) = {0}; // XXX assuming 8x8 kernel
//	ALIGNED( double pK[64], 64 ) = {0};
#else
	double pT[144] = {0};
//	double pK[96] = {0};
#endif

	if(pC!=pD)
		// copy strmat submatrix
		blasfeo_dgecp(m, n, sC, ci, cj, sD, di, dj);

	int ii, jj, ll, imax0;
	int imax = m<n ? m : n;
#if 0
	kernel_dgelqf_pd_vs_lib8(m, n, imax, dir, pD, sdd, dD);
#else
	if(dir>0)
		{
		imax0 = (ps-dir)&(ps-1);
		imax0 = imax<imax0 ? imax : imax0;
		kernel_dgelqf_pd_vs_lib8(m, n, imax0, dir, pD, sdd, dD);
		pD += imax0-ps+ps*sdd+imax0*ps;
		dD += imax0;
		m -= imax0;
		n -= imax0;
		imax -= imax0;
		}
	ii = 0;
	// rank 8 update
	for(ii=0; ii<imax-8; ii+=8)
		{
//		kernel_dgelqf_pd_vs_lib8(8, n-ii, 8, 0, pD+ii*sdd+ii*ps, sdd, dD+ii);
//		kernel_dgelqf_pd_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
//		kernel_dlarft_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii, pT);
		kernel_dgelqf_pd_dlarft8_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii, pT);
		jj = ii+8;
//d_print_mat(1, 8, dD+ii, 1);
//d_print_mat(8, 8, pT, 8);
//return;
#if 1
		for(; jj<m-23; jj+=24)
			{
			kernel_dlarfb8_rn_24_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps, sdd);
			}
#endif
#if 1
		for(; jj<m-15; jj+=16)
			{
			kernel_dlarfb8_rn_16_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps, sdd);
			}
#endif
		for(; jj<m-7; jj+=8)
			{
			kernel_dlarfb8_rn_8_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps);
			}
		if(jj<m)
			{
			kernel_dlarfb8_rn_8_vs_lib8(n-ii, pD+ii*sdd+ii*ps, pT, pD+jj*sdd+ii*ps, m-jj);
			}
		}
	if(ii<imax)
		{
		if(ii==imax-8)
			{
			kernel_dgelqf_pd_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
			}
		else
			{
			kernel_dgelqf_pd_vs_lib8(m-ii, n-ii, imax-ii, ii&(ps-1), pD+ii*sdd+ii*ps, sdd, dD+ii);
			}
		}
#endif

	return;
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, A] <= lq( [L. A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void blasfeo_hp_dgelqf_pd_la(int m, int n1, struct blasfeo_dmat *sD, int di, int dj, struct blasfeo_dmat *sA, int ai, int aj, void *work)
	{
	if(m<=0)
		return;
//	printf("\nblasfeo_dgelqf_pd_la: feature not implemented yet\n");
//	exit(1);

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;
	sA->use_dA = 0;

	const int ps = 8;

	// extract dimensions
	int sda = sA->cn;
	int sdd = sD->cn;

	int air = ai & (ps-1);
	int dir = di & (ps-1);

	// go to submatrix
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;

	double *dD = sD->dA + di;
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	ALIGNED( double pT[64], 64 ) = {0}; // XXX assuming 8x8 kernel
//	ALIGNED( double pK[96], 64 ) = {0};
#else
	double pT[144] = {0};
//	double pK[96] = {0};
#endif

	int ii, jj, ll;
	int imax0 = (ps-dir)&(ps-1);
	int imax = m;
	imax0 = imax<imax0 ? imax : imax0;
	// different block alignment
	if( dir != air )
		{
		// XXX vecorized kernel requires air==dir
//		kernel_dgelqf_pd_la_vs_lib4(m, n1, imax, dir, pD, sdd, dD, air, pA, sda);
//		return;
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dgelqf_pd_la(m, n1, sD, di, dj, sA, ai, aj, work);
		return;
#else
		printf("\nblasfeo_dgelqf_pd_la: feature not implemented yet: ai!=di\n");
		exit(1);
#endif
		}
	// same block alignment
#if 0
	kernel_dgelqf_pd_la_vs_lib8(m, n1, imax, dir, pD, sdd, dD, air, pA, sda);
#else
	if(imax0>0)
		{
		kernel_dgelqf_pd_la_vs_lib8(m, n1, imax0, dir, pD, sdd, dD, air, pA, sda);
		pD += imax0-ps+ps*sdd+imax0*ps;
		dD += imax0;
		pA += imax0-ps+ps*sda+0*ps;
		m -= imax0;
		imax -= imax0;
		}
	ii = 0;
	for(ii=0; ii<imax-8; ii+=8)
		{
//		kernel_dgelqf_pd_la_vs_lib8(8, n1, 8, 0, pD+ii*sdd+ii*ps, sdd, dD+ii, 0, pA+ii*sda+0*ps, sda);
//		kernel_dgelqf_pd_la_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
//		kernel_dlarft_la_8_lib8(n1, dD+ii, pA+ii*sda+0*ps, pT);
		kernel_dgelqf_pd_la_dlarft8_8_lib8(n1, pD+ii*sdd+ii*ps, dD+ii, pA+ii*sda+0*ps, pT);
		jj = ii+8;
//d_print_mat(1, 8, dD+ii, 1);
//d_print_mat(8, 8, pT, 8);
//return;
#if 1
		for(; jj<m-15; jj+=16)
			{
			kernel_dlarfb8_rn_la_16_lib8(n1, pA+ii*sda+0*ps, pT, pD+jj*sdd+ii*ps, sdd, pA+jj*sda+0*ps, sda);
			}
#endif
		for(; jj<m-7; jj+=8)
			{
			kernel_dlarfb8_rn_la_8_lib8(n1, pA+ii*sda+0*ps, pT, pD+jj*sdd+ii*ps, pA+jj*sda+0*ps);
			}
		if(jj<m)
			{
			kernel_dlarfb8_rn_la_8_vs_lib8(n1, pA+ii*sda+0*ps, pT, pD+jj*sdd+ii*ps, pA+jj*sda+0*ps, m-jj);
			}
		}
	if(ii<imax)
		{
//		if(ii==imax-8)
//			{
//			kernel_dgelqf_pd_la_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
//			}
//		else
//			{
			kernel_dgelqf_pd_la_vs_lib8(m-ii, n1, imax-ii, 0, pD+ii*sdd+ii*ps, sdd, dD+ii, 0, pA+ii*sda+0*ps, sda);
//			}
		}
#endif
	return;
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, L, A] <= lq( [L. L, A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void blasfeo_hp_dgelqf_pd_lla(int m, int n1, struct blasfeo_dmat *sD, int di, int dj, struct blasfeo_dmat *sL, int li, int lj, struct blasfeo_dmat *sA, int ai, int aj, void *work)
	{
	if(m<=0)
		return;

	if(li!=ai)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dgelqf_pd_lla(m, n1, sD, di, dj, sL, li, lj, sA, ai, aj, work);
		return;
#else
		printf("\nblasfeo_dgelqf_pd_lla: feature not implemented yet: li!=ai\n");
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;
	sL->use_dA = 0;
	sA->use_dA = 0;

	const int ps = 8;

	// extract dimensions
	int sda = sA->cn;
	int sdl = sL->cn;
	int sdd = sD->cn;

	int air = ai & (ps-1);
	int lir = li & (ps-1);
	int dir = di & (ps-1);

	// go to submatrix
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pL = sL->pA + lj*ps + (li-lir)*sdl;
	double *pD = sD->pA + dj*ps + (di-dir)*sdd;

	double *dD = sD->dA + di;
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	ALIGNED( double pT[144], 64 ) = {0}; // XXX assuming 8x8 kernel
//	ALIGNED( double pK[96], 64 ) = {0};
#else
	double pT[144] = {0};
//	double pK[96] = {0};
#endif

	int ii, jj, ll;
	int imax0 = (ps-(di&(ps-1)))&(ps-1);
	int imax = m;
	imax0 = imax<imax0 ? imax : imax0;
	// different block alignment
	if( (di&(ps-1)) != (ai&(ps-1)) | imax0>0 )
		{
//		kernel_dgelqf_pd_lla_vs_lib4(m, 0, n1, imax, di&(ps-1), pD, sdd, dD, li&(ps-1), pL, sdl, ai&(ps-1), pA, sda);
//		return;
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dgelqf_pd_lla(m, n1, sD, di, dj, sL, li, lj, sA, ai, aj, work);
		return;
#else
		printf("\nblasfeo_dgelqf_pd_lla: feature not implemented yet: ai!=di\n");
		exit(1);
#endif
		}
	// same block alignment
#if 0
	kernel_dgelqf_pd_lla_vs_lib8(m, 0, n1, imax, dir, pD, sdd, dD, lir, pL, sdl, air, pA, sda);
#else
	if(imax0>0)
		{
		kernel_dgelqf_pd_lla_vs_lib8(m, 0, n1, imax0, dir, pD, sdd, dD, lir, pL, sdl, air, pA, sda);
		pD += imax0-ps+ps*sdd+imax0*ps;
		dD += imax0;
		pA += imax0-ps+ps*sda+0*ps;
		pL += imax0-ps+ps*sdl+0*ps;
		m -= imax0;
		imax -= imax0;
		}
	ii = 0;
	for(ii=0; ii<imax-8; ii+=8)
		{
//		kernel_dgelqf_pd_lla_vs_lib8(8, imax0+ii, n1, 8, 0, pD+ii*sdd+ii*ps, sdd, dD+ii, 0, pL+ii*sdl+0*ps, sdl, 0, pA+ii*sda+0*ps, sda);
//		kernel_dgelqf_pd_lla_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
//		kernel_dlarft_lla_8_lib8(imax0+ii, n1, dD+ii, pL+ii*sdl+0*ps, pA+ii*sda+0*ps, pT);
		kernel_dgelqf_pd_lla_dlarft8_8_lib8(imax0+ii, n1, pD+ii*sdd+ii*ps, dD+ii, pL+ii*sdl+0*ps, pA+ii*sda+0*ps, pT);
		jj = ii+8;
//d_print_mat(1, 8, dD+ii, 1);
//d_print_mat(8, 8, pT, 8);
//return;
#if 1
		for(; jj<m-15; jj+=16)
			{
			kernel_dlarfb8_rn_lla_16_lib8(imax0+ii, n1, pL+ii*sdl+0*ps, pA+ii*sda+0*ps, pT, pD+jj*sdd+ii*ps, sdd, pL+jj*sdl+0*ps, sdl, pA+jj*sda+0*ps, sda);
			}
#endif
		for(; jj<m-7; jj+=8)
			{
			kernel_dlarfb8_rn_lla_8_lib8(imax0+ii, n1, pL+ii*sdl+0*ps, pA+ii*sda+0*ps, pT, pD+jj*sdd+ii*ps, pL+jj*sdl+0*ps, pA+jj*sda+0*ps);
			}
		if(jj<m)
			{
			kernel_dlarfb8_rn_lla_8_vs_lib8(imax0+ii, n1, pL+ii*sdl+0*ps, pA+ii*sda+0*ps, pT, pD+jj*sdd+ii*ps, pL+jj*sdl+0*ps, pA+jj*sda+0*ps, m-jj);
			}
		}
	if(ii<imax)
		{
//		if(ii==imax-8)
//			{
//			kernel_dgelqf_pd_8_lib8(n-ii, pD+ii*sdd+ii*ps, dD+ii);
//			}
//		else
//			{
			kernel_dgelqf_pd_lla_vs_lib8(m-ii, imax0+ii, n1, imax-ii, 0, pD+ii*sdd+ii*ps, sdd, dD+ii, 0, pL+ii*sdl+0*ps, sdl, 0, pA+ii*sda+0*ps, sda);
//			}
		}
#endif
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dpotrf_l(int m, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dpotrf_l(m, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dpotrf_l_mn(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dpotrf_l_mn(m, n, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dpotrf_u(int m, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dpotrf_u(m, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk_dpotrf_ln(int m, int k, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk_dpotrf_ln(m, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dsyrk_dpotrf_ln_mn(int m, int n, int k, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dsyrk_dpotrf_ln_mn(m, n, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dgetrf_np(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dgetrf_np(m, n, sC, ci, cj, sD, di, dj);
	}



void blasfeo_dgetrf_rp(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, int *ipiv)
	{
	blasfeo_hp_dgetrf_rp(m, n, sC, ci, cj, sD, di, dj, ipiv);
	}



int blasfeo_dgeqrf_worksize(int m, int n)
	{
	return blasfeo_hp_dgeqrf_worksize(m, n);
	}



void blasfeo_dgeqrf(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *v_work)
	{
	blasfeo_hp_dgeqrf(m, n, sC, ci, cj, sD, di, dj, v_work);
	}



int blasfeo_dgelqf_worksize(int m, int n)
	{
	return blasfeo_hp_dgelqf_worksize(m, n);
	}



void blasfeo_dgelqf(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *work)
	{
	blasfeo_hp_dgelqf(m, n, sC, ci, cj, sD, di, dj, work);
	}



int blasfeo_dorglq_worksize(int m, int n, int k)
	{
	return blasfeo_hp_dorglq_worksize(m, n, k);
	}



void blasfeo_dorglq(int m, int n, int k, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *work)
	{
	blasfeo_hp_dorglq(m, n, k, sC, ci, cj, sD, di, dj, work);
	}



void blasfeo_dgelqf_pd(int m, int n, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj, void *work)
	{
	blasfeo_hp_dgelqf_pd(m, n, sC, ci, cj, sD, di, cj, work);
	}



void blasfeo_dgelqf_pd_la(int m, int n1, struct blasfeo_dmat *sD, int di, int dj, struct blasfeo_dmat *sA, int ai, int aj, void *work)
	{
	blasfeo_hp_dgelqf_pd_la(m, n1, sD, di, dj, sA, ai, aj, work);
	}



void blasfeo_dgelqf_pd_lla(int m, int n1, struct blasfeo_dmat *sD, int di, int dj, struct blasfeo_dmat *sL, int li, int lj, struct blasfeo_dmat *sA, int ai, int aj, void *work)
	{
	blasfeo_hp_dgelqf_pd_lla(m, n1, sD, di, dj, sL, li, lj, sA, ai, aj, work);
	}



#endif

