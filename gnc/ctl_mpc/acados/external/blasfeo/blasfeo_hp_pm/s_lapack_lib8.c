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



void blasfeo_hp_spotrf_l(int m, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0)
		return;

	if(ci>0 | di>0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_spotrf_l(m, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_spotrf_l: feature not implemented yet: ci>0, di>0\n");
		exit(1);
#endif
		}

	const int bs = 8;

	int i, j;

	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pC = sC->pA + cj*bs;
	float *pD = sD->pA + dj*bs;
	float *dD = sD->dA; // XXX what to do if di and dj are not zero
	if(di==0 & dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_strsm_nt_rl_inv_24x4_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0]);
			kernel_strsm_nt_rl_inv_24x4_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4]);
			}
		kernel_spotrf_nt_l_24x4_lib8((j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0]);
		kernel_spotrf_nt_l_20x4_lib8((j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4]);
		kernel_spotrf_nt_l_16x4_lib8((j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8]);
		kernel_spotrf_nt_l_12x4_lib8((j+12), &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12]);
		kernel_spotrf_nt_l_8x8_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16]);
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
		else if(m-i<=16)
			{
			goto left_16;
			}
		else
			{
			goto left_24;
			}
		}
#else
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_strsm_nt_rl_inv_16x4_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0]);
			kernel_strsm_nt_rl_inv_16x4_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4]);
			}
		kernel_spotrf_nt_l_16x4_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0]);
		kernel_spotrf_nt_l_12x4_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4]);
		kernel_spotrf_nt_l_8x8_lib8((j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8]);
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
#endif

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_24: // 17 <= m <= 23
	j = 0;
	for(; j<i & j<m-7; j+=8)
		{
		kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, m-(j+0));
		kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, m-(j+4));
		}
	kernel_spotrf_nt_l_24x4_vs_lib8((j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0], m-(i+0), m-(j+0));
	kernel_spotrf_nt_l_20x4_vs_lib8((j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4], m-(i+0), m-(j+4));
	kernel_spotrf_nt_l_16x4_vs_lib8((j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), m-(j+8));
	kernel_spotrf_nt_l_12x4_vs_lib8((j+12), &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12], m-(i+8), m-(j+12));
	if(j<m-20) // 21 - 23
		{
		kernel_spotrf_nt_l_8x8_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), m-(j+16));
		}
	else // 17 18 19 20
		{
		kernel_spotrf_nt_l_8x4_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), m-(j+16));
		}
	return;
#endif

	left_16: // 9 <= m <= 16
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, m-(j+0));
		kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, m-(j+4));
		}
	kernel_spotrf_nt_l_16x4_vs_lib8(j+0, &pD[(i+0)*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+j*sdc], sdc, &pD[(j+0)*bs+j*sdd], sdd, &dD[j+0], m-(i+0), m-(j+0));
	kernel_spotrf_nt_l_12x4_vs_lib8(j+4, &pD[(i+0)*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+j*sdc], sdc, &pD[(j+4)*bs+j*sdd], sdd, &dD[j+4], m-(i+0), m-(j+4));
	if(j<m-12) // 13 - 16
		{
		kernel_spotrf_nt_l_8x8_vs_lib8((j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), m-(j+8));
		}
	else // 9 - 12
		{
		kernel_spotrf_nt_l_8x4_vs_lib8((j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), m-(j+8));
		}
	return;

#if defined(TARGET_X64_INTEL_HASWELL)
	left_12: // 9 <= m <= 12
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_strsm_nt_rl_inv_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, m-j);
		kernel_strsm_nt_rl_inv_4x8_vs_lib8(j, &pD[(i+8)*sdd], &pD[j*sdd], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], &dD[j], m-(i+8), m-j);
		}
	kernel_spotrf_nt_l_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, m-j);
	kernel_strsm_nt_rl_inv_4x8_vs_lib8(j, &pD[(i+8)*sdd], &pD[j*sdd], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], &dD[j], m-(i+8), m-j);
	if(j<m-8) // 9 - 12
		{
		kernel_spotrf_nt_l_8x4_vs_lib8((j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[(j+8)], m-(i+8), m-(j+8));
		}
	return;
#endif

	left_8: // 1 <= m <= 8
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_strsm_nt_rl_inv_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, m-j);
		}
	if(j<m-4) // 5 - 8
		{
		kernel_spotrf_nt_l_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, m-j);
		}
	else // 1 - 4
		{
		kernel_spotrf_nt_l_8x4_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, m-j);
		}
	return;

#if defined(TARGET_X64_INTEL_HASWELL)
	left_4: // 1 <= m <= 4
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_strsm_nt_rl_inv_4x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, m-j);
		}
	kernel_spotrf_nt_l_8x4_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, m-j);
	return;
#endif

	}



void blasfeo_hp_spotrf_l_mn(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 | n<=0)
		return;

	if(ci>0 | di>0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_spotrf_l_mn(m, n, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_spotrf_l_mn: feature not implemented yet: ci>0, di>0\n");
		exit(1);
#endif
		}

	const int bs = 8;

	int i, j;

	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pC = sC->pA + cj*bs;
	float *pD = sD->pA + dj*bs;
	float *dD = sD->dA; // XXX what to do if di and dj are not zero
	if(di==0 & dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_strsm_nt_rl_inv_24x4_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0]);
			kernel_strsm_nt_rl_inv_24x4_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4]);
			}
		if(j<n)
			{
			if(i<j) // dtrsm
				{
				kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
				if(j<n-4) // 5 6 7
					{
					kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[(j+4)*bs+(j+4)*sdd], &dD[j+4], m-i, n-(j+4));
					}
				}
			else // dpotrf
				{
				if(j<n-23)
					{
					kernel_spotrf_nt_l_24x4_lib8((j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0]);
					kernel_spotrf_nt_l_20x4_lib8((j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4]);
					kernel_spotrf_nt_l_16x4_lib8((j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8]);
					kernel_spotrf_nt_l_12x4_lib8((j+12), &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12]);
					kernel_spotrf_nt_l_8x8_lib8((j+16), &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16]);
					}
				else
					{
					if(j<n-4) // 5 - 23
						{
						kernel_spotrf_nt_l_24x4_vs_lib8((j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
						kernel_spotrf_nt_l_20x4_vs_lib8((j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
						if(j==n-8)
							return;
						if(j<n-12) // 13 - 23
							{
							kernel_spotrf_nt_l_16x4_vs_lib8((j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
							kernel_spotrf_nt_l_12x4_vs_lib8((j+12), &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12], m-(i+8), n-(j+12));
							if(j==n-16)
								return;
							if(j<n-20) // 21 - 23
								{
								kernel_spotrf_nt_l_8x8_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
								}
							else // 17 18 19 20
								{
								kernel_spotrf_nt_l_8x4_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
								}
							}
						else // 9 10 11 12
							{
							kernel_spotrf_nt_l_16x4_vs_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
							}
						}
					else // 1 2 3 4
						{
						kernel_spotrf_nt_l_24x4_vs_lib8(j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
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
#else
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_strsm_nt_rl_inv_16x4_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0]);
			kernel_strsm_nt_rl_inv_16x4_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4]);
			}
		if(j<n)
			{
			if(i<j) // dtrsm
				{
				kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
				if(j<n-4) // 5 6 7
					{
					kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[(j+4)*bs+(j+4)*sdd], &dD[j+4], m-i, n-(j+4));
					}
				}
			else // dpotrf
				{
				if(j<n-15)
					{
					kernel_spotrf_nt_l_16x4_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0]);
					kernel_spotrf_nt_l_12x4_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4]);
					kernel_spotrf_nt_l_8x8_lib8((j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8]);
					}
				else
					{
					if(j<n-4) // 5 - 15
						{
						kernel_spotrf_nt_l_16x4_vs_lib8((j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
						kernel_spotrf_nt_l_12x4_vs_lib8((j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
						if(j==n-8) // 8
							return;
						if(j<n-12) // 13 - 15
							{
							kernel_spotrf_nt_l_8x8_vs_lib8(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
							}
						else // 9 10 11 12
							{
							kernel_spotrf_nt_l_8x4_vs_lib8(j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
							}
						}
					else // 1 2 3 4
						{
						kernel_spotrf_nt_l_16x4_vs_lib8(j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
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
#endif

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_24:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
		kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
		}
	if(j<n)
		{
		if(j<i) // dtrsm
			{
			kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
			if(j<n-4) // 5 6 7
				{
				kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
				}
			}
		else // dpotrf
			{
			if(j<n-4) // 5 - 23
				{
				kernel_spotrf_nt_l_24x4_vs_lib8((j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
				kernel_spotrf_nt_l_20x4_vs_lib8((j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
				if(j>=n-8)
					return;
				if(j<n-12) // 13 - 23
					{
					kernel_spotrf_nt_l_16x4_vs_lib8((j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
					kernel_spotrf_nt_l_12x4_vs_lib8((j+12), &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12], m-(i+8), n-(j+12));
					if(j>=n-16)
						return;
					if(j<n-20) // 21 - 23
						{
						kernel_spotrf_nt_l_8x8_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
						}
					else // 17 18 19 20
						{
						kernel_spotrf_nt_l_8x4_vs_lib8(j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
						}
					}
				else // 9 10 11 12
					{
					kernel_spotrf_nt_l_16x4_vs_lib8(j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
					}
				}
			else // 1 2 3 4
				{
				kernel_spotrf_nt_l_24x4_vs_lib8(j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
				}
			}
		}
	return;
#endif

	left_16:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
		kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
		}
	if(j<n)
		{
		if(j<i) // dtrsm
			{
			kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
			if(j<n-4) // 5 6 7
				{
				kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
				}
			}
		else // dpotrf
			{
			if(j<n-4) // 5 - 15
				{
				kernel_spotrf_nt_l_16x4_vs_lib8(j+0, &pD[(i+0)*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+j*sdc], sdc, &pD[(j+0)*bs+j*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
				kernel_spotrf_nt_l_12x4_vs_lib8(j+4, &pD[(i+0)*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+j*sdc], sdc, &pD[(j+4)*bs+j*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
				if(j>=n-8)
					return;
				if(j<n-12) // 13 - 15
					{
					kernel_spotrf_nt_l_8x8_vs_lib8((j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
					}
				else // 9 - 12
					{
					kernel_spotrf_nt_l_8x4_vs_lib8((j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
					}
				}
			else // 1 2 3 4
				{
				kernel_spotrf_nt_l_16x4_vs_lib8(j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
				}
			}
		}
	return;

	left_8:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_strsm_nt_rl_inv_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		if(j<i) // dtrsm
			{
			if(j<n-4) // 5 6 7
				{
				kernel_strsm_nt_rl_inv_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
			else // 1 2 3 4
				{
				kernel_strsm_nt_rl_inv_8x4_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
			}
		else // dpotrf
			{
			if(j<n-4) // 5 6 7
				{
				kernel_spotrf_nt_l_8x8_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
			else // 1 2 3 4
				{
				kernel_spotrf_nt_l_8x4_vs_lib8(j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
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



void blasfeo_hp_ssyrk_spotrf_ln_mn(int m, int n, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(ai!=0 | bi!=0 | ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_ssyrk_spotrf_ln_mn(m, n, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_ssyrk_spotrf_ln_mn: feature not implemented yet: ai=%d, bi=%d, ci=%d, di=%d\n", ai, bi, ci, di);
		exit(1);
#endif
		}

	const int bs = 8;

	int i, j;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*bs;
	float *pB = sB->pA + bj*bs;
	float *pC = sC->pA + cj*bs;
	float *pD = sD->pA + dj*bs;
	float *dD = sD->dA; // XXX what to do if di and dj are not zero

//	ssyrk_spotrf_nt_l_lib(m, n, k, pA, sda, pB, sdb, pC, sdc, pD, sdd, dD);

	if(di==0 && dj==0)
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_sgemm_strsm_nt_rl_inv_24x4_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0]);
			kernel_sgemm_strsm_nt_rl_inv_24x4_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4]);
			}
		if(j<n)
			{
			if(i<j) // dtrsm
				{
				kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
				if(j<n-4) // 5 6 7
					{
					kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[(j+4)*bs+(j+4)*sdd], &dD[j+4], m-i, n-(j+4));
					}
				}
			else // dpotrf
				{
				if(j<n-23)
					{
					kernel_ssyrk_spotrf_nt_l_24x4_lib8(k, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], (j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0]);
					kernel_ssyrk_spotrf_nt_l_20x4_lib8(k, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], (j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4]);
					kernel_ssyrk_spotrf_nt_l_16x4_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], (j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8]);
					kernel_ssyrk_spotrf_nt_l_12x4_lib8(k, &pA[(i+8)*sda], sda, &pB[4+(j+8)*sdb], (j+12), &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12]);
					kernel_ssyrk_spotrf_nt_l_8x8_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], (j+16), &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16]);
					}
				else
					{
					if(j<n-4) // 5 - 23
						{
						kernel_ssyrk_spotrf_nt_l_24x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], (j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
						kernel_ssyrk_spotrf_nt_l_20x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], (j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
						if(j==n-8)
							return;
						if(j<n-12) // 13 - 23
							{
							kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], (j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
							kernel_ssyrk_spotrf_nt_l_12x4_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[4+(j+8)*sdb], (j+12), &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12], m-(i+8), n-(j+12));
							if(j==n-16)
								return;
							if(j<n-20) // 21 - 23
								{
								kernel_ssyrk_spotrf_nt_l_8x8_vs_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
								}
							else // 17 18 19 20
								{
								kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
								}
							}
						else // 9 10 11 12
							{
							kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
							}
						}
					else // 1 2 3 4
						{
						kernel_ssyrk_spotrf_nt_l_24x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[j*sdb], j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
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
#else
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_sgemm_strsm_nt_rl_inv_16x4_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0]);
			kernel_sgemm_strsm_nt_rl_inv_16x4_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4]);
			}
		if(j<n)
			{
			if(i<j) // dtrsm
				{
				kernel_sgemm_strsm_nt_rl_inv_16x4_vs_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
				if(j<n-4) // 5 6 7
					{
					kernel_sgemm_strsm_nt_rl_inv_16x4_vs_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[(j+4)*bs+(j+4)*sdd], &dD[j+4], m-i, n-(j+4));
					}
				}
			else // dpotrf
				{
				if(j<n-15)
					{
					kernel_ssyrk_spotrf_nt_l_16x4_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0]);
					kernel_ssyrk_spotrf_nt_l_12x4_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4]);
					kernel_ssyrk_spotrf_nt_l_8x8_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], (j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8]);
					}
				else
					{
					if(j<n-4) // 5 - 15
						{
						kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], (j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
						kernel_ssyrk_spotrf_nt_l_12x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], j+4, &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
						if(j==n-8) // 8
							return;
						if(j<n-12) // 13 - 15
							{
							kernel_ssyrk_spotrf_nt_l_8x8_vs_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
							}
						else // 9 10 11 12
							{
							kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
							}
						}
					else // 1 2 3 4
						{
						kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[j*sdb], j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
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
#endif

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_24:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
		kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
		}
	if(j<n)
		{
		if(j<i) // dtrsm
			{
			kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
			if(j<n-4) // 5 6 7
				{
				kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
				}
			}
		else // dpotrf
			{
			if(j<n-4) // 5 - 23
				{
				kernel_ssyrk_spotrf_nt_l_24x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], (j+0), &pD[(i+0)*sdd], sdd, &pD[(j+0)*sdd], &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
				kernel_ssyrk_spotrf_nt_l_20x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], (j+4), &pD[(i+0)*sdd], sdd, &pD[4+(j+0)*sdd], &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
				if(j>=n-8)
					return;
				if(j<n-12) // 13 - 23
					{
					kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], (j+8), &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
					kernel_ssyrk_spotrf_nt_l_12x4_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[4+(j+8)*sdb], j+12, &pD[(i+8)*sdd], sdd, &pD[4+(j+8)*sdd], &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, &dD[j+12], m-(i+8), n-(j+12));
					if(j>=n-16)
						return;
					if(j<n-20) // 21 - 23
						{
						kernel_ssyrk_spotrf_nt_l_8x8_vs_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
						}
					else // 17 18 19 20
						{
						kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8(k, &pA[(i+16)*sda], &pB[(j+16)*sdb], j+16, &pD[(i+16)*sdd], &pD[(j+16)*sdd], &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], &dD[j+16], m-(i+16), n-(j+16));
						}
					}
				else // 9 10 11 12
					{
					kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], sdd, &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, &dD[j+8], m-(i+8), n-(j+8));
					}
				}
			else // 1 2 3 4
				{
				kernel_ssyrk_spotrf_nt_l_24x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[j*sdb], j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
				}
			}
		}
	return;
#endif

	left_16:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_sgemm_strsm_nt_rl_inv_16x4_vs_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[0+(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
		kernel_sgemm_strsm_nt_rl_inv_16x4_vs_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
		}
	if(j<n)
		{
		if(j<i) // dtrsm
			{
			kernel_sgemm_strsm_nt_rl_inv_16x4_vs_lib8(k, &pA[i*sda], sda, &pB[0+j*sdb], j+0, &pD[i*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, &pD[(j+0)*bs+(j+0)*sdd], &dD[j+0], m-i, n-(j+0));
			if(j<n-4) // 5 6 7
				{
				kernel_sgemm_strsm_nt_rl_inv_16x4_vs_lib8(k, &pA[i*sda], sda, &pB[4+j*sdb], j+4, &pD[i*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, &pD[4+(j+4)*bs+(j+0)*sdd], &dD[j+4], m-i, n-(j+4));
				}
			}
		else // dpotrf
			{
			if(j<n-4) // 5 - 15
				{
				kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[0+j*sdb], j+0, &pD[(i+0)*sdd], sdd, &pD[0+j*sdd], &pC[(j+0)*bs+j*sdc], sdc, &pD[(j+0)*bs+j*sdd], sdd, &dD[j+0], m-(i+0), n-(j+0));
				kernel_ssyrk_spotrf_nt_l_12x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[4+j*sdb], j+4, &pD[(i+0)*sdd], sdd, &pD[4+j*sdd], &pC[(j+4)*bs+j*sdc], sdc, &pD[(j+4)*bs+j*sdd], sdd, &dD[j+4], m-(i+0), n-(j+4));
				if(j>=n-8)
					return;
				if(j<n-12) // 13 - 15
					{
					kernel_ssyrk_spotrf_nt_l_8x8_vs_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], (j+8), &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
					}
				else // 9 - 12
					{
					kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], &dD[j+8], m-(i+8), n-(j+8));
					}
				}
			else // 1 2 3 4
				{
				kernel_ssyrk_spotrf_nt_l_16x4_vs_lib8(k, &pA[(i+0)*sda], sda, &pB[j*sdb], j, &pD[(i+0)*sdd], sdd, &pD[j*sdd], &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, &dD[j], m-(i+0), n-j);
				}
			}
		}
	return;

	left_8:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_sgemm_strsm_nt_rl_inv_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
		}
	if(j<n)
		{
		if(j<i) // dtrsm
			{
			if(j<n-4) // 5 6 7
				{
				kernel_sgemm_strsm_nt_rl_inv_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
			else // 1 2 3 4
				{
				kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
			}
		else // dpotrf
			{
			if(j<n-4) // 5 6 7
				{
				kernel_ssyrk_spotrf_nt_l_8x8_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
			else // 1 2 3 4
				{
				kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], &dD[j], m-i, n-j);
				}
			}
		}
	return;

	}



void blasfeo_hp_ssyrk_spotrf_ln(int m, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_ssyrk_spotrf_ln_mn(m, m, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
	return;
	}



// dgetrf no pivoting
void blasfeo_hp_sgetrf_np(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgetrf_np(m, n, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_sgetf_np: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}




// dgetrf row pivoting
void blasfeo_hp_sgetrf_rp(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, int *ipiv)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgetrf_rp(m, n, sC, ci, cj, sD, di, dj, ipiv);
	return;
#else
	printf("\nblasfeo_sgetrf_rp: feature not implemented yet\n");
	exit(1);
#endif
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
	printf("\nblasfeo_dgelqf_pd_lla: feature not implemented yet\n");
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

