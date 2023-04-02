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



void blasfeo_hp_sgemm_nt(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m==0 | n==0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

#if defined(DIM_CHECK)
	// TODO check that sA=!sD or that if sA==sD then they do not overlap (same for sB)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_sgemm_nt : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_sgemm_nt : n<0 : %d<0 *****\n", n);
	if(k<0) printf("\n****** blasfeo_sgemm_nt : k<0 : %d<0 *****\n", k);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_sgemm_nt : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_sgemm_nt : aj<0 : %d<0 *****\n", aj);
	if(bi<0) printf("\n****** blasfeo_sgemm_nt : bi<0 : %d<0 *****\n", bi);
	if(bj<0) printf("\n****** blasfeo_sgemm_nt : bj<0 : %d<0 *****\n", bj);
	if(ci<0) printf("\n****** blasfeo_sgemm_nt : ci<0 : %d<0 *****\n", ci);
	if(cj<0) printf("\n****** blasfeo_sgemm_nt : cj<0 : %d<0 *****\n", cj);
	if(di<0) printf("\n****** blasfeo_sgemm_nt : di<0 : %d<0 *****\n", di);
	if(dj<0) printf("\n****** blasfeo_sgemm_nt : dj<0 : %d<0 *****\n", dj);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_sgemm_nt : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+k > sA->n) printf("\n***** blasfeo_sgemm_nt : aj+k > col(A) : %d+%d > %d *****\n", aj, k, sA->n);
	// B: n x k
	if(bi+n > sB->m) printf("\n***** blasfeo_sgemm_nt : bi+n > row(B) : %d+%d > %d *****\n", bi, n, sB->m);
	if(bj+k > sB->n) printf("\n***** blasfeo_sgemm_nt : bj+k > col(B) : %d+%d > %d *****\n", bj, k, sB->n);
	// C: m x n
	if(ci+m > sC->m) printf("\n***** blasfeo_sgemm_nt : ci+m > row(C) : %d+%d > %d *****\n", ci, n, sC->m);
	if(cj+n > sC->n) printf("\n***** blasfeo_sgemm_nt : cj+n > col(C) : %d+%d > %d *****\n", cj, k, sC->n);
	// D: m x n
	if(di+m > sD->m) printf("\n***** blasfeo_sgemm_nt : di+m > row(D) : %d+%d > %d *****\n", di, n, sD->m);
	if(dj+n > sD->n) printf("\n***** blasfeo_sgemm_nt : dj+n > col(D) : %d+%d > %d *****\n", dj, k, sD->n);
#endif

	if(ai>0 | bi>0 | ci>0 | di>0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_sgemm_nt(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_sgemm_nt: feature not implemented yet: ai>0, bi>0, ci>0, di>0\n");
		exit(1);
#endif
		}

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*bs;
	float *pB = sB->pA + bj*bs;
	float *pC = sC->pA + cj*bs;
	float *pD = sD->pA + dj*bs;

	int i, j, l;

	// register spil space
//	ALIGNED( float spil[32], 64 );

	i = 0;

#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nt_24x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nt_24x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}
		if(j<n)
			{
			if(j<n-3)
				{
				kernel_sgemm_nt_24x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
				if(j<n-4)
					{
					kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, 8, n-(j+4));
					}
				}
			else
				{
				kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, 8, n-j);
				}
			}
		}
	if(m-i>0)
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
//		else if(m-i<=20)
//			{
//			goto left_20;
//			}
		else
			{
			goto left_24;
			}
		}
#else
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
//			kernel_sgemm_nt_16x8_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, spil);
			kernel_sgemm_nt_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nt_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}
		if(j<n)
			{
			if(j<n-3)
				{
				kernel_sgemm_nt_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
				if(j<n-4)
					{
					kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, 8, n-(j+4));
					}
				}
			else
				{
				kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, 8, n-j);
				}
			}
		}
	if(m-i>0)
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
#endif

	// common return if i==m
	return;

	// clean up loops definitions

	left_24:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, 4);
		kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<n)
		{
		kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-j);
		}
	return;

#if defined(TARGET_X64_INTEL_HASWELL)
	left_20:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, 4);
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
		kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[(i+16)*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(i+16)*sdc], &pD[(j+0)*bs+(i+16)*sdd], m-(i+16), n-j);
		}
	if(j<n)
		{
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-j);
//		kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[(i+16)*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(i+16)*sdc], &pD[(j+0)*bs+(i+16)*sdd], m-(i+16), n-j);
		kernel_sgemm_nt_8x4_vs_lib8(k, &alpha, &pA[(i+16)*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(i+16)*sdc], &pD[(j+0)*bs+(i+16)*sdd], m-(i+16), n-j);
		}
	return;
#endif

	left_16:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, 4);
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<n)
		{
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-j);
		}
	return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
left_12:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib8(k, &alpha, &pA[i*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
		kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(i+8)*sdc], &pD[(j+0)*bs+(i+8)*sdd], m-(i+8), n-j);
		}
	if(j<n)
		{
		kernel_sgemm_nt_8x4_vs_lib8(k, &alpha, &pA[i*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
//		kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(i+8)*sdc], &pD[(j+0)*bs+(i+8)*sdd], m-(i+8), n-j);
		kernel_sgemm_nt_8x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(i+8)*sdc], &pD[(j+0)*bs+(i+8)*sdd], m-(i+8), n-j);
		}
	return;
#endif

	left_8:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib8(k, &alpha, &pA[i*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		kernel_sgemm_nt_8x4_vs_lib8(k, &alpha, &pA[i*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
		}
	return;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	left_4:
	j = 0;
	for(; j<n; j+=8)
		{
//		kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[i*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
		kernel_sgemm_nt_8x4_vs_lib8(k, &alpha, &pA[i*sda], &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
		}
	return;
#endif

	}



void blasfeo_hp_sgemm_nn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m==0 | n==0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_sgemm_nt : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_sgemm_nt : n<0 : %d<0 *****\n", n);
	if(k<0) printf("\n****** blasfeo_sgemm_nt : k<0 : %d<0 *****\n", k);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_sgemm_nt : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_sgemm_nt : aj<0 : %d<0 *****\n", aj);
	if(bi<0) printf("\n****** blasfeo_sgemm_nt : bi<0 : %d<0 *****\n", bi);
	if(bj<0) printf("\n****** blasfeo_sgemm_nt : bj<0 : %d<0 *****\n", bj);
	if(ci<0) printf("\n****** blasfeo_sgemm_nt : ci<0 : %d<0 *****\n", ci);
	if(cj<0) printf("\n****** blasfeo_sgemm_nt : cj<0 : %d<0 *****\n", cj);
	if(di<0) printf("\n****** blasfeo_sgemm_nt : di<0 : %d<0 *****\n", di);
	if(dj<0) printf("\n****** blasfeo_sgemm_nt : dj<0 : %d<0 *****\n", dj);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_sgemm_nn : ai+m > row(A) : %d+%d > %d *****\n\n", ai, m, sA->m);
	if(aj+k > sA->n) printf("\n***** blasfeo_sgemm_nn : aj+k > col(A) : %d+%d > %d *****\n\n", aj, k, sA->n);
	// B: k x n
	if(bi+k > sB->m) printf("\n***** blasfeo_sgemm_nn : bi+k > row(B) : %d+%d > %d *****\n\n", bi, k, sB->m);
	if(bj+n > sB->n) printf("\n***** blasfeo_sgemm_nn : bj+n > col(B) : %d+%d > %d *****\n\n", bj, n, sB->n);
	// C: m x n
	if(ci+m > sC->m) printf("\n***** blasfeo_sgemm_nn : ci+m > row(C) : %d+%d > %d *****\n\n", ci, n, sC->m);
	if(cj+n > sC->n) printf("\n***** blasfeo_sgemm_nn : cj+n > col(C) : %d+%d > %d *****\n\n", cj, k, sC->n);
	// D: m x n
	if(di+m > sD->m) printf("\n***** blasfeo_sgemm_nn : di+m > row(D) : %d+%d > %d *****\n\n", di, n, sD->m);
	if(dj+n > sD->n) printf("\n***** blasfeo_sgemm_nn : dj+n > col(D) : %d+%d > %d *****\n\n", dj, k, sD->n);
#endif

	if(ai>0 | ci>0 | di>0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_sgemm_nn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_sgemm_nn: feature not implemented yet: ai>0, ci>0, di>0\n");
		exit(1);
#endif
		}

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*bs;
	float *pB = sB->pA + bj*bs + bi/bs*bs*sdb;
	float *pC = sC->pA + cj*bs;
	float *pD = sD->pA + dj*bs;

	int offsetB = bi%bs;

	int i, j, l;

	i = 0;

#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nn_24x4_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nn_24x4_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+4)*bs], sdb, &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}
		if(j<n)
			{
			if(j<n-3)
				{
				kernel_sgemm_nn_24x4_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
				if(j<n-4)
					{
					kernel_sgemm_nn_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+4)*bs], sdb, &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, 16, n-(j+4));
					}
				}
			else
				{
				kernel_sgemm_nn_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, 16, n-j);
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
#if 1
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nn_16x4_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nn_16x4_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+4)*bs], sdb, &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}
		if(j<n)
			{
			if(j<n-3)
				{
				kernel_sgemm_nn_16x4_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
				if(j<n-4)
					{
					kernel_sgemm_nn_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+4)*bs], sdb, &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, 16, n-(j+4));
					}
				}
			else
				{
				kernel_sgemm_nn_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, 16, n-j);
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
		for(; j<n-7; j+=8)
			{
#if 1
			kernel_sgemm_nn_8x8_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd]);
#else
			kernel_sgemm_nn_8x4_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd]);
			kernel_sgemm_nn_8x4_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+4)*bs], sdb, &beta, &pC[(j+4)*bs+i*sdc], &pD[(j+4)*bs+i*sdd]);
#endif
			}
		if(j<n)
			{
			if(j<n-3)
				{
				kernel_sgemm_nn_8x4_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd]);
				if(j<n-4)
					{
					kernel_sgemm_nn_8x4_gen_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+4)*bs], sdb, &beta, 0, &pC[(j+4)*bs+i*sdc], sdc, 0, &pD[(j+4)*bs+i*sdd], sdd, 0, 8, 0, n-(j+4));
					}
				}
			else
				{
				kernel_sgemm_nn_8x4_gen_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+0)*bs], sdb, &beta, 0, &pC[(j+0)*bs+i*sdc], sdc, 0, &pD[(j+0)*bs+i*sdd], sdd, 0, 8, 0, n-j);
				}
			}
		}
	if(m>i)
		{
		goto left_8;
		}
#endif
#endif

	// common return if i==m
	return;

#if defined(TARGET_X64_INTEL_HASWELL)
	left_24:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nn_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-j);
		kernel_sgemm_nn_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+4)*bs], sdb, &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<n)
		{
		kernel_sgemm_nn_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-j);
		}
	return;
#endif

	left_16:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nn_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-j);
		kernel_sgemm_nn_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+4)*bs], sdb, &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<n)
		{
		kernel_sgemm_nn_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-j);
		}
	return;

	left_8:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_sgemm_nn_8x8_vs_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		kernel_sgemm_nn_8x4_vs_lib8(k, &alpha, &pA[i*sda], offsetB, &pB[(j+0)*bs], sdb, &beta, &pC[(j+0)*bs+i*sdc], &pD[(j+0)*bs+i*sdd], m-i, n-j);
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
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_llnu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_llnu: feature not implemented yet\n");
	exit(1);
#endif
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
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_lunn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_lunn: feature not implemented yet\n");
	exit(1);
#endif
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

	if(ai!=0 | bi!=0 | di!=0 | alpha!=1.0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsm_rltn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_strsm_rltn: feature not implemented yet: ai=%d, bi=%d, di=%d, alpha=%f\n", ai, bi, di, alpha);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int bs = 8;

	// TODO alpha

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*bs;
	float *pB = sB->pA + bj*bs;
	float *pD = sD->pA + dj*bs;
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

#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_strsm_nt_rl_inv_24x4_lib8(j+0, &pD[i*sdd], sdd, &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], sdb, &pD[(j+0)*bs+i*sdd], sdd, &pA[0+(j+0)*bs+j*sda], &dA[j+0]);
			kernel_strsm_nt_rl_inv_24x4_lib8(j+4, &pD[i*sdd], sdd, &pA[4+j*sda], &pB[(j+4)*bs+i*sdb], sdb, &pD[(j+4)*bs+i*sdd], sdd, &pA[4+(j+4)*bs+j*sda], &dA[j+0]);
			}
		if(n-j>0)
			{
			kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], sdb, &pD[(j+0)*bs+i*sdd], sdd, &pA[0+(j+0)*bs+j*sda], &dA[j+0], m-i, n-j-0);
			if(n-j>4)
				{
				kernel_strsm_nt_rl_inv_24x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pA[4+j*sda], &pB[(j+4)*bs+i*sdb], sdb, &pD[(j+4)*bs+i*sdd], sdd, &pA[4+(j+4)*bs+j*sda], &dA[j+4], m-i, n-j-4);
				}
			}
		}
#endif
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-15; i+=16)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_strsm_nt_rl_inv_16x4_lib8(j+0, &pD[i*sdd], sdd, &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], sdb, &pD[(j+0)*bs+i*sdd], sdd, &pA[0+(j+0)*bs+j*sda], &dA[j+0]);
			kernel_strsm_nt_rl_inv_16x4_lib8(j+4, &pD[i*sdd], sdd, &pA[4+j*sda], &pB[(j+4)*bs+i*sdb], sdb, &pD[(j+4)*bs+i*sdd], sdd, &pA[4+(j+4)*bs+j*sda], &dA[j+0]);
			}
		if(n-j>0)
			{
			kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+0, &pD[i*sdd], sdd, &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], sdb, &pD[(j+0)*bs+i*sdd], sdd, &pA[0+(j+0)*bs+j*sda], &dA[j+0], m-i, n-j-0);
			if(n-j>4)
				{
				kernel_strsm_nt_rl_inv_16x4_vs_lib8(j+4, &pD[i*sdd], sdd, &pA[4+j*sda], &pB[(j+4)*bs+i*sdb], sdb, &pD[(j+4)*bs+i*sdd], sdd, &pA[4+(j+4)*bs+j*sda], &dA[j+4], m-i, n-j-4);
				}
			}
		}
#endif
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_strsm_nt_rl_inv_8x4_lib8(j+0, &pD[i*sdd], &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], &pD[(j+0)*bs+i*sdd], &pA[0+(j+0)*bs+j*sda], &dA[j+0]);
			kernel_strsm_nt_rl_inv_8x4_lib8(j+4, &pD[i*sdd], &pA[4+j*sda], &pB[(j+4)*bs+i*sdb], &pD[(j+4)*bs+i*sdd], &pA[4+(j+4)*bs+j*sda], &dA[j+0]);
			}
		if(n-j>0)
			{
			kernel_strsm_nt_rl_inv_8x4_vs_lib8(j+0, &pD[i*sdd], &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], &pD[(j+0)*bs+i*sdd], &pA[0+(j+0)*bs+j*sda], &dA[j+0], m-i, n-j-0);
			if(n-j>4)
				{
				kernel_strsm_nt_rl_inv_8x4_vs_lib8(j+4, &pD[i*sdd], &pA[4+j*sda], &pB[(j+4)*bs+i*sdb], &pD[(j+4)*bs+i*sdd], &pA[4+(j+4)*bs+j*sda], &dA[j+4], m-i, n-j-4);
				}
			}
		}
	if(m>i)
		{
		goto left_8;
		}

	// common return if i==m
	return;

	left_8:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_strsm_nt_rl_inv_8x4_vs_lib8(j+0, &pD[i*sdd], &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], &pD[(j+0)*bs+i*sdd], &pA[0+(j+0)*bs+j*sda], &dA[j+0], m-i, n-j-0);
		kernel_strsm_nt_rl_inv_8x4_vs_lib8(j+4, &pD[i*sdd], &pA[4+j*sda], &pB[(j+4)*bs+i*sdb], &pD[(j+4)*bs+i*sdd], &pA[4+(j+4)*bs+j*sda], &dA[j+4], m-i, n-j-4);
		}
	if(n-j>0)
		{
		kernel_strsm_nt_rl_inv_8x4_vs_lib8(j+0, &pD[i*sdd], &pA[0+j*sda], &pB[(j+0)*bs+i*sdb], &pD[(j+0)*bs+i*sdd], &pA[0+(j+0)*bs+j*sda], &dA[j+0], m-i, n-j-0);
		}
	return;

	}



// strsm_rltu
void blasfeo_hp_strsm_rltu(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_rltu(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_rltu: feature not implemented yet\n");
	exit(1);
#endif
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
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsm_rutn(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_strsm_rutn: feature not implemented yet\n");
	exit(1);
#endif
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



void blasfeo_hp_ssyrk_ln(int m, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0)
		return;

	if(ci>0 | di>0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_ssyrk_ln(m,  k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_ssyrk_ln: feature not implemented yet: ci>0, di>0\n");
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

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

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i; j+=8)
			{
			kernel_sgemm_nt_24x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nt_24x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}

		kernel_ssyrk_nt_l_24x4_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd);
		kernel_ssyrk_nt_l_20x4_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd);
		kernel_ssyrk_nt_l_16x4_lib8(k, &alpha, &pA[(j+8)*sda], sda, &pB[0+(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd);
		kernel_ssyrk_nt_l_12x4_lib8(k, &alpha, &pA[(j+8)*sda], sda, &pB[4+(j+8)*sdb], &beta, &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd);
		kernel_ssyrk_nt_l_8x8_lib8(k, &alpha, &pA[(j+16)*sda], &pB[0+(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd]);
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
			kernel_sgemm_nt_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nt_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}
		kernel_ssyrk_nt_l_16x4_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd);
		kernel_ssyrk_nt_l_12x4_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[4+(j+0)*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd);
		kernel_ssyrk_nt_l_8x8_lib8(k, &alpha, &pA[(j+8)*sda], &pB[0+(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd]);
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
#endif

	// common return if i==m
	return;

	// clean up loops definitions

#if defined(TARGET_X64_INTEL_HASWELL)
	left_24: // 17 <= m <= 23
	j = 0;
	for(; j<i & j<m-7; j+=8)
		{
		kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, m-(j+0));
		kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, m-(j+4));
		}
	kernel_ssyrk_nt_l_24x4_vs_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, m-(i+0), m-(j+0));
	kernel_ssyrk_nt_l_20x4_vs_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, m-(i+0), m-(j+4));
	kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(j+8)*sda], sda, &pB[0+(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, m-(i+8), m-(j+8));
	kernel_ssyrk_nt_l_12x4_vs_lib8(k, &alpha, &pA[(j+8)*sda], sda, &pB[4+(j+8)*sdb], &beta, &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, m-(i+8), m-(j+12));
	if(j<m-20) // 21 - 23
		{
		kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(j+16)*sda], &pB[0+(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], m-(i+16), m-(j+16));
		}
	else // 17 18 19 20
		{
		kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(j+16)*sda], &pB[0+(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], m-(i+16), m-(j+16));
		}
	return;
#endif

	left_16: // 13 <= m <= 16
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, m-(j+0));
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, m-(j+4));
		}
	kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, m-(i+0), m-(j+0));
	kernel_ssyrk_nt_l_12x4_vs_lib8(k, &alpha, &pA[(j+0)*sda], sda, &pB[4+(j+0)*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, m-(i+0), m-(j+4));
	if(j<m-12) // 13 - 16
		{
		kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(j+8)*sda], &pB[0+(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], m-(i+8), m-(j+8));
		}
	else // 9 - 12
		{
		kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(j+8)*sda], &pB[0+(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], m-(i+8), m-(j+8));
		}
	return;

	left_12: // 9 <= m <= 12
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib8(k, &alpha, &pA[(i+0)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(i+0)*sdc], &pD[(j+0)*bs+(i+0)*sdd], m-(i+0), m-(j+0));
		kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(i+8)*sdc], &pD[(j+0)*bs+(i+8)*sdd], m-(i+0), m-(j+0));
		}
	kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(j+0)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], &pD[(j+0)*bs+(j+0)*sdd], m-(i+0), m-(j+0));
	kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[(j+8)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+8)*sdc], &pD[(j+0)*bs+(j+8)*sdd], m-(i+8), m-(j+0));
	if(j<m-8) // 9 - 12
		{
		kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(j+8)*sda], &pB[0+(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], m-(i+8), m-(j+8));
		}
	return;

	left_8: // 5 <= m <= 8
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib8(k, &alpha, &pA[(i+0)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(i+0)*sdc], &pD[(j+0)*bs+(i+0)*sdd], m-(i+0), m-(j+0));
		}
	if(j<m-4) // 5 - 8
		{
		kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(j+0)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], &pD[(j+0)*bs+(j+0)*sdd], m-(i+0), m-(j+0));
		}
	else // 1 - 4
		{
		kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(j+0)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], &pD[(j+0)*bs+(j+0)*sdd], m-(i+0), m-(j+0));
		}
	return;

	left_4: // 1 <= m <= 4
	j = 0;
	for(; j<i; j+=8)
		{
		kernel_sgemm_nt_4x8_vs_lib8(k, &alpha, &pA[(i+0)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(i+0)*sdc], &pD[(j+0)*bs+(i+0)*sdd], m-(i+0), m-(j+0));
		}
	kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(j+0)*sda], &pB[0+(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], &pD[(j+0)*bs+(j+0)*sdd], m-(i+0), m-(j+0));
	return;

	}



void blasfeo_hp_ssyrk_ln_mn(int m, int n, int k, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0)
		return;

	if(ci>0 | di>0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_ssyrk_ln_mn(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_ssyrk_ln_mn: feature not implemented yet: ci>0, di>0\n");
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

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

	i = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; i<m-23; i+=24)
		{
		j = 0;
		for(; j<i & j<n-7; j+=8)
			{
			kernel_sgemm_nt_24x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nt_24x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}
		if(j<n)
			{
			if(i<j) // sgemm
				{
				kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-(j+0));
				if(j<n-4) // 5 6 7
					{
					kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
					}
				}
			else // ssyrk
				{
				if(j<n-23)
					{
					kernel_ssyrk_nt_l_24x4_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd);
					kernel_ssyrk_nt_l_20x4_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd);
					kernel_ssyrk_nt_l_16x4_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd);
					kernel_ssyrk_nt_l_12x4_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[4+(j+8)*sdb], &beta, &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd);
					kernel_ssyrk_nt_l_8x8_lib8(k, &alpha, &pA[(i+16)*sda], &pB[(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd]);
					}
				else
					{
					if(j<n-4) // 5 - 23
						{
						kernel_ssyrk_nt_l_24x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, m-(i+0), n-(j+0));
						kernel_ssyrk_nt_l_20x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, m-(i+0), n-(j+4));
						if(j==n-8)
							return;
						if(j<n-12) // 13 - 23
							{
							kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, m-(i+8), n-(j+8));
							kernel_ssyrk_nt_l_12x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[4+(j+8)*sdb], &beta, &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, m-(i+8), n-(j+12));
							if(j==n-16)
								return;
							if(j<n-20) // 21 - 23
								{
								kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(i+16)*sda], &pB[(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], m-(i+16), n-(j+16));
								}
							else // 17 18 19 20
								{
								kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(i+16)*sda], &pB[(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], m-(i+16), n-(j+16));
								}
							}
						else // 9 10 11 12
							{
							kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, m-(i+8), n-(j+8));
							}
						}
					else // 1 2 3 4
						{
						kernel_ssyrk_nt_l_24x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[j*sdb], &beta, &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, m-(i+0), n-j);
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
			kernel_sgemm_nt_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd);
			kernel_sgemm_nt_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd);
			}
		if(j<n)
			{
			if(i<j) // sgemm
				{
				kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-(j+0));
				if(j<n-4) // 5 6 7
					{
					kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
					}
				}
			else // ssyrk
				{
				if(j<n-15)
					{
					kernel_ssyrk_nt_l_16x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd);
					kernel_ssyrk_nt_l_12x4_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd);
					kernel_ssyrk_nt_l_8x8_lib8(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd]);
					}
				else
					{
					if(j<n-4) // 5 - 15
						{
						kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, m-(i+0), n-(j+0));
						kernel_ssyrk_nt_l_12x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, m-(i+0), n-(j+4));
						if(j==n-8) // 8
							return;
						if(j<n-12) // 13 - 15
							{
							kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], m-(i+8), n-(j+8));
							}
						else // 9 10 11 12
							{
							kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], m-(i+8), n-(j+8));
							}
						}
					else // 1 2 3 4
						{
						kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[j*sdb], &beta, &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, m-(i+0), n-j);
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
		kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-(j+0));
		kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<n)
		{
		if(j<i) // sgemm
			{
			kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-(j+0));
			if(j<n-4) // 5 6 7
				{
				kernel_sgemm_nt_24x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
				}
			}
		else // ssyrk
			{
			if(j<n-4) // 5 - 23
				{
				kernel_ssyrk_nt_l_24x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[(j+0)*sdb], &beta, &pC[(j+0)*bs+(j+0)*sdc], sdc, &pD[(j+0)*bs+(j+0)*sdd], sdd, m-(i+0), n-(j+0));
				kernel_ssyrk_nt_l_20x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[4+(j+0)*sdb], &beta, &pC[(j+4)*bs+(j+0)*sdc], sdc, &pD[(j+4)*bs+(j+0)*sdd], sdd, m-(i+0), n-(j+4));
				if(j>=n-8)
					return;
				if(j<n-12) // 13 - 23
					{
					kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, m-(i+8), n-(j+8));
					kernel_ssyrk_nt_l_12x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[4+(j+8)*sdb], &beta, &pC[(j+12)*bs+(j+8)*sdc], sdc, &pD[(j+12)*bs+(j+8)*sdd], sdd, m-(i+8), n-(j+12));
					if(j>=n-16)
						return;
					if(j<n-20) // 21 - 23
						{
						kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(i+16)*sda], &pB[(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], m-(i+16), n-(j+16));
						}
					else // 17 18 19 20
						{
						kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(i+16)*sda], &pB[(j+16)*sdb], &beta, &pC[(j+16)*bs+(j+16)*sdc], &pD[(j+16)*bs+(j+16)*sdd], m-(i+16), n-(j+16));
						}
					}
				else // 9 10 11 12
					{
					kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], sda, &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], sdc, &pD[(j+8)*bs+(j+8)*sdd], sdd, m-(i+8), n-(j+8));
					}
				}
			else // 1 2 3 4
				{
				kernel_ssyrk_nt_l_24x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[j*sdb], &beta, &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, m-(i+0), n-j);
				}
			}
		}
	return;
#endif

	left_16:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-(j+0));
		kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
		}
	if(j<n)
		{
		if(j<i) // sgemm
			{
			kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+i*sdc], sdc, &pD[(j+0)*bs+i*sdd], sdd, m-i, n-(j+0));
			if(j<n-4) // 5 6 7
				{
				kernel_sgemm_nt_16x4_vs_lib8(k, &alpha, &pA[i*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+i*sdc], sdc, &pD[(j+4)*bs+i*sdd], sdd, m-i, n-(j+4));
				}
			}
		else // ssyrk
			{
			if(j<n-4) // 5 - 15
				{
				kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[0+j*sdb], &beta, &pC[(j+0)*bs+j*sdc], sdc, &pD[(j+0)*bs+j*sdd], sdd, m-(i+0), n-(j+0));
				kernel_ssyrk_nt_l_12x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[4+j*sdb], &beta, &pC[(j+4)*bs+j*sdc], sdc, &pD[(j+4)*bs+j*sdd], sdd, m-(i+0), n-(j+4));
				if(j>=n-8)
					return;
				if(j<n-12) // 13 - 15
					{
					kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], m-(i+8), n-(j+8));
					}
				else // 9 - 12
					{
					kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[(i+8)*sda], &pB[(j+8)*sdb], &beta, &pC[(j+8)*bs+(j+8)*sdc], &pD[(j+8)*bs+(j+8)*sdd], m-(i+8), n-(j+8));
					}
				}
			else // 1 2 3 4
				{
				kernel_ssyrk_nt_l_16x4_vs_lib8(k, &alpha, &pA[(i+0)*sda], sda, &pB[j*sdb], &beta, &pC[j*bs+j*sdc], sdc, &pD[j*bs+j*sdd], sdd, m-(i+0), n-j);
				}
			}
		}
	return;

	left_8:
	j = 0;
	for(; j<i & j<n-7; j+=8)
		{
		kernel_sgemm_nt_8x8_vs_lib8(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], m-i, n-j);
		}
	if(j<n)
		{
		if(j<i) // sgemm
			{
			if(j<n-4) // 5 6 7
				{
				kernel_sgemm_nt_8x8_vs_lib8(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], m-i, n-j);
				}
			else // 1 2 3 4
				{
				kernel_sgemm_nt_8x4_vs_lib8(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], m-i, n-j);
				}
			}
		else // ssyrk
			{
			if(j<n-4) // 5 6 7
				{
				kernel_ssyrk_nt_l_8x8_vs_lib8(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], m-i, n-j);
				}
			else // 1 2 3 4
				{
				kernel_ssyrk_nt_l_8x4_vs_lib8(k, &alpha, &pA[i*sda], &pB[j*sdb], &beta, &pC[j*bs+j*sdc], &pD[j*bs+j*sdd], m-i, n-j);
				}
			}
		}
	return;

	}



void blasfeo_hp_ssyrk_lt(int m, int k, double alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, double beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
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



void blasfeo_hp_ssyrk_un(int m, int k, double alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, double beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
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



void blasfeo_hp_ssyrk_ut(int m, int k, double alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, double beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
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



// dtrmm_right_lower_nottransposed_notunit (B, i.e. the first matrix, is triangular !!!)
void blasfeo_hp_strmm_rlnn(int m, int n, float alpha, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*bs;
	float *pB = sB->pA + bj*bs;
	float *pD = sD->pA + dj*bs;

	pA += ai/bs*bs*sda;
	pB += bi/bs*bs*sdb;
	int offsetB = bi%bs;
	int di0 = di-ai%bs;
	int offsetD;
	if(di0>=0)
		{
		pD += di0/bs*bs*sdd;
		offsetD = di0%bs;
		}
	else
		{
		pD += -8*sdd;
		offsetD = bs+di0;
		}
	
	int ii, jj;

	int offsetB4;

	if(offsetB<4)
		{
		offsetB4 = offsetB+4;
		ii = 0;
		if(ai%bs!=0)
			{
			jj = 0;
			for(; jj<n-4; jj+=8)
				{
				kernel_strmm_nn_rl_8x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, ai%bs, m-ii, 0, n-jj);
				kernel_strmm_nn_rl_8x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, ai%bs, m-ii, 0, n-jj-4);
				}
			m -= bs-ai%bs;
			pA += bs*sda;
			pD += bs*sdd;
			}
		if(offsetD==0)
			{
#if defined(TARGET_X64_INTEL_HASWELL)
			// XXX create left_24 once the _gen_ kernel exist !!!
			for(; ii<m-23; ii+=24)
				{
				jj = 0;
				for(; jj<n-7; jj+=8)
					{
					kernel_strmm_nn_rl_24x4_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, &pD[ii*sdd+jj*bs], sdd);
					kernel_strmm_nn_rl_24x4_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, &pD[ii*sdd+(jj+4)*bs], sdd);
					}
				if(n-jj>0)
					{
					kernel_strmm_nn_rl_24x4_vs_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, &pD[ii*sdd+jj*bs], sdd, 24, n-jj);
					if(n-jj>4)
						{
						kernel_strmm_nn_rl_24x4_vs_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, &pD[ii*sdd+(jj+4)*bs], sdd, 24, n-jj-4);
						}
					}
				}
#endif
			for(; ii<m-15; ii+=16)
				{
				jj = 0;
				for(; jj<n-7; jj+=8)
					{
					kernel_strmm_nn_rl_16x4_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, &pD[ii*sdd+jj*bs], sdd);
					kernel_strmm_nn_rl_16x4_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, &pD[ii*sdd+(jj+4)*bs], sdd);
					}
				if(n-jj>0)
					{
					kernel_strmm_nn_rl_16x4_vs_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, &pD[ii*sdd+jj*bs], sdd, 16, n-jj);
					if(n-jj>4)
						{
						kernel_strmm_nn_rl_16x4_vs_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, &pD[ii*sdd+(jj+4)*bs], sdd, 16, n-jj-4);
						}
					}
				}
			if(m-ii>0)
				{
				if(m-ii<=8)
					goto left_8;
				else
					goto left_16;
				}
			}
		else
			{
			for(; ii<m-8; ii+=16)
				{
				jj = 0;
				for(; jj<n-4; jj+=8)
					{
					kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
					kernel_strmm_nn_rl_16x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, 0, m-ii, 0, n-jj-4);
					}
				if(n-jj>0)
					{
					kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
					}
				}
			if(m-ii>0)
				goto left_8;
			}
		}
	else
		{
		offsetB4 = offsetB-4;
		ii = 0;
		if(ai%bs!=0)
			{
			jj = 0;
			for(; jj<n-4; jj+=8)
				{
				kernel_strmm_nn_rl_8x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, ai%bs, m-ii, 0, n-jj);
				kernel_strmm_nn_rl_8x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], offsetB4, &pB[(jj+8)*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, ai%bs, m-ii, 0, n-jj-4);
				}
			m -= bs-ai%bs;
			pA += bs*sda;
			pD += bs*sdd;
			}
		if(offsetD==0)
			{
			for(; ii<m-15; ii+=16)
				{
				jj = 0;
				for(; jj<n-7; jj+=8)
					{
					kernel_strmm_nn_rl_16x4_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, &pD[ii*sdd+jj*bs], sdd);
					kernel_strmm_nn_rl_16x4_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[(jj+8)*sdb+(jj+4)*bs], sdb, &pD[ii*sdd+(jj+4)*bs], sdd);
					}
				if(n-jj>0)
					{
					kernel_strmm_nn_rl_16x4_vs_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, &pD[ii*sdd+jj*bs], sdd, 8, n-jj);
					if(n-jj>4)
						{
						kernel_strmm_nn_rl_16x4_vs_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[(jj+8)*sdb+(jj+4)*bs], sdb, &pD[ii*sdd+(jj+4)*bs], sdd, 8, n-jj-4);
						}
					}
				}
			if(m-ii>0)
				{
				if(m-ii<=8)
					goto left_8;
				else
					goto left_16;
				}
			}
		else
			{
			for(; ii<m-8; ii+=16)
				{
				jj = 0;
				for(; jj<n-4; jj+=8)
					{
					kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
					kernel_strmm_nn_rl_16x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[(jj+8)*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, 0, m-ii, 0, n-jj-4);
					}
				if(n-jj>0)
					{
					kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
					}
				}
			if(m-ii>0)
				goto left_8;
			}
		}

	// common return if i==m
	return;

	// clean up loops definitions

	left_16:
	if(offsetB<4)
		{
		jj = 0;
		for(; jj<n-4; jj+=8)
			{
			kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			kernel_strmm_nn_rl_16x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, 0, m-ii, 0, n-jj-4);
			}
		if(n-jj>0)
			{
			kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			}
		}
	else
		{
		jj = 0;
		for(; jj<n-4; jj+=8)
			{
			kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			kernel_strmm_nn_rl_16x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], sda, offsetB4, &pB[(jj+8)*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, 0, m-ii, 0, n-jj-4);
			}
		if(n-jj>0)
			{
			kernel_strmm_nn_rl_16x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], sda, offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			}
		}
	return;

	left_8:
	if(offsetB<4)
		{
		jj = 0;
		for(; jj<n-4; jj+=8)
			{
			kernel_strmm_nn_rl_8x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			kernel_strmm_nn_rl_8x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], offsetB4, &pB[jj*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, 0, m-ii, 0, n-jj-4);
			}
		if(n-jj>0)
			{
			kernel_strmm_nn_rl_8x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			}
		}
	else
		{
		jj = 0;
		for(; jj<n-4; jj+=8)
			{
			kernel_strmm_nn_rl_8x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			kernel_strmm_nn_rl_8x4_gen_lib8(n-jj-4, &alpha, &pA[ii*sda+(jj+4)*bs], offsetB4, &pB[(jj+8)*sdb+(jj+4)*bs], sdb, offsetD, &pD[ii*sdd+(jj+4)*bs], sdd, 0, m-ii, 0, n-jj-4);
			}
		if(n-jj>0)
			{
			kernel_strmm_nn_rl_8x4_gen_lib8(n-jj, &alpha, &pA[ii*sda+jj*bs], offsetB, &pB[jj*sdb+jj*bs], sdb, offsetD, &pD[ii*sdd+jj*bs], sdd, 0, m-ii, 0, n-jj);
			}
		}
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

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*bs;
	float *pB = sB->pA + bj*bs;
	float *pD = sD->pA + dj*bs;

	int i, j;

	float beta = 0.0;
	
	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_strmm_nt_ru_8x4_lib8(n-(j+0), &alpha, &pA[(j+0)*bs+i*sda], &pB[(j+0)*bs+(j+0)*sdb], &pD[(j+0)*bs+i*sdd]);
			kernel_strmm_nt_ru_8x4_lib8(n-(j+4), &alpha, &pA[(j+4)*bs+i*sda], &pB[(j+4)*bs+(j+0)*sdb+4], &pD[(j+4)*bs+i*sdd]);
			}
		if(j<n) // TODO specialized edge routine
			{
			if(j<n-4)
				{
				kernel_strmm_nt_ru_8x4_vs_lib8(n-(j+0), &alpha, &pA[(j+0)*bs+i*sda], &pB[(j+0)*bs+(j+0)*sdb], &pD[(j+0)*bs+i*sdd], m-i, n-(j+0));
				kernel_strmm_nt_ru_8x4_vs_lib8(n-(j+4), &alpha, &pA[(j+4)*bs+i*sda], &pB[(j+4)*bs+(j+0)*sdb+4], &pD[(j+4)*bs+i*sdd], m-i, n-(j+4));
				}
			else
				{
				kernel_strmm_nt_ru_8x4_vs_lib8(n-j, &alpha, &pA[j*bs+i*sda], &pB[j*bs+j*sdb], &pD[j*bs+i*sdd], m-i, n-j);
				}
			}
		}
	if(i<m)
		{
		goto left_8;
		}
	
	// common return
	return;

	left_8:
	j = 0;
	for(; j<n-4; j+=8)
		{
		kernel_strmm_nt_ru_8x4_vs_lib8(n-(j+0), &alpha, &pA[(j+0)*bs+i*sda], &pB[(j+0)*bs+(j+0)*sdb], &pD[(j+0)*bs+i*sdd], m-i, n-(j+0));
		kernel_strmm_nt_ru_8x4_vs_lib8(n-(j+4), &alpha, &pA[(j+4)*bs+i*sda], &pB[(j+4)*bs+(j+0)*sdb+4], &pD[(j+4)*bs+i*sdd], m-i, n-(j+4));
		}
	if(j<n) // TODO specialized edge routine
		{
		kernel_strmm_nt_ru_8x4_vs_lib8(n-j, &alpha, &pA[j*bs+i*sda], &pB[j*bs+j*sdb], &pD[j*bs+i*sdd], m-i, n-j);
		}

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



