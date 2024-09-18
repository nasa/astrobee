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
#if defined(BLASFEO_REF_API)
#include <blasfeo_s_blasfeo_ref_api.h>
#endif



// dgemm with A diagonal matrix (stored as strvec)
void blasfeo_hp_sgemm_dn(int m, int n, float alpha, struct blasfeo_svec *sA, int ai, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 | n<=0)
		return;

	if(bi!=0 | ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_sgemm_dn(m, n, alpha, sA, ai, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_sgemm_dn: feature not implemented yet: bi=%d, ci=%d, di=%d\n", bi, ci, di);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int bs = 4;

	int sdb = sB->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *dA = sA->pa + ai;
	float *pB = sB->pA + bj*bs;
	float *pC = sC->pA + cj*bs;
	float *pD = sD->pA + dj*bs;

//	sgemm_diag_left_lib(m, n, alpha, dA, pB, sdb, beta, pC, sdc, pD, sdd);
	int ii;

	ii = 0;
	if(beta==0.0)
		{
		for( ; ii<m-3; ii+=4)
			{
			kernel_sgemm_diag_left_4_a0_lib4(n, &alpha, &dA[ii], &pB[ii*sdb], &pD[ii*sdd]);
			}
		}
	else
		{
		for( ; ii<m-3; ii+=4)
			{
			kernel_sgemm_diag_left_4_lib4(n, &alpha, &dA[ii], &pB[ii*sdb], &beta, &pC[ii*sdc], &pD[ii*sdd]);
			}
		}
	if(m-ii>0)
		{
		if(m-ii==1)
			kernel_sgemm_diag_left_1_lib4(n, &alpha, &dA[ii], &pB[ii*sdb], &beta, &pC[ii*sdc], &pD[ii*sdd]);
		else if(m-ii==2)
			kernel_sgemm_diag_left_2_lib4(n, &alpha, &dA[ii], &pB[ii*sdb], &beta, &pC[ii*sdc], &pD[ii*sdd]);
		else // if(m-ii==3)
			kernel_sgemm_diag_left_3_lib4(n, &alpha, &dA[ii], &pB[ii*sdb], &beta, &pC[ii*sdc], &pD[ii*sdd]);
		}
	
	return;

	}



// dgemm with B diagonal matrix (stored as strvec)
void blasfeo_hp_sgemm_nd(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sB, int bi, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{

	if(m<=0 | n<=0)
		return;

	if(ai!=0 | ci!=0 | di!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_sgemm_nd(m, n, alpha, sA, ai, aj, sB, bi, beta, sC, ci, cj, sD, di, dj);
		return;
#else
		printf("\nblasfeo_sgemm_nd: feature not implemented yet: ai=%d, ci=%d, di=%d\n", ai, ci, di);
		exit(1);
#endif
		}

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	const int bs = 4;

	int sda = sA->cn;
	int sdc = sC->cn;
	int sdd = sD->cn;
	float *pA = sA->pA + aj*bs;
	float *dB = sB->pa + bi;
	float *pC = sC->pA + cj*bs;
	float *pD = sD->pA + dj*bs;

	int ii;

	ii = 0;
	if(beta==0.0)
		{
		for( ; ii<n-3; ii+=4)
			{
			kernel_sgemm_diag_right_4_a0_lib4(m, &alpha, &pA[ii*bs], sda, &dB[ii], &pD[ii*bs], sdd);
			}
		}
	else
		{
		for( ; ii<n-3; ii+=4)
			{
			kernel_sgemm_diag_right_4_lib4(m, &alpha, &pA[ii*bs], sda, &dB[ii], &beta, &pC[ii*bs], sdc, &pD[ii*bs], sdd);
			}
		}
	if(n-ii>0)
		{
		if(n-ii==1)
			kernel_sgemm_diag_right_1_lib4(m, &alpha, &pA[ii*bs], sda, &dB[ii], &beta, &pC[ii*bs], sdc, &pD[ii*bs], sdd);
		else if(n-ii==2)
			kernel_sgemm_diag_right_2_lib4(m, &alpha, &pA[ii*bs], sda, &dB[ii], &beta, &pC[ii*bs], sdc, &pD[ii*bs], sdd);
		else // if(n-ii==3)
			kernel_sgemm_diag_right_3_lib4(m, &alpha, &pA[ii*bs], sda, &dB[ii], &beta, &pC[ii*bs], sdc, &pD[ii*bs], sdd);
		}
		return;

	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_sgemm_dn(int m, int n, float alpha, struct blasfeo_svec *sA, int ai, struct blasfeo_smat *sB, int bi, int bj, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_dn(m, n, alpha, sA, ai, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}



void blasfeo_sgemm_nd(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sB, int bi, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sgemm_nd(m, n, alpha, sA, ai, aj, sB, bi, beta, sC, ci, cj, sD, di, dj);
	}



#endif
