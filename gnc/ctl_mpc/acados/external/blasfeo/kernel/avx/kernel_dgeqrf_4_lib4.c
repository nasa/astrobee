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

#include <math.h>
#include <stdio.h>

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX

#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>



void kernel_dgeqrf_4_lib4(int m, double *pD, int sdd, double *dD)
	{
	int ii, jj, ll;
	double alpha, beta, tmp, w1, w2, w3;
	const int ps = 4;
	// first column
	beta = 0.0;
	ii = 1;
	if(m>1)
		{
		tmp = pD[1+ps*0];
		beta += tmp*tmp;
		if(m>2)
			{
			tmp = pD[2+ps*0];
			beta += tmp*tmp;
			if(m>3)
				{
				tmp = pD[3+ps*0];
				beta += tmp*tmp;
				}
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		tmp = pD[0+ii*sdd+ps*0];
		beta += tmp*tmp;
		tmp = pD[1+ii*sdd+ps*0];
		beta += tmp*tmp;
		tmp = pD[2+ii*sdd+ps*0];
		beta += tmp*tmp;
		tmp = pD[3+ii*sdd+ps*0];
		beta += tmp*tmp;
		}
	for(ll=0; ll<m-ii; ll++)
		{
		tmp = pD[ll+ii*sdd+ps*0];
		beta += tmp*tmp;
		}
	if(beta==0.0)
		{
		// tau
		dD[0] = 0.0;
		}
	else
		{
		alpha = pD[0+ps*0];
		beta += alpha*alpha;
		beta = sqrt(beta);
		if(alpha>0)
			beta = -beta;
		// tau0
		dD[0] = (beta-alpha) / beta;
		tmp = 1.0 / (alpha-beta);
		// compute v0
		pD[0+ps*0] = beta;
		ii = 1;
		if(m>1)
			{
			pD[1+ps*0] *= tmp;
			if(m>2)
				{
				pD[2+ps*0] *= tmp;
				if(m>3)
					{
					pD[3+ps*0] *= tmp;
					}
				}
			}
		for(ii=4; ii<m-3; ii+=4)
			{
			pD[0+ii*sdd+ps*0] *= tmp;
			pD[1+ii*sdd+ps*0] *= tmp;
			pD[2+ii*sdd+ps*0] *= tmp;
			pD[3+ii*sdd+ps*0] *= tmp;
			}
		for(ll=0; ll<m-ii; ll++)
			{
			pD[ll+ii*sdd+ps*0] *= tmp;
			}
		}
	// gemv_t & ger
	w1 = pD[0+ps*1];
	w2 = pD[0+ps*2];
	w3 = pD[0+ps*3];
	if(m>1)
		{
		w1 += pD[1+ps*1] * pD[1+ps*0];
		w2 += pD[1+ps*2] * pD[1+ps*0];
		w3 += pD[1+ps*3] * pD[1+ps*0];
		if(m>2)
			{
			w1 += pD[2+ps*1] * pD[2+ps*0];
			w2 += pD[2+ps*2] * pD[2+ps*0];
			w3 += pD[2+ps*3] * pD[2+ps*0];
			if(m>3)
				{
				w1 += pD[3+ps*1] * pD[3+ps*0];
				w2 += pD[3+ps*2] * pD[3+ps*0];
				w3 += pD[3+ps*3] * pD[3+ps*0];
				}
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		w1 += pD[0+ii*sdd+ps*1] * pD[0+ii*sdd+ps*0];
		w2 += pD[0+ii*sdd+ps*2] * pD[0+ii*sdd+ps*0];
		w3 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*0];
		w1 += pD[1+ii*sdd+ps*1] * pD[1+ii*sdd+ps*0];
		w2 += pD[1+ii*sdd+ps*2] * pD[1+ii*sdd+ps*0];
		w3 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*0];
		w1 += pD[2+ii*sdd+ps*1] * pD[2+ii*sdd+ps*0];
		w2 += pD[2+ii*sdd+ps*2] * pD[2+ii*sdd+ps*0];
		w3 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*0];
		w1 += pD[3+ii*sdd+ps*1] * pD[3+ii*sdd+ps*0];
		w2 += pD[3+ii*sdd+ps*2] * pD[3+ii*sdd+ps*0];
		w3 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*0];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		w1 += pD[ll+ii*sdd+ps*1] * pD[ll+ii*sdd+ps*0];
		w2 += pD[ll+ii*sdd+ps*2] * pD[ll+ii*sdd+ps*0];
		w3 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*0];
		}
	w1 = - dD[0] * w1;
	w2 = - dD[0] * w2;
	w3 = - dD[0] * w3;
	pD[0+ps*1] += w1;
	pD[0+ps*2] += w2;
	pD[0+ps*3] += w3;
	if(m>1)
		{
		pD[1+ps*1] += w1 * pD[1+ps*0];
		pD[1+ps*2] += w2 * pD[1+ps*0];
		pD[1+ps*3] += w3 * pD[1+ps*0];
		if(m>2)
			{
			pD[2+ps*1] += w1 * pD[2+ps*0];
			pD[2+ps*2] += w2 * pD[2+ps*0];
			pD[2+ps*3] += w3 * pD[2+ps*0];
			if(m>3)
				{
				pD[3+ps*1] += w1 * pD[3+ps*0];
				pD[3+ps*2] += w2 * pD[3+ps*0];
				pD[3+ps*3] += w3 * pD[3+ps*0];
				}
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		pD[0+ii*sdd+ps*1] += w1 * pD[0+ii*sdd+ps*0];
		pD[0+ii*sdd+ps*2] += w2 * pD[0+ii*sdd+ps*0];
		pD[0+ii*sdd+ps*3] += w3 * pD[0+ii*sdd+ps*0];
		pD[1+ii*sdd+ps*1] += w1 * pD[1+ii*sdd+ps*0];
		pD[1+ii*sdd+ps*2] += w2 * pD[1+ii*sdd+ps*0];
		pD[1+ii*sdd+ps*3] += w3 * pD[1+ii*sdd+ps*0];
		pD[2+ii*sdd+ps*1] += w1 * pD[2+ii*sdd+ps*0];
		pD[2+ii*sdd+ps*2] += w2 * pD[2+ii*sdd+ps*0];
		pD[2+ii*sdd+ps*3] += w3 * pD[2+ii*sdd+ps*0];
		pD[3+ii*sdd+ps*1] += w1 * pD[3+ii*sdd+ps*0];
		pD[3+ii*sdd+ps*2] += w2 * pD[3+ii*sdd+ps*0];
		pD[3+ii*sdd+ps*3] += w3 * pD[3+ii*sdd+ps*0];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		pD[ll+ii*sdd+ps*1] += w1 * pD[ll+ii*sdd+ps*0];
		pD[ll+ii*sdd+ps*2] += w2 * pD[ll+ii*sdd+ps*0];
		pD[ll+ii*sdd+ps*3] += w3 * pD[ll+ii*sdd+ps*0];
		}
	if(m==1)
		return;
	// second column
	beta = 0.0;
	if(m>2)
		{
		tmp = pD[2+ps*1];
		beta += tmp*tmp;
		if(m>3)
			{
			tmp = pD[3+ps*1];
			beta += tmp*tmp;
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		tmp = pD[0+ii*sdd+ps*1];
		beta += tmp*tmp;
		tmp = pD[1+ii*sdd+ps*1];
		beta += tmp*tmp;
		tmp = pD[2+ii*sdd+ps*1];
		beta += tmp*tmp;
		tmp = pD[3+ii*sdd+ps*1];
		beta += tmp*tmp;
		}
	for(ll=0; ll<m-ii; ll++)
		{
		tmp = pD[ll+ii*sdd+ps*1];
		beta += tmp*tmp;
		}
	if(beta==0.0)
		{
		// tau
		dD[1] = 0.0;
		}
	else
		{
		alpha = pD[1+ps*1];
		beta += alpha*alpha;
		beta = sqrt(beta);
		if(alpha>0)
			beta = -beta;
		// tau0
		dD[1] = (beta-alpha) / beta;
		tmp = 1.0 / (alpha-beta);
		// compute v0
		pD[1+ps*1] = beta;
		if(m>2)
			{
			pD[2+ps*1] *= tmp;
			if(m>3)
				{
				pD[3+ps*1] *= tmp;
				}
			}
		for(ii=4; ii<m-3; ii+=4)
			{
			pD[0+ii*sdd+ps*1] *= tmp;
			pD[1+ii*sdd+ps*1] *= tmp;
			pD[2+ii*sdd+ps*1] *= tmp;
			pD[3+ii*sdd+ps*1] *= tmp;
			}
		for(ll=0; ll<m-ii; ll++)
			{
			pD[ll+ii*sdd+ps*1] *= tmp;
			}
		}
	// gemv_t & ger
	w2 = pD[1+ps*2];
	w3 = pD[1+ps*3];
	if(m>2)
		{
		w2 += pD[2+ps*2] * pD[2+ps*1];
		w3 += pD[2+ps*3] * pD[2+ps*1];
		if(m>3)
			{
			w2 += pD[3+ps*2] * pD[3+ps*1];
			w3 += pD[3+ps*3] * pD[3+ps*1];
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		w2 += pD[0+ii*sdd+ps*2] * pD[0+ii*sdd+ps*1];
		w3 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*1];
		w2 += pD[1+ii*sdd+ps*2] * pD[1+ii*sdd+ps*1];
		w3 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*1];
		w2 += pD[2+ii*sdd+ps*2] * pD[2+ii*sdd+ps*1];
		w3 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*1];
		w2 += pD[3+ii*sdd+ps*2] * pD[3+ii*sdd+ps*1];
		w3 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*1];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		w2 += pD[ll+ii*sdd+ps*2] * pD[ll+ii*sdd+ps*1];
		w3 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*1];
		}
	w2 = - dD[1] * w2;
	w3 = - dD[1] * w3;
	pD[1+ps*2] += w2;
	pD[1+ps*3] += w3;
	if(m>2)
		{
		pD[2+ps*2] += w2 * pD[2+ps*1];
		pD[2+ps*3] += w3 * pD[2+ps*1];
		if(m>3)
			{
			pD[3+ps*2] += w2 * pD[3+ps*1];
			pD[3+ps*3] += w3 * pD[3+ps*1];
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		pD[0+ii*sdd+ps*2] += w2 * pD[0+ii*sdd+ps*1];
		pD[0+ii*sdd+ps*3] += w3 * pD[0+ii*sdd+ps*1];
		pD[1+ii*sdd+ps*2] += w2 * pD[1+ii*sdd+ps*1];
		pD[1+ii*sdd+ps*3] += w3 * pD[1+ii*sdd+ps*1];
		pD[2+ii*sdd+ps*2] += w2 * pD[2+ii*sdd+ps*1];
		pD[2+ii*sdd+ps*3] += w3 * pD[2+ii*sdd+ps*1];
		pD[3+ii*sdd+ps*2] += w2 * pD[3+ii*sdd+ps*1];
		pD[3+ii*sdd+ps*3] += w3 * pD[3+ii*sdd+ps*1];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		pD[ll+ii*sdd+ps*2] += w2 * pD[ll+ii*sdd+ps*1];
		pD[ll+ii*sdd+ps*3] += w3 * pD[ll+ii*sdd+ps*1];
		}
	if(m==2)
		return;
	// third column
	beta = 0.0;
	if(m>3)
		{
		tmp = pD[3+ps*2];
		beta += tmp*tmp;
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		tmp = pD[0+ii*sdd+ps*2];
		beta += tmp*tmp;
		tmp = pD[1+ii*sdd+ps*2];
		beta += tmp*tmp;
		tmp = pD[2+ii*sdd+ps*2];
		beta += tmp*tmp;
		tmp = pD[3+ii*sdd+ps*2];
		beta += tmp*tmp;
		}
	for(ll=0; ll<m-ii; ll++)
		{
		tmp = pD[ll+ii*sdd+ps*2];
		beta += tmp*tmp;
		}
	if(beta==0.0)
		{
		// tau
		dD[2] = 0.0;
		}
	else
		{
		alpha = pD[2+ps*2];
		beta += alpha*alpha;
		beta = sqrt(beta);
		if(alpha>0)
			beta = -beta;
		// tau0
		dD[2] = (beta-alpha) / beta;
		tmp = 1.0 / (alpha-beta);
		// compute v0
		pD[2+ps*2] = beta;
		if(m>3)
			{
			pD[3+ps*2] *= tmp;
			}
		for(ii=4; ii<m-3; ii+=4)
			{
			pD[0+ii*sdd+ps*2] *= tmp;
			pD[1+ii*sdd+ps*2] *= tmp;
			pD[2+ii*sdd+ps*2] *= tmp;
			pD[3+ii*sdd+ps*2] *= tmp;
			}
		for(ll=0; ll<m-ii; ll++)
			{
			pD[ll+ii*sdd+ps*2] *= tmp;
			}
		}
	// gemv_t & ger
	w3 = pD[2+ps*3];
	if(m>3)
		{
		w3 += pD[3+ps*3] * pD[3+ps*2];
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		w3 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*2];
		w3 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*2];
		w3 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*2];
		w3 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*2];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		w3 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*2];
		}
	w3 = - dD[2] * w3;
	pD[2+ps*3] += w3;
	if(m>3)
		{
		pD[3+ps*3] += w3 * pD[3+ps*2];
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		pD[0+ii*sdd+ps*3] += w3 * pD[0+ii*sdd+ps*2];
		pD[1+ii*sdd+ps*3] += w3 * pD[1+ii*sdd+ps*2];
		pD[2+ii*sdd+ps*3] += w3 * pD[2+ii*sdd+ps*2];
		pD[3+ii*sdd+ps*3] += w3 * pD[3+ii*sdd+ps*2];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		pD[ll+ii*sdd+ps*3] += w3 * pD[ll+ii*sdd+ps*2];
		}
	if(m==3)
		return;
	// fourth column
	beta = 0.0;
	for(ii=4; ii<m-3; ii+=4)
		{
		tmp = pD[0+ii*sdd+ps*3];
		beta += tmp*tmp;
		tmp = pD[1+ii*sdd+ps*3];
		beta += tmp*tmp;
		tmp = pD[2+ii*sdd+ps*3];
		beta += tmp*tmp;
		tmp = pD[3+ii*sdd+ps*3];
		beta += tmp*tmp;
		}
	for(ll=0; ll<m-ii; ll++)
		{
		tmp = pD[ll+ii*sdd+ps*3];
		beta += tmp*tmp;
		}
	if(beta==0.0)
		{
		// tau
		dD[3] = 0.0;
		}
	else
		{
		alpha = pD[3+ps*3];
		beta += alpha*alpha;
		beta = sqrt(beta);
		if(alpha>0)
			beta = -beta;
		// tau0
		dD[3] = (beta-alpha) / beta;
		tmp = 1.0 / (alpha-beta);
		// compute v0
		pD[3+ps*3] = beta;
		for(ii=4; ii<m-3; ii+=4)
			{
			pD[0+ii*sdd+ps*3] *= tmp;
			pD[1+ii*sdd+ps*3] *= tmp;
			pD[2+ii*sdd+ps*3] *= tmp;
			pD[3+ii*sdd+ps*3] *= tmp;
			}
		for(ll=0; ll<m-ii; ll++)
			{
			pD[ll+ii*sdd+ps*3] *= tmp;
			}
		}
	return;
	}


// unblocked algorithm
void kernel_dgeqrf_vs_lib4(int m, int n, int k, int offD, double *pD, int sdd, double *dD)
	{
	if(m<=0 | n<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0, kmax, kmax0;
	const int ps = 4;
	imax = k;//m<n ? m : n;
	double alpha, beta, tmp, w0;
	double *pC00, *pC10, *pC01, *pC11;
	int offset;
	double *pD0 = pD-offD;
	for(ii=0; ii<imax; ii++)
		{
		pC00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		pC10 = &pD0[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
		beta = 0.0;
		jmax = m-ii-1;
		jmax0 = (ps-((ii+1+offD)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		offset = 0;
		jj = 0;
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				tmp = pC10[0+offset];
				beta += tmp*tmp;
				offset += 1;
				}
			offset += -ps+ps*sdd;
			}
		for( ; jj<jmax-3; jj+=4)
			{
			tmp = pC10[0+offset];
			beta += tmp*tmp;
			tmp = pC10[1+offset];
			beta += tmp*tmp;
			tmp = pC10[2+offset];
			beta += tmp*tmp;
			tmp = pC10[3+offset];
			beta += tmp*tmp;
			offset += ps*sdd;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			tmp = pC10[0+offset];
			beta += tmp*tmp;
			offset += 1;
			}
		if(beta==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pC00[0];
			beta += alpha*alpha;
			beta = sqrt(beta);
			if(alpha>0)
				beta = -beta;
			dD[ii] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			offset = 0;
			jj = 0;
			if(jmax0>0)
				{
				for( ; jj<jmax0; jj++)
					{
					pC10[0+offset] *= tmp;
					offset += 1;
					}
				offset += -ps+ps*sdd;
				}
			for( ; jj<jmax-3; jj+=4)
				{
				pC10[0+offset] *= tmp;
				pC10[1+offset] *= tmp;
				pC10[2+offset] *= tmp;
				pC10[3+offset] *= tmp;
				offset += ps*sdd;
				}
			for(ll=0; ll<jmax-jj; ll++)
				{
				pC10[0+offset] *= tmp;
				offset += 1;
				}
			pC00[0] = beta;
			}
		if(ii<n)
			{
			pC01 = pC00 + ps;
			pC11 = pC10 + ps;
			kmax = jmax;
			kmax0 = jmax0;
			jmax = n-ii-1;
			jj = 0;
			for( ; jj<jmax; jj++)
				{
				w0 = pC01[0+ps*jj] * 1.0;
				offset = 0;
				kk = 0;
				if(kmax0>0)
					{
					for( ; kk<kmax0; kk++)
						{
						w0 += pC11[0+offset+ps*jj] * pC10[0+offset];
						offset += 1;
						}
					offset += -ps+ps*sdd;
					}
				for( ; kk<kmax-3; kk+=4)
					{
					w0 += pC11[0+offset+ps*jj] * pC10[0+offset];
					w0 += pC11[1+offset+ps*jj] * pC10[1+offset];
					w0 += pC11[2+offset+ps*jj] * pC10[2+offset];
					w0 += pC11[3+offset+ps*jj] * pC10[3+offset];
					offset += ps*sdd;
					}
				for(ll=0; ll<kmax-kk; ll++)
					{
					w0 += pC11[0+offset+ps*jj] * pC10[0+offset];
					offset += 1;
					}
				w0 = - dD[ii] * w0;
				pC01[0+ps*jj] += w0;
				offset = 0;
				kk = 0;
				if(kmax0>0)
					{
					for( ; kk<kmax0; kk++)
						{
						pC11[0+offset+ps*jj] += w0 * pC10[0+offset];
						offset += 1;
						}
					offset = offset-ps+ps*sdd;
					}
				for( ; kk<kmax-3; kk+=4)
					{
					pC11[0+offset+ps*jj] += w0 * pC10[0+offset];
					pC11[1+offset+ps*jj] += w0 * pC10[1+offset];
					pC11[2+offset+ps*jj] += w0 * pC10[2+offset];
					pC11[3+offset+ps*jj] += w0 * pC10[3+offset];
					offset += ps*sdd;
					}
				for(ll=0; ll<kmax-kk; ll++)
					{
					pC11[0+offset+ps*jj] += w0 * pC10[0+offset];
					offset += 1;
					}
				}
			}
		}
	return;
	}



void kernel_dlarf_4_lib4(int m, int n, double *pD, int sdd, double *dD, double *pC0, int sdc)
	{
	if(m<=0 | n<=0)
		return;
	int ii, jj, ll;
	const int ps = 4;
	double v10,
	       v20, v21,
		   v30, v31, v32;
	double tmp, d0, d1, d2, d3;
	double *pC;
	double pT[16];// = {};
	int ldt = 4;
	double pW[8];// = {};
	int ldw = 2;
	// dot product of v
	v10 = 0.0;
	v20 = 0.0;
	v30 = 0.0;
	v21 = 0.0;
	v31 = 0.0;
	v32 = 0.0;
	if(m>1)
		{
		v10 = 1.0 * pD[1+ps*0];
		if(m>2)
			{
			v10 += pD[2+ps*1] * pD[2+ps*0];
			v20 = 1.0 * pD[2+ps*0];
			v21 = 1.0 * pD[2+ps*1];
			if(m>3)
				{
				v10 += pD[3+ps*1] * pD[3+ps*0];
				v20 += pD[3+ps*2] * pD[3+ps*0];
				v21 += pD[3+ps*2] * pD[3+ps*1];
				v30 = 1.0 * pD[3+ps*0];
				v31 = 1.0 * pD[3+ps*1];
				v32 = 1.0 * pD[3+ps*2];
				}
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		v10 += pD[0+ii*sdd+ps*1] * pD[0+ii*sdd+ps*0];
		v20 += pD[0+ii*sdd+ps*2] * pD[0+ii*sdd+ps*0];
		v21 += pD[0+ii*sdd+ps*2] * pD[0+ii*sdd+ps*1];
		v30 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*0];
		v31 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*1];
		v32 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*2];
		v10 += pD[1+ii*sdd+ps*1] * pD[1+ii*sdd+ps*0];
		v20 += pD[1+ii*sdd+ps*2] * pD[1+ii*sdd+ps*0];
		v21 += pD[1+ii*sdd+ps*2] * pD[1+ii*sdd+ps*1];
		v30 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*0];
		v31 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*1];
		v32 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*2];
		v10 += pD[2+ii*sdd+ps*1] * pD[2+ii*sdd+ps*0];
		v20 += pD[2+ii*sdd+ps*2] * pD[2+ii*sdd+ps*0];
		v21 += pD[2+ii*sdd+ps*2] * pD[2+ii*sdd+ps*1];
		v30 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*0];
		v31 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*1];
		v32 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*2];
		v10 += pD[3+ii*sdd+ps*1] * pD[3+ii*sdd+ps*0];
		v20 += pD[3+ii*sdd+ps*2] * pD[3+ii*sdd+ps*0];
		v21 += pD[3+ii*sdd+ps*2] * pD[3+ii*sdd+ps*1];
		v30 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*0];
		v31 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*1];
		v32 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*2];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		v10 += pD[ll+ii*sdd+ps*1] * pD[ll+ii*sdd+ps*0];
		v20 += pD[ll+ii*sdd+ps*2] * pD[ll+ii*sdd+ps*0];
		v21 += pD[ll+ii*sdd+ps*2] * pD[ll+ii*sdd+ps*1];
		v30 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*0];
		v31 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*1];
		v32 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*2];
		}
	// compute lower triangular T containing tau for matrix update
	pT[0+ldt*0] = dD[0];
	pT[1+ldt*1] = dD[1];
	pT[2+ldt*2] = dD[2];
	pT[3+ldt*3] = dD[3];
	pT[1+ldt*0] = - dD[1] * (v10*pT[0+ldt*0]);
	pT[2+ldt*1] = - dD[2] * (v21*pT[1+ldt*1]);
	pT[3+ldt*2] = - dD[3] * (v32*pT[2+ldt*2]);
	pT[2+ldt*0] = - dD[2] * (v20*pT[0+ldt*0] + v21*pT[1+ldt*0]);
	pT[3+ldt*1] = - dD[3] * (v31*pT[1+ldt*1] + v32*pT[2+ldt*1]);
	pT[3+ldt*0] = - dD[3] * (v30*pT[0+ldt*0] + v31*pT[1+ldt*0] + v32*pT[2+ldt*0]);
	// downgrade matrix
	pW[0] = 0.0;
	pW[1] = 0.0;
	pW[2] = 0.0;
	pW[3] = 0.0;
	pW[4] = 0.0;
	pW[5] = 0.0;
	pW[6] = 0.0;
	pW[7] = 0.0;
	ii = 0;
	for( ; ii<n-1; ii+=2)
		{
		pC = pC0+ii*ps;
		// compute W^T = C^T * V
		tmp = pC[0+ps*0];
		pW[0+ldw*0] = tmp;
		tmp = pC[0+ps*1];
		pW[1+ldw*0] = tmp;
		if(m>1)
			{
			d0 = pD[1+ps*0];
			tmp = pC[1+ps*0];
			pW[0+ldw*0] += tmp * d0;
			pW[0+ldw*1] = tmp;
			tmp = pC[1+ps*1];
			pW[1+ldw*0] += tmp * d0;
			pW[1+ldw*1] = tmp;
			if(m>2)
				{
				d0 = pD[2+ps*0];
				d1 = pD[2+ps*1];
				tmp = pC[2+ps*0];
				pW[0+ldw*0] += tmp * d0;
				pW[0+ldw*1] += tmp * d1;
				pW[0+ldw*2] = tmp;
				tmp = pC[2+ps*1];
				pW[1+ldw*0] += tmp * d0;
				pW[1+ldw*1] += tmp * d1;
				pW[1+ldw*2] = tmp;
				if(m>3)
					{
					d0 = pD[3+ps*0];
					d1 = pD[3+ps*1];
					d2 = pD[3+ps*2];
					tmp = pC[3+ps*0];
					pW[0+ldw*0] += tmp * d0;
					pW[0+ldw*1] += tmp * d1;
					pW[0+ldw*2] += tmp * d2;
					pW[0+ldw*3] = tmp;
					tmp = pC[3+ps*1];
					pW[1+ldw*0] += tmp * d0;
					pW[1+ldw*1] += tmp * d1;
					pW[1+ldw*2] += tmp * d2;
					pW[1+ldw*3] = tmp;
					}
				}
			}
		for(jj=4; jj<m-3; jj+=4)
			{
			//
			d0 = pD[0+jj*sdd+ps*0];
			d1 = pD[0+jj*sdd+ps*1];
			d2 = pD[0+jj*sdd+ps*2];
			d3 = pD[0+jj*sdd+ps*3];
			tmp = pC[0+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * d0;
			pW[0+ldw*1] += tmp * d1;
			pW[0+ldw*2] += tmp * d2;
			pW[0+ldw*3] += tmp * d3;
			tmp = pC[0+jj*sdc+ps*1];
			pW[1+ldw*0] += tmp * d0;
			pW[1+ldw*1] += tmp * d1;
			pW[1+ldw*2] += tmp * d2;
			pW[1+ldw*3] += tmp * d3;
			//
			d0 = pD[1+jj*sdd+ps*0];
			d1 = pD[1+jj*sdd+ps*1];
			d2 = pD[1+jj*sdd+ps*2];
			d3 = pD[1+jj*sdd+ps*3];
			tmp = pC[1+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * d0;
			pW[0+ldw*1] += tmp * d1;
			pW[0+ldw*2] += tmp * d2;
			pW[0+ldw*3] += tmp * d3;
			tmp = pC[1+jj*sdc+ps*1];
			pW[1+ldw*0] += tmp * d0;
			pW[1+ldw*1] += tmp * d1;
			pW[1+ldw*2] += tmp * d2;
			pW[1+ldw*3] += tmp * d3;
			//
			d0 = pD[2+jj*sdd+ps*0];
			d1 = pD[2+jj*sdd+ps*1];
			d2 = pD[2+jj*sdd+ps*2];
			d3 = pD[2+jj*sdd+ps*3];
			tmp = pC[2+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * d0;
			pW[0+ldw*1] += tmp * d1;
			pW[0+ldw*2] += tmp * d2;
			pW[0+ldw*3] += tmp * d3;
			tmp = pC[2+jj*sdc+ps*1];
			pW[1+ldw*0] += tmp * d0;
			pW[1+ldw*1] += tmp * d1;
			pW[1+ldw*2] += tmp * d2;
			pW[1+ldw*3] += tmp * d3;
			//
			d0 = pD[3+jj*sdd+ps*0];
			d1 = pD[3+jj*sdd+ps*1];
			d2 = pD[3+jj*sdd+ps*2];
			d3 = pD[3+jj*sdd+ps*3];
			tmp = pC[3+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * d0;
			pW[0+ldw*1] += tmp * d1;
			pW[0+ldw*2] += tmp * d2;
			pW[0+ldw*3] += tmp * d3;
			tmp = pC[3+jj*sdc+ps*1];
			pW[1+ldw*0] += tmp * d0;
			pW[1+ldw*1] += tmp * d1;
			pW[1+ldw*2] += tmp * d2;
			pW[1+ldw*3] += tmp * d3;
			}
		for(ll=0; ll<m-jj; ll++)
			{
			d0 = pD[ll+jj*sdd+ps*0];
			d1 = pD[ll+jj*sdd+ps*1];
			d2 = pD[ll+jj*sdd+ps*2];
			d3 = pD[ll+jj*sdd+ps*3];
			tmp = pC[ll+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * d0;
			pW[0+ldw*1] += tmp * d1;
			pW[0+ldw*2] += tmp * d2;
			pW[0+ldw*3] += tmp * d3;
			tmp = pC[ll+jj*sdc+ps*1];
			pW[1+ldw*0] += tmp * d0;
			pW[1+ldw*1] += tmp * d1;
			pW[1+ldw*2] += tmp * d2;
			pW[1+ldw*3] += tmp * d3;
			}
		// compute W^T *= T
		pW[0+ldw*3] = pT[3+ldt*0]*pW[0+ldw*0] + pT[3+ldt*1]*pW[0+ldw*1] + pT[3+ldt*2]*pW[0+ldw*2] + pT[3+ldt*3]*pW[0+ldw*3];
		pW[1+ldw*3] = pT[3+ldt*0]*pW[1+ldw*0] + pT[3+ldt*1]*pW[1+ldw*1] + pT[3+ldt*2]*pW[1+ldw*2] + pT[3+ldt*3]*pW[1+ldw*3];
		pW[0+ldw*2] = pT[2+ldt*0]*pW[0+ldw*0] + pT[2+ldt*1]*pW[0+ldw*1] + pT[2+ldt*2]*pW[0+ldw*2];
		pW[1+ldw*2] = pT[2+ldt*0]*pW[1+ldw*0] + pT[2+ldt*1]*pW[1+ldw*1] + pT[2+ldt*2]*pW[1+ldw*2];
		pW[0+ldw*1] = pT[1+ldt*0]*pW[0+ldw*0] + pT[1+ldt*1]*pW[0+ldw*1];
		pW[1+ldw*1] = pT[1+ldt*0]*pW[1+ldw*0] + pT[1+ldt*1]*pW[1+ldw*1];
		pW[0+ldw*0] = pT[0+ldt*0]*pW[0+ldw*0];
		pW[1+ldw*0] = pT[0+ldt*0]*pW[1+ldw*0];
		// compute C -= V * W^T
		pC[0+ps*0] -= pW[0+ldw*0];
		pC[0+ps*1] -= pW[1+ldw*0];
		if(m>1)
			{
			pC[1+ps*0] -= pD[1+ps*0]*pW[0+ldw*0] + pW[0+ldw*1];
			pC[1+ps*1] -= pD[1+ps*0]*pW[1+ldw*0] + pW[1+ldw*1];
			if(m>2)
				{
				pC[2+ps*0] -= pD[2+ps*0]*pW[0+ldw*0] + pD[2+ps*1]*pW[0+ldw*1] + pW[0+ldw*2];
				pC[2+ps*1] -= pD[2+ps*0]*pW[1+ldw*0] + pD[2+ps*1]*pW[1+ldw*1] + pW[1+ldw*2];
				if(m>3)
					{
					pC[3+ps*0] -= pD[3+ps*0]*pW[0+ldw*0] + pD[3+ps*1]*pW[0+ldw*1] + pD[3+ps*2]*pW[0+ldw*2] + pW[0+ldw*3];
					pC[3+ps*1] -= pD[3+ps*0]*pW[1+ldw*0] + pD[3+ps*1]*pW[1+ldw*1] + pD[3+ps*2]*pW[1+ldw*2] + pW[1+ldw*3];
					}
				}
			}
		for(jj=4; jj<m-3; jj+=4)
			{
			//
			d0 = pD[0+jj*sdd+ps*0];
			d1 = pD[0+jj*sdd+ps*1];
			d2 = pD[0+jj*sdd+ps*2];
			d3 = pD[0+jj*sdd+ps*3];
			pC[0+jj*sdc+ps*0] -= d0*pW[0+ldw*0] + d1*pW[0+ldw*1] + d2*pW[0+ldw*2] + d3*pW[0+ldw*3];
			pC[0+jj*sdc+ps*1] -= d0*pW[1+ldw*0] + d1*pW[1+ldw*1] + d2*pW[1+ldw*2] + d3*pW[1+ldw*3];
			//
			d0 = pD[1+jj*sdd+ps*0];
			d1 = pD[1+jj*sdd+ps*1];
			d2 = pD[1+jj*sdd+ps*2];
			d3 = pD[1+jj*sdd+ps*3];
			pC[1+jj*sdc+ps*0] -= d0*pW[0+ldw*0] + d1*pW[0+ldw*1] + d2*pW[0+ldw*2] + d3*pW[0+ldw*3];
			pC[1+jj*sdc+ps*1] -= d0*pW[1+ldw*0] + d1*pW[1+ldw*1] + d2*pW[1+ldw*2] + d3*pW[1+ldw*3];
			//
			d0 = pD[2+jj*sdd+ps*0];
			d1 = pD[2+jj*sdd+ps*1];
			d2 = pD[2+jj*sdd+ps*2];
			d3 = pD[2+jj*sdd+ps*3];
			pC[2+jj*sdc+ps*0] -= d0*pW[0+ldw*0] + d1*pW[0+ldw*1] + d2*pW[0+ldw*2] + d3*pW[0+ldw*3];
			pC[2+jj*sdc+ps*1] -= d0*pW[1+ldw*0] + d1*pW[1+ldw*1] + d2*pW[1+ldw*2] + d3*pW[1+ldw*3];
			//
			d0 = pD[3+jj*sdd+ps*0];
			d1 = pD[3+jj*sdd+ps*1];
			d2 = pD[3+jj*sdd+ps*2];
			d3 = pD[3+jj*sdd+ps*3];
			pC[3+jj*sdc+ps*0] -= d0*pW[0+ldw*0] + d1*pW[0+ldw*1] + d2*pW[0+ldw*2] + d3*pW[0+ldw*3];
			pC[3+jj*sdc+ps*1] -= d0*pW[1+ldw*0] + d1*pW[1+ldw*1] + d2*pW[1+ldw*2] + d3*pW[1+ldw*3];
			}
		for(ll=0; ll<m-jj; ll++)
			{
			d0 = pD[ll+jj*sdd+ps*0];
			d1 = pD[ll+jj*sdd+ps*1];
			d2 = pD[ll+jj*sdd+ps*2];
			d3 = pD[ll+jj*sdd+ps*3];
			pC[ll+jj*sdc+ps*0] -= d0*pW[0+ldw*0] + d1*pW[0+ldw*1] + d2*pW[0+ldw*2] + d3*pW[0+ldw*3];
			pC[ll+jj*sdc+ps*1] -= d0*pW[1+ldw*0] + d1*pW[1+ldw*1] + d2*pW[1+ldw*2] + d3*pW[1+ldw*3];
			}
		}
	for( ; ii<n; ii++)
		{
		pC = pC0+ii*ps;
		// compute W^T = C^T * V
		tmp = pC[0+ps*0];
		pW[0+ldw*0] = tmp;
		if(m>1)
			{
			tmp = pC[1+ps*0];
			pW[0+ldw*0] += tmp * pD[1+ps*0];
			pW[0+ldw*1] = tmp;
			if(m>2)
				{
				tmp = pC[2+ps*0];
				pW[0+ldw*0] += tmp * pD[2+ps*0];
				pW[0+ldw*1] += tmp * pD[2+ps*1];
				pW[0+ldw*2] = tmp;
				if(m>3)
					{
					tmp = pC[3+ps*0];
					pW[0+ldw*0] += tmp * pD[3+ps*0];
					pW[0+ldw*1] += tmp * pD[3+ps*1];
					pW[0+ldw*2] += tmp * pD[3+ps*2];
					pW[0+ldw*3] = tmp;
					}
				}
			}
		for(jj=4; jj<m-3; jj+=4)
			{
			tmp = pC[0+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * pD[0+jj*sdd+ps*0];
			pW[0+ldw*1] += tmp * pD[0+jj*sdd+ps*1];
			pW[0+ldw*2] += tmp * pD[0+jj*sdd+ps*2];
			pW[0+ldw*3] += tmp * pD[0+jj*sdd+ps*3];
			tmp = pC[1+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * pD[1+jj*sdd+ps*0];
			pW[0+ldw*1] += tmp * pD[1+jj*sdd+ps*1];
			pW[0+ldw*2] += tmp * pD[1+jj*sdd+ps*2];
			pW[0+ldw*3] += tmp * pD[1+jj*sdd+ps*3];
			tmp = pC[2+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * pD[2+jj*sdd+ps*0];
			pW[0+ldw*1] += tmp * pD[2+jj*sdd+ps*1];
			pW[0+ldw*2] += tmp * pD[2+jj*sdd+ps*2];
			pW[0+ldw*3] += tmp * pD[2+jj*sdd+ps*3];
			tmp = pC[3+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * pD[3+jj*sdd+ps*0];
			pW[0+ldw*1] += tmp * pD[3+jj*sdd+ps*1];
			pW[0+ldw*2] += tmp * pD[3+jj*sdd+ps*2];
			pW[0+ldw*3] += tmp * pD[3+jj*sdd+ps*3];
			}
		for(ll=0; ll<m-jj; ll++)
			{
			tmp = pC[ll+jj*sdc+ps*0];
			pW[0+ldw*0] += tmp * pD[ll+jj*sdd+ps*0];
			pW[0+ldw*1] += tmp * pD[ll+jj*sdd+ps*1];
			pW[0+ldw*2] += tmp * pD[ll+jj*sdd+ps*2];
			pW[0+ldw*3] += tmp * pD[ll+jj*sdd+ps*3];
			}
		// compute W^T *= T
		pW[0+ldw*3] = pT[3+ldt*0]*pW[0+ldw*0] + pT[3+ldt*1]*pW[0+ldw*1] + pT[3+ldt*2]*pW[0+ldw*2] + pT[3+ldt*3]*pW[0+ldw*3];
		pW[0+ldw*2] = pT[2+ldt*0]*pW[0+ldw*0] + pT[2+ldt*1]*pW[0+ldw*1] + pT[2+ldt*2]*pW[0+ldw*2];
		pW[0+ldw*1] = pT[1+ldt*0]*pW[0+ldw*0] + pT[1+ldt*1]*pW[0+ldw*1];
		pW[0+ldw*0] = pT[0+ldt*0]*pW[0+ldw*0];
		// compute C -= V * W^T
		pC[0+ps*0] -= pW[0+ldw*0];
		if(m>1)
			{
			pC[1+ps*0] -= pD[1+ps*0]*pW[0+ldw*0] + pW[0+ldw*1];
			if(m>2)
				{
				pC[2+ps*0] -= pD[2+ps*0]*pW[0+ldw*0] + pD[2+ps*1]*pW[0+ldw*1] + pW[0+ldw*2];
				if(m>3)
					{
					pC[3+ps*0] -= pD[3+ps*0]*pW[0+ldw*0] + pD[3+ps*1]*pW[0+ldw*1] + pD[3+ps*2]*pW[0+ldw*2] + pW[0+ldw*3];
					}
				}
			}
		for(jj=4; jj<m-3; jj+=4)
			{
			pC[0+jj*sdc+ps*0] -= pD[0+jj*sdd+ps*0]*pW[0+ldw*0] + pD[0+jj*sdd+ps*1]*pW[0+ldw*1] + pD[0+jj*sdd+ps*2]*pW[0+ldw*2] + pD[0+jj*sdd+ps*3]*pW[0+ldw*3];
			pC[1+jj*sdc+ps*0] -= pD[1+jj*sdd+ps*0]*pW[0+ldw*0] + pD[1+jj*sdd+ps*1]*pW[0+ldw*1] + pD[1+jj*sdd+ps*2]*pW[0+ldw*2] + pD[1+jj*sdd+ps*3]*pW[0+ldw*3];
			pC[2+jj*sdc+ps*0] -= pD[2+jj*sdd+ps*0]*pW[0+ldw*0] + pD[2+jj*sdd+ps*1]*pW[0+ldw*1] + pD[2+jj*sdd+ps*2]*pW[0+ldw*2] + pD[2+jj*sdd+ps*3]*pW[0+ldw*3];
			pC[3+jj*sdc+ps*0] -= pD[3+jj*sdd+ps*0]*pW[0+ldw*0] + pD[3+jj*sdd+ps*1]*pW[0+ldw*1] + pD[3+jj*sdd+ps*2]*pW[0+ldw*2] + pD[3+jj*sdd+ps*3]*pW[0+ldw*3];
			}
		for(ll=0; ll<m-jj; ll++)
			{
			pC[ll+jj*sdc+ps*0] -= pD[ll+jj*sdd+ps*0]*pW[0+ldw*0] + pD[ll+jj*sdd+ps*1]*pW[0+ldw*1] + pD[ll+jj*sdd+ps*2]*pW[0+ldw*2] + pD[ll+jj*sdd+ps*3]*pW[0+ldw*3];
			}
		}

	return;
	}



void kernel_dlarf_t_4_lib4(int m, int n, double *pD, int sdd, double *pVt, double *dD, double *pC0, int sdc, double *pW0)
	{
	if(m<=0 | n<=0)
		return;
	int ii, jj, ll;
	const int ps = 4;
	double v10,
	       v20, v21,
		   v30, v31, v32;
	double c00, c01,
	       c10, c11,
	       c20, c21,
	       c30, c31;
	double a0, a1, a2, a3, b0, b1;
	double tmp, d0, d1, d2, d3;
	double *pC, *pW;
	double pT[16];// = {};
	int ldt = 4;
	// dot product of v
	v10 = 0.0;
	v20 = 0.0;
	v30 = 0.0;
	v21 = 0.0;
	v31 = 0.0;
	v32 = 0.0;
	if(m>1)
		{
		v10 = 1.0 * pD[1+ps*0];
		if(m>2)
			{
			v10 += pD[2+ps*1] * pD[2+ps*0];
			v20 = 1.0 * pD[2+ps*0];
			v21 = 1.0 * pD[2+ps*1];
			if(m>3)
				{
				v10 += pD[3+ps*1] * pD[3+ps*0];
				v20 += pD[3+ps*2] * pD[3+ps*0];
				v21 += pD[3+ps*2] * pD[3+ps*1];
				v30 = 1.0 * pD[3+ps*0];
				v31 = 1.0 * pD[3+ps*1];
				v32 = 1.0 * pD[3+ps*2];
				}
			}
		}
	for(ii=4; ii<m-3; ii+=4)
		{
		v10 += pD[0+ii*sdd+ps*1] * pD[0+ii*sdd+ps*0];
		v20 += pD[0+ii*sdd+ps*2] * pD[0+ii*sdd+ps*0];
		v21 += pD[0+ii*sdd+ps*2] * pD[0+ii*sdd+ps*1];
		v30 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*0];
		v31 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*1];
		v32 += pD[0+ii*sdd+ps*3] * pD[0+ii*sdd+ps*2];
		v10 += pD[1+ii*sdd+ps*1] * pD[1+ii*sdd+ps*0];
		v20 += pD[1+ii*sdd+ps*2] * pD[1+ii*sdd+ps*0];
		v21 += pD[1+ii*sdd+ps*2] * pD[1+ii*sdd+ps*1];
		v30 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*0];
		v31 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*1];
		v32 += pD[1+ii*sdd+ps*3] * pD[1+ii*sdd+ps*2];
		v10 += pD[2+ii*sdd+ps*1] * pD[2+ii*sdd+ps*0];
		v20 += pD[2+ii*sdd+ps*2] * pD[2+ii*sdd+ps*0];
		v21 += pD[2+ii*sdd+ps*2] * pD[2+ii*sdd+ps*1];
		v30 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*0];
		v31 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*1];
		v32 += pD[2+ii*sdd+ps*3] * pD[2+ii*sdd+ps*2];
		v10 += pD[3+ii*sdd+ps*1] * pD[3+ii*sdd+ps*0];
		v20 += pD[3+ii*sdd+ps*2] * pD[3+ii*sdd+ps*0];
		v21 += pD[3+ii*sdd+ps*2] * pD[3+ii*sdd+ps*1];
		v30 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*0];
		v31 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*1];
		v32 += pD[3+ii*sdd+ps*3] * pD[3+ii*sdd+ps*2];
		}
	for(ll=0; ll<m-ii; ll++)
		{
		v10 += pD[ll+ii*sdd+ps*1] * pD[ll+ii*sdd+ps*0];
		v20 += pD[ll+ii*sdd+ps*2] * pD[ll+ii*sdd+ps*0];
		v21 += pD[ll+ii*sdd+ps*2] * pD[ll+ii*sdd+ps*1];
		v30 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*0];
		v31 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*1];
		v32 += pD[ll+ii*sdd+ps*3] * pD[ll+ii*sdd+ps*2];
		}
	// compute lower triangular T containing tau for matrix update
	pT[0+ldt*0] = dD[0];
	pT[1+ldt*1] = dD[1];
	pT[2+ldt*2] = dD[2];
	pT[3+ldt*3] = dD[3];
	pT[1+ldt*0] = - dD[1] * (v10*pT[0+ldt*0]);
	pT[2+ldt*1] = - dD[2] * (v21*pT[1+ldt*1]);
	pT[3+ldt*2] = - dD[3] * (v32*pT[2+ldt*2]);
	pT[2+ldt*0] = - dD[2] * (v20*pT[0+ldt*0] + v21*pT[1+ldt*0]);
	pT[3+ldt*1] = - dD[3] * (v31*pT[1+ldt*1] + v32*pT[2+ldt*1]);
	pT[3+ldt*0] = - dD[3] * (v30*pT[0+ldt*0] + v31*pT[1+ldt*0] + v32*pT[2+ldt*0]);
	// downgrade matrix
	__m256d
		_w0, _w1, _w2, _w3, _d0, _t0, _tp, _c0, _c1, _c2, _c3, _a0, _b0, _tz;

	ii = 0;
#if 1
	double alpha = 1.0;
	double beta = 0.0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for( ; ii<n-11; ii+=12)
		{
		kernel_dgemm_nn_4x12_lib4(m, &alpha, &pVt[0+ps*0], 0, &pC0[0+ps*ii], sdc, &beta, &pW0[0+ps*ii], &pW0[0+ps*ii]);
		}
#endif
	for( ; ii<n-7; ii+=8)
		{
		kernel_dgemm_nn_4x8_lib4(m, &alpha, &pVt[0+ps*0], 0, &pC0[0+ps*ii], sdc, &beta, &pW0[0+ps*ii], &pW0[0+ps*ii]);
		}
	for( ; ii<n-3; ii+=4)
		{
		kernel_dgemm_nn_4x4_lib4(m, &alpha, &pVt[0+ps*0], 0, &pC0[0+ps*ii], sdc, &beta, &pW0[0+ps*ii], &pW0[0+ps*ii]);
		}
	if(ii<n)
		{
//		kernel_dgemm_nn_4x4_vs_lib4(m, &alpha, &pVt[0+ps*0], 0, &pC0[0+ps*ii], sdc, &beta, &pW0[0+ps*ii], &pW0[0+ps*ii], 4, n-ii);
		kernel_dgemm_nn_4x4_gen_lib4(m, &alpha, &pVt[0+ps*0], 0, &pC0[0+ps*ii], sdc, &beta, 0, &pW0[0+ps*ii], 0, 0, &pW0[0+ps*ii], 0, 0, 4, 0, n-ii);
		}
#else
	for( ; ii<n-3; ii+=4)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;
		// compute W^T = C^T * V
		_w0 = _mm256_setzero_pd();
		_w1 = _mm256_setzero_pd();
		_w2 = _mm256_setzero_pd();
		_w3 = _mm256_setzero_pd();
		for(jj=0; jj<m-3; jj+=4)
			{
			//
			_d0 = _mm256_load_pd( &pVt[0+ps*(0+jj)] );
			_t0 = _mm256_broadcast_sd( &pC[0+jj*sdc+ps*0] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w0 = _mm256_add_pd( _w0, _tp );
			_t0 = _mm256_broadcast_sd( &pC[0+jj*sdc+ps*1] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w1 = _mm256_add_pd( _w1, _tp );
			_t0 = _mm256_broadcast_sd( &pC[0+jj*sdc+ps*2] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w2 = _mm256_add_pd( _w2, _tp );
			_t0 = _mm256_broadcast_sd( &pC[0+jj*sdc+ps*3] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w3 = _mm256_add_pd( _w3, _tp );
			//
			_d0 = _mm256_load_pd( &pVt[0+ps*(1+jj)] );
			_t0 = _mm256_broadcast_sd( &pC[1+jj*sdc+ps*0] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w0 = _mm256_add_pd( _w0, _tp );
			_t0 = _mm256_broadcast_sd( &pC[1+jj*sdc+ps*1] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w1 = _mm256_add_pd( _w1, _tp );
			_t0 = _mm256_broadcast_sd( &pC[1+jj*sdc+ps*2] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w2 = _mm256_add_pd( _w2, _tp );
			_t0 = _mm256_broadcast_sd( &pC[1+jj*sdc+ps*3] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w3 = _mm256_add_pd( _w3, _tp );
			//
			_d0 = _mm256_load_pd( &pVt[0+ps*(2+jj)] );
			_t0 = _mm256_broadcast_sd( &pC[2+jj*sdc+ps*0] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w0 = _mm256_add_pd( _w0, _tp );
			_t0 = _mm256_broadcast_sd( &pC[2+jj*sdc+ps*1] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w1 = _mm256_add_pd( _w1, _tp );
			_t0 = _mm256_broadcast_sd( &pC[2+jj*sdc+ps*2] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w2 = _mm256_add_pd( _w2, _tp );
			_t0 = _mm256_broadcast_sd( &pC[2+jj*sdc+ps*3] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w3 = _mm256_add_pd( _w3, _tp );
			//
			_d0 = _mm256_load_pd( &pVt[0+ps*(3+jj)] );
			_t0 = _mm256_broadcast_sd( &pC[3+jj*sdc+ps*0] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w0 = _mm256_add_pd( _w0, _tp );
			_t0 = _mm256_broadcast_sd( &pC[3+jj*sdc+ps*1] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w1 = _mm256_add_pd( _w1, _tp );
			_t0 = _mm256_broadcast_sd( &pC[3+jj*sdc+ps*2] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w2 = _mm256_add_pd( _w2, _tp );
			_t0 = _mm256_broadcast_sd( &pC[3+jj*sdc+ps*3] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w3 = _mm256_add_pd( _w3, _tp );
			}
		for(ll=0; ll<m-jj; ll++)
			{
			_d0 = _mm256_load_pd( &pVt[0+ps*(ll+jj)] );
			_t0 = _mm256_broadcast_sd( &pC[ll+jj*sdc+ps*0] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w0 = _mm256_add_pd( _w0, _tp );
			_t0 = _mm256_broadcast_sd( &pC[ll+jj*sdc+ps*1] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w1 = _mm256_add_pd( _w1, _tp );
			_t0 = _mm256_broadcast_sd( &pC[ll+jj*sdc+ps*2] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w2 = _mm256_add_pd( _w2, _tp );
			_t0 = _mm256_broadcast_sd( &pC[ll+jj*sdc+ps*3] );
			_tp = _mm256_mul_pd( _d0, _t0 );
			_w3 = _mm256_add_pd( _w3, _tp );
			}
		// TODO mask store
		_mm256_storeu_pd( &pW[0+ps*0], _w0 );
		_mm256_storeu_pd( &pW[0+ps*1], _w1 );
		_mm256_storeu_pd( &pW[0+ps*2], _w2 );
		_mm256_storeu_pd( &pW[0+ps*3], _w3 );
		}
	for( ; ii<n; ii++)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;
		// compute W^T = C^T * V
		tmp = pC[0+ps*0];
		pW[0+ps*0] = tmp;
		if(m>1)
			{
			d0 = pVt[0+ps*1];
			tmp = pC[1+ps*0];
			pW[0+ps*0] += d0 * tmp;
			pW[1+ps*0] = tmp;
			if(m>2)
				{
				d0 = pVt[0+ps*2];
				d1 = pVt[1+ps*2];
				tmp = pC[2+ps*0];
				pW[0+ps*0] += d0 * tmp;
				pW[1+ps*0] += d1 * tmp;
				pW[2+ps*0] = tmp;
				if(m>3)
					{
					d0 = pVt[0+ps*3];
					d1 = pVt[1+ps*3];
					d2 = pVt[2+ps*3];
					tmp = pC[3+ps*0];
					pW[0+ps*0] += d0 * tmp;
					pW[1+ps*0] += d1 * tmp;
					pW[2+ps*0] += d2 * tmp;
					pW[3+ps*0] = tmp;
					}
				}
			}
		for(jj=4; jj<m-3; jj+=4)
			{
			//
			d0 = pVt[0+ps*(0+jj)];
			d1 = pVt[1+ps*(0+jj)];
			d2 = pVt[2+ps*(0+jj)];
			d3 = pVt[3+ps*(0+jj)];
			tmp = pC[0+jj*sdc+ps*0];
			pW[0+ps*0] += d0 * tmp;
			pW[1+ps*0] += d1 * tmp;
			pW[2+ps*0] += d2 * tmp;
			pW[3+ps*0] += d3 * tmp;
			//
			d0 = pVt[0+ps*(1+jj)];
			d1 = pVt[1+ps*(1+jj)];
			d2 = pVt[2+ps*(1+jj)];
			d3 = pVt[3+ps*(1+jj)];
			tmp = pC[1+jj*sdc+ps*0];
			pW[0+ps*0] += d0 * tmp;
			pW[1+ps*0] += d1 * tmp;
			pW[2+ps*0] += d2 * tmp;
			pW[3+ps*0] += d3 * tmp;
			//
			d0 = pVt[0+ps*(2+jj)];
			d1 = pVt[1+ps*(2+jj)];
			d2 = pVt[2+ps*(2+jj)];
			d3 = pVt[3+ps*(2+jj)];
			tmp = pC[2+jj*sdc+ps*0];
			pW[0+ps*0] += d0 * tmp;
			pW[1+ps*0] += d1 * tmp;
			pW[2+ps*0] += d2 * tmp;
			pW[3+ps*0] += d3 * tmp;
			//
			d0 = pVt[0+ps*(3+jj)];
			d1 = pVt[1+ps*(3+jj)];
			d2 = pVt[2+ps*(3+jj)];
			d3 = pVt[3+ps*(3+jj)];
			tmp = pC[3+jj*sdc+ps*0];
			pW[0+ps*0] += d0 * tmp;
			pW[1+ps*0] += d1 * tmp;
			pW[2+ps*0] += d2 * tmp;
			pW[3+ps*0] += d3 * tmp;
			}
		for(ll=0; ll<m-jj; ll++)
			{
			d0 = pVt[0+ps*(ll+jj)];
			d1 = pVt[1+ps*(ll+jj)];
			d2 = pVt[2+ps*(ll+jj)];
			d3 = pVt[3+ps*(ll+jj)];
			tmp = pC[ll+jj*sdc+ps*0];
			pW[0+ps*0] += d0 * tmp;
			pW[1+ps*0] += d1 * tmp;
			pW[2+ps*0] += d2 * tmp;
			pW[3+ps*0] += d3 * tmp;
			}
		}
#endif

	ii = 0;
	for( ; ii<n-3; ii+=4)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;

		// compute W^T *= T
		_tz = _mm256_setzero_pd();

		_t0 = _mm256_load_pd( &pT[0+ldt*0] );
		_tp = _mm256_broadcast_sd( &pW[0+ps*0] );
		_w0 = _mm256_mul_pd( _t0, _tp );
		_tp = _mm256_broadcast_sd( &pW[0+ps*1] );
		_w1 = _mm256_mul_pd( _t0, _tp );
		_tp = _mm256_broadcast_sd( &pW[0+ps*2] );
		_w2 = _mm256_mul_pd( _t0, _tp );
		_tp = _mm256_broadcast_sd( &pW[0+ps*3] );
		_w3 = _mm256_mul_pd( _t0, _tp );

#if defined(TARGET_X64_INTEL_GASWELL)
		_t0 = _mm256_load_pd( &pT[0+ldt*1] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x1 );
		_tp = _mm256_broadcast_sd( &pW[1+ps*0] );
		_w0 = _mm256_fmadd_pd( _t0, _tp, _w0 );
		_tp = _mm256_broadcast_sd( &pW[1+ps*1] );
		_w1 = _mm256_fmadd_pd( _t0, _tp, _w1 );
		_tp = _mm256_broadcast_sd( &pW[1+ps*2] );
		_w2 = _mm256_fmadd_pd( _t0, _tp, _w2 );
		_tp = _mm256_broadcast_sd( &pW[1+ps*3] );
		_w3 = _mm256_fmadd_pd( _t0, _tp, _w3 );

		_t0 = _mm256_load_pd( &pT[0+ldt*2] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x3 );
		_tp = _mm256_broadcast_sd( &pW[2+ps*0] );
		_w0 = _mm256_fmadd_pd( _t0, _tp, _w0 );
		_tp = _mm256_broadcast_sd( &pW[2+ps*1] );
		_w1 = _mm256_fmadd_pd( _t0, _tp, _w1 );
		_tp = _mm256_broadcast_sd( &pW[2+ps*2] );
		_w2 = _mm256_fmadd_pd( _t0, _tp, _w2 );
		_tp = _mm256_broadcast_sd( &pW[2+ps*3] );
		_w3 = _mm256_fmadd_pd( _t0, _tp, _w3 );

		_t0 = _mm256_load_pd( &pT[0+ldt*3] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x7 );
		_tp = _mm256_broadcast_sd( &pW[3+ps*0] );
		_w0 = _mm256_fmadd_pd( _t0, _tp, _w0 );
		_tp = _mm256_broadcast_sd( &pW[3+ps*1] );
		_w1 = _mm256_fmadd_pd( _t0, _tp, _w1 );
		_tp = _mm256_broadcast_sd( &pW[3+ps*2] );
		_w2 = _mm256_fmadd_pd( _t0, _tp, _w2 );
		_tp = _mm256_broadcast_sd( &pW[3+ps*3] );
		_w3 = _mm256_fmadd_pd( _t0, _tp, _w3 );
#else
		_t0 = _mm256_load_pd( &pT[0+ldt*1] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x1 );
		_tp = _mm256_broadcast_sd( &pW[1+ps*0] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w0 = _mm256_add_pd( _w0, _tp );
		_tp = _mm256_broadcast_sd( &pW[1+ps*1] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w1 = _mm256_add_pd( _w1, _tp );
		_tp = _mm256_broadcast_sd( &pW[1+ps*2] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w2 = _mm256_add_pd( _w2, _tp );
		_tp = _mm256_broadcast_sd( &pW[1+ps*3] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w3 = _mm256_add_pd( _w3, _tp );

		_t0 = _mm256_load_pd( &pT[0+ldt*2] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x3 );
		_tp = _mm256_broadcast_sd( &pW[2+ps*0] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w0 = _mm256_add_pd( _w0, _tp );
		_tp = _mm256_broadcast_sd( &pW[2+ps*1] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w1 = _mm256_add_pd( _w1, _tp );
		_tp = _mm256_broadcast_sd( &pW[2+ps*2] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w2 = _mm256_add_pd( _w2, _tp );
		_tp = _mm256_broadcast_sd( &pW[2+ps*3] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w3 = _mm256_add_pd( _w3, _tp );

		_t0 = _mm256_load_pd( &pT[0+ldt*3] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x7 );
		_tp = _mm256_broadcast_sd( &pW[3+ps*0] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w0 = _mm256_add_pd( _w0, _tp );
		_tp = _mm256_broadcast_sd( &pW[3+ps*1] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w1 = _mm256_add_pd( _w1, _tp );
		_tp = _mm256_broadcast_sd( &pW[3+ps*2] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w2 = _mm256_add_pd( _w2, _tp );
		_tp = _mm256_broadcast_sd( &pW[3+ps*3] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w3 = _mm256_add_pd( _w3, _tp );
#endif

		_mm256_store_pd( &pW[0+ps*0], _w0 );
		_mm256_store_pd( &pW[0+ps*1], _w1 );
		_mm256_store_pd( &pW[0+ps*2], _w2 );
		_mm256_store_pd( &pW[0+ps*3], _w3 );
		}
	for( ; ii<n; ii++)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;

		// compute W^T *= T
		_tz = _mm256_setzero_pd();

		_t0 = _mm256_load_pd( &pT[0+ldt*0] );
		_tp = _mm256_broadcast_sd( &pW[0+ps*0] );
		_w0 = _mm256_mul_pd( _t0, _tp );

		_t0 = _mm256_load_pd( &pT[0+ldt*1] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x1 );
		_tp = _mm256_broadcast_sd( &pW[1+ps*0] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w0 = _mm256_add_pd( _w0, _tp );

		_t0 = _mm256_load_pd( &pT[0+ldt*2] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x3 );
		_tp = _mm256_broadcast_sd( &pW[2+ps*0] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w0 = _mm256_add_pd( _w0, _tp );

		_t0 = _mm256_load_pd( &pT[0+ldt*3] );
		_t0 = _mm256_blend_pd( _t0, _tz, 0x7 );
		_tp = _mm256_broadcast_sd( &pW[3+ps*0] );
		_tp = _mm256_mul_pd( _t0, _tp );
		_w0 = _mm256_add_pd( _w0, _tp );

		_mm256_store_pd( &pW[0+ps*0], _w0 );
		}

	ii = 0;
	for( ; ii<n-3; ii+=4)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;
		// compute C -= V * W^T
		jj = 0;
		// load
		c00 = pC[0+jj*sdc+ps*0];
		c10 = pC[1+jj*sdc+ps*0];
		c20 = pC[2+jj*sdc+ps*0];
		c30 = pC[3+jj*sdc+ps*0];
		c01 = pC[0+jj*sdc+ps*1];
		c11 = pC[1+jj*sdc+ps*1];
		c21 = pC[2+jj*sdc+ps*1];
		c31 = pC[3+jj*sdc+ps*1];
		// rank1
		a1 = pD[1+jj*sdd+ps*0];
		a2 = pD[2+jj*sdd+ps*0];
		a3 = pD[3+jj*sdd+ps*0];
		b0 = pW[0+ps*0];
		c00 -= b0;
		c10 -= a1*b0;
		c20 -= a2*b0;
		c30 -= a3*b0;
		b1 = pW[0+ps*1];
		c01 -= b1;
		c11 -= a1*b1;
		c21 -= a2*b1;
		c31 -= a3*b1;
		// rank2
		a2 = pD[2+jj*sdd+ps*1];
		a3 = pD[3+jj*sdd+ps*1];
		b0 = pW[1+ps*0];
		c10 -= b0;
		c20 -= a2*b0;
		c30 -= a3*b0;
		b1 = pW[1+ps*1];
		c11 -= b1;
		c21 -= a2*b1;
		c31 -= a3*b1;
		// rank3
		a3 = pD[3+jj*sdd+ps*2];
		b0 = pW[2+ps*0];
		c20 -= b0;
		c30 -= a3*b0;
		b1 = pW[2+ps*1];
		c21 -= b1;
		c31 -= a3*b1;
		// rank4
		a3 = pD[3+jj*sdd+ps*3];
		b0 = pW[3+ps*0];
		c30 -= b0;
		b1 = pW[3+ps*1];
		c31 -= b1;
		// store
		pC[0+jj*sdc+ps*0] = c00;
		pC[0+jj*sdc+ps*1] = c01;
		if(m>1)
			{
			pC[1+jj*sdc+ps*0] = c10;
			pC[1+jj*sdc+ps*1] = c11;
			if(m>2)
				{
				pC[2+jj*sdc+ps*0] = c20;
				pC[2+jj*sdc+ps*1] = c21;
				if(m>3)
					{
					pC[3+jj*sdc+ps*0] = c30;
					pC[3+jj*sdc+ps*1] = c31;
					}
				}
			}
		// load
		c00 = pC[0+jj*sdc+ps*2];
		c10 = pC[1+jj*sdc+ps*2];
		c20 = pC[2+jj*sdc+ps*2];
		c30 = pC[3+jj*sdc+ps*2];
		c01 = pC[0+jj*sdc+ps*3];
		c11 = pC[1+jj*sdc+ps*3];
		c21 = pC[2+jj*sdc+ps*3];
		c31 = pC[3+jj*sdc+ps*3];
		// rank1
		a1 = pD[1+jj*sdd+ps*0];
		a2 = pD[2+jj*sdd+ps*0];
		a3 = pD[3+jj*sdd+ps*0];
		b0 = pW[0+ps*2];
		c00 -= b0;
		c10 -= a1*b0;
		c20 -= a2*b0;
		c30 -= a3*b0;
		b1 = pW[0+ps*3];
		c01 -= b1;
		c11 -= a1*b1;
		c21 -= a2*b1;
		c31 -= a3*b1;
		// rank2
		a2 = pD[2+jj*sdd+ps*1];
		a3 = pD[3+jj*sdd+ps*1];
		b0 = pW[1+ps*2];
		c10 -= b0;
		c20 -= a2*b0;
		c30 -= a3*b0;
		b1 = pW[1+ps*3];
		c11 -= b1;
		c21 -= a2*b1;
		c31 -= a3*b1;
		// rank3
		a3 = pD[3+jj*sdd+ps*2];
		b0 = pW[2+ps*2];
		c20 -= b0;
		c30 -= a3*b0;
		b1 = pW[2+ps*3];
		c21 -= b1;
		c31 -= a3*b1;
		// rank4
		a3 = pD[3+jj*sdd+ps*3];
		b0 = pW[3+ps*2];
		c30 -= b0;
		b1 = pW[3+ps*3];
		c31 -= b1;
		// store
		pC[0+jj*sdc+ps*2] = c00;
		pC[0+jj*sdc+ps*3] = c01;
		if(m>1)
			{
			pC[1+jj*sdc+ps*2] = c10;
			pC[1+jj*sdc+ps*3] = c11;
			if(m>2)
				{
				pC[2+jj*sdc+ps*2] = c20;
				pC[2+jj*sdc+ps*3] = c21;
				if(m>3)
					{
					pC[3+jj*sdc+ps*2] = c30;
					pC[3+jj*sdc+ps*3] = c31;
					}
				}
			}
		}
	for( ; ii<n; ii++)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;
		// compute C -= V * W^T
		jj = 0;
		// load
		c00 = pC[0+jj*sdc+ps*0];
		c10 = pC[1+jj*sdc+ps*0];
		c20 = pC[2+jj*sdc+ps*0];
		c30 = pC[3+jj*sdc+ps*0];
		// rank1
		a1 = pD[1+jj*sdd+ps*0];
		a2 = pD[2+jj*sdd+ps*0];
		a3 = pD[3+jj*sdd+ps*0];
		b0 = pW[0+ps*0];
		c00 -= b0;
		c10 -= a1*b0;
		c20 -= a2*b0;
		c30 -= a3*b0;
		// rank2
		a2 = pD[2+jj*sdd+ps*1];
		a3 = pD[3+jj*sdd+ps*1];
		b0 = pW[1+ps*0];
		c10 -= b0;
		c20 -= a2*b0;
		c30 -= a3*b0;
		// rank3
		a3 = pD[3+jj*sdd+ps*2];
		b0 = pW[2+ps*0];
		c20 -= b0;
		c30 -= a3*b0;
		// rank4
		a3 = pD[3+jj*sdd+ps*3];
		b0 = pW[3+ps*0];
		c30 -= b0;
		// store
		pC[0+jj*sdc+ps*0] = c00;
		if(m>1)
			{
			pC[1+jj*sdc+ps*0] = c10;
			if(m>2)
				{
				pC[2+jj*sdc+ps*0] = c20;
				if(m>3)
					{
					pC[3+jj*sdc+ps*0] = c30;
					}
				}
			}
		}

#if 1
	jj = 4;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; jj<m-11; jj+=12)
		{
		kernel_dger4_sub_12r_lib4(n, &pD[jj*sdd], sdd, &pW0[0], &pC0[jj*sdc], sdc);
		}
#endif
	for(; jj<m-7; jj+=8)
		{
		kernel_dger4_sub_8r_lib4(n, &pD[jj*sdd], sdd, &pW0[0], &pC0[jj*sdc], sdc);
		}
	for(; jj<m-3; jj+=4)
		{
		kernel_dger4_sub_4r_lib4(n, &pD[jj*sdd], &pW0[0], &pC0[jj*sdc]);
		}
	if(jj<m)
		{
		kernel_dger4_sub_4r_vs_lib4(n, &pD[jj*sdd], &pW0[0], &pC0[jj*sdc], m-jj);
		}
#else
	ii = 0;
	for( ; ii<n-3; ii+=4)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;
		for(jj=4; jj<m-3; jj+=4)
			{
			// load
			_c0 = _mm256_load_pd( &pC[0+jj*sdc+ps*0] );
			_c1 = _mm256_load_pd( &pC[0+jj*sdc+ps*1] );
			_c2 = _mm256_load_pd( &pC[0+jj*sdc+ps*2] );
			_c3 = _mm256_load_pd( &pC[0+jj*sdc+ps*3] );
			//
			_a0 = _mm256_load_pd( &pD[0+jj*sdd+ps*0] );
			_b0 = _mm256_broadcast_sd( &pW[0+ps*0] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c0 = _mm256_sub_pd( _c0, _tp );
			_b0 = _mm256_broadcast_sd( &pW[0+ps*1] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c1 = _mm256_sub_pd( _c1, _tp );
			_b0 = _mm256_broadcast_sd( &pW[0+ps*2] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c2 = _mm256_sub_pd( _c2, _tp );
			_b0 = _mm256_broadcast_sd( &pW[0+ps*3] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c3 = _mm256_sub_pd( _c3, _tp );
			//
			_a0 = _mm256_load_pd( &pD[0+jj*sdd+ps*1] );
			_b0 = _mm256_broadcast_sd( &pW[1+ps*0] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c0 = _mm256_sub_pd( _c0, _tp );
			_b0 = _mm256_broadcast_sd( &pW[1+ps*1] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c1 = _mm256_sub_pd( _c1, _tp );
			_b0 = _mm256_broadcast_sd( &pW[1+ps*2] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c2 = _mm256_sub_pd( _c2, _tp );
			_b0 = _mm256_broadcast_sd( &pW[1+ps*3] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c3 = _mm256_sub_pd( _c3, _tp );
			//
			_a0 = _mm256_load_pd( &pD[0+jj*sdd+ps*2] );
			_b0 = _mm256_broadcast_sd( &pW[2+ps*0] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c0 = _mm256_sub_pd( _c0, _tp );
			_b0 = _mm256_broadcast_sd( &pW[2+ps*1] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c1 = _mm256_sub_pd( _c1, _tp );
			_b0 = _mm256_broadcast_sd( &pW[2+ps*2] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c2 = _mm256_sub_pd( _c2, _tp );
			_b0 = _mm256_broadcast_sd( &pW[2+ps*3] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c3 = _mm256_sub_pd( _c3, _tp );
			//
			_a0 = _mm256_load_pd( &pD[0+jj*sdd+ps*3] );
			_b0 = _mm256_broadcast_sd( &pW[3+ps*0] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c0 = _mm256_sub_pd( _c0, _tp );
			_b0 = _mm256_broadcast_sd( &pW[3+ps*1] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c1 = _mm256_sub_pd( _c1, _tp );
			_b0 = _mm256_broadcast_sd( &pW[3+ps*2] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c2 = _mm256_sub_pd( _c2, _tp );
			_b0 = _mm256_broadcast_sd( &pW[3+ps*3] );
			_tp = _mm256_mul_pd( _a0, _b0 );
			_c3 = _mm256_sub_pd( _c3, _tp );
			// store
			_mm256_store_pd( &pC[0+jj*sdc+ps*0], _c0 );
			_mm256_store_pd( &pC[0+jj*sdc+ps*1], _c1 );
			_mm256_store_pd( &pC[0+jj*sdc+ps*2], _c2 );
			_mm256_store_pd( &pC[0+jj*sdc+ps*3], _c3 );
			}
		for(ll=0; ll<m-jj; ll++)
			{
			// load
			c00 = pC[ll+jj*sdc+ps*0];
			c01 = pC[ll+jj*sdc+ps*1];
			//
			a0 = pD[ll+jj*sdd+ps*0];
			b0 = pW[0+ps*0];
			c00 -= a0*b0;
			b1 = pW[0+ps*1];
			c01 -= a0*b1;
			//
			a0 = pD[ll+jj*sdd+ps*1];
			b0 = pW[1+ps*0];
			c00 -= a0*b0;
			b1 = pW[1+ps*1];
			c01 -= a0*b1;
			//
			a0 = pD[ll+jj*sdd+ps*2];
			b0 = pW[2+ps*0];
			c00 -= a0*b0;
			b1 = pW[2+ps*1];
			c01 -= a0*b1;
			//
			a0 = pD[ll+jj*sdd+ps*3];
			b0 = pW[3+ps*0];
			c00 -= a0*b0;
			b1 = pW[3+ps*1];
			c01 -= a0*b1;
			// store
			pC[ll+jj*sdc+ps*0] = c00;
			pC[ll+jj*sdc+ps*1] = c01;
			// load
			c00 = pC[ll+jj*sdc+ps*2];
			c01 = pC[ll+jj*sdc+ps*3];
			//
			a0 = pD[ll+jj*sdd+ps*0];
			b0 = pW[0+ps*2];
			c00 -= a0*b0;
			b1 = pW[0+ps*3];
			c01 -= a0*b1;
			//
			a0 = pD[ll+jj*sdd+ps*1];
			b0 = pW[1+ps*2];
			c00 -= a0*b0;
			b1 = pW[1+ps*3];
			c01 -= a0*b1;
			//
			a0 = pD[ll+jj*sdd+ps*2];
			b0 = pW[2+ps*2];
			c00 -= a0*b0;
			b1 = pW[2+ps*3];
			c01 -= a0*b1;
			//
			a0 = pD[ll+jj*sdd+ps*3];
			b0 = pW[3+ps*2];
			c00 -= a0*b0;
			b1 = pW[3+ps*3];
			c01 -= a0*b1;
			// store
			pC[ll+jj*sdc+ps*2] = c00;
			pC[ll+jj*sdc+ps*3] = c01;
			}
		}
	for( ; ii<n; ii++)
		{
		pW = pW0+ii*ps;
		pC = pC0+ii*ps;
		for(jj=4; jj<m-3; jj+=4)
			{
			// load
			c00 = pC[0+jj*sdc+ps*0];
			c10 = pC[1+jj*sdc+ps*0];
			c20 = pC[2+jj*sdc+ps*0];
			c30 = pC[3+jj*sdc+ps*0];
			//
			a0 = pD[0+jj*sdd+ps*0];
			a1 = pD[1+jj*sdd+ps*0];
			a2 = pD[2+jj*sdd+ps*0];
			a3 = pD[3+jj*sdd+ps*0];
			b0 = pW[0+ps*0];
			c00 -= a0*b0;
			c10 -= a1*b0;
			c20 -= a2*b0;
			c30 -= a3*b0;
			//
			a0 = pD[0+jj*sdd+ps*1];
			a1 = pD[1+jj*sdd+ps*1];
			a2 = pD[2+jj*sdd+ps*1];
			a3 = pD[3+jj*sdd+ps*1];
			b0 = pW[1+ps*0];
			c00 -= a0*b0;
			c10 -= a1*b0;
			c20 -= a2*b0;
			c30 -= a3*b0;
			//
			a0 = pD[0+jj*sdd+ps*2];
			a1 = pD[1+jj*sdd+ps*2];
			a2 = pD[2+jj*sdd+ps*2];
			a3 = pD[3+jj*sdd+ps*2];
			b0 = pW[2+ps*0];
			c00 -= a0*b0;
			c10 -= a1*b0;
			c20 -= a2*b0;
			c30 -= a3*b0;
			//
			a0 = pD[0+jj*sdd+ps*3];
			a1 = pD[1+jj*sdd+ps*3];
			a2 = pD[2+jj*sdd+ps*3];
			a3 = pD[3+jj*sdd+ps*3];
			b0 = pW[3+ps*0];
			c00 -= a0*b0;
			c10 -= a1*b0;
			c20 -= a2*b0;
			c30 -= a3*b0;
			// store
			pC[0+jj*sdc+ps*0] = c00;
			pC[1+jj*sdc+ps*0] = c10;
			pC[2+jj*sdc+ps*0] = c20;
			pC[3+jj*sdc+ps*0] = c30;
			}
		for(ll=0; ll<m-jj; ll++)
			{
			// load
			c00 = pC[ll+jj*sdc+ps*0];
			//
			a0 = pD[ll+jj*sdd+ps*0];
			b0 = pW[0+ps*0];
			c00 -= a0*b0;
			//
			a0 = pD[ll+jj*sdd+ps*1];
			b0 = pW[1+ps*0];
			c00 -= a0*b0;
			//
			a0 = pD[ll+jj*sdd+ps*2];
			b0 = pW[2+ps*0];
			c00 -= a0*b0;
			//
			a0 = pD[ll+jj*sdd+ps*3];
			b0 = pW[3+ps*0];
			c00 -= a0*b0;
			// store
			pC[ll+jj*sdc+ps*0] = c00;
			}
		}
#endif

	return;
	}



// assume n>=4
void kernel_dgelqf_4_lib4(int n, double *pD, double *dD)
	{
	int ii, jj, ll;
	double alpha, beta, tmp, w1, w2, w3;
	const int ps = 4;
	// first column
	beta = 0.0;
	for(ii=1; ii<n; ii++)
		{
		tmp = pD[0+ps*ii];
		beta += tmp*tmp;
		}
	if(beta==0.0)
		{
		dD[0] = 0.0;
		tmp = 0.0;
		goto col2;
		}
	alpha = pD[0+ps*0];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[0] = (beta-alpha) / beta;
	tmp = 1.0 / (alpha-beta);
	//
	pD[0+ps*0] = beta;
	w1 = pD[1+ps*0];
	w2 = pD[2+ps*0];
	w3 = pD[3+ps*0];
	//
	pD[0+ps*1] *= tmp;
	w1 += pD[1+ps*1] * pD[0+ps*1];
	w2 += pD[2+ps*1] * pD[0+ps*1];
	w3 += pD[3+ps*1] * pD[0+ps*1];
	//
	pD[0+ps*2] *= tmp;
	w1 += pD[1+ps*2] * pD[0+ps*2];
	w2 += pD[2+ps*2] * pD[0+ps*2];
	w3 += pD[3+ps*2] * pD[0+ps*2];
	//
	pD[0+ps*3] *= tmp;
	w1 += pD[1+ps*3] * pD[0+ps*3];
	w2 += pD[2+ps*3] * pD[0+ps*3];
	w3 += pD[3+ps*3] * pD[0+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[0+ps*ii] *= tmp;
		w1 += pD[1+ps*ii] * pD[0+ps*ii];
		w2 += pD[2+ps*ii] * pD[0+ps*ii];
		w3 += pD[3+ps*ii] * pD[0+ps*ii];
		}
	//
	w1 = - dD[0] * w1;
	w2 = - dD[0] * w2;
	w3 = - dD[0] * w3;
	//
	pD[1+ps*0] += w1;
	pD[2+ps*0] += w2;
	pD[3+ps*0] += w3;
	//
	pD[1+ps*1] += w1 * pD[0+ps*1];
	pD[2+ps*1] += w2 * pD[0+ps*1];
	pD[3+ps*1] += w3 * pD[0+ps*1];
	//
	pD[1+ps*2] += w1 * pD[0+ps*2];
	pD[2+ps*2] += w2 * pD[0+ps*2];
	pD[3+ps*2] += w3 * pD[0+ps*2];
	beta = pD[1+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] += w1 * pD[0+ps*3];
	pD[2+ps*3] += w2 * pD[0+ps*3];
	pD[3+ps*3] += w3 * pD[0+ps*3];
	beta += pD[1+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] += w1 * pD[0+ps*ii];
		pD[2+ps*ii] += w2 * pD[0+ps*ii];
		pD[3+ps*ii] += w3 * pD[0+ps*ii];
		beta += pD[1+ps*ii] * pD[1+ps*ii];
		}
	// second column
col2:
	if(beta==0.0)
		{
		dD[1] = 0.0;
		tmp = 0.0;
		goto col3;
		}
	alpha = pD[1+ps*1];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[1] = (beta-alpha) / beta;
	tmp = 1.0 / (alpha-beta);
	//
	pD[1+ps*1] = beta;
	w2 = pD[2+ps*1];
	w3 = pD[3+ps*1];
	//
	pD[1+ps*2] *= tmp;
	w2 += pD[2+ps*2] * pD[1+ps*2];
	w3 += pD[3+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] *= tmp;
	w2 += pD[2+ps*3] * pD[1+ps*3];
	w3 += pD[3+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] *= tmp;
		w2 += pD[2+ps*ii] * pD[1+ps*ii];
		w3 += pD[3+ps*ii] * pD[1+ps*ii];
		}
	//
	w2 = - dD[1] * w2;
	w3 = - dD[1] * w3;
	//
	pD[2+ps*1] += w2;
	pD[3+ps*1] += w3;
	//
	pD[2+ps*2] += w2 * pD[1+ps*2];
	pD[3+ps*2] += w3 * pD[1+ps*2];
	//
	pD[2+ps*3] += w2 * pD[1+ps*3];
	pD[3+ps*3] += w3 * pD[1+ps*3];
	beta = pD[2+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] += w2 * pD[1+ps*ii];
		pD[3+ps*ii] += w3 * pD[1+ps*ii];
		beta += pD[2+ps*ii] * pD[2+ps*ii];
		}
	// third column
col3:
	if(beta==0.0)
		{
		dD[2] = 0.0;
		tmp = 0.0;
		goto col4;
		}
	alpha = pD[2+ps*2];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[2] = (beta-alpha) / beta;
	tmp = 1.0 / (alpha-beta);
	//
	pD[2+ps*2] = beta;
	w3 = pD[3+ps*2];
	//
	pD[2+ps*3] *= tmp;
	w3 += pD[3+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] *= tmp;
		w3 += pD[3+ps*ii] * pD[2+ps*ii];
		}
	//
	w3 = - dD[2] * w3;
	//
	pD[3+ps*2] += w3;
	//
	pD[3+ps*3] += w3 * pD[2+ps*3];
	//
	beta = 0.0;
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] += w3 * pD[2+ps*ii];
		beta += pD[3+ps*ii] * pD[3+ps*ii];
		}
	// fourth column
col4:
	if(beta==0.0)
		{
		dD[3] = 0.0;
		tmp = 0.0;
		return;
		}
	alpha = pD[3+ps*3];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[3] = (beta-alpha) / beta;
	tmp = 1.0 / (alpha-beta);
	//
	pD[3+ps*3] = beta;
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] *= tmp;
		}
	return;
	}



// unblocked algorithm
void kernel_dgelqf_vs_lib4(int m, int n, int k, int offD, double *pD, int sdd, double *dD)
	{
	if(m<=0 | n<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0, kmax, kmax0;
	const int ps = 4;
	imax = k;//m<n ? m : n;
	double alpha, beta, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m256d
		_a0, _b0, _t0, _w0, _w1;
	double *pC00, *pC10, *pC10a, *pC20, *pC20a, *pC01, *pC11;
	double pT[4];
	int ldt = 2;
	double *pD0 = pD-offD;
	ii = 0;
#if 1 // rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pC00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		beta = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = pC00[0+ps*jj];
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pC00[0];
			beta += alpha*alpha;
			beta = sqrt(beta);
			if(alpha>0)
				beta = -beta;
			dD[ii] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			pC00[0] = beta;
			for(jj=1; jj<n-ii; jj++)
				pC00[0+ps*jj] *= tmp;
			}
		pC10 = &pD0[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
		kmax = n-ii;
		w00 = pC10[0+ps*0]; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
			}
		w00 = - w00*dD[ii];
		pC10[0+ps*0] += w00; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
			}
		// second row
		pC11 = pC10+ps*1;
		beta = 0.0;
		for(jj=1; jj<n-(ii+1); jj++)
			{
			tmp = pC11[0+ps*jj];
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = pC11[0+ps*0];
			beta += alpha*alpha;
			beta = sqrt(beta);
			if(alpha>0)
				beta = -beta;
			dD[(ii+1)] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			pC11[0+ps*0] = beta;
			for(jj=1; jj<n-(ii+1); jj++)
				pC11[0+ps*jj] *= tmp;
			}
		// compute T
		kmax = n-ii;
		tmp = 1.0*0.0 + pC00[0+ps*1]*1.0;
		for(kk=2; kk<kmax; kk++)
			tmp += pC00[0+ps*kk]*pC10[0+ps*kk];
		pT[0+ldt*0] = - dD[ii+0];
		pT[0+ldt*1] = + dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldt*1] = - dD[ii+1];
		// downgrade
		kmax = n-ii;
		jmax = m-ii-2;
		jmax0 = (ps-((ii+2+offD)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pC20a = &pD0[((offD+ii+2)&(ps-1))+((offD+ii+2)-((offD+ii+2)&(ps-1)))*sdd+ii*ps];
		pC20 = pC20a;
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
				w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
					w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
					}
				w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
				w00 = w00*pT[0+ldt*0];
				pC20[0+ps*0] += w00*1.0          + w01*0.0;
				pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
					}
				pC20 += 1;
				}
			pC20 += -ps+ps*sdd;
			}
		for( ; jj<jmax-3; jj+=4)
			{
			//
			_w0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _a0, _b0 );
			_w0 = _mm256_add_pd( _w0, _t0 );
			_w1 = _mm256_load_pd( &pC20[0+ps*1] );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w0 = _mm256_add_pd( _w0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w1 = _mm256_add_pd( _w1, _t0 );
				}
			//
			_b0 = _mm256_broadcast_sd( &pT[1+ldt*1] );
			_w1 = _mm256_mul_pd( _w1, _b0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_w1 = _mm256_add_pd( _w1, _t0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
			_w0 = _mm256_mul_pd( _w0, _b0 );
			//
			_a0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_add_pd( _a0, _w0 );
			_mm256_store_pd( &pC20[0+ps*0], _a0 );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_a0 = _mm256_add_pd( _a0, _t0 );
			_a0 = _mm256_add_pd( _a0, _w1 );
			_mm256_store_pd( &pC20[0+ps*1], _a0 );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w0, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w1, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_mm256_store_pd( &pC20[0+ps*kk], _a0 );
				}
			pC20 += ps*sdd;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
			w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
				w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
				}
			w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
			w00 = w00*pT[0+ldt*0];
			pC20[0+ps*0] += w00*1.0          + w01*0.0;
			pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
				}
			pC20 += 1;
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		pC00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		beta = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = pC00[0+ps*jj];
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pC00[0];
			beta += alpha*alpha;
			beta = sqrt(beta);
			if(alpha>0)
				beta = -beta;
			dD[ii] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			pC00[0] = beta;
			for(jj=1; jj<n-ii; jj++)
				pC00[0+ps*jj] *= tmp;
			}
		if(ii<n)
			{
			// compute T
			pT[0+ldt*0] = - dD[ii+0];
			// downgrade
			kmax = n-ii;
			jmax = m-ii-1;
			jmax0 = (ps-((ii+1+offD)&(ps-1)))&(ps-1);
			jmax0 = jmax<jmax0 ? jmax : jmax0;
			jj = 0;
			pC10a = &pD0[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
			pC10 = pC10a;
			if(jmax0>0)
				{
				for( ; jj<jmax0; jj++)
					{
					w00 = pC10[0+ps*0];
					for(kk=1; kk<kmax; kk++)
						{
						w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
						}
					w00 = w00*pT[0+ldt*0];
					pC10[0+ps*0] += w00;
					for(kk=1; kk<kmax; kk++)
						{
						pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
						}
					pC10 += 1;
					}
				pC10 += -ps+ps*sdd;
				}
			for( ; jj<jmax-3; jj+=4)
				{
				//
				_w0 = _mm256_load_pd( &pC10[0+ps*0] );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm256_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
					_t0 = _mm256_mul_pd( _a0, _b0 );
					_w0 = _mm256_add_pd( _w0, _t0 );
					}
				//
				_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
				_w0 = _mm256_mul_pd( _w0, _b0 );
				//
				_a0 = _mm256_load_pd( &pC10[0+ps*0] );
				_a0 = _mm256_add_pd( _a0, _w0 );
				_mm256_store_pd( &pC10[0+ps*0], _a0 );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm256_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
					_t0 = _mm256_mul_pd( _w0, _b0 );
					_a0 = _mm256_add_pd( _a0, _t0 );
					_mm256_store_pd( &pC10[0+ps*kk], _a0 );
					}
				pC10 += ps*sdd;
				}
			for(ll=0; ll<jmax-jj; ll++)
				{
				w00 = pC10[0+ps*0];
				for(kk=1; kk<kmax; kk++)
					{
					w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
					}
				w00 = w00*pT[0+ldt*0];
				pC10[0+ps*0] += w00;
				for(kk=1; kk<kmax; kk++)
					{
					pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
					}
				pC10 += 1;
				}
			}
		}
	return;
	}



// assume kmax>=4
void kernel_dlarft_4_lib4(int kmax, double *pD, double *dD, double *pT)
	{
	const int ps = 4;
	int kk;
	double v10,
	       v20, v21,
		   v30, v31, v32;
	// 0
	// 1
	v10 =  pD[0+ps*1];
	// 2
	v10 += pD[1+ps*2]*pD[0+ps*2];
	v20 =  pD[0+ps*2];
	v21 =  pD[1+ps*2];
	// 3
	v10 += pD[1+ps*3]*pD[0+ps*3];
	v20 += pD[2+ps*3]*pD[0+ps*3];
	v21 += pD[2+ps*3]*pD[1+ps*3];
	v30 =  pD[0+ps*3];
	v31 =  pD[1+ps*3];
	v32 =  pD[2+ps*3];
	//
	for(kk=4; kk<kmax; kk++)
		{
		v10 += pD[1+ps*kk]*pD[0+ps*kk];
		v20 += pD[2+ps*kk]*pD[0+ps*kk];
		v30 += pD[3+ps*kk]*pD[0+ps*kk];
		v21 += pD[2+ps*kk]*pD[1+ps*kk];
		v31 += pD[3+ps*kk]*pD[1+ps*kk];
		v32 += pD[3+ps*kk]*pD[2+ps*kk];
		}
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[2+ps*2] = - dD[2];
	pT[3+ps*3] = - dD[3];
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	pT[1+ps*2] = - dD[2] * (v21*pT[1+ps*1]);
	pT[2+ps*3] = - dD[3] * (v32*pT[2+ps*2]);
	pT[0+ps*2] = - dD[2] * (v20*pT[0+ps*0] + v21*pT[0+ps*1]);
	pT[1+ps*3] = - dD[3] * (v31*pT[1+ps*1] + v32*pT[1+ps*2]);
	pT[0+ps*3] = - dD[3] * (v30*pT[0+ps*0] + v31*pT[0+ps*1] + v32*pT[0+ps*2]);
	return;
	}



// assume kmax>=3
void kernel_dlarft_3_lib4(int kmax, double *pD, double *dD, double *pT)
	{
	const int ps = 4;
	int kk;
	double v10,
	       v20, v21;
	// 0
	// 1
	v10 =  pD[0+ps*1];
	// 2
	v10 += pD[1+ps*2]*pD[0+ps*2];
	v20 =  pD[0+ps*2];
	v21 =  pD[1+ps*2];
	//
	for(kk=3; kk<kmax; kk++)
		{
		v10 += pD[1+ps*kk]*pD[0+ps*kk];
		v20 += pD[2+ps*kk]*pD[0+ps*kk];
		v21 += pD[2+ps*kk]*pD[1+ps*kk];
		}
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[2+ps*2] = - dD[2];
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	pT[1+ps*2] = - dD[2] * (v21*pT[1+ps*1]);
	pT[0+ps*2] = - dD[2] * (v20*pT[0+ps*0] + v21*pT[0+ps*1]);
	return;
	}



// assume kmax>=2
void kernel_dlarft_2_lib4(int kmax, double *pD, double *dD, double *pT)
	{
	const int ps = 4;
	int kk;
	double v10;
	// 0
	// 1
	v10 =  pD[0+ps*1];
	//
	for(kk=2; kk<kmax; kk++)
		{
		v10 += pD[1+ps*kk]*pD[0+ps*kk];
		}
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	return;
	}



// assume kmax>=1
void kernel_dlarft_1_lib4(int kmax, double *pD, double *dD, double *pT)
	{
	const int ps = 4;
	int kk;
	double v10;
	// 0
	pT[0+ps*0] = - dD[0];
	return;
	}



// assume n>=4
#if ! defined(TARGET_X64_INTEL_HASWELL)
void kernel_dgelqf_dlarft4_4_lib4(int n, double *pD, double *dD, double *pT)
	{
	int ii, jj, ll;
	double alpha, beta, tmp, w0, w1, w2, w3;
	const int ps = 4;
	// zero tau matrix
	for(ii=0; ii<16; ii++)
		pT[ii] = 0.0;
	// first column
	beta = 0.0;
	for(ii=1; ii<n; ii++)
		{
		tmp = pD[0+ps*ii];
		beta += tmp*tmp;
		}
	if(beta==0.0)
		{
		dD[0] = 0.0;
		tmp = 0.0;
		goto col2;
		}
	alpha = pD[0+ps*0];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[0] = (beta-alpha) / beta;
	pT[0+ps*0] = - dD[0];
	tmp = 1.0 / (alpha-beta);
	//
	pD[0+ps*0] = beta;
	w1 = pD[1+ps*0];
	w2 = pD[2+ps*0];
	w3 = pD[3+ps*0];
	//
	pD[0+ps*1] *= tmp;
	w1 += pD[1+ps*1] * pD[0+ps*1];
	w2 += pD[2+ps*1] * pD[0+ps*1];
	w3 += pD[3+ps*1] * pD[0+ps*1];
	//
	pD[0+ps*2] *= tmp;
	w1 += pD[1+ps*2] * pD[0+ps*2];
	w2 += pD[2+ps*2] * pD[0+ps*2];
	w3 += pD[3+ps*2] * pD[0+ps*2];
	//
	pD[0+ps*3] *= tmp;
	w1 += pD[1+ps*3] * pD[0+ps*3];
	w2 += pD[2+ps*3] * pD[0+ps*3];
	w3 += pD[3+ps*3] * pD[0+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[0+ps*ii] *= tmp;
		w1 += pD[1+ps*ii] * pD[0+ps*ii];
		w2 += pD[2+ps*ii] * pD[0+ps*ii];
		w3 += pD[3+ps*ii] * pD[0+ps*ii];
		}
	//
	w1 = - dD[0] * w1;
	w2 = - dD[0] * w2;
	w3 = - dD[0] * w3;
	//
	pD[1+ps*0] += w1;
	pD[2+ps*0] += w2;
	pD[3+ps*0] += w3;
	//
	pD[1+ps*1] += w1 * pD[0+ps*1];
	pD[2+ps*1] += w2 * pD[0+ps*1];
	pD[3+ps*1] += w3 * pD[0+ps*1];
	//
	pD[1+ps*2] += w1 * pD[0+ps*2];
	pD[2+ps*2] += w2 * pD[0+ps*2];
	pD[3+ps*2] += w3 * pD[0+ps*2];
	beta = pD[1+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] += w1 * pD[0+ps*3];
	pD[2+ps*3] += w2 * pD[0+ps*3];
	pD[3+ps*3] += w3 * pD[0+ps*3];
	beta += pD[1+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] += w1 * pD[0+ps*ii];
		pD[2+ps*ii] += w2 * pD[0+ps*ii];
		pD[3+ps*ii] += w3 * pD[0+ps*ii];
		beta += pD[1+ps*ii] * pD[1+ps*ii];
		}
	// second column
col2:
	if(beta==0.0)
		{
		dD[1] = 0.0;
		tmp = 0.0;
		goto col3;
		}
	alpha = pD[1+ps*1];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[1] = (beta-alpha) / beta;
	pT[1+ps*1] = - dD[1];
	tmp = 1.0 / (alpha-beta);
	//
	pD[1+ps*1] = beta;
	w0 = pD[0+ps*1]; //
	w2 = pD[2+ps*1];
	w3 = pD[3+ps*1];
	//
	pD[1+ps*2] *= tmp;
	w0 += pD[0+ps*2] * pD[1+ps*2]; //
	w2 += pD[2+ps*2] * pD[1+ps*2];
	w3 += pD[3+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] *= tmp;
	w0 += pD[0+ps*3] * pD[1+ps*3]; //
	w2 += pD[2+ps*3] * pD[1+ps*3];
	w3 += pD[3+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[1+ps*ii]; //
		w2 += pD[2+ps*ii] * pD[1+ps*ii];
		w3 += pD[3+ps*ii] * pD[1+ps*ii];
		}
	//
	pT[0+ps*1] = - dD[1] * (w0*pT[0+ps*0]);
	w2 = - dD[1] * w2;
	w3 = - dD[1] * w3;
	//
	pD[2+ps*1] += w2;
	pD[3+ps*1] += w3;
	//
	pD[2+ps*2] += w2 * pD[1+ps*2];
	pD[3+ps*2] += w3 * pD[1+ps*2];
	//
	pD[2+ps*3] += w2 * pD[1+ps*3];
	pD[3+ps*3] += w3 * pD[1+ps*3];
	beta = pD[2+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] += w2 * pD[1+ps*ii];
		pD[3+ps*ii] += w3 * pD[1+ps*ii];
		beta += pD[2+ps*ii] * pD[2+ps*ii];
		}
	// third column
col3:
	if(beta==0.0)
		{
		dD[2] = 0.0;
		tmp = 0.0;
		goto col4;
		}
	alpha = pD[2+ps*2];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[2] = (beta-alpha) / beta;
	pT[2+ps*2] = - dD[2];
	tmp = 1.0 / (alpha-beta);
	//
	pD[2+ps*2] = beta;
	w0 = pD[0+ps*2];
	w1 = pD[1+ps*2];
	w3 = pD[3+ps*2];
	//
	pD[2+ps*3] *= tmp;
	w0 += pD[0+ps*3] * pD[2+ps*3];
	w1 += pD[1+ps*3] * pD[2+ps*3];
	w3 += pD[3+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[2+ps*ii];
		w1 += pD[1+ps*ii] * pD[2+ps*ii];
		w3 += pD[3+ps*ii] * pD[2+ps*ii];
		}
	//
	pT[1+ps*2] = - dD[2] * (w1*pT[1+ps*1]);
	pT[0+ps*2] = - dD[2] * (w0*pT[0+ps*0] + w1*pT[0+ps*1]);
	w3 = - dD[2] * w3;
	//
	pD[3+ps*2] += w3;
	//
	pD[3+ps*3] += w3 * pD[2+ps*3];
	//
	beta = 0.0;
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] += w3 * pD[2+ps*ii];
		beta += pD[3+ps*ii] * pD[3+ps*ii];
		}
	// fourth column
col4:
	if(beta==0.0)
		{
		dD[3] = 0.0;
		tmp = 0.0;
		return;
		}
	alpha = pD[3+ps*3];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[3] = (beta-alpha) / beta;
	pT[3+ps*3] = - dD[3];
	tmp = 1.0 / (alpha-beta);
	//
	pD[3+ps*3] = beta;
	w0 =  pD[0+ps*3];
	w1 =  pD[1+ps*3];
	w2 =  pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[3+ps*ii];
		w1 += pD[1+ps*ii] * pD[3+ps*ii];
		w2 += pD[2+ps*ii] * pD[3+ps*ii];
		}
	//
	pT[2+ps*3] = - dD[3] * (w2*pT[2+ps*2]);
	pT[1+ps*3] = - dD[3] * (w1*pT[1+ps*1] + w2*pT[1+ps*2]);
	pT[0+ps*3] = - dD[3] * (w0*pT[0+ps*0] + w1*pT[0+ps*1] + w2*pT[0+ps*2]);
	return;
	}
#endif



void kernel_dlarfb3_rn_4_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	pW[1+ps*0] = pD[1+ps*0];
	pW[2+ps*0] = pD[2+ps*0];
	pW[3+ps*0] = pD[3+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[1+ps*0] += pD[1+ps*1]*pV[0+ps*1];
	pW[2+ps*0] += pD[2+ps*1]*pV[0+ps*1];
	pW[3+ps*0] += pD[3+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	pW[1+ps*1] = pD[1+ps*1];
	pW[2+ps*1] = pD[2+ps*1];
	pW[3+ps*1] = pD[3+ps*1];
	// 2
	pW[0+ps*0] += pD[0+ps*2]*pV[0+ps*2];
	pW[1+ps*0] += pD[1+ps*2]*pV[0+ps*2];
	pW[2+ps*0] += pD[2+ps*2]*pV[0+ps*2];
	pW[3+ps*0] += pD[3+ps*2]*pV[0+ps*2];
	pW[0+ps*1] += pD[0+ps*2]*pV[1+ps*2];
	pW[1+ps*1] += pD[1+ps*2]*pV[1+ps*2];
	pW[2+ps*1] += pD[2+ps*2]*pV[1+ps*2];
	pW[3+ps*1] += pD[3+ps*2]*pV[1+ps*2];
	pW[0+ps*2] = pD[0+ps*2];
	pW[1+ps*2] = pD[1+ps*2];
	pW[2+ps*2] = pD[2+ps*2];
	pW[3+ps*2] = pD[3+ps*2];
	//
	for(kk=3; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[1+ps*0] += pD[1+ps*kk]*pV[0+ps*kk];
		pW[2+ps*0] += pD[2+ps*kk]*pV[0+ps*kk];
		pW[3+ps*0] += pD[3+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[1+ps*1] += pD[1+ps*kk]*pV[1+ps*kk];
		pW[2+ps*1] += pD[2+ps*kk]*pV[1+ps*kk];
		pW[3+ps*1] += pD[3+ps*kk]*pV[1+ps*kk];
		pW[0+ps*2] += pD[0+ps*kk]*pV[2+ps*kk];
		pW[1+ps*2] += pD[1+ps*kk]*pV[2+ps*kk];
		pW[2+ps*2] += pD[2+ps*kk]*pV[2+ps*kk];
		pW[3+ps*2] += pD[3+ps*kk]*pV[2+ps*kk];
		}
	//
	pW[0+ps*2] = pW[0+ps*0]*pT[0+ps*2] + pW[0+ps*1]*pT[1+ps*2] + pW[0+ps*2]*pT[2+ps*2];
	pW[1+ps*2] = pW[1+ps*0]*pT[0+ps*2] + pW[1+ps*1]*pT[1+ps*2] + pW[1+ps*2]*pT[2+ps*2];
	pW[2+ps*2] = pW[2+ps*0]*pT[0+ps*2] + pW[2+ps*1]*pT[1+ps*2] + pW[2+ps*2]*pT[2+ps*2];
	pW[3+ps*2] = pW[3+ps*0]*pT[0+ps*2] + pW[3+ps*1]*pT[1+ps*2] + pW[3+ps*2]*pT[2+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	pW[1+ps*1] = pW[1+ps*0]*pT[0+ps*1] + pW[1+ps*1]*pT[1+ps*1];
	pW[2+ps*1] = pW[2+ps*0]*pT[0+ps*1] + pW[2+ps*1]*pT[1+ps*1];
	pW[3+ps*1] = pW[3+ps*0]*pT[0+ps*1] + pW[3+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	pW[1+ps*0] = pW[1+ps*0]*pT[0+ps*0];
	pW[2+ps*0] = pW[2+ps*0]*pT[0+ps*0];
	pW[3+ps*0] = pW[3+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	pD[1+ps*0] += pW[1+ps*0];
	pD[2+ps*0] += pW[2+ps*0];
	pD[3+ps*0] += pW[3+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	pD[1+ps*1] += pW[1+ps*0]*pV[0+ps*1] + pW[1+ps*1];
	pD[2+ps*1] += pW[2+ps*0]*pV[0+ps*1] + pW[2+ps*1];
	pD[3+ps*1] += pW[3+ps*0]*pV[0+ps*1] + pW[3+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*0]*pV[0+ps*2] + pW[0+ps*1]*pV[1+ps*2] + pW[0+ps*2];
	pD[1+ps*2] += pW[1+ps*0]*pV[0+ps*2] + pW[1+ps*1]*pV[1+ps*2] + pW[1+ps*2];
	pD[2+ps*2] += pW[2+ps*0]*pV[0+ps*2] + pW[2+ps*1]*pV[1+ps*2] + pW[2+ps*2];
	pD[3+ps*2] += pW[3+ps*0]*pV[0+ps*2] + pW[3+ps*1]*pV[1+ps*2] + pW[3+ps*2];
	for(kk=3; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk] + pW[0+ps*2]*pV[2+ps*kk];
		pD[1+ps*kk] += pW[1+ps*0]*pV[0+ps*kk] + pW[1+ps*1]*pV[1+ps*kk] + pW[1+ps*2]*pV[2+ps*kk];
		pD[2+ps*kk] += pW[2+ps*0]*pV[0+ps*kk] + pW[2+ps*1]*pV[1+ps*kk] + pW[2+ps*2]*pV[2+ps*kk];
		pD[3+ps*kk] += pW[3+ps*0]*pV[0+ps*kk] + pW[3+ps*1]*pV[1+ps*kk] + pW[3+ps*2]*pV[2+ps*kk];
		}
	return;
	}



void kernel_dlarfb2_rn_4_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	pW[1+ps*0] = pD[1+ps*0];
	pW[2+ps*0] = pD[2+ps*0];
	pW[3+ps*0] = pD[3+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[1+ps*0] += pD[1+ps*1]*pV[0+ps*1];
	pW[2+ps*0] += pD[2+ps*1]*pV[0+ps*1];
	pW[3+ps*0] += pD[3+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	pW[1+ps*1] = pD[1+ps*1];
	pW[2+ps*1] = pD[2+ps*1];
	pW[3+ps*1] = pD[3+ps*1];
	//
	for(kk=2; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[1+ps*0] += pD[1+ps*kk]*pV[0+ps*kk];
		pW[2+ps*0] += pD[2+ps*kk]*pV[0+ps*kk];
		pW[3+ps*0] += pD[3+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[1+ps*1] += pD[1+ps*kk]*pV[1+ps*kk];
		pW[2+ps*1] += pD[2+ps*kk]*pV[1+ps*kk];
		pW[3+ps*1] += pD[3+ps*kk]*pV[1+ps*kk];
		}
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	pW[1+ps*1] = pW[1+ps*0]*pT[0+ps*1] + pW[1+ps*1]*pT[1+ps*1];
	pW[2+ps*1] = pW[2+ps*0]*pT[0+ps*1] + pW[2+ps*1]*pT[1+ps*1];
	pW[3+ps*1] = pW[3+ps*0]*pT[0+ps*1] + pW[3+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	pW[1+ps*0] = pW[1+ps*0]*pT[0+ps*0];
	pW[2+ps*0] = pW[2+ps*0]*pT[0+ps*0];
	pW[3+ps*0] = pW[3+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	pD[1+ps*0] += pW[1+ps*0];
	pD[2+ps*0] += pW[2+ps*0];
	pD[3+ps*0] += pW[3+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	pD[1+ps*1] += pW[1+ps*0]*pV[0+ps*1] + pW[1+ps*1];
	pD[2+ps*1] += pW[2+ps*0]*pV[0+ps*1] + pW[2+ps*1];
	pD[3+ps*1] += pW[3+ps*0]*pV[0+ps*1] + pW[3+ps*1];
	for(kk=2; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk];
		pD[1+ps*kk] += pW[1+ps*0]*pV[0+ps*kk] + pW[1+ps*1]*pV[1+ps*kk];
		pD[2+ps*kk] += pW[2+ps*0]*pV[0+ps*kk] + pW[2+ps*1]*pV[1+ps*kk];
		pD[3+ps*kk] += pW[3+ps*0]*pV[0+ps*kk] + pW[3+ps*1]*pV[1+ps*kk];
		}
	return;
	}



void kernel_dlarfb1_rn_4_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	pW[1+ps*0] = pD[1+ps*0];
	pW[2+ps*0] = pD[2+ps*0];
	pW[3+ps*0] = pD[3+ps*0];
	//
	for(kk=1; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[1+ps*0] += pD[1+ps*kk]*pV[0+ps*kk];
		pW[2+ps*0] += pD[2+ps*kk]*pV[0+ps*kk];
		pW[3+ps*0] += pD[3+ps*kk]*pV[0+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	pW[1+ps*0] = pW[1+ps*0]*pT[0+ps*0];
	pW[2+ps*0] = pW[2+ps*0]*pT[0+ps*0];
	pW[3+ps*0] = pW[3+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	pD[1+ps*0] += pW[1+ps*0];
	pD[2+ps*0] += pW[2+ps*0];
	pD[3+ps*0] += pW[3+ps*0];
	for(kk=1; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk];
		pD[1+ps*kk] += pW[1+ps*0]*pV[0+ps*kk];
		pD[2+ps*kk] += pW[2+ps*0]*pV[0+ps*kk];
		pD[3+ps*kk] += pW[3+ps*0]*pV[0+ps*kk];
		}
	return;
	}



void kernel_dlarfb4_rn_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	// 2
	pW[0+ps*0] += pD[0+ps*2]*pV[0+ps*2];
	pW[0+ps*1] += pD[0+ps*2]*pV[1+ps*2];
	pW[0+ps*2] = pD[0+ps*2];
	// 3
	pW[0+ps*0] += pD[0+ps*3]*pV[0+ps*3];
	pW[0+ps*1] += pD[0+ps*3]*pV[1+ps*3];
	pW[0+ps*2] += pD[0+ps*3]*pV[2+ps*3];
	pW[0+ps*3] = pD[0+ps*3];
	//
	for(kk=4; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[0+ps*2] += pD[0+ps*kk]*pV[2+ps*kk];
		pW[0+ps*3] += pD[0+ps*kk]*pV[3+ps*kk];
		}
	//
	pW[0+ps*3] = pW[0+ps*0]*pT[0+ps*3] + pW[0+ps*1]*pT[1+ps*3] + pW[0+ps*2]*pT[2+ps*3] + pW[0+ps*3]*pT[3+ps*3];
	//
	pW[0+ps*2] = pW[0+ps*0]*pT[0+ps*2] + pW[0+ps*1]*pT[1+ps*2] + pW[0+ps*2]*pT[2+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*0]*pV[0+ps*2] + pW[0+ps*1]*pV[1+ps*2] + pW[0+ps*2];
	//
	pD[0+ps*3] += pW[0+ps*0]*pV[0+ps*3] + pW[0+ps*1]*pV[1+ps*3] + pW[0+ps*2]*pV[2+ps*3] + pW[0+ps*3];
	for(kk=4; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk] + pW[0+ps*2]*pV[2+ps*kk] + pW[0+ps*3]*pV[3+ps*kk];
		}
	return;
	}



void kernel_dlarfb3_rn_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	// 2
	pW[0+ps*0] += pD[0+ps*2]*pV[0+ps*2];
	pW[0+ps*1] += pD[0+ps*2]*pV[1+ps*2];
	pW[0+ps*2] = pD[0+ps*2];
	//
	for(kk=3; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[0+ps*2] += pD[0+ps*kk]*pV[2+ps*kk];
		}
	pW[0+ps*2] = pW[0+ps*0]*pT[0+ps*2] + pW[0+ps*1]*pT[1+ps*2] + pW[0+ps*2]*pT[2+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*0]*pV[0+ps*2] + pW[0+ps*1]*pV[1+ps*2] + pW[0+ps*2];
	for(kk=3; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk] + pW[0+ps*2]*pV[2+ps*kk];
		}
	return;
	}



void kernel_dlarfb2_rn_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	//
	for(kk=2; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		}
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	for(kk=2; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk];
		}
	return;
	}



void kernel_dlarfb1_rn_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	//
	for(kk=1; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		}
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	for(kk=1; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk];
		}
	return;
	}



void kernel_dlarfb3_rt_4_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	pW[1+ps*0] = pD[1+ps*0];
	pW[2+ps*0] = pD[2+ps*0];
	pW[3+ps*0] = pD[3+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[1+ps*0] += pD[1+ps*1]*pV[0+ps*1];
	pW[2+ps*0] += pD[2+ps*1]*pV[0+ps*1];
	pW[3+ps*0] += pD[3+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	pW[1+ps*1] = pD[1+ps*1];
	pW[2+ps*1] = pD[2+ps*1];
	pW[3+ps*1] = pD[3+ps*1];
	// 2
	pW[0+ps*0] += pD[0+ps*2]*pV[0+ps*2];
	pW[1+ps*0] += pD[1+ps*2]*pV[0+ps*2];
	pW[2+ps*0] += pD[2+ps*2]*pV[0+ps*2];
	pW[3+ps*0] += pD[3+ps*2]*pV[0+ps*2];
	pW[0+ps*1] += pD[0+ps*2]*pV[1+ps*2];
	pW[1+ps*1] += pD[1+ps*2]*pV[1+ps*2];
	pW[2+ps*1] += pD[2+ps*2]*pV[1+ps*2];
	pW[3+ps*1] += pD[3+ps*2]*pV[1+ps*2];
	pW[0+ps*2] = pD[0+ps*2];
	pW[1+ps*2] = pD[1+ps*2];
	pW[2+ps*2] = pD[2+ps*2];
	pW[3+ps*2] = pD[3+ps*2];
	//
	for(kk=3; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[1+ps*0] += pD[1+ps*kk]*pV[0+ps*kk];
		pW[2+ps*0] += pD[2+ps*kk]*pV[0+ps*kk];
		pW[3+ps*0] += pD[3+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[1+ps*1] += pD[1+ps*kk]*pV[1+ps*kk];
		pW[2+ps*1] += pD[2+ps*kk]*pV[1+ps*kk];
		pW[3+ps*1] += pD[3+ps*kk]*pV[1+ps*kk];
		pW[0+ps*2] += pD[0+ps*kk]*pV[2+ps*kk];
		pW[1+ps*2] += pD[1+ps*kk]*pV[2+ps*kk];
		pW[2+ps*2] += pD[2+ps*kk]*pV[2+ps*kk];
		pW[3+ps*2] += pD[3+ps*kk]*pV[2+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0] + pW[0+ps*1]*pT[0+ps*1] + pW[0+ps*2]*pT[0+ps*2];
	pW[1+ps*0] = pW[1+ps*0]*pT[0+ps*0] + pW[1+ps*1]*pT[0+ps*1] + pW[1+ps*2]*pT[0+ps*2];
	pW[2+ps*0] = pW[2+ps*0]*pT[0+ps*0] + pW[2+ps*1]*pT[0+ps*1] + pW[2+ps*2]*pT[0+ps*2];
	pW[3+ps*0] = pW[3+ps*0]*pT[0+ps*0] + pW[3+ps*1]*pT[0+ps*1] + pW[3+ps*2]*pT[0+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*1]*pT[1+ps*1] + pW[0+ps*2]*pT[1+ps*2];
	pW[1+ps*1] = pW[1+ps*1]*pT[1+ps*1] + pW[1+ps*2]*pT[1+ps*2];
	pW[2+ps*1] = pW[2+ps*1]*pT[1+ps*1] + pW[2+ps*2]*pT[1+ps*2];
	pW[3+ps*1] = pW[3+ps*1]*pT[1+ps*1] + pW[3+ps*2]*pT[1+ps*2];
	//
	pW[0+ps*2] = pW[0+ps*2]*pT[2+ps*2];
	pW[1+ps*2] = pW[1+ps*2]*pT[2+ps*2];
	pW[2+ps*2] = pW[2+ps*2]*pT[2+ps*2];
	pW[3+ps*2] = pW[3+ps*2]*pT[2+ps*2];
	//
	pD[0+ps*0] += pW[0+ps*0];
	pD[1+ps*0] += pW[1+ps*0];
	pD[2+ps*0] += pW[2+ps*0];
	pD[3+ps*0] += pW[3+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	pD[1+ps*1] += pW[1+ps*0]*pV[0+ps*1] + pW[1+ps*1];
	pD[2+ps*1] += pW[2+ps*0]*pV[0+ps*1] + pW[2+ps*1];
	pD[3+ps*1] += pW[3+ps*0]*pV[0+ps*1] + pW[3+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*0]*pV[0+ps*2] + pW[0+ps*1]*pV[1+ps*2] + pW[0+ps*2];
	pD[1+ps*2] += pW[1+ps*0]*pV[0+ps*2] + pW[1+ps*1]*pV[1+ps*2] + pW[1+ps*2];
	pD[2+ps*2] += pW[2+ps*0]*pV[0+ps*2] + pW[2+ps*1]*pV[1+ps*2] + pW[2+ps*2];
	pD[3+ps*2] += pW[3+ps*0]*pV[0+ps*2] + pW[3+ps*1]*pV[1+ps*2] + pW[3+ps*2];
	for(kk=3; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk] + pW[0+ps*2]*pV[2+ps*kk];
		pD[1+ps*kk] += pW[1+ps*0]*pV[0+ps*kk] + pW[1+ps*1]*pV[1+ps*kk] + pW[1+ps*2]*pV[2+ps*kk];
		pD[2+ps*kk] += pW[2+ps*0]*pV[0+ps*kk] + pW[2+ps*1]*pV[1+ps*kk] + pW[2+ps*2]*pV[2+ps*kk];
		pD[3+ps*kk] += pW[3+ps*0]*pV[0+ps*kk] + pW[3+ps*1]*pV[1+ps*kk] + pW[3+ps*2]*pV[2+ps*kk];
		}
	return;
	}



void kernel_dlarfb2_rt_4_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	pW[1+ps*0] = pD[1+ps*0];
	pW[2+ps*0] = pD[2+ps*0];
	pW[3+ps*0] = pD[3+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[1+ps*0] += pD[1+ps*1]*pV[0+ps*1];
	pW[2+ps*0] += pD[2+ps*1]*pV[0+ps*1];
	pW[3+ps*0] += pD[3+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	pW[1+ps*1] = pD[1+ps*1];
	pW[2+ps*1] = pD[2+ps*1];
	pW[3+ps*1] = pD[3+ps*1];
	//
	for(kk=2; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[1+ps*0] += pD[1+ps*kk]*pV[0+ps*kk];
		pW[2+ps*0] += pD[2+ps*kk]*pV[0+ps*kk];
		pW[3+ps*0] += pD[3+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[1+ps*1] += pD[1+ps*kk]*pV[1+ps*kk];
		pW[2+ps*1] += pD[2+ps*kk]*pV[1+ps*kk];
		pW[3+ps*1] += pD[3+ps*kk]*pV[1+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0] + pW[0+ps*1]*pT[0+ps*1];
	pW[1+ps*0] = pW[1+ps*0]*pT[0+ps*0] + pW[1+ps*1]*pT[0+ps*1];
	pW[2+ps*0] = pW[2+ps*0]*pT[0+ps*0] + pW[2+ps*1]*pT[0+ps*1];
	pW[3+ps*0] = pW[3+ps*0]*pT[0+ps*0] + pW[3+ps*1]*pT[0+ps*1];
	//
	pW[0+ps*1] = pW[0+ps*1]*pT[1+ps*1];
	pW[1+ps*1] = pW[1+ps*1]*pT[1+ps*1];
	pW[2+ps*1] = pW[2+ps*1]*pT[1+ps*1];
	pW[3+ps*1] = pW[3+ps*1]*pT[1+ps*1];
	//
	pD[0+ps*0] += pW[0+ps*0];
	pD[1+ps*0] += pW[1+ps*0];
	pD[2+ps*0] += pW[2+ps*0];
	pD[3+ps*0] += pW[3+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	pD[1+ps*1] += pW[1+ps*0]*pV[0+ps*1] + pW[1+ps*1];
	pD[2+ps*1] += pW[2+ps*0]*pV[0+ps*1] + pW[2+ps*1];
	pD[3+ps*1] += pW[3+ps*0]*pV[0+ps*1] + pW[3+ps*1];
	for(kk=2; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk];
		pD[1+ps*kk] += pW[1+ps*0]*pV[0+ps*kk] + pW[1+ps*1]*pV[1+ps*kk];
		pD[2+ps*kk] += pW[2+ps*0]*pV[0+ps*kk] + pW[2+ps*1]*pV[1+ps*kk];
		pD[3+ps*kk] += pW[3+ps*0]*pV[0+ps*kk] + pW[3+ps*1]*pV[1+ps*kk];
		}
	return;
	}



void kernel_dlarfb1_rt_4_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	pW[1+ps*0] = pD[1+ps*0];
	pW[2+ps*0] = pD[2+ps*0];
	pW[3+ps*0] = pD[3+ps*0];
	//
	for(kk=1; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[1+ps*0] += pD[1+ps*kk]*pV[0+ps*kk];
		pW[2+ps*0] += pD[2+ps*kk]*pV[0+ps*kk];
		pW[3+ps*0] += pD[3+ps*kk]*pV[0+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	pW[1+ps*0] = pW[1+ps*0]*pT[0+ps*0];
	pW[2+ps*0] = pW[2+ps*0]*pT[0+ps*0];
	pW[3+ps*0] = pW[3+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	pD[1+ps*0] += pW[1+ps*0];
	pD[2+ps*0] += pW[2+ps*0];
	pD[3+ps*0] += pW[3+ps*0];
	for(kk=1; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk];
		pD[1+ps*kk] += pW[1+ps*0]*pV[0+ps*kk];
		pD[2+ps*kk] += pW[2+ps*0]*pV[0+ps*kk];
		pD[3+ps*kk] += pW[3+ps*0]*pV[0+ps*kk];
		}
	return;
	}



void kernel_dlarfb4_rt_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	// 2
	pW[0+ps*0] += pD[0+ps*2]*pV[0+ps*2];
	pW[0+ps*1] += pD[0+ps*2]*pV[1+ps*2];
	pW[0+ps*2] = pD[0+ps*2];
	// 3
	pW[0+ps*0] += pD[0+ps*3]*pV[0+ps*3];
	pW[0+ps*1] += pD[0+ps*3]*pV[1+ps*3];
	pW[0+ps*2] += pD[0+ps*3]*pV[2+ps*3];
	pW[0+ps*3] = pD[0+ps*3];
	//
	for(kk=4; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[0+ps*2] += pD[0+ps*kk]*pV[2+ps*kk];
		pW[0+ps*3] += pD[0+ps*kk]*pV[3+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0] + pW[0+ps*1]*pT[0+ps*1] + pW[0+ps*2]*pT[0+ps*2] + pW[0+ps*3]*pT[0+ps*3];
	//
	pW[0+ps*1] = pW[0+ps*1]*pT[1+ps*1] + pW[0+ps*2]*pT[1+ps*2] + pW[0+ps*3]*pT[1+ps*3];
	//
	pW[0+ps*2] = pW[0+ps*2]*pT[2+ps*2] + pW[0+ps*3]*pT[2+ps*3];
	//
	pW[0+ps*3] = pW[0+ps*3]*pT[3+ps*3];
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*0]*pV[0+ps*2] + pW[0+ps*1]*pV[1+ps*2] + pW[0+ps*2];
	//
	pD[0+ps*3] += pW[0+ps*0]*pV[0+ps*3] + pW[0+ps*1]*pV[1+ps*3] + pW[0+ps*2]*pV[2+ps*3] + pW[0+ps*3];
	//
	for(kk=4; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk] + pW[0+ps*2]*pV[2+ps*kk] + pW[0+ps*3]*pV[3+ps*kk];
		}
	return;
	}



void kernel_dlarfb3_rt_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	// 2
	pW[0+ps*0] += pD[0+ps*2]*pV[0+ps*2];
	pW[0+ps*1] += pD[0+ps*2]*pV[1+ps*2];
	pW[0+ps*2] = pD[0+ps*2];
	//
	for(kk=3; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[0+ps*2] += pD[0+ps*kk]*pV[2+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0] + pW[0+ps*1]*pT[0+ps*1] + pW[0+ps*2]*pT[0+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*1]*pT[1+ps*1] + pW[0+ps*2]*pT[1+ps*2];
	//
	pW[0+ps*2] = pW[0+ps*2]*pT[2+ps*2];
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*0]*pV[0+ps*2] + pW[0+ps*1]*pV[1+ps*2] + pW[0+ps*2];
	for(kk=3; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk] + pW[0+ps*2]*pV[2+ps*kk];
		}
	return;
	}



void kernel_dlarfb2_rt_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*0] += pD[0+ps*1]*pV[0+ps*1];
	pW[0+ps*1] = pD[0+ps*1];
	//
	for(kk=2; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0] + pW[0+ps*1]*pT[0+ps*1];
	//
	pW[0+ps*1] = pW[0+ps*1]*pT[1+ps*1];
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*0]*pV[0+ps*1] + pW[0+ps*1];
	for(kk=2; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk];
		}
	return;
	}



void kernel_dlarfb1_rt_1_lib4(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	//
	for(kk=1; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		}
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	for(kk=1; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk];
		}
	return;
	}



// assume n>=4
// positive diagonal
#if ! defined(TARGET_X64_INTEL_HASWELL)
void kernel_dgelqf_pd_dlarft4_4_lib4(int n, double *pD, double *dD, double *pT)
	{
	int ii, jj, ll;
	double alpha, beta, sigma, tmp, w0, w1, w2, w3;
	const int ps = 4;
	// zero tau matrix
	for(ii=0; ii<16; ii++)
		pT[ii] = 0.0;
	// first column
	sigma = 0.0;
	for(ii=1; ii<n; ii++)
		{
		tmp = pD[0+ps*ii];
		sigma += tmp*tmp;
		}
	if(sigma==0.0)
		{
		dD[0] = 0.0;
		tmp = 0.0;
		goto col2;
		}
	alpha = pD[0+ps*0];
	beta = sigma + alpha*alpha;
	beta = sqrt(beta);
	if(alpha<=0)
		tmp = alpha-beta;
	else
		tmp = -sigma / (alpha+beta);
	dD[0] = 2*tmp*tmp / (sigma+tmp*tmp);
	pT[0+ps*0] = - dD[0];
	tmp = 1.0 / tmp;
	//
	pD[0+ps*0] = beta;
	w1 = pD[1+ps*0];
	w2 = pD[2+ps*0];
	w3 = pD[3+ps*0];
	//
	pD[0+ps*1] *= tmp;
	w1 += pD[1+ps*1] * pD[0+ps*1];
	w2 += pD[2+ps*1] * pD[0+ps*1];
	w3 += pD[3+ps*1] * pD[0+ps*1];
	//
	pD[0+ps*2] *= tmp;
	w1 += pD[1+ps*2] * pD[0+ps*2];
	w2 += pD[2+ps*2] * pD[0+ps*2];
	w3 += pD[3+ps*2] * pD[0+ps*2];
	//
	pD[0+ps*3] *= tmp;
	w1 += pD[1+ps*3] * pD[0+ps*3];
	w2 += pD[2+ps*3] * pD[0+ps*3];
	w3 += pD[3+ps*3] * pD[0+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[0+ps*ii] *= tmp;
		w1 += pD[1+ps*ii] * pD[0+ps*ii];
		w2 += pD[2+ps*ii] * pD[0+ps*ii];
		w3 += pD[3+ps*ii] * pD[0+ps*ii];
		}
	//
	w1 = - dD[0] * w1;
	w2 = - dD[0] * w2;
	w3 = - dD[0] * w3;
	//
	pD[1+ps*0] += w1;
	pD[2+ps*0] += w2;
	pD[3+ps*0] += w3;
	//
	pD[1+ps*1] += w1 * pD[0+ps*1];
	pD[2+ps*1] += w2 * pD[0+ps*1];
	pD[3+ps*1] += w3 * pD[0+ps*1];
	//
	pD[1+ps*2] += w1 * pD[0+ps*2];
	pD[2+ps*2] += w2 * pD[0+ps*2];
	pD[3+ps*2] += w3 * pD[0+ps*2];
	sigma = pD[1+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] += w1 * pD[0+ps*3];
	pD[2+ps*3] += w2 * pD[0+ps*3];
	pD[3+ps*3] += w3 * pD[0+ps*3];
	sigma += pD[1+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] += w1 * pD[0+ps*ii];
		pD[2+ps*ii] += w2 * pD[0+ps*ii];
		pD[3+ps*ii] += w3 * pD[0+ps*ii];
		sigma += pD[1+ps*ii] * pD[1+ps*ii];
		}
	// second column
col2:
	if(sigma==0.0)
		{
		dD[1] = 0.0;
		tmp = 0.0;
		goto col3;
		}
	alpha = pD[1+ps*1];
	beta = sigma + alpha*alpha;
	beta = sqrt(beta);
	if(alpha<=0)
		tmp = alpha-beta;
	else
		tmp = -sigma / (alpha+beta);
	dD[1] = 2*tmp*tmp / (sigma+tmp*tmp);;
	pT[1+ps*1] = - dD[1];
	tmp = 1.0 / tmp;
	//
	pD[1+ps*1] = beta;
	w0 = pD[0+ps*1]; //
	w2 = pD[2+ps*1];
	w3 = pD[3+ps*1];
	//
	pD[1+ps*2] *= tmp;
	w0 += pD[0+ps*2] * pD[1+ps*2]; //
	w2 += pD[2+ps*2] * pD[1+ps*2];
	w3 += pD[3+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] *= tmp;
	w0 += pD[0+ps*3] * pD[1+ps*3]; //
	w2 += pD[2+ps*3] * pD[1+ps*3];
	w3 += pD[3+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[1+ps*ii]; //
		w2 += pD[2+ps*ii] * pD[1+ps*ii];
		w3 += pD[3+ps*ii] * pD[1+ps*ii];
		}
	//
	pT[0+ps*1] = - dD[1] * (w0*pT[0+ps*0]);
	w2 = - dD[1] * w2;
	w3 = - dD[1] * w3;
	//
	pD[2+ps*1] += w2;
	pD[3+ps*1] += w3;
	//
	pD[2+ps*2] += w2 * pD[1+ps*2];
	pD[3+ps*2] += w3 * pD[1+ps*2];
	//
	pD[2+ps*3] += w2 * pD[1+ps*3];
	pD[3+ps*3] += w3 * pD[1+ps*3];
	sigma = pD[2+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] += w2 * pD[1+ps*ii];
		pD[3+ps*ii] += w3 * pD[1+ps*ii];
		sigma += pD[2+ps*ii] * pD[2+ps*ii];
		}
	// third column
col3:
	if(sigma==0.0)
		{
		dD[2] = 0.0;
		tmp = 0.0;
		goto col4;
		}
	alpha = pD[2+ps*2];
	beta = sigma + alpha*alpha;
	beta = sqrt(beta);
	if(alpha<=0)
		tmp = alpha-beta;
	else
		tmp = -sigma / (alpha+beta);
	dD[2] = 2*tmp*tmp / (sigma+tmp*tmp);;
	pT[2+ps*2] = - dD[2];
	tmp = 1.0 / tmp;
	//
	pD[2+ps*2] = beta;
	w0 = pD[0+ps*2];
	w1 = pD[1+ps*2];
	w3 = pD[3+ps*2];
	//
	pD[2+ps*3] *= tmp;
	w0 += pD[0+ps*3] * pD[2+ps*3];
	w1 += pD[1+ps*3] * pD[2+ps*3];
	w3 += pD[3+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[2+ps*ii];
		w1 += pD[1+ps*ii] * pD[2+ps*ii];
		w3 += pD[3+ps*ii] * pD[2+ps*ii];
		}
	//
	pT[0+ps*2] = - dD[2] * (w0*pT[0+ps*0] + w1*pT[0+ps*1]);
	pT[1+ps*2] = - dD[2] * (w1*pT[1+ps*1]);
	w3 = - dD[2] * w3;
	//
	pD[3+ps*2] += w3;
	//
	pD[3+ps*3] += w3 * pD[2+ps*3];
	//
	sigma = 0.0;
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] += w3 * pD[2+ps*ii];
		sigma += pD[3+ps*ii] * pD[3+ps*ii];
		}
	// fourth column
col4:
	if(sigma==0.0)
		{
		dD[3] = 0.0;
		tmp = 0.0;
		return;
		}
	alpha = pD[3+ps*3];
	beta = sigma + alpha*alpha;
	beta = sqrt(beta);
	if(alpha<=0)
		tmp = alpha-beta;
	else
		tmp = -sigma / (alpha+beta);
	dD[3] = 2*tmp*tmp / (sigma+tmp*tmp);;
	pT[3+ps*3] = - dD[3];
	tmp = 1.0 / tmp;
	//
	pD[3+ps*3] = beta;
	w0 =  pD[0+ps*3];
	w1 =  pD[1+ps*3];
	w2 =  pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[3+ps*ii];
		w1 += pD[1+ps*ii] * pD[3+ps*ii];
		w2 += pD[2+ps*ii] * pD[3+ps*ii];
		}
	//
	pT[0+ps*3] = - dD[3] * (w0*pT[0+ps*0] + w1*pT[0+ps*1] + w2*pT[0+ps*2]);
	pT[1+ps*3] = - dD[3] * (w1*pT[1+ps*1] + w2*pT[1+ps*2]);
	pT[2+ps*3] = - dD[3] * (w2*pT[2+ps*2]);
	return;
	}
#endif



// assume n>=4
// positive diagonal
void kernel_dgelqf_pd_4_lib4(int n, double *pD, double *dD)
	{
	int ii, jj, ll;
	double alpha, beta, sigma, tmp, w1, w2, w3;
	const int ps = 4;
	// first column
	sigma = 0.0;
	for(ii=1; ii<n; ii++)
		{
		tmp = pD[0+ps*ii];
		sigma += tmp*tmp;
		}
	if(sigma==0.0)
		{
		// tau
		dD[0] = 0.0;
		}
	else
		{
		alpha = pD[0+ps*0];
		beta = sigma + alpha*alpha;
		beta = sqrt(beta);
		if(alpha<=0)
			tmp = alpha-beta;
		else
			tmp = -sigma / (alpha+beta);
		// tau0
		dD[0] = 2*tmp*tmp / (sigma+tmp*tmp);
		tmp = 1.0 / tmp;
		// compute v0
		pD[0+ps*0] = beta;
		for(ii=1; ii<n; ii++)
			{
			pD[0+ps*ii] *= tmp;
			}
		}
	// gemv_t & ger
	w1 = pD[1+ps*0];
	w2 = pD[2+ps*0];
	w3 = pD[3+ps*0];
	w1 += pD[1+ps*1] * pD[0+ps*1];
	w2 += pD[2+ps*1] * pD[0+ps*1];
	w3 += pD[3+ps*1] * pD[0+ps*1];
	w1 += pD[1+ps*2] * pD[0+ps*2];
	w2 += pD[2+ps*2] * pD[0+ps*2];
	w3 += pD[3+ps*2] * pD[0+ps*2];
	w1 += pD[1+ps*3] * pD[0+ps*3];
	w2 += pD[2+ps*3] * pD[0+ps*3];
	w3 += pD[3+ps*3] * pD[0+ps*3];
	for(ii=4; ii<n; ii++)
		{
		w1 += pD[1+ps*ii] * pD[0+ps*ii];
		w2 += pD[2+ps*ii] * pD[0+ps*ii];
		w3 += pD[3+ps*ii] * pD[0+ps*ii];
		}
	w1 = - dD[0] * w1;
	w2 = - dD[0] * w2;
	w3 = - dD[0] * w3;
	pD[1+ps*0] += w1;
	pD[2+ps*0] += w2;
	pD[3+ps*0] += w3;
	pD[1+ps*1] += w1 * pD[0+ps*1];
	pD[2+ps*1] += w2 * pD[0+ps*1];
	pD[3+ps*1] += w3 * pD[0+ps*1];
	pD[1+ps*2] += w1 * pD[0+ps*2];
	pD[2+ps*2] += w2 * pD[0+ps*2];
	pD[3+ps*2] += w3 * pD[0+ps*2];
	pD[1+ps*3] += w1 * pD[0+ps*3];
	pD[2+ps*3] += w2 * pD[0+ps*3];
	pD[3+ps*3] += w3 * pD[0+ps*3];
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] += w1 * pD[0+ps*ii];
		pD[2+ps*ii] += w2 * pD[0+ps*ii];
		pD[3+ps*ii] += w3 * pD[0+ps*ii];
		}
	// second column
	sigma = 0.0;
	for(ii=2; ii<n; ii++)
		{
		tmp = pD[1+ps*ii];
		sigma += tmp*tmp;
		}
	if(sigma==0.0)
		{
		// tau
		dD[1] = 0.0;
		}
	else
		{
		alpha = pD[1+ps*1];
		beta = sigma + alpha*alpha;
		beta = sqrt(beta);
		if(alpha<=0)
			tmp = alpha-beta;
		else
			tmp = -sigma / (alpha+beta);
		// tau0
		dD[1] = 2*tmp*tmp / (sigma+tmp*tmp);
		tmp = 1.0 / tmp;
		// compute v0
		pD[1+ps*1] = beta;
		for(ii=2; ii<n; ii++)
			{
			pD[1+ps*ii] *= tmp;
			}
		}
	// gemv_t & ger
	w2 = pD[2+ps*1];
	w3 = pD[3+ps*1];
	w2 += pD[2+ps*2] * pD[1+ps*2];
	w3 += pD[3+ps*2] * pD[1+ps*2];
	w2 += pD[2+ps*3] * pD[1+ps*3];
	w3 += pD[3+ps*3] * pD[1+ps*3];
	for(ii=4; ii<n; ii++)
		{
		w2 += pD[2+ps*ii] * pD[1+ps*ii];
		w3 += pD[3+ps*ii] * pD[1+ps*ii];
		}
	w2 = - dD[1] * w2;
	w3 = - dD[1] * w3;
	pD[2+ps*1] += w2;
	pD[3+ps*1] += w3;
	pD[2+ps*2] += w2 * pD[1+ps*2];
	pD[3+ps*2] += w3 * pD[1+ps*2];
	pD[2+ps*3] += w2 * pD[1+ps*3];
	pD[3+ps*3] += w3 * pD[1+ps*3];
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] += w2 * pD[1+ps*ii];
		pD[3+ps*ii] += w3 * pD[1+ps*ii];
		}
	// third column
	sigma = 0.0;
	for(ii=3; ii<n; ii++)
		{
		tmp = pD[2+ps*ii];
		sigma += tmp*tmp;
		}
	if(sigma==0.0)
		{
		// tau
		dD[2] = 0.0;
		}
	else
		{
		alpha = pD[2+ps*2];
		beta = sigma + alpha*alpha;
		beta = sqrt(beta);
		if(alpha<=0)
			tmp = alpha-beta;
		else
			tmp = -sigma / (alpha+beta);
		// tau0
		dD[2] = 2*tmp*tmp / (sigma+tmp*tmp);
		tmp = 1.0 / tmp;
		// compute v0
		pD[2+ps*2] = beta;
		for(ii=3; ii<n; ii++)
			{
			pD[2+ps*ii] *= tmp;
			}
		}
	// gemv_t & ger
	w3 = pD[3+ps*2];
	w3 += pD[3+ps*3] * pD[2+ps*3];
	for(ii=4; ii<n; ii++)
		{
		w3 += pD[3+ps*ii] * pD[2+ps*ii];
		}
	w3 = - dD[2] * w3;
	pD[3+ps*2] += w3;
	pD[3+ps*3] += w3 * pD[2+ps*3];
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] += w3 * pD[2+ps*ii];
		}
	// fourth column
	sigma = 0.0;
	for(ii=4; ii<n; ii++)
		{
		tmp = pD[3+ps*ii];
		sigma += tmp*tmp;
		}
	if(sigma==0.0)
		{
		// tau
		dD[3] = 0.0;
		}
	else
		{
		alpha = pD[3+ps*3];
		beta = sigma + alpha*alpha;
		beta = sqrt(beta);
		if(alpha<=0)
			tmp = alpha-beta;
		else
			tmp = -sigma / (alpha+beta);
		// tau0
		dD[3] = 2*tmp*tmp / (sigma+tmp*tmp);
		tmp = 1.0 / tmp;
		// compute v0
		pD[3+ps*3] = beta;
		for(ii=4; ii<n; ii++)
			{
			pD[3+ps*ii] *= tmp;
			}
		}
	return;
	}



// unblocked algorithm
// positive diagonal
void kernel_dgelqf_pd_vs_lib4(int m, int n, int k, int offD, double *pD, int sdd, double *dD)
	{
	if(m<=0 | n<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0, kmax, kmax0;
	const int ps = 4;
	imax = k;//m<n ? m : n;
	double alpha, beta, sigma, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m256d
		_a0, _b0, _t0, _w0, _w1;
	double *pC00, *pC10, *pC10a, *pC20, *pC20a, *pC01, *pC11;
	double pT[4];
	int ldt = 2;
	double *pD0 = pD-offD;
	ii = 0;
#if 1 // rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pC00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		sigma = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = pC00[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pC00[0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pC00[0] = beta;
			for(jj=1; jj<n-ii; jj++)
				pC00[0+ps*jj] *= tmp;
			}
		pC10 = &pD0[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
		kmax = n-ii;
		w00 = pC10[0+ps*0]; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
			}
		w00 = - w00*dD[ii];
		pC10[0+ps*0] += w00; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
			}
		// second row
		pC11 = pC10+ps*1;
		sigma = 0.0;
		for(jj=1; jj<n-(ii+1); jj++)
			{
			tmp = pC11[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = pC11[0+ps*0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii+1] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pC11[0+ps*0] = beta;
			for(jj=1; jj<n-(ii+1); jj++)
				pC11[0+ps*jj] *= tmp;
			}
		// compute T
		kmax = n-ii;
		tmp = 1.0*0.0 + pC00[0+ps*1]*1.0;
		for(kk=2; kk<kmax; kk++)
			tmp += pC00[0+ps*kk]*pC10[0+ps*kk];
		pT[0+ldt*0] = - dD[ii+0];
		pT[0+ldt*1] = + dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldt*1] = - dD[ii+1];
		// downgrade
		kmax = n-ii;
		jmax = m-ii-2;
		jmax0 = (ps-((ii+2+offD)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pC20a = &pD0[((offD+ii+2)&(ps-1))+((offD+ii+2)-((offD+ii+2)&(ps-1)))*sdd+ii*ps];
		pC20 = pC20a;
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
				w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
					w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
					}
				w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
				w00 = w00*pT[0+ldt*0];
				pC20[0+ps*0] += w00*1.0          + w01*0.0;
				pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
					}
				pC20 += 1;
				}
			pC20 += -ps+ps*sdd;
			}
		for( ; jj<jmax-3; jj+=4)
			{
			//
			_w0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _a0, _b0 );
			_w0 = _mm256_add_pd( _w0, _t0 );
			_w1 = _mm256_load_pd( &pC20[0+ps*1] );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w0 = _mm256_add_pd( _w0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w1 = _mm256_add_pd( _w1, _t0 );
				}
			//
			_b0 = _mm256_broadcast_sd( &pT[1+ldt*1] );
			_w1 = _mm256_mul_pd( _w1, _b0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_w1 = _mm256_add_pd( _w1, _t0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
			_w0 = _mm256_mul_pd( _w0, _b0 );
			//
			_a0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_add_pd( _a0, _w0 );
			_mm256_store_pd( &pC20[0+ps*0], _a0 );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_a0 = _mm256_add_pd( _a0, _t0 );
			_a0 = _mm256_add_pd( _a0, _w1 );
			_mm256_store_pd( &pC20[0+ps*1], _a0 );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w0, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w1, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_mm256_store_pd( &pC20[0+ps*kk], _a0 );
				}
			pC20 += ps*sdd;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
			w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
				w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
				}
			w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
			w00 = w00*pT[0+ldt*0];
			pC20[0+ps*0] += w00*1.0          + w01*0.0;
			pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
				}
			pC20 += 1;
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		pC00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		sigma = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = pC00[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pC00[0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pC00[0] = beta;
			for(jj=1; jj<n-ii; jj++)
				pC00[0+ps*jj] *= tmp;
			}
		if(ii<n)
			{
			// compute T
			pT[0+ldt*0] = - dD[ii+0];
			// downgrade
			kmax = n-ii;
			jmax = m-ii-1;
			jmax0 = (ps-((ii+1+offD)&(ps-1)))&(ps-1);
			jmax0 = jmax<jmax0 ? jmax : jmax0;
			jj = 0;
			pC10a = &pD0[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
			pC10 = pC10a;
			if(jmax0>0)
				{
				for( ; jj<jmax0; jj++)
					{
					w00 = pC10[0+ps*0];
					for(kk=1; kk<kmax; kk++)
						{
						w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
						}
					w00 = w00*pT[0+ldt*0];
					pC10[0+ps*0] += w00;
					for(kk=1; kk<kmax; kk++)
						{
						pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
						}
					pC10 += 1;
					}
				pC10 += -ps+ps*sdd;
				}
			for( ; jj<jmax-3; jj+=4)
				{
				//
				_w0 = _mm256_load_pd( &pC10[0+ps*0] );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm256_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
					_t0 = _mm256_mul_pd( _a0, _b0 );
					_w0 = _mm256_add_pd( _w0, _t0 );
					}
				//
				_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
				_w0 = _mm256_mul_pd( _w0, _b0 );
				//
				_a0 = _mm256_load_pd( &pC10[0+ps*0] );
				_a0 = _mm256_add_pd( _a0, _w0 );
				_mm256_store_pd( &pC10[0+ps*0], _a0 );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm256_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
					_t0 = _mm256_mul_pd( _w0, _b0 );
					_a0 = _mm256_add_pd( _a0, _t0 );
					_mm256_store_pd( &pC10[0+ps*kk], _a0 );
					}
				pC10 += ps*sdd;
				}
			for(ll=0; ll<jmax-jj; ll++)
				{
				w00 = pC10[0+ps*0];
				for(kk=1; kk<kmax; kk++)
					{
					w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
					}
				w00 = w00*pT[0+ldt*0];
				pC10[0+ps*0] += w00;
				for(kk=1; kk<kmax; kk++)
					{
					pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
					}
				pC10 += 1;
				}
			}
		}
	return;
	}



// unblocked algorithm
// positive diagonal; array algorithm [L, A]
void kernel_dgelqf_pd_la_vs_lib4(int m, int n1, int k, int offD, double *pD, int sdd, double *dD, int offA, double *pA, int sda)
	{
	if(m<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0, kmax, kmax0;
	const int ps = 4;
	imax = k;//m<n ? m : n;
	double alpha, beta, sigma, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m256d
		_a0, _b0, _t0, _w0, _w1;
	double *pC00, *pC10, *pC10a, *pC20, *pC20a, *pC01, *pC11; // TODO remove
	double *pD00, *pD10, *pD20, *pD20a, *pD01, *pD11;
	double *pA00, *pA10, *pA20, *pA20a, *pA01, *pA11;
	double pT[4];
	int ldt = 2;
	double *pD0 = pD-offD;
	double *pA0 = pA-offA;
	ii = 0;
#if 0 // TODO rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pC00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		sigma = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = pC00[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pC00[0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pC00[0] = beta;
			for(jj=1; jj<n-ii; jj++)
				pC00[0+ps*jj] *= tmp;
			}
		pC10 = &pD0[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
		kmax = n-ii;
		w00 = pC10[0+ps*0]; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
			}
		w00 = - w00*dD[ii];
		pC10[0+ps*0] += w00; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
			}
		// second row
		pC11 = pC10+ps*1;
		sigma = 0.0;
		for(jj=1; jj<n-(ii+1); jj++)
			{
			tmp = pC11[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = pC11[0+ps*0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii+1] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pC11[0+ps*0] = beta;
			for(jj=1; jj<n-(ii+1); jj++)
				pC11[0+ps*jj] *= tmp;
			}
		// compute T
		kmax = n-ii;
		tmp = 1.0*0.0 + pC00[0+ps*1]*1.0;
		for(kk=2; kk<kmax; kk++)
			tmp += pC00[0+ps*kk]*pC10[0+ps*kk];
		pT[0+ldt*0] = - dD[ii+0];
		pT[0+ldt*1] = + dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldt*1] = - dD[ii+1];
		// downgrade
		kmax = n-ii;
		jmax = m-ii-2;
		jmax0 = (ps-((ii+2+offD)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pC20a = &pD0[((offD+ii+2)&(ps-1))+((offD+ii+2)-((offD+ii+2)&(ps-1)))*sdd+ii*ps];
		pC20 = pC20a;
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
				w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
					w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
					}
				w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
				w00 = w00*pT[0+ldt*0];
				pC20[0+ps*0] += w00*1.0          + w01*0.0;
				pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
					}
				pC20 += 1;
				}
			pC20 += -ps+ps*sdd;
			}
		for( ; jj<jmax-3; jj+=4)
			{
			//
			_w0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _a0, _b0 );
			_w0 = _mm256_add_pd( _w0, _t0 );
			_w1 = _mm256_load_pd( &pC20[0+ps*1] );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w0 = _mm256_add_pd( _w0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w1 = _mm256_add_pd( _w1, _t0 );
				}
			//
			_b0 = _mm256_broadcast_sd( &pT[1+ldt*1] );
			_w1 = _mm256_mul_pd( _w1, _b0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_w1 = _mm256_add_pd( _w1, _t0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
			_w0 = _mm256_mul_pd( _w0, _b0 );
			//
			_a0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_add_pd( _a0, _w0 );
			_mm256_store_pd( &pC20[0+ps*0], _a0 );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_a0 = _mm256_add_pd( _a0, _t0 );
			_a0 = _mm256_add_pd( _a0, _w1 );
			_mm256_store_pd( &pC20[0+ps*1], _a0 );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w0, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w1, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_mm256_store_pd( &pC20[0+ps*kk], _a0 );
				}
			pC20 += ps*sdd;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
			w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
				w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
				}
			w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
			w00 = w00*pT[0+ldt*0];
			pC20[0+ps*0] += w00*1.0          + w01*0.0;
			pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
				}
			pC20 += 1;
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		pD00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		pA00 = &pA0[((offA+ii)&(ps-1))+((offA+ii)-((offA+ii)&(ps-1)))*sda+0*ps];
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = pA00[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pD00[0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pD00[0] = beta;
			for(jj=0; jj<n1; jj++)
				pA00[0+ps*jj] *= tmp;
			}
		// compute T
		pT[0+ldt*0] = - dD[ii+0];
		// downgrade
		jmax = m-ii-1;
		jmax0 = (ps-((ii+1+offA)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pA10 = &pA0[((offA+ii+1)&(ps-1))+((offA+ii+1)-((offA+ii+1)&(ps-1)))*sda+0*ps];
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				pD10 = &pD0[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
				w00 = pD10[0+ps*0];
				for(kk=0; kk<n1; kk++)
					{
					w00 += pA10[0+ps*kk] * pA00[0+ps*kk];
					}
				w00 = w00*pT[0+ldt*0];
				pD10[0+ps*0] += w00;
				for(kk=0; kk<n1; kk++)
					{
					pA10[0+ps*kk] += w00 * pA00[0+ps*kk];
					}
				pA10 += 1;
				}
			pA10 += -ps+ps*sda;
			}
		for( ; jj<jmax-3; jj+=4)
			{
			//
			pD10 = &pD0[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
			_w0 = _mm256_load_pd( &pD10[0+ps*0] );
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm256_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pA00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w0 = _mm256_add_pd( _w0, _t0 );
				}
			//
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
			_w0 = _mm256_mul_pd( _w0, _b0 );
			//
			_a0 = _mm256_load_pd( &pD10[0+ps*0] );
			_a0 = _mm256_add_pd( _a0, _w0 );
			_mm256_store_pd( &pD10[0+ps*0], _a0 );
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm256_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pA00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w0, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_mm256_store_pd( &pA10[0+ps*kk], _a0 );
				}
			pA10 += ps*sda;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			pD10 = &pD0[((offD+ii+jj+ll+1)&(ps-1))+((offD+ii+jj+ll+1)-((offD+ii+jj+ll+1)&(ps-1)))*sdd+ii*ps];
			w00 = pD10[0+ps*0];
			for(kk=0; kk<n1; kk++)
				{
				w00 += pA10[0+ps*kk] * pA00[0+ps*kk];
				}
			w00 = w00*pT[0+ldt*0];
			pD10[0+ps*0] += w00;
			for(kk=0; kk<n1; kk++)
				{
				pA10[0+ps*kk] += w00 * pA00[0+ps*kk];
				}
			pA10 += 1;
			}
		}
	return;
	}



void kernel_dlarft_4_la_lib4(int n1, double *dD, double *pA, double *pT)
	{
	const int ps = 4;
	int kk;
	double v10,
	       v20, v21,
		   v30, v31, v32;
	// orthogonal pD
	v10 = 0.0;
	v20 = 0.0;
	v30 = 0.0;
	v21 = 0.0;
	v31 = 0.0;
	v32 = 0.0;
	// A
	for(kk=0; kk<n1; kk++)
		{
		v10 += pA[1+ps*kk]*pA[0+ps*kk];
		v20 += pA[2+ps*kk]*pA[0+ps*kk];
		v30 += pA[3+ps*kk]*pA[0+ps*kk];
		v21 += pA[2+ps*kk]*pA[1+ps*kk];
		v31 += pA[3+ps*kk]*pA[1+ps*kk];
		v32 += pA[3+ps*kk]*pA[2+ps*kk];
		}
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[2+ps*2] = - dD[2];
	pT[3+ps*3] = - dD[3];
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	pT[1+ps*2] = - dD[2] * (v21*pT[1+ps*1]);
	pT[2+ps*3] = - dD[3] * (v32*pT[2+ps*2]);
	pT[0+ps*2] = - dD[2] * (v20*pT[0+ps*0] + v21*pT[0+ps*1]);
	pT[1+ps*3] = - dD[3] * (v31*pT[1+ps*1] + v32*pT[1+ps*2]);
	pT[0+ps*3] = - dD[3] * (v30*pT[0+ps*0] + v31*pT[0+ps*1] + v32*pT[0+ps*2]);
	return;
	}



void kernel_dlarfb4_rn_1_la_lib4(int n1, double *pVA, double *pT, double *pD, double *pA)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*1] = pD[0+ps*1];
	// 2
	pW[0+ps*2] = pD[0+ps*2];
	// 3
	pW[0+ps*3] = pD[0+ps*3];
	//
	for(kk=0; kk<n1; kk++)
		{
		pW[0+ps*0] += pA[0+ps*kk]*pVA[0+ps*kk];
		pW[0+ps*1] += pA[0+ps*kk]*pVA[1+ps*kk];
		pW[0+ps*2] += pA[0+ps*kk]*pVA[2+ps*kk];
		pW[0+ps*3] += pA[0+ps*kk]*pVA[3+ps*kk];
		}
	//
	pW[0+ps*3] = pW[0+ps*0]*pT[0+ps*3] + pW[0+ps*1]*pT[1+ps*3] + pW[0+ps*2]*pT[2+ps*3] + pW[0+ps*3]*pT[3+ps*3];
	//
	pW[0+ps*2] = pW[0+ps*0]*pT[0+ps*2] + pW[0+ps*1]*pT[1+ps*2] + pW[0+ps*2]*pT[2+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*2];
	//
	pD[0+ps*3] += pW[0+ps*3];
	//
	for(kk=0; kk<n1; kk++)
		{
		pA[0+ps*kk] += pW[0+ps*0]*pVA[0+ps*kk] + pW[0+ps*1]*pVA[1+ps*kk] + pW[0+ps*2]*pVA[2+ps*kk] + pW[0+ps*3]*pVA[3+ps*kk];
		}
	return;
	}



// unblocked algorithm
// positive diagonal; array algorithm [L, L, A]
// assume offL==offA
void kernel_dgelqf_pd_lla_vs_lib4(int m, int n0, int n1, int k, int offD, double *pD, int sdd, double *dD, int offL, double *pL, int sdl, int offA, double *pA, int sda)
	{
	if(m<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0, kmax, kmax0;
	const int ps = 4;
	imax = k;//m<n ? m : n;
	double alpha, beta, sigma, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m256d
		_a0, _b0, _t0, _w0, _w1;
	double *pC00, *pC10, *pC10a, *pC20, *pC20a, *pC01, *pC11; // TODO remove
	double *pD00, *pD10, *pD20, *pD20a, *pD01, *pD11;
	double *pL00, *pL10, *pL20, *pL20a, *pL01, *pL11;
	double *pA00, *pA10, *pA20, *pA20a, *pA01, *pA11;
	double pT[4];
	int ldt = 2;
	double *pD0 = pD-offD;
	double *pL0 = pL-offL;
	double *pA0 = pA-offA;
	ii = 0;
#if 0 // TODO rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pC00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		sigma = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = pC00[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pC00[0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pC00[0] = beta;
			for(jj=1; jj<n-ii; jj++)
				pC00[0+ps*jj] *= tmp;
			}
		pC10 = &pD0[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
		kmax = n-ii;
		w00 = pC10[0+ps*0]; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			w00 += pC10[0+ps*kk] * pC00[0+ps*kk];
			}
		w00 = - w00*dD[ii];
		pC10[0+ps*0] += w00; // pC00[0+ps*0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			pC10[0+ps*kk] += w00 * pC00[0+ps*kk];
			}
		// second row
		pC11 = pC10+ps*1;
		sigma = 0.0;
		for(jj=1; jj<n-(ii+1); jj++)
			{
			tmp = pC11[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = pC11[0+ps*0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii+1] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pC11[0+ps*0] = beta;
			for(jj=1; jj<n-(ii+1); jj++)
				pC11[0+ps*jj] *= tmp;
			}
		// compute T
		kmax = n-ii;
		tmp = 1.0*0.0 + pC00[0+ps*1]*1.0;
		for(kk=2; kk<kmax; kk++)
			tmp += pC00[0+ps*kk]*pC10[0+ps*kk];
		pT[0+ldt*0] = - dD[ii+0];
		pT[0+ldt*1] = + dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldt*1] = - dD[ii+1];
		// downgrade
		kmax = n-ii;
		jmax = m-ii-2;
		jmax0 = (ps-((ii+2+offD)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pC20a = &pD0[((offD+ii+2)&(ps-1))+((offD+ii+2)-((offD+ii+2)&(ps-1)))*sdd+ii*ps];
		pC20 = pC20a;
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
				w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
					w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
					}
				w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
				w00 = w00*pT[0+ldt*0];
				pC20[0+ps*0] += w00*1.0          + w01*0.0;
				pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
				for(kk=2; kk<kmax; kk++)
					{
					pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
					}
				pC20 += 1;
				}
			pC20 += -ps+ps*sdd;
			}
		for( ; jj<jmax-3; jj+=4)
			{
			//
			_w0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _a0, _b0 );
			_w0 = _mm256_add_pd( _w0, _t0 );
			_w1 = _mm256_load_pd( &pC20[0+ps*1] );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w0 = _mm256_add_pd( _w0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w1 = _mm256_add_pd( _w1, _t0 );
				}
			//
			_b0 = _mm256_broadcast_sd( &pT[1+ldt*1] );
			_w1 = _mm256_mul_pd( _w1, _b0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_w1 = _mm256_add_pd( _w1, _t0 );
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
			_w0 = _mm256_mul_pd( _w0, _b0 );
			//
			_a0 = _mm256_load_pd( &pC20[0+ps*0] );
			_a0 = _mm256_add_pd( _a0, _w0 );
			_mm256_store_pd( &pC20[0+ps*0], _a0 );
			_a0 = _mm256_load_pd( &pC20[0+ps*1] );
			_b0 = _mm256_broadcast_sd( &pC00[0+ps*1] );
			_t0 = _mm256_mul_pd( _w0, _b0 );
			_a0 = _mm256_add_pd( _a0, _t0 );
			_a0 = _mm256_add_pd( _a0, _w1 );
			_mm256_store_pd( &pC20[0+ps*1], _a0 );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm256_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pC00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w0, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_b0 = _mm256_broadcast_sd( &pC10[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w1, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_mm256_store_pd( &pC20[0+ps*kk], _a0 );
				}
			pC20 += ps*sdd;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			w00 = pC20[0+ps*0]*1.0 + pC20[0+ps*1]*pC00[0+ps*1];
			w01 = pC20[0+ps*0]*0.0 + pC20[0+ps*1]*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				w00 += pC20[0+ps*kk]*pC00[0+ps*kk];
				w01 += pC20[0+ps*kk]*pC10[0+ps*kk];
				}
			w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
			w00 = w00*pT[0+ldt*0];
			pC20[0+ps*0] += w00*1.0          + w01*0.0;
			pC20[0+ps*1] += w00*pC00[0+ps*1] + w01*1.0;
			for(kk=2; kk<kmax; kk++)
				{
				pC20[0+ps*kk] += w00*pC00[0+ps*kk] + w01*pC10[0+ps*kk];
				}
			pC20 += 1;
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		pD00 = &pD0[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		pL00 = &pL0[((offL+ii)&(ps-1))+((offL+ii)-((offL+ii)&(ps-1)))*sdl+0*ps];
		pA00 = &pA0[((offA+ii)&(ps-1))+((offA+ii)-((offA+ii)&(ps-1)))*sda+0*ps];
		sigma = 0.0;
		for(jj=0; jj<=n0+ii; jj++)
			{
			tmp = pL00[0+ps*jj];
			sigma += tmp*tmp;
			}
		for(jj=0; jj<n1; jj++)
			{
			tmp = pA00[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pD00[0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pD00[0] = beta;
			for(jj=0; jj<=n0+ii; jj++)
				{
				pL00[0+ps*jj] *= tmp;
				}
			for(jj=0; jj<n1; jj++)
				{
				pA00[0+ps*jj] *= tmp;
				}
			}
		// compute T
		pT[0+ldt*0] = - dD[ii+0];
		// downgrade
		jmax = m-ii-1;
		jmax0 = (ps-((ii+1+offA)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pL10 = &pL0[((offL+ii+1)&(ps-1))+((offL+ii+1)-((offL+ii+1)&(ps-1)))*sdl+0*ps];
		pA10 = &pA0[((offA+ii+1)&(ps-1))+((offA+ii+1)-((offA+ii+1)&(ps-1)))*sda+0*ps];
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				pD10 = &pD0[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
				w00 = pD10[0+ps*0];
				for(kk=0; kk<=n0+ii; kk++)
					{
					w00 += pL10[0+ps*kk] * pL00[0+ps*kk];
					}
				for(kk=0; kk<n1; kk++)
					{
					w00 += pA10[0+ps*kk] * pA00[0+ps*kk];
					}
				w00 = w00*pT[0+ldt*0];
				pD10[0+ps*0] += w00;
				for(kk=0; kk<=n0+ii; kk++)
					{
					pL10[0+ps*kk] += w00 * pL00[0+ps*kk];
					}
				for(kk=0; kk<n1; kk++)
					{
					pA10[0+ps*kk] += w00 * pA00[0+ps*kk];
					}
				pL10 += 1;
				pA10 += 1;
				}
			pL10 += -ps+ps*sdl;
			pA10 += -ps+ps*sda;
			}
		for( ; jj<jmax-3; jj+=4)
			{
			//
			pD10 = &pD0[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
			_w0 = _mm256_load_pd( &pD10[0+ps*0] );
			for(kk=0; kk<=n0+ii; kk++)
				{
				_a0 = _mm256_load_pd( &pL10[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pL00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w0 = _mm256_add_pd( _w0, _t0 );
				}
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm256_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pA00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _a0, _b0 );
				_w0 = _mm256_add_pd( _w0, _t0 );
				}
			//
			_b0 = _mm256_broadcast_sd( &pT[0+ldt*0] );
			_w0 = _mm256_mul_pd( _w0, _b0 );
			//
			_a0 = _mm256_load_pd( &pD10[0+ps*0] );
			_a0 = _mm256_add_pd( _a0, _w0 );
			_mm256_store_pd( &pD10[0+ps*0], _a0 );
			for(kk=0; kk<=n0+ii; kk++)
				{
				_a0 = _mm256_load_pd( &pL10[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pL00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w0, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_mm256_store_pd( &pL10[0+ps*kk], _a0 );
				}
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm256_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm256_broadcast_sd( &pA00[0+ps*kk] );
				_t0 = _mm256_mul_pd( _w0, _b0 );
				_a0 = _mm256_add_pd( _a0, _t0 );
				_mm256_store_pd( &pA10[0+ps*kk], _a0 );
				}
			pL10 += ps*sdl;
			pA10 += ps*sda;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			pD10 = &pD0[((offD+ii+jj+ll+1)&(ps-1))+((offD+ii+jj+ll+1)-((offD+ii+jj+ll+1)&(ps-1)))*sdd+ii*ps];
			w00 = pD10[0+ps*0];
			for(kk=0; kk<=n0+ii; kk++)
				{
				w00 += pL10[0+ps*kk] * pL00[0+ps*kk];
				}
			for(kk=0; kk<n1; kk++)
				{
				w00 += pA10[0+ps*kk] * pA00[0+ps*kk];
				}
			w00 = w00*pT[0+ldt*0];
			pD10[0+ps*0] += w00;
			for(kk=0; kk<=n0+ii; kk++)
				{
				pL10[0+ps*kk] += w00 * pL00[0+ps*kk];
				}
			for(kk=0; kk<n1; kk++)
				{
				pA10[0+ps*kk] += w00 * pA00[0+ps*kk];
				}
			pL10 += 1;
			pA10 += 1;
			}
		}
	return;
	}



void kernel_dlarft_4_lla_lib4(int n0, int n1, double *dD, double *pL, double *pA, double *pT)
	{
	const int ps = 4;
	int kk;
	double v10,
	       v20, v21,
		   v30, v31, v32;
	// orthogonal pD
	v10 = 0.0;
	v20 = 0.0;
	v30 = 0.0;
	v21 = 0.0;
	v31 = 0.0;
	v32 = 0.0;
	// L
	for(kk=0; kk<=n0; kk++)
		{
		v10 += pL[1+ps*kk]*pL[0+ps*kk];
		v20 += pL[2+ps*kk]*pL[0+ps*kk];
		v30 += pL[3+ps*kk]*pL[0+ps*kk];
		v21 += pL[2+ps*kk]*pL[1+ps*kk];
		v31 += pL[3+ps*kk]*pL[1+ps*kk];
		v32 += pL[3+ps*kk]*pL[2+ps*kk];
		}
	// 3
	v21 += pL[2+ps*kk]*pL[1+ps*kk];
	v31 += pL[3+ps*kk]*pL[1+ps*kk];
	v32 += pL[3+ps*kk]*pL[2+ps*kk];
	kk++;
	// 2
	v32 += pL[3+ps*kk]*pL[2+ps*kk];
	kk++;
	// 1
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		v10 += pA[1+ps*kk]*pA[0+ps*kk];
		v20 += pA[2+ps*kk]*pA[0+ps*kk];
		v30 += pA[3+ps*kk]*pA[0+ps*kk];
		v21 += pA[2+ps*kk]*pA[1+ps*kk];
		v31 += pA[3+ps*kk]*pA[1+ps*kk];
		v32 += pA[3+ps*kk]*pA[2+ps*kk];
		}
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[2+ps*2] = - dD[2];
	pT[3+ps*3] = - dD[3];
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	pT[1+ps*2] = - dD[2] * (v21*pT[1+ps*1]);
	pT[2+ps*3] = - dD[3] * (v32*pT[2+ps*2]);
	pT[0+ps*2] = - dD[2] * (v20*pT[0+ps*0] + v21*pT[0+ps*1]);
	pT[1+ps*3] = - dD[3] * (v31*pT[1+ps*1] + v32*pT[1+ps*2]);
	pT[0+ps*3] = - dD[3] * (v30*pT[0+ps*0] + v31*pT[0+ps*1] + v32*pT[0+ps*2]);
	return;
	}



#if 0
void kernel_dlarfb4_rn_4_lla_lib4(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, double *pL, double *pA)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// D
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	pW[1+ps*0] = pD[1+ps*0];
	pW[2+ps*0] = pD[2+ps*0];
	pW[3+ps*0] = pD[3+ps*0];
	// 1
	pW[0+ps*1] = pD[0+ps*1];
	pW[1+ps*1] = pD[1+ps*1];
	pW[2+ps*1] = pD[2+ps*1];
	pW[3+ps*1] = pD[3+ps*1];
	// 2
	pW[0+ps*2] = pD[0+ps*2];
	pW[1+ps*2] = pD[1+ps*2];
	pW[2+ps*2] = pD[2+ps*2];
	pW[3+ps*2] = pD[3+ps*2];
	// 3
	pW[0+ps*3] = pD[0+ps*3];
	pW[1+ps*3] = pD[1+ps*3];
	pW[2+ps*3] = pD[2+ps*3];
	pW[3+ps*3] = pD[3+ps*3];
	// L
	for(kk=0; kk<=n0; kk++)
		{
		pW[0+ps*0] += pL[0+ps*kk]*pVL[0+ps*kk];
		pW[1+ps*0] += pL[1+ps*kk]*pVL[0+ps*kk];
		pW[2+ps*0] += pL[2+ps*kk]*pVL[0+ps*kk];
		pW[3+ps*0] += pL[3+ps*kk]*pVL[0+ps*kk];
		pW[0+ps*1] += pL[0+ps*kk]*pVL[1+ps*kk];
		pW[1+ps*1] += pL[1+ps*kk]*pVL[1+ps*kk];
		pW[2+ps*1] += pL[2+ps*kk]*pVL[1+ps*kk];
		pW[3+ps*1] += pL[3+ps*kk]*pVL[1+ps*kk];
		pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
		pW[1+ps*2] += pL[1+ps*kk]*pVL[2+ps*kk];
		pW[2+ps*2] += pL[2+ps*kk]*pVL[2+ps*kk];
		pW[3+ps*2] += pL[3+ps*kk]*pVL[2+ps*kk];
		pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
		pW[1+ps*3] += pL[1+ps*kk]*pVL[3+ps*kk];
		pW[2+ps*3] += pL[2+ps*kk]*pVL[3+ps*kk];
		pW[3+ps*3] += pL[3+ps*kk]*pVL[3+ps*kk];
		}
	// 3
	pW[0+ps*1] += pL[0+ps*kk]*pVL[1+ps*kk];
	pW[1+ps*1] += pL[1+ps*kk]*pVL[1+ps*kk];
	pW[2+ps*1] += pL[2+ps*kk]*pVL[1+ps*kk];
	pW[3+ps*1] += pL[3+ps*kk]*pVL[1+ps*kk];
	pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
	pW[1+ps*2] += pL[1+ps*kk]*pVL[2+ps*kk];
	pW[2+ps*2] += pL[2+ps*kk]*pVL[2+ps*kk];
	pW[3+ps*2] += pL[3+ps*kk]*pVL[2+ps*kk];
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	pW[1+ps*3] += pL[1+ps*kk]*pVL[3+ps*kk];
	pW[2+ps*3] += pL[2+ps*kk]*pVL[3+ps*kk];
	pW[3+ps*3] += pL[3+ps*kk]*pVL[3+ps*kk];
	kk++;
	// 2
	pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
	pW[1+ps*2] += pL[1+ps*kk]*pVL[2+ps*kk];
	pW[2+ps*2] += pL[2+ps*kk]*pVL[2+ps*kk];
	pW[3+ps*2] += pL[3+ps*kk]*pVL[2+ps*kk];
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	pW[1+ps*3] += pL[1+ps*kk]*pVL[3+ps*kk];
	pW[2+ps*3] += pL[2+ps*kk]*pVL[3+ps*kk];
	pW[3+ps*3] += pL[3+ps*kk]*pVL[3+ps*kk];
	kk++;
	// 1
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	pW[1+ps*3] += pL[1+ps*kk]*pVL[3+ps*kk];
	pW[2+ps*3] += pL[2+ps*kk]*pVL[3+ps*kk];
	pW[3+ps*3] += pL[3+ps*kk]*pVL[3+ps*kk];
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		pW[0+ps*0] += pA[0+ps*kk]*pVA[0+ps*kk];
		pW[1+ps*0] += pA[1+ps*kk]*pVA[0+ps*kk];
		pW[2+ps*0] += pA[2+ps*kk]*pVA[0+ps*kk];
		pW[3+ps*0] += pA[3+ps*kk]*pVA[0+ps*kk];
		pW[0+ps*1] += pA[0+ps*kk]*pVA[1+ps*kk];
		pW[1+ps*1] += pA[1+ps*kk]*pVA[1+ps*kk];
		pW[2+ps*1] += pA[2+ps*kk]*pVA[1+ps*kk];
		pW[3+ps*1] += pA[3+ps*kk]*pVA[1+ps*kk];
		pW[0+ps*2] += pA[0+ps*kk]*pVA[2+ps*kk];
		pW[1+ps*2] += pA[1+ps*kk]*pVA[2+ps*kk];
		pW[2+ps*2] += pA[2+ps*kk]*pVA[2+ps*kk];
		pW[3+ps*2] += pA[3+ps*kk]*pVA[2+ps*kk];
		pW[0+ps*3] += pA[0+ps*kk]*pVA[3+ps*kk];
		pW[1+ps*3] += pA[1+ps*kk]*pVA[3+ps*kk];
		pW[2+ps*3] += pA[2+ps*kk]*pVA[3+ps*kk];
		pW[3+ps*3] += pA[3+ps*kk]*pVA[3+ps*kk];
		}
	//
	pW[0+ps*3] = pW[0+ps*0]*pT[0+ps*3] + pW[0+ps*1]*pT[1+ps*3] + pW[0+ps*2]*pT[2+ps*3] + pW[0+ps*3]*pT[3+ps*3];
	pW[1+ps*3] = pW[1+ps*0]*pT[0+ps*3] + pW[1+ps*1]*pT[1+ps*3] + pW[1+ps*2]*pT[2+ps*3] + pW[1+ps*3]*pT[3+ps*3];
	pW[2+ps*3] = pW[2+ps*0]*pT[0+ps*3] + pW[2+ps*1]*pT[1+ps*3] + pW[2+ps*2]*pT[2+ps*3] + pW[2+ps*3]*pT[3+ps*3];
	pW[3+ps*3] = pW[3+ps*0]*pT[0+ps*3] + pW[3+ps*1]*pT[1+ps*3] + pW[3+ps*2]*pT[2+ps*3] + pW[3+ps*3]*pT[3+ps*3];
	//
	pW[0+ps*2] = pW[0+ps*0]*pT[0+ps*2] + pW[0+ps*1]*pT[1+ps*2] + pW[0+ps*2]*pT[2+ps*2];
	pW[1+ps*2] = pW[1+ps*0]*pT[0+ps*2] + pW[1+ps*1]*pT[1+ps*2] + pW[1+ps*2]*pT[2+ps*2];
	pW[2+ps*2] = pW[2+ps*0]*pT[0+ps*2] + pW[2+ps*1]*pT[1+ps*2] + pW[2+ps*2]*pT[2+ps*2];
	pW[3+ps*2] = pW[3+ps*0]*pT[0+ps*2] + pW[3+ps*1]*pT[1+ps*2] + pW[3+ps*2]*pT[2+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	pW[1+ps*1] = pW[1+ps*0]*pT[0+ps*1] + pW[1+ps*1]*pT[1+ps*1];
	pW[2+ps*1] = pW[2+ps*0]*pT[0+ps*1] + pW[2+ps*1]*pT[1+ps*1];
	pW[3+ps*1] = pW[3+ps*0]*pT[0+ps*1] + pW[3+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	pW[1+ps*0] = pW[1+ps*0]*pT[0+ps*0];
	pW[2+ps*0] = pW[2+ps*0]*pT[0+ps*0];
	pW[3+ps*0] = pW[3+ps*0]*pT[0+ps*0];
	// D
	//
	pD[0+ps*0] += pW[0+ps*0];
	pD[1+ps*0] += pW[1+ps*0];
	pD[2+ps*0] += pW[2+ps*0];
	pD[3+ps*0] += pW[3+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*1];
	pD[1+ps*1] += pW[1+ps*1];
	pD[2+ps*1] += pW[2+ps*1];
	pD[3+ps*1] += pW[3+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*2];
	pD[1+ps*2] += pW[1+ps*2];
	pD[2+ps*2] += pW[2+ps*2];
	pD[3+ps*2] += pW[3+ps*2];
	//
	pD[0+ps*3] += pW[0+ps*3];
	pD[1+ps*3] += pW[1+ps*3];
	pD[2+ps*3] += pW[2+ps*3];
	pD[3+ps*3] += pW[3+ps*3];
	// L
	for(kk=0; kk<=n0; kk++)
		{
		pL[0+ps*kk] += pW[0+ps*0]*pVL[0+ps*kk] + pW[0+ps*1]*pVL[1+ps*kk] + pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk];
		pL[1+ps*kk] += pW[1+ps*0]*pVL[0+ps*kk] + pW[1+ps*1]*pVL[1+ps*kk] + pW[1+ps*2]*pVL[2+ps*kk] + pW[1+ps*3]*pVL[3+ps*kk];
		pL[2+ps*kk] += pW[2+ps*0]*pVL[0+ps*kk] + pW[2+ps*1]*pVL[1+ps*kk] + pW[2+ps*2]*pVL[2+ps*kk] + pW[2+ps*3]*pVL[3+ps*kk];
		pL[3+ps*kk] += pW[3+ps*0]*pVL[0+ps*kk] + pW[3+ps*1]*pVL[1+ps*kk] + pW[3+ps*2]*pVL[2+ps*kk] + pW[3+ps*3]*pVL[3+ps*kk];
		}
	// 3
	pL[0+ps*kk] += pW[0+ps*1]*pVL[1+ps*kk] + pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk];
	pL[1+ps*kk] += pW[1+ps*1]*pVL[1+ps*kk] + pW[1+ps*2]*pVL[2+ps*kk] + pW[1+ps*3]*pVL[3+ps*kk];
	pL[2+ps*kk] += pW[2+ps*1]*pVL[1+ps*kk] + pW[2+ps*2]*pVL[2+ps*kk] + pW[2+ps*3]*pVL[3+ps*kk];
	pL[3+ps*kk] += pW[3+ps*1]*pVL[1+ps*kk] + pW[3+ps*2]*pVL[2+ps*kk] + pW[3+ps*3]*pVL[3+ps*kk];
	kk++;
	// 2
	pL[0+ps*kk] += pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk];
	pL[1+ps*kk] += pW[1+ps*2]*pVL[2+ps*kk] + pW[1+ps*3]*pVL[3+ps*kk];
	pL[2+ps*kk] += pW[2+ps*2]*pVL[2+ps*kk] + pW[2+ps*3]*pVL[3+ps*kk];
	pL[3+ps*kk] += pW[3+ps*2]*pVL[2+ps*kk] + pW[3+ps*3]*pVL[3+ps*kk];
	kk++;
	// 1
	pL[0+ps*kk] += pW[0+ps*3]*pVL[3+ps*kk];
	pL[1+ps*kk] += pW[1+ps*3]*pVL[3+ps*kk];
	pL[2+ps*kk] += pW[2+ps*3]*pVL[3+ps*kk];
	pL[3+ps*kk] += pW[3+ps*3]*pVL[3+ps*kk];
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		pA[0+ps*kk] += pW[0+ps*0]*pVA[0+ps*kk] + pW[0+ps*1]*pVA[1+ps*kk] + pW[0+ps*2]*pVA[2+ps*kk] + pW[0+ps*3]*pVA[3+ps*kk];
		pA[1+ps*kk] += pW[1+ps*0]*pVA[0+ps*kk] + pW[1+ps*1]*pVA[1+ps*kk] + pW[1+ps*2]*pVA[2+ps*kk] + pW[1+ps*3]*pVA[3+ps*kk];
		pA[2+ps*kk] += pW[2+ps*0]*pVA[0+ps*kk] + pW[2+ps*1]*pVA[1+ps*kk] + pW[2+ps*2]*pVA[2+ps*kk] + pW[2+ps*3]*pVA[3+ps*kk];
		pA[3+ps*kk] += pW[3+ps*0]*pVA[0+ps*kk] + pW[3+ps*1]*pVA[1+ps*kk] + pW[3+ps*2]*pVA[2+ps*kk] + pW[3+ps*3]*pVA[3+ps*kk];
		}
	return;
	}
#endif



void kernel_dlarfb4_rn_1_lla_lib4(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, double *pL, double *pA)
	{
	const int ps = 4;
	double pW[16];
	int kk;
	// D
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*1] = pD[0+ps*1];
	// 2
	pW[0+ps*2] = pD[0+ps*2];
	// 3
	pW[0+ps*3] = pD[0+ps*3];
	// L
	for(kk=0; kk<=n0; kk++)
		{
		pW[0+ps*0] += pL[0+ps*kk]*pVL[0+ps*kk];
		pW[0+ps*1] += pL[0+ps*kk]*pVL[1+ps*kk];
		pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
		pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
		}
	// 3
	pW[0+ps*1] += pL[0+ps*kk]*pVL[1+ps*kk];
	pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	kk++;
	// 2
	pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	kk++;
	// 1
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		pW[0+ps*0] += pA[0+ps*kk]*pVA[0+ps*kk];
		pW[0+ps*1] += pA[0+ps*kk]*pVA[1+ps*kk];
		pW[0+ps*2] += pA[0+ps*kk]*pVA[2+ps*kk];
		pW[0+ps*3] += pA[0+ps*kk]*pVA[3+ps*kk];
		}
	//
	pW[0+ps*3] = pW[0+ps*0]*pT[0+ps*3] + pW[0+ps*1]*pT[1+ps*3] + pW[0+ps*2]*pT[2+ps*3] + pW[0+ps*3]*pT[3+ps*3];
	//
	pW[0+ps*2] = pW[0+ps*0]*pT[0+ps*2] + pW[0+ps*1]*pT[1+ps*2] + pW[0+ps*2]*pT[2+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	// D
	//
	pD[0+ps*0] += pW[0+ps*0];
	//
	pD[0+ps*1] += pW[0+ps*1];
	//
	pD[0+ps*2] += pW[0+ps*2];
	//
	pD[0+ps*3] += pW[0+ps*3];
	// L
	for(kk=0; kk<=n0; kk++)
		{
		pL[0+ps*kk] += pW[0+ps*0]*pVL[0+ps*kk] + pW[0+ps*1]*pVL[1+ps*kk] + pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk];
		}
	// 3
	pL[0+ps*kk] += pW[0+ps*1]*pVL[1+ps*kk] + pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk];
	kk++;
	// 2
	pL[0+ps*kk] += pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk];
	kk++;
	// 1
	pL[0+ps*kk] += pW[0+ps*3]*pVL[3+ps*kk];
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		pA[0+ps*kk] += pW[0+ps*0]*pVA[0+ps*kk] + pW[0+ps*1]*pVA[1+ps*kk] + pW[0+ps*2]*pVA[2+ps*kk] + pW[0+ps*3]*pVA[3+ps*kk];
		}
	return;
	}




