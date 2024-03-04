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



// unblocked algorithm
void kernel_dgelqf_vs_lib8(int m, int n, int k, int offD, double *pD, int sdd, double *dD)
	{
	if(m<=0 | n<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0, kmax, kmax0;
	const int ps = 8;
	imax = k;//m<n ? m : n;
	double alpha, beta, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m512d
		_a0, _b0, _t0, _w0, _w1;
	double *pC00, *pC10, *pC10a, *pC20, *pC20a, *pC01, *pC11;
	double pT[4];
	int ldt = 2;
	ii = 0;
#if 1 // rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pC00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
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
		pC10 = &pD[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
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
		pC20a = &pD[((offD+ii+2)&(ps-1))+((offD+ii+2)-((offD+ii+2)&(ps-1)))*sdd+ii*ps];
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
		for( ; jj<jmax-7; jj+=8)
			{
			//
			_w0 = _mm512_load_pd( &pC20[0+ps*0] );
			_a0 = _mm512_load_pd( &pC20[0+ps*1] );
			_b0 = _mm512_set1_pd( pC00[0+ps*1] );
			_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
			_w1 = _mm512_load_pd( &pC20[0+ps*1] );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm512_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				_b0 = _mm512_set1_pd( pC10[0+ps*kk] );
				_w1 = _mm512_fmadd_pd( _a0, _b0, _w1 );
				}
			//
			_b0 = _mm512_set1_pd( pT[1+ldt*1] );
			_w1 = _mm512_mul_pd( _w1, _b0 );
			_b0 = _mm512_set1_pd( pT[0+ldt*1] );
			_w1 = _mm512_fmadd_pd( _w0, _b0, _w1 );
			_b0 = _mm512_set1_pd( pT[0+ldt*0] );
			_w0 = _mm512_mul_pd( _w0, _b0 );
			//
			_a0 = _mm512_load_pd( &pC20[0+ps*0] );
			_a0 = _mm512_add_pd( _a0, _w0 );
			_mm512_store_pd( &pC20[0+ps*0], _a0 );
			_a0 = _mm512_load_pd( &pC20[0+ps*1] );
			_b0 = _mm512_set1_pd( pC00[0+ps*1] );
			_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
			_a0 = _mm512_add_pd( _a0, _w1 );
			_mm512_store_pd( &pC20[0+ps*1], _a0 );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm512_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_b0 = _mm512_set1_pd( pC10[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w1, _b0, _a0 );
				_mm512_store_pd( &pC20[0+ps*kk], _a0 );
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
		pC00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
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
			pC10a = &pD[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
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
			for( ; jj<jmax-7; jj+=8)
				{
				//
				_w0 = _mm512_load_pd( &pC10[0+ps*0] );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm512_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
					_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
					}
				//
				_b0 = _mm512_set1_pd( pT[0+ldt*0] );
				_w0 = _mm512_mul_pd( _w0, _b0 );
				//
				_a0 = _mm512_load_pd( &pC10[0+ps*0] );
				_a0 = _mm512_add_pd( _a0, _w0 );
				_mm512_store_pd( &pC10[0+ps*0], _a0 );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm512_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
					_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
					_mm512_store_pd( &pC10[0+ps*kk], _a0 );
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
// positive diagonal
void kernel_dgelqf_pd_vs_lib8(int m, int n, int k, int offD, double *pD, int sdd, double *dD)
	{
	if(m<=0 | n<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0, kmax, kmax0;
	const int ps = 8;
	imax = k;//m<n ? m : n;
	double alpha, beta, sigma, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m512d
		_a0, _b0, _t0, _w0, _w1;
	double *pC00, *pC10, *pC10a, *pC20, *pC20a, *pC01, *pC11;
	double pT[4];
	int ldt = 2;
	ii = 0;
#if 1 // rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pC00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
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
		pC10 = &pD[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
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
		pC20a = &pD[((offD+ii+2)&(ps-1))+((offD+ii+2)-((offD+ii+2)&(ps-1)))*sdd+ii*ps];
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
		for( ; jj<jmax-7; jj+=8)
			{
			//
			_w0 = _mm512_load_pd( &pC20[0+ps*0] );
			_a0 = _mm512_load_pd( &pC20[0+ps*1] );
			_b0 = _mm512_set1_pd( pC00[0+ps*1] );
			_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
			_w1 = _mm512_load_pd( &pC20[0+ps*1] );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm512_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				_b0 = _mm512_set1_pd( pC10[0+ps*kk] );
				_w1 = _mm512_fmadd_pd( _a0, _b0, _w1 );
				}
			//
			_b0 = _mm512_set1_pd( pT[1+ldt*1] );
			_w1 = _mm512_mul_pd( _w1, _b0 );
			_b0 = _mm512_set1_pd( pT[0+ldt*1] );
			_w1 = _mm512_fmadd_pd( _w0, _b0, _w1 );
			_b0 = _mm512_set1_pd( pT[0+ldt*0] );
			_w0 = _mm512_mul_pd( _w0, _b0 );
			//
			_a0 = _mm512_load_pd( &pC20[0+ps*0] );
			_a0 = _mm512_add_pd( _a0, _w0 );
			_mm512_store_pd( &pC20[0+ps*0], _a0 );
			_a0 = _mm512_load_pd( &pC20[0+ps*1] );
			_b0 = _mm512_set1_pd( pC00[0+ps*1] );
			_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
			_a0 = _mm512_add_pd( _a0, _w1 );
			_mm512_store_pd( &pC20[0+ps*1], _a0 );
			for(kk=2; kk<kmax; kk++)
				{
				_a0 = _mm512_load_pd( &pC20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_b0 = _mm512_set1_pd( pC10[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w1, _b0, _a0 );
				_mm512_store_pd( &pC20[0+ps*kk], _a0 );
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
		pC00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
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
			pC10a = &pD[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
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
			for( ; jj<jmax-7; jj+=8)
				{
				//
				_w0 = _mm512_load_pd( &pC10[0+ps*0] );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm512_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
					_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
					}
				//
				_b0 = _mm512_set1_pd( pT[0+ldt*0] );
				_w0 = _mm512_mul_pd( _w0, _b0 );
				//
				_a0 = _mm512_load_pd( &pC10[0+ps*0] );
				_a0 = _mm512_add_pd( _a0, _w0 );
				_mm512_store_pd( &pC10[0+ps*0], _a0 );
				for(kk=1; kk<kmax; kk++)
					{
					_a0 = _mm512_load_pd( &pC10[0+ps*kk] );
					_b0 = _mm512_set1_pd( pC00[0+ps*kk] );
					_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
					_mm512_store_pd( &pC10[0+ps*kk], _a0 );
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



// assume kmax>=8
// 1 d d d d
// 0 1 d d d
// 0 0 1 d d
// 0 0 0 1 d
void kernel_dlarft_8_lib8(int kmax, double *pD, double *dD, double *pT)
	{
	const int ps = 8;
	int kk;
	double v10,
	       v20, v21,
		   v30, v31, v32,
		   v40, v41, v42, v43,
		   v50, v51, v52, v53, v54,
		   v60, v61, v62, v63, v64, v65,
		   v70, v71, v72, v73, v74, v75, v76;
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
	// 4
	v10 += pD[1+ps*4]*pD[0+ps*4];
	v20 += pD[2+ps*4]*pD[0+ps*4];
	v21 += pD[2+ps*4]*pD[1+ps*4];
	v30 += pD[3+ps*4]*pD[0+ps*4];
	v31 += pD[3+ps*4]*pD[1+ps*4];
	v32 += pD[3+ps*4]*pD[2+ps*4];
	v40 =  pD[0+ps*4];
	v41 =  pD[1+ps*4];
	v42 =  pD[2+ps*4];
	v43 =  pD[3+ps*4];
	// 5
	v10 += pD[1+ps*5]*pD[0+ps*5];
	v20 += pD[2+ps*5]*pD[0+ps*5];
	v21 += pD[2+ps*5]*pD[1+ps*5];
	v30 += pD[3+ps*5]*pD[0+ps*5];
	v31 += pD[3+ps*5]*pD[1+ps*5];
	v32 += pD[3+ps*5]*pD[2+ps*5];
	v40 += pD[4+ps*5]*pD[0+ps*5];
	v41 += pD[4+ps*5]*pD[1+ps*5];
	v42 += pD[4+ps*5]*pD[2+ps*5];
	v43 += pD[4+ps*5]*pD[3+ps*5];
	v50 =  pD[0+ps*5];
	v51 =  pD[1+ps*5];
	v52 =  pD[2+ps*5];
	v53 =  pD[3+ps*5];
	v54 =  pD[4+ps*5];
	// 6
	v10 += pD[1+ps*6]*pD[0+ps*6];
	v20 += pD[2+ps*6]*pD[0+ps*6];
	v21 += pD[2+ps*6]*pD[1+ps*6];
	v30 += pD[3+ps*6]*pD[0+ps*6];
	v31 += pD[3+ps*6]*pD[1+ps*6];
	v32 += pD[3+ps*6]*pD[2+ps*6];
	v40 += pD[4+ps*6]*pD[0+ps*6];
	v41 += pD[4+ps*6]*pD[1+ps*6];
	v42 += pD[4+ps*6]*pD[2+ps*6];
	v43 += pD[4+ps*6]*pD[3+ps*6];
	v50 += pD[5+ps*6]*pD[0+ps*6];
	v51 += pD[5+ps*6]*pD[1+ps*6];
	v52 += pD[5+ps*6]*pD[2+ps*6];
	v53 += pD[5+ps*6]*pD[3+ps*6];
	v54 += pD[5+ps*6]*pD[4+ps*6];
	v60 =  pD[0+ps*6];
	v61 =  pD[1+ps*6];
	v62 =  pD[2+ps*6];
	v63 =  pD[3+ps*6];
	v64 =  pD[4+ps*6];
	v65 =  pD[5+ps*6];
	// 7
	v10 += pD[1+ps*7]*pD[0+ps*7];
	v20 += pD[2+ps*7]*pD[0+ps*7];
	v21 += pD[2+ps*7]*pD[1+ps*7];
	v30 += pD[3+ps*7]*pD[0+ps*7];
	v31 += pD[3+ps*7]*pD[1+ps*7];
	v32 += pD[3+ps*7]*pD[2+ps*7];
	v40 += pD[4+ps*7]*pD[0+ps*7];
	v41 += pD[4+ps*7]*pD[1+ps*7];
	v42 += pD[4+ps*7]*pD[2+ps*7];
	v43 += pD[4+ps*7]*pD[3+ps*7];
	v50 += pD[5+ps*7]*pD[0+ps*7];
	v51 += pD[5+ps*7]*pD[1+ps*7];
	v52 += pD[5+ps*7]*pD[2+ps*7];
	v53 += pD[5+ps*7]*pD[3+ps*7];
	v54 += pD[5+ps*7]*pD[4+ps*7];
	v60 += pD[6+ps*7]*pD[0+ps*7];
	v61 += pD[6+ps*7]*pD[1+ps*7];
	v62 += pD[6+ps*7]*pD[2+ps*7];
	v63 += pD[6+ps*7]*pD[3+ps*7];
	v64 += pD[6+ps*7]*pD[4+ps*7];
	v65 += pD[6+ps*7]*pD[5+ps*7];
	v70 =  pD[0+ps*7];
	v71 =  pD[1+ps*7];
	v72 =  pD[2+ps*7];
	v73 =  pD[3+ps*7];
	v74 =  pD[4+ps*7];
	v75 =  pD[5+ps*7];
	v76 =  pD[6+ps*7];
	//
	for(kk=8; kk<kmax; kk++)
		{
		v10 += pD[1+ps*kk]*pD[0+ps*kk];
		v20 += pD[2+ps*kk]*pD[0+ps*kk];
		v30 += pD[3+ps*kk]*pD[0+ps*kk];
		v40 += pD[4+ps*kk]*pD[0+ps*kk];
		v50 += pD[5+ps*kk]*pD[0+ps*kk];
		v60 += pD[6+ps*kk]*pD[0+ps*kk];
		v70 += pD[7+ps*kk]*pD[0+ps*kk];
		//
		v21 += pD[2+ps*kk]*pD[1+ps*kk];
		v31 += pD[3+ps*kk]*pD[1+ps*kk];
		v41 += pD[4+ps*kk]*pD[1+ps*kk];
		v51 += pD[5+ps*kk]*pD[1+ps*kk];
		v61 += pD[6+ps*kk]*pD[1+ps*kk];
		v71 += pD[7+ps*kk]*pD[1+ps*kk];
		//
		v32 += pD[3+ps*kk]*pD[2+ps*kk];
		v42 += pD[4+ps*kk]*pD[2+ps*kk];
		v52 += pD[5+ps*kk]*pD[2+ps*kk];
		v62 += pD[6+ps*kk]*pD[2+ps*kk];
		v72 += pD[7+ps*kk]*pD[2+ps*kk];
		//
		v43 += pD[4+ps*kk]*pD[3+ps*kk];
		v53 += pD[5+ps*kk]*pD[3+ps*kk];
		v63 += pD[6+ps*kk]*pD[3+ps*kk];
		v73 += pD[7+ps*kk]*pD[3+ps*kk];
		//
		v54 += pD[5+ps*kk]*pD[4+ps*kk];
		v64 += pD[6+ps*kk]*pD[4+ps*kk];
		v74 += pD[7+ps*kk]*pD[4+ps*kk];
		//
		v65 += pD[6+ps*kk]*pD[5+ps*kk];
		v75 += pD[7+ps*kk]*pD[5+ps*kk];
		//
		v76 += pD[7+ps*kk]*pD[6+ps*kk];
		}
	//
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[2+ps*2] = - dD[2];
	pT[3+ps*3] = - dD[3];
	pT[4+ps*4] = - dD[4];
	pT[5+ps*5] = - dD[5];
	pT[6+ps*6] = - dD[6];
	pT[7+ps*7] = - dD[7];
	//
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	pT[1+ps*2] = - dD[2] * (v21*pT[1+ps*1]);
	pT[2+ps*3] = - dD[3] * (v32*pT[2+ps*2]);
	pT[3+ps*4] = - dD[4] * (v43*pT[3+ps*3]);
	pT[4+ps*5] = - dD[5] * (v54*pT[4+ps*4]);
	pT[5+ps*6] = - dD[6] * (v65*pT[5+ps*5]);
	pT[6+ps*7] = - dD[7] * (v76*pT[6+ps*6]);
	//
	pT[0+ps*2] = - dD[2] * (v20*pT[0+ps*0] + v21*pT[0+ps*1]);
	pT[1+ps*3] = - dD[3] * (v31*pT[1+ps*1] + v32*pT[1+ps*2]);
	pT[2+ps*4] = - dD[4] * (v42*pT[2+ps*2] + v43*pT[2+ps*3]);
	pT[3+ps*5] = - dD[5] * (v53*pT[3+ps*3] + v54*pT[3+ps*4]);
	pT[4+ps*6] = - dD[6] * (v64*pT[4+ps*4] + v65*pT[4+ps*5]);
	pT[5+ps*7] = - dD[7] * (v75*pT[5+ps*5] + v76*pT[5+ps*6]);
	//
	pT[0+ps*3] = - dD[3] * (v30*pT[0+ps*0] + v31*pT[0+ps*1] + v32*pT[0+ps*2]);
	pT[1+ps*4] = - dD[4] * (v41*pT[1+ps*1] + v42*pT[1+ps*2] + v43*pT[1+ps*3]);
	pT[2+ps*5] = - dD[5] * (v52*pT[2+ps*2] + v53*pT[2+ps*3] + v54*pT[2+ps*4]);
	pT[3+ps*6] = - dD[6] * (v63*pT[3+ps*3] + v64*pT[3+ps*4] + v65*pT[3+ps*5]);
	pT[4+ps*7] = - dD[7] * (v74*pT[4+ps*4] + v75*pT[4+ps*5] + v76*pT[4+ps*6]);
	//
	pT[0+ps*4] = - dD[4] * (v40*pT[0+ps*0] + v41*pT[0+ps*1] + v42*pT[0+ps*2] + v43*pT[0+ps*3]);
	pT[1+ps*5] = - dD[5] * (v51*pT[1+ps*1] + v52*pT[1+ps*2] + v53*pT[1+ps*3] + v54*pT[1+ps*4]);
	pT[2+ps*6] = - dD[6] * (v62*pT[2+ps*2] + v63*pT[2+ps*3] + v64*pT[2+ps*4] + v65*pT[2+ps*5]);
	pT[3+ps*7] = - dD[7] * (v73*pT[3+ps*3] + v74*pT[3+ps*4] + v75*pT[3+ps*5] + v76*pT[3+ps*6]);
	//
	pT[0+ps*5] = - dD[5] * (v50*pT[0+ps*0] + v51*pT[0+ps*1] + v52*pT[0+ps*2] + v53*pT[0+ps*3] + v54*pT[0+ps*4]);
	pT[1+ps*6] = - dD[6] * (v61*pT[1+ps*1] + v62*pT[1+ps*2] + v63*pT[1+ps*3] + v64*pT[1+ps*4] + v65*pT[1+ps*5]);
	pT[2+ps*7] = - dD[7] * (v72*pT[2+ps*2] + v73*pT[2+ps*3] + v74*pT[2+ps*4] + v75*pT[2+ps*5] + v76*pT[2+ps*6]);
	//
	pT[0+ps*6] = - dD[6] * (v60*pT[0+ps*0] + v61*pT[0+ps*1] + v62*pT[0+ps*2] + v63*pT[0+ps*3] + v64*pT[0+ps*4] + v65*pT[0+ps*5]);
	pT[1+ps*7] = - dD[7] * (v71*pT[1+ps*1] + v72*pT[1+ps*2] + v73*pT[1+ps*3] + v74*pT[1+ps*4] + v75*pT[1+ps*5] + v76*pT[1+ps*6]);
	//
	pT[0+ps*7] = - dD[7] * (v70*pT[0+ps*0] + v71*pT[0+ps*1] + v72*pT[0+ps*2] + v73*pT[0+ps*3] + v74*pT[0+ps*4] + v75*pT[0+ps*5] + v76*pT[0+ps*6]);

//printf("\n%f\n", v10);
//printf("\n%f %f\n", v20, v21);
//printf("\n%f %f %f\n", v30, v31, v32);
	return;
	}



void kernel_dlarfb8_rn_1_lib8(int kmax, double *pV, double *pT, double *pD)
	{
	const int ps = 8;
	double pW[8];
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
	// 4
	pW[0+ps*0] += pD[0+ps*4]*pV[0+ps*4];
	pW[0+ps*1] += pD[0+ps*4]*pV[1+ps*4];
	pW[0+ps*2] += pD[0+ps*4]*pV[2+ps*4];
	pW[0+ps*3] += pD[0+ps*4]*pV[3+ps*4];
	pW[0+ps*4] = pD[0+ps*4];
	// 5
	pW[0+ps*0] += pD[0+ps*5]*pV[0+ps*5];
	pW[0+ps*1] += pD[0+ps*5]*pV[1+ps*5];
	pW[0+ps*2] += pD[0+ps*5]*pV[2+ps*5];
	pW[0+ps*3] += pD[0+ps*5]*pV[3+ps*5];
	pW[0+ps*4] += pD[0+ps*5]*pV[4+ps*5];
	pW[0+ps*5] = pD[0+ps*5];
	// 6
	pW[0+ps*0] += pD[0+ps*6]*pV[0+ps*6];
	pW[0+ps*1] += pD[0+ps*6]*pV[1+ps*6];
	pW[0+ps*2] += pD[0+ps*6]*pV[2+ps*6];
	pW[0+ps*3] += pD[0+ps*6]*pV[3+ps*6];
	pW[0+ps*4] += pD[0+ps*6]*pV[4+ps*6];
	pW[0+ps*5] += pD[0+ps*6]*pV[5+ps*6];
	pW[0+ps*6] = pD[0+ps*6];
	// 7
	pW[0+ps*0] += pD[0+ps*7]*pV[0+ps*7];
	pW[0+ps*1] += pD[0+ps*7]*pV[1+ps*7];
	pW[0+ps*2] += pD[0+ps*7]*pV[2+ps*7];
	pW[0+ps*3] += pD[0+ps*7]*pV[3+ps*7];
	pW[0+ps*4] += pD[0+ps*7]*pV[4+ps*7];
	pW[0+ps*5] += pD[0+ps*7]*pV[5+ps*7];
	pW[0+ps*6] += pD[0+ps*7]*pV[6+ps*7];
	pW[0+ps*7] = pD[0+ps*7];
	//
	for(kk=8; kk<kmax; kk++)
		{
		pW[0+ps*0] += pD[0+ps*kk]*pV[0+ps*kk];
		pW[0+ps*1] += pD[0+ps*kk]*pV[1+ps*kk];
		pW[0+ps*2] += pD[0+ps*kk]*pV[2+ps*kk];
		pW[0+ps*3] += pD[0+ps*kk]*pV[3+ps*kk];
		pW[0+ps*4] += pD[0+ps*kk]*pV[4+ps*kk];
		pW[0+ps*5] += pD[0+ps*kk]*pV[5+ps*kk];
		pW[0+ps*6] += pD[0+ps*kk]*pV[6+ps*kk];
		pW[0+ps*7] += pD[0+ps*kk]*pV[7+ps*kk];
		}
	//
	pW[0+ps*7] = pW[0+ps*0]*pT[0+ps*7] + pW[0+ps*1]*pT[1+ps*7] + pW[0+ps*2]*pT[2+ps*7] + pW[0+ps*3]*pT[3+ps*7] + pW[0+ps*4]*pT[4+ps*7] + pW[0+ps*5]*pT[5+ps*7] + pW[0+ps*6]*pT[6+ps*7] + pW[0+ps*7]*pT[7+ps*7];
	//
	pW[0+ps*6] = pW[0+ps*0]*pT[0+ps*6] + pW[0+ps*1]*pT[1+ps*6] + pW[0+ps*2]*pT[2+ps*6] + pW[0+ps*3]*pT[3+ps*6] + pW[0+ps*4]*pT[4+ps*6] + pW[0+ps*5]*pT[5+ps*6] + pW[0+ps*6]*pT[6+ps*6];
	//
	pW[0+ps*5] = pW[0+ps*0]*pT[0+ps*5] + pW[0+ps*1]*pT[1+ps*5] + pW[0+ps*2]*pT[2+ps*5] + pW[0+ps*3]*pT[3+ps*5] + pW[0+ps*4]*pT[4+ps*5] + pW[0+ps*5]*pT[5+ps*5];
	//
	pW[0+ps*4] = pW[0+ps*0]*pT[0+ps*4] + pW[0+ps*1]*pT[1+ps*4] + pW[0+ps*2]*pT[2+ps*4] + pW[0+ps*3]*pT[3+ps*4] + pW[0+ps*4]*pT[4+ps*4];
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
	//
	pD[0+ps*4] += pW[0+ps*0]*pV[0+ps*4] + pW[0+ps*1]*pV[1+ps*4] + pW[0+ps*2]*pV[2+ps*4] + pW[0+ps*3]*pV[3+ps*4] + pW[0+ps*4];
	//
	pD[0+ps*5] += pW[0+ps*0]*pV[0+ps*5] + pW[0+ps*1]*pV[1+ps*5] + pW[0+ps*2]*pV[2+ps*5] + pW[0+ps*3]*pV[3+ps*5] + pW[0+ps*4]*pV[4+ps*5] + pW[0+ps*5];
	//
	pD[0+ps*6] += pW[0+ps*0]*pV[0+ps*6] + pW[0+ps*1]*pV[1+ps*6] + pW[0+ps*2]*pV[2+ps*6] + pW[0+ps*3]*pV[3+ps*6] + pW[0+ps*4]*pV[4+ps*6] + pW[0+ps*5]*pV[5+ps*6] + pW[0+ps*6];
	//
	pD[0+ps*7] += pW[0+ps*0]*pV[0+ps*7] + pW[0+ps*1]*pV[1+ps*7] + pW[0+ps*2]*pV[2+ps*7] + pW[0+ps*3]*pV[3+ps*7] + pW[0+ps*4]*pV[4+ps*7] + pW[0+ps*5]*pV[5+ps*7] + pW[0+ps*6]*pV[6+ps*7] + pW[0+ps*7];
	//
	for(kk=8; kk<kmax; kk++)
		{
		pD[0+ps*kk] += pW[0+ps*0]*pV[0+ps*kk] + pW[0+ps*1]*pV[1+ps*kk] + pW[0+ps*2]*pV[2+ps*kk] + pW[0+ps*3]*pV[3+ps*kk] + pW[0+ps*4]*pV[4+ps*kk] + pW[0+ps*5]*pV[5+ps*kk] + pW[0+ps*6]*pV[6+ps*kk] + pW[0+ps*7]*pV[7+ps*kk];
		}
	return;
	}



// unblocked algorithm
// positive diagonal; array algorithm [L, A]
void kernel_dgelqf_pd_la_vs_lib8(int m, int n1, int k, int offD, double *pD, int sdd, double *dD, int offA, double *pA, int sda)
	{
	if(m<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0;
	const int ps = 8;
	imax = k;//m<n ? m : n;
	double alpha, beta, sigma, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m512d
		_a0, _b0, _t0, _w0, _w1;
	double *pD00, *pD10, *pD20, *pD01, *pD11;
	double *pA00, *pA10, *pA20, *pA01, *pA11;
	double pT[4];
	int ldt = 2;
	ii = 0;
#if 1 // rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pD00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		pA00 = &pA[((offA+ii)&(ps-1))+((offA+ii)-((offA+ii)&(ps-1)))*sda+0*ps];
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
		pD10 = &pD[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
		pA10 = &pA[((offA+ii+1)&(ps-1))+((offA+ii+1)-((offA+ii+1)&(ps-1)))*sda+0*ps];
		w00 = pD10[0+ps*0]; // pD00[0+ps*0] = 1.0
		for(kk=0; kk<n1; kk++)
			{
			w00 += pA10[0+ps*kk] * pA00[0+ps*kk];
			}
		w00 = - w00*dD[ii];
		pD10[0+ps*0] += w00; // pC00[0+ps*0] = 1.0
		for(kk=0; kk<n1; kk++)
			{
			pA10[0+ps*kk] += w00 * pA00[0+ps*kk];
			}
		// second row
		pD11 = pD10+ps*1;
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = pA10[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = pD11[0+ps*0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii+1] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pD11[0+ps*0] = beta;
			for(jj=0; jj<n1; jj++)
				pA10[0+ps*jj] *= tmp;
			}
		// compute T
		tmp = 1.0*0.0 + pD00[0+ps*1]*1.0;
		for(kk=0; kk<n1; kk++)
			tmp += pA00[0+ps*kk]*pA10[0+ps*kk];
		pT[0+ldt*0] = - dD[ii+0];
		pT[0+ldt*1] = + dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldt*1] = - dD[ii+1];
		// downgrade
		jmax = m-ii-2;
		jmax0 = (ps-((ii+2+offA)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pA20 = &pA[((offA+ii+2)&(ps-1))+((offA+ii+2)-((offA+ii+2)&(ps-1)))*sda+0*ps];
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				pD20 = &pD[((offD+ii+jj+2)&(ps-1))+((offD+ii+jj+2)-((offD+ii+jj+2)&(ps-1)))*sdd+ii*ps];
				w00 = pD20[0+ps*0]*1.0 + pD20[0+ps*1]*pD00[0+ps*1];
				w01 = pD20[0+ps*0]*0.0 + pD20[0+ps*1]*1.0;
				for(kk=0; kk<n1; kk++)
					{
					w00 += pA20[0+ps*kk]*pA00[0+ps*kk];
					w01 += pA20[0+ps*kk]*pA10[0+ps*kk];
					}
				w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
				w00 = w00*pT[0+ldt*0];
				pD20[0+ps*0] += w00*1.0          + w01*0.0;
				pD20[0+ps*1] += w00*pD00[0+ps*1] + w01*1.0;
				for(kk=0; kk<n1; kk++)
					{
					pA20[0+ps*kk] += w00*pA00[0+ps*kk] + w01*pA10[0+ps*kk];
					}
				pA20 += 1;
				}
			pA20 += -ps+ps*sda;
			}
		for( ; jj<jmax-7; jj+=8)
			{
			//
			pD20 = &pD[((offD+ii+jj+2)&(ps-1))+((offD+ii+jj+2)-((offD+ii+jj+2)&(ps-1)))*sdd+ii*ps];
			_w0 = _mm512_load_pd( &pD20[0+ps*0] );
			_a0 = _mm512_load_pd( &pD20[0+ps*1] );
			_b0 = _mm512_set1_pd( pD00[0+ps*1] );
			_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
			_w1 = _mm512_load_pd( &pD20[0+ps*1] );
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				_b0 = _mm512_set1_pd( pA10[0+ps*kk] );
				_w1 = _mm512_fmadd_pd( _a0, _b0, _w1 );
				}
			//
			_b0 = _mm512_set1_pd( pT[1+ldt*1] );
			_w1 = _mm512_mul_pd( _w1, _b0 );
			_b0 = _mm512_set1_pd( pT[0+ldt*1] );
			_w1 = _mm512_fmadd_pd( _w0, _b0, _w1 );
			_b0 = _mm512_set1_pd( pT[0+ldt*0] );
			_w0 = _mm512_mul_pd( _w0, _b0 );
			//
			_a0 = _mm512_load_pd( &pD20[0+ps*0] );
			_a0 = _mm512_add_pd( _a0, _w0 );
			_mm512_store_pd( &pD20[0+ps*0], _a0 );
			_a0 = _mm512_load_pd( &pD20[0+ps*1] );
			_b0 = _mm512_set1_pd( pD00[0+ps*1] );
			_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
			_a0 = _mm512_add_pd( _a0, _w1 );
			_mm512_store_pd( &pD20[0+ps*1], _a0 );
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_b0 = _mm512_set1_pd( pA10[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w1, _b0, _a0 );
				_mm512_store_pd( &pA20[0+ps*kk], _a0 );
				}
			pA20 += ps*sda;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			pD20 = &pD[((offD+ii+jj+ll+2)&(ps-1))+((offD+ii+jj+ll+2)-((offD+ii+jj+ll+2)&(ps-1)))*sdd+ii*ps];
			w00 = pD20[0+ps*0]*1.0 + pD20[0+ps*1]*pD00[0+ps*1];
			w01 = pD20[0+ps*0]*0.0 + pD20[0+ps*1]*1.0;
			for(kk=0; kk<n1; kk++)
				{
				w00 += pA20[0+ps*kk]*pA00[0+ps*kk];
				w01 += pA20[0+ps*kk]*pA10[0+ps*kk];
				}
			w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
			w00 = w00*pT[0+ldt*0];
			pD20[0+ps*0] += w00*1.0          + w01*0.0;
			pD20[0+ps*1] += w00*pD00[0+ps*1] + w01*1.0;
			for(kk=0; kk<n1; kk++)
				{
				pA20[0+ps*kk] += w00*pA00[0+ps*kk] + w01*pA10[0+ps*kk];
				}
			pA20 += 1;
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		pD00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		pA00 = &pA[((offA+ii)&(ps-1))+((offA+ii)-((offA+ii)&(ps-1)))*sda+0*ps];
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
		pA10 = &pA[((offA+ii+1)&(ps-1))+((offA+ii+1)-((offA+ii+1)&(ps-1)))*sda+0*ps];
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				pD10 = &pD[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
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
		for( ; jj<jmax-7; jj+=8)
			{
			//
			pD10 = &pD[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
			_w0 = _mm512_load_pd( &pD10[0+ps*0] );
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				}
			//
			_b0 = _mm512_set1_pd( pT[0+ldt*0] );
			_w0 = _mm512_mul_pd( _w0, _b0 );
			//
			_a0 = _mm512_load_pd( &pD10[0+ps*0] );
			_a0 = _mm512_add_pd( _a0, _w0 );
			_mm512_store_pd( &pD10[0+ps*0], _a0 );
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_mm512_store_pd( &pA10[0+ps*kk], _a0 );
				}
			pA10 += ps*sda;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			pD10 = &pD[((offD+ii+jj+ll+1)&(ps-1))+((offD+ii+jj+ll+1)-((offD+ii+jj+ll+1)&(ps-1)))*sdd+ii*ps];
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



// unblocked algorithm
// positive diagonal; array algorithm [L, L, A]
// assume offL==offA
void kernel_dgelqf_pd_lla_vs_lib8(int m, int n0, int n1, int k, int offD, double *pD, int sdd, double *dD, int offL, double *pL, int sdl, int offA, double *pA, int sda)
	{
	if(m<=0)
		return;
	int ii, jj, kk, ll, imax, jmax, jmax0;
	const int ps = 8;
	imax = k;//m<n ? m : n;
	double alpha, beta, sigma, tmp;
	double w00, w01,
		   w10, w11,
		   w20, w21,
		   w30, w31;
	__m512d
		_a0, _b0, _t0, _w0, _w1;
	double *pD00, *pD10, *pD20, *pD01, *pD11;
	double *pL00, *pL10, *pL20, *pL01, *pL11;
	double *pA00, *pA10, *pA20, *pA01, *pA11;
	double pT[4];
	int ldt = 2;
	ii = 0;
#if 1 // rank 2
	for(; ii<imax-1; ii+=2)
		{
		// first row
		pD00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		pL00 = &pL[((offL+ii)&(ps-1))+((offL+ii)-((offL+ii)&(ps-1)))*sdl+0*ps];
		pA00 = &pA[((offA+ii)&(ps-1))+((offA+ii)-((offA+ii)&(ps-1)))*sda+0*ps];
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
		pD10 = &pD[((offD+ii+1)&(ps-1))+((offD+ii+1)-((offD+ii+1)&(ps-1)))*sdd+ii*ps];
		pL10 = &pL[((offL+ii+1)&(ps-1))+((offL+ii+1)-((offL+ii+1)&(ps-1)))*sdl+0*ps];
		pA10 = &pA[((offA+ii+1)&(ps-1))+((offA+ii+1)-((offA+ii+1)&(ps-1)))*sda+0*ps];
		w00 = pD10[0+ps*0]; // pD00[0+ps*0] = 1.0
		for(kk=0; kk<=n0+ii; kk++)
			{
			w00 += pL10[0+ps*kk] * pL00[0+ps*kk];
			}
		for(kk=0; kk<n1; kk++)
			{
			w00 += pA10[0+ps*kk] * pA00[0+ps*kk];
			}
		w00 = - w00*dD[ii];
		pD10[0+ps*0] += w00; // pC00[0+ps*0] = 1.0
		for(kk=0; kk<=n0+ii; kk++)
			{
			pL10[0+ps*kk] += w00 * pL00[0+ps*kk];
			}
		for(kk=0; kk<n1; kk++)
			{
			pA10[0+ps*kk] += w00 * pA00[0+ps*kk];
			}
		// second row
		pD11 = pD10+ps*1;
		sigma = 0.0;
		for(jj=0; jj<=n0+ii+1; jj++)
			{
			tmp = pL10[0+ps*jj];
			sigma += tmp*tmp;
			}
		for(jj=0; jj<n1; jj++)
			{
			tmp = pA10[0+ps*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = pD11[0+ps*0];
			beta = sigma + alpha*alpha;
			beta = sqrt(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma / (alpha+beta);
			dD[ii+1] = 2*tmp*tmp / (sigma+tmp*tmp);
			tmp = 1.0 / tmp;
			pD11[0+ps*0] = beta;
			for(jj=0; jj<=n0+ii+1; jj++)
				{
				pL10[0+ps*jj] *= tmp;
				}
			for(jj=0; jj<n1; jj++)
				{
				pA10[0+ps*jj] *= tmp;
				}
			}
		// compute T
		tmp = 1.0*0.0 + pD00[0+ps*1]*1.0;
		for(kk=0; kk<=n0+ii; kk++)
			{
			tmp += pL00[0+ps*kk]*pL10[0+ps*kk];
			}
		for(kk=0; kk<n1; kk++)
			{
			tmp += pA00[0+ps*kk]*pA10[0+ps*kk];
			}
		pT[0+ldt*0] = - dD[ii+0];
		pT[0+ldt*1] = + dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldt*1] = - dD[ii+1];
		// downgrade
		jmax = m-ii-2;
		jmax0 = (ps-((ii+2+offA)&(ps-1)))&(ps-1);
		jmax0 = jmax<jmax0 ? jmax : jmax0;
		jj = 0;
		pA20 = &pA[((offA+ii+2)&(ps-1))+((offA+ii+2)-((offA+ii+2)&(ps-1)))*sda+0*ps];
		pL20 = &pL[((offL+ii+2)&(ps-1))+((offL+ii+2)-((offL+ii+2)&(ps-1)))*sdl+0*ps];
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				pD20 = &pD[((offD+ii+jj+2)&(ps-1))+((offD+ii+jj+2)-((offD+ii+jj+2)&(ps-1)))*sdd+ii*ps];
				w00 = pD20[0+ps*0]*1.0 + pD20[0+ps*1]*pD00[0+ps*1];
				w01 = pD20[0+ps*0]*0.0 + pD20[0+ps*1]*1.0;
				for(kk=0; kk<=n0+ii; kk++)
					{
					w00 += pL20[0+ps*kk]*pL00[0+ps*kk];
					w01 += pL20[0+ps*kk]*pL10[0+ps*kk];
					}
				w01 += pL20[0+ps*kk]*pL10[0+ps*kk]; // XXX
				for(kk=0; kk<n1; kk++)
					{
					w00 += pA20[0+ps*kk]*pA00[0+ps*kk];
					w01 += pA20[0+ps*kk]*pA10[0+ps*kk];
					}
				w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
				w00 = w00*pT[0+ldt*0];
				pD20[0+ps*0] += w00*1.0          + w01*0.0;
				pD20[0+ps*1] += w00*pD00[0+ps*1] + w01*1.0;
				for(kk=0; kk<=n0+ii; kk++)
					{
					pL20[0+ps*kk] += w00*pL00[0+ps*kk] + w01*pL10[0+ps*kk];
					}
				pL20[0+ps*kk] += w01*pL10[0+ps*kk]; // XXX
				for(kk=0; kk<n1; kk++)
					{
					pA20[0+ps*kk] += w00*pA00[0+ps*kk] + w01*pA10[0+ps*kk];
					}
				pA20 += 1;
				pL20 += 1;
				}
			pA20 += -ps+ps*sda;
			pL20 += -ps+ps*sdl;
			}
		for( ; jj<jmax-7; jj+=8)
			{
			//
			pD20 = &pD[((offD+ii+jj+2)&(ps-1))+((offD+ii+jj+2)-((offD+ii+jj+2)&(ps-1)))*sdd+ii*ps];
			_w0 = _mm512_load_pd( &pD20[0+ps*0] );
			_a0 = _mm512_load_pd( &pD20[0+ps*1] );
			_b0 = _mm512_set1_pd( pD00[0+ps*1] );
			_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
			_w1 = _mm512_load_pd( &pD20[0+ps*1] );
			for(kk=0; kk<=n0+ii; kk++)
				{
				_a0 = _mm512_load_pd( &pL20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pL00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				_b0 = _mm512_set1_pd( pL10[0+ps*kk] );
				_w1 = _mm512_fmadd_pd( _a0, _b0, _w1 );
				}
			_a0 = _mm512_load_pd( &pL20[0+ps*kk] ); // XXX
			_b0 = _mm512_set1_pd( pL10[0+ps*kk] ); // XXX
			_w1 = _mm512_fmadd_pd( _a0, _b0, _w1 ); // XXX
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				_b0 = _mm512_set1_pd( pA10[0+ps*kk] );
				_w1 = _mm512_fmadd_pd( _a0, _b0, _w1 );
				}
			//
			_b0 = _mm512_set1_pd( pT[1+ldt*1] );
			_w1 = _mm512_mul_pd( _w1, _b0 );
			_b0 = _mm512_set1_pd( pT[0+ldt*1] );
			_w1 = _mm512_fmadd_pd( _w0, _b0, _w1 );
			_b0 = _mm512_set1_pd( pT[0+ldt*0] );
			_w0 = _mm512_mul_pd( _w0, _b0 );
			//
			_a0 = _mm512_load_pd( &pD20[0+ps*0] );
			_a0 = _mm512_add_pd( _a0, _w0 );
			_mm512_store_pd( &pD20[0+ps*0], _a0 );
			_a0 = _mm512_load_pd( &pD20[0+ps*1] );
			_b0 = _mm512_set1_pd( pD00[0+ps*1] );
			_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
			_a0 = _mm512_add_pd( _a0, _w1 );
			_mm512_store_pd( &pD20[0+ps*1], _a0 );
			for(kk=0; kk<=n0+ii; kk++)
				{
				_a0 = _mm512_load_pd( &pL20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pL00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_b0 = _mm512_set1_pd( pL10[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w1, _b0, _a0 );
				_mm512_store_pd( &pL20[0+ps*kk], _a0 );
				}
			_a0 = _mm512_load_pd( &pL20[0+ps*kk] ); // XXX
			_b0 = _mm512_set1_pd( pL10[0+ps*kk] ); // XXX
			_a0 = _mm512_fmadd_pd( _w1, _b0, _a0 ); // XXX
			_mm512_store_pd( &pL20[0+ps*kk], _a0 ); // XXX
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA20[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_b0 = _mm512_set1_pd( pA10[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w1, _b0, _a0 );
				_mm512_store_pd( &pA20[0+ps*kk], _a0 );
				}
			pA20 += ps*sda;
			pL20 += ps*sdl;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			pD20 = &pD[((offD+ii+jj+ll+2)&(ps-1))+((offD+ii+jj+ll+2)-((offD+ii+jj+ll+2)&(ps-1)))*sdd+ii*ps];
			w00 = pD20[0+ps*0]*1.0 + pD20[0+ps*1]*pD00[0+ps*1];
			w01 = pD20[0+ps*0]*0.0 + pD20[0+ps*1]*1.0;
			for(kk=0; kk<=n0+ii; kk++)
				{
				w00 += pL20[0+ps*kk]*pL00[0+ps*kk];
				w01 += pL20[0+ps*kk]*pL10[0+ps*kk];
				}
			w01 += pL20[0+ps*kk]*pL10[0+ps*kk]; // XXX
			for(kk=0; kk<n1; kk++)
				{
				w00 += pA20[0+ps*kk]*pA00[0+ps*kk];
				w01 += pA20[0+ps*kk]*pA10[0+ps*kk];
				}
			w01 = w00*pT[0+ldt*1] + w01*pT[1+ldt*1];
			w00 = w00*pT[0+ldt*0];
			pD20[0+ps*0] += w00*1.0          + w01*0.0;
			pD20[0+ps*1] += w00*pD00[0+ps*1] + w01*1.0;
			for(kk=0; kk<=n0+ii; kk++)
				{
				pL20[0+ps*kk] += w00*pL00[0+ps*kk] + w01*pL10[0+ps*kk];
				}
			pL20[0+ps*kk] += w01*pL10[0+ps*kk]; // XXX
			for(kk=0; kk<n1; kk++)
				{
				pA20[0+ps*kk] += w00*pA00[0+ps*kk] + w01*pA10[0+ps*kk];
				}
			pA20 += 1;
			pL20 += 1;
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		pD00 = &pD[((offD+ii)&(ps-1))+((offD+ii)-((offD+ii)&(ps-1)))*sdd+ii*ps];
		pL00 = &pL[((offL+ii)&(ps-1))+((offL+ii)-((offL+ii)&(ps-1)))*sdl+0*ps];
		pA00 = &pA[((offA+ii)&(ps-1))+((offA+ii)-((offA+ii)&(ps-1)))*sda+0*ps];
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
		pL10 = &pL[((offL+ii+1)&(ps-1))+((offL+ii+1)-((offL+ii+1)&(ps-1)))*sdl+0*ps];
		pA10 = &pA[((offA+ii+1)&(ps-1))+((offA+ii+1)-((offA+ii+1)&(ps-1)))*sda+0*ps];
		if(jmax0>0)
			{
			for( ; jj<jmax0; jj++)
				{
				pD10 = &pD[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
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
		for( ; jj<jmax-7; jj+=8)
			{
			//
			pD10 = &pD[((offD+ii+jj+1)&(ps-1))+((offD+ii+jj+1)-((offD+ii+jj+1)&(ps-1)))*sdd+ii*ps];
			_w0 = _mm512_load_pd( &pD10[0+ps*0] );
			for(kk=0; kk<=n0+ii; kk++)
				{
				_a0 = _mm512_load_pd( &pL10[0+ps*kk] );
				_b0 = _mm512_set1_pd( pL00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				}
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_w0 = _mm512_fmadd_pd( _a0, _b0, _w0 );
				}
			//
			_b0 = _mm512_set1_pd( pT[0+ldt*0] );
			_w0 = _mm512_mul_pd( _w0, _b0 );
			//
			_a0 = _mm512_load_pd( &pD10[0+ps*0] );
			_a0 = _mm512_add_pd( _a0, _w0 );
			_mm512_store_pd( &pD10[0+ps*0], _a0 );
			for(kk=0; kk<=n0+ii; kk++)
				{
				_a0 = _mm512_load_pd( &pL10[0+ps*kk] );
				_b0 = _mm512_set1_pd( pL00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_mm512_store_pd( &pL10[0+ps*kk], _a0 );
				}
			for(kk=0; kk<n1; kk++)
				{
				_a0 = _mm512_load_pd( &pA10[0+ps*kk] );
				_b0 = _mm512_set1_pd( pA00[0+ps*kk] );
				_a0 = _mm512_fmadd_pd( _w0, _b0, _a0 );
				_mm512_store_pd( &pA10[0+ps*kk], _a0 );
				}
			pL10 += ps*sdl;
			pA10 += ps*sda;
			}
		for(ll=0; ll<jmax-jj; ll++)
			{
			pD10 = &pD[((offD+ii+jj+ll+1)&(ps-1))+((offD+ii+jj+ll+1)-((offD+ii+jj+ll+1)&(ps-1)))*sdd+ii*ps];
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



// assume kmax>=8
// 1 d d d d
// 0 1 d d d
// 0 0 1 d d
// 0 0 0 1 d
void kernel_dlarft_la_8_lib8(int kmax, double *dD, double *pA, double *pT)
	{
	const int ps = 8;
	int kk;
	double v10,
	       v20, v21,
		   v30, v31, v32,
		   v40, v41, v42, v43,
		   v50, v51, v52, v53, v54,
		   v60, v61, v62, v63, v64, v65,
		   v70, v71, v72, v73, v74, v75, v76;
	v10 = 0.0;
	v20 = 0.0;
	v21 = 0.0;
	v30 = 0.0;
	v31 = 0.0;
	v32 = 0.0;
	v40 = 0.0;
	v41 = 0.0;
	v42 = 0.0;
	v43 = 0.0;
	v50 = 0.0;
	v51 = 0.0;
	v52 = 0.0;
	v53 = 0.0;
	v54 = 0.0;
	v60 = 0.0;
	v61 = 0.0;
	v62 = 0.0;
	v63 = 0.0;
	v64 = 0.0;
	v65 = 0.0;
	v70 = 0.0;
	v71 = 0.0;
	v72 = 0.0;
	v73 = 0.0;
	v74 = 0.0;
	v75 = 0.0;
	v76 = 0.0;
	//
	for(kk=0; kk<kmax; kk++)
		{
		v10 += pA[1+ps*kk]*pA[0+ps*kk];
		v20 += pA[2+ps*kk]*pA[0+ps*kk];
		v30 += pA[3+ps*kk]*pA[0+ps*kk];
		v40 += pA[4+ps*kk]*pA[0+ps*kk];
		v50 += pA[5+ps*kk]*pA[0+ps*kk];
		v60 += pA[6+ps*kk]*pA[0+ps*kk];
		v70 += pA[7+ps*kk]*pA[0+ps*kk];
		//
		v21 += pA[2+ps*kk]*pA[1+ps*kk];
		v31 += pA[3+ps*kk]*pA[1+ps*kk];
		v41 += pA[4+ps*kk]*pA[1+ps*kk];
		v51 += pA[5+ps*kk]*pA[1+ps*kk];
		v61 += pA[6+ps*kk]*pA[1+ps*kk];
		v71 += pA[7+ps*kk]*pA[1+ps*kk];
		//
		v32 += pA[3+ps*kk]*pA[2+ps*kk];
		v42 += pA[4+ps*kk]*pA[2+ps*kk];
		v52 += pA[5+ps*kk]*pA[2+ps*kk];
		v62 += pA[6+ps*kk]*pA[2+ps*kk];
		v72 += pA[7+ps*kk]*pA[2+ps*kk];
		//
		v43 += pA[4+ps*kk]*pA[3+ps*kk];
		v53 += pA[5+ps*kk]*pA[3+ps*kk];
		v63 += pA[6+ps*kk]*pA[3+ps*kk];
		v73 += pA[7+ps*kk]*pA[3+ps*kk];
		//
		v54 += pA[5+ps*kk]*pA[4+ps*kk];
		v64 += pA[6+ps*kk]*pA[4+ps*kk];
		v74 += pA[7+ps*kk]*pA[4+ps*kk];
		//
		v65 += pA[6+ps*kk]*pA[5+ps*kk];
		v75 += pA[7+ps*kk]*pA[5+ps*kk];
		//
		v76 += pA[7+ps*kk]*pA[6+ps*kk];
		}
	//
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[2+ps*2] = - dD[2];
	pT[3+ps*3] = - dD[3];
	pT[4+ps*4] = - dD[4];
	pT[5+ps*5] = - dD[5];
	pT[6+ps*6] = - dD[6];
	pT[7+ps*7] = - dD[7];
	//
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	pT[1+ps*2] = - dD[2] * (v21*pT[1+ps*1]);
	pT[2+ps*3] = - dD[3] * (v32*pT[2+ps*2]);
	pT[3+ps*4] = - dD[4] * (v43*pT[3+ps*3]);
	pT[4+ps*5] = - dD[5] * (v54*pT[4+ps*4]);
	pT[5+ps*6] = - dD[6] * (v65*pT[5+ps*5]);
	pT[6+ps*7] = - dD[7] * (v76*pT[6+ps*6]);
	//
	pT[0+ps*2] = - dD[2] * (v20*pT[0+ps*0] + v21*pT[0+ps*1]);
	pT[1+ps*3] = - dD[3] * (v31*pT[1+ps*1] + v32*pT[1+ps*2]);
	pT[2+ps*4] = - dD[4] * (v42*pT[2+ps*2] + v43*pT[2+ps*3]);
	pT[3+ps*5] = - dD[5] * (v53*pT[3+ps*3] + v54*pT[3+ps*4]);
	pT[4+ps*6] = - dD[6] * (v64*pT[4+ps*4] + v65*pT[4+ps*5]);
	pT[5+ps*7] = - dD[7] * (v75*pT[5+ps*5] + v76*pT[5+ps*6]);
	//
	pT[0+ps*3] = - dD[3] * (v30*pT[0+ps*0] + v31*pT[0+ps*1] + v32*pT[0+ps*2]);
	pT[1+ps*4] = - dD[4] * (v41*pT[1+ps*1] + v42*pT[1+ps*2] + v43*pT[1+ps*3]);
	pT[2+ps*5] = - dD[5] * (v52*pT[2+ps*2] + v53*pT[2+ps*3] + v54*pT[2+ps*4]);
	pT[3+ps*6] = - dD[6] * (v63*pT[3+ps*3] + v64*pT[3+ps*4] + v65*pT[3+ps*5]);
	pT[4+ps*7] = - dD[7] * (v74*pT[4+ps*4] + v75*pT[4+ps*5] + v76*pT[4+ps*6]);
	//
	pT[0+ps*4] = - dD[4] * (v40*pT[0+ps*0] + v41*pT[0+ps*1] + v42*pT[0+ps*2] + v43*pT[0+ps*3]);
	pT[1+ps*5] = - dD[5] * (v51*pT[1+ps*1] + v52*pT[1+ps*2] + v53*pT[1+ps*3] + v54*pT[1+ps*4]);
	pT[2+ps*6] = - dD[6] * (v62*pT[2+ps*2] + v63*pT[2+ps*3] + v64*pT[2+ps*4] + v65*pT[2+ps*5]);
	pT[3+ps*7] = - dD[7] * (v73*pT[3+ps*3] + v74*pT[3+ps*4] + v75*pT[3+ps*5] + v76*pT[3+ps*6]);
	//
	pT[0+ps*5] = - dD[5] * (v50*pT[0+ps*0] + v51*pT[0+ps*1] + v52*pT[0+ps*2] + v53*pT[0+ps*3] + v54*pT[0+ps*4]);
	pT[1+ps*6] = - dD[6] * (v61*pT[1+ps*1] + v62*pT[1+ps*2] + v63*pT[1+ps*3] + v64*pT[1+ps*4] + v65*pT[1+ps*5]);
	pT[2+ps*7] = - dD[7] * (v72*pT[2+ps*2] + v73*pT[2+ps*3] + v74*pT[2+ps*4] + v75*pT[2+ps*5] + v76*pT[2+ps*6]);
	//
	pT[0+ps*6] = - dD[6] * (v60*pT[0+ps*0] + v61*pT[0+ps*1] + v62*pT[0+ps*2] + v63*pT[0+ps*3] + v64*pT[0+ps*4] + v65*pT[0+ps*5]);
	pT[1+ps*7] = - dD[7] * (v71*pT[1+ps*1] + v72*pT[1+ps*2] + v73*pT[1+ps*3] + v74*pT[1+ps*4] + v75*pT[1+ps*5] + v76*pT[1+ps*6]);
	//
	pT[0+ps*7] = - dD[7] * (v70*pT[0+ps*0] + v71*pT[0+ps*1] + v72*pT[0+ps*2] + v73*pT[0+ps*3] + v74*pT[0+ps*4] + v75*pT[0+ps*5] + v76*pT[0+ps*6]);

//printf("\n%f\n", v10);
//printf("\n%f %f\n", v20, v21);
//printf("\n%f %f %f\n", v30, v31, v32);
	return;
	}



void kernel_dlarfb8_rn_la_1_lib8(int n1, double *pVA, double *pT, double *pD, double *pA)
	{
	const int ps = 8;
	double pW[8];
	int kk;
	// 0
	pW[0+ps*0] = pD[0+ps*0];
	// 1
	pW[0+ps*1] = pD[0+ps*1];
	// 2
	pW[0+ps*2] = pD[0+ps*2];
	// 3
	pW[0+ps*3] = pD[0+ps*3];
	// 4
	pW[0+ps*4] = pD[0+ps*4];
	// 5
	pW[0+ps*5] = pD[0+ps*5];
	// 6
	pW[0+ps*6] = pD[0+ps*6];
	// 7
	pW[0+ps*7] = pD[0+ps*7];
	//
	for(kk=0; kk<n1; kk++)
		{
		pW[0+ps*0] += pA[0+ps*kk]*pVA[0+ps*kk];
		pW[0+ps*1] += pA[0+ps*kk]*pVA[1+ps*kk];
		pW[0+ps*2] += pA[0+ps*kk]*pVA[2+ps*kk];
		pW[0+ps*3] += pA[0+ps*kk]*pVA[3+ps*kk];
		pW[0+ps*4] += pA[0+ps*kk]*pVA[4+ps*kk];
		pW[0+ps*5] += pA[0+ps*kk]*pVA[5+ps*kk];
		pW[0+ps*6] += pA[0+ps*kk]*pVA[6+ps*kk];
		pW[0+ps*7] += pA[0+ps*kk]*pVA[7+ps*kk];
		}
	//
	pW[0+ps*7] = pW[0+ps*0]*pT[0+ps*7] + pW[0+ps*1]*pT[1+ps*7] + pW[0+ps*2]*pT[2+ps*7] + pW[0+ps*3]*pT[3+ps*7] + pW[0+ps*4]*pT[4+ps*7] + pW[0+ps*5]*pT[5+ps*7] + pW[0+ps*6]*pT[6+ps*7] + pW[0+ps*7]*pT[7+ps*7];
	//
	pW[0+ps*6] = pW[0+ps*0]*pT[0+ps*6] + pW[0+ps*1]*pT[1+ps*6] + pW[0+ps*2]*pT[2+ps*6] + pW[0+ps*3]*pT[3+ps*6] + pW[0+ps*4]*pT[4+ps*6] + pW[0+ps*5]*pT[5+ps*6] + pW[0+ps*6]*pT[6+ps*6];
	//
	pW[0+ps*5] = pW[0+ps*0]*pT[0+ps*5] + pW[0+ps*1]*pT[1+ps*5] + pW[0+ps*2]*pT[2+ps*5] + pW[0+ps*3]*pT[3+ps*5] + pW[0+ps*4]*pT[4+ps*5] + pW[0+ps*5]*pT[5+ps*5];
	//
	pW[0+ps*4] = pW[0+ps*0]*pT[0+ps*4] + pW[0+ps*1]*pT[1+ps*4] + pW[0+ps*2]*pT[2+ps*4] + pW[0+ps*3]*pT[3+ps*4] + pW[0+ps*4]*pT[4+ps*4];
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
	pD[0+ps*4] += pW[0+ps*4];
	//
	pD[0+ps*5] += pW[0+ps*5];
	//
	pD[0+ps*6] += pW[0+ps*6];
	//
	pD[0+ps*7] += pW[0+ps*7];
	//
	for(kk=0; kk<n1; kk++)
		{
		pA[0+ps*kk] += pW[0+ps*0]*pVA[0+ps*kk] + pW[0+ps*1]*pVA[1+ps*kk] + pW[0+ps*2]*pVA[2+ps*kk] + pW[0+ps*3]*pVA[3+ps*kk] + pW[0+ps*4]*pVA[4+ps*kk] + pW[0+ps*5]*pVA[5+ps*kk] + pW[0+ps*6]*pVA[6+ps*kk] + pW[0+ps*7]*pVA[7+ps*kk];
		}
	return;
	}



// assume kmax>=8
// 1 d d d d
// 0 1 d d d
// 0 0 1 d d
// 0 0 0 1 d
void kernel_dlarft_lla_8_lib8(int n0, int n1, double *dD, double *pL, double *pA, double *pT)
	{
	const int ps = 8;
	int kk;
	double v10,
	       v20, v21,
		   v30, v31, v32,
		   v40, v41, v42, v43,
		   v50, v51, v52, v53, v54,
		   v60, v61, v62, v63, v64, v65,
		   v70, v71, v72, v73, v74, v75, v76;
	// orthogonal pD
	v10 = 0.0;
	v20 = 0.0;
	v21 = 0.0;
	v30 = 0.0;
	v31 = 0.0;
	v32 = 0.0;
	v40 = 0.0;
	v41 = 0.0;
	v42 = 0.0;
	v43 = 0.0;
	v50 = 0.0;
	v51 = 0.0;
	v52 = 0.0;
	v53 = 0.0;
	v54 = 0.0;
	v60 = 0.0;
	v61 = 0.0;
	v62 = 0.0;
	v63 = 0.0;
	v64 = 0.0;
	v65 = 0.0;
	v70 = 0.0;
	v71 = 0.0;
	v72 = 0.0;
	v73 = 0.0;
	v74 = 0.0;
	v75 = 0.0;
	v76 = 0.0;
	// L
	for(kk=0; kk<=n0; kk++)
		{
		v10 += pL[1+ps*kk]*pL[0+ps*kk];
		v20 += pL[2+ps*kk]*pL[0+ps*kk];
		v30 += pL[3+ps*kk]*pL[0+ps*kk];
		v40 += pL[4+ps*kk]*pL[0+ps*kk];
		v50 += pL[5+ps*kk]*pL[0+ps*kk];
		v60 += pL[6+ps*kk]*pL[0+ps*kk];
		v70 += pL[7+ps*kk]*pL[0+ps*kk];
		//
		v21 += pL[2+ps*kk]*pL[1+ps*kk];
		v31 += pL[3+ps*kk]*pL[1+ps*kk];
		v41 += pL[4+ps*kk]*pL[1+ps*kk];
		v51 += pL[5+ps*kk]*pL[1+ps*kk];
		v61 += pL[6+ps*kk]*pL[1+ps*kk];
		v71 += pL[7+ps*kk]*pL[1+ps*kk];
		//
		v32 += pL[3+ps*kk]*pL[2+ps*kk];
		v42 += pL[4+ps*kk]*pL[2+ps*kk];
		v52 += pL[5+ps*kk]*pL[2+ps*kk];
		v62 += pL[6+ps*kk]*pL[2+ps*kk];
		v72 += pL[7+ps*kk]*pL[2+ps*kk];
		//
		v43 += pL[4+ps*kk]*pL[3+ps*kk];
		v53 += pL[5+ps*kk]*pL[3+ps*kk];
		v63 += pL[6+ps*kk]*pL[3+ps*kk];
		v73 += pL[7+ps*kk]*pL[3+ps*kk];
		//
		v54 += pL[5+ps*kk]*pL[4+ps*kk];
		v64 += pL[6+ps*kk]*pL[4+ps*kk];
		v74 += pL[7+ps*kk]*pL[4+ps*kk];
		//
		v65 += pL[6+ps*kk]*pL[5+ps*kk];
		v75 += pL[7+ps*kk]*pL[5+ps*kk];
		//
		v76 += pL[7+ps*kk]*pL[6+ps*kk];
		}
	// L 6
	v21 += pL[2+ps*kk]*pL[1+ps*kk];
	v31 += pL[3+ps*kk]*pL[1+ps*kk];
	v41 += pL[4+ps*kk]*pL[1+ps*kk];
	v51 += pL[5+ps*kk]*pL[1+ps*kk];
	v61 += pL[6+ps*kk]*pL[1+ps*kk];
	v71 += pL[7+ps*kk]*pL[1+ps*kk];
	//
	v32 += pL[3+ps*kk]*pL[2+ps*kk];
	v42 += pL[4+ps*kk]*pL[2+ps*kk];
	v52 += pL[5+ps*kk]*pL[2+ps*kk];
	v62 += pL[6+ps*kk]*pL[2+ps*kk];
	v72 += pL[7+ps*kk]*pL[2+ps*kk];
	//
	v43 += pL[4+ps*kk]*pL[3+ps*kk];
	v53 += pL[5+ps*kk]*pL[3+ps*kk];
	v63 += pL[6+ps*kk]*pL[3+ps*kk];
	v73 += pL[7+ps*kk]*pL[3+ps*kk];
	//
	v54 += pL[5+ps*kk]*pL[4+ps*kk];
	v64 += pL[6+ps*kk]*pL[4+ps*kk];
	v74 += pL[7+ps*kk]*pL[4+ps*kk];
	//
	v65 += pL[6+ps*kk]*pL[5+ps*kk];
	v75 += pL[7+ps*kk]*pL[5+ps*kk];
	//
	v76 += pL[7+ps*kk]*pL[6+ps*kk];
	kk++;
	// L 5
	v32 += pL[3+ps*kk]*pL[2+ps*kk];
	v42 += pL[4+ps*kk]*pL[2+ps*kk];
	v52 += pL[5+ps*kk]*pL[2+ps*kk];
	v62 += pL[6+ps*kk]*pL[2+ps*kk];
	v72 += pL[7+ps*kk]*pL[2+ps*kk];
	//
	v43 += pL[4+ps*kk]*pL[3+ps*kk];
	v53 += pL[5+ps*kk]*pL[3+ps*kk];
	v63 += pL[6+ps*kk]*pL[3+ps*kk];
	v73 += pL[7+ps*kk]*pL[3+ps*kk];
	//
	v54 += pL[5+ps*kk]*pL[4+ps*kk];
	v64 += pL[6+ps*kk]*pL[4+ps*kk];
	v74 += pL[7+ps*kk]*pL[4+ps*kk];
	//
	v65 += pL[6+ps*kk]*pL[5+ps*kk];
	v75 += pL[7+ps*kk]*pL[5+ps*kk];
	//
	v76 += pL[7+ps*kk]*pL[6+ps*kk];
	kk++;
	// L 4
	v43 += pL[4+ps*kk]*pL[3+ps*kk];
	v53 += pL[5+ps*kk]*pL[3+ps*kk];
	v63 += pL[6+ps*kk]*pL[3+ps*kk];
	v73 += pL[7+ps*kk]*pL[3+ps*kk];
	//
	v54 += pL[5+ps*kk]*pL[4+ps*kk];
	v64 += pL[6+ps*kk]*pL[4+ps*kk];
	v74 += pL[7+ps*kk]*pL[4+ps*kk];
	//
	v65 += pL[6+ps*kk]*pL[5+ps*kk];
	v75 += pL[7+ps*kk]*pL[5+ps*kk];
	//
	v76 += pL[7+ps*kk]*pL[6+ps*kk];
	kk++;
	// L 3
	v54 += pL[5+ps*kk]*pL[4+ps*kk];
	v64 += pL[6+ps*kk]*pL[4+ps*kk];
	v74 += pL[7+ps*kk]*pL[4+ps*kk];
	//
	v65 += pL[6+ps*kk]*pL[5+ps*kk];
	v75 += pL[7+ps*kk]*pL[5+ps*kk];
	//
	v76 += pL[7+ps*kk]*pL[6+ps*kk];
	kk++;
	// L 2
	v65 += pL[6+ps*kk]*pL[5+ps*kk];
	v75 += pL[7+ps*kk]*pL[5+ps*kk];
	//
	v76 += pL[7+ps*kk]*pL[6+ps*kk];
	kk++;
	// L 1
	v76 += pL[7+ps*kk]*pL[6+ps*kk];
	kk++;
	// L 0
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		v10 += pA[1+ps*kk]*pA[0+ps*kk];
		v20 += pA[2+ps*kk]*pA[0+ps*kk];
		v30 += pA[3+ps*kk]*pA[0+ps*kk];
		v40 += pA[4+ps*kk]*pA[0+ps*kk];
		v50 += pA[5+ps*kk]*pA[0+ps*kk];
		v60 += pA[6+ps*kk]*pA[0+ps*kk];
		v70 += pA[7+ps*kk]*pA[0+ps*kk];
		//
		v21 += pA[2+ps*kk]*pA[1+ps*kk];
		v31 += pA[3+ps*kk]*pA[1+ps*kk];
		v41 += pA[4+ps*kk]*pA[1+ps*kk];
		v51 += pA[5+ps*kk]*pA[1+ps*kk];
		v61 += pA[6+ps*kk]*pA[1+ps*kk];
		v71 += pA[7+ps*kk]*pA[1+ps*kk];
		//
		v32 += pA[3+ps*kk]*pA[2+ps*kk];
		v42 += pA[4+ps*kk]*pA[2+ps*kk];
		v52 += pA[5+ps*kk]*pA[2+ps*kk];
		v62 += pA[6+ps*kk]*pA[2+ps*kk];
		v72 += pA[7+ps*kk]*pA[2+ps*kk];
		//
		v43 += pA[4+ps*kk]*pA[3+ps*kk];
		v53 += pA[5+ps*kk]*pA[3+ps*kk];
		v63 += pA[6+ps*kk]*pA[3+ps*kk];
		v73 += pA[7+ps*kk]*pA[3+ps*kk];
		//
		v54 += pA[5+ps*kk]*pA[4+ps*kk];
		v64 += pA[6+ps*kk]*pA[4+ps*kk];
		v74 += pA[7+ps*kk]*pA[4+ps*kk];
		//
		v65 += pA[6+ps*kk]*pA[5+ps*kk];
		v75 += pA[7+ps*kk]*pA[5+ps*kk];
		//
		v76 += pA[7+ps*kk]*pA[6+ps*kk];
		}
	//
	pT[0+ps*0] = - dD[0];
	pT[1+ps*1] = - dD[1];
	pT[2+ps*2] = - dD[2];
	pT[3+ps*3] = - dD[3];
	pT[4+ps*4] = - dD[4];
	pT[5+ps*5] = - dD[5];
	pT[6+ps*6] = - dD[6];
	pT[7+ps*7] = - dD[7];
	//
	pT[0+ps*1] = - dD[1] * (v10*pT[0+ps*0]);
	pT[1+ps*2] = - dD[2] * (v21*pT[1+ps*1]);
	pT[2+ps*3] = - dD[3] * (v32*pT[2+ps*2]);
	pT[3+ps*4] = - dD[4] * (v43*pT[3+ps*3]);
	pT[4+ps*5] = - dD[5] * (v54*pT[4+ps*4]);
	pT[5+ps*6] = - dD[6] * (v65*pT[5+ps*5]);
	pT[6+ps*7] = - dD[7] * (v76*pT[6+ps*6]);
	//
	pT[0+ps*2] = - dD[2] * (v20*pT[0+ps*0] + v21*pT[0+ps*1]);
	pT[1+ps*3] = - dD[3] * (v31*pT[1+ps*1] + v32*pT[1+ps*2]);
	pT[2+ps*4] = - dD[4] * (v42*pT[2+ps*2] + v43*pT[2+ps*3]);
	pT[3+ps*5] = - dD[5] * (v53*pT[3+ps*3] + v54*pT[3+ps*4]);
	pT[4+ps*6] = - dD[6] * (v64*pT[4+ps*4] + v65*pT[4+ps*5]);
	pT[5+ps*7] = - dD[7] * (v75*pT[5+ps*5] + v76*pT[5+ps*6]);
	//
	pT[0+ps*3] = - dD[3] * (v30*pT[0+ps*0] + v31*pT[0+ps*1] + v32*pT[0+ps*2]);
	pT[1+ps*4] = - dD[4] * (v41*pT[1+ps*1] + v42*pT[1+ps*2] + v43*pT[1+ps*3]);
	pT[2+ps*5] = - dD[5] * (v52*pT[2+ps*2] + v53*pT[2+ps*3] + v54*pT[2+ps*4]);
	pT[3+ps*6] = - dD[6] * (v63*pT[3+ps*3] + v64*pT[3+ps*4] + v65*pT[3+ps*5]);
	pT[4+ps*7] = - dD[7] * (v74*pT[4+ps*4] + v75*pT[4+ps*5] + v76*pT[4+ps*6]);
	//
	pT[0+ps*4] = - dD[4] * (v40*pT[0+ps*0] + v41*pT[0+ps*1] + v42*pT[0+ps*2] + v43*pT[0+ps*3]);
	pT[1+ps*5] = - dD[5] * (v51*pT[1+ps*1] + v52*pT[1+ps*2] + v53*pT[1+ps*3] + v54*pT[1+ps*4]);
	pT[2+ps*6] = - dD[6] * (v62*pT[2+ps*2] + v63*pT[2+ps*3] + v64*pT[2+ps*4] + v65*pT[2+ps*5]);
	pT[3+ps*7] = - dD[7] * (v73*pT[3+ps*3] + v74*pT[3+ps*4] + v75*pT[3+ps*5] + v76*pT[3+ps*6]);
	//
	pT[0+ps*5] = - dD[5] * (v50*pT[0+ps*0] + v51*pT[0+ps*1] + v52*pT[0+ps*2] + v53*pT[0+ps*3] + v54*pT[0+ps*4]);
	pT[1+ps*6] = - dD[6] * (v61*pT[1+ps*1] + v62*pT[1+ps*2] + v63*pT[1+ps*3] + v64*pT[1+ps*4] + v65*pT[1+ps*5]);
	pT[2+ps*7] = - dD[7] * (v72*pT[2+ps*2] + v73*pT[2+ps*3] + v74*pT[2+ps*4] + v75*pT[2+ps*5] + v76*pT[2+ps*6]);
	//
	pT[0+ps*6] = - dD[6] * (v60*pT[0+ps*0] + v61*pT[0+ps*1] + v62*pT[0+ps*2] + v63*pT[0+ps*3] + v64*pT[0+ps*4] + v65*pT[0+ps*5]);
	pT[1+ps*7] = - dD[7] * (v71*pT[1+ps*1] + v72*pT[1+ps*2] + v73*pT[1+ps*3] + v74*pT[1+ps*4] + v75*pT[1+ps*5] + v76*pT[1+ps*6]);
	//
	pT[0+ps*7] = - dD[7] * (v70*pT[0+ps*0] + v71*pT[0+ps*1] + v72*pT[0+ps*2] + v73*pT[0+ps*3] + v74*pT[0+ps*4] + v75*pT[0+ps*5] + v76*pT[0+ps*6]);

//printf("\n%f\n", v10);
//printf("\n%f %f\n", v20, v21);
//printf("\n%f %f %f\n", v30, v31, v32);
	return;
	}



void kernel_dlarfb8_rn_lla_1_lib8(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, double *pL, double *pA)
	{
	const int ps = 8;
	double pW[8];
	int kk;
	// D 0
	pW[0+ps*0] = pD[0+ps*0];
	// D 1
	pW[0+ps*1] = pD[0+ps*1];
	// D 2
	pW[0+ps*2] = pD[0+ps*2];
	// D 3
	pW[0+ps*3] = pD[0+ps*3];
	// D 4
	pW[0+ps*4] = pD[0+ps*4];
	// D 5
	pW[0+ps*5] = pD[0+ps*5];
	// D 6
	pW[0+ps*6] = pD[0+ps*6];
	// D 7
	pW[0+ps*7] = pD[0+ps*7];
	// L
	for(kk=0; kk<=n0; kk++)
		{
		pW[0+ps*0] += pL[0+ps*kk]*pVL[0+ps*kk];
		pW[0+ps*1] += pL[0+ps*kk]*pVL[1+ps*kk];
		pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
		pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
		pW[0+ps*4] += pL[0+ps*kk]*pVL[4+ps*kk];
		pW[0+ps*5] += pL[0+ps*kk]*pVL[5+ps*kk];
		pW[0+ps*6] += pL[0+ps*kk]*pVL[6+ps*kk];
		pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
		}
	// L 6
	pW[0+ps*1] += pL[0+ps*kk]*pVL[1+ps*kk];
	pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	pW[0+ps*4] += pL[0+ps*kk]*pVL[4+ps*kk];
	pW[0+ps*5] += pL[0+ps*kk]*pVL[5+ps*kk];
	pW[0+ps*6] += pL[0+ps*kk]*pVL[6+ps*kk];
	pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
	kk++;
	// L 5
	pW[0+ps*2] += pL[0+ps*kk]*pVL[2+ps*kk];
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	pW[0+ps*4] += pL[0+ps*kk]*pVL[4+ps*kk];
	pW[0+ps*5] += pL[0+ps*kk]*pVL[5+ps*kk];
	pW[0+ps*6] += pL[0+ps*kk]*pVL[6+ps*kk];
	pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
	kk++;
	// L 4
	pW[0+ps*3] += pL[0+ps*kk]*pVL[3+ps*kk];
	pW[0+ps*4] += pL[0+ps*kk]*pVL[4+ps*kk];
	pW[0+ps*5] += pL[0+ps*kk]*pVL[5+ps*kk];
	pW[0+ps*6] += pL[0+ps*kk]*pVL[6+ps*kk];
	pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
	kk++;
	// L 3
	pW[0+ps*4] += pL[0+ps*kk]*pVL[4+ps*kk];
	pW[0+ps*5] += pL[0+ps*kk]*pVL[5+ps*kk];
	pW[0+ps*6] += pL[0+ps*kk]*pVL[6+ps*kk];
	pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
	kk++;
	// L 2
	pW[0+ps*5] += pL[0+ps*kk]*pVL[5+ps*kk];
	pW[0+ps*6] += pL[0+ps*kk]*pVL[6+ps*kk];
	pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
	kk++;
	// L 1
	pW[0+ps*6] += pL[0+ps*kk]*pVL[6+ps*kk];
	pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
	kk++;
	// L 0
	pW[0+ps*7] += pL[0+ps*kk]*pVL[7+ps*kk];
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		pW[0+ps*0] += pA[0+ps*kk]*pVA[0+ps*kk];
		pW[0+ps*1] += pA[0+ps*kk]*pVA[1+ps*kk];
		pW[0+ps*2] += pA[0+ps*kk]*pVA[2+ps*kk];
		pW[0+ps*3] += pA[0+ps*kk]*pVA[3+ps*kk];
		pW[0+ps*4] += pA[0+ps*kk]*pVA[4+ps*kk];
		pW[0+ps*5] += pA[0+ps*kk]*pVA[5+ps*kk];
		pW[0+ps*6] += pA[0+ps*kk]*pVA[6+ps*kk];
		pW[0+ps*7] += pA[0+ps*kk]*pVA[7+ps*kk];
		}
	//
	pW[0+ps*7] = pW[0+ps*0]*pT[0+ps*7] + pW[0+ps*1]*pT[1+ps*7] + pW[0+ps*2]*pT[2+ps*7] + pW[0+ps*3]*pT[3+ps*7] + pW[0+ps*4]*pT[4+ps*7] + pW[0+ps*5]*pT[5+ps*7] + pW[0+ps*6]*pT[6+ps*7] + pW[0+ps*7]*pT[7+ps*7];
	//
	pW[0+ps*6] = pW[0+ps*0]*pT[0+ps*6] + pW[0+ps*1]*pT[1+ps*6] + pW[0+ps*2]*pT[2+ps*6] + pW[0+ps*3]*pT[3+ps*6] + pW[0+ps*4]*pT[4+ps*6] + pW[0+ps*5]*pT[5+ps*6] + pW[0+ps*6]*pT[6+ps*6];
	//
	pW[0+ps*5] = pW[0+ps*0]*pT[0+ps*5] + pW[0+ps*1]*pT[1+ps*5] + pW[0+ps*2]*pT[2+ps*5] + pW[0+ps*3]*pT[3+ps*5] + pW[0+ps*4]*pT[4+ps*5] + pW[0+ps*5]*pT[5+ps*5];
	//
	pW[0+ps*4] = pW[0+ps*0]*pT[0+ps*4] + pW[0+ps*1]*pT[1+ps*4] + pW[0+ps*2]*pT[2+ps*4] + pW[0+ps*3]*pT[3+ps*4] + pW[0+ps*4]*pT[4+ps*4];
	//
	pW[0+ps*3] = pW[0+ps*0]*pT[0+ps*3] + pW[0+ps*1]*pT[1+ps*3] + pW[0+ps*2]*pT[2+ps*3] + pW[0+ps*3]*pT[3+ps*3];
	//
	pW[0+ps*2] = pW[0+ps*0]*pT[0+ps*2] + pW[0+ps*1]*pT[1+ps*2] + pW[0+ps*2]*pT[2+ps*2];
	//
	pW[0+ps*1] = pW[0+ps*0]*pT[0+ps*1] + pW[0+ps*1]*pT[1+ps*1];
	//
	pW[0+ps*0] = pW[0+ps*0]*pT[0+ps*0];
	// D 0
	pD[0+ps*0] += pW[0+ps*0];
	// D 1
	pD[0+ps*1] += pW[0+ps*1];
	// D 2
	pD[0+ps*2] += pW[0+ps*2];
	// D 3
	pD[0+ps*3] += pW[0+ps*3];
	// D 4
	pD[0+ps*4] += pW[0+ps*4];
	// D 5
	pD[0+ps*5] += pW[0+ps*5];
	// D 6
	pD[0+ps*6] += pW[0+ps*6];
	// D 7
	pD[0+ps*7] += pW[0+ps*7];
	// L
	for(kk=0; kk<=n0; kk++)
		{
		pL[0+ps*kk] += pW[0+ps*0]*pVL[0+ps*kk] + pW[0+ps*1]*pVL[1+ps*kk] + pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk] + pW[0+ps*4]*pVL[4+ps*kk] + pW[0+ps*5]*pVL[5+ps*kk] + pW[0+ps*6]*pVL[6+ps*kk] + pW[0+ps*7]*pVL[7+ps*kk];
		}
	// L 6
	pL[0+ps*kk] += pW[0+ps*1]*pVL[1+ps*kk] + pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk] + pW[0+ps*4]*pVL[4+ps*kk] + pW[0+ps*5]*pVL[5+ps*kk] + pW[0+ps*6]*pVL[6+ps*kk] + pW[0+ps*7]*pVL[7+ps*kk];
	kk++;
	// L 5
	pL[0+ps*kk] += pW[0+ps*2]*pVL[2+ps*kk] + pW[0+ps*3]*pVL[3+ps*kk] + pW[0+ps*4]*pVL[4+ps*kk] + pW[0+ps*5]*pVL[5+ps*kk] + pW[0+ps*6]*pVL[6+ps*kk] + pW[0+ps*7]*pVL[7+ps*kk];
	kk++;
	// L 4
	pL[0+ps*kk] += pW[0+ps*3]*pVL[3+ps*kk] + pW[0+ps*4]*pVL[4+ps*kk] + pW[0+ps*5]*pVL[5+ps*kk] + pW[0+ps*6]*pVL[6+ps*kk] + pW[0+ps*7]*pVL[7+ps*kk];
	kk++;
	// L 3
	pL[0+ps*kk] += pW[0+ps*4]*pVL[4+ps*kk] + pW[0+ps*5]*pVL[5+ps*kk] + pW[0+ps*6]*pVL[6+ps*kk] + pW[0+ps*7]*pVL[7+ps*kk];
	kk++;
	// L 2
	pL[0+ps*kk] += pW[0+ps*5]*pVL[5+ps*kk] + pW[0+ps*6]*pVL[6+ps*kk] + pW[0+ps*7]*pVL[7+ps*kk];
	kk++;
	// L 1
	pL[0+ps*kk] += pW[0+ps*6]*pVL[6+ps*kk] + pW[0+ps*7]*pVL[7+ps*kk];
	kk++;
	// L 0
	pL[0+ps*kk] += pW[0+ps*7]*pVL[7+ps*kk];
	kk++;
	// A
	for(kk=0; kk<n1; kk++)
		{
		pA[0+ps*kk] += pW[0+ps*0]*pVA[0+ps*kk] + pW[0+ps*1]*pVA[1+ps*kk] + pW[0+ps*2]*pVA[2+ps*kk] + pW[0+ps*3]*pVA[3+ps*kk] + pW[0+ps*4]*pVA[4+ps*kk] + pW[0+ps*5]*pVA[5+ps*kk] + pW[0+ps*6]*pVA[6+ps*kk] + pW[0+ps*7]*pVA[7+ps*kk];
		}
	return;
	}




