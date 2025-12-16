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




void blasfeo_hp_sgemv_n(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{

	if(m<0)
		return;

	const int bs = 4;

	int i;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda;
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;

	i = 0;
	// clean up at the beginning
	if(ai%bs!=0)
		{
		kernel_sgemv_n_4_gen_lib4(n, &alpha, pA, x, &beta, y-ai%bs, z-ai%bs, ai%bs, m+ai%bs);
		pA += bs*sda;
		y += 4 - ai%bs;
		z += 4 - ai%bs;
		m -= 4 - ai%bs;
		}
	// main loop
	for( ; i<m-3; i+=4)
		{
		kernel_sgemv_n_4_lib4(n, &alpha, &pA[i*sda], x, &beta, &y[i], &z[i]);
		}
	if(i<m)
		{
		kernel_sgemv_n_4_vs_lib4(n, &alpha, &pA[i*sda], x, &beta, &y[i], &z[i], m-i);
		}
		
	return;

	}



void blasfeo_hp_sgemv_t(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{

	if(n<=0)
		return;
	
	const int bs = 4;

	int i;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int offsetA = ai%bs;
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;

	i = 0;
	for( ; i<n-3; i+=4)
		{
		kernel_sgemv_t_4_lib4(m, &alpha, offsetA, &pA[i*bs], sda, x, &beta, &y[i], &z[i]);
		}
	if(i<n)
		{
		kernel_sgemv_t_4_vs_lib4(m, &alpha, offsetA, &pA[i*bs], sda, x, &beta, &y[i], &z[i], n-i);
		}
	
	return;

	}



void blasfeo_hp_sgemv_nt(int m, int n, float alpha_n, float alpha_t, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx_n, int xi_n, struct blasfeo_svec *sx_t, int xi_t, float beta_n, float beta_t, struct blasfeo_svec *sy_n, int yi_n, struct blasfeo_svec *sy_t, int yi_t, struct blasfeo_svec *sz_n, int zi_n, struct blasfeo_svec *sz_t, int zi_t)
	{

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_sgemv_nt(m, n, alpha_n, alpha_t, sA, ai, aj, sx_n, xi_n, sx_t, xi_t, beta_n, beta_t, sy_n, yi_n, sy_t, yi_t, sz_n, zi_n, sz_t, zi_t);
		return;
#else
		printf("\nblasfeo_sgemv_nt: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs; // TODO ai
	float *x_n = sx_n->pa + xi_n;
	float *x_t = sx_t->pa + xi_t;
	float *y_n = sy_n->pa + yi_n;
	float *y_t = sy_t->pa + yi_t;
	float *z_n = sz_n->pa + zi_n;
	float *z_t = sz_t->pa + zi_t;

//	sgemv_nt_lib(m, n, alpha_n, alpha_t, pA, sda, x_n, x_t, beta_n, beta_t, y_n, y_t, z_n, z_t);

//	if(m<=0 | n<=0)
//		return;

	int ii;

	// copy and scale y_n int z_n
	if(beta_n==0.0)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z_n[ii+0] = 0.0;
			z_n[ii+1] = 0.0;
			z_n[ii+2] = 0.0;
			z_n[ii+3] = 0.0;
			}
		for(; ii<m; ii++)
			{
			z_n[ii+0] = 0.0;
			}
		}
	else
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z_n[ii+0] = beta_n*y_n[ii+0];
			z_n[ii+1] = beta_n*y_n[ii+1];
			z_n[ii+2] = beta_n*y_n[ii+2];
			z_n[ii+3] = beta_n*y_n[ii+3];
			}
		for(; ii<m; ii++)
			{
			z_n[ii+0] = beta_n*y_n[ii+0];
			}
		}
	
	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_sgemv_nt_4_lib4(m, &alpha_n, &alpha_t, pA+ii*bs, sda, x_n+ii, x_t, &beta_t, y_t+ii, z_n, z_t+ii);
		}
	if(ii<n)
		{
		kernel_sgemv_nt_4_vs_lib4(m, &alpha_n, &alpha_t, pA+ii*bs, sda, x_n+ii, x_t, &beta_t, y_t+ii, z_n, z_t+ii, n-ii);
		}
	
		return;
	}



void blasfeo_hp_ssymv_l(int m, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return;
	
	const int bs = 4;

	int ii, n1;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;

	// copy and scale y int z
	if(beta==0.0)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z[ii+0] = 0.0;
			z[ii+1] = 0.0;
			z[ii+2] = 0.0;
			z[ii+3] = 0.0;
			}
		for(; ii<m; ii++)
			{
			z[ii+0] = 0.0;
			}
		}
	else
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z[ii+0] = beta*y[ii+0];
			z[ii+1] = beta*y[ii+1];
			z[ii+2] = beta*y[ii+2];
			z[ii+3] = beta*y[ii+3];
			}
		for(; ii<m; ii++)
			{
			z[ii+0] = beta*y[ii+0];
			}
		}
	
	// clean up at the beginning
	if(ai%bs!=0) // 1, 2, 3
		{
		n1 = 4-ai%bs;
		kernel_ssymv_l_4_gen_lib4(m, &alpha, ai%bs, &pA[0], sda, &x[0], &z[0], m<n1 ? m : n1);
		pA += n1 + n1*bs + (sda-1)*bs;
		x += n1;
		z += n1;
		m -= n1;
		}
	// main loop
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		kernel_ssymv_l_4_lib4(m-ii, &alpha, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii]);
		}
	// clean up at the end
	if(ii<m)
		{
		kernel_ssymv_l_4_gen_lib4(m-ii, &alpha, 0, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii], m-ii);
		}
	
	return;
	}



// m >= n
void blasfeo_hp_ssymv_l_mn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{

	if(m<n)
		n = m;

	if(m<=0 | n<=0)
		return;
	
	const int bs = 4;

	int ii, n1;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;

	// copy and scale y int z
	if(beta==0.0)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z[ii+0] = 0.0;
			z[ii+1] = 0.0;
			z[ii+2] = 0.0;
			z[ii+3] = 0.0;
			}
		for(; ii<m; ii++)
			{
			z[ii+0] = 0.0;
			}
		}
	else
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z[ii+0] = beta*y[ii+0];
			z[ii+1] = beta*y[ii+1];
			z[ii+2] = beta*y[ii+2];
			z[ii+3] = beta*y[ii+3];
			}
		for(; ii<m; ii++)
			{
			z[ii+0] = beta*y[ii+0];
			}
		}
	
	// clean up at the beginning
	if(ai%bs!=0) // 1, 2, 3
		{
		n1 = 4-ai%bs;
		kernel_ssymv_l_4_gen_lib4(m, &alpha, ai%bs, &pA[0], sda, &x[0], &z[0], n<n1 ? n : n1);
		pA += n1 + n1*bs + (sda-1)*bs;
		x += n1;
		z += n1;
		m -= n1;
		n -= n1;
		}
	// main loop
	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_ssymv_l_4_lib4(m-ii, &alpha, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii]);
		}
	// clean up at the end
	if(ii<n)
		{
		kernel_ssymv_l_4_gen_lib4(m-ii, &alpha, 0, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii], n-ii);
		}
	
	return;
	}



void blasfeo_hp_ssymv_u(int m, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssymv_u(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
#else
	printf("\nblasfeo_ssymv_u: feature not implemented yet\n");
	exit(1);
#endif
	}



// m >= n
static void blasfeo_hp_strmv_lnn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

	if(m-n>0)
		blasfeo_hp_sgemv_n(m-n, n, 1.0, sA, ai+n, aj, sx, xi, 0.0, sz, zi+n, sz, zi+n);

	float *pA2 = pA;
	float *z2 = z;
	int m2 = n;
	int n2 = 0;
	float *pA3, *x3;

	float alpha = 1.0;
	float beta = 1.0;

	float zt[4];

	int ii, jj, jj_end;

	ii = 0;

	if(ai%4!=0)
		{
		pA2 += sda*bs - ai%bs;
		z2 += bs-ai%bs;
		m2 -= bs-ai%bs;
		n2 += bs-ai%bs;
		}
	
	pA2 += m2/bs*bs*sda;
	z2 += m2/bs*bs;
	n2 += m2/bs*bs;

	if(m2%bs!=0)
		{
		//
		pA3 = pA2 + bs*n2;
		x3 = x + n2;
		zt[3] = pA3[3+bs*0]*x3[0] + pA3[3+bs*1]*x3[1] + pA3[3+bs*2]*x3[2] + pA3[3+bs*3]*x3[3];
		zt[2] = pA3[2+bs*0]*x3[0] + pA3[2+bs*1]*x3[1] + pA3[2+bs*2]*x3[2];
		zt[1] = pA3[1+bs*0]*x3[0] + pA3[1+bs*1]*x3[1];
		zt[0] = pA3[0+bs*0]*x3[0];
		kernel_sgemv_n_4_lib4(n2, &alpha, pA2, x, &beta, zt, zt);
		for(jj=0; jj<m2%bs; jj++)
			z2[jj] = zt[jj];
		}
	for(; ii<m2-3; ii+=4)
		{
		pA2 -= bs*sda;
		z2 -= 4;
		n2 -= 4;
		pA3 = pA2 + bs*n2;
		x3 = x + n2;
		z2[3] = pA3[3+bs*0]*x3[0] + pA3[3+bs*1]*x3[1] + pA3[3+bs*2]*x3[2] + pA3[3+bs*3]*x3[3];
		z2[2] = pA3[2+bs*0]*x3[0] + pA3[2+bs*1]*x3[1] + pA3[2+bs*2]*x3[2];
		z2[1] = pA3[1+bs*0]*x3[0] + pA3[1+bs*1]*x3[1];
		z2[0] = pA3[0+bs*0]*x3[0];
		kernel_sgemv_n_4_lib4(n2, &alpha, pA2, x, &beta, z2, z2);
		}
	if(ai%4!=0)
		{
		if(ai%bs==1)
			{
			zt[2] = pA[2+bs*0]*x[0] + pA[2+bs*1]*x[1] + pA[2+bs*2]*x[2];
			zt[1] = pA[1+bs*0]*x[0] + pA[1+bs*1]*x[1];
			zt[0] = pA[0+bs*0]*x[0];
			jj_end = 4-ai%bs<n ? 4-ai%bs : n;
			for(jj=0; jj<jj_end; jj++)
				z[jj] = zt[jj];
			}
		else if(ai%bs==2)
			{
			zt[1] = pA[1+bs*0]*x[0] + pA[1+bs*1]*x[1];
			zt[0] = pA[0+bs*0]*x[0];
			jj_end = 4-ai%bs<n ? 4-ai%bs : n;
			for(jj=0; jj<jj_end; jj++)
				z[jj] = zt[jj];
			}
		else // if (ai%bs==3)
			{
			z[0] = pA[0+bs*0]*x[0];
			}
		}

	return;

	}



void blasfeo_hp_strmv_lnn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_lnn_mn(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



// m >= n
static void blasfeo_hp_strmv_ltn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

	float xt[4];
	float zt[4];

	float alpha = 1.0;
	float beta = 1.0;

	int ii, jj, ll, ll_max;

	jj = 0;

	if(ai%bs!=0)
		{

		if(ai%bs==1)
			{
			ll_max = m-jj<3 ? m-jj : 3;
			for(ll=0; ll<ll_max; ll++)
				xt[ll] = x[ll];
			for(; ll<3; ll++)
				xt[ll] = 0.0;
			zt[0] = pA[0+bs*0]*xt[0] + pA[1+bs*0]*xt[1] + pA[2+bs*0]*xt[2];
			zt[1] = pA[1+bs*1]*xt[1] + pA[2+bs*1]*xt[2];
			zt[2] = pA[2+bs*2]*xt[2];
			pA += bs*sda - 1;
			x += 3;
			kernel_sgemv_t_4_lib4(m-3-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
			ll_max = n-jj<3 ? n-jj : 3;
			for(ll=0; ll<ll_max; ll++)
				z[ll] = zt[ll];
			pA += bs*3;
			z += 3;
			jj += 3;
			}
		else if(ai%bs==2)
			{
			ll_max = m-jj<2 ? m-jj : 2;
			for(ll=0; ll<ll_max; ll++)
				xt[ll] = x[ll];
			for(; ll<2; ll++)
				xt[ll] = 0.0;
			zt[0] = pA[0+bs*0]*xt[0] + pA[1+bs*0]*xt[1];
			zt[1] = pA[1+bs*1]*xt[1];
			pA += bs*sda - 2;
			x += 2;
			kernel_sgemv_t_4_lib4(m-2-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
			ll_max = n-jj<2 ? n-jj : 2;
			for(ll=0; ll<ll_max; ll++)
				z[ll] = zt[ll];
			pA += bs*2;
			z += 2;
			jj += 2;
			}
		else // if(ai%bs==3)
			{
			ll_max = m-jj<1 ? m-jj : 1;
			for(ll=0; ll<ll_max; ll++)
				xt[ll] = x[ll];
			for(; ll<1; ll++)
				xt[ll] = 0.0;
			zt[0] = pA[0+bs*0]*xt[0];
			pA += bs*sda - 3;
			x += 1;
			kernel_sgemv_t_4_lib4(m-1-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
			ll_max = n-jj<1 ? n-jj : 1;
			for(ll=0; ll<ll_max; ll++)
				z[ll] = zt[ll];
			pA += bs*1;
			z += 1;
			jj += 1;
			}

		}
	
	for(; jj<n-3; jj+=4)
		{
		zt[0] = pA[0+bs*0]*x[0] + pA[1+bs*0]*x[1] + pA[2+bs*0]*x[2] + pA[3+bs*0]*x[3];
		zt[1] = pA[1+bs*1]*x[1] + pA[2+bs*1]*x[2] + pA[3+bs*1]*x[3];
		zt[2] = pA[2+bs*2]*x[2] + pA[3+bs*2]*x[3];
		zt[3] = pA[3+bs*3]*x[3];
		pA += bs*sda;
		x += 4;
		kernel_sgemv_t_4_lib4(m-4-jj, &alpha, 0, pA, sda, x, &beta, zt, z);
		pA += bs*4;
		z += 4;
		}
	if(jj<n)
		{
		ll_max = m-jj<4 ? m-jj : 4;
		for(ll=0; ll<ll_max; ll++)
			xt[ll] = x[ll];
		for(; ll<4; ll++)
			xt[ll] = 0.0;
		zt[0] = pA[0+bs*0]*xt[0] + pA[1+bs*0]*xt[1] + pA[2+bs*0]*xt[2] + pA[3+bs*0]*xt[3];
		zt[1] = pA[1+bs*1]*xt[1] + pA[2+bs*1]*xt[2] + pA[3+bs*1]*xt[3];
		zt[2] = pA[2+bs*2]*xt[2] + pA[3+bs*2]*xt[3];
		zt[3] = pA[3+bs*3]*xt[3];
		pA += bs*sda;
		x += 4;
		kernel_sgemv_t_4_lib4(m-4-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
		for(ll=0; ll<n-jj; ll++)
			z[ll] = zt[ll];
//		pA += bs*4;
//		z += 4;
		}

	return;

	}



void blasfeo_hp_strmv_ltn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_ltn_mn(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



void blasfeo_hp_strmv_unn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return;

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strmv_unn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_strmv_unn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs; // TODO ai
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

	int i;
	
	i=0;
	for(; i<m-3; i+=4)
		{
		kernel_strmv_un_4_lib4(m-i, pA, x, z);
		pA += 4*sda+4*bs;
		x  += 4;
		z  += 4;
		}
	if(m>i)
		{
		if(m-i==1)
			{
			z[0] = pA[0+bs*0]*x[0];
			}
		else if(m-i==2)
			{
			z[0] = pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1];
			z[1] = pA[1+bs*1]*x[1];
			}
		else // if(m-i==3)
			{
			z[0] = pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1] + pA[0+bs*2]*x[2];
			z[1] = pA[1+bs*1]*x[1] + pA[1+bs*2]*x[2];
			z[2] = pA[2+bs*2]*x[2];
			}
		}

	return;

	}



void blasfeo_hp_strmv_utn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return;

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strmv_utn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_strmv_utn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs; // TODO ai
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

	int ii, idx;
	
	float *ptrA;
	
	ii=0;
	idx = m/bs*bs;
	if(m%bs!=0)
		{
		kernel_strmv_ut_4_vs_lib4(m, pA+idx*bs, sda, x, z+idx, m%bs);
		ii += m%bs;
		}
	idx -= 4;
	for(; ii<m; ii+=4)
		{
		kernel_strmv_ut_4_lib4(idx+4, pA+idx*bs, sda, x, z+idx);
		idx -= 4;
		}

	return;

	}



void blasfeo_hp_strsv_lnn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m==0)
		return;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_lnn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_lnn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_lnn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_lnn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_lnn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_lnn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_strsv_lnn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_lnn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_lnn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsv_lnn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_strsv_lnn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs; // TODO ai
	float *dA = sA->dA;
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

	int ii;

	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			sdiaex_lib(m, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		sdiaex_lib(m, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}

	int i;

	if(x!=z)
		{
		for(i=0; i<m; i++)
			z[i] = x[i];
		}
	
	i = 0;
	for( ; i<m-3; i+=4)
		{
		kernel_strsv_ln_inv_4_lib4(i, &pA[i*sda], &dA[i], z, &z[i], &z[i]);
		}
	if(i<m)
		{
		kernel_strsv_ln_inv_4_vs_lib4(i, &pA[i*sda], &dA[i], z, &z[i], &z[i], m-i, m-i);
		i+=4;
		}

	return;

	}



void blasfeo_hp_strsv_lnn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m==0 | n==0)
		return;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_lnn_mn : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_strsv_lnn_mn : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_lnn_mn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_lnn_mn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_lnn_mn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_lnn_mn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_lnn_mn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_strsv_lnn_mn : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_lnn_mn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_lnn_mn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsv_lnn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_strsv_lnn_mn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs; // TODO ai
	float *dA = sA->dA;
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

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

	if(m<n)
		m = n;

	float alpha = -1.0;
	float beta = 1.0;

	int i;

	if(x!=z)
		{
		for(i=0; i<m; i++)
			z[i] = x[i];
		}
	
	i = 0;
	for( ; i<n-3; i+=4)
		{
		kernel_strsv_ln_inv_4_lib4(i, &pA[i*sda], &dA[i], z, &z[i], &z[i]);
		}
	if(i<n)
		{
		kernel_strsv_ln_inv_4_vs_lib4(i, &pA[i*sda], &dA[i], z, &z[i], &z[i], m-i, n-i);
		i+=4;
		}
	for( ; i<m-3; i+=4)
		{
		kernel_sgemv_n_4_lib4(n, &alpha, &pA[i*sda], z, &beta, &z[i], &z[i]);
		}
	if(i<m)
		{
		kernel_sgemv_n_4_vs_lib4(n, &alpha, &pA[i*sda], z, &beta, &z[i], &z[i], m-i);
		i+=4;
		}

	return;

	}



void blasfeo_hp_strsv_ltn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m==0)
		return;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_ltn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_ltn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_ltn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_ltn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_ltn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_ltn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_strsv_ltn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_ltn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_ltn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_strsv_ltn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs; // TODO ai
	float *dA = sA->dA;
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

	int ii;

	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			sdiaex_lib(m, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		sdiaex_lib(m, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}

	int i;
	
	if(x!=z)
		for(i=0; i<m; i++)
			z[i] = x[i];
			
	i=0;
	if(m%4==1)
		{
		kernel_strsv_lt_inv_1_lib4(i+1, &pA[m/bs*bs*sda+(m-i-1)*bs], sda, &dA[m-i-1], &z[m-i-1], &z[m-i-1], &z[m-i-1]);
		i++;
		}
	else if(m%4==2)
		{
		kernel_strsv_lt_inv_2_lib4(i+2, &pA[m/bs*bs*sda+(m-i-2)*bs], sda, &dA[m-i-2], &z[m-i-2], &z[m-i-2], &z[m-i-2]);
		i+=2;
		}
	else if(m%4==3)
		{
		kernel_strsv_lt_inv_3_lib4(i+3, &pA[m/bs*bs*sda+(m-i-3)*bs], sda, &dA[m-i-3], &z[m-i-3], &z[m-i-3], &z[m-i-3]);
		i+=3;
		}
	for(; i<m-3; i+=4)
		{
		kernel_strsv_lt_inv_4_lib4(i+4, &pA[(m-i-4)/bs*bs*sda+(m-i-4)*bs], sda, &dA[m-i-4], &z[m-i-4], &z[m-i-4], &z[m-i-4]);
		}

	return;

	}



void blasfeo_hp_strsv_ltn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{

	if(m==0)
		return;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_ltn_mn : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_strsv_ltn_mn : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_ltn_mn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_ltn_mn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_ltn_mn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_ltn_mn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_ltn_mn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_strsv_ltn_mn : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_ltn_mn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_ltn_mn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_strsv_ltn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_strsv_ltn_mn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	float *pA = sA->pA + aj*bs; // TODO ai
	float *dA = sA->dA;
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;

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

	if(n>m)
		n = m;
	
	int i;
	
	if(x!=z)
		for(i=0; i<m; i++)
			z[i] = x[i];
			
	i=0;
	if(n%4==1)
		{
		kernel_strsv_lt_inv_1_lib4(m-n+i+1, &pA[n/bs*bs*sda+(n-i-1)*bs], sda, &dA[n-i-1], &z[n-i-1], &z[n-i-1], &z[n-i-1]);
		i++;
		}
	else if(n%4==2)
		{
		kernel_strsv_lt_inv_2_lib4(m-n+i+2, &pA[n/bs*bs*sda+(n-i-2)*bs], sda, &dA[n-i-2], &z[n-i-2], &z[n-i-2], &z[n-i-2]);
		i+=2;
		}
	else if(n%4==3)
		{
		kernel_strsv_lt_inv_3_lib4(m-n+i+3, &pA[n/bs*bs*sda+(n-i-3)*bs], sda, &dA[n-i-3], &z[n-i-3], &z[n-i-3], &z[n-i-3]);
		i+=3;
		}
	for(; i<n-3; i+=4)
		{
		kernel_strsv_lt_inv_4_lib4(m-n+i+4, &pA[(n-i-4)/bs*bs*sda+(n-i-4)*bs], sda, &dA[n-i-4], &z[n-i-4], &z[n-i-4], &z[n-i-4]);
		}

	return;

	}



void blasfeo_hp_strsv_lnu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_lnu : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_lnu : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_lnu : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_lnu : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_lnu : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_lnu : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_strsv_lnu : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_lnu : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_lnu : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
	return;
#else
	printf("\nblasfeo_strsv_lnu: feature not implemented yet: ai=%d\n", ai);
	exit(1);
#endif
	}



void blasfeo_hp_strsv_ltu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_ltu : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_ltu : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_ltu : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_ltu : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_ltu : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_ltu : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_strsv_ltu : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_ltu : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_ltu : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
	return;
#else
	printf("\nblasfeo_strsv_ltu : feature not implemented yet *****\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_unn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_unn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_unn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_unn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_unn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_unn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_unn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_strsv_unn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_unn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_unn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_unn(m, sA, ai, aj, sx, xi, sz, zi);
	return;
#else
	printf("\nblasfeo_strsv_unn : feature not implemented yet *****\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_utn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_strsv_utn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_strsv_utn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_strsv_utn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_strsv_utn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_strsv_utn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_strsv_utn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_strsv_utn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_strsv_utn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_strsv_utn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_utn(m, sA, ai, aj, sx, xi, sz, zi);
	return;
#else
	printf("\n***** blasfeo_strsv_utn : feature not implemented yet *****\n");
	exit(1);
#endif
	}



void blasfeo_hp_sger(int m, int n, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sger(m, n, alpha, sx, xi, sy, yi, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_sger: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_sgemv_n(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_sgemv_n(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_sgemv_t(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_sgemv_t(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_sgemv_nt(int m, int n, float alpha_n, float alpha_t, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx_n, int xi_n, struct blasfeo_svec *sx_t, int xi_t, float beta_n, float beta_t, struct blasfeo_svec *sy_n, int yi_n, struct blasfeo_svec *sy_t, int yi_t, struct blasfeo_svec *sz_n, int zi_n, struct blasfeo_svec *sz_t, int zi_t)
	{
	blasfeo_hp_sgemv_nt(m, n, alpha_n, alpha_t, sA, ai, aj, sx_n, xi_n, sx_t, xi_t, beta_n, beta_t, sy_n, yi_n, sy_t, yi_t, sz_n, zi_n, sz_t, zi_t);
	}



void blasfeo_ssymv_l(int m, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_ssymv_l(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_ssymv_l_mn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_ssymv_l_mn(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_ssymv_u(int m, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_ssymv_u(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_strmv_lnn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_lnn(m, sA, ai, aj, sx, xi, sz, zi);
	}



//void blasfeo_strmv_lnu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
//	{
//	blasfeo_hp_strmv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
//	}



void blasfeo_strmv_ltn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
	}



//void blasfeo_strmv_ltu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
//	{
//	blasfeo_hp_strmv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
//	}



void blasfeo_strmv_unn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_unn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strmv_utn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_utn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_lnn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_lnn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_lnn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_lnn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_lnu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_ltn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_ltn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_ltn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_ltu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_unn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_unn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strsv_utn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strsv_utn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_sger(int m, int n, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
	blasfeo_hp_sger(m, n, alpha, sx, xi, sy, yi, sC, ci, cj, sD, di, dj);
	}



#endif
