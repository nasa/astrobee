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
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_d_blasfeo_ref_api.h>
#endif



// TODO CHECK THE EARLY RETURN CONDITIONS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



void blasfeo_hp_dgemv_n(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<0)
		return;

	const int bs = 4;

	int i;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;
	double *z = sz->pa + zi;

	i = 0;
	// clean up at the beginning
	if(ai%bs!=0)
		{
		kernel_dgemv_n_4_gen_lib4(n, &alpha, pA, x, &beta, y-ai%bs, z-ai%bs, ai%bs, m+ai%bs);
		pA += bs*sda;
		y += 4 - ai%bs;
		z += 4 - ai%bs;
		m -= 4 - ai%bs;
		}
	// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for( ; i<m-11; i+=12)
		{
		kernel_dgemv_n_12_lib4(n, &alpha, &pA[i*sda], sda, x, &beta, &y[i], &z[i]);
		}
#endif
	for( ; i<m-7; i+=8)
		{
		kernel_dgemv_n_8_lib4(n, &alpha, &pA[i*sda], sda, x, &beta, &y[i], &z[i]);
		}
	if(i<m-3)
		{
		kernel_dgemv_n_4_lib4(n, &alpha, &pA[i*sda], x, &beta, &y[i], &z[i]);
		i+=4;
		}
#else
	for( ; i<m-3; i+=4)
		{
		kernel_dgemv_n_4_lib4(n, &alpha, &pA[i*sda], x, &beta, &y[i], &z[i]);
		}
#endif
	if(i<m)
		{
		kernel_dgemv_n_4_vs_lib4(n, &alpha, &pA[i*sda], x, &beta, &y[i], &z[i], m-i);
		}
		
	return;

	}



void blasfeo_hp_dgemv_t(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{

	if(n<=0)
		return;
	
	const int bs = 4;

	int i;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int offsetA = ai%bs;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;
	double *z = sz->pa + zi;

	i = 0;
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for( ; i<n-11; i+=12)
		{
		kernel_dgemv_t_12_lib4(m, &alpha, offsetA, &pA[i*bs], sda, x, &beta, &y[i], &z[i]);
		}
#endif
	for( ; i<n-7; i+=8)
		{
		kernel_dgemv_t_8_lib4(m, &alpha, offsetA, &pA[i*bs], sda, x, &beta, &y[i], &z[i]);
		}
	if(i<n-3)
		{
		kernel_dgemv_t_4_lib4(m, &alpha, offsetA, &pA[i*bs], sda, x, &beta, &y[i], &z[i]);
		i+=4;
		}
#else
	for( ; i<n-3; i+=4)
		{
		kernel_dgemv_t_4_lib4(m, &alpha, offsetA, &pA[i*bs], sda, x, &beta, &y[i], &z[i]);
		}
#endif
	if(i<n)
		{
		kernel_dgemv_t_4_vs_lib4(m, &alpha, offsetA, &pA[i*bs], sda, x, &beta, &y[i], &z[i], n-i);
		}
	
	return;

	}



void blasfeo_hp_dgemv_nt(int m, int n, double alpha_n, double alpha_t, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx_n, int xi_n, struct blasfeo_dvec *sx_t, int xi_t, double beta_n, double beta_t, struct blasfeo_dvec *sy_n, int yi_n, struct blasfeo_dvec *sy_t, int yi_t, struct blasfeo_dvec *sz_n, int zi_n, struct blasfeo_dvec *sz_t, int zi_t)
	{
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dgemv_nt(m, n, alpha_n, alpha_t, sA, ai, aj, sx_n, xi_n, sx_t, xi_t, beta_n, beta_t, sy_n, yi_n, sy_t, yi_t, sz_n, zi_n, sz_t, zi_t);
		return;
#else
		printf("\nblasfeo_dgemv_nt: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	const int bs = 4;
#if defined(TARGET_X86_AMD_BARCELONA) | defined(TARGET_X86_AMD_JAGUAR)
	blasfeo_hp_dgemv_n(m, n, alpha_n, sA, ai, aj, sx_n, xi_n, beta_n, sy_n, yi_n, sz_n, zi_n);
	blasfeo_hp_dgemv_t(m, n, alpha_t, sA, ai, aj, sx_t, xi_t, beta_t, sy_t, yi_t, sz_t, zi_t);
	return;
#endif
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *x_n = sx_n->pa + xi_n;
	double *x_t = sx_t->pa + xi_t;
	double *y_n = sy_n->pa + yi_n;
	double *y_t = sy_t->pa + yi_t;
	double *z_n = sz_n->pa + zi_n;
	double *z_t = sz_t->pa + zi_t;

	if(m<=0 | n<=0)
		return;

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
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-5; ii+=6)
		{
		kernel_dgemv_nt_6_lib4(m, &alpha_n, &alpha_t, pA+ii*bs, sda, x_n+ii, x_t, &beta_t, y_t+ii, z_n, z_t+ii);
		}
#endif
	for(; ii<n-3; ii+=4)
		{
		kernel_dgemv_nt_4_lib4(m, &alpha_n, &alpha_t, pA+ii*bs, sda, x_n+ii, x_t, &beta_t, y_t+ii, z_n, z_t+ii);
		}
	if(ii<n)
		{
		kernel_dgemv_nt_4_vs_lib4(m, &alpha_n, &alpha_t, pA+ii*bs, sda, x_n+ii, x_t, &beta_t, y_t+ii, z_n, z_t+ii, n-ii);
		}
	
	return;

	}



void blasfeo_hp_dsymv_l(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{

	if((m<=0) | (alpha==0 & beta==0))
		return;

	const int bs = 4;

	int ii, n1;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;
	double *z = sz->pa + zi;

	// copy and scale y into z
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
		kernel_dsymv_l_4_gen_lib4(m, &alpha, ai%bs, &pA[0], sda, &x[0], &z[0], m<n1 ? m : n1);
		pA += n1 + n1*bs + (sda-1)*bs;
		x += n1;
		z += n1;
		m -= n1;
		}

#if defined(TARGET_X86_AMD_BARCELONA) | defined(TARGET_X86_AMD_JAGUAR)
	// using dgemv_n and dgemv_t kernels
	double beta1 = 1.0;
	double xx[4];
	for(ii=0; ii<m-3; ii+=4)
		{
		// gemv_n
		kernel_dgemv_n_4_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii);
		// 4x4
		z[ii+0] += alpha*(pA[ii*sda+0+bs*(ii+0)]*x[ii+0] + pA[ii*sda+1+bs*(ii+0)]*x[ii+1] + pA[ii*sda+2+bs*(ii+0)]*x[ii+2] + pA[ii*sda+3+bs*(ii+0)]*x[ii+3]);
		z[ii+1] += alpha*(pA[ii*sda+1+bs*(ii+0)]*x[ii+0] + pA[ii*sda+1+bs*(ii+1)]*x[ii+1] + pA[ii*sda+2+bs*(ii+1)]*x[ii+2] + pA[ii*sda+3+bs*(ii+1)]*x[ii+3]);
		z[ii+2] += alpha*(pA[ii*sda+2+bs*(ii+0)]*x[ii+0] + pA[ii*sda+2+bs*(ii+1)]*x[ii+1] + pA[ii*sda+2+bs*(ii+2)]*x[ii+2] + pA[ii*sda+3+bs*(ii+2)]*x[ii+3]);
		z[ii+3] += alpha*(pA[ii*sda+3+bs*(ii+0)]*x[ii+0] + pA[ii*sda+3+bs*(ii+1)]*x[ii+1] + pA[ii*sda+3+bs*(ii+2)]*x[ii+2] + pA[ii*sda+3+bs*(ii+3)]*x[ii+3]);
		// gemv_t
		kernel_dgemv_t_4_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii);
		}
	if(ii<m)
		{
		if(m-ii==1)
			{
			xx[0] = alpha*x[ii+0];
			xx[1] = alpha*x[ii+1];
			xx[2] = alpha*x[ii+2];
			xx[3] = alpha*x[ii+3];
			// gemv_n
			kernel_dgemv_n_4_vs_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii, m-ii);
			// 4x4
			z[ii+0] += pA[ii*sda+0+bs*(ii+0)]*xx[0];
			if(m-ii>1)
				{
				z[ii+0] += pA[ii*sda+1+bs*(ii+0)]*xx[1];
				z[ii+1] += pA[ii*sda+1+bs*(ii+0)]*xx[0];
				}
			if(m-ii>2)
				{
				z[ii+0] += pA[ii*sda+2+bs*(ii+0)]*xx[2];
				z[ii+2] += pA[ii*sda+2+bs*(ii+0)]*xx[0];
				}
			if(m-ii>3)
				{
				z[ii+0] += pA[ii*sda+3+bs*(ii+0)]*xx[3];
				z[ii+3] += pA[ii*sda+3+bs*(ii+0)]*xx[0];
				}
			// gemv_t
			kernel_dgemv_t_4_vs_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii, m-ii);
			ii += 4;
			}
		else if(m-ii==2)
			{
			xx[0] = alpha*x[ii+0];
			xx[1] = alpha*x[ii+1];
			xx[2] = alpha*x[ii+2];
			xx[3] = alpha*x[ii+3];
			// gemv_n
			kernel_dgemv_n_4_vs_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii, m-ii);
			// 4x4
			z[ii+0] += pA[ii*sda+0+bs*(ii+0)]*xx[0];
			if(m-ii>1)
				{
				z[ii+0] += pA[ii*sda+1+bs*(ii+0)]*xx[1];
				z[ii+1] += pA[ii*sda+1+bs*(ii+0)]*xx[0] + pA[ii*sda+1+bs*(ii+1)]*xx[1];
				}
			if(m-ii>2)
				{
				z[ii+0] += pA[ii*sda+2+bs*(ii+0)]*xx[2];
				z[ii+1] += pA[ii*sda+2+bs*(ii+1)]*xx[2];
				z[ii+2] += pA[ii*sda+2+bs*(ii+0)]*xx[0] + pA[ii*sda+2+bs*(ii+1)]*xx[1];
				}
			if(m-ii>3)
				{
				z[ii+0] += pA[ii*sda+3+bs*(ii+0)]*xx[3];
				z[ii+1] += pA[ii*sda+3+bs*(ii+1)]*xx[3];
				z[ii+3] += pA[ii*sda+3+bs*(ii+0)]*xx[0] + pA[ii*sda+3+bs*(ii+1)]*xx[1];
				}
			// gemv_t
			kernel_dgemv_t_4_vs_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii, m-ii);
			ii += 4;
			}
		else // if(m-ii==3)
			{
			xx[0] = alpha*x[ii+0];
			xx[1] = alpha*x[ii+1];
			xx[2] = alpha*x[ii+2];
			xx[3] = alpha*x[ii+3];
			// gemv_n
			kernel_dgemv_n_4_vs_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii, m-ii);
			// 4x4
			z[ii+0] += pA[ii*sda+0+bs*(ii+0)]*xx[0];
			if(m-ii>1)
				{
				z[ii+0] += pA[ii*sda+1+bs*(ii+0)]*xx[1];
				z[ii+1] += pA[ii*sda+1+bs*(ii+0)]*xx[0] + pA[ii*sda+1+bs*(ii+1)]*xx[1];
				}
			if(m-ii>2)
				{
				z[ii+0] += pA[ii*sda+2+bs*(ii+0)]*xx[2];
				z[ii+1] += pA[ii*sda+2+bs*(ii+1)]*xx[2];
				z[ii+2] += pA[ii*sda+2+bs*(ii+0)]*xx[0] + pA[ii*sda+2+bs*(ii+1)]*xx[1] + pA[ii*sda+2+bs*(ii+2)]*xx[2];
				}
			if(m-ii>3)
				{
				z[ii+0] += pA[ii*sda+3+bs*(ii+0)]*xx[3];
				z[ii+1] += pA[ii*sda+3+bs*(ii+1)]*xx[3];
				z[ii+2] += pA[ii*sda+3+bs*(ii+2)]*xx[3];
				z[ii+3] += pA[ii*sda+3+bs*(ii+0)]*xx[0] + pA[ii*sda+3+bs*(ii+1)]*xx[1] + pA[ii*sda+3+bs*(ii+2)]*xx[2];
				}
			// gemv_t
			kernel_dgemv_t_4_vs_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii, m-ii);
			ii += 4;
			}
		}
	return;
#endif

	// main loop
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		kernel_dsymv_l_4_lib4(m-ii, &alpha, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii]);
		}
	// clean up at the end
	if(ii<m)
		{
		kernel_dsymv_l_4_gen_lib4(m-ii, &alpha, 0, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii], m-ii);
		}
	
	return;
	}


// m >= n
void blasfeo_hp_dsymv_l_mn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<n)
		n = m;

	if(m<=0 | n<=0)
		return;
	
	const int bs = 4;

	int ii, n1;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;
	double *z = sz->pa + zi;

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
		kernel_dsymv_l_4_gen_lib4(m, &alpha, ai%bs, &pA[0], sda, &x[0], &z[0], n<n1 ? n : n1);
		pA += n1 + n1*bs + (sda-1)*bs;
		x += n1;
		z += n1;
		m -= n1;
		n -= n1;
		}

#if defined(TARGET_X86_AMD_BARCELONA) | defined(TARGET_X86_AMD_JAGUAR)
	// using dgemv_n and dgemv_t kernels
	double beta1 = 1.0;
	double xx[4];
	for(ii=0; ii<n-3; ii+=4)
		{
		// gemv_n
		kernel_dgemv_n_4_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii);
		// 4x4
		z[ii+0] += alpha*(pA[ii*sda+0+bs*(ii+0)]*x[ii+0] + pA[ii*sda+1+bs*(ii+0)]*x[ii+1] + pA[ii*sda+2+bs*(ii+0)]*x[ii+2] + pA[ii*sda+3+bs*(ii+0)]*x[ii+3]);
		z[ii+1] += alpha*(pA[ii*sda+1+bs*(ii+0)]*x[ii+0] + pA[ii*sda+1+bs*(ii+1)]*x[ii+1] + pA[ii*sda+2+bs*(ii+1)]*x[ii+2] + pA[ii*sda+3+bs*(ii+1)]*x[ii+3]);
		z[ii+2] += alpha*(pA[ii*sda+2+bs*(ii+0)]*x[ii+0] + pA[ii*sda+2+bs*(ii+1)]*x[ii+1] + pA[ii*sda+2+bs*(ii+2)]*x[ii+2] + pA[ii*sda+3+bs*(ii+2)]*x[ii+3]);
		z[ii+3] += alpha*(pA[ii*sda+3+bs*(ii+0)]*x[ii+0] + pA[ii*sda+3+bs*(ii+1)]*x[ii+1] + pA[ii*sda+3+bs*(ii+2)]*x[ii+2] + pA[ii*sda+3+bs*(ii+3)]*x[ii+3]);
		// gemv_t
		kernel_dgemv_t_4_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii);
		}
	if(ii<n)
		{
		if(n-ii==1)
			{
			xx[0] = alpha*x[ii+0];
			xx[1] = alpha*x[ii+1];
			xx[2] = alpha*x[ii+2];
			xx[3] = alpha*x[ii+3];
			// gemv_n
			kernel_dgemv_n_4_vs_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii, m-ii);
			// 4x4
			z[ii+0] += pA[ii*sda+0+bs*(ii+0)]*xx[0];
			if(m-ii>1)
				{
				z[ii+0] += pA[ii*sda+1+bs*(ii+0)]*xx[1];
				z[ii+1] += pA[ii*sda+1+bs*(ii+0)]*xx[0];
				}
			if(m-ii>2)
				{
				z[ii+0] += pA[ii*sda+2+bs*(ii+0)]*xx[2];
				z[ii+2] += pA[ii*sda+2+bs*(ii+0)]*xx[0];
				}
			if(m-ii>3)
				{
				z[ii+0] += pA[ii*sda+3+bs*(ii+0)]*xx[3];
				z[ii+3] += pA[ii*sda+3+bs*(ii+0)]*xx[0];
				}
			// gemv_t
			kernel_dgemv_t_4_vs_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii, n-ii);
			ii += 4;
			}
		else if(n-ii==2)
			{
			xx[0] = alpha*x[ii+0];
			xx[1] = alpha*x[ii+1];
			xx[2] = alpha*x[ii+2];
			xx[3] = alpha*x[ii+3];
			// gemv_n
			kernel_dgemv_n_4_vs_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii, m-ii);
			// 4x4
			z[ii+0] += pA[ii*sda+0+bs*(ii+0)]*xx[0];
			if(m-ii>1)
				{
				z[ii+0] += pA[ii*sda+1+bs*(ii+0)]*xx[1];
				z[ii+1] += pA[ii*sda+1+bs*(ii+0)]*xx[0] + pA[ii*sda+1+bs*(ii+1)]*xx[1];
				}
			if(m-ii>2)
				{
				z[ii+0] += pA[ii*sda+2+bs*(ii+0)]*xx[2];
				z[ii+1] += pA[ii*sda+2+bs*(ii+1)]*xx[2];
				z[ii+2] += pA[ii*sda+2+bs*(ii+0)]*xx[0] + pA[ii*sda+2+bs*(ii+1)]*xx[1];
				}
			if(m-ii>3)
				{
				z[ii+0] += pA[ii*sda+3+bs*(ii+0)]*xx[3];
				z[ii+1] += pA[ii*sda+3+bs*(ii+1)]*xx[3];
				z[ii+3] += pA[ii*sda+3+bs*(ii+0)]*xx[0] + pA[ii*sda+3+bs*(ii+1)]*xx[1];
				}
			// gemv_t
			kernel_dgemv_t_4_vs_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii, n-ii);
			ii += 4;
			}
		else // if(n-ii==3)
			{
			xx[0] = alpha*x[ii+0];
			xx[1] = alpha*x[ii+1];
			xx[2] = alpha*x[ii+2];
			xx[3] = alpha*x[ii+3];
			// gemv_n
			kernel_dgemv_n_4_vs_lib4(ii, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii, m-ii);
			// 4x4
			z[ii+0] += pA[ii*sda+0+bs*(ii+0)]*xx[0];
			if(m-ii>1)
				{
				z[ii+0] += pA[ii*sda+1+bs*(ii+0)]*xx[1];
				z[ii+1] += pA[ii*sda+1+bs*(ii+0)]*xx[0] + pA[ii*sda+1+bs*(ii+1)]*xx[1];
				}
			if(m-ii>2)
				{
				z[ii+0] += pA[ii*sda+2+bs*(ii+0)]*xx[2];
				z[ii+1] += pA[ii*sda+2+bs*(ii+1)]*xx[2];
				z[ii+2] += pA[ii*sda+2+bs*(ii+0)]*xx[0] + pA[ii*sda+2+bs*(ii+1)]*xx[1] + pA[ii*sda+2+bs*(ii+2)]*xx[2];
				}
			if(m-ii>3)
				{
				z[ii+0] += pA[ii*sda+3+bs*(ii+0)]*xx[3];
				z[ii+1] += pA[ii*sda+3+bs*(ii+1)]*xx[3];
				z[ii+2] += pA[ii*sda+3+bs*(ii+2)]*xx[3];
				z[ii+3] += pA[ii*sda+3+bs*(ii+0)]*xx[0] + pA[ii*sda+3+bs*(ii+1)]*xx[1] + pA[ii*sda+3+bs*(ii+2)]*xx[2];
				}
			// gemv_t
			kernel_dgemv_t_4_vs_lib4(m-ii-4, &alpha, 0, pA+(ii+4)*sda+ii*bs, sda, x+ii+4, &beta1, z+ii, z+ii, n-ii);
			ii += 4;
			}
		for( ; ii<m-3; ii+=4)
			{
			// gemv_n
			kernel_dgemv_n_4_lib4(n, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii);
			}
		if(ii<m)
			{
			// gemv_n
			kernel_dgemv_n_4_vs_lib4(n, &alpha, pA+ii*sda, x, &beta1, z+ii, z+ii, m-ii);
			}
		}
	return;
#endif

	// main loop
	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_dsymv_l_4_lib4(m-ii, &alpha, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii]);
		}
	// clean up at the end
	if(ii<n)
		{
		kernel_dsymv_l_4_gen_lib4(m-ii, &alpha, 0, &pA[ii*bs+ii*sda], sda, &x[ii], &z[ii], n-ii);
		}
	
	return;
	}



void blasfeo_hp_dsymv_u(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<=0)
		return;

#if defined(BLASFEO_REF_API)
		blasfeo_ref_dsymv_u(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
		return;
#else
		printf("\nblasfeo_dsymv_u: feature not implemented yet\n");
		exit(1);
#endif

	}



// m >= n
static void blasfeo_hp_dtrmv_lnn_mn(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;

	if(m-n>0)
		blasfeo_hp_dgemv_n(m-n, n, 1.0, sA, ai+n, aj, sx, xi, 0.0, sz, zi+n, sz, zi+n);

	double *pA2 = pA;
	double *z2 = z;
	int m2 = n;
	int n2 = 0;
	double *pA3, *x3;

	double alpha = 1.0;
	double beta = 1.0;

	double zt[4] = {0, 0, 0, 0};
	double xt[4] = {0, 0, 0, 0};

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

	if(m2%bs != 0)
		{
		//
		pA3 = pA2 + bs*n2;
		x3 = x + n2;

		// access only valid memory
		for(jj=0; jj<m2%bs; jj++)
			xt[jj] = x3[jj];

		zt[2] = pA3[2+bs*0]*xt[0] + pA3[2+bs*1]*xt[1] + pA3[2+bs*2]*xt[2];
		zt[1] = pA3[1+bs*0]*xt[0] + pA3[1+bs*1]*xt[1];
		zt[0] = pA3[0+bs*0]*xt[0];
		kernel_dgemv_n_4_lib4(n2, &alpha, pA2, x, &beta, zt, zt);
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
		kernel_dgemv_n_4_lib4(n2, &alpha, pA2, x, &beta, z2, z2);
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



void blasfeo_hp_dtrmv_lnn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_lnn_mn(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



// m >= n
static void blasfeo_hp_dtrmv_lnu_mn(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;

	if(m-n>0)
		blasfeo_hp_dgemv_n(m-n, n, 1.0, sA, ai+n, aj, sx, xi, 0.0, sz, zi+n, sz, zi+n);

	double *pA2 = pA;
	double *z2 = z;
	int m2 = n;
	int n2 = 0;
	double *pA3, *x3;

	double alpha = 1.0;
	double beta = 1.0;

	double zt[4];

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
		zt[3] = pA3[3+bs*0]*x3[0] + pA3[3+bs*1]*x3[1] + pA3[3+bs*2]*x3[2] + 1.0*x3[3];
		zt[2] = pA3[2+bs*0]*x3[0] + pA3[2+bs*1]*x3[1] + 1.0*x3[2];
		zt[1] = pA3[1+bs*0]*x3[0] + 1.0*x3[1];
		zt[0] = 1.0*x3[0];
		kernel_dgemv_n_4_lib4(n2, &alpha, pA2, x, &beta, zt, zt);
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
		z2[3] = pA3[3+bs*0]*x3[0] + pA3[3+bs*1]*x3[1] + pA3[3+bs*2]*x3[2] + 1.0*x3[3];
		z2[2] = pA3[2+bs*0]*x3[0] + pA3[2+bs*1]*x3[1] + 1.0*x3[2];
		z2[1] = pA3[1+bs*0]*x3[0] + 1.0*x3[1];
		z2[0] = 1.0*x3[0];
		kernel_dgemv_n_4_lib4(n2, &alpha, pA2, x, &beta, z2, z2);
		}
	if(ai%4!=0)
		{
		if(ai%bs==1)
			{
			zt[2] = pA[2+bs*0]*x[0] + pA[2+bs*1]*x[1] + 1.0*x[2];
			zt[1] = pA[1+bs*0]*x[0] + 1.0*x[1];
			zt[0] = 1.0*x[0];
			jj_end = 4-ai%bs<n ? 4-ai%bs : n;
			for(jj=0; jj<jj_end; jj++)
				z[jj] = zt[jj];
			}
		else if(ai%bs==2)
			{
			zt[1] = pA[1+bs*0]*x[0] + 1.0*x[1];
			zt[0] = 1.0*x[0];
			jj_end = 4-ai%bs<n ? 4-ai%bs : n;
			for(jj=0; jj<jj_end; jj++)
				z[jj] = zt[jj];
			}
		else // if (ai%bs==3)
			{
			z[0] = 1.0*x[0];
			}
		}

	return;

	}



void blasfeo_hp_dtrmv_lnu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_lnu_mn(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



// m >= n
static void blasfeo_hp_dtrmv_ltn_mn(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;

	double xt[4];
	double zt[4];

	double alpha = 1.0;
	double beta = 1.0;

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
			kernel_dgemv_t_4_lib4(m-3-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
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
			kernel_dgemv_t_4_lib4(m-2-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
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
			kernel_dgemv_t_4_lib4(m-1-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
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
		kernel_dgemv_t_4_lib4(m-4-jj, &alpha, 0, pA, sda, x, &beta, zt, z);
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
		kernel_dgemv_t_4_lib4(m-4-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
		for(ll=0; ll<n-jj; ll++)
			z[ll] = zt[ll];
//		pA += bs*4;
//		z += 4;
		}

	return;

	}



void blasfeo_hp_dtrmv_ltn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_ltn_mn(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



// m >= n
static void blasfeo_hp_dtrmv_ltu_mu(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;

	double xt[4];
	double zt[4];

	double alpha = 1.0;
	double beta = 1.0;

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
			zt[0] = 1.0*xt[0] + pA[1+bs*0]*xt[1] + pA[2+bs*0]*xt[2];
			zt[1] = 1.0*xt[1] + pA[2+bs*1]*xt[2];
			zt[2] = 1.0*xt[2];
			pA += bs*sda - 1;
			x += 3;
			kernel_dgemv_t_4_lib4(m-3-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
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
			zt[0] = 1.0*xt[0] + pA[1+bs*0]*xt[1];
			zt[1] = 1.0*xt[1];
			pA += bs*sda - 2;
			x += 2;
			kernel_dgemv_t_4_lib4(m-2-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
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
			zt[0] = 1.0*xt[0];
			pA += bs*sda - 3;
			x += 1;
			kernel_dgemv_t_4_lib4(m-1-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
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
		zt[0] = 1.0*x[0] + pA[1+bs*0]*x[1] + pA[2+bs*0]*x[2] + pA[3+bs*0]*x[3];
		zt[1] = 1.0*x[1] + pA[2+bs*1]*x[2] + pA[3+bs*1]*x[3];
		zt[2] = 1.0*x[2] + pA[3+bs*2]*x[3];
		zt[3] = 1.0*x[3];
		pA += bs*sda;
		x += 4;
		kernel_dgemv_t_4_lib4(m-4-jj, &alpha, 0, pA, sda, x, &beta, zt, z);
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
		zt[0] = 1.0*xt[0] + pA[1+bs*0]*xt[1] + pA[2+bs*0]*xt[2] + pA[3+bs*0]*xt[3];
		zt[1] = 1.0*xt[1] + pA[2+bs*1]*xt[2] + pA[3+bs*1]*xt[3];
		zt[2] = 1.0*xt[2] + pA[3+bs*2]*xt[3];
		zt[3] = 1.0*xt[3];
		pA += bs*sda;
		x += 4;
		kernel_dgemv_t_4_lib4(m-4-jj, &alpha, 0, pA, sda, x, &beta, zt, zt);
		for(ll=0; ll<n-jj; ll++)
			z[ll] = zt[ll];
//		pA += bs*4;
//		z += 4;
		}

	return;

	}



void blasfeo_hp_dtrmv_ltu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_ltu_mu(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



void blasfeo_hp_dtrmv_unn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<=0)
		return;

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrmv_unn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrmv_unn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;

	int i;
	
	i=0;
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; i<m-7; i+=8)
		{
		kernel_dtrmv_un_8_lib4(m-i, pA, sda, x, z);
		pA += 8*sda+8*bs;
		x  += 8;
		z  += 8;
		}
#endif
	for(; i<m-3; i+=4)
		{
		kernel_dtrmv_un_4_lib4(m-i, pA, x, z);
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



void blasfeo_hp_dtrmv_utn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{

	if(m<=0)
		return;

	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrmv_utn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrmv_utn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}

	const int bs = 4;

	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;

	int ii, idx;
	
	double *ptrA;
	
	ii=0;
	idx = m/bs*bs;
	if(m%bs!=0)
		{
		kernel_dtrmv_ut_4_vs_lib4(m, pA+idx*bs, sda, x, z+idx, m%bs);
		ii += m%bs;
		}
	idx -= 4;
	for(; ii<m; ii+=4)
		{
		kernel_dtrmv_ut_4_lib4(idx+4, pA+idx*bs, sda, x, z+idx);
		idx -= 4;
		}

	return;

	}



void blasfeo_hp_dtrsv_lnn_mn(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0 | n==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_lnn_mn : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_dtrsv_lnn_mn : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_lnn_mn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_lnn_mn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_lnn_mn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_lnn_mn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_lnn_mn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_dtrsv_lnn_mn : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_lnn_mn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_lnn_mn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsv_lnn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrsv_lnn_mn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *inv_diag_A = sA->dA;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			ddiaex_lib(n, 1.0, ai, pA, sda, inv_diag_A);
			for(ii=0; ii<n; ii++)
				inv_diag_A[ii] = 1.0 / inv_diag_A[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		ddiaex_lib(n, 1.0, ai, pA, sda, inv_diag_A);
		for(ii=0; ii<n; ii++)
			inv_diag_A[ii] = 1.0 / inv_diag_A[ii];
		sA->use_dA = 0;
		}
	// suppose m>=n
	if(m<n)
		m = n;

	double alpha = -1.0;
	double beta = 1.0;

	int i;

	if(x!=z)
		{
		for(i=0; i<m; i++)
			z[i] = x[i];
		}
	
	i = 0;
	for( ; i<n-3; i+=4)
		{
		kernel_dtrsv_ln_inv_4_lib4(i, &pA[i*sda], &inv_diag_A[i], z, &z[i], &z[i]);
		}
	if(i<n)
		{
		kernel_dtrsv_ln_inv_4_vs_lib4(i, &pA[i*sda], &inv_diag_A[i], z, &z[i], &z[i], m-i, n-i);
		i+=4;
		}
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
	for( ; i<m-7; i+=8)
		{
		kernel_dgemv_n_8_lib4(n, &alpha, &pA[i*sda], sda, z, &beta, &z[i], &z[i]);
		}
	if(i<m-3)
		{
		kernel_dgemv_n_4_lib4(n, &alpha, &pA[i*sda], z, &beta, &z[i], &z[i]);
		i+=4;
		}
#else
	for( ; i<m-3; i+=4)
		{
		kernel_dgemv_n_4_lib4(n, &alpha, &pA[i*sda], z, &beta, &z[i], &z[i]);
		}
#endif
	if(i<m)
		{
		kernel_dgemv_n_4_vs_lib4(n, &alpha, &pA[i*sda], z, &beta, &z[i], &z[i], m-i);
		i+=4;
		}


	return;
	}



void blasfeo_hp_dtrsv_lnn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_lnn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_lnn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_lnn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_lnn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_lnn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_lnn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_dtrsv_lnn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_lnn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_lnn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	blasfeo_hp_dtrsv_lnn_mn(m, m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_hp_dtrsv_lnu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_lnu : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_lnu : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_lnu : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_lnu : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_lnu : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_lnu : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_dtrsv_lnu : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_lnu : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_lnu : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrsv_lnu: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *dA = sA->dA;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	if(x!=z)
		{
		for(ii=0; ii<m; ii++)
			z[ii] = x[ii];
		}
	ii = 0;
	for( ; ii<m-3; ii+=4)
		{
		kernel_dtrsv_ln_one_4_lib4(ii, &pA[ii*sda], z, &z[ii], &z[ii]);
		}
	if(ii<m)
		{
		kernel_dtrsv_ln_one_4_vs_lib4(ii, &pA[ii*sda], z, &z[ii], &z[ii], m-ii, m-ii);
		ii+=4;
		}
	return;
	}



void blasfeo_hp_dtrsv_ltn_mn(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_ltn_mn : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_dtrsv_ltn_mn : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_ltn_mn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_ltn_mn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_ltn_mn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_ltn_mn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_ltn_mn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_dtrsv_ltn_mn : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_ltn_mn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_ltn_mn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsv_ltn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrsv_ltn_mn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *inv_diag_A = sA->dA;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			ddiaex_lib(n, 1.0, ai, pA, sda, inv_diag_A);
			for(ii=0; ii<n; ii++)
				inv_diag_A[ii] = 1.0 / inv_diag_A[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		ddiaex_lib(n, 1.0, ai, pA, sda, inv_diag_A);
		for(ii=0; ii<n; ii++)
			inv_diag_A[ii] = 1.0 / inv_diag_A[ii];
		sA->use_dA = 0;
		}
	if(m<=0 || n<=0)
		return;

	if(n>m)
		n = m;
	
	int i;
	
	if(x!=z)
		for(i=0; i<m; i++)
			z[i] = x[i];
			
	i=0;
	if(n%4==1)
		{
		kernel_dtrsv_lt_inv_1_lib4(m-n+i+1, &pA[n/bs*bs*sda+(n-i-1)*bs], sda, &inv_diag_A[n-i-1], &z[n-i-1], &z[n-i-1], &z[n-i-1]);
		i++;
		}
	else if(n%4==2)
		{
		kernel_dtrsv_lt_inv_2_lib4(m-n+i+2, &pA[n/bs*bs*sda+(n-i-2)*bs], sda, &inv_diag_A[n-i-2], &z[n-i-2], &z[n-i-2], &z[n-i-2]);
		i+=2;
		}
	else if(n%4==3)
		{
		kernel_dtrsv_lt_inv_3_lib4(m-n+i+3, &pA[n/bs*bs*sda+(n-i-3)*bs], sda, &inv_diag_A[n-i-3], &z[n-i-3], &z[n-i-3], &z[n-i-3]);
		i+=3;
		}
	for(; i<n-3; i+=4)
		{
		kernel_dtrsv_lt_inv_4_lib4(m-n+i+4, &pA[(n-i-4)/bs*bs*sda+(n-i-4)*bs], sda, &inv_diag_A[n-i-4], &z[n-i-4], &z[n-i-4], &z[n-i-4]);
		}


	return;
	}



void blasfeo_hp_dtrsv_ltn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_ltn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_ltn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_ltn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_ltn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_ltn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_ltn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_dtrsv_ltn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_ltn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_ltn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrsv_ltn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	blasfeo_hp_dtrsv_ltn_mn(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



void blasfeo_hp_dtrsv_ltu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_ltu : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_ltu : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_ltu : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_ltu : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_ltu : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_ltu : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_dtrsv_ltu : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_ltu : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_ltu : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrsv_ltu: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *dA = sA->dA;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	if(x!=z)
		for(ii=0; ii<m; ii++)
			z[ii] = x[ii];
	ii=0;
	if(m%4==1)
		{
		kernel_dtrsv_lt_one_1_lib4(1, &pA[m/bs*bs*sda+(m-1)*bs], sda, &z[m-1], &z[m-1], &z[m-1]);
		ii++;
		}
	else if(m%4==2)
		{
		kernel_dtrsv_lt_one_2_lib4(2, &pA[m/bs*bs*sda+(m-2)*bs], sda, &z[m-2], &z[m-2], &z[m-2]);
		ii+=2;
		}
	else if(m%4==3)
		{
		kernel_dtrsv_lt_one_3_lib4(3, &pA[m/bs*bs*sda+(m-3)*bs], sda, &z[m-3], &z[m-3], &z[m-3]);
		ii+=3;
		}
	for(; ii<m-3; ii+=4)
		{
		kernel_dtrsv_lt_one_4_lib4(ii+4, &pA[(m-ii-4)/bs*bs*sda+(m-ii-4)*bs], sda, &z[m-ii-4], &z[m-ii-4], &z[m-ii-4]);
		}
	return;
	}



void blasfeo_hp_dtrsv_unn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_unn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_unn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_unn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_unn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_unn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_unn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_dtrsv_unn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_unn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_unn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsv_unn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrsv_unn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *dA = sA->dA;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			ddiaex_lib(m, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		ddiaex_lib(m, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}
	if(x!=z)
		{
		for(ii=0; ii<m; ii++)
			z[ii] = x[ii];
		}
	ii = 0;
	if(m%4==1)
		{
		z[m-ii-1] *= dA[m-ii-1];
		ii+=1;
		}
	else if(m%4==2)
		{
		z[m-ii-1] *= dA[m-ii-1];
		z[m-ii-2] -= pA[m/bs*bs*sda+(m-ii-1)*bs]*z[m-ii-1];
		z[m-ii-2] *= dA[m-ii-2];
		ii+=2;
		}
	else if(m%4==3)
		{
		z[m-ii-1] *= dA[m-ii-1];
		z[m-ii-2] -= pA[1+m/bs*bs*sda+(m-ii-1)*bs]*z[m-ii-1];
		z[m-ii-2] *= dA[m-ii-2];
		z[m-ii-3] -= pA[m/bs*bs*sda+(m-ii-2)*bs]*z[m-ii-2];
		z[m-ii-3] -= pA[m/bs*bs*sda+(m-ii-1)*bs]*z[m-ii-1];
		z[m-ii-3] *= dA[m-ii-3];
		ii+=3;
		}
	for(; ii<m-3; ii+=4)
		{
		// TODO
		kernel_dtrsv_un_inv_4_lib4(ii+4, &pA[(m-ii-4)/bs*bs*sda+(m-ii-4)*bs], &dA[m-ii-4], &z[m-ii-4], &z[m-ii-4], &z[m-ii-4]);
		}
	return;
	}



void blasfeo_hp_dtrsv_utn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_dtrsv_utn : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_dtrsv_utn : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_dtrsv_utn : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** blasfeo_dtrsv_utn : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** blasfeo_dtrsv_utn : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** blasfeo_dtrsv_utn : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** blasfeo_dtrsv_utn : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** blasfeo_dtrsv_utn : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** blasfeo_dtrsv_utn : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	if(ai!=0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_dtrsv_utn(m, sA, ai, aj, sx, xi, sz, zi);
		return;
#else
		printf("\nblasfeo_dtrsv_utn: feature not implemented yet: ai=%d\n", ai);
		exit(1);
#endif
		}
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs; // TODO ai
	double *dA = sA->dA;
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			ddiaex_lib(m, 1.0, ai, pA, sda, dA);
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / dA[ii];
			sA->use_dA = 1;
			}
		}
	else
		{
		ddiaex_lib(m, 1.0, ai, pA, sda, dA);
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / dA[ii];
		sA->use_dA = 0;
		}
	if(x!=z)
		{
		for(ii=0; ii<m; ii++)
			z[ii] = x[ii];
		}
	ii = 0;
	for( ; ii<m-3; ii+=4)
		{
		kernel_dtrsv_ut_inv_4_lib4(ii, &pA[ii*bs], sda, &dA[ii], z, &z[ii], &z[ii]);
		}
	if(ii<m)
		{
		kernel_dtrsv_ut_inv_4_vs_lib4(ii, &pA[ii*bs], sda, &dA[ii], z, &z[ii], &z[ii], m-ii, m-ii);
		ii+=4;
		}
	return;
	}



void blasfeo_hp_dger(int m, int n, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dger(m, n, alpha, sx, xi, sy, yi, sC, ci, cj, sD, di, dj);
	return;
#else
	printf("\nblasfeo_dger: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dgemv_n(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dgemv_n(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_dgemv_t(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dgemv_t(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_dgemv_nt(int m, int n, double alpha_n, double alpha_t, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx_n, int xi_n, struct blasfeo_dvec *sx_t, int xi_t, double beta_n, double beta_t, struct blasfeo_dvec *sy_n, int yi_n, struct blasfeo_dvec *sy_t, int yi_t, struct blasfeo_dvec *sz_n, int zi_n, struct blasfeo_dvec *sz_t, int zi_t)
	{
	blasfeo_hp_dgemv_nt(m, n, alpha_n, alpha_t, sA, ai, aj, sx_n, xi_n, sx_t, xi_t, beta_n, beta_t, sy_n, yi_n, sy_t, yi_t, sz_n, zi_n, sz_t, zi_t);
	}



void blasfeo_dsymv_l(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dsymv_l(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_dsymv_l_mn(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dsymv_l_mn(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_dsymv_u(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dsymv_u(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_dtrmv_lnn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_lnn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrmv_lnu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrmv_ltn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrmv_ltu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrmv_unn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_unn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrmv_utn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrmv_utn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_lnn_mn(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_lnn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_lnn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_lnn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_lnu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_ltn_mn(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_ltn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_ltn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_ltu(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_unn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_unn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dtrsv_utn(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dtrsv_utn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_dger(int m, int n, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dmat *sC, int ci, int cj, struct blasfeo_dmat *sD, int di, int dj)
	{
	blasfeo_hp_dger(m, n, alpha, sx, xi, sy, yi, sC, ci, cj, sD, di, dj);
	}



#endif
