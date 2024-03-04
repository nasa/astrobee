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
#include <blasfeo_s_blasfeo_api.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_s_blasfeo_ref_api.h>
#endif



// spotrf
void blasfeo_hp_spotrf_l(int m, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_spotrf_l(m, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_spotrf_l: feature not implemented yet\n");
	exit(1);
#endif
	}



// spotrf
void blasfeo_hp_spotrf_l_mn(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_spotrf_l_mn(m, n, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_spotrf_l_mn: feature not implemented yet\n");
	exit(1);
#endif
	}



// spotrf
void blasfeo_hp_spotrf_u(int m, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_spotrf_u(m, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_spotrf_u: feature not implemented yet\n");
	exit(1);
#endif
	
	}



// dsyrk spotrf
void blasfeo_hp_ssyrk_spotrf_ln_mn(int m, int n, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssyrk_spotrf_ln_mn(m, n, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_ssyrk_spotrf_ln_mn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_ssyrk_spotrf_ln(int m, int k, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssyrk_spotrf_ln(m, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_ssyrk_spotrf_ln: feature not implemented yet\n");
	exit(1);
#endif
	}



// dgetrf no pivoting
void blasfeo_hp_sgetrf_np(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgetrf_np(m, n, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_sgetf_np: feature not implemented yet\n");
	exit(1);
#endif
	}



// dgetrf row pivoting
void blasfeo_hp_sgetrf_rp(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, int *ipiv)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgetrf_rp(m, n, sC, ci, cj, sD, di, dj, ipiv);
#else
	printf("\nblasfeo_sgetrf_rp: feature not implemented yet\n");
	exit(1);
#endif
	}



int blasfeo_hp_sgeqrf_worksize(int m, int n)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgeqrf_worksize(m, n);
#else
	printf("\nblasfeo_sgeqrf_worksize: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_sgeqrf(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgeqrf(m, n, sC, ci, cj, sD, di, dj, work);
#else
	printf("\nblasfeo_sgeqrf: feature not implemented yet\n");
	exit(1);
#endif
	}



int blasfeo_hp_sgelqf_worksize(int m, int n)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf_worksize(m, n);
#else
	printf("\nblasfeo_sgelqf_worksize: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_sgelqf(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf(m, n, sC, ci, cj, sD, di, dj, work);
#else
	printf("\nblasfeo_sgelqf: feature not implemented yet\n");
	exit(1);
#endif
	}



int blasfeo_hp_sorglq_worksize(int m, int n, int k)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sorglq_worksize(m, n, k);
#else
	printf("\nblasfeo_sorglq_worksize: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_sorglq(int m, int n, int k, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sorglq(m, n, k, sC, ci, cj, sD, di, dj, work);
#else
	printf("\nblasfeo_sorglq: feature not implemented yet\n");
	exit(1);
#endif
	}



// LQ factorization with positive diagonal elements
void blasfeo_hp_sgelqf_pd(int m, int n, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf_pd(m, n, sC, ci, cj, sD, di, dj, work);
#else
	printf("\nblasfeo_sgelqf_pd: feature not implemented yet\n");
	exit(1);
#endif
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, A] <= lq( [L. A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void blasfeo_hp_sgelqf_pd_la(int m, int n1, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_smat *sA, int ai, int aj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf_pd_la(m, n1, sD, di, dj, sA, ai, aj, work);
#else
	printf("\nblasfeo_sgelqf_pd_la: feature not implemented yet\n");
	exit(1);
#endif
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, L, A] <= lq( [L. L, A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void blasfeo_hp_sgelqf_pd_lla(int m, int n1, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_smat *sL, int li, int lj, struct blasfeo_smat *sA, int ai, int aj, void *work)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgelqf_pd_lla(m, n1, sD, di, dj, sL, li, lj, sA, ai, aj, work);
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


