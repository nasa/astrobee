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
#include <blasfeo_s_blas.h>
#include <blasfeo_s_aux.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_s_blasfeo_ref_api.h>
#endif



void blasfeo_hp_sgemv_n(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgemv_n(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
#else
	printf("\nblasfeo_sgemv_n: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_sgemv_t(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgemv_t(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
#else
	printf("\nblasfeo_sgemv_t: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_sgemv_nt(int m, int n, float alpha_n, float alpha_t, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx_n, int xi_n, struct blasfeo_svec *sx_t, int xi_t, float beta_n, float beta_t, struct blasfeo_svec *sy_n, int yi_n, struct blasfeo_svec *sy_t, int yi_t, struct blasfeo_svec *sz_n, int zi_n, struct blasfeo_svec *sz_t, int zi_t)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgemv_nt(m, n, alpha_n, alpha_t, sA, ai, aj, sx_n, xi_n, sx_t, xi_t, beta_n, beta_t, sy_n, yi_n, sy_t, yi_t, sz_n, zi_n, sz_t, zi_t);
#else
	printf("\nblasfeo_sgemv_nt: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_ssymv_l(int m, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssymv_l(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
#else
	printf("\nblasfeo_ssymv_l: feature not implemented yet\n");
	exit(1);
#endif
	}


// m >= n
void blasfeo_hp_ssymv_l_mn(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_ssymv_l_mn(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
#else
	printf("\nblasfeo_ssymv_l_mn: feature not implemented yet\n");
	exit(1);
#endif
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
void blasfeo_hp_strmv_lnn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strmv_lnn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strmv_lnn: feature not implemented yet\n");
	exit(1);
#endif
	}



// m >= n
void blasfeo_hp_strmv_lnu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strmv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strmv_lnu: feature not implemented yet\n");
	exit(1);
#endif
	}



// m >= n
void blasfeo_hp_strmv_ltn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strmv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strmv_ltn: feature not implemented yet\n");
	exit(1);
#endif
	}



// m >= n
void blasfeo_hp_strmv_ltu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strmv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strmv_ltu: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strmv_unn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strmv_unn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strmv_unn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strmv_utn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strmv_utn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strmv_utn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_lnn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_lnn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_lnn_mn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_lnn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_lnn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_lnn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_lnu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_lnu: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_ltn_mn(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_ltn_mn(m, n, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_ltn_mn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_ltn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_ltn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_ltu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_ltu: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_unn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_unn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_unn: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_strsv_utn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsv_utn(m, sA, ai, aj, sx, xi, sz, zi);
#else
	printf("\nblasfeo_strsv_utn: feature not implemented yet\n");
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



void blasfeo_strmv_lnu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_lnu(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strmv_ltn(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_ltn(m, sA, ai, aj, sx, xi, sz, zi);
	}



void blasfeo_strmv_ltu(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_strmv_ltu(m, sA, ai, aj, sx, xi, sz, zi);
	}



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


