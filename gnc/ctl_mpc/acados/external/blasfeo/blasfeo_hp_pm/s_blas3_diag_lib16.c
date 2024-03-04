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
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgemm_dn(m, n, alpha, sA, ai, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_sgemm_dn: feature not implemented yet\n");
	exit(1);
#endif
	}



// dgemm with B diagonal matrix (stored as strvec)
void blasfeo_hp_sgemm_nd(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sB, int bi, float beta, struct blasfeo_smat *sC, int ci, int cj, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgemm_nd(m, n, alpha, sA, ai, aj, sB, bi, beta, sC, ci, cj, sD, di, dj);
#else
	printf("\nblasfeo_sgemm_nd: feature not implemented yet\n");
	exit(1);
#endif
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


