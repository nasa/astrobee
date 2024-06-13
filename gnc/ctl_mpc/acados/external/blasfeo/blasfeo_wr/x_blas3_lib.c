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



#if defined(LA_EXTERNAL_BLAS_WRAPPER)



// dgemm nn
void GEMM_NN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cn = 'n';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	GEMM(&cn, &cn, &m, &n, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



// dgemm nt
void GEMM_NT(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cn = 'n';
	char ct = 't';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	GEMM(&cn, &ct, &m, &n, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



// dgemm tn
void GEMM_TN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cn = 'n';
	char ct = 't';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	GEMM(&ct, &cn, &m, &n, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



// dgemm tt
void GEMM_TT(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cn = 'n';
	char ct = 't';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	GEMM(&ct, &ct, &m, &n, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



// dtrsm_left_lower_nottransposed_notunit
void TRSM_LLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*sD->m, &i1);
		}
	TRSM(&cl, &cl, &cn, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_left_lower_nottransposed_unit
void TRSM_LLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*sD->m, &i1);
		}
	TRSM(&cl, &cl, &cn, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_left_lower_transposed_notunit
void TRSM_LLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char ct = 't';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*sD->m, &i1);
		}
	TRSM(&cl, &cl, &ct, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_left_lower_transposed_unit
void TRSM_LLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*sD->m, &i1);
		}
	TRSM(&cl, &cl, &ct, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_left_upper_nottransposed_notunit
void TRSM_LUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cl, &cu, &cn, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_left_upper_nottransposed_unit
void TRSM_LUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cl, &cu, &cn, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_left_upper_transposed_notunit
void TRSM_LUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cl, &cu, &ct, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_left_upper_transposed_unit
void TRSM_LUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cl, &cu, &ct, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_lower_nottransposed_notunit
void TRSM_RLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cl, &cn, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_lower_nottransposed_unit
void TRSM_RLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cl, &cn, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_lower_transposed_notunit
void TRSM_RLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cl, &ct, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_lower_transposed_unit
void TRSM_RLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cl, &ct, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_upper_nottransposed_notunit
void TRSM_RUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cu, &cn, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_upper_nottransposed_unit
void TRSM_RUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cu, &cn, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_upper_transposed_notunit
void TRSM_RUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cu, &ct, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrsm_right_upper_transposed_unit
void TRSM_RUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRSM(&cr, &cu, &ct, &cu, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrmm_right_upper_transposed_notunit (A triangular !!!)
void TRMM_RUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRMM(&cr, &cu, &ct, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dtrmm_right_lower_nottransposed_notunit (A triangular !!!)
void TRMM_RLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA+ai+aj*sA->m;
	REAL *pB = sB->pA+bi+bj*sB->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	if(!(pB==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pB+jj*ldb, &i1, pD+jj*ldd, &i1);
		}
	TRMM(&cr, &cl, &cn, &cn, &m, &n, &alpha, pA, &lda, pD, &ldd);
	return;
	}



// dsyrk lower not-transposed (allowing for different factors => use dgemm !!!)
void SYRK_LN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	if(pA==pB)
		{
		SYRK(&cl, &cn, &m, &k, &alpha, pA, &lda, &beta, pD, &ldd);
		}
	else
		{
		GEMM(&cn, &ct, &m, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
		}
	return;
	}



// dsyrk lower not-transposed (allowing for different factors => use dgemm !!!)
void SYRK_LN_MN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int mmn = m-n;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	if(pA==pB)
		{
		SYRK(&cl, &cn, &n, &k, &alpha, pA, &lda, &beta, pD, &ldd);
		GEMM(&cn, &ct, &mmn, &n, &k, &alpha, pA+n, &lda, pB, &ldb, &beta, pD+n, &ldd);
		}
	else
		{
		GEMM(&cn, &ct, &m, &n, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
		}
	return;
	}



// dsyrk lower transposed (allowing for different factors => use dgemm !!!)
void SYRK_LT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	if(pA==pB)
		{
		SYRK(&cl, &ct, &m, &k, &alpha, pA, &lda, &beta, pD, &ldd);
		}
	else
		{
		GEMM(&ct, &cn, &m, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
		}
	return;
	}



// dsyrk upper not-transposed (allowing for different factors => use dgemm !!!)
void SYRK_UN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	if(pA==pB)
		{
		SYRK(&cu, &cn, &m, &k, &alpha, pA, &lda, &beta, pD, &ldd);
		}
	else
		{
		GEMM(&cn, &ct, &m, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
		}
	return;
	}



// dsyrk upper transposed (allowing for different factors => use dgemm !!!)
void SYRK_UT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	if(pA==pB)
		{
		SYRK(&cu, &ct, &m, &k, &alpha, pA, &lda, &beta, pD, &ldd);
		}
	else
		{
		GEMM(&ct, &cn, &m, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
		}
	return;
	}



// dsyr2k lower not-transposed
void SYR2K_LN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	SYR2K(&cl, &cn, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



// dsyr2k lower transposed
void SYR2K_LT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	SYR2K(&cl, &ct, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



// dsyr2k lower not-transposed
void SYR2K_UN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	SYR2K(&cu, &cn, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



// dsyr2k lower transposed
void SYR2K_UT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(beta==0.0 || pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*sC->m, &i1, pD+jj*sD->m, &i1);
		}
	SYR2K(&cu, &ct, &m, &k, &alpha, pA, &lda, pB, &ldb, &beta, pD, &ldd);
	return;
	}



#else

#error : wrong LA choice

#endif
