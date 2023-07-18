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



void GEMV_N(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	COPY(&m, y, &i1, z, &i1);
	GEMV(&cn, &m, &n, &alpha, pA, &lda, x, &i1, &beta, z, &i1);
	return;
	}



void GEMV_T(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	COPY(&n, y, &i1, z, &i1);
	GEMV(&ct, &m, &n, &alpha, pA, &lda, x, &i1, &beta, z, &i1);
	return;
	}



void GEMV_NT(int m, int n, REAL alpha_n, REAL alpha_t, struct XMAT *sA, int ai, int aj, struct XVEC *sx_n, int xi_n, struct XVEC *sx_t, int xi_t, REAL beta_n, REAL beta_t, struct XVEC *sy_n, int yi_n, struct XVEC *sy_t, int yi_t, struct XVEC *sz_n, int zi_n, struct XVEC *sz_t, int zi_t)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x_n = sx_n->pa + xi_n;
	REAL *x_t = sx_t->pa + xi_t;
	REAL *y_n = sy_n->pa + yi_n;
	REAL *y_t = sy_t->pa + yi_t;
	REAL *z_n = sz_n->pa + zi_n;
	REAL *z_t = sz_t->pa + zi_t;
	COPY(&m, y_n, &i1, z_n, &i1);
	GEMV(&cn, &m, &n, &alpha_n, pA, &lda, x_n, &i1, &beta_n, z_n, &i1);
	COPY(&n, y_t, &i1, z_t, &i1);
	GEMV(&ct, &m, &n, &alpha_t, pA, &lda, x_t, &i1, &beta_t, z_t, &i1);
	return;
	}



void SYMV_L(int m, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	COPY(&m, y, &i1, z, &i1);
	SYMV(&cl, &m, &alpha, pA, &lda, x, &i1, &beta, z, &i1);
	return;
	}



void SYMV_L_MN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	int tmp = m-n;
	COPY(&m, y, &i1, z, &i1);
	SYMV(&cl, &n, &alpha, pA, &lda, x, &i1, &beta, z, &i1);
	GEMV(&cn, &tmp, &n, &alpha, pA+n, &lda, x, &i1, &beta, z+n, &i1);
	GEMV(&ct, &tmp, &n, &alpha, pA+n, &lda, x+n, &i1, &d1, z, &i1);
	return;
	}



void TRMV_LNN(int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL d0 = 0.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(x!=z)
		COPY(&n, x, &i1, z, &i1);
	TRMV(&cl, &cn, &cn, &n, pA, &lda, z, &i1);
	return;
	}



void TRMV_LNN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL d0 = 0.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	int tmp = m-n;
	if(x!=z)
		COPY(&n, x, &i1, z, &i1);
	GEMV(&cn, &tmp, &n, &d1, pA+n, &lda, x, &i1, &d0, z+n, &i1);
	TRMV(&cl, &cn, &cn, &n, pA, &lda, z, &i1);
	return;
	}



void TRMV_LTN(int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(x!=z)
		COPY(&n, x, &i1, z, &i1);
	TRMV(&cl, &ct, &cn, &n, pA, &lda, z, &i1);
	return;
	}



void TRMV_LTN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	int tmp = m-n;
	if(x!=z)
		COPY(&n, x, &i1, z, &i1);
	TRMV(&cl, &ct, &cn, &n, pA, &lda, z, &i1);
	GEMV(&ct, &tmp, &n, &d1, pA+n, &lda, x+n, &i1, &d1, z, &i1);
	return;
	}



void TRMV_UNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRMV(&cu, &cn, &cn, &m, pA, &lda, z, &i1);
	return;
	}



void TRMV_UTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRMV(&cu, &ct, &cn, &m, pA, &lda, z, &i1);
	return;
	}



void TRSV_LNN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0 | n==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_lnn_mn_libstr : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** trsv_lnn_mn_libstr : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_lnn_mn_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_lnn_mn_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_lnn_mn_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_lnn_mn_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_lnn_mn_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** trsv_lnn_mn_libstr : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_lnn_mn_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_lnn_mn_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int mmn = m-n;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRSV(&cl, &cn, &cn, &n, pA, &lda, z, &i1);
	GEMV(&cn, &mmn, &n, &dm1, pA+n, &lda, z, &i1, &d1, z+n, &i1);
	return;
	}



void TRSV_LTN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_ltn_mn_libstr : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** trsv_ltn_mn_libstr : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_ltn_mn_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_ltn_mn_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_ltn_mn_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_ltn_mn_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_ltn_mn_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** trsv_ltn_mn_libstr : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_ltn_mn_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_ltn_mn_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int mmn = m-n;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	GEMV(&ct, &mmn, &n, &dm1, pA+n, &lda, z+n, &i1, &d1, z, &i1);
	TRSV(&cl, &ct, &cn, &n, pA, &lda, z, &i1);
	return;
	}



void TRSV_LNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_lnn_libstr : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_lnn_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_lnn_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_lnn_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_lnn_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_lnn_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** trsv_lnn_libstr : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_lnn_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_lnn_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRSV(&cl, &cn, &cn, &m, pA, &lda, z, &i1);
	return;
	}



void TRSV_LNU(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_lnu_libstr : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_lnu_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_lnu_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_lnu_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_lnu_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_lnu_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** trsv_lnu_libstr : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_lnu_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_lnu_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRSV(&cl, &cn, &cu, &m, pA, &lda, z, &i1);
	return;
	}



void TRSV_LTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_ltn_libstr : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_ltn_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_ltn_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_ltn_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_ltn_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_ltn_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** trsv_ltn_libstr : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_ltn_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_ltn_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRSV(&cl, &ct, &cn, &m, pA, &lda, z, &i1);
	return;
	}



void TRSV_LTU(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_ltu_libstr : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_ltu_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_ltu_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_ltu_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_ltu_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_ltu_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** trsv_ltu_libstr : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_ltu_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_ltu_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRSV(&cl, &ct, &cu, &m, pA, &lda, z, &i1);
	return;
	}



void TRSV_UNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_unn_libstr : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_unn_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_unn_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_unn_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_unn_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_unn_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** trsv_unn_libstr : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_unn_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_unn_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRSV(&cu, &cn, &cn, &m, pA, &lda, z, &i1);
	return;
	}



void TRSV_UTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	if(m==0)
		return;
#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** trsv_utn_libstr : m<0 : %d<0 *****\n", m);
	// non-negative offset
	if(ai<0) printf("\n****** trsv_utn_libstr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** trsv_utn_libstr : aj<0 : %d<0 *****\n", aj);
	if(xi<0) printf("\n****** trsv_utn_libstr : xi<0 : %d<0 *****\n", xi);
	if(zi<0) printf("\n****** trsv_utn_libstr : zi<0 : %d<0 *****\n", zi);
	// inside matrix
	// A: m x k
	if(ai+m > sA->m) printf("\n***** trsv_utn_libstr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+m > sA->n) printf("\n***** trsv_utn_libstr : aj+m > col(A) : %d+%d > %d *****\n", aj, m, sA->n);
	// x: m
	if(xi+m > sx->m) printf("\n***** trsv_utn_libstr : xi+m > size(x) : %d+%d > %d *****\n", xi, m, sx->m);
	// z: m
	if(zi+m > sz->m) printf("\n***** trsv_utn_libstr : zi+m > size(z) : %d+%d > %d *****\n", zi, m, sz->m);
#endif
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	REAL d1 = 1.0;
	REAL dm1 = -1.0;
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	COPY(&m, x, &i1, z, &i1);
	TRSV(&cu, &ct, &cn, &m, pA, &lda, z, &i1);
	return;
	}



void GER(int m, int n, REAL alpha, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int ldc = sC->m;
	int ldd = sD->m;
	int i1 = 1;
	int tmp;
	int jj;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			{
			tmp = m-jj;
			COPY(&tmp, pC+jj*ldc+jj, &i1, pD+jj*ldd+jj, &i1);
			}
		}
	GER_(&m, &n, &alpha, x, &i1, y, &i1, pD, &ldd);
	return;
	}



#else

#error : wrong LA choice

#endif


