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



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dgemm nn
void REF_GEMM_NN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+1));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dgemm nt
void REF_GEMM_NT(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				c_11 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+1));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+(jj+0), bbj+kk);
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+(jj+1), bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+(jj+0), bbj+kk);
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+(jj+0), bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+(jj+0), bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dgemm tn
void REF_GEMM_TN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+1));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dgemm tt
void REF_GEMM_TT(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+(jj+0), bbj+kk);
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+(jj+0), bbj+kk);
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+(jj+1), bbj+kk);
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+(jj+1), bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+1));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+(jj+0), bbj+kk);
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+(jj+1), bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+1));
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+(jj+0), bbj+kk);
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+(jj+0), bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10 + beta * XMATEL_C(cci+(ii+1), ccj+(jj+0));
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+(jj+0), bbj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00 + beta * XMATEL_C(cci+(ii+0), ccj+(jj+0));
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_left_lower_nottransposed_notunit
void REF_TRSM_LLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA<m)
			{
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = m;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			kk = 0;
			for(; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			d_00 *= dA[ii+0];
			d_01 *= dA[ii+0];
			d_10 -= XMATEL_A(aai+ii+1, aaj+ii) * d_00;
			d_11 -= XMATEL_A(aai+ii+1, aaj+ii) * d_01;
			d_10 *= dA[ii+1];
			d_11 *= dA[ii+1];
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii, bbj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+ii, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			d_00 *= dA[ii+0];
			d_01 *= dA[ii+0];
			XMATEL_D(ddi+ii, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+jj);
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			d_00 *= dA[ii+0];
			d_10 -= XMATEL_A(aai+ii+1, aaj+kk) * d_00;
			d_10 *= dA[ii+1];
			XMATEL_D(ddi+ii+0, ddj+jj) = d_00;
			XMATEL_D(ddi+ii+1, ddj+jj) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii, aaj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			d_00 *= dA[ii+0];
			XMATEL_D(ddi+ii, ddj+jj) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_left_lower_nottransposed_unit
void REF_TRSM_LLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			kk = 0;
			for(; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			d_10 -= XMATEL_A(aai+ii+1, aaj+kk) * d_00;
			d_11 -= XMATEL_A(aai+ii+1, aaj+kk) * d_01;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii, bbj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+ii, aaj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			XMATEL_D(ddi+ii, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+jj);
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			d_10 -= XMATEL_A(aai+ii+1, aaj+kk) * d_00;
			XMATEL_D(ddi+ii+0, ddj+jj) = d_00;
			XMATEL_D(ddi+ii+1, ddj+jj) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+ii, aaj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			XMATEL_D(ddi+ii, ddj+jj) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_lltn
void REF_TRSM_LLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if (sA->use_dA<m)
			{
			// invert diagonal of pA
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0/XMATEL_A(aai+ii, aaj+ii);
			// use only now
			sA->use_dA = m;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+1));
			kk = id+2;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			d_10 *= dA[id+1];
			d_11 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_10;
			d_01 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_11;
			d_00 *= dA[id+0];
			d_01 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+id+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			d_00 *= dA[id+0];
			d_01 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			kk = id+2;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			d_10 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_10;
			d_00 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			d_00 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_lltu
void REF_TRSM_LLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+1));
			kk = id+2;

			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_10;
			d_01 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_11;
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+id+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			kk = id+2;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_10;
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_left_upper_nottransposed_notunit
void REF_TRSM_LUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if (sA->use_dA<m)
			{
			// invert diagonal of pA
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0/XMATEL_A(aai+ii, aaj+ii);
			// use only now
			sA->use_dA = m;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+1));
			kk = id+2;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			d_10 *= dA[id+1];
			d_11 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_10;
			d_01 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_11;
			d_00 *= dA[id+0];
			d_01 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+id+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			d_00 *= dA[id+0];
			d_01 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			kk = id+2;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			d_10 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_10;
			d_00 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			d_00 *= dA[id+0];
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_lunu
void REF_TRSM_LUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+1));
			kk = id+2;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_10;
			d_01 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_11;
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+id+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+1));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+1));
				}
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+0, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			id = m-ii-2;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+id+1, bbj+(jj+0));
			kk = id+2;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_10;
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+id+1, ddj+(jj+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			id = m-ii-1;
			d_00 = alpha * XMATEL_B(bbi+id+0, bbj+(jj+0));
			kk = id+1;
			for(; kk<m; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+kk+0, ddj+(jj+0));
				}
			XMATEL_D(ddi+id+0, ddj+(jj+0)) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_lutn
void REF_TRSM_LUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA<m)
			{
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = m;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			kk = 0;
			for(; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			d_00 *= dA[ii+0];
			d_01 *= dA[ii+0];
			d_10 -= XMATEL_A(aai+ii, aaj+(ii+1)) * d_00;
			d_11 -= XMATEL_A(aai+ii, aaj+(ii+1)) * d_01;
			d_10 *= dA[ii+1];
			d_11 *= dA[ii+1];
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii, bbj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+ii) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk, aaj+ii) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			d_00 *= dA[ii+0];
			d_01 *= dA[ii+0];
			XMATEL_D(ddi+ii, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+jj);
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_D(ddi+kk, ddj+jj);
				}
			d_00 *= dA[ii+0];
			d_10 -= XMATEL_A(aai+kk, aaj+(ii+1)) * d_00;
			d_10 *= dA[ii+1];
			XMATEL_D(ddi+ii+0, ddj+jj) = d_00;
			XMATEL_D(ddi+ii+1, ddj+jj) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+ii) * XMATEL_D(ddi+kk, ddj+jj);
				}
			d_00 *= dA[ii+0];
			XMATEL_D(ddi+ii, ddj+jj) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_lutu
void REF_TRSM_LUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			kk = 0;
			for(; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			d_10 -= XMATEL_A(aai+ii, aaj+(ii+1)) * d_00;
			d_11 -= XMATEL_A(aai+ii, aaj+(ii+1)) * d_01;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(jj+0));
			d_01 = alpha * XMATEL_B(bbi+ii, bbj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+ii) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_A(aai+kk, aaj+ii) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			XMATEL_D(ddi+ii, ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+ii, ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+jj);
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_D(ddi+kk, ddj+jj);
				}
			d_10 -= XMATEL_A(aai+kk, aaj+(ii+1)) * d_00;
			XMATEL_D(ddi+ii+0, ddj+jj) = d_00;
			XMATEL_D(ddi+ii+1, ddj+jj) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_A(aai+kk, aaj+ii) * XMATEL_D(ddi+kk, ddj+jj);
				}
			XMATEL_D(ddi+ii, ddj+jj) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_rlnn
void REF_TRSM_RLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if (sA->use_dA<n)
			{
			// invert diagonal of pA
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0/XMATEL_A(aai+ii, aaj+ii);
			// use only now
			sA->use_dA = n;
			}
		}
	else
		{
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		id = n-jj-2;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_11 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			d_01 *= dA[id+1];
			d_11 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_01;
			d_10 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_11;
			d_00 *= dA[id+0];
			d_10 *= dA[id+0];
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(id+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				}
			d_01 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_01;
			d_00 *= dA[id+0];
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		id = n-jj-1;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			kk = id+1;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			d_00 *= dA[id+0];
			d_10 *= dA[id+0];
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(id));
			kk = id+1;
			for(; kk<n; kk++)
				d_00 -= XMATEL_A(aai+kk, aaj+(id)) * XMATEL_D(ddi+ii, ddj+(kk));
			XMATEL_D(ddi+ii, ddj+(id)) = d_00 * dA[id];
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_rlnu
void REF_TRSM_RLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		id = n-jj-2;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_11 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_01;
			d_10 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_11;
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(id+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+kk+0, aaj+(id+1)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				}
			d_00 -= XMATEL_A(aai+id+1, aaj+(id+0)) * d_01;
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		id = n-jj-1;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			kk = id+1;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+kk+0, aaj+(id+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(id));
			kk = id+1;
			for(; kk<n; kk++)
				d_00 -= XMATEL_A(aai+kk, aaj+(id)) * XMATEL_D(ddi+ii, ddj+(kk));
			XMATEL_D(ddi+ii, ddj+(id)) = d_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_right_lower_transposed_not-unit
void REF_TRSM_RLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA<n)
			{
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = n;
			}
		}
	else
		{
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	REAL
		f_00_inv,
		f_10, f_11_inv,
		c_00, c_01,
		c_10, c_11;
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		f_00_inv = dA[jj+0];
		f_10 = XMATEL_A(aai+jj+1, aaj+(jj+0));
		f_11_inv = dA[jj+1];
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			c_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+0, aaj+kk);
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+jj+0, aaj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+1, aaj+kk);
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+jj+1, aaj+kk);
				}
			c_00 *= f_00_inv;
			c_10 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			c_01 *= f_11_inv;
			c_11 *= f_11_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+0, aaj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+1, aaj+kk);
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			c_01 *= f_11_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			}
		}
	for(; jj<n; jj++)
		{
		// factorize diagonal
		f_00_inv = dA[jj];
		for(ii=0; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_A(aai+jj, aaj+kk);
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii, ddj+jj) = c_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_right_lower_transposed_unit
void REF_TRSM_RLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		f_10,
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		f_10 = XMATEL_A(aai+jj+1, aaj+(jj+0));
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			c_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+0, aaj+kk);
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+jj+0, aaj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+1, aaj+kk);
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+jj+1, aaj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+0, aaj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+jj+1, aaj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			}
		}
	for(; jj<n; jj++)
		{
		for(ii=0; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_A(aai+jj, aaj+kk);
				}
			XMATEL_D(ddi+ii, ddj+jj) = c_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_runn
void REF_TRSM_RUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA<n)
			{
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = n;
			}
		}
	else
		{
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	REAL
		f_00_inv,
		f_10, f_11_inv,
		c_00, c_01,
		c_10, c_11;
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		f_00_inv = dA[jj+0];
		f_10 = XMATEL_A(aai+jj+0, aaj+(jj+1));
		f_11_inv = dA[jj+1];
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			c_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			c_00 *= f_00_inv;
			c_10 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			c_01 *= f_11_inv;
			c_11 *= f_11_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			c_01 *= f_11_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			}
		}
	for(; jj<n; jj++)
		{
		// factorize diagonal
		f_00_inv = dA[jj];
		for(ii=0; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_A(aai+kk, aaj+jj);
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii, ddj+jj) = c_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_runu
void REF_TRSM_RUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL
		f_10,
		c_00, c_01,
		c_10, c_11;
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		f_10 = XMATEL_A(aai+jj+0, aaj+(jj+1));
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			c_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+0));
			c_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01;
			}
		}
	for(; jj<n; jj++)
		{
		// factorize diagonal
		for(ii=0; ii<m; ii++)
			{
			c_00 = alpha * XMATEL_B(bbi+ii, bbj+jj);
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_A(aai+kk, aaj+jj);
				}
			XMATEL_D(ddi+ii, ddj+jj) = c_00;
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_right_upper_transposed_notunit
void REF_TRSM_RUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA;
	if(ai==0 & aj==0)
		{
		if (sA->use_dA<n)
			{
			// invert diagonal of pA
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0/XMATEL_A(aai+ii, aaj+ii);
			// use only now
			sA->use_dA = n;
			}
		}
	else
		{
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0; // nonzero offset makes diagonal dirty
		}
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		id = n-jj-2;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_11 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			d_01 *= dA[id+1];
			d_11 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_01;
			d_10 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_11;
			d_00 *= dA[id+0];
			d_10 *= dA[id+0];
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(id+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				}
			d_01 *= dA[id+1];
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_01;
			d_00 *= dA[id+0];
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		id = n-jj-1;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			kk = id+1;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			d_00 *= dA[id+0];
			d_10 *= dA[id+0];
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(id));
			kk = id+1;
			for(; kk<n; kk++)
				d_00 -= XMATEL_A(aai+id, aaj+(kk)) * XMATEL_D(ddi+ii, ddj+(kk));
			XMATEL_D(ddi+ii, ddj+(id)) = d_00 * dA[id];
			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
// dtrsm_rutu
void REF_TRSM_RUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	int i1 = 1;
	int ii, jj, kk, id;
	REAL
		d_00, d_01,
		d_10, d_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	// solve
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		id = n-jj-2;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			d_11 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_11 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_01;
			d_10 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_11;
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			XMATEL_D(ddi+ii+1, ddj+(id+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_01 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+1));
			kk = id+2;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_01 -= XMATEL_A(aai+id+1, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				}
			d_00 -= XMATEL_A(aai+id+0, aaj+(id+1)) * d_01;
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+0, ddj+(id+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		id = n-jj-1;
		for(; ii<m-1; ii+=2)
			{
			d_00 = alpha * XMATEL_B(bbi+ii+0, bbj+(id+0));
			d_10 = alpha * XMATEL_B(bbi+ii+1, bbj+(id+0));
			kk = id+1;
			for(; kk<n; kk++)
				{
				d_00 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+0, ddj+(kk+0));
				d_10 -= XMATEL_A(aai+id+0, aaj+(kk+0)) * XMATEL_D(ddi+ii+1, ddj+(kk+0));
				}
			XMATEL_D(ddi+ii+0, ddj+(id+0)) = d_00;
			XMATEL_D(ddi+ii+1, ddj+(id+0)) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = alpha * XMATEL_B(bbi+ii, bbj+(id));
			kk = id+1;
			for(; kk<n; kk++)
				d_00 -= XMATEL_A(aai+id, aaj+(kk)) * XMATEL_D(ddi+ii, ddj+(kk));
			XMATEL_D(ddi+ii, ddj+(id)) = d_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_lower_nottransposed_notunit (A triangular !!!)
void REF_TRMM_LLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_11 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_11 += XMATEL_A(aai+(ii+1), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_lower_nottransposed_unit (A triangular !!!)
void REF_TRMM_LLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			c_11 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_11 += XMATEL_B(bbi+kk+1, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_lower_transposed_notunit (A triangular !!!)
void REF_TRMM_LLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+kk+1, aaj+(ii+0)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk+1, aaj+(ii+1)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_01 += XMATEL_A(aai+kk+1, aaj+(ii+0)) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			c_11 += XMATEL_A(aai+kk+1, aaj+(ii+1)) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+kk+1, aaj+(ii+0)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk+1, aaj+(ii+1)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_lower_transposed_unit (A triangular !!!)
void REF_TRMM_LLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+kk+1, aaj+(ii+0)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			c_01 += XMATEL_A(aai+kk+1, aaj+(ii+0)) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			c_11 += XMATEL_B(bbi+kk+1, bbj+(jj+1));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+kk+1, aaj+(ii+0)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_upper_nottransposed_notunit (A triangular !!!)
void REF_TRMM_LUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_01 += XMATEL_A(aai+(ii+0), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			c_11 += XMATEL_A(aai+(ii+1), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_A(aai+(ii+1), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			kk = ii;
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_upper_nottransposed_unit (A triangular !!!)
void REF_TRMM_LUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			c_01 += XMATEL_A(aai+(ii+0), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			c_11 += XMATEL_B(bbi+kk+1, bbj+(jj+1));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_00 += XMATEL_A(aai+(ii+0), aaj+kk+1) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			kk += 2;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+(ii+1), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			kk += 1;
			for(; kk<m; kk++)
				{
				c_00 += XMATEL_A(aai+(ii+0), aaj+kk) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_upper_transposed_notunit (A triangular !!!)
void REF_TRMM_LUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_10 += XMATEL_A(aai+kk+1, aaj+(ii+1)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_11 += XMATEL_A(aai+kk+1, aaj+(ii+1)) * XMATEL_B(bbi+kk+1, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk+1, aaj+(ii+1)) * XMATEL_B(bbi+kk+1, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_left_upper_transposed_unit (A triangular !!!)
void REF_TRMM_LUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			c_11 += XMATEL_B(bbi+kk+1, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_B(bbi+kk, bbj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_B(bbi+kk+1, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			for(kk=0; kk<ii; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			kk = ii;
			c_00 += XMATEL_B(bbi+kk, bbj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_lower_nottransposed_notunit (A triangular !!!)
void REF_TRMM_RLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+0));
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+0));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+1));
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+1));
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+0));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+1));
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_lower_nottransposed_unit (A triangular !!!)
void REF_TRMM_RLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			c_01 = 0.0; ;
			c_11 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+0));
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+0));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1);
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			c_01 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+0));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0; ;
			c_10 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0; ;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_lower_nottransposed_notunit (A triangular !!!)
void REF_TRMM_RLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+1), aaj+kk+1);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+(jj+1), aaj+kk+1);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+1), aaj+kk+1);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_lower_nottransposed_notunit (A triangular !!!)
void REF_TRMM_RLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_lower_nottransposed_notunit (A triangular !!!)
void REF_TRMM_RUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+1));
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+kk+1, aaj+(jj+1));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_lower_nottransposed_notunit (A triangular !!!)
void REF_TRMM_RUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL 
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+1));
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			for(kk=0; kk<jj; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+kk, aaj+(jj+0));
				}
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_upper_transposed_notunit (A triangular !!!)
void REF_TRMM_RUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+0), aaj+kk+1);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+(jj+0), aaj+kk+1);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+1), aaj+kk+1);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+(jj+1), aaj+kk+1);
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+0), aaj+kk+1);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+1), aaj+kk+1);
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}	
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dtrmm_right_upper_transposed_unit (A triangular !!!)
void REF_TRMM_RUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+0), aaj+kk+1);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk+1) * XMATEL_A(aai+(jj+0), aaj+kk+1);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			c_11 += XMATEL_B(bbi+(ii+1), bbj+kk+1);
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				c_11 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk+1) * XMATEL_A(aai+(jj+0), aaj+kk+1);
			c_01 += XMATEL_B(bbi+(ii+0), bbj+kk+1);
			kk += 2;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_01 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+1), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			c_10 += XMATEL_B(bbi+(ii+1), bbj+kk);
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				c_10 += XMATEL_B(bbi+(ii+1), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = alpha * c_10;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			kk = jj;
			c_00 += XMATEL_B(bbi+(ii+0), bbj+kk);
			kk += 1;
			for(; kk<n; kk++)
				{
				c_00 += XMATEL_B(bbi+(ii+0), bbj+kk) * XMATEL_A(aai+(jj+0), aaj+kk);
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = alpha * c_00;
			}
		}	
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyrk_lower not-transposed
void REF_SYRK_LN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// diagonal
		c_00 = 0.0;
		c_10 = 0.0;
		c_11 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			c_10 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			c_11 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
			}
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
		XMATEL_D(ddi+jj+1, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+0)) + alpha * c_10;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+1)) + alpha * c_11;
		// lower
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				c_11 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+0)) + alpha * c_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+1)) + alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			}
		}
	if(jj<m)
		{
		// diagonal
		c_00 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj, aaj+kk) * XMATEL_B(bbi+jj, bbj+kk);
			}
		XMATEL_D(ddi+jj, ddj+jj) = beta * XMATEL_C(cci+jj, ccj+jj) + alpha * c_00;
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyrk_lower not-transposed
void REF_SYRK_LN_MN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		// diagonal
		ii = jj;
		if(ii<m-1)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_11 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+jj+1, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+0)) + alpha * c_10;
			XMATEL_D(ddi+jj+1, ddj+(jj+1)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+1)) + alpha * c_11;
			ii += 2;
			}
		else if(ii<m)
			{
			c_00 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				}
			XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
			ii += 1;
			}
		// lower
//		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				c_11 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+0)) + alpha * c_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+1)) + alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			}
		}
	for(; jj<n; jj++)
		{
		// diagonal
		ii = jj;
		if(ii<m)
			{
			c_00 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+jj, aaj+kk) * XMATEL_B(bbi+jj, bbj+kk);
				}
			XMATEL_D(ddi+jj, ddj+jj) = beta * XMATEL_C(cci+jj, ccj+jj) + alpha * c_00;
			ii += 1;
			}
		// lower
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii, aaj+kk) * XMATEL_B(bbi+jj, bbj+kk);
				}
			XMATEL_D(ddi+ii, ddj+jj) = beta * XMATEL_C(cci+ii, ccj+jj) + alpha * c_00;
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyrk_lower transposed
void REF_SYRK_LT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// diagonal
		c_00 = 0.0;
		c_10 = 0.0;
		c_11 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+kk, aaj+(jj+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_10 += XMATEL_A(aai+kk, aaj+(jj+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_11 += XMATEL_A(aai+kk, aaj+(jj+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			}
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
		XMATEL_D(ddi+jj+1, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+0)) + alpha * c_10;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+1)) + alpha * c_11;
		// lower
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+0)) + alpha * c_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+1)) + alpha * c_11;
			}
		for(; ii<m; ii++)
			{
			c_00 = 0.0;
			c_01 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			}
		}
	if(jj<m)
		{
		// diagonal
		c_00 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+kk, aaj+jj) * XMATEL_B(bbi+kk, bbj+jj);
			}
		XMATEL_D(ddi+jj, ddj+jj) = beta * XMATEL_C(cci+jj, ccj+jj) + alpha * c_00;
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyrk_upper not-transposed
void REF_SYRK_UN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				c_11 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+0)) + alpha * c_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+1)) + alpha * c_11;
			}
		// diagonal
		c_00 = 0.0;
		c_01 = 0.0;
		c_11 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			c_01 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
			c_11 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
			}
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
		XMATEL_D(ddi+jj+0, ddj+(jj+1)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+1)) + alpha * c_01;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+1)) + alpha * c_11;
		}
	if(jj<m)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+0)) + alpha * c_10;
			}
		// diagonal
		c_00 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			}
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyrk_upper transposed
void REF_SYRK_UT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
	REAL
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			c_01 = 0.0;
			c_11 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_01 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				c_11 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+0)) + alpha * c_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+1)) + alpha * c_01;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+1)) + alpha * c_11;
			}
		// diagonal
		c_00 = 0.0;
		c_01 = 0.0;
		c_11 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+kk, aaj+(jj+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			c_01 += XMATEL_A(aai+kk, aaj+(jj+0)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			c_11 += XMATEL_A(aai+kk, aaj+(jj+1)) * XMATEL_B(bbi+kk, bbj+(jj+1));
			}
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
		XMATEL_D(ddi+jj+0, ddj+(jj+1)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+1)) + alpha * c_01;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = beta * XMATEL_C(cci+jj+1, ccj+(jj+1)) + alpha * c_11;
		}
	if(jj<m)
		{
		// upper
		ii = 0;
		for(; ii<jj; ii+=2)
			{
			c_00 = 0.0;
			c_10 = 0.0;
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+kk, aaj+(ii+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				c_10 += XMATEL_A(aai+kk, aaj+(ii+1)) * XMATEL_B(bbi+kk, bbj+(jj+0));
				}
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+0, ccj+(jj+0)) + alpha * c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = beta * XMATEL_C(cci+ii+1, ccj+(jj+0)) + alpha * c_10;
			}
		// diagonal
		c_00 = 0.0;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+kk, aaj+(jj+0)) * XMATEL_B(bbi+kk, bbj+(jj+0));
			}
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = beta * XMATEL_C(cci+jj+0, ccj+(jj+0)) + alpha * c_00;
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyr2k lower not-transposed
void REF_SYR2K_LN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REAL d_1 = 1.0;
	REF_SYRK_LN(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	REF_SYRK_LN(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyr2k lower transposed
void REF_SYR2K_LT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REAL d_1 = 1.0;
	REF_SYRK_LT(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	REF_SYRK_LT(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyr2k upper not-transposed
void REF_SYR2K_UN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REAL d_1 = 1.0;
	REF_SYRK_UN(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	REF_SYRK_UN(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dsyr2k upper transposed
void REF_SYR2K_UT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REAL d_1 = 1.0;
	REF_SYRK_UT(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	REF_SYRK_UT(m, k, alpha, sB, bi, bj, sA, ai, aj, d_1, sD, di, dj, sD, di, dj);
	return;
	}
#endif



#if (defined(LA_REFERENCE) & defined(REF)) | (defined(LA_HIGH_PERFORMANCE) & defined(HP_CM)) | (defined(REF_BLAS))



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void GEMM_NN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_GEMM_NN(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void GEMM_NT(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_GEMM_NT(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void GEMM_TN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_GEMM_TN(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void GEMM_TT(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_GEMM_TT(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LLNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LLNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LLTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LLTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LUNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LUNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LUTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_LUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_LUTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RLNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RLNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RLTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RLTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RUNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RUNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RUTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! defined(HP_CM)
void TRSM_RUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRSM_RUTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LLNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LLNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LLTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LLTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LUNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LUNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LUTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_LUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_LUTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RLNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RLNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RLNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RLNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RLTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RLTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RLTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RLTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RUTN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RUTN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RUTU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RUTU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RUNN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RUNN(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void TRMM_RUNU(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sD, int di, int dj)
	{
	REF_TRMM_RUNU(m, n, alpha, sA, ai, aj, sB, bi, bj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYRK_LN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYRK_LN(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYRK_LN_MN(int m, int n, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYRK_LN_MN(m, n, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYRK_LT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYRK_LT(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYRK_UN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYRK_UN(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYRK_UT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYRK_UT(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYR2K_LN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYR2K_LN(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYR2K_LT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYR2K_LT(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYR2K_UN(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYR2K_UN(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void SYR2K_UT(int m, int k, REAL alpha, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, REAL beta, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_SYR2K_UT(m, k, alpha, sA, ai, aj, sB, bi, bj, beta, sC, ci, cj, sD, di, dj);
	}
#endif

#endif
