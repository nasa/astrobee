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
#if ! ( defined(HP_CM) )
// dpotrf_l
void REF_POTRF_L(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	int ii, jj, kk;
	REAL
		f_00_inv,
		f_10, f_11_inv,
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA;
	if(di==0 & dj==0)
		// if zero offset potrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj+0, ccj+(jj+0));;
		c_10 = XMATEL_C(cci+jj+1, ccj+(jj+0));;
		c_11 = XMATEL_C(cci+jj+1, ccj+(jj+1));;
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_10 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_11 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj+0] = f_00_inv;
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = c_00 * f_00_inv;
		f_10 = c_10 * f_00_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+0)) = f_10;
		c_11 -= f_10 * f_10;
		if(c_11>0)
			{
			f_11_inv = 1.0/SQRT(c_11);
			}
		else
			{
			f_11_inv = 0.0;
			}
		dD[jj+1] = f_11_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = c_11 * f_11_inv;
		// solve lower
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_10 = XMATEL_C(cci+ii+1, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			c_11 = XMATEL_C(cci+ii+1, ccj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			c_10 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11 * f_11_inv;
			}
		for(; ii<m; ii++)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			}
		}
	for(; jj<m; jj++)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj, ccj+jj);;
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj] = f_00_inv;
		XMATEL_D(ddi+jj, ddj+jj) = c_00 * f_00_inv;
		// solve lower
//		for(ii=jj+1; ii<m; ii++)
//			{
//			c_00 = XMATEL_C(cci+ii, ccj+jj);
//			for(kk=0; kk<jj; kk++)
//				{
//				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
//				}
//			XMATEL_D(ddi+ii, ddj+jj) = c_00 * f_00_inv;
//			}
		}
	return;
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! ( defined(HP_CM) )
#if ! ( defined(REF_BLAS) )
// dpotrf
void REF_POTRF_L_MN(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;
	if(n>m)
		n = m;
	int ii, jj, kk;
	REAL
		f_00_inv, 
		f_10, f_11_inv,
		c_00, c_01,
		c_10, c_11;
#if defined(MF_COLMAJ)
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA;

	if(di==0 & dj==0)
		// if zero offset potrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;

	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj+0, ccj+(jj+0));;
		c_10 = XMATEL_C(cci+jj+1, ccj+(jj+0));;
		c_11 = XMATEL_C(cci+jj+1, ccj+(jj+1));;
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_10 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_11 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj+0] = f_00_inv;
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = c_00 * f_00_inv;
		f_10 = c_10 * f_00_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+0)) = f_10;
		c_11 -= f_10 * f_10;
		if(c_11>0)
			{
			f_11_inv = 1.0/SQRT(c_11);
			}
		else
			{
			f_11_inv = 0.0;
			}
		dD[jj+1] = f_11_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = c_11 * f_11_inv;
		// solve lower
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_10 = XMATEL_C(cci+ii+1, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			c_11 = XMATEL_C(cci+ii+1, ccj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			c_10 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11 * f_11_inv;
			}
		for(; ii<m; ii++)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			}
		}
	for(; jj<n; jj++)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj, ccj+jj);;
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj] = f_00_inv;
		XMATEL_D(ddi+jj, ddj+jj) = c_00 * f_00_inv;
		// solve lower
		for(ii=jj+1; ii<m; ii++)
			{
			c_00 = XMATEL_C(cci+ii, ccj+jj);
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
				}
			XMATEL_D(ddi+ii, ddj+jj) = c_00 * f_00_inv;
			}
		}
	return;
	}
#endif
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! ( defined(HP_CM) )
// dpotrf_u
void REF_POTRF_U(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	int ii, jj, kk;
	REAL
		f_00_inv,
		f_10, f_11_inv,
		c_00, c_01,
		c_10, c_11;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA;
	if(di==0 & dj==0)
		// if zero offset potrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;
	jj = 0;
	for(; jj<m; jj++)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj, ccj+jj);;
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+kk, ddj+jj) * XMATEL_D(ddi+kk, ddj+jj);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj] = f_00_inv;
		XMATEL_D(ddi+jj, ddj+jj) = c_00 * f_00_inv;
		// solve lower
		for(ii=jj+1; ii<m; ii++)
			{
			c_00 = XMATEL_C(cci+jj, ccj+ii);
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+kk, ddj+ii) * XMATEL_D(ddi+kk, ddj+jj);
				}
			XMATEL_D(ddi+jj, ddj+ii) = c_00 * f_00_inv;
			}
		}
	return;
	}
#endif



// dsyrk dpotrf
#if ! ( defined(REF_BLAS) )
void REF_SYRK_POTRF_LN(int m, int k, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	int ii, jj, kk;
	REAL
		f_00_inv, 
		f_10, f_11_inv,
		c_00, c_01,
		c_10, c_11;
#if defined(MF_COLMAJ)
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
	REAL *dD = sD->dA;
	if(di==0 & dj==0)
		// if zero offset potrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj+0, ccj+(jj+0));;
		c_10 = XMATEL_C(cci+jj+1, ccj+(jj+0));;
		c_11 = XMATEL_C(cci+jj+1, ccj+(jj+1));;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			c_10 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			c_11 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
			}
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_10 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_11 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj+0] = f_00_inv;
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = c_00 * f_00_inv;
		f_10 = c_10 * f_00_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+0)) = f_10;
		c_11 -= f_10 * f_10;
		if(c_11>0)
			{
			f_11_inv = 1.0/SQRT(c_11);
			}
		else
			{
			f_11_inv = 0.0;
			}
		dD[jj+1] = f_11_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = c_11 * f_11_inv;
		// solve lower
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_10 = XMATEL_C(cci+ii+1, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			c_11 = XMATEL_C(cci+ii+1, ccj+(jj+1));
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				c_11 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			c_10 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11 * f_11_inv;
			}
		for(; ii<m; ii++)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			}
		}
	for(; jj<m; jj++)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj, ccj+jj);;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj, aaj+kk) * XMATEL_B(bbi+jj, bbj+kk);
			}
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj] = f_00_inv;
		XMATEL_D(ddi+jj, ddj+jj) = c_00 * f_00_inv;
		// solve lower
//		for(ii=jj+1; ii<m; ii++)
//			{
//			c_00 = XMATEL_C(cci+ii, ccj+jj);
//			for(kk=0; kk<k; kk++)
//				{
//				c_00 += XMATEL_A(aai+ii, aaj+kk) * XMATEL_B(bbi+jj, bbj+kk);
//				}
//			for(kk=0; kk<jj; kk++)
//				{
//				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
//				}
//			XMATEL_D(ddi+ii, ddj+jj) = c_00 * f_00_inv;
//			}
		}
	return;
	}
#endif



#if ! ( defined(REF_BLAS) )
void REF_SYRK_POTRF_LN_MN(int m, int n, int k, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	int ii, jj, kk;
	REAL
		f_00_inv, 
		f_10, f_11_inv,
		c_00, c_01,
		c_10, c_11;
#if defined(MF_COLMAJ)
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
	REAL *dD = sD->dA;
	if(di==0 & dj==0)
		// if zero offset potrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj+0, ccj+(jj+0));;
		c_10 = XMATEL_C(cci+jj+1, ccj+(jj+0));;
		c_11 = XMATEL_C(cci+jj+1, ccj+(jj+1));;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			c_10 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
			c_11 += XMATEL_A(aai+jj+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
			}
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_10 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
			c_11 -= XMATEL_D(ddi+jj+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj+0] = f_00_inv;
		XMATEL_D(ddi+jj+0, ddj+(jj+0)) = c_00 * f_00_inv;
		f_10 = c_10 * f_00_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+0)) = f_10;
		c_11 -= f_10 * f_10;
		if(c_11>0)
			{
			f_11_inv = 1.0/SQRT(c_11);
			}
		else
			{
			f_11_inv = 0.0;
			}
		dD[jj+1] = f_11_inv;
		XMATEL_D(ddi+jj+1, ddj+(jj+1)) = c_11 * f_11_inv;
		// solve lower
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_10 = XMATEL_C(cci+ii+1, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			c_11 = XMATEL_C(cci+ii+1, ccj+(jj+1));
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_10 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				c_11 += XMATEL_A(aai+ii+1, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_10 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				c_11 -= XMATEL_D(ddi+ii+1, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			c_10 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			XMATEL_D(ddi+ii+1, ddj+(jj+0)) = c_10;
			c_01 -= c_00 * f_10;
			c_11 -= c_10 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			XMATEL_D(ddi+ii+1, ddj+(jj+1)) = c_11 * f_11_inv;
			}
		for(; ii<m; ii++)
			{
			c_00 = XMATEL_C(cci+ii+0, ccj+(jj+0));
			c_01 = XMATEL_C(cci+ii+0, ccj+(jj+1));
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+0, bbj+kk);
				c_01 += XMATEL_A(aai+ii+0, aaj+kk) * XMATEL_B(bbi+jj+1, bbj+kk);
				}
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+0, ddj+kk);
				c_01 -= XMATEL_D(ddi+ii+0, ddj+kk) * XMATEL_D(ddi+jj+1, ddj+kk);
				}
			c_00 *= f_00_inv;
			XMATEL_D(ddi+ii+0, ddj+(jj+0)) = c_00;
			c_01 -= c_00 * f_10;
			XMATEL_D(ddi+ii+0, ddj+(jj+1)) = c_01 * f_11_inv;
			}
		}
	for(; jj<n; jj++)
		{
		// factorize diagonal
		c_00 = XMATEL_C(cci+jj, ccj+jj);;
		for(kk=0; kk<k; kk++)
			{
			c_00 += XMATEL_A(aai+jj, aaj+kk) * XMATEL_B(bbi+jj, bbj+kk);
			}
		for(kk=0; kk<jj; kk++)
			{
			c_00 -= XMATEL_D(ddi+jj, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
			}
		if(c_00>0)
			{
			f_00_inv = 1.0/SQRT(c_00);
			}
		else
			{
			f_00_inv = 0.0;
			}
		dD[jj] = f_00_inv;
		XMATEL_D(ddi+jj, ddj+jj) = c_00 * f_00_inv;
		// solve lower
		for(ii=jj+1; ii<m; ii++)
			{
			c_00 = XMATEL_C(cci+ii, ccj+jj);
			for(kk=0; kk<k; kk++)
				{
				c_00 += XMATEL_A(aai+ii, aaj+kk) * XMATEL_B(bbi+jj, bbj+kk);
				}
			for(kk=0; kk<jj; kk++)
				{
				c_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_D(ddi+jj, ddj+kk);
				}
			XMATEL_D(ddi+ii, ddj+jj) = c_00 * f_00_inv;
			}
		}
	return;
	}
#endif



#if ! ( defined(REF_BLAS) )
// cholesky factorization with pivot
void REF_PSTRF_L(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, int *ipiv)
	{
	if(m<=0)
		return;
	int ii, jj, kk;
	REAL
		tmp,
		f_00_inv,
		c_00,
		c_max;
	int idx;
#if defined(MF_COLMAJ)
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA;
	if(di==0 & dj==0)
		// if zero offset potrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;
	// copy C in D
	for(ii=0; ii<m; ii++)
		for(jj=ii; jj<m; jj++)
			XMATEL_D(ddi+jj, ddj+ii) = XMATEL_C(cci+jj, ccj+ii);
	// fact
	for(ii=0; ii<m; ii++)
		{
		// pivot
		c_max = XMATEL_D(cci+ii, ccj+ii);
		idx = ii;
		for(jj=ii+1; jj<m; jj++)
			{
			if(XMATEL_D(ddi+jj, ddj+jj)>c_max)
				{
				c_max = XMATEL_D(ddi+jj, ddj+jj);
				idx = jj;
				}
			}
		ipiv[ii] = idx;
		// swap ii and idx
		if(ii!=idx)
			{
			//
			for(kk=0; kk<ii; kk++)
				{
				tmp = XMATEL_D(ddi+ii, ddj+kk);
				XMATEL_D(ddi+ii, ddj+kk) = XMATEL_D(ddi+idx, ddj+kk);
				XMATEL_D(ddi+idx, ddj+kk) = tmp;
				}
			//
			tmp = XMATEL_D(ddi+ii, ddj+ii);
			XMATEL_D(ddi+ii, ddj+ii) = XMATEL_D(ddi+idx, ddj+idx);
			XMATEL_D(ddi+idx, ddj+idx) = tmp;
			//
			for(kk=ii+1; kk<idx; kk++)
				{
				tmp = XMATEL_D(ddi+kk, ddj+ii);
				XMATEL_D(ddi+kk, ddj+ii) = XMATEL_D(ddi+idx, ddj+kk);
				XMATEL_D(ddi+idx, ddj+kk) = tmp;
				}
			//
			for(kk=idx+1; kk<m; kk++)
				{
				tmp = XMATEL_D(ddi+kk, ddj+ii);
				XMATEL_D(ddi+kk, ddj+ii) = XMATEL_D(ddi+kk, ddj+idx);
				XMATEL_D(ddi+kk, ddj+idx) = tmp;
				}
			}
		c_00 = XMATEL_D(cci+ii, ccj+ii);
		if(c_00>0)
			f_00_inv = 1.0/SQRT(c_00);
		else
			f_00_inv = 0.0;
		dD[ii+0] = f_00_inv;
		for(jj=ii; jj<m; jj++)
			{
			XMATEL_D(ddi+jj, ddj+ii) *= f_00_inv;
			}
		for(jj=ii+1; jj<m; jj++)
			{
			for(kk=jj; kk<m; kk++)
				{
				XMATEL_D(ddi+kk, ddj+jj) -= XMATEL_D(ddi+kk, ddj+ii) * XMATEL_D(ddi+jj, ddj+ii);
				}
			}
		}
	return;
	}
#endif



// dgetrf without pivoting
#if 0
void GETF2_NOPIVOT(int m, int n, REAL *A, int lda, REAL *dA)
	{
	int ii, jj, kk, itmp0, itmp1;
	int iimax = m<n ? m : n;
	int i1 = 1;
	REAL dtmp;
	REAL dm1 = -1.0;

	for(ii=0; ii<iimax; ii++)
		{
		itmp0 = m-ii-1;
		dtmp = 1.0/A[ii+lda*ii];
		dA[ii] = dtmp;
		for(jj=0; jj<itmp0; jj++)
			{
			A[ii+1+jj+lda*ii] *= dtmp;
			}
		itmp1 = n-ii-1;
		for(jj=0; jj<itmp1; jj++)
			{
			for(kk=0; kk<itmp0; kk++)
				{
				A[(ii+1+kk)+lda*(ii+1+jj)] -= A[(ii+1+kk)+lda*ii] * A[ii+lda*(ii+1+jj)];
				}
			}
		}
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
#if ! ( defined(REF_BLAS) )
// dgetrf without pivoting
void REF_GETRF_NOPIVOT(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	int ii, jj, kk;
//	int i1 = 1;
//	REAL d1 = 1.0;
	REAL
		d_00_inv, d_11_inv,
		d_00, d_01,
		d_10, d_11;
#if defined(MF_COLMAJ)
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA;
	if(di==0 & dj==0)
		// if zero offset potrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;
#if 1
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		// upper
		ii = 0;
		for(; ii<jj-1; ii+=2)
			{
			// correct upper
			d_00 = XMATEL_C(cci+(ii+0), ccj+(jj+0));
			d_10 = XMATEL_C(cci+(ii+1), ccj+(jj+0));
			d_01 = XMATEL_C(cci+(ii+0), ccj+(jj+1));
			d_11 = XMATEL_C(cci+(ii+1), ccj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// solve upper
			d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * d_00;
			d_11 -= XMATEL_D(ddi+(ii+1), ddj+kk) * d_01;
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = d_11;
			}
		for(; ii<jj; ii++)
			{
			// correct upper
			d_00 = XMATEL_C(cci+(ii+0), ccj+(jj+0));
			d_01 = XMATEL_C(cci+(ii+0), ccj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// solve upper
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			}
		// diagonal
		ii = jj;
		if(ii<m-1)
			{
			// correct diagonal
			d_00 = XMATEL_C(cci+(ii+0), ccj+(jj+0));
			d_10 = XMATEL_C(cci+(ii+1), ccj+(jj+0));
			d_01 = XMATEL_C(cci+(ii+0), ccj+(jj+1));
			d_11 = XMATEL_C(cci+(ii+1), ccj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// factorize diagonal
			d_00_inv = 1.0/d_00;
			d_10 *= d_00_inv;
			d_11 -= d_10 * d_01;
			d_11_inv = 1.0/d_11;
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = d_11;
			dD[ii+0] = d_00_inv;
			dD[ii+1] = d_11_inv;
			ii += 2;
			}
		else if(ii<m)
			{
			// correct diagonal
			d_00 = XMATEL_C(cci+(ii+0), ccj+(jj+0));
			d_01 = XMATEL_C(cci+(ii+0), ccj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// factorize diagonal
			d_00_inv = 1.0/d_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			dD[ii+0] = d_00_inv;
			ii += 1;
			}
		// lower
		for(; ii<m-1; ii+=2)
			{
			// correct lower
			d_00 = XMATEL_C(cci+(ii+0), ccj+(jj+0));
			d_10 = XMATEL_C(cci+(ii+1), ccj+(jj+0));
			d_01 = XMATEL_C(cci+(ii+0), ccj+(jj+1));
			d_11 = XMATEL_C(cci+(ii+1), ccj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// solve lower
			d_00 *= d_00_inv;
			d_10 *= d_00_inv;
			d_01 -= d_00 * XMATEL_D(ddi+kk, ddj+(jj+1));
			d_11 -= d_10 * XMATEL_D(ddi+kk, ddj+(jj+1));
			d_01 *= d_11_inv;
			d_11 *= d_11_inv;
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			// correct lower
			d_00 = XMATEL_C(cci+(ii+0), ccj+(jj+0));
			d_01 = XMATEL_C(cci+(ii+0), ccj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// solve lower
			d_00 *= d_00_inv;
			d_01 -= d_00 * XMATEL_D(ddi+kk, ddj+(jj+1));
			d_01 *= d_11_inv;
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			}
		}
	for(; jj<n; jj++)
		{
		// upper
		ii = 0;
		for(; ii<jj-1; ii+=2)
			{
			// correct upper
			d_00 = XMATEL_C(cci+(ii+0), ccj+jj);
			d_10 = XMATEL_C(cci+(ii+1), ccj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// solve upper
			d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * d_00;
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+jj) = d_10;
			}
		for(; ii<jj; ii++)
			{
			// correct upper
			d_00 = XMATEL_C(cci+(ii+0), ccj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// solve upper
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			}
		// diagonal
		ii = jj;
		if(ii<m-1)
			{
			// correct diagonal
			d_00 = XMATEL_C(cci+(ii+0), ccj+jj);
			d_10 = XMATEL_C(cci+(ii+1), ccj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// factorize diagonal
			d_00_inv = 1.0/d_00;
			d_10 *= d_00_inv;
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+jj) = d_10;
			dD[ii+0] = d_00_inv;
			ii += 2;
			}
		else if(ii<m)
			{
			// correct diagonal
			d_00 = XMATEL_C(cci+(ii+0), ccj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// factorize diagonal
			d_00_inv = 1.0/d_00;
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			dD[ii+0] = d_00_inv;
			ii += 1;
			}
		// lower
		for(; ii<m-1; ii+=2)
			{
			// correct lower
			d_00 = XMATEL_C(cci+(ii+0), ccj+jj);
			d_10 = XMATEL_C(cci+(ii+1), ccj+jj);
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// solve lower
			d_00 *= d_00_inv;
			d_10 *= d_00_inv;
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+jj) = d_10;
			}
		for(; ii<m; ii++)
			{
			// correct lower
			d_00 = XMATEL_C(cci+(ii+0), ccj+jj);
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// solve lower
			d_00 *= d_00_inv;
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			}
		}
#else
	if(pC!=pD)
		{
		for(jj=0; jj<n; jj++)
			{
			for(ii=0; ii<m; ii++)
				{
				pD[ii+ldd*jj] = pC[ii+ldc*jj];
				}
			}
		}
	GETF2_NOPIVOT(m, n, pD, ldd, dD);
#endif
	return;
	}
#endif
#endif



#if ! ( defined(HP_CM) & defined(DP) )
// dgetrf pivoting
void REF_GETRF_ROWPIVOT(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, int *ipiv)
	{
	int ii, i0, jj, kk, ip, itmp0, itmp1;
	REAL dtmp, dmax;
	REAL
		d_00_inv, d_11_inv,
		d_00, d_01,
		d_10, d_11;
	int i1 = 1;
	REAL d1 = 1.0;
//#if defined(MF_COLMAJ)
#if defined(MF_COLMAJ) | defined(REF_BLAS)
	int ldc = sC->m;
	int ldd = sD->m;
	REAL *pC = sC->pA + ci + cj*ldc;
	REAL *pD = sD->pA + di + dj*ldd;
	const int cci=0; const int ccj=0;
	const int ddi=0; const int ddj=0;
#else
	int cci=ci; int ccj=cj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA;
	if(di==0 & dj==0)
		// if zero offset getrf recomputes the right values
		// for the stored inverse diagonal of sD
		sD->use_dA = 1;
	else
		sD->use_dA = 0;
	// copy if needed
	if(&XMATEL_C(cci, ccj)!=&XMATEL_D(ddi, ddj))
		{
		for(jj=0; jj<n; jj++)
			{
			for(ii=0; ii<m; ii++)
				{
				XMATEL_D(ddi+ii, ddj+jj) = XMATEL_C(cci+ii, ccj+jj);
				}
			}
		}
	// factorize
#if 1
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		ii = 0;
		for(; ii<jj-1; ii+=2)
			{
			// correct upper
			d_00 = XMATEL_D(ddi+(ii+0), ddj+(jj+0));
			d_10 = XMATEL_D(ddi+(ii+1), ddj+(jj+0));
			d_01 = XMATEL_D(ddi+(ii+0), ddj+(jj+1));
			d_11 = XMATEL_D(ddi+(ii+1), ddj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// solve upper
			d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * d_00;
			d_11 -= XMATEL_D(ddi+(ii+1), ddj+kk) * d_01;
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = d_11;
			}
		for(; ii<jj; ii++)
			{
			// correct upper
			d_00 = XMATEL_D(ddi+(ii+0), ddj+(jj+0));
			d_01 = XMATEL_D(ddi+(ii+0), ddj+(jj+1));
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			// solve upper
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			}
		// correct diagonal and lower and look for pivot
		// correct
		ii = jj;
		i0 = ii;
		for(; ii<m-1; ii+=2)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+(jj+0));
			d_10 = XMATEL_D(ddi+(ii+1), ddj+(jj+0));
			d_01 = XMATEL_D(ddi+(ii+0), ddj+(jj+1));
			d_11 = XMATEL_D(ddi+(ii+1), ddj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				d_11 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+(jj+0));
			d_01 = XMATEL_D(ddi+(ii+0), ddj+(jj+1));
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+0));
				d_01 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+(jj+1));
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			}
		// look for pivot & solve
		// left column
		ii = i0;
		dmax = 0;
		ip = ii;
		for(; ii<m-1; ii+=2)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+jj);
			d_10 = XMATEL_D(ddi+(ii+1), ddj+jj);
			dtmp = d_00>0.0 ? d_00 : -d_00;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+0;
				}
			dtmp = d_10>0.0 ? d_10 : -d_10;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+1;
				}
			}
		for(; ii<m; ii++)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+jj);
			dtmp = d_00>0.0 ? d_00 : -d_00;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+0;
				}
			}
		// row swap
		ii = i0;
		ipiv[ii] = ip;
		if(ip!=ii)
			{
			for(kk=0; kk<n; kk++)
				{
				dtmp = XMATEL_D(ddi+ii, ddj+kk);
				XMATEL_D(ddi+ii, ddj+kk) = XMATEL_D(ddi+ip, ddj+kk);
				XMATEL_D(ddi+ip, ddj+kk) = dtmp;
				}
			}
		// factorize diagonal
		d_00 = XMATEL_D(ddi+(ii+0), ddj+(jj+0));
		d_00_inv = 1.0/d_00;
		XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
		dD[ii] = d_00_inv;
		ii += 1;
		// solve & compute next pivot
		dmax = 0;
		ip = ii;
		for(; ii<m-1; ii+=2)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+(jj+0));
			d_10 = XMATEL_D(ddi+(ii+1), ddj+(jj+0));
			d_00 *= d_00_inv;
			d_10 *= d_00_inv;
			d_01 = XMATEL_D(ddi+(ii+0), ddj+(jj+1));
			d_11 = XMATEL_D(ddi+(ii+1), ddj+(jj+1));
			d_01 -= d_00 * XMATEL_D(ddi+jj, ddj+(jj+1));
			d_11 -= d_10 * XMATEL_D(ddi+jj, ddj+(jj+1));
			dtmp = d_01>0.0 ? d_01 : -d_01;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+0;
				}
			dtmp = d_11>0.0 ? d_11 : -d_11;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+1;
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+(jj+0)) = d_10;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			XMATEL_D(ddi+(ii+1), ddj+(jj+1)) = d_11;
			}
		for(; ii<m; ii++)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+(jj+0));
			d_00 *= d_00_inv;
			d_01 = XMATEL_D(ddi+(ii+0), ddj+(jj+1));
			d_01 -= d_00 * XMATEL_D(ddi+jj, ddj+(jj+1));
			dtmp = d_01>0.0 ? d_01 : -d_01;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+0;
				}
			XMATEL_D(ddi+(ii+0), ddj+(jj+0)) = d_00;
			XMATEL_D(ddi+(ii+0), ddj+(jj+1)) = d_01;
			}
		// row swap
		ii = i0+1;
		ipiv[ii] = ip;
		if(ip!=ii)
			{
			for(kk=0; kk<n; kk++)
				{
				dtmp = XMATEL_D(ddi+ii, ddj+kk);
				XMATEL_D(ddi+ii, ddj+kk) = XMATEL_D(ddi+ip, ddj+kk);
				XMATEL_D(ddi+ip, ddj+kk) = dtmp;
				}
			}
		// factorize diagonal
		d_00 = XMATEL_D(ddi+ii, ddj+(jj+1));
		d_00_inv = 1.0/d_00;
		XMATEL_D(ddi+ii, ddj+(jj+1)) = d_00;
		dD[ii] = d_00_inv;
		ii += 1;
		// solve lower
		for(; ii<m; ii++)
			{
			d_00 = XMATEL_D(ddi+ii, ddj+(jj+1));
			d_00 *= d_00_inv;
			XMATEL_D(ddi+ii, ddj+(jj+1)) = d_00;
			}
		}
	for(; jj<n; jj++)
		{
		ii = 0;
		for(; ii<jj-1; ii+=2)
			{
			// correct upper
			d_00 = XMATEL_D(ddi+(ii+0), ddj+jj);
			d_10 = XMATEL_D(ddi+(ii+1), ddj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// solve upper
			d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * d_00;
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+jj) = d_10;
			}
		for(; ii<jj; ii++)
			{
			// correct upper
			d_00 = XMATEL_D(ddi+ii, ddj+jj);
			for(kk=0; kk<ii; kk++)
				{
				d_00 -= XMATEL_D(ddi+ii, ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			// solve upper
			XMATEL_D(ddi+ii, ddj+jj) = d_00;
			}
		i0 = ii;
		ii = jj;
		// correct diagonal and lower and look for pivot
		dmax = 0;
		ip = ii;
		for(; ii<m-1; ii+=2)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+jj);
			d_10 = XMATEL_D(ddi+(ii+1), ddj+jj);
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				d_10 -= XMATEL_D(ddi+(ii+1), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			dtmp = d_00>0.0 ? d_00 : -d_00;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+0;
				}
			dtmp = d_10>0.0 ? d_10 : -d_10;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+1;
				}
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			XMATEL_D(ddi+(ii+1), ddj+jj) = d_10;
			}
		for(; ii<m; ii++)
			{
			d_00 = XMATEL_D(ddi+(ii+0), ddj+jj);
			for(kk=0; kk<jj; kk++)
				{
				d_00 -= XMATEL_D(ddi+(ii+0), ddj+kk) * XMATEL_D(ddi+kk, ddj+jj);
				}
			dtmp = d_00>0.0 ? d_00 : -d_00;
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+0;
				}
			XMATEL_D(ddi+(ii+0), ddj+jj) = d_00;
			}
		// row swap
		ii = i0;
		ipiv[ii] = ip;
		if(ip!=ii)
			{
			for(kk=0; kk<n; kk++)
				{
				dtmp = XMATEL_D(ddi+ii, ddj+kk);
				XMATEL_D(ddi+ii, ddj+kk) = XMATEL_D(ddi+ip, ddj+kk);
				XMATEL_D(ddi+ip, ddj+kk) = dtmp;
				}
			}
		// factorize diagonal
		d_00 = XMATEL_D(ddi+ii, ddj+jj);
		d_00_inv = 1.0/d_00;
		XMATEL_D(ddi+ii, ddj+jj) = d_00;
		dD[ii] = d_00_inv;
		ii += 1;
		for(; ii<m; ii++)
			{
			// correct lower
			d_00 = XMATEL_D(ddi+ii, ddj+jj);
			// solve lower
			d_00 *= d_00_inv;
			XMATEL_D(ddi+ii, ddj+jj) = d_00;
			}
		}
#else
	int iimax = m<n ? m : n;
	for(ii=0; ii<iimax; ii++)
		{
		dmax = (XMATEL_D(ddi+ii, ddj+ii)>0 ? XMATEL_D(ddi+ii, ddj+ii) : -XMATEL_D(ddi+ii, ddj+ii));
		ip = ii;
		for(jj=1; jj<m-ii; jj++)
			{
			dtmp = XMATEL_D(ddi+ii+jj, ddj+ii)>0 ? XMATEL_D(ddi+ii+jj, ddj+ii) : -XMATEL_D(ddi+ii+jj, ddj+ii);
			if(dtmp>dmax)
				{
				dmax = dtmp;
				ip = ii+jj;
				}
			}
		ipiv[ii] = ip;
		if(ip!=ii)
			{
			for(jj=0; jj<n; jj++)
				{
				dtmp = XMATEL_D(ddi+ii, ddj+jj);
				XMATEL_D(ddi+ii, ddj+jj) = XMATEL_D(ddi+ip, ddj+jj);
				XMATEL_D(ddi+ip, ddj+jj) = dtmp;
				}
			}
		itmp0 = m-ii-1;
		dtmp = 1.0/XMATEL_D(ddi+ii, ddj+ii);
		dD[ii] = dtmp;
		for(jj=0; jj<itmp0; jj++)
			{
			XMATEL_D(ddi+ii+1+jj, ddj+ii) *= dtmp;
			}
		itmp1 = n-ii-1;
		for(jj=0; jj<itmp1; jj++)
			{
			for(kk=0; kk<itmp0; kk++)
				{
				XMATEL_D(ddi+(ii+1+kk), ddj+(ii+1+jj)) -= XMATEL_D(ddi+(ii+1+kk), ddj+ii) * XMATEL_D(ddi+ii, ddj+(ii+1+jj));
				}
			}
		}
#endif
	return;
	}
#endif



#if ! ( defined(REF_BLAS) )
int REF_GEQRF_WORK_SIZE(int m, int n)
	{
	return 0;
	}
#endif



#if ! ( defined(REF_BLAS) )
void REF_GEQRF(int m, int n, struct XMAT *sA, int ai, int aj, struct XMAT *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA+di; // vectors of tau
	REAL alpha, beta, tmp, w0, w1;
	REAL pW[4] = {0.0, 0.0, 0.0, 0.0};
	int ldw = 2;
	REAL pT[4] = {0.0, 0.0, 0.0, 0.0};
	int ldb = 2;
	int imax, jmax, kmax;
	// copy if needed
	if(&XMATEL_A(aai, aaj)!=&XMATEL_D(ddi, ddj))
		{
		for(jj=0; jj<n; jj++)
			{
			for(ii=0; ii<m; ii++)
				{
				XMATEL_D(ddi+ii, ddj+jj) = XMATEL_A(aai+ii, aaj+jj);
				}
			}
		}
	imax = m<n ? m : n;
	ii = 0;
#if 1
	for(; ii<imax-1; ii+=2)
		{
		// first column
		beta = 0.0;
		for(jj=1; jj<m-ii; jj++)
			{
			tmp = XMATEL_D(ddi+ii+jj, ddj+ii+0);
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			// tau0
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta += alpha*alpha;
			beta = SQRT(beta);
			if(alpha>0)
				beta = -beta;
			// tau0
			dD[ii] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=1; jj<m-ii; jj++)
				{
				XMATEL_D(ddi+ii+jj, ddj+ii+0) *= tmp;
				}
			}
		// gemv_t & ger
		kmax = m-ii;
		w0 = XMATEL_D(ddi+ii+0, ddj+ii+1+0); // pv0[0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			w0 += XMATEL_D(ddi+ii+kk, ddj+ii+1+0) * XMATEL_D(ddi+ii+kk, ddj+ii);
			}
		w0 = - dD[ii] * w0;
		XMATEL_D(ddi+ii+0, ddj+ii+1+0) += w0; // pv0[0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			XMATEL_D(ddi+ii+kk, ddj+ii+1+0) += w0 * XMATEL_D(ddi+ii+kk, ddj+ii);
			}
		// second column
		beta = 0.0;
		for(jj=1; jj<m-(ii+1); jj++)
			{
			tmp = XMATEL_D(ddi+ii+1+jj, ddj+ii+1+0);
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			// tau1
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+1+0, ddj+ii+1+0);
			beta += alpha*alpha;
			beta = SQRT(beta);
			if(alpha>0)
				beta = -beta;
			// tau1
			dD[(ii+1)] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			// compute v1
			XMATEL_D(ddi+ii+1+0, ddj+ii+1+0) = beta;
			for(jj=1; jj<m-(ii+1); jj++)
				XMATEL_D(ddi+ii+1+jj, ddj+ii+1+0) *= tmp;
			}
		// compute lower triangular T containing tau for matrix update
		kmax = m-ii;
		tmp = XMATEL_D(ddi+ii+1, ddj+ii);
		for(kk=2; kk<kmax; kk++)
			tmp += XMATEL_D(ddi+ii+kk, ddj+ii)*XMATEL_D(ddi+ii+kk, ddj+ii+1);
		pT[0+ldb*0] = dD[ii+0];
		pT[1+ldb*0] = - dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldb*1] = dD[ii+1];
		jmax = n-ii-2;
		jj = 0;
		for(; jj<jmax-1; jj+=2)
			{
			// compute W^T = C^T * V
			pW[0+ldw*0] = XMATEL_D(ddi+ii+0, ddj+ii+(jj+0+2)) + XMATEL_D(ddi+ii+1, ddj+ii+(jj+0+2)) * XMATEL_D(ddi+ii+1, ddj+ii);
			pW[1+ldw*0] = XMATEL_D(ddi+ii+0, ddj+ii+(jj+1+2)) + XMATEL_D(ddi+ii+1, ddj+ii+(jj+1+2)) * XMATEL_D(ddi+ii+1, ddj+ii);
			pW[0+ldw*1] =                        XMATEL_D(ddi+ii+1, ddj+ii+(jj+0+2));
			pW[1+ldw*1] =                        XMATEL_D(ddi+ii+1, ddj+ii+(jj+1+2));
			kk = 2;
			for(; kk<kmax; kk++)
				{
				tmp = XMATEL_D(ddi+ii+kk, ddj+ii+(jj+0+2));
				pW[0+ldw*0] += tmp * XMATEL_D(ddi+ii+kk, ddj+ii);
				pW[0+ldw*1] += tmp * XMATEL_D(ddi+ii+kk, ddj+ii+1);
				tmp = XMATEL_D(ddi+ii+kk, ddj+ii+(jj+1+2));
				pW[1+ldw*0] += tmp * XMATEL_D(ddi+ii+kk, ddj+ii);
				pW[1+ldw*1] += tmp * XMATEL_D(ddi+ii+kk, ddj+ii+1);
				}
			// compute W^T *= T
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[1+ldw*1] = pT[1+ldb*0]*pW[1+ldw*0] + pT[1+ldb*1]*pW[1+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			pW[1+ldw*0] = pT[0+ldb*0]*pW[1+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+0, ddj+ii+(jj+0+2)) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+0, ddj+ii+(jj+1+2)) -= pW[1+ldw*0];
			XMATEL_D(ddi+ii+1, ddj+ii+(jj+0+2)) -= XMATEL_D(ddi+ii+1, ddj+ii)*pW[0+ldw*0] + pW[0+ldw*1];
			XMATEL_D(ddi+ii+1, ddj+ii+(jj+1+2)) -= XMATEL_D(ddi+ii+1, ddj+ii)*pW[1+ldw*0] + pW[1+ldw*1];
			kk = 2;
			for(; kk<kmax-1; kk+=2)
				{
				XMATEL_D(ddi+ii+kk+0, ddj+ii+(jj+0+2)) -= XMATEL_D(ddi+ii+kk+0, ddj+ii)*pW[0+ldw*0] + XMATEL_D(ddi+ii+kk+0, ddj+ii+1)*pW[0+ldw*1];
				XMATEL_D(ddi+ii+kk+1, ddj+ii+(jj+0+2)) -= XMATEL_D(ddi+ii+kk+1, ddj+ii)*pW[0+ldw*0] + XMATEL_D(ddi+ii+kk+1, ddj+ii+1)*pW[0+ldw*1];
				XMATEL_D(ddi+ii+kk+0, ddj+ii+(jj+1+2)) -= XMATEL_D(ddi+ii+kk+0, ddj+ii)*pW[1+ldw*0] + XMATEL_D(ddi+ii+kk+0, ddj+ii+1)*pW[1+ldw*1];
				XMATEL_D(ddi+ii+kk+1, ddj+ii+(jj+1+2)) -= XMATEL_D(ddi+ii+kk+1, ddj+ii)*pW[1+ldw*0] + XMATEL_D(ddi+ii+kk+1, ddj+ii+1)*pW[1+ldw*1];
				}
			for(; kk<kmax; kk++)
				{
				XMATEL_D(ddi+ii+kk, ddj+ii+(jj+0+2)) -= XMATEL_D(ddi+ii+kk, ddj+ii)*pW[0+ldw*0] + XMATEL_D(ddi+ii+kk, ddj+ii+1)*pW[0+ldw*1];
				XMATEL_D(ddi+ii+kk, ddj+ii+(jj+1+2)) -= XMATEL_D(ddi+ii+kk, ddj+ii)*pW[1+ldw*0] + XMATEL_D(ddi+ii+kk, ddj+ii+1)*pW[1+ldw*1];
				}
			}
		for(; jj<jmax; jj++)
			{
			// compute W = T * V^T * C
			pW[0+ldw*0] = XMATEL_D(ddi+ii+0, ddj+ii+(jj+0+2)) + XMATEL_D(ddi+ii+1, ddj+ii+(jj+0+2)) * XMATEL_D(ddi+ii+1, ddj+ii);
			pW[0+ldw*1] =                        XMATEL_D(ddi+ii+1, ddj+ii+(jj+0+2));
			for(kk=2; kk<kmax; kk++)
				{
				tmp = XMATEL_D(ddi+ii+kk, ddj+ii+(jj+0+2));
				pW[0+ldw*0] += tmp * XMATEL_D(ddi+ii+kk, ddj+ii);
				pW[0+ldw*1] += tmp * XMATEL_D(ddi+ii+kk, ddj+ii+1);
				}
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+0, ddj+ii+(jj+0+2)) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+1, ddj+ii+(jj+0+2)) -= XMATEL_D(ddi+ii+1, ddj+ii)*pW[0+ldw*0] + pW[0+ldw*1];
			for(kk=2; kk<kmax; kk++)
				{
				XMATEL_D(ddi+ii+kk, ddj+ii+(jj+0+2)) -= XMATEL_D(ddi+ii+kk, ddj+ii)*pW[0+ldw*0] + XMATEL_D(ddi+ii+kk, ddj+ii+1)*pW[0+ldw*1];
				}
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		beta = 0.0;
		for(jj=1; jj<m-ii; jj++)
			{
			tmp = XMATEL_D(ddi+ii+jj, ddj+ii+0);
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta += alpha*alpha;
			beta = SQRT(beta);
			if(alpha>0)
				beta = -beta;
			dD[ii] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			for(jj=1; jj<m-ii; jj++)
				XMATEL_D(ddi+ii+jj, ddj+ii+0) *= tmp;
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			}
		if(ii<n)
			{
			// gemv_t & ger
			jmax = n-ii-1;
			kmax = m-ii;
			jj = 0;
			for(; jj<jmax-1; jj+=2)
				{
				w0 = XMATEL_D(ddi+ii+0, ddj+ii+1+(jj+0)); // pv0[0] = 1.0
				w1 = XMATEL_D(ddi+ii+0, ddj+ii+1+(jj+1)); // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					w0 += XMATEL_D(ddi+ii+kk, ddj+ii+1+(jj+0)) * XMATEL_D(ddi+ii+kk, ddj+ii);
					w1 += XMATEL_D(ddi+ii+kk, ddj+ii+1+(jj+1)) * XMATEL_D(ddi+ii+kk, ddj+ii);
					}
				w0 = - dD[ii] * w0;
				w1 = - dD[ii] * w1;
				XMATEL_D(ddi+ii+0, ddj+ii+1+(jj+0)) += w0; // pv0[0] = 1.0
				XMATEL_D(ddi+ii+0, ddj+ii+1+(jj+1)) += w1; // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					XMATEL_D(ddi+ii+kk, ddj+ii+1+(jj+0)) += w0 * XMATEL_D(ddi+ii+kk, ddj+ii);
					XMATEL_D(ddi+ii+kk, ddj+ii+1+(jj+1)) += w1 * XMATEL_D(ddi+ii+kk, ddj+ii);
					}
				}
			for(; jj<jmax; jj++)
				{
				w0 = XMATEL_D(ddi+ii+0, ddj+ii+1+jj); // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					w0 += XMATEL_D(ddi+ii+kk, ddj+ii+1+jj) * XMATEL_D(ddi+ii+kk, ddj+ii);
					}
				w0 = - dD[ii] * w0;
				XMATEL_D(ddi+ii+0, ddj+ii+1+jj) += w0; // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					XMATEL_D(ddi+ii+kk, ddj+ii+1+jj) += w0 * XMATEL_D(ddi+ii+kk, ddj+ii);
					}
				}
			}
		}
	return;
	}
#endif



#if ! ( defined(REF_BLAS) )
int REF_GELQF_WORK_SIZE(int m, int n)
	{
	return 0;
	}
#endif



#if ! ( defined(REF_BLAS) )
void REF_GELQF(int m, int n, struct XMAT *sA, int ai, int aj, struct XMAT *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA+di; // vectors of tau
	REAL alpha, beta, sigma, tmp, w0, w1;
	REAL pW[4] = {0.0, 0.0, 0.0, 0.0};
	int ldw = 2;
	REAL pT[4] = {0.0, 0.0, 0.0, 0.0};
	int ldb = 2;
	int imax, jmax, kmax;
	// copy if needed
	if(&XMATEL_A(aai, aaj)!=&XMATEL_D(ddi, ddj))
		{
		for(jj=0; jj<n; jj++)
			{
			for(ii=0; ii<m; ii++)
				{
				XMATEL_D(ddi+ii, ddj+jj) = XMATEL_A(aai+ii, aaj+jj);
				}
			}
		}
	imax = m<n ? m : n;
	ii = 0;
#if 1
	for(; ii<imax-1; ii+=2)
		{
		// first column
		beta = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = XMATEL_D(ddi+ii+0, ddj+ii+jj);
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			// tau0
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta += alpha*alpha;
			beta = SQRT(beta);
			if(alpha>0)
				beta = -beta;
			// tau0
			dD[ii] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=1; jj<n-ii; jj++)
				{
				XMATEL_D(ddi+ii+0, ddj+ii+jj) *= tmp;
				}
			}
		// gemv_t & ger
		kmax = n-ii;
		w0 = XMATEL_D(ddi+ii+1+0, ddj+ii+0); // pv0[0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			w0 += XMATEL_D(ddi+ii+1+0, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
			}
		w0 = - dD[ii] * w0;
		XMATEL_D(ddi+ii+1+0, ddj+ii+0) += w0; // pv0[0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			XMATEL_D(ddi+ii+1+0, ddj+ii+kk) += w0 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
			}
		// second row
		beta = 0.0;
		for(jj=1; jj<n-(ii+1); jj++)
			{
			tmp = XMATEL_D(ddi+ii+1+0, ddj+ii+1+jj);
			beta += tmp*tmp;
			}
		if(beta==0.0)
			{
			// tau1
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+1+0, ddj+ii+1+0);
			beta += alpha*alpha;
			beta = SQRT(beta);
			if(alpha>0)
				beta = -beta;
			// tau1
			dD[(ii+1)] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			// compute v1
			XMATEL_D(ddi+ii+1+0, ddj+ii+1+0) = beta;
			for(jj=1; jj<n-(ii+1); jj++)
				XMATEL_D(ddi+ii+1+0, ddj+ii+1+jj) *= tmp;
			}
		// compute lower triangular T containing tau for matrix update
		kmax = n-ii;
		tmp = XMATEL_D(ddi+ii+0, ddj+ii+1);
		for(kk=2; kk<kmax; kk++)
			tmp += XMATEL_D(ddi+ii+0, ddj+ii+kk)*XMATEL_D(ddi+ii+1, ddj+ii+kk);
		pT[0+ldb*0] = dD[ii+0];
		pT[1+ldb*0] = - dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldb*1] = dD[ii+1];
		// downgrade
		jmax = m-ii-2;
		jj = 0;
		for(; jj<jmax-1; jj+=2)
			{
			// compute W^T = C^T * V
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) + XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) * XMATEL_D(ddi+ii+0, ddj+ii+1);
			pW[1+ldw*0] = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0) + XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1) * XMATEL_D(ddi+ii+0, ddj+ii+1);
			pW[0+ldw*1] =                      XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1);
			pW[1+ldw*1] =                      XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1);
			kk = 2;
			for(; kk<kmax; kk++)
				{
				tmp = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk);
				pW[0+ldw*0] += tmp * XMATEL_D(ddi+ii+0, ddj+ii+kk);
				pW[0+ldw*1] += tmp * XMATEL_D(ddi+ii+1, ddj+ii+kk);
				tmp = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+kk);
				pW[1+ldw*0] += tmp * XMATEL_D(ddi+ii+0, ddj+ii+kk);
				pW[1+ldw*1] += tmp * XMATEL_D(ddi+ii+1, ddj+ii+kk);
				}
			// compute W^T *= T
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[1+ldw*1] = pT[1+ldb*0]*pW[1+ldw*0] + pT[1+ldb*1]*pW[1+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			pW[1+ldw*0] = pT[0+ldb*0]*pW[1+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0) -= pW[1+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= XMATEL_D(ddi+ii+0, ddj+ii+1)*pW[0+ldw*0] + pW[0+ldw*1];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1) -= XMATEL_D(ddi+ii+0, ddj+ii+1)*pW[1+ldw*0] + pW[1+ldw*1];
			kk = 2;
			for(; kk<kmax-1; kk+=2)
				{
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+(kk+0)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+0))*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+0))*pW[0+ldw*1];
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+(kk+1)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+1))*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+1))*pW[0+ldw*1];
				XMATEL_D(ddi+ii+jj+1+2, ddj+ii+(kk+0)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+0))*pW[1+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+0))*pW[1+ldw*1];
				XMATEL_D(ddi+ii+jj+1+2, ddj+ii+(kk+1)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+1))*pW[1+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+1))*pW[1+ldw*1];
				}
			for(; kk<kmax; kk++)
				{
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk) -= XMATEL_D(ddi+ii+0, ddj+ii+kk)*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+kk)*pW[0+ldw*1];
				XMATEL_D(ddi+ii+jj+1+2, ddj+ii+kk) -= XMATEL_D(ddi+ii+0, ddj+ii+kk)*pW[1+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+kk)*pW[1+ldw*1];
				}
			}
		for(; jj<jmax; jj++)
			{
			// compute W = T * V^T * C
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) + XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) * XMATEL_D(ddi+ii+0, ddj+ii+1);
			pW[0+ldw*1] =                      XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1);
			for(kk=2; kk<kmax; kk++)
				{
				tmp = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk);
				pW[0+ldw*0] += tmp * XMATEL_D(ddi+ii+0, ddj+ii+kk);
				pW[0+ldw*1] += tmp * XMATEL_D(ddi+ii+1, ddj+ii+kk);
				}
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= XMATEL_D(ddi+ii+0, ddj+ii+1)*pW[0+ldw*0] + pW[0+ldw*1];
			for(kk=2; kk<kmax; kk++)
				{
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk) -= XMATEL_D(ddi+ii+0, ddj+ii+kk)*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+kk)*pW[0+ldw*1];
				}
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		sigma = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = XMATEL_D(ddi+ii+0, ddj+ii+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha>0)
				beta = -beta;
			dD[ii] = (beta-alpha) / beta;
			tmp = 1.0 / (alpha-beta);
			for(jj=1; jj<n-ii; jj++)
				XMATEL_D(ddi+ii+0, ddj+ii+jj) *= tmp;
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			}
		if(ii<n)
			{
			// gemv_t & ger
			jmax = m-ii-1;
			kmax = n-ii;
			jj = 0;
			for(; jj<jmax-1; jj+=2)
				{
				w0 = XMATEL_D(ddi+ii+1+jj+0, ddj+ii+0); // pv0[0] = 1.0
				w1 = XMATEL_D(ddi+ii+1+jj+1, ddj+ii+0); // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					w0 += XMATEL_D(ddi+ii+1+jj+0, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					w1 += XMATEL_D(ddi+ii+1+jj+1, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				w0 = - dD[ii] * w0;
				w1 = - dD[ii] * w1;
				XMATEL_D(ddi+ii+1+jj+0, ddj+ii+0) += w0; // pv0[0] = 1.0
				XMATEL_D(ddi+ii+1+jj+1, ddj+ii+0) += w1; // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					XMATEL_D(ddi+ii+1+jj+0, ddj+ii+kk) += w0 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					XMATEL_D(ddi+ii+1+jj+1, ddj+ii+kk) += w1 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				}
			for(; jj<jmax; jj++)
				{
				w0 = XMATEL_D(ddi+ii+1+jj, ddj+ii+0); // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					w0 += XMATEL_D(ddi+ii+1+jj, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				w0 = - dD[ii] * w0;
				XMATEL_D(ddi+ii+1+jj, ddj+ii+0) += w0; // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					XMATEL_D(ddi+ii+1+jj, ddj+ii+kk) += w0 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				}
			}
		}
	return;
	}
#endif



#if ! ( defined(REF_BLAS) )
// generate Q matrix
int REF_ORGLQ_WORK_SIZE(int m, int n, int k)
	{
#if defined(EXT_DEP)
	printf("\nblasfeo_orglq_worksize: feature not implemented yet\n");
#endif
	exit(1);
	return 0;
	}
#endif



#if ! ( defined(REF_BLAS) )
// generate Q matrix
void REF_ORGLQ(int m, int n, int k, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;
#if defined(EXT_DEP)
	printf("\nblasfeo_orglq: feature not implemented yet\n");
#endif
	exit(1);
	}
#endif



#if ! ( defined(REF_BLAS) )
// LQ factorization with positive diagonal elements
void REF_GELQF_PD(int m, int n, struct XMAT *sA, int ai, int aj, struct XMAT *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int ii, jj, kk;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dD = sD->dA+di; // vectors of tau
	REAL alpha, beta, sigma, tmp, w0, w1;
	REAL pW[4] = {0.0, 0.0, 0.0, 0.0};
	int ldw = 2;
	REAL pT[4] = {0.0, 0.0, 0.0, 0.0};
	int ldb = 2;
	int imax, jmax, kmax;
	// copy if needed
	if(&XMATEL_A(aai, aaj)!=&XMATEL_D(ddi, ddj))
		{
		for(jj=0; jj<n; jj++)
			{
			for(ii=0; ii<m; ii++)
				{
				XMATEL_D(ddi+ii, ddj+jj) = XMATEL_A(aai+ii, aaj+jj);
				}
			}
		}
	imax = m<n ? m : n;
	ii = 0;
#if 1
	for(; ii<imax-1; ii+=2)
		{
		// first column
		sigma = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = XMATEL_D(ddi+ii+0, ddj+ii+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau0
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=1; jj<n-ii; jj++)
				XMATEL_D(ddi+ii+0, ddj+ii+jj) *= tmp;
			}
		// gemv_t & ger
		kmax = n-ii;
		w0 = XMATEL_D(ddi+ii+1+0, ddj+ii+0); // pv0[0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			w0 += XMATEL_D(ddi+ii+1+0, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
			}
		w0 = - dD[ii] * w0;
		XMATEL_D(ddi+ii+1+0, ddj+ii+0) += w0; // pv0[0] = 1.0
		for(kk=1; kk<kmax; kk++)
			{
			XMATEL_D(ddi+ii+1+0, ddj+ii+kk) += w0 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
			}
		// second row
		sigma = 0.0;
		for(jj=1; jj<n-(ii+1); jj++)
			{
			tmp = XMATEL_D(ddi+ii+1+0, ddj+ii+1+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau1
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+1+0, ddj+ii+1+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau1
			dD[ii+1] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v1
			XMATEL_D(ddi+ii+1+0, ddj+ii+1+0) = beta;
			for(jj=1; jj<n-(ii+1); jj++)
				XMATEL_D(ddi+ii+1+0, ddj+ii+1+jj) *= tmp;
			}
		// compute lower triangular T containing tau for matrix update
		kmax = n-ii;
		tmp = XMATEL_D(ddi+ii+0, ddj+ii+1);
		for(kk=2; kk<kmax; kk++)
			tmp += XMATEL_D(ddi+ii+0, ddj+ii+kk)*XMATEL_D(ddi+ii+1, ddj+ii+kk);
		pT[0+ldb*0] = dD[ii+0];
		pT[1+ldb*0] = - dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldb*1] = dD[ii+1];
		// downgrade
		jmax = m-ii-2;
		jj = 0;
		for(; jj<jmax-1; jj+=2)
			{
			// compute W^T = C^T * V
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) + XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) * XMATEL_D(ddi+ii+0, ddj+ii+1);
			pW[1+ldw*0] = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0) + XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1) * XMATEL_D(ddi+ii+0, ddj+ii+1);
			pW[0+ldw*1] =                      XMATEL_D(ddi+jj+ii+0+2, ddj+ii+1);
			pW[1+ldw*1] =                      XMATEL_D(ddi+jj+ii+1+2, ddj+ii+1);
			kk = 2;
			for(; kk<kmax; kk++)
				{
				tmp = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk);
				pW[0+ldw*0] += tmp * XMATEL_D(ddi+ii+0, ddj+ii+kk);
				pW[0+ldw*1] += tmp * XMATEL_D(ddi+ii+1, ddj+ii+kk);
				tmp = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+kk);
				pW[1+ldw*0] += tmp * XMATEL_D(ddi+ii+0, ddj+ii+kk);
				pW[1+ldw*1] += tmp * XMATEL_D(ddi+ii+1, ddj+ii+kk);
				}
			// compute W^T *= T
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[1+ldw*1] = pT[1+ldb*0]*pW[1+ldw*0] + pT[1+ldb*1]*pW[1+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			pW[1+ldw*0] = pT[0+ldb*0]*pW[1+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0) -= pW[1+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= XMATEL_D(ddi+ii+0, ddj+ii+1)*pW[0+ldw*0] + pW[0+ldw*1];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1) -= XMATEL_D(ddi+ii+0, ddj+ii+1)*pW[1+ldw*0] + pW[1+ldw*1];
			kk = 2;
			for(; kk<kmax-1; kk+=2)
				{
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+(kk+0)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+0))*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+0))*pW[0+ldw*1];
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+(kk+1)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+1))*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+1))*pW[0+ldw*1];
				XMATEL_D(ddi+ii+jj+1+2, ddj+ii+(kk+0)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+0))*pW[1+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+0))*pW[1+ldw*1];
				XMATEL_D(ddi+ii+jj+1+2, ddj+ii+(kk+1)) -= XMATEL_D(ddi+ii+0, ddj+ii+(kk+1))*pW[1+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+(kk+1))*pW[1+ldw*1];
				}
			for(; kk<kmax; kk++)
				{
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk) -= XMATEL_D(ddi+ii+0, ddj+ii+kk)*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+kk)*pW[0+ldw*1];
				XMATEL_D(ddi+ii+jj+1+2, ddj+ii+kk) -= XMATEL_D(ddi+ii+0, ddj+ii+kk)*pW[1+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+kk)*pW[1+ldw*1];
				}
			}
		for(; jj<jmax; jj++)
			{
			// compute W = T * V^T * C
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) + XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) * XMATEL_D(ddi+ii+0, ddj+ii+1);
			pW[0+ldw*1] =                      XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1);
			for(kk=2; kk<kmax; kk++)
				{
				tmp = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk);
				pW[0+ldw*0] += tmp * XMATEL_D(ddi+ii+0, ddj+ii+kk);
				pW[0+ldw*1] += tmp * XMATEL_D(ddi+ii+1, ddj+ii+kk);
				}
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= XMATEL_D(ddi+ii+0, ddj+ii+1)*pW[0+ldw*0] + pW[0+ldw*1];
			for(kk=2; kk<kmax; kk++)
				{
				XMATEL_D(ddi+ii+jj+0+2, ddj+ii+kk) -= XMATEL_D(ddi+ii+0, ddj+ii+kk)*pW[0+ldw*0] + XMATEL_D(ddi+ii+1, ddj+ii+kk)*pW[0+ldw*1];
				}
			}
		}
#endif
	for(; ii<imax; ii++)
		{
		sigma = 0.0;
		for(jj=1; jj<n-ii; jj++)
			{
			tmp = XMATEL_D(ddi+ii+0, ddj+ii+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=1; jj<n-ii; jj++)
				XMATEL_D(ddi+ii+0, ddj+ii+jj) *= tmp;
			}
		if(ii<n)
			{
			// gemv_t & ger
			jmax = m-ii-1;
			kmax = n-ii;
			jj = 0;
#if 1
			for(; jj<jmax-1; jj+=2)
				{
				w0 = XMATEL_D(ddi+ii+1+jj+0, ddj+ii+0); // pv0[0] = 1.0
				w1 = XMATEL_D(ddi+ii+1+jj+1, ddj+ii+0); // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					w0 += XMATEL_D(ddi+ii+1+jj+0, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					w1 += XMATEL_D(ddi+ii+1+jj+1, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				w0 = - dD[ii] * w0;
				w1 = - dD[ii] * w1;
				XMATEL_D(ddi+ii+1+jj+0, ddj+ii+0) += w0; // pv0[0] = 1.0
				XMATEL_D(ddi+ii+1+jj+1, ddj+ii+0) += w1; // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					XMATEL_D(ddi+ii+1+jj+0, ddj+ii+kk) += w0 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					XMATEL_D(ddi+ii+1+jj+1, ddj+ii+kk) += w1 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				}
#endif
			for(; jj<jmax; jj++)
				{
				w0 = XMATEL_D(ddi+ii+1+jj, ddj+ii+0); // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					w0 += XMATEL_D(ddi+ii+1+jj, ddj+ii+kk) * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				w0 = - dD[ii] * w0;
				XMATEL_D(ddi+ii+1+jj, ddj+ii+0) += w0; // pv0[0] = 1.0
				for(kk=1; kk<kmax; kk++)
					{
					XMATEL_D(ddi+ii+1+jj, ddj+ii+kk) += w0 * XMATEL_D(ddi+ii+0, ddj+ii+kk);
					}
				}
			}
		}
	return;
	}
#endif



#if 0
// LQ factorization with positive diagonal elements
// [D, A] <= lq( [D. A] )
// array of matrices [D, A] with
// D diagonal (on input), of size (m)x(m)
// A full of size (m)x(n1)
void GELQF_PD_DA(int m, int n1, struct XMAT *sD, int di, int dj, struct XMAT *sA, int ai, int aj, void *work)
	{
	if(m<=0)
		return;
	
	// invalidate stored inverse diagonal of result matrix
	sA->use_dA = 0;
	sD->use_dA = 0;

	int ii, jj, kk;
	int lda = sA->m;
	int ldd = sD->m;
	REAL *pA = sA->pA+ai+aj*lda; // full matrix
	REAL *pD = sD->pA+di+dj*ldd; // diagonal matrix
	REAL *dA = sA->dA+ai; // vectors of tau
	REAL *dD = sD->dA+di; // vectors of tau
	REAL alpha, beta, sigma, tmp, w0, w1;
	REAL *pA00, *pA10, *pD00, *pD10, *pD11, *pv0, *pv1;
	REAL pW[4] = {0.0, 0.0, 0.0, 0.0};
	int ldw = 2;
	REAL pT[4] = {0.0, 0.0, 0.0, 0.0};
	int ldb = 2;
	int jmax, kmax;
	ii = 0;
#if 1
	for(; ii<m-1; ii+=2)
		{
		// first row
		pD00 = &pD[ii+ldd*ii];
		pA00 = &pA[ii+lda*0];
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = pA00[0+lda*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau0
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pD00[0+ldd*0];
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			pD00[0+ldd*0] = beta;
			for(jj=0; jj<n1; jj++)
				pA00[0+lda*jj] *= tmp;
			}
		// gemv_t & ger
		pD10 = &pD00[1+ldd*0];
		pA10 = &pA00[1+lda*0];
		pv0 = &pA00[0+lda*0];
		w0 = 0.0;
		for(kk=0; kk<n1; kk++)
			{
			w0 += pA10[0+lda*kk] * pv0[0+lda*kk];
			}
		w0 = - dD[ii] * w0;
		pD10[0+ldd*0] = w0; // pv0[0] = 1.0
		for(kk=0; kk<n1; kk++)
			{
			pA10[0+lda*kk] += w0 * pv0[0+lda*kk];
			}
		// second row
		pD11 = &pD[(ii+1)+ldd*(ii+1)];
		pA10 = &pA[(ii+1)+lda*0];
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = pA10[0+lda*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau1
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = pD11[0+ldd*0];
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau1
			dD[ii+1] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v1
			pD11[0+lda*0] = beta;
			for(jj=0; jj<n1; jj++)
				pA10[0+lda*jj] *= tmp;
			}
		// compute lower triangular T containing tau for matrix update
		pv0 = &pA00[0+lda*0];
		pv1 = &pA00[1+lda*0];
		tmp = 0.0;
		for(kk=0; kk<n1; kk++)
			tmp += pv0[0+lda*kk]*pv1[0+lda*kk];
		pT[0+ldb*0] = dD[ii+0];
		pT[1+ldb*0] = - dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldb*1] = dD[ii+1];
		// downgrade
		jmax = m-ii-2;
		jj = 0;
#if 1
		for(; jj<jmax-1; jj+=2)
			{
			// compute W^T = C^T * V
			pW[0+ldw*0] = 0.0;
			pW[1+ldw*0] = 0.0;
			pW[0+ldw*1] = 0.0;
			pW[1+ldw*1] = 0.0;
			kk = 0;
			for(; kk<n1; kk++)
				{
				tmp = pA00[jj+0+2+ldd*kk];
				pW[0+ldw*0] += tmp * pv0[0+lda*kk];
				pW[0+ldw*1] += tmp * pv1[0+lda*kk];
				tmp = pA00[jj+1+2+ldd*kk];
				pW[1+ldw*0] += tmp * pv0[0+lda*kk];
				pW[1+ldw*1] += tmp * pv1[0+lda*kk];
				}
			// compute W^T *= T
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[1+ldw*1] = pT[1+ldb*0]*pW[1+ldw*0] + pT[1+ldb*1]*pW[1+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			pW[1+ldw*0] = pT[0+ldb*0]*pW[1+ldw*0];
			// compute C -= V * W^T
			pD00[jj+0+2+ldd*0] = - pW[0+ldw*0];
			pD00[jj+1+2+ldd*0] = - pW[1+ldw*0];
			pD00[jj+0+2+ldd*1] = - pW[0+ldw*1];
			pD00[jj+1+2+ldd*1] = - pW[1+ldw*1];
			kk = 0;
			for(; kk<n1-1; kk+=2)
				{
				pA00[jj+0+2+lda*(kk+0)] -= pv0[0+lda*(kk+0)]*pW[0+ldw*0] + pv1[0+lda*(kk+0)]*pW[0+ldw*1];
				pA00[jj+0+2+lda*(kk+1)] -= pv0[0+lda*(kk+1)]*pW[0+ldw*0] + pv1[0+lda*(kk+1)]*pW[0+ldw*1];
				pA00[jj+1+2+lda*(kk+0)] -= pv0[0+lda*(kk+0)]*pW[1+ldw*0] + pv1[0+lda*(kk+0)]*pW[1+ldw*1];
				pA00[jj+1+2+lda*(kk+1)] -= pv0[0+lda*(kk+1)]*pW[1+ldw*0] + pv1[0+lda*(kk+1)]*pW[1+ldw*1];
				}
			for(; kk<n1; kk++)
				{
				pA00[jj+0+2+lda*kk] -= pv0[0+lda*kk]*pW[0+ldw*0] + pv1[0+lda*kk]*pW[0+ldw*1];
				pA00[jj+1+2+lda*kk] -= pv0[0+lda*kk]*pW[1+ldw*0] + pv1[0+lda*kk]*pW[1+ldw*1];
				}
			}
#endif
		for(; jj<jmax; jj++)
			{
			// compute W = T * V^T * C
//			pW[0+ldw*0] = pC00[jj+0+2+ldd*0] + pC00[jj+0+2+ldd*1] * pv0[0+ldd*1];
//			pW[0+ldw*1] =                      pC00[jj+0+2+ldd*1];
			pW[0+ldw*0] = 0.0;
			pW[0+ldw*1] = 0.0;
			for(kk=0; kk<n1; kk++)
				{
				tmp = pA00[jj+0+2+ldd*kk];
				pW[0+ldw*0] += tmp * pv0[0+lda*kk];
				pW[0+ldw*1] += tmp * pv1[0+lda*kk];
				}
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			// compute C -= V * W^T
			pD00[jj+0+2+ldd*0] = - pW[0+ldw*0];
			pD00[jj+0+2+ldd*1] = - pW[0+ldw*1];
			for(kk=0; kk<n1; kk++)
				{
				pA00[jj+0+2+lda*kk] -= pv0[0+lda*kk]*pW[0+ldw*0] + pv1[0+lda*kk]*pW[0+ldw*1];
				}
			}
//		return;
		}
#endif
	for(; ii<m; ii++)
		{
		pD00 = &pD[ii+ldd*ii];
		pA00 = &pA[ii+lda*0];
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = pA00[0+lda*jj];
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = pD00[0+ldd*0];
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			pD00[0+ldd*0] = beta;
			for(jj=0; jj<n1; jj++)
				pA00[0+lda*jj] *= tmp;
			}
		// gemv_t & ger
		pD10 = &pD00[1+ldd*0];
		pA10 = &pA00[1+lda*0];
		pv0 = &pA00[0+lda*0];
		jmax = m-ii-1;
		jj = 0;
#if 1
		for(; jj<jmax-1; jj+=2)
			{
			w0 = 0.0;
			w1 = 0.0;
			for(kk=0; kk<n1; kk++)
				{
				w0 += pA10[jj+0+lda*kk] * pv0[0+lda*kk];
				w1 += pA10[jj+1+lda*kk] * pv0[0+lda*kk];
				}
			w0 = - dD[ii] * w0;
			w1 = - dD[ii] * w1;
			pD10[jj+0+ldd*0] = w0; // pv0[0] = 1.0
			pD10[jj+1+ldd*0] = w1; // pv0[0] = 1.0
			for(kk=0; kk<n1; kk++)
				{
				pA10[jj+0+lda*kk] += w0 * pv0[0+lda*kk];
				pA10[jj+1+lda*kk] += w1 * pv0[0+lda*kk];
				}
			}
#endif
		for(; jj<jmax; jj++)
			{
			w0 = 0.0;
			for(kk=0; kk<n1; kk++)
				{
				w0 += pA10[jj+lda*kk] * pv0[0+lda*kk];
				}
			w0 = - dD[ii] * w0;
			pD10[jj+ldd*0] = w0; // pv0[0] = 1.0
			for(kk=0; kk<n1; kk++)
				{
				pA10[jj+lda*kk] += w0 * pv0[0+lda*kk];
				}
			}
		}
	return;
	}
#endif



#if ! ( defined(REF_BLAS) )
// LQ factorization with positive diagonal elements, array of matrices
// [D, A] <= lq( [D. A] )
// D lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void REF_GELQF_PD_LA(int m, int n1, struct XMAT *sD, int di, int dj, struct XMAT *sA, int ai, int aj, void *work)
	{
	if(m<=0)
		return;
	
	// invalidate stored inverse diagonal of result matrix
	sA->use_dA = 0;
	sD->use_dA = 0;

	int ii, jj, kk;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldd = sD->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pD = sD->pA + di + dj*ldd;
	const int aai=0; const int aaj=0;
	const int ddi=0; const int ddj=0;
#else
	int aai=ai; int aaj=aj;
	int ddi=di; int ddj=dj;
#endif
	REAL *dA = sA->dA+ai; // vectors of tau
	REAL *dD = sD->dA+di; // vectors of tau
	REAL alpha, beta, sigma, tmp, w0, w1;
	REAL pW[4] = {0.0, 0.0, 0.0, 0.0};
	int ldw = 2;
	REAL pT[4] = {0.0, 0.0, 0.0, 0.0};
	int ldb = 2;
	int jmax, kmax;
	ii = 0;
#if 1
	for(; ii<m-1; ii+=2)
		{
		// first row
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = XMATEL_A(aai+ii+0, aaj+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau0
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=0; jj<n1; jj++)
				XMATEL_A(aai+ii+0, aaj+jj) *= tmp;
			}
		// gemv_t & ger
		w0 = XMATEL_D(ddi+ii+0+1, ddj+ii+0); // pvd0[0] = 1.0
		for(kk=0; kk<n1; kk++)
			{
			w0 += XMATEL_A(aai+ii+0+1, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
			}
		w0 = - dD[ii] * w0;
		XMATEL_D(ddi+ii+0+1, ddj+ii+0) += w0; // pv0[0] = 1.0
		for(kk=0; kk<n1; kk++)
			{
			XMATEL_A(aai+ii+0+1, aaj+kk) += w0 * XMATEL_A(aai+ii+0, aaj+kk);
			}
		// second row
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = XMATEL_A(aai+ii+1, aaj+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau1
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+1, ddj+ii+1);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau1
			dD[ii+1] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v1
			XMATEL_D(ddi+ii+1, ddj+ii+1) = beta;
			for(jj=0; jj<n1; jj++)
				XMATEL_A(aai+ii+1, aaj+jj) *= tmp;
			}
		// compute lower triangular T containing tau for matrix update
		tmp = 0.0;
		for(kk=0; kk<n1; kk++)
			tmp += XMATEL_A(aai+ii+0, aaj+kk)*XMATEL_A(aai+ii+1+0, aaj+kk);
		pT[0+ldb*0] = dD[ii+0];
		pT[1+ldb*0] = - dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldb*1] = dD[ii+1];
		// downgrade
		jmax = m-ii-2;
		jj = 0;
#if 1
		for(; jj<jmax-1; jj+=2)
			{
			// compute W^T = C^T * V
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0);
			pW[1+ldw*0] = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0);
			pW[0+ldw*1] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1);
			pW[1+ldw*1] = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1);
			kk = 0;
			for(; kk<n1; kk++)
				{
				tmp = XMATEL_A(aai+ii+jj+0+2, aaj+kk);
				pW[0+ldw*0] += tmp * XMATEL_A(aai+ii+0, aaj+kk);
				pW[0+ldw*1] += tmp * XMATEL_A(aai+ii+1+0, aaj+kk);
				tmp = XMATEL_A(aai+ii+jj+1+2, aaj+kk);
				pW[1+ldw*0] += tmp * XMATEL_A(aai+ii+0, aaj+kk);
				pW[1+ldw*1] += tmp * XMATEL_A(aai+ii+1+0, aaj+kk);
				}
			// compute W^T *= T
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[1+ldw*1] = pT[1+ldb*0]*pW[1+ldw*0] + pT[1+ldb*1]*pW[1+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			pW[1+ldw*0] = pT[0+ldb*0]*pW[1+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0) -= pW[1+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= pW[0+ldw*1];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1) -= pW[1+ldw*1];
			kk = 0;
			for(; kk<n1-1; kk+=2)
				{
				XMATEL_A(aai+ii+jj+0+2, aaj+(kk+0)) -= XMATEL_A(aai+ii+0, aaj+(kk+0))*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+0))*pW[0+ldw*1];
				XMATEL_A(aai+ii+jj+0+2, aaj+(kk+1)) -= XMATEL_A(aai+ii+0, aaj+(kk+1))*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+1))*pW[0+ldw*1];
				XMATEL_A(aai+ii+jj+1+2, aaj+(kk+0)) -= XMATEL_A(aai+ii+0, aaj+(kk+0))*pW[1+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+0))*pW[1+ldw*1];
				XMATEL_A(aai+ii+jj+1+2, aaj+(kk+1)) -= XMATEL_A(aai+ii+0, aaj+(kk+1))*pW[1+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+1))*pW[1+ldw*1];
				}
			for(; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+0+2, aaj+kk) -= XMATEL_A(aai+ii+0, aaj+kk)*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+kk)*pW[0+ldw*1];
				XMATEL_A(aai+ii+jj+1+2, aaj+kk) -= XMATEL_A(aai+ii+0, aaj+kk)*pW[1+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+kk)*pW[1+ldw*1];
				}
			}
#endif
		for(; jj<jmax; jj++)
			{
			// compute W = T * V^T * C
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0);
			pW[0+ldw*1] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1);
			for(kk=0; kk<n1; kk++)
				{
				tmp = XMATEL_A(aai+ii+jj+0+2, aaj+kk);
				pW[0+ldw*0] += tmp * XMATEL_A(aai+ii+0, aaj+kk);
				pW[0+ldw*1] += tmp * XMATEL_A(aai+ii+1+0, aaj+kk);
				}
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= pW[0+ldw*1];
			for(kk=0; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+0+2, aaj+kk) -= XMATEL_A(aai+ii+0, aaj+kk)*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+kk)*pW[0+ldw*1];
				}
			}
//		return;
		}
#endif
	for(; ii<m; ii++)
		{
		sigma = 0.0;
		for(jj=0; jj<n1; jj++)
			{
			tmp = XMATEL_A(aai+ii+0, aaj+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=0; jj<n1; jj++)
				XMATEL_A(aai+ii+0, aaj+jj) *= tmp;
			}
		// gemv_t & ger
		jmax = m-ii-1;
		jj = 0;
#if 1
		for(; jj<jmax-1; jj+=2)
			{
			w0 = XMATEL_D(ddi+ii+jj+1+0, ddj+ii+0); // pv0[0] = 1.0
			w1 = XMATEL_D(ddi+ii+jj+1+1, ddj+ii+0); // pv0[0] = 1.0
			for(kk=0; kk<n1; kk++)
				{
				w0 += XMATEL_A(aai+ii+jj+1+0, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
				w1 += XMATEL_A(aai+ii+jj+1+1, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
				}
			w0 = - dD[ii] * w0;
			w1 = - dD[ii] * w1;
			XMATEL_D(ddi+ii+jj+1+0, ddj+ii+0) += w0; // pv0[0] = 1.0
			XMATEL_D(ddi+ii+jj+1+1, ddj+ii+0) += w1; // pv0[0] = 1.0
			for(kk=0; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+1+0, aaj+kk) += w0 * XMATEL_A(aai+ii+0, aaj+kk);
				XMATEL_A(aai+ii+jj+1+1, aaj+kk) += w1 * XMATEL_A(aai+ii+0, aaj+kk);
				}
			}
#endif
		for(; jj<jmax; jj++)
			{
			w0 = XMATEL_D(ddi+ii+jj+1, ddj+ii+0); // pvd0[0] = 1.0
			for(kk=0; kk<n1; kk++)
				{
				w0 += XMATEL_A(aai+ii+jj+1, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
				}
			w0 = - dD[ii] * w0;
			XMATEL_D(ddi+ii+jj+1, ddj+ii+0) += w0; // pv0[0] = 1.0
			for(kk=0; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+1, aaj+kk) += w0 * XMATEL_A(aai+ii+0, aaj+kk);
				}
			}
		}
	return;
	}
#endif



#if ! ( defined(REF_BLAS) )
// LQ factorization with positive diagonal elements, array of matrices
// [D, L, A] <= lq( [D. L, A] )
// D, L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void REF_GELQF_PD_LLA(int m, int n1, struct XMAT *sD, int di, int dj, struct XMAT *sL, int li, int lj, struct XMAT *sA, int ai, int aj, void *work)
	{
	if(m<=0)
		return;
	
	// invalidate stored inverse diagonal of result matrix
	sA->use_dA = 0;
	sD->use_dA = 0;

	int ii, jj, kk;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldd = sD->m;
	int ldl = sL->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pD = sD->pA + di + dj*ldd;
	REAL *pL = sL->pA + li + lj*ldl;
	const int aai=0; const int aaj=0;
	const int ddi=0; const int ddj=0;
	const int lli=0; const int llj=0;
#else
	int aai=ai; int aaj=aj;
	int ddi=di; int ddj=dj;
	int lli=li; int llj=lj;
#endif
	REAL *dD = sD->dA+di; // vectors of tau
	REAL *dL = sL->dA+li; // vectors of tau
	REAL *dA = sA->dA+ai; // vectors of tau
	REAL alpha, beta, sigma, tmp, w0, w1;
	REAL pW[4] = {0.0, 0.0, 0.0, 0.0};
	int ldw = 2;
	REAL pT[4] = {0.0, 0.0, 0.0, 0.0};
	int ldb = 2;
	int jmax, kmax;
	ii = 0;
#if 1
	for(; ii<m-1; ii+=2)
		{
		// first row
		sigma = 0.0;
		for(jj=0; jj<=ii; jj++)
			{
			tmp = XMATEL_L(lli+ii+0, llj+jj);
			sigma += tmp*tmp;
			}
		for(jj=0; jj<n1; jj++)
			{
			tmp = XMATEL_A(aai+ii+0, aaj+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau0
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=0; jj<=ii; jj++)
				{
				XMATEL_L(lli+ii+0, llj+jj) *= tmp;
				}
			for(jj=0; jj<n1; jj++)
				{
				XMATEL_A(aai+ii+0, aaj+jj) *= tmp;
				}
			}
		// gemv_t & ger
		w0 = XMATEL_D(ddi+ii+0+1, ddj+ii+0); // pvd0[0] = 1.0
		for(kk=0; kk<=ii; kk++)
			{
			w0 += XMATEL_L(lli+ii+0+1, llj+kk) * XMATEL_L(lli+ii+0, llj+kk);
			}
		for(kk=0; kk<n1; kk++)
			{
			w0 += XMATEL_A(aai+ii+0+1, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
			}
		w0 = - dD[ii] * w0;
		XMATEL_D(ddi+ii+0+1, ddj+ii+0) += w0; // pv0[0] = 1.0
		for(kk=0; kk<=ii; kk++)
			{
			XMATEL_L(lli+ii+0+1, llj+kk) += w0 * XMATEL_L(lli+ii+0, llj+kk);
			}
		for(kk=0; kk<n1; kk++)
			{
			XMATEL_A(aai+ii+0+1, aaj+kk) += w0 * XMATEL_A(aai+ii+0, aaj+kk);
			}
		// second row
		sigma = 0.0;
		for(jj=0; jj<=ii+1; jj++)
			{
			tmp = XMATEL_L(lli+ii+1, llj+jj);
			sigma += tmp*tmp;
			}
		for(jj=0; jj<n1; jj++)
			{
			tmp = XMATEL_A(aai+ii+1, aaj+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			// tau1
			dD[(ii+1)] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+1, ddj+ii+1);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau1
			dD[ii+1] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v1
			XMATEL_D(ddi+ii+1, ddj+ii+1) = beta;
			for(jj=0; jj<=ii+1; jj++)
				{
				XMATEL_L(lli+ii+1, llj+jj) *= tmp;
				}
			for(jj=0; jj<n1; jj++)
				{
				XMATEL_A(aai+ii+1, aaj+jj) *= tmp;
				}
			}
		// compute lower triangular T containing tau for matrix update
		tmp = 0.0;
		for(kk=0; kk<=ii+2; kk++)
			{
			tmp += XMATEL_L(lli+ii+0, llj+kk)*XMATEL_L(lli+ii+1+0, llj+kk);
			}
		for(kk=0; kk<n1; kk++)
			{
			tmp += XMATEL_A(aai+ii+0, aaj+kk)*XMATEL_A(aai+ii+1+0, aaj+kk);
			}
		pT[0+ldb*0] = dD[ii+0];
		pT[1+ldb*0] = - dD[ii+1] * tmp * dD[ii+0];
		pT[1+ldb*1] = dD[ii+1];
		// downgrade
		jmax = m-ii-2;
		jj = 0;
#if 1
		for(; jj<jmax-1; jj+=2)
			{
			// compute W^T = C^T * V
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0);
			pW[1+ldw*0] = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0);
			pW[0+ldw*1] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1);
			pW[1+ldw*1] = XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1);
			kk = 0;
			for(; kk<=ii+2; kk++)
				{
				tmp = XMATEL_L(lli+ii+jj+0+2, llj+kk);
				pW[0+ldw*0] += tmp * XMATEL_L(lli+ii+0, llj+kk);
				pW[0+ldw*1] += tmp * XMATEL_L(lli+ii+1+0, llj+kk);
				tmp = XMATEL_L(lli+ii+jj+1+2, llj+kk);
				pW[1+ldw*0] += tmp * XMATEL_L(lli+ii+0, llj+kk);
				pW[1+ldw*1] += tmp * XMATEL_L(lli+ii+1+0, llj+kk);
				}
			kk = 0;
			for(; kk<n1; kk++)
				{
				tmp = XMATEL_A(aai+ii+jj+0+2, aaj+kk);
				pW[0+ldw*0] += tmp * XMATEL_A(aai+ii+0, aaj+kk);
				pW[0+ldw*1] += tmp * XMATEL_A(aai+ii+1+0, aaj+kk);
				tmp = XMATEL_A(aai+ii+jj+1+2, aaj+kk);
				pW[1+ldw*0] += tmp * XMATEL_A(aai+ii+0, aaj+kk);
				pW[1+ldw*1] += tmp * XMATEL_A(aai+ii+1+0, aaj+kk);
				}
			// compute W^T *= T
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[1+ldw*1] = pT[1+ldb*0]*pW[1+ldw*0] + pT[1+ldb*1]*pW[1+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			pW[1+ldw*0] = pT[0+ldb*0]*pW[1+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+0) -= pW[1+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= pW[0+ldw*1];
			XMATEL_D(ddi+ii+jj+1+2, ddj+ii+1) -= pW[1+ldw*1];
			kk = 0;
			for(; kk<=ii+2-1; kk+=2)
				{
				XMATEL_L(lli+ii+jj+0+2, llj+(kk+0)) -= XMATEL_L(lli+ii+0, llj+(kk+0))*pW[0+ldw*0] + XMATEL_L(lli+ii+1+0, llj+(kk+0))*pW[0+ldw*1];
				XMATEL_L(lli+ii+jj+0+2, llj+(kk+1)) -= XMATEL_L(lli+ii+0, llj+(kk+1))*pW[0+ldw*0] + XMATEL_L(lli+ii+1+0, llj+(kk+1))*pW[0+ldw*1];
				XMATEL_L(lli+ii+jj+1+2, llj+(kk+0)) -= XMATEL_L(lli+ii+0, llj+(kk+0))*pW[1+ldw*0] + XMATEL_L(lli+ii+1+0, llj+(kk+0))*pW[1+ldw*1];
				XMATEL_L(lli+ii+jj+1+2, llj+(kk+1)) -= XMATEL_L(lli+ii+0, llj+(kk+1))*pW[1+ldw*0] + XMATEL_L(lli+ii+1+0, llj+(kk+1))*pW[1+ldw*1];
				}
			for(; kk<=ii+2; kk++)
				{
				XMATEL_L(lli+ii+jj+0+2, llj+kk) -= XMATEL_L(lli+ii+0, llj+kk)*pW[0+ldw*0] + XMATEL_L(lli+ii+1+0, llj+kk)*pW[0+ldw*1];
				XMATEL_L(lli+ii+jj+1+2, llj+kk) -= XMATEL_L(lli+ii+0, llj+kk)*pW[1+ldw*0] + XMATEL_L(lli+ii+1+0, llj+kk)*pW[1+ldw*1];
				}
			kk = 0;
			for(; kk<n1-1; kk+=2)
				{
				XMATEL_A(aai+ii+jj+0+2, aaj+(kk+0)) -= XMATEL_A(aai+ii+0, aaj+(kk+0))*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+0))*pW[0+ldw*1];
				XMATEL_A(aai+ii+jj+0+2, aaj+(kk+1)) -= XMATEL_A(aai+ii+0, aaj+(kk+1))*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+1))*pW[0+ldw*1];
				XMATEL_A(aai+ii+jj+1+2, aaj+(kk+0)) -= XMATEL_A(aai+ii+0, aaj+(kk+0))*pW[1+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+0))*pW[1+ldw*1];
				XMATEL_A(aai+ii+jj+1+2, aaj+(kk+1)) -= XMATEL_A(aai+ii+0, aaj+(kk+1))*pW[1+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+(kk+1))*pW[1+ldw*1];
				}
			for(; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+0+2, aaj+kk) -= XMATEL_A(aai+ii+0, aaj+kk)*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+kk)*pW[0+ldw*1];
				XMATEL_A(aai+ii+jj+1+2, aaj+kk) -= XMATEL_A(aai+ii+0, aaj+kk)*pW[1+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+kk)*pW[1+ldw*1];
				}
			}
#endif
		for(; jj<jmax; jj++)
			{
			// compute W = T * V^T * C
			pW[0+ldw*0] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0);
			pW[0+ldw*1] = XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1);
			for(kk=0; kk<=ii+2; kk++)
				{
				tmp = XMATEL_L(lli+ii+jj+0+2, llj+kk);
				pW[0+ldw*0] += tmp * XMATEL_L(lli+ii+0, llj+kk);
				pW[0+ldw*1] += tmp * XMATEL_L(lli+ii+1+0, llj+kk);
				}
			for(kk=0; kk<n1; kk++)
				{
				tmp = XMATEL_A(aai+ii+jj+0+2, aaj+kk);
				pW[0+ldw*0] += tmp * XMATEL_A(aai+ii+0, aaj+kk);
				pW[0+ldw*1] += tmp * XMATEL_A(aai+ii+1+0, aaj+kk);
				}
			pW[0+ldw*1] = pT[1+ldb*0]*pW[0+ldw*0] + pT[1+ldb*1]*pW[0+ldw*1];
			pW[0+ldw*0] = pT[0+ldb*0]*pW[0+ldw*0];
			// compute C -= V * W^T
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+0) -= pW[0+ldw*0];
			XMATEL_D(ddi+ii+jj+0+2, ddj+ii+1) -= pW[0+ldw*1];
			for(kk=0; kk<=ii+2; kk++)
				{
				XMATEL_L(lli+ii+jj+0+2, llj+kk) -= XMATEL_L(lli+ii+0, llj+kk)*pW[0+ldw*0] + XMATEL_L(lli+ii+1+0, llj+kk)*pW[0+ldw*1];
				}
			for(kk=0; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+0+2, aaj+kk) -= XMATEL_A(aai+ii+0, aaj+kk)*pW[0+ldw*0] + XMATEL_A(aai+ii+1+0, aaj+kk)*pW[0+ldw*1];
				}
			}
//		return;
		}
#endif
	for(; ii<m; ii++)
		{
		sigma = 0.0;
		for(jj=0; jj<=ii; jj++)
			{
			tmp = XMATEL_L(lli+ii+0, llj+jj);
			sigma += tmp*tmp;
			}
		for(jj=0; jj<n1; jj++)
			{
			tmp = XMATEL_A(aai+ii+0, aaj+jj);
			sigma += tmp*tmp;
			}
		if(sigma==0.0)
			{
			dD[ii] = 0.0;
			}
		else
			{
			alpha = XMATEL_D(ddi+ii+0, ddj+ii+0);
			beta = sigma + alpha*alpha;
			beta = SQRT(beta);
			if(alpha<=0)
				tmp = alpha-beta;
			else
				tmp = -sigma/(alpha+beta);
			// tau0
			dD[ii] = 2*tmp*tmp/(sigma+tmp*tmp);
			tmp = 1.0/tmp;
			// compute v0
			XMATEL_D(ddi+ii+0, ddj+ii+0) = beta;
			for(jj=0; jj<=ii; jj++)
				{
				XMATEL_L(lli+ii+0, llj+jj) *= tmp;
				}
			for(jj=0; jj<n1; jj++)
				{
				XMATEL_A(aai+ii+0, aaj+jj) *= tmp;
				}
			}
		// gemv_t & ger
		jmax = m-ii-1;
		jj = 0;
#if 1
		for(; jj<jmax-1; jj+=2)
			{
			w0 = XMATEL_D(ddi+ii+jj+1+0, ddj+ii+0); // pv0[0] = 1.0
			w1 = XMATEL_D(ddi+ii+jj+1+1, ddj+ii+0); // pv0[0] = 1.0
			for(kk=0; kk<=ii; kk++)
				{
				w0 += XMATEL_L(lli+ii+jj+1+0, llj+kk) * XMATEL_L(lli+ii+0, llj+kk);
				w1 += XMATEL_L(lli+ii+jj+1+1, llj+kk) * XMATEL_L(lli+ii+0, llj+kk);
				}
			for(kk=0; kk<n1; kk++)
				{
				w0 += XMATEL_A(aai+ii+jj+1+0, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
				w1 += XMATEL_A(aai+ii+jj+1+1, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
				}
			w0 = - dD[ii] * w0;
			w1 = - dD[ii] * w1;
			XMATEL_D(ddi+ii+jj+1+0, ddj+ii+0) += w0; // pv0[0] = 1.0
			XMATEL_D(ddi+ii+jj+1+1, ddj+ii+0) += w1; // pv0[0] = 1.0
			for(kk=0; kk<=ii; kk++)
				{
				XMATEL_L(lli+ii+jj+1+0, llj+kk) += w0 * XMATEL_L(lli+ii+0, llj+kk);
				XMATEL_L(lli+ii+jj+1+1, llj+kk) += w1 * XMATEL_L(lli+ii+0, llj+kk);
				}
			for(kk=0; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+1+0, aaj+kk) += w0 * XMATEL_A(aai+ii+0, aaj+kk);
				XMATEL_A(aai+ii+jj+1+1, aaj+kk) += w1 * XMATEL_A(aai+ii+0, aaj+kk);
				}
			}
#endif
		for(; jj<jmax; jj++)
			{
			w0 = XMATEL_D(ddi+ii+jj+1, ddj+ii+0); // pvd0[0] = 1.0
			for(kk=0; kk<=ii; kk++)
				{
				w0 += XMATEL_L(lli+ii+jj+1, llj+kk) * XMATEL_L(lli+ii+0, llj+kk);
				}
			for(kk=0; kk<n1; kk++)
				{
				w0 += XMATEL_A(aai+ii+jj+1, aaj+kk) * XMATEL_A(aai+ii+0, aaj+kk);
				}
			w0 = - dD[ii] * w0;
			XMATEL_D(ddi+ii+jj+1, ddj+ii+0) += w0; // pv0[0] = 1.0
			for(kk=0; kk<=ii; kk++)
				{
				XMATEL_L(lli+ii+jj+1, llj+kk) += w0 * XMATEL_L(lli+ii+0, llj+kk);
				}
			for(kk=0; kk<n1; kk++)
				{
				XMATEL_A(aai+ii+jj+1, aaj+kk) += w0 * XMATEL_A(aai+ii+0, aaj+kk);
				}
			}
		}
	return;
	}
#endif



#if (defined(LA_REFERENCE) & defined(REF)) | (defined(LA_HIGH_PERFORMANCE) & defined(HP_CM)) | (defined(REF_BLAS))



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! ( defined(HP_CM) )
void POTRF_L(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_POTRF_L(m, sC, ci, cj, sD, di, dj);
	}
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! ( defined(HP_CM) )
#if ! ( defined(REF_BLAS) )
void POTRF_L_MN(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_POTRF_L_MN(m, n, sC, ci, cj, sD, di, dj);
	}
#endif
#endif



//#if ! ( defined(HP_CM) & defined(DP) )
#if ! ( defined(HP_CM) )
void POTRF_U(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_POTRF_U(m, sC, ci, cj, sD, di, dj);
	}
#endif


#if ! ( defined(REF_BLAS) )
void SYRK_POTRF_LN(int m, int k, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
#if ! ( defined(HP_CM) & defined(DP) )
	REF_SYRK_POTRF_LN(m, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
#else
	SYRK_LN(m, k, 1.0, sA, ai, aj, sB, bi, bj, 1.0, sC, ci, cj, sD, di, dj);
	POTRF_L(m, sD, di, dj, sD, di, dj);
#endif
	}
#endif



#if ! ( defined(REF_BLAS) )
void SYRK_POTRF_LN_MN(int m, int n, int k, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
#if ! ( defined(HP_CM) & defined(DP) )
	REF_SYRK_POTRF_LN_MN(m, n, k, sA, ai, aj, sB, bi, bj, sC, ci, cj, sD, di, dj);
#else
	SYRK_LN_MN(m, n, k, 1.0, sA, ai, aj, sB, bi, bj, 1.0, sC, ci, cj, sD, di, dj);
	POTRF_L_MN(m, n, sD, di, dj, sD, di, dj);
#endif
	}
#endif



#if ! ( defined(REF_BLAS) )
void PSTRF_L(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, int *ipiv)
	{
	REF_PSTRF_L(m, sC, ci, cj, sD, di, dj, ipiv);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
#if ! ( defined(REF_BLAS) )
void GETRF_NOPIVOT(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_GETRF_NOPIVOT(m, n, sC, ci, cj, sD, di, dj);
	}
#endif
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void GETRF_ROWPIVOT(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, int *ipiv)
	{
	REF_GETRF_ROWPIVOT(m, n, sC, ci, cj, sD, di, dj, ipiv);
	}
#endif



#if ! ( defined(REF_BLAS) )
int GEQRF_WORK_SIZE(int m, int n)
	{
	return REF_GEQRF_WORK_SIZE(m, n);
	}
#endif



#if ! ( defined(REF_BLAS) )
void GEQRF(int m, int n, struct XMAT *sA, int ai, int aj, struct XMAT *sD, int di, int dj, void *work)
	{
	REF_GEQRF(m, n, sA, ai, aj, sD, di, dj, work);
	}
#endif



#if ! ( defined(REF_BLAS) )
int GELQF_WORK_SIZE(int m, int n)
	{
	return REF_GELQF_WORK_SIZE(m, n);
	}
#endif



#if ! ( defined(REF_BLAS) )
void GELQF(int m, int n, struct XMAT *sA, int ai, int aj, struct XMAT *sD, int di, int dj, void *work)
	{
	REF_GELQF(m, n, sA, ai, aj, sD, di, dj, work);
	}
#endif



#if ! ( defined(REF_BLAS) )
int ORGLQ_WORK_SIZE(int m, int n, int k)
	{
	return REF_ORGLQ_WORK_SIZE(m, n, k);
	}
#endif



#if ! ( defined(REF_BLAS) )
void ORGLQ(int m, int n, int k, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, void *work)
	{
	REF_ORGLQ(m, n, k, sC, ci, cj, sD, di, dj, work);
	}
#endif



#if ! ( defined(REF_BLAS) )
void GELQF_PD(int m, int n, struct XMAT *sA, int ai, int aj, struct XMAT *sD, int di, int dj, void *work)
	{
	REF_GELQF_PD(m, n, sA, ai, aj, sD, di, dj, work);
	}
#endif



#if ! ( defined(REF_BLAS) )
void GELQF_PD_LA(int m, int n1, struct XMAT *sD, int di, int dj, struct XMAT *sA, int ai, int aj, void *work)
	{
	REF_GELQF_PD_LA(m, n1, sD, di, dj, sA, ai, aj, work);
	}
#endif



#if ! ( defined(REF_BLAS) )
void GELQF_PD_LLA(int m, int n1, struct XMAT *sD, int di, int dj, struct XMAT *sL, int li, int lj, struct XMAT *sA, int ai, int aj, void *work)
	{
	REF_GELQF_PD_LLA(m, n1, sD, di, dj, sL, li, lj, sA, ai, aj, work);
	}
#endif



#endif
