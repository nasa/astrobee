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



#if ! ( defined(HP_CM) & defined(DP) )
void REF_GEMV_N(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL 
		y_0, y_1, y_2, y_3,
		x_0, x_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
#if 1 // y reg version
	if(beta==0.0)
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = beta * y[ii];
			}
		}
	ii = 0;
	for(; ii<m-1; ii+=2)
		{
		y_0 = 0.0;
		y_1 = 0.0;
		jj = 0;
		for(; jj<n-1; jj+=2)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x[jj+0] + XMATEL_A(aai+ii+0, aaj+(jj+1)) * x[jj+1];
			y_1 += XMATEL_A(aai+ii+1, aaj+(jj+0)) * x[jj+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x[jj+1];
			}
		if(jj<n)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+jj) * x[jj];
			y_1 += XMATEL_A(aai+ii+1, aaj+jj) * x[jj];
			}
		z[ii+0] += alpha * y_0;
		z[ii+1] += alpha * y_1;
		}
	for(; ii<m; ii++)
		{
		y_0 = 0.0;
		for(jj=0; jj<n; jj++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[jj];
			}
		z[ii] += alpha * y_0;
		}
#else // x reg version
	if(beta==0.0)
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = beta * y[ii];
			}
		}
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		x_0 = alpha * x[jj+0];
		x_1 = alpha * x[jj+1];
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			z[ii+0] += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii+0, aaj+(jj+1)) * x_1;
			z[ii+1] += XMATEL_A(aai+ii+1, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x_1;
			}
		for(; ii<m; ii++)
			{
			z[ii] += XMATEL_A(aai+ii, aaj+(jj+0)) * x_0;
			z[ii] += XMATEL_A(aai+ii, aaj+(jj+1)) * x_1;
			}
		}
	for(; jj<n; jj++)
		{
		x_0 = alpha * x[jj+0];
		for(ii=0; ii<m; ii++)
			{
			z[ii] += XMATEL_A(aai+ii, aaj+(jj+0)) * x_0;
			}
		}
#endif
	return;
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void REF_GEMV_T(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL 
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	if(beta==0.0)
		{
		for(ii=0; ii<n; ii++)
			{
			z[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<n; ii++)
			{
			z[ii] = beta * y[ii];
			}
		}
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		y_0 = 0.0;
		y_1 = 0.0;
		ii = 0;
		for(; ii<m-1; ii+=2)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+0)) * x[ii+1];
			y_1 += XMATEL_A(aai+ii+0, aaj+(jj+1)) * x[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x[ii+1];
			}
		if(ii<m)
			{
			y_0 += XMATEL_A(aai+ii, aaj+(jj+0)) * x[ii];
			y_1 += XMATEL_A(aai+ii, aaj+(jj+1)) * x[ii];
			}
		z[jj+0] += alpha * y_0;
		z[jj+1] += alpha * y_1;
		}
	for(; jj<n; jj++)
		{
		y_0 = 0.0;
		for(ii=0; ii<m; ii++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+(jj+0)) * x[ii];
			}
		z[jj+0] += alpha * y_0;
		}
	return;
	}
#endif



// TODO optimize !!!!!
void REF_GEMV_NT(int m, int n, REAL alpha_n, REAL alpha_t, struct XMAT *sA, int ai, int aj, struct XVEC *sx_n, int xi_n, struct XVEC *sx_t, int xi_t, REAL beta_n, REAL beta_t, struct XVEC *sy_n, int yi_n, struct XVEC *sy_t, int yi_t, struct XVEC *sz_n, int zi_n, struct XVEC *sz_t, int zi_t)
	{
	int ii, jj;
	REAL
		a_00,
		x_n_0,
		y_t_0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x_n = sx_n->pa + xi_n;
	REAL *x_t = sx_t->pa + xi_t;
	REAL *y_n = sy_n->pa + yi_n;
	REAL *y_t = sy_t->pa + yi_t;
	REAL *z_n = sz_n->pa + zi_n;
	REAL *z_t = sz_t->pa + zi_t;
	if(beta_n==0.0)
		{
		for(ii=0; ii<m; ii++)
			{
			z_n[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			{
			z_n[ii] = beta_n * y_n[ii];
			}
		}
	if(beta_t==0.0)
		{
		for(ii=0; ii<n; ii++)
			{
			z_t[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			{
			z_t[ii] = beta_t * y_t[ii];
			}
		}
	for(jj=0; jj<n; jj++)
		{
		y_t_0 = 0.0;
		x_n_0 = alpha_n * x_n[jj];
		for(ii=0; ii<m; ii++)
			{
			a_00 = XMATEL_A(aai+ii, aaj+jj);
			z_n[ii] += a_00 * x_n_0;
			y_t_0 += a_00 * x_t[ii];
			}
		z_t[jj] += alpha_t * y_t_0;
		}
	return;
	}



#if ! ( defined(HP_CM) & defined(DP) )
void REF_SYMV_L(int m, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	if(beta==0.0)
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = beta * y[ii];
			}
		}
	ii = 0;
	for(; ii<m-1; ii+=2)
		{
		y_0 = 0.0;
		y_1 = 0.0;
		jj = 0;
		// left block
		for(; jj<ii-1; jj+=2)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+jj+0) * x[jj+0] + XMATEL_A(aai+ii+0, aaj+jj+1) * x[jj+1];
			y_1 += XMATEL_A(aai+ii+1, aaj+jj+0) * x[jj+0] + XMATEL_A(aai+ii+1, aaj+jj+1) * x[jj+1];
			}
		for(; jj<ii; jj++)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+jj) * x[jj];
			y_1 += XMATEL_A(aai+ii+1, aaj+jj) * x[jj];
			}
		// diagonal block
		y_0 += XMATEL_A(aai+ii+0, aaj+jj+0) * x[jj+0] + XMATEL_A(aai+ii+1, aaj+jj+0) * x[jj+1];
		y_1 += XMATEL_A(aai+ii+1, aaj+jj+0) * x[jj+0] + XMATEL_A(aai+ii+1, aaj+jj+1) * x[jj+1];
		jj += 2;
		// bottom block
		for( ; jj<m-1; jj+=2)
			{
			y_0 += XMATEL_A(aai+jj+0, aaj+ii+0) * x[jj+0] + XMATEL_A(aai+jj+1, aaj+ii+0) * x[jj+1];
			y_1 += XMATEL_A(aai+jj+0, aaj+ii+1) * x[jj+0] + XMATEL_A(aai+jj+1, aaj+ii+1) * x[jj+1];
			}
		for( ; jj<m; jj++)
			{
			y_0 += XMATEL_A(aai+jj, aaj+ii+0) * x[jj];
			y_1 += XMATEL_A(aai+jj, aaj+ii+1) * x[jj];
			}
		z[ii+0] += alpha * y_0;
		z[ii+1] += alpha * y_1;
		}
	for(; ii<m; ii++)
		{
		y_0 = 0.0;
		jj = 0;
		for(; jj<=ii; jj++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[jj];
			}
		for( ; jj<m; jj++)
			{
			y_0 += XMATEL_A(aai+jj, aaj+ii) * x[jj];
			}
		z[ii] += alpha * y_0;
		}
	return;
	}
#endif



void REF_SYMV_L_MN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL
		y_0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	if(beta==0.0)
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = beta * y[ii];
			}
		}
	ii = 0;
	for(; ii<n; ii++)
		{
		y_0 = 0.0;
		jj = 0;
		for(; jj<=ii; jj++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[jj];
			}
		for( ; jj<m; jj++)
			{
			y_0 += XMATEL_A(aai+jj, aaj+ii) * x[jj];
			}
		z[ii] += alpha * y_0;
		}
	for(; ii<m; ii++)
		{
		y_0 = 0.0;
		jj = 0;
		for(; jj<n; jj++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[jj];
			}
		z[ii] += alpha * y_0;
		}
	return;
	}



#if ! ( defined(HP_CM) & defined(DP) )
void REF_SYMV_U(int m, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	if(beta==0.0)
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = 0.0;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			{
			z[ii] = beta * y[ii];
			}
		}
	ii = 0;
	for(; ii<m; ii++)
		{
		y_0 = 0.0;
		jj = 0;
		for(; jj<=ii; jj++)
			{
			y_0 += XMATEL_A(aai+jj, aaj+ii) * x[jj];
			}
		for( ; jj<m; jj++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[jj];
			}
		z[ii] += alpha * y_0;
		}
	return;
	}
#endif



static void REF_TRMV_LNN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(m-n>0)
		{
		REF_GEMV_N(m-n, n, 1.0, sA, ai+n, aj, sx, xi, 0.0, sz, zi+n, sz, zi+n);
		}
	if(n%2!=0)
		{
		ii = n-1;
		y_0 = x[ii];
		y_0 *= XMATEL_A(aai+ii, aaj+ii);
		for(jj=0; jj<ii; jj++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[jj];
			}
		z[ii] = y_0;
		n -= 1;
		}
	for(ii=n-2; ii>=0; ii-=2)
		{
		y_0 = x[ii+0];
		y_1 = x[ii+1];
		y_1 *= XMATEL_A(aai+ii+1, aaj+(ii+1));
		y_1 += XMATEL_A(aai+ii+1, aaj+(ii+0)) * y_0;
		y_0 *= XMATEL_A(aai+ii+0, aaj+(ii+0));
		jj = 0;
		for(; jj<ii-1; jj+=2)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x[jj+0] + XMATEL_A(aai+ii+0, aaj+(jj+1)) * x[jj+1];
			y_1 += XMATEL_A(aai+ii+1, aaj+(jj+0)) * x[jj+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x[jj+1];
			}
//	XXX there is no clean up loop !!!!!
//		for(; jj<ii; jj++)
//			{
//			y_0 += XMATEL_A(aai+ii+0, aaj+jj) * x[jj];
//			y_1 += XMATEL_A(aai+ii+1, aaj+jj) * x[jj];
//			}
		z[ii+0] = y_0;
		z[ii+1] = y_1;
		}
	return;
	}


	
void REF_TRMV_LNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRMV_LNN_MN(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



static void REF_TRMV_LTN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		y_0 = x[jj+0];
		y_1 = x[jj+1];
		y_0 *= XMATEL_A(aai+jj+0, aaj+(jj+0));
		y_0 += XMATEL_A(aai+jj+1, aaj+(jj+0)) * y_1;
		y_1 *= XMATEL_A(aai+jj+1, aaj+(jj+1));
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+0)) * x[ii+1];
			y_1 += XMATEL_A(aai+ii+0, aaj+(jj+1)) * x[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x[ii+1];
			}
		for(; ii<m; ii++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+(jj+0)) * x[ii];
			y_1 += XMATEL_A(aai+ii, aaj+(jj+1)) * x[ii];
			}
		z[jj+0] = y_0;
		z[jj+1] = y_1;
		}
	for(; jj<n; jj++)
		{
		y_0 = x[jj];
		y_0 *= XMATEL_A(aai+jj, aaj+jj);
		for(ii=jj+1; ii<m; ii++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[ii];
			}
		z[jj] = y_0;
		}
	return;
	}



void REF_TRMV_LTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRMV_LTN_MN(m, m, sA, ai, aj, sx, xi, sz, zi);
	return;
	}



void REF_TRMV_UNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL
		y_0, y_1,
		x_0, x_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
#if 1 // y reg version
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		y_0 = x[jj+0];
		y_1 = x[jj+1];
		y_0 = XMATEL_A(aai+jj+0, aaj+(jj+0)) * y_0;
		y_0 += XMATEL_A(aai+jj+0, aaj+(jj+1)) * y_1;
		y_1 = XMATEL_A(aai+jj+1, aaj+(jj+1)) * y_1;
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			y_0 += XMATEL_A(aai+jj+0, aaj+(ii+0)) * x[ii+0] + XMATEL_A(aai+jj+0, aaj+(ii+1)) * x[ii+1];
			y_1 += XMATEL_A(aai+jj+1, aaj+(ii+0)) * x[ii+0] + XMATEL_A(aai+jj+1, aaj+(ii+1)) * x[ii+1];
			}
		if(ii<m)
			{
			y_0 += XMATEL_A(aai+jj+0, aaj+(ii+0)) * x[ii+0];
			y_1 += XMATEL_A(aai+jj+1, aaj+(ii+0)) * x[ii+0];
			}
		z[jj+0] = y_0;
		z[jj+1] = y_1;
		}
	for(; jj<m; jj++)
		{
		y_0 = XMATEL_A(aai+jj, aaj+jj) * x[jj];
		for(ii=jj+1; ii<m; ii++)
			{
			y_0 += XMATEL_A(aai+jj, aaj+ii) * x[ii];
			}
		z[jj] = y_0;
		}
#else // x reg version
	if(x != z)
		{
		for(ii=0; ii<m; ii++)
			z[ii] = x[ii];
		}
	jj = 0;
	for(; jj<m-1; jj+=2)
		{
		x_0 = z[jj+0];
		x_1 = z[jj+1];
		ii = 0;
		for(; ii<jj-1; ii+=2)
			{
			z[ii+0] += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii+0, aaj+(jj+1)) * x_1;
			z[ii+1] += XMATEL_A(aai+ii+1, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x_1;
			}
//	XXX there is no clean-up loop, since jj+=2 !!!!!
//		for(; ii<jj; ii++)
//			{
//			z[ii+0] += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii+0, aaj+(jj+1)) * x_1;
//			}
		x_0 *= XMATEL_A(aai+jj+0, aaj+(jj+0));
		x_0 += XMATEL_A(aai+jj+0, aaj+(jj+1)) * x_1;
		x_1 *= XMATEL_A(aai+jj+1, aaj+(jj+1));
		z[jj+0] = x_0;
		z[jj+1] = x_1;
		}
	for(; jj<m; jj++)
		{
		x_0 = z[jj];
		for(ii=0; ii<jj; ii++)
			{
			z[ii] += XMATEL_A(aai+ii, aaj+jj) * x_0;
			}
		x_0 *= XMATEL_A(aai+jj, aaj+jj);
		z[jj] = x_0;
		}
#endif
	return;
	}



void REF_TRMV_UTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(m%2!=0)
		{
		jj = m-1;
		y_0 = XMATEL_A(aai+jj, aaj+jj) * x[jj];
		for(ii=0; ii<jj; ii++)
			{
			y_0 += XMATEL_A(aai+ii, aaj+jj) * x[ii];
			}
		z[jj] = y_0;
		m -= 1; // XXX
		}
	for(jj=m-2; jj>=0; jj-=2)
		{
		y_1 = XMATEL_A(aai+jj+1, aaj+(jj+1)) * x[jj+1];
		y_1 += XMATEL_A(aai+jj+0, aaj+(jj+1)) * x[jj+0];
		y_0 = XMATEL_A(aai+jj+0, aaj+(jj+0)) * x[jj+0];
		for(ii=0; ii<jj-1; ii+=2)
			{
			y_0 += XMATEL_A(aai+ii+0, aaj+(jj+0)) * x[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+0)) * x[ii+1];
			y_1 += XMATEL_A(aai+ii+0, aaj+(jj+1)) * x[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x[ii+1];
			}
//	XXX there is no clean-up loop !!!!!
//		if(ii<jj)
//			{
//			y_0 += XMATEL_A(aai+ii, aaj+(jj+0)) * x[ii];
//			y_1 += XMATEL_A(aai+ii, aaj+(jj+1)) * x[ii];
//			}
		z[jj+0] = y_0;
		z[jj+1] = y_1;
		}
	return;
	}



void REF_TRSV_LNN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj, j1;
	REAL
		y_0, y_1,
		x_0, x_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *dA = sA->dA;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = 1;
			}
		}
	else
		{
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0;
		}
#if 1 // y reg version
	ii = 0;
	for(; ii<n-1; ii+=2)
		{
		y_0 = x[ii+0];
		y_1 = x[ii+1];
		jj = 0;
		for(; jj<ii-1; jj+=2)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+0, aaj+(jj+1)) * z[jj+1];
			y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * z[jj+1];
			}
//	XXX there is no clean-up loop !!!!!
//		if(jj<ii)
//			{
//			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[jj+0];
//			y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[jj+0];
//			}
		y_0 *= dA[ii+0];
		y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * y_0;
		y_1 *= dA[ii+1];
		z[ii+0] = y_0;
		z[ii+1] = y_1;
		}
	for(; ii<n; ii++)
		{
		y_0 = x[ii];
		for(jj=0; jj<ii; jj++)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+jj) * z[jj];
			}
		y_0 *= dA[ii];
		z[ii] = y_0;
		}
	for(; ii<m-1; ii+=2)
		{
		y_0 = x[ii+0];
		y_1 = x[ii+1];
		jj = 0;
		for(; jj<n-1; jj+=2)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+0, aaj+(jj+1)) * z[jj+1];
			y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * z[jj+1];
			}
		if(jj<n)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[jj+0];
			y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[jj+0];
			}
		z[ii+0] = y_0;
		z[ii+1] = y_1;
		}
	for(; ii<m; ii++)
		{
		y_0 = x[ii];
		for(jj=0; jj<n; jj++)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+jj) * z[jj];
			}
		z[ii] = y_0;
		}
#else // x reg version
	if(x != z)
		{
		for(ii=0; ii<m; ii++)
			z[ii] = x[ii];
		}
	jj = 0;
	for(; jj<n-1; jj+=2)
		{
		x_0 = dA[jj+0] * z[jj+0];
		x_1 = z[jj+1] - XMATEL_A(aai+jj+1, aaj+(jj+0)) * x_0;
		x_1 = dA[jj+1] * x_1;
		z[jj+0] = x_0;
		z[jj+1] = x_1;
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			z[ii+0] -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii+0, aaj+(jj+1)) * x_1;
			z[ii+1] -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii+1, aaj+(jj+1)) * x_1;
			}
		for(; ii<m; ii++)
			{
			z[ii] -= XMATEL_A(aai+ii, aaj+(jj+0)) * x_0 + XMATEL_A(aai+ii, aaj+(jj+1)) * x_1;
			}
		}
	for(; jj<n; jj++)
		{
		x_0 = dA[jj] * z[jj];
		z[jj] = x_0;
		for(ii=jj+1; ii<m; ii++)
			{
			z[ii] -= XMATEL_A(aai+ii, aaj+jj) * x_0;
			}
		}
#endif
	return;
	}



void REF_TRSV_LTN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *dA = sA->dA;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			for(ii=0; ii<n; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = 1;
			}
		}
	else
		{
		for(ii=0; ii<n; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0;
		}
	if(x!=z)
		for(ii=0; ii<m; ii++)
			z[ii] = x[ii];
	if(n%2!=0)
		{
		jj = n-1;
		y_0 = z[jj];
		for(ii=jj+1; ii<m; ii++)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+jj) * z[ii];
			}
		y_0 *= dA[jj];
		z[jj] = y_0;
		jj -= 2;
		}
	else
		{
		jj = n-2;
		}
	for(; jj>=0; jj-=2)
		{
		y_0 = z[jj+0];
		y_1 = z[jj+1];
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[ii+1];
			y_1 -= XMATEL_A(aai+ii+0, aaj+(jj+1)) * z[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * z[ii+1];
			}
		if(ii<m)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+(jj+0)) * z[ii];
			y_1 -= XMATEL_A(aai+ii, aaj+(jj+1)) * z[ii];
			}
		y_1 *= dA[jj+1];
		y_0 -= XMATEL_A(aai+jj+1, aaj+(jj+0)) * y_1;
		y_0 *= dA[jj+0];
		z[jj+0] = y_0;
		z[jj+1] = y_1;
		}
	return;
	}



void REF_TRSV_LNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj, j1;
	REAL
		y_0, y_1,
		x_0, x_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *dA = sA->dA;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = 1;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0;
		}
	ii = 0;
	for(; ii<m-1; ii+=2)
		{
		y_0 = x[ii+0];
		y_1 = x[ii+1];
		jj = 0;
		for(; jj<ii-1; jj+=2)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+0, aaj+(jj+1)) * z[jj+1];
			y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * z[jj+1];
			}
		y_0 *= dA[ii+0];
		y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * y_0;
		y_1 *= dA[ii+1];
		z[ii+0] = y_0;
		z[ii+1] = y_1;
		}
	for(; ii<m; ii++)
		{
		y_0 = x[ii];
		for(jj=0; jj<ii; jj++)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+jj) * z[jj];
			}
		y_0 *= dA[ii];
		z[ii] = y_0;
		}
	return;
	}



void REF_TRSV_LNU(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj, j1;
	REAL
		y_0, y_1,
		x_0, x_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	ii = 0;
	for(; ii<m-1; ii+=2)
		{
		y_0 = x[ii+0];
		y_1 = x[ii+1];
		jj = 0;
		for(; jj<ii-1; jj+=2)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+0, aaj+(jj+1)) * z[jj+1];
			y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[jj+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * z[jj+1];
			}
		y_1 -= XMATEL_A(aai+ii+1, aaj+(jj+0)) * y_0;
		z[ii+0] = y_0;
		z[ii+1] = y_1;
		}
	for(; ii<m; ii++)
		{
		y_0 = x[ii];
		for(jj=0; jj<ii; jj++)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+jj) * z[jj];
			}
		z[ii] = y_0;
		}
	return;
	}



void REF_TRSV_LTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *dA = sA->dA;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = 1;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0;
		}
	if(m%2!=0)
		{
		jj = m-1;
		y_0 = x[jj];
		y_0 *= dA[jj];
		z[jj] = y_0;
		jj -= 2;
		}
	else
		{
		jj = m-2;
		}
	for(; jj>=0; jj-=2)
		{
		y_0 = x[jj+0];
		y_1 = x[jj+1];
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[ii+1];
			y_1 -= XMATEL_A(aai+ii+0, aaj+(jj+1)) * z[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * z[ii+1];
			}
		if(ii<m)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+(jj+0)) * z[ii];
			y_1 -= XMATEL_A(aai+ii, aaj+(jj+1)) * z[ii];
			}
		y_1 *= dA[jj+1];
		y_0 -= XMATEL_A(aai+jj+1, aaj+(jj+0)) * y_1;
		y_0 *= dA[jj+0];
		z[jj+0] = y_0;
		z[jj+1] = y_1;
		}
	return;
	}



void REF_TRSV_LTU(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(m%2!=0)
		{
		jj = m-1;
		y_0 = x[jj];
		z[jj] = y_0;
		jj -= 2;
		}
	else
		{
		jj = m-2;
		}
	for(; jj>=0; jj-=2)
		{
		y_0 = x[jj+0];
		y_1 = x[jj+1];
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			y_0 -= XMATEL_A(aai+ii+0, aaj+(jj+0)) * z[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+0)) * z[ii+1];
			y_1 -= XMATEL_A(aai+ii+0, aaj+(jj+1)) * z[ii+0] + XMATEL_A(aai+ii+1, aaj+(jj+1)) * z[ii+1];
			}
		if(ii<m)
			{
			y_0 -= XMATEL_A(aai+ii, aaj+(jj+0)) * z[ii];
			y_1 -= XMATEL_A(aai+ii, aaj+(jj+1)) * z[ii];
			}
		y_0 -= XMATEL_A(aai+jj+1, aaj+(jj+0)) * y_1;
		z[jj+0] = y_0;
		z[jj+1] = y_1;
		}
	return;
	}



void REF_TRSV_UNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj;
	REAL
		y_0, y_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *dA = sA->dA;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = 1;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0;
		}
	if(m%2!=0)
		{
		jj = m-1;
		y_0 = x[jj];
		y_0 *= dA[jj];
		z[jj] = y_0;
		jj -= 2;
		}
	else
		{
		jj = m-2;
		}
	for(; jj>=0; jj-=2)
		{
		y_0 = x[jj+0];
		y_1 = x[jj+1];
		ii = jj+2;
		for(; ii<m-1; ii+=2)
			{
			y_0 -= XMATEL_A(aai+jj+0, aaj+(ii+0)) * z[ii+0] + XMATEL_A(aai+jj+0, aaj+(ii+1)) * z[ii+1];
			y_1 -= XMATEL_A(aai+jj+1, aaj+(ii+0)) * z[ii+0] + XMATEL_A(aai+jj+1, aaj+(ii+1)) * z[ii+1];
			}
		if(ii<m)
			{
			y_0 -= XMATEL_A(aai+jj+0, aaj+(ii+0)) * z[ii];
			y_1 -= XMATEL_A(aai+jj+1, aaj+(ii+0)) * z[ii];
			}
		y_1 *= dA[jj+1];
		y_0 -= XMATEL_A(aai+jj+0, aaj+(jj+1)) * y_1;
		y_0 *= dA[jj+0];
		z[jj+0] = y_0;
		z[jj+1] = y_1;
		}
	return;
	}



void REF_TRSV_UTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
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
	int ii, jj, j1;
	REAL
		y_0, y_1,
		x_0, x_1;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *dA = sA->dA;
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	if(ai==0 & aj==0)
		{
		if(sA->use_dA!=1)
			{
			for(ii=0; ii<m; ii++)
				dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
			sA->use_dA = 1;
			}
		}
	else
		{
		for(ii=0; ii<m; ii++)
			dA[ii] = 1.0 / XMATEL_A(aai+ii, aaj+ii);
		sA->use_dA = 0;
		}
	ii = 0;
	for(; ii<m-1; ii+=2)
		{
		y_0 = x[ii+0];
		y_1 = x[ii+1];
		jj = 0;
		for(; jj<ii-1; jj+=2)
			{
			y_0 -= XMATEL_A(aai+jj+0, aaj+(ii+0)) * z[jj+0] + XMATEL_A(aai+jj+1, aaj+(ii+0)) * z[jj+1];
			y_1 -= XMATEL_A(aai+jj+0, aaj+(ii+1)) * z[jj+0] + XMATEL_A(aai+jj+1, aaj+(ii+1)) * z[jj+1];
			}
		y_0 *= dA[ii+0];
		y_1 -= XMATEL_A(aai+jj+0, aaj+(ii+1)) * y_0;
		y_1 *= dA[ii+1];
		z[ii+0] = y_0;
		z[ii+1] = y_1;
		}
	for(; ii<m; ii++)
		{
		y_0 = x[ii];
		for(jj=0; jj<ii; jj++)
			{
			y_0 -= XMATEL_A(aai+jj, aaj+ii) * z[jj];
			}
		y_0 *= dA[ii];
		z[ii] = y_0;
		}
	return;
	}



#if ! ( defined(HP_CM) & defined(DP) )
void REF_GER(int m, int n, REAL alpha, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	int ii, jj;
	REAL
		c_00,
		x_0,
		y_0, y_1;
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
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	jj = 0;
#if 1
	for(; jj<n-1; jj+=2)
		{
		y_0 = alpha * y[jj+0];
		y_1 = alpha * y[jj+1];
		ii = 0;
		for(; ii<m; ii++)
			{
			x_0 = x[ii];
			XMATEL_D(ddi+ii, ddj+jj+0) = XMATEL_C(cci+ii, ccj+jj+0) + x_0 * y_0;
			XMATEL_D(ddi+ii, ddj+jj+1) = XMATEL_C(cci+ii, ccj+jj+1) + x_0 * y_1;
			}
		}
#endif
	for(; jj<n; jj++)
		{
		y_0 = alpha * y[jj];
		ii = 0;
		for(; ii<m; ii++)
			{
			x_0 = x[ii];
			XMATEL_D(ddi+ii, ddj+jj) = XMATEL_C(cci+ii, ccj+jj) + x_0 * y_0;
			}
		}
	return;
	}
#endif



#if (defined(LA_REFERENCE) & defined(REF)) | (defined(LA_HIGH_PERFORMANCE) & defined(HP_CM))



#if ! ( defined(HP_CM) & defined(DP) )
void GEMV_N(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_GEMV_N(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}
#endif



#if ! ( defined(HP_CM) & defined(DP) )
void GEMV_T(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_GEMV_T(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}
#endif



void GEMV_NT(int m, int n, REAL alpha_n, REAL alpha_t, struct XMAT *sA, int ai, int aj, struct XVEC *sx_n, int xi_n, struct XVEC *sx_t, int xi_t, REAL beta_n, REAL beta_t, struct XVEC *sy_n, int yi_n, struct XVEC *sy_t, int yi_t, struct XVEC *sz_n, int zi_n, struct XVEC *sz_t, int zi_t)
	{
	REF_GEMV_NT(m, n, alpha_n, alpha_t, sA, ai, aj, sx_n, xi_n, sx_t, xi_t, beta_n, beta_t, sy_n, yi_n, sy_t, yi_t, sz_n, zi_n, sz_t, zi_t);
	}



#if ! ( defined(HP_CM) & defined(DP) )
void SYMV_L(int m, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_SYMV_L(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}
#endif



void SYMV_L_MN(int m, int n, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_SYMV_L_MN(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



#if ! ( defined(HP_CM) & defined(DP) )
void SYMV_U(int m, REAL alpha, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_SYMV_U(m, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}
#endif



void TRMV_LNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRMV_LNN(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRMV_LTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRMV_LTN(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRMV_UNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRMV_UNN(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRMV_UTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRMV_UTN(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_LNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_LNN(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_LNN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_LNN_MN(m, n, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_LNU(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_LNU(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_LTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_LTN(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_LTN_MN(int m, int n, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_LTN_MN(m, n, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_LTU(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_LTU(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_UNN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_UNN(m, sA, ai, aj, sx, xi, sz, zi);
	}



void TRSV_UTN(int m, struct XMAT *sA, int ai, int aj, struct XVEC *sx, int xi, struct XVEC *sz, int zi)
	{
	REF_TRSV_UTN(m, sA, ai, aj, sx, xi, sz, zi);
	}



#if ! ( defined(HP_CM) & defined(DP) )
void GER(int m, int n, REAL alpha, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	REF_GER(m, n, alpha, sx, xi, sy, yi, sC, ci, cj, sD, di, dj);
	}
#endif



#endif
