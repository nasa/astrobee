/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
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

//#include "../include/blasfeo_d_aux.h"


// return memory size (in bytes) needed for a strmat
size_t REF_MEMSIZE_MAT(int m, int n)
	{
#if defined(MF_COLMAJ)
	int tmp = m<n ? m : n; // al(min(m,n)) // XXX max ???
	size_t memsize = (m*n+tmp)*sizeof(REAL);
#else // MF_PANELMAJ
	const int bs = PS;
	const int nc = PLD;
	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	int cn = (n+nc-1)/nc*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = (pm*cn+tmp)*sizeof(REAL);
#endif
	memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	return memsize;
	}



// return memory size (in bytes) needed for the diagonal of a strmat
size_t REF_MEMSIZE_DIAG_MAT(int m, int n)
	{
#if defined(MF_COLMAJ)
	int tmp = m<n ? m : n; // al(min(m,n)) // XXX max ???
	size_t size = tmp*sizeof(REAL);
#else // MF_PANELMAJ
	const int bs = PS;
	const int nc = PLD;
	const int al = bs*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t size = tmp*sizeof(REAL);
#endif
	return size;
	}



// return memory size (in bytes) needed for a strvec
size_t REF_MEMSIZE_VEC(int m)
	{
#if defined(MF_COLMAJ)
	size_t size = m*sizeof(REAL);
#else // MF_PANELMAJ
	const int bs = PS;
//	const int nc = PLD;
//	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	size_t size = pm*sizeof(REAL);
#endif
	return size;
	}



// create a matrix structure for a matrix of size m*n by using memory passed by a pointer
void REF_CREATE_MAT(int m, int n, struct MAT *sA, void *memory)
	{
	sA->mem = memory;
	sA->m = m;
	sA->n = n;
	sA->use_dA = 0; // invalidate stored inverse diagonal
#if defined(MF_COLMAJ)
	REAL *ptr = (REAL *) memory;
	sA->pA = ptr;
	ptr += m*n;
	int tmp = m<n ? m : n; // al(min(m,n)) // XXX max ???
	sA->dA = ptr;
	ptr += tmp;
	sA->use_dA = 0;
	size_t memsize = (m*n+tmp)*sizeof(REAL);
#else // MF_PANELMAJ
	const int bs = PS; // 4
	const int nc = PLD;
	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	int cn = (n+nc-1)/nc*nc;
	sA->pm = pm;
	sA->cn = cn;
	REAL *ptr = (REAL *) memory;
	sA->pA = ptr;
	ptr += pm*cn;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	sA->dA = ptr;
	ptr += tmp;
	size_t memsize = (pm*cn+tmp)*sizeof(REAL);
#endif
	sA->memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	return;
	}



// create a matrix structure for a matrix of size m*n by using memory passed by a pointer
void REF_CREATE_VEC(int m, struct VEC *sa, void *memory)
	{
	sa->mem = memory;
	sa->m = m;
#if defined(MF_COLMAJ)
	REAL *ptr = (REAL *) memory;
	sa->pa = ptr;
	sa->memsize = m*sizeof(REAL);
#else // MF_PANELMAJ
	const int bs = PS; // 4
//	const int nc = PLD;
//	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	sa->pm = pm;
	REAL *ptr = (REAL *) memory;
	sa->pa = ptr;
//	ptr += pm;
	sa->memsize = pm*sizeof(REAL);
#endif
	return;
	}



// convert a matrix into a matrix structure
void REF_PACK_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
	int ii, jj;
#if defined(MF_COLMAJ)
	int ldb = sB->m;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int bbi=0; const int bbj=0;
#else
	int bbi=bi; int bbj=bj;
#endif
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = A[ii+0+jj*lda];
			XMATEL_B(bbi+ii+1, bbj+jj) = A[ii+1+jj*lda];
			XMATEL_B(bbi+ii+2, bbj+jj) = A[ii+2+jj*lda];
			XMATEL_B(bbi+ii+3, bbj+jj) = A[ii+3+jj*lda];
			}
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = A[ii+0+jj*lda];
			}
		}
	return;
	}



// convert a lower triangualr matrix into a matrix structure
void REF_PACK_L_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
	int ii, jj;
#if defined(MF_COLMAJ)
	int ldb = sB->m;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int bbi=0; const int bbj=0;
#else
	int bbi=bi; int bbj=bj;
#endif
	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<m; ii++)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = A[ii+0+jj*lda];
			}
		}
	return;
	}



// convert an upper triangualr matrix into a matrix structure
void REF_PACK_U_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
	int ii, jj;
#if defined(MF_COLMAJ)
	int ldb = sB->m;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int bbi=0; const int bbj=0;
#else
	int bbi=bi; int bbj=bj;
#endif
	for(jj=0; jj<n; jj++)
		{
		for(ii=0; ii<=jj; ii++)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = A[ii+0+jj*lda];
			}
		}
	return;
	}



// convert and transpose a matrix into a matrix structure
void REF_PACK_TRAN_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
	int ii, jj;
#if defined(MF_COLMAJ)
	int ldb = sB->m;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int bbi=0; const int bbj=0;
#else
	int bbi=bi; int bbj=bj;
#endif
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			XMATEL_B(bbi+jj, bbj+(ii+0)) = A[ii+0+jj*lda];
			XMATEL_B(bbi+jj, bbj+(ii+1)) = A[ii+1+jj*lda];
			XMATEL_B(bbi+jj, bbj+(ii+2)) = A[ii+2+jj*lda];
			XMATEL_B(bbi+jj, bbj+(ii+3)) = A[ii+3+jj*lda];
			}
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+jj, bbj+(ii+0)) = A[ii+0+jj*lda];
			}
		}
	return;
	}



// convert a vector into a vector structure
void REF_PACK_VEC(int m, REAL *x, int xi, struct VEC *sa, int ai)
	{
	REAL *pa = sa->pa + ai;
	int ii;
	if(xi==1)
		{
		for(ii=0; ii<m; ii++)
			pa[ii] = x[ii];
		}
	else
		{
		for(ii=0; ii<m; ii++)
			pa[ii] = x[ii*xi];
		}
	return;
	}



// convert a matrix structure into a matrix
void REF_UNPACK_MAT(int m, int n, struct MAT *sA, int ai, int aj, REAL *B, int ldb)
	{
	int ii, jj;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			B[ii+0+jj*ldb] = XMATEL_A(aai+ii+0, aaj+jj);
			B[ii+1+jj*ldb] = XMATEL_A(aai+ii+1, aaj+jj);
			B[ii+2+jj*ldb] = XMATEL_A(aai+ii+2, aaj+jj);
			B[ii+3+jj*ldb] = XMATEL_A(aai+ii+3, aaj+jj);
			}
		for(; ii<m; ii++)
			{
			B[ii+0+jj*ldb] = XMATEL_A(aai+ii+0, aaj+jj);
			}
		}
	return;
	}



// convert and transpose a matrix structure into a matrix
void REF_UNPACK_TRAN_MAT(int m, int n, struct MAT *sA, int ai, int aj, REAL *B, int ldb)
	{
	int ii, jj;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			B[jj+(ii+0)*ldb] = XMATEL_A(aai+ii+0, aaj+jj);
			B[jj+(ii+1)*ldb] = XMATEL_A(aai+ii+1, aaj+jj);
			B[jj+(ii+2)*ldb] = XMATEL_A(aai+ii+2, aaj+jj);
			B[jj+(ii+3)*ldb] = XMATEL_A(aai+ii+3, aaj+jj);
			}
		for(; ii<m; ii++)
			{
			B[jj+(ii+0)*ldb] = XMATEL_A(aai+ii+0, aaj+jj);
			}
		}
	return;
	}



// convert a vector structure into a vector
void REF_UNPACK_VEC(int m, struct VEC *sa, int ai, REAL *x, int xi)
	{
	REAL *pa = sa->pa + ai;
	int ii;
	if(xi==1)
		{
		for(ii=0; ii<m; ii++)
			x[ii] = pa[ii];
		}
	else
		{
		for(ii=0; ii<m; ii++)
			x[ii*xi] = pa[ii];
		}
	return;
	}



// copy a generic strmat into a generic strmat
void REF_GECP(int m, int n, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = XMATEL_A(aai+ii+0, aaj+jj);
			XMATEL_B(bbi+ii+1, bbj+jj) = XMATEL_A(aai+ii+1, aaj+jj);
			XMATEL_B(bbi+ii+2, bbj+jj) = XMATEL_A(aai+ii+2, aaj+jj);
			XMATEL_B(bbi+ii+3, bbj+jj) = XMATEL_A(aai+ii+3, aaj+jj);
			}
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = XMATEL_A(aai+ii+0, aaj+jj);
			}
		}
	return;
	}



// scale a generic strmat
void REF_GESC(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			XMATEL_A(aai+ii+0, aaj+jj) *= alpha;
			XMATEL_A(aai+ii+1, aaj+jj) *= alpha;
			XMATEL_A(aai+ii+2, aaj+jj) *= alpha;
			XMATEL_A(aai+ii+3, aaj+jj) *= alpha;
			}
		for(; ii<m; ii++)
			{
			XMATEL_A(aai+ii+0, aaj+jj) *= alpha;
			}
		}
	return;
	}



// scale an generic strmat and copy into generic strmat
void REF_GECPSC(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = XMATEL_A(aai+ii+0, aaj+jj) * alpha;
			XMATEL_B(bbi+ii+1, bbj+jj) = XMATEL_A(aai+ii+1, aaj+jj) * alpha;
			XMATEL_B(bbi+ii+2, bbj+jj) = XMATEL_A(aai+ii+2, aaj+jj) * alpha;
			XMATEL_B(bbi+ii+3, bbj+jj) = XMATEL_A(aai+ii+3, aaj+jj) * alpha;
			}
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) = XMATEL_A(aai+ii+0, aaj+jj) * alpha;
			}
		}
	return;
	}



// scale and add a generic strmat into a generic strmat
void REF_GEAD(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) += alpha*XMATEL_A(aai+ii+0, aaj+jj);
			XMATEL_B(bbi+ii+1, bbj+jj) += alpha*XMATEL_A(aai+ii+1, aaj+jj);
			XMATEL_B(bbi+ii+2, bbj+jj) += alpha*XMATEL_A(aai+ii+2, aaj+jj);
			XMATEL_B(bbi+ii+3, bbj+jj) += alpha*XMATEL_A(aai+ii+3, aaj+jj);
			}
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+ii+0, bbj+jj) += alpha*XMATEL_A(aai+ii+0, aaj+jj);
			}
		}
	return;
	}


// set all elements of a strmat to a value
void REF_GESE(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			XMATEL_A(aai+ii+0, aaj+jj) = alpha;
			XMATEL_A(aai+ii+1, aaj+jj) = alpha;
			XMATEL_A(aai+ii+2, aaj+jj) = alpha;
			XMATEL_A(aai+ii+3, aaj+jj) = alpha;
			}
		for(; ii<m; ii++)
			{
			XMATEL_A(aai+ii+0, aaj+jj) = alpha;
			}
		}
	return;
	}



// copy and transpose a generic strmat into a generic strmat
#ifndef HP_CM
void REF_GETR(int m, int n, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			XMATEL_B(bbi+jj, bbj+ii+0) = XMATEL_A(aai+ii+0, aaj+jj);
			XMATEL_B(bbi+jj, bbj+ii+1) = XMATEL_A(aai+ii+1, aaj+jj);
			XMATEL_B(bbi+jj, bbj+ii+2) = XMATEL_A(aai+ii+2, aaj+jj);
			XMATEL_B(bbi+jj, bbj+ii+3) = XMATEL_A(aai+ii+3, aaj+jj);
			}
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+jj, bbj+ii+0) = XMATEL_A(aai+ii+0, aaj+jj);
			}
		}
	return;
	}
#endif



// insert element into strmat
void REF_GEIN1(REAL alpha, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	MATEL(sA, ai, aj) = alpha;
	return;
	}



// extract element from strmat
REAL REF_GEEX1(struct MAT *sA, int ai, int aj)
	{
	return MATEL(sA, ai, aj);
	}



// copy a lower triangular strmat into a lower triangular strmat
void REF_TRCP_L(int m, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii, jj;
	for(jj=0; jj<m; jj++)
		{
		ii = jj;
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+ii, bbj+jj) = XMATEL_A(aai+ii, aaj+jj);
			}
		}
	return;
	}



// copy and transpose a lower triangular strmat into an upper triangular strmat
void REF_TRTR_L(int m, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii, jj;
	for(jj=0; jj<m; jj++)
		{
		ii = jj;
		for(; ii<m; ii++)
			{
			XMATEL_B(bbi+jj, bbj+ii) = XMATEL_A(aai+ii, aaj+jj);
			}
		}
	return;
	}



// copy and transpose an upper triangular strmat into a lower triangular strmat
void REF_TRTR_U(int m, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii, jj;
	for(jj=0; jj<m; jj++)
		{
		ii = 0;
		for(; ii<=jj; ii++)
			{
			XMATEL_B(bbi+jj, bbj+ii) = XMATEL_A(aai+ii, aaj+jj);
			}
		}
	return;
	}



// set all elements of a strvec to a value
void REF_VECSE(int m, REAL alpha, struct VEC *sx, int xi)
	{
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<m; ii++)
		x[ii] = alpha;
	return;
	}



// copy a strvec into a strvec
void REF_VECCP(int m, struct VEC *sa, int ai, struct VEC *sc, int ci)
	{
	REAL *pa = sa->pa + ai;
	REAL *pc = sc->pa + ci;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pc[ii+0] = pa[ii+0];
		pc[ii+1] = pa[ii+1];
		pc[ii+2] = pa[ii+2];
		pc[ii+3] = pa[ii+3];
		}
	for(; ii<m; ii++)
		{
		pc[ii+0] = pa[ii+0];
		}
	return;
	}



// scale a strvec
void REF_VECSC(int m, REAL alpha, struct VEC *sa, int ai)
	{
	REAL *pa = sa->pa + ai;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pa[ii+0] *= alpha;
		pa[ii+1] *= alpha;
		pa[ii+2] *= alpha;
		pa[ii+3] *= alpha;
		}
	for(; ii<m; ii++)
		{
		pa[ii+0] *= alpha;
		}
	return;
	}



// copy and scale a strvec into a strvec
void REF_VECCPSC(int m, REAL alpha, struct VEC *sa, int ai, struct VEC *sc, int ci)
	{
	REAL *pa = sa->pa + ai;
	REAL *pc = sc->pa + ci;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pc[ii+0] = alpha*pa[ii+0];
		pc[ii+1] = alpha*pa[ii+1];
		pc[ii+2] = alpha*pa[ii+2];
		pc[ii+3] = alpha*pa[ii+3];
		}
	for(; ii<m; ii++)
		{
		pc[ii+0] = alpha*pa[ii+0];
		}
	return;
	}



// scales and adds a strvec into a strvec
void REF_VECAD(int m, REAL alpha, struct VEC *sa, int ai, struct VEC *sc, int ci)
	{
	REAL *pa = sa->pa + ai;
	REAL *pc = sc->pa + ci;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pc[ii+0] += alpha*pa[ii+0];
		pc[ii+1] += alpha*pa[ii+1];
		pc[ii+2] += alpha*pa[ii+2];
		pc[ii+3] += alpha*pa[ii+3];
		}
	for(; ii<m; ii++)
		{
		pc[ii+0] += alpha*pa[ii+0];
		}
	return;
	}



// add scaled strvec to strvec, sparse formulation
void REF_VECAD_SP(int m, REAL alpha, struct VEC *sx, int xi, int *idx, struct VEC *sz, int zi)
	{
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] += alpha * x[ii];
	return;
	}


// insert scaled strvec to strvec, sparse formulation
void REF_VECIN_SP(int m, REAL alpha, struct VEC *sx, int xi, int *idx, struct VEC *sz, int zi)
	{
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] = alpha * x[ii];
	return;
	}



// extract scaled strvec to strvec, sparse formulation
void REF_VECEX_SP(int m, REAL alpha, int *idx, struct VEC *sx, int xi, struct VEC *sz, int zi)
	{
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[ii] = alpha * x[idx[ii]];
	return;
	}


// z += alpha * x[idx]
void REF_VECEXAD_SP(int m, REAL alpha, int *idx, struct VEC *sx, int xi, struct VEC *sz, int zi)
	{
	REAL *x = sx->pa + xi;
	REAL *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[ii] += alpha * x[idx[ii]];
	return;
	}

// insert element into strvec
void REF_VECIN1(REAL alpha, struct VEC *sx, int xi)
	{
	VECEL(sx, xi) = alpha;
	return;
	}



// extract element from strvec
REAL REF_VECEX1(struct VEC *sx, int xi)
	{
	return VECEL(sx, xi);
	}



// permute elements of a vector struct
void REF_VECPE(int kmax, int *ipiv, struct VEC *sx, int xi)
	{
	int ii;
	REAL tmp;
	REAL *x = sx->pa + xi;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			tmp = x[ipiv[ii]];
			x[ipiv[ii]] = x[ii];
			x[ii] = tmp;
			}
		}
	return;
	}



// inverse permute elements of a vector struct
void REF_VECPEI(int kmax, int *ipiv, struct VEC *sx, int xi)
	{
	int ii;
	REAL tmp;
	REAL *x = sx->pa + xi;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			{
			tmp = x[ipiv[ii]];
			x[ipiv[ii]] = x[ii];
			x[ii] = tmp;
			}
		}
	return;
	}



// clip strvec between two strvec
void REF_VECCL(int m, struct VEC *sxm, int xim, struct VEC *sx, int xi, struct VEC *sxp, int xip, struct VEC *sz, int zi)
	{
	REAL *xm = sxm->pa + xim;
	REAL *x  = sx->pa + xi;
	REAL *xp = sxp->pa + xip;
	REAL *z  = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		{
#ifdef USE_C99_MATH
		z[ii] = FMAX( FMIN( x[ii], xp[ii] ), xm[ii] );
#else // no C99
		if(x[ii]>=xp[ii])
			{
			z[ii] = xp[ii];
			}
		else if(x[ii]<=xm[ii])
			{
			z[ii] = xm[ii];
			}
		else
			{
			z[ii] = x[ii];
			}
#endif
		}
	return;
	}



// clip strvec between two strvec, with mask
void REF_VECCL_MASK(int m, struct VEC *sxm, int xim, struct VEC *sx, int xi, struct VEC *sxp, int xip, struct VEC *sz, int zi, struct VEC *sm, int mi)
	{
	REAL *xm = sxm->pa + xim;
	REAL *x  = sx->pa + xi;
	REAL *xp = sxp->pa + xip;
	REAL *z  = sz->pa + zi;
	REAL *mask  = sm->pa + mi;
	int ii;
	for(ii=0; ii<m; ii++)
		{
		if(x[ii]>=xp[ii])
			{
			z[ii] = xp[ii];
			mask[ii] = 1.0;
			}
		else if(x[ii]<=xm[ii])
			{
			z[ii] = xm[ii];
			mask[ii] = -1.0;
			}
		else
			{
			z[ii] = x[ii];
			mask[ii] = 0.0;
			}
		}
	return;
	}


// zero out strvec, with mask
void REF_VECZE(int m, struct VEC *sm, int mi, struct VEC *sv, int vi, struct VEC *se, int ei)
	{
	REAL *mask = sm->pa + mi;
	REAL *v = sv->pa + vi;
	REAL *e = se->pa + ei;
	int ii;
	for(ii=0; ii<m; ii++)
		{
		if(mask[ii]==0)
			{
			e[ii] = v[ii];
			}
		else
			{
			e[ii] = 0;
			}
		}
	return;
	}


// compute inf norm of vector
void REF_VECNRM_INF(int m, struct VEC *sx, int xi, REAL *ptr_norm)
	{
	int ii;
	REAL *x = sx->pa + xi;
	REAL norm = 0.0;
	REAL tmp;
	for(ii=0; ii<m; ii++)
		{
#if 0 //def USE_C99_MATH // does not propagate NaN !!!
		norm = FMAX(norm, FABS(x[ii]));
#else // no c99
		tmp = FABS(x[ii]);
//		norm = tmp>norm ? tmp : norm; // does not propagate NaN !!!
		norm = norm>=tmp ? norm : tmp;
#endif
		}
	*ptr_norm = norm;
	return;
	}



// compute inf norm of vector
void REF_VECNRM_2(int m, struct VEC *sx, int xi, REAL *ptr_norm)
	{
	int ii;
	REAL *x = sx->pa + xi;
	REAL norm = 0.0;
	for(ii=0; ii<m; ii++)
		{
		norm += x[ii]*x[ii];
		}
	norm = SQRT(norm);
	*ptr_norm = norm;
	return;
	}



// insert a vector into diagonal
void REF_DIAIN(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai+ii, aaj+ii) = alpha*x[ii];
	return;
	}



// insert a strvec to the diagonal of a strmat, sparse formulation
void REF_DIAIN_SP(int kmax, REAL alpha, struct VEC *sx, int xi, int *idx, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		XMATEL_A(aai+ii, aaj+ii) = alpha * x[jj];
		}
	return;
	}



// extract a vector from diagonal
void REF_DIAEX(int kmax, REAL alpha, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		x[ii] = alpha*XMATEL_A(aai+ii, aaj+ii);
	return;
	}



// extract the diagonal of a strmat from a strvec, sparse formulation
void REF_DIAEX_SP(int kmax, REAL alpha, int *idx, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		x[jj] = alpha * XMATEL_A(aai+ii, aaj+ii);
		}
	return;
	}



// add a vector to diagonal
void REF_DIAAD(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai+ii, aaj+ii) += alpha*x[ii];
	return;
	}



// add scaled strvec to another strvec and add to diagonal of strmat, sparse formulation
void REF_DIAAD_SP(int kmax, REAL alpha, struct VEC *sx, int xi, int *idx, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		XMATEL_A(aai+ii, aaj+ii) += alpha * x[jj];
		}
	return;
	}



// add scaled strvec to another strvec and insert to diagonal of strmat, sparse formulation
void REF_DIAADIN_SP(int kmax, REAL alpha, struct VEC *sx, int xi, struct VEC *sy, int yi, int *idx, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		XMATEL_A(aai+ii, aaj+ii) = y[jj] + alpha * x[jj];
		}
	return;
	}



// add scalar to diagonal
void REF_DIARE(int kmax, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai+ii, aaj+ii) += alpha;
	return;
	}



// extract a row into a vector
void REF_ROWEX(int kmax, REAL alpha, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		x[ii] = alpha*XMATEL_A(aai, aaj+ii);
	return;
	}



// insert a vector into a row
void REF_ROWIN(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai, aaj+ii) = alpha*x[ii];
	return;
	}



// add a vector to a row
void REF_ROWAD(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai, aaj+ii) += alpha*x[ii];
	return;
	}



// add scaled strvec to row of strmat, sparse formulation
void REF_ROWAD_SP(int kmax, REAL alpha, struct VEC *sx, int xi, int *idx, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	REAL *x = sx->pa + xi;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		XMATEL_A(aai, aaj+ii) += alpha * x[jj];
		}
	return;
	}


// swap two rows of two matrix structs
void REF_ROWSW(int kmax, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii;
	REAL tmp;
	for(ii=0; ii<kmax; ii++)
		{
		tmp = XMATEL_A(aai, aaj+ii);
		XMATEL_A(aai, aaj+ii) = XMATEL_B(bbi, bbj+ii);
		XMATEL_B(bbi, bbj+ii) = tmp;
		}
	return;
	}



// permute the rows of a matrix struct
void REF_ROWPE(int kmax, int *ipiv, struct MAT *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			REF_ROWSW(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}



// inverse permute the rows of a matrix struct
void REF_ROWPEI(int kmax, int *ipiv, struct MAT *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			REF_ROWSW(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}



// extract vector from column
void REF_COLEX(int kmax, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		x[ii] = XMATEL_A(aai+ii, aaj);
	return;
	}



// insert a vector into a calumn
void REF_COLIN(int kmax, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai+ii, aaj) = x[ii];
	return;
	}



// add a scaled vector to a calumn
void REF_COLAD(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	REAL *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai+ii, aaj) += alpha*x[ii];
	return;
	}



// scale a column
void REF_COLSC(int kmax, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	REAL *pA = sA->pA + ai + aj*lda;
	const int aai=0; const int aaj=0;
#else
	int aai=ai; int aaj=aj;
#endif
	int ii;
	for(ii=0; ii<kmax; ii++)
		XMATEL_A(aai+ii, aaj) *= alpha;
	return;
	}



// swap two cols of two matrix structs
void REF_COLSW(int kmax, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sB->use_dA = 0;
#if defined(MF_COLMAJ)
	int lda = sA->m;
	int ldb = sB->m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL *pB = sB->pA + bi + bj*ldb;
	const int aai=0; const int aaj=0;
	const int bbi=0; const int bbj=0;
#else
	int aai=ai; int aaj=aj;
	int bbi=bi; int bbj=bj;
#endif
	int ii;
	REAL tmp;
	for(ii=0; ii<kmax; ii++)
		{
		tmp = XMATEL_A(aai+ii, aaj);
		XMATEL_A(aai+ii, aaj) = XMATEL_B(bbi+ii, bbj);
		XMATEL_B(bbi+ii, bbj) = tmp;
		}
	return;
	}



// permute the cols of a matrix struct
void REF_COLPE(int kmax, int *ipiv, struct MAT *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			REF_COLSW(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// inverse permute the cols of a matrix struct
void REF_COLPEI(int kmax, int *ipiv, struct MAT *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			REF_COLSW(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// 1 norm, lower triangular, non-unit
#if 0
REAL dtrcon_1ln_libstr(int n, struct MAT *sA, int ai, int aj, REAL *work, int *iwork)
	{
	if(n<=0)
		return 1.0;
	int ii, jj;
	int lda = sA.m;
	REAL *pA = sA->pA + ai + aj*lda;
	REAL a00, sum;
	REAL rcond = 0.0;
	// compute norm 1 of A
	REAL anorm = 0.0;
	for(jj=0; jj<n; jj++)
		{
		sum = 0.0;
		for(ii=jj; ii<n; ii++)
			{
			sum += abs(pA[ii+lda*jj]);
			}
		anorm = sum>anorm ? sum : anorm;
		}
	if(anorm>0)
		{
		// estimate norm 1 of inv(A)
		ainorm = 0.0;
		}
	return rcond;
	}
#endif



#if (defined(LA_REFERENCE) & defined(REF)) | (defined(LA_HIGH_PERFORMANCE) & defined(HP_CM)) | defined(LA_EXTERNAL_BLAS_WRAPPER)



size_t MEMSIZE_MAT(int m, int n)
	{
	return REF_MEMSIZE_MAT(m, n);
	}



size_t MEMSIZE_DIAG_MAT(int m, int n)
	{
	return REF_MEMSIZE_DIAG_MAT(m, n);
	}



size_t MEMSIZE_VEC(int m)
	{
	return REF_MEMSIZE_VEC(m);
	}



void CREATE_MAT(int m, int n, struct MAT *sA, void *memory)
	{
	REF_CREATE_MAT(m, n, sA, memory);
	}



void CREATE_VEC(int m, struct VEC *sa, void *memory)
	{
	REF_CREATE_VEC(m, sa, memory);
	}



void PACK_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	REF_PACK_MAT(m, n, A, lda, sB, bi, bj);
	}



void PACK_L_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	REF_PACK_L_MAT(m, n, A, lda, sB, bi, bj);
	}



void PACK_U_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	REF_PACK_U_MAT(m, n, A, lda, sB, bi, bj);
	}



void PACK_TRAN_MAT(int m, int n, REAL *A, int lda, struct MAT *sB, int bi, int bj)
	{
	REF_PACK_TRAN_MAT(m, n, A, lda, sB, bi, bj);
	}



void PACK_VEC(int m, REAL *x, int xi, struct VEC *sa, int ai)
	{
	REF_PACK_VEC(m, x, xi, sa, ai);
	}



void UNPACK_MAT(int m, int n, struct MAT *sA, int ai, int aj, REAL *B, int ldb)
	{
	REF_UNPACK_MAT(m, n, sA, ai, aj, B, ldb);
	}



void UNPACK_TRAN_MAT(int m, int n, struct MAT *sA, int ai, int aj, REAL *B, int ldb)
	{
	REF_UNPACK_TRAN_MAT(m, n, sA, ai, aj, B, ldb);
	}



void UNPACK_VEC(int m, struct VEC *sa, int ai, REAL *x, int xi)
	{
	REF_UNPACK_VEC(m, sa, ai, x, xi);
	}



void GECP(int m, int n, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_GECP(m, n, sA, ai, aj, sB, bi, bj);
	}



void GESC(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	REF_GESC(m, n, alpha, sA, ai, aj);
	}



void GECPSC(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_GECPSC(m, n, alpha, sA, ai, aj, sB, bi, bj);
	}



void GEAD(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_GEAD(m, n, alpha, sA, ai, aj, sB, bi, bj);
	}



void GESE(int m, int n, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	REF_GESE(m, n, alpha, sA, ai, aj);
	}



#ifndef HP_CM
void GETR(int m, int n, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_GETR(m, n, sA, ai, aj, sB, bi, bj);
	}
#endif



void GEIN1(REAL alpha, struct MAT *sA, int ai, int aj)
	{
	REF_GEIN1(alpha, sA, ai, aj);
	}



REAL GEEX1(struct MAT *sA, int ai, int aj)
	{
	return REF_GEEX1(sA, ai, aj);
	}



void TRCP_L(int m, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_TRCP_L(m, sA, ai, aj, sB, bi, bj);
	}



void TRTR_L(int m, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_TRTR_L(m, sA, ai, aj, sB, bi, bj);
	}



void TRTR_U(int m, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_TRTR_U(m, sA, ai, aj, sB, bi, bj);
	}



void VECSE(int m, REAL alpha, struct VEC *sx, int xi)
	{
	REF_VECSE(m, alpha, sx, xi);
	}



void VECCP(int m, struct VEC *sa, int ai, struct VEC *sc, int ci)
	{
	REF_VECCP(m, sa, ai, sc, ci);
	}



void VECSC(int m, REAL alpha, struct VEC *sa, int ai)
	{
	REF_VECSC(m, alpha, sa, ai);
	}



void VECCPSC(int m, REAL alpha, struct VEC *sa, int ai, struct VEC *sc, int ci)
	{
	REF_VECCPSC(m, alpha, sa, ai, sc, ci);
	}



void VECAD(int m, REAL alpha, struct VEC *sa, int ai, struct VEC *sc, int ci)
	{
	REF_VECAD(m, alpha, sa, ai, sc, ci);
	}



void VECAD_SP(int m, REAL alpha, struct VEC *sx, int xi, int *idx, struct VEC *sz, int zi)
	{
	REF_VECAD_SP(m, alpha, sx, xi, idx, sz, zi);
	}



void VECIN_SP(int m, REAL alpha, struct VEC *sx, int xi, int *idx, struct VEC *sz, int zi)
	{
	REF_VECIN_SP(m, alpha, sx, xi, idx, sz, zi);
	}



void VECEX_SP(int m, REAL alpha, int *idx, struct VEC *sx, int xi, struct VEC *sz, int zi)
	{
	REF_VECEX_SP(m, alpha, idx, sx, xi, sz, zi);
	}


void VECEXAD_SP(int m, REAL alpha, int *idx, struct VEC *sx, int xi, struct VEC *sz, int zi)
	{
	REF_VECEXAD_SP(m, alpha, idx, sx, xi, sz, zi);
	}



void VECIN1(REAL alpha, struct VEC *sx, int xi)
	{
	REF_VECIN1(alpha, sx, xi);
	}



REAL VECEX1(struct VEC *sx, int xi)
	{
	return REF_VECEX1(sx, xi);
	}



void VECPE(int kmax, int *ipiv, struct VEC *sx, int xi)
	{
	REF_VECPE(kmax, ipiv, sx, xi);
	}



void VECPEI(int kmax, int *ipiv, struct VEC *sx, int xi)
	{
	REF_VECPEI(kmax, ipiv, sx, xi);
	}



void VECCL(int m, struct VEC *sxm, int xim, struct VEC *sx, int xi, struct VEC *sxp, int xip, struct VEC *sz, int zi)
	{
	REF_VECCL(m, sxm, xim, sx, xi, sxp, xip, sz, zi);
	}



void VECCL_MASK(int m, struct VEC *sxm, int xim, struct VEC *sx, int xi, struct VEC *sxp, int xip, struct VEC *sz, int zi, struct VEC *sm, int mi)
	{
	REF_VECCL_MASK(m, sxm, xim, sx, xi, sxp, xip, sz, zi, sm, mi);
	}



void VECZE(int m, struct VEC *sm, int mi, struct VEC *sv, int vi, struct VEC *se, int ei)
	{
	REF_VECZE(m, sm, mi, sv, vi, se, ei);
	}



void VECNRM_INF(int m, struct VEC *sx, int xi, REAL *ptr_norm)
	{
	REF_VECNRM_INF(m, sx, xi, ptr_norm);
	}



void VECNRM_2(int m, struct VEC *sx, int xi, REAL *ptr_norm)
	{
	REF_VECNRM_2(m, sx, xi, ptr_norm);
	}



void DIAIN(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	REF_DIAIN(kmax, alpha, sx, xi, sA, ai, aj);
	}



void DIAIN_SP(int kmax, REAL alpha, struct VEC *sx, int xi, int *idx, struct MAT *sA, int ai, int aj)
	{
	REF_DIAIN_SP(kmax, alpha, sx, xi, idx, sA, ai, aj);
	}



void DIAEX(int kmax, REAL alpha, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
	REF_DIAEX(kmax, alpha, sA, ai, aj, sx, xi);
	}



void DIAEX_SP(int kmax, REAL alpha, int *idx, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
	REF_DIAEX_SP(kmax, alpha, idx, sA, ai, aj, sx, xi);
	}



void DIAAD(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	REF_DIAAD(kmax, alpha, sx, xi, sA, ai, aj);
	}



void DIAAD_SP(int kmax, REAL alpha, struct VEC *sx, int xi, int *idx, struct MAT *sA, int ai, int aj)
	{
	REF_DIAAD_SP(kmax, alpha, sx, xi, idx, sA, ai, aj);
	}



void DIAADIN_SP(int kmax, REAL alpha, struct VEC *sx, int xi, struct VEC *sy, int yi, int *idx, struct MAT *sA, int ai, int aj)
	{
	REF_DIAADIN_SP(kmax, alpha, sx, xi, sy, yi, idx, sA, ai, aj);
	}



void DIARE(int kmax, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	REF_DIARE(kmax, alpha, sA, ai, aj);
	}



void ROWEX(int kmax, REAL alpha, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
	REF_ROWEX(kmax, alpha, sA, ai, aj, sx, xi);
	}



void ROWIN(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	REF_ROWIN(kmax, alpha, sx, xi, sA, ai, aj);
	}



void ROWAD(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	REF_ROWAD(kmax, alpha, sx, xi, sA, ai, aj);
	}



void ROWAD_SP(int kmax, REAL alpha, struct VEC *sx, int xi, int *idx, struct MAT *sA, int ai, int aj)
	{
	REF_ROWAD_SP(kmax, alpha, sx, xi, idx, sA, ai, aj);
	}



void ROWSW(int kmax, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_ROWSW(kmax, sA, ai, aj, sB, bi, bj);
	}



void ROWPE(int kmax, int *ipiv, struct MAT *sA)
	{
	REF_ROWPE(kmax, ipiv, sA);
	}



void ROWPEI(int kmax, int *ipiv, struct MAT *sA)
	{
	REF_ROWPEI(kmax, ipiv, sA);
	}



void COLEX(int kmax, struct MAT *sA, int ai, int aj, struct VEC *sx, int xi)
	{
	REF_COLEX(kmax, sA, ai, aj, sx, xi);
	}



void COLIN(int kmax, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	REF_COLIN(kmax, sx, xi, sA, ai, aj);
	}



void COLAD(int kmax, REAL alpha, struct VEC *sx, int xi, struct MAT *sA, int ai, int aj)
	{
	REF_COLAD(kmax, alpha, sx, xi, sA, ai, aj);
	}



void COLSC(int kmax, REAL alpha, struct MAT *sA, int ai, int aj)
	{
	REF_COLSC(kmax, alpha, sA, ai, aj);
	}



void COLSW(int kmax, struct MAT *sA, int ai, int aj, struct MAT *sB, int bi, int bj)
	{
	REF_COLSW(kmax, sA, ai, aj, sB, bi, bj);
	}



void COLPE(int kmax, int *ipiv, struct MAT *sA)
	{
	REF_COLPE(kmax, ipiv, sA);
	}



void COLPEI(int kmax, int *ipiv, struct MAT *sA)
	{
	REF_COLPEI(kmax, ipiv, sA);
	}



#endif
