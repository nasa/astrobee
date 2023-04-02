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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <blasfeo_common.h>
#include <blasfeo_block_size.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_d_aux_ref.h>
#endif



#if defined(LA_HIGH_PERFORMANCE)



// return the memory size (in bytes) needed for a strmat
size_t blasfeo_memsize_dmat(int m, int n)
	{
	const int bs = D_PS; // 4
	const int nc = D_PLD;
	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	int cn = (n+nc-1)/nc*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = (pm*cn+tmp)*sizeof(double);
	memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	return memsize;
	}



// return the memory size (in bytes) needed for the digonal of a strmat
size_t blasfeo_memsize_diag_dmat(int m, int n)
	{
	const int bs = D_PS; // 4
	const int nc = D_PLD;
	const int al = bs*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = tmp*sizeof(double);
	return memsize;
	}



// create a matrix structure for a matrix of size m*n by using memory passed by a pointer
void blasfeo_create_dmat(int m, int n, struct blasfeo_dmat *sA, void *memory)
	{
	sA->mem = memory;
	const int bs = D_PS; // 4
	const int nc = D_PLD;
	const int al = bs*nc;
	sA->m = m;
	sA->n = n;
	int pm = (m+bs-1)/bs*bs;
	int cn = (n+nc-1)/nc*nc;
	sA->pm = pm;
	sA->cn = cn;
	double *ptr = (double *) memory;
	sA->pA = ptr;
	ptr += pm*cn;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	sA->dA = ptr;
	ptr += tmp;
	size_t memsize = (pm*cn+tmp)*sizeof(double);
	sA->memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	sA->use_dA = 0; // invalidate stored inverse diagonal
	return;
	}



// return memory size (in bytes) needed for a strvec
size_t blasfeo_memsize_dvec(int m)
	{
	const int bs = D_PS; // 4
//	const int nc = D_PLD;
//	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	size_t memsize = pm*sizeof(double);
	return memsize;
	}



// create a vector structure for a vector of size m by using memory passed by a pointer
void blasfeo_create_dvec(int m, struct blasfeo_dvec *sa, void *memory)
	{
	sa->mem = memory;
	const int bs = D_PS; // 4
//	const int nc = D_PLD;
//	const int al = bs*nc;
	sa->m = m;
	int pm = (m+bs-1)/bs*bs;
	sa->pm = pm;
	double *ptr = (double *) memory;
	sa->pa = ptr;
//	ptr += pm;
	sa->memsize = pm*sizeof(double);
	return;
	}



// convert a matrix into a matrix structure
void blasfeo_pack_dmat(int m, int n, double *A, int lda, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_dmat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_dmat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert a lower triangular matrix into a matrix structure
void blasfeo_pack_l_dmat(int m, int n, double *A, int lda, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_l_dmat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_l_dmat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert a upper triangular matrix into a matrix structure
void blasfeo_pack_u_dmat(int m, int n, double *A, int lda, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_u_dmat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_u_dmat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert and transpose a matrix into a matrix structure
void blasfeo_pack_tran_dmat(int m, int n, double *A, int lda, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_tran_dmat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_tran_dmat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert a vector into a vector structure
void blasfeo_pack_dvec(int m, double *x, int xi, struct blasfeo_dvec *sa, int ai)
	{
	double *pa = sa->pa + ai;
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
void blasfeo_unpack_dmat(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, double *A, int lda)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_unpack_dmat(m, n, sA, ai, aj, A, lda);
#else
	printf("\nblasfeo_unpack_dmat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert and transpose a matrix structure into a matrix
void blasfeo_unpack_tran_dmat(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, double *A, int lda)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_unpack_tran_dmat(m, n, sA, ai, aj, A, lda);
#else
	printf("\nblasfeo_unpack_tran_dmat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert a vector structure into a vector
void blasfeo_unpack_dvec(int m, struct blasfeo_dvec *sa, int ai, double *x, int xi)
	{
	double *pa = sa->pa + ai;
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



// insert element into strmat
void blasfeo_dgein1(double a, struct blasfeo_dmat *sA, int ai, int aj)
	{

	if (ai==aj)
		{
		// invalidate stored inverse diagonal
		sA->use_dA = 0;
		}

	const int bs = D_PS;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	pA[0] = a;
	return;
	}



// extract element from strmat
double blasfeo_dgeex1(struct blasfeo_dmat *sA, int ai, int aj)
	{
	const int bs = D_PS;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	return pA[0];
	}



// insert element into strvec
void blasfeo_dvecin1(double a, struct blasfeo_dvec *sx, int xi)
	{
	double *x = sx->pa + xi;
	x[0] = a;
	return;
	}



// extract element from strvec
double blasfeo_dvecex1(struct blasfeo_dvec *sx, int xi)
	{
	double *x = sx->pa + xi;
	return x[0];
	}



// set all elements of a strmat to a value
void blasfeo_dgese(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	double *pA = sA->pA + ai%bs + ai/bs*bs*sda + aj*bs;
	int m0 = m<(bs-ai%bs)%bs ? m : (bs-ai%bs)%bs;
	int ii, jj;
	if(m0>0)
		{
		for(ii=0; ii<m0; ii++)
			{
			for(jj=0; jj<n; jj++)
				{
				pA[jj*bs] = alpha;
				}
			pA += 1;
			}
		pA += bs*(sda-1);
		m -= m0;
		}
	for(ii=0; ii<m-7; ii+=8)
		{
		for(jj=0; jj<n; jj++)
			{
			pA[0+jj*bs] = alpha;
			pA[1+jj*bs] = alpha;
			pA[2+jj*bs] = alpha;
			pA[3+jj*bs] = alpha;
			pA[4+jj*bs] = alpha;
			pA[5+jj*bs] = alpha;
			pA[6+jj*bs] = alpha;
			pA[7+jj*bs] = alpha;
			}
		pA += bs*sda;
		}
	for( ; ii<m; ii++)
		{
		for(jj=0; jj<n; jj++)
			{
			pA[jj*bs] = alpha;
			}
		pA += 1;
		}
	return;
	}



// set all elements of a strvec to a value
void blasfeo_dvecse(int m, double alpha, struct blasfeo_dvec *sx, int xi)
	{
	double *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<m; ii++)
		x[ii] = alpha;
	return;
	}



// insert a vector into diagonal
void blasfeo_ddiain(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	int offsetA = ai%bs;

	int kna = (bs-offsetA%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pA[ll+bs*ll] = alpha*x[ll];
			}
		pA += kna + bs*(sda-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pA[jj*sda+(jj+0)*bs+0] = alpha*x[jj+0];
		pA[jj*sda+(jj+1)*bs+1] = alpha*x[jj+1];
		pA[jj*sda+(jj+2)*bs+2] = alpha*x[jj+2];
		pA[jj*sda+(jj+3)*bs+3] = alpha*x[jj+3];
		pA[jj*sda+(jj+4)*bs+4] = alpha*x[jj+4];
		pA[jj*sda+(jj+5)*bs+5] = alpha*x[jj+5];
		pA[jj*sda+(jj+6)*bs+6] = alpha*x[jj+6];
		pA[jj*sda+(jj+7)*bs+7] = alpha*x[jj+7];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+(jj+ll)*bs+ll] = alpha*x[jj+ll];
		}
	return;
	}



// add scalar to diagonal
void blasfeo_ddiare(int kmax, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int offsetA = ai%bs;

	int kna = (bs-offsetA%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pA[ll+bs*ll] += alpha;
			}
		pA += kna + bs*(sda-1) + kna*bs;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pA[jj*sda+(jj+0)*bs+0] += alpha;
		pA[jj*sda+(jj+1)*bs+1] += alpha;
		pA[jj*sda+(jj+2)*bs+2] += alpha;
		pA[jj*sda+(jj+3)*bs+3] += alpha;
		pA[jj*sda+(jj+4)*bs+4] += alpha;
		pA[jj*sda+(jj+5)*bs+5] += alpha;
		pA[jj*sda+(jj+6)*bs+6] += alpha;
		pA[jj*sda+(jj+7)*bs+7] += alpha;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+(jj+ll)*bs+ll] += alpha;
		}
	return;
	}



// swap two rows of two matrix structs
void blasfeo_drowsw(int kmax, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_drowsw(kmax, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_drowsw: feature not implemented yet\n");
	exit(1);
#endif
	}



// extract a row int a vector
void blasfeo_drowex(int kmax, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_drowex(kmax, alpha, sA, ai, aj, sx, xi);
#else
	printf("\nblasfeo_drowex: feature not implemented yet\n");
	exit(1);
#endif
	}



// insert a vector into a row
void blasfeo_drowin(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_drowin(kmax, alpha, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_drowin: feature not implemented yet\n");
	exit(1);
#endif
	}



// add a vector to a row
void blasfeo_drowad(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_drowad(kmax, alpha, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_drowad: feature not implemented yet\n");
	exit(1);
#endif
	}



// extract vector from column
void blasfeo_dcolex(int kmax, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dcolex(kmax, sA, ai, aj, sx, xi);
#else
	printf("\nblasfeo_dcolex: feature not implemented yet\n");
	exit(1);
#endif
	}



// insert as vector as a column
void blasfeo_dcolin(int kmax, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dcolin(kmax, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_dcolin: feature not implemented yet\n");
	exit(1);
#endif
	}



// add scaled vector to column
void blasfeo_dcolad(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dcolad(kmax, alpha, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_dcolad: feature not implemented yet\n");
	exit(1);
#endif
	}



// scale a column
void blasfeo_dcolsc(int kmax, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;

	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;

	int kna = (bs-ai%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pA[ll] *= alpha;
			}
		pA += kna + bs*(sda-1);
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pA[jj*sda+0] *= alpha;
		pA[jj*sda+1] *= alpha;
		pA[jj*sda+2] *= alpha;
		pA[jj*sda+3] *= alpha;
		pA[jj*sda+4] *= alpha;
		pA[jj*sda+5] *= alpha;
		pA[jj*sda+6] *= alpha;
		pA[jj*sda+7] *= alpha;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+ll] *= alpha;
		}

	return;
	}



// swap two cols of two matrix structs
void blasfeo_dcolsw(int kmax, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dcolsw(kmax, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_dcolsw: feature not implemented yet\n");
	exit(1);
#endif
	}



// permute the cols of a matrix struct
void blasfeo_dcolpe(int kmax, int *ipiv, struct blasfeo_dmat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			blasfeo_dcolsw(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// inverse permute the cols of a matrix struct
void blasfeo_dcolpei(int kmax, int *ipiv, struct blasfeo_dmat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			blasfeo_dcolsw(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// copy a generic strmat into a generic strmat
void blasfeo_dgecp(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int ps = 8;

	int sda = sA->cn;
	int sdb = sB->cn;

	int air = ai & (ps-1);
	int bir = bi & (ps-1);

	// pA, pB point to panels edges
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;

	int ii, mmax;

	int offsetA = (air+ps-bir) & (ps-1);

	// clean bir
	if(bir!=0)
		{
		mmax = m<ps-bir ? m : ps-bir;
		kernel_dpacp_nn_8_vs_lib8(n, air, pA, sda, pB+bir, mmax);
		if(air>=bir)
			pA += ps*sda;
		pB += ps*sdb;
		m -= mmax;
		}
	ii = 0;	
	// main loop
	for(; ii<m-15; ii+=16)
		{
		kernel_dpacp_nn_16_lib8(n, offsetA, pA+ii*sda, sda, pB+ii*sdb, sdb);
		}
	for(; ii<m-7; ii+=8)
		{
		kernel_dpacp_nn_8_lib8(n, offsetA, pA+ii*sda, sda, pB+ii*sdb);
		}
	if(ii<m)
		{
		kernel_dpacp_nn_8_vs_lib8(n, offsetA, pA+ii*sda, sda, pB+ii*sdb, m-ii);
		}

	return;

	}



// copy a lower triangular strmat into a lower triangular strmat
void blasfeo_dtrcp_l(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int ps = 8;

	int sda = sA->cn;
	int sdb = sB->cn;

	int air = ai & (ps-1);
	int bir = bi & (ps-1);

	// pA, pB point to panels edges
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;

	int ii, mmax, kmax;

	int offsetA = (air+ps-bir) & (ps-1);

	// clean bir
	kmax = 0;
	if(bir!=0)
		{
		mmax = m<ps-bir ? m : ps-bir;
		kernel_dpacp_l_nn_8_vs_lib8(kmax, air, pA, sda, pB+bir, mmax);
		if(air>=bir)
			pA += ps*sda;
		pB += ps*sdb;
		m -= mmax;
		kmax += ps-bir;
		}
	ii = 0;	
	// main loop
//	for(; ii<m-15; ii+=16)
//		{
//		kernel_dpacp_nn_16_lib8(n, offsetA, pA+ii*sda, sda, pB+ii*sdb, sdb);
//		}
	for(; ii<m-7; ii+=8, kmax+=8)
		{
		kernel_dpacp_l_nn_8_lib8(kmax, offsetA, pA+ii*sda, sda, pB+ii*sdb);
		}
	if(ii<m)
		{
		kernel_dpacp_l_nn_8_vs_lib8(kmax, offsetA, pA+ii*sda, sda, pB+ii*sdb, m-ii);
		}

	return;

	}



// copy and scale a generic strmat into a generic strmat
void blasfeo_dgecpsc(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dgecpsc(m, n, alpha, sA, ai, aj, sB, bi, bj);
#else
	printf("\nblasfeo_dgecpsc: feature not implemented yet\n");
	exit(1);
#endif
	}



#if 0
// copy  and scale a lower triangular strmat into a lower triangular strmat
void blasfeo_dtrcpsc_l(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrcpsc_l(m, alpha, sA, ai, aj, sB, bi, bj);
#else
	printf("\nblasfeo_dtrcpsc_l: feature not implemented yet\n");
	exit(1);
#endif
	}
#endif



// scale a generic strmat
void blasfeo_dgesc(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dgesc(m, n, alpha, sA, ai, aj);
#else
	printf("\nblasfeo_dgesc: feature not implemented yet\n");
	exit(1);
#endif
	}



#if 0
// scale a triangular strmat
void blasfeo_dtrsc_l(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrsc_l(m, alpha, sA, ai, aj);
#else
	printf("\nblasfeo_dtrsc_l: feature not implemented yet\n");
	exit(1);
#endif
	}
#endif



// copy a strvec into a strvec
void blasfeo_dveccp(int m, struct blasfeo_dvec *sa, int ai, struct blasfeo_dvec *sc, int ci)
	{
	double *pa = sa->pa + ai;
	double *pc = sc->pa + ci;
#if 0
	kernel_dveccp_inc1(m, pa, pc);
#else
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
#endif
	return;
	}



// scale a strvec
void blasfeo_dvecsc(int m, double alpha, struct blasfeo_dvec *sa, int ai)
	{
	double *pa = sa->pa + ai;
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
void blasfeo_dveccpsc(int m, double alpha, struct blasfeo_dvec *sa, int ai, struct blasfeo_dvec *sc, int ci)
	{
	double *pa = sa->pa + ai;
	double *pc = sc->pa + ci;
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



// scale and add a generic strmat into a generic strmat
void blasfeo_dgead(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int ps = 8;

	int sda = sA->cn;
	int sdb = sB->cn;

	int air = ai & (ps-1);
	int bir = bi & (ps-1);

	// pA, pB point to panels edges
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;

	int ii, mmax;

	int offsetA = (air+ps-bir) & (ps-1);

	// clean bir
	if(bir!=0)
		{
		mmax = m<ps-bir ? m : ps-bir;
		kernel_dpaad_nn_8_vs_lib8(n, &alpha, air, pA, sda, pB+bir, mmax);
		if(air>=bir)
			pA += ps*sda;
		pB += ps*sdb;
		m -= mmax;
		}
	ii = 0;	
	// main loop
//	for(; ii<m-15; ii+=16)
//		{
//		kernel_dpacp_nn_16_lib8(n, &alpha, offsetA, pA+ii*sda, sda, pB+ii*sdb, sdb);
//		}
	for(; ii<m-7; ii+=8)
		{
		kernel_dpaad_nn_8_lib8(n, &alpha, offsetA, pA+ii*sda, sda, pB+ii*sdb);
		}
	if(ii<m)
		{
		kernel_dpaad_nn_8_vs_lib8(n, &alpha, offsetA, pA+ii*sda, sda, pB+ii*sdb, m-ii);
		}

	return;

	}



// scales and adds a strvec into a strvec
void blasfeo_dvecad(int m, double alpha, struct blasfeo_dvec *sa, int ai, struct blasfeo_dvec *sc, int ci)
	{
	double *pa = sa->pa + ai;
	double *pc = sc->pa + ci;
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


// copy and transpose a generic strmat into a generic strmat
void blasfeo_dgetr(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int ps = 8;

	int sda = sA->cn;
	int sdb = sB->cn;

	int air = ai & (ps-1);
	int bir = bi & (ps-1);

	// pA, pB point to panels edges
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;

	int ii, nmax;

	// clean bir
	if(bir!=0)
		{
		nmax = n<ps-bir ? n : ps-bir;
		kernel_dpacp_tn_8_vs_lib8(m, air, pA, sda, pB+bir, nmax);
		pA += nmax*ps;
		pB += ps*sdb;
		n -= nmax;
		}
	ii = 0;	
	// main loop
	for(; ii<n-7; ii+=8)
		{
		kernel_dpacp_tn_8_lib8(m, air, pA+ii*ps, sda, pB+ii*sdb);
		}
	if(ii<n)
		{
		kernel_dpacp_tn_8_vs_lib8(m, air, pA+ii*ps, sda, pB+ii*sdb, n-ii);
		}

	return;

	}



// copy and transpose a lower triangular strmat into an upper triangular strmat
void blasfeo_dtrtr_l(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int ps = 8;

	int sda = sA->cn;
	int sdb = sB->cn;

	int air = ai & (ps-1);
	int bir = bi & (ps-1);

	// pA, pB point to panels edges
	double *pA = sA->pA + aj*ps + (ai-air)*sda;
	double *pB = sB->pA + bj*ps + (bi-bir)*sdb;

	int ii, mmax;

	int offsetA = (air+ps-bir) & (ps-1);

	// clean bir
	if(bir!=0)
		{
		mmax = m<ps-bir ? m : ps-bir;
		kernel_dpacp_l_tn_8_vs_lib8(m-mmax, air, pA, sda, pB+bir, mmax);
		pA += mmax*ps;
		if(air>=bir)
			pA += ps*sda;
		pB += ps*sdb + mmax*ps;
		m -= mmax;
		}
	ii = 0;	
	// main loop
	for(; ii<m-7; ii+=8)
		{
		kernel_dpacp_l_tn_8_lib8(m-ii-8, offsetA, pA+ii*sda+ii*ps, sda, pB+ii*sdb+ii*ps);
		}
	if(ii<m)
		{
		kernel_dpacp_l_tn_8_vs_lib8(m-ii-8, offsetA, pA+ii*sda+ii*ps, sda, pB+ii*sdb+ii*ps, m-ii);
		}

	return;

	}



// copy and transpose an upper triangular strmat into a lower triangular strmat
void blasfeo_dtrtr_u(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dtrtr_u(m, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_dtrtr_u: feature not implemented yet\n");
	exit(1);
#endif
	}



// insert a strvec to diagonal of strmat, sparse formulation
void blasfeo_ddiain_sp(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, int *idx, struct blasfeo_dmat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = D_PS;
	double *x = sx->pa + xi;
	int sdd = sD->cn;
	double *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs] = alpha * x[jj];
		}
	return;
	}



// extract a vector from diagonal
void blasfeo_ddiaex(int kmax, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi)
	{
	const int bs = 8;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	int offsetA = ai%bs;

	int kna = (bs-offsetA%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			x[ll] = alpha*pA[ll+bs*ll];
			}
		pA += kna + bs*(sda-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		x[jj+0] = alpha*pA[jj*sda+(jj+0)*bs+0];
		x[jj+1] = alpha*pA[jj*sda+(jj+1)*bs+1];
		x[jj+2] = alpha*pA[jj*sda+(jj+2)*bs+2];
		x[jj+3] = alpha*pA[jj*sda+(jj+3)*bs+3];
		x[jj+4] = alpha*pA[jj*sda+(jj+4)*bs+4];
		x[jj+5] = alpha*pA[jj*sda+(jj+5)*bs+5];
		x[jj+6] = alpha*pA[jj*sda+(jj+6)*bs+6];
		x[jj+7] = alpha*pA[jj*sda+(jj+7)*bs+7];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[jj+ll] = alpha*pA[jj*sda+(jj+ll)*bs+ll];
		}
	return;
	}



// extract the diagonal of a strmat to a strvec, sparse formulation
void blasfeo_ddiaex_sp(int kmax, double alpha, int *idx, struct blasfeo_dmat *sD, int di, int dj, struct blasfeo_dvec *sx, int xi)
	{
	const int bs = D_PS;
	double *x = sx->pa + xi;
	int sdd = sD->cn;
	double *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		x[jj] = alpha * pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs];
		}
	return;
	}



// add a vector to diagonal
void blasfeo_ddiaad(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	int offsetA = ai%bs;

	int kna = (bs-offsetA%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pA[ll+bs*ll] += alpha*x[ll];
			}
		pA += kna + bs*(sda-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pA[jj*sda+(jj+0)*bs+0] += alpha*x[jj+0];
		pA[jj*sda+(jj+1)*bs+1] += alpha*x[jj+1];
		pA[jj*sda+(jj+2)*bs+2] += alpha*x[jj+2];
		pA[jj*sda+(jj+3)*bs+3] += alpha*x[jj+3];
		pA[jj*sda+(jj+4)*bs+4] += alpha*x[jj+4];
		pA[jj*sda+(jj+5)*bs+5] += alpha*x[jj+5];
		pA[jj*sda+(jj+6)*bs+6] += alpha*x[jj+6];
		pA[jj*sda+(jj+7)*bs+7] += alpha*x[jj+7];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+(jj+ll)*bs+ll] += alpha*x[jj+ll];
		}
	return;
	}



// add scaled strvec to diagonal of strmat, sparse formulation
void blasfeo_ddiaad_sp(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, int *idx, struct blasfeo_dmat *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = D_PS;
	double *x = sx->pa + xi;
	int sdd = sD->cn;
	double *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs] += alpha * x[jj];
		}
	return;
	}



// add scaled strvec to another strvec and insert to diagonal of strmat, sparse formulation
void blasfeo_ddiaadin_sp(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, int *idx, struct blasfeo_dmat *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = D_PS;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;
	int sdd = sD->cn;
	double *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs] = y[jj] + alpha * x[jj];
		}
	return;
	}



// add scaled strvec to row of strmat, sparse formulation
void blasfeo_drowad_sp(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, int *idx, struct blasfeo_dmat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_drowad_sp(kmax, alpha, sx, xi, idx, sD, di, dj);
#else
	printf("\nblasfeo_drowad_sp: feature not implemented yet\n");
	exit(1);
#endif
	}



// add scaled strvec to strvec, sparse formulation
void blasfeo_dvecad_sp(int m, double alpha, struct blasfeo_dvec *sx, int xi, int *idx, struct blasfeo_dvec *sz, int zi)
	{
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] += alpha * x[ii];
	return;
	}



// insert scaled strvec to strvec, sparse formulation
void blasfeo_dvecin_sp(int m, double alpha, struct blasfeo_dvec *sx, int xi, int *idx, struct blasfeo_dvec *sz, int zi)
	{
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] = alpha * x[ii];
	return;
	}



// extract scaled strvec to strvec, sparse formulation
void blasfeo_dvecex_sp(int m, double alpha, int *idx, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[ii] = alpha * x[idx[ii]];
	return;
	}



// z += alpha * x[idx]
void blasfeo_dvecexad_sp(int m, double alpha, int *idx, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sz, int zi)
	{
	double *x = sx->pa + xi;
	double *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[ii] += alpha * x[idx[ii]];
	return;
	}



// clip strvec between two strvec
void blasfeo_dveccl(int m, struct blasfeo_dvec *sxm, int xim, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sxp, int xip, struct blasfeo_dvec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dveccl(m, sxm, xim, sx, xi, sxp, xip, sz, zi);
#else
	printf("\nblasfeo_dveccl: feature not implemented yet\n");
	exit(1);
#endif
	}



// clip strvec between two strvec, with mask
void blasfeo_dveccl_mask(int m, struct blasfeo_dvec *sxm, int xim, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sxp, int xip, struct blasfeo_dvec *sz, int zi, struct blasfeo_dvec *sm, int mi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dveccl_mask(m, sxm, xim, sx, xi, sxp, xip, sz, zi, sm, mi);
#else
	printf("\nblasfeo_dveccl_mask: feature not implemented yet\n");
	exit(1);
#endif
	}



// zero out strvec to strvec with mask
void blasfeo_dvecze(int m, struct blasfeo_dvec *sm, int mi, struct blasfeo_dvec *sv, int vi, struct blasfeo_dvec *se, int ei)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dvecze(m, sm, mi, sv, vi, se, ei);
#else
	printf("\nblasfeo_dvecze: feature not implemented yet\n");
	exit(1);
#endif
	}



// compute inf norm of vector
void blasfeo_dvecnrm_inf(int m, struct blasfeo_dvec *sx, int xi, double *ptr_norm)
	{
	int ii;
	double *x = sx->pa + xi;
	double norm = 0.0;
	double tmp;
	for(ii=0; ii<m; ii++)
		{
#if 0 //def USE_C99_MATH // does not propagate NaN !!!
		norm = fmax(norm, fabs(x[ii]));
#else // no c99
		tmp = fabs(x[ii]);
//		norm = tmp>norm ? tmp : norm; // does not propagate NaN !!!
		norm = norm>=tmp ? norm : tmp;
#endif
		}
	*ptr_norm = norm;
	return;
	}



// compute 2 norm of vector
void blasfeo_dvecnrm_2(int m, struct blasfeo_dvec *sx, int xi, double *ptr_norm)
	{
	int ii;
	double *x = sx->pa + xi;
	double norm = 0.0;
	for(ii=0; ii<m; ii++)
		{
		norm += x[ii]*x[ii];
		}
	norm = sqrt(norm);
	*ptr_norm = norm;
	return;
	}



// permute elements of a vector struct
void blasfeo_dvecpe(int kmax, int *ipiv, struct blasfeo_dvec *sx, int xi)
	{
	int ii;
	double tmp;
	double *x = sx->pa + xi;
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
void blasfeo_dvecpei(int kmax, int *ipiv, struct blasfeo_dvec *sx, int xi)
	{
	int ii;
	double tmp;
	double *x = sx->pa + xi;
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



#else

#error : wrong LA choice

#endif
