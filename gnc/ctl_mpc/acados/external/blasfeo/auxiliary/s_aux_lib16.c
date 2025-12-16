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
#include <blasfeo_s_aux.h>
//#include <blasfeo_s_kernel.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_s_aux_ref.h>
#endif



#if defined(LA_HIGH_PERFORMANCE)



// return the memory size (in bytes) needed for a strmat
size_t blasfeo_memsize_smat(int m, int n)
	{
	const int bs = S_PS; // 4
	const int nc = S_PLD;
	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	int cn = (n+nc-1)/nc*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = (pm*cn+tmp)*sizeof(float);
	memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	return memsize;
	}



// return the memory size (in bytes) needed for the digonal of a strmat
size_t blasfeo_memsize_diag_smat(int m, int n)
	{
	const int bs = S_PS; // 4
	const int nc = S_PLD;
	const int al = bs*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = tmp*sizeof(float);
	return memsize;
	}



// create a matrix structure for a matrix of size m*n by using memory passed by a pointer
void blasfeo_create_smat(int m, int n, struct blasfeo_smat *sA, void *memory)
	{
	sA->mem = memory;
	const int bs = S_PS; // 4
	const int nc = S_PLD;
	const int al = bs*nc;
	sA->m = m;
	sA->n = n;
	int pm = (m+bs-1)/bs*bs;
	int cn = (n+nc-1)/nc*nc;
	sA->pm = pm;
	sA->cn = cn;
	float *ptr = (float *) memory;
	sA->pA = ptr;
	ptr += pm*cn;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	sA->dA = ptr;
	ptr += tmp;
	size_t memsize = (pm*cn+tmp)*sizeof(float);
	sA->memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	sA->use_dA = 0; // invalidate stored inverse diagonal
	return;
	}



// return memory size (in bytes) needed for a strvec
size_t blasfeo_memsize_svec(int m)
	{
	const int bs = S_PS; // 4
//	const int nc = S_PLD;
//	const int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	size_t memsize = pm*sizeof(float);
	return memsize;
	}



// create a vector structure for a vector of size m by using memory passed by a pointer
void blasfeo_create_svec(int m, struct blasfeo_svec *sa, void *memory)
	{
	sa->mem = memory;
	const int bs = S_PS; // 4
//	const int nc = S_PLD;
//	const int al = bs*nc;
	sa->m = m;
	int pm = (m+bs-1)/bs*bs;
	sa->pm = pm;
	float *ptr = (float *) memory;
	sa->pa = ptr;
//	ptr += pm;
	sa->memsize = pm*sizeof(float);
	return;
	}



// convert a matrix into a matrix structure
void blasfeo_pack_smat(int m, int n, float *A, int lda, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_smat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_smat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert a lower triangular matrix into a matrix structure
void blasfeo_pack_l_smat(int m, int n, float *A, int lda, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_l_smat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_l_smat: feature not implemented yet\n");
	exit(1);
#endif
	}



#if 0
// convert a upper triangular matrix into a matrix structure
void blasfeo_pack_u_smat(int m, int n, float *A, int lda, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_u_smat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_u_smat: feature not implemented yet\n");
	exit(1);
#endif
	}
#endif



// convert and transpose a matrix into a matrix structure
void blasfeo_pack_tran_smat(int m, int n, float *A, int lda, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_pack_tran_smat(m, n, A, lda, sA, ai, aj);
#else
	printf("\nblasfeo_pack_tran_smat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert a vector into a vector structure
void blasfeo_pack_svec(int m, float *x, int xi, struct blasfeo_svec *sa, int ai)
	{
	float *pa = sa->pa + ai;
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
void blasfeo_unpack_smat(int m, int n, struct blasfeo_smat *sA, int ai, int aj, float *A, int lda)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_unpack_smat(m, n, sA, ai, aj, A, lda);
#else
	printf("\nblasfeo_unpack_smat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert and transpose a matrix structure into a matrix
void blasfeo_unpack_tran_smat(int m, int n, struct blasfeo_smat *sA, int ai, int aj, float *A, int lda)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_unpack_tran_smat(m, n, sA, ai, aj, A, lda);
#else
	printf("\nblasfeo_unpack_tran_smat: feature not implemented yet\n");
	exit(1);
#endif
	}



// convert a vector structure into a vector
void blasfeo_unpack_svec(int m, struct blasfeo_svec *sa, int ai, float *x, int xi)
	{
	float *pa = sa->pa + ai;
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
void blasfeo_sgein1(float a, struct blasfeo_smat *sA, int ai, int aj)
	{

	if (ai==aj)
		{
		// invalidate stored inverse diagonal
		sA->use_dA = 0;
		}

	const int bs = S_PS;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	pA[0] = a;
	return;
	}



// extract element from strmat
float blasfeo_sgeex1(struct blasfeo_smat *sA, int ai, int aj)
	{
	const int bs = S_PS;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	return pA[0];
	}



// insert element into strvec
void blasfeo_svecin1(float a, struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	x[0] = a;
	return;
	}



// extract element from strvec
float blasfeo_svecex1(struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	return x[0];
	}



// set all elements of a strmat to a value
void blasfeo_sgese(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 16;
	int sda = sA->cn;
	float *pA = sA->pA + ai%bs + ai/bs*bs*sda + aj*bs;
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
	for(ii=0; ii<m-15; ii+=16)
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
			pA[8+jj*bs] = alpha;
			pA[9+jj*bs] = alpha;
			pA[10+jj*bs] = alpha;
			pA[11+jj*bs] = alpha;
			pA[12+jj*bs] = alpha;
			pA[13+jj*bs] = alpha;
			pA[14+jj*bs] = alpha;
			pA[15+jj*bs] = alpha;
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
void blasfeo_svecse(int m, float alpha, struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<m; ii++)
		x[ii] = alpha;
	return;
	}



// insert a vector into diagonal
void blasfeo_sdiain(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 16;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
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
	for(jj=0; jj<kmax-15; jj+=16)
		{
		pA[jj*sda+(jj+0)*bs+0] = alpha*x[jj+0];
		pA[jj*sda+(jj+1)*bs+1] = alpha*x[jj+1];
		pA[jj*sda+(jj+2)*bs+2] = alpha*x[jj+2];
		pA[jj*sda+(jj+3)*bs+3] = alpha*x[jj+3];
		pA[jj*sda+(jj+4)*bs+4] = alpha*x[jj+4];
		pA[jj*sda+(jj+5)*bs+5] = alpha*x[jj+5];
		pA[jj*sda+(jj+6)*bs+6] = alpha*x[jj+6];
		pA[jj*sda+(jj+7)*bs+7] = alpha*x[jj+7];
		pA[jj*sda+(jj+8)*bs+8] = alpha*x[jj+8];
		pA[jj*sda+(jj+9)*bs+9] = alpha*x[jj+9];
		pA[jj*sda+(jj+10)*bs+10] = alpha*x[jj+10];
		pA[jj*sda+(jj+11)*bs+11] = alpha*x[jj+11];
		pA[jj*sda+(jj+12)*bs+12] = alpha*x[jj+12];
		pA[jj*sda+(jj+13)*bs+13] = alpha*x[jj+13];
		pA[jj*sda+(jj+14)*bs+14] = alpha*x[jj+14];
		pA[jj*sda+(jj+15)*bs+15] = alpha*x[jj+15];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+(jj+ll)*bs+ll] = alpha*x[jj+ll];
		}
	return;
	}



// add scalar to diagonal
void blasfeo_sdiare(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 16;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
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
	for(jj=0; jj<kmax-15; jj+=16)
		{
		pA[jj*sda+(jj+0)*bs+0] += alpha;
		pA[jj*sda+(jj+1)*bs+1] += alpha;
		pA[jj*sda+(jj+2)*bs+2] += alpha;
		pA[jj*sda+(jj+3)*bs+3] += alpha;
		pA[jj*sda+(jj+4)*bs+4] += alpha;
		pA[jj*sda+(jj+5)*bs+5] += alpha;
		pA[jj*sda+(jj+6)*bs+6] += alpha;
		pA[jj*sda+(jj+7)*bs+7] += alpha;
		pA[jj*sda+(jj+8)*bs+8] += alpha;
		pA[jj*sda+(jj+9)*bs+9] += alpha;
		pA[jj*sda+(jj+10)*bs+10] += alpha;
		pA[jj*sda+(jj+11)*bs+11] += alpha;
		pA[jj*sda+(jj+12)*bs+12] += alpha;
		pA[jj*sda+(jj+13)*bs+13] += alpha;
		pA[jj*sda+(jj+14)*bs+14] += alpha;
		pA[jj*sda+(jj+15)*bs+15] += alpha;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+(jj+ll)*bs+ll] += alpha;
		}
	return;
	}



// swap two rows of two matrix structs
void blasfeo_srowsw(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_srowsw(kmax, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_srowsw: feature not implemented yet\n");
	exit(1);
#endif
	}



// extract a row int a vector
void blasfeo_srowex(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_srowex(kmax, alpha, sA, ai, aj, sx, xi);
#else
	printf("\nblasfeo_srowex: feature not implemented yet\n");
	exit(1);
#endif
	}



// insert a vector into a row
void blasfeo_srowin(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_srowin(kmax, alpha, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_srowin: feature not implemented yet\n");
	exit(1);
#endif
	}



// add a vector to a row
void blasfeo_srowad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_srowad(kmax, alpha, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_srowad: feature not implemented yet\n");
	exit(1);
#endif
	}



// extract vector from column
void blasfeo_scolex(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_scolex(kmax, sA, ai, aj, sx, xi);
#else
	printf("\nblasfeo_scolex: feature not implemented yet\n");
	exit(1);
#endif
	}



// insert as vector as a column
void blasfeo_scolin(int kmax, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_scolin(kmax, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_scolin: feature not implemented yet\n");
	exit(1);
#endif
	}



// add scaled vector to column
void blasfeo_scolad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_scolad(kmax, alpha, sx, xi, sA, ai, aj);
#else
	printf("\nblasfeo_scolad: feature not implemented yet\n");
	exit(1);
#endif
	}



// scale a column
void blasfeo_scolsc(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 16;

	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;

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
	for(jj=0; jj<kmax-15; jj+=16)
		{
		pA[jj*sda+0] *= alpha;
		pA[jj*sda+1] *= alpha;
		pA[jj*sda+2] *= alpha;
		pA[jj*sda+3] *= alpha;
		pA[jj*sda+4] *= alpha;
		pA[jj*sda+5] *= alpha;
		pA[jj*sda+6] *= alpha;
		pA[jj*sda+7] *= alpha;
		pA[jj*sda+8] *= alpha;
		pA[jj*sda+9] *= alpha;
		pA[jj*sda+10] *= alpha;
		pA[jj*sda+11] *= alpha;
		pA[jj*sda+12] *= alpha;
		pA[jj*sda+13] *= alpha;
		pA[jj*sda+14] *= alpha;
		pA[jj*sda+15] *= alpha;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+ll] *= alpha;
		}

	return;
	}



// swap two cols of two matrix structs
void blasfeo_scolsw(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_scolsw(kmax, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_scolsw: feature not implemented yet\n");
	exit(1);
#endif
	}



// permute the cols of a matrix struct
void blasfeo_scolpe(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			blasfeo_scolsw(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// inverse permute the cols of a matrix struct
void blasfeo_scolpei(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			blasfeo_scolsw(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// copy a generic strmat into a generic strmat
void blasfeo_sgecp(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgecp(m, n, sA, ai, aj, sB, bi, bj);
#else
	printf("\nblasfeo_sgecp: feature not implemented yet\n");
	exit(1);
#endif
	}



// copy a lower triangular strmat into a lower triangular strmat
void blasfeo_strcp_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strcp_l(m, sA, ai, aj, sB, bi, bj);
#else
	printf("\nblasfeo_strcp_l: feature not implemented yet\n");
	exit(1);
#endif
	}



// copy and scale a generic strmat into a generic strmat
void blasfeo_sgecpsc(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgecpsc(m, n, alpha, sA, ai, aj, sB, bi, bj);
#else
	printf("\nblasfeo_sgecpsc: feature not implemented yet\n");
	exit(1);
#endif
	}



#if 0
// copy  and scale a lower triangular strmat into a lower triangular strmat
void blasfeo_strcpsc_l(int m, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strcpsc_l(m, alpha, sA, ai, aj, sB, bi, bj);
#else
	printf("\nblasfeo_strcpsc_l: feature not implemented yet\n");
	exit(1);
#endif
	}
#endif



// scale a generic strmat
void blasfeo_sgesc(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgesc(m, n, alpha, sA, ai, aj);
#else
	printf("\nblasfeo_sgesc: feature not implemented yet\n");
	exit(1);
#endif
	}



#if 0
// scale a triangular strmat
void blasfeo_strsc_l(int m, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strsc_l(m, alpha, sA, ai, aj);
#else
	printf("\nblasfeo_strsc_l: feature not implemented yet\n");
	exit(1);
#endif
	}
#endif



// copy a strvec into a strvec
void blasfeo_sveccp(int m, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float *pa = sa->pa + ai;
	float *pc = sc->pa + ci;
#if 0
	kernel_sveccp_inc1(m, pa, pc);
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
void blasfeo_svecsc(int m, float alpha, struct blasfeo_svec *sa, int ai)
	{
	float *pa = sa->pa + ai;
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
void blasfeo_sveccpsc(int m, float alpha, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float *pa = sa->pa + ai;
	float *pc = sc->pa + ci;
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
void blasfeo_sgead(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgead(m, n, alpha, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_sgead: feature not implemented yet\n");
	exit(1);
#endif
	}



// scales and adds a strvec into a strvec
void blasfeo_svecad(int m, float alpha, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float *pa = sa->pa + ai;
	float *pc = sc->pa + ci;
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
void blasfeo_sgetr(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sgetr(m, n, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_sgetr: feature not implemented yet\n");
	exit(1);
#endif
	}



// copy and transpose a lower triangular strmat into an upper triangular strmat
void blasfeo_strtr_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strtr_l(m, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_strtr_l: feature not implemented yet\n");
	exit(1);
#endif
	}



// copy and transpose an upper triangular strmat into a lower triangular strmat
void blasfeo_strtr_u(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strtr_u(m, sA, ai, aj, sC, ci, cj);
#else
	printf("\nblasfeo_strtr_u: feature not implemented yet\n");
	exit(1);
#endif
	}



// insert a strvec to diagonal of strmat, sparse formulation
void blasfeo_sdiain_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = S_PS;
	float *x = sx->pa + xi;
	int sdd = sD->cn;
	float *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs] = alpha * x[jj];
		}
	return;
	}



// extract a vector from diagonal
void blasfeo_sdiaex(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
	const int bs = 16;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
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
	for(jj=0; jj<kmax-15; jj+=16)
		{
		x[jj+0] = alpha*pA[jj*sda+(jj+0)*bs+0];
		x[jj+1] = alpha*pA[jj*sda+(jj+1)*bs+1];
		x[jj+2] = alpha*pA[jj*sda+(jj+2)*bs+2];
		x[jj+3] = alpha*pA[jj*sda+(jj+3)*bs+3];
		x[jj+4] = alpha*pA[jj*sda+(jj+4)*bs+4];
		x[jj+5] = alpha*pA[jj*sda+(jj+5)*bs+5];
		x[jj+6] = alpha*pA[jj*sda+(jj+6)*bs+6];
		x[jj+7] = alpha*pA[jj*sda+(jj+7)*bs+7];
		x[jj+8] = alpha*pA[jj*sda+(jj+8)*bs+8];
		x[jj+9] = alpha*pA[jj*sda+(jj+9)*bs+9];
		x[jj+10] = alpha*pA[jj*sda+(jj+10)*bs+10];
		x[jj+11] = alpha*pA[jj*sda+(jj+11)*bs+11];
		x[jj+12] = alpha*pA[jj*sda+(jj+12)*bs+12];
		x[jj+13] = alpha*pA[jj*sda+(jj+13)*bs+13];
		x[jj+14] = alpha*pA[jj*sda+(jj+14)*bs+14];
		x[jj+15] = alpha*pA[jj*sda+(jj+15)*bs+15];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[jj+ll] = alpha*pA[jj*sda+(jj+ll)*bs+ll];
		}
	return;
	}



// extract the diagonal of a strmat to a strvec, sparse formulation
void blasfeo_sdiaex_sp(int kmax, float alpha, int *idx, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_svec *sx, int xi)
	{
	const int bs = S_PS;
	float *x = sx->pa + xi;
	int sdd = sD->cn;
	float *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		x[jj] = alpha * pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs];
		}
	return;
	}



// add a vector to diagonal
void blasfeo_sdiaad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 16;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
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
	for(jj=0; jj<kmax-15; jj+=16)
		{
		pA[jj*sda+(jj+0)*bs+0] += alpha*x[jj+0];
		pA[jj*sda+(jj+1)*bs+1] += alpha*x[jj+1];
		pA[jj*sda+(jj+2)*bs+2] += alpha*x[jj+2];
		pA[jj*sda+(jj+3)*bs+3] += alpha*x[jj+3];
		pA[jj*sda+(jj+4)*bs+4] += alpha*x[jj+4];
		pA[jj*sda+(jj+5)*bs+5] += alpha*x[jj+5];
		pA[jj*sda+(jj+6)*bs+6] += alpha*x[jj+6];
		pA[jj*sda+(jj+7)*bs+7] += alpha*x[jj+7];
		pA[jj*sda+(jj+8)*bs+8] += alpha*x[jj+8];
		pA[jj*sda+(jj+9)*bs+9] += alpha*x[jj+9];
		pA[jj*sda+(jj+10)*bs+10] += alpha*x[jj+10];
		pA[jj*sda+(jj+11)*bs+11] += alpha*x[jj+11];
		pA[jj*sda+(jj+12)*bs+12] += alpha*x[jj+12];
		pA[jj*sda+(jj+13)*bs+13] += alpha*x[jj+13];
		pA[jj*sda+(jj+14)*bs+14] += alpha*x[jj+14];
		pA[jj*sda+(jj+15)*bs+15] += alpha*x[jj+15];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pA[jj*sda+(jj+ll)*bs+ll] += alpha*x[jj+ll];
		}
	return;
	}



// add scaled strvec to diagonal of strmat, sparse formulation
void blasfeo_sdiaad_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = S_PS;
	float *x = sx->pa + xi;
	int sdd = sD->cn;
	float *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs] += alpha * x[jj];
		}
	return;
	}



// add scaled strvec to another strvec and insert to diagonal of strmat, sparse formulation
void blasfeo_sdiaadin_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = S_PS;
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	int sdd = sD->cn;
	float *pD = sD->pA;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[(ii+di)/bs*bs*sdd+(ii+di)%bs+(ii+dj)*bs] = y[jj] + alpha * x[jj];
		}
	return;
	}



// add scaled strvec to row of strmat, sparse formulation
void blasfeo_srowad_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_srowad_sp(kmax, alpha, sx, xi, idx, sD, di, dj);
#else
	printf("\nblasfeo_srowad_sp: feature not implemented yet\n");
	exit(1);
#endif
	}



// add scaled strvec to strvec, sparse formulation
void blasfeo_svecad_sp(int m, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] += alpha * x[ii];
	return;
	}



// insert scaled strvec to strvec, sparse formulation
void blasfeo_svecin_sp(int m, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] = alpha * x[ii];
	return;
	}



// extract scaled strvec to strvec, sparse formulation
void blasfeo_svecex_sp(int m, float alpha, int *idx, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[ii] = alpha * x[idx[ii]];
	return;
	}



// z += alpha * x[idx]
void blasfeo_svecadd_sp_in(int m, float alpha, int *idx, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[ii] += alpha * x[idx[ii]];
	return;
	}



// clip strvec between two strvec
void blasfeo_sveccl(int m, struct blasfeo_svec *sxm, int xim, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sxp, int xip, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sveccl(m, sxm, xim, sx, xi, sxp, xip, sz, zi);
#else
	printf("\nblasfeo_sveccl: feature not implemented yet\n");
	exit(1);
#endif
	}



// clip strvec between two strvec, with mask
void blasfeo_sveccl_mask(int m, struct blasfeo_svec *sxm, int xim, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sxp, int xip, struct blasfeo_svec *sz, int zi, struct blasfeo_svec *sm, int mi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_sveccl_mask(m, sxm, xim, sx, xi, sxp, xip, sz, zi, sm, mi);
#else
	printf("\nblasfeo_sveccl_mask: feature not implemented yet\n");
	exit(1);
#endif
	}



// zero out strvec to strvec with mask
void blasfeo_svecze(int m, struct blasfeo_svec *sm, int mi, struct blasfeo_svec *sv, int vi, struct blasfeo_svec *se, int ei)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_svecze(m, sm, mi, sv, vi, se, ei);
#else
	printf("\nblasfeo_svecze: feature not implemented yet\n");
	exit(1);
#endif
	}



// compute inf norm of vector
void blasfeo_svecnrm_inf(int m, struct blasfeo_svec *sx, int xi, float *ptr_norm)
	{
	int ii;
	float *x = sx->pa + xi;
	float norm = 0.0;
	float tmp;
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
void blasfeo_svecnrm_2(int m, struct blasfeo_svec *sx, int xi, float *ptr_norm)
	{
	int ii;
	float *x = sx->pa + xi;
	float norm = 0.0;
	for(ii=0; ii<m; ii++)
		{
		norm += x[ii]*x[ii];
		}
	norm = sqrt(norm);
	*ptr_norm = norm;
	return;
	}



// permute elements of a vector struct
void blasfeo_svecpe(int kmax, int *ipiv, struct blasfeo_svec *sx, int xi)
	{
	int ii;
	float tmp;
	float *x = sx->pa + xi;
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
void blasfeo_svecpei(int kmax, int *ipiv, struct blasfeo_svec *sx, int xi)
	{
	int ii;
	float tmp;
	float *x = sx->pa + xi;
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

