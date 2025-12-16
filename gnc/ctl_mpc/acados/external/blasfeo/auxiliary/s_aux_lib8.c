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
#include <blasfeo_s_kernel.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_s_aux_ref.h>
#endif



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



// regularize diagonal
void sdiareg_lib(int kmax, float reg, int offset, float *pD, int sdd)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	float *pD2;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] += reg;
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		kmax -= kna;
		}
	pD2 = pD;
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pD2[0+0*bs] += reg;
		pD2[1+1*bs] += reg;
		pD2[2+2*bs] += reg;
		pD2[3+3*bs] += reg;
		pD2[4+4*bs] += reg;
		pD2[5+5*bs] += reg;
		pD2[6+6*bs] += reg;
		pD2[7+7*bs] += reg;
		pD2 += bs*sdd+bs*bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] += reg;
		}

	}



// insert vector to diagonal
void sdiain_lib(int kmax, float alpha, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	float *pD2, *x2;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] = alpha*x[ll];
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	pD2 = pD;
	x2 = x;
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pD2[0+bs*0] = alpha*x2[0];
		pD2[1+bs*1] = alpha*x2[1];
		pD2[2+bs*2] = alpha*x2[2];
		pD2[3+bs*3] = alpha*x2[3];
		pD2[4+bs*4] = alpha*x2[4];
		pD2[5+bs*5] = alpha*x2[5];
		pD2[6+bs*6] = alpha*x2[6];
		pD2[7+bs*7] = alpha*x2[7];
		pD2 += bs*sdd+bs*bs;
		x2 += bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] = alpha*x[jj+ll];
		}

	}



// add scalar to diagonal
void blasfeo_sdiare(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{
	const int bs = 8;
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




// insert sqrt of vector to diagonal
void sdiain_sqrt_lib(int kmax, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	float *pD2, *x2;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] = sqrt(x[ll]);
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	pD2 = pD;
	x2 = x;
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pD2[0+bs*0] = sqrt(x2[0]);
		pD2[1+bs*1] = sqrt(x2[1]);
		pD2[2+bs*2] = sqrt(x2[2]);
		pD2[3+bs*3] = sqrt(x2[3]);
		pD2[4+bs*4] = sqrt(x2[4]);
		pD2[5+bs*5] = sqrt(x2[5]);
		pD2[5+bs*6] = sqrt(x2[6]);
		pD2[7+bs*7] = sqrt(x2[7]);
		pD2 += bs*sdd+bs*bs;
		x2 += bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] = sqrt(x[jj+ll]);
		}

	}



// extract diagonal to vector
void sdiaex_lib(int kmax, float alpha, int offset, float *pD, int sdd, float *x)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	float *pD2, *x2;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			x[ll] = alpha * pD[ll+bs*ll];
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	pD2 = pD;
	x2 = x;
	for(jj=0; jj<kmax-7; jj+=8)
		{
		x2[0] = alpha * pD2[0+bs*0];
		x2[1] = alpha * pD2[1+bs*1];
		x2[2] = alpha * pD2[2+bs*2];
		x2[3] = alpha * pD2[3+bs*3];
		x2[4] = alpha * pD2[4+bs*4];
		x2[5] = alpha * pD2[5+bs*5];
		x2[6] = alpha * pD2[6+bs*6];
		x2[7] = alpha * pD2[7+bs*7];
		pD2 += bs*sdd+bs*bs;
		x2 += bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[jj+ll] = alpha * pD[jj*sdd+(jj+ll)*bs+ll];
		}

	}



// add scaled vector to diagonal
void sdiaad_lib(int kmax, float alpha, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	float *pD2, *x2;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] += alpha * x[ll];
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	pD2 = pD;
	x2 = x;
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pD2[0+bs*0] += alpha * x2[0];
		pD2[1+bs*1] += alpha * x2[1];
		pD2[2+bs*2] += alpha * x2[2];
		pD2[3+bs*3] += alpha * x2[3];
		pD2[4+bs*4] += alpha * x2[4];
		pD2[5+bs*5] += alpha * x2[5];
		pD2[6+bs*6] += alpha * x2[6];
		pD2[7+bs*7] += alpha * x2[7];
		pD2 += bs*sdd+bs*bs;
		x2 += bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] += alpha * x[jj+ll];
		}
	return;
	}



// insert vector to diagonal, sparse formulation
void sdiain_libsp(int kmax, int *idx, float alpha, float *x, float *pD, int sdd)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] = alpha * x[jj];
		}
	return;
	}



// extract diagonal to vector, sparse formulation
void sdiaex_libsp(int kmax, int *idx, float alpha, float *pD, int sdd, float *x)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		x[jj] = alpha * pD[ii/bs*bs*sdd+ii%bs+ii*bs];
		}
	return;
	}



// add scaled vector to diagonal, sparse formulation
void sdiaad_libsp(int kmax, int *idx, float alpha, float *x, float *pD, int sdd)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] += alpha * x[jj];
		}
	return;
	}



// add scaled vector to another vector and insert to diagonal, sparse formulation
void sdiaadin_libsp(int kmax, int *idx, float alpha, float *x, float *y, float *pD, int sdd)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] = y[jj] + alpha * x[jj];
		}
	return;
	}



// insert vector to row
void srowin_lib(int kmax, float alpha, float *x, float *pD)
	{

	const int bs = 8;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[0*bs] = alpha * x[0];
		pD[1*bs] = alpha * x[1];
		pD[2*bs] = alpha * x[2];
		pD[3*bs] = alpha * x[3];
		pD += 4*bs;
		x += 4;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[ll*bs] = alpha*x[ll];
		}
	return;
	}



// extract row to vector
void srowex_lib(int kmax, float alpha, float *pD, float *x)
	{

	const int bs = 8;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		x[0] = alpha * pD[0*bs];
		x[1] = alpha * pD[1*bs];
		x[2] = alpha * pD[2*bs];
		x[3] = alpha * pD[3*bs];
		pD += 4*bs;
		x += 4;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[ll] = alpha*pD[ll*bs];
		}
	return;
	}



// add scaled vector to row
void srowad_lib(int kmax, float alpha, float *x, float *pD)
	{

	const int bs = 8;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[0*bs] += alpha * x[0];
		pD[1*bs] += alpha * x[1];
		pD[2*bs] += alpha * x[2];
		pD[3*bs] += alpha * x[3];
		pD += 4*bs;
		x += 4;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[ll*bs] += alpha * x[ll];
		}
	return;
	}



// insert vector to row, sparse formulation
void srowin_libsp(int kmax, float alpha, int *idx, float *x, float *pD)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] = alpha*x[jj];
		}
	return;
	}



// add scaled vector to row, sparse formulation
void srowad_libsp(int kmax, int *idx, float alpha, float *x, float *pD)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] += alpha * x[jj];
		}
	return;
	}



// add scaled vector to another vector and insert to row, sparse formulation
void srowadin_libsp(int kmax, int *idx, float alpha, float *x, float *y, float *pD)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] = y[jj] + alpha * x[jj];
		}
	return;
	}



// swap two rows
void srowsw_lib(int kmax, float *pA, float *pC)
	{

	const int bs = 8;

	int ii;
	float tmp;

	for(ii=0; ii<kmax-3; ii+=4)
		{
		tmp = pA[0+bs*0];
		pA[0+bs*0] = pC[0+bs*0];
		pC[0+bs*0] = tmp;
		tmp = pA[0+bs*1];
		pA[0+bs*1] = pC[0+bs*1];
		pC[0+bs*1] = tmp;
		tmp = pA[0+bs*2];
		pA[0+bs*2] = pC[0+bs*2];
		pC[0+bs*2] = tmp;
		tmp = pA[0+bs*3];
		pA[0+bs*3] = pC[0+bs*3];
		pC[0+bs*3] = tmp;
		pA += 4*bs;
		pC += 4*bs;
		}
	for( ; ii<kmax; ii++)
		{
		tmp = pA[0+bs*0];
		pA[0+bs*0] = pC[0+bs*0];
		pC[0+bs*0] = tmp;
		pA += 1*bs;
		pC += 1*bs;
		}
	return;
	}



// extract vector from column
void scolex_lib(int kmax, int offset, float *pD, int sdd, float *x)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			x[ll] = pD[ll];
			}
		pD += kna + bs*(sdd-1);
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		x[0] = pD[0];
		x[1] = pD[1];
		x[2] = pD[2];
		x[3] = pD[3];
		x[4] = pD[4];
		x[5] = pD[5];
		x[6] = pD[6];
		x[7] = pD[7];
		pD += bs*sdd;
		x += bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[ll] = pD[ll];
		}

	}



// insert vector to column
void scolin_lib(int kmax, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll] = x[ll];
			}
		pD += kna + bs*(sdd-1);
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pD[0] = x[0];
		pD[1] = x[1];
		pD[2] = x[2];
		pD[3] = x[3];
		pD[4] = x[4];
		pD[5] = x[5];
		pD[6] = x[6];
		pD[7] = x[7];
		pD += bs*sdd;
		x += bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[ll] = x[ll];
		}

	}



// add scaled vector to column
void scolad_lib(int kmax, float alpha, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 8;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll] += alpha * x[ll];
			}
		pD += kna + bs*(sdd-1);
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-7; jj+=8)
		{
		pD[0] += alpha * x[0];
		pD[1] += alpha * x[1];
		pD[2] += alpha * x[2];
		pD[3] += alpha * x[3];
		pD[4] += alpha * x[4];
		pD[5] += alpha * x[5];
		pD[6] += alpha * x[6];
		pD[7] += alpha * x[7];
		pD += bs*sdd;
		x += bs;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[ll] += alpha * x[ll];
		}

	}



// insert vector to diagonal, sparse formulation
void scolin_libsp(int kmax, int *idx, float *x, float *pD, int sdd)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs] = x[jj];
		}

	}



// add scaled vector to diagonal, sparse formulation
void scolad_libsp(int kmax, float alpha, int *idx, float *x, float *pD, int sdd)
	{

	const int bs = 8;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs] += alpha * x[jj];
		}

	}



// swaps two cols
void scolsw_lib(int kmax, int offsetA, float *pA, int sda, int offsetC, float *pC, int sdc)
	{

	const int bs = 8;

	int ii;

	float tmp;

	if(offsetA==offsetC)
		{
		if(offsetA>0)
			{
			ii = 0;
			for(; ii<bs-offsetA; ii++)
				{
				tmp = pA[0+bs*0];
				pA[0+bs*0] = pC[0+bs*0];
				pC[0+bs*0] = tmp;
				pA += 1;
				pC += 1;
				}
			pA += bs*(sda-1);
			pC += bs*(sdc-1);
			kmax -= bs-offsetA;
			}
		ii = 0;
		for(; ii<kmax-7; ii+=8)
			{
			tmp = pA[0+bs*0];
			pA[0+bs*0] = pC[0+bs*0];
			pC[0+bs*0] = tmp;
			tmp = pA[1+bs*0];
			pA[1+bs*0] = pC[1+bs*0];
			pC[1+bs*0] = tmp;
			tmp = pA[2+bs*0];
			pA[2+bs*0] = pC[2+bs*0];
			pC[2+bs*0] = tmp;
			tmp = pA[3+bs*0];
			pA[3+bs*0] = pC[3+bs*0];
			pC[3+bs*0] = tmp;
			tmp = pA[4+bs*0];
			pA[4+bs*0] = pC[4+bs*0];
			pC[4+bs*0] = tmp;
			tmp = pA[5+bs*0];
			pA[5+bs*0] = pC[5+bs*0];
			pC[5+bs*0] = tmp;
			tmp = pA[6+bs*0];
			pA[6+bs*0] = pC[6+bs*0];
			pC[6+bs*0] = tmp;
			tmp = pA[7+bs*0];
			pA[7+bs*0] = pC[7+bs*0];
			pC[7+bs*0] = tmp;
			pA += bs*sda;
			pC += bs*sdc;
			}
		for(; ii<kmax; ii++)
			{
			tmp = pA[0+bs*0];
			pA[0+bs*0] = pC[0+bs*0];
			pC[0+bs*0] = tmp;
			pA += 1;
			pC += 1;
			}
		}
	else
		{
#if defined(EXT_DEP)
		printf("\nscolsw: feature not implemented yet: offsetA!=offsetC\n\n");
#endif
		exit(1);
		}

	return;

	}



// insert vector to vector, sparse formulation
void svecin_libsp(int kmax, int *idx, float *x, float *y)
	{

	int jj;

	for(jj=0; jj<kmax; jj++)
		{
		y[idx[jj]] = x[jj];
		}

	}



// adds vector to vector, sparse formulation
void svecad_libsp(int kmax, int *idx, float alpha, float *x, float *y)
	{

	int jj;

	for(jj=0; jj<kmax; jj++)
		{
		y[idx[jj]] += alpha * x[jj];
		}

	}



/****************************
* new interface
****************************/



#if defined(LA_HIGH_PERFORMANCE)



// return the memory size (in bytes) needed for a strmat
size_t blasfeo_memsize_smat(int m, int n)
	{
	const int bs = 8;
	int nc = S_PLD;
	int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	int cn = (n+nc-1)/nc*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = (pm*cn+tmp)*sizeof(float);
	memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	return memsize;
	}



size_t blasfeo_memsize_smat_ps(int ps, int m, int n)
	{
	int nc = S_PLD;
	int al = ps*nc;
	int pm = (m+ps-1)/ps*ps;
	int cn = (n+nc-1)/nc*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = (pm*cn+tmp)*sizeof(float);
	memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	return memsize;
	}



// return the memory size (in bytes) needed for the digonal of a strmat
size_t blasfeo_memsize_diag_smat(int m, int n)
	{
	const int bs = 8;
	int nc = S_PLD;
	int al = bs*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = tmp*sizeof(float);
	return memsize;
	}



// create a matrix structure for a matrix of size m*n by using memory passed by a pointer
void blasfeo_create_smat(int m, int n, struct blasfeo_smat *sA, void *memory)
	{
	sA->mem = memory;
	const int bs = 8;
	int nc = S_PLD;
	int al = bs*nc;
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
	sA->use_dA = 0;
	size_t memsize = (pm*cn+tmp)*sizeof(float);
	sA->memsize = (memsize + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
	sA->use_dA = 0; // invalidate stored inverse diagonal
	return;
	}



void blasfeo_create_smat_ps(int ps, int m, int n, struct blasfeo_smat *sA, void *memory)
	{
	sA->mem = memory;
	int nc = S_PLD;
	int al = ps*nc;
	sA->m = m;
	sA->n = n;
	int pm = (m+ps-1)/ps*ps;
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
	const int bs = 8;
//	int nc = S_PLD;
//	int al = bs*nc;
	int pm = (m+bs-1)/bs*bs;
	size_t memsize = pm*sizeof(float);
	return memsize;
	}



// create a vector structure for a vector of size m by using memory passed by a pointer
void blasfeo_create_svec(int m, struct blasfeo_svec *sa, void *memory)
	{
	sa->mem = memory;
	const int bs = 8;
//	int nc = S_PLD;
//	int al = bs*nc;
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

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, j, jj, m0, m1, m2;
	float *B, *pB;
	m0 = (bs-ai%bs)%bs;
	if(m0>m)
		m0 = m;
	m1 = m - m0;
	jj = 0;
	for( ; jj<n-3; jj+=4)
		{
		B  =  A + jj*lda;
		pB = pA + jj*bs;
		ii = 0;
		if(m0>0)
			{
			for( ; ii<m0; ii++)
				{
				pB[ii+bs*0] = B[ii+lda*0];
				pB[ii+bs*1] = B[ii+lda*1];
				pB[ii+bs*2] = B[ii+lda*2];
				pB[ii+bs*3] = B[ii+lda*3];
				}
			B  += m0;
			pB += m0 + bs*(sda-1);
			}
		for( ; ii<m-7; ii+=8)
			{
			// unroll 0
			pB[0+bs*0] = B[0+lda*0];
			pB[1+bs*0] = B[1+lda*0];
			pB[2+bs*0] = B[2+lda*0];
			pB[3+bs*0] = B[3+lda*0];
			pB[4+bs*0] = B[4+lda*0];
			pB[5+bs*0] = B[5+lda*0];
			pB[6+bs*0] = B[6+lda*0];
			pB[7+bs*0] = B[7+lda*0];
			// unroll 1
			pB[0+bs*1] = B[0+lda*1];
			pB[1+bs*1] = B[1+lda*1];
			pB[2+bs*1] = B[2+lda*1];
			pB[3+bs*1] = B[3+lda*1];
			pB[4+bs*1] = B[4+lda*1];
			pB[5+bs*1] = B[5+lda*1];
			pB[6+bs*1] = B[6+lda*1];
			pB[7+bs*1] = B[7+lda*1];
			// unroll 2
			pB[0+bs*2] = B[0+lda*2];
			pB[1+bs*2] = B[1+lda*2];
			pB[2+bs*2] = B[2+lda*2];
			pB[3+bs*2] = B[3+lda*2];
			pB[4+bs*2] = B[4+lda*2];
			pB[5+bs*2] = B[5+lda*2];
			pB[6+bs*2] = B[6+lda*2];
			pB[7+bs*2] = B[7+lda*2];
			// unroll 3
			pB[0+bs*3] = B[0+lda*3];
			pB[1+bs*3] = B[1+lda*3];
			pB[2+bs*3] = B[2+lda*3];
			pB[3+bs*3] = B[3+lda*3];
			pB[4+bs*3] = B[4+lda*3];
			pB[5+bs*3] = B[5+lda*3];
			pB[6+bs*3] = B[6+lda*3];
			pB[7+bs*3] = B[7+lda*3];
			// update
			B  += 8;
			pB += bs*sda;
			}
		for( ; ii<m; ii++)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			// col 1
			pB[0+bs*1] = B[0+lda*1];
			// col 2
			pB[0+bs*2] = B[0+lda*2];
			// col 3
			pB[0+bs*3] = B[0+lda*3];
			// update
			B  += 1;
			pB += 1;
			}
		}
	for( ; jj<n; jj++)
		{

		B  =  A + jj*lda;
		pB = pA + jj*bs;

		ii = 0;
		if(m0>0)
			{
			for( ; ii<m0; ii++)
				{
				pB[ii+bs*0] = B[ii+lda*0];
				}
			B  += m0;
			pB += m0 + bs*(sda-1);
			}
		for( ; ii<m-7; ii+=8)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			pB[1+bs*0] = B[1+lda*0];
			pB[2+bs*0] = B[2+lda*0];
			pB[3+bs*0] = B[3+lda*0];
			pB[4+bs*0] = B[4+lda*0];
			pB[5+bs*0] = B[5+lda*0];
			pB[6+bs*0] = B[6+lda*0];
			pB[7+bs*0] = B[7+lda*0];
			// update
			B  += 8;
			pB += bs*sda;
			}
		for( ; ii<m; ii++)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			// update
			B  += 1;
			pB += 1;
			}
		}
	return;
	}



// convert and transpose a matrix into a matrix structure
void blasfeo_pack_tran_smat(int m, int n, float *A, int lda, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, j, m0, m1, m2;
	float 	*B, *pB;
	m0 = (bs-ai%bs)%bs;
	if(m0>n)
		m0 = n;
	m1 = n - m0;
	ii = 0;
	if(m0>0)
		{
		for(j=0; j<m; j++)
			{
			for(i=0; i<m0; i++)
				{
				pA[i+j*bs+ii*sda] = A[j+(i+ii)*lda];
				}
			}
		A  += m0*lda;
		pA += m0 + bs*(sda-1);
		}
	ii = 0;
	for(; ii<m1-7; ii+=bs)
		{
		j=0;
		B  = A + ii*lda;
		pB = pA + ii*sda;
		for(; j<m-3; j+=4)
			{
			// unroll 0
			pB[0+0*bs] = B[0+0*lda];
			pB[1+0*bs] = B[0+1*lda];
			pB[2+0*bs] = B[0+2*lda];
			pB[3+0*bs] = B[0+3*lda];
			pB[4+0*bs] = B[0+4*lda];
			pB[5+0*bs] = B[0+5*lda];
			pB[6+0*bs] = B[0+6*lda];
			pB[7+0*bs] = B[0+7*lda];
			// unroll 1
			pB[0+1*bs] = B[1+0*lda];
			pB[1+1*bs] = B[1+1*lda];
			pB[2+1*bs] = B[1+2*lda];
			pB[3+1*bs] = B[1+3*lda];
			pB[4+1*bs] = B[1+4*lda];
			pB[5+1*bs] = B[1+5*lda];
			pB[6+1*bs] = B[1+6*lda];
			pB[7+1*bs] = B[1+7*lda];
			// unroll 2
			pB[0+2*bs] = B[2+0*lda];
			pB[1+2*bs] = B[2+1*lda];
			pB[2+2*bs] = B[2+2*lda];
			pB[3+2*bs] = B[2+3*lda];
			pB[4+2*bs] = B[2+4*lda];
			pB[5+2*bs] = B[2+5*lda];
			pB[6+2*bs] = B[2+6*lda];
			pB[7+2*bs] = B[2+7*lda];
			// unroll 3
			pB[0+3*bs] = B[3+0*lda];
			pB[1+3*bs] = B[3+1*lda];
			pB[2+3*bs] = B[3+2*lda];
			pB[3+3*bs] = B[3+3*lda];
			pB[4+3*bs] = B[3+4*lda];
			pB[5+3*bs] = B[3+5*lda];
			pB[6+3*bs] = B[3+6*lda];
			pB[7+3*bs] = B[3+7*lda];
			B  += 4;
			pB += 4*bs;
			}
		for(; j<m; j++)
			{
			// unroll 0
			pB[0+0*bs] = B[0+0*lda];
			pB[1+0*bs] = B[0+1*lda];
			pB[2+0*bs] = B[0+2*lda];
			pB[3+0*bs] = B[0+3*lda];
			pB[4+0*bs] = B[0+4*lda];
			pB[5+0*bs] = B[0+5*lda];
			pB[6+0*bs] = B[0+6*lda];
			pB[7+0*bs] = B[0+7*lda];
			B  += 1;
			pB += 1*bs;
			}
		}
	if(ii<m1)
		{
		m2 = m1-ii;
		if(bs<m2) m2 = bs;
		for(j=0; j<m; j++)
			{
			for(i=0; i<m2; i++)
				{
				pA[i+j*bs+ii*sda] = A[j+(i+ii)*lda];
				}
			}
		}
	return;
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
	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, jj;
	int m0 = (bs-ai%bs)%bs;
	if(m0>m)
		m0 = m;
	float *ptr_pA;
	jj=0;
	for(; jj<n-3; jj+=4)
		{
		ptr_pA = pA + jj*bs;
		ii = 0;
		if(m0>0)
			{
			for(; ii<m0; ii++)
				{
				// unroll 0
				A[ii+lda*(jj+0)] = ptr_pA[0+bs*0];
				// unroll 1
				A[ii+lda*(jj+1)] = ptr_pA[0+bs*1];
				// unroll 2
				A[ii+lda*(jj+2)] = ptr_pA[0+bs*2];
				// unroll 3
				A[ii+lda*(jj+3)] = ptr_pA[0+bs*3];
				ptr_pA++;
				}
			ptr_pA += (sda-1)*bs;
			}
		// TODO update A !!!!!
		for(; ii<m-bs+1; ii+=bs)
			{
			// unroll 0
			A[0+ii+lda*(jj+0)] = ptr_pA[0+bs*0];
			A[1+ii+lda*(jj+0)] = ptr_pA[1+bs*0];
			A[2+ii+lda*(jj+0)] = ptr_pA[2+bs*0];
			A[3+ii+lda*(jj+0)] = ptr_pA[3+bs*0];
			A[4+ii+lda*(jj+0)] = ptr_pA[4+bs*0];
			A[5+ii+lda*(jj+0)] = ptr_pA[5+bs*0];
			A[6+ii+lda*(jj+0)] = ptr_pA[6+bs*0];
			A[7+ii+lda*(jj+0)] = ptr_pA[7+bs*0];
			// unroll 0
			A[0+ii+lda*(jj+1)] = ptr_pA[0+bs*1];
			A[1+ii+lda*(jj+1)] = ptr_pA[1+bs*1];
			A[2+ii+lda*(jj+1)] = ptr_pA[2+bs*1];
			A[3+ii+lda*(jj+1)] = ptr_pA[3+bs*1];
			A[4+ii+lda*(jj+1)] = ptr_pA[4+bs*1];
			A[5+ii+lda*(jj+1)] = ptr_pA[5+bs*1];
			A[6+ii+lda*(jj+1)] = ptr_pA[6+bs*1];
			A[7+ii+lda*(jj+1)] = ptr_pA[7+bs*1];
			// unroll 0
			A[0+ii+lda*(jj+2)] = ptr_pA[0+bs*2];
			A[1+ii+lda*(jj+2)] = ptr_pA[1+bs*2];
			A[2+ii+lda*(jj+2)] = ptr_pA[2+bs*2];
			A[3+ii+lda*(jj+2)] = ptr_pA[3+bs*2];
			A[4+ii+lda*(jj+2)] = ptr_pA[4+bs*2];
			A[5+ii+lda*(jj+2)] = ptr_pA[5+bs*2];
			A[6+ii+lda*(jj+2)] = ptr_pA[6+bs*2];
			A[7+ii+lda*(jj+2)] = ptr_pA[7+bs*2];
			// unroll 0
			A[0+ii+lda*(jj+3)] = ptr_pA[0+bs*3];
			A[1+ii+lda*(jj+3)] = ptr_pA[1+bs*3];
			A[2+ii+lda*(jj+3)] = ptr_pA[2+bs*3];
			A[3+ii+lda*(jj+3)] = ptr_pA[3+bs*3];
			A[4+ii+lda*(jj+3)] = ptr_pA[4+bs*3];
			A[5+ii+lda*(jj+3)] = ptr_pA[5+bs*3];
			A[6+ii+lda*(jj+3)] = ptr_pA[6+bs*3];
			A[7+ii+lda*(jj+3)] = ptr_pA[7+bs*3];
			ptr_pA += sda*bs;
			}
		for(; ii<m; ii++)
			{
			// unroll 0
			A[ii+lda*(jj+0)] = ptr_pA[0+bs*0];
			// unroll 1
			A[ii+lda*(jj+1)] = ptr_pA[0+bs*1];
			// unroll 2
			A[ii+lda*(jj+2)] = ptr_pA[0+bs*2];
			// unroll 3
			A[ii+lda*(jj+3)] = ptr_pA[0+bs*3];
			ptr_pA++;
			}
		}
	for(; jj<n; jj++)
		{
		ptr_pA = pA + jj*bs;
		ii = 0;
		if(m0>0)
			{
			for(; ii<m0; ii++)
				{
				A[ii+lda*jj] = ptr_pA[0];
				ptr_pA++;
				}
			ptr_pA += (sda-1)*bs;
			}
		for(; ii<m-bs+1; ii+=bs)
			{
			A[0+ii+lda*(jj+0)] = ptr_pA[0+bs*0];
			A[1+ii+lda*(jj+0)] = ptr_pA[1+bs*0];
			A[2+ii+lda*(jj+0)] = ptr_pA[2+bs*0];
			A[3+ii+lda*(jj+0)] = ptr_pA[3+bs*0];
			A[4+ii+lda*(jj+0)] = ptr_pA[4+bs*0];
			A[5+ii+lda*(jj+0)] = ptr_pA[5+bs*0];
			A[6+ii+lda*(jj+0)] = ptr_pA[6+bs*0];
			A[7+ii+lda*(jj+0)] = ptr_pA[7+bs*0];
			ptr_pA += sda*bs;
			}
		for(; ii<m; ii++)
			{
			A[ii+lda*jj] = ptr_pA[0];
			ptr_pA++;
			}
		}
	return;
	}



// convert and transpose a matrix structure into a matrix
void blasfeo_unpack_tran_smat(int m, int n, struct blasfeo_smat *sA, int ai, int aj, float *A, int lda)
	{
	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, jj;
	int m0 = (bs-ai%bs)%bs;
	if(m0>m)
		m0 = m;
	float *ptr_pA;
	jj=0;
	for(; jj<n-3; jj+=4)
		{
		ptr_pA = pA + jj*bs;
		ii = 0;
		if(m0>0)
			{
			for(; ii<m0; ii++)
				{
				// unroll 0
				A[jj+0+lda*ii] = ptr_pA[0+bs*0];
				// unroll 1
				A[jj+1+lda*ii] = ptr_pA[0+bs*1];
				// unroll 2
				A[jj+2+lda*ii] = ptr_pA[0+bs*2];
				// unroll 3
				A[jj+3+lda*ii] = ptr_pA[0+bs*3];
				ptr_pA++;
				}
			ptr_pA += (sda-1)*bs;
			}
		// TODO update A !!!!!
		for(; ii<m-bs+1; ii+=bs)
			{
			// unroll 0
			A[jj+0+lda*(ii+0)] = ptr_pA[0+bs*0];
			A[jj+0+lda*(ii+1)] = ptr_pA[1+bs*0];
			A[jj+0+lda*(ii+2)] = ptr_pA[2+bs*0];
			A[jj+0+lda*(ii+3)] = ptr_pA[3+bs*0];
			A[jj+0+lda*(ii+4)] = ptr_pA[4+bs*0];
			A[jj+0+lda*(ii+5)] = ptr_pA[5+bs*0];
			A[jj+0+lda*(ii+6)] = ptr_pA[6+bs*0];
			A[jj+0+lda*(ii+7)] = ptr_pA[7+bs*0];
			// unroll 1
			A[jj+1+lda*(ii+0)] = ptr_pA[0+bs*1];
			A[jj+1+lda*(ii+1)] = ptr_pA[1+bs*1];
			A[jj+1+lda*(ii+2)] = ptr_pA[2+bs*1];
			A[jj+1+lda*(ii+3)] = ptr_pA[3+bs*1];
			A[jj+1+lda*(ii+4)] = ptr_pA[4+bs*1];
			A[jj+1+lda*(ii+5)] = ptr_pA[5+bs*1];
			A[jj+1+lda*(ii+6)] = ptr_pA[6+bs*1];
			A[jj+1+lda*(ii+7)] = ptr_pA[7+bs*1];
			// unroll 2
			A[jj+2+lda*(ii+0)] = ptr_pA[0+bs*2];
			A[jj+2+lda*(ii+1)] = ptr_pA[1+bs*2];
			A[jj+2+lda*(ii+2)] = ptr_pA[2+bs*2];
			A[jj+2+lda*(ii+3)] = ptr_pA[3+bs*2];
			A[jj+2+lda*(ii+4)] = ptr_pA[4+bs*2];
			A[jj+2+lda*(ii+5)] = ptr_pA[5+bs*2];
			A[jj+2+lda*(ii+6)] = ptr_pA[6+bs*2];
			A[jj+2+lda*(ii+7)] = ptr_pA[7+bs*2];
			// unroll 3
			A[jj+3+lda*(ii+0)] = ptr_pA[0+bs*3];
			A[jj+3+lda*(ii+1)] = ptr_pA[1+bs*3];
			A[jj+3+lda*(ii+2)] = ptr_pA[2+bs*3];
			A[jj+3+lda*(ii+3)] = ptr_pA[3+bs*3];
			A[jj+3+lda*(ii+4)] = ptr_pA[4+bs*3];
			A[jj+3+lda*(ii+5)] = ptr_pA[5+bs*3];
			A[jj+3+lda*(ii+6)] = ptr_pA[6+bs*3];
			A[jj+3+lda*(ii+7)] = ptr_pA[7+bs*3];
			ptr_pA += sda*bs;
			}
		for(; ii<m; ii++)
			{
			// unroll 0
			A[jj+0+lda*ii] = ptr_pA[0+bs*0];
			// unroll 1
			A[jj+1+lda*ii] = ptr_pA[0+bs*1];
			// unroll 2
			A[jj+2+lda*ii] = ptr_pA[0+bs*2];
			// unroll 3
			A[jj+3+lda*ii] = ptr_pA[0+bs*3];
			ptr_pA++;
			}
		}
	for(; jj<n; jj++)
		{
		ptr_pA = pA + jj*bs;
		ii = 0;
		if(m0>0)
			{
			for(; ii<m0; ii++)
				{
				A[jj+lda*ii] = ptr_pA[0];
				ptr_pA++;
				}
			ptr_pA += (sda-1)*bs;
			}
		for(; ii<m-bs+1; ii+=bs)
			{
			i=0;
			// TODO update A !!!!!
			// TODO unroll !!!!!!
			for(; i<bs; i++)
				{
				A[jj+lda*(i+ii)] = ptr_pA[0];
				ptr_pA++;
				}
			ptr_pA += (sda-1)*bs;
			}
		for(; ii<m; ii++)
			{
			A[jj+lda*ii] = ptr_pA[0];
			ptr_pA++;
			}
		}
	return;
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



#if 0
// cast a matrix into a matrix structure
void s_cast_mat2strmat(float *A, struct blasfeo_smat *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	sA->pA = A;
	return;
	}



// cast a matrix into the diagonal of a matrix structure
void s_cast_diag_mat2strmat(float *dA, struct blasfeo_smat *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	sA->dA = dA;
	return;
	}



// cast a vector into a vector structure
void s_cast_vec2vecmat(float *a, struct blasfeo_svec *sa)
	{
	sa->pa = a;
	return;
	}
#endif



// insert element into strmat
void blasfeo_sgein1(float a, struct blasfeo_smat *sA, int ai, int aj)
	{

	if (ai == aj)
		{
		// invalidate stored inverse diagonal
		sA->use_dA = 0;
		}

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	pA[0] = a;
	return;
	}



// extract element from strmat
float blasfeo_sgeex1(struct blasfeo_smat *sA, int ai, int aj)
	{
	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	return pA[0];
	}



// insert element into strvec
void blasfeo_svecin1(float a, struct blasfeo_svec *sx, int xi)
	{
	const int bs = 8;
	float *x = sx->pa + xi;
	x[0] = a;
	return;
	}



// extract element from strvec
float blasfeo_svecex1(struct blasfeo_svec *sx, int xi)
	{
	const int bs = 8;
	float *x = sx->pa + xi;
	return x[0];
	}



// set all elements of a strmat to a value
void blasfeo_sgese(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
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
void blasfeo_svecse(int m, float alpha, struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<m; ii++)
		x[ii] = alpha;
	return;
	}



// extract diagonal to vector
void blasfeo_sdiaex(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	sdiaex_lib(kmax, alpha, ai%bs, pA, sda, x);
	return;
	}



// insert a vector into diagonal
void blasfeo_sdiain(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	sdiain_lib(kmax, alpha, x, ai%bs, pA, sda);
	return;
	}



// swap two rows of a matrix struct
void blasfeo_srowsw(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	float *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	srowsw_lib(kmax, pA, pC);
	return;
	}



// permute the rows of a matrix struct
void blasfeo_srowpe(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			blasfeo_srowsw(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}


// inverse permute the rows of a matrix struct
void blasfeo_srowpei(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			blasfeo_srowsw(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}


// extract a row int a vector
void blasfeo_srowex(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	srowex_lib(kmax, alpha, pA, x);
	return;
	}



// insert a vector into a row
void blasfeo_srowin(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	srowin_lib(kmax, alpha, x, pA);
	return;
	}



// add a vector to a row
void blasfeo_srowad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	srowad_lib(kmax, alpha, x, pA);
	return;
	}



// extract vector from column
void blasfeo_scolex(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	scolex_lib(kmax, ai%bs, pA, sda, x);
	return;
	}



// insert as vector as a column
void blasfeo_scolin(int kmax, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	scolin_lib(kmax, x, ai%bs, pA, sda);
	return;
	}



// add scaled vector to column
void blasfeo_scolad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	float *x = sx->pa + xi;
	scolad_lib(kmax, alpha, x, ai%bs, pA, sda);
	return;
	}



// scale a column
void blasfeo_scolsc(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 8;

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



// swap two cols of a matrix struct
void blasfeo_scolsw(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	const int bs = 8;
#if defined(BLASFEO_REF_API)
	if(ai%bs!=ci%bs)
		{
		blasfeo_ref_scolsw(kmax, sA, ai, aj, sC, ci, cj);
		return;
		}
#endif
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	float *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	scolsw_lib(kmax, ai%bs, pA, sda, ci%bs, pC, sdc);
	return;
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



// scale a generic strmat
void blasfeo_sgesc(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	// early return
	if(m==0 | n==0)
		return;

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_sgesc : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_sgesc : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_sgesc : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_sgesc : aj<0 : %d<0 *****\n", aj);
	// inside matrix
	// A: m x n
	if(ai+m > sA->m) printf("\n***** blasfeo_sgesc : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_sgesc : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
#endif

	const int bs = 8;

	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	int offsetA = ai%bs;

	int ii, mna;

	// alignment
	if(offsetA>0)
		{
		mna = bs-offsetA;
		mna = m<mna ? m : mna;
		kernel_sgesc_8_0_gen_u_lib8(n, &alpha, &pA[offsetA], mna);
		m -= mna;
		pA += 8*sda;
		}

	ii = 0;
	// main loop
	for( ; ii<m-7; ii+=8)
		{
		kernel_sgesc_8_0_lib8(n, &alpha, &pA[0]);
		pA += 8*sda;
		}
	// clean up
	if(ii<m)
		{
		kernel_sgesc_8_0_gen_lib8(n, &alpha, &pA[0], m-ii);
		}

	return;

	}



// copy a generic strmat into a generic strmat
void blasfeo_sgecp(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	// early return
	if(m==0 | n==0)
		return;

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_sgecp : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_sgecp : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_sgecp : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_sgecp : aj<0 : %d<0 *****\n", aj);
	if(bi<0) printf("\n****** blasfeo_sgecp : bi<0 : %d<0 *****\n", bi);
	if(bj<0) printf("\n****** blasfeo_sgecp : bj<0 : %d<0 *****\n", bj);
	// inside matrix
	// A: m x n
	if(ai+m > sA->m) printf("\n***** blasfeo_sgecp : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_sgecp : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// B: m x n
	if(bi+m > sB->m) printf("\n***** blasfeo_sgecp : bi+m > row(B) : %d+%d > %d *****\n", bi, m, sB->m);
	if(bj+n > sB->n) printf("\n***** blasfeo_sgecp : bj+n > col(B) : %d+%d > %d *****\n", bj, n, sB->n);
#endif

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offsetA = ai%bs;
	int offsetB = bi%bs;

	int ii, mna;

	if(offsetB>0)
		{
		if(offsetB>offsetA)
			{
			mna = bs-offsetB;
			mna = m<mna ? m : mna;
			kernel_sgecp_8_0_gen_u_lib8(n, &pA[offsetA], &pB[offsetB], mna);
			m -= mna;
			pB += 8*sdb;
			}
		else
			{
			if(offsetA==0)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_0_gen_lib8(n, &pA[0], &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==1)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_1_gen_lib8(n, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==2)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_2_gen_lib8(n, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==3)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_3_gen_lib8(n, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==4)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_4_gen_lib8(n, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==5)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_5_gen_lib8(n, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==6)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_6_gen_lib8(n, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==7)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecp_8_7_gen_lib8(n, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			}
		}

	// same alignment
	if(offsetA==offsetB)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_0_lib8(n, pA, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_0_gen_lib8(n, pA, pB, m-ii);
			}
		return;
		}
	// XXX different alignment: search tree ???
	// skip one element of A
	else if(offsetA==(offsetB+1)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_1_lib8(n, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_1_gen_lib8(n, pA, sda, pB, m-ii);
			}
		}
	// skip two elements of A
	else if(offsetA==(offsetB+2)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_2_lib8(n, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_2_gen_lib8(n, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip three elements of A
	else if(offsetA==(offsetB+3)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_3_lib8(n, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_3_gen_lib8(n, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip four elements of A
	else if(offsetA==(offsetB+4)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_4_lib8(n, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_4_gen_lib8(n, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip five elements of A
	else if(offsetA==(offsetB+5)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_5_lib8(n, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_5_gen_lib8(n, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip six elements of A
	else if(offsetA==(offsetB+6)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_6_lib8(n, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_6_gen_lib8(n, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip seven elements of A
	else //if(offsetA==(offsetB+7)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecp_8_7_lib8(n, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecp_8_7_gen_lib8(n, pA, sda, pB, m-ii);
			}
		return;
		}

	return;

	}



// copy and scale a generic strmat into a generic strmat
void blasfeo_sgecpsc(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	// early return
	if(m==0 | n==0)
		return;

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_sgecpsc : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_sgecpsc : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_sgecpsc : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_sgecpsc : aj<0 : %d<0 *****\n", aj);
	if(bi<0) printf("\n****** blasfeo_sgecpsc : bi<0 : %d<0 *****\n", bi);
	if(bj<0) printf("\n****** blasfeo_sgecpsc : bj<0 : %d<0 *****\n", bj);
	// inside matrix
	// A: m x n
	if(ai+m > sA->m) printf("\n***** blasfeo_sgecpsc : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_sgecpsc : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// B: m x n
	if(bi+m > sB->m) printf("\n***** blasfeo_sgecpsc : bi+m > row(B) : %d+%d > %d *****\n", bi, m, sB->m);
	if(bj+n > sB->n) printf("\n***** blasfeo_sgecpsc : bj+n > col(B) : %d+%d > %d *****\n", bj, n, sB->n);
#endif

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offsetA = ai%bs;
	int offsetB = bi%bs;

	int ii, mna;

	if(offsetB>0)
		{
		if(offsetB>offsetA)
			{
			mna = bs-offsetB;
			mna = m<mna ? m : mna;
			kernel_sgecpsc_8_0_gen_u_lib8(n, &alpha, &pA[offsetA], &pB[offsetB], mna);
			m -= mna;
			pB += 8*sdb;
			}
		else
			{
			if(offsetA==0)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_0_gen_lib8(n, &alpha, &pA[0], &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==1)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_1_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==2)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_2_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==3)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_3_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==4)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_4_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==5)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_5_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==6)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_6_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==7)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgecpsc_8_7_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			}
		}

	// same alignment
	if(offsetA==offsetB)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_0_lib8(n, &alpha, pA, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_0_gen_lib8(n, &alpha, pA, pB, m-ii);
			}
		return;
		}
	// XXX different alignment: search tree ???
	// skip one element of A
	else if(offsetA==(offsetB+1)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_1_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_1_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		}
	// skip two elements of A
	else if(offsetA==(offsetB+2)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_2_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_2_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip three elements of A
	else if(offsetA==(offsetB+3)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_3_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_3_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip four elements of A
	else if(offsetA==(offsetB+4)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_4_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_4_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip five elements of A
	else if(offsetA==(offsetB+5)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_5_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_5_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip six elements of A
	else if(offsetA==(offsetB+6)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_6_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_6_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip seven elements of A
	else //if(offsetA==(offsetB+7)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgecpsc_8_7_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgecpsc_8_7_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}

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



// copy a strvec into a strvec
void blasfeo_sveccp(int m, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float *pa = sa->pa + ai;
	float *pc = sc->pa + ci;
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



// copy a lower triangular strmat into a lower triangular strmat
void blasfeo_strcp_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strcp_l(m, sA, ai, aj, sC, ci, cj);
	return;
#else
	printf("\nblasfeo_strcp_l: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// scale and add a generic strmat into a generic strmat
void blasfeo_sgead(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	// early return
	if(m==0 | n==0)
		return;

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_sgead : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_sgead : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_sgead : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_sgead : aj<0 : %d<0 *****\n", aj);
	if(bi<0) printf("\n****** blasfeo_sgead : bi<0 : %d<0 *****\n", bi);
	if(bj<0) printf("\n****** blasfeo_sgead : bj<0 : %d<0 *****\n", bj);
	// inside matrix
	// A: m x n
	if(ai+m > sA->m) printf("\n***** blasfeo_sgead : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_sgead : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// B: m x n
	if(bi+m > sB->m) printf("\n***** blasfeo_sgead : bi+m > row(B) : %d+%d > %d *****\n", bi, m, sB->m);
	if(bj+n > sB->n) printf("\n***** blasfeo_sgead : bj+n > col(B) : %d+%d > %d *****\n", bj, n, sB->n);
#endif

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offsetA = ai%bs;
	int offsetB = bi%bs;

	int ii, mna;

#if 1
	if(offsetB>0)
		{
		if(offsetB>offsetA)
			{
			mna = bs-offsetB;
			mna = m<mna ? m : mna;
			kernel_sgead_8_0_gen_lib8(n, &alpha, &pA[offsetA], &pB[offsetB], mna);
			m -= mna;
			//pA += 8*sda;
			pB += 8*sdb;
			}
		else
			{
			if(offsetA==0)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_0_gen_lib8(n, &alpha, &pA[0], &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==1)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_1_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==2)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_2_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==3)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_3_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==4)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_4_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==5)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_5_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==6)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_6_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			else if(offsetA==7)
				{
				mna = bs-offsetB;
				mna = m<mna ? m : mna;
				kernel_sgead_8_7_gen_lib8(n, &alpha, &pA[0], sda, &pB[offsetB], mna);
				m -= mna;
				pA += 8*sda;
				pB += 8*sdb;
				}
			}
		}
#endif

	// same alignment
	if(offsetA==offsetB)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_0_lib8(n, &alpha, pA, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_0_gen_lib8(n, &alpha, pA, pB, m-ii);
			}
		return;
		}
	// XXX different alignment: search tree ???
	// skip one element of A
	else if(offsetA==(offsetB+1)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_1_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_1_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		}
	// skip two elements of A
	else if(offsetA==(offsetB+2)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_2_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_2_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip three elements of A
	else if(offsetA==(offsetB+3)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_3_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_3_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip four elements of A
	else if(offsetA==(offsetB+4)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_4_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_4_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip five elements of A
	else if(offsetA==(offsetB+5)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_5_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_5_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip six elements of A
	else if(offsetA==(offsetB+6)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_6_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_6_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}
	// skip seven elements of A
	else //if(offsetA==(offsetB+7)%bs)
		{
		ii = 0;
		for( ; ii<m-7; ii+=8)
			{
			kernel_sgead_8_7_lib8(n, &alpha, pA, sda, pB);
			pA += 8*sda;
			pB += 8*sdb;
			}
		if(ii<m)
			{
			kernel_sgead_8_7_gen_lib8(n, &alpha, pA, sda, pB, m-ii);
			}
		return;
		}

	return;

	}



// copy and transpose a generic strmat into a generic strmat
void blasfeo_sgetr(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	// early return
	if(m==0 | n==0)
		return;

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

#if defined(DIM_CHECK)
	// non-negative size
	if(m<0) printf("\n****** blasfeo_sgetr : m<0 : %d<0 *****\n", m);
	if(n<0) printf("\n****** blasfeo_sgetr : n<0 : %d<0 *****\n", n);
	// non-negative offset
	if(ai<0) printf("\n****** blasfeo_sgetr : ai<0 : %d<0 *****\n", ai);
	if(aj<0) printf("\n****** blasfeo_sgetr : aj<0 : %d<0 *****\n", aj);
	if(bi<0) printf("\n****** blasfeo_sgetr : bi<0 : %d<0 *****\n", bi);
	if(bj<0) printf("\n****** blasfeo_sgetr : bj<0 : %d<0 *****\n", bj);
	// inside matrix
	// A: m x n
	if(ai+m > sA->m) printf("\n***** blasfeo_sgetr : ai+m > row(A) : %d+%d > %d *****\n", ai, m, sA->m);
	if(aj+n > sA->n) printf("\n***** blasfeo_sgetr : aj+n > col(A) : %d+%d > %d *****\n", aj, n, sA->n);
	// B: n x m
	if(bi+n > sB->m) printf("\n***** blasfeo_sgetr : bi+n > row(B) : %d+%d > %d *****\n", bi, n, sB->m);
	if(bj+m > sB->n) printf("\n***** blasfeo_sgetr : bj+m > col(B) : %d+%d > %d *****\n", bj, m, sB->n);
#endif

	const int bs = 8;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offsetA = ai%bs;
	int offsetB = bi%bs;

	int ii, nna;

	if(offsetA==0)
		{
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_0_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for(ii=0; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_0_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_0_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}
	// TODO log serach for offsetA>0 ???
	else if(offsetA==1)
		{
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_1_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for(ii=0; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_1_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_1_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}
	else if(offsetA==2)
		{
		ii = 0;
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_2_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for( ; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_2_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_2_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}
	else if(offsetA==3)
		{
		ii = 0;
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_3_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for( ; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_3_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_3_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}
	else if(offsetA==4)
		{
		ii = 0;
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_4_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for( ; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_4_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_4_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}
	else if(offsetA==5)
		{
		ii = 0;
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_5_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for( ; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_5_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_5_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}
	else if(offsetA==6)
		{
		ii = 0;
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_6_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for( ; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_6_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_6_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}
	else if(offsetA==7)
		{
		ii = 0;
		if(offsetB>0)
			{
			nna = bs-offsetB;
			nna = n<nna ? n : nna;
			kernel_sgetr_8_7_gen_lib8(m, &pA[0], sda, &pB[offsetB], nna);
			n -= nna;
			pA += nna*bs;
			pB += 8*sdb;
			}
		for( ; ii<n-7; ii+=8)
			{
			kernel_sgetr_8_7_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb]);
			}
		if(ii<n)
			{
			kernel_sgetr_8_7_gen_lib8(m, &pA[ii*bs], sda, &pB[ii*sdb], n-ii);
			}
		}

	return;

	}



// copy and transpose a lower triangular strmat into an upper triangular strmat
void blasfeo_strtr_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strtr_l(m, sA, ai, aj, sC, ci, cj);
	return;
#else
	printf("\nblasfeo_strtr_l: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// copy and transpose an upper triangular strmat into a lower triangular strmat
void blasfeo_strtr_u(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;
#if defined(BLASFEO_REF_API)
	blasfeo_ref_strtr_u(m, sA, ai, aj, sC, ci, cj);
	return;
#else
	printf("\nblasfeo_strtr_u: feature not implemented yet\n");
	exit(1);
#endif
	return;
	}



// insert a strvec to diagonal of strmat, sparse formulation
void blasfeo_sdiain_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 8;
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



// extract the diagonal of a strmat to a strvec, sparse formulation
void blasfeo_sdiaex_sp(int kmax, float alpha, int *idx, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_svec *sx, int xi)
	{
	const int bs = 8;
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



// add scaled strvec to diagonal of strmat, sparse formulation
void blasfeo_sdiaad_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 8;
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

	const int bs = 8;
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

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 8;
	float *x = sx->pa + xi;
	int sdd = sD->cn;
	float *pD = sD->pA + di/bs*bs*sdd + di%bs + dj*bs;
	srowad_libsp(kmax, idx, alpha, x, pD);
	return;
	}



// add strvec to strvec, sparse formulation
void blasfeo_svecad_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_svec *sy, int yi)
	{
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	svecad_libsp(kmax, idx, alpha, x, y);
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
	float *xm = sxm->pa + xim;
	float *x  = sx->pa + xi;
	float *xp = sxp->pa + xip;
	float *z  = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		{
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
		}
	return;
	}



// clip strvec between two strvec, with mask
void blasfeo_sveccl_mask(int m, struct blasfeo_svec *sxm, int xim, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sxp, int xip, struct blasfeo_svec *sz, int zi, struct blasfeo_svec *sm, int mi)
	{
	float *xm = sxm->pa + xim;
	float *x  = sx->pa + xi;
	float *xp = sxp->pa + xip;
	float *z  = sz->pa + zi;
	float *mask  = sm->pa + mi;
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
void blasfeo_svecze(int m, struct blasfeo_svec *sm, int mi, struct blasfeo_svec *sv, int vi, struct blasfeo_svec *se, int ei)
	{
	float *mask = sm->pa + mi;
	float *v = sv->pa + vi;
	float *e = se->pa + ei;
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
	norm = sqrtf(norm);
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




