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
void blasfeo_svecad(int m, float *alphap, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float alpha = alphap[0];
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



// transpose general matrix; m and n are referred to the original matrix
void sgetr_lib(int m, int n, float alpha, int offsetA, float *pA, int sda, int offsetC, float *pC, int sdc)
	{

/*

m = 5
n = 3
offsetA = 1
offsetC = 2

A =
 x x x
 -
 x x x
 x x x
 x x x
 x x x

C =
 x x x x x
 x x x x x
 -
 x x x x x

*/

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna = (bs-offsetA%bs)%bs;
	mna = m<mna ? m : mna;
	int nna = (bs-offsetC%bs)%bs;
	nna = n<nna ? n : nna;

	int ii;

	ii = 0;

	if(mna>0)
		{
		if(mna==1)
			kernel_sgetr_1_lib4(0, n, nna, alpha, pA, pC, sdc);
		else if(mna==2)
			kernel_sgetr_2_lib4(0, n, nna, alpha, pA, pC, sdc);
		else //if(mna==3)
			kernel_sgetr_3_lib4(0, n, nna, alpha, pA, pC, sdc);
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
	for( ; ii<m-3; ii+=4)
//	for( ; ii<m; ii+=4)
		{
		kernel_sgetr_4_lib4(0, n, nna, alpha, pA, pC, sdc);
		pA += bs*sda;
		pC += bs*bs;
		}

	// clean-up at the end using smaller kernels
	if(ii==m)
		return;

	if(m-ii==1)
		kernel_sgetr_1_lib4(0, n, nna, alpha, pA, pC, sdc);
	else if(m-ii==2)
		kernel_sgetr_2_lib4(0, n, nna, alpha, pA, pC, sdc);
	else if(m-ii==3)
		kernel_sgetr_3_lib4(0, n, nna, alpha, pA, pC, sdc);

	return;

	}



// transpose lower triangular matrix
void strtr_l_lib(int m, float alpha, int offsetA, float *pA, int sda, int offsetC, float *pC, int sdc)
	{

/*

A =
 x
 x x
 x x x
 x x x x

 x x x x x
 x x x x x x
 x x x x x x x
 x x x x x x x x

C =
 x x x x x x x x

   x x x x x x x
     x x x x x x
	   x x x x x
	     x x x x

	       x x x
	         x x
	           x

*/

	int n = m;

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna = (bs-offsetA%bs)%bs;
	mna = m<mna ? m : mna;
	int nna = (bs-offsetC%bs)%bs;
	nna = n<nna ? n : nna;

	int ii;

	ii = 0;

	if(mna>0)
		{
		if(mna==1)
			{
			pC[0] = alpha * pA[0];
			}
		else if(mna==2)
			{
			if(nna==1)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[0+bs*1] = alpha * pA[1+bs*0];
				pC[1+bs*(0+sdc)] = alpha * pA[1+bs*1];
				}
			else
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[0+bs*1] = alpha * pA[1+bs*0];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				}
			}
		else //if(mna==3)
			{
			if(nna==1)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[0+bs*1] = alpha * pA[1+bs*0];
				pC[0+bs*2] = alpha * pA[2+bs*0];
				pC[1+bs*(0+sdc)] = alpha * pA[1+bs*1];
				pC[1+bs*(1+sdc)] = alpha * pA[2+bs*1];
				pC[2+bs*(1+sdc)] = alpha * pA[2+bs*2];
				}
			else if(nna==2)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[0+bs*1] = alpha * pA[1+bs*0];
				pC[0+bs*2] = alpha * pA[2+bs*0];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pC[1+bs*2] = alpha * pA[2+bs*1];
				pC[2+bs*(1+sdc)] = alpha * pA[2+bs*2];
				}
			else
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[0+bs*1] = alpha * pA[1+bs*0];
				pC[0+bs*2] = alpha * pA[2+bs*0];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pC[1+bs*2] = alpha * pA[2+bs*1];
				pC[2+bs*2] = alpha * pA[2+bs*2];
				}
			}
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
	for( ; ii<m-3; ii+=4)
		{
		kernel_sgetr_4_lib4(1, ii, nna, alpha, pA, pC, sdc);
		pA += bs*sda;
		pC += bs*bs;
		}

	// clean-up at the end using smaller kernels
	if(ii==m)
		return;

	if(m-ii==1)
		kernel_sgetr_1_lib4(1, ii, nna, alpha, pA, pC, sdc);
	else if(m-ii==2)
		kernel_sgetr_2_lib4(1, ii, nna, alpha, pA, pC, sdc);
	else if(m-ii==3)
		kernel_sgetr_3_lib4(1, ii, nna, alpha, pA, pC, sdc);

	return;

	}



// transpose an aligned upper triangular matrix into an aligned lower triangular matrix
void strtr_u_lib(int m, float alpha, int offsetA, float *pA, int sda, int offsetC, float *pC, int sdc)
	{

/*

A =
 x x x x x x x x
   x x x x x x x

     x x x x x x
       x x x x x
         x x x x
           x x x
             x x
               x

C =
 x

 x x
 x x x
 x x x x
 x x x x x
 x x x x x x
 x x x x x x x
 x x x x x x x x

*/

	int n = m;

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna = (bs-offsetA%bs)%bs;
	mna = m<mna ? m : mna;
	int nna = (bs-offsetC%bs)%bs;
	nna = n<nna ? n : nna;
	int tna = nna;

	int ii;

	ii = 0;

	if(mna>0)
		{
		if(mna==1)
			{
			kernel_sgetr_1_lib4(0, n, nna, alpha, pA, pC, sdc);
			if(nna!=1)
				{
//				pC[0+bs*0] = alpha * pA[0+bs*0];
				pA += 1*bs;
				pC += 1;
				tna = (bs-(offsetC+1)%bs)%bs;
				}
			else //if(nna==1)
				{
//				pC[0+bs*0] = alpha * pA[0+bs*0];
				pA += 1*bs;
				pC += 1 + (sdc-1)*bs;
				tna = 0; //(bs-(offsetC+1)%bs)%bs;
				}
//			kernel_sgetr_1_lib4(0, n-1, tna, alpha, pA, pC, sdc);
			}
		else if(mna==2)
			{
			if(nna==0 || nna==3)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[1+bs*0] = alpha * pA[0+bs*1];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pA += 2*bs;
				pC += 2;
				tna = (bs-(offsetC+2)%bs)%bs;
				kernel_sgetr_2_lib4(0, n-2, tna, alpha, pA, pC, sdc);
				}
			else if(nna==1)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pA += 1*bs;
				pC += 1 + (sdc-1)*bs;
//				pC[0+bs*0] = alpha * pA[0+bs*0];
//				pC[0+bs*1] = alpha * pA[1+bs*0];
				kernel_sgetr_2_lib4(0, n-1, 0, alpha, pA, pC, sdc);
				pA += 1*bs;
				pC += 1;
				tna = 3; //(bs-(offsetC+2)%bs)%bs;
//				kernel_sgetr_2_lib4(0, n-2, tna, alpha, pA, pC, sdc);
				}
			else if(nna==2)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[1+bs*0] = alpha * pA[0+bs*1];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pA += 2*bs;
				pC += 2 + (sdc-1)*bs;
				tna = 0; //(bs-(offsetC+2)%bs)%bs;
				kernel_sgetr_2_lib4(0, n-2, tna, alpha, pA, pC, sdc);
				}
			}
		else //if(mna==3)
			{
			if(nna==0)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[1+bs*0] = alpha * pA[0+bs*1];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pC[2+bs*0] = alpha * pA[0+bs*2];
				pC[2+bs*1] = alpha * pA[1+bs*2];
				pC[2+bs*2] = alpha * pA[2+bs*2];
				pA += 3*bs;
				pC += 3;
				tna = 1;
				kernel_sgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
				}
			else if(nna==1)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pA += bs;
				pC += 1 + (sdc-1)*bs;
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[0+bs*1] = alpha * pA[1+bs*0];
				pC[1+bs*0] = alpha * pA[0+bs*1];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pC[1+bs*2] = alpha * pA[2+bs*1];
				pA += 2*bs;
				pC += 2;
				tna = 2;
				kernel_sgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
				}
			else if(nna==2)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[1+bs*0] = alpha * pA[0+bs*1];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pA += 2*bs;
				pC += 2 + (sdc-1)*bs;
//				pC[0+bs*0] = alpha * pA[0+bs*0];
//				pC[0+bs*1] = alpha * pA[1+bs*0];
//				pC[0+bs*2] = alpha * pA[2+bs*0];
				kernel_sgetr_3_lib4(0, n-2, 0, alpha, pA, pC, sdc);
				pA += 1*bs;
				pC += 1;
				tna = 3;
//				kernel_sgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
				}
			else //if(nna==3)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[1+bs*0] = alpha * pA[0+bs*1];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pC[2+bs*0] = alpha * pA[0+bs*2];
				pC[2+bs*1] = alpha * pA[1+bs*2];
				pC[2+bs*2] = alpha * pA[2+bs*2];
				pA += 3*bs;
				pC += 3 + (sdc-1)*bs;
				tna = 0;
				kernel_sgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
				}
			}
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
	for( ; ii<m-3; ii+=4)
		{
		if(tna==0)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pC[2+bs*0] = alpha * pA[0+bs*2];
			pC[2+bs*1] = alpha * pA[1+bs*2];
			pC[2+bs*2] = alpha * pA[2+bs*2];
			pC[3+bs*0] = alpha * pA[0+bs*3];
			pC[3+bs*1] = alpha * pA[1+bs*3];
			pC[3+bs*2] = alpha * pA[2+bs*3];
			pC[3+bs*3] = alpha * pA[3+bs*3];
			pA += 4*bs;
			pC += sdc*bs;
			kernel_sgetr_4_lib4(0, n-ii-4, 0, alpha, pA, pC, sdc);
			}
		else if(tna==1)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pA += bs;
			pC += 1 + (sdc-1)*bs;
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[0+bs*1] = alpha * pA[1+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pC[1+bs*2] = alpha * pA[2+bs*1];
			pC[2+bs*0] = alpha * pA[0+bs*2];
			pC[2+bs*1] = alpha * pA[1+bs*2];
			pC[2+bs*2] = alpha * pA[2+bs*2];
			pC[2+bs*3] = alpha * pA[3+bs*2];
			pA += 3*bs;
			pC += 3;
			kernel_sgetr_4_lib4(0, n-ii-4, 1, alpha, pA, pC, sdc);
			}
		else if(tna==2)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pA += 2*bs;
			pC += 2 + (sdc-1)*bs;
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[0+bs*1] = alpha * pA[1+bs*0];
			pC[0+bs*2] = alpha * pA[2+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pC[1+bs*2] = alpha * pA[2+bs*1];
			pC[1+bs*3] = alpha * pA[3+bs*1];
			pA += 2*bs;
			pC += 2;
			kernel_sgetr_4_lib4(0, n-ii-4, 2, alpha, pA, pC, sdc);
			}
		else //if(tna==3)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pC[2+bs*0] = alpha * pA[0+bs*2];
			pC[2+bs*1] = alpha * pA[1+bs*2];
			pC[2+bs*2] = alpha * pA[2+bs*2];
			pA += 3*bs;
			pC += 3 + (sdc-1)*bs;
			kernel_sgetr_4_lib4(0, n-ii-3, 0, alpha, pA, pC, sdc);
//			pC[0+bs*0] = alpha * pA[0+bs*0];
//			pC[0+bs*1] = alpha * pA[1+bs*0];
//			pC[0+bs*2] = alpha * pA[2+bs*0];
//			pC[0+bs*3] = alpha * pA[3+bs*0];
			pA += bs;
			pC += 1;
//			kernel_sgetr_4_lib4(0, n-ii-4, tna, alpha, pA, pC, sdc);
			}
		pA += bs*sda;
		pC += bs*bs;
		}

	// clean-up at the end
	if(ii==m)
		return;

	if(m-ii==1)
		{
		pC[0+bs*0] = alpha * pA[0+bs*0];
		}
	else if(m-ii==2)
		{
		if(tna!=1)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			}
		else //if(tna==1)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pA += bs;
			pC += 1 + (sdc-1)*bs;
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[0+bs*1] = alpha * pA[1+bs*0];
			}
		}
	else if(m-ii==3)
		{
		if(tna==0 || tna==3)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pC[2+bs*0] = alpha * pA[0+bs*2];
			pC[2+bs*1] = alpha * pA[1+bs*2];
			pC[2+bs*2] = alpha * pA[2+bs*2];
			}
		else if(tna==1)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pA += bs;
			pC += 1 + (sdc-1)*bs;
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[0+bs*1] = alpha * pA[1+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*0];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pC[1+bs*2] = alpha * pA[2+bs*1];
			}
		else //if(tna==2)
			{
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[1+bs*0] = alpha * pA[0+bs*1];
			pC[1+bs*1] = alpha * pA[1+bs*1];
			pA += 2*bs;
			pC += 2 + (sdc-1)*bs;
			pC[0+bs*0] = alpha * pA[0+bs*0];
			pC[0+bs*1] = alpha * pA[1+bs*0];
			pC[0+bs*2] = alpha * pA[2+bs*0];
			}
		}

	return;

	}



// regularize diagonal
void sdiareg_lib(int kmax, float reg, int offset, float *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] += reg;
		pD[jj*sdd+(jj+1)*bs+1] += reg;
		pD[jj*sdd+(jj+2)*bs+2] += reg;
		pD[jj*sdd+(jj+3)*bs+3] += reg;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] += reg;
		}

	}



// insert vector to diagonal
void sdiain_lib(int kmax, float alpha, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] = alpha*x[jj+0];
		pD[jj*sdd+(jj+1)*bs+1] = alpha*x[jj+1];
		pD[jj*sdd+(jj+2)*bs+2] = alpha*x[jj+2];
		pD[jj*sdd+(jj+3)*bs+3] = alpha*x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] = alpha*x[jj+ll];
		}

	}



// insert sqrt of vector to diagonal
void sdiain_sqrt_lib(int kmax, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] = sqrt(x[jj+0]);
		pD[jj*sdd+(jj+1)*bs+1] = sqrt(x[jj+1]);
		pD[jj*sdd+(jj+2)*bs+2] = sqrt(x[jj+2]);
		pD[jj*sdd+(jj+3)*bs+3] = sqrt(x[jj+3]);
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] = sqrt(x[jj+ll]);
		}

	}



// extract diagonal to vector
void sdiaex_lib(int kmax, float alpha, int offset, float *pD, int sdd, float *x)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		x[jj+0] = alpha * pD[jj*sdd+(jj+0)*bs+0];
		x[jj+1] = alpha * pD[jj*sdd+(jj+1)*bs+1];
		x[jj+2] = alpha * pD[jj*sdd+(jj+2)*bs+2];
		x[jj+3] = alpha * pD[jj*sdd+(jj+3)*bs+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[jj+ll] = alpha * pD[jj*sdd+(jj+ll)*bs+ll];
		}

	}



// add scaled vector to diagonal
void sdiaad_lib(int kmax, float alpha, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] += alpha * x[jj+0];
		pD[jj*sdd+(jj+1)*bs+1] += alpha * x[jj+1];
		pD[jj*sdd+(jj+2)*bs+2] += alpha * x[jj+2];
		pD[jj*sdd+(jj+3)*bs+3] += alpha * x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] += alpha * x[jj+ll];
		}

	}



// insert vector to diagonal, sparse formulation
void sdiain_libsp(int kmax, int *idx, float alpha, float *x, float *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] = alpha * x[jj];
		}

	}



// extract diagonal to vector, sparse formulation
void sdiaex_libsp(int kmax, int *idx, float alpha, float *pD, int sdd, float *x)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		x[jj] = alpha * pD[ii/bs*bs*sdd+ii%bs+ii*bs];
		}

	}



// add scaled vector to diagonal, sparse formulation
void sdiaad_libsp(int kmax, int *idx, float alpha, float *x, float *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] += alpha * x[jj];
		}

	}



// add scaled vector to another vector and insert to diagonal, sparse formulation
void sdiaadin_libsp(int kmax, int *idx, float alpha, float *x, float *y, float *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] = y[jj] + alpha * x[jj];
		}

	}



// insert vector to row
void srowin_lib(int kmax, float alpha, float *x, float *pD)
	{

	const int bs = 4;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[(jj+0)*bs] = alpha*x[jj+0];
		pD[(jj+1)*bs] = alpha*x[jj+1];
		pD[(jj+2)*bs] = alpha*x[jj+2];
		pD[(jj+3)*bs] = alpha*x[jj+3];
		}
	for(; jj<kmax; jj++)
		{
		pD[(jj)*bs] = alpha*x[jj];
		}

	}



// extract row to vector
void srowex_lib(int kmax, float alpha, float *pD, float *x)
	{

	const int bs = 4;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		x[jj+0] = alpha*pD[(jj+0)*bs];
		x[jj+1] = alpha*pD[(jj+1)*bs];
		x[jj+2] = alpha*pD[(jj+2)*bs];
		x[jj+3] = alpha*pD[(jj+3)*bs];
		}
	for(; jj<kmax; jj++)
		{
		x[jj] = alpha*pD[(jj)*bs];
		}

	}



// add scaled vector to row
void srowad_lib(int kmax, float alpha, float *x, float *pD)
	{

	const int bs = 4;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[(jj+0)*bs] += alpha * x[jj+0];
		pD[(jj+1)*bs] += alpha * x[jj+1];
		pD[(jj+2)*bs] += alpha * x[jj+2];
		pD[(jj+3)*bs] += alpha * x[jj+3];
		}
	for(; jj<kmax; jj++)
		{
		pD[(jj)*bs] += alpha * x[jj];
		}

	}



// insert vector to row, sparse formulation
void srowin_libsp(int kmax, float alpha, int *idx, float *x, float *pD)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] = alpha*x[jj];
		}

	}



// add scaled vector to row, sparse formulation
void srowad_libsp(int kmax, int *idx, float alpha, float *x, float *pD)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] += alpha * x[jj];
		}

	}



// add scaled vector to another vector and insert to row, sparse formulation
void srowadin_libsp(int kmax, int *idx, float alpha, float *x, float *y, float *pD)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] = y[jj] + alpha * x[jj];
		}

	}



// swap two rows
void srowsw_lib(int kmax, float *pA, float *pC)
	{

	const int bs = 4;

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

	}



// extract vector from column
void scolex_lib(int kmax, int offset, float *pD, int sdd, float *x)
	{

	const int bs = 4;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		x[jj*sdd+0] = pD[jj+0];
		x[jj*sdd+1] = pD[jj+1];
		x[jj*sdd+2] = pD[jj+2];
		x[jj*sdd+3] = pD[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[jj*sdd+ll] = pD[jj+ll];
		}

	}



// insert vector to column
void scolin_lib(int kmax, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 4;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+0] = x[jj+0];
		pD[jj*sdd+1] = x[jj+1];
		pD[jj*sdd+2] = x[jj+2];
		pD[jj*sdd+3] = x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+ll] = x[jj+ll];
		}

	}



// add scaled vector to column
void scolad_lib(int kmax, float alpha, float *x, int offset, float *pD, int sdd)
	{

	const int bs = 4;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+0] += alpha * x[jj+0];
		pD[jj*sdd+1] += alpha * x[jj+1];
		pD[jj*sdd+2] += alpha * x[jj+2];
		pD[jj*sdd+3] += alpha * x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+ll] += alpha * x[jj+ll];
		}

	}



// insert vector to diagonal, sparse formulation
void scolin_libsp(int kmax, int *idx, float *x, float *pD, int sdd)
	{

	const int bs = 4;

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

	const int bs = 4;

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

	const int bs = 4;

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
		for(; ii<kmax-3; ii+=4)
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
		printf("\nscolsw: feature not implemented yet: offsetA!=offsetC\n\n");
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
	const int bs = 4;
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
	const int bs = 4;
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
	const int bs = 4;
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
	const int bs = 4;
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
	const int bs = 4;
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

	const int bs = 4;
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
		for( ; ii<m-3; ii+=4)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			pB[1+bs*0] = B[1+lda*0];
			pB[2+bs*0] = B[2+lda*0];
			pB[3+bs*0] = B[3+lda*0];
			// col 1
			pB[0+bs*1] = B[0+lda*1];
			pB[1+bs*1] = B[1+lda*1];
			pB[2+bs*1] = B[2+lda*1];
			pB[3+bs*1] = B[3+lda*1];
			// col 2
			pB[0+bs*2] = B[0+lda*2];
			pB[1+bs*2] = B[1+lda*2];
			pB[2+bs*2] = B[2+lda*2];
			pB[3+bs*2] = B[3+lda*2];
			// col 3
			pB[0+bs*3] = B[0+lda*3];
			pB[1+bs*3] = B[1+lda*3];
			pB[2+bs*3] = B[2+lda*3];
			pB[3+bs*3] = B[3+lda*3];
			// update
			B  += 4;
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
		for( ; ii<m-3; ii+=4)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			pB[1+bs*0] = B[1+lda*0];
			pB[2+bs*0] = B[2+lda*0];
			pB[3+bs*0] = B[3+lda*0];
			// update
			B  += 4;
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

	const int bs = 4;
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
	for(; ii<m1-3; ii+=bs)
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
			// unroll 1
			pB[0+1*bs] = B[1+0*lda];
			pB[1+1*bs] = B[1+1*lda];
			pB[2+1*bs] = B[1+2*lda];
			pB[3+1*bs] = B[1+3*lda];
			// unroll 2
			pB[0+2*bs] = B[2+0*lda];
			pB[1+2*bs] = B[2+1*lda];
			pB[2+2*bs] = B[2+2*lda];
			pB[3+2*bs] = B[2+3*lda];
			// unroll 3
			pB[0+3*bs] = B[3+0*lda];
			pB[1+3*bs] = B[3+1*lda];
			pB[2+3*bs] = B[3+2*lda];
			pB[3+3*bs] = B[3+3*lda];
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
	const int bs = 4;
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
		for(; ii<m-bs+1; ii+=bs)
			{
			// unroll 0
			A[0+ii+lda*(jj+0)] = ptr_pA[0+bs*0];
			A[1+ii+lda*(jj+0)] = ptr_pA[1+bs*0];
			A[2+ii+lda*(jj+0)] = ptr_pA[2+bs*0];
			A[3+ii+lda*(jj+0)] = ptr_pA[3+bs*0];
			// unroll 0
			A[0+ii+lda*(jj+1)] = ptr_pA[0+bs*1];
			A[1+ii+lda*(jj+1)] = ptr_pA[1+bs*1];
			A[2+ii+lda*(jj+1)] = ptr_pA[2+bs*1];
			A[3+ii+lda*(jj+1)] = ptr_pA[3+bs*1];
			// unroll 0
			A[0+ii+lda*(jj+2)] = ptr_pA[0+bs*2];
			A[1+ii+lda*(jj+2)] = ptr_pA[1+bs*2];
			A[2+ii+lda*(jj+2)] = ptr_pA[2+bs*2];
			A[3+ii+lda*(jj+2)] = ptr_pA[3+bs*2];
			// unroll 0
			A[0+ii+lda*(jj+3)] = ptr_pA[0+bs*3];
			A[1+ii+lda*(jj+3)] = ptr_pA[1+bs*3];
			A[2+ii+lda*(jj+3)] = ptr_pA[2+bs*3];
			A[3+ii+lda*(jj+3)] = ptr_pA[3+bs*3];
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
			A[0+ii+lda*jj] = ptr_pA[0];
			A[1+ii+lda*jj] = ptr_pA[1];
			A[2+ii+lda*jj] = ptr_pA[2];
			A[3+ii+lda*jj] = ptr_pA[3];
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
	const int bs = 4;
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
		for(; ii<m-bs+1; ii+=bs)
			{
			// unroll 0
			A[jj+0+lda*(ii+0)] = ptr_pA[0+bs*0];
			A[jj+0+lda*(ii+1)] = ptr_pA[1+bs*0];
			A[jj+0+lda*(ii+2)] = ptr_pA[2+bs*0];
			A[jj+0+lda*(ii+3)] = ptr_pA[3+bs*0];
			// unroll 1
			A[jj+1+lda*(ii+0)] = ptr_pA[0+bs*1];
			A[jj+1+lda*(ii+1)] = ptr_pA[1+bs*1];
			A[jj+1+lda*(ii+2)] = ptr_pA[2+bs*1];
			A[jj+1+lda*(ii+3)] = ptr_pA[3+bs*1];
			// unroll 2
			A[jj+2+lda*(ii+0)] = ptr_pA[0+bs*2];
			A[jj+2+lda*(ii+1)] = ptr_pA[1+bs*2];
			A[jj+2+lda*(ii+2)] = ptr_pA[2+bs*2];
			A[jj+2+lda*(ii+3)] = ptr_pA[3+bs*2];
			// unroll 3
			A[jj+3+lda*(ii+0)] = ptr_pA[0+bs*3];
			A[jj+3+lda*(ii+1)] = ptr_pA[1+bs*3];
			A[jj+3+lda*(ii+2)] = ptr_pA[2+bs*3];
			A[jj+3+lda*(ii+3)] = ptr_pA[3+bs*3];
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

	const int bs = 4;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	pA[0] = a;
	return;
	}



// extract element from strmat
float blasfeo_sgeex1(struct blasfeo_smat *sA, int ai, int aj)
	{
	const int bs = 4;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	return pA[0];
	}



// insert element into strvec
void blasfeo_svecin1(float a, struct blasfeo_svec *sx, int xi)
	{
	const int bs = 4;
	float *x = sx->pa + xi;
	x[0] = a;
	return;
	}



// extract element from strvec
float blasfeo_svecex1(struct blasfeo_svec *sx, int xi)
	{
	const int bs = 4;
	float *x = sx->pa + xi;
	return x[0];
	}



// set all elements of a strmat to a value
void blasfeo_sgese(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
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
	for(ii=0; ii<m-3; ii+=4)
		{
		for(jj=0; jj<n; jj++)
			{
			pA[0+jj*bs] = alpha;
			pA[1+jj*bs] = alpha;
			pA[2+jj*bs] = alpha;
			pA[3+jj*bs] = alpha;
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
	const int bs = 4;
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

	const int bs = 4;
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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pA[jj*sda+(jj+0)*bs+0] = alpha*x[jj+0];
		pA[jj*sda+(jj+1)*bs+1] = alpha*x[jj+1];
		pA[jj*sda+(jj+2)*bs+2] = alpha*x[jj+2];
		pA[jj*sda+(jj+3)*bs+3] = alpha*x[jj+3];
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

	const int bs = 4;
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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pA[jj*sda+(jj+0)*bs+0] += alpha;
		pA[jj*sda+(jj+1)*bs+1] += alpha;
		pA[jj*sda+(jj+2)*bs+2] += alpha;
		pA[jj*sda+(jj+3)*bs+3] += alpha;
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

	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	const int bs = 4;
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
	const int bs = 4;
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

	const int bs = 4;
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

	const int bs = 4;
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

	const int bs = 4;
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

	const int bs = 4;
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

	const int bs = 4;
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

	const int bs = 4;

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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pA[jj*sda+0] *= alpha;
		pA[jj*sda+1] *= alpha;
		pA[jj*sda+2] *= alpha;
		pA[jj*sda+3] *= alpha;
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

	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	const int bs = 4;
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



// --- ge


// scale a generic strmat
void blasfeo_sgesc(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{

	if(m<=0 | n<=0)
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

	const int bs = 4;

	int mna, ii;

	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	int offA = ai%bs;

	// same alignment
	ii = 0;
	// clean up at the beginning
	mna = (4-offA)%bs;
	if(mna>0)
		{
		if(m<mna) // mna<=3  ==>  m = { 1, 2 }
			{
			if(m==1)
				{
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pA+offA);
				return;
				}
			else //if(m==2 && mna==3)
				{
				kernel_sgecpsc_2_0_lib4(n, &alpha, pA+offA, pA+offA);
				return;
				}
			}
		if(mna==1)
			{
			kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pA+offA);
			pA += 4*sda;
			ii += 1;
			}
		else if(mna==2)
			{
			kernel_sgecpsc_2_0_lib4(n, &alpha, pA+offA, pA+offA);
			pA += 4*sda;
			ii += 2;
			}
		else // if(mna==3)
			{
			kernel_sgecpsc_3_0_lib4(n, &alpha, pA+offA, pA+offA);
			pA += 4*sda;
			ii += 3;
			}
		}
	// main loop
	for(; ii<m-3; ii+=4)
		{
		kernel_sgecpsc_4_0_lib4(n, &alpha, pA, pA);
		pA += 4*sda;
		}
	// clean up at the end
	if(ii<m)
		{
		if(m-ii==1)
			kernel_sgecpsc_1_0_lib4(n, &alpha, pA, pA);
		else if(m-ii==2)
			kernel_sgecpsc_2_0_lib4(n, &alpha, pA, pA);
		else // if(m-ii==3)
			kernel_sgecpsc_3_0_lib4(n, &alpha, pA, pA);
		}

	return;

	}



// copy and scale a generic strmat into a generic strmat
void blasfeo_sgecpsc(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	if(m<=0 | n<=0)
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

	const int bs = 4;

	int mna, ii;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offA = ai%bs;
	int offB = bi%bs;

	// same alignment
	if(offA==offB)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_sgecpsc_2_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecpsc_2_0_lib4(n, &alpha, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecpsc_3_0_lib4(n, &alpha, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgecpsc_4_0_lib4(n, &alpha, pA, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA, pB);
			else if(m-ii==2)
				kernel_sgecpsc_2_0_lib4(n, &alpha, pA, pB);
			else // if(m-ii==3)
				kernel_sgecpsc_3_0_lib4(n, &alpha, pA, pB);
			}
		}
	// skip one element of pA
	else if(offA==(offB+1)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_sgecpsc_2_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pB+offB);
				//pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecpsc_2_3_lib4(n, &alpha, pA, sda, pB+2);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecpsc_3_2_lib4(n, &alpha, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for( ; ii<m-3; ii+=4)
			{
			kernel_sgecpsc_4_1_lib4(n, &alpha, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+1, pB);
			else if(m-ii==2)
				kernel_sgecpsc_2_0_lib4(n, &alpha, pA+1, pB);
			else // if(m-ii==3)
				kernel_sgecpsc_3_0_lib4(n, &alpha, pA+1, pB);
			}
		}
	// skip 2 elements of pA
	else if(offA==(offB+2)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_sgecpsc_2_3_lib4(n, &alpha, pA, sda, pB+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+1, pB+3);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecpsc_2_0_lib4(n, &alpha, pA, pB+2);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecpsc_3_3_lib4(n, &alpha, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgecpsc_4_2_lib4(n, &alpha, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+2, pB);
			else if(m-ii==2)
				kernel_sgecpsc_2_0_lib4(n, &alpha, pA+2, pB);
			else // if(m-ii==3)
				kernel_sgecpsc_3_2_lib4(n, &alpha, pA, sda, pB);
			}
		}
	// skip 3 elements of pA
	else // if(offA==(offB+3)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;

		if(mna>0)
			{

			if(m<mna)
				{
				if(m==1)
					{
					kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_sgecpsc_2_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				}

			if(mna==1)
				{
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecpsc_2_0_lib4(n, &alpha, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecpsc_3_0_lib4(n, &alpha, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}

			}

		// main loop

		for(; ii<m-3; ii+=4)
			{
			kernel_sgecpsc_4_3_lib4(n, &alpha, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}

		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecpsc_1_0_lib4(n, &alpha, pA+3, pB);
			else if(m-ii==2)
				kernel_sgecpsc_2_3_lib4(n, &alpha, pA, sda, pB);
			else // if(m-ii==3)
				kernel_sgecpsc_3_3_lib4(n, &alpha, pA, sda, pB);
			}
		}

	return;

	}



// copy a generic strmat into a generic strmat
void blasfeo_sgecp(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	if(m<=0 | n<=0)
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

	const int bs = 4;

	int mna, ii;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offA = ai%bs;
	int offB = bi%bs;

	// same alignment
	if(offA==offB)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_sgecp_1_0_lib4(n, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_sgecp_2_0_lib4(n, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgecp_1_0_lib4(n, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecp_2_0_lib4(n, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecp_3_0_lib4(n, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgecp_4_0_lib4(n, pA, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecp_1_0_lib4(n, pA, pB);
			else if(m-ii==2)
				kernel_sgecp_2_0_lib4(n, pA, pB);
			else // if(m-ii==3)
				kernel_sgecp_3_0_lib4(n, pA, pB);
			}
		}
	// skip one element of pA
	else if(offA==(offB+1)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_sgecp_1_0_lib4(n, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_sgecp_2_0_lib4(n, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgecp_1_0_lib4(n, pA+offA, pB+offB);
				//pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecp_2_3_lib4(n, pA, sda, pB+2);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecp_3_2_lib4(n, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for( ; ii<m-3; ii+=4)
			{
			kernel_sgecp_4_1_lib4(n, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecp_1_0_lib4(n, pA+1, pB);
			else if(m-ii==2)
				kernel_sgecp_2_0_lib4(n, pA+1, pB);
			else // if(m-ii==3)
				kernel_sgecp_3_0_lib4(n, pA+1, pB);
			}
		}
	// skip 2 elements of pA
	else if(offA==(offB+2)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_sgecp_1_0_lib4(n, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_sgecp_2_3_lib4(n, pA, sda, pB+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgecp_1_0_lib4(n, pA+1, pB+3);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecp_2_0_lib4(n, pA, pB+2);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecp_3_3_lib4(n, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgecp_4_2_lib4(n, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecp_1_0_lib4(n, pA+2, pB);
			else if(m-ii==2)
				kernel_sgecp_2_0_lib4(n, pA+2, pB);
			else // if(m-ii==3)
				kernel_sgecp_3_2_lib4(n, pA, sda, pB);
			}
		}
	// skip 3 elements of pA
	else // if(offA==(offB+3)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_sgecp_1_0_lib4(n, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_sgecp_2_0_lib4(n, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgecp_1_0_lib4(n, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgecp_2_0_lib4(n, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgecp_3_0_lib4(n, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgecp_4_3_lib4(n, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgecp_1_0_lib4(n, pA+3, pB);
			else if(m-ii==2)
				kernel_sgecp_2_3_lib4(n, pA, sda, pB);
			else // if(m-ii==3)
				kernel_sgecp_3_3_lib4(n, pA, sda, pB);
			}
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
void blasfeo_strcp_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	if(m<=0)
		return;

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int bs = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offA = ai%bs;
	int offB = bi%bs;

	int ii, mna;

	// same alignment
	if(offA==offB)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_strcp_l_1_0_lib4(ii, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_strcp_l_2_0_lib4(ii, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_strcp_l_1_0_lib4(ii, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_strcp_l_2_0_lib4(ii, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_strcp_l_3_0_lib4(ii, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_strcp_l_4_0_lib4(ii, pA, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_strcp_l_1_0_lib4(ii, pA, pB);
			else if(m-ii==2)
				kernel_strcp_l_2_0_lib4(ii, pA, pB);
			else // if(m-ii==3)
				kernel_strcp_l_3_0_lib4(ii, pA, pB);
			}
		}
	// skip one element of pA
	else if(offA==(offB+1)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_strcp_l_1_0_lib4(ii, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_strcp_l_2_0_lib4(ii, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_strcp_l_1_0_lib4(ii, pA+offA, pB+offB);
				//pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_strcp_l_2_3_lib4(ii, pA, sda, pB+2);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_strcp_l_3_2_lib4(ii, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for( ; ii<m-3; ii+=4)
			{
			kernel_strcp_l_4_1_lib4(ii, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_strcp_l_1_0_lib4(ii, pA+1, pB);
			else if(m-ii==2)
				kernel_strcp_l_2_0_lib4(ii, pA+1, pB);
			else // if(m-ii==3)
				kernel_strcp_l_3_0_lib4(ii, pA+1, pB);
			}
		}
	// skip 2 elements of pA
	else if(offA==(offB+2)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_strcp_l_1_0_lib4(ii, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_strcp_l_2_3_lib4(ii, pA, sda, pB+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_strcp_l_1_0_lib4(ii, pA+1, pB+3);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_strcp_l_2_0_lib4(ii, pA, pB+2);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_strcp_l_3_3_lib4(ii, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_strcp_l_4_2_lib4(ii, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_strcp_l_1_0_lib4(ii, pA+2, pB);
			else if(m-ii==2)
				kernel_strcp_l_2_0_lib4(ii, pA+2, pB);
			else // if(m-ii==3)
				kernel_strcp_l_3_2_lib4(ii, pA, sda, pB);
			}
		}
	// skip 3 elements of pA
	else // if(offA==(offB+3)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_strcp_l_1_0_lib4(ii, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_strcp_l_2_0_lib4(ii, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_strcp_l_1_0_lib4(ii, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_strcp_l_2_0_lib4(ii, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_strcp_l_3_0_lib4(ii, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_strcp_l_4_3_lib4(ii, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_strcp_l_1_0_lib4(ii, pA+3, pB);
			else if(m-ii==2)
				kernel_strcp_l_2_3_lib4(ii, pA, sda, pB);
			else // if(m-ii==3)
				kernel_strcp_l_3_3_lib4(ii, pA, sda, pB);
			}
		}

	return;

	}



// scale and add a generic strmat into a generic strmat
void blasfeo_sgead(int m, int n, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sB, int bi, int bj)
	{

	if(m<=0 || n<=0)
		return;

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int bs = 4;

	int sda = sA->cn;
	int sdb = sB->cn;
	float *pA = sA->pA + ai/bs*bs*sda + aj*bs;
	float *pB = sB->pA + bi/bs*bs*sdb + bj*bs;
	int offA = ai%bs;
	int offB = bi%bs;

	int ii, mna;

	// same alignment
	if(offA==offB)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_sgead_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_sgead_2_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgead_1_0_lib4(n, &alpha, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgead_2_0_lib4(n, &alpha, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgead_3_0_lib4(n, &alpha, pA+offA, pB+offB);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgead_4_0_lib4(n, &alpha, pA, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgead_1_0_lib4(n, &alpha, pA, pB);
			else if(m-ii==2)
				kernel_sgead_2_0_lib4(n, &alpha, pA, pB);
			else // if(m-ii==3)
				kernel_sgead_3_0_lib4(n, &alpha, pA, pB);
			}
		}
	// skip one element of pA
	else if(offA==(offB+1)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_sgead_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_sgead_2_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgead_1_0_lib4(n, &alpha, pA+offA, pB+offB);
				//pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgead_2_3_lib4(n, &alpha, pA, sda, pB+2);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgead_3_2_lib4(n, &alpha, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for( ; ii<m-3; ii+=4)
			{
			kernel_sgead_4_1_lib4(n, &alpha, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgead_1_0_lib4(n, &alpha, pA+1, pB);
			else if(m-ii==2)
				kernel_sgead_2_0_lib4(n, &alpha, pA+1, pB);
			else // if(m-ii==3)
				kernel_sgead_3_0_lib4(n, &alpha, pA+1, pB);
			}
		}
	// skip 2 elements of pA
	else if(offA==(offB+2)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_sgead_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_sgead_2_3_lib4(n, &alpha, pA, sda, pB+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgead_1_0_lib4(n, &alpha, pA+1, pB+3);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgead_2_0_lib4(n, &alpha, pA, pB+2);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgead_3_3_lib4(n, &alpha, pA, sda, pB+1);
				pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgead_4_2_lib4(n, &alpha, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgead_1_0_lib4(n, &alpha, pA+2, pB);
			else if(m-ii==2)
				kernel_sgead_2_0_lib4(n, &alpha, pA+2, pB);
			else // if(m-ii==3)
				kernel_sgead_3_2_lib4(n, &alpha, pA, sda, pB);
			}
		}
	// skip 3 elements of pA
	else // if(offA==(offB+3)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_sgead_1_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_sgead_2_0_lib4(n, &alpha, pA+offA, pB+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_sgead_1_0_lib4(n, &alpha, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_sgead_2_0_lib4(n, &alpha, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_sgead_3_0_lib4(n, &alpha, pA+offA, pB+offB);
				// pA += 4*sda;
				pB += 4*sdb;
				ii += 3;
				}
			}
		// main loop
		for(; ii<m-3; ii+=4)
			{
			kernel_sgead_4_3_lib4(n, &alpha, pA, sda, pB);
			pA += 4*sda;
			pB += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_sgead_1_0_lib4(n, &alpha, pA+3, pB);
			else if(m-ii==2)
				kernel_sgead_2_3_lib4(n, &alpha, pA, sda, pB);
			else // if(m-ii==3)
				kernel_sgead_3_3_lib4(n, &alpha, pA, sda, pB);
			}
		}

	return;

	}



// copy and transpose a generic strmat into a generic strmat
void blasfeo_sgetr(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	if(m<=0 || n<=0)
		return;

	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	float *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	sgetr_lib(m, n, 1.0, ai%bs, pA, sda, ci%bs, pC, sdc); // TODO remove alpha !!!
	return;
	}



// copy and transpose a lower triangular strmat into an upper triangular strmat
void blasfeo_strtr_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	float *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	strtr_l_lib(m, 1.0, ai%bs, pA, sda, ci%bs, pC, sdc); // TODO remove alpha !!!
	return;
	}



// copy and transpose an upper triangular strmat into a lower triangular strmat
void blasfeo_strtr_u(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	float *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	float *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	strtr_u_lib(m, 1.0, ai%bs, pA, sda, ci%bs, pC, sdc); // TODO remove alpha !!!
	return;
	}



// insert a strvec to diagonal of strmat, sparse formulation
void blasfeo_sdiain_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{
	if(kmax<=0)
		return;

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 4;
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
	const int bs = 4;
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
	if(kmax<=0)
		return;

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 4;
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
	if(kmax<=0)
		return;

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 4;
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
	if(kmax<=0)
		return;

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 4;
	float *x = sx->pa + xi;
	int sdd = sD->cn;
	float *pD = sD->pA + di/bs*bs*sdd + di%bs + dj*bs;
	srowad_libsp(kmax, idx, alpha, x, pD);
	return;
	}



// adds strvec to strvec, sparse formulation
void blasfeo_svecad_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_svec *sy, int yi)
	{
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	svecad_libsp(kmax, idx, alpha, x, y);
	return;
	}



void blasfeo_svecin_sp(int m, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] = alpha * x[ii];
	return;
	}



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



