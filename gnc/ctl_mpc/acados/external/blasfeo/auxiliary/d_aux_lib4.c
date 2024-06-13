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
/*
 * auxiliary functions for LA:HIGH_PERFORMANCE PS_4 (panel major)
 *
 * auxiliary/d_aux_lib*.c
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#endif

#include <blasfeo_common.h>
#include <blasfeo_block_size.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_d_aux_ref.h>
#endif


/*
 * Auxiliary functions for PS:4 (panel-major)
 *
 * auxiliary/d_aux_lib4.c
 * TOFIX: move inside str* interface
 *
 */



// scales and adds a packed matrix into a packed matrix: B = B + alpha*A
void dgead_lib(int m, int n, double alpha, int offsetA, double *A, int sda, int offsetB, double *B, int sdb)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna, ii;

	int offA = offsetA%bs;
	int offB = offsetB%bs;

	// A at the beginning of the block
	A -= offA;

	// A at the beginning of the block
	B -= offB;

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
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_0_lib4(n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgead_8_0_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgead_4_0_lib4(n, alpha, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A, B);
			else if(m-ii==2)
				kernel_dgead_2_0_lib4(n, alpha, A, B);
			else // if(m-ii==3)
				kernel_dgead_3_0_lib4(n, alpha, A, B);
			}
		}
	// skip one element of A
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
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_3_lib4(n, alpha, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_2_lib4(n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgead_8_1_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgead_4_1_lib4(n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A+1, B);
			else if(m-ii==2)
				kernel_dgead_2_0_lib4(n, alpha, A+1, B);
			else // if(m-ii==3)
				kernel_dgead_3_0_lib4(n, alpha, A+1, B);
			}
		}
	// skip 2 elements of A
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
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgead_2_3_lib4(n, alpha, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_0_lib4(n, alpha, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_3_lib4(n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgead_8_2_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgead_4_2_lib4(n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A+2, B);
			else if(m-ii==2)
				kernel_dgead_2_0_lib4(n, alpha, A+2, B);
			else // if(m-ii==3)
				kernel_dgead_3_2_lib4(n, alpha, A, sda, B);
			}
		}
	// skip 3 elements of A
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
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_0_lib4(n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgead_8_3_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgead_4_3_lib4(n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A+3, B);
			else if(m-ii==2)
				kernel_dgead_2_3_lib4(n, alpha, A, sda, B);
			else // if(m-ii==3)
				kernel_dgead_3_3_lib4(n, alpha, A, sda, B);
			}
		}

	}



// --------- Transpose

// transpose general matrix; m and n are referred to the original matrix
void dgetr_lib(int m, int n, double alpha, int offsetA, double *pA, int sda, int offsetC, double *pC, int sdc)
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
			kernel_dgetr_1_lib4(0, n, nna, alpha, pA, pC, sdc);
		else if(mna==2)
			kernel_dgetr_2_lib4(0, n, nna, alpha, pA, pC, sdc);
		else //if(mna==3)
			kernel_dgetr_3_lib4(0, n, nna, alpha, pA, pC, sdc);
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
#if defined(TARGET_X64_INTEL_HASWELL)
	for( ; ii<m-7; ii+=8)
		{
		kernel_dgetr_8_lib4(0, n, nna, alpha, pA, sda, pC, sdc);
		pA += 2*bs*sda;
		pC += 2*bs*bs;
		}
#endif
	for( ; ii<m-3; ii+=4)
//	for( ; ii<m; ii+=4)
		{
		kernel_dgetr_4_lib4(0, n, nna, alpha, pA, pC, sdc);
		pA += bs*sda;
		pC += bs*bs;
		}

	// clean-up at the end using smaller kernels
	if(ii==m)
		return;

	if(m-ii==1)
		kernel_dgetr_1_lib4(0, n, nna, alpha, pA, pC, sdc);
	else if(m-ii==2)
		kernel_dgetr_2_lib4(0, n, nna, alpha, pA, pC, sdc);
	else if(m-ii==3)
		kernel_dgetr_3_lib4(0, n, nna, alpha, pA, pC, sdc);

	return;

	}



// transpose lower triangular matrix
void dtrtr_l_lib(int m, double alpha, int offsetA, double *pA, int sda, int offsetC, double *pC, int sdc)
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
#if 0 //defined(TARGET_X64_INTEL_HASWELL)
	for( ; ii<m-7; ii+=8)
		{
		kernel_dgetr_8_lib4(1, n, nna, alpha, pA, sda, pC, sdc);
		pA += 2*bs*sda;
		pC += 2*bs*bs;
		}
#endif
	for( ; ii<m-3; ii+=4)
		{
		kernel_dgetr_4_lib4(1, ii, nna, alpha, pA, pC, sdc);
		pA += bs*sda;
		pC += bs*bs;
		}

	// clean-up at the end using smaller kernels
	if(ii==m)
		return;

	if(m-ii==1)
		kernel_dgetr_1_lib4(1, ii, nna, alpha, pA, pC, sdc);
	else if(m-ii==2)
		kernel_dgetr_2_lib4(1, ii, nna, alpha, pA, pC, sdc);
	else if(m-ii==3)
		kernel_dgetr_3_lib4(1, ii, nna, alpha, pA, pC, sdc);

	return;

	}



// transpose an aligned upper triangular matrix into an aligned lower triangular matrix
void dtrtr_u_lib(int m, double alpha, int offsetA, double *pA, int sda, int offsetC, double *pC, int sdc)
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
			kernel_dgetr_1_lib4(0, n, nna, alpha, pA, pC, sdc);
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
//			kernel_dgetr_1_lib4(0, n-1, tna, alpha, pA, pC, sdc);
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
				kernel_dgetr_2_lib4(0, n-2, tna, alpha, pA, pC, sdc);
				}
			else if(nna==1)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pA += 1*bs;
				pC += 1 + (sdc-1)*bs;
//				pC[0+bs*0] = alpha * pA[0+bs*0];
//				pC[0+bs*1] = alpha * pA[1+bs*0];
				kernel_dgetr_2_lib4(0, n-1, 0, alpha, pA, pC, sdc);
				pA += 1*bs;
				pC += 1;
				tna = 3; //(bs-(offsetC+2)%bs)%bs;
//				kernel_dgetr_2_lib4(0, n-2, tna, alpha, pA, pC, sdc);
				}
			else if(nna==2)
				{
				pC[0+bs*0] = alpha * pA[0+bs*0];
				pC[1+bs*0] = alpha * pA[0+bs*1];
				pC[1+bs*1] = alpha * pA[1+bs*1];
				pA += 2*bs;
				pC += 2 + (sdc-1)*bs;
				tna = 0; //(bs-(offsetC+2)%bs)%bs;
				kernel_dgetr_2_lib4(0, n-2, tna, alpha, pA, pC, sdc);
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
				kernel_dgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
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
				kernel_dgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
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
				kernel_dgetr_3_lib4(0, n-2, 0, alpha, pA, pC, sdc);
				pA += 1*bs;
				pC += 1;
				tna = 3;
//				kernel_dgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
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
				kernel_dgetr_3_lib4(0, n-3, tna, alpha, pA, pC, sdc);
				}
			}
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
#if 0 //defined(TARGET_X64_AVX2)
	for( ; ii<m-7; ii+=8)
		{
		kernel_dgetr_8_lib4(0, n, nna, alpha, pA, sda, pC, sdc);
		pA += 2*bs*sda;
		pC += 2*bs*bs;
		}
#endif
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
			kernel_dgetr_4_lib4(0, n-ii-4, 0, alpha, pA, pC, sdc);
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
			kernel_dgetr_4_lib4(0, n-ii-4, 1, alpha, pA, pC, sdc);
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
			kernel_dgetr_4_lib4(0, n-ii-4, 2, alpha, pA, pC, sdc);
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
			kernel_dgetr_4_lib4(0, n-ii-3, 0, alpha, pA, pC, sdc);
//			pC[0+bs*0] = alpha * pA[0+bs*0];
//			pC[0+bs*1] = alpha * pA[1+bs*0];
//			pC[0+bs*2] = alpha * pA[2+bs*0];
//			pC[0+bs*3] = alpha * pA[3+bs*0];
			pA += bs;
			pC += 1;
//			kernel_dgetr_4_lib4(0, n-ii-4, tna, alpha, pA, pC, sdc);
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
void ddiareg_lib(int kmax, double reg, int offset, double *pD, int sdd)
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



// insert sqrt of vector to diagonal
void ddiain_sqrt_lib(int kmax, double *x, int offset, double *pD, int sdd)
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
void ddiaex_lib(int kmax, double alpha, int offset, double *pD, int sdd, double *x)
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
void ddiaad_lib(int kmax, double alpha, double *x, int offset, double *pD, int sdd)
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
void ddiain_libsp(int kmax, int *idx, double alpha, double *x, double *pD, int sdd)
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
void ddiaex_libsp(int kmax, int *idx, double alpha, double *pD, int sdd, double *x)
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
void ddiaad_libsp(int kmax, int *idx, double alpha, double *x, double *pD, int sdd)
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
void ddiaadin_libsp(int kmax, int *idx, double alpha, double *x, double *y, double *pD, int sdd)
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
void drowin_lib(int kmax, double alpha, double *x, double *pD)
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
void drowex_lib(int kmax, double alpha, double *pD, double *x)
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
void drowad_lib(int kmax, double alpha, double *x, double *pD)
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
void drowin_libsp(int kmax, double alpha, int *idx, double *x, double *pD)
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
void drowad_libsp(int kmax, int *idx, double alpha, double *x, double *pD)
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
void drowadin_libsp(int kmax, int *idx, double alpha, double *x, double *y, double *pD)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] = y[jj] + alpha * x[jj];
		}

	}



// extract vector from column
void dcolex_lib(int kmax, int offset, double *pD, int sdd, double *x)
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
		x[jj+0] = pD[jj*sdd+0];
		x[jj+1] = pD[jj*sdd+1];
		x[jj+2] = pD[jj*sdd+2];
		x[jj+3] = pD[jj*sdd+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[jj+ll] = pD[jj*sdd+ll];
		}

	}



// insert vector to column
void dcolin_lib(int kmax, double *x, int offset, double *pD, int sdd)
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
void dcolad_lib(int kmax, double alpha, double *x, int offset, double *pD, int sdd)
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
void dcolin_libsp(int kmax, int *idx, double *x, double *pD, int sdd)
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
void dcolad_libsp(int kmax, double alpha, int *idx, double *x, double *pD, int sdd)
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
void dcolsw_lib(int kmax, int offsetA, double *pA, int sda, int offsetC, double *pC, int sdc)
	{

	const int bs = 4;

	int ii;

	double tmp;

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
#if defined(EXT_DEP)
		printf("\ndcolsw: feature not implemented yet: offsetA!=offsetC\n\n");
#endif
		exit(1);
		}

	return;

	}



// insert vector to vector, sparse formulation
void dvecin_libsp(int kmax, int *idx, double *x, double *y)
	{

	int jj;

	for(jj=0; jj<kmax; jj++)
		{
		y[idx[jj]] = x[jj];
		}

	}



// adds vector to vector, sparse formulation
void dvecad_libsp(int kmax, int *idx, double alpha, double *x, double *y)
	{

	int jj;

	for(jj=0; jj<kmax; jj++)
		{
		y[idx[jj]] += alpha * x[jj];
		}

	}



/*
 * STRMAT interface (General to specific interface)
 *
 */

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
	if(m<=0 || n<=0)
		return;

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, j, jj, m0, m1, m2;
	double 	*B, *pB;
	sA->use_dA = 0;

	// row vector in sA
	if(m==1)
		{
		for(jj=0; jj<n; jj++)
			{
			pA[jj*bs] = A[jj*lda];
			}
		return;
		}

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256d
		tmp;
#endif
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
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for( ; ii<m-3; ii+=4)
			{
			tmp = _mm256_loadu_pd( &B[0+lda*0] );
			_mm256_store_pd( &pB[0+bs*0], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*1] );
			_mm256_store_pd( &pB[0+bs*1], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*2] );
			_mm256_store_pd( &pB[0+bs*2], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*3] );
			_mm256_store_pd( &pB[0+bs*3], tmp );
			B  += 4;
			pB += bs*sda;
			}
#else
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
#endif
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



// convert a lower triangular matrix into a matrix structure
// TODO	vectorize triangle in the 4x4 diag blocks
void blasfeo_pack_l_dmat(int m, int n, double *A, int lda, struct blasfeo_dmat *sA, int ai, int aj)
	{
	if(m<=0 || n<=0)
		return;

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, j, jj, m0, m1, m2;
	double 	*B, *pB;
	sA->use_dA = 0;

	// row vector in sA
	if(m==1)
		{
		for(jj=0; jj<n; jj++)
			{
			pA[jj*bs] = A[jj*lda];
			}
		return;
		}

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256d
		tmp;
#endif
	m0 = (bs-ai%bs)%bs;
	if(m0>m)
		m0 = m;
	m1 = m - m0;
	if(m0>0)
		{
#if defined(BLASFEO_REF_API)
		blasfeo_ref_pack_l_dmat(m, n, A, lda, sA, ai, aj);
		return;
#else
		printf("\nblasfeo_pack_l_dmat: feature not implemented yet: ai!=0\n");
		exit(1);
#endif
		}
	jj = 0;
	for( ; jj<n-3; jj+=4)
		{
		B  =  A + jj + jj*lda;
		pB = pA + jj*bs + jj*sda;
		ii = jj;
		// col 0
		pB[0+bs*0] = B[0+lda*0];
		pB[1+bs*0] = B[1+lda*0];
		pB[2+bs*0] = B[2+lda*0];
		pB[3+bs*0] = B[3+lda*0];
		// col 1
//		pB[0+bs*1] = B[0+lda*1];
		pB[1+bs*1] = B[1+lda*1];
		pB[2+bs*1] = B[2+lda*1];
		pB[3+bs*1] = B[3+lda*1];
		// col 2
//		pB[0+bs*2] = B[0+lda*2];
//		pB[1+bs*2] = B[1+lda*2];
		pB[2+bs*2] = B[2+lda*2];
		pB[3+bs*2] = B[3+lda*2];
		// col 3
//		pB[0+bs*3] = B[0+lda*3];
//		pB[1+bs*3] = B[1+lda*3];
//		pB[2+bs*3] = B[2+lda*3];
		pB[3+bs*3] = B[3+lda*3];
		B  += 4;
		pB += bs*sda;
		ii += 4;
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for( ; ii<m-3; ii+=4)
			{
			tmp = _mm256_loadu_pd( &B[0+lda*0] );
			_mm256_store_pd( &pB[0+bs*0], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*1] );
			_mm256_store_pd( &pB[0+bs*1], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*2] );
			_mm256_store_pd( &pB[0+bs*2], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*3] );
			_mm256_store_pd( &pB[0+bs*3], tmp );
			B  += 4;
			pB += bs*sda;
			}
#else
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
#endif
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
	if(jj<n)
		{
		B  =  A + jj + jj*lda;
		pB = pA + jj*bs + jj*sda;
		if(n-jj==1)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			}
		else if(n-jj==2)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			pB[1+bs*0] = B[1+lda*0];
			// col 1
//			pB[0+bs*1] = B[0+lda*1];
			pB[1+bs*1] = B[1+lda*1];
			}
		else //if(n-jj==3)
			{
			// col 0
			pB[0+bs*0] = B[0+lda*0];
			pB[1+bs*0] = B[1+lda*0];
			pB[2+bs*0] = B[2+lda*0];
			// col 1
//			pB[0+bs*1] = B[0+lda*1];
			pB[1+bs*1] = B[1+lda*1];
			pB[2+bs*1] = B[2+lda*1];
			// col 2
//			pB[0+bs*2] = B[0+lda*2];
//			pB[1+bs*2] = B[1+lda*2];
			pB[2+bs*2] = B[2+lda*2];
			}
		}
	return;
	}



// convert a upper triangular matrix into a matrix structure
// TODO	vectorize triangle in the 4x4 diag blocks
void blasfeo_pack_u_dmat(int m, int n, double *A, int lda, struct blasfeo_dmat *sA, int ai, int aj)
	{
	if(m<=0 || n<=0)
		return;

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, j, jj, m0, m1, m2;
	double 	*B, *pB;
	sA->use_dA = 0;

	// row vector in sA
	if(m==1)
		{
		for(jj=0; jj<n; jj++)
			{
			pA[jj*bs] = A[jj*lda];
			}
		return;
		}

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256d
		tmp;
#endif
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
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for( ; ii<jj-3; ii+=4)
			{
			tmp = _mm256_loadu_pd( &B[0+lda*0] );
			_mm256_store_pd( &pB[0+bs*0], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*1] );
			_mm256_store_pd( &pB[0+bs*1], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*2] );
			_mm256_store_pd( &pB[0+bs*2], tmp );
			tmp = _mm256_loadu_pd( &B[0+lda*3] );
			_mm256_store_pd( &pB[0+bs*3], tmp );
			B  += 4;
			pB += bs*sda;
			}
#else
		for( ; ii<jj-3; ii+=4)
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
#endif
		// col 0
		pB[0+bs*0] = B[0+lda*0];
//		pB[1+bs*0] = B[1+lda*0];
//		pB[2+bs*0] = B[2+lda*0];
//		pB[3+bs*0] = B[3+lda*0];
		// col 1
		pB[0+bs*1] = B[0+lda*1];
		pB[1+bs*1] = B[1+lda*1];
//		pB[2+bs*1] = B[2+lda*1];
//		pB[3+bs*1] = B[3+lda*1];
		// col 2
		pB[0+bs*2] = B[0+lda*2];
		pB[1+bs*2] = B[1+lda*2];
		pB[2+bs*2] = B[2+lda*2];
//		pB[3+bs*2] = B[3+lda*2];
		// col 3
		pB[0+bs*3] = B[0+lda*3];
		pB[1+bs*3] = B[1+lda*3];
		pB[2+bs*3] = B[2+lda*3];
		pB[3+bs*3] = B[3+lda*3];
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
		for( ; ii<jj-3; ii+=4)
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
		for( ; ii<=jj; ii++)
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
void blasfeo_pack_tran_dmat(int m, int n, double *A, int lda, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, j, m0, m1, m2;
	double 	*B, *pB;
	sA->use_dA = 0;

	// row vector in sA
	if(n==1)
		{
		for(ii=0; ii<m; ii++)
			{
			pA[ii*bs] = A[ii];
			}
		return;
		}

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256d
		v0, v1, v2, v3,
		v4, v5, v6, v7;
#endif
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
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for(; j<m-3; j+=4)
			{
			v0 = _mm256_loadu_pd( &B[0+0*lda] ); // 00 10 20 30
			v1 = _mm256_loadu_pd( &B[0+1*lda] ); // 01 11 21 31
			v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 20 21
			v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 30 31
			v2 = _mm256_loadu_pd( &B[0+2*lda] ); // 02 12 22 32
			v3 = _mm256_loadu_pd( &B[0+3*lda] ); // 03 13 23 33
			v6 = _mm256_unpacklo_pd( v2, v3 ); // 02 03 22 23
			v7 = _mm256_unpackhi_pd( v2, v3 ); // 12 13 32 33

			B += 4;

			v0 = _mm256_permute2f128_pd( v4, v6, 0x20 ); // 00 01 02 03
			_mm256_store_pd( &pB[0+bs*0], v0 );
			v2 = _mm256_permute2f128_pd( v4, v6, 0x31 ); // 20 21 22 23
			_mm256_store_pd( &pB[0+bs*2], v2 );
			v1 = _mm256_permute2f128_pd( v5, v7, 0x20 ); // 10 11 12 13
			_mm256_store_pd( &pB[0+bs*1], v1 );
			v3 = _mm256_permute2f128_pd( v5, v7, 0x31 ); // 30 31 32 33
			_mm256_store_pd( &pB[0+bs*3], v3 );

			pB += 4*bs;
			}
#else
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
#endif
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
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, jj;
	int m0 = (bs-ai%bs)%bs;
	if(m0>m)
		m0 = m;
	double *ptr_pA;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256d
		tmp;
#endif

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
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for(; ii<m-bs+1; ii+=bs)
			{
			tmp = _mm256_load_pd( &ptr_pA[0+bs*0] );
			_mm256_storeu_pd( &A[ii+lda*(jj+0)], tmp );
			tmp = _mm256_load_pd( &ptr_pA[0+bs*1] );
			_mm256_storeu_pd( &A[ii+lda*(jj+1)], tmp );
			tmp = _mm256_load_pd( &ptr_pA[0+bs*2] );
			_mm256_storeu_pd( &A[ii+lda*(jj+2)], tmp );
			tmp = _mm256_load_pd( &ptr_pA[0+bs*3] );
			_mm256_storeu_pd( &A[ii+lda*(jj+3)], tmp );
			ptr_pA += sda*bs;
			}
#else
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
#endif
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
void blasfeo_unpack_tran_dmat(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, double *A, int lda)
	{
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + aj*bs + ai/bs*bs*sda + ai%bs;
	int i, ii, jj;
	int m0 = (bs-ai%bs)%bs;
	if(m0>m)
		m0 = m;
	double *ptr_pA;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256d
		v0, v1, v2, v3,
		v4, v5, v6, v7;
#endif

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
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
		for(; ii<m-bs+1; ii+=bs)
			{
			v0 = _mm256_load_pd( &ptr_pA[0+bs*0] ); // 00 10 20 30
			v1 = _mm256_load_pd( &ptr_pA[0+bs*1] ); // 01 11 21 31
			v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 20 21
			v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 30 31
			v2 = _mm256_load_pd( &ptr_pA[0+bs*2] ); // 02 12 22 32
			v3 = _mm256_load_pd( &ptr_pA[0+bs*3] ); // 03 13 23 33
			v6 = _mm256_unpacklo_pd( v2, v3 ); // 02 03 22 23
			v7 = _mm256_unpackhi_pd( v2, v3 ); // 12 13 32 33

			v0 = _mm256_permute2f128_pd( v4, v6, 0x20 ); // 00 01 02 03
			_mm256_storeu_pd( &A[jj+lda*(ii+0)], v0 );
			v2 = _mm256_permute2f128_pd( v4, v6, 0x31 ); // 20 21 22 23
			_mm256_storeu_pd( &A[jj+lda*(ii+2)], v2 );
			v1 = _mm256_permute2f128_pd( v5, v7, 0x20 ); // 10 11 12 13
			_mm256_storeu_pd( &A[jj+lda*(ii+1)], v1 );
			v3 = _mm256_permute2f128_pd( v5, v7, 0x31 ); // 30 31 32 33
			_mm256_storeu_pd( &A[jj+lda*(ii+3)], v3 );

			ptr_pA += sda*bs;
			}
#else
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
#endif
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



#if 0
// cast a matrix into a matrix structure
void d_cast_mat2strmat(double *A, struct blasfeo_dmat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	sA->pA = A;
	return;
	}



// cast a matrix into the diagonal of a matrix structure
void d_cast_diag_mat2strmat(double *dA, struct blasfeo_dmat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	sA->dA = dA;
	return;
	}



// cast a vector into a vector structure
void d_cast_vec2vecmat(double *a, struct blasfeo_dvec *sa)
	{
	sa->pa = a;
	return;
	}
#endif



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

	const int bs = 4;
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

	const int bs = 4;
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
void blasfeo_ddiare(int kmax, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
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
void blasfeo_drowsw(int kmax, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	double *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	kernel_drowsw_lib4(kmax, pA, pC);
	return;
	}



// permute the rows of a matrix struct
void blasfeo_drowpe(int kmax, int *ipiv, struct blasfeo_dmat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			blasfeo_drowsw(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}



// inverse permute the rows of a matrix struct
void blasfeo_drowpei(int kmax, int *ipiv, struct blasfeo_dmat *sA)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			blasfeo_drowsw(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}



// extract a row int a vector
void blasfeo_drowex(int kmax, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi)
	{
	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	drowex_lib(kmax, alpha, pA, x);
	return;
	}



// insert a vector into a row
void blasfeo_drowin(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	drowin_lib(kmax, alpha, x, pA);
	return;
	}



// add a vector to a row
void blasfeo_drowad(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	drowad_lib(kmax, alpha, x, pA);
	return;
	}



// extract vector from column
void blasfeo_dcolex(int kmax, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	dcolex_lib(kmax, ai%bs, pA, sda, x);
	return;
	}



// insert as vector as a column
void blasfeo_dcolin(int kmax, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	dcolin_lib(kmax, x, ai%bs, pA, sda);
	return;
	}



// add scaled vector to column
void blasfeo_dcolad(int kmax, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *x = sx->pa + xi;
	dcolad_lib(kmax, alpha, x, ai%bs, pA, sda);
	return;
	}



// scale a column
void blasfeo_dcolsc(int kmax, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	const int bs = 4;

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
void blasfeo_dcolsw(int kmax, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{

	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	const int bs = 4;
#if defined(BLASFEO_REF_API)
	if(ai%bs!=ci%bs)
		{
		blasfeo_ref_dcolsw(kmax, sA, ai, aj, sC, ci, cj);
		return;
		}
#endif
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	double *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	dcolsw_lib(kmax, ai%bs, pA, sda, ci%bs, pC, sdc);
	return;
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

	const int bs = 4;
	const double alpha = 1.0;

	// get submatrices
	int sda = sA->cn;
	double *A = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdb = sB->cn;
	double *B = sB->pA + bi/bs*bs*sdb + bi%bs + bj*bs;

	if(m<=0 || n<=0)
		return;

	int mna, ii;

	// compute offset from closest panels
	int offA = ai%bs;
	int offB = bi%bs;

	// A at the beginning of the block
	A -= offA;
	// A at the beginning of the block
	B -= offB;

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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_0_lib4(0, n, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_0_lib4(0, n, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A, B);
			}
		}
	// skip one element of A
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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_3_lib4(0, n, alpha, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_2_lib4(0, n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_1_lib4(0, n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_1_lib4(0, n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+1, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+1, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A+1, B);
			}
		}
	// skip 2 elements of A
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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_3_lib4(0, n, alpha, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_3_lib4(0, n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_2_lib4(0, n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_2_lib4(0, n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+2, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+2, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_2_lib4(0, n, alpha, A, sda, B);
			}
		}
	// skip 3 elements of A
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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_3_lib4(0, n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_3_lib4(0, n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+3, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_3_lib4(0, n, alpha, A, sda, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_3_lib4(0, n, alpha, A, sda, B);
			}
		}

	return;
	}



// copy a lower triangular strmat into a lower triangular strmat
void blasfeo_dtrcp_l(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int bs = 4;
	const double alpha = 1;

	// get submatrices
	int sda = sA->cn;
	double *A = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdb = sB->cn;
	double *B = sB->pA + bi/bs*bs*sdb + bi%bs + bj*bs;

	if(m<=0)
		return;

	int n = m;

	int mna, ii;

	int offA = ai%bs;
	int offB = bi%bs;

	// A at the beginning of the block
	A -= offA;

	// A at the beginning of the block
	B -= offB;

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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_0_lib4(1, ii, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_0_lib4(1, ii, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A, B);
			}
		}
	// skip one element of A
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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_3_lib4(1, ii, alpha, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_2_lib4(1, ii, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_1_lib4(1, ii, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_1_lib4(1, ii, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+1, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+1, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A+1, B);
			}
		}
	// skip 2 elements of A
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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_3_lib4(1, ii, alpha, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_3_lib4(1, ii, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_2_lib4(1, ii, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_2_lib4(1, ii, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+2, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+2, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_2_lib4(1, ii, alpha, A, sda, B);
			}
		}
	// skip 3 elements of A
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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_3_lib4(1, ii, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_3_lib4(1, ii, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+3, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_3_lib4(1, ii, alpha, A, sda, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_3_lib4(1, ii, alpha, A, sda, B);
			}
		}

	/* } */
	return;
	}



// copy and scale a generic strmat into a generic strmat
void blasfeo_dgecpsc(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int bs = 4;

	// extract dimension
	int sda = sA->cn;
	int sdb = sB->cn;

	// extract submatrix
	double *A = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	double *B = sB->pA + bi/bs*bs*sdb + bi%bs + bj*bs;

	if(m<=0 || n<=0)
		return;

	int mna, ii;

	int offA = ai%bs;
	int offB = bi%bs;

	// A at the beginning of the block
	A -= offA;

	// A at the beginning of the block
	B -= offB;

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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_0_lib4(0, n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_0_lib4(0, n, alpha, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A, B);
			}
		}
	// skip one element of A
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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_3_lib4(0, n, alpha, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_2_lib4(0, n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_1_lib4(0, n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_1_lib4(0, n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+1, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+1, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A+1, B);
			}
		}
	// skip 2 elements of A
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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_3_lib4(0, n, alpha, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_3_lib4(0, n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_2_lib4(0, n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_2_lib4(0, n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+2, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+2, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_2_lib4(0, n, alpha, A, sda, B);
			}
		}
	// skip 3 elements of A
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
					kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(0, n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(0, n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_3_lib4(0, n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_3_lib4(0, n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(0, n, alpha, A+3, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_3_lib4(0, n, alpha, A, sda, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_3_lib4(0, n, alpha, A, sda, B);
			}
		}
	return;
	}



// copy  and scale a lower triangular strmat into a lower triangular strmat
void blasfeo_dtrcpsc_l(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{

	// invalidate stored inverse diagonal
	sB->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *A = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdb = sB->cn;
	double *B = sB->pA + bi/bs*bs*sdb + bi%bs + bj*bs;

	/* dtrcp_l_lib(m, 1.0, ai%bs, pA, sda, ci%bs, pC, sdc); */
	/* // copies a lower triangular packed matrix into a lower triangular packed matrix */
	/* void dtrcp_l_lib(int m, double alpha, int offsetA, double *A, int sda, int offsetB, double *B, int sdb) */
	/* { */

	if(m<=0)
		return;

	int n = m;

	int mna, ii;

	int offA = ai%bs;
	int offB = bi%bs;

	// A at the beginning of the block
	A -= offA;

	// A at the beginning of the block
	B -= offB;

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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_0_lib4(1, ii, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_0_lib4(1, ii, alpha, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A, B);
			}
		}
	// skip one element of A
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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_3_lib4(1, ii, alpha, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_2_lib4(1, ii, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_1_lib4(1, ii, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_1_lib4(1, ii, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+1, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+1, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A+1, B);
			}
		}
	// skip 2 elements of A
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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_3_lib4(1, ii, alpha, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_3_lib4(1, ii, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_2_lib4(1, ii, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_2_lib4(1, ii, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+2, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+2, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_2_lib4(1, ii, alpha, A, sda, B);
			}
		}
	// skip 3 elements of A
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
					kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecpsc_2_0_lib4(1, ii, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecpsc_3_0_lib4(1, ii, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_HASWELL)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecpsc_8_3_lib4(1, ii, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecpsc_4_3_lib4(1, ii, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecpsc_1_0_lib4(1, ii, alpha, A+3, B);
			else if(m-ii==2)
				kernel_dgecpsc_2_3_lib4(1, ii, alpha, A, sda, B);
			else // if(m-ii==3)
				kernel_dgecpsc_3_3_lib4(1, ii, alpha, A, sda, B);
			}
		}

	/* } */
	return;
	}



// scale a generic strmat
void blasfeo_dgesc(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	blasfeo_dgecpsc(m, n, alpha, sA, ai, aj, sA, ai, aj);
	}



// scale a triangular strmat
void blasfeo_dtrsc_l(int m, double alpha, struct blasfeo_dmat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	blasfeo_dtrcpsc_l(m, alpha, sA, ai, aj, sA, ai, aj);
	}



// copy a strvec into a strvec
void blasfeo_dveccp(int m, struct blasfeo_dvec *sa, int ai, struct blasfeo_dvec *sc, int ci)
	{
	double *pa = sa->pa + ai;
	double *pc = sc->pa + ci;
#if 1
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
void blasfeo_dgead(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	double *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	dgead_lib(m, n, alpha, ai%bs, pA, sda, ci%bs, pC, sdc);
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
void blasfeo_dgetr(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	double *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	dgetr_lib(m, n, 1.0, ai%bs, pA, sda, ci%bs, pC, sdc); // TODO remove alpha !!!
	return;
	}



// copy and transpose a lower triangular strmat into an upper triangular strmat
void blasfeo_dtrtr_l(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	double *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	dtrtr_l_lib(m, 1.0, ai%bs, pA, sda, ci%bs, pC, sdc); // TODO remove alpha !!!
	return;
	}



// copy and transpose an upper triangular strmat into a lower triangular strmat
void blasfeo_dtrtr_u(int m, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	const int bs = 4;
	int sda = sA->cn;
	double *pA = sA->pA + ai/bs*bs*sda + ai%bs + aj*bs;
	int sdc = sC->cn;
	double *pC = sC->pA + ci/bs*bs*sdc + ci%bs + cj*bs;
	dtrtr_u_lib(m, 1.0, ai%bs, pA, sda, ci%bs, pC, sdc); // TODO remove alpha !!!
	return;
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
	const int bs = 4;
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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		x[jj+0] = alpha*pA[jj*sda+(jj+0)*bs+0];
		x[jj+1] = alpha*pA[jj*sda+(jj+1)*bs+1];
		x[jj+2] = alpha*pA[jj*sda+(jj+2)*bs+2];
		x[jj+3] = alpha*pA[jj*sda+(jj+3)*bs+3];
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

	const int bs = 4;
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
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pA[jj*sda+(jj+0)*bs+0] += alpha*x[jj+0];
		pA[jj*sda+(jj+1)*bs+1] += alpha*x[jj+1];
		pA[jj*sda+(jj+2)*bs+2] += alpha*x[jj+2];
		pA[jj*sda+(jj+3)*bs+3] += alpha*x[jj+3];
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

	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	const int bs = 4;
	double *x = sx->pa + xi;
	int sdd = sD->cn;
	double *pD = sD->pA + di/bs*bs*sdd + di%bs + dj*bs;
	drowad_libsp(kmax, idx, alpha, x, pD);
	return;
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

	double *xm = sxm->pa + xim;
	double *x  = sx->pa + xi;
	double *xp = sxp->pa + xip;
	double *z  = sz->pa + zi;

	int ii;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	double d0;

	__m256d
		xm0, x0, xp0, z0, tmp0, tmp1, ones, mones, mask1, mask2;

	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	mones = _mm256_set_pd( -1.0, -1.0, -1.0, -1.0 );
	mask1 = _mm256_set_pd( 3.5, 2.5, 1.5, 0.5 );

	for(ii=0; ii<m-3; ii+=4)
		{
		x0  = _mm256_loadu_pd( &x[ii] );
		xp0 = _mm256_loadu_pd( &xp[ii] );
		xm0 = _mm256_loadu_pd( &xm[ii] );
		tmp0 = _mm256_cmp_pd( xp0, x0, 0x2 );
		tmp1 = _mm256_cmp_pd( x0, xm0, 0x2 );
		z0 = _mm256_blendv_pd( x0, xp0, tmp0 );
		z0 = _mm256_blendv_pd( z0, xm0, tmp1 );
		_mm256_storeu_pd( &z[ii], z0 );
		}
	if(ii<m)
		{
		d0 = (double) m-ii;
		mask2 = _mm256_broadcast_sd( &d0 );
		mask2 = _mm256_sub_pd( mask1, mask2 );
		x0  = _mm256_loadu_pd( &x[ii] );
		xp0 = _mm256_loadu_pd( &xp[ii] );
		xm0 = _mm256_loadu_pd( &xm[ii] );
		tmp0 = _mm256_cmp_pd( xp0, x0, 0x2 );
		tmp1 = _mm256_cmp_pd( x0, xm0, 0x2 );
		z0 = _mm256_blendv_pd( x0, xp0, tmp0 );
		z0 = _mm256_blendv_pd( z0, xm0, tmp1 );
		_mm256_maskstore_pd( &z[ii], _mm256_castpd_si256( mask2 ), z0 );
		}
#else
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
#endif

	return;

	}



// clip strvec between two strvec, with mask
void blasfeo_dveccl_mask(int m, struct blasfeo_dvec *sxm, int xim, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sxp, int xip, struct blasfeo_dvec *sz, int zi, struct blasfeo_dvec *sm, int mi)
	{

	double *xm = sxm->pa + xim;
	double *x  = sx->pa + xi;
	double *xp = sxp->pa + xip;
	double *z  = sz->pa + zi;
	double *mask  = sm->pa + mi;

	int ii;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	double d0;

	__m256d
		xm0, x0, xp0, z0, mask0, tmp0, tmp1, ones, mones, mask1, mask2;

	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	mones = _mm256_set_pd( -1.0, -1.0, -1.0, -1.0 );
	mask1 = _mm256_set_pd( 3.5, 2.5, 1.5, 0.5 );

	for(ii=0; ii<m-3; ii+=4)
		{
		mask0 = _mm256_setzero_pd();
		x0  = _mm256_loadu_pd( &x[ii] );
		xp0 = _mm256_loadu_pd( &xp[ii] );
		xm0 = _mm256_loadu_pd( &xm[ii] );
		tmp0 = _mm256_cmp_pd( xp0, x0, 0x2 );
		tmp1 = _mm256_cmp_pd( x0, xm0, 0x2 );
		z0 = _mm256_blendv_pd( x0, xp0, tmp0 );
		z0 = _mm256_blendv_pd( z0, xm0, tmp1 );
		mask0 = _mm256_blendv_pd( mask0, ones, tmp0 );
		mask0 = _mm256_blendv_pd( mask0, mones, tmp1 );
		_mm256_storeu_pd( &z[ii], z0 );
		_mm256_storeu_pd( &mask[ii], mask0 );
		}
	if(ii<m)
		{
		d0 = (double) m-ii;
		mask2 = _mm256_broadcast_sd( &d0 );
		mask2 = _mm256_sub_pd( mask1, mask2 );
		mask0 = _mm256_setzero_pd();
		x0  = _mm256_loadu_pd( &x[ii] );
		xp0 = _mm256_loadu_pd( &xp[ii] );
		xm0 = _mm256_loadu_pd( &xm[ii] );
		tmp0 = _mm256_cmp_pd( xp0, x0, 0x2 );
		tmp1 = _mm256_cmp_pd( x0, xm0, 0x2 );
		z0 = _mm256_blendv_pd( x0, xp0, tmp0 );
		z0 = _mm256_blendv_pd( z0, xm0, tmp1 );
		mask0 = _mm256_blendv_pd( mask0, ones, tmp0 );
		mask0 = _mm256_blendv_pd( mask0, mones, tmp1 );
		_mm256_maskstore_pd( &z[ii], _mm256_castpd_si256( mask2 ), z0 );
		_mm256_maskstore_pd( &mask[ii], _mm256_castpd_si256( mask2 ), mask0 );
		}
#else
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
#endif

	return;

	}


// zero out strvec to strvec with mask
void blasfeo_dvecze(int m, struct blasfeo_dvec *sm, int mi, struct blasfeo_dvec *sv, int vi, struct blasfeo_dvec *se, int ei)
	{
	double *mask = sm->pa + mi;
	double *v = sv->pa + vi;
	double *e = se->pa + ei;

	int ii;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	double d0;

	__m256d
		mask0, mask1, mask2, mask3, fives, zeros, e0, v0;

	fives = _mm256_set_pd( 0.5, 0.5, 0.5, 0.5 );
	zeros = _mm256_setzero_pd();
	mask3 = _mm256_set_pd( 3.5, 2.5, 1.5, 0.5 );

	for(ii=0; ii<m-3; ii+=4)
		{
		v0 = _mm256_loadu_pd( &v[ii] );
		mask0 = _mm256_loadu_pd( &mask[ii] );
		mask1 = mask0;
		mask0 = _mm256_sub_pd( mask0, fives);
		mask1 = _mm256_add_pd( mask1, fives);
		mask0 = _mm256_xor_pd( mask0, mask1);
		e0 = _mm256_blendv_pd( zeros, v0, mask0 );
		_mm256_storeu_pd( &e[ii], e0 );
		}
	if(ii<m)
		{
		d0 = (double) m-ii;
		mask2 = _mm256_broadcast_sd( &d0 );
		mask2 = _mm256_sub_pd( mask3, mask2 );
		v0 = _mm256_loadu_pd( &v[ii] );
		mask0 = _mm256_loadu_pd( &mask[ii] );
		mask1 = mask0;
		mask0 = _mm256_sub_pd( mask0, fives);
		mask1 = _mm256_add_pd( mask1, fives);
		mask0 = _mm256_xor_pd( mask0, mask1);
		e0 = _mm256_blendv_pd( zeros, v0, mask0 );
		_mm256_maskstore_pd( &e[ii], _mm256_castpd_si256( mask2 ), e0 );
		}
#else
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
#endif

	}


// compute inf norm of vector
void blasfeo_dvecnrm_inf(int m, struct blasfeo_dvec *sx, int xi, double *ptr_norm)
	{
	int ii;
	double *x = sx->pa + xi;
	double norm = 0.0;
	int is_nan = 0;
	double tmp;
	for(ii=0; ii<m; ii++)
		{
#ifdef USE_C99_MATH
		norm = fmax(norm, fabs(x[ii]));
		is_nan |= x[ii]!=x[ii];
#else // no c99
		tmp = fabs(x[ii]);
		norm = tmp>norm ? tmp : norm;
		is_nan |= x[ii]!=x[ii];
#endif
		}
#ifdef NAN
	*ptr_norm = is_nan==0 ? norm : NAN;
#else
	*ptr_norm = is_nan==0 ? norm : 0.0/0.0;
#endif
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
