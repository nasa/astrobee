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

#include <blasfeo_d_kernel.h>


#if ! ( defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53) )
void kernel_dgetr_tn_4_lib(int kmax, double *A, int lda, double *C, int ldc)
{

	int ii;
	ii = 0;

	for(; ii<kmax-3; ii+=4)
		{
		C[0+ldc*0] = A[0+lda*0];
		C[1+ldc*0] = A[0+lda*1];
		C[2+ldc*0] = A[0+lda*2];
		C[3+ldc*0] = A[0+lda*3];

		C[0+ldc*1] = A[1+lda*0];
		C[1+ldc*1] = A[1+lda*1];
		C[2+ldc*1] = A[1+lda*2];
		C[3+ldc*1] = A[1+lda*3];

		C[0+ldc*2] = A[2+lda*0];
		C[1+ldc*2] = A[2+lda*1];
		C[2+ldc*2] = A[2+lda*2];
		C[3+ldc*2] = A[2+lda*3];

		C[0+ldc*3] = A[3+lda*0];
		C[1+ldc*3] = A[3+lda*1];
		C[2+ldc*3] = A[3+lda*2];
		C[3+ldc*3] = A[3+lda*3];

		A += 4;
		C += 4*ldc;
		}
	for(; ii<kmax; ii++)
		{
		C[0+ldc*0] = A[0+lda*0];
		C[1+ldc*0] = A[0+lda*1];
		C[2+ldc*0] = A[0+lda*2];
		C[3+ldc*0] = A[0+lda*3];

		A += 1;
		C += 1*ldc;
		}

	return;

	}
#endif



void kernel_dgetr_tn_4_vs_lib(int kmax, double *A, int lda, double *C, int ldc, int m1)
{

	if(m1<=0)
		return;

	int ii;
	ii = 0;

	if(m1>=4)
		{
		kernel_dgetr_tn_4_lib(kmax, A, lda, C, ldc);
		return;
		}
	else if(m1==1)
		{
		goto l1;
		}
	else if(m1==2)
		{
		goto l2;
		}
	else //if(m1==3)
		{
		goto l3;
		}
	return;

l1:
	ii = 0;
	for(; ii<kmax; ii++)
		{
		C[0+ldc*0] = A[0+lda*0];

		A += 1;
		C += 1*ldc;
		}
	return;

l2:
	ii = 0;
	for(; ii<kmax; ii++)
		{
		C[0+ldc*0] = A[0+lda*0];
		C[1+ldc*0] = A[0+lda*1];

		A += 1;
		C += 1*ldc;
		}
	return;

l3:
	ii = 0;
	for(; ii<kmax; ii++)
		{
		C[0+ldc*0] = A[0+lda*0];
		C[1+ldc*0] = A[0+lda*1];
		C[2+ldc*0] = A[0+lda*2];

		A += 1;
		C += 1*ldc;
		}
	return;

	}




