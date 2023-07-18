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



// transposed of general matrices, read along panels, write across panels
void kernel_sgetr_4_lib4(int tri, int kmax, int kna, float alpha, float *A, float *C, int sdc)
	{

	if(tri==1)
		{
		// A is lower triangular, C is upper triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	const int bs = 4;
	
	int k;

	k = 0;

	if(kmax<kna)
		goto cleanup_loop;

	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			C[0+bs*0] = alpha * A[0+bs*0];
			C[0+bs*1] = alpha * A[1+bs*0];
			C[0+bs*2] = alpha * A[2+bs*0];
			C[0+bs*3] = alpha * A[3+bs*0];

			C += 1;
			A += bs;
			}
		C += bs*(sdc-1);
		}
	
	for( ; k<kmax-3; k+=4)
		{
		C[0+bs*0] = alpha * A[0+bs*0];
		C[0+bs*1] = alpha * A[1+bs*0];
		C[0+bs*2] = alpha * A[2+bs*0];
		C[0+bs*3] = alpha * A[3+bs*0];

		C[1+bs*0] = alpha * A[0+bs*1];
		C[1+bs*1] = alpha * A[1+bs*1];
		C[1+bs*2] = alpha * A[2+bs*1];
		C[1+bs*3] = alpha * A[3+bs*1];

		C[2+bs*0] = alpha * A[0+bs*2];
		C[2+bs*1] = alpha * A[1+bs*2];
		C[2+bs*2] = alpha * A[2+bs*2];
		C[2+bs*3] = alpha * A[3+bs*2];

		C[3+bs*0] = alpha * A[0+bs*3];
		C[3+bs*1] = alpha * A[1+bs*3];
		C[3+bs*2] = alpha * A[2+bs*3];
		C[3+bs*3] = alpha * A[3+bs*3];

		C += bs*sdc;
		A += bs*bs;
		}

	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = alpha * A[0+bs*0];
		C[0+bs*1] = alpha * A[1+bs*0];
		C[0+bs*2] = alpha * A[2+bs*0];
		C[0+bs*3] = alpha * A[3+bs*0];

		C += 1;
		A += bs;
		}

	if(tri==1)
		{
		// end 3x3 triangle
		kna = (bs-(bs-kna+kmax)%bs)%bs;

		if(kna==1)
			{
			C[0+bs*1] = alpha * A[1+bs*0];
			C[0+bs*2] = alpha * A[2+bs*0];
			C[0+bs*3] = alpha * A[3+bs*0];
			C[1+bs*(sdc+1)] = alpha * A[2+bs*1];
			C[1+bs*(sdc+2)] = alpha * A[3+bs*1];
			C[2+bs*(sdc+2)] = alpha * A[3+bs*2];
			}
		else if(kna==2)
			{
			C[0+bs*1] = alpha * A[1+bs*0];
			C[0+bs*2] = alpha * A[2+bs*0];
			C[0+bs*3] = alpha * A[3+bs*0];
			C[1+bs*2] = alpha * A[2+bs*1];
			C[1+bs*3] = alpha * A[3+bs*1];
			C[2+bs*(sdc+2)] = alpha * A[3+bs*2];
			}
		else
			{
			C[0+bs*1] = alpha * A[1+bs*0];
			C[0+bs*2] = alpha * A[2+bs*0];
			C[0+bs*3] = alpha * A[3+bs*0];
			C[1+bs*2] = alpha * A[2+bs*1];
			C[1+bs*3] = alpha * A[3+bs*1];
			C[2+bs*3] = alpha * A[3+bs*2];
			}
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_sgetr_3_lib4(int tri, int kmax, int kna, float alpha, float *A, float *C, int sdc)
	{

	if(tri==1)
		{
		// A is lower triangular, C is upper triangular
		// kmax+1 3-wide + end 2x2 triangle

		kmax += 1;
		}

	const int bs = 4;
	
	int k;

	k = 0;

	if(kmax<kna)
		goto cleanup_loop;

	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			C[0+bs*0] = alpha * A[0+bs*0];
			C[0+bs*1] = alpha * A[1+bs*0];
			C[0+bs*2] = alpha * A[2+bs*0];

			C += 1;
			A += bs;
			}
		C += bs*(sdc-1);
		}
	
	for( ; k<kmax-3; k+=4)
		{
		C[0+bs*0] = alpha * A[0+bs*0];
		C[0+bs*1] = alpha * A[1+bs*0];
		C[0+bs*2] = alpha * A[2+bs*0];

		C[1+bs*0] = alpha * A[0+bs*1];
		C[1+bs*1] = alpha * A[1+bs*1];
		C[1+bs*2] = alpha * A[2+bs*1];

		C[2+bs*0] = alpha * A[0+bs*2];
		C[2+bs*1] = alpha * A[1+bs*2];
		C[2+bs*2] = alpha * A[2+bs*2];

		C[3+bs*0] = alpha * A[0+bs*3];
		C[3+bs*1] = alpha * A[1+bs*3];
		C[3+bs*2] = alpha * A[2+bs*3];

		C += bs*sdc;
		A += bs*bs;
		}
	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = alpha * A[0+bs*0];
		C[0+bs*1] = alpha * A[1+bs*0];
		C[0+bs*2] = alpha * A[2+bs*0];

		C += 1;
		A += bs;
		}

	if(tri==1)
		{
		// end 2x2 triangle
		kna = (bs-(bs-kna+kmax)%bs)%bs;

		if(kna==1)
			{
			C[0+bs*1] = alpha * A[1+bs*0];
			C[0+bs*2] = alpha * A[2+bs*0];
			C[1+bs*(sdc+1)] = alpha * A[2+bs*1];
			}
		else
			{
			C[0+bs*1] = alpha * A[1+bs*0];
			C[0+bs*2] = alpha * A[2+bs*0];
			C[1+bs*2] = alpha * A[2+bs*1];
			}
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_sgetr_2_lib4(int tri, int kmax, int kna, float alpha, float *A, float *C, int sdc)
	{

	if(tri==1)
		{
		// A is lower triangular, C is upper triangular
		// kmax+1 2-wide + end 1x1 triangle

		kmax += 1;
		}

	const int bs = 4;
	
	int k;

	k = 0;

	if(kmax<kna)
		goto cleanup_loop;

	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			C[0+bs*0] = alpha * A[0+bs*0];
			C[0+bs*1] = alpha * A[1+bs*0];

			C += 1;
			A += bs;
			}
		C += bs*(sdc-1);
		}
	
	for( ; k<kmax-3; k+=4)
		{
		C[0+bs*0] = alpha * A[0+bs*0];
		C[0+bs*1] = alpha * A[1+bs*0];

		C[1+bs*0] = alpha * A[0+bs*1];
		C[1+bs*1] = alpha * A[1+bs*1];

		C[2+bs*0] = alpha * A[0+bs*2];
		C[2+bs*1] = alpha * A[1+bs*2];

		C[3+bs*0] = alpha * A[0+bs*3];
		C[3+bs*1] = alpha * A[1+bs*3];

		C += bs*sdc;
		A += bs*bs;
		}
	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = alpha * A[0+bs*0];
		C[0+bs*1] = alpha * A[1+bs*0];

		C += 1;
		A += bs;
		}
	
	if(tri==1)
		{
		// end 1x1 triangle
		C[0+bs*1] = alpha * A[1+bs*0];
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_sgetr_1_lib4(int tri, int kmax, int kna, float alpha, float *A, float *C, int sdc)
	{

	if(tri==1)
		{
		// A is lower triangular, C is upper triangular
		// kmax+1 1-wide

		kmax += 1;
		}

	const int bs = 4;
	
	int k;

	k = 0;

	if(kmax<kna)
		goto cleanup_loop;

	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			C[0+bs*0] = alpha * A[0+bs*0];

			C += 1;
			A += bs;
			}
		C += bs*(sdc-1);
		}
	
	for( ; k<kmax-3; k+=4)
		{
		C[0+bs*0] = alpha * A[0+bs*0];

		C[1+bs*0] = alpha * A[0+bs*1];

		C[2+bs*0] = alpha * A[0+bs*2];

		C[3+bs*0] = alpha * A[0+bs*3];

		C += bs*sdc;
		A += bs*bs;
		}
	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = alpha * A[0+bs*0];

		C += 1;
		A += bs;
		}

	}




