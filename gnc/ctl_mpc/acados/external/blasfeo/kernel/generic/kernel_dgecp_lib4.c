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
 * Copy only
 */





// both A and B are aligned to 256-bit boundaries
void kernel_dgecp_4_0_lib4(int tri, int kmax, double *A, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];
		B[2+bs*0] = A[2+bs*0];
		B[3+bs*0] = A[3+bs*0];

		B[0+bs*1] = A[0+bs*1];
		B[1+bs*1] = A[1+bs*1];
		B[2+bs*1] = A[2+bs*1];
		B[3+bs*1] = A[3+bs*1];

		B[0+bs*2] = A[0+bs*2];
		B[1+bs*2] = A[1+bs*2];
		B[2+bs*2] = A[2+bs*2];
		B[3+bs*2] = A[3+bs*2];

		B[0+bs*3] = A[0+bs*3];
		B[1+bs*3] = A[1+bs*3];
		B[2+bs*3] = A[2+bs*3];
		B[3+bs*3] = A[3+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];
		B[2+bs*0] = A[2+bs*0];
		B[3+bs*0] = A[3+bs*0];

		A += 4;
		B += 4;

		}
	
	if(tri==1)
		{
		// 3x3 triangle

		B[1+bs*0] = A[1+bs*0];
		B[2+bs*0] = A[2+bs*0];
		B[3+bs*0] = A[3+bs*0];

		B[2+bs*1] = A[2+bs*1];
		B[3+bs*1] = A[3+bs*1];

		B[3+bs*2] = A[3+bs*2];

		}

	}






/*
 * Copy and Scale
 *
 * Used by: dge dtr
 */





// both.o A and B are aligned to 256-bit boundaries
void kernel_dgecpsc_4_0_lib4(int tri, int kmax, double alpha, double *A, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] = alpha*A[0+bs*0];
		B[1+bs*0] = alpha*A[1+bs*0];
		B[2+bs*0] = alpha*A[2+bs*0];
		B[3+bs*0] = alpha*A[3+bs*0];

		B[0+bs*1] = alpha*A[0+bs*1];
		B[1+bs*1] = alpha*A[1+bs*1];
		B[2+bs*1] = alpha*A[2+bs*1];
		B[3+bs*1] = alpha*A[3+bs*1];

		B[0+bs*2] = alpha*A[0+bs*2];
		B[1+bs*2] = alpha*A[1+bs*2];
		B[2+bs*2] = alpha*A[2+bs*2];
		B[3+bs*2] = alpha*A[3+bs*2];

		B[0+bs*3] = alpha*A[0+bs*3];
		B[1+bs*3] = alpha*A[1+bs*3];
		B[2+bs*3] = alpha*A[2+bs*3];
		B[3+bs*3] = alpha*A[3+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A[0+bs*0];
		B[1+bs*0] = alpha*A[1+bs*0];
		B[2+bs*0] = alpha*A[2+bs*0];
		B[3+bs*0] = alpha*A[3+bs*0];

		A += 4;
		B += 4;

		}
	
	if(tri==1)
		{
		// 3x3 triangle

		B[1+bs*0] = alpha*A[1+bs*0];
		B[2+bs*0] = alpha*A[2+bs*0];
		B[3+bs*0] = alpha*A[3+bs*0];

		B[2+bs*1] = alpha*A[2+bs*1];
		B[3+bs*1] = alpha*A[3+bs*1];

		B[3+bs*2] = alpha*A[3+bs*2];

		}

	}



// both A and B are aligned to 256-bit boundaries, 1 element of A must be skipped
void kernel_dgecpsc_4_1_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] = alpha*A0[1+bs*0];
		B[1+bs*0] = alpha*A0[2+bs*0];
		B[2+bs*0] = alpha*A0[3+bs*0];
		B[3+bs*0] = alpha*A1[0+bs*0];

		B[0+bs*1] = alpha*A0[1+bs*1];
		B[1+bs*1] = alpha*A0[2+bs*1];
		B[2+bs*1] = alpha*A0[3+bs*1];
		B[3+bs*1] = alpha*A1[0+bs*1];

		B[0+bs*2] = alpha*A0[1+bs*2];
		B[1+bs*2] = alpha*A0[2+bs*2];
		B[2+bs*2] = alpha*A0[3+bs*2];
		B[3+bs*2] = alpha*A1[0+bs*2];

		B[0+bs*3] = alpha*A0[1+bs*3];
		B[1+bs*3] = alpha*A0[2+bs*3];
		B[2+bs*3] = alpha*A0[3+bs*3];
		B[3+bs*3] = alpha*A1[0+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A0[1+bs*0];
		B[1+bs*0] = alpha*A0[2+bs*0];
		B[2+bs*0] = alpha*A0[3+bs*0];
		B[3+bs*0] = alpha*A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}
	
	if(tri==1)
		{
		// 3x3 triangle

		B[1+0*bs] = alpha*A0[2+0*bs];
		B[2+0*bs] = alpha*A0[3+0*bs];
		B[3+0*bs] = alpha*A1[0+0*bs];

		B[2+1*bs] = alpha*A0[3+1*bs];
		B[3+1*bs] = alpha*A1[0+1*bs];

		B[3+2*bs] = alpha*A1[0+2*bs];

		}

	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_dgecpsc_4_2_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] = alpha*A0[2+bs*0];
		B[1+bs*0] = alpha*A0[3+bs*0];
		B[2+bs*0] = alpha*A1[0+bs*0];
		B[3+bs*0] = alpha*A1[1+bs*0];

		B[0+bs*1] = alpha*A0[2+bs*1];
		B[1+bs*1] = alpha*A0[3+bs*1];
		B[2+bs*1] = alpha*A1[0+bs*1];
		B[3+bs*1] = alpha*A1[1+bs*1];

		B[0+bs*2] = alpha*A0[2+bs*2];
		B[1+bs*2] = alpha*A0[3+bs*2];
		B[2+bs*2] = alpha*A1[0+bs*2];
		B[3+bs*2] = alpha*A1[1+bs*2];

		B[0+bs*3] = alpha*A0[2+bs*3];
		B[1+bs*3] = alpha*A0[3+bs*3];
		B[2+bs*3] = alpha*A1[0+bs*3];
		B[3+bs*3] = alpha*A1[1+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A0[2+bs*0];
		B[1+bs*0] = alpha*A0[3+bs*0];
		B[2+bs*0] = alpha*A1[0+bs*0];
		B[3+bs*0] = alpha*A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}
	
	if(tri==1)
		{
		// 3x3 triangle}

		B[1+bs*0] = alpha*A0[3+bs*0];
		B[2+bs*0] = alpha*A1[0+bs*0];
		B[3+bs*0] = alpha*A1[1+bs*0];

		B[2+bs*1] = alpha*A1[0+bs*1];
		B[3+bs*1] = alpha*A1[1+bs*1];

		B[3+bs*2] = alpha*A1[1+bs*2];

		}

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_dgecpsc_4_3_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] = alpha*A0[3+bs*0];
		B[1+bs*0] = alpha*A1[0+bs*0];
		B[2+bs*0] = alpha*A1[1+bs*0];
		B[3+bs*0] = alpha*A1[2+bs*0];

		B[0+bs*1] = alpha*A0[3+bs*1];
		B[1+bs*1] = alpha*A1[0+bs*1];
		B[2+bs*1] = alpha*A1[1+bs*1];
		B[3+bs*1] = alpha*A1[2+bs*1];

		B[0+bs*2] = alpha*A0[3+bs*2];
		B[1+bs*2] = alpha*A1[0+bs*2];
		B[2+bs*2] = alpha*A1[1+bs*2];
		B[3+bs*2] = alpha*A1[2+bs*2];

		B[0+bs*3] = alpha*A0[3+bs*3];
		B[1+bs*3] = alpha*A1[0+bs*3];
		B[2+bs*3] = alpha*A1[1+bs*3];
		B[3+bs*3] = alpha*A1[2+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A0[3+bs*0];
		B[1+bs*0] = alpha*A1[0+bs*0];
		B[2+bs*0] = alpha*A1[1+bs*0];
		B[3+bs*0] = alpha*A1[2+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}
	
	if(tri==1)
		{
		// 3x3 triangle

		B[1+bs*0] = alpha*A1[0+bs*0];
		B[2+bs*0] = alpha*A1[1+bs*0];
		B[3+bs*0] = alpha*A1[2+bs*0];

		B[2+bs*1] = alpha*A1[1+bs*1];
		B[3+bs*1] = alpha*A1[2+bs*1];

		B[3+bs*2] = alpha*A1[2+bs*2];

		}

	}



// both A and B are aligned to 64-bit boundaries
void kernel_dgecpsc_3_0_lib4(int tri, int kmax, double alpha, double *A, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 3-wide + end 2x2 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] = alpha*A[0+bs*0];
		B[1+bs*0] = alpha*A[1+bs*0];
		B[2+bs*0] = alpha*A[2+bs*0];

		B[0+bs*1] = alpha*A[0+bs*1];
		B[1+bs*1] = alpha*A[1+bs*1];
		B[2+bs*1] = alpha*A[2+bs*1];

		B[0+bs*2] = alpha*A[0+bs*2];
		B[1+bs*2] = alpha*A[1+bs*2];
		B[2+bs*2] = alpha*A[2+bs*2];

		B[0+bs*3] = alpha*A[0+bs*3];
		B[1+bs*3] = alpha*A[1+bs*3];
		B[2+bs*3] = alpha*A[2+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A[0+bs*0];
		B[1+bs*0] = alpha*A[1+bs*0];
		B[2+bs*0] = alpha*A[2+bs*0];

		A += 4;
		B += 4;

		}
	
	if(tri==1)
		{
		// 2x2 triangle

		B[1+bs*0] = alpha*A[1+bs*0];
		B[2+bs*0] = alpha*A[2+bs*0];

		B[2+bs*1] = alpha*A[2+bs*1];

		}

	}


// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_dgecpsc_3_2_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 3-wide + end 2x2 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] = alpha*A0[2+bs*0];
		B[1+bs*0] = alpha*A0[3+bs*0];
		B[2+bs*0] = alpha*A1[0+bs*0];

		B[0+bs*1] = alpha*A0[2+bs*1];
		B[1+bs*1] = alpha*A0[3+bs*1];
		B[2+bs*1] = alpha*A1[0+bs*1];

		B[0+bs*2] = alpha*A0[2+bs*2];
		B[1+bs*2] = alpha*A0[3+bs*2];
		B[2+bs*2] = alpha*A1[0+bs*2];

		B[0+bs*3] = alpha*A0[2+bs*3];
		B[1+bs*3] = alpha*A0[3+bs*3];
		B[2+bs*3] = alpha*A1[0+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A0[2+bs*0];
		B[1+bs*0] = alpha*A0[3+bs*0];
		B[2+bs*0] = alpha*A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}
	
	if(tri==1)
		{
		// 2x2 triangle

		B[1+bs*0] = alpha*A0[3+bs*0];
		B[2+bs*0] = alpha*A1[0+bs*0];

		B[2+bs*1] = alpha*A1[0+bs*1];

		}

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_dgecpsc_3_3_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 3-wide + end 2x2 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] = alpha*A0[3+bs*0];
		B[1+bs*0] = alpha*A1[0+bs*0];
		B[2+bs*0] = alpha*A1[1+bs*0];

		B[0+bs*1] = alpha*A0[3+bs*1];
		B[1+bs*1] = alpha*A1[0+bs*1];
		B[2+bs*1] = alpha*A1[1+bs*1];

		B[0+bs*2] = alpha*A0[3+bs*2];
		B[1+bs*2] = alpha*A1[0+bs*2];
		B[2+bs*2] = alpha*A1[1+bs*2];

		B[0+bs*3] = alpha*A0[3+bs*3];
		B[1+bs*3] = alpha*A1[0+bs*3];
		B[2+bs*3] = alpha*A1[1+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A0[3+bs*0];
		B[1+bs*0] = alpha*A1[0+bs*0];
		B[2+bs*0] = alpha*A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}
	
	if(tri==1)
		{
		// 2x2 triangle

		B[1+bs*0] = alpha*A1[0+bs*0];
		B[2+bs*0] = alpha*A1[1+bs*0];

		B[2+bs*1] = alpha*A1[1+bs*1];

		}

	}



// both A and B are aligned to 64-bit boundaries
void kernel_dgecpsc_2_0_lib4(int tri, int kmax, double alpha, double *A, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 2-wide + end 1x1 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] = alpha*A[0+bs*0];
		B[1+bs*0] = alpha*A[1+bs*0];

		B[0+bs*1] = alpha*A[0+bs*1];
		B[1+bs*1] = alpha*A[1+bs*1];

		B[0+bs*2] = alpha*A[0+bs*2];
		B[1+bs*2] = alpha*A[1+bs*2];

		B[0+bs*3] = alpha*A[0+bs*3];
		B[1+bs*3] = alpha*A[1+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A[0+bs*0];
		B[1+bs*0] = alpha*A[1+bs*0];

		A += 4;
		B += 4;

		}
	
	if(tri==1)
		{
		// 1x1 triangle

		B[1+bs*0] = alpha*A[1+bs*0];

		}

	}



// both A and B are aligned to 128-bit boundaries, 3 elements of A must be skipped
void kernel_dgecpsc_2_3_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 2-wide + end 1x1 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] = alpha*A0[3+bs*0];
		B[1+bs*0] = alpha*A1[0+bs*0];

		B[0+bs*1] = alpha*A0[3+bs*1];
		B[1+bs*1] = alpha*A1[0+bs*1];

		B[0+bs*2] = alpha*A0[3+bs*2];
		B[1+bs*2] = alpha*A1[0+bs*2];

		B[0+bs*3] = alpha*A0[3+bs*3];
		B[1+bs*3] = alpha*A1[0+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A0[3+bs*0];
		B[1+bs*0] = alpha*A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}
	
	if(tri==1)
		{
		// 1x1 triangle

		B[1+bs*0] = alpha*A1[0+bs*0];

		}

	}



// both A and B are aligned 64-bit boundaries
void kernel_dgecpsc_1_0_lib4(int tri, int kmax, double alpha, double *A, double *B)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 1-wide

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] = alpha*A[0+bs*0];
		B[0+bs*1] = alpha*A[0+bs*1];
		B[0+bs*2] = alpha*A[0+bs*2];
		B[0+bs*3] = alpha*A[0+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] = alpha*A[0+bs*0];

		A += 4;
		B += 4;

		}

	}





/*
 * Add and scale
 *
 */




// both A and B are aligned to 256-bit boundaries
void kernel_dgead_4_0_lib4(int kmax, double alpha, double *A, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];
		B[2+bs*0] += alpha * A[2+bs*0];
		B[3+bs*0] += alpha * A[3+bs*0];

		B[0+bs*1] += alpha * A[0+bs*1];
		B[1+bs*1] += alpha * A[1+bs*1];
		B[2+bs*1] += alpha * A[2+bs*1];
		B[3+bs*1] += alpha * A[3+bs*1];

		B[0+bs*2] += alpha * A[0+bs*2];
		B[1+bs*2] += alpha * A[1+bs*2];
		B[2+bs*2] += alpha * A[2+bs*2];
		B[3+bs*2] += alpha * A[3+bs*2];

		B[0+bs*3] += alpha * A[0+bs*3];
		B[1+bs*3] += alpha * A[1+bs*3];
		B[2+bs*3] += alpha * A[2+bs*3];
		B[3+bs*3] += alpha * A[3+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];
		B[2+bs*0] += alpha * A[2+bs*0];
		B[3+bs*0] += alpha * A[3+bs*0];

		A += 4;
		B += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 1 element of A must be skipped
void kernel_dgead_4_1_lib4(int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] += alpha * A0[1+bs*0];
		B[1+bs*0] += alpha * A0[2+bs*0];
		B[2+bs*0] += alpha * A0[3+bs*0];
		B[3+bs*0] += alpha * A1[0+bs*0];

		B[0+bs*1] += alpha * A0[1+bs*1];
		B[1+bs*1] += alpha * A0[2+bs*1];
		B[2+bs*1] += alpha * A0[3+bs*1];
		B[3+bs*1] += alpha * A1[0+bs*1];

		B[0+bs*2] += alpha * A0[1+bs*2];
		B[1+bs*2] += alpha * A0[2+bs*2];
		B[2+bs*2] += alpha * A0[3+bs*2];
		B[3+bs*2] += alpha * A1[0+bs*2];

		B[0+bs*3] += alpha * A0[1+bs*3];
		B[1+bs*3] += alpha * A0[2+bs*3];
		B[2+bs*3] += alpha * A0[3+bs*3];
		B[3+bs*3] += alpha * A1[0+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A0[1+bs*0];
		B[1+bs*0] += alpha * A0[2+bs*0];
		B[2+bs*0] += alpha * A0[3+bs*0];
		B[3+bs*0] += alpha * A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_dgead_4_2_lib4(int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] += alpha * A0[2+bs*0];
		B[1+bs*0] += alpha * A0[3+bs*0];
		B[2+bs*0] += alpha * A1[0+bs*0];
		B[3+bs*0] += alpha * A1[1+bs*0];

		B[0+bs*1] += alpha * A0[2+bs*1];
		B[1+bs*1] += alpha * A0[3+bs*1];
		B[2+bs*1] += alpha * A1[0+bs*1];
		B[3+bs*1] += alpha * A1[1+bs*1];

		B[0+bs*2] += alpha * A0[2+bs*2];
		B[1+bs*2] += alpha * A0[3+bs*2];
		B[2+bs*2] += alpha * A1[0+bs*2];
		B[3+bs*2] += alpha * A1[1+bs*2];

		B[0+bs*3] += alpha * A0[2+bs*3];
		B[1+bs*3] += alpha * A0[3+bs*3];
		B[2+bs*3] += alpha * A1[0+bs*3];
		B[3+bs*3] += alpha * A1[1+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A0[2+bs*0];
		B[1+bs*0] += alpha * A0[3+bs*0];
		B[2+bs*0] += alpha * A1[0+bs*0];
		B[3+bs*0] += alpha * A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_dgead_4_3_lib4(int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] += alpha * A0[3+bs*0];
		B[1+bs*0] += alpha * A1[0+bs*0];
		B[2+bs*0] += alpha * A1[1+bs*0];
		B[3+bs*0] += alpha * A1[2+bs*0];

		B[0+bs*1] += alpha * A0[3+bs*1];
		B[1+bs*1] += alpha * A1[0+bs*1];
		B[2+bs*1] += alpha * A1[1+bs*1];
		B[3+bs*1] += alpha * A1[2+bs*1];

		B[0+bs*2] += alpha * A0[3+bs*2];
		B[1+bs*2] += alpha * A1[0+bs*2];
		B[2+bs*2] += alpha * A1[1+bs*2];
		B[3+bs*2] += alpha * A1[2+bs*2];

		B[0+bs*3] += alpha * A0[3+bs*3];
		B[1+bs*3] += alpha * A1[0+bs*3];
		B[2+bs*3] += alpha * A1[1+bs*3];
		B[3+bs*3] += alpha * A1[2+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A0[3+bs*0];
		B[1+bs*0] += alpha * A1[0+bs*0];
		B[2+bs*0] += alpha * A1[1+bs*0];
		B[3+bs*0] += alpha * A1[2+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}



// both A and B are aligned to 64-bit boundaries
void kernel_dgead_3_0_lib4(int kmax, double alpha, double *A, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];
		B[2+bs*0] += alpha * A[2+bs*0];

		B[0+bs*1] += alpha * A[0+bs*1];
		B[1+bs*1] += alpha * A[1+bs*1];
		B[2+bs*1] += alpha * A[2+bs*1];

		B[0+bs*2] += alpha * A[0+bs*2];
		B[1+bs*2] += alpha * A[1+bs*2];
		B[2+bs*2] += alpha * A[2+bs*2];

		B[0+bs*3] += alpha * A[0+bs*3];
		B[1+bs*3] += alpha * A[1+bs*3];
		B[2+bs*3] += alpha * A[2+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];
		B[2+bs*0] += alpha * A[2+bs*0];

		A += 4;
		B += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_dgead_3_2_lib4(int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] += alpha * A0[2+bs*0];
		B[1+bs*0] += alpha * A0[3+bs*0];
		B[2+bs*0] += alpha * A1[0+bs*0];

		B[0+bs*1] += alpha * A0[2+bs*1];
		B[1+bs*1] += alpha * A0[3+bs*1];
		B[2+bs*1] += alpha * A1[0+bs*1];

		B[0+bs*2] += alpha * A0[2+bs*2];
		B[1+bs*2] += alpha * A0[3+bs*2];
		B[2+bs*2] += alpha * A1[0+bs*2];

		B[0+bs*3] += alpha * A0[2+bs*3];
		B[1+bs*3] += alpha * A0[3+bs*3];
		B[2+bs*3] += alpha * A1[0+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A0[2+bs*0];
		B[1+bs*0] += alpha * A0[3+bs*0];
		B[2+bs*0] += alpha * A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_dgead_3_3_lib4(int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] += alpha * A0[3+bs*0];
		B[1+bs*0] += alpha * A1[0+bs*0];
		B[2+bs*0] += alpha * A1[1+bs*0];

		B[0+bs*1] += alpha * A0[3+bs*1];
		B[1+bs*1] += alpha * A1[0+bs*1];
		B[2+bs*1] += alpha * A1[1+bs*1];

		B[0+bs*2] += alpha * A0[3+bs*2];
		B[1+bs*2] += alpha * A1[0+bs*2];
		B[2+bs*2] += alpha * A1[1+bs*2];

		B[0+bs*3] += alpha * A0[3+bs*3];
		B[1+bs*3] += alpha * A1[0+bs*3];
		B[2+bs*3] += alpha * A1[1+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A0[3+bs*0];
		B[1+bs*0] += alpha * A1[0+bs*0];
		B[2+bs*0] += alpha * A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}



// both A and B are aligned to 64-bit boundaries
void kernel_dgead_2_0_lib4(int kmax, double alpha, double *A, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];

		B[0+bs*1] += alpha * A[0+bs*1];
		B[1+bs*1] += alpha * A[1+bs*1];

		B[0+bs*2] += alpha * A[0+bs*2];
		B[1+bs*2] += alpha * A[1+bs*2];

		B[0+bs*3] += alpha * A[0+bs*3];
		B[1+bs*3] += alpha * A[1+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];

		A += 4;
		B += 4;

		}

	}



// both A and B are aligned to 128-bit boundaries, 3 elements of A must be skipped
void kernel_dgead_2_3_lib4(int kmax, double alpha, double *A0, int sda, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		B[0+bs*0] += alpha * A0[3+bs*0];
		B[1+bs*0] += alpha * A1[0+bs*0];

		B[0+bs*1] += alpha * A0[3+bs*1];
		B[1+bs*1] += alpha * A1[0+bs*1];

		B[0+bs*2] += alpha * A0[3+bs*2];
		B[1+bs*2] += alpha * A1[0+bs*2];

		B[0+bs*3] += alpha * A0[3+bs*3];
		B[1+bs*3] += alpha * A1[0+bs*3];

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A0[3+bs*0];
		B[1+bs*0] += alpha * A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}



// both A and B are aligned 64-bit boundaries
void kernel_dgead_1_0_lib4(int kmax, double alpha, double *A, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{
		B[0+bs*0] += alpha * A[0+bs*0];

		B[0+bs*1] += alpha * A[0+bs*1];

		B[0+bs*2] += alpha * A[0+bs*2];

		B[0+bs*3] += alpha * A[0+bs*3];

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A[0+bs*0];

		A += 4;
		B += 4;

		}

	}
