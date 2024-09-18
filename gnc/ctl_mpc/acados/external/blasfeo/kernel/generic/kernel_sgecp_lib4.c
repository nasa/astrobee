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


// ---- ge

// 4

// both A and B are aligned to 256-bit boundaries
void kernel_sgecpsc_4_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;
	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A[0+bs*0];
		B[1+bs*0] = alpha * A[1+bs*0];
		B[2+bs*0] = alpha * A[2+bs*0];
		B[3+bs*0] = alpha * A[3+bs*0];

		A += 4;
		B += 4;

		}

	}

void kernel_sgecp_4_0_lib4(int kmax, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];
		B[2+bs*0] = A[2+bs*0];
		B[3+bs*0] = A[3+bs*0];

		A += 4;
		B += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 1 element of A must be skipped
void kernel_sgecpsc_4_1_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;
	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A0[1+bs*0];
		B[1+bs*0] = alpha * A0[2+bs*0];
		B[2+bs*0] = alpha * A0[3+bs*0];

		B[3+bs*0] = alpha * A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 1 element of A must be skipped
void kernel_sgecp_4_1_lib4(int kmax, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[1+bs*0];
		B[1+bs*0] = A0[2+bs*0];
		B[2+bs*0] = A0[3+bs*0];

		B[3+bs*0] = A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 2 element of A must be skipped
void kernel_sgecpsc_4_2_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;
	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A0[2+bs*0];
		B[1+bs*0] = alpha * A0[3+bs*0];

		B[2+bs*0] = alpha * A1[0+bs*0];
		B[3+bs*0] = alpha * A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 2 element of A must be skipped
void kernel_sgecp_4_2_lib4(int kmax, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[2+bs*0];
		B[1+bs*0] = A0[3+bs*0];

		B[2+bs*0] = A1[0+bs*0];
		B[3+bs*0] = A1[1+bs*0];


		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 3 element of A must be skipped
void kernel_sgecpsc_4_3_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;
	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A0[3+bs*0];

		B[1+bs*0] = alpha * A1[0+bs*0];
		B[2+bs*0] = alpha * A1[1+bs*0];
		B[3+bs*0] = alpha * A1[2+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 3 element of A must be skipped
void kernel_sgecp_4_3_lib4(int kmax, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[3+bs*0];

		B[1+bs*0] = A1[0+bs*0];
		B[2+bs*0] = A1[1+bs*0];
		B[3+bs*0] = A1[2+bs*0];


		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// 3

void kernel_sgecpsc_3_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A[0+bs*0];
		B[1+bs*0] = alpha * A[1+bs*0];
		B[2+bs*0] = alpha * A[2+bs*0];

		A += 4;
		B += 4;

		}

	}

void kernel_sgecp_3_0_lib4(int kmax, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];
		B[2+bs*0] = A[2+bs*0];

		A += 4;
		B += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_sgecpsc_3_2_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;
	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A0[2+bs*0];
		B[1+bs*0] = alpha * A0[3+bs*0];

		B[2+bs*0] = alpha * A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_sgecp_3_2_lib4(int kmax, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[2+bs*0];

		B[1+bs*0] = A0[3+bs*0];
		B[2+bs*0] = A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_sgecpsc_3_3_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;
	float alpha = *alphap;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A0[3+bs*0];

		B[1+bs*0] = alpha * A1[0+bs*0];
		B[2+bs*0] = alpha * A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_sgecp_3_3_lib4(int kmax, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[3+bs*0];

		B[1+bs*0] = A1[0+bs*0];
		B[2+bs*0] = A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// 2

void kernel_sgecpsc_2_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;
	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A[0+bs*0];
		B[1+bs*0] = alpha * A[1+bs*0];

		A += 4;
		B += 4;

		}

	}

void kernel_sgecp_2_0_lib4(int kmax, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];

		A += 4;
		B += 4;

		}

	}

// both A and B are aligned to 128-bit boundaries, 3 elements of A must be skipped
void kernel_sgecpsc_2_3_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;
	float alpha = alphap[0];
	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A0[3+bs*0];
		B[1+bs*0] = alpha * A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// both A and B are aligned to 128-bit boundaries, 3 elements of A must be skipped
void kernel_sgecp_2_3_lib4(int kmax, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[3+bs*0];
		B[1+bs*0] = A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}

// 1

void kernel_sgecpsc_1_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = alpha * A[0+bs*0];

		A += 4;
		B += 4;

		}

	}

void kernel_sgecp_1_0_lib4(int kmax, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];

		A += 4;
		B += 4;

		}

	}



// ---- tr

// both A and B are aligned to 256-bit boundaries
void kernel_strcp_l_4_0_lib4(int kmax, float *A, float *B)
	{

	// A and C are lower triangular
	// kmax+1 4-wide + end 3x3 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];
		B[2+bs*0] = A[2+bs*0];
		B[3+bs*0] = A[3+bs*0];

		A += 4;
		B += 4;

		}

	// 3x3 triangle

	B[1+bs*0] = A[1+bs*0];
	B[2+bs*0] = A[2+bs*0];
	B[3+bs*0] = A[3+bs*0];

	B[2+bs*1] = A[2+bs*1];
	B[3+bs*1] = A[3+bs*1];

	B[3+bs*2] = A[3+bs*2];

	}



// both A and B are aligned to 256-bit boundaries, 1 element of A must be skipped
void kernel_strcp_l_4_1_lib4(int kmax, float *A0, int sda, float *B)
	{

	// A and C are lower triangular
	// kmax+1 4-wide + end 3x3 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[1+bs*0];
		B[1+bs*0] = A0[2+bs*0];
		B[2+bs*0] = A0[3+bs*0];
		B[3+bs*0] = A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	// 3x3 triangle

	B[1+0*bs] = A0[2+0*bs];
	B[2+0*bs] = A0[3+0*bs];
	B[3+0*bs] = A1[0+0*bs];

	B[2+1*bs] = A0[3+1*bs];
	B[3+1*bs] = A1[0+1*bs];

	B[3+2*bs] = A1[0+2*bs];

	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_strcp_l_4_2_lib4(int kmax, float *A0, int sda, float *B)
	{

	// A and C are lower triangular
	// kmax+1 4-wide + end 3x3 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[2+bs*0];
		B[1+bs*0] = A0[3+bs*0];
		B[2+bs*0] = A1[0+bs*0];
		B[3+bs*0] = A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	// 3x3 triangle}

	B[1+bs*0] = A0[3+bs*0];
	B[2+bs*0] = A1[0+bs*0];
	B[3+bs*0] = A1[1+bs*0];

	B[2+bs*1] = A1[0+bs*1];
	B[3+bs*1] = A1[1+bs*1];

	B[3+bs*2] = A1[1+bs*2];

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_strcp_l_4_3_lib4(int kmax, float *A0, int sda, float *B)
	{

	// A and C are lower triangular
	// kmax+1 4-wide + end 3x3 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[3+bs*0];
		B[1+bs*0] = A1[0+bs*0];
		B[2+bs*0] = A1[1+bs*0];
		B[3+bs*0] = A1[2+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	// 3x3 triangle

	B[1+bs*0] = A1[0+bs*0];
	B[2+bs*0] = A1[1+bs*0];
	B[3+bs*0] = A1[2+bs*0];

	B[2+bs*1] = A1[1+bs*1];
	B[3+bs*1] = A1[2+bs*1];

	B[3+bs*2] = A1[2+bs*2];

	}



// both A and B are aligned to 64-bit boundaries
void kernel_strcp_l_3_0_lib4(int kmax, float *A, float *B)
	{

	// A and C are lower triangular
	// kmax+1 3-wide + end 2x2 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];
		B[2+bs*0] = A[2+bs*0];

		A += 4;
		B += 4;

		}

	// 2x2 triangle

	B[1+bs*0] = A[1+bs*0];
	B[2+bs*0] = A[2+bs*0];

	B[2+bs*1] = A[2+bs*1];

	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_strcp_l_3_2_lib4(int kmax, float *A0, int sda, float *B)
	{

	// A and C are lower triangular
	// kmax+1 3-wide + end 2x2 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[2+bs*0];
		B[1+bs*0] = A0[3+bs*0];
		B[2+bs*0] = A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	// 2x2 triangle

	B[1+bs*0] = A0[3+bs*0];
	B[2+bs*0] = A1[0+bs*0];

	B[2+bs*1] = A1[0+bs*1];

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_strcp_l_3_3_lib4(int kmax, float *A0, int sda, float *B)
	{

	// A and C are lower triangular
	// kmax+1 3-wide + end 2x2 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[3+bs*0];
		B[1+bs*0] = A1[0+bs*0];
		B[2+bs*0] = A1[1+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	// 2x2 triangle

	B[1+bs*0] = A1[0+bs*0];
	B[2+bs*0] = A1[1+bs*0];

	B[2+bs*1] = A1[1+bs*1];

	}



// both A and B are aligned to 64-bit boundaries
void kernel_strcp_l_2_0_lib4(int kmax, float *A, float *B)
	{

	// A and C are lower triangular
	// kmax+1 2-wide + end 1x1 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];
		B[1+bs*0] = A[1+bs*0];

		A += 4;
		B += 4;

		}

	// 1x1 triangle

	B[1+bs*0] = A[1+bs*0];

	}



// both A and B are aligned to 128-bit boundaries, 3 elements of A must be skipped
void kernel_strcp_l_2_3_lib4(int kmax, float *A0, int sda, float *B)
	{

	// A and C are lower triangular
	// kmax+1 2-wide + end 1x1 triangle

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A0[3+bs*0];
		B[1+bs*0] = A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	// 1x1 triangle

	B[1+bs*0] = A1[0+bs*0];

	}



// both A and B are aligned 64-bit boundaries
void kernel_strcp_l_1_0_lib4(int kmax, float *A, float *B)
	{

	// A and C are lower triangular
	// kmax+1 1-wide

	kmax += 1;

	if(kmax<=0)
		return;

	const int bs = 4;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] = A[0+bs*0];

		A += 4;
		B += 4;

		}

	}


// --- add

// both A and B are aligned to 256-bit boundaries
void kernel_sgead_4_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
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
void kernel_sgead_4_1_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
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
void kernel_sgead_4_2_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
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
void kernel_sgead_4_3_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
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
void kernel_sgead_3_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];
		B[2+bs*0] += alpha * A[2+bs*0];

		A += 4;
		B += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_sgead_3_2_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
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
void kernel_sgead_3_3_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
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
void kernel_sgead_2_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A[0+bs*0];
		B[1+bs*0] += alpha * A[1+bs*0];

		A += 4;
		B += 4;

		}

	}



// both A and B are aligned to 128-bit boundaries, 3 elements of A must be skipped
void kernel_sgead_2_3_lib4(int kmax, float *alphap, float *A0, int sda, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	float *A1 = A0 + bs*sda;

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A0[3+bs*0];
		B[1+bs*0] += alpha * A1[0+bs*0];

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	}



// both A and B are aligned 64-bit boundaries
void kernel_sgead_1_0_lib4(int kmax, float *alphap, float *A, float *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	float alpha = alphap[0];

	int k;

	for(k=0; k<kmax; k++)
		{

		B[0+bs*0] += alpha * A[0+bs*0];

		A += 4;
		B += 4;

		}

	}





