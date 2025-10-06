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

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#include "../../include/blasfeo_d_kernel.h"





/*
 * Copy only
 */





// both A and B are aligned to 256-bit boundaries
void kernel_dgecp_8_0_lib4(int tri, int kmax, double *A0, int sda, double *B, int sdb)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 8-wide + end 7x7 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		a_0;

	__m128d
		c_0;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		// 4x4
		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		_mm256_store_pd( &B[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		_mm256_store_pd( &B[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		_mm256_store_pd( &B[0+bs*3], a_0 );

		A0 += 16;
		B += 16;

		// 4x4
		a_0 = _mm256_load_pd( &A1[0+bs*0] );
		_mm256_store_pd( &B1[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*1] );
		_mm256_store_pd( &B1[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*2] );
		_mm256_store_pd( &B1[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*3] );
		_mm256_store_pd( &B1[0+bs*3], a_0 );

		A1 += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		// 4x1
		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		A0 += 4;
		B += 4;

		// 4x1
		a_0 = _mm256_load_pd( &A1[0+bs*0] );
		_mm256_store_pd( &B1[0+bs*0], a_0 );

		A1 += 4;
		B1 += 4;

		}

	if(tri==1)
		{
		// 7x7 triangle

		// 7x1
		// 1 SSE2
		c_0 = _mm_load_sd( &A0[1+0*bs] );
		_mm_store_sd( &B[1+0*bs], c_0 );
		// 2 SSE2
		c_0 = _mm_load_pd( &A0[2+0*bs] );
		_mm_store_pd( &B[2+0*bs], c_0 );
		// 4 AVX
		a_0 = _mm256_load_pd( &A1[0+0*bs] );
		_mm256_store_pd( &B1[0+0*bs], a_0 );

		// 6x1
		// 2 SSE2
		c_0 = _mm_load_pd( &A0[2+1*bs] );
		_mm_store_pd( &B[2+1*bs], c_0 );
		// 4 AVX
		a_0 = _mm256_load_pd( &A1[0+1*bs] );
		_mm256_store_pd( &B1[0+1*bs], a_0 );

		// 5x1
		// 1 SSE2
		c_0 = _mm_load_sd( &A0[3+2*bs] );
		_mm_store_sd( &B[3+2*bs], c_0 );
		// 4 AVX
		a_0 = _mm256_load_pd( &A1[0+2*bs] );
		_mm256_store_pd( &B1[0+2*bs], a_0 );

		// 4x1
		// 4 AVX
		a_0 = _mm256_load_pd( &A1[0+3*bs] );
		_mm256_store_pd( &B1[0+3*bs], a_0 );

		// 3x1
		// 1 SSE2
		c_0 = _mm_load_sd( &A1[1+4*bs] );
		_mm_store_sd( &B1[1+4*bs], c_0 );
		// 2 SSE2
		c_0 = _mm_load_pd( &A1[2+4*bs] );
		_mm_store_pd( &B1[2+4*bs], c_0 );

		// 2x1
		// 2 SSE2
		c_0 = _mm_load_pd( &A1[2+5*bs] );
		_mm_store_pd( &B1[2+5*bs], c_0 );

		// 1x1
		// 1 SSE2
		c_0 = _mm_load_sd( &A1[3+6*bs] );
		_mm_store_sd( &B1[3+6*bs], c_0 );

		}

	}



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

	__m256d
		a_0;

	__m128d
		c_0;

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*1] );
		_mm256_store_pd( &B[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*2] );
		_mm256_store_pd( &B[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*3] );
		_mm256_store_pd( &B[0+bs*3], a_0 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		A += 4;
		B += 4;

		}

	if(tri==1)
		{
		// 3x3 triangle

		// 3
		c_0 = _mm_load_sd( &A[1+bs*0] );
		_mm_store_sd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_pd( &A[2+bs*0] );
		_mm_store_pd( &B[2+bs*0], c_0 );

		// 2
		c_0 = _mm_load_pd( &A[2+bs*1] );
		_mm_store_pd( &B[2+bs*1], c_0 );

		// 1
		c_0 = _mm_load_sd( &A[3+bs*2] );
		_mm_store_sd( &B[3+bs*2], c_0 );

		}

	}





/*
 * Copy and scale
 */





// both A and B are aligned to 256-bit boundaries
void kernel_dgecpsc_8_0_lib4(int tri, int kmax, double alpha, double *A0, int sda,  double *B, int sdb)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 8-wide + end 7x7 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		alpha_0,
		a_0;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*3], a_0 );

		A0 += 16;
		B += 16;

		a_0 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*1] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*2] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*3] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+bs*3], a_0 );

		A1 += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		A0 += 4;
		B += 4;

		a_0 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+bs*0], a_0 );

		A1 += 4;
		B1 += 4;

		}

	if(tri==1)
		{
		// 7x7 triangle

		c_0 = _mm_load_sd( &A0[1+0*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[1+0*bs], c_0 );
		c_0 = _mm_load_pd( &A0[2+0*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+0*bs], c_0 );
		a_0 = _mm256_load_pd( &A1[0+0*bs] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+0*bs], a_0 );

		c_0 = _mm_load_pd( &A0[2+1*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+1*bs], c_0 );
		a_0 = _mm256_load_pd( &A1[0+1*bs] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+1*bs], a_0 );

		c_0 = _mm_load_sd( &A0[3+2*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+2*bs], c_0 );
		a_0 = _mm256_load_pd( &A1[0+2*bs] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+2*bs], a_0 );

		a_0 = _mm256_load_pd( &A1[0+3*bs] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B1[0+3*bs], a_0 );

		c_0 = _mm_load_sd( &A1[1+4*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[1+4*bs], c_0 );
		c_0 = _mm_load_pd( &A1[2+4*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+4*bs], c_0 );

		c_0 = _mm_load_pd( &A1[2+5*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+5*bs], c_0 );

		c_0 = _mm_load_sd( &A1[3+6*bs] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+6*bs], c_0 );

		}

	}



// both A and B are aligned to 256-bit boundaries, 1 element of A must be skipped
void kernel_dgecpsc_8_1_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B, int sdb)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 8-wide + end 7x7 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *A2 = A1 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		alpha_0,
		a_0, a_1, a_2,
		b_0, b_1;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_2 = _mm256_load_pd( &A2[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B1[0+bs*1], b_1 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_2 = _mm256_load_pd( &A2[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B1[0+bs*2], b_1 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_2 = _mm256_load_pd( &A2[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B1[0+bs*3], b_1 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		A0 += 4;
		A1 += 4;
		A2 += 4;
		B += 4;
		B1 += 4;

		}

	if(tri==1)
		{
		// 7x7 triangle

		c_0 = _mm_load_pd( &A0[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_sd( &A1[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*0], c_0 );
		c_0 = _mm_load_sd( &A1[1+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*0], c_0 );
		c_0 = _mm_load_pd( &A1[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*0], c_0 );
		c_0 = _mm_load_sd( &A2[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*0], c_0 );

		c_0 = _mm_load_sd( &A0[3+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[2+bs*1], c_0 );
		c_0 = _mm_load_sd( &A1[0+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*1], c_0 );
		c_0 = _mm_load_sd( &A1[1+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*1], c_0 );
		c_0 = _mm_load_pd( &A1[2+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*1], c_0 );
		c_0 = _mm_load_sd( &A2[0+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*1], c_0 );

		c_0 = _mm_load_sd( &A1[0+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*2], c_0 );
		c_0 = _mm_load_sd( &A1[1+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*2], c_0 );
		c_0 = _mm_load_pd( &A1[2+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*2], c_0 );
		c_0 = _mm_load_sd( &A2[0+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*2], c_0 );

		c_0 = _mm_load_sd( &A1[1+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*3], c_0 );
		c_0 = _mm_load_pd( &A1[2+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*3], c_0 );
		c_0 = _mm_load_sd( &A2[0+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*3], c_0 );

		c_0 = _mm_load_pd( &A1[2+bs*4] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*4], c_0 );
		c_0 = _mm_load_sd( &A2[0+bs*4] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*4], c_0 );

		c_0 = _mm_load_sd( &A1[3+bs*5] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[2+bs*5], c_0 );
		c_0 = _mm_load_sd( &A2[0+bs*5] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*5], c_0 );

		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		c_0 = _mm_load_sd( &A2[0+bs*6] );
		_mm_store_sd( &B1[3+bs*6], c_0 );

		}
	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_dgecpsc_8_2_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B, int sdb)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 8-wide + end 7x7 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *A2 = A1 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		alpha_0,
		a_0, a_1, a_2,
		b_0, b_1;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_2 = _mm256_load_pd( &A2[0+bs*1] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*1], b_0 );
		_mm256_store_pd( &B1[0+bs*1], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_2 = _mm256_load_pd( &A2[0+bs*2] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*2], b_0 );
		_mm256_store_pd( &B1[0+bs*2], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_2 = _mm256_load_pd( &A2[0+bs*3] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*3], b_0 );
		_mm256_store_pd( &B1[0+bs*3], b_1 );

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		A0 += 4;
		A1 += 4;
		A2 += 4;
		B += 4;
		B1 += 4;

		}

	if(tri==1)
		{
		// 7x7 triangle

		c_0 = _mm_load_sd( &A0[3+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_pd( &A1[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+bs*0], c_0 );
		c_0 = _mm_load_pd( &A1[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[0+bs*0], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+bs*0], c_0 );

		c_0 = _mm_load_pd( &A1[0+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+bs*1], c_0 );
		c_0 = _mm_load_pd( &A1[2+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[0+bs*1], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+bs*1], c_0 );

		c_0 = _mm_load_sd( &A1[1+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*2], c_0 );
		c_0 = _mm_load_pd( &A1[2+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[0+bs*2], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+bs*2], c_0 );

		c_0 = _mm_load_pd( &A1[2+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[0+bs*3], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+bs*3], c_0 );

		c_0 = _mm_load_sd( &A1[3+bs*4] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[1+bs*4], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*4] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+bs*4], c_0 );

		c_0 = _mm_load_pd( &A2[0+bs*5] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B1[2+bs*5], c_0 );

		c_0 = _mm_load_sd( &A2[1+bs*6] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*6], c_0 );

		}

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_dgecpsc_8_3_lib4(int tri, int kmax, double alpha, double *A0, int sda, double *B, int sdb)
	{

	if(tri==1)
		{
		// A and C are lower triangular
		// kmax+1 8-wide + end 7x7 triangle

		kmax += 1;
		}

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *A2 = A1 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		alpha_0,
		a_0, a_1, a_2,
		b_0, b_1;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_2 = _mm256_load_pd( &A2[0+bs*1] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*1], b_0 );
		_mm256_store_pd( &B1[0+bs*1], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_2 = _mm256_load_pd( &A2[0+bs*2] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*2], b_0 );
		_mm256_store_pd( &B1[0+bs*2], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_2 = _mm256_load_pd( &A2[0+bs*3] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*3], b_0 );
		_mm256_store_pd( &B1[0+bs*3], b_1 );

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		A0 += 4;
		A1 += 4;
		A2 += 4;
		B += 4;
		B1 += 4;

		}

	if(tri==1)
		{
		// 7x7 triangle

		c_0 = _mm_load_pd( &A1[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_sd( &A1[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*0], c_0 );
		c_0 = _mm_load_sd( &A1[3+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*0], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*0], c_0 );
		c_0 = _mm_load_sd( &A2[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*0], c_0 );

		c_0 = _mm_load_sd( &A1[1+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[2+bs*1], c_0 );
		c_0 = _mm_load_sd( &A1[2+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*1], c_0 );
		c_0 = _mm_load_sd( &A1[3+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*1], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*1], c_0 );
		c_0 = _mm_load_sd( &A2[2+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*1], c_0 );

		c_0 = _mm_load_sd( &A1[2+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*2], c_0 );
		c_0 = _mm_load_sd( &A1[3+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*2], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*2], c_0 );
		c_0 = _mm_load_sd( &A2[2+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*2], c_0 );

		c_0 = _mm_load_sd( &A1[3+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[0+bs*3], c_0 );
		c_0 = _mm_load_pd( &A2[0+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*3], c_0 );
		c_0 = _mm_load_sd( &A2[2+bs*3] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*3], c_0 );

		c_0 = _mm_load_pd( &A2[0+bs*4] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B1[1+bs*4], c_0 );
		c_0 = _mm_load_sd( &A2[2+bs*4] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*4], c_0 );

		c_0 = _mm_load_sd( &A2[1+bs*5] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[2+bs*5], c_0 );
		c_0 = _mm_load_sd( &A2[2+bs*5] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*5], c_0 );

		c_0 = _mm_load_sd( &A2[2+bs*6] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B1[3+bs*6], c_0 );

		}

	}



// both A and B are aligned to 256-bit boundaries
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

	__m256d
		alpha_0,
		a_0;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*1] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*2] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*3] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*3], a_0 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		A += 4;
		B += 4;

		}

	if(tri==1)
		{
		// 3x3 triangle

		c_0 = _mm_load_sd( &A[1+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_pd( &A[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+bs*0], c_0 );

		c_0 = _mm_load_pd( &A[2+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+bs*1], c_0 );

		c_0 = _mm_load_sd( &A[3+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*2], c_0 );

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

	__m256d
		alpha_0,
		a_0, a_1,
		b_0;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	if(tri==1)
		{
		// 3x3 triangle

		c_0 = _mm_load_pd( &A0[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_sd( &A1[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*0], c_0 );

		c_0 = _mm_load_sd( &A0[3+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[2+bs*1], c_0 );
		c_0 = _mm_load_sd( &A1[0+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*1], c_0 );

		c_0 = _mm_load_sd( &A1[0+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*2], c_0 );

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

	__m256d
		alpha_0,
		a_0, a_1,
		b_0;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	if(tri==1)
		{
		// 3x3 triangle

		c_0 = _mm_load_sd( &A0[3+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_pd( &A1[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+bs*0], c_0 );

		c_0 = _mm_load_pd( &A1[0+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_pd( &B[2+bs*1], c_0 );

		c_0 = _mm_load_sd( &A1[1+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*2], c_0 );

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

	__m256d
		alpha_0,
		a_0, a_1,
		b_0;

	__m128d
		c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	if(tri==1)
		{
		// 3x3 triangle

		c_0 = _mm_load_pd( &A1[0+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_storeu_pd( &B[1+bs*0], c_0 );
		c_0 = _mm_load_sd( &A1[2+bs*0] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*0], c_0 );

		c_0 = _mm_load_sd( &A1[1+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[2+bs*1], c_0 );
		c_0 = _mm_load_sd( &A1[2+bs*1] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*1], c_0 );

		c_0 = _mm_load_sd( &A1[2+bs*2] );
		c_0 = _mm_mul_pd( _mm256_castpd256_pd128( alpha_0 ), c_0 );
		_mm_store_sd( &B[3+bs*2], c_0 );
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

	__m128d
		alpha_0,
		a_0, a_1;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		a_1 = _mm_load_sd( &A[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		_mm_store_sd( &B[2+bs*0], a_1 );

		a_0 = _mm_loadu_pd( &A[0+bs*1] );
		a_1 = _mm_load_sd( &A[2+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );
		_mm_store_sd( &B[2+bs*1], a_1 );

		a_0 = _mm_loadu_pd( &A[0+bs*2] );
		a_1 = _mm_load_sd( &A[2+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );
		_mm_store_sd( &B[2+bs*2], a_1 );

		a_0 = _mm_loadu_pd( &A[0+bs*3] );
		a_1 = _mm_load_sd( &A[2+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );
		_mm_store_sd( &B[2+bs*3], a_1 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		a_1 = _mm_load_sd( &A[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		_mm_store_sd( &B[2+bs*0], a_1 );

		A += 4;
		B += 4;

		}

	if(tri==1)
		{
		// 2x2 triangle

		a_0 = _mm_loadu_pd( &A[1+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[1+bs*0], a_0 );

		a_0 = _mm_load_sd( &A[2+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[2+bs*1], a_0 );

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

	__m128d
		alpha_0,
		a_0, a_1;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_loadu_pd( &A0[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		a_1 = _mm_load_sd( &A1[0+bs*0] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_store_sd( &B[2+bs*0], a_1 );

		a_0 = _mm_loadu_pd( &A0[2+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );
		a_1 = _mm_load_sd( &A1[0+bs*1] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_store_sd( &B[2+bs*1], a_1 );

		a_0 = _mm_loadu_pd( &A0[2+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );
		a_1 = _mm_load_sd( &A1[0+bs*2] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_store_sd( &B[2+bs*2], a_1 );

		a_0 = _mm_loadu_pd( &A0[2+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );
		a_1 = _mm_load_sd( &A1[0+bs*3] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_store_sd( &B[2+bs*3], a_1 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_loadu_pd( &A0[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		a_1 = _mm_load_sd( &A1[0+bs*0] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_store_sd( &B[2+bs*0], a_1 );

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	if(tri==1)
		{
		// 2x2 triangle

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[1+bs*0], a_0 );
		a_0 = _mm_load_sd( &A1[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[2+bs*0], a_0 );

		a_0 = _mm_load_sd( &A1[0+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[2+bs*1], a_0 );

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

	__m128d
		alpha_0,
		a_0, a_1;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*0], a_0 );
		a_1 = _mm_loadu_pd( &A1[0+bs*0] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[1+bs*0], a_1 );

		a_0 = _mm_load_sd( &A0[3+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*1], a_0 );
		a_1 = _mm_loadu_pd( &A1[0+bs*1] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[1+bs*1], a_1 );

		a_0 = _mm_load_sd( &A0[3+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*2], a_0 );
		a_1 = _mm_loadu_pd( &A1[0+bs*2] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[1+bs*2], a_1 );

		a_0 = _mm_load_sd( &A0[3+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*3], a_0 );
		a_1 = _mm_loadu_pd( &A1[0+bs*3] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[1+bs*3], a_1 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*0], a_0 );
		a_1 = _mm_loadu_pd( &A1[0+bs*0] );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		_mm_storeu_pd( &B[1+bs*0], a_1 );

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	if(tri==1)
		{
		// 2x2 triangle

		a_0 = _mm_loadu_pd( &A1[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[1+bs*0], a_0 );

		a_0 = _mm_load_sd( &A1[1+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[2+bs*1], a_0 );

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

	__m128d
		alpha_0,
		a_0;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

		a_0 = _mm_loadu_pd( &A[0+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );

		a_0 = _mm_loadu_pd( &A[0+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );

		a_0 = _mm_loadu_pd( &A[0+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

		A += 4;
		B += 4;

		}

	if(tri==1)
		{
		// 1x1 triangle

		a_0 = _mm_load_sd( &A[1+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[1+bs*0], a_0 );

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

	__m128d
		alpha_0,
		a_0;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

		a_0 = _mm_load_sd( &A0[3+bs*1] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );

		a_0 = _mm_load_sd( &A0[3+bs*2] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );

		a_0 = _mm_load_sd( &A0[3+bs*3] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

		A0 += 4;
		A1 += 4;
		B  += 4;

		}

	if(tri==1)
		{
		// 1x1 triangle

		a_0 = _mm_load_sd( &A1[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[1+bs*0], a_0 );

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

	__m128d
		alpha_0,
		a_0;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_sd( &A[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*0], a_0 );

		a_0 = _mm_load_sd( &A[0+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*1], a_0 );

		a_0 = _mm_load_sd( &A[0+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*2], a_0 );

		a_0 = _mm_load_sd( &A[0+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*3], a_0 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_sd( &A[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		_mm_store_sd( &B[0+bs*0], a_0 );

		A += 4;
		B += 4;

		}

	}




// both A and B are aligned to 256-bit boundaries
void kernel_dgead_8_0_lib4(int kmax, double alpha, double *A0, int sda,  double *B, int sdb)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		a_0, c_0, alpha_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B[0+bs*3], a_0 );

		A0 += 16;
		B += 16;

		a_0 = _mm256_load_pd( &A1[0+bs*0] );
		c_0 = _mm256_load_pd( &B1[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B1[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*1] );
		c_0 = _mm256_load_pd( &B1[0+bs*1] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B1[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*2] );
		c_0 = _mm256_load_pd( &B1[0+bs*2] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B1[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A1[0+bs*3] );
		c_0 = _mm256_load_pd( &B1[0+bs*3] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B1[0+bs*3], a_0 );

		A1 += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		A0 += 4;
		B += 4;

		a_0 = _mm256_load_pd( &A1[0+bs*0] );
		c_0 = _mm256_load_pd( &B1[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( a_0, c_0 );
		_mm256_store_pd( &B1[0+bs*0], a_0 );

		A1 += 4;
		B1 += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 1 element of A must be skipped
void kernel_dgead_8_1_lib4(int kmax, double alpha, double *A0, int sda, double *B, int sdb)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *A2 = A1 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		a_0, a_1, a_2,
		b_0, b_1,
		alpha_0, c_0, c_1;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		c_1 = _mm256_load_pd( &B1[0+bs*0] );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_2 = _mm256_load_pd( &A2[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		c_1 = _mm256_load_pd( &B1[0+bs*1] );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		_mm256_store_pd( &B1[0+bs*1], b_1 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_2 = _mm256_load_pd( &A2[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		c_1 = _mm256_load_pd( &B1[0+bs*2] );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		_mm256_store_pd( &B1[0+bs*2], b_1 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_2 = _mm256_load_pd( &A2[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		c_1 = _mm256_load_pd( &B1[0+bs*3] );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		_mm256_store_pd( &B1[0+bs*3], b_1 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_2 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_shuffle_pd( a_1, a_2, 0x5 );
		b_0 = _mm256_shuffle_pd( a_0, b_0, 0x5 );
		c_1 = _mm256_load_pd( &B1[0+bs*0] );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		A0 += 4;
		A1 += 4;
		A2 += 4;
		B += 4;
		B1 += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 2 elements of A must be skipped
void kernel_dgead_8_2_lib4(int kmax, double alpha, double *A0, int sda, double *B, int sdb)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *A2 = A1 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		a_0, a_1, a_2,
		b_0, b_1,
		alpha_0, c_0, c_1;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		c_1 = _mm256_load_pd( &B1[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_2 = _mm256_load_pd( &A2[0+bs*1] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		c_1 = _mm256_load_pd( &B1[0+bs*1] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*1], b_0 );
		_mm256_store_pd( &B1[0+bs*1], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_2 = _mm256_load_pd( &A2[0+bs*2] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		c_1 = _mm256_load_pd( &B1[0+bs*2] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*2], b_0 );
		_mm256_store_pd( &B1[0+bs*2], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_2 = _mm256_load_pd( &A2[0+bs*3] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		c_1 = _mm256_load_pd( &B1[0+bs*3] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*3], b_0 );
		_mm256_store_pd( &B1[0+bs*3], b_1 );

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		c_1 = _mm256_load_pd( &B1[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		A0 += 4;
		A1 += 4;
		A2 += 4;
		B += 4;
		B1 += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries, 3 elements of A must be skipped
void kernel_dgead_8_3_lib4(int kmax, double alpha, double *A0, int sda, double *B, int sdb)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	double *A1 = A0 + bs*sda;
	double *A2 = A1 + bs*sda;
	double *B1 = B + bs*sdb;

	__m256d
		a_0, a_1, a_2,
		b_0, b_1,
		alpha_0, c_0, c_1;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		c_1 = _mm256_load_pd( &B1[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_2 = _mm256_load_pd( &A2[0+bs*1] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		c_1 = _mm256_load_pd( &B1[0+bs*1] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*1], b_0 );
		_mm256_store_pd( &B1[0+bs*1], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_2 = _mm256_load_pd( &A2[0+bs*2] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		c_1 = _mm256_load_pd( &B1[0+bs*2] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*2], b_0 );
		_mm256_store_pd( &B1[0+bs*2], b_1 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_2 = _mm256_load_pd( &A2[0+bs*3] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		c_1 = _mm256_load_pd( &B1[0+bs*3] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*3], b_0 );
		_mm256_store_pd( &B1[0+bs*3], b_1 );

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B += 16;
		B1 += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_2 = _mm256_load_pd( &A2[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_1 = _mm256_permute2f128_pd( a_1, a_2, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		b_1 = _mm256_shuffle_pd( b_1, a_2, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		c_1 = _mm256_load_pd( &B1[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_1 = _mm256_mul_pd( alpha_0, b_1 );
		b_0 = _mm256_add_pd ( c_0, b_0 );
		b_1 = _mm256_add_pd ( c_1, b_1 );
		_mm256_store_pd( &B[0+bs*0], b_0 );
		_mm256_store_pd( &B1[0+bs*0], b_1 );

		A0 += 4;
		A1 += 4;
		A2 += 4;
		B += 4;
		B1 += 4;

		}

	}



// both A and B are aligned to 256-bit boundaries
void kernel_dgead_4_0_lib4(int kmax, double alpha, double *A, double *B)
	{

	if(kmax<=0)
		return;

	const int bs = 4;

	__m256d
		a_0, c_0, alpha_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( c_0, a_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*1] );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( c_0, a_0 );
		_mm256_store_pd( &B[0+bs*1], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*2] );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( c_0, a_0 );
		_mm256_store_pd( &B[0+bs*2], a_0 );

		a_0 = _mm256_load_pd( &A[0+bs*3] );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( c_0, a_0 );
		_mm256_store_pd( &B[0+bs*3], a_0 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		a_0 = _mm256_mul_pd( alpha_0, a_0 );
		a_0 = _mm256_add_pd( c_0, a_0 );
		_mm256_store_pd( &B[0+bs*0], a_0 );

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

	__m256d
		a_0, a_1,
		b_0,
		alpha_0, c_0;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	int k;

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_1 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

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

	__m256d
		a_0, a_1,
		b_0,
		alpha_0, c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

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

	__m256d
		a_0, a_1,
		b_0,
		alpha_0, c_0;

	int k;

	alpha_0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_1 = _mm256_load_pd( &A1[0+bs*1] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*1] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*1], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_1 = _mm256_load_pd( &A1[0+bs*2] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*2] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*2], b_0 );

		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_1 = _mm256_load_pd( &A1[0+bs*3] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*3] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*3], b_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_1 = _mm256_load_pd( &A1[0+bs*0] );
		a_0 = _mm256_permute2f128_pd( a_0, a_1, 0x21 );
		b_0 = _mm256_shuffle_pd( a_0, a_1, 0x5 );
		c_0 = _mm256_load_pd( &B[0+bs*0] );
		b_0 = _mm256_mul_pd( alpha_0, b_0 );
		b_0 = _mm256_add_pd( c_0, b_0 );
		_mm256_store_pd( &B[0+bs*0], b_0 );

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

	__m128d
		a_0, a_1,
		alpha_0, c_0, c_1;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		a_1 = _mm_load_sd( &A[2+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		c_1 = _mm_load_sd( &B[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		_mm_store_sd( &B[2+bs*0], a_1 );

		a_0 = _mm_loadu_pd( &A[0+bs*1] );
		a_1 = _mm_load_sd( &A[2+bs*1] );
		c_0 = _mm_loadu_pd( &B[0+bs*1] );
		c_1 = _mm_load_sd( &B[2+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );
		_mm_store_sd( &B[2+bs*1], a_1 );

		a_0 = _mm_loadu_pd( &A[0+bs*2] );
		a_1 = _mm_load_sd( &A[2+bs*2] );
		c_0 = _mm_loadu_pd( &B[0+bs*2] );
		c_1 = _mm_load_sd( &B[2+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );
		_mm_store_sd( &B[2+bs*2], a_1 );

		a_0 = _mm_loadu_pd( &A[0+bs*3] );
		a_1 = _mm_load_sd( &A[2+bs*3] );
		c_0 = _mm_loadu_pd( &B[0+bs*3] );
		c_1 = _mm_load_sd( &B[2+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );
		_mm_store_sd( &B[2+bs*3], a_1 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		a_1 = _mm_load_sd( &A[2+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		c_1 = _mm_load_sd( &B[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		_mm_store_sd( &B[2+bs*0], a_1 );

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

	__m128d
		a_0, a_1,
		alpha_0, c_0, c_1;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_loadu_pd( &A0[2+bs*0] );
		a_1 = _mm_load_sd( &A1[0+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		c_1 = _mm_load_sd( &B[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		_mm_store_sd( &B[2+bs*0], a_1 );

		a_0 = _mm_loadu_pd( &A0[2+bs*1] );
		a_1 = _mm_load_sd( &A1[0+bs*1] );
		c_0 = _mm_loadu_pd( &B[0+bs*1] );
		c_1 = _mm_load_sd( &B[2+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );
		_mm_store_sd( &B[2+bs*1], a_1 );

		a_0 = _mm_loadu_pd( &A0[2+bs*2] );
		a_1 = _mm_load_sd( &A1[0+bs*2] );
		c_0 = _mm_loadu_pd( &B[0+bs*2] );
		c_1 = _mm_load_sd( &B[2+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );
		_mm_store_sd( &B[2+bs*2], a_1 );

		a_0 = _mm_loadu_pd( &A0[2+bs*3] );
		a_1 = _mm_load_sd( &A1[0+bs*3] );
		c_0 = _mm_loadu_pd( &B[0+bs*3] );
		c_1 = _mm_load_sd( &B[2+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );
		_mm_store_sd( &B[2+bs*3], a_1 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_loadu_pd( &A0[2+bs*0] );
		a_1 = _mm_load_sd( &A1[0+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		c_1 = _mm_load_sd( &B[2+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_1 = _mm_mul_sd( alpha_0, a_1 );
		a_0 = _mm_add_pd( c_0, a_0 );
		a_1 = _mm_add_sd( c_1, a_1 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );
		_mm_store_sd( &B[2+bs*0], a_1 );

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

	__m128d
		a_0, a_1,
		alpha_0, c_0, c_1;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_1 = _mm_loadu_pd( &A1[0+bs*0] );
		c_0 = _mm_load_sd( &B[0+bs*0] );
		c_1 = _mm_loadu_pd( &B[1+bs*0] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		a_0 = _mm_add_sd( c_0, a_0 );
		a_1 = _mm_add_pd( c_1, a_1 );
		_mm_store_sd( &B[0+bs*0], a_0 );
		_mm_storeu_pd( &B[1+bs*0], a_1 );

		a_0 = _mm_load_sd( &A0[3+bs*1] );
		a_1 = _mm_loadu_pd( &A1[0+bs*1] );
		c_0 = _mm_load_sd( &B[0+bs*1] );
		c_1 = _mm_loadu_pd( &B[1+bs*1] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		a_0 = _mm_add_sd( c_0, a_0 );
		a_1 = _mm_add_pd( c_1, a_1 );
		_mm_store_sd( &B[0+bs*1], a_0 );
		_mm_storeu_pd( &B[1+bs*1], a_1 );

		a_0 = _mm_load_sd( &A0[3+bs*2] );
		a_1 = _mm_loadu_pd( &A1[0+bs*2] );
		c_0 = _mm_load_sd( &B[0+bs*2] );
		c_1 = _mm_loadu_pd( &B[1+bs*2] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		a_0 = _mm_add_sd( c_0, a_0 );
		a_1 = _mm_add_pd( c_1, a_1 );
		_mm_store_sd( &B[0+bs*2], a_0 );
		_mm_storeu_pd( &B[1+bs*2], a_1 );

		a_0 = _mm_load_sd( &A0[3+bs*3] );
		a_1 = _mm_loadu_pd( &A1[0+bs*3] );
		c_0 = _mm_load_sd( &B[0+bs*3] );
		c_1 = _mm_loadu_pd( &B[1+bs*3] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		a_0 = _mm_add_sd( c_0, a_0 );
		a_1 = _mm_add_pd( c_1, a_1 );
		_mm_store_sd( &B[0+bs*3], a_0 );
		_mm_storeu_pd( &B[1+bs*3], a_1 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_1 = _mm_loadu_pd( &A1[0+bs*0] );
		c_0 = _mm_load_sd( &B[0+bs*0] );
		c_1 = _mm_loadu_pd( &B[1+bs*0] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_1 = _mm_mul_pd( alpha_0, a_1 );
		a_0 = _mm_add_sd( c_0, a_0 );
		a_1 = _mm_add_pd( c_1, a_1 );
		_mm_store_sd( &B[0+bs*0], a_0 );
		_mm_storeu_pd( &B[1+bs*0], a_1 );

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

	__m128d
		a_0, c_0, alpha_0;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

		a_0 = _mm_loadu_pd( &A[0+bs*1] );
		c_0 = _mm_loadu_pd( &B[0+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );

		a_0 = _mm_loadu_pd( &A[0+bs*2] );
		c_0 = _mm_loadu_pd( &B[0+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );

		a_0 = _mm_loadu_pd( &A[0+bs*3] );
		c_0 = _mm_loadu_pd( &B[0+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_loadu_pd( &A[0+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

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

	__m128d
		a_0, c_0, alpha_0;

	int k;

	alpha_0 = _mm_loaddup_pd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

		a_0 = _mm_load_sd( &A0[3+bs*1] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*1] );
		c_0 = _mm_loadu_pd( &B[0+bs*1] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*1], a_0 );

		a_0 = _mm_load_sd( &A0[3+bs*2] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*2] );
		c_0 = _mm_loadu_pd( &B[0+bs*2] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*2], a_0 );

		a_0 = _mm_load_sd( &A0[3+bs*3] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*3] );
		c_0 = _mm_loadu_pd( &B[0+bs*3] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*3], a_0 );

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_sd( &A0[3+bs*0] );
		a_0 = _mm_loadh_pd( a_0, &A1[0+bs*0] );
		c_0 = _mm_loadu_pd( &B[0+bs*0] );
		a_0 = _mm_mul_pd( alpha_0, a_0 );
		a_0 = _mm_add_pd( c_0, a_0 );
		_mm_storeu_pd( &B[0+bs*0], a_0 );

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

	__m128d
		a_0, c_0, alpha_0;

	int k;

	alpha_0 = _mm_load_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_sd( &A[0+bs*0] );
		c_0 = _mm_load_sd( &B[0+bs*0] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_0 = _mm_add_sd( c_0, a_0 );
		_mm_store_sd( &B[0+bs*0], a_0 );

		a_0 = _mm_load_sd( &A[0+bs*1] );
		c_0 = _mm_load_sd( &B[0+bs*1] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_0 = _mm_add_sd( c_0, a_0 );
		_mm_store_sd( &B[0+bs*1], a_0 );

		a_0 = _mm_load_sd( &A[0+bs*2] );
		c_0 = _mm_load_sd( &B[0+bs*2] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_0 = _mm_add_sd( c_0, a_0 );
		_mm_store_sd( &B[0+bs*2], a_0 );

		a_0 = _mm_load_sd( &A[0+bs*3] );
		c_0 = _mm_load_sd( &B[0+bs*3] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_0 = _mm_add_sd( c_0, a_0 );
		_mm_store_sd( &B[0+bs*3], a_0 );

		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_sd( &A[0+bs*0] );
		c_0 = _mm_load_sd( &B[0+bs*0] );
		a_0 = _mm_mul_sd( alpha_0, a_0 );
		a_0 = _mm_add_sd( c_0, a_0 );
		_mm_store_sd( &B[0+bs*0], a_0 );

		A += 4;
		B += 4;

		}

	}



void kernel_dgeset_4_lib4(int kmax, double alpha, double *A)
	{

	int k;

	__m256d
		a0;

	a0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		_mm256_store_pd( &A[0], a0 );
		_mm256_store_pd( &A[4], a0 );
		_mm256_store_pd( &A[8], a0 );
		_mm256_store_pd( &A[12], a0 );

		A += 16;

		}
	for(; k<kmax; k++)
		{

		_mm256_store_pd( &A[0], a0 );

		A += 4;

		}

	}


// A lower triangular
void kernel_dtrset_4_lib4(int kmax, double alpha, double *A)
	{

	int k;

	__m256d
		a0;

	a0 = _mm256_broadcast_sd( &alpha );

	for(k=0; k<kmax-3; k+=4)
		{

		_mm256_store_pd( &A[0], a0 );
		_mm256_store_pd( &A[4], a0 );
		_mm256_store_pd( &A[8], a0 );
		_mm256_store_pd( &A[12], a0 );

		A += 16;

		}
	for(; k<kmax; k++)
		{

		_mm256_store_pd( &A[0], a0 );

		A += 4;

		}

	// final 4x4 triangle
	_mm256_store_pd( &A[0], a0 );

	_mm_store_sd( &A[5], _mm256_castpd256_pd128( a0 ) );
	_mm_store_pd( &A[6], _mm256_castpd256_pd128( a0 ) );

	_mm_store_pd( &A[10], _mm256_castpd256_pd128( a0 ) );

	_mm_store_sd( &A[15], _mm256_castpd256_pd128( a0 ) );

	}



