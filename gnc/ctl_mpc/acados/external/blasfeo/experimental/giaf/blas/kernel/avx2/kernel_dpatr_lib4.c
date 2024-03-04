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



// transposed of panel matrices, read across panels, write along panels
void kernel_dpatr_tn_4_lib4(int kmax, double *A, int sda, double *C)
	{

	const int bs = 4;
	
	__m256d
		v0, v1, v2, v3,
		v4, v5, v6, v7;
	
	int k;

	k = 0;
	for( ; k<kmax-3; k+=4)
		{

#if 1
		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*0] ) ), _mm_load_pd( &A[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*1] ) ), _mm_load_pd( &A[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*0] ) ), _mm_load_pd( &A[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*1] ) ), _mm_load_pd( &A[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A += sda*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		_mm256_store_pd( &C[0+bs*3], v7 );

		C += 4*bs;

#else // TODO alpha

		v0 = _mm256_load_pd( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_pd( &A[0+bs*1] ); // 01 11 21 31
		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 20 21
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 30 31
		v2 = _mm256_load_pd( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_pd( &A[0+bs*3] ); // 03 13 23 33
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 02 03 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 12 13 32 33
		
		A += sda*bs;

		v0 = _mm256_permute2f128_pd( v4, v6, 0x20 ); // 00 01 02 03
		_mm256_store_pd( &C[0+bs*0], v0 );
		v2 = _mm256_permute2f128_pd( v4, v6, 0x31 ); // 20 21 22 23
		_mm256_store_pd( &C[0+bs*2], v2 );
		v1 = _mm256_permute2f128_pd( v5, v7, 0x20 ); // 10 11 12 13
		_mm256_store_pd( &C[0+bs*1], v1 );
		v3 = _mm256_permute2f128_pd( v5, v7, 0x31 ); // 30 31 32 33
		_mm256_store_pd( &C[0+bs*3], v3 );

		C += 4*bs;

#endif

		}
	for( ; k<kmax; k++)
		{
		C[0+bs*0] = A[0+bs*0];
		C[1+bs*0] = A[0+bs*1];
		C[2+bs*0] = A[0+bs*2];
		C[3+bs*0] = A[0+bs*3];

		C += bs;
		A += 1;
		}
	
	return;

	}



#if 0
void kernel_dpack_nn_12_lib4(int kmax, double *A, int lda, double *C, int sdc)
	{

	const int bs = 4;

	int k;

	__m256d
		v0;
	
	double *C1 = C + bs*sdc;
	double *C2 = C1 + bs*sdc;
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{

		v0 = _mm256_loadu_pd( &A[0+lda*0] );
		_mm256_store_pd( &C[0+bs*0], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*1] );
		_mm256_store_pd( &C[0+bs*1], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*2] );
		_mm256_store_pd( &C[0+bs*2], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*3] );
		_mm256_store_pd( &C[0+bs*3], v0 );

		v0 = _mm256_loadu_pd( &A[4+lda*0] );
		_mm256_store_pd( &C1[0+bs*0], v0 );
		v0 = _mm256_loadu_pd( &A[4+lda*1] );
		_mm256_store_pd( &C1[0+bs*1], v0 );
		v0 = _mm256_loadu_pd( &A[4+lda*2] );
		_mm256_store_pd( &C1[0+bs*2], v0 );
		v0 = _mm256_loadu_pd( &A[4+lda*3] );
		_mm256_store_pd( &C1[0+bs*3], v0 );

		v0 = _mm256_loadu_pd( &A[8+lda*0] );
		_mm256_store_pd( &C2[0+bs*0], v0 );
		v0 = _mm256_loadu_pd( &A[8+lda*1] );
		_mm256_store_pd( &C2[0+bs*1], v0 );
		v0 = _mm256_loadu_pd( &A[8+lda*2] );
		_mm256_store_pd( &C2[0+bs*2], v0 );
		v0 = _mm256_loadu_pd( &A[8+lda*3] );
		_mm256_store_pd( &C2[0+bs*3], v0 );

		A += 4*lda;
		C += 4*bs;
		C1 += 4*bs;
		C2 += 4*bs;

		}
	for(; k<kmax; k++)
		{

		v0 = _mm256_loadu_pd( &A[0+lda*0] );
		_mm256_store_pd( &C[0+bs*0], v0 );

		v0 = _mm256_loadu_pd( &A[4+lda*0] );
		_mm256_store_pd( &C1[0+bs*0], v0 );

		v0 = _mm256_loadu_pd( &A[8+lda*0] );
		_mm256_store_pd( &C2[0+bs*0], v0 );

		A += 1*lda;
		C += 1*bs;
		C1 += 4*bs;
		C2 += 4*bs;

		}

	return;

	}



void kernel_dpack_nn_8_lib4(int kmax, double *A, int lda, double *C, int sdc)
	{

	const int bs = 4;

	int k;

	__m256d
		v0;
	
	double *C1 = C + bs*sdc;
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{

		v0 = _mm256_loadu_pd( &A[0+lda*0] );
		_mm256_store_pd( &C[0+bs*0], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*1] );
		_mm256_store_pd( &C[0+bs*1], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*2] );
		_mm256_store_pd( &C[0+bs*2], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*3] );
		_mm256_store_pd( &C[0+bs*3], v0 );

		v0 = _mm256_loadu_pd( &A[4+lda*0] );
		_mm256_store_pd( &C1[0+bs*0], v0 );
		v0 = _mm256_loadu_pd( &A[4+lda*1] );
		_mm256_store_pd( &C1[0+bs*1], v0 );
		v0 = _mm256_loadu_pd( &A[4+lda*2] );
		_mm256_store_pd( &C1[0+bs*2], v0 );
		v0 = _mm256_loadu_pd( &A[4+lda*3] );
		_mm256_store_pd( &C1[0+bs*3], v0 );

		A += 4*lda;
		C += 4*bs;
		C1 += 4*bs;

		}
	for(; k<kmax; k++)
		{

		v0 = _mm256_loadu_pd( &A[0+lda*0] );
		_mm256_store_pd( &C[0+bs*0], v0 );

		v0 = _mm256_loadu_pd( &A[4+lda*0] );
		_mm256_store_pd( &C1[0+bs*0], v0 );

		A += 1*lda;
		C += 1*bs;
		C1 += 4*bs;

		}

	return;

	}



void kernel_dpack_nn_4_lib4(int kmax, double *A, int lda, double *C)
	{

	const int bs = 4;

	int k;

	__m256d
		v0;
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{
		v0 = _mm256_loadu_pd( &A[0+lda*0] );
		_mm256_store_pd( &C[0+bs*0], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*1] );
		_mm256_store_pd( &C[0+bs*1], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*2] );
		_mm256_store_pd( &C[0+bs*2], v0 );
		v0 = _mm256_loadu_pd( &A[0+lda*3] );
		_mm256_store_pd( &C[0+bs*3], v0 );
		A += 4*lda;
		C += 4*bs;
		}
	for(; k<kmax; k++)
		{
		v0 = _mm256_loadu_pd( &A[0+lda*0] );
		_mm256_store_pd( &C[0+bs*0], v0 );
		A += 1*lda;
		C += 1*bs;
		}

	return;

	}



void kernel_dpack_tn_4_lib4(int kmax, double *A, int lda, double *C)
	{

	const int bs = 4;

	int k;

	__m256d
		v0, v1, v2, v3, v4, v5, v6, v7;
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{
		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+lda*0] ) ), _mm_load_pd( &A[0+lda*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+lda*1] ) ), _mm_load_pd( &A[0+lda*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+lda*0] ) ), _mm_load_pd( &A[2+lda*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+lda*1] ) ), _mm_load_pd( &A[2+lda*3]) , 0x1 ); // 21 31 23 33

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		_mm256_store_pd( &C[0+bs*3], v7 );

		A += 4;
		C += 4*bs;
		}
	for(; k<kmax; k++)
		{
		C[0+bs*0] = A[0+lda*0];
		C[1+bs*0] = A[0+lda*1];
		C[2+bs*0] = A[0+lda*2];
		C[3+bs*0] = A[0+lda*3];

		A += 1;
		C += bs;
		}

	return;

	}
#endif
