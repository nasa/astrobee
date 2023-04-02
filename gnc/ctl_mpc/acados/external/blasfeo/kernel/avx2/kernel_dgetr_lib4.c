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




// TODO tri !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void kernel_dgetr_8_lib4(int tri, int kmax, int kna, double alpha, double *A0, int sda, double *C, int sdc)
	{

	const int bs = 4;
	
	double *A1 = A0 + bs*sda;

	int k;

	__m256d
		alph, 
		v0, v1, v2, v3, v4, v5, v6, v7,
		v8, v9, va, vb, vc, vd, ve, vf;
	
	alph = _mm256_broadcast_sd( &alpha );
	
	k = 0;

	if(kmax<kna)
		goto cleanup_loop;

	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			C[0+bs*0] = alpha * A0[0+bs*0];
			C[0+bs*1] = alpha * A0[1+bs*0];
			C[0+bs*2] = alpha * A0[2+bs*0];
			C[0+bs*3] = alpha * A0[3+bs*0];

			C[0+bs*4] = alpha * A1[0+bs*0];
			C[0+bs*5] = alpha * A1[1+bs*0];
			C[0+bs*6] = alpha * A1[2+bs*0];
			C[0+bs*7] = alpha * A1[3+bs*0];

			C  += 1;
			A0 += bs;
			A1 += bs;
			}
		C += bs*(sdc-1);
		}
	
	for(; k<kmax-7; k+=8)
		{

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*0] ) ), _mm_load_pd( &A0[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*1] ) ), _mm_load_pd( &A0[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*0] ) ), _mm_load_pd( &A0[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*1] ) ), _mm_load_pd( &A0[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A0 += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*3], v7 );

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*0] ) ), _mm_load_pd( &A1[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*1] ) ), _mm_load_pd( &A1[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*0] ) ), _mm_load_pd( &A1[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*1] ) ), _mm_load_pd( &A1[2+bs*3]) , 0x1 ); // 21 31 23 33

		A1 += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*4], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*5], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*6], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*7], v7 );

		C += sdc*bs;

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*0] ) ), _mm_load_pd( &A0[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*1] ) ), _mm_load_pd( &A0[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*0] ) ), _mm_load_pd( &A0[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*1] ) ), _mm_load_pd( &A0[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A0 += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*3], v7 );

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*0] ) ), _mm_load_pd( &A1[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*1] ) ), _mm_load_pd( &A1[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*0] ) ), _mm_load_pd( &A1[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*1] ) ), _mm_load_pd( &A1[2+bs*3]) , 0x1 ); // 21 31 23 33

		A1 += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*4], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*5], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*6], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*7], v7 );

		C += sdc*bs;

		}

	for(; k<kmax-3; k+=4)
		{

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*0] ) ), _mm_load_pd( &A0[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*1] ) ), _mm_load_pd( &A0[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*0] ) ), _mm_load_pd( &A0[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*1] ) ), _mm_load_pd( &A0[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A0 += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*3], v7 );

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*0] ) ), _mm_load_pd( &A1[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*1] ) ), _mm_load_pd( &A1[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*0] ) ), _mm_load_pd( &A1[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*1] ) ), _mm_load_pd( &A1[2+bs*3]) , 0x1 ); // 21 31 23 33

		A1 += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*4], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*5], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*6], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*7], v7 );

		C += sdc*bs;

		}

	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = alpha * A0[0+bs*0];
		C[0+bs*1] = alpha * A0[1+bs*0];
		C[0+bs*2] = alpha * A0[2+bs*0];
		C[0+bs*3] = alpha * A0[3+bs*0];

		C[0+bs*4] = alpha * A1[0+bs*0];
		C[0+bs*5] = alpha * A1[1+bs*0];
		C[0+bs*6] = alpha * A1[2+bs*0];
		C[0+bs*7] = alpha * A1[3+bs*0];

		C  += 1;
		A0 += bs;
		A1 += bs;
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_dgetr_4_lib4(int tri, int kmax, int kna, double alpha, double *A, double *C, int sdc)
	{

	if(tri==1)
		{
		// A is lower triangular, C is upper triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	const int bs = 4;
	
	__m256d
		alph,
		v0, v1, v2, v3,
		v4, v5, v6, v7;
	
	alph = _mm256_broadcast_sd( &alpha );
	
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

	for( ; k<kmax-7; k+=8)
		{

#if 1

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*0] ) ), _mm_load_pd( &A[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*1] ) ), _mm_load_pd( &A[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*0] ) ), _mm_load_pd( &A[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*1] ) ), _mm_load_pd( &A[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*3], v7 );

		C += sdc*bs;

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*0] ) ), _mm_load_pd( &A[0+bs*2]) , 0x1 );
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*1] ) ), _mm_load_pd( &A[0+bs*3]) , 0x1 );
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*0] ) ), _mm_load_pd( &A[2+bs*2]) , 0x1 );
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*1] ) ), _mm_load_pd( &A[2+bs*3]) , 0x1 );
		
		A += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*3], v7 );

		C += sdc*bs;

#else // TODO alpha

		v0 = _mm256_load_pd( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_pd( &A[0+bs*1] ); // 01 11 21 31
		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 20 21
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 30 31
		v2 = _mm256_load_pd( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_pd( &A[0+bs*3] ); // 03 13 23 33
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 02 03 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 12 13 32 33
		
		A += bs*bs;

		v0 = _mm256_permute2f128_pd( v4, v6, 0x20 ); // 00 01 02 03
		_mm256_store_pd( &C[0+bs*0], v0 );
		v2 = _mm256_permute2f128_pd( v4, v6, 0x31 ); // 20 21 22 23
		_mm256_store_pd( &C[0+bs*2], v2 );
		v1 = _mm256_permute2f128_pd( v5, v7, 0x20 ); // 10 11 12 13
		_mm256_store_pd( &C[0+bs*1], v1 );
		v3 = _mm256_permute2f128_pd( v5, v7, 0x31 ); // 30 31 32 33
		_mm256_store_pd( &C[0+bs*3], v3 );

		C += bs*sdc;

		v0 = _mm256_load_pd( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_pd( &A[0+bs*1] ); // 01 11 21 31
		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 20 21
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 30 31
		v2 = _mm256_load_pd( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_pd( &A[0+bs*3] ); // 03 13 23 33
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 02 03 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 12 13 32 33
		
		A += bs*bs;

		v0 = _mm256_permute2f128_pd( v4, v6, 0x20 ); // 00 01 02 03
		_mm256_store_pd( &C[0+bs*0], v0 );
		v2 = _mm256_permute2f128_pd( v4, v6, 0x31 ); // 20 21 22 23
		_mm256_store_pd( &C[0+bs*2], v2 );
		v1 = _mm256_permute2f128_pd( v5, v7, 0x20 ); // 10 11 12 13
		_mm256_store_pd( &C[0+bs*1], v1 );
		v3 = _mm256_permute2f128_pd( v5, v7, 0x31 ); // 30 31 32 33
		_mm256_store_pd( &C[0+bs*3], v3 );

		C += bs*sdc;

#endif

		}

	for( ; k<kmax-3; k+=4)
		{

#if 1

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*0] ) ), _mm_load_pd( &A[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*1] ) ), _mm_load_pd( &A[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*0] ) ), _mm_load_pd( &A[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*1] ) ), _mm_load_pd( &A[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v4 = _mm256_mul_pd( v4, alph );
		_mm256_store_pd( &C[0+bs*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v5 = _mm256_mul_pd( v5, alph );
		_mm256_store_pd( &C[0+bs*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v6 = _mm256_mul_pd( v6, alph );
		_mm256_store_pd( &C[0+bs*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		v7 = _mm256_mul_pd( v7, alph );
		_mm256_store_pd( &C[0+bs*3], v7 );

		C += sdc*bs;

#else

		v0 = _mm256_load_pd( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_pd( &A[0+bs*1] ); // 01 11 21 31
		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 20 21
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 30 31
		v2 = _mm256_load_pd( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_pd( &A[0+bs*3] ); // 03 13 23 33
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 02 03 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 12 13 32 33
		
		A += bs*bs;

		v0 = _mm256_permute2f128_pd( v4, v6, 0x20 ); // 00 01 02 03
		_mm256_store_pd( &C[0+bs*0], v0 );
		v2 = _mm256_permute2f128_pd( v4, v6, 0x31 ); // 20 21 22 23
		_mm256_store_pd( &C[0+bs*2], v2 );
		v1 = _mm256_permute2f128_pd( v5, v7, 0x20 ); // 10 11 12 13
		_mm256_store_pd( &C[0+bs*1], v1 );
		v3 = _mm256_permute2f128_pd( v5, v7, 0x31 ); // 30 31 32 33
		_mm256_store_pd( &C[0+bs*3], v3 );

		C += bs*sdc;

#endif

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
void kernel_dgetr_3_lib4(int tri, int kmax, int kna, double alpha, double *A, double *C, int sdc)
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
void kernel_dgetr_2_lib4(int tri, int kmax, int kna, double alpha, double *A, double *C, int sdc)
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
void kernel_dgetr_1_lib4(int tri, int kmax, int kna, double alpha, double *A, double *C, int sdc)
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



// transposed of general matrices, read across panels, write along panels
void kernel_dgetr_4_0_lib4(int kmax, double *A, int sda, double *B)
	{
	const int ps = 4;
	__m256d
		v0, v1, v2, v3, v4, v5, v6, v7;
	int k;
	for(k=0; k<kmax-3; k+=4)
		{

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+ps*0] ) ), _mm_load_pd( &A[0+ps*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+ps*1] ) ), _mm_load_pd( &A[0+ps*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+ps*0] ) ), _mm_load_pd( &A[2+ps*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+ps*1] ) ), _mm_load_pd( &A[2+ps*3]) , 0x1 ); // 21 31 23 33
		
		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		_mm256_store_pd( &B[0+ps*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		_mm256_store_pd( &B[0+ps*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		_mm256_store_pd( &B[0+ps*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		_mm256_store_pd( &B[0+ps*3], v7 );

		A += ps*sda;
		B += ps*ps;
		}
	for( ; k<kmax; k++)
		{
		//
		B[0+ps*0] = A[0+ps*0];
		B[1+ps*0] = A[0+ps*1];
		B[2+ps*0] = A[0+ps*2];
		B[3+ps*0] = A[0+ps*3];

		A += 1;
		B += ps;
		}
	return;
	}



#if 0
// transposed of general matrices, read across panels, write along panels
void kernel_dpacp_tn_4_lib4(int kmax, int offsetA, double *A, int sda, double *B)
	{

	const int ps = 4;

	__m256d
		v0, v1, v2, v3, v4, v5, v6, v7;

	int k;

	int kna = (ps-offsetA)%ps;
	kna = kmax<kna ? kmax : kna;

	k = 0;
	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			//
			B[0+ps*0] = A[0+ps*0];
			B[1+ps*0] = A[0+ps*1];
			B[2+ps*0] = A[0+ps*2];
			B[3+ps*0] = A[0+ps*3];

			A += 1;
			B += ps;
			}
		A += ps*(sda-1);
		}
	for(; k<kmax-3; k+=4)
		{

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+ps*0] ) ), _mm_load_pd( &A[0+ps*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+ps*1] ) ), _mm_load_pd( &A[0+ps*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+ps*0] ) ), _mm_load_pd( &A[2+ps*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+ps*1] ) ), _mm_load_pd( &A[2+ps*3]) , 0x1 ); // 21 31 23 33
		
		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		_mm256_store_pd( &B[0+ps*0], v4 );
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		_mm256_store_pd( &B[0+ps*1], v5 );
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		_mm256_store_pd( &B[0+ps*2], v6 );
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33
		_mm256_store_pd( &B[0+ps*3], v7 );

		A += ps*sda;
		B += ps*ps;
		}
	for( ; k<kmax; k++)
		{
		//
		B[0+ps*0] = A[0+ps*0];
		B[1+ps*0] = A[0+ps*1];
		B[2+ps*0] = A[0+ps*2];
		B[3+ps*0] = A[0+ps*3];

		A += 1;
		B += ps;
		}
	return;
	}
#endif

