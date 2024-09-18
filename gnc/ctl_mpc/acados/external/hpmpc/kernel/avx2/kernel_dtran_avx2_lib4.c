/**************************************************************************************************
*                                                                                                 *
* This file is part of HPMPC.                                                                     *
*                                                                                                 *
* HPMPC -- Library for High-Performance implementation of solvers for MPC.                        *
* Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                *
*                                                                                                 *
* HPMPC is free software; you can redistribute it and/or                                          *
* modify it under the terms of the GNU Lesser General Public                                      *
* License as published by the Free Software Foundation; either                                    *
* version 2.1 of the License, or (at your option) any later version.                              *
*                                                                                                 *
* HPMPC is distributed in the hope that it will be useful,                                        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  *
*                                                                                                 *
* Author: Gianluca Frison, giaf (at) dtu.dk                                                       *
*                                                                                                 *
**************************************************************************************************/

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX




#if ! defined(BLASFEO)
// TODO tri !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void kernel_dgetr_8_lib4(int tri, int kmax, int kna, double *A0, int sda, double *C, int sdc)
	{

	const int bs = 4;
	
	double *A1 = A0 + bs*sda;

	int k;

	__m256d
		v0, v1, v2, v3, v4, v5, v6, v7,
		v8, v9, va, vb, vc, vd, ve, vf;
	
	k = 0;

	if(kmax<kna)
		goto cleanup_loop;

	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			C[0+bs*0] = A0[0+bs*0];
			C[0+bs*1] = A0[1+bs*0];
			C[0+bs*2] = A0[2+bs*0];
			C[0+bs*3] = A0[3+bs*0];

			C[0+bs*4] = A1[0+bs*0];
			C[0+bs*5] = A1[1+bs*0];
			C[0+bs*6] = A1[2+bs*0];
			C[0+bs*7] = A1[3+bs*0];

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
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33

		v8 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*0] ) ), _mm_load_pd( &A1[0+bs*2]) , 0x1 ); // 00 10 02 12
		v9 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*1] ) ), _mm_load_pd( &A1[0+bs*3]) , 0x1 ); // 01 11 03 13
		va = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*0] ) ), _mm_load_pd( &A1[2+bs*2]) , 0x1 ); // 20 30 22 32
		vb = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*1] ) ), _mm_load_pd( &A1[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A1 += 4*bs;

		vc = _mm256_unpacklo_pd( v8, v9 ); // 00 01 02 03
		vd = _mm256_unpackhi_pd( v8, v9 ); // 10 11 12 13
		ve = _mm256_unpacklo_pd( va, vb ); // 20 21 22 23
		vf = _mm256_unpackhi_pd( va, vb ); // 30 31 32 33

		_mm256_store_pd( &C[0+bs*0], v4 );
		_mm256_store_pd( &C[0+bs*1], v5 );
		_mm256_store_pd( &C[0+bs*2], v6 );
		_mm256_store_pd( &C[0+bs*3], v7 );

		_mm256_store_pd( &C[0+bs*4], vc );
		_mm256_store_pd( &C[0+bs*5], vd );
		_mm256_store_pd( &C[0+bs*6], ve );
		_mm256_store_pd( &C[0+bs*7], vf );

		C += sdc*bs;

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*0] ) ), _mm_load_pd( &A0[0+bs*2]) , 0x1 ); // 00 10 02 12
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[0+bs*1] ) ), _mm_load_pd( &A0[0+bs*3]) , 0x1 ); // 01 11 03 13
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*0] ) ), _mm_load_pd( &A0[2+bs*2]) , 0x1 ); // 20 30 22 32
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A0[2+bs*1] ) ), _mm_load_pd( &A0[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A0 += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 ); // 00 01 02 03
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33

		v8 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*0] ) ), _mm_load_pd( &A1[0+bs*2]) , 0x1 ); // 00 10 02 12
		v9 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*1] ) ), _mm_load_pd( &A1[0+bs*3]) , 0x1 ); // 01 11 03 13
		va = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*0] ) ), _mm_load_pd( &A1[2+bs*2]) , 0x1 ); // 20 30 22 32
		vb = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*1] ) ), _mm_load_pd( &A1[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A1 += 4*bs;

		vc = _mm256_unpacklo_pd( v8, v9 ); // 00 01 02 03
		vd = _mm256_unpackhi_pd( v8, v9 ); // 10 11 12 13
		ve = _mm256_unpacklo_pd( va, vb ); // 20 21 22 23
		vf = _mm256_unpackhi_pd( va, vb ); // 30 31 32 33

		_mm256_store_pd( &C[0+bs*0], v4 );
		_mm256_store_pd( &C[0+bs*1], v5 );
		_mm256_store_pd( &C[0+bs*2], v6 );
		_mm256_store_pd( &C[0+bs*3], v7 );

		_mm256_store_pd( &C[0+bs*4], vc );
		_mm256_store_pd( &C[0+bs*5], vd );
		_mm256_store_pd( &C[0+bs*6], ve );
		_mm256_store_pd( &C[0+bs*7], vf );

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
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33

		v8 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*0] ) ), _mm_load_pd( &A1[0+bs*2]) , 0x1 ); // 00 10 02 12
		v9 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[0+bs*1] ) ), _mm_load_pd( &A1[0+bs*3]) , 0x1 ); // 01 11 03 13
		va = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*0] ) ), _mm_load_pd( &A1[2+bs*2]) , 0x1 ); // 20 30 22 32
		vb = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A1[2+bs*1] ) ), _mm_load_pd( &A1[2+bs*3]) , 0x1 ); // 21 31 23 33
		
		A1 += 4*bs;

		vc = _mm256_unpacklo_pd( v8, v9 ); // 00 01 02 03
		vd = _mm256_unpackhi_pd( v8, v9 ); // 10 11 12 13
		ve = _mm256_unpacklo_pd( va, vb ); // 20 21 22 23
		vf = _mm256_unpackhi_pd( va, vb ); // 30 31 32 33

		_mm256_store_pd( &C[0+bs*0], v4 );
		_mm256_store_pd( &C[0+bs*1], v5 );
		_mm256_store_pd( &C[0+bs*2], v6 );
		_mm256_store_pd( &C[0+bs*3], v7 );

		_mm256_store_pd( &C[0+bs*4], vc );
		_mm256_store_pd( &C[0+bs*5], vd );
		_mm256_store_pd( &C[0+bs*6], ve );
		_mm256_store_pd( &C[0+bs*7], vf );

		C += sdc*bs;

		}

	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = A0[0+bs*0];
		C[0+bs*1] = A0[1+bs*0];
		C[0+bs*2] = A0[2+bs*0];
		C[0+bs*3] = A0[3+bs*0];

		C[0+bs*4] = A1[0+bs*0];
		C[0+bs*5] = A1[1+bs*0];
		C[0+bs*6] = A1[2+bs*0];
		C[0+bs*7] = A1[3+bs*0];

		C  += 1;
		A0 += bs;
		A1 += bs;
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_dgetr_4_lib4(int tri, int kmax, int kna, double *A, double *C, int sdc)
	{

	if(tri==1)
		{
		// A is lower triangular, C is upper triangular
		// kmax+1 4-wide + end 3x3 triangle

		kmax += 1;
		}

	const int bs = 4;
	
	__m256d
		v0, v1, v2, v3,
		v4, v5, v6, v7;
	
	int k;

	k = 0;

	if(kmax<kna)
		goto cleanup_loop;

	if(kna>0)
		{
		for( ; k<kna; k++)
			{
			C[0+bs*0] = A[0+bs*0];
			C[0+bs*1] = A[1+bs*0];
			C[0+bs*2] = A[2+bs*0];
			C[0+bs*3] = A[3+bs*0];

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
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33

		_mm256_store_pd( &C[0+bs*0], v4 );
		_mm256_store_pd( &C[0+bs*1], v5 );
		_mm256_store_pd( &C[0+bs*2], v6 );
		_mm256_store_pd( &C[0+bs*3], v7 );

		C += sdc*bs;

		v0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*0] ) ), _mm_load_pd( &A[0+bs*2]) , 0x1 );
		v1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[0+bs*1] ) ), _mm_load_pd( &A[0+bs*3]) , 0x1 );
		v2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*0] ) ), _mm_load_pd( &A[2+bs*2]) , 0x1 );
		v3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( _mm_load_pd( &A[2+bs*1] ) ), _mm_load_pd( &A[2+bs*3]) , 0x1 );
		
		A += 4*bs;

		v4 = _mm256_unpacklo_pd( v0, v1 );
		v5 = _mm256_unpackhi_pd( v0, v1 );
		v6 = _mm256_unpacklo_pd( v2, v3 );
		v7 = _mm256_unpackhi_pd( v2, v3 );

		_mm256_store_pd( &C[0+bs*0], v4 );
		_mm256_store_pd( &C[0+bs*1], v5 );
		_mm256_store_pd( &C[0+bs*2], v6 );
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
		v5 = _mm256_unpackhi_pd( v0, v1 ); // 10 11 12 13
		v6 = _mm256_unpacklo_pd( v2, v3 ); // 20 21 22 23
		v7 = _mm256_unpackhi_pd( v2, v3 ); // 30 31 32 33

		_mm256_store_pd( &C[0+bs*0], v4 );
		_mm256_store_pd( &C[0+bs*1], v5 );
		_mm256_store_pd( &C[0+bs*2], v6 );
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
		C[0+bs*0] = A[0+bs*0];
		C[0+bs*1] = A[1+bs*0];
		C[0+bs*2] = A[2+bs*0];
		C[0+bs*3] = A[3+bs*0];

		C += 1;
		A += bs;
		}

	if(tri==1)
		{
		// end 3x3 triangle
		kna = (bs-(bs-kna+kmax)%bs)%bs;

		if(kna==1)
			{
			C[0+bs*1] = A[1+bs*0];
			C[0+bs*2] = A[2+bs*0];
			C[0+bs*3] = A[3+bs*0];
			C[1+bs*(sdc+1)] = A[2+bs*1];
			C[1+bs*(sdc+2)] = A[3+bs*1];
			C[2+bs*(sdc+2)] = A[3+bs*2];
			}
		else if(kna==2)
			{
			C[0+bs*1] = A[1+bs*0];
			C[0+bs*2] = A[2+bs*0];
			C[0+bs*3] = A[3+bs*0];
			C[1+bs*2] = A[2+bs*1];
			C[1+bs*3] = A[3+bs*1];
			C[2+bs*(sdc+2)] = A[3+bs*2];
			}
		else
			{
			C[0+bs*1] = A[1+bs*0];
			C[0+bs*2] = A[2+bs*0];
			C[0+bs*3] = A[3+bs*0];
			C[1+bs*2] = A[2+bs*1];
			C[1+bs*3] = A[3+bs*1];
			C[2+bs*3] = A[3+bs*2];
			}
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_dgetr_3_lib4(int tri, int kmax, int kna, double *A, double *C, int sdc)
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
			C[0+bs*0] = A[0+bs*0];
			C[0+bs*1] = A[1+bs*0];
			C[0+bs*2] = A[2+bs*0];

			C += 1;
			A += bs;
			}
		C += bs*(sdc-1);
		}
	
	for( ; k<kmax-3; k+=4)
		{
		C[0+bs*0] = A[0+bs*0];
		C[0+bs*1] = A[1+bs*0];
		C[0+bs*2] = A[2+bs*0];

		C[1+bs*0] = A[0+bs*1];
		C[1+bs*1] = A[1+bs*1];
		C[1+bs*2] = A[2+bs*1];

		C[2+bs*0] = A[0+bs*2];
		C[2+bs*1] = A[1+bs*2];
		C[2+bs*2] = A[2+bs*2];

		C[3+bs*0] = A[0+bs*3];
		C[3+bs*1] = A[1+bs*3];
		C[3+bs*2] = A[2+bs*3];

		C += bs*sdc;
		A += bs*bs;
		}
	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = A[0+bs*0];
		C[0+bs*1] = A[1+bs*0];
		C[0+bs*2] = A[2+bs*0];

		C += 1;
		A += bs;
		}

	if(tri==1)
		{
		// end 2x2 triangle
		kna = (bs-(bs-kna+kmax)%bs)%bs;

		if(kna==1)
			{
			C[0+bs*1] = A[1+bs*0];
			C[0+bs*2] = A[2+bs*0];
			C[1+bs*(sdc+1)] = A[2+bs*1];
			}
		else
			{
			C[0+bs*1] = A[1+bs*0];
			C[0+bs*2] = A[2+bs*0];
			C[1+bs*2] = A[2+bs*1];
			}
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_dgetr_2_lib4(int tri, int kmax, int kna, double *A, double *C, int sdc)
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
			C[0+bs*0] = A[0+bs*0];
			C[0+bs*1] = A[1+bs*0];

			C += 1;
			A += bs;
			}
		C += bs*(sdc-1);
		}
	
	for( ; k<kmax-3; k+=4)
		{
		C[0+bs*0] = A[0+bs*0];
		C[0+bs*1] = A[1+bs*0];

		C[1+bs*0] = A[0+bs*1];
		C[1+bs*1] = A[1+bs*1];

		C[2+bs*0] = A[0+bs*2];
		C[2+bs*1] = A[1+bs*2];

		C[3+bs*0] = A[0+bs*3];
		C[3+bs*1] = A[1+bs*3];

		C += bs*sdc;
		A += bs*bs;
		}
	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = A[0+bs*0];
		C[0+bs*1] = A[1+bs*0];

		C += 1;
		A += bs;
		}
	
	if(tri==1)
		{
		// end 1x1 triangle
		C[0+bs*1] = A[1+bs*0];
		}

	}



// transposed of general matrices, read along panels, write across panels
void kernel_dgetr_1_lib4(int tri, int kmax, int kna, double *A, double *C, int sdc)
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
			C[0+bs*0] = A[0+bs*0];

			C += 1;
			A += bs;
			}
		C += bs*(sdc-1);
		}
	
	for( ; k<kmax-3; k+=4)
		{
		C[0+bs*0] = A[0+bs*0];

		C[1+bs*0] = A[0+bs*1];

		C[2+bs*0] = A[0+bs*2];

		C[3+bs*0] = A[0+bs*3];

		C += bs*sdc;
		A += bs*bs;
		}
	
	cleanup_loop:

	for( ; k<kmax; k++)
		{
		C[0+bs*0] = A[0+bs*0];

		C += 1;
		A += bs;
		}

	}
#endif


