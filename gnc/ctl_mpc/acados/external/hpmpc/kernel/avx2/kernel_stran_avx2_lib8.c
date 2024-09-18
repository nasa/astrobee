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




void kernel_stran_8_lib8(int kmax, int kna, float *A, int sda, float *C) // TODO 8 ???
	{
	
	// kmax is at least 4 !!!
	
/*	printf("\n%d %d\n", kmax, kna);*/
	
	int k;

	const int bs = 8;

	__m128
		u0, u1, u2, u3, u4, u5, u6, u7,
		u8, u9, ua, ub, uc, ud, ue, uf;

	__m256
		v0, v1, v2, v3, v4, v5, v6, v7,
		v8, v9, va, vb, vc, vd, ve, vf;

	__m128i
		mask2, mask3;

	mask2 = _mm_set_epi32( 1, 1, -1, -1 );
	mask3 = _mm_set_epi32( 1, -1, -1, -1 );

	k=0;

	if(kna==0)
		{
		
		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm_store_ss( &C[0+bs*0], _mm256_castps256_ps128( v0 ) );
		if(kmax==1)
			return;
		_mm_maskstore_ps( &C[0+bs*1], mask2, _mm256_castps256_ps128( v1 ) );
		if(kmax==2)
			return;
		_mm_maskstore_ps( &C[0+bs*2], mask3, _mm256_castps256_ps128( v2 ) );
		if(kmax==3)
			return;
		_mm_store_ps( &C[0+bs*3], _mm256_castps256_ps128( v3 ) );
		if(kmax==4)
			return;
		_mm_store_ps( &C[0+bs*4], _mm256_extractf128_ps( v0, 0x1 ) );
		_mm_store_ss( &C[4+bs*4], _mm256_extractf128_ps( v4, 0x1 ) );
		if(kmax==5)
			return;
		_mm_store_ps( &C[0+bs*5], _mm256_extractf128_ps( v1, 0x1 ) );
		_mm_maskstore_ps( &C[4+bs*5], mask2, _mm256_extractf128_ps( v5, 0x1 ) );
		if(kmax==6)
			return;
		_mm_store_ps( &C[0+bs*6], _mm256_extractf128_ps( v2, 0x1 ) );
		_mm_maskstore_ps( &C[4+bs*6], mask3, _mm256_extractf128_ps( v6, 0x1 ) );
		if(kmax==7)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;
		
		}
	else if(kna==1)
		{
		
		// top 1x1 triangle
		_mm_store_ss( &C[0+bs*0], _mm_load_ss( &A[0+bs*0] ) );
		
		if(kmax==1)
			return;

		A += 1 + bs*(sda-1);
		C += bs;
		k += 1;

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm_maskstore_ps( &C[0+bs*0], mask2, _mm256_castps256_ps128( v0 ) );
		if(kmax==2)
			return;
		_mm_maskstore_ps( &C[0+bs*1], mask3, _mm256_castps256_ps128( v1 ) );
		if(kmax==3)
			return;
		_mm_store_ps( &C[0+bs*2], _mm256_castps256_ps128( v2 ) );
		if(kmax==4)
			return;
		_mm_store_ps( &C[0+bs*3], _mm256_castps256_ps128( v3 ) );
		_mm_store_ss( &C[4+bs*3], _mm256_castps256_ps128( v7 ) );
		if(kmax==5)
			return;
		_mm_store_ps( &C[0+bs*4], _mm256_extractf128_ps( v0, 0x1 ) );
		_mm_maskstore_ps( &C[4+bs*4], mask2, _mm256_extractf128_ps( v4, 0x1 ) );
		if(kmax==6)
			return;
		_mm_store_ps( &C[0+bs*5], _mm256_extractf128_ps( v1, 0x1 ) );
		_mm_maskstore_ps( &C[4+bs*5], mask3, _mm256_extractf128_ps( v5, 0x1 ) );
		if(kmax==7)
			return;
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
		if(kmax==8)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;

		}
	else if(kna==2)
		{

		// top 2x2 triangle
		_mm_store_ss( &C[0+bs*0], _mm_load_ss( &A[0+bs*0] ) );
		if(kmax==1)
			return;
		_mm_store_ss( &C[0+bs*1], _mm_load_ss( &A[1+bs*0] ) );
		_mm_store_ss( &C[1+bs*1], _mm_load_ss( &A[1+bs*1] ) );

		if(kmax==2)
			return;

		A += 2 + bs*(sda-1);
		C += 2*bs;
		k += 2;

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm_maskstore_ps( &C[0+bs*0], mask3, _mm256_castps256_ps128( v0 ) );
		if(kmax==3)
			return;
		_mm_store_ps( &C[0+bs*1], _mm256_castps256_ps128( v1 ) );
		if(kmax==4)
			return;
		_mm_store_ps( &C[0+bs*2], _mm256_castps256_ps128( v2 ) );
		_mm_store_ss( &C[4+bs*2], _mm256_castps256_ps128( v6 ) );
		if(kmax==5)
			return;
		_mm_store_ps( &C[0+bs*3], _mm256_castps256_ps128( v3 ) );
		_mm_maskstore_ps( &C[4+bs*3], mask2, _mm256_castps256_ps128( v7 ) );
		if(kmax==6)
			return;
		_mm_store_ps( &C[0+bs*4], _mm256_extractf128_ps( v0, 0x1 ) );
		_mm_maskstore_ps( &C[4+bs*4], mask3, _mm256_extractf128_ps( v4, 0x1 ) );
		if(kmax==7)
			return;
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );
		if(kmax==8)
			return;
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
		if(kmax==9)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;

		}
	else if(kna==3)
		{

		// top 2x2 triangle
		_mm_store_ss( &C[0+bs*0], _mm_load_ss( &A[0+bs*0] ) );
		if(kmax==1)
			return;
		_mm_store_ss( &C[0+bs*1], _mm_load_ss( &A[1+bs*0] ) );
		_mm_store_ss( &C[1+bs*1], _mm_load_ss( &A[1+bs*1] ) );
		if(kmax==2)
			return;
		_mm_store_ss( &C[0+bs*2], _mm_load_ss( &A[2+bs*0] ) );
		_mm_store_ss( &C[1+bs*2], _mm_load_ss( &A[2+bs*1] ) );
		_mm_store_ss( &C[2+bs*2], _mm_load_ss( &A[2+bs*2] ) );

		if(kmax==3)
			return;

		A += 3 + bs*(sda-1);
		C += 3*bs;
		k += 3;

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm_store_ps( &C[0+bs*0], _mm256_castps256_ps128( v0 ) );
		if(kmax==4)
			return;
		_mm_store_ps( &C[0+bs*1], _mm256_castps256_ps128( v1 ) );
		_mm_store_ss( &C[4+bs*1], _mm256_castps256_ps128( v5 ) );
		if(kmax==5)
			return;
		_mm_store_ps( &C[0+bs*2], _mm256_castps256_ps128( v2 ) );
		_mm_maskstore_ps( &C[4+bs*2], mask2, _mm256_castps256_ps128( v6 ) );
		if(kmax==6)
			return;
		_mm_store_ps( &C[0+bs*3], _mm256_castps256_ps128( v3 ) );
		_mm_maskstore_ps( &C[4+bs*3], mask3, _mm256_castps256_ps128( v7 ) );
		if(kmax==7)
			return;
		_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );
		if(kmax==8)
			return;
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );
		if(kmax==9)
			return;
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
		if(kmax==10)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;

		}
	else if(kna==4)
		{

		// top 2x2 triangle
		u0 = _mm_load_ps( &A[0+bs*0] ); // 00 10 20 30
		u1 = _mm_load_ps( &A[0+bs*1] ); // 01 11 21 31
		u8 = _mm_unpacklo_ps( u0, u1 ); // 00 01 10 11
		u9 = _mm_unpackhi_ps( u0, u1 ); // 20 21 30 31

		u2 = _mm_load_ps( &A[0+bs*2] ); // 02 12 22 32
		u3 = _mm_load_ps( &A[0+bs*3] ); // 03 13 23 33
		ua = _mm_unpacklo_ps( u2, u3 ); // 02 03 12 13
		ub = _mm_unpackhi_ps( u2, u3 ); // 22 23 32 33

		u0 = _mm_shuffle_ps( u8, ua, 0x44 ); // 00 01 02 03
		_mm_store_ss( &C[0+bs*0], u0 );
		if(kmax==1)
			return;
		u1 = _mm_shuffle_ps( u8, ua, 0xee ); // 10 11 12 13
		_mm_maskstore_ps( &C[0+bs*1], mask2, u1 );
		if(kmax==2)
			return;
		u2 = _mm_shuffle_ps( u9, ub, 0x44 );
		_mm_maskstore_ps( &C[0+bs*2], mask3, u2 );
		if(kmax==3)
			return;
		u3 = _mm_shuffle_ps( u9, ub, 0xee );
		_mm_store_ps( &C[0+bs*3], u3 );

		if(kmax==4)
			return;

		A += 4 + bs*(sda-1);
		C += 4*bs;
		k += 4;

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm_store_ps( &C[0+bs*0], _mm256_castps256_ps128( v0 ) );
		_mm_store_ss( &C[4+bs*0], _mm256_castps256_ps128( v4 ) );
		if(kmax==5)
			return;
		_mm_store_ps( &C[0+bs*1], _mm256_castps256_ps128( v1 ) );
		_mm_maskstore_ps( &C[4+bs*1], mask2, _mm256_castps256_ps128( v5 ) );
		if(kmax==6)
			return;
		_mm_store_ps( &C[0+bs*2], _mm256_castps256_ps128( v2 ) );
		_mm_maskstore_ps( &C[4+bs*2], mask3, _mm256_castps256_ps128( v6 ) );
		if(kmax==7)
			return;
		_mm256_store_ps( &C[0+bs*3], _mm256_permute2f128_ps( v3, v7, 0x20 ) );
		if(kmax==8)
			return;
		_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );
		if(kmax==9)
			return;
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );
		if(kmax==10)
			return;
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
		if(kmax==11)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;

		}
	else if(kna==5)
		{

		// top 2x2 triangle
		_mm_store_ss( &C[0+bs*0], _mm_load_ss( &A[0+bs*0] ) );

		if(kmax==1)
			return;

		A += 1;
		C += 1*bs;
		k += 1;

		// 4x4 triangle
		u0 = _mm_load_ps( &A[0+bs*0] ); // 00 10 20 30
		u1 = _mm_load_ps( &A[0+bs*1] ); // 01 11 21 31
		u8 = _mm_unpacklo_ps( u0, u1 ); // 00 01 10 11
		u9 = _mm_unpackhi_ps( u0, u1 ); // 20 21 30 31

		u2 = _mm_load_ps( &A[0+bs*2] ); // 02 12 22 32
		u3 = _mm_load_ps( &A[0+bs*3] ); // 03 13 23 33
		ua = _mm_unpacklo_ps( u2, u3 ); // 02 03 12 13
		ub = _mm_unpackhi_ps( u2, u3 ); // 22 23 32 33

		u0 = _mm_shuffle_ps( u8, ua, 0x44 ); // 00 01 02 03
		_mm_maskstore_ps( &C[0+bs*0], mask2, u0 );
		if(kmax==2)
			return;
		u1 = _mm_shuffle_ps( u8, ua, 0xee ); // 10 11 12 13
		_mm_maskstore_ps( &C[0+bs*1], mask3, u1 );
		if(kmax==3)
			return;
		u2 = _mm_shuffle_ps( u9, ub, 0x44 );
		_mm_store_ps( &C[0+bs*2], u2 );
		if(kmax==4)
			return;
		u3 = _mm_shuffle_ps( u9, ub, 0xee );
		_mm_store_ps( &C[0+bs*3], u3 );

		_mm_store_ss( &C[4+bs*3], _mm_load_ss( &A[3+bs*4] ) );

		if(kmax==5)
			return;

		A += 4 + bs*(sda-1);
		C += 4*bs;
		k += 4;

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );

		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );

		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );

		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm_store_ps( &C[0+bs*0], _mm256_castps256_ps128( v0 ) );
		_mm_maskstore_ps( &C[4+bs*0], mask2, _mm256_castps256_ps128( v4 ) );
		if(kmax==6)
			return;
		_mm_store_ps( &C[0+bs*1], _mm256_castps256_ps128( v1 ) );
		_mm_maskstore_ps( &C[4+bs*1], mask3, _mm256_castps256_ps128( v5 ) );
		if(kmax==7)
			return;
		_mm256_store_ps( &C[0+bs*2], _mm256_permute2f128_ps( v2, v6, 0x20 ) );
		if(kmax==8)
			return;
		_mm256_store_ps( &C[0+bs*3], _mm256_permute2f128_ps( v3, v7, 0x20 ) );
		if(kmax==9)
			return;
		_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );
		if(kmax==10)
			return;
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );
		if(kmax==11)
			return;
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
		if(kmax==12)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;

		}
	else if(kna==6)
		{

		// top 2x2 triangle
		_mm_store_ss( &C[0+bs*0], _mm_load_ss( &A[0+bs*0] ) );
		if(kmax==1)
			return;
		_mm_store_ss( &C[0+bs*1], _mm_load_ss( &A[1+bs*0] ) );
		_mm_store_ss( &C[1+bs*1], _mm_load_ss( &A[1+bs*1] ) );

		if(kmax==2)
			return;

		A += 2;
		C += 2*bs;
		k += 2;

		// 4x4 triangle
		u0 = _mm_load_ps( &A[0+bs*0] ); // 00 10 20 30
		u1 = _mm_load_ps( &A[0+bs*1] ); // 01 11 21 31
		u8 = _mm_unpacklo_ps( u0, u1 ); // 00 01 10 11
		u9 = _mm_unpackhi_ps( u0, u1 ); // 20 21 30 31

		u2 = _mm_load_ps( &A[0+bs*2] ); // 02 12 22 32
		u3 = _mm_load_ps( &A[0+bs*3] ); // 03 13 23 33
		ua = _mm_unpacklo_ps( u2, u3 ); // 02 03 12 13
		ub = _mm_unpackhi_ps( u2, u3 ); // 22 23 32 33

		u0 = _mm_shuffle_ps( u8, ua, 0x44 ); // 00 01 02 03
		_mm_maskstore_ps( &C[0+bs*0], mask3, u0 );
		if(kmax==3)
			return;
		u1 = _mm_shuffle_ps( u8, ua, 0xee ); // 10 11 12 13
		_mm_store_ps( &C[0+bs*1], u1 );
		if(kmax==4)
			return;
		u2 = _mm_shuffle_ps( u9, ub, 0x44 );
		_mm_store_ps( &C[0+bs*2], u2 );
		_mm_store_ss( &C[4+bs*2], _mm_load_ss( &A[2+bs*4] ) );
		if(kmax==5)
			return;
		u3 = _mm_shuffle_ps( u9, ub, 0xee );
		_mm_store_ps( &C[0+bs*3], u3 );

		_mm_store_ss( &C[4+bs*3], _mm_load_ss( &A[3+bs*4] ) );
		_mm_store_ss( &C[5+bs*3], _mm_load_ss( &A[3+bs*5] ) );

		if(kmax==6)
			return;

		A += 4 + bs*(sda-1);
		C += 4*bs;
		k += 4;

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );

		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );

		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );

		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm_store_ps( &C[0+bs*0], _mm256_castps256_ps128( v0 ) );
		_mm_maskstore_ps( &C[4+bs*0], mask3, _mm256_castps256_ps128( v4 ) );
		if(kmax==7)
			return;
		_mm256_store_ps( &C[0+bs*1], _mm256_permute2f128_ps( v1, v5, 0x20 ) );
		if(kmax==8)
			return;
		_mm256_store_ps( &C[0+bs*2], _mm256_permute2f128_ps( v2, v6, 0x20 ) );
		if(kmax==9)
			return;
		_mm256_store_ps( &C[0+bs*3], _mm256_permute2f128_ps( v3, v7, 0x20 ) );
		if(kmax==10)
			return;
		_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );
		if(kmax==11)
			return;
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );
		if(kmax==12)
			return;
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
		if(kmax==13)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;

		}
	else // if(kna==7)
		{

		// top 3x3 triangle
		_mm_store_ss( &C[0+bs*0], _mm_load_ss( &A[0+bs*0] ) );
		if(kmax==1)
			return;
		_mm_store_ss( &C[0+bs*1], _mm_load_ss( &A[1+bs*0] ) );
		_mm_store_ss( &C[1+bs*1], _mm_load_ss( &A[1+bs*1] ) );
		if(kmax==2)
			return;
		_mm_store_ss( &C[0+bs*2], _mm_load_ss( &A[2+bs*0] ) );
		_mm_store_ss( &C[1+bs*2], _mm_load_ss( &A[2+bs*1] ) );
		_mm_store_ss( &C[2+bs*2], _mm_load_ss( &A[2+bs*2] ) );

		if(kmax==3)
			return;

		A += 3;
		C += 3*bs;
		k += 3;

		// 4x4 triangle
		u0 = _mm_load_ps( &A[0+bs*0] ); // 00 10 20 30
		u1 = _mm_load_ps( &A[0+bs*1] ); // 01 11 21 31
		u8 = _mm_unpacklo_ps( u0, u1 ); // 00 01 10 11
		u9 = _mm_unpackhi_ps( u0, u1 ); // 20 21 30 31

		u2 = _mm_load_ps( &A[0+bs*2] ); // 02 12 22 32
		u3 = _mm_load_ps( &A[0+bs*3] ); // 03 13 23 33
		ua = _mm_unpacklo_ps( u2, u3 ); // 02 03 12 13
		ub = _mm_unpackhi_ps( u2, u3 ); // 22 23 32 33

		u0 = _mm_shuffle_ps( u8, ua, 0x44 ); // 00 01 02 03
		_mm_store_ps( &C[0+bs*0], u0 );
		if(kmax==4)
			return;
		u1 = _mm_shuffle_ps( u8, ua, 0xee ); // 10 11 12 13
		_mm_store_ps( &C[0+bs*1], u1 );
		_mm_store_ss( &C[4+bs*1], _mm_load_ss( &A[1+bs*4] ) );
		if(kmax==5)
			return;
		u2 = _mm_shuffle_ps( u9, ub, 0x44 );
		_mm_store_ps( &C[0+bs*2], u2 );
		_mm_store_ss( &C[4+bs*2], _mm_load_ss( &A[2+bs*4] ) );
		_mm_store_ss( &C[5+bs*2], _mm_load_ss( &A[2+bs*5] ) );
		if(kmax==6)
			return;
		u3 = _mm_shuffle_ps( u9, ub, 0xee );
		_mm_store_ps( &C[0+bs*3], u3 );
		_mm_store_ss( &C[4+bs*3], _mm_load_ss( &A[3+bs*4] ) );
		_mm_store_ss( &C[5+bs*3], _mm_load_ss( &A[3+bs*5] ) );
		_mm_store_ss( &C[6+bs*3], _mm_load_ss( &A[3+bs*6] ) );

		if(kmax==7)
			return;

		A += 4 + bs*(sda-1);
		C += 4*bs;
		k += 4;

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );

		_mm256_store_ps( &C[0+bs*0], _mm256_permute2f128_ps( v0, v4, 0x20 ) );
		if(kmax==8)
			return;
		_mm256_store_ps( &C[0+bs*1], _mm256_permute2f128_ps( v1, v5, 0x20 ) );
		if(kmax==9)
			return;
		_mm256_store_ps( &C[0+bs*2], _mm256_permute2f128_ps( v2, v6, 0x20 ) );
		if(kmax==10)
			return;
		_mm256_store_ps( &C[0+bs*3], _mm256_permute2f128_ps( v3, v7, 0x20 ) );
		if(kmax==11)
			return;
		_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );
		if(kmax==12)
			return;
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );
		if(kmax==13)
			return;
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
		if(kmax==14)
			return;
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		k += bs;

		}

// TODO

	for(; k<kmax-7; k+=8)
		{

		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		_mm256_store_ps( &C[0+bs*0], _mm256_permute2f128_ps( v0, v4, 0x20 ) );
		_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );

		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		_mm256_store_ps( &C[0+bs*1], _mm256_permute2f128_ps( v1, v5, 0x20 ) );
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );

		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		_mm256_store_ps( &C[0+bs*2], _mm256_permute2f128_ps( v2, v6, 0x20 ) );
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );

		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );
		_mm256_store_ps( &C[0+bs*3], _mm256_permute2f128_ps( v3, v7, 0x20 ) );
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		}
	
	if(k==kmax)
		return;

	v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
	v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
	v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
	v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

	v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
	v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
	va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
	vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

	v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
	v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
	vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
	vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

	v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
	v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
	ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
	vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
	
	v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
	v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
	v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
	v5 = _mm256_shuffle_ps( vc, ve, 0xee );
	v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
	v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
	v3 = _mm256_shuffle_ps( v9, vb, 0xee );
	v7 = _mm256_shuffle_ps( vd, vf, 0xee );

	_mm256_store_ps( &C[0+bs*0], _mm256_permute2f128_ps( v0, v4, 0x20 ) );
	if(kmax-k==1)
		return;
	_mm256_store_ps( &C[0+bs*1], _mm256_permute2f128_ps( v1, v5, 0x20 ) );
	if(kmax-k==2)
		return;
	_mm256_store_ps( &C[0+bs*2], _mm256_permute2f128_ps( v2, v6, 0x20 ) );
	if(kmax-k==3)
		return;
	_mm256_store_ps( &C[0+bs*3], _mm256_permute2f128_ps( v3, v7, 0x20 ) );
	if(kmax-k==4)
		return;
	_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );
	if(kmax-k==5)
		return;
	_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );
	if(kmax-k==6)
		return;
	_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );
	if(kmax-k==7)
		return;
	_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

	}



void kernel_stran_8_lib8_test_gather(int kmax, int kna, float *A, int sda, float *C) // TODO 8 ???
	{
	
	// kmax is at least 4 !!!
	
	int k;

	const int bs = 8;
	
/*	__m256i*/
/*		index;*/

	__m256
		v0, v1, v2, v3, v4, v5, v6, v7,
		v8, v9, va, vb, vc, vd, ve, vf;
	
/*	index = _mm256_set_epi32( 7*4, 6*4, 5*4, 4*4, 3*4, 2*4, 1*4, 0*4 );*/

	k=0;

// TODO

	for(; k<kmax-7; k+=8)
		{
		
/*		v0 = _mm256_i32gather_ps( &A[0+bs*0], index, 8 );*/
/*		v1 = _mm256_i32gather_ps( &A[1+bs*0], index, 8 );*/
/*		v2 = _mm256_i32gather_ps( &A[2+bs*0], index, 8 );*/
/*		v3 = _mm256_i32gather_ps( &A[3+bs*0], index, 8 );*/
/*		v4 = _mm256_i32gather_ps( &A[4+bs*0], index, 8 );*/
/*		v5 = _mm256_i32gather_ps( &A[5+bs*0], index, 8 );*/
/*		v6 = _mm256_i32gather_ps( &A[6+bs*0], index, 8 );*/
/*		v7 = _mm256_i32gather_ps( &A[7+bs*0], index, 8 );*/
/*		*/
/*		_mm256_store_ps( &C[0+bs*0], v0 );*/
/*		_mm256_store_ps( &C[0+bs*1], v1 );*/
/*		_mm256_store_ps( &C[0+bs*2], v2 );*/
/*		_mm256_store_ps( &C[0+bs*3], v3 );*/
/*		_mm256_store_ps( &C[0+bs*4], v4 );*/
/*		_mm256_store_ps( &C[0+bs*5], v5 );*/
/*		_mm256_store_ps( &C[0+bs*6], v6 );*/
/*		_mm256_store_ps( &C[0+bs*7], v7 );*/


		v0 = _mm256_load_ps( &A[0+bs*0] ); // 00 10 20 30
		v1 = _mm256_load_ps( &A[0+bs*1] ); // 01 11 21 31
		v8 = _mm256_unpacklo_ps( v0, v1 ); // 00 01 10 11
		v9 = _mm256_unpackhi_ps( v0, v1 ); // 20 21 30 31

		v2 = _mm256_load_ps( &A[0+bs*2] ); // 02 12 22 32
		v3 = _mm256_load_ps( &A[0+bs*3] ); // 03 13 23 33
		va = _mm256_unpacklo_ps( v2, v3 ); // 02 03 12 13
		vb = _mm256_unpackhi_ps( v2, v3 ); // 22 23 32 33

		v4 = _mm256_load_ps( &A[0+bs*4] ); // 04 14 24 34
		v5 = _mm256_load_ps( &A[0+bs*5] ); // 05 15 25 35
		vc = _mm256_unpacklo_ps( v4, v5 ); // 04 05 14 15
		vd = _mm256_unpackhi_ps( v4, v5 ); // 24 25 34 35

		v6 = _mm256_load_ps( &A[0+bs*6] ); // 06 16 26 36
		v7 = _mm256_load_ps( &A[0+bs*7] ); // 07 17 27 37
		ve = _mm256_unpacklo_ps( v6, v7 ); // 06 07 16 17
		vf = _mm256_unpackhi_ps( v6, v7 ); // 26 27 36 37
		
		A += bs*sda;
		
		v0 = _mm256_shuffle_ps( v8, va, 0x44 ); // 00 01 02 03
		v4 = _mm256_shuffle_ps( vc, ve, 0x44 );
		_mm256_store_ps( &C[0+bs*0], _mm256_permute2f128_ps( v0, v4, 0x20 ) );
		_mm256_store_ps( &C[0+bs*4], _mm256_permute2f128_ps( v0, v4, 0x31 ) );

		v1 = _mm256_shuffle_ps( v8, va, 0xee ); // 10 11 12 13
		v5 = _mm256_shuffle_ps( vc, ve, 0xee );
		_mm256_store_ps( &C[0+bs*1], _mm256_permute2f128_ps( v1, v5, 0x20 ) );
		_mm256_store_ps( &C[0+bs*5], _mm256_permute2f128_ps( v1, v5, 0x31 ) );

		v2 = _mm256_shuffle_ps( v9, vb, 0x44 );
		v6 = _mm256_shuffle_ps( vd, vf, 0x44 );
		_mm256_store_ps( &C[0+bs*2], _mm256_permute2f128_ps( v2, v6, 0x20 ) );
		_mm256_store_ps( &C[0+bs*6], _mm256_permute2f128_ps( v2, v6, 0x31 ) );

		v3 = _mm256_shuffle_ps( v9, vb, 0xee );
		v7 = _mm256_shuffle_ps( vd, vf, 0xee );
		_mm256_store_ps( &C[0+bs*3], _mm256_permute2f128_ps( v3, v7, 0x20 ) );
		_mm256_store_ps( &C[0+bs*7], _mm256_permute2f128_ps( v3, v7, 0x31 ) );

		C += bs*bs;

		}
	
	}

