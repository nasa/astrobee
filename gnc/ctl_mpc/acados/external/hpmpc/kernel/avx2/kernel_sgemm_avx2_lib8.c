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



void kernel_sgemm_nt_24x4_lib8(int kmax, float *A0, float *A1, float *A2, float *B, float *C0, float *C1, float *C2, float *D0, float *D1, float *D2, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m256
		a_0, a_8, a_g,
		b_0,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83,
		c_g0, c_g1, c_g2, c_g3;
	
	// prefetch
	a_0 = _mm256_load_ps( &A0[0] );
	a_8 = _mm256_load_ps( &A1[0] );
	a_g = _mm256_load_ps( &A2[0] );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();
	c_g0 = _mm256_setzero_ps();
	c_g1 = _mm256_setzero_ps();
	c_g2 = _mm256_setzero_ps();
	c_g3 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		b_0 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );
		
		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[16] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[16] );

		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[24] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[24] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[24] );

		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[32] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[32] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[32] );


		A0 += 32;
		A1 += 32;
		A2 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );
		
		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[16] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[16] );


		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );

		}

	__m256
		c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, c_a, c_b,
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83,
		d_g0, d_g1, d_g2, d_g3;

	c_0 = _mm256_blend_ps( c_00, c_01, 0x55 );
	c_1 = _mm256_blend_ps( c_01, c_00, 0x55 );
	c_2 = _mm256_blend_ps( c_03, c_02, 0x55 );
	c_3 = _mm256_blend_ps( c_02, c_03, 0x55 );
	c_4 = _mm256_blend_ps( c_80, c_81, 0x55 );
	c_5 = _mm256_blend_ps( c_81, c_80, 0x55 );
	c_6 = _mm256_blend_ps( c_83, c_82, 0x55 );
	c_7 = _mm256_blend_ps( c_82, c_83, 0x55 );
	c_8 = _mm256_blend_ps( c_g0, c_g1, 0x55 );
	c_9 = _mm256_blend_ps( c_g1, c_g0, 0x55 );
	c_a = _mm256_blend_ps( c_g3, c_g2, 0x55 );
	c_b = _mm256_blend_ps( c_g2, c_g3, 0x55 );
	
	c_01 = _mm256_blend_ps( c_0, c_2, 0xcc );
	c_03 = _mm256_blend_ps( c_2, c_0, 0xcc );
	c_00 = _mm256_blend_ps( c_1, c_3, 0xcc );
	c_02 = _mm256_blend_ps( c_3, c_1, 0xcc );
	c_81 = _mm256_blend_ps( c_4, c_6, 0xcc );
	c_83 = _mm256_blend_ps( c_6, c_4, 0xcc );
	c_80 = _mm256_blend_ps( c_5, c_7, 0xcc );
	c_82 = _mm256_blend_ps( c_7, c_5, 0xcc );
	c_g1 = _mm256_blend_ps( c_8, c_a, 0xcc );
	c_g3 = _mm256_blend_ps( c_a, c_8, 0xcc );
	c_g0 = _mm256_blend_ps( c_9, c_b, 0xcc );
	c_g2 = _mm256_blend_ps( c_b, c_9, 0xcc );

	if(alg==0)
		{
		_mm256_store_ps( &D0[0+ldc*0], c_00 );
		_mm256_store_ps( &D0[0+ldc*1], c_01 );
		_mm256_store_ps( &D0[0+ldc*2], c_02 );
		_mm256_store_ps( &D0[0+ldc*3], c_03 );
		_mm256_store_ps( &D1[0+ldc*0], c_80 );
		_mm256_store_ps( &D1[0+ldc*1], c_81 );
		_mm256_store_ps( &D1[0+ldc*2], c_82 );
		_mm256_store_ps( &D1[0+ldc*3], c_83 );
		_mm256_store_ps( &D2[0+ldc*0], c_g0 );
		_mm256_store_ps( &D2[0+ldc*1], c_g1 );
		_mm256_store_ps( &D2[0+ldc*2], c_g2 );
		_mm256_store_ps( &D2[0+ldc*3], c_g3 );
		}
	else
		{
		d_00 = _mm256_load_ps( &C0[0+ldc*0] );
		d_01 = _mm256_load_ps( &C0[0+ldc*1] );
		d_02 = _mm256_load_ps( &C0[0+ldc*2] );
		d_03 = _mm256_load_ps( &C0[0+ldc*3] );
		d_80 = _mm256_load_ps( &C1[0+ldc*0] );
		d_81 = _mm256_load_ps( &C1[0+ldc*1] );
		d_82 = _mm256_load_ps( &C1[0+ldc*2] );
		d_83 = _mm256_load_ps( &C1[0+ldc*3] );
		d_g0 = _mm256_load_ps( &C2[0+ldc*0] );
		d_g1 = _mm256_load_ps( &C2[0+ldc*1] );
		d_g2 = _mm256_load_ps( &C2[0+ldc*2] );
		d_g3 = _mm256_load_ps( &C2[0+ldc*3] );
		
		if(alg==1)
			{
			d_00 = _mm256_add_ps( d_00, c_00 );
			d_01 = _mm256_add_ps( d_01, c_01 );
			d_02 = _mm256_add_ps( d_02, c_02 );
			d_03 = _mm256_add_ps( d_03, c_03 );
			d_80 = _mm256_add_ps( d_80, c_80 );
			d_81 = _mm256_add_ps( d_81, c_81 );
			d_82 = _mm256_add_ps( d_82, c_82 );
			d_83 = _mm256_add_ps( d_83, c_83 );
			d_g0 = _mm256_add_ps( d_g0, c_g0 );
			d_g1 = _mm256_add_ps( d_g1, c_g1 );
			d_g2 = _mm256_add_ps( d_g2, c_g2 );
			d_g3 = _mm256_add_ps( d_g3, c_g3 );
			}
		else // alg == -1
			{
			d_00 = _mm256_sub_ps( d_00, c_00 );
			d_01 = _mm256_sub_ps( d_01, c_01 );
			d_02 = _mm256_sub_ps( d_02, c_02 );
			d_03 = _mm256_sub_ps( d_03, c_03 );
			d_80 = _mm256_sub_ps( d_80, c_80 );
			d_81 = _mm256_sub_ps( d_81, c_81 );
			d_82 = _mm256_sub_ps( d_82, c_82 );
			d_83 = _mm256_sub_ps( d_83, c_83 );
			d_g0 = _mm256_sub_ps( d_g0, c_g0 );
			d_g1 = _mm256_sub_ps( d_g1, c_g1 );
			d_g2 = _mm256_sub_ps( d_g2, c_g2 );
			d_g3 = _mm256_sub_ps( d_g3, c_g3 );
			}

		_mm256_store_ps( &D0[0+ldc*0], d_00 );
		_mm256_store_ps( &D0[0+ldc*1], d_01 );
		_mm256_store_ps( &D0[0+ldc*2], d_02 );
		_mm256_store_ps( &D0[0+ldc*3], d_03 );
		_mm256_store_ps( &D1[0+ldc*0], d_80 );
		_mm256_store_ps( &D1[0+ldc*1], d_81 );
		_mm256_store_ps( &D1[0+ldc*2], d_82 );
		_mm256_store_ps( &D1[0+ldc*3], d_83 );
		_mm256_store_ps( &D2[0+ldc*0], d_g0 );
		_mm256_store_ps( &D2[0+ldc*1], d_g1 );
		_mm256_store_ps( &D2[0+ldc*2], d_g2 );
		_mm256_store_ps( &D2[0+ldc*3], d_g3 );
		}

	}



void kernel_sgemm_nt_24x4_lib8_broadcast(int kmax, float *A0, float *A1, float *A2, float *B, float *C0, float *C1, float *C2, float *D0, float *D1, float *D2, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m256
		a_0, a_8, a_g,
		b_0,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83,
		c_g0, c_g1, c_g2, c_g3;
	
	// prefetch
	a_0 = _mm256_load_ps( &A0[0] );
	a_8 = _mm256_load_ps( &A1[0] );
	a_g = _mm256_load_ps( &A2[0] );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();
	c_g0 = _mm256_setzero_ps();
	c_g1 = _mm256_setzero_ps();
	c_g2 = _mm256_setzero_ps();
	c_g3 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		b_0 = _mm256_broadcast_ss( &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0 = _mm256_broadcast_ss( &B[1] );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0 = _mm256_broadcast_ss( &B[2] );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0 = _mm256_broadcast_ss( &B[3] );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );
		
		
		
		b_0 = _mm256_broadcast_ss( &B[8] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0 = _mm256_broadcast_ss( &B[9] );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0 = _mm256_broadcast_ss( &B[10] );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0 = _mm256_broadcast_ss( &B[11] );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[16] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[16] );

		
		
		b_0 = _mm256_broadcast_ss( &B[16] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0 = _mm256_broadcast_ss( &B[17] );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0 = _mm256_broadcast_ss( &B[18] );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0 = _mm256_broadcast_ss( &B[19] );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[24] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[24] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[24] );

		
		
		b_0 = _mm256_broadcast_ss( &B[24] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0 = _mm256_broadcast_ss( &B[25] );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0 = _mm256_broadcast_ss( &B[26] );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0 = _mm256_broadcast_ss( &B[27] );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[32] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[32] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[32] );


		A0 += 32;
		A1 += 32;
		A2 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		b_0 = _mm256_broadcast_ss( &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0 = _mm256_broadcast_ss( &B[1] );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0 = _mm256_broadcast_ss( &B[2] );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0 = _mm256_broadcast_ss( &B[3] );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );
		
		
		
		b_0 = _mm256_broadcast_ss( &B[8] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0 = _mm256_broadcast_ss( &B[9] );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0 = _mm256_broadcast_ss( &B[10] );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0 = _mm256_broadcast_ss( &B[11] );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[16] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[16] );


		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		b_0 = _mm256_broadcast_ss( &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0 = _mm256_broadcast_ss( &B[1] );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0 = _mm256_broadcast_ss( &B[2] );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0 = _mm256_broadcast_ss( &B[3] );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );

		}

	__m256
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83,
		d_g0, d_g1, d_g2, d_g3;

	if(alg==0)
		{
		_mm256_store_ps( &D0[0+ldc*0], c_00 );
		_mm256_store_ps( &D0[0+ldc*1], c_01 );
		_mm256_store_ps( &D0[0+ldc*2], c_02 );
		_mm256_store_ps( &D0[0+ldc*3], c_03 );
		_mm256_store_ps( &D1[0+ldc*0], c_80 );
		_mm256_store_ps( &D1[0+ldc*1], c_81 );
		_mm256_store_ps( &D1[0+ldc*2], c_82 );
		_mm256_store_ps( &D1[0+ldc*3], c_83 );
		_mm256_store_ps( &D2[0+ldc*0], c_g0 );
		_mm256_store_ps( &D2[0+ldc*1], c_g1 );
		_mm256_store_ps( &D2[0+ldc*2], c_g2 );
		_mm256_store_ps( &D2[0+ldc*3], c_g3 );
		}
	else
		{
		d_00 = _mm256_load_ps( &C0[0+ldc*0] );
		d_01 = _mm256_load_ps( &C0[0+ldc*1] );
		d_02 = _mm256_load_ps( &C0[0+ldc*2] );
		d_03 = _mm256_load_ps( &C0[0+ldc*3] );
		d_80 = _mm256_load_ps( &C1[0+ldc*0] );
		d_81 = _mm256_load_ps( &C1[0+ldc*1] );
		d_82 = _mm256_load_ps( &C1[0+ldc*2] );
		d_83 = _mm256_load_ps( &C1[0+ldc*3] );
		d_g0 = _mm256_load_ps( &C2[0+ldc*0] );
		d_g1 = _mm256_load_ps( &C2[0+ldc*1] );
		d_g2 = _mm256_load_ps( &C2[0+ldc*2] );
		d_g3 = _mm256_load_ps( &C2[0+ldc*3] );
		
		if(alg==1)
			{
			d_00 = _mm256_add_ps( d_00, c_00 );
			d_01 = _mm256_add_ps( d_01, c_01 );
			d_02 = _mm256_add_ps( d_02, c_02 );
			d_03 = _mm256_add_ps( d_03, c_03 );
			d_80 = _mm256_add_ps( d_80, c_80 );
			d_81 = _mm256_add_ps( d_81, c_81 );
			d_82 = _mm256_add_ps( d_82, c_82 );
			d_83 = _mm256_add_ps( d_83, c_83 );
			d_g0 = _mm256_add_ps( d_g0, c_g0 );
			d_g1 = _mm256_add_ps( d_g1, c_g1 );
			d_g2 = _mm256_add_ps( d_g2, c_g2 );
			d_g3 = _mm256_add_ps( d_g3, c_g3 );
			}
		else // alg == -1
			{
			d_00 = _mm256_sub_ps( d_00, c_00 );
			d_01 = _mm256_sub_ps( d_01, c_01 );
			d_02 = _mm256_sub_ps( d_02, c_02 );
			d_03 = _mm256_sub_ps( d_03, c_03 );
			d_80 = _mm256_sub_ps( d_80, c_80 );
			d_81 = _mm256_sub_ps( d_81, c_81 );
			d_82 = _mm256_sub_ps( d_82, c_82 );
			d_83 = _mm256_sub_ps( d_83, c_83 );
			d_g0 = _mm256_sub_ps( d_g0, c_g0 );
			d_g1 = _mm256_sub_ps( d_g1, c_g1 );
			d_g2 = _mm256_sub_ps( d_g2, c_g2 );
			d_g3 = _mm256_sub_ps( d_g3, c_g3 );
			}

		_mm256_store_ps( &D0[0+ldc*0], d_00 );
		_mm256_store_ps( &D0[0+ldc*1], d_01 );
		_mm256_store_ps( &D0[0+ldc*2], d_02 );
		_mm256_store_ps( &D0[0+ldc*3], d_03 );
		_mm256_store_ps( &D1[0+ldc*0], d_80 );
		_mm256_store_ps( &D1[0+ldc*1], d_81 );
		_mm256_store_ps( &D1[0+ldc*2], d_82 );
		_mm256_store_ps( &D1[0+ldc*3], d_83 );
		_mm256_store_ps( &D2[0+ldc*0], d_g0 );
		_mm256_store_ps( &D2[0+ldc*1], d_g1 );
		_mm256_store_ps( &D2[0+ldc*2], d_g2 );
		_mm256_store_ps( &D2[0+ldc*3], d_g3 );
		}

	}



// normal-transposed, 16x4 with data packed in 8
void kernel_sgemm_nt_16x4_lib8_new(int kmax, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m256
		tmp0, tmp1,
		a_07, a_8f, b_0,
		A_07, A_8f, B_0,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83;
	
	// prefetch
	b_0  = _mm256_broadcast_ps( (__m128 *) &B[0] );
	a_07 = _mm256_load_ps( &A0[0] );
	a_8f = _mm256_load_ps( &A1[0] );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{

		c_00 = _mm256_fmadd_ps( a_07, b_0, c_00 );
		B_0  = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_80 = _mm256_fmadd_ps( a_8f, b_0, c_80 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		
		c_01 = _mm256_fmadd_ps( a_07, b_0, c_01 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_81 = _mm256_fmadd_ps( a_8f, b_0, c_81 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		
		c_02 = _mm256_fmadd_ps( a_07, b_0, c_02 );
		A_8f = _mm256_load_ps( &A1[8] );
		c_82 = _mm256_fmadd_ps( a_8f, b_0, c_82 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
	
		c_03 = _mm256_fmadd_ps( a_07, b_0, c_03 );
		c_83 = _mm256_fmadd_ps( a_8f, b_0, c_83 );
		
		
		
		c_00 = _mm256_fmadd_ps( A_07, B_0, c_00 );
		b_0  = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_80 = _mm256_fmadd_ps( A_8f, B_0, c_80 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
		
		c_01 = _mm256_fmadd_ps( A_07, B_0, c_01 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_81 = _mm256_fmadd_ps( A_8f, B_0, c_81 );
		B_0  = _mm256_permute_ps( B_0, 0x4e );
		
		c_02 = _mm256_fmadd_ps( A_07, B_0, c_02 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_82 = _mm256_fmadd_ps( A_8f, B_0, c_82 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
	
		c_03 = _mm256_fmadd_ps( A_07, B_0, c_03 );
		c_83 = _mm256_fmadd_ps( A_8f, B_0, c_83 );

		
		
		c_00 = _mm256_fmadd_ps( a_07, b_0, c_00 );
		B_0  = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_80 = _mm256_fmadd_ps( a_8f, b_0, c_80 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		
		c_01 = _mm256_fmadd_ps( a_07, b_0, c_01 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_81 = _mm256_fmadd_ps( a_8f, b_0, c_81 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		
		c_02 = _mm256_fmadd_ps( a_07, b_0, c_02 );
		A_8f = _mm256_load_ps( &A1[24] );
		c_82 = _mm256_fmadd_ps( a_8f, b_0, c_82 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
	
		c_03 = _mm256_fmadd_ps( a_07, b_0, c_03 );
		c_83 = _mm256_fmadd_ps( a_8f, b_0, c_83 );
		
		
		
		c_00 = _mm256_fmadd_ps( A_07, B_0, c_00 );
		b_0  = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_80 = _mm256_fmadd_ps( A_8f, B_0, c_80 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
		
		c_01 = _mm256_fmadd_ps( A_07, B_0, c_01 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_81 = _mm256_fmadd_ps( A_8f, B_0, c_81 );
		B_0  = _mm256_permute_ps( B_0, 0x4e );
		
		c_02 = _mm256_fmadd_ps( A_07, B_0, c_02 );
		a_8f = _mm256_load_ps( &A1[32] );
		c_82 = _mm256_fmadd_ps( A_8f, B_0, c_82 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
	
		c_03 = _mm256_fmadd_ps( A_07, B_0, c_03 );
		c_83 = _mm256_fmadd_ps( A_8f, B_0, c_83 );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		B_0  = _mm256_broadcast_ps( (__m128 *) &B[8] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_02 = _mm256_add_ps( c_02, tmp0 );
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		c_03 = _mm256_add_ps( c_03, tmp0 );
		c_83 = _mm256_add_ps( c_83, tmp1 );
		
		
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		b_0  = _mm256_broadcast_ps( (__m128 *) &B[16] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
		c_02 = _mm256_add_ps( c_02, tmp0 );
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		c_03 = _mm256_add_ps( c_03, tmp0 );
		c_83 = _mm256_add_ps( c_83, tmp1 );


		A0 += 16;
		A1 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_02 = _mm256_add_ps( c_02, tmp0 );
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		c_03 = _mm256_add_ps( c_03, tmp0 );
		c_83 = _mm256_add_ps( c_83, tmp1 );

		}

	__m256
		c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7,
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83;
	
	c_0 = _mm256_blend_ps( c_00, c_01, 0x55 );
	c_1 = _mm256_blend_ps( c_01, c_00, 0x55 );
	c_2 = _mm256_blend_ps( c_03, c_02, 0x55 );
	c_3 = _mm256_blend_ps( c_02, c_03, 0x55 );
	c_4 = _mm256_blend_ps( c_80, c_81, 0x55 );
	c_5 = _mm256_blend_ps( c_81, c_80, 0x55 );
	c_6 = _mm256_blend_ps( c_83, c_82, 0x55 );
	c_7 = _mm256_blend_ps( c_82, c_83, 0x55 );
	
	c_01 = _mm256_blend_ps( c_0, c_2, 0xcc );
	c_03 = _mm256_blend_ps( c_2, c_0, 0xcc );
	c_00 = _mm256_blend_ps( c_1, c_3, 0xcc );
	c_02 = _mm256_blend_ps( c_3, c_1, 0xcc );
	c_81 = _mm256_blend_ps( c_4, c_6, 0xcc );
	c_83 = _mm256_blend_ps( c_6, c_4, 0xcc );
	c_80 = _mm256_blend_ps( c_5, c_7, 0xcc );
	c_82 = _mm256_blend_ps( c_7, c_5, 0xcc );

	if(alg==0)
		{
		_mm256_store_ps( &D0[0+ldc*0], c_00 );
		_mm256_store_ps( &D0[0+ldc*1], c_01 );
		_mm256_store_ps( &D0[0+ldc*2], c_02 );
		_mm256_store_ps( &D0[0+ldc*3], c_03 );
		_mm256_store_ps( &D1[0+ldc*0], c_80 );
		_mm256_store_ps( &D1[0+ldc*1], c_81 );
		_mm256_store_ps( &D1[0+ldc*2], c_82 );
		_mm256_store_ps( &D1[0+ldc*3], c_83 );
		}
	else
		{
		d_00 = _mm256_load_ps( &C0[0+ldc*0] );
		d_01 = _mm256_load_ps( &C0[0+ldc*1] );
		d_02 = _mm256_load_ps( &C0[0+ldc*2] );
		d_03 = _mm256_load_ps( &C0[0+ldc*3] );
		d_80 = _mm256_load_ps( &C1[0+ldc*0] );
		d_81 = _mm256_load_ps( &C1[0+ldc*1] );
		d_82 = _mm256_load_ps( &C1[0+ldc*2] );
		d_83 = _mm256_load_ps( &C1[0+ldc*3] );
		
		if(alg==1)
			{
			d_00 = _mm256_add_ps( d_00, c_00 );
			d_01 = _mm256_add_ps( d_01, c_01 );
			d_02 = _mm256_add_ps( d_02, c_02 );
			d_03 = _mm256_add_ps( d_03, c_03 );
			d_80 = _mm256_add_ps( d_80, c_80 );
			d_81 = _mm256_add_ps( d_81, c_81 );
			d_82 = _mm256_add_ps( d_82, c_82 );
			d_83 = _mm256_add_ps( d_83, c_83 );
			}
		else // alg == -1
			{
			d_00 = _mm256_sub_ps( d_00, c_00 );
			d_01 = _mm256_sub_ps( d_01, c_01 );
			d_02 = _mm256_sub_ps( d_02, c_02 );
			d_03 = _mm256_sub_ps( d_03, c_03 );
			d_80 = _mm256_sub_ps( d_80, c_80 );
			d_81 = _mm256_sub_ps( d_81, c_81 );
			d_82 = _mm256_sub_ps( d_82, c_82 );
			d_83 = _mm256_sub_ps( d_83, c_83 );
			}

		_mm256_store_ps( &D0[0+ldc*0], d_00 );
		_mm256_store_ps( &D0[0+ldc*1], d_01 );
		_mm256_store_ps( &D0[0+ldc*2], d_02 );
		_mm256_store_ps( &D0[0+ldc*3], d_03 );
		_mm256_store_ps( &D1[0+ldc*0], d_80 );
		_mm256_store_ps( &D1[0+ldc*1], d_81 );
		_mm256_store_ps( &D1[0+ldc*2], d_82 );
		_mm256_store_ps( &D1[0+ldc*3], d_83 );
		}

	}



void kernel_sgemm_nt_16x4_lib8(int kmax, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m256
		temp,
		a_07, a_8f,
		b_03, b_0, b_1, b_2, b_3,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83;
	
	// prefetch
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
	a_07 = _mm256_load_ps( &A0[0] );
	a_8f = _mm256_load_ps( &A1[0] );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_00 = _mm256_fmadd_ps( a_07, b_0, c_00 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_fmadd_ps( a_8f, b_0, c_80 );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_01 = _mm256_fmadd_ps( a_07, b_1, c_01 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_81 = _mm256_fmadd_ps( a_8f, b_1, c_81 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		
		c_02 = _mm256_fmadd_ps( a_07, b_2, c_02 );
		c_82 = _mm256_fmadd_ps( a_8f, b_2, c_82 );
	
		c_03 = _mm256_fmadd_ps( a_07, b_3, c_03 );
		a_07 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8f, b_3, c_83 );
		a_8f = _mm256_load_ps( &A1[8] );
		
		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_00 = _mm256_fmadd_ps( a_07, b_0, c_00 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_fmadd_ps( a_8f, b_0, c_80 );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_01 = _mm256_fmadd_ps( a_07, b_1, c_01 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_81 = _mm256_fmadd_ps( a_8f, b_1, c_81 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		
		c_02 = _mm256_fmadd_ps( a_07, b_2, c_02 );
		c_82 = _mm256_fmadd_ps( a_8f, b_2, c_82 );
	
		c_03 = _mm256_fmadd_ps( a_07, b_3, c_03 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmadd_ps( a_8f, b_3, c_83 );
		a_8f = _mm256_load_ps( &A1[16] );

		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_00 = _mm256_fmadd_ps( a_07, b_0, c_00 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_fmadd_ps( a_8f, b_0, c_80 );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_01 = _mm256_fmadd_ps( a_07, b_1, c_01 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_81 = _mm256_fmadd_ps( a_8f, b_1, c_81 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		
		c_02 = _mm256_fmadd_ps( a_07, b_2, c_02 );
		c_82 = _mm256_fmadd_ps( a_8f, b_2, c_82 );
	
		c_03 = _mm256_fmadd_ps( a_07, b_3, c_03 );
		a_07 = _mm256_load_ps( &A0[24] );
		c_83 = _mm256_fmadd_ps( a_8f, b_3, c_83 );
		a_8f = _mm256_load_ps( &A1[24] );

		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_00 = _mm256_fmadd_ps( a_07, b_0, c_00 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_fmadd_ps( a_8f, b_0, c_80 );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_01 = _mm256_fmadd_ps( a_07, b_1, c_01 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_81 = _mm256_fmadd_ps( a_8f, b_1, c_81 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		
		c_02 = _mm256_fmadd_ps( a_07, b_2, c_02 );
		c_82 = _mm256_fmadd_ps( a_8f, b_2, c_82 );
	
		c_03 = _mm256_fmadd_ps( a_07, b_3, c_03 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_83 = _mm256_fmadd_ps( a_8f, b_3, c_83 );
		a_8f = _mm256_load_ps( &A1[32] );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_3 );
		a_07 = _mm256_load_ps( &A0[8] );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[8] );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_3 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_83 = _mm256_add_ps( c_83, temp );


		A0 += 16;
		A1 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_3 );
		c_83 = _mm256_add_ps( c_83, temp );

		}

	__m256
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83;

	if(alg==0)
		{
		_mm256_store_ps( &D0[0+ldc*0], c_00 );
		_mm256_store_ps( &D0[0+ldc*1], c_01 );
		_mm256_store_ps( &D0[0+ldc*2], c_02 );
		_mm256_store_ps( &D0[0+ldc*3], c_03 );
		_mm256_store_ps( &D1[0+ldc*0], c_80 );
		_mm256_store_ps( &D1[0+ldc*1], c_81 );
		_mm256_store_ps( &D1[0+ldc*2], c_82 );
		_mm256_store_ps( &D1[0+ldc*3], c_83 );
		}
	else
		{
		d_00 = _mm256_load_ps( &C0[0+ldc*0] );
		d_01 = _mm256_load_ps( &C0[0+ldc*1] );
		d_02 = _mm256_load_ps( &C0[0+ldc*2] );
		d_03 = _mm256_load_ps( &C0[0+ldc*3] );
		d_80 = _mm256_load_ps( &C1[0+ldc*0] );
		d_81 = _mm256_load_ps( &C1[0+ldc*1] );
		d_82 = _mm256_load_ps( &C1[0+ldc*2] );
		d_83 = _mm256_load_ps( &C1[0+ldc*3] );
		
		if(alg==1)
			{
			d_00 = _mm256_add_ps( d_00, c_00 );
			d_01 = _mm256_add_ps( d_01, c_01 );
			d_02 = _mm256_add_ps( d_02, c_02 );
			d_03 = _mm256_add_ps( d_03, c_03 );
			d_80 = _mm256_add_ps( d_80, c_80 );
			d_81 = _mm256_add_ps( d_81, c_81 );
			d_82 = _mm256_add_ps( d_82, c_82 );
			d_83 = _mm256_add_ps( d_83, c_83 );
			}
		else // alg == -1
			{
			d_00 = _mm256_sub_ps( d_00, c_00 );
			d_01 = _mm256_sub_ps( d_01, c_01 );
			d_02 = _mm256_sub_ps( d_02, c_02 );
			d_03 = _mm256_sub_ps( d_03, c_03 );
			d_80 = _mm256_sub_ps( d_80, c_80 );
			d_81 = _mm256_sub_ps( d_81, c_81 );
			d_82 = _mm256_sub_ps( d_82, c_82 );
			d_83 = _mm256_sub_ps( d_83, c_83 );
			}

		_mm256_store_ps( &D0[0+ldc*0], d_00 );
		_mm256_store_ps( &D0[0+ldc*1], d_01 );
		_mm256_store_ps( &D0[0+ldc*2], d_02 );
		_mm256_store_ps( &D0[0+ldc*3], d_03 );
		_mm256_store_ps( &D1[0+ldc*0], d_80 );
		_mm256_store_ps( &D1[0+ldc*1], d_81 );
		_mm256_store_ps( &D1[0+ldc*2], d_82 );
		_mm256_store_ps( &D1[0+ldc*3], d_83 );
		}

	}



void kernel_sgemm_nt_12x4_lib8(int kmax, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m128
		tmp0,
		a_47,
		c_40, c_41, c_42, c_43;

	__m256
		temp,
		a_07, a_8f,
		b_03, b_0, b_1, b_2, b_3,
/*		c_00, c_01, c_02, c_03,*/
		c_80, c_81, c_82, c_83;
	
	// prefetch
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
/*	a_07 = _mm256_load_ps( &A0[0] );*/
	a_47 = _mm_load_ps( &A0[4] );
	a_8f = _mm256_load_ps( &A1[0] );

	c_40 = _mm_setzero_ps();
	c_41 = _mm_setzero_ps();
	c_42 = _mm_setzero_ps();
	c_43 = _mm_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*		temp = _mm256_mul_ps( a_07, b_0 );*/
/*		c_00 = _mm256_add_ps( c_00, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_0 ) );
		c_40 = _mm_add_ps( c_40, tmp0 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
/*		temp = _mm256_mul_ps( a_07, b_1 );*/
/*		c_01 = _mm256_add_ps( c_01, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_1 ) );
		c_41 = _mm_add_ps( c_41, tmp0 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		
/*		temp = _mm256_mul_ps( a_07, b_2 );*/
/*		c_02 = _mm256_add_ps( c_02, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_2 ) );
		c_42 = _mm_add_ps( c_42, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
/*		temp = _mm256_mul_ps( a_07, b_3 );*/
/*		a_07 = _mm256_load_ps( &A0[8] );*/
/*		c_03 = _mm256_add_ps( c_03, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_3 ) );
		a_47 = _mm_load_ps( &A0[12] );
		c_43 = _mm_add_ps( c_43, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[8] );
		c_83 = _mm256_add_ps( c_83, temp );
		
		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*		temp = _mm256_mul_ps( a_07, b_0 );*/
/*		c_00 = _mm256_add_ps( c_00, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_0 ) );
		c_40 = _mm_add_ps( c_40, tmp0 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
/*		temp = _mm256_mul_ps( a_07, b_1 );*/
/*		c_01 = _mm256_add_ps( c_01, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_1 ) );
		c_41 = _mm_add_ps( c_41, tmp0 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		
/*		temp = _mm256_mul_ps( a_07, b_2 );*/
/*		c_02 = _mm256_add_ps( c_02, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_2 ) );
		c_42 = _mm_add_ps( c_42, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
/*		temp = _mm256_mul_ps( a_07, b_3 );*/
/*		a_07 = _mm256_load_ps( &A0[16] );*/
/*		c_03 = _mm256_add_ps( c_03, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_3 ) );
		a_47 = _mm_load_ps( &A0[20] );
		c_43 = _mm_add_ps( c_43, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*		temp = _mm256_mul_ps( a_07, b_0 );*/
/*		c_00 = _mm256_add_ps( c_00, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_0 ) );
		c_40 = _mm_add_ps( c_40, tmp0 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
/*		temp = _mm256_mul_ps( a_07, b_1 );*/
/*		c_01 = _mm256_add_ps( c_01, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_1 ) );
		c_41 = _mm_add_ps( c_41, tmp0 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		
/*		temp = _mm256_mul_ps( a_07, b_2 );*/
/*		c_02 = _mm256_add_ps( c_02, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_2 ) );
		c_42 = _mm_add_ps( c_42, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
/*		temp = _mm256_mul_ps( a_07, b_3 );*/
/*		a_07 = _mm256_load_ps( &A0[24] );*/
/*		c_03 = _mm256_add_ps( c_03, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_3 ) );
		a_47 = _mm_load_ps( &A0[28] );
		c_43 = _mm_add_ps( c_43, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[24] );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*		temp = _mm256_mul_ps( a_07, b_0 );*/
/*		c_00 = _mm256_add_ps( c_00, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_0 ) );
		c_40 = _mm_add_ps( c_40, tmp0 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
/*		temp = _mm256_mul_ps( a_07, b_1 );*/
/*		c_01 = _mm256_add_ps( c_01, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_1 ) );
		c_41 = _mm_add_ps( c_41, tmp0 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		
/*		temp = _mm256_mul_ps( a_07, b_2 );*/
/*		c_02 = _mm256_add_ps( c_02, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_2 ) );
		c_42 = _mm_add_ps( c_42, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
/*		temp = _mm256_mul_ps( a_07, b_3 );*/
/*		a_07 = _mm256_load_ps( &A0[32] );*/
/*		c_03 = _mm256_add_ps( c_03, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_3 ) );
		a_47 = _mm_load_ps( &A0[36] );
		c_43 = _mm_add_ps( c_43, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[32] );
		c_83 = _mm256_add_ps( c_83, temp );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*		temp = _mm256_mul_ps( a_07, b_0 );*/
/*		c_00 = _mm256_add_ps( c_00, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_0 ) );
		c_40 = _mm_add_ps( c_40, tmp0 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
/*		temp = _mm256_mul_ps( a_07, b_1 );*/
/*		c_01 = _mm256_add_ps( c_01, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_1 ) );
		c_41 = _mm_add_ps( c_41, tmp0 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		
/*		temp = _mm256_mul_ps( a_07, b_2 );*/
/*		c_02 = _mm256_add_ps( c_02, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_2 ) );
		c_42 = _mm_add_ps( c_42, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
/*		temp = _mm256_mul_ps( a_07, b_3 );*/
/*		a_07 = _mm256_load_ps( &A0[8] );*/
/*		c_03 = _mm256_add_ps( c_03, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_3 ) );
		a_47 = _mm_load_ps( &A0[12] );
		c_43 = _mm_add_ps( c_43, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[8] );
		c_83 = _mm256_add_ps( c_83, temp );
		
		
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*		temp = _mm256_mul_ps( a_07, b_0 );*/
/*		c_00 = _mm256_add_ps( c_00, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_0 ) );
		c_40 = _mm_add_ps( c_40, tmp0 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
/*		temp = _mm256_mul_ps( a_07, b_1 );*/
/*		c_01 = _mm256_add_ps( c_01, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_1 ) );
		c_41 = _mm_add_ps( c_41, tmp0 );
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		
/*		temp = _mm256_mul_ps( a_07, b_2 );*/
/*		c_02 = _mm256_add_ps( c_02, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_2 ) );
		c_42 = _mm_add_ps( c_42, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
/*		temp = _mm256_mul_ps( a_07, b_3 );*/
/*		a_07 = _mm256_load_ps( &A0[16] );*/
/*		c_03 = _mm256_add_ps( c_03, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_3 ) );
		a_47 = _mm_load_ps( &A0[20] );
		c_43 = _mm_add_ps( c_43, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_3 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_83 = _mm256_add_ps( c_83, temp );


		A0 += 16;
		A1 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*		temp = _mm256_mul_ps( a_07, b_0 );*/
/*		c_00 = _mm256_add_ps( c_00, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_0 ) );
		c_40 = _mm_add_ps( c_40, tmp0 );
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_8f, b_0 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
/*		temp = _mm256_mul_ps( a_07, b_1 );*/
/*		c_01 = _mm256_add_ps( c_01, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_1 ) );
		c_41 = _mm_add_ps( c_41, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_1 );
		c_81 = _mm256_add_ps( c_81, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
/*		temp = _mm256_mul_ps( a_07, b_2 );*/
/*		c_02 = _mm256_add_ps( c_02, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_2 ) );
		c_42 = _mm_add_ps( c_42, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_2 );
		c_82 = _mm256_add_ps( c_82, temp );
	
/*		temp = _mm256_mul_ps( a_07, b_3 );*/
/*		c_03 = _mm256_add_ps( c_03, temp );*/
		tmp0 = _mm_mul_ps( a_47, _mm256_castps256_ps128( b_3 ) );
		c_43 = _mm_add_ps( c_43, tmp0 );
		temp = _mm256_mul_ps( a_8f, b_3 );
		c_83 = _mm256_add_ps( c_83, temp );

		}

	__m128
		d_40, d_41, d_42, d_43;

	__m256
/*		d_00, d_01, d_02, d_03,*/
		d_80, d_81, d_82, d_83;

	if(alg==0)
		{
		_mm_store_ps( &D0[4+ldc*0], c_40 );
		_mm_store_ps( &D0[4+ldc*1], c_41 );
		_mm_store_ps( &D0[4+ldc*2], c_42 );
		_mm_store_ps( &D0[4+ldc*3], c_43 );
		_mm256_store_ps( &D1[0+ldc*0], c_80 );
		_mm256_store_ps( &D1[0+ldc*1], c_81 );
		_mm256_store_ps( &D1[0+ldc*2], c_82 );
		_mm256_store_ps( &D1[0+ldc*3], c_83 );
		}
	else
		{
		d_40 = _mm_load_ps( &C0[4+ldc*0] );
		d_41 = _mm_load_ps( &C0[4+ldc*1] );
		d_42 = _mm_load_ps( &C0[4+ldc*2] );
		d_43 = _mm_load_ps( &C0[4+ldc*3] );
		d_80 = _mm256_load_ps( &C1[0+ldc*0] );
		d_81 = _mm256_load_ps( &C1[0+ldc*1] );
		d_82 = _mm256_load_ps( &C1[0+ldc*2] );
		d_83 = _mm256_load_ps( &C1[0+ldc*3] );
		
		if(alg==1)
			{
			d_40 = _mm_add_ps( d_40, c_40 );
			d_41 = _mm_add_ps( d_41, c_41 );
			d_42 = _mm_add_ps( d_42, c_42 );
			d_43 = _mm_add_ps( d_43, c_43 );
			d_80 = _mm256_add_ps( d_80, c_80 );
			d_81 = _mm256_add_ps( d_81, c_81 );
			d_82 = _mm256_add_ps( d_82, c_82 );
			d_83 = _mm256_add_ps( d_83, c_83 );
			}
		else // alg == -1
			{
			d_40 = _mm_sub_ps( d_40, c_40 );
			d_41 = _mm_sub_ps( d_41, c_41 );
			d_42 = _mm_sub_ps( d_42, c_42 );
			d_43 = _mm_sub_ps( d_43, c_43 );
			d_80 = _mm256_sub_ps( d_80, c_80 );
			d_81 = _mm256_sub_ps( d_81, c_81 );
			d_82 = _mm256_sub_ps( d_82, c_82 );
			d_83 = _mm256_sub_ps( d_83, c_83 );
			}

		_mm_store_ps( &D0[4+ldc*0], d_40 );
		_mm_store_ps( &D0[4+ldc*1], d_41 );
		_mm_store_ps( &D0[4+ldc*2], d_42 );
		_mm_store_ps( &D0[4+ldc*3], d_43 );
		_mm256_store_ps( &D1[0+ldc*0], d_80 );
		_mm256_store_ps( &D1[0+ldc*1], d_81 );
		_mm256_store_ps( &D1[0+ldc*2], d_82 );
		_mm256_store_ps( &D1[0+ldc*3], d_83 );
		}

	}



// normal-transposed, 12x4 with data packed in 8
void kernel_sgemm_nt_12x4_lib8_test(int kmax, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m128
		a_tp;

	__m256
		tmp0, tmp1,
		a_07, a_8f, b_0,
		A_07, A_8f, B_0,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83;
	
	// prefetch
	b_0  = _mm256_broadcast_ps( (__m128 *) &B[0] );
/*	a_07 = _mm256_load_ps( &A0[0] );*/
	a_07 = _mm256_broadcast_ps( (__m128 *) &A0[4] );
	a_tp = _mm_permute_ps( _mm256_castps256_ps128( a_07 ), 0x4e );
	a_07 = _mm256_blend_ps( a_07, _mm256_castps128_ps256 (a_tp ), 0x0f );
	a_8f = _mm256_load_ps( &A1[0] );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{
		

		tmp0 = _mm256_mul_ps( a_07, b_0 );
		B_0  = _mm256_broadcast_ps( (__m128 *) &B[8] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
/*		A_07 = _mm256_load_ps( &A0[8] );*/
	A_07 = _mm256_broadcast_ps( (__m128 *) &A0[12] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
	a_tp = _mm_permute_ps( _mm256_castps256_ps128( A_07 ), 0x4e );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
/*		c_02 = _mm256_add_ps( c_02, tmp0 );*/
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
	A_07 = _mm256_blend_ps( A_07, _mm256_castps128_ps256 (a_tp ), 0x0f );
/*		c_03 = _mm256_add_ps( c_03, tmp0 );*/
		c_83 = _mm256_add_ps( c_83, tmp1 );
		
		
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		b_0  = _mm256_broadcast_ps( (__m128 *) &B[16] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
/*		a_07 = _mm256_load_ps( &A0[16] );*/
	a_07 = _mm256_broadcast_ps( (__m128 *) &A0[20] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
	a_tp = _mm_permute_ps( _mm256_castps256_ps128( a_07 ), 0x4e );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
/*		tmp0 = _mm256_mul_ps( A_07, B_0 );*/
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
/*		c_02 = _mm256_add_ps( c_02, tmp0 );*/
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
/*		tmp0 = _mm256_mul_ps( A_07, B_0 );*/
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
	a_07 = _mm256_blend_ps( a_07, _mm256_castps128_ps256 (a_tp ), 0x0f );
/*		c_03 = _mm256_add_ps( c_03, tmp0 );*/
		c_83 = _mm256_add_ps( c_83, tmp1 );

		
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		B_0  = _mm256_broadcast_ps( (__m128 *) &B[24] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
/*		A_07 = _mm256_load_ps( &A0[24] );*/
	A_07 = _mm256_broadcast_ps( (__m128 *) &A0[28] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[24] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
	a_tp = _mm_permute_ps( _mm256_castps256_ps128( A_07 ), 0x4e );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
/*		c_02 = _mm256_add_ps( c_02, tmp0 );*/
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
	A_07 = _mm256_blend_ps( A_07, _mm256_castps128_ps256 (a_tp ), 0x0f );
/*		c_03 = _mm256_add_ps( c_03, tmp0 );*/
		c_83 = _mm256_add_ps( c_83, tmp1 );
		
		
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		b_0  = _mm256_broadcast_ps( (__m128 *) &B[32] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
/*		a_07 = _mm256_load_ps( &A0[32] );*/
	a_07 = _mm256_broadcast_ps( (__m128 *) &A0[36] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		a_8f = _mm256_load_ps( &A1[32] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
	a_tp = _mm_permute_ps( _mm256_castps256_ps128( a_07 ), 0x4e );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
/*		tmp0 = _mm256_mul_ps( A_07, B_0 );*/
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
/*		c_02 = _mm256_add_ps( c_02, tmp0 );*/
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
/*		tmp0 = _mm256_mul_ps( A_07, B_0 );*/
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
	a_07 = _mm256_blend_ps( a_07, _mm256_castps128_ps256 (a_tp ), 0x0f );
/*		c_03 = _mm256_add_ps( c_03, tmp0 );*/
		c_83 = _mm256_add_ps( c_83, tmp1 );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		B_0  = _mm256_broadcast_ps( (__m128 *) &B[8] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
/*		A_07 = _mm256_load_ps( &A0[8] );*/
	A_07 = _mm256_broadcast_ps( (__m128 *) &A0[12] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
	a_tp = _mm_permute_ps( _mm256_castps256_ps128( A_07 ), 0x4e );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
/*		c_02 = _mm256_add_ps( c_02, tmp0 );*/
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
	A_07 = _mm256_blend_ps( A_07, _mm256_castps128_ps256 (a_tp ), 0x0f );
/*		c_03 = _mm256_add_ps( c_03, tmp0 );*/
		c_83 = _mm256_add_ps( c_83, tmp1 );
		
		
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		b_0  = _mm256_broadcast_ps( (__m128 *) &B[16] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
/*		a_07 = _mm256_load_ps( &A0[16] );*/
	a_07 = _mm256_broadcast_ps( (__m128 *) &A0[20] );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( A_07, B_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
	a_tp = _mm_permute_ps( _mm256_castps256_ps128( a_07 ), 0x4e );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
/*		tmp0 = _mm256_mul_ps( A_07, B_0 );*/
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
		B_0  = _mm256_permute_ps( B_0, 0xb1 );
/*		c_02 = _mm256_add_ps( c_02, tmp0 );*/
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
/*		tmp0 = _mm256_mul_ps( A_07, B_0 );*/
		tmp1 = _mm256_mul_ps( A_8f, B_0 );
	a_07 = _mm256_blend_ps( a_07, _mm256_castps128_ps256 (a_tp ), 0x0f );
/*		c_03 = _mm256_add_ps( c_03, tmp0 );*/
		c_83 = _mm256_add_ps( c_83, tmp1 );


		A0 += 16;
		A1 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_00 = _mm256_add_ps( c_00, tmp0 );
		c_80 = _mm256_add_ps( c_80, tmp1 );
		
		tmp0 = _mm256_mul_ps( a_07, b_0 );
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_01 = _mm256_add_ps( c_01, tmp0 );
		c_81 = _mm256_add_ps( c_81, tmp1 );
		
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
/*		c_02 = _mm256_add_ps( c_02, tmp0 );*/
		c_82 = _mm256_add_ps( c_82, tmp1 );
	
/*		tmp0 = _mm256_mul_ps( a_07, b_0 );*/
		tmp1 = _mm256_mul_ps( a_8f, b_0 );
/*		c_03 = _mm256_add_ps( c_03, tmp0 );*/
		c_83 = _mm256_add_ps( c_83, tmp1 );

		}

	__m256
		c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7,
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83;
	
// TODO

	c_0 = _mm256_blend_ps( c_00, c_01, 0x55 );
	c_1 = _mm256_blend_ps( c_01, c_00, 0x55 );
	c_2 = _mm256_blend_ps( c_03, c_02, 0x55 );
	c_3 = _mm256_blend_ps( c_02, c_03, 0x55 );
	c_4 = _mm256_blend_ps( c_80, c_81, 0x55 );
	c_5 = _mm256_blend_ps( c_81, c_80, 0x55 );
	c_6 = _mm256_blend_ps( c_83, c_82, 0x55 );
	c_7 = _mm256_blend_ps( c_82, c_83, 0x55 );
	
	c_01 = _mm256_blend_ps( c_0, c_2, 0xcc );
	c_03 = _mm256_blend_ps( c_2, c_0, 0xcc );
	c_00 = _mm256_blend_ps( c_1, c_3, 0xcc );
	c_02 = _mm256_blend_ps( c_3, c_1, 0xcc );
	c_81 = _mm256_blend_ps( c_4, c_6, 0xcc );
	c_83 = _mm256_blend_ps( c_6, c_4, 0xcc );
	c_80 = _mm256_blend_ps( c_5, c_7, 0xcc );
	c_82 = _mm256_blend_ps( c_7, c_5, 0xcc );

	if(alg==0)
		{
		_mm256_store_ps( &D0[0+ldc*0], c_00 );
		_mm256_store_ps( &D0[0+ldc*1], c_01 );
		_mm256_store_ps( &D0[0+ldc*2], c_02 );
		_mm256_store_ps( &D0[0+ldc*3], c_03 );
		_mm256_store_ps( &D1[0+ldc*0], c_80 );
		_mm256_store_ps( &D1[0+ldc*1], c_81 );
		_mm256_store_ps( &D1[0+ldc*2], c_82 );
		_mm256_store_ps( &D1[0+ldc*3], c_83 );
		}
	else
		{
		d_00 = _mm256_load_ps( &C0[0+ldc*0] );
		d_01 = _mm256_load_ps( &C0[0+ldc*1] );
		d_02 = _mm256_load_ps( &C0[0+ldc*2] );
		d_03 = _mm256_load_ps( &C0[0+ldc*3] );
		d_80 = _mm256_load_ps( &C1[0+ldc*0] );
		d_81 = _mm256_load_ps( &C1[0+ldc*1] );
		d_82 = _mm256_load_ps( &C1[0+ldc*2] );
		d_83 = _mm256_load_ps( &C1[0+ldc*3] );
		
		if(alg==1)
			{
			d_00 = _mm256_add_ps( d_00, c_00 );
			d_01 = _mm256_add_ps( d_01, c_01 );
			d_02 = _mm256_add_ps( d_02, c_02 );
			d_03 = _mm256_add_ps( d_03, c_03 );
			d_80 = _mm256_add_ps( d_80, c_80 );
			d_81 = _mm256_add_ps( d_81, c_81 );
			d_82 = _mm256_add_ps( d_82, c_82 );
			d_83 = _mm256_add_ps( d_83, c_83 );
			}
		else // alg == -1
			{
			d_00 = _mm256_sub_ps( d_00, c_00 );
			d_01 = _mm256_sub_ps( d_01, c_01 );
			d_02 = _mm256_sub_ps( d_02, c_02 );
			d_03 = _mm256_sub_ps( d_03, c_03 );
			d_80 = _mm256_sub_ps( d_80, c_80 );
			d_81 = _mm256_sub_ps( d_81, c_81 );
			d_82 = _mm256_sub_ps( d_82, c_82 );
			d_83 = _mm256_sub_ps( d_83, c_83 );
			}

		_mm256_store_ps( &D0[0+ldc*0], d_00 );
		_mm256_store_ps( &D0[0+ldc*1], d_01 );
		_mm256_store_ps( &D0[0+ldc*2], d_02 );
		_mm256_store_ps( &D0[0+ldc*3], d_03 );
		_mm256_store_ps( &D1[0+ldc*0], d_80 );
		_mm256_store_ps( &D1[0+ldc*1], d_81 );
		_mm256_store_ps( &D1[0+ldc*2], d_82 );
		_mm256_store_ps( &D1[0+ldc*3], d_83 );
		}

	}



// normal-transposed, 8x8 with data packed in 8
void kernel_sgemm_nt_8x8_lib8(int kmax, float *A0, float *B, float *C0, float *D0, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m256
		t_0, t_1,
		a_0, A_0,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7;
	
	// prefetch
	a_0 = _mm256_load_ps( &A0[0] );
	b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[0] ) );
	b_1 = _mm256_permute_ps( b_0, 0x4e );

	c_0 = _mm256_setzero_ps();
	c_1 = _mm256_setzero_ps();
	c_2 = _mm256_setzero_ps();
	c_3 = _mm256_setzero_ps();
	c_4 = _mm256_setzero_ps();
	c_5 = _mm256_setzero_ps();
	c_6 = _mm256_setzero_ps();
	c_7 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{
		
		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[0] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_0 = _mm256_add_ps( c_0, t_0 );
		c_2 = _mm256_add_ps( c_2, t_1 );

		A_0 = _mm256_load_ps( &A0[8] );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_4 = _mm256_add_ps( c_4, t_0 );
		c_6 = _mm256_add_ps( c_6, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );

		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_5 = _mm256_add_ps( c_5, t_0 );
		c_7 = _mm256_add_ps( c_7, t_1 );



		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_0 = _mm256_add_ps( c_0, t_0 );
		c_2 = _mm256_add_ps( c_2, t_1 );

		a_0 = _mm256_load_ps( &A0[16] );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( A_0, b_2 );
		t_1 = _mm256_mul_ps( A_0, b_3 );
		c_4 = _mm256_add_ps( c_4, t_0 );
		c_6 = _mm256_add_ps( c_6, t_1 );

		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[16] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );

		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( A_0, b_2 );
		t_1 = _mm256_mul_ps( A_0, b_3 );
		c_5 = _mm256_add_ps( c_5, t_0 );
		c_7 = _mm256_add_ps( c_7, t_1 );


		
		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[16] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_0 = _mm256_add_ps( c_0, t_0 );
		c_2 = _mm256_add_ps( c_2, t_1 );

		A_0 = _mm256_load_ps( &A0[24] );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_4 = _mm256_add_ps( c_4, t_0 );
		c_6 = _mm256_add_ps( c_6, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[24] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );

		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_5 = _mm256_add_ps( c_5, t_0 );
		c_7 = _mm256_add_ps( c_7, t_1 );



		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[24] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_0 = _mm256_add_ps( c_0, t_0 );
		c_2 = _mm256_add_ps( c_2, t_1 );

		a_0 = _mm256_load_ps( &A0[32] );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( A_0, b_2 );
		t_1 = _mm256_mul_ps( A_0, b_3 );
		c_4 = _mm256_add_ps( c_4, t_0 );
		c_6 = _mm256_add_ps( c_6, t_1 );

		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[32] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );

		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( A_0, b_2 );
		t_1 = _mm256_mul_ps( A_0, b_3 );
		c_5 = _mm256_add_ps( c_5, t_0 );
		c_7 = _mm256_add_ps( c_7, t_1 );


		A0 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[0] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_0 = _mm256_add_ps( c_0, t_0 );
		c_2 = _mm256_add_ps( c_2, t_1 );

		A_0 = _mm256_load_ps( &A0[8] );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_4 = _mm256_add_ps( c_4, t_0 );
		c_6 = _mm256_add_ps( c_6, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );

		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_5 = _mm256_add_ps( c_5, t_0 );
		c_7 = _mm256_add_ps( c_7, t_1 );



		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_0 = _mm256_add_ps( c_0, t_0 );
		c_2 = _mm256_add_ps( c_2, t_1 );

		a_0 = _mm256_load_ps( &A0[16] );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( A_0, b_2 );
		t_1 = _mm256_mul_ps( A_0, b_3 );
		c_4 = _mm256_add_ps( c_4, t_0 );
		c_6 = _mm256_add_ps( c_6, t_1 );

		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[16] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );

		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( A_0, b_2 );
		t_1 = _mm256_mul_ps( A_0, b_3 );
		c_5 = _mm256_add_ps( c_5, t_0 );
		c_7 = _mm256_add_ps( c_7, t_1 );


		A0 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[0] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_0 = _mm256_add_ps( c_0, t_0 );
		c_2 = _mm256_add_ps( c_2, t_1 );

/*		A_0 = _mm256_load_ps( &A0[8] );*/
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_4 = _mm256_add_ps( c_4, t_0 );
		c_6 = _mm256_add_ps( c_6, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_2 = _mm256_permute2f128_ps( b_0, b_0, 0x03 );
/*		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[8] ) );*/
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_3 = _mm256_permute2f128_ps( b_1, b_1, 0x03 );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );

		b_1 = _mm256_permute_ps( b_0, 0x4e );
		t_0 = _mm256_mul_ps( a_0, b_2 );
		t_1 = _mm256_mul_ps( a_0, b_3 );
		c_5 = _mm256_add_ps( c_5, t_0 );
		c_7 = _mm256_add_ps( c_7, t_1 );
		
		}

	__m256
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7;
	
	d_0 = _mm256_blend_ps( c_0, c_2, 0xcc );
	d_2 = _mm256_blend_ps( c_0, c_2, 0x33 );
	d_4 = _mm256_blend_ps( c_4, c_6, 0xcc );
	d_6 = _mm256_blend_ps( c_4, c_6, 0x33 );
	d_1 = _mm256_blend_ps( c_1, c_3, 0xcc );
	d_3 = _mm256_blend_ps( c_1, c_3, 0x33 );
	d_5 = _mm256_blend_ps( c_5, c_7, 0xcc );
	d_7 = _mm256_blend_ps( c_5, c_7, 0x33 );

	c_0 = _mm256_blend_ps( d_0, d_4, 0xf0 );
	c_4 = _mm256_blend_ps( d_0, d_4, 0x0f );
	c_2 = _mm256_blend_ps( d_2, d_6, 0xf0 );
	c_6 = _mm256_blend_ps( d_2, d_6, 0x0f );
	c_1 = _mm256_blend_ps( d_1, d_5, 0xf0 );
	c_5 = _mm256_blend_ps( d_1, d_5, 0x0f );
	c_3 = _mm256_blend_ps( d_3, d_7, 0xf0 );
	c_7 = _mm256_blend_ps( d_3, d_7, 0x0f );

	if(alg==0)
		{
		_mm256_store_ps( &D0[0+ldc*0], c_0 );
		_mm256_store_ps( &D0[0+ldc*1], c_1 );
		_mm256_store_ps( &D0[0+ldc*2], c_2 );
		_mm256_store_ps( &D0[0+ldc*3], c_3 );
		_mm256_store_ps( &D0[0+ldc*4], c_4 );
		_mm256_store_ps( &D0[0+ldc*5], c_5 );
		_mm256_store_ps( &D0[0+ldc*6], c_6 );
		_mm256_store_ps( &D0[0+ldc*7], c_7 );
		}
	else
		{
		d_0 = _mm256_load_ps( &C0[0+ldc*0] );
		d_1 = _mm256_load_ps( &C0[0+ldc*1] );
		d_2 = _mm256_load_ps( &C0[0+ldc*2] );
		d_3 = _mm256_load_ps( &C0[0+ldc*3] );
		d_4 = _mm256_load_ps( &C0[0+ldc*4] );
		d_5 = _mm256_load_ps( &C0[0+ldc*5] );
		d_6 = _mm256_load_ps( &C0[0+ldc*6] );
		d_7 = _mm256_load_ps( &C0[0+ldc*7] );
		
		if(alg==1)
			{
			d_0 = _mm256_add_ps( d_0, c_0 );
			d_1 = _mm256_add_ps( d_1, c_1 );
			d_2 = _mm256_add_ps( d_2, c_2 );
			d_3 = _mm256_add_ps( d_3, c_3 );
			d_4 = _mm256_add_ps( d_4, c_4 );
			d_5 = _mm256_add_ps( d_5, c_5 );
			d_6 = _mm256_add_ps( d_6, c_6 );
			d_7 = _mm256_add_ps( d_7, c_7 );
			}
		else // alg == -1
			{
			d_0 = _mm256_sub_ps( d_0, c_0 );
			d_1 = _mm256_sub_ps( d_1, c_1 );
			d_2 = _mm256_sub_ps( d_2, c_2 );
			d_3 = _mm256_sub_ps( d_3, c_3 );
			d_4 = _mm256_sub_ps( d_4, c_4 );
			d_5 = _mm256_sub_ps( d_5, c_5 );
			d_6 = _mm256_sub_ps( d_6, c_6 );
			d_7 = _mm256_sub_ps( d_7, c_7 );
			}

		_mm256_store_ps( &D0[0+ldc*0], d_0 );
		_mm256_store_ps( &D0[0+ldc*1], d_1 );
		_mm256_store_ps( &D0[0+ldc*2], d_2 );
		_mm256_store_ps( &D0[0+ldc*3], d_3 );
		_mm256_store_ps( &D0[0+ldc*4], d_4 );
		_mm256_store_ps( &D0[0+ldc*5], d_5 );
		_mm256_store_ps( &D0[0+ldc*6], d_6 );
		_mm256_store_ps( &D0[0+ldc*7], d_7 );
		}

	}



// normal-transposed, 8x4 with data packed in 8
void kernel_sgemm_nt_8x4_lib8(int kmax, float *A0, float *B, float *C0, float *D0, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m256
		temp,
		a_07, A_07,
		b_03, B_03,
		b_0, b_1, b_2, b_3,
		c_00, c_01, c_02, c_03;
	
	// prefetch
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
	a_07 = _mm256_load_ps( &A0[0] );
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-7; k+=8)
		{
		
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		A_07 = _mm256_load_ps( &A0[8] );
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		a_07 = _mm256_load_ps( &A0[16] );
		
		b_1 = _mm256_shuffle_ps( B_03, B_03, 85 );
		temp = _mm256_mul_ps( A_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( B_03, B_03, 170 );
		temp = _mm256_mul_ps( A_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( B_03, B_03, 255 );
		temp = _mm256_mul_ps( A_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( A_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );



		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		A_07 = _mm256_load_ps( &A0[24] );
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		a_07 = _mm256_load_ps( &A0[32] );
		
		b_1 = _mm256_shuffle_ps( B_03, B_03, 85 );
		temp = _mm256_mul_ps( A_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( B_03, B_03, 170 );
		temp = _mm256_mul_ps( A_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( B_03, B_03, 255 );
		temp = _mm256_mul_ps( A_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( A_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		A0 += 32;
		B  += 32;


		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		A_07 = _mm256_load_ps( &A0[8] );
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		a_07 = _mm256_load_ps( &A0[16] );
		
		b_1 = _mm256_shuffle_ps( B_03, B_03, 85 );
		temp = _mm256_mul_ps( A_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( B_03, B_03, 170 );
		temp = _mm256_mul_ps( A_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( B_03, B_03, 255 );
		temp = _mm256_mul_ps( A_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( A_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );



		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		A_07 = _mm256_load_ps( &A0[24] );
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		a_07 = _mm256_load_ps( &A0[32] );
		
		b_1 = _mm256_shuffle_ps( B_03, B_03, 85 );
		temp = _mm256_mul_ps( A_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( B_03, B_03, 170 );
		temp = _mm256_mul_ps( A_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( B_03, B_03, 255 );
		temp = _mm256_mul_ps( A_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( A_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		A0 += 32;
		B  += 32;

		}
	for(; k<kmax-3; k+=4)
		{
		
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		A_07 = _mm256_load_ps( &A0[8] );
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		a_07 = _mm256_load_ps( &A0[16] );
		
		b_1 = _mm256_shuffle_ps( B_03, B_03, 85 );
		temp = _mm256_mul_ps( A_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( B_03, B_03, 170 );
		temp = _mm256_mul_ps( A_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( B_03, B_03, 255 );
		temp = _mm256_mul_ps( A_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( A_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );



		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		A_07 = _mm256_load_ps( &A0[24] );
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		a_07 = _mm256_load_ps( &A0[32] );
		
		b_1 = _mm256_shuffle_ps( B_03, B_03, 85 );
		temp = _mm256_mul_ps( A_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( B_03, B_03, 170 );
		temp = _mm256_mul_ps( A_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( B_03, B_03, 255 );
		temp = _mm256_mul_ps( A_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( A_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		A0 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		A_07 = _mm256_load_ps( &A0[8] );
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );


		
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		a_07 = _mm256_load_ps( &A0[16] );
		
		b_1 = _mm256_shuffle_ps( B_03, B_03, 85 );
		temp = _mm256_mul_ps( A_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( B_03, B_03, 170 );
		temp = _mm256_mul_ps( A_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( B_03, B_03, 255 );
		temp = _mm256_mul_ps( A_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		temp = _mm256_mul_ps( A_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );

		A0 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
/*		b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );*/
/*		a_07 = _mm256_load_ps( &A0[0] );*/
		
		b_1 = _mm256_shuffle_ps( b_03, b_03, 85 );
		temp = _mm256_mul_ps( a_07, b_0 );
		c_00 = _mm256_add_ps( c_00, temp );
		
		b_2 = _mm256_shuffle_ps( b_03, b_03, 170 );
		temp = _mm256_mul_ps( a_07, b_1 );
		c_01 = _mm256_add_ps( c_01, temp );
		
		b_3 = _mm256_shuffle_ps( b_03, b_03, 255 );
		temp = _mm256_mul_ps( a_07, b_2 );
		c_02 = _mm256_add_ps( c_02, temp );
	
/*		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );*/
		temp = _mm256_mul_ps( a_07, b_3 );
		c_03 = _mm256_add_ps( c_03, temp );

/*		A0 += 8;*/
/*		B  += 8;*/
		
		}

	__m256
		d_00, d_01, d_02, d_03;

	if(alg==0)
		{
		_mm256_store_ps( &D0[0+ldc*0], c_00 );
		_mm256_store_ps( &D0[0+ldc*1], c_01 );
		_mm256_store_ps( &D0[0+ldc*2], c_02 );
		_mm256_store_ps( &D0[0+ldc*3], c_03 );
		}
	else
		{
		d_00 = _mm256_load_ps( &C0[0+ldc*0] );
		d_01 = _mm256_load_ps( &C0[0+ldc*1] );
		d_02 = _mm256_load_ps( &C0[0+ldc*2] );
		d_03 = _mm256_load_ps( &C0[0+ldc*3] );
		
		if(alg==1)
			{
			d_00 = _mm256_add_ps( d_00, c_00 );
			d_01 = _mm256_add_ps( d_01, c_01 );
			d_02 = _mm256_add_ps( d_02, c_02 );
			d_03 = _mm256_add_ps( d_03, c_03 );
			}
		else // alg == -1
			{
			d_00 = _mm256_sub_ps( d_00, c_00 );
			d_01 = _mm256_sub_ps( d_01, c_01 );
			d_02 = _mm256_sub_ps( d_02, c_02 );
			d_03 = _mm256_sub_ps( d_03, c_03 );
			}

		_mm256_store_ps( &D0[0+ldc*0], d_00 );
		_mm256_store_ps( &D0[0+ldc*1], d_01 );
		_mm256_store_ps( &D0[0+ldc*2], d_02 );
		_mm256_store_ps( &D0[0+ldc*3], d_03 );
		}

	}



// normal-transposed, 4x8 with data packed in 8
void kernel_sgemm_nt_4x8_lib8(int kmax, float *A0, float *B, float *C0, float *D0, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m256
		t_0, t_1,
		a_0, A_0,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
	
	// prefetch
	a_0 = _mm256_broadcast_ps( (__m128 *) &A0[0] );
	b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[0] ) );
	b_1 = _mm256_permute_ps( b_0, 0x4e );

	c_0 = _mm256_setzero_ps();
	c_1 = _mm256_setzero_ps();
	c_2 = _mm256_setzero_ps();
	c_3 = _mm256_setzero_ps();

	k = 0;
	for(; k<kmax-3; k+=4)
		{
		
		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[0] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_0 = _mm256_add_ps( c_0, t_0 );
		A_0 = _mm256_broadcast_ps( (__m128*) &A0[8] );
		c_2 = _mm256_add_ps( c_2, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );




		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_0 = _mm256_add_ps( c_0, t_0 );
		a_0 = _mm256_broadcast_ps( (__m128*) &A0[16] );
		c_2 = _mm256_add_ps( c_2, t_1 );

		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[16] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );


		
		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[16] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_0 = _mm256_add_ps( c_0, t_0 );
		A_0 = _mm256_broadcast_ps( (__m128*) &A0[24] );
		c_2 = _mm256_add_ps( c_2, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[24] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );



		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[24] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_0 = _mm256_add_ps( c_0, t_0 );
		a_0 = _mm256_broadcast_ps( (__m128*) &A0[32] );
		c_2 = _mm256_add_ps( c_2, t_1 );

		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[32] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );


		A0 += 32;
		B  += 32;

		}
	if(kmax%4>=2)
		{
		
		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[0] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_0 = _mm256_add_ps( c_0, t_0 );
		A_0 = _mm256_broadcast_ps( (__m128*) &A0[8] );
		c_2 = _mm256_add_ps( c_2, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );



		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[8] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_0 = _mm256_add_ps( c_0, t_0 );
		a_0 = _mm256_broadcast_ps( (__m128*) &A0[16] );
		c_2 = _mm256_add_ps( c_2, t_1 );

		t_0 = _mm256_mul_ps( A_0, b_0 );
		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[16] ) );
		t_1 = _mm256_mul_ps( A_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );


		A0 += 16;
		B  += 16;
		
		}
	if(kmax%2==1)
		{
		
		t_0 = _mm256_mul_ps( a_0, b_0 );
/*		b_0 = _mm256_movehdup_ps( _mm256_load_ps( &B[0] ) );*/
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_0 = _mm256_add_ps( c_0, t_0 );
/*		A_0 = _mm256_broadcast_ps( (__m128*) &A0[8] );*/
		c_2 = _mm256_add_ps( c_2, t_1 );

		t_0 = _mm256_mul_ps( a_0, b_0 );
/*		b_0 = _mm256_moveldup_ps( _mm256_load_ps( &B[8] ) );*/
		t_1 = _mm256_mul_ps( a_0, b_1 );
		b_1 = _mm256_permute_ps( b_0, 0x4e );
		c_1 = _mm256_add_ps( c_1, t_0 );
		c_3 = _mm256_add_ps( c_3, t_1 );
		
		}

	__m256
		r_0, r_1, r_2, r_3;

	__m128
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		e_0, e_1, e_2, e_3,
		e_4, e_5, e_6, e_7;
	
	r_0 = _mm256_blend_ps( c_0, c_2, 0xcc );
	r_2 = _mm256_blend_ps( c_0, c_2, 0x33 );
	r_1 = _mm256_blend_ps( c_1, c_3, 0xcc );
	r_3 = _mm256_blend_ps( c_1, c_3, 0x33 );
	
	e_4 = _mm256_extractf128_ps( r_0, 0x1 );
	e_0 = _mm256_castps256_ps128( r_0 );
	e_6 = _mm256_extractf128_ps( r_2, 0x1 );
	e_2 = _mm256_castps256_ps128( r_2 );
	e_5 = _mm256_extractf128_ps( r_1, 0x1 );
	e_1 = _mm256_castps256_ps128( r_1 );
	e_7 = _mm256_extractf128_ps( r_3, 0x1 );
	e_3 = _mm256_castps256_ps128( r_3 );

	if(alg==0)
		{
		_mm_store_ps( &D0[0+ldc*0], e_0 );
		_mm_store_ps( &D0[0+ldc*1], e_1 );
		_mm_store_ps( &D0[0+ldc*2], e_2 );
		_mm_store_ps( &D0[0+ldc*3], e_3 );
		_mm_store_ps( &D0[0+ldc*4], e_4 );
		_mm_store_ps( &D0[0+ldc*5], e_5 );
		_mm_store_ps( &D0[0+ldc*6], e_6 );
		_mm_store_ps( &D0[0+ldc*7], e_7 );
		}
	else
		{
		d_0 = _mm_load_ps( &C0[0+ldc*0] );
		d_1 = _mm_load_ps( &C0[0+ldc*1] );
		d_2 = _mm_load_ps( &C0[0+ldc*2] );
		d_3 = _mm_load_ps( &C0[0+ldc*3] );
		d_4 = _mm_load_ps( &C0[0+ldc*4] );
		d_5 = _mm_load_ps( &C0[0+ldc*5] );
		d_6 = _mm_load_ps( &C0[0+ldc*6] );
		d_7 = _mm_load_ps( &C0[0+ldc*7] );
		
		if(alg==1)
			{
			d_0 = _mm_add_ps( d_0, e_0 );
			d_1 = _mm_add_ps( d_1, e_1 );
			d_2 = _mm_add_ps( d_2, e_2 );
			d_3 = _mm_add_ps( d_3, e_3 );
			d_4 = _mm_add_ps( d_4, e_4 );
			d_5 = _mm_add_ps( d_5, e_5 );
			d_6 = _mm_add_ps( d_6, e_6 );
			d_7 = _mm_add_ps( d_7, e_7 );
			}
		else // alg == -1
			{
			d_0 = _mm_sub_ps( d_0, e_0 );
			d_1 = _mm_sub_ps( d_1, e_1 );
			d_2 = _mm_sub_ps( d_2, e_2 );
			d_3 = _mm_sub_ps( d_3, e_3 );
			d_4 = _mm_sub_ps( d_4, e_4 );
			d_5 = _mm_sub_ps( d_5, e_5 );
			d_6 = _mm_sub_ps( d_6, e_6 );
			d_7 = _mm_sub_ps( d_7, e_7 );
			}

		_mm_store_ps( &D0[0+ldc*0], d_0 );
		_mm_store_ps( &D0[0+ldc*1], d_1 );
		_mm_store_ps( &D0[0+ldc*2], d_2 );
		_mm_store_ps( &D0[0+ldc*3], d_3 );
		_mm_store_ps( &D0[0+ldc*4], d_4 );
		_mm_store_ps( &D0[0+ldc*5], d_5 );
		_mm_store_ps( &D0[0+ldc*6], d_6 );
		_mm_store_ps( &D0[0+ldc*7], d_7 );
		}

	}



// normal-transposed, 4x4 with data packed in 8
void kernel_sgemm_nt_4x4_lib8(int kmax, float *A, float *B, float *C, float *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int ldc = 8;

	int k;
	
	__m128
		c_03_0, c_03_1, c_03_2, c_03_3,
		a_03,
		b_0, b_1; 
	
	c_03_0 = _mm_setzero_ps();
	c_03_1 = _mm_setzero_ps();
	c_03_2 = _mm_setzero_ps();
	c_03_3 = _mm_setzero_ps();

	for(k=0; k<kmax-3; k+=4)
		{
		
		b_0 = _mm_load_ps( &B[0] );
		
		a_03 = _mm_load_ps( &A[0] );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 0 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_0 = _mm_add_ps( c_03_0, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 85 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_1 = _mm_add_ps( c_03_1, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 170 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_2 = _mm_add_ps( c_03_2, b_1 );
	
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 255 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_3 = _mm_add_ps( c_03_3, b_1 );



		b_0 = _mm_load_ps( &B[8] );
		
		a_03 = _mm_load_ps( &A[8] );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 0 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_0 = _mm_add_ps( c_03_0, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 85 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_1 = _mm_add_ps( c_03_1, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 170 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_2 = _mm_add_ps( c_03_2, b_1 );
	
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 255 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_3 = _mm_add_ps( c_03_3, b_1 );


		
		b_0 = _mm_load_ps( &B[16] );
		
		a_03 = _mm_load_ps( &A[16] );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 0 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_0 = _mm_add_ps( c_03_0, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 85 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_1 = _mm_add_ps( c_03_1, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 170 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_2 = _mm_add_ps( c_03_2, b_1 );
	
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 255 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_3 = _mm_add_ps( c_03_3, b_1 );



		b_0 = _mm_load_ps( &B[24] );
		
		a_03 = _mm_load_ps( &A[24] );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 0 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_0 = _mm_add_ps( c_03_0, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 85 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_1 = _mm_add_ps( c_03_1, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 170 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_2 = _mm_add_ps( c_03_2, b_1 );
	
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 255 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_3 = _mm_add_ps( c_03_3, b_1 );


		A += 32;
		B += 32;

		}
	
	for(; k<kmax; k++)
		{
		
		b_0 = _mm_load_ps( &B[0] );
		
		a_03 = _mm_load_ps( &A[0] );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 0 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_0 = _mm_add_ps( c_03_0, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 85 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_1 = _mm_add_ps( c_03_1, b_1 );
		
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 170 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_2 = _mm_add_ps( c_03_2, b_1 );
	
		b_1 = b_0;
		b_1 = _mm_shuffle_ps( b_1, b_1, 255 );
		b_1 = _mm_mul_ps( a_03, b_1 );
		c_03_3 = _mm_add_ps( c_03_3, b_1 );
		
		A += 8;
		B += 8;
		
		}

	__m128
		d_03_0, d_03_1, d_03_2, d_03_3;

	if(alg==0)
		{
		_mm_store_ps( &D[0+ldc*0], c_03_0 );
		_mm_store_ps( &D[0+ldc*1], c_03_1 );
		_mm_store_ps( &D[0+ldc*2], c_03_2 );
		_mm_store_ps( &D[0+ldc*3], c_03_3 );
		}
	else if(alg==1)
		{
		d_03_0 = _mm_load_ps( &C[0+ldc*0] );
		d_03_1 = _mm_load_ps( &C[0+ldc*1] );
		d_03_2 = _mm_load_ps( &C[0+ldc*2] );
		d_03_3 = _mm_load_ps( &C[0+ldc*3] );
		
		d_03_0 = _mm_add_ps( d_03_0, c_03_0 );
		d_03_1 = _mm_add_ps( d_03_1, c_03_1 );
		d_03_2 = _mm_add_ps( d_03_2, c_03_2 );
		d_03_3 = _mm_add_ps( d_03_3, c_03_3 );

		_mm_store_ps( &C[0+ldc*0], d_03_0 );
		_mm_store_ps( &C[0+ldc*1], d_03_1 );
		_mm_store_ps( &C[0+ldc*2], d_03_2 );
		_mm_store_ps( &C[0+ldc*3], d_03_3 );
		}
	else
		{
		d_03_0 = _mm_load_ps( &C[0+ldc*0] );
		d_03_1 = _mm_load_ps( &C[0+ldc*1] );
		d_03_2 = _mm_load_ps( &C[0+ldc*2] );
		d_03_3 = _mm_load_ps( &C[0+ldc*3] );
		
		d_03_0 = _mm_sub_ps( d_03_0, c_03_0 );
		d_03_1 = _mm_sub_ps( d_03_1, c_03_1 );
		d_03_2 = _mm_sub_ps( d_03_2, c_03_2 );
		d_03_3 = _mm_sub_ps( d_03_3, c_03_3 );

		_mm_store_ps( &D[0+ldc*0], d_03_0 );
		_mm_store_ps( &D[0+ldc*1], d_03_1 );
		_mm_store_ps( &D[0+ldc*2], d_03_2 );
		_mm_store_ps( &D[0+ldc*3], d_03_3 );
		}

	}

