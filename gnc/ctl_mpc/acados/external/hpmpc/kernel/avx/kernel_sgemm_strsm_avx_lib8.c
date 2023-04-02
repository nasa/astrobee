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

#include "../../include/block_size.h"



// normal-transposed, 16x4 with data packed in 4
void kernel_sgemm_strsm_nt_16x4_lib8(int kadd, int ksub, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, float *fact)
	{
	
	const int bs = 8;
	const int ncl = S_NCL;
	const int ldc = 8;
	
	int k;

	__m256
		temp,
		a_07, a_8f, A_07, A_8f,
		b_03, B_03, b_0,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83;
	
	// prefetch
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
	a_07 = _mm256_load_ps( &A0[0] );
	a_8f = _mm256_load_ps( &A1[0] );
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();

	k = 0;
	for(; k<kadd-3; k+=4)
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[24] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[32] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}
	if(kadd%4>=2)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );


		A0 += 16;
		A1 += 16;
		B  += 16;
		
		}
	if(kadd%2==1)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );*/
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		A_07 = _mm256_load_ps( &A0[8] );*/
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
/*		A_8f = _mm256_load_ps( &A1[8] );*/
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		A0 += 8; // keep it !!!
		A1 += 8; // keep it !!!
		B  += 8; // keep it !!!

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A0 += bs*((ncl-kadd%ncl)%ncl);
			A1 += bs*((ncl-kadd%ncl)%ncl);
			B  += bs*((ncl-kadd%ncl)%ncl);
/*printf("\nk0 = %d\n", (ncl-kadd%ncl)%ncl);*/
			}
		// prefetch
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		a_07 = _mm256_load_ps( &A0[0] );
		a_8f = _mm256_load_ps( &A1[0] );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		}

	for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[24] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[32] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}

	__m256
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83;

	d_00 = _mm256_load_ps( &C0[0+ldc*0] );
	d_00 = _mm256_add_ps( d_00, c_00 );
	d_01 = _mm256_load_ps( &C0[0+ldc*1] );
	d_01 = _mm256_add_ps( d_01, c_01 );
	d_02 = _mm256_load_ps( &C0[0+ldc*2] );
	d_02 = _mm256_add_ps( d_02, c_02 );
	d_03 = _mm256_load_ps( &C0[0+ldc*3] );
	d_03 = _mm256_add_ps( d_03, c_03 );
	d_80 = _mm256_load_ps( &C1[0+ldc*0] );
	d_80 = _mm256_add_ps( d_80, c_80 );
	d_81 = _mm256_load_ps( &C1[0+ldc*1] );
	d_81 = _mm256_add_ps( d_81, c_81 );
	d_82 = _mm256_load_ps( &C1[0+ldc*2] );
	d_82 = _mm256_add_ps( d_82, c_82 );
	d_83 = _mm256_load_ps( &C1[0+ldc*3] );
	d_83 = _mm256_add_ps( d_83, c_83 );

	// factorize the upper 4x4 matrix
	__m256
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	// first row
	a_00 = _mm256_broadcast_ss( &fact[0] );
	d_00  = _mm256_mul_ps( d_00, a_00 );
	_mm256_store_ps( &D0[0+ldc*0], d_00 ); // a_00
	d_80 = _mm256_mul_ps( d_80, a_00 );
	_mm256_store_ps( &D1[0+ldc*0], d_80 );
		
	// second row
	a_10 = _mm256_broadcast_ss( &fact[1] );
	a_11 = _mm256_broadcast_ss( &fact[2] );
	temp  = _mm256_mul_ps( d_00, a_10 );
	d_01  = _mm256_sub_ps( d_01, temp );
	temp  = _mm256_mul_ps( d_80, a_10 );
	d_81  = _mm256_sub_ps( d_81, temp );
	d_01  = _mm256_mul_ps( d_01, a_11 );
	_mm256_store_ps( &D0[0+ldc*1], d_01 ); // a_00
	d_81 = _mm256_mul_ps( d_81, a_11 );
	_mm256_store_ps( &D1[0+ldc*1], d_81 );

	// third row
	a_20 = _mm256_broadcast_ss( &fact[3] );
	a_21 = _mm256_broadcast_ss( &fact[4] );
	a_22 = _mm256_broadcast_ss( &fact[5] );
	temp  = _mm256_mul_ps( d_00, a_20 );
	d_02  = _mm256_sub_ps( d_02, temp );
	temp  = _mm256_mul_ps( d_80, a_20 );
	d_82  = _mm256_sub_ps( d_82, temp );
	temp  = _mm256_mul_ps( d_01, a_21 );
	d_02  = _mm256_sub_ps( d_02, temp );
	temp  = _mm256_mul_ps( d_81, a_21 );
	d_82  = _mm256_sub_ps( d_82, temp );
	d_02  = _mm256_mul_ps( d_02, a_22 );
	_mm256_store_ps( &D0[0+ldc*2], d_02 ); // a_00
	d_82 = _mm256_mul_ps( d_82, a_22 );
	_mm256_store_ps( &D1[0+ldc*2], d_82 );

	// fourth row
	a_30 = _mm256_broadcast_ss( &fact[6] );
	a_31 = _mm256_broadcast_ss( &fact[7] );
	a_32 = _mm256_broadcast_ss( &fact[8] );
	a_33 = _mm256_broadcast_ss( &fact[9] );
	temp  = _mm256_mul_ps( d_00, a_30 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_80, a_30 );
	d_83  = _mm256_sub_ps( d_83, temp );
	temp  = _mm256_mul_ps( d_01, a_31 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_81, a_31 );
	d_83  = _mm256_sub_ps( d_83, temp );
	temp  = _mm256_mul_ps( d_02, a_32 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_82, a_32 );
	d_83  = _mm256_sub_ps( d_83, temp );
	d_03  = _mm256_mul_ps( d_03, a_33 );
	_mm256_store_ps( &D0[0+ldc*3], d_03 ); // a_00
	d_83 = _mm256_mul_ps( d_83, a_33 );
	_mm256_store_ps( &D1[0+ldc*3], d_83 );

	}



// normal-transposed, 16x4 with data packed in 4
void kernel_sgemm_strsm_nt_8x4_lib8(int kadd, int ksub, float *A0, float *B, float *C0, float *D0, float *fact)
	{
	
	const int bs = 8;
	const int ncl = S_NCL;
	const int ldc = 8;
	
	int k;

	__m256
		temp,
		a_07, A_07,
		b_03, B_03, b_0,
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
	for(; k<kadd-3; k+=4)
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );


		A0 += 32;
		B  += 32;

		}
	if(kadd%4>=2)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );


		A0 += 16;
		B  += 16;
		
		}
	if(kadd%2==1)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );*/
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		A_07 = _mm256_load_ps( &A0[8] );*/
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		A0 += 8; // keep it !!!
		B  += 8; // keep it !!!

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A0 += bs*((ncl-kadd%ncl)%ncl);
			B  += bs*((ncl-kadd%ncl)%ncl);
/*printf("\nk0 = %d\n", (ncl-kadd%ncl)%ncl);*/
			}
		// prefetch
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		a_07 = _mm256_load_ps( &A0[0] );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		}

	for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );


		A0 += 32;
		B  += 32;

		}

	__m256
		d_00, d_01, d_02, d_03;

	d_00 = _mm256_load_ps( &C0[0+ldc*0] );
	d_00 = _mm256_add_ps( d_00, c_00 );
	d_01 = _mm256_load_ps( &C0[0+ldc*1] );
	d_01 = _mm256_add_ps( d_01, c_01 );
	d_02 = _mm256_load_ps( &C0[0+ldc*2] );
	d_02 = _mm256_add_ps( d_02, c_02 );
	d_03 = _mm256_load_ps( &C0[0+ldc*3] );
	d_03 = _mm256_add_ps( d_03, c_03 );

	// factorize the upper 4x4 matrix
	__m256
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	// first row
	a_00 = _mm256_broadcast_ss( &fact[0] );
	d_00  = _mm256_mul_ps( d_00, a_00 );
	_mm256_store_ps( &D0[0+ldc*0], d_00 ); // a_00
		
	// second row
	a_10 = _mm256_broadcast_ss( &fact[1] );
	a_11 = _mm256_broadcast_ss( &fact[2] );
	temp  = _mm256_mul_ps( d_00, a_10 );
	d_01  = _mm256_sub_ps( d_01, temp );
	d_01  = _mm256_mul_ps( d_01, a_11 );
	_mm256_store_ps( &D0[0+ldc*1], d_01 ); // a_00

	// third row
	a_20 = _mm256_broadcast_ss( &fact[3] );
	a_21 = _mm256_broadcast_ss( &fact[4] );
	a_22 = _mm256_broadcast_ss( &fact[5] );
	temp  = _mm256_mul_ps( d_00, a_20 );
	d_02  = _mm256_sub_ps( d_02, temp );
	temp  = _mm256_mul_ps( d_01, a_21 );
	d_02  = _mm256_sub_ps( d_02, temp );
	d_02  = _mm256_mul_ps( d_02, a_22 );
	_mm256_store_ps( &D0[0+ldc*2], d_02 ); // a_00

	// fourth row
	a_30 = _mm256_broadcast_ss( &fact[6] );
	a_31 = _mm256_broadcast_ss( &fact[7] );
	a_32 = _mm256_broadcast_ss( &fact[8] );
	a_33 = _mm256_broadcast_ss( &fact[9] );
	temp  = _mm256_mul_ps( d_00, a_30 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_01, a_31 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_02, a_32 );
	d_03  = _mm256_sub_ps( d_03, temp );
	d_03  = _mm256_mul_ps( d_03, a_33 );
	_mm256_store_ps( &D0[0+ldc*3], d_03 ); // a_00

	}

