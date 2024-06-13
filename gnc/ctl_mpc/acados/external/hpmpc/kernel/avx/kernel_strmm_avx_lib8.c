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



// normal-transposed, 16x4 with data packed in 8
void kernel_strmm_nt_16x4_lib8(int kadd, float *A0, float *A1, float *B, float *D0, float *D1)
	{
	
/*	if(kadd<=0)*/
/*		return;*/

/*s_print_mat(8, kadd, A0, 8);*/
/*s_print_mat(8, kadd, A1, 8);*/
/*s_print_mat(4, kadd, B, 8);*/
/*exit(1);*/

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

/*	c_00 = _mm256_setzero_ps();*/
/*	c_01 = _mm256_setzero_ps();*/
/*	c_02 = _mm256_setzero_ps();*/
/*	c_03 = _mm256_setzero_ps();*/
/*	c_80 = _mm256_setzero_ps();*/
/*	c_81 = _mm256_setzero_ps();*/
/*	c_82 = _mm256_setzero_ps();*/
/*	c_83 = _mm256_setzero_ps();*/



	// k==0
/*	temp = _mm256_mul_ps( a_07, b_0 );*/
	c_00 = _mm256_mul_ps( a_07, b_0 );
	B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
/*	c_00 = _mm256_add_ps( c_00, temp );*/
/*	temp = _mm256_mul_ps( a_8f, b_0 );*/
	c_80 = _mm256_mul_ps( a_8f, b_0 );
/*	b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );*/
/*	c_80 = _mm256_add_ps( c_80, temp );*/
	
/*	temp = _mm256_mul_ps( a_07, b_0 );*/
	A_07 = _mm256_load_ps( &A0[8] );
/*	c_01 = _mm256_add_ps( c_01, temp );*/
/*	temp = _mm256_mul_ps( a_8f, b_0 );*/
/*	b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );*/
/*	c_81 = _mm256_add_ps( c_81, temp );*/

/*	temp = _mm256_mul_ps( a_07, b_0 );*/
	A_8f = _mm256_load_ps( &A1[8] );
/*	c_02 = _mm256_add_ps( c_02, temp );*/
/*	temp = _mm256_mul_ps( a_8f, b_0 );*/
/*	b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );*/
/*	c_82 = _mm256_add_ps( c_82, temp );*/

/*	temp = _mm256_mul_ps( a_07, b_0 );*/
/*	c_03 = _mm256_add_ps( c_03, temp );*/
/*	temp = _mm256_mul_ps( a_8f, b_0 );*/
	b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
/*	c_83 = _mm256_add_ps( c_83, temp );*/

	
	
	// k==1
	temp = _mm256_mul_ps( A_07, b_0 );
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
	c_00 = _mm256_add_ps( c_00, temp );
	temp = _mm256_mul_ps( A_8f, b_0 );
	b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
	c_80 = _mm256_add_ps( c_80, temp );
	
/*	temp = _mm256_mul_ps( A_07, b_0 );*/
	c_01 = _mm256_mul_ps( A_07, b_0 );
	a_07 = _mm256_load_ps( &A0[16] );
/*	c_01 = _mm256_add_ps( c_01, temp );*/
/*	temp = _mm256_mul_ps( A_8f, b_0 );*/
	c_81 = _mm256_mul_ps( A_8f, b_0 );
/*	b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );*/
/*	c_81 = _mm256_add_ps( c_81, temp );*/

/*	temp = _mm256_mul_ps( A_07, b_0 );*/
	a_8f = _mm256_load_ps( &A1[16] );
/*	c_02 = _mm256_add_ps( c_02, temp );*/
/*	temp = _mm256_mul_ps( A_8f, b_0 );*/
/*	b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );*/
/*	c_82 = _mm256_add_ps( c_82, temp );*/

/*	temp = _mm256_mul_ps( A_07, b_0 );*/
/*	c_03 = _mm256_add_ps( c_03, temp );*/
/*	temp = _mm256_mul_ps( A_8f, b_0 );*/
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*	c_83 = _mm256_add_ps( c_83, temp );*/

	
	
	// k==2
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

/*	temp = _mm256_mul_ps( a_07, b_0 );*/
	c_02 = _mm256_mul_ps( a_07, b_0 );
	A_8f = _mm256_load_ps( &A1[24] );
/*	c_02 = _mm256_add_ps( c_02, temp );*/
/*	temp = _mm256_mul_ps( a_8f, b_0 );*/
	c_82 = _mm256_mul_ps( a_8f, b_0 );
/*	b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );*/
/*	c_82 = _mm256_add_ps( c_82, temp );*/

/*	temp = _mm256_mul_ps( a_07, b_0 );*/
/*	c_03 = _mm256_add_ps( c_03, temp );*/
/*	temp = _mm256_mul_ps( a_8f, b_0 );*/
	b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
/*	c_83 = _mm256_add_ps( c_83, temp );*/

	
	
	// k==3
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

/*	temp = _mm256_mul_ps( A_07, b_0 );*/
	c_03 = _mm256_mul_ps( A_07, b_0 );
/*	c_03 = _mm256_add_ps( c_03, temp );*/
/*	temp = _mm256_mul_ps( A_8f, b_0 );*/
	c_83 = _mm256_mul_ps( A_8f, b_0 );
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
/*	c_83 = _mm256_add_ps( c_83, temp );*/


	A0 += 32;
	A1 += 32;
	B  += 32;


	k = 4;
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

		}

	_mm256_store_ps( &D0[0+ldc*0], c_00 );
	_mm256_store_ps( &D0[0+ldc*1], c_01 );
	_mm256_store_ps( &D0[0+ldc*2], c_02 );
	_mm256_store_ps( &D0[0+ldc*3], c_03 );
	_mm256_store_ps( &D1[0+ldc*0], c_80 );
	_mm256_store_ps( &D1[0+ldc*1], c_81 );
	_mm256_store_ps( &D1[0+ldc*2], c_82 );
	_mm256_store_ps( &D1[0+ldc*3], c_83 );

	}



// normal-transposed, 8x4 with data packed in 8
void kernel_strmm_nt_8x4_lib8(int kadd, float *A0, float *B, float *D0)
	{
	
/*	if(kadd<=0)*/
/*		return;*/
	
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



	// k==0
/*	temp = _mm256_mul_ps( a_07, b_0 );*/
	c_00 = _mm256_mul_ps( a_07, b_0 );
	B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
/*	c_00 = _mm256_add_ps( c_00, temp );*/
	A_07 = _mm256_load_ps( &A0[8] );
	b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

	
	
	// k==1
	temp = _mm256_mul_ps( A_07, b_0 );
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
	c_00 = _mm256_add_ps( c_00, temp );
	b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
	
/*	temp = _mm256_mul_ps( A_07, b_0 );*/
	c_01 = _mm256_mul_ps( A_07, b_0 );
	a_07 = _mm256_load_ps( &A0[16] );
/*	c_01 = _mm256_add_ps( c_01, temp );*/
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

	
	
	// k==2
	temp = _mm256_mul_ps( a_07, b_0 );
	B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
	c_00 = _mm256_add_ps( c_00, temp );
	b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
	
	temp = _mm256_mul_ps( a_07, b_0 );
	A_07 = _mm256_load_ps( &A0[24] );
	c_01 = _mm256_add_ps( c_01, temp );
	b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

/*	temp = _mm256_mul_ps( a_07, b_0 );*/
	c_02 = _mm256_mul_ps( a_07, b_0 );
/*	c_02 = _mm256_add_ps( c_02, temp );*/
	b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

	
	
	// k==3
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

/*	temp = _mm256_mul_ps( A_07, b_0 );*/
	c_03 = _mm256_mul_ps( A_07, b_0 );
/*	c_03 = _mm256_add_ps( c_03, temp );*/
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );


	A0 += 32;
/*	A1 += 32;*/
	B  += 32;


	k = 4;
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
/*		A1 += 32;*/
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
/*		A1 += 16;*/
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

		}

	_mm256_store_ps( &D0[0+ldc*0], c_00 );
	_mm256_store_ps( &D0[0+ldc*1], c_01 );
	_mm256_store_ps( &D0[0+ldc*2], c_02 );
	_mm256_store_ps( &D0[0+ldc*3], c_03 );

	}



void corner_strmm_nt_16x3_lib8(float *A0, float *A1, float *B, float *C0, float *C1)
	{
	
	const int ldc = 8;

	__m256
		temp,
		a_00, a_01, a_02, a_80, a_81, a_82,
		b_00, b_10, b_20, b_11, b_21, b_22,
		c_00, c_01, c_02, c_80, c_81, c_82;
	
	a_00 = _mm256_load_ps( &A0[0+8*0] );
	a_80 = _mm256_load_ps( &A1[0+8*0] );
	a_01 = _mm256_load_ps( &A0[0+8*1] );
	a_81 = _mm256_load_ps( &A1[0+8*1] );
	a_02 = _mm256_load_ps( &A0[0+8*2] );
	a_82 = _mm256_load_ps( &A1[0+8*2] );
	
	// first column 
	b_00 = _mm256_broadcast_ss( &B[0+8*0] );
	b_10 = _mm256_broadcast_ss( &B[0+8*1] );
	b_20 = _mm256_broadcast_ss( &B[0+8*2] );
	
	c_00 = _mm256_mul_ps( a_00, b_00 );
	c_80 = _mm256_mul_ps( a_80, b_00 );

	temp = _mm256_mul_ps( a_01, b_10 );
	c_00 = _mm256_add_ps( c_00, temp );
	temp = _mm256_mul_ps( a_81, b_10 );
	c_80 = _mm256_add_ps( c_80, temp );

	temp = _mm256_mul_ps( a_02, b_20 );
	c_00 = _mm256_add_ps( c_00, temp );
	temp = _mm256_mul_ps( a_82, b_20 );
	c_80 = _mm256_add_ps( c_80, temp );

	_mm256_store_ps( &C0[0+ldc*0], c_00 );
	_mm256_store_ps( &C1[0+ldc*0], c_80 );
	
	// second column 
	b_11 = _mm256_broadcast_ss( &B[1+8*1] );
	b_21 = _mm256_broadcast_ss( &B[1+8*2] );

	c_01 = _mm256_mul_ps( a_01, b_11 );
	c_81 = _mm256_mul_ps( a_81, b_11 );

	temp = _mm256_mul_ps( a_02, b_21 );
	c_01 = _mm256_add_ps( c_01, temp );
	temp = _mm256_mul_ps( a_82, b_21 );
	c_81 = _mm256_add_ps( c_81, temp );
	
	_mm256_store_ps( &C0[0+ldc*1], c_01 );
	_mm256_store_ps( &C1[0+ldc*1], c_81 );
	
	// third column 
	b_22 = _mm256_broadcast_ss( &B[2+8*2] );

	c_02 = _mm256_mul_ps( a_02, b_22 );
	c_82 = _mm256_mul_ps( a_82, b_22 );

	_mm256_store_ps( &C0[0+ldc*2], c_02 );
	_mm256_store_ps( &C1[0+ldc*2], c_82 );

	}



void corner_strmm_nt_16x2_lib8(float *A0, float *A1, float *B, float *C0, float *C1)
	{
	
	const int ldc = 8;

	__m256
		temp,
		a_00, a_01, a_80, a_81,
		b_00, b_10, b_11,
		c_00, c_01, c_80, c_81;
	
	a_00 = _mm256_load_ps( &A0[0+8*0] );
	a_80 = _mm256_load_ps( &A1[0+8*0] );
	a_01 = _mm256_load_ps( &A0[0+8*1] );
	a_81 = _mm256_load_ps( &A1[0+8*1] );
	
	// first column 
	b_00 = _mm256_broadcast_ss( &B[0+8*0] );
	b_10 = _mm256_broadcast_ss( &B[0+8*1] );
	
	c_00 = _mm256_mul_ps( a_00, b_00 );
	c_80 = _mm256_mul_ps( a_80, b_00 );

	temp = _mm256_mul_ps( a_01, b_10 );
	c_00 = _mm256_add_ps( c_00, temp );
	temp = _mm256_mul_ps( a_81, b_10 );
	c_80 = _mm256_add_ps( c_80, temp );

	_mm256_store_ps( &C0[0+ldc*0], c_00 );
	_mm256_store_ps( &C1[0+ldc*0], c_80 );
	
	// second column 
	b_11 = _mm256_broadcast_ss( &B[1+8*1] );

	c_01 = _mm256_mul_ps( a_01, b_11 );
	c_81 = _mm256_mul_ps( a_81, b_11 );
	
	_mm256_store_ps( &C0[0+ldc*1], c_01 );
	_mm256_store_ps( &C1[0+ldc*1], c_81 );

	}


void corner_strmm_nt_16x1_lib8(float *A0, float *A1, float *B, float *C0, float *C1)
	{
	
	const int ldc = 8;

	__m256
		temp,
		a_00, a_80,
		b_00,
		c_00, c_80;
	
	a_00 = _mm256_load_ps( &A0[0+8*0] );
	a_80 = _mm256_load_ps( &A1[0+8*0] );
	
	// first column 
	b_00 = _mm256_broadcast_ss( &B[0+8*0] );
	
	c_00 = _mm256_mul_ps( a_00, b_00 );
	c_80 = _mm256_mul_ps( a_80, b_00 );

	_mm256_store_ps( &C0[0+ldc*0], c_00 );
	_mm256_store_ps( &C1[0+ldc*0], c_80 );

	}



void corner_strmm_nt_8x3_lib8(float *A0, float *B, float *C0)
	{
	
	const int ldc = 8;

	__m256
		temp,
		a_00, a_01, a_02,
		b_00, b_10, b_20, b_11, b_21, b_22,
		c_00, c_01, c_02;
	
	a_00 = _mm256_load_ps( &A0[0+8*0] );
	a_01 = _mm256_load_ps( &A0[0+8*1] );
	a_02 = _mm256_load_ps( &A0[0+8*2] );
	
	// first column 
	b_00 = _mm256_broadcast_ss( &B[0+8*0] );
	b_10 = _mm256_broadcast_ss( &B[0+8*1] );
	b_20 = _mm256_broadcast_ss( &B[0+8*2] );
	
	c_00 = _mm256_mul_ps( a_00, b_00 );

	temp = _mm256_mul_ps( a_01, b_10 );
	c_00 = _mm256_add_ps( c_00, temp );

	temp = _mm256_mul_ps( a_02, b_20 );
	c_00 = _mm256_add_ps( c_00, temp );

	_mm256_store_ps( &C0[0+ldc*0], c_00 );
	
	// second column 
	b_11 = _mm256_broadcast_ss( &B[1+8*1] );
	b_21 = _mm256_broadcast_ss( &B[1+8*2] );

	c_01 = _mm256_mul_ps( a_01, b_11 );

	temp = _mm256_mul_ps( a_02, b_21 );
	c_01 = _mm256_add_ps( c_01, temp );
	
	_mm256_store_ps( &C0[0+ldc*1], c_01 );
	
	// third column 
	b_22 = _mm256_broadcast_ss( &B[2+8*2] );

	c_02 = _mm256_mul_ps( a_02, b_22 );

	_mm256_store_ps( &C0[0+ldc*2], c_02 );

	}



void corner_strmm_nt_8x2_lib8(float *A0, float *B, float *C0)
	{
	
	const int ldc = 8;

	__m256
		temp,
		a_00, a_01,
		b_00, b_10, b_11,
		c_00, c_01;
	
	a_00 = _mm256_load_ps( &A0[0+8*0] );
	a_01 = _mm256_load_ps( &A0[0+8*1] );
	
	// first column 
	b_00 = _mm256_broadcast_ss( &B[0+8*0] );
	b_10 = _mm256_broadcast_ss( &B[0+8*1] );
	
	c_00 = _mm256_mul_ps( a_00, b_00 );

	temp = _mm256_mul_ps( a_01, b_10 );
	c_00 = _mm256_add_ps( c_00, temp );

	_mm256_store_ps( &C0[0+ldc*0], c_00 );
	
	// second column 
	b_11 = _mm256_broadcast_ss( &B[1+8*1] );

	c_01 = _mm256_mul_ps( a_01, b_11 );
	
	_mm256_store_ps( &C0[0+ldc*1], c_01 );

	}


void corner_strmm_nt_8x1_lib8(float *A0, float *B, float *C0)
	{
	
	const int ldc = 8;

	__m256
		a_00,
		b_00,
		c_00;
	
	a_00 = _mm256_load_ps( &A0[0+8*0] );
	
	// first column 
	b_00 = _mm256_broadcast_ss( &B[0+8*0] );
	
	c_00 = _mm256_mul_ps( a_00, b_00 );

	_mm256_store_ps( &C0[0+ldc*0], c_00 );

	}

