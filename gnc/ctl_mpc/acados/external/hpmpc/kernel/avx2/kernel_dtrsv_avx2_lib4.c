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

// new kernels

void kernel_dtrsv_n_12_lib4_new(int kmax, double *A0, int sda, int use_inv_diag_A, double *inv_diag_A, double *x, double *y)
	{
	
	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	__m256d
		a_0,
		x_0, x_1,
		y_0, y_0_b, y_0_c, y_0_d,
		y_4, y_4_b, y_4_c, y_4_d,
		y_8, y_8_b, y_8_c, y_8_d,
		z_0_1_2_3, z_4_5_6_7, z_8_9_a_b;
	
	y_0   = _mm256_setzero_pd();	
	y_4   = _mm256_setzero_pd();	
	y_8   = _mm256_setzero_pd();	
	y_0_b = _mm256_setzero_pd();	
	y_4_b = _mm256_setzero_pd();	
	y_8_b = _mm256_setzero_pd();	
	y_0_c = _mm256_setzero_pd();	
	y_4_c = _mm256_setzero_pd();	
	y_8_c = _mm256_setzero_pd();	
	y_0_d = _mm256_setzero_pd();	
	y_4_d = _mm256_setzero_pd();	
	y_8_d = _mm256_setzero_pd();	

	k=0;

	for(; k<kmax-11; k+=12)
		{

//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );
		a_0 = _mm256_load_pd( &A2[0+lda*0] );
		y_8   = _mm256_fmadd_pd( a_0, x_0, y_8 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*1] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );
		a_0 = _mm256_load_pd( &A2[0+lda*1] );
		y_8_b = _mm256_fmadd_pd( a_0, x_1, y_8_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*2] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );
		a_0 = _mm256_load_pd( &A2[0+lda*2] );
		y_8_c = _mm256_fmadd_pd( a_0, x_0, y_8_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*3] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
		a_0 = _mm256_load_pd( &A2[0+lda*3] );
		y_8_d = _mm256_fmadd_pd( a_0, x_1, y_8_d );
	
//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*4] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*4] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );
		a_0 = _mm256_load_pd( &A2[0+lda*4] );
		y_8   = _mm256_fmadd_pd( a_0, x_0, y_8 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*5] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*5] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );
		a_0 = _mm256_load_pd( &A2[0+lda*5] );
		y_8_b = _mm256_fmadd_pd( a_0, x_1, y_8_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*6] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*6] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );
		a_0 = _mm256_load_pd( &A2[0+lda*6] );
		y_8_c = _mm256_fmadd_pd( a_0, x_0, y_8_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*7] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*7] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
		a_0 = _mm256_load_pd( &A2[0+lda*7] );
		y_8_d = _mm256_fmadd_pd( a_0, x_1, y_8_d );
		
//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[8] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*8] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*8] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );
		a_0 = _mm256_load_pd( &A2[0+lda*8] );
		y_8   = _mm256_fmadd_pd( a_0, x_0, y_8 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*9] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*9] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );
		a_0 = _mm256_load_pd( &A2[0+lda*9] );
		y_8_b = _mm256_fmadd_pd( a_0, x_1, y_8_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[10] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*10] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*10] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );
		a_0 = _mm256_load_pd( &A2[0+lda*10] );
		y_8_c = _mm256_fmadd_pd( a_0, x_0, y_8_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*11] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*11] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
		a_0 = _mm256_load_pd( &A2[0+lda*11] );
		y_8_d = _mm256_fmadd_pd( a_0, x_1, y_8_d );
	
		A0 += 12*lda;
		A1 += 12*lda;
		A2 += 12*lda;
		x  += 12;

		}
	
	y_0   = _mm256_add_pd( y_0, y_0_c );
	y_4   = _mm256_add_pd( y_4, y_4_c );
	y_8   = _mm256_add_pd( y_8, y_8_c );
	y_0_b = _mm256_add_pd( y_0_b, y_0_d );
	y_4_b = _mm256_add_pd( y_4_b, y_4_d );
	y_8_b = _mm256_add_pd( y_8_b, y_8_d );

	y_0 = _mm256_add_pd( y_0, y_0_b );
	y_4 = _mm256_add_pd( y_4, y_4_b );
	y_8 = _mm256_add_pd( y_8, y_8_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_4_5_6_7 = _mm256_loadu_pd( &y[4] );
	z_8_9_a_b = _mm256_loadu_pd( &y[8] );

	z_0_1_2_3 = _mm256_sub_pd( z_0_1_2_3, y_0 );
	z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, y_4 );
	z_8_9_a_b = _mm256_sub_pd( z_8_9_a_b, y_8 );

	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	if(use_inv_diag_A)
		{
#if 0
		// A_00
		z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

		a_00 = _mm_load_sd( &inv_diag_A[0] );
		a_10 = _mm_load_sd( &A0[1+lda*0] );
		a_11 = _mm_load_sd( &inv_diag_A[1] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+lda*0] );
		a_21_31 = _mm_load_pd( &A0[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );
		
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*0] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*1] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );


		// A_11
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		a_10 = _mm_load_sd( &A0[3+lda*2] );
		a_11 = _mm_load_sd( &inv_diag_A[3] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*2] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*3] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );


		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &inv_diag_A[4] );
		a_10 = _mm_load_sd( &A1[1+lda*4] );
		a_11 = _mm_load_sd( &inv_diag_A[5] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+lda*4] );
		a_21_31 = _mm_load_pd( &A1[2+lda*5] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A2[0+lda*4] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A2[0+lda*5] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );
	

		// A_33
		a_00 = _mm_load_sd( &inv_diag_A[6] );
		a_10 = _mm_load_sd( &A1[3+lda*6] );
		a_11 = _mm_load_sd( &inv_diag_A[7] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[6], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[7], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A2[0+lda*6] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A2[0+lda*7] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );


		// A_44
		z_2_3 = _mm256_extractf128_pd( z_8_9_a_b, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_8_9_a_b );

		a_00 = _mm_load_sd( &inv_diag_A[8] );
		a_10 = _mm_load_sd( &A2[1+lda*8] );
		a_11 = _mm_load_sd( &inv_diag_A[9] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A2[2+lda*8] );
		a_21_31 = _mm_load_pd( &A2[2+lda*9] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[8], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[9], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );



		// A_55
		a_00 = _mm_load_sd( &inv_diag_A[10] );
		a_10 = _mm_load_sd( &A2[3+lda*10] );
		a_11 = _mm_load_sd( &inv_diag_A[11] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[10], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[11], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/

#else

		a_0       = _mm256_broadcast_sd( &inv_diag_A[0] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0x00 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*0] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[0], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[1] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0x55 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A0[0+lda*1] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*1] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[1], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[2] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0xaa );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A0[0+lda*2] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*2] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[2], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[3] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0xff );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*3] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[3], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[4] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0x00 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*2] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*4] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*4] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[4], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[5] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0x55 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*5] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*5] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[5], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[6] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0xaa );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*2] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*6] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*6] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[6], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[7] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0xff );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
//		a_0 = _mm256_load_pd( &A1[0+lda*3] );
//		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*7] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[7], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[8] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_8_9_a_b, 0x00 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*2] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
//		a_0 = _mm256_load_pd( &A1[0+lda*4] );
//		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*8] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[8], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[9] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_8_9_a_b, 0x55 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
//		a_0 = _mm256_load_pd( &A1[0+lda*5] );
//		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*9] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[9], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[10] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_8_9_a_b, 0xaa );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*2] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
//		a_0 = _mm256_load_pd( &A1[0+lda*6] );
//		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*10] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[10], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[11] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_8_9_a_b, 0xff );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
//		a_0 = _mm256_load_pd( &A1[0+lda*3] );
//		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
//		a_0 = _mm256_load_pd( &A2[0+lda*7] );
//		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		_mm_store_sd( &y[11], _mm256_castpd256_pd128( x_0 ) );
		
#endif
		
		
#if 0
		double temp[16];
		_mm256_storeu_pd( temp+0, z_0_1_2_3 );
		_mm256_storeu_pd( temp+4, z_4_5_6_7 );
		_mm256_storeu_pd( temp+8, z_8_9_a_b );
		_mm_storeu_pd( temp+12, z_2_3 );
		d_print_mat(4, 4, temp, 4);
		exit(2);
#endif


		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// A_00
		z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

		a_00 = _mm_load_sd( &A0[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[1+lda*0] );
		a_11 = _mm_load_sd( &A0[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+lda*0] );
		a_21_31 = _mm_load_pd( &A0[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );
		
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*0] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*1] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );
		
		

		// A_11
		a_00 = _mm_load_sd( &A0[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[3+lda*2] );
		a_11 = _mm_load_sd( &A0[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*2] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A2[0+lda*3] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );



		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &A1[0+lda*4] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[1+lda*4] );
		a_11 = _mm_load_sd( &A1[1+lda*5] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+lda*4] );
		a_21_31 = _mm_load_pd( &A1[2+lda*5] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A2[0+lda*4] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A2[0+lda*5] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );
	

		// A_33
		a_00 = _mm_load_sd( &A1[2+lda*6] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[3+lda*6] );
		a_11 = _mm_load_sd( &A1[3+lda*7] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[6], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[7], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A2[0+lda*6] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_0, z_8_9_a_b );
		a_0 = _mm256_load_pd( &A2[0+lda*7] );
		z_8_9_a_b = _mm256_fnmadd_pd( a_0, x_1, z_8_9_a_b );

		// A_44
		z_2_3 = _mm256_extractf128_pd( z_8_9_a_b, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_8_9_a_b );

		//a_00 = _mm_load_sd( &inv_diag_A[8] );
		a_00 = _mm_load_sd( &A2[0+lda*8] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A2[1+lda*8] );
		//a_11 = _mm_load_sd( &inv_diag_A[9] );
		a_11 = _mm_load_sd( &A2[1+lda*9] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A2[2+lda*8] );
		a_21_31 = _mm_load_pd( &A2[2+lda*9] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[8], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[9], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );



		// A_55
		//a_00 = _mm_load_sd( &inv_diag_A[10] );
		a_00 = _mm_load_sd( &A2[2+lda*10] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A2[3+lda*10] );
		//a_11 = _mm_load_sd( &inv_diag_A[11] );
		a_11 = _mm_load_sd( &A2[3+lda*11] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[10], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[11], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/

		}


	}



// it moves horizontally inside a block ( assume ksv>0 !!! )
void kernel_dtrsv_n_8_lib4_new(int kmax, double *A0, int sda, int use_inv_diag_A, double *inv_diag_A, double *x, double *y)
	{
	
	double *A1 = A0 + 4*sda;

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	__m256d
		a_0,
		x_0, x_1,
		y_0, y_0_b, y_0_c, y_0_d,
		y_4, y_4_b, y_4_c, y_4_d,
		z_0_1_2_3, z_4_5_6_7;
	
	y_0   = _mm256_setzero_pd();	
	y_4   = _mm256_setzero_pd();	
	y_0_b = _mm256_setzero_pd();	
	y_4_b = _mm256_setzero_pd();	
	y_0_c = _mm256_setzero_pd();	
	y_4_c = _mm256_setzero_pd();	
	y_0_d = _mm256_setzero_pd();	
	y_4_d = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-7; k+=8)
		{

//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*1] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*2] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*3] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
	
//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*4] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*4] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*5] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*5] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*6] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*6] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*7] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*7] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
	
		A0 += 8*lda;
		A1 += 8*lda;
		x  += 8;

		}
	for(; k<kmax-3; k+=4)
		{

//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*1] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*2] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*3] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
	
		A0 += 4*lda;
		A1 += 4*lda;
		x  += 4;

		}
	
	y_0   = _mm256_add_pd( y_0, y_0_c );
	y_4   = _mm256_add_pd( y_4, y_4_c );
	y_0_b = _mm256_add_pd( y_0_b, y_0_d );
	y_4_b = _mm256_add_pd( y_4_b, y_4_d );

	y_0 = _mm256_add_pd( y_0, y_0_b );
	y_4 = _mm256_add_pd( y_4, y_4_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_4_5_6_7 = _mm256_loadu_pd( &y[4] );

	z_0_1_2_3 = _mm256_sub_pd( z_0_1_2_3, y_0 );
	z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, y_4 );

	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	if(use_inv_diag_A)
		{
#if 0
		// A_00
		z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

		a_00 = _mm_load_sd( &inv_diag_A[0] );
		a_10 = _mm_load_sd( &A0[1+lda*0] );
		a_11 = _mm_load_sd( &inv_diag_A[1] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+lda*0] );
		a_21_31 = _mm_load_pd( &A0[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );
		
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		
		

		// A_11
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		a_10 = _mm_load_sd( &A0[3+lda*2] );
		a_11 = _mm_load_sd( &inv_diag_A[3] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );



		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &inv_diag_A[4] );
		a_10 = _mm_load_sd( &A1[1+lda*4] );
		a_11 = _mm_load_sd( &inv_diag_A[5] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+lda*4] );
		a_21_31 = _mm_load_pd( &A1[2+lda*5] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );



		// A_33
		a_00 = _mm_load_sd( &inv_diag_A[6] );
		a_10 = _mm_load_sd( &A1[3+lda*6] );
		a_11 = _mm_load_sd( &inv_diag_A[7] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[6], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[7], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/

#else

		a_0       = _mm256_broadcast_sd( &inv_diag_A[0] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0x00 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[0], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[1] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0x55 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A0[0+lda*1] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[1], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[2] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0xaa );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A0[0+lda*2] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[2], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[3] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0xff );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[3], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[4] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0x00 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*2] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*4] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[4], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[5] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0x55 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*5] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[5], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[6] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0xaa );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*2] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		a_0 = _mm256_load_pd( &A1[0+lda*6] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[6], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[7] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_4_5_6_7, 0xff );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A0[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
//		a_0 = _mm256_load_pd( &A1[0+lda*3] );
//		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		_mm_store_sd( &y[7], _mm256_castpd256_pd128( x_0 ) );


#endif
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// A_00
		z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

		a_00 = _mm_load_sd( &A0[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[1+lda*0] );
		a_11 = _mm_load_sd( &A0[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+lda*0] );
		a_21_31 = _mm_load_pd( &A0[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );
		
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		
		

		// A_11
		a_00 = _mm_load_sd( &A0[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[3+lda*2] );
		a_11 = _mm_load_sd( &A0[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );



		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &A1[0+lda*4] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[1+lda*4] );
		a_11 = _mm_load_sd( &A1[1+lda*5] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+lda*4] );
		a_21_31 = _mm_load_pd( &A1[2+lda*5] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );



		// A_33
		a_00 = _mm_load_sd( &A1[2+lda*6] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[3+lda*6] );
		a_11 = _mm_load_sd( &A1[3+lda*7] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[6], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[7], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}


	}



// it moves horizontally inside a block ( assume ksv>0 !!! )
void kernel_dtrsv_n_4_lib4_new(int kmax, double *A, int use_inv_diag_A, double *inv_diag_A, double *x, double *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	__m256d
		a_0,
		x_0, x_1,
		y_0_1_2_3  , y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3,
		y_0_1_2_3_e, y_0_1_2_3_f, y_0_1_2_3_g, y_0_1_2_3_h;
	
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	
	y_0_1_2_3_e = _mm256_setzero_pd();	
	y_0_1_2_3_f = _mm256_setzero_pd();	
	y_0_1_2_3_g = _mm256_setzero_pd();	
	y_0_1_2_3_h = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-7; k+=8)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
		x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*4] );
		y_0_1_2_3_e = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_e );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*5] );
		y_0_1_2_3_f = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_f );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*6] );
		y_0_1_2_3_g = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_g );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*7] );
		y_0_1_2_3_h = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_h );
	
		A += 8*lda;
		x += 8;

		}

	y_0_1_2_3   = _mm256_add_pd( y_0_1_2_3  , y_0_1_2_3_e );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_f );
	y_0_1_2_3_c = _mm256_add_pd( y_0_1_2_3_c, y_0_1_2_3_g );
	y_0_1_2_3_d = _mm256_add_pd( y_0_1_2_3_d, y_0_1_2_3_h );

	for(; k<kmax-3; k+=4)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
	
		A += 4*lda;
		x += 4;

		}
	
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_c );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_d );
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_0_1_2_3 = _mm256_sub_pd ( z_0_1_2_3, y_0_1_2_3 );
	
	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
	z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

	if(use_inv_diag_A)
		{

#if 0
		// a_00
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
		
		// a_11
		a_11 = _mm_load_sd( &inv_diag_A[1] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );

		// a_22
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/

		// a_33
		a_11 = _mm_load_sd( &inv_diag_A[3] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/

#else

		a_0       = _mm256_broadcast_sd( &inv_diag_A[0] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0x00 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		_mm_store_sd( &y[0], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[1] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0x55 );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		_mm_store_sd( &y[1], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[2] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0x0 );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0xaa );
		x_0       = _mm256_mul_pd( a_0, x_0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		_mm_store_sd( &y[2], _mm256_castpd256_pd128( x_0 ) );

		a_0       = _mm256_broadcast_sd( &inv_diag_A[3] );
//		x_0       = _mm256_permute2f128_pd( z_0_1_2_3, z_0_1_2_3, 0x00 );
//		x_0       = _mm256_shuffle_pd( x_0, x_0, 0xf );
		x_0       = _mm256_permute4x64_pd( z_0_1_2_3, 0xff );
		x_0       = _mm256_mul_pd( a_0, x_0 );
//		a_0 = _mm256_load_pd( &A[0+lda*3] );
//		z_0_1_2_3 = _mm256_fnmadd_pd( a_0, x_0, z_0_1_2_3 );
		_mm_store_sd( &y[3], _mm256_castpd256_pd128( x_0 ) );


#endif
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// a_00
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
		
		// a_11
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );

		// a_22
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/

		// a_33
		a_11 = _mm_load_sd( &A[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}

	}



// it moves horizontally inside a block ( assume ksv>0 !!! )
void kernel_dtrsv_n_4_vs_lib4_new(int km, int kn, int kmax, double *A, int use_inv_diag_A, double *inv_diag_A, double *x, double *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	__m256d
		a_0,
		x_0, x_1,
		y_0_1_2_3  , y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3,
		y_0_1_2_3_e, y_0_1_2_3_f, y_0_1_2_3_g, y_0_1_2_3_h;
	
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	
	y_0_1_2_3_e = _mm256_setzero_pd();	
	y_0_1_2_3_f = _mm256_setzero_pd();	
	y_0_1_2_3_g = _mm256_setzero_pd();	
	y_0_1_2_3_h = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-7; k+=8)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
		x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*4] );
		y_0_1_2_3_e = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_e );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*5] );
		y_0_1_2_3_f = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_f );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*6] );
		y_0_1_2_3_g = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_g );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*7] );
		y_0_1_2_3_h = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_h );
	
		A += 8*lda;
		x += 8;

		}

	y_0_1_2_3   = _mm256_add_pd( y_0_1_2_3  , y_0_1_2_3_e );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_f );
	y_0_1_2_3_c = _mm256_add_pd( y_0_1_2_3_c, y_0_1_2_3_g );
	y_0_1_2_3_d = _mm256_add_pd( y_0_1_2_3_d, y_0_1_2_3_h );

	for(; k<kmax-3; k+=4)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
	
		A += 4*lda;
		x += 4;

		}
	
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_c );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_d );
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_0_1_2_3 = _mm256_sub_pd ( z_0_1_2_3, y_0_1_2_3 );
	
	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
	z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

	if(use_inv_diag_A)
		{
		// a_00
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
			if(kn==1)
				{
				if(km==1)
					return;
				if(km<3) // km==2
					{
					_mm_store_sd( &y[1], z_1 );
					}
				else if(km==3)
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_sd( &y[2], z_2_3 );
					}
				else // km>=4
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_pd( &y[2], z_2_3 );
					}
				return;
				}
		
		// a_11
		a_11 = _mm_load_sd( &inv_diag_A[1] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );
			if(kn==2)
				{
				if(km==2)
					return;
				if(km==3)
					_mm_store_sd( &y[2], z_2_3 );
				else // km>=4
					_mm_store_pd( &y[2], z_2_3 );
				return;
				}

		// a_22
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
			if(kn==3)
				{
				if(km==3)
					return;
				_mm_store_sd( &y[3], z_1 );
				return;
				}

		// a_33
		a_11 = _mm_load_sd( &inv_diag_A[3] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// a_00
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
			if(kn==1)
				{
				if(km==1)
					return;
				if(km<3) // km==2
					{
					_mm_store_sd( &y[1], z_1 );
					}
				else if(km==3)
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_sd( &y[2], z_2_3 );
					}
				else // km>=4
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_pd( &y[2], z_2_3 );
					}
				return;
				}
		
		// a_11
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );
			if(kn==2)
				{
				if(km==2)
					return;
				if(km==3)
					_mm_store_sd( &y[2], z_2_3 );
				else // km>=4
					_mm_store_pd( &y[2], z_2_3 );
				return;
				}

		// a_22
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
			if(kn==3)
				{
				if(km==3)
					return;
				_mm_store_sd( &y[3], z_1 );
				return;
				}

		// a_33
		a_11 = _mm_load_sd( &A[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}
	
		

	}



// it moves vertically across blocks
void kernel_dtrsv_t_4_lib4_new(int kmax, double *A, int sda, int use_inv_diag_A, double *inv_diag_A, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		zeros,
		mask,
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0_1_2_3,
		y_0a, y_1a, y_2a, y_3a,
		y_0b, y_1b, y_2b, y_3b;
	
	__m128d
/*		tmp,*/
		ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		y_2_3,
		z_0, z_1, z_2, z_3,
		y_0, y_1, y_2, y_3;
		mask = _mm256_loadu_pd( mask_f ); 
	
	k=4;
	if(kmax>4)
		{

		zeros = _mm256_setzero_pd();

		y_0a = _mm256_setzero_pd();
		y_1a = _mm256_setzero_pd();
		y_2a = _mm256_setzero_pd();
		y_3a = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		y_1b = _mm256_setzero_pd();
		y_2b = _mm256_setzero_pd();
		y_3b = _mm256_setzero_pd();
		
		A += 4 + (sda-1)*lda;
		x += 4;
		//for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0a = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0a );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1a = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1a );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2a = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2a );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3a = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3a );
			
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1b = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1b );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2b = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2b );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3b = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3b );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		
		y_0a = _mm256_add_pd( y_0a, y_0b );
		y_1a = _mm256_add_pd( y_1a, y_1b );
		y_2a = _mm256_add_pd( y_2a, y_2b );
		y_3a = _mm256_add_pd( y_3a, y_3b );

		//for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0a = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0a );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1a = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1a );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2a = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2a );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3a = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3a );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0a = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0a );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1a = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1a );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2a = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2a );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3a = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3a );
			
	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}
			
		A = tA;
		x = tx;

		y_0 = _mm256_extractf128_pd( y_0a, 0x1 );
		y_1 = _mm256_extractf128_pd( y_1a, 0x1 );
		y_2 = _mm256_extractf128_pd( y_2a, 0x1 );
		y_3 = _mm256_extractf128_pd( y_3a, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_0a ) );
		y_1 = _mm_add_pd( y_1, _mm256_castpd256_pd128( y_1a ) );
		y_2 = _mm_add_pd( y_2, _mm256_castpd256_pd128( y_2a ) );
		y_3 = _mm_add_pd( y_3, _mm256_castpd256_pd128( y_3a ) );
		}
	else
		{
		y_0 = _mm_setzero_pd();
		y_1 = _mm_setzero_pd();
		y_2 = _mm_setzero_pd();
		y_3 = _mm_setzero_pd();
		}
			
	if(use_inv_diag_A)
		{
		// bottom trinagle
		z_3  = _mm_load_sd( &x[3] );
		y_3  = _mm_hadd_pd( y_3, y_3 );
		a_11 = _mm_load_sd( &inv_diag_A[3] );
		y_3  = _mm_sub_sd( z_3, y_3 );
		y_3  = _mm_mul_sd( y_3, a_11 );
		_mm_store_sd( &x[3], y_3 );

		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_2  = _mm_load_sd( &x[2] );
		z_2  = _mm_fnmadd_sd( a_10, y_3, z_2 );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		y_2_3   = _mm_shuffle_pd( y_2, y_3, 0x0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		y_0     = _mm_fmadd_pd( a_20_30, y_2_3, y_0 );
		y_1     = _mm_fmadd_pd( a_21_31, y_2_3, y_1 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &inv_diag_A[1] );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_3  = _mm_load_sd( &x[3] );
		y_3  = _mm_hadd_pd( y_3, y_3 );
		a_11 = _mm_load_sd( &A[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_3  = _mm_sub_sd( z_3, y_3 );
		y_3  = _mm_mul_sd( y_3, a_11 );
		_mm_store_sd( &x[3], y_3 );

		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_2  = _mm_load_sd( &x[2] );
		z_2  = _mm_fnmadd_sd( a_10, y_3, z_2 );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		y_2_3   = _mm_shuffle_pd( y_2, y_3, 0x0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		y_0     = _mm_fmadd_pd( a_20_30, y_2_3, y_0 );
		y_1     = _mm_fmadd_pd( a_21_31, y_2_3, y_1 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}



// it moves vertically across blocks
void kernel_dtrsv_t_3_lib4_new(int kmax, double *A, int sda, int use_inv_diag_A, double *inv_diag_A, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		mask,
		zeros,
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32,
		x_0_1_2_3,
		y_00, y_11, y_22,
		y_0b, y_1b, y_2b;
	
	__m128d
		tmp,
		ones,
		a_00, a_01, a_02, a_10, a_11, a_20, a_21,
		x_0,
		y_2_3,
		z_0, z_1, z_2,
		y_0, y_1, y_2;

	k = 3;
	if(kmax>4)
		{
		
		mask = _mm256_loadu_pd( mask_f ); 

		zeros = _mm256_setzero_pd();

		y_00 = _mm256_setzero_pd();
		y_11 = _mm256_setzero_pd();
		y_22 = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		y_1b = _mm256_setzero_pd();
		y_2b = _mm256_setzero_pd();
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x7 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k=4;
	//	for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
			
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1b = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1b );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2b = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2b );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}

		y_00 = _mm256_add_pd( y_00, y_0b );
		y_11 = _mm256_add_pd( y_11, y_1b );
		y_22 = _mm256_add_pd( y_22, y_2b );

//		for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
			
	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}
		
		y_0 = _mm256_extractf128_pd( y_00, 0x1 );
		y_1 = _mm256_extractf128_pd( y_11, 0x1 );
		y_2 = _mm256_extractf128_pd( y_22, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_00 ) );
		y_1 = _mm_add_pd( y_1, _mm256_castpd256_pd128( y_11 ) );
		y_2 = _mm_add_pd( y_2, _mm256_castpd256_pd128( y_22 ) );
		}
	else // tract as scalar
		{

		A += 3;
		x += 3;

		y_0 = _mm_setzero_pd();
		y_1 = _mm_setzero_pd();
		y_2 = _mm_setzero_pd();

		for(; k<kmax; k++)
			{
			
			x_0 = _mm_load_sd( &x[0] );

			a_00 = _mm_load_sd( &A[0+lda*0] );
			a_01 = _mm_load_sd( &A[0+lda*1] );
			a_02 = _mm_load_sd( &A[0+lda*2] );

			y_0 = _mm_fmadd_sd( a_00, x_0, y_0);
			y_1 = _mm_fmadd_sd( a_01, x_0, y_1);
			y_2 = _mm_fmadd_sd( a_02, x_0, y_2);
			
			A += 1;//sda*bs;
			x += 1;

			}

		}
	
	A = tA;
	x = tx;
			
	if(use_inv_diag_A)
		{
		// bottom trinagle
		z_2  = _mm_load_sd( &x[2] );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		a_20 = _mm_load_sd( &A[2+lda*0] );
		a_21 = _mm_load_sd( &A[2+lda*1] );
		a_20 = _mm_mul_sd( a_20, y_2 );
		a_21 = _mm_mul_sd( a_21, y_2 );
		y_0  = _mm_add_sd( y_0, a_20 );
		y_1  = _mm_add_sd( y_1, a_21 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &inv_diag_A[1] );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_2  = _mm_load_sd( &x[2] );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		a_20 = _mm_load_sd( &A[2+lda*0] );
		a_21 = _mm_load_sd( &A[2+lda*1] );
		a_20 = _mm_mul_sd( a_20, y_2 );
		a_21 = _mm_mul_sd( a_21, y_2 );
		y_0  = _mm_add_sd( y_0, a_20 );
		y_1  = _mm_add_sd( y_1, a_21 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}



// it moves vertically across blocks (A is supposed to be aligned)
void kernel_dtrsv_t_2_lib4_new(int kmax, double *A, int sda, int use_inv_diag_A, double *inv_diag_A, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		mask,
		zeros,
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31,
		x_0_1_2_3,
		y_00, y_11,
		y_0b, y_1b;
	
	__m128d
		ones,
		tmp,
		a_00, a_01, a_10, a_11,
		x_0,
		z_0, z_1,
		y_0, y_1;

	k = 2;
	if(kmax>4)
		{

		mask = _mm256_loadu_pd( mask_f ); 

		zeros = _mm256_setzero_pd();

		y_00 = _mm256_setzero_pd();
		y_11 = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		y_1b = _mm256_setzero_pd();
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x3 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k=4;
		//for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1b = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1b );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		
		y_00 = _mm256_add_pd( y_00, y_0b );
		y_11 = _mm256_add_pd( y_11, y_1b );

		//for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );

	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}
			
		y_0 = _mm256_extractf128_pd( y_00, 0x1 );
		y_1 = _mm256_extractf128_pd( y_11, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_00 ) );
		y_1 = _mm_add_pd( y_1, _mm256_castpd256_pd128( y_11 ) );
		}
	else // tract as scalar
		{

		A += 2;
		x += 2;

		y_0 = _mm_setzero_pd();
		y_1 = _mm_setzero_pd();

		for(; k<kmax; k++)
			{
			
			x_0 = _mm_load_sd( &x[0] );

			a_00 = _mm_load_sd( &A[0+lda*0] );
			a_01 = _mm_load_sd( &A[0+lda*1] );

			y_0 = _mm_fmadd_sd( a_00, x_0, y_0);
			y_1 = _mm_fmadd_sd( a_01, x_0, y_1);
			
			A += 1;//sda*bs;
			x += 1;

			}

		}
		
	A = tA;
	x = tx;
		
		//
	
	if(use_inv_diag_A)
		{
		// bottom trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &inv_diag_A[1] );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}



// it moves vertically across blocks
void kernel_dtrsv_t_1_lib4_new(int kmax, double *A, int sda, int use_inv_diag_A, double *inv_diag_A, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		mask,
		zeros,
		tmp0,
		a_00_10_20_30,
		x_0_1_2_3,
		y_00,
		y_0b;
	
	__m128d
		ones,
		tmp,
		a_00,
		x_0,
		z_0,
		y_0;

	k = 1;
	if(kmax>4)
		{
		mask = _mm256_loadu_pd( mask_f ); 

		zeros = _mm256_setzero_pd();

		y_00 = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x1 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k=4;
		//for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		
		y_00 = _mm256_add_pd( y_00, y_0b );

		//for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			
	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}

		y_0 = _mm256_extractf128_pd( y_00, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_00 ) );
		
		}
	else // tract as scalar
		{

		A += 1;
		x += 1;

		y_0 = _mm_setzero_pd();

		for(; k<kmax; k++)
			{
			
			x_0 = _mm_load_sd( &x[0] );

			a_00 = _mm_load_sd( &A[0+lda*0] );

			y_0 = _mm_fmadd_sd( a_00, x_0, y_0);
			
			A += 1;//sda*bs;
			x += 1;

			}

		}
	
	A = tA;
	x = tx;

	if(use_inv_diag_A)
		{
		// bottom trinagle
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}





// old kernels

// it moves horizontally inside a block
void kernel_dtrsv_n_8_lib4(int kmax, int inverted_diag, double *A0, int sda, double *x, double *y)
	{
	
	double *A1 = A0 + 4*sda;

/*printf("\nciao\n");*/

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	__m256d
		a_0,
		x_0, x_1,
		y_0, y_0_b, y_0_c, y_0_d,
		y_4, y_4_b, y_4_c, y_4_d,
		z_0_1_2_3, z_4_5_6_7;
	
	y_0   = _mm256_setzero_pd();	
	y_4   = _mm256_setzero_pd();	
	y_0_b = _mm256_setzero_pd();	
	y_4_b = _mm256_setzero_pd();	
	y_0_c = _mm256_setzero_pd();	
	y_4_c = _mm256_setzero_pd();	
	y_0_d = _mm256_setzero_pd();	
	y_4_d = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-7; k+=8)
		{

//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*1] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*2] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*3] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
	
//		__builtin_prefetch( A0 + 4*lda );
//		__builtin_prefetch( A1 + 4*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*4] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*4] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*5] );
		y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
		a_0 = _mm256_load_pd( &A1[0+lda*5] );
		y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );

//		__builtin_prefetch( A0 + 5*lda );
//		__builtin_prefetch( A1 + 5*lda );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A0[0+lda*6] );
		y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
		a_0 = _mm256_load_pd( &A1[0+lda*6] );
		y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A0[0+lda*7] );
		y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
		a_0 = _mm256_load_pd( &A1[0+lda*7] );
		y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );
	
		A0 += 8*lda;
		A1 += 8*lda;
		x  += 8;

		}
	
	y_0   = _mm256_add_pd( y_0, y_0_c );
	y_4   = _mm256_add_pd( y_4, y_4_c );
	y_0_b = _mm256_add_pd( y_0_b, y_0_d );
	y_4_b = _mm256_add_pd( y_4_b, y_4_d );

	y_0 = _mm256_add_pd( y_0, y_0_b );
	y_4 = _mm256_add_pd( y_4, y_4_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_4_5_6_7 = _mm256_loadu_pd( &y[4] );

	z_0_1_2_3 = _mm256_sub_pd( z_0_1_2_3, y_0 );
	z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, y_4 );

	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	if(inverted_diag)
		{
		// A_00
		z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

		a_00 = _mm_load_sd( &A0[0+lda*0] );
		a_10 = _mm_load_sd( &A0[1+lda*0] );
		a_11 = _mm_load_sd( &A0[1+lda*1] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+lda*0] );
		a_21_31 = _mm_load_pd( &A0[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );
		
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		
		

		// A_11
		a_00 = _mm_load_sd( &A0[2+lda*2] );
		a_10 = _mm_load_sd( &A0[3+lda*2] );
		a_11 = _mm_load_sd( &A0[3+lda*3] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );



		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &A1[0+lda*4] );
		a_10 = _mm_load_sd( &A1[1+lda*4] );
		a_11 = _mm_load_sd( &A1[1+lda*5] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+lda*4] );
		a_21_31 = _mm_load_pd( &A1[2+lda*5] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );



		// A_33
		a_00 = _mm_load_sd( &A1[2+lda*6] );
		a_10 = _mm_load_sd( &A1[3+lda*6] );
		a_11 = _mm_load_sd( &A1[3+lda*7] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[6], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[7], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// A_00
		z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

		a_00 = _mm_load_sd( &A0[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[1+lda*0] );
		a_11 = _mm_load_sd( &A0[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+lda*0] );
		a_21_31 = _mm_load_pd( &A0[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );
		
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*1] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );
		
		

		// A_11
		a_00 = _mm_load_sd( &A0[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[3+lda*2] );
		a_11 = _mm_load_sd( &A0[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A1[0+lda*2] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_0, z_4_5_6_7 );
		a_0 = _mm256_load_pd( &A1[0+lda*3] );
		z_4_5_6_7 = _mm256_fnmadd_pd( a_0, x_1, z_4_5_6_7 );



		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &A1[0+lda*4] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[1+lda*4] );
		a_11 = _mm_load_sd( &A1[1+lda*5] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+lda*4] );
		a_21_31 = _mm_load_pd( &A1[2+lda*5] );
		z_1   = _mm_fnmadd_pd( a_10, z_0, z_1 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0, z_2_3 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1, z_2_3 );



		// A_33
		a_00 = _mm_load_sd( &A1[2+lda*6] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[3+lda*6] );
		a_11 = _mm_load_sd( &A1[3+lda*7] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[6], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[7], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}


	}



// it moves horizontally inside a block ( assume ksv>0 !!! )
void kernel_dtrsv_n_4_lib4(int kmax, int inverted_diag, double *A, double *x, double *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	__m256d
		a_0,
		x_0, x_1,
		y_0_1_2_3  , y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3,
		y_0_1_2_3_e, y_0_1_2_3_f, y_0_1_2_3_g, y_0_1_2_3_h;
	
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	
	y_0_1_2_3_e = _mm256_setzero_pd();	
	y_0_1_2_3_f = _mm256_setzero_pd();	
	y_0_1_2_3_g = _mm256_setzero_pd();	
	y_0_1_2_3_h = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-7; k+=8)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
		x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*4] );
		y_0_1_2_3_e = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_e );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*5] );
		y_0_1_2_3_f = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_f );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*6] );
		y_0_1_2_3_g = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_g );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*7] );
		y_0_1_2_3_h = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_h );
	
		A += 8*lda;
		x += 8;

		}

	y_0_1_2_3   = _mm256_add_pd( y_0_1_2_3  , y_0_1_2_3_e );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_f );
	y_0_1_2_3_c = _mm256_add_pd( y_0_1_2_3_c, y_0_1_2_3_g );
	y_0_1_2_3_d = _mm256_add_pd( y_0_1_2_3_d, y_0_1_2_3_h );

	for(; k<kmax-3; k+=4)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
	
		A += 4*lda;
		x += 4;

		}
	
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_c );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_d );
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_0_1_2_3 = _mm256_sub_pd ( z_0_1_2_3, y_0_1_2_3 );
	
	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
	z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

	if(inverted_diag)
		{
		// a_00
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
		
		// a_11
		a_11 = _mm_load_sd( &A[1+lda*1] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );

		// a_22
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/

		// a_33
		a_11 = _mm_load_sd( &A[3+lda*3] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// a_00
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
		
		// a_11
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );

		// a_22
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/

		// a_33
		a_11 = _mm_load_sd( &A[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}

	}



// it moves horizontally inside a block ( assume ksv>0 !!! )
void kernel_dtrsv_n_4_vs_lib4(int km, int kn, int kmax, int inverted_diag, double *A, double *x, double *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	__m256d
		a_0,
		x_0, x_1,
		y_0_1_2_3  , y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3,
		y_0_1_2_3_e, y_0_1_2_3_f, y_0_1_2_3_g, y_0_1_2_3_h;
	
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	
	y_0_1_2_3_e = _mm256_setzero_pd();	
	y_0_1_2_3_f = _mm256_setzero_pd();	
	y_0_1_2_3_g = _mm256_setzero_pd();	
	y_0_1_2_3_h = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-7; k+=8)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
		x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*4] );
		y_0_1_2_3_e = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_e );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*5] );
		y_0_1_2_3_f = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_f );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*6] );
		y_0_1_2_3_g = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_g );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*7] );
		y_0_1_2_3_h = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_h );
	
		A += 8*lda;
		x += 8;

		}

	y_0_1_2_3   = _mm256_add_pd( y_0_1_2_3  , y_0_1_2_3_e );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_f );
	y_0_1_2_3_c = _mm256_add_pd( y_0_1_2_3_c, y_0_1_2_3_g );
	y_0_1_2_3_d = _mm256_add_pd( y_0_1_2_3_d, y_0_1_2_3_h );

	for(; k<kmax-3; k+=4)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*2] );
		y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*3] );
		y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
		
	
		A += 4*lda;
		x += 4;

		}
	
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_c );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_d );
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_0_1_2_3 = _mm256_sub_pd ( z_0_1_2_3, y_0_1_2_3 );
	
	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
	z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

	if(inverted_diag)
		{
		// a_00
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
			if(kn==1)
				{
				if(km==1)
					return;
				if(km<3) // km==2
					{
					_mm_store_sd( &y[1], z_1 );
					}
				else if(km==3)
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_sd( &y[2], z_2_3 );
					}
				else // km>=4
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_pd( &y[2], z_2_3 );
					}
				return;
				}
		
		// a_11
		a_11 = _mm_load_sd( &A[1+lda*1] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );
			if(kn==2)
				{
				if(km==2)
					return;
				if(km==3)
					_mm_store_sd( &y[2], z_2_3 );
				else // km>=4
					_mm_store_pd( &y[2], z_2_3 );
				return;
				}

		// a_22
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
			if(kn==3)
				{
				if(km==3)
					return;
				_mm_store_sd( &y[3], z_1 );
				return;
				}

		// a_33
		a_11 = _mm_load_sd( &A[3+lda*3] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// a_00
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_2_3 = _mm_fnmadd_pd( a_20_30, z_0,  z_2_3 );
			if(kn==1)
				{
				if(km==1)
					return;
				if(km<3) // km==2
					{
					_mm_store_sd( &y[1], z_1 );
					}
				else if(km==3)
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_sd( &y[2], z_2_3 );
					}
				else // km>=4
					{
					_mm_store_sd( &y[1], z_1 );
					_mm_store_pd( &y[2], z_2_3 );
					}
				return;
				}
		
		// a_11
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		z_2_3 = _mm_fnmadd_pd( a_21_31, z_1,  z_2_3 );
			if(kn==2)
				{
				if(km==2)
					return;
				if(km==3)
					_mm_store_sd( &y[2], z_2_3 );
				else // km>=4
					_mm_store_pd( &y[2], z_2_3 );
				return;
				}

		// a_22
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		z_1   = _mm_fnmadd_pd( a_10, z_0,  z_1 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
			if(kn==3)
				{
				if(km==3)
					return;
				_mm_store_sd( &y[3], z_1 );
				return;
				}

		// a_33
		a_11 = _mm_load_sd( &A[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
	/*	z_1   = _mm_movedup_pd( z_1 );*/
		}
	
		

	}



// it moves vertically across blocks
void kernel_dtrsv_t_4_lib4(int kmax, int inverted_diag, double *A, int sda, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		zeros,
		mask,
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0_1_2_3,
		y_0a, y_1a, y_2a, y_3a,
		y_0b, y_1b, y_2b, y_3b;
	
	__m128d
/*		tmp,*/
		ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		y_2_3,
		z_0, z_1, z_2, z_3,
		y_0, y_1, y_2, y_3;
		mask = _mm256_loadu_pd( mask_f ); 
	
	k=4;
	if(kmax>4)
		{

		zeros = _mm256_setzero_pd();

		y_0a = _mm256_setzero_pd();
		y_1a = _mm256_setzero_pd();
		y_2a = _mm256_setzero_pd();
		y_3a = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		y_1b = _mm256_setzero_pd();
		y_2b = _mm256_setzero_pd();
		y_3b = _mm256_setzero_pd();
		
		A += 4 + (sda-1)*lda;
		x += 4;
		//for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0a = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0a );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1a = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1a );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2a = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2a );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3a = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3a );
			
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1b = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1b );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2b = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2b );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3b = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3b );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		
		y_0a = _mm256_add_pd( y_0a, y_0b );
		y_1a = _mm256_add_pd( y_1a, y_1b );
		y_2a = _mm256_add_pd( y_2a, y_2b );
		y_3a = _mm256_add_pd( y_3a, y_3b );

		//for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0a = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0a );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1a = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1a );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2a = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2a );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3a = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3a );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0a = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0a );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1a = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1a );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2a = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2a );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
			y_3a = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_3a );
			
	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}
			
		A = tA;
		x = tx;

		y_0 = _mm256_extractf128_pd( y_0a, 0x1 );
		y_1 = _mm256_extractf128_pd( y_1a, 0x1 );
		y_2 = _mm256_extractf128_pd( y_2a, 0x1 );
		y_3 = _mm256_extractf128_pd( y_3a, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_0a ) );
		y_1 = _mm_add_pd( y_1, _mm256_castpd256_pd128( y_1a ) );
		y_2 = _mm_add_pd( y_2, _mm256_castpd256_pd128( y_2a ) );
		y_3 = _mm_add_pd( y_3, _mm256_castpd256_pd128( y_3a ) );
		}
	else
		{
		y_0 = _mm_setzero_pd();
		y_1 = _mm_setzero_pd();
		y_2 = _mm_setzero_pd();
		y_3 = _mm_setzero_pd();
		}
			
	if(inverted_diag)
		{
		// bottom trinagle
		z_3  = _mm_load_sd( &x[3] );
		y_3  = _mm_hadd_pd( y_3, y_3 );
		a_11 = _mm_load_sd( &A[3+lda*3] );
		y_3  = _mm_sub_sd( z_3, y_3 );
		y_3  = _mm_mul_sd( y_3, a_11 );
		_mm_store_sd( &x[3], y_3 );

		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_2  = _mm_load_sd( &x[2] );
		z_2  = _mm_fnmadd_sd( a_10, y_3, z_2 );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &A[2+lda*2] );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		y_2_3   = _mm_shuffle_pd( y_2, y_3, 0x0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		y_0     = _mm_fmadd_pd( a_20_30, y_2_3, y_0 );
		y_1     = _mm_fmadd_pd( a_21_31, y_2_3, y_1 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_3  = _mm_load_sd( &x[3] );
		y_3  = _mm_hadd_pd( y_3, y_3 );
		a_11 = _mm_load_sd( &A[3+lda*3] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_3  = _mm_sub_sd( z_3, y_3 );
		y_3  = _mm_mul_sd( y_3, a_11 );
		_mm_store_sd( &x[3], y_3 );

		a_10 = _mm_load_sd( &A[3+lda*2] );
		z_2  = _mm_load_sd( &x[2] );
		z_2  = _mm_fnmadd_sd( a_10, y_3, z_2 );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		y_2_3   = _mm_shuffle_pd( y_2, y_3, 0x0 );
		a_20_30 = _mm_load_pd( &A[2+lda*0] );
		a_21_31 = _mm_load_pd( &A[2+lda*1] );
		y_0     = _mm_fmadd_pd( a_20_30, y_2_3, y_0 );
		y_1     = _mm_fmadd_pd( a_21_31, y_2_3, y_1 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}



// it moves vertically across blocks
void kernel_dtrsv_t_3_lib4(int kmax, int inverted_diag, double *A, int sda, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		mask,
		zeros,
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32,
		x_0_1_2_3,
		y_00, y_11, y_22,
		y_0b, y_1b, y_2b;
	
	__m128d
		tmp,
		ones,
		a_00, a_01, a_02, a_10, a_11, a_20, a_21,
		x_0,
		y_2_3,
		z_0, z_1, z_2,
		y_0, y_1, y_2;

	k = 3;
	if(kmax>4)
		{
		
		mask = _mm256_loadu_pd( mask_f ); 

		zeros = _mm256_setzero_pd();

		y_00 = _mm256_setzero_pd();
		y_11 = _mm256_setzero_pd();
		y_22 = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		y_1b = _mm256_setzero_pd();
		y_2b = _mm256_setzero_pd();
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x7 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k=4;
	//	for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
			
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1b = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1b );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_2b = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_2b );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}

		y_00 = _mm256_add_pd( y_00, y_0b );
		y_11 = _mm256_add_pd( y_11, y_1b );
		y_22 = _mm256_add_pd( y_22, y_2b );

//		for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
			
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
			
	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}
		
		y_0 = _mm256_extractf128_pd( y_00, 0x1 );
		y_1 = _mm256_extractf128_pd( y_11, 0x1 );
		y_2 = _mm256_extractf128_pd( y_22, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_00 ) );
		y_1 = _mm_add_pd( y_1, _mm256_castpd256_pd128( y_11 ) );
		y_2 = _mm_add_pd( y_2, _mm256_castpd256_pd128( y_22 ) );
		}
	else // tract as scalar
		{

		A += 3;
		x += 3;

		y_0 = _mm_setzero_pd();
		y_1 = _mm_setzero_pd();
		y_2 = _mm_setzero_pd();

		for(; k<kmax; k++)
			{
			
			x_0 = _mm_load_sd( &x[0] );

			a_00 = _mm_load_sd( &A[0+lda*0] );
			a_01 = _mm_load_sd( &A[0+lda*1] );
			a_02 = _mm_load_sd( &A[0+lda*2] );

			y_0 = _mm_fmadd_sd( a_00, x_0, y_0);
			y_1 = _mm_fmadd_sd( a_01, x_0, y_1);
			y_2 = _mm_fmadd_sd( a_02, x_0, y_2);
			
			A += 1;//sda*bs;
			x += 1;

			}

		}
	
	A = tA;
	x = tx;
			
	if(inverted_diag)
		{
		// bottom trinagle
		z_2  = _mm_load_sd( &x[2] );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &A[2+lda*2] );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		a_20 = _mm_load_sd( &A[2+lda*0] );
		a_21 = _mm_load_sd( &A[2+lda*1] );
		a_20 = _mm_mul_sd( a_20, y_2 );
		a_21 = _mm_mul_sd( a_21, y_2 );
		y_0  = _mm_add_sd( y_0, a_20 );
		y_1  = _mm_add_sd( y_1, a_21 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_2  = _mm_load_sd( &x[2] );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &A[2+lda*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		a_20 = _mm_load_sd( &A[2+lda*0] );
		a_21 = _mm_load_sd( &A[2+lda*1] );
		a_20 = _mm_mul_sd( a_20, y_2 );
		a_21 = _mm_mul_sd( a_21, y_2 );
		y_0  = _mm_add_sd( y_0, a_20 );
		y_1  = _mm_add_sd( y_1, a_21 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}



// it moves vertically across blocks (A is supposed to be aligned)
void kernel_dtrsv_t_2_lib4(int kmax, int inverted_diag, double *A, int sda, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		mask,
		zeros,
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31,
		x_0_1_2_3,
		y_00, y_11,
		y_0b, y_1b;
	
	__m128d
		ones,
		tmp,
		a_00, a_01, a_10, a_11,
		x_0,
		z_0, z_1,
		y_0, y_1;

	k = 2;
	if(kmax>4)
		{

		mask = _mm256_loadu_pd( mask_f ); 

		zeros = _mm256_setzero_pd();

		y_00 = _mm256_setzero_pd();
		y_11 = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		y_1b = _mm256_setzero_pd();
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x3 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k=4;
		//for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_1b = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_1b );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		
		y_00 = _mm256_add_pd( y_00, y_0b );
		y_11 = _mm256_add_pd( y_11, y_1b );

		//for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );

	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}
			
		y_0 = _mm256_extractf128_pd( y_00, 0x1 );
		y_1 = _mm256_extractf128_pd( y_11, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_00 ) );
		y_1 = _mm_add_pd( y_1, _mm256_castpd256_pd128( y_11 ) );
		}
	else // tract as scalar
		{

		A += 2;
		x += 2;

		y_0 = _mm_setzero_pd();
		y_1 = _mm_setzero_pd();

		for(; k<kmax; k++)
			{
			
			x_0 = _mm_load_sd( &x[0] );

			a_00 = _mm_load_sd( &A[0+lda*0] );
			a_01 = _mm_load_sd( &A[0+lda*1] );

			y_0 = _mm_fmadd_sd( a_00, x_0, y_0);
			y_1 = _mm_fmadd_sd( a_01, x_0, y_1);
			
			A += 1;//sda*bs;
			x += 1;

			}

		}
		
	A = tA;
	x = tx;
		
		//
	
	if(inverted_diag)
		{
		// bottom trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+lda*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+lda*0] );
		z_0  = _mm_load_sd( &x[0] );
		z_0  = _mm_fnmadd_sd( a_10, y_1, z_0 );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}



// it moves vertically across blocks
void kernel_dtrsv_t_1_lib4(int kmax, int inverted_diag, double *A, int sda, double *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

	double *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const double mask_f[] = {3.5, 2.5, 1.5, 0.5};

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		mask,
		zeros,
		tmp0,
		a_00_10_20_30,
		x_0_1_2_3,
		y_00,
		y_0b;
	
	__m128d
		ones,
		tmp,
		a_00,
		x_0,
		z_0,
		y_0;

	k = 1;
	if(kmax>4)
		{
		mask = _mm256_loadu_pd( mask_f ); 

		zeros = _mm256_setzero_pd();

		y_00 = _mm256_setzero_pd();
		y_0b = _mm256_setzero_pd();
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x1 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k=4;
		//for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		
			A += 4 + (sda-1)*lda;
			x += 4;


	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		
		y_00 = _mm256_add_pd( y_00, y_0b );

		//for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		
			A += 4 + (sda-1)*lda;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
	/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
			
	/*		A += 4 + (sda-1)*lda;*/
	/*		x += 4;*/

			}

		y_0 = _mm256_extractf128_pd( y_00, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_00 ) );
		
		}
	else // tract as scalar
		{

		A += 1;
		x += 1;

		y_0 = _mm_setzero_pd();

		for(; k<kmax; k++)
			{
			
			x_0 = _mm_load_sd( &x[0] );

			a_00 = _mm_load_sd( &A[0+lda*0] );

			y_0 = _mm_fmadd_sd( a_00, x_0, y_0);
			
			A += 1;//sda*bs;
			x += 1;

			}

		}
	
	A = tA;
	x = tx;

	if(inverted_diag)
		{
		// bottom trinagle
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones  = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}

#endif
