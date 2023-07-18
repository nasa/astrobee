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

// it moves horizontally inside a block
void kernel_dtrsv_n_8_lib4_new(int kmax, double *A0, int sda, int use_inv_diag_A, double *inv_diag_A, double *x, double *y)
	{
	
	double *A1 = A0 + 4*sda;

/*	if(kmax<=0) */
/*		return;*/
	
	const int bs = 4;
	
	int k;

	__m256d
		ax_temp, temp0, temp1,
		a_00_10_20_30, a_01_11_21_31,
		a_40_50_60_70, a_41_51_61_71,
		x_0, x_1,
		y_0_1_2_3, y_0_1_2_3_b, z_0_1_2_3,
		y_4_5_6_7, y_4_5_6_7_b, z_4_5_6_7;
	
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_4_5_6_7   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_4_5_6_7_b = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-7; k+=8)
		{

/*		__builtin_prefetch( A0 + 4*bs );*/
/*		__builtin_prefetch( A1 + 4*bs );*/

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+bs*0] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*0] );
		a_01_11_21_31 = _mm256_load_pd( &A0[0+bs*1] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*1] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
		y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

/*		__builtin_prefetch( A0 + 5*bs );*/
/*		__builtin_prefetch( A1 + 5*bs );*/

		x_0 = _mm256_broadcast_sd( &x[2] );
		x_1 = _mm256_broadcast_sd( &x[3] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+bs*2] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*2] );
		a_01_11_21_31 = _mm256_load_pd( &A0[0+bs*3] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*3] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
		y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
	
		A0 += 4*bs;
		A1 += 4*bs;
		x  += 4;

/*		__builtin_prefetch( A0 + 4*bs );*/
/*		__builtin_prefetch( A1 + 4*bs );*/

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+bs*0] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*0] );
		a_01_11_21_31 = _mm256_load_pd( &A0[0+bs*1] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*1] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
		y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

/*		__builtin_prefetch( A0 + 5*bs );*/
/*		__builtin_prefetch( A1 + 5*bs );*/

		x_0 = _mm256_broadcast_sd( &x[2] );
		x_1 = _mm256_broadcast_sd( &x[3] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+bs*2] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*2] );
		a_01_11_21_31 = _mm256_load_pd( &A0[0+bs*3] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*3] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
		y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
	
		A0 += 4*bs;
		A1 += 4*bs;
		x  += 4;

		}
	
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_b );
	y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, y_4_5_6_7_b );

	z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	z_4_5_6_7 = _mm256_loadu_pd( &y[4] );
	z_0_1_2_3 = _mm256_sub_pd( z_0_1_2_3, y_0_1_2_3 );
	z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, y_4_5_6_7 );

	// solve

	__m128d
		zeros, ones,
		a_00, a_10, a_11, a_20_30, a_21_31,
		z_0, z_1,
		z_0_1, z_2_3, tmp0, tmp1;
	
	zeros = _mm_setzero_pd();
	
	if(use_inv_diag_A)
		{
		// A_00
		z_2_3 = _mm256_extractf128_pd( z_0_1_2_3, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_0_1_2_3 );

		a_00 = _mm_load_sd( &inv_diag_A[0] );
		a_10 = _mm_load_sd( &A0[1+bs*0] );
		a_11 = _mm_load_sd( &inv_diag_A[1] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+bs*0] );
		a_21_31 = _mm_load_pd( &A0[2+bs*1] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );
		
		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*0] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*1] );
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		temp0 = _mm256_mul_pd( a_40_50_60_70, x_0 );
		temp1 = _mm256_mul_pd( a_41_51_61_71, x_1 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp0 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp1 );
		
		

		// A_11
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		a_10 = _mm_load_sd( &A0[3+bs*2] );
		a_11 = _mm_load_sd( &inv_diag_A[3] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*2] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*3] );
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		temp0 = _mm256_mul_pd( a_40_50_60_70, x_0 );
		temp1 = _mm256_mul_pd( a_41_51_61_71, x_1 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp0 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp1 );



		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &inv_diag_A[4] );
		a_10 = _mm_load_sd( &A1[1+bs*4] );
		a_11 = _mm_load_sd( &inv_diag_A[5] );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+bs*4] );
		a_21_31 = _mm_load_pd( &A1[2+bs*5] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );



		// A_33
		a_00 = _mm_load_sd( &inv_diag_A[6] );
		a_10 = _mm_load_sd( &A1[3+bs*6] );
		a_11 = _mm_load_sd( &inv_diag_A[7] );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[6], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_sub_pd( z_1, tmp0 );
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

		a_00 = _mm_load_sd( &A0[0+bs*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[1+bs*0] );
		a_11 = _mm_load_sd( &A0[1+bs*1] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A0[2+bs*0] );
		a_21_31 = _mm_load_pd( &A0[2+bs*1] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );
		
		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*0] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*1] );
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		temp0 = _mm256_mul_pd( a_40_50_60_70, x_0 );
		temp1 = _mm256_mul_pd( a_41_51_61_71, x_1 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp0 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp1 );
		
		

		// A_11
		a_00 = _mm_load_sd( &A0[2+bs*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A0[3+bs*2] );
		a_11 = _mm_load_sd( &A0[3+bs*3] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[2], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[3], z_1 );
		z_1   = _mm_movedup_pd( z_1 );

		a_40_50_60_70 = _mm256_load_pd( &A1[0+bs*2] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+bs*3] );
		x_0   = _mm256_castpd128_pd256( z_0 );
		x_1   = _mm256_castpd128_pd256( z_1 );
		x_0   = _mm256_permute2f128_pd( x_0, x_0, 0x0 );
		x_1   = _mm256_permute2f128_pd( x_1, x_1, 0x0 );
		temp0 = _mm256_mul_pd( a_40_50_60_70, x_0 );
		temp1 = _mm256_mul_pd( a_41_51_61_71, x_1 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp0 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, temp1 );



		// A_22
		z_2_3 = _mm256_extractf128_pd( z_4_5_6_7, 0x1 );
		z_0_1 = _mm256_castpd256_pd128( z_4_5_6_7 );

		a_00 = _mm_load_sd( &A1[0+bs*4] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[1+bs*4] );
		a_11 = _mm_load_sd( &A1[1+bs*5] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A1[2+bs*4] );
		a_21_31 = _mm_load_pd( &A1[2+bs*5] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[4], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
		_mm_store_sd( &y[5], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );



		// A_33
		a_00 = _mm_load_sd( &A1[2+bs*6] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A1[3+bs*6] );
		a_11 = _mm_load_sd( &A1[3+bs*7] );
		a_11 = _mm_div_sd( ones, a_11 );

		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[6], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_sub_pd( z_1, tmp0 );
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
	
	const int bs = 4;
	
	int k;

	__m256d
		ax_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0, x_1, x_2, x_3,
		y_0_1_2_3, y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3;
	
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );

		x_2 = _mm256_broadcast_sd( &x[2] );
		x_3 = _mm256_broadcast_sd( &x[3] );

		a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+bs*3] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );

		ax_temp = _mm256_mul_pd( a_02_12_22_32, x_2 );
		y_0_1_2_3_c = _mm256_add_pd( y_0_1_2_3_c, ax_temp );
		ax_temp = _mm256_mul_pd( a_03_13_23_33, x_3 );
		y_0_1_2_3_d = _mm256_add_pd( y_0_1_2_3_d, ax_temp );
		
		A += 4*bs;
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
		a_10 = _mm_load_sd( &A[1+bs*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+bs*0] );
		a_21_31 = _mm_load_pd( &A[2+bs*1] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
		
		// a_11
		a_11 = _mm_load_sd( &inv_diag_A[1] );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );

		// a_22
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		a_10 = _mm_load_sd( &A[3+bs*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_sub_pd( z_1, tmp0 );

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
		a_00 = _mm_load_sd( &A[0+bs*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[1+bs*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+bs*0] );
		a_21_31 = _mm_load_pd( &A[2+bs*1] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
		
		// a_11
		a_11 = _mm_load_sd( &A[1+bs*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );

		// a_22
		a_00 = _mm_load_sd( &A[2+bs*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[3+bs*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_sub_pd( z_1, tmp0 );

		// a_33
		a_11 = _mm_load_sd( &A[3+bs*3] );
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
	
	const int bs = 4;
	
	int k;

	__m256d
		ax_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0, x_1, x_2, x_3,
		y_0_1_2_3, y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3;
	
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	

	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );

		x_2 = _mm256_broadcast_sd( &x[2] );
		x_3 = _mm256_broadcast_sd( &x[3] );

		a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+bs*3] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );

		ax_temp = _mm256_mul_pd( a_02_12_22_32, x_2 );
		y_0_1_2_3_c = _mm256_add_pd( y_0_1_2_3_c, ax_temp );
		ax_temp = _mm256_mul_pd( a_03_13_23_33, x_3 );
		y_0_1_2_3_d = _mm256_add_pd( y_0_1_2_3_d, ax_temp );
		
		A += 4*bs;
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
		a_10 = _mm_load_sd( &A[1+bs*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+bs*0] );
		a_21_31 = _mm_load_pd( &A[2+bs*1] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
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
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );
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
		a_10 = _mm_load_sd( &A[3+bs*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_sub_pd( z_1, tmp0 );
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
		a_00 = _mm_load_sd( &A[0+bs*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[1+bs*0] );
		z_0   = _mm_shuffle_pd( z_0_1, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_0_1, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		a_20_30 = _mm_load_pd( &A[2+bs*0] );
		a_21_31 = _mm_load_pd( &A[2+bs*1] );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[0], z_0 );
		z_0   = _mm_movedup_pd( z_0 );
		z_1   = _mm_sub_pd( z_1, tmp0 );
		tmp0  = _mm_mul_pd( a_20_30, z_0 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp0 );
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
		a_11 = _mm_load_sd( &A[1+bs*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		z_1   = _mm_mul_sd( a_11, z_1 );
		_mm_store_sd( &y[1], z_1 );
		z_1   = _mm_movedup_pd( z_1 );
		tmp1  = _mm_mul_pd( a_21_31, z_1 );
		z_2_3 = _mm_sub_pd( z_2_3, tmp1 );
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
		a_00 = _mm_load_sd( &A[2+bs*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		a_10 = _mm_load_sd( &A[3+bs*2] );
		z_0   = _mm_shuffle_pd( z_2_3, zeros, 0x0 );
		z_1   = _mm_shuffle_pd( z_2_3, zeros, 0x1 );
		z_0   = _mm_mul_sd( a_00, z_0 );
		tmp0  = _mm_mul_sd( a_10, z_0 );
		_mm_store_sd( &y[2], z_0 );
	/*	z_0   = _mm_movedup_pd( z_0 );*/
		z_1   = _mm_sub_pd( z_1, tmp0 );
		if(kn==3)
			{
			if(km==3)
				return;
			_mm_store_sd( &y[3], z_1 );
			return;
			}

		// a_33
		a_11 = _mm_load_sd( &A[3+bs*3] );
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
	
	const int bs = 4;
	
/*	__builtin_prefetch( A + 0*bs );*/
/*	__builtin_prefetch( A + 2*bs );*/

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
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0_1_2_3,
		y_00, y_11, y_22, y_33;
	
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

		y_00 = _mm256_setzero_pd();
		y_11 = _mm256_setzero_pd();
		y_22 = _mm256_setzero_pd();
		y_33 = _mm256_setzero_pd();
		
		A += 4 + (sda-1)*bs;
		x += 4;
	/*	for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+bs*3] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			y_33 = _mm256_add_pd( y_33, tmp1 );
			
			A += 4 + (sda-1)*bs;
			x += 4;


	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+bs*3] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			y_33 = _mm256_add_pd( y_33, tmp1 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
	/*	for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+bs*3] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			y_33 = _mm256_add_pd( y_33, tmp1 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+bs*3] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			y_33 = _mm256_add_pd( y_33, tmp1 );
			
	/*		A += 4 + (sda-1)*bs;*/
	/*		x += 4;*/

			}
		
		A = tA;
		x = tx;

		y_0 = _mm256_extractf128_pd( y_00, 0x1 );
		y_1 = _mm256_extractf128_pd( y_11, 0x1 );
		y_2 = _mm256_extractf128_pd( y_22, 0x1 );
		y_3 = _mm256_extractf128_pd( y_33, 0x1 );
		
		y_0 = _mm_add_pd( y_0, _mm256_castpd256_pd128( y_00 ) );
		y_1 = _mm_add_pd( y_1, _mm256_castpd256_pd128( y_11 ) );
		y_2 = _mm_add_pd( y_2, _mm256_castpd256_pd128( y_22 ) );
		y_3 = _mm_add_pd( y_3, _mm256_castpd256_pd128( y_33 ) );
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

		a_10 = _mm_load_sd( &A[3+bs*2] );
		a_10 = _mm_mul_sd( a_10, y_3 );
		z_2  = _mm_load_sd( &x[2] );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		z_2  = _mm_sub_sd( z_2, a_10 );
		a_00 = _mm_load_sd( &inv_diag_A[2] );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		y_2_3   = _mm_shuffle_pd( y_2, y_3, 0x0 );
		a_20_30 = _mm_load_pd( &A[2+bs*0] );
		a_21_31 = _mm_load_pd( &A[2+bs*1] );
		a_20_30 = _mm_mul_pd( a_20_30, y_2_3 );
		a_21_31 = _mm_mul_pd( a_21_31, y_2_3 );
		y_0     = _mm_add_pd( y_0, a_20_30 );
		y_1     = _mm_add_pd( y_1, a_21_31 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &inv_diag_A[1] );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+bs*0] );
		a_10 = _mm_mul_sd( a_10, y_1 );
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		z_0  = _mm_sub_sd( z_0, a_10 );
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_3  = _mm_load_sd( &x[3] );
		y_3  = _mm_hadd_pd( y_3, y_3 );
		a_11 = _mm_load_sd( &A[3+bs*3] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_3  = _mm_sub_sd( z_3, y_3 );
		y_3  = _mm_mul_sd( y_3, a_11 );
		_mm_store_sd( &x[3], y_3 );

		a_10 = _mm_load_sd( &A[3+bs*2] );
		a_10 = _mm_mul_sd( a_10, y_3 );
		z_2  = _mm_load_sd( &x[2] );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		z_2  = _mm_sub_sd( z_2, a_10 );
		a_00 = _mm_load_sd( &A[2+bs*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		y_2_3   = _mm_shuffle_pd( y_2, y_3, 0x0 );
		a_20_30 = _mm_load_pd( &A[2+bs*0] );
		a_21_31 = _mm_load_pd( &A[2+bs*1] );
		a_20_30 = _mm_mul_pd( a_20_30, y_2_3 );
		a_21_31 = _mm_mul_pd( a_21_31, y_2_3 );
		y_0     = _mm_add_pd( y_0, a_20_30 );
		y_1     = _mm_add_pd( y_1, a_21_31 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+bs*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+bs*0] );
		a_10 = _mm_mul_sd( a_10, y_1 );
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		z_0  = _mm_sub_sd( z_0, a_10 );
		a_00 = _mm_load_sd( &A[0+bs*0] );
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
	
	const int bs = 4;
	
/*	__builtin_prefetch( A + 0*bs );*/
/*	__builtin_prefetch( A + 2*bs );*/

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
		y_00, y_11, y_22;
	
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

		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x7 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, tmp0 );

		A += 4 + (sda-1)*bs;
		x += 4;

		k=4;
	/*	for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			
			A += 4 + (sda-1)*bs;
			x += 4;


	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
	/*	for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+bs*2] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, tmp0 );
			
	/*		A += 4 + (sda-1)*bs;*/
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

			a_00 = _mm_load_sd( &A[0+bs*0] );
			a_01 = _mm_load_sd( &A[0+bs*1] );
			a_02 = _mm_load_sd( &A[0+bs*2] );

			tmp = _mm_mul_sd( a_00, x_0 );
			y_0 = _mm_add_sd( y_0, tmp);
			tmp = _mm_mul_sd( a_01, x_0 );
			y_1 = _mm_add_sd( y_1, tmp);
			tmp = _mm_mul_sd( a_02, x_0 );
			y_2 = _mm_add_sd( y_2, tmp);
			
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
		a_20 = _mm_load_sd( &A[2+bs*0] );
		a_21 = _mm_load_sd( &A[2+bs*1] );
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

		a_10 = _mm_load_sd( &A[1+bs*0] );
		a_10 = _mm_mul_sd( a_10, y_1 );
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		z_0  = _mm_sub_sd( z_0, a_10 );
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones = _mm_set_pd( 1.0, 1.0 );

		// bottom trinagle
		z_2  = _mm_load_sd( &x[2] );
		y_2  = _mm_hadd_pd( y_2, y_2 );
		a_00 = _mm_load_sd( &A[2+bs*2] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_2  = _mm_sub_sd( z_2, y_2 );
		y_2  = _mm_mul_sd( y_2, a_00 );
		_mm_store_sd( &x[2], y_2 );

		// square
		a_20 = _mm_load_sd( &A[2+bs*0] );
		a_21 = _mm_load_sd( &A[2+bs*1] );
		a_20 = _mm_mul_sd( a_20, y_2 );
		a_21 = _mm_mul_sd( a_21, y_2 );
		y_0  = _mm_add_sd( y_0, a_20 );
		y_1  = _mm_add_sd( y_1, a_21 );
			
		// top trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+bs*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+bs*0] );
		a_10 = _mm_mul_sd( a_10, y_1 );
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		z_0  = _mm_sub_sd( z_0, a_10 );
		a_00 = _mm_load_sd( &A[0+bs*0] );
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
	
	const int bs = 4;
	
/*	__builtin_prefetch( A + 0*bs );*/
/*	__builtin_prefetch( A + 2*bs );*/

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
		y_00, y_11;
	
	__m128d
		tmp,
		ones,
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
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x3 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );

		A += 4 + (sda-1)*bs;
		x += 4;

		k=4;
	/*	for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			
			A += 4 + (sda-1)*bs;
			x += 4;


	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
	/*	for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+bs*1] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			y_11 = _mm256_add_pd( y_11, tmp1 );
			
	/*		A += 4 + (sda-1)*bs;*/
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

			a_00 = _mm_load_sd( &A[0+bs*0] );
			a_01 = _mm_load_sd( &A[0+bs*1] );

			tmp = _mm_mul_sd( a_00, x_0 );
			y_0 = _mm_add_sd( y_0, tmp);
			tmp = _mm_mul_sd( a_01, x_0 );
			y_1 = _mm_add_sd( y_1, tmp);
			
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

		a_10 = _mm_load_sd( &A[1+bs*0] );
		a_10 = _mm_mul_sd( a_10, y_1 );
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		z_0  = _mm_sub_sd( z_0, a_10 );
		a_00 = _mm_load_sd( &inv_diag_A[0] );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}
	else
		{
		ones = _mm_set_pd( 1.0, 1.0 );
		// bottom trinagle
		z_1  = _mm_load_sd( &x[1] );
		y_1  = _mm_hadd_pd( y_1, y_1 );
		a_11 = _mm_load_sd( &A[1+bs*1] );
		a_11 = _mm_div_sd( ones, a_11 );
		y_1  = _mm_sub_sd( z_1, y_1 );
		y_1  = _mm_mul_sd( y_1, a_11 );
		_mm_store_sd( &x[1], y_1 );

		a_10 = _mm_load_sd( &A[1+bs*0] );
		a_10 = _mm_mul_sd( a_10, y_1 );
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		z_0  = _mm_sub_sd( z_0, a_10 );
		a_00 = _mm_load_sd( &A[0+bs*0] );
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
	
	const int bs = 4;
	
/*	__builtin_prefetch( A + 0*bs );*/
/*	__builtin_prefetch( A + 2*bs );*/

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
		y_00;
	
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
		
		// clean up at the beginning
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
		x_0_1_2_3 = _mm256_blend_pd( x_0_1_2_3, zeros, 0x1 );

		a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );

		A += 4 + (sda-1)*bs;
		x += 4;

		k=4;
	/*	for(; k<kmax-4; k+=8) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-7; k+=8)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			
			A += 4 + (sda-1)*bs;
			x += 4;


	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
	/*	for(; k<kmax; k+=4) // TODO correct end & mask !!!!!!!!!!!*/
		for(; k<kmax-3; k+=4)
			{
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			
			A += 4 + (sda-1)*bs;
			x += 4;

			}
		if(k<kmax)
			{
			
			k_left_d = 4.0 - (kmax - k);
			
	/*		__builtin_prefetch( A + sda*bs + 0*bs );*/
	/*		__builtin_prefetch( A + sda*bs + 2*bs );*/

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );
			x_0_1_2_3 = _mm256_blendv_pd( x_0_1_2_3, zeros, _mm256_sub_pd( mask, _mm256_broadcast_sd( &k_left_d) ) );

			a_00_10_20_30 = _mm256_load_pd( &A[0+bs*0] );
			
			tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, tmp0 );
			
	/*		A += 4 + (sda-1)*bs;*/
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

			a_00 = _mm_load_sd( &A[0+bs*0] );

			tmp = _mm_mul_sd( a_00, x_0 );
			y_0 = _mm_add_sd( y_0, tmp);
			
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
		// bottom trinagle
		ones = _mm_set_pd( 1.0, 1.0 );
		z_0  = _mm_load_sd( &x[0] );
		y_0  = _mm_hadd_pd( y_0, y_0 );
		a_00 = _mm_load_sd( &A[0+bs*0] );
		a_00 = _mm_div_sd( ones, a_00 );
		y_0  = _mm_sub_sd( z_0, y_0 );
		y_0  = _mm_mul_sd( y_0, a_00 );
		_mm_store_sd( &x[0], y_0 );
		}

	}


#endif


