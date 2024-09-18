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

// it moves vertically across blocks
void kernel_dtrmv_u_t_8_lib4(int kmax, double *A, int sda, double *x, double *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/
/*	__builtin_prefetch( A + 4*lda );*/
/*	__builtin_prefetch( A + 6*lda );*/

/*	double *tA, *tx;*/

	int k;
/*	int ka = kmax-kna; // number from aligned positon*/
	
	__m256d
		zeros,
		tmp0, tmp1, 
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0_1_2_3,
		y_00, y_11, y_22, y_33, y_44, y_55, y_66, y_77;
	
/*	__m128d*/
/*		ax_temp,*/
/*		a_00_10, a_01_11, a_02_12, a_03_13,*/
/*		x_0_1,*/
/*		y_0, y_1, y_2, y_3, y_4, y_5, y_6, y_7;*/
	
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_22 = _mm256_setzero_pd();
	y_33 = _mm256_setzero_pd();
	y_44 = _mm256_setzero_pd();
	y_55 = _mm256_setzero_pd();
	y_66 = _mm256_setzero_pd();
	y_77 = _mm256_setzero_pd();
		
	k=0;
	for(; k<kmax-7; k+=8)
		{

/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, tmp0 );
		y_33 = _mm256_add_pd( y_33, tmp1 );
	
/*		__builtin_prefetch( A + sda*lda + 4*lda );*/
/*		__builtin_prefetch( A + sda*lda + 6*lda );*/

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
	
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_44 = _mm256_add_pd( y_44, tmp0 );
		y_55 = _mm256_add_pd( y_55, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_66 = _mm256_add_pd( y_66, tmp0 );
		y_77 = _mm256_add_pd( y_77, tmp1 );

		A += 4 + (sda-1)*lda;
		x += 4;


/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, tmp0 );
		y_33 = _mm256_add_pd( y_33, tmp1 );
	
/*		__builtin_prefetch( A + sda*lda + 4*lda );*/
/*		__builtin_prefetch( A + sda*lda + 6*lda );*/

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
	
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_44 = _mm256_add_pd( y_44, tmp0 );
		y_55 = _mm256_add_pd( y_55, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_66 = _mm256_add_pd( y_66, tmp0 );
		y_77 = _mm256_add_pd( y_77, tmp1 );

		A += 4 + (sda-1)*lda;
		x += 4;

		}
	if(k<kmax-3)
		{

/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, tmp0 );
		y_33 = _mm256_add_pd( y_33, tmp1 );
	
/*		__builtin_prefetch( A + sda*lda + 4*lda );*/
/*		__builtin_prefetch( A + sda*lda + 6*lda );*/

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
	
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_44 = _mm256_add_pd( y_44, tmp0 );
		y_55 = _mm256_add_pd( y_55, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_66 = _mm256_add_pd( y_66, tmp0 );
		y_77 = _mm256_add_pd( y_77, tmp1 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k += 4;

		}
		
	zeros = _mm256_setzero_pd();

	// top triangle
	x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
	a_00_10_20_30 = _mm256_blend_pd( a_00_10_20_30, zeros, 0xe );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
	a_01_11_21_31 = _mm256_blend_pd( a_01_11_21_31, zeros, 0xc );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
	a_02_12_22_32 = _mm256_blend_pd( a_02_12_22_32, zeros, 0x8 );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	
	tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
	y_00 = _mm256_add_pd( y_00, tmp0 );
	y_11 = _mm256_add_pd( y_11, tmp1 );
	tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
	y_22 = _mm256_add_pd( y_22, tmp0 );
	y_33 = _mm256_add_pd( y_33, tmp1 );

	// top square
	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );

	tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
	y_44 = _mm256_add_pd( y_44, tmp0 );
	y_55 = _mm256_add_pd( y_55, tmp1 );
	tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
	y_66 = _mm256_add_pd( y_66, tmp0 );
	y_77 = _mm256_add_pd( y_77, tmp1 );

	A += 4 + (sda-1)*lda;
	x += 4;

	// bottom triangle
	x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
	a_00_10_20_30 = _mm256_blend_pd( a_00_10_20_30, zeros, 0xe );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
	a_01_11_21_31 = _mm256_blend_pd( a_01_11_21_31, zeros, 0xc );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
	a_02_12_22_32 = _mm256_blend_pd( a_02_12_22_32, zeros, 0x8 );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );

	tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
	y_44 = _mm256_add_pd( y_44, tmp0 );
	y_55 = _mm256_add_pd( y_55, tmp1 );
	tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
	y_66 = _mm256_add_pd( y_66, tmp0 );
	y_77 = _mm256_add_pd( y_77, tmp1 );

	// store
	__m256d
		y_0_1_2_3, y_4_5_6_7;

	y_00 = _mm256_hadd_pd(y_00, y_11);
	y_22 = _mm256_hadd_pd(y_22, y_33);
	y_44 = _mm256_hadd_pd(y_44, y_55);
	y_66 = _mm256_hadd_pd(y_66, y_77);

	y_11 = _mm256_permute2f128_pd(y_22, y_00, 2 );	
	y_00 = _mm256_permute2f128_pd(y_22, y_00, 19);	
	y_55 = _mm256_permute2f128_pd(y_66, y_44, 2 );	
	y_44 = _mm256_permute2f128_pd(y_66, y_44, 19);	

	y_00 = _mm256_add_pd( y_00, y_11 );
	y_44 = _mm256_add_pd( y_44, y_55 );

	if(alg==0)
		{
		_mm256_storeu_pd(&y[0], y_00);
		_mm256_storeu_pd(&y[4], y_44);
		}
	else if(alg==1)
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );

		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, y_44 );

		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		_mm256_storeu_pd(&y[4], y_4_5_6_7);
		}
	else // alg==-1
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );
	
		y_0_1_2_3 = _mm256_sub_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_sub_pd( y_4_5_6_7, y_44 );
	
		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		_mm256_storeu_pd(&y[4], y_4_5_6_7);
		}

	}



// it moves vertically across blocks
void kernel_dtrmv_u_t_4_lib4(int kmax, double *A, int sda, double *x, double *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
/*	__builtin_prefetch( A + 0*lda );*/
/*	__builtin_prefetch( A + 2*lda );*/

/*	double *tA, *tx;*/

	int k;
	
	__m256d
		zeros,
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0_1_2_3,
		y_00, y_11, y_22, y_33;

	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_22 = _mm256_setzero_pd();
	y_33 = _mm256_setzero_pd();

	k=0;
	for(; k<kmax-7; k+=8)
		{
		
/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, tmp0 );
		y_33 = _mm256_add_pd( y_33, tmp1 );
		
		A += 4 + (sda-1)*lda;
		x += 4;


/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, tmp0 );
		y_33 = _mm256_add_pd( y_33, tmp1 );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}
	for(; k<kmax-3; k+=4)
		{
		
/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );
		tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, tmp0 );
		y_33 = _mm256_add_pd( y_33, tmp1 );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}

	zeros = _mm256_setzero_pd();

	x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
	a_00_10_20_30 = _mm256_blend_pd( a_00_10_20_30, zeros, 0xe );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
	a_01_11_21_31 = _mm256_blend_pd( a_01_11_21_31, zeros, 0xc );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
	a_02_12_22_32 = _mm256_blend_pd( a_02_12_22_32, zeros, 0x8 );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	
	tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
	y_00 = _mm256_add_pd( y_00, tmp0 );
	y_11 = _mm256_add_pd( y_11, tmp1 );
	tmp0 = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
	tmp1 = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
	y_22 = _mm256_add_pd( y_22, tmp0 );
	y_33 = _mm256_add_pd( y_33, tmp1 );

	__m256d
		y_0_1_2_3;

	y_00 = _mm256_hadd_pd(y_00, y_11);
	y_22 = _mm256_hadd_pd(y_22, y_33);

	y_11 = _mm256_permute2f128_pd(y_22, y_00, 2 );	
	y_00 = _mm256_permute2f128_pd(y_22, y_00, 19);	

	y_00 = _mm256_add_pd( y_00, y_11 );

	if(alg==0)
		{
		_mm256_storeu_pd(&y[0], y_00);
		}
	else if(alg==1)
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );

		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_00 );

		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		}
	else // alg==-1
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	
		y_0_1_2_3 = _mm256_sub_pd( y_0_1_2_3, y_00 );
	
		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		}

	}



// it moves vertically across blocks
void kernel_dtrmv_u_t_2_lib4(int kmax, double *A, int sda, double *x, double *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	double *tA, *tx;

	int k;
	
	__m256d
		tmp0, tmp1,
		a_00_10_20_30, a_01_11_21_31,
		x_0_1_2_3,
		y_00, y_11;
	
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();

	k=0;
	for(; k<kmax-7; k+=8)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );

		A += 4 + (sda-1)*lda;
		x += 4;

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );

		A += 4 + (sda-1)*lda;
		x += 4;

		}
	for(; k<kmax-3; k+=4)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		tmp1 = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		y_11 = _mm256_add_pd( y_11, tmp1 );

		A += 4 + (sda-1)*lda;
		x += 4;

		}

	__m128d
		tm0, tm1,
		a_00_10, a_01_11,
		x_0_1,
		y_0, y_1, y_0_1;
	
	tm0 = _mm256_extractf128_pd( y_00, 0x1 );
	tm1 = _mm256_extractf128_pd( y_11, 0x1 );
	y_0 = _mm256_castpd256_pd128( y_00 );
	y_1 = _mm256_castpd256_pd128( y_11 );
	y_0 = _mm_add_pd( y_0, tm0 );
	y_1 = _mm_add_pd( y_1, tm1 );
	
	x_0_1 = _mm_loadu_pd( &x[0] );
	a_00_10 = _mm_load_sd( &A[0+lda*0] );
	a_01_11 = _mm_load_pd( &A[0+lda*1] );
	tm0 = _mm_mul_sd( a_00_10, x_0_1 );
	tm1 = _mm_mul_pd( a_01_11, x_0_1 );
	y_0 = _mm_add_sd( y_0, tm0 );
	y_1 = _mm_add_pd( y_1, tm1 );

	y_0 = _mm_hadd_pd( y_0, y_1 );


	if(alg==0)
		{
		_mm_storeu_pd(&y[0], y_0);
		}
	else if(alg==1)
		{
		y_0_1 = _mm_loadu_pd( &y[0] );

		y_0_1 = _mm_add_pd( y_0_1, y_0 );

		_mm_storeu_pd(&y[0], y_0_1);
		}
	else // alg==-1
		{
		y_0_1 = _mm_loadu_pd( &y[0] );
	
		y_0_1 = _mm_sub_pd( y_0_1, y_0 );
	
		_mm_storeu_pd(&y[0], y_0_1);
		}

	}



// it moves vertically across blocks
void kernel_dtrmv_u_t_1_lib4(int kmax, double *A, int sda, double *x, double *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	double *tA, *tx;

	int k;
	
	__m256d
		tmp0,
		a_00_10_20_30,
		x_0_1_2_3,
		y_00;
	
	
	y_00 = _mm256_setzero_pd();

	k=0;
	for(; k<kmax-3; k+=4)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		
		tmp0 = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, tmp0 );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}

	__m128d
		tm0,
		a_00_10, a_01_11,
		x_0_1,
		y_0, y_1, y_0_1;
	
	tm0 = _mm256_extractf128_pd( y_00, 0x1 );
	y_0 = _mm256_castpd256_pd128( y_00 );
	y_0 = _mm_add_pd( y_0, tm0 );

	if(k<kmax-1)
		{
		
		x_0_1 = _mm_loadu_pd( &x[0] );

		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		
		tm0 = _mm_mul_pd( a_00_10, x_0_1 );
		y_0 = _mm_add_pd( y_0, tm0 );
		
		A += 2;
		x += 2;

		}
	
	x_0_1 = _mm_load_sd( &x[0] );
	a_00_10 = _mm_load_sd( &A[0+lda*0] );
	tm0 = _mm_mul_sd( a_00_10, x_0_1 );
	y_0 = _mm_add_sd( y_0, tm0 );

	y_0 = _mm_hadd_pd( y_0, y_0 );


	if(alg==0)
		{
		_mm_store_sd(&y[0], y_0);
		}
	else if(alg==1)
		{
		y_0_1 = _mm_load_sd( &y[0] );

		y_0_1 = _mm_add_sd( y_0_1, y_0 );

		_mm_store_sd(&y[0], y_0_1);
		}
	else // alg==-1
		{
		y_0_1 = _mm_load_sd( &y[0] );
	
		y_0_1 = _mm_sub_sd( y_0_1, y_0 );
	
		_mm_store_sd(&y[0], y_0_1);
		}

	}



// it moves horizontally inside a block
void kernel_dtrmv_u_n_8_lib4(int kmax, double *A0, int sda, double *x, double *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	double *A1 = A0 + 4*sda;

	const int lda = 4;
	
	int k;

	__m128d
		tmp0,
		z_0, y_0_1, a_00_10;

	__m256d
		zeros,
		ax_temp,
		a_00_10_20_30, a_01_11_21_31,
		a_40_50_60_70, a_41_51_61_71,
		x_0, x_1,
		y_0_1_2_3, y_0_1_2_3_b, z_0_1_2_3,
		y_4_5_6_7, y_4_5_6_7_b, z_4_5_6_7;
	
/*	y_0_1_2_3   = _mm256_setzero_pd();	*/
/*	y_4_5_6_7   = _mm256_setzero_pd();	*/
/*	y_0_1_2_3_b = _mm256_setzero_pd();	*/
/*	y_4_5_6_7_b = _mm256_setzero_pd();	*/
		
	zeros = _mm256_setzero_pd();
	
/*	y_0_1_2_3   = _mm256_setzero_pd();	*/
/*	y_0_1_2_3_b = _mm256_setzero_pd();	*/
/*	y_0_1_2_3_c = _mm256_setzero_pd();	*/
/*	y_0_1_2_3_d = _mm256_setzero_pd();*/
	
	// upper triangular

	// second col (avoid zero y_0_1)
	z_0     = _mm_loaddup_pd( &x[1] );
	a_00_10 = _mm_load_pd( &A0[0+lda*1] );
	y_0_1   = _mm_mul_pd( a_00_10, z_0 );

	// first col
	z_0     = _mm_load_sd( &x[0] );
	a_00_10 = _mm_load_sd( &A0[0+lda*0] );
	tmp0    = _mm_mul_sd( a_00_10, z_0 );
	y_0_1   = _mm_add_sd( y_0_1, tmp0 );
	y_0_1_2_3_b = _mm256_castpd128_pd256( y_0_1 );
	y_0_1_2_3_b = _mm256_blend_pd( y_0_1_2_3_b, y_0_1_2_3_b, 0xc );

	// forth col (avoid zero y_0_1_2_3)
	x_1     = _mm256_broadcast_sd( &x[3] );
	a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*3] );
	y_0_1_2_3 = _mm256_mul_pd( a_01_11_21_31, x_1 );

	// first col
	x_0     = _mm256_broadcast_sd( &x[2] );
	x_0     = _mm256_blend_pd( x_0, zeros, 0x8 );
	a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*2] );
	ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );


	A0 += 4*lda;
	A1 += 4*lda;
	x  += 4;


	// upper squared
	x_0 = _mm256_broadcast_sd( &x[0] );
	x_1 = _mm256_broadcast_sd( &x[1] );

	a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
	a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*1] );

	ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
	ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );

	x_0 = _mm256_broadcast_sd( &x[2] );
	x_1 = _mm256_broadcast_sd( &x[3] );

	a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*2] );
	a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*3] );

	ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
	ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );


	// lower triangular


	// second col (avoid zero y_0_1)
	z_0     = _mm_loaddup_pd( &x[1] );
	a_00_10 = _mm_load_pd( &A1[0+lda*1] );
	y_0_1   = _mm_mul_pd( a_00_10, z_0 );

	// first col
	z_0     = _mm_load_sd( &x[0] );
	a_00_10 = _mm_load_sd( &A1[0+lda*0] );
	tmp0    = _mm_mul_sd( a_00_10, z_0 );
	y_0_1   = _mm_add_sd( y_0_1, tmp0 );
	y_4_5_6_7_b = _mm256_castpd128_pd256( y_0_1 );
	y_4_5_6_7_b = _mm256_blend_pd( y_4_5_6_7_b, y_4_5_6_7_b, 0xc );

	// forth col (avoid zero y_4_5_6_7)
	x_1     = _mm256_broadcast_sd( &x[3] );
	a_01_11_21_31 = _mm256_load_pd( &A1[0+lda*3] );
	y_4_5_6_7 = _mm256_mul_pd( a_01_11_21_31, x_1 );

	// first col
	x_0     = _mm256_broadcast_sd( &x[2] );
	x_0     = _mm256_blend_pd( x_0, zeros, 0x8 );
	a_00_10_20_30 = _mm256_load_pd( &A1[0+lda*2] );
	ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
	y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );


	A0 += 4*lda;
	A1 += 4*lda;
	x  += 4;


	k=8;
	for(; k<kmax-3; k+=4)
		{

/*		__builtin_prefetch( A0 + 4*lda );*/
/*		__builtin_prefetch( A1 + 4*lda );*/

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*1] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*1] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
		y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

/*		__builtin_prefetch( A0 + 5*lda );*/
/*		__builtin_prefetch( A1 + 5*lda );*/

		x_0 = _mm256_broadcast_sd( &x[2] );
		x_1 = _mm256_broadcast_sd( &x[3] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*2] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*2] );
		a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*3] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*3] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
		y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
	
		A0 += 4*lda;
		A1 += 4*lda;
		x  += 4;

		}
		
	if(kmax%4>=2)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*1] );
		a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*1] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
		y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
		
		A0 += 2*lda;
		A1 += 2*lda;
		x  += 2;

		}
	
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_b );
	y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, y_4_5_6_7_b );

	if(kmax%2==1)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
		a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*0] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
		
/*		A0 += 1*lda;*/
/*		A1 += 1*lda;*/
/*		x  += 1;*/

		}

	if(alg==0)
		{
		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		_mm256_storeu_pd(&y[4], y_4_5_6_7);
		}
	else if(alg==1)
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		z_4_5_6_7 = _mm256_loadu_pd( &y[4] );

		z_0_1_2_3 = _mm256_add_pd( z_0_1_2_3, y_0_1_2_3 );
		z_4_5_6_7 = _mm256_add_pd( z_4_5_6_7, y_4_5_6_7 );

		_mm256_storeu_pd(&y[0], z_0_1_2_3);
		_mm256_storeu_pd(&y[4], z_4_5_6_7);
		}
	else // alg==-1
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		z_4_5_6_7 = _mm256_loadu_pd( &y[4] );

		z_0_1_2_3 = _mm256_sub_pd( z_0_1_2_3, y_0_1_2_3 );
		z_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, y_4_5_6_7 );

		_mm256_storeu_pd(&y[0], z_0_1_2_3);
		_mm256_storeu_pd(&y[4], z_4_5_6_7);
		}

	}



// it moves horizontally inside a block (A upper triangular)
void kernel_dtrmv_u_n_4_lib4(int kmax, double *A, double *x, double *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	int k;
	
	__m128d
		tmp0,
		z_0, y_0_1, a_00_10;

	__m256d
		zeros,
		ax_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0, x_1, x_2, x_3,
		y_0_1_2_3, y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3;
		
	zeros = _mm256_setzero_pd();
	
/*	y_0_1_2_3   = _mm256_setzero_pd();	*/
/*	y_0_1_2_3_b = _mm256_setzero_pd();	*/
/*	y_0_1_2_3_c = _mm256_setzero_pd();	*/
	y_0_1_2_3_d = _mm256_setzero_pd();
	
	// second col (avoid zero y_0_1)
	z_0     = _mm_loaddup_pd( &x[1] );
	a_00_10 = _mm_load_pd( &A[0+lda*1] );
	y_0_1   = _mm_mul_pd( a_00_10, z_0 );

	// first col
	z_0     = _mm_load_sd( &x[0] );
	a_00_10 = _mm_load_sd( &A[0+lda*0] );
	tmp0    = _mm_mul_sd( a_00_10, z_0 );
	y_0_1   = _mm_add_sd( y_0_1, tmp0 );
	y_0_1_2_3_c = _mm256_castpd128_pd256( y_0_1 );
	y_0_1_2_3_c = _mm256_blend_pd( y_0_1_2_3_c, y_0_1_2_3_d, 0xc );

	// forth col (avoid zero y_0_1_2_3)
	x_3     = _mm256_broadcast_sd( &x[3] );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	y_0_1_2_3 = _mm256_mul_pd( a_03_13_23_33, x_3 );

	// first col
	x_2     = _mm256_broadcast_sd( &x[2] );
	x_2     = _mm256_blend_pd( x_2, zeros, 0x8 );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
	y_0_1_2_3_b = _mm256_mul_pd( a_02_12_22_32, x_2 );

	A += 4*lda;
	x += 4;

	k=4;
	for(; k<kmax-3; k+=4)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );

		x_2 = _mm256_broadcast_sd( &x[2] );
		x_3 = _mm256_broadcast_sd( &x[3] );

		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );

		ax_temp = _mm256_mul_pd( a_02_12_22_32, x_2 );
		y_0_1_2_3_c = _mm256_add_pd( y_0_1_2_3_c, ax_temp );
		ax_temp = _mm256_mul_pd( a_03_13_23_33, x_3 );
		y_0_1_2_3_d = _mm256_add_pd( y_0_1_2_3_d, ax_temp );
		
		A += 4*lda;
		x += 4;

		}
	
	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_c );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_d );

	if(kmax%4>=2)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );
		x_1 = _mm256_broadcast_sd( &x[1] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
		y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );

		A += 2*lda;
		x += 2;

		}

	y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_0_1_2_3_b );

	if(kmax%2==1)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );

		ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
		
		}

	if(alg==0)
		{
		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		}
	else if(alg==1)
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );

		z_0_1_2_3 = _mm256_add_pd ( z_0_1_2_3, y_0_1_2_3 );

		_mm256_storeu_pd(&y[0], z_0_1_2_3);
		}
	else // alg==-1
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );

		z_0_1_2_3 = _mm256_sub_pd ( z_0_1_2_3, y_0_1_2_3 );

		_mm256_storeu_pd(&y[0], z_0_1_2_3);
		}

	}



// it moves horizontally inside a block
void kernel_dtrmv_u_n_2_lib4(int kmax, double *A, double *x, double *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	int k;

	__m128d
		ax_temp,
		a_00_10, a_01_11, a_02_12, a_03_13,
		x_0, x_1, x_2, x_3,
		y_0_1, y_0_1_b, y_0_1_c, y_0_1_d, z_0_1;
	
/*	y_0_1 = _mm_setzero_pd();	*/

	// second col (avoid zero y_0_1)
	x_0     = _mm_loaddup_pd( &x[1] );
	a_00_10 = _mm_load_pd( &A[0+lda*1] );
	y_0_1   = _mm_mul_pd( a_00_10, x_0 );

	// first col
	x_0     = _mm_load_sd( &x[0] );
	a_00_10 = _mm_load_sd( &A[0+lda*0] );
	ax_temp = _mm_mul_sd( a_00_10, x_0 );
	y_0_1   = _mm_add_sd( y_0_1, ax_temp );

	A += 2*lda;
	x += 2;

	k=2;
	for(; k<kmax-1; k+=2)
		{

		x_0 = _mm_loaddup_pd( &x[0] );
		x_1 = _mm_loaddup_pd( &x[1] );

		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		a_01_11 = _mm_load_pd( &A[0+lda*1] );

		ax_temp = _mm_mul_pd( a_00_10, x_0 );
		y_0_1 = _mm_add_pd( y_0_1, ax_temp );
		ax_temp = _mm_mul_pd( a_01_11, x_1 );
		y_0_1 = _mm_add_pd( y_0_1, ax_temp );

		A += 2*lda;
		x += 2;

		}
	if(kmax%2==1)
		{

		x_0 = _mm_loaddup_pd( &x[0] );

		a_00_10 = _mm_load_pd( &A[0+lda*0] );

		ax_temp = _mm_mul_pd( a_00_10, x_0 );
		y_0_1 = _mm_add_pd( y_0_1, ax_temp );

		}

	if(alg==0)
		{
		_mm_storeu_pd(&y[0], y_0_1);
		}
	else if(alg==1)
		{
		z_0_1 = _mm_loadu_pd( &y[0] );

		z_0_1 = _mm_add_pd( z_0_1, y_0_1 );

		_mm_storeu_pd(&y[0], z_0_1);
		}
	else // alg==-1
		{
		z_0_1 = _mm_loadu_pd( &y[0] );

		z_0_1 = _mm_sub_pd( z_0_1, y_0_1 );

		_mm_storeu_pd(&y[0], z_0_1);
		}

	}

#endif
