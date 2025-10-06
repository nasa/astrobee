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

// it moves vertically across panels
void kernel_dtrmv_u_t_12_lib4(int kmax, double *A, int sda, double *x, double *y, int alg)
	{

//	if(kmax<=0) 
//		return;
	
	const int lda = 4;
	
//	__builtin_prefetch( A + 0*lda );
//	__builtin_prefetch( A + 2*lda );
//	__builtin_prefetch( A + 4*lda );
//	__builtin_prefetch( A + 6*lda );

	double *tA, *tx;

	int k;
	int ka = kmax; // number from aligned positon
	
	__m256d
		zeros,
		a_00,
		x_00,
		y_00, y_11, y_22, y_33, y_44, y_55, y_66, y_77, y_88, y_99, y_aa, y_bb;
	

	zeros = _mm256_setzero_pd();
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_22 = _mm256_setzero_pd();
	y_33 = _mm256_setzero_pd();
	y_44 = _mm256_setzero_pd();
	y_55 = _mm256_setzero_pd();
	y_66 = _mm256_setzero_pd();
	y_77 = _mm256_setzero_pd();
	y_88 = _mm256_setzero_pd();
	y_99 = _mm256_setzero_pd();
	y_aa = _mm256_setzero_pd();
	y_bb = _mm256_setzero_pd();

	k=0;
	for(; k<ka-11; k+=12)
		{

//		__builtin_prefetch( A + sda*lda + 0*lda );
//		__builtin_prefetch( A + sda*lda + 2*lda );

		x_00 = _mm256_loadu_pd( &x[0] );

		a_00 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00, x_00, y_00 );
		a_00 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_00, x_00, y_11 );
		a_00 = _mm256_load_pd( &A[0+lda*2] );
		y_22 = _mm256_fmadd_pd( a_00, x_00, y_22 );
		a_00 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_00, x_00, y_33 );
	
//		__builtin_prefetch( A + sda*lda + 4*lda );
//		__builtin_prefetch( A + sda*lda + 6*lda );
	
		a_00 = _mm256_load_pd( &A[0+lda*4] );
		y_44 = _mm256_fmadd_pd( a_00, x_00, y_44 );
		a_00 = _mm256_load_pd( &A[0+lda*5] );
		y_55 = _mm256_fmadd_pd( a_00, x_00, y_55 );
		a_00 = _mm256_load_pd( &A[0+lda*6] );
		y_66 = _mm256_fmadd_pd( a_00, x_00, y_66 );
		a_00 = _mm256_load_pd( &A[0+lda*7] );
		y_77 = _mm256_fmadd_pd( a_00, x_00, y_77 );

//		__builtin_prefetch( A + sda*lda + 8*lda );
//		__builtin_prefetch( A + sda*lda + 10*lda );
	
		a_00 = _mm256_load_pd( &A[0+lda*8] );
		y_88 = _mm256_fmadd_pd( a_00, x_00, y_88 );
		a_00 = _mm256_load_pd( &A[0+lda*9] );
		y_99 = _mm256_fmadd_pd( a_00, x_00, y_99 );
		a_00 = _mm256_load_pd( &A[0+lda*10] );
		y_aa = _mm256_fmadd_pd( a_00, x_00, y_aa );
		a_00 = _mm256_load_pd( &A[0+lda*11] );
		y_bb = _mm256_fmadd_pd( a_00, x_00, y_bb );

		A += 4 + (sda-1)*lda;
		x += 4;

//		__builtin_prefetch( A + sda*lda + 0*lda );
//		__builtin_prefetch( A + sda*lda + 2*lda );

		x_00 = _mm256_loadu_pd( &x[0] );

		a_00 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00, x_00, y_00 );
		a_00 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_00, x_00, y_11 );
		a_00 = _mm256_load_pd( &A[0+lda*2] );
		y_22 = _mm256_fmadd_pd( a_00, x_00, y_22 );
		a_00 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_00, x_00, y_33 );
	
//		__builtin_prefetch( A + sda*lda + 4*lda );
//		__builtin_prefetch( A + sda*lda + 6*lda );

		a_00 = _mm256_load_pd( &A[0+lda*4] );
		y_44 = _mm256_fmadd_pd( a_00, x_00, y_44 );
		a_00 = _mm256_load_pd( &A[0+lda*5] );
		y_55 = _mm256_fmadd_pd( a_00, x_00, y_55 );
		a_00 = _mm256_load_pd( &A[0+lda*6] );
		y_66 = _mm256_fmadd_pd( a_00, x_00, y_66 );
		a_00 = _mm256_load_pd( &A[0+lda*7] );
		y_77 = _mm256_fmadd_pd( a_00, x_00, y_77 );

//		__builtin_prefetch( A + sda*lda + 8*lda );
//		__builtin_prefetch( A + sda*lda + 10*lda );

		a_00 = _mm256_load_pd( &A[0+lda*8] );
		y_88 = _mm256_fmadd_pd( a_00, x_00, y_88 );
		a_00 = _mm256_load_pd( &A[0+lda*9] );
		y_99 = _mm256_fmadd_pd( a_00, x_00, y_99 );
		a_00 = _mm256_load_pd( &A[0+lda*10] );
		y_aa = _mm256_fmadd_pd( a_00, x_00, y_aa );
		a_00 = _mm256_load_pd( &A[0+lda*11] );
		y_bb = _mm256_fmadd_pd( a_00, x_00, y_bb );

		A += 4 + (sda-1)*lda;
		x += 4;

//		__builtin_prefetch( A + sda*lda + 0*lda );
//		__builtin_prefetch( A + sda*lda + 2*lda );

		x_00 = _mm256_loadu_pd( &x[0] );

		a_00 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00, x_00, y_00 );
		a_00 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_00, x_00, y_11 );
		a_00 = _mm256_load_pd( &A[0+lda*2] );
		y_22 = _mm256_fmadd_pd( a_00, x_00, y_22 );
		a_00 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_00, x_00, y_33 );
	
//		__builtin_prefetch( A + sda*lda + 4*lda );
//		__builtin_prefetch( A + sda*lda + 6*lda );

		a_00 = _mm256_load_pd( &A[0+lda*4] );
		y_44 = _mm256_fmadd_pd( a_00, x_00, y_44 );
		a_00 = _mm256_load_pd( &A[0+lda*5] );
		y_55 = _mm256_fmadd_pd( a_00, x_00, y_55 );
		a_00 = _mm256_load_pd( &A[0+lda*6] );
		y_66 = _mm256_fmadd_pd( a_00, x_00, y_66 );
		a_00 = _mm256_load_pd( &A[0+lda*7] );
		y_77 = _mm256_fmadd_pd( a_00, x_00, y_77 );

//		__builtin_prefetch( A + sda*lda + 8*lda );
//		__builtin_prefetch( A + sda*lda + 10*lda );

		a_00 = _mm256_load_pd( &A[0+lda*8] );
		y_88 = _mm256_fmadd_pd( a_00, x_00, y_88 );
		a_00 = _mm256_load_pd( &A[0+lda*9] );
		y_99 = _mm256_fmadd_pd( a_00, x_00, y_99 );
		a_00 = _mm256_load_pd( &A[0+lda*10] );
		y_aa = _mm256_fmadd_pd( a_00, x_00, y_aa );
		a_00 = _mm256_load_pd( &A[0+lda*11] );
		y_bb = _mm256_fmadd_pd( a_00, x_00, y_bb );

		A += 4 + (sda-1)*lda;
		x += 4;

		}

	x_00 = _mm256_loadu_pd( &x[0] );

	a_00 = _mm256_load_pd( &A[0+lda*0] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x1 );
	y_00 = _mm256_fmadd_pd( a_00, x_00, y_00 );
	a_00 = _mm256_load_pd( &A[0+lda*1] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x3 );
	y_11 = _mm256_fmadd_pd( a_00, x_00, y_11 );
	a_00 = _mm256_load_pd( &A[0+lda*2] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x7 );
	y_22 = _mm256_fmadd_pd( a_00, x_00, y_22 );
	a_00 = _mm256_load_pd( &A[0+lda*3] );
	y_33 = _mm256_fmadd_pd( a_00, x_00, y_33 );

	a_00 = _mm256_load_pd( &A[0+lda*4] );
	y_44 = _mm256_fmadd_pd( a_00, x_00, y_44 );
	a_00 = _mm256_load_pd( &A[0+lda*5] );
	y_55 = _mm256_fmadd_pd( a_00, x_00, y_55 );
	a_00 = _mm256_load_pd( &A[0+lda*6] );
	y_66 = _mm256_fmadd_pd( a_00, x_00, y_66 );
	a_00 = _mm256_load_pd( &A[0+lda*7] );
	y_77 = _mm256_fmadd_pd( a_00, x_00, y_77 );

	a_00 = _mm256_load_pd( &A[0+lda*8] );
	y_88 = _mm256_fmadd_pd( a_00, x_00, y_88 );
	a_00 = _mm256_load_pd( &A[0+lda*9] );
	y_99 = _mm256_fmadd_pd( a_00, x_00, y_99 );
	a_00 = _mm256_load_pd( &A[0+lda*10] );
	y_aa = _mm256_fmadd_pd( a_00, x_00, y_aa );
	a_00 = _mm256_load_pd( &A[0+lda*11] );
	y_bb = _mm256_fmadd_pd( a_00, x_00, y_bb );

	A += 4 + (sda-1)*lda;
	x += 4;

	x_00 = _mm256_loadu_pd( &x[0] );

	a_00 = _mm256_load_pd( &A[0+lda*4] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x1 );
	y_44 = _mm256_fmadd_pd( a_00, x_00, y_44 );
	a_00 = _mm256_load_pd( &A[0+lda*5] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x3 );
	y_55 = _mm256_fmadd_pd( a_00, x_00, y_55 );
	a_00 = _mm256_load_pd( &A[0+lda*6] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x7 );
	y_66 = _mm256_fmadd_pd( a_00, x_00, y_66 );
	a_00 = _mm256_load_pd( &A[0+lda*7] );
	y_77 = _mm256_fmadd_pd( a_00, x_00, y_77 );

	a_00 = _mm256_load_pd( &A[0+lda*8] );
	y_88 = _mm256_fmadd_pd( a_00, x_00, y_88 );
	a_00 = _mm256_load_pd( &A[0+lda*9] );
	y_99 = _mm256_fmadd_pd( a_00, x_00, y_99 );
	a_00 = _mm256_load_pd( &A[0+lda*10] );
	y_aa = _mm256_fmadd_pd( a_00, x_00, y_aa );
	a_00 = _mm256_load_pd( &A[0+lda*11] );
	y_bb = _mm256_fmadd_pd( a_00, x_00, y_bb );

	A += 4 + (sda-1)*lda;
	x += 4;

	x_00 = _mm256_loadu_pd( &x[0] );

	a_00 = _mm256_load_pd( &A[0+lda*8] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x1 );
	y_88 = _mm256_fmadd_pd( a_00, x_00, y_88 );
	a_00 = _mm256_load_pd( &A[0+lda*9] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x3 );
	y_99 = _mm256_fmadd_pd( a_00, x_00, y_99 );
	a_00 = _mm256_load_pd( &A[0+lda*10] );
	a_00 = _mm256_blend_pd( zeros, a_00, 0x7 );
	y_aa = _mm256_fmadd_pd( a_00, x_00, y_aa );
	a_00 = _mm256_load_pd( &A[0+lda*11] );
	y_bb = _mm256_fmadd_pd( a_00, x_00, y_bb );

	A += 4 + (sda-1)*lda;
	x += 4;


	__m256d
		y_0_1_2_3, y_4_5_6_7, y_8_9_a_b;

	y_00 = _mm256_hadd_pd(y_00, y_11);
	y_22 = _mm256_hadd_pd(y_22, y_33);
	y_44 = _mm256_hadd_pd(y_44, y_55);
	y_66 = _mm256_hadd_pd(y_66, y_77);
	y_88 = _mm256_hadd_pd(y_88, y_99);
	y_aa = _mm256_hadd_pd(y_aa, y_bb);

	y_11 = _mm256_permute2f128_pd(y_22, y_00, 2 );	
	y_00 = _mm256_permute2f128_pd(y_22, y_00, 19);	
	y_55 = _mm256_permute2f128_pd(y_66, y_44, 2 );	
	y_44 = _mm256_permute2f128_pd(y_66, y_44, 19);	
	y_99 = _mm256_permute2f128_pd(y_aa, y_88, 2 );	
	y_88 = _mm256_permute2f128_pd(y_aa, y_88, 19);	

	y_00 = _mm256_add_pd( y_00, y_11 );
	y_44 = _mm256_add_pd( y_44, y_55 );
	y_88 = _mm256_add_pd( y_88, y_99 );

	if(alg==0)
		{
		_mm256_storeu_pd(&y[0], y_00);
		_mm256_storeu_pd(&y[4], y_44);
		_mm256_storeu_pd(&y[8], y_88);
		}
	else if(alg==1)
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );
		y_8_9_a_b = _mm256_loadu_pd( &y[8] );

		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, y_44 );
		y_8_9_a_b = _mm256_add_pd( y_8_9_a_b, y_88 );

		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		_mm256_storeu_pd(&y[4], y_4_5_6_7);
		_mm256_storeu_pd(&y[8], y_8_9_a_b);
		}
	else // alg==-1
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );
		y_8_9_a_b = _mm256_loadu_pd( &y[8] );
	
		y_0_1_2_3 = _mm256_sub_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_sub_pd( y_4_5_6_7, y_44 );
		y_8_9_a_b = _mm256_sub_pd( y_8_9_a_b, y_88 );
	
		_mm256_storeu_pd(&y[0], y_0_1_2_3);
		_mm256_storeu_pd(&y[4], y_4_5_6_7);
		_mm256_storeu_pd(&y[8], y_8_9_a_b);
		}

	}



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
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_33 );
	
/*		__builtin_prefetch( A + sda*lda + 4*lda );*/
/*		__builtin_prefetch( A + sda*lda + 6*lda );*/

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
		y_44 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_44 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
		y_55 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_55 );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
		y_66 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_66 );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		y_77 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_77 );

		A += 4 + (sda-1)*lda;
		x += 4;


/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_33 );
	
/*		__builtin_prefetch( A + sda*lda + 4*lda );*/
/*		__builtin_prefetch( A + sda*lda + 6*lda );*/

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
		y_44 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_44 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
		y_55 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_55 );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
		y_66 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_66 );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		y_77 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_77 );

		A += 4 + (sda-1)*lda;
		x += 4;

		}
	if(k<kmax-3)
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
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_33 );
	
/*		__builtin_prefetch( A + sda*lda + 4*lda );*/
/*		__builtin_prefetch( A + sda*lda + 6*lda );*/

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
		y_44 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_44 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
		y_55 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_55 );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
		y_66 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_66 );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		y_77 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_77 );

		A += 4 + (sda-1)*lda;
		x += 4;

		k += 4;

		}
		
	zeros = _mm256_setzero_pd();

	// top triangle
	x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
	a_00_10_20_30 = _mm256_blend_pd( a_00_10_20_30, zeros, 0xe );
	y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
	a_01_11_21_31 = _mm256_blend_pd( a_01_11_21_31, zeros, 0xc );
	y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
	a_02_12_22_32 = _mm256_blend_pd( a_02_12_22_32, zeros, 0x8 );
	y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	y_33 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_33 );

	// top square
	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
	y_44 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_44 );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
	y_55 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_55 );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
	y_66 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_66 );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
	y_77 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_77 );

	A += 4 + (sda-1)*lda;
	x += 4;

	// bottom triangle
	x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
	a_00_10_20_30 = _mm256_blend_pd( a_00_10_20_30, zeros, 0xe );
	y_44 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_44 );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
	a_01_11_21_31 = _mm256_blend_pd( a_01_11_21_31, zeros, 0xc );
	y_55 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_55 );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
	a_02_12_22_32 = _mm256_blend_pd( a_02_12_22_32, zeros, 0x8 );
	y_66 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_66 );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
	y_77 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_77 );

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
		y_00, y_11, y_22, y_33,
		y_0b, y_1b, y_2b, y_3b;

	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_22 = _mm256_setzero_pd();
	y_33 = _mm256_setzero_pd();
	y_0b = _mm256_setzero_pd();
	y_1b = _mm256_setzero_pd();
	y_2b = _mm256_setzero_pd();
	y_3b = _mm256_setzero_pd();

	k=0;
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
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_33 );
		
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
	
	y_00 = _mm256_add_pd( y_00, y_0b );
	y_11 = _mm256_add_pd( y_11, y_1b );
	y_22 = _mm256_add_pd( y_22, y_2b );
	y_33 = _mm256_add_pd( y_33, y_3b );

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
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		y_33 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_33 );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}

	zeros = _mm256_setzero_pd();

	x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

	a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
	a_00_10_20_30 = _mm256_blend_pd( a_00_10_20_30, zeros, 0xe );
	y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
	a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
	a_01_11_21_31 = _mm256_blend_pd( a_01_11_21_31, zeros, 0xc );
	y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );
	a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
	a_02_12_22_32 = _mm256_blend_pd( a_02_12_22_32, zeros, 0x8 );
	y_22 = _mm256_fmadd_pd( a_02_12_22_32, x_0_1_2_3, y_22 );
	a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
	y_33 = _mm256_fmadd_pd( a_03_13_23_33, x_0_1_2_3, y_33 );

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
		y_00, y_11,
		y_0b, y_1b;
	
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_0b = _mm256_setzero_pd();
	y_1b = _mm256_setzero_pd();

	k=0;
	for(; k<kmax-7; k+=8)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );

		A += 4 + (sda-1)*lda;
		x += 4;

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

	for(; k<kmax-3; k+=4)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		y_11 = _mm256_fmadd_pd( a_01_11_21_31, x_0_1_2_3, y_11 );

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
		y_00,
		y_0b;
	
	
	y_00 = _mm256_setzero_pd();
	y_0b = _mm256_setzero_pd();

	k=0;
	for(; k<kmax-7; k+=8)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		
		A += 4 + (sda-1)*lda;
		x += 4;
	
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_0b = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_0b );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}
	
	y_00 = _mm256_add_pd( y_00, y_0b );

	for(; k<kmax-3; k+=4)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		y_00 = _mm256_fmadd_pd( a_00_10_20_30, x_0_1_2_3, y_00 );
		
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
		
		y_0 = _mm_fmadd_pd( a_00_10, x_0_1, y_0 );
		
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
void kernel_dtrmv_u_n_12_lib4(int kmax, double *A0, int sda, double *x, double *y, int alg)
	{
	if(kmax<=0) 
		return;
	
	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;

	const int lda = 4;
	
	int k;

	__m256d
		zeros,
		a_0,
		x_0, x_1,
		y_0, y_0_b, y_0_c, y_0_d, z_0,
		y_4, y_4_b, y_4_c, y_4_d, z_4,
		y_8, y_8_b, y_8_c, y_8_d, z_8;
	
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

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*1] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*2] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*3] );
	y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*4] );
	y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
	a_0 = _mm256_load_pd( &A1[0+lda*4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*5] );
	y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
	a_0 = _mm256_load_pd( &A1[0+lda*5] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*6] );
	y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
	a_0 = _mm256_load_pd( &A1[0+lda*6] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*7] );
	y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
	a_0 = _mm256_load_pd( &A1[0+lda*7] );
	y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[8] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*8] );
	y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
	a_0 = _mm256_load_pd( &A1[0+lda*8] );
	y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );
	a_0 = _mm256_load_pd( &A2[0+lda*8] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	y_8   = _mm256_fmadd_pd( a_0, x_0, y_8 );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*9] );
	y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
	a_0 = _mm256_load_pd( &A1[0+lda*9] );
	y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );
	a_0 = _mm256_load_pd( &A2[0+lda*9] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	y_8_b = _mm256_fmadd_pd( a_0, x_1, y_8_b );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[10] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*10] );
	y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
	a_0 = _mm256_load_pd( &A1[0+lda*10] );
	y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );
	a_0 = _mm256_load_pd( &A2[0+lda*10] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
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

	k=12;
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
	
		A0 += 8*lda;
		A1 += 8*lda;
		A2 += 8*lda;
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
	
		A0 += 4*lda;
		A1 += 4*lda;
		A2 += 4*lda;
		x  += 4;

		}

	y_0   = _mm256_add_pd( y_0, y_0_c );
	y_4   = _mm256_add_pd( y_4, y_4_c );
	y_8   = _mm256_add_pd( y_8, y_8_c );
	y_0_b = _mm256_add_pd( y_0_b, y_0_d );
	y_4_b = _mm256_add_pd( y_4_b, y_4_d );
	y_8_b = _mm256_add_pd( y_8_b, y_8_d );

	if(kmax%4>=2)
		{

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

	
		A0 += 2*lda;
		A1 += 2*lda;
		A2 += 2*lda;
		x  += 2;

		}
	
	y_0 = _mm256_add_pd( y_0, y_0_b );
	y_4 = _mm256_add_pd( y_4, y_4_b );
	y_8 = _mm256_add_pd( y_8, y_8_b );

	if(kmax%2==1)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );

		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );
		a_0 = _mm256_load_pd( &A2[0+lda*0] );
		y_8   = _mm256_fmadd_pd( a_0, x_0, y_8 );
		
/*		A0 += 1*lda;*/
/*		A1 += 1*lda;*/
/*		x  += 1;*/

		}

	if(alg==0)
		{
		_mm256_storeu_pd(&y[0], y_0);
		_mm256_storeu_pd(&y[4], y_4);
		_mm256_storeu_pd(&y[8], y_8);
		}
	else if(alg==1)
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );
		z_8 = _mm256_loadu_pd( &y[8] );

		z_0 = _mm256_add_pd( z_0, y_0 );
		z_4 = _mm256_add_pd( z_4, y_4 );
		z_8 = _mm256_add_pd( z_8, y_8 );

		_mm256_storeu_pd(&y[0], z_0);
		_mm256_storeu_pd(&y[4], z_4);
		_mm256_storeu_pd(&y[8], z_8);
		}
	else // alg==-1
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );
		z_8 = _mm256_loadu_pd( &y[8] );

		z_0 = _mm256_sub_pd( z_0, y_0 );
		z_4 = _mm256_sub_pd( z_4, y_4 );
		z_8 = _mm256_sub_pd( z_8, y_8 );

		_mm256_storeu_pd(&y[0], z_0);
		_mm256_storeu_pd(&y[4], z_4);
		_mm256_storeu_pd(&y[8], z_8);
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

	__m256d
		zeros,
		a_0,
		x_0, x_1,
		y_0, y_0_b, y_0_c, y_0_d, z_0,
		y_4, y_4_b, y_4_c, y_4_d, z_4;
	
	zeros = _mm256_setzero_pd();
	
	y_0   = _mm256_setzero_pd();	
	y_4   = _mm256_setzero_pd();	
	y_0_b = _mm256_setzero_pd();	
	y_4_b = _mm256_setzero_pd();	
	y_0_c = _mm256_setzero_pd();	
	y_4_c = _mm256_setzero_pd();	
	y_0_d = _mm256_setzero_pd();	
	y_4_d = _mm256_setzero_pd();	
	
	// upper triangular

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*1] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*2] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*3] );
	y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*4] );
	y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
	a_0 = _mm256_load_pd( &A1[0+lda*4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*5] );
	y_0_b = _mm256_fmadd_pd( a_0, x_1, y_0_b );
	a_0 = _mm256_load_pd( &A1[0+lda*5] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	y_4_b = _mm256_fmadd_pd( a_0, x_1, y_4_b );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A0[0+lda*6] );
	y_0_c = _mm256_fmadd_pd( a_0, x_0, y_0_c );
	a_0 = _mm256_load_pd( &A1[0+lda*6] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	y_4_c = _mm256_fmadd_pd( a_0, x_0, y_4_c );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A0[0+lda*7] );
	y_0_d = _mm256_fmadd_pd( a_0, x_1, y_0_d );
	a_0 = _mm256_load_pd( &A1[0+lda*7] );
	y_4_d = _mm256_fmadd_pd( a_0, x_1, y_4_d );

	A0 += 8*lda;
	A1 += 8*lda;
	x  += 8;


	k=8;
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

	if(kmax%4>=2)
		{

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

	
		A0 += 2*lda;
		A1 += 2*lda;
		x  += 2;

		}
	
	y_0 = _mm256_add_pd( y_0, y_0_b );
	y_4 = _mm256_add_pd( y_4, y_4_b );

	if(kmax%2==1)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );

		a_0 = _mm256_load_pd( &A0[0+lda*0] );
		y_0   = _mm256_fmadd_pd( a_0, x_0, y_0 );
		a_0 = _mm256_load_pd( &A1[0+lda*0] );
		y_4   = _mm256_fmadd_pd( a_0, x_0, y_4 );
		
/*		A0 += 1*lda;*/
/*		A1 += 1*lda;*/
/*		x  += 1;*/

		}

	if(alg==0)
		{
		_mm256_storeu_pd(&y[0], y_0);
		_mm256_storeu_pd(&y[4], y_4);
		}
	else if(alg==1)
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );

		z_0 = _mm256_add_pd( z_0, y_0 );
		z_4 = _mm256_add_pd( z_4, y_4 );

		_mm256_storeu_pd(&y[0], z_0);
		_mm256_storeu_pd(&y[4], z_4);
		}
	else // alg==-1
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );

		z_0 = _mm256_sub_pd( z_0, y_0 );
		z_4 = _mm256_sub_pd( z_4, y_4 );

		_mm256_storeu_pd(&y[0], z_0);
		_mm256_storeu_pd(&y[4], z_4);
		}

	}



// it moves horizontally inside a block (A upper triangular)
void kernel_dtrmv_u_n_4_lib4(int kmax, double *A, double *x, double *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	int k;
	
	__m256d
		zeros,
		a_0,
		x_0, x_1,
		y_0_1_2_3  , y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3;
	
	zeros       = _mm256_setzero_pd();
	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A[0+lda*0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A[0+lda*1] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );

	x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

	x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
	a_0 = _mm256_load_pd( &A[0+lda*2] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	y_0_1_2_3_c = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3_c );

	x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
	a_0 = _mm256_load_pd( &A[0+lda*3] );
	y_0_1_2_3_d = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_d );
	
	A += 4*lda;
	x += 4;

	k=4;
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
	
	y_0_1_2_3   = _mm256_add_pd( y_0_1_2_3  , y_0_1_2_3_c );
	y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, y_0_1_2_3_d );

	if(kmax%4>=2)
		{

		x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

		x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3   = _mm256_fmadd_pd( a_0, x_0, y_0_1_2_3 );

		x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
		a_0 = _mm256_load_pd( &A[0+lda*1] );
		y_0_1_2_3_b = _mm256_fmadd_pd( a_0, x_1, y_0_1_2_3_b );


		A += 2*lda;
		x += 2;

		}

	y_0_1_2_3   = _mm256_add_pd( y_0_1_2_3  , y_0_1_2_3_b );

	if(kmax%2==1)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );
		a_0 = _mm256_load_pd( &A[0+lda*0] );
		y_0_1_2_3 = _mm256_fmadd_pd( a_0, x_0,  y_0_1_2_3 );
		
/*		A += 1*lda;*/
/*		x += 1;*/

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
	y_0_1_b = _mm_setzero_pd();	
	y_0_1_c = _mm_setzero_pd();	
	y_0_1_d = _mm_setzero_pd();	

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

	if(k<kmax-1)
		{
		x_0 = _mm_loaddup_pd( &x[0] );
		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		y_0_1   = _mm_fmadd_pd( a_00_10, x_0, y_0_1 );

		x_1 = _mm_loaddup_pd( &x[1] );
		a_01_11 = _mm_load_pd( &A[0+lda*1] );
		y_0_1_b = _mm_fmadd_pd( a_01_11, x_1, y_0_1_b );

		A += 2*lda;
		x += 2;
		k += 2;
		}

	for(; k<kmax-3; k+=4)
		{

		x_0 = _mm_loaddup_pd( &x[0] );
		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		y_0_1   = _mm_fmadd_pd( a_00_10, x_0, y_0_1 );

		x_1 = _mm_loaddup_pd( &x[1] );
		a_01_11 = _mm_load_pd( &A[0+lda*1] );
		y_0_1_b = _mm_fmadd_pd( a_01_11, x_1, y_0_1_b );

		x_2 = _mm_loaddup_pd( &x[2] );
		a_02_12 = _mm_load_pd( &A[0+lda*2] );
		y_0_1_c = _mm_fmadd_pd( a_02_12, x_2, y_0_1_c );

		x_3 = _mm_loaddup_pd( &x[3] );
		a_03_13 = _mm_load_pd( &A[0+lda*3] );
		y_0_1_d = _mm_fmadd_pd( a_03_13, x_3, y_0_1_d );
		
		A += 4*lda;
		x += 4;

		}

	y_0_1   = _mm_add_pd( y_0_1, y_0_1_c );
	y_0_1_b = _mm_add_pd( y_0_1_b, y_0_1_d );

	for(; k<kmax-1; k+=2)
		{

		x_0 = _mm_loaddup_pd( &x[0] );
		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		y_0_1   = _mm_fmadd_pd( a_00_10, x_0, y_0_1 );

		x_1 = _mm_loaddup_pd( &x[1] );
		a_01_11 = _mm_load_pd( &A[0+lda*1] );
		y_0_1_b = _mm_fmadd_pd( a_01_11, x_1, y_0_1_b );

		A += 2*lda;
		x += 2;

		}

	y_0_1 = _mm_add_pd( y_0_1, y_0_1_b );

	if(kmax%2==1)
		{

		x_0 = _mm_loaddup_pd( &x[0] );
		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		y_0_1 = _mm_fmadd_pd( a_00_10, x_0,  y_0_1 );
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
