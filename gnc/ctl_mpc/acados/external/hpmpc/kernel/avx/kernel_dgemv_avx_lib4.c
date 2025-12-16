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


// TODO use cast & extract to process data in order !!!

#define ENABLE_PREFETCH 1



#if ! defined(BLASFEO)

void kernel_dgemv_t_12_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	double *tA, *tx;

	int k;
	int ka = kmax; // number from aligned positon
	
	__m256d
		aaxx_temp, 
		a_00,
		x_00,
		y_00, y_11, y_22, y_33, 
		y_44, y_55, y_66, y_77,
		y_88, y_99, y_aa, y_bb;
	
	__m128d
		ax_temp,
		a_0,
		x_0,
		y_0, y_1, y_2, y_3, 
		y_4, y_5, y_6, y_7,
		y_8, y_9, y_a, y_b;
	
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
	
	y_0 = _mm256_castpd256_pd128(y_00);
	y_1 = _mm256_castpd256_pd128(y_11);
	y_2 = _mm256_castpd256_pd128(y_22);
	y_3 = _mm256_castpd256_pd128(y_33);
	y_4 = _mm256_castpd256_pd128(y_44);
	y_5 = _mm256_castpd256_pd128(y_55);
	y_6 = _mm256_castpd256_pd128(y_66);
	y_7 = _mm256_castpd256_pd128(y_77);
	y_8 = _mm256_castpd256_pd128(y_88);
	y_9 = _mm256_castpd256_pd128(y_99);
	y_a = _mm256_castpd256_pd128(y_aa);
	y_b = _mm256_castpd256_pd128(y_bb);

	k = ka/lda*lda;
	tA = A + k*sda;
	tx = x + k;

	if(ka-k>0) // it can be only ka-k = {1, 2, 3}
		{
		if((ka-k)>=2)
			{
		
			x_0 = _mm_load_pd( &tx[0] );

			a_0 = _mm_load_pd( &tA[0+lda*0] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_0 = _mm_add_pd (y_0, ax_temp );

			a_0 = _mm_load_pd( &tA[0+lda*1] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_1 = _mm_add_pd (y_1, ax_temp );

			a_0 = _mm_load_pd( &tA[0+lda*2] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_2 = _mm_add_pd (y_2, ax_temp );

			a_0 = _mm_load_pd( &tA[0+lda*3] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_3 = _mm_add_pd (y_3, ax_temp );
		
			a_0 = _mm_load_pd( &tA[0+lda*4] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_4 = _mm_add_pd (y_4, ax_temp );

			a_0 = _mm_load_pd( &tA[0+lda*5] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_5 = _mm_add_pd (y_5, ax_temp );

			a_0 = _mm_load_pd( &tA[0+lda*6] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_6 = _mm_add_pd (y_6, ax_temp );

			a_0 = _mm_load_pd( &tA[0+lda*7] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_7 = _mm_add_pd (y_7, ax_temp );
		
			a_0 = _mm_load_pd( &tA[0+lda*8] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_8 = _mm_add_pd (y_8, ax_temp );
		
			a_0 = _mm_load_pd( &tA[0+lda*9] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_9 = _mm_add_pd (y_9, ax_temp );
		
			a_0 = _mm_load_pd( &tA[0+lda*10] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_a = _mm_add_pd (y_a, ax_temp );
		
			a_0 = _mm_load_pd( &tA[0+lda*11] );
			ax_temp = _mm_mul_pd( a_0, x_0 );	
			y_b = _mm_add_pd (y_b, ax_temp );
		
			tA += 2;
			tx += 2;
			k  += 2;
		
			}

		if((ka-k)==1)
			{
		
			x_0 = _mm_load_sd( &tx[0] );

			a_0 = _mm_load_sd( &tA[0+lda*0] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_0 = _mm_add_sd (y_0, ax_temp );

			a_0 = _mm_load_sd( &tA[0+lda*1] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_1 = _mm_add_sd (y_1, ax_temp );

			a_0 = _mm_load_sd( &tA[0+lda*2] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_2 = _mm_add_sd (y_2, ax_temp );

			a_0 = _mm_load_sd( &tA[0+lda*3] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_3 = _mm_add_sd (y_3, ax_temp );
		
			a_0 = _mm_load_sd( &tA[0+lda*4] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_4 = _mm_add_sd (y_4, ax_temp );

			a_0 = _mm_load_sd( &tA[0+lda*5] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_5 = _mm_add_sd (y_5, ax_temp );

			a_0 = _mm_load_sd( &tA[0+lda*6] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_6 = _mm_add_sd (y_6, ax_temp );

			a_0 = _mm_load_sd( &tA[0+lda*7] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_7 = _mm_add_sd (y_7, ax_temp );
		
			a_0 = _mm_load_sd( &tA[0+lda*8] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_8 = _mm_add_sd (y_8, ax_temp );
		
			a_0 = _mm_load_sd( &tA[0+lda*9] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_9 = _mm_add_sd (y_9, ax_temp );
		
			a_0 = _mm_load_sd( &tA[0+lda*10] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_a = _mm_add_sd (y_a, ax_temp );
		
			a_0 = _mm_load_sd( &tA[0+lda*11] );
			ax_temp = _mm_mul_sd( a_0, x_0 );	
			y_b = _mm_add_sd (y_b, ax_temp );
		
			tA += 1;
			tx += 1;
			k++;
		
			}

		}

	y_00 = _mm256_castpd128_pd256(y_0);
	y_11 = _mm256_castpd128_pd256(y_1);
	y_22 = _mm256_castpd128_pd256(y_2);
	y_33 = _mm256_castpd128_pd256(y_3);
	y_44 = _mm256_castpd128_pd256(y_4);
	y_55 = _mm256_castpd128_pd256(y_5);
	y_66 = _mm256_castpd128_pd256(y_6);
	y_77 = _mm256_castpd128_pd256(y_7);
	y_88 = _mm256_castpd128_pd256(y_8);
	y_99 = _mm256_castpd128_pd256(y_9);
	y_aa = _mm256_castpd128_pd256(y_a);
	y_bb = _mm256_castpd128_pd256(y_b);
		
#if (ENABLE_PREFETCH==1)
	if(kmax<=64)
		{
#endif

		k=0;
		for(; k<ka-7; k+=8)
			{

//			__builtin_prefetch( A + sda*lda + 0*lda );
//			__builtin_prefetch( A + sda*lda + 2*lda );

			x_00 = _mm256_loadu_pd( &x[0] );
		
			a_00 = _mm256_load_pd( &A[0+lda*0] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*1] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*2] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*3] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
//			__builtin_prefetch( A + sda*lda + 4*lda );
//			__builtin_prefetch( A + sda*lda + 6*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*4] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*5] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*6] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*7] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

//			__builtin_prefetch( A + sda*lda + 8*lda );
//			__builtin_prefetch( A + sda*lda + 10*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*8] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_88 = _mm256_add_pd( y_88, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*9] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_99 = _mm256_add_pd( y_99, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*10] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_aa = _mm256_add_pd( y_aa, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*11] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_bb = _mm256_add_pd( y_bb, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;


//			__builtin_prefetch( A + sda*lda + 0*lda );
//			__builtin_prefetch( A + sda*lda + 2*lda );

			x_00 = _mm256_loadu_pd( &x[0] );
		
			a_00 = _mm256_load_pd( &A[0+lda*0] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*1] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*2] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*3] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
//			__builtin_prefetch( A + sda*lda + 4*lda );
//			__builtin_prefetch( A + sda*lda + 6*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*4] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*5] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*6] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*7] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

//			__builtin_prefetch( A + sda*lda + 8*lda );
//			__builtin_prefetch( A + sda*lda + 10*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*8] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_88 = _mm256_add_pd( y_88, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*9] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_99 = _mm256_add_pd( y_99, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*10] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_aa = _mm256_add_pd( y_aa, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*11] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_bb = _mm256_add_pd( y_bb, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
		for(; k<ka-3; k+=4)
			{

//			__builtin_prefetch( A + sda*lda + 0*lda );
//			__builtin_prefetch( A + sda*lda + 2*lda );

			x_00 = _mm256_loadu_pd( &x[0] );
		
			a_00 = _mm256_load_pd( &A[0+lda*0] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*1] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*2] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*3] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
//			__builtin_prefetch( A + sda*lda + 4*lda );
//			__builtin_prefetch( A + sda*lda + 6*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*4] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*5] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*6] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*7] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

//			__builtin_prefetch( A + sda*lda + 8*lda );
//			__builtin_prefetch( A + sda*lda + 10*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*8] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_88 = _mm256_add_pd( y_88, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*9] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_99 = _mm256_add_pd( y_99, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*10] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_aa = _mm256_add_pd( y_aa, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*11] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_bb = _mm256_add_pd( y_bb, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
			
#if (ENABLE_PREFETCH==1)
		}
	else
		{

		k=0;
		for(; k<ka-7; k+=8)
			{

			__builtin_prefetch( A + sda*lda + 0*lda );
			__builtin_prefetch( A + sda*lda + 2*lda );

			x_00 = _mm256_loadu_pd( &x[0] );
		
			a_00 = _mm256_load_pd( &A[0+lda*0] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*1] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*2] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*3] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
			__builtin_prefetch( A + sda*lda + 4*lda );
			__builtin_prefetch( A + sda*lda + 6*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*4] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*5] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*6] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*7] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			__builtin_prefetch( A + sda*lda + 8*lda );
			__builtin_prefetch( A + sda*lda + 10*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*8] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_88 = _mm256_add_pd( y_88, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*9] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_99 = _mm256_add_pd( y_99, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*10] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_aa = _mm256_add_pd( y_aa, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*11] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_bb = _mm256_add_pd( y_bb, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;


			__builtin_prefetch( A + sda*lda + 0*lda );
			__builtin_prefetch( A + sda*lda + 2*lda );

			x_00 = _mm256_loadu_pd( &x[0] );
		
			a_00 = _mm256_load_pd( &A[0+lda*0] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*1] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*2] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*3] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
			__builtin_prefetch( A + sda*lda + 4*lda );
			__builtin_prefetch( A + sda*lda + 6*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*4] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*5] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*6] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*7] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			__builtin_prefetch( A + sda*lda + 8*lda );
			__builtin_prefetch( A + sda*lda + 10*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*8] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_88 = _mm256_add_pd( y_88, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*9] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_99 = _mm256_add_pd( y_99, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*10] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_aa = _mm256_add_pd( y_aa, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*11] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_bb = _mm256_add_pd( y_bb, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
		for(; k<ka-3; k+=4)
			{

			__builtin_prefetch( A + sda*lda + 0*lda );
			__builtin_prefetch( A + sda*lda + 2*lda );

			x_00 = _mm256_loadu_pd( &x[0] );
		
			a_00 = _mm256_load_pd( &A[0+lda*0] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*1] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*2] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*3] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
			__builtin_prefetch( A + sda*lda + 4*lda );
			__builtin_prefetch( A + sda*lda + 6*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*4] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*5] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*6] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*7] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			__builtin_prefetch( A + sda*lda + 8*lda );
			__builtin_prefetch( A + sda*lda + 10*lda );
		
			a_00 = _mm256_load_pd( &A[0+lda*8] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_88 = _mm256_add_pd( y_88, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*9] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_99 = _mm256_add_pd( y_99, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*10] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_aa = _mm256_add_pd( y_aa, aaxx_temp );

			a_00 = _mm256_load_pd( &A[0+lda*11] );
			aaxx_temp = _mm256_mul_pd( a_00, x_00 );
			y_bb = _mm256_add_pd( y_bb, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
	
		}
#endif
		
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
		_mm256_storeu_pd(&z[0], y_00);
		_mm256_storeu_pd(&z[4], y_44);
		_mm256_storeu_pd(&z[8], y_88);
		}
	else if(alg==1)
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );
		y_8_9_a_b = _mm256_loadu_pd( &y[8] );
	
		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, y_44 );
		y_8_9_a_b = _mm256_add_pd( y_8_9_a_b, y_88 );
	
		_mm256_storeu_pd(&z[0], y_0_1_2_3);
		_mm256_storeu_pd(&z[4], y_4_5_6_7);
		_mm256_storeu_pd(&z[8], y_8_9_a_b);
		}
	else // alg==-1
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );
		y_8_9_a_b = _mm256_loadu_pd( &y[8] );
	
		y_0_1_2_3 = _mm256_sub_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_sub_pd( y_4_5_6_7, y_44 );
		y_8_9_a_b = _mm256_sub_pd( y_8_9_a_b, y_88 );
	
		_mm256_storeu_pd(&z[0], y_0_1_2_3);
		_mm256_storeu_pd(&z[4], y_4_5_6_7);
		_mm256_storeu_pd(&z[8], y_8_9_a_b);
		}

	}



void kernel_dgemv_t_8_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	double *tA, *tx;

	int k;
	int ka = kmax; // number from aligned positon
	
	__m256d
		aaxx_temp, 
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0_1_2_3,
		y_00, y_11, y_22, y_33, y_44, y_55, y_66, y_77;
	
	__m128d
		ax_temp,
		a_00_10, a_01_11, a_02_12, a_03_13,
		x_0_1,
		y_0, y_1, y_2, y_3, y_4, y_5, y_6, y_7;
	
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_22 = _mm256_setzero_pd();
	y_33 = _mm256_setzero_pd();
	y_44 = _mm256_setzero_pd();
	y_55 = _mm256_setzero_pd();
	y_66 = _mm256_setzero_pd();
	y_77 = _mm256_setzero_pd();
	
	y_0 = _mm256_castpd256_pd128(y_00);
	y_1 = _mm256_castpd256_pd128(y_11);
	y_2 = _mm256_castpd256_pd128(y_22);
	y_3 = _mm256_castpd256_pd128(y_33);
	y_4 = _mm256_castpd256_pd128(y_44);
	y_5 = _mm256_castpd256_pd128(y_55);
	y_6 = _mm256_castpd256_pd128(y_66);
	y_7 = _mm256_castpd256_pd128(y_77);

	k = lda*(ka/lda);
	tA = A + (ka/lda)*sda*lda;
	tx = x + (ka/lda)*lda;

	if(ka-k>0) // it can be only ka-k = {1, 2, 3}
		{
		if((ka-k)>=2)
			{
		
			x_0_1 = _mm_load_pd( &tx[0] );

			a_00_10 = _mm_load_pd( &tA[0+lda*0] );
			a_01_11 = _mm_load_pd( &tA[0+lda*1] );
			a_02_12 = _mm_load_pd( &tA[0+lda*2] );
			a_03_13 = _mm_load_pd( &tA[0+lda*3] );

			ax_temp = _mm_mul_pd( a_00_10, x_0_1 );	
			y_0 = _mm_add_pd (y_0, ax_temp );
			ax_temp = _mm_mul_pd( a_01_11, x_0_1 );	
			y_1 = _mm_add_pd (y_1, ax_temp );
			ax_temp = _mm_mul_pd( a_02_12, x_0_1 );	
			y_2 = _mm_add_pd (y_2, ax_temp );
			ax_temp = _mm_mul_pd( a_03_13, x_0_1 );	
			y_3 = _mm_add_pd (y_3, ax_temp );
		
			a_00_10 = _mm_load_pd( &tA[0+lda*4] );
			a_01_11 = _mm_load_pd( &tA[0+lda*5] );
			a_02_12 = _mm_load_pd( &tA[0+lda*6] );
			a_03_13 = _mm_load_pd( &tA[0+lda*7] );

			ax_temp = _mm_mul_pd( a_00_10, x_0_1 );	
			y_4 = _mm_add_pd (y_4, ax_temp );
			ax_temp = _mm_mul_pd( a_01_11, x_0_1 );	
			y_5 = _mm_add_pd (y_5, ax_temp );
			ax_temp = _mm_mul_pd( a_02_12, x_0_1 );	
			y_6 = _mm_add_pd (y_6, ax_temp );
			ax_temp = _mm_mul_pd( a_03_13, x_0_1 );	
			y_7 = _mm_add_pd (y_7, ax_temp );
		
			tA += 2;
			tx += 2;
			k+=2;
		
			}

		if((ka-k)==1)
			{
		
			x_0_1 = _mm_load_sd( &tx[0] );

			a_00_10 = _mm_load_sd( &tA[0+lda*0] );
			a_01_11 = _mm_load_sd( &tA[0+lda*1] );
			a_02_12 = _mm_load_sd( &tA[0+lda*2] );
			a_03_13 = _mm_load_sd( &tA[0+lda*3] );

			ax_temp = _mm_mul_sd( a_00_10, x_0_1 );	
			y_0 = _mm_add_sd (y_0, ax_temp );
			ax_temp = _mm_mul_sd( a_01_11, x_0_1 );	
			y_1 = _mm_add_sd (y_1, ax_temp );
			ax_temp = _mm_mul_sd( a_02_12, x_0_1 );	
			y_2 = _mm_add_sd (y_2, ax_temp );
			ax_temp = _mm_mul_sd( a_03_13, x_0_1 );	
			y_3 = _mm_add_sd (y_3, ax_temp );
		
			a_00_10 = _mm_load_sd( &tA[0+lda*4] );
			a_01_11 = _mm_load_sd( &tA[0+lda*5] );
			a_02_12 = _mm_load_sd( &tA[0+lda*6] );
			a_03_13 = _mm_load_sd( &tA[0+lda*7] );

			ax_temp = _mm_mul_sd( a_00_10, x_0_1 );	
			y_4 = _mm_add_sd (y_4, ax_temp );
			ax_temp = _mm_mul_sd( a_01_11, x_0_1 );	
			y_5 = _mm_add_sd (y_5, ax_temp );
			ax_temp = _mm_mul_sd( a_02_12, x_0_1 );	
			y_6 = _mm_add_sd (y_6, ax_temp );
			ax_temp = _mm_mul_sd( a_03_13, x_0_1 );	
			y_7 = _mm_add_sd (y_7, ax_temp );
		
			tA += 1;
			tx += 1;
			k++;
		
			}

		}

	y_00 = _mm256_castpd128_pd256(y_0);
	y_11 = _mm256_castpd128_pd256(y_1);
	y_22 = _mm256_castpd128_pd256(y_2);
	y_33 = _mm256_castpd128_pd256(y_3);
	y_44 = _mm256_castpd128_pd256(y_4);
	y_55 = _mm256_castpd128_pd256(y_5);
	y_66 = _mm256_castpd128_pd256(y_6);
	y_77 = _mm256_castpd128_pd256(y_7);
		
#if (ENABLE_PREFETCH==1)
	if(kmax<=64)
		{
#endif

		k=0;
		for(; k<ka-7; k+=8)
			{

//			__builtin_prefetch( A + sda*lda + 0*lda );
//			__builtin_prefetch( A + sda*lda + 2*lda );

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
//			__builtin_prefetch( A + sda*lda + 4*lda );
//			__builtin_prefetch( A + sda*lda + 6*lda );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;


//			__builtin_prefetch( A + sda*lda + 0*lda );
//			__builtin_prefetch( A + sda*lda + 2*lda );

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
//			__builtin_prefetch( A + sda*lda + 4*lda );
//			__builtin_prefetch( A + sda*lda + 6*lda );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
		for(; k<ka-3; k+=4)
			{

//			__builtin_prefetch( A + sda*lda + 0*lda );
//			__builtin_prefetch( A + sda*lda + 2*lda );

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
//			__builtin_prefetch( A + sda*lda + 4*lda );
//			__builtin_prefetch( A + sda*lda + 6*lda );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
			
#if (ENABLE_PREFETCH==1)
		}
	else
		{

		k=0;
		for(; k<ka-7; k+=8)
			{

			__builtin_prefetch( A + sda*lda + 0*lda );
			__builtin_prefetch( A + sda*lda + 2*lda );

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
			__builtin_prefetch( A + sda*lda + 4*lda );
			__builtin_prefetch( A + sda*lda + 6*lda );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;


			__builtin_prefetch( A + sda*lda + 0*lda );
			__builtin_prefetch( A + sda*lda + 2*lda );

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
			__builtin_prefetch( A + sda*lda + 4*lda );
			__builtin_prefetch( A + sda*lda + 6*lda );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
		for(; k<ka-3; k+=4)
			{

			__builtin_prefetch( A + sda*lda + 0*lda );
			__builtin_prefetch( A + sda*lda + 2*lda );

			x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_00 = _mm256_add_pd( y_00, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_11 = _mm256_add_pd( y_11, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_22 = _mm256_add_pd( y_22, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
			__builtin_prefetch( A + sda*lda + 4*lda );
			__builtin_prefetch( A + sda*lda + 6*lda );

			a_00_10_20_30 = _mm256_load_pd( &A[0+lda*4] );
			a_01_11_21_31 = _mm256_load_pd( &A[0+lda*5] );
			a_02_12_22_32 = _mm256_load_pd( &A[0+lda*6] );
			a_03_13_23_33 = _mm256_load_pd( &A[0+lda*7] );
		
			aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
			y_44 = _mm256_add_pd( y_44, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
			y_55 = _mm256_add_pd( y_55, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
			y_66 = _mm256_add_pd( y_66, aaxx_temp );
			aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
			y_77 = _mm256_add_pd( y_77, aaxx_temp );

			A += 4 + (sda-1)*lda;
			x += 4;

			}
	
		}
#endif
		
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
		_mm256_storeu_pd(&z[0], y_00);
		_mm256_storeu_pd(&z[4], y_44);
		}
	else if(alg==1)
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );

		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, y_44 );

		_mm256_storeu_pd(&z[0], y_0_1_2_3);
		_mm256_storeu_pd(&z[4], y_4_5_6_7);
		}
	else // alg==-1
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		y_4_5_6_7 = _mm256_loadu_pd( &y[4] );
	
		y_0_1_2_3 = _mm256_sub_pd( y_0_1_2_3, y_00 );
		y_4_5_6_7 = _mm256_sub_pd( y_4_5_6_7, y_44 );
	
		_mm256_storeu_pd(&z[0], y_0_1_2_3);
		_mm256_storeu_pd(&z[4], y_4_5_6_7);
		}

	}



void kernel_dgemv_t_4_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
#if (ENABLE_PREFETCH==1)
	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );
#endif

	double *tA, *tx;

	int k;
	int ka = kmax; // number from aligned positon
	
	__m256d
		aaxx_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0_1_2_3,
		y_00, y_11, y_22, y_33;
	
	__m128d
		ax_temp,
		a_00_10, a_01_11, a_02_12, a_03_13,
		x_0_1,
		y_0, y_1, y_2, y_3;
	
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_22 = _mm256_setzero_pd();
	y_33 = _mm256_setzero_pd();
	
	y_0 = _mm256_castpd256_pd128(y_00);
	y_1 = _mm256_castpd256_pd128(y_11);
	y_2 = _mm256_castpd256_pd128(y_22);
	y_3 = _mm256_castpd256_pd128(y_33);

	k = lda*(ka/lda);
	tA = A + (ka/lda)*sda*lda;
	tx = x + (ka/lda)*lda;

	for(; k<ka; k++)
		{
		x_0_1 = _mm_load_sd( &tx[0] );

		a_00_10 = _mm_load_sd( &tA[0+lda*0] );
		a_01_11 = _mm_load_sd( &tA[0+lda*1] );
		a_02_12 = _mm_load_sd( &tA[0+lda*2] );
		a_03_13 = _mm_load_sd( &tA[0+lda*3] );
		
			ax_temp = _mm_mul_sd( a_00_10, x_0_1 );	
			y_0 = _mm_add_sd (y_0, ax_temp );
			ax_temp = _mm_mul_sd( a_01_11, x_0_1 );	
			y_1 = _mm_add_sd (y_1, ax_temp );
			ax_temp = _mm_mul_sd( a_02_12, x_0_1 );	
			y_2 = _mm_add_sd (y_2, ax_temp );
			ax_temp = _mm_mul_sd( a_03_13, x_0_1 );	
			y_3 = _mm_add_sd (y_3, ax_temp );
		
		tA += 1;
		tx += 1;

		}

	y_00 = _mm256_castpd128_pd256(y_0);
	y_11 = _mm256_castpd128_pd256(y_1);
	y_22 = _mm256_castpd128_pd256(y_2);
	y_33 = _mm256_castpd128_pd256(y_3);

	k=0;
	for(; k<ka-7; k+=8)
		{
		
#if (ENABLE_PREFETCH==1)
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );
#endif

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_11 = _mm256_add_pd( y_11, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
		A += 4 + (sda-1)*lda;
		x += 4;


#if (ENABLE_PREFETCH==1)
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );
#endif

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_11 = _mm256_add_pd( y_11, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}
	for(; k<ka-3; k+=4)
		{
		
#if (ENABLE_PREFETCH==1)
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );
#endif

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		a_03_13_23_33 = _mm256_load_pd( &A[0+lda*3] );
		
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_11 = _mm256_add_pd( y_11, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_03_13_23_33, x_0_1_2_3 );
		y_33 = _mm256_add_pd( y_33, aaxx_temp );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}

	__m256d
		y_0_1_2_3;

	y_00 = _mm256_hadd_pd(y_00, y_11);
	y_22 = _mm256_hadd_pd(y_22, y_33);

	y_11 = _mm256_permute2f128_pd(y_22, y_00, 2 );	
	y_00 = _mm256_permute2f128_pd(y_22, y_00, 19);	

	y_00 = _mm256_add_pd( y_00, y_11 );

	if(alg==0)
		{
		_mm256_storeu_pd(&z[0], y_00);
		}
	else if(alg==1)
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );

		y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_00 );

		_mm256_storeu_pd(&z[0], y_0_1_2_3);
		}
	else // alg==-1
		{
		y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
	
		y_0_1_2_3 = _mm256_sub_pd( y_0_1_2_3, y_00 );
	
		_mm256_storeu_pd(&z[0], y_0_1_2_3);
		}

	}



void kernel_dgemv_t_3_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
#if (ENABLE_PREFETCH==1)
	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );
#endif

	double *tA, *tx;

	int k;
	int ka = kmax; // number from aligned positon
	
	__m256d
		aaxx_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32,
		x_0_1_2_3,
		y_00, y_11, y_22;
	
	__m128d
		ax_temp,
		a_00_10, a_01_11, a_02_12,
		x_0_1,
		y_0, y_1, y_2;
	
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	y_22 = _mm256_setzero_pd();
	
	y_0 = _mm256_castpd256_pd128(y_00);
	y_1 = _mm256_castpd256_pd128(y_11);
	y_2 = _mm256_castpd256_pd128(y_22);

	k = lda*(ka/lda);
	tA = A + (ka/lda)*sda*lda;
	tx = x + (ka/lda)*lda;

	for(; k<ka; k++)
		{
		x_0_1 = _mm_load_sd( &tx[0] );

		a_00_10 = _mm_load_sd( &tA[0+lda*0] );
		a_01_11 = _mm_load_sd( &tA[0+lda*1] );
		a_02_12 = _mm_load_sd( &tA[0+lda*2] );
		
			ax_temp = _mm_mul_sd( a_00_10, x_0_1 );	
			y_0 = _mm_add_sd (y_0, ax_temp );
			ax_temp = _mm_mul_sd( a_01_11, x_0_1 );	
			y_1 = _mm_add_sd (y_1, ax_temp );
			ax_temp = _mm_mul_sd( a_02_12, x_0_1 );	
			y_2 = _mm_add_sd (y_2, ax_temp );
		
		tA += 1;
		tx += 1;

		}

	y_00 = _mm256_castpd128_pd256(y_0);
	y_11 = _mm256_castpd128_pd256(y_1);
	y_22 = _mm256_castpd128_pd256(y_2);

	k=0;
	for(; k<ka-7; k+=8)
		{
		
#if (ENABLE_PREFETCH==1)
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );
#endif

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_11 = _mm256_add_pd( y_11, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, aaxx_temp );
		
		A += 4 + (sda-1)*lda;
		x += 4;


#if (ENABLE_PREFETCH==1)
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );
#endif

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_11 = _mm256_add_pd( y_11, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, aaxx_temp );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}
	for(; k<ka-3; k+=4)
		{
		
#if (ENABLE_PREFETCH==1)
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );
#endif

		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		a_02_12_22_32 = _mm256_load_pd( &A[0+lda*2] );
		
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_11 = _mm256_add_pd( y_11, aaxx_temp );
		aaxx_temp = _mm256_mul_pd( a_02_12_22_32, x_0_1_2_3 );
		y_22 = _mm256_add_pd( y_22, aaxx_temp );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}

	__m256d
		zeros,
		y_0_1_2_3;

	zeros = _mm256_setzero_pd();

	y_00 = _mm256_hadd_pd(y_00, y_11);
	y_22 = _mm256_hadd_pd(y_22, zeros);

	y_11 = _mm256_permute2f128_pd(y_22, y_00, 2 );	
	y_00 = _mm256_permute2f128_pd(y_22, y_00, 19);	

	y_00 = _mm256_add_pd( y_00, y_11 );

	if(alg==0)
		{
		_mm256_storeu_pd(&z[0], y_00);
		}
	else 
		{

		y_00 = _mm256_blend_pd( zeros, y_00, 0x7 );

		if(alg==1)
			{
			y_0_1_2_3 = _mm256_loadu_pd( &y[0] );

			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, y_00 );

			_mm256_storeu_pd(&z[0], y_0_1_2_3);
			}
		else // alg==-1
			{
			y_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		
			y_0_1_2_3 = _mm256_sub_pd( y_0_1_2_3, y_00 );
		
			_mm256_storeu_pd(&z[0], y_0_1_2_3);
			}
		}

	}



void kernel_dgemv_t_2_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	double *tA, *tx;

	int k;
	int ka = kmax; // number from aligned positon
	
	__m256d
		aaxx_temp,
		a_00_10_20_30, a_01_11_21_31,
		x_0_1_2_3,
		y_00, y_11;
	
	__m128d
		ax_temp,
		a_00_10, a_01_11,
		x_0_1,
		y_0, y_1, y_0_1;
	
	y_00 = _mm256_setzero_pd();
	y_11 = _mm256_setzero_pd();
	
	y_0 = _mm256_castpd256_pd128(y_00);
	y_1 = _mm256_castpd256_pd128(y_11);

	k = lda*(ka/lda);
	tA = A + (ka/lda)*sda*lda;
	tx = x + (ka/lda)*lda;

	for(; k<ka; k++)
		{
		x_0_1 = _mm_load_sd( &tx[0] );

		a_00_10 = _mm_load_sd( &tA[0+lda*0] );
		a_01_11 = _mm_load_sd( &tA[0+lda*1] );
		
/*		y_0 += a_00_10 * x_0_1;*/
		ax_temp = _mm_mul_sd( a_00_10, x_0_1 );	
		y_0 = _mm_add_sd (y_0, ax_temp );
/*		y_1 += a_01_11 * x_0_1;*/
		ax_temp = _mm_mul_sd( a_01_11, x_0_1 );	
		y_1 = _mm_add_sd (y_1, ax_temp );
		
		tA += 1;
		tx += 1;

		}

	y_00 = _mm256_castpd128_pd256(y_0);
	y_11 = _mm256_castpd128_pd256(y_1);

	k=0;
	for(; k<ka-3; k+=4)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		a_01_11_21_31 = _mm256_load_pd( &A[0+lda*1] );
		
/*		y_00 += a_00_10_20_30 * x_0_1_2_3;*/
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
/*		y_11 += a_01_11_21_31 * x_0_1_2_3;*/
		aaxx_temp = _mm256_mul_pd( a_01_11_21_31, x_0_1_2_3 );
		y_11 = _mm256_add_pd( y_11, aaxx_temp );

		A += 4 + (sda-1)*lda;
		x += 4;

		}

	y_00 = _mm256_hadd_pd(y_00, y_11);

	y_1 = _mm256_extractf128_pd(y_00, 1);
	y_0 = _mm256_castpd256_pd128(y_00);

/*	y_0 += y_1;*/
	y_0 = _mm_add_pd( y_0, y_1 );

	if(alg==0)
		{
		_mm_storeu_pd(&z[0], y_0);
		}
	else if(alg==1)
		{
		y_0_1 = _mm_loadu_pd( &y[0] );

/*		y_0_1 += y_0;*/
		y_0_1 = _mm_add_pd( y_0_1, y_0 );

		_mm_storeu_pd(&z[0], y_0_1);
		}
	else // alg==-1
		{
		y_0_1 = _mm_loadu_pd( &y[0] );
	
/*		y_0_1 -= y_0;*/
		y_0_1 = _mm_sub_pd( y_0_1, y_0 );
	
		_mm_storeu_pd(&z[0], y_0_1);
		}

	}



void kernel_dgemv_t_1_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	double *tA, *tx;

	int k;
	int ka = kmax; // number from aligned positon
	
	__m256d
		aaxx_temp,
		a_00_10_20_30,
		x_0_1_2_3,
		y_00;
	
	__m128d
		ax_temp,
		a_00_10,
		x_0_1,
		y_0, y_1, y_0_1;
	
	y_00 = _mm256_setzero_pd();
	
	y_0 = _mm256_castpd256_pd128(y_00);

	k = lda*(ka/lda);
	tA = A + (ka/lda)*sda*lda;
	tx = x + (ka/lda)*lda;

	for(; k<ka; k++)
		{
		x_0_1 = _mm_load_sd( &tx[0] );

		a_00_10 = _mm_load_sd( &tA[0+lda*0] );
		
/*		y_0 += a_00_10 * x_0_1;*/
		ax_temp = _mm_mul_sd( a_00_10, x_0_1 );	
		y_0 = _mm_add_sd (y_0, ax_temp );
		
		tA += 1;
		tx += 1;

		}

	y_00 = _mm256_castpd128_pd256(y_0);

	k=0;
	for(; k<ka-3; k+=4)
		{
		
		x_0_1_2_3 = _mm256_loadu_pd( &x[0] );

		a_00_10_20_30 = _mm256_load_pd( &A[0+lda*0] );
		
/*		y_00 += a_00_10_20_30 * x_0_1_2_3;*/
		aaxx_temp = _mm256_mul_pd( a_00_10_20_30, x_0_1_2_3 );
		y_00 = _mm256_add_pd( y_00, aaxx_temp );
		
		A += 4 + (sda-1)*lda;
		x += 4;

		}

	y_00 = _mm256_hadd_pd(y_00, y_00);

	y_1 = _mm256_extractf128_pd(y_00, 1);
	y_0 = _mm256_castpd256_pd128(y_00);

/*	y_0 += y_1;*/
	y_0 = _mm_add_sd( y_0, y_1 );

	if(alg==0)
		{
		_mm_store_sd(&z[0], y_0);
		}
	else if(alg==1)
		{
		y_0_1 = _mm_load_sd( &y[0] );

/*		y_0_1 += y_0;*/
		y_0_1 = _mm_add_sd( y_0_1, y_0 );

		_mm_store_sd(&z[0], y_0_1);
		}
	else // alg==-1
		{
		y_0_1 = _mm_load_sd( &y[0] );
	
/*		y_0_1 -= y_0;*/
		y_0_1 = _mm_sub_sd( y_0_1, y_0 );
	
		_mm_store_sd(&z[0], y_0_1);
		}

	}



// it moves horizontally inside a block
void kernel_dgemv_n_12_vs_lib4(int km, int kmax, double *A0, int sda, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;

	const int lda = 4;
	
	int k;

	__m256d
		ax_temp,
		a_00, a_01,
		a_40, a_41,
		a_80, a_81,
		x_0, x_1,
		y_0, y_0_b, z_0,
		y_4, y_4_b, z_4,
		y_8, y_8_b, z_8;
	
	__m256i
		mask_i;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double m_left;

	y_0   = _mm256_setzero_pd();	
	y_4   = _mm256_setzero_pd();	
	y_8   = _mm256_setzero_pd();	
	y_0_b = _mm256_setzero_pd();	
	y_4_b = _mm256_setzero_pd();	
	y_8_b = _mm256_setzero_pd();	

#if (ENABLE_PREFETCH==1)
	if(kmax<=64)
		{
#endif

		k=0;
		for(; k<kmax-7; k+=8)
			{

/*			__builtin_prefetch( A0 + 4*lda );*/
/*			__builtin_prefetch( A1 + 4*lda );*/
/*			__builtin_prefetch( A2 + 4*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 6*lda );*/
/*			__builtin_prefetch( A1 + 6*lda );*/
/*			__builtin_prefetch( A2 + 6*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 8*lda );*/
/*			__builtin_prefetch( A1 + 8*lda );*/
/*			__builtin_prefetch( A2 + 8*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

			x_0 = _mm256_broadcast_sd( &x[4] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[5] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 10*lda );*/
/*			__builtin_prefetch( A1 + 10*lda );*/
/*			__builtin_prefetch( A2 + 10*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

			x_0 = _mm256_broadcast_sd( &x[6] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[7] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 8*lda;
			A1 += 8*lda;
			A2 += 8*lda;
			x  += 8;

			}
		for(; k<kmax-3; k+=4)
			{

/*			__builtin_prefetch( A0 + 4*lda );*/
/*			__builtin_prefetch( A1 + 4*lda );*/
/*			__builtin_prefetch( A2 + 4*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 6*lda );*/
/*			__builtin_prefetch( A1 + 6*lda );*/
/*			__builtin_prefetch( A2 + 6*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 4*lda;
			A1 += 4*lda;
			A2 += 4*lda;
			x  += 4;

			}
		
#if (ENABLE_PREFETCH==1)
		}
	else
		{

		k=0;
		for(; k<kmax-7; k+=8)
			{

			__builtin_prefetch( A0 + 4*lda );
			__builtin_prefetch( A1 + 4*lda );
			__builtin_prefetch( A2 + 4*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 6*lda );
			__builtin_prefetch( A1 + 6*lda );
			__builtin_prefetch( A2 + 6*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 8*lda );
			__builtin_prefetch( A1 + 8*lda );
			__builtin_prefetch( A2 + 8*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

			x_0 = _mm256_broadcast_sd( &x[4] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[5] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 10*lda );
			__builtin_prefetch( A1 + 10*lda );
			__builtin_prefetch( A2 + 10*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

			x_0 = _mm256_broadcast_sd( &x[6] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[7] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 8*lda;
			A1 += 8*lda;
			A2 += 8*lda;
			x  += 8;

			}
		for(; k<kmax-3; k+=4)
			{

			__builtin_prefetch( A0 + 4*lda );
			__builtin_prefetch( A1 + 4*lda );
			__builtin_prefetch( A2 + 4*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 6*lda );
			__builtin_prefetch( A1 + 6*lda );
			__builtin_prefetch( A2 + 6*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 4*lda;
			A1 += 4*lda;
			A2 += 4*lda;
			x  += 4;

			}
	
		}
#endif


	if(kmax%4>=2)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );

		a_00 = _mm256_load_pd( &A0[0+lda*0] );
		a_40 = _mm256_load_pd( &A1[0+lda*0] );
		a_80 = _mm256_load_pd( &A2[0+lda*0] );

		ax_temp = _mm256_mul_pd( a_00, x_0 );
		y_0 = _mm256_add_pd( y_0, ax_temp );
		ax_temp = _mm256_mul_pd( a_40, x_0 );
		y_4 = _mm256_add_pd( y_4, ax_temp );
		ax_temp = _mm256_mul_pd( a_80, x_0 );
		y_8 = _mm256_add_pd( y_8, ax_temp );

		x_1 = _mm256_broadcast_sd( &x[1] );

		a_01 = _mm256_load_pd( &A0[0+lda*1] );
		a_41 = _mm256_load_pd( &A1[0+lda*1] );
		a_81 = _mm256_load_pd( &A2[0+lda*1] );

		ax_temp = _mm256_mul_pd( a_01, x_1 );
		y_0_b = _mm256_add_pd( y_0_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41, x_1 );
		y_4_b = _mm256_add_pd( y_4_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_81, x_1 );
		y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
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

		a_00 = _mm256_load_pd( &A0[0+lda*0] );
		a_40 = _mm256_load_pd( &A1[0+lda*0] );
		a_80 = _mm256_load_pd( &A2[0+lda*0] );

		ax_temp = _mm256_mul_pd( a_00, x_0 );
		y_0 = _mm256_add_pd( y_0, ax_temp );
		ax_temp = _mm256_mul_pd( a_40, x_0 );
		y_4 = _mm256_add_pd( y_4, ax_temp );
		ax_temp = _mm256_mul_pd( a_80, x_0 );
		y_8 = _mm256_add_pd( y_8, ax_temp );
		
/*		A0 += 1*lda;*/
/*		A1 += 1*lda;*/
/*		A2 += 1*lda;*/
/*		x  += 1;*/

		}

	if(alg==0)
		{
		goto store;
		}
	else if(alg==1)
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );
		z_8 = _mm256_loadu_pd( &y[8] );

		y_0 = _mm256_add_pd( z_0, y_0 );
		y_4 = _mm256_add_pd( z_4, y_4 );
		y_8 = _mm256_add_pd( z_8, y_8 );

		goto store;
		}
	else // alg==-1
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );
		z_8 = _mm256_loadu_pd( &y[8] );

		y_0 = _mm256_sub_pd( z_0, y_0 );
		y_4 = _mm256_sub_pd( z_4, y_4 );
		y_8 = _mm256_sub_pd( z_8, y_8 );

		goto store;
		}
	
	store:
	_mm256_storeu_pd( &z[0], y_0 );
	_mm256_storeu_pd( &z[4], y_4 );
	if(km>=12)
		_mm256_storeu_pd(&z[8], y_8);
	else
		{
		m_left = km-8.0;
		mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &m_left ) ) );
		_mm256_maskstore_pd( &z[8], mask_i, y_8 );
		}

	}



// it moves horizontally inside a block
void kernel_dgemv_n_12_lib4(int kmax, double *A0, int sda, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;

	const int lda = 4;
	
	int k;

	__m256d
		ax_temp,
		a_00, a_01,
		a_40, a_41,
		a_80, a_81,
		x_0, x_1,
		y_0, y_0_b, z_0,
		y_4, y_4_b, z_4,
		y_8, y_8_b, z_8;
	
	y_0   = _mm256_setzero_pd();	
	y_4   = _mm256_setzero_pd();	
	y_8   = _mm256_setzero_pd();	
	y_0_b = _mm256_setzero_pd();	
	y_4_b = _mm256_setzero_pd();	
	y_8_b = _mm256_setzero_pd();	

#if (ENABLE_PREFETCH==1)
	if(kmax<=64)
		{
#endif

		k=0;
		for(; k<kmax-7; k+=8)
			{

/*			__builtin_prefetch( A0 + 4*lda );*/
/*			__builtin_prefetch( A1 + 4*lda );*/
/*			__builtin_prefetch( A2 + 4*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 6*lda );*/
/*			__builtin_prefetch( A1 + 6*lda );*/
/*			__builtin_prefetch( A2 + 6*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 8*lda );*/
/*			__builtin_prefetch( A1 + 8*lda );*/
/*			__builtin_prefetch( A2 + 8*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

			x_0 = _mm256_broadcast_sd( &x[4] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[5] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 10*lda );*/
/*			__builtin_prefetch( A1 + 10*lda );*/
/*			__builtin_prefetch( A2 + 10*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

			x_0 = _mm256_broadcast_sd( &x[6] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[7] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 8*lda;
			A1 += 8*lda;
			A2 += 8*lda;
			x  += 8;

			}
		for(; k<kmax-3; k+=4)
			{

/*			__builtin_prefetch( A0 + 4*lda );*/
/*			__builtin_prefetch( A1 + 4*lda );*/
/*			__builtin_prefetch( A2 + 4*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

/*			__builtin_prefetch( A0 + 6*lda );*/
/*			__builtin_prefetch( A1 + 6*lda );*/
/*			__builtin_prefetch( A2 + 6*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 4*lda;
			A1 += 4*lda;
			A2 += 4*lda;
			x  += 4;

			}
		
#if (ENABLE_PREFETCH==1)
		}
	else
		{

		k=0;
		for(; k<kmax-7; k+=8)
			{

			__builtin_prefetch( A0 + 4*lda );
			__builtin_prefetch( A1 + 4*lda );
			__builtin_prefetch( A2 + 4*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 6*lda );
			__builtin_prefetch( A1 + 6*lda );
			__builtin_prefetch( A2 + 6*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 8*lda );
			__builtin_prefetch( A1 + 8*lda );
			__builtin_prefetch( A2 + 8*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

			x_0 = _mm256_broadcast_sd( &x[4] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[5] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 10*lda );
			__builtin_prefetch( A1 + 10*lda );
			__builtin_prefetch( A2 + 10*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

			x_0 = _mm256_broadcast_sd( &x[6] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[7] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 8*lda;
			A1 += 8*lda;
			A2 += 8*lda;
			x  += 8;

			}
		for(; k<kmax-3; k+=4)
			{

			__builtin_prefetch( A0 + 4*lda );
			__builtin_prefetch( A1 + 4*lda );
			__builtin_prefetch( A2 + 4*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );

			__builtin_prefetch( A0 + 6*lda );
			__builtin_prefetch( A1 + 6*lda );
			__builtin_prefetch( A2 + 6*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00, x_0 );
			y_0 = _mm256_add_pd( y_0, ax_temp );
			a_40 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40, x_0 );
			y_4 = _mm256_add_pd( y_4, ax_temp );
			a_80 = _mm256_load_pd( &A2[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_80, x_0 );
			y_8 = _mm256_add_pd( y_8, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01, x_1 );
			y_0_b = _mm256_add_pd( y_0_b, ax_temp );
			a_41 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41, x_1 );
			y_4_b = _mm256_add_pd( y_4_b, ax_temp );
			a_81 = _mm256_load_pd( &A2[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_81, x_1 );
			y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
			A0 += 4*lda;
			A1 += 4*lda;
			A2 += 4*lda;
			x  += 4;

			}
	
		}
#endif


	if(kmax%4>=2)
		{

		x_0 = _mm256_broadcast_sd( &x[0] );

		a_00 = _mm256_load_pd( &A0[0+lda*0] );
		a_40 = _mm256_load_pd( &A1[0+lda*0] );
		a_80 = _mm256_load_pd( &A2[0+lda*0] );

		ax_temp = _mm256_mul_pd( a_00, x_0 );
		y_0 = _mm256_add_pd( y_0, ax_temp );
		ax_temp = _mm256_mul_pd( a_40, x_0 );
		y_4 = _mm256_add_pd( y_4, ax_temp );
		ax_temp = _mm256_mul_pd( a_80, x_0 );
		y_8 = _mm256_add_pd( y_8, ax_temp );

		x_1 = _mm256_broadcast_sd( &x[1] );

		a_01 = _mm256_load_pd( &A0[0+lda*1] );
		a_41 = _mm256_load_pd( &A1[0+lda*1] );
		a_81 = _mm256_load_pd( &A2[0+lda*1] );

		ax_temp = _mm256_mul_pd( a_01, x_1 );
		y_0_b = _mm256_add_pd( y_0_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_41, x_1 );
		y_4_b = _mm256_add_pd( y_4_b, ax_temp );
		ax_temp = _mm256_mul_pd( a_81, x_1 );
		y_8_b = _mm256_add_pd( y_8_b, ax_temp );
		
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

		a_00 = _mm256_load_pd( &A0[0+lda*0] );
		a_40 = _mm256_load_pd( &A1[0+lda*0] );
		a_80 = _mm256_load_pd( &A2[0+lda*0] );

		ax_temp = _mm256_mul_pd( a_00, x_0 );
		y_0 = _mm256_add_pd( y_0, ax_temp );
		ax_temp = _mm256_mul_pd( a_40, x_0 );
		y_4 = _mm256_add_pd( y_4, ax_temp );
		ax_temp = _mm256_mul_pd( a_80, x_0 );
		y_8 = _mm256_add_pd( y_8, ax_temp );
		
/*		A0 += 1*lda;*/
/*		A1 += 1*lda;*/
/*		A2 += 1*lda;*/
/*		x  += 1;*/

		}

	if(alg==0)
		{
		_mm256_storeu_pd(&z[0], y_0);
		_mm256_storeu_pd(&z[4], y_4);
		_mm256_storeu_pd(&z[8], y_8);
		}
	else if(alg==1)
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );
		z_8 = _mm256_loadu_pd( &y[8] );

		z_0 = _mm256_add_pd( z_0, y_0 );
		z_4 = _mm256_add_pd( z_4, y_4 );
		z_8 = _mm256_add_pd( z_8, y_8 );

		_mm256_storeu_pd(&z[0], z_0);
		_mm256_storeu_pd(&z[4], z_4);
		_mm256_storeu_pd(&z[8], z_8);
		}
	else // alg==-1
		{
		z_0 = _mm256_loadu_pd( &y[0] );
		z_4 = _mm256_loadu_pd( &y[4] );
		z_8 = _mm256_loadu_pd( &y[8] );

		z_0 = _mm256_sub_pd( z_0, y_0 );
		z_4 = _mm256_sub_pd( z_4, y_4 );
		z_8 = _mm256_sub_pd( z_8, y_8 );

		_mm256_storeu_pd(&z[0], z_0);
		_mm256_storeu_pd(&z[4], z_4);
		_mm256_storeu_pd(&z[8], z_8);
		}

	}



// it moves horizontally inside a block
void kernel_dgemv_n_8_vs_lib4(int km, int kmax, double *A0, int sda, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	double *A1 = A0 + 4*sda;

	const int lda = 4;
	
	int k;

	__m256d
		ax_temp,
		a_00_10_20_30, a_01_11_21_31,
		a_40_50_60_70, a_41_51_61_71,
		x_0, x_1,
		y_0_1_2_3, y_0_1_2_3_b, z_0_1_2_3,
		y_4_5_6_7, y_4_5_6_7_b, z_4_5_6_7;
	
	__m256i
		mask_i;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double m_left;

	y_0_1_2_3   = _mm256_setzero_pd();	
	y_4_5_6_7   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_4_5_6_7_b = _mm256_setzero_pd();	

#if (ENABLE_PREFETCH==1)
	if(kmax<=64)
		{
#endif

		k=0;
		for(; k<kmax-7; k+=8)
			{

/*			__builtin_prefetch( A0 + 4*lda );*/
/*			__builtin_prefetch( A1 + 4*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

/*			__builtin_prefetch( A0 + 6*lda );*/
/*			__builtin_prefetch( A1 + 6*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

/*			__builtin_prefetch( A0 + 8*lda );*/
/*			__builtin_prefetch( A1 + 8*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

			x_0 = _mm256_broadcast_sd( &x[4] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[5] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

/*			__builtin_prefetch( A0 + 10*lda );*/
/*			__builtin_prefetch( A1 + 10*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

			x_0 = _mm256_broadcast_sd( &x[6] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[7] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
		
			A0 += 8*lda;
			A1 += 8*lda;
			x  += 8;

			}
		for(; k<kmax-3; k+=4)
			{

/*			__builtin_prefetch( A0 + 4*lda );*/
/*			__builtin_prefetch( A1 + 4*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

/*			__builtin_prefetch( A0 + 5*lda );*/
/*			__builtin_prefetch( A1 + 5*lda );*/

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
		
			A0 += 4*lda;
			A1 += 4*lda;
			x  += 4;

			}
		
#if (ENABLE_PREFETCH==1)
		}
	else
		{

		k=0;
		for(; k<kmax-7; k+=8)
			{

			__builtin_prefetch( A0 + 4*lda );
			__builtin_prefetch( A1 + 4*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

			__builtin_prefetch( A0 + 6*lda );
			__builtin_prefetch( A1 + 6*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

			__builtin_prefetch( A0 + 8*lda );
			__builtin_prefetch( A1 + 8*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[4] );

			x_0 = _mm256_broadcast_sd( &x[4] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*4] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[5] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*5] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

			__builtin_prefetch( A0 + 10*lda );
			__builtin_prefetch( A1 + 10*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[6] );

			x_0 = _mm256_broadcast_sd( &x[6] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*6] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[7] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*7] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
		
			A0 += 8*lda;
			A1 += 8*lda;
			x  += 8;

			}
		for(; k<kmax-3; k+=4)
			{

			__builtin_prefetch( A0 + 4*lda );
			__builtin_prefetch( A1 + 4*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[0] );

			x_0 = _mm256_broadcast_sd( &x[0] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*0] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );

			x_1 = _mm256_broadcast_sd( &x[1] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*1] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );

			__builtin_prefetch( A0 + 5*lda );
			__builtin_prefetch( A1 + 5*lda );

//			x_1 = _mm256_broadcast_pd( (__m128d *) &x[2] );

			x_0 = _mm256_broadcast_sd( &x[2] );
//			x_0 = _mm256_shuffle_pd( x_1, x_1, 0x0 );
			a_00_10_20_30 = _mm256_load_pd( &A0[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_00_10_20_30, x_0 );
			y_0_1_2_3 = _mm256_add_pd( y_0_1_2_3, ax_temp );
			a_40_50_60_70 = _mm256_load_pd( &A1[0+lda*2] );
			ax_temp = _mm256_mul_pd( a_40_50_60_70, x_0 );
			y_4_5_6_7 = _mm256_add_pd( y_4_5_6_7, ax_temp );
			
			x_1 = _mm256_broadcast_sd( &x[3] );
//			x_1 = _mm256_shuffle_pd( x_1, x_1, 0xf );
			a_01_11_21_31 = _mm256_load_pd( &A0[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_01_11_21_31, x_1 );
			y_0_1_2_3_b = _mm256_add_pd( y_0_1_2_3_b, ax_temp );
			a_41_51_61_71 = _mm256_load_pd( &A1[0+lda*3] );
			ax_temp = _mm256_mul_pd( a_41_51_61_71, x_1 );
			y_4_5_6_7_b = _mm256_add_pd( y_4_5_6_7_b, ax_temp );
		
			A0 += 4*lda;
			A1 += 4*lda;
			x  += 4;

			}
	
		}
#endif


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
		goto store;
		}
	else if(alg==1)
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		z_4_5_6_7 = _mm256_loadu_pd( &y[4] );

		y_0_1_2_3 = _mm256_add_pd( z_0_1_2_3, y_0_1_2_3 );
		y_4_5_6_7 = _mm256_add_pd( z_4_5_6_7, y_4_5_6_7 );

		goto store;
		}
	else // alg==-1
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );
		z_4_5_6_7 = _mm256_loadu_pd( &y[4] );

		y_0_1_2_3 = _mm256_sub_pd( z_0_1_2_3, y_0_1_2_3 );
		y_4_5_6_7 = _mm256_sub_pd( z_4_5_6_7, y_4_5_6_7 );

		goto store;
		}
	
	store:
	_mm256_storeu_pd( &z[0], y_0_1_2_3 );
	if(km>=8)
		_mm256_storeu_pd( &z[4], y_4_5_6_7 );
	else
		{
		m_left = km-4.0;
		mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &m_left ) ) );
		_mm256_maskstore_pd( &z[4], mask_i, y_4_5_6_7 );
		}


	}



void kernel_dgemv_n_8_lib4(int kmax, double *A0, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0)
		return;

	kernel_dgemv_n_8_vs_lib4(8, kmax, A0, sda, x, y, z, alg);

	}



// it moves horizontally inside a block
void kernel_dgemv_n_4_vs_lib4(int km, int kmax, double *A, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	int k;

	__m256d
		ax_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_03_13_23_33,
		x_0, x_1, x_2, x_3,
		y_0_1_2_3, y_0_1_2_3_b, y_0_1_2_3_c, y_0_1_2_3_d, z_0_1_2_3;
	
	__m256i
		mask_i;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double m_left;

	y_0_1_2_3   = _mm256_setzero_pd();	
	y_0_1_2_3_b = _mm256_setzero_pd();	
	y_0_1_2_3_c = _mm256_setzero_pd();	
	y_0_1_2_3_d = _mm256_setzero_pd();	

	k=0;
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
		
/*		A += 1*lda;*/
/*		x += 1;*/

		}

	if(alg==0)
		{
		goto store;
		}
	else if(alg==1)
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );

		y_0_1_2_3 = _mm256_add_pd ( z_0_1_2_3, y_0_1_2_3 );

		goto store;
		}
	else // alg==-1
		{
		z_0_1_2_3 = _mm256_loadu_pd( &y[0] );

		y_0_1_2_3 = _mm256_sub_pd ( z_0_1_2_3, y_0_1_2_3 );

		goto store;
		}

	store:
	if(km>=4)
		_mm256_storeu_pd(&z[0], y_0_1_2_3);
	else
		{
		m_left = km-0.0;
		mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &m_left ) ) );
		_mm256_maskstore_pd(&z[0], mask_i, y_0_1_2_3);
		}

	}



void kernel_dgemv_n_4_lib4(int kmax, double *A, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0)
		return;
	
	kernel_dgemv_n_4_vs_lib4(4, kmax, A, x, y, z, alg);

	}



// it moves horizontally inside a block
void kernel_dgemv_n_2_lib4(int kmax, double *A, double *x, double *y, double *z, int alg)
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
	
	y_0_1 = _mm_setzero_pd();	
	y_0_1_b = _mm_setzero_pd();	
	y_0_1_c = _mm_setzero_pd();	
	y_0_1_d = _mm_setzero_pd();	

	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = _mm_loaddup_pd( &x[0] );
		x_1 = _mm_loaddup_pd( &x[1] );

		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		a_01_11 = _mm_load_pd( &A[0+lda*1] );

		x_2 = _mm_loaddup_pd( &x[2] );
		x_3 = _mm_loaddup_pd( &x[3] );

		a_02_12 = _mm_load_pd( &A[0+lda*2] );
		a_03_13 = _mm_load_pd( &A[0+lda*3] );

		ax_temp = _mm_mul_pd( a_00_10, x_0 );
		y_0_1 = _mm_add_pd( y_0_1, ax_temp );
		ax_temp = _mm_mul_pd( a_01_11, x_1 );
		y_0_1_b = _mm_add_pd( y_0_1_b, ax_temp );

		ax_temp = _mm_mul_pd( a_02_12, x_2 );
		y_0_1_c = _mm_add_pd( y_0_1_c, ax_temp );
		ax_temp = _mm_mul_pd( a_03_13, x_3 );
		y_0_1_d = _mm_add_pd( y_0_1_d, ax_temp );
		
		A += 4*lda;
		x += 4;

		}

	y_0_1 = _mm_add_pd( y_0_1, y_0_1_c );
	y_0_1_b = _mm_add_pd( y_0_1_b, y_0_1_d );

	if(kmax%4>=2)
		{

		x_0 = _mm_loaddup_pd( &x[0] );
		x_1 = _mm_loaddup_pd( &x[1] );

		a_00_10 = _mm_load_pd( &A[0+lda*0] );
		a_01_11 = _mm_load_pd( &A[0+lda*1] );

		ax_temp = _mm_mul_pd( a_00_10, x_0 );
		y_0_1 = _mm_add_pd( y_0_1, ax_temp );
		ax_temp = _mm_mul_pd( a_01_11, x_1 );
		y_0_1_b = _mm_add_pd( y_0_1_b, ax_temp );

		A += 2*lda;
		x += 2;

		}

	y_0_1 = _mm_add_pd( y_0_1, y_0_1_b );

	if(kmax%2==1)
		{

		x_0 = _mm_loaddup_pd( &x[0] );

		a_00_10 = _mm_load_pd( &A[0+lda*0] );

		ax_temp = _mm_mul_pd( a_00_10, x_0 );
		y_0_1 = _mm_add_pd( y_0_1, ax_temp );

		}

	if(alg==0)
		{
		_mm_storeu_pd(&z[0], y_0_1);
		}
	else if(alg==1)
		{
		z_0_1 = _mm_loadu_pd( &y[0] );

		z_0_1 = _mm_add_pd( z_0_1, y_0_1 );

		_mm_storeu_pd(&z[0], z_0_1);
		}
	else // alg==-1
		{
		z_0_1 = _mm_loadu_pd( &y[0] );

		z_0_1 = _mm_sub_pd( z_0_1, y_0_1 );

		_mm_storeu_pd(&z[0], z_0_1);
		}

	}



// it moves horizontally inside a block
void kernel_dgemv_n_1_lib4(int kmax, double *A, double *x, double *y, double *z, int alg)
	{
	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	int k;

	__m128d
		ax_temp,
		a_00, a_01, a_02, a_03,
		x_0, x_1, x_2, x_3,
		y_0, y_0_b, y_0_c, y_0_d, z_0;
	
	y_0 = _mm_setzero_pd();	
	y_0_b = _mm_setzero_pd();	
	y_0_c = _mm_setzero_pd();	
	y_0_d = _mm_setzero_pd();	

	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = _mm_load_sd( &x[0] );
		x_1 = _mm_load_sd( &x[1] );

		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_01 = _mm_load_sd( &A[0+lda*1] );

		x_2 = _mm_load_sd( &x[2] );
		x_3 = _mm_load_sd( &x[3] );

		a_02 = _mm_load_sd( &A[0+lda*2] );
		a_03 = _mm_load_sd( &A[0+lda*3] );

/*		y_0 += a_00 * x_0;*/
		ax_temp = _mm_mul_sd( a_00, x_0 );
		y_0 = _mm_add_sd( y_0, ax_temp );
/*		y_0 += a_01 * x_1;*/
		ax_temp = _mm_mul_sd( a_01, x_1 );
		y_0_b = _mm_add_sd( y_0_b, ax_temp );

/*		y_0 += a_02 * x_2;*/
		ax_temp = _mm_mul_sd( a_02, x_2 );
		y_0_c = _mm_add_sd( y_0_c, ax_temp );
/*		y_0 += a_03 * x_3;*/
		ax_temp = _mm_mul_sd( a_03, x_3 );
		y_0_d = _mm_add_sd( y_0_d, ax_temp );
		
		A += 4*lda;
		x += 4;

		}

	y_0 = _mm_add_pd( y_0, y_0_c );
	y_0_b = _mm_add_pd( y_0_b, y_0_d );

	if(kmax%4>=2)
		{

		x_0 = _mm_load_sd( &x[0] );
		x_1 = _mm_load_sd( &x[1] );

		a_00 = _mm_load_sd( &A[0+lda*0] );
		a_01 = _mm_load_sd( &A[0+lda*1] );

/*		y_0 += a_00 * x_0;*/
		ax_temp = _mm_mul_sd( a_00, x_0 );
		y_0 = _mm_add_sd( y_0, ax_temp );
/*		y_0 += a_01 * x_1;*/
		ax_temp = _mm_mul_sd( a_01, x_1 );
		y_0_b = _mm_add_sd( y_0_b, ax_temp );

		A += 2*lda;
		x += 2;

		}

	y_0 = _mm_add_pd( y_0, y_0_b );

	if(kmax%2==1)
		{

		x_0 = _mm_load_sd( &x[0] );

		a_00 = _mm_load_sd( &A[0+lda*0] );

/*		y_0 += a_00 * x_0;*/
		ax_temp = _mm_mul_sd( a_00, x_0 );
		y_0 = _mm_add_sd( y_0, ax_temp );
		
/*		A += 1*lda;*/
/*		x += 1;*/

		}

	if(alg==0)
		{
		_mm_store_sd(&z[0], y_0);
		}
	else if(alg==1)
		{
		z_0 = _mm_load_sd( &y[0] );

/*		z_0 += y_0;*/
		z_0 = _mm_add_sd( z_0, y_0 );

		_mm_store_sd(&z[0], z_0);
		}
	else // alg==-1
		{
		z_0 = _mm_load_sd( &y[0] );

/*		z_0 -= y_0;*/
		z_0 = _mm_sub_sd( z_0, y_0 );

		_mm_store_sd(&z[0], z_0);
		}

	}

#endif


void kernel_dgemv_diag_lib4(int kmax, double *dA, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0)
		return;
	
	int k;

	__m256d
		a0, x0, y0, z0;
	
	__m128d
		a1, x1, y1, z1;
	
	if(alg==0)
		{
		k = 0;
		for( ; k<kmax-7; k+=8)
			{

			a0 = _mm256_loadu_pd( &dA[0] );
			x0 = _mm256_loadu_pd( &x[0] );
			z0 = _mm256_mul_pd( a0, x0 );
			_mm256_storeu_pd( &z[0], z0 );

			a0 = _mm256_loadu_pd( &dA[4] );
			x0 = _mm256_loadu_pd( &x[4] );
			z0 = _mm256_mul_pd( a0, x0 );
			_mm256_storeu_pd( &z[4], z0 );

			dA += 8;
			x  += 8;
			z  += 8;

			}
		for( ; k<kmax-3; k+=4)
			{

			a0 = _mm256_loadu_pd( &dA[0] );
			x0 = _mm256_loadu_pd( &x[0] );
			z0 = _mm256_mul_pd( a0, x0 );
			_mm256_storeu_pd( &z[0], z0 );

			dA += 4;
			x  += 4;
			z  += 4;

			}
		for( ; k<kmax; k++)
			{

			a1 = _mm_load_sd( &dA[0] );
			x1 = _mm_load_sd( &x[0] );
			z1 = _mm_mul_sd( a1, x1 );
			_mm_store_sd( &z[0], z1 );

			dA += 1;
			x  += 1;
			z  += 1;

			}
		}
	else if(alg==1)
		{
		k = 0;
		for( ; k<kmax-7; k+=8)
			{

			a0 = _mm256_loadu_pd( &dA[0] );
			x0 = _mm256_loadu_pd( &x[0] );
			z0 = _mm256_mul_pd( a0, x0 );
			y0 = _mm256_loadu_pd( &y[0] );
			z0 = _mm256_add_pd( y0, z0 );
			_mm256_storeu_pd( &z[0], z0 );

			a0 = _mm256_loadu_pd( &dA[4] );
			x0 = _mm256_loadu_pd( &x[4] );
			z0 = _mm256_mul_pd( a0, x0 );
			y0 = _mm256_loadu_pd( &y[4] );
			z0 = _mm256_add_pd( y0, z0 );
			_mm256_storeu_pd( &z[4], z0 );

			dA += 8;
			x  += 8;
			y  += 8;
			z  += 8;

			}
		for( ; k<kmax-3; k+=4)
			{

			a0 = _mm256_loadu_pd( &dA[0] );
			x0 = _mm256_loadu_pd( &x[0] );
			z0 = _mm256_mul_pd( a0, x0 );
			y0 = _mm256_loadu_pd( &y[0] );
			z0 = _mm256_add_pd( y0, z0 );
			_mm256_storeu_pd( &z[0], z0 );

			dA += 4;
			x  += 4;
			y  += 4;
			z  += 4;

			}
		for( ; k<kmax; k++)
			{

			a1 = _mm_load_sd( &dA[0] );
			x1 = _mm_load_sd( &x[0] );
			z1 = _mm_mul_sd( a1, x1 );
			y1 = _mm_load_sd( &y[0] );
			z1 = _mm_add_sd( y1, z1 );
			_mm_store_sd( &z[0], z1 );

			dA += 1;
			x  += 1;
			y  += 1;
			z  += 1;

			}
		}
	else // if(alg==-1)
		{
		k = 0;
		for( ; k<kmax-7; k+=8)
			{

			a0 = _mm256_loadu_pd( &dA[0] );
			x0 = _mm256_loadu_pd( &x[0] );
			z0 = _mm256_mul_pd( a0, x0 );
			y0 = _mm256_loadu_pd( &y[0] );
			z0 = _mm256_sub_pd( y0, z0 );
			_mm256_storeu_pd( &z[0], z0 );

			a0 = _mm256_loadu_pd( &dA[4] );
			x0 = _mm256_loadu_pd( &x[4] );
			z0 = _mm256_mul_pd( a0, x0 );
			y0 = _mm256_loadu_pd( &y[4] );
			z0 = _mm256_sub_pd( y0, z0 );
			_mm256_storeu_pd( &z[4], z0 );

			dA += 8;
			x  += 8;
			y  += 8;
			z  += 8;

			}
		for( ; k<kmax-3; k+=4)
			{

			a0 = _mm256_loadu_pd( &dA[0] );
			x0 = _mm256_loadu_pd( &x[0] );
			z0 = _mm256_mul_pd( a0, x0 );
			y0 = _mm256_loadu_pd( &y[0] );
			z0 = _mm256_sub_pd( y0, z0 );
			_mm256_storeu_pd( &z[0], z0 );

			dA += 4;
			x  += 4;
			y  += 4;
			z  += 4;

			}
		for( ; k<kmax; k++)
			{

			a1 = _mm_load_sd( &dA[0] );
			x1 = _mm_load_sd( &x[0] );
			z1 = _mm_mul_sd( a1, x1 );
			y1 = _mm_load_sd( &y[0] );
			z1 = _mm_sub_sd( y1, z1 );
			_mm_store_sd( &z[0], z1 );

			dA += 1;
			x  += 1;
			y  += 1;
			z  += 1;

			}
		}
	
	}
			

