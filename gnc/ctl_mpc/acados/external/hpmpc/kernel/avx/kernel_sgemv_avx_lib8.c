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



void kernel_sgemv_t_8_lib8(int kmax, int kna, float *A, int sda, float *x, float *y, int alg)
	{

/*printf("\nkernel sgemv t 8\n");*/

	if(kmax<=0) 
		return;
	
	const int lda = 8;

	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );
	__builtin_prefetch( A + 4*lda );
	__builtin_prefetch( A + 6*lda );

	int
		k, k_pre, ka=kmax-kna, k_left;
	
	float 
		k_left_d, kna_d = (float) kna;
	
	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};
	
	__m256
		mask,
		zeros,
		ax_temp,
		a_00, a_01, a_02, a_03,
		x_0,
		y_0, y_1, y_2, y_3, y_4, y_5, y_6, y_7;

/*	mask = _mm256_set_ps( 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 ); */
	mask = _mm256_loadu_ps( mask_f ); 

	zeros = _mm256_setzero_ps();

	y_0 = _mm256_setzero_ps();
	y_1 = _mm256_setzero_ps();
	y_2 = _mm256_setzero_ps();
	y_3 = _mm256_setzero_ps();
	y_4 = _mm256_setzero_ps();
	y_5 = _mm256_setzero_ps();
	y_6 = _mm256_setzero_ps();
	y_7 = _mm256_setzero_ps();

	if(kna>0) // it can be only kna = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_pre = kna-lda; // < 0 !!!

		x_0 = _mm256_loadu_ps( &x[k_pre] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( _mm256_broadcast_ss( &kna_d), mask ) );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_load_ps( &A[k_pre+lda*0] );
		a_01 = _mm256_load_ps( &A[k_pre+lda*1] );
		a_02 = _mm256_load_ps( &A[k_pre+lda*2] );
		a_03 = _mm256_load_ps( &A[k_pre+lda*3] );
	
		y_0 = _mm256_mul_ps( a_00, x_0 );
		y_1 = _mm256_mul_ps( a_01, x_0 );
		y_2 = _mm256_mul_ps( a_02, x_0 );
		y_3 = _mm256_mul_ps( a_03, x_0 );
	
		__builtin_prefetch( A + sda*lda + 4*lda );
		__builtin_prefetch( A + sda*lda + 6*lda );

		a_00 = _mm256_load_ps( &A[k_pre+lda*4] );
		a_01 = _mm256_load_ps( &A[k_pre+lda*5] );
		a_02 = _mm256_load_ps( &A[k_pre+lda*6] );
		a_03 = _mm256_load_ps( &A[k_pre+lda*7] );
	
		y_4 = _mm256_mul_ps( a_00, x_0 );
		y_5 = _mm256_mul_ps( a_01, x_0 );
		y_6 = _mm256_mul_ps( a_02, x_0 );
		y_7 = _mm256_mul_ps( a_03, x_0 );

		A += kna + (sda-1)*lda;
		x += kna;
	
	}

	k = 0;
	for(; k<ka-7; k+=8)
		{

		x_0 = _mm256_loadu_ps( &x[0] );

		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
		a_02 = _mm256_load_ps( &A[0+lda*2] );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_2 = _mm256_add_ps( y_2, ax_temp );
		a_03 = _mm256_load_ps( &A[0+lda*3] );
		ax_temp = _mm256_mul_ps( a_03, x_0 );
		y_3 = _mm256_add_ps( y_3, ax_temp );
	
		__builtin_prefetch( A + sda*lda + 4*lda );
		__builtin_prefetch( A + sda*lda + 6*lda );

		a_00 = _mm256_load_ps( &A[0+lda*4] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_4 = _mm256_add_ps( y_4, ax_temp );
		a_01 = _mm256_load_ps( &A[0+lda*5] );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_5 = _mm256_add_ps( y_5, ax_temp );
		a_02 = _mm256_load_ps( &A[0+lda*6] );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_6 = _mm256_add_ps( y_6, ax_temp );
		a_03 = _mm256_load_ps( &A[0+lda*7] );
		ax_temp = _mm256_mul_ps( a_03, x_0 );
		y_7 = _mm256_add_ps( y_7, ax_temp );

		A += sda*lda;
		x += lda;

		}
	
	k_left = ka-k;

	if(k_left>0) // it can be only k_left = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_left_d = 8.0 - k_left;
		
		x_0 = _mm256_loadu_ps( &x[0] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );

/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		a_02 = _mm256_load_ps( &A[0+lda*2] );
		a_03 = _mm256_load_ps( &A[0+lda*3] );
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_2 = _mm256_add_ps( y_2, ax_temp );
		ax_temp = _mm256_mul_ps( a_03, x_0 );
		y_3 = _mm256_add_ps( y_3, ax_temp );
	
/*		__builtin_prefetch( A + sda*lda + 4*lda );*/
/*		__builtin_prefetch( A + sda*lda + 6*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*4] );
		a_01 = _mm256_load_ps( &A[0+lda*5] );
		a_02 = _mm256_load_ps( &A[0+lda*6] );
		a_03 = _mm256_load_ps( &A[0+lda*7] );
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_4 = _mm256_add_ps( y_4, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_5 = _mm256_add_ps( y_5, ax_temp );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_6 = _mm256_add_ps( y_6, ax_temp );
		ax_temp = _mm256_mul_ps( a_03, x_0 );
		y_7 = _mm256_add_ps( y_7, ax_temp );

/*		A += sda*lda;*/
/*		x += lda;*/
	
		}

	__m256
		z_0;

	y_0 = _mm256_hadd_ps(y_0, y_1);
	y_2 = _mm256_hadd_ps(y_2, y_3);
	y_4 = _mm256_hadd_ps(y_4, y_5);
	y_6 = _mm256_hadd_ps(y_6, y_7);

	y_0 = _mm256_hadd_ps(y_0, y_2);
	y_4 = _mm256_hadd_ps(y_4, y_6);

	y_1 = _mm256_permute2f128_ps(y_0, y_4, 0x20);
	y_2 = _mm256_permute2f128_ps(y_0, y_4, 0x31);
	
	y_0 = _mm256_add_ps(y_1, y_2);

	if(alg==0)
		{
		_mm256_storeu_ps(&y[0], y_0);
		}
	else if(alg==1)
		{
		z_0 = _mm256_loadu_ps( &y[0] );

		z_0 = _mm256_add_ps(z_0, y_0);

		_mm256_storeu_ps(&y[0], z_0);
		}
	else // alg==-1
		{
		z_0 = _mm256_loadu_ps( &y[0] );

		z_0 = _mm256_sub_ps(z_0, y_0);

		_mm256_storeu_ps(&y[0], z_0);
		}

	}
	
	
	
void kernel_sgemv_t_4_lib8(int kmax, int kna, float *A, int sda, float *x, float *y, int alg)
	{

/*printf("\nkernel sgemv t 4\n");*/

	if(kmax<=0) 
		return;
	
	const int lda = 8;

	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );

	int
		k, k_pre, ka=kmax-kna, k_left;
	
	float 
		k_left_d, kna_d = (float) kna;

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};

	__m256
		mask,
		zeros,
		ax_temp,
		a_00, a_01, a_02, a_03,
		x_0,
		y_0, y_1, y_2, y_3;

/*	mask = _mm256_set_ps( 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 ); */
	mask = _mm256_loadu_ps( mask_f ); 

	zeros = _mm256_setzero_ps();

	y_0 = _mm256_setzero_ps();
	y_1 = _mm256_setzero_ps();
	y_2 = _mm256_setzero_ps();
	y_3 = _mm256_setzero_ps();

	if(kna>0) // it can be only kna = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_pre = kna-lda; // < 0 !!!

		x_0 = _mm256_loadu_ps( &x[k_pre] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( _mm256_broadcast_ss( &kna_d), mask ) );

		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_load_ps( &A[k_pre+lda*0] );
		a_01 = _mm256_load_ps( &A[k_pre+lda*1] );
		a_02 = _mm256_load_ps( &A[k_pre+lda*2] );
		a_03 = _mm256_load_ps( &A[k_pre+lda*3] );
	
		y_0 = _mm256_mul_ps( a_00, x_0 );
		y_1 = _mm256_mul_ps( a_01, x_0 );
		y_2 = _mm256_mul_ps( a_02, x_0 );
		y_3 = _mm256_mul_ps( a_03, x_0 );

		A += kna + (sda-1)*lda;
		x += kna;
	
	}

	k = 0;
	for(; k<ka-7; k+=8)
		{

		x_0 = _mm256_loadu_ps( &x[0] );

		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		a_02 = _mm256_load_ps( &A[0+lda*2] );
		a_03 = _mm256_load_ps( &A[0+lda*3] );
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_2 = _mm256_add_ps( y_2, ax_temp );
		ax_temp = _mm256_mul_ps( a_03, x_0 );
		y_3 = _mm256_add_ps( y_3, ax_temp );
	
		A += sda*lda;
		x += lda;

		}
	
	k_left = ka-k;

	if(k_left>0) // it can be only k_left = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_left_d = 8.0 - k_left;

		x_0 = _mm256_loadu_ps( &x[0] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		a_02 = _mm256_load_ps( &A[0+lda*2] );
		a_03 = _mm256_load_ps( &A[0+lda*3] );
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_2 = _mm256_add_ps( y_2, ax_temp );
		ax_temp = _mm256_mul_ps( a_03, x_0 );
		y_3 = _mm256_add_ps( y_3, ax_temp );
	
/*		A += sda*lda;*/
/*		x += lda;*/
	
		}

	__m128
		z_0, z_1;

	y_0 = _mm256_hadd_ps(y_0, y_1);
	y_2 = _mm256_hadd_ps(y_2, y_3);

	y_0 = _mm256_hadd_ps(y_0, y_2);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
	z_1 = _mm_add_ps(z_0, z_1);

	if(alg==0)
		{
		_mm_storeu_ps(&y[0], z_1);
		}
	else if(alg==1)
		{
		z_0 = _mm_loadu_ps( &y[0] );

		z_0 = _mm_add_ps(z_0, z_1);

		_mm_storeu_ps(&y[0], z_0);
		}
	else // alg==-1
		{
		z_0 = _mm_loadu_ps( &y[0] );

		z_0 = _mm_sub_ps(z_0, z_1);

		_mm_storeu_ps(&y[0], z_0);
		}

	}
	
	
	
void kernel_sgemv_t_3_lib8(int kmax, int kna, float *A, int sda, float *x, float *y, int alg)
	{

/*printf("\nkernel sgemv t 4\n");*/

	if(kmax<=0) 
		return;
	
	const int lda = 8;

	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );

	int
		k, k_pre, ka=kmax-kna, k_left;
	
	float 
		k_left_d, kna_d = (float) kna;

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};

	__m256
		mask,
		zeros,
		ax_temp,
		a_00, a_01, a_02,// a_03,
		x_0,
		y_0, y_1, y_2;//, y_3;

/*	mask = _mm256_set_ps( 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 ); */
	mask = _mm256_loadu_ps( mask_f ); 

	zeros = _mm256_setzero_ps();

	y_0 = _mm256_setzero_ps();
	y_1 = _mm256_setzero_ps();
	y_2 = _mm256_setzero_ps();
/*	y_3 = _mm256_setzero_ps();*/

	if(kna>0) // it can be only kna = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_pre = kna-lda; // < 0 !!!

		x_0 = _mm256_loadu_ps( &x[k_pre] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( _mm256_broadcast_ss( &kna_d), mask ) );

		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_load_ps( &A[k_pre+lda*0] );
		a_01 = _mm256_load_ps( &A[k_pre+lda*1] );
		a_02 = _mm256_load_ps( &A[k_pre+lda*2] );
/*		a_03 = _mm256_load_ps( &A[k_pre+lda*3] );*/
	
		y_0 = _mm256_mul_ps( a_00, x_0 );
		y_1 = _mm256_mul_ps( a_01, x_0 );
		y_2 = _mm256_mul_ps( a_02, x_0 );
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

		A += kna + (sda-1)*lda;
		x += kna;
	
	}

	k = 0;
	for(; k<ka-7; k+=8)
		{

		x_0 = _mm256_loadu_ps( &x[0] );

		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		a_02 = _mm256_load_ps( &A[0+lda*2] );
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_2 = _mm256_add_ps( y_2, ax_temp );
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
		A += sda*lda;
		x += lda;

		}
	
	k_left = ka-k;

	if(k_left>0) // it can be only k_left = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_left_d = 8.0 - k_left;

		x_0 = _mm256_loadu_ps( &x[0] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		a_02 = _mm256_load_ps( &A[0+lda*2] );
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_2 = _mm256_add_ps( y_2, ax_temp );
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
/*		A += sda*lda;*/
/*		x += lda;*/
	
		}

	__m128
		z_0, z_1;

	y_0 = _mm256_hadd_ps(y_0, y_1);
/*	y_2 = _mm256_hadd_ps(y_2, y_3);*/
	y_2 = _mm256_hadd_ps(y_2, zeros);

	y_0 = _mm256_hadd_ps(y_0, y_2);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
	z_1 = _mm_add_ps(z_0, z_1);

	if(alg==0)
		{
		_mm_storeu_ps(&y[0], z_1);
		}
	else if(alg==1)
		{
		z_0 = _mm_loadu_ps( &y[0] );

		z_0 = _mm_add_ps(z_0, z_1);

		_mm_storeu_ps(&y[0], z_0);
		}
	else // alg==-1
		{
		z_0 = _mm_loadu_ps( &y[0] );

		z_0 = _mm_sub_ps(z_0, z_1);

		_mm_storeu_ps(&y[0], z_0);
		}

	}



void kernel_sgemv_t_2_lib8(int kmax, int kna, float *A, int sda, float *x, float *y, int alg)
	{

/*printf("\nkernel sgemv t 4\n");*/

	if(kmax<=0) 
		return;
	
	const int lda = 8;

	__builtin_prefetch( A + 0*lda );
/*	__builtin_prefetch( A + 2*lda );*/

	int
		k, k_pre, ka=kmax-kna, k_left;
	
	float 
		k_left_d, kna_d = (float) kna;

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};

	__m256
		mask,
		zeros,
		ax_temp,
		a_00, a_01,// a_02, a_03,
		x_0,
		y_0, y_1;//, y_2, y_3;

/*	mask = _mm256_set_ps( 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 ); */
	mask = _mm256_loadu_ps( mask_f ); 

	zeros = _mm256_setzero_ps();

	y_0 = _mm256_setzero_ps();
	y_1 = _mm256_setzero_ps();
/*	y_2 = _mm256_setzero_ps();*/
/*	y_3 = _mm256_setzero_ps();*/

	if(kna>0) // it can be only kna = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_pre = kna-lda; // < 0 !!!

		x_0 = _mm256_loadu_ps( &x[k_pre] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( _mm256_broadcast_ss( &kna_d), mask ) );

		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[k_pre+lda*0] );
		a_01 = _mm256_load_ps( &A[k_pre+lda*1] );
/*		a_02 = _mm256_load_ps( &A[k_pre+lda*2] );*/
/*		a_03 = _mm256_load_ps( &A[k_pre+lda*3] );*/
	
		y_0 = _mm256_mul_ps( a_00, x_0 );
		y_1 = _mm256_mul_ps( a_01, x_0 );
/*		y_2 = _mm256_mul_ps( a_02, x_0 );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

		A += kna + (sda-1)*lda;
		x += kna;
	
	}

	k = 0;
	for(; k<ka-7; k+=8)
		{

		x_0 = _mm256_loadu_ps( &x[0] );

		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
		A += sda*lda;
		x += lda;

		}
	
	k_left = ka-k;

	if(k_left>0) // it can be only k_left = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_left_d = 8.0 - k_left;

		x_0 = _mm256_loadu_ps( &x[0] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
/*		A += sda*lda;*/
/*		x += lda;*/
	
		}

	__m128
		z_0, z_1;

	y_0 = _mm256_hadd_ps(y_0, y_1);
/*	y_2 = _mm256_hadd_ps(y_2, y_3);*/

/*	y_0 = _mm256_hadd_ps(y_0, y_2);*/
	y_0 = _mm256_hadd_ps(y_0, zeros);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
	z_1 = _mm_add_ps(z_0, z_1);

	if(alg==0)
		{
		_mm_storeu_ps(&y[0], z_1);
		}
	else if(alg==1)
		{
		z_0 = _mm_loadu_ps( &y[0] );

		z_0 = _mm_add_ps(z_0, z_1);

		_mm_storeu_ps(&y[0], z_0);
		}
	else // alg==-1
		{
		z_0 = _mm_loadu_ps( &y[0] );

		z_0 = _mm_sub_ps(z_0, z_1);

		_mm_storeu_ps(&y[0], z_0);
		}

	}



void kernel_sgemv_t_1_lib8(int kmax, int kna, float *A, int sda, float *x, float *y, int alg)
	{

/*printf("\nkernel sgemv t 4\n");*/

	if(kmax<=0) 
		return;
	
	const int lda = 8;

	__builtin_prefetch( A + 0*lda );
/*	__builtin_prefetch( A + 2*lda );*/

	int
		k, k_pre, ka=kmax-kna, k_left;
	
	float 
		k_left_d, kna_d = (float) kna;

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};

	__m256
		mask,
		zeros,
		ax_temp,
		a_00,// a_01,// a_02, a_03,
		x_0,
		y_0, y_1;//, y_2, y_3;

/*	mask = _mm256_set_ps( 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 ); */
	mask = _mm256_loadu_ps( mask_f ); 

	zeros = _mm256_setzero_ps();

	y_0 = _mm256_setzero_ps();
/*	y_1 = _mm256_setzero_ps();*/
/*	y_2 = _mm256_setzero_ps();*/
/*	y_3 = _mm256_setzero_ps();*/

	if(kna>0) // it can be only kna = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_pre = kna-lda; // < 0 !!!

		x_0 = _mm256_loadu_ps( &x[k_pre] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( _mm256_broadcast_ss( &kna_d), mask ) );

		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[k_pre+lda*0] );
/*		a_01 = _mm256_load_ps( &A[k_pre+lda*1] );*/
/*		a_02 = _mm256_load_ps( &A[k_pre+lda*2] );*/
/*		a_03 = _mm256_load_ps( &A[k_pre+lda*3] );*/
	
		y_0 = _mm256_mul_ps( a_00, x_0 );
/*		y_1 = _mm256_mul_ps( a_01, x_0 );*/
/*		y_2 = _mm256_mul_ps( a_02, x_0 );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

		A += kna + (sda-1)*lda;
		x += kna;
	
	}

	k = 0;
	for(; k<ka-7; k+=8)
		{

		x_0 = _mm256_loadu_ps( &x[0] );

		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
/*		a_01 = _mm256_load_ps( &A[0+lda*1] );*/
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
/*		ax_temp = _mm256_mul_ps( a_01, x_0 );*/
/*		y_1 = _mm256_add_ps( y_1, ax_temp );*/
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
		A += sda*lda;
		x += lda;

		}
	
	k_left = ka-k;

	if(k_left>0) // it can be only k_left = {1, 2, 3, 4, 5, 6, 7}
		{
		
		k_left_d = 8.0 - k_left;

		x_0 = _mm256_loadu_ps( &x[0] );
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
/*		a_01 = _mm256_load_ps( &A[0+lda*1] );*/
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
	
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
/*		ax_temp = _mm256_mul_ps( a_01, x_0 );*/
/*		y_1 = _mm256_add_ps( y_1, ax_temp );*/
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
/*		A += sda*lda;*/
/*		x += lda;*/
	
		}

	__m128
		z_0, z_1;

/*	y_0 = _mm256_hadd_ps(y_0, y_1);*/
	y_0 = _mm256_hadd_ps(y_0, zeros);
/*	y_2 = _mm256_hadd_ps(y_2, y_3);*/

/*	y_0 = _mm256_hadd_ps(y_0, y_2);*/
	y_0 = _mm256_hadd_ps(y_0, zeros);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
/*	z_1 = _mm_add_ps(z_0, z_1);*/
	z_1 = _mm_add_ss(z_0, z_1);

	if(alg==0)
		{
/*		_mm_storeu_ps(&y[0], z_1);*/
		_mm_store_ss(&y[0], z_1);
		}
	else if(alg==1)
		{
/*		z_0 = _mm_loadu_ps( &y[0] );*/
		z_0 = _mm_load_ss( &y[0] );

/*		z_0 = _mm_add_ps(z_0, z_1);*/
		z_0 = _mm_add_ss(z_0, z_1);

/*		_mm_storeu_ps(&y[0], z_0);*/
		_mm_store_ss(&y[0], z_0);
		}
	else // alg==-1
		{
/*		z_0 = _mm_loadu_ps( &y[0] );*/
		z_0 = _mm_load_ss( &y[0] );

/*		z_0 = _mm_sub_ps(z_0, z_1);*/
		z_0 = _mm_sub_ss(z_0, z_1);

/*		_mm_storeu_ps(&y[0], z_0);*/
		_mm_store_ss(&y[0], z_0);
		}

	}
	
	
	
// it moves horizontally inside a block
void kernel_sgemv_n_16_lib8(int kmax, float *A0, float *A1, float *x, float *y, int alg)
	{

/*printf("\nkernel sgemv n 16\n");*/

	if(kmax<=0) 
		return;
	
	const int lda = 8;
	
	__builtin_prefetch( A0 + 0*lda );
	__builtin_prefetch( A1 + 0*lda );
	__builtin_prefetch( A0 + 1*lda );
	__builtin_prefetch( A1 + 1*lda );

	int k;

	__m256
		ax_temp,
		a_00, a_01, a_80, a_81,
		x_0, x_1,
		y_0, y_0_b, y_8, y_8_b, z_0, z_8;
	
	y_0   = _mm256_setzero_ps();	
	y_0_b = _mm256_setzero_ps();	
	y_8   = _mm256_setzero_ps();	
	y_8_b = _mm256_setzero_ps();	



	k=0;
	for(; k<kmax-3; k+=4)
		{

		__builtin_prefetch( A0 + 4*lda );
		__builtin_prefetch( A1 + 4*lda );

		a_00 = _mm256_load_ps( &A0[0+lda*0] );
		a_80 = _mm256_load_ps( &A1[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_80, x_0 );
		y_8 = _mm256_add_ps( y_8, ax_temp );

		a_01 = _mm256_load_ps( &A0[0+lda*1] );
		a_81 = _mm256_load_ps( &A1[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		ax_temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, ax_temp );
		ax_temp = _mm256_mul_ps( a_81, x_1 );
		y_8_b = _mm256_add_ps( y_8_b, ax_temp );

		__builtin_prefetch( A0 + 6*lda );
		__builtin_prefetch( A1 + 6*lda );

		a_00 = _mm256_load_ps( &A0[0+lda*2] );
		a_80 = _mm256_load_ps( &A1[0+lda*2] );
		x_0  = _mm256_broadcast_ss( &x[2] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_80, x_0 );
		y_8 = _mm256_add_ps( y_8, ax_temp );

		a_01 = _mm256_load_ps( &A0[0+lda*3] );
		a_81 = _mm256_load_ps( &A1[0+lda*3] );
		x_1  = _mm256_broadcast_ss( &x[3] );
		ax_temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, ax_temp );
		ax_temp = _mm256_mul_ps( a_81, x_1 );
		y_8_b = _mm256_add_ps( y_8_b, ax_temp );

		A0 += 4*lda;
		A1 += 4*lda;
		x += 4;

		}

	if(kmax%4>=2)
		{

/*		__builtin_prefetch( A0 + 4*lda );*/
/*		__builtin_prefetch( A1 + 4*lda );*/

		a_00 = _mm256_load_ps( &A0[0+lda*0] );
		a_80 = _mm256_load_ps( &A1[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_80, x_0 );
		y_8 = _mm256_add_ps( y_8, ax_temp );

		a_01 = _mm256_load_ps( &A0[0+lda*1] );
		a_81 = _mm256_load_ps( &A1[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		ax_temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, ax_temp );
		ax_temp = _mm256_mul_ps( a_81, x_1 );
		y_8_b = _mm256_add_ps( y_8_b, ax_temp );

		A0 += 2*lda;
		A1 += 2*lda;
		x += 2;

		}
	
	y_0   = _mm256_add_ps( y_0  , y_0_b );
	y_8   = _mm256_add_ps( y_8  , y_8_b );

	if(kmax%2==1)
		{

		a_00 = _mm256_load_ps( &A0[0+lda*0] );
		a_80 = _mm256_load_ps( &A1[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		ax_temp = _mm256_mul_ps( a_80, x_0 );
		y_8 = _mm256_add_ps( y_8, ax_temp );
		
//		A += 1*lda;
//		x += 1;

		}

	if(alg==0)
		{
		_mm256_storeu_ps(&y[0], y_0);
		_mm256_storeu_ps(&y[8], y_8);
		}
	else if(alg==1)
		{
		z_0 = _mm256_loadu_ps( &y[0] );
		z_8 = _mm256_loadu_ps( &y[8] );

		z_0 = _mm256_add_ps( z_0, y_0 );
		z_8 = _mm256_add_ps( z_8, y_8 );

		_mm256_storeu_ps(&y[0], z_0);
		_mm256_storeu_ps(&y[8], z_8);
		}
	else // alg==-1
		{
		z_0 = _mm256_loadu_ps( &y[0] );
		z_8 = _mm256_loadu_ps( &y[8] );

		z_0 = _mm256_sub_ps( z_0, y_0 );
		z_8 = _mm256_sub_ps( z_8, y_8 );

		_mm256_storeu_ps(&y[0], z_0);
		_mm256_storeu_ps(&y[8], z_8);
		}

	}

	
	
// it moves horizontally inside a block
void kernel_sgemv_n_8_lib8(int kmax, float *A, float *x, float *y, int alg)
	{

/*printf("\nkernel sgemv n 8\n");*/

	if(kmax<=0) 
		return;
	
	const int lda = 8;
	
	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );

	int k;

	__m256
		ax_temp,
		a_00, a_01, a_02, a_03,
		x_0, x_1, x_2, x_3,
		y_0, y_0_b, y_0_c, y_0_d, z_0;
	
	y_0   = _mm256_setzero_ps();	
	y_0_b = _mm256_setzero_ps();	
	y_0_c = _mm256_setzero_ps();	
	y_0_d = _mm256_setzero_ps();	



	k=0;
	for(; k<kmax-7; k+=8)
		{

		__builtin_prefetch( A + 4*lda );
		__builtin_prefetch( A + 6*lda );

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );

		a_01 = _mm256_load_ps( &A[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		ax_temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, ax_temp );

		a_02 = _mm256_load_ps( &A[0+lda*2] );
		x_2  = _mm256_broadcast_ss( &x[2] );
		ax_temp = _mm256_mul_ps( a_02, x_2 );
		y_0_c = _mm256_add_ps( y_0_c, ax_temp );

		a_03 = _mm256_load_ps( &A[0+lda*3] );
		x_3  = _mm256_broadcast_ss( &x[3] );
		ax_temp = _mm256_mul_ps( a_03, x_3 );
		y_0_d = _mm256_add_ps( y_0_d, ax_temp );

		A += 4*lda;
		x += 4;

		__builtin_prefetch( A + 4*lda );
		__builtin_prefetch( A + 6*lda );

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );

		a_01 = _mm256_load_ps( &A[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		ax_temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, ax_temp );

		a_02 = _mm256_load_ps( &A[0+lda*2] );
		x_2  = _mm256_broadcast_ss( &x[2] );
		ax_temp = _mm256_mul_ps( a_02, x_2 );
		y_0_c = _mm256_add_ps( y_0_c, ax_temp );

		a_03 = _mm256_load_ps( &A[0+lda*3] );
		x_3  = _mm256_broadcast_ss( &x[3] );
		ax_temp = _mm256_mul_ps( a_03, x_3 );
		y_0_d = _mm256_add_ps( y_0_d, ax_temp );

		A += 4*lda;
		x += 4;

		}
	for(; k<kmax-3; k+=4)
		{

		__builtin_prefetch( A + 4*lda );
		__builtin_prefetch( A + 6*lda );

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );

		a_01 = _mm256_load_ps( &A[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		ax_temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, ax_temp );

		a_02 = _mm256_load_ps( &A[0+lda*2] );
		x_2  = _mm256_broadcast_ss( &x[2] );
		ax_temp = _mm256_mul_ps( a_02, x_2 );
		y_0_c = _mm256_add_ps( y_0_c, ax_temp );

		a_03 = _mm256_load_ps( &A[0+lda*3] );
		x_3  = _mm256_broadcast_ss( &x[3] );
		ax_temp = _mm256_mul_ps( a_03, x_3 );
		y_0_d = _mm256_add_ps( y_0_d, ax_temp );

		A += 4*lda;
		x += 4;

		}

	y_0   = _mm256_add_ps( y_0  , y_0_c );
	y_0_b = _mm256_add_ps( y_0_b, y_0_d );

	if(kmax%4>=2)
		{

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );

		a_01 = _mm256_load_ps( &A[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		ax_temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, ax_temp );

		A += 2*lda;
		x += 2;

		}
	
	y_0   = _mm256_add_ps( y_0  , y_0_b );

	if(kmax%2==1)
		{

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		
/*		A += 1*lda;*/
/*		x += 1;*/

		}

	if(alg==0)
		{
		_mm256_storeu_ps(&y[0], y_0);
		}
	else if(alg==1)
		{
		z_0 = _mm256_loadu_ps( &y[0] );

		z_0 = _mm256_add_ps( z_0, y_0 );

		_mm256_storeu_ps(&y[0], z_0);
		}
	else // alg==-1
		{
		z_0 = _mm256_loadu_ps( &y[0] );

		z_0 = _mm256_sub_ps( z_0, y_0 );

		_mm256_storeu_ps(&y[0], z_0);
		}

	}

	
	
void kernel_sgemv_n_4_lib8(int kmax, float *A, float *x, float *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 8;
	
	int k;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;
		y_2 += A[2+lda*0] * x_0;
		y_3 += A[3+lda*0] * x_0;

		y_0 += A[0+lda*1] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[2+lda*1] * x_1;
		y_3 += A[3+lda*1] * x_1;

		y_0 += A[0+lda*2] * x_2;
		y_1 += A[1+lda*2] * x_2;
		y_2 += A[2+lda*2] * x_2;
		y_3 += A[3+lda*2] * x_2;

		y_0 += A[0+lda*3] * x_3;
		y_1 += A[1+lda*3] * x_3;
		y_2 += A[2+lda*3] * x_3;
		y_3 += A[3+lda*3] * x_3;
		
		A += 4*lda;
		x += 4;

		}

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;
		y_2 += A[2+lda*0] * x_0;
		y_3 += A[3+lda*0] * x_0;
		
		A += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		y[0] = y_0;
		y[1] = y_1;
		y[2] = y_2;
		y[3] = y_3;
		}
	else if(alg==1)
		{
		y[0] += y_0;
		y[1] += y_1;
		y[2] += y_2;
		y[3] += y_3;
		}
	else // alg==-1
		{
		y[0] -= y_0;
		y[1] -= y_1;
		y[2] -= y_2;
		y[3] -= y_3;
		}

	}
	
	
	
void kernel_sgemv_n_2_lib8(int kmax, float *A, float *x, float *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 8;
	
	int k;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;

		y_0 += A[0+lda*1] * x_1;
		y_1 += A[1+lda*1] * x_1;

		y_0 += A[0+lda*2] * x_2;
		y_1 += A[1+lda*2] * x_2;

		y_0 += A[0+lda*3] * x_3;
		y_1 += A[1+lda*3] * x_3;
		
		A += 4*lda;
		x += 4;

		}

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;
		
		A += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		y[0] = y_0;
		y[1] = y_1;
		}
	else if(alg==1)
		{
		y[0] += y_0;
		y[1] += y_1;
		}
	else // alg==-1
		{
		y[0] -= y_0;
		y[1] -= y_1;
		}

	}
	
	
	
void kernel_sgemv_n_1_lib8(int kmax, float *A, float *x, float *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 8;
	
	int k;

	float
		x_0, x_1, x_2, x_3,
		y_0=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A[0+lda*0] * x_0;
		y_0 += A[0+lda*1] * x_1;
		y_0 += A[0+lda*2] * x_2;
		y_0 += A[0+lda*3] * x_3;
		
		A += 4*lda;
		x += 4;

		}

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A[0+lda*0] * x_0;
		
		A += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		y[0] = y_0;
		}
	else if(alg==1)
		{
		y[0] += y_0;
		}
	else // alg==-1
		{
		y[0] -= y_0;
		}

	}
	
	
	

