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



void kernel_strsv_n_8_lib8(int kmax, int ksv, float *A, float *x, float *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 8;
	
	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );

	int k;

	__m256
		temp,
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
		temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, temp );

		a_01 = _mm256_load_ps( &A[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, temp );

		a_02 = _mm256_load_ps( &A[0+lda*2] );
		x_2  = _mm256_broadcast_ss( &x[2] );
		temp = _mm256_mul_ps( a_02, x_2 );
		y_0_c = _mm256_add_ps( y_0_c, temp );

		a_03 = _mm256_load_ps( &A[0+lda*3] );
		x_3  = _mm256_broadcast_ss( &x[3] );
		temp = _mm256_mul_ps( a_03, x_3 );
		y_0_d = _mm256_add_ps( y_0_d, temp );

		A += 4*lda;
		x += 4;

		__builtin_prefetch( A + 4*lda );
		__builtin_prefetch( A + 6*lda );

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		x_0  = _mm256_broadcast_ss( &x[0] );
		temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, temp );

		a_01 = _mm256_load_ps( &A[0+lda*1] );
		x_1  = _mm256_broadcast_ss( &x[1] );
		temp = _mm256_mul_ps( a_01, x_1 );
		y_0_b = _mm256_add_ps( y_0_b, temp );

		a_02 = _mm256_load_ps( &A[0+lda*2] );
		x_2  = _mm256_broadcast_ss( &x[2] );
		temp = _mm256_mul_ps( a_02, x_2 );
		y_0_c = _mm256_add_ps( y_0_c, temp );

		a_03 = _mm256_load_ps( &A[0+lda*3] );
		x_3  = _mm256_broadcast_ss( &x[3] );
		temp = _mm256_mul_ps( a_03, x_3 );
		y_0_d = _mm256_add_ps( y_0_d, temp );

		A += 4*lda;
		x += 4;

		}
	
	y_0   = _mm256_add_ps( y_0  , y_0_c );
	y_0_b = _mm256_add_ps( y_0_b, y_0_d );
	y_0   = _mm256_add_ps( y_0  , y_0_b );

	z_0 = _mm256_loadu_ps( &y[0] );

	y_0 = _mm256_sub_ps( z_0, y_0 );

	
	// a_00
	a_00 = _mm256_load_ps( &A[0+lda*0] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x01 ); // save element in result vector
	temp = _mm256_shuffle_ps( temp, temp, 0x00 ); // broadcast in line
	temp = _mm256_permute2f128_ps( temp, temp, 0x00 ); // broadcast over line
	temp = _mm256_mul_ps( temp, a_00 );
	y_0 = _mm256_sub_ps( y_0, temp );
	if(ksv==1)
		{
		z_0 = _mm256_blend_ps( z_0, y_0, 0xfe ); // save element in result vector
		goto END;
/*		_mm256_storeu_ps(&y[0], z_0);*/
/*		return;*/
		}

	// a_11
	a_00 = _mm256_load_ps( &A[0+lda*1] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x02 ); // save element in result vector
	temp = _mm256_shuffle_ps( temp, temp, 0x55 ); // broadcast in line
	temp = _mm256_permute2f128_ps( temp, temp, 0x00 ); // broadcast over line
	temp = _mm256_mul_ps( temp, a_00 );
	y_0 = _mm256_sub_ps( y_0, temp );
	if(ksv==2)
		{
		z_0 = _mm256_blend_ps( z_0, y_0, 0xfc ); // save element in result vector
		goto END;
/*		_mm256_storeu_ps(&y[0], z_0);*/
/*		return;*/
		}

	// a_22
	a_00 = _mm256_load_ps( &A[0+lda*2] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x04 ); // save element in result vector
	temp = _mm256_shuffle_ps( temp, temp, 0xaa ); // broadcast in line
	temp = _mm256_permute2f128_ps( temp, temp, 0x00 ); // broadcast over line
	temp = _mm256_mul_ps( temp, a_00 );
	y_0 = _mm256_sub_ps( y_0, temp );
	if(ksv==3)
		{
		z_0 = _mm256_blend_ps( z_0, y_0, 0xf8 ); // save element in result vector
		goto END;
/*		_mm256_storeu_ps(&y[0], z_0);*/
/*		return;*/
		}

	// a_33
	a_00 = _mm256_load_ps( &A[0+lda*3] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x08 ); // save element in result vector
	temp = _mm256_shuffle_ps( temp, temp, 0xff ); // broadcast in line
	temp = _mm256_permute2f128_ps( temp, temp, 0x00 ); // broadcast over line
	temp = _mm256_mul_ps( temp, a_00 );
	y_0 = _mm256_sub_ps( y_0, temp );
	if(ksv==4)
		{
		z_0 = _mm256_blend_ps( z_0, y_0, 0xf0 ); // save element in result vector
		goto END;
/*		_mm256_storeu_ps(&y[0], z_0);*/
/*		return;*/
		}

	// a_44
	a_00 = _mm256_load_ps( &A[0+lda*4] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x10 ); // save element in result vector
	temp = _mm256_shuffle_ps( temp, temp, 0x00 ); // broadcast in line
/*	temp = _mm256_permute2f128_ps( temp, temp, 0x11 ); // broadcast over line*/
	temp = _mm256_mul_ps( temp, a_00 );
	y_0 = _mm256_sub_ps( y_0, temp );
	if(ksv==5)
		{
		z_0 = _mm256_blend_ps( z_0, y_0, 0xe0 ); // save element in result vector
		goto END;
/*		_mm256_storeu_ps(&y[0], z_0);*/
/*		return;*/
		}

	// a_55
	a_00 = _mm256_load_ps( &A[0+lda*5] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x20 ); // save element in result vector
	temp = _mm256_shuffle_ps( temp, temp, 0x55 ); // broadcast in line
/*	temp = _mm256_permute2f128_ps( temp, temp, 0x11 ); // broadcast over line*/
	temp = _mm256_mul_ps( temp, a_00 );
	y_0 = _mm256_sub_ps( y_0, temp );
	if(ksv==6)
		{
		z_0 = _mm256_blend_ps( z_0, y_0, 0xc0 ); // save element in result vector
		goto END;
/*		_mm256_storeu_ps(&y[0], z_0);*/
/*		return;*/
		}

	// a_66
	a_00 = _mm256_load_ps( &A[0+lda*6] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x40 ); // save element in result vector
	temp = _mm256_shuffle_ps( temp, temp, 0xaa ); // broadcast in line
/*	temp = _mm256_permute2f128_ps( temp, temp, 0x11 ); // broadcast over line*/
	temp = _mm256_mul_ps( temp, a_00 );
	y_0 = _mm256_sub_ps( y_0, temp );
	if(ksv==7)
		{
		z_0 = _mm256_blend_ps( z_0, y_0, 0x80 ); // save element in result vector
		goto END;
/*		_mm256_storeu_ps(&y[0], z_0);*/
/*		return;*/
		}

	// a_77
	a_00 = _mm256_load_ps( &A[0+lda*7] );
	temp = _mm256_mul_ps( y_0, a_00 ); // [y_0*a_00, ... ]
	z_0 = _mm256_blend_ps( z_0, temp, 0x80 ); // save element in result vector

END:
	_mm256_storeu_ps(&y[0], z_0);
	return;

	}
	
	
	
/*void kernel_strsv_n_4_lib8(int kmax, int ksv, float *A, float *x, float *y)*/
/*	{*/
/*	*/
/*	const int lda = 8;*/
/*	*/
/*	int k;*/

/*	float*/
/*		x_0, x_1, x_2, x_3,*/
/*		y_0=0, y_1=0, y_2=0, y_3=0;*/
/*	*/
/*	k=0;*/
/*	for(; k<kmax-3; k+=4)*/
/*		{*/

/*		x_0 = x[0];*/
/*		x_1 = x[1];*/
/*		x_2 = x[2];*/
/*		x_3 = x[3];*/

/*		y_0 += A[0+lda*0] * x_0;*/
/*		y_1 += A[1+lda*0] * x_0;*/
/*		y_2 += A[2+lda*0] * x_0;*/
/*		y_3 += A[3+lda*0] * x_0;*/

/*		y_0 += A[0+lda*1] * x_1;*/
/*		y_1 += A[1+lda*1] * x_1;*/
/*		y_2 += A[2+lda*1] * x_1;*/
/*		y_3 += A[3+lda*1] * x_1;*/

/*		y_0 += A[0+lda*2] * x_2;*/
/*		y_1 += A[1+lda*2] * x_2;*/
/*		y_2 += A[2+lda*2] * x_2;*/
/*		y_3 += A[3+lda*2] * x_2;*/

/*		y_0 += A[0+lda*3] * x_3;*/
/*		y_1 += A[1+lda*3] * x_3;*/
/*		y_2 += A[2+lda*3] * x_3;*/
/*		y_3 += A[3+lda*3] * x_3;*/
/*		*/
/*		A += 4*lda;*/
/*		x += 4;*/

/*		}*/

/*	y_0 = y[0] - y_0;*/
/*	y_1 = y[1] - y_1;*/
/*	y_2 = y[2] - y_2;*/
/*	y_3 = y[3] - y_3;*/

/*	float*/
/*		a_00, a_10, a_20, a_30,*/
/*		a_11, a_21, a_31;*/
/*	*/
/*	// a_00*/
/*	a_00 = A[0+lda*0];*/
/*	a_10 = A[1+lda*0];*/
/*	a_20 = A[2+lda*0];*/
/*	a_30 = A[3+lda*0];*/
/*	y_0 *= a_00;*/
/*	y[0] = y_0;*/
/*	y_1 -= a_10 * y_0;*/
/*	y_2 -= a_20 * y_0;*/
/*	y_3 -= a_30 * y_0;*/

/*	if(ksv==1)*/
/*		{*/
/*		y[1] = y_1;*/
/*		y[2] = y_2;*/
/*		y[3] = y_3;*/
/*		return;*/
/*		}*/

/*	// a_11*/
/*	a_11 = A[1+lda*1];*/
/*	a_21 = A[2+lda*1];*/
/*	a_31 = A[3+lda*1];*/
/*	y_1 *= a_11;	*/
/*	y[1] = y_1;*/
/*	y_2 -= a_21 * y_1;*/
/*	y_3 -= a_31 * y_1;*/

/*	if(ksv==2)*/
/*		{*/
/*		y[2] = y_2;*/
/*		y[3] = y_3;*/
/*		return;*/
/*		}*/

/*	// a_22*/
/*	a_00 = A[2+lda*2];*/
/*	a_10 = A[3+lda*2];*/
/*	y_2 *= a_00;*/
/*	y[2] = y_2;*/
/*	y_3 -= a_10 * y_2;*/

/*	if(ksv==3)*/
/*		{*/
/*		y[3] = y_3;*/
/*		return;*/
/*		}*/

/*	// a_33*/
/*	a_11 = A[3+lda*3];*/
/*	y_3 *= a_11;	*/
/*	y[3] = y_3;*/

/*	}*/
	
	
	
void kernel_strsv_t_4_lib8(int kmax, int kna, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
/*printf("\n%d %d\n", kmax, kna);*/

	const int lda = 8;
	
	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );

	int
		k;
	
	float *tA, *tx, k_left_d;
	tA = A;
	tx = x;

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
	
	k = 4;
	A += 4;// + (sda-1)*lda;
	x += 4;

	for(; k<4+kna-3; k+=4)
		{
		
		x_0 = _mm256_loadu_ps( &x[-4] );
		x_0 = _mm256_blend_ps( x_0, zeros, 0x0f );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_load_ps( &A[-4+lda*0] );
		y_0 = _mm256_mul_ps( a_00, x_0 );
		a_01 = _mm256_load_ps( &A[-4+lda*1] );
		y_1 = _mm256_mul_ps( a_01, x_0 );
		a_02 = _mm256_load_ps( &A[-4+lda*2] );
		y_2 = _mm256_mul_ps( a_02, x_0 );
		a_03 = _mm256_load_ps( &A[-4+lda*3] );
		y_3 = _mm256_mul_ps( a_03, x_0 );

		A += 4;
		x += 4;

		}

	A += (sda-1)*lda;

/*printf("\n%d\n", k);*/
/*	for(; k<kmax-4; k+=8)*/
	for(; k<kmax-7; k+=8)
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
	
		A += sda*lda;
		x += lda;

		}
/*	for(; k<kmax; k+=4)*/
	if(k<kmax)
		{
		
		k_left_d = 8.0 - (kmax - k);
		
		x_0 = _mm256_loadu_ps( &x[0] );
/*		x_0 = _mm256_blend_ps( x_0, zeros, 0xf0 );*/
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*float temp[8] = {};*/
/*_mm256_storeu_ps( temp, x_0 );*/
/*s_print_mat(1, 8, temp, 1);*/

/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

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
		
/*		A += 4;*/
/*		x += 4;*/

		}
	

	/* reduction */

	__m128
		tmp, tmA,
		z_0, z_1, z_2, z_3;

	y_0 = _mm256_hadd_ps(y_0, y_1);
	y_2 = _mm256_hadd_ps(y_2, y_3);

	y_0 = _mm256_hadd_ps(y_0, y_2);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
	z_0 = _mm_add_ps(z_0, z_1);
	z_1 = _mm_shuffle_ps( z_0, z_0, 0x1 );
	z_2 = _mm_shuffle_ps( z_0, z_0, 0x2 );
	z_3 = _mm_shuffle_ps( z_0, z_0, 0x3 );


	/* solve */
	A = tA;
	x = tx;

	// A_33
	tmp = _mm_load_ss( &x[3] );
	z_3 = _mm_sub_ss( tmp, z_3 );
	tmA = _mm_load_ss( &A[3+lda*3] );
	z_3 = _mm_mul_ss( z_3, tmA );
	_mm_store_ss( &x[3], z_3 );
	
	// A_22
	tmp = _mm_load_ss( &x[2] );
	z_2 = _mm_sub_ss(tmp, z_2 );
	tmA = _mm_load_ss( &A[3+lda*2] );
	tmp = _mm_mul_ss( z_3, tmA );
	z_2 = _mm_sub_ss(z_2, tmp );
	tmA = _mm_load_ss( &A[2+lda*2] );
	z_2 = _mm_mul_ss( z_2, tmA );
	_mm_store_ss( &x[2], z_2 );
	
	// A_11
	tmp = _mm_load_ss( &x[1] );
	z_1 = _mm_sub_ss(tmp, z_1 );
	tmA = _mm_load_ss( &A[3+lda*1] );
	tmp = _mm_mul_ss( z_3, tmA );
	z_1 = _mm_sub_ss(z_1, tmp );
	tmA = _mm_load_ss( &A[2+lda*1] );
	tmp = _mm_mul_ss( z_2, tmA );
	z_1 = _mm_sub_ss(z_1, tmp );
	tmA = _mm_load_ss( &A[1+lda*1] );
	z_1 = _mm_mul_ss( z_1, tmA );
	_mm_store_ss( &x[1], z_1 );
	
	// A_00
	tmp = _mm_load_ss( &x[0] );
	z_0 = _mm_sub_ss(tmp, z_0 );
	tmA = _mm_load_ss( &A[3+lda*0] );
	tmp = _mm_mul_ss( z_3, tmA );
	z_0 = _mm_sub_ss(z_0, tmp );
	tmA = _mm_load_ss( &A[2+lda*0] );
	tmp = _mm_mul_ss( z_2, tmA );
	z_0 = _mm_sub_ss(z_0, tmp );
	tmA = _mm_load_ss( &A[1+lda*0] );
	tmp = _mm_mul_ss( z_1, tmA );
	z_0 = _mm_sub_ss(z_0, tmp );
	tmA = _mm_load_ss( &A[0+lda*0] );
	z_0 = _mm_mul_ss( z_0, tmA );
	_mm_store_ss( &x[0], z_0 );

	}
	
	
	
void kernel_strsv_t_3_lib8(int kmax, int kna, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
/*printf("\n%d %d\n", kmax, kna);*/

	const int lda = 8;
	
	__builtin_prefetch( A + 0*lda );
	__builtin_prefetch( A + 2*lda );

	int
		k;
	
	float *tA, *tx, k_left_d;
	tA = A;
	tx = x;

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
	
	k = 4;
	A += 4;// + (sda-1)*lda;
	x += 4;
	
/*	kna += 1;*/

/*	for(; k<4+kna-3; k+=4)*/
/*	for(; k<4+kna; k+=4)*/
	if(kna==0)
		{
		
		x_0 = _mm256_loadu_ps( &x[-8] );
		x_0 = _mm256_blend_ps( x_0, zeros, 0x7f );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_loadu_ps( &A[-8+lda*0] );
		y_0 = _mm256_mul_ps( a_00, x_0 );
		a_01 = _mm256_loadu_ps( &A[-8+lda*1] );
		y_1 = _mm256_mul_ps( a_01, x_0 );
		a_02 = _mm256_loadu_ps( &A[-8+lda*2] );
		y_2 = _mm256_mul_ps( a_02, x_0 );
/*		a_03 = _mm256_loadu_ps( &A[-4+lda*3] );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

/*		A += 4;*/
/*		x += 4;*/

		}
	else if(kna==4)
		{

		x_0 = _mm256_loadu_ps( &x[-4] );
		x_0 = _mm256_blend_ps( x_0, zeros, 0x07 );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
		__builtin_prefetch( A + sda*lda + 2*lda );

		a_00 = _mm256_loadu_ps( &A[-4+lda*0] );
		y_0 = _mm256_mul_ps( a_00, x_0 );
		a_01 = _mm256_loadu_ps( &A[-4+lda*1] );
		y_1 = _mm256_mul_ps( a_01, x_0 );
		a_02 = _mm256_loadu_ps( &A[-4+lda*2] );
		y_2 = _mm256_mul_ps( a_02, x_0 );
/*		a_03 = _mm256_loadu_ps( &A[-4+lda*3] );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

		A += 4;
		x += 4;
		k += 4;

		}

	A += (sda-1)*lda;

/*printf("\n%d\n", k);*/
/*	for(; k<kmax-4; k+=8)*/
	for(; k<kmax-7; k+=8)
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
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
		A += sda*lda;
		x += lda;

		}
/*	for(; k<kmax; k+=4)*/
	if(k<kmax)
		{
		
		k_left_d = 8.0 - (kmax - k);
		
		x_0 = _mm256_loadu_ps( &x[0] );
/*		x_0 = _mm256_blend_ps( x_0, zeros, 0xf0 );*/
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*float temp[8] = {};*/
/*_mm256_storeu_ps( temp, x_0 );*/
/*s_print_mat(1, 8, temp, 1);*/

/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
		a_02 = _mm256_load_ps( &A[0+lda*2] );
		ax_temp = _mm256_mul_ps( a_02, x_0 );
		y_2 = _mm256_add_ps( y_2, ax_temp );
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
		
/*		A += 4;*/
/*		x += 4;*/

		}
	

	/* reduction */

	__m128
		tmp, tmA,
		z_0, z_1, z_2, z_3;

	y_0 = _mm256_hadd_ps(y_0, y_1);
/*	y_2 = _mm256_hadd_ps(y_2, y_3);*/
	y_2 = _mm256_hadd_ps(y_2, zeros);

	y_0 = _mm256_hadd_ps(y_0, y_2);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
	z_0 = _mm_add_ps(z_0, z_1);
	z_1 = _mm_shuffle_ps( z_0, z_0, 0x1 );
	z_2 = _mm_shuffle_ps( z_0, z_0, 0x2 );
/*	z_3 = _mm_shuffle_ps( z_0, z_0, 0x3 );*/


	/* solve */
	A = tA;
	x = tx;

	// A_22
	tmp = _mm_load_ss( &x[2] );
	z_2 = _mm_sub_ss( tmp, z_2 );
	tmA = _mm_load_ss( &A[2+lda*2] );
	z_2 = _mm_mul_ss( z_2, tmA );
	_mm_store_ss( &x[2], z_2 );
	
	// A_11
	tmp = _mm_load_ss( &x[1] );
	z_1 = _mm_sub_ss(tmp, z_1 );
	tmA = _mm_load_ss( &A[2+lda*1] );
	tmp = _mm_mul_ss( z_2, tmA );
	z_1 = _mm_sub_ss(z_1, tmp );
	tmA = _mm_load_ss( &A[1+lda*1] );
	z_1 = _mm_mul_ss( z_1, tmA );
	_mm_store_ss( &x[1], z_1 );
	
	// A_00
	tmp = _mm_load_ss( &x[0] );
	z_0 = _mm_sub_ss(tmp, z_0 );
	tmA = _mm_load_ss( &A[2+lda*0] );
	tmp = _mm_mul_ss( z_2, tmA );
	z_0 = _mm_sub_ss(z_0, tmp );
	tmA = _mm_load_ss( &A[1+lda*0] );
	tmp = _mm_mul_ss( z_1, tmA );
	z_0 = _mm_sub_ss(z_0, tmp );
	tmA = _mm_load_ss( &A[0+lda*0] );
	z_0 = _mm_mul_ss( z_0, tmA );
	_mm_store_ss( &x[0], z_0 );
	
	// A_00
/*	tmp = _mm_load_ss( &x[0] );*/
/*	z_0 = _mm_sub_ss(tmp, z_0 );*/
/*	tmA = _mm_load_ss( &A[3+lda*0] );*/
/*	tmp = _mm_mul_ss( z_3, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[2+lda*0] );*/
/*	tmp = _mm_mul_ss( z_2, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[1+lda*0] );*/
/*	tmp = _mm_mul_ss( z_1, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[0+lda*0] );*/
/*	z_0 = _mm_mul_ss( z_0, tmA );*/
/*	_mm_store_ss( &x[0], z_0 );*/

	}



/*void kernel_strsv_t_3_lib8_old(int kmax, int kna, float *A, int sda, float *x)*/
/*	{*/

/*	if(kmax<=0) */
/*		return;*/
/*	*/
/*	const int lda = 8;*/
/*	const int bs  = 8;*/
/*	*/
/*	int*/
/*		k;*/
/*	*/
/*	float *tA, *tx;*/
/*	tA = A;*/
/*	tx = x;*/

/*	float*/
/*		x_0, x_1, x_2, x_3,*/
/*		y_0=0, y_1=0, y_2=0;*/
/*	*/
/*	// clean up at the beginning*/
/*	x_3 = x[3];*/

/*	y_0 += A[3+lda*0] * x_3;*/
/*	y_1 += A[3+lda*1] * x_3;*/
/*	y_2 += A[3+lda*2] * x_3;*/

/*	k=4;*/
/*	A += 4;*/
/*	x += 4;*/

/*	for(; k<4+kna-3; k+=4)*/
/*		{*/

/*		x_0 = x[0];*/
/*		x_1 = x[1];*/
/*		x_2 = x[2];*/
/*		x_3 = x[3];*/
/*		*/
/*		y_0 += A[0+lda*0] * x_0;*/
/*		y_1 += A[0+lda*1] * x_0;*/
/*		y_2 += A[0+lda*2] * x_0;*/

/*		y_0 += A[1+lda*0] * x_1;*/
/*		y_1 += A[1+lda*1] * x_1;*/
/*		y_2 += A[1+lda*2] * x_1;*/
/*		*/
/*		y_0 += A[2+lda*0] * x_2;*/
/*		y_1 += A[2+lda*1] * x_2;*/
/*		y_2 += A[2+lda*2] * x_2;*/

/*		y_0 += A[3+lda*0] * x_3;*/
/*		y_1 += A[3+lda*1] * x_3;*/
/*		y_2 += A[3+lda*2] * x_3;*/
/*		*/
/*		A += 4;*/
/*		x += 4;*/

/*		}*/

/*	A += (sda-1)*lda;*/

/*	for(; k<kmax-7; k+=8)*/
/*		{*/
/*		*/
/*		x_0 = x[0];*/
/*		x_1 = x[1];*/
/*		x_2 = x[2];*/
/*		x_3 = x[3];*/
/*		*/
/*		y_0 += A[0+lda*0] * x_0;*/
/*		y_1 += A[0+lda*1] * x_0;*/
/*		y_2 += A[0+lda*2] * x_0;*/

/*		y_0 += A[1+lda*0] * x_1;*/
/*		y_1 += A[1+lda*1] * x_1;*/
/*		y_2 += A[1+lda*2] * x_1;*/
/*		*/
/*		y_0 += A[2+lda*0] * x_2;*/
/*		y_1 += A[2+lda*1] * x_2;*/
/*		y_2 += A[2+lda*2] * x_2;*/

/*		y_0 += A[3+lda*0] * x_3;*/
/*		y_1 += A[3+lda*1] * x_3;*/
/*		y_2 += A[3+lda*2] * x_3;*/
/*		*/
/*//		A += sda*bs;*/
/*//		x += 4;*/

/*		x_0 = x[4];*/
/*		x_1 = x[5];*/
/*		x_2 = x[6];*/
/*		x_3 = x[7];*/
/*		*/
/*		y_0 += A[4+lda*0] * x_0;*/
/*		y_1 += A[4+lda*1] * x_0;*/
/*		y_2 += A[4+lda*2] * x_0;*/

/*		y_0 += A[5+lda*0] * x_1;*/
/*		y_1 += A[5+lda*1] * x_1;*/
/*		y_2 += A[5+lda*2] * x_1;*/
/*		*/
/*		y_0 += A[6+lda*0] * x_2;*/
/*		y_1 += A[6+lda*1] * x_2;*/
/*		y_2 += A[6+lda*2] * x_2;*/

/*		y_0 += A[7+lda*0] * x_3;*/
/*		y_1 += A[7+lda*1] * x_3;*/
/*		y_2 += A[7+lda*2] * x_3;*/
/*		*/
/*		A += sda*bs;*/
/*		x += 8;*/

/*		}*/
/*	for(; k<kmax; k++)*/
/*		{*/
/*		*/
/*		x_0 = x[0];*/
/*		*/
/*		y_0 += A[0+lda*0] * x_0;*/
/*		y_1 += A[0+lda*1] * x_0;*/
/*		y_2 += A[0+lda*2] * x_0;*/
/*		*/
/*		A += 1;//sda*bs;*/
/*		x += 1;*/

/*		}*/

/*	A = tA;*/
/*	x = tx;*/

/*	// bottom trinagle*/
/*	y_2  = x[2] - y_2;*/
/*	y_2 *= A[2+lda*2];*/
/*	x[2] = y_2;*/

/*	// square*/
/*	y_0 += A[2+lda*0]*y_2;*/
/*	y_1 += A[2+lda*1]*y_2;*/
/*		*/
/*	// top trinagle*/
/*	y_1  = x[1] - y_1;*/
/*	y_1 *= A[1+lda*1];*/
/*	x[1] = y_1;*/

/*	y_0  = x[0] - A[1+lda*0] * y_1 - y_0;*/
/*	y_0 *= A[0+lda*0];*/
/*	x[0] = y_0;*/

/*	}*/
	
	
	
void kernel_strsv_t_2_lib8(int kmax, int kna, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
/*printf("\n%d %d\n", kmax, kna);*/

	const int lda = 8;
	
	__builtin_prefetch( A + 0*lda );
/*	__builtin_prefetch( A + 2*lda );*/

	int
		k;
	
	float *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};

	__m256
		mask,
		zeros,
		ax_temp,
		a_00, a_01,// a_02,// a_03,
		x_0,
		y_0, y_1;//, y_2;//, y_3;

/*	mask = _mm256_set_ps( 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 ); */
	mask = _mm256_loadu_ps( mask_f ); 

	zeros = _mm256_setzero_ps();

	y_0 = _mm256_setzero_ps();
	y_1 = _mm256_setzero_ps();
/*	y_2 = _mm256_setzero_ps();*/
/*	y_3 = _mm256_setzero_ps();*/
	
	k = 4;
	A += 4;// + (sda-1)*lda;
	x += 4;
	
/*	kna += 1;*/

/*	for(; k<4+kna-3; k+=4)*/
/*	for(; k<4+kna; k+=4)*/
	if(kna==0)
		{
		
		x_0 = _mm256_loadu_ps( &x[-8] );
		x_0 = _mm256_blend_ps( x_0, zeros, 0x3f );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_loadu_ps( &A[-8+lda*0] );
		y_0 = _mm256_mul_ps( a_00, x_0 );
		a_01 = _mm256_loadu_ps( &A[-8+lda*1] );
		y_1 = _mm256_mul_ps( a_01, x_0 );
/*		a_02 = _mm256_loadu_ps( &A[-8+lda*2] );*/
/*		y_2 = _mm256_mul_ps( a_02, x_0 );*/
/*		a_03 = _mm256_loadu_ps( &A[-4+lda*3] );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

/*		A += 4;*/
/*		x += 4;*/

		}
	else if(kna==4)
		{

		x_0 = _mm256_loadu_ps( &x[-4] );
		x_0 = _mm256_blend_ps( x_0, zeros, 0x03 );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_loadu_ps( &A[-4+lda*0] );
		y_0 = _mm256_mul_ps( a_00, x_0 );
		a_01 = _mm256_loadu_ps( &A[-4+lda*1] );
		y_1 = _mm256_mul_ps( a_01, x_0 );
/*		a_02 = _mm256_loadu_ps( &A[-4+lda*2] );*/
/*		y_2 = _mm256_mul_ps( a_02, x_0 );*/
/*		a_03 = _mm256_loadu_ps( &A[-4+lda*3] );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

		A += 4;
		x += 4;
		k += 4;

		}

	A += (sda-1)*lda;

/*printf("\n%d\n", k);*/
/*	for(; k<kmax-4; k+=8)*/
	for(; k<kmax-7; k+=8)
		{
		
		x_0 = _mm256_loadu_ps( &x[0] );

		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
		A += sda*lda;
		x += lda;

		}
/*	for(; k<kmax; k+=4)*/
	if(k<kmax)
		{
		
		k_left_d = 8.0 - (kmax - k);
		
		x_0 = _mm256_loadu_ps( &x[0] );
/*		x_0 = _mm256_blend_ps( x_0, zeros, 0xf0 );*/
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*float temp[8] = {};*/
/*_mm256_storeu_ps( temp, x_0 );*/
/*s_print_mat(1, 8, temp, 1);*/

/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
		a_01 = _mm256_load_ps( &A[0+lda*1] );
		ax_temp = _mm256_mul_ps( a_01, x_0 );
		y_1 = _mm256_add_ps( y_1, ax_temp );
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
		
/*		A += 4;*/
/*		x += 4;*/

		}
	

	/* reduction */

	__m128
		tmp, tmA,
		z_0, z_1, z_2, z_3;

	y_0 = _mm256_hadd_ps(y_0, y_1);
/*	y_2 = _mm256_hadd_ps(y_2, y_3);*/
/*	y_2 = _mm256_hadd_ps(y_2, zeros);*/

/*	y_0 = _mm256_hadd_ps(y_0, y_2);*/
	y_0 = _mm256_hadd_ps(y_0, zeros);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
	z_0 = _mm_add_ps(z_0, z_1);
	z_1 = _mm_shuffle_ps( z_0, z_0, 0x1 );
/*	z_2 = _mm_shuffle_ps( z_0, z_0, 0x2 );*/
/*	z_3 = _mm_shuffle_ps( z_0, z_0, 0x3 );*/


	/* solve */
	A = tA;
	x = tx;

	// A_22
	tmp = _mm_load_ss( &x[1] );
	z_1 = _mm_sub_ss( tmp, z_1 );
	tmA = _mm_load_ss( &A[1+lda*1] );
	z_1 = _mm_mul_ss( z_1, tmA );
	_mm_store_ss( &x[1], z_1 );
	
	// A_11
	tmp = _mm_load_ss( &x[0] );
	z_0 = _mm_sub_ss(tmp, z_0 );
	tmA = _mm_load_ss( &A[1+lda*0] );
	tmp = _mm_mul_ss( z_1, tmA );
	z_0 = _mm_sub_ss(z_0, tmp );
	tmA = _mm_load_ss( &A[0+lda*0] );
	z_0 = _mm_mul_ss( z_0, tmA );
	_mm_store_ss( &x[0], z_0 );
	
	// A_00
/*	tmp = _mm_load_ss( &x[0] );*/
/*	z_0 = _mm_sub_ss(tmp, z_0 );*/
/*	tmA = _mm_load_ss( &A[2+lda*0] );*/
/*	tmp = _mm_mul_ss( z_2, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[1+lda*0] );*/
/*	tmp = _mm_mul_ss( z_1, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[0+lda*0] );*/
/*	z_0 = _mm_mul_ss( z_0, tmA );*/
/*	_mm_store_ss( &x[0], z_0 );*/
	
	// A_00
/*	tmp = _mm_load_ss( &x[0] );*/
/*	z_0 = _mm_sub_ss(tmp, z_0 );*/
/*	tmA = _mm_load_ss( &A[3+lda*0] );*/
/*	tmp = _mm_mul_ss( z_3, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[2+lda*0] );*/
/*	tmp = _mm_mul_ss( z_2, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[1+lda*0] );*/
/*	tmp = _mm_mul_ss( z_1, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[0+lda*0] );*/
/*	z_0 = _mm_mul_ss( z_0, tmA );*/
/*	_mm_store_ss( &x[0], z_0 );*/

	}



/*void kernel_strsv_t_2_lib8_old(int kmax, int kna, float *A, int sda, float *x)*/
/*	{*/

/*	if(kmax<=0) */
/*		return;*/
/*	*/
/*	const int lda = 8;*/
/*	const int bs  = 8;*/
/*	*/
/*	int*/
/*		k;*/
/*	*/
/*	float *tA, *tx;*/
/*	tA = A;*/
/*	tx = x;*/

/*	float*/
/*		x_0, x_1, x_2, x_3,*/
/*		y_0=0, y_1=0;*/
/*	*/
/*	// clean up at the beginning*/
/*	x_2 = x[2];*/
/*	x_3 = x[3];*/

/*	y_0 += A[2+lda*0] * x_2;*/
/*	y_1 += A[2+lda*1] * x_2;*/

/*	y_0 += A[3+lda*0] * x_3;*/
/*	y_1 += A[3+lda*1] * x_3;*/

/*	k=4;*/
/*	A += 4;*/
/*	x += 4;*/

/*	for(; k<4+kna-3; k+=4)*/
/*		{*/
/*		*/
/*		x_0 = x[0];*/
/*		x_1 = x[1];*/
/*		x_2 = x[2];*/
/*		x_3 = x[3];*/
/*		*/
/*		y_0 += A[0+lda*0] * x_0;*/
/*		y_1 += A[0+lda*1] * x_0;*/

/*		y_0 += A[1+lda*0] * x_1;*/
/*		y_1 += A[1+lda*1] * x_1;*/
/*		*/
/*		y_0 += A[2+lda*0] * x_2;*/
/*		y_1 += A[2+lda*1] * x_2;*/

/*		y_0 += A[3+lda*0] * x_3;*/
/*		y_1 += A[3+lda*1] * x_3;*/
/*		*/
/*		A += 4;*/
/*		x += 4;*/

/*		}*/

/*	A += (sda-1)*bs;*/

/*	for(; k<kmax-7; k+=8)*/
/*		{*/
/*		*/
/*		x_0 = x[0];*/
/*		x_1 = x[1];*/
/*		x_2 = x[2];*/
/*		x_3 = x[3];*/
/*		*/
/*		y_0 += A[0+lda*0] * x_0;*/
/*		y_1 += A[0+lda*1] * x_0;*/

/*		y_0 += A[1+lda*0] * x_1;*/
/*		y_1 += A[1+lda*1] * x_1;*/
/*		*/
/*		y_0 += A[2+lda*0] * x_2;*/
/*		y_1 += A[2+lda*1] * x_2;*/

/*		y_0 += A[3+lda*0] * x_3;*/
/*		y_1 += A[3+lda*1] * x_3;*/
/*		*/
/*//		A += sda*bs;*/
/*//		x += 4;*/

/*		x_0 = x[4];*/
/*		x_1 = x[5];*/
/*		x_2 = x[6];*/
/*		x_3 = x[7];*/
/*		*/
/*		y_0 += A[4+lda*0] * x_0;*/
/*		y_1 += A[4+lda*1] * x_0;*/

/*		y_0 += A[5+lda*0] * x_1;*/
/*		y_1 += A[5+lda*1] * x_1;*/
/*		*/
/*		y_0 += A[6+lda*0] * x_2;*/
/*		y_1 += A[6+lda*1] * x_2;*/

/*		y_0 += A[7+lda*0] * x_3;*/
/*		y_1 += A[7+lda*1] * x_3;*/
/*		*/
/*		A += sda*bs;*/
/*		x += 8;*/

/*		}*/
/*	for(; k<kmax; k++)*/
/*		{*/
/*		*/
/*		x_0 = x[0];*/
/*		*/
/*		y_0 += A[0+lda*0] * x_0;*/
/*		y_1 += A[0+lda*1] * x_0;*/
/*		*/
/*		A += 1;//sda*bs;*/
/*		x += 1;*/

/*		}*/

/*	A = tA;*/
/*	x = tx;*/

/*	// top trinagle*/
/*	y_1  = x[1] - y_1;*/
/*	y_1 *= A[1+lda*1];*/
/*	x[1] = y_1;*/

/*	y_0  = x[0] - A[1+lda*0] * y_1 - y_0;*/
/*	y_0 *= A[0+lda*0];*/
/*	x[0] = y_0;*/

/*	}*/
	
	
	
void kernel_strsv_t_1_lib8(int kmax, int kna, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
/*printf("\n%d %d\n", kmax, kna);*/

	const int lda = 8;
	
	__builtin_prefetch( A + 0*lda );
/*	__builtin_prefetch( A + 2*lda );*/

	int
		k;
	
	float *tA, *tx, k_left_d;
	tA = A;
	tx = x;

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};

	__m256
		mask,
		zeros,
		ax_temp,
		a_00,// a_01,// a_02,// a_03,
		x_0,
		y_0, y_1;//, y_2;//, y_3;

/*	mask = _mm256_set_ps( 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 ); */
	mask = _mm256_loadu_ps( mask_f ); 

	zeros = _mm256_setzero_ps();

	y_0 = _mm256_setzero_ps();
/*	y_1 = _mm256_setzero_ps();*/
/*	y_2 = _mm256_setzero_ps();*/
/*	y_3 = _mm256_setzero_ps();*/
	
	k = 4;
	A += 4;// + (sda-1)*lda;
	x += 4;
	
/*	kna += 1;*/

/*	for(; k<4+kna-3; k+=4)*/
/*	for(; k<4+kna; k+=4)*/
	if(kna==0)
		{
		
		x_0 = _mm256_loadu_ps( &x[-8] );
		x_0 = _mm256_blend_ps( x_0, zeros, 0x1f );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_loadu_ps( &A[-8+lda*0] );
		y_0 = _mm256_mul_ps( a_00, x_0 );
/*		a_01 = _mm256_loadu_ps( &A[-8+lda*1] );*/
/*		y_1 = _mm256_mul_ps( a_01, x_0 );*/
/*		a_02 = _mm256_loadu_ps( &A[-8+lda*2] );*/
/*		y_2 = _mm256_mul_ps( a_02, x_0 );*/
/*		a_03 = _mm256_loadu_ps( &A[-4+lda*3] );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

/*		A += 4;*/
/*		x += 4;*/

		}
	else if(kna==4)
		{

		x_0 = _mm256_loadu_ps( &x[-4] );
		x_0 = _mm256_blend_ps( x_0, zeros, 0x01 );
		
		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_loadu_ps( &A[-4+lda*0] );
		y_0 = _mm256_mul_ps( a_00, x_0 );
/*		a_01 = _mm256_loadu_ps( &A[-4+lda*1] );*/
/*		y_1 = _mm256_mul_ps( a_01, x_0 );*/
/*		a_02 = _mm256_loadu_ps( &A[-4+lda*2] );*/
/*		y_2 = _mm256_mul_ps( a_02, x_0 );*/
/*		a_03 = _mm256_loadu_ps( &A[-4+lda*3] );*/
/*		y_3 = _mm256_mul_ps( a_03, x_0 );*/

		A += 4;
		x += 4;
		k += 4;

		}

	A += (sda-1)*lda;

/*printf("\n%d\n", k);*/
/*	for(; k<kmax-4; k+=8)*/
	for(; k<kmax-7; k+=8)
		{
		
		x_0 = _mm256_loadu_ps( &x[0] );

		__builtin_prefetch( A + sda*lda + 0*lda );
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
/*		a_01 = _mm256_load_ps( &A[0+lda*1] );*/
/*		ax_temp = _mm256_mul_ps( a_01, x_0 );*/
/*		y_1 = _mm256_add_ps( y_1, ax_temp );*/
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
	
		A += sda*lda;
		x += lda;

		}
/*	for(; k<kmax; k+=4)*/
	if(k<kmax)
		{
		
		k_left_d = 8.0 - (kmax - k);
		
		x_0 = _mm256_loadu_ps( &x[0] );
/*		x_0 = _mm256_blend_ps( x_0, zeros, 0xf0 );*/
		x_0 = _mm256_blendv_ps( x_0, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) ) );
		
/*float temp[8] = {};*/
/*_mm256_storeu_ps( temp, x_0 );*/
/*s_print_mat(1, 8, temp, 1);*/

/*		__builtin_prefetch( A + sda*lda + 0*lda );*/
/*		__builtin_prefetch( A + sda*lda + 2*lda );*/

		a_00 = _mm256_load_ps( &A[0+lda*0] );
		ax_temp = _mm256_mul_ps( a_00, x_0 );
		y_0 = _mm256_add_ps( y_0, ax_temp );
/*		a_01 = _mm256_load_ps( &A[0+lda*1] );*/
/*		ax_temp = _mm256_mul_ps( a_01, x_0 );*/
/*		y_1 = _mm256_add_ps( y_1, ax_temp );*/
/*		a_02 = _mm256_load_ps( &A[0+lda*2] );*/
/*		ax_temp = _mm256_mul_ps( a_02, x_0 );*/
/*		y_2 = _mm256_add_ps( y_2, ax_temp );*/
/*		a_03 = _mm256_load_ps( &A[0+lda*3] );*/
/*		ax_temp = _mm256_mul_ps( a_03, x_0 );*/
/*		y_3 = _mm256_add_ps( y_3, ax_temp );*/
		
/*		A += 4;*/
/*		x += 4;*/

		}
	

	/* reduction */

	__m128
		tmp, tmA,
		z_0, z_1, z_2, z_3;

/*	y_0 = _mm256_hadd_ps(y_0, y_1);*/
	y_0 = _mm256_hadd_ps(y_0, zeros);
/*	y_2 = _mm256_hadd_ps(y_2, y_3);*/
/*	y_2 = _mm256_hadd_ps(y_2, zeros);*/

/*	y_0 = _mm256_hadd_ps(y_0, y_2);*/
	y_0 = _mm256_hadd_ps(y_0, zeros);

	y_1 = _mm256_permute2f128_ps(y_0, y_0, 0x01);
	
	z_0 = _mm256_castps256_ps128(y_0);
	z_1 = _mm256_castps256_ps128(y_1);
	
	z_0 = _mm_add_ps(z_0, z_1);
	z_1 = _mm_shuffle_ps( z_0, z_0, 0x1 );
/*	z_2 = _mm_shuffle_ps( z_0, z_0, 0x2 );*/
/*	z_3 = _mm_shuffle_ps( z_0, z_0, 0x3 );*/


	/* solve */
	A = tA;
	x = tx;

	// A_22
	tmp = _mm_load_ss( &x[0] );
	z_0 = _mm_sub_ss( tmp, z_0 );
	tmA = _mm_load_ss( &A[0+lda*0] );
	z_0 = _mm_mul_ss( z_0, tmA );
	_mm_store_ss( &x[0], z_0 );
	
	// A_11
/*	tmp = _mm_load_ss( &x[0] );*/
/*	z_0 = _mm_sub_ss(tmp, z_0 );*/
/*	tmA = _mm_load_ss( &A[1+lda*0] );*/
/*	tmp = _mm_mul_ss( z_1, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[0+lda*0] );*/
/*	z_0 = _mm_mul_ss( z_0, tmA );*/
/*	_mm_store_ss( &x[0], z_0 );*/
	
	// A_00
/*	tmp = _mm_load_ss( &x[0] );*/
/*	z_0 = _mm_sub_ss(tmp, z_0 );*/
/*	tmA = _mm_load_ss( &A[2+lda*0] );*/
/*	tmp = _mm_mul_ss( z_2, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[1+lda*0] );*/
/*	tmp = _mm_mul_ss( z_1, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[0+lda*0] );*/
/*	z_0 = _mm_mul_ss( z_0, tmA );*/
/*	_mm_store_ss( &x[0], z_0 );*/
	
	// A_00
/*	tmp = _mm_load_ss( &x[0] );*/
/*	z_0 = _mm_sub_ss(tmp, z_0 );*/
/*	tmA = _mm_load_ss( &A[3+lda*0] );*/
/*	tmp = _mm_mul_ss( z_3, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[2+lda*0] );*/
/*	tmp = _mm_mul_ss( z_2, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[1+lda*0] );*/
/*	tmp = _mm_mul_ss( z_1, tmA );*/
/*	z_0 = _mm_sub_ss(z_0, tmp );*/
/*	tmA = _mm_load_ss( &A[0+lda*0] );*/
/*	z_0 = _mm_mul_ss( z_0, tmA );*/
/*	_mm_store_ss( &x[0], z_0 );*/

	}



void kernel_strsv_t_1_lib8_old(int kmax, int kna, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 8;
	const int bs  = 8;
	
	int
		k;
	
	float *tA, *tx;
	tA = A;
	tx = x;

	float
		x_0, x_1, x_2, x_3,
		y_0=0;
	
	// clean up at the beginning
	x_1 = x[1];
	x_2 = x[2];
	x_3 = x[3];

	y_0 += A[1+lda*0] * x_1;
	y_0 += A[2+lda*0] * x_2;
	y_0 += A[3+lda*0] * x_3;

	k=4;
	A += 4;
	x += 4;

	for(; k<4+kna-3; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_0 += A[1+lda*0] * x_1;
		y_0 += A[2+lda*0] * x_2;
		y_0 += A[3+lda*0] * x_3;
		
		A += 4;
		x += 4;

		}

	A += (sda-1)*lda;

	for(; k<kmax-7; k+=8)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_0 += A[1+lda*0] * x_1;
		y_0 += A[2+lda*0] * x_2;
		y_0 += A[3+lda*0] * x_3;
		
/*		A += sda*bs;*/
/*		x += 4;*/

		x_0 = x[4];
		x_1 = x[5];
		x_2 = x[6];
		x_3 = x[7];
		
		y_0 += A[4+lda*0] * x_0;
		y_0 += A[5+lda*0] * x_1;
		y_0 += A[6+lda*0] * x_2;
		y_0 += A[7+lda*0] * x_3;
		
		A += sda*bs;
		x += 8;

		}
	for(; k<kmax; k++)
		{
		
		x_0 = x[0];
		
		y_0 += A[0+lda*0] * x_0;
		
		A += 1;//sda*bs;
		x += 1;

		}

	A = tA;
	x = tx;

	// top trinagle
	y_0  = x[0] - y_0;
	y_0 *= A[0+lda*0];
	x[0] = y_0;

	}
	
	
	

