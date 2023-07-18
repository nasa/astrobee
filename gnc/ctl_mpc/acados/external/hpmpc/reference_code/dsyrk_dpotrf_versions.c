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

#include <math.h>



void kernel_dgemm_nt_8x4_unpack_lib(int kmax, double *A, int lda, double *B, int ldb, double *C, int ldc, double *D, int ldd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	int k;

	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		temp,
		ab_tmp0, ab_tmp1, // temporary results
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	// prefetch
	a_0123 = _mm256_load_pd( &A[0+0*lda] );
	a_4567 = _mm256_load_pd( &A[4+0*lda] );
	b_0123 = _mm256_load_pd( &B[0+0*ldb] );

	for(k=0; k<kmax-3; k+=4)
		{
		
		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123  = _mm256_load_pd( &A[0+1*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[0+1*ldb] ); // prefetch
		d_4    = _mm256_add_pd( d_4, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4567  = _mm256_load_pd( &A[4+1*lda] ); // prefetch
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_5    = _mm256_add_pd( d_5, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
		d_7    = _mm256_add_pd( d_7, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_6    = _mm256_add_pd( d_6, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123  = _mm256_load_pd( &A[0+2*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[0+2*ldb] ); // prefetch
		d_4    = _mm256_add_pd( d_4, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4567  = _mm256_load_pd( &A[4+2*lda] ); // prefetch
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
		d_5    = _mm256_add_pd( d_5, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
		d_7    = _mm256_add_pd( d_7, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
		d_6    = _mm256_add_pd( d_6, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123  = _mm256_load_pd( &A[0+3*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[0+3*ldb] ); // prefetch
		d_4    = _mm256_add_pd( d_4, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4567  = _mm256_load_pd( &A[4+3*lda] ); // prefetch
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_5    = _mm256_add_pd( d_5, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
		d_7    = _mm256_add_pd( d_7, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_6    = _mm256_add_pd( d_6, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123  = _mm256_load_pd( &A[0+4*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[0+4*ldb] ); // prefetch
		d_4    = _mm256_add_pd( d_4, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4567  = _mm256_load_pd( &A[4+4*lda] ); // prefetch
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
		d_5    = _mm256_add_pd( d_5, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
		d_7    = _mm256_add_pd( d_7, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
		d_6    = _mm256_add_pd( d_6, ab_tmp0 );
		
		
		A += 4*lda;
		B += 4*ldb;

		}
	
	if(kmax%4>=2)
		{
		
		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123  = _mm256_load_pd( &A[0+1*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[0+1*ldb] ); // prefetch
		d_4    = _mm256_add_pd( d_4, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4567  = _mm256_load_pd( &A[4+1*lda] ); // prefetch
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_5    = _mm256_add_pd( d_5, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
		d_7    = _mm256_add_pd( d_7, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_6    = _mm256_add_pd( d_6, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123  = _mm256_load_pd( &A[0+2*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[0+2*ldb] ); // prefetch
		d_4    = _mm256_add_pd( d_4, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4567  = _mm256_load_pd( &A[4+2*lda] ); // prefetch
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
		d_5    = _mm256_add_pd( d_5, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
		d_7    = _mm256_add_pd( d_7, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
		d_6    = _mm256_add_pd( d_6, ab_tmp0 );
		
	
		
		A += 2*lda;
		B += 2*ldb;

		}

	if(kmax%2==1)
		{
		
		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
//			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
//			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
		d_4    = _mm256_add_pd( d_4, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
//			A_4567  = _mm256_load_pd( &A1[4] ); // prefetch
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_5    = _mm256_add_pd( d_5, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
		d_7    = _mm256_add_pd( d_7, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
		d_6    = _mm256_add_pd( d_6, ab_tmp0 );


//			A0 += 4;
//			A1 += 4;
//			B  += 4;

		}

	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;
	
	t_0 = _mm256_blend_pd( d_0, d_1, 0xa );
	t_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
	t_2 = _mm256_blend_pd( d_2, d_3, 0xa );
	t_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

	d_0 = _mm256_blend_pd( t_0, t_2, 0xc );
	d_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
	d_1 = _mm256_blend_pd( t_1, t_3, 0xc );
	d_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

	t_0 = _mm256_blend_pd( d_4, d_5, 0xa );
	t_1 = _mm256_blend_pd( d_4, d_5, 0x5 );
	t_2 = _mm256_blend_pd( d_6, d_7, 0xa );
	t_3 = _mm256_blend_pd( d_6, d_7, 0x5 );

	d_4 = _mm256_blend_pd( t_0, t_2, 0xc );
	d_6 = _mm256_blend_pd( t_0, t_2, 0x3 );
	d_5 = _mm256_blend_pd( t_1, t_3, 0xc );
	d_7 = _mm256_blend_pd( t_1, t_3, 0x3 );


	if(alg==0)
		{
		goto store;
		}
	else if(alg==1) // AB = A*B'
		{
		c_0 = _mm256_load_pd( &C[0+ldc*0] );
		c_1 = _mm256_load_pd( &C[0+ldc*1] );
		c_2 = _mm256_load_pd( &C[0+ldc*2] );
		c_3 = _mm256_load_pd( &C[0+ldc*3] );
	
		d_0 = _mm256_add_pd( c_0, d_0 );
		d_1 = _mm256_add_pd( c_1, d_1 );
		d_2 = _mm256_add_pd( c_2, d_2 );
		d_3 = _mm256_add_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C[4+ldc*0] );
		c_5 = _mm256_load_pd( &C[4+ldc*1] );
		c_6 = _mm256_load_pd( &C[4+ldc*2] );
		c_7 = _mm256_load_pd( &C[4+ldc*3] );
	
		d_4 = _mm256_add_pd( c_4, d_4 );
		d_5 = _mm256_add_pd( c_5, d_5 );
		d_6 = _mm256_add_pd( c_6, d_6 );
		d_7 = _mm256_add_pd( c_7, d_7 );
		}
	else // AB = - A*B'
		{
		c_0 = _mm256_load_pd( &C[0+ldc*0] );
		c_1 = _mm256_load_pd( &C[0+ldc*1] );
		c_2 = _mm256_load_pd( &C[0+ldc*2] );
		c_3 = _mm256_load_pd( &C[0+ldc*3] );
	
		d_0 = _mm256_sub_pd( c_0, d_0 );
		d_1 = _mm256_sub_pd( c_1, d_1 );
		d_2 = _mm256_sub_pd( c_2, d_2 );
		d_3 = _mm256_sub_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C[4+ldc*0] );
		c_5 = _mm256_load_pd( &C[4+ldc*1] );
		c_6 = _mm256_load_pd( &C[4+ldc*2] );
		c_7 = _mm256_load_pd( &C[4+ldc*3] );
	
		d_4 = _mm256_sub_pd( c_4, d_4 );
		d_5 = _mm256_sub_pd( c_5, d_5 );
		d_6 = _mm256_sub_pd( c_6, d_6 );
		d_7 = _mm256_sub_pd( c_7, d_7 );
		}

	goto store;

	store:
	_mm256_store_pd( &D[0+ldd*0], d_0 );
	_mm256_store_pd( &D[0+ldd*1], d_1 );
	_mm256_store_pd( &D[0+ldd*2], d_2 );
	_mm256_store_pd( &D[0+ldd*3], d_3 );

	_mm256_store_pd( &D[4+ldd*0], d_4 );
	_mm256_store_pd( &D[4+ldd*1], d_5 );
	_mm256_store_pd( &D[4+ldd*2], d_6 );
	_mm256_store_pd( &D[4+ldd*3], d_7 );

	return;

	}



void kernel_dgemm_nt_4x4_unpack_lib(int kmax, double *A, int lda, double *B, int ldb, double *C, int ldc, double *D, int ldd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	int k;

	__m256d
		a_0123, A_0123, 
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		temp,
		ab_tmp0, ab_tmp1, // temporary results
		d_0, d_1, d_3, d_2;
	
	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();

	// prefetch
	a_0123 = _mm256_load_pd( &A[0+0*lda] );
	b_0123 = _mm256_load_pd( &B[0+0*ldb] );

	for(k=0; k<kmax-3; k+=4)
		{
		
		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123  = _mm256_load_pd( &A[0+1*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		b_0123  = _mm256_load_pd( &B[0+1*ldb] ); // prefetch
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123  = _mm256_load_pd( &A[0+2*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		b_0123  = _mm256_load_pd( &B[0+2*ldb] ); // prefetch
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123  = _mm256_load_pd( &A[0+3*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		b_0123  = _mm256_load_pd( &B[0+3*ldb] ); // prefetch
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123  = _mm256_load_pd( &A[0+4*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		b_0123  = _mm256_load_pd( &B[0+4*ldb] ); // prefetch
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		
		
		A += 4*lda;
		B += 4*ldb;

		}
	
	if(kmax%4>=2)
		{
		
		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123  = _mm256_load_pd( &A[0+1*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		b_0123  = _mm256_load_pd( &B[0+1*ldb] ); // prefetch
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		

		ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123  = _mm256_load_pd( &A[0+2*lda] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
		b_0123  = _mm256_load_pd( &B[0+2*ldb] ); // prefetch
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );
		
		
		A += 2*lda;
		B += 2*ldb;

		}

	if(kmax%2==1)
		{
		
		ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
//			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
		d_0    = _mm256_add_pd( d_0, ab_tmp0 );
//			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		d_1    = _mm256_add_pd( d_1, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_3    = _mm256_add_pd( d_3, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
		d_2    = _mm256_add_pd( d_2, ab_tmp0 );


//			A0 += 4;
//			A1 += 4;
//			B  += 4;

		}

	__m256d
		c_0, c_1, c_2, c_3,
		t_0, t_1, t_2, t_3;
	

	t_0 = _mm256_blend_pd( d_0, d_1, 0xa );
	t_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
	t_2 = _mm256_blend_pd( d_2, d_3, 0xa );
	t_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

	d_0 = _mm256_blend_pd( t_0, t_2, 0xc );
	d_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
	d_1 = _mm256_blend_pd( t_1, t_3, 0xc );
	d_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

	if(alg==0)
		{
		goto store_n;
		}
	else if(alg==1) // AB = A*B'
		{
		c_0 = _mm256_load_pd( &C[0+ldc*0] );
		c_1 = _mm256_load_pd( &C[0+ldc*1] );
		c_2 = _mm256_load_pd( &C[0+ldc*2] );
		c_3 = _mm256_load_pd( &C[0+ldc*3] );
	
		d_0 = _mm256_add_pd( c_0, d_0 );
		d_1 = _mm256_add_pd( c_1, d_1 );
		d_2 = _mm256_add_pd( c_2, d_2 );
		d_3 = _mm256_add_pd( c_3, d_3 );
		}
	else // AB = - A*B'
		{
		c_0 = _mm256_load_pd( &C[0+ldc*0] );
		c_1 = _mm256_load_pd( &C[0+ldc*1] );
		c_2 = _mm256_load_pd( &C[0+ldc*2] );
		c_3 = _mm256_load_pd( &C[0+ldc*3] );
	
		d_0 = _mm256_sub_pd( c_0, d_0 );
		d_1 = _mm256_sub_pd( c_1, d_1 );
		d_2 = _mm256_sub_pd( c_2, d_2 );
		d_3 = _mm256_sub_pd( c_3, d_3 );
		}

		goto store_n;
	
	// store (5 - 8) x (3 - 4)
	store_n:
	_mm256_store_pd( &D[0+ldd*0], d_0 );
	_mm256_store_pd( &D[0+ldd*1], d_1 );
	_mm256_store_pd( &D[0+ldd*2], d_2 );
	_mm256_store_pd( &D[0+ldd*3], d_3 );

	return;

	}



void kernel_dgemm_nt_4x2_unpack_lib(int kmax, double *A, int lda, double *B, int ldb, double *C, int ldc, double *D, int ldd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		b_0 = B[0+ldb*1];
		b_1 = B[1+ldb*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		b_0 = B[0+ldb*2];
		b_1 = B[1+ldb*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		b_0 = B[0+ldb*3];
		b_1 = B[1+ldb*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		
		A += 4*lda;
		B += 4*ldb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 1*lda;
		B += 1*ldb;

		}
		
	double
		d_00, d_01,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		goto store;
		}
	else 
		{
		d_00 = C[0+ldc*0];
		d_10 = C[1+ldc*0];
		d_20 = C[2+ldc*0];
		d_30 = C[3+ldc*0];
		
		d_01 = C[0+ldc*1];
		d_11 = C[1+ldc*1];
		d_21 = C[2+ldc*1];
		d_31 = C[3+ldc*1];
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;
			}

		goto store;
		}
	
	store:
	D[0+ldd*0] = c_00;
	D[1+ldd*0] = c_10;
	D[2+ldd*0] = c_20;
	D[3+ldd*0] = c_30;

	D[0+ldd*1] = c_01;
	D[1+ldd*1] = c_11;
	D[2+ldd*1] = c_21;
	D[3+ldd*1] = c_31;

	return;

	}



// TODO implement tc & td
void kernel_dgemm_nn_8x4_unpack_lib(int kmax, double *A, int lda, double *B, int ldb, double *C, int ldc, double *D, int ldd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	int k;

	__m256d
		a_0123, a_4567,
		b_0,
		temp,
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	for(k=0; k<kmax; k++)
		{

		a_0123 = _mm256_load_pd( &A[0+0*lda] );
		a_4567 = _mm256_load_pd( &A[4+0*lda] );

		b_0    = _mm256_broadcast_sd( &B[0+0*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_0    = _mm256_add_pd( d_0, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_4    = _mm256_add_pd( d_4, temp );

		b_0    = _mm256_broadcast_sd( &B[0+1*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_1    = _mm256_add_pd( d_1, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_5    = _mm256_add_pd( d_5, temp );

		b_0    = _mm256_broadcast_sd( &B[0+2*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_2    = _mm256_add_pd( d_2, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_6    = _mm256_add_pd( d_6, temp );

		b_0    = _mm256_broadcast_sd( &B[0+3*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_3    = _mm256_add_pd( d_3, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_7    = _mm256_add_pd( d_7, temp );


		a_0123 = _mm256_load_pd( &A[0+1*lda] );
		a_4567 = _mm256_load_pd( &A[4+1*lda] );

		b_0    = _mm256_broadcast_sd( &B[1+0*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_0    = _mm256_add_pd( d_0, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_4    = _mm256_add_pd( d_4, temp );

		b_0    = _mm256_broadcast_sd( &B[1+1*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_1    = _mm256_add_pd( d_1, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_5    = _mm256_add_pd( d_5, temp );

		b_0    = _mm256_broadcast_sd( &B[1+2*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_2    = _mm256_add_pd( d_2, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_6    = _mm256_add_pd( d_6, temp );

		b_0    = _mm256_broadcast_sd( &B[1+3*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_3    = _mm256_add_pd( d_3, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_7    = _mm256_add_pd( d_7, temp );


		a_0123 = _mm256_load_pd( &A[0+2*lda] );
		a_4567 = _mm256_load_pd( &A[4+2*lda] );

		b_0    = _mm256_broadcast_sd( &B[2+0*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_0    = _mm256_add_pd( d_0, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_4    = _mm256_add_pd( d_4, temp );

		b_0    = _mm256_broadcast_sd( &B[2+1*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_1    = _mm256_add_pd( d_1, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_5    = _mm256_add_pd( d_5, temp );

		b_0    = _mm256_broadcast_sd( &B[2+2*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_2    = _mm256_add_pd( d_2, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_6    = _mm256_add_pd( d_6, temp );

		b_0    = _mm256_broadcast_sd( &B[2+3*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_3    = _mm256_add_pd( d_3, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_7    = _mm256_add_pd( d_7, temp );


		a_0123 = _mm256_load_pd( &A[0+3*lda] );
		a_4567 = _mm256_load_pd( &A[4+3*lda] );

		b_0    = _mm256_broadcast_sd( &B[3+0*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_0    = _mm256_add_pd( d_0, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_4    = _mm256_add_pd( d_4, temp );

		b_0    = _mm256_broadcast_sd( &B[3+1*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_1    = _mm256_add_pd( d_1, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_5    = _mm256_add_pd( d_5, temp );

		b_0    = _mm256_broadcast_sd( &B[3+2*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_2    = _mm256_add_pd( d_2, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_6    = _mm256_add_pd( d_6, temp );

		b_0    = _mm256_broadcast_sd( &B[3+3*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_3    = _mm256_add_pd( d_3, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_7    = _mm256_add_pd( d_7, temp );


		A += 4*lda;
		B += 4;

		}
	for(; k<kmax; k++)
		{

		a_0123 = _mm256_load_pd( &A[0+0*lda] );
		a_4567 = _mm256_load_pd( &A[4+0*lda] );

		b_0    = _mm256_broadcast_sd( &B[0+0*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_0    = _mm256_add_pd( d_0, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_4    = _mm256_add_pd( d_4, temp );

		b_0    = _mm256_broadcast_sd( &B[0+1*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_1    = _mm256_add_pd( d_1, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_5    = _mm256_add_pd( d_5, temp );

		b_0    = _mm256_broadcast_sd( &B[0+2*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_2    = _mm256_add_pd( d_2, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_6    = _mm256_add_pd( d_6, temp );

		b_0    = _mm256_broadcast_sd( &B[0+3*ldb] );
		temp   = _mm256_mul_pd( a_0123, b_0 );
		d_3    = _mm256_add_pd( d_3, temp );
		temp   = _mm256_mul_pd( a_4567, b_0 );
		d_7    = _mm256_add_pd( d_7, temp );


		A += 1*lda;
		B += 1;

		}
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7;
	

	if(alg==0)
		{
		goto store;
		}
	else
		{
		c_0 = _mm256_load_pd( &C[0+lda*0] );
		c_1 = _mm256_load_pd( &C[0+lda*1] );
		c_2 = _mm256_load_pd( &C[0+lda*2] );
		c_3 = _mm256_load_pd( &C[0+lda*3] );
		c_4 = _mm256_load_pd( &C[4+lda*0] );
		c_5 = _mm256_load_pd( &C[4+lda*1] );
		c_6 = _mm256_load_pd( &C[4+lda*2] );
		c_7 = _mm256_load_pd( &C[4+lda*3] );

		if(alg==1)
			{
			d_0 = _mm256_add_pd( c_0, d_0 );
			d_1 = _mm256_add_pd( c_1, d_1 );
			d_2 = _mm256_add_pd( c_2, d_2 );
			d_3 = _mm256_add_pd( c_3, d_3 );
			d_4 = _mm256_add_pd( c_4, d_4 );
			d_5 = _mm256_add_pd( c_5, d_5 );
			d_6 = _mm256_add_pd( c_6, d_6 );
			d_7 = _mm256_add_pd( c_7, d_7 );
			}
		else
			{
			d_0 = _mm256_sub_pd( c_0, d_0 );
			d_1 = _mm256_sub_pd( c_1, d_1 );
			d_2 = _mm256_sub_pd( c_2, d_2 );
			d_3 = _mm256_sub_pd( c_3, d_3 );
			d_4 = _mm256_sub_pd( c_4, d_4 );
			d_5 = _mm256_sub_pd( c_5, d_5 );
			d_6 = _mm256_sub_pd( c_6, d_6 );
			d_7 = _mm256_sub_pd( c_7, d_7 );
			}

		goto store;

		}

	store:
	_mm256_store_pd( &D[0+ldd*0], d_0 );
	_mm256_store_pd( &D[0+ldd*1], d_1 );
	_mm256_store_pd( &D[0+ldd*2], d_2 );
	_mm256_store_pd( &D[0+ldd*3], d_3 );
	_mm256_store_pd( &D[4+ldd*0], d_4 );
	_mm256_store_pd( &D[4+ldd*1], d_5 );
	_mm256_store_pd( &D[4+ldd*2], d_6 );
	_mm256_store_pd( &D[4+ldd*3], d_7 );

	return;

	}



void kernel_dpotrf_dtrsm_4_unpack_lib(int kmax, double *A, int lda)
	{

	double
		a_00,
		a_10, a_11,
		a_20, a_21, a_22,
		a_30, a_31, a_32, a_33,
		b_00, b_01, b_02, b_03;

	int k;

	if(kmax>4)
		{

		a_00 = A[0+lda*0];
		a_10 = A[1+lda*0];
		a_20 = A[2+lda*0];
		a_30 = A[3+lda*0];
		a_00 = sqrt(a_00);
		A[0+lda*0] = a_00;
		a_00 = 1.0/a_00;
		a_10 = a_10 * a_00;
		a_20 = a_20 * a_00;
		a_30 = a_30 * a_00;
		A[1+lda*0] = a_10;
		A[2+lda*0] = a_20;
		A[3+lda*0] = a_30;

		a_11 = A[1+lda*1];
		a_21 = A[2+lda*1];
		a_31 = A[3+lda*1];
		a_11 = a_11 - a_10 * a_10;
		a_21 = a_21 - a_20 * a_10;
		a_31 = a_31 - a_30 * a_10;
		a_11 = sqrt(a_11);
		A[1+lda*1] = a_11;
		a_11 = 1.0/a_11;
		a_21 = a_21 * a_11;
		a_31 = a_31 * a_11;
		A[2+lda*1] = a_21;
		A[3+lda*1] = a_31;

		a_22 = A[2+lda*2];
		a_32 = A[3+lda*2];
		a_22 = a_22 - a_20 * a_20;
		a_32 = a_32 - a_30 * a_20;
		a_22 = a_22 - a_21 * a_21;
		a_32 = a_32 - a_31 * a_21;
		a_22 = sqrt(a_22);
		A[2+lda*2] = a_22;
		a_22 = 1.0/a_22;
		a_32 = a_32 * a_22;
		A[3+lda*2] = a_32;

		a_33 = A[3+lda*3];
		a_33 = a_33 - a_30 * a_30;
		a_33 = a_33 - a_31 * a_31;
		a_33 = a_33 - a_32 * a_32;
		a_33 = sqrt(a_33);
		A[3+lda*3] = a_33;
		a_33 = 1.0/a_33;

		A += 4;
		k  = 4;

		}
	else
		{

		a_00 = A[0+lda*0];
		a_10 = A[1+lda*0];
		a_20 = A[2+lda*0];
		a_30 = A[3+lda*0];
		a_00 = sqrt(a_00);
		A[0+lda*0] = a_00;
		a_00 = 1.0/a_00;
		a_10 = a_10 * a_00;
		a_20 = a_20 * a_00;
		a_30 = a_30 * a_00;
		A[1+lda*0] = a_10;
		A[2+lda*0] = a_20;
		A[3+lda*0] = a_30;

		if(kmax>1)
			{

			a_11 = A[1+lda*1];
			a_21 = A[2+lda*1];
			a_31 = A[3+lda*1];
			a_11 = a_11 - a_10 * a_10;
			a_21 = a_21 - a_20 * a_10;
			a_31 = a_31 - a_30 * a_10;
			a_11 = sqrt(a_11);
			A[1+lda*1] = a_11;
			a_11 = 1.0/a_11;
			a_21 = a_21 * a_11;
			a_31 = a_31 * a_11;
			A[2+lda*1] = a_21;
			A[3+lda*1] = a_31;

			if(kmax>2)
				{

				a_22 = A[2+lda*2];
				a_32 = A[3+lda*2];
				a_22 = a_22 - a_20 * a_20;
				a_32 = a_32 - a_30 * a_20;
				a_22 = a_22 - a_21 * a_21;
				a_32 = a_32 - a_31 * a_21;
				a_22 = sqrt(a_22);
				A[2+lda*2] = a_22;
				a_22 = 1.0/a_22;
				a_32 = a_32 * a_22;
				A[3+lda*2] = a_32;

				if(kmax>3)
					{

					a_33 = A[3+lda*3];
					a_33 = a_33 - a_30 * a_30;
					a_33 = a_33 - a_31 * a_31;
					a_33 = a_33 - a_32 * a_32;
					a_33 = sqrt(a_33);
					A[3+lda*3] = a_33;

					}
				}
			}

		return;

		}
	
	for( ; k<kmax; k++)
		{

		b_00 = A[0+lda*0];
		b_00 *= a_00;
		A[0+lda*0] = b_00;

		b_01 = A[0+lda*1];
		b_01 = b_01 - b_00 * a_10;
		b_01 *= a_11;
		A[0+lda*1] = b_01;

		b_02 = A[0+lda*2];
		b_02 = b_02 - b_00 * a_20;
		b_02 = b_02 - b_01 * a_21;
		b_02 *= a_22;
		A[0+lda*2] = b_02;

		b_03 = A[0+lda*3];
		b_03 = b_03 - b_00 * a_30;
		b_03 = b_03 - b_01 * a_31;
		b_03 = b_03 - b_02 * a_32;
		b_03 *= a_33;
		A[0+lda*3] = b_03;

		A += 1;

		}
	
	}




