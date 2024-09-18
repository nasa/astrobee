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

// normal-transposed, 12x4 with data packed in 4
void kernel_dtrmm_nt_u_12x4_lib4(int kadd, double *A0, int sda, double *B, double *D0, int sdd)
	{
	
	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;
	
	const int ldc = 4;

	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82,
		d_00, d_01, d_03, d_02,
		d_40, d_41, d_43, d_42,
		d_80, d_81, d_83, d_82,
		a_0123, a_4567, //A_0123,
		b_0, b_1, b_2, b_3,
		b_0101, b_1010, b_2323, b_3232,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		c_40_50_60_70, c_41_51_61_71, c_42_52_62_72, c_43_53_63_73,
		c_00_11_20_31, c_01_10_21_30, c_03_12_23_32, c_02_13_22_33,
		c_40_51_60_71, c_41_50_61_70, c_43_52_63_72, c_42_53_62_73,
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71;
	
	zeros = _mm256_setzero_pd();
	
	// prefetch
	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_8 = _mm256_load_pd( &A2[0] );
	b_0 = _mm256_broadcast_sd( &B[0] );


/*	__builtin_prefetch( A+32 );*/
	d_00 = _mm256_mul_pd( a_0, b_0 );
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	d_40 = _mm256_mul_pd( a_4, b_0 );
	a_4  = _mm256_load_pd( &A1[4] ); // prefetch
	d_80 = _mm256_mul_pd( a_8, b_0 );
	a_8  = _mm256_load_pd( &A2[4] ); // prefetch
	b_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch*/
	d_01 = _mm256_blend_pd( d_00, zeros, 0x5 );
	d_00 = _mm256_blend_pd( d_00, zeros, 0xa );
	d_41 = _mm256_blend_pd( d_40, zeros, 0x5 );
	d_40 = _mm256_blend_pd( d_40, zeros, 0xa );
	d_81 = _mm256_blend_pd( d_80, zeros, 0x5 );
	d_80 = _mm256_blend_pd( d_80, zeros, 0xa );
	
	
/*	__builtin_prefetch( A+40 );*/
	d_00 = _mm256_fmadd_pd( a_0, b_0, d_00 );
	d_40 = _mm256_fmadd_pd( a_4, b_0, d_40 );
	d_80 = _mm256_fmadd_pd( a_8, b_0, d_80 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	d_01 = _mm256_fmadd_pd( a_0, b_0, d_01 );
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	d_41 = _mm256_fmadd_pd( a_4, b_0, d_41 );
	a_4  = _mm256_load_pd( &A1[8] ); // prefetch
	d_81 = _mm256_fmadd_pd( a_8, b_0, d_81 );
	b_0  = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch*/
	a_8  = _mm256_load_pd( &A2[8] ); // prefetch



/*	__builtin_prefetch( A+48 );*/
	d_00 = _mm256_fmadd_pd( a_0, b_0, d_00 );
	d_40 = _mm256_fmadd_pd( a_4, b_0, d_40 );
	d_80 = _mm256_fmadd_pd( a_8, b_0, d_80 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	d_01 = _mm256_fmadd_pd( a_0, b_0, d_01 );
	d_41 = _mm256_fmadd_pd( a_4, b_0, d_41 );
	d_81 = _mm256_fmadd_pd( a_8, b_0, d_81 );
	b_0  = _mm256_broadcast_sd( &B[10] );
	d_02 = _mm256_mul_pd( a_0, b_0 );
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	d_03 = _mm256_blend_pd( d_02, zeros, 0x5 );
	d_02 = _mm256_blend_pd( d_02, zeros, 0xa );
	d_42 = _mm256_mul_pd( a_4, b_0 );
	a_4  = _mm256_load_pd( &A1[12] ); // prefetch
	d_43 = _mm256_blend_pd( d_42, zeros, 0x5 );
	d_42 = _mm256_blend_pd( d_42, zeros, 0xa );
	d_82 = _mm256_mul_pd( a_8, b_0 );
	a_8  = _mm256_load_pd( &A2[12] ); // prefetch
	d_83 = _mm256_blend_pd( d_82, zeros, 0x5 );
	d_82 = _mm256_blend_pd( d_82, zeros, 0xa );
	b_0  = _mm256_load_pd( &B[12] ); // prefetch*/

	c_00 = _mm256_blend_pd( d_00, d_02, 0xc );
	c_02 = _mm256_blend_pd( d_00, d_02, 0x3 );
	c_01 = _mm256_blend_pd( d_01, d_03, 0xc );
	c_03 = _mm256_blend_pd( d_01, d_03, 0x3 );
	c_40 = _mm256_blend_pd( d_40, d_42, 0xc );
	c_42 = _mm256_blend_pd( d_40, d_42, 0x3 );
	c_41 = _mm256_blend_pd( d_41, d_43, 0xc );
	c_43 = _mm256_blend_pd( d_41, d_43, 0x3 );
	c_80 = _mm256_blend_pd( d_80, d_82, 0xc );
	c_82 = _mm256_blend_pd( d_80, d_82, 0x3 );
	c_81 = _mm256_blend_pd( d_81, d_83, 0xc );
	c_83 = _mm256_blend_pd( d_81, d_83, 0x3 );

/*	__builtin_prefetch( A+56 );*/
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[16] ); // prefetch
	a_8  = _mm256_load_pd( &A2[16] ); // prefetch

	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;

	for(k=4; k<kadd-3; k+=4)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[4] ); // prefetch
		a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
		
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[8] ); // prefetch
		a_8  = _mm256_load_pd( &A2[8] ); // prefetch



		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[12] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[12] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[12] ); // prefetch
		a_8  = _mm256_load_pd( &A2[12] ); // prefetch


		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[16] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[16] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[16] ); // prefetch
		a_8  = _mm256_load_pd( &A2[16] ); // prefetch
		
		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}
	
	if(kadd%4>=2)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[4] );
		a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
		
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[8] );
		a_8  = _mm256_load_pd( &A2[8] ); // prefetch
		
		
		A0 += 8;
		A1 += 8;
		A2 += 8;
		B  += 8;

		}

	if(kadd%2==1)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
/*		a_0  = _mm256_load_pd( &A0[4] ); // prefetch*/
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
/*		a_4  = _mm256_load_pd( &A1[4] ); // prefetch*/
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
//		b_0  = _mm256_load_pd( &B[4] );
/*		a_8  = _mm256_load_pd( &A2[4] ); // prefetch*/
		
		}

	__m256d
		e_00, e_01, e_02, e_03,
		e_40, e_41, e_42, e_43,
		e_80, e_81, e_82, e_83;

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );

	c_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	c_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	c_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	c_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	_mm256_store_pd( &D0[0+ldc*0], c_00 );
	_mm256_store_pd( &D0[0+ldc*1], c_01 );
	_mm256_store_pd( &D0[0+ldc*2], c_02 );
	_mm256_store_pd( &D0[0+ldc*3], c_03 );

	e_40 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_41 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_42 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_43 = _mm256_blend_pd( c_42, c_43, 0x5 );

	c_40 = _mm256_blend_pd( e_40, e_42, 0xc );
	c_42 = _mm256_blend_pd( e_40, e_42, 0x3 );
	c_41 = _mm256_blend_pd( e_41, e_43, 0xc );
	c_43 = _mm256_blend_pd( e_41, e_43, 0x3 );

	_mm256_store_pd( &D1[0+ldc*0], c_40 );
	_mm256_store_pd( &D1[0+ldc*1], c_41 );
	_mm256_store_pd( &D1[0+ldc*2], c_42 );
	_mm256_store_pd( &D1[0+ldc*3], c_43 );

	e_80 = _mm256_blend_pd( c_80, c_81, 0xa );
	e_81 = _mm256_blend_pd( c_80, c_81, 0x5 );
	e_82 = _mm256_blend_pd( c_82, c_83, 0xa );
	e_83 = _mm256_blend_pd( c_82, c_83, 0x5 );
	
	c_80 = _mm256_blend_pd( e_80, e_82, 0xc );
	c_82 = _mm256_blend_pd( e_80, e_82, 0x3 );
	c_81 = _mm256_blend_pd( e_81, e_83, 0xc );
	c_83 = _mm256_blend_pd( e_81, e_83, 0x3 );

	_mm256_store_pd( &D2[0+ldc*0], c_80 );
	_mm256_store_pd( &D2[0+ldc*1], c_81 );
	_mm256_store_pd( &D2[0+ldc*2], c_82 );
	_mm256_store_pd( &D2[0+ldc*3], c_83 );

	}



// normal-transposed, 8x4 with data packed in 4
void kernel_dtrmm_nt_u_8x4_lib4(int kadd, double *A0, int sda, double *B, double *D0, int sdd)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdd;
	
	const int ldc = 4;

	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, //A_0123,
		b_0, b_1, b_2, b_3,
		b_0101, b_1010, b_2323, b_3232,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		c_40_50_60_70, c_41_51_61_71, c_42_52_62_72, c_43_53_63_73,
		c_00_11_20_31, c_01_10_21_30, c_03_12_23_32, c_02_13_22_33,
		c_40_51_60_71, c_41_50_61_70, c_43_52_63_72, c_42_53_62_73,
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71;
	
	zeros = _mm256_setzero_pd();
	
	// prefetch
	a_0123        = _mm256_load_pd( &A0[0] );
	a_4567        = _mm256_load_pd( &A1[0] );
	b_0           = _mm256_broadcast_sd( &B[0] );


/*	__builtin_prefetch( A+32 );*/
	c_00_10_20_30 = _mm256_mul_pd( a_0123, b_0 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	c_00_11_20_31 = _mm256_blend_pd( c_00_10_20_30, zeros, 0xa );
	c_01_10_21_30 = _mm256_blend_pd( c_00_10_20_30, zeros, 0x5 );
	c_40_50_60_70 = _mm256_mul_pd( a_4567, b_0 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch*/
	c_40_51_60_71 = _mm256_blend_pd( c_40_50_60_70, zeros, 0xa );
	c_41_50_61_70 = _mm256_blend_pd( c_40_50_60_70, zeros, 0x5 );
	
	
/*	__builtin_prefetch( A+40 );*/
	c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	c_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_2           = _mm256_broadcast_sd( &B[10] );
	c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	c_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch*/
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch


/*	__builtin_prefetch( A+48 );*/
	c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	c_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	c_02_12_22_32 = _mm256_mul_pd( a_0123, b_2 );
	c_02_13_22_33 = _mm256_blend_pd( c_02_12_22_32, zeros, 0xa );
	c_03_12_23_32 = _mm256_blend_pd( c_02_12_22_32, zeros, 0x5 );
	c_42_52_62_72 = _mm256_mul_pd( a_4567, b_2 );
	c_42_53_62_73 = _mm256_blend_pd( c_42_52_62_72, zeros, 0xa );
	c_43_52_63_72 = _mm256_blend_pd( c_42_52_62_72, zeros, 0x5 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch*/
	c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	c_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	c_00_11_22_33 = _mm256_blend_pd( c_00_11_20_31, c_02_13_22_33, 0xc );
	c_02_13_20_31 = _mm256_blend_pd( c_00_11_20_31, c_02_13_22_33, 0x3 );
	c_01_10_23_32 = _mm256_blend_pd( c_01_10_21_30, c_03_12_23_32, 0xc );
	c_03_12_21_30 = _mm256_blend_pd( c_01_10_21_30, c_03_12_23_32, 0x3 );
	c_40_51_62_73 = _mm256_blend_pd( c_40_51_60_71, c_42_53_62_73, 0xc );
	c_42_53_60_71 = _mm256_blend_pd( c_40_51_60_71, c_42_53_62_73, 0x3 );
	c_41_50_63_72 = _mm256_blend_pd( c_41_50_61_70, c_43_52_63_72, 0xc );
	c_43_52_61_70 = _mm256_blend_pd( c_41_50_61_70, c_43_52_63_72, 0x3 );

/*	__builtin_prefetch( A+56 );*/
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch

	A0 += 16;
	A1 += 16;
	B  += 16;

	for(k=4; k<kadd-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
/*	__builtin_prefetch( A+40 );*/
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch


/*	__builtin_prefetch( A+48 );*/
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch


/*	__builtin_prefetch( A+56 );*/
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	if(kadd%4>=2)
		{
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kadd%2==1)
		{
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
/*		b_0123        = _mm256_load_pd( &B[4] ); // prefetch*/
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
/*		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch*/
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
/*		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch*/
		
		}

	__m256d
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_40_50_62_72, c_41_51_63_73, c_42_52_60_70, c_43_53_61_71;
	
	c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	c_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
	c_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	_mm256_store_pd( &D0[0+ldc*2], c_02_12_22_32 );

	c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
	c_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	_mm256_store_pd( &D0[0+ldc*1], c_01_11_21_31 );
	c_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );
	_mm256_store_pd( &D0[0+ldc*3], c_03_13_23_33 );

	c_40_50_62_72 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
	c_42_52_60_70 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
	c_40_50_60_70 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0xc );
	_mm256_store_pd( &D1[0+ldc*0], c_40_50_60_70 );
	c_42_52_62_72 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0x3 );
	_mm256_store_pd( &D1[0+ldc*2], c_42_52_62_72 );

	c_41_51_63_73 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
	c_43_53_61_71 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
	c_41_51_61_71 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0xc );
	_mm256_store_pd( &D1[0+ldc*1], c_41_51_61_71 );
	c_43_53_63_73 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0x3 );
	_mm256_store_pd( &D1[0+ldc*3], c_43_53_63_73 );

	}



// normal-transposed, 4x4 with data packed in 4
void kernel_dtrmm_nt_u_4x4_lib4(int kadd, double *A, double *B, double *D)
	{
	
	const int ldc = 4;

	int k;
	
	__m256d
		zeros,
		a_0, A_0,
		b_0, B_0, b_1, b_2, B_2, b_3,
		c_00, c_01, c_03, c_02,
		C_00, C_01, C_03, C_02;
	
	zeros = _mm256_setzero_pd();
	
	// prefetch
	b_0 = _mm256_broadcast_sd( &B[0] );
	a_0 = _mm256_load_pd( &A[0] );


/*	__builtin_prefetch( A+32 );*/
	B_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch*/
	A_0  = _mm256_load_pd( &A[4] ); // prefetch
	c_00 = _mm256_mul_pd( a_0, b_0 );
	c_01 = _mm256_blend_pd( c_00, zeros, 0x5 );
	c_00 = _mm256_blend_pd( c_00, zeros, 0xa );
	
	
/*	__builtin_prefetch( A+40 );*/
	b_0  = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch*/
	a_0  = _mm256_load_pd( &A[8] ); // prefetch
	b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
	C_00 = _mm256_mul_pd( A_0, B_0 );
	C_01 = _mm256_mul_pd( A_0, b_1 );
	b_2  = _mm256_broadcast_sd( &B[10] );


/*	__builtin_prefetch( A+48 );*/
	B_0  = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
	A_0  = _mm256_load_pd( &A[12] ); // prefetch
	b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
	c_02 = _mm256_mul_pd( a_0, b_2 );
	c_03 = _mm256_blend_pd( c_02, zeros, 0x5 );
	c_02 = _mm256_blend_pd( c_02, zeros, 0xa );
	B_2  = _mm256_broadcast_pd( (__m128d *) &B[14] ); // prefetch

/*	__builtin_prefetch( A+56 );*/
	b_0  = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
	a_0  = _mm256_load_pd( &A[16] ); // prefetch
	b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
	C_00 = _mm256_fmadd_pd( A_0, B_0, C_00 );
	C_01 = _mm256_fmadd_pd( A_0, b_1, C_01 );
	b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
	C_02 = _mm256_mul_pd( A_0, B_2 );
	C_03 = _mm256_mul_pd( A_0, b_3 );
	b_2  = _mm256_broadcast_pd( (__m128d *) &B[18] ); // prefetch

	A += 16;
	B += 16;

	for(k=4; k<kadd-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		B_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		A_0  = _mm256_load_pd( &A[4] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
		B_2  = _mm256_broadcast_pd( (__m128d *) &B[6] ); // prefetch
		
		
/*	__builtin_prefetch( A+40 );*/
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		a_0  = _mm256_load_pd( &A[8] ); // prefetch
		b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
		C_00 = _mm256_fmadd_pd( A_0, B_0, C_00 );
		C_01 = _mm256_fmadd_pd( A_0, b_1, C_01 );
		b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
		C_02 = _mm256_fmadd_pd( A_0, B_2, C_02 );
		C_03 = _mm256_fmadd_pd( A_0, b_3, C_03 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[10] ); // prefetch


/*	__builtin_prefetch( A+48 );*/
		B_0  = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		A_0  = _mm256_load_pd( &A[12] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
		B_2  = _mm256_broadcast_pd( (__m128d *) &B[14] ); // prefetch


/*	__builtin_prefetch( A+56 );*/
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		a_0  = _mm256_load_pd( &A[16] ); // prefetch
		b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
		C_00 = _mm256_fmadd_pd( A_0, B_0, C_00 );
		C_01 = _mm256_fmadd_pd( A_0, b_1, C_01 );
		b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
		C_02 = _mm256_fmadd_pd( A_0, B_2, C_02 );
		C_03 = _mm256_fmadd_pd( A_0, b_3, C_03 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[18] ); // prefetch
		
		A += 16;
		B += 16;

		}
	
	if(kadd%4>=2)
		{
		
		B_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		A_0  = _mm256_load_pd( &A[4] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
		B_2  = _mm256_broadcast_pd( (__m128d *) &B[6] ); // prefetch
		
		
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		a_0  = _mm256_load_pd( &A[8] ); // prefetch
		b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
		C_00 = _mm256_fmadd_pd( A_0, B_0, C_00 );
		C_01 = _mm256_fmadd_pd( A_0, b_1, C_01 );
		b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
		C_02 = _mm256_fmadd_pd( A_0, B_2, C_02 );
		C_03 = _mm256_fmadd_pd( A_0, b_3, C_03 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[10] ); // prefetch

	
		
		A += 8;
		B += 8;

		}

	c_00 = _mm256_add_pd( c_00, C_00 );
	c_01 = _mm256_add_pd( c_01, C_01 );
	c_03 = _mm256_add_pd( c_03, C_03 );
	c_02 = _mm256_add_pd( c_02, C_02 );

	if(kadd%2==1)
		{
		
//		B_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
//		A_0  = _mm256_load_pd( &A[4] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
//		B_2  = _mm256_broadcast_pd( (__m128d *) &B[6] ); // prefetch
		
		}

	__m256d
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33;
	
	c_00_10_20_30 = _mm256_blend_pd( c_00, c_01, 0xa );
	c_01_11_21_31 = _mm256_blend_pd( c_00, c_01, 0x5 );
	c_02_12_22_32 = _mm256_blend_pd( c_02, c_03, 0xa );
	c_03_13_23_33 = _mm256_blend_pd( c_02, c_03, 0x5 );

	_mm256_store_pd( &D[0+ldc*0], c_00_10_20_30 );
	_mm256_store_pd( &D[0+ldc*1], c_01_11_21_31 );
	_mm256_store_pd( &D[0+ldc*2], c_02_12_22_32 );
	_mm256_store_pd( &D[0+ldc*3], c_03_13_23_33 );

	}



// normal-transposed, 4x4 with data packed in 4
#if 0
void kernel_dtrmm_nt_u_4x4_lib4_old(int kadd, double *A, double *B, double *D)
	{
	
/*	if(kmax<=0)*/
/*		return;*/
	
	const int ldc = 4;

	int k;
	
	__m256d
		a_0123, //A_0123,
		b_0, b_1, b_2, b_3,
		ab_temp, // temporary results
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33;
	
	// prefetch
	a_0123        = _mm256_load_pd( &A[0] );
	b_0           = _mm256_broadcast_sd( &B[0] );
	b_1           = _mm256_broadcast_sd( &B[5] );
	b_2           = _mm256_broadcast_sd( &B[10] );
	b_3           = _mm256_broadcast_sd( &B[15] );

	// zero registers
	c_00_10_20_30 = _mm256_mul_pd( a_0123, b_0 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	b_0           = _mm256_broadcast_sd( &B[4] ); // prefetch
	
	
/*	__builtin_prefetch( A+40 );*/
	ab_temp       = _mm256_mul_pd( a_0123, b_0 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
	b_0           = _mm256_broadcast_sd( &B[8] ); // prefetch
	c_01_11_21_31 = _mm256_mul_pd( a_0123, b_1 );
	a_0123        = _mm256_load_pd( &A[8] ); // prefetch
	b_1           = _mm256_broadcast_sd( &B[9] );


/*	__builtin_prefetch( A+48 );*/
	ab_temp       = _mm256_mul_pd( a_0123, b_0 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
	b_0           = _mm256_broadcast_sd( &B[12] ); // prefetch
	ab_temp       = _mm256_mul_pd( a_0123, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
	b_1           = _mm256_broadcast_sd( &B[13] );
	c_02_12_22_32 = _mm256_mul_pd( a_0123, b_2 );
	a_0123        = _mm256_load_pd( &A[12] ); // prefetch
	b_2           = _mm256_broadcast_sd( &B[14] );


/*	__builtin_prefetch( A+56 );*/
	ab_temp       = _mm256_mul_pd( a_0123, b_0 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
	b_0           = _mm256_broadcast_sd( &B[16] ); // prefetch
	ab_temp       = _mm256_mul_pd( a_0123, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
	b_1           = _mm256_broadcast_sd( &B[17] );
	ab_temp       = _mm256_mul_pd( a_0123, b_2 );
	c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
	b_2           = _mm256_broadcast_sd( &B[18] );
	c_03_13_23_33 = _mm256_mul_pd( a_0123, b_3 );
	a_0123        = _mm256_load_pd( &A[16] ); // prefetch
	b_3           = _mm256_broadcast_sd( &B[19] );
	
	A += 16;
	B += 16;

	for(k=4; k<kadd-3; k+=4)
		{
		
	/*	__builtin_prefetch( A+32 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0 );
		c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
		b_0           = _mm256_broadcast_sd( &B[4] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1 );
		c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
		b_1           = _mm256_broadcast_sd( &B[5] );
		ab_temp       = _mm256_mul_pd( a_0123, b_2 );
		c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
		b_2           = _mm256_broadcast_sd( &B[6] );
		ab_temp       = _mm256_mul_pd( a_0123, b_3 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
		b_3           = _mm256_broadcast_sd( &B[7] );
	
	
	/*	__builtin_prefetch( A+40 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0 );
		c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
		b_0           = _mm256_broadcast_sd( &B[8] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1 );
		c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
		b_1           = _mm256_broadcast_sd( &B[9] );
		ab_temp       = _mm256_mul_pd( a_0123, b_2 );
		c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
		b_2           = _mm256_broadcast_sd( &B[10] );
		ab_temp       = _mm256_mul_pd( a_0123, b_3 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
		b_3           = _mm256_broadcast_sd( &B[11] );


	/*	__builtin_prefetch( A+48 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0 );
		c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
		b_0           = _mm256_broadcast_sd( &B[12] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1 );
		c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
		b_1           = _mm256_broadcast_sd( &B[13] );
		ab_temp       = _mm256_mul_pd( a_0123, b_2 );
		c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
		b_2           = _mm256_broadcast_sd( &B[14] );
		ab_temp       = _mm256_mul_pd( a_0123, b_3 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
		b_3           = _mm256_broadcast_sd( &B[15] );


	/*	__builtin_prefetch( A+56 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0 );
		c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
		b_0           = _mm256_broadcast_sd( &B[16] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1 );
		c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
		b_1           = _mm256_broadcast_sd( &B[17] );
		ab_temp       = _mm256_mul_pd( a_0123, b_2 );
		c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
		b_2           = _mm256_broadcast_sd( &B[18] );
		ab_temp       = _mm256_mul_pd( a_0123, b_3 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
		b_3           = _mm256_broadcast_sd( &B[19] );
		
		A += 16;
		B += 16;

		}
	
	if(kadd%4>=2)
		{
		
	/*	__builtin_prefetch( A+32 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0 );
		c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
		b_0           = _mm256_broadcast_sd( &B[4] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1 );
		c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
		b_1           = _mm256_broadcast_sd( &B[5] );
		ab_temp       = _mm256_mul_pd( a_0123, b_2 );
		c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
		b_2           = _mm256_broadcast_sd( &B[6] );
		ab_temp       = _mm256_mul_pd( a_0123, b_3 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
		b_3           = _mm256_broadcast_sd( &B[7] );
	
	
	/*	__builtin_prefetch( A+40 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0 );
		c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
		b_0           = _mm256_broadcast_sd( &B[8] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1 );
		c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
		b_1           = _mm256_broadcast_sd( &B[9] );
		ab_temp       = _mm256_mul_pd( a_0123, b_2 );
		c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
		b_2           = _mm256_broadcast_sd( &B[10] );
		ab_temp       = _mm256_mul_pd( a_0123, b_3 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
		b_3           = _mm256_broadcast_sd( &B[11] );
		
		A += 8;
		B += 8;

		}

	if(kadd%2==1)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0 );
		c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1 );
		c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2 );
		c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
		
		}

	_mm256_store_pd( &D[0+ldc*0], c_00_10_20_30 );
	_mm256_store_pd( &D[0+ldc*2], c_02_12_22_32 );
	_mm256_store_pd( &D[0+ldc*1], c_01_11_21_31 );
	_mm256_store_pd( &D[0+ldc*3], c_03_13_23_33 );

	}
#endif



/*inline void corner_dtrmm_pp_nt_8x3_lib4(double *A0, double *A1, double *B, double *C0, double *C1, int ldc)*/
void corner_dtrmm_nt_u_12x3_lib4(double *A0, int sda, double *B, double *C0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	
	const int ldc = 4;

	__m256d
		a_00, a_01, a_02, 
		a_40, a_41, a_42,
		a_80, a_81, a_82,
		b_0, b_1, b_2,
		c_00, c_01, c_02, 
		c_40, c_41, c_42,
		c_80, c_81, c_82;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A0[0+4*0] );
	a_40 = _mm256_load_pd( &A1[0+4*0] );
	a_80 = _mm256_load_pd( &A2[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );
	c_40 = _mm256_mul_pd( a_40, b_0 );
	c_80 = _mm256_mul_pd( a_80, b_0 );

	b_1 = _mm256_broadcast_sd( &B[0+4*1] );
	a_01 = _mm256_load_pd( &A0[0+4*1] );
	a_41 = _mm256_load_pd( &A1[0+4*1] );
	a_81 = _mm256_load_pd( &A2[0+4*1] );

	c_00 = _mm256_fmadd_pd( a_01, b_1, c_00 );
	c_40 = _mm256_fmadd_pd( a_41, b_1, c_40 );
	c_80 = _mm256_fmadd_pd( a_81, b_1, c_80 );

	b_2 = _mm256_broadcast_sd( &B[0+4*2] );
	a_02 = _mm256_load_pd( &A0[0+4*2] );
	a_42 = _mm256_load_pd( &A1[0+4*2] );
	a_82 = _mm256_load_pd( &A2[0+4*2] );

	c_00 = _mm256_fmadd_pd( a_02, b_2, c_00 );
	c_40 = _mm256_fmadd_pd( a_42, b_2, c_40 );
	c_80 = _mm256_fmadd_pd( a_82, b_2, c_80 );

	_mm256_store_pd( &C0[0+ldc*0], c_00 );
	_mm256_store_pd( &C1[0+ldc*0], c_40 );
	_mm256_store_pd( &C2[0+ldc*0], c_80 );
	
	// second column 
	b_1 = _mm256_broadcast_sd( &B[1+4*1] );

	c_01 = _mm256_mul_pd( a_01, b_1 );
	c_41 = _mm256_mul_pd( a_41, b_1 );
	c_81 = _mm256_mul_pd( a_81, b_1 );

	b_2 = _mm256_broadcast_sd( &B[1+4*2] );

	c_01 = _mm256_fmadd_pd( a_02, b_2, c_01 );
	c_41 = _mm256_fmadd_pd( a_42, b_2, c_41 );
	c_81 = _mm256_fmadd_pd( a_82, b_2, c_81 );
	
	_mm256_store_pd( &C0[0+ldc*1], c_01 );
	_mm256_store_pd( &C1[0+ldc*1], c_41 );
	_mm256_store_pd( &C2[0+ldc*1], c_81 );
	
	// third column 
	b_2 = _mm256_broadcast_sd( &B[2+4*2] );

	c_02 = _mm256_mul_pd( a_02, b_2 );
	c_42 = _mm256_mul_pd( a_42, b_2 );
	c_82 = _mm256_mul_pd( a_82, b_2 );

	_mm256_store_pd( &C0[0+ldc*2], c_02 );
	_mm256_store_pd( &C1[0+ldc*2], c_42 );
	_mm256_store_pd( &C2[0+ldc*2], c_82 );

	}
	


/*inline void corner_dtrmm_pp_nt_8x2_lib4(double *A0, double *A1, double *B, double *C0, double *C1, int ldc)*/
void corner_dtrmm_nt_u_12x2_lib4(double *A0, int sda, double *B, double *C0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	
	const int ldc = 4;

	__m256d
		a_00, a_01, 
		a_40, a_41,
		a_80, a_81,
		b_0, b_1,
		c_00, c_01, 
		c_40, c_41,
		c_80, c_81;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A0[0+4*0] );
	a_40 = _mm256_load_pd( &A1[0+4*0] );
	a_80 = _mm256_load_pd( &A2[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );
	c_40 = _mm256_mul_pd( a_40, b_0 );
	c_80 = _mm256_mul_pd( a_80, b_0 );

	b_1 = _mm256_broadcast_sd( &B[0+4*1] );
	a_01 = _mm256_load_pd( &A0[0+4*1] );
	a_41 = _mm256_load_pd( &A1[0+4*1] );
	a_81 = _mm256_load_pd( &A2[0+4*1] );

	c_00 = _mm256_fmadd_pd( a_01, b_1, c_00 );
	c_40 = _mm256_fmadd_pd( a_41, b_1, c_40 );
	c_80 = _mm256_fmadd_pd( a_81, b_1, c_80 );
	
	_mm256_store_pd( &C0[0+ldc*0], c_00 );
	_mm256_store_pd( &C1[0+ldc*0], c_40 );
	_mm256_store_pd( &C2[0+ldc*0], c_80 );

	// second column 
	b_1 = _mm256_broadcast_sd( &B[1+4*1] );

	c_01 = _mm256_mul_pd( a_01, b_1 );
	c_41 = _mm256_mul_pd( a_41, b_1 );
	c_81 = _mm256_mul_pd( a_81, b_1 );
	
	_mm256_store_pd( &C0[0+ldc*1], c_01 );
	_mm256_store_pd( &C1[0+ldc*1], c_41 );
	_mm256_store_pd( &C2[0+ldc*1], c_81 );
	
	}



/*inline void corner_dtrmm_pp_nt_8x1_lib4(double *A0, double *A1, double *B, double *C0, double *C1, int ldc)*/
void corner_dtrmm_nt_u_12x1_lib4(double *A0, int sda, double *B, double *C0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	
	const int ldc = 4;

	__m256d
		a_00, 
		a_40,
		a_80,
		b_0,
		c_00, 
		c_40,
		c_80;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A0[0+4*0] );
	a_40 = _mm256_load_pd( &A1[0+4*0] );
	a_80 = _mm256_load_pd( &A2[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );
	c_40 = _mm256_mul_pd( a_40, b_0 );
	c_80 = _mm256_mul_pd( a_80, b_0 );

	_mm256_store_pd( &C0[0+ldc*0], c_00 );
	_mm256_store_pd( &C1[0+ldc*0], c_40 );
	_mm256_store_pd( &C2[0+ldc*0], c_80 );
	
	}


/*inline void corner_dtrmm_pp_nt_8x3_lib4(double *A0, double *A1, double *B, double *C0, double *C1, int ldc)*/
void corner_dtrmm_nt_u_8x3_lib4(double *A0, int sda, double *B, double *C0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int ldc = 4;

	__m256d
		a_00, a_01, a_02, 
		a_40, a_41, a_42,
		b_0, b_1, b_2,
		c_00, c_01, c_02, 
		c_40, c_41, c_42;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A0[0+4*0] );
	a_40 = _mm256_load_pd( &A1[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );
	c_40 = _mm256_mul_pd( a_40, b_0 );

	b_1 = _mm256_broadcast_sd( &B[0+4*1] );
	a_01 = _mm256_load_pd( &A0[0+4*1] );
	a_41 = _mm256_load_pd( &A1[0+4*1] );

	c_00 = _mm256_fmadd_pd( a_01, b_1, c_00 );
	c_40 = _mm256_fmadd_pd( a_41, b_1, c_40 );

	b_2 = _mm256_broadcast_sd( &B[0+4*2] );
	a_02 = _mm256_load_pd( &A0[0+4*2] );
	a_42 = _mm256_load_pd( &A1[0+4*2] );

	c_00 = _mm256_fmadd_pd( a_02, b_2, c_00 );
	c_40 = _mm256_fmadd_pd( a_42, b_2, c_40 );

	_mm256_store_pd( &C0[0+ldc*0], c_00 );
	_mm256_store_pd( &C1[0+ldc*0], c_40 );
	
	// second column 
	b_1 = _mm256_broadcast_sd( &B[1+4*1] );

	c_01 = _mm256_mul_pd( a_01, b_1 );
	c_41 = _mm256_mul_pd( a_41, b_1 );

	b_2 = _mm256_broadcast_sd( &B[1+4*2] );

	c_01 = _mm256_fmadd_pd( a_02, b_2, c_01 );
	c_41 = _mm256_fmadd_pd( a_42, b_2, c_41 );
	
	_mm256_store_pd( &C0[0+ldc*1], c_01 );
	_mm256_store_pd( &C1[0+ldc*1], c_41 );
	
	// third column 
	b_2 = _mm256_broadcast_sd( &B[2+4*2] );

	c_02 = _mm256_mul_pd( a_02, b_2 );
	c_42 = _mm256_mul_pd( a_42, b_2 );

	_mm256_store_pd( &C0[0+ldc*2], c_02 );
	_mm256_store_pd( &C1[0+ldc*2], c_42 );

	}
	


/*inline void corner_dtrmm_pp_nt_8x2_lib4(double *A0, double *A1, double *B, double *C0, double *C1, int ldc)*/
void corner_dtrmm_nt_u_8x2_lib4(double *A0, int sda, double *B, double *C0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int ldc = 4;

	__m256d
		a_00, a_01, 
		a_40, a_41,
		b_0, b_1,
		c_00, c_01, 
		c_40, c_41;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A0[0+4*0] );
	a_40 = _mm256_load_pd( &A1[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );
	c_40 = _mm256_mul_pd( a_40, b_0 );

	b_1 = _mm256_broadcast_sd( &B[0+4*1] );
	a_01 = _mm256_load_pd( &A0[0+4*1] );
	a_41 = _mm256_load_pd( &A1[0+4*1] );

	c_00 = _mm256_fmadd_pd( a_01, b_1, c_00 );
	c_40 = _mm256_fmadd_pd( a_41, b_1, c_40 );
	
	_mm256_store_pd( &C0[0+ldc*0], c_00 );
	_mm256_store_pd( &C1[0+ldc*0], c_40 );

	// second column 
	b_1 = _mm256_broadcast_sd( &B[1+4*1] );

	c_01 = _mm256_mul_pd( a_01, b_1 );
	c_41 = _mm256_mul_pd( a_41, b_1 );
	
	_mm256_store_pd( &C0[0+ldc*1], c_01 );
	_mm256_store_pd( &C1[0+ldc*1], c_41 );
	
	}



/*inline void corner_dtrmm_pp_nt_8x1_lib4(double *A0, double *A1, double *B, double *C0, double *C1, int ldc)*/
void corner_dtrmm_nt_u_8x1_lib4(double *A0, int sda, double *B, double *C0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int ldc = 4;

	__m256d
		a_00, 
		a_40,
		b_0,
		c_00, 
		c_40;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A0[0+4*0] );
	a_40 = _mm256_load_pd( &A1[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );
	c_40 = _mm256_mul_pd( a_40, b_0 );

	_mm256_store_pd( &C0[0+ldc*0], c_00 );
	_mm256_store_pd( &C1[0+ldc*0], c_40 );
	
	}


/*inline void corner_dtrmm_pp_nt_4x3_lib4(double *A, double *B, double *C, int ldc)*/
void corner_dtrmm_nt_u_4x3_lib4(double *A, double *B, double *C)
	{
	
	const int ldc = 4;

	__m256d
		a_00, a_01, a_02,
		b_0, b_1, b_2,
		c_00, c_01, c_02;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );

	b_1 = _mm256_broadcast_sd( &B[0+4*1] );
	a_01 = _mm256_load_pd( &A[0+4*1] );

	c_00 = _mm256_fmadd_pd( a_01, b_1, c_00 );

	b_2 = _mm256_broadcast_sd( &B[0+4*2] );
	a_02 = _mm256_load_pd( &A[0+4*2] );

	c_00 = _mm256_fmadd_pd( a_02, b_2, c_00 );

	_mm256_store_pd( &C[0+ldc*0], c_00 );
	
	// second column 
	b_1 = _mm256_broadcast_sd( &B[1+4*1] );

	c_01 = _mm256_mul_pd( a_01, b_1 );

	b_2 = _mm256_broadcast_sd( &B[1+4*2] );

	c_01 = _mm256_fmadd_pd( a_02, b_2, c_01 );
	
	_mm256_store_pd( &C[0+ldc*1], c_01 );
	
	// third column 
	b_2 = _mm256_broadcast_sd( &B[2+4*2] );

	c_02 = _mm256_mul_pd( a_02, b_2 );

	_mm256_store_pd( &C[0+ldc*2], c_02 );

	}
	


/*inline void corner_dtrmm_pp_nt_4x2_lib4(double *A, double *B, double *C, int ldc)*/
void corner_dtrmm_nt_u_4x2_lib4(double *A, double *B, double *C)
	{
	
	const int ldc = 4;

	__m256d
		a_00, a_01,
		b_0, b_1,
		c_00, c_01;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );

	b_1 = _mm256_broadcast_sd( &B[0+4*1] );
	a_01 = _mm256_load_pd( &A[0+4*1] );

	c_00 = _mm256_fmadd_pd( a_01, b_1, c_00 );
	
	_mm256_store_pd( &C[0+ldc*0], c_00 );

	// second column 
	b_1 = _mm256_broadcast_sd( &B[1+4*1] );

	c_01 = _mm256_mul_pd( a_01, b_1 );
	
	_mm256_store_pd( &C[0+ldc*1], c_01 );
	
	}



/*inline void corner_dtrmm_pp_nt_4x1_lib4(double *A, double *B, double *C, int ldc)*/
void corner_dtrmm_nt_u_4x1_lib4(double *A, double *B, double *C)
	{
	
	const int ldc = 4;

	__m256d
		a_00,
		b_0,
		c_00;
	
	// first column 
	b_0 = _mm256_broadcast_sd( &B[0+4*0] );
	a_00 = _mm256_load_pd( &A[0+4*0] );
	
	c_00 = _mm256_mul_pd( a_00, b_0 );

	_mm256_store_pd( &C[0+ldc*0], c_00 );
	
	}



// normal-transposed, 8x4 with data packed in 4
void kernel_dtrmm_nt_l_8x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567, //A_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();
	c_40_51_62_73 = _mm256_setzero_pd();
	c_41_50_63_72 = _mm256_setzero_pd();
	c_43_52_61_70 = _mm256_setzero_pd();
	c_42_53_60_71 = _mm256_setzero_pd();


	for(k=0; k<kmax-4; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp1 );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp1 );


/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp1 );


/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp1 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	// final triangle

	__m256d
		b_1, b_2, b_3,
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_40_50_62_72, c_41_51_63_73, c_42_52_60_70, c_43_53_61_71,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		c_40_50_60_70, c_41_51_61_71, c_42_52_62_72, c_43_53_63_73,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33,
		d_40_50_60_70, d_41_51_61_71, d_42_52_62_72, d_43_53_63_73;

	// k=kmax-3
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
	//b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
	c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp1 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
	c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp1 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
	c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
	c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp1 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
	c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp1 );
	
	c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
	c_40_50_62_72 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
	c_41_51_63_73 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
	c_42_52_60_70 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
	c_43_53_61_71 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
	
	c_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	c_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	c_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	c_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );
	c_40_50_60_70 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0xc );
	c_42_52_62_72 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0x3 );
	c_41_51_61_71 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0xc );
	c_43_53_63_73 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0x3 );

	b_1           = _mm256_broadcast_sd( &B[5] );
	b_2           = _mm256_broadcast_sd( &B[6] );
	b_3           = _mm256_broadcast_sd( &B[7] );

	_mm256_store_pd( &C0[0+ldc*0], c_00_10_20_30 );
	_mm256_store_pd( &C1[0+ldc*0], c_40_50_60_70 );

	// k=kmax-2
	ab_tmp0      = _mm256_mul_pd( a_0123, b_1 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_tmp0 );
	c_41_51_61_71 = _mm256_add_pd( c_41_51_61_71, ab_tmp1 );
	ab_tmp0      = _mm256_mul_pd( a_0123, b_2 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_2 );
	c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_tmp0 );
	c_42_52_62_72 = _mm256_add_pd( c_42_52_62_72, ab_tmp1 );
	b_2           = _mm256_broadcast_sd( &B[10] );
	ab_tmp0      = _mm256_mul_pd( a_0123, b_3 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_3 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_tmp0 );
	c_43_53_63_73 = _mm256_add_pd( c_43_53_63_73, ab_tmp1 );
	b_3           = _mm256_broadcast_sd( &B[11] );

	_mm256_store_pd( &C0[0+ldc*1], c_01_11_21_31 );
	_mm256_store_pd( &C1[0+ldc*1], c_41_51_61_71 );

	// k=kmax-1
	ab_tmp0      = _mm256_mul_pd( a_0123, b_2 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_2 );
	c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_tmp0 );
	c_42_52_62_72 = _mm256_add_pd( c_42_52_62_72, ab_tmp1 );
	ab_tmp0      = _mm256_mul_pd( a_0123, b_3 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_3 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_tmp0 );
	c_43_53_63_73 = _mm256_add_pd( c_43_53_63_73, ab_tmp1 );
	b_3           = _mm256_broadcast_sd( &B[15] );

	_mm256_store_pd( &C0[0+ldc*2], c_02_12_22_32 );
	_mm256_store_pd( &C1[0+ldc*2], c_42_52_62_72 );

	// k=kmax
	ab_tmp0      = _mm256_mul_pd( a_0123, b_3 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_3 );
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_tmp0 );
	c_43_53_63_73 = _mm256_add_pd( c_43_53_63_73, ab_tmp1 );

	_mm256_store_pd( &C0[0+ldc*3], c_03_13_23_33 );
	_mm256_store_pd( &C1[0+ldc*3], c_43_53_63_73 );

	}



// normal-transposed, 8x2 with data packed in 4
void kernel_dtrmm_nt_l_8x2_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;

	int k;
	
	__m256d
		a_0123, a_4567, //A_0123,
		b_0101, b_1010,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_20_31, c_01_10_21_30,
		c_40_51_60_71, c_41_50_61_70;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	// zero registers
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	c_40_51_60_71 = _mm256_setzero_pd();
	c_41_50_61_70 = _mm256_setzero_pd();


	for(k=0; k<kmax-4; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp1 );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp1 );


/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp1 );


/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp1 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	// final triangle

	__m256d
		b_1,
		c_00_10_20_30, c_01_11_21_31,
		c_40_50_60_70, c_41_51_61_71,
		d_00_10_20_30, d_01_11_21_31,
		d_40_50_60_70, d_41_51_61_71;

	// k=kmax-3
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	ab_tmp1       = _mm256_mul_pd( a_4567, b_0101 );
	//b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
	c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp1 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	ab_tmp1       = _mm256_mul_pd( a_4567, b_1010 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
	c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp1 );
	
	c_00_10_20_30 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
	c_01_11_21_31 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
	c_40_50_60_70 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0xa );
	c_41_51_61_71 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0x5 );

	b_1           = _mm256_broadcast_sd( &B[5] );

	_mm256_store_pd( &C0[0+ldc*0], c_00_10_20_30 );
	_mm256_store_pd( &C1[0+ldc*0], c_40_50_60_70 );

	// k=kmax-2
	ab_tmp0      = _mm256_mul_pd( a_0123, b_1 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_tmp0 );
	c_41_51_61_71 = _mm256_add_pd( c_41_51_61_71, ab_tmp1 );

	_mm256_store_pd( &C0[0+ldc*1], c_01_11_21_31 );
	_mm256_store_pd( &C1[0+ldc*1], c_41_51_61_71 );

	}



// normal-transposed, 4x4 with data packed in 4
void kernel_dtrmm_nt_l_4x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;

	int k;
	
	__m256d
		a_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_temp, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();


	for(k=0; k<kmax-4; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );


/*	__builtin_prefetch( A+48 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );


/*	__builtin_prefetch( A+56 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		A += 16;
		B += 16;

		}

	// final triangle

	__m256d
		b_1, b_2, b_3,
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33;

	// k=kmax-3
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	//b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
	
	c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
	
	c_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	c_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	c_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	c_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );

	b_1           = _mm256_broadcast_sd( &B[5] );
	b_2           = _mm256_broadcast_sd( &B[6] );
	b_3           = _mm256_broadcast_sd( &B[7] );

	_mm256_store_pd( &C[0+ldc*0], c_00_10_20_30 );

	// k=kmax-2
	ab_temp       = _mm256_mul_pd( a_0123, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2 );
	c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
	b_2           = _mm256_broadcast_sd( &B[10] );
	ab_temp       = _mm256_mul_pd( a_0123, b_3 );
	a_0123        = _mm256_load_pd( &A[8] ); // prefetch
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
	b_3           = _mm256_broadcast_sd( &B[11] );

	_mm256_store_pd( &C[0+ldc*1], c_01_11_21_31 );

	// k=kmax-1
	ab_temp       = _mm256_mul_pd( a_0123, b_2 );
	c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3 );
	a_0123        = _mm256_load_pd( &A[12] ); // prefetch
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
	b_3           = _mm256_broadcast_sd( &B[15] );

	_mm256_store_pd( &C[0+ldc*2], c_02_12_22_32 );
	
	// k=kmax
	ab_temp       = _mm256_mul_pd( a_0123, b_3 );
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );

	_mm256_store_pd( &C[0+ldc*3], c_03_13_23_33 );

	}



void kernel_dtrmm_nt_l_4x2_lib4(int kmax, double *A, double *B, double *C)
	{
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;

	int k;
	
	__m256d
		a_0123,
		b_0101, b_1010,
		ab_temp, // temporary results
		c_00_11_20_31, c_01_10_21_30, C_00_11_20_31, C_01_10_21_30;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	// zero registers
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	C_00_11_20_31 = _mm256_setzero_pd();
	C_01_10_21_30 = _mm256_setzero_pd();


	for(k=0; k<kmax-4; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, ab_temp );


/*	__builtin_prefetch( A+48 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );


/*	__builtin_prefetch( A+56 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, ab_temp );
		
		A += 16;
		B += 16;

		}
	
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, C_00_11_20_31 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, C_01_10_21_30 );

	// final triangle

	__m256d
		b_1,
		c_00_10_20_30, c_01_11_21_31,
		d_00_10_20_30, d_01_11_21_31;

	// k=kmax-3
	ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	//b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
	ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );

	c_00_10_20_30 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
	c_01_11_21_31 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );

	b_1           = _mm256_broadcast_sd( &B[5] );

	_mm256_store_pd( &C[0+ldc*0], c_00_10_20_30 );

	// k=kmax-2
	ab_temp       = _mm256_mul_pd( a_0123, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );

	_mm256_store_pd( &C[0+ldc*1], c_01_11_21_31 );

	}



void kernel_dtrmm_nt_l_2x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
		
	// kmax is multiple of bs
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		b_2 = B[2+lda*1];
		b_3 = B[3+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		b_2 = B[2+lda*2];
		b_3 = B[3+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		b_2 = B[2+lda*3];
		b_3 = B[3+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		
		
		A += 16;
		B += 16;

		}
		
	// final triangle

	// k=kmax-3
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
		
	b_0 = B[0+lda*0];
	b_1 = B[1+lda*0];
	b_2 = B[2+lda*0];
	b_3 = B[3+lda*0];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=kmax-2
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
		
	b_1 = B[1+lda*1];
	b_2 = B[2+lda*1];
	b_3 = B[3+lda*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
		
	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=kmax-1
	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
		
	b_2 = B[2+lda*2];
	b_3 = B[3+lda*2];

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	C[0+ldc*2] = c_02;
	C[1+ldc*2] = c_12;
		
	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=kmax
	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
		
	b_3 = B[3+lda*3];

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	C[0+ldc*3] = c_03;
	C[1+ldc*3] = c_13;
		
	}



void kernel_dtrmm_nt_l_2x2_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
		
	// kmax is multiple of bs
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		
		
		A += 16;
		B += 16;

		}
		
	// final triangle

	// k=kmax-3
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
		
	b_0 = B[0+lda*0];
	b_1 = B[1+lda*0];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	// k=kmax-2
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
		
	b_1 = B[1+lda*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	
	}

#endif


// A upper triangle matrix on the left
void kernel_dtrmm_l_u_nt_12x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D1 + 4*sdd;

	const int bs = 4;

	int k;

	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82;
	
	// prefetch
	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_8 = _mm256_load_pd( &A2[0] );
	b_0 = _mm256_load_pd( &B[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_80 = _mm256_setzero_pd();
	c_81 = _mm256_setzero_pd();
	c_82 = _mm256_setzero_pd();
	c_83 = _mm256_setzero_pd();

	k = 0;

	// XXX assume kmax >= 12 !!!!!!!!!!!!!!!!

	// triangle at the beginning

	// k = 0
	a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	b_0  = _mm256_load_pd( &B[4] ); // prefetch

	
	// k = 1
	a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	b_0  = _mm256_load_pd( &B[8] ); // prefetch


	// k = 2
	a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	b_0  = _mm256_load_pd( &B[12] ); // prefetch


	// k = 3
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
	b_0  = _mm256_load_pd( &B[16] ); // prefetch


	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;

	// k = 4
	a_4  = _mm256_blend_pd( zeros, a_4, 0x1 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[4] ); // prefetch
	b_0  = _mm256_load_pd( &B[4] ); // prefetch

		
	// k = 5
	a_4  = _mm256_blend_pd( zeros, a_4, 0x3 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[8] ); // prefetch
	b_0  = _mm256_load_pd( &B[8] ); // prefetch


	// k = 6
	a_4  = _mm256_blend_pd( zeros, a_4, 0x7 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[12] ); // prefetch
	b_0  = _mm256_load_pd( &B[12] ); // prefetch


	// k = 7
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
	b_0  = _mm256_load_pd( &B[16] ); // prefetch
	a_8  = _mm256_load_pd( &A2[16] ); // prefetch

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	// k = 8
	a_8  = _mm256_blend_pd( zeros, a_8, 0x1 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[4] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[4] ); // prefetch
	a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
	// k = 9
	a_8  = _mm256_blend_pd( zeros, a_8, 0x3 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[8] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[8] ); // prefetch
	a_8  = _mm256_load_pd( &A2[8] ); // prefetch


	// k = 10
	a_8  = _mm256_blend_pd( zeros, a_8, 0x7 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[12] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[12] ); // prefetch
	a_8  = _mm256_load_pd( &A2[12] ); // prefetch


	// k = 11
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[16] ); // prefetch
	a_8  = _mm256_load_pd( &A2[16] ); // prefetch

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	k = 12;

	for(; k<kmax-3; k+=4)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[4] ); // prefetch
		a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
		
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[8] ); // prefetch
		a_8  = _mm256_load_pd( &A2[8] ); // prefetch



		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[12] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[12] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[12] ); // prefetch
		a_8  = _mm256_load_pd( &A2[12] ); // prefetch


		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[16] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[16] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[16] ); // prefetch
		a_8  = _mm256_load_pd( &A2[16] ); // prefetch


		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}
	if(kmax%4>=2)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[4] );
		a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
		
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[8] );
		a_8  = _mm256_load_pd( &A2[8] ); // prefetch
	
		
		A0 += 8;
		A1 += 8;
		A2 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
/*		a_0  = _mm256_load_pd( &A0[4] ); // prefetch*/
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
/*		a_4  = _mm256_load_pd( &A1[4] ); // prefetch*/
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
//		b_0  = _mm256_load_pd( &B[4] );
/*		a_8  = _mm256_load_pd( &A2[4] ); // prefetch*/
			

		}


	__m256d
		e_0, e_1, e_2, e_3,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_80, d_81, d_82, d_83;

	e_0 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_1 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_2 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_3 = _mm256_blend_pd( c_02, c_03, 0x5 );

	c_00 = _mm256_blend_pd( e_0, e_2, 0xc );
	c_02 = _mm256_blend_pd( e_0, e_2, 0x3 );
	c_01 = _mm256_blend_pd( e_1, e_3, 0xc );
	c_03 = _mm256_blend_pd( e_1, e_3, 0x3 );

	e_0 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_1 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_2 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_3 = _mm256_blend_pd( c_42, c_43, 0x5 );

	c_40 = _mm256_blend_pd( e_0, e_2, 0xc );
	c_42 = _mm256_blend_pd( e_0, e_2, 0x3 );
	c_41 = _mm256_blend_pd( e_1, e_3, 0xc );
	c_43 = _mm256_blend_pd( e_1, e_3, 0x3 );

	e_0 = _mm256_blend_pd( c_80, c_81, 0xa );
	e_1 = _mm256_blend_pd( c_80, c_81, 0x5 );
	e_2 = _mm256_blend_pd( c_82, c_83, 0xa );
	e_3 = _mm256_blend_pd( c_82, c_83, 0x5 );

	c_80 = _mm256_blend_pd( e_0, e_2, 0xc );
	c_82 = _mm256_blend_pd( e_0, e_2, 0x3 );
	c_81 = _mm256_blend_pd( e_1, e_3, 0xc );
	c_83 = _mm256_blend_pd( e_1, e_3, 0x3 );

	if(alg==0) // C = A * B'
		{
		_mm256_store_pd( &D0[0+bs*0], c_00 );
		_mm256_store_pd( &D0[0+bs*1], c_01 );
		_mm256_store_pd( &D0[0+bs*2], c_02 );
		_mm256_store_pd( &D0[0+bs*3], c_03 );

		_mm256_store_pd( &D1[0+bs*0], c_40 );
		_mm256_store_pd( &D1[0+bs*1], c_41 );
		_mm256_store_pd( &D1[0+bs*2], c_42 );
		_mm256_store_pd( &D1[0+bs*3], c_43 );

		_mm256_store_pd( &D2[0+bs*0], c_80 );
		_mm256_store_pd( &D2[0+bs*1], c_81 );
		_mm256_store_pd( &D2[0+bs*2], c_82 );
		_mm256_store_pd( &D2[0+bs*3], c_83 );
		}
	else 
		{
		d_00 = _mm256_load_pd( &C0[0+bs*0] );
		d_01 = _mm256_load_pd( &C0[0+bs*1] );
		d_02 = _mm256_load_pd( &C0[0+bs*2] );
		d_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_40 = _mm256_load_pd( &C1[0+bs*0] );
		d_41 = _mm256_load_pd( &C1[0+bs*1] );
		d_42 = _mm256_load_pd( &C1[0+bs*2] );
		d_43 = _mm256_load_pd( &C1[0+bs*3] );
		
		d_80 = _mm256_load_pd( &C2[0+bs*0] );
		d_81 = _mm256_load_pd( &C2[0+bs*1] );
		d_82 = _mm256_load_pd( &C2[0+bs*2] );
		d_83 = _mm256_load_pd( &C2[0+bs*3] );
		
		if(alg==1) // C += A * B'
			{
			d_00 = _mm256_add_pd( d_00, c_00 );
			d_01 = _mm256_add_pd( d_01, c_01 );
			d_02 = _mm256_add_pd( d_02, c_02 );
			d_03 = _mm256_add_pd( d_03, c_03 );

			d_40 = _mm256_add_pd( d_40, c_40 );
			d_41 = _mm256_add_pd( d_41, c_41 );
			d_42 = _mm256_add_pd( d_42, c_42 );
			d_43 = _mm256_add_pd( d_43, c_43 );

			d_80 = _mm256_add_pd( d_80, c_80 );
			d_81 = _mm256_add_pd( d_81, c_81 );
			d_82 = _mm256_add_pd( d_82, c_82 );
			d_83 = _mm256_add_pd( d_83, c_83 );
			}
		else // C -= A * B'
			{
			d_00 = _mm256_sub_pd( d_00, c_00 );
			d_01 = _mm256_sub_pd( d_01, c_01 );
			d_02 = _mm256_sub_pd( d_02, c_02 );
			d_03 = _mm256_sub_pd( d_03, c_03 );

			d_40 = _mm256_sub_pd( d_40, c_40 );
			d_41 = _mm256_sub_pd( d_41, c_41 );
			d_42 = _mm256_sub_pd( d_42, c_42 );
			d_43 = _mm256_sub_pd( d_43, c_43 );

			d_80 = _mm256_sub_pd( d_80, c_80 );
			d_81 = _mm256_sub_pd( d_81, c_81 );
			d_82 = _mm256_sub_pd( d_82, c_82 );
			d_83 = _mm256_sub_pd( d_83, c_83 );
			}

		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_store_pd( &D0[0+bs*1], d_01 );
		_mm256_store_pd( &D0[0+bs*2], d_02 );
		_mm256_store_pd( &D0[0+bs*3], d_03 );

		_mm256_store_pd( &D1[0+bs*0], d_40 );
		_mm256_store_pd( &D1[0+bs*1], d_41 );
		_mm256_store_pd( &D1[0+bs*2], d_42 );
		_mm256_store_pd( &D1[0+bs*3], d_43 );

		_mm256_store_pd( &D2[0+bs*0], d_80 );
		_mm256_store_pd( &D2[0+bs*1], d_81 );
		_mm256_store_pd( &D2[0+bs*2], d_82 );
		_mm256_store_pd( &D2[0+bs*3], d_83 );
		}

	}



// A upper triangle matrix on the left
void kernel_dtrmm_l_u_nt_8x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;

	const int bs = 4;

	int k;

	__m256d
		zeros,
		a_0123, a_4567, //A_0123,
		b_0123, b_1032, b_3210, b_2301,
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	// zero registers
	zeros         = _mm256_setzero_pd();
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();
	c_40_51_62_73 = _mm256_setzero_pd();
	c_41_50_63_72 = _mm256_setzero_pd();
	c_43_52_61_70 = _mm256_setzero_pd();
	c_42_53_60_71 = _mm256_setzero_pd();

	k = 0;

	// XXX assume kmax >= 8 !!!!!!!!!!!!!!!!!!!

	// triangle at the beginning

	// k = 0
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	
	// k = 1
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch

	// k = 2
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch

	// k = 3
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch

	A0 += 16;
	A1 += 16;
	B  += 16;
	k  += 4;

	// k = 4
	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x1 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	
	// k = 5
	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x3 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch

	// k = 6
	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x7 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch

	// k = 7
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	
	A0 += 16;
	A1 += 16;
	B  += 16;
	k  += 4;


	for(; k<kmax-3; k+=4)
		{
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch


		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch


		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	if(kmax%4>=2)
		{
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
/*		b_0123        = _mm256_load_pd( &B[4] ); // prefetch*/
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
/*		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch*/
		c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
/*		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch*/
		
		}


	__m256d
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_40_50_62_72, c_41_51_63_73, c_42_52_60_70, c_43_53_61_71,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		c_40_50_60_70, c_41_51_61_71, c_42_52_62_72, c_43_53_63_73,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33,
		d_40_50_60_70, d_41_51_61_71, d_42_52_62_72, d_43_53_63_73;

	c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
	c_40_50_62_72 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
	c_41_51_63_73 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
	c_42_52_60_70 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
	c_43_53_61_71 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
	
	c_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	c_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	c_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	c_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );
	c_40_50_60_70 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0xc );
	c_42_52_62_72 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0x3 );
	c_41_51_61_71 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0xc );
	c_43_53_63_73 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0x3 );
		
	if(alg==0) // C = A * B'
		{
		_mm256_store_pd( &D0[0+bs*0], c_00_10_20_30 );
		_mm256_store_pd( &D0[0+bs*1], c_01_11_21_31 );
		_mm256_store_pd( &D0[0+bs*2], c_02_12_22_32 );
		_mm256_store_pd( &D0[0+bs*3], c_03_13_23_33 );
		_mm256_store_pd( &D1[0+bs*0], c_40_50_60_70 );
		_mm256_store_pd( &D1[0+bs*1], c_41_51_61_71 );
		_mm256_store_pd( &D1[0+bs*2], c_42_52_62_72 );
		_mm256_store_pd( &D1[0+bs*3], c_43_53_63_73 );
		}
	else 
		{
		d_00_10_20_30 = _mm256_load_pd( &C0[0+bs*0] );
		d_01_11_21_31 = _mm256_load_pd( &C0[0+bs*1] );
		d_02_12_22_32 = _mm256_load_pd( &C0[0+bs*2] );
		d_03_13_23_33 = _mm256_load_pd( &C0[0+bs*3] );
		d_40_50_60_70 = _mm256_load_pd( &C1[0+bs*0] );
		d_41_51_61_71 = _mm256_load_pd( &C1[0+bs*1] );
		d_42_52_62_72 = _mm256_load_pd( &C1[0+bs*2] );
		d_43_53_63_73 = _mm256_load_pd( &C1[0+bs*3] );
		
		if(alg==1) // C += A * B'
			{
			d_00_10_20_30 = _mm256_add_pd( d_00_10_20_30, c_00_10_20_30 );
			d_01_11_21_31 = _mm256_add_pd( d_01_11_21_31, c_01_11_21_31 );
			d_02_12_22_32 = _mm256_add_pd( d_02_12_22_32, c_02_12_22_32 );
			d_03_13_23_33 = _mm256_add_pd( d_03_13_23_33, c_03_13_23_33 );
			d_40_50_60_70 = _mm256_add_pd( d_40_50_60_70, c_40_50_60_70 );
			d_41_51_61_71 = _mm256_add_pd( d_41_51_61_71, c_41_51_61_71 );
			d_42_52_62_72 = _mm256_add_pd( d_42_52_62_72, c_42_52_62_72 );
			d_43_53_63_73 = _mm256_add_pd( d_43_53_63_73, c_43_53_63_73 );
			}
		else // C -= A * B'
			{
			d_00_10_20_30 = _mm256_sub_pd( d_00_10_20_30, c_00_10_20_30 );
			d_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, c_01_11_21_31 );
			d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, c_02_12_22_32 );
			d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, c_03_13_23_33 );
			d_40_50_60_70 = _mm256_sub_pd( d_40_50_60_70, c_40_50_60_70 );
			d_41_51_61_71 = _mm256_sub_pd( d_41_51_61_71, c_41_51_61_71 );
			d_42_52_62_72 = _mm256_sub_pd( d_42_52_62_72, c_42_52_62_72 );
			d_43_53_63_73 = _mm256_sub_pd( d_43_53_63_73, c_43_53_63_73 );
			}

		_mm256_store_pd( &D0[0+bs*0], d_00_10_20_30 );
		_mm256_store_pd( &D0[0+bs*1], d_01_11_21_31 );
		_mm256_store_pd( &D0[0+bs*2], d_02_12_22_32 );
		_mm256_store_pd( &D0[0+bs*3], d_03_13_23_33 );
		_mm256_store_pd( &D1[0+bs*0], d_40_50_60_70 );
		_mm256_store_pd( &D1[0+bs*1], d_41_51_61_71 );
		_mm256_store_pd( &D1[0+bs*2], d_42_52_62_72 );
		_mm256_store_pd( &D1[0+bs*3], d_43_53_63_73 );
		}

	}



// A upper triangle matrix on the left
void kernel_dtrmm_l_u_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		zeros,
		a_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_temp, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();

	k = 0;

	// XXX assume kmax >= 4 !!!!!!!!!!!!!!!!!!!!!!!!!

	// triangle at the beginning

	// k = 0
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
	
	// k = 1
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[8] ); // prefetch
	c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );

	// k = 2
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[12] ); // prefetch
	c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );

	// k = 3
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[16] ); // prefetch
	c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
	
	A += 16;
	B += 16;
	k += 4;

	for(; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );


/*	__builtin_prefetch( A+48 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );


/*	__builtin_prefetch( A+56 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );

		
		A += 16;
		B += 16;

		}
	for(; k<kmax-1; k+=2)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		
		A += 8;
		B += 8;

		}

	for(; k<kmax; k+=1)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
/*		b_0123        = _mm256_load_pd( &B[4] ); // prefetch*/
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
/*		a_0123        = _mm256_load_pd( &A[4] ); // prefetch*/
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
//		A += 4; 
//		B += 4;

		}

	__m256d
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33;

	c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
	
	c_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	c_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	c_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	c_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );
		
	if(alg==0) // C = A * B'
		{
		_mm256_store_pd( &D[0+bs*0], c_00_10_20_30 );
		_mm256_store_pd( &D[0+bs*1], c_01_11_21_31 );
		_mm256_store_pd( &D[0+bs*2], c_02_12_22_32 );
		_mm256_store_pd( &D[0+bs*3], c_03_13_23_33 );
		}
	else 
		{
		d_00_10_20_30 = _mm256_load_pd( &C[0+bs*0] );
		d_01_11_21_31 = _mm256_load_pd( &C[0+bs*1] );
		d_02_12_22_32 = _mm256_load_pd( &C[0+bs*2] );
		d_03_13_23_33 = _mm256_load_pd( &C[0+bs*3] );
		
		if(alg==1) // C += A * B'
			{
			d_00_10_20_30 = _mm256_add_pd( d_00_10_20_30, c_00_10_20_30 );
			d_01_11_21_31 = _mm256_add_pd( d_01_11_21_31, c_01_11_21_31 );
			d_02_12_22_32 = _mm256_add_pd( d_02_12_22_32, c_02_12_22_32 );
			d_03_13_23_33 = _mm256_add_pd( d_03_13_23_33, c_03_13_23_33 );
			}
		else // C -= A * B'
			{
			d_00_10_20_30 = _mm256_sub_pd( d_00_10_20_30, c_00_10_20_30 );
			d_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, c_01_11_21_31 );
			d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, c_02_12_22_32 );
			d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, c_03_13_23_33 );
			}

		_mm256_store_pd( &D[0+bs*0], d_00_10_20_30 );
		_mm256_store_pd( &D[0+bs*1], d_01_11_21_31 );
		_mm256_store_pd( &D[0+bs*2], d_02_12_22_32 );
		_mm256_store_pd( &D[0+bs*3], d_03_13_23_33 );
		}

	}



// rank 4 update
void corner_dtrmm_l_u_nt_4x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	__m256d
		tmp, zeros,
		a_0,
		b_0,
		d_0, d_1, d_2, d_3;
	

	zeros = _mm256_setzero_pd();


	// k = 0
	a_0 = _mm256_load_pd( &A[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );

	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_mul_pd( a_0, b_0 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_mul_pd( a_0, b_0 );
	b_0 = _mm256_broadcast_sd( &B[2] );
	d_2 = _mm256_mul_pd( a_0, b_0 );
	b_0 = _mm256_broadcast_sd( &B[3] );
	d_3 = _mm256_mul_pd( a_0, b_0 );


	// k = 1
	a_0 = _mm256_load_pd( &A[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );

	b_0 = _mm256_broadcast_sd( &B[4] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_add_pd( d_0, tmp );
	b_0 = _mm256_broadcast_sd( &B[5] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_1 = _mm256_add_pd( d_1, tmp );
	b_0 = _mm256_broadcast_sd( &B[6] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_2 = _mm256_add_pd( d_2, tmp );
	b_0 = _mm256_broadcast_sd( &B[7] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_3 = _mm256_add_pd( d_3, tmp );


	// k = 2
	a_0 = _mm256_load_pd( &A[8] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );

	b_0 = _mm256_broadcast_sd( &B[8] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_add_pd( d_0, tmp );
	b_0 = _mm256_broadcast_sd( &B[9] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_1 = _mm256_add_pd( d_1, tmp );
	b_0 = _mm256_broadcast_sd( &B[10] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_2 = _mm256_add_pd( d_2, tmp );
	b_0 = _mm256_broadcast_sd( &B[11] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_3 = _mm256_add_pd( d_3, tmp );


	// k = 3
	a_0 = _mm256_load_pd( &A[12] );

	b_0 = _mm256_broadcast_sd( &B[12] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_add_pd( d_0, tmp );
	b_0 = _mm256_broadcast_sd( &B[13] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_1 = _mm256_add_pd( d_1, tmp );
	b_0 = _mm256_broadcast_sd( &B[14] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_2 = _mm256_add_pd( d_2, tmp );
	b_0 = _mm256_broadcast_sd( &B[15] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_3 = _mm256_add_pd( d_3, tmp );


	if(alg!=0)
		{
		if(alg==1)
			{
			tmp = _mm256_load_pd( &C[0+bs*0] );
			d_0 = _mm256_add_pd( tmp, d_0 );
			tmp = _mm256_load_pd( &C[0+bs*1] );
			d_1 = _mm256_add_pd( tmp, d_1 );
			tmp = _mm256_load_pd( &C[0+bs*2] );
			d_2 = _mm256_add_pd( tmp, d_2 );
			tmp = _mm256_load_pd( &C[0+bs*3] );
			d_3 = _mm256_add_pd( tmp, d_3 );
			}
		else
			{
			tmp = _mm256_load_pd( &C[0+bs*0] );
			d_0 = _mm256_sub_pd( tmp, d_0 );
			tmp = _mm256_load_pd( &C[0+bs*1] );
			d_1 = _mm256_sub_pd( tmp, d_1 );
			tmp = _mm256_load_pd( &C[0+bs*2] );
			d_2 = _mm256_sub_pd( tmp, d_2 );
			tmp = _mm256_load_pd( &C[0+bs*3] );
			d_3 = _mm256_sub_pd( tmp, d_3 );
			}
		}
	
	_mm256_store_pd( &D[0+bs*0], d_0 );
	_mm256_store_pd( &D[0+bs*1], d_1 );
	_mm256_store_pd( &D[0+bs*2], d_2 );
	_mm256_store_pd( &D[0+bs*3], d_3 );


	}



// rank 3 update
void corner_dtrmm_l_u_nt_3x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	__m256d
		tmp, zeros,
		a_0,
		b_0,
		d_0, d_1, d_2, d_3;
	

	zeros = _mm256_setzero_pd();


	// k = 0
	a_0 = _mm256_load_pd( &A[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );

	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_mul_pd( a_0, b_0 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_mul_pd( a_0, b_0 );
	b_0 = _mm256_broadcast_sd( &B[2] );
	d_2 = _mm256_mul_pd( a_0, b_0 );
	b_0 = _mm256_broadcast_sd( &B[3] );
	d_3 = _mm256_mul_pd( a_0, b_0 );


	// k = 1
	a_0 = _mm256_load_pd( &A[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );

	b_0 = _mm256_broadcast_sd( &B[4] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_add_pd( d_0, tmp );
	b_0 = _mm256_broadcast_sd( &B[5] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_1 = _mm256_add_pd( d_1, tmp );
	b_0 = _mm256_broadcast_sd( &B[6] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_2 = _mm256_add_pd( d_2, tmp );
	b_0 = _mm256_broadcast_sd( &B[7] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_3 = _mm256_add_pd( d_3, tmp );


	// k = 2
	a_0 = _mm256_load_pd( &A[8] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );

	b_0 = _mm256_broadcast_sd( &B[8] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_add_pd( d_0, tmp );
	b_0 = _mm256_broadcast_sd( &B[9] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_1 = _mm256_add_pd( d_1, tmp );
	b_0 = _mm256_broadcast_sd( &B[10] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_2 = _mm256_add_pd( d_2, tmp );
	b_0 = _mm256_broadcast_sd( &B[11] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_3 = _mm256_add_pd( d_3, tmp );


	if(alg!=0)
		{
		if(alg==1)
			{
			tmp = _mm256_load_pd( &C[0+bs*0] );
			d_0 = _mm256_add_pd( tmp, d_0 );
			tmp = _mm256_load_pd( &C[0+bs*1] );
			d_1 = _mm256_add_pd( tmp, d_1 );
			tmp = _mm256_load_pd( &C[0+bs*2] );
			d_2 = _mm256_add_pd( tmp, d_2 );
			tmp = _mm256_load_pd( &C[0+bs*3] );
			d_3 = _mm256_add_pd( tmp, d_3 );
			}
		else
			{
			tmp = _mm256_load_pd( &C[0+bs*0] );
			d_0 = _mm256_sub_pd( tmp, d_0 );
			tmp = _mm256_load_pd( &C[0+bs*1] );
			d_1 = _mm256_sub_pd( tmp, d_1 );
			tmp = _mm256_load_pd( &C[0+bs*2] );
			d_2 = _mm256_sub_pd( tmp, d_2 );
			tmp = _mm256_load_pd( &C[0+bs*3] );
			d_3 = _mm256_sub_pd( tmp, d_3 );
			}
		}
	
	tmp = _mm256_load_pd( &D[0+bs*0] );
	d_0 = _mm256_blend_pd( tmp, d_0, 0x7 );
	_mm256_store_pd( &D[0+bs*0], d_0 );
	tmp = _mm256_load_pd( &D[0+bs*1] );
	d_1 = _mm256_blend_pd( tmp, d_1, 0x7 );
	_mm256_store_pd( &D[0+bs*1], d_1 );
	tmp = _mm256_load_pd( &D[0+bs*2] );
	d_2 = _mm256_blend_pd( tmp, d_2, 0x7 );
	_mm256_store_pd( &D[0+bs*2], d_2 );
	tmp = _mm256_load_pd( &D[0+bs*3] );
	d_3 = _mm256_blend_pd( tmp, d_3, 0x7 );
	_mm256_store_pd( &D[0+bs*3], d_3 );


	}



// rank 2 update
void corner_dtrmm_l_u_nt_2x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	__m128d
		tmp, zeros,
		a_0,
		b_0,
		d_0, d_1, d_2, d_3;
	

	zeros = _mm_setzero_pd();


	// k = 0
	a_0 = _mm_load_pd( &A[0] );
	a_0 = _mm_blend_pd( zeros, a_0, 0x1 );

	b_0 = _mm_loaddup_pd( &B[0] );
	d_0 = _mm_mul_pd( a_0, b_0 );
	b_0 = _mm_loaddup_pd( &B[1] );
	d_1 = _mm_mul_pd( a_0, b_0 );
	b_0 = _mm_loaddup_pd( &B[2] );
	d_2 = _mm_mul_pd( a_0, b_0 );
	b_0 = _mm_loaddup_pd( &B[3] );
	d_3 = _mm_mul_pd( a_0, b_0 );


	// k = 1
	a_0 = _mm_load_pd( &A[4] );
	a_0 = _mm_blend_pd( zeros, a_0, 0x3 );

	b_0 = _mm_loaddup_pd( &B[4] );
	tmp = _mm_mul_pd( a_0, b_0 );
	d_0 = _mm_add_pd( d_0, tmp );
	b_0 = _mm_loaddup_pd( &B[5] );
	tmp = _mm_mul_pd( a_0, b_0 );
	d_1 = _mm_add_pd( d_1, tmp );
	b_0 = _mm_loaddup_pd( &B[6] );
	tmp = _mm_mul_pd( a_0, b_0 );
	d_2 = _mm_add_pd( d_2, tmp );
	b_0 = _mm_loaddup_pd( &B[7] );
	tmp = _mm_mul_pd( a_0, b_0 );
	d_3 = _mm_add_pd( d_3, tmp );


	if(alg!=0)
		{
		if(alg==1)
			{
			tmp = _mm_load_pd( &C[0+bs*0] );
			d_0 = _mm_add_pd( tmp, d_0 );
			tmp = _mm_load_pd( &C[0+bs*1] );
			d_1 = _mm_add_pd( tmp, d_1 );
			tmp = _mm_load_pd( &C[0+bs*2] );
			d_2 = _mm_add_pd( tmp, d_2 );
			tmp = _mm_load_pd( &C[0+bs*3] );
			d_3 = _mm_add_pd( tmp, d_3 );
			}
		else
			{
			tmp = _mm_load_pd( &C[0+bs*0] );
			d_0 = _mm_sub_pd( tmp, d_0 );
			tmp = _mm_load_pd( &C[0+bs*1] );
			d_1 = _mm_sub_pd( tmp, d_1 );
			tmp = _mm_load_pd( &C[0+bs*2] );
			d_2 = _mm_sub_pd( tmp, d_2 );
			tmp = _mm_load_pd( &C[0+bs*3] );
			d_3 = _mm_sub_pd( tmp, d_3 );
			}
		}
	
	_mm_store_pd( &D[0+bs*0], d_0 );
	_mm_store_pd( &D[0+bs*1], d_1 );
	_mm_store_pd( &D[0+bs*2], d_2 );
	_mm_store_pd( &D[0+bs*3], d_3 );

	}



// rank 1 update
void corner_dtrmm_l_u_nt_1x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0,
		b_0,
		d_0, d_1, d_2, d_3;
	

	a_0 = A[0];

	b_0 = B[0];
	d_0 = a_0*b_0;
	b_0 = B[1];
	d_1 = a_0*b_0;
	b_0 = B[2];
	d_2 = a_0*b_0;
	b_0 = B[3];
	d_3 = a_0*b_0;


	if(alg!=0)
		{
		if(alg==1)
			{
			d_0 = C[0+bs*0] + d_0;
			d_1 = C[0+bs*1] + d_1;
			d_2 = C[0+bs*2] + d_2;
			d_3 = C[0+bs*3] + d_3;
			}
		else
			{
			d_0 = C[0+bs*0] - d_0;
			d_1 = C[0+bs*1] - d_1;
			d_2 = C[0+bs*2] - d_2;
			d_3 = C[0+bs*3] - d_3;
			}
		}
	
	D[0+bs*0] = d_0;
	D[0+bs*1] = d_1;
	D[0+bs*2] = d_2;
	D[0+bs*3] = d_3;

	}




