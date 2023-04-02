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



#define LOW_ACC 0
#define NEWTON_IT 1


//#define NO_BRANCH



// new kernels

void kernel_dpotrf_nt_12x4_lib4_new(int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Am1 = Am0 + 4*sdam;
	double *Am2 = Am0 + 8*sdam;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;
	
	const int bs = 4;
	
	double d_temp;
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82;
	
	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_80 = _mm256_setzero_pd();
	c_81 = _mm256_setzero_pd();
	c_83 = _mm256_setzero_pd();
	c_82 = _mm256_setzero_pd();

	k = 0;

	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am0[0] );
		a_4 = _mm256_load_pd( &Am1[0] );
		a_8 = _mm256_load_pd( &Am2[0] );
		b_0 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[4] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[4] ); // prefetch
			
			
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[8] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[8] ); // prefetch



			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[12] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[12] ); // prefetch


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[16] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[16] ); // prefetch
			
			
			Am0 += 16;
			Am1 += 16;
			Am2 += 16;
			Bm  += 16;

			}

		}

	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_80, d_81, d_82, d_83,
		e_00, e_01, e_02, e_03;

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_80, c_81, 0xa );
	e_01 = _mm256_blend_pd( c_80, c_81, 0x5 );
	e_02 = _mm256_blend_pd( c_82, c_83, 0xa );
	e_03 = _mm256_blend_pd( c_82, c_83, 0x5 );
	
	d_80 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_82 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_81 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_83 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );

		c_80 = _mm256_load_pd( &C2[0+bs*0] );
		c_81 = _mm256_load_pd( &C2[0+bs*1] );
		c_82 = _mm256_load_pd( &C2[0+bs*2] );
		c_83 = _mm256_load_pd( &C2[0+bs*3] );

		d_80 = _mm256_add_pd( d_80, c_80 );
		d_81 = _mm256_add_pd( d_81, c_81 );
		d_82 = _mm256_add_pd( d_82, c_82 );
		d_83 = _mm256_add_pd( d_83, c_83 );
		}
		
	// factorize
	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	d_80  = _mm256_mul_pd( d_80, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 );
	_mm256_store_pd( &D1[0+bs*0], d_40 );
	_mm256_store_pd( &D2[0+bs*0], d_80 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	d_81  = _mm256_fnmadd_pd( d_80, a_10, d_81 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	d_41  = _mm256_mul_pd( d_41, a_11 );
	d_81  = _mm256_mul_pd( d_81, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 );
	_mm256_store_pd( &D1[0+bs*1], d_41 );
	_mm256_store_pd( &D2[0+bs*1], d_81 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	d_82  = _mm256_fnmadd_pd( d_80, a_20, d_82 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	d_82  = _mm256_fnmadd_pd( d_81, a_21, d_82 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	d_82  = _mm256_mul_pd( d_82, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_store_pd( &D1[0+bs*2], d_42 );
	_mm256_store_pd( &D2[0+bs*2], d_82 );

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
	d_83  = _mm256_fnmadd_pd( d_80, a_30, d_83 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
	d_83  = _mm256_fnmadd_pd( d_81, a_31, d_83 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
	d_83  = _mm256_fnmadd_pd( d_82, a_32, d_83 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
	d_03  = _mm256_mul_pd( d_03, a_33 );
	d_43  = _mm256_mul_pd( d_43, a_33 );
	d_83  = _mm256_mul_pd( d_83, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
	_mm256_store_pd( &D1[0+bs*3], d_43 );
	_mm256_store_pd( &D2[0+bs*3], d_83 );

	}



void kernel_dsyrk_dpotrf_nt_12x4_lib4_new(int kadd, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Ap2 = Ap0 + 8*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *Am2 = Am0 + 8*sdam;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;
	
	const int bs = 4;
	
	double d_temp;
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82;
	
	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_80 = _mm256_setzero_pd();
	c_81 = _mm256_setzero_pd();
	c_83 = _mm256_setzero_pd();
	c_82 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Ap0[0] );
		a_4 = _mm256_load_pd( &Ap1[0] );
		a_8 = _mm256_load_pd( &Ap2[0] );
		b_0 = _mm256_load_pd( &Bp[0] );

		for(; k<kadd-3; k+=4)
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
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
			
			
			
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
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[8] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch



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
			a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[12] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[12] ); // prefetch


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
			a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[16] ); // prefetch
			
			Ap0 += 16;
			Ap1 += 16;
			Ap2 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
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
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[4] );
			a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
			
			
			
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
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[8] );
			a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch
				
			
			Ap0 += 8;
			Ap1 += 8;
			Ap2 += 8;
			Bp  += 8;
			k   += 2;

			}

		if(k<kadd)
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
	/*		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch*/
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	/*		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch*/
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	//		b_0  = _mm256_load_pd( &Bp[4] );
	/*		a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch*/

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			c_00 = _mm256_xor_pd( c_00, sign );
			c_40 = _mm256_xor_pd( c_40, sign );
			c_80 = _mm256_xor_pd( c_80, sign );
			c_01 = _mm256_xor_pd( c_01, sign );
			c_41 = _mm256_xor_pd( c_41, sign );
			c_81 = _mm256_xor_pd( c_81, sign );
			c_02 = _mm256_xor_pd( c_02, sign );
			c_42 = _mm256_xor_pd( c_42, sign );
			c_82 = _mm256_xor_pd( c_82, sign );
			c_03 = _mm256_xor_pd( c_03, sign );
			c_43 = _mm256_xor_pd( c_43, sign );
			c_83 = _mm256_xor_pd( c_83, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am0[0] );
		a_4 = _mm256_load_pd( &Am1[0] );
		a_8 = _mm256_load_pd( &Am2[0] );
		b_0 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[4] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[4] ); // prefetch
			
			
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[8] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[8] ); // prefetch



			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[12] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[12] ); // prefetch


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[16] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[16] ); // prefetch
			
			
			Am0 += 16;
			Am1 += 16;
			Am2 += 16;
			Bm  += 16;

			}

		}

	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_80, d_81, d_82, d_83,
		e_00, e_01, e_02, e_03;

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_80, c_81, 0xa );
	e_01 = _mm256_blend_pd( c_80, c_81, 0x5 );
	e_02 = _mm256_blend_pd( c_82, c_83, 0xa );
	e_03 = _mm256_blend_pd( c_82, c_83, 0x5 );
	
	d_80 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_82 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_81 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_83 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );

		c_80 = _mm256_load_pd( &C2[0+bs*0] );
		c_81 = _mm256_load_pd( &C2[0+bs*1] );
		c_82 = _mm256_load_pd( &C2[0+bs*2] );
		c_83 = _mm256_load_pd( &C2[0+bs*3] );

		d_80 = _mm256_add_pd( d_80, c_80 );
		d_81 = _mm256_add_pd( d_81, c_81 );
		d_82 = _mm256_add_pd( d_82, c_82 );
		d_83 = _mm256_add_pd( d_83, c_83 );
		}
		
	// factorize
	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;


#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	d_80  = _mm256_mul_pd( d_80, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 );
	_mm256_store_pd( &D1[0+bs*0], d_40 );
	_mm256_store_pd( &D2[0+bs*0], d_80 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	d_81  = _mm256_fnmadd_pd( d_80, a_10, d_81 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	d_41  = _mm256_mul_pd( d_41, a_11 );
	d_81  = _mm256_mul_pd( d_81, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 );
	_mm256_store_pd( &D1[0+bs*1], d_41 );
	_mm256_store_pd( &D2[0+bs*1], d_81 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	d_82  = _mm256_fnmadd_pd( d_80, a_20, d_82 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	d_82  = _mm256_fnmadd_pd( d_81, a_21, d_82 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	d_82  = _mm256_mul_pd( d_82, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_store_pd( &D1[0+bs*2], d_42 );
	_mm256_store_pd( &D2[0+bs*2], d_82 );

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
	d_83  = _mm256_fnmadd_pd( d_80, a_30, d_83 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
	d_83  = _mm256_fnmadd_pd( d_81, a_31, d_83 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
	d_83  = _mm256_fnmadd_pd( d_82, a_32, d_83 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
	d_03  = _mm256_mul_pd( d_03, a_33 );
	d_43  = _mm256_mul_pd( d_43, a_33 );
	d_83  = _mm256_mul_pd( d_83, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
	_mm256_store_pd( &D1[0+bs*3], d_43 );
	_mm256_store_pd( &D2[0+bs*3], d_83 );

	}



void kernel_dlauum_dpotrf_nt_12x4_lib4_new(int kadd, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Ap2 = Ap0 + 8*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *Am2 = Am0 + 8*sdam;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;
	
	const int bs = 4;
	
	double d_temp;
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82;
	
	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_80, d_81, d_82, d_83,
		e_00, e_01, e_02, e_03;

	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

	// zero registers
	zeros = _mm256_setzero_pd();
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_80 = _mm256_setzero_pd();
	c_81 = _mm256_setzero_pd();
	c_83 = _mm256_setzero_pd();
	c_82 = _mm256_setzero_pd();

	k = 0;


	dlauum:

	if(kadd<=0)
		goto dpotrf_corr;

	// XXX assume kadd >= 12 !!!!!

	// prefetch
	a_0 = _mm256_load_pd( &Ap0[0] );
	a_4 = _mm256_load_pd( &Ap1[0] );
	a_8 = _mm256_load_pd( &Ap2[0] );
	b_0 = _mm256_load_pd( &Bp[0] );


	// triangle at the beginning

	// XXX blend A to keep lower-tr clean; no need to blend B since upper-tr is not stored

	// k = 0
	a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[4] ); // prefetch

	
	// k = 1
	a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

	a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[8] ); // prefetch


	// k = 2
	a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[12] ); // prefetch


	// k = 3
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
	a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[16] ); // prefetch


	Ap0 += 16;
	Ap1 += 16;
	Ap2 += 16;
	Bp  += 16;

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
	a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[4] ); // prefetch

		
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
	a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[8] ); // prefetch


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
	a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[12] ); // prefetch


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
	a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
	b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
	a_8  = _mm256_load_pd( &Ap2[16] ); // prefetch

	
	Ap0 += 16;
	Ap1 += 16;
	Ap2 += 16;
	Bp  += 16;


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
	a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
	a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
		
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
	a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &Bp[8] ); // prefetch
	a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch


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
	a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &Bp[12] ); // prefetch
	a_8  = _mm256_load_pd( &Ap2[12] ); // prefetch


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
	a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
	c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
	a_8  = _mm256_load_pd( &Ap2[16] ); // prefetch

	
	Ap0 += 16;
	Ap1 += 16;
	Ap2 += 16;
	Bp  += 16;

	k = 12;


	dsyrk:

	for(; k<kadd-3; k+=4)
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
		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
		a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
		
		
		
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
		a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bp[8] ); // prefetch
		a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch



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
		a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bp[12] ); // prefetch
		a_8  = _mm256_load_pd( &Ap2[12] ); // prefetch


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
		a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
		a_8  = _mm256_load_pd( &Ap2[16] ); // prefetch
		
		Ap0 += 16;
		Ap1 += 16;
		Ap2 += 16;
		Bp  += 16;

		}
	
	if(k<kadd-1)
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
		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bp[4] );
		a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
		
		
		
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
		a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bp[8] );
		a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch
			
		
		Ap0 += 8;
		Ap1 += 8;
		Ap2 += 8;
		Bp  += 8;
		k   += 2;

		}

	if(k<kadd)
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
/*		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch*/
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
/*		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch*/
		c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
//		b_0  = _mm256_load_pd( &Bp[4] );
/*		a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch*/

		}

	if(alg==-1)
		{

		long long long_sign = 0x8000000000000000;
		__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

		c_00 = _mm256_xor_pd( c_00, sign );
		c_40 = _mm256_xor_pd( c_40, sign );
		c_80 = _mm256_xor_pd( c_80, sign );
		c_01 = _mm256_xor_pd( c_01, sign );
		c_41 = _mm256_xor_pd( c_41, sign );
		c_81 = _mm256_xor_pd( c_81, sign );
		c_02 = _mm256_xor_pd( c_02, sign );
		c_42 = _mm256_xor_pd( c_42, sign );
		c_82 = _mm256_xor_pd( c_82, sign );
		c_03 = _mm256_xor_pd( c_03, sign );
		c_43 = _mm256_xor_pd( c_43, sign );
		c_83 = _mm256_xor_pd( c_83, sign );

		}


	dpotrf_corr:

	if(ksub<=0)
		goto dpotrf_blen;

	// prefetch
	a_0 = _mm256_load_pd( &Am0[0] );
	a_4 = _mm256_load_pd( &Am1[0] );
	a_8 = _mm256_load_pd( &Am2[0] );
	b_0 = _mm256_load_pd( &Bm[0] );

	for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bm[4] ); // prefetch
		a_8  = _mm256_load_pd( &Am2[4] ); // prefetch
		
		
		
		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bm[8] ); // prefetch
		a_8  = _mm256_load_pd( &Am2[8] ); // prefetch



		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bm[12] ); // prefetch
		a_8  = _mm256_load_pd( &Am2[12] ); // prefetch


		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &Bm[16] ); // prefetch
		a_8  = _mm256_load_pd( &Am2[16] ); // prefetch
		
		
		Am0 += 16;
		Am1 += 16;
		Am2 += 16;
		Bm  += 16;

		}



	dpotrf_blen:

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_80, c_81, 0xa );
	e_01 = _mm256_blend_pd( c_80, c_81, 0x5 );
	e_02 = _mm256_blend_pd( c_82, c_83, 0xa );
	e_03 = _mm256_blend_pd( c_82, c_83, 0x5 );
	
	d_80 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_82 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_81 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_83 = _mm256_blend_pd( e_01, e_03, 0x3 );

	dpotrf_fact:

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );

		c_80 = _mm256_load_pd( &C2[0+bs*0] );
		c_81 = _mm256_load_pd( &C2[0+bs*1] );
		c_82 = _mm256_load_pd( &C2[0+bs*2] );
		c_83 = _mm256_load_pd( &C2[0+bs*3] );

		d_80 = _mm256_add_pd( d_80, c_80 );
		d_81 = _mm256_add_pd( d_81, c_81 );
		d_82 = _mm256_add_pd( d_82, c_82 );
		d_83 = _mm256_add_pd( d_83, c_83 );
		}

#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	d_80  = _mm256_mul_pd( d_80, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 );
	_mm256_store_pd( &D1[0+bs*0], d_40 );
	_mm256_store_pd( &D2[0+bs*0], d_80 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	d_81  = _mm256_fnmadd_pd( d_80, a_10, d_81 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	d_41  = _mm256_mul_pd( d_41, a_11 );
	d_81  = _mm256_mul_pd( d_81, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 );
	_mm256_store_pd( &D1[0+bs*1], d_41 );
	_mm256_store_pd( &D2[0+bs*1], d_81 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	d_82  = _mm256_fnmadd_pd( d_80, a_20, d_82 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	d_82  = _mm256_fnmadd_pd( d_81, a_21, d_82 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	d_82  = _mm256_mul_pd( d_82, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_store_pd( &D1[0+bs*2], d_42 );
	_mm256_store_pd( &D2[0+bs*2], d_82 );

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
	d_83  = _mm256_fnmadd_pd( d_80, a_30, d_83 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
	d_83  = _mm256_fnmadd_pd( d_81, a_31, d_83 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
	d_83  = _mm256_fnmadd_pd( d_82, a_32, d_83 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
	d_03  = _mm256_mul_pd( d_03, a_33 );
	d_43  = _mm256_mul_pd( d_43, a_33 );
	d_83  = _mm256_mul_pd( d_83, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
	_mm256_store_pd( &D1[0+bs*3], d_43 );
	_mm256_store_pd( &D2[0+bs*3], d_83 );

	}



void kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Ap2 = Ap0 + 8*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *Am2 = Am0 + 8*sdam;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;
	
	const int bs = 4;
	
	static __m256i mask_bkp[4];
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82;
	
	__m256i
		mask0;
	
	if(km>=12)
		{
		mask0 = _mm256_set_epi64x( -1, -1, -1, -1 );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}
	else
		{
		d_temp = km-8.0;
		mask0 = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}

	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_80 = _mm256_setzero_pd();
	c_81 = _mm256_setzero_pd();
	c_83 = _mm256_setzero_pd();
	c_82 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Ap0[0] );
		a_4 = _mm256_load_pd( &Ap1[0] );
		a_8 = _mm256_load_pd( &Ap2[0] );
		b_0 = _mm256_load_pd( &Bp[0] );

		if(tri_A==1)
			{
			
			if(kadd>=4)
				{

				zeros = _mm256_setzero_pd(); // TODO use mask load instead !!!!!!!!!!

				// k = 0
				a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
				b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
				
				// k = 1
				a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
				a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
				b_0  = _mm256_load_pd( &Bp[8] ); // prefetch

				// k = 2
				a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );
				b_0  = _mm256_blend_pd( zeros, b_0, 0x7 ); // XXX
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
				b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
				c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
				a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
				b_0  = _mm256_load_pd( &Bp[12] ); // prefetch

				// k = 3
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
				b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
				c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
				a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
				b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
				a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch

				Ap0 += 16;
				Ap1 += 16;
				Ap2 += 16;
				Bp  += 16;
				k   += 4;

				if(kadd>=8)
					{

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
					a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
					b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
								
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
					a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
					b_0  = _mm256_load_pd( &Bp[8] ); // prefetch

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
					a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
					b_0  = _mm256_load_pd( &Bp[12] ); // prefetch

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
					a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
					b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
					a_8  = _mm256_load_pd( &Ap2[16] ); // prefetch
				
					Ap0 += 16;
					Ap1 += 16;
					Ap2 += 16;
					Bp  += 16;
					k  += 4;

					if(kadd>=12)
						{

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
						a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
						c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
						a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
						c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
						b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
						a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
									
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
						a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
						c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
						a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
						c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
						b_0  = _mm256_load_pd( &Bp[8] ); // prefetch
						a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch

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
						a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
						c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
						a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
						c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
						b_0  = _mm256_load_pd( &Bp[12] ); // prefetch
						a_8  = _mm256_load_pd( &Ap2[12] ); // prefetch

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
						a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
						c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
						a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
						c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
						b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
						a_8  = _mm256_load_pd( &Ap2[16] ); // prefetch
										
						Ap0 += 16;
						Ap1 += 16;
						Ap2 += 16;
						Bp  += 16;
						k  += 4;

						}
					else
						{

						if(kadd>8)
							{

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
							a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
							c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
							a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
							c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
							b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
							a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
		
							k += 1;

							if(kadd>9)
								{

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
								a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
								c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
								a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
								c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
								b_0  = _mm256_load_pd( &Bp[8] ); // prefetch
								a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch

								k += 1;

								if(kadd>10)
									{

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
									a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
									c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
									a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
									c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
									b_0  = _mm256_load_pd( &Bp[12] ); // prefetch
									a_8  = _mm256_load_pd( &Ap2[12] ); // prefetch

									k += 1;

									}
								}
							}
						}
					}
				else
					{

					if(kadd>4)
						{

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
						a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
						c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
						a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
						b_0  = _mm256_load_pd( &Bp[4] ); // prefetch

						k += 1;

						if(kadd>5)
							{

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
							a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
							c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
							a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
							b_0  = _mm256_load_pd( &Bp[8] ); // prefetch

							k += 1;

							if(kadd>6)
								{

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
								a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
								c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
								a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
								b_0  = _mm256_load_pd( &Bp[12] ); // prefetch

								k   += 1;

								}

							}

						}

					}

				}
			else
				{

				zeros  = _mm256_setzero_pd();

				// k = 0
				a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
				b_0  = _mm256_load_pd( &Bp[4] ); // prefetch

				k += 1;

				if(kadd>1)
					{

					// k = 1
					a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );
					c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
					a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
					b_0  = _mm256_load_pd( &Bp[8] ); // prefetch

					k += 1;

					if(kadd>2)
						{

						// k = 2
						a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );
						c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
						b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
						c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
						b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
						c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
						b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
						c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
						a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
						b_0  = _mm256_load_pd( &Bp[12] ); // prefetch

						k += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
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
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[4] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
			
			
			
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
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[8] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch



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
			a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[12] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[12] ); // prefetch


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
			a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[16] ); // prefetch
			a_8  = _mm256_load_pd( &Ap2[16] ); // prefetch
			
			Ap0 += 16;
			Ap1 += 16;
			Ap2 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
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
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[4] );
			a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch
			
			
			
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
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bp[8] );
			a_8  = _mm256_load_pd( &Ap2[8] ); // prefetch
				
			
			Ap0 += 8;
			Ap1 += 8;
			Ap2 += 8;
			Bp  += 8;
			k   += 2;

			}

		if(k<kadd)
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
	/*		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch*/
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	/*		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch*/
			c_82 = _mm256_fmadd_pd( a_8, b_0, c_82 );
	//		b_0  = _mm256_load_pd( &Bp[4] );
	/*		a_8  = _mm256_load_pd( &Ap2[4] ); // prefetch*/

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			c_00 = _mm256_xor_pd( c_00, sign );
			c_40 = _mm256_xor_pd( c_40, sign );
			c_80 = _mm256_xor_pd( c_80, sign );
			c_01 = _mm256_xor_pd( c_01, sign );
			c_41 = _mm256_xor_pd( c_41, sign );
			c_81 = _mm256_xor_pd( c_81, sign );
			c_02 = _mm256_xor_pd( c_02, sign );
			c_42 = _mm256_xor_pd( c_42, sign );
			c_82 = _mm256_xor_pd( c_82, sign );
			c_03 = _mm256_xor_pd( c_03, sign );
			c_43 = _mm256_xor_pd( c_43, sign );
			c_83 = _mm256_xor_pd( c_83, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am0[0] );
		a_4 = _mm256_load_pd( &Am1[0] );
		a_8 = _mm256_load_pd( &Am2[0] );
		b_0 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[4] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[4] ); // prefetch
			
			
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[8] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[8] ); // prefetch



			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[12] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[12] ); // prefetch


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
			c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
			b_0  = _mm256_load_pd( &Bm[16] ); // prefetch
			a_8  = _mm256_load_pd( &Am2[16] ); // prefetch
			
			
			Am0 += 16;
			Am1 += 16;
			Am2 += 16;
			Bm  += 16;

			}

		}

	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_80, d_81, d_82, d_83,
		e_00, e_01, e_02, e_03;

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_80, c_81, 0xa );
	e_01 = _mm256_blend_pd( c_80, c_81, 0x5 );
	e_02 = _mm256_blend_pd( c_82, c_83, 0xa );
	e_03 = _mm256_blend_pd( c_82, c_83, 0x5 );
	
	d_80 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_82 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_81 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_83 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );

		c_80 = _mm256_load_pd( &C2[0+bs*0] );
		c_81 = _mm256_load_pd( &C2[0+bs*1] );
		c_82 = _mm256_load_pd( &C2[0+bs*2] );
		c_83 = _mm256_load_pd( &C2[0+bs*3] );

		d_80 = _mm256_add_pd( d_80, c_80 );
		d_81 = _mm256_add_pd( d_81, c_81 );
		d_82 = _mm256_add_pd( d_82, c_82 );
		d_83 = _mm256_add_pd( d_83, c_83 );
		}
		
	// factorize
	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	d_80  = _mm256_mul_pd( d_80, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 );
	_mm256_store_pd( &D1[0+bs*0], d_40 );
	_mm256_maskstore_pd( &D2[0+bs*0], mask0, d_80 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	d_81  = _mm256_fnmadd_pd( d_80, a_10, d_81 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	d_41  = _mm256_mul_pd( d_41, a_11 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_81  = _mm256_mul_pd( d_81, a_11 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 );
	_mm256_store_pd( &D1[0+bs*1], d_41 );
	_mm256_maskstore_pd( &D2[0+bs*1], mask0, d_81 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	d_82  = _mm256_fnmadd_pd( d_80, a_20, d_82 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	d_82  = _mm256_fnmadd_pd( d_81, a_21, d_82 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_82  = _mm256_mul_pd( d_82, a_22 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_store_pd( &D1[0+bs*2], d_42 );
	_mm256_maskstore_pd( &D2[0+bs*2], mask0, d_82 );

	if(kn>=4)
		{
		// fourth row
		a_30  = _mm256_permute4x64_pd( d_00, 0xff );
		d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
		d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
		d_83  = _mm256_fnmadd_pd( d_80, a_30, d_83 );
		a_31  = _mm256_permute4x64_pd( d_01, 0xff );
		d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
		d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
		d_83  = _mm256_fnmadd_pd( d_81, a_31, d_83 );
		a_32  = _mm256_permute4x64_pd( d_02, 0xff );
		d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
		d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
		d_83  = _mm256_fnmadd_pd( d_82, a_32, d_83 );
		sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
		sa_33 = _mm_permute_pd( sa_33, 0x3 );
	#if defined(NO_BRANCH)
		ia_00 = _mm_castpd_si128( sa_33 );
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
		ia_00 = _mm_and_si128( ia_00, imask );
		sa_33 = _mm_div_sd( one, sa_33 );
		ia_00 = _mm_cmpeq_epi64( ia_00, imask );
		sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
	#else
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		if( _mm_comigt_sd ( sa_33, zeros_ones ) )
			{
			sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
			zeros_ones = _mm_set_sd( 1.0 );
			sa_33 = _mm_div_sd( zeros_ones, sa_33 );
			a_33  = _mm256_broadcastsd_pd( sa_33 );
			}
		else // comile
			{
			a_33 = _mm256_setzero_pd( );
			}
	#endif
		_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
		d_03  = _mm256_mul_pd( d_03, a_33 );
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		d_43  = _mm256_mul_pd( d_43, a_33 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		d_83  = _mm256_mul_pd( d_83, a_33 );
		_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
		_mm256_store_pd( &D1[0+bs*3], d_43 );
		_mm256_maskstore_pd( &D2[0+bs*3], mask0, d_83 );
		}

	}



void kernel_dpotrf_nt_8x8_lib4_new(int ksub, double *Am0, int sdam, double *Bm0, int sdbm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Am1 = Am0 + 4*sdam;
	double *Bm1 = Bm0 + 4*sdbm;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4,
		b_0, b_4,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_44, c_45, c_47, c_46;
	
	double *Bp, *Ap2;

	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_44 = _mm256_setzero_pd();
	c_45 = _mm256_setzero_pd();
	c_47 = _mm256_setzero_pd();
	c_46 = _mm256_setzero_pd();

	k = 0;

	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am0[0] );
		a_4 = _mm256_load_pd( &Am1[0] );
		b_0 = _mm256_load_pd( &Bm0[0] );
		b_4 = _mm256_load_pd( &Bm1[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[4] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[4] ); // prefetch
			
			
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[8] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[8] ); // prefetch
		


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[12] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[12] ); // prefetch
		


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[16] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[16] ); // prefetch
			


			Am0 += 16;
			Am1 += 16;
			Bm0 += 16;
			Bm1 += 16;

			}

		}

	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_44, d_45, d_46, d_47,
		e_00, e_01, e_02, e_03;

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );
		}
		
	// factorize
	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
	_mm256_store_pd( &D1[0+bs*0], d_40 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	d_41  = _mm256_mul_pd( d_41, a_11 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
	_mm256_store_pd( &D1[0+bs*1], d_41 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_store_pd( &D1[0+bs*2], d_42 );

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
	d_03  = _mm256_mul_pd( d_03, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	d_43  = _mm256_mul_pd( d_43, a_33 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
	_mm256_store_pd( &D1[0+bs*3], d_43 );


	a_4  = d_40;
	b_4  = d_40;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_41;
	b_4  = d_41;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_42;
	b_4  = d_42;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_43;
	b_4  = d_43;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );



	e_00 = _mm256_blend_pd( c_44, c_45, 0xa );
	e_01 = _mm256_blend_pd( c_44, c_45, 0x5 );
	e_02 = _mm256_blend_pd( c_46, c_47, 0xa );
	e_03 = _mm256_blend_pd( c_46, c_47, 0x5 );
	
	d_44 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_46 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_45 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_47 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_44 = _mm256_load_pd( &C1[0+bs*4] );
		c_45 = _mm256_load_pd( &C1[0+bs*5] );
		c_46 = _mm256_load_pd( &C1[0+bs*6] );
		c_47 = _mm256_load_pd( &C1[0+bs*7] );

		d_44 = _mm256_add_pd( d_44, c_44 );
		d_45 = _mm256_add_pd( d_45, c_45 );
		d_46 = _mm256_add_pd( d_46, c_46 );
		d_47 = _mm256_add_pd( d_47, c_47 );
		}
	
	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_44) );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[4], _mm256_castpd256_pd128( a_00 ) );
	d_44  = _mm256_mul_pd( d_44, a_00 );
	_mm256_store_pd( &D1[0+bs*4], d_44 ); // a_00

	// second row
	a_10  = _mm256_permute4x64_pd( d_44, 0x55 );
	d_45  = _mm256_fnmadd_pd( d_44, a_10, d_45 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_45), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[5], _mm256_castpd256_pd128( a_11 ) );
	d_45  = _mm256_mul_pd( d_45, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	_mm256_maskstore_pd( &D1[0+bs*5], mask1, d_45 ); // a_00

	// third row
	a_20  = _mm256_permute4x64_pd( d_44, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_44, a_20, d_46 );
	a_21  = _mm256_permute4x64_pd( d_45, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_45, a_21, d_46 );
	sa_22 = _mm256_extractf128_pd( d_46, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[6], _mm256_castpd256_pd128( a_22 ) );
	d_46  = _mm256_mul_pd( d_46, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	_mm256_maskstore_pd( &D1[0+bs*6], mask1, d_46 ); // a_00

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_44, 0xff );
	d_47  = _mm256_fnmadd_pd( d_44, a_30, d_47 );
	a_31  = _mm256_permute4x64_pd( d_45, 0xff );
	d_47  = _mm256_fnmadd_pd( d_45, a_31, d_47 );
	a_32  = _mm256_permute4x64_pd( d_46, 0xff );
	d_47  = _mm256_fnmadd_pd( d_46, a_32, d_47 );
	sa_33 = _mm256_extractf128_pd( d_47, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[7], _mm256_castpd256_pd128( a_33 ) );
	d_47  = _mm256_mul_pd( d_47, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	_mm256_maskstore_pd( &D1[0+bs*7], mask1, d_47 ); // a_00



	}



void kernel_dlauum_dpotrf_nt_8x8_lib4_new(int kadd, double *Ap0, int sdap, double *Bp0, int sdbp, int ksub, double *Am0, int sdam, double *Bm0, int sdbm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *Bp1 = Bp0 + 4*sdbp;
	double *Bm1 = Bm0 + 4*sdbm;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0, b_4,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_44, c_45, c_47, c_46;
	
	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_44, d_45, d_46, d_47,
		e_00, e_01, e_02, e_03;

	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

	double *Bp, *Ap2;

	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_44 = _mm256_setzero_pd();
	c_45 = _mm256_setzero_pd();
	c_47 = _mm256_setzero_pd();
	c_46 = _mm256_setzero_pd();

	k = 0;

	dlauum:

	if(kadd<=0)
		goto dpotrf_corr;

	// XXX assume kadd >= 8 !!!!!

	// prefetch
	a_0 = _mm256_load_pd( &Ap0[0] );
	a_4 = _mm256_load_pd( &Ap1[0] );
	b_0 = _mm256_load_pd( &Bp0[0] );
	b_4 = _mm256_load_pd( &Bp1[0] );

	// initial triangles

	// XXX blend A to keep lower-tr clean; no need to blend B since upper-tr is not stored

	// k = 0
	a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
	b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch


	// k = 1
	a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
	b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
	

	// k = 2
	a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
	b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch
	

	// k = 3
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
	b_0  = _mm256_load_pd( &Bp0[16] ); // prefetch
	a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
	b_4  = _mm256_load_pd( &Bp1[16] ); // prefetch
		

	Ap0 += 16;
	Ap1 += 16;
	Bp0 += 16;
	Bp1 += 16;


	// k = 4
	a_4  = _mm256_blend_pd( zeros, a_4, 0x1 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
//	c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
//	c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
//	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
	b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
		

	// k = 5
	a_4  = _mm256_blend_pd( zeros, a_4, 0x3 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
//	c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
//	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
	b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch


	// k = 6
	a_4  = _mm256_blend_pd( zeros, a_4, 0x7 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch
	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
	b_4  = _mm256_load_pd( &Bp1[12] ); // prefetch
	

	// k = 7
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

	c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

	c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &Bp0[16] ); // prefetch
	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
	b_4  = _mm256_load_pd( &Bp1[16] ); // prefetch
				

	Ap0 += 16;
	Ap1 += 16;
	Bp0 += 16;
	Bp1 += 16;

	k = 8;


	dsyrk:

	for(; k<kadd-3; k+=4)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
		b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
		
		
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
		b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
	


		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
		b_4  = _mm256_load_pd( &Bp1[12] ); // prefetch
	


		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bp0[16] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
		b_4  = _mm256_load_pd( &Bp1[16] ); // prefetch
		


		Ap0 += 16;
		Ap1 += 16;
		Bp0 += 16;
		Bp1 += 16;

		}
	
	if(k<kadd-1)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
		b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
		
		
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
		b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
	
	
		
		Ap0 += 8;
		Ap1 += 8;
		Bp0 += 8;
		Bp1 += 8;
		k   += 2;

		}

	if(k<kadd)
		{
		
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
//		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
//		b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
//		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
//		b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch

		}

	if(alg==-1)
		{

		long long long_sign = 0x8000000000000000;
		__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

		c_00 = _mm256_xor_pd( c_00, sign );
		c_40 = _mm256_xor_pd( c_40, sign );
		c_44 = _mm256_xor_pd( c_44, sign );
		c_01 = _mm256_xor_pd( c_01, sign );
		c_41 = _mm256_xor_pd( c_41, sign );
		c_45 = _mm256_xor_pd( c_45, sign );
		c_02 = _mm256_xor_pd( c_02, sign );
		c_42 = _mm256_xor_pd( c_42, sign );
		c_46 = _mm256_xor_pd( c_46, sign );
		c_03 = _mm256_xor_pd( c_03, sign );
		c_43 = _mm256_xor_pd( c_43, sign );
		c_47 = _mm256_xor_pd( c_47, sign );

		}

	dpotrf_corr:

	if(ksub<=0)
		goto dpotrf_blen;

	// prefetch
	a_0 = _mm256_load_pd( &Am0[0] );
	a_4 = _mm256_load_pd( &Am1[0] );
	b_0 = _mm256_load_pd( &Bm0[0] );
	b_4 = _mm256_load_pd( &Bm1[0] );

	for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bm0[4] ); // prefetch
		c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
		b_4  = _mm256_load_pd( &Bm1[4] ); // prefetch
		
		
		
		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bm0[8] ); // prefetch
		c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
		b_4  = _mm256_load_pd( &Bm1[8] ); // prefetch
	


		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bm0[12] ); // prefetch
		c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
		b_4  = _mm256_load_pd( &Bm1[12] ); // prefetch
	


		c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
		c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
		b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
		c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
		b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

		c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
		c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
		b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
		b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

		c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
		a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &Bm0[16] ); // prefetch
		c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
		b_4  = _mm256_load_pd( &Bm1[16] ); // prefetch
		


		Am0 += 16;
		Am1 += 16;
		Bm0 += 16;
		Bm1 += 16;

		}



	dpotrf_blen:

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	dpotrf_fact:

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );
		}
		
#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
	_mm256_store_pd( &D1[0+bs*0], d_40 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	d_41  = _mm256_mul_pd( d_41, a_11 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
	_mm256_store_pd( &D1[0+bs*1], d_41 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_store_pd( &D1[0+bs*2], d_42 );

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
	d_03  = _mm256_mul_pd( d_03, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	d_43  = _mm256_mul_pd( d_43, a_33 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
	_mm256_store_pd( &D1[0+bs*3], d_43 );


	a_4  = d_40;
	b_4  = d_40;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_41;
	b_4  = d_41;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_42;
	b_4  = d_42;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_43;
	b_4  = d_43;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );



	e_00 = _mm256_blend_pd( c_44, c_45, 0xa );
	e_01 = _mm256_blend_pd( c_44, c_45, 0x5 );
	e_02 = _mm256_blend_pd( c_46, c_47, 0xa );
	e_03 = _mm256_blend_pd( c_46, c_47, 0x5 );
	
	d_44 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_46 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_45 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_47 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_44 = _mm256_load_pd( &C1[0+bs*4] );
		c_45 = _mm256_load_pd( &C1[0+bs*5] );
		c_46 = _mm256_load_pd( &C1[0+bs*6] );
		c_47 = _mm256_load_pd( &C1[0+bs*7] );

		d_44 = _mm256_add_pd( d_44, c_44 );
		d_45 = _mm256_add_pd( d_45, c_45 );
		d_46 = _mm256_add_pd( d_46, c_46 );
		d_47 = _mm256_add_pd( d_47, c_47 );
		}
	
	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_44) );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[4], _mm256_castpd256_pd128( a_00 ) );
	d_44  = _mm256_mul_pd( d_44, a_00 );
	_mm256_store_pd( &D1[0+bs*4], d_44 ); // a_00

	// second row
	a_10  = _mm256_permute4x64_pd( d_44, 0x55 );
	d_45  = _mm256_fnmadd_pd( d_44, a_10, d_45 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_45), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[5], _mm256_castpd256_pd128( a_11 ) );
	d_45  = _mm256_mul_pd( d_45, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	_mm256_maskstore_pd( &D1[0+bs*5], mask1, d_45 ); // a_00

	// third row
	a_20  = _mm256_permute4x64_pd( d_44, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_44, a_20, d_46 );
	a_21  = _mm256_permute4x64_pd( d_45, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_45, a_21, d_46 );
	sa_22 = _mm256_extractf128_pd( d_46, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[6], _mm256_castpd256_pd128( a_22 ) );
	d_46  = _mm256_mul_pd( d_46, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	_mm256_maskstore_pd( &D1[0+bs*6], mask1, d_46 ); // a_00

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_44, 0xff );
	d_47  = _mm256_fnmadd_pd( d_44, a_30, d_47 );
	a_31  = _mm256_permute4x64_pd( d_45, 0xff );
	d_47  = _mm256_fnmadd_pd( d_45, a_31, d_47 );
	a_32  = _mm256_permute4x64_pd( d_46, 0xff );
	d_47  = _mm256_fnmadd_pd( d_46, a_32, d_47 );
	sa_33 = _mm256_extractf128_pd( d_47, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[7], _mm256_castpd256_pd128( a_33 ) );
	d_47  = _mm256_mul_pd( d_47, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	_mm256_maskstore_pd( &D1[0+bs*7], mask1, d_47 ); // a_00



	}



void kernel_dsyrk_dpotrf_nt_8x8_lib4_new(int kadd, double *Ap0, int sdap, double *Bp0, int sdbp, int ksub, double *Am0, int sdam, double *Bm0, int sdbm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *Bp1 = Bp0 + 4*sdbp;
	double *Bm1 = Bm0 + 4*sdbm;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0, b_4,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_44, c_45, c_47, c_46;
	
	double *Bp, *Ap2;

	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_44 = _mm256_setzero_pd();
	c_45 = _mm256_setzero_pd();
	c_47 = _mm256_setzero_pd();
	c_46 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Ap0[0] );
		a_4 = _mm256_load_pd( &Ap1[0] );
		b_0 = _mm256_load_pd( &Bp0[0] );
		b_4 = _mm256_load_pd( &Bp1[0] );

		for(; k<kadd-3; k+=4)
			{
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
			
			
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
		


			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[12] ); // prefetch
		


			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[16] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[16] ); // prefetch
			


			Ap0 += 16;
			Ap1 += 16;
			Bp0 += 16;
			Bp1 += 16;

			}
		
		if(k<kadd-1)
			{
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
			
			
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
		
		
			
			Ap0 += 8;
			Ap1 += 8;
			Bp0 += 8;
			Bp1 += 8;
			k   += 2;

			}

		if(k<kadd)
			{
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	//		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	//		b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	//		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
	//		b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			c_00 = _mm256_xor_pd( c_00, sign );
			c_40 = _mm256_xor_pd( c_40, sign );
			c_44 = _mm256_xor_pd( c_44, sign );
			c_01 = _mm256_xor_pd( c_01, sign );
			c_41 = _mm256_xor_pd( c_41, sign );
			c_45 = _mm256_xor_pd( c_45, sign );
			c_02 = _mm256_xor_pd( c_02, sign );
			c_42 = _mm256_xor_pd( c_42, sign );
			c_46 = _mm256_xor_pd( c_46, sign );
			c_03 = _mm256_xor_pd( c_03, sign );
			c_43 = _mm256_xor_pd( c_43, sign );
			c_47 = _mm256_xor_pd( c_47, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am0[0] );
		a_4 = _mm256_load_pd( &Am1[0] );
		b_0 = _mm256_load_pd( &Bm0[0] );
		b_4 = _mm256_load_pd( &Bm1[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[4] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[4] ); // prefetch
			
			
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[8] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[8] ); // prefetch
		


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[12] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[12] ); // prefetch
		


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[16] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[16] ); // prefetch
			


			Am0 += 16;
			Am1 += 16;
			Bm0 += 16;
			Bm1 += 16;

			}

		}

	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_44, d_45, d_46, d_47,
		e_00, e_01, e_02, e_03;

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );
		}
		
	// factorize
	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
	_mm256_store_pd( &D1[0+bs*0], d_40 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	d_41  = _mm256_mul_pd( d_41, a_11 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
	_mm256_store_pd( &D1[0+bs*1], d_41 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_store_pd( &D1[0+bs*2], d_42 );

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
	d_03  = _mm256_mul_pd( d_03, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	d_43  = _mm256_mul_pd( d_43, a_33 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
	_mm256_store_pd( &D1[0+bs*3], d_43 );


	a_4  = d_40;
	b_4  = d_40;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_41;
	b_4  = d_41;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_42;
	b_4  = d_42;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_43;
	b_4  = d_43;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );



	e_00 = _mm256_blend_pd( c_44, c_45, 0xa );
	e_01 = _mm256_blend_pd( c_44, c_45, 0x5 );
	e_02 = _mm256_blend_pd( c_46, c_47, 0xa );
	e_03 = _mm256_blend_pd( c_46, c_47, 0x5 );
	
	d_44 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_46 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_45 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_47 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_44 = _mm256_load_pd( &C1[0+bs*4] );
		c_45 = _mm256_load_pd( &C1[0+bs*5] );
		c_46 = _mm256_load_pd( &C1[0+bs*6] );
		c_47 = _mm256_load_pd( &C1[0+bs*7] );

		d_44 = _mm256_add_pd( d_44, c_44 );
		d_45 = _mm256_add_pd( d_45, c_45 );
		d_46 = _mm256_add_pd( d_46, c_46 );
		d_47 = _mm256_add_pd( d_47, c_47 );
		}
	
	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_44) );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[4], _mm256_castpd256_pd128( a_00 ) );
	d_44  = _mm256_mul_pd( d_44, a_00 );
	_mm256_store_pd( &D1[0+bs*4], d_44 ); // a_00

	// second row
	a_10  = _mm256_permute4x64_pd( d_44, 0x55 );
	d_45  = _mm256_fnmadd_pd( d_44, a_10, d_45 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_45), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[5], _mm256_castpd256_pd128( a_11 ) );
	d_45  = _mm256_mul_pd( d_45, a_11 );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	_mm256_maskstore_pd( &D1[0+bs*5], mask1, d_45 ); // a_00

	// third row
	a_20  = _mm256_permute4x64_pd( d_44, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_44, a_20, d_46 );
	a_21  = _mm256_permute4x64_pd( d_45, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_45, a_21, d_46 );
	sa_22 = _mm256_extractf128_pd( d_46, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[6], _mm256_castpd256_pd128( a_22 ) );
	d_46  = _mm256_mul_pd( d_46, a_22 );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	_mm256_maskstore_pd( &D1[0+bs*6], mask1, d_46 ); // a_00

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_44, 0xff );
	d_47  = _mm256_fnmadd_pd( d_44, a_30, d_47 );
	a_31  = _mm256_permute4x64_pd( d_45, 0xff );
	d_47  = _mm256_fnmadd_pd( d_45, a_31, d_47 );
	a_32  = _mm256_permute4x64_pd( d_46, 0xff );
	d_47  = _mm256_fnmadd_pd( d_46, a_32, d_47 );
	sa_33 = _mm256_extractf128_pd( d_47, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[7], _mm256_castpd256_pd128( a_33 ) );
	d_47  = _mm256_mul_pd( d_47, a_33 );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	_mm256_maskstore_pd( &D1[0+bs*7], mask1, d_47 ); // a_00



	}



void kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap0, int sdap, double *Bp0, int sdbp, int ksub, double *Am0, int sdam, double *Bm0, int sdbm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *Bp1 = Bp0 + 4*sdbp;
	double *Bm1 = Bm0 + 4*sdbm;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	static __m256i mask_bkp[4];
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		zeros,
		a_0, a_4, a_8,
		b_0, b_4,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_44, c_45, c_47, c_46;
	
	__m256i
		mask0;

	if(km>=8)
		{
		mask0 = _mm256_set_epi64x( -1, -1, -1, -1 );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}
	else
		{
		d_temp = km-4.0;
		mask0 = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}

	double *Bp, *Ap2;

	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();
	c_44 = _mm256_setzero_pd();
	c_45 = _mm256_setzero_pd();
	c_47 = _mm256_setzero_pd();
	c_46 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Ap0[0] );
		a_4 = _mm256_load_pd( &Ap1[0] );
		b_0 = _mm256_load_pd( &Bp0[0] );
		b_4 = _mm256_load_pd( &Bp1[0] );

		if(tri_A==1)
			{
			
			if(kadd>=4)
			// TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
				{

				zeros = _mm256_setzero_pd(); // TODO use mask load instead !!!!!!!!!!

				// k = 0
				a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
				b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
				
				// k = 1
				a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
				a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
				b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch

				// k = 2
				a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );
				b_0  = _mm256_blend_pd( zeros, b_0, 0x7 ); // XXX
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
				b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
				c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
				a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
				b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch

				// k = 3
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
				b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
				c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
				b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
				a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
				b_0  = _mm256_load_pd( &Bp0[16] ); // prefetch
				a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
				b_4  = _mm256_load_pd( &Bp1[16] ); // prefetch

				Ap0 += 16;
				Ap1 += 16;
				Bp0 += 16;
				Bp1 += 16;
				k   += 4;

				if(kadd>=8)
					{

					// k = 4
					a_4  = _mm256_blend_pd( zeros, a_4, 0x1 );

					c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
					c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
					c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
					b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
//					c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
					b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

					c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
					c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
//					c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
					a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
//					c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
					a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
					b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
									

					// k = 5
					a_4  = _mm256_blend_pd( zeros, a_4, 0x3 );
	
					c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
					c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
					c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
					b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
					c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
					b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

					c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
					c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
//					c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
					a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
//					c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
					a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
					b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
	

					// k = 6
					a_4  = _mm256_blend_pd( zeros, a_4, 0x7 );
					b_4  = _mm256_blend_pd( zeros, b_4, 0x7 ); // XXX

					c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
					c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
					c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
					b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
					c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
					b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

					c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
					c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
					a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch
					c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
					a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
					b_4  = _mm256_load_pd( &Bp1[12] ); // prefetch
	

					// k = 7
					c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
					c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
					c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
					b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
					c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
					b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

					c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
					c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
					b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

					c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
					a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
					c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
					b_0  = _mm256_load_pd( &Bp0[16] ); // prefetch
					c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
					a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
					b_4  = _mm256_load_pd( &Bp1[16] ); // prefetch
					

					Ap0 += 16;
					Ap1 += 16;
					Bp0 += 16;
					Bp1 += 16;
					k  += 4;

					}
				else
					{

					if(kadd>4)
						{

						// k = 4
						a_4  = _mm256_blend_pd( zeros, a_4, 0x1 );

						c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
						c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
						b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
						c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
						b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

						c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
						c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
						b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	//					c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
						b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

						c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
						c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
						b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	//					c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
						b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

						c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
						a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
						c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
						b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
	//					c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
						a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
						b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
	
						k += 1;

						if(kadd>5)
							{

							// k = 5
							a_4  = _mm256_blend_pd( zeros, a_4, 0x3 );
			
							c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
							c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
							b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
							c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
							b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

							c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
							c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
							b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
							c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
							b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

							c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
							c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
							b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		//					c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
							b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

							c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
							a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
							c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
							b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
		//					c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
							a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
							b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
	
							k += 1;

							if(kadd>6)
								{

								// k = 6
								a_4  = _mm256_blend_pd( zeros, a_4, 0x7 );
								b_4  = _mm256_blend_pd( zeros, b_4, 0x7 ); // XXX

								c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
								c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
								b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
								c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
								b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

								c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
								c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
								b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
								c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
								b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

								c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
								c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
								b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
								c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
								b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

								c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
								a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
								c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
								b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch
								c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
								a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
								b_4  = _mm256_load_pd( &Bp1[12] ); // prefetch
	
								k += 1;

								}

							}

						}

					}

				}
			else
				{

				zeros  = _mm256_setzero_pd();

				// k = 0
				a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
				b_0  = _mm256_load_pd( &Bp[4] ); // prefetch

				k += 1;

				if(kadd>1)
					{

					// k = 1
					a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );
					c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
					b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
					a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
					b_0  = _mm256_load_pd( &Bp[8] ); // prefetch

					k += 1;

					if(kadd>2)
						{

						// k = 2
						a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );
						b_0  = _mm256_blend_pd( zeros, b_0, 0x7 ); // XXX
						c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
						b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
						c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
						b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
						c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
						b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
						c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
						a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
						b_0  = _mm256_load_pd( &Bp[12] ); // prefetch

						k += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
			
			
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
		


			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[12] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[12] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[12] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[12] ); // prefetch
		


			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[16] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[16] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[16] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[16] ); // prefetch
			


			Ap0 += 16;
			Ap1 += 16;
			Bp0 += 16;
			Bp1 += 16;

			}
		
		if(k<kadd-1)
			{
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch
			
			
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bp0[8] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Ap1[8] ); // prefetch
			b_4  = _mm256_load_pd( &Bp1[8] ); // prefetch
		
		
			
			Ap0 += 8;
			Ap1 += 8;
			Bp0 += 8;
			Bp1 += 8;
			k   += 2;

			}

		if(k<kadd)
			{
			
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fmadd_pd( a_0, b_0, c_02 );
	//		a_0  = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	//		b_0  = _mm256_load_pd( &Bp0[4] ); // prefetch
			c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	//		a_4  = _mm256_load_pd( &Ap1[4] ); // prefetch
	//		b_4  = _mm256_load_pd( &Bp1[4] ); // prefetch

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			c_00 = _mm256_xor_pd( c_00, sign );
			c_40 = _mm256_xor_pd( c_40, sign );
			c_44 = _mm256_xor_pd( c_44, sign );
			c_01 = _mm256_xor_pd( c_01, sign );
			c_41 = _mm256_xor_pd( c_41, sign );
			c_45 = _mm256_xor_pd( c_45, sign );
			c_02 = _mm256_xor_pd( c_02, sign );
			c_42 = _mm256_xor_pd( c_42, sign );
			c_46 = _mm256_xor_pd( c_46, sign );
			c_03 = _mm256_xor_pd( c_03, sign );
			c_43 = _mm256_xor_pd( c_43, sign );
			c_47 = _mm256_xor_pd( c_47, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am0[0] );
		a_4 = _mm256_load_pd( &Am1[0] );
		b_0 = _mm256_load_pd( &Bm0[0] );
		b_4 = _mm256_load_pd( &Bm1[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[4] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[4] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[4] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[4] ); // prefetch
			
			
			
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[8] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[8] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[8] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[8] ); // prefetch
		


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[12] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[12] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[12] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[12] ); // prefetch
		


			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
			c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
			b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
			c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );
			b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );

			c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
			c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
			b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );
			b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );

			c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
			a_0  = _mm256_load_pd( &Am0[16] ); // prefetch
			c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
			b_0  = _mm256_load_pd( &Bm0[16] ); // prefetch
			c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );
			a_4  = _mm256_load_pd( &Am1[16] ); // prefetch
			b_4  = _mm256_load_pd( &Bm1[16] ); // prefetch
			


			Am0 += 16;
			Am1 += 16;
			Bm0 += 16;
			Bm1 += 16;

			}

		}

	__m256d
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_44, d_45, d_46, d_47,
		e_00, e_01, e_02, e_03;

	e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	d_00 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_01 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

	e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
	e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
	e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
	e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
	
	d_40 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_41 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );

		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );

		d_40 = _mm256_add_pd( d_40, c_40 );
		d_41 = _mm256_add_pd( d_41, c_41 );
		d_42 = _mm256_add_pd( d_42, c_42 );
		d_43 = _mm256_add_pd( d_43, c_43 );
		}
		
	// factorize
	__m128i
		ia_00, imask;
		
	__m128
		ssa_00, szeros_ones;

	__m128d
		zero, one,
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

#if defined(NO_BRANCH)
	zero  = _mm_setzero_pd();
	one   = _mm_set_sd ( 1.0 );
#endif

	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
	d_00  = _mm256_mul_pd( d_00, a_00 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_40  = _mm256_mul_pd( d_40, a_00 );
	_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
	_mm256_maskstore_pd( &D1[0+bs*0], mask0, d_40 );

	// second row
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 );
	d_01  = _mm256_mul_pd( d_01, a_11 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_41  = _mm256_mul_pd( d_41, a_11 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
	_mm256_maskstore_pd( &D1[0+bs*1], mask0, d_41 );

	// third row
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	d_02  = _mm256_mul_pd( d_02, a_22 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_42  = _mm256_mul_pd( d_42, a_22 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
	_mm256_maskstore_pd( &D1[0+bs*2], mask0, d_42 );

	// fourth row
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_33 );
	sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_33 = _mm_div_sd( one, sa_33 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
	a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		}
	else // comile
		{
		a_33 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
	mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
	d_03  = _mm256_mul_pd( d_03, a_33 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_43  = _mm256_mul_pd( d_43, a_33 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
	_mm256_maskstore_pd( &D1[0+bs*3], mask0, d_43 );


	a_4  = d_40;
	b_4  = d_40;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_41;
	b_4  = d_41;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_42;
	b_4  = d_42;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );


	a_4  = d_43;
	b_4  = d_43;
	c_44 = _mm256_fnmadd_pd( a_4, b_4, c_44 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_45 = _mm256_fnmadd_pd( a_4, b_4, c_45 );

	b_4  = _mm256_permute2f128_pd( b_4, b_4, 0x1 );
	c_47 = _mm256_fnmadd_pd( a_4, b_4, c_47 );

	b_4  = _mm256_shuffle_pd( b_4, b_4, 0x5 );
	c_46 = _mm256_fnmadd_pd( a_4, b_4, c_46 );



	e_00 = _mm256_blend_pd( c_44, c_45, 0xa );
	e_01 = _mm256_blend_pd( c_44, c_45, 0x5 );
	e_02 = _mm256_blend_pd( c_46, c_47, 0xa );
	e_03 = _mm256_blend_pd( c_46, c_47, 0x5 );
	
	d_44 = _mm256_blend_pd( e_00, e_02, 0xc );
	d_46 = _mm256_blend_pd( e_00, e_02, 0x3 );
	d_45 = _mm256_blend_pd( e_01, e_03, 0xc );
	d_47 = _mm256_blend_pd( e_01, e_03, 0x3 );

	if(alg!=0)
		{
		c_44 = _mm256_load_pd( &C1[0+bs*4] );
		c_45 = _mm256_load_pd( &C1[0+bs*5] );
		c_46 = _mm256_load_pd( &C1[0+bs*6] );
		c_47 = _mm256_load_pd( &C1[0+bs*7] );

		d_44 = _mm256_add_pd( d_44, c_44 );
		d_45 = _mm256_add_pd( d_45, c_45 );
		d_46 = _mm256_add_pd( d_46, c_46 );
		d_47 = _mm256_add_pd( d_47, c_47 );
		}
	
	// first row
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_44) );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_00 );
	sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_00 = _mm_div_sd( one, sa_00 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_00 = _mm_blendv_pd( sa_00, zero, _mm_castsi128_pd( ia_00 ) );
	a_00  = _mm256_broadcastsd_pd( sa_00 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[4], _mm256_castpd256_pd128( a_00 ) );
	mask0 = _mm256_loadu_si256( mask_bkp );
	d_44  = _mm256_mul_pd( d_44, a_00 );
	_mm256_maskstore_pd( &D1[0+bs*4], mask0, d_44 ); // a_00

	// second row
	a_10  = _mm256_permute4x64_pd( d_44, 0x55 );
	d_45  = _mm256_fnmadd_pd( d_44, a_10, d_45 );
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_45), 0x3 );
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_11 );
	sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_11 = _mm_div_sd( one, sa_11 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_11 = _mm_blendv_pd( sa_11, zero, _mm_castsi128_pd( ia_00 ) );
	a_11  = _mm256_broadcastsd_pd( sa_11 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		}
	else // comile
		{
		a_11 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[5], _mm256_castpd256_pd128( a_11 ) );
	mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
	mask0 = _mm256_loadu_si256( mask_bkp );
	mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
	d_45  = _mm256_mul_pd( d_45, a_11 );
	_mm256_maskstore_pd( &D1[0+bs*5], mask0, d_45 ); // a_00

	// third row
	a_20  = _mm256_permute4x64_pd( d_44, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_44, a_20, d_46 );
	a_21  = _mm256_permute4x64_pd( d_45, 0xaa );
	d_46  = _mm256_fnmadd_pd( d_45, a_21, d_46 );
	sa_22 = _mm256_extractf128_pd( d_46, 0x1 ); // a_22 & a_32
#if defined(NO_BRANCH)
	ia_00 = _mm_castpd_si128( sa_22 );
	sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
	imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
	ia_00 = _mm_and_si128( ia_00, imask );
	sa_22 = _mm_div_sd( one, sa_22 );
	ia_00 = _mm_cmpeq_epi64( ia_00, imask );
	sa_22 = _mm_blendv_pd( sa_22, zero, _mm_castsi128_pd( ia_00 ) );
	a_22  = _mm256_broadcastsd_pd( sa_22 );
#else
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		}
	else // comile
		{
		a_22 = _mm256_setzero_pd( );
		}
#endif
	_mm_store_sd( &inv_diag_D[6], _mm256_castpd256_pd128( a_22 ) );
	mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
	mask0 = _mm256_loadu_si256( mask_bkp );
	mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
	d_46  = _mm256_mul_pd( d_46, a_22 );
	_mm256_maskstore_pd( &D1[0+bs*6], mask0, d_46 ); // a_00

	if(kn>=8)
		{
		// fourth row
		a_30  = _mm256_permute4x64_pd( d_44, 0xff );
		d_47  = _mm256_fnmadd_pd( d_44, a_30, d_47 );
		a_31  = _mm256_permute4x64_pd( d_45, 0xff );
		d_47  = _mm256_fnmadd_pd( d_45, a_31, d_47 );
		a_32  = _mm256_permute4x64_pd( d_46, 0xff );
		d_47  = _mm256_fnmadd_pd( d_46, a_32, d_47 );
		sa_33 = _mm256_extractf128_pd( d_47, 0x1 ); // a_33
		sa_33 = _mm_permute_pd( sa_33, 0x3 );
#if defined(NO_BRANCH)
		ia_00 = _mm_castpd_si128( sa_33 );
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		imask = _mm_set_epi64x( 0x7ff0000000000000, 0x7ff0000000000000 );
		ia_00 = _mm_and_si128( ia_00, imask );
		sa_33 = _mm_div_sd( one, sa_33 );
		ia_00 = _mm_cmpeq_epi64( ia_00, imask );
		sa_33 = _mm_blendv_pd( sa_33, zero, _mm_castsi128_pd( ia_00 ) );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
#else
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		if( _mm_comigt_sd ( sa_33, zeros_ones ) )
			{
			sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
			zeros_ones = _mm_set_sd( 1.0 );
			sa_33 = _mm_div_sd( zeros_ones, sa_33 );
			a_33  = _mm256_broadcastsd_pd( sa_33 );
			}
		else // comile
			{
			a_33 = _mm256_setzero_pd( );
			}
#endif
		_mm_store_sd( &inv_diag_D[7], _mm256_castpd256_pd128( a_33 ) );
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		d_47  = _mm256_mul_pd( d_47, a_33 );
		_mm256_maskstore_pd( &D1[0+bs*7], mask0, d_47 ); // a_00

		}

	}



void kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *C1  = C0  + 4*sdc;
	double *D1  = D0  + 4*sdd;
	
	const int bs = 4;
	
	static __m256i mask_bkp[4];
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, //A_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71;
	
	__m256i
		mask0;

	if(km>=8)
		{
		mask0 = _mm256_set_epi64x( -1, -1, -1, -1 );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}
	else
		{
		d_temp = km-4.0;
		mask0 = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}

		// zero registers
	zeros = _mm256_setzero_pd();
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();
	c_40_51_62_73 = _mm256_setzero_pd();
	c_41_50_63_72 = _mm256_setzero_pd();
	c_43_52_61_70 = _mm256_setzero_pd();
	c_42_53_60_71 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0123        = _mm256_load_pd( &Ap0[0] );
		a_4567        = _mm256_load_pd( &Ap1[0] );
		b_0123        = _mm256_load_pd( &Bp[0] );

		if(tri_A==1)
			{
			
			if(kadd>=4)
				{

				// k = 0
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
				c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
				b_0123        = _mm256_load_pd( &Bp[4] ); // prefetch
				a_0123        = _mm256_load_pd( &Ap0[4] ); // prefetch

				// k = 1
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
				c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
				b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123        = _mm256_load_pd( &Bp[8] ); // prefetch
				c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
				a_0123        = _mm256_load_pd( &Ap0[8] ); // prefetch

				// k = 2
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
//				b_0123        = _mm256_blend_pd( ab_tmp1, b_0123, 0x7 ); // XXX
				c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
				b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123        = _mm256_load_pd( &Bp[12] ); // prefetch
				c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
				b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
				b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
				a_0123        = _mm256_load_pd( &Ap0[12] ); // prefetch

				// k = 3
				c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
				b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123        = _mm256_load_pd( &Bp[16] ); // prefetch
				c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
				b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
				b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
				a_0123        = _mm256_load_pd( &Ap0[16] ); // prefetch
				a_4567        = _mm256_load_pd( &Ap1[16] ); // prefetch
				
				Ap0 += 16;
				Ap1 += 16;
				Bp  += 16;
				k   += 4;

				if(kadd>=8)
					{

					// k = 4
					a_4567        = _mm256_blend_pd( zeros, a_4567, 0x1 );
					c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
					b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
					b_0123        = _mm256_load_pd( &Bp[4] ); // prefetch
					c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
					b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
					c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
					b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
					c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
					a_0123        = _mm256_load_pd( &Ap0[4] ); // prefetch
					c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
					a_4567        = _mm256_load_pd( &Ap1[4] ); // prefetch
					
					// k = 5
					a_4567        = _mm256_blend_pd( zeros, a_4567, 0x3 );
					c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
					b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
					b_0123        = _mm256_load_pd( &Bp[8] ); // prefetch
					c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
					b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
					c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
					b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
					c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
					a_0123        = _mm256_load_pd( &Ap0[8] ); // prefetch
					c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
					a_4567        = _mm256_load_pd( &Ap1[8] ); // prefetch

					// k = 6
					a_4567        = _mm256_blend_pd( zeros, a_4567, 0x7 );
					c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
					b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
					b_0123        = _mm256_load_pd( &Bp[12] ); // prefetch
					c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
					b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
					c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
					b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
					c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
					a_0123        = _mm256_load_pd( &Ap0[12] ); // prefetch
					c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
					a_4567        = _mm256_load_pd( &Ap1[12] ); // prefetch

					// k = 7
					c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
					b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
					b_0123        = _mm256_load_pd( &Bp[16] ); // prefetch
					c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
					b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
					c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
					b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
					c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
					a_0123        = _mm256_load_pd( &Ap0[16] ); // prefetch
					c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
					a_4567        = _mm256_load_pd( &Ap1[16] ); // prefetch
						
					Ap0 += 16;
					Ap1 += 16;
					Bp  += 16;
					k   += 4;

					}
				else
					{

					if(kadd>4)
						{

						// k = 4
						a_4567        = _mm256_blend_pd( zeros, a_4567, 0x1 );
						c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
						b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
						c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
						b_0123        = _mm256_load_pd( &Bp[4] ); // prefetch
						c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
						b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
						c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
						c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
						b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
						c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
						c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
						a_0123        = _mm256_load_pd( &Ap0[4] ); // prefetch
						c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
						a_4567        = _mm256_load_pd( &Ap1[4] ); // prefetch

						k += 1;

						if(kadd>5)
							{

							// k = 5
							a_4567        = _mm256_blend_pd( zeros, a_4567, 0x3 );
							c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
							b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
							c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
							b_0123        = _mm256_load_pd( &Bp[8] ); // prefetch
							c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
							b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
							c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
							c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
							b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
							c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
							c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
							a_0123        = _mm256_load_pd( &Ap0[8] ); // prefetch
							c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
							a_4567        = _mm256_load_pd( &Ap1[8] ); // prefetch

							k += 1;

							if(kadd>6)
								{

								// k = 6
								a_4567        = _mm256_blend_pd( zeros, a_4567, 0x7 );
								c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
								b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
								c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
								b_0123        = _mm256_load_pd( &Bp[12] ); // prefetch
								c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
								b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
								c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
								c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
								b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
								c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
								c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
								a_0123        = _mm256_load_pd( &Ap0[12] ); // prefetch
								c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
								a_4567        = _mm256_load_pd( &Ap1[12] ); // prefetch

								k   += 1;

								}

							}

						}

					}

				}
			else
				{

				// k = 0
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
				c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
				b_0123        = _mm256_load_pd( &Bp[4] ); // prefetch
				a_0123        = _mm256_load_pd( &Ap0[4] ); // prefetch

				k += 1;

				if(kadd>1)
					{

					// k = 1
					a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
					b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
					b_0123        = _mm256_load_pd( &Bp[8] ); // prefetch
					c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
					a_0123        = _mm256_load_pd( &Ap0[8] ); // prefetch

					k += 1;

					if(kadd>2)
						{

						// k = 2
						a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
						b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
						c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
						b_0123        = _mm256_load_pd( &Bp[12] ); // prefetch
						c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
						b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
						c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
						b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
						a_0123        = _mm256_load_pd( &Ap0[12] ); // prefetch
						c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );

						k += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bp[4] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Ap1[4] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bp[8] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Ap1[8] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bp[12] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Ap0[12] ); // prefetch
			c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Ap1[12] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bp[16] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Ap0[16] ); // prefetch
			c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Ap1[16] ); // prefetch
			
			Ap0 += 16;
			Ap1 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
			{
			
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bp[4] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Ap0[4] ); // prefetch
			c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Ap1[4] ); // prefetch
			
			
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bp[8] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Ap0[8] ); // prefetch
			c_42_53_60_71 = _mm256_fmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Ap1[8] ); // prefetch
			
			
			Ap0 += 8;
			Ap1 += 8;
			Bp  += 8;
			k   += 2;

			}

		if(k<kadd)
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
			
//			A0 += 4; // keep it !!!
//			A1 += 4; // keep it !!!
//			B  += 4; // keep it !!!

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			c_00_11_22_33 = _mm256_xor_pd( c_00_11_22_33, sign );
			c_40_51_62_73 = _mm256_xor_pd( c_40_51_62_73, sign );
			c_01_10_23_32 = _mm256_xor_pd( c_01_10_23_32, sign );
			c_41_50_63_72 = _mm256_xor_pd( c_41_50_63_72, sign );
			c_02_13_20_31 = _mm256_xor_pd( c_02_13_20_31, sign );
			c_42_53_60_71 = _mm256_xor_pd( c_42_53_60_71, sign );
			c_03_12_21_30 = _mm256_xor_pd( c_03_12_21_30, sign );
			c_43_52_61_70 = _mm256_xor_pd( c_43_52_61_70, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0123        = _mm256_load_pd( &Am0[0] );
		a_4567        = _mm256_load_pd( &Am1[0] );
		b_0123        = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bm[4] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Am0[4] ); // prefetch
			c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Am1[4] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bm[8] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Am0[8] ); // prefetch
			c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Am1[8] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bm[12] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Am0[12] ); // prefetch
			c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Am1[12] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
			c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
			b_0123        = _mm256_load_pd( &Bm[16] ); // prefetch
			b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
			c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
			b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
			c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
			c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
			a_0123        = _mm256_load_pd( &Am0[16] ); // prefetch
			c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
			a_4567        = _mm256_load_pd( &Am1[16] ); // prefetch
			
			Am0 += 16;
			Am1 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0, e_1, e_2, e_3,
		c_00, c_01, c_02, c_03,
		c_40, c_41, c_42, c_43,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43;

	e_0 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	e_1 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	e_2 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	e_3 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );

	d_00 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_02 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_01 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_03 = _mm256_blend_pd( e_1, e_3, 0x3 );

	e_0 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
	e_1 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
	e_2 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
	e_3 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
	
	d_40 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_42 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_41 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_43 = _mm256_blend_pd( e_1, e_3, 0x3 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C0[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, c_00 );
		c_02 = _mm256_load_pd( &C0[0+bs*2] );
		d_02 = _mm256_add_pd( d_02, c_02 );
		c_01 = _mm256_load_pd( &C0[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, c_01 );
		c_03 = _mm256_load_pd( &C0[0+bs*3] );
		d_03 = _mm256_add_pd( d_03, c_03 );
		c_40 = _mm256_load_pd( &C1[0+bs*0] );
		d_40 = _mm256_add_pd( d_40, c_40 );
		c_42 = _mm256_load_pd( &C1[0+bs*2] );
		d_42 = _mm256_add_pd( d_42, c_42 );
		c_41 = _mm256_load_pd( &C1[0+bs*1] );
		d_41 = _mm256_add_pd( d_41, c_41 );
		c_43 = _mm256_load_pd( &C1[0+bs*3] );
		d_43 = _mm256_add_pd( d_43, c_43 );
		}
		

	// factorize
	__m128
		ssa_00, szeros_ones;

	__m128d
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

	// first row
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		//sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		//a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
		d_40 = _mm256_mul_pd( d_40, a_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], mask0, d_40 );
		}
	else // comile
		{
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_00 = _mm256_setzero_pd( ); // zero a_00 in d, continue factorizing with 1
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) ); // store 0 in fact
		_mm256_store_pd( &D0[0+bs*0], a_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], mask0, a_00 );
		}

	// second row
	//sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	//_mm_store_sd( &fact[1], sa_10 );
	//a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	d_41  = _mm256_fnmadd_pd( d_40, a_10, d_41 );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
		//sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		//a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
		d_41 = _mm256_mul_pd( d_41, a_11 );
		_mm256_maskstore_pd( &D1[0+bs*1], mask0, d_41 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_11 = _mm256_setzero_pd( ); // zero a_00 in d, continue factorizing with 1
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) ); // store 0 in fact
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, a_11 );
		_mm256_maskstore_pd( &D1[0+bs*1], mask0, a_11 );
		}

	// third row
	//sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	//sa_20 = _mm_permute_pd( sa_20, 0x0 );
	//_mm_store_sd( &fact[3], sa_20 );
	//a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	d_42  = _mm256_fnmadd_pd( d_40, a_20, d_42 );
	//sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	//sa_21 = _mm_permute_pd( sa_21, 0x0 );
	//_mm_store_sd( &fact[4], sa_21 );
	//a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	d_42  = _mm256_fnmadd_pd( d_41, a_21, d_42 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		//sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		//a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
		d_42 = _mm256_mul_pd( d_42, a_22 );
		_mm256_maskstore_pd( &D1[0+bs*2], mask0, d_42 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_22 = _mm256_setzero_pd( ); // zero a_00 in d, continue factorizing with 1
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) ); // store 0 in fact
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, a_22 );
		_mm256_maskstore_pd( &D1[0+bs*2], mask0, a_22 );
		}

	if(kn>=4)
		{
		// fourth row
		//sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
		//sa_30 = _mm_permute_pd( sa_30, 0x3 );
		//_mm_store_sd( &fact[6], sa_30 );
		//a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
		a_30  = _mm256_permute4x64_pd( d_00, 0xff );
		d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
		d_43  = _mm256_fnmadd_pd( d_40, a_30, d_43 );
		//sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
		//sa_31 = _mm_permute_pd( sa_31, 0x3 );
		//_mm_store_sd( &fact[7], sa_31 );
		//a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
		a_31  = _mm256_permute4x64_pd( d_01, 0xff );
		d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
		d_43  = _mm256_fnmadd_pd( d_41, a_31, d_43 );
		//sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
		//sa_32 = _mm_permute_pd( sa_32, 0x3 );
		//_mm_store_sd( &fact[8], sa_32 );
		//a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
		a_32  = _mm256_permute4x64_pd( d_02, 0xff );
		d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
		d_43  = _mm256_fnmadd_pd( d_42, a_32, d_43 );
		sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
		sa_33 = _mm_permute_pd( sa_33, 0x3 );
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		if( _mm_comigt_sd ( sa_33, zeros_ones ) )
			{
			sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
			zeros_ones = _mm_set_sd( 1.0 );
			sa_33 = _mm_div_sd( zeros_ones, sa_33 );
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			//sa_33 = _mm_movedup_pd( sa_33 );
			_mm_store_sd( &inv_diag_D[3], sa_33 );
			//a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
			a_33  = _mm256_broadcastsd_pd( sa_33 );
			d_03  = _mm256_mul_pd( d_03, a_33 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
			d_43 = _mm256_mul_pd( d_43, a_33 );
			_mm256_maskstore_pd( &D1[0+bs*3], mask0, d_43 );
			}
		else // comile
			{
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			a_33 = _mm256_setzero_pd( ); // zero a_00 in d, continue factorizing with 1
			_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) ); // store 0 in fact
			//d_03  = _mm256_blend_pd( d_03, a_33, 0xf );
			//_mm256_maskstore_pd( &D0[0+bs*3], mask, d_03 );
			_mm256_maskstore_pd( &D0[0+bs*3], mask1, a_33 );
			_mm256_maskstore_pd( &D1[0+bs*3], mask0, a_33 ); // a_00
			}

		}

	}



void kernel_dpotrf_nt_4x4_lib4_new(int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{
	
	const int bs = 4;
	
	double d_temp;
	
	int k;
	
	__m256d
		zeros, ab_temp,
		a_0, A_0,
		b_0, B_0, b_1, b_2, B_2, b_3,
		c_00, c_01, c_03, c_02,
		C_00, C_01, C_03, C_02;
	
	__m256i
		mask0;

		// zero registers
	zeros = _mm256_setzero_pd();
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	C_00 = _mm256_setzero_pd();
	C_01 = _mm256_setzero_pd();
	C_03 = _mm256_setzero_pd();
	C_02 = _mm256_setzero_pd();

	k = 0;

	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am[0] );
		b_0 = _mm256_broadcast_pd( (__m128d *) &Bm[0] );
		b_2 = _mm256_broadcast_pd( (__m128d *) &Bm[2] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			B_0  = _mm256_broadcast_pd( (__m128d *) &Bm[4] ); // prefetch
			A_0  = _mm256_load_pd( &Am[4] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fnmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fnmadd_pd( a_0, b_3, c_03 );
			B_2  = _mm256_broadcast_pd( (__m128d *) &Bm[6] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			b_0  = _mm256_broadcast_pd( (__m128d *) &Bm[8] ); // prefetch
			a_0  = _mm256_load_pd( &Am[8] ); // prefetch
			b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
			C_00 = _mm256_fnmadd_pd( A_0, B_0, C_00 );
			C_01 = _mm256_fnmadd_pd( A_0, b_1, C_01 );
			b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
			C_02 = _mm256_fnmadd_pd( A_0, B_2, C_02 );
			C_03 = _mm256_fnmadd_pd( A_0, b_3, C_03 );
			b_2  = _mm256_broadcast_pd( (__m128d *) &Bm[10] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			B_0  = _mm256_broadcast_pd( (__m128d *) &Bm[12] ); // prefetch
			A_0  = _mm256_load_pd( &Am[12] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fnmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fnmadd_pd( a_0, b_3, c_03 );
			B_2  = _mm256_broadcast_pd( (__m128d *) &Bm[14] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			b_0  = _mm256_broadcast_pd( (__m128d *) &Bm[16] ); // prefetch
			a_0  = _mm256_load_pd( &Am[16] ); // prefetch
			b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
			C_00 = _mm256_fnmadd_pd( A_0, B_0, C_00 );
			C_01 = _mm256_fnmadd_pd( A_0, b_1, C_01 );
			b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
			C_02 = _mm256_fnmadd_pd( A_0, B_2, C_02 );
			C_03 = _mm256_fnmadd_pd( A_0, b_3, C_03 );
			b_2  = _mm256_broadcast_pd( (__m128d *) &Bm[18] ); // prefetch
			
	
			Am += 16;
			Bm += 16;

			}

		}

	c_00 = _mm256_add_pd( c_00, C_00 );
	c_01 = _mm256_add_pd( c_01, C_01 );
	c_03 = _mm256_add_pd( c_03, C_03 );
	c_02 = _mm256_add_pd( c_02, C_02 );

	__m256d
		d_00, d_01, d_02, d_03;

	d_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	d_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	d_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	d_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C[0+bs*0] );
		c_01 = _mm256_load_pd( &C[0+bs*1] );
		c_02 = _mm256_load_pd( &C[0+bs*2] );
		c_03 = _mm256_load_pd( &C[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );
		}
		
	// factorize
	__m128
		ssa_00, szeros_ones;

	__m128d
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

	// first row
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		//sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		//a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_store_pd( &D[0+bs*0], d_00 ); // a_00
		}
	else // comile
		{
		a_00  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128(a_00) );
		_mm256_store_pd( &D[0+bs*0], a_00 );
		}

	// second row
	//sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	//_mm_store_sd( &fact[1], sa_10 );
	//a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load  ???
		//sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		//a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		_mm256_maskstore_pd( &D[0+bs*1], mask1, d_01 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load  ???
		a_11  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128(a_11) );
		_mm256_maskstore_pd( &D[0+bs*1], mask1, a_11 );
		}

	// third row
	//sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	//sa_20 = _mm_permute_pd( sa_20, 0x0 );
	//_mm_store_sd( &fact[3], sa_20 );
	//a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	//sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	//sa_21 = _mm_permute_pd( sa_21, 0x0 );
	//_mm_store_sd( &fact[4], sa_21 );
	//a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		//sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		//a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		_mm256_maskstore_pd( &D[0+bs*2], mask1, d_02 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		a_22  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128(a_22) );
		_mm256_maskstore_pd( &D[0+bs*2], mask1, a_22 );
		}

	// fourth row
	//sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	//sa_30 = _mm_permute_pd( sa_30, 0x3 );
	//_mm_store_sd( &fact[6], sa_30 );
	//a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
	a_30  = _mm256_permute4x64_pd( d_00, 0xff );
	d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
	//sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
	//sa_31 = _mm_permute_pd( sa_31, 0x3 );
	//_mm_store_sd( &fact[7], sa_31 );
	//a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
	a_31  = _mm256_permute4x64_pd( d_01, 0xff );
	d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
	//sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	//sa_32 = _mm_permute_pd( sa_32, 0x3 );
	//_mm_store_sd( &fact[8], sa_32 );
	//a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
	a_32  = _mm256_permute4x64_pd( d_02, 0xff );
	d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		//sa_33 = _mm_movedup_pd( sa_33 );
		_mm_store_sd( &inv_diag_D[3], sa_33 );
		//a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
		a_33  = _mm256_broadcastsd_pd( sa_33 );
		d_03  = _mm256_mul_pd( d_03, a_33 );
		_mm256_maskstore_pd( &D[0+bs*3], mask1, d_03 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		a_33  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128(a_33) );
		_mm256_maskstore_pd( &D[0+bs*3], mask1, a_33 );
		}

	}



void kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{
	
	const int bs = 4;
	
	static __m256i mask_bkp[4];
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		zeros, ab_temp,
		a_0, A_0,
		b_0, B_0, b_1, b_2, B_2, b_3,
		c_00, c_01, c_03, c_02,
		C_00, C_01, C_03, C_02;
	
	__m256i
		mask0;

	if(km>=4)
		{
		mask0 = _mm256_set_epi64x( -1, -1, -1, -1 );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}
	else
		{
		d_temp = km-0.0;
		mask0 = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}

		// zero registers
	zeros = _mm256_setzero_pd();
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	C_00 = _mm256_setzero_pd();
	C_01 = _mm256_setzero_pd();
	C_03 = _mm256_setzero_pd();
	C_02 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Ap[0] );
		b_0 = _mm256_broadcast_pd( (__m128d *) &Bp[0] );
		b_2 = _mm256_broadcast_pd( (__m128d *) &Bp[2] );

		if(tri_A==1)
			{

			if(kadd>=4)
				{

				// k = 0
				a_0  = _mm256_blend_pd(zeros, a_0, 0x1 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				a_0  = _mm256_load_pd( &Ap[4] ); // prefetch
				b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch

				// k = 1
				a_0  = _mm256_blend_pd(zeros, a_0, 0x3 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
				c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
				a_0  = _mm256_load_pd( &Ap[8] ); // prefetch
				b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[10] ); // prefetch

				// k = 2
				a_0  = _mm256_blend_pd(zeros, a_0, 0x7 );
				b_0  = _mm256_blend_pd(zeros, b_0, 0x7 ); // XXX
				b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
				c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
				b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
				c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
				b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[14] ); // prefetch
				c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
				a_0  = _mm256_load_pd( &Ap[12] ); // prefetch

				// k = 3
				b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
				c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
				b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
				c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
				b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[18] ); // prefetch
				c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
				a_0  = _mm256_load_pd( &Ap[16] ); // prefetch

			
				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0  = _mm256_blend_pd(zeros, a_0, 0x1 );
				c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
				a_0  = _mm256_load_pd( &Ap[4] ); // prefetch
				b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0  = _mm256_blend_pd(zeros, a_0, 0x3 );
					c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
					b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
					b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
					c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
					a_0  = _mm256_load_pd( &Ap[8] ); // prefetch
					b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[10] ); // prefetch

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0  = _mm256_blend_pd(zeros, a_0, 0x7 );
						b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
						c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
						b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
						c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
						b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
						c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
						b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[14] ); // prefetch
						c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
						a_0  = _mm256_load_pd( &Ap[12] ); // prefetch

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			B_0  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			A_0  = _mm256_load_pd( &Ap[4] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
			B_2  = _mm256_broadcast_pd( (__m128d *) &Bp[6] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			a_0  = _mm256_load_pd( &Ap[8] ); // prefetch
			b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
			C_00 = _mm256_fmadd_pd( A_0, B_0, C_00 );
			C_01 = _mm256_fmadd_pd( A_0, b_1, C_01 );
			b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
			C_02 = _mm256_fmadd_pd( A_0, B_2, C_02 );
			C_03 = _mm256_fmadd_pd( A_0, b_3, C_03 );
			b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[10] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			B_0  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
			A_0  = _mm256_load_pd( &Ap[12] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
			B_2  = _mm256_broadcast_pd( (__m128d *) &Bp[14] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
			a_0  = _mm256_load_pd( &Ap[16] ); // prefetch
			b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
			C_00 = _mm256_fmadd_pd( A_0, B_0, C_00 );
			C_01 = _mm256_fmadd_pd( A_0, b_1, C_01 );
			b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
			C_02 = _mm256_fmadd_pd( A_0, B_2, C_02 );
			C_03 = _mm256_fmadd_pd( A_0, b_3, C_03 );
			b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[18] ); // prefetch
				

			Ap += 16;
			Bp += 16;

			}
		
		if(k<kadd-1)
			{
			
			B_0  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			A_0  = _mm256_load_pd( &Ap[4] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
			B_2  = _mm256_broadcast_pd( (__m128d *) &Bp[6] ); // prefetch
			
			
			b_0  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			a_0  = _mm256_load_pd( &Ap[8] ); // prefetch
			b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
			C_00 = _mm256_fmadd_pd( A_0, B_0, C_00 );
			C_01 = _mm256_fmadd_pd( A_0, b_1, C_01 );
			b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
			C_02 = _mm256_fmadd_pd( A_0, B_2, C_02 );
			C_03 = _mm256_fmadd_pd( A_0, b_3, C_03 );
			b_2  = _mm256_broadcast_pd( (__m128d *) &Bp[10] ); // prefetch
		
			
			Ap += 8;
			Bp += 8;
			k  += 2;

			}

		if(k<kadd)
			{
			
//			B_0  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
//			A_0  = _mm256_load_pd( &Ap[4] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fmadd_pd( a_0, b_3, c_03 );
//			B_2  = _mm256_broadcast_pd( (__m128d *) &Bp[6] ); // prefetch
			
//			Ap += 4; // keep it !!!
//			Bp += 4; // keep it !!!
			
			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			c_00 = _mm256_xor_pd( c_00, sign );
			c_01 = _mm256_xor_pd( c_01, sign );
			c_02 = _mm256_xor_pd( c_02, sign );
			c_03 = _mm256_xor_pd( c_03, sign );
			C_00 = _mm256_xor_pd( C_00, sign );
			C_01 = _mm256_xor_pd( C_01, sign );
			C_02 = _mm256_xor_pd( C_02, sign );
			C_03 = _mm256_xor_pd( C_03, sign );

			}

		}
		
	if(ksub>0)
		{

		// prefetch
		a_0 = _mm256_load_pd( &Am[0] );
		b_0 = _mm256_broadcast_pd( (__m128d *) &Bm[0] );
		b_2 = _mm256_broadcast_pd( (__m128d *) &Bm[2] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			B_0  = _mm256_broadcast_pd( (__m128d *) &Bm[4] ); // prefetch
			A_0  = _mm256_load_pd( &Am[4] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fnmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fnmadd_pd( a_0, b_3, c_03 );
			B_2  = _mm256_broadcast_pd( (__m128d *) &Bm[6] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			b_0  = _mm256_broadcast_pd( (__m128d *) &Bm[8] ); // prefetch
			a_0  = _mm256_load_pd( &Am[8] ); // prefetch
			b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
			C_00 = _mm256_fnmadd_pd( A_0, B_0, C_00 );
			C_01 = _mm256_fnmadd_pd( A_0, b_1, C_01 );
			b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
			C_02 = _mm256_fnmadd_pd( A_0, B_2, C_02 );
			C_03 = _mm256_fnmadd_pd( A_0, b_3, C_03 );
			b_2  = _mm256_broadcast_pd( (__m128d *) &Bm[10] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			B_0  = _mm256_broadcast_pd( (__m128d *) &Bm[12] ); // prefetch
			A_0  = _mm256_load_pd( &Am[12] ); // prefetch
			b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
			c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
			c_01 = _mm256_fnmadd_pd( a_0, b_1, c_01 );
			b_3  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
			c_02 = _mm256_fnmadd_pd( a_0, b_2, c_02 );
			c_03 = _mm256_fnmadd_pd( a_0, b_3, c_03 );
			B_2  = _mm256_broadcast_pd( (__m128d *) &Bm[14] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			b_0  = _mm256_broadcast_pd( (__m128d *) &Bm[16] ); // prefetch
			a_0  = _mm256_load_pd( &Am[16] ); // prefetch
			b_1  = _mm256_shuffle_pd( B_0, B_0, 0x5 );
			C_00 = _mm256_fnmadd_pd( A_0, B_0, C_00 );
			C_01 = _mm256_fnmadd_pd( A_0, b_1, C_01 );
			b_3  = _mm256_shuffle_pd( B_2, B_2, 0x5 );
			C_02 = _mm256_fnmadd_pd( A_0, B_2, C_02 );
			C_03 = _mm256_fnmadd_pd( A_0, b_3, C_03 );
			b_2  = _mm256_broadcast_pd( (__m128d *) &Bm[18] ); // prefetch
			
	
			Am += 16;
			Bm += 16;

			}

		}

	c_00 = _mm256_add_pd( c_00, C_00 );
	c_01 = _mm256_add_pd( c_01, C_01 );
	c_03 = _mm256_add_pd( c_03, C_03 );
	c_02 = _mm256_add_pd( c_02, C_02 );

	__m256d
		d_00, d_01, d_02, d_03;

	d_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	d_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	d_02 = _mm256_blend_pd( c_02, c_03, 0xa );
	d_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
	
	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C[0+bs*0] );
		c_01 = _mm256_load_pd( &C[0+bs*1] );
		c_02 = _mm256_load_pd( &C[0+bs*2] );
		c_03 = _mm256_load_pd( &C[0+bs*3] );

		d_00 = _mm256_add_pd( d_00, c_00 );
		d_01 = _mm256_add_pd( d_01, c_01 );
		d_02 = _mm256_add_pd( d_02, c_02 );
		d_03 = _mm256_add_pd( d_03, c_03 );
		}
		
	// factorize
	__m128
		ssa_00, szeros_ones;

	__m128d
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

	// first row
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		//sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		//a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_maskstore_pd( &D[0+bs*0], mask0, d_00 ); // a_00
		}
	else // comile
		{
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_00  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128(a_00) );
		_mm256_maskstore_pd( &D[0+bs*0], mask0, a_00 );
		}

	// second row
	//sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	//_mm_store_sd( &fact[1], sa_10 );
	//a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
	d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load  ???
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		//sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		//a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		a_11  = _mm256_broadcastsd_pd( sa_11 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		_mm256_maskstore_pd( &D[0+bs*1], mask0, d_01 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load  ???
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		a_11  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128(a_11) );
		_mm256_maskstore_pd( &D[0+bs*1], mask0, a_11 );
		}

	// third row
	//sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	//sa_20 = _mm_permute_pd( sa_20, 0x0 );
	//_mm_store_sd( &fact[3], sa_20 );
	//a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	a_20  = _mm256_permute4x64_pd( d_00, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_00, a_20, d_02 );
	//sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	//sa_21 = _mm_permute_pd( sa_21, 0x0 );
	//_mm_store_sd( &fact[4], sa_21 );
	//a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	a_21  = _mm256_permute4x64_pd( d_01, 0xaa );
	d_02  = _mm256_fnmadd_pd( d_01, a_21, d_02 );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		//sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		//a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		a_22  = _mm256_broadcastsd_pd( sa_22 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		_mm256_maskstore_pd( &D[0+bs*2], mask0, d_02 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		a_22  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128(a_22) );
		_mm256_maskstore_pd( &D[0+bs*2], mask0, a_22 );
		}

	if(kn>=4)
		{
		// fourth row
		//sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
		//sa_30 = _mm_permute_pd( sa_30, 0x3 );
		//_mm_store_sd( &fact[6], sa_30 );
		//a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
		a_30  = _mm256_permute4x64_pd( d_00, 0xff );
		d_03  = _mm256_fnmadd_pd( d_00, a_30, d_03 );
		//sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
		//sa_31 = _mm_permute_pd( sa_31, 0x3 );
		//_mm_store_sd( &fact[7], sa_31 );
		//a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
		a_31  = _mm256_permute4x64_pd( d_01, 0xff );
		d_03  = _mm256_fnmadd_pd( d_01, a_31, d_03 );
		//sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
		//sa_32 = _mm_permute_pd( sa_32, 0x3 );
		//_mm_store_sd( &fact[8], sa_32 );
		//a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
		a_32  = _mm256_permute4x64_pd( d_02, 0xff );
		d_03  = _mm256_fnmadd_pd( d_02, a_32, d_03 );
		sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
		sa_33 = _mm_permute_pd( sa_33, 0x3 );
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		if( _mm_comigt_sd ( sa_33, zeros_ones ) )
			{
			sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
			zeros_ones = _mm_set_sd( 1.0 );
			sa_33 = _mm_div_sd( zeros_ones, sa_33 );
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			//sa_33 = _mm_movedup_pd( sa_33 );
			_mm_store_sd( &inv_diag_D[3], sa_33 );
			//a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
			a_33  = _mm256_broadcastsd_pd( sa_33 );
			d_03  = _mm256_mul_pd( d_03, a_33 );
			_mm256_maskstore_pd( &D[0+bs*3], mask0, d_03 ); // a_00
			}
		else // comile
			{
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			a_33  = _mm256_setzero_pd();
			_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128(a_33) );
			_mm256_maskstore_pd( &D[0+bs*3], mask0, a_33 );
			}

		}

	}



void kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{
	
	const int bs = 4;
	
	static __m256i mask_bkp[4];
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		zeros,
		a_0123,
		b_0101, b_1010,
		ab_temp, // temporary results
		c_00_11_20_31, c_01_10_21_30, C_00_11_20_31, C_01_10_21_30,
		d_00_11_20_31, d_01_10_21_30, D_00_11_20_31, D_01_10_21_30;
	
	__m256i
		mask0;

	if(km>=4)
		{
		mask0 = _mm256_set_epi64x( -1, -1, -1, -1 );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}
	else
		{
		d_temp = km-0.0;
		mask0 = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
		_mm256_storeu_si256( mask_bkp, mask0 );
		}

		// zero registers
	zeros = _mm256_setzero_pd();
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	C_00_11_20_31 = _mm256_setzero_pd();
	C_01_10_21_30 = _mm256_setzero_pd();
	d_00_11_20_31 = _mm256_setzero_pd();
	d_01_10_21_30 = _mm256_setzero_pd();
	D_00_11_20_31 = _mm256_setzero_pd();
	D_01_10_21_30 = _mm256_setzero_pd();
	
	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Ap[0] );
		b_0101 = _mm256_broadcast_pd( (__m128d *) &Bp[0] );

		if(tri_A==1)
			{

			if(kadd>=4)
				{

				// k = 0
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
				a_0123        = _mm256_load_pd( &Ap[4] ); // prefetch
				c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
				b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
		
				// k = 1
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
				C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, ab_temp );
				b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
				ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
				a_0123        = _mm256_load_pd( &Ap[8] ); // prefetch
				C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, ab_temp );

				// k = 2
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
				ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
				c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
				b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
				ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
				a_0123        = _mm256_load_pd( &Ap[12] ); // prefetch
				c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );

				// k = 3
				ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
				C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, ab_temp );
				b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
				ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
				a_0123        = _mm256_load_pd( &Ap[16] ); // prefetch
				C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, ab_temp );
				
				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{


				// k = 0
				a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
				a_0123        = _mm256_load_pd( &Ap[4] ); // prefetch
				c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
				b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch

				k  += 1;

				if(kadd>1)
					{
		

					// k = 1
					a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
					ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
					C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, ab_temp );
					b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
					ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
					a_0123        = _mm256_load_pd( &Ap[8] ); // prefetch
					C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, ab_temp );

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
						ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
						c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
						b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
						b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
						ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
						a_0123        = _mm256_load_pd( &Ap[12] ); // prefetch
						c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Ap[4] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			C_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, C_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			C_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, C_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Ap[8] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			d_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, d_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
			d_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, d_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Ap[12] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			D_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, D_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
			D_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, D_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Ap[16] ); // prefetch
			
			Ap += 16;
			Bp += 16;

			}
		
		if(k<kadd-1)
			{
			
			c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Ap[4] ); // prefetch
		
			
			C_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, C_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			C_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, C_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Ap[8] ); // prefetch
				
			
			Ap += 8;
			Bp += 8;
			k  += 2;

			}

		if(k<kadd)
			{
			

			c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			//b_0101        = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
			//a_0123        = _mm256_load_pd( &Ap[4] ); // prefetch
		
//			Ap += 4; // keep it !!!
//			Bp += 4; // keep it !!!

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			c_00_11_20_31 = _mm256_xor_pd( c_00_11_20_31, sign );
			c_01_10_21_30 = _mm256_xor_pd( c_01_10_21_30, sign );
			d_00_11_20_31 = _mm256_xor_pd( d_00_11_20_31, sign );
			d_01_10_21_30 = _mm256_xor_pd( d_01_10_21_30, sign );
			C_00_11_20_31 = _mm256_xor_pd( C_00_11_20_31, sign );
			C_01_10_21_30 = _mm256_xor_pd( C_01_10_21_30, sign );
			D_00_11_20_31 = _mm256_xor_pd( D_00_11_20_31, sign );
			D_01_10_21_30 = _mm256_xor_pd( D_01_10_21_30, sign );

			}

		}
			
	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am[0] );
		b_0101 = _mm256_broadcast_pd( (__m128d *) &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bm[4] ); // prefetch
			c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Am[4] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			C_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, C_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bm[8] ); // prefetch
			C_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, C_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Am[8] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			d_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, d_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bm[12] ); // prefetch
			d_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, d_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Am[12] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			D_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, D_00_11_20_31 );
			b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101        = _mm256_broadcast_pd( (__m128d *) &Bm[16] ); // prefetch
			D_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, D_01_10_21_30 );
			a_0123        = _mm256_load_pd( &Am[16] ); // prefetch
				
			Am += 16;
			Bm += 16;

			}

		}

	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, d_00_11_20_31 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, d_01_10_21_30 );
	C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, D_00_11_20_31 );
	C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, D_01_10_21_30 );
	
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, C_00_11_20_31 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, C_01_10_21_30 );

	__m256d
		c_00, c_01,
		d_00, d_01;

	d_00 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
	d_01 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );

	if(alg!=0)
		{
		c_00 = _mm256_load_pd( &C[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, c_00 );
		c_01 = _mm256_load_pd( &C[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, c_01 );
		}
		
	// factorize
	__m128
		ssa_00, szeros_ones;

	__m128d
		x_half, t_const, y2_const,
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_20, sa_30, sa_11, sa_21, sa_31, sa_22, sa_32, sa_33;

	__m256d
		temp,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	__m256i
		mask1;

	// first row
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_00 = _mm_move_sd( sa_00, _mm256_castpd256_pd128(d_00) );
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
#if defined(FAST_RSQRT)
		t_const = sa_00;
		sa_00 = _mm_cvtss_sd( sa_00, _mm_rsqrt_ss ( _mm_cvtsd_ss ( ssa_00, sa_00 ) ) );
		x_half = _mm_set_sd( 0.5 );
		y2_const = _mm_mul_sd( sa_00, sa_00 );
		x_half = _mm_mul_sd( x_half, t_const );
		t_const = _mm_set_sd( 1.5 );
		t_const = _mm_fnmadd_sd( x_half, y2_const, t_const );
		sa_00 = _mm_mul_pd( sa_00, t_const );
#else
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		zeros_ones = _mm_set_sd( 1.0 );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
#endif
		//sa_00 = _mm_movedup_pd( sa_00 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		//a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		a_00  = _mm256_broadcastsd_pd( sa_00 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_maskstore_pd( &D[0+bs*0], mask0, d_00 ); // a_00
		}
	else // comile
		{
		a_00  = _mm256_setzero_pd();
		mask0 = _mm256_loadu_si256( mask_bkp );
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128(a_00) );
		_mm256_maskstore_pd( &D[0+bs*0], mask0, a_00 );
		}

	if(kn>=2)
		{
		// second row
		//sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
		//_mm_store_sd( &fact[1], sa_10 );
		//a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
		a_10  = _mm256_permute4x64_pd( d_00, 0x55 );
		d_01  = _mm256_fnmadd_pd( d_00, a_10, d_01 );
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
		if( _mm_comigt_sd ( sa_11, zeros_ones ) )
			{
	#if defined(FAST_RSQRT)
			t_const = sa_11;
			sa_11 = _mm_cvtss_sd( sa_11, _mm_rsqrt_ss ( _mm_cvtsd_ss ( ssa_00, sa_11 ) ) );
			x_half = _mm_set_sd( 0.5 );
			y2_const = _mm_mul_sd( sa_11, sa_11 );
			x_half = _mm_mul_sd( x_half, t_const );
			t_const = _mm_set_sd( 1.5 );
			t_const = _mm_fnmadd_sd( x_half, y2_const, t_const );
			sa_11 = _mm_mul_pd( sa_11, t_const );
	#else
			sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
			zeros_ones = _mm_set_sd( 1.0 );
			sa_11 = _mm_div_sd( zeros_ones, sa_11 );
	#endif
			mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			//sa_11 = _mm_movedup_pd( sa_11 );
			_mm_store_sd( &inv_diag_D[1], sa_11 );
			//a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
			a_11  = _mm256_broadcastsd_pd( sa_11 );
			d_01  = _mm256_mul_pd( d_01, a_11 );
			_mm256_maskstore_pd( &D[0+bs*1], mask0, d_01 ); // a_00
			}
		else // comile
			{
			mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load ???
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			a_11  = _mm256_setzero_pd();
			_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128(a_11) );
			_mm256_maskstore_pd( &D[0+bs*1], mask0, a_11 );
			}

		}

	}



// normal-transposed, 2x2 with data packed in 4
void kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;
	
	int k;
	
	__m128d
		a_01,
		b_01, b_10,
		ab_temp, // temporary results
		c_00_11, c_01_10, C_00_11, C_01_10,
		d_00_11, d_01_10, D_00_11, D_01_10;
	
	// zero registers
	c_00_11 = _mm_setzero_pd();
	c_01_10 = _mm_setzero_pd();
	C_00_11 = _mm_setzero_pd();
	C_01_10 = _mm_setzero_pd();
	d_00_11 = _mm_setzero_pd();
	d_01_10 = _mm_setzero_pd();
	D_00_11 = _mm_setzero_pd();
	D_01_10 = _mm_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_01 = _mm_load_pd( &Ap[0] );
		b_01 = _mm_load_pd( &Bp[0] );

		if(tri_A==1)
			{

			if(kadd>=2)
				{

				// k = 0
				ab_temp = _mm_mul_sd( a_01, b_01 );
				c_00_11 = _mm_add_sd( c_00_11, ab_temp );
				b_01    = _mm_load_pd( &Bp[4] ); // prefetch
				a_01    = _mm_load_pd( &Ap[4] ); // prefetch

				// k = 1
				ab_temp = _mm_mul_pd( a_01, b_01 );
				C_00_11 = _mm_add_pd( C_00_11, ab_temp );
				b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
				b_01    = _mm_load_pd( &Bp[8] ); // prefetch
				ab_temp = _mm_mul_pd( a_01, b_10 );
				a_01    = _mm_load_pd( &Ap[8] ); // prefetch
				C_01_10 = _mm_add_pd( C_01_10, ab_temp );

				Ap += 8;
				Bp += 8;
				k  += 2;

				}
			else
				{

				// k = 0
				ab_temp = _mm_mul_sd( a_01, b_01 );
				c_00_11 = _mm_add_sd( c_00_11, ab_temp );
				b_01    = _mm_load_pd( &Bp[4] ); // prefetch
				a_01    = _mm_load_pd( &Ap[4] ); // prefetch

				k  += 1;

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			c_00_11 = _mm_fmadd_pd( a_01, b_01, c_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[4] ); // prefetch
			c_01_10 = _mm_fmadd_pd( a_01, b_10, c_01_10 );
			a_01    = _mm_load_pd( &Ap[4] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			C_00_11 = _mm_fmadd_pd( a_01, b_01, C_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[8] ); // prefetch
			C_01_10 = _mm_fmadd_pd( a_01, b_10, C_01_10 );
			a_01    = _mm_load_pd( &Ap[8] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			d_00_11 = _mm_fmadd_pd( a_01, b_01, d_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[12] ); // prefetch
			d_01_10 = _mm_fmadd_pd( a_01, b_10, d_01_10 );
			a_01    = _mm_load_pd( &Ap[12] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			D_00_11 = _mm_fmadd_pd( a_01, b_01, D_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[16] ); // prefetch
			D_01_10 = _mm_fmadd_pd( a_01, b_10, D_01_10 );
			a_01    = _mm_load_pd( &Ap[16] ); // prefetch
				
			Ap += 16;
			Bp += 16;

			}
		
		if(k<kadd-1)
			{
			
			c_00_11 = _mm_fmadd_pd( a_01, b_01, c_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[4] ); // prefetch
			c_01_10 = _mm_fmadd_pd( a_01, b_10, c_01_10 );
			a_01    = _mm_load_pd( &Ap[4] ); // prefetch
		
			
			C_00_11 = _mm_fmadd_pd( a_01, b_01, C_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[8] ); // prefetch
			C_01_10 = _mm_fmadd_pd( a_01, b_10, C_01_10 );
			a_01    = _mm_load_pd( &Ap[8] ); // prefetch
		
			
			Ap += 8;
			Bp += 8;
			k  += 2;

			}

		if(k<kadd)
			{
			
			c_00_11 = _mm_fmadd_pd( a_01, b_01, c_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			//b_01    = _mm_load_pd( &B[4] ); // prefetch
			c_01_10 = _mm_fmadd_pd( a_01, b_10, c_01_10 );
			//a_01    = _mm_load_pd( &A[4] ); // prefetch
		
//			Ap += 4; // keep it !!!
//			Bp += 4; // keep it !!!

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m128d sign = _mm_loaddup_pd( (double *) &long_sign );

			c_00_11 = _mm_xor_pd( c_00_11, sign );
			c_01_10 = _mm_xor_pd( c_01_10, sign );
			d_00_11 = _mm_xor_pd( d_00_11, sign );
			d_01_10 = _mm_xor_pd( d_01_10, sign );
			C_00_11 = _mm_xor_pd( C_00_11, sign );
			C_01_10 = _mm_xor_pd( C_01_10, sign );
			D_00_11 = _mm_xor_pd( D_00_11, sign );
			D_01_10 = _mm_xor_pd( D_01_10, sign );

			}

		}
		
	if(ksub>0)
		{

		// prefetch
		a_01 = _mm_load_pd( &Am[0] );
		b_01 = _mm_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			c_00_11 = _mm_fnmadd_pd( a_01, b_01, c_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[4] ); // prefetch
			c_01_10 = _mm_fnmadd_pd( a_01, b_10, c_01_10 );
			a_01    = _mm_load_pd( &Am[4] ); // prefetch
			
			
	/*	__builtin_prefetch( A+40 );*/
			C_00_11 = _mm_fnmadd_pd( a_01, b_01, C_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[8] ); // prefetch
			C_01_10 = _mm_fnmadd_pd( a_01, b_10, C_01_10 );
			a_01    = _mm_load_pd( &Am[8] ); // prefetch


	/*	__builtin_prefetch( A+48 );*/
			d_00_11 = _mm_fnmadd_pd( a_01, b_01, d_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[12] ); // prefetch
			d_01_10 = _mm_fnmadd_pd( a_01, b_10, d_01_10 );
			a_01    = _mm_load_pd( &Am[12] ); // prefetch


	/*	__builtin_prefetch( A+56 );*/
			D_00_11 = _mm_fnmadd_pd( a_01, b_01, D_00_11 );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[16] ); // prefetch
			D_01_10 = _mm_fnmadd_pd( a_01, b_10, D_01_10 );
			a_01    = _mm_load_pd( &Am[16] ); // prefetch
				
			Am += 16;
			Bm += 16;

			}

		}

	c_00_11 = _mm_add_pd( c_00_11, d_00_11 );
	c_01_10 = _mm_add_pd( c_01_10, d_01_10 );
	C_00_11 = _mm_add_pd( C_00_11, D_00_11 );
	C_01_10 = _mm_add_pd( C_01_10, D_01_10 );
	c_00_11 = _mm_add_pd( c_00_11, C_00_11 );
	c_01_10 = _mm_add_pd( c_01_10, C_01_10 );

	__m128d
		c_00_10, c_01_11,
		d_00_10, d_01_11;

	d_00_10 = _mm_blend_pd( c_00_11, c_01_10, 0x2 );
	d_01_11 = _mm_blend_pd( c_00_11, c_01_10, 0x1 );
	if(alg!=0)
		{
		c_00_10 = _mm_load_pd( &C[0+bs*0] );
		d_00_10 = _mm_add_pd( d_00_10, c_00_10 );
		c_01_11 = _mm_load_pd( &C[0+bs*1] );
		d_01_11 = _mm_add_pd( d_01_11, c_01_11 );
		}
		
	// factorize the upper 4x4 matrix
	__m128d
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_11;
	
	__m128
		szeros_ones, ssa_00, ssa_11;


	// first row
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_00 = _mm_move_sd( sa_00, d_00_10 );
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		sa_10 = _mm_shuffle_pd( d_00_10, zeros_ones, 0x1 );
		zeros_ones = _mm_set_sd( 1.0 );
		_mm_store_sd( &D[0+bs*0], sa_00 ); // a_00
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		sa_10 = _mm_mul_sd( sa_10, sa_00 );
		if(km==1)
			return;
		_mm_store_sd( &D[1+bs*0], sa_10 ); // a_10
		}
	else // comile
		{
		sa_00 = _mm_setzero_pd();
		_mm_store_sd( &D[0+bs*0], sa_00 ); // a_00
		//sa_10 = sa_00;
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		if(km==1)
			return;
		_mm_store_sd( &D[1+bs*0], sa_00 ); // a_10
		}

	if(kn>=2)
		{

		// second row
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		sa_11 = _mm_shuffle_pd( d_01_11, zeros_ones, 0x1 );
		sab_temp = _mm_mul_sd( sa_10, sa_10 );
		sa_11 = _mm_sub_sd( sa_11, sab_temp );
		if( _mm_comigt_sd ( sa_11, zeros_ones ) )
			{
			sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
			zeros_ones = _mm_set_sd( 1.0 );
			_mm_store_sd( &D[1+bs*1], sa_11 ); // a_11
			sa_11 = _mm_div_sd( zeros_ones, sa_11 );
			_mm_store_sd( &inv_diag_D[1], sa_11 );
			}
		else // comile
			{
			sa_11 = _mm_setzero_pd();
			_mm_store_sd( &D[1+bs*1], sa_11 ); // a_11
			_mm_store_sd( &inv_diag_D[1], sa_11 );
			}

		}
	

	}



