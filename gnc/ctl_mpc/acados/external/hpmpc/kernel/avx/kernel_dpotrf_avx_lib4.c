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
#include <math.h>  // TODO remove !!!




// new kernels

// TODO fix tri_A==1 in another routine !!!!!
void kernel_dpotrf_nt_8x4_lib4_new(int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Am1 = Am0 + 4*sdam;
	double *C1  = C0  + 4*sdc;
	double *D1  = D0  + 4*sdd;
	
	const int bs = 4;

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, zeros,
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	k = 0;

	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am0[0] );
		a_4567 = _mm256_load_pd( &Am1[0] );
		b_0123 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[4] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[4] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Am1[8] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[12] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Am1[16] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );
			
			Am0 += 16;
			Am1 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0, e_1, e_2, e_3,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43;

	e_0 = _mm256_blend_pd( d_0, d_1, 0xa );
	e_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
	e_2 = _mm256_blend_pd( d_2, d_3, 0xa );
	e_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

	d_00 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_02 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_01 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_03 = _mm256_blend_pd( e_1, e_3, 0x3 );

	e_0 = _mm256_blend_pd( d_4, d_5, 0xa );
	e_1 = _mm256_blend_pd( d_4, d_5, 0x5 );
	e_2 = _mm256_blend_pd( d_6, d_7, 0xa );
	e_3 = _mm256_blend_pd( d_6, d_7, 0x5 );

	d_40 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_42 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_41 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_43 = _mm256_blend_pd( e_1, e_3, 0x3 );


	if(alg!=0)
		{

		e_0  = _mm256_load_pd( &C0[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*2] );
		d_02 = _mm256_add_pd( d_02, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*3] );
		d_03 = _mm256_add_pd( d_03, e_0 );

		e_0  = _mm256_load_pd( &C1[0+bs*0] );
		d_40 = _mm256_add_pd( d_40, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*1] );
		d_41 = _mm256_add_pd( d_41, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*2] );
		d_42 = _mm256_add_pd( d_42, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*3] );
		d_43 = _mm256_add_pd( d_43, e_0 );

		}
		

	// factorize
	__m128d
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
		sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
		d_40 = _mm256_mul_pd( d_40, a_00 );
		_mm256_store_pd( &D1[0+bs*0], d_40 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
		_mm256_store_pd( &D0[0+bs*0], a_00 );
		_mm256_store_pd( &D1[0+bs*0], a_00 );
		}

	// second row
	sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_10 );
	d_01  = _mm256_sub_pd( d_01, temp );
	temp  = _mm256_mul_pd( d_40, a_10 );
	d_41  = _mm256_sub_pd( d_41, temp );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
		d_41 = _mm256_mul_pd( d_41, a_11 );
		_mm256_store_pd( &D1[0+bs*1], d_41 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		a_11 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, a_11 );
		_mm256_store_pd( &D1[0+bs*1], a_11 );
		}

	// third row
	sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_20 = _mm_permute_pd( sa_20, 0x0 );
	a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_20 );
	d_02  = _mm256_sub_pd( d_02, temp );
	temp  = _mm256_mul_pd( d_40, a_20 );
	d_42  = _mm256_sub_pd( d_42, temp );
	sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	sa_21 = _mm_permute_pd( sa_21, 0x0 );
	a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_21 );
	d_02  = _mm256_sub_pd( d_02, temp );
	temp  = _mm256_mul_pd( d_41, a_21 );
	d_42  = _mm256_sub_pd( d_42, temp );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
		d_42 = _mm256_mul_pd( d_42, a_22 );
		_mm256_store_pd( &D1[0+bs*2], d_42 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		a_22 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, a_22 );
		_mm256_store_pd( &D1[0+bs*2], a_22 );
		}
	
	// fourth row
	sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_30 = _mm_permute_pd( sa_30, 0x3 );
	a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_30 );
	d_03  = _mm256_sub_pd( d_03, temp );
	temp  = _mm256_mul_pd( d_40, a_30 );
	d_43  = _mm256_sub_pd( d_43, temp );
	sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
	sa_31 = _mm_permute_pd( sa_31, 0x3 );
	a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_31 );
	d_03  = _mm256_sub_pd( d_03, temp );
	temp  = _mm256_mul_pd( d_41, a_31 );
	d_43  = _mm256_sub_pd( d_43, temp );
	sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	sa_32 = _mm_permute_pd( sa_32, 0x3 );
	a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
	temp  = _mm256_mul_pd( d_02, a_32 );
	d_03  = _mm256_sub_pd( d_03, temp );
	temp  = _mm256_mul_pd( d_42, a_32 );
	d_43  = _mm256_sub_pd( d_43, temp );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		sa_33 = _mm_movedup_pd( sa_33 );
		_mm_store_sd( &inv_diag_D[3], sa_33 );
		a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
		d_03  = _mm256_mul_pd( d_03, a_33 );
		_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
		d_43 = _mm256_mul_pd( d_43, a_33 );
		_mm256_store_pd( &D1[0+bs*3], d_43 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		a_33 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
		_mm256_maskstore_pd( &D0[0+bs*3], mask1, a_33 );
		_mm256_store_pd( &D1[0+bs*3], a_33 ); // a_00
		}
		
	return;

	}



void kernel_dsyrk_dpotrf_nt_8x4_lib4_new(int kadd, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *inv_diag_D)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *C1  = C0  + 4*sdc;
	double *D1  = D0  + 4*sdd;
	
	const int bs = 4;

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, zeros,
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Ap0[0] );
		a_4567 = _mm256_load_pd( &Ap1[0] );
		b_0123 = _mm256_load_pd( &Bp[0] );

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			Ap0 += 16;
			Ap1 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );

			
			Ap0 += 8;
			Ap1 += 8;
			Bp  += 8;
			k   += 2;

			}
		if(k<kadd)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
//			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
//			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
//			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );

		
//			Ap0 += 4; // keep it !!!
//			Ap1 += 4; // keep it !!!
//			Bp  += 4; // keep it !!!

			}
		
		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			d_0 = _mm256_xor_pd( d_0, sign );
			d_1 = _mm256_xor_pd( d_1, sign );
			d_2 = _mm256_xor_pd( d_2, sign );
			d_3 = _mm256_xor_pd( d_3, sign );
			d_4 = _mm256_xor_pd( d_4, sign );
			d_5 = _mm256_xor_pd( d_5, sign );
			d_6 = _mm256_xor_pd( d_6, sign );
			d_7 = _mm256_xor_pd( d_7, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am0[0] );
		a_4567 = _mm256_load_pd( &Am1[0] );
		b_0123 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[4] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[4] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Am1[8] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[12] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Am1[16] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );
			
			Am0 += 16;
			Am1 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0, e_1, e_2, e_3,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43;

	e_0 = _mm256_blend_pd( d_0, d_1, 0xa );
	e_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
	e_2 = _mm256_blend_pd( d_2, d_3, 0xa );
	e_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

	d_00 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_02 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_01 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_03 = _mm256_blend_pd( e_1, e_3, 0x3 );

	e_0 = _mm256_blend_pd( d_4, d_5, 0xa );
	e_1 = _mm256_blend_pd( d_4, d_5, 0x5 );
	e_2 = _mm256_blend_pd( d_6, d_7, 0xa );
	e_3 = _mm256_blend_pd( d_6, d_7, 0x5 );

	d_40 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_42 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_41 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_43 = _mm256_blend_pd( e_1, e_3, 0x3 );


	if(alg!=0)
		{

		e_0  = _mm256_load_pd( &C0[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*2] );
		d_02 = _mm256_add_pd( d_02, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*3] );
		d_03 = _mm256_add_pd( d_03, e_0 );

		e_0  = _mm256_load_pd( &C1[0+bs*0] );
		d_40 = _mm256_add_pd( d_40, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*1] );
		d_41 = _mm256_add_pd( d_41, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*2] );
		d_42 = _mm256_add_pd( d_42, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*3] );
		d_43 = _mm256_add_pd( d_43, e_0 );

		}
		

	// factorize
	__m128d
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
		sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
		d_40 = _mm256_mul_pd( d_40, a_00 );
		_mm256_store_pd( &D1[0+bs*0], d_40 );
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
		_mm256_store_pd( &D0[0+bs*0], a_00 );
		_mm256_store_pd( &D1[0+bs*0], a_00 );
		}

	// second row
	sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_10 );
	d_01  = _mm256_sub_pd( d_01, temp );
	temp  = _mm256_mul_pd( d_40, a_10 );
	d_41  = _mm256_sub_pd( d_41, temp );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
		d_41 = _mm256_mul_pd( d_41, a_11 );
		_mm256_store_pd( &D1[0+bs*1], d_41 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		a_11 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, a_11 );
		_mm256_store_pd( &D1[0+bs*1], a_11 );
		}

	// third row
	sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_20 = _mm_permute_pd( sa_20, 0x0 );
	a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_20 );
	d_02  = _mm256_sub_pd( d_02, temp );
	temp  = _mm256_mul_pd( d_40, a_20 );
	d_42  = _mm256_sub_pd( d_42, temp );
	sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	sa_21 = _mm_permute_pd( sa_21, 0x0 );
	a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_21 );
	d_02  = _mm256_sub_pd( d_02, temp );
	temp  = _mm256_mul_pd( d_41, a_21 );
	d_42  = _mm256_sub_pd( d_42, temp );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
		d_42 = _mm256_mul_pd( d_42, a_22 );
		_mm256_store_pd( &D1[0+bs*2], d_42 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		a_22 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, a_22 );
		_mm256_store_pd( &D1[0+bs*2], a_22 );
		}
	
	// fourth row
	sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_30 = _mm_permute_pd( sa_30, 0x3 );
	a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_30 );
	d_03  = _mm256_sub_pd( d_03, temp );
	temp  = _mm256_mul_pd( d_40, a_30 );
	d_43  = _mm256_sub_pd( d_43, temp );
	sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
	sa_31 = _mm_permute_pd( sa_31, 0x3 );
	a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_31 );
	d_03  = _mm256_sub_pd( d_03, temp );
	temp  = _mm256_mul_pd( d_41, a_31 );
	d_43  = _mm256_sub_pd( d_43, temp );
	sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	sa_32 = _mm_permute_pd( sa_32, 0x3 );
	a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
	temp  = _mm256_mul_pd( d_02, a_32 );
	d_03  = _mm256_sub_pd( d_03, temp );
	temp  = _mm256_mul_pd( d_42, a_32 );
	d_43  = _mm256_sub_pd( d_43, temp );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		sa_33 = _mm_movedup_pd( sa_33 );
		_mm_store_sd( &inv_diag_D[3], sa_33 );
		a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
		d_03  = _mm256_mul_pd( d_03, a_33 );
		_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
		d_43 = _mm256_mul_pd( d_43, a_33 );
		_mm256_store_pd( &D1[0+bs*3], d_43 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		a_33 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
		_mm256_maskstore_pd( &D0[0+bs*3], mask1, a_33 );
		_mm256_store_pd( &D1[0+bs*3], a_33 ); // a_00
		}
		
	return;

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
	
	__m256i
		mask0;

	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, zeros,
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	// compute store mask
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
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Ap0[0] );
		a_4567 = _mm256_load_pd( &Ap1[0] );
		b_0123 = _mm256_load_pd( &Bp[0] );

		if(tri_A==1)
			{

			if(kadd>=4)
				{

				zeros   = _mm256_setzero_pd();

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch

				// k = 1
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch

				// k = 2
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
				b_0123  = _mm256_blend_pd( zeros, b_0123, 0x7 ); // XXX
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );

				// k = 3
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
				a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );
				
				Ap0 += 16;
				Ap1 += 16;
				Bp  += 16;
				k   += 4;

				if(kadd>=8)
					{

					// k = 4
					a_4567  = _mm256_blend_pd( zeros, a_4567, 0x1 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
					b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
					d_5     = _mm256_add_pd( d_5, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
					d_7     = _mm256_add_pd( d_7, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_2301 );
					a_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
					d_6     = _mm256_add_pd( d_6, ab_tmp0 );

					// k = 5
					a_4567  = _mm256_blend_pd( zeros, a_4567, 0x3 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
					b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
					d_5     = _mm256_add_pd( d_5, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
					d_7     = _mm256_add_pd( d_7, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_2301 );
					a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
					d_6     = _mm256_add_pd( d_6, ab_tmp0 );

					// k = 6
					a_4567  = _mm256_blend_pd( zeros, a_4567, 0x7 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
					b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
					d_5     = _mm256_add_pd( d_5, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
					d_7     = _mm256_add_pd( d_7, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_2301 );
					a_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
					d_6     = _mm256_add_pd( d_6, ab_tmp0 );

					// k = 7
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
					b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
					d_5     = _mm256_add_pd( d_5, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
					d_7     = _mm256_add_pd( d_7, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_2301 );
					a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
					d_6     = _mm256_add_pd( d_6, ab_tmp0 );
						
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
						a_4567  = _mm256_blend_pd( zeros, a_4567, 0x1 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
						b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
						d_0     = _mm256_add_pd( d_0, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
						b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
						d_4     = _mm256_add_pd( d_4, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
						b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
						d_1     = _mm256_add_pd( d_1, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
						d_5     = _mm256_add_pd( d_5, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
						b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
						d_3     = _mm256_add_pd( d_3, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
						d_7     = _mm256_add_pd( d_7, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
						a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
						d_2     = _mm256_add_pd( d_2, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_4567, b_2301 );
						a_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
						d_6     = _mm256_add_pd( d_6, ab_tmp0 );

						k += 1;

						if(kadd>5)
							{

							// k = 5
							a_4567  = _mm256_blend_pd( zeros, a_4567, 0x3 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
							b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
							d_0     = _mm256_add_pd( d_0, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
							b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
							d_4     = _mm256_add_pd( d_4, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
							b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
							d_1     = _mm256_add_pd( d_1, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
							d_5     = _mm256_add_pd( d_5, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
							b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
							d_3     = _mm256_add_pd( d_3, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
							d_7     = _mm256_add_pd( d_7, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
							a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
							d_2     = _mm256_add_pd( d_2, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_4567, b_2301 );
							a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
							d_6     = _mm256_add_pd( d_6, ab_tmp0 );

							k += 1;

							if(kadd>6)
								{

								// k = 6
								a_4567  = _mm256_blend_pd( zeros, a_4567, 0x7 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
								b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
								d_0     = _mm256_add_pd( d_0, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
								b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
								d_4     = _mm256_add_pd( d_4, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
								b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
								d_1     = _mm256_add_pd( d_1, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
								d_5     = _mm256_add_pd( d_5, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
								b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
								d_3     = _mm256_add_pd( d_3, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
								d_7     = _mm256_add_pd( d_7, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
								a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
								d_2     = _mm256_add_pd( d_2, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_4567, b_2301 );
								a_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
								d_6     = _mm256_add_pd( d_6, ab_tmp0 );

								k   += 1;

								}

							}

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			Ap0 += 16;
			Ap1 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );

			
			Ap0 += 8;
			Ap1 += 8;
			Bp  += 8;
			k   += 2;

			}
		if(k<kadd)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
//			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
//			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
//			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );

		
//			Ap0 += 4; // keep it !!!
//			Ap1 += 4; // keep it !!!
//			Bp  += 4; // keep it !!!

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			d_0 = _mm256_xor_pd( d_0, sign );
			d_1 = _mm256_xor_pd( d_1, sign );
			d_2 = _mm256_xor_pd( d_2, sign );
			d_3 = _mm256_xor_pd( d_3, sign );
			d_4 = _mm256_xor_pd( d_4, sign );
			d_5 = _mm256_xor_pd( d_5, sign );
			d_6 = _mm256_xor_pd( d_6, sign );
			d_7 = _mm256_xor_pd( d_7, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am0[0] );
		a_4567 = _mm256_load_pd( &Am1[0] );
		b_0123 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[4] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[4] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Am1[8] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[12] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Am1[16] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7     = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6     = _mm256_sub_pd( d_6, ab_tmp0 );
			
			Am0 += 16;
			Am1 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0, e_1, e_2, e_3,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43;

	e_0 = _mm256_blend_pd( d_0, d_1, 0xa );
	e_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
	e_2 = _mm256_blend_pd( d_2, d_3, 0xa );
	e_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

	d_00 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_02 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_01 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_03 = _mm256_blend_pd( e_1, e_3, 0x3 );

	e_0 = _mm256_blend_pd( d_4, d_5, 0xa );
	e_1 = _mm256_blend_pd( d_4, d_5, 0x5 );
	e_2 = _mm256_blend_pd( d_6, d_7, 0xa );
	e_3 = _mm256_blend_pd( d_6, d_7, 0x5 );

	d_40 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_42 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_41 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_43 = _mm256_blend_pd( e_1, e_3, 0x3 );


	if(alg!=0)
		{

		e_0  = _mm256_load_pd( &C0[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*2] );
		d_02 = _mm256_add_pd( d_02, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*3] );
		d_03 = _mm256_add_pd( d_03, e_0 );

		e_0  = _mm256_load_pd( &C1[0+bs*0] );
		d_40 = _mm256_add_pd( d_40, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*1] );
		d_41 = _mm256_add_pd( d_41, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*2] );
		d_42 = _mm256_add_pd( d_42, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*3] );
		d_43 = _mm256_add_pd( d_43, e_0 );

		}
		

	// factorize
	__m128d
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
		mask0 = _mm256_loadu_si256( mask_bkp );
		sa_00 = _mm_div_sd( zeros_ones, sa_00 );
		sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
		d_40 = _mm256_mul_pd( d_40, a_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], mask0, d_40 );
		}
	else // comile
		{
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_00 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
		_mm256_store_pd( &D0[0+bs*0], a_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], mask0, a_00 );
		}

	// second row
	sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_10 );
	d_01  = _mm256_sub_pd( d_01, temp );
	temp  = _mm256_mul_pd( d_40, a_10 );
	d_41  = _mm256_sub_pd( d_41, temp );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
		d_41 = _mm256_mul_pd( d_41, a_11 );
		_mm256_maskstore_pd( &D1[0+bs*1], mask0, d_41 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_11 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, a_11 );
		_mm256_maskstore_pd( &D1[0+bs*1], mask0, a_11 );
		}

	// third row
	sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_20 = _mm_permute_pd( sa_20, 0x0 );
	a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_20 );
	d_02  = _mm256_sub_pd( d_02, temp );
	temp  = _mm256_mul_pd( d_40, a_20 );
	d_42  = _mm256_sub_pd( d_42, temp );
	sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	sa_21 = _mm_permute_pd( sa_21, 0x0 );
	a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_21 );
	d_02  = _mm256_sub_pd( d_02, temp );
	temp  = _mm256_mul_pd( d_41, a_21 );
	d_42  = _mm256_sub_pd( d_42, temp );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
		d_42 = _mm256_mul_pd( d_42, a_22 );
		_mm256_maskstore_pd( &D1[0+bs*2], mask0, d_42 );
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_22 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, a_22 );
		_mm256_maskstore_pd( &D1[0+bs*2], mask0, a_22 );
		}
	
	if(kn>=4)
		{

		// fourth row
		sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
		sa_30 = _mm_permute_pd( sa_30, 0x3 );
		a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
		temp  = _mm256_mul_pd( d_00, a_30 );
		d_03  = _mm256_sub_pd( d_03, temp );
		temp  = _mm256_mul_pd( d_40, a_30 );
		d_43  = _mm256_sub_pd( d_43, temp );
		sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
		sa_31 = _mm_permute_pd( sa_31, 0x3 );
		a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
		temp  = _mm256_mul_pd( d_01, a_31 );
		d_03  = _mm256_sub_pd( d_03, temp );
		temp  = _mm256_mul_pd( d_41, a_31 );
		d_43  = _mm256_sub_pd( d_43, temp );
		sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
		sa_32 = _mm_permute_pd( sa_32, 0x3 );
		a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
		temp  = _mm256_mul_pd( d_02, a_32 );
		d_03  = _mm256_sub_pd( d_03, temp );
		temp  = _mm256_mul_pd( d_42, a_32 );
		d_43  = _mm256_sub_pd( d_43, temp );
		sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
		sa_33 = _mm_permute_pd( sa_33, 0x3 );
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		if( _mm_comigt_sd ( sa_33, zeros_ones ) )
			{
			sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
			zeros_ones = _mm_set_sd( 1.0 );
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			sa_33 = _mm_div_sd( zeros_ones, sa_33 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			sa_33 = _mm_movedup_pd( sa_33 );
			_mm_store_sd( &inv_diag_D[3], sa_33 );
			a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
			d_03  = _mm256_mul_pd( d_03, a_33 );
			_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
			d_43 = _mm256_mul_pd( d_43, a_33 );
			_mm256_maskstore_pd( &D1[0+bs*3], mask0, d_43 );
			}
		else // comile
			{
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			a_33 = _mm256_setzero_pd( );
			_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
			_mm256_maskstore_pd( &D0[0+bs*3], mask1, a_33 );
			_mm256_maskstore_pd( &D1[0+bs*3], mask0, a_33 ); // a_00
			}

		}
		
	return;

	}



void kernel_dpotrf_nt_4x4_lib4_new(int ksub, double *Am0, double *Bm, int alg, double *C0, double *D0, double *inv_diag_D)
	{
	
	const int bs = 4;

	double d_temp;
	
	int k;
	
	__m256i
		mask0;

	__m256d
		a_0123, A_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, zeros,
		d_0, d_1, d_3, d_2;
	
	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();

	k = 0;

	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am0[0] );
		b_0123 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[4] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			
			Am0 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0, e_1, e_2, e_3,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43;

	e_0 = _mm256_blend_pd( d_0, d_1, 0xa );
	e_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
	e_2 = _mm256_blend_pd( d_2, d_3, 0xa );
	e_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

	d_00 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_02 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_01 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_03 = _mm256_blend_pd( e_1, e_3, 0x3 );

	if(alg!=0)
		{

		e_0  = _mm256_load_pd( &C0[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*2] );
		d_02 = _mm256_add_pd( d_02, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*3] );
		d_03 = _mm256_add_pd( d_03, e_0 );

		}
		

	// factorize
	__m128d
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
		sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_store_pd( &D0[0+bs*0], d_00 ); // a_00
		}
	else // comile
		{
		a_00 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
		_mm256_store_pd( &D0[0+bs*0], a_00 );
		}

	// second row
	sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_10 );
	d_01  = _mm256_sub_pd( d_01, temp );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, d_01 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		a_11 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
		_mm256_maskstore_pd( &D0[0+bs*1], mask1, a_11 );
		}

	// third row
	sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_20 = _mm_permute_pd( sa_20, 0x0 );
	a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_20 );
	d_02  = _mm256_sub_pd( d_02, temp );
	sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	sa_21 = _mm_permute_pd( sa_21, 0x0 );
	a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_21 );
	d_02  = _mm256_sub_pd( d_02, temp );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, d_02 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		a_22 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
		_mm256_maskstore_pd( &D0[0+bs*2], mask1, a_22 );
		}
	
	// fourth row
	sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_30 = _mm_permute_pd( sa_30, 0x3 );
	a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_30 );
	d_03  = _mm256_sub_pd( d_03, temp );
	sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
	sa_31 = _mm_permute_pd( sa_31, 0x3 );
	a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_31 );
	d_03  = _mm256_sub_pd( d_03, temp );
	sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	sa_32 = _mm_permute_pd( sa_32, 0x3 );
	a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
	temp  = _mm256_mul_pd( d_02, a_32 );
	d_03  = _mm256_sub_pd( d_03, temp );
	sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
	sa_33 = _mm_permute_pd( sa_33, 0x3 );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_33, zeros_ones ) )
		{
		sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		sa_33 = _mm_div_sd( zeros_ones, sa_33 );
		sa_33 = _mm_movedup_pd( sa_33 );
		_mm_store_sd( &inv_diag_D[3], sa_33 );
		a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
		d_03  = _mm256_mul_pd( d_03, a_33 );
		_mm256_maskstore_pd( &D0[0+bs*3], mask1, d_03 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
		a_33 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
		_mm256_maskstore_pd( &D0[0+bs*3], mask1, a_33 );
		}
	
	return;

	}



void kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap0, double *Bp, int ksub, double *Am0, double *Bm, int alg, double *C0, double *D0, double *inv_diag_D)
	{
	
	const int bs = 4;

	static __m256i mask_bkp[4];
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256i
		mask0;

	__m256d
		a_0123, A_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, zeros,
		d_0, d_1, d_3, d_2;
	
	// compute store mask
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
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Ap0[0] );
		b_0123 = _mm256_load_pd( &Bp[0] );

		if(tri_A==1)
			{

			if(kadd>=4)
				{

				zeros   = _mm256_setzero_pd();

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch

				// k = 1
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch

				// k = 2
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
				b_0123  = _mm256_blend_pd( zeros, b_0123, 0x7 ); // XXX
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );

				// k = 3
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );
				
				Ap0 += 16;
				Bp  += 16;
				k   += 4;

				if(kadd>=8)
					{

					// k = 4
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );

					// k = 5
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );

					// k = 6
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );

					// k = 7
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );
						
					Ap0 += 16;
					Bp  += 16;
					k   += 4;

					}
				else
					{

					if(kadd>4)
						{

						// k = 4
						ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
						b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
						d_0     = _mm256_add_pd( d_0, ab_tmp0 );
						b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
						ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
						b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
						d_1     = _mm256_add_pd( d_1, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
						b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
						d_3     = _mm256_add_pd( d_3, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
						a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
						d_2     = _mm256_add_pd( d_2, ab_tmp0 );

						k += 1;

						if(kadd>5)
							{

							// k = 5
							ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
							b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
							d_0     = _mm256_add_pd( d_0, ab_tmp0 );
							b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
							ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
							b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
							d_1     = _mm256_add_pd( d_1, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
							b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
							d_3     = _mm256_add_pd( d_3, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
							a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
							d_2     = _mm256_add_pd( d_2, ab_tmp0 );

							k += 1;

							if(kadd>6)
								{

								// k = 6
								ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
								b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
								d_0     = _mm256_add_pd( d_0, ab_tmp0 );
								b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
								ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
								b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
								d_1     = _mm256_add_pd( d_1, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
								b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
								d_3     = _mm256_add_pd( d_3, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
								a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
								d_2     = _mm256_add_pd( d_2, ab_tmp0 );

								k   += 1;

								}

							}

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			
			Ap0 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			
			
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );

			
			Ap0 += 8;
			Bp  += 8;
			k   += 2;

			}
		if(k<kadd)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
//			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
//			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );

		
//			Ap0 += 4;
//			Bp  += 4;

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			d_0 = _mm256_xor_pd( d_0, sign );
			d_1 = _mm256_xor_pd( d_1, sign );
			d_2 = _mm256_xor_pd( d_2, sign );
			d_3 = _mm256_xor_pd( d_3, sign );

			}

		}

	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am0[0] );
		b_0123 = _mm256_load_pd( &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[4] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3     = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2     = _mm256_sub_pd( d_2, ab_tmp0 );
			
			Am0 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0, e_1, e_2, e_3,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43;

	e_0 = _mm256_blend_pd( d_0, d_1, 0xa );
	e_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
	e_2 = _mm256_blend_pd( d_2, d_3, 0xa );
	e_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

	d_00 = _mm256_blend_pd( e_0, e_2, 0xc );
	d_02 = _mm256_blend_pd( e_0, e_2, 0x3 );
	d_01 = _mm256_blend_pd( e_1, e_3, 0xc );
	d_03 = _mm256_blend_pd( e_1, e_3, 0x3 );

	if(alg!=0)
		{

		e_0  = _mm256_load_pd( &C0[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*2] );
		d_02 = _mm256_add_pd( d_02, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*3] );
		d_03 = _mm256_add_pd( d_03, e_0 );

		}
		

	// factorize
	__m128d
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
		sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_maskstore_pd( &D0[0+bs*0], mask0, d_00 ); // a_00
		}
	else // comile
		{
		mask0 = _mm256_loadu_si256( mask_bkp );
		a_00 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128( a_00 ) );
		_mm256_maskstore_pd( &D0[0+bs*0], mask0, a_00 );
		}

	// second row
	sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
	a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_10 );
	d_01  = _mm256_sub_pd( d_01, temp );
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
	if( _mm_comigt_sd ( sa_11, zeros_ones ) )
		{
		sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		sa_11 = _mm_div_sd( zeros_ones, sa_11 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		sa_11 = _mm_movedup_pd( sa_11 );
		_mm_store_sd( &inv_diag_D[1], sa_11 );
		a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
		d_01  = _mm256_mul_pd( d_01, a_11 );
		_mm256_maskstore_pd( &D0[0+bs*1], mask0, d_01 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		a_11 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128( a_11 ) );
		_mm256_maskstore_pd( &D0[0+bs*1], mask0, a_11 );
		}

	// third row
	sa_20 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
	sa_20 = _mm_permute_pd( sa_20, 0x0 );
	a_20  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_20 ), _mm256_castpd128_pd256( sa_20 ), 0x0 );
	temp  = _mm256_mul_pd( d_00, a_20 );
	d_02  = _mm256_sub_pd( d_02, temp );
	sa_21 = _mm256_extractf128_pd( d_01, 0x1 ); // a_20 & a_30
	sa_21 = _mm_permute_pd( sa_21, 0x0 );
	a_21  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_21 ), _mm256_castpd128_pd256( sa_21 ), 0x0 );
	temp  = _mm256_mul_pd( d_01, a_21 );
	d_02  = _mm256_sub_pd( d_02, temp );
	sa_22 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	if( _mm_comigt_sd ( sa_22, zeros_ones ) )
		{
		sa_22 = _mm_sqrt_sd( sa_22, sa_22 );
		zeros_ones = _mm_set_sd( 1.0 );
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		sa_22 = _mm_div_sd( zeros_ones, sa_22 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		sa_22 = _mm_movedup_pd( sa_22 );
		_mm_store_sd( &inv_diag_D[2], sa_22 );
		a_22  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_22 ), _mm256_castpd128_pd256( sa_22 ), 0x0 );
		d_02  = _mm256_mul_pd( d_02, a_22 );
		_mm256_maskstore_pd( &D0[0+bs*2], mask0, d_02 ); // a_00
		}
	else // comile
		{
		mask1 = _mm256_set_epi64x( -1, -1, 1, 1 );
		mask0 = _mm256_loadu_si256( mask_bkp );
		mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
		a_22 = _mm256_setzero_pd( );
		_mm_store_sd( &inv_diag_D[2], _mm256_castpd256_pd128( a_22 ) );
		_mm256_maskstore_pd( &D0[0+bs*2], mask0, a_22 );
		}
	
	if(kn>=4)
		{

		// fourth row
		sa_30 = _mm256_extractf128_pd( d_00, 0x1 ); // a_20 & a_30
		sa_30 = _mm_permute_pd( sa_30, 0x3 );
		a_30  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_30 ), _mm256_castpd128_pd256( sa_30 ), 0x0 );
		temp  = _mm256_mul_pd( d_00, a_30 );
		d_03  = _mm256_sub_pd( d_03, temp );
		sa_31 = _mm256_extractf128_pd( d_01, 0x1 ); // a_21 & a_31
		sa_31 = _mm_permute_pd( sa_31, 0x3 );
		a_31  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_31 ), _mm256_castpd128_pd256( sa_31 ), 0x0 );
		temp  = _mm256_mul_pd( d_01, a_31 );
		d_03  = _mm256_sub_pd( d_03, temp );
		sa_32 = _mm256_extractf128_pd( d_02, 0x1 ); // a_22 & a_32
		sa_32 = _mm_permute_pd( sa_32, 0x3 );
		a_32  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_32 ), _mm256_castpd128_pd256( sa_32 ), 0x00 );
		temp  = _mm256_mul_pd( d_02, a_32 );
		d_03  = _mm256_sub_pd( d_03, temp );
		sa_33 = _mm256_extractf128_pd( d_03, 0x1 ); // a_33
		sa_33 = _mm_permute_pd( sa_33, 0x3 );
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		if( _mm_comigt_sd ( sa_33, zeros_ones ) )
			{
			sa_33 = _mm_sqrt_sd( sa_33, sa_33 );
			zeros_ones = _mm_set_sd( 1.0 );
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			sa_33 = _mm_div_sd( zeros_ones, sa_33 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			sa_33 = _mm_movedup_pd( sa_33 );
			_mm_store_sd( &inv_diag_D[3], sa_33 );
			a_33  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_33 ), _mm256_castpd128_pd256( sa_33 ), 0x00 );
			d_03  = _mm256_mul_pd( d_03, a_33 );
			_mm256_maskstore_pd( &D0[0+bs*3], mask0, d_03 ); // a_00
			}
		else // comile
			{
			mask1 = _mm256_set_epi64x( -1, 1, 1, 1 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			a_33 = _mm256_setzero_pd( );
			_mm_store_sd( &inv_diag_D[3], _mm256_castpd256_pd128( a_33 ) );
			_mm256_maskstore_pd( &D0[0+bs*3], mask0, a_33 );
			}

		}
	
	return;

	}



void kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{
	
	const int bs = 4;
	
	static __m256i mask_bkp[4];
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;
	
	__m256i
		mask0;

	__m256d
		a_0123,
		b_0101, b_1010,
		ab_temp, zeros,
		d_0, d_1, D_0, D_1;
	
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
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	D_0 = _mm256_setzero_pd();
	D_1 = _mm256_setzero_pd();
	
	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Ap[0] );
		b_0101 = _mm256_broadcast_pd( (__m128d *) &Bp[0] );

		if(tri_A==1)
			{

			zeros = _mm256_setzero_pd();

			if(kadd>=4)
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_temp = _mm256_mul_pd( a_0123, b_0101 );
				a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_temp );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
		
				// k = 1
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_temp = _mm256_mul_pd( a_0123, b_0101 );
				D_0     = _mm256_add_pd( D_0, ab_temp );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
				ab_temp = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
				D_1     = _mm256_add_pd( D_1, ab_temp );

				// k = 2
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
				ab_temp = _mm256_mul_pd( a_0123, b_0101 );
				d_0     = _mm256_add_pd( d_0, ab_temp );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
				ab_temp = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[12] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_temp );

				// k = 3
				ab_temp = _mm256_mul_pd( a_0123, b_0101 );
				D_0     = _mm256_add_pd( D_0, ab_temp );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
				ab_temp = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[16] ); // prefetch
				D_1     = _mm256_add_pd( D_1, ab_temp );
				
				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{


				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_temp = _mm256_mul_pd( a_0123, b_0101 );
				a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_temp );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch

				k  += 1;

				if(kadd>1)
					{
		

					// k = 1
					a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
					ab_temp = _mm256_mul_pd( a_0123, b_0101 );
					D_0     = _mm256_add_pd( D_0, ab_temp );
					b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
					ab_temp = _mm256_mul_pd( a_0123, b_1010 );
					a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
					D_1     = _mm256_add_pd( D_1, ab_temp );

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
						ab_temp = _mm256_mul_pd( a_0123, b_0101 );
						d_0     = _mm256_add_pd( d_0, ab_temp );
						b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
						b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
						ab_temp = _mm256_mul_pd( a_0123, b_1010 );
						a_0123  = _mm256_load_pd( &Ap[12] ); // prefetch
						d_1     = _mm256_add_pd( d_1, ab_temp );

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_temp );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_add_pd( D_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
			D_1     = _mm256_add_pd( D_1, ab_temp );


	/*	__builtin_prefetch( A+48 );*/
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[12] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_temp );


	/*	__builtin_prefetch( A+56 );*/
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_add_pd( D_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[16] ); // prefetch
			D_1     = _mm256_add_pd( D_1, ab_temp );
			
			Ap += 16;
			Bp += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_temp );
			
			
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_add_pd( D_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
			D_1     = _mm256_add_pd( D_1, ab_temp );
			
			
			Ap += 8;
			Bp += 8;
			k  += 2;

			}

		if(k<kadd)
			{
			
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	/*		b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch*/
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
	/*		a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch*/
			d_1     = _mm256_add_pd( d_1, ab_temp );
			
//			Ap += 4;
//			Bp += 4;

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m256d sign = _mm256_broadcast_sd( (double *) &long_sign );

			d_0 = _mm256_xor_pd( d_0, sign );
			d_1 = _mm256_xor_pd( d_1, sign );

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
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_sub_pd( d_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[4] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[4] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_temp );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_sub_pd( D_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[8] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[8] ); // prefetch
			D_1     = _mm256_sub_pd( D_1, ab_temp );


	/*	__builtin_prefetch( A+48 );*/
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_sub_pd( d_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[12] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[12] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_temp );


	/*	__builtin_prefetch( A+56 );*/
			ab_temp = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_sub_pd( D_0, ab_temp );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[16] ); // prefetch
			ab_temp = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[16] ); // prefetch
			D_1     = _mm256_sub_pd( D_1, ab_temp );
			
			Am += 16;
			Bm += 16;

			}

		}

	d_0 = _mm256_add_pd( d_0, D_0 );
	d_1 = _mm256_add_pd( d_1, D_1 );

	__m256d
		e_0, e_1,
		d_00, d_01;

	d_00 = _mm256_blend_pd( d_0, d_1, 0xa );
	d_01 = _mm256_blend_pd( d_0, d_1, 0x5 );

	if(alg!=0)
		{

		e_0  = _mm256_load_pd( &C[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, e_0 );
		e_0  = _mm256_load_pd( &C[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, e_0 );

		}
		

	
	// factorize
	__m128d
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
		sa_00 = _mm_movedup_pd( sa_00 );
		_mm_store_sd( &inv_diag_D[0], sa_00 );
		a_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_00 ), _mm256_castpd128_pd256( sa_00 ), 0x0 );
		d_00  = _mm256_mul_pd( d_00, a_00 );
		_mm256_maskstore_pd( &D[0+bs*0], mask0, d_00 ); // a_00
		}
	else // comile
		{
		a_00  = _mm256_setzero_pd();
		_mm_store_sd( &inv_diag_D[0], _mm256_castpd256_pd128(a_00) );
		_mm256_maskstore_pd( &D[0+bs*0], mask0, a_00 );
		}

	if(kn>=2)
		{

		// second row
		sa_10 = _mm_permute_pd( _mm256_castpd256_pd128(d_00), 0x3 );
		a_10 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_10 ), _mm256_castpd128_pd256( sa_10 ), 0x0 );
		temp  = _mm256_mul_pd( d_00, a_10 );
		d_01  = _mm256_sub_pd( d_01, temp );
		zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
		sa_11 = _mm_permute_pd( _mm256_castpd256_pd128(d_01), 0x3 );
		if( _mm_comigt_sd ( sa_11, zeros_ones ) )
			{
			sa_11 = _mm_sqrt_sd( sa_11, sa_11 );
			zeros_ones = _mm_set_sd( 1.0 );
			mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
			sa_11 = _mm_div_sd( zeros_ones, sa_11 );
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			sa_11 = _mm_movedup_pd( sa_11 );
			_mm_store_sd( &inv_diag_D[1], sa_11 );
			a_11  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( sa_11 ), _mm256_castpd128_pd256( sa_11 ), 0x0 );
			d_01  = _mm256_mul_pd( d_01, a_11 );
			_mm256_maskstore_pd( &D[0+bs*1], mask0, d_01 ); // a_00
			}
		else // comile
			{
			mask1 = _mm256_set_epi64x( -1, -1, -1, 1 ); // static memory and load instead ???
			mask0 = _mm256_loadu_si256( mask_bkp );
			mask0 = _mm256_castpd_si256( _mm256_and_pd( _mm256_castsi256_pd( mask0 ),  _mm256_castsi256_pd( mask1 ) ) );
			a_11  = _mm256_setzero_pd();
			_mm_store_sd( &inv_diag_D[1], _mm256_castpd256_pd128(a_11) );
			_mm256_maskstore_pd( &D[0+bs*1], mask0, a_11 );
			}

		}

	}



void kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;
	
	int k;
	
	__m128d
		a_01,
		b_01, b_10,
		ab_temp, zeros,
		d_0, d_1, D_0, D_1;
	
	// zero registers
	d_0 = _mm_setzero_pd();
	d_1 = _mm_setzero_pd();
	D_0 = _mm_setzero_pd();
	D_1 = _mm_setzero_pd();

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
				d_0     = _mm_add_sd( d_0, ab_temp );
				b_01    = _mm_load_pd( &Bp[4] ); // prefetch
				a_01    = _mm_load_pd( &Ap[4] ); // prefetch

				// k = 1
				ab_temp = _mm_mul_pd( a_01, b_01 );
				D_0     = _mm_add_pd( D_0, ab_temp );
				b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
				b_01    = _mm_load_pd( &Bp[8] ); // prefetch
				ab_temp = _mm_mul_pd( a_01, b_10 );
				a_01    = _mm_load_pd( &Ap[8] ); // prefetch
				D_1     = _mm_add_pd( D_1, ab_temp );

				Ap += 8;
				Bp += 8;
				k  += 2;

				}
			else
				{

				// k = 0
				ab_temp = _mm_mul_sd( a_01, b_01 );
				d_0     = _mm_add_sd( d_0, ab_temp );
				b_01    = _mm_load_pd( &Bp[4] ); // prefetch
				a_01    = _mm_load_pd( &Ap[4] ); // prefetch

				k  += 1;

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_temp = _mm_mul_pd( a_01, b_01 );
			d_0     = _mm_add_pd( d_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[4] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Ap[4] ); // prefetch
			d_1     = _mm_add_pd( d_1, ab_temp );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_temp = _mm_mul_pd( a_01, b_01 );
			D_0     = _mm_add_pd( D_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[8] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Ap[8] ); // prefetch
			D_1     = _mm_add_pd( D_1, ab_temp );


	/*	__builtin_prefetch( A+48 );*/
			ab_temp = _mm_mul_pd( a_01, b_01 );
			d_0     = _mm_add_pd( d_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[12] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Ap[12] ); // prefetch
			d_1     = _mm_add_pd( d_1, ab_temp );


	/*	__builtin_prefetch( A+56 );*/
			ab_temp = _mm_mul_pd( a_01, b_01 );
			D_0     = _mm_add_pd( D_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[16] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Ap[16] ); // prefetch
			D_1     = _mm_add_pd( D_1, ab_temp );
			
			Ap += 16;
			Bp += 16;

			}
		
		for(; k<kadd-1; k+=2)
			{
			
			ab_temp = _mm_mul_pd( a_01, b_01 );
			d_0     = _mm_add_pd( d_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[4] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Ap[4] ); // prefetch
			d_1     = _mm_add_pd( d_1, ab_temp );
			
			
			ab_temp = _mm_mul_pd( a_01, b_01 );
			D_0     = _mm_add_pd( D_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bp[8] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Ap[8] ); // prefetch
			D_1     = _mm_add_pd( D_1, ab_temp );
			
			
			Ap += 8;
			Bp += 8;

			}

		for(; k<kadd; k+=1)
			{
			
			ab_temp = _mm_mul_pd( a_01, b_01 );
			d_0     = _mm_add_pd( d_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
	/*		b_01    = _mm_load_pd( &Bp[4] ); // prefetch*/
			ab_temp = _mm_mul_pd( a_01, b_10 );
	/*		a_01    = _mm_load_pd( &Ap[4] ); // prefetch*/
			d_1     = _mm_add_pd( d_1, ab_temp );
			
//			Ap += 4; // keep it !!!
//			Bp += 4; // keep it !!!

			}

		if(alg==-1)
			{

			long long long_sign = 0x8000000000000000;
			__m128d sign = _mm_loaddup_pd( (double *) &long_sign );

			d_0 = _mm_xor_pd( d_0, sign );
			d_1 = _mm_xor_pd( d_1, sign );
			D_0 = _mm_xor_pd( D_0, sign );
			D_1 = _mm_xor_pd( D_1, sign );

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
			ab_temp = _mm_mul_pd( a_01, b_01 );
			d_0     = _mm_sub_pd( d_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[4] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Am[4] ); // prefetch
			d_1     = _mm_sub_pd( d_1, ab_temp );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_temp = _mm_mul_pd( a_01, b_01 );
			D_0     = _mm_sub_pd( D_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[8] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Am[8] ); // prefetch
			D_1     = _mm_sub_pd( D_1, ab_temp );


	/*	__builtin_prefetch( A+48 );*/
			ab_temp = _mm_mul_pd( a_01, b_01 );
			d_0     = _mm_sub_pd( d_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[12] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Am[12] ); // prefetch
			d_1     = _mm_sub_pd( d_1, ab_temp );


	/*	__builtin_prefetch( A+56 );*/
			ab_temp = _mm_mul_pd( a_01, b_01 );
			D_0     = _mm_sub_pd( D_0, ab_temp );
			b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
			b_01    = _mm_load_pd( &Bm[16] ); // prefetch
			ab_temp = _mm_mul_pd( a_01, b_10 );
			a_01    = _mm_load_pd( &Am[16] ); // prefetch
			D_1     = _mm_sub_pd( D_1, ab_temp );
			
			Am += 16;
			Bm += 16;

			}

		}

	d_0 = _mm_add_pd( d_0, D_0 );
	d_1 = _mm_add_pd( d_1, D_1 );

	__m128d
		e_0, e_1,
		d_00, d_01;

	d_00 = _mm_blend_pd( d_0, d_1, 0x2 );
	d_01 = _mm_blend_pd( d_0, d_1, 0x1 );

	if(alg!=0)
		{

		e_0  = _mm_load_pd( &C[0+bs*0] );
		d_00 = _mm_add_pd( d_00, e_0 );
		e_0  = _mm_load_pd( &C[0+bs*1] );
		d_01 = _mm_add_pd( d_01, e_0 );

		}
		

	__m128d
		zeros_ones, sab_temp,
		sa_00, sa_10, sa_11;


	// first row
	zeros_ones = _mm_set_sd( 1e-15 ); // 0.0 ???
	sa_00 = _mm_move_sd( sa_00, d_00 );
	if( _mm_comigt_sd ( sa_00, zeros_ones ) )
		{
		sa_00 = _mm_sqrt_sd( sa_00, sa_00 );
		sa_10 = _mm_shuffle_pd( d_00, zeros_ones, 0x1 );
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
		sa_11 = _mm_shuffle_pd( d_01, zeros_ones, 0x1 );
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





