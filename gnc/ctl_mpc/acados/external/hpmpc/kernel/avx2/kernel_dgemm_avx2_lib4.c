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

void kernel_dgemm_nt_12x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;
	
	const int bs = 4;

	int k;
	
	__m256d
		a_0, a_4, a_8,
		b_0,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82;
	
	__m256d
		e_00, e_01, e_02, e_03,
		e_10, e_12, e_50, e_52, e_90, e_92,
		e_40, e_41, e_42, e_43,
		e_80, e_81, e_82, e_83,
		c_10, c_20, c_30,
		c_50, c_60, c_70,
		c_90, c_a0, c_b0,
		d_10, d_20, d_30,
		d_50, d_60, d_70,
		d_90, d_a0, d_b0,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		d_80, d_81, d_82, d_83;

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

	if(kmax<=0)
		goto add;

	// prefetch
	a_0 = _mm256_load_pd( &A0[0] );
	b_0 = _mm256_load_pd( &B[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_8 = _mm256_load_pd( &A2[0] );

	for(k=0; k<kmax-3; k+=4)
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

	add:

	if(alg==0) // D = A * B'
		{
		if(td==0) // AB = A * B'
			{
			e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
			e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
			e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
			e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );

			c_00 = _mm256_blend_pd( e_00, e_02, 0xc );
			c_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
			c_01 = _mm256_blend_pd( e_01, e_03, 0xc );
			c_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

			_mm256_store_pd( &D0[0+bs*0], c_00 );
			_mm256_store_pd( &D0[0+bs*1], c_01 );
			_mm256_store_pd( &D0[0+bs*2], c_02 );
			_mm256_store_pd( &D0[0+bs*3], c_03 );

			e_40 = _mm256_blend_pd( c_40, c_41, 0xa );
			e_41 = _mm256_blend_pd( c_40, c_41, 0x5 );
			e_42 = _mm256_blend_pd( c_42, c_43, 0xa );
			e_43 = _mm256_blend_pd( c_42, c_43, 0x5 );

			c_40 = _mm256_blend_pd( e_40, e_42, 0xc );
			c_42 = _mm256_blend_pd( e_40, e_42, 0x3 );
			c_41 = _mm256_blend_pd( e_41, e_43, 0xc );
			c_43 = _mm256_blend_pd( e_41, e_43, 0x3 );

			_mm256_store_pd( &D1[0+bs*0], c_40 );
			_mm256_store_pd( &D1[0+bs*1], c_41 );
			_mm256_store_pd( &D1[0+bs*2], c_42 );
			_mm256_store_pd( &D1[0+bs*3], c_43 );

			e_80 = _mm256_blend_pd( c_80, c_81, 0xa );
			e_81 = _mm256_blend_pd( c_80, c_81, 0x5 );
			e_82 = _mm256_blend_pd( c_82, c_83, 0xa );
			e_83 = _mm256_blend_pd( c_82, c_83, 0x5 );
			
			c_80 = _mm256_blend_pd( e_80, e_82, 0xc );
			c_82 = _mm256_blend_pd( e_80, e_82, 0x3 );
			c_81 = _mm256_blend_pd( e_81, e_83, 0xc );
			c_83 = _mm256_blend_pd( e_81, e_83, 0x3 );

			_mm256_store_pd( &D2[0+bs*0], c_80 );
			_mm256_store_pd( &D2[0+bs*1], c_81 );
			_mm256_store_pd( &D2[0+bs*2], c_82 );
			_mm256_store_pd( &D2[0+bs*3], c_83 );
			}
		else // AB = t( A * B' )
			{
			e_00 = _mm256_unpacklo_pd( c_00, c_01 );
			e_10 = _mm256_unpackhi_pd( c_01, c_00 );
			e_02 = _mm256_unpacklo_pd( c_02, c_03 );
			e_12 = _mm256_unpackhi_pd( c_03, c_02 );

			c_00 = _mm256_permute2f128_pd( e_00, e_02, 0x20 );
			c_10 = _mm256_permute2f128_pd( e_10, e_12, 0x20 );
			c_20 = _mm256_permute2f128_pd( e_02, e_00, 0x31 );
			c_30 = _mm256_permute2f128_pd( e_12, e_10, 0x31 );

			_mm256_store_pd( &D0[0+bs*0], c_00 );
			_mm256_store_pd( &D0[0+bs*1], c_10 );
			_mm256_store_pd( &D0[0+bs*2], c_20 );
			_mm256_store_pd( &D0[0+bs*3], c_30 );

			e_40 = _mm256_unpacklo_pd( c_40, c_41 );
			e_50 = _mm256_unpackhi_pd( c_41, c_40 );
			e_42 = _mm256_unpacklo_pd( c_42, c_43 );
			e_52 = _mm256_unpackhi_pd( c_43, c_42 );

			c_40 = _mm256_permute2f128_pd( e_40, e_42, 0x20 );
			c_50 = _mm256_permute2f128_pd( e_50, e_52, 0x20 );
			c_60 = _mm256_permute2f128_pd( e_42, e_40, 0x31 );
			c_70 = _mm256_permute2f128_pd( e_52, e_50, 0x31 );

			_mm256_store_pd( &D0[0+bs*4], c_40 );
			_mm256_store_pd( &D0[0+bs*5], c_50 );
			_mm256_store_pd( &D0[0+bs*6], c_60 );
			_mm256_store_pd( &D0[0+bs*7], c_70 );

			e_80 = _mm256_unpacklo_pd( c_80, c_81 );
			e_90 = _mm256_unpackhi_pd( c_81, c_80 );
			e_82 = _mm256_unpacklo_pd( c_82, c_83 );
			e_92 = _mm256_unpackhi_pd( c_83, c_82 );

			c_80 = _mm256_permute2f128_pd( e_80, e_82, 0x20 );
			c_90 = _mm256_permute2f128_pd( e_90, e_92, 0x20 );
			c_a0 = _mm256_permute2f128_pd( e_82, e_80, 0x31 );
			c_b0 = _mm256_permute2f128_pd( e_92, e_90, 0x31 );

			_mm256_store_pd( &D0[0+bs*8], c_80 );
			_mm256_store_pd( &D0[0+bs*9], c_90 );
			_mm256_store_pd( &D0[0+bs*10], c_a0 );
			_mm256_store_pd( &D0[0+bs*11], c_b0 );
			}
		}
	else
		{
		if(tc==0) // C
			{
			e_00 = _mm256_blend_pd( c_00, c_01, 0xa );
			e_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
			e_02 = _mm256_blend_pd( c_02, c_03, 0xa );
			e_03 = _mm256_blend_pd( c_02, c_03, 0x5 );
			
			c_00 = _mm256_blend_pd( e_00, e_02, 0xc );
			c_02 = _mm256_blend_pd( e_00, e_02, 0x3 );
			c_01 = _mm256_blend_pd( e_01, e_03, 0xc );
			c_03 = _mm256_blend_pd( e_01, e_03, 0x3 );

			e_00 = _mm256_blend_pd( c_40, c_41, 0xa );
			e_01 = _mm256_blend_pd( c_40, c_41, 0x5 );
			e_02 = _mm256_blend_pd( c_42, c_43, 0xa );
			e_03 = _mm256_blend_pd( c_42, c_43, 0x5 );
			
			c_40 = _mm256_blend_pd( e_00, e_02, 0xc );
			c_42 = _mm256_blend_pd( e_00, e_02, 0x3 );
			c_41 = _mm256_blend_pd( e_01, e_03, 0xc );
			c_43 = _mm256_blend_pd( e_01, e_03, 0x3 );

			e_00 = _mm256_blend_pd( c_80, c_81, 0xa );
			e_01 = _mm256_blend_pd( c_80, c_81, 0x5 );
			e_02 = _mm256_blend_pd( c_82, c_83, 0xa );
			e_03 = _mm256_blend_pd( c_82, c_83, 0x5 );
			
			c_80 = _mm256_blend_pd( e_00, e_02, 0xc );
			c_82 = _mm256_blend_pd( e_00, e_02, 0x3 );
			c_81 = _mm256_blend_pd( e_01, e_03, 0xc );
			c_83 = _mm256_blend_pd( e_01, e_03, 0x3 );

			if(alg==1) // AB = A * B'
				{
				d_00 = _mm256_load_pd( &C0[0+bs*0] );
				d_01 = _mm256_load_pd( &C0[0+bs*1] );
				d_02 = _mm256_load_pd( &C0[0+bs*2] );
				d_03 = _mm256_load_pd( &C0[0+bs*3] );

				d_00 = _mm256_add_pd( d_00, c_00 );
				d_01 = _mm256_add_pd( d_01, c_01 );
				d_02 = _mm256_add_pd( d_02, c_02 );
				d_03 = _mm256_add_pd( d_03, c_03 );

				d_40 = _mm256_load_pd( &C1[0+bs*0] );
				d_41 = _mm256_load_pd( &C1[0+bs*1] );
				d_42 = _mm256_load_pd( &C1[0+bs*2] );
				d_43 = _mm256_load_pd( &C1[0+bs*3] );

				d_40 = _mm256_add_pd( d_40, c_40 );
				d_41 = _mm256_add_pd( d_41, c_41 );
				d_42 = _mm256_add_pd( d_42, c_42 );
				d_43 = _mm256_add_pd( d_43, c_43 );

				d_80 = _mm256_load_pd( &C2[0+bs*0] );
				d_81 = _mm256_load_pd( &C2[0+bs*1] );
				d_82 = _mm256_load_pd( &C2[0+bs*2] );
				d_83 = _mm256_load_pd( &C2[0+bs*3] );
		
				d_80 = _mm256_add_pd( d_80, c_80 );
				d_81 = _mm256_add_pd( d_81, c_81 );
				d_82 = _mm256_add_pd( d_82, c_82 );
				d_83 = _mm256_add_pd( d_83, c_83 );
				}
			else // AB = - A * B'
				{
				d_00 = _mm256_load_pd( &C0[0+bs*0] );
				d_01 = _mm256_load_pd( &C0[0+bs*1] );
				d_02 = _mm256_load_pd( &C0[0+bs*2] );
				d_03 = _mm256_load_pd( &C0[0+bs*3] );

				d_00 = _mm256_sub_pd( d_00, c_00 );
				d_01 = _mm256_sub_pd( d_01, c_01 );
				d_02 = _mm256_sub_pd( d_02, c_02 );
				d_03 = _mm256_sub_pd( d_03, c_03 );

				d_40 = _mm256_load_pd( &C1[0+bs*0] );
				d_41 = _mm256_load_pd( &C1[0+bs*1] );
				d_42 = _mm256_load_pd( &C1[0+bs*2] );
				d_43 = _mm256_load_pd( &C1[0+bs*3] );

				d_40 = _mm256_sub_pd( d_40, c_40 );
				d_41 = _mm256_sub_pd( d_41, c_41 );
				d_42 = _mm256_sub_pd( d_42, c_42 );
				d_43 = _mm256_sub_pd( d_43, c_43 );

				d_80 = _mm256_load_pd( &C2[0+bs*0] );
				d_81 = _mm256_load_pd( &C2[0+bs*1] );
				d_82 = _mm256_load_pd( &C2[0+bs*2] );
				d_83 = _mm256_load_pd( &C2[0+bs*3] );
		
				d_80 = _mm256_sub_pd( d_80, c_80 );
				d_81 = _mm256_sub_pd( d_81, c_81 );
				d_82 = _mm256_sub_pd( d_82, c_82 );
				d_83 = _mm256_sub_pd( d_83, c_83 );
				}

			if(td==0) // AB + C 
				{
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
			else // t(AB + C)
				{
				e_00 = _mm256_unpacklo_pd( d_00, d_01 );
				e_10 = _mm256_unpackhi_pd( d_00, d_01 );
				e_02 = _mm256_unpacklo_pd( d_02, d_03 );
				e_12 = _mm256_unpackhi_pd( d_02, d_03 );

				d_00 = _mm256_permute2f128_pd( e_00, e_02, 0x20 );
				d_20 = _mm256_permute2f128_pd( e_00, e_02, 0x31 );
				d_10 = _mm256_permute2f128_pd( e_10, e_12, 0x20 );
				d_30 = _mm256_permute2f128_pd( e_10, e_12, 0x31 );

				_mm256_store_pd( &D0[0+bs*0], d_00 );
				_mm256_store_pd( &D0[0+bs*1], d_10 );
				_mm256_store_pd( &D0[0+bs*2], d_20 );
				_mm256_store_pd( &D0[0+bs*3], d_30 );

				e_40 = _mm256_unpacklo_pd( d_40, d_41 );
				e_50 = _mm256_unpackhi_pd( d_40, d_41 );
				e_42 = _mm256_unpacklo_pd( d_42, d_43 );
				e_52 = _mm256_unpackhi_pd( d_42, d_43 );

				d_40 = _mm256_permute2f128_pd( e_40, e_42, 0x20 );
				d_60 = _mm256_permute2f128_pd( e_40, e_42, 0x31 );
				d_50 = _mm256_permute2f128_pd( e_50, e_52, 0x20 );
				d_70 = _mm256_permute2f128_pd( e_50, e_52, 0x31 );

				_mm256_store_pd( &D0[0+bs*4], d_40 );
				_mm256_store_pd( &D0[0+bs*5], d_50 );
				_mm256_store_pd( &D0[0+bs*6], d_60 );
				_mm256_store_pd( &D0[0+bs*7], d_70 );

				e_80 = _mm256_unpacklo_pd( d_80, d_81 );
				e_90 = _mm256_unpackhi_pd( d_80, d_81 );
				e_82 = _mm256_unpacklo_pd( d_82, d_83 );
				e_92 = _mm256_unpackhi_pd( d_82, d_83 );

				d_80 = _mm256_permute2f128_pd( e_80, e_82, 0x20 );
				d_90 = _mm256_permute2f128_pd( e_80, e_82, 0x31 );
				d_a0 = _mm256_permute2f128_pd( e_90, e_92, 0x20 );
				d_b0 = _mm256_permute2f128_pd( e_90, e_92, 0x31 );

				_mm256_store_pd( &D0[0+bs*8], d_80 );
				_mm256_store_pd( &D0[0+bs*9], d_90 );
				_mm256_store_pd( &D0[0+bs*10], d_a0 );
				_mm256_store_pd( &D0[0+bs*11], d_b0 );
				}
			}
		else // t(C)
			{

			e_00 = _mm256_unpacklo_pd( c_00, c_01 );
			e_10 = _mm256_unpackhi_pd( c_01, c_00 );
			e_02 = _mm256_unpacklo_pd( c_02, c_03 );
			e_12 = _mm256_unpackhi_pd( c_03, c_02 );

			c_00 = _mm256_permute2f128_pd( e_00, e_02, 0x20 );
			c_10 = _mm256_permute2f128_pd( e_10, e_12, 0x20 );
			c_20 = _mm256_permute2f128_pd( e_02, e_00, 0x31 );
			c_30 = _mm256_permute2f128_pd( e_12, e_10, 0x31 );

			e_40 = _mm256_unpacklo_pd( c_40, c_41 );
			e_50 = _mm256_unpackhi_pd( c_41, c_40 );
			e_42 = _mm256_unpacklo_pd( c_42, c_43 );
			e_52 = _mm256_unpackhi_pd( c_43, c_42 );

			c_40 = _mm256_permute2f128_pd( e_40, e_42, 0x20 );
			c_50 = _mm256_permute2f128_pd( e_50, e_52, 0x20 );
			c_60 = _mm256_permute2f128_pd( e_42, e_40, 0x31 );
			c_70 = _mm256_permute2f128_pd( e_52, e_50, 0x31 );

			e_80 = _mm256_unpacklo_pd( c_80, c_81 );
			e_90 = _mm256_unpackhi_pd( c_81, c_80 );
			e_82 = _mm256_unpacklo_pd( c_82, c_83 );
			e_92 = _mm256_unpackhi_pd( c_83, c_82 );

			c_80 = _mm256_permute2f128_pd( e_80, e_82, 0x20 );
			c_90 = _mm256_permute2f128_pd( e_90, e_92, 0x20 );
			c_a0 = _mm256_permute2f128_pd( e_82, e_80, 0x31 );
			c_b0 = _mm256_permute2f128_pd( e_92, e_90, 0x31 );

			if(alg==1) // AB = A*B'
				{
				d_00 = _mm256_load_pd( &C0[0+bs*0] );
				d_01 = _mm256_load_pd( &C0[0+bs*1] );
				d_02 = _mm256_load_pd( &C0[0+bs*2] );
				d_03 = _mm256_load_pd( &C0[0+bs*3] );

				d_00 = _mm256_add_pd( d_00, c_00 );
				d_01 = _mm256_add_pd( d_01, c_10 );
				d_02 = _mm256_add_pd( d_02, c_20 );
				d_03 = _mm256_add_pd( d_03, c_30 );

				d_40 = _mm256_load_pd( &C0[0+bs*4] );
				d_41 = _mm256_load_pd( &C0[0+bs*5] );
				d_42 = _mm256_load_pd( &C0[0+bs*6] );
				d_43 = _mm256_load_pd( &C0[0+bs*7] );

				d_40 = _mm256_add_pd( d_40, c_40 );
				d_41 = _mm256_add_pd( d_41, c_50 );
				d_42 = _mm256_add_pd( d_42, c_60 );
				d_43 = _mm256_add_pd( d_43, c_70 );

				d_80 = _mm256_load_pd( &C0[0+bs*8] );
				d_81 = _mm256_load_pd( &C0[0+bs*9] );
				d_82 = _mm256_load_pd( &C0[0+bs*10] );
				d_83 = _mm256_load_pd( &C0[0+bs*11] );

				d_80 = _mm256_add_pd( d_80, c_80 );
				d_81 = _mm256_add_pd( d_81, c_90 );
				d_82 = _mm256_add_pd( d_82, c_a0 );
				d_83 = _mm256_add_pd( d_83, c_b0 );
				}
			else // AB = - A*B'
				{
				d_00 = _mm256_load_pd( &C0[0+bs*0] );
				d_01 = _mm256_load_pd( &C0[0+bs*1] );
				d_02 = _mm256_load_pd( &C0[0+bs*2] );
				d_03 = _mm256_load_pd( &C0[0+bs*3] );

				d_00 = _mm256_sub_pd( d_00, c_00 );
				d_01 = _mm256_sub_pd( d_01, c_10 );
				d_02 = _mm256_sub_pd( d_02, c_20 );
				d_03 = _mm256_sub_pd( d_03, c_30 );

				d_40 = _mm256_load_pd( &C0[0+bs*4] );
				d_41 = _mm256_load_pd( &C0[0+bs*5] );
				d_42 = _mm256_load_pd( &C0[0+bs*6] );
				d_43 = _mm256_load_pd( &C0[0+bs*7] );

				d_40 = _mm256_sub_pd( d_40, c_40 );
				d_41 = _mm256_sub_pd( d_41, c_50 );
				d_42 = _mm256_sub_pd( d_42, c_60 );
				d_43 = _mm256_sub_pd( d_43, c_70 );

				d_80 = _mm256_load_pd( &C0[0+bs*8] );
				d_81 = _mm256_load_pd( &C0[0+bs*9] );
				d_82 = _mm256_load_pd( &C0[0+bs*10] );
				d_83 = _mm256_load_pd( &C0[0+bs*11] );

				d_80 = _mm256_sub_pd( d_80, c_80 );
				d_81 = _mm256_sub_pd( d_81, c_90 );
				d_82 = _mm256_sub_pd( d_82, c_a0 );
				d_83 = _mm256_sub_pd( d_83, c_b0 );
				}

			if(td==0) // t( t(AB) + C )
				{
				e_00 = _mm256_unpacklo_pd( d_00, d_01 );
				e_10 = _mm256_unpackhi_pd( d_00, d_01 );
				e_02 = _mm256_unpacklo_pd( d_02, d_03 );
				e_12 = _mm256_unpackhi_pd( d_02, d_03 );

				c_00 = _mm256_permute2f128_pd( e_00, e_02, 0x20 );
				c_20 = _mm256_permute2f128_pd( e_00, e_02, 0x31 );
				c_10 = _mm256_permute2f128_pd( e_10, e_12, 0x20 );
				c_30 = _mm256_permute2f128_pd( e_10, e_12, 0x31 );

				_mm256_store_pd( &D0[0+bs*0], c_00 );
				_mm256_store_pd( &D0[0+bs*1], c_10 );
				_mm256_store_pd( &D0[0+bs*2], c_20 );
				_mm256_store_pd( &D0[0+bs*3], c_30 );

				e_00 = _mm256_unpacklo_pd( d_40, d_41 );
				e_10 = _mm256_unpackhi_pd( d_40, d_41 );
				e_02 = _mm256_unpacklo_pd( d_42, d_43 );
				e_12 = _mm256_unpackhi_pd( d_42, d_43 );

				c_40 = _mm256_permute2f128_pd( e_00, e_02, 0x20 );
				c_60 = _mm256_permute2f128_pd( e_00, e_02, 0x31 );
				c_50 = _mm256_permute2f128_pd( e_10, e_12, 0x20 );
				c_70 = _mm256_permute2f128_pd( e_10, e_12, 0x31 );

				_mm256_store_pd( &D1[0+bs*0], c_40 );
				_mm256_store_pd( &D1[0+bs*1], c_50 );
				_mm256_store_pd( &D1[0+bs*2], c_60 );
				_mm256_store_pd( &D1[0+bs*3], c_70 );

				e_00 = _mm256_unpacklo_pd( d_80, d_81 );
				e_10 = _mm256_unpackhi_pd( d_80, d_81 );
				e_02 = _mm256_unpacklo_pd( d_82, d_83 );
				e_12 = _mm256_unpackhi_pd( d_82, d_83 );

				c_80 = _mm256_permute2f128_pd( e_00, e_02, 0x20 );
				c_a0 = _mm256_permute2f128_pd( e_00, e_02, 0x31 );
				c_90 = _mm256_permute2f128_pd( e_10, e_12, 0x20 );
				c_b0 = _mm256_permute2f128_pd( e_10, e_12, 0x31 );

				_mm256_store_pd( &D2[0+bs*0], c_80 );
				_mm256_store_pd( &D2[0+bs*1], c_90 );
				_mm256_store_pd( &D2[0+bs*2], c_a0 );
				_mm256_store_pd( &D2[0+bs*3], c_b0 );
				}
			else // t(AB) + C
				{
				_mm256_store_pd( &D0[0+bs*0], d_00 );
				_mm256_store_pd( &D0[0+bs*1], d_01 );
				_mm256_store_pd( &D0[0+bs*2], d_02 );
				_mm256_store_pd( &D0[0+bs*3], d_03 );

				_mm256_store_pd( &D0[0+bs*4], d_40 );
				_mm256_store_pd( &D0[0+bs*5], d_41 );
				_mm256_store_pd( &D0[0+bs*6], d_42 );
				_mm256_store_pd( &D0[0+bs*7], d_43 );

				_mm256_store_pd( &D0[0+bs*8], d_80 );
				_mm256_store_pd( &D0[0+bs*9], d_81 );
				_mm256_store_pd( &D0[0+bs*10], d_82 );
				_mm256_store_pd( &D0[0+bs*11], d_83 );
				}

			}

		}

	}



void kernel_dgemm_nt_12x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;
	
	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0, a_4, a_8,
		b_0,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42,
		c_80, c_81, c_83, c_82;
	
	__m256d
		e_0, e_1, e_2, e_3,
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		c_8, c_9, c_a, c_b,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9, d_a, d_b;

	__m256i 
		mask_m, mask_n;

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

	if(kmax<=0)
		goto add;

	// prefetch
	a_0 = _mm256_load_pd( &A0[0] );
	b_0 = _mm256_load_pd( &B[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_8 = _mm256_load_pd( &A2[0] );

	for(k=0; k<kmax-3; k+=4)
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

	add:

	if(alg==0) // D = A * B'
		{
		if(td==0) // AB = A * B'
			{
			e_0 = _mm256_blend_pd( c_00, c_01, 0xa );
			e_1 = _mm256_blend_pd( c_00, c_01, 0x5 );
			e_2 = _mm256_blend_pd( c_02, c_03, 0xa );
			e_3 = _mm256_blend_pd( c_02, c_03, 0x5 );

			d_0 = _mm256_blend_pd( e_0, e_2, 0xc );
			d_2 = _mm256_blend_pd( e_0, e_2, 0x3 );
			d_1 = _mm256_blend_pd( e_1, e_3, 0xc );
			d_3 = _mm256_blend_pd( e_1, e_3, 0x3 );

			e_0 = _mm256_blend_pd( c_40, c_41, 0xa );
			e_1 = _mm256_blend_pd( c_40, c_41, 0x5 );
			e_2 = _mm256_blend_pd( c_42, c_43, 0xa );
			e_3 = _mm256_blend_pd( c_42, c_43, 0x5 );

			d_4 = _mm256_blend_pd( e_0, e_2, 0xc );
			d_6 = _mm256_blend_pd( e_0, e_2, 0x3 );
			d_5 = _mm256_blend_pd( e_1, e_3, 0xc );
			d_7 = _mm256_blend_pd( e_1, e_3, 0x3 );

			e_0 = _mm256_blend_pd( c_80, c_81, 0xa );
			e_1 = _mm256_blend_pd( c_80, c_81, 0x5 );
			e_2 = _mm256_blend_pd( c_82, c_83, 0xa );
			e_3 = _mm256_blend_pd( c_82, c_83, 0x5 );
			
			d_8 = _mm256_blend_pd( e_0, e_2, 0xc );
			d_a = _mm256_blend_pd( e_0, e_2, 0x3 );
			d_9 = _mm256_blend_pd( e_1, e_3, 0xc );
			d_b = _mm256_blend_pd( e_1, e_3, 0x3 );

			goto store_n;
			}
		else // AB = t( A * B' )
			{
			e_0 = _mm256_unpacklo_pd( c_00, c_01 );
			e_1 = _mm256_unpackhi_pd( c_01, c_00 );
			e_2 = _mm256_unpacklo_pd( c_02, c_03 );
			e_3 = _mm256_unpackhi_pd( c_03, c_02 );

			d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			d_2 = _mm256_permute2f128_pd( e_2, e_0, 0x31 );
			d_3 = _mm256_permute2f128_pd( e_3, e_1, 0x31 );

			e_0 = _mm256_unpacklo_pd( c_40, c_41 );
			e_1 = _mm256_unpackhi_pd( c_41, c_40 );
			e_2 = _mm256_unpacklo_pd( c_42, c_43 );
			e_3 = _mm256_unpackhi_pd( c_43, c_42 );

			d_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			d_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			d_6 = _mm256_permute2f128_pd( e_2, e_0, 0x31 );
			d_7 = _mm256_permute2f128_pd( e_3, e_1, 0x31 );

			e_0 = _mm256_unpacklo_pd( c_80, c_81 );
			e_1 = _mm256_unpackhi_pd( c_81, c_80 );
			e_2 = _mm256_unpacklo_pd( c_82, c_83 );
			e_3 = _mm256_unpackhi_pd( c_83, c_82 );

			d_8 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			d_9 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			d_a = _mm256_permute2f128_pd( e_2, e_0, 0x31 );
			d_b = _mm256_permute2f128_pd( e_3, e_1, 0x31 );

			goto store_t;
			}
		}
	else
		{
		if(tc==0) // C
			{
			e_0 = _mm256_blend_pd( c_00, c_01, 0xa );
			e_1 = _mm256_blend_pd( c_00, c_01, 0x5 );
			e_2 = _mm256_blend_pd( c_02, c_03, 0xa );
			e_3 = _mm256_blend_pd( c_02, c_03, 0x5 );
			
			c_0 = _mm256_blend_pd( e_0, e_2, 0xc );
			c_2 = _mm256_blend_pd( e_0, e_2, 0x3 );
			c_1 = _mm256_blend_pd( e_1, e_3, 0xc );
			c_3 = _mm256_blend_pd( e_1, e_3, 0x3 );

			e_0 = _mm256_blend_pd( c_40, c_41, 0xa );
			e_1 = _mm256_blend_pd( c_40, c_41, 0x5 );
			e_2 = _mm256_blend_pd( c_42, c_43, 0xa );
			e_3 = _mm256_blend_pd( c_42, c_43, 0x5 );
			
			c_4 = _mm256_blend_pd( e_0, e_2, 0xc );
			c_6 = _mm256_blend_pd( e_0, e_2, 0x3 );
			c_5 = _mm256_blend_pd( e_1, e_3, 0xc );
			c_7 = _mm256_blend_pd( e_1, e_3, 0x3 );

			e_0 = _mm256_blend_pd( c_80, c_81, 0xa );
			e_1 = _mm256_blend_pd( c_80, c_81, 0x5 );
			e_2 = _mm256_blend_pd( c_82, c_83, 0xa );
			e_3 = _mm256_blend_pd( c_82, c_83, 0x5 );
			
			c_8 = _mm256_blend_pd( e_0, e_2, 0xc );
			c_a = _mm256_blend_pd( e_0, e_2, 0x3 );
			c_9 = _mm256_blend_pd( e_1, e_3, 0xc );
			c_b = _mm256_blend_pd( e_1, e_3, 0x3 );

			if(alg==1) // AB = A * B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+bs*0] );
				d_5 = _mm256_load_pd( &C1[0+bs*1] );
				d_6 = _mm256_load_pd( &C1[0+bs*2] );
				d_7 = _mm256_load_pd( &C1[0+bs*3] );

				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C2[0+bs*0] );
				d_9 = _mm256_load_pd( &C2[0+bs*1] );
				d_a = _mm256_load_pd( &C2[0+bs*2] );
				d_b = _mm256_load_pd( &C2[0+bs*3] );
		
				d_8 = _mm256_add_pd( d_8, c_8 );
				d_9 = _mm256_add_pd( d_9, c_9 );
				d_a = _mm256_add_pd( d_a, c_a );
				d_b = _mm256_add_pd( d_b, c_b );
				}
			else // AB = - A * B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+bs*0] );
				d_5 = _mm256_load_pd( &C1[0+bs*1] );
				d_6 = _mm256_load_pd( &C1[0+bs*2] );
				d_7 = _mm256_load_pd( &C1[0+bs*3] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C2[0+bs*0] );
				d_9 = _mm256_load_pd( &C2[0+bs*1] );
				d_a = _mm256_load_pd( &C2[0+bs*2] );
				d_b = _mm256_load_pd( &C2[0+bs*3] );
		
				d_8 = _mm256_sub_pd( d_8, c_8 );
				d_9 = _mm256_sub_pd( d_9, c_9 );
				d_a = _mm256_sub_pd( d_a, c_a );
				d_b = _mm256_sub_pd( d_b, c_b );
				}

			if(td==0) // AB + C 
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				e_0 = _mm256_unpacklo_pd( d_0, d_1 );
				e_1 = _mm256_unpackhi_pd( d_0, d_1 );
				e_2 = _mm256_unpacklo_pd( d_2, d_3 );
				e_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				e_0 = _mm256_unpacklo_pd( d_4, d_5 );
				e_1 = _mm256_unpackhi_pd( d_4, d_5 );
				e_2 = _mm256_unpacklo_pd( d_6, d_7 );
				e_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				e_0 = _mm256_unpacklo_pd( d_8, d_9 );
				e_1 = _mm256_unpackhi_pd( d_8, d_9 );
				e_2 = _mm256_unpacklo_pd( d_a, d_b );
				e_3 = _mm256_unpackhi_pd( d_a, d_b );

				d_8 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_9 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_a = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_b = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				goto store_t;
				}
			}
		else // t(C)
			{

			e_0 = _mm256_unpacklo_pd( c_00, c_01 );
			e_1 = _mm256_unpackhi_pd( c_01, c_00 );
			e_2 = _mm256_unpacklo_pd( c_02, c_03 );
			e_3 = _mm256_unpackhi_pd( c_03, c_02 );

			c_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			c_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			c_2 = _mm256_permute2f128_pd( e_2, e_0, 0x31 );
			c_3 = _mm256_permute2f128_pd( e_3, e_1, 0x31 );

			e_0 = _mm256_unpacklo_pd( c_40, c_41 );
			e_1 = _mm256_unpackhi_pd( c_41, c_40 );
			e_2 = _mm256_unpacklo_pd( c_42, c_43 );
			e_3 = _mm256_unpackhi_pd( c_43, c_42 );

			c_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			c_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			c_6 = _mm256_permute2f128_pd( e_2, e_0, 0x31 );
			c_7 = _mm256_permute2f128_pd( e_3, e_1, 0x31 );

			e_0 = _mm256_unpacklo_pd( c_80, c_81 );
			e_1 = _mm256_unpackhi_pd( c_81, c_80 );
			e_2 = _mm256_unpacklo_pd( c_82, c_83 );
			e_3 = _mm256_unpackhi_pd( c_83, c_82 );

			c_8 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			c_9 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			c_a = _mm256_permute2f128_pd( e_2, e_0, 0x31 );
			c_b = _mm256_permute2f128_pd( e_3, e_1, 0x31 );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+bs*4] );
				d_5 = _mm256_load_pd( &C0[0+bs*5] );
				d_6 = _mm256_load_pd( &C0[0+bs*6] );
				d_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C0[0+bs*8] );
				d_9 = _mm256_load_pd( &C0[0+bs*9] );
				d_a = _mm256_load_pd( &C0[0+bs*10] );
				d_b = _mm256_load_pd( &C0[0+bs*11] );

				d_8 = _mm256_add_pd( d_8, c_8 );
				d_9 = _mm256_add_pd( d_9, c_9 );
				d_a = _mm256_add_pd( d_a, c_a );
				d_b = _mm256_add_pd( d_b, c_b );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+bs*4] );
				d_5 = _mm256_load_pd( &C0[0+bs*5] );
				d_6 = _mm256_load_pd( &C0[0+bs*6] );
				d_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C0[0+bs*8] );
				d_9 = _mm256_load_pd( &C0[0+bs*9] );
				d_a = _mm256_load_pd( &C0[0+bs*10] );
				d_b = _mm256_load_pd( &C0[0+bs*11] );

				d_8 = _mm256_sub_pd( d_8, c_8 );
				d_9 = _mm256_sub_pd( d_9, c_9 );
				d_a = _mm256_sub_pd( d_a, c_a );
				d_b = _mm256_sub_pd( d_b, c_b );
				}

			if(td==0) // t( t(AB) + C )
				{
				e_0 = _mm256_unpacklo_pd( d_0, d_1 );
				e_1 = _mm256_unpackhi_pd( d_0, d_1 );
				e_2 = _mm256_unpacklo_pd( d_2, d_3 );
				e_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				e_0 = _mm256_unpacklo_pd( d_4, d_5 );
				e_1 = _mm256_unpackhi_pd( d_4, d_5 );
				e_2 = _mm256_unpacklo_pd( d_6, d_7 );
				e_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				e_0 = _mm256_unpacklo_pd( d_8, d_9 );
				e_1 = _mm256_unpackhi_pd( d_8, d_9 );
				e_2 = _mm256_unpacklo_pd( d_a, d_b );
				e_3 = _mm256_unpackhi_pd( d_a, d_b );

				d_8 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_a = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_9 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_b = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}

		}

	store_n:
	d_temp = km - 8.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );

	_mm256_store_pd( &D1[0+bs*0], d_4 );
	_mm256_store_pd( &D1[0+bs*1], d_5 );
	_mm256_store_pd( &D1[0+bs*2], d_6 );

	_mm256_maskstore_pd( &D2[0+bs*0], mask_m, d_8 );
	_mm256_maskstore_pd( &D2[0+bs*1], mask_m, d_9 );
	_mm256_maskstore_pd( &D2[0+bs*2], mask_m, d_a );

	if(kn>=4)
		{
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_store_pd( &D1[0+bs*3], d_7 );
		_mm256_maskstore_pd( &D2[0+bs*3], mask_m, d_b );
		}
	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );

	_mm256_maskstore_pd( &D0[0+bs*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask_n, d_3 );

	_mm256_maskstore_pd( &D0[0+bs*4], mask_n, d_4 );
	_mm256_maskstore_pd( &D0[0+bs*5], mask_n, d_5 );
	_mm256_maskstore_pd( &D0[0+bs*6], mask_n, d_6 );
	_mm256_maskstore_pd( &D0[0+bs*7], mask_n, d_7 );

	if(km>=12)
		{
		_mm256_maskstore_pd( &D0[0+bs*8], mask_n, d_8 );
		_mm256_maskstore_pd( &D0[0+bs*9], mask_n, d_9 );
		_mm256_maskstore_pd( &D0[0+bs*10], mask_n, d_a );
		_mm256_maskstore_pd( &D0[0+bs*11], mask_n, d_b );
		}
	else
		{
		_mm256_maskstore_pd( &D0[0+bs*8], mask_n, d_8 );
		if(km>=10)
			{
			_mm256_maskstore_pd( &D0[0+bs*9], mask_n, d_9 );
			if(km>10)
				{
				_mm256_maskstore_pd( &D0[0+bs*10], mask_n, d_a );
				}
			}
		}
		
	return;



	}



void kernel_dgemm_nt_8x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0, a_4, A_0, A_4,
		b_0, b_1, b_2,
		c_00, c_01, c_03, c_02,
		c_40, c_41, c_43, c_42;
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		e_0, e_1, e_2, e_3;

	__m256i 
		mask_m, mask_n;

	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	c_40 = _mm256_setzero_pd();
	c_41 = _mm256_setzero_pd();
	c_43 = _mm256_setzero_pd();
	c_42 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	b_0 = _mm256_broadcast_pd( (__m128d *) &B[0] );
	b_2 = _mm256_broadcast_pd( (__m128d *) &B[2] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		A_0  = _mm256_load_pd( &A0[4] ); // prefetch
		A_4  = _mm256_load_pd( &A1[4] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_1, c_41 );
		b_1  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_42 = _mm256_fmadd_pd( a_4, b_2, c_42 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[6] ); // prefetch
		c_03 = _mm256_fmadd_pd( a_0, b_1, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_1, c_43 );
		
/*	__builtin_prefetch( A+40 );*/
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( A_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( A_4, b_0, c_40 );
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		c_01 = _mm256_fmadd_pd( A_0, b_1, c_01 );
		c_41 = _mm256_fmadd_pd( A_4, b_1, c_41 );
		b_1  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( A_0, b_2, c_02 );
		c_42 = _mm256_fmadd_pd( A_4, b_2, c_42 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[10] ); // prefetch
		c_03 = _mm256_fmadd_pd( A_0, b_1, c_03 );
		c_43 = _mm256_fmadd_pd( A_4, b_1, c_43 );
	
/*	__builtin_prefetch( A+48 );*/
		A_0  = _mm256_load_pd( &A0[12] ); // prefetch
		A_4  = _mm256_load_pd( &A1[12] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_1, c_41 );
		b_1  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_42 = _mm256_fmadd_pd( a_4, b_2, c_42 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[14] ); // prefetch
		c_03 = _mm256_fmadd_pd( a_0, b_1, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_1, c_43 );
	
/*	__builtin_prefetch( A+56 );*/
		a_0  = _mm256_load_pd( &A0[16] ); // prefetch
		a_4  = _mm256_load_pd( &A1[16] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( A_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( A_4, b_0, c_40 );
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		c_01 = _mm256_fmadd_pd( A_0, b_1, c_01 );
		c_41 = _mm256_fmadd_pd( A_4, b_1, c_41 );
		b_1  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( A_0, b_2, c_02 );
		c_42 = _mm256_fmadd_pd( A_4, b_2, c_42 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[18] ); // prefetch
		c_03 = _mm256_fmadd_pd( A_0, b_1, c_03 );
		c_43 = _mm256_fmadd_pd( A_4, b_1, c_43 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
		A_0  = _mm256_load_pd( &A0[4] ); // prefetch
		A_4  = _mm256_load_pd( &A1[4] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_1, c_41 );
		b_1  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_42 = _mm256_fmadd_pd( a_4, b_2, c_42 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[6] ); // prefetch
		c_03 = _mm256_fmadd_pd( a_0, b_1, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_1, c_43 );
		
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( A_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( A_4, b_0, c_40 );
		b_0  = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		c_01 = _mm256_fmadd_pd( A_0, b_1, c_01 );
		c_41 = _mm256_fmadd_pd( A_4, b_1, c_41 );
		b_1  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( A_0, b_2, c_02 );
		c_42 = _mm256_fmadd_pd( A_4, b_2, c_42 );
		b_2  = _mm256_broadcast_pd( (__m128d *) &B[10] ); // prefetch
		c_03 = _mm256_fmadd_pd( A_0, b_1, c_03 );
		c_43 = _mm256_fmadd_pd( A_4, b_1, c_43 );
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
//		A_0  = _mm256_load_pd( &A0[4] ); // prefetch
//		A_4  = _mm256_load_pd( &A1[4] ); // prefetch
		b_1  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
		c_40 = _mm256_fmadd_pd( a_4, b_0, c_40 );
//		b_0  = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01 = _mm256_fmadd_pd( a_0, b_1, c_01 );
		c_41 = _mm256_fmadd_pd( a_4, b_1, c_41 );
		b_1  = _mm256_shuffle_pd( b_2, b_2, 0x5 );
		c_02 = _mm256_fmadd_pd( a_0, b_2, c_02 );
		c_42 = _mm256_fmadd_pd( a_4, b_2, c_42 );
//		b_2  = _mm256_broadcast_pd( (__m128d *) &B[6] ); // prefetch
		c_03 = _mm256_fmadd_pd( a_0, b_1, c_03 );
		c_43 = _mm256_fmadd_pd( a_4, b_1, c_43 );
		
		}

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			d_0 = _mm256_blend_pd( c_00, c_01, 0xa );
			d_1 = _mm256_blend_pd( c_00, c_01, 0x5 );
			d_2 = _mm256_blend_pd( c_02, c_03, 0xa );
			d_3 = _mm256_blend_pd( c_02, c_03, 0x5 );

			d_4 = _mm256_blend_pd( c_40, c_41, 0xa );
			d_5 = _mm256_blend_pd( c_40, c_41, 0x5 );
			d_6 = _mm256_blend_pd( c_42, c_43, 0xa );
			d_7 = _mm256_blend_pd( c_42, c_43, 0x5 );

			goto store_n;
			}
		else // transposed
			{
			e_0 = _mm256_unpacklo_pd( c_00, c_01 );
			e_1 = _mm256_unpackhi_pd( c_01, c_00 );
			e_2 = _mm256_unpacklo_pd( c_02, c_03 );
			e_3 = _mm256_unpackhi_pd( c_03, c_02 );

			d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			d_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
			d_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

			e_0 = _mm256_unpacklo_pd( c_40, c_41 );
			e_1 = _mm256_unpackhi_pd( c_41, c_40 );
			e_2 = _mm256_unpacklo_pd( c_42, c_43 );
			e_3 = _mm256_unpackhi_pd( c_43, c_42 );

			d_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			d_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			d_6 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
			d_7 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C

			c_0 = _mm256_blend_pd( c_00, c_01, 0xa );
			c_1 = _mm256_blend_pd( c_00, c_01, 0x5 );
			c_2 = _mm256_blend_pd( c_02, c_03, 0xa );
			c_3 = _mm256_blend_pd( c_02, c_03, 0x5 );

			c_4 = _mm256_blend_pd( c_40, c_41, 0xa );
			c_5 = _mm256_blend_pd( c_40, c_41, 0x5 );
			c_6 = _mm256_blend_pd( c_42, c_43, 0xa );
			c_7 = _mm256_blend_pd( c_42, c_43, 0x5 );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );
			
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+bs*0] );
				d_5 = _mm256_load_pd( &C1[0+bs*1] );
				d_6 = _mm256_load_pd( &C1[0+bs*2] );
				d_7 = _mm256_load_pd( &C1[0+bs*3] );
			
				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );
			
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+bs*0] );
				d_5 = _mm256_load_pd( &C1[0+bs*1] );
				d_6 = _mm256_load_pd( &C1[0+bs*2] );
				d_7 = _mm256_load_pd( &C1[0+bs*3] );
			
				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );
				}

			if(td==0) // AB + C 
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				e_0 = _mm256_unpacklo_pd( d_0, d_1 );
				e_1 = _mm256_unpackhi_pd( d_0, d_1 );
				e_2 = _mm256_unpacklo_pd( d_2, d_3 );
				e_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				e_0 = _mm256_unpacklo_pd( d_4, d_5 );
				e_1 = _mm256_unpackhi_pd( d_4, d_5 );
				e_2 = _mm256_unpacklo_pd( d_6, d_7 );
				e_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				goto store_t;
				}

			}
		else // t(C)
			{

			e_0 = _mm256_unpacklo_pd( c_00, c_01 );
			e_1 = _mm256_unpackhi_pd( c_01, c_00 );
			e_2 = _mm256_unpacklo_pd( c_02, c_03 );
			e_3 = _mm256_unpackhi_pd( c_03, c_02 );

			c_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			c_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			c_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
			c_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

			e_0 = _mm256_unpacklo_pd( c_40, c_41 );
			e_1 = _mm256_unpackhi_pd( c_41, c_40 );
			e_2 = _mm256_unpacklo_pd( c_42, c_43 );
			e_3 = _mm256_unpackhi_pd( c_43, c_42 );

			c_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			c_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			c_6 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
			c_7 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+bs*4] );
				d_5 = _mm256_load_pd( &C0[0+bs*5] );
				d_6 = _mm256_load_pd( &C0[0+bs*6] );
				d_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+bs*0] );
				d_1 = _mm256_load_pd( &C0[0+bs*1] );
				d_2 = _mm256_load_pd( &C0[0+bs*2] );
				d_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+bs*4] );
				d_5 = _mm256_load_pd( &C0[0+bs*5] );
				d_6 = _mm256_load_pd( &C0[0+bs*6] );
				d_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );
				}

			if(td==0) // t( t(AB) + C )
				{
				e_0 = _mm256_unpacklo_pd( d_0, d_1 );
				e_1 = _mm256_unpackhi_pd( d_0, d_1 );
				e_2 = _mm256_unpacklo_pd( d_2, d_3 );
				e_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				e_0 = _mm256_unpacklo_pd( d_4, d_5 );
				e_1 = _mm256_unpackhi_pd( d_4, d_5 );
				e_2 = _mm256_unpacklo_pd( d_6, d_7 );
				e_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}

		}

	store_n:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );

	_mm256_maskstore_pd( &D1[0+bs*0], mask_m, d_4 );
	_mm256_maskstore_pd( &D1[0+bs*1], mask_m, d_5 );
	_mm256_maskstore_pd( &D1[0+bs*2], mask_m, d_6 );

	if(kn>=4)
		{
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_maskstore_pd( &D1[0+bs*3], mask_m, d_7 );
		}

	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );
	
	_mm256_maskstore_pd( &D0[0+bs*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask_n, d_3 );

	if(km>=8)
		{
		_mm256_maskstore_pd( &D0[0+bs*4], mask_n, d_4 );
		_mm256_maskstore_pd( &D0[0+bs*5], mask_n, d_5 );
		_mm256_maskstore_pd( &D0[0+bs*6], mask_n, d_6 );
		_mm256_maskstore_pd( &D0[0+bs*7], mask_n, d_7 );
		}
	else
		{
		_mm256_maskstore_pd( &D0[0+bs*4], mask_n, d_4 );
		if(km>=6)
			{
			_mm256_maskstore_pd( &D0[0+bs*5], mask_n, d_5 );
			if(km>6)
				{
				_mm256_maskstore_pd( &D0[0+bs*6], mask_n, d_6 );
				}
			}
		}

	}



void kernel_dgemm_nt_8x2_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_4567, //A_0123,
		b_0101, b_1010,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_20_31, c_01_10_21_30,
		c_40_51_60_71, c_41_50_61_70,
		C_00_11_20_31, C_01_10_21_30,
		C_40_51_60_71, C_41_50_61_70;
	
	__m128d
		u_0, u_1, u_2, u_3,
		u_4, u_5, u_6, u_7,
		v_0, v_1, v_2, v_3,
		v_4, v_5, v_6, v_7;

	__m256d
		e_0, e_1, e_4, e_5,
		c_0, c_1, c_4, c_5,
		d_0, d_1, d_4, d_5;

	__m256i 
		mask_m, mask_n;

	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );
	
	// zero registers
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	c_40_51_60_71 = _mm256_setzero_pd();
	c_41_50_61_70 = _mm256_setzero_pd();
	C_00_11_20_31 = _mm256_setzero_pd();
	C_01_10_21_30 = _mm256_setzero_pd();
	C_40_51_60_71 = _mm256_setzero_pd();
	C_41_50_61_70 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
/*	__builtin_prefetch( A+40 );*/
		C_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, C_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		C_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, C_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		C_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, C_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		C_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, C_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	

/*	__builtin_prefetch( A+48 );*/
		c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	

/*	__builtin_prefetch( A+56 );*/
		C_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, C_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		C_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, C_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		C_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, C_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		C_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, C_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		

		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
		c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
		C_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, C_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		C_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, C_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		C_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, C_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		C_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, C_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}
	
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, C_00_11_20_31 );
	c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, C_40_51_60_71 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, C_01_10_21_30 );
	c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, C_41_50_61_70 );

	if(kmax%2==1)
		{
		
		c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		//b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		//a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_41_50_61_70 = _mm256_fmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		//a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		}

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // AB = A * B'
			{
			d_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
			d_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
			d_4 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0xa );
			d_5 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0x5 );

			goto store_n;
			}
		else // AB = t( A * B' )
			{
			e_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			e_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );

			u_2 = _mm256_extractf128_pd( e_0, 0x1 );
			u_0 = _mm256_castpd256_pd128( e_0 );
			u_3 = _mm256_extractf128_pd( e_1, 0x1 );
			u_1 = _mm256_castpd256_pd128( e_1 );

			e_0 = _mm256_unpacklo_pd( c_40_51_60_71, c_41_50_61_70 );
			e_1 = _mm256_unpackhi_pd( c_41_50_61_70, c_40_51_60_71 );

			u_6 = _mm256_extractf128_pd( e_0, 0x1 );
			u_4 = _mm256_castpd256_pd128( e_0 );
			u_7 = _mm256_extractf128_pd( e_1, 0x1 );
			u_5 = _mm256_castpd256_pd128( e_1 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{
			c_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
			c_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
			c_4 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0xa );
			c_5 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0x5 );

			d_0 = _mm256_load_pd( &C0[0+bs*0] );
			d_1 = _mm256_load_pd( &C0[0+bs*1] );
			d_4 = _mm256_load_pd( &C1[0+bs*0] );
			d_5 = _mm256_load_pd( &C1[0+bs*1] );
		
			if(alg==1) // AB = A * B'
				{
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				e_0 = _mm256_unpacklo_pd( d_0, d_1 );
				e_1 = _mm256_unpackhi_pd( d_0, d_1 );
				
				u_2 = _mm256_extractf128_pd( e_0, 0x1 );
				u_0 = _mm256_castpd256_pd128( e_0 );
				u_3 = _mm256_extractf128_pd( e_1, 0x1 );
				u_1 = _mm256_castpd256_pd128( e_1 );

				e_0 = _mm256_unpacklo_pd( d_4, d_5 );
				e_1 = _mm256_unpackhi_pd( d_4, d_5 );
				
				u_6 = _mm256_extractf128_pd( e_0, 0x1 );
				u_4 = _mm256_castpd256_pd128( e_0 );
				u_7 = _mm256_extractf128_pd( e_1, 0x1 );
				u_5 = _mm256_castpd256_pd128( e_1 );

				goto store_t;
				}
			}
		else // t(C)
			{
			e_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			e_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );
			e_4 = _mm256_unpacklo_pd( c_40_51_60_71, c_41_50_61_70 );
			e_5 = _mm256_unpackhi_pd( c_41_50_61_70, c_40_51_60_71 );

			v_2 = _mm256_extractf128_pd( e_0, 0x1 );
			v_0 = _mm256_castpd256_pd128( e_0 );
			v_3 = _mm256_extractf128_pd( e_1, 0x1 );
			v_1 = _mm256_castpd256_pd128( e_1 );
			v_6 = _mm256_extractf128_pd( e_4, 0x1 );
			v_4 = _mm256_castpd256_pd128( e_4 );
			v_7 = _mm256_extractf128_pd( e_5, 0x1 );
			v_5 = _mm256_castpd256_pd128( e_5 );

			u_0 = _mm_load_pd( &C0[0+bs*0] );
			u_1 = _mm_load_pd( &C0[0+bs*1] );
			u_2 = _mm_load_pd( &C0[0+bs*2] );
			u_3 = _mm_load_pd( &C0[0+bs*3] );
			u_4 = _mm_load_pd( &C0[0+bs*4] );
			u_5 = _mm_load_pd( &C0[0+bs*5] );
			u_6 = _mm_load_pd( &C0[0+bs*6] );
			u_7 = _mm_load_pd( &C0[0+bs*7] );

			if(alg==1) // AB = A * B'
				{
				u_0 = _mm_add_pd( u_0, v_0 );
				u_1 = _mm_add_pd( u_1, v_1 );
				u_2 = _mm_add_pd( u_2, v_2 );
				u_3 = _mm_add_pd( u_3, v_3 );
				u_4 = _mm_add_pd( u_4, v_4 );
				u_5 = _mm_add_pd( u_5, v_5 );
				u_6 = _mm_add_pd( u_6, v_6 );
				u_7 = _mm_add_pd( u_7, v_7 );
				}
			else // AB = - A * B'
				{
				u_0 = _mm_sub_pd( u_0, v_0 );
				u_1 = _mm_sub_pd( u_1, v_1 );
				u_2 = _mm_sub_pd( u_2, v_2 );
				u_3 = _mm_sub_pd( u_3, v_3 );
				u_4 = _mm_sub_pd( u_4, v_4 );
				u_5 = _mm_sub_pd( u_5, v_5 );
				u_6 = _mm_sub_pd( u_6, v_6 );
				u_7 = _mm_sub_pd( u_7, v_7 );
				}

			if(td==0) // t( t(AB) + C )
				{
				e_0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_0 ), u_2, 0x1 );
				e_1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_1 ), u_3, 0x1 );
				e_4 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_4 ), u_6, 0x1 );
				e_5 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_5 ), u_7, 0x1 );

				d_0 = _mm256_unpacklo_pd( e_0, e_1 );
				d_1 = _mm256_unpackhi_pd( e_0, e_1 );
				d_4 = _mm256_unpacklo_pd( e_4, e_5 );
				d_5 = _mm256_unpackhi_pd( e_4, e_5 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (5 - 8) x (1 - 2)
	store_n:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_maskstore_pd( &D1[0+bs*0], mask_m, d_4 );

	if(kn>=2)
		{
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_maskstore_pd( &D1[0+bs*1], mask_m, d_5 );
		}
	return;

	store_t:
	if(kn>=2)
		{
		_mm_store_pd( &D0[0+bs*0], u_0 );
		_mm_store_pd( &D0[0+bs*1], u_1 );
		_mm_store_pd( &D0[0+bs*2], u_2 );
		_mm_store_pd( &D0[0+bs*3], u_3 );

		if(km>=8)
			{
			_mm_store_pd( &D0[0+bs*4], u_4 );
			_mm_store_pd( &D0[0+bs*5], u_5 );
			_mm_store_pd( &D0[0+bs*6], u_6 );
			_mm_store_pd( &D0[0+bs*7], u_7 );
			}
		else
			{
			_mm_store_pd( &D0[0+bs*4], u_4 );
			if(km>=6)
				{
				_mm_store_pd( &D0[0+bs*5], u_5 );
				if(km>6)
					{
					_mm_store_pd( &D0[0+bs*6], u_6 );
					}
				}
			}
		}
	else
		{
		_mm_store_sd( &D0[0+bs*0], u_0 );
		_mm_store_sd( &D0[0+bs*1], u_1 );
		_mm_store_sd( &D0[0+bs*2], u_2 );
		_mm_store_sd( &D0[0+bs*3], u_3 );

		if(km>=8)
			{
			_mm_store_sd( &D0[0+bs*4], u_4 );
			_mm_store_sd( &D0[0+bs*5], u_5 );
			_mm_store_sd( &D0[0+bs*6], u_6 );
			_mm_store_sd( &D0[0+bs*7], u_7 );
			}
		else
			{
			_mm_store_sd( &D0[0+bs*4], u_4 );
			if(km>=6)
				{
				_mm_store_sd( &D0[0+bs*5], u_5 );
				if(km>6)
					{
					_mm_store_sd( &D0[0+bs*6], u_6 );
					}
				}
			}
		}
	return;

	}



void kernel_dgemm_nt_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0, A_0,
		b_0, B_0, b_1, b_2, B_2, b_3,
		c_00, c_01, c_03, c_02,
		C_00, C_01, C_03, C_02;
	
	__m256d
		c_0, c_1, c_2, c_3,
		d_0, d_1, d_2, d_3,
		e_0, e_1, e_2, e_3;

	__m256i 
		mask_m, mask_n;

	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_03 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();
	C_00 = _mm256_setzero_pd();
	C_01 = _mm256_setzero_pd();
	C_03 = _mm256_setzero_pd();
	C_02 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0 = _mm256_load_pd( &A[0] );
	b_0 = _mm256_broadcast_pd( (__m128d *) &B[0] );
	b_2 = _mm256_broadcast_pd( (__m128d *) &B[2] );

	for(k=0; k<kmax-3; k+=4)
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
	
	if(kmax%4>=2)
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

	if(kmax%2==1)
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

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			d_0 = _mm256_blend_pd( c_00, c_01, 0xa );
			d_1 = _mm256_blend_pd( c_00, c_01, 0x5 );
			d_2 = _mm256_blend_pd( c_02, c_03, 0xa );
			d_3 = _mm256_blend_pd( c_02, c_03, 0x5 );

			goto store_n;
			}
		else // transposed
			{
			e_0 = _mm256_unpacklo_pd( c_00, c_01 );
			e_1 = _mm256_unpackhi_pd( c_01, c_00 );
			e_2 = _mm256_unpacklo_pd( c_02, c_03 );
			e_3 = _mm256_unpackhi_pd( c_03, c_02 );

			d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			d_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
			d_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			c_0 = _mm256_blend_pd( c_00, c_01, 0xa );
			c_1 = _mm256_blend_pd( c_00, c_01, 0x5 );
			c_2 = _mm256_blend_pd( c_02, c_03, 0xa );
			c_3 = _mm256_blend_pd( c_02, c_03, 0x5 );

			d_0 = _mm256_load_pd( &C[0+bs*0] );
			d_1 = _mm256_load_pd( &C[0+bs*1] );
			d_2 = _mm256_load_pd( &C[0+bs*2] );
			d_3 = _mm256_load_pd( &C[0+bs*3] );
			
			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );
				}

			if(td==0) // AB + C 
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				e_0 = _mm256_unpacklo_pd( d_0, d_1 );
				e_1 = _mm256_unpackhi_pd( d_0, d_1 );
				e_2 = _mm256_unpacklo_pd( d_2, d_3 );
				e_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				goto store_t;
				}

			}
		else // t(C)
			{

			e_0 = _mm256_unpacklo_pd( c_00, c_01 );
			e_1 = _mm256_unpackhi_pd( c_01, c_00 );
			e_2 = _mm256_unpacklo_pd( c_02, c_03 );
			e_3 = _mm256_unpackhi_pd( c_03, c_02 );

			c_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
			c_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
			c_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
			c_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

			d_0 = _mm256_load_pd( &C[0+bs*0] );
			d_1 = _mm256_load_pd( &C[0+bs*1] );
			d_2 = _mm256_load_pd( &C[0+bs*2] );
			d_3 = _mm256_load_pd( &C[0+bs*3] );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				e_0 = _mm256_unpacklo_pd( d_0, d_1 );
				e_1 = _mm256_unpackhi_pd( d_0, d_1 );
				e_2 = _mm256_unpacklo_pd( d_2, d_3 );
				e_3 = _mm256_unpackhi_pd( d_2, d_3 );

				c_0 = _mm256_permute2f128_pd( e_0, e_2, 0x20 );
				c_2 = _mm256_permute2f128_pd( e_0, e_2, 0x31 );
				c_1 = _mm256_permute2f128_pd( e_1, e_3, 0x20 );
				c_3 = _mm256_permute2f128_pd( e_1, e_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (1 - 4) x (3 - 4)
	store_n:
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_maskstore_pd( &D[0+bs*0], mask_m, d_0 );
	_mm256_maskstore_pd( &D[0+bs*1], mask_m, d_1 );
	_mm256_maskstore_pd( &D[0+bs*2], mask_m, d_2 );

	if(kn>=4)
		{
		_mm256_maskstore_pd( &D[0+bs*3], mask_m, d_3 );
		}
	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );

	if(km>=4)
		{
		_mm256_maskstore_pd( &D[0+bs*0], mask_n, d_0 );
		_mm256_maskstore_pd( &D[0+bs*1], mask_n, d_1 );
		_mm256_maskstore_pd( &D[0+bs*2], mask_n, d_2 );
		_mm256_maskstore_pd( &D[0+bs*3], mask_n, d_3 );
		}
	else
		{
		_mm256_maskstore_pd( &D[0+bs*0], mask_n, d_0 );
		if(km>=2)
			{
			_mm256_maskstore_pd( &D[0+bs*1], mask_n, d_1 );
			if(km>2)
				{
				_mm256_maskstore_pd( &D[0+bs*2], mask_n, d_2 );
				}
			}
		}
	return;

	}



void kernel_dgemm_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nt_4x4_vs_lib4(4, 4, kmax, A, B, C, D, alg, tc, td);

	}



void kernel_dgemm_nt_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123,
		b_0101, b_1010,
		ab_temp, // temporary results
		c_00_11_20_31, c_01_10_21_30, C_00_11_20_31, C_01_10_21_30,
		d_00_11_20_31, d_01_10_21_30, D_00_11_20_31, D_01_10_21_30;
	
	__m128d
		u_0, u_1, u_2, u_3,
		v_0, v_1, v_2, v_3;

	__m256d
		c_0, c_1,
		d_0, d_1,
		e_0, e_1;

	__m256i 
		mask_m, mask_n;

	// zero registers
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	C_00_11_20_31 = _mm256_setzero_pd();
	C_01_10_21_30 = _mm256_setzero_pd();
	d_00_11_20_31 = _mm256_setzero_pd();
	d_01_10_21_30 = _mm256_setzero_pd();
	D_00_11_20_31 = _mm256_setzero_pd();
	D_01_10_21_30 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		
		
/*	__builtin_prefetch( A+40 );*/
		C_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, C_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		C_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, C_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch


/*	__builtin_prefetch( A+48 );*/
		d_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, d_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		d_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, d_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch


/*	__builtin_prefetch( A+56 );*/
		D_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, D_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		D_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, D_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		
		A += 16;
		B += 16;

		}

	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, d_00_11_20_31 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, d_01_10_21_30 );
	C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, D_00_11_20_31 );
	C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, D_01_10_21_30 );
	
	if(kmax%4>=2)
		{
		
		c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	
		
		C_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, C_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		C_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, C_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
	
		
		A += 8;
		B += 8;

		}
	
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, C_00_11_20_31 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, C_01_10_21_30 );

	if(kmax%2==1)
		{
		
		c_00_11_20_31 = _mm256_fmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		//b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		//a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	
		}

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // AB = A * B'
			{
			d_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
			d_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );

			goto store_n;
			}
		else // AB = t( A * B' )
			{
			e_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			e_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );

			u_2 = _mm256_extractf128_pd( e_0, 0x1 );
			u_0 = _mm256_castpd256_pd128( e_0 );
			u_3 = _mm256_extractf128_pd( e_1, 0x1 );
			u_1 = _mm256_castpd256_pd128( e_1 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{
			c_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
			c_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );

			d_0 = _mm256_load_pd( &C[0+bs*0] );
			d_1 = _mm256_load_pd( &C[0+bs*1] );
		
			if(alg==1) // AB = A * B'
				{
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				e_0 = _mm256_unpacklo_pd( d_00_11_20_31, d_01_10_21_30 );
				e_1 = _mm256_unpackhi_pd( d_00_11_20_31, d_01_10_21_30 );
				
				u_2 = _mm256_extractf128_pd( e_0, 0x1 );
				u_0 = _mm256_castpd256_pd128( e_0 );
				u_3 = _mm256_extractf128_pd( e_1, 0x1 );
				u_1 = _mm256_castpd256_pd128( e_1 );

				goto store_t;
				}
			}
		else // t(C)
			{
			e_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			e_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );
				
			v_2 = _mm256_extractf128_pd( e_0, 0x1 );
			v_0 = _mm256_castpd256_pd128( e_0 );
			v_3 = _mm256_extractf128_pd( e_1, 0x1 );
			v_1 = _mm256_castpd256_pd128( e_1 );

			u_0 = _mm_load_pd( &C[0+bs*0] );
			u_1 = _mm_load_pd( &C[0+bs*1] );
			u_2 = _mm_load_pd( &C[0+bs*2] );
			u_3 = _mm_load_pd( &C[0+bs*3] );
		
			if(alg==1) // AB = A * B'
				{
				u_0 = _mm_add_pd( u_0, v_0 );
				u_1 = _mm_add_pd( u_1, v_1 );
				u_2 = _mm_add_pd( u_2, v_2 );
				u_3 = _mm_add_pd( u_3, v_3 );
				}
			else // AB = - A * B'
				{
				u_0 = _mm_sub_pd( u_0, v_0 );
				u_1 = _mm_sub_pd( u_1, v_1 );
				u_2 = _mm_sub_pd( u_2, v_2 );
				u_3 = _mm_sub_pd( u_3, v_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				e_0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_0 ), u_2, 0x1 );
				e_1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_1 ), u_3, 0x1 );

				d_0 = _mm256_unpacklo_pd( e_0, e_1 );
				d_1 = _mm256_unpackhi_pd( e_0, e_1 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (1 - 4) x (1 - 2)
	store_n:
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_maskstore_pd( &D[0+bs*0], mask_m, d_0 );

	if(kn>=2)
		{
		_mm256_maskstore_pd( &D[0+bs*1], mask_m, d_1 );
		}
	return;

	store_t:
	if(kn>=2)
		{
		if(km>=4)
			{
			_mm_store_pd( &D[0+bs*0], u_0 );
			_mm_store_pd( &D[0+bs*1], u_1 );
			_mm_store_pd( &D[0+bs*2], u_2 );
			_mm_store_pd( &D[0+bs*3], u_3 );
			}
		else
			{
			_mm_store_pd( &D[0+bs*0], u_0 );
			if(km>=2)
				{
				_mm_store_pd( &D[0+bs*1], u_1 );
				if(km>2)
					{
					_mm_store_pd( &D[0+bs*2], u_2 );
					}
				}
			}
		}
	else
		{
		if(km>=4)
			{
			_mm_store_sd( &D[0+bs*0], u_0 );
			_mm_store_sd( &D[0+bs*1], u_1 );
			_mm_store_sd( &D[0+bs*2], u_2 );
			_mm_store_sd( &D[0+bs*3], u_3 );
			}
		else
			{
			_mm_store_sd( &D[0+bs*0], u_0 );
			if(km>=2)
				{
				_mm_store_sd( &D[0+bs*1], u_1 );
				if(km>2)
					{
					_mm_store_sd( &D[0+bs*2], u_2 );
					}
				}
			}
		}
	return;


	}



void kernel_dgemm_nt_4x2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nt_4x2_vs_lib4(4, 2, kmax, A, B, C, D, alg, tc, td);

	}



void kernel_dgemm_nt_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;
	
	__m128d
		a_01,
		b_01, b_10,
		ab_temp, // temporary results
		c_00_11, c_01_10, C_00_11, C_01_10,
		d_00_11, d_01_10, D_00_11, D_01_10;
	
	__m128d
		t_0,
		c_0, c_1,
		d_0, d_1,
		c_00_10, c_01_11,
		c_00_01, c_10_11,
		d_00_10, d_01_11,
		d_00_01, d_10_11;

	// zero registers
	c_00_11 = _mm_setzero_pd();
	c_01_10 = _mm_setzero_pd();
	C_00_11 = _mm_setzero_pd();
	C_01_10 = _mm_setzero_pd();
	d_00_11 = _mm_setzero_pd();
	d_01_10 = _mm_setzero_pd();
	D_00_11 = _mm_setzero_pd();
	D_01_10 = _mm_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_01 = _mm_load_pd( &A[0] );
	b_01 = _mm_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		c_00_11 = _mm_fmadd_pd( a_01, b_01, c_00_11 );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[4] ); // prefetch
		c_01_10 = _mm_fmadd_pd( a_01, b_10, c_01_10 );
		a_01    = _mm_load_pd( &A[4] ); // prefetch
		
		
/*	__builtin_prefetch( A+40 );*/
		C_00_11 = _mm_fmadd_pd( a_01, b_01, C_00_11 );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[8] ); // prefetch
		C_01_10 = _mm_fmadd_pd( a_01, b_10, C_01_10 );
		a_01    = _mm_load_pd( &A[8] ); // prefetch


/*	__builtin_prefetch( A+48 );*/
		d_00_11 = _mm_fmadd_pd( a_01, b_01, d_00_11 );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[12] ); // prefetch
		d_01_10 = _mm_fmadd_pd( a_01, b_10, d_01_10 );
		a_01    = _mm_load_pd( &A[12] ); // prefetch


/*	__builtin_prefetch( A+56 );*/
		D_00_11 = _mm_fmadd_pd( a_01, b_01, D_00_11 );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[16] ); // prefetch
		D_01_10 = _mm_fmadd_pd( a_01, b_10, D_01_10 );
		a_01    = _mm_load_pd( &A[16] ); // prefetch
		
		A += 16;
		B += 16;

		}
	
	c_00_11 = _mm_add_pd( c_00_11, d_00_11 );
	c_01_10 = _mm_add_pd( c_01_10, d_01_10 );
	C_00_11 = _mm_add_pd( C_00_11, D_00_11 );
	C_01_10 = _mm_add_pd( C_01_10, D_01_10 );

	if(kmax%4>=2)
		{
		
		c_00_11 = _mm_fmadd_pd( a_01, b_01, c_00_11 );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[4] ); // prefetch
		c_01_10 = _mm_fmadd_pd( a_01, b_10, c_01_10 );
		a_01    = _mm_load_pd( &A[4] ); // prefetch
	
		
		C_00_11 = _mm_fmadd_pd( a_01, b_01, C_00_11 );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[8] ); // prefetch
		C_01_10 = _mm_fmadd_pd( a_01, b_10, C_01_10 );
		a_01    = _mm_load_pd( &A[8] ); // prefetch
	
		
		A += 8;
		B += 8;

		}
	
	c_00_11 = _mm_add_pd( c_00_11, C_00_11 );
	c_01_10 = _mm_add_pd( c_01_10, C_01_10 );

	if(kmax%2==1)
		{
		
		c_00_11 = _mm_fmadd_pd( a_01, b_01, c_00_11 );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		//b_01    = _mm_load_pd( &B[4] ); // prefetch
		c_01_10 = _mm_fmadd_pd( a_01, b_10, c_01_10 );
		//a_01    = _mm_load_pd( &A[4] ); // prefetch
	
		}

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			d_0 = _mm_blend_pd( c_00_11, c_01_10, 0x2 );
			d_1 = _mm_blend_pd( c_00_11, c_01_10, 0x1 );

			goto store_n;
			}
		else
			{
			//c_00_01 = _mm_shuffle_pd( c_00_11, c_01_10, 0x0 );
			//c_10_11 = _mm_shuffle_pd( c_01_10, c_00_11, 0x3 );
			d_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			d_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{
			c_0 = _mm_blend_pd( c_00_11, c_01_10, 0x2 );
			c_1 = _mm_blend_pd( c_00_11, c_01_10, 0x1 );

			d_0 = _mm_load_pd( &C[0+bs*0] );
			d_1 = _mm_load_pd( &C[0+bs*1] );
		
			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 );
				d_1 = _mm_add_pd( d_1, c_1 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 );
				d_1 = _mm_sub_pd( d_1, c_1 );
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				//d_00_01 = _mm_shuffle_pd( d_00_11, d_01_10, 0x0 );
				//d_10_11 = _mm_shuffle_pd( d_00_11, d_01_10, 0x3 );
				t_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( d_0, d_1 );
				d_0 = t_0;

				goto store_t;
				}
			}
		else // t(C)
			{
			//c_00_01 = _mm_shuffle_pd( c_00_11, c_01_10, 0x0 );
			//c_10_11 = _mm_shuffle_pd( c_01_10, c_00_11, 0x3 );
			c_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			c_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );

			d_0 = _mm_load_pd( &C[0+bs*0] );
			d_1 = _mm_load_pd( &C[0+bs*1] );

			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 );
				d_1 = _mm_add_pd( d_1, c_1 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 );
				d_1 = _mm_sub_pd( d_1, c_1 );
				}

			if(td==0) // t( t(AB) + C )
				{
				//d_00_01 = _mm_shuffle_pd( d_00_10, d_01_11, 0x0 );
				//d_10_11 = _mm_shuffle_pd( d_00_10, d_01_11, 0x3 );
				t_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( d_0, d_1 );
				d_0 = t_0;

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}
			}
		}

	store_n:
	if(km>=2)
		{
		_mm_store_pd( &D[0+bs*0], d_0 );
		if(kn>=2)
			_mm_store_pd( &D[0+bs*1], d_1 );
		}
	else
		{
		_mm_store_sd( &D[0+bs*0], d_0 );
		if(kn>=2)
			_mm_store_sd( &D[0+bs*1], d_1 );
		}

	store_t:
	if(kn>=2)
		{
		_mm_store_pd( &D[0+bs*0], d_0 );
		if(km>=2)
			_mm_store_pd( &D[0+bs*1], d_1 );
		}
	else
		{
		_mm_store_sd( &D[0+bs*0], d_0 );
		if(km>=2)
			_mm_store_sd( &D[0+bs*1], d_1 );
		}

	}



#if ! defined(BLASFEO)
void kernel_dgemm_nn_12x4_lib4(int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, int tc, int td)
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;

	__builtin_prefetch( B+0 );
	__builtin_prefetch( B+8 );

	const int bs = 4;

	const int B_next = bs*sdb;

	__builtin_prefetch( B+B_next+0 );
	__builtin_prefetch( B+B_next+8 );

	int k;

	__m256d
		a_0, a_4, a_8,
		b_0,
		tmp,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9, d_a, d_b;
	
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();
	d_8 = _mm256_setzero_pd();
	d_9 = _mm256_setzero_pd();
	d_a = _mm256_setzero_pd();
	d_b = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		__builtin_prefetch( B+2*B_next+0 );

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		a_8 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		a_8 = _mm256_load_pd( &A2[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		__builtin_prefetch( B+2*B_next+8 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		a_8 = _mm256_load_pd( &A2[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		a_8 = _mm256_load_pd( &A2[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		A0 += 4*bs;
		A1 += 4*bs;
		A2 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		a_8 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		A0 += 1*bs;
		A1 += 1*bs;
		A2 += 1*bs;
		B  += 1;

		}
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		c_8, c_9, c_a, c_b,
		t_0, t_1, t_2, t_3;
	
	if(alg==0) // D = A * B , there is no tc
		{
		if(td==0)
			{
			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_8, d_9 );
			t_1 = _mm256_unpackhi_pd( d_8, d_9 );
			t_2 = _mm256_unpacklo_pd( d_a, d_b );
			t_3 = _mm256_unpackhi_pd( d_a, d_b );

			d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_a = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_9 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			if(alg==1) // AB = A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				d_2 = _mm256_add_pd( c_2, d_2 );
				d_3 = _mm256_add_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C1[0+bs*0] );
				c_5 = _mm256_load_pd( &C1[0+bs*1] );
				c_6 = _mm256_load_pd( &C1[0+bs*2] );
				c_7 = _mm256_load_pd( &C1[0+bs*3] );
			
				d_4 = _mm256_add_pd( c_4, d_4 );
				d_5 = _mm256_add_pd( c_5, d_5 );
				d_6 = _mm256_add_pd( c_6, d_6 );
				d_7 = _mm256_add_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C2[0+bs*0] );
				c_9 = _mm256_load_pd( &C2[0+bs*1] );
				c_a = _mm256_load_pd( &C2[0+bs*2] );
				c_b = _mm256_load_pd( &C2[0+bs*3] );
			
				d_8 = _mm256_add_pd( c_8, d_8 );
				d_9 = _mm256_add_pd( c_9, d_9 );
				d_a = _mm256_add_pd( c_a, d_a );
				d_b = _mm256_add_pd( c_b, d_b );
				}
			else // AB = - A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				d_2 = _mm256_sub_pd( c_2, d_2 );
				d_3 = _mm256_sub_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C1[0+bs*0] );
				c_5 = _mm256_load_pd( &C1[0+bs*1] );
				c_6 = _mm256_load_pd( &C1[0+bs*2] );
				c_7 = _mm256_load_pd( &C1[0+bs*3] );
			
				d_4 = _mm256_sub_pd( c_4, d_4 );
				d_5 = _mm256_sub_pd( c_5, d_5 );
				d_6 = _mm256_sub_pd( c_6, d_6 );
				d_7 = _mm256_sub_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C2[0+bs*0] );
				c_9 = _mm256_load_pd( &C2[0+bs*1] );
				c_a = _mm256_load_pd( &C2[0+bs*2] );
				c_b = _mm256_load_pd( &C2[0+bs*3] );
			
				d_8 = _mm256_sub_pd( c_8, d_8 );
				d_9 = _mm256_sub_pd( c_9, d_9 );
				d_a = _mm256_sub_pd( c_a, d_a );
				d_b = _mm256_sub_pd( c_b, d_b );
				}

			if(td==0) // t(AB + C)
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_8, d_9 );
				t_1 = _mm256_unpackhi_pd( d_8, d_9 );
				t_2 = _mm256_unpacklo_pd( d_a, d_b );
				t_3 = _mm256_unpackhi_pd( d_a, d_b );

				d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_9 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_a = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_t;
				}

			}
		else // t(C)
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_8, d_9 );
			t_1 = _mm256_unpackhi_pd( d_8, d_9 );
			t_2 = _mm256_unpacklo_pd( d_a, d_b );
			t_3 = _mm256_unpackhi_pd( d_a, d_b );

			d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_a = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_9 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			if(alg==1) // AB = A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				d_2 = _mm256_add_pd( c_2, d_2 );
				d_3 = _mm256_add_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C0[0+bs*4] );
				c_5 = _mm256_load_pd( &C0[0+bs*5] );
				c_6 = _mm256_load_pd( &C0[0+bs*6] );
				c_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_add_pd( c_4, d_4 );
				d_5 = _mm256_add_pd( c_5, d_5 );
				d_6 = _mm256_add_pd( c_6, d_6 );
				d_7 = _mm256_add_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C0[0+bs*8] );
				c_9 = _mm256_load_pd( &C0[0+bs*9] );
				c_a = _mm256_load_pd( &C0[0+bs*10] );
				c_b = _mm256_load_pd( &C0[0+bs*11] );

				d_8 = _mm256_add_pd( c_8, d_8 );
				d_9 = _mm256_add_pd( c_9, d_9 );
				d_a = _mm256_add_pd( c_a, d_a );
				d_b = _mm256_add_pd( c_b, d_b );
				}
			else // AB = - A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				d_2 = _mm256_sub_pd( c_2, d_2 );
				d_3 = _mm256_sub_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C0[0+bs*4] );
				c_5 = _mm256_load_pd( &C0[0+bs*5] );
				c_6 = _mm256_load_pd( &C0[0+bs*6] );
				c_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_sub_pd( c_4, d_4 );
				d_5 = _mm256_sub_pd( c_5, d_5 );
				d_6 = _mm256_sub_pd( c_6, d_6 );
				d_7 = _mm256_sub_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C0[0+bs*8] );
				c_9 = _mm256_load_pd( &C0[0+bs*9] );
				c_a = _mm256_load_pd( &C0[0+bs*10] );
				c_b = _mm256_load_pd( &C0[0+bs*11] );

				d_8 = _mm256_sub_pd( c_8, d_8 );
				d_9 = _mm256_sub_pd( c_9, d_9 );
				d_a = _mm256_sub_pd( c_a, d_a );
				d_b = _mm256_sub_pd( c_b, d_b );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_8, d_9 );
				t_1 = _mm256_unpackhi_pd( d_8, d_9 );
				t_2 = _mm256_unpacklo_pd( d_a, d_b );
				t_3 = _mm256_unpackhi_pd( d_a, d_b );

				d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_a = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_9 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (5 - 8) x (3 - 4)
	store_n:
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );
	_mm256_store_pd( &D0[0+bs*3], d_3 );

	_mm256_store_pd( &D1[0+bs*0], d_4 );
	_mm256_store_pd( &D1[0+bs*1], d_5 );
	_mm256_store_pd( &D1[0+bs*2], d_6 );
	_mm256_store_pd( &D1[0+bs*3], d_7 );

	_mm256_store_pd( &D2[0+bs*0], d_8 );
	_mm256_store_pd( &D2[0+bs*1], d_9 );
	_mm256_store_pd( &D2[0+bs*2], d_a );
	_mm256_store_pd( &D2[0+bs*3], d_b );

	return;

	store_t:
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );
	_mm256_store_pd( &D0[0+bs*3], d_3 );

	_mm256_store_pd( &D0[0+bs*4], d_4 );
	_mm256_store_pd( &D0[0+bs*5], d_5 );
	_mm256_store_pd( &D0[0+bs*6], d_6 );
	_mm256_store_pd( &D0[0+bs*7], d_7 );

	_mm256_store_pd( &D0[0+bs*8], d_8 );
	_mm256_store_pd( &D0[0+bs*9], d_9 );
	_mm256_store_pd( &D0[0+bs*10], d_a );
	_mm256_store_pd( &D0[0+bs*11], d_b );

	return;

	}



void kernel_dgemm_nn_12x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, int tc, int td)
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A0 + 8*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C0 + 8*sdc;
	double *D1 = D0 + 4*sdd;
	double *D2 = D0 + 8*sdd;

	__builtin_prefetch( B+0 );
	__builtin_prefetch( B+8 );

	const int bs = 4;

	const int B_next = bs*sdb;

	__builtin_prefetch( B+B_next+0 );
	__builtin_prefetch( B+B_next+8 );

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		a_0, a_4, a_8,
		b_0,
		tmp,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9, d_a, d_b;
	
	__m256i 
		mask_m, mask_n;

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();
	d_8 = _mm256_setzero_pd();
	d_9 = _mm256_setzero_pd();
	d_a = _mm256_setzero_pd();
	d_b = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		__builtin_prefetch( B+2*B_next+0 );

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		a_8 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		a_8 = _mm256_load_pd( &A2[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		__builtin_prefetch( B+2*B_next+8 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		a_8 = _mm256_load_pd( &A2[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		a_8 = _mm256_load_pd( &A2[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		A0 += 4*bs;
		A1 += 4*bs;
		A2 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		a_8 = _mm256_load_pd( &A2[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		d_8 = _mm256_fmadd_pd( a_8, b_0, d_8 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		d_9 = _mm256_fmadd_pd( a_8, b_0, d_9 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		d_a = _mm256_fmadd_pd( a_8, b_0, d_a );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );
		d_b = _mm256_fmadd_pd( a_8, b_0, d_b );


		A0 += 1*bs;
		A1 += 1*bs;
		A2 += 1*bs;
		B  += 1;

		}
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		c_8, c_9, c_a, c_b,
		t_0, t_1, t_2, t_3;
	
	if(alg==0) // D = A * B , there is no tc
		{
		if(td==0)
			{
			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_8, d_9 );
			t_1 = _mm256_unpackhi_pd( d_8, d_9 );
			t_2 = _mm256_unpacklo_pd( d_a, d_b );
			t_3 = _mm256_unpackhi_pd( d_a, d_b );

			d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_a = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_9 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			if(alg==1) // AB = A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				d_2 = _mm256_add_pd( c_2, d_2 );
				d_3 = _mm256_add_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C1[0+bs*0] );
				c_5 = _mm256_load_pd( &C1[0+bs*1] );
				c_6 = _mm256_load_pd( &C1[0+bs*2] );
				c_7 = _mm256_load_pd( &C1[0+bs*3] );
			
				d_4 = _mm256_add_pd( c_4, d_4 );
				d_5 = _mm256_add_pd( c_5, d_5 );
				d_6 = _mm256_add_pd( c_6, d_6 );
				d_7 = _mm256_add_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C2[0+bs*0] );
				c_9 = _mm256_load_pd( &C2[0+bs*1] );
				c_a = _mm256_load_pd( &C2[0+bs*2] );
				c_b = _mm256_load_pd( &C2[0+bs*3] );
			
				d_8 = _mm256_add_pd( c_8, d_8 );
				d_9 = _mm256_add_pd( c_9, d_9 );
				d_a = _mm256_add_pd( c_a, d_a );
				d_b = _mm256_add_pd( c_b, d_b );
				}
			else // AB = - A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				d_2 = _mm256_sub_pd( c_2, d_2 );
				d_3 = _mm256_sub_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C1[0+bs*0] );
				c_5 = _mm256_load_pd( &C1[0+bs*1] );
				c_6 = _mm256_load_pd( &C1[0+bs*2] );
				c_7 = _mm256_load_pd( &C1[0+bs*3] );
			
				d_4 = _mm256_sub_pd( c_4, d_4 );
				d_5 = _mm256_sub_pd( c_5, d_5 );
				d_6 = _mm256_sub_pd( c_6, d_6 );
				d_7 = _mm256_sub_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C2[0+bs*0] );
				c_9 = _mm256_load_pd( &C2[0+bs*1] );
				c_a = _mm256_load_pd( &C2[0+bs*2] );
				c_b = _mm256_load_pd( &C2[0+bs*3] );
			
				d_8 = _mm256_sub_pd( c_8, d_8 );
				d_9 = _mm256_sub_pd( c_9, d_9 );
				d_a = _mm256_sub_pd( c_a, d_a );
				d_b = _mm256_sub_pd( c_b, d_b );
				}

			if(td==0) // t(AB + C)
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_8, d_9 );
				t_1 = _mm256_unpackhi_pd( d_8, d_9 );
				t_2 = _mm256_unpacklo_pd( d_a, d_b );
				t_3 = _mm256_unpackhi_pd( d_a, d_b );

				d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_9 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_a = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_t;
				}

			}
		else // t(C)
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_8, d_9 );
			t_1 = _mm256_unpackhi_pd( d_8, d_9 );
			t_2 = _mm256_unpacklo_pd( d_a, d_b );
			t_3 = _mm256_unpackhi_pd( d_a, d_b );

			d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_a = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_9 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			if(alg==1) // AB = A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				d_2 = _mm256_add_pd( c_2, d_2 );
				d_3 = _mm256_add_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C0[0+bs*4] );
				c_5 = _mm256_load_pd( &C0[0+bs*5] );
				c_6 = _mm256_load_pd( &C0[0+bs*6] );
				c_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_add_pd( c_4, d_4 );
				d_5 = _mm256_add_pd( c_5, d_5 );
				d_6 = _mm256_add_pd( c_6, d_6 );
				d_7 = _mm256_add_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C0[0+bs*8] );
				c_9 = _mm256_load_pd( &C0[0+bs*9] );
				c_a = _mm256_load_pd( &C0[0+bs*10] );
				c_b = _mm256_load_pd( &C0[0+bs*11] );

				d_8 = _mm256_add_pd( c_8, d_8 );
				d_9 = _mm256_add_pd( c_9, d_9 );
				d_a = _mm256_add_pd( c_a, d_a );
				d_b = _mm256_add_pd( c_b, d_b );
				}
			else // AB = - A*B'
				{
				c_0 = _mm256_load_pd( &C0[0+bs*0] );
				c_1 = _mm256_load_pd( &C0[0+bs*1] );
				c_2 = _mm256_load_pd( &C0[0+bs*2] );
				c_3 = _mm256_load_pd( &C0[0+bs*3] );

				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				d_2 = _mm256_sub_pd( c_2, d_2 );
				d_3 = _mm256_sub_pd( c_3, d_3 );

				c_4 = _mm256_load_pd( &C0[0+bs*4] );
				c_5 = _mm256_load_pd( &C0[0+bs*5] );
				c_6 = _mm256_load_pd( &C0[0+bs*6] );
				c_7 = _mm256_load_pd( &C0[0+bs*7] );

				d_4 = _mm256_sub_pd( c_4, d_4 );
				d_5 = _mm256_sub_pd( c_5, d_5 );
				d_6 = _mm256_sub_pd( c_6, d_6 );
				d_7 = _mm256_sub_pd( c_7, d_7 );

				c_8 = _mm256_load_pd( &C0[0+bs*8] );
				c_9 = _mm256_load_pd( &C0[0+bs*9] );
				c_a = _mm256_load_pd( &C0[0+bs*10] );
				c_b = _mm256_load_pd( &C0[0+bs*11] );

				d_8 = _mm256_sub_pd( c_8, d_8 );
				d_9 = _mm256_sub_pd( c_9, d_9 );
				d_a = _mm256_sub_pd( c_a, d_a );
				d_b = _mm256_sub_pd( c_b, d_b );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_8, d_9 );
				t_1 = _mm256_unpackhi_pd( d_8, d_9 );
				t_2 = _mm256_unpacklo_pd( d_a, d_b );
				t_3 = _mm256_unpackhi_pd( d_a, d_b );

				d_8 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_a = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_9 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_b = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (5 - 8) x (3 - 4)
	store_n:
	d_temp = km - 8.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );

	_mm256_store_pd( &D1[0+bs*0], d_4 );
	_mm256_store_pd( &D1[0+bs*1], d_5 );
	_mm256_store_pd( &D1[0+bs*2], d_6 );

	_mm256_maskstore_pd( &D2[0+bs*0], mask_m, d_8 );
	_mm256_maskstore_pd( &D2[0+bs*1], mask_m, d_9 );
	_mm256_maskstore_pd( &D2[0+bs*2], mask_m, d_a );

	if(kn>=4)
		{
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_store_pd( &D1[0+bs*3], d_7 );
		_mm256_maskstore_pd( &D2[0+bs*3], mask_m, d_b );
		}

	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );
	
	_mm256_maskstore_pd( &D0[0+bs*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask_n, d_3 );

	_mm256_maskstore_pd( &D0[0+bs*4], mask_n, d_4 );
	_mm256_maskstore_pd( &D0[0+bs*5], mask_n, d_5 );
	_mm256_maskstore_pd( &D0[0+bs*6], mask_n, d_6 );
	_mm256_maskstore_pd( &D0[0+bs*7], mask_n, d_7 );

	if(km>=12)
		{
		_mm256_maskstore_pd( &D0[0+bs*8], mask_n, d_4 );
		_mm256_maskstore_pd( &D0[0+bs*9], mask_n, d_5 );
		_mm256_maskstore_pd( &D0[0+bs*10], mask_n, d_6 );
		_mm256_maskstore_pd( &D0[0+bs*11], mask_n, d_7 );
		}
	else
		{
		_mm256_maskstore_pd( &D0[0+bs*8], mask_n, d_8 );
		if(km>=10)
			{
			_mm256_maskstore_pd( &D0[0+bs*9], mask_n, d_9 );
			if(km>10)
				{
				_mm256_maskstore_pd( &D0[0+bs*10], mask_n, d_a );
				}
			}
		}

	return;

	}

#endif


void kernel_dgemm_nn_8x4_lib4(int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, int tc, int td)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;

	__builtin_prefetch( B+0 );
	__builtin_prefetch( B+8 );

	const int bs = 4;

	const int B_next = bs*sdb;

	__builtin_prefetch( B+B_next+0 );
	__builtin_prefetch( B+B_next+8 );

	int k;

	__m256d
		a_0, a_4,
		b_0,
		tmp,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7;
	
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		__builtin_prefetch( B+2*B_next+0 );

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );

		__builtin_prefetch( B+2*B_next+8 );

		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );



		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		A0 += 4*bs;
		A1 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;
	
	if(alg==0) // D = A * B , there is no tc
		{
		if(td==0)
			{
			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			c_0 = _mm256_load_pd( &C0[0+bs*0] );
			c_1 = _mm256_load_pd( &C0[0+bs*1] );
			c_2 = _mm256_load_pd( &C0[0+bs*2] );
			c_3 = _mm256_load_pd( &C0[0+bs*3] );
			c_4 = _mm256_load_pd( &C1[0+bs*0] );
			c_5 = _mm256_load_pd( &C1[0+bs*1] );
			c_6 = _mm256_load_pd( &C1[0+bs*2] );
			c_7 = _mm256_load_pd( &C1[0+bs*3] );
			
			if(alg==1) // AB = A*B'
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
			else // AB = - A*B'
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

			if(td==0) // t(AB + C)
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_t;
				}

			}
		else // t(C)
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			c_0 = _mm256_load_pd( &C0[0+bs*0] );
			c_1 = _mm256_load_pd( &C0[0+bs*1] );
			c_2 = _mm256_load_pd( &C0[0+bs*2] );
			c_3 = _mm256_load_pd( &C0[0+bs*3] );
			c_4 = _mm256_load_pd( &C0[0+bs*4] );
			c_5 = _mm256_load_pd( &C0[0+bs*5] );
			c_6 = _mm256_load_pd( &C0[0+bs*6] );
			c_7 = _mm256_load_pd( &C0[0+bs*7] );

			if(alg==1) // AB = A*B'
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
			else // AB = - A*B'
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

			if(td==0) // t( t(AB) + C )
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (5 - 8) x (3 - 4)
	store_n:
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );
	_mm256_store_pd( &D0[0+bs*3], d_3 );

	_mm256_store_pd( &D1[0+bs*0], d_4 );
	_mm256_store_pd( &D1[0+bs*1], d_5 );
	_mm256_store_pd( &D1[0+bs*2], d_6 );
	_mm256_store_pd( &D1[0+bs*3], d_7 );

	return;

	store_t:
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );
	_mm256_store_pd( &D0[0+bs*3], d_3 );

	_mm256_store_pd( &D0[0+bs*4], d_4 );
	_mm256_store_pd( &D0[0+bs*5], d_5 );
	_mm256_store_pd( &D0[0+bs*6], d_6 );
	_mm256_store_pd( &D0[0+bs*7], d_7 );

	return;

	}



void kernel_dgemm_nn_8x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, int tc, int td)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;

	__builtin_prefetch( B+0 );
	__builtin_prefetch( B+8 );

	const int bs = 4;

	const int B_next = bs*sdb;

	__builtin_prefetch( B+B_next+0 );
	__builtin_prefetch( B+B_next+8 );

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		a_0, a_4,
		b_0,
		tmp,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7;
	
	__m256i 
		mask_m, mask_n;

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		__builtin_prefetch( B+2*B_next+0 );

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		__builtin_prefetch( B+2*B_next+8 );

		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		A0 += 4*bs;
		A1 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm256_fmadd_pd( a_4, b_0, d_6 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm256_fmadd_pd( a_4, b_0, d_7 );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;
	
	if(alg==0) // D = A * B , there is no tc
		{
		if(td==0)
			{
			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			c_0 = _mm256_load_pd( &C0[0+bs*0] );
			c_1 = _mm256_load_pd( &C0[0+bs*1] );
			c_2 = _mm256_load_pd( &C0[0+bs*2] );
			c_3 = _mm256_load_pd( &C0[0+bs*3] );
			c_4 = _mm256_load_pd( &C1[0+bs*0] );
			c_5 = _mm256_load_pd( &C1[0+bs*1] );
			c_6 = _mm256_load_pd( &C1[0+bs*2] );
			c_7 = _mm256_load_pd( &C1[0+bs*3] );
			
			if(alg==1) // AB = A*B'
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
			else // AB = - A*B'
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

			if(td==0) // t(AB + C)
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_t;
				}

			}
		else // t(C)
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			t_2 = _mm256_unpacklo_pd( d_6, d_7 );
			t_3 = _mm256_unpackhi_pd( d_6, d_7 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			c_0 = _mm256_load_pd( &C0[0+bs*0] );
			c_1 = _mm256_load_pd( &C0[0+bs*1] );
			c_2 = _mm256_load_pd( &C0[0+bs*2] );
			c_3 = _mm256_load_pd( &C0[0+bs*3] );
			c_4 = _mm256_load_pd( &C0[0+bs*4] );
			c_5 = _mm256_load_pd( &C0[0+bs*5] );
			c_6 = _mm256_load_pd( &C0[0+bs*6] );
			c_7 = _mm256_load_pd( &C0[0+bs*7] );

			if(alg==1) // AB = A*B'
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
			else // AB = - A*B'
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

			if(td==0) // t( t(AB) + C )
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				t_2 = _mm256_unpacklo_pd( d_6, d_7 );
				t_3 = _mm256_unpackhi_pd( d_6, d_7 );

				d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_6 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_7 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (5 - 8) x (3 - 4)
	store_n:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );

	_mm256_maskstore_pd( &D1[0+bs*0], mask_m, d_4 );
	_mm256_maskstore_pd( &D1[0+bs*1], mask_m, d_5 );
	_mm256_maskstore_pd( &D1[0+bs*2], mask_m, d_6 );

	if(kn>=4)
		{
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_maskstore_pd( &D1[0+bs*3], mask_m, d_7 );
		}

	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );
	
	_mm256_maskstore_pd( &D0[0+bs*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+bs*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+bs*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+bs*3], mask_n, d_3 );

	if(km>=8)
		{
		_mm256_maskstore_pd( &D0[0+bs*4], mask_n, d_4 );
		_mm256_maskstore_pd( &D0[0+bs*5], mask_n, d_5 );
		_mm256_maskstore_pd( &D0[0+bs*6], mask_n, d_6 );
		_mm256_maskstore_pd( &D0[0+bs*7], mask_n, d_7 );
		}
	else
		{
		_mm256_maskstore_pd( &D0[0+bs*4], mask_n, d_4 );
		if(km>=6)
			{
			_mm256_maskstore_pd( &D0[0+bs*5], mask_n, d_5 );
			if(km>6)
				{
				_mm256_maskstore_pd( &D0[0+bs*6], mask_n, d_6 );
				}
			}
		}

	return;

	}
#endif



void kernel_dgemm_nn_8x2_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, int tc, int td)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		a_0, a_4,
		b_0,
		tmp,
		d_0, d_1,
		d_4, d_5; // TODO add more accumulation registers !!!!!!!!!
	
	__m256i 
		mask_m, mask_n;

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );


		A0 += 4*bs;
		A1 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm256_fmadd_pd( a_4, b_0, d_4 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm256_fmadd_pd( a_4, b_0, d_5 );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
	__m128d
		u_0, u_1, u_2, u_3,
		u_4, u_5, u_6, u_7,
		v_0, v_1, v_2, v_3,
		v_4, v_5, v_6, v_7;

	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;
	
	if(alg==0) // D = A * B , there is no tc
		{
		if(td==0)
			{
			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			
			u_2 = _mm256_extractf128_pd( t_0, 0x1 );
			u_0 = _mm256_castpd256_pd128( t_0 );
			u_3 = _mm256_extractf128_pd( t_1, 0x1 );
			u_1 = _mm256_castpd256_pd128( t_1 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			
			u_6 = _mm256_extractf128_pd( t_0, 0x1 );
			u_4 = _mm256_castpd256_pd128( t_0 );
			u_7 = _mm256_extractf128_pd( t_1, 0x1 );
			u_5 = _mm256_castpd256_pd128( t_1 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			c_0 = _mm256_load_pd( &C0[0+bs*0] );
			c_1 = _mm256_load_pd( &C0[0+bs*1] );
			c_4 = _mm256_load_pd( &C1[0+bs*0] );
			c_5 = _mm256_load_pd( &C1[0+bs*1] );
			
			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				d_4 = _mm256_add_pd( c_4, d_4 );
				d_5 = _mm256_add_pd( c_5, d_5 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				d_4 = _mm256_sub_pd( c_4, d_4 );
				d_5 = _mm256_sub_pd( c_5, d_5 );
				}

			if(td==0) // t(AB + C)
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				
				u_2 = _mm256_extractf128_pd( t_0, 0x1 );
				u_0 = _mm256_castpd256_pd128( t_0 );
				u_3 = _mm256_extractf128_pd( t_1, 0x1 );
				u_1 = _mm256_castpd256_pd128( t_1 );

				t_0 = _mm256_unpacklo_pd( d_4, d_5 );
				t_1 = _mm256_unpackhi_pd( d_4, d_5 );
				
				u_6 = _mm256_extractf128_pd( t_0, 0x1 );
				u_4 = _mm256_castpd256_pd128( t_0 );
				u_7 = _mm256_extractf128_pd( t_1, 0x1 );
				u_5 = _mm256_castpd256_pd128( t_1 );

				goto store_t;
				}

			}
		else // t(C)
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			
			u_2 = _mm256_extractf128_pd( t_0, 0x1 );
			u_0 = _mm256_castpd256_pd128( t_0 );
			u_3 = _mm256_extractf128_pd( t_1, 0x1 );
			u_1 = _mm256_castpd256_pd128( t_1 );

			t_0 = _mm256_unpacklo_pd( d_4, d_5 );
			t_1 = _mm256_unpackhi_pd( d_4, d_5 );
			
			u_6 = _mm256_extractf128_pd( t_0, 0x1 );
			u_4 = _mm256_castpd256_pd128( t_0 );
			u_7 = _mm256_extractf128_pd( t_1, 0x1 );
			u_5 = _mm256_castpd256_pd128( t_1 );

			v_0 = _mm_load_pd( &C0[0+bs*0] );
			v_1 = _mm_load_pd( &C0[0+bs*1] );
			v_2 = _mm_load_pd( &C0[0+bs*2] );
			v_3 = _mm_load_pd( &C0[0+bs*3] );
			v_4 = _mm_load_pd( &C0[0+bs*4] );
			v_5 = _mm_load_pd( &C0[0+bs*5] );
			v_6 = _mm_load_pd( &C0[0+bs*6] );
			v_7 = _mm_load_pd( &C0[0+bs*7] );

			if(alg==1) // AB = A*B'
				{
				u_0 = _mm_add_pd( v_0, u_0 );
				u_1 = _mm_add_pd( v_1, u_1 );
				u_2 = _mm_add_pd( v_2, u_2 );
				u_3 = _mm_add_pd( v_3, u_3 );
				u_4 = _mm_add_pd( v_4, u_4 );
				u_5 = _mm_add_pd( v_5, u_5 );
				u_6 = _mm_add_pd( v_6, u_6 );
				u_7 = _mm_add_pd( v_7, u_7 );
				}
			else // AB = - A*B'
				{
				u_0 = _mm_sub_pd( v_0, u_0 );
				u_1 = _mm_sub_pd( v_1, u_1 );
				u_2 = _mm_sub_pd( v_2, u_2 );
				u_3 = _mm_sub_pd( v_3, u_3 );
				u_4 = _mm_sub_pd( v_4, u_4 );
				u_5 = _mm_sub_pd( v_5, u_5 );
				u_6 = _mm_sub_pd( v_6, u_6 );
				u_7 = _mm_sub_pd( v_7, u_7 );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_0 ), u_2, 0x1 );
				t_1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_1 ), u_3, 0x1 );
				t_2 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_4 ), u_6, 0x1 );
				t_3 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_5 ), u_7, 0x1 );

				d_0 = _mm256_unpacklo_pd( t_0, t_1 );
				d_1 = _mm256_unpackhi_pd( t_0, t_1 );
				d_4 = _mm256_unpacklo_pd( t_2, t_3 );
				d_5 = _mm256_unpackhi_pd( t_2, t_3 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (5 - 8) x (1 - 2)
	store_n:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_maskstore_pd( &D1[0+bs*0], mask_m, d_4 );

	if(kn>=2)
		{
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_maskstore_pd( &D1[0+bs*1], mask_m, d_5 );
		}
	return;

	store_t:
	if(kn>=2)
		{
		_mm_store_pd( &D0[0+bs*0], u_0 );
		_mm_store_pd( &D0[0+bs*1], u_1 );
		_mm_store_pd( &D0[0+bs*2], u_2 );
		_mm_store_pd( &D0[0+bs*3], u_3 );

		if(km>=8)
			{
			_mm_store_pd( &D0[0+bs*4], u_4 );
			_mm_store_pd( &D0[0+bs*5], u_5 );
			_mm_store_pd( &D0[0+bs*6], u_6 );
			_mm_store_pd( &D0[0+bs*7], u_7 );
			}
		else
			{
			_mm_store_pd( &D0[0+bs*4], u_4 );
			if(km>=6)
				{
				_mm_store_pd( &D0[0+bs*5], u_5 );
				if(km>6)
					{
					_mm_store_pd( &D0[0+bs*6], u_6 );
					}
				}
			}
		}
	else
		{
		_mm_store_sd( &D0[0+bs*0], u_0 );
		_mm_store_sd( &D0[0+bs*1], u_1 );
		_mm_store_sd( &D0[0+bs*2], u_2 );
		_mm_store_sd( &D0[0+bs*3], u_3 );

		if(km>=8)
			{
			_mm_store_sd( &D0[0+bs*4], u_4 );
			_mm_store_sd( &D0[0+bs*5], u_5 );
			_mm_store_sd( &D0[0+bs*6], u_6 );
			_mm_store_sd( &D0[0+bs*7], u_7 );
			}
		else
			{
			_mm_store_sd( &D0[0+bs*4], u_4 );
			if(km>=6)
				{
				_mm_store_sd( &D0[0+bs*5], u_5 );
				if(km>6)
					{
					_mm_store_sd( &D0[0+bs*6], u_6 );
					}
				}
			}
		}
	return;

	}



#if ! defined(BLASFEO)
void kernel_dgemm_nn_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		a_0,
		b_0,
		tmp,
		d_0, d_1, d_2, d_3; // TODO add more accumulation registers !!!!!!!!!!!!!!!!!!
	
	__m256i 
		mask_m, mask_n;

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );


		a_0 = _mm256_load_pd( &A[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );


		a_0 = _mm256_load_pd( &A[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );


		a_0 = _mm256_load_pd( &A[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );


		A += 4*bs;
		B += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		d_2 = _mm256_fmadd_pd( a_0, b_0, d_2 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		d_3 = _mm256_fmadd_pd( a_0, b_0, d_3 );


		A += 1*bs;
		B += 1;

		}
	
	__m256d
		c_0, c_1, c_2, c_3,
		t_0, t_1, t_2, t_3;
	
	if(alg==0) // D = A * B , there is no tc
		{
		if(td==0)
			{
			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			c_0 = _mm256_load_pd( &C[0+bs*0] );
			c_1 = _mm256_load_pd( &C[0+bs*1] );
			c_2 = _mm256_load_pd( &C[0+bs*2] );
			c_3 = _mm256_load_pd( &C[0+bs*3] );
			
			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				d_2 = _mm256_add_pd( c_2, d_2 );
				d_3 = _mm256_add_pd( c_3, d_3 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				d_2 = _mm256_sub_pd( c_2, d_2 );
				d_3 = _mm256_sub_pd( c_3, d_3 );
				}

			if(td==0) // t(AB + C)
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_t;
				}

			}
		else // t(C)
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			t_2 = _mm256_unpacklo_pd( d_2, d_3 );
			t_3 = _mm256_unpackhi_pd( d_2, d_3 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

			c_0 = _mm256_load_pd( &C[0+bs*0] );
			c_1 = _mm256_load_pd( &C[0+bs*1] );
			c_2 = _mm256_load_pd( &C[0+bs*2] );
			c_3 = _mm256_load_pd( &C[0+bs*3] );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				d_2 = _mm256_add_pd( c_2, d_2 );
				d_3 = _mm256_add_pd( c_3, d_3 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				d_2 = _mm256_sub_pd( c_2, d_2 );
				d_3 = _mm256_sub_pd( c_3, d_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (1 - 4) x (3 - 4)
	store_n:
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_maskstore_pd( &D[0+bs*0], mask_m, d_0 );
	_mm256_maskstore_pd( &D[0+bs*1], mask_m, d_1 );
	_mm256_maskstore_pd( &D[0+bs*2], mask_m, d_2 );

	if(kn>=4)
		{
		_mm256_maskstore_pd( &D[0+bs*3], mask_m, d_3 );
		}
	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );

	if(km>=4)
		{
		_mm256_maskstore_pd( &D[0+bs*0], mask_n, d_0 );
		_mm256_maskstore_pd( &D[0+bs*1], mask_n, d_1 );
		_mm256_maskstore_pd( &D[0+bs*2], mask_n, d_2 );
		_mm256_maskstore_pd( &D[0+bs*3], mask_n, d_3 );
		}
	else
		{
		_mm256_maskstore_pd( &D[0+bs*0], mask_n, d_0 );
		if(km>=2)
			{
			_mm256_maskstore_pd( &D[0+bs*1], mask_n, d_1 );
			if(km>2)
				{
				_mm256_maskstore_pd( &D[0+bs*2], mask_n, d_2 );
				}
			}
		}
	return;

	}



void kernel_dgemm_nn_4x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{

	kernel_dgemm_nn_4x4_vs_lib4(4, 4, kmax, A, B, sdb, alg, C, D, tc, td);
	
	}
#endif



void kernel_dgemm_nn_4x2_vs_lib4(int km, int kn, int kmax, double *A0, double *B, int sdb, int alg, double *C0, double *D0, int tc, int td)
	{

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		a_0,
		b_0,
		tmp,
		d_0, d_1; // TODO add more accumulation registers !!!!!!!!!!!!!!!!
	
	__m256i 
		mask_m, mask_n;

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );


		A0 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		d_0 = _mm256_fmadd_pd( a_0, b_0, d_0 );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		d_1 = _mm256_fmadd_pd( a_0, b_0, d_1 );


		A0 += 1*bs;
		B  += 1;

		}
	
	__m128d
		u_0, u_1, u_2, u_3,
		v_0, v_1, v_2, v_3;

	__m256d
		c_0, c_1, c_2, c_3,
		t_0, t_1, t_2, t_3;
	
	if(alg==0) // D = A * B , there is no tc
		{
		if(td==0)
			{
			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			
			u_2 = _mm256_extractf128_pd( t_0, 0x1 );
			u_0 = _mm256_castpd256_pd128( t_0 );
			u_3 = _mm256_extractf128_pd( t_1, 0x1 );
			u_1 = _mm256_castpd256_pd128( t_1 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			c_0 = _mm256_load_pd( &C0[0+bs*0] );
			c_1 = _mm256_load_pd( &C0[0+bs*1] );
			
			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( c_0, d_0 );
				d_1 = _mm256_add_pd( c_1, d_1 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( c_0, d_0 );
				d_1 = _mm256_sub_pd( c_1, d_1 );
				}

			if(td==0) // t(AB + C)
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				
				u_2 = _mm256_extractf128_pd( t_0, 0x1 );
				u_0 = _mm256_castpd256_pd128( t_0 );
				u_3 = _mm256_extractf128_pd( t_1, 0x1 );
				u_1 = _mm256_castpd256_pd128( t_1 );

				goto store_t;
				}

			}
		else // t(C)
			{
			t_0 = _mm256_unpacklo_pd( d_0, d_1 );
			t_1 = _mm256_unpackhi_pd( d_0, d_1 );
			
			u_2 = _mm256_extractf128_pd( t_0, 0x1 );
			u_0 = _mm256_castpd256_pd128( t_0 );
			u_3 = _mm256_extractf128_pd( t_1, 0x1 );
			u_1 = _mm256_castpd256_pd128( t_1 );

			v_0 = _mm_load_pd( &C0[0+bs*0] );
			v_1 = _mm_load_pd( &C0[0+bs*1] );
			v_2 = _mm_load_pd( &C0[0+bs*2] );
			v_3 = _mm_load_pd( &C0[0+bs*3] );

			if(alg==1) // AB = A*B'
				{
				u_0 = _mm_add_pd( v_0, u_0 );
				u_1 = _mm_add_pd( v_1, u_1 );
				u_2 = _mm_add_pd( v_2, u_2 );
				u_3 = _mm_add_pd( v_3, u_3 );
				}
			else // AB = - A*B'
				{
				u_0 = _mm_sub_pd( v_0, u_0 );
				u_1 = _mm_sub_pd( v_1, u_1 );
				u_2 = _mm_sub_pd( v_2, u_2 );
				u_3 = _mm_sub_pd( v_3, u_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_0 ), u_2, 0x1 );
				t_1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_1 ), u_3, 0x1 );

				d_0 = _mm256_unpacklo_pd( t_0, t_1 );
				d_1 = _mm256_unpackhi_pd( t_0, t_1 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}

			}
		}

	// store (1 - 4) x (1 - 2)
	store_n:
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_maskstore_pd( &D0[0+bs*0], mask_m, d_0 );

	if(kn>=2)
		{
		_mm256_maskstore_pd( &D0[0+bs*1], mask_m, d_1 );
		}
	return;

	store_t:
	if(kn>=2)
		{
		if(km>=4)
			{
			_mm_store_pd( &D0[0+bs*0], u_0 );
			_mm_store_pd( &D0[0+bs*1], u_1 );
			_mm_store_pd( &D0[0+bs*2], u_2 );
			_mm_store_pd( &D0[0+bs*3], u_3 );
			}
		else
			{
			_mm_store_pd( &D0[0+bs*0], u_0 );
			if(km>=2)
				{
				_mm_store_pd( &D0[0+bs*1], u_1 );
				if(km>2)
					{
					_mm_store_pd( &D0[0+bs*2], u_2 );
					}
				}
			}
		}
	else
		{
		if(km>=4)
			{
			_mm_store_sd( &D0[0+bs*0], u_0 );
			_mm_store_sd( &D0[0+bs*1], u_1 );
			_mm_store_sd( &D0[0+bs*2], u_2 );
			_mm_store_sd( &D0[0+bs*3], u_3 );
			}
		else
			{
			_mm_store_sd( &D0[0+bs*0], u_0 );
			if(km>=2)
				{
				_mm_store_sd( &D0[0+bs*1], u_1 );
				if(km>2)
					{
					_mm_store_sd( &D0[0+bs*2], u_2 );
					}
				}
			}
		}
	return;


	}



void kernel_dgemm_nn_4x2_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{

	kernel_dgemm_nn_4x2_vs_lib4(4, 2, kmax, A, B, sdb, alg, C, D, tc, td);
	
	}



void kernel_dgemm_nn_2x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		A += 4;
		B += 1;

		}
		
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			
			d_02 = C[0+bs*2];
			d_12 = C[1+bs*2];
			
			d_03 = C[0+bs*3];
			d_13 = C[1+bs*3];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			d_02 = C[2+bs*0];
			d_03 = C[3+bs*0];

			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_12 = C[2+bs*1];
			d_13 = C[3+bs*1];
			}
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;

			c_02 = d_02 + c_02;
			c_12 = d_12 + c_12;

			c_03 = d_03 + c_03;
			c_13 = d_13 + c_13;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;

			c_02 = d_02 - c_02;
			c_12 = d_12 - c_12;

			c_03 = d_03 - c_03;
			c_13 = d_13 - c_13;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			}
		}
	else // km==1
		{
		D[0+bs*0] = c_00;

		D[0+bs*1] = c_01;

		D[0+bs*2] = c_02;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			}
		}
	return;

	store_t:
	if(kn>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		D[3+bs*0] = c_03;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			D[3+bs*1] = c_13;
			}
		}
	else // kn==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			}
		}
	return;

	}



void kernel_dgemm_nn_2x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{
	
	kernel_dgemm_nn_2x4_vs_lib4(2, 4, kmax, A, B, sdb, alg, C, D, tc, td);

	}



void kernel_dgemm_nn_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		A += 4;
		B += 1;

		}
		
	double
		d_00, d_01,
		d_10, d_11;
	
	if(alg==0) // D = A * B'
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			
			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			}
		}
	else // km==1
		{
		D[0+bs*0] = c_00;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			}
		}
	return;

	store_t:
	if(kn>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			}
		}
	else // kn==1
		{
		D[0+bs*0] = c_00;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			}
		}
	return;

	}



void kernel_dgemm_nn_2x2_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{

	kernel_dgemm_nn_2x2_vs_lib4(2, 2, kmax, A, B, sdb, alg, C, D, tc, td);

	}



#if ! defined(BLASFEO)

// B is the diagonal of a matrix
void kernel_dgemm_diag_right_4_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22, b_33,
		c_00,
		d_00, d_01, d_02, d_03;
	
	__m256i
		mask_i;
	
	if(alg==-1)
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		b_11 = _mm256_broadcast_sd( &B[1] );
		b_22 = _mm256_broadcast_sd( &B[2] );
		b_33 = _mm256_broadcast_sd( &B[3] );
		long long long_sign = 0x8000000000000000;
		sign = _mm256_broadcast_sd( (double *) &long_sign );
		b_00 = _mm256_xor_pd( sign, b_00 );
		b_11 = _mm256_xor_pd( sign, b_11 );
		b_22 = _mm256_xor_pd( sign, b_22 );
		b_33 = _mm256_xor_pd( sign, b_33 );
		}
	else
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		b_11 = _mm256_broadcast_sd( &B[1] );
		b_22 = _mm256_broadcast_sd( &B[2] );
		b_33 = _mm256_broadcast_sd( &B[3] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );
			a_00 = _mm256_load_pd( &A[12] );
			d_03 = _mm256_mul_pd( a_00, b_33 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );
			_mm256_store_pd( &D[12], d_03 );

			A += 4*sda;
			D += 4*sdd;

			}
		if(k<kmax)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );
			a_00 = _mm256_load_pd( &A[12] );
			d_03 = _mm256_mul_pd( a_00, b_33 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );
			_mm256_maskstore_pd( &D[4], mask_i, d_01 );
			_mm256_maskstore_pd( &D[8], mask_i, d_02 );
			_mm256_maskstore_pd( &D[12], mask_i, d_03 );
	
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );
			a_00 = _mm256_load_pd( &A[12] );
			d_03 = _mm256_mul_pd( a_00, b_33 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_00 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_00, d_01 );
			c_00 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_00, d_02 );
			c_00 = _mm256_load_pd( &C[12] );
			d_03 = _mm256_add_pd( c_00, d_03 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );
			_mm256_store_pd( &D[12], d_03 );

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;

			}
		for( ; k<kmax; k++)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );
			a_00 = _mm256_load_pd( &A[12] );
			d_03 = _mm256_mul_pd( a_00, b_33 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_00 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_00, d_01 );
			c_00 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_00, d_02 );
			c_00 = _mm256_load_pd( &C[12] );
			d_03 = _mm256_add_pd( c_00, d_03 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );
			_mm256_maskstore_pd( &D[4], mask_i, d_01 );
			_mm256_maskstore_pd( &D[8], mask_i, d_02 );
			_mm256_maskstore_pd( &D[12], mask_i, d_03 );

			}

		}
	
	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_3_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22,
		c_00,
		d_00, d_01, d_02;
	
	__m256i
		mask_i;
	
	if(alg==-1)
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		b_11 = _mm256_broadcast_sd( &B[1] );
		b_22 = _mm256_broadcast_sd( &B[2] );
		long long long_sign = 0x8000000000000000;
		sign = _mm256_broadcast_sd( (double *) &long_sign );
		b_00 = _mm256_xor_pd( sign, b_00 );
		b_11 = _mm256_xor_pd( sign, b_11 );
		b_22 = _mm256_xor_pd( sign, b_22 );
		}
	else
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		b_11 = _mm256_broadcast_sd( &B[1] );
		b_22 = _mm256_broadcast_sd( &B[2] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );

			A += 4*sda;
			D += 4*sdd;

			}
		if(k<kmax)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );
			_mm256_maskstore_pd( &D[4], mask_i, d_01 );
			_mm256_maskstore_pd( &D[8], mask_i, d_02 );
	
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_00 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_00, d_01 );
			c_00 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_00, d_02 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;

			}
		for( ; k<kmax; k++)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );
			a_00 = _mm256_load_pd( &A[8] );
			d_02 = _mm256_mul_pd( a_00, b_22 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_00 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_00, d_01 );
			c_00 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_00, d_02 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );
			_mm256_maskstore_pd( &D[4], mask_i, d_01 );
			_mm256_maskstore_pd( &D[8], mask_i, d_02 );

			}

		}
	
	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_2_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		mask_f,
		sign,
		a_00,
		b_00, b_11,
		c_00,
		d_00, d_01;
	
	__m256i
		mask_i;
	
	if(alg==-1)
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		b_11 = _mm256_broadcast_sd( &B[1] );
		long long long_sign = 0x8000000000000000;
		sign = _mm256_broadcast_sd( (double *) &long_sign );
		b_00 = _mm256_xor_pd( sign, b_00 );
		b_11 = _mm256_xor_pd( sign, b_11 );
		}
	else
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		b_11 = _mm256_broadcast_sd( &B[1] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );

			A += 4*sda;
			D += 4*sdd;

			}
		if(k<kmax)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );
			_mm256_maskstore_pd( &D[4], mask_i, d_01 );
	
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_00 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_00, d_01 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;

			}
		for( ; k<kmax; k++)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			a_00 = _mm256_load_pd( &A[4] );
			d_01 = _mm256_mul_pd( a_00, b_11 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_00 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_00, d_01 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );
			_mm256_maskstore_pd( &D[4], mask_i, d_01 );

			}

		}
	
	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_1_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		mask_f,
		sign,
		a_00,
		b_00,
		c_00,
		d_00;
	
	__m256i
		mask_i;
	
	if(alg==-1)
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		long long long_sign = 0x8000000000000000;
		sign = _mm256_broadcast_sd( (double *) &long_sign );
		b_00 = _mm256_xor_pd( sign, b_00 );
		}
	else
		{
		b_00 = _mm256_broadcast_sd( &B[0] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );

			_mm256_store_pd( &D[0], d_00 );

			A += 4*sda;
			D += 4*sdd;

			}
		if(k<kmax)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );
	
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );

			_mm256_store_pd( &D[0], d_00 );

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;

			}
		for( ; k<kmax; k++)
			{

			const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
			double m_f = kmax-k;

			mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

			a_00 = _mm256_load_pd( &A[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );

			_mm256_maskstore_pd( &D[0], mask_i, d_00 );

			}

		}
	
	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

#if 1

	__m256d
		sign,
		a_00,
		b_00,
		c_00, c_01, c_02, c_03,
		d_00, d_01, d_02, d_03;
		
	if(alg==-1)
		{
		a_00 = _mm256_load_pd( &A[0] );
		long long long_sign = 0x8000000000000000;
		sign = _mm256_broadcast_sd( (double *) &long_sign );
		a_00 = _mm256_xor_pd( sign, a_00 );
		}
	else
		{
		a_00 = _mm256_load_pd( &A[0] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( a_00, b_00 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );
			_mm256_store_pd( &D[12], d_03 );
			
			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_00 = _mm256_load_pd( &B[0] );
			c_00 = _mm256_mul_pd( a_00, b_00 );

			_mm256_store_pd( &D[0], c_00 );
		
			B += 4;
			D += 4;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( a_00, b_00 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_01 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_01, d_01 );
			c_02 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_02, d_02 );
			c_03 = _mm256_load_pd( &C[12] );
			d_03 = _mm256_add_pd( c_03, d_03 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );
			_mm256_store_pd( &D[12], d_03 );
	
			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );

			_mm256_store_pd( &D[0], d_00 );
	
			B += 4;
			C += 4;
			D += 4;
			
			}

		}


#else

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		a_0 = - A[0];
		a_1 = - A[1];
		a_2 = - A[2];
		a_3 = - A[3];
		}
	else
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
		
			B += 4;
			D += 4;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;
			c_3 = C[3+bs*0] + a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_2;
			c_3 = C[3+bs*1] + a_3 * b_3;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;
			c_1 = C[1+bs*2] + a_1 * b_1;
			c_2 = C[2+bs*2] + a_2 * b_2;
			c_3 = C[3+bs*2] + a_3 * b_3;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;
			c_1 = C[1+bs*3] + a_1 * b_1;
			c_2 = C[2+bs*3] + a_2 * b_2;
			c_3 = C[3+bs*3] + a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;
			c_3 = C[3+bs*0] + a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
		
			B += 4;
			C += 4;
			D += 4;
			
			}

		}

#endif
	
	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_3_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256i
		mask;

	__m256d
		sign,
		a_00,
		b_00,
		c_00, c_01, c_02, c_03,
		d_00, d_01, d_02, d_03;
	
	mask = _mm256_set_epi64x( 1, -1, -1, -1 );
		
	if(alg==-1)
		{
		a_00 = _mm256_load_pd( &A[0] );
		long long long_sign = 0x8000000000000000;
		sign = _mm256_broadcast_sd( (double *) &long_sign );
		a_00 = _mm256_xor_pd( sign, a_00 );
		}
	else
		{
		a_00 = _mm256_load_pd( &A[0] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( a_00, b_00 );

			_mm256_maskstore_pd( &D[0], mask, d_00 );
			_mm256_maskstore_pd( &D[4], mask, d_01 );
			_mm256_maskstore_pd( &D[8], mask, d_02 );
			_mm256_maskstore_pd( &D[12], mask, d_03 );
			
			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_00 = _mm256_load_pd( &B[0] );
			c_00 = _mm256_mul_pd( a_00, b_00 );

			_mm256_maskstore_pd( &D[0], mask, c_00 );
		
			B += 4;
			D += 4;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( a_00, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( a_00, b_00 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_01 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_01, d_01 );
			c_02 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_02, d_02 );
			c_03 = _mm256_load_pd( &C[12] );
			d_03 = _mm256_add_pd( c_03, d_03 );

			_mm256_maskstore_pd( &D[0], mask, d_00 );
			_mm256_maskstore_pd( &D[4], mask, d_01 );
			_mm256_maskstore_pd( &D[8], mask, d_02 );
			_mm256_maskstore_pd( &D[12], mask, d_03 );
	
			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( a_00, b_00 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );

			_mm256_maskstore_pd( &D[0], mask, d_00 );
	
			B += 4;
			C += 4;
			D += 4;
			
			}

		}

	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m128d
		sign,
		a_00,
		b_00,
		c_00, c_01, c_02, c_03,
		d_00, d_01, d_02, d_03;
		
	if(alg==-1)
		{
		a_00 = _mm_load_pd( &A[0] );
		long long long_sign = 0x8000000000000000;
		sign = _mm_loaddup_pd( (double *) &long_sign );
		a_00 = _mm_xor_pd( sign, a_00 );
		}
	else
		{
		a_00 = _mm_load_pd( &A[0] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			b_00 = _mm_load_pd( &B[0] );
			d_00 = _mm_mul_pd( a_00, b_00 );
			b_00 = _mm_load_pd( &B[4] );
			d_01 = _mm_mul_pd( a_00, b_00 );
			b_00 = _mm_load_pd( &B[8] );
			d_02 = _mm_mul_pd( a_00, b_00 );
			b_00 = _mm_load_pd( &B[12] );
			d_03 = _mm_mul_pd( a_00, b_00 );

			_mm_store_pd( &D[0], d_00 );
			_mm_store_pd( &D[4], d_01 );
			_mm_store_pd( &D[8], d_02 );
			_mm_store_pd( &D[12], d_03 );
			
			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_00 = _mm_load_pd( &B[0] );
			c_00 = _mm_mul_pd( a_00, b_00 );

			_mm_store_pd( &D[0], c_00 );
		
			B += 4;
			D += 4;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_00 = _mm_load_pd( &B[0] );
			d_00 = _mm_mul_pd( a_00, b_00 );
			b_00 = _mm_load_pd( &B[4] );
			d_01 = _mm_mul_pd( a_00, b_00 );
			b_00 = _mm_load_pd( &B[8] );
			d_02 = _mm_mul_pd( a_00, b_00 );
			b_00 = _mm_load_pd( &B[12] );
			d_03 = _mm_mul_pd( a_00, b_00 );

			c_00 = _mm_load_pd( &C[0] );
			d_00 = _mm_add_pd( c_00, d_00 );
			c_01 = _mm_load_pd( &C[4] );
			d_01 = _mm_add_pd( c_01, d_01 );
			c_02 = _mm_load_pd( &C[8] );
			d_02 = _mm_add_pd( c_02, d_02 );
			c_03 = _mm_load_pd( &C[12] );
			d_03 = _mm_add_pd( c_03, d_03 );

			_mm_store_pd( &D[0], d_00 );
			_mm_store_pd( &D[4], d_01 );
			_mm_store_pd( &D[8], d_02 );
			_mm_store_pd( &D[12], d_03 );
	
			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_00 = _mm_load_pd( &B[0] );
			d_00 = _mm_mul_pd( a_00, b_00 );

			c_00 = _mm_load_pd( &C[0] );
			d_00 = _mm_add_pd( c_00, d_00 );

			_mm_store_pd( &D[0], d_00 );
	
			B += 4;
			C += 4;
			D += 4;
			
			}

		}

	
	}


// A is the diagonal of a matrix
void kernel_dgemm_diag_left_1_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0,
		b_0,
		c_0;
		
	if(alg==-1)
		{
		a_0 = A[0];
		}
	else
		{
		a_0 = A[0];
		}
		
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
			

			b_0 = B[0+bs*1];
			
			c_0 = a_0 * b_0;

			D[0+bs*1] = c_0;
			

			b_0 = B[0+bs*2];
			
			c_0 = a_0 * b_0;

			D[0+bs*2] = c_0;
			

			b_0 = B[0+bs*3];
			
			c_0 = a_0 * b_0;

			D[0+bs*3] = c_0;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		
			B += 4;
			D += 4;
			
			}
		
		}
	else
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			b_0 = B[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;

			D[0+bs*1] = c_0;
			

			b_0 = B[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;

			D[0+bs*2] = c_0;
			

			b_0 = B[0+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;

			D[0+bs*3] = c_0;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
		
			B += 4;
			C += 4;
			D += 4;
			
			}

		}
		
	}

#endif

