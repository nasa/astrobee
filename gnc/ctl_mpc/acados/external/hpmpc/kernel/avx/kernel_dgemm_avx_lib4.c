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

#define LOW_RANK_N 7
#define LOW_RANK_T 3



#if ! defined(BLASFEO)

#if 1
void kernel_dgemm_nt_12x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *A2 = A1 + 4*sda;
	double *C2 = C1 + 4*sdc;
	double *D2 = D1 + 4*sdd;
	
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567,
		b_0123,
		ab_tmp0,
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71,
		c_80_91_a2_b3, c_81_90_a3_b2, c_83_92_a1_b0, c_82_93_a0_b1;
	
	__m256d
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

	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();
	c_40_51_62_73 = _mm256_setzero_pd();
	c_41_50_63_72 = _mm256_setzero_pd();
	c_43_52_61_70 = _mm256_setzero_pd();
	c_42_53_60_71 = _mm256_setzero_pd();
	c_80_91_a2_b3 = _mm256_setzero_pd();
	c_81_90_a3_b2 = _mm256_setzero_pd();
	c_83_92_a1_b0 = _mm256_setzero_pd();
	c_82_93_a0_b1 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch



/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch



/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch



/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch

	

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch



/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch

		
		A0 += 8;
		A1 += 8;
		A2 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
//		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
//		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
//		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch

	
		}
	
	add:
	
	c_00 = c_00_11_22_33;
	c_01 = c_01_10_23_32;
	c_02 = c_02_13_20_31;
	c_03 = c_03_12_21_30;
	c_40 = c_40_51_62_73;
	c_41 = c_41_50_63_72;
	c_42 = c_42_53_60_71;
	c_43 = c_43_52_61_70;
	c_80 = c_80_91_a2_b3;
	c_81 = c_81_90_a3_b2;
	c_82 = c_82_93_a0_b1;
	c_83 = c_83_92_a1_b0;

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
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+ldc*0] );
				d_5 = _mm256_load_pd( &C1[0+ldc*1] );
				d_6 = _mm256_load_pd( &C1[0+ldc*2] );
				d_7 = _mm256_load_pd( &C1[0+ldc*3] );

				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C2[0+ldc*0] );
				d_9 = _mm256_load_pd( &C2[0+ldc*1] );
				d_a = _mm256_load_pd( &C2[0+ldc*2] );
				d_b = _mm256_load_pd( &C2[0+ldc*3] );
		
				d_8 = _mm256_add_pd( d_8, c_8 );
				d_9 = _mm256_add_pd( d_9, c_9 );
				d_a = _mm256_add_pd( d_a, c_a );
				d_b = _mm256_add_pd( d_b, c_b );
				}
			else // AB = - A * B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+ldc*0] );
				d_5 = _mm256_load_pd( &C1[0+ldc*1] );
				d_6 = _mm256_load_pd( &C1[0+ldc*2] );
				d_7 = _mm256_load_pd( &C1[0+ldc*3] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C2[0+ldc*0] );
				d_9 = _mm256_load_pd( &C2[0+ldc*1] );
				d_a = _mm256_load_pd( &C2[0+ldc*2] );
				d_b = _mm256_load_pd( &C2[0+ldc*3] );
		
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
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_0 = _mm256_load_pd( &C0[0+ldc*4] );
				d_1 = _mm256_load_pd( &C0[0+ldc*5] );
				d_2 = _mm256_load_pd( &C0[0+ldc*6] );
				d_3 = _mm256_load_pd( &C0[0+ldc*7] );

				d_0 = _mm256_add_pd( d_4, c_4 );
				d_1 = _mm256_add_pd( d_5, c_5 );
				d_2 = _mm256_add_pd( d_6, c_6 );
				d_3 = _mm256_add_pd( d_7, c_7 );

				d_0 = _mm256_load_pd( &C0[0+ldc*8] );
				d_1 = _mm256_load_pd( &C0[0+ldc*9] );
				d_2 = _mm256_load_pd( &C0[0+ldc*10] );
				d_3 = _mm256_load_pd( &C0[0+ldc*11] );

				d_0 = _mm256_add_pd( d_8, c_8 );
				d_1 = _mm256_add_pd( d_9, c_9 );
				d_2 = _mm256_add_pd( d_a, c_a );
				d_3 = _mm256_add_pd( d_b, c_b );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+ldc*4] );
				d_5 = _mm256_load_pd( &C0[0+ldc*5] );
				d_6 = _mm256_load_pd( &C0[0+ldc*6] );
				d_7 = _mm256_load_pd( &C0[0+ldc*7] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C0[0+ldc*8] );
				d_9 = _mm256_load_pd( &C0[0+ldc*9] );
				d_a = _mm256_load_pd( &C0[0+ldc*10] );
				d_b = _mm256_load_pd( &C0[0+ldc*11] );

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
	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_store_pd( &D0[0+ldc*3], d_3 );

	_mm256_store_pd( &D1[0+ldc*0], d_4 );
	_mm256_store_pd( &D1[0+ldc*1], d_5 );
	_mm256_store_pd( &D1[0+ldc*2], d_6 );
	_mm256_store_pd( &D1[0+ldc*3], d_7 );

	_mm256_store_pd( &D2[0+ldc*0], d_8 );
	_mm256_store_pd( &D2[0+ldc*1], d_9 );
	_mm256_store_pd( &D2[0+ldc*2], d_a );
	_mm256_store_pd( &D2[0+ldc*3], d_b );

	return;

	store_t:
	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_store_pd( &D0[0+ldc*3], d_3 );

	_mm256_store_pd( &D0[0+ldc*4], d_4 );
	_mm256_store_pd( &D0[0+ldc*5], d_5 );
	_mm256_store_pd( &D0[0+ldc*6], d_6 );
	_mm256_store_pd( &D0[0+ldc*7], d_7 );

	_mm256_store_pd( &D0[0+ldc*8], d_8 );
	_mm256_store_pd( &D0[0+ldc*9], d_9 );
	_mm256_store_pd( &D0[0+ldc*10], d_a );
	_mm256_store_pd( &D0[0+ldc*11], d_b );
		
	return;

}



void kernel_dgemm_nt_12x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *A2 = A1 + 4*sda;
	double *C2 = C1 + 4*sdc;
	double *D2 = D1 + 4*sdd;
	
	const int ldc = 4;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_4567,
		b_0123,
		ab_tmp0,
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71,
		c_80_91_a2_b3, c_81_90_a3_b2, c_83_92_a1_b0, c_82_93_a0_b1;
	
	__m256d
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
		mask_n, mask_m;

	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();
	c_40_51_62_73 = _mm256_setzero_pd();
	c_41_50_63_72 = _mm256_setzero_pd();
	c_43_52_61_70 = _mm256_setzero_pd();
	c_42_53_60_71 = _mm256_setzero_pd();
	c_80_91_a2_b3 = _mm256_setzero_pd();
	c_81_90_a3_b2 = _mm256_setzero_pd();
	c_83_92_a1_b0 = _mm256_setzero_pd();
	c_82_93_a0_b1 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch



/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch



/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[8] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch



/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[12] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch

	

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch



/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[4] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch

		
		A0 += 8;
		A1 += 8;
		A2 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_a2_b3 = _mm256_add_pd( c_80_91_a2_b3, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 3210
		c_81_90_a3_b2 = _mm256_add_pd( c_81_90_a3_b2, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
		b_0123        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 2301
		c_83_92_a1_b0 = _mm256_add_pd( c_83_92_a1_b0, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
//		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_load_pd( &A2[0] );
		ab_tmp0       = _mm256_mul_pd( ab_tmp0, b_0123 );
//		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_82_93_a0_b1 = _mm256_add_pd( c_82_93_a0_b1, ab_tmp0 );
//		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch

	
		}
	
	add:
	
	c_00 = c_00_11_22_33;
	c_01 = c_01_10_23_32;
	c_02 = c_02_13_20_31;
	c_03 = c_03_12_21_30;
	c_40 = c_40_51_62_73;
	c_41 = c_41_50_63_72;
	c_42 = c_42_53_60_71;
	c_43 = c_43_52_61_70;
	c_80 = c_80_91_a2_b3;
	c_81 = c_81_90_a3_b2;
	c_82 = c_82_93_a0_b1;
	c_83 = c_83_92_a1_b0;

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
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+ldc*0] );
				d_5 = _mm256_load_pd( &C1[0+ldc*1] );
				d_6 = _mm256_load_pd( &C1[0+ldc*2] );
				d_7 = _mm256_load_pd( &C1[0+ldc*3] );

				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C2[0+ldc*0] );
				d_9 = _mm256_load_pd( &C2[0+ldc*1] );
				d_a = _mm256_load_pd( &C2[0+ldc*2] );
				d_b = _mm256_load_pd( &C2[0+ldc*3] );
		
				d_8 = _mm256_add_pd( d_8, c_8 );
				d_9 = _mm256_add_pd( d_9, c_9 );
				d_a = _mm256_add_pd( d_a, c_a );
				d_b = _mm256_add_pd( d_b, c_b );
				}
			else // AB = - A * B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+ldc*0] );
				d_5 = _mm256_load_pd( &C1[0+ldc*1] );
				d_6 = _mm256_load_pd( &C1[0+ldc*2] );
				d_7 = _mm256_load_pd( &C1[0+ldc*3] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C2[0+ldc*0] );
				d_9 = _mm256_load_pd( &C2[0+ldc*1] );
				d_a = _mm256_load_pd( &C2[0+ldc*2] );
				d_b = _mm256_load_pd( &C2[0+ldc*3] );
		
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
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_0 = _mm256_load_pd( &C0[0+ldc*4] );
				d_1 = _mm256_load_pd( &C0[0+ldc*5] );
				d_2 = _mm256_load_pd( &C0[0+ldc*6] );
				d_3 = _mm256_load_pd( &C0[0+ldc*7] );

				d_0 = _mm256_add_pd( d_4, c_4 );
				d_1 = _mm256_add_pd( d_5, c_5 );
				d_2 = _mm256_add_pd( d_6, c_6 );
				d_3 = _mm256_add_pd( d_7, c_7 );

				d_0 = _mm256_load_pd( &C0[0+ldc*8] );
				d_1 = _mm256_load_pd( &C0[0+ldc*9] );
				d_2 = _mm256_load_pd( &C0[0+ldc*10] );
				d_3 = _mm256_load_pd( &C0[0+ldc*11] );

				d_0 = _mm256_add_pd( d_8, c_8 );
				d_1 = _mm256_add_pd( d_9, c_9 );
				d_2 = _mm256_add_pd( d_a, c_a );
				d_3 = _mm256_add_pd( d_b, c_b );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+ldc*4] );
				d_5 = _mm256_load_pd( &C0[0+ldc*5] );
				d_6 = _mm256_load_pd( &C0[0+ldc*6] );
				d_7 = _mm256_load_pd( &C0[0+ldc*7] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C0[0+ldc*8] );
				d_9 = _mm256_load_pd( &C0[0+ldc*9] );
				d_a = _mm256_load_pd( &C0[0+ldc*10] );
				d_b = _mm256_load_pd( &C0[0+ldc*11] );

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
	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D0[0+ldc*2], d_2 );

	_mm256_store_pd( &D1[0+ldc*0], d_4 );
	_mm256_store_pd( &D1[0+ldc*1], d_5 );
	_mm256_store_pd( &D1[0+ldc*2], d_6 );

	_mm256_maskstore_pd( &D2[0+ldc*0], mask_m, d_8 );
	_mm256_maskstore_pd( &D2[0+ldc*1], mask_m, d_9 );
	_mm256_maskstore_pd( &D2[0+ldc*2], mask_m, d_a );

	if(kn>=4)
		{
		_mm256_store_pd( &D0[0+ldc*3], d_3 );
		_mm256_store_pd( &D1[0+ldc*3], d_7 );
		_mm256_maskstore_pd( &D2[0+ldc*3], mask_m, d_b );
		}
	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );

	_mm256_maskstore_pd( &D0[0+ldc*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+ldc*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+ldc*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+ldc*3], mask_n, d_3 );

	_mm256_maskstore_pd( &D0[0+ldc*4], mask_n, d_4 );
	_mm256_maskstore_pd( &D0[0+ldc*5], mask_n, d_5 );
	_mm256_maskstore_pd( &D0[0+ldc*6], mask_n, d_6 );
	_mm256_maskstore_pd( &D0[0+ldc*7], mask_n, d_7 );

	if(km>=12)
		{
		_mm256_maskstore_pd( &D0[0+ldc*8], mask_n, d_8 );
		_mm256_maskstore_pd( &D0[0+ldc*9], mask_n, d_9 );
		_mm256_maskstore_pd( &D0[0+ldc*10], mask_n, d_a );
		_mm256_maskstore_pd( &D0[0+ldc*11], mask_n, d_b );
		}
	else
		{
		_mm256_maskstore_pd( &D0[0+ldc*8], mask_n, d_8 );
		if(km>=10)
			{
			_mm256_maskstore_pd( &D0[0+ldc*9], mask_n, d_9 );
			if(km>10)
				{
				_mm256_maskstore_pd( &D0[0+ldc*10], mask_n, d_a );
				}
			}
		}
		
	return;

}


#endif



void kernel_dgemm_nt_10x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *A2 = A1 + 4*sda;
	double *C2 = C1 + 4*sdc;
	double *D2 = D1 + 4*sdd;
	
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567, a_8989,
		b_0123, b_temp,
		ab_tmp0,
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71,
		c_80_91_82_93, c_81_90_83_92;
	
	__m128d
		u_8, u_9, u_a, u_b,
		v_8, v_9, v_a, v_b;

	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		c_8, c_9,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9,
		t_0, t_1, t_2, t_3;

	__m256i mask_n;

	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();
	c_40_51_62_73 = _mm256_setzero_pd();
	c_41_50_63_72 = _mm256_setzero_pd();
	c_43_52_61_70 = _mm256_setzero_pd();
	c_42_53_60_71 = _mm256_setzero_pd();
	c_80_91_82_93 = _mm256_setzero_pd();
	c_81_90_83_92 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_8989, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_82_93 = _mm256_add_pd( c_80_91_82_93, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_8989, b_temp );
		a_8989        = _mm256_broadcast_pd( (__m128d *) &A2[4] ); // prefetch
		c_81_90_83_92 = _mm256_add_pd( c_81_90_83_92, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		

/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_8989, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_82_93 = _mm256_add_pd( c_80_91_82_93, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_8989, b_temp );
		a_8989        = _mm256_broadcast_pd( (__m128d *) &A2[8] ); // prefetch
		c_81_90_83_92 = _mm256_add_pd( c_81_90_83_92, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
	

/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_8989, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_82_93 = _mm256_add_pd( c_80_91_82_93, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_8989, b_temp );
		a_8989        = _mm256_broadcast_pd( (__m128d *) &A2[12] ); // prefetch
		c_81_90_83_92 = _mm256_add_pd( c_81_90_83_92, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
	

/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( a_8989, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_82_93 = _mm256_add_pd( c_80_91_82_93, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_8989, b_temp );
		a_8989        = _mm256_broadcast_pd( (__m128d *) &A2[16] ); // prefetch
		c_81_90_83_92 = _mm256_add_pd( c_81_90_83_92, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_8989, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_82_93 = _mm256_add_pd( c_80_91_82_93, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_8989, b_temp );
		a_8989        = _mm256_broadcast_pd( (__m128d *) &A2[4] ); // prefetch
		c_81_90_83_92 = _mm256_add_pd( c_81_90_83_92, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		

/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_8989, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_82_93 = _mm256_add_pd( c_80_91_82_93, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_8989, b_temp );
		a_8989        = _mm256_broadcast_pd( (__m128d *) &A2[8] ); // prefetch
		c_81_90_83_92 = _mm256_add_pd( c_81_90_83_92, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
	
		
		A0 += 8;
		A1 += 8;
		A2 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_8989, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		c_80_91_82_93 = _mm256_add_pd( c_80_91_82_93, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_8989, b_temp );
//		a_8989        = _mm256_broadcast_pd( (__m128d *) &A2[4] ); // prefetch
		c_81_90_83_92 = _mm256_add_pd( c_81_90_83_92, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
		b_0123        = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );

		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_temp        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
//		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_temp );
//		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_temp );
//		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		
		}
	
	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			t_0 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			t_1 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			t_2 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			t_3 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );

			d_0 = _mm256_blend_pd( t_0, t_2, 0xc );
			d_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
			d_1 = _mm256_blend_pd( t_1, t_3, 0xc );
			d_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

			t_0 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
			t_1 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
			t_2 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
			t_3 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
			
			d_4 = _mm256_blend_pd( t_0, t_2, 0xc );
			d_6 = _mm256_blend_pd( t_0, t_2, 0x3 );
			d_5 = _mm256_blend_pd( t_1, t_3, 0xc );
			d_7 = _mm256_blend_pd( t_1, t_3, 0x3 );

			t_0 = _mm256_blend_pd( c_80_91_82_93, c_81_90_83_92, 0xa );
			t_1 = _mm256_blend_pd( c_80_91_82_93, c_81_90_83_92, 0x5 );
			
			v_a = _mm256_extractf128_pd( t_0, 0x1 );
			v_8 = _mm256_castpd256_pd128( t_0 );
			v_b = _mm256_extractf128_pd( t_1, 0x1 );
			v_9 = _mm256_castpd256_pd128( t_1 );

			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( c_00_11_22_33, c_01_10_23_32 );
			t_1 = _mm256_unpackhi_pd( c_01_10_23_32, c_00_11_22_33 );
			t_2 = _mm256_unpacklo_pd( c_02_13_20_31, c_03_12_21_30 );
			t_3 = _mm256_unpackhi_pd( c_03_12_21_30, c_02_13_20_31 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			d_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			t_0 = _mm256_unpacklo_pd( c_40_51_62_73, c_41_50_63_72 );
			t_1 = _mm256_unpackhi_pd( c_41_50_63_72, c_40_51_62_73 );
			t_2 = _mm256_unpacklo_pd( c_42_53_60_71, c_43_52_61_70 );
			t_3 = _mm256_unpackhi_pd( c_43_52_61_70, c_42_53_60_71 );

			d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_6 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			d_7 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			d_8 = _mm256_unpacklo_pd( c_80_91_82_93, c_81_90_83_92 );
			d_9 = _mm256_unpackhi_pd( c_81_90_83_92, c_80_91_82_93 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			t_0 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			t_1 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			t_2 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			t_3 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );

			c_0 = _mm256_blend_pd( t_0, t_2, 0xc );
			c_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
			c_1 = _mm256_blend_pd( t_1, t_3, 0xc );
			c_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

			t_0 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
			t_1 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
			t_2 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
			t_3 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
			
			c_4 = _mm256_blend_pd( t_0, t_2, 0xc );
			c_6 = _mm256_blend_pd( t_0, t_2, 0x3 );
			c_5 = _mm256_blend_pd( t_1, t_3, 0xc );
			c_7 = _mm256_blend_pd( t_1, t_3, 0x3 );

			t_0 = _mm256_blend_pd( c_80_91_82_93, c_81_90_83_92, 0xa );
			t_1 = _mm256_blend_pd( c_80_91_82_93, c_81_90_83_92, 0x5 );
			
			u_a = _mm256_extractf128_pd( t_0, 0x1 );
			u_8 = _mm256_castpd256_pd128( t_0 );
			u_b = _mm256_extractf128_pd( t_1, 0x1 );
			u_9 = _mm256_castpd256_pd128( t_1 );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );
			
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+ldc*0] );
				d_5 = _mm256_load_pd( &C1[0+ldc*1] );
				d_6 = _mm256_load_pd( &C1[0+ldc*2] );
				d_7 = _mm256_load_pd( &C1[0+ldc*3] );
			
				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );
			
				v_8 = _mm_load_pd( &C2[0+ldc*0] );
				v_9 = _mm_load_pd( &C2[0+ldc*1] );
				v_a = _mm_load_pd( &C2[0+ldc*2] );
				v_b = _mm_load_pd( &C2[0+ldc*3] );

				v_8 = _mm_add_pd( v_8, u_8 );
				v_9 = _mm_add_pd( v_9, u_9 );
				v_a = _mm_add_pd( v_a, u_a );
				v_b = _mm_add_pd( v_b, u_b );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );
			
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C1[0+ldc*0] );
				d_5 = _mm256_load_pd( &C1[0+ldc*1] );
				d_6 = _mm256_load_pd( &C1[0+ldc*2] );
				d_7 = _mm256_load_pd( &C1[0+ldc*3] );
			
				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );
			
				v_8 = _mm_load_pd( &C2[0+ldc*0] );
				v_9 = _mm_load_pd( &C2[0+ldc*1] );
				v_a = _mm_load_pd( &C2[0+ldc*2] );
				v_b = _mm_load_pd( &C2[0+ldc*3] );

				v_8 = _mm_sub_pd( v_8, u_8 );
				v_9 = _mm_sub_pd( v_9, u_9 );
				v_a = _mm_sub_pd( v_a, u_a );
				v_b = _mm_sub_pd( v_b, u_b );
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else
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

				t_0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( v_8 ), v_a, 0x1 );
				t_1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( v_9 ), v_b, 0x1 );

				d_8 = _mm256_unpacklo_pd( t_0, t_1 );
				d_9 = _mm256_unpackhi_pd( t_0, t_1 );

				goto store_t;
				}

			}
		else // t(C)
			{

			t_0 = _mm256_unpacklo_pd( c_00_11_22_33, c_01_10_23_32 );
			t_1 = _mm256_unpackhi_pd( c_01_10_23_32, c_00_11_22_33 );
			t_2 = _mm256_unpacklo_pd( c_02_13_20_31, c_03_12_21_30 );
			t_3 = _mm256_unpackhi_pd( c_03_12_21_30, c_02_13_20_31 );

			c_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			c_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			c_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			c_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			t_0 = _mm256_unpacklo_pd( c_40_51_62_73, c_41_50_63_72 );
			t_1 = _mm256_unpackhi_pd( c_41_50_63_72, c_40_51_62_73 );
			t_2 = _mm256_unpacklo_pd( c_42_53_60_71, c_43_52_61_70 );
			t_3 = _mm256_unpackhi_pd( c_43_52_61_70, c_42_53_60_71 );

			c_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			c_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			c_6 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			c_7 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			c_8 = _mm256_unpacklo_pd( c_80_91_82_93, c_81_90_83_92 );
			c_9 = _mm256_unpackhi_pd( c_81_90_83_92, c_80_91_82_93 );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+ldc*4] );
				d_5 = _mm256_load_pd( &C0[0+ldc*5] );
				d_6 = _mm256_load_pd( &C0[0+ldc*6] );
				d_7 = _mm256_load_pd( &C0[0+ldc*7] );

				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				d_6 = _mm256_add_pd( d_6, c_6 );
				d_7 = _mm256_add_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C0[0+ldc*8] );
				d_9 = _mm256_load_pd( &C0[0+ldc*9] );

				d_8 = _mm256_add_pd( d_8, c_8 );
				d_9 = _mm256_add_pd( d_9, c_9 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_load_pd( &C0[0+ldc*0] );
				d_1 = _mm256_load_pd( &C0[0+ldc*1] );
				d_2 = _mm256_load_pd( &C0[0+ldc*2] );
				d_3 = _mm256_load_pd( &C0[0+ldc*3] );

				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_load_pd( &C0[0+ldc*4] );
				d_5 = _mm256_load_pd( &C0[0+ldc*5] );
				d_6 = _mm256_load_pd( &C0[0+ldc*6] );
				d_7 = _mm256_load_pd( &C0[0+ldc*7] );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
				d_6 = _mm256_sub_pd( d_6, c_6 );
				d_7 = _mm256_sub_pd( d_7, c_7 );

				d_8 = _mm256_load_pd( &C0[0+ldc*8] );
				d_9 = _mm256_load_pd( &C0[0+ldc*9] );

				d_8 = _mm256_sub_pd( d_8, c_8 );
				d_9 = _mm256_sub_pd( d_9, c_9 );
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

				v_a = _mm256_extractf128_pd( t_0, 0x1 );
				v_8 = _mm256_castpd256_pd128( t_0 );
				v_b = _mm256_extractf128_pd( t_1, 0x1 );
				v_9 = _mm256_castpd256_pd128( t_1 );

				goto store_n;
				}
			else
				{
				goto store_t;
				}

			}

		}
	
	// store (9 - 10) x (3 - 4)
	store_n:
	if(km>=10)
		{
		_mm256_store_pd( &D0[0+ldc*0], d_0 );
		_mm256_store_pd( &D0[0+ldc*1], d_1 );
		_mm256_store_pd( &D0[0+ldc*2], d_2 );

		_mm256_store_pd( &D1[0+ldc*0], d_4 );
		_mm256_store_pd( &D1[0+ldc*1], d_5 );
		_mm256_store_pd( &D1[0+ldc*2], d_6 );

		_mm_store_pd( &D2[0+ldc*0], v_8 );
		_mm_store_pd( &D2[0+ldc*1], v_9 );
		_mm_store_pd( &D2[0+ldc*2], v_a );

		if(kn>=4)
			{
			_mm256_store_pd( &D0[0+ldc*3], d_3 );
			_mm256_store_pd( &D1[0+ldc*3], d_7 );
			_mm_store_pd( &D2[0+ldc*3], v_b );
			}
		}
	else
		{
		_mm256_store_pd( &D0[0+ldc*0], d_0 );
		_mm256_store_pd( &D0[0+ldc*1], d_1 );
		_mm256_store_pd( &D0[0+ldc*2], d_2 );

		_mm256_store_pd( &D1[0+ldc*0], d_4 );
		_mm256_store_pd( &D1[0+ldc*1], d_5 );
		_mm256_store_pd( &D1[0+ldc*2], d_6 );

		_mm_store_sd( &D2[0+ldc*0], v_8 );
		_mm_store_sd( &D2[0+ldc*1], v_9 );
		_mm_store_sd( &D2[0+ldc*2], v_a );

		if(kn>=4)
			{
			_mm256_store_pd( &D0[0+ldc*3], d_3 );
			_mm256_store_pd( &D1[0+ldc*3], d_7 );
			_mm_store_sd( &D2[0+ldc*3], v_b );
			}
		}

	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );
	
	_mm256_maskstore_pd( &D0[0+ldc*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+ldc*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+ldc*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+ldc*3], mask_n, d_3 );
	_mm256_maskstore_pd( &D0[0+ldc*4], mask_n, d_4 );
	_mm256_maskstore_pd( &D0[0+ldc*5], mask_n, d_5 );
	_mm256_maskstore_pd( &D0[0+ldc*6], mask_n, d_6 );
	_mm256_maskstore_pd( &D0[0+ldc*7], mask_n, d_7 );
	_mm256_maskstore_pd( &D0[0+ldc*8], mask_n, d_8 );

	if(km>=10)
		{
		_mm256_maskstore_pd( &D0[0+ldc*9], mask_n, d_9 );
		}

	return;

	}



void kernel_dgemm_nt_8x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs  = 4;
	const int ldc = 4;

	int k;

	if(alg==0)
		tc = td;

	int low_rank = 0;
	if(tc==0)
		low_rank = LOW_RANK_N;
	else
		low_rank = LOW_RANK_T;

	
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

	if(kmax<=0)
		goto add_n; // it is the same for add_t;

	// low-rank update
	if(kmax<=low_rank)
		{

		if(tc==0)
			{

			k = 0;

			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_4567 = _mm256_load_pd( &A1[0+bs*0] );
			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );

#if (LOW_RANK_N>=4)
#if (LOW_RANK_N<8)
			if(k<kmax-3)
#else
			for(; k<kmax-3; )
#endif
				{

				b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				A_0123 = _mm256_load_pd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				A_4567 = _mm256_load_pd( &A1[0+bs*1] );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				


				b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				a_0123 = _mm256_load_pd( &A0[0+bs*2] );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				a_4567 = _mm256_load_pd( &A1[0+bs*2] );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );



				b_1    = _mm256_broadcast_sd( &B[1+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				A_0123 = _mm256_load_pd( &A0[0+bs*3] );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				A_4567 = _mm256_load_pd( &A1[0+bs*3] );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				


				b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				a_0123 = _mm256_load_pd( &A0[0+bs*4] );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				a_4567 = _mm256_load_pd( &A1[0+bs*4] );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*4] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );


				A0 += 16;
				A1 += 16;
				B  += 16;
				k  += 4;

				}
#endif
			if(k<kmax-1)
				{


				b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				A_0123 = _mm256_load_pd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				A_4567 = _mm256_load_pd( &A1[0+bs*1] );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				


				b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				a_0123 = _mm256_load_pd( &A0[0+bs*2] );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				a_4567 = _mm256_load_pd( &A1[0+bs*2] );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );


				A0 += 8;
				A1 += 8;
				B  += 8;
				k  += 2;

				}
			if(k<kmax)
				{


				b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
	//			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
	//			a_4567 = _mm256_load_pd( &A1[0+bs*0] );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

	//			b_0    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

		//		A += 4;
		//		B += 4;
		//		k += 1;

				}

			goto add_n;

			}
		else
			{

			k = 0;

			a_0123 = _mm256_load_pd( &B[0+bs*0] );
			b_0    = _mm256_broadcast_sd( &A0[0+bs*0] );

#if (LOW_RANK_T>=4)
#if (LOW_RANK_T<8)
			if(k<kmax-3)
#else
			for(; k<kmax-3; )
#endif
				{

				A_0123 = _mm256_load_pd( &B[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &B[0+bs*2] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				A_0123 = _mm256_load_pd( &B[0+bs*3] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*3] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &B[0+bs*4] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*4] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				A0 += 16;
				A1 += 16;
				B  += 16;
				k  += 4;

				}
#endif
			if(k<kmax-1)
				{

				A_0123 = _mm256_load_pd( &B[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &B[0+bs*2] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
	

				A0 += 8;
				A1 += 8;
				B  += 8;
				k  += 2;

				}
			if(k<kmax)
				{

//				A_0123 = _mm256_load_pd( &B[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
//				b_0    = _mm256_broadcast_sd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
	
		//		A += 4;
		//		B += 4;
		//		k += 1;

				}

			goto add_t;

			}

		}
	else // high-rank update
		{

		// prefetch
		a_0123 = _mm256_load_pd( &A0[0] );
		a_4567 = _mm256_load_pd( &A1[0] );
		b_0123 = _mm256_load_pd( &B[0] );

		for(k=0; k<kmax-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &A1[4] ); // prefetch
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
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &A1[8] ); // prefetch
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


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[12] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[12] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &A1[12] ); // prefetch
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


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &A1[16] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[16] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
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
			
			A0 += 16;
			A1 += 16;
			B  += 16;

			}
		
		if(kmax%4>=2)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &A1[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &A1[8] ); // prefetch
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
			
			
			A0 += 8;
			A1 += 8;
			B  += 8;

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

		}

	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;
	

	if(tc!=0)
		{
		goto end_t;
		}


	// tc==0
	end_n:

	// AB + C
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

	add_n:

	if(alg==0)
		{
		goto store_n;
		}
	else if(alg==1) // AB = A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );
	
		d_0 = _mm256_add_pd( c_0, d_0 );
		d_1 = _mm256_add_pd( c_1, d_1 );
		d_2 = _mm256_add_pd( c_2, d_2 );
		d_3 = _mm256_add_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C1[0+ldc*0] );
		c_5 = _mm256_load_pd( &C1[0+ldc*1] );
		c_6 = _mm256_load_pd( &C1[0+ldc*2] );
		c_7 = _mm256_load_pd( &C1[0+ldc*3] );
	
		d_4 = _mm256_add_pd( c_4, d_4 );
		d_5 = _mm256_add_pd( c_5, d_5 );
		d_6 = _mm256_add_pd( c_6, d_6 );
		d_7 = _mm256_add_pd( c_7, d_7 );
		}
	else // AB = - A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );
	
		d_0 = _mm256_sub_pd( c_0, d_0 );
		d_1 = _mm256_sub_pd( c_1, d_1 );
		d_2 = _mm256_sub_pd( c_2, d_2 );
		d_3 = _mm256_sub_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C1[0+ldc*0] );
		c_5 = _mm256_load_pd( &C1[0+ldc*1] );
		c_6 = _mm256_load_pd( &C1[0+ldc*2] );
		c_7 = _mm256_load_pd( &C1[0+ldc*3] );
	
		d_4 = _mm256_sub_pd( c_4, d_4 );
		d_5 = _mm256_sub_pd( c_5, d_5 );
		d_6 = _mm256_sub_pd( c_6, d_6 );
		d_7 = _mm256_sub_pd( c_7, d_7 );
		}

	if(td==0) // AB + C
		{
		goto store_n;
		}
	else
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


	// tc==1
	end_t:

	t_0 = _mm256_unpacklo_pd( d_0, d_1 );
	t_1 = _mm256_unpackhi_pd( d_1, d_0 );
	t_2 = _mm256_unpacklo_pd( d_2, d_3 );
	t_3 = _mm256_unpackhi_pd( d_3, d_2 );

	d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
	d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
	d_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
	d_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

	t_0 = _mm256_unpacklo_pd( d_4, d_5 );
	t_1 = _mm256_unpackhi_pd( d_5, d_4 );
	t_2 = _mm256_unpacklo_pd( d_6, d_7 );
	t_3 = _mm256_unpackhi_pd( d_7, d_6 );

	d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
	d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
	d_6 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
	d_7 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

	add_t:

	if(alg==0)
		{
		goto store_t;
		}
	else if(alg==1) // AB = A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );

		d_0 = _mm256_add_pd( c_0, d_0 );
		d_1 = _mm256_add_pd( c_1, d_1 );
		d_2 = _mm256_add_pd( c_2, d_2 );
		d_3 = _mm256_add_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C0[0+ldc*4] );
		c_5 = _mm256_load_pd( &C0[0+ldc*5] );
		c_6 = _mm256_load_pd( &C0[0+ldc*6] );
		c_7 = _mm256_load_pd( &C0[0+ldc*7] );

		d_4 = _mm256_add_pd( c_4, d_4 );
		d_5 = _mm256_add_pd( c_5, d_5 );
		d_6 = _mm256_add_pd( c_6, d_6 );
		d_7 = _mm256_add_pd( c_7, d_7 );
		}
	else // AB = - A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );

		d_0 = _mm256_sub_pd( c_0, d_0 );
		d_1 = _mm256_sub_pd( c_1, d_1 );
		d_2 = _mm256_sub_pd( c_2, d_2 );
		d_3 = _mm256_sub_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C0[0+ldc*4] );
		c_5 = _mm256_load_pd( &C0[0+ldc*5] );
		c_6 = _mm256_load_pd( &C0[0+ldc*6] );
		c_7 = _mm256_load_pd( &C0[0+ldc*7] );

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
	else
		{
		goto store_t;
		}

	
	// store (5 - 8) x (3 - 4)
	store_n:
	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_store_pd( &D0[0+ldc*3], d_3 );

	_mm256_store_pd( &D1[0+ldc*0], d_4 );
	_mm256_store_pd( &D1[0+ldc*1], d_5 );
	_mm256_store_pd( &D1[0+ldc*2], d_6 );
	_mm256_store_pd( &D1[0+ldc*3], d_7 );

	return;

	store_t:
	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_store_pd( &D0[0+ldc*3], d_3 );

	_mm256_store_pd( &D0[0+ldc*4], d_4 );
	_mm256_store_pd( &D0[0+ldc*5], d_5 );
	_mm256_store_pd( &D0[0+ldc*6], d_6 );
	_mm256_store_pd( &D0[0+ldc*7], d_7 );

	return;

	}



#if 0
void kernel_dgemm_nt_8x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs  = 4;
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		temp,
		c_00, c_01, c_02, c_03,
		c_40, c_41, c_42, c_43,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_62_73, c_41_50_63_72, c_43_52_61_70, c_42_53_60_71;
	
	if(kmax<8 && tc==0 && td==0)
		{

		k = 0;

		a_0123 = _mm256_load_pd( &A0[0+bs*0] );
		a_4567 = _mm256_load_pd( &A1[0+bs*0] );
		b_0    = _mm256_broadcast_sd( &B[0+bs*0] );

#if 1
		//for(; k<kmax-3; )
		if(k<kmax-3)
			{

			A_0123 = _mm256_load_pd( &A0[0+bs*1] );
			A_4567 = _mm256_load_pd( &A1[0+bs*1] );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_00   = _mm256_add_pd( c_00, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_40   = _mm256_add_pd( c_40, temp );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_01   = _mm256_add_pd( c_01, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_41   = _mm256_add_pd( c_41, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_02   = _mm256_add_pd( c_02, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_42   = _mm256_add_pd( c_42, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_03   = _mm256_add_pd( c_03, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_43   = _mm256_add_pd( c_43, temp );
			

			a_0123 = _mm256_load_pd( &A0[0+bs*2] );
			a_4567 = _mm256_load_pd( &A1[0+bs*2] );

			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			c_00   = _mm256_add_pd( c_00, temp );
			temp   = _mm256_mul_pd( A_4567, b_0 );
			c_40   = _mm256_add_pd( c_40, temp );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			c_01   = _mm256_add_pd( c_01, temp );
			temp   = _mm256_mul_pd( A_4567, b_1 );
			c_41   = _mm256_add_pd( c_41, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			c_02   = _mm256_add_pd( c_02, temp );
			temp   = _mm256_mul_pd( A_4567, b_0 );
			c_42   = _mm256_add_pd( c_42, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			c_03   = _mm256_add_pd( c_03, temp );
			temp   = _mm256_mul_pd( A_4567, b_1 );
			c_43   = _mm256_add_pd( c_43, temp );


			A_0123 = _mm256_load_pd( &A0[0+bs*3] );
			A_4567 = _mm256_load_pd( &A1[0+bs*3] );

			b_1    = _mm256_broadcast_sd( &B[1+bs*2] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_00   = _mm256_add_pd( c_00, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_40   = _mm256_add_pd( c_40, temp );

			b_0    = _mm256_broadcast_sd( &B[2+bs*2] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_01   = _mm256_add_pd( c_01, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_41   = _mm256_add_pd( c_41, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*2] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_02   = _mm256_add_pd( c_02, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_42   = _mm256_add_pd( c_42, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_03   = _mm256_add_pd( c_03, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_43   = _mm256_add_pd( c_43, temp );
			

			a_0123 = _mm256_load_pd( &A0[0+bs*4] );
			a_4567 = _mm256_load_pd( &A1[0+bs*4] );

			b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			c_00   = _mm256_add_pd( c_00, temp );
			temp   = _mm256_mul_pd( A_4567, b_0 );
			c_40   = _mm256_add_pd( c_40, temp );

			b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			c_01   = _mm256_add_pd( c_01, temp );
			temp   = _mm256_mul_pd( A_4567, b_1 );
			c_41   = _mm256_add_pd( c_41, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			c_02   = _mm256_add_pd( c_02, temp );
			temp   = _mm256_mul_pd( A_4567, b_0 );
			c_42   = _mm256_add_pd( c_42, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*4] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			c_03   = _mm256_add_pd( c_03, temp );
			temp   = _mm256_mul_pd( A_4567, b_1 );
			c_43   = _mm256_add_pd( c_43, temp );


			A0 += 16;
			A1 += 16;
			B  += 16;
			k  += 4;

			}
#endif
		if(k<kmax-1)
			{

			A_0123 = _mm256_load_pd( &A0[0+bs*1] );
			A_4567 = _mm256_load_pd( &A1[0+bs*1] );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_00   = _mm256_add_pd( c_00, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_40   = _mm256_add_pd( c_40, temp );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_01   = _mm256_add_pd( c_01, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_41   = _mm256_add_pd( c_41, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_02   = _mm256_add_pd( c_02, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_42   = _mm256_add_pd( c_42, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_03   = _mm256_add_pd( c_03, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_43   = _mm256_add_pd( c_43, temp );
			

			a_0123 = _mm256_load_pd( &A0[0+bs*2] );
			a_4567 = _mm256_load_pd( &A1[0+bs*2] );

			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			c_00   = _mm256_add_pd( c_00, temp );
			temp   = _mm256_mul_pd( A_4567, b_0 );
			c_40   = _mm256_add_pd( c_40, temp );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			c_01   = _mm256_add_pd( c_01, temp );
			temp   = _mm256_mul_pd( A_4567, b_1 );
			c_41   = _mm256_add_pd( c_41, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			c_02   = _mm256_add_pd( c_02, temp );
			temp   = _mm256_mul_pd( A_4567, b_0 );
			c_42   = _mm256_add_pd( c_42, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			c_03   = _mm256_add_pd( c_03, temp );
			temp   = _mm256_mul_pd( A_4567, b_1 );
			c_43   = _mm256_add_pd( c_43, temp );


			A0 += 8;
			A1 += 8;
			B  += 8;
			k  += 2;

			}
		if(k<kmax)
			{

//			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
//			a_4567 = _mm256_load_pd( &A1[0+bs*0] );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_00   = _mm256_add_pd( c_00, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_40   = _mm256_add_pd( c_40, temp );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_01   = _mm256_add_pd( c_01, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_41   = _mm256_add_pd( c_41, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			c_02   = _mm256_add_pd( c_02, temp );
			temp   = _mm256_mul_pd( a_4567, b_0 );
			c_42   = _mm256_add_pd( c_42, temp );

//			b_0    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			c_03   = _mm256_add_pd( c_03, temp );
			temp   = _mm256_mul_pd( a_4567, b_1 );
			c_43   = _mm256_add_pd( c_43, temp );
			

	//		A += 4;
	//		B += 4;
	//		k += 1;

			}

		if(alg!=0)
			{
			d_00 = _mm256_load_pd( &D0[0+bs+0] );
			d_01 = _mm256_load_pd( &D0[0+bs+1] );
			d_02 = _mm256_load_pd( &D0[0+bs+2] );
			d_03 = _mm256_load_pd( &D0[0+bs+3] );
			d_40 = _mm256_load_pd( &D1[0+bs+0] );
			d_41 = _mm256_load_pd( &D1[0+bs+1] );
			d_42 = _mm256_load_pd( &D1[0+bs+2] );
			d_43 = _mm256_load_pd( &D1[0+bs+3] );

			if(alg==1)
				{
				c_00 = _mm256_add_pd( d_00, c_00 );
				c_01 = _mm256_add_pd( d_01, c_01 );
				c_02 = _mm256_add_pd( d_02, c_02 );
				c_03 = _mm256_add_pd( d_03, c_03 );
				c_40 = _mm256_add_pd( d_40, c_40 );
				c_41 = _mm256_add_pd( d_41, c_41 );
				c_42 = _mm256_add_pd( d_42, c_42 );
				c_43 = _mm256_add_pd( d_43, c_43 );
				}
			else
				{
				c_00 = _mm256_sub_pd( d_00, c_00 );
				c_01 = _mm256_sub_pd( d_01, c_01 );
				c_02 = _mm256_sub_pd( d_02, c_02 );
				c_03 = _mm256_sub_pd( d_03, c_03 );
				c_40 = _mm256_sub_pd( d_40, c_40 );
				c_41 = _mm256_sub_pd( d_41, c_41 );
				c_42 = _mm256_sub_pd( d_42, c_42 );
				c_43 = _mm256_sub_pd( d_43, c_43 );
				}

			}

		_mm256_store_pd( &D0[0+bs*0], c_00 );
		_mm256_store_pd( &D0[0+bs*1], c_01 );
		_mm256_store_pd( &D0[0+bs*2], c_02 );
		_mm256_store_pd( &D0[0+bs*3], c_03 );
		_mm256_store_pd( &D1[0+bs*0], c_40 );
		_mm256_store_pd( &D1[0+bs*1], c_41 );
		_mm256_store_pd( &D1[0+bs*2], c_42 );
		_mm256_store_pd( &D1[0+bs*3], c_43 );

		return;

		}

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


#if 0
	for(k=0; k<kmax-3; k+=4) // TODO prefetch A0 and A1 using 2 extra registers ????????????????
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
	
	if(kmax%4>=2)
		{
		
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
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
/*		b_0123        = _mm256_load_pd( &B[4] ); // prefetch*/
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
/*		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch*/
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
/*		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch*/
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp1 );
		
		}
#else

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( A_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_1032 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_3210 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_1032 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );


/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );


/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( A_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_1032 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_3210 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_1032 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		
		ab_tmp0       = _mm256_mul_pd( A_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_1032 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_3210 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4567, b_1032 );
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
/*		b_0123        = _mm256_load_pd( &B[4] ); // prefetch*/
		c_40_51_62_73 = _mm256_add_pd( c_40_51_62_73, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
		c_41_50_63_72 = _mm256_add_pd( c_41_50_63_72, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
		c_43_52_61_70 = _mm256_add_pd( c_43_52_61_70, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
/*		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch*/
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
/*		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch*/
		c_42_53_60_71 = _mm256_add_pd( c_42_53_60_71, ab_tmp0 );
		
		}
#endif

	__m256d
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_40_50_62_72, c_41_51_63_73, c_42_52_60_70, c_43_53_61_71,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		c_40_50_60_70, c_41_51_61_71, c_42_52_62_72, c_43_53_63_73,
		c_00_01_22_23, c_10_11_32_33, c_02_03_20_21, c_12_13_30_31,
		c_40_41_62_63, c_50_51_72_73, c_42_43_60_61, c_52_53_70_71,
		c_00_01_02_03, c_10_11_12_13, c_20_21_22_23, c_30_31_32_33,
		c_40_41_42_43, c_50_51_52_53, c_60_61_62_63, c_70_71_72_73,
		c_00_01_20_21, c_10_11_30_31, c_02_03_22_23, c_12_13_32_33,
		c_40_41_60_61, c_50_51_70_71, c_42_43_62_63, c_52_53_72_73,
		d_00_01_02_03, d_10_11_12_13, d_20_21_22_23, d_30_31_32_33,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33,
		d_40_50_60_70, d_41_51_61_71, d_42_52_62_72, d_43_53_63_73,
		d_00_10_02_12, d_01_11_03_13, d_20_30_22_32, d_21_31_23_33; 

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );

			c_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
			c_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
			c_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
			c_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );

			_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
			_mm256_store_pd( &D0[0+ldc*1], c_01_11_21_31 );
			_mm256_store_pd( &D0[0+ldc*2], c_02_12_22_32 );
			_mm256_store_pd( &D0[0+ldc*3], c_03_13_23_33 );

			c_40_50_62_72 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
			c_41_51_63_73 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
			c_42_52_60_70 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
			c_43_53_61_71 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
			
			c_40_50_60_70 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0xc );
			c_42_52_62_72 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0x3 );
			c_41_51_61_71 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0xc );
			c_43_53_63_73 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0x3 );

			_mm256_store_pd( &D1[0+ldc*0], c_40_50_60_70 );
			_mm256_store_pd( &D1[0+ldc*1], c_41_51_61_71 );
			_mm256_store_pd( &D1[0+ldc*2], c_42_52_62_72 );
			_mm256_store_pd( &D1[0+ldc*3], c_43_53_63_73 );
			}
		else // transposed
			{
			c_00_01_22_23 = _mm256_unpacklo_pd( c_00_11_22_33, c_01_10_23_32 );
			c_10_11_32_33 = _mm256_unpackhi_pd( c_01_10_23_32, c_00_11_22_33 );
			c_02_03_20_21 = _mm256_unpacklo_pd( c_02_13_20_31, c_03_12_21_30 );
			c_12_13_30_31 = _mm256_unpackhi_pd( c_03_12_21_30, c_02_13_20_31 );

			c_00_01_02_03 = _mm256_permute2f128_pd( c_00_01_22_23, c_02_03_20_21, 0x20 );
			c_10_11_12_13 = _mm256_permute2f128_pd( c_10_11_32_33, c_12_13_30_31, 0x20 );
			c_20_21_22_23 = _mm256_permute2f128_pd( c_02_03_20_21, c_00_01_22_23, 0x31 );
			c_30_31_32_33 = _mm256_permute2f128_pd( c_12_13_30_31, c_10_11_32_33, 0x31 );

			_mm256_store_pd( &D0[0+ldc*0], c_00_01_02_03 );
			_mm256_store_pd( &D0[0+ldc*1], c_10_11_12_13 );
			_mm256_store_pd( &D0[0+ldc*2], c_20_21_22_23 );
			_mm256_store_pd( &D0[0+ldc*3], c_30_31_32_33 );

#if 0
			c_40_41_62_63 = _mm256_shuffle_pd( c_40_51_62_73, c_41_50_63_72, 0x0 );
			c_50_51_72_73 = _mm256_shuffle_pd( c_41_50_63_72, c_40_51_62_73, 0xf );
			c_42_43_60_61 = _mm256_shuffle_pd( c_42_53_60_71, c_43_52_61_70, 0x0 );
			c_52_53_70_71 = _mm256_shuffle_pd( c_43_52_61_70, c_42_53_60_71, 0xf );
#else
			c_40_41_62_63 = _mm256_unpacklo_pd( c_40_51_62_73, c_41_50_63_72 );
			c_50_51_72_73 = _mm256_unpackhi_pd( c_41_50_63_72, c_40_51_62_73 );
			c_42_43_60_61 = _mm256_unpacklo_pd( c_42_53_60_71, c_43_52_61_70 );
			c_52_53_70_71 = _mm256_unpackhi_pd( c_43_52_61_70, c_42_53_60_71 );
#endif

			c_40_41_42_43 = _mm256_permute2f128_pd( c_40_41_62_63, c_42_43_60_61, 0x20 );
			c_50_51_52_53 = _mm256_permute2f128_pd( c_50_51_72_73, c_52_53_70_71, 0x20 );
			c_60_61_62_63 = _mm256_permute2f128_pd( c_42_43_60_61, c_40_41_62_63, 0x31 );
			c_70_71_72_73 = _mm256_permute2f128_pd( c_52_53_70_71, c_50_51_72_73, 0x31 );

			_mm256_store_pd( &D0[0+ldc*4], c_40_41_42_43 );
			_mm256_store_pd( &D0[0+ldc*5], c_50_51_52_53 );
			_mm256_store_pd( &D0[0+ldc*6], c_60_61_62_63 );
			_mm256_store_pd( &D0[0+ldc*7], c_70_71_72_73 );
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
			
			c_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
			c_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
			c_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
			c_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );

			c_40_50_62_72 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0xa );
			c_41_51_63_73 = _mm256_blend_pd( c_40_51_62_73, c_41_50_63_72, 0x5 );
			c_42_52_60_70 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0xa );
			c_43_53_61_71 = _mm256_blend_pd( c_42_53_60_71, c_43_52_61_70, 0x5 );
			
			c_40_50_60_70 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0xc );
			c_42_52_62_72 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0x3 );
			c_41_51_61_71 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0xc );
			c_43_53_63_73 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0x3 );

			d_00_10_20_30 = _mm256_load_pd( &C0[0+ldc*0] );
			d_01_11_21_31 = _mm256_load_pd( &C0[0+ldc*1] );
			d_02_12_22_32 = _mm256_load_pd( &C0[0+ldc*2] );
			d_03_13_23_33 = _mm256_load_pd( &C0[0+ldc*3] );
			
			d_40_50_60_70 = _mm256_load_pd( &C1[0+ldc*0] );
			d_41_51_61_71 = _mm256_load_pd( &C1[0+ldc*1] );
			d_42_52_62_72 = _mm256_load_pd( &C1[0+ldc*2] );
			d_43_53_63_73 = _mm256_load_pd( &C1[0+ldc*3] );
			
			if(alg==1) // AB = A*B'
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
			else // AB = - A*B'
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

			if(td==0) // AB + C 
				{
				_mm256_store_pd( &D0[0+ldc*0], d_00_10_20_30 );
				_mm256_store_pd( &D0[0+ldc*1], d_01_11_21_31 );
				_mm256_store_pd( &D0[0+ldc*2], d_02_12_22_32 );
				_mm256_store_pd( &D0[0+ldc*3], d_03_13_23_33 );

				_mm256_store_pd( &D1[0+ldc*0], d_40_50_60_70 );
				_mm256_store_pd( &D1[0+ldc*1], d_41_51_61_71 );
				_mm256_store_pd( &D1[0+ldc*2], d_42_52_62_72 );
				_mm256_store_pd( &D1[0+ldc*3], d_43_53_63_73 );
				}
			else // t(AB + C)
				{
				c_00_01_20_21 = _mm256_unpacklo_pd( d_00_10_20_30, d_01_11_21_31 );
				c_10_11_30_31 = _mm256_unpackhi_pd( d_00_10_20_30, d_01_11_21_31 );
				c_02_03_22_23 = _mm256_unpacklo_pd( d_02_12_22_32, d_03_13_23_33 );
				c_12_13_32_33 = _mm256_unpackhi_pd( d_02_12_22_32, d_03_13_23_33 );

				c_00_01_02_03 = _mm256_permute2f128_pd( c_00_01_20_21, c_02_03_22_23, 0x20 );
				c_20_21_22_23 = _mm256_permute2f128_pd( c_00_01_20_21, c_02_03_22_23, 0x31 );
				c_10_11_12_13 = _mm256_permute2f128_pd( c_10_11_30_31, c_12_13_32_33, 0x20 );
				c_30_31_32_33 = _mm256_permute2f128_pd( c_10_11_30_31, c_12_13_32_33, 0x31 );

				_mm256_store_pd( &D0[0+ldc*0], c_00_01_02_03 );
				_mm256_store_pd( &D0[0+ldc*1], c_10_11_12_13 );
				_mm256_store_pd( &D0[0+ldc*2], c_20_21_22_23 );
				_mm256_store_pd( &D0[0+ldc*3], c_30_31_32_33 );

				c_40_41_60_61 = _mm256_unpacklo_pd( d_40_50_60_70, d_41_51_61_71 );
				c_50_51_70_71 = _mm256_unpackhi_pd( d_40_50_60_70, d_41_51_61_71 );
				c_42_43_62_63 = _mm256_unpacklo_pd( d_42_52_62_72, d_43_53_63_73 );
				c_52_53_72_73 = _mm256_unpackhi_pd( d_42_52_62_72, d_43_53_63_73 );

				c_40_41_42_43 = _mm256_permute2f128_pd( c_40_41_60_61, c_42_43_62_63, 0x20 );
				c_60_61_62_63 = _mm256_permute2f128_pd( c_40_41_60_61, c_42_43_62_63, 0x31 );
				c_50_51_52_53 = _mm256_permute2f128_pd( c_50_51_70_71, c_52_53_72_73, 0x20 );
				c_70_71_72_73 = _mm256_permute2f128_pd( c_50_51_70_71, c_52_53_72_73, 0x31 );

				_mm256_store_pd( &D0[0+ldc*4], c_40_41_42_43 );
				_mm256_store_pd( &D0[0+ldc*5], c_50_51_52_53 );
				_mm256_store_pd( &D0[0+ldc*6], c_60_61_62_63 );
				_mm256_store_pd( &D0[0+ldc*7], c_70_71_72_73 );
				}

			}
		else // t(C)
			{

			c_00_01_22_23 = _mm256_unpacklo_pd( c_00_11_22_33, c_01_10_23_32 );
			c_10_11_32_33 = _mm256_unpackhi_pd( c_01_10_23_32, c_00_11_22_33 );
			c_02_03_20_21 = _mm256_unpacklo_pd( c_02_13_20_31, c_03_12_21_30 );
			c_12_13_30_31 = _mm256_unpackhi_pd( c_03_12_21_30, c_02_13_20_31 );

			c_00_01_02_03 = _mm256_permute2f128_pd( c_00_01_22_23, c_02_03_20_21, 0x20 );
			c_10_11_12_13 = _mm256_permute2f128_pd( c_10_11_32_33, c_12_13_30_31, 0x20 );
			c_20_21_22_23 = _mm256_permute2f128_pd( c_02_03_20_21, c_00_01_22_23, 0x31 );
			c_30_31_32_33 = _mm256_permute2f128_pd( c_12_13_30_31, c_10_11_32_33, 0x31 );

			d_00_10_20_30 = _mm256_load_pd( &C0[0+ldc*0] );
			d_01_11_21_31 = _mm256_load_pd( &C0[0+ldc*1] );
			d_02_12_22_32 = _mm256_load_pd( &C0[0+ldc*2] );
			d_03_13_23_33 = _mm256_load_pd( &C0[0+ldc*3] );

			c_40_41_62_63 = _mm256_unpacklo_pd( c_40_51_62_73, c_41_50_63_72 );
			c_50_51_72_73 = _mm256_unpackhi_pd( c_41_50_63_72, c_40_51_62_73 );
			c_42_43_60_61 = _mm256_unpacklo_pd( c_42_53_60_71, c_43_52_61_70 );
			c_52_53_70_71 = _mm256_unpackhi_pd( c_43_52_61_70, c_42_53_60_71 );

			c_40_41_42_43 = _mm256_permute2f128_pd( c_40_41_62_63, c_42_43_60_61, 0x20 );
			c_50_51_52_53 = _mm256_permute2f128_pd( c_50_51_72_73, c_52_53_70_71, 0x20 );
			c_60_61_62_63 = _mm256_permute2f128_pd( c_42_43_60_61, c_40_41_62_63, 0x31 );
			c_70_71_72_73 = _mm256_permute2f128_pd( c_52_53_70_71, c_50_51_72_73, 0x31 );

			d_40_50_60_70 = _mm256_load_pd( &C0[0+ldc*4] );
			d_41_51_61_71 = _mm256_load_pd( &C0[0+ldc*5] );
			d_42_52_62_72 = _mm256_load_pd( &C0[0+ldc*6] );
			d_43_53_63_73 = _mm256_load_pd( &C0[0+ldc*7] );

			if(alg==1) // AB = A*B'
				{
				d_00_10_20_30 = _mm256_add_pd( d_00_10_20_30, c_00_01_02_03 );
				d_01_11_21_31 = _mm256_add_pd( d_01_11_21_31, c_10_11_12_13 );
				d_02_12_22_32 = _mm256_add_pd( d_02_12_22_32, c_20_21_22_23 );
				d_03_13_23_33 = _mm256_add_pd( d_03_13_23_33, c_30_31_32_33 );

				d_40_50_60_70 = _mm256_add_pd( d_40_50_60_70, c_40_41_42_43 );
				d_41_51_61_71 = _mm256_add_pd( d_41_51_61_71, c_50_51_52_53 );
				d_42_52_62_72 = _mm256_add_pd( d_42_52_62_72, c_60_61_62_63 );
				d_43_53_63_73 = _mm256_add_pd( d_43_53_63_73, c_70_71_72_73 );
				}
			else // AB = - A*B'
				{
				d_00_10_20_30 = _mm256_sub_pd( d_00_10_20_30, c_00_01_02_03 );
				d_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, c_10_11_12_13 );
				d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, c_20_21_22_23 );
				d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, c_30_31_32_33 );

				d_40_50_60_70 = _mm256_sub_pd( d_40_50_60_70, c_40_41_42_43 );
				d_41_51_61_71 = _mm256_sub_pd( d_41_51_61_71, c_50_51_52_53 );
				d_42_52_62_72 = _mm256_sub_pd( d_42_52_62_72, c_60_61_62_63 );
				d_43_53_63_73 = _mm256_sub_pd( d_43_53_63_73, c_70_71_72_73 );
				}

			if(td==0) // t( t(AB) + C )
				{
				c_00_01_20_21 = _mm256_unpacklo_pd( d_00_10_20_30, d_01_11_21_31 );
				c_10_11_30_31 = _mm256_unpackhi_pd( d_00_10_20_30, d_01_11_21_31 );
				c_02_03_22_23 = _mm256_unpacklo_pd( d_02_12_22_32, d_03_13_23_33 );
				c_12_13_32_33 = _mm256_unpackhi_pd( d_02_12_22_32, d_03_13_23_33 );

				c_00_01_02_03 = _mm256_permute2f128_pd( c_00_01_20_21, c_02_03_22_23, 0x20 );
				c_20_21_22_23 = _mm256_permute2f128_pd( c_00_01_20_21, c_02_03_22_23, 0x31 );
				c_10_11_12_13 = _mm256_permute2f128_pd( c_10_11_30_31, c_12_13_32_33, 0x20 );
				c_30_31_32_33 = _mm256_permute2f128_pd( c_10_11_30_31, c_12_13_32_33, 0x31 );

				_mm256_store_pd( &D0[0+ldc*0], c_00_01_02_03 );
				_mm256_store_pd( &D0[0+ldc*1], c_10_11_12_13 );
				_mm256_store_pd( &D0[0+ldc*2], c_20_21_22_23 );
				_mm256_store_pd( &D0[0+ldc*3], c_30_31_32_33 );

				c_40_41_60_61 = _mm256_unpacklo_pd( d_40_50_60_70, d_41_51_61_71 );
				c_50_51_70_71 = _mm256_unpackhi_pd( d_40_50_60_70, d_41_51_61_71 );
				c_42_43_62_63 = _mm256_unpacklo_pd( d_42_52_62_72, d_43_53_63_73 );
				c_52_53_72_73 = _mm256_unpackhi_pd( d_42_52_62_72, d_43_53_63_73 );

				c_40_41_42_43 = _mm256_permute2f128_pd( c_40_41_60_61, c_42_43_62_63, 0x20 );
				c_60_61_62_63 = _mm256_permute2f128_pd( c_40_41_60_61, c_42_43_62_63, 0x31 );
				c_50_51_52_53 = _mm256_permute2f128_pd( c_50_51_70_71, c_52_53_72_73, 0x20 );
				c_70_71_72_73 = _mm256_permute2f128_pd( c_50_51_70_71, c_52_53_72_73, 0x31 );

				_mm256_store_pd( &D1[0+ldc*0], c_40_41_42_43 );
				_mm256_store_pd( &D1[0+ldc*1], c_50_51_52_53 );
				_mm256_store_pd( &D1[0+ldc*2], c_60_61_62_63 );
				_mm256_store_pd( &D1[0+ldc*3], c_70_71_72_73 );
				}
			else // t(AB) + C
				{
				_mm256_store_pd( &D0[0+ldc*0], d_00_10_20_30 );
				_mm256_store_pd( &D0[0+ldc*1], d_01_11_21_31 );
				_mm256_store_pd( &D0[0+ldc*2], d_02_12_22_32 );
				_mm256_store_pd( &D0[0+ldc*3], d_03_13_23_33 );

				_mm256_store_pd( &D0[0+ldc*4], d_40_50_60_70 );
				_mm256_store_pd( &D0[0+ldc*5], d_41_51_61_71 );
				_mm256_store_pd( &D0[0+ldc*6], d_42_52_62_72 );
				_mm256_store_pd( &D0[0+ldc*7], d_43_53_63_73 );
				}

			}

		}

	}
#endif



void kernel_dgemm_nt_8x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs  = 4;
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	if(alg==0)
		tc = td;

	int low_rank = 0;
	if(tc==0)
		low_rank = LOW_RANK_N;
	else
		low_rank = LOW_RANK_T;

	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		temp,
		d_00, d_01, d_02, d_03,
		d_40, d_41, d_42, d_43,
		ab_tmp0, ab_tmp1, // temporary results
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	__m256i 
		mask_m, mask_n;

	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add_n; // it is the same for add_t

	// low-rank update
	if(kmax<=low_rank)
		{

		if(tc==0)
			{

			k = 0;

			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_4567 = _mm256_load_pd( &A1[0+bs*0] );
			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );

#if (LOW_RANK_N>=4)
#if (LOW_RANK_N<8)
			if(k<kmax-3)
#else
			for(; k<kmax-3; )
#endif
				{

				A_0123 = _mm256_load_pd( &A0[0+bs*1] );
				A_4567 = _mm256_load_pd( &A1[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &A0[0+bs*2] );
				a_4567 = _mm256_load_pd( &A1[0+bs*2] );

				b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );


				A_0123 = _mm256_load_pd( &A0[0+bs*3] );
				A_4567 = _mm256_load_pd( &A1[0+bs*3] );

				b_1    = _mm256_broadcast_sd( &B[1+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &A0[0+bs*4] );
				a_4567 = _mm256_load_pd( &A1[0+bs*4] );

				b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*4] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );


				A0 += 16;
				A1 += 16;
				B  += 16;
				k  += 4;

				}
#endif
			if(k<kmax-1)
				{

				A_0123 = _mm256_load_pd( &A0[0+bs*1] );
				A_4567 = _mm256_load_pd( &A1[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &A0[0+bs*2] );
				a_4567 = _mm256_load_pd( &A1[0+bs*2] );

				b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( A_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( A_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );


				A0 += 8;
				A1 += 8;
				B  += 8;
				k  += 2;

				}
			if(k<kmax)
				{

	//			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
	//			a_4567 = _mm256_load_pd( &A1[0+bs*0] );

				b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_1   = _mm256_add_pd( d_1, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				temp   = _mm256_mul_pd( a_4567, b_0 );
				d_6   = _mm256_add_pd( d_6, temp );

	//			b_0    = _mm256_broadcast_sd( &B[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_3   = _mm256_add_pd( d_3, temp );
				temp   = _mm256_mul_pd( a_4567, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

		//		A += 4;
		//		B += 4;
		//		k += 1;

				}

			goto add_n;

			}
		else
			{

			k = 0;

			a_0123 = _mm256_load_pd( &B[0+bs*0] );
			b_0    = _mm256_broadcast_sd( &A0[0+bs*0] );

#if (LOW_RANK_T>=4)
#if (LOW_RANK_T<8)
			if(k<kmax-3)
#else
			for(; k<kmax-3; )
#endif
				{

				A_0123 = _mm256_load_pd( &B[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &B[0+bs*2] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				A_0123 = _mm256_load_pd( &B[0+bs*3] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*2] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*3] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &B[0+bs*4] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*3] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*4] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				A0 += 16;
				A1 += 16;
				B  += 16;
				k  += 4;

				}
#endif
			if(k<kmax-1)
				{

				A_0123 = _mm256_load_pd( &B[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
				

				a_0123 = _mm256_load_pd( &B[0+bs*2] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*1] );
				temp   = _mm256_mul_pd( A_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
				b_0    = _mm256_broadcast_sd( &A0[0+bs*2] );
				temp   = _mm256_mul_pd( A_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
	

				A0 += 8;
				A1 += 8;
				B  += 8;
				k  += 2;

				}
			if(k<kmax)
				{

//				A_0123 = _mm256_load_pd( &B[0+bs*1] );

				b_1    = _mm256_broadcast_sd( &A1[0+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_0   = _mm256_add_pd( d_0, temp );
				b_0    = _mm256_broadcast_sd( &A0[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_4   = _mm256_add_pd( d_4, temp );

				b_1    = _mm256_broadcast_sd( &A1[1+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_1   = _mm256_add_pd( d_1, temp );
				b_0    = _mm256_broadcast_sd( &A0[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_5   = _mm256_add_pd( d_5, temp );

				b_1    = _mm256_broadcast_sd( &A1[2+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_2   = _mm256_add_pd( d_2, temp );
				b_0    = _mm256_broadcast_sd( &A0[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_6   = _mm256_add_pd( d_6, temp );

				b_1    = _mm256_broadcast_sd( &A1[3+bs*0] );
				temp   = _mm256_mul_pd( a_0123, b_0 );
				d_3   = _mm256_add_pd( d_3, temp );
//				b_0    = _mm256_broadcast_sd( &A0[0+bs*1] );
				temp   = _mm256_mul_pd( a_0123, b_1 );
				d_7   = _mm256_add_pd( d_7, temp );
	
		//		A += 4;
		//		B += 4;
		//		k += 1;

				}

			goto add_t;

			}
		}
	else // high-rank update
		{

		// prefetch
		a_0123 = _mm256_load_pd( &A0[0] );
		a_4567 = _mm256_load_pd( &A1[0] );
		b_0123 = _mm256_load_pd( &B[0] );

		for(k=0; k<kmax-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &A1[4] ); // prefetch
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
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &A1[8] ); // prefetch
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


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[12] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[12] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &A1[12] ); // prefetch
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


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &A1[16] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[16] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
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
			
			A0 += 16;
			A1 += 16;
			B  += 16;

			}
		
		if(kmax%4>=2)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &A1[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &A1[8] ); // prefetch
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
			
			
			A0 += 8;
			A1 += 8;
			B  += 8;

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

		}

	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;
	

	if(tc!=0)
		{
		goto end_t;
		}


	// tc==0
	end_n:

	// AB + C
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

	add_n:

	if(alg==0)
		{
		goto store_n;
		}
	else if(alg==1) // AB = A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );
	
		d_0 = _mm256_add_pd( c_0, d_0 );
		d_1 = _mm256_add_pd( c_1, d_1 );
		d_2 = _mm256_add_pd( c_2, d_2 );
		d_3 = _mm256_add_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C1[0+ldc*0] );
		c_5 = _mm256_load_pd( &C1[0+ldc*1] );
		c_6 = _mm256_load_pd( &C1[0+ldc*2] );
		c_7 = _mm256_load_pd( &C1[0+ldc*3] );
	
		d_4 = _mm256_add_pd( c_4, d_4 );
		d_5 = _mm256_add_pd( c_5, d_5 );
		d_6 = _mm256_add_pd( c_6, d_6 );
		d_7 = _mm256_add_pd( c_7, d_7 );
		}
	else // AB = - A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );
	
		d_0 = _mm256_sub_pd( c_0, d_0 );
		d_1 = _mm256_sub_pd( c_1, d_1 );
		d_2 = _mm256_sub_pd( c_2, d_2 );
		d_3 = _mm256_sub_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C1[0+ldc*0] );
		c_5 = _mm256_load_pd( &C1[0+ldc*1] );
		c_6 = _mm256_load_pd( &C1[0+ldc*2] );
		c_7 = _mm256_load_pd( &C1[0+ldc*3] );
	
		d_4 = _mm256_sub_pd( c_4, d_4 );
		d_5 = _mm256_sub_pd( c_5, d_5 );
		d_6 = _mm256_sub_pd( c_6, d_6 );
		d_7 = _mm256_sub_pd( c_7, d_7 );
		}

	if(td==0) // AB + C
		{
		goto store_n;
		}
	else
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


	// tc==1
	end_t:

	t_0 = _mm256_unpacklo_pd( d_0, d_1 );
	t_1 = _mm256_unpackhi_pd( d_1, d_0 );
	t_2 = _mm256_unpacklo_pd( d_2, d_3 );
	t_3 = _mm256_unpackhi_pd( d_3, d_2 );

	d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
	d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
	d_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
	d_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

	t_0 = _mm256_unpacklo_pd( d_4, d_5 );
	t_1 = _mm256_unpackhi_pd( d_5, d_4 );
	t_2 = _mm256_unpacklo_pd( d_6, d_7 );
	t_3 = _mm256_unpackhi_pd( d_7, d_6 );

	d_4 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
	d_5 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
	d_6 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
	d_7 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

	add_t:

	if(alg==0)
		{
		goto store_t;
		}
	else if(alg==1) // AB = A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );

		d_0 = _mm256_add_pd( c_0, d_0 );
		d_1 = _mm256_add_pd( c_1, d_1 );
		d_2 = _mm256_add_pd( c_2, d_2 );
		d_3 = _mm256_add_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C0[0+ldc*4] );
		c_5 = _mm256_load_pd( &C0[0+ldc*5] );
		c_6 = _mm256_load_pd( &C0[0+ldc*6] );
		c_7 = _mm256_load_pd( &C0[0+ldc*7] );

		d_4 = _mm256_add_pd( c_4, d_4 );
		d_5 = _mm256_add_pd( c_5, d_5 );
		d_6 = _mm256_add_pd( c_6, d_6 );
		d_7 = _mm256_add_pd( c_7, d_7 );
		}
	else // AB = - A*B'
		{
		c_0 = _mm256_load_pd( &C0[0+ldc*0] );
		c_1 = _mm256_load_pd( &C0[0+ldc*1] );
		c_2 = _mm256_load_pd( &C0[0+ldc*2] );
		c_3 = _mm256_load_pd( &C0[0+ldc*3] );

		d_0 = _mm256_sub_pd( c_0, d_0 );
		d_1 = _mm256_sub_pd( c_1, d_1 );
		d_2 = _mm256_sub_pd( c_2, d_2 );
		d_3 = _mm256_sub_pd( c_3, d_3 );

		c_4 = _mm256_load_pd( &C0[0+ldc*4] );
		c_5 = _mm256_load_pd( &C0[0+ldc*5] );
		c_6 = _mm256_load_pd( &C0[0+ldc*6] );
		c_7 = _mm256_load_pd( &C0[0+ldc*7] );

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
	else
		{
		goto store_t;
		}

	
	// store (5 - 8) x (3 - 4)
	store_n:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D0[0+ldc*2], d_2 );

	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, d_4 );
	_mm256_maskstore_pd( &D1[0+ldc*1], mask_m, d_5 );
	_mm256_maskstore_pd( &D1[0+ldc*2], mask_m, d_6 );

	if(kn>=4)
		{
		_mm256_store_pd( &D0[0+ldc*3], d_3 );
		_mm256_maskstore_pd( &D1[0+ldc*3], mask_m, d_7 );
		}

	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );
	
	_mm256_maskstore_pd( &D0[0+ldc*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+ldc*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+ldc*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+ldc*3], mask_n, d_3 );

	if(km>=8)
		{
		_mm256_maskstore_pd( &D0[0+ldc*4], mask_n, d_4 );
		_mm256_maskstore_pd( &D0[0+ldc*5], mask_n, d_5 );
		_mm256_maskstore_pd( &D0[0+ldc*6], mask_n, d_6 );
		_mm256_maskstore_pd( &D0[0+ldc*7], mask_n, d_7 );
		}
	else
		{
		_mm256_maskstore_pd( &D0[0+ldc*4], mask_n, d_4 );
		if(km>=6)
			{
			_mm256_maskstore_pd( &D0[0+ldc*5], mask_n, d_5 );
			if(km>6)
				{
				_mm256_maskstore_pd( &D0[0+ldc*6], mask_n, d_6 );
				}
			}
		}

	return;

	}



#if 1
void kernel_dgemm_nt_6x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_4545, A_0123, A_4545,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31,
		c_40_51_42_53, c_41_50_43_52;
	
	__m128d
		u_4, u_5, u_6, u_7,
		v_4, v_5, v_6, v_7;

	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		t_0, t_1, t_2, t_3;

	__m256i mask_n;

	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();
	c_40_51_42_53 = _mm256_setzero_pd();
	c_41_50_43_52 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4545 = _mm256_broadcast_pd( (__m128d *) &A1[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4) // TODO prefetch A0 and A1 using 2 extra registers ????????????????
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_40_51_42_53 = _mm256_add_pd( c_40_51_42_53, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4545        = _mm256_broadcast_pd( (__m128d *) &A1[4] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_1032 );
		c_41_50_43_52 = _mm256_add_pd( c_41_50_43_52, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( A_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4545, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_40_51_42_53 = _mm256_add_pd( c_40_51_42_53, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4545        = _mm256_broadcast_pd( (__m128d *) &A1[8] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4545, b_1032 );
		c_41_50_43_52 = _mm256_add_pd( c_41_50_43_52, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );


/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_0123 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_40_51_42_53 = _mm256_add_pd( c_40_51_42_53, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4545        = _mm256_broadcast_pd( (__m128d *) &A1[12] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_1032 );
		c_41_50_43_52 = _mm256_add_pd( c_41_50_43_52, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );


/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( A_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4545, b_0123 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_40_51_42_53 = _mm256_add_pd( c_40_51_42_53, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4545        = _mm256_broadcast_pd( (__m128d *) &A1[16] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4545, b_1032 );
		c_41_50_43_52 = _mm256_add_pd( c_41_50_43_52, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		A_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_40_51_42_53 = _mm256_add_pd( c_40_51_42_53, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		A_4545        = _mm256_broadcast_pd( (__m128d *) &A1[4] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_1032 );
		c_41_50_43_52 = _mm256_add_pd( c_41_50_43_52, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		
		ab_tmp0       = _mm256_mul_pd( A_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4545, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_40_51_42_53 = _mm256_add_pd( c_40_51_42_53, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_4545        = _mm256_broadcast_pd( (__m128d *) &A1[8] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_4545, b_1032 );
		c_41_50_43_52 = _mm256_add_pd( c_41_50_43_52, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( A_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		//A_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_0123 );
		//b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_40_51_42_53 = _mm256_add_pd( c_40_51_42_53, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		//A_4545        = _mm256_broadcast_pd( (__m128d *) &A1[4] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4545, b_1032 );
		c_41_50_43_52 = _mm256_add_pd( c_41_50_43_52, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_1032        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		}

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			t_0 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			t_1 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			t_2 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			t_3 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );

			d_0 = _mm256_blend_pd( t_0, t_2, 0xc );
			d_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
			d_1 = _mm256_blend_pd( t_1, t_3, 0xc );
			d_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

			t_0 = _mm256_blend_pd( c_40_51_42_53, c_41_50_43_52, 0xa );
			t_1 = _mm256_blend_pd( c_40_51_42_53, c_41_50_43_52, 0x5 );
			
			v_6 = _mm256_extractf128_pd( t_0, 0x1 );
			v_4 = _mm256_castpd256_pd128( t_0 );
			v_7 = _mm256_extractf128_pd( t_1, 0x1 );
			v_5 = _mm256_castpd256_pd128( t_1 );

			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_unpacklo_pd( c_00_11_22_33, c_01_10_23_32 );
			t_1 = _mm256_unpackhi_pd( c_01_10_23_32, c_00_11_22_33 );
			t_2 = _mm256_unpacklo_pd( c_02_13_20_31, c_03_12_21_30 );
			t_3 = _mm256_unpackhi_pd( c_03_12_21_30, c_02_13_20_31 );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			d_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			d_4 = _mm256_unpacklo_pd( c_40_51_42_53, c_41_50_43_52 );
			d_5 = _mm256_unpackhi_pd( c_41_50_43_52, c_40_51_42_53 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			t_0 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			t_1 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			t_2 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			t_3 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );

			c_0 = _mm256_blend_pd( t_0, t_2, 0xc );
			c_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
			c_1 = _mm256_blend_pd( t_1, t_3, 0xc );
			c_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

			t_0 = _mm256_blend_pd( c_40_51_42_53, c_41_50_43_52, 0xa );
			t_1 = _mm256_blend_pd( c_40_51_42_53, c_41_50_43_52, 0x5 );
			
			u_6 = _mm256_extractf128_pd( t_0, 0x1 );
			u_4 = _mm256_castpd256_pd128( t_0 );
			u_7 = _mm256_extractf128_pd( t_1, 0x1 );
			u_5 = _mm256_castpd256_pd128( t_1 );

			d_0 = _mm256_load_pd( &C0[0+ldc*0] );
			d_1 = _mm256_load_pd( &C0[0+ldc*1] );
			d_2 = _mm256_load_pd( &C0[0+ldc*2] );
			d_3 = _mm256_load_pd( &C0[0+ldc*3] );
			
			v_4 = _mm_load_pd( &C1[0+ldc*0] );
			v_5 = _mm_load_pd( &C1[0+ldc*1] );
			v_6 = _mm_load_pd( &C1[0+ldc*2] );
			v_7 = _mm_load_pd( &C1[0+ldc*3] );
			
			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				v_4 = _mm_add_pd( v_4, u_4 );
				v_5 = _mm_add_pd( v_5, u_5 );
				v_6 = _mm_add_pd( v_6, u_6 );
				v_7 = _mm_add_pd( v_7, u_7 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				v_4 = _mm_sub_pd( v_4, u_4 );
				v_5 = _mm_sub_pd( v_5, u_5 );
				v_6 = _mm_sub_pd( v_6, u_6 );
				v_7 = _mm_sub_pd( v_7, u_7 );
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else
				{
				t_0 = _mm256_unpacklo_pd( d_0, d_1 );
				t_1 = _mm256_unpackhi_pd( d_0, d_1 );
				t_2 = _mm256_unpacklo_pd( d_2, d_3 );
				t_3 = _mm256_unpackhi_pd( d_2, d_3 );

				d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
				d_2 = _mm256_permute2f128_pd( t_0, t_2, 0x31 );
				d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
				d_3 = _mm256_permute2f128_pd( t_1, t_3, 0x31 );

				t_0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( v_4 ), v_6, 0x1 );
				t_1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( v_5 ), v_7, 0x1 );

				d_4 = _mm256_unpacklo_pd( t_0, t_1 );
				d_5 = _mm256_unpackhi_pd( t_0, t_1 );

				goto store_t;
				}

			}
		else // t(C)
			{

			t_0 = _mm256_unpacklo_pd( c_00_11_22_33, c_01_10_23_32 );
			t_1 = _mm256_unpackhi_pd( c_01_10_23_32, c_00_11_22_33 );
			t_2 = _mm256_unpacklo_pd( c_02_13_20_31, c_03_12_21_30 );
			t_3 = _mm256_unpackhi_pd( c_03_12_21_30, c_02_13_20_31 );

			c_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			c_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			c_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			c_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			c_4 = _mm256_unpacklo_pd( c_40_51_42_53, c_41_50_43_52 );
			c_5 = _mm256_unpackhi_pd( c_41_50_43_52, c_40_51_42_53 );

			d_0 = _mm256_load_pd( &C0[0+ldc*0] );
			d_1 = _mm256_load_pd( &C0[0+ldc*1] );
			d_2 = _mm256_load_pd( &C0[0+ldc*2] );
			d_3 = _mm256_load_pd( &C0[0+ldc*3] );

			d_4 = _mm256_load_pd( &C0[0+ldc*4] );
			d_5 = _mm256_load_pd( &C0[0+ldc*5] );

			if(alg==1) // AB = A*B'
				{
				d_0 = _mm256_add_pd( d_0, c_0 );
				d_1 = _mm256_add_pd( d_1, c_1 );
				d_2 = _mm256_add_pd( d_2, c_2 );
				d_3 = _mm256_add_pd( d_3, c_3 );

				d_4 = _mm256_add_pd( d_4, c_4 );
				d_5 = _mm256_add_pd( d_5, c_5 );
				}
			else // AB = - A*B'
				{
				d_0 = _mm256_sub_pd( d_0, c_0 );
				d_1 = _mm256_sub_pd( d_1, c_1 );
				d_2 = _mm256_sub_pd( d_2, c_2 );
				d_3 = _mm256_sub_pd( d_3, c_3 );

				d_4 = _mm256_sub_pd( d_4, c_4 );
				d_5 = _mm256_sub_pd( d_5, c_5 );
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

				v_6 = _mm256_extractf128_pd( t_0, 0x1 );
				v_4 = _mm256_castpd256_pd128( t_0 );
				v_7 = _mm256_extractf128_pd( t_1, 0x1 );
				v_5 = _mm256_castpd256_pd128( t_1 );

				goto store_n;
				}
			else
				{
				goto store_t;
				}

			}

		}
#endif
	
	// store (5 - 6) x (3 - 4)
	store_n:
	if(km>=6)
		{
		_mm256_store_pd( &D0[0+ldc*0], d_0 );
		_mm256_store_pd( &D0[0+ldc*1], d_1 );
		_mm256_store_pd( &D0[0+ldc*2], d_2 );

		_mm_store_pd( &D1[0+ldc*0], v_4 );
		_mm_store_pd( &D1[0+ldc*1], v_5 );
		_mm_store_pd( &D1[0+ldc*2], v_6 );

		if(kn>=4)
			{
			_mm256_store_pd( &D0[0+ldc*3], d_3 );
			_mm_store_pd( &D1[0+ldc*3], v_7 );
			}
		}
	else
		{
		_mm256_store_pd( &D0[0+ldc*0], d_0 );
		_mm256_store_pd( &D0[0+ldc*1], d_1 );
		_mm256_store_pd( &D0[0+ldc*2], d_2 );

		_mm_store_sd( &D1[0+ldc*0], v_4 );
		_mm_store_sd( &D1[0+ldc*1], v_5 );
		_mm_store_sd( &D1[0+ldc*2], v_6 );

		if(kn>=4)
			{
			_mm256_store_pd( &D0[0+ldc*3], d_3 );
			_mm_store_sd( &D1[0+ldc*3], v_7 );
			}
		}

	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );
	
	_mm256_maskstore_pd( &D0[0+ldc*0], mask_n, d_0 );
	_mm256_maskstore_pd( &D0[0+ldc*1], mask_n, d_1 );
	_mm256_maskstore_pd( &D0[0+ldc*2], mask_n, d_2 );
	_mm256_maskstore_pd( &D0[0+ldc*3], mask_n, d_3 );
	_mm256_maskstore_pd( &D0[0+ldc*4], mask_n, d_4 );

	if(km>=6)
		{
		_mm256_maskstore_pd( &D0[0+ldc*5], mask_n, d_5 );
		}

	return;

	}



void kernel_dgemm_nt_8x2_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256i
		mask_m;

	__m256d
		a_0123, a_4567, //A_0123,
		b_0101, b_1010,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_20_31, c_01_10_21_30,
		c_40_51_60_71, c_41_50_61_70;
	
	__m128d
		u_0, u_1, u_2, u_3,
		u_4, u_5, u_6, u_7,
		v_0, v_1, v_2, v_3,
		v_4, v_5, v_6, v_7,
		d_00_10, d_01_11, d_02_12, d_03_13,
		d_04_14, d_05_15, d_06_16, d_07_17,
		c_00_01, c_10_11, c_20_21, c_30_31,
		c_40_41, c_50_51, c_60_61, c_70_71;

	__m256d
		t_0, t_1, t_4, t_5,
		c_0, c_1, c_4, c_5,
		d_0, d_1, d_4, d_5,
		c_00_01_20_21, c_10_11_30_31,
		c_40_41_60_61, c_50_51_70_71,
		c_00_10_20_30, c_01_11_21_31,
		c_40_50_60_70, c_41_51_61_71,
		d_00_10_20_30, d_01_11_21_31,
		d_40_50_60_70, d_41_51_61_71;

	// zero registers
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	c_40_51_60_71 = _mm256_setzero_pd();
	c_41_50_61_70 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	a_4567 = _mm256_load_pd( &A1[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp0 );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp0 );


/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp0 );


/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp0 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}
	
	if(kmax%4>=2)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp0 );
		
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0101 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1010 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp0 );
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_0101 );
/*		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch*/
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
/*		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch*/
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_4567, b_1010 );
/*		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch*/
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp0 );
		
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
			t_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			t_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );

			u_2 = _mm256_extractf128_pd( t_0, 0x1 );
			u_0 = _mm256_castpd256_pd128( t_0 );
			u_3 = _mm256_extractf128_pd( t_1, 0x1 );
			u_1 = _mm256_castpd256_pd128( t_1 );

			t_0 = _mm256_unpacklo_pd( c_40_51_60_71, c_41_50_61_70 );
			t_1 = _mm256_unpackhi_pd( c_41_50_61_70, c_40_51_60_71 );

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
			c_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
			c_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
			c_4 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0xa );
			c_5 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0x5 );

			d_0 = _mm256_load_pd( &C0[0+ldc*0] );
			d_1 = _mm256_load_pd( &C0[0+ldc*1] );
			d_4 = _mm256_load_pd( &C1[0+ldc*0] );
			d_5 = _mm256_load_pd( &C1[0+ldc*1] );
		
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
			else
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
			t_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			t_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );
			t_4 = _mm256_unpacklo_pd( c_40_51_60_71, c_41_50_61_70 );
			t_5 = _mm256_unpackhi_pd( c_41_50_61_70, c_40_51_60_71 );

			v_2 = _mm256_extractf128_pd( t_0, 0x1 );
			v_0 = _mm256_castpd256_pd128( t_0 );
			v_3 = _mm256_extractf128_pd( t_1, 0x1 );
			v_1 = _mm256_castpd256_pd128( t_1 );
			v_6 = _mm256_extractf128_pd( t_4, 0x1 );
			v_4 = _mm256_castpd256_pd128( t_4 );
			v_7 = _mm256_extractf128_pd( t_5, 0x1 );
			v_5 = _mm256_castpd256_pd128( t_5 );

			u_0 = _mm_load_pd( &C0[0+ldc*0] );
			u_1 = _mm_load_pd( &C0[0+ldc*1] );
			u_2 = _mm_load_pd( &C0[0+ldc*2] );
			u_3 = _mm_load_pd( &C0[0+ldc*3] );
			u_4 = _mm_load_pd( &C0[0+ldc*4] );
			u_5 = _mm_load_pd( &C0[0+ldc*5] );
			u_6 = _mm_load_pd( &C0[0+ldc*6] );
			u_7 = _mm_load_pd( &C0[0+ldc*7] );

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
				t_0 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_0 ), u_2, 0x1 );
				t_1 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_1 ), u_3, 0x1 );
				t_4 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_4 ), u_6, 0x1 );
				t_5 = _mm256_insertf128_pd( _mm256_castpd128_pd256( u_5 ), u_7, 0x1 );

				d_0 = _mm256_unpacklo_pd( t_0, t_1 );
				d_1 = _mm256_unpackhi_pd( t_0, t_1 );
				d_4 = _mm256_unpacklo_pd( t_4, t_5 );
				d_5 = _mm256_unpackhi_pd( t_4, t_5 );
				
				goto store_n;
				}
			else
				{
				goto store_t;
				}

			}
		}

	// store (5 - 8) x (1 - 2)
	store_n:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, d_4 );

	if(kn>=2)
		{
		_mm256_store_pd( &D0[0+ldc*1], d_1 );
		_mm256_maskstore_pd( &D1[0+ldc*1], mask_m, d_5 );
		}
	return;

	store_t:
	if(kn>=2)
		{
		_mm_store_pd( &D0[0+ldc*0], u_0 );
		_mm_store_pd( &D0[0+ldc*1], u_1 );
		_mm_store_pd( &D0[0+ldc*2], u_2 );
		_mm_store_pd( &D0[0+ldc*3], u_3 );

		if(km>=8)
			{
			_mm_store_pd( &D0[0+ldc*4], u_4 );
			_mm_store_pd( &D0[0+ldc*5], u_5 );
			_mm_store_pd( &D0[0+ldc*6], u_6 );
			_mm_store_pd( &D0[0+ldc*7], u_7 );
			}
		else
			{
			_mm_store_pd( &D0[0+ldc*4], u_4 );
			if(km>=6)
				{
				_mm_store_pd( &D0[0+ldc*5], u_5 );
				if(km>6)
					{
					_mm_store_pd( &D0[0+ldc*6], u_6 );
					}
				}
			}
		}
	else
		{
		_mm_store_sd( &D0[0+ldc*0], u_0 );
		_mm_store_sd( &D0[0+ldc*1], u_1 );
		_mm_store_sd( &D0[0+ldc*2], u_2 );
		_mm_store_sd( &D0[0+ldc*3], u_3 );

		if(km>=8)
			{
			_mm_store_sd( &D0[0+ldc*4], u_4 );
			_mm_store_sd( &D0[0+ldc*5], u_5 );
			_mm_store_sd( &D0[0+ldc*6], u_6 );
			_mm_store_sd( &D0[0+ldc*7], u_7 );
			}
		else
			{
			_mm_store_sd( &D0[0+ldc*4], u_4 );
			if(km>=6)
				{
				_mm_store_sd( &D0[0+ldc*5], u_5 );
				if(km>6)
					{
					_mm_store_sd( &D0[0+ldc*6], u_6 );
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
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_2323,
		b_0123, b_1032, b_3210, b_2301,
		ab_temp, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31;
	
	__m256d
		c_0, c_1, c_2, c_3,
		d_0, d_1, d_2, d_3,
		t_0, t_1, t_2, t_3;

	__m256i 
		mask_m, mask_n;

	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0123 = _mm256_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
#if 1
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
#else

// test to compute the lower triangular using 3 fmadd

/*	__builtin_prefetch( A+32 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		a_2323        = _mm256_permute2f128_pd( a_0123, a_0123, 0x9 );
		//a_2323        = _mm256_broadcast_pd( (__m128d *) &A[2] );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_0123, b_1032, 0x0 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_2323, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		a_2323        = _mm256_permute2f128_pd( a_0123, a_0123, 0x9 );
		//a_2323        = _mm256_broadcast_pd( (__m128d *) &A[6] );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_0123, b_1032, 0x0 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_2323, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );


/*	__builtin_prefetch( A+48 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		a_2323        = _mm256_permute2f128_pd( a_0123, a_0123, 0x9 );
		//a_2323        = _mm256_broadcast_pd( (__m128d *) &A[10] );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_0123, b_1032, 0x0 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_2323, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );


/*	__builtin_prefetch( A+56 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		a_2323        = _mm256_permute2f128_pd( a_0123, a_0123, 0x9 );
		//a_2323        = _mm256_broadcast_pd( (__m128d *) &A[14] );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_0123, b_1032, 0x0 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_2323, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );

#endif
		
		A += 16;
		B += 16;

		}
	
	if(kmax%4>=2)
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

	if(kmax%2==1)
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
		
		}

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			t_0 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			t_1 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			t_2 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			t_3 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
			
			d_0 = _mm256_blend_pd( t_0, t_2, 0xc );
			d_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
			d_1 = _mm256_blend_pd( t_1, t_3, 0xc );
			d_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

			goto store_n;
			}
		else // transposed
			{
			t_0 = _mm256_shuffle_pd( c_00_11_22_33, c_01_10_23_32, 0x0 );
			t_1 = _mm256_shuffle_pd( c_01_10_23_32, c_00_11_22_33, 0xf );
			t_2 = _mm256_shuffle_pd( c_02_13_20_31, c_03_12_21_30, 0x0 );
			t_3 = _mm256_shuffle_pd( c_03_12_21_30, c_02_13_20_31, 0xf );

			d_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			d_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			d_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			d_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{

			// AB + C
			t_0 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
			t_1 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
			t_2 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
			t_3 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
			
			c_0 = _mm256_blend_pd( t_0, t_2, 0xc );
			c_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
			c_1 = _mm256_blend_pd( t_1, t_3, 0xc );
			c_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

			d_0 = _mm256_load_pd( &C[0+ldc*0] );
			d_1 = _mm256_load_pd( &C[0+ldc*1] );
			d_2 = _mm256_load_pd( &C[0+ldc*2] );
			d_3 = _mm256_load_pd( &C[0+ldc*3] );
			
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

			t_0 = _mm256_shuffle_pd( c_00_11_22_33, c_01_10_23_32, 0x0 );
			t_1 = _mm256_shuffle_pd( c_01_10_23_32, c_00_11_22_33, 0xf );
			t_2 = _mm256_shuffle_pd( c_02_13_20_31, c_03_12_21_30, 0x0 );
			t_3 = _mm256_shuffle_pd( c_03_12_21_30, c_02_13_20_31, 0xf );

			c_0 = _mm256_permute2f128_pd( t_0, t_2, 0x20 );
			c_1 = _mm256_permute2f128_pd( t_1, t_3, 0x20 );
			c_2 = _mm256_permute2f128_pd( t_2, t_0, 0x31 );
			c_3 = _mm256_permute2f128_pd( t_3, t_1, 0x31 );

			d_0 = _mm256_load_pd( &C[0+ldc*0] );
			d_1 = _mm256_load_pd( &C[0+ldc*1] );
			d_2 = _mm256_load_pd( &C[0+ldc*2] );
			d_3 = _mm256_load_pd( &C[0+ldc*3] );

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
	_mm256_maskstore_pd( &D[0+ldc*0], mask_m, d_0 );
	_mm256_maskstore_pd( &D[0+ldc*1], mask_m, d_1 );
	_mm256_maskstore_pd( &D[0+ldc*2], mask_m, d_2 );

	if(kn>=4)
		{
		_mm256_maskstore_pd( &D[0+ldc*3], mask_m, d_3 );
		}
	return;

	store_t:
	if(kn==3)
		mask_n = _mm256_set_epi64x( 1, -1, -1, -1 );
	else // kn>=4
		mask_n = _mm256_set_epi64x( -1, -1, -1, -1 );

	if(km>=4)
		{
		_mm256_maskstore_pd( &D[0+ldc*0], mask_n, d_0 );
		_mm256_maskstore_pd( &D[0+ldc*1], mask_n, d_1 );
		_mm256_maskstore_pd( &D[0+ldc*2], mask_n, d_2 );
		_mm256_maskstore_pd( &D[0+ldc*3], mask_n, d_3 );
		}
	else
		{
		_mm256_maskstore_pd( &D[0+ldc*0], mask_n, d_0 );
		if(km>=2)
			{
			_mm256_maskstore_pd( &D[0+ldc*1], mask_n, d_1 );
			if(km>2)
				{
				_mm256_maskstore_pd( &D[0+ldc*2], mask_n, d_2 );
				}
			}
		}
	return;

	}



void kernel_dgemm_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{


	kernel_dgemm_nt_4x4_vs_lib4(4, 4, kmax, A, B, C, D, alg, tc, td);

	}



void kernel_dgemm_nt_4x3_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;

	int k;
	
	__m256d
		a_0123, A_0123,
		b_0, b_1, b_2,
		ab_temp, // temporary results
		c_00, c_01, c_02,
		d_00, d_01, d_02;
	
	// zero registers
	c_00 = _mm256_setzero_pd();
	c_01 = _mm256_setzero_pd();
	c_02 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0    = _mm256_broadcast_pd( (__m128d *) &B[0] );
	b_2    = _mm256_broadcast_sd( &B[2] );
	b_1    = _mm256_shuffle_pd( b_0, b_0, 0x5 );

	for(k=0; k<kmax-3; k+=4)
		{
		
		A_0123  = _mm256_load_pd( &A[4] ); // prefetch
		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		b_0     = _mm256_broadcast_pd( (__m128d *) &B[4] );
		c_00    = _mm256_add_pd( c_00, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_1 );
		b_1     = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01    = _mm256_add_pd( c_01, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_2 );
		b_2     = _mm256_broadcast_sd( &B[6] ); // prefetch
		c_02    = _mm256_add_pd( c_02, ab_temp );

		a_0123  = _mm256_load_pd( &A[8] ); // prefetch
		ab_temp = _mm256_mul_pd( A_0123, b_0 );
		b_0     = _mm256_broadcast_pd( (__m128d *) &B[8] );
		c_00    = _mm256_add_pd( c_00, ab_temp );
		ab_temp = _mm256_mul_pd( A_0123, b_1 );
		b_1     = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01    = _mm256_add_pd( c_01, ab_temp );
		ab_temp = _mm256_mul_pd( A_0123, b_2 );
		b_2     = _mm256_broadcast_sd( &B[10] ); // prefetch
		c_02    = _mm256_add_pd( c_02, ab_temp );

		A_0123  = _mm256_load_pd( &A[12] ); // prefetch
		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		b_0     = _mm256_broadcast_pd( (__m128d *) &B[12] );
		c_00    = _mm256_add_pd( c_00, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_1 );
		b_1     = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01    = _mm256_add_pd( c_01, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_2 );
		b_2     = _mm256_broadcast_sd( &B[14] ); // prefetch
		c_02    = _mm256_add_pd( c_02, ab_temp );

		a_0123  = _mm256_load_pd( &A[16] ); // prefetch
		ab_temp = _mm256_mul_pd( A_0123, b_0 );
		b_0     = _mm256_broadcast_pd( (__m128d *) &B[16] );
		c_00    = _mm256_add_pd( c_00, ab_temp );
		ab_temp = _mm256_mul_pd( A_0123, b_1 );
		b_1     = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01    = _mm256_add_pd( c_01, ab_temp );
		ab_temp = _mm256_mul_pd( A_0123, b_2 );
		b_2     = _mm256_broadcast_sd( &B[18] ); // prefetch
		c_02    = _mm256_add_pd( c_02, ab_temp );

		A += 16;
		B += 16;

		}
	
	if(kmax%4>=2)
		{
		
		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		b_0     = _mm256_broadcast_pd( (__m128d *) &B[4] );
		c_00    = _mm256_add_pd( c_00, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_1 );
		b_1     = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01    = _mm256_add_pd( c_01, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_2 );
		b_2     = _mm256_broadcast_sd( &B[6] ); // prefetch
		a_0123  = _mm256_load_pd( &A[4] ); // prefetch
		c_02    = _mm256_add_pd( c_02, ab_temp );

		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		b_0     = _mm256_broadcast_pd( (__m128d *) &B[8] );
		c_00    = _mm256_add_pd( c_00, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_1 );
		b_1     = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01    = _mm256_add_pd( c_01, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_2 );
		b_2     = _mm256_broadcast_sd( &B[10] ); // prefetch
		a_0123  = _mm256_load_pd( &A[8] ); // prefetch
		c_02    = _mm256_add_pd( c_02, ab_temp );

		A += 8;
		B += 8;

		}
	
	if(kmax%2==1)
		{
		
		ab_temp = _mm256_mul_pd( a_0123, b_0 );
//		b_0     = _mm256_broadcast_pd( (__m128d *) &B[4] );
		c_00    = _mm256_add_pd( c_00, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_1 );
//		b_1     = _mm256_shuffle_pd( b_0, b_0, 0x5 );
		c_01    = _mm256_add_pd( c_01, ab_temp );
		ab_temp = _mm256_mul_pd( a_0123, b_2 );
//		b_2     = _mm256_broadcast_sd( &B[6] ); // prefetch
//		a_0123  = _mm256_load_pd( &A[4] ); // prefetch
		c_02    = _mm256_add_pd( c_02, ab_temp );


		}

	add:

	d_00 = _mm256_blend_pd( c_00, c_01, 0xa );
	d_01 = _mm256_blend_pd( c_00, c_01, 0x5 );
	d_02 = c_02;

	if(alg==0) // D = A * B' , there is no tc
		{
		goto store_n;
		}
	else 
		{
		d_00 = _mm256_load_pd( &C[0+ldc*0] );
		d_01 = _mm256_load_pd( &C[0+ldc*1] );
		d_02 = _mm256_load_pd( &C[0+ldc*2] );
	
		if(alg==1) // AB = A * B'
			{
			d_00 = _mm256_add_pd( d_00, c_00 );
			d_01 = _mm256_add_pd( d_01, c_01 );
			d_02 = _mm256_add_pd( d_02, c_02 );
			}
		else // AB = - A * B'
			{
			d_00 = _mm256_sub_pd( d_00, c_00 );
			d_01 = _mm256_sub_pd( d_01, c_01 );
			d_02 = _mm256_sub_pd( d_02, c_02 );
			}

			goto store_n;
		}

	store_n:
	_mm256_store_pd( &D[0+ldc*0], d_00 );
	_mm256_store_pd( &D[0+ldc*1], d_01 );
	_mm256_store_pd( &D[0+ldc*2], d_02 );


	}



// normal-transposed, 4x2 with data packed in 4
void kernel_dgemm_nt_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123,
		b_0101, b_1010,
		ab_temp, // temporary results
		c_00_11_20_31, c_01_10_21_30, C_00_11_20_31, C_01_10_21_30;
	
	__m128d
		u_0, u_1, u_2, u_3,
		v_0, v_1, v_2, v_3;

	__m256d
		c_0, c_1,
		d_0, d_1,
		t_0, t_1;
	

	__m256i
		mask_m;
	
	// zero registers
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	C_00_11_20_31 = _mm256_setzero_pd();
	C_01_10_21_30 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	for(k=0; k<kmax-3; k+=4)
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
	
	if(kmax%4>=2)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );
		
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		C_00_11_20_31 = _mm256_add_pd( C_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		C_01_10_21_30 = _mm256_add_pd( C_01_10_21_30, ab_temp );
		
		
		A += 8;
		B += 8;

		}
	
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, C_00_11_20_31 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, C_01_10_21_30 );

	if(kmax%2==1)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
/*		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch*/
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
/*		a_0123        = _mm256_load_pd( &A[4] ); // prefetch*/
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );
		
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
			t_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			t_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );

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
			c_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
			c_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );

			d_0 = _mm256_load_pd( &C[0+ldc*0] );
			d_1 = _mm256_load_pd( &C[0+ldc*1] );
		
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
			t_0 = _mm256_unpacklo_pd( c_00_11_20_31, c_01_10_21_30 );
			t_1 = _mm256_unpackhi_pd( c_01_10_21_30, c_00_11_20_31 );
				
			v_2 = _mm256_extractf128_pd( t_0, 0x1 );
			v_0 = _mm256_castpd256_pd128( t_0 );
			v_3 = _mm256_extractf128_pd( t_1, 0x1 );
			v_1 = _mm256_castpd256_pd128( t_1 );

			u_0 = _mm_load_pd( &C[0+ldc*0] );
			u_1 = _mm_load_pd( &C[0+ldc*1] );
			u_2 = _mm_load_pd( &C[0+ldc*2] );
			u_3 = _mm_load_pd( &C[0+ldc*3] );
		
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
	_mm256_maskstore_pd( &D[0+ldc*0], mask_m, d_0 );

	if(kn>=2)
		{
		_mm256_maskstore_pd( &D[0+ldc*1], mask_m, d_1 );
		}
	return;

	store_t:
	if(kn>=2)
		{
		if(km>=4)
			{
			_mm_store_pd( &D[0+ldc*0], u_0 );
			_mm_store_pd( &D[0+ldc*1], u_1 );
			_mm_store_pd( &D[0+ldc*2], u_2 );
			_mm_store_pd( &D[0+ldc*3], u_3 );
			}
		else
			{
			_mm_store_pd( &D[0+ldc*0], u_0 );
			if(km>=2)
				{
				_mm_store_pd( &D[0+ldc*1], u_1 );
				if(km>2)
					{
					_mm_store_pd( &D[0+ldc*2], u_2 );
					}
				}
			}
		}
	else
		{
		if(km>=4)
			{
			_mm_store_sd( &D[0+ldc*0], u_0 );
			_mm_store_sd( &D[0+ldc*1], u_1 );
			_mm_store_sd( &D[0+ldc*2], u_2 );
			_mm_store_sd( &D[0+ldc*3], u_3 );
			}
		else
			{
			_mm_store_sd( &D[0+ldc*0], u_0 );
			if(km>=2)
				{
				_mm_store_sd( &D[0+ldc*1], u_1 );
				if(km>2)
					{
					_mm_store_sd( &D[0+ldc*2], u_2 );
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



void kernel_dgemm_nt_4x1_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;

	int k;
	
	__m256d
		a_0123,
		b_0,
		ab_temp, // temporary results
		c_00, C_00, D_00, d_00;
	
	// zero registers
	c_00 = _mm256_setzero_pd();
	C_00 = _mm256_setzero_pd();
	d_00 = _mm256_setzero_pd();
	D_00 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123 = _mm256_load_pd( &A[0] );
	b_0    = _mm256_broadcast_sd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		a_0123  = _mm256_load_pd( &A[4] ); // prefetch
		b_0     = _mm256_broadcast_sd( &B[4] ); // prefetch
		c_00    = _mm256_add_pd( c_00, ab_temp );

		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		a_0123  = _mm256_load_pd( &A[8] ); // prefetch
		b_0     = _mm256_broadcast_sd( &B[8] ); // prefetch
		C_00    = _mm256_add_pd( C_00, ab_temp );

		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		a_0123  = _mm256_load_pd( &A[12] ); // prefetch
		b_0     = _mm256_broadcast_sd( &B[12] ); // prefetch
		d_00    = _mm256_add_pd( d_00, ab_temp );

		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		a_0123  = _mm256_load_pd( &A[16] ); // prefetch
		b_0     = _mm256_broadcast_sd( &B[16] ); // prefetch
		D_00    = _mm256_add_pd( D_00, ab_temp );
		
		A += 16;
		B += 16;

		}
	
	c_00    = _mm256_add_pd( c_00, C_00 );
	d_00    = _mm256_add_pd( d_00, D_00 );
		
	if(kmax%4>=2)
		{
		
		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		a_0123  = _mm256_load_pd( &A[4] ); // prefetch
		b_0     = _mm256_broadcast_sd( &B[4] ); // prefetch
		c_00    = _mm256_add_pd( c_00, ab_temp );

		ab_temp = _mm256_mul_pd( a_0123, b_0 );
		a_0123  = _mm256_load_pd( &A[8] ); // prefetch
		b_0     = _mm256_broadcast_sd( &B[8] ); // prefetch
		d_00    = _mm256_add_pd( d_00, ab_temp );

		A += 8;
		B += 8;

		}
	
	c_00    = _mm256_add_pd( c_00, d_00 );

	if(kmax%2==1)
		{
		
		ab_temp = _mm256_mul_pd( a_0123, b_0 );
//		a_0123  = _mm256_load_pd( &A[4] ); // prefetch
//		b_0     = _mm256_broadcast_sd( &B[4] ); // prefetch
		c_00    = _mm256_add_pd( c_00, ab_temp );
		
		}

	add:

	if(alg==0) // D = A * B' , there is no tc
		{
		goto store_n;
		}
	else 
		{
		d_00 = _mm256_load_pd( &C[0+ldc*0] );
	
		if(alg==1) // AB = A * B'
			{
			c_00 = _mm256_add_pd( d_00, c_00 );
			}
		else // AB = - A * B'
			{
			c_00 = _mm256_sub_pd( d_00, c_00 );
			}

			goto store_n;
		}

	store_n:
	_mm256_store_pd( &D[0+ldc*0], c_00 );


	}



// normal-transposed, 2x2 with data packed in 4
void kernel_dgemm_nt_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int ldc = 4;

	int k;
	
	__m128d
		a_01,
		b_01, b_10,
		ab_temp, // temporary results
		c_00_11, c_01_10, C_00_11, C_01_10;
	
	__m128d
		t_0,
		c_0, c_1,
		d_0, d_1,
		c_00_10, c_01_11,
		c_00_01, c_10_11,
		d_00_11, d_01_10,
		d_00_10, d_01_11,
		d_00_01, d_10_11;

	// zero registers
	c_00_11 = _mm_setzero_pd();
	c_01_10 = _mm_setzero_pd();
	C_00_11 = _mm_setzero_pd();
	C_01_10 = _mm_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_01 = _mm_load_pd( &A[0] );
	b_01 = _mm_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_temp = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[4] ); // prefetch
		ab_temp = _mm_mul_pd( a_01, b_10 );
		a_01    = _mm_load_pd( &A[4] ); // prefetch
		c_01_10 = _mm_add_pd( c_01_10, ab_temp );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_temp = _mm_mul_pd( a_01, b_01 );
		C_00_11 = _mm_add_pd( C_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[8] ); // prefetch
		ab_temp = _mm_mul_pd( a_01, b_10 );
		a_01    = _mm_load_pd( &A[8] ); // prefetch
		C_01_10 = _mm_add_pd( C_01_10, ab_temp );


/*	__builtin_prefetch( A+48 );*/
		ab_temp = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[12] ); // prefetch
		ab_temp = _mm_mul_pd( a_01, b_10 );
		a_01    = _mm_load_pd( &A[12] ); // prefetch
		c_01_10 = _mm_add_pd( c_01_10, ab_temp );


/*	__builtin_prefetch( A+56 );*/
		ab_temp = _mm_mul_pd( a_01, b_01 );
		C_00_11 = _mm_add_pd( C_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[16] ); // prefetch
		ab_temp = _mm_mul_pd( a_01, b_10 );
		a_01    = _mm_load_pd( &A[16] ); // prefetch
		C_01_10 = _mm_add_pd( C_01_10, ab_temp );
		
		A += 16;
		B += 16;

		}
	
	if(kmax%4>=2)
		{
		
		ab_temp = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[4] ); // prefetch
		ab_temp = _mm_mul_pd( a_01, b_10 );
		a_01    = _mm_load_pd( &A[4] ); // prefetch
		c_01_10 = _mm_add_pd( c_01_10, ab_temp );
		
		
		ab_temp = _mm_mul_pd( a_01, b_01 );
		C_00_11 = _mm_add_pd( C_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
		b_01    = _mm_load_pd( &B[8] ); // prefetch
		ab_temp = _mm_mul_pd( a_01, b_10 );
		a_01    = _mm_load_pd( &A[8] ); // prefetch
		C_01_10 = _mm_add_pd( C_01_10, ab_temp );
		
		
		A += 8;
		B += 8;

		}
	
	c_00_11 = _mm_add_pd( c_00_11, C_00_11 );
	c_01_10 = _mm_add_pd( c_01_10, C_01_10 );

	if(kmax%2==1)
		{
		
		ab_temp = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
/*		b_01    = _mm_load_pd( &B[4] ); // prefetch*/
		ab_temp = _mm_mul_pd( a_01, b_10 );
/*		a_01    = _mm_load_pd( &A[4] ); // prefetch*/
		c_01_10 = _mm_add_pd( c_01_10, ab_temp );
		
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

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
		
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

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );

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
		_mm_store_pd( &D[0+ldc*0], d_0 );
		if(kn>=2)
			_mm_store_pd( &D[0+ldc*1], d_1 );
		}
	else
		{
		_mm_store_sd( &D[0+ldc*0], d_0 );
		if(kn>=2)
			_mm_store_sd( &D[0+ldc*1], d_1 );
		}
	return;

	store_t:
	if(kn>=2)
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		if(km>=2)
			_mm_store_pd( &D[0+ldc*1], d_1 );
		}
	else
		{
		_mm_store_sd( &D[0+ldc*0], d_0 );
		if(km>=2)
			_mm_store_sd( &D[0+ldc*1], d_1 );
		}
	return;

	}



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
		a_0, a_4, A_0, A_4,
		b_0,
		B_0, B_1, B_2, B_3,
		tmp,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7;
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;
	
	int
		lft;
	
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

#if 1

	// prefetch
	B_0 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*0] );
	B_1 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*1] );
	B_2 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*2] );
	B_3 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*3] );

	for(k=0; k<kmax-4; k+=4)
//	for(k=0; k<kmax-7; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		__builtin_prefetch( B+2*B_next+0 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		__builtin_prefetch( B+2*B_next+8 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		d_3 = _mm256_add_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );
		B  += B_next;


		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );
		A0 += 4*bs;


		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		A1 += 4*bs;
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );

		}
	if(k==kmax-4)
//	if(k>kmax-3)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		A0 += 4*bs;
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		A1 += 4*bs;
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		k += 4;
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );

		}
	else
		{
		for( ; k<kmax; k++)
			{

			a_0 = _mm256_load_pd( &A0[0+bs*0] );
			a_4 = _mm256_load_pd( &A1[0+bs*0] );
			b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_add_pd( d_0, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_4 = _mm256_add_pd( d_4, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_1 = _mm256_add_pd( d_1, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_5 = _mm256_add_pd( d_5, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_2 = _mm256_add_pd( d_2, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_6 = _mm256_add_pd( d_6, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_3 = _mm256_add_pd( d_3, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_7 = _mm256_add_pd( d_7, tmp );


			A0 += 1*bs;
			A1 += 1*bs;
			B  += 1;

			}
		}


#else

	// prefetch
	a_0 = _mm256_load_pd( &A0[0+bs*0] );
	a_4 = _mm256_load_pd( &A1[0+bs*0] );

	for(k=0; k<kmax-3; k+=4)
		{

		__builtin_prefetch( B+2*B_next+0 );

		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );

		__builtin_prefetch( B+2*B_next+8 );

		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		A0 += 4*bs;
		A1 += 4*bs;
		B  += B_next;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}

#endif
	
	add:

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
		a_0, a_4, A_0, A_4,
		b_0,
		tmp,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7;
	
	__m256d
		c_0, c_1, c_2, c_3,
		c_4, c_5, c_6, c_7,
		t_0, t_1, t_2, t_3;

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

	if(kmax<=0)
		goto add;

	// prefetch
	a_0 = _mm256_load_pd( &A0[0+bs*0] );
	a_4 = _mm256_load_pd( &A1[0+bs*0] );

	for(k=0; k<kmax-3; k+=4)
		{

		__builtin_prefetch( B+2*B_next+0 );

		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );

		__builtin_prefetch( B+2*B_next+8 );

		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		A0 += 4*bs;
		A1 += 4*bs;
		B  += B_next;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_add_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_add_pd( d_7, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
	add:

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
		d_4, d_5;
	
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
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );


		A0 += 4*bs;
		A1 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_add_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_add_pd( d_5, tmp );


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
		d_0, d_1, d_2, d_3;
	
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
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );


		A += 4*bs;
		B += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_add_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_add_pd( d_3, tmp );


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
		d_0, d_1; // TODO use extra accumulation registers !!!!!!!!!!!!!!!!
	
	__m256i 
		mask_m, mask_n;

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );


		A0 += 4*bs;
		B  += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_add_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_add_pd( d_1, tmp );


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
		c_00,
		d_00, d_01, d_02, d_03;
	
#define DIAG 0
#if (DIAG==1)
	if(0)
		{

		if(alg==0)
			{
			
			for(k=0; k<kmax-3; k+=4)
				{

				d_00 = _mm256_load_pd( &B[0] );
				d_01 = _mm256_load_pd( &B[4] );
				d_02 = _mm256_load_pd( &B[8] );
				d_03 = _mm256_load_pd( &B[12] );

				_mm256_store_pd( &D[0], d_00 );
				_mm256_store_pd( &D[4], d_01 );
				_mm256_store_pd( &D[8], d_02 );
				_mm256_store_pd( &D[12], d_03 );
				
				B += 16;
				D += 16;
				
				}
			for(; k<kmax; k++)
				{
				
				d_00 = _mm256_load_pd( &B[0] );

				_mm256_store_pd( &D[0], c_00 );
			
				B += 4;
				D += 4;
				
				}

			}
		else
			{

			for(k=0; k<kmax-3; k+=4)
				{
				
				d_00 = _mm256_load_pd( &B[0] );
				d_01 = _mm256_load_pd( &B[4] );
				d_02 = _mm256_load_pd( &B[8] );
				d_03 = _mm256_load_pd( &B[12] );

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
		
				B += 16;
				C += 16;
				D += 16;
				
				}
			for(; k<kmax; k++)
				{
				
				d_00 = _mm256_load_pd( &B[0] );

				c_00 = _mm256_load_pd( &C[0] );
				d_00 = _mm256_add_pd( c_00, d_00 );

				_mm256_store_pd( &D[0], d_00 );
		
				B += 4;
				C += 4;
				D += 4;
				
				}

			}



		}
	else
		{
#endif
		
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

#if (DIAG==1)
		}
#endif


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


