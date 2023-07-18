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

#define LOW_RANK 11


#if ! defined(BLASFEO)

void kernel_dtrmm_nt_u_10x4_vs_lib4(int km, int kmax, double *A0, int sda, double *B, double *D0, int sdd)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdd;
	double *A2 = A1 + 4*sda;
	double *D2 = D1 + 4*sdd;

	double *tA0 = A0;
	double *tA1 = A1;
	double *tA2 = A2;
	double *tB  = B;
	
	const int bs  = 4;
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567, a_8989,
		b_0123, b_temp,
		b_0, b_1,
		ab_tmp0,
		tmp0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9,
		d_89, d_ab;
	
	__m128d
		a_89,
		ab_tmp1,
		v_8, v_9, v_a, v_b;

	__m128i 
		mask_m;

	if(kmax<=LOW_RANK)
		{

		k = 0;

		a_0123 = _mm256_load_pd( &A0[0+bs*3] );
		a_4567 = _mm256_load_pd( &A1[0+bs*3] );
		a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*3] );

		b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
		d_0    = _mm256_mul_pd( a_0123, b_0 );
		d_4    = _mm256_mul_pd( a_4567, b_0 );

		b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
		d_1    = _mm256_mul_pd( a_0123, b_1 );
		b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
		d_5    = _mm256_mul_pd( a_4567, b_1 );
		d_89   = _mm256_mul_pd( a_8989, b_0 );

		b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
		d_2    = _mm256_mul_pd( a_0123, b_0 );
		d_6    = _mm256_mul_pd( a_4567, b_0 );

		b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
		d_3    = _mm256_mul_pd( a_0123, b_1 );
		b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
		d_7    = _mm256_mul_pd( a_4567, b_1 );
		d_ab   = _mm256_mul_pd( a_8989, b_0 );


		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;
		k  += 4;

#if (LOW_RANK>=8)
#if (LOW_RANK<12)
		if(k<kmax-3)
#else
		for(; k<kmax-3; )
#endif
			{

			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_4567 = _mm256_load_pd( &A1[0+bs*0] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*0] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_4    = _mm256_add_pd( d_4, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_5    = _mm256_add_pd( d_5, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_6    = _mm256_add_pd( d_6, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_7    = _mm256_add_pd( d_7, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*1] );
			a_4567 = _mm256_load_pd( &A1[0+bs*1] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*1] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_4    = _mm256_add_pd( d_4, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_5    = _mm256_add_pd( d_5, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_6    = _mm256_add_pd( d_6, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_7    = _mm256_add_pd( d_7, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*2] );
			a_4567 = _mm256_load_pd( &A1[0+bs*2] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*2] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_4    = _mm256_add_pd( d_4, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_5    = _mm256_add_pd( d_5, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_6    = _mm256_add_pd( d_6, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_7    = _mm256_add_pd( d_7, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*3] );
			a_4567 = _mm256_load_pd( &A1[0+bs*3] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*3] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_4    = _mm256_add_pd( d_4, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_5    = _mm256_add_pd( d_5, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_6    = _mm256_add_pd( d_6, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_7    = _mm256_add_pd( d_7, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			A0 += 16;
			A1 += 16;
			A2 += 16;
			B  += 16;
			k  += 4;

			}
#endif
		if(k<kmax-1)
			{

			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_4567 = _mm256_load_pd( &A1[0+bs*0] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*0] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_4    = _mm256_add_pd( d_4, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_5    = _mm256_add_pd( d_5, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_6    = _mm256_add_pd( d_6, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_7    = _mm256_add_pd( d_7, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*1] );
			a_4567 = _mm256_load_pd( &A1[0+bs*1] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*1] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_4    = _mm256_add_pd( d_4, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_5    = _mm256_add_pd( d_5, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_6    = _mm256_add_pd( d_6, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_7    = _mm256_add_pd( d_7, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );



			A0 += 8;
			A1 += 8;
			A2 += 8;
			B  += 8;
			k  += 2;

			}
		if(k<kmax)
			{


			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_4567 = _mm256_load_pd( &A1[0+bs*0] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[0+bs*0] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_4    = _mm256_add_pd( d_4, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_5    = _mm256_add_pd( d_5, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );
			tmp0   = _mm256_mul_pd( a_4567, b_0 );
			d_6    = _mm256_add_pd( d_6, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_4567, b_1 );
			d_7    = _mm256_add_pd( d_7, tmp0 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );

		

	//		A += 4;
	//		B += 4;
	//		k += 1;

			}

		v_9 = _mm256_extractf128_pd( d_89, 0x1 );
		v_8 = _mm256_castpd256_pd128( d_89 );
		v_b = _mm256_extractf128_pd( d_ab, 0x1 );
		v_a = _mm256_castpd256_pd128( d_ab );

		}
	else
		{

		// prefetch
		a_0123 = _mm256_load_pd( &A0[12] );
		a_4567 = _mm256_load_pd( &A1[12] );
		a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[12] );
		b_0123 = _mm256_load_pd( &B[12] );

		// k = 0, 1, 2 at the end !!!!!!!!
		k = 0;

		// k = 3
		d_8    = _mm256_mul_pd( a_8989, b_0123 );
		b_temp = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		d_0    = _mm256_mul_pd( a_0123, b_0123 );
		d_4    = _mm256_mul_pd( a_4567, b_0123 );

		d_9    = _mm256_mul_pd( a_8989, b_temp );
		a_8989 = _mm256_broadcast_pd( (__m128d *) &A2[16] ); // prefetch
		d_1    = _mm256_mul_pd( a_0123, b_temp );
		b_0123 = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
		d_5    = _mm256_mul_pd( a_4567, b_temp );

		d_2    = _mm256_mul_pd( a_0123, b_0123 );
		b_temp = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		d_6    = _mm256_mul_pd( a_4567, b_0123 );
		b_0123 = _mm256_load_pd( &B[16] ); // prefetch
		
		d_3    = _mm256_mul_pd( a_0123, b_temp );
		a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
		d_7    = _mm256_mul_pd( a_4567, b_temp );
		a_4567 = _mm256_load_pd( &A1[16] ); // prefetch


		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;
		k  += 4;
			

		for( ; k<kmax-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[4] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			a_4567  = _mm256_load_pd( &A1[4] ); // prefetch
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			

	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[8] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			a_4567  = _mm256_load_pd( &A1[8] ); // prefetch
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
		

	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[12] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[12] ); // prefetch
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[12] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			a_4567  = _mm256_load_pd( &A1[12] ); // prefetch
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
		

	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[16] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[16] ); // prefetch
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			a_4567  = _mm256_load_pd( &A1[16] ); // prefetch
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			

			A0 += 16;
			A1 += 16;
			A2 += 16;
			B  += 16;

			}
		
		if(kmax%4>=2)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[4] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			a_4567  = _mm256_load_pd( &A1[4] ); // prefetch
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			

	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[8] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			a_4567  = _mm256_load_pd( &A1[8] ); // prefetch
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
		
			
			A0 += 8;
			A1 += 8;
			A2 += 8;
			B  += 8;

			}

		if(kmax%2==1)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
	//		a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[4] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
	//		b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_6     = _mm256_add_pd( d_6, ab_tmp0 );
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
	//		a_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_temp );
	//		a_4567  = _mm256_load_pd( &A1[4] ); // prefetch
			d_7     = _mm256_add_pd( d_7, ab_tmp0 );
			
			}

		__m128d
			u_8, u_9, u_a, u_b;

		__m256d
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

		t_0 = _mm256_blend_pd( d_8, d_9, 0xa );
		t_1 = _mm256_blend_pd( d_8, d_9, 0x5 );
		
		v_a = _mm256_extractf128_pd( t_0, 0x1 );
		v_8 = _mm256_castpd256_pd128( t_0 );
		v_b = _mm256_extractf128_pd( t_1, 0x1 );
		v_9 = _mm256_castpd256_pd128( t_1 );

		}
	
	if(km>=10)
		mask_m = _mm_set_epi64x( -1, -1 );
	else
		mask_m = _mm_set_epi64x(  1, -1 );



	// store 4th column
	_mm256_store_pd( &D0[0+ldc*3], d_3 );
	_mm256_store_pd( &D1[0+ldc*3], d_7 );
	_mm_maskstore_pd( &D2[0+ldc*3], mask_m, v_b );


	// k = 2
	a_0123 = _mm256_load_pd( &tA0[0+bs*2] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*2] );
	a_89   = _mm_load_pd( &tA2[0+bs*2] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_8   = _mm_add_pd( v_8, ab_tmp1 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_5   = _mm256_add_pd( d_5, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_9   = _mm_add_pd( v_9, ab_tmp1 );

	b_0    = _mm256_broadcast_sd( &tB[2+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_2   = _mm256_add_pd( d_2, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_6   = _mm256_add_pd( d_6, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_a   = _mm_add_pd( v_a, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_store_pd( &D1[0+ldc*2], d_6 );
	_mm_maskstore_pd( &D2[0+ldc*2], mask_m, v_a );

	// k = 1
	a_0123 = _mm256_load_pd( &tA0[0+bs*1] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*1] );
	a_89   = _mm_load_pd( &tA2[0+bs*1] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_8   = _mm_add_pd( v_8, ab_tmp1 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_5   = _mm256_add_pd( d_5, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_9   = _mm_add_pd( v_9, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D1[0+ldc*1], d_5 );
	_mm_maskstore_pd( &D2[0+ldc*1], mask_m, v_9 );

	// k = 0
	a_0123 = _mm256_load_pd( &tA0[0+bs*0] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*0] );
	a_89   = _mm_load_pd( &tA2[0+bs*0] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*0] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_8   = _mm_add_pd( v_8, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D1[0+ldc*0], d_4 );
	_mm_maskstore_pd( &D2[0+ldc*0], mask_m, v_8 );

	return;

	}



void kernel_dtrmm_nt_u_8x4_lib4(int kmax, double *A0, int sda, double *B, double *D0, int sdd)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdd;

	double *tA0 = A0;
	double *tA1 = A1;
	double *tB  = B;
	
	const int bs  = 4;
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		ab_tmp0, temp,
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	__m128d
		a_89,
		ab_tmp1;

	__m256i mask_n;
	
	if(kmax<=LOW_RANK)
		{

		k = 0;

		A_0123 = _mm256_load_pd( &A0[0+bs*3] );
		A_4567 = _mm256_load_pd( &A1[0+bs*3] );
		b_0    = _mm256_broadcast_sd( &B[0+bs*3] );

		b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
		d_0    = _mm256_mul_pd( A_0123, b_0 );
		a_0123 = _mm256_load_pd( &A0[0+bs*4] );
		d_4    = _mm256_mul_pd( A_4567, b_0 );

		b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
		d_1    = _mm256_mul_pd( A_0123, b_1 );
		a_4567 = _mm256_load_pd( &A1[0+bs*4] );
		d_5    = _mm256_mul_pd( A_4567, b_1 );

		b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
		d_2    = _mm256_mul_pd( A_0123, b_0 );
		d_6    = _mm256_mul_pd( A_4567, b_0 );

		b_0    = _mm256_broadcast_sd( &B[0+bs*4] );
		d_3    = _mm256_mul_pd( A_0123, b_1 );
		d_7    = _mm256_mul_pd( A_4567, b_1 );


		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;

#if (LOW_RANK>=8)
#if (LOW_RANK<12)
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



		}
	else
		{

		// prefetch
		A_0123 = _mm256_load_pd( &A0[12] );
		A_4567 = _mm256_load_pd( &A1[12] );
		b_0123 = _mm256_load_pd( &B[12] );

		// k = 0, 1, 2 at the end !!!!!!!!
		k = 0;

		// k = 3
		d_0     = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_4567  = _mm256_load_pd( &A1[16] ); // prefetch
		d_4     = _mm256_mul_pd( A_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[16] ); // prefetch
		d_1     = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
		d_5     = _mm256_mul_pd( A_4567, b_1032 );
		d_3     = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_7     = _mm256_mul_pd( A_4567, b_3210 );
		d_2     = _mm256_mul_pd( A_0123, b_1032 );
		d_6     = _mm256_mul_pd( A_4567, b_1032 );


		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;
			

		for( ; k<kmax-3; k+=4)
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


			
			A0 += 8;
			A1 += 8;
			B  += 8;

			}

		if(kmax%2==1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	//		A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
	//		b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	//		A_4567  = _mm256_load_pd( &A1[4] ); // prefetch
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

		
	//		A0 += 4;
	//		A1 += 4;
	//		B  += 4;

			}

		__m256d
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

		}



	// store 4th column
	_mm256_store_pd( &D0[0+ldc*3], d_3 );
	_mm256_store_pd( &D1[0+ldc*3], d_7 );


	// k = 2
	a_0123 = _mm256_load_pd( &tA0[0+bs*2] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*2] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_5   = _mm256_add_pd( d_5, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[2+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_2   = _mm256_add_pd( d_2, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_6   = _mm256_add_pd( d_6, ab_tmp0 );

	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_store_pd( &D1[0+ldc*2], d_6 );

	// k = 1
	a_0123 = _mm256_load_pd( &tA0[0+bs*1] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*1] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_5   = _mm256_add_pd( d_5, ab_tmp0 );

	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D1[0+ldc*1], d_5 );

	// k = 0
	a_0123 = _mm256_load_pd( &tA0[0+bs*0] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*0] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*0] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );

	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D1[0+ldc*0], d_4 );

#if 1
	return;
#else
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
#endif

	}



void kernel_dtrmm_nt_u_8x4_vs_lib4(int km, int kmax, double *A0, int sda, double *B, double *D0, int sdd)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdd;

	double *tA0 = A0;
	double *tA1 = A1;
	double *tB  = B;
	
	const int bs  = 4;
	const int ldc = 4;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		ab_tmp0, temp,
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	__m128d
		a_89,
		ab_tmp1;

	__m256i 
		mask_m;
	
	if(kmax<=LOW_RANK)
		{

		k = 0;

		A_0123 = _mm256_load_pd( &A0[0+bs*3] );
		A_4567 = _mm256_load_pd( &A1[0+bs*3] );
		b_0    = _mm256_broadcast_sd( &B[0+bs*3] );

		b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
		d_0    = _mm256_mul_pd( A_0123, b_0 );
		a_0123 = _mm256_load_pd( &A0[0+bs*4] );
		d_4    = _mm256_mul_pd( A_4567, b_0 );

		b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
		d_1    = _mm256_mul_pd( A_0123, b_1 );
		a_4567 = _mm256_load_pd( &A1[0+bs*4] );
		d_5    = _mm256_mul_pd( A_4567, b_1 );

		b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
		d_2    = _mm256_mul_pd( A_0123, b_0 );
		d_6    = _mm256_mul_pd( A_4567, b_0 );

		b_0    = _mm256_broadcast_sd( &B[0+bs*4] );
		d_3    = _mm256_mul_pd( A_0123, b_1 );
		d_7    = _mm256_mul_pd( A_4567, b_1 );


		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;

#if (LOW_RANK>=8)
#if (LOW_RANK<12)
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



		}
	else
		{

		// prefetch
		A_0123 = _mm256_load_pd( &A0[12] );
		A_4567 = _mm256_load_pd( &A1[12] );
		b_0123 = _mm256_load_pd( &B[12] );

		// k = 0, 1, 2 at the end !!!!!!!!
		k = 0;

		// k = 3
		d_0     = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		a_4567  = _mm256_load_pd( &A1[16] ); // prefetch
		d_4     = _mm256_mul_pd( A_4567, b_0123 );
		b_0123  = _mm256_load_pd( &B[16] ); // prefetch
		d_1     = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
		d_5     = _mm256_mul_pd( A_4567, b_1032 );
		d_3     = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_7     = _mm256_mul_pd( A_4567, b_3210 );
		d_2     = _mm256_mul_pd( A_0123, b_1032 );
		d_6     = _mm256_mul_pd( A_4567, b_1032 );


		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;
			

		for( ; k<kmax-3; k+=4)
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


			
			A0 += 8;
			A1 += 8;
			B  += 8;

			}

		if(kmax%2==1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	//		A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
	//		b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	//		A_4567  = _mm256_load_pd( &A1[4] ); // prefetch
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

		
	//		A0 += 4;
	//		A1 += 4;
	//		B  += 4;

			}

		__m256d
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

		}


	// compute mask
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );


	// store 4th column
	_mm256_store_pd( &D0[0+ldc*3], d_3 );
	_mm256_maskstore_pd( &D1[0+ldc*3], mask_m, d_7 );


	// k = 2
	a_0123 = _mm256_load_pd( &tA0[0+bs*2] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*2] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_5   = _mm256_add_pd( d_5, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[2+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_2   = _mm256_add_pd( d_2, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_6   = _mm256_add_pd( d_6, ab_tmp0 );

	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_maskstore_pd( &D1[0+ldc*2], mask_m, d_6 );

	// k = 1
	a_0123 = _mm256_load_pd( &tA0[0+bs*1] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*1] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_5   = _mm256_add_pd( d_5, ab_tmp0 );

	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_maskstore_pd( &D1[0+ldc*1], mask_m, d_5 );

	// k = 0
	a_0123 = _mm256_load_pd( &tA0[0+bs*0] );
	a_4567 = _mm256_load_pd( &tA1[0+bs*0] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*0] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp0 = _mm256_mul_pd( a_4567, b_0 );
	d_4   = _mm256_add_pd( d_4, ab_tmp0 );

	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, d_4 );

	return;

	}



void kernel_dtrmm_nt_u_6x4_vs_lib4(int km, int kmax, double *A0, int sda, double *B, double *D0, int sdd)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdd;

	double *tA0 = A0;
	double *tA1 = A1;
	double *tB  = B;
	
	const int bs  = 4;
	const int ldc = 4;
	
	int k;
	
	__m256d
		a_0123, a_8989,
		b_0123, b_temp,
		b_0, b_1,
		ab_tmp0,
		tmp0,
		d_0, d_1, d_2, d_3,
		d_8, d_9,
		d_89, d_ab;
	
	__m128d
		a_89,
		ab_tmp1,
		v_8, v_9, v_a, v_b;

	__m128i 
		mask_m;

	if(kmax<=LOW_RANK)
		{

		k = 0;

		a_0123 = _mm256_load_pd( &A0[0+bs*3] );
		a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*3] );

		b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
		d_0    = _mm256_mul_pd( a_0123, b_0 );

		b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
		d_1    = _mm256_mul_pd( a_0123, b_1 );
		b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
		d_89   = _mm256_mul_pd( a_8989, b_0 );

		b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
		d_2    = _mm256_mul_pd( a_0123, b_0 );

		b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
		d_3    = _mm256_mul_pd( a_0123, b_1 );
		b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
		d_ab   = _mm256_mul_pd( a_8989, b_0 );


		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;

#if (LOW_RANK>=8)
#if (LOW_RANK<12)
		if(k<kmax-3)
#else
		for(; k<kmax-3; )
#endif
			{

			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*0] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*1] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*1] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*2] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*2] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*2] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*3] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*3] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			A0 += 16;
			A1 += 16;
			B  += 16;
			k  += 4;

			}
#endif
		if(k<kmax-1)
			{

			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*0] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );


			a_0123 = _mm256_load_pd( &A0[0+bs*1] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*1] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );



			A0 += 8;
			A1 += 8;
			B  += 8;
			k  += 2;

			}
		if(k<kmax)
			{


			a_0123 = _mm256_load_pd( &A0[0+bs*0] );
			a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[0+bs*0] );

			b_0    = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_0    = _mm256_add_pd( d_0, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_1    = _mm256_add_pd( d_1, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_89   = _mm256_add_pd( d_89, tmp0 );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_0 );
			d_2    = _mm256_add_pd( d_2, tmp0 );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			tmp0   = _mm256_mul_pd( a_0123, b_1 );
			d_3    = _mm256_add_pd( d_3, tmp0 );
			b_0    = _mm256_permute2f128_pd( b_0, b_1, 0x20 );
			tmp0   = _mm256_mul_pd( a_8989, b_0 );
			d_ab   = _mm256_add_pd( d_ab, tmp0 );

		

	//		A += 4;
	//		B += 4;
	//		k += 1;

			}

		v_9 = _mm256_extractf128_pd( d_89, 0x1 );
		v_8 = _mm256_castpd256_pd128( d_89 );
		v_b = _mm256_extractf128_pd( d_ab, 0x1 );
		v_a = _mm256_castpd256_pd128( d_ab );

		}
	else
		{

		// prefetch
		a_0123 = _mm256_load_pd( &A0[12] );
		a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[12] );
		b_0123 = _mm256_load_pd( &B[12] );

		// k = 0, 1, 2 at the end !!!!!!!!
		k = 0;

		// k = 3
		d_8    = _mm256_mul_pd( a_8989, b_0123 );
		b_temp = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
		d_0    = _mm256_mul_pd( a_0123, b_0123 );

		d_9    = _mm256_mul_pd( a_8989, b_temp );
		a_8989 = _mm256_broadcast_pd( (__m128d *) &A1[16] ); // prefetch
		d_1    = _mm256_mul_pd( a_0123, b_temp );
		b_0123 = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301

		d_2    = _mm256_mul_pd( a_0123, b_0123 );
		b_temp = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
		b_0123 = _mm256_load_pd( &B[16] ); // prefetch
		
		d_3    = _mm256_mul_pd( a_0123, b_temp );
		a_0123 = _mm256_load_pd( &A0[16] ); // prefetch


		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;
			

		for( ; k<kmax-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A1[4] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			

	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A1[8] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
		

	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A1[12] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[12] ); // prefetch
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[12] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
		

	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A1[16] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[16] ); // prefetch
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			

			A0 += 16;
			A1 += 16;
			B  += 16;

			}
		
		if(kmax%4>=2)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A1[4] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			

	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
			a_8989  = _mm256_broadcast_pd( (__m128d *) &A1[8] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
		
			
			A0 += 8;
			A1 += 8;
			B  += 8;

			}

		if(kmax%2==1)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_8989, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 1032
			d_8     = _mm256_add_pd( d_8, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_8989, b_temp );
	//		a_8989  = _mm256_broadcast_pd( (__m128d *) &A2[4] ); // prefetch
			d_9     = _mm256_add_pd( d_9, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
			b_0123  = _mm256_permute2f128_pd( b_0123, b_0123, 0x1 ); // 2301
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );

			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_temp  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 ); // 3210
			d_2     = _mm256_add_pd( d_2, ab_tmp0 );
	//		b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_temp );
	//		a_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_3     = _mm256_add_pd( d_3, ab_tmp0 );
			
			}

		__m128d
			u_8, u_9, u_a, u_b;

		__m256d
			t_0, t_1, t_2, t_3;

		t_0 = _mm256_blend_pd( d_0, d_1, 0xa );
		t_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
		t_2 = _mm256_blend_pd( d_2, d_3, 0xa );
		t_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

		d_0 = _mm256_blend_pd( t_0, t_2, 0xc );
		d_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
		d_1 = _mm256_blend_pd( t_1, t_3, 0xc );
		d_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

		t_0 = _mm256_blend_pd( d_8, d_9, 0xa );
		t_1 = _mm256_blend_pd( d_8, d_9, 0x5 );
		
		v_a = _mm256_extractf128_pd( t_0, 0x1 );
		v_8 = _mm256_castpd256_pd128( t_0 );
		v_b = _mm256_extractf128_pd( t_1, 0x1 );
		v_9 = _mm256_castpd256_pd128( t_1 );

		}


	if(km>=6)
		mask_m = _mm_set_epi64x( -1, -1 );
	else
		mask_m = _mm_set_epi64x(  1, -1 );


	// store 4th column
	_mm256_store_pd( &D0[0+ldc*3], d_3 );
	_mm_maskstore_pd( &D1[0+ldc*3], mask_m, v_b );


	// k = 2
	a_0123 = _mm256_load_pd( &tA0[0+bs*2] );
	a_89   = _mm_load_pd( &tA1[0+bs*2] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_8   = _mm_add_pd( v_8, ab_tmp1 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_9   = _mm_add_pd( v_9, ab_tmp1 );

	b_0    = _mm256_broadcast_sd( &tB[2+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_2   = _mm256_add_pd( d_2, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_a   = _mm_add_pd( v_a, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm_maskstore_pd( &D1[0+ldc*2], mask_m, v_a );

	// k = 1
	a_0123 = _mm256_load_pd( &tA0[0+bs*1] );
	a_89   = _mm_load_pd( &tA1[0+bs*1] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_8   = _mm_add_pd( v_8, ab_tmp1 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_9   = _mm_add_pd( v_9, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm_maskstore_pd( &D1[0+ldc*1], mask_m, v_9 );

	// k = 0
	a_0123 = _mm256_load_pd( &tA0[0+bs*0] );
	a_89   = _mm_load_pd( &tA1[0+bs*0] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*0] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );
	ab_tmp1 = _mm_mul_pd( a_89, _mm256_castpd256_pd128( b_0 ) );
	v_8   = _mm_add_pd( v_8, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm_maskstore_pd( &D1[0+ldc*0], mask_m, v_8 );

	return;

	}



void kernel_dtrmm_nt_u_4x4_vs_lib4(int km, int kmax, double *A0, double *B, double *D0)
	{
	
	double *tA0 = A0;
	double *tB  = B;
	
	const int bs  = 4;
	const int ldc = 4;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, A_0123, 
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		ab_tmp0, temp,
		d_0, d_1, d_3, d_2;
	
	__m128d
		a_89,
		ab_tmp1;

	__m256i 
		mask_m;
	
	if(kmax<=LOW_RANK)
		{

		k = 0;

		A_0123 = _mm256_load_pd( &A0[0+bs*3] );
		b_0    = _mm256_broadcast_sd( &B[0+bs*3] );

		b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
		d_0    = _mm256_mul_pd( A_0123, b_0 );
		a_0123 = _mm256_load_pd( &A0[0+bs*4] );

		b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
		d_1    = _mm256_mul_pd( A_0123, b_1 );

		b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
		d_2    = _mm256_mul_pd( A_0123, b_0 );

		b_0    = _mm256_broadcast_sd( &B[0+bs*4] );
		d_3    = _mm256_mul_pd( A_0123, b_1 );


		A0 += 16;
		B  += 16;
		k  += 4;

#if (LOW_RANK>=8)
#if (LOW_RANK<12)
		if(k<kmax-3)
#else
		for(; k<kmax-3; )
#endif
			{

			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			d_0   = _mm256_add_pd( d_0, temp );
			A_0123 = _mm256_load_pd( &A0[0+bs*1] );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_1   = _mm256_add_pd( d_1, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			d_2   = _mm256_add_pd( d_2, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_3   = _mm256_add_pd( d_3, temp );
			


			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			d_0   = _mm256_add_pd( d_0, temp );
			a_0123 = _mm256_load_pd( &A0[0+bs*2] );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			d_1   = _mm256_add_pd( d_1, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			d_2   = _mm256_add_pd( d_2, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			d_3   = _mm256_add_pd( d_3, temp );



			b_1    = _mm256_broadcast_sd( &B[1+bs*2] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			d_0   = _mm256_add_pd( d_0, temp );
			A_0123 = _mm256_load_pd( &A0[0+bs*3] );

			b_0    = _mm256_broadcast_sd( &B[2+bs*2] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_1   = _mm256_add_pd( d_1, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*2] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			d_2   = _mm256_add_pd( d_2, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*3] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_3   = _mm256_add_pd( d_3, temp );
			


			b_1    = _mm256_broadcast_sd( &B[1+bs*3] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			d_0   = _mm256_add_pd( d_0, temp );
			a_0123 = _mm256_load_pd( &A0[0+bs*4] );

			b_0    = _mm256_broadcast_sd( &B[2+bs*3] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			d_1   = _mm256_add_pd( d_1, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*3] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			d_2   = _mm256_add_pd( d_2, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*4] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			d_3   = _mm256_add_pd( d_3, temp );


			A0 += 16;
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

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_1   = _mm256_add_pd( d_1, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			d_2   = _mm256_add_pd( d_2, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_3   = _mm256_add_pd( d_3, temp );
			


			b_1    = _mm256_broadcast_sd( &B[1+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			d_0   = _mm256_add_pd( d_0, temp );
			a_0123 = _mm256_load_pd( &A0[0+bs*2] );

			b_0    = _mm256_broadcast_sd( &B[2+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			d_1   = _mm256_add_pd( d_1, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*1] );
			temp   = _mm256_mul_pd( A_0123, b_0 );
			d_2   = _mm256_add_pd( d_2, temp );

			b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
			temp   = _mm256_mul_pd( A_0123, b_1 );
			d_3   = _mm256_add_pd( d_3, temp );


			A0 += 8;
			B  += 8;
			k  += 2;

			}
		if(k<kmax)
			{


			b_1    = _mm256_broadcast_sd( &B[1+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			d_0   = _mm256_add_pd( d_0, temp );
//			a_0123 = _mm256_load_pd( &A0[0+bs*0] );

			b_0    = _mm256_broadcast_sd( &B[2+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_1   = _mm256_add_pd( d_1, temp );

			b_1    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_0 );
			d_2   = _mm256_add_pd( d_2, temp );

//			b_0    = _mm256_broadcast_sd( &B[3+bs*0] );
			temp   = _mm256_mul_pd( a_0123, b_1 );
			d_3   = _mm256_add_pd( d_3, temp );
			

	//		A += 4;
	//		B += 4;
	//		k += 1;

			}



		}
	else
		{

		// prefetch
		A_0123 = _mm256_load_pd( &A0[12] );
		b_0123 = _mm256_load_pd( &B[12] );

		// k = 0, 1, 2 at the end !!!!!!!!
		k = 0;

		// k = 3
		d_0     = _mm256_mul_pd( A_0123, b_0123 );
		b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123  = _mm256_load_pd( &B[16] ); // prefetch
		d_1     = _mm256_mul_pd( A_0123, b_1032 );
		b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
		d_3     = _mm256_mul_pd( A_0123, b_3210 );
		b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		d_2     = _mm256_mul_pd( A_0123, b_1032 );


		A0 += 16;
		B  += 16;
		k  += 4;
			

		for( ; k<kmax-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[12] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[12] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &A0[16] ); // prefetch
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );
			

			A0 += 16;
			B  += 16;

			}
		
		if(kmax%4>=2)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &A0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &B[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );


			
			A0 += 8;
			B  += 8;

			}

		if(kmax%2==1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	//		A_0123  = _mm256_load_pd( &A0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
	//		b_0123  = _mm256_load_pd( &B[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );

		
	//		A0 += 4;
	//		A1 += 4;
	//		B  += 4;

			}

		__m256d
			t_0, t_1, t_2, t_3;

		t_0 = _mm256_blend_pd( d_0, d_1, 0xa );
		t_1 = _mm256_blend_pd( d_0, d_1, 0x5 );
		t_2 = _mm256_blend_pd( d_2, d_3, 0xa );
		t_3 = _mm256_blend_pd( d_2, d_3, 0x5 );

		d_0 = _mm256_blend_pd( t_0, t_2, 0xc );
		d_2 = _mm256_blend_pd( t_0, t_2, 0x3 );
		d_1 = _mm256_blend_pd( t_1, t_3, 0xc );
		d_3 = _mm256_blend_pd( t_1, t_3, 0x3 );

		}


	// compute mask
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );


	// store 4th column
	_mm256_maskstore_pd( &D0[0+ldc*3], mask_m, d_3 );


	// k = 2
	a_0123 = _mm256_load_pd( &tA0[0+bs*2] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[2+bs*2] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_2   = _mm256_add_pd( d_2, ab_tmp0 );

	_mm256_maskstore_pd( &D0[0+ldc*2], mask_m, d_2 );

	// k = 1
	a_0123 = _mm256_load_pd( &tA0[0+bs*1] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );

	b_0    = _mm256_broadcast_sd( &tB[1+bs*1] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_1   = _mm256_add_pd( d_1, ab_tmp0 );

	_mm256_maskstore_pd( &D0[0+ldc*1], mask_m, d_1 );

	// k = 0
	a_0123 = _mm256_load_pd( &tA0[0+bs*0] );

	b_0    = _mm256_broadcast_sd( &tB[0+bs*0] );
	ab_tmp0 = _mm256_mul_pd( a_0123, b_0 );
	d_0   = _mm256_add_pd( d_0, ab_tmp0 );

	_mm256_maskstore_pd( &D0[0+ldc*0], mask_m, d_0 );

	return;

	}



void corner_dtrmm_nt_u_8x3_vs_lib4(int km, double *A0, int sda, double *B, double *D0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdc;
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	__m256d
		ab_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32, a_40_50_60_70, a_41_51_61_71, a_42_52_62_72,
		b_00, b_10, b_20, b_11, b_21, b_22,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_40_50_60_70, c_41_51_61_71, c_42_52_62_72;
	
	__m256i
		mask_m;
	
	a_00_10_20_30 = _mm256_load_pd( &A0[0+4*0] );
	a_40_50_60_70 = _mm256_load_pd( &A1[0+4*0] );
	a_01_11_21_31 = _mm256_load_pd( &A0[0+4*1] );
	a_41_51_61_71 = _mm256_load_pd( &A1[0+4*1] );
	a_02_12_22_32 = _mm256_load_pd( &A0[0+4*2] );
	a_42_52_62_72 = _mm256_load_pd( &A1[0+4*2] );
	
	// compute mask
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	// first column 
	b_00 = _mm256_broadcast_sd( &B[0+4*0] );
	c_00_10_20_30 = _mm256_mul_pd( a_00_10_20_30, b_00 );
	c_40_50_60_70 = _mm256_mul_pd( a_40_50_60_70, b_00 );

	b_10 = _mm256_broadcast_sd( &B[0+4*1] );
	ab_temp = _mm256_mul_pd( a_01_11_21_31, b_10 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
	ab_temp = _mm256_mul_pd( a_41_51_61_71, b_10 );
	c_40_50_60_70 = _mm256_add_pd( c_40_50_60_70, ab_temp );

	b_20 = _mm256_broadcast_sd( &B[0+4*2] );
	ab_temp = _mm256_mul_pd( a_02_12_22_32, b_20 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
	ab_temp = _mm256_mul_pd( a_42_52_62_72, b_20 );
	c_40_50_60_70 = _mm256_add_pd( c_40_50_60_70, ab_temp );

	_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, c_40_50_60_70 );
	
	// second column 
	b_11 = _mm256_broadcast_sd( &B[1+4*1] );
	c_01_11_21_31 = _mm256_mul_pd( a_01_11_21_31, b_11 );
	c_41_51_61_71 = _mm256_mul_pd( a_41_51_61_71, b_11 );

	b_21 = _mm256_broadcast_sd( &B[1+4*2] );
	ab_temp = _mm256_mul_pd( a_02_12_22_32, b_21 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
	ab_temp = _mm256_mul_pd( a_42_52_62_72, b_21 );
	c_41_51_61_71 = _mm256_add_pd( c_41_51_61_71, ab_temp );
	
	_mm256_store_pd( &D0[0+ldc*1], c_01_11_21_31 );
	_mm256_maskstore_pd( &D1[0+ldc*1], mask_m, c_41_51_61_71 );
	
	// third column 
	b_22 = _mm256_broadcast_sd( &B[2+4*2] );
	c_02_12_22_32 = _mm256_mul_pd( a_02_12_22_32, b_22 );
	c_42_52_62_72 = _mm256_mul_pd( a_42_52_62_72, b_22 );

	_mm256_store_pd( &D0[0+ldc*2], c_02_12_22_32 );
	_mm256_maskstore_pd( &D1[0+ldc*2], mask_m, c_42_52_62_72 );

	}
	


void corner_dtrmm_nt_u_8x2_vs_lib4(int km, double *A0, int sda, double *B, double *D0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdc;
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	__m256d
		ab_temp,
		a_00_10_20_30, a_01_11_21_31, a_40_50_60_70, a_41_51_61_71,
		b_00, b_10, b_11,
		c_00_10_20_30, c_01_11_21_31, c_40_50_60_70, c_41_51_61_71;
	
	__m256i
		mask_m;
	
	a_00_10_20_30 = _mm256_load_pd( &A0[0+4*0] );
	a_40_50_60_70 = _mm256_load_pd( &A1[0+4*0] );
	a_01_11_21_31 = _mm256_load_pd( &A0[0+4*1] );
	a_41_51_61_71 = _mm256_load_pd( &A1[0+4*1] );
	
	// compute mask
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	// first column 
	b_00 = _mm256_broadcast_sd( &B[0+4*0] );
	c_00_10_20_30 = _mm256_mul_pd( a_00_10_20_30, b_00 );
	c_40_50_60_70 = _mm256_mul_pd( a_40_50_60_70, b_00 );

	b_10 = _mm256_broadcast_sd( &B[0+4*1] );
	ab_temp = _mm256_mul_pd( a_01_11_21_31, b_10 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
	ab_temp = _mm256_mul_pd( a_41_51_61_71, b_10 );
	c_40_50_60_70 = _mm256_add_pd( c_40_50_60_70, ab_temp );
	
	_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, c_40_50_60_70 );

	// second column 
	b_11 = _mm256_broadcast_sd( &B[1+4*1] );
	c_01_11_21_31 = _mm256_mul_pd( a_01_11_21_31, b_11 );
	c_41_51_61_71 = _mm256_mul_pd( a_41_51_61_71, b_11 );
	
	_mm256_store_pd( &D0[0+ldc*1], c_01_11_21_31 );
	_mm256_maskstore_pd( &D1[0+ldc*1], mask_m, c_41_51_61_71 );
	
	}



void corner_dtrmm_nt_u_8x1_vs_lib4(int km, double *A0, int sda, double *B, double *D0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdc;
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	__m256d
		a_00_10_20_30, a_40_50_60_70,
		b_00,
		c_00_10_20_30, c_40_50_60_70;
	
	__m256i
		mask_m;
	
	// compute mask
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	a_00_10_20_30 = _mm256_load_pd( &A0[0+4*0] );
	a_40_50_60_70 = _mm256_load_pd( &A1[0+4*0] );
	
	// first column 
	b_00 = _mm256_broadcast_sd( &B[0+4*0] );
	c_00_10_20_30 = _mm256_mul_pd( a_00_10_20_30, b_00 );
	c_40_50_60_70 = _mm256_mul_pd( a_40_50_60_70, b_00 );

	_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, c_40_50_60_70 );
	
	}


void corner_dtrmm_nt_u_4x3_vs_lib4(int km, double *A, double *B, double *D)
	{
	
#if 1
	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	__m256d
		temp,
		a_0123,
		b_0,
		c_00, c_01, c_02;
	
	__m256i
		mask_m;
	
	// compute mask
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	// k = 0
	a_0123 = _mm256_load_pd( &A[0+bs*0] );

	b_0    = _mm256_broadcast_sd( &B[0+bs*0] );
	c_00   = _mm256_mul_pd( a_0123, b_0 );

	// k = 1
	a_0123 = _mm256_load_pd( &A[0+bs*1] );

	b_0    = _mm256_broadcast_sd( &B[0+bs*1] );
	temp   = _mm256_mul_pd( a_0123, b_0 );
	c_00   = _mm256_add_pd( c_00, temp );

	b_0    = _mm256_broadcast_sd( &B[1+bs*1] );
	c_01   = _mm256_mul_pd( a_0123, b_0 );

	// k = 2
	a_0123 = _mm256_load_pd( &A[0+bs*2] );

	b_0    = _mm256_broadcast_sd( &B[0+bs*2] );
	temp   = _mm256_mul_pd( a_0123, b_0 );
	c_00   = _mm256_add_pd( c_00, temp );
	_mm256_maskstore_pd( &D[0+bs*0], mask_m, c_00 );

	b_0    = _mm256_broadcast_sd( &B[1+bs*2] );
	temp   = _mm256_mul_pd( a_0123, b_0 );
	c_01   = _mm256_add_pd( c_01, temp );
	_mm256_maskstore_pd( &D[0+bs*1], mask_m, c_01 );

	b_0    = _mm256_broadcast_sd( &B[2+bs*2] );
	c_02   = _mm256_mul_pd( a_0123, b_0 );
	_mm256_maskstore_pd( &D[0+bs*2], mask_m, c_02 );

#else
	const int ldc = 4;

	__m256d
		ab_temp,
		a_00_10_20_30, a_01_11_21_31, a_02_12_22_32,
		b_00, b_10, b_20, b_11, b_21, b_22,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32;
	
	a_00_10_20_30 = _mm256_load_pd( &A[0+4*0] );
	a_01_11_21_31 = _mm256_load_pd( &A[0+4*1] );
	a_02_12_22_32 = _mm256_load_pd( &A[0+4*2] );
	
	// first column 
	b_00 = _mm256_broadcast_sd( &B[0+4*0] );
	c_00_10_20_30 = _mm256_mul_pd( a_00_10_20_30, b_00 );

	b_10 = _mm256_broadcast_sd( &B[0+4*1] );
	ab_temp = _mm256_mul_pd( a_01_11_21_31, b_10 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );

	b_20 = _mm256_broadcast_sd( &B[0+4*2] );
	ab_temp = _mm256_mul_pd( a_02_12_22_32, b_20 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );

	_mm256_store_pd( &D[0+ldc*0], c_00_10_20_30 );
	
	// second column 
	b_11 = _mm256_broadcast_sd( &B[1+4*1] );
	c_01_11_21_31 = _mm256_mul_pd( a_01_11_21_31, b_11 );

	b_21 = _mm256_broadcast_sd( &B[1+4*2] );
	ab_temp = _mm256_mul_pd( a_02_12_22_32, b_21 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );
	
	_mm256_store_pd( &D[0+ldc*1], c_01_11_21_31 );
	
	// third column 
	b_22 = _mm256_broadcast_sd( &B[2+4*2] );
	c_02_12_22_32 = _mm256_mul_pd( a_02_12_22_32, b_22 );

	_mm256_store_pd( &D[0+ldc*2], c_02_12_22_32 );
#endif

	}
	


void corner_dtrmm_nt_u_4x2_vs_lib4(int km, double *A, double *B, double *D)
	{
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	__m256d
		ab_temp,
		a_00_10_20_30, a_01_11_21_31,
		b_00, b_10, b_11,
		c_00_10_20_30, c_01_11_21_31;
	
	__m256i
		mask_m;
	
	// compute mask
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	a_00_10_20_30 = _mm256_load_pd( &A[0+4*0] );
	a_01_11_21_31 = _mm256_load_pd( &A[0+4*1] );
	
	// first column 
	b_00 = _mm256_broadcast_sd( &B[0+4*0] );
	c_00_10_20_30 = _mm256_mul_pd( a_00_10_20_30, b_00 );

	b_10 = _mm256_broadcast_sd( &B[0+4*1] );
	ab_temp = _mm256_mul_pd( a_01_11_21_31, b_10 );
	c_00_10_20_30 = _mm256_add_pd( c_00_10_20_30, ab_temp );
	
	_mm256_maskstore_pd( &D[0+ldc*0], mask_m, c_00_10_20_30 );

	// second column 
	b_11 = _mm256_broadcast_sd( &B[1+4*1] );
	c_01_11_21_31 = _mm256_mul_pd( a_01_11_21_31, b_11 );
	
	_mm256_maskstore_pd( &D[0+ldc*1], mask_m, c_01_11_21_31 );
	
	}



void corner_dtrmm_nt_u_4x1_vs_lib4(int km, double *A, double *B, double *D)
	{
	
	const int ldc = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	__m256d
		a_00_10_20_30,
		b_00,
		c_00_10_20_30;
	
	__m256i
		mask_m;
	
	// compute mask
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	a_00_10_20_30 = _mm256_load_pd( &A[0+4*0] );
	
	// first column 
	b_00 = _mm256_broadcast_sd( &B[0+4*0] );
	c_00_10_20_30 = _mm256_mul_pd( a_00_10_20_30, b_00 );

	_mm256_maskstore_pd( &D[0+ldc*0], mask_m, c_00_10_20_30 );
	
	}



void kernel_dtrmm_nt_l_8x4_lib4(int kmax, double *A0, int sda, double *B, double *D0, int sdc)
	{
	
	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdc;
	
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

	_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
	_mm256_store_pd( &D1[0+ldc*0], c_40_50_60_70 );

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

	_mm256_store_pd( &D0[0+ldc*1], c_01_11_21_31 );
	_mm256_store_pd( &D1[0+ldc*1], c_41_51_61_71 );

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

	_mm256_store_pd( &D0[0+ldc*2], c_02_12_22_32 );
	_mm256_store_pd( &D1[0+ldc*2], c_42_52_62_72 );

	// k=kmax
	ab_tmp0      = _mm256_mul_pd( a_0123, b_3 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_3 );
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_tmp0 );
	c_43_53_63_73 = _mm256_add_pd( c_43_53_63_73, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*3], c_03_13_23_33 );
	_mm256_store_pd( &D1[0+ldc*3], c_43_53_63_73 );

	}



void kernel_dtrmm_nt_l_8x2_lib4(int kmax, double *A0, int sda, double *B, double *D0, int sdc)
	{

	double *A1 = A0 + 4*sda;
	double *D1 = D0 + 4*sdc;
	
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

	_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
	_mm256_store_pd( &D1[0+ldc*0], c_40_50_60_70 );

	// k=kmax-2
	ab_tmp0      = _mm256_mul_pd( a_0123, b_1 );
	ab_tmp1      = _mm256_mul_pd( a_4567, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_tmp0 );
	c_41_51_61_71 = _mm256_add_pd( c_41_51_61_71, ab_tmp1 );

	_mm256_store_pd( &D0[0+ldc*1], c_01_11_21_31 );
	_mm256_store_pd( &D1[0+ldc*1], c_41_51_61_71 );

	}



void kernel_dtrmm_nt_l_4x4_lib4(int kmax, double *A, double *B, double *D)
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

	_mm256_store_pd( &D[0+ldc*0], c_00_10_20_30 );

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

	_mm256_store_pd( &D[0+ldc*1], c_01_11_21_31 );

	// k=kmax-1
	ab_temp       = _mm256_mul_pd( a_0123, b_2 );
	c_02_12_22_32 = _mm256_add_pd( c_02_12_22_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3 );
	a_0123        = _mm256_load_pd( &A[12] ); // prefetch
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );
	b_3           = _mm256_broadcast_sd( &B[15] );

	_mm256_store_pd( &D[0+ldc*2], c_02_12_22_32 );
	
	// k=kmax
	ab_temp       = _mm256_mul_pd( a_0123, b_3 );
	c_03_13_23_33 = _mm256_add_pd( c_03_13_23_33, ab_temp );

	_mm256_store_pd( &D[0+ldc*3], c_03_13_23_33 );

	}



void kernel_dtrmm_nt_l_4x2_lib4(int kmax, double *A, double *B, double *D)
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

	_mm256_store_pd( &D[0+ldc*0], c_00_10_20_30 );

	// k=kmax-2
	ab_temp       = _mm256_mul_pd( a_0123, b_1 );
	c_01_11_21_31 = _mm256_add_pd( c_01_11_21_31, ab_temp );

	_mm256_store_pd( &D[0+ldc*1], c_01_11_21_31 );

	}



void kernel_dtrmm_nt_l_2x4_lib4(int kmax, double *A, double *B, double *D)
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

	D[0+ldc*0] = c_00;
	D[1+ldc*0] = c_10;
		
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

	D[0+ldc*1] = c_01;
	D[1+ldc*1] = c_11;
		
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

	D[0+ldc*2] = c_02;
	D[1+ldc*2] = c_12;
		
	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=kmax
	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
		
	b_3 = B[3+lda*3];

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	D[0+ldc*3] = c_03;
	D[1+ldc*3] = c_13;
		
	}



void kernel_dtrmm_nt_l_2x2_lib4(int kmax, double *A, double *B, double *D)
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

	D[0+ldc*0] = c_00;
	D[1+ldc*0] = c_10;
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	// k=kmax-2
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
		
	b_1 = B[1+lda*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	D[0+ldc*1] = c_01;
	D[1+ldc*1] = c_11;
	
	}

#endif


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

	k = 0;

	// triangle at the beginning
	if(kmax>=8)
		{

		ab_tmp1       = _mm256_setzero_pd();

		// k = 0
		a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		// k = 1
		a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x3 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );

		// k = 2
		a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x7 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );

		// k = 3
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );

		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;

		// k = 4
		a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x1 );
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
		
		// k = 5
		ab_tmp1       = _mm256_setzero_pd();
		a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x3 );
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

		// k = 6
		ab_tmp1       = _mm256_setzero_pd();
		a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x7 );
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

		// k = 7
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
		k  += 4;


		for(; k<kmax-3; k+=4)
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
		for(; k<kmax-1; k+=2)
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

		for(; k<kmax; k+=1)
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
			
//			A0 += 4;
//			A1 += 4;
//			B  += 4;

			}
		}
	else
		{

		// kmax is at least 5 !!!

		ab_tmp1       = _mm256_setzero_pd();

		// k = 0
		a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		// k = 1
		a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x3 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );

		// k = 2
		a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x7 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );

		// k = 3
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_tmp0 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_tmp0 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;
		k  += 4;

		// k = 4
		a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x1 );
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

		if(kmax>5)
			{
		
			// k = 5
			ab_tmp1       = _mm256_setzero_pd();
			a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x3 );
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

			if(kmax>6)
				{

				// k = 6
				ab_tmp1       = _mm256_setzero_pd();
				a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x7 );
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

				}

			}

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

	// triangle at the beginning
	if(kmax>=4)
		{

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
		}
	else
		{

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

		if(kmax>1)
			{
		
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

			if(kmax>2)
				{

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

				}

			}

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


void kernel_dtrmm_l_u_nt_4x4_vs_lib4(int km, int kmax, double *A, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;

	__m256d
		zeros,
		a_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_temp, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31;
	
	__m256i 
		mask_n, mask_m;

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

	// triangle at the beginning
	if(kmax>=4)
		{

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
		}
	else
		{

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

		if(kmax>1)
			{
		
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

			if(kmax>2)
				{

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

				}

			}

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
		
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00_10_20_30 = _mm256_load_pd( &C[0+bs*0] );
		d_01_11_21_31 = _mm256_load_pd( &C[0+bs*1] );
		d_02_12_22_32 = _mm256_load_pd( &C[0+bs*2] );
		d_03_13_23_33 = _mm256_load_pd( &C[0+bs*3] );
		
		if(alg==1) // C += A * B'
			{
			c_00_10_20_30 = _mm256_add_pd( d_00_10_20_30, c_00_10_20_30 );
			c_01_11_21_31 = _mm256_add_pd( d_01_11_21_31, c_01_11_21_31 );
			c_02_12_22_32 = _mm256_add_pd( d_02_12_22_32, c_02_12_22_32 );
			c_03_13_23_33 = _mm256_add_pd( d_03_13_23_33, c_03_13_23_33 );
			}
		else // C -= A * B'
			{
			c_00_10_20_30 = _mm256_sub_pd( d_00_10_20_30, c_00_10_20_30 );
			c_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, c_01_11_21_31 );
			c_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, c_02_12_22_32 );
			c_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, c_03_13_23_33 );
			}
		}

	store:
	_mm256_maskstore_pd( &D[0+bs*0], mask_m, c_00_10_20_30 );
	_mm256_maskstore_pd( &D[0+bs*1], mask_m, c_01_11_21_31 );
	_mm256_maskstore_pd( &D[0+bs*2], mask_m, c_02_12_22_32 );
	_mm256_maskstore_pd( &D[0+bs*3], mask_m, c_03_13_23_33 );

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



