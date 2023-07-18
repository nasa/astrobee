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

#include "../../include/block_size.h"

#define LOW_RANK 7


#if ! defined(BLASFEO)

// normal-transposed, 8x4 with data packed in 4
void kernel_dsyrk_nt_8x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	const int ldc = bs;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		b_0, b_1,
		temp,
		ab_tmp0, ab_tmp1, // temporary results
		d_0, d_1, d_3, d_2,
		d_4, d_5, d_7, d_6;
	
	__m256i
		mask_m;
	
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
		goto add_n;

	// low-rank update
	if(kmax<8)
		{

		k = 0;

		a_0123 = _mm256_load_pd( &A0[0+bs*0] );
		a_4567 = _mm256_load_pd( &A1[0+bs*0] );
		b_0    = _mm256_broadcast_sd( &B[0+bs*0] );

#if (LOW_RANK>=4)
#if (LOW_RANK<8)
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

	//goto store_n;
	

	// store (5 - 8) x (3 - 4)
	store_n:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	c_1 = _mm256_load_pd( &D0[0+ldc*1] );
	c_2 = _mm256_load_pd( &D0[0+ldc*2] );

	d_1 = _mm256_blend_pd( d_1, c_1, 0x1 );
	d_2 = _mm256_blend_pd( d_2, c_2, 0x3 );

	_mm256_store_pd( &D0[0+ldc*0], d_0 );
	_mm256_store_pd( &D0[0+ldc*1], d_1 );
	_mm256_store_pd( &D0[0+ldc*2], d_2 );
	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, d_4 );
	_mm256_maskstore_pd( &D1[0+ldc*1], mask_m, d_5 );
	_mm256_maskstore_pd( &D1[0+ldc*2], mask_m, d_6 );

	if(kn>=4)
		{
		c_3 = _mm256_load_pd( &D0[0+ldc*3] );
		d_3 = _mm256_blend_pd( d_3, c_3, 0x7 );
		_mm256_store_pd( &D0[0+ldc*3], d_3 );
		_mm256_maskstore_pd( &D1[0+ldc*3], mask_m, d_7 );
		}

	}



// normal-transposed, 8x2 with data packed in 4
void kernel_dsyrk_nt_8x2_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg)
	{
	
//	if(kmax<=0)
//		return;
	
	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	const int ldc = bs;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, a_4567, //A_0123,
		b_0101, b_1010,
		ab_tmp0, ab_tmp1, // temporary results
		c_00_11_20_31, c_01_10_21_30,
		c_40_51_60_71, c_41_50_61_70;
	
	__m256d
		c_00_10_20_30, c_01_11_21_31,
		c_40_50_60_70, c_41_51_61_71,
		d_00_10_20_30, d_01_11_21_31,
		d_40_50_60_70, d_41_51_61_71;
	
	__m256i
		mask_m;
	
	// zero registers
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	c_40_51_60_71 = _mm256_setzero_pd();
	c_41_50_61_70 = _mm256_setzero_pd();

	if(kmax<=-0)
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
	
	if(kmax%4>=2)
		{
		
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
		
		
		A0 += 8;
		A1 += 8;
		B  += 8;

		}

	if(kmax%2==1)
		{
		
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0101 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0101 );
/*		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch*/
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_tmp0 );
		c_40_51_60_71 = _mm256_add_pd( c_40_51_60_71, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1010 );
/*		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch*/
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1010 );
/*		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch*/
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_tmp0 );
		c_41_50_61_70 = _mm256_add_pd( c_41_50_61_70, ab_tmp1 );
		
		}

	add:

	c_00_10_20_30 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
	c_01_11_21_31 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
	c_40_50_60_70 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0xa );
	c_41_51_61_71 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0x5 );
		
	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00_10_20_30 = _mm256_load_pd( &C0[0+ldc*0] );
		d_01_11_21_31 = _mm256_load_pd( &C0[0+ldc*1] );
		d_40_50_60_70 = _mm256_load_pd( &C1[0+ldc*0] );
		d_41_51_61_71 = _mm256_load_pd( &C1[0+ldc*1] );
		
		if(alg==1) // C += A * B'
			{
			c_00_10_20_30 = _mm256_add_pd( d_00_10_20_30, c_00_10_20_30 );
			c_01_11_21_31 = _mm256_add_pd( d_01_11_21_31, c_01_11_21_31 );
			c_40_50_60_70 = _mm256_add_pd( d_40_50_60_70, c_40_50_60_70 );
			c_41_51_61_71 = _mm256_add_pd( d_41_51_61_71, c_41_51_61_71 );
			}
		else // C -= A * B'
			{
			c_00_10_20_30 = _mm256_sub_pd( d_00_10_20_30, c_00_10_20_30 );
			c_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, c_01_11_21_31 );
			c_40_50_60_70 = _mm256_sub_pd( d_40_50_60_70, c_40_50_60_70 );
			c_41_51_61_71 = _mm256_sub_pd( d_41_51_61_71, c_41_51_61_71 );
			}

		goto store;
		}
	
	// store (5 - 8) x (1 - 2)
	store:
	d_temp = km - 4.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	_mm256_store_pd( &D0[0+ldc*0], c_00_10_20_30 );
	_mm256_maskstore_pd( &D1[0+ldc*0], mask_m, c_40_50_60_70 );
	if(kn>=2)
		{
		d_01_11_21_31 = _mm256_load_pd( &D0[0+ldc*1] );
		c_01_11_21_31 = _mm256_blend_pd( c_01_11_21_31, d_01_11_21_31, 0x1 );

		_mm256_store_pd( &D0[0+ldc*1], c_01_11_21_31 );
		_mm256_maskstore_pd( &D1[0+ldc*1], mask_m, c_41_51_61_71 );
		}

	}



// normal-transposed, 4x4 with data packed in 4
void kernel_dsyrk_nt_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;
	const int ldc = bs;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123, //A_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_temp, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31;
	
	__m256d
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33;

	__m256i
		mask_m;
	
	// zero registers
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();

	if(kmax<=0)
		goto add;

	// prefetch
	a_0123        = _mm256_load_pd( &A[0] );
	b_0123        = _mm256_load_pd( &B[0] );

	for(k=0; k<kmax-3; k+=4)
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );


/*	__builtin_prefetch( A+48 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );


/*	__builtin_prefetch( A+56 );*/
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch 
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		A += 16;
		B += 16;

		}
	
	if(kmax%4>=2)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch 
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		
		A += 8;
		B += 8;

		}

	if(kmax%2==1)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
/*		b_0123        = _mm256_load_pd( &B[4] ); // prefetch */
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		c_03_12_21_30 = _mm256_add_pd( c_03_12_21_30, ab_temp );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
/*		a_0123        = _mm256_load_pd( &A[4] ); // prefetch */
		c_02_13_20_31 = _mm256_add_pd( c_02_13_20_31, ab_temp );
		
		A += 4; // keep it !!!
		B += 4; // keep it !!!
		
		}

	add:

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
		goto store;
		}
	else 
		{
		d_00_10_20_30 = _mm256_load_pd( &C[0+ldc*0] );
		d_01_11_21_31 = _mm256_load_pd( &C[0+ldc*1] );
		d_02_12_22_32 = _mm256_load_pd( &C[0+ldc*2] );
		d_03_13_23_33 = _mm256_load_pd( &C[0+ldc*3] );
		
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

		goto store;
		}
	
	// store (1 - 4) x (3 - 4)
	store:
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	d_01_11_21_31 = _mm256_load_pd( &D[0+ldc*1] );
	d_02_12_22_32 = _mm256_load_pd( &D[0+ldc*2] );

	c_01_11_21_31 = _mm256_blend_pd( c_01_11_21_31, d_01_11_21_31, 0x1 );
	c_02_12_22_32 = _mm256_blend_pd( c_02_12_22_32, d_02_12_22_32, 0x3 );

	_mm256_maskstore_pd( &D[0+ldc*0], mask_m, c_00_10_20_30 );
	_mm256_maskstore_pd( &D[0+ldc*1], mask_m, c_01_11_21_31 );
	_mm256_maskstore_pd( &D[0+ldc*2], mask_m, c_02_12_22_32 );

	if(kn>=4)
		{
		d_03_13_23_33 = _mm256_load_pd( &D[0+ldc*3] );
		c_03_13_23_33 = _mm256_blend_pd( c_03_13_23_33, d_03_13_23_33, 0x7 );

		_mm256_maskstore_pd( &D[0+ldc*3], mask_m, c_03_13_23_33 );
		}

	}



// normal-transposed, 4x2 with data packed in 4
void kernel_dsyrk_nt_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;
	const int ldc = bs;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256d
		a_0123,
		b_0101, b_1010,
		ab_temp, // temporary results
		c_00_11_20_31, c_01_10_21_30, C_00_11_20_31, C_01_10_21_30;
	
	__m256d
		c_00_10_20_30, c_01_11_21_31,
		d_00_10_20_30, d_01_11_21_31;

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

	if(kmax%2==1)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0101 );
		c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, ab_temp );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
/*		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch*/
		ab_temp       = _mm256_mul_pd( a_0123, b_1010 );
/*		a_0123        = _mm256_load_pd( &A[4] ); // prefetch*/
		c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, ab_temp );
		
		A += 4; // keep it !!!
		B += 4; // keep it !!!

		}
		
	c_00_11_20_31 = _mm256_add_pd( c_00_11_20_31, C_00_11_20_31 );
	c_01_10_21_30 = _mm256_add_pd( c_01_10_21_30, C_01_10_21_30 );

	add:

	c_00_10_20_30 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
	c_01_11_21_31 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
		
	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00_10_20_30 = _mm256_load_pd( &C[0+ldc*0] );
		d_01_11_21_31 = _mm256_load_pd( &C[0+ldc*1] );
		
		if(alg==1) // C += A * B'
			{
			c_00_10_20_30 = _mm256_add_pd( d_00_10_20_30, c_00_10_20_30 );
			c_01_11_21_31 = _mm256_add_pd( d_01_11_21_31, c_01_11_21_31 );
			}
		else // C -= A * B'
			{
			c_00_10_20_30 = _mm256_sub_pd( d_00_10_20_30, c_00_10_20_30 );
			c_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, c_01_11_21_31 );
			}

		goto store;
		}

	// store (1 - 4) x (1 - 2)
	store:
	d_temp = km - 0.0;
	mask_m = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	_mm256_maskstore_pd( &D[0+ldc*0], mask_m, c_00_10_20_30 );

	if(kn>=2)
		{
		d_01_11_21_31 = _mm256_load_pd( &D[0+ldc*1] );

		c_01_11_21_31 = _mm256_blend_pd( c_01_11_21_31, d_01_11_21_31, 0x1 );

		_mm256_maskstore_pd( &D[0+ldc*1], mask_m, c_01_11_21_31 );
		}

	}



// normal-transposed, 2x2 with data packed in 4
void kernel_dsyrk_nt_2x2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;
	const int ldc = bs;
	
	int k;
	
	__m128d
		a_01,
		b_01, b_10,
		ab_temp, // temporary results
		c_00_11, c_01_10, C_00_11, C_01_10;
	
	__m128d
		c_00_10, c_01_11,
		d_00_10, d_01_11;

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

	if(kmax%2==1)
		{
		
		ab_temp = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, ab_temp );
		b_10    = _mm_shuffle_pd( b_01, b_01, 0x5 );
/*		b_01    = _mm_load_pd( &B[4] ); // prefetch*/
		ab_temp = _mm_mul_pd( a_01, b_10 );
/*		a_01    = _mm_load_pd( &A[4] ); // prefetch*/
		c_01_10 = _mm_add_pd( c_01_10, ab_temp );
		
		A += 4; // keep it !!!
		B += 4; // keep it !!!

		}
		


	c_00_11 = _mm_add_pd( c_00_11, C_00_11 );
	c_01_10 = _mm_add_pd( c_01_10, C_01_10 );

	add:

	c_00_10 = _mm_blend_pd( c_00_11, c_01_10, 0x2 );
	c_01_11 = _mm_blend_pd( c_00_11, c_01_10, 0x1 );
		
	if(alg==0) // C = A * B'
		{
		d_01_11 = _mm_load_pd( &D[0+ldc*1] );

		c_01_11 = _mm_blend_pd( c_01_11, d_01_11, 0x1 );

		_mm_store_pd( &D[0+ldc*0], c_00_10 );
		_mm_store_pd( &D[0+ldc*1], c_01_11 );
		}
	else 
		{
		d_00_10 = _mm_load_pd( &C[0+ldc*0] );
		d_01_11 = _mm_load_pd( &C[0+ldc*1] );
		
		if(alg==1) // C += A * B'
			{
			d_00_10 = _mm_add_pd( d_00_10, c_00_10 );
			d_01_11 = _mm_add_pd( d_01_11, c_01_11 );
			}
		else // C -= A * B'
			{
			d_00_10 = _mm_sub_pd( d_00_10, c_00_10 );
			d_01_11 = _mm_sub_pd( d_01_11, c_01_11 );
			}

		c_01_11 = _mm_load_pd( &D[0+ldc*1] );

		d_01_11 = _mm_blend_pd( d_01_11, c_01_11, 0x1 );

		_mm_store_pd( &D[0+ldc*0], d_00_10 );
		_mm_store_pd( &D[0+ldc*1], d_01_11 );
		}

	}



void kernel_dsyrk_nn_4x4_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		A += 4;
		B += 1;

		}

	double
		d_00,
		d_10, d_11,
		d_20, d_21, d_22,
		d_30, d_31, d_32, d_33;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		D[3+bs*3] = c_33;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		d_22 = C[2+bs*2];
		d_32 = C[3+bs*2];
		
		d_33 = C[3+bs*3];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;

			d_22 += c_22;
			d_32 += c_32;

			d_33 += c_33;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;

			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;

			d_22 -= c_22;
			d_32 -= c_32;

			d_33 -= c_33;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;

		D[2+bs*2] = d_22;
		D[3+bs*2] = d_32;

		D[3+bs*3] = d_33;
		}
	
	}



void kernel_dsyrk_nn_4x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0,  
		c_30=0, c_31=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 1;

		}
	double
		d_00,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;

			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;
		}

	}



void kernel_dsyrk_nn_2x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

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

		c_11 += a_1 * b_1;


		A += 4;
		B += 1;

		}
	
	double
		d_00,
		d_10, d_11;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		D[1+bs*1] = c_11;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		
		d_11 = C[1+bs*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;

			d_11 += c_11;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;

			d_11 -= c_11;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;

		D[1+bs*1] = d_11;
		}

	}

#endif


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_4_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

#if 1

	__m256d
		sign,
		a_l, a_r,
		b_00,
		c_00, c_01, c_02, c_03,
		d_00, d_01, d_02, d_03;
	
	//if(eye)
	if(0)
		{

		if(alg==0)
			{
			
			for(k=0; k<kmax-4; k+=4)
				{

				d_00 = _mm256_load_pd( &B[0] );
				d_01 = _mm256_load_pd( &B[4] );
				d_02 = _mm256_load_pd( &B[8] );
				d_03 = _mm256_load_pd( &B[12] );

				_mm256_store_pd( &D[0], d_00 );
				_mm256_store_pd( &D[4], d_01 );
				_mm256_store_pd( &D[8], d_02 );
				_mm256_store_pd( &D[12], d_03 );

				Ar += 4;
				B  += 16;
				C  += 16;
				D  += 16;
				
				}

			d_00 = _mm256_load_pd( &B[0] );
			d_01 = _mm256_load_pd( &B[4] );
			c_01 = _mm256_load_pd( &D[4] );
			d_02 = _mm256_load_pd( &B[8] );
			c_02 = _mm256_load_pd( &D[8] );
			d_03 = _mm256_load_pd( &B[12] );
			c_03 = _mm256_load_pd( &D[12] );

			_mm256_store_pd( &D[0], d_00 );
			d_01 = _mm256_blend_pd( d_01, c_01, 0x1 );
			_mm256_store_pd( &D[4], d_01 );
			d_02 = _mm256_blend_pd( d_02, c_02, 0x3 );
			_mm256_store_pd( &D[8], d_02 );
			d_03 = _mm256_blend_pd( d_03, c_03, 0x7 );
			_mm256_store_pd( &D[12], d_03 );

			}
		else
			{

			for(k=0; k<kmax-4; k+=4)
				{

				d_00 = _mm256_load_pd( &B[0] );
				d_01 = _mm256_load_pd( &B[4] );
				d_02 = _mm256_load_pd( &B[8] );
				d_03 = _mm256_load_pd( &B[12] );

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

				Ar += 4;
				B  += 16;
				C  += 16;
				D  += 16;
				
				}

			d_00 = _mm256_load_pd( &B[0] );
			d_01 = _mm256_load_pd( &B[4] );
			d_02 = _mm256_load_pd( &B[8] );
			d_03 = _mm256_load_pd( &B[12] );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_01 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_01, d_01 );
			c_01 = _mm256_load_pd( &D[4] );
			c_02 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_02, d_02 );
			c_02 = _mm256_load_pd( &D[8] );
			c_03 = _mm256_load_pd( &C[12] );
			d_03 = _mm256_add_pd( c_03, d_03 );
			c_03 = _mm256_load_pd( &D[12] );

			_mm256_store_pd( &D[0], d_00 );
			d_01 = _mm256_blend_pd( d_01, c_01, 0x1 );
			_mm256_store_pd( &D[4], d_01 );
			d_02 = _mm256_blend_pd( d_02, c_02, 0x3 );
			_mm256_store_pd( &D[8], d_02 );
			d_03 = _mm256_blend_pd( d_03, c_03, 0x7 );
			_mm256_store_pd( &D[12], d_03 );

			}

		}
	else
		{
		
		if(alg==-1)
			{
			a_l = _mm256_load_pd( &Al[0] );
			long long long_sign = 0x8000000000000000;
			sign = _mm256_broadcast_sd( (double *) &long_sign );
			a_l = _mm256_xor_pd( sign, a_l );
			}
		else
			{
			a_l = _mm256_load_pd( &Al[0] );
			}
		
		if(alg==0)
			{
			
			for(k=0; k<kmax-4; k+=4)
				{

				a_r  = _mm256_broadcast_sd( &Ar[0] );
				d_00 = _mm256_mul_pd( a_l, a_r );
				a_r  = _mm256_broadcast_sd( &Ar[1] );
				d_01 = _mm256_mul_pd( a_l, a_r );
				a_r  = _mm256_broadcast_sd( &Ar[2] );
				d_02 = _mm256_mul_pd( a_l, a_r );
				a_r  = _mm256_broadcast_sd( &Ar[3] );
				d_03 = _mm256_mul_pd( a_l, a_r );

				b_00 = _mm256_load_pd( &B[0] );
				d_00 = _mm256_mul_pd( d_00, b_00 );
				b_00 = _mm256_load_pd( &B[4] );
				d_01 = _mm256_mul_pd( d_01, b_00 );
				b_00 = _mm256_load_pd( &B[8] );
				d_02 = _mm256_mul_pd( d_02, b_00 );
				b_00 = _mm256_load_pd( &B[12] );
				d_03 = _mm256_mul_pd( d_03, b_00 );

				_mm256_store_pd( &D[0], d_00 );
				_mm256_store_pd( &D[4], d_01 );
				_mm256_store_pd( &D[8], d_02 );
				_mm256_store_pd( &D[12], d_03 );

				Ar += 4;
				B  += 16;
				C  += 16;
				D  += 16;
				
				}

			a_r  = _mm256_broadcast_sd( &Ar[0] );
			d_00 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[1] );
			d_01 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[2] );
			d_02 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[3] );
			d_03 = _mm256_mul_pd( a_l, a_r );

			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( d_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( d_01, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( d_02, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( d_03, b_00 );

			c_01 = _mm256_load_pd( &D[4] );
			c_02 = _mm256_load_pd( &D[8] );
			c_03 = _mm256_load_pd( &D[12] );

			d_01 = _mm256_blend_pd( d_01, c_01, 0x1 );
			d_02 = _mm256_blend_pd( d_02, c_02, 0x3 );
			d_03 = _mm256_blend_pd( d_03, c_03, 0x7 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );
			_mm256_store_pd( &D[12], d_03 );

			}
		else
			{

			for(k=0; k<kmax-4; k+=4)
				{

				a_r  = _mm256_broadcast_sd( &Ar[0] );
				d_00 = _mm256_mul_pd( a_l, a_r );
				a_r  = _mm256_broadcast_sd( &Ar[1] );
				d_01 = _mm256_mul_pd( a_l, a_r );
				a_r  = _mm256_broadcast_sd( &Ar[2] );
				d_02 = _mm256_mul_pd( a_l, a_r );
				a_r  = _mm256_broadcast_sd( &Ar[3] );
				d_03 = _mm256_mul_pd( a_l, a_r );

				b_00 = _mm256_load_pd( &B[0] );
				d_00 = _mm256_mul_pd( d_00, b_00 );
				b_00 = _mm256_load_pd( &B[4] );
				d_01 = _mm256_mul_pd( d_01, b_00 );
				b_00 = _mm256_load_pd( &B[8] );
				d_02 = _mm256_mul_pd( d_02, b_00 );
				b_00 = _mm256_load_pd( &B[12] );
				d_03 = _mm256_mul_pd( d_03, b_00 );

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

				Ar += 4;
				B  += 16;
				C  += 16;
				D  += 16;
				
				}

			a_r  = _mm256_broadcast_sd( &Ar[0] );
			d_00 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[1] );
			d_01 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[2] );
			d_02 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[3] );
			d_03 = _mm256_mul_pd( a_l, a_r );

			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( d_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( d_01, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( d_02, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( d_03, b_00 );

			c_00 = _mm256_load_pd( &C[0] );
			d_00 = _mm256_add_pd( c_00, d_00 );
			c_01 = _mm256_load_pd( &C[4] );
			d_01 = _mm256_add_pd( c_01, d_01 );
			c_02 = _mm256_load_pd( &C[8] );
			d_02 = _mm256_add_pd( c_02, d_02 );
			c_03 = _mm256_load_pd( &C[12] );
			d_03 = _mm256_add_pd( c_03, d_03 );

			c_01 = _mm256_load_pd( &D[4] );
			c_02 = _mm256_load_pd( &D[8] );
			c_03 = _mm256_load_pd( &D[12] );

			d_01 = _mm256_blend_pd( d_01, c_01, 0x1 );
			d_02 = _mm256_blend_pd( d_02, c_02, 0x3 );
			d_03 = _mm256_blend_pd( d_03, c_03, 0x7 );

			_mm256_store_pd( &D[0], d_00 );
			_mm256_store_pd( &D[4], d_01 );
			_mm256_store_pd( &D[8], d_02 );
			_mm256_store_pd( &D[12], d_03 );

			}

		}

#else

	double
		a_r,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		a_2 = - Al[2];
		a_3 = - Al[3];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		a_2 = Al[2];
		a_3 = Al[3];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-4; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;


		a_r = Ar[3];
		
		b_3 = B[3+bs*3];
		
		c_3 = a_3 * b_3 * a_r;

		D[3+bs*3] = c_3;

		}
	else
		{

		for(k=0; k<kmax-4; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*0] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*0] + a_3 * b_3 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*1] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*1] + a_3 * b_3 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*2] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*2] + a_3 * b_3 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*3] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*3] + a_3 * b_3 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];

		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*0] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*0] + a_3 * b_3 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*1] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*1] + a_3 * b_3 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_2 = C[2+bs*2] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*2] + a_3 * b_3 * a_r;

		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;


		a_r = Ar[3];
		
		b_3 = B[3+bs*3];
		
		c_3 = C[3+bs*3] + a_3 * b_3 * a_r;

		D[3+bs*3] = c_3;

		}
	

#endif
	
	}



// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_3_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

#if 1

	__m256i
		mask;

	__m256d
		sign,
		a_l, a_r,
		b_00,
		c_00, c_01, c_02, c_03,
		d_00, d_01, d_02, d_03;
	
	mask = _mm256_set_epi64x( 1, -1, -1, -1 );
		
	if(alg==-1)
		{
		a_l = _mm256_load_pd( &Al[0] );
		long long long_sign = 0x8000000000000000;
		sign = _mm256_broadcast_sd( (double *) &long_sign );
		a_l = _mm256_xor_pd( sign, a_l );
		}
	else
		{
		a_l = _mm256_load_pd( &Al[0] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_r  = _mm256_broadcast_sd( &Ar[0] );
			d_00 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[1] );
			d_01 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[2] );
			d_02 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[3] );
			d_03 = _mm256_mul_pd( a_l, a_r );

			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( d_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( d_01, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( d_02, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( d_03, b_00 );

			_mm256_maskstore_pd( &D[0], mask, d_00 );
			_mm256_maskstore_pd( &D[4], mask, d_01 );
			_mm256_maskstore_pd( &D[8], mask, d_02 );
			_mm256_maskstore_pd( &D[12], mask, d_03 );

			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r  = _mm256_broadcast_sd( &Ar[0] );
		d_00 = _mm256_mul_pd( a_l, a_r );
		a_r  = _mm256_broadcast_sd( &Ar[1] );
		d_01 = _mm256_mul_pd( a_l, a_r );
		a_r  = _mm256_broadcast_sd( &Ar[2] );
		d_02 = _mm256_mul_pd( a_l, a_r );

		b_00 = _mm256_load_pd( &B[0] );
		d_00 = _mm256_mul_pd( d_00, b_00 );
		b_00 = _mm256_load_pd( &B[4] );
		d_01 = _mm256_mul_pd( d_01, b_00 );
		b_00 = _mm256_load_pd( &B[8] );
		d_02 = _mm256_mul_pd( d_02, b_00 );

		c_01 = _mm256_load_pd( &D[4] );
		c_02 = _mm256_load_pd( &D[8] );

		d_01 = _mm256_blend_pd( d_01, c_01, 0x9 );
		d_02 = _mm256_blend_pd( d_02, c_02, 0xb );

		_mm256_store_pd( &D[0], d_00 );
		_mm256_store_pd( &D[4], d_01 );
		_mm256_store_pd( &D[8], d_02 );

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_r  = _mm256_broadcast_sd( &Ar[0] );
			d_00 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[1] );
			d_01 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[2] );
			d_02 = _mm256_mul_pd( a_l, a_r );
			a_r  = _mm256_broadcast_sd( &Ar[3] );
			d_03 = _mm256_mul_pd( a_l, a_r );

			b_00 = _mm256_load_pd( &B[0] );
			d_00 = _mm256_mul_pd( d_00, b_00 );
			b_00 = _mm256_load_pd( &B[4] );
			d_01 = _mm256_mul_pd( d_01, b_00 );
			b_00 = _mm256_load_pd( &B[8] );
			d_02 = _mm256_mul_pd( d_02, b_00 );
			b_00 = _mm256_load_pd( &B[12] );
			d_03 = _mm256_mul_pd( d_03, b_00 );

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

			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r  = _mm256_broadcast_sd( &Ar[0] );
		d_00 = _mm256_mul_pd( a_l, a_r );
		a_r  = _mm256_broadcast_sd( &Ar[1] );
		d_01 = _mm256_mul_pd( a_l, a_r );
		a_r  = _mm256_broadcast_sd( &Ar[2] );
		d_02 = _mm256_mul_pd( a_l, a_r );

		b_00 = _mm256_load_pd( &B[0] );
		d_00 = _mm256_mul_pd( d_00, b_00 );
		b_00 = _mm256_load_pd( &B[4] );
		d_01 = _mm256_mul_pd( d_01, b_00 );
		b_00 = _mm256_load_pd( &B[8] );
		d_02 = _mm256_mul_pd( d_02, b_00 );

		c_00 = _mm256_load_pd( &C[0] );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_01 = _mm256_load_pd( &C[4] );
		d_01 = _mm256_add_pd( c_01, d_01 );
		c_02 = _mm256_load_pd( &C[8] );
		d_02 = _mm256_add_pd( c_02, d_02 );

		c_01 = _mm256_load_pd( &D[4] );
		c_02 = _mm256_load_pd( &D[8] );

		d_01 = _mm256_blend_pd( d_01, c_01, 0x9 );
		d_02 = _mm256_blend_pd( d_02, c_02, 0xb );

		_mm256_store_pd( &D[0], d_00 );
		_mm256_store_pd( &D[4], d_01 );
		_mm256_store_pd( &D[8], d_02 );

		}
#else

	double
		a_r,
		a_0, a_1, a_2,
		b_0, b_1, b_2,
		c_0, c_1, c_2;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		a_2 = - Al[2];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		a_2 = Al[2];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		
		c_2 = a_2 * b_2 * a_r;

		D[2+bs*2] = c_2;

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*0] + a_2 * b_2 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*1] + a_2 * b_2 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*2] + a_2 * b_2 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*3] + a_2 * b_2 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
	

			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*0] + a_2 * b_2 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*1] + a_2 * b_2 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		
		c_2 = C[2+bs*2] + a_2 * b_2 * a_r;

		D[2+bs*2] = c_2;

		}

#endif
	
	}


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_2_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

#if 1

	__m128d
		sign,
		a_l, a_r,
		b_00,
		c_00, c_01, c_02, c_03,
		d_00, d_01, d_02, d_03;
		
	if(alg==-1)
		{
		a_l = _mm_load_pd( &Al[0] );
		long long long_sign = 0x8000000000000000;
		sign = _mm_loaddup_pd( (double *) &long_sign );
		a_l = _mm_xor_pd( sign, a_l );
		}
	else
		{
		a_l = _mm_load_pd( &Al[0] );
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-2; k+=4)
			{

			a_r  = _mm_loaddup_pd( &Ar[0] );
			d_00 = _mm_mul_pd( a_l, a_r );
			a_r  = _mm_loaddup_pd( &Ar[1] );
			d_01 = _mm_mul_pd( a_l, a_r );
			a_r  = _mm_loaddup_pd( &Ar[2] );
			d_02 = _mm_mul_pd( a_l, a_r );
			a_r  = _mm_loaddup_pd( &Ar[3] );
			d_03 = _mm_mul_pd( a_l, a_r );

			b_00 = _mm_load_pd( &B[0] );
			d_00 = _mm_mul_pd( d_00, b_00 );
			b_00 = _mm_load_pd( &B[4] );
			d_01 = _mm_mul_pd( d_01, b_00 );
			b_00 = _mm_load_pd( &B[8] );
			d_02 = _mm_mul_pd( d_02, b_00 );
			b_00 = _mm_load_pd( &B[12] );
			d_03 = _mm_mul_pd( d_03, b_00 );

			_mm_store_pd( &D[0], d_00 );
			_mm_store_pd( &D[4], d_01 );
			_mm_store_pd( &D[8], d_02 );
			_mm_store_pd( &D[12], d_03 );

			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r  = _mm_loaddup_pd( &Ar[0] );
		d_00 = _mm_mul_pd( a_l, a_r );
		a_r  = _mm_loaddup_pd( &Ar[1] );
		d_01 = _mm_mul_pd( a_l, a_r );

		b_00 = _mm_load_pd( &B[0] );
		d_00 = _mm_mul_pd( d_00, b_00 );
		b_00 = _mm_load_pd( &B[4] );
		d_01 = _mm_mul_pd( d_01, b_00 );

		c_01 = _mm_load_pd( &D[4] );

		d_01 = _mm_blend_pd( d_01, c_01, 0x1 );

		_mm_store_pd( &D[0], d_00 );
		_mm_store_pd( &D[4], d_01 );

		}
	else
		{

		for(k=0; k<kmax-2; k+=4)
			{

			a_r  = _mm_loaddup_pd( &Ar[0] );
			d_00 = _mm_mul_pd( a_l, a_r );
			a_r  = _mm_loaddup_pd( &Ar[1] );
			d_01 = _mm_mul_pd( a_l, a_r );
			a_r  = _mm_loaddup_pd( &Ar[2] );
			d_02 = _mm_mul_pd( a_l, a_r );
			a_r  = _mm_loaddup_pd( &Ar[3] );
			d_03 = _mm_mul_pd( a_l, a_r );

			b_00 = _mm_load_pd( &B[0] );
			d_00 = _mm_mul_pd( d_00, b_00 );
			b_00 = _mm_load_pd( &B[4] );
			d_01 = _mm_mul_pd( d_01, b_00 );
			b_00 = _mm_load_pd( &B[8] );
			d_02 = _mm_mul_pd( d_02, b_00 );
			b_00 = _mm_load_pd( &B[12] );
			d_03 = _mm_mul_pd( d_03, b_00 );

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

			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r  = _mm_loaddup_pd( &Ar[0] );
		d_00 = _mm_mul_pd( a_l, a_r );
		a_r  = _mm_loaddup_pd( &Ar[1] );
		d_01 = _mm_mul_pd( a_l, a_r );

		b_00 = _mm_load_pd( &B[0] );
		d_00 = _mm_mul_pd( d_00, b_00 );
		b_00 = _mm_load_pd( &B[4] );
		d_01 = _mm_mul_pd( d_01, b_00 );

		c_00 = _mm_load_pd( &C[0] );
		d_00 = _mm_add_pd( c_00, d_00 );
		c_01 = _mm_load_pd( &C[4] );
		d_01 = _mm_add_pd( c_01, d_01 );

		c_01 = _mm_load_pd( &D[4] );

		d_01 = _mm_blend_pd( d_01, c_01, 0x1 );

		_mm_store_pd( &D[0], d_00 );
		_mm_store_pd( &D[4], d_01 );

		}
#else


	double
		a_r,
		a_0, a_1,
		b_0, b_1,
		c_0, c_1;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		
		c_1 = a_1 * b_1 * a_r;

		D[1+bs*1] = c_1;

		}
	else
		{

		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;

		D[1+bs*1] = c_1;

		}

#endif
	
	}


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_1_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0,
		b_0,
		c_0;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		}
	else
		{
		a_0 = Al[0];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-1; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*0] = c_0;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*1] = c_0;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*2] = c_0;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*3] = c_0;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		
		c_0 = a_0 * b_0 * a_r;

		D[0+bs*0] = c_0;

		}
	else
		{

		for(k=0; k<kmax-1; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;

			D[0+bs*0] = c_0;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;

			D[0+bs*1] = c_0;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;

			D[0+bs*2] = c_0;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;

			D[0+bs*3] = c_0;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;

		D[0+bs*0] = c_0;

		}
	
	}


#if ! defined(BLASFEO)

// rank 0 update
void kernel_dsyr0_4_lib4(int kmax, int tri, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		c_0,
		d_0;
	
	zero = _mm256_setzero_pd();

	kmax = tri<kmax ? tri : kmax;

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			_mm256_store_pd( &D[0], zero );
			_mm256_store_pd( &D[4], zero );
			_mm256_store_pd( &D[8], zero );
			_mm256_store_pd( &D[12], zero );

			D += 16;
			}
		for(; k<kmax; k++)
			{
			_mm256_store_pd( &D[0], zero );

			D += 4;
			}
		if(tri-k>0)
			{
			_mm256_store_pd( &D[0], zero );
			if(tri-k>1)
				{
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( zero, d_0, 0x1 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( zero, d_0, 0x3 );
					_mm256_store_pd( &D[8], d_0 );
					if(tri-k>3)
						{
						d_0 = _mm256_load_pd( &D[12] );
						d_0 = _mm256_blend_pd( zero, d_0, 0x7 );
						_mm256_store_pd( &D[12], d_0 );
						}
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			c_0 = _mm256_load_pd( &C[0] );
			_mm256_store_pd( &D[0], c_0 );
			c_0 = _mm256_load_pd( &C[4] );
			_mm256_store_pd( &D[4], c_0 );
			c_0 = _mm256_load_pd( &C[8] );
			_mm256_store_pd( &D[8], c_0 );
			c_0 = _mm256_load_pd( &C[12] );
			_mm256_store_pd( &D[12], c_0 );

			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			c_0 = _mm256_load_pd( &C[0] );
			_mm256_store_pd( &D[0], c_0 );

			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			c_0 = _mm256_load_pd( &C[0] );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				c_0 = _mm256_load_pd( &C[4] );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( c_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					c_0 = _mm256_load_pd( &C[8] );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( c_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], d_0 );
					if(tri-k>3)
						{
						c_0 = _mm256_load_pd( &C[12] );
						d_0 = _mm256_load_pd( &D[12] );
						d_0 = _mm256_blend_pd( c_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], d_0 );
						}
					}
				}
			}

		}
	
	}



void kernel_dsyr0_3_lib4(int kmax, int tri, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		c_0,
		d_0;
	
	zero = _mm256_setzero_pd();

	kmax = tri<kmax ? tri : kmax;

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			d_0 = _mm256_load_pd( &D[0] );
			d_0 = _mm256_blend_pd( d_0, zero, 0x7 );
			_mm256_store_pd( &D[0], d_0 );
			d_0 = _mm256_load_pd( &D[4] );
			d_0 = _mm256_blend_pd( d_0, zero, 0x7 );
			_mm256_store_pd( &D[4], d_0 );
			d_0 = _mm256_load_pd( &D[8] );
			d_0 = _mm256_blend_pd( d_0, zero, 0x7 );
			_mm256_store_pd( &D[8], d_0 );
			d_0 = _mm256_load_pd( &D[12] );
			d_0 = _mm256_blend_pd( d_0, zero, 0x7 );
			_mm256_store_pd( &D[12], d_0 );

			D += 16;
			}
		for(; k<kmax; k++)
			{
			d_0 = _mm256_load_pd( &D[0] );
			d_0 = _mm256_blend_pd( d_0, zero, 0x7 );
			_mm256_store_pd( &D[0], d_0 );

			D += 4;
			}
		if(tri-k>0)
			{
			d_0 = _mm256_load_pd( &D[0] );
			d_0 = _mm256_blend_pd( d_0, zero, 0x7 );
			_mm256_store_pd( &D[0], d_0 );
			if(tri-k>1)
				{
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( zero, d_0, 0x9 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( zero, d_0, 0xb );
					_mm256_store_pd( &D[8], d_0 );
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			c_0 = _mm256_load_pd( &C[0] );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );
			c_0 = _mm256_load_pd( &C[4] );
			d_0 = _mm256_load_pd( &D[4] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[4], c_0 );
			c_0 = _mm256_load_pd( &C[8] );
			d_0 = _mm256_load_pd( &D[8] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[8], c_0 );
			c_0 = _mm256_load_pd( &C[12] );
			d_0 = _mm256_load_pd( &D[12] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[12], c_0 );

			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			c_0 = _mm256_load_pd( &C[0] );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			c_0 = _mm256_load_pd( &C[0] );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				c_0 = _mm256_load_pd( &C[4] );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					c_0 = _mm256_load_pd( &C[8] );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0xb );
					_mm256_store_pd( &D[8], c_0 );
					}
				}
			}

		}
	
	}



void kernel_dsyr0_2_lib4(int kmax, int tri, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		c_0,
		d_0;
	
	zero = _mm_setzero_pd();

	kmax = tri<kmax ? tri : kmax;

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			_mm_store_pd( &D[0], zero );
			_mm_store_pd( &D[4], zero );
			_mm_store_pd( &D[8], zero );
			_mm_store_pd( &D[12], zero );

			D += 16;
			}
		for(; k<kmax; k++)
			{
			_mm_store_pd( &D[0], zero );

			D += 4;
			}
		if(tri-k>0)
			{
			_mm_store_pd( &D[0], zero );
			if(tri-k>1)
				{
				_mm_storeh_pd( &D[5], d_0 );
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			c_0 = _mm_load_pd( &C[0] );
			_mm_store_pd( &D[0], c_0 );
			c_0 = _mm_load_pd( &C[4] );
			_mm_store_pd( &D[4], c_0 );
			c_0 = _mm_load_pd( &C[8] );
			_mm_store_pd( &D[8], c_0 );
			c_0 = _mm_load_pd( &C[12] );
			_mm_store_pd( &D[12], c_0 );

			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			c_0 = _mm_load_pd( &C[0] );
			_mm_store_pd( &D[0], c_0 );

			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			c_0 = _mm_load_pd( &C[0] );
			_mm_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				c_0 = _mm_load_pd( &C[4] );
				_mm_storeh_pd( &D[5], c_0 );
				}
			}

		}
	
	}



void kernel_dsyr0_1_lib4(int kmax, int tri, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		c_0,
		d_0;
	
	zero = _mm_setzero_pd();

	kmax = tri<kmax ? tri : kmax;

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			_mm_store_sd( &D[0], zero );
			_mm_store_sd( &D[4], zero );
			_mm_store_sd( &D[8], zero );
			_mm_store_sd( &D[12], zero );

			D += 16;
			}
		for(; k<kmax; k++)
			{
			_mm_store_sd( &D[0], zero );

			D += 4;
			}
		if(tri-k>0)
			{
			_mm_store_sd( &D[0], zero );
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			c_0 = _mm_load_sd( &C[0] );
			_mm_store_sd( &D[0], c_0 );
			c_0 = _mm_load_sd( &C[4] );
			_mm_store_sd( &D[4], c_0 );
			c_0 = _mm_load_sd( &C[8] );
			_mm_store_sd( &D[8], c_0 );
			c_0 = _mm_load_sd( &C[12] );
			_mm_store_sd( &D[12], c_0 );

			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			c_0 = _mm_load_sd( &C[0] );
			_mm_store_sd( &D[0], c_0 );

			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			c_0 = _mm_load_sd( &C[0] );
			_mm_store_sd( &D[0], c_0 );
			}

		}
	
	}



// rank 1 update
void kernel_dsyr1_4_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0,
		b_0,
		c_0,
		d_0,
		t_0;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], d_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						d_0 = _mm256_load_pd( &D[12] );
						d_0 = _mm256_blend_pd( t_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], d_0 );
						}
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], c_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						c_0 = _mm256_load_pd( &C[12] );
						c_0 = _mm256_add_pd( c_0, t_0 );
						d_0 = _mm256_load_pd( &D[12] );
						c_0 = _mm256_blend_pd( c_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], c_0 );
						}
					}
				}
			}

		}
	
	}



void kernel_dsyr1_3_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0,
		b_0, b_1,
		c_0,
		d_0,
		t_0;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_load_pd( &D[4] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_load_pd( &D[8] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_load_pd( &D[12] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0xb );
					_mm256_store_pd( &D[8], d_0 );
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[4] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[8] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[12] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0xb );
					_mm256_store_pd( &D[8], c_0 );
					}
				}
			}

		}
	
	}



void kernel_dsyr1_2_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0,
		b_0,
		c_0,
		d_0,
		t_0;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_pd( &A[0] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_pd( zero, a_0 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			_mm_store_pd( &D[0], t_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			_mm_store_pd( &D[4], t_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			_mm_store_pd( &D[8], t_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			_mm_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			_mm_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			_mm_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				_mm_storeh_pd( &D[5], d_0 );
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[4] );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[4], c_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[8] );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[8], c_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[12] );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				c_0 = _mm_load_pd( &C[4] );
				c_0 = _mm_add_pd( c_0, t_0 );
				_mm_storeh_pd( &D[5], c_0 );
				}
			}

		}
	
	}



void kernel_dsyr1_1_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0,
		b_0,
		c_0,
		d_0,
		t_0;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_sd( &A[0] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_sd( zero, a_0 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			_mm_store_sd( &D[0], t_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			_mm_store_sd( &D[4], t_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			_mm_store_sd( &D[8], t_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			_mm_store_sd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			_mm_store_sd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			_mm_store_sd( &D[0], t_0 );
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[4] );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[4], c_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[8] );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[8], c_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[12] );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );
			}

		}
	
	}



// rank 2 update
void kernel_dsyr2_4_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0, a_1,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	a_1 = _mm256_load_pd( &A[4] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		a_1 = _mm256_sub_pd( zero, a_1 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_1 = _mm256_mul_pd( a_1, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_1 = _mm256_mul_pd( a_1, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], d_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						b_0 = _mm256_broadcast_sd( &B[7] );
						t_1 = _mm256_mul_pd( a_1, b_0 );
						t_0 = _mm256_add_pd( t_0, t_1 );
						d_0 = _mm256_load_pd( &D[12] );
						d_0 = _mm256_blend_pd( t_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], d_0 );
						}
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_0 = _mm256_mul_pd( a_1, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_0 = _mm256_mul_pd( a_1, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], c_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						c_0 = _mm256_load_pd( &C[12] );
						c_0 = _mm256_add_pd( c_0, t_0 );
						b_0 = _mm256_broadcast_sd( &B[7] );
						t_0 = _mm256_mul_pd( a_1, b_0 );
						c_0 = _mm256_add_pd( c_0, t_0 );
						d_0 = _mm256_load_pd( &D[12] );
						c_0 = _mm256_blend_pd( c_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], c_0 );
						}
					}
				}
			}

		}
	
	}



void kernel_dsyr2_3_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0, a_1,
		b_0, 
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	a_1 = _mm256_load_pd( &A[4] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		a_1 = _mm256_sub_pd( zero, a_1 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[4] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[8] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[12] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_1 = _mm256_mul_pd( a_1, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_1 = _mm256_mul_pd( a_1, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0xb );
					_mm256_store_pd( &D[8], d_0 );
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[4] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[8] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[12] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_0 = _mm256_mul_pd( a_1, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_0 = _mm256_mul_pd( a_1, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0xb );
					_mm256_store_pd( &D[8], c_0 );
					}
				}
			}

		}
	
	}



void kernel_dsyr2_2_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0, a_1,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_pd( &A[0] );
	a_1 = _mm_load_pd( &A[4] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_pd( zero, a_0 );
		a_1 = _mm_sub_pd( zero, a_1 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[5] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[4], t_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[6] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[8], t_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[7] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				b_0 = _mm_loaddup_pd( &B[5] );
				t_1 = _mm_mul_pd( a_1, b_0 );
				t_0 = _mm_add_pd( t_0, t_1 );
				_mm_storeh_pd( &D[5], d_0 );
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[4] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[5] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[4], c_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[8] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[6] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[8], c_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[12] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[7] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				c_0 = _mm_load_pd( &C[4] );
				c_0 = _mm_add_pd( c_0, t_0 );
				b_0 = _mm_loaddup_pd( &B[5] );
				t_0 = _mm_mul_pd( a_1, b_0 );
				c_0 = _mm_add_pd( c_0, t_0 );
				_mm_storeh_pd( &D[5], c_0 );
				}
			}

		}
	
	}



void kernel_dsyr2_1_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0, a_1,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_sd( &A[0] );
	a_1 = _mm_load_sd( &A[4] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_sd( zero, a_0 );
		a_1 = _mm_sub_sd( zero, a_1 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[5] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[4], t_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[6] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[8], t_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[7] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[4] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[5] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[4], c_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[8] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[6] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[8], c_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[12] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[7] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );
			}

		}
	
	}



// rank 3 update
void kernel_dsyr3_4_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0, a_1, a_2,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	a_1 = _mm256_load_pd( &A[4] );
	a_2 = _mm256_load_pd( &A[8] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		a_1 = _mm256_sub_pd( zero, a_1 );
		a_2 = _mm256_sub_pd( zero, a_2 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_1 = _mm256_mul_pd( a_1, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_1 = _mm256_mul_pd( a_2, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_1 = _mm256_mul_pd( a_1, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_1 = _mm256_mul_pd( a_2, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], d_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						b_0 = _mm256_broadcast_sd( &B[7] );
						t_1 = _mm256_mul_pd( a_1, b_0 );
						t_0 = _mm256_add_pd( t_0, t_1 );
						b_0 = _mm256_broadcast_sd( &B[11] );
						t_1 = _mm256_mul_pd( a_2, b_0 );
						t_0 = _mm256_add_pd( t_0, t_1 );
						d_0 = _mm256_load_pd( &D[12] );
						d_0 = _mm256_blend_pd( t_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], d_0 );
						}
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_0 = _mm256_mul_pd( a_1, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_0 = _mm256_mul_pd( a_2, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_0 = _mm256_mul_pd( a_1, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_0 = _mm256_mul_pd( a_2, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], c_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						c_0 = _mm256_load_pd( &C[12] );
						c_0 = _mm256_add_pd( c_0, t_0 );
						b_0 = _mm256_broadcast_sd( &B[7] );
						t_0 = _mm256_mul_pd( a_1, b_0 );
						c_0 = _mm256_add_pd( c_0, t_0 );
						b_0 = _mm256_broadcast_sd( &B[11] );
						t_0 = _mm256_mul_pd( a_2, b_0 );
						c_0 = _mm256_add_pd( c_0, t_0 );
						d_0 = _mm256_load_pd( &D[12] );
						c_0 = _mm256_blend_pd( c_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], c_0 );
						}
					}
				}
			}

		}
	
	}



void kernel_dsyr3_3_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0, a_1, a_2,
		b_0, 
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	a_1 = _mm256_load_pd( &A[4] );
	a_2 = _mm256_load_pd( &A[8] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		a_1 = _mm256_sub_pd( zero, a_1 );
		a_2 = _mm256_sub_pd( zero, a_2 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[4] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[8] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[12] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_1 = _mm256_mul_pd( a_1, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_1 = _mm256_mul_pd( a_2, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_1 = _mm256_mul_pd( a_1, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_1 = _mm256_mul_pd( a_2, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0xb );
					_mm256_store_pd( &D[8], d_0 );
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[4] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[8] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[12] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_0 = _mm256_mul_pd( a_1, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_0 = _mm256_mul_pd( a_2, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_0 = _mm256_mul_pd( a_1, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_0 = _mm256_mul_pd( a_2, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0xb );
					_mm256_store_pd( &D[8], c_0 );
					}
				}
			}

		}
	
	}



void kernel_dsyr3_2_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0, a_1, a_2,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_pd( &A[0] );
	a_1 = _mm_load_pd( &A[4] );
	a_2 = _mm_load_pd( &A[8] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_pd( zero, a_0 );
		a_1 = _mm_sub_pd( zero, a_1 );
		a_2 = _mm_sub_pd( zero, a_2 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[5] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[9] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[4], t_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[6] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[10] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[8], t_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[7] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[11] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				b_0 = _mm_loaddup_pd( &B[5] );
				t_1 = _mm_mul_pd( a_1, b_0 );
				t_0 = _mm_add_pd( t_0, t_1 );
				b_0 = _mm_loaddup_pd( &B[9] );
				t_1 = _mm_mul_pd( a_2, b_0 );
				t_0 = _mm_add_pd( t_0, t_1 );
				_mm_storeh_pd( &D[5], d_0 );
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[4] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[5] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[9] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[4], c_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[8] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[6] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[10] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[8], c_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[12] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[7] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[11] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				c_0 = _mm_load_pd( &C[4] );
				c_0 = _mm_add_pd( c_0, t_0 );
				b_0 = _mm_loaddup_pd( &B[5] );
				t_0 = _mm_mul_pd( a_1, b_0 );
				c_0 = _mm_add_pd( c_0, t_0 );
				b_0 = _mm_loaddup_pd( &B[9] );
				t_0 = _mm_mul_pd( a_2, b_0 );
				c_0 = _mm_add_pd( c_0, t_0 );
				_mm_storeh_pd( &D[5], c_0 );
				}
			}

		}
	
	}



void kernel_dsyr3_1_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0, a_1, a_2,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_sd( &A[0] );
	a_1 = _mm_load_sd( &A[4] );
	a_2 = _mm_load_sd( &A[8] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_sd( zero, a_0 );
		a_1 = _mm_sub_sd( zero, a_1 );
		a_2 = _mm_sub_sd( zero, a_2 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[8] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[5] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[9] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[4], t_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[6] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[10] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[8], t_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[7] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[11] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[8] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[8] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[8] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[4] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[5] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[9] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[4], c_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[8] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[6] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[10] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[8], c_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[12] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[7] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[11] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[8] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[8] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );
			}

		}
	
	}



// rank 4 update
void kernel_dsyr4_4_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0, a_1, a_2, a_3,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	a_1 = _mm256_load_pd( &A[4] );
	a_2 = _mm256_load_pd( &A[8] );
	a_3 = _mm256_load_pd( &A[12] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		a_1 = _mm256_sub_pd( zero, a_1 );
		a_2 = _mm256_sub_pd( zero, a_2 );
		a_3 = _mm256_sub_pd( zero, a_3 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[13] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[14] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[15] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_1 = _mm256_mul_pd( a_1, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_1 = _mm256_mul_pd( a_2, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				b_0 = _mm256_broadcast_sd( &B[13] );
				t_1 = _mm256_mul_pd( a_3, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_1 = _mm256_mul_pd( a_1, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_1 = _mm256_mul_pd( a_2, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					b_0 = _mm256_broadcast_sd( &B[14] );
					t_1 = _mm256_mul_pd( a_3, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], d_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						b_0 = _mm256_broadcast_sd( &B[7] );
						t_1 = _mm256_mul_pd( a_1, b_0 );
						t_0 = _mm256_add_pd( t_0, t_1 );
						b_0 = _mm256_broadcast_sd( &B[11] );
						t_1 = _mm256_mul_pd( a_2, b_0 );
						t_0 = _mm256_add_pd( t_0, t_1 );
						b_0 = _mm256_broadcast_sd( &B[15] );
						t_1 = _mm256_mul_pd( a_3, b_0 );
						t_0 = _mm256_add_pd( t_0, t_1 );
						d_0 = _mm256_load_pd( &D[12] );
						d_0 = _mm256_blend_pd( t_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], d_0 );
						}
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[13] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[14] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[15] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_0 = _mm256_mul_pd( a_1, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_0 = _mm256_mul_pd( a_2, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[13] );
				t_0 = _mm256_mul_pd( a_3, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x1 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_0 = _mm256_mul_pd( a_1, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_0 = _mm256_mul_pd( a_2, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[14] );
					t_0 = _mm256_mul_pd( a_3, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0x3 );
					_mm256_store_pd( &D[8], c_0 );
					if(tri-k>3)
						{
						b_0 = _mm256_broadcast_sd( &B[3] );
						t_0 = _mm256_mul_pd( a_0, b_0 );
						c_0 = _mm256_load_pd( &C[12] );
						c_0 = _mm256_add_pd( c_0, t_0 );
						b_0 = _mm256_broadcast_sd( &B[7] );
						t_0 = _mm256_mul_pd( a_1, b_0 );
						c_0 = _mm256_add_pd( c_0, t_0 );
						b_0 = _mm256_broadcast_sd( &B[11] );
						t_0 = _mm256_mul_pd( a_2, b_0 );
						c_0 = _mm256_add_pd( c_0, t_0 );
						b_0 = _mm256_broadcast_sd( &B[15] );
						t_0 = _mm256_mul_pd( a_3, b_0 );
						c_0 = _mm256_add_pd( c_0, t_0 );
						d_0 = _mm256_load_pd( &D[12] );
						c_0 = _mm256_blend_pd( c_0, d_0, 0x7 );
						_mm256_store_pd( &D[12], c_0 );
						}
					}
				}
			}

		}
	
	}



void kernel_dsyr4_3_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zero,
		a_0, a_1, a_2, a_3,
		b_0, 
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm256_load_pd( &A[0] );
	a_1 = _mm256_load_pd( &A[4] );
	a_2 = _mm256_load_pd( &A[8] );
	a_3 = _mm256_load_pd( &A[12] );
	if(alg==-1)
		{
		zero = _mm256_setzero_pd();
		a_0 = _mm256_sub_pd( zero, a_0 );
		a_1 = _mm256_sub_pd( zero, a_1 );
		a_2 = _mm256_sub_pd( zero, a_2 );
		a_3 = _mm256_sub_pd( zero, a_3 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[13] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[4] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[4], t_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[14] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[8] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[8], t_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[15] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[12] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_1 = _mm256_mul_pd( a_1, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_1 = _mm256_mul_pd( a_2, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_1 = _mm256_mul_pd( a_3, b_0 );
			t_0 = _mm256_add_pd( t_0, t_1 );
			d_0 = _mm256_load_pd( &D[0] );
			t_0 = _mm256_blend_pd( d_0, t_0, 0x7 );
			_mm256_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_1 = _mm256_mul_pd( a_1, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_1 = _mm256_mul_pd( a_2, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				b_0 = _mm256_broadcast_sd( &B[13] );
				t_1 = _mm256_mul_pd( a_3, b_0 );
				t_0 = _mm256_add_pd( t_0, t_1 );
				d_0 = _mm256_load_pd( &D[4] );
				d_0 = _mm256_blend_pd( t_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], d_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_1 = _mm256_mul_pd( a_1, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_1 = _mm256_mul_pd( a_2, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					b_0 = _mm256_broadcast_sd( &B[14] );
					t_1 = _mm256_mul_pd( a_3, b_0 );
					t_0 = _mm256_add_pd( t_0, t_1 );
					d_0 = _mm256_load_pd( &D[8] );
					d_0 = _mm256_blend_pd( t_0, d_0, 0xb );
					_mm256_store_pd( &D[8], d_0 );
					}
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			b_0 = _mm256_broadcast_sd( &B[1] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[4] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[5] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[9] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[13] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[4] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[4], c_0 );

			b_0 = _mm256_broadcast_sd( &B[2] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[8] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[6] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[10] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[14] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[8] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[8], c_0 );

			b_0 = _mm256_broadcast_sd( &B[3] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[12] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[7] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[11] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[15] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[12] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm256_broadcast_sd( &B[0] );
			t_0 = _mm256_mul_pd( a_0, b_0 );
			c_0 = _mm256_load_pd( &C[0] );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[4] );
			t_0 = _mm256_mul_pd( a_1, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[8] );
			t_0 = _mm256_mul_pd( a_2, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			b_0 = _mm256_broadcast_sd( &B[12] );
			t_0 = _mm256_mul_pd( a_3, b_0 );
			c_0 = _mm256_add_pd( c_0, t_0 );
			d_0 = _mm256_load_pd( &D[0] );
			c_0 = _mm256_blend_pd( d_0, c_0, 0x7 );
			_mm256_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm256_broadcast_sd( &B[1] );
				t_0 = _mm256_mul_pd( a_0, b_0 );
				c_0 = _mm256_load_pd( &C[4] );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[5] );
				t_0 = _mm256_mul_pd( a_1, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[9] );
				t_0 = _mm256_mul_pd( a_2, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				b_0 = _mm256_broadcast_sd( &B[13] );
				t_0 = _mm256_mul_pd( a_3, b_0 );
				c_0 = _mm256_add_pd( c_0, t_0 );
				d_0 = _mm256_load_pd( &D[4] );
				c_0 = _mm256_blend_pd( c_0, d_0, 0x9 );
				_mm256_store_pd( &D[4], c_0 );
				if(tri-k>2)
					{
					b_0 = _mm256_broadcast_sd( &B[2] );
					t_0 = _mm256_mul_pd( a_0, b_0 );
					c_0 = _mm256_load_pd( &C[8] );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[6] );
					t_0 = _mm256_mul_pd( a_1, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[10] );
					t_0 = _mm256_mul_pd( a_2, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					b_0 = _mm256_broadcast_sd( &B[14] );
					t_0 = _mm256_mul_pd( a_3, b_0 );
					c_0 = _mm256_add_pd( c_0, t_0 );
					d_0 = _mm256_load_pd( &D[8] );
					c_0 = _mm256_blend_pd( c_0, d_0, 0xb );
					_mm256_store_pd( &D[8], c_0 );
					}
				}
			}

		}
	
	}



void kernel_dsyr4_2_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0, a_1, a_2, a_3,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_pd( &A[0] );
	a_1 = _mm_load_pd( &A[4] );
	a_2 = _mm_load_pd( &A[8] );
	a_3 = _mm_load_pd( &A[12] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_pd( zero, a_0 );
		a_1 = _mm_sub_pd( zero, a_1 );
		a_2 = _mm_sub_pd( zero, a_2 );
		a_3 = _mm_sub_pd( zero, a_3 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[12] );
			t_1 = _mm_mul_pd( a_3, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[5] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[9] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[13] );
			t_1 = _mm_mul_pd( a_3, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[4], t_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[6] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[10] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[14] );
			t_1 = _mm_mul_pd( a_3, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[8], t_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[7] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[11] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[15] );
			t_1 = _mm_mul_pd( a_3, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[12] );
			t_1 = _mm_mul_pd( a_3, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_1 = _mm_mul_pd( a_1, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_1 = _mm_mul_pd( a_2, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			b_0 = _mm_loaddup_pd( &B[12] );
			t_1 = _mm_mul_pd( a_3, b_0 );
			t_0 = _mm_add_pd( t_0, t_1 );
			_mm_store_pd( &D[0], t_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				b_0 = _mm_loaddup_pd( &B[5] );
				t_1 = _mm_mul_pd( a_1, b_0 );
				t_0 = _mm_add_pd( t_0, t_1 );
				b_0 = _mm_loaddup_pd( &B[9] );
				t_1 = _mm_mul_pd( a_2, b_0 );
				t_0 = _mm_add_pd( t_0, t_1 );
				b_0 = _mm_loaddup_pd( &B[13] );
				t_1 = _mm_mul_pd( a_3, b_0 );
				t_0 = _mm_add_pd( t_0, t_1 );
				_mm_storeh_pd( &D[5], d_0 );
				}
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[12] );
			t_0 = _mm_mul_pd( a_3, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			b_0 = _mm_loaddup_pd( &B[1] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[4] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[5] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[9] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[13] );
			t_0 = _mm_mul_pd( a_3, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[4], c_0 );

			b_0 = _mm_loaddup_pd( &B[2] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[8] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[6] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[10] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[14] );
			t_0 = _mm_mul_pd( a_3, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[8], c_0 );

			b_0 = _mm_loaddup_pd( &B[3] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[12] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[7] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[11] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[15] );
			t_0 = _mm_mul_pd( a_3, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[12] );
			t_0 = _mm_mul_pd( a_3, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_loaddup_pd( &B[0] );
			t_0 = _mm_mul_pd( a_0, b_0 );
			c_0 = _mm_load_pd( &C[0] );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[4] );
			t_0 = _mm_mul_pd( a_1, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[8] );
			t_0 = _mm_mul_pd( a_2, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			b_0 = _mm_loaddup_pd( &B[12] );
			t_0 = _mm_mul_pd( a_3, b_0 );
			c_0 = _mm_add_pd( c_0, t_0 );
			_mm_store_pd( &D[0], c_0 );
			if(tri-k>1)
				{
				b_0 = _mm_loaddup_pd( &B[1] );
				t_0 = _mm_mul_pd( a_0, b_0 );
				c_0 = _mm_load_pd( &C[4] );
				c_0 = _mm_add_pd( c_0, t_0 );
				b_0 = _mm_loaddup_pd( &B[5] );
				t_0 = _mm_mul_pd( a_1, b_0 );
				c_0 = _mm_add_pd( c_0, t_0 );
				b_0 = _mm_loaddup_pd( &B[9] );
				t_0 = _mm_mul_pd( a_2, b_0 );
				c_0 = _mm_add_pd( c_0, t_0 );
				b_0 = _mm_loaddup_pd( &B[13] );
				t_0 = _mm_mul_pd( a_3, b_0 );
				c_0 = _mm_add_pd( c_0, t_0 );
				_mm_storeh_pd( &D[5], c_0 );
				}
			}

		}
	
	}



void kernel_dsyr4_1_lib4(int kmax, int tri, double *A, double *B, int sdb, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m128d
		zero,
		a_0, a_1, a_2, a_3,
		b_0,
		c_0,
		d_0,
		t_0, t_1;
	
	kmax = tri<kmax ? tri : kmax;

	a_0 = _mm_load_sd( &A[0] );
	a_1 = _mm_load_sd( &A[4] );
	a_2 = _mm_load_sd( &A[8] );
	a_3 = _mm_load_sd( &A[12] );
	if(alg==-1)
		{
		zero = _mm_setzero_pd();
		a_0 = _mm_sub_sd( zero, a_0 );
		a_1 = _mm_sub_sd( zero, a_1 );
		a_2 = _mm_sub_sd( zero, a_2 );
		a_3 = _mm_sub_sd( zero, a_3 );
		}

	if(alg==0)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[8] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[12] );
			t_1 = _mm_mul_sd( a_3, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[5] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[9] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[13] );
			t_1 = _mm_mul_sd( a_3, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[4], t_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[6] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[10] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[14] );
			t_1 = _mm_mul_sd( a_3, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[8], t_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[7] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[11] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[15] );
			t_1 = _mm_mul_sd( a_3, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[12], t_0 );

			B += bs*sdb;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[8] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[12] );
			t_1 = _mm_mul_sd( a_3, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );

			B += 1;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_1 = _mm_mul_sd( a_1, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[8] );
			t_1 = _mm_mul_sd( a_2, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			b_0 = _mm_load_sd( &B[12] );
			t_1 = _mm_mul_sd( a_3, b_0 );
			t_0 = _mm_add_sd( t_0, t_1 );
			_mm_store_sd( &D[0], t_0 );
			}

		}
	else // if(alg==1 || alg==-1)
		{

		k = 0;
		for(; k<kmax-3; k+=4)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[8] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[12] );
			t_0 = _mm_mul_sd( a_3, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			b_0 = _mm_load_sd( &B[1] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[4] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[5] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[9] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[13] );
			t_0 = _mm_mul_sd( a_3, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[4], c_0 );

			b_0 = _mm_load_sd( &B[2] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[8] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[6] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[10] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[14] );
			t_0 = _mm_mul_sd( a_3, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[8], c_0 );

			b_0 = _mm_load_sd( &B[3] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[12] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[7] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[11] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[15] );
			t_0 = _mm_mul_sd( a_3, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[12], c_0 );

			B += bs*sdb;
			C += 16;
			D += 16;
			}
		for(; k<kmax; k++)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[8] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[12] );
			t_0 = _mm_mul_sd( a_3, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );

			B += 1;
			C += 4;
			D += 4;
			}
		if(tri-k>0)
			{
			b_0 = _mm_load_sd( &B[0] );
			t_0 = _mm_mul_sd( a_0, b_0 );
			c_0 = _mm_load_sd( &C[0] );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[4] );
			t_0 = _mm_mul_sd( a_1, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[8] );
			t_0 = _mm_mul_sd( a_2, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			b_0 = _mm_load_sd( &B[12] );
			t_0 = _mm_mul_sd( a_3, b_0 );
			c_0 = _mm_add_sd( c_0, t_0 );
			_mm_store_sd( &D[0], c_0 );
			}

		}
	
	}


#endif










