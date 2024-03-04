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



// new kernels

// _ll_ = left-lower ; _diag_ = unit diagoanl
void kernel_dtrsm_nn_ll_diag_8x4_lib4(int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, double *E0, int sde)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *E1 = E0 + 4*sde;

	__builtin_prefetch( B+0 );
	__builtin_prefetch( B+8 );

	const int bs = 4;

	const int B_next = bs*sdb;

	__builtin_prefetch( B+B_next+0 );
	__builtin_prefetch( B+B_next+8 );

	int k;

	__m256d
		zeros, tmp, t_0,
		a_0, a_4, A_0, A_4,
		b_0,
		B_0, B_1, B_2, B_3,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		c_0,
		e_0;
	
	// correction phase

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
		{


		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		__builtin_prefetch( B+2*B_next+0 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		__builtin_prefetch( B+2*B_next+8 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		B  += B_next;


		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		A0 += 4*bs;


		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		A1 += 4*bs;
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		}
	
	if(kmax-k==4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		}
	else
		{

		for(; k<kmax; k++)
			{

			a_0 = _mm256_load_pd( &A0[0+bs*0] );
			a_4 = _mm256_load_pd( &A1[0+bs*0] );
			b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_sub_pd( d_0, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_4 = _mm256_sub_pd( d_4, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_1 = _mm256_sub_pd( d_1, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_5 = _mm256_sub_pd( d_5, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_2 = _mm256_sub_pd( d_2, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_6 = _mm256_sub_pd( d_6, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_3 = _mm256_sub_pd( d_3, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		__builtin_prefetch( B+2*B_next+8 );

		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
#endif

	add:

	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C0[0+bs*0]);
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C0[0+bs*1]);
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C0[0+bs*2]);
		d_2 = _mm256_add_pd( c_0, d_2 );
		c_0 = _mm256_load_pd( &C0[0+bs*3]);
		d_3 = _mm256_add_pd( c_0, d_3 );
		c_0 = _mm256_load_pd( &C1[0+bs*0]);
		d_4 = _mm256_add_pd( c_0, d_4 );
		c_0 = _mm256_load_pd( &C1[0+bs*1]);
		d_5 = _mm256_add_pd( c_0, d_5 );
		c_0 = _mm256_load_pd( &C1[0+bs*2]);
		d_6 = _mm256_add_pd( c_0, d_6 );
		c_0 = _mm256_load_pd( &C1[0+bs*3]);
		d_7 = _mm256_add_pd( c_0, d_7 );
		}


	// solution phase

	zeros = _mm256_setzero_pd();

	// solve top-left
	
	e_0 = _mm256_load_pd( &E0[0+bs*0] );
	a_0 = _mm256_load_pd( &E1[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*1] );
	a_0 = _mm256_load_pd( &E1[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*2] );
	a_0 = _mm256_load_pd( &E1[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	a_0 = _mm256_load_pd( &E1[0+bs*3] );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	// store upper
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );
	_mm256_store_pd( &D0[0+bs*3], d_3 );

	// correct bottom-left TODO merge with previous

	E0 = E1; // += 4*sde;

//	a_0 = _mm256_load_pd( &E0[0+bs*0] );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

//	a_0 = _mm256_load_pd( &E0[0+bs*1] );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

//	a_0 = _mm256_load_pd( &E0[0+bs*2] );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

//	a_0 = _mm256_load_pd( &E0[0+bs*3] );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

	// solve bottom-right

	E0 += 4*bs;
	
	e_0 = _mm256_load_pd( &E0[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	// store upper
	_mm256_store_pd( &D1[0+bs*0], d_4 );
	_mm256_store_pd( &D1[0+bs*1], d_5 );
	_mm256_store_pd( &D1[0+bs*2], d_6 );
	_mm256_store_pd( &D1[0+bs*3], d_7 );

	return;

	}



void kernel_dtrsm_nn_ll_diag_8x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, double *E0, int sde)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *E1 = E0 + 4*sde;

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
		zeros, tmp, t_0,
		a_0, a_4, A_0, A_4,
		b_0,
		B_0, B_1, B_2, B_3,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		c_0,
		e_0;
	
	__m256i 
		msk;

	// correction phase

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
		{


		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		__builtin_prefetch( B+2*B_next+0 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		__builtin_prefetch( B+2*B_next+8 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		B  += B_next;


		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		A0 += 4*bs;


		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		A1 += 4*bs;
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		}
	
	if(kmax-k==4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		}
	else
		{

		for(; k<kmax; k++)
			{

			a_0 = _mm256_load_pd( &A0[0+bs*0] );
			a_4 = _mm256_load_pd( &A1[0+bs*0] );
			b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_sub_pd( d_0, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_4 = _mm256_sub_pd( d_4, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_1 = _mm256_sub_pd( d_1, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_5 = _mm256_sub_pd( d_5, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_2 = _mm256_sub_pd( d_2, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_6 = _mm256_sub_pd( d_6, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_3 = _mm256_sub_pd( d_3, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		__builtin_prefetch( B+2*B_next+8 );

		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
#endif

	add:

	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C0[0+bs*0]);
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C0[0+bs*1]);
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C0[0+bs*2]);
		d_2 = _mm256_add_pd( c_0, d_2 );
		c_0 = _mm256_load_pd( &C0[0+bs*3]);
		d_3 = _mm256_add_pd( c_0, d_3 );
		c_0 = _mm256_load_pd( &C1[0+bs*0]);
		d_4 = _mm256_add_pd( c_0, d_4 );
		c_0 = _mm256_load_pd( &C1[0+bs*1]);
		d_5 = _mm256_add_pd( c_0, d_5 );
		c_0 = _mm256_load_pd( &C1[0+bs*2]);
		d_6 = _mm256_add_pd( c_0, d_6 );
		c_0 = _mm256_load_pd( &C1[0+bs*3]);
		d_7 = _mm256_add_pd( c_0, d_7 );
		}


	// solution phase

	zeros = _mm256_setzero_pd();

	// solve top-left
	
	e_0 = _mm256_load_pd( &E0[0+bs*0] );
	a_0 = _mm256_load_pd( &E1[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*1] );
	a_0 = _mm256_load_pd( &E1[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*2] );
	a_0 = _mm256_load_pd( &E1[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	a_0 = _mm256_load_pd( &E1[0+bs*3] );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );


	// correct bottom-left TODO merge with previous

	E0 = E1; // += 4*sde;

//	a_0 = _mm256_load_pd( &E0[0+bs*0] );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[0+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

//	a_0 = _mm256_load_pd( &E0[0+bs*1] );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[1+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

//	a_0 = _mm256_load_pd( &E0[0+bs*2] );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[2+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

//	a_0 = _mm256_load_pd( &E0[0+bs*3] );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*0] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_4 = _mm256_sub_pd( d_4, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*1] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_5 = _mm256_sub_pd( d_5, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*2] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_6 = _mm256_sub_pd( d_6, tmp );
//	b_0 = _mm256_broadcast_sd( &D0[3+bs*3] );
//	tmp = _mm256_mul_pd( a_0, b_0 );
//	d_7 = _mm256_sub_pd( d_7, tmp );

	// solve bottom-right

	E0 += 4*bs;
	
	e_0 = _mm256_load_pd( &E0[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );
	t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_6 = _mm256_sub_pd( d_6, tmp );
	t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_7 = _mm256_sub_pd( d_7, tmp );

	// store 
	d_temp = km - 4.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );
	_mm256_store_pd( &D0[0+bs*1], d_1 );
	_mm256_store_pd( &D0[0+bs*2], d_2 );

	_mm256_maskstore_pd( &D1[0+bs*0], msk, d_4 );
	_mm256_maskstore_pd( &D1[0+bs*1], msk, d_5 );
	_mm256_maskstore_pd( &D1[0+bs*2], msk, d_6 );

	if(kn>=4)
		{
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_maskstore_pd( &D1[0+bs*3], msk, d_7 );
		}


	return;

	}



void kernel_dtrsm_nn_ll_diag_8x2_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, double *E0, int sde)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *E1 = E0 + 4*sde;

	__builtin_prefetch( B+0 );

	const int bs = 4;

	const int B_next = bs*sdb;

	__builtin_prefetch( B+B_next+0 );

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		zeros, tmp, t_0,
		a_0, a_4, A_0, A_4,
		b_0,
		B_0, B_1,
		d_0, d_1,
		d_4, d_5,
		c_0,
		e_0;
	
	__m256i 
		msk;

	// correction phase

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();

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
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
	add:

	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C0[0+bs*0]);
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C0[0+bs*1]);
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C1[0+bs*0]);
		d_4 = _mm256_add_pd( c_0, d_4 );
		c_0 = _mm256_load_pd( &C1[0+bs*1]);
		d_5 = _mm256_add_pd( c_0, d_5 );
		}


	// solution phase

	zeros = _mm256_setzero_pd();

	// solve top-left
	
	e_0 = _mm256_load_pd( &E0[0+bs*0] );
	a_0 = _mm256_load_pd( &E1[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*1] );
	a_0 = _mm256_load_pd( &E1[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*2] );
	a_0 = _mm256_load_pd( &E1[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );

	a_0 = _mm256_load_pd( &E1[0+bs*3] );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( a_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );


	// solve bottom-right

	E0 = E1 + 4*bs;
	
	e_0 = _mm256_load_pd( &E0[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );

	e_0 = _mm256_load_pd( &E0[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_4 = _mm256_sub_pd( d_4, tmp );
	t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x00 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_5 = _mm256_sub_pd( d_5, tmp );

	// store 
	d_temp = km - 4.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_store_pd( &D0[0+bs*0], d_0 );

	_mm256_maskstore_pd( &D1[0+bs*0], msk, d_4 );

	if(kn>=2)
		{
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_maskstore_pd( &D1[0+bs*1], msk, d_5 );
		}


	return;

	}



#if 0
// rank 0 update
void kernel_dtrsm_nn_ll_diag_r0_8_lib4(int n, double *C0, int sdc, double *D0, int sdd, double *E0, int sde)
	{

	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	double *E1 = E0 + 4*sde;

	const int bs = 4;

	int k;

	__m256d
		zeros, tmp, t_0,
		a_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		e_0;
	

	zeros = _mm256_setzero_pd();



	for(k=0; k<n-3; k+=4)
		{

		d_0 = _mm256_load_pd( &C0[0+bs*0]);
		d_1 = _mm256_load_pd( &C0[0+bs*1]);
		d_2 = _mm256_load_pd( &C0[0+bs*2]);
		d_3 = _mm256_load_pd( &C0[0+bs*3]);
		d_4 = _mm256_load_pd( &C1[0+bs*0]);
		d_5 = _mm256_load_pd( &C1[0+bs*1]);
		d_6 = _mm256_load_pd( &C1[0+bs*2]);
		d_7 = _mm256_load_pd( &C1[0+bs*3]);
		C0 += 4*bs;
		C1 += 4*bs;

		// solution phase

		// solve top-left
		
		e_0 = _mm256_load_pd( &E0[0+bs*0] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
		a_0 = _mm256_load_pd( &E1[0+bs*0] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		e_0 = _mm256_load_pd( &E0[0+bs*1] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
		a_0 = _mm256_load_pd( &E1[0+bs*1] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		e_0 = _mm256_load_pd( &E0[0+bs*2] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
		a_0 = _mm256_load_pd( &E1[0+bs*2] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		a_0 = _mm256_load_pd( &E1[0+bs*3] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		// store upper
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_store_pd( &D0[0+bs*2], d_2 );
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		D0 += 4*bs;

		// solve bottom-right

		e_0 = _mm256_load_pd( &E1[0+bs*4] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
		t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		e_0 = _mm256_load_pd( &E1[0+bs*5] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
		t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		e_0 = _mm256_load_pd( &E1[0+bs*6] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
		t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x00 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		t_0 = _mm256_permute2f128_pd( d_5, d_5, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x00 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		t_0 = _mm256_permute2f128_pd( d_6, d_6, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x00 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		t_0 = _mm256_permute2f128_pd( d_7, d_7, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x00 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		// store upper
		_mm256_store_pd( &D1[0+bs*0], d_4 );
		_mm256_store_pd( &D1[0+bs*1], d_5 );
		_mm256_store_pd( &D1[0+bs*2], d_6 );
		_mm256_store_pd( &D1[0+bs*3], d_7 );
		D1 += 4*bs;

		}
	for(; k<n; k++)
		{

		d_0 = _mm256_load_pd( &C0[0+bs*0]);
		d_4 = _mm256_load_pd( &C1[0+bs*0]);
		C0 += 1*bs;
		C1 += 1*bs;

		// solution phase

		// solve top-left
		
		e_0 = _mm256_load_pd( &E0[0+bs*0] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
		a_0 = _mm256_load_pd( &E1[0+bs*0] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );

		e_0 = _mm256_load_pd( &E0[0+bs*1] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
		a_0 = _mm256_load_pd( &E1[0+bs*1] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );

		e_0 = _mm256_load_pd( &E0[0+bs*2] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
		a_0 = _mm256_load_pd( &E1[0+bs*2] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );

		a_0 = _mm256_load_pd( &E1[0+bs*3] );
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( a_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );

		// store upper
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		D0 += 1*bs;

		// solve bottom-right

		e_0 = _mm256_load_pd( &E1[0+bs*4] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
		t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );

		e_0 = _mm256_load_pd( &E1[0+bs*5] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
		t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );

		e_0 = _mm256_load_pd( &E1[0+bs*6] );
		e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
		t_0 = _mm256_permute2f128_pd( d_4, d_4, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x00 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );

		// store upper
		_mm256_store_pd( &D1[0+bs*0], d_4 );
		D1 += 1*bs;

		}

	return;

	}



// rank 0 update
void kernel_dtrsm_nn_ll_diag_r0_4_lib4(int n, double *C, double *D, double *E)
	{

	const int bs = 4;

	int k;

	__m256d
		zeros, tmp, t_0,
		d_0, d_1, d_2, d_3,
		e_0, e_1, e_2;
	

	zeros = _mm256_setzero_pd();
	
	e_0 = _mm256_load_pd( &E[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	e_1 = _mm256_load_pd( &E[0+bs*1] );
	e_1 = _mm256_blend_pd( e_1, zeros, 0x3 );
	e_2 = _mm256_load_pd( &E[0+bs*2] );
	e_2 = _mm256_blend_pd( e_2, zeros, 0x7 );

	for(k=0; k<n-3; k+=4)
		{

		// load
		d_0 = _mm256_load_pd( &C[0+bs*0]);
		d_1 = _mm256_load_pd( &C[0+bs*1]);
		d_2 = _mm256_load_pd( &C[0+bs*2]);
		d_3 = _mm256_load_pd( &C[0+bs*3]);
		C += 4*bs;

		// solution 
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );

		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_1, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_1, t_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_1, t_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_1, t_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );

		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_2, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_2, t_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_2, t_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_2, t_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );

		// store
		_mm256_store_pd( &D[0+bs*0], d_0 );
		_mm256_store_pd( &D[0+bs*1], d_1 );
		_mm256_store_pd( &D[0+bs*2], d_2 );
		_mm256_store_pd( &D[0+bs*3], d_3 );
		D += 4*bs;

		}
	for(; k<n; k++)
		{

		// load
		d_0 = _mm256_load_pd( &C[0+bs*0]);
		C += 1*bs;

		// solution 
		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_0, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );

		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
		t_0 = _mm256_permute_pd( t_0, 0xf );
		tmp = _mm256_mul_pd( e_1, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );

		t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
		t_0 = _mm256_permute_pd( t_0, 0x0 );
		tmp = _mm256_mul_pd( e_2, t_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );

		// store
		_mm256_store_pd( &D[0+bs*0], d_0 );
		D += 1*bs;

		}

	}
#endif



void kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		zeros, tmp, t_0,
		a_0,
		b_0,
		d_0, d_1, d_2, d_3,
		c_0,
		e_0;
	
	__m256i
		msk;
	
	// correction phase

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		A += 4*bs;
		B += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		A += 1*bs;
		B += 1;

		}
	
	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C[0+bs*0]);
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C[0+bs*1]);
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C[0+bs*2]);
		d_2 = _mm256_add_pd( c_0, d_2 );
		c_0 = _mm256_load_pd( &C[0+bs*3]);
		d_3 = _mm256_add_pd( c_0, d_3 );
		}


	// solution phase
	
	zeros = _mm256_setzero_pd();

	e_0 = _mm256_load_pd( &E[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );

	e_0 = _mm256_load_pd( &E[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );

	e_0 = _mm256_load_pd( &E[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );
	t_0 = _mm256_permute2f128_pd( d_2, d_2, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_2 = _mm256_sub_pd( d_2, tmp );
	t_0 = _mm256_permute2f128_pd( d_3, d_3, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_3 = _mm256_sub_pd( d_3, tmp );

	// store
	d_temp = km - 0.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_maskstore_pd( &D[0+bs*0], msk, d_0 );
	_mm256_maskstore_pd( &D[0+bs*1], msk, d_1 );
	_mm256_maskstore_pd( &D[0+bs*2], msk, d_2 );

	if(kn>=4)
		{
		_mm256_maskstore_pd( &D[0+bs*3], msk, d_3 );
		}
	return;


	}



void kernel_dtrsm_nn_ll_diag_4x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(4, 4, kmax, A, B, sdb, alg, C, D, E);

	return;

	}



void kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m256d
		zeros, tmp, t_0,
		a_0,
		b_0,
		d_0, d_1, // TODO use more accumulation registers !!!
		c_0,
		e_0;
	
	__m256i
		msk;
	
	// correction phase

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		A += 4*bs;
		B += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		A += 1*bs;
		B += 1;

		}
	
	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C[0+bs*0]);
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C[0+bs*1]);
		d_1 = _mm256_add_pd( c_0, d_1 );
		}


	// solution phase
	
	zeros = _mm256_setzero_pd();

	e_0 = _mm256_load_pd( &E[0+bs*0] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x1 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );

	e_0 = _mm256_load_pd( &E[0+bs*1] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x3 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x00 );
	t_0 = _mm256_permute_pd( t_0, 0xf );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );

	e_0 = _mm256_load_pd( &E[0+bs*2] );
	e_0 = _mm256_blend_pd( e_0, zeros, 0x7 );
	t_0 = _mm256_permute2f128_pd( d_0, d_0, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_0 = _mm256_sub_pd( d_0, tmp );
	t_0 = _mm256_permute2f128_pd( d_1, d_1, 0x11 );
	t_0 = _mm256_permute_pd( t_0, 0x0 );
	tmp = _mm256_mul_pd( e_0, t_0 );
	d_1 = _mm256_sub_pd( d_1, tmp );

	// store
	d_temp = km - 0.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );
	_mm256_maskstore_pd( &D[0+bs*0], msk, d_0 );

	if(kn>=2)
		{
		_mm256_maskstore_pd( &D[0+bs*1], msk, d_1 );
		}
	return;


	}



#if 0
void corner_dtrsm_nn_ll_diag_4x4_lib4(double *C, double *D, double *E)
	{

	const int bs = 4;

	__m128d
		v_0;

	__m256d
		zeros, tmp,
		c_0, c_1, c_2, c_3,
		e_00, e_01, e_02; //, e_03;
	
	zeros = _mm256_setzero_pd();

	e_00 = _mm256_load_pd( &E[0+bs*0] );
	e_01 = _mm256_load_pd( &E[0+bs*1] );
	e_02 = _mm256_load_pd( &E[0+bs*2] );
//	e_03 = _mm256_load_pd( &E[0+bs*3] );

	e_00 = _mm256_blend_pd( e_00, zeros, 0x1 );
	e_01 = _mm256_blend_pd( e_01, zeros, 0x3 );
	e_02 = _mm256_blend_pd( e_02, zeros, 0x7 );
//	e_03 = _mm256_blend_pd( e_03, zeros, 0xf );

	
	c_0 = _mm256_load_pd( &C[0+bs*0]);
	c_1 = _mm256_load_pd( &C[0+bs*1]);
	c_2 = _mm256_load_pd( &C[0+bs*2]);
	c_3 = _mm256_load_pd( &C[0+bs*3]);

	v_0 = _mm_movedup_pd( _mm256_castpd256_pd128( c_0 ) );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_00 );
	c_0 = _mm256_sub_pd( c_0, tmp );
	v_0 = _mm_movedup_pd( _mm256_castpd256_pd128( c_1 ) );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_00 );
	c_1 = _mm256_sub_pd( c_1, tmp );
	v_0 = _mm_movedup_pd( _mm256_castpd256_pd128( c_2 ) );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_00 );
	c_2 = _mm256_sub_pd( c_2, tmp );
	v_0 = _mm_movedup_pd( _mm256_castpd256_pd128( c_3 ) );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_00 );
	c_3 = _mm256_sub_pd( c_3, tmp );

	v_0 = _mm_permute_pd( _mm256_castpd256_pd128( c_0 ), 0x3 );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_01 );
	c_0 = _mm256_sub_pd( c_0, tmp );
	v_0 = _mm_permute_pd( _mm256_castpd256_pd128( c_1 ), 0x3 );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_01 );
	c_1 = _mm256_sub_pd( c_1, tmp );
	v_0 = _mm_permute_pd( _mm256_castpd256_pd128( c_2 ), 0x3 );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_01 );
	c_2 = _mm256_sub_pd( c_2, tmp );
	v_0 = _mm_permute_pd( _mm256_castpd256_pd128( c_3 ), 0x3 );
	tmp = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
	tmp = _mm256_mul_pd( tmp, e_01 );
	c_3 = _mm256_sub_pd( c_3, tmp );

	tmp = _mm256_permute2f128_pd( c_0, c_0, 0x11 );
	tmp = _mm256_permute_pd( tmp, 0x00 );
	tmp = _mm256_mul_pd( tmp, e_02 );
	c_0 = _mm256_sub_pd( c_0, tmp );
	tmp = _mm256_permute2f128_pd( c_1, c_1, 0x11 );
	tmp = _mm256_permute_pd( tmp, 0x00 );
	tmp = _mm256_mul_pd( tmp, e_02 );
	c_1 = _mm256_sub_pd( c_1, tmp );
	tmp = _mm256_permute2f128_pd( c_2, c_2, 0x11 );
	tmp = _mm256_permute_pd( tmp, 0x00 );
	tmp = _mm256_mul_pd( tmp, e_02 );
	c_2 = _mm256_sub_pd( c_2, tmp );
	tmp = _mm256_permute2f128_pd( c_3, c_3, 0x11 );
	tmp = _mm256_permute_pd( tmp, 0x00 );
	tmp = _mm256_mul_pd( tmp, e_02 );
	c_3 = _mm256_sub_pd( c_3, tmp );

	// store
	_mm256_store_pd( &D[0+bs*0], c_0 );
	_mm256_store_pd( &D[0+bs*1], c_1 );
	_mm256_store_pd( &D[0+bs*2], c_2 );
	_mm256_store_pd( &D[0+bs*3], c_3 );

	}
#endif



// _ru_ = right-upper (i.e. side-uplo format, the transa in in the dgemm sub-kernel name, diag is not considered)
void kernel_dtrsm_nn_ru_8x4_lib4(int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, double *E, int use_inv_diag_E, double *inv_diag_E)
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

	__m128d
		ones,
		v_0;

	__m256d
		tmp,
		a_0, a_4, A_0, A_4,
		b_0,
		B_0, B_1, B_2, B_3,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		c_0,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;
	
	// correction phase

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
		{


		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		__builtin_prefetch( B+2*B_next+0 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		__builtin_prefetch( B+2*B_next+8 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		B  += B_next;


		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		A0 += 4*bs;


		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		A1 += 4*bs;
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		}
	
	if(kmax-k==4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		}
	else
		{

		for(; k<kmax; k++)
			{

			a_0 = _mm256_load_pd( &A0[0+bs*0] );
			a_4 = _mm256_load_pd( &A1[0+bs*0] );
			b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_sub_pd( d_0, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_4 = _mm256_sub_pd( d_4, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_1 = _mm256_sub_pd( d_1, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_5 = _mm256_sub_pd( d_5, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_2 = _mm256_sub_pd( d_2, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_6 = _mm256_sub_pd( d_6, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_3 = _mm256_sub_pd( d_3, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		__builtin_prefetch( B+2*B_next+8 );

		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
#endif


	add:
	
	// load C
	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C0[0+bs*0] );
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C0[0+bs*1] );
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C0[0+bs*2] );
		d_2 = _mm256_add_pd( c_0, d_2 );
		c_0 = _mm256_load_pd( &C0[0+bs*3] );
		d_3 = _mm256_add_pd( c_0, d_3 );
		c_0 = _mm256_load_pd( &C1[0+bs*0] );
		d_4 = _mm256_add_pd( c_0, d_4 );
		c_0 = _mm256_load_pd( &C1[0+bs*1] );
		d_5 = _mm256_add_pd( c_0, d_5 );
		c_0 = _mm256_load_pd( &C1[0+bs*2] );
		d_6 = _mm256_add_pd( c_0, d_6 );
		c_0 = _mm256_load_pd( &C1[0+bs*3] );
		d_7 = _mm256_add_pd( c_0, d_7 );
		}

	// solution phase

	ones = _mm_set_pd( 1.0, 1.0 );

	if(use_inv_diag_E)
		{

		// first column
		e_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		d_4  = _mm256_mul_pd( d_4, e_00 );
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		_mm256_store_pd( &D1[0+bs*0], d_4 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		e_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		tmp  = _mm256_mul_pd( d_4, e_01 );
		d_5  = _mm256_sub_pd( d_5, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		d_5  = _mm256_mul_pd( d_5, e_11 );
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_store_pd( &D1[0+bs*1], d_5 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		e_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		tmp  = _mm256_mul_pd( d_0, e_02 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_4, e_02 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		tmp  = _mm256_mul_pd( d_1, e_12 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_5, e_12 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		d_2  = _mm256_mul_pd( d_2, e_22 );
		d_6  = _mm256_mul_pd( d_6, e_22 );
		_mm256_store_pd( &D0[0+bs*2], d_2 );
		_mm256_store_pd( &D1[0+bs*2], d_6 );

		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		e_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		tmp  = _mm256_mul_pd( d_0, e_03 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_4, e_03 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_1, e_13 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_5, e_13 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_2, e_23 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_6, e_23 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		d_3  = _mm256_mul_pd( d_3, e_33 );
		d_7  = _mm256_mul_pd( d_7, e_33 );
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_store_pd( &D1[0+bs*3], d_7 );

		}
	else
		{

		// first column
		v_0  = _mm_loaddup_pd( &E[0+bs*0] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		d_4  = _mm256_mul_pd( d_4, e_00 );
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		_mm256_store_pd( &D1[0+bs*0], d_4 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		v_0  = _mm_loaddup_pd( &E[1+bs*1] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_11 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		tmp  = _mm256_mul_pd( d_4, e_01 );
		d_5  = _mm256_sub_pd( d_5, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		d_5  = _mm256_mul_pd( d_5, e_11 );
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_store_pd( &D1[0+bs*1], d_5 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		v_0  = _mm_loaddup_pd( &E[2+bs*2] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_22 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_02 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_4, e_02 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		tmp  = _mm256_mul_pd( d_1, e_12 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_5, e_12 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		d_2  = _mm256_mul_pd( d_2, e_22 );
		d_6  = _mm256_mul_pd( d_6, e_22 );
		_mm256_store_pd( &D0[0+bs*2], d_2 );
		_mm256_store_pd( &D1[0+bs*2], d_6 );

		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		v_0  = _mm_loaddup_pd( &E[3+bs*3] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_33 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_03 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_4, e_03 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_1, e_13 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_5, e_13 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_2, e_23 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_6, e_23 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		d_3  = _mm256_mul_pd( d_3, e_33 );
		d_7  = _mm256_mul_pd( d_7, e_33 );
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_store_pd( &D1[0+bs*3], d_7 );

		}
	
	}



void kernel_dtrsm_nn_ru_8x4_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, double *E, int use_inv_diag_E, double *inv_diag_E)
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

	__m128d
		ones,
		v_0;

	__m256d
		tmp,
		a_0, a_4, A_0, A_4,
		b_0,
		B_0, B_1, B_2, B_3,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		c_0,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;
	
	__m256i 
		msk;

	// correction phase

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
		{


		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		__builtin_prefetch( B+2*B_next+0 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		__builtin_prefetch( B+2*B_next+8 );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		B  += B_next;


		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );
		A0 += 4*bs;


		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		A1 += 4*bs;
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		}
	
	if(kmax-k==4)
		{

		a_0 = _mm256_load_pd( &A0[0+bs*0] );
		a_4 = _mm256_load_pd( &A1[0+bs*0] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*1] );
		a_4 = _mm256_load_pd( &A1[0+bs*1] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		B_0 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		B_1 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		B_2 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		B_3 = _mm256_broadcast_pd( (__m128d *) &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		b_0 = _mm256_permute_pd( B_0, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0x0 );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		a_0 = _mm256_load_pd( &A0[0+bs*3] );
		a_4 = _mm256_load_pd( &A1[0+bs*3] );
		b_0 = _mm256_permute_pd( B_0, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_permute_pd( B_1, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_permute_pd( B_2, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_permute_pd( B_3, 0xf );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		}
	else
		{

		for(; k<kmax; k++)
			{

			a_0 = _mm256_load_pd( &A0[0+bs*0] );
			a_4 = _mm256_load_pd( &A1[0+bs*0] );
			b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_0 = _mm256_sub_pd( d_0, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_4 = _mm256_sub_pd( d_4, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_1 = _mm256_sub_pd( d_1, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_5 = _mm256_sub_pd( d_5, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_2 = _mm256_sub_pd( d_2, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_6 = _mm256_sub_pd( d_6, tmp );
			b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
			tmp = _mm256_mul_pd( a_0, b_0 );
			d_3 = _mm256_sub_pd( d_3, tmp );
			tmp = _mm256_mul_pd( a_4, b_0 );
			d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );

		__builtin_prefetch( B+2*B_next+8 );

		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_6 = _mm256_sub_pd( d_6, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_7 = _mm256_sub_pd( d_7, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
#endif


	add:
	
	// load C
	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C0[0+bs*0] );
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C0[0+bs*1] );
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C0[0+bs*2] );
		d_2 = _mm256_add_pd( c_0, d_2 );
		c_0 = _mm256_load_pd( &C0[0+bs*3] );
		d_3 = _mm256_add_pd( c_0, d_3 );
		c_0 = _mm256_load_pd( &C1[0+bs*0] );
		d_4 = _mm256_add_pd( c_0, d_4 );
		c_0 = _mm256_load_pd( &C1[0+bs*1] );
		d_5 = _mm256_add_pd( c_0, d_5 );
		c_0 = _mm256_load_pd( &C1[0+bs*2] );
		d_6 = _mm256_add_pd( c_0, d_6 );
		c_0 = _mm256_load_pd( &C1[0+bs*3] );
		d_7 = _mm256_add_pd( c_0, d_7 );
		}

	// solution phase

	ones = _mm_set_pd( 1.0, 1.0 );
	d_temp = km - 4.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(use_inv_diag_E)
		{

		// first column
		e_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		d_4  = _mm256_mul_pd( d_4, e_00 );
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_4 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		e_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		tmp  = _mm256_mul_pd( d_4, e_01 );
		d_5  = _mm256_sub_pd( d_5, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		d_5  = _mm256_mul_pd( d_5, e_11 );
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_maskstore_pd( &D1[0+bs*1], msk, d_5 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		e_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		tmp  = _mm256_mul_pd( d_0, e_02 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_4, e_02 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		tmp  = _mm256_mul_pd( d_1, e_12 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_5, e_12 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		d_2  = _mm256_mul_pd( d_2, e_22 );
		d_6  = _mm256_mul_pd( d_6, e_22 );
		_mm256_store_pd( &D0[0+bs*2], d_2 );
		_mm256_maskstore_pd( &D1[0+bs*2], msk, d_6 );

		if(kn<4)
			return;

		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		e_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		tmp  = _mm256_mul_pd( d_0, e_03 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_4, e_03 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_1, e_13 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_5, e_13 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_2, e_23 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_6, e_23 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		d_3  = _mm256_mul_pd( d_3, e_33 );
		d_7  = _mm256_mul_pd( d_7, e_33 );
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_maskstore_pd( &D1[0+bs*3], msk, d_7 );

		}
	else
		{

		// first column
		v_0  = _mm_loaddup_pd( &E[0+bs*0] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		d_4  = _mm256_mul_pd( d_4, e_00 );
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_4 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		v_0  = _mm_loaddup_pd( &E[1+bs*1] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_11 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		tmp  = _mm256_mul_pd( d_4, e_01 );
		d_5  = _mm256_sub_pd( d_5, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		d_5  = _mm256_mul_pd( d_5, e_11 );
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_maskstore_pd( &D1[0+bs*1], msk, d_5 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		v_0  = _mm_loaddup_pd( &E[2+bs*2] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_22 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_02 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_4, e_02 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		tmp  = _mm256_mul_pd( d_1, e_12 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_5, e_12 );
		d_6  = _mm256_sub_pd( d_6, tmp );
		d_2  = _mm256_mul_pd( d_2, e_22 );
		d_6  = _mm256_mul_pd( d_6, e_22 );
		_mm256_store_pd( &D0[0+bs*2], d_2 );
		_mm256_maskstore_pd( &D1[0+bs*2], msk, d_6 );

		if(kn<4)
			return;

		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		v_0  = _mm_loaddup_pd( &E[3+bs*3] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_33 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_03 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_4, e_03 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_1, e_13 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_5, e_13 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		tmp  = _mm256_mul_pd( d_2, e_23 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_6, e_23 );
		d_7  = _mm256_sub_pd( d_7, tmp );
		d_3  = _mm256_mul_pd( d_3, e_33 );
		d_7  = _mm256_mul_pd( d_7, e_33 );
		_mm256_store_pd( &D0[0+bs*3], d_3 );
		_mm256_maskstore_pd( &D1[0+bs*3], msk, d_7 );

		}
	
	return;
	
	}



void kernel_dtrsm_nn_ru_8x2_vs_lib4(int km, int kn, int kmax, double *A0, int sda, double *B, int sdb, int alg, double *C0, int sdc, double *D0, int sdd, double *E, int use_inv_diag_E, double *inv_diag_E)
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

	__m128d
		ones,
		v_0;

	__m256d
		tmp,
		a_0, a_4, A_0, A_4,
		b_0,
		d_0, d_1,
		d_4, d_5,
		c_0,
		e_00, e_01,
		      e_11;
	
	__m256i 
		msk;

	// correction phase

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();

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
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*1] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*2] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );

		__builtin_prefetch( B+2*B_next+8 );

		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		A_0 = _mm256_load_pd( &A0[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		A_4 = _mm256_load_pd( &A1[0+bs*3] );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		a_0 = _mm256_load_pd( &A0[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( A_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		a_4 = _mm256_load_pd( &A1[0+bs*4] );
		tmp = _mm256_mul_pd( A_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


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
		d_0 = _mm256_sub_pd( d_0, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_4 = _mm256_sub_pd( d_4, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		tmp = _mm256_mul_pd( a_4, b_0 );
		d_5 = _mm256_sub_pd( d_5, tmp );


		A0 += 1*bs;
		A1 += 1*bs;
		B  += 1;

		}
	
	add:
	
	// load C
	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C0[0+bs*0] );
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C0[0+bs*1] );
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C1[0+bs*0] );
		d_4 = _mm256_add_pd( c_0, d_4 );
		c_0 = _mm256_load_pd( &C1[0+bs*1] );
		d_5 = _mm256_add_pd( c_0, d_5 );
		}

	// solution phase

	ones = _mm_set_pd( 1.0, 1.0 );

	if(use_inv_diag_E)
		{

		// first column
		e_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		d_4  = _mm256_mul_pd( d_4, e_00 );
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_4 );

		if(kn<2)
			return;

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		e_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		tmp  = _mm256_mul_pd( d_4, e_01 );
		d_5  = _mm256_sub_pd( d_5, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		d_5  = _mm256_mul_pd( d_5, e_11 );
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_maskstore_pd( &D1[0+bs*1], msk, d_5 );

		}
	else
		{

		// first column
		v_0  = _mm_loaddup_pd( &E[0+bs*0] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		d_4  = _mm256_mul_pd( d_4, e_00 );
		_mm256_store_pd( &D0[0+bs*0], d_0 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_4 );

		if(kn<2)
			return;

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		v_0  = _mm_loaddup_pd( &E[1+bs*1] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_11 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		tmp  = _mm256_mul_pd( d_4, e_01 );
		d_5  = _mm256_sub_pd( d_5, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		d_5  = _mm256_mul_pd( d_5, e_11 );
		_mm256_store_pd( &D0[0+bs*1], d_1 );
		_mm256_maskstore_pd( &D1[0+bs*1], msk, d_5 );

		}
	
	return;
	
	}



void kernel_dtrsm_nn_ru_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m128d
		ones,
		v_0;

	__m256d
		tmp,
		a_0,
		b_0,
		d_0,  d_1,  d_2,  d_3,
		c_0,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;
	
	__m256i
		msk;
	
	// correction phase

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_3 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		A += 4*bs;
		B += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_2 = _mm256_sub_pd( d_2, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_3 = _mm256_sub_pd( d_3, tmp );


		A += 1*bs;
		B += 1;

		}
	
	// load C
	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C[0+bs*0] );
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C[0+bs*1] );
		d_1 = _mm256_add_pd( c_0, d_1 );
		c_0 = _mm256_load_pd( &C[0+bs*2] );
		d_2 = _mm256_add_pd( c_0, d_2 );
		c_0 = _mm256_load_pd( &C[0+bs*3] );
		d_3 = _mm256_add_pd( c_0, d_3 );
		}

	// solution phase

	ones = _mm_set_pd( 1.0, 1.0 );
	d_temp = km - 0.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(use_inv_diag_E)
		{

		// first column
		e_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		_mm256_maskstore_pd( &D[0+bs*0], msk, d_0 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		e_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		_mm256_maskstore_pd( &D[0+bs*1], msk, d_1 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		e_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		tmp  = _mm256_mul_pd( d_0, e_02 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_1, e_12 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		d_2  = _mm256_mul_pd( d_2, e_22 );
		_mm256_maskstore_pd( &D[0+bs*2], msk, d_2 );

		if(kn<4)
			return;
		
		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		e_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		tmp  = _mm256_mul_pd( d_0, e_03 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_1, e_13 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_2, e_23 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		d_3  = _mm256_mul_pd( d_3, e_33 );
		_mm256_maskstore_pd( &D[0+bs*3], msk, d_3 );

		}
	else
		{

		// first column
		v_0  = _mm_loaddup_pd( &E[0+bs*0] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		_mm256_maskstore_pd( &D[0+bs*0], msk, d_0 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		v_0  = _mm_loaddup_pd( &E[1+bs*1] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_11 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		_mm256_maskstore_pd( &D[0+bs*1], msk, d_1 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		v_0  = _mm_loaddup_pd( &E[2+bs*2] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_22 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_02 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		tmp  = _mm256_mul_pd( d_1, e_12 );
		d_2  = _mm256_sub_pd( d_2, tmp );
		d_2  = _mm256_mul_pd( d_2, e_22 );
		_mm256_maskstore_pd( &D[0+bs*2], msk, d_2 );

		if(kn<4)
			return;

		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		v_0  = _mm_loaddup_pd( &E[3+bs*3] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_33 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_03 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_1, e_13 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		tmp  = _mm256_mul_pd( d_2, e_23 );
		d_3  = _mm256_sub_pd( d_3, tmp );
		d_3  = _mm256_mul_pd( d_3, e_33 );
		_mm256_maskstore_pd( &D[0+bs*3], msk, d_3 );

		}
	
	return;
	
	}



void kernel_dtrsm_nn_ru_4x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	kernel_dtrsm_nn_ru_4x4_vs_lib4(4, 4, kmax, A, B, sdb, alg, C, D, E, use_inv_diag_E, inv_diag_E);

	}



void kernel_dtrsm_nn_ru_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;

	int k;

	__m128d
		ones,
		v_0;

	__m256d
		tmp,
		a_0,
		b_0,
		d_0,  d_1,  // TODO use more accumulation registers !!!
		c_0,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;
	
	__m256i
		msk;
	
	// correction phase

	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();

	for(k=0; k<kmax-3; k+=4)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*1] );
		b_0 = _mm256_broadcast_sd( &B[1+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[1+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*2] );
		b_0 = _mm256_broadcast_sd( &B[2+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[2+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		a_0 = _mm256_load_pd( &A[0+bs*3] );
		b_0 = _mm256_broadcast_sd( &B[3+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[3+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		A += 4*bs;
		B += bs*sdb;

		}
	for(; k<kmax; k++)
		{

		a_0 = _mm256_load_pd( &A[0+bs*0] );
		b_0 = _mm256_broadcast_sd( &B[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		b_0 = _mm256_broadcast_sd( &B[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_1 = _mm256_sub_pd( d_1, tmp );


		A += 1*bs;
		B += 1;

		}
	
	// load C
	if(alg!=0)
		{
		c_0 = _mm256_load_pd( &C[0+bs*0] );
		d_0 = _mm256_add_pd( c_0, d_0 );
		c_0 = _mm256_load_pd( &C[0+bs*1] );
		d_1 = _mm256_add_pd( c_0, d_1 );
		}

	// solution phase

	ones = _mm_set_pd( 1.0, 1.0 );
	d_temp = km - 0.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(use_inv_diag_E)
		{

		// first column
		e_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		_mm256_maskstore_pd( &D[0+bs*0], msk, d_0 );

		if(kn<2)
			return;

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		e_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		_mm256_maskstore_pd( &D[0+bs*1], msk, d_1 );

		}
	else
		{

		// first column
		v_0  = _mm_loaddup_pd( &E[0+bs*0] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		d_0  = _mm256_mul_pd( d_0, e_00 );
		_mm256_maskstore_pd( &D[0+bs*0], msk, d_0 );

		if(kn<2)
			return;

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		v_0  = _mm_loaddup_pd( &E[1+bs*1] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_11 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( d_0, e_01 );
		d_1  = _mm256_sub_pd( d_1, tmp );
		d_1  = _mm256_mul_pd( d_1, e_11 );
		_mm256_maskstore_pd( &D[0+bs*1], msk, d_1 );

		}
	
	return;
	
	}



#if 0
void corner_dtrsm_nn_ru_4x4_lib4(double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	__m128d
		ones,
		v_0;

	__m256d
		tmp,
		c_0,  c_1,  c_2,  c_3,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33;
	
	ones = _mm_set_pd( 1.0, 1.0 );

	if(use_inv_diag_E)
		{

		// load C
		c_0  = _mm256_load_pd( &C[0+bs*0] );
		c_1  = _mm256_load_pd( &C[0+bs*1] );
		c_2  = _mm256_load_pd( &C[0+bs*2] );
		c_3  = _mm256_load_pd( &C[0+bs*3] );

		// first column
		e_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		c_0  = _mm256_mul_pd( c_0, e_00 );
		_mm256_store_pd( &D[0+bs*0], c_0 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		e_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		tmp  = _mm256_mul_pd( c_0, e_01 );
		c_1  = _mm256_sub_pd( c_1, tmp );
		c_1  = _mm256_mul_pd( c_1, e_11 );
		_mm256_store_pd( &D[0+bs*1], c_1 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		e_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		tmp  = _mm256_mul_pd( c_0, e_02 );
		c_2  = _mm256_sub_pd( c_2, tmp );
		tmp  = _mm256_mul_pd( c_1, e_12 );
		c_2  = _mm256_sub_pd( c_2, tmp );
		c_2  = _mm256_mul_pd( c_2, e_22 );
		_mm256_store_pd( &D[0+bs*2], c_2 );

		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		e_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		tmp  = _mm256_mul_pd( c_0, e_03 );
		c_3  = _mm256_sub_pd( c_3, tmp );
		tmp  = _mm256_mul_pd( c_1, e_13 );
		c_3  = _mm256_sub_pd( c_3, tmp );
		tmp  = _mm256_mul_pd( c_2, e_23 );
		c_3  = _mm256_sub_pd( c_3, tmp );
		c_3  = _mm256_mul_pd( c_3, e_33 );
		_mm256_store_pd( &D[0+bs*3], c_3 );

		}
	else
		{

		// load C
		c_0  = _mm256_load_pd( &C[0+bs*0] );
		c_1  = _mm256_load_pd( &C[0+bs*1] );
		c_2  = _mm256_load_pd( &C[0+bs*2] );
		c_3  = _mm256_load_pd( &C[0+bs*3] );

		// first column
		v_0  = _mm_loaddup_pd( &E[0+bs*0] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_00 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		c_0  = _mm256_mul_pd( c_0, e_00 );
		_mm256_store_pd( &D[0+bs*0], c_0 );

		// second column
		e_01 = _mm256_broadcast_sd( &E[0+bs*1] );
		v_0  = _mm_loaddup_pd( &E[1+bs*1] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_11 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( c_0, e_01 );
		c_1  = _mm256_sub_pd( c_1, tmp );
		c_1  = _mm256_mul_pd( c_1, e_11 );
		_mm256_store_pd( &D[0+bs*1], c_1 );

		// third column
		e_02 = _mm256_broadcast_sd( &E[0+bs*2] );
		e_12 = _mm256_broadcast_sd( &E[1+bs*2] );
		v_0  = _mm_loaddup_pd( &E[2+bs*2] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_22 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( c_0, e_02 );
		c_2  = _mm256_sub_pd( c_2, tmp );
		tmp  = _mm256_mul_pd( c_1, e_12 );
		c_2  = _mm256_sub_pd( c_2, tmp );
		c_2  = _mm256_mul_pd( c_2, e_22 );
		_mm256_store_pd( &D[0+bs*2], c_2 );

		// fourth column
		e_03 = _mm256_broadcast_sd( &E[0+bs*3] );
		e_13 = _mm256_broadcast_sd( &E[1+bs*3] );
		e_23 = _mm256_broadcast_sd( &E[2+bs*3] );
		v_0  = _mm_loaddup_pd( &E[3+bs*3] );
		v_0  = _mm_div_pd( ones, v_0 );
		e_33 = _mm256_permute2f128_pd( _mm256_castpd128_pd256( v_0 ), _mm256_castpd128_pd256( v_0 ), 0x00 );
		tmp  = _mm256_mul_pd( c_0, e_03 );
		c_3  = _mm256_sub_pd( c_3, tmp );
		tmp  = _mm256_mul_pd( c_1, e_13 );
		c_3  = _mm256_sub_pd( c_3, tmp );
		tmp  = _mm256_mul_pd( c_2, e_23 );
		c_3  = _mm256_sub_pd( c_3, tmp );
		c_3  = _mm256_mul_pd( c_3, e_33 );
		_mm256_store_pd( &D[0+bs*3], c_3 );

		}
	
	}
#endif





// TODO fix tri==1 in another routine !!!
void kernel_dtrsm_nt_8x4_lib4_new(int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *Am1 = Am0 + 4*sdam;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0,
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
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[4] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Am1[8] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[12] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Am1[16] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );
			
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
		
	__m256d
		a_ii;
	
	__m128d
		s_ii, ones;
	
	if(use_inv_diag_E)
		{
		a_ii = _mm256_broadcast_sd( &inv_diag_E[0] ); // E_00
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_store_pd( &D1[0+bs*0], d_40 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[1] ); // E_11
		d_01 = _mm256_mul_pd( d_01, a_ii );
		d_41 = _mm256_mul_pd( d_41, a_ii );
		_mm256_store_pd( &D0[0+bs*1], d_01 );
		_mm256_store_pd( &D1[0+bs*1], d_41 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[2] ); // E_22
		d_02 = _mm256_mul_pd( d_02, a_ii );
		d_42 = _mm256_mul_pd( d_42, a_ii );
		_mm256_store_pd( &D0[0+bs*2], d_02 );
		_mm256_store_pd( &D1[0+bs*2], d_42 );

		a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
		ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_42, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[3] ); // E_33
		d_03 = _mm256_mul_pd( d_03, a_ii );
		d_43 = _mm256_mul_pd( d_43, a_ii );
		_mm256_store_pd( &D0[0+bs*3], d_03 );
		_mm256_store_pd( &D1[0+bs*3], d_43 );
		}
	else
		{
		ones = _mm_set_sd( 1.0 );
		s_ii = _mm_load_sd( &E[0+bs*0] ); // E_00
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_store_pd( &D1[0+bs*0], d_40 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[1+bs*1] ); // E_11
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_01 = _mm256_mul_pd( d_01, a_ii );
		d_41 = _mm256_mul_pd( d_41, a_ii );
		_mm256_store_pd( &D0[0+bs*1], d_01 );
		_mm256_store_pd( &D1[0+bs*1], d_41 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[2+bs*2] ); // E_22
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_02 = _mm256_mul_pd( d_02, a_ii );
		d_42 = _mm256_mul_pd( d_42, a_ii );
		_mm256_store_pd( &D0[0+bs*2], d_02 );
		_mm256_store_pd( &D1[0+bs*2], d_42 );

		a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
		ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_42, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[3+bs*3] ); // E_33
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_03 = _mm256_mul_pd( d_03, a_ii );
		d_43 = _mm256_mul_pd( d_43, a_ii );
		_mm256_store_pd( &D0[0+bs*3], d_03 );
		_mm256_store_pd( &D1[0+bs*3], d_43 );
		}

	return;

	}



void kernel_dgemm_dtrsm_nt_8x4_lib4_new(int kadd, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0,
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
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
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
			A_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
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
			a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
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
			
			Ap0 += 16;
			Ap1 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
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
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
//			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
//			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
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

		
//			Ap0 += 4; // keep it !!!
//			Ap1 += 4; // keep it !!!
//			Bp  += 4; // keep it !!!

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
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[4] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Am1[8] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[12] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Am1[16] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );
			
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
		
	__m256d
		a_ii;
	
	__m128d
		s_ii, ones;
	
	if(use_inv_diag_E)
		{
		a_ii = _mm256_broadcast_sd( &inv_diag_E[0] ); // E_00
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_store_pd( &D1[0+bs*0], d_40 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[1] ); // E_11
		d_01 = _mm256_mul_pd( d_01, a_ii );
		d_41 = _mm256_mul_pd( d_41, a_ii );
		_mm256_store_pd( &D0[0+bs*1], d_01 );
		_mm256_store_pd( &D1[0+bs*1], d_41 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[2] ); // E_22
		d_02 = _mm256_mul_pd( d_02, a_ii );
		d_42 = _mm256_mul_pd( d_42, a_ii );
		_mm256_store_pd( &D0[0+bs*2], d_02 );
		_mm256_store_pd( &D1[0+bs*2], d_42 );

		a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
		ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_42, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[3] ); // E_33
		d_03 = _mm256_mul_pd( d_03, a_ii );
		d_43 = _mm256_mul_pd( d_43, a_ii );
		_mm256_store_pd( &D0[0+bs*3], d_03 );
		_mm256_store_pd( &D1[0+bs*3], d_43 );
		}
	else
		{
		ones = _mm_set_sd( 1.0 );
		s_ii = _mm_load_sd( &E[0+bs*0] ); // E_00
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_store_pd( &D1[0+bs*0], d_40 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[1+bs*1] ); // E_11
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_01 = _mm256_mul_pd( d_01, a_ii );
		d_41 = _mm256_mul_pd( d_41, a_ii );
		_mm256_store_pd( &D0[0+bs*1], d_01 );
		_mm256_store_pd( &D1[0+bs*1], d_41 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[2+bs*2] ); // E_22
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_02 = _mm256_mul_pd( d_02, a_ii );
		d_42 = _mm256_mul_pd( d_42, a_ii );
		_mm256_store_pd( &D0[0+bs*2], d_02 );
		_mm256_store_pd( &D1[0+bs*2], d_42 );

		a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
		ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
		d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_42, a_ii );
		d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[3+bs*3] ); // E_33
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_03 = _mm256_mul_pd( d_03, a_ii );
		d_43 = _mm256_mul_pd( d_43, a_ii );
		_mm256_store_pd( &D0[0+bs*3], d_03 );
		_mm256_store_pd( &D1[0+bs*3], d_43 );
		}

	return;

	}



void kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256i
		msk;

	__m256d
		a_0123, a_4567, A_0123, A_4567,
		b_0123, b_1032, b_3210, b_2301,
		ab_tmp0, zeros,
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

	k = 0;

	if(kadd>0)
		{

		if(tri_A)
			{

			zeros = _mm256_setzero_pd();

			if(kadd>=4)
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );
				
				// k = 1
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );

				// k = 2
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
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
				k  += 4;

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
					k  += 4;

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

						k  += 1;

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

							k  += 1;

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

								k  += 1;

								}

							}

						}

					}

				}
			else // kadd = {1 2 3}
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );

				k  += 1;

				if(kadd>1)
					{
					
					// k = 1
					a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
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

						k  += 1;

						}

					}

				}


			}

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
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
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
			A_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
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
			a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
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
			
			Ap0 += 16;
			Ap1 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
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
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
//			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			d_4    = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
//			A_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
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

		
//			Ap0 += 4; // keep it !!!
//			Ap1 += 4; // keep it !!!
//			Bp  += 4; // keep it !!!

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
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[4] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_4567  = _mm256_load_pd( &Am1[8] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			A_4567  = _mm256_load_pd( &Am1[12] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_4567  = _mm256_load_pd( &Am1[16] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_0123 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			d_4    = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_5    = _mm256_sub_pd( d_5, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_3210 );
			d_7    = _mm256_sub_pd( d_7, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_4567, b_1032 );
			d_6    = _mm256_sub_pd( d_6, ab_tmp0 );
			
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
		
	__m256d
		a_ii;
	
	__m128d
		s_ii, ones;
	
	d_temp = km - 4.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(use_inv_diag_E)
		{
		a_ii = _mm256_broadcast_sd( &inv_diag_E[0] ); // E_00
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_40 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[1] ); // E_11
		d_01 = _mm256_mul_pd( d_01, a_ii );
		d_41 = _mm256_mul_pd( d_41, a_ii );
		_mm256_store_pd( &D0[0+bs*1], d_01 );
		_mm256_maskstore_pd( &D1[0+bs*1], msk, d_41 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[2] ); // E_22
		d_02 = _mm256_mul_pd( d_02, a_ii );
		d_42 = _mm256_mul_pd( d_42, a_ii );
		_mm256_store_pd( &D0[0+bs*2], d_02 );
		_mm256_maskstore_pd( &D1[0+bs*2], msk, d_42 );

		if(kn>=4)
			{
			a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
			d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
			ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
			d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
			ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_42, a_ii );
			d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &inv_diag_E[3] ); // E_33
			d_03 = _mm256_mul_pd( d_03, a_ii );
			d_43 = _mm256_mul_pd( d_43, a_ii );
			_mm256_store_pd( &D0[0+bs*3], d_03 );
			_mm256_maskstore_pd( &D1[0+bs*3], msk, d_43 );
			}
		}
	else
		{
		ones = _mm_set_sd( 1.0 );
		s_ii = _mm_load_sd( &E[0+bs*0] ); // E_00
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_40 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[1+bs*1] ); // E_11
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_01 = _mm256_mul_pd( d_01, a_ii );
		d_41 = _mm256_mul_pd( d_41, a_ii );
		_mm256_store_pd( &D0[0+bs*1], d_01 );
		_mm256_maskstore_pd( &D1[0+bs*1], msk, d_41 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
		d_42 = _mm256_sub_pd( d_42, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[2+bs*2] ); // E_22
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_02 = _mm256_mul_pd( d_02, a_ii );
		d_42 = _mm256_mul_pd( d_42, a_ii );
		_mm256_store_pd( &D0[0+bs*2], d_02 );
		_mm256_maskstore_pd( &D1[0+bs*2], msk, d_42 );

		if(kn>=4)
			{
			a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
			d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
			ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_41, a_ii );
			d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
			ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_42, a_ii );
			d_43 = _mm256_sub_pd( d_43, ab_tmp0 );
			s_ii = _mm_load_sd( &inv_diag_E[3+bs*3] ); // E_33
			s_ii = _mm_div_sd( ones, s_ii );
			s_ii = _mm_movedup_pd( s_ii );
			a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
			d_03 = _mm256_mul_pd( d_03, a_ii );
			d_43 = _mm256_mul_pd( d_43, a_ii );
			_mm256_store_pd( &D0[0+bs*3], d_03 );
			_mm256_maskstore_pd( &D1[0+bs*3], msk, d_43 );
			}
		}

	return;

	}



void kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap0, int sdap, double *Bp, int ksub, double *Am0, int sdam, double *Bm, int alg, double *C0, int sdc, double *D0, int sdd, double *E, int use_inv_diag_E, double *inv_diag_E)
	{
	
	double *Ap1 = Ap0 + 4*sdap;
	double *Am1 = Am0 + 4*sdam;
	double *C1 = C0 + 4*sdc;
	double *D1 = D0 + 4*sdd;
	
	const int bs = 4;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256i
		msk;

	__m256d
		a_0123, a_4567,
		b_0101, b_1010,
		ab_tmp0, zeros,
		d_0, d_1,
		d_4, d_5;
	
	// zero registers
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();

	k = 0;

	if(kadd>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Ap0[0] );
		a_4567 = _mm256_load_pd( &Ap1[0] );
		b_0101 = _mm256_broadcast_pd( (__m128d *) &Bp[0] );

		if(tri_A==1)
			{

			if(kadd>=4)
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				
				// k = 1
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );

				// k = 2
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );

				// k = 2
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
				a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				
				Ap0 += 16;
				Ap1 += 16;
				Bp  += 16;
				k  += 4;

				if(kadd>=8)
					{

					// k = 4
					a_4567  = _mm256_blend_pd( zeros, a_4567, 0x1 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
					b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
					b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
					a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
					d_1 = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
					a_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
					d_5 = _mm256_add_pd( d_5, ab_tmp0 );
					
					// k = 5
					a_4567  = _mm256_blend_pd( zeros, a_4567, 0x3 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
					b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
					b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
					a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
					a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
					d_5     = _mm256_add_pd( d_5, ab_tmp0 );

					// k = 6
					a_4567  = _mm256_blend_pd( zeros, a_4567, 0x7 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
					b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
					b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
					a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
					a_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
					d_5     = _mm256_add_pd( d_5, ab_tmp0 );

					// k = 7
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
					b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
					b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
					d_4     = _mm256_add_pd( d_4, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
					a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
					a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
					d_5     = _mm256_add_pd( d_5, ab_tmp0 );
					
					Ap0 += 16;
					Ap1 += 16;
					Bp  += 16;
					k  += 4;

					}
				else
					{

					if(kadd>4)
						{

						// k = 4
						a_4567  = _mm256_blend_pd( zeros, a_4567, 0x1 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
						b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
						d_0     = _mm256_add_pd( d_0, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
						b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
						d_4     = _mm256_add_pd( d_4, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
						a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
						d_1     = _mm256_add_pd( d_1, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
						a_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
						d_5     = _mm256_add_pd( d_5, ab_tmp0 );

						k  += 1;

						if(kadd>5)
							{
						
							// k = 5
							a_4567  = _mm256_blend_pd( zeros, a_4567, 0x3 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
							b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
							d_0     = _mm256_add_pd( d_0, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
							b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
							d_4     = _mm256_add_pd( d_4, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
							a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
							d_1     = _mm256_add_pd( d_1, ab_tmp0 );
							ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
							a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
							d_5     = _mm256_add_pd( d_5, ab_tmp0 );

							k  += 1;

							if(kadd>6)
								{

								// k = 6
								a_4567  = _mm256_blend_pd( zeros, a_4567, 0x7 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
								b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
								d_0     = _mm256_add_pd( d_0, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
								b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
								d_4     = _mm256_add_pd( d_4, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
								a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
								d_1     = _mm256_add_pd( d_1, ab_tmp0 );
								ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
								a_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
								d_5     = _mm256_add_pd( d_5, ab_tmp0 );

								k  += 1;

								}

							}

						}

					}

				}
			else
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );

				k  += 1;

				if(kadd>1)
					{
					
					// k = 1
					a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
					b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
					a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
						b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
						b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
						d_0     = _mm256_add_pd( d_0, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
						a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
						d_1     = _mm256_add_pd( d_1, ab_tmp0 );

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Ap1[12] ); // prefetch
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Ap1[16] ); // prefetch
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			
			Ap0 += 16;
			Ap1 += 16;
			Bp  += 16;

			}
		
		for(; k<kadd-1; k+=2)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Ap1[8] ); // prefetch
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			
			
			Ap0 += 8;
			Ap1 += 8;
			Bp  += 8;

			}

		for(; k<kadd; k+=1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
	/*		b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch*/
			d_4     = _mm256_add_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
	/*		a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch*/
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
	/*		a_4567  = _mm256_load_pd( &Ap1[4] ); // prefetch*/
			d_5     = _mm256_add_pd( d_5, ab_tmp0 );
			
//			Ap0 += 4;
//			Ap1 += 4;
//			Bp  += 4;

			}

		}
		
	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am0[0] );
		a_4567 = _mm256_load_pd( &Am1[0] );
		b_0101 = _mm256_broadcast_pd( (__m128d *) &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[4] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am0[4] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Am1[4] ); // prefetch
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[8] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Am1[8] ); // prefetch
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[12] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Am1[12] ); // prefetch
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_0101 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[16] ); // prefetch
			d_4     = _mm256_sub_pd( d_4, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_4567, b_1010 );
			a_4567  = _mm256_load_pd( &Am1[16] ); // prefetch
			d_5     = _mm256_sub_pd( d_5, ab_tmp0 );
			
			Am0 += 16;
			Am1 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0,
		d_00, d_01,
		d_40, d_41;

		d_00 = _mm256_blend_pd( d_0, d_1, 0xa );
		d_01 = _mm256_blend_pd( d_0, d_1, 0x5 );
		d_40 = _mm256_blend_pd( d_4, d_5, 0xa );
		d_41 = _mm256_blend_pd( d_4, d_5, 0x5 );

	if(alg!=0)
		{

		e_0  = _mm256_load_pd( &C0[0+bs*0] );
		d_00 = _mm256_add_pd( d_00, e_0 );
		e_0  = _mm256_load_pd( &C0[0+bs*1] );
		d_01 = _mm256_add_pd( d_01, e_0 );

		e_0  = _mm256_load_pd( &C1[0+bs*0] );
		d_40 = _mm256_add_pd( d_40, e_0 );
		e_0  = _mm256_load_pd( &C1[0+bs*1] );
		d_41 = _mm256_add_pd( d_41, e_0 );

		}
		
	__m256d
		a_ii;
	
	__m128d
		s_ii, ones;
	
	d_temp = km - 4.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(use_inv_diag_E)
		{
		a_ii = _mm256_broadcast_sd( &inv_diag_E[0] ); // E_00
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_40 );

		if(kn>=2)
			{
			a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
			d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &inv_diag_E[1] ); // E_11
			d_01 = _mm256_mul_pd( d_01, a_ii );
			d_41 = _mm256_mul_pd( d_41, a_ii );
			_mm256_store_pd( &D0[0+bs*1], d_01 );
			_mm256_maskstore_pd( &D1[0+bs*1], msk, d_41 );
			}
		}
	else
		{
		ones = _mm_set_sd( 1.0 );
		s_ii = _mm_load_sd( &E[0+bs*0] ); // E_00
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_00 = _mm256_mul_pd( d_00, a_ii );
		d_40 = _mm256_mul_pd( d_40, a_ii );
		_mm256_store_pd( &D0[0+bs*0], d_00 );
		_mm256_maskstore_pd( &D1[0+bs*0], msk, d_40 );

		if(kn>=4)
			{
			a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( d_40, a_ii );
			d_41 = _mm256_sub_pd( d_41, ab_tmp0 );
			s_ii = _mm_load_sd( &inv_diag_E[1+bs*1] ); // E_11
			s_ii = _mm_div_sd( ones, s_ii );
			s_ii = _mm_movedup_pd( s_ii );
			a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
			d_01 = _mm256_mul_pd( d_01, a_ii );
			d_41 = _mm256_mul_pd( d_41, a_ii );
			_mm256_store_pd( &D0[0+bs*1], d_01 );
			_mm256_maskstore_pd( &D1[0+bs*1], msk, d_41 );
			}
		}

	return;

	}



void kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap0, double *Bp, int ksub, double *Am0, double *Bm, int alg, double *C0, double *D0, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256i
		msk;

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

	if(kadd>0)
		{

		if(tri_A)
			{

			zeros = _mm256_setzero_pd();

			if(kadd>=4)
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );
				
				// k = 1
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );

				// k = 2
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
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
				k  += 4;

				}
			else // kadd = {1 2 3}
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
				b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
				b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
				b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
				b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
				d_3     = _mm256_add_pd( d_3, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
				a_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
				d_2     = _mm256_add_pd( d_2, ab_tmp0 );

				k  += 1;

				if(kadd>1)
					{
					
					// k = 1
					a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
					b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
					b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
					b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
					b_2301  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
					d_3     = _mm256_add_pd( d_3, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_2301 );
					a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
					d_2     = _mm256_add_pd( d_2, ab_tmp0 );

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
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

						k  += 1;

						}

					}

				}


			}

		// prefetch
		a_0123 = _mm256_load_pd( &Ap0[0] );
		b_0123 = _mm256_load_pd( &Bp[0] );

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
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
			A_0123  = _mm256_load_pd( &Ap0[12] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[12] ); // prefetch
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
			b_0123  = _mm256_load_pd( &Bp[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Ap0[16] ); // prefetch
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );
			
			Ap0 += 16;
			Bp  += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
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
			a_0123  = _mm256_load_pd( &Ap0[8] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bp[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );

			
			Ap0 += 8;
			Bp  += 8;
			k   += 2;

			}
		if(k<kadd)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
//			A_0123  = _mm256_load_pd( &Ap0[4] ); // prefetch
			d_0    = _mm256_add_pd( d_0, ab_tmp0 );
//			b_0123  = _mm256_load_pd( &Bp[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_add_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_add_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_add_pd( d_2, ab_tmp0 );

		
//			Ap0 += 4;
//			Bp  += 4;

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
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			a_0123  = _mm256_load_pd( &Am0[8] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			A_0123  = _mm256_load_pd( &Am0[12] ); // prefetch
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[12] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( A_0123, b_0123 );
			b_1032  = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
			d_0    = _mm256_sub_pd( d_0, ab_tmp0 );
			b_0123  = _mm256_load_pd( &Bm[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			b_3210  = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
			a_0123  = _mm256_load_pd( &Am0[16] ); // prefetch
			d_1    = _mm256_sub_pd( d_1, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_3210 );
			b_1032  = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
			d_3    = _mm256_sub_pd( d_3, ab_tmp0 );
			ab_tmp0 = _mm256_mul_pd( A_0123, b_1032 );
			d_2    = _mm256_sub_pd( d_2, ab_tmp0 );
			
			Am0 += 16;
			Bm  += 16;

			}

		}

	__m256d
		e_0, e_1, e_2, e_3,
		d_00, d_01, d_02, d_03;

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
		
	__m256d
		a_ii;
	
	__m128d
		s_ii, ones;
	
	d_temp = km - 0.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(use_inv_diag_E)
		{
		a_ii = _mm256_broadcast_sd( &inv_diag_E[0] ); // E_00
		d_00 = _mm256_mul_pd( d_00, a_ii );
		_mm256_maskstore_pd( &D0[0+bs*0], msk, d_00 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[1] ); // E_11
		d_01 = _mm256_mul_pd( d_01, a_ii );
		_mm256_maskstore_pd( &D0[0+bs*1], msk, d_01 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &inv_diag_E[2] ); // E_22
		d_02 = _mm256_mul_pd( d_02, a_ii );
		_mm256_maskstore_pd( &D0[0+bs*2], msk, d_02 );

		if(kn>=4)
			{
			a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
			ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
			ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &inv_diag_E[3] ); // E_33
			d_03 = _mm256_mul_pd( d_03, a_ii );
			_mm256_maskstore_pd( &D0[0+bs*3], msk, d_03 );
			}
		}
	else
		{
		ones = _mm_set_sd( 1.0 );
		s_ii = _mm_load_sd( &E[0+bs*0] ); // E_00
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_00 = _mm256_mul_pd( d_00, a_ii );
		_mm256_maskstore_pd( &D0[0+bs*0], msk, d_00 );

		a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[1+bs*1] ); // E_11
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_01 = _mm256_mul_pd( d_01, a_ii );
		_mm256_maskstore_pd( &D0[0+bs*1], msk, d_01 );

		a_ii = _mm256_broadcast_sd( &E[2+bs*0] ); // E_20
		ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		a_ii = _mm256_broadcast_sd( &E[2+bs*1] ); // E_21
		ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
		d_02 = _mm256_sub_pd( d_02, ab_tmp0 );
		s_ii = _mm_load_sd( &inv_diag_E[2+bs*2] ); // E_22
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_02 = _mm256_mul_pd( d_02, a_ii );
		_mm256_maskstore_pd( &D0[0+bs*2], msk, d_02 );

		if(kn>=4)
			{
			a_ii = _mm256_broadcast_sd( &E[3+bs*0] ); // E_30
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*1] ); // E_31
			ab_tmp0 = _mm256_mul_pd( d_01, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &E[3+bs*2] ); // E_32
			ab_tmp0 = _mm256_mul_pd( d_02, a_ii );
			d_03 = _mm256_sub_pd( d_03, ab_tmp0 );
			s_ii = _mm_load_sd( &inv_diag_E[3+bs*3] ); // E_33
			s_ii = _mm_div_sd( ones, s_ii );
			s_ii = _mm_movedup_pd( s_ii );
			a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
			d_03 = _mm256_mul_pd( d_03, a_ii );
			_mm256_maskstore_pd( &D0[0+bs*3], msk, d_03 );
			}
		}

	return;

	}



void kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{
	
	const int bs = 4;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	double d_temp;
	
	int k;
	
	__m256i
		msk;

	__m256d
		a_0123,
		b_0101, b_1010,
		ab_tmp0, zeros,
		d_0, d_1,
		D_0, D_1;
	
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

			if(kadd>=4)
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				
				// k = 1
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );

				// k = 2
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[12] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );

				// k = 2
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[16] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );
				
				Ap += 16;
				Bp  += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0123  = _mm256_blend_pd( zeros, a_0123, 0x1 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
				b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
				b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
				d_0     = _mm256_add_pd( d_0, ab_tmp0 );
				ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
				a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
				d_1     = _mm256_add_pd( d_1, ab_tmp0 );

				k  += 1;

				if(kadd>1)
					{
					
					// k = 1
					a_0123  = _mm256_blend_pd( zeros, a_0123, 0x3 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
					b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
					b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
					d_0     = _mm256_add_pd( d_0, ab_tmp0 );
					ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
					a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
					d_1     = _mm256_add_pd( d_1, ab_tmp0 );

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0123  = _mm256_blend_pd( zeros, a_0123, 0x7 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
						b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
						b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
						d_0     = _mm256_add_pd( d_0, ab_tmp0 );
						ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
						a_0123  = _mm256_load_pd( &Ap[12] ); // prefetch
						d_1     = _mm256_add_pd( d_1, ab_tmp0 );

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_add_pd( D_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
			D_1     = _mm256_add_pd( D_1, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[12] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[12] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_add_pd( D_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[16] ); // prefetch
			D_1     = _mm256_add_pd( D_1, ab_tmp0 );
			
			Ap += 16;
			Bp += 16;

			}
		
		if(k<kadd-1)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_add_pd( D_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Ap[8] ); // prefetch
			D_1     = _mm256_add_pd( D_1, ab_tmp0 );
			
			
			Ap += 8;
			Bp += 8;
			k  += 2;

			}

		if(k<kadd)
			{
			
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_add_pd( d_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	/*		b_0101  = _mm256_broadcast_pd( (__m128d *) &Bp[4] ); // prefetch*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
	/*		a_0123  = _mm256_load_pd( &Ap[4] ); // prefetch*/
			d_1     = _mm256_add_pd( d_1, ab_tmp0 );
			
//			Ap += 4;
//			Bp += 4;

			}		}
		
	if(ksub>0)
		{

		// prefetch
		a_0123 = _mm256_load_pd( &Am[0] );
		b_0101 = _mm256_broadcast_pd( (__m128d *) &Bm[0] );

		for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
			{
			
	/*	__builtin_prefetch( A+32 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[4] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[4] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );
			
			
	/*	__builtin_prefetch( A+40 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_sub_pd( D_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[8] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[8] ); // prefetch
			D_1     = _mm256_sub_pd( D_1, ab_tmp0 );


	/*	__builtin_prefetch( A+48 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			d_0     = _mm256_sub_pd( d_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[12] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[12] ); // prefetch
			d_1     = _mm256_sub_pd( d_1, ab_tmp0 );


	/*	__builtin_prefetch( A+56 );*/
			ab_tmp0 = _mm256_mul_pd( a_0123, b_0101 );
			D_0     = _mm256_sub_pd( D_0, ab_tmp0 );
			b_1010  = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
			b_0101  = _mm256_broadcast_pd( (__m128d *) &Bm[16] ); // prefetch
			ab_tmp0 = _mm256_mul_pd( a_0123, b_1010 );
			a_0123  = _mm256_load_pd( &Am[16] ); // prefetch
			D_1     = _mm256_sub_pd( D_1, ab_tmp0 );
			
			Am += 16;
			Bm += 16;

			}

		}

	d_0 = _mm256_add_pd( d_0, D_0 );
	d_1 = _mm256_add_pd( d_1, D_1 );

	__m256d
		e_0,
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
		
	__m256d
		a_ii;
	
	__m128d
		s_ii, ones;
	
	d_temp = km - 0.0;
	msk = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &d_temp ) ) );

	if(use_inv_diag_E)
		{
		a_ii = _mm256_broadcast_sd( &inv_diag_E[0] ); // E_00
		d_00 = _mm256_mul_pd( d_00, a_ii );
		_mm256_maskstore_pd( &D[0+bs*0], msk, d_00 );

		if(kn>=2)
			{
			a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
			a_ii = _mm256_broadcast_sd( &inv_diag_E[1] ); // E_11
			d_01 = _mm256_mul_pd( d_01, a_ii );
			_mm256_maskstore_pd( &D[0+bs*1], msk, d_01 );
			}
		}
	else
		{
		ones = _mm_set_sd( 1.0 );
		s_ii = _mm_load_sd( &E[0+bs*0] ); // E_00
		s_ii = _mm_div_sd( ones, s_ii );
		s_ii = _mm_movedup_pd( s_ii );
		a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
		d_00 = _mm256_mul_pd( d_00, a_ii );
		_mm256_maskstore_pd( &D[0+bs*0], msk, d_00 );

		if(kn>=4)
			{
			a_ii = _mm256_broadcast_sd( &E[1+bs*0] ); // E_10
			ab_tmp0 = _mm256_mul_pd( d_00, a_ii );
			d_01 = _mm256_sub_pd( d_01, ab_tmp0 );
			s_ii = _mm_load_sd( &inv_diag_E[1+bs*1] ); // E_11
			s_ii = _mm_div_sd( ones, s_ii );
			s_ii = _mm_movedup_pd( s_ii );
			a_ii  = _mm256_permute2f128_pd( _mm256_castpd128_pd256( s_ii ), _mm256_castpd128_pd256( s_ii ), 0x0 );
			d_01 = _mm256_mul_pd( d_01, a_ii );
			_mm256_maskstore_pd( &D[0+bs*1], msk, d_01 );
			}
		}

	return;

	}







