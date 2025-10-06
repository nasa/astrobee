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
* MERCHANTAA_00_invILITY or FITNESS FOR A PARTICULAR PURPOSE.                                     *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, A_00_invoston, MA  02110-1301  USA           *
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




void corner_dtrtri_12x4_lib4(double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros, ab_temp,
		a_0, a_4, a_8,
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9, d_a, d_b;
	
	// zero registers
	zeros = _mm256_setzero_pd();
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


	a_0 = _mm256_load_pd( &A0[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[2] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	b_0 = _mm256_broadcast_sd( &B[3] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[6] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	b_0 = _mm256_broadcast_sd( &B[7] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[9] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[10] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	b_0 = _mm256_broadcast_sd( &B[11] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[13] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[14] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	b_0 = _mm256_broadcast_sd( &B[15] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[2] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
	b_0 = _mm256_broadcast_sd( &B[3] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_4 = _mm256_load_pd( &A1[4] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[6] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
	b_0 = _mm256_broadcast_sd( &B[7] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_4 = _mm256_load_pd( &A1[8] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[9] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[10] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
	b_0 = _mm256_broadcast_sd( &B[11] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[12] );
	a_4 = _mm256_load_pd( &A1[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[13] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[14] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
	b_0 = _mm256_broadcast_sd( &B[15] );
	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	

	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		}
	else
		{
		ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &E[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_8  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	d_8  = _mm256_mul_pd( d_8, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );
//	_mm256_store_pd( &C2[0+bs*0], d_8 );
	ab_temp = _mm256_load_pd( &C2[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_8, 0x1 );
	_mm256_store_pd( &C2[0+bs*0], ab_temp );

	d_9  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_9 = _mm256_fnmadd_pd( d_8, a_10, d_9 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_9 = _mm256_mul_pd( d_9, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_1 );
	_mm256_store_pd( &C1[0+bs*1], d_5 );
//	_mm256_store_pd( &C2[0+bs*1], d_9 );
	ab_temp = _mm256_load_pd( &C2[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_9, 0x3 );
	_mm256_store_pd( &C2[0+bs*1], ab_temp );

	d_a = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
	d_a = _mm256_fnmadd_pd( d_8, a_20, d_a );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
	d_a = _mm256_fnmadd_pd( d_9, a_21, d_a );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	d_6 = _mm256_mul_pd( d_6, a_22 );
	d_a = _mm256_mul_pd( d_a, a_22 );
	_mm256_store_pd( &C0[0+bs*2], d_2 );
	_mm256_store_pd( &C1[0+bs*2], d_6 );
//	_mm256_store_pd( &C2[0+bs*2], d_a );
	ab_temp = _mm256_load_pd( &C2[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_a, 0x7 );
	_mm256_store_pd( &C2[0+bs*2], ab_temp );

	d_b = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
	a_30 = _mm256_broadcast_sd( &E[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &E[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &E[3+bs*2] );
	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
	d_b = _mm256_fnmadd_pd( d_8, a_30, d_b );
	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
	d_b = _mm256_fnmadd_pd( d_9, a_31, d_b );
	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
	d_b = _mm256_fnmadd_pd( d_a, a_32, d_b );
	d_3 = _mm256_mul_pd( d_3, a_33 );
	d_7 = _mm256_mul_pd( d_7, a_33 );
	d_b = _mm256_mul_pd( d_b, a_33 );
	_mm256_store_pd( &C0[0+bs*3], d_3 );
	_mm256_store_pd( &C1[0+bs*3], d_7 );
	_mm256_store_pd( &C2[0+bs*3], d_b );



	}



void corner_dtrtri_11x3_lib4(double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)	
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros, ab_temp,
		a_0, a_4, a_8,
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9, d_a, d_b;
	
	// zero registers
	zeros = _mm256_setzero_pd();
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
//	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
//	d_7 = _mm256_setzero_pd();
	d_8 = _mm256_setzero_pd();
	d_9 = _mm256_setzero_pd();
	d_a = _mm256_setzero_pd();
//	d_b = _mm256_setzero_pd();


	a_0 = _mm256_load_pd( &A0[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[2] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[3] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[6] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[7] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[9] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[10] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[11] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[13] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	b_0 = _mm256_broadcast_sd( &B[14] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[15] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[2] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[3] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_4 = _mm256_load_pd( &A1[4] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[6] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[7] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_4 = _mm256_load_pd( &A1[8] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[9] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[10] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[11] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[12] );
	a_4 = _mm256_load_pd( &A1[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[13] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
	b_0 = _mm256_broadcast_sd( &B[14] );
	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[15] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	

	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
//		a_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		}
	else
		{
		ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
//		a_33 = _mm256_broadcast_sd( &E[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
//		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_8  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	d_8  = _mm256_mul_pd( d_8, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );
	ab_temp = _mm256_load_pd( &C2[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_8, 0x1 );
	_mm256_store_pd( &C2[0+bs*0], ab_temp );

	d_9  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_9 = _mm256_fnmadd_pd( d_8, a_10, d_9 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_9 = _mm256_mul_pd( d_9, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_1 );
	_mm256_store_pd( &C1[0+bs*1], d_5 );
	ab_temp = _mm256_load_pd( &C2[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_9, 0x3 );
	_mm256_store_pd( &C2[0+bs*1], ab_temp );

	d_a = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
	d_a = _mm256_fnmadd_pd( d_8, a_20, d_a );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
	d_a = _mm256_fnmadd_pd( d_9, a_21, d_a );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	d_6 = _mm256_mul_pd( d_6, a_22 );
	d_a = _mm256_mul_pd( d_a, a_22 );
	_mm256_store_pd( &C0[0+bs*2], d_2 );
	_mm256_store_pd( &C1[0+bs*2], d_6 );
	ab_temp = _mm256_load_pd( &C2[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_a, 0x7 );
	_mm256_store_pd( &C2[0+bs*2], ab_temp );

//	d_b = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
//	a_30 = _mm256_broadcast_sd( &E[3+bs*0] );
//	a_31 = _mm256_broadcast_sd( &E[3+bs*1] );
//	a_32 = _mm256_broadcast_sd( &E[3+bs*2] );
//	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
//	d_b = _mm256_fnmadd_pd( d_8, a_30, d_b );
//	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
//	d_b = _mm256_fnmadd_pd( d_9, a_31, d_b );
//	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
//	d_b = _mm256_fnmadd_pd( d_a, a_32, d_b );
//	d_3 = _mm256_mul_pd( d_3, a_33 );
//	d_7 = _mm256_mul_pd( d_7, a_33 );
//	d_b = _mm256_mul_pd( d_b, a_33 );
//	_mm256_store_pd( &C0[0+bs*3], d_3 );
//	_mm256_store_pd( &C1[0+bs*3], d_7 );
//	_mm256_store_pd( &C2[0+bs*3], d_b );

	}



void corner_dtrtri_10x2_lib4(double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)	
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros, ab_temp,
		a_0, a_4, a_8,
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9, d_a, d_b;
	
	// zero registers
	zeros = _mm256_setzero_pd();
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
//	d_2 = _mm256_setzero_pd();
//	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
//	d_6 = _mm256_setzero_pd();
//	d_7 = _mm256_setzero_pd();
	d_8 = _mm256_setzero_pd();
	d_9 = _mm256_setzero_pd();
//	d_a = _mm256_setzero_pd();
//	d_b = _mm256_setzero_pd();


	a_0 = _mm256_load_pd( &A0[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[2] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[3] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[6] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[7] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[9] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[10] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[11] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	b_0 = _mm256_broadcast_sd( &B[13] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[14] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[15] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[1] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[2] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[3] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_4 = _mm256_load_pd( &A1[4] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[6] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[7] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_4 = _mm256_load_pd( &A1[8] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[9] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[10] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[11] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[12] );
	a_4 = _mm256_load_pd( &A1[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
	b_0 = _mm256_broadcast_sd( &B[13] );
	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[14] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[15] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	

	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
//		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
//		a_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		}
	else
		{
		ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
//		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
//		a_33 = _mm256_broadcast_sd( &E[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
//		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
//		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_8  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	d_8  = _mm256_mul_pd( d_8, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );
	ab_temp = _mm256_load_pd( &C2[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_8, 0x1 );
	_mm256_store_pd( &C2[0+bs*0], ab_temp );

	d_9  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_9 = _mm256_fnmadd_pd( d_8, a_10, d_9 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_9 = _mm256_mul_pd( d_9, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_1 );
	_mm256_store_pd( &C1[0+bs*1], d_5 );
	ab_temp = _mm256_load_pd( &C2[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_9, 0x3 );
	_mm256_store_pd( &C2[0+bs*1], ab_temp );

//	d_a = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
//	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
//	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
//	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
//	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
//	d_a = _mm256_fnmadd_pd( d_8, a_20, d_a );
//	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
//	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
//	d_a = _mm256_fnmadd_pd( d_9, a_21, d_a );
//	d_2 = _mm256_mul_pd( d_2, a_22 );
//	d_6 = _mm256_mul_pd( d_6, a_22 );
//	d_a = _mm256_mul_pd( d_a, a_22 );
//	_mm256_store_pd( &C0[0+bs*2], d_2 );
//	_mm256_store_pd( &C1[0+bs*2], d_6 );
//	ab_temp = _mm256_load_pd( &C2[0+bs*2] );
//	ab_temp = _mm256_blend_pd( ab_temp, d_a, 0x7 );
//	_mm256_store_pd( &C2[0+bs*2], ab_temp );

//	d_b = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
//	a_30 = _mm256_broadcast_sd( &E[3+bs*0] );
//	a_31 = _mm256_broadcast_sd( &E[3+bs*1] );
//	a_32 = _mm256_broadcast_sd( &E[3+bs*2] );
//	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
//	d_b = _mm256_fnmadd_pd( d_8, a_30, d_b );
//	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
//	d_b = _mm256_fnmadd_pd( d_9, a_31, d_b );
//	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
//	d_b = _mm256_fnmadd_pd( d_a, a_32, d_b );
//	d_3 = _mm256_mul_pd( d_3, a_33 );
//	d_7 = _mm256_mul_pd( d_7, a_33 );
//	d_b = _mm256_mul_pd( d_b, a_33 );
//	_mm256_store_pd( &C0[0+bs*3], d_3 );
//	_mm256_store_pd( &C1[0+bs*3], d_7 );
//	_mm256_store_pd( &C2[0+bs*3], d_b );

	}



void corner_dtrtri_9x1_lib4(double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)	
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros, ab_temp,
		a_0, a_4, a_8,
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		d_8, d_9, d_a, d_b;
	
	// zero registers
	zeros = _mm256_setzero_pd();
	d_0 = _mm256_setzero_pd();
//	d_1 = _mm256_setzero_pd();
//	d_2 = _mm256_setzero_pd();
//	d_3 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
//	d_5 = _mm256_setzero_pd();
//	d_6 = _mm256_setzero_pd();
//	d_7 = _mm256_setzero_pd();
	d_8 = _mm256_setzero_pd();
//	d_9 = _mm256_setzero_pd();
//	d_a = _mm256_setzero_pd();
//	d_b = _mm256_setzero_pd();


	a_0 = _mm256_load_pd( &A0[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
//	b_0 = _mm256_broadcast_sd( &B[1] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[2] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[3] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
//	b_0 = _mm256_broadcast_sd( &B[5] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[6] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[7] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
//	b_0 = _mm256_broadcast_sd( &B[9] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[10] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[11] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );


	a_0 = _mm256_load_pd( &A0[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
//	b_0 = _mm256_broadcast_sd( &B[13] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	b_0 = _mm256_broadcast_sd( &B[14] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	b_0 = _mm256_broadcast_sd( &B[15] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x1 );
	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
//	b_0 = _mm256_broadcast_sd( &B[1] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[2] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[3] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[4] );
	a_4 = _mm256_load_pd( &A1[4] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x3 );
	b_0 = _mm256_broadcast_sd( &B[4] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
//	b_0 = _mm256_broadcast_sd( &B[5] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[6] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[7] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[8] );
	a_4 = _mm256_load_pd( &A1[8] );
	a_4 = _mm256_blend_pd( zeros, a_4, 0x7 );
	b_0 = _mm256_broadcast_sd( &B[8] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
//	b_0 = _mm256_broadcast_sd( &B[9] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[10] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[11] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	a_0 = _mm256_load_pd( &A0[12] );
	a_4 = _mm256_load_pd( &A1[12] );
	b_0 = _mm256_broadcast_sd( &B[12] );
	d_0 = _mm256_fnmadd_pd( a_0, b_0, d_0 );
	d_4 = _mm256_fnmadd_pd( a_4, b_0, d_4 );
//	b_0 = _mm256_broadcast_sd( &B[13] );
//	d_1 = _mm256_fnmadd_pd( a_0, b_0, d_1 );
//	d_5 = _mm256_fnmadd_pd( a_4, b_0, d_5 );
//	b_0 = _mm256_broadcast_sd( &B[14] );
//	d_2 = _mm256_fnmadd_pd( a_0, b_0, d_2 );
//	d_6 = _mm256_fnmadd_pd( a_4, b_0, d_6 );
//	b_0 = _mm256_broadcast_sd( &B[15] );
//	d_3 = _mm256_fnmadd_pd( a_0, b_0, d_3 );
//	d_7 = _mm256_fnmadd_pd( a_4, b_0, d_7 );


	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	

	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
//		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
//		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
//		a_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		}
	else
		{
		ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
//		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
//		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
//		a_33 = _mm256_broadcast_sd( &E[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
//		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
//		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
//		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_8  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	d_8  = _mm256_mul_pd( d_8, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );
	ab_temp = _mm256_load_pd( &C2[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_8, 0x1 );
	_mm256_store_pd( &C2[0+bs*0], ab_temp );

//	d_9  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
//	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
//	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
//	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
//	d_9 = _mm256_fnmadd_pd( d_8, a_10, d_9 );
//	d_1 = _mm256_mul_pd( d_1, a_11 );
//	d_5 = _mm256_mul_pd( d_5, a_11 );
//	d_9 = _mm256_mul_pd( d_9, a_11 );
//	_mm256_store_pd( &C0[0+bs*1], d_1 );
//	_mm256_store_pd( &C1[0+bs*1], d_5 );
//	ab_temp = _mm256_load_pd( &C2[0+bs*1] );
//	ab_temp = _mm256_blend_pd( ab_temp, d_9, 0x3 );
//	_mm256_store_pd( &C2[0+bs*1], ab_temp );

//	d_a = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
//	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
//	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
//	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
//	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
//	d_a = _mm256_fnmadd_pd( d_8, a_20, d_a );
//	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
//	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
//	d_a = _mm256_fnmadd_pd( d_9, a_21, d_a );
//	d_2 = _mm256_mul_pd( d_2, a_22 );
//	d_6 = _mm256_mul_pd( d_6, a_22 );
//	d_a = _mm256_mul_pd( d_a, a_22 );
//	_mm256_store_pd( &C0[0+bs*2], d_2 );
//	_mm256_store_pd( &C1[0+bs*2], d_6 );
//	ab_temp = _mm256_load_pd( &C2[0+bs*2] );
//	ab_temp = _mm256_blend_pd( ab_temp, d_a, 0x7 );
//	_mm256_store_pd( &C2[0+bs*2], ab_temp );

//	d_b = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
//	a_30 = _mm256_broadcast_sd( &E[3+bs*0] );
//	a_31 = _mm256_broadcast_sd( &E[3+bs*1] );
//	a_32 = _mm256_broadcast_sd( &E[3+bs*2] );
//	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
//	d_b = _mm256_fnmadd_pd( d_8, a_30, d_b );
//	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
//	d_b = _mm256_fnmadd_pd( d_9, a_31, d_b );
//	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
//	d_b = _mm256_fnmadd_pd( d_a, a_32, d_b );
//	d_3 = _mm256_mul_pd( d_3, a_33 );
//	d_7 = _mm256_mul_pd( d_7, a_33 );
//	d_b = _mm256_mul_pd( d_b, a_33 );
//	_mm256_store_pd( &C0[0+bs*3], d_3 );
//	_mm256_store_pd( &C1[0+bs*3], d_7 );
//	_mm256_store_pd( &C2[0+bs*3], d_b );

	}



void corner_dtrtri_8x8_lib4(double *A0, int sda, int use_inv_diag_A, double *inv_diag_A, double *C0, int sdc)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		ones, ab_temp, zeros,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	zeros = _mm256_setzero_pd();
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	// top-left triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_A[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A0[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &A0[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &A0[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &A0[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 );
		a_11 = _mm256_div_pd( ones, a_11 );
		a_22 = _mm256_div_pd( ones, a_22 );
		a_33 = _mm256_div_pd( ones, a_33 );
		}
	
//	d_0 = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
//	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_0 = _mm256_blend_pd( zeros, a_00, 0x1 );
	//_mm256_store_pd( &C[0+bs*0], d_0 );
	ab_temp = _mm256_load_pd( &C0[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C0[0+bs*0], ab_temp );

	d_1 = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &A0[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	//_mm256_store_pd( &C[0+bs*1], d_1 );
	ab_temp = _mm256_load_pd( &C0[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C0[0+bs*1], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A0[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &A0[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	//_mm256_store_pd( &C[0+bs*2], d_2 );
	ab_temp = _mm256_load_pd( &C0[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C0[0+bs*2], ab_temp );

	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
	a_30 = _mm256_broadcast_sd( &A0[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &A0[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &A0[3+bs*2] );
	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
	d_3 = _mm256_mul_pd( d_3, a_33 );
	_mm256_store_pd( &C0[0+bs*3], d_3 );



	// correction of top-right square

	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_7 = _mm256_setzero_pd();

	b_0     = _mm256_broadcast_sd( &A1[0] );
	d_4     = _mm256_fnmadd_pd( d_0, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[1] );
	d_5     = _mm256_fnmadd_pd( d_0, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[2] );
	d_6     = _mm256_fnmadd_pd( d_0, b_0, d_6 );
	b_0     = _mm256_broadcast_sd( &A1[3] );
	d_7     = _mm256_fnmadd_pd( d_0, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[4] );
	d_4     = _mm256_fnmadd_pd( d_1, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[5] );
	d_5     = _mm256_fnmadd_pd( d_1, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[6] );
	d_6     = _mm256_fnmadd_pd( d_1, b_0, d_6 );
	b_0     = _mm256_broadcast_sd( &A1[7] );
	d_7     = _mm256_fnmadd_pd( d_1, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[8] );
	d_4     = _mm256_fnmadd_pd( d_2, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[9] );
	d_5     = _mm256_fnmadd_pd( d_2, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[10] );
	d_6     = _mm256_fnmadd_pd( d_2, b_0, d_6 );
	b_0     = _mm256_broadcast_sd( &A1[11] );
	d_7     = _mm256_fnmadd_pd( d_2, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[12] );
	d_4     = _mm256_fnmadd_pd( d_3, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[13] );
	d_5     = _mm256_fnmadd_pd( d_3, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[14] );
	d_6     = _mm256_fnmadd_pd( d_3, b_0, d_6 );
	b_0     = _mm256_broadcast_sd( &A1[15] );
	d_7     = _mm256_fnmadd_pd( d_3, b_0, d_7 );


	// solution of top-right square & bottom-right triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[4] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[5] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[6] );
		a_33 = _mm256_broadcast_sd( &inv_diag_A[7] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A1[0+bs*4] );
		a_11 = _mm256_broadcast_sd( &A1[1+bs*5] );
		a_22 = _mm256_broadcast_sd( &A1[2+bs*6] );
		a_33 = _mm256_broadcast_sd( &A1[3+bs*7] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_0  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	_mm256_store_pd( &C0[0+bs*4], d_4 );
//	_mm256_store_pd( &C1[0+bs*4], d_0 );
	ab_temp = _mm256_load_pd( &C1[0+bs*4] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C1[0+bs*4], ab_temp );

	a_10 = _mm256_broadcast_sd( &A1[1+bs*4] );
	d_1  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	_mm256_store_pd( &C0[0+bs*5], d_5 );
//	_mm256_store_pd( &C1[0+bs*5], d_1 );
	ab_temp = _mm256_load_pd( &C1[0+bs*5] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C1[0+bs*5], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A1[2+bs*4] );
	a_21 = _mm256_broadcast_sd( &A1[2+bs*5] );
	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_6 = _mm256_mul_pd( d_6, a_22 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	_mm256_store_pd( &C0[0+bs*6], d_6 );
//	_mm256_store_pd( &C1[0+bs*6], d_2 );
	ab_temp = _mm256_load_pd( &C1[0+bs*6] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C1[0+bs*6], ab_temp );

	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
	a_30 = _mm256_broadcast_sd( &A1[3+bs*4] );
	a_31 = _mm256_broadcast_sd( &A1[3+bs*5] );
	a_32 = _mm256_broadcast_sd( &A1[3+bs*6] );
	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
	d_7 = _mm256_mul_pd( d_7, a_33 );
	d_3 = _mm256_mul_pd( d_3, a_33 );
	_mm256_store_pd( &C0[0+bs*7], d_7 );
	_mm256_store_pd( &C1[0+bs*7], d_3 );


	}



void corner_dtrtri_7x7_lib4(double *A0, int sda, int use_inv_diag_A, double *inv_diag_A, double *C0, int sdc)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		ones, ab_temp, zeros,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	zeros = _mm256_setzero_pd();
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	// top-left triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_A[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A0[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &A0[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &A0[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &A0[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 );
		a_11 = _mm256_div_pd( ones, a_11 );
		a_22 = _mm256_div_pd( ones, a_22 );
		a_33 = _mm256_div_pd( ones, a_33 );
		}
	
//	d_0 = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
//	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_0 = _mm256_blend_pd( zeros, a_00, 0x1 );
	//_mm256_store_pd( &C[0+bs*0], d_0 );
	ab_temp = _mm256_load_pd( &C0[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C0[0+bs*0], ab_temp );

	d_1 = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &A0[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	//_mm256_store_pd( &C[0+bs*1], d_1 );
	ab_temp = _mm256_load_pd( &C0[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C0[0+bs*1], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A0[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &A0[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	//_mm256_store_pd( &C[0+bs*2], d_2 );
	ab_temp = _mm256_load_pd( &C0[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C0[0+bs*2], ab_temp );

	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
	a_30 = _mm256_broadcast_sd( &A0[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &A0[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &A0[3+bs*2] );
	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
	d_3 = _mm256_mul_pd( d_3, a_33 );
	_mm256_store_pd( &C0[0+bs*3], d_3 );



	// correction of top-right square

	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
//	d_7 = _mm256_setzero_pd();

	b_0     = _mm256_broadcast_sd( &A1[0] );
	d_4     = _mm256_fnmadd_pd( d_0, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[1] );
	d_5     = _mm256_fnmadd_pd( d_0, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[2] );
	d_6     = _mm256_fnmadd_pd( d_0, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[3] );
//	d_7     = _mm256_fnmadd_pd( d_0, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[4] );
	d_4     = _mm256_fnmadd_pd( d_1, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[5] );
	d_5     = _mm256_fnmadd_pd( d_1, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[6] );
	d_6     = _mm256_fnmadd_pd( d_1, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[7] );
//	d_7     = _mm256_fnmadd_pd( d_1, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[8] );
	d_4     = _mm256_fnmadd_pd( d_2, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[9] );
	d_5     = _mm256_fnmadd_pd( d_2, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[10] );
	d_6     = _mm256_fnmadd_pd( d_2, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[11] );
//	d_7     = _mm256_fnmadd_pd( d_2, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[12] );
	d_4     = _mm256_fnmadd_pd( d_3, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[13] );
	d_5     = _mm256_fnmadd_pd( d_3, b_0, d_5 );
	b_0     = _mm256_broadcast_sd( &A1[14] );
	d_6     = _mm256_fnmadd_pd( d_3, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[15] );
//	d_7     = _mm256_fnmadd_pd( d_3, b_0, d_7 );


	// solution of top-left square & bottom-right triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[4] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[5] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[6] );
//		a_33 = _mm256_broadcast_sd( &inv_diag_A[7] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A1[0+bs*4] );
		a_11 = _mm256_broadcast_sd( &A1[1+bs*5] );
		a_22 = _mm256_broadcast_sd( &A1[2+bs*6] );
//		a_33 = _mm256_broadcast_sd( &A1[3+bs*7] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
//		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_0  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	_mm256_store_pd( &C0[0+bs*4], d_4 );
//	_mm256_store_pd( &C1[0+bs*4], d_0 );
	ab_temp = _mm256_load_pd( &C1[0+bs*4] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C1[0+bs*4], ab_temp );

	a_10 = _mm256_broadcast_sd( &A1[1+bs*4] );
	d_1  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	_mm256_store_pd( &C0[0+bs*5], d_5 );
//	_mm256_store_pd( &C1[0+bs*5], d_1 );
	ab_temp = _mm256_load_pd( &C1[0+bs*5] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C1[0+bs*5], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A1[2+bs*4] );
	a_21 = _mm256_broadcast_sd( &A1[2+bs*5] );
	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_6 = _mm256_mul_pd( d_6, a_22 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	_mm256_store_pd( &C0[0+bs*6], d_6 );
//	_mm256_store_pd( &C1[0+bs*6], d_2 );
	ab_temp = _mm256_load_pd( &C1[0+bs*6] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C1[0+bs*6], ab_temp );

//	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
//	a_30 = _mm256_broadcast_sd( &A1[3+bs*4] );
//	a_31 = _mm256_broadcast_sd( &A1[3+bs*5] );
//	a_32 = _mm256_broadcast_sd( &A1[3+bs*6] );
//	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
//	d_7 = _mm256_mul_pd( d_7, a_33 );
//	d_3 = _mm256_mul_pd( d_3, a_33 );
//	_mm256_store_pd( &C0[0+bs*7], d_7 );
//	_mm256_store_pd( &C1[0+bs*7], d_3 );


	}



void corner_dtrtri_6x6_lib4(double *A0, int sda, int use_inv_diag_A, double *inv_diag_A, double *C0, int sdc)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		ones, ab_temp, zeros,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	zeros = _mm256_setzero_pd();
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	// top-left triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_A[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A0[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &A0[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &A0[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &A0[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 );
		a_11 = _mm256_div_pd( ones, a_11 );
		a_22 = _mm256_div_pd( ones, a_22 );
		a_33 = _mm256_div_pd( ones, a_33 );
		}
	
//	d_0 = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
//	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_0 = _mm256_blend_pd( zeros, a_00, 0x1 );
	//_mm256_store_pd( &C[0+bs*0], d_0 );
	ab_temp = _mm256_load_pd( &C0[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C0[0+bs*0], ab_temp );

	d_1 = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &A0[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	//_mm256_store_pd( &C[0+bs*1], d_1 );
	ab_temp = _mm256_load_pd( &C0[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C0[0+bs*1], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A0[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &A0[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	//_mm256_store_pd( &C[0+bs*2], d_2 );
	ab_temp = _mm256_load_pd( &C0[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C0[0+bs*2], ab_temp );

	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
	a_30 = _mm256_broadcast_sd( &A0[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &A0[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &A0[3+bs*2] );
	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
	d_3 = _mm256_mul_pd( d_3, a_33 );
	_mm256_store_pd( &C0[0+bs*3], d_3 );



	// correction of top-right square

	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
//	d_6 = _mm256_setzero_pd();
//	d_7 = _mm256_setzero_pd();

	b_0     = _mm256_broadcast_sd( &A1[0] );
	d_4     = _mm256_fnmadd_pd( d_0, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[1] );
	d_5     = _mm256_fnmadd_pd( d_0, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[2] );
//	d_6     = _mm256_fnmadd_pd( d_0, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[3] );
//	d_7     = _mm256_fnmadd_pd( d_0, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[4] );
	d_4     = _mm256_fnmadd_pd( d_1, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[5] );
	d_5     = _mm256_fnmadd_pd( d_1, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[6] );
//	d_6     = _mm256_fnmadd_pd( d_1, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[7] );
//	d_7     = _mm256_fnmadd_pd( d_1, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[8] );
	d_4     = _mm256_fnmadd_pd( d_2, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[9] );
	d_5     = _mm256_fnmadd_pd( d_2, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[10] );
//	d_6     = _mm256_fnmadd_pd( d_2, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[11] );
//	d_7     = _mm256_fnmadd_pd( d_2, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[12] );
	d_4     = _mm256_fnmadd_pd( d_3, b_0, d_4 );
	b_0     = _mm256_broadcast_sd( &A1[13] );
	d_5     = _mm256_fnmadd_pd( d_3, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[14] );
//	d_6     = _mm256_fnmadd_pd( d_3, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[15] );
//	d_7     = _mm256_fnmadd_pd( d_3, b_0, d_7 );


	// solution of top-left square & bottom-right triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[4] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[5] );
//		a_22 = _mm256_broadcast_sd( &inv_diag_A[6] );
//		a_33 = _mm256_broadcast_sd( &inv_diag_A[7] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A1[0+bs*4] );
		a_11 = _mm256_broadcast_sd( &A1[1+bs*5] );
//		a_22 = _mm256_broadcast_sd( &A1[2+bs*6] );
//		a_33 = _mm256_broadcast_sd( &A1[3+bs*7] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
//		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
//		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_0  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	_mm256_store_pd( &C0[0+bs*4], d_4 );
//	_mm256_store_pd( &C1[0+bs*4], d_0 );
	ab_temp = _mm256_load_pd( &C1[0+bs*4] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C1[0+bs*4], ab_temp );

	a_10 = _mm256_broadcast_sd( &A1[1+bs*4] );
	d_1  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	_mm256_store_pd( &C0[0+bs*5], d_5 );
//	_mm256_store_pd( &C1[0+bs*5], d_1 );
	ab_temp = _mm256_load_pd( &C1[0+bs*5] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C1[0+bs*5], ab_temp );

//	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
//	a_20 = _mm256_broadcast_sd( &A1[2+bs*4] );
//	a_21 = _mm256_broadcast_sd( &A1[2+bs*5] );
//	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
//	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
//	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
//	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
//	d_6 = _mm256_mul_pd( d_6, a_22 );
//	d_2 = _mm256_mul_pd( d_2, a_22 );
//	_mm256_store_pd( &C0[0+bs*6], d_6 );
//	ab_temp = _mm256_load_pd( &C1[0+bs*6] );
//	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
//	_mm256_store_pd( &C1[0+bs*6], ab_temp );

//	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
//	a_30 = _mm256_broadcast_sd( &A1[3+bs*4] );
//	a_31 = _mm256_broadcast_sd( &A1[3+bs*5] );
//	a_32 = _mm256_broadcast_sd( &A1[3+bs*6] );
//	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
//	d_7 = _mm256_mul_pd( d_7, a_33 );
//	d_3 = _mm256_mul_pd( d_3, a_33 );
//	_mm256_store_pd( &C0[0+bs*7], d_7 );
//	_mm256_store_pd( &C1[0+bs*7], d_3 );


	}



void corner_dtrtri_5x5_lib4(double *A0, int sda, int use_inv_diag_A, double *inv_diag_A, double *C0, int sdc)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		b_0,
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		ones, ab_temp, zeros,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	zeros = _mm256_setzero_pd();
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	// top-left triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_A[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A0[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &A0[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &A0[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &A0[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 );
		a_11 = _mm256_div_pd( ones, a_11 );
		a_22 = _mm256_div_pd( ones, a_22 );
		a_33 = _mm256_div_pd( ones, a_33 );
		}
	
//	d_0 = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
//	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_0 = _mm256_blend_pd( zeros, a_00, 0x1 );
	//_mm256_store_pd( &C[0+bs*0], d_0 );
	ab_temp = _mm256_load_pd( &C0[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C0[0+bs*0], ab_temp );

	d_1 = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &A0[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	//_mm256_store_pd( &C[0+bs*1], d_1 );
	ab_temp = _mm256_load_pd( &C0[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C0[0+bs*1], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A0[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &A0[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	//_mm256_store_pd( &C[0+bs*2], d_2 );
	ab_temp = _mm256_load_pd( &C0[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C0[0+bs*2], ab_temp );

	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
	a_30 = _mm256_broadcast_sd( &A0[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &A0[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &A0[3+bs*2] );
	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
	d_3 = _mm256_mul_pd( d_3, a_33 );
	_mm256_store_pd( &C0[0+bs*3], d_3 );



	// correction of top-right square

	d_4 = _mm256_setzero_pd();
//	d_5 = _mm256_setzero_pd();
//	d_6 = _mm256_setzero_pd();
//	d_7 = _mm256_setzero_pd();

	b_0     = _mm256_broadcast_sd( &A1[0] );
	d_4     = _mm256_fnmadd_pd( d_0, b_0, d_4 );
//	b_0     = _mm256_broadcast_sd( &A1[1] );
//	d_5     = _mm256_fnmadd_pd( d_0, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[2] );
//	d_6     = _mm256_fnmadd_pd( d_0, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[3] );
//	d_7     = _mm256_fnmadd_pd( d_0, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[4] );
	d_4     = _mm256_fnmadd_pd( d_1, b_0, d_4 );
//	b_0     = _mm256_broadcast_sd( &A1[5] );
//	d_5     = _mm256_fnmadd_pd( d_1, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[6] );
//	d_6     = _mm256_fnmadd_pd( d_1, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[7] );
//	d_7     = _mm256_fnmadd_pd( d_1, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[8] );
	d_4     = _mm256_fnmadd_pd( d_2, b_0, d_4 );
//	b_0     = _mm256_broadcast_sd( &A1[9] );
//	d_5     = _mm256_fnmadd_pd( d_2, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[10] );
//	d_6     = _mm256_fnmadd_pd( d_2, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[11] );
//	d_7     = _mm256_fnmadd_pd( d_2, b_0, d_7 );


	b_0     = _mm256_broadcast_sd( &A1[12] );
	d_4     = _mm256_fnmadd_pd( d_3, b_0, d_4 );
//	b_0     = _mm256_broadcast_sd( &A1[13] );
//	d_5     = _mm256_fnmadd_pd( d_3, b_0, d_5 );
//	b_0     = _mm256_broadcast_sd( &A1[14] );
//	d_6     = _mm256_fnmadd_pd( d_3, b_0, d_6 );
//	b_0     = _mm256_broadcast_sd( &A1[15] );
//	d_7     = _mm256_fnmadd_pd( d_3, b_0, d_7 );


	// solution of top-left square & bottom-right triangle

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[4] );
//		a_11 = _mm256_broadcast_sd( &inv_diag_A[5] );
//		a_22 = _mm256_broadcast_sd( &inv_diag_A[6] );
//		a_33 = _mm256_broadcast_sd( &inv_diag_A[7] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A1[0+bs*4] );
//		a_11 = _mm256_broadcast_sd( &A1[1+bs*5] );
//		a_22 = _mm256_broadcast_sd( &A1[2+bs*6] );
//		a_33 = _mm256_broadcast_sd( &A1[3+bs*7] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
//		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
//		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
//		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_0  = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	d_0  = _mm256_mul_pd( d_0, a_00 );
	d_4  = _mm256_mul_pd( d_4, a_00 );
	_mm256_store_pd( &C0[0+bs*4], d_4 );
//	_mm256_store_pd( &C1[0+bs*4], d_0 );
	ab_temp = _mm256_load_pd( &C1[0+bs*4] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C1[0+bs*4], ab_temp );

//	a_10 = _mm256_broadcast_sd( &A1[1+bs*4] );
//	d_1  = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
//	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
//	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
//	d_5 = _mm256_mul_pd( d_5, a_11 );
//	d_1 = _mm256_mul_pd( d_1, a_11 );
//	_mm256_store_pd( &C0[0+bs*5], d_5 );
//	ab_temp = _mm256_load_pd( &C1[0+bs*5] );
//	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
//	_mm256_store_pd( &C1[0+bs*5], ab_temp );

//	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
//	a_20 = _mm256_broadcast_sd( &A1[2+bs*4] );
//	a_21 = _mm256_broadcast_sd( &A1[2+bs*5] );
//	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
//	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
//	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
//	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
//	d_6 = _mm256_mul_pd( d_6, a_22 );
//	d_2 = _mm256_mul_pd( d_2, a_22 );
//	_mm256_store_pd( &C0[0+bs*6], d_6 );
//	ab_temp = _mm256_load_pd( &C1[0+bs*6] );
//	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
//	_mm256_store_pd( &C1[0+bs*6], ab_temp );

//	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
//	a_30 = _mm256_broadcast_sd( &A1[3+bs*4] );
//	a_31 = _mm256_broadcast_sd( &A1[3+bs*5] );
//	a_32 = _mm256_broadcast_sd( &A1[3+bs*6] );
//	d_7 = _mm256_fnmadd_pd( d_4, a_30, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_5, a_31, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
//	d_7 = _mm256_fnmadd_pd( d_6, a_32, d_7 );
//	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
//	d_7 = _mm256_mul_pd( d_7, a_33 );
//	d_3 = _mm256_mul_pd( d_3, a_33 );
//	_mm256_store_pd( &C0[0+bs*7], d_7 );
//	_mm256_store_pd( &C1[0+bs*7], d_3 );


	}



void corner_dtrtri_4x4_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	int k;

	__m256d
		d_0, d_1, d_2, d_3,
		ones, ab_temp, zeros,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	zeros = _mm256_setzero_pd();
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_A[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &A[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &A[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &A[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 );
		a_11 = _mm256_div_pd( ones, a_11 );
		a_22 = _mm256_div_pd( ones, a_22 );
		a_33 = _mm256_div_pd( ones, a_33 );
		}
	
	//d_0 = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	//d_0 = _mm256_mul_pd( d_0, a_00 );
	d_0 = _mm256_blend_pd( zeros, a_00, 0x1 );
	//_mm256_store_pd( &C[0+bs*0], d_0 );
	ab_temp = _mm256_load_pd( &C[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C[0+bs*0], ab_temp );

	d_1 = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &A[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	//_mm256_store_pd( &C[0+bs*1], d_1 );
	ab_temp = _mm256_load_pd( &C[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C[0+bs*1], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &A[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	//_mm256_store_pd( &C[0+bs*2], d_2 );
	ab_temp = _mm256_load_pd( &C[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C[0+bs*2], ab_temp );

	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
	a_30 = _mm256_broadcast_sd( &A[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &A[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &A[3+bs*2] );
	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
	d_3 = _mm256_mul_pd( d_3, a_33 );
	_mm256_store_pd( &C[0+bs*3], d_3 );

	return;

	}



void corner_dtrtri_3x3_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	int k;

	__m256d
		d_0, d_1, d_2, d_3,
		ones, ab_temp, zeros,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	zeros = _mm256_setzero_pd();
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	if(use_inv_diag_A)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_A[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_A[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_A[2] );
//		a_33 = _mm256_broadcast_sd( &inv_diag_A[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &A[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &A[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &A[2+bs*2] );
//		a_33 = _mm256_broadcast_sd( &A[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 );
		a_11 = _mm256_div_pd( ones, a_11 );
		a_22 = _mm256_div_pd( ones, a_22 );
//		a_33 = _mm256_div_pd( ones, a_33 );
		}
	
	//d_0 = _mm256_set_pd( 0.0, 0.0, 0.0, 1.0 );
	//d_0 = _mm256_mul_pd( d_0, a_00 );
	d_0 = _mm256_blend_pd( zeros, a_00, 0x1 );
	//_mm256_store_pd( &C[0+bs*0], d_0 );
	ab_temp = _mm256_load_pd( &C[0+bs*0] );
	ab_temp = _mm256_blend_pd( ab_temp, d_0, 0x1 );
	_mm256_store_pd( &C[0+bs*0], ab_temp );

	d_1 = _mm256_set_pd( 0.0, 0.0, 1.0, 0.0 );
	a_10 = _mm256_broadcast_sd( &A[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	//_mm256_store_pd( &C[0+bs*1], d_1 );
	ab_temp = _mm256_load_pd( &C[0+bs*1] );
	ab_temp = _mm256_blend_pd( ab_temp, d_1, 0x3 );
	_mm256_store_pd( &C[0+bs*1], ab_temp );

	d_2 = _mm256_set_pd( 0.0, 1.0, 0.0, 0.0 );
	a_20 = _mm256_broadcast_sd( &A[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &A[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	//_mm256_store_pd( &C[0+bs*2], d_2 );
	ab_temp = _mm256_load_pd( &C[0+bs*2] );
	ab_temp = _mm256_blend_pd( ab_temp, d_2, 0x7 );
	_mm256_store_pd( &C[0+bs*2], ab_temp );

//	d_3 = _mm256_set_pd( 1.0, 0.0, 0.0, 0.0 );
//	a_30 = _mm256_broadcast_sd( &A[3+bs*0] );
//	a_31 = _mm256_broadcast_sd( &A[3+bs*1] );
//	a_32 = _mm256_broadcast_sd( &A[3+bs*2] );
//	d_3 = _mm256_fnmadd_pd( d_0, a_30, d_3 );
//	d_3 = _mm256_fnmadd_pd( d_1, a_31, d_3 );
//	d_3 = _mm256_fnmadd_pd( d_2, a_32, d_3 );
//	d_3 = _mm256_mul_pd( d_3, a_33 );
//	_mm256_store_pd( &C[0+bs*3], d_3 );

	return;

	}



void corner_dtrtri_2x2_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	double
		c_00,
		c_10, c_11;

	if(use_inv_diag_A)
		{
		c_00 = inv_diag_A[0];
		c_11 = inv_diag_A[1];
		}
	else
		{
		c_00 = 1.0/A[0+bs*0];
		c_11 = 1.0/A[1+bs*1];
		}

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;

	c_10 = A[1+bs*0];

	c_10 = - c_11*c_10*c_00;

	C[0+bs*1] = c_10;

	return;

	}



void corner_dtrtri_1x1_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	double
		c_00,
		c_10, c_11;

	if(use_inv_diag_A)
		{
		c_00 = inv_diag_A[0];
//		c_11 = inv_diag_A[1];
		}
	else
		{
		c_00 = 1.0/A[0+bs*0];
//		c_11 = 1.0/A[1+bs*1];
		}

	C[0+bs*0] = c_00;
//	C[1+bs*1] = c_11;

//	c_10 = A[1+bs*0];

//	c_10 = - c_11*c_10*c_00;

//	C[0+bs*1] = c_10;

	return;

	}



void kernel_dtrtri_12x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
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
//	a_4 = _mm256_load_pd( &A1[0] );
//	a_8 = _mm256_load_pd( &A2[0] );
	b_0 = _mm256_load_pd( &B[0] );

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

	a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );
	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
//	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
//	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
//	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
//	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
//	a_4  = _mm256_load_pd( &A1[4] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[4] ); // prefetch
//	a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
	
	a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );
	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
//	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
//	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
//	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
//	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
//	a_4  = _mm256_load_pd( &A1[8] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[8] ); // prefetch
//	a_8  = _mm256_load_pd( &A2[8] ); // prefetch


	a_0  = _mm256_blend_pd( zeros, a_0, 0x7 );
	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
//	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
//	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
//	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
//	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
//	a_4  = _mm256_load_pd( &A1[12] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[12] ); // prefetch
//	a_8  = _mm256_load_pd( &A2[12] ); // prefetch


	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
//	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
//	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
//	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
//	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[16] ); // prefetch
//	a_8  = _mm256_load_pd( &A2[16] ); // prefetch

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_4  = _mm256_blend_pd( zeros, a_4, 0x1 );
	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[4] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[4] ); // prefetch
//	a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
	
	a_4   = _mm256_blend_pd( zeros, a_4, 0x3 );
	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[8] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[8] ); // prefetch
//	a_8  = _mm256_load_pd( &A2[8] ); // prefetch
	
	
	a_4  = _mm256_blend_pd( zeros, a_4, 0x7 );
	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[12] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[12] ); // prefetch
//	a_8  = _mm256_load_pd( &A2[12] ); // prefetch


	c_00 = _mm256_fnmadd_pd( a_0, b_0, c_00 );
	c_40 = _mm256_fnmadd_pd( a_4, b_0, c_40 );
//	c_80 = _mm256_fnmadd_pd( a_8, b_0, c_80 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fnmadd_pd( a_0, b_0, c_01 );
	c_41 = _mm256_fnmadd_pd( a_4, b_0, c_41 );
//	c_81 = _mm256_fnmadd_pd( a_8, b_0, c_81 );

	b_0  = _mm256_permute2f128_pd( b_0, b_0, 0x1 );
	c_03 = _mm256_fnmadd_pd( a_0, b_0, c_03 );
	c_43 = _mm256_fnmadd_pd( a_4, b_0, c_43 );
//	c_83 = _mm256_fnmadd_pd( a_8, b_0, c_83 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_02 = _mm256_fnmadd_pd( a_0, b_0, c_02 );
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
//	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[16] ); // prefetch
	a_8  = _mm256_load_pd( &A2[16] ); // prefetch
		
	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_8  = _mm256_blend_pd( zeros, a_8, 0x1 );
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
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[4] ); // prefetch
	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[4] ); // prefetch
	a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
	
	a_8  = _mm256_blend_pd( zeros, a_8, 0x3 );
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
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[8] ); // prefetch
	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[8] ); // prefetch
	a_8  = _mm256_load_pd( &A2[8] ); // prefetch

	
	
	a_8  = _mm256_blend_pd( zeros, a_8, 0x7 );
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
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[12] ); // prefetch
	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[12] ); // prefetch
	a_8  = _mm256_load_pd( &A2[12] ); // prefetch

	
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
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
	c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
	b_0  = _mm256_load_pd( &B[16] ); // prefetch
	a_8  = _mm256_load_pd( &A2[16] ); // prefetch
	
	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;

	
	k = 12;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
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
		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[4] ); // prefetch
		a_8  = _mm256_load_pd( &A2[4] ); // prefetch
		
		
		
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
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[8] ); // prefetch
		a_8  = _mm256_load_pd( &A2[8] ); // prefetch



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
		a_0  = _mm256_load_pd( &A0[12] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[12] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[12] ); // prefetch
		a_8  = _mm256_load_pd( &A2[12] ); // prefetch


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
		a_0  = _mm256_load_pd( &A0[16] ); // prefetch
		c_42 = _mm256_fnmadd_pd( a_4, b_0, c_42 );
		a_4  = _mm256_load_pd( &A1[16] ); // prefetch
		c_82 = _mm256_fnmadd_pd( a_8, b_0, c_82 );
		b_0  = _mm256_load_pd( &B[16] ); // prefetch
		a_8  = _mm256_load_pd( &A2[16] ); // prefetch
		
	
		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}


	__m256d
		e_00, e_01, e_02, e_03;

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


	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &E[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	c_00 = _mm256_mul_pd( c_00, a_00 );
	c_40 = _mm256_mul_pd( c_40, a_00 );
	c_80 = _mm256_mul_pd( c_80, a_00 );
	_mm256_store_pd( &C0[0+bs*0], c_00 );
	_mm256_store_pd( &C1[0+bs*0], c_40 );
	_mm256_store_pd( &C2[0+bs*0], c_80 );

	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	c_01 = _mm256_fnmadd_pd( c_00, a_10, c_01 );
	c_41 = _mm256_fnmadd_pd( c_40, a_10, c_41 );
	c_81 = _mm256_fnmadd_pd( c_80, a_10, c_81 );
	c_01 = _mm256_mul_pd( c_01, a_11 );
	c_41 = _mm256_mul_pd( c_41, a_11 );
	c_81 = _mm256_mul_pd( c_81, a_11 );
	_mm256_store_pd( &C0[0+bs*1], c_01 );
	_mm256_store_pd( &C1[0+bs*1], c_41 );
	_mm256_store_pd( &C2[0+bs*1], c_81 );

	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
	c_02 = _mm256_fnmadd_pd( c_00, a_20, c_02 );
	c_42 = _mm256_fnmadd_pd( c_40, a_20, c_42 );
	c_82 = _mm256_fnmadd_pd( c_80, a_20, c_82 );
	c_02 = _mm256_fnmadd_pd( c_01, a_21, c_02 );
	c_42 = _mm256_fnmadd_pd( c_41, a_21, c_42 );
	c_82 = _mm256_fnmadd_pd( c_81, a_21, c_82 );
	c_02 = _mm256_mul_pd( c_02, a_22 );
	c_42 = _mm256_mul_pd( c_42, a_22 );
	c_82 = _mm256_mul_pd( c_82, a_22 );
	_mm256_store_pd( &C0[0+bs*2], c_02 );
	_mm256_store_pd( &C1[0+bs*2], c_42 );
	_mm256_store_pd( &C2[0+bs*2], c_82 );

	a_30 = _mm256_broadcast_sd( &E[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &E[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &E[3+bs*2] );
	c_03 = _mm256_fnmadd_pd( c_00, a_30, c_03 );
	c_43 = _mm256_fnmadd_pd( c_40, a_30, c_43 );
	c_83 = _mm256_fnmadd_pd( c_80, a_30, c_83 );
	c_03 = _mm256_fnmadd_pd( c_01, a_31, c_03 );
	c_43 = _mm256_fnmadd_pd( c_41, a_31, c_43 );
	c_83 = _mm256_fnmadd_pd( c_81, a_31, c_83 );
	c_03 = _mm256_fnmadd_pd( c_02, a_32, c_03 );
	c_43 = _mm256_fnmadd_pd( c_42, a_32, c_43 );
	c_83 = _mm256_fnmadd_pd( c_82, a_32, c_83 );
	c_03 = _mm256_mul_pd( c_03, a_33 );
	c_43 = _mm256_mul_pd( c_43, a_33 );
	c_83 = _mm256_mul_pd( c_83, a_33 );
	_mm256_store_pd( &C0[0+bs*3], c_03 );
	_mm256_store_pd( &C1[0+bs*3], c_43 );
	_mm256_store_pd( &C2[0+bs*3], c_83 );


	}



void kernel_dtrtri_12x3_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, a_89ab,
		b_0,
		d_0, d_1, d_2,
		d_4, d_5, d_6,
		d_8, d_9, d_a;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();
	d_8 = _mm256_setzero_pd();
	d_9 = _mm256_setzero_pd();
	d_a = _mm256_setzero_pd();

	k = 0;

	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[1] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[2] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[4] );
	
	
	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[5] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[6] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[8] );


	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[9] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[10] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[12] );


	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[13] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[14] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[16] );
	a_4567 = _mm256_load_pd( &A1[16] );

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[1] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[2] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[4] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[4] );
		
	
	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[5] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[6] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[8] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[8] );
		
	
	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[9] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[10] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[12] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[12] );
		
	
	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[13] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[14] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[16] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[16] );
	a_89ab = _mm256_load_pd( &A2[16] );
		
	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_89ab = _mm256_blend_pd( zeros, a_89ab, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	b_0    = _mm256_broadcast_sd( &B[1] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
	b_0    = _mm256_broadcast_sd( &B[2] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[4] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[4] );
	d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
	a_89ab = _mm256_load_pd( &A2[4] );
		
	
	a_89ab = _mm256_blend_pd( zeros, a_89ab, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	b_0    = _mm256_broadcast_sd( &B[5] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
	b_0    = _mm256_broadcast_sd( &B[6] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[8] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[8] );
	d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
	a_89ab = _mm256_load_pd( &A2[8] );
		
	
	a_89ab = _mm256_blend_pd( zeros, a_89ab, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	b_0    = _mm256_broadcast_sd( &B[9] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
	b_0    = _mm256_broadcast_sd( &B[10] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[12] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[12] );
	d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
	a_89ab = _mm256_load_pd( &A2[12] );
		
	
	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	b_0    = _mm256_broadcast_sd( &B[13] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
	b_0    = _mm256_broadcast_sd( &B[14] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[16] );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_4567 = _mm256_load_pd( &A1[16] );
	d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
	a_89ab = _mm256_load_pd( &A2[16] );
		
	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	k = 12;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		b_0    = _mm256_broadcast_sd( &B[0] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		b_0    = _mm256_broadcast_sd( &B[1] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
		b_0    = _mm256_broadcast_sd( &B[2] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		a_0123 = _mm256_load_pd( &A0[4] );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_4567 = _mm256_load_pd( &A1[4] );
		d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
		a_89ab = _mm256_load_pd( &A2[4] );
			
		
		b_0    = _mm256_broadcast_sd( &B[4] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		b_0    = _mm256_broadcast_sd( &B[5] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
		b_0    = _mm256_broadcast_sd( &B[6] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		a_0123 = _mm256_load_pd( &A0[8] );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_4567 = _mm256_load_pd( &A1[8] );
		d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
		a_89ab = _mm256_load_pd( &A2[8] );
			
		
		b_0    = _mm256_broadcast_sd( &B[8] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		b_0    = _mm256_broadcast_sd( &B[9] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
		b_0    = _mm256_broadcast_sd( &B[10] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		a_0123 = _mm256_load_pd( &A0[12] );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_4567 = _mm256_load_pd( &A1[12] );
		d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
		a_89ab = _mm256_load_pd( &A2[12] );
			
		
		b_0    = _mm256_broadcast_sd( &B[12] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		b_0    = _mm256_broadcast_sd( &B[13] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		d_9    = _mm256_fnmadd_pd( a_89ab, b_0, d_9 );
		b_0    = _mm256_broadcast_sd( &B[14] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		a_0123 = _mm256_load_pd( &A0[16] );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_4567 = _mm256_load_pd( &A1[16] );
		d_a    = _mm256_fnmadd_pd( a_89ab, b_0, d_a );
		a_89ab = _mm256_load_pd( &A2[16] );
				

		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
		}
	
	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_4 = _mm256_mul_pd( d_4, a_00 );
	d_8 = _mm256_mul_pd( d_8, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );
	_mm256_store_pd( &C2[0+bs*0], d_8 );

	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_9 = _mm256_fnmadd_pd( d_8, a_10, d_9 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_9 = _mm256_mul_pd( d_9, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_1 );
	_mm256_store_pd( &C1[0+bs*1], d_5 );
	_mm256_store_pd( &C2[0+bs*1], d_9 );

	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
	d_a = _mm256_fnmadd_pd( d_8, a_20, d_a );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
	d_a = _mm256_fnmadd_pd( d_9, a_21, d_a );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	d_6 = _mm256_mul_pd( d_6, a_22 );
	d_a = _mm256_mul_pd( d_a, a_22 );
	_mm256_store_pd( &C0[0+bs*2], d_2 );
	_mm256_store_pd( &C1[0+bs*2], d_6 );
	_mm256_store_pd( &C2[0+bs*2], d_a );

	}


void kernel_dtrtri_12x2_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, a_89ab,
		b_0101, b_1010,
		c_00_11_20_31, c_01_10_21_30,
		c_40_51_60_71, c_41_50_61_70,
		c_80_91_a0_b1, c_81_90_a1_b0;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	c_40_51_60_71 = _mm256_setzero_pd();
	c_41_50_61_70 = _mm256_setzero_pd();
	c_80_91_a0_b1 = _mm256_setzero_pd();
	c_81_90_a1_b0 = _mm256_setzero_pd();

	k = 0;

	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	
	
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010 , c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch


	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch


	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	
	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x1 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	
	
	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x3 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	

	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x7 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	

	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	a_89ab        = _mm256_load_pd( &A2[16] ); // prefetch
		
	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_89ab        = _mm256_blend_pd( zeros, a_89ab, 0x1 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
	a_89ab        = _mm256_load_pd( &A2[4] ); // prefetch
	
	
	a_89ab        = _mm256_blend_pd( zeros, a_89ab, 0x3 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
	a_89ab        = _mm256_load_pd( &A2[8] ); // prefetch
	

	a_89ab        = _mm256_blend_pd( zeros, a_89ab, 0x7 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
	a_89ab        = _mm256_load_pd( &A2[12] ); // prefetch
	

	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
	a_89ab        = _mm256_load_pd( &A2[16] ); // prefetch
		
	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;

	k = 12;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
		a_89ab        = _mm256_load_pd( &A2[4] ); // prefetch
		
		
		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
		a_89ab        = _mm256_load_pd( &A2[8] ); // prefetch
		

		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
		a_89ab        = _mm256_load_pd( &A2[12] ); // prefetch
		

		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		c_80_91_a0_b1 = _mm256_fnmadd_pd( a_89ab, b_0101, c_80_91_a0_b1 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_81_90_a1_b0 = _mm256_fnmadd_pd( a_89ab, b_1010, c_81_90_a1_b0 );
		a_89ab        = _mm256_load_pd( &A2[16] ); // prefetch

		
		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}

	__m256d
		d_0, d_1, d_4, d_5, d_8, d_9;

	d_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
	d_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
	d_4 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0xa );
	d_5 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0x5 );
	d_8 = _mm256_blend_pd( c_80_91_a0_b1, c_81_90_a1_b0, 0xa );
	d_9 = _mm256_blend_pd( c_80_91_a0_b1, c_81_90_a1_b0, 0x5 );

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		}
	
	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_4 = _mm256_mul_pd( d_4, a_00 );
	d_8 = _mm256_mul_pd( d_8, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );
	_mm256_store_pd( &C2[0+bs*0], d_8 );

	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_9 = _mm256_fnmadd_pd( d_8, a_10, d_9 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	d_9 = _mm256_mul_pd( d_9, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_1 );
	_mm256_store_pd( &C1[0+bs*1], d_5 );
	_mm256_store_pd( &C2[0+bs*1], d_9 );

	}



void kernel_dtrtri_12x1_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *A2 = A1 + 4*sda;
	double *C1 = C0 + 4*sdc;
	double *C2 = C1 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, a_89ab,
		b_0,
		d_0,
		d_4,
		d_8;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	d_0 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_8 = _mm256_setzero_pd();

	k = 0;

	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[4] ); // prefetch


	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[8] ); // prefetch


	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[12] ); // prefetch


	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[16] ); // prefetch

	
	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[4] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[4] ); // prefetch


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[8] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[8] ); // prefetch


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[12] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[12] ); // prefetch


	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[16] ); // prefetch
	a_89ab = _mm256_load_pd( &A2[16] ); // prefetch


	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	a_89ab = _mm256_blend_pd( zeros, a_89ab, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[4] ); // prefetch
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_4567 = _mm256_load_pd( &A1[4] ); // prefetch
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	a_89ab = _mm256_load_pd( &A2[4] ); // prefetch


	a_89ab = _mm256_blend_pd( zeros, a_89ab, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[8] ); // prefetch
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_4567 = _mm256_load_pd( &A1[8] ); // prefetch
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	a_89ab = _mm256_load_pd( &A2[8] ); // prefetch


	a_89ab = _mm256_blend_pd( zeros, a_89ab, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[12] ); // prefetch
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_4567 = _mm256_load_pd( &A1[12] ); // prefetch
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	a_89ab = _mm256_load_pd( &A2[12] ); // prefetch


	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_4567 = _mm256_load_pd( &A1[16] ); // prefetch
	d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
	a_89ab = _mm256_load_pd( &A2[16] ); // prefetch


	A0 += 16;
	A1 += 16;
	A2 += 16;
	B  += 16;


	k = 12;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		b_0    = _mm256_broadcast_sd( &B[0] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		a_0123 = _mm256_load_pd( &A0[4] ); // prefetch
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_4567 = _mm256_load_pd( &A1[4] ); // prefetch
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		a_89ab = _mm256_load_pd( &A2[4] ); // prefetch


		b_0    = _mm256_broadcast_sd( &B[4] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		a_0123 = _mm256_load_pd( &A0[8] ); // prefetch
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_4567 = _mm256_load_pd( &A1[8] ); // prefetch
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		a_89ab = _mm256_load_pd( &A2[8] ); // prefetch


		b_0    = _mm256_broadcast_sd( &B[8] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		a_0123 = _mm256_load_pd( &A0[12] ); // prefetch
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_4567 = _mm256_load_pd( &A1[12] ); // prefetch
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		a_89ab = _mm256_load_pd( &A2[12] ); // prefetch


		b_0    = _mm256_broadcast_sd( &B[12] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_4567 = _mm256_load_pd( &A1[16] ); // prefetch
		d_8    = _mm256_fnmadd_pd( a_89ab, b_0, d_8 );
		a_89ab = _mm256_load_pd( &A2[16] ); // prefetch


		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;

		}

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		}
	
	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_4 = _mm256_mul_pd( d_4, a_00 );
	d_8 = _mm256_mul_pd( d_8, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );
	_mm256_store_pd( &C2[0+bs*0], d_8 );

	}



// XXX keep for skylake
#if 0 
void kernel_dtrtri_8x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
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

	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	
	
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch


	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch


	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	
	A0 += 16;
	A1 += 16;
	B  += 16;


	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x1 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	
	
	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x3 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	
	
	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x7 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	
	
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
	c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	
	
	A0 += 16;
	A1 += 16;
	B  += 16;

	k = 8;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch


		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch


		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		c_00_11_22_33 = _mm256_fnmadd_pd( a_0123, b_0123, c_00_11_22_33 );
		c_40_51_62_73 = _mm256_fnmadd_pd( a_4567, b_0123, c_40_51_62_73 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_fnmadd_pd( a_0123, b_1032, c_01_10_23_32 );
		c_41_50_63_72 = _mm256_fnmadd_pd( a_4567, b_1032, c_41_50_63_72 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_fnmadd_pd( a_0123, b_3210, c_03_12_21_30 );
		c_43_52_61_70 = _mm256_fnmadd_pd( a_4567, b_3210, c_43_52_61_70 );
		c_02_13_20_31 = _mm256_fnmadd_pd( a_0123, b_2301, c_02_13_20_31 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_42_53_60_71 = _mm256_fnmadd_pd( a_4567, b_2301, c_42_53_60_71 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
			
		A0 += 16;
		A1 += 16;
		B  += 16;

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
	
	d_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	d_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	d_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	d_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );
	d_40_50_60_70 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0xc );
	d_42_52_62_72 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0x3 );
	d_41_51_61_71 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0xc );
	d_43_53_63_73 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0x3 );

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &E[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
		a_33 = _mm256_div_pd( ones, a_33 ); // TODO sd + broadcast
		}
	
	d_00_10_20_30 = _mm256_mul_pd( d_00_10_20_30, a_00 );
	d_40_50_60_70 = _mm256_mul_pd( d_40_50_60_70, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_00_10_20_30 );
	_mm256_store_pd( &C1[0+bs*0], d_40_50_60_70 );

	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_01_11_21_31 = _mm256_fnmadd_pd( d_00_10_20_30, a_10, d_01_11_21_31 );
	d_41_51_61_71 = _mm256_fnmadd_pd( d_40_50_60_70, a_10, d_41_51_61_71 );
	d_01_11_21_31 = _mm256_mul_pd( d_01_11_21_31, a_11 );
	d_41_51_61_71 = _mm256_mul_pd( d_41_51_61_71, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_01_11_21_31 );
	_mm256_store_pd( &C1[0+bs*1], d_41_51_61_71 );

	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
	d_02_12_22_32 = _mm256_fnmadd_pd( d_00_10_20_30, a_20, d_02_12_22_32 );
	d_42_52_62_72 = _mm256_fnmadd_pd( d_40_50_60_70, a_20, d_42_52_62_72 );
	d_02_12_22_32 = _mm256_fnmadd_pd( d_01_11_21_31, a_21, d_02_12_22_32 );
	d_42_52_62_72 = _mm256_fnmadd_pd( d_41_51_61_71, a_21, d_42_52_62_72 );
	d_02_12_22_32 = _mm256_mul_pd( d_02_12_22_32, a_22 );
	d_42_52_62_72 = _mm256_mul_pd( d_42_52_62_72, a_22 );
	_mm256_store_pd( &C0[0+bs*2], d_02_12_22_32 );
	_mm256_store_pd( &C1[0+bs*2], d_42_52_62_72 );

	a_30 = _mm256_broadcast_sd( &E[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &E[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &E[3+bs*2] );
	d_03_13_23_33 = _mm256_fnmadd_pd( d_00_10_20_30, a_30, d_03_13_23_33 );
	d_43_53_63_73 = _mm256_fnmadd_pd( d_40_50_60_70, a_30, d_43_53_63_73 );
	d_03_13_23_33 = _mm256_fnmadd_pd( d_01_11_21_31, a_31, d_03_13_23_33 );
	d_43_53_63_73 = _mm256_fnmadd_pd( d_41_51_61_71, a_31, d_43_53_63_73 );
	d_03_13_23_33 = _mm256_fnmadd_pd( d_02_12_22_32, a_32, d_03_13_23_33 );
	d_43_53_63_73 = _mm256_fnmadd_pd( d_42_52_62_72, a_32, d_43_53_63_73 );
	d_03_13_23_33 = _mm256_mul_pd( d_03_13_23_33, a_33 );
	d_43_53_63_73 = _mm256_mul_pd( d_43_53_63_73, a_33 );
	_mm256_store_pd( &C0[0+bs*3], d_03_13_23_33 );
	_mm256_store_pd( &C1[0+bs*3], d_43_53_63_73 );


	}



void kernel_dtrtri_8x3_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, //A_0123,
		b_0,
		d_0, d_1, d_2,
		d_4, d_5, d_6;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	d_0 = _mm256_setzero_pd();
	d_1 = _mm256_setzero_pd();
	d_2 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();
	d_5 = _mm256_setzero_pd();
	d_6 = _mm256_setzero_pd();

	k = 0;

	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[1] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[2] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[4] );
	
	
	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[5] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[6] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[8] );


	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[9] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[10] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[12] );


	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	b_0    = _mm256_broadcast_sd( &B[13] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	b_0    = _mm256_broadcast_sd( &B[14] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	a_0123 = _mm256_load_pd( &A0[16] );
	a_4567 = _mm256_load_pd( &A1[16] );

	
	A0 += 16;
	A1 += 16;
	B  += 16;


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[1] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[2] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_0123 = _mm256_load_pd( &A0[4] );
	a_4567 = _mm256_load_pd( &A1[4] );
		
	
	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[5] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[6] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_0123 = _mm256_load_pd( &A0[8] );
	a_4567 = _mm256_load_pd( &A1[8] );
		
	
	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[9] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[10] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_0123 = _mm256_load_pd( &A0[12] );
	a_4567 = _mm256_load_pd( &A1[12] );
		
	
	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	b_0    = _mm256_broadcast_sd( &B[13] );
	d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
	d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
	b_0    = _mm256_broadcast_sd( &B[14] );
	d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
	d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
	a_0123 = _mm256_load_pd( &A0[16] );
	a_4567 = _mm256_load_pd( &A1[16] );
		
	
	A0 += 16;
	A1 += 16;
	B  += 16;

	k = 8;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		b_0    = _mm256_broadcast_sd( &B[0] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		b_0    = _mm256_broadcast_sd( &B[1] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		b_0    = _mm256_broadcast_sd( &B[2] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_0123 = _mm256_load_pd( &A0[4] );
		a_4567 = _mm256_load_pd( &A1[4] );
			
		
		b_0    = _mm256_broadcast_sd( &B[4] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		b_0    = _mm256_broadcast_sd( &B[5] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		b_0    = _mm256_broadcast_sd( &B[6] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_0123 = _mm256_load_pd( &A0[8] );
		a_4567 = _mm256_load_pd( &A1[8] );
			
		
		b_0    = _mm256_broadcast_sd( &B[8] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		b_0    = _mm256_broadcast_sd( &B[9] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		b_0    = _mm256_broadcast_sd( &B[10] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_0123 = _mm256_load_pd( &A0[12] );
		a_4567 = _mm256_load_pd( &A1[12] );
			
		
		b_0    = _mm256_broadcast_sd( &B[12] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		b_0    = _mm256_broadcast_sd( &B[13] );
		d_1    = _mm256_fnmadd_pd( a_0123, b_0, d_1 );
		d_5    = _mm256_fnmadd_pd( a_4567, b_0, d_5 );
		b_0    = _mm256_broadcast_sd( &B[14] );
		d_2    = _mm256_fnmadd_pd( a_0123, b_0, d_2 );
		d_6    = _mm256_fnmadd_pd( a_4567, b_0, d_6 );
		a_0123 = _mm256_load_pd( &A0[16] );
		a_4567 = _mm256_load_pd( &A1[16] );
				

		A0 += 16;
		A1 += 16;
		B  += 16;

		}

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		a_22 = _mm256_div_pd( ones, a_22 ); // TODO sd + broadcast
		}
	
	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_4 = _mm256_mul_pd( d_4, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );

	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_1 );
	_mm256_store_pd( &C1[0+bs*1], d_5 );

	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
	d_2 = _mm256_fnmadd_pd( d_0, a_20, d_2 );
	d_6 = _mm256_fnmadd_pd( d_4, a_20, d_6 );
	d_2 = _mm256_fnmadd_pd( d_1, a_21, d_2 );
	d_6 = _mm256_fnmadd_pd( d_5, a_21, d_6 );
	d_2 = _mm256_mul_pd( d_2, a_22 );
	d_6 = _mm256_mul_pd( d_6, a_22 );
	_mm256_store_pd( &C0[0+bs*2], d_2 );
	_mm256_store_pd( &C1[0+bs*2], d_6 );

	}



void kernel_dtrtri_8x2_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, //A_0123,
		b_0101, b_1010,
		c_00_11_20_31, c_01_10_21_30,
		c_40_51_60_71, c_41_50_61_70;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );
	b_0101 = _mm256_broadcast_pd( (__m128d *) &B[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	c_00_11_20_31 = _mm256_setzero_pd();
	c_01_10_21_30 = _mm256_setzero_pd();
	c_40_51_60_71 = _mm256_setzero_pd();
	c_41_50_61_70 = _mm256_setzero_pd();

	k = 0;

	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	
	
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010 , c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch


	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch


	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	
	
	A0 += 16;
	A1 += 16;
	B  += 16;


	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x1 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	
	
	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x3 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	

	a_4567        = _mm256_blend_pd( zeros, a_4567, 0x7 );
	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	

	c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
	b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
	c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
	b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
	c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		
	
	A0 += 16;
	A1 += 16;
	B  += 16;

	k = 8;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[4] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		
		
		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[8] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		

		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[12] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		

		c_00_11_20_31 = _mm256_fnmadd_pd( a_0123, b_0101, c_00_11_20_31 );
		b_1010        = _mm256_shuffle_pd( b_0101, b_0101, 0x5 );
		c_40_51_60_71 = _mm256_fnmadd_pd( a_4567, b_0101, c_40_51_60_71 );
		b_0101        = _mm256_broadcast_pd( (__m128d *) &B[16] ); // prefetch
		c_01_10_21_30 = _mm256_fnmadd_pd( a_0123, b_1010, c_01_10_21_30 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		c_41_50_61_70 = _mm256_fnmadd_pd( a_4567, b_1010, c_41_50_61_70 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch

		
		A0 += 16;
		A1 += 16;
		B  += 16;

		}

	__m256d
		d_0, d_1, d_4, d_5;

	d_0 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0xa );
	d_1 = _mm256_blend_pd( c_00_11_20_31, c_01_10_21_30, 0x5 );
	d_4 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0xa );
	d_5 = _mm256_blend_pd( c_40_51_60_71, c_41_50_61_70, 0x5 );

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		a_11 = _mm256_div_pd( ones, a_11 ); // TODO sd + broadcast
		}
	
	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_4 = _mm256_mul_pd( d_4, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );

	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
	d_1 = _mm256_fnmadd_pd( d_0, a_10, d_1 );
	d_5 = _mm256_fnmadd_pd( d_4, a_10, d_5 );
	d_1 = _mm256_mul_pd( d_1, a_11 );
	d_5 = _mm256_mul_pd( d_5, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_1 );
	_mm256_store_pd( &C1[0+bs*1], d_5 );

	}



void kernel_dtrtri_8x1_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
	const int bs = 4;
	
	int k;
	
	__m256d
		zeros,
		a_0123, a_4567, //A_0123,
		b_0,
		d_0,
		d_4;
	
	// prefetch
	a_0123 = _mm256_load_pd( &A0[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	d_0 = _mm256_setzero_pd();
	d_4 = _mm256_setzero_pd();

	k = 0;

	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[4] ); // prefetch


	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[8] ); // prefetch


	a_0123 = _mm256_blend_pd( zeros, a_0123, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[12] ); // prefetch


	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[16] ); // prefetch

	
	A0 += 16;
	A1 += 16;
	B  += 16;


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x1 );
	b_0    = _mm256_broadcast_sd( &B[0] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[4] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[4] ); // prefetch


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x3 );
	b_0    = _mm256_broadcast_sd( &B[4] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[8] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[8] ); // prefetch


	a_4567 = _mm256_blend_pd( zeros, a_4567, 0x7 );
	b_0    = _mm256_broadcast_sd( &B[8] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[12] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[12] ); // prefetch


	b_0    = _mm256_broadcast_sd( &B[12] );
	d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
	d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
	a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567 = _mm256_load_pd( &A1[16] ); // prefetch


	A0 += 16;
	A1 += 16;
	B  += 16;

	k = 8;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
		b_0    = _mm256_broadcast_sd( &B[0] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_0123 = _mm256_load_pd( &A0[4] ); // prefetch
		a_4567 = _mm256_load_pd( &A1[4] ); // prefetch


		b_0    = _mm256_broadcast_sd( &B[4] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_0123 = _mm256_load_pd( &A0[8] ); // prefetch
		a_4567 = _mm256_load_pd( &A1[8] ); // prefetch


		b_0    = _mm256_broadcast_sd( &B[8] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_0123 = _mm256_load_pd( &A0[12] ); // prefetch
		a_4567 = _mm256_load_pd( &A1[12] ); // prefetch


		b_0    = _mm256_broadcast_sd( &B[12] );
		d_0    = _mm256_fnmadd_pd( a_0123, b_0, d_0 );
		d_4    = _mm256_fnmadd_pd( a_4567, b_0, d_4 );
		a_0123 = _mm256_load_pd( &A0[16] ); // prefetch
		a_4567 = _mm256_load_pd( &A1[16] ); // prefetch


		A0 += 16;
		A1 += 16;
		B  += 16;

		}

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	
	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_00 = _mm256_div_pd( ones, a_00 ); // TODO sd + broadcast
		}
	
	d_0 = _mm256_mul_pd( d_0, a_00 );
	d_4 = _mm256_mul_pd( d_4, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_0 );
	_mm256_store_pd( &C1[0+bs*0], d_4 );

	}
#endif



// XXX keep for reference
#if 0 
void kernel_dtrtri_4x4_lib4(int kmax, double *A, double *B, double *C, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

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

	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
	
	
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[8] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[12] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[16] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
	
	A += 16;
	B += 16;
	k = 4;

	for(; k<kmax-3; k+=4)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
		
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
		
		A += 16;
		B += 16;

		}
	
	__m256d
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33;

	c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
	
	d_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	d_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	d_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	d_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );

	__m256d
		ones,
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	if(use_inv_diag_E)
		{
		a_00 = _mm256_broadcast_sd( &inv_diag_E[0] );
		a_11 = _mm256_broadcast_sd( &inv_diag_E[1] );
		a_22 = _mm256_broadcast_sd( &inv_diag_E[2] );
		a_33 = _mm256_broadcast_sd( &inv_diag_E[3] );
		}
	else
		{
		a_00 = _mm256_broadcast_sd( &E[0+bs*0] );
		a_11 = _mm256_broadcast_sd( &E[1+bs*1] );
		a_22 = _mm256_broadcast_sd( &E[2+bs*2] );
		a_33 = _mm256_broadcast_sd( &E[3+bs*3] );
		a_00 = _mm256_div_pd( ones, a_00 );
		a_11 = _mm256_div_pd( ones, a_11 );
		a_22 = _mm256_div_pd( ones, a_22 );
		a_33 = _mm256_div_pd( ones, a_33 );
		}
	
//	a_00 = _mm256_broadcast_sd( &fact[0] );
	d_00_10_20_30 = _mm256_mul_pd( d_00_10_20_30, a_00 );
	_mm256_store_pd( &C[0+bs*0], d_00_10_20_30 );

	a_10 = _mm256_broadcast_sd( &E[1+bs*0] );
//	a_11 = _mm256_broadcast_sd( &fact[2] );
	ab_temp = _mm256_mul_pd( d_00_10_20_30, a_10 );
	d_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, ab_temp );
	d_01_11_21_31 = _mm256_mul_pd( d_01_11_21_31, a_11 );
	_mm256_store_pd( &C[0+bs*1], d_01_11_21_31 );

	a_20 = _mm256_broadcast_sd( &E[2+bs*0] );
	a_21 = _mm256_broadcast_sd( &E[2+bs*1] );
//	a_22 = _mm256_broadcast_sd( &fact[5] );
	ab_temp = _mm256_mul_pd( d_00_10_20_30, a_20 );
	d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, ab_temp );
	ab_temp = _mm256_mul_pd( d_01_11_21_31, a_21 );
	d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, ab_temp );
	d_02_12_22_32 = _mm256_mul_pd( d_02_12_22_32, a_22 );
	_mm256_store_pd( &C[0+bs*2], d_02_12_22_32 );

	a_30 = _mm256_broadcast_sd( &E[3+bs*0] );
	a_31 = _mm256_broadcast_sd( &E[3+bs*1] );
	a_32 = _mm256_broadcast_sd( &E[3+bs*2] );
//	a_33 = _mm256_broadcast_sd( &fact[9] );
	ab_temp = _mm256_mul_pd( d_00_10_20_30, a_30 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_temp );
	ab_temp = _mm256_mul_pd( d_01_11_21_31, a_31 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_temp );
	ab_temp = _mm256_mul_pd( d_02_12_22_32, a_32 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_temp );
	d_03_13_23_33 = _mm256_mul_pd( d_03_13_23_33, a_33 );
	_mm256_store_pd( &C[0+bs*3], d_03_13_23_33 );

	}
	
	
	
void kernel_dtrtri_4x2_lib4(int kmax, double *A, double *B, double *C, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
		
	c_00 -= a_0 * b_0;

	c_01 -= a_0 * b_1;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;
	c_30 -= a_3 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;
	c_31 -= a_3 * b_1;


	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}

	// dtrsm
	double
		a_00, a_10, a_11;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		a_11 = inv_diag_E[1];
		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		a_11 = 1.0/E[1+bs*1];
		}

//	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	a_10 = E[1+bs*0];
//	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_21 -= c_20*a_10;
	c_31 -= c_30*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	c_21 *= a_11;
	c_31 *= a_11;
	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	}
#endif
	
	
	
// old kernels



void corner_dtrinv_4x4_lib4(double *fact, double *C)
	{

	const int bs = 4;

	double
		c_00=0.0,
		c_10=0.0, c_11=0.0,
		c_20=0.0, c_21=0.0, c_22=0.0,
		c_30=0.0, c_31=0.0, c_32=0.0, c_33=0.0,
		d_20=0.0, d_21=0.0,
		d_30=0.0, d_31=0.0;

//	c_00 = 1.0/A[0+bs*0];
//	c_11 = 1.0/A[1+bs*1];
//	c_22 = 1.0/A[2+bs*2];
//	c_33 = 1.0/A[3+bs*3];
	c_00 = fact[0];
	c_11 = fact[2];
	c_22 = fact[5];
	c_33 = fact[9];

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;
	C[2+bs*2] = c_22;
	C[3+bs*3] = c_33;

//	c_10 = A[1+bs*0];
//	c_32 = A[3+bs*2];
	c_10 = fact[1];
	c_32 = fact[8];

	c_10 = - c_11*c_10*c_00;
	c_32 = - c_33*c_32*c_22;

	C[0+bs*1] = c_10;
	C[2+bs*3] = c_32;

//	c_20 = A[2+bs*1]*c_10;
//	c_30 = A[3+bs*1]*c_10;
//	c_21 = A[2+bs*1]*c_11;
//	c_31 = A[3+bs*1]*c_11;
//	c_20 = fact[3];
//	c_30 = fact[6];
	c_21 = fact[4];
	c_31 = fact[7];

	c_20 = c_21*c_10;
	c_30 = c_31*c_10;
	c_21 = c_21*c_11;
	c_31 = c_31*c_11;

	c_20 += fact[3]*c_00;
	c_30 += fact[6]*c_00;
//	c_21 += A[2+bs*0]*c_01;
//	c_31 += A[3+bs*0]*c_01;

	d_20 = c_22*c_20;
	d_30 = c_32*c_20;
	d_21 = c_22*c_21;
	d_31 = c_32*c_21;

//	d_20 += c_23*c_30;
	d_30 += c_33*c_30;
//	d_21 += c_23*c_31;
	d_31 += c_33*c_31;

	C[0+bs*2] = - d_20;
	C[0+bs*3] = - d_30;
	C[1+bs*2] = - d_21;
	C[1+bs*3] = - d_31;

	return;

	}



void corner_dtrinv_2x2_lib4(double *fact, double *C)
	{

	const int bs = 4;

	double
		c_00=0.0,
		c_10=0.0, c_11=0.0;

	c_00 = fact[0];
	c_11 = fact[2];

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;

	c_10 = fact[1];

	c_10 = - c_11*c_10*c_00;

	C[0+bs*1] = c_10;

	return;

	}



void kernel_dtrinv_8x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *fact)
	{

	double *A1 = A0 + 4*sda;
	double *C1 = C0 + 4*sdc;
	
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

	ab_tmp1       = _mm256_setzero_pd();

	a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x1 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
	
	
	a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x3 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );


	a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x7 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );


	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );

	
	A0 += 16;
	A1 += 16;
	B  += 16;


	a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x1 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
	c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
	c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_2301 );
	a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
	c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp0 );
	
	
	a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x3 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
	c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
	c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_2301 );
	a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
	c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp0 );
	
	
	a_4567        = _mm256_blend_pd( ab_tmp1, a_4567, 0x7 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
	c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
	c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_2301 );
	a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
	c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp0 );
	
	
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_0123 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_1032 );
	c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_3210 );
	c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_4567, b_2301 );
	a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
	c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp0 );
	
	
	A0 += 16;
	A1 += 16;
	B  += 16;

	k = 8;

	for(; k<kmax-3; k+=4) // correction in cholesky is multiple of block size 4
		{
		
/*	__builtin_prefetch( A+32 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[4] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[4] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp1 );
		
		
/*	__builtin_prefetch( A+40 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[8] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[8] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp1 );


/*	__builtin_prefetch( A+48 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[12] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[12] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp1 );


/*	__builtin_prefetch( A+56 );*/
		ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_0123 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_tmp0 );
		c_40_51_62_73 = _mm256_sub_pd( c_40_51_62_73, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_1032 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_tmp0 );
		c_41_50_63_72 = _mm256_sub_pd( c_41_50_63_72, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		ab_tmp1       = _mm256_mul_pd( a_4567, b_3210 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_tmp0 );
		c_43_52_61_70 = _mm256_sub_pd( c_43_52_61_70, ab_tmp1 );
		ab_tmp0       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A0[16] ); // prefetch
		ab_tmp1       = _mm256_mul_pd( a_4567, b_2301 );
		a_4567        = _mm256_load_pd( &A1[16] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_tmp0 );
		c_42_53_60_71 = _mm256_sub_pd( c_42_53_60_71, ab_tmp1 );
		
		A0 += 16;
		A1 += 16;
		B  += 16;

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
	
	d_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	d_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	d_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	d_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );
	d_40_50_60_70 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0xc );
	d_42_52_62_72 = _mm256_blend_pd( c_40_50_62_72, c_42_52_60_70, 0x3 );
	d_41_51_61_71 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0xc );
	d_43_53_63_73 = _mm256_blend_pd( c_41_51_63_73, c_43_53_61_71, 0x3 );

	__m256d
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	a_00 = _mm256_broadcast_sd( &fact[0] );
	d_00_10_20_30 = _mm256_mul_pd( d_00_10_20_30, a_00 );
	d_40_50_60_70 = _mm256_mul_pd( d_40_50_60_70, a_00 );
	_mm256_store_pd( &C0[0+bs*0], d_00_10_20_30 );
	_mm256_store_pd( &C1[0+bs*0], d_40_50_60_70 );

	a_10 = _mm256_broadcast_sd( &fact[1] );
	a_11 = _mm256_broadcast_sd( &fact[2] );
	ab_tmp0 = _mm256_mul_pd( d_00_10_20_30, a_10 );
	ab_tmp1 = _mm256_mul_pd( d_40_50_60_70, a_10 );
	d_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, ab_tmp0 );
	d_41_51_61_71 = _mm256_sub_pd( d_41_51_61_71, ab_tmp1 );
	d_01_11_21_31 = _mm256_mul_pd( d_01_11_21_31, a_11 );
	d_41_51_61_71 = _mm256_mul_pd( d_41_51_61_71, a_11 );
	_mm256_store_pd( &C0[0+bs*1], d_01_11_21_31 );
	_mm256_store_pd( &C1[0+bs*1], d_41_51_61_71 );

	a_20 = _mm256_broadcast_sd( &fact[3] );
	a_21 = _mm256_broadcast_sd( &fact[4] );
	a_22 = _mm256_broadcast_sd( &fact[5] );
	ab_tmp0 = _mm256_mul_pd( d_00_10_20_30, a_20 );
	ab_tmp1 = _mm256_mul_pd( d_40_50_60_70, a_20 );
	d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, ab_tmp0 );
	d_42_52_62_72 = _mm256_sub_pd( d_42_52_62_72, ab_tmp1 );
	ab_tmp0 = _mm256_mul_pd( d_01_11_21_31, a_21 );
	ab_tmp1 = _mm256_mul_pd( d_41_51_61_71, a_21 );
	d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, ab_tmp0 );
	d_42_52_62_72 = _mm256_sub_pd( d_42_52_62_72, ab_tmp1 );
	d_02_12_22_32 = _mm256_mul_pd( d_02_12_22_32, a_22 );
	d_42_52_62_72 = _mm256_mul_pd( d_42_52_62_72, a_22 );
	_mm256_store_pd( &C0[0+bs*2], d_02_12_22_32 );
	_mm256_store_pd( &C1[0+bs*2], d_42_52_62_72 );

	a_30 = _mm256_broadcast_sd( &fact[6] );
	a_31 = _mm256_broadcast_sd( &fact[7] );
	a_32 = _mm256_broadcast_sd( &fact[8] );
	a_33 = _mm256_broadcast_sd( &fact[9] );
	ab_tmp0 = _mm256_mul_pd( d_00_10_20_30, a_30 );
	ab_tmp1 = _mm256_mul_pd( d_40_50_60_70, a_30 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_tmp0 );
	d_43_53_63_73 = _mm256_sub_pd( d_43_53_63_73, ab_tmp1 );
	ab_tmp0 = _mm256_mul_pd( d_01_11_21_31, a_31 );
	ab_tmp1 = _mm256_mul_pd( d_41_51_61_71, a_31 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_tmp0 );
	d_43_53_63_73 = _mm256_sub_pd( d_43_53_63_73, ab_tmp1 );
	ab_tmp0 = _mm256_mul_pd( d_02_12_22_32, a_32 );
	ab_tmp1 = _mm256_mul_pd( d_42_52_62_72, a_32 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_tmp0 );
	d_43_53_63_73 = _mm256_sub_pd( d_43_53_63_73, ab_tmp1 );
	d_03_13_23_33 = _mm256_mul_pd( d_03_13_23_33, a_33 );
	d_43_53_63_73 = _mm256_mul_pd( d_43_53_63_73, a_33 );
	_mm256_store_pd( &C0[0+bs*3], d_03_13_23_33 );
	_mm256_store_pd( &C1[0+bs*3], d_43_53_63_73 );


	}



void kernel_dtrinv_4x4_lib4(int kmax, double *A, double *B, double *C, double *fact)
	{

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

	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
	
	
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[8] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[12] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
	ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
	a_0123        = _mm256_load_pd( &A[16] ); // prefetch
	c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
	
	A += 16;
	B += 16;
	k = 4;

	for(; k<kmax-3; k+=4)
		{
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[4] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[4] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
		
		
		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[8] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[8] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[12] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[12] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );


		ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
		b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
		b_0123        = _mm256_load_pd( &B[16] ); // prefetch
		c_00_11_22_33 = _mm256_sub_pd( c_00_11_22_33, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
		b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
		c_01_10_23_32 = _mm256_sub_pd( c_01_10_23_32, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_3210 );
		b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
		c_03_12_21_30 = _mm256_sub_pd( c_03_12_21_30, ab_temp );
		ab_temp       = _mm256_mul_pd( a_0123, b_2301 );
		a_0123        = _mm256_load_pd( &A[16] ); // prefetch
		c_02_13_20_31 = _mm256_sub_pd( c_02_13_20_31, ab_temp );
		
		A += 16;
		B += 16;

		}
	
	__m256d
		c_00_10_22_32, c_01_11_23_33, c_02_12_20_30, c_03_13_21_31,
		c_00_10_20_30, c_01_11_21_31, c_02_12_22_32, c_03_13_23_33,
		d_00_10_20_30, d_01_11_21_31, d_02_12_22_32, d_03_13_23_33;

	c_00_10_22_32 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0xa );
	c_01_11_23_33 = _mm256_blend_pd( c_00_11_22_33, c_01_10_23_32, 0x5 );
	c_02_12_20_30 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0xa );
	c_03_13_21_31 = _mm256_blend_pd( c_02_13_20_31, c_03_12_21_30, 0x5 );
	
	d_00_10_20_30 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0xc );
	d_02_12_22_32 = _mm256_blend_pd( c_00_10_22_32, c_02_12_20_30, 0x3 );
	d_01_11_21_31 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0xc );
	d_03_13_23_33 = _mm256_blend_pd( c_01_11_23_33, c_03_13_21_31, 0x3 );

	__m256d
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	a_00 = _mm256_broadcast_sd( &fact[0] );
	d_00_10_20_30 = _mm256_mul_pd( d_00_10_20_30, a_00 );
	_mm256_store_pd( &C[0+bs*0], d_00_10_20_30 );

	a_10 = _mm256_broadcast_sd( &fact[1] );
	a_11 = _mm256_broadcast_sd( &fact[2] );
	ab_temp = _mm256_mul_pd( d_00_10_20_30, a_10 );
	d_01_11_21_31 = _mm256_sub_pd( d_01_11_21_31, ab_temp );
	d_01_11_21_31 = _mm256_mul_pd( d_01_11_21_31, a_11 );
	_mm256_store_pd( &C[0+bs*1], d_01_11_21_31 );

	a_20 = _mm256_broadcast_sd( &fact[3] );
	a_21 = _mm256_broadcast_sd( &fact[4] );
	a_22 = _mm256_broadcast_sd( &fact[5] );
	ab_temp = _mm256_mul_pd( d_00_10_20_30, a_20 );
	d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, ab_temp );
	ab_temp = _mm256_mul_pd( d_01_11_21_31, a_21 );
	d_02_12_22_32 = _mm256_sub_pd( d_02_12_22_32, ab_temp );
	d_02_12_22_32 = _mm256_mul_pd( d_02_12_22_32, a_22 );
	_mm256_store_pd( &C[0+bs*2], d_02_12_22_32 );

	a_30 = _mm256_broadcast_sd( &fact[6] );
	a_31 = _mm256_broadcast_sd( &fact[7] );
	a_32 = _mm256_broadcast_sd( &fact[8] );
	a_33 = _mm256_broadcast_sd( &fact[9] );
	ab_temp = _mm256_mul_pd( d_00_10_20_30, a_30 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_temp );
	ab_temp = _mm256_mul_pd( d_01_11_21_31, a_31 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_temp );
	ab_temp = _mm256_mul_pd( d_02_12_22_32, a_32 );
	d_03_13_23_33 = _mm256_sub_pd( d_03_13_23_33, ab_temp );
	d_03_13_23_33 = _mm256_mul_pd( d_03_13_23_33, a_33 );
	_mm256_store_pd( &C[0+bs*3], d_03_13_23_33 );

	}
	
	
	
void kernel_dtrinv_4x2_lib4(int kmax, double *A, double *B, double *C, double *fact)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
		
	c_00 -= a_0 * b_0;

	c_01 -= a_0 * b_1;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;
	c_30 -= a_3 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;
	c_31 -= a_3 * b_1;


	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}

//	c_00 += D[0+bs*0];
//	c_10 += D[1+bs*0];
//	c_20 += D[2+bs*0];
//	c_30 += D[3+bs*0];

//	c_01 += D[0+bs*1];
//	c_11 += D[1+bs*1];
//	c_21 += D[2+bs*1];
//	c_31 += D[3+bs*1];
	
	// dtrsm
	double
		a_00, a_10, a_11;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	a_10 = fact[1];
	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_21 -= c_20*a_10;
	c_31 -= c_30*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	c_21 *= a_11;
	c_31 *= a_11;
	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	}
	
	
	
#if 0
// computes the inverse of a 4x4 lower trinagular matrix, and stores it at an upper triangular matrix
void corner_dtrinv_4x4_lib4_old(double *A, double *C)
	{

	const int bs = 4;

	double
		c_00=0.0,
		c_10=0.0, c_11=0.0,
		c_20=0.0, c_21=0.0, c_22=0.0,
		c_30=0.0, c_31=0.0, c_32=0.0, c_33=0.0,
		d_20=0.0, d_21=0.0,
		d_30=0.0, d_31=0.0;

	c_00 = 1.0/A[0+bs*0];
	c_11 = 1.0/A[1+bs*1];
	c_22 = 1.0/A[2+bs*2];
	c_33 = 1.0/A[3+bs*3];

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;
	C[2+bs*2] = c_22;
	C[3+bs*3] = c_33;

	c_10 = - c_11*A[1+bs*0]*c_00;
	c_32 = - c_33*A[3+bs*2]*c_22;

	C[0+bs*1] = c_10;
	C[2+bs*3] = c_32;

	c_20 = A[2+bs*1]*c_10;
	c_30 = A[3+bs*1]*c_10;
	c_21 = A[2+bs*1]*c_11;
	c_31 = A[3+bs*1]*c_11;

	c_20 += A[2+bs*0]*c_00;
	c_30 += A[3+bs*0]*c_00;
//	c_21 += A[2+bs*0]*c_01;
//	c_31 += A[3+bs*0]*c_01;

	d_20 = c_22*c_20;
	d_30 = c_32*c_20;
	d_21 = c_22*c_21;
	d_31 = c_32*c_21;

//	d_20 += c_23*c_30;
	d_30 += c_33*c_30;
//	d_21 += c_23*c_31;
	d_31 += c_33*c_31;

	C[0+bs*2] = - d_20;
	C[0+bs*3] = - d_30;
	C[1+bs*2] = - d_21;
	C[1+bs*3] = - d_31;

	return;

	}
#endif



#if 0
// A_00_inv is the mxm top-left matrix already inverted; A_11_inv is the 4x4 bottom-right matrix already inverted, A_10 it the 4xm matrix to invert, C is the mx4 matrix to write the result
void kernel_dtrinv_4x4_lib4_old(int kmax, double *A_00_inv, double *A_10, double *A_11_inv, double *C, int sdc)
	{

	// assume kmax multile of bs !!!

	const int bs = 4;

	int k;

	double
		temp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;

	// initial triangle

	a_0 = A_10[0+bs*0];
	a_1 = A_10[1+bs*0];
	a_2 = A_10[2+bs*0];
	a_3 = A_10[3+bs*0];
	
	b_0 = A_00_inv[0+bs*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	a_0 = A_10[0+bs*1];
	a_1 = A_10[1+bs*1];
	a_2 = A_10[2+bs*1];
	a_3 = A_10[3+bs*1];
	
	b_0 = A_00_inv[0+bs*1];
	b_1 = A_00_inv[1+bs*1];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;


	a_0 = A_10[0+bs*2];
	a_1 = A_10[1+bs*2];
	a_2 = A_10[2+bs*2];
	a_3 = A_10[3+bs*2];
	
	b_0 = A_00_inv[0+bs*2];
	b_1 = A_00_inv[1+bs*2];
	b_2 = A_00_inv[2+bs*2];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;


	a_0 = A_10[0+bs*3];
	a_1 = A_10[1+bs*3];
	a_2 = A_10[2+bs*3];
	a_3 = A_10[3+bs*3];
	
	b_0 = A_00_inv[0+bs*3];
	b_1 = A_00_inv[1+bs*3];
	b_2 = A_00_inv[2+bs*3];
	b_3 = A_00_inv[3+bs*3];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;


	A_10 += 16;
	A_00_inv += 16;

	for(k=4; k<kmax-3; k+=4)
		{
		
		a_0 = A_10[0+bs*0];
		a_1 = A_10[1+bs*0];
		a_2 = A_10[2+bs*0];
		a_3 = A_10[3+bs*0];
		
		b_0 = A_00_inv[0+bs*0];
		b_1 = A_00_inv[1+bs*0];
		b_2 = A_00_inv[2+bs*0];
		b_3 = A_00_inv[3+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;


		a_0 = A_10[0+bs*1];
		a_1 = A_10[1+bs*1];
		a_2 = A_10[2+bs*1];
		a_3 = A_10[3+bs*1];
		
		b_0 = A_00_inv[0+bs*1];
		b_1 = A_00_inv[1+bs*1];
		b_2 = A_00_inv[2+bs*1];
		b_3 = A_00_inv[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;


		a_0 = A_10[0+bs*2];
		a_1 = A_10[1+bs*2];
		a_2 = A_10[2+bs*2];
		a_3 = A_10[3+bs*2];
		
		b_0 = A_00_inv[0+bs*2];
		b_1 = A_00_inv[1+bs*2];
		b_2 = A_00_inv[2+bs*2];
		b_3 = A_00_inv[3+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;


		a_0 = A_10[0+bs*3];
		a_1 = A_10[1+bs*3];
		a_2 = A_10[2+bs*3];
		a_3 = A_10[3+bs*3];
		
		b_0 = A_00_inv[0+bs*3];
		b_1 = A_00_inv[1+bs*3];
		b_2 = A_00_inv[2+bs*3];
		b_3 = A_00_inv[3+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;
		
		
		A_10 += 16;
		A_00_inv += 16;

		}



	// transpose
	temp = c_10;
	c_10 = c_01;
	c_01 = temp;

	temp = c_20;
	c_20 = c_02;
	c_02 = temp;

	temp = c_30;
	c_30 = c_03;
	c_03 = temp;

	temp = c_21;
	c_21 = c_12;
	c_12 = temp;

	temp = c_31;
	c_31 = c_13;
	c_13 = temp;

	temp = c_32;
	c_32 = c_23;
	c_23 = temp;



	// final triangle

	b_3 = A_11_inv[3+bs*3];

	c_03 = c_03 * b_3;
	c_13 = c_13 * b_3;
	c_23 = c_23 * b_3;
	c_33 = c_33 * b_3;


	b_3 = A_11_inv[2+bs*3];
	b_2 = A_11_inv[2+bs*2];

	c_03 += c_02 * b_3;
	c_13 += c_12 * b_3;
	c_23 += c_22 * b_3;
	c_33 += c_32 * b_3;

	c_02 = c_02 * b_2;
	c_12 = c_12 * b_2;
	c_22 = c_22 * b_2;
	c_32 = c_32 * b_2;


	b_3 = A_11_inv[1+bs*3];
	b_2 = A_11_inv[1+bs*2];
	b_1 = A_11_inv[1+bs*1];

	c_03 += c_01 * b_3;
	c_13 += c_11 * b_3;
	c_23 += c_21 * b_3;
	c_33 += c_31 * b_3;

	c_02 += c_01 * b_2;
	c_12 += c_11 * b_2;
	c_22 += c_21 * b_2;
	c_32 += c_31 * b_2;

	c_01 = c_01 * b_1;
	c_11 = c_11 * b_1;
	c_21 = c_21 * b_1;
	c_31 = c_31 * b_1;
	

	b_3 = A_11_inv[0+bs*3];
	b_2 = A_11_inv[0+bs*2];
	b_1 = A_11_inv[0+bs*1];
	b_0 = A_11_inv[0+bs*0];

	c_03 += c_00 * b_3;
	c_13 += c_10 * b_3;
	c_23 += c_20 * b_3;
	c_33 += c_30 * b_3;

	c_02 += c_00 * b_2;
	c_12 += c_10 * b_2;
	c_22 += c_20 * b_2;
	c_32 += c_30 * b_2;

	c_01 += c_00 * b_1;
	c_11 += c_10 * b_1;
	c_21 += c_20 * b_1;
	c_31 += c_30 * b_1;

	c_00 = c_00 * b_0;
	c_10 = c_10 * b_0;
	c_20 = c_20 * b_0;
	c_30 = c_30 * b_0;


	// change sign & store result
	C[0+bs*0] = - c_00;
	C[1+bs*0] = - c_10;
	C[2+bs*0] = - c_20;
	C[3+bs*0] = - c_30;

	C[0+bs*1] = - c_01;
	C[1+bs*1] = - c_11;
	C[2+bs*1] = - c_21;
	C[3+bs*1] = - c_31;

	C[0+bs*2] = - c_02;
	C[1+bs*2] = - c_12;
	C[2+bs*2] = - c_22;
	C[3+bs*2] = - c_32;

	C[0+bs*3] = - c_03;
	C[1+bs*3] = - c_13;
	C[2+bs*3] = - c_23;
	C[3+bs*3] = - c_33;

	}
#endif







