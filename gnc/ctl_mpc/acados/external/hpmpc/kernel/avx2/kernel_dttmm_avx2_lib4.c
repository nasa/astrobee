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




void corner_dlauum_nt_4x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;


	__m256d
		tmp, zeros,
		a_0, b_0,
		d_0, d_1, d_2, d_3;
	

	zeros = _mm256_setzero_pd();


	// k = 0
	a_0 = _mm256_load_pd( &A[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );

	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_mul_pd( a_0, b_0 );


	// k = 1
	a_0 = _mm256_load_pd( &A[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );

	b_0 = _mm256_broadcast_sd( &B[4] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_add_pd( d_0, tmp );

	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_mul_pd( a_0, b_0 );


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
	d_2 = _mm256_mul_pd( a_0, b_0 );


	// k = 3
	a_0 = _mm256_load_pd( &A[8] );

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
	d_3 = _mm256_mul_pd( a_0, b_0 );



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
	tmp = _mm256_load_pd( &D[0+bs*1] );
	d_1 = _mm256_blend_pd( d_1, zeros, 0x1 );
	_mm256_store_pd( &D[0+bs*1], d_1 );
	tmp = _mm256_load_pd( &D[0+bs*2] );
	d_2 = _mm256_blend_pd( d_2, zeros, 0x3 );
	_mm256_store_pd( &D[0+bs*2], d_2 );
	tmp = _mm256_load_pd( &D[0+bs*3] );
	d_3 = _mm256_blend_pd( d_3, zeros, 0x7 );
	_mm256_store_pd( &D[0+bs*3], d_3 );


	}



void corner_dlauum_nt_3x3_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;


	__m256d
		tmp, zeros,
		a_0, b_0,
		d_0, d_1, d_2;
	

	zeros = _mm256_setzero_pd();


	// k = 0
	a_0 = _mm256_load_pd( &A[0] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x1 );

	b_0 = _mm256_broadcast_sd( &B[0] );
	d_0 = _mm256_mul_pd( a_0, b_0 );


	// k = 1
	a_0 = _mm256_load_pd( &A[4] );
	a_0 = _mm256_blend_pd( zeros, a_0, 0x3 );

	b_0 = _mm256_broadcast_sd( &B[4] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_add_pd( d_0, tmp );

	b_0 = _mm256_broadcast_sd( &B[5] );
	d_1 = _mm256_mul_pd( a_0, b_0 );


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
	d_2 = _mm256_mul_pd( a_0, b_0 );


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
			}
		else
			{
			tmp = _mm256_load_pd( &C[0+bs*0] );
			d_0 = _mm256_sub_pd( tmp, d_0 );
			tmp = _mm256_load_pd( &C[0+bs*1] );
			d_1 = _mm256_sub_pd( tmp, d_1 );
			tmp = _mm256_load_pd( &C[0+bs*2] );
			d_2 = _mm256_sub_pd( tmp, d_2 );
			}
		}
	
	_mm256_store_pd( &D[0+bs*0], d_0 );
	tmp = _mm256_load_pd( &D[0+bs*1] );
	d_1 = _mm256_blend_pd( d_1, zeros, 0x1 );
	_mm256_store_pd( &D[0+bs*1], d_1 );
	tmp = _mm256_load_pd( &D[0+bs*2] );
	d_2 = _mm256_blend_pd( d_2, zeros, 0x3 );
	_mm256_store_pd( &D[0+bs*2], d_2 );


	}



void corner_dlauum_nt_2x2_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0, a_1,
		b_0, b_1,
		d_00, 
		d_10, d_11;
	

	// k = 0
	a_0 = A[0+bs*0];
	b_0 = B[0+bs*0];

	d_00 = a_0*b_0;

	// k = 1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	d_00 += a_0*b_0;
	d_10  = a_1*b_0;
	d_11  = a_1*b_1;


	if(alg!=0)
		{
		if(alg==1)
			{
			d_00 = C[0+bs*0] + d_00;
			d_10 = C[1+bs*0] + d_10;
			d_11 = C[1+bs*1] + d_11;
			}
		else
			{
			d_00 = C[0+bs*0] - d_00;
			d_10 = C[1+bs*0] - d_10;
			d_11 = C[1+bs*1] - d_11;
			}
		}
	
	D[0+bs*0] = d_00;
	D[1+bs*0] = d_10;
	D[1+bs*1] = d_11;

	}



void corner_dlauum_nt_1x1_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	double
		a_0,
		b_0,
		d_00;
	
	a_0 = A[0];
	b_0 = B[0];

	d_00 = a_0*b_0;

	if(alg!=0)
		{
		if(alg==1)
			{
			d_00 = C[0] + d_00;
			}
		else
			{
			d_00 = C[0] - d_00;
			}
		}
	
	D[0] = d_00;

	}



void kernel_dlauum_nt_12x4_lib4(int kmax, double *A0, int sda, double *B, int alg, double *C0, int sdc, double *D0, int sdd)
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

	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	b_0  = _mm256_load_pd( &B[4] ); // prefetch

	
	// k = 1
	a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );

	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );

	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );
	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );

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
		d_01 = _mm256_load_pd( &D0[0+bs*1] );
		d_02 = _mm256_load_pd( &D0[0+bs*2] );
		d_03 = _mm256_load_pd( &D0[0+bs*3] );

		c_01 = _mm256_blend_pd( c_01, d_01, 0x1 );
		c_02 = _mm256_blend_pd( c_02, d_02, 0x3 );
		c_03 = _mm256_blend_pd( c_03, d_03, 0x7 );

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

		c_01 = _mm256_load_pd( &D0[0+bs*1] );
		c_02 = _mm256_load_pd( &D0[0+bs*2] );
		c_03 = _mm256_load_pd( &D0[0+bs*3] );

		d_01 = _mm256_blend_pd( d_01, c_01, 0x1 );
		d_02 = _mm256_blend_pd( d_02, c_02, 0x3 );
		d_03 = _mm256_blend_pd( d_03, c_03, 0x7 );

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



void kernel_dlauum_nt_8x8_lib4(int kmax, double *A0, int sda, double *B0, int sdb, int alg, double *C0, int sdc, double *D0, int sdd)
	{

	double *A1 = A0 + 4*sda;
	double *B1 = B0 + 4*sdb;
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
		
	// prefetch
	a_0 = _mm256_load_pd( &A0[0] );
	a_4 = _mm256_load_pd( &A1[0] );
	b_0 = _mm256_load_pd( &B0[0] );
	b_4 = _mm256_load_pd( &B1[0] );

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

	// initial triangles

	// XXX blend A to keep lower-tr clean; no need to blend B since upper-tr is not stored

	// k = 0
	a_0  = _mm256_blend_pd( zeros, a_0, 0x1 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	b_0  = _mm256_load_pd( &B0[4] ); // prefetch


	// k = 1
	a_0  = _mm256_blend_pd( zeros, a_0, 0x3 );
	c_00 = _mm256_fmadd_pd( a_0, b_0, c_00 );
	b_0  = _mm256_shuffle_pd( b_0, b_0, 0x5 );

	c_01 = _mm256_fmadd_pd( a_0, b_0, c_01 );
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	b_0  = _mm256_load_pd( &B0[8] ); // prefetch
	

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
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	b_0  = _mm256_load_pd( &B0[12] ); // prefetch
	

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
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	b_0  = _mm256_load_pd( &B0[16] ); // prefetch
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
	b_4  = _mm256_load_pd( &B1[16] ); // prefetch
		

	A0 += 16;
	A1 += 16;
	B0 += 16;
	B1 += 16;
	k  += 4;


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
	a_0  = _mm256_load_pd( &A0[4] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &B0[4] ); // prefetch
//	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &A1[4] ); // prefetch
	b_4  = _mm256_load_pd( &B1[4] ); // prefetch
		

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
	a_0  = _mm256_load_pd( &A0[8] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &B0[8] ); // prefetch
//	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &A1[8] ); // prefetch
	b_4  = _mm256_load_pd( &B1[8] ); // prefetch


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
	a_0  = _mm256_load_pd( &A0[12] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &B0[12] ); // prefetch
	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &A1[12] ); // prefetch
	b_4  = _mm256_load_pd( &B1[12] ); // prefetch
	

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
	a_0  = _mm256_load_pd( &A0[16] ); // prefetch
	c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
	b_0  = _mm256_load_pd( &B0[16] ); // prefetch
	c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
	a_4  = _mm256_load_pd( &A1[16] ); // prefetch
	b_4  = _mm256_load_pd( &B1[16] ); // prefetch
				

	A0 += 16;
	A1 += 16;
	B0 += 16;
	B1 += 16;
	k  += 4;


	for(; k<kmax-3; k+=4)
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
		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &B0[4] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
		b_4  = _mm256_load_pd( &B1[4] ); // prefetch
		
		
		
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
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &B0[8] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		b_4  = _mm256_load_pd( &B1[8] ); // prefetch
	


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
		a_0  = _mm256_load_pd( &A0[12] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &B0[12] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &A1[12] ); // prefetch
		b_4  = _mm256_load_pd( &B1[12] ); // prefetch
	


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
		a_0  = _mm256_load_pd( &A0[16] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &B0[16] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &A1[16] ); // prefetch
		b_4  = _mm256_load_pd( &B1[16] ); // prefetch
			
		A0 += 16;
		A1 += 16;
		B0 += 16;
		B1 += 16;

		}
	if(kmax%4>=2)
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
		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &B0[4] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
		b_4  = _mm256_load_pd( &B1[4] ); // prefetch
		
		
		
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
		a_0  = _mm256_load_pd( &A0[8] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
		b_0  = _mm256_load_pd( &B0[8] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
		a_4  = _mm256_load_pd( &A1[8] ); // prefetch
		b_4  = _mm256_load_pd( &B1[8] ); // prefetch
				
		A0 += 8;
		A1 += 8;
		B0 += 8;
		B1 += 8;

		}
	if(kmax%2==1)
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
//		a_0  = _mm256_load_pd( &A0[4] ); // prefetch
		c_42 = _mm256_fmadd_pd( a_4, b_0, c_42 );
//		b_0  = _mm256_load_pd( &B0[4] ); // prefetch
		c_46 = _mm256_fmadd_pd( a_4, b_4, c_46 );
//		a_4  = _mm256_load_pd( &A1[4] ); // prefetch
//		b_4  = _mm256_load_pd( &B1[4] ); // prefetch
			
		}



	// store
	__m256d
		e_0, e_1, e_2, e_3;

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
		
	e_0 = _mm256_blend_pd( c_44, c_45, 0xa );
	e_1 = _mm256_blend_pd( c_44, c_45, 0x5 );
	e_2 = _mm256_blend_pd( c_46, c_47, 0xa );
	e_3 = _mm256_blend_pd( c_46, c_47, 0x5 );
	
	c_44 = _mm256_blend_pd( e_0, e_2, 0xc );
	c_46 = _mm256_blend_pd( e_0, e_2, 0x3 );
	c_45 = _mm256_blend_pd( e_1, e_3, 0xc );
	c_47 = _mm256_blend_pd( e_1, e_3, 0x3 );
		
	if(alg==0) // C = A * B'
		{
		e_1 = _mm256_load_pd( &D0[0+bs*1] );
		e_2 = _mm256_load_pd( &D0[0+bs*2] );
		e_3 = _mm256_load_pd( &D0[0+bs*3] );

		c_01 = _mm256_blend_pd( c_01, e_1, 0x1 );
		c_02 = _mm256_blend_pd( c_02, e_2, 0x3 );
		c_03 = _mm256_blend_pd( c_03, e_3, 0x7 );

		_mm256_store_pd( &D0[0+bs*0], c_00 );
		_mm256_store_pd( &D0[0+bs*1], c_01 );
		_mm256_store_pd( &D0[0+bs*2], c_02 );
		_mm256_store_pd( &D0[0+bs*3], c_03 );

		_mm256_store_pd( &D1[0+bs*0], c_40 );
		_mm256_store_pd( &D1[0+bs*1], c_41 );
		_mm256_store_pd( &D1[0+bs*2], c_42 );
		_mm256_store_pd( &D1[0+bs*3], c_43 );

		e_1 = _mm256_load_pd( &D1[0+bs*5] );
		e_2 = _mm256_load_pd( &D1[0+bs*6] );
		e_3 = _mm256_load_pd( &D1[0+bs*7] );

		c_45 = _mm256_blend_pd( c_45, e_1, 0x1 );
		c_46 = _mm256_blend_pd( c_46, e_2, 0x3 );
		c_47 = _mm256_blend_pd( c_47, e_3, 0x7 );

		_mm256_store_pd( &D1[0+bs*4], c_44 );
		_mm256_store_pd( &D1[0+bs*5], c_45 );
		_mm256_store_pd( &D1[0+bs*6], c_46 );
		_mm256_store_pd( &D1[0+bs*7], c_47 );
		}
	else 
		{
		if(alg==1) // C += A * B'
			{
			e_0 = _mm256_load_pd( &C0[0+bs*0] );
			e_1 = _mm256_load_pd( &C0[0+bs*1] );
			e_2 = _mm256_load_pd( &C0[0+bs*2] );
			e_3 = _mm256_load_pd( &C0[0+bs*3] );

			c_00 = _mm256_add_pd( e_0, c_00 );
			c_01 = _mm256_add_pd( e_1, c_01 );
			c_02 = _mm256_add_pd( e_2, c_02 );
			c_03 = _mm256_add_pd( e_3, c_03 );

			e_0 = _mm256_load_pd( &C1[0+bs*0] );
			e_1 = _mm256_load_pd( &C1[0+bs*1] );
			e_2 = _mm256_load_pd( &C1[0+bs*2] );
			e_3 = _mm256_load_pd( &C1[0+bs*3] );
		
			c_40 = _mm256_add_pd( e_0, c_40 );
			c_41 = _mm256_add_pd( e_1, c_41 );
			c_42 = _mm256_add_pd( e_2, c_42 );
			c_43 = _mm256_add_pd( e_3, c_43 );

			e_0 = _mm256_load_pd( &C1[0+bs*4] );
			e_1 = _mm256_load_pd( &C1[0+bs*5] );
			e_2 = _mm256_load_pd( &C1[0+bs*6] );
			e_3 = _mm256_load_pd( &C1[0+bs*7] );
		
			c_44 = _mm256_add_pd( e_0, c_44 );
			c_45 = _mm256_add_pd( e_1, c_45 );
			c_46 = _mm256_add_pd( e_2, c_46 );
			c_47 = _mm256_add_pd( e_3, c_47 );
			}
		else // C -= A * B'
			{
			e_0 = _mm256_load_pd( &C0[0+bs*0] );
			e_1 = _mm256_load_pd( &C0[0+bs*1] );
			e_2 = _mm256_load_pd( &C0[0+bs*2] );
			e_3 = _mm256_load_pd( &C0[0+bs*3] );

			c_00 = _mm256_sub_pd( e_0, c_00 );
			c_01 = _mm256_sub_pd( e_1, c_01 );
			c_02 = _mm256_sub_pd( e_2, c_02 );
			c_03 = _mm256_sub_pd( e_3, c_03 );

			e_0 = _mm256_load_pd( &C1[0+bs*0] );
			e_1 = _mm256_load_pd( &C1[0+bs*1] );
			e_2 = _mm256_load_pd( &C1[0+bs*2] );
			e_3 = _mm256_load_pd( &C1[0+bs*3] );
		
			c_40 = _mm256_sub_pd( e_0, c_40 );
			c_41 = _mm256_sub_pd( e_1, c_41 );
			c_42 = _mm256_sub_pd( e_2, c_42 );
			c_43 = _mm256_sub_pd( e_3, c_43 );

			e_0 = _mm256_load_pd( &C1[0+bs*4] );
			e_1 = _mm256_load_pd( &C1[0+bs*5] );
			e_2 = _mm256_load_pd( &C1[0+bs*6] );
			e_3 = _mm256_load_pd( &C1[0+bs*7] );
		
			c_44 = _mm256_sub_pd( e_0, c_44 );
			c_45 = _mm256_sub_pd( e_1, c_45 );
			c_46 = _mm256_sub_pd( e_2, c_46 );
			c_47 = _mm256_sub_pd( e_3, c_47 );
			}

		e_1 = _mm256_load_pd( &D0[0+bs*1] );
		e_2 = _mm256_load_pd( &D0[0+bs*2] );
		e_3 = _mm256_load_pd( &D0[0+bs*3] );

		c_01 = _mm256_blend_pd( c_01, e_1, 0x1 );
		c_02 = _mm256_blend_pd( c_02, e_2, 0x3 );
		c_03 = _mm256_blend_pd( c_03, e_3, 0x7 );

		_mm256_store_pd( &D0[0+bs*0], c_00 );
		_mm256_store_pd( &D0[0+bs*1], c_01 );
		_mm256_store_pd( &D0[0+bs*2], c_02 );
		_mm256_store_pd( &D0[0+bs*3], c_03 );

		_mm256_store_pd( &D1[0+bs*0], c_40 );
		_mm256_store_pd( &D1[0+bs*1], c_41 );
		_mm256_store_pd( &D1[0+bs*2], c_42 );
		_mm256_store_pd( &D1[0+bs*3], c_43 );

		e_1 = _mm256_load_pd( &D1[0+bs*5] );
		e_2 = _mm256_load_pd( &D1[0+bs*6] );
		e_3 = _mm256_load_pd( &D1[0+bs*7] );

		c_45 = _mm256_blend_pd( c_45, e_1, 0x1 );
		c_46 = _mm256_blend_pd( c_46, e_2, 0x3 );
		c_47 = _mm256_blend_pd( c_47, e_3, 0x7 );

		_mm256_store_pd( &D1[0+bs*4], c_44 );
		_mm256_store_pd( &D1[0+bs*5], c_45 );
		_mm256_store_pd( &D1[0+bs*6], c_46 );
		_mm256_store_pd( &D1[0+bs*7], c_47 );
		}

		
	}



void kernel_dlauum_nt_8x4_lib4(int kmax, double *A0, int sda, double *B, int alg, double *C0, int sdc, double *D0, int sdd)
	{

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
	a_0123        = _mm256_load_pd( &A0[0] );
	a_4567        = _mm256_load_pd( &A1[0] );
	b_0123        = _mm256_load_pd( &B[0] );

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

	// initial triangles

	// XXX blend A to keep lower-tr clean; no need to blend B since upper-tr is not stored

	// k = 0
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x1 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch

	// k = 1
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x3 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch

	// k = 2
	a_0123        = _mm256_blend_pd( zeros, a_0123, 0x7 );
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[12] ); // prefetch
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
	c_02_13_20_31 = _mm256_fmadd_pd( a_0123, b_2301, c_02_13_20_31 );
	a_0123        = _mm256_load_pd( &A0[12] ); // prefetch

	// k = 3
	c_00_11_22_33 = _mm256_fmadd_pd( a_0123, b_0123, c_00_11_22_33 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[16] ); // prefetch
	c_01_10_23_32 = _mm256_fmadd_pd( a_0123, b_1032, c_01_10_23_32 );
	b_3210        = _mm256_permute2f128_pd( b_1032, b_1032, 0x1 );
	c_03_12_21_30 = _mm256_fmadd_pd( a_0123, b_3210, c_03_12_21_30 );
	b_2301        = _mm256_shuffle_pd( b_3210, b_3210, 0x5 );
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



	// store
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
		d_01_11_21_31 = _mm256_load_pd( &D0[0+bs*1] );
		d_02_12_22_32 = _mm256_load_pd( &D0[0+bs*2] );
		d_03_13_23_33 = _mm256_load_pd( &D0[0+bs*3] );

		c_01_11_21_31 = _mm256_blend_pd( c_01_11_21_31, d_01_11_21_31, 0x1 );
		c_02_12_22_32 = _mm256_blend_pd( c_02_12_22_32, d_02_12_22_32, 0x3 );
		c_03_13_23_33 = _mm256_blend_pd( c_03_13_23_33, d_03_13_23_33, 0x7 );

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

		c_01_11_21_31 = _mm256_load_pd( &D0[0+bs*1] );
		c_02_12_22_32 = _mm256_load_pd( &D0[0+bs*2] );
		c_03_13_23_33 = _mm256_load_pd( &D0[0+bs*3] );

		d_01_11_21_31 = _mm256_blend_pd( d_01_11_21_31, c_01_11_21_31, 0x1 );
		d_02_12_22_32 = _mm256_blend_pd( d_02_12_22_32, c_02_12_22_32, 0x3 );
		d_03_13_23_33 = _mm256_blend_pd( d_03_13_23_33, c_03_13_23_33, 0x7 );

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



void kernel_dlauum_nt_4x4_lib4(int kmax, double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	int k;

	__m256d
		zeros,
		a_0123, //A_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_temp, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31;
	
	// prefetch
	a_0123        = _mm256_load_pd( &A[0] );
	b_0123        = _mm256_load_pd( &B[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();

	k = 0;

	// initial triangles

	// k = 0
	a_0123        = _mm256_blend_pd(zeros, a_0123, 0x1 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch

	// k = 1
	a_0123        = _mm256_blend_pd(zeros, a_0123, 0x3 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
	a_0123        = _mm256_load_pd( &A[8] ); // prefetch

	// k = 2
	a_0123        = _mm256_blend_pd(zeros, a_0123, 0x7 );
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

	// k = 3
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
	k += 4;

	for(; k<kmax-3; k+=4)
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
	for(; k<kmax-1; k+=2)
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

	for(; k<kmax; k+=1)
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
		
//		A += 4;
//		B += 4;
		
		}

	// store
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
		d_01_11_21_31 = _mm256_load_pd( &D[0+bs*1] );
		d_02_12_22_32 = _mm256_load_pd( &D[0+bs*2] );
		d_03_13_23_33 = _mm256_load_pd( &D[0+bs*3] );

		c_01_11_21_31 = _mm256_blend_pd( c_01_11_21_31, d_01_11_21_31, 0x1 );
		c_02_12_22_32 = _mm256_blend_pd( c_02_12_22_32, d_02_12_22_32, 0x3 );
		c_03_13_23_33 = _mm256_blend_pd( c_03_13_23_33, d_03_13_23_33, 0x7 );

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

		c_01_11_21_31 = _mm256_load_pd( &D[0+bs*1] );
		c_02_12_22_32 = _mm256_load_pd( &D[0+bs*2] );
		c_03_13_23_33 = _mm256_load_pd( &D[0+bs*3] );

		d_01_11_21_31 = _mm256_blend_pd( d_01_11_21_31, c_01_11_21_31, 0x1 );
		d_02_12_22_32 = _mm256_blend_pd( d_02_12_22_32, c_02_12_22_32, 0x3 );
		d_03_13_23_33 = _mm256_blend_pd( d_03_13_23_33, c_03_13_23_33, 0x7 );

		_mm256_store_pd( &D[0+bs*0], d_00_10_20_30 );
		_mm256_store_pd( &D[0+bs*1], d_01_11_21_31 );
		_mm256_store_pd( &D[0+bs*2], d_02_12_22_32 );
		_mm256_store_pd( &D[0+bs*3], d_03_13_23_33 );
		}
		
	}


#if ! defined(BLASFEO)

// computes the (lower triangular) diagonal blocks of the symmetric matrix U*U'
void kernel_dsyttmm_ul_nt_8x4_lib4(int kmax, double *A0, int sda, double *B, double *C0, int sdc, double *D0, int sdd, int alg)
	{

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
	a_0123        = _mm256_load_pd( &A0[0] );
	a_4567        = _mm256_load_pd( &A1[0] );
	b_0123        = _mm256_load_pd( &B[0] );

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

	// initial triangles

	ab_tmp1       = _mm256_setzero_pd();

	// k = 0
	a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x1 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
	a_0123        = _mm256_load_pd( &A0[4] ); // prefetch

	// k = 1
	a_0123        = _mm256_blend_pd( ab_tmp1, a_0123, 0x3 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_0123 );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_tmp0 );
	ab_tmp0       = _mm256_mul_pd( a_0123, b_1032 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_tmp0 );
	a_0123        = _mm256_load_pd( &A0[8] ); // prefetch

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
		
//		A0 += 4;
//		A1 += 4;
//		B  += 4;

	
		}

	// store
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
		d_01_11_21_31 = _mm256_load_pd( &D0[0+bs*1] );
		d_02_12_22_32 = _mm256_load_pd( &D0[0+bs*2] );
		d_03_13_23_33 = _mm256_load_pd( &D0[0+bs*3] );

		c_01_11_21_31 = _mm256_blend_pd( c_01_11_21_31, d_01_11_21_31, 0x1 );
		c_02_12_22_32 = _mm256_blend_pd( c_02_12_22_32, d_02_12_22_32, 0x3 );
		c_03_13_23_33 = _mm256_blend_pd( c_03_13_23_33, d_03_13_23_33, 0x7 );

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

		c_01_11_21_31 = _mm256_load_pd( &D0[0+bs*1] );
		c_02_12_22_32 = _mm256_load_pd( &D0[0+bs*2] );
		c_03_13_23_33 = _mm256_load_pd( &D0[0+bs*3] );

		d_01_11_21_31 = _mm256_blend_pd( d_01_11_21_31, c_01_11_21_31, 0x1 );
		d_02_12_22_32 = _mm256_blend_pd( d_02_12_22_32, c_02_12_22_32, 0x3 );
		d_03_13_23_33 = _mm256_blend_pd( d_03_13_23_33, c_03_13_23_33, 0x7 );

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



// computes the (lower triangular) diagonal blocks of the symmetric matrix U*U'
void kernel_dsyttmm_ul_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{

	const int bs = 4;

	int k;

	__m256d
		zeros,
		a_0123, //A_0123,
		b_0123, b_1032, b_3210, b_2301,
		ab_temp, // temporary results
		c_00_11_22_33, c_01_10_23_32, c_03_12_21_30, c_02_13_20_31;
	
	// prefetch
	a_0123        = _mm256_load_pd( &A[0] );
	b_0123        = _mm256_load_pd( &B[0] );

	// zero registers
	zeros = _mm256_setzero_pd();
	c_00_11_22_33 = _mm256_setzero_pd();
	c_01_10_23_32 = _mm256_setzero_pd();
	c_03_12_21_30 = _mm256_setzero_pd();
	c_02_13_20_31 = _mm256_setzero_pd();

	k = 0;

	// initial triangles

	// k = 0
	a_0123        = _mm256_blend_pd(zeros, a_0123, 0x1 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	a_0123        = _mm256_load_pd( &A[4] ); // prefetch
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	b_0123        = _mm256_load_pd( &B[4] ); // prefetch

	// k = 1
	a_0123        = _mm256_blend_pd(zeros, a_0123, 0x3 );
	ab_temp       = _mm256_mul_pd( a_0123, b_0123 );
	c_00_11_22_33 = _mm256_add_pd( c_00_11_22_33, ab_temp );
	b_1032        = _mm256_shuffle_pd( b_0123, b_0123, 0x5 );
	b_0123        = _mm256_load_pd( &B[8] ); // prefetch
	ab_temp       = _mm256_mul_pd( a_0123, b_1032 );
	c_01_10_23_32 = _mm256_add_pd( c_01_10_23_32, ab_temp );
	a_0123        = _mm256_load_pd( &A[8] ); // prefetch

	// k = 2
	a_0123        = _mm256_blend_pd(zeros, a_0123, 0x7 );
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

	// k = 3
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
	k += 4;

	for(; k<kmax-3; k+=4)
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
	for(; k<kmax-1; k+=2)
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

	for(; k<kmax; k+=1)
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
		
//		A += 4;
//		B += 4;
		
		}

	// store
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
		d_01_11_21_31 = _mm256_load_pd( &D[0+bs*1] );
		d_02_12_22_32 = _mm256_load_pd( &D[0+bs*2] );
		d_03_13_23_33 = _mm256_load_pd( &D[0+bs*3] );

		c_01_11_21_31 = _mm256_blend_pd( c_01_11_21_31, d_01_11_21_31, 0x1 );
		c_02_12_22_32 = _mm256_blend_pd( c_02_12_22_32, d_02_12_22_32, 0x3 );
		c_03_13_23_33 = _mm256_blend_pd( c_03_13_23_33, d_03_13_23_33, 0x7 );

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

		c_01_11_21_31 = _mm256_load_pd( &D[0+bs*1] );
		c_02_12_22_32 = _mm256_load_pd( &D[0+bs*2] );
		c_03_13_23_33 = _mm256_load_pd( &D[0+bs*3] );

		d_01_11_21_31 = _mm256_blend_pd( d_01_11_21_31, c_01_11_21_31, 0x1 );
		d_02_12_22_32 = _mm256_blend_pd( d_02_12_22_32, c_02_12_22_32, 0x3 );
		d_03_13_23_33 = _mm256_blend_pd( d_03_13_23_33, c_03_13_23_33, 0x7 );

		_mm256_store_pd( &D[0+bs*0], d_00_10_20_30 );
		_mm256_store_pd( &D[0+bs*1], d_01_11_21_31 );
		_mm256_store_pd( &D[0+bs*2], d_02_12_22_32 );
		_mm256_store_pd( &D[0+bs*3], d_03_13_23_33 );
		}
		
	}



// computes the (lower triangular) diagonal blocks of the symmetric matrix L*L'
void kernel_dsyttmm_lu_nt_4x4_lib4(int kmax, double *A, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;
		
		
		A += 16;

		}
			
	// clean up at the end
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
		
	c_00 += a_0 * a_0;
	c_10 += a_1 * a_0;
	c_20 += a_2 * a_0;
	c_30 += a_3 * a_0;

	c_11 += a_1 * a_1;
	c_21 += a_2 * a_1;
	c_31 += a_3 * a_1;

	c_22 += a_2 * a_2;
	c_32 += a_3 * a_2;

	c_33 += a_3 * a_3;


	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];

	c_11 += a_1 * a_1;
	c_21 += a_2 * a_1;
	c_31 += a_3 * a_1;

	c_22 += a_2 * a_2;
	c_32 += a_3 * a_2;

	c_33 += a_3 * a_3;


	a_2 = A[2+lda*2];
	a_3 = A[3+lda*2];

	c_22 += a_2 * a_2;
	c_32 += a_3 * a_2;

	c_33 += a_3 * a_3;


	a_3 = A[3+lda*3];

	c_33 += a_3 * a_3;
		
		
	// store

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;

	C[2+ldc*2] = c_22;
	C[3+ldc*2] = c_32;

	C[3+ldc*3] = c_33;
	
	}



// normal-transposed, 2x2 with data packed in 4
void kernel_dsyttmm_lu_nt_2x2_lib4(int kmax, double *A, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1,
		c_00=0, 
		c_10=0, c_11=0; 
		
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;
		
		
		A += 16;

		}
			
	for(; k<kmax-2; k+=2)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		A += 8;

		}
			
	// clean up at the end
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
		
	c_00 += a_0 * a_0;
	c_10 += a_1 * a_0;

	c_11 += a_1 * a_1;


	a_1 = A[1+lda*1];

	c_11 += a_1 * a_1;

		
	// store

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;

	C[1+ldc*1] = c_11;
	
	}



// normal-transposed, 4x4 with data packed in 4
void corner_dttmm_ll_nt_4x4_lib4(double *A, double *B, double *C)
	{
	
	const int bs = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];

	b_0 = B[0+bs*0];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;
	
	// k=1
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];

	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	// k=2
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];

	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	// k=3
	a_3 = A[3+bs*3];

	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];

	c_30 += a_3 * b_0;

	c_31 += a_3 * b_1;

	c_32 += a_3 * b_2;

	c_33 += a_3 * b_3;

	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	C[3+bs*3] = c_33;
	
	}



// normal-transposed, 4x4 with data packed in 4
void kernel_dttmm_ll_nt_4x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int bs = 4;

	int k = 0;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];

	b_0 = B[0+bs*0];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;
	
	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];

	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];

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

	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];

	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];

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

	A += 4*bs;
	B += 4*bs;

	k = 4;
	for( ; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
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


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		b_3 = B[3+bs*3];
		
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
		
		A += 4*bs;
		B += 4*bs;

		}

	// k = kmax-4
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];
		
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
		
	// k = kmax-3
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];
		
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k = kmax-2
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];
		
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k = kmax-1
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];
		
	c_30 += a_3 * b_0;

	c_31 += a_3 * b_1;

	c_32 += a_3 * b_2;

	c_33 += a_3 * b_3;
		
	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;
	
	}	



// normal-transposed, 4x4 with data packed in 4
void corner_dttmm_uu_nt_4x4_lib4(double *A, double *B, double *C)
	{
	
	const int bs = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		        c_11=0, c_12=0, c_13=0,
		                c_22=0, c_23=0, 
		                        c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];

	c_00 += a_0 * b_0;

	c_01 += a_0 * b_1;

	c_02 += a_0 * b_2;

	c_03 += a_0 * b_3;

	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];

	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];

	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;

	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];

	b_3 = B[3+bs*3];

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;
	
	// store result
	C[0+bs*0] = c_00;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;

	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;

	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;
	
	}



// normal-transposed, 4x4 with data packed in 4
void corner_dttmm_uu_nt_2x2_lib4(double *A, double *B, double *C)
	{
	
	const int bs = 4;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		        c_11=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];

	c_00 += a_0 * b_0;

	c_01 += a_0 * b_1;

	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];

	b_1 = B[1+bs*1];

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	// store result
	C[0+bs*0] = c_00;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	
	}



// normal-transposed, 4x4 with data packed in 4
void kernel_dttmm_uu_nt_4x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int bs = 4;

	int k = 0;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];

	c_00 += a_0 * b_0;
	
	c_01 += a_0 * b_1;
	
	c_02 += a_0 * b_2;
	
	c_03 += a_0 * b_3;
	
	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];

	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;

	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];

	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];

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

	A += 4*bs;
	B += 4*bs;

	k = 4;
	for( ; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
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


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		b_3 = B[3+bs*3];
		
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
		
		A += 4*bs;
		B += 4*bs;

		}

	// k = kmax-4
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];
		
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
		
	// k = kmax-3
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];
		
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];
		
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

	// k = kmax-2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];
		
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];
		
	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k = kmax-1
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_3 = B[3+bs*3];
		
	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;
		
	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;
	
	}	



// normal-transposed, 4x4 with data packed in 4
void kernel_dttmm_uu_nt_4x2_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int bs = 4;

	int k = 0;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];

	c_00 += a_0 * b_0;
	
	c_01 += a_0 * b_1;
	
	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];

	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;

	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];

	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	A += 4*bs;
	B += 4*bs;

	k = 4;
	for( ; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		A += 4*bs;
		B += 4*bs;

		}

	// k = kmax-4
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;
		
	// k = kmax-3
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];
		
	b_1 = B[1+bs*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;
	
	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;
	
	}	




#endif
