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

//#define TARGET_AMD_K10
//#define TARGET_AMD_BULLDOZER

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#if defined(TARGET_AMD_BULLDOZER)
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#endif


#if ! defined(BLASFEO)

// normal-transposed, 4x4 with data packed in 4
void kernel_dgemm_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

#if defined(TARGET_AMD_BULLDOZER)

	const int ldc = 4;

	int k;
	
	__m128d
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		a_0, a_2,
		b_0, b_1;
	
	d_0 = _mm_setzero_pd();
	d_1 = _mm_setzero_pd();
	d_2 = _mm_setzero_pd();
	d_3 = _mm_setzero_pd();
	d_4 = _mm_setzero_pd();
	d_5 = _mm_setzero_pd();
	d_6 = _mm_setzero_pd();
	d_7 = _mm_setzero_pd();
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_pd(&A[0]);
		a_2 = _mm_load_pd(&A[2]);
		
		b_0 = _mm_loaddup_pd(&B[0]);
		d_0 = _mm_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm_fmadd_pd( a_2, b_0, d_4 );

		b_0 = _mm_loaddup_pd(&B[1]);
		d_1 = _mm_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm_fmadd_pd( a_2, b_0, d_5 );

		b_0 = _mm_loaddup_pd(&B[2]);
		d_2 = _mm_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm_fmadd_pd( a_2, b_0, d_6 );

		b_0 = _mm_loaddup_pd(&B[3]);
		d_3 = _mm_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm_fmadd_pd( a_2, b_0, d_7 );



		a_0 = _mm_load_pd(&A[4]);
		a_2 = _mm_load_pd(&A[6]);
		
		b_0 = _mm_loaddup_pd(&B[4]);
		d_0 = _mm_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm_fmadd_pd( a_2, b_0, d_4 );

		b_0 = _mm_loaddup_pd(&B[5]);
		d_1 = _mm_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm_fmadd_pd( a_2, b_0, d_5 );

		b_0 = _mm_loaddup_pd(&B[6]);
		d_2 = _mm_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm_fmadd_pd( a_2, b_0, d_6 );

		b_0 = _mm_loaddup_pd(&B[7]);
		d_3 = _mm_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm_fmadd_pd( a_2, b_0, d_7 );



		a_0 = _mm_load_pd(&A[8]);
		a_2 = _mm_load_pd(&A[10]);
		
		b_0 = _mm_loaddup_pd(&B[8]);
		d_0 = _mm_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm_fmadd_pd( a_2, b_0, d_4 );

		b_0 = _mm_loaddup_pd(&B[9]);
		d_1 = _mm_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm_fmadd_pd( a_2, b_0, d_5 );

		b_0 = _mm_loaddup_pd(&B[10]);
		d_2 = _mm_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm_fmadd_pd( a_2, b_0, d_6 );

		b_0 = _mm_loaddup_pd(&B[11]);
		d_3 = _mm_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm_fmadd_pd( a_2, b_0, d_7 );



		a_0 = _mm_load_pd(&A[12]);
		a_2 = _mm_load_pd(&A[14]);
		
		b_0 = _mm_loaddup_pd(&B[12]);
		d_0 = _mm_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm_fmadd_pd( a_2, b_0, d_4 );

		b_0 = _mm_loaddup_pd(&B[13]);
		d_1 = _mm_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm_fmadd_pd( a_2, b_0, d_5 );

		b_0 = _mm_loaddup_pd(&B[14]);
		d_2 = _mm_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm_fmadd_pd( a_2, b_0, d_6 );

		b_0 = _mm_loaddup_pd(&B[15]);
		d_3 = _mm_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm_fmadd_pd( a_2, b_0, d_7 );


		
		A += 16;
		B += 16;

		}
	
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_pd(&A[0]);
		a_2 = _mm_load_pd(&A[2]);
		
		b_0 = _mm_loaddup_pd(&B[0]);
		d_0 = _mm_fmadd_pd( a_0, b_0, d_0 );
		d_4 = _mm_fmadd_pd( a_2, b_0, d_4 );

		b_0 = _mm_loaddup_pd(&B[1]);
		d_1 = _mm_fmadd_pd( a_0, b_0, d_1 );
		d_5 = _mm_fmadd_pd( a_2, b_0, d_5 );

		b_0 = _mm_loaddup_pd(&B[2]);
		d_2 = _mm_fmadd_pd( a_0, b_0, d_2 );
		d_6 = _mm_fmadd_pd( a_2, b_0, d_6 );

		b_0 = _mm_loaddup_pd(&B[3]);
		d_3 = _mm_fmadd_pd( a_0, b_0, d_3 );
		d_7 = _mm_fmadd_pd( a_2, b_0, d_7 );


		A += 4;
		B += 4;

		}

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{

			goto store_n;
			}
		else
			{

			goto store_t;
			}
		}
#if 0
	else
		{
		if(tc==0) // C
			{
			c_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			c_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );
			c_2 = _mm_shuffle_pd( c_20_31, c_21_30, 0x2 );
			c_3 = _mm_shuffle_pd( c_21_30, c_20_31, 0x2 );

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_2 = _mm_load_pd( &C[2+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
			d_3 = _mm_load_pd( &C[2+ldc*1] );
		
			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 ); 
				d_1 = _mm_add_pd( d_1, c_1 ); 
				d_2 = _mm_add_pd( d_2, c_2 ); 
				d_3 = _mm_add_pd( d_3, c_3 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 ); 
				d_1 = _mm_sub_pd( d_1, c_1 ); 
				d_2 = _mm_sub_pd( d_2, c_2 ); 
				d_3 = _mm_sub_pd( d_3, c_3 ); 
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_t;
				}
			}
		else // t(C)
			{
			c_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			c_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );
			c_2 = _mm_unpacklo_pd( c_20_31, c_21_30 );
			c_3 = _mm_unpackhi_pd( c_21_30, c_20_31 );

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
			d_2 = _mm_load_pd( &C[0+ldc*2] );
			d_3 = _mm_load_pd( &C[0+ldc*3] );

			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 );
				d_1 = _mm_add_pd( d_1, c_1 );
				d_2 = _mm_add_pd( d_2, c_2 );
				d_3 = _mm_add_pd( d_3, c_3 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 );
				d_1 = _mm_sub_pd( d_1, c_1 );
				d_2 = _mm_sub_pd( d_2, c_2 );
				d_3 = _mm_sub_pd( d_3, c_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}
			}
		}
#endif

	// store (3 - 4) x (1 - 2)
	store_n:
	_mm_store_pd( &D[0+ldc*0], d_0 );
	_mm_store_pd( &D[0+ldc*1], d_1 );
	_mm_store_pd( &D[0+ldc*2], d_2 );
	_mm_store_pd( &D[0+ldc*3], d_3 );
	_mm_store_pd( &D[2+ldc*0], d_4 );
	_mm_store_pd( &D[2+ldc*1], d_5 );
	_mm_store_pd( &D[2+ldc*2], d_6 );
	_mm_store_pd( &D[2+ldc*3], d_7 );
	return;

	store_t:
#if 0
	if(kn>=2)
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_pd( &D[0+ldc*1], d_1 );
		_mm_store_pd( &D[0+ldc*2], d_2 );
		if(km>=4)
			_mm_store_pd( &D[0+ldc*3], d_3 );
		}
	else	
		{
		_mm_store_sd( &D[0+ldc*0], d_0 );
		_mm_store_sd( &D[0+ldc*1], d_1 );
		_mm_store_sd( &D[0+ldc*2], d_2 );
		if(km>=4)
			_mm_store_sd( &D[0+ldc*3], d_3 );
		}
#endif
	return;


#elif defined(TARGET_AMD_K10)

	const int ldc = 4;

	int k;
	
	__m128d
		d_0, d_1, d_2, d_3,
		d_4, d_5, d_6, d_7,
		a_0, a_2,
		b_0, b_1;
	
	d_0 = _mm_setzero_pd();
	d_1 = _mm_setzero_pd();
	d_2 = _mm_setzero_pd();
	d_3 = _mm_setzero_pd();
	d_4 = _mm_setzero_pd();
	d_5 = _mm_setzero_pd();
	d_6 = _mm_setzero_pd();
	d_7 = _mm_setzero_pd();
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{

		a_0 = _mm_load_pd(&A[0]);
		a_2 = _mm_load_pd(&A[2]);
		
		b_0 = _mm_loaddup_pd(&B[0]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_0 = _mm_add_pd( d_0, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_4 = _mm_add_pd( d_4, b_1 );

		b_0 = _mm_loaddup_pd(&B[1]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_1 = _mm_add_pd( d_1, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_5 = _mm_add_pd( d_5, b_1 );

		b_0 = _mm_loaddup_pd(&B[2]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_2 = _mm_add_pd( d_2, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_6 = _mm_add_pd( d_6, b_1 );

		b_0 = _mm_loaddup_pd(&B[3]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_3 = _mm_add_pd( d_3, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_7 = _mm_add_pd( d_7, b_1 );


		a_0 = _mm_load_pd(&A[4]);
		a_2 = _mm_load_pd(&A[6]);
		
		b_0 = _mm_loaddup_pd(&B[4]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_0 = _mm_add_pd( d_0, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_4 = _mm_add_pd( d_4, b_1 );

		b_0 = _mm_loaddup_pd(&B[5]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_1 = _mm_add_pd( d_1, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_5 = _mm_add_pd( d_5, b_1 );

		b_0 = _mm_loaddup_pd(&B[6]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_2 = _mm_add_pd( d_2, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_6 = _mm_add_pd( d_6, b_1 );

		b_0 = _mm_loaddup_pd(&B[7]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_3 = _mm_add_pd( d_3, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_7 = _mm_add_pd( d_7, b_1 );


		a_0 = _mm_load_pd(&A[8]);
		a_2 = _mm_load_pd(&A[10]);
		
		b_0 = _mm_loaddup_pd(&B[8]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_0 = _mm_add_pd( d_0, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_4 = _mm_add_pd( d_4, b_1 );

		b_0 = _mm_loaddup_pd(&B[9]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_1 = _mm_add_pd( d_1, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_5 = _mm_add_pd( d_5, b_1 );

		b_0 = _mm_loaddup_pd(&B[10]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_2 = _mm_add_pd( d_2, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_6 = _mm_add_pd( d_6, b_1 );

		b_0 = _mm_loaddup_pd(&B[11]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_3 = _mm_add_pd( d_3, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_7 = _mm_add_pd( d_7, b_1 );


		a_0 = _mm_load_pd(&A[12]);
		a_2 = _mm_load_pd(&A[14]);
		
		b_0 = _mm_loaddup_pd(&B[12]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_0 = _mm_add_pd( d_0, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_4 = _mm_add_pd( d_4, b_1 );

		b_0 = _mm_loaddup_pd(&B[13]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_1 = _mm_add_pd( d_1, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_5 = _mm_add_pd( d_5, b_1 );

		b_0 = _mm_loaddup_pd(&B[14]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_2 = _mm_add_pd( d_2, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_6 = _mm_add_pd( d_6, b_1 );

		b_0 = _mm_loaddup_pd(&B[15]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_3 = _mm_add_pd( d_3, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_7 = _mm_add_pd( d_7, b_1 );

		
		A += 16;
		B += 16;

		}
	
	for(; k<kmax; k++)
		{

		a_0 = _mm_load_pd(&A[0]);
		a_2 = _mm_load_pd(&A[2]);
		
		b_0 = _mm_loaddup_pd(&B[0]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_0 = _mm_add_pd( d_0, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_4 = _mm_add_pd( d_4, b_1 );

		b_0 = _mm_loaddup_pd(&B[1]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_1 = _mm_add_pd( d_1, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_5 = _mm_add_pd( d_5, b_1 );

		b_0 = _mm_loaddup_pd(&B[2]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_2 = _mm_add_pd( d_2, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_6 = _mm_add_pd( d_6, b_1 );

		b_0 = _mm_loaddup_pd(&B[3]);
		b_1 = b_0;
		b_0 = _mm_mul_pd( a_0, b_0 );
		d_3 = _mm_add_pd( d_3, b_0 );
		b_1 = _mm_mul_pd( a_2, b_1 );
		d_7 = _mm_add_pd( d_7, b_1 );


		A += 4;
		B += 4;

		}

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{

			goto store_n;
			}
		else
			{

			goto store_t;
			}
		}
#if 0
	else
		{
		if(tc==0) // C
			{
			c_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			c_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );
			c_2 = _mm_shuffle_pd( c_20_31, c_21_30, 0x2 );
			c_3 = _mm_shuffle_pd( c_21_30, c_20_31, 0x2 );

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_2 = _mm_load_pd( &C[2+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
			d_3 = _mm_load_pd( &C[2+ldc*1] );
		
			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 ); 
				d_1 = _mm_add_pd( d_1, c_1 ); 
				d_2 = _mm_add_pd( d_2, c_2 ); 
				d_3 = _mm_add_pd( d_3, c_3 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 ); 
				d_1 = _mm_sub_pd( d_1, c_1 ); 
				d_2 = _mm_sub_pd( d_2, c_2 ); 
				d_3 = _mm_sub_pd( d_3, c_3 ); 
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_t;
				}
			}
		else // t(C)
			{
			c_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			c_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );
			c_2 = _mm_unpacklo_pd( c_20_31, c_21_30 );
			c_3 = _mm_unpackhi_pd( c_21_30, c_20_31 );

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
			d_2 = _mm_load_pd( &C[0+ldc*2] );
			d_3 = _mm_load_pd( &C[0+ldc*3] );

			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 );
				d_1 = _mm_add_pd( d_1, c_1 );
				d_2 = _mm_add_pd( d_2, c_2 );
				d_3 = _mm_add_pd( d_3, c_3 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 );
				d_1 = _mm_sub_pd( d_1, c_1 );
				d_2 = _mm_sub_pd( d_2, c_2 );
				d_3 = _mm_sub_pd( d_3, c_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}
			}
		}
#endif

	// store (3 - 4) x (1 - 2)
	store_n:
	_mm_store_pd( &D[0+ldc*0], d_0 );
	_mm_store_pd( &D[0+ldc*1], d_1 );
	_mm_store_pd( &D[0+ldc*2], d_2 );
	_mm_store_pd( &D[0+ldc*3], d_3 );
	_mm_store_pd( &D[2+ldc*0], d_4 );
	_mm_store_pd( &D[2+ldc*1], d_5 );
	_mm_store_pd( &D[2+ldc*2], d_6 );
	_mm_store_pd( &D[2+ldc*3], d_7 );
	return;

	store_t:
#if 0
	if(kn>=2)
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_pd( &D[0+ldc*1], d_1 );
		_mm_store_pd( &D[0+ldc*2], d_2 );
		if(km>=4)
			_mm_store_pd( &D[0+ldc*3], d_3 );
		}
	else	
		{
		_mm_store_sd( &D[0+ldc*0], d_0 );
		_mm_store_sd( &D[0+ldc*1], d_1 );
		_mm_store_sd( &D[0+ldc*2], d_2 );
		if(km>=4)
			_mm_store_sd( &D[0+ldc*3], d_3 );
		}
#endif
	return;


#else
	
	if(kmax<=0)
		{

		const int bs = 4;

		double
			c_00=0, c_01=0, c_02=0, c_03=0,
			c_10=0, c_11=0, c_12=0, c_13=0,
			c_20=0, c_21=0, c_22=0, c_23=0,
			c_30=0, c_31=0, c_32=0, c_33=0;
		
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
		else // D = C +/- A * B'
			{
			if(tc==0) // not transpose C
				{
				c_00 = C[0+bs*0];
				c_10 = C[1+bs*0];
				c_20 = C[2+bs*0];
				c_30 = C[3+bs*0];
				
				c_01 = C[0+bs*1];
				c_11 = C[1+bs*1];
				c_21 = C[2+bs*1];
				c_31 = C[3+bs*1];
				
				c_02 = C[0+bs*2];
				c_12 = C[1+bs*2];
				c_22 = C[2+bs*2];
				c_32 = C[3+bs*2];
				
				c_03 = C[0+bs*3];
				c_13 = C[1+bs*3];
				c_23 = C[2+bs*3];
				c_33 = C[3+bs*3];
				}
			else // transpose C
				{
				c_00 = C[0+bs*0];
				c_01 = C[1+bs*0];
				c_02 = C[2+bs*0];
				c_03 = C[3+bs*0];
				
				c_10 = C[0+bs*1];
				c_11 = C[1+bs*1];
				c_12 = C[2+bs*1];
				c_13 = C[3+bs*1];
				
				c_20 = C[0+bs*2];
				c_21 = C[1+bs*2];
				c_22 = C[2+bs*2];
				c_23 = C[3+bs*2];
				
				c_30 = C[0+bs*3];
				c_31 = C[1+bs*3];
				c_32 = C[2+bs*3];
				c_33 = C[3+bs*3];
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
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		D[0+bs*3] = c_03;
		D[1+bs*3] = c_13;
		D[2+bs*3] = c_23;
		D[3+bs*3] = c_33;
		return;

		store_t:
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		D[3+bs*0] = c_03;
		
		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_12;
		D[3+bs*1] = c_13;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_23;

		D[0+bs*3] = c_30;
		D[1+bs*3] = c_31;
		D[2+bs*3] = c_32;
		D[3+bs*3] = c_33;
		return;

		}

	int k_iter = kmax / 4;
	int k_left = kmax % 4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %2, %%rax         \n\t" // load address of A
		"movq          %3, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
		"movaps    %%xmm3, %%xmm10       \n\t"
		"movaps    %%xmm3, %%xmm11       \n\t"
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // i = k_iter;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .SCONSIDKLEFT            \n\t" // if i == 0, jump to code that
		"                                \n\t" // contains the k_left loop.
		"                                \n\t"
		"                                \n\t"
		".SLOOPKITER:                    \n\t" // MAIN LOOP
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .SLOOPKITER              \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SCONSIDKLEFT:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .SPOSTACCUM              \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".SLOOPKLEFT:                    \n\t" // EDGE LOOP
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"addq          $32, %%rax        \n\t" // A += 4
		"addq          $32, %%rbx        \n\t" // B += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .SLOOPKLEFT              \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SPOSTACCUM:                    \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t" // OK from here !!!!!
		"                                \n\t"
		"movl   %5, %%ecx                \n\t" // alg
		"testl  %%ecx, %%ecx             \n\t" // check alg
		"je     .S0                      \n\t" // if alg==0, jump
		"                                \n\t"
		"movq   %4, %%rax                \n\t" // load address of C
		"                                \n\t"
		"movl   %7, %%edx                \n\t" // tc
		"testl  %%edx, %%edx             \n\t" // check alg
		"je     .S_N                     \n\t" // if tc==0, jump
		"                                \n\t"
		"                                \n\t" // tc==1
		"movaps   %%xmm9,  %%xmm0        \n\t"
		"unpckhpd %%xmm8,  %%xmm0        \n\t"
		"unpcklpd %%xmm9,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm9        \n\t"
		"movaps   %%xmm0,  %%xmm8        \n\t"
		"                                \n\t"
		"movaps   %%xmm11, %%xmm0        \n\t"
		"unpckhpd %%xmm10, %%xmm0        \n\t"
		"unpcklpd %%xmm11, %%xmm10       \n\t"
		"movaps   %%xmm10, %%xmm11       \n\t"
		"movaps   %%xmm0,  %%xmm10       \n\t"
		"                                \n\t"
		"movaps   %%xmm13, %%xmm0        \n\t"
		"unpckhpd %%xmm12, %%xmm0        \n\t"
		"unpcklpd %%xmm13, %%xmm12       \n\t"
		"movaps   %%xmm12, %%xmm13       \n\t"
		"movaps   %%xmm0,  %%xmm12       \n\t"
		"                                \n\t"
		"movaps   %%xmm15, %%xmm0        \n\t"
		"unpckhpd %%xmm14, %%xmm0        \n\t"
		"unpcklpd %%xmm15, %%xmm14       \n\t"
		"movaps   %%xmm14, %%xmm15       \n\t"
		"movaps   %%xmm0,  %%xmm14       \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm1       \n\t" // load C
		"movaps  16(%%rax), %%xmm3       \n\t"
		"movaps  32(%%rax), %%xmm0       \n\t"
		"movaps  48(%%rax), %%xmm2       \n\t"
		"movaps  64(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm7       \n\t"
		"movaps  96(%%rax), %%xmm4       \n\t"
		"movaps 112(%%rax), %%xmm6       \n\t"
		"                                \n\t"
		"cmpl	$1, %%ecx                \n\t"
		"                                \n\t"
		"je     .S1_T                    \n\t" // if alg==1, jump
		"                                \n\t"// alg==-1
		"subpd  %%xmm8,  %%xmm0          \n\t"
		"subpd  %%xmm9,  %%xmm1          \n\t"
		"subpd  %%xmm10, %%xmm2          \n\t"
		"subpd  %%xmm11, %%xmm3          \n\t"
		"subpd  %%xmm12, %%xmm4          \n\t"
		"subpd  %%xmm13, %%xmm5          \n\t"
		"subpd  %%xmm14, %%xmm6          \n\t"
		"subpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		"jmp    .S1_T_E                  \n\t"
		"                                \n\t"
		".S1_T:                          \n\t" // alg==1
		"                                \n\t"
		"addpd  %%xmm8,  %%xmm0          \n\t"
		"addpd  %%xmm9,  %%xmm1          \n\t"
		"addpd  %%xmm10, %%xmm2          \n\t"
		"addpd  %%xmm11, %%xmm3          \n\t"
		"addpd  %%xmm12, %%xmm4          \n\t"
		"addpd  %%xmm13, %%xmm5          \n\t"
		"addpd  %%xmm14, %%xmm6          \n\t"
		"addpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		".S1_T_E:                        \n\t" // end
		"                                \n\t"
		"movl   %8, %%edx                \n\t" // td
		"testl  %%edx, %%edx             \n\t" // check alg
		"jne    .STORE_T                 \n\t" // if td==1, jump
		"                                \n\t"
		"movaps   %%xmm1,  %%xmm8        \n\t"
		"unpcklpd %%xmm0,  %%xmm1        \n\t"
		"unpckhpd %%xmm0,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"                                \n\t"
		"movaps   %%xmm3,  %%xmm9        \n\t"
		"unpcklpd %%xmm2,  %%xmm3        \n\t"
		"unpckhpd %%xmm2,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm2        \n\t"
		"                                \n\t"
		"movaps   %%xmm5,  %%xmm8        \n\t"
		"unpcklpd %%xmm4,  %%xmm5        \n\t"
		"unpckhpd %%xmm4,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm4        \n\t"
		"                                \n\t"
		"movaps   %%xmm7,  %%xmm9        \n\t"
		"unpcklpd %%xmm6,  %%xmm7        \n\t"
		"unpckhpd %%xmm6,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm6        \n\t"
		"                                \n\t"
		"jmp    .STORE_N                 \n\t"
		"                                \n\t"
		"                                \n\t"
		".S_N:                           \n\t" // tc==0
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movsd    %%xmm9,  %%xmm8        \n\t"
		"movsd    %%xmm0,  %%xmm9        \n\t"
		"                                \n\t"
		"movaps  %%xmm10,  %%xmm0        \n\t"
		"movsd   %%xmm11, %%xmm10        \n\t"
		"movsd    %%xmm0, %%xmm11        \n\t"
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm0        \n\t"
		"movsd   %%xmm13, %%xmm12        \n\t"
		"movsd    %%xmm0, %%xmm13        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm0        \n\t"
		"movsd   %%xmm15, %%xmm14        \n\t"
		"movsd    %%xmm0, %%xmm15        \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm1       \n\t" // load C
		"movaps  16(%%rax), %%xmm5       \n\t"
		"movaps  32(%%rax), %%xmm0       \n\t"
		"movaps  48(%%rax), %%xmm4       \n\t"
		"movaps  64(%%rax), %%xmm3       \n\t"
		"movaps  80(%%rax), %%xmm7       \n\t"
		"movaps  96(%%rax), %%xmm2       \n\t"
		"movaps 112(%%rax), %%xmm6       \n\t"
		"                                \n\t"
		"cmpl	$1, %%ecx                \n\t"
		"                                \n\t"
		"je     .S1_N                    \n\t" // if alg==1, jump
		"                                \n\t"// alg==-1
		"subpd  %%xmm8,  %%xmm0          \n\t"
		"subpd  %%xmm9,  %%xmm1          \n\t"
		"subpd  %%xmm10, %%xmm2          \n\t"
		"subpd  %%xmm11, %%xmm3          \n\t"
		"subpd  %%xmm12, %%xmm4          \n\t"
		"subpd  %%xmm13, %%xmm5          \n\t"
		"subpd  %%xmm14, %%xmm6          \n\t"
		"subpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		"jmp    .S1_N_E                  \n\t"
		"                                \n\t"
		".S1_N:                          \n\t" // alg==1
		"                                \n\t"
		"addpd  %%xmm8,  %%xmm0          \n\t"
		"addpd  %%xmm9,  %%xmm1          \n\t"
		"addpd  %%xmm10, %%xmm2          \n\t"
		"addpd  %%xmm11, %%xmm3          \n\t"
		"addpd  %%xmm12, %%xmm4          \n\t"
		"addpd  %%xmm13, %%xmm5          \n\t"
		"addpd  %%xmm14, %%xmm6          \n\t"
		"addpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		".S1_N_E:                        \n\t" // end
		"                                \n\t"
		"movl   %8, %%edx                \n\t" // td
		"testl  %%edx, %%edx             \n\t" // check alg
		"je     .S_N_N                   \n\t" // if td==0, jump
		"                                \n\t"
		"movaps   %%xmm1,  %%xmm8        \n\t"
		"unpcklpd %%xmm0,  %%xmm1        \n\t"
		"unpckhpd %%xmm0,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"                                \n\t"
		"movaps   %%xmm3,  %%xmm9        \n\t"
		"unpcklpd %%xmm2,  %%xmm3        \n\t"
		"unpckhpd %%xmm2,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm2        \n\t"
		"                                \n\t"
		"movaps   %%xmm5,  %%xmm8        \n\t"
		"unpcklpd %%xmm4,  %%xmm5        \n\t"
		"unpckhpd %%xmm4,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm4        \n\t"
		"                                \n\t"
		"movaps   %%xmm7,  %%xmm9        \n\t"
		"unpcklpd %%xmm6,  %%xmm7        \n\t"
		"unpckhpd %%xmm6,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm6        \n\t"
		"                                \n\t"
		"jmp    .STORE_T                 \n\t"
		"                                \n\t"
		".S_N_N:                         \n\t" // td==0
		"                                \n\t"
		"jmp    .STORE_N                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".S0:                            \n\t" // alg==0
		"                                \n\t"
		"movl   %8, %%ecx                \n\t" // td
		"testl  %%ecx, %%ecx             \n\t" // check alg
		"je     .S0_N                    \n\t" // if td==0, jump
		"                                \n\t"
		"                                \n\t" // td==1
		"movaps   %%xmm8,  %%xmm1        \n\t"
		"movaps   %%xmm9,  %%xmm0        \n\t"
		"unpckhpd %%xmm8,  %%xmm0        \n\t"
		"unpcklpd %%xmm9,  %%xmm1        \n\t"
		"                                \n\t"
		"movaps   %%xmm10,  %%xmm3       \n\t"
		"movaps   %%xmm11,  %%xmm2       \n\t"
		"unpckhpd %%xmm10,  %%xmm2       \n\t"
		"unpcklpd %%xmm11,  %%xmm3       \n\t"
		"                                \n\t"
		"movaps   %%xmm12,  %%xmm5       \n\t"
		"movaps   %%xmm13,  %%xmm4       \n\t"
		"unpckhpd %%xmm12,  %%xmm4       \n\t"
		"unpcklpd %%xmm13,  %%xmm5       \n\t"
		"                                \n\t"
		"movaps   %%xmm14,  %%xmm7       \n\t"
		"movaps   %%xmm15,  %%xmm6       \n\t"
		"unpckhpd %%xmm14,  %%xmm6       \n\t"
		"unpcklpd %%xmm15,  %%xmm7       \n\t"
		"                                \n\t"
		"jmp    .STORE_T                 \n\t" // if td==0, jump
		"                                \n\t"
		"                                \n\t"
		".S0_N:                          \n\t" // td==0
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movaps   %%xmm9,  %%xmm1        \n\t"
		"movsd    %%xmm9,  %%xmm0        \n\t"
		"movsd    %%xmm8,  %%xmm1        \n\t"
		"                                \n\t"
		"movaps  %%xmm10,  %%xmm2        \n\t"
		"movaps  %%xmm11,  %%xmm3        \n\t"
		"movsd   %%xmm11,  %%xmm2        \n\t"
		"movsd   %%xmm10,  %%xmm3        \n\t"
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm4        \n\t"
		"movaps  %%xmm13,  %%xmm5        \n\t"
		"movsd   %%xmm13,  %%xmm4        \n\t"
		"movsd   %%xmm12,  %%xmm5        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm6        \n\t"
		"movaps  %%xmm15,  %%xmm7        \n\t"
		"movsd   %%xmm15,  %%xmm6        \n\t"
		"movsd   %%xmm14,  %%xmm7        \n\t"
		"                                \n\t"
		"jmp    .STORE_N                 \n\t" // if td==0, jump
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".STORE_N:                       \n\t"
		"                                \n\t"
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movaps  %%xmm1, (%%rbx)         \n\t"
		"movaps  %%xmm5, 16(%%rbx)       \n\t"
		"movaps  %%xmm0, 32(%%rbx)       \n\t"
		"movaps  %%xmm4, 48(%%rbx)       \n\t"
		"movaps  %%xmm3, 64(%%rbx)       \n\t"
		"movaps  %%xmm7, 80(%%rbx)       \n\t"
		"movaps  %%xmm2, 96(%%rbx)       \n\t"
		"movaps  %%xmm6, 112(%%rbx)      \n\t"
		"                                \n\t"
		"jmp    .SDONE                   \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".STORE_T:                       \n\t"
		"                                \n\t"
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movaps  %%xmm1, (%%rbx)         \n\t"
		"movaps  %%xmm3, 16(%%rbx)       \n\t"
		"movaps  %%xmm0, 32(%%rbx)       \n\t"
		"movaps  %%xmm2, 48(%%rbx)       \n\t"
		"movaps  %%xmm5, 64(%%rbx)       \n\t"
		"movaps  %%xmm7, 80(%%rbx)       \n\t"
		"movaps  %%xmm4, 96(%%rbx)       \n\t"
		"movaps  %%xmm6, 112(%%rbx)      \n\t"
		"                                \n\t"
		"                                \n\t"
		".SDONE:                         \n\t"
		"                                \n\t"

		: // output operands (none)
		: // input operands
		  "m" (k_iter),		// %0
		  "m" (k_left),		// %1
		  "m" (A),			// %2
		  "m" (B),			// %3
		  "m" (C),			// %4
		  "m" (alg),		// %5
		  "m" (D),			// %6
		  "m" (tc),			// %7
		  "m" (td)			// %8
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);

#endif

	}




// normal-transposed, 4x4 with data packed in 4
void kernel_dgemm_nt_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	if(kmax<=0)
		{

		const int bs = 4;

		double
			c_00=0, c_01=0, c_02=0, c_03=0,
			c_10=0, c_11=0, c_12=0, c_13=0,
			c_20=0, c_21=0, c_22=0, c_23=0,
			c_30=0, c_31=0, c_32=0, c_33=0;
		
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
		else // D = C +/- A * B'
			{
			if(tc==0) // not transpose C
				{
				c_00 = C[0+bs*0];
				c_10 = C[1+bs*0];
				c_20 = C[2+bs*0];
				c_30 = C[3+bs*0];
				
				c_01 = C[0+bs*1];
				c_11 = C[1+bs*1];
				c_21 = C[2+bs*1];
				c_31 = C[3+bs*1];
				
				c_02 = C[0+bs*2];
				c_12 = C[1+bs*2];
				c_22 = C[2+bs*2];
				c_32 = C[3+bs*2];
				
				c_03 = C[0+bs*3];
				c_13 = C[1+bs*3];
				c_23 = C[2+bs*3];
				c_33 = C[3+bs*3];
				}
			else // transpose C
				{
				c_00 = C[0+bs*0];
				c_01 = C[1+bs*0];
				c_02 = C[2+bs*0];
				c_03 = C[3+bs*0];
				
				c_10 = C[0+bs*1];
				c_11 = C[1+bs*1];
				c_12 = C[2+bs*1];
				c_13 = C[3+bs*1];
				
				c_20 = C[0+bs*2];
				c_21 = C[1+bs*2];
				c_22 = C[2+bs*2];
				c_23 = C[3+bs*2];
				
				c_30 = C[0+bs*3];
				c_31 = C[1+bs*3];
				c_32 = C[2+bs*3];
				c_33 = C[3+bs*3];
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
		if(km>=4)
			{
			D[0+bs*0] = c_00;
			D[1+bs*0] = c_10;
			D[2+bs*0] = c_20;
			D[3+bs*0] = c_30;

			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;

			D[0+bs*2] = c_02;
			D[1+bs*2] = c_12;
			D[2+bs*2] = c_22;
			D[3+bs*2] = c_32;

			if(kn>=4)
				{
				D[0+bs*3] = c_03;
				D[1+bs*3] = c_13;
				D[2+bs*3] = c_23;
				D[3+bs*3] = c_33;
				}
			}
		else // km==3
			{
			D[0+bs*0] = c_00;
			D[1+bs*0] = c_10;
			D[2+bs*0] = c_20;

			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;

			D[0+bs*2] = c_02;
			D[1+bs*2] = c_12;
			D[2+bs*2] = c_22;

			if(kn>=4)
				{
				D[0+bs*3] = c_03;
				D[1+bs*3] = c_13;
				D[2+bs*3] = c_23;
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
			
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			D[3+bs*1] = c_13;

			D[0+bs*2] = c_20;
			D[1+bs*2] = c_21;
			D[2+bs*2] = c_22;
			D[3+bs*2] = c_23;

			if(km>=4)
				{
				D[0+bs*3] = c_30;
				D[1+bs*3] = c_31;
				D[2+bs*3] = c_32;
				D[3+bs*3] = c_33;
				}
			}
		else // kn==3
			{
			D[0+bs*0] = c_00;
			D[1+bs*0] = c_01;
			D[2+bs*0] = c_02;
			
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;

			D[0+bs*2] = c_20;
			D[1+bs*2] = c_21;
			D[2+bs*2] = c_22;

			if(km>=4)
				{
				D[0+bs*3] = c_30;
				D[1+bs*3] = c_31;
				D[2+bs*3] = c_32;
				}
			}
		return;

		}

	int k_iter = kmax / 4;
	int k_left = kmax % 4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %2, %%rax         \n\t" // load address of A
		"movq          %3, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
		"movaps    %%xmm3, %%xmm10       \n\t"
		"movaps    %%xmm3, %%xmm11       \n\t"
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // i = k_iter;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .TCONSIDKLEFT            \n\t" // if i == 0, jump to code that
		"                                \n\t" // contains the k_left loop.
		"                                \n\t"
		"                                \n\t"
		".TLOOPKITER:                    \n\t" // MAIN LOOP
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .TLOOPKITER              \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".TCONSIDKLEFT:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .TPOSTACCUM              \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".TLOOPKLEFT:                    \n\t" // EDGE LOOP
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"addq          $32, %%rax        \n\t" // A += 4
		"addq          $32, %%rbx        \n\t" // B += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .TLOOPKLEFT              \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".TPOSTACCUM:                    \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t" // OK from here !!!!!
		"                                \n\t"
		"movl   %5, %%ecx                \n\t" // alg
		"testl  %%ecx, %%ecx             \n\t" // check alg
		"je     .T0                      \n\t" // if alg==0, jump
		"                                \n\t"
		"movq   %4, %%rax                \n\t" // load address of C
		"                                \n\t"
		"movl   %7, %%edx                \n\t" // tc
		"testl  %%edx, %%edx             \n\t" // check alg
		"je     .T_N                     \n\t" // if tc==0, jump
		"                                \n\t"
		"                                \n\t" // tc==1
		"movaps   %%xmm9,  %%xmm0        \n\t"
		"unpckhpd %%xmm8,  %%xmm0        \n\t"
		"unpcklpd %%xmm9,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm9        \n\t"
		"movaps   %%xmm0,  %%xmm8        \n\t"
		"                                \n\t"
		"movaps   %%xmm11, %%xmm0        \n\t"
		"unpckhpd %%xmm10, %%xmm0        \n\t"
		"unpcklpd %%xmm11, %%xmm10       \n\t"
		"movaps   %%xmm10, %%xmm11       \n\t"
		"movaps   %%xmm0,  %%xmm10       \n\t"
		"                                \n\t"
		"movaps   %%xmm13, %%xmm0        \n\t"
		"unpckhpd %%xmm12, %%xmm0        \n\t"
		"unpcklpd %%xmm13, %%xmm12       \n\t"
		"movaps   %%xmm12, %%xmm13       \n\t"
		"movaps   %%xmm0,  %%xmm12       \n\t"
		"                                \n\t"
		"movaps   %%xmm15, %%xmm0        \n\t"
		"unpckhpd %%xmm14, %%xmm0        \n\t"
		"unpcklpd %%xmm15, %%xmm14       \n\t"
		"movaps   %%xmm14, %%xmm15       \n\t"
		"movaps   %%xmm0,  %%xmm14       \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm1       \n\t" // load C
		"movaps  16(%%rax), %%xmm3       \n\t"
		"movaps  32(%%rax), %%xmm0       \n\t"
		"movaps  48(%%rax), %%xmm2       \n\t"
		"movaps  64(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm7       \n\t"
		"movaps  96(%%rax), %%xmm4       \n\t"
		"movaps 112(%%rax), %%xmm6       \n\t"
		"                                \n\t"
		"cmpl	$1, %%ecx                \n\t"
		"                                \n\t"
		"je     .T1_T                    \n\t" // if alg==1, jump
		"                                \n\t"// alg==-1
		"subpd  %%xmm8,  %%xmm0          \n\t"
		"subpd  %%xmm9,  %%xmm1          \n\t"
		"subpd  %%xmm10, %%xmm2          \n\t"
		"subpd  %%xmm11, %%xmm3          \n\t"
		"subpd  %%xmm12, %%xmm4          \n\t"
		"subpd  %%xmm13, %%xmm5          \n\t"
		"subpd  %%xmm14, %%xmm6          \n\t"
		"subpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		"jmp    .T1_T_E                  \n\t"
		"                                \n\t"
		".T1_T:                          \n\t" // alg==1
		"                                \n\t"
		"addpd  %%xmm8,  %%xmm0          \n\t"
		"addpd  %%xmm9,  %%xmm1          \n\t"
		"addpd  %%xmm10, %%xmm2          \n\t"
		"addpd  %%xmm11, %%xmm3          \n\t"
		"addpd  %%xmm12, %%xmm4          \n\t"
		"addpd  %%xmm13, %%xmm5          \n\t"
		"addpd  %%xmm14, %%xmm6          \n\t"
		"addpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		".T1_T_E:                        \n\t" // end
		"                                \n\t"
		"movl   %8, %%edx                \n\t" // td
		"testl  %%edx, %%edx             \n\t" // check alg
		"jne    .TTORE_T                 \n\t" // if td==1, jump
		"                                \n\t"
		"movaps   %%xmm1,  %%xmm8        \n\t"
		"unpcklpd %%xmm0,  %%xmm1        \n\t"
		"unpckhpd %%xmm0,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"                                \n\t"
		"movaps   %%xmm3,  %%xmm9        \n\t"
		"unpcklpd %%xmm2,  %%xmm3        \n\t"
		"unpckhpd %%xmm2,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm2        \n\t"
		"                                \n\t"
		"movaps   %%xmm5,  %%xmm8        \n\t"
		"unpcklpd %%xmm4,  %%xmm5        \n\t"
		"unpckhpd %%xmm4,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm4        \n\t"
		"                                \n\t"
		"movaps   %%xmm7,  %%xmm9        \n\t"
		"unpcklpd %%xmm6,  %%xmm7        \n\t"
		"unpckhpd %%xmm6,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm6        \n\t"
		"                                \n\t"
		"jmp    .TTORE_N                 \n\t"
		"                                \n\t"
		"                                \n\t"
		".T_N:                           \n\t" // tc==0
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movsd    %%xmm9,  %%xmm8        \n\t"
		"movsd    %%xmm0,  %%xmm9        \n\t"
		"                                \n\t"
		"movaps  %%xmm10,  %%xmm0        \n\t"
		"movsd   %%xmm11, %%xmm10        \n\t"
		"movsd    %%xmm0, %%xmm11        \n\t"
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm0        \n\t"
		"movsd   %%xmm13, %%xmm12        \n\t"
		"movsd    %%xmm0, %%xmm13        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm0        \n\t"
		"movsd   %%xmm15, %%xmm14        \n\t"
		"movsd    %%xmm0, %%xmm15        \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm1       \n\t" // load C
		"movaps  16(%%rax), %%xmm5       \n\t"
		"movaps  32(%%rax), %%xmm0       \n\t"
		"movaps  48(%%rax), %%xmm4       \n\t"
		"movaps  64(%%rax), %%xmm3       \n\t"
		"movaps  80(%%rax), %%xmm7       \n\t"
		"movaps  96(%%rax), %%xmm2       \n\t"
		"movaps 112(%%rax), %%xmm6       \n\t"
		"                                \n\t"
		"cmpl	$1, %%ecx                \n\t"
		"                                \n\t"
		"je     .T1_N                    \n\t" // if alg==1, jump
		"                                \n\t"// alg==-1
		"subpd  %%xmm8,  %%xmm0          \n\t"
		"subpd  %%xmm9,  %%xmm1          \n\t"
		"subpd  %%xmm10, %%xmm2          \n\t"
		"subpd  %%xmm11, %%xmm3          \n\t"
		"subpd  %%xmm12, %%xmm4          \n\t"
		"subpd  %%xmm13, %%xmm5          \n\t"
		"subpd  %%xmm14, %%xmm6          \n\t"
		"subpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		"jmp    .T1_N_E                  \n\t"
		"                                \n\t"
		".T1_N:                          \n\t" // alg==1
		"                                \n\t"
		"addpd  %%xmm8,  %%xmm0          \n\t"
		"addpd  %%xmm9,  %%xmm1          \n\t"
		"addpd  %%xmm10, %%xmm2          \n\t"
		"addpd  %%xmm11, %%xmm3          \n\t"
		"addpd  %%xmm12, %%xmm4          \n\t"
		"addpd  %%xmm13, %%xmm5          \n\t"
		"addpd  %%xmm14, %%xmm6          \n\t"
		"addpd  %%xmm15, %%xmm7          \n\t"
		"                                \n\t"
		".T1_N_E:                        \n\t" // end
		"                                \n\t"
		"movl   %8, %%edx                \n\t" // td
		"testl  %%edx, %%edx             \n\t" // check alg
		"je     .T_N_N                   \n\t" // if td==0, jump
		"                                \n\t"
		"movaps   %%xmm1,  %%xmm8        \n\t"
		"unpcklpd %%xmm0,  %%xmm1        \n\t"
		"unpckhpd %%xmm0,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"                                \n\t"
		"movaps   %%xmm3,  %%xmm9        \n\t"
		"unpcklpd %%xmm2,  %%xmm3        \n\t"
		"unpckhpd %%xmm2,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm2        \n\t"
		"                                \n\t"
		"movaps   %%xmm5,  %%xmm8        \n\t"
		"unpcklpd %%xmm4,  %%xmm5        \n\t"
		"unpckhpd %%xmm4,  %%xmm8        \n\t"
		"movaps   %%xmm8,  %%xmm4        \n\t"
		"                                \n\t"
		"movaps   %%xmm7,  %%xmm9        \n\t"
		"unpcklpd %%xmm6,  %%xmm7        \n\t"
		"unpckhpd %%xmm6,  %%xmm9        \n\t"
		"movaps   %%xmm9,  %%xmm6        \n\t"
		"                                \n\t"
		"jmp    .TTORE_T                 \n\t"
		"                                \n\t"
		".T_N_N:                         \n\t" // td==0
		"                                \n\t"
		"jmp    .TTORE_N                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".T0:                            \n\t" // alg==0
		"                                \n\t"
		"movl   %8, %%ecx                \n\t" // td
		"testl  %%ecx, %%ecx             \n\t" // check alg
		"je     .T0_N                    \n\t" // if td==0, jump
		"                                \n\t"
		"                                \n\t" // td==1
		"movaps   %%xmm8,  %%xmm1        \n\t"
		"movaps   %%xmm9,  %%xmm0        \n\t"
		"unpckhpd %%xmm8,  %%xmm0        \n\t"
		"unpcklpd %%xmm9,  %%xmm1        \n\t"
		"                                \n\t"
		"movaps   %%xmm10,  %%xmm3       \n\t"
		"movaps   %%xmm11,  %%xmm2       \n\t"
		"unpckhpd %%xmm10,  %%xmm2       \n\t"
		"unpcklpd %%xmm11,  %%xmm3       \n\t"
		"                                \n\t"
		"movaps   %%xmm12,  %%xmm5       \n\t"
		"movaps   %%xmm13,  %%xmm4       \n\t"
		"unpckhpd %%xmm12,  %%xmm4       \n\t"
		"unpcklpd %%xmm13,  %%xmm5       \n\t"
		"                                \n\t"
		"movaps   %%xmm14,  %%xmm7       \n\t"
		"movaps   %%xmm15,  %%xmm6       \n\t"
		"unpckhpd %%xmm14,  %%xmm6       \n\t"
		"unpcklpd %%xmm15,  %%xmm7       \n\t"
		"                                \n\t"
		"jmp    .TTORE_T                 \n\t" // if td==0, jump
		"                                \n\t"
		"                                \n\t"
		".T0_N:                          \n\t" // td==0
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movaps   %%xmm9,  %%xmm1        \n\t"
		"movsd    %%xmm9,  %%xmm0        \n\t"
		"movsd    %%xmm8,  %%xmm1        \n\t"
		"                                \n\t"
		"movaps  %%xmm10,  %%xmm2        \n\t"
		"movaps  %%xmm11,  %%xmm3        \n\t"
		"movsd   %%xmm11,  %%xmm2        \n\t"
		"movsd   %%xmm10,  %%xmm3        \n\t"
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm4        \n\t"
		"movaps  %%xmm13,  %%xmm5        \n\t"
		"movsd   %%xmm13,  %%xmm4        \n\t"
		"movsd   %%xmm12,  %%xmm5        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm6        \n\t"
		"movaps  %%xmm15,  %%xmm7        \n\t"
		"movsd   %%xmm15,  %%xmm6        \n\t"
		"movsd   %%xmm14,  %%xmm7        \n\t"
		"                                \n\t"
		"jmp    .TTORE_N                 \n\t" // if td==0, jump
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".TTORE_N:                       \n\t"
		"                                \n\t"
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movl   %9, %%edx                \n\t" // km
		"movl  %10, %%ecx                \n\t" // kn
		"cmpl	$4, %%edx                \n\t"
		"jl      .T_N_3                  \n\t" // if km<4, jump
		"                                \n\t"
		"movaps  %%xmm1, (%%rbx)         \n\t"
		"movaps  %%xmm5, 16(%%rbx)       \n\t"
		"cmpl	$4, %%ecx                \n\t"
		"movaps  %%xmm0, 32(%%rbx)       \n\t"
		"movaps  %%xmm4, 48(%%rbx)       \n\t"
		"movaps  %%xmm3, 64(%%rbx)       \n\t"
		"movaps  %%xmm7, 80(%%rbx)       \n\t"
		"jl      .TDONE                  \n\t" // if kn<4, jump
		"movaps  %%xmm2, 96(%%rbx)       \n\t"
		"movaps  %%xmm6, 112(%%rbx)      \n\t"
		"                                \n\t"
		".T_N_3:                         \n\t"
		"                                \n\t"
		"movaps  %%xmm1, (%%rbx)         \n\t"
		"movsd   %%xmm5, 16(%%rbx)       \n\t"
		"cmpl	$4, %%ecx                \n\t"
		"movaps  %%xmm0, 32(%%rbx)       \n\t"
		"movsd   %%xmm4, 48(%%rbx)       \n\t"
		"movaps  %%xmm3, 64(%%rbx)       \n\t"
		"movsd   %%xmm7, 80(%%rbx)       \n\t"
		"jl      .TDONE                  \n\t" // if km<4, jump
		"movaps  %%xmm2, 96(%%rbx)       \n\t"
		"movsd   %%xmm6, 112(%%rbx)      \n\t"
		"                                \n\t"
		"jmp    .TDONE                   \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".TTORE_T:                       \n\t"
		"                                \n\t"
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movl  %10, %%ecx                \n\t" // kn
		"movl   %9, %%edx                \n\t" // km
		"cmpl	$4, %%ecx                \n\t"
		"jl      .T_T_3                  \n\t" // if kn<4, jump
		"                                \n\t"
		"movaps  %%xmm1, (%%rbx)         \n\t"
		"movaps  %%xmm3, 16(%%rbx)       \n\t"
		"cmpl	$4, %%edx                \n\t"
		"movaps  %%xmm0, 32(%%rbx)       \n\t"
		"movaps  %%xmm2, 48(%%rbx)       \n\t"
		"movaps  %%xmm5, 64(%%rbx)       \n\t"
		"movaps  %%xmm7, 80(%%rbx)       \n\t"
		"jl      .TDONE                  \n\t" // if km<4, jump
		"movaps  %%xmm4, 96(%%rbx)       \n\t"
		"movaps  %%xmm6, 112(%%rbx)      \n\t"
		"                                \n\t"
		".T_T_3:                         \n\t"
		"                                \n\t"
		"movaps  %%xmm1, (%%rbx)         \n\t"
		"movsd   %%xmm3, 16(%%rbx)       \n\t"
		"cmpl	$4, %%edx                \n\t"
		"movaps  %%xmm0, 32(%%rbx)       \n\t"
		"movsd   %%xmm2, 48(%%rbx)       \n\t"
		"movaps  %%xmm5, 64(%%rbx)       \n\t"
		"movsd   %%xmm7, 80(%%rbx)       \n\t"
		"jl      .TDONE                  \n\t" // if km<4, jump
		"movaps  %%xmm4, 96(%%rbx)       \n\t"
		"movsd   %%xmm6, 112(%%rbx)      \n\t"
		"                                \n\t"
		"                                \n\t"
		".TDONE:                         \n\t"
		"                                \n\t"

		: // output operands (none)
		: // input operands
		  "m" (k_iter),		// %0
		  "m" (k_left),		// %1
		  "m" (A),			// %2
		  "m" (B),			// %3
		  "m" (C),			// %4
		  "m" (alg),		// %5
		  "m" (D),			// %6
		  "m" (tc),			// %7
		  "m" (td),			// %8
		  "m" (km),			// %9
		  "m" (kn)			// %10
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}




// 4x2 with data packed in 4
void kernel_dgemm_nt_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;

	const int ldc = 4;

	int k;
	
	__m128d
		c_00_11, c_01_10, c_20_31, c_21_30,
		a_01, a_23,
		b_01, b_10, b_temp_0, b_temp_1;
	
	c_00_11 = _mm_setzero_pd();
	c_01_10 = _mm_setzero_pd();
	c_20_31 = _mm_setzero_pd();
	c_21_30 = _mm_setzero_pd();
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{
		a_01 = _mm_load_pd(&A[0]);
		a_23 = _mm_load_pd(&A[2]);
		
		b_01 = _mm_load_pd(&B[0]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_temp_0 = b_01;
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );
		b_temp_0 = _mm_mul_pd( a_23, b_temp_0 );
		c_20_31 = _mm_add_pd( c_20_31, b_temp_0 );

		b_temp_1 = b_10;
		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		b_temp_1 = _mm_mul_pd( a_23, b_temp_1 );
		c_21_30 = _mm_add_pd( c_21_30, b_temp_1 );
		
		
		a_01 = _mm_load_pd(&A[4]);
		a_23 = _mm_load_pd(&A[6]);
		
		b_01 = _mm_load_pd(&B[4]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_temp_0 = b_01;
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );
		b_temp_0 = _mm_mul_pd( a_23, b_temp_0 );
		c_20_31 = _mm_add_pd( c_20_31, b_temp_0 );

		b_temp_1 = b_10;
		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		b_temp_1 = _mm_mul_pd( a_23, b_temp_1 );
		c_21_30 = _mm_add_pd( c_21_30, b_temp_1 );
		
		
		a_01 = _mm_load_pd(&A[8]);
		a_23 = _mm_load_pd(&A[10]);
		
		b_01 = _mm_load_pd(&B[8]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_temp_0 = b_01;
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );
		b_temp_0 = _mm_mul_pd( a_23, b_temp_0 );
		c_20_31 = _mm_add_pd( c_20_31, b_temp_0 );

		b_temp_1 = b_10;
		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		b_temp_1 = _mm_mul_pd( a_23, b_temp_1 );
		c_21_30 = _mm_add_pd( c_21_30, b_temp_1 );
		
		
		a_01 = _mm_load_pd(&A[12]);
		a_23 = _mm_load_pd(&A[14]);
		
		b_01 = _mm_load_pd(&B[12]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_temp_0 = b_01;
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );
		b_temp_0 = _mm_mul_pd( a_23, b_temp_0 );
		c_20_31 = _mm_add_pd( c_20_31, b_temp_0 );

		b_temp_1 = b_10;
		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		b_temp_1 = _mm_mul_pd( a_23, b_temp_1 );
		c_21_30 = _mm_add_pd( c_21_30, b_temp_1 );

		
		A += 16;
		B += 16;

		}
	
	for(; k<kmax; k++)
		{

		a_01 = _mm_load_pd(&A[0]);
		a_23 = _mm_load_pd(&A[2]);
		
		b_01 = _mm_load_pd(&B[0]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_temp_0 = b_01;
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );
		b_temp_0 = _mm_mul_pd( a_23, b_temp_0 );
		c_20_31 = _mm_add_pd( c_20_31, b_temp_0 );

		b_temp_1 = b_10;
		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		b_temp_1 = _mm_mul_pd( a_23, b_temp_1 );
		c_21_30 = _mm_add_pd( c_21_30, b_temp_1 );
		

		A += 4;
		B += 4;

		}

	__m128d
		t_0,
		c_0, c_1, c_2, c_3,
		d_0, d_1, d_2, d_3;

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			d_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			d_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );
			d_2 = _mm_shuffle_pd( c_20_31, c_21_30, 0x2 );
			d_3 = _mm_shuffle_pd( c_21_30, c_20_31, 0x2 );

			goto store_n;
			}
		else
			{
			d_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			d_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );
			d_2 = _mm_unpacklo_pd( c_20_31, c_21_30 );
			d_3 = _mm_unpackhi_pd( c_21_30, c_20_31 );

			goto store_t;
			}
		}
	else
		{
		if(tc==0) // C
			{
			c_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			c_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );
			c_2 = _mm_shuffle_pd( c_20_31, c_21_30, 0x2 );
			c_3 = _mm_shuffle_pd( c_21_30, c_20_31, 0x2 );

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_2 = _mm_load_pd( &C[2+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
			d_3 = _mm_load_pd( &C[2+ldc*1] );
		
			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 ); 
				d_1 = _mm_add_pd( d_1, c_1 ); 
				d_2 = _mm_add_pd( d_2, c_2 ); 
				d_3 = _mm_add_pd( d_3, c_3 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 ); 
				d_1 = _mm_sub_pd( d_1, c_1 ); 
				d_2 = _mm_sub_pd( d_2, c_2 ); 
				d_3 = _mm_sub_pd( d_3, c_3 ); 
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_t;
				}
			}
		else // t(C)
			{
			c_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			c_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );
			c_2 = _mm_unpacklo_pd( c_20_31, c_21_30 );
			c_3 = _mm_unpackhi_pd( c_21_30, c_20_31 );

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
			d_2 = _mm_load_pd( &C[0+ldc*2] );
			d_3 = _mm_load_pd( &C[0+ldc*3] );

			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 );
				d_1 = _mm_add_pd( d_1, c_1 );
				d_2 = _mm_add_pd( d_2, c_2 );
				d_3 = _mm_add_pd( d_3, c_3 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 );
				d_1 = _mm_sub_pd( d_1, c_1 );
				d_2 = _mm_sub_pd( d_2, c_2 );
				d_3 = _mm_sub_pd( d_3, c_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}
			}
		}

	// store (3 - 4) x (1 - 2)
	store_n:
	if(km>=4)
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_pd( &D[2+ldc*0], d_2 );

		if(kn>=2)
			{
			_mm_store_pd( &D[0+ldc*1], d_1 );
			_mm_store_pd( &D[2+ldc*1], d_3 );
			}
		}
	else // km==3
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_sd( &D[2+ldc*0], d_2 );

		if(kn>=2)
			{
			_mm_store_pd( &D[0+ldc*1], d_1 );
			_mm_store_sd( &D[2+ldc*1], d_3 );
			}
		}
	return;

	store_t:
	if(kn>=2)
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_pd( &D[0+ldc*1], d_1 );
		_mm_store_pd( &D[0+ldc*2], d_2 );
		if(km>=4)
			_mm_store_pd( &D[0+ldc*3], d_3 );
		}
	else	
		{
		_mm_store_sd( &D[0+ldc*0], d_0 );
		_mm_store_sd( &D[0+ldc*1], d_1 );
		_mm_store_sd( &D[0+ldc*2], d_2 );
		if(km>=4)
			_mm_store_sd( &D[0+ldc*3], d_3 );
		}
	return;

	}



// 4x2 with data packed in 4
void kernel_dgemm_nt_4x2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nt_4x2_vs_lib4(4, 2, kmax, A, B, C, D, alg, tc, td);

	}



// 2x4 with data packed in 4
void kernel_dgemm_nt_2x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;

	const int ldc = 4;

	int k;
	
	__m128d
		c_00_11, c_01_10, c_02_13, c_03_12,
		a_01,
		b_01, b_10, b_23, b_32;
	
	c_00_11 = _mm_setzero_pd();
	c_01_10 = _mm_setzero_pd();
	c_02_13 = _mm_setzero_pd();
	c_03_12 = _mm_setzero_pd();
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{
		a_01 = _mm_load_pd(&A[0]);
		
		b_01 = _mm_load_pd(&B[0]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
		b_23 = _mm_load_pd(&B[2]);
		b_32 = _mm_shuffle_pd(b_23, b_23, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		b_23 = _mm_mul_pd( a_01, b_23 );
		c_02_13 = _mm_add_pd( c_02_13, b_23 );

		b_32 = _mm_mul_pd( a_01, b_32 );
		c_03_12 = _mm_add_pd( c_03_12, b_32 );
		
		
		a_01 = _mm_load_pd(&A[4]);
		
		b_01 = _mm_load_pd(&B[4]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
		b_23 = _mm_load_pd(&B[6]);
		b_32 = _mm_shuffle_pd(b_23, b_23, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		b_23 = _mm_mul_pd( a_01, b_23 );
		c_02_13 = _mm_add_pd( c_02_13, b_23 );

		b_32 = _mm_mul_pd( a_01, b_32 );
		c_03_12 = _mm_add_pd( c_03_12, b_32 );
		
		
		a_01 = _mm_load_pd(&A[8]);
		
		b_01 = _mm_load_pd(&B[8]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
		b_23 = _mm_load_pd(&B[10]);
		b_32 = _mm_shuffle_pd(b_23, b_23, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		b_23 = _mm_mul_pd( a_01, b_23 );
		c_02_13 = _mm_add_pd( c_02_13, b_23 );

		b_32 = _mm_mul_pd( a_01, b_32 );
		c_03_12 = _mm_add_pd( c_03_12, b_32 );
		
		
		a_01 = _mm_load_pd(&A[12]);
		
		b_01 = _mm_load_pd(&B[12]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
		b_23 = _mm_load_pd(&B[14]);
		b_32 = _mm_shuffle_pd(b_23, b_23, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		b_23 = _mm_mul_pd( a_01, b_23 );
		c_02_13 = _mm_add_pd( c_02_13, b_23 );

		b_32 = _mm_mul_pd( a_01, b_32 );
		c_03_12 = _mm_add_pd( c_03_12, b_32 );

		
		A += 16;
		B += 16;

		}
	
	for(; k<kmax; k++)
		{

		a_01 = _mm_load_pd(&A[0]);
		
		b_01 = _mm_load_pd(&B[0]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
		b_23 = _mm_load_pd(&B[2]);
		b_32 = _mm_shuffle_pd(b_23, b_23, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		b_23 = _mm_mul_pd( a_01, b_23 );
		c_02_13 = _mm_add_pd( c_02_13, b_23 );

		b_32 = _mm_mul_pd( a_01, b_32 );
		c_03_12 = _mm_add_pd( c_03_12, b_32 );
		

		A += 4;
		B += 4;

		}

	__m128d
		t_0,
		c_0, c_1, c_2, c_3,
		d_0, d_1, d_2, d_3;

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			d_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			d_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );
			d_2 = _mm_shuffle_pd( c_02_13, c_03_12, 0x2 );
			d_3 = _mm_shuffle_pd( c_03_12, c_02_13, 0x2 );

			goto store_n;
			}
		else
			{
			d_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			d_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );
			d_2 = _mm_unpacklo_pd( c_02_13, c_03_12 );
			d_3 = _mm_unpackhi_pd( c_03_12, c_02_13 );

			goto store_t;
			}
		}
	else
		{
		if(tc==0) // C
			{
			c_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			c_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );
			c_2 = _mm_shuffle_pd( c_02_13, c_03_12, 0x2 );
			c_3 = _mm_shuffle_pd( c_03_12, c_02_13, 0x2 );

			d_0 = _mm_load_pd(&C[0+ldc*0]);
			d_1 = _mm_load_pd(&C[0+ldc*1]);
			d_2 = _mm_load_pd(&C[0+ldc*2]);
			d_3 = _mm_load_pd(&C[0+ldc*3]);
		
			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 ); 
				d_1 = _mm_add_pd( d_1, c_1 ); 
				d_2 = _mm_add_pd( d_2, c_2 ); 
				d_3 = _mm_add_pd( d_3, c_3 ); 
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 ); 
				d_1 = _mm_sub_pd( d_1, c_1 ); 
				d_2 = _mm_sub_pd( d_2, c_2 ); 
				d_3 = _mm_sub_pd( d_3, c_3 ); 
				}

			if(td==0) // AB + C
				{
				goto store_n;
				}
			else // t(AB + C)
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_t;
				}
			}
		else // t(C)
			{
			c_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			c_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );
			c_2 = _mm_unpacklo_pd( c_02_13, c_03_12 );
			c_3 = _mm_unpackhi_pd( c_03_12, c_02_13 );

			d_0 = _mm_load_pd( &C[0+ldc*0] );
			d_1 = _mm_load_pd( &C[0+ldc*1] );
			d_2 = _mm_load_pd( &C[2+ldc*0] );
			d_3 = _mm_load_pd( &C[2+ldc*1] );

			if(alg==1) // AB = A * B'
				{
				d_0 = _mm_add_pd( d_0, c_0 );
				d_1 = _mm_add_pd( d_1, c_1 );
				d_2 = _mm_add_pd( d_2, c_2 );
				d_3 = _mm_add_pd( d_3, c_3 );
				}
			else // AB = - A * B'
				{
				d_0 = _mm_sub_pd( d_0, c_0 );
				d_1 = _mm_sub_pd( d_1, c_1 );
				d_2 = _mm_sub_pd( d_2, c_2 );
				d_3 = _mm_sub_pd( d_3, c_3 );
				}

			if(td==0) // t( t(AB) + C )
				{
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );
				t_0 = d_2;
				d_2 = _mm_unpacklo_pd( d_2, d_3 );
				d_3 = _mm_unpackhi_pd( t_0, d_3 );

				goto store_n;
				}
			else // t(AB) + C
				{
				goto store_t;
				}
			}
		}

	// store (1 - 2) x (3 - 4)
	store_n:
	if(km>=2)
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_pd( &D[0+ldc*1], d_1 );
		_mm_store_pd( &D[0+ldc*2], d_2 );

		if(kn>=4)
			_mm_store_pd( &D[0+ldc*3], d_3 );
		}
	else
		{
		_mm_store_sd( &D[0+ldc*0], d_0 );
		_mm_store_sd( &D[0+ldc*1], d_1 );
		_mm_store_sd( &D[0+ldc*2], d_2 );

		if(kn>=4)
			_mm_store_sd( &D[0+ldc*3], d_3 );
		}
	return;

	store_t:
	if(kn>=4)
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_pd( &D[2+ldc*0], d_2 );

		if(km>=2)
			{
			_mm_store_pd( &D[0+ldc*1], d_1 );
			_mm_store_pd( &D[2+ldc*1], d_3 );
			}
		}
	else // kn==3
		{
		_mm_store_pd( &D[0+ldc*0], d_0 );
		_mm_store_sd( &D[2+ldc*0], d_2 );

		if(km>=2)
			{
			_mm_store_pd( &D[0+ldc*1], d_1 );
			_mm_store_sd( &D[2+ldc*1], d_3 );
			}
		}
	return;

	}



// 2x4 with data packed in 4
void kernel_dgemm_nt_2x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nt_2x4_vs_lib4(2, 4, kmax, A, B, C, D, alg, tc, td);

	}
	


// 2x2 with data packed in 4
void kernel_dgemm_nt_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;

	const int ldc = 4;

	int k;
	
	__m128d
		c_00_11, c_01_10,
		a_01,
		b_01, b_10;
	
	c_00_11 = _mm_setzero_pd();
	c_01_10 = _mm_setzero_pd();
	
	k = 0;
	for(; k<kmax-3; k+=4)
		{
		a_01 = _mm_load_pd(&A[0]);
		
		b_01 = _mm_load_pd(&B[0]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		
		a_01 = _mm_load_pd(&A[4]);
		
		b_01 = _mm_load_pd(&B[4]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		
		a_01 = _mm_load_pd(&A[8]);
		
		b_01 = _mm_load_pd(&B[8]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		
		
		a_01 = _mm_load_pd(&A[12]);
		
		b_01 = _mm_load_pd(&B[12]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );

		
		A += 16;
		B += 16;

		}
	
	for(; k<kmax; k++)
		{

		a_01 = _mm_load_pd(&A[0]);
		
		b_01 = _mm_load_pd(&B[0]);
		b_10 = _mm_shuffle_pd(b_01, b_01, 1);
	
		b_01 = _mm_mul_pd( a_01, b_01 );
		c_00_11 = _mm_add_pd( c_00_11, b_01 );

		b_10 = _mm_mul_pd( a_01, b_10 );
		c_01_10 = _mm_add_pd( c_01_10, b_10 );
		

		A += 4;
		B += 4;

		}

	__m128d
		t_0,
		c_0, c_1,
		d_0, d_1,
		c_00_10, c_01_11,
		c_00_01, c_10_11,
		d_00_11, d_01_10,
		d_00_10, d_01_11,
		d_00_01, d_10_11;

	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0)
			{
			d_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			d_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );

			goto store_n;
			}
		else
			{
			d_0 = _mm_unpacklo_pd( c_00_11, c_01_10 );
			d_1 = _mm_unpackhi_pd( c_01_10, c_00_11 );

			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // C
			{
			c_0 = _mm_shuffle_pd( c_00_11, c_01_10, 0x2 );
			c_1 = _mm_shuffle_pd( c_01_10, c_00_11, 0x2 );

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
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );

				goto store_t;
				}
			}
		else // t(C)
			{
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
				t_0 = d_0;
				d_0 = _mm_unpacklo_pd( d_0, d_1 );
				d_1 = _mm_unpackhi_pd( t_0, d_1 );

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



// 2x2 with data packed in 4
void kernel_dgemm_nt_2x2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nt_2x2_vs_lib4(2, 2, kmax, A, B, C, D, alg, tc, td);

	}


#endif
	
#if ! defined(BLASFEO)
// normal-normal, 4x4 with data packed in 4
void kernel_dgemm_nn_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
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
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
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
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
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
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
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


		A += 4;
		B += 1;

		}
		
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13,
		d_20, d_21, d_22, d_23,
		d_30, d_31, d_32, d_33;
	
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
	else // D = C +/- A * B'
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			d_20 = C[2+bs*0];
			d_30 = C[3+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_21 = C[2+bs*1];
			d_31 = C[3+bs*1];
			
			d_02 = C[0+bs*2];
			d_12 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_32 = C[3+bs*2];
			
			d_03 = C[0+bs*3];
			d_13 = C[1+bs*3];
			d_23 = C[2+bs*3];
			d_33 = C[3+bs*3];
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
			
			d_20 = C[0+bs*2];
			d_21 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_23 = C[3+bs*2];
			
			d_30 = C[0+bs*3];
			d_31 = C[1+bs*3];
			d_32 = C[2+bs*3];
			d_33 = C[3+bs*3];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;

			c_02 = d_02 + c_02;
			c_12 = d_12 + c_12;
			c_22 = d_22 + c_22;
			c_32 = d_32 + c_32;

			c_03 = d_03 + c_03;
			c_13 = d_13 + c_13;
			c_23 = d_23 + c_23;
			c_33 = d_33 + c_33;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;

			c_02 = d_02 - c_02;
			c_12 = d_12 - c_12;
			c_22 = d_22 - c_22;
			c_32 = d_32 - c_32;

			c_03 = d_03 - c_03;
			c_13 = d_13 - c_13;
			c_23 = d_23 - c_23;
			c_33 = d_33 - c_33;
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
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			D[2+bs*3] = c_23;
			D[3+bs*3] = c_33;
			}
		}
	else // km==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			D[2+bs*3] = c_23;
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
		
		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_12;
		D[3+bs*1] = c_13;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_23;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			D[2+bs*3] = c_32;
			D[3+bs*3] = c_33;
			}
		}
	else // kn==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		
		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_12;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;
		D[2+bs*2] = c_22;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			D[2+bs*3] = c_32;
			}
		}
	return;
	
	}



void kernel_dgemm_nn_4x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{

	kernel_dgemm_nn_4x4_vs_lib4(4, 4, kmax, A, B, sdb, alg, C, D, tc, td);
	
	}
#endif



// normal-normal, 4x2 with data packed in 4
void kernel_dgemm_nn_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
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

		c_01 += a_0 * b_1;
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

		c_01 += a_0 * b_1;
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

		c_01 += a_0 * b_1;
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

		c_01 += a_0 * b_1;
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

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 1;

		}
		
	double
		d_00, d_01,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
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
			d_20 = C[2+bs*0];
			d_30 = C[3+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_21 = C[2+bs*1];
			d_31 = C[3+bs*1];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];

			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];

			d_20 = C[0+bs*2];
			d_21 = C[1+bs*2];

			d_30 = C[0+bs*3];
			d_31 = C[1+bs*3];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;
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
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;
			}
		}
	else // km==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			}
		}
	return;

	store_t:
	if(kn>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;

		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			}
		}
	else // kn==1
		{
		D[0+bs*0] = c_00;

		D[0+bs*1] = c_10;

		D[0+bs*2] = c_20;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			}
		}
	return;

	}



void kernel_dgemm_nn_4x2_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, int tc, int td)
	{

	kernel_dgemm_nn_4x2_vs_lib4(4, 2, kmax, A, B, sdb, alg, C, D, tc, td);
	
	}



// normal-normal, 2x4 with data packed in 4
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



// normal-normal, 2x2 with data packed in 4
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



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_4_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		b_1 = - B[1];
		b_2 = - B[2];
		b_3 = - B[3];
		}
	else
		{
		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];
		b_3 = B[3];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = a_0 * b_1;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_1;
			c_3 = a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = a_0 * b_2;
			c_1 = a_1 * b_2;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			a_0 = A[0+bs*3];
			a_1 = A[1+bs*3];
			a_2 = A[2+bs*3];
			a_3 = A[3+bs*3];
			
			c_0 = a_0 * b_3;
			c_1 = a_1 * b_3;
			c_2 = a_2 * b_3;
			c_3 = a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			a_0 = A[0+bs*1];
			
			c_0 = a_0 * b_1;

			D[0+bs*1] = c_0;
		

			a_0 = A[0+bs*2];
			
			c_0 = a_0 * b_2;

			D[0+bs*2] = c_0;
		

			a_0 = A[0+bs*3];
			
			c_0 = a_0 * b_3;

			D[0+bs*3] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_1;
			c_3 = C[3+bs*1] + a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;
			c_1 = C[1+bs*2] + a_1 * b_2;
			c_2 = C[2+bs*2] + a_2 * b_2;
			c_3 = C[3+bs*2] + a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			a_0 = A[0+bs*3];
			a_1 = A[1+bs*3];
			a_2 = A[2+bs*3];
			a_3 = A[3+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_3;
			c_1 = C[1+bs*3] + a_1 * b_3;
			c_2 = C[2+bs*3] + a_2 * b_3;
			c_3 = C[3+bs*3] + a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			a_0 = A[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;

			D[0+bs*1] = c_0;
			

			a_0 = A[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;

			D[0+bs*2] = c_0;
			

			a_0 = A[0+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_3;

			D[0+bs*3] = c_0;

	
			A += 1;
			C += 1;
			D += 1;
			
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

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		b_1 = - B[1];
		b_2 = - B[2];
		}
	else
		{
		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = a_0 * b_1;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_1;
			c_3 = a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = a_0 * b_2;
			c_1 = a_1 * b_2;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			a_0 = A[0+bs*1];
			
			c_0 = a_0 * b_1;

			D[0+bs*1] = c_0;
		

			a_0 = A[0+bs*2];
			
			c_0 = a_0 * b_2;

			D[0+bs*2] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_1;
			c_3 = C[3+bs*1] + a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;
			c_1 = C[1+bs*2] + a_1 * b_2;
			c_2 = C[2+bs*2] + a_2 * b_2;
			c_3 = C[3+bs*2] + a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			a_0 = A[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;

			D[0+bs*1] = c_0;
			

			a_0 = A[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;

			D[0+bs*2] = c_0;
			

			A += 1;
			C += 1;
			D += 1;
			
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

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		b_1 = - B[1];
		}
	else
		{
		b_0 = B[0];
		b_1 = B[1];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = a_0 * b_1;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_1;
			c_3 = a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			a_0 = A[0+bs*1];
			
			c_0 = a_0 * b_1;

			D[0+bs*1] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_1;
			c_3 = C[3+bs*1] + a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			a_0 = A[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;

			D[0+bs*1] = c_0;
			

			A += 1;
			C += 1;
			D += 1;
			
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

	double
		a_0, a_1, a_2, a_3,
		b_0,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		}
	else
		{
		b_0 = B[0];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			A += 1;
			C += 1;
			D += 1;
			
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
	
	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_3_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2,
		b_0, b_1, b_2,
		c_0, c_1, c_2;
		
	if(alg==-1)
		{
		a_0 = - A[0];
		a_1 = - A[1];
		a_2 = - A[2];
		}
	else
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		}
		
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
		
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
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_2;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;
			c_1 = C[1+bs*2] + a_1 * b_1;
			c_2 = C[2+bs*2] + a_2 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;
			c_1 = C[1+bs*3] + a_1 * b_1;
			c_2 = C[2+bs*3] + a_2 * b_2;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
		
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

	double
		a_0, a_1,
		b_0, b_1,
		c_0, c_1;
		
	if(alg==-1)
		{
		a_0 = - A[0];
		a_1 = - A[1];
		}
	else
		{
		a_0 = A[0];
		a_1 = A[1];
		}
		
	if(alg==0)
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
		
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
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;
			c_1 = C[1+bs*1] + a_1 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;
			c_1 = C[1+bs*2] + a_1 * b_1;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;
			c_1 = C[1+bs*3] + a_1 * b_1;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
		
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



