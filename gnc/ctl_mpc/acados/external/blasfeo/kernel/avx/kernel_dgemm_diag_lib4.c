/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#include "../../include/blasfeo_d_kernel.h"



// B is the diagonal of a matrix, beta==0.0 case
void kernel_dgemm_diag_right_4_a0_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		alpha0,
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22, b_33,
		d_00, d_01, d_02, d_03;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_sd( alpha );
	
	b_00 = _mm256_broadcast_sd( &B[0] );
	b_00 = _mm256_mul_pd( b_00, alpha0 );
	b_11 = _mm256_broadcast_sd( &B[1] );
	b_11 = _mm256_mul_pd( b_11, alpha0 );
	b_22 = _mm256_broadcast_sd( &B[2] );
	b_22 = _mm256_mul_pd( b_22, alpha0 );
	b_33 = _mm256_broadcast_sd( &B[3] );
	b_33 = _mm256_mul_pd( b_33, alpha0 );
	
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



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22, b_33,
		c_00,
		d_00, d_01, d_02, d_03;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_sd( alpha );
	beta0  = _mm256_broadcast_sd( beta );
	
	b_00 = _mm256_broadcast_sd( &B[0] );
	b_00 = _mm256_mul_pd( b_00, alpha0 );
	b_11 = _mm256_broadcast_sd( &B[1] );
	b_11 = _mm256_mul_pd( b_11, alpha0 );
	b_22 = _mm256_broadcast_sd( &B[2] );
	b_22 = _mm256_mul_pd( b_22, alpha0 );
	b_33 = _mm256_broadcast_sd( &B[3] );
	b_33 = _mm256_mul_pd( b_33, alpha0 );
	
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
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );
		c_00 = _mm256_load_pd( &C[8] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_02 = _mm256_add_pd( c_00, d_02 );
		c_00 = _mm256_load_pd( &C[12] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_03 = _mm256_add_pd( c_00, d_03 );

		_mm256_store_pd( &D[0], d_00 );
		_mm256_store_pd( &D[4], d_01 );
		_mm256_store_pd( &D[8], d_02 );
		_mm256_store_pd( &D[12], d_03 );

		A += 4*sda;
		C += 4*sdc;
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

		c_00 = _mm256_load_pd( &C[0] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );
		c_00 = _mm256_load_pd( &C[8] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_02 = _mm256_add_pd( c_00, d_02 );
		c_00 = _mm256_load_pd( &C[12] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_03 = _mm256_add_pd( c_00, d_03 );

		_mm256_maskstore_pd( &D[0], mask_i, d_00 );
		_mm256_maskstore_pd( &D[4], mask_i, d_01 );
		_mm256_maskstore_pd( &D[8], mask_i, d_02 );
		_mm256_maskstore_pd( &D[12], mask_i, d_03 );

		}
	
	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_3_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22,
		c_00,
		d_00, d_01, d_02;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_sd( alpha );
	beta0  = _mm256_broadcast_sd( beta );
	
	b_00 = _mm256_broadcast_sd( &B[0] );
	b_00 = _mm256_mul_pd( b_00, alpha0 );
	b_11 = _mm256_broadcast_sd( &B[1] );
	b_11 = _mm256_mul_pd( b_11, alpha0 );
	b_22 = _mm256_broadcast_sd( &B[2] );
	b_22 = _mm256_mul_pd( b_22, alpha0 );
	
	for(k=0; k<kmax-3; k+=4)
		{

		a_00 = _mm256_load_pd( &A[0] );
		d_00 = _mm256_mul_pd( a_00, b_00 );
		a_00 = _mm256_load_pd( &A[4] );
		d_01 = _mm256_mul_pd( a_00, b_11 );
		a_00 = _mm256_load_pd( &A[8] );
		d_02 = _mm256_mul_pd( a_00, b_22 );

		c_00 = _mm256_load_pd( &C[0] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );
		c_00 = _mm256_load_pd( &C[8] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_02 = _mm256_add_pd( c_00, d_02 );

		_mm256_store_pd( &D[0], d_00 );
		_mm256_store_pd( &D[4], d_01 );
		_mm256_store_pd( &D[8], d_02 );

		A += 4*sda;
		C += 4*sdc;
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

		c_00 = _mm256_load_pd( &C[0] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );
		c_00 = _mm256_load_pd( &C[8] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_02 = _mm256_add_pd( c_00, d_02 );

		_mm256_maskstore_pd( &D[0], mask_i, d_00 );
		_mm256_maskstore_pd( &D[4], mask_i, d_01 );
		_mm256_maskstore_pd( &D[8], mask_i, d_02 );

		}
	
	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_2_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00, b_11,
		c_00,
		d_00, d_01;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_sd( alpha );
	beta0  = _mm256_broadcast_sd( beta );
	
	b_00 = _mm256_broadcast_sd( &B[0] );
	b_00 = _mm256_mul_pd( b_00, alpha0 );
	b_11 = _mm256_broadcast_sd( &B[1] );
	b_11 = _mm256_mul_pd( b_11, alpha0 );
	
	for(k=0; k<kmax-3; k+=4)
		{

		a_00 = _mm256_load_pd( &A[0] );
		d_00 = _mm256_mul_pd( a_00, b_00 );
		a_00 = _mm256_load_pd( &A[4] );
		d_01 = _mm256_mul_pd( a_00, b_11 );

		c_00 = _mm256_load_pd( &C[0] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );

		_mm256_store_pd( &D[0], d_00 );
		_mm256_store_pd( &D[4], d_01 );

		A += 4*sda;
		C += 4*sdc;
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

		c_00 = _mm256_load_pd( &C[0] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );

		_mm256_maskstore_pd( &D[0], mask_i, d_00 );
		_mm256_maskstore_pd( &D[4], mask_i, d_01 );

		}

	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_1_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00,
		c_00,
		d_00;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_sd( alpha );
	beta0  = _mm256_broadcast_sd( beta );
	
	b_00 = _mm256_broadcast_sd( &B[0] );
	
	for(k=0; k<kmax-3; k+=4)
		{

		a_00 = _mm256_load_pd( &A[0] );
		d_00 = _mm256_mul_pd( a_00, b_00 );

		c_00 = _mm256_load_pd( &C[0] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );

		_mm256_store_pd( &D[0], d_00 );

		A += 4*sda;
		C += 4*sdc;
		D += 4*sdd;

		}
	if(k<kmax)
		{

		const double mask_f[] = {0.5, 1.5, 2.5, 3.5};
		double m_f = kmax-k;

		mask_i = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( mask_f ), _mm256_broadcast_sd( &m_f ) ) );

		a_00 = _mm256_load_pd( &A[0] );
		d_00 = _mm256_mul_pd( a_00, b_00 );

		c_00 = _mm256_load_pd( &C[0] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );

		_mm256_maskstore_pd( &D[0], mask_i, d_00 );

		}
	
	}



// A is the diagonal of a matrix, beta=0.0 case
void kernel_dgemm_diag_left_4_a0_lib4(int kmax, double *alpha, double *A, double *B, double *D)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		alpha0,
		sign,
		a_00,
		b_00,
		d_00, d_01, d_02, d_03;
	
	alpha0 = _mm256_broadcast_sd( alpha );
	
	a_00 = _mm256_load_pd( &A[0] );
	a_00 = _mm256_mul_pd( a_00, alpha0 );
	
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
		d_00 = _mm256_mul_pd( a_00, b_00 );

		_mm256_store_pd( &D[0], d_00 );

		B += 4;
		D += 4;
		
		}

	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_4_lib4(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256d
		alpha0, beta0,
		sign,
		a_00,
		b_00,
		c_00,
		d_00, d_01, d_02, d_03;
	
	alpha0 = _mm256_broadcast_sd( alpha );
	beta0  = _mm256_broadcast_sd( beta );
	
	a_00 = _mm256_load_pd( &A[0] );
	a_00 = _mm256_mul_pd( a_00, alpha0 );
	
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
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );
		c_00 = _mm256_load_pd( &C[8] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_02 = _mm256_add_pd( c_00, d_02 );
		c_00 = _mm256_load_pd( &C[12] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
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
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );

		_mm256_store_pd( &D[0], d_00 );

		B += 4;
		C += 4;
		D += 4;
		
		}

	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_3_lib4(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256i
		mask;

	__m256d
		alpha0, beta0,
		sign,
		a_00,
		b_00,
		c_00,
		d_00, d_01, d_02, d_03;
	
	mask = _mm256_set_epi64x( 1, -1, -1, -1 );
		
	alpha0 = _mm256_broadcast_sd( alpha );
	beta0  = _mm256_broadcast_sd( beta );
	
	a_00 = _mm256_load_pd( &A[0] );
	a_00 = _mm256_mul_pd( a_00, alpha0 );
	
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
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );
		c_00 = _mm256_load_pd( &C[4] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_01 = _mm256_add_pd( c_00, d_01 );
		c_00 = _mm256_load_pd( &C[8] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_02 = _mm256_add_pd( c_00, d_02 );
		c_00 = _mm256_load_pd( &C[12] );
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_03 = _mm256_add_pd( c_00, d_03 );

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
		c_00 = _mm256_mul_pd( c_00, beta0 );
		d_00 = _mm256_add_pd( c_00, d_00 );

		_mm256_maskstore_pd( &D[0], mask, d_00 );

		B += 4;
		C += 4;
		D += 4;
		
		}

	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_2_lib4(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m128d
		alpha0, beta0,
		sign,
		a_00,
		b_00,
		c_00,
		d_00, d_01, d_02, d_03;
		
	alpha0 = _mm_loaddup_pd( alpha );
	beta0  = _mm_loaddup_pd( beta );
	
	a_00 = _mm_load_pd( &A[0] );
	a_00 = _mm_mul_pd( a_00, alpha0 );
	
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
		c_00 = _mm_mul_pd( c_00, beta0 );
		d_00 = _mm_add_pd( c_00, d_00 );
		c_00 = _mm_load_pd( &C[4] );
		c_00 = _mm_mul_pd( c_00, beta0 );
		d_01 = _mm_add_pd( c_00, d_01 );
		c_00 = _mm_load_pd( &C[8] );
		c_00 = _mm_mul_pd( c_00, beta0 );
		d_02 = _mm_add_pd( c_00, d_02 );
		c_00 = _mm_load_pd( &C[12] );
		c_00 = _mm_mul_pd( c_00, beta0 );
		d_03 = _mm_add_pd( c_00, d_03 );

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
		c_00 = _mm_mul_pd( c_00, beta0 );
		d_00 = _mm_add_pd( c_00, d_00 );

		_mm_store_pd( &D[0], d_00 );

		B += 4;
		C += 4;
		D += 4;
		
		}

	
	}


// A is the diagonal of a matrix
void kernel_dgemm_diag_left_1_lib4(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		alpha0, beta0,
		a_0,
		b_0,
		c_0;
	
	alpha0 = alpha[0];
	beta0  = beta[0];
		
	a_0 = A[0] * alpha0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		b_0 = B[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
		

		b_0 = B[0+bs*1];
		
		c_0 = beta0 * C[0+bs*1] + a_0 * b_0;

		D[0+bs*1] = c_0;
		

		b_0 = B[0+bs*2];
		
		c_0 = beta0 * C[0+bs*2] + a_0 * b_0;

		D[0+bs*2] = c_0;
		

		b_0 = B[0+bs*3];
		
		c_0 = beta0 * C[0+bs*3] + a_0 * b_0;

		D[0+bs*3] = c_0;

		B += 16;
		C += 16;
		D += 16;
		
		}
	for(; k<kmax; k++)
		{
		
		b_0 = B[0+bs*0];
		
		c_0 = beta0 * C[0+bs*0] + a_0 * b_0;

		D[0+bs*0] = c_0;
	
		B += 4;
		C += 4;
		D += 4;
		
		}
		
	}



