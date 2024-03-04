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
#include "../../include/blasfeo_s_kernel.h"


// B is the diagonal of a matrix, beta==0.0 case
void kernel_sgemm_diag_right_4_a0_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 8;

	int k;

	__m256
		alpha0,
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22, b_33,
		d_00, d_01, d_02, d_03;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_ss( alpha );
	
	b_00 = _mm256_broadcast_ss( &B[0] );
	b_00 = _mm256_mul_ps( b_00, alpha0 );
	b_11 = _mm256_broadcast_ss( &B[1] );
	b_11 = _mm256_mul_ps( b_11, alpha0 );
	b_22 = _mm256_broadcast_ss( &B[2] );
	b_22 = _mm256_mul_ps( b_22, alpha0 );
	b_33 = _mm256_broadcast_ss( &B[3] );
	b_33 = _mm256_mul_ps( b_33, alpha0 );
	
	for(k=0; k<kmax-7; k+=8)
		{

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );
		a_00 = _mm256_load_ps( &A[16] );
		d_02 = _mm256_mul_ps( a_00, b_22 );
		a_00 = _mm256_load_ps( &A[24] );
		d_03 = _mm256_mul_ps( a_00, b_33 );

		_mm256_store_ps( &D[0], d_00 );
		_mm256_store_ps( &D[8], d_01 );
		_mm256_store_ps( &D[16], d_02 );
		_mm256_store_ps( &D[24], d_03 );

		A += 8*sda;
		D += 8*sdd;

		}
	if(k<kmax)
		{

		const float mask_f[] = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5};
		float m_f = kmax-k;

		mask_i = _mm256_castps_si256( _mm256_sub_ps( _mm256_loadu_ps( mask_f ), _mm256_broadcast_ss( &m_f ) ) );

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );
		a_00 = _mm256_load_ps( &A[16] );
		d_02 = _mm256_mul_ps( a_00, b_22 );
		a_00 = _mm256_load_ps( &A[24] );
		d_03 = _mm256_mul_ps( a_00, b_33 );

		_mm256_maskstore_ps( &D[0], mask_i, d_00 );
		_mm256_maskstore_ps( &D[8], mask_i, d_01 );
		_mm256_maskstore_ps( &D[16], mask_i, d_02 );
		_mm256_maskstore_ps( &D[24], mask_i, d_03 );

		}
	
	}



// B is the diagonal of a matrix
void kernel_sgemm_diag_right_4_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 8;

	int k;

	__m256
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22, b_33,
		c_00,
		d_00, d_01, d_02, d_03;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_ss( alpha );
	beta0  = _mm256_broadcast_ss( beta );
	
	b_00 = _mm256_broadcast_ss( &B[0] );
	b_00 = _mm256_mul_ps( b_00, alpha0 );
	b_11 = _mm256_broadcast_ss( &B[1] );
	b_11 = _mm256_mul_ps( b_11, alpha0 );
	b_22 = _mm256_broadcast_ss( &B[2] );
	b_22 = _mm256_mul_ps( b_22, alpha0 );
	b_33 = _mm256_broadcast_ss( &B[3] );
	b_33 = _mm256_mul_ps( b_33, alpha0 );
	
	for(k=0; k<kmax-7; k+=8)
		{

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );
		a_00 = _mm256_load_ps( &A[16] );
		d_02 = _mm256_mul_ps( a_00, b_22 );
		a_00 = _mm256_load_ps( &A[24] );
		d_03 = _mm256_mul_ps( a_00, b_33 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );
		c_00 = _mm256_load_ps( &C[8] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_01 = _mm256_add_ps( c_00, d_01 );
		c_00 = _mm256_load_ps( &C[16] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_02 = _mm256_add_ps( c_00, d_02 );
		c_00 = _mm256_load_ps( &C[24] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_03 = _mm256_add_ps( c_00, d_03 );

		_mm256_store_ps( &D[0], d_00 );
		_mm256_store_ps( &D[8], d_01 );
		_mm256_store_ps( &D[16], d_02 );
		_mm256_store_ps( &D[24], d_03 );

		A += 8*sda;
		C += 8*sdc;
		D += 8*sdd;

		}
	if(k<kmax)
		{

		const float mask_f[] = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5};
		float m_f = kmax-k;

		mask_i = _mm256_castps_si256( _mm256_sub_ps( _mm256_loadu_ps( mask_f ), _mm256_broadcast_ss( &m_f ) ) );

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );
		a_00 = _mm256_load_ps( &A[16] );
		d_02 = _mm256_mul_ps( a_00, b_22 );
		a_00 = _mm256_load_ps( &A[24] );
		d_03 = _mm256_mul_ps( a_00, b_33 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );
		c_00 = _mm256_load_ps( &C[8] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_01 = _mm256_add_ps( c_00, d_01 );
		c_00 = _mm256_load_ps( &C[16] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_02 = _mm256_add_ps( c_00, d_02 );
		c_00 = _mm256_load_ps( &C[24] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_03 = _mm256_add_ps( c_00, d_03 );

		_mm256_maskstore_ps( &D[0], mask_i, d_00 );
		_mm256_maskstore_ps( &D[8], mask_i, d_01 );
		_mm256_maskstore_ps( &D[16], mask_i, d_02 );
		_mm256_maskstore_ps( &D[24], mask_i, d_03 );

		}
	
	}



// B is the diagonal of a matrix
void kernel_sgemm_diag_right_3_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 8;

	int k;

	__m256
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00, b_11, b_22,
		c_00,
		d_00, d_01, d_02;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_ss( alpha );
	beta0  = _mm256_broadcast_ss( beta );
	
	b_00 = _mm256_broadcast_ss( &B[0] );
	b_00 = _mm256_mul_ps( b_00, alpha0 );
	b_11 = _mm256_broadcast_ss( &B[1] );
	b_11 = _mm256_mul_ps( b_11, alpha0 );
	b_22 = _mm256_broadcast_ss( &B[2] );
	b_22 = _mm256_mul_ps( b_22, alpha0 );
	
	for(k=0; k<kmax-7; k+=8)
		{

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );
		a_00 = _mm256_load_ps( &A[16] );
		d_02 = _mm256_mul_ps( a_00, b_22 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );
		c_00 = _mm256_load_ps( &C[8] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_01 = _mm256_add_ps( c_00, d_01 );
		c_00 = _mm256_load_ps( &C[16] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_02 = _mm256_add_ps( c_00, d_02 );

		_mm256_store_ps( &D[0], d_00 );
		_mm256_store_ps( &D[8], d_01 );
		_mm256_store_ps( &D[16], d_02 );

		A += 8*sda;
		C += 8*sdc;
		D += 8*sdd;

		}
	if(k<kmax)
		{

		const float mask_f[] = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5};
		float m_f = kmax-k;

		mask_i = _mm256_castps_si256( _mm256_sub_ps( _mm256_loadu_ps( mask_f ), _mm256_broadcast_ss( &m_f ) ) );

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );
		a_00 = _mm256_load_ps( &A[16] );
		d_02 = _mm256_mul_ps( a_00, b_22 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );
		c_00 = _mm256_load_ps( &C[8] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_01 = _mm256_add_ps( c_00, d_01 );
		c_00 = _mm256_load_ps( &C[16] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_02 = _mm256_add_ps( c_00, d_02 );

		_mm256_maskstore_ps( &D[0], mask_i, d_00 );
		_mm256_maskstore_ps( &D[8], mask_i, d_01 );
		_mm256_maskstore_ps( &D[16], mask_i, d_02 );

		}
	
	}



// B is the diagonal of a matrix
void kernel_sgemm_diag_right_2_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00, b_11,
		c_00,
		d_00, d_01;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_ss( alpha );
	beta0  = _mm256_broadcast_ss( beta );
	
	b_00 = _mm256_broadcast_ss( &B[0] );
	b_00 = _mm256_mul_ps( b_00, alpha0 );
	b_11 = _mm256_broadcast_ss( &B[1] );
	b_11 = _mm256_mul_ps( b_11, alpha0 );
	
	for(k=0; k<kmax-7; k+=8)
		{

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );
		c_00 = _mm256_load_ps( &C[8] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_01 = _mm256_add_ps( c_00, d_01 );

		_mm256_store_ps( &D[0], d_00 );
		_mm256_store_ps( &D[8], d_01 );

		A += 8*sda;
		C += 8*sdc;
		D += 8*sdd;

		}
	if(k<kmax)
		{

		const float mask_f[] = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5};
		float m_f = kmax-k;

		mask_i = _mm256_castps_si256( _mm256_sub_ps( _mm256_loadu_ps( mask_f ), _mm256_broadcast_ss( &m_f ) ) );

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );
		a_00 = _mm256_load_ps( &A[8] );
		d_01 = _mm256_mul_ps( a_00, b_11 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );
		c_00 = _mm256_load_ps( &C[8] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_01 = _mm256_add_ps( c_00, d_01 );

		_mm256_maskstore_ps( &D[0], mask_i, d_00 );
		_mm256_maskstore_ps( &D[8], mask_i, d_01 );

		}
	
	}



// B is the diagonal of a matrix
void kernel_sgemm_diag_right_1_lib4(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	__m256
		alpha0, beta0,
		mask_f,
		sign,
		a_00,
		b_00,
		c_00,
		d_00;
	
	__m256i
		mask_i;
	
	alpha0 = _mm256_broadcast_ss( alpha );
	beta0  = _mm256_broadcast_ss( beta );
	
	b_00 = _mm256_broadcast_ss( &B[0] );
	b_00 = _mm256_mul_ps( b_00, alpha0 );
	
	for(k=0; k<kmax-7; k+=8)
		{

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );

		_mm256_store_ps( &D[0], d_00 );

		A += 8*sda;
		C += 8*sdc;
		D += 8*sdd;

		}
	if(k<kmax)
		{

		const float mask_f[] = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5};
		float m_f = kmax-k;

		mask_i = _mm256_castps_si256( _mm256_sub_ps( _mm256_loadu_ps( mask_f ), _mm256_broadcast_ss( &m_f ) ) );

		a_00 = _mm256_load_ps( &A[0] );
		d_00 = _mm256_mul_ps( a_00, b_00 );

		c_00 = _mm256_load_ps( &C[0] );
		c_00 = _mm256_mul_ps( c_00, beta0 );
		d_00 = _mm256_add_ps( c_00, d_00 );

		_mm256_maskstore_ps( &D[0], mask_i, d_00 );

		}
	
	}




