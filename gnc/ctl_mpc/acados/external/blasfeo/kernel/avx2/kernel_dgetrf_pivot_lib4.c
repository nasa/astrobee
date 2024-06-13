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

#include "../../include/blasfeo_common.h"
#include "../../include/blasfeo_d_aux.h"
#include "../../include/blasfeo_d_kernel.h"



// swap two rows
void kernel_drowsw_lib4(int kmax, double *pA, double *pC)
	{

	const int bs = 4;

	int ii;
	double tmp;

	for(ii=0; ii<kmax-3; ii+=4)
		{
		tmp = pA[0+bs*0];
		pA[0+bs*0] = pC[0+bs*0];
		pC[0+bs*0] = tmp;
		tmp = pA[0+bs*1];
		pA[0+bs*1] = pC[0+bs*1];
		pC[0+bs*1] = tmp;
		tmp = pA[0+bs*2];
		pA[0+bs*2] = pC[0+bs*2];
		pC[0+bs*2] = tmp;
		tmp = pA[0+bs*3];
		pA[0+bs*3] = pC[0+bs*3];
		pC[0+bs*3] = tmp;
		pA += 4*bs;
		pC += 4*bs;
		}
	for( ; ii<kmax; ii++)
		{
		tmp = pA[0+bs*0];
		pA[0+bs*0] = pC[0+bs*0];
		pC[0+bs*0] = tmp;
		pA += 1*bs;
		pC += 1*bs;
		}

	}



// C numering (starting from zero) in the ipiv
void kernel_dgetrf_pivot_12_lib4(int m, double *pA, int sda, double *inv_diag_A, int* ipiv)
	{

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	__m128d
		max0, max1, msk0, imx0, imx1,
		inv;
	
		
	__m256d
		lft, msk,
		sgn, vna, max, imx, idx,
		ones,
		tmp,
		a_0, a_i,
		u_1, u_2, u_3, u_4, u_5, u_6, u_7, u_8, u_9, u_10, u_11,
		b_0, b_1, b_2,
		scl,
		c_0,
		d_0;
	
	double
		dlft;

	sgn = _mm256_set_pd( -0.0, -0.0, -0.0, -0.0 );
	vna = _mm256_set_pd( 4.0, 4.0, 4.0, 4.0 );
	lft  = _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	double
		tmp0;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	int B_pref = bs*sda;
	

	// first column

	// find pivot
	pB = &pA[0+bs*0];
	idx = lft; // _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	k = 0;
	for( ; k<m-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for( ; k<m-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0] );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<m)
		{
		dlft = m-k;
		msk = _mm256_broadcast_sd( &dlft );
		a_0 = _mm256_load_pd( &pB[0] );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_blendv_pd( a_0, sgn, msk );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// pivot & compute scaling
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			kernel_drowsw_lib4(12, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		inv = _mm_loaddup_pd( &pA[0+bs*0] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[0], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[0] = 0.0;
		}


	// second column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 2.2, 1.2, 0.2, -0.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_1 = _mm256_broadcast_sd( &pA[0+bs*1] );
	u_2 = _mm256_broadcast_sd( &pA[0+bs*2] );
	u_3 = _mm256_broadcast_sd( &pA[0+bs*3] );
	u_4 = _mm256_broadcast_sd( &pA[0+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[0+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[0+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[0+bs*7] );
	u_8 = _mm256_broadcast_sd( &pA[0+bs*8] );
	u_9 = _mm256_broadcast_sd( &pA[0+bs*9] );
	u_10 = _mm256_broadcast_sd( &pA[0+bs*10] );
	u_11 = _mm256_broadcast_sd( &pA[0+bs*11] );
	// col 0
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x1 );
	_mm256_store_pd( &pA[0+bs*0], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x1 );
	// col 1
	c_0 = _mm256_load_pd( &pA[0+bs*1] );
	c_0 = _mm256_fnmadd_pd( a_0, u_1, c_0 );
	_mm256_store_pd( &pA[0+bs*1], c_0 );
	// col 2
	a_i = _mm256_load_pd( &pA[0+bs*2] );
	a_i = _mm256_fnmadd_pd( a_0, u_2, a_i );
	_mm256_store_pd( &pA[0+bs*2], a_i );
	// col 3
	a_i = _mm256_load_pd( &pA[0+bs*3] );
	a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
	_mm256_store_pd( &pA[0+bs*3], a_i );
	// col 4
	a_i = _mm256_load_pd( &pA[0+bs*4] );
	a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
	_mm256_store_pd( &pA[0+bs*4], a_i );
	// col 5
	a_i = _mm256_load_pd( &pA[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pA[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pA[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pA[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pA[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pA[0+bs*7], a_i );
	// col 8
	a_i = _mm256_load_pd( &pA[0+bs*8] );
	a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
	_mm256_store_pd( &pA[0+bs*8], a_i );
	// col 9
	a_i = _mm256_load_pd( &pA[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pA[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pA[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pA[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pA[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pA[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x1 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-3; k+=4)
		{
		// col 0
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		// col 1
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		c_0 = _mm256_fnmadd_pd( a_0, u_1, c_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		// col 2
		a_i = _mm256_load_pd( &pB[0+bs*2] );
		a_i = _mm256_fnmadd_pd( a_0, u_2, a_i );
		_mm256_store_pd( &pB[0+bs*2], a_i );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 0
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 1
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		c_0 = _mm256_fnmadd_pd( a_0, u_1, c_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		// col 2
		a_i = _mm256_load_pd( &pB[0+bs*2] );
		a_i = _mm256_fnmadd_pd( a_0, u_2, a_i );
		_mm256_store_pd( &pB[0+bs*2], a_i );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[1] = idamax+1;
	if(tmp0!=0)
		{
		if(ipiv[1]!=1)
			kernel_drowsw_lib4(12, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

		inv = _mm_loaddup_pd( &pA[1+bs*1] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[1], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[1] = 0.0;
		}



	// third column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 1.2, 0.2, -0.8, -1.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_2 = _mm256_broadcast_sd( &pA[1+bs*2] );
	u_3 = _mm256_broadcast_sd( &pA[1+bs*3] );
	u_4 = _mm256_broadcast_sd( &pA[1+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[1+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[1+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[1+bs*7] );
	u_8 = _mm256_broadcast_sd( &pA[1+bs*8] );
	u_9 = _mm256_broadcast_sd( &pA[1+bs*9] );
	u_10 = _mm256_broadcast_sd( &pA[1+bs*10] );
	u_11 = _mm256_broadcast_sd( &pA[1+bs*11] );
	// col 1
	a_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x3 );
	_mm256_store_pd( &pA[0+bs*1], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x3 );
	// col 2
	c_0 = _mm256_load_pd( &pA[0+bs*2] );
	c_0 = _mm256_fnmadd_pd( a_0, u_2, c_0 );
	_mm256_store_pd( &pA[0+bs*2], c_0 );
	// col 3
	a_i = _mm256_load_pd( &pA[0+bs*3] );
	a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
	_mm256_store_pd( &pA[0+bs*3], a_i );
	// col 4
	a_i = _mm256_load_pd( &pA[0+bs*4] );
	a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
	_mm256_store_pd( &pA[0+bs*4], a_i );
	// col 5
	a_i = _mm256_load_pd( &pA[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pA[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pA[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pA[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pA[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pA[0+bs*7], a_i );
	// col 8
	a_i = _mm256_load_pd( &pA[0+bs*8] );
	a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
	_mm256_store_pd( &pA[0+bs*8], a_i );
	// col 9
	a_i = _mm256_load_pd( &pA[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pA[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pA[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pA[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pA[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pA[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x3 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-3; k+=4)
		{
		// col 1
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		// col 2
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		c_0 = _mm256_fnmadd_pd( a_0, u_2, c_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 1
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 2
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		c_0 = _mm256_fnmadd_pd( a_0, u_2, c_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[2] = idamax+2;
	if(tmp0!=0)
		{
		if(ipiv[2]!=2)
			kernel_drowsw_lib4(12, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

		inv = _mm_loaddup_pd( &pA[2+bs*2] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[2], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[2] = 0.0;
		}



	// fourth column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 0.2, -0.8, -1.8, -2.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_3 = _mm256_broadcast_sd( &pA[2+bs*3] );
	u_4 = _mm256_broadcast_sd( &pA[2+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[2+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[2+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[2+bs*7] );
	u_8 = _mm256_broadcast_sd( &pA[2+bs*8] );
	u_9 = _mm256_broadcast_sd( &pA[2+bs*9] );
	u_10 = _mm256_broadcast_sd( &pA[2+bs*10] );
	u_11 = _mm256_broadcast_sd( &pA[2+bs*11] );
	// col 2
	a_0 = _mm256_load_pd( &pA[0+bs*2] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x7 );
	_mm256_store_pd( &pA[0+bs*2], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x7 );
	// col 3
	c_0 = _mm256_load_pd( &pA[0+bs*3] );
	c_0 = _mm256_fnmadd_pd( a_0, u_3, c_0 );
	_mm256_store_pd( &pA[0+bs*3], c_0 );
	// col 4
	a_i = _mm256_load_pd( &pA[0+bs*4] );
	a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
	_mm256_store_pd( &pA[0+bs*4], a_i );
	// col 5
	a_i = _mm256_load_pd( &pA[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pA[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pA[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pA[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pA[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pA[0+bs*7], a_i );
	// col 8
	a_i = _mm256_load_pd( &pA[0+bs*8] );
	a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
	_mm256_store_pd( &pA[0+bs*8], a_i );
	// col 9
	a_i = _mm256_load_pd( &pA[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pA[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pA[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pA[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pA[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pA[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x7 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-3; k+=4)
		{
		// col 2
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		// col 3
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_fnmadd_pd( a_0, u_3, c_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 2
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 3
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_fnmadd_pd( a_0, u_3, c_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[3] = idamax+3;
	if(tmp0!=0)
		{
		if(ipiv[3]!=3)
			kernel_drowsw_lib4(12, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

		inv = _mm_loaddup_pd( &pA[3+bs*3] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[3], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[3] = 0.0;
		}
	

	// fifth column

	// scale & correct & find pivot
	// prep
	idx = lft; // _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_4 = _mm256_broadcast_sd( &pA[3+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[3+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[3+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[3+bs*7] );
	u_8 = _mm256_broadcast_sd( &pA[3+bs*8] );
	u_9 = _mm256_broadcast_sd( &pA[3+bs*9] );
	u_10 = _mm256_broadcast_sd( &pA[3+bs*10] );
	u_11 = _mm256_broadcast_sd( &pA[3+bs*11] );
	pB = pA + B_pref; // XXX
	// col 3
	a_0 = _mm256_load_pd( &pB[0+bs*3] );
	a_0 = _mm256_mul_pd( a_0, scl );
//	tmp = _mm256_mul_pd( a_0, scl );
//	a_0 = _mm256_blend_pd( tmp, a_0, 0xf );
	_mm256_store_pd( &pB[0+bs*3], a_0 );
//	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0xf );
	// col 4
	c_0 = _mm256_load_pd( &pB[0+bs*4] );
	c_0 = _mm256_fnmadd_pd( a_0, u_4, c_0 );
	_mm256_store_pd( &pB[0+bs*4], c_0 );
	// col 5
	a_i = _mm256_load_pd( &pB[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pB[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pB[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pB[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pB[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pB[0+bs*7], a_i );
	// col 8
	a_i = _mm256_load_pd( &pB[0+bs*8] );
	a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
	_mm256_store_pd( &pB[0+bs*8], a_i );
	// col 9
	a_i = _mm256_load_pd( &pB[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pB[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pB[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pB[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pB[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pB[0+bs*11], a_i );
	// search pivot
//	c_0 = _mm256_blend_pd( c_0, sgn, 0xf );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 3
		a_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*3], a_0 );
		// col 4
		c_0 = _mm256_load_pd( &pB[0+bs*4] );
		c_0 = _mm256_fnmadd_pd( a_0, u_4, c_0 );
		_mm256_store_pd( &pB[0+bs*4], c_0 );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 3
		a_0 = _mm256_load_pd( &pB[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*3], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 4
		c_0 = _mm256_load_pd( &pB[0+bs*4] );
		c_0 = _mm256_fnmadd_pd( a_0, u_4, c_0 );
		_mm256_store_pd( &pB[0+bs*4], c_0 );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[4] = idamax+4;
	if(tmp0!=0)
		{
		if(ipiv[4]!=4)
			kernel_drowsw_lib4(12, pA+B_pref+0, pA+ipiv[4]/bs*bs*sda+ipiv[4]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+0+bs*4] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[4], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[4] = 0.0;
		}
	

	// sixth column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 2.2, 1.2, 0.2, -0.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + B_pref; // XXX
	u_5 = _mm256_broadcast_sd( &pB[0+bs*5] );
	u_6 = _mm256_broadcast_sd( &pB[0+bs*6] );
	u_7 = _mm256_broadcast_sd( &pB[0+bs*7] );
	u_8 = _mm256_broadcast_sd( &pB[0+bs*8] );
	u_9 = _mm256_broadcast_sd( &pB[0+bs*9] );
	u_10 = _mm256_broadcast_sd( &pB[0+bs*10] );
	u_11 = _mm256_broadcast_sd( &pB[0+bs*11] );
	// col 4
	a_0 = _mm256_load_pd( &pB[0+bs*4] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x1 );
	_mm256_store_pd( &pB[0+bs*4], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x1 );
	// col 5
	c_0 = _mm256_load_pd( &pB[0+bs*5] );
	c_0 = _mm256_fnmadd_pd( a_0, u_5, c_0 );
	_mm256_store_pd( &pB[0+bs*5], c_0 );
	// col 6
	a_i = _mm256_load_pd( &pB[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pB[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pB[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pB[0+bs*7], a_i );
	// col 8
	a_i = _mm256_load_pd( &pB[0+bs*8] );
	a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
	_mm256_store_pd( &pB[0+bs*8], a_i );
	// col 9
	a_i = _mm256_load_pd( &pB[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pB[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pB[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pB[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pB[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pB[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x1 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 4
		a_0 = _mm256_load_pd( &pB[0+bs*4] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*4], a_0 );
		// col 5
		c_0 = _mm256_load_pd( &pB[0+bs*5] );
		c_0 = _mm256_fnmadd_pd( a_0, u_5, c_0 );
		_mm256_store_pd( &pB[0+bs*5], c_0 );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 4
		a_0 = _mm256_load_pd( &pB[0+bs*4] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*4], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 5
		c_0 = _mm256_load_pd( &pB[0+bs*5] );
		c_0 = _mm256_fnmadd_pd( a_0, u_5, c_0 );
		_mm256_store_pd( &pB[0+bs*5], c_0 );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[5] = idamax+5;
	if(tmp0!=0)
		{
		if(ipiv[5]!=5)
			kernel_drowsw_lib4(12, pA+B_pref+1, pA+ipiv[5]/bs*bs*sda+ipiv[5]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+1+bs*5] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[5], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[5] = 0.0;
		}
	

	// seventh column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 1.2, 0.2, -0.8, -1.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + B_pref; // XXX
	u_6 = _mm256_broadcast_sd( &pB[1+bs*6] );
	u_7 = _mm256_broadcast_sd( &pB[1+bs*7] );
	u_8 = _mm256_broadcast_sd( &pB[1+bs*8] );
	u_9 = _mm256_broadcast_sd( &pB[1+bs*9] );
	u_10 = _mm256_broadcast_sd( &pB[1+bs*10] );
	u_11 = _mm256_broadcast_sd( &pB[1+bs*11] );
	// col 5
	a_0 = _mm256_load_pd( &pB[0+bs*5] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x3 );
	_mm256_store_pd( &pB[0+bs*5], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x3 );
	// col 6
	c_0 = _mm256_load_pd( &pB[0+bs*6] );
	c_0 = _mm256_fnmadd_pd( a_0, u_6, c_0 );
	_mm256_store_pd( &pB[0+bs*6], c_0 );
	// col 7
	a_i = _mm256_load_pd( &pB[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pB[0+bs*7], a_i );
	// col 8
	a_i = _mm256_load_pd( &pB[0+bs*8] );
	a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
	_mm256_store_pd( &pB[0+bs*8], a_i );
	// col 9
	a_i = _mm256_load_pd( &pB[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pB[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pB[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pB[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pB[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pB[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x3 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 5
		a_0 = _mm256_load_pd( &pB[0+bs*5] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*5], a_0 );
		// col 6
		c_0 = _mm256_load_pd( &pB[0+bs*6] );
		c_0 = _mm256_fnmadd_pd( a_0, u_6, c_0 );
		_mm256_store_pd( &pB[0+bs*6], c_0 );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 5
		a_0 = _mm256_load_pd( &pB[0+bs*5] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*5], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 6
		c_0 = _mm256_load_pd( &pB[0+bs*6] );
		c_0 = _mm256_fnmadd_pd( a_0, u_6, c_0 );
		_mm256_store_pd( &pB[0+bs*6], c_0 );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[6] = idamax+6;
	if(tmp0!=0)
		{
		if(ipiv[6]!=6)
			kernel_drowsw_lib4(12, pA+B_pref+2, pA+ipiv[6]/bs*bs*sda+ipiv[6]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+2+bs*6] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[6], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[6] = 0.0;
		}

	// eight column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 0.2, -0.8, -1.8, -2.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + B_pref; // XXX
	u_7 = _mm256_broadcast_sd( &pB[2+bs*7] );
	u_8 = _mm256_broadcast_sd( &pB[2+bs*8] );
	u_9 = _mm256_broadcast_sd( &pB[2+bs*9] );
	u_10 = _mm256_broadcast_sd( &pB[2+bs*10] );
	u_11 = _mm256_broadcast_sd( &pB[2+bs*11] );
	// col 6
	a_0 = _mm256_load_pd( &pB[0+bs*6] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x7 );
	_mm256_store_pd( &pB[0+bs*6], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x7 );
	// col 7
	c_0 = _mm256_load_pd( &pB[0+bs*7] );
	c_0 = _mm256_fnmadd_pd( a_0, u_7, c_0 );
	_mm256_store_pd( &pB[0+bs*7], c_0 );
	// col 8
	a_i = _mm256_load_pd( &pB[0+bs*8] );
	a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
	_mm256_store_pd( &pB[0+bs*8], a_i );
	// col 9
	a_i = _mm256_load_pd( &pB[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pB[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pB[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pB[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pB[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pB[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x7 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 6
		a_0 = _mm256_load_pd( &pB[0+bs*6] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*6], a_0 );
		// col 7
		c_0 = _mm256_load_pd( &pB[0+bs*7] );
		c_0 = _mm256_fnmadd_pd( a_0, u_7, c_0 );
		_mm256_store_pd( &pB[0+bs*7], c_0 );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 6
		a_0 = _mm256_load_pd( &pB[0+bs*6] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*6], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 7
		c_0 = _mm256_load_pd( &pB[0+bs*7] );
		c_0 = _mm256_fnmadd_pd( a_0, u_7, c_0 );
		_mm256_store_pd( &pB[0+bs*7], c_0 );
		// col 8
		a_i = _mm256_load_pd( &pB[0+bs*8] );
		a_i = _mm256_fnmadd_pd( a_0, u_8, a_i );
		_mm256_store_pd( &pB[0+bs*8], a_i );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[7] = idamax+7;
	if(tmp0!=0)
		{
		if(ipiv[7]!=7)
			kernel_drowsw_lib4(12, pA+B_pref+3, pA+ipiv[7]/bs*bs*sda+ipiv[7]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+3+bs*7] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[7], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[7] = 0.0;
		}


	// ninth column

	// scale & correct & find pivot
	// prep
	idx = lft; // _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + B_pref; // XXX
	u_8 = _mm256_broadcast_sd( &pB[3+bs*8] );
	u_9 = _mm256_broadcast_sd( &pB[3+bs*9] );
	u_10 = _mm256_broadcast_sd( &pB[3+bs*10] );
	u_11 = _mm256_broadcast_sd( &pB[3+bs*11] );
	pB += B_pref; // XXX
	// col 7
	a_0 = _mm256_load_pd( &pB[0+bs*7] );
	a_0 = _mm256_mul_pd( a_0, scl );
//	tmp = _mm256_mul_pd( a_0, scl );
//	a_0 = _mm256_blend_pd( tmp, a_0, 0xf );
	_mm256_store_pd( &pB[0+bs*7], a_0 );
//	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0xf );
	// col 8
	c_0 = _mm256_load_pd( &pB[0+bs*8] );
	c_0 = _mm256_fnmadd_pd( a_0, u_8, c_0 );
	_mm256_store_pd( &pB[0+bs*8], c_0 );
	// col 9
	a_i = _mm256_load_pd( &pB[0+bs*9] );
	a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
	_mm256_store_pd( &pB[0+bs*9], a_i );
	// col 10
	a_i = _mm256_load_pd( &pB[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pB[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pB[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pB[0+bs*11], a_i );
	// search pivot
//	c_0 = _mm256_blend_pd( c_0, sgn, 0xf );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 8;
	for(; k<ma-3; k+=4)
		{
		// col 7
		a_0 = _mm256_load_pd( &pB[0+bs*7] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*7], a_0 );
		// col 8
		c_0 = _mm256_load_pd( &pB[0+bs*8] );
		c_0 = _mm256_fnmadd_pd( a_0, u_8, c_0 );
		_mm256_store_pd( &pB[0+bs*8], c_0 );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 7
		a_0 = _mm256_load_pd( &pB[0+bs*7] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*7], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 8
		c_0 = _mm256_load_pd( &pB[0+bs*8] );
		c_0 = _mm256_fnmadd_pd( a_0, u_8, c_0 );
		_mm256_store_pd( &pB[0+bs*8], c_0 );
		// col 9
		a_i = _mm256_load_pd( &pB[0+bs*9] );
		a_i = _mm256_fnmadd_pd( a_0, u_9, a_i );
		_mm256_store_pd( &pB[0+bs*9], a_i );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[8] = idamax+8;
	if(tmp0!=0)
		{
		if(ipiv[8]!=8)
			kernel_drowsw_lib4(12, pA+2*B_pref, pA+ipiv[8]/bs*bs*sda+ipiv[8]%bs);

		inv = _mm_loaddup_pd( &pA[2*B_pref+bs*8] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[8], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[8] = 0.0;
		}


	// tenth column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 2.2, 1.2, 0.2, -0.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + 2*B_pref; // XXX
	u_9 = _mm256_broadcast_sd( &pB[0+bs*9] );
	u_10 = _mm256_broadcast_sd( &pB[0+bs*10] );
	u_11 = _mm256_broadcast_sd( &pB[0+bs*11] );
	// col 8
	a_0 = _mm256_load_pd( &pB[0+bs*8] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x1 );
	_mm256_store_pd( &pB[0+bs*8], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x1 );
	// col 9
	c_0 = _mm256_load_pd( &pB[0+bs*9] );
	c_0 = _mm256_fnmadd_pd( a_0, u_9, c_0 );
	_mm256_store_pd( &pB[0+bs*9], c_0 );
	// col 10
	a_i = _mm256_load_pd( &pB[0+bs*10] );
	a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
	_mm256_store_pd( &pB[0+bs*10], a_i );
	// col 11
	a_i = _mm256_load_pd( &pB[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pB[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x1 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 8;
	for(; k<ma-3; k+=4)
		{
		// col 8
		a_0 = _mm256_load_pd( &pB[0+bs*8] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*8], a_0 );
		// col 9
		c_0 = _mm256_load_pd( &pB[0+bs*9] );
		c_0 = _mm256_fnmadd_pd( a_0, u_9, c_0 );
		_mm256_store_pd( &pB[0+bs*9], c_0 );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 8
		a_0 = _mm256_load_pd( &pB[0+bs*8] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*8], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 9
		c_0 = _mm256_load_pd( &pB[0+bs*9] );
		c_0 = _mm256_fnmadd_pd( a_0, u_9, c_0 );
		_mm256_store_pd( &pB[0+bs*9], c_0 );
		// col 10
		a_i = _mm256_load_pd( &pB[0+bs*10] );
		a_i = _mm256_fnmadd_pd( a_0, u_10, a_i );
		_mm256_store_pd( &pB[0+bs*10], a_i );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[9] = idamax+9;
	if(tmp0!=0)
		{
		if(ipiv[9]!=9)
			kernel_drowsw_lib4(12, pA+2*B_pref+1, pA+ipiv[9]/bs*bs*sda+ipiv[9]%bs);

		inv = _mm_loaddup_pd( &pA[2*B_pref+1+bs*9] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[9], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[9] = 0.0;
		}


	// eleventh column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 1.2, 0.2, -0.8, -1.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + 2*B_pref; // XXX
	u_10 = _mm256_broadcast_sd( &pB[1+bs*10] );
	u_11 = _mm256_broadcast_sd( &pB[1+bs*11] );
	// col 9
	a_0 = _mm256_load_pd( &pB[0+bs*9] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x3 );
	_mm256_store_pd( &pB[0+bs*9], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x3 );
	// col 10
	c_0 = _mm256_load_pd( &pB[0+bs*10] );
	c_0 = _mm256_fnmadd_pd( a_0, u_10, c_0 );
	_mm256_store_pd( &pB[0+bs*10], c_0 );
	// col 11
	a_i = _mm256_load_pd( &pB[0+bs*11] );
	a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
	_mm256_store_pd( &pB[0+bs*11], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x3 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 8;
	for(; k<ma-3; k+=4)
		{
		// col 9
		a_0 = _mm256_load_pd( &pB[0+bs*9] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*9], a_0 );
		// col 10
		c_0 = _mm256_load_pd( &pB[0+bs*10] );
		c_0 = _mm256_fnmadd_pd( a_0, u_10, c_0 );
		_mm256_store_pd( &pB[0+bs*10], c_0 );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 9
		a_0 = _mm256_load_pd( &pB[0+bs*9] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*9], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 10
		c_0 = _mm256_load_pd( &pB[0+bs*10] );
		c_0 = _mm256_fnmadd_pd( a_0, u_10, c_0 );
		_mm256_store_pd( &pB[0+bs*10], c_0 );
		// col 11
		a_i = _mm256_load_pd( &pB[0+bs*11] );
		a_i = _mm256_fnmadd_pd( a_0, u_11, a_i );
		_mm256_store_pd( &pB[0+bs*11], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[10] = idamax+10;
	if(tmp0!=0)
		{
		if(ipiv[10]!=10)
			kernel_drowsw_lib4(12, pA+2*B_pref+2, pA+ipiv[10]/bs*bs*sda+ipiv[10]%bs);

		inv = _mm_loaddup_pd( &pA[2*B_pref+2+bs*10] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[10], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[10] = 0.0;
		}


	// twelfth column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 0.2, -0.8, -1.8, -2.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + 2*B_pref; // XXX
	u_11 = _mm256_broadcast_sd( &pB[2+bs*11] );
	// col 10
	a_0 = _mm256_load_pd( &pB[0+bs*10] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x7 );
	_mm256_store_pd( &pB[0+bs*10], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x7 );
	// col 11
	c_0 = _mm256_load_pd( &pB[0+bs*11] );
	c_0 = _mm256_fnmadd_pd( a_0, u_11, c_0 );
	_mm256_store_pd( &pB[0+bs*11], c_0 );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x7 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 8;
	for(; k<ma-3; k+=4)
		{
		// col 10
		a_0 = _mm256_load_pd( &pB[0+bs*10] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*10], a_0 );
		// col 11
		c_0 = _mm256_load_pd( &pB[0+bs*11] );
		c_0 = _mm256_fnmadd_pd( a_0, u_11, c_0 );
		_mm256_store_pd( &pB[0+bs*11], c_0 );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 10
		a_0 = _mm256_load_pd( &pB[0+bs*10] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*10], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 11
		c_0 = _mm256_load_pd( &pB[0+bs*11] );
		c_0 = _mm256_fnmadd_pd( a_0, u_11, c_0 );
		_mm256_store_pd( &pB[0+bs*11], c_0 );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[11] = idamax+11;
	if(tmp0!=0)
		{
		if(ipiv[11]!=11)
			kernel_drowsw_lib4(12, pA+2*B_pref+3, pA+ipiv[11]/bs*bs*sda+ipiv[11]%bs);

		inv = _mm_loaddup_pd( &pA[2*B_pref+3+bs*11] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[11], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[11] = 0.0;
		}


	// scale
	pB = pA + 3*B_pref;
	k = 8;
	for(; k<ma-3; k+=4)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*11] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*11], c_0 );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*11] );
		tmp = _mm256_mul_pd( c_0, scl );
		c_0 = _mm256_blendv_pd( tmp, c_0, msk );
		_mm256_store_pd( &pB[0+bs*11], c_0 );
//		pB += B_pref;
		}

	return;

	}

	

// m>=1 and n={9,10,11,12}
void kernel_dgetrf_pivot_12_vs_lib4(int m, double *pC, int sdc, double *pd, int *ipiv, int n)
	{

	const int ps = 4;

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// saturate n to 12
	n = 12<n ? 12 : n;

	int p = m<n ? m : n;

	int n_max;

	// fact left column
	kernel_dgetrf_pivot_8_vs_lib4(m, pC, sdc, pd, ipiv, n);

	n_max = p<8 ? p : 8;

	// apply pivot to right column
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(n-8, pC+ii/ps*ps*sdc+ii%ps+8*ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps+8*ps);
			}
		}

	// solve top right block
	kernel_dtrsm_nn_ll_one_8x4_vs_lib4(0, dummy, 0, dummy, 0, &d1, pC+8*ps, sdc, pC+8*ps, sdc, pC, sdc, m, n-8);

	if(m>8)
		{

		// correct rigth block
		ii = 8;
		for(; ii<m-8; ii+=12)
			{
			kernel_dgemm_nn_12x4_vs_lib4(8, &dm1, pC+ii*sdc, sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, sdc, pC+ii*sdc+8*ps, sdc, m-ii, n-8);
			}
		if(ii<m)
			{
			if(m-ii<=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(8, &dm1, pC+ii*sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, pC+ii*sdc+8*ps, m-ii, n-8);
				}
			else //if(m-ii<=8)
				{
				kernel_dgemm_nn_8x4_vs_lib4(8, &dm1, pC+ii*sdc, sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, sdc, pC+ii*sdc+8*ps, sdc, m-ii, n-8);
				}
			}

		// fact right column
		kernel_dgetrf_pivot_4_vs_lib4(m-8, pC+8*sdc+8*ps, sdc, pd+8, ipiv+8, n-8);

		n_max = p;

		for(ii=8; ii<n_max; ii++)
			ipiv[ii] += 8;

		// apply pivot to left column
		for(ii=8; ii<n_max; ii++)
			{
			if(ipiv[ii]!=ii)
				{
				kernel_drowsw_lib4(8, pC+ii/ps*ps*sdc+ii%ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps);
				}
			}

		}

	return;

	}
// C numering (starting from zero) in the ipiv
void kernel_dgetrf_pivot_8_lib4(int m, double *pA, int sda, double *inv_diag_A, int* ipiv)
	{

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	__m128d
		max0, max1, msk0, imx0, imx1,
		inv;
	
		
	__m256d
		lft, msk,
		sgn, vna, max, imx, idx,
		ones,
		tmp,
		a_0, a_i,
		u_1, u_2, u_3, u_4, u_5, u_6, u_7,
		b_0, b_1, b_2,
		scl,
		c_0,
		d_0;
	
	double
		dlft;

	sgn = _mm256_set_pd( -0.0, -0.0, -0.0, -0.0 );
	vna = _mm256_set_pd( 4.0, 4.0, 4.0, 4.0 );
	lft  = _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	double
		tmp0;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	int B_pref = bs*sda;
	

	// first column

	// find pivot
	pB = &pA[0+bs*0];
	idx = lft; // _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	k = 0;
	for( ; k<m-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for( ; k<m-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0] );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<m)
		{
		dlft = m-k;
		msk = _mm256_broadcast_sd( &dlft );
		a_0 = _mm256_load_pd( &pB[0] );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_blendv_pd( a_0, sgn, msk );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// pivot & compute scaling
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			kernel_drowsw_lib4(8, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		inv = _mm_loaddup_pd( &pA[0+bs*0] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[0], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[0] = 0.0;
		}


	// second column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 2.2, 1.2, 0.2, -0.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_1 = _mm256_broadcast_sd( &pA[0+bs*1] );
	u_2 = _mm256_broadcast_sd( &pA[0+bs*2] );
	u_3 = _mm256_broadcast_sd( &pA[0+bs*3] );
	u_4 = _mm256_broadcast_sd( &pA[0+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[0+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[0+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[0+bs*7] );
	// col 0
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x1 );
	_mm256_store_pd( &pA[0+bs*0], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x1 );
	// col 1
	c_0 = _mm256_load_pd( &pA[0+bs*1] );
	c_0 = _mm256_fnmadd_pd( a_0, u_1, c_0 );
	_mm256_store_pd( &pA[0+bs*1], c_0 );
	// col 2
	a_i = _mm256_load_pd( &pA[0+bs*2] );
	a_i = _mm256_fnmadd_pd( a_0, u_2, a_i );
	_mm256_store_pd( &pA[0+bs*2], a_i );
	// col 3
	a_i = _mm256_load_pd( &pA[0+bs*3] );
	a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
	_mm256_store_pd( &pA[0+bs*3], a_i );
	// col 4
	a_i = _mm256_load_pd( &pA[0+bs*4] );
	a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
	_mm256_store_pd( &pA[0+bs*4], a_i );
	// col 5
	a_i = _mm256_load_pd( &pA[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pA[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pA[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pA[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pA[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pA[0+bs*7], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x1 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-3; k+=4)
		{
		// col 0
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		// col 1
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		c_0 = _mm256_fnmadd_pd( a_0, u_1, c_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		// col 2
		a_i = _mm256_load_pd( &pB[0+bs*2] );
		a_i = _mm256_fnmadd_pd( a_0, u_2, a_i );
		_mm256_store_pd( &pB[0+bs*2], a_i );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 0
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 1
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		c_0 = _mm256_fnmadd_pd( a_0, u_1, c_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		// col 2
		a_i = _mm256_load_pd( &pB[0+bs*2] );
		a_i = _mm256_fnmadd_pd( a_0, u_2, a_i );
		_mm256_store_pd( &pB[0+bs*2], a_i );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[1] = idamax+1;
	if(tmp0!=0)
		{
		if(ipiv[1]!=1)
			kernel_drowsw_lib4(8, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

		inv = _mm_loaddup_pd( &pA[1+bs*1] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[1], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[1] = 0.0;
		}



	// third column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 1.2, 0.2, -0.8, -1.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_2 = _mm256_broadcast_sd( &pA[1+bs*2] );
	u_3 = _mm256_broadcast_sd( &pA[1+bs*3] );
	u_4 = _mm256_broadcast_sd( &pA[1+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[1+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[1+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[1+bs*7] );
	// col 1
	a_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x3 );
	_mm256_store_pd( &pA[0+bs*1], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x3 );
	// col 2
	c_0 = _mm256_load_pd( &pA[0+bs*2] );
	c_0 = _mm256_fnmadd_pd( a_0, u_2, c_0 );
	_mm256_store_pd( &pA[0+bs*2], c_0 );
	// col 3
	a_i = _mm256_load_pd( &pA[0+bs*3] );
	a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
	_mm256_store_pd( &pA[0+bs*3], a_i );
	// col 4
	a_i = _mm256_load_pd( &pA[0+bs*4] );
	a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
	_mm256_store_pd( &pA[0+bs*4], a_i );
	// col 5
	a_i = _mm256_load_pd( &pA[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pA[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pA[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pA[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pA[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pA[0+bs*7], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x3 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-3; k+=4)
		{
		// col 1
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		// col 2
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		c_0 = _mm256_fnmadd_pd( a_0, u_2, c_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 1
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 2
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		c_0 = _mm256_fnmadd_pd( a_0, u_2, c_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		// col 3
		a_i = _mm256_load_pd( &pB[0+bs*3] );
		a_i = _mm256_fnmadd_pd( a_0, u_3, a_i );
		_mm256_store_pd( &pB[0+bs*3], a_i );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[2] = idamax+2;
	if(tmp0!=0)
		{
		if(ipiv[2]!=2)
			kernel_drowsw_lib4(8, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

		inv = _mm_loaddup_pd( &pA[2+bs*2] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[2], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[2] = 0.0;
		}



	// fourth column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 0.2, -0.8, -1.8, -2.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_3 = _mm256_broadcast_sd( &pA[2+bs*3] );
	u_4 = _mm256_broadcast_sd( &pA[2+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[2+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[2+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[2+bs*7] );
	// col 2
	a_0 = _mm256_load_pd( &pA[0+bs*2] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x7 );
	_mm256_store_pd( &pA[0+bs*2], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x7 );
	// col 3
	c_0 = _mm256_load_pd( &pA[0+bs*3] );
	c_0 = _mm256_fnmadd_pd( a_0, u_3, c_0 );
	_mm256_store_pd( &pA[0+bs*3], c_0 );
	// col 4
	a_i = _mm256_load_pd( &pA[0+bs*4] );
	a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
	_mm256_store_pd( &pA[0+bs*4], a_i );
	// col 5
	a_i = _mm256_load_pd( &pA[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pA[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pA[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pA[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pA[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pA[0+bs*7], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x7 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-3; k+=4)
		{
		// col 2
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		// col 3
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_fnmadd_pd( a_0, u_3, c_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 2
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 3
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_fnmadd_pd( a_0, u_3, c_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		// col 4
		a_i = _mm256_load_pd( &pB[0+bs*4] );
		a_i = _mm256_fnmadd_pd( a_0, u_4, a_i );
		_mm256_store_pd( &pB[0+bs*4], a_i );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[3] = idamax+3;
	if(tmp0!=0)
		{
		if(ipiv[3]!=3)
			kernel_drowsw_lib4(8, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

		inv = _mm_loaddup_pd( &pA[3+bs*3] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[3], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[3] = 0.0;
		}
	
	// fifth column

	// scale & correct & find pivot
	// prep
	idx = lft; // _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	u_4 = _mm256_broadcast_sd( &pA[3+bs*4] );
	u_5 = _mm256_broadcast_sd( &pA[3+bs*5] );
	u_6 = _mm256_broadcast_sd( &pA[3+bs*6] );
	u_7 = _mm256_broadcast_sd( &pA[3+bs*7] );
	pB = pA + B_pref; // XXX
	// col 3
	a_0 = _mm256_load_pd( &pB[0+bs*3] );
	a_0 = _mm256_mul_pd( a_0, scl );
//	tmp = _mm256_mul_pd( a_0, scl );
//	a_0 = _mm256_blend_pd( tmp, a_0, 0xf );
	_mm256_store_pd( &pB[0+bs*3], a_0 );
//	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0xf );
	// col 4
	c_0 = _mm256_load_pd( &pB[0+bs*4] );
	c_0 = _mm256_fnmadd_pd( a_0, u_4, c_0 );
	_mm256_store_pd( &pB[0+bs*4], c_0 );
	// col 5
	a_i = _mm256_load_pd( &pB[0+bs*5] );
	a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
	_mm256_store_pd( &pB[0+bs*5], a_i );
	// col 6
	a_i = _mm256_load_pd( &pB[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pB[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pB[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pB[0+bs*7], a_i );
	// search pivot
//	c_0 = _mm256_blend_pd( c_0, sgn, 0xf );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 3
		a_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*3], a_0 );
		// col 4
		c_0 = _mm256_load_pd( &pB[0+bs*4] );
		c_0 = _mm256_fnmadd_pd( a_0, u_4, c_0 );
		_mm256_store_pd( &pB[0+bs*4], c_0 );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 3
		a_0 = _mm256_load_pd( &pB[0+bs*3] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*3], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 4
		c_0 = _mm256_load_pd( &pB[0+bs*4] );
		c_0 = _mm256_fnmadd_pd( a_0, u_4, c_0 );
		_mm256_store_pd( &pB[0+bs*4], c_0 );
		// col 5
		a_i = _mm256_load_pd( &pB[0+bs*5] );
		a_i = _mm256_fnmadd_pd( a_0, u_5, a_i );
		_mm256_store_pd( &pB[0+bs*5], a_i );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[4] = idamax+4;
	if(tmp0!=0)
		{
		if(ipiv[4]!=4)
			kernel_drowsw_lib4(8, pA+B_pref+0, pA+ipiv[4]/bs*bs*sda+ipiv[4]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+0+bs*4] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[4], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[4] = 0.0;
		}
	
	// sixth column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 2.2, 1.2, 0.2, -0.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + B_pref; // XXX
	u_5 = _mm256_broadcast_sd( &pB[0+bs*5] );
	u_6 = _mm256_broadcast_sd( &pB[0+bs*6] );
	u_7 = _mm256_broadcast_sd( &pB[0+bs*7] );
	// col 4
	a_0 = _mm256_load_pd( &pB[0+bs*4] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x1 );
	_mm256_store_pd( &pB[0+bs*4], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x1 );
	// col 5
	c_0 = _mm256_load_pd( &pB[0+bs*5] );
	c_0 = _mm256_fnmadd_pd( a_0, u_5, c_0 );
	_mm256_store_pd( &pB[0+bs*5], c_0 );
	// col 6
	a_i = _mm256_load_pd( &pB[0+bs*6] );
	a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
	_mm256_store_pd( &pB[0+bs*6], a_i );
	// col 7
	a_i = _mm256_load_pd( &pB[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pB[0+bs*7], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x1 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 4
		a_0 = _mm256_load_pd( &pB[0+bs*4] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*4], a_0 );
		// col 5
		c_0 = _mm256_load_pd( &pB[0+bs*5] );
		c_0 = _mm256_fnmadd_pd( a_0, u_5, c_0 );
		_mm256_store_pd( &pB[0+bs*5], c_0 );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 4
		a_0 = _mm256_load_pd( &pB[0+bs*4] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*4], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 5
		c_0 = _mm256_load_pd( &pB[0+bs*5] );
		c_0 = _mm256_fnmadd_pd( a_0, u_5, c_0 );
		_mm256_store_pd( &pB[0+bs*5], c_0 );
		// col 6
		a_i = _mm256_load_pd( &pB[0+bs*6] );
		a_i = _mm256_fnmadd_pd( a_0, u_6, a_i );
		_mm256_store_pd( &pB[0+bs*6], a_i );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[5] = idamax+5;
	if(tmp0!=0)
		{
		if(ipiv[5]!=5)
			kernel_drowsw_lib4(8, pA+B_pref+1, pA+ipiv[5]/bs*bs*sda+ipiv[5]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+1+bs*5] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[5], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[5] = 0.0;
		}
	
	// seventh column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 1.2, 0.2, -0.8, -1.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + B_pref; // XXX
	u_6 = _mm256_broadcast_sd( &pB[1+bs*6] );
	u_7 = _mm256_broadcast_sd( &pB[1+bs*7] );
	// col 5
	a_0 = _mm256_load_pd( &pB[0+bs*5] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x3 );
	_mm256_store_pd( &pB[0+bs*5], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x3 );
	// col 6
	c_0 = _mm256_load_pd( &pB[0+bs*6] );
	c_0 = _mm256_fnmadd_pd( a_0, u_6, c_0 );
	_mm256_store_pd( &pB[0+bs*6], c_0 );
	// col 7
	a_i = _mm256_load_pd( &pB[0+bs*7] );
	a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
	_mm256_store_pd( &pB[0+bs*7], a_i );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x3 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 5
		a_0 = _mm256_load_pd( &pB[0+bs*5] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*5], a_0 );
		// col 6
		c_0 = _mm256_load_pd( &pB[0+bs*6] );
		c_0 = _mm256_fnmadd_pd( a_0, u_6, c_0 );
		_mm256_store_pd( &pB[0+bs*6], c_0 );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 5
		a_0 = _mm256_load_pd( &pB[0+bs*5] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*5], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 6
		c_0 = _mm256_load_pd( &pB[0+bs*6] );
		c_0 = _mm256_fnmadd_pd( a_0, u_6, c_0 );
		_mm256_store_pd( &pB[0+bs*6], c_0 );
		// col 7
		a_i = _mm256_load_pd( &pB[0+bs*7] );
		a_i = _mm256_fnmadd_pd( a_0, u_7, a_i );
		_mm256_store_pd( &pB[0+bs*7], a_i );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[6] = idamax+6;
	if(tmp0!=0)
		{
		if(ipiv[6]!=6)
			kernel_drowsw_lib4(8, pA+B_pref+2, pA+ipiv[6]/bs*bs*sda+ipiv[6]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+2+bs*6] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[6], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[6] = 0.0;
		}

	// eight column

	// scale & correct & find pivot
	// prep
	idx = _mm256_set_pd( 0.2, -0.8, -1.8, -2.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	pB = pA + B_pref; // XXX
	u_7 = _mm256_broadcast_sd( &pB[2+bs*7] );
	// col 6
	a_0 = _mm256_load_pd( &pB[0+bs*6] );
	tmp = _mm256_mul_pd( a_0, scl );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x7 );
	_mm256_store_pd( &pB[0+bs*6], a_0 );
	a_0 = _mm256_blend_pd( a_0, _mm256_setzero_pd(), 0x7 );
	// col 7
	c_0 = _mm256_load_pd( &pB[0+bs*7] );
	c_0 = _mm256_fnmadd_pd( a_0, u_7, c_0 );
	_mm256_store_pd( &pB[0+bs*7], c_0 );
	// search pivot
	c_0 = _mm256_blend_pd( c_0, sgn, 0x7 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB += B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		// col 6
		a_0 = _mm256_load_pd( &pB[0+bs*6] );
		a_0 = _mm256_mul_pd( a_0, scl );
		_mm256_store_pd( &pB[0+bs*6], a_0 );
		// col 7
		c_0 = _mm256_load_pd( &pB[0+bs*7] );
		c_0 = _mm256_fnmadd_pd( a_0, u_7, c_0 );
		_mm256_store_pd( &pB[0+bs*7], c_0 );
		// search pivot
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		// col 6
		a_0 = _mm256_load_pd( &pB[0+bs*6] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pB[0+bs*6], a_0 );
		a_0 = _mm256_blendv_pd( a_0, _mm256_setzero_pd(), msk );
		// col 7
		c_0 = _mm256_load_pd( &pB[0+bs*7] );
		c_0 = _mm256_fnmadd_pd( a_0, u_7, c_0 );
		_mm256_store_pd( &pB[0+bs*7], c_0 );
		// search pivot
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[7] = idamax+7;
	if(tmp0!=0)
		{
		if(ipiv[7]!=7)
			kernel_drowsw_lib4(8, pA+B_pref+3, pA+ipiv[7]/bs*bs*sda+ipiv[7]%bs);

		inv = _mm_loaddup_pd( &pA[B_pref+3+bs*7] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[7], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[7] = 0.0;
		}

	// scale
	pB = pA + 2*B_pref;
	k = 4;
	for(; k<ma-3; k+=4)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*7] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*7], c_0 );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*7] );
		tmp = _mm256_mul_pd( c_0, scl );
		c_0 = _mm256_blendv_pd( tmp, c_0, msk );
		_mm256_store_pd( &pB[0+bs*7], c_0 );
//		pB += B_pref;
		}

	return;

	}



// m>=1 and n={5,6,7,8}
void kernel_dgetrf_pivot_8_vs_lib4(int m, double *pC, int sdc, double *pd, int *ipiv, int n)
	{

	const int ps = 4;

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// saturate n to 8
	n = 8<n ? 8 : n;

	int p = m<n ? m : n;

	int n_max;

	// fact left column
	kernel_dgetrf_pivot_4_vs_lib4(m, pC, sdc, pd, ipiv, 4);

	n_max = p<4 ? p : 4;

	// apply pivot to right column
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(n-4, pC+ii/ps*ps*sdc+ii%ps+4*ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps+4*ps);
			}
		}

	// solve top right block
	kernel_dtrsm_nn_ll_one_4x4_vs_lib4(0, dummy, dummy, 0, &d1, pC+4*ps, pC+4*ps, pC, m, n-4);

	if(m>4)
		{

		// correct rigth block
		ii = 4;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; ii<m-8; ii+=12)
			{
			kernel_dgemm_nn_12x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, n-4);
			}
		if(ii<m)
			{
			if(m-ii<=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(4, &dm1, pC+ii*sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, pC+ii*sdc+4*ps, m-ii, n-4);
				}
			else //if(m-ii<=8)
				{
				kernel_dgemm_nn_8x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, n-4);
				}
			}
#else
		for(; ii<m-4; ii+=8)
			{
			kernel_dgemm_nn_8x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, n-4);
			}
		if(ii<m)
			{
			kernel_dgemm_nn_4x4_vs_lib4(4, &dm1, pC+ii*sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, pC+ii*sdc+4*ps, m-ii, n-4);
			}
#endif

		// fact right column
		kernel_dgetrf_pivot_4_vs_lib4(m-4, pC+4*sdc+4*ps, sdc, pd+4, ipiv+4, n-4);

		n_max = p;

		for(ii=4; ii<n_max; ii++)
			ipiv[ii] += 4;

		// apply pivot to left column
		for(ii=4; ii<n_max; ii++)
			{
			if(ipiv[ii]!=ii)
				{
				kernel_drowsw_lib4(4, pC+ii/ps*ps*sdc+ii%ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps);
				}
			}

		}

	return;

	}



// C numering (starting from zero) in the ipiv
void kernel_dgetrf_pivot_4_lib4(int m, double *pA, int sda, double *inv_diag_A, int* ipiv)
	{

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	__m128d
		max0, max1, msk0, imx0, imx1,
		inv;
	
		
	__m256d
		lft, msk,
		sgn, vna, max, imx, idx,
		ones,
		tmp,
		a_0,
		b_0, b_1, b_2,
		scl,
		c_0,
		d_0;
	
	double
		dlft;

	sgn = _mm256_set_pd( -0.0, -0.0, -0.0, -0.0 );
	vna = _mm256_set_pd( 4.0, 4.0, 4.0, 4.0 );
	lft  = _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	double
		tmp0;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	int B_pref = bs*sda;
	

	// first column

	// find pivot
	pB = &pA[0+bs*0];
	idx = lft; // _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	k = 0;
	for( ; k<m-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for( ; k<m-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0] );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<m)
		{
		dlft = m-k;
		msk = _mm256_broadcast_sd( &dlft );
		a_0 = _mm256_load_pd( &pB[0] );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_blendv_pd( a_0, sgn, msk );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			kernel_drowsw_lib4(4, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		inv = _mm_loaddup_pd( &pA[0+bs*0] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[0], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[0] = 0.0;
		}


	// second column

	// scale & correct & find pivot
	idx = _mm256_set_pd( 2.2, 1.2, 0.2, -0.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	c_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, scl );
	b_0 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x1 );
	b_0 = _mm256_permute_pd( b_0, 0x0 );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_sub_pd( c_0, tmp );
	c_0 = _mm256_blend_pd( d_0, c_0, 0x1 );
	_mm256_store_pd( &pA[0+bs*0], a_0 );
	_mm256_store_pd( &pA[0+bs*1], c_0 );
	c_0 = _mm256_blend_pd( c_0, sgn, 0x1 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( c_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[1] = idamax+1;
	if(tmp0!=0)
		{
		if(ipiv[1]!=1)
			kernel_drowsw_lib4(4, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

		inv = _mm_loaddup_pd( &pA[1+bs*1] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[1], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[1] = 0.0;
		}


	// third column

	// scale & correct & find pivot
	idx = _mm256_set_pd( 1.2, 0.2, -0.8, -1.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	c_0 = _mm256_load_pd( &pA[0+bs*2] );
	b_0 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	b_0 = _mm256_permute_pd( b_0, 0x0 );
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	tmp = _mm256_sub_pd( c_0, tmp );
	c_0 = _mm256_blend_pd( tmp, c_0, 0x1 );
	a_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, scl );
	b_1 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x3 );
	b_1 = _mm256_permute_pd( b_1, 0xf );
	tmp = _mm256_mul_pd( a_0, b_1 );
	tmp = _mm256_sub_pd( c_0, tmp );
	c_0 = _mm256_blend_pd( tmp, c_0, 0x3 );
	_mm256_store_pd( &pA[0+bs*1], a_0 );
	_mm256_store_pd( &pA[0+bs*2], c_0 );
	c_0 = _mm256_blend_pd( c_0, sgn, 0x3 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		tmp = _mm256_mul_pd( a_0, b_1 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[2] = idamax+2;
	if(tmp0!=0)
		{
		if(ipiv[2]!=2)
			kernel_drowsw_lib4(4, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

		inv = _mm_loaddup_pd( &pA[2+bs*2] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[2], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[2] = 0.0;
		}


	// fourth column

	// scale & correct & find pivot
	idx = _mm256_set_pd( 0.2, -0.8, -1.8, -2.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	c_0 = _mm256_load_pd( &pA[0+bs*3] );
	b_0 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	b_0 = _mm256_permute_pd( b_0, 0x0 );
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	tmp = _mm256_sub_pd( c_0, tmp );
	c_0 = _mm256_blend_pd( tmp, c_0, 0x1 );
	b_1 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	b_1 = _mm256_permute_pd( b_1, 0xf );
	a_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, b_1 );
	tmp = _mm256_sub_pd( c_0, tmp );
	c_0 = _mm256_blend_pd( tmp, c_0, 0x3 );
	a_0 = _mm256_load_pd( &pA[0+bs*2] );
	tmp = _mm256_mul_pd( a_0, scl );
	b_2 = _mm256_permute2f128_pd( c_0, c_0, 0x11 );
	a_0 = _mm256_blend_pd( tmp, a_0, 0x7 );
	b_2 = _mm256_permute_pd( b_2, 0x0 );
	tmp = _mm256_mul_pd( a_0, b_2 );
	tmp = _mm256_sub_pd( c_0, tmp );
	c_0 = _mm256_blend_pd( tmp, c_0, 0x7 );
	_mm256_store_pd( &pA[0+bs*2], a_0 );
	_mm256_store_pd( &pA[0+bs*3], c_0 );
	c_0 = _mm256_blend_pd( c_0, sgn, 0x7 );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_2 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_2 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_mul_pd( a_0, b_2 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( c_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_1 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		tmp = _mm256_mul_pd( a_0, b_2 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[3] = idamax+3;
	if(tmp0!=0)
		{
		if(ipiv[3]!=3)
			kernel_drowsw_lib4(4, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

		inv = _mm_loaddup_pd( &pA[3+bs*3] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[3], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[3] = 0.0;
		}

	// scale
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
//		__builtin_prefetch( pB+2*B_pref+8 );
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		pB += B_pref;
//		__builtin_prefetch( pB+2*B_pref+8 );
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		tmp = _mm256_mul_pd( c_0, scl );
		c_0 = _mm256_blendv_pd( tmp, c_0, msk );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
//		pB += B_pref;
		}

	return;

	}

	

// C numering (starting from zero) in the ipiv
void kernel_dgetrf_pivot_4_vs_lib4(int m, double *pA, int sda, double *inv_diag_A, int* ipiv, int n)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	__m128d
		max0, max1, msk0, imx0, imx1,
		inv;
	
		
	__m256d
		lft, msk,
		sgn, vna, max, imx, idx,
		ones,
		tmp,
		a_0,
		b_0, b_1, b_2,
		scl,
		c_0,
		d_0;
	
	double
		dlft;

	sgn = _mm256_set_pd( -0.0, -0.0, -0.0, -0.0 );
	vna = _mm256_set_pd( 4.0, 4.0, 4.0, 4.0 );
	lft  = _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	double
		tmp0;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	int B_pref = bs*sda;
	
	int n4 = n<4 ? n : 4;
	

	// first column

	// find pivot
	pB = &pA[0+bs*0];
	idx = lft; // _mm256_set_pd( 3.2, 2.2, 1.2, 0.2 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	k = 0;
	for( ; k<m-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0] );
//		__builtin_prefetch( pB+2*B_pref );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for( ; k<m-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0] );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<m)
		{
		dlft = m-k;
		msk = _mm256_broadcast_sd( &dlft );
		a_0 = _mm256_load_pd( &pB[0] );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_blendv_pd( a_0, sgn, msk );
		a_0 = _mm256_andnot_pd( sgn, a_0 ); // abs
		msk = _mm256_cmp_pd( a_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, a_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			kernel_drowsw_lib4(n4, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		inv = _mm_loaddup_pd( &pA[0+bs*0] );
		inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
		scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
		_mm_store_sd( &inv_diag_A[0], inv );
		}
	else
		{
		scl = ones;
		inv_diag_A[0] = 0.0;
		}
	
	if(n==1)
		{
		// scale & return
		dlft = m;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_load_pd( &pA[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_blend_pd( tmp, a_0, 0x1 );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pA[0+bs*0], a_0 );
		pB = pA + B_pref;
		k = 0;
		for(; k<ma-7; k+=8)
			{
			a_0 = _mm256_load_pd( &pB[0+bs*0] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*0], a_0 );
			pB += B_pref;
			a_0 = _mm256_load_pd( &pB[0+bs*0] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*0], a_0 );
			pB += B_pref;
			}
		for(; k<ma-3; k+=4)
			{
			a_0 = _mm256_load_pd( &pB[0+bs*0] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*0], a_0 );
			pB += B_pref;
			}
		if(k<ma)
			{
			dlft = ma-k;
			msk = _mm256_broadcast_sd( &dlft );
			msk = _mm256_cmp_pd( lft, msk, 14 ); // >
			a_0 = _mm256_load_pd( &pB[0+bs*0] );
			tmp = _mm256_mul_pd( a_0, scl );
			a_0 = _mm256_blendv_pd( tmp, a_0, msk );
			_mm256_store_pd( &pB[0+bs*0], a_0 );
	//		pB += B_pref;
			}

		return;
		}


	// second column

	// scale & correct & find pivot
	dlft = m;
	msk = _mm256_broadcast_sd( &dlft );
	msk = _mm256_cmp_pd( lft, msk, 14 ); // >
	idx = _mm256_set_pd( 2.2, 1.2, 0.2, -0.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	c_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, scl );
	b_0 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	tmp = _mm256_blend_pd( tmp, a_0, 0x1 );
	a_0 = _mm256_blendv_pd( tmp, a_0, msk );
	b_0 = _mm256_permute_pd( b_0, 0x0 );
	tmp = _mm256_mul_pd( a_0, b_0 );
	d_0 = _mm256_sub_pd( c_0, tmp );
	d_0 = _mm256_blend_pd( d_0, c_0, 0x1 );
	c_0 = _mm256_blendv_pd( d_0, c_0, msk );
	_mm256_store_pd( &pA[0+bs*0], a_0 );
	_mm256_store_pd( &pA[0+bs*1], c_0 );
	c_0 = _mm256_blend_pd( c_0, sgn, 0x1 );
	c_0 = _mm256_blendv_pd( c_0, sgn, msk );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( c_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk );
		_mm256_store_pd( &pB[0+bs*0], a_0 );
		_mm256_store_pd( &pB[0+bs*1], c_0 );
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	if(m>1)
		{
		ipiv[1] = idamax+1;
		if(tmp0!=0)
			{
			if(ipiv[1]!=1)
				kernel_drowsw_lib4(n4, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

			inv = _mm_loaddup_pd( &pA[1+bs*1] );
			inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
			scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
			_mm_store_sd( &inv_diag_A[1], inv );
			}
		else
			{
			scl = ones;
			inv_diag_A[1] = 0.0;
			}
		}

	if(n==2)
		{
		// scale & return
		dlft = m;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_load_pd( &pA[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_blend_pd( tmp, a_0, 0x3 );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pA[0+bs*1], a_0 );
		pB = pA + B_pref;
		k = 0;
		for(; k<ma-7; k+=8)
			{
			a_0 = _mm256_load_pd( &pB[0+bs*1] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*1], a_0 );
			pB += B_pref;
			a_0 = _mm256_load_pd( &pB[0+bs*1] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*1], a_0 );
			pB += B_pref;
			}
		for(; k<ma-3; k+=4)
			{
			a_0 = _mm256_load_pd( &pB[0+bs*1] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*1], a_0 );
			pB += B_pref;
			}
		if(k<ma)
			{
			dlft = ma-k;
			msk = _mm256_broadcast_sd( &dlft );
			msk = _mm256_cmp_pd( lft, msk, 14 ); // >
			a_0 = _mm256_load_pd( &pB[0+bs*1] );
			tmp = _mm256_mul_pd( a_0, scl );
			a_0 = _mm256_blendv_pd( tmp, a_0, msk );
			_mm256_store_pd( &pB[0+bs*1], a_0 );
	//		pB += B_pref;
			}

		return;
		}

	// third column

	// scale & correct & find pivot
	dlft = m;
	msk = _mm256_broadcast_sd( &dlft );
	msk = _mm256_cmp_pd( lft, msk, 14 ); // >
	idx = _mm256_set_pd( 1.2, 0.2, -0.8, -1.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	c_0 = _mm256_load_pd( &pA[0+bs*2] );
	b_0 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	b_0 = _mm256_permute_pd( b_0, 0x0 );
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	tmp = _mm256_sub_pd( c_0, tmp );
	tmp = _mm256_blend_pd( tmp, c_0, 0x1 );
	c_0 = _mm256_blendv_pd( tmp, c_0, msk );
	a_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, scl );
	b_1 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	tmp = _mm256_blend_pd( tmp, a_0, 0x3 );
	a_0 = _mm256_blendv_pd( tmp, a_0, msk );
	b_1 = _mm256_permute_pd( b_1, 0xf );
	tmp = _mm256_mul_pd( a_0, b_1 );
	tmp = _mm256_sub_pd( c_0, tmp );
	tmp = _mm256_blend_pd( tmp, c_0, 0x3 );
	c_0 = _mm256_blendv_pd( tmp, c_0, msk );
	_mm256_store_pd( &pA[0+bs*1], a_0 );
	_mm256_store_pd( &pA[0+bs*2], c_0 );
	c_0 = _mm256_blend_pd( c_0, sgn, 0x3 );
	c_0 = _mm256_blendv_pd( c_0, sgn, msk );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		a_0 = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		tmp = _mm256_mul_pd( a_0, b_1 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		_mm256_store_pd( &pB[0+bs*1], a_0 );
		_mm256_store_pd( &pB[0+bs*2], c_0 );
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	if(m>2)
		{
		ipiv[2] = idamax+2;
		if(tmp0!=0)
			{
			if(ipiv[2]!=2)
				kernel_drowsw_lib4(n4, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

			inv = _mm_loaddup_pd( &pA[2+bs*2] );
			inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
			scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
			_mm_store_sd( &inv_diag_A[2], inv );
			}
		else
			{
			scl = ones;
			inv_diag_A[2] = 0.0;
			}
		}

	if(n==3)
		{
		// scale & return
		dlft = m;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		a_0 = _mm256_load_pd( &pA[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_blend_pd( tmp, a_0, 0x7 );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		_mm256_store_pd( &pA[0+bs*2], a_0 );
		pB = pA + B_pref;
		k = 0;
		for(; k<ma-7; k+=8)
			{
			a_0 = _mm256_load_pd( &pB[0+bs*2] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*2], a_0 );
			pB += B_pref;
			a_0 = _mm256_load_pd( &pB[0+bs*2] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*2], a_0 );
			pB += B_pref;
			}
		for(; k<ma-3; k+=4)
			{
			a_0 = _mm256_load_pd( &pB[0+bs*2] );
			a_0 = _mm256_mul_pd( a_0, scl );
			_mm256_store_pd( &pB[0+bs*2], a_0 );
			pB += B_pref;
			}
		if(k<ma)
			{
			dlft = ma-k;
			msk = _mm256_broadcast_sd( &dlft );
			msk = _mm256_cmp_pd( lft, msk, 14 ); // >
			a_0 = _mm256_load_pd( &pB[0+bs*2] );
			tmp = _mm256_mul_pd( a_0, scl );
			a_0 = _mm256_blendv_pd( tmp, a_0, msk );
			_mm256_store_pd( &pB[0+bs*2], a_0 );
	//		pB += B_pref;
			}

		return;
		}

	// fourth column

	// scale & correct & find pivot
	dlft = m;
	msk = _mm256_broadcast_sd( &dlft );
	msk = _mm256_cmp_pd( lft, msk, 14 ); // >
	idx = _mm256_set_pd( 0.2, -0.8, -1.8, -2.8 );
	max = _mm256_setzero_pd();
	imx = _mm256_setzero_pd();
	c_0 = _mm256_load_pd( &pA[0+bs*3] );
	b_0 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	b_0 = _mm256_permute_pd( b_0, 0x0 );
	a_0 = _mm256_load_pd( &pA[0+bs*0] );
	tmp = _mm256_mul_pd( a_0, b_0 );
	tmp = _mm256_sub_pd( c_0, tmp );
	tmp = _mm256_blend_pd( tmp, c_0, 0x1 );
	c_0 = _mm256_blendv_pd( tmp, c_0, msk );
	b_1 = _mm256_permute2f128_pd( c_0, c_0, 0x00 );
	b_1 = _mm256_permute_pd( b_1, 0xf );
	a_0 = _mm256_load_pd( &pA[0+bs*1] );
	tmp = _mm256_mul_pd( a_0, b_1 );
	tmp = _mm256_sub_pd( c_0, tmp );
	tmp = _mm256_blend_pd( tmp, c_0, 0x3 );
	c_0 = _mm256_blendv_pd( tmp, c_0, msk );
	a_0 = _mm256_load_pd( &pA[0+bs*2] );
	tmp = _mm256_mul_pd( a_0, scl );
	b_2 = _mm256_permute2f128_pd( c_0, c_0, 0x11 );
	tmp = _mm256_blend_pd( tmp, a_0, 0x7 );
	a_0 = _mm256_blendv_pd( tmp, a_0, msk );
	b_2 = _mm256_permute_pd( b_2, 0x0 );
	tmp = _mm256_mul_pd( a_0, b_2 );
	tmp = _mm256_sub_pd( c_0, tmp );
	tmp = _mm256_blend_pd( tmp, c_0, 0x7 );
	c_0 = _mm256_blendv_pd( tmp, c_0, msk );
	_mm256_store_pd( &pA[0+bs*2], a_0 );
	_mm256_store_pd( &pA[0+bs*3], c_0 );
	c_0 = _mm256_blend_pd( c_0, sgn, 0x7 );
	c_0 = _mm256_blendv_pd( c_0, sgn, msk );
	c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
	msk = _mm256_cmp_pd( c_0, max, 14 ); // >
	max = _mm256_blendv_pd( max, c_0, msk );
	imx = _mm256_blendv_pd( imx, idx, msk );
	idx = _mm256_add_pd( idx, vna );
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_2 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
//		__builtin_prefetch( pB+2*B_pref );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
//		__builtin_prefetch( pB+2*B_pref+8 );
		tmp = _mm256_mul_pd( a_0, b_2 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_1 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		a_0 = _mm256_mul_pd( a_0, scl );
		tmp = _mm256_mul_pd( a_0, b_2 );
		c_0 = _mm256_sub_pd( c_0, tmp );
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
		idx = _mm256_add_pd( idx, vna );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		a_0 = _mm256_load_pd( &pB[0+bs*0] );
		tmp = _mm256_mul_pd( a_0, b_0 );
		d_0 = _mm256_sub_pd( c_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		a_0 = _mm256_load_pd( &pB[0+bs*1] );
		tmp = _mm256_mul_pd( a_0, b_1 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		a_0 = _mm256_load_pd( &pB[0+bs*2] );
		tmp = _mm256_mul_pd( a_0, scl );
		a_0 = _mm256_blendv_pd( tmp, a_0, msk );
		tmp = _mm256_mul_pd( a_0, b_2 );
		d_0 = _mm256_sub_pd( d_0, tmp );
		c_0 = _mm256_blendv_pd( d_0, c_0, msk);
		_mm256_store_pd( &pB[0+bs*2], a_0 );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		c_0 = _mm256_blendv_pd( c_0, sgn, msk );
		c_0 = _mm256_andnot_pd( sgn, c_0 ); // abs
		msk = _mm256_cmp_pd( c_0, max, 14 ); // >
		max = _mm256_blendv_pd( max, c_0, msk );
		imx = _mm256_blendv_pd( imx, idx, msk );
//		idx = _mm256_add_pd( idx, vna );
//		pB += B_pref;
		}
	max0 = _mm256_extractf128_pd( max, 0x0 );
	max1 = _mm256_extractf128_pd( max, 0x1 );
	imx0 = _mm256_extractf128_pd( imx, 0x0 ); // lower indexes in case of identical max value
	imx1 = _mm256_extractf128_pd( imx, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	max1 = _mm_permute_pd( max0, 0x1 );
	imx1 = _mm_permute_pd( imx0, 0x1 );
	msk0 = _mm_cmp_pd( max1, max0, 14 );
	max0 = _mm_blendv_pd( max0, max1, msk0 );
	imx0 = _mm_blendv_pd( imx0, imx1, msk0 );
	_mm_store_sd( &tmp0, max0 );
	idamax = _mm_cvtsd_si32( imx0 );

	// compute scaling
	if(m>3)
		{
		ipiv[3] = idamax+3;
		if(tmp0!=0)
			{
			if(ipiv[3]!=3)
				kernel_drowsw_lib4(n4, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

			inv = _mm_loaddup_pd( &pA[3+bs*3] );
			inv = _mm_div_pd( _mm256_castpd256_pd128( ones ), inv );
			scl = _mm256_permute2f128_pd( _mm256_castpd128_pd256( inv ), _mm256_castpd128_pd256( inv ), 0x00 );
			_mm_store_sd( &inv_diag_A[3], inv );
			}
		else
			{
			scl = ones;
			inv_diag_A[3] = 0.0;
			}
		}

	// scale
	pB = pA + B_pref;
	k = 0;
	for(; k<ma-7; k+=8)
		{
//		__builtin_prefetch( pB+2*B_pref+8 );
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		pB += B_pref;
//		__builtin_prefetch( pB+2*B_pref+8 );
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		pB += B_pref;
		}
	for(; k<ma-3; k+=4)
		{
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		c_0 = _mm256_mul_pd( c_0, scl );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
		pB += B_pref;
		}
	if(k<ma)
		{
		dlft = ma-k;
		msk = _mm256_broadcast_sd( &dlft );
		msk = _mm256_cmp_pd( lft, msk, 14 ); // >
		c_0 = _mm256_load_pd( &pB[0+bs*3] );
		tmp = _mm256_mul_pd( c_0, scl );
		c_0 = _mm256_blendv_pd( tmp, c_0, msk );
		_mm256_store_pd( &pB[0+bs*3], c_0 );
//		pB += B_pref;
		}

	return;

	}





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgetrf_pivot_lib.c"

#endif
