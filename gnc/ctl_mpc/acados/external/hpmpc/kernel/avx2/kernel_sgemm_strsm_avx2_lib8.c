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



void kernel_sgemm_strsm_nt_24x4_lib8(int kadd, int ksub, float *A0, float *A1, float *A2, float *B, float *C0, float *C1, float *C2, float *D0, float *D1, float *D2, float *fact)
	{
	
	const int bs = 8;
	const int ncl = S_NCL;
	const int ldc = 8;
	
	int k;

	__m256
		a_0, a_8, a_g,
		b_0,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83,
		c_g0, c_g1, c_g2, c_g3;
	
	// prefetch
	a_0 = _mm256_load_ps( &A0[0] );
	a_8 = _mm256_load_ps( &A1[0] );
	a_g = _mm256_load_ps( &A2[0] );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();
	c_g0 = _mm256_setzero_ps();
	c_g1 = _mm256_setzero_ps();
	c_g2 = _mm256_setzero_ps();
	c_g3 = _mm256_setzero_ps();

	k = 0;
	for(; k<kadd-3; k+=4)
		{

		b_0 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );
		
		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[16] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[16] );

		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[24] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[24] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[24] );

		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[32] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[32] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[32] );


		A0 += 32;
		A1 += 32;
		A2 += 32;
		B  += 32;

		}
	if(kadd%4>=2)
		{
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );
		
		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[16] );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[16] );


		A0 += 16;
		A1 += 16;
		A2 += 16;
		B  += 16;
		
		}
	if(kadd%2==1)
		{
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		c_00 = _mm256_fmadd_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmadd_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmadd_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmadd_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmadd_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmadd_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmadd_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmadd_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmadd_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmadd_ps( a_0, b_0, c_03 );
		c_83 = _mm256_fmadd_ps( a_8, b_0, c_83 );
		c_g3 = _mm256_fmadd_ps( a_g, b_0, c_g3 );

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A0 += bs*((ncl-kadd%ncl)%ncl);
			A1 += bs*((ncl-kadd%ncl)%ncl);
			A2 += bs*((ncl-kadd%ncl)%ncl);
			B  += bs*((ncl-kadd%ncl)%ncl);
/*printf("\nk0 = %d\n", (ncl-kadd%ncl)%ncl);*/
			}
		// prefetch
		a_0 = _mm256_load_ps( &A0[0] );
		a_8 = _mm256_load_ps( &A1[0] );
		a_g = _mm256_load_ps( &A2[0] );
		}

	for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
		{

		b_0 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		c_00 = _mm256_fmsub_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmsub_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmsub_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmsub_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmsub_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmsub_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmsub_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmsub_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmsub_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmsub_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[8] );
		c_83 = _mm256_fmsub_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[8] );
		c_g3 = _mm256_fmsub_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[8] );
		
		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_fmsub_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmsub_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmsub_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmsub_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmsub_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmsub_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmsub_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmsub_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmsub_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmsub_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[16] );
		c_83 = _mm256_fmsub_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[16] );
		c_g3 = _mm256_fmsub_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[16] );

		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_fmsub_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmsub_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmsub_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmsub_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmsub_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmsub_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmsub_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmsub_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmsub_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmsub_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[24] );
		c_83 = _mm256_fmsub_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[24] );
		c_g3 = _mm256_fmsub_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[24] );

		
		
		b_0 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_fmsub_ps( a_0, b_0, c_00 );
		c_80 = _mm256_fmsub_ps( a_8, b_0, c_80 );
		c_g0 = _mm256_fmsub_ps( a_g, b_0, c_g0 );
		
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_01 = _mm256_fmsub_ps( a_0, b_0, c_01 );
		c_81 = _mm256_fmsub_ps( a_8, b_0, c_81 );
		c_g1 = _mm256_fmsub_ps( a_g, b_0, c_g1 );
		
		b_0  = _mm256_permute_ps( b_0, 0x4e );
		c_02 = _mm256_fmsub_ps( a_0, b_0, c_02 );
		c_82 = _mm256_fmsub_ps( a_8, b_0, c_82 );
		c_g2 = _mm256_fmsub_ps( a_g, b_0, c_g2 );
	
		b_0  = _mm256_permute_ps( b_0, 0xb1 );
		c_03 = _mm256_fmsub_ps( a_0, b_0, c_03 );
		a_0 = _mm256_load_ps( &A0[32] );
		c_83 = _mm256_fmsub_ps( a_8, b_0, c_83 );
		a_8 = _mm256_load_ps( &A1[32] );
		c_g3 = _mm256_fmsub_ps( a_g, b_0, c_g3 );
		a_g = _mm256_load_ps( &A2[32] );


		A0 += 32;
		A1 += 32;
		A2 += 32;
		B  += 32;

		}

	__m256
		c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, c_a, c_b,
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83,
		d_g0, d_g1, d_g2, d_g3;

	c_0 = _mm256_blend_ps( c_00, c_01, 0x55 );
	c_1 = _mm256_blend_ps( c_01, c_00, 0x55 );
	c_2 = _mm256_blend_ps( c_03, c_02, 0x55 );
	c_3 = _mm256_blend_ps( c_02, c_03, 0x55 );
	c_4 = _mm256_blend_ps( c_80, c_81, 0x55 );
	c_5 = _mm256_blend_ps( c_81, c_80, 0x55 );
	c_6 = _mm256_blend_ps( c_83, c_82, 0x55 );
	c_7 = _mm256_blend_ps( c_82, c_83, 0x55 );
	c_8 = _mm256_blend_ps( c_g0, c_g1, 0x55 );
	c_9 = _mm256_blend_ps( c_g1, c_g0, 0x55 );
	c_a = _mm256_blend_ps( c_g3, c_g2, 0x55 );
	c_b = _mm256_blend_ps( c_g2, c_g3, 0x55 );
	
	c_01 = _mm256_blend_ps( c_0, c_2, 0xcc );
	c_03 = _mm256_blend_ps( c_2, c_0, 0xcc );
	c_00 = _mm256_blend_ps( c_1, c_3, 0xcc );
	c_02 = _mm256_blend_ps( c_3, c_1, 0xcc );
	c_81 = _mm256_blend_ps( c_4, c_6, 0xcc );
	c_83 = _mm256_blend_ps( c_6, c_4, 0xcc );
	c_80 = _mm256_blend_ps( c_5, c_7, 0xcc );
	c_82 = _mm256_blend_ps( c_7, c_5, 0xcc );
	c_g1 = _mm256_blend_ps( c_8, c_a, 0xcc );
	c_g3 = _mm256_blend_ps( c_a, c_8, 0xcc );
	c_g0 = _mm256_blend_ps( c_9, c_b, 0xcc );
	c_g2 = _mm256_blend_ps( c_b, c_9, 0xcc );

	d_00 = _mm256_load_ps( &C0[0+ldc*0] );
	d_00 = _mm256_add_ps( d_00, c_00 );
	d_01 = _mm256_load_ps( &C0[0+ldc*1] );
	d_01 = _mm256_add_ps( d_01, c_01 );
	d_02 = _mm256_load_ps( &C0[0+ldc*2] );
	d_02 = _mm256_add_ps( d_02, c_02 );
	d_03 = _mm256_load_ps( &C0[0+ldc*3] );
	d_03 = _mm256_add_ps( d_03, c_03 );
	d_80 = _mm256_load_ps( &C1[0+ldc*0] );
	d_80 = _mm256_add_ps( d_80, c_80 );
	d_81 = _mm256_load_ps( &C1[0+ldc*1] );
	d_81 = _mm256_add_ps( d_81, c_81 );
	d_82 = _mm256_load_ps( &C1[0+ldc*2] );
	d_82 = _mm256_add_ps( d_82, c_82 );
	d_83 = _mm256_load_ps( &C1[0+ldc*3] );
	d_83 = _mm256_add_ps( d_83, c_83 );
	d_g0 = _mm256_load_ps( &C2[0+ldc*0] );
	d_g0 = _mm256_add_ps( d_g0, c_g0 );
	d_g1 = _mm256_load_ps( &C2[0+ldc*1] );
	d_g1 = _mm256_add_ps( d_g1, c_g1 );
	d_g2 = _mm256_load_ps( &C2[0+ldc*2] );
	d_g2 = _mm256_add_ps( d_g2, c_g2 );
	d_g3 = _mm256_load_ps( &C2[0+ldc*3] );
	d_g3 = _mm256_add_ps( d_g3, c_g3 );

	// factorize the upper 4x4 matrix
	__m256
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	// first row
	a_00 = _mm256_broadcast_ss( &fact[0] );
	d_00  = _mm256_mul_ps( d_00, a_00 );
	_mm256_store_ps( &D0[0+ldc*0], d_00 ); // a_00
	d_80 = _mm256_mul_ps( d_80, a_00 );
	_mm256_store_ps( &D1[0+ldc*0], d_80 );
	d_g0 = _mm256_mul_ps( d_g0, a_00 );
	_mm256_store_ps( &D2[0+ldc*0], d_g0 );
		
	// second row
	a_10 = _mm256_broadcast_ss( &fact[1] );
	a_11 = _mm256_broadcast_ss( &fact[2] );
	d_01  = _mm256_fmsub_ps( d_00, a_10, d_01 );
	d_81  = _mm256_fmsub_ps( d_80, a_10, d_81 );
	d_g1  = _mm256_fmsub_ps( d_g0, a_10, d_g1 );
	d_01  = _mm256_mul_ps( d_01, a_11 );
	_mm256_store_ps( &D0[0+ldc*1], d_01 ); // a_00
	d_81 = _mm256_mul_ps( d_81, a_11 );
	_mm256_store_ps( &D1[0+ldc*1], d_81 );
	d_g1 = _mm256_mul_ps( d_g1, a_11 );
	_mm256_store_ps( &D2[0+ldc*1], d_g1 );

	// third row
	a_20 = _mm256_broadcast_ss( &fact[3] );
	a_21 = _mm256_broadcast_ss( &fact[4] );
	a_22 = _mm256_broadcast_ss( &fact[5] );
	d_02  = _mm256_fmsub_ps( d_00, a_20, d_02 );
	d_82  = _mm256_fmsub_ps( d_80, a_20, d_82 );
	d_g2  = _mm256_fmsub_ps( d_g0, a_20, d_g2 );
	d_02  = _mm256_fmsub_ps( d_01, a_21, d_02 );
	d_82  = _mm256_fmsub_ps( d_81, a_21, d_82 );
	d_g2  = _mm256_fmsub_ps( d_g1, a_21, d_g2 );
	d_02  = _mm256_mul_ps( d_02, a_22 );
	_mm256_store_ps( &D0[0+ldc*2], d_02 ); // a_00
	d_82 = _mm256_mul_ps( d_82, a_22 );
	_mm256_store_ps( &D1[0+ldc*2], d_82 );
	d_g2 = _mm256_mul_ps( d_g2, a_22 );
	_mm256_store_ps( &D2[0+ldc*2], d_g2 );

	// fourth row
	a_30 = _mm256_broadcast_ss( &fact[6] );
	a_31 = _mm256_broadcast_ss( &fact[7] );
	a_32 = _mm256_broadcast_ss( &fact[8] );
	a_33 = _mm256_broadcast_ss( &fact[9] );
	d_03  = _mm256_fmsub_ps( d_00, a_30, d_03 );
	d_83  = _mm256_fmsub_ps( d_80, a_30, d_83 );
	d_g3  = _mm256_fmsub_ps( d_g0, a_30, d_g3 );
	d_03  = _mm256_fmsub_ps( d_01, a_31, d_03 );
	d_83  = _mm256_fmsub_ps( d_81, a_31, d_83 );
	d_g3  = _mm256_fmsub_ps( d_g1, a_31, d_g3 );
	d_03  = _mm256_fmsub_ps( d_02, a_32, d_03 );
	d_83  = _mm256_fmsub_ps( d_82, a_32, d_83 );
	d_g3  = _mm256_fmsub_ps( d_g2, a_32, d_g3 );
	d_03  = _mm256_mul_ps( d_03, a_33 );
	_mm256_store_ps( &D0[0+ldc*3], d_03 ); // a_00
	d_83 = _mm256_mul_ps( d_83, a_33 );
	_mm256_store_ps( &D1[0+ldc*3], d_83 );

	}





// normal-transposed, 16x4 with data packed in 4
void kernel_sgemm_strsm_nt_16x4_lib8(int kadd, int ksub, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, float *fact)
	{
	
	const int bs = 8;
	const int ncl = S_NCL;
	const int ldc = 8;
	
	int k;

	__m256
		temp,
		a_07, a_8f, A_07, A_8f,
		b_03, B_03, b_0,
		c_00, c_01, c_02, c_03,
		c_80, c_81, c_82, c_83;
	
	// prefetch
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
	a_07 = _mm256_load_ps( &A0[0] );
	a_8f = _mm256_load_ps( &A1[0] );
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();
	c_80 = _mm256_setzero_ps();
	c_81 = _mm256_setzero_ps();
	c_82 = _mm256_setzero_ps();
	c_83 = _mm256_setzero_ps();

	k = 0;
	for(; k<kadd-3; k+=4)
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[24] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[32] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}
	if(kadd%4>=2)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );


		A0 += 16;
		A1 += 16;
		B  += 16;
		
		}
	if(kadd%2==1)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );*/
		c_00 = _mm256_add_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_add_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		A_07 = _mm256_load_ps( &A0[8] );*/
		c_01 = _mm256_add_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_add_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
/*		A_8f = _mm256_load_ps( &A1[8] );*/
		c_02 = _mm256_add_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_add_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_add_ps( c_83, temp );

		A0 += 8; // keep it !!!
		A1 += 8; // keep it !!!
		B  += 8; // keep it !!!

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A0 += bs*((ncl-kadd%ncl)%ncl);
			A1 += bs*((ncl-kadd%ncl)%ncl);
			B  += bs*((ncl-kadd%ncl)%ncl);
/*printf("\nk0 = %d\n", (ncl-kadd%ncl)%ncl);*/
			}
		// prefetch
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		a_07 = _mm256_load_ps( &A0[0] );
		a_8f = _mm256_load_ps( &A1[0] );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		}

	for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[8] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[16] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( a_07, b_0 );
		A_8f = _mm256_load_ps( &A1[24] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( a_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_sub_ps( c_00, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		c_80 = _mm256_sub_ps( c_80, temp );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_sub_ps( c_01, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );
		c_81 = _mm256_sub_ps( c_81, temp );

		temp = _mm256_mul_ps( A_07, b_0 );
		a_8f = _mm256_load_ps( &A1[32] );
		c_02 = _mm256_sub_ps( c_02, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
		c_82 = _mm256_sub_ps( c_82, temp );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		temp = _mm256_mul_ps( A_8f, b_0 );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		c_83 = _mm256_sub_ps( c_83, temp );


		A0 += 32;
		A1 += 32;
		B  += 32;

		}

	__m256
		d_00, d_01, d_02, d_03,
		d_80, d_81, d_82, d_83;

	d_00 = _mm256_load_ps( &C0[0+ldc*0] );
	d_00 = _mm256_add_ps( d_00, c_00 );
	d_01 = _mm256_load_ps( &C0[0+ldc*1] );
	d_01 = _mm256_add_ps( d_01, c_01 );
	d_02 = _mm256_load_ps( &C0[0+ldc*2] );
	d_02 = _mm256_add_ps( d_02, c_02 );
	d_03 = _mm256_load_ps( &C0[0+ldc*3] );
	d_03 = _mm256_add_ps( d_03, c_03 );
	d_80 = _mm256_load_ps( &C1[0+ldc*0] );
	d_80 = _mm256_add_ps( d_80, c_80 );
	d_81 = _mm256_load_ps( &C1[0+ldc*1] );
	d_81 = _mm256_add_ps( d_81, c_81 );
	d_82 = _mm256_load_ps( &C1[0+ldc*2] );
	d_82 = _mm256_add_ps( d_82, c_82 );
	d_83 = _mm256_load_ps( &C1[0+ldc*3] );
	d_83 = _mm256_add_ps( d_83, c_83 );

	// factorize the upper 4x4 matrix
	__m256
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	// first row
	a_00 = _mm256_broadcast_ss( &fact[0] );
	d_00  = _mm256_mul_ps( d_00, a_00 );
	_mm256_store_ps( &D0[0+ldc*0], d_00 ); // a_00
	d_80 = _mm256_mul_ps( d_80, a_00 );
	_mm256_store_ps( &D1[0+ldc*0], d_80 );
		
	// second row
	a_10 = _mm256_broadcast_ss( &fact[1] );
	a_11 = _mm256_broadcast_ss( &fact[2] );
	temp  = _mm256_mul_ps( d_00, a_10 );
	d_01  = _mm256_sub_ps( d_01, temp );
	temp  = _mm256_mul_ps( d_80, a_10 );
	d_81  = _mm256_sub_ps( d_81, temp );
	d_01  = _mm256_mul_ps( d_01, a_11 );
	_mm256_store_ps( &D0[0+ldc*1], d_01 ); // a_00
	d_81 = _mm256_mul_ps( d_81, a_11 );
	_mm256_store_ps( &D1[0+ldc*1], d_81 );

	// third row
	a_20 = _mm256_broadcast_ss( &fact[3] );
	a_21 = _mm256_broadcast_ss( &fact[4] );
	a_22 = _mm256_broadcast_ss( &fact[5] );
	temp  = _mm256_mul_ps( d_00, a_20 );
	d_02  = _mm256_sub_ps( d_02, temp );
	temp  = _mm256_mul_ps( d_80, a_20 );
	d_82  = _mm256_sub_ps( d_82, temp );
	temp  = _mm256_mul_ps( d_01, a_21 );
	d_02  = _mm256_sub_ps( d_02, temp );
	temp  = _mm256_mul_ps( d_81, a_21 );
	d_82  = _mm256_sub_ps( d_82, temp );
	d_02  = _mm256_mul_ps( d_02, a_22 );
	_mm256_store_ps( &D0[0+ldc*2], d_02 ); // a_00
	d_82 = _mm256_mul_ps( d_82, a_22 );
	_mm256_store_ps( &D1[0+ldc*2], d_82 );

	// fourth row
	a_30 = _mm256_broadcast_ss( &fact[6] );
	a_31 = _mm256_broadcast_ss( &fact[7] );
	a_32 = _mm256_broadcast_ss( &fact[8] );
	a_33 = _mm256_broadcast_ss( &fact[9] );
	temp  = _mm256_mul_ps( d_00, a_30 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_80, a_30 );
	d_83  = _mm256_sub_ps( d_83, temp );
	temp  = _mm256_mul_ps( d_01, a_31 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_81, a_31 );
	d_83  = _mm256_sub_ps( d_83, temp );
	temp  = _mm256_mul_ps( d_02, a_32 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_82, a_32 );
	d_83  = _mm256_sub_ps( d_83, temp );
	d_03  = _mm256_mul_ps( d_03, a_33 );
	_mm256_store_ps( &D0[0+ldc*3], d_03 ); // a_00
	d_83 = _mm256_mul_ps( d_83, a_33 );
	_mm256_store_ps( &D1[0+ldc*3], d_83 );

	}



// normal-transposed, 16x4 with data packed in 4
void kernel_sgemm_strsm_nt_8x4_lib8(int kadd, int ksub, float *A0, float *B, float *C0, float *D0, float *fact)
	{
	
	const int bs = 8;
	const int ncl = S_NCL;
	const int ldc = 8;
	
	int k;

	__m256
		temp,
		a_07, A_07,
		b_03, B_03, b_0,
		c_00, c_01, c_02, c_03;
	
	// prefetch
	b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
	a_07 = _mm256_load_ps( &A0[0] );
	b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

	c_00 = _mm256_setzero_ps();
	c_01 = _mm256_setzero_ps();
	c_02 = _mm256_setzero_ps();
	c_03 = _mm256_setzero_ps();

	k = 0;
	for(; k<kadd-3; k+=4)
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );


		A0 += 32;
		B  += 32;

		}
	if(kadd%4>=2)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );


		A0 += 16;
		B  += 16;
		
		}
	if(kadd%2==1)
		{
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );*/
		c_00 = _mm256_add_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
/*		A_07 = _mm256_load_ps( &A0[8] );*/
		c_01 = _mm256_add_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_add_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_add_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		A0 += 8; // keep it !!!
		B  += 8; // keep it !!!

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A0 += bs*((ncl-kadd%ncl)%ncl);
			B  += bs*((ncl-kadd%ncl)%ncl);
/*printf("\nk0 = %d\n", (ncl-kadd%ncl)%ncl);*/
			}
		// prefetch
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[0] );
		a_07 = _mm256_load_ps( &A0[0] );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );
		}

	for(k=0; k<ksub-3; k+=4) // correction in cholesky is multiple of block size 4
		{

		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[8] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[8] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[16] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[16] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );

		
		
		temp = _mm256_mul_ps( a_07, b_0 );
		B_03 = _mm256_broadcast_ps( (__m128 *) &B[24] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 85 );
		
		temp = _mm256_mul_ps( a_07, b_0 );
		A_07 = _mm256_load_ps( &A0[24] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 170 );

		temp = _mm256_mul_ps( a_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 255 );
	
		temp = _mm256_mul_ps( a_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 0 );

		
		
		temp = _mm256_mul_ps( A_07, b_0 );
		b_03 = _mm256_broadcast_ps( (__m128 *) &B[32] );
		c_00 = _mm256_sub_ps( c_00, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 85 );
		
		temp = _mm256_mul_ps( A_07, b_0 );
		a_07 = _mm256_load_ps( &A0[32] );
		c_01 = _mm256_sub_ps( c_01, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 170 );

		temp = _mm256_mul_ps( A_07, b_0 );
		c_02 = _mm256_sub_ps( c_02, temp );
		b_0 = _mm256_shuffle_ps( B_03, B_03, 255 );
	
		temp = _mm256_mul_ps( A_07, b_0 );
		c_03 = _mm256_sub_ps( c_03, temp );
		b_0 = _mm256_shuffle_ps( b_03, b_03, 0 );


		A0 += 32;
		B  += 32;

		}

	__m256
		d_00, d_01, d_02, d_03;

	d_00 = _mm256_load_ps( &C0[0+ldc*0] );
	d_00 = _mm256_add_ps( d_00, c_00 );
	d_01 = _mm256_load_ps( &C0[0+ldc*1] );
	d_01 = _mm256_add_ps( d_01, c_01 );
	d_02 = _mm256_load_ps( &C0[0+ldc*2] );
	d_02 = _mm256_add_ps( d_02, c_02 );
	d_03 = _mm256_load_ps( &C0[0+ldc*3] );
	d_03 = _mm256_add_ps( d_03, c_03 );

	// factorize the upper 4x4 matrix
	__m256
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;

	// first row
	a_00 = _mm256_broadcast_ss( &fact[0] );
	d_00  = _mm256_mul_ps( d_00, a_00 );
	_mm256_store_ps( &D0[0+ldc*0], d_00 ); // a_00
		
	// second row
	a_10 = _mm256_broadcast_ss( &fact[1] );
	a_11 = _mm256_broadcast_ss( &fact[2] );
	temp  = _mm256_mul_ps( d_00, a_10 );
	d_01  = _mm256_sub_ps( d_01, temp );
	d_01  = _mm256_mul_ps( d_01, a_11 );
	_mm256_store_ps( &D0[0+ldc*1], d_01 ); // a_00

	// third row
	a_20 = _mm256_broadcast_ss( &fact[3] );
	a_21 = _mm256_broadcast_ss( &fact[4] );
	a_22 = _mm256_broadcast_ss( &fact[5] );
	temp  = _mm256_mul_ps( d_00, a_20 );
	d_02  = _mm256_sub_ps( d_02, temp );
	temp  = _mm256_mul_ps( d_01, a_21 );
	d_02  = _mm256_sub_ps( d_02, temp );
	d_02  = _mm256_mul_ps( d_02, a_22 );
	_mm256_store_ps( &D0[0+ldc*2], d_02 ); // a_00

	// fourth row
	a_30 = _mm256_broadcast_ss( &fact[6] );
	a_31 = _mm256_broadcast_ss( &fact[7] );
	a_32 = _mm256_broadcast_ss( &fact[8] );
	a_33 = _mm256_broadcast_ss( &fact[9] );
	temp  = _mm256_mul_ps( d_00, a_30 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_01, a_31 );
	d_03  = _mm256_sub_ps( d_03, temp );
	temp  = _mm256_mul_ps( d_02, a_32 );
	d_03  = _mm256_sub_ps( d_03, temp );
	d_03  = _mm256_mul_ps( d_03, a_33 );
	_mm256_store_ps( &D0[0+ldc*3], d_03 ); // a_00

	}

