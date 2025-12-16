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

//#include "../../include/block_size.h" // TODO remove when not needed any longer



#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
/*#include <smmintrin.h>  // SSE4*/
//#include <immintrin.h>  // AVX



void kernel_dtrsm_nn_ll_diag_4x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		e_1, e_2, e_3,
		d_00=0, d_01=0, d_02=0, d_03=0,
		d_10=0, d_11=0, d_12=0, d_13=0,
		d_20=0, d_21=0, d_22=0, d_23=0,
		d_30=0, d_31=0, d_32=0, d_33=0;
		
	if(kmax<=0)
		goto add;

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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];
		d_20 += C[2+bs*0];
		d_30 += C[3+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		d_21 += C[2+bs*1];
		d_31 += C[3+bs*1];

		d_02 += C[0+bs*2];
		d_12 += C[1+bs*2];
		d_22 += C[2+bs*2];
		d_32 += C[3+bs*2];

		d_03 += C[0+bs*3];
		d_13 += C[1+bs*3];
		d_23 += C[2+bs*3];
		d_33 += C[3+bs*3];
		}

	// solution

	D[0+bs*0] = d_00;
	D[0+bs*1] = d_01;
	D[0+bs*2] = d_02;
	D[0+bs*3] = d_03;

	e_1 = E[1+bs*0];
	e_2 = E[2+bs*0];
	e_3 = E[3+bs*0];
	d_10 -= e_1 * d_00;
	d_20 -= e_2 * d_00;
	d_30 -= e_3 * d_00;
	d_11 -= e_1 * d_01;
	d_21 -= e_2 * d_01;
	d_31 -= e_3 * d_01;
	d_12 -= e_1 * d_02;
	d_22 -= e_2 * d_02;
	d_32 -= e_3 * d_02;
	d_13 -= e_1 * d_03;
	d_23 -= e_2 * d_03;
	d_33 -= e_3 * d_03;
	D[1+bs*0] = d_10;
	D[1+bs*1] = d_11;
	D[1+bs*2] = d_12;
	D[1+bs*3] = d_13;

	e_2 = E[2+bs*1];
	e_3 = E[3+bs*1];
	d_20 -= e_2 * d_10;
	d_30 -= e_3 * d_10;
	d_21 -= e_2 * d_11;
	d_31 -= e_3 * d_11;
	d_22 -= e_2 * d_12;
	d_32 -= e_3 * d_12;
	d_23 -= e_2 * d_13;
	d_33 -= e_3 * d_13;
	D[2+bs*0] = d_20;
	D[2+bs*1] = d_21;
	D[2+bs*2] = d_22;
	D[2+bs*3] = d_23;

	e_3 = E[3+bs*2];
	d_30 -= e_3 * d_20;
	d_31 -= e_3 * d_21;
	d_32 -= e_3 * d_22;
	d_33 -= e_3 * d_23;
	D[3+bs*0] = d_30;
	D[3+bs*1] = d_31;
	D[3+bs*2] = d_32;
	D[3+bs*3] = d_33;

	return;

	}



void kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		e_1, e_2, e_3,
		d_00=0, d_01=0, d_02=0, d_03=0,
		d_10=0, d_11=0, d_12=0, d_13=0,
		d_20=0, d_21=0, d_22=0, d_23=0,
		d_30=0, d_31=0, d_32=0, d_33=0;
		
	if(kmax<=0)
		goto add;

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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];
		d_20 += C[2+bs*0];
		d_30 += C[3+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		d_21 += C[2+bs*1];
		d_31 += C[3+bs*1];

		d_02 += C[0+bs*2];
		d_12 += C[1+bs*2];
		d_22 += C[2+bs*2];
		d_32 += C[3+bs*2];

		d_03 += C[0+bs*3];
		d_13 += C[1+bs*3];
		d_23 += C[2+bs*3];
		d_33 += C[3+bs*3];
		}

	// solution

	D[0+bs*0] = d_00;
	D[0+bs*1] = d_01;
	D[0+bs*2] = d_02;
	if(kn>=4)
		D[0+bs*3] = d_03;

	e_1 = E[1+bs*0];
	e_2 = E[2+bs*0];
	e_3 = E[3+bs*0];
	d_10 -= e_1 * d_00;
	d_20 -= e_2 * d_00;
	d_30 -= e_3 * d_00;
	d_11 -= e_1 * d_01;
	d_21 -= e_2 * d_01;
	d_31 -= e_3 * d_01;
	d_12 -= e_1 * d_02;
	d_22 -= e_2 * d_02;
	d_32 -= e_3 * d_02;
	d_13 -= e_1 * d_03;
	d_23 -= e_2 * d_03;
	d_33 -= e_3 * d_03;
	D[1+bs*0] = d_10;
	D[1+bs*1] = d_11;
	D[1+bs*2] = d_12;
	if(kn>=4)
		D[1+bs*3] = d_13;

	e_2 = E[2+bs*1];
	e_3 = E[3+bs*1];
	d_20 -= e_2 * d_10;
	d_30 -= e_3 * d_10;
	d_21 -= e_2 * d_11;
	d_31 -= e_3 * d_11;
	d_22 -= e_2 * d_12;
	d_32 -= e_3 * d_12;
	d_23 -= e_2 * d_13;
	d_33 -= e_3 * d_13;
	D[2+bs*0] = d_20;
	D[2+bs*1] = d_21;
	D[2+bs*2] = d_22;
	if(kn>=4)
		D[2+bs*3] = d_23;

	if(km<4)
		return;

	e_3 = E[3+bs*2];
	d_30 -= e_3 * d_20;
	d_31 -= e_3 * d_21;
	d_32 -= e_3 * d_22;
	d_33 -= e_3 * d_23;
	D[3+bs*0] = d_30;
	D[3+bs*1] = d_31;
	D[3+bs*2] = d_32;
	if(kn>=4)
		D[3+bs*3] = d_33;

	return;

	}



void kernel_dtrsm_nn_ll_diag_2x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		e_1, e_2, e_3,
		d_00=0, d_01=0, d_02=0, d_03=0,
		d_10=0, d_11=0, d_12=0, d_13=0;
		
	if(kmax<=0)
		goto add;

	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];

		d_02 += C[0+bs*2];
		d_12 += C[1+bs*2];

		d_03 += C[0+bs*3];
		d_13 += C[1+bs*3];
		}

	// solution

	D[0+bs*0] = d_00;
	D[0+bs*1] = d_01;
	D[0+bs*2] = d_02;
	if(kn>=4)
		D[0+bs*3] = d_03;

	if(km<2)
		return;

	e_1 = E[1+bs*0];
	e_2 = E[2+bs*0];
	e_3 = E[3+bs*0];
	d_10 -= e_1 * d_00;
	d_11 -= e_1 * d_01;
	d_12 -= e_1 * d_02;
	d_13 -= e_1 * d_03;
	D[1+bs*0] = d_10;
	D[1+bs*1] = d_11;
	D[1+bs*2] = d_12;
	if(kn>=4)
		D[1+bs*3] = d_13;

	return;

	}



void kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		e_1, e_2, e_3,
		d_00=0, d_01=0,
		d_10=0, d_11=0,
		d_20=0, d_21=0,
		d_30=0, d_31=0;
		
	if(kmax<=0)
		goto add;

	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];
		d_20 += C[2+bs*0];
		d_30 += C[3+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		d_21 += C[2+bs*1];
		d_31 += C[3+bs*1];
		}

	// solution

	D[0+bs*0] = d_00;
	if(kn>=2)
		D[0+bs*1] = d_01;

	e_1 = E[1+bs*0];
	e_2 = E[2+bs*0];
	e_3 = E[3+bs*0];
	d_10 -= e_1 * d_00;
	d_20 -= e_2 * d_00;
	d_30 -= e_3 * d_00;
	d_11 -= e_1 * d_01;
	d_21 -= e_2 * d_01;
	d_31 -= e_3 * d_01;
	D[1+bs*0] = d_10;
	if(kn>=2)
		D[1+bs*1] = d_11;

	e_2 = E[2+bs*1];
	e_3 = E[3+bs*1];
	d_20 -= e_2 * d_10;
	d_30 -= e_3 * d_10;
	d_21 -= e_2 * d_11;
	d_31 -= e_3 * d_11;
	D[2+bs*0] = d_20;
	if(kn>=2)
		D[2+bs*1] = d_21;

	if(km<4)
		return;

	e_3 = E[3+bs*2];
	d_30 -= e_3 * d_20;
	d_31 -= e_3 * d_21;
	D[3+bs*0] = d_30;
	if(kn>=2)
		D[3+bs*1] = d_31;


	return;

	}



void kernel_dtrsm_nn_ll_diag_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1,
		b_0, b_1,
		e_1, e_2, e_3,
		d_00=0, d_01=0,
		d_10=0, d_11=0;
		
	if(kmax<=0)
		goto add;

	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		}

	// solution

	D[0+bs*0] = d_00;
	if(kn>=2)
		D[0+bs*1] = d_01;

	if(km<2)
		return;

	e_1 = E[1+bs*0];
	d_10 -= e_1 * d_00;
	d_11 -= e_1 * d_01;
	D[1+bs*0] = d_10;
	if(kn>=2)
		D[1+bs*1] = d_11;

	return;

	}



void kernel_dtrsm_nn_ru_4x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33,
		d_00=0, d_01=0, d_02=0, d_03=0,
		d_10=0, d_11=0, d_12=0, d_13=0,
		d_20=0, d_21=0, d_22=0, d_23=0,
		d_30=0, d_31=0, d_32=0, d_33=0;
		
	if(kmax<=0)
		goto add;

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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];
		d_20 += C[2+bs*0];
		d_30 += C[3+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		d_21 += C[2+bs*1];
		d_31 += C[3+bs*1];

		d_02 += C[0+bs*2];
		d_12 += C[1+bs*2];
		d_22 += C[2+bs*2];
		d_32 += C[3+bs*2];

		d_03 += C[0+bs*3];
		d_13 += C[1+bs*3];
		d_23 += C[2+bs*3];
		d_33 += C[3+bs*3];
		}
	
	if(use_inv_diag_E)
		{
		e_00 = inv_diag_E[0];
		d_00 *= e_00;
		d_10 *= e_00;
		d_20 *= e_00;
		d_30 *= e_00;
		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		e_01 = E[0+bs*1];
		e_11 = inv_diag_E[1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_21 -= d_20 * e_01;
		d_31 -= d_30 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		d_21 *= e_11;
		d_31 *= e_11;
		D[0+bs*1] = d_01;
		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;

		e_02 = E[0+bs*2];
		e_12 = E[1+bs*2];
		e_22 = inv_diag_E[2];
		d_02 -= d_00 * e_02;
		d_12 -= d_10 * e_02;
		d_22 -= d_20 * e_02;
		d_32 -= d_30 * e_02;
		d_02 -= d_01 * e_12;
		d_12 -= d_11 * e_12;
		d_22 -= d_21 * e_12;
		d_32 -= d_31 * e_12;
		d_02 *= e_22;
		d_12 *= e_22;
		d_22 *= e_22;
		d_32 *= e_22;
		D[0+bs*2] = d_02;
		D[1+bs*2] = d_12;
		D[2+bs*2] = d_22;
		D[3+bs*2] = d_32;

		e_03 = E[0+bs*3];
		e_13 = E[1+bs*3];
		e_23 = E[2+bs*3];
		e_33 = inv_diag_E[3];
		d_03 -= d_00 * e_03;
		d_13 -= d_10 * e_03;
		d_23 -= d_20 * e_03;
		d_33 -= d_30 * e_03;
		d_03 -= d_01 * e_13;
		d_13 -= d_11 * e_13;
		d_23 -= d_21 * e_13;
		d_33 -= d_31 * e_13;
		d_03 -= d_02 * e_23;
		d_13 -= d_12 * e_23;
		d_23 -= d_22 * e_23;
		d_33 -= d_32 * e_23;
		d_03 *= e_33;
		d_13 *= e_33;
		d_23 *= e_33;
		d_33 *= e_33;
		D[0+bs*3] = d_03;
		D[1+bs*3] = d_13;
		D[2+bs*3] = d_23;
		D[3+bs*3] = d_33;
		}
	else
		{
		e_00 = 1.0 / E[0+bs*0];
		d_00 *= e_00;
		d_10 *= e_00;
		d_20 *= e_00;
		d_30 *= e_00;
		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		e_01 = E[0+bs*1];
		e_11 = 1.0 / E[1+bs*1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_21 -= d_20 * e_01;
		d_31 -= d_30 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		d_21 *= e_11;
		d_31 *= e_11;
		D[0+bs*1] = d_01;
		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;

		e_02 = E[0+bs*2];
		e_12 = E[1+bs*2];
		e_22 = 1.0 / E[2+bs*2];
		d_02 -= d_00 * e_02;
		d_12 -= d_10 * e_02;
		d_22 -= d_20 * e_02;
		d_32 -= d_30 * e_02;
		d_02 -= d_01 * e_12;
		d_12 -= d_11 * e_12;
		d_22 -= d_21 * e_12;
		d_32 -= d_31 * e_12;
		d_02 *= e_22;
		d_12 *= e_22;
		d_22 *= e_22;
		d_32 *= e_22;
		D[0+bs*2] = d_02;
		D[1+bs*2] = d_12;
		D[2+bs*2] = d_22;
		D[3+bs*2] = d_32;

		e_03 = E[0+bs*3];
		e_13 = E[1+bs*3];
		e_23 = E[2+bs*3];
		e_33 = 1.0 / E[2+bs*3];
		d_03 -= d_00 * e_03;
		d_13 -= d_10 * e_03;
		d_23 -= d_20 * e_03;
		d_33 -= d_30 * e_03;
		d_03 -= d_01 * e_13;
		d_13 -= d_11 * e_13;
		d_23 -= d_21 * e_13;
		d_33 -= d_31 * e_13;
		d_03 -= d_02 * e_23;
		d_13 -= d_12 * e_23;
		d_23 -= d_22 * e_23;
		d_33 -= d_32 * e_23;
		d_03 *= e_33;
		d_13 *= e_33;
		d_23 *= e_33;
		d_33 *= e_33;
		D[0+bs*3] = d_03;
		D[1+bs*3] = d_13;
		D[2+bs*3] = d_23;
		D[3+bs*3] = d_33;
		}

	return;

	}



void kernel_dtrsm_nn_ru_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33,
		d_00=0, d_01=0, d_02=0, d_03=0,
		d_10=0, d_11=0, d_12=0, d_13=0,
		d_20=0, d_21=0, d_22=0, d_23=0,
		d_30=0, d_31=0, d_32=0, d_33=0;
		
	if(kmax<=0)
		goto add;

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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;
		d_22 -= a_2 * b_2;
		d_32 -= a_3 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		d_23 -= a_2 * b_3;
		d_33 -= a_3 * b_3;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];
		d_20 += C[2+bs*0];
		d_30 += C[3+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		d_21 += C[2+bs*1];
		d_31 += C[3+bs*1];

		d_02 += C[0+bs*2];
		d_12 += C[1+bs*2];
		d_22 += C[2+bs*2];
		d_32 += C[3+bs*2];

		d_03 += C[0+bs*3];
		d_13 += C[1+bs*3];
		d_23 += C[2+bs*3];
		d_33 += C[3+bs*3];
		}
	
	if(use_inv_diag_E)
		{
		e_00 = inv_diag_E[0];
		d_00 *= e_00;
		d_10 *= e_00;
		d_20 *= e_00;
		d_30 *= e_00;
		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		if(km>=4)
			D[3+bs*0] = d_30;

		e_01 = E[0+bs*1];
		e_11 = inv_diag_E[1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_21 -= d_20 * e_01;
		d_31 -= d_30 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		d_21 *= e_11;
		d_31 *= e_11;
		D[0+bs*1] = d_01;
		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		if(km>=4)
			D[3+bs*1] = d_31;

		e_02 = E[0+bs*2];
		e_12 = E[1+bs*2];
		e_22 = inv_diag_E[2];
		d_02 -= d_00 * e_02;
		d_12 -= d_10 * e_02;
		d_22 -= d_20 * e_02;
		d_32 -= d_30 * e_02;
		d_02 -= d_01 * e_12;
		d_12 -= d_11 * e_12;
		d_22 -= d_21 * e_12;
		d_32 -= d_31 * e_12;
		d_02 *= e_22;
		d_12 *= e_22;
		d_22 *= e_22;
		d_32 *= e_22;
		D[0+bs*2] = d_02;
		D[1+bs*2] = d_12;
		D[2+bs*2] = d_22;
		if(km>=4)
			D[3+bs*2] = d_32;

		if(kn<4)
			return;

		e_03 = E[0+bs*3];
		e_13 = E[1+bs*3];
		e_23 = E[2+bs*3];
		e_33 = inv_diag_E[3];
		d_03 -= d_00 * e_03;
		d_13 -= d_10 * e_03;
		d_23 -= d_20 * e_03;
		d_33 -= d_30 * e_03;
		d_03 -= d_01 * e_13;
		d_13 -= d_11 * e_13;
		d_23 -= d_21 * e_13;
		d_33 -= d_31 * e_13;
		d_03 -= d_02 * e_23;
		d_13 -= d_12 * e_23;
		d_23 -= d_22 * e_23;
		d_33 -= d_32 * e_23;
		d_03 *= e_33;
		d_13 *= e_33;
		d_23 *= e_33;
		d_33 *= e_33;
		D[0+bs*3] = d_03;
		D[1+bs*3] = d_13;
		D[2+bs*3] = d_23;
		if(km>=4)
			D[3+bs*3] = d_33;
		}
	else
		{
		e_00 = 1.0 / E[0+bs*0];
		d_00 *= e_00;
		d_10 *= e_00;
		d_20 *= e_00;
		d_30 *= e_00;
		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		if(km>=4)
			D[3+bs*0] = d_30;

		e_01 = E[0+bs*1];
		e_11 = 1.0 / E[1+bs*1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_21 -= d_20 * e_01;
		d_31 -= d_30 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		d_21 *= e_11;
		d_31 *= e_11;
		D[0+bs*1] = d_01;
		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		if(km>=4)
			D[3+bs*1] = d_31;

		e_02 = E[0+bs*2];
		e_12 = E[1+bs*2];
		e_22 = 1.0 / E[2+bs*2];
		d_02 -= d_00 * e_02;
		d_12 -= d_10 * e_02;
		d_22 -= d_20 * e_02;
		d_32 -= d_30 * e_02;
		d_02 -= d_01 * e_12;
		d_12 -= d_11 * e_12;
		d_22 -= d_21 * e_12;
		d_32 -= d_31 * e_12;
		d_02 *= e_22;
		d_12 *= e_22;
		d_22 *= e_22;
		d_32 *= e_22;
		D[0+bs*2] = d_02;
		D[1+bs*2] = d_12;
		D[2+bs*2] = d_22;
		if(km>=4)
			D[3+bs*2] = d_32;

		if(kn<4)
			return;

		e_03 = E[0+bs*3];
		e_13 = E[1+bs*3];
		e_23 = E[2+bs*3];
		e_33 = 1.0 / E[2+bs*3];
		d_03 -= d_00 * e_03;
		d_13 -= d_10 * e_03;
		d_23 -= d_20 * e_03;
		d_33 -= d_30 * e_03;
		d_03 -= d_01 * e_13;
		d_13 -= d_11 * e_13;
		d_23 -= d_21 * e_13;
		d_33 -= d_31 * e_13;
		d_03 -= d_02 * e_23;
		d_13 -= d_12 * e_23;
		d_23 -= d_22 * e_23;
		d_33 -= d_32 * e_23;
		d_03 *= e_33;
		d_13 *= e_33;
		d_23 *= e_33;
		d_33 *= e_33;
		D[0+bs*3] = d_03;
		D[1+bs*3] = d_13;
		D[2+bs*3] = d_23;
		if(km>=4)
			D[3+bs*3] = d_33;
		}

	return;

	}



void kernel_dtrsm_nn_ru_2x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		e_00, e_01, e_02, e_03,
		      e_11, e_12, e_13,
			        e_22, e_23,
					      e_33,
		d_00=0, d_01=0, d_02=0, d_03=0,
		d_10=0, d_11=0, d_12=0, d_13=0;
		
	if(kmax<=0)
		goto add;

	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;

		d_02 -= a_0 * b_2;
		d_12 -= a_1 * b_2;

		d_03 -= a_0 * b_3;
		d_13 -= a_1 * b_3;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];

		d_02 += C[0+bs*2];
		d_12 += C[1+bs*2];

		d_03 += C[0+bs*3];
		d_13 += C[1+bs*3];
		}
	
	if(use_inv_diag_E)
		{
		e_00 = inv_diag_E[0];
		d_00 *= e_00;
		d_10 *= e_00;
		D[0+bs*0] = d_00;
		if(km>=2)
			D[1+bs*0] = d_10;

		e_01 = E[0+bs*1];
		e_11 = inv_diag_E[1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		D[0+bs*1] = d_01;
		if(km>=2)
			D[1+bs*1] = d_11;

		e_02 = E[0+bs*2];
		e_12 = E[1+bs*2];
		e_22 = inv_diag_E[2];
		d_02 -= d_00 * e_02;
		d_12 -= d_10 * e_02;
		d_02 -= d_01 * e_12;
		d_12 -= d_11 * e_12;
		d_02 *= e_22;
		d_12 *= e_22;
		D[0+bs*2] = d_02;
		if(km>=2)
			D[1+bs*2] = d_12;

		if(kn<4)
			return;

		e_03 = E[0+bs*3];
		e_13 = E[1+bs*3];
		e_23 = E[2+bs*3];
		e_33 = inv_diag_E[3];
		d_03 -= d_00 * e_03;
		d_13 -= d_10 * e_03;
		d_03 -= d_01 * e_13;
		d_13 -= d_11 * e_13;
		d_03 -= d_02 * e_23;
		d_13 -= d_12 * e_23;
		d_03 *= e_33;
		d_13 *= e_33;
		D[0+bs*3] = d_03;
		if(km>=2)
			D[1+bs*3] = d_13;
		}
	else
		{
		e_00 = 1.0 / E[0+bs*0];
		d_00 *= e_00;
		d_10 *= e_00;
		D[0+bs*0] = d_00;
		if(km>=2)
			D[1+bs*0] = d_10;

		e_01 = E[0+bs*1];
		e_11 = 1.0 / E[1+bs*1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		D[0+bs*1] = d_01;
		if(km>=2)
			D[1+bs*1] = d_11;

		e_02 = E[0+bs*2];
		e_12 = E[1+bs*2];
		e_22 = 1.0 / E[2+bs*2];
		d_02 -= d_00 * e_02;
		d_12 -= d_10 * e_02;
		d_02 -= d_01 * e_12;
		d_12 -= d_11 * e_12;
		d_02 *= e_22;
		d_12 *= e_22;
		D[0+bs*2] = d_02;
		if(km>=2)
			D[1+bs*2] = d_12;

		if(kn<4)
			return;

		e_03 = E[0+bs*3];
		e_13 = E[1+bs*3];
		e_23 = E[2+bs*3];
		e_33 = 1.0 / E[2+bs*3];
		d_03 -= d_00 * e_03;
		d_13 -= d_10 * e_03;
		d_03 -= d_01 * e_13;
		d_13 -= d_11 * e_13;
		d_03 -= d_02 * e_23;
		d_13 -= d_12 * e_23;
		d_03 *= e_33;
		d_13 *= e_33;
		D[0+bs*3] = d_03;
		if(km>=2)
			D[1+bs*3] = d_13;
		}

	return;

	}



void kernel_dtrsm_nn_ru_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		e_00, e_01,
		      e_11,
		d_00=0, d_01=0,
		d_10=0, d_11=0,
		d_20=0, d_21=0,
		d_30=0, d_31=0;
		
	if(kmax<=0)
		goto add;

	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;
		
		
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
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;
		d_20 -= a_2 * b_0;
		d_30 -= a_3 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		d_21 -= a_2 * b_1;
		d_31 -= a_3 * b_1;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];
		d_20 += C[2+bs*0];
		d_30 += C[3+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		d_21 += C[2+bs*1];
		d_31 += C[3+bs*1];
		}
	
	if(use_inv_diag_E)
		{
		e_00 = inv_diag_E[0];
		d_00 *= e_00;
		d_10 *= e_00;
		d_20 *= e_00;
		d_30 *= e_00;
		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		if(km>=4)
			D[3+bs*0] = d_30;

		if(kn<2)
			return;

		e_01 = E[0+bs*1];
		e_11 = inv_diag_E[1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_21 -= d_20 * e_01;
		d_31 -= d_30 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		d_21 *= e_11;
		d_31 *= e_11;
		D[0+bs*1] = d_01;
		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		if(km>=4)
			D[3+bs*1] = d_31;

		}
	else
		{
		e_00 = 1.0 / E[0+bs*0];
		d_00 *= e_00;
		d_10 *= e_00;
		d_20 *= e_00;
		d_30 *= e_00;
		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		if(km>=4)
			D[3+bs*0] = d_30;

		if(kn<2)
			return;

		e_01 = E[0+bs*1];
		e_11 = 1.0 / E[1+bs*1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_21 -= d_20 * e_01;
		d_31 -= d_30 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		d_21 *= e_11;
		d_31 *= e_11;
		D[0+bs*1] = d_01;
		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		if(km>=4)
			D[3+bs*1] = d_31;

		}

	return;

	}



void kernel_dtrsm_nn_ru_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1,
		b_0, b_1,
		e_00, e_01,
		      e_11,
		d_00=0, d_01=0,
		d_10=0, d_11=0;
		
	if(kmax<=0)
		goto add;

	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		d_00 -= a_0 * b_0;
		d_10 -= a_1 * b_0;

		d_01 -= a_0 * b_1;
		d_11 -= a_1 * b_1;


		A += 4;
		B += 1;

		}
		
	add:

	if(alg!=0)
		{
		d_00 += C[0+bs*0];
		d_10 += C[1+bs*0];

		d_01 += C[0+bs*1];
		d_11 += C[1+bs*1];
		}
	
	if(use_inv_diag_E)
		{
		e_00 = inv_diag_E[0];
		d_00 *= e_00;
		d_10 *= e_00;
		D[0+bs*0] = d_00;
		if(km>=2)
			D[1+bs*0] = d_10;

		if(kn<2)
			return;

		e_01 = E[0+bs*1];
		e_11 = inv_diag_E[1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		D[0+bs*1] = d_01;
		if(km>=2)
			D[1+bs*1] = d_11;

		}
	else
		{
		e_00 = 1.0 / E[0+bs*0];
		d_00 *= e_00;
		d_10 *= e_00;
		D[0+bs*0] = d_00;
		if(km>=2)
			D[1+bs*0] = d_10;

		if(kn<2)
			return;

		e_01 = E[0+bs*1];
		e_11 = 1.0 / E[1+bs*1];
		d_01 -= d_00 * e_01;
		d_11 -= d_10 * e_01;
		d_01 *= e_11;
		d_11 *= e_11;
		D[0+bs*1] = d_01;
		if(km>=2)
			D[1+bs*1] = d_11;

		}

	return;

	}




// new kernels



void kernel_dtrsm_nt_4x4_lib4_new(int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{
	
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %5, %%rax         \n\t" // load address of A
		"movq          %6, %%rbx         \n\t" // load address of B
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
		"movl   %0, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check ki_sub via logical AND.
		"je     .DPOSTACC_2                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB_2:                      \n\t" // main loop 2
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB_2                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_2:                      \n\t"
		"                                \n\t"
		"                                \n\t"
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
		"                                \n\t"
		"                                \n\t"
		"movl      %4, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE_2                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %1, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
		"movaps  64(%%rax), %%xmm2       \n\t"
		"movaps  96(%%rax), %%xmm3       \n\t"
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
		"addpd  %%xmm2, %%xmm11          \n\t"
		"addpd  %%xmm3, %%xmm10          \n\t"
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE_2:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movl   %7, %%ecx               \n\t" // load use_diag_E
		"testl     %%ecx, %%ecx          \n\t" 
		"je        .DNO_INV_E_2          \n\t" // do not use inv_diag_E
		"                                \n\t"
		"                                \n\t" // use inv_diag_E
		"                                \n\t"
		"movq   %3, %%rax                \n\t" // load address of E
		"movq   %8, %%r8                \n\t" // load address of inv_diag_E
		"movq   %2, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup (%%r8), %%xmm0          \n\t" // load inv_diag_E[0]
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13         \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)         \n\t"
		"movaps	%%xmm13, 16(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0        \n\t" // load E[1+bs*0]
		"movddup 8(%%r8), %%xmm1         \n\t" // load inv_diag_E[1]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12         \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12         \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)       \n\t"
		"movaps	%%xmm12, 48(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 16(%%rax), %%xmm0       \n\t" // load E[2+bs*0]
		"movddup 48(%%rax), %%xmm1       \n\t" // load E[2+bs*1]
		"movddup 16(%%r8), %%xmm2        \n\t" // load inv_diag_E[2]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"jl      .END1_2                   \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load E[3+bs*0]
		"movddup 56(%%rax), %%xmm1       \n\t" // load E[3+bs*1]
		"movddup 88(%%rax), %%xmm2       \n\t" // load E[3+bs*2]
		"movddup 24(%%r8), %%xmm3        \n\t" // load inv_diag_E[3]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2         \n\t"
		"mulpd   %%xmm15, %%xmm4         \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"movaps	%%xmm14, 112(%%rbx)      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp .END1_2                     \n\t"
		"                                \n\t"
		".DNO_INV_E_2:                   \n\t" // not use inv_diag_E
		"                                \n\t"
		"movsd   .DONE_2(%%rip), %%xmm6  \n\t" // 1.0
		"                                \n\t"
		"movq   %3, %%rax                \n\t" // load address of E
		"movq   %2, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"                                \n\t"
		"movddup (%%rax), %%xmm0         \n\t" // load E[0+bs*0]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movddup %%xmm5, %%xmm0          \n\t" 
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13         \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)         \n\t"
		"movaps	%%xmm13, 16(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0        \n\t" // load E[1+bs*0]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movddup 40(%%rax), %%xmm1       \n\t" // load E[1+bs*1]
		"movddup %%xmm5, %%xmm0          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12         \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12         \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)       \n\t"
		"movaps	%%xmm12, 48(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 16(%%rax), %%xmm0       \n\t" // load E[2+bs*0]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movddup 48(%%rax), %%xmm1       \n\t" // load E[2+bs*1]
		"movddup 80(%%rax), %%xmm2       \n\t" // load E[2+bs*2]
		"movddup %%xmm5, %%xmm0          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"jl      .END1_2                   \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load E[3+bs*0]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movddup 56(%%rax), %%xmm1       \n\t" // load E[3+bs*1]
		"movddup 88(%%rax), %%xmm2       \n\t" // load E[3+bs*2]
		"movddup 120(%%rax), %%xmm3       \n\t" // load E[3+bs*3]
		"movddup %%xmm5, %%xmm0          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2         \n\t"
		"mulpd   %%xmm15, %%xmm4         \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"movaps	%%xmm14, 112(%%rbx)      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .END1_2                    \n\t"
		".align 16                       \n\t"
		".DONE_2:                          \n\t"
		".long  0                        \n\t"
		".long  1072693248               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".END1_2:                        \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_sub),		// %0
		  "m" (C),			// %1
		  "m" (D),			// %2
		  "m" (E),			// %3
		  "m" (alg),		// %4
		  "m" (Am),			// %5
		  "m" (Bm),			// %6
		  "m" (use_inv_diag_E),	// %7
		  "m" (inv_diag_E)	// %8
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dgemm_dtrsm_nt_4x4_lib4_new(int kadd, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{
	
	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A
		"movq          %4, %%rbx         \n\t" // load address of B
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
		"movl      %0, %%esi             \n\t"
		"movl      %1, %%ecx             \n\t"
		"addl      %%esi, %%ecx          \n\t"
		"testl     %%ecx, %%ecx          \n\t" 
		"je        .DCONSIDERSUB_1         \n\t" // kadd = 0
		"                                \n\t" // else kadd > 0
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DCONSIDERADD_1            \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD_1:                      \n\t" // MAIN LOOP add
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
		"jne    .DLOOPADD_1                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD_1:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DCONSIDERSUB_1            \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT_1:                     \n\t" // EDGE LOOP
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
		"jne    .DLOOPLEFT_1               \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_1:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_NOCLEAN_1:          \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check ki_sub via logical AND.
		"je     .DPOSTACC_1                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %9,  %%rax               \n\t"
		"movq   %10, %%rbx               \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB_1:                      \n\t" // main loop 2
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB_1                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_1:                      \n\t"
		"                                \n\t"
		"                                \n\t"
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
		"                                \n\t"
		"                                \n\t"
		"movl      %8, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE_1                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
		"movaps  64(%%rax), %%xmm2       \n\t"
		"movaps  96(%%rax), %%xmm3       \n\t"
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
		"addpd  %%xmm2, %%xmm11          \n\t"
		"addpd  %%xmm3, %%xmm10          \n\t"
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE_1:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movl   %11, %%ecx               \n\t" // load use_diag_E
		"testl     %%ecx, %%ecx          \n\t" 
		"je        .DNO_INV_E_1          \n\t" // do not use inv_diag_E
		"                                \n\t"
		"                                \n\t" // use inv_diag_E
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of E
		"movq   %12, %%r8                \n\t" // load address of inv_diag_E
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup (%%r8), %%xmm0          \n\t" // load inv_diag_E[0]
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13         \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)         \n\t"
		"movaps	%%xmm13, 16(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0        \n\t" // load E[1+bs*0]
		"movddup 8(%%r8), %%xmm1         \n\t" // load inv_diag_E[1]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12         \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12         \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)       \n\t"
		"movaps	%%xmm12, 48(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 16(%%rax), %%xmm0       \n\t" // load E[2+bs*0]
		"movddup 48(%%rax), %%xmm1       \n\t" // load E[2+bs*1]
		"movddup 16(%%r8), %%xmm2        \n\t" // load inv_diag_E[2]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl      $4,  %%edx            \n\t"
//		"jl      .END1_1                   \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load E[3+bs*0]
		"movddup 56(%%rax), %%xmm1       \n\t" // load E[3+bs*1]
		"movddup 88(%%rax), %%xmm2       \n\t" // load E[3+bs*2]
		"movddup 24(%%r8), %%xmm3        \n\t" // load inv_diag_E[3]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
//		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2         \n\t"
		"mulpd   %%xmm15, %%xmm4         \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"movaps	%%xmm14, 112(%%rbx)      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp .END1_1                     \n\t"
		"                                \n\t"
		".DNO_INV_E_1:                   \n\t" // not use inv_diag_E
		"                                \n\t"
		"movsd   .DONE_1(%%rip), %%xmm6  \n\t" // 1.0
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of E
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"                                \n\t"
		"movddup (%%rax), %%xmm0         \n\t" // load E[0+bs*0]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movddup %%xmm5, %%xmm0          \n\t" 
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13         \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)         \n\t"
		"movaps	%%xmm13, 16(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0        \n\t" // load E[1+bs*0]
		"movddup 40(%%rax), %%xmm1       \n\t" // load E[1+bs*1]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm1, %%xmm5          \n\t" // 1.0/a_11
		"movddup %%xmm5, %%xmm1          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12         \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12         \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)       \n\t"
		"movaps	%%xmm12, 48(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 16(%%rax), %%xmm0       \n\t" // load E[2+bs*0]
		"movddup 48(%%rax), %%xmm1       \n\t" // load E[2+bs*1]
		"movddup 80(%%rax), %%xmm2       \n\t" // load E[2+bs*2]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm2, %%xmm5          \n\t" // 1.0/a_22
		"movddup %%xmm5, %%xmm2          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"jl      .END1_1                   \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load E[3+bs*0]
		"movddup 56(%%rax), %%xmm1       \n\t" // load E[3+bs*1]
		"movddup 88(%%rax), %%xmm2       \n\t" // load E[3+bs*2]
		"movddup 120(%%rax), %%xmm3       \n\t" // load E[3+bs*3]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm3, %%xmm5          \n\t" // 1.0/e_33
		"movddup %%xmm5, %%xmm3          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2         \n\t"
		"mulpd   %%xmm15, %%xmm4         \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"movaps	%%xmm14, 112(%%rbx)      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .END1_1                    \n\t"
		".align 16                       \n\t"
		".DONE_1:                          \n\t"
		".long  0                        \n\t"
		".long  1072693248               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".END1_1:                        \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (Ap),			// %3
		  "m" (Bp),			// %4
		  "m" (C),			// %5
		  "m" (D),			// %6
		  "m" (E),			// %7
		  "m" (alg),		// %8
		  "m" (Am),			// %9
		  "m" (Bm),			// %10
		  "m" (use_inv_diag_E),	// %11
		  "m" (inv_diag_E)	// %12
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{
	
	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A
		"movq          %4, %%rbx         \n\t" // load address of B
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
		"movl      %0, %%esi             \n\t"
		"movl      %1, %%ecx             \n\t"
		"addl      %%esi, %%ecx          \n\t"
		"testl     %%ecx, %%ecx          \n\t" 
		"je        .DCONSIDERSUB_0         \n\t" // kadd = 0
		"                                \n\t" // else kadd > 0
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %11, %%ecx            \n\t" 
		"cmpl      $1,  %%ecx            \n\t"
		"jne       .DCONSIDERLOOPADD_0     \n\t" // tri != 1
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DTRIADD_0                 \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulsd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addsd   %%xmm7, %%xmm9          \n\t"
		"mulsd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addsd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"addsd   %%xmm4, %%xmm11         \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addsd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
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
		"jmp     .DCONSIDERLOOPADD_0       \n\t"
		"                                \n\t"
		"                                \n\t"
		".DTRIADD_0:                       \n\t"
		"                                \n\t"
		"movl    %1, %%esi               \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps       16(%%rbx), %%xmm6  \n\t" // iteration 0
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm7          \n\t"
		"addsd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulsd   %%xmm0, %%xmm6          \n\t"
		"addsd   %%xmm6, %%xmm10         \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm4          \n\t"
		"addsd   %%xmm4, %%xmm11         \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $1, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"cmpl    $2, %%esi               \n\t"
		"jl     .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $2, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"je     .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"                                \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"                                \n\t"
//		"movaps       96(%%rbx), %%xmm2  \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"addsd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
//		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
//		"movaps      112(%%rax), %%xmm1  \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLOOPADD_0:              \n\t" // MAIN LOOP add
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DCONSIDERADD_0            \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD_0:                      \n\t" // MAIN LOOP add
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
		"jne    .DLOOPADD_0                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD_0:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DCONSIDERSUB_0            \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT_0:                     \n\t" // EDGE LOOP
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
		"jne    .DLOOPLEFT_0               \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_0:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_NOCLEAN_0:          \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check ki_sub via logical AND.
		"je     .DPOSTACC_0                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %9,  %%rax               \n\t"
		"movq   %10, %%rbx               \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB_0:                      \n\t" // main loop 2
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB_0                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_0:                      \n\t"
		"                                \n\t"
		"                                \n\t"
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
		"                                \n\t"
		"                                \n\t"
		"movl      %8, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE_0                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
		"movaps  64(%%rax), %%xmm2       \n\t"
		"movaps  96(%%rax), %%xmm3       \n\t"
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
		"addpd  %%xmm2, %%xmm11          \n\t"
		"addpd  %%xmm3, %%xmm10          \n\t"
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE_0:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movl   %14, %%ecx               \n\t" // load use_diag_E
		"testl     %%ecx, %%ecx          \n\t" 
		"je        .DNO_INV_E_0          \n\t" // do not use inv_diag_E
		"                                \n\t"
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of E
		"movq   %15, %%r8                \n\t" // load address of inv_diag_E
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movl   %12, %%ecx               \n\t" // load km
		"movl   %13, %%edx               \n\t" // load kn
		"                                \n\t"
		"cmpl   $4,  %%ecx               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup (%%r8), %%xmm0          \n\t" // load inv_diag_E[0]
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13         \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)         \n\t"
		"jge    .STORE_0_4_INV_0               \n\t"
		"movsd	%%xmm13, 16(%%rbx)       \n\t"
		"jmp    .STORE_0_4_INV_END_0           \n\t"
		".STORE_0_4_INV_0:                     \n\t"
		"movaps	%%xmm13, 16(%%rbx)       \n\t"
		".STORE_0_4_INV_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0        \n\t" // load E[1+bs*0]
		"movddup 8(%%r8), %%xmm1         \n\t" // load inv_diag_E[1]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12         \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12         \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)       \n\t"
		"jge    .STORE_1_4_INV_0               \n\t"
		"movsd	%%xmm12, 48(%%rbx)       \n\t"
		"jmp    .STORE_1_4_INV_END_0           \n\t"
		".STORE_1_4_INV_0:                     \n\t"
		"movaps	%%xmm12, 48(%%rbx)       \n\t"
		".STORE_1_4_INV_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 16(%%rax), %%xmm0       \n\t" // load E[2+bs*0]
		"movddup 48(%%rax), %%xmm1       \n\t" // load E[2+bs*1]
		"movddup 16(%%r8), %%xmm2        \n\t" // load inv_diag_E[2]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"jge    .STORE_2_4_INV_0               \n\t"
		"movsd	%%xmm15, 80(%%rbx)       \n\t"
		"jmp    .STORE_2_4_INV_END_0           \n\t"
		".STORE_2_4_INV_0:                     \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		".STORE_2_4_INV_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"jl      .END1_0                   \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load E[3+bs*0]
		"movddup 56(%%rax), %%xmm1       \n\t" // load E[3+bs*1]
		"movddup 88(%%rax), %%xmm2       \n\t" // load E[3+bs*2]
		"movddup 24(%%r8), %%xmm3        \n\t" // load inv_diag_E[3]
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2         \n\t"
		"mulpd   %%xmm15, %%xmm4         \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"jge    .STORE_3_4_INV_0               \n\t"
		"movsd	%%xmm14, 112(%%rbx)      \n\t"
		"jmp    .STORE_3_4_INV_END_0           \n\t"
		".STORE_3_4_INV_0:                     \n\t"
		"movaps	%%xmm14, 112(%%rbx)      \n\t"
		".STORE_3_4_INV_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp .END1_0                     \n\t"
		"                                \n\t"
		".DNO_INV_E_0:                   \n\t" // use inv_diag_E
		"                                \n\t"
		"movsd   .DONE_0(%%rip), %%xmm6  \n\t" // 1.0
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of E
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movl   %12, %%ecx               \n\t" // load km
		"movl   %13, %%edx               \n\t" // load kn
		"                                \n\t"
		"cmpl   $4,  %%ecx               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup (%%rax), %%xmm0         \n\t" // load E[0+bs*0]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movddup %%xmm5, %%xmm0          \n\t" 
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13         \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)         \n\t"
		"jge    .STORE_0_4_0               \n\t"
		"movsd	%%xmm13, 16(%%rbx)       \n\t"
		"jmp    .STORE_0_4_END_0           \n\t"
		".STORE_0_4_0:                     \n\t"
		"movaps	%%xmm13, 16(%%rbx)       \n\t"
		".STORE_0_4_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0        \n\t" // load E[1+bs*0]
		"movddup 40(%%rax), %%xmm1       \n\t" // load E[1+bs*1]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm1, %%xmm5          \n\t" // 1.0/a_11
		"movddup %%xmm5, %%xmm1          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12         \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12         \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)       \n\t"
		"jge    .STORE_1_4_0               \n\t"
		"movsd	%%xmm12, 48(%%rbx)       \n\t"
		"jmp    .STORE_1_4_END_0           \n\t"
		".STORE_1_4_0:                     \n\t"
		"movaps	%%xmm12, 48(%%rbx)       \n\t"
		".STORE_1_4_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 16(%%rax), %%xmm0       \n\t" // load E[2+bs*0]
		"movddup 48(%%rax), %%xmm1       \n\t" // load E[2+bs*1]
		"movddup 80(%%rax), %%xmm2       \n\t" // load E[2+bs*2]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm2, %%xmm5          \n\t" // 1.0/a_22
		"movddup %%xmm5, %%xmm2          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"jge    .STORE_2_4_0               \n\t"
		"movsd	%%xmm15, 80(%%rbx)       \n\t"
		"jmp    .STORE_2_4_END_0           \n\t"
		".STORE_2_4_0:                     \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		".STORE_2_4_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"jl      .END1_0                   \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load E[3+bs*0]
		"movddup 56(%%rax), %%xmm1       \n\t" // load E[3+bs*1]
		"movddup 88(%%rax), %%xmm2       \n\t" // load E[3+bs*2]
		"movddup 120(%%rax), %%xmm3       \n\t" // load E[3+bs*3]
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm3, %%xmm5          \n\t" // 1.0/a_33
		"movddup %%xmm5, %%xmm3          \n\t" 
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2         \n\t"
		"mulpd   %%xmm15, %%xmm4         \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"jge    .STORE_3_4_0               \n\t"
		"movsd	%%xmm14, 112(%%rbx)      \n\t"
		"jmp    .STORE_3_4_END_0           \n\t"
		".STORE_3_4_0:                     \n\t"
		"movaps	%%xmm14, 112(%%rbx)      \n\t"
		".STORE_3_4_END_0:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .END1_0                    \n\t"
		".align 16                       \n\t"
		".DONE_0:                          \n\t"
		".long  0                        \n\t"
		".long  1072693248               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".END1_0:                          \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (Ap),			// %3
		  "m" (Bp),			// %4
		  "m" (C),			// %5
		  "m" (D),			// %6
		  "m" (E),			// %7
		  "m" (alg),		// %8
		  "m" (Am),			// %9
		  "m" (Bm),			// %10
		  "m" (tri_A),		// %11
		  "m" (km),			// %12
		  "m" (kn),			// %13
		  "m" (use_inv_diag_E),	// %14
		  "m" (inv_diag_E)	// %15
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
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
	
	k = 0;

	if(kadd>0)
		{

		if(tri_A==1)
			{

			// initial triangle

			if(kadd>=4)
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;


				// k=1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;


				// k=2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
					
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;


				// k=3
				a_0 = Ap[0+bs*3];
				a_1 = Ap[1+bs*3];
				a_2 = Ap[2+bs*3];
				a_3 = Ap[3+bs*3];
					
				b_0 = Bp[0+bs*3];
				b_1 = Bp[1+bs*3];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;
				c_30 += a_3 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;


				Ap += 16;
				Bp += 16;
				k += 4;

				}
			else
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				k += 1;

				if(kadd>1)
					{

					// k=1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
						
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
						
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_01 += a_0 * b_1;
					c_11 += a_1 * b_1;

					k += 1;

					if(kadd>2)
						{

						// k=2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
							
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
							
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_01 += a_0 * b_1;
						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						k += 1;

						}

					}

				}

			}
		
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			a_2 = Ap[2+bs*1];
			a_3 = Ap[3+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			a_2 = Ap[2+bs*2];
			a_3 = Ap[3+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			a_2 = Ap[2+bs*3];
			a_3 = Ap[3+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		a_2 = Am[2+bs*0];
		a_3 = Am[3+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		a_2 = Am[2+bs*1];
		a_3 = Am[3+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		a_2 = Am[2+bs*2];
		a_3 = Am[3+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		a_2 = Am[2+bs*3];
		a_3 = Am[3+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];
		c_20 += C[2+bs*0];
		c_30 += C[3+bs*0];

		c_01 += C[0+bs*1];
		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];
		}
	
	// dtrsm
	double
		a_00, a_10, a_11;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		c_00 *= a_00;
		c_10 *= a_00;
		c_20 *= a_00;
		c_30 *= a_00;
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		if(km>=4)
			D[3+bs*0] = c_30;

		if(kn==1)
			return;

		a_10 = E[1+bs*0];
		a_11 = inv_diag_E[1];
		c_01 -= c_00*a_10;
		c_11 -= c_10*a_10;
		c_21 -= c_20*a_10;
		c_31 -= c_30*a_10;
		c_01 *= a_11;
		c_11 *= a_11;
		c_21 *= a_11;
		c_31 *= a_11;
		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		if(km>=4)
			D[3+bs*1] = c_31;

		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		c_00 *= a_00;
		c_10 *= a_00;
		c_20 *= a_00;
		c_30 *= a_00;
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		if(km>=4)
			D[3+bs*0] = c_30;

		if(kn==1)
			return;

		a_10 = E[1+bs*0];
		a_11 = 1.0/E[1+bs*1];
		c_01 -= c_00*a_10;
		c_11 -= c_10*a_10;
		c_21 -= c_20*a_10;
		c_31 -= c_30*a_10;
		c_01 *= a_11;
		c_11 *= a_11;
		c_21 *= a_11;
		c_31 *= a_11;
		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		if(km>=4)
			D[3+bs*1] = c_31;

		}

	}


void kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
	
	k = 0;

	if(kadd>0)
		{

		if(tri_A==1)
			{

			// initial triangle

			if(kadd>=2)
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
				b_2 = Bp[2+bs*0];
				b_3 = Bp[3+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				c_02 += a_0 * b_2;

				c_03 += a_0 * b_3;


				// k=1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				b_2 = Bp[2+bs*1];
				b_3 = Bp[3+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;

				c_02 += a_0 * b_2;
				c_12 += a_1 * b_2;

				c_03 += a_0 * b_3;
				c_13 += a_1 * b_3;


				Ap += 8;
				Bp += 8;
				k += 2;
			
				}
			else
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
				b_2 = Bp[2+bs*0];
				b_3 = Bp[3+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				c_02 += a_0 * b_2;

				c_03 += a_0 * b_3;

				k += 1;

				}

			}
			
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			b_2 = Bp[2+bs*1];
			b_3 = Bp[3+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			b_2 = Bp[2+bs*2];
			b_3 = Bp[3+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			b_2 = Bp[2+bs*3];
			b_3 = Bp[3+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		b_2 = Bm[2+bs*0];
		b_3 = Bm[3+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		b_2 = Bm[2+bs*1];
		b_3 = Bm[3+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		b_2 = Bm[2+bs*2];
		b_3 = Bm[3+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		b_2 = Bm[2+bs*3];
		b_3 = Bm[3+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_01 += C[0+bs*1];
		c_11 += C[1+bs*1];

		c_02 += C[0+bs*2];
		c_12 += C[1+bs*2];

		c_03 += C[0+bs*3];
		c_13 += C[1+bs*3];
		}
	
	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		c_00 *= a_00;
		c_10 *= a_00;
		D[0+bs*0] = c_00;
		if(km>=2)
			D[1+bs*0] = c_10;

		a_10 = E[1+bs*0];
		a_11 = inv_diag_E[1];
		c_01 -= c_00*a_10;
		c_11 -= c_10*a_10;
		c_01 *= a_11;
		c_11 *= a_11;
		D[0+bs*1] = c_01;
		if(km>=2)
			D[1+bs*1] = c_11;

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = inv_diag_E[2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		D[0+bs*2] = c_02;
		if(km>=2)
			D[1+bs*2] = c_12;

		if(kn==3)
			return;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = inv_diag_E[3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		D[0+bs*3] = c_03;
		if(km>=2)
			D[1+bs*3] = c_13;

		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		c_00 *= a_00;
		c_10 *= a_00;
		D[0+bs*0] = c_00;
		if(km>=2)
			D[1+bs*0] = c_10;

		a_10 = E[1+bs*0];
		a_11 = 1.0/E[1+bs*1];
		c_01 -= c_00*a_10;
		c_11 -= c_10*a_10;
		c_01 *= a_11;
		c_11 *= a_11;
		D[0+bs*1] = c_01;
		if(km>=2)
			D[1+bs*1] = c_11;

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = 1.0/E[2+bs*2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		D[0+bs*2] = c_02;
		if(km>=2)
			D[1+bs*2] = c_12;

		if(kn==3)
			return;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = 1.0/E[3+bs*3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		D[0+bs*3] = c_03;
		if(km>=2)
			D[1+bs*3] = c_13;

		}

	}


void kernel_dgemm_dtrsm_nt_2x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
	
	k = 0;

	if(kadd>0)
		{

		if(tri_A==1)
			{

			// initial triangle

			if(kadd>=2)
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;


				// k=1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;


				Ap += 8;
				Bp += 8;
				k += 2;

				}
			else
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				k += 1;


				}

			}
			
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_01 += C[0+bs*1];
		c_11 += C[1+bs*1];
		}
	
	// dtrsm
	double
		a_00, a_10, a_11;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		c_00 *= a_00;
		c_10 *= a_00;
		D[0+bs*0] = c_00;
		if(km>=2)
			D[1+bs*0] = c_10;

		if(kn==1)
			return;

		a_10 = E[1+bs*0];
		a_11 = inv_diag_E[1];
		c_01 -= c_00*a_10;
		c_11 -= c_10*a_10;
		c_01 *= a_11;
		c_11 *= a_11;
		D[0+bs*1] = c_01;
		if(km>=2)
			D[1+bs*1] = c_11;

		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		c_00 *= a_00;
		c_10 *= a_00;
		D[0+bs*0] = c_00;
		if(km>=2)
			D[1+bs*0] = c_10;

		if(kn==1)
			return;

		a_10 = E[1+bs*0];
		a_11 = 1.0/E[1+bs*1];
		c_01 -= c_00*a_10;
		c_11 -= c_10*a_10;
		c_01 *= a_11;
		c_11 *= a_11;
		D[0+bs*1] = c_01;
		if(km>=2)
			D[1+bs*1] = c_11;

		}

	}


// old kernels

void kernel_dgemm_dtrsm_nt_4x4_lib4(int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg)
	{
	
	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A
		"movq          %4, %%rbx         \n\t" // load address of B
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
		"movl      %0, %%esi             \n\t"
		"movl      %1, %%ecx             \n\t"
		"addl      %%esi, %%ecx          \n\t"
		"testl     %%ecx, %%ecx          \n\t" 
		"je        .DCONSIDERSUB         \n\t" // kadd = 0
		"                                \n\t" // else kadd > 0
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %11, %%ecx            \n\t" 
		"cmpl      $1,  %%ecx            \n\t"
		"jne       .DCONSIDERLOOPADD     \n\t" // tri != 1
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DTRIADD                 \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulsd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addsd   %%xmm7, %%xmm9          \n\t"
		"mulsd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addsd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"addsd   %%xmm4, %%xmm11         \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addsd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
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
		"jmp     .DCONSIDERLOOPADD       \n\t"
		"                                \n\t"
		"                                \n\t"
		".DTRIADD:                       \n\t"
		"                                \n\t"
		"movl    %1, %%esi               \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps       16(%%rbx), %%xmm6  \n\t" // iteration 0
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm7          \n\t"
		"addsd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulsd   %%xmm0, %%xmm6          \n\t"
		"addsd   %%xmm6, %%xmm10         \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm4          \n\t"
		"addsd   %%xmm4, %%xmm11         \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $1, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN    \n\t" //
		"cmpl    $2, %%esi               \n\t"
		"jl     .DCONSIDERSUB_NOCLEAN    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $2, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN    \n\t" //
		"je     .DCONSIDERSUB_NOCLEAN    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"                                \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"                                \n\t"
//		"movaps       96(%%rbx), %%xmm2  \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"addsd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
//		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
//		"movaps      112(%%rax), %%xmm1  \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DCONSIDERSUB_NOCLEAN    \n\t" //
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLOOPADD:              \n\t" // MAIN LOOP add
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DCONSIDERADD            \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD:                      \n\t" // MAIN LOOP add
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
		"jne    .DLOOPADD                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DCONSIDERSUB            \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT:                     \n\t" // EDGE LOOP
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
		"jne    .DLOOPLEFT               \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_NOCLEAN:          \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check ki_sub via logical AND.
		"je     .DPOSTACC                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %9,  %%rax               \n\t"
		"movq   %10, %%rbx               \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB:                      \n\t" // main loop 2
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC:                      \n\t"
		"                                \n\t"
		"                                \n\t"
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
		"                                \n\t"
		"                                \n\t"
		"movl      %8, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
		"movaps  64(%%rax), %%xmm2       \n\t"
		"movaps  96(%%rax), %%xmm3       \n\t"
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
		"addpd  %%xmm2, %%xmm11          \n\t"
		"addpd  %%xmm3, %%xmm10          \n\t"
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of fact
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup (%%rax), %%xmm0         \n\t" // load fact elements
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13          \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)          \n\t"
		"movaps	%%xmm13, 16(%%rbx)        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0         \n\t" // load fact elements
		"movddup 16(%%rax), %%xmm1         \n\t" // load fact elements
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4          \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12          \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12          \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)          \n\t"
		"movaps	%%xmm12, 48(%%rbx)        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load fact elements
		"movddup 32(%%rax), %%xmm1       \n\t" // load fact elements
		"movddup 40(%%rax), %%xmm2       \n\t" // load fact elements
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4          \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4          \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 48(%%rax), %%xmm0       \n\t" // load fact elements
		"movddup 56(%%rax), %%xmm1       \n\t" // load fact elements
		"movddup 64(%%rax), %%xmm2       \n\t" // load fact elements
		"movddup 72(%%rax), %%xmm3       \n\t" // load fact elements
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4          \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4          \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2          \n\t"
		"mulpd   %%xmm15, %%xmm4          \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"movaps	%%xmm14, 112(%%rbx)       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (Ap),			// %3
		  "m" (Bp),			// %4
		  "m" (C),			// %5
		  "m" (D),			// %6
		  "m" (fact),		// %7
		  "m" (alg),		// %8
		  "m" (Am),			// %9
		  "m" (Bm),			// %10
		  "m" (tri)			// %11
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dgemm_dtrsm_nt_4x4_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg)
	{
	
	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A
		"movq          %4, %%rbx         \n\t" // load address of B
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
		"movl      %0, %%esi             \n\t"
		"movl      %1, %%ecx             \n\t"
		"addl      %%esi, %%ecx          \n\t"
		"testl     %%ecx, %%ecx          \n\t" 
		"je        .DCONSIDERSUB1        \n\t" // kadd = 0
		"                                \n\t" // else kadd > 0
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %11, %%ecx            \n\t" 
		"cmpl      $1,  %%ecx            \n\t"
		"jne       .DCONSIDERLOOPADD1    \n\t" // tri != 1
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DTRIADD                 \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulsd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addsd   %%xmm7, %%xmm9          \n\t"
		"mulsd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addsd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"addsd   %%xmm4, %%xmm11         \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addsd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
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
		"jmp     .DCONSIDERLOOPADD       \n\t"
		"                                \n\t"
		"                                \n\t"
		".DTRIADD1:                      \n\t"
		"                                \n\t"
		"movl    %1, %%esi               \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps       16(%%rbx), %%xmm6  \n\t" // iteration 0
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm7          \n\t"
		"addsd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulsd   %%xmm0, %%xmm6          \n\t"
		"addsd   %%xmm6, %%xmm10         \n\t"
		"                                \n\t"
		"mulsd   %%xmm0, %%xmm4          \n\t"
		"addsd   %%xmm4, %%xmm11         \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $1, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN1    \n\t" //
		"cmpl    $2, %%esi               \n\t"
		"jl     .DCONSIDERSUB_NOCLEAN1    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $2, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN1    \n\t" //
		"je     .DCONSIDERSUB_NOCLEAN1    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"                                \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"                                \n\t"
//		"movaps       96(%%rbx), %%xmm2  \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"addsd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
//		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
//		"movaps      112(%%rax), %%xmm1  \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DCONSIDERSUB_NOCLEAN1    \n\t" //
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLOOPADD1:              \n\t" // MAIN LOOP add
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DCONSIDERADD1            \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD1:                      \n\t" // MAIN LOOP add
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
		"jne    .DLOOPADD1                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD1:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DCONSIDERSUB1            \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT1:                     \n\t" // EDGE LOOP
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
		"jne    .DLOOPLEFT1               \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB1:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm6, %%xmm10         \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"addpd   %%xmm4, %%xmm11         \n\t"
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_NOCLEAN1:          \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check ki_sub via logical AND.
		"je     .DPOSTACC1                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %9,  %%rax               \n\t"
		"movq   %10, %%rbx               \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB1:                      \n\t" // main loop 2
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB1                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm6, %%xmm10         \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"subpd   %%xmm4, %%xmm11         \n\t"
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC1:                      \n\t"
		"                                \n\t"
		"                                \n\t"
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
		"                                \n\t"
		"                                \n\t"
		"movl      %8, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE1                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
		"movaps  64(%%rax), %%xmm2       \n\t"
		"movaps  96(%%rax), %%xmm3       \n\t"
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
		"addpd  %%xmm2, %%xmm11          \n\t"
		"addpd  %%xmm3, %%xmm10          \n\t"
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE1:                       \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of fact
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movl   %12, %%ecx               \n\t" // load km
		"movl   %13, %%edx               \n\t" // load kn
		"                                \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup (%%rax), %%xmm0         \n\t" // load fact elements
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm9          \n\t"
		"mulpd   %%xmm0, %%xmm13         \n\t"
		"                                \n\t"
		"movaps	%%xmm9,  (%%rbx)         \n\t"
		"jge    .STORE_0_4               \n\t"
		"movsd	%%xmm13, 16(%%rbx)       \n\t"
		"jmp    .STORE_0_4_END           \n\t"
		".STORE_0_4:                     \n\t"
		"movaps	%%xmm13, 16(%%rbx)       \n\t"
		".STORE_0_4_END:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 8(%%rax), %%xmm0        \n\t" // load fact elements
		"movddup 16(%%rax), %%xmm1       \n\t" // load fact elements
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm4, %%xmm12         \n\t"
		"                                \n\t"
		"mulpd   %%xmm1, %%xmm8          \n\t"
		"mulpd   %%xmm1, %%xmm12         \n\t"
		"                                \n\t"
		"movaps	%%xmm8,  32(%%rbx)       \n\t"
		"jge    .STORE_1_4               \n\t"
		"movsd	%%xmm12, 48(%%rbx)       \n\t"
		"jmp    .STORE_1_4_END           \n\t"
		".STORE_1_4:                     \n\t"
		"movaps	%%xmm12, 48(%%rbx)       \n\t"
		".STORE_1_4_END:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movddup 24(%%rax), %%xmm0       \n\t" // load fact elements
		"movddup 32(%%rax), %%xmm1       \n\t" // load fact elements
		"movddup 40(%%rax), %%xmm2       \n\t" // load fact elements
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm11         \n\t"
		"subpd   %%xmm4, %%xmm15         \n\t"
		"                                \n\t"
		"mulpd   %%xmm2, %%xmm11         \n\t"
		"mulpd   %%xmm2, %%xmm15         \n\t"
		"                                \n\t"
		"movaps	%%xmm11, 64(%%rbx)       \n\t"
		"jge    .STORE_2_4               \n\t"
		"movsd	%%xmm15, 80(%%rbx)       \n\t"
		"jmp    .STORE_2_4_END           \n\t"
		".STORE_2_4:                     \n\t"
		"movaps	%%xmm15, 80(%%rbx)       \n\t"
		".STORE_2_4_END:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"jl      .END1                   \n\t"
		"                                \n\t"
		"movddup 48(%%rax), %%xmm0       \n\t" // load fact elements
		"movddup 56(%%rax), %%xmm1       \n\t" // load fact elements
		"movddup 64(%%rax), %%xmm2       \n\t" // load fact elements
		"movddup 72(%%rax), %%xmm3       \n\t" // load fact elements
		"                                \n\t"
		"movaps  %%xmm0,  %%xmm4         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm4         \n\t"
		"subpd   %%xmm0, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"                                \n\t"
		"movaps  %%xmm1,  %%xmm4         \n\t"
		"mulpd   %%xmm8, %%xmm1          \n\t"
		"mulpd   %%xmm12, %%xmm4         \n\t"
		"subpd   %%xmm1, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm2,  %%xmm4         \n\t"
		"mulpd   %%xmm11, %%xmm2         \n\t"
		"mulpd   %%xmm15, %%xmm4         \n\t"
		"subpd   %%xmm2, %%xmm10         \n\t"
		"subpd   %%xmm4, %%xmm14         \n\t"
		"                                \n\t"
		"mulpd   %%xmm3, %%xmm10         \n\t"
		"mulpd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps	%%xmm10, 96(%%rbx)       \n\t"
		"jge    .STORE_3_4               \n\t"
		"movsd	%%xmm14, 112(%%rbx)      \n\t"
		"jmp    .STORE_3_4_END           \n\t"
		".STORE_3_4:                     \n\t"
		"movaps	%%xmm14, 112(%%rbx)      \n\t"
		".STORE_3_4_END:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		".END1:                          \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (Ap),			// %3
		  "m" (Bp),			// %4
		  "m" (C),			// %5
		  "m" (D),			// %6
		  "m" (fact),		// %7
		  "m" (alg),		// %8
		  "m" (Am),			// %9
		  "m" (Bm),			// %10
		  "m" (tri),		// %11
		  "m" (km),			// %12
		  "m" (kn) 			// %13
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dgemm_dtrsm_nt_4x2_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg)
	{

	const int bs = 4;
//	const int d_ncl = D_NCL;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
	
	k = 0;

	if(kadd>0)
		{

		if(tri==1)
			{

			// initial triangle

			if(kadd>=4)
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;


				// k=1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;


				// k=2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
					
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;


				// k=3
				a_0 = Ap[0+bs*3];
				a_1 = Ap[1+bs*3];
				a_2 = Ap[2+bs*3];
				a_3 = Ap[3+bs*3];
					
				b_0 = Bp[0+bs*3];
				b_1 = Bp[1+bs*3];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;
				c_30 += a_3 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;


				Ap += 16;
				Bp += 16;
				k += 4;

				}
			else
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				k += 1;

				if(kadd>1)
					{

					// k=1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
						
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
						
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_01 += a_0 * b_1;
					c_11 += a_1 * b_1;

					k += 1;

					if(kadd>2)
						{

						// k=2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
							
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
							
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_01 += a_0 * b_1;
						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						k += 1;

						}

					}

				}

			}
		
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			a_2 = Ap[2+bs*1];
			a_3 = Ap[3+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			a_2 = Ap[2+bs*2];
			a_3 = Ap[3+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			a_2 = Ap[2+bs*3];
			a_3 = Ap[3+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		a_2 = Am[2+bs*0];
		a_3 = Am[3+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		a_2 = Am[2+bs*1];
		a_3 = Am[3+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		a_2 = Am[2+bs*2];
		a_3 = Am[3+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		a_2 = Am[2+bs*3];
		a_3 = Am[3+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];
		c_20 += C[2+bs*0];
		c_30 += C[3+bs*0];

		c_01 += C[0+bs*1];
		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];
		}
	
	// dtrsm
	double
		a_00, a_10, a_11;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	D[0+bs*0] = c_00;
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	if(km>=4)
		D[3+bs*0] = c_30;

	if(kn==1)
		return;

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
	D[0+bs*1] = c_01;
	D[1+bs*1] = c_11;
	D[2+bs*1] = c_21;
	if(km>=4)
		D[3+bs*1] = c_31;

	}
	
	
	
void kernel_dgemm_dtrsm_nt_2x4_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg)
	{

	const int bs = 4;
//	const int d_ncl = D_NCL;

	int k;

	double
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
	
	k = 0;

	if(kadd>0)
		{

		if(tri==1)
			{

			// initial triangle

			if(kadd>=2)
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
				b_2 = Bp[2+bs*0];
				b_3 = Bp[3+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				c_02 += a_0 * b_2;

				c_03 += a_0 * b_3;


				// k=1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				b_2 = Bp[2+bs*1];
				b_3 = Bp[3+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;

				c_02 += a_0 * b_2;
				c_12 += a_1 * b_2;

				c_03 += a_0 * b_3;
				c_13 += a_1 * b_3;


				Ap += 8;
				Bp += 8;
				k += 2;
			
				}
			else
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
				b_2 = Bp[2+bs*0];
				b_3 = Bp[3+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				c_02 += a_0 * b_2;

				c_03 += a_0 * b_3;

				k += 1;

				}

			}
			
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			b_2 = Bp[2+bs*1];
			b_3 = Bp[3+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			b_2 = Bp[2+bs*2];
			b_3 = Bp[3+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			b_2 = Bp[2+bs*3];
			b_3 = Bp[3+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;

			c_02 += a_0 * b_2;
			c_12 += a_1 * b_2;

			c_03 += a_0 * b_3;
			c_13 += a_1 * b_3;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		b_2 = Bm[2+bs*0];
		b_3 = Bm[3+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		b_2 = Bm[2+bs*1];
		b_3 = Bm[3+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		b_2 = Bm[2+bs*2];
		b_3 = Bm[3+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		b_2 = Bm[2+bs*3];
		b_3 = Bm[3+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_01 += C[0+bs*1];
		c_11 += C[1+bs*1];

		c_02 += C[0+bs*2];
		c_12 += C[1+bs*2];

		c_03 += C[0+bs*3];
		c_13 += C[1+bs*3];
		}
	
	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	D[0+bs*0] = c_00;
	if(km>=4)
		D[1+bs*0] = c_10;

	a_10 = fact[1];
	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	D[0+bs*1] = c_01;
	if(km>=4)
		D[1+bs*1] = c_11;

	a_20 = fact[3];
	a_21 = fact[4];
	a_22 = fact[5];
	c_02 -= c_00*a_20;
	c_12 -= c_10*a_20;
	c_02 -= c_01*a_21;
	c_12 -= c_11*a_21;
	c_02 *= a_22;
	c_12 *= a_22;
	D[0+bs*2] = c_02;
	if(km>=4)
		D[1+bs*2] = c_12;

	if(kn==3)
		return;

	a_30 = fact[6];
	a_31 = fact[7];
	a_32 = fact[8];
	a_33 = fact[9];
	c_03 -= c_00*a_30;
	c_13 -= c_10*a_30;
	c_03 -= c_01*a_31;
	c_13 -= c_11*a_31;
	c_03 -= c_02*a_32;
	c_13 -= c_12*a_32;
	c_03 *= a_33;
	c_13 *= a_33;
	D[0+bs*3] = c_03;
	if(km>=4)
		D[1+bs*3] = c_13;

	}
	
	
	
void kernel_dgemm_dtrsm_nt_2x2_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg)
	{

	const int bs = 4;
//	const int d_ncl = D_NCL;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
	
	k = 0;

	if(kadd>0)
		{

		if(tri==1)
			{

			// initial triangle

			if(kadd>=2)
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;


				// k=1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_01 += a_0 * b_1;
				c_11 += a_1 * b_1;


				Ap += 8;
				Bp += 8;
				k += 2;

				}
			else
				{

				// k=0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
				b_1 = Bp[1+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				k += 1;


				}

			}
			
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_01 += a_0 * b_1;
			c_11 += a_1 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_01 += C[0+bs*1];
		c_11 += C[1+bs*1];
		}
	
	// dtrsm
	double
		a_00, a_10, a_11;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	D[0+bs*0] = c_00;
	if(km>=2)
		D[1+bs*0] = c_10;

	if(kn==1)
		return;

	a_10 = fact[1];
	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	D[0+bs*1] = c_01;
	if(km>=2)
		D[1+bs*1] = c_11;

	}
	
	
	
