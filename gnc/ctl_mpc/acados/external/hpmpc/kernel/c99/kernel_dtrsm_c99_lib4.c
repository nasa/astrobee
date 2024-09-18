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

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;

	k = 0;

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		a_2 = Am[2+bs*0];
		a_3 = Am[3+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		b_2 = Bm[2+bs*0];
		b_3 = Bm[3+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		a_2 = Am[2+bs*1];
		a_3 = Am[3+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		b_2 = Bm[2+bs*1];
		b_3 = Bm[3+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		a_2 = Am[2+bs*2];
		a_3 = Am[3+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		b_2 = Bm[2+bs*2];
		b_3 = Bm[3+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		a_2 = Am[2+bs*3];
		a_3 = Am[3+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		b_2 = Bm[2+bs*3];
		b_3 = Bm[3+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;
		
		
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

		c_02 += C[0+bs*2];
		c_12 += C[1+bs*2];
		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_03 += C[0+bs*3];
		c_13 += C[1+bs*3];
		c_23 += C[2+bs*3];
		c_33 += C[3+bs*3];
		}
	
	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
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
		D[3+bs*0] = c_30;

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
		D[3+bs*1] = c_31;

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = inv_diag_E[2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_22 -= c_20*a_20;
		c_32 -= c_30*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_22 -= c_21*a_21;
		c_32 -= c_31*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		c_22 *= a_22;
		c_32 *= a_22;
		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = inv_diag_E[3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_23 -= c_20*a_30;
		c_33 -= c_30*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_23 -= c_21*a_31;
		c_33 -= c_31*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_23 -= c_22*a_32;
		c_33 -= c_32*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		c_23 *= a_33;
		c_33 *= a_33;
		D[0+bs*3] = c_03;
		D[1+bs*3] = c_13;
		D[2+bs*3] = c_23;
		D[3+bs*3] = c_33;

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
		D[3+bs*0] = c_30;

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
		D[3+bs*1] = c_31;

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = 1.0/E[2+bs*2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_22 -= c_20*a_20;
		c_32 -= c_30*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_22 -= c_21*a_21;
		c_32 -= c_31*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		c_22 *= a_22;
		c_32 *= a_22;
		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = 1.0/E[3+bs*3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_23 -= c_20*a_30;
		c_33 -= c_30*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_23 -= c_21*a_31;
		c_33 -= c_31*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_23 -= c_22*a_32;
		c_33 -= c_32*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		c_23 *= a_33;
		c_33 *= a_33;
		D[0+bs*3] = c_03;
		D[1+bs*3] = c_13;
		D[2+bs*3] = c_23;
		D[3+bs*3] = c_33;

		}

	}
	
	
	
void kernel_dgemm_dtrsm_nt_4x4_lib4_new(int kadd, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;

	k = 0;

	if(kadd>0)
		{

		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
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


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			a_2 = Ap[2+bs*1];
			a_3 = Ap[3+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			b_2 = Bp[2+bs*1];
			b_3 = Bp[3+bs*1];
			
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


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			a_2 = Ap[2+bs*2];
			a_3 = Ap[3+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			b_2 = Bp[2+bs*2];
			b_3 = Bp[3+bs*2];
			
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


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			a_2 = Ap[2+bs*3];
			a_3 = Ap[3+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			b_2 = Bp[2+bs*3];
			b_3 = Bp[3+bs*3];
			
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
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
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
		b_2 = Bm[2+bs*0];
		b_3 = Bm[3+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		a_2 = Am[2+bs*1];
		a_3 = Am[3+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		b_2 = Bm[2+bs*1];
		b_3 = Bm[3+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		a_2 = Am[2+bs*2];
		a_3 = Am[3+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		b_2 = Bm[2+bs*2];
		b_3 = Bm[3+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		a_2 = Am[2+bs*3];
		a_3 = Am[3+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		b_2 = Bm[2+bs*3];
		b_3 = Bm[3+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;
		
		
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

		c_02 += C[0+bs*2];
		c_12 += C[1+bs*2];
		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_03 += C[0+bs*3];
		c_13 += C[1+bs*3];
		c_23 += C[2+bs*3];
		c_33 += C[3+bs*3];
		}
	
	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
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
		D[3+bs*0] = c_30;

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
		D[3+bs*1] = c_31;

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = inv_diag_E[2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_22 -= c_20*a_20;
		c_32 -= c_30*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_22 -= c_21*a_21;
		c_32 -= c_31*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		c_22 *= a_22;
		c_32 *= a_22;
		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = inv_diag_E[3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_23 -= c_20*a_30;
		c_33 -= c_30*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_23 -= c_21*a_31;
		c_33 -= c_31*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_23 -= c_22*a_32;
		c_33 -= c_32*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		c_23 *= a_33;
		c_33 *= a_33;
		D[0+bs*3] = c_03;
		D[1+bs*3] = c_13;
		D[2+bs*3] = c_23;
		D[3+bs*3] = c_33;

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
		D[3+bs*0] = c_30;

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
		D[3+bs*1] = c_31;

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = 1.0/E[2+bs*2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_22 -= c_20*a_20;
		c_32 -= c_30*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_22 -= c_21*a_21;
		c_32 -= c_31*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		c_22 *= a_22;
		c_32 *= a_22;
		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = 1.0/E[3+bs*3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_23 -= c_20*a_30;
		c_33 -= c_30*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_23 -= c_21*a_31;
		c_33 -= c_31*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_23 -= c_22*a_32;
		c_33 -= c_32*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		c_23 *= a_33;
		c_33 *= a_33;
		D[0+bs*3] = c_03;
		D[1+bs*3] = c_13;
		D[2+bs*3] = c_23;
		D[3+bs*3] = c_33;

		}

	}
	
	
	
void kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;

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


				// k=2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
					
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
				b_2 = Bp[2+bs*2];
				b_3 = Bp[3+bs*2];
					
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
				a_0 = Ap[0+bs*3];
				a_1 = Ap[1+bs*3];
				a_2 = Ap[2+bs*3];
				a_3 = Ap[3+bs*3];
					
				b_0 = Bp[0+bs*3];
				b_1 = Bp[1+bs*3];
				b_2 = Bp[2+bs*3];
				b_3 = Bp[3+bs*3];
					
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
				b_2 = Bp[2+bs*0];
				b_3 = Bp[3+bs*0];
					
				c_00 += a_0 * b_0;

				c_01 += a_0 * b_1;

				c_02 += a_0 * b_2;

				c_03 += a_0 * b_3;

				k += 1;

				if(kadd>1)
					{

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

					k += 1;

					if(kadd>2)
						{

						// k=2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
							
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
						b_2 = Bp[2+bs*2];
						b_3 = Bp[3+bs*2];
							
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
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
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


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			a_2 = Ap[2+bs*1];
			a_3 = Ap[3+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			b_2 = Bp[2+bs*1];
			b_3 = Bp[3+bs*1];
			
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


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			a_2 = Ap[2+bs*2];
			a_3 = Ap[3+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			b_2 = Bp[2+bs*2];
			b_3 = Bp[3+bs*2];
			
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


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			a_2 = Ap[2+bs*3];
			a_3 = Ap[3+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			b_2 = Bp[2+bs*3];
			b_3 = Bp[3+bs*3];
			
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
			b_2 = Bp[2+bs*0];
			b_3 = Bp[3+bs*0];
			
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
		b_2 = Bm[2+bs*0];
		b_3 = Bm[3+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		a_2 = Am[2+bs*1];
		a_3 = Am[3+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		b_2 = Bm[2+bs*1];
		b_3 = Bm[3+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		a_2 = Am[2+bs*2];
		a_3 = Am[3+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		b_2 = Bm[2+bs*2];
		b_3 = Bm[3+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		a_2 = Am[2+bs*3];
		a_3 = Am[3+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		b_2 = Bm[2+bs*3];
		b_3 = Bm[3+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;
		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		c_23 -= a_2 * b_3;
		c_33 -= a_3 * b_3;
		
		
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

		c_02 += C[0+bs*2];
		c_12 += C[1+bs*2];
		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_03 += C[0+bs*3];
		c_13 += C[1+bs*3];
		c_23 += C[2+bs*3];
		c_33 += C[3+bs*3];
		}
	
	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
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

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = inv_diag_E[2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_22 -= c_20*a_20;
		c_32 -= c_30*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_22 -= c_21*a_21;
		c_32 -= c_31*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		c_22 *= a_22;
		c_32 *= a_22;
		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		if(km>=4)
			D[3+bs*2] = c_32;

		if(kn==3)
			return;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = inv_diag_E[3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_23 -= c_20*a_30;
		c_33 -= c_30*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_23 -= c_21*a_31;
		c_33 -= c_31*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_23 -= c_22*a_32;
		c_33 -= c_32*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		c_23 *= a_33;
		c_33 *= a_33;
		D[0+bs*3] = c_03;
		D[1+bs*3] = c_13;
		D[2+bs*3] = c_23;
		if(km>=4)
			D[3+bs*3] = c_33;

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

		a_20 = E[2+bs*0];
		a_21 = E[2+bs*1];
		a_22 = 1.0/E[2+bs*2];
		c_02 -= c_00*a_20;
		c_12 -= c_10*a_20;
		c_22 -= c_20*a_20;
		c_32 -= c_30*a_20;
		c_02 -= c_01*a_21;
		c_12 -= c_11*a_21;
		c_22 -= c_21*a_21;
		c_32 -= c_31*a_21;
		c_02 *= a_22;
		c_12 *= a_22;
		c_22 *= a_22;
		c_32 *= a_22;
		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		if(km>=4)
			D[3+bs*2] = c_32;

		if(kn==3)
			return;

		a_30 = E[3+bs*0];
		a_31 = E[3+bs*1];
		a_32 = E[3+bs*2];
		a_33 = 1.0/E[3+bs*3];
		c_03 -= c_00*a_30;
		c_13 -= c_10*a_30;
		c_23 -= c_20*a_30;
		c_33 -= c_30*a_30;
		c_03 -= c_01*a_31;
		c_13 -= c_11*a_31;
		c_23 -= c_21*a_31;
		c_33 -= c_31*a_31;
		c_03 -= c_02*a_32;
		c_13 -= c_12*a_32;
		c_23 -= c_22*a_32;
		c_33 -= c_32*a_32;
		c_03 *= a_33;
		c_13 *= a_33;
		c_23 *= a_33;
		c_33 *= a_33;
		D[0+bs*3] = c_03;
		D[1+bs*3] = c_13;
		D[2+bs*3] = c_23;
		if(km>=4)
			D[3+bs*3] = c_33;

		}

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


