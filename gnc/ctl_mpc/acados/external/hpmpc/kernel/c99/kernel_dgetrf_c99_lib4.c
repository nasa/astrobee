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

#include <math.h>

#include "../../include/d_blas_aux.h"
#include "../../include/blas_d.h"


#if ! defined(BLASFEO)
void kernel_dgetrf_nn_4x4_lib4(int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
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

	// factorization

	// first column
	tmp = 1.0 / d_00;
	d_10 *= tmp;
	d_20 *= tmp;
	d_30 *= tmp;

	inv_diag_D[0] = tmp;
	D[0+bs*0] = d_00;
	D[1+bs*0] = d_10;
	D[2+bs*0] = d_20;
	D[3+bs*0] = d_30;


	// second column
	d_11 -= d_10 * d_01;
	d_21 -= d_20 * d_01;
	d_31 -= d_30 * d_01;

	tmp = 1.0 / d_11;
	d_21 *= tmp;
	d_31 *= tmp;
	
	inv_diag_D[1] = tmp;
	D[0+bs*1] = d_01;
	D[1+bs*1] = d_11;
	D[2+bs*1] = d_21;
	D[3+bs*1] = d_31;


	// third column
	d_12 -= d_10 * d_02;
	d_22 -= d_20 * d_02;
	d_32 -= d_30 * d_02;

	d_22 -= d_21 * d_12;
	d_32 -= d_31 * d_12;

	tmp = 1.0 / d_22;
	d_32 *= tmp;

	inv_diag_D[2] = tmp;
	D[0+bs*2] = d_02;
	D[1+bs*2] = d_12;
	D[2+bs*2] = d_22;
	D[3+bs*2] = d_32;


	// fourth column
	d_13 -= d_10 * d_03;
	d_23 -= d_20 * d_03;
	d_33 -= d_30 * d_03;

	d_23 -= d_21 * d_13;
	d_33 -= d_31 * d_13;

	d_33 -= d_32 * d_23;

	tmp = 1.0 / d_33;

	inv_diag_D[3] = tmp;
	D[0+bs*3] = d_03;
	D[1+bs*3] = d_13;
	D[2+bs*3] = d_23;
	D[3+bs*3] = d_33;


	return;

	}



void kernel_dgetrf_nn_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
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

	// factorization

	// first column
	tmp = 1.0 / d_00;
	d_10 *= tmp;
	d_20 *= tmp;
	d_30 *= tmp;

	inv_diag_D[0] = tmp;
	D[0+bs*0] = d_00;
	D[1+bs*0] = d_10;
	D[2+bs*0] = d_20;
	if(km>=4)
		D[3+bs*0] = d_30;


	// second column
	d_11 -= d_10 * d_01;
	d_21 -= d_20 * d_01;
	d_31 -= d_30 * d_01;

	tmp = 1.0 / d_11;
	d_21 *= tmp;
	d_31 *= tmp;
	
	inv_diag_D[1] = tmp;
	D[0+bs*1] = d_01;
	D[1+bs*1] = d_11;
	D[2+bs*1] = d_21;
	if(km>=4)
		D[3+bs*1] = d_31;


	// third column
	d_12 -= d_10 * d_02;
	d_22 -= d_20 * d_02;
	d_32 -= d_30 * d_02;

	d_22 -= d_21 * d_12;
	d_32 -= d_31 * d_12;

	tmp = 1.0 / d_22;
	d_32 *= tmp;

	inv_diag_D[2] = tmp;
	D[0+bs*2] = d_02;
	D[1+bs*2] = d_12;
	D[2+bs*2] = d_22;
	if(km>=4)
		D[3+bs*2] = d_32;

	if(kn<4)
		return;

	// fourth column
	d_13 -= d_10 * d_03;
	d_23 -= d_20 * d_03;
	d_33 -= d_30 * d_03;

	d_23 -= d_21 * d_13;
	d_33 -= d_31 * d_13;

	d_33 -= d_32 * d_23;

	tmp = 1.0 / d_33;

	inv_diag_D[3] = tmp;
	D[0+bs*3] = d_03;
	D[1+bs*3] = d_13;
	D[2+bs*3] = d_23;
	if(km>=4)
		D[3+bs*3] = d_33;


	return;

	}



void kernel_dgetrf_nn_2x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1,
		b_0, b_1, b_2, b_3,
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

	// factorization

	// first column
	tmp = 1.0 / d_00;
	d_10 *= tmp;

	inv_diag_D[0] = tmp;
	D[0+bs*0] = d_00;
	if(km>=2)
		D[1+bs*0] = d_10;


	// second column
	d_11 -= d_10 * d_01;

	tmp = 1.0 / d_11;
	
	inv_diag_D[1] = tmp;
	D[0+bs*1] = d_01;
	if(km>=2)
		D[1+bs*1] = d_11;


	// third column
	d_12 -= d_10 * d_02;

	D[0+bs*2] = d_02;
	if(km>=2)
		D[1+bs*2] = d_12;

	if(kn<4)
		return;

	// fourth column
	d_13 -= d_10 * d_03;

	D[0+bs*3] = d_03;
	if(km>=2)
		D[1+bs*3] = d_13;


	return;

	}



void kernel_dgetrf_nn_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1, a_2, a_3,
		b_0, b_1,
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

	// factorization

	// first column
	tmp = 1.0 / d_00;
	d_10 *= tmp;
	d_20 *= tmp;
	d_30 *= tmp;

	inv_diag_D[0] = tmp;
	D[0+bs*0] = d_00;
	D[1+bs*0] = d_10;
	D[2+bs*0] = d_20;
	if(km>=4)
		D[3+bs*0] = d_30;

	if(kn<2)
		return;

	// second column
	d_11 -= d_10 * d_01;
	d_21 -= d_20 * d_01;
	d_31 -= d_30 * d_01;

	tmp = 1.0 / d_11;
	d_21 *= tmp;
	d_31 *= tmp;
	
	inv_diag_D[1] = tmp;
	D[0+bs*1] = d_01;
	D[1+bs*1] = d_11;
	D[2+bs*1] = d_21;
	if(km>=4)
		D[3+bs*1] = d_31;


	return;

	}



void kernel_dgetrf_nn_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		tmp,
		a_0, a_1,
		b_0, b_1,
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

	// factorization

	// first column
	tmp = 1.0 / d_00;
	d_10 *= tmp;

	inv_diag_D[0] = tmp;
	D[0+bs*0] = d_00;
	if(km>=2)
		D[1+bs*0] = d_10;

	if(kn<2)
		return;

	// second column
	d_11 -= d_10 * d_01;

	tmp = 1.0 / d_11;
	
	inv_diag_D[1] = tmp;
	D[0+bs*1] = d_01;
	if(km>=2)
		D[1+bs*1] = d_11;


	return;

	}



#if 0
void corner_dgetrf_nn_4x4_lib4(double *C, double *LU, double *inv_diag_U)
	{

	const int bs = 4;

	double
		// elements of C
		c_00, c_01, c_02, c_03,
		c_10, c_11, c_12, c_13,
		c_20, c_21, c_22, c_23,
		c_30, c_31, c_32, c_33,
		// LU
		u_00, u_01, u_02, u_03,
		l_10, u_11, u_12, u_13,
		l_20, l_21, u_22, u_23,
		l_30, l_31, l_32, u_33,
		// inv diag U
		iu_0, iu_1, iu_2, iu_3;


	// first column
	c_00 = C[0+bs*0];
	c_10 = C[1+bs*0];
	c_20 = C[2+bs*0];
	c_30 = C[3+bs*0];

	u_00 = c_00;
	iu_0 = 1.0 / u_00;
	l_10 = c_10 * iu_0;
	l_20 = c_20 * iu_0;
	l_30 = c_30 * iu_0;


	// second column
	c_01 = C[0+bs*1];
	c_11 = C[1+bs*1];
	c_21 = C[2+bs*1];
	c_31 = C[3+bs*1];

	u_01 = c_01;
	c_11 -= l_10 * u_01;
	c_21 -= l_20 * u_01;
	c_31 -= l_30 * u_01;

	u_11 = c_11;
	iu_1 = 1.0 / u_11;
	l_21 = c_21 * iu_1;
	l_31 = c_31 * iu_1;


	// third column
	c_02 = C[0+bs*2];
	c_12 = C[1+bs*2];
	c_22 = C[2+bs*2];
	c_32 = C[3+bs*2];

	u_02 = c_02;
	c_12 -= l_10 * u_02;
	c_22 -= l_20 * u_02;
	c_32 -= l_30 * u_02;

	u_12 = c_12;
	c_22 -= l_21 * u_12;
	c_32 -= l_31 * u_12;

	u_22 = c_22;
	iu_2 = 1.0 / u_22;
	l_32 = c_32 * iu_2;


	// fourth column
	c_03 = C[0+bs*3];
	c_13 = C[1+bs*3];
	c_23 = C[2+bs*3];
	c_33 = C[3+bs*3];

	u_03 = c_03;
	c_13 -= l_10 * u_03;
	c_23 -= l_20 * u_03;
	c_33 -= l_30 * u_03;

	u_13 = c_13;
	c_23 -= l_21 * u_13;
	c_33 -= l_31 * u_13;

	u_23 = c_23;
	c_33 -= l_32 * u_23;

	u_33 = c_33;
	iu_3 = 1.0 / u_33;

	
	// store LU
	LU[0+bs*0] = u_00;
	LU[1+bs*0] = l_10;
	LU[2+bs*0] = l_20;
	LU[3+bs*0] = l_30;

	LU[0+bs*1] = u_01;
	LU[1+bs*1] = u_11;
	LU[2+bs*1] = l_21;
	LU[3+bs*1] = l_31;

	LU[0+bs*2] = u_02;
	LU[1+bs*2] = u_12;
	LU[2+bs*2] = u_22;
	LU[3+bs*2] = l_32;

	LU[0+bs*3] = u_03;
	LU[1+bs*3] = u_13;
	LU[2+bs*3] = u_23;
	LU[3+bs*3] = u_33;


	
	// return
	return;
	
	}
#endif



// C numbering, starting from 0
void idamax_lib4(int n, int offset, double *pA, int sda, int *p_idamax, double *p_amax)
	{

	int idamax, ii;
	double tmp, amax;
		
	p_idamax[0] = -1;
	if(n<1)
		return;

	const int bs = 4;

	int na = (bs - offset%bs)%bs;
	na = n<na ? n : na;

	amax = -1.0;
	ii = 0;
	if(na>0)
		{
		for( ; ii<na; ii++)
			{
			tmp = fabs(pA[0]);
			if(tmp>amax)
				{
				idamax = ii+0;
				amax = tmp;
				}
			pA += 1;
			}
		pA += bs*(sda-1);
		}
	for( ; ii<n-3; ii+=4)
		{
		tmp = fabs(pA[0]);
		if(tmp>amax)
			{
			idamax = ii+0;
			amax = tmp;
			}
		tmp = fabs(pA[1]);
		if(tmp>amax)
			{
			idamax = ii+1;
			amax = tmp;
			}
		tmp = fabs(pA[2]);
		if(tmp>amax)
			{
			idamax = ii+2;
			amax = tmp;
			}
		tmp = fabs(pA[3]);
		if(tmp>amax)
			{
			idamax = ii+3;
			amax = tmp;
			}
		pA += bs*sda;
		}
	for( ; ii<n; ii++)
		{
		tmp = fabs(pA[0]);
		if(tmp>amax)
			{
			idamax = ii+0;
			amax = tmp;
			}
		pA += 1;
		}
	
	p_amax[0] = amax;
	p_idamax[0] = idamax;

	return;

	}



// C numering (starting from zero) in the ipiv
// it process m>=4 rows and 4 cols
void kernel_dgetrf_pivot_4_lib4(int m, double *pA, int sda, double *inv_diag_A, int* ipiv)
	{

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	double
		tmp0, tmp1, tmp2, tmp3,
		u_00, u_01, u_02, u_03,
		      u_11, u_12, u_13,
		            u_22, u_23,
		                  u_33;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	// first column
	idamax_lib4(m-0, 0, &pA[0+bs*0], sda, &idamax, &tmp0);
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			drowsw_lib(4, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		tmp0 = 1.0 / pA[0+bs*0];
		inv_diag_A[0] = tmp0;
		pA[1+bs*0] *= tmp0;
		pA[2+bs*0] *= tmp0;
		pA[3+bs*0] *= tmp0;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*0] *= tmp0;
			pB[1+bs*0] *= tmp0;
			pB[2+bs*0] *= tmp0;
			pB[3+bs*0] *= tmp0;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*0] *= tmp0;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[0] = 0.0;
		}

	// second column
	u_01  = pA[0+bs*1];
	tmp1  = pA[1+bs*1];
	tmp2  = pA[2+bs*1];
	tmp3  = pA[3+bs*1];
	tmp1 -= pA[1+bs*0] * u_01;
	tmp2 -= pA[2+bs*0] * u_01;
	tmp3 -= pA[3+bs*0] * u_01;
	pA[1+bs*1] = tmp1;
	pA[2+bs*1] = tmp2;
	pA[3+bs*1] = tmp3;
	pB = pA + bs*sda;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+bs*1];
		tmp1  = pB[1+bs*1];
		tmp2  = pB[2+bs*1];
		tmp3  = pB[3+bs*1];
		tmp0 -= pB[0+bs*0] * u_01;
		tmp1 -= pB[1+bs*0] * u_01;
		tmp2 -= pB[2+bs*0] * u_01;
		tmp3 -= pB[3+bs*0] * u_01;
		pB[0+bs*1] = tmp0;
		pB[1+bs*1] = tmp1;
		pB[2+bs*1] = tmp2;
		pB[3+bs*1] = tmp3;
		pB += bs*sda;
		}
	for( ; k<ma; k++)
		{
		tmp0 = pB[0+bs*1];
		tmp0 -= pB[0+bs*0] * u_01;
		pB[0+bs*1] = tmp0;
		pB += 1;
		}

	idamax_lib4(m-1, 1, &pA[1+bs*1], sda, &idamax, &tmp1);
	ipiv[1] = idamax+1;
	if(tmp1!=0)
		{
		if(ipiv[1]!=1)
			drowsw_lib(4, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

		tmp1 = 1.0 / pA[1+bs*1];
		inv_diag_A[1] = tmp1;
		pA[2+bs*1] *= tmp1;
		pA[3+bs*1] *= tmp1;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*1] *= tmp1;
			pB[1+bs*1] *= tmp1;
			pB[2+bs*1] *= tmp1;
			pB[3+bs*1] *= tmp1;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*1] *= tmp1;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[1] = 0.0;
		}

	// third column
	u_02  = pA[0+bs*2];
	u_12  = pA[1+bs*2];
	u_12 -= pA[1+bs*0] * u_02;
	pA[1+bs*2] = u_12;
	tmp2  = pA[2+bs*2];
	tmp3  = pA[3+bs*2];
	tmp2 -= pA[2+bs*0] * u_02;
	tmp3 -= pA[3+bs*0] * u_02;
	tmp2 -= pA[2+bs*1] * u_12;
	tmp3 -= pA[3+bs*1] * u_12;
	pA[2+bs*2] = tmp2;
	pA[3+bs*2] = tmp3;
	pB = pA + bs*sda;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+bs*2];
		tmp1  = pB[1+bs*2];
		tmp2  = pB[2+bs*2];
		tmp3  = pB[3+bs*2];
		tmp0 -= pB[0+bs*0] * u_02;
		tmp1 -= pB[1+bs*0] * u_02;
		tmp2 -= pB[2+bs*0] * u_02;
		tmp3 -= pB[3+bs*0] * u_02;
		tmp0 -= pB[0+bs*1] * u_12;
		tmp1 -= pB[1+bs*1] * u_12;
		tmp2 -= pB[2+bs*1] * u_12;
		tmp3 -= pB[3+bs*1] * u_12;
		pB[0+bs*2] = tmp0;
		pB[1+bs*2] = tmp1;
		pB[2+bs*2] = tmp2;
		pB[3+bs*2] = tmp3;
		pB += bs*sda;
		}
	for( ; k<ma; k++)
		{
		tmp0  = pB[0+bs*2];
		tmp0 -= pB[0+bs*0] * u_02;
		tmp0 -= pB[0+bs*1] * u_12;
		pB[0+bs*2] = tmp0;
		pB += 1;
		}

	idamax_lib4(m-2, 2, &pA[2+bs*2], sda, &idamax, &tmp2);
	ipiv[2] = idamax+2;
	if(tmp2!=0)
		{
		if(ipiv[2]!=2)
			drowsw_lib(4, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

		tmp2 = 1.0 / pA[2+bs*2];
		inv_diag_A[2] = tmp2;
		pA[3+bs*2] *= tmp2;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*2] *= tmp2;
			pB[1+bs*2] *= tmp2;
			pB[2+bs*2] *= tmp2;
			pB[3+bs*2] *= tmp2;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*2] *= tmp2;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[2] = 0.0;
		}

	// fourth column
	u_03  = pA[0+bs*3];
	u_13  = pA[1+bs*3];
	u_13 -= pA[1+bs*0] * u_03;
	pA[1+bs*3] = u_13;
	u_23  = pA[2+bs*3];
	u_23 -= pA[2+bs*0] * u_03;
	u_23 -= pA[2+bs*1] * u_13;
	pA[2+bs*3] = u_23;
	tmp3  = pA[3+bs*3];
	tmp3 -= pA[3+bs*0] * u_03;
	tmp3 -= pA[3+bs*1] * u_13;
	tmp3 -= pA[3+bs*2] * u_23;
	pA[3+bs*3] = tmp3;
	pB = pA + bs*sda;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+bs*3];
		tmp1  = pB[1+bs*3];
		tmp2  = pB[2+bs*3];
		tmp3  = pB[3+bs*3];
		tmp0 -= pB[0+bs*0] * u_03;
		tmp1 -= pB[1+bs*0] * u_03;
		tmp2 -= pB[2+bs*0] * u_03;
		tmp3 -= pB[3+bs*0] * u_03;
		tmp0 -= pB[0+bs*1] * u_13;
		tmp1 -= pB[1+bs*1] * u_13;
		tmp2 -= pB[2+bs*1] * u_13;
		tmp3 -= pB[3+bs*1] * u_13;
		tmp0 -= pB[0+bs*2] * u_23;
		tmp1 -= pB[1+bs*2] * u_23;
		tmp2 -= pB[2+bs*2] * u_23;
		tmp3 -= pB[3+bs*2] * u_23;
		pB[0+bs*3] = tmp0;
		pB[1+bs*3] = tmp1;
		pB[2+bs*3] = tmp2;
		pB[3+bs*3] = tmp3;
		pB += bs*sda;
		}
	for( ; k<ma; k++)
		{
		tmp0  = pB[0+bs*3];
		tmp0 -= pB[0+bs*0] * u_03;
		tmp0 -= pB[0+bs*1] * u_13;
		tmp0 -= pB[0+bs*2] * u_23;
		pB[0+bs*3] = tmp0;
		pB += 1;
		}

	idamax_lib4(m-3, 3, &pA[3+bs*3], sda, &idamax, &tmp3);
	ipiv[3] = idamax+3;
	if(tmp3!=0)
		{
		if(ipiv[3]!=3)
			drowsw_lib(4, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

		tmp3 = 1.0 / pA[3+bs*3];
		inv_diag_A[3] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*3] *= tmp3;
			pB[1+bs*3] *= tmp3;
			pB[2+bs*3] *= tmp3;
			pB[3+bs*3] *= tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*3] *= tmp3;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[3] = 0.0;
		}
	
	return;

	}



// it process m>0 rows and 0<n<=4 cols
void kernel_dgetrf_pivot_4_vs_lib4(int m, int n, double *pA, int sda, double *inv_diag_A, int* ipiv)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	double
		tmp0, tmp1, tmp2, tmp3,
		u_00, u_01, u_02, u_03,
		      u_11, u_12, u_13,
		            u_22, u_23,
		                  u_33;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	// first column

	// find pivot & scale
	idamax_lib4(m-0, 0, &pA[0+bs*0], sda, &idamax, &tmp0);
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			drowsw_lib(4, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		tmp0 = 1.0 / pA[0+bs*0];
		inv_diag_A[0] = tmp0;
		if(m>=4)
			{
			pA[1+bs*0] *= tmp0;
			pA[2+bs*0] *= tmp0;
			pA[3+bs*0] *= tmp0;
			pB = pA + bs*sda;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+bs*0] *= tmp0;
				pB[1+bs*0] *= tmp0;
				pB[2+bs*0] *= tmp0;
				pB[3+bs*0] *= tmp0;
				pB += bs*sda;
				}
			for( ; k<ma; k++)
				{
				pB[0+bs*0] *= tmp0;
				pB += 1;
				}
			}
		else // m = {1,2,3}
			{
			if(m>1)
				{
				pA[1+bs*0] *= tmp0;
				if(m>2)
					pA[2+bs*0] *= tmp0;
				}
			}
		}
	else
		{
		inv_diag_A[0] = 0.0;
		}
	
	if(n==1 || m==1) // XXX for the first row there is nothing to do, so we can return here
		return;

	// second column

	// correct
	if(m>=4)
		{
		u_01  = pA[0+bs*1];
		tmp1  = pA[1+bs*1];
		tmp2  = pA[2+bs*1];
		tmp3  = pA[3+bs*1];
		tmp1 -= pA[1+bs*0] * u_01;
		tmp2 -= pA[2+bs*0] * u_01;
		tmp3 -= pA[3+bs*0] * u_01;
		pA[1+bs*1] = tmp1;
		pA[2+bs*1] = tmp2;
		pA[3+bs*1] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+bs*1];
			tmp1  = pB[1+bs*1];
			tmp2  = pB[2+bs*1];
			tmp3  = pB[3+bs*1];
			tmp0 -= pB[0+bs*0] * u_01;
			tmp1 -= pB[1+bs*0] * u_01;
			tmp2 -= pB[2+bs*0] * u_01;
			tmp3 -= pB[3+bs*0] * u_01;
			pB[0+bs*1] = tmp0;
			pB[1+bs*1] = tmp1;
			pB[2+bs*1] = tmp2;
			pB[3+bs*1] = tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			tmp0 = pB[0+bs*1];
			tmp0 -= pB[0+bs*0] * u_01;
			pB[0+bs*1] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_01  = pA[0+bs*1];
		tmp1  = pA[1+bs*1];
		tmp1 -= pA[1+bs*0] * u_01;
		pA[1+bs*1] = tmp1;
		if(m>2)
			{
			tmp2  = pA[2+bs*1];
			tmp2 -= pA[2+bs*0] * u_01;
			pA[2+bs*1] = tmp2;
			}
		}

	// find pivot & scale
	idamax_lib4(m-1, 1, &pA[1+bs*1], sda, &idamax, &tmp1);
	ipiv[1] = idamax+1;
	if(tmp1!=0)
		{
		if(ipiv[1]!=1)
			drowsw_lib(4, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

		tmp1 = 1.0 / pA[1+bs*1];
		inv_diag_A[1] = tmp1;
		if(m>=4)
			{
			pA[2+bs*1] *= tmp1;
			pA[3+bs*1] *= tmp1;
			pB = pA + bs*sda;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+bs*1] *= tmp1;
				pB[1+bs*1] *= tmp1;
				pB[2+bs*1] *= tmp1;
				pB[3+bs*1] *= tmp1;
				pB += bs*sda;
				}
			for( ; k<ma; k++)
				{
				pB[0+bs*1] *= tmp1;
				pB += 1;
				}
			}
		else // m = {2,3}
			{
			if(m>2)
				pA[2+bs*1] *= tmp1;
			}
		}
	else
		{
		inv_diag_A[1] = 0.0;
		}

	if(n==2)
		return;

	// third column

	// correct
	if(m>=4)
		{
		u_02  = pA[0+bs*2];
		u_12  = pA[1+bs*2];
		u_12 -= pA[1+bs*0] * u_02;
		pA[1+bs*2] = u_12;
		tmp2  = pA[2+bs*2];
		tmp3  = pA[3+bs*2];
		tmp2 -= pA[2+bs*0] * u_02;
		tmp3 -= pA[3+bs*0] * u_02;
		tmp2 -= pA[2+bs*1] * u_12;
		tmp3 -= pA[3+bs*1] * u_12;
		pA[2+bs*2] = tmp2;
		pA[3+bs*2] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+bs*2];
			tmp1  = pB[1+bs*2];
			tmp2  = pB[2+bs*2];
			tmp3  = pB[3+bs*2];
			tmp0 -= pB[0+bs*0] * u_02;
			tmp1 -= pB[1+bs*0] * u_02;
			tmp2 -= pB[2+bs*0] * u_02;
			tmp3 -= pB[3+bs*0] * u_02;
			tmp0 -= pB[0+bs*1] * u_12;
			tmp1 -= pB[1+bs*1] * u_12;
			tmp2 -= pB[2+bs*1] * u_12;
			tmp3 -= pB[3+bs*1] * u_12;
			pB[0+bs*2] = tmp0;
			pB[1+bs*2] = tmp1;
			pB[2+bs*2] = tmp2;
			pB[3+bs*2] = tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			tmp0  = pB[0+bs*2];
			tmp0 -= pB[0+bs*0] * u_02;
			tmp0 -= pB[0+bs*1] * u_12;
			pB[0+bs*2] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_02  = pA[0+bs*2];
		u_12  = pA[1+bs*2];
		u_12 -= pA[1+bs*0] * u_02;
		pA[1+bs*2] = u_12;
		if(m>2)
			{
			tmp2  = pA[2+bs*2];
			tmp2 -= pA[2+bs*0] * u_02;
			tmp2 -= pA[2+bs*1] * u_12;
			pA[2+bs*2] = tmp2;
			}
		}

	// find pivot & scale
	if(m>2)
		{
		idamax_lib4(m-2, 2, &pA[2+bs*2], sda, &idamax, &tmp2);
		ipiv[2] = idamax+2;
		if(tmp2!=0)
			{
			if(ipiv[2]!=2)
				drowsw_lib(4, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

			tmp2 = 1.0 / pA[2+bs*2];
			inv_diag_A[2] = tmp2;
			if(m>=4)
				{
				pA[3+bs*2] *= tmp2;
				pB = pA + bs*sda;
				for(k=0; k<ma-3; k+=4)
					{
					pB[0+bs*2] *= tmp2;
					pB[1+bs*2] *= tmp2;
					pB[2+bs*2] *= tmp2;
					pB[3+bs*2] *= tmp2;
					pB += bs*sda;
					}
				for( ; k<ma; k++)
					{
					pB[0+bs*2] *= tmp2;
					pB += 1;
					}
				}
			}
		else
			{
			inv_diag_A[2] = 0.0;
			}
		}

	if(n<4)
		return;

	// fourth column

	// correct
	if(m>=4)
		{
		u_03  = pA[0+bs*3];
		u_13  = pA[1+bs*3];
		u_13 -= pA[1+bs*0] * u_03;
		pA[1+bs*3] = u_13;
		u_23  = pA[2+bs*3];
		u_23 -= pA[2+bs*0] * u_03;
		u_23 -= pA[2+bs*1] * u_13;
		pA[2+bs*3] = u_23;
		tmp3  = pA[3+bs*3];
		tmp3 -= pA[3+bs*0] * u_03;
		tmp3 -= pA[3+bs*1] * u_13;
		tmp3 -= pA[3+bs*2] * u_23;
		pA[3+bs*3] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+bs*3];
			tmp1  = pB[1+bs*3];
			tmp2  = pB[2+bs*3];
			tmp3  = pB[3+bs*3];
			tmp0 -= pB[0+bs*0] * u_03;
			tmp1 -= pB[1+bs*0] * u_03;
			tmp2 -= pB[2+bs*0] * u_03;
			tmp3 -= pB[3+bs*0] * u_03;
			tmp0 -= pB[0+bs*1] * u_13;
			tmp1 -= pB[1+bs*1] * u_13;
			tmp2 -= pB[2+bs*1] * u_13;
			tmp3 -= pB[3+bs*1] * u_13;
			tmp0 -= pB[0+bs*2] * u_23;
			tmp1 -= pB[1+bs*2] * u_23;
			tmp2 -= pB[2+bs*2] * u_23;
			tmp3 -= pB[3+bs*2] * u_23;
			pB[0+bs*3] = tmp0;
			pB[1+bs*3] = tmp1;
			pB[2+bs*3] = tmp2;
			pB[3+bs*3] = tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			tmp0  = pB[0+bs*3];
			tmp0 -= pB[0+bs*0] * u_03;
			tmp0 -= pB[0+bs*1] * u_13;
			tmp0 -= pB[0+bs*2] * u_23;
			pB[0+bs*3] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_03  = pA[0+bs*3];
		u_13  = pA[1+bs*3];
		u_13 -= pA[1+bs*0] * u_03;
		pA[1+bs*3] = u_13;
		if(m>2)
			{
			u_23  = pA[2+bs*3];
			u_23 -= pA[2+bs*0] * u_03;
			u_23 -= pA[2+bs*1] * u_13;
			pA[2+bs*3] = u_23;
			}
		}

	if(m>3)
		{
		// find pivot & scale
		idamax_lib4(m-3, 3, &pA[3+bs*3], sda, &idamax, &tmp3);
		ipiv[3] = idamax+3;
		if(tmp3!=0)
			{
			if(ipiv[3]!=3)
				drowsw_lib(4, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

			tmp3 = 1.0 / pA[3+bs*3];
			inv_diag_A[3] = tmp3;
			pB = pA + bs*sda;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+bs*3] *= tmp3;
				pB[1+bs*3] *= tmp3;
				pB[2+bs*3] *= tmp3;
				pB[3+bs*3] *= tmp3;
				pB += bs*sda;
				}
			for( ; k<ma; k++)
				{
				pB[0+bs*3] *= tmp3;
				pB += 1;
				}
			}
		else
			{
			inv_diag_A[3] = 0.0;
			}
		}
	
	return;

	}
#endif

	


