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



void kernel_strmm_nt_4x4_lib4(int kadd, float *A, float *B, float *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = bs;

	int k;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;

	// initial triangle

	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
	
	b_0 = B[0+lda*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];
	
	b_0 = B[0+lda*1];
	b_1 = B[1+lda*1];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;


	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
	a_2 = A[2+lda*2];
	a_3 = A[3+lda*2];
	
	b_0 = B[0+lda*2];
	b_1 = B[1+lda*2];
	b_2 = B[2+lda*2];
	
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


	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
	a_2 = A[2+lda*3];
	a_3 = A[3+lda*3];
	
	b_0 = B[0+lda*3];
	b_1 = B[1+lda*3];
	b_2 = B[2+lda*3];
	b_3 = B[3+lda*3];
	
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
	B += 16;

	for(k=4; k<kadd-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
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


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		b_2 = B[2+lda*1];
		b_3 = B[3+lda*1];
		
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


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		b_2 = B[2+lda*2];
		b_3 = B[3+lda*2];
		
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


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		b_2 = B[2+lda*3];
		b_3 = B[3+lda*3];
		
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
		B += 16;

		}
	for(; k<kadd; k++)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
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
		B += 4;

		}

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;

	C[0+ldc*2] = c_02;
	C[1+ldc*2] = c_12;
	C[2+ldc*2] = c_22;
	C[3+ldc*2] = c_32;

	C[0+ldc*3] = c_03;
	C[1+ldc*3] = c_13;
	C[2+ldc*3] = c_23;
	C[3+ldc*3] = c_33;

	}



void corner_strmm_nt_4x3_lib4(float *A, float *B, float *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = bs;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0,
		c_10=0, c_11=0, c_12=0,
		c_20=0, c_21=0, c_22=0,
		c_30=0, c_31=0, c_32=0;

	// initial triangle

	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
	
	b_0 = B[0+lda*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];
	
	b_0 = B[0+lda*1];
	b_1 = B[1+lda*1];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;


	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
	a_2 = A[2+lda*2];
	a_3 = A[3+lda*2];
	
	b_0 = B[0+lda*2];
	b_1 = B[1+lda*2];
	b_2 = B[2+lda*2];
	
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


	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;

	C[0+ldc*2] = c_02;
	C[1+ldc*2] = c_12;
	C[2+ldc*2] = c_22;
	C[3+ldc*2] = c_32;

	}



void corner_strmm_nt_4x2_lib4(float *A, float *B, float *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = bs;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;

	// initial triangle

	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
	
	b_0 = B[0+lda*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];
	
	b_0 = B[0+lda*1];
	b_1 = B[1+lda*1];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;


	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;

	}



void corner_strmm_nt_4x1_lib4(float *A, float *B, float *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = bs;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0,
		c_10=0,
		c_20=0,
		c_30=0;

	// initial triangle

	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
	
	b_0 = B[0+lda*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	}




