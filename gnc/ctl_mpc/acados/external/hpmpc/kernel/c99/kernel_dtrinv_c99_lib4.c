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
* MERCHANTAA_00_invILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, A_00_invoston, MA  02110-1301  USA                  *
*                                                                                                 *
* Author: Gianluca Frison, giaf (at) dtu.dk                                                       *
*                                                                                                 *
**************************************************************************************************/



void corner_dtrtri_4x4_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	double
		c_00,
		c_10, c_11,
		c_20, c_21, c_22,
		c_30, c_31, c_32, c_33;

	if(use_inv_diag_A)
		{
		c_00 = inv_diag_A[0];
		c_11 = inv_diag_A[1];
		c_22 = inv_diag_A[2];
		c_33 = inv_diag_A[3];
		}
	else
		{
		c_00 = 1.0/A[0+bs*0];
		c_11 = 1.0/A[1+bs*1];
		c_22 = 1.0/A[2+bs*2];
		c_33 = 1.0/A[3+bs*3];
		}

	// 1x1 diagoanl blocks
	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;
	C[2+bs*2] = c_22;
	C[3+bs*3] = c_33;

	// 2x2 diagonal blocks
	c_10 = A[1+bs*0];
	c_32 = A[3+bs*2];

	c_10 = - c_11*c_10*c_00;
	c_32 = - c_33*c_32*c_22;

	C[0+bs*1] = c_10;
	C[2+bs*3] = c_32;

	// 4x4 diagonal blocks
	c_20 = A[2+bs*0];
	c_30 = A[3+bs*0];
	c_21 = A[2+bs*1];
	c_31 = A[3+bs*1];

	c_30 = - c_32*c_20 - c_33*c_30;
	c_31 = - c_32*c_21 - c_33*c_31;
	c_20 = - c_22*c_20;
	c_21 = - c_22*c_21;

	c_30 = c_30*c_00 + c_31*c_10;
	c_20 = c_20*c_00 + c_21*c_10;
	c_31 = c_31*c_11;
	c_21 = c_21*c_11;

	C[0+2*bs] = c_20;
	C[1+2*bs] = c_21;
	C[0+3*bs] = c_30;
	C[1+3*bs] = c_31;

	return;

	}



void corner_dtrtri_3x3_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	double
		c_00,
		c_10, c_11,
		c_20, c_21, c_22;

	if(use_inv_diag_A)
		{
		c_00 = inv_diag_A[0];
		c_11 = inv_diag_A[1];
		c_22 = inv_diag_A[2];
		}
	else
		{
		c_00 = 1.0/A[0+bs*0];
		c_11 = 1.0/A[1+bs*1];
		c_22 = 1.0/A[2+bs*2];
		}

	// 1x1 diagoanl blocks
	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;
	C[2+bs*2] = c_22;

	// 2x2 diagonal blocks
	c_10 = A[1+bs*0];

	c_10 = - c_11*c_10*c_00;

	C[0+bs*1] = c_10;

	// 4x4 diagonal blocks
	c_20 = A[2+bs*0];
	c_21 = A[2+bs*1];

	c_20 = - c_22*c_20;
	c_21 = - c_22*c_21;

	c_20 = c_20*c_00 + c_21*c_10;
	c_21 = c_21*c_11;

	C[0+2*bs] = c_20;
	C[1+2*bs] = c_21;

	return;

	}



void corner_dtrtri_2x2_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	double
		c_00,
		c_10, c_11;

	if(use_inv_diag_A)
		{
		c_00 = inv_diag_A[0];
		c_11 = inv_diag_A[1];
		}
	else
		{
		c_00 = 1.0/A[0+bs*0];
		c_11 = 1.0/A[1+bs*1];
		}

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;

	c_10 = A[1+bs*0];

	c_10 = - c_11*c_10*c_00;

	C[0+bs*1] = c_10;

	return;

	}



void corner_dtrtri_1x1_lib4(double *A, int use_inv_diag_A, double *inv_diag_A, double *C)
	{

	const int bs = 4;

	double
		c_00,
		c_10, c_11;

	if(use_inv_diag_A)
		{
		c_00 = inv_diag_A[0];
//		c_11 = inv_diag_A[1];
		}
	else
		{
		c_00 = 1.0/A[0+bs*0];
//		c_11 = 1.0/A[1+bs*1];
		}

	C[0+bs*0] = c_00;
//	C[1+bs*1] = c_11;

//	c_10 = A[1+bs*0];

//	c_10 = - c_11*c_10*c_00;

//	C[0+bs*1] = c_10;

	return;

	}



void kernel_dtrtri_4x4_lib4(int kmax, double *A, double *B, double *C, double *E, int use_inv_diag_E, double *inv_diag_E)
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
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];
		
	c_00 -= a_0 * b_0;

	c_01 -= a_0 * b_1;

	c_02 -= a_0 * b_2;

	c_03 -= a_0 * b_3;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;

	c_02 -= a_0 * b_2;
	c_12 -= a_1 * b_2;

	c_03 -= a_0 * b_3;
	c_13 -= a_1 * b_3;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;

	c_02 -= a_0 * b_2;
	c_12 -= a_1 * b_2;
	c_22 -= a_2 * b_2;

	c_03 -= a_0 * b_3;
	c_13 -= a_1 * b_3;
	c_23 -= a_2 * b_3;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];
		
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

	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
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


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		b_3 = B[3+bs*3];
		
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
		
		
		A += 16;
		B += 16;

		}

	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		a_11 = inv_diag_E[1];
		a_22 = inv_diag_E[2];
		a_33 = inv_diag_E[3];
		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		a_11 = 1.0/E[1+bs*1];
		a_22 = 1.0/E[2+bs*2];
		a_33 = 1.0/E[3+bs*3];
		}
//	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	a_10 = E[1+bs*0]; //fact[1];
//	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_21 -= c_20*a_10;
	c_31 -= c_30*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	c_21 *= a_11;
	c_31 *= a_11;
	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	a_20 = E[2+bs*0]; //fact[3];
	a_21 = E[2+bs*1]; //fact[4];
//	a_22 = fact[5];
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
	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	a_30 = E[3+bs*0]; //fact[6];
	a_31 = E[3+bs*1]; //fact[7];
	a_32 = E[3+bs*2]; //fact[8];
//	a_33 = fact[9];
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
	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;

	}
	
	
	
void kernel_dtrtri_4x3_lib4(int kmax, double *A, double *B, double *C, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0,
		c_10=0, c_11=0, c_12=0,
		c_20=0, c_21=0, c_22=0,
		c_30=0, c_31=0, c_32=0;
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
		
	c_00 -= a_0 * b_0;

	c_01 -= a_0 * b_1;

	c_02 -= a_0 * b_2;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;

	c_02 -= a_0 * b_2;
	c_12 -= a_1 * b_2;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;

	c_02 -= a_0 * b_2;
	c_12 -= a_1 * b_2;
	c_22 -= a_2 * b_2;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
		
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

	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		
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


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		
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
		
		
		A += 16;
		B += 16;

		}

	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		a_11 = inv_diag_E[1];
		a_22 = inv_diag_E[2];
		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		a_11 = 1.0/E[1+bs*1];
		a_22 = 1.0/E[2+bs*2];
		}
//	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	a_10 = E[1+bs*0]; //fact[1];
//	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_21 -= c_20*a_10;
	c_31 -= c_30*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	c_21 *= a_11;
	c_31 *= a_11;
	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	a_20 = E[2+bs*0]; //fact[3];
	a_21 = E[2+bs*1]; //fact[4];
//	a_22 = fact[5];
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
	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	}
	
	
	
void kernel_dtrtri_4x2_lib4(int kmax, double *A, double *B, double *C, double *E, int use_inv_diag_E, double *inv_diag_E)
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
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
		
	c_00 -= a_0 * b_0;

	c_01 -= a_0 * b_1;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;
	c_30 -= a_3 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;
	c_31 -= a_3 * b_1;


	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}

	// dtrsm
	double
		a_00, a_10, a_11;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		a_11 = inv_diag_E[1];
		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		a_11 = 1.0/E[1+bs*1];
		}

//	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	a_10 = E[1+bs*0];
//	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_21 -= c_20*a_10;
	c_31 -= c_30*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	c_21 *= a_11;
	c_31 *= a_11;
	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	}
	
	
	
void kernel_dtrtri_4x1_lib4(int kmax, double *A, double *B, double *C, double *E, int use_inv_diag_E, double *inv_diag_E)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0,
		c_10=0,
		c_20=0,
		c_30=0;
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
		
	c_00 -= a_0 * b_0;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;
	c_30 -= a_3 * b_0;


	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;
		
		
		A += 16;
		B += 16;

		}

	// dtrsm
	double
		a_00, a_10, a_11;
	
	if(use_inv_diag_E)
		{
		a_00 = inv_diag_E[0];
		}
	else
		{
		a_00 = 1.0/E[0+bs*0];
		}

//	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	}
	
	
	
// old kernels



void corner_dtrinv_4x4_lib4(double *fact, double *C)
	{

	const int bs = 4;

	double
		c_00=0.0,
		c_10=0.0, c_11=0.0,
		c_20=0.0, c_21=0.0, c_22=0.0,
		c_30=0.0, c_31=0.0, c_32=0.0, c_33=0.0,
		d_20=0.0, d_21=0.0,
		d_30=0.0, d_31=0.0;

//	c_00 = 1.0/A[0+bs*0];
//	c_11 = 1.0/A[1+bs*1];
//	c_22 = 1.0/A[2+bs*2];
//	c_33 = 1.0/A[3+bs*3];
	c_00 = fact[0];
	c_11 = fact[2];
	c_22 = fact[5];
	c_33 = fact[9];

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;
	C[2+bs*2] = c_22;
	C[3+bs*3] = c_33;

//	c_10 = A[1+bs*0];
//	c_32 = A[3+bs*2];
	c_10 = fact[1];
	c_32 = fact[8];

	c_10 = - c_11*c_10*c_00;
	c_32 = - c_33*c_32*c_22;

	C[0+bs*1] = c_10;
	C[2+bs*3] = c_32;

//	c_20 = A[2+bs*1]*c_10;
//	c_30 = A[3+bs*1]*c_10;
//	c_21 = A[2+bs*1]*c_11;
//	c_31 = A[3+bs*1]*c_11;
//	c_20 = fact[3];
//	c_30 = fact[6];
	c_21 = fact[4];
	c_31 = fact[7];

	c_20 = c_21*c_10;
	c_30 = c_31*c_10;
	c_21 = c_21*c_11;
	c_31 = c_31*c_11;

	c_20 += fact[3]*c_00;
	c_30 += fact[6]*c_00;
//	c_21 += A[2+bs*0]*c_01;
//	c_31 += A[3+bs*0]*c_01;

	d_20 = c_22*c_20;
	d_30 = c_32*c_20;
	d_21 = c_22*c_21;
	d_31 = c_32*c_21;

//	d_20 += c_23*c_30;
	d_30 += c_33*c_30;
//	d_21 += c_23*c_31;
	d_31 += c_33*c_31;

	C[0+bs*2] = - d_20;
	C[0+bs*3] = - d_30;
	C[1+bs*2] = - d_21;
	C[1+bs*3] = - d_31;

	return;

	}



void corner_dtrinv_2x2_lib4(double *fact, double *C)
	{

	const int bs = 4;

	double
		c_00=0.0,
		c_10=0.0, c_11=0.0;

	c_00 = fact[0];
	c_11 = fact[2];

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;

	c_10 = fact[1];

	c_10 = - c_11*c_10*c_00;

	C[0+bs*1] = c_10;

	return;

	}



void kernel_dtrinv_4x4_lib4(int kmax, double *A, double *B, double *C, double *fact)
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
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];
		
	c_00 -= a_0 * b_0;

	c_01 -= a_0 * b_1;

	c_02 -= a_0 * b_2;

	c_03 -= a_0 * b_3;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;

	c_02 -= a_0 * b_2;
	c_12 -= a_1 * b_2;

	c_03 -= a_0 * b_3;
	c_13 -= a_1 * b_3;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;

	c_02 -= a_0 * b_2;
	c_12 -= a_1 * b_2;
	c_22 -= a_2 * b_2;

	c_03 -= a_0 * b_3;
	c_13 -= a_1 * b_3;
	c_23 -= a_2 * b_3;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];
		
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

	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
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


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		b_3 = B[3+bs*3];
		
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
		
		
		A += 16;
		B += 16;

		}

//	c_00 += D[0+bs*0];
//	c_10 += D[1+bs*0];
//	c_20 += D[2+bs*0];
//	c_30 += D[3+bs*0];

//	c_01 += D[0+bs*1];
//	c_11 += D[1+bs*1];
//	c_21 += D[2+bs*1];
//	c_31 += D[3+bs*1];

//	c_02 += D[0+bs*2];
//	c_12 += D[1+bs*2];
//	c_22 += D[2+bs*2];
//	c_32 += D[3+bs*2];

//	c_03 += D[0+bs*3];
//	c_13 += D[1+bs*3];
//	c_23 += D[2+bs*3];
//	c_33 += D[3+bs*3];
	
	// dtrsm
	double
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

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
	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	a_20 = fact[3];
	a_21 = fact[4];
	a_22 = fact[5];
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
	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	a_30 = fact[6];
	a_31 = fact[7];
	a_32 = fact[8];
	a_33 = fact[9];
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
	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;

	}
	
	
	
void kernel_dtrinv_4x2_lib4(int kmax, double *A, double *B, double *C, double *fact)
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
	
	// triangle at the beginning

	// k=0
	a_0 = A[0+bs*0];
		
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
		
	c_00 -= a_0 * b_0;

	c_01 -= a_0 * b_1;


	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;


	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;


	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
		
	c_00 -= a_0 * b_0;
	c_10 -= a_1 * b_0;
	c_20 -= a_2 * b_0;
	c_30 -= a_3 * b_0;

	c_01 -= a_0 * b_1;
	c_11 -= a_1 * b_1;
	c_21 -= a_2 * b_1;
	c_31 -= a_3 * b_1;


	A += 16;
	B += 16;
	k = 4;
		
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}

//	c_00 += D[0+bs*0];
//	c_10 += D[1+bs*0];
//	c_20 += D[2+bs*0];
//	c_30 += D[3+bs*0];

//	c_01 += D[0+bs*1];
//	c_11 += D[1+bs*1];
//	c_21 += D[2+bs*1];
//	c_31 += D[3+bs*1];
	
	// dtrsm
	double
		a_00, a_10, a_11;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

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
	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	}
	
	
	
#if 0
// computes the inverse of a 4x4 lower trinagular matrix, and stores it at an upper triangular matrix
void corner_dtrinv_4x4_lib4_old(double *A, double *C)
	{

	const int bs = 4;

	double
		c_00=0.0,
		c_10=0.0, c_11=0.0,
		c_20=0.0, c_21=0.0, c_22=0.0,
		c_30=0.0, c_31=0.0, c_32=0.0, c_33=0.0,
		d_20=0.0, d_21=0.0,
		d_30=0.0, d_31=0.0;

	c_00 = 1.0/A[0+bs*0];
	c_11 = 1.0/A[1+bs*1];
	c_22 = 1.0/A[2+bs*2];
	c_33 = 1.0/A[3+bs*3];

	C[0+bs*0] = c_00;
	C[1+bs*1] = c_11;
	C[2+bs*2] = c_22;
	C[3+bs*3] = c_33;

	c_10 = - c_11*A[1+bs*0]*c_00;
	c_32 = - c_33*A[3+bs*2]*c_22;

	C[0+bs*1] = c_10;
	C[2+bs*3] = c_32;

	c_20 = A[2+bs*1]*c_10;
	c_30 = A[3+bs*1]*c_10;
	c_21 = A[2+bs*1]*c_11;
	c_31 = A[3+bs*1]*c_11;

	c_20 += A[2+bs*0]*c_00;
	c_30 += A[3+bs*0]*c_00;
//	c_21 += A[2+bs*0]*c_01;
//	c_31 += A[3+bs*0]*c_01;

	d_20 = c_22*c_20;
	d_30 = c_32*c_20;
	d_21 = c_22*c_21;
	d_31 = c_32*c_21;

//	d_20 += c_23*c_30;
	d_30 += c_33*c_30;
//	d_21 += c_23*c_31;
	d_31 += c_33*c_31;

	C[0+bs*2] = - d_20;
	C[0+bs*3] = - d_30;
	C[1+bs*2] = - d_21;
	C[1+bs*3] = - d_31;

	return;

	}
#endif



#if 0
// A_00_inv is the mxm top-left matrix already inverted; A_11_inv is the 4x4 bottom-right matrix already inverted, A_10 it the 4xm matrix to invert, C is the mx4 matrix to write the result
void kernel_dtrinv_4x4_lib4_old(int kmax, double *A_00_inv, double *A_10, double *A_11_inv, double *C, int sdc)
	{

	// assume kmax multile of bs !!!

	const int bs = 4;

	int k;

	double
		temp,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;

	// initial triangle

	a_0 = A_10[0+bs*0];
	a_1 = A_10[1+bs*0];
	a_2 = A_10[2+bs*0];
	a_3 = A_10[3+bs*0];
	
	b_0 = A_00_inv[0+bs*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	a_0 = A_10[0+bs*1];
	a_1 = A_10[1+bs*1];
	a_2 = A_10[2+bs*1];
	a_3 = A_10[3+bs*1];
	
	b_0 = A_00_inv[0+bs*1];
	b_1 = A_00_inv[1+bs*1];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;


	a_0 = A_10[0+bs*2];
	a_1 = A_10[1+bs*2];
	a_2 = A_10[2+bs*2];
	a_3 = A_10[3+bs*2];
	
	b_0 = A_00_inv[0+bs*2];
	b_1 = A_00_inv[1+bs*2];
	b_2 = A_00_inv[2+bs*2];
	
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


	a_0 = A_10[0+bs*3];
	a_1 = A_10[1+bs*3];
	a_2 = A_10[2+bs*3];
	a_3 = A_10[3+bs*3];
	
	b_0 = A_00_inv[0+bs*3];
	b_1 = A_00_inv[1+bs*3];
	b_2 = A_00_inv[2+bs*3];
	b_3 = A_00_inv[3+bs*3];
	
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


	A_10 += 16;
	A_00_inv += 16;

	for(k=4; k<kmax-3; k+=4)
		{
		
		a_0 = A_10[0+bs*0];
		a_1 = A_10[1+bs*0];
		a_2 = A_10[2+bs*0];
		a_3 = A_10[3+bs*0];
		
		b_0 = A_00_inv[0+bs*0];
		b_1 = A_00_inv[1+bs*0];
		b_2 = A_00_inv[2+bs*0];
		b_3 = A_00_inv[3+bs*0];
		
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


		a_0 = A_10[0+bs*1];
		a_1 = A_10[1+bs*1];
		a_2 = A_10[2+bs*1];
		a_3 = A_10[3+bs*1];
		
		b_0 = A_00_inv[0+bs*1];
		b_1 = A_00_inv[1+bs*1];
		b_2 = A_00_inv[2+bs*1];
		b_3 = A_00_inv[3+bs*1];
		
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


		a_0 = A_10[0+bs*2];
		a_1 = A_10[1+bs*2];
		a_2 = A_10[2+bs*2];
		a_3 = A_10[3+bs*2];
		
		b_0 = A_00_inv[0+bs*2];
		b_1 = A_00_inv[1+bs*2];
		b_2 = A_00_inv[2+bs*2];
		b_3 = A_00_inv[3+bs*2];
		
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


		a_0 = A_10[0+bs*3];
		a_1 = A_10[1+bs*3];
		a_2 = A_10[2+bs*3];
		a_3 = A_10[3+bs*3];
		
		b_0 = A_00_inv[0+bs*3];
		b_1 = A_00_inv[1+bs*3];
		b_2 = A_00_inv[2+bs*3];
		b_3 = A_00_inv[3+bs*3];
		
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
		
		
		A_10 += 16;
		A_00_inv += 16;

		}



	// transpose
	temp = c_10;
	c_10 = c_01;
	c_01 = temp;

	temp = c_20;
	c_20 = c_02;
	c_02 = temp;

	temp = c_30;
	c_30 = c_03;
	c_03 = temp;

	temp = c_21;
	c_21 = c_12;
	c_12 = temp;

	temp = c_31;
	c_31 = c_13;
	c_13 = temp;

	temp = c_32;
	c_32 = c_23;
	c_23 = temp;



	// final triangle

	b_3 = A_11_inv[3+bs*3];

	c_03 = c_03 * b_3;
	c_13 = c_13 * b_3;
	c_23 = c_23 * b_3;
	c_33 = c_33 * b_3;


	b_3 = A_11_inv[2+bs*3];
	b_2 = A_11_inv[2+bs*2];

	c_03 += c_02 * b_3;
	c_13 += c_12 * b_3;
	c_23 += c_22 * b_3;
	c_33 += c_32 * b_3;

	c_02 = c_02 * b_2;
	c_12 = c_12 * b_2;
	c_22 = c_22 * b_2;
	c_32 = c_32 * b_2;


	b_3 = A_11_inv[1+bs*3];
	b_2 = A_11_inv[1+bs*2];
	b_1 = A_11_inv[1+bs*1];

	c_03 += c_01 * b_3;
	c_13 += c_11 * b_3;
	c_23 += c_21 * b_3;
	c_33 += c_31 * b_3;

	c_02 += c_01 * b_2;
	c_12 += c_11 * b_2;
	c_22 += c_21 * b_2;
	c_32 += c_31 * b_2;

	c_01 = c_01 * b_1;
	c_11 = c_11 * b_1;
	c_21 = c_21 * b_1;
	c_31 = c_31 * b_1;
	

	b_3 = A_11_inv[0+bs*3];
	b_2 = A_11_inv[0+bs*2];
	b_1 = A_11_inv[0+bs*1];
	b_0 = A_11_inv[0+bs*0];

	c_03 += c_00 * b_3;
	c_13 += c_10 * b_3;
	c_23 += c_20 * b_3;
	c_33 += c_30 * b_3;

	c_02 += c_00 * b_2;
	c_12 += c_10 * b_2;
	c_22 += c_20 * b_2;
	c_32 += c_30 * b_2;

	c_01 += c_00 * b_1;
	c_11 += c_10 * b_1;
	c_21 += c_20 * b_1;
	c_31 += c_30 * b_1;

	c_00 = c_00 * b_0;
	c_10 = c_10 * b_0;
	c_20 = c_20 * b_0;
	c_30 = c_30 * b_0;


	// change sign & store result
	C[0+bs*0] = - c_00;
	C[1+bs*0] = - c_10;
	C[2+bs*0] = - c_20;
	C[3+bs*0] = - c_30;

	C[0+bs*1] = - c_01;
	C[1+bs*1] = - c_11;
	C[2+bs*1] = - c_21;
	C[3+bs*1] = - c_31;

	C[0+bs*2] = - c_02;
	C[1+bs*2] = - c_12;
	C[2+bs*2] = - c_22;
	C[3+bs*2] = - c_32;

	C[0+bs*3] = - c_03;
	C[1+bs*3] = - c_13;
	C[2+bs*3] = - c_23;
	C[3+bs*3] = - c_33;

	}
#endif





