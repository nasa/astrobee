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



void kernel_strsv_n_8_lib4(int kmax, float *A0, float *A1, float *x, float *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0,
		y_4=0, y_5=0, y_6=0, y_7=0;
	
	k=0;
	for(; k<kmax-7; k+=8)
		{

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A0[0+lda*0] * x_0;
		y_1 += A0[1+lda*0] * x_0;
		y_2 += A0[2+lda*0] * x_0;
		y_3 += A0[3+lda*0] * x_0;
		y_4 += A1[0+lda*0] * x_0;
		y_5 += A1[1+lda*0] * x_0;
		y_6 += A1[2+lda*0] * x_0;
		y_7 += A1[3+lda*0] * x_0;

		y_0 += A0[0+lda*1] * x_1;
		y_1 += A0[1+lda*1] * x_1;
		y_2 += A0[2+lda*1] * x_1;
		y_3 += A0[3+lda*1] * x_1;
		y_4 += A1[0+lda*1] * x_1;
		y_5 += A1[1+lda*1] * x_1;
		y_6 += A1[2+lda*1] * x_1;
		y_7 += A1[3+lda*1] * x_1;

		y_0 += A0[0+lda*2] * x_2;
		y_1 += A0[1+lda*2] * x_2;
		y_2 += A0[2+lda*2] * x_2;
		y_3 += A0[3+lda*2] * x_2;
		y_4 += A1[0+lda*2] * x_2;
		y_5 += A1[1+lda*2] * x_2;
		y_6 += A1[2+lda*2] * x_2;
		y_7 += A1[3+lda*2] * x_2;

		y_0 += A0[0+lda*3] * x_3;
		y_1 += A0[1+lda*3] * x_3;
		y_2 += A0[2+lda*3] * x_3;
		y_3 += A0[3+lda*3] * x_3;
		y_4 += A1[0+lda*3] * x_3;
		y_5 += A1[1+lda*3] * x_3;
		y_6 += A1[2+lda*3] * x_3;
		y_7 += A1[3+lda*3] * x_3;
		
		A0 += 4*lda;
		A1 += 4*lda;
		x += 4;

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A0[0+lda*0] * x_0;
		y_1 += A0[1+lda*0] * x_0;
		y_2 += A0[2+lda*0] * x_0;
		y_3 += A0[3+lda*0] * x_0;
		y_4 += A1[0+lda*0] * x_0;
		y_5 += A1[1+lda*0] * x_0;
		y_6 += A1[2+lda*0] * x_0;
		y_7 += A1[3+lda*0] * x_0;

		y_0 += A0[0+lda*1] * x_1;
		y_1 += A0[1+lda*1] * x_1;
		y_2 += A0[2+lda*1] * x_1;
		y_3 += A0[3+lda*1] * x_1;
		y_4 += A1[0+lda*1] * x_1;
		y_5 += A1[1+lda*1] * x_1;
		y_6 += A1[2+lda*1] * x_1;
		y_7 += A1[3+lda*1] * x_1;

		y_0 += A0[0+lda*2] * x_2;
		y_1 += A0[1+lda*2] * x_2;
		y_2 += A0[2+lda*2] * x_2;
		y_3 += A0[3+lda*2] * x_2;
		y_4 += A1[0+lda*2] * x_2;
		y_5 += A1[1+lda*2] * x_2;
		y_6 += A1[2+lda*2] * x_2;
		y_7 += A1[3+lda*2] * x_2;

		y_0 += A0[0+lda*3] * x_3;
		y_1 += A0[1+lda*3] * x_3;
		y_2 += A0[2+lda*3] * x_3;
		y_3 += A0[3+lda*3] * x_3;
		y_4 += A1[0+lda*3] * x_3;
		y_5 += A1[1+lda*3] * x_3;
		y_6 += A1[2+lda*3] * x_3;
		y_7 += A1[3+lda*3] * x_3;
		
		A0 += 4*lda;
		A1 += 4*lda;
		x += 4;

		}
	
	y_0 = y[0] - y_0;
	y_1 = y[1] - y_1;
	y_2 = y[2] - y_2;
	y_3 = y[3] - y_3;
	y_4 = y[4] - y_4;
	y_5 = y[5] - y_5;
	y_6 = y[6] - y_6;
	y_7 = y[7] - y_7;

	float
		a_00, a_10, a_20, a_30, a_40, a_50, a_60, a_70,
		a_11, a_21, a_31, a_41, a_51, a_61, a_71;
	
	// A_00
	a_00 = A0[0+lda*0];
	a_10 = A0[1+lda*0];
	a_11 = A0[1+lda*1];
	y_0 *= a_00;
	y_1 -= a_10 * y_0;
	y_1 *= a_11;	
	y[0] = y_0;
	y[1] = y_1;

	a_20 = A0[2+lda*0];
	a_30 = A0[3+lda*0];
	a_21 = A0[2+lda*1];
	a_31 = A0[3+lda*1];
	y_2 -= a_20 * y_0;
	y_3 -= a_30 * y_0;
	y_2 -= a_21 * y_1;
	y_3 -= a_31 * y_1;
	
	a_40 = A1[0+lda*0];
	a_50 = A1[1+lda*0];
	a_60 = A1[2+lda*0];
	a_70 = A1[3+lda*0];
	a_41 = A1[0+lda*1];
	a_51 = A1[1+lda*1];
	a_61 = A1[2+lda*1];
	a_71 = A1[3+lda*1];
	y_4 -= a_40 * y_0;
	y_5 -= a_50 * y_0;
	y_6 -= a_60 * y_0;
	y_7 -= a_70 * y_0;
	y_4 -= a_41 * y_1;
	y_5 -= a_51 * y_1;
	y_6 -= a_61 * y_1;
	y_7 -= a_71 * y_1;

	// A_11
	a_00 = A0[2+lda*2];
	a_10 = A0[3+lda*2];
	a_11 = A0[3+lda*3];
	y_2 *= a_00;
	y_3 -= a_10 * y_2;
	y_3 *= a_11;	
	y[2] = y_2;
	y[3] = y_3;

	a_40 = A1[0+lda*2];
	a_50 = A1[1+lda*2];
	a_60 = A1[2+lda*2];
	a_70 = A1[3+lda*2];
	a_41 = A1[0+lda*3];
	a_51 = A1[1+lda*3];
	a_61 = A1[2+lda*3];
	a_71 = A1[3+lda*3];
	y_4 -= a_40 * y_2;
	y_5 -= a_50 * y_2;
	y_6 -= a_60 * y_2;
	y_7 -= a_70 * y_2;
	y_4 -= a_41 * y_3;
	y_5 -= a_51 * y_3;
	y_6 -= a_61 * y_3;
	y_7 -= a_71 * y_3;

	// A_22
	a_00 = A1[0+lda*4];
	a_10 = A1[1+lda*4];
	a_11 = A1[1+lda*5];
	y_4 *= a_00;
	y_5 -= a_10 * y_4;
	y_5 *= a_11;	
	y[4] = y_4;
	y[5] = y_5;

	a_20 = A1[2+lda*4];
	a_30 = A1[3+lda*4];
	a_21 = A1[2+lda*5];
	a_31 = A1[3+lda*5];
	y_6 -= a_20 * y_4;
	y_7 -= a_30 * y_4;
	y_6 -= a_21 * y_5;
	y_7 -= a_31 * y_5;

	// A_33
	a_00 = A1[2+lda*6];
	a_10 = A1[3+lda*6];
	a_11 = A1[3+lda*7];
	y_6 *= a_00;
	y_7 -= a_10 * y_6;
	y_7 *= a_11;	
	y[6] = y_6;
	y[7] = y_7;

	}
	
	
	
void kernel_strsv_n_4_lib4(int kmax, int ksv, float *A, float *x, float *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int lda = 4;
	
	int k;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;
		y_2 += A[2+lda*0] * x_0;
		y_3 += A[3+lda*0] * x_0;

		y_0 += A[0+lda*1] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[2+lda*1] * x_1;
		y_3 += A[3+lda*1] * x_1;

		y_0 += A[0+lda*2] * x_2;
		y_1 += A[1+lda*2] * x_2;
		y_2 += A[2+lda*2] * x_2;
		y_3 += A[3+lda*2] * x_2;

		y_0 += A[0+lda*3] * x_3;
		y_1 += A[1+lda*3] * x_3;
		y_2 += A[2+lda*3] * x_3;
		y_3 += A[3+lda*3] * x_3;
		
		A += 4*lda;
		x += 4;

		}

	y_0 = y[0] - y_0;
	y_1 = y[1] - y_1;
	y_2 = y[2] - y_2;
	y_3 = y[3] - y_3;

	float
		a_00, a_10, a_20, a_30,
		a_11, a_21, a_31;
	
	// a_00
	a_00 = A[0+lda*0];
	a_10 = A[1+lda*0];
	a_20 = A[2+lda*0];
	a_30 = A[3+lda*0];
	y_0 *= a_00;
	y[0] = y_0;
	y_1 -= a_10 * y_0;
	y_2 -= a_20 * y_0;
	y_3 -= a_30 * y_0;

	if(ksv==1)
		{
		y[1] = y_1;
		y[2] = y_2;
		y[3] = y_3;
		return;
		}

	// a_11
	a_11 = A[1+lda*1];
	a_21 = A[2+lda*1];
	a_31 = A[3+lda*1];
	y_1 *= a_11;	
	y[1] = y_1;
	y_2 -= a_21 * y_1;
	y_3 -= a_31 * y_1;

	if(ksv==2)
		{
		y[2] = y_2;
		y[3] = y_3;
		return;
		}

	// a_22
	a_00 = A[2+lda*2];
	a_10 = A[3+lda*2];
	y_2 *= a_00;
	y[2] = y_2;
	y_3 -= a_10 * y_2;

	if(ksv==3)
		{
		y[3] = y_3;
		return;
		}

	// a_33
	a_11 = A[3+lda*3];
	y_3 *= a_11;	
	y[3] = y_3;

	}
	
	
	
void kernel_strsv_t_4_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k;
	
	float *tA, *tx;
	tA = A;
	tx = x;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0;
	
	k=4;
	A += 4 + (sda-1)*lda;
	x += 4;
	for(; k<kmax-4; k+=8)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;
		y_3 += A[0+lda*3] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[1+lda*2] * x_1;
		y_3 += A[1+lda*3] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;
		y_2 += A[2+lda*2] * x_2;
		y_3 += A[2+lda*3] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		y_2 += A[3+lda*2] * x_3;
		y_3 += A[3+lda*3] * x_3;
		
		A += sda*bs;
		x += 4;

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;
		y_3 += A[0+lda*3] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[1+lda*2] * x_1;
		y_3 += A[1+lda*3] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;
		y_2 += A[2+lda*2] * x_2;
		y_3 += A[2+lda*3] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		y_2 += A[3+lda*2] * x_3;
		y_3 += A[3+lda*3] * x_3;
		
		A += sda*bs;
		x += 4;

		}
	for(; k<kmax; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;
		y_3 += A[0+lda*3] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[1+lda*2] * x_1;
		y_3 += A[1+lda*3] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;
		y_2 += A[2+lda*2] * x_2;
		y_3 += A[2+lda*3] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		y_2 += A[3+lda*2] * x_3;
		y_3 += A[3+lda*3] * x_3;
		
		A += sda*bs;
		x += 4;

		}
	
	A = tA;
	x = tx;

	// bottom trinagle
	y_3  = x[3] - y_3;
	y_3 *= A[3+lda*3];
	x[3] = y_3;

	y_2  = x[2] - A[3+lda*2] * y_3 - y_2;
	y_2 *= A[2+lda*2];
	x[2] = y_2;

	// square
	y_0 += A[2+lda*0]*y_2 + A[3+lda*0]*y_3;
	y_1 += A[2+lda*1]*y_2 + A[3+lda*1]*y_3;
		
	// top trinagle
	y_1  = x[1] - y_1;
	y_1 *= A[1+lda*1];
	x[1] = y_1;

	y_0  = x[0] - A[1+lda*0] * y_1 - y_0;
	y_0 *= A[0+lda*0];
	x[0] = y_0;

	}
	
	
	
void kernel_strsv_t_3_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k;
	
	float *tA, *tx;
	tA = A;
	tx = x;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0;
	
	// clean up at the beginning
	x_3 = x[3];

	y_0 += A[3+lda*0] * x_3;
	y_1 += A[3+lda*1] * x_3;
	y_2 += A[3+lda*2] * x_3;

	k=4;
	A += 4 + (sda-1)*lda;
	x += 4;
	for(; k<kmax-4; k+=8)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[1+lda*2] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;
		y_2 += A[2+lda*2] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		y_2 += A[3+lda*2] * x_3;
		
		A += sda*bs;
		x += 4;

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[1+lda*2] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;
		y_2 += A[2+lda*2] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		y_2 += A[3+lda*2] * x_3;
		
		A += sda*bs;
		x += 4;

		}
	for(; k<kmax; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[1+lda*2] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;
		y_2 += A[2+lda*2] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		y_2 += A[3+lda*2] * x_3;
		
		A += sda*bs;
		x += 4;

		}

	A = tA;
	x = tx;

	// bottom trinagle
	y_2  = x[2] - y_2;
	y_2 *= A[2+lda*2];
	x[2] = y_2;

	// square
	y_0 += A[2+lda*0]*y_2;
	y_1 += A[2+lda*1]*y_2;
		
	// top trinagle
	y_1  = x[1] - y_1;
	y_1 *= A[1+lda*1];
	x[1] = y_1;

	y_0  = x[0] - A[1+lda*0] * y_1 - y_0;
	y_0 *= A[0+lda*0];
	x[0] = y_0;

	}
	
	
	
void kernel_strsv_t_2_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k;
	
	float *tA, *tx;
	tA = A;
	tx = x;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0;
	
	// clean up at the beginning
	x_2 = x[2];
	x_3 = x[3];

	y_0 += A[2+lda*0] * x_2;
	y_1 += A[2+lda*1] * x_2;

	y_0 += A[3+lda*0] * x_3;
	y_1 += A[3+lda*1] * x_3;

	k=4;
	A += 4 + (sda-1)*lda;
	x += 4;
	for(; k<kmax-4; k+=8)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		
		A += sda*bs;
		x += 4;

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		
		A += sda*bs;
		x += 4;

		}
	for(; k<kmax; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		
		A += sda*bs;
		x += 4;

		}

	A = tA;
	x = tx;

	// top trinagle
	y_1  = x[1] - y_1;
	y_1 *= A[1+lda*1];
	x[1] = y_1;

	y_0  = x[0] - A[1+lda*0] * y_1 - y_0;
	y_0 *= A[0+lda*0];
	x[0] = y_0;

	}
	
	
	
void kernel_strsv_t_1_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k;
	
	float *tA, *tx;
	tA = A;
	tx = x;

	float
		x_0, x_1, x_2, x_3,
		y_0=0;
	
	// clean up at the beginning
	x_1 = x[1];
	x_2 = x[2];
	x_3 = x[3];

	y_0 += A[1+lda*0] * x_1;
	y_0 += A[2+lda*0] * x_2;
	y_0 += A[3+lda*0] * x_3;

	k=4;
	A += 4 + (sda-1)*lda;
	x += 4;
	for(; k<kmax-4; k+=8)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_0 += A[1+lda*0] * x_1;
		y_0 += A[2+lda*0] * x_2;
		y_0 += A[3+lda*0] * x_3;
		
		A += sda*bs;
		x += 4;

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_0 += A[1+lda*0] * x_1;
		y_0 += A[2+lda*0] * x_2;
		y_0 += A[3+lda*0] * x_3;
		
		A += sda*bs;
		x += 4;

		}
	for(; k<kmax; k+=4)
		{
		
		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_0 += A[1+lda*0] * x_1;
		y_0 += A[2+lda*0] * x_2;
		y_0 += A[3+lda*0] * x_3;
		
		A += sda*bs;
		x += 4;

		}

	A = tA;
	x = tx;

	// top trinagle
	y_0  = x[0] - y_0;
	y_0 *= A[0+lda*0];
	x[0] = y_0;

	}
	
	
	

