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



// normal-transposed, 4x4 with data packed in 4
void kernel_dgemm_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		A_0, A_1, A_2, A_3,
		B_0, B_1, B_2, B_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];
	
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];

	k=0;
	for(; k<kmax-3; k+=4)
		{
		
		A_0 = A[0+bs*1];
		A_1 = A[1+bs*1];
		A_2 = A[2+bs*1];
		A_3 = A[3+bs*1];
		
		B_0 = B[0+bs*1];
		B_1 = B[1+bs*1];
		B_2 = B[2+bs*1];
		B_3 = B[3+bs*1];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_00 += A_0 * B_0;
		c_10 += A_1 * B_0;
		c_20 += A_2 * B_0;
		c_30 += A_3 * B_0;

		c_01 += A_0 * B_1;
		c_11 += A_1 * B_1;
		c_21 += A_2 * B_1;
		c_31 += A_3 * B_1;

		c_02 += A_0 * B_2;
		c_12 += A_1 * B_2;
		c_22 += A_2 * B_2;
		c_32 += A_3 * B_2;

		c_03 += A_0 * B_3;
		c_13 += A_1 * B_3;
		c_23 += A_2 * B_3;
		c_33 += A_3 * B_3;


		A_0 = A[0+bs*3];
		A_1 = A[1+bs*3];
		A_2 = A[2+bs*3];
		A_3 = A[3+bs*3];
		
		B_0 = B[0+bs*3];
		B_1 = B[1+bs*3];
		B_2 = B[2+bs*3];
		B_3 = B[3+bs*3];
		
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


		a_0 = A[0+bs*4];
		a_1 = A[1+bs*4];
		a_2 = A[2+bs*4];
		a_3 = A[3+bs*4];
		
		b_0 = B[0+bs*4];
		b_1 = B[1+bs*4];
		b_2 = B[2+bs*4];
		b_3 = B[3+bs*4];
		
		c_00 += A_0 * B_0;
		c_10 += A_1 * B_0;
		c_20 += A_2 * B_0;
		c_30 += A_3 * B_0;

		c_01 += A_0 * B_1;
		c_11 += A_1 * B_1;
		c_21 += A_2 * B_1;
		c_31 += A_3 * B_1;

		c_02 += A_0 * B_2;
		c_12 += A_1 * B_2;
		c_22 += A_2 * B_2;
		c_32 += A_3 * B_2;

		c_03 += A_0 * B_3;
		c_13 += A_1 * B_3;
		c_23 += A_2 * B_3;
		c_33 += A_3 * B_3;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
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
		
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13,
		d_20, d_21, d_22, d_23,
		d_30, d_31, d_32, d_33;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			D[0+bs*0] = c_00;
			D[1+bs*0] = c_10;
			D[2+bs*0] = c_20;
			D[3+bs*0] = c_30;

			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;

			D[0+bs*2] = c_02;
			D[1+bs*2] = c_12;
			D[2+bs*2] = c_22;
			D[3+bs*2] = c_32;

			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			D[2+bs*3] = c_23;
			D[3+bs*3] = c_33;
			}
		else // transpose D
			{
			D[0+bs*0] = c_00;
			D[1+bs*0] = c_01;
			D[2+bs*0] = c_02;
			D[3+bs*0] = c_03;
			
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			D[3+bs*1] = c_13;

			D[0+bs*2] = c_20;
			D[1+bs*2] = c_21;
			D[2+bs*2] = c_22;
			D[3+bs*2] = c_23;

			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			D[2+bs*3] = c_32;
			D[3+bs*3] = c_33;
			}
		}
	else // D = C +/- A * B'
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			d_20 = C[2+bs*0];
			d_30 = C[3+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_21 = C[2+bs*1];
			d_31 = C[3+bs*1];
			
			d_02 = C[0+bs*2];
			d_12 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_32 = C[3+bs*2];
			
			d_03 = C[0+bs*3];
			d_13 = C[1+bs*3];
			d_23 = C[2+bs*3];
			d_33 = C[3+bs*3];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			d_02 = C[2+bs*0];
			d_03 = C[3+bs*0];
			
			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_12 = C[2+bs*1];
			d_13 = C[3+bs*1];
			
			d_20 = C[0+bs*2];
			d_21 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_23 = C[3+bs*2];
			
			d_30 = C[0+bs*3];
			d_31 = C[1+bs*3];
			d_32 = C[2+bs*3];
			d_33 = C[3+bs*3];
			}
		
		if(alg==1) // D = C + A * B'
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

			d_01 += c_01;
			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;

			d_02 += c_02;
			d_12 += c_12;
			d_22 += c_22;
			d_32 += c_32;

			d_03 += c_03;
			d_13 += c_13;
			d_23 += c_23;
			d_33 += c_33;
			}
		else // D = C - A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;

			d_01 -= c_01;
			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;

			d_02 -= c_02;
			d_12 -= c_12;
			d_22 -= c_22;
			d_32 -= c_32;

			d_03 -= c_03;
			d_13 -= c_13;
			d_23 -= c_23;
			d_33 -= c_33;
			}

		if(td==0) // not transpose D
			{
			D[0+bs*0] = d_00;
			D[1+bs*0] = d_10;
			D[2+bs*0] = d_20;
			D[3+bs*0] = d_30;

			D[0+bs*1] = d_01;
			D[1+bs*1] = d_11;
			D[2+bs*1] = d_21;
			D[3+bs*1] = d_31;

			D[0+bs*2] = d_02;
			D[1+bs*2] = d_12;
			D[2+bs*2] = d_22;
			D[3+bs*2] = d_32;

			D[0+bs*3] = d_03;
			D[1+bs*3] = d_13;
			D[2+bs*3] = d_23;
			D[3+bs*3] = d_33;
			}
		else // transpose D
			{
			D[0+bs*0] = d_00;
			D[1+bs*0] = d_01;
			D[2+bs*0] = d_02;
			D[3+bs*0] = d_03;

			D[0+bs*1] = d_10;
			D[1+bs*1] = d_11;
			D[2+bs*1] = d_12;
			D[3+bs*1] = d_13;

			D[0+bs*2] = d_20;
			D[1+bs*2] = d_21;
			D[2+bs*2] = d_22;
			D[3+bs*2] = d_23;

			D[0+bs*3] = d_30;
			D[1+bs*3] = d_31;
			D[2+bs*3] = d_32;
			D[3+bs*3] = d_33;
			}
		}
	
	}



void kernel_dgemm_nt_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		A_0, A_1, A_2, A_3,
		B_0, B_1, B_2, B_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];
	
	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];

	k=0;
	for(; k<kmax-3; k+=4)
		{
		
		A_0 = A[0+bs*1];
		A_1 = A[1+bs*1];
		A_2 = A[2+bs*1];
		A_3 = A[3+bs*1];
		
		B_0 = B[0+bs*1];
		B_1 = B[1+bs*1];
		B_2 = B[2+bs*1];
		B_3 = B[3+bs*1];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_00 += A_0 * B_0;
		c_10 += A_1 * B_0;
		c_20 += A_2 * B_0;
		c_30 += A_3 * B_0;

		c_01 += A_0 * B_1;
		c_11 += A_1 * B_1;
		c_21 += A_2 * B_1;
		c_31 += A_3 * B_1;

		c_02 += A_0 * B_2;
		c_12 += A_1 * B_2;
		c_22 += A_2 * B_2;
		c_32 += A_3 * B_2;

		c_03 += A_0 * B_3;
		c_13 += A_1 * B_3;
		c_23 += A_2 * B_3;
		c_33 += A_3 * B_3;


		A_0 = A[0+bs*3];
		A_1 = A[1+bs*3];
		A_2 = A[2+bs*3];
		A_3 = A[3+bs*3];
		
		B_0 = B[0+bs*3];
		B_1 = B[1+bs*3];
		B_2 = B[2+bs*3];
		B_3 = B[3+bs*3];
		
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


		a_0 = A[0+bs*4];
		a_1 = A[1+bs*4];
		a_2 = A[2+bs*4];
		a_3 = A[3+bs*4];
		
		b_0 = B[0+bs*4];
		b_1 = B[1+bs*4];
		b_2 = B[2+bs*4];
		b_3 = B[3+bs*4];
		
		c_00 += A_0 * B_0;
		c_10 += A_1 * B_0;
		c_20 += A_2 * B_0;
		c_30 += A_3 * B_0;

		c_01 += A_0 * B_1;
		c_11 += A_1 * B_1;
		c_21 += A_2 * B_1;
		c_31 += A_3 * B_1;

		c_02 += A_0 * B_2;
		c_12 += A_1 * B_2;
		c_22 += A_2 * B_2;
		c_32 += A_3 * B_2;

		c_03 += A_0 * B_3;
		c_13 += A_1 * B_3;
		c_23 += A_2 * B_3;
		c_33 += A_3 * B_3;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
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
		
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13,
		d_20, d_21, d_22, d_23,
		d_30, d_31, d_32, d_33;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else // D = C +/- A * B'
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			d_20 = C[2+bs*0];
			d_30 = C[3+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_21 = C[2+bs*1];
			d_31 = C[3+bs*1];
			
			d_02 = C[0+bs*2];
			d_12 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_32 = C[3+bs*2];
			
			d_03 = C[0+bs*3];
			d_13 = C[1+bs*3];
			d_23 = C[2+bs*3];
			d_33 = C[3+bs*3];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			d_02 = C[2+bs*0];
			d_03 = C[3+bs*0];
			
			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_12 = C[2+bs*1];
			d_13 = C[3+bs*1];
			
			d_20 = C[0+bs*2];
			d_21 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_23 = C[3+bs*2];
			
			d_30 = C[0+bs*3];
			d_31 = C[1+bs*3];
			d_32 = C[2+bs*3];
			d_33 = C[3+bs*3];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;

			c_02 = d_02 + c_02;
			c_12 = d_12 + c_12;
			c_22 = d_22 + c_22;
			c_32 = d_32 + c_32;

			c_03 = d_03 + c_03;
			c_13 = d_13 + c_13;
			c_23 = d_23 + c_23;
			c_33 = d_33 + c_33;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;

			c_02 = d_02 - c_02;
			c_12 = d_12 - c_12;
			c_22 = d_22 - c_22;
			c_32 = d_32 - c_32;

			c_03 = d_03 - c_03;
			c_13 = d_13 - c_13;
			c_23 = d_23 - c_23;
			c_33 = d_33 - c_33;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			D[2+bs*3] = c_23;
			D[3+bs*3] = c_33;
			}
		}
	else // km==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			D[2+bs*3] = c_23;
			}
		}
	return;

	store_t:
	if(kn>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		D[3+bs*0] = c_03;
		
		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_12;
		D[3+bs*1] = c_13;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_23;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			D[2+bs*3] = c_32;
			D[3+bs*3] = c_33;
			}
		}
	else // kn==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		
		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_12;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;
		D[2+bs*2] = c_22;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			D[2+bs*3] = c_32;
			}
		}
	return;

	}



// normal-transposed, 4x2 with data packed in 4
void kernel_dgemm_nt_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 4;

		}
		
	double
		d_00, d_01,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			d_20 = C[2+bs*0];
			d_30 = C[3+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_21 = C[2+bs*1];
			d_31 = C[3+bs*1];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];

			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];

			d_20 = C[0+bs*2];
			d_21 = C[1+bs*2];

			d_30 = C[0+bs*3];
			d_31 = C[1+bs*3];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;
			}
		}
	else // km==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			}
		}
	return;

	store_t:
	if(kn>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;

		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			}
		}
	else // kn==1
		{
		D[0+bs*0] = c_00;

		D[0+bs*1] = c_10;

		D[0+bs*2] = c_20;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			}
		}
	return;

	}



// normal-transposed, 4x2 with data packed in 4
void kernel_dgemm_nt_4x2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nt_4x2_vs_lib4(4, 2, kmax, A, B, C, D, alg, tc, td);
	
	}



// normal-transposed, 2x4 with data packed in 4
void kernel_dgemm_nt_2x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
		b_3 = B[3+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		A += 4;
		B += 4;

		}
		
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			
			d_02 = C[0+bs*2];
			d_12 = C[1+bs*2];
			
			d_03 = C[0+bs*3];
			d_13 = C[1+bs*3];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			d_02 = C[2+bs*0];
			d_03 = C[3+bs*0];

			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_12 = C[2+bs*1];
			d_13 = C[3+bs*1];
			}
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;

			c_02 = d_02 + c_02;
			c_12 = d_12 + c_12;

			c_03 = d_03 + c_03;
			c_13 = d_13 + c_13;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;

			c_02 = d_02 - c_02;
			c_12 = d_12 - c_12;

			c_03 = d_03 - c_03;
			c_13 = d_13 - c_13;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			}
		}
	else // km==1
		{
		D[0+bs*0] = c_00;

		D[0+bs*1] = c_01;

		D[0+bs*2] = c_02;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			}
		}
	return;

	store_t:
	if(kn>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		D[3+bs*0] = c_03;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			D[3+bs*1] = c_13;
			}
		}
	else // kn==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			}
		}
	return;

	}



// normal-transposed, 2x4 with data packed in 4
void kernel_dgemm_nt_2x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
	kernel_dgemm_nt_2x4_vs_lib4(2, 4, kmax, A, B, C, D, alg, tc, td);

	}



// normal-transposed, 2x2 with data packed in 4
void kernel_dgemm_nt_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{
	
//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		A += 4;
		B += 4;

		}
		
	double
		d_00, d_01,
		d_10, d_11;
	
	if(alg==0) // D = A * B'
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			
			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			}
		}
	else // km==1
		{
		D[0+bs*0] = c_00;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			}
		}
	return;

	store_t:
	if(kn>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			}
		}
	else // kn==1
		{
		D[0+bs*0] = c_00;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			}
		}
	return;

	}



// normal-transposed, 2x2 with data packed in 4
void kernel_dgemm_nt_2x2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nt_2x2_vs_lib4(2, 2, kmax, A, B, C, D, alg, tc, td);

	}



// normal-normal, 4x4 with data packed in 4
void kernel_dgemm_nn_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
		
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
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


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
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


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
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
		B += 1;

		}
		
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13,
		d_20, d_21, d_22, d_23,
		d_30, d_31, d_32, d_33;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else // D = C +/- A * B'
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			d_20 = C[2+bs*0];
			d_30 = C[3+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_21 = C[2+bs*1];
			d_31 = C[3+bs*1];
			
			d_02 = C[0+bs*2];
			d_12 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_32 = C[3+bs*2];
			
			d_03 = C[0+bs*3];
			d_13 = C[1+bs*3];
			d_23 = C[2+bs*3];
			d_33 = C[3+bs*3];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			d_02 = C[2+bs*0];
			d_03 = C[3+bs*0];
			
			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_12 = C[2+bs*1];
			d_13 = C[3+bs*1];
			
			d_20 = C[0+bs*2];
			d_21 = C[1+bs*2];
			d_22 = C[2+bs*2];
			d_23 = C[3+bs*2];
			
			d_30 = C[0+bs*3];
			d_31 = C[1+bs*3];
			d_32 = C[2+bs*3];
			d_33 = C[3+bs*3];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;

			c_02 = d_02 + c_02;
			c_12 = d_12 + c_12;
			c_22 = d_22 + c_22;
			c_32 = d_32 + c_32;

			c_03 = d_03 + c_03;
			c_13 = d_13 + c_13;
			c_23 = d_23 + c_23;
			c_33 = d_33 + c_33;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;

			c_02 = d_02 - c_02;
			c_12 = d_12 - c_12;
			c_22 = d_22 - c_22;
			c_32 = d_32 - c_32;

			c_03 = d_03 - c_03;
			c_13 = d_13 - c_13;
			c_23 = d_23 - c_23;
			c_33 = d_33 - c_33;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			D[2+bs*3] = c_23;
			D[3+bs*3] = c_33;
			}
		}
	else // km==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;
		D[2+bs*2] = c_22;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			D[2+bs*3] = c_23;
			}
		}
	return;

	store_t:
	if(kn>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		D[3+bs*0] = c_03;
		
		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_12;
		D[3+bs*1] = c_13;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;
		D[2+bs*2] = c_22;
		D[3+bs*2] = c_23;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			D[2+bs*3] = c_32;
			D[3+bs*3] = c_33;
			}
		}
	else // kn==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		
		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;
		D[2+bs*1] = c_12;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;
		D[2+bs*2] = c_22;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			D[2+bs*3] = c_32;
			}
		}
	return;
	
	}



void kernel_dgemm_nn_4x4_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nn_4x4_vs_lib4(4, 4, kmax, A, B, sdb, C, D, alg, tc, td);
	
	}



// normal-normal, 4x2 with data packed in 4
void kernel_dgemm_nn_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		
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
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 1;

		}
		
	double
		d_00, d_01,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			d_20 = C[2+bs*0];
			d_30 = C[3+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_21 = C[2+bs*1];
			d_31 = C[3+bs*1];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];

			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];

			d_20 = C[0+bs*2];
			d_21 = C[1+bs*2];

			d_30 = C[0+bs*3];
			d_31 = C[1+bs*3];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;
			}
		}
	else // km==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			}
		}
	return;

	store_t:
	if(kn>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;

		D[0+bs*1] = c_10;
		D[1+bs*1] = c_11;

		D[0+bs*2] = c_20;
		D[1+bs*2] = c_21;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			D[1+bs*3] = c_31;
			}
		}
	else // kn==1
		{
		D[0+bs*0] = c_00;

		D[0+bs*1] = c_10;

		D[0+bs*2] = c_20;

		if(km>=4)
			{
			D[0+bs*3] = c_30;
			}
		}
	return;

	}



void kernel_dgemm_nn_4x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nn_4x2_vs_lib4(4, 2, kmax, A, B, sdb, C, D, alg, tc, td);
	
	}



// normal-normal, 2x4 with data packed in 4
void kernel_dgemm_nn_2x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		
		
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
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		A += 4;
		B += 1;

		}
		
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13;
	
	if(alg==0) // D = A * B' , there is no tc
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			
			d_02 = C[0+bs*2];
			d_12 = C[1+bs*2];
			
			d_03 = C[0+bs*3];
			d_13 = C[1+bs*3];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			d_02 = C[2+bs*0];
			d_03 = C[3+bs*0];

			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			d_12 = C[2+bs*1];
			d_13 = C[3+bs*1];
			}
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;

			c_02 = d_02 + c_02;
			c_12 = d_12 + c_12;

			c_03 = d_03 + c_03;
			c_13 = d_13 + c_13;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;

			c_02 = d_02 - c_02;
			c_12 = d_12 - c_12;

			c_03 = d_03 - c_03;
			c_13 = d_13 - c_13;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		D[0+bs*1] = c_01;
		D[1+bs*1] = c_11;

		D[0+bs*2] = c_02;
		D[1+bs*2] = c_12;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			D[1+bs*3] = c_13;
			}
		}
	else // km==1
		{
		D[0+bs*0] = c_00;

		D[0+bs*1] = c_01;

		D[0+bs*2] = c_02;

		if(kn>=4)
			{
			D[0+bs*3] = c_03;
			}
		}
	return;

	store_t:
	if(kn>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;
		D[3+bs*0] = c_03;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			D[3+bs*1] = c_13;
			}
		}
	else // kn==3
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;
		D[2+bs*0] = c_02;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_12;
			}
		}
	return;

	}



void kernel_dgemm_nn_2x4_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{
	
	kernel_dgemm_nn_2x4_vs_lib4(2, 4, kmax, A, B, sdb, C, D, alg, tc, td);

	}



// normal-normal, 2x2 with data packed in 4
void kernel_dgemm_nn_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		A += 4;
		B += 1;

		}
		
	double
		d_00, d_01,
		d_10, d_11;
	
	if(alg==0) // D = A * B'
		{
		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	else 
		{
		if(tc==0) // not transpose C
			{
			d_00 = C[0+bs*0];
			d_10 = C[1+bs*0];
			
			d_01 = C[0+bs*1];
			d_11 = C[1+bs*1];
			}
		else // transpose C
			{
			d_00 = C[0+bs*0];
			d_01 = C[1+bs*0];
			
			d_10 = C[0+bs*1];
			d_11 = C[1+bs*1];
			}
		
		if(alg==1) // D = C + A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_01 = d_01 + c_01;
			c_11 = d_11 + c_11;
			}
		else // D = C - A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_01 = d_01 - c_01;
			c_11 = d_11 - c_11;
			}

		if(td==0) // not transpose D
			{
			goto store_n;
			}
		else // transpose D
			{
			goto store_t;
			}
		}
	
	store_n:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			D[1+bs*1] = c_11;
			}
		}
	else // km==1
		{
		D[0+bs*0] = c_00;

		if(kn>=2)
			{
			D[0+bs*1] = c_01;
			}
		}
	return;

	store_t:
	if(kn>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_01;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			D[1+bs*1] = c_11;
			}
		}
	else // kn==1
		{
		D[0+bs*0] = c_00;

		if(km>=2)
			{
			D[0+bs*1] = c_10;
			}
		}
	return;

	}



void kernel_dgemm_nn_2x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg, int tc, int td)
	{

	kernel_dgemm_nn_2x2_vs_lib4(2, 2, kmax, A, B, sdb, C, D, alg, tc, td);

	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_4_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		b_1 = - B[1];
		b_2 = - B[2];
		b_3 = - B[3];
		}
	else
		{
		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];
		b_3 = B[3];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = a_0 * b_1;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_1;
			c_3 = a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = a_0 * b_2;
			c_1 = a_1 * b_2;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			a_0 = A[0+bs*3];
			a_1 = A[1+bs*3];
			a_2 = A[2+bs*3];
			a_3 = A[3+bs*3];
			
			c_0 = a_0 * b_3;
			c_1 = a_1 * b_3;
			c_2 = a_2 * b_3;
			c_3 = a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			a_0 = A[0+bs*1];
			
			c_0 = a_0 * b_1;

			D[0+bs*1] = c_0;
		

			a_0 = A[0+bs*2];
			
			c_0 = a_0 * b_2;

			D[0+bs*2] = c_0;
		

			a_0 = A[0+bs*3];
			
			c_0 = a_0 * b_3;

			D[0+bs*3] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_1;
			c_3 = C[3+bs*1] + a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;
			c_1 = C[1+bs*2] + a_1 * b_2;
			c_2 = C[2+bs*2] + a_2 * b_2;
			c_3 = C[3+bs*2] + a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			a_0 = A[0+bs*3];
			a_1 = A[1+bs*3];
			a_2 = A[2+bs*3];
			a_3 = A[3+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_3;
			c_1 = C[1+bs*3] + a_1 * b_3;
			c_2 = C[2+bs*3] + a_2 * b_3;
			c_3 = C[3+bs*3] + a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			a_0 = A[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;

			D[0+bs*1] = c_0;
			

			a_0 = A[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;

			D[0+bs*2] = c_0;
			

			a_0 = A[0+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_3;

			D[0+bs*3] = c_0;

	
			A += 1;
			C += 1;
			D += 1;
			
			}

		}
	
	}




// B is the diagonal of a matrix
void kernel_dgemm_diag_right_3_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		b_1 = - B[1];
		b_2 = - B[2];
		}
	else
		{
		b_0 = B[0];
		b_1 = B[1];
		b_2 = B[2];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = a_0 * b_1;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_1;
			c_3 = a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = a_0 * b_2;
			c_1 = a_1 * b_2;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			a_0 = A[0+bs*1];
			
			c_0 = a_0 * b_1;

			D[0+bs*1] = c_0;
		

			a_0 = A[0+bs*2];
			
			c_0 = a_0 * b_2;

			D[0+bs*2] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_1;
			c_3 = C[3+bs*1] + a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;
			c_1 = C[1+bs*2] + a_1 * b_2;
			c_2 = C[2+bs*2] + a_2 * b_2;
			c_3 = C[3+bs*2] + a_3 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			a_0 = A[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;

			D[0+bs*1] = c_0;
			

			a_0 = A[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_2;

			D[0+bs*2] = c_0;
			

			A += 1;
			C += 1;
			D += 1;
			
			}

		}
	
	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_2_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		b_1 = - B[1];
		}
	else
		{
		b_0 = B[0];
		b_1 = B[1];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = a_0 * b_1;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_1;
			c_3 = a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			a_0 = A[0+bs*1];
			
			c_0 = a_0 * b_1;

			D[0+bs*1] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_1;
			c_3 = C[3+bs*1] + a_3 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			a_0 = A[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_1;

			D[0+bs*1] = c_0;
			

			A += 1;
			C += 1;
			D += 1;
			
			}

		}
	
	}



// B is the diagonal of a matrix
void kernel_dgemm_diag_right_1_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		b_0 = - B[0];
		}
	else
		{
		b_0 = B[0];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_0;
			c_2 = a_2 * b_0;
			c_3 = a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			A += 4*sda;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		

			A += 1;
			D += 1;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_0;
			c_2 = C[2+bs*0] + a_2 * b_0;
			c_3 = C[3+bs*0] + a_3 * b_0;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			A += 4*sda;
			C += 4*sdc;
			D += 4*sdd;
			
			}
		for(; k<kmax; k++)
			{
			
			a_0 = A[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			A += 1;
			C += 1;
			D += 1;
			
			}

		}
	
	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		a_0 = - A[0];
		a_1 = - A[1];
		a_2 = - A[2];
		a_3 = - A[3];
		}
	else
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		a_3 = A[3];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;
			c_3 = a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
		
			B += 4;
			D += 4;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;
			c_3 = C[3+bs*0] + a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_2;
			c_3 = C[3+bs*1] + a_3 * b_3;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;
			c_1 = C[1+bs*2] + a_1 * b_1;
			c_2 = C[2+bs*2] + a_2 * b_2;
			c_3 = C[3+bs*2] + a_3 * b_3;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;
			c_1 = C[1+bs*3] + a_1 * b_1;
			c_2 = C[2+bs*3] + a_2 * b_2;
			c_3 = C[3+bs*3] + a_3 * b_3;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;
			c_3 = C[3+bs*0] + a_3 * b_3;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
		
			B += 4;
			C += 4;
			D += 4;
			
			}

		}
	
	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_3_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2,
		b_0, b_1, b_2,
		c_0, c_1, c_2;
		
	if(alg==-1)
		{
		a_0 = - A[0];
		a_1 = - A[1];
		a_2 = - A[2];
		}
	else
		{
		a_0 = A[0];
		a_1 = A[1];
		a_2 = A[2];
		}
		
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;
			c_2 = a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
		
			B += 4;
			D += 4;
			
			}
		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;
			c_1 = C[1+bs*1] + a_1 * b_1;
			c_2 = C[2+bs*1] + a_2 * b_2;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;
			c_1 = C[1+bs*2] + a_1 * b_1;
			c_2 = C[2+bs*2] + a_2 * b_2;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;
			c_1 = C[1+bs*3] + a_1 * b_1;
			c_2 = C[2+bs*3] + a_2 * b_2;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;
			c_2 = C[2+bs*0] + a_2 * b_2;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
		
			B += 4;
			C += 4;
			D += 4;
			
			}

		}
	
	}



// A is the diagonal of a matrix
void kernel_dgemm_diag_left_2_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_0, c_1;
		
	if(alg==-1)
		{
		a_0 = - A[0];
		a_1 = - A[1];
		}
	else
		{
		a_0 = A[0];
		a_1 = A[1];
		}
		
	if(alg==0)
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = a_0 * b_0;
			c_1 = a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
		
			B += 4;
			D += 4;
			
			}

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			

			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;
			c_1 = C[1+bs*1] + a_1 * b_1;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			

			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;
			c_1 = C[1+bs*2] + a_1 * b_1;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			

			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;
			c_1 = C[1+bs*3] + a_1 * b_1;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;
			c_1 = C[1+bs*0] + a_1 * b_1;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
		
			B += 4;
			C += 4;
			D += 4;
			
			}

		}
	
	}


// A is the diagonal of a matrix
void kernel_dgemm_diag_left_1_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int bs = 4;

	int k;

	double
		a_0,
		b_0,
		c_0;
		
	if(alg==-1)
		{
		a_0 = A[0];
		}
	else
		{
		a_0 = A[0];
		}
		
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
			

			b_0 = B[0+bs*1];
			
			c_0 = a_0 * b_0;

			D[0+bs*1] = c_0;
			

			b_0 = B[0+bs*2];
			
			c_0 = a_0 * b_0;

			D[0+bs*2] = c_0;
			

			b_0 = B[0+bs*3];
			
			c_0 = a_0 * b_0;

			D[0+bs*3] = c_0;

			B += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = a_0 * b_0;

			D[0+bs*0] = c_0;
		
			B += 4;
			D += 4;
			
			}
		
		}
	else
		{
		
		for(k=0; k<kmax-3; k+=4)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
			

			b_0 = B[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0;

			D[0+bs*1] = c_0;
			

			b_0 = B[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0;

			D[0+bs*2] = c_0;
			

			b_0 = B[0+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0;

			D[0+bs*3] = c_0;

			B += 16;
			C += 16;
			D += 16;
			
			}
		for(; k<kmax; k++)
			{
			
			b_0 = B[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0;

			D[0+bs*0] = c_0;
		
			B += 4;
			C += 4;
			D += 4;
			
			}

		}
		
	}



