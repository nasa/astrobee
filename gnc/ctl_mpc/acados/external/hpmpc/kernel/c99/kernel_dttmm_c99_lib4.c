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



void corner_dlauum_nt_4x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		d_00, 
		d_10, d_11,
		d_20, d_21, d_22,
		d_30, d_31, d_32, d_33;
	

	// k = 0
	a_0 = A[0+bs*0];
	b_0 = B[0+bs*0];

	d_00 = a_0*b_0;

	// k = 1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	d_00 += a_0*b_0;
	d_10  = a_1*b_0;
	d_11  = a_1*b_1;


	// k = 2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];

	d_00 += a_0*b_0;
	d_10 += a_1*b_0;
	d_11 += a_1*b_1;
	d_20  = a_2*b_0;
	d_21  = a_2*b_1;
	d_22  = a_2*b_2;


	// k = 3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];

	d_00 += a_0*b_0;
	d_10 += a_1*b_0;
	d_11 += a_1*b_1;
	d_20 += a_2*b_0;
	d_21 += a_2*b_1;
	d_22 += a_2*b_2;
	d_30  = a_3*b_0;
	d_31  = a_3*b_1;
	d_32  = a_3*b_2;
	d_33  = a_3*b_3;


	if(alg!=0)
		{
		if(alg==1)
			{
			d_00 = C[0+bs*0] + d_00;
			d_10 = C[1+bs*0] + d_10;
			d_11 = C[1+bs*1] + d_11;
			d_20 = C[2+bs*0] + d_20;
			d_21 = C[2+bs*1] + d_21;
			d_22 = C[2+bs*2] + d_22;
			d_30 = C[3+bs*0] + d_30;
			d_31 = C[3+bs*1] + d_31;
			d_32 = C[3+bs*2] + d_32;
			d_33 = C[3+bs*3] + d_33;
			}
		else
			{
			d_00 = C[0+bs*0] - d_00;
			d_10 = C[1+bs*0] - d_10;
			d_11 = C[1+bs*1] - d_11;
			d_20 = C[2+bs*0] - d_20;
			d_21 = C[2+bs*1] - d_21;
			d_22 = C[2+bs*2] - d_22;
			d_30 = C[3+bs*0] + d_30;
			d_31 = C[3+bs*1] + d_31;
			d_32 = C[3+bs*2] + d_32;
			d_33 = C[3+bs*3] + d_33;
			}
		}
	
	D[0+bs*0] = d_00;
	D[1+bs*0] = d_10;
	D[1+bs*1] = d_11;
	D[2+bs*0] = d_20;
	D[2+bs*1] = d_21;
	D[2+bs*2] = d_22;
	D[3+bs*0] = d_30;
	D[3+bs*1] = d_31;
	D[3+bs*2] = d_32;
	D[3+bs*3] = d_33;

	}



void corner_dlauum_nt_3x3_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0, a_1, a_2,
		b_0, b_1, b_2,
		d_00, 
		d_10, d_11,
		d_20, d_21, d_22;
	

	// k = 0
	a_0 = A[0+bs*0];
	b_0 = B[0+bs*0];

	d_00 = a_0*b_0;

	// k = 1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	d_00 += a_0*b_0;
	d_10  = a_1*b_0;
	d_11  = a_1*b_1;


	// k = 2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];

	d_00 += a_0*b_0;
	d_10 += a_1*b_0;
	d_11 += a_1*b_1;
	d_20  = a_2*b_0;
	d_21  = a_2*b_1;
	d_22  = a_2*b_2;


	if(alg!=0)
		{
		if(alg==1)
			{
			d_00 = C[0+bs*0] + d_00;
			d_10 = C[1+bs*0] + d_10;
			d_11 = C[1+bs*1] + d_11;
			d_20 = C[2+bs*0] + d_20;
			d_21 = C[2+bs*1] + d_21;
			d_22 = C[2+bs*2] + d_22;
			}
		else
			{
			d_00 = C[0+bs*0] - d_00;
			d_10 = C[1+bs*0] - d_10;
			d_11 = C[1+bs*1] - d_11;
			d_20 = C[2+bs*0] - d_20;
			d_21 = C[2+bs*1] - d_21;
			d_22 = C[2+bs*2] - d_22;
			}
		}
	
	D[0+bs*0] = d_00;
	D[1+bs*0] = d_10;
	D[1+bs*1] = d_11;
	D[2+bs*0] = d_20;
	D[2+bs*1] = d_21;
	D[2+bs*2] = d_22;

	}



void corner_dlauum_nt_2x2_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0, a_1,
		b_0, b_1,
		d_00, 
		d_10, d_11;
	

	// k = 0
	a_0 = A[0+bs*0];
	b_0 = B[0+bs*0];

	d_00 = a_0*b_0;

	// k = 1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	d_00 += a_0*b_0;
	d_10  = a_1*b_0;
	d_11  = a_1*b_1;


	if(alg!=0)
		{
		if(alg==1)
			{
			d_00 = C[0+bs*0] + d_00;
			d_10 = C[1+bs*0] + d_10;
			d_11 = C[1+bs*1] + d_11;
			}
		else
			{
			d_00 = C[0+bs*0] - d_00;
			d_10 = C[1+bs*0] - d_10;
			d_11 = C[1+bs*1] - d_11;
			}
		}
	
	D[0+bs*0] = d_00;
	D[1+bs*0] = d_10;
	D[1+bs*1] = d_11;

	}



void corner_dlauum_nt_1x1_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	double
		a_0,
		b_0,
		d_00;
	
	a_0 = A[0];
	b_0 = B[0];

	d_00 = a_0*b_0;

	if(alg!=0)
		{
		if(alg==1)
			{
			d_00 = C[0] + d_00;
			}
		else
			{
			d_00 = C[0] - d_00;
			}
		}
	
	D[0] = d_00;

	}



void kernel_dlauum_nt_4x4_lib4(int kmax, double *A, double *B, int alg, double *C, double *D)
	{

	const int lda = 4;
	const int ldb = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;

	// initial triangles

	// k = 0
	a_0 = A[0+lda*0];
	b_0 = B[0+ldb*0];
		
	c_00 += a_0 * b_0;
	
	// k = 1
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	b_0 = B[0+ldb*1];
	b_1 = B[1+ldb*1];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	c_11 += a_1 * b_1;
	
	// k = 2
	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
	a_2 = A[2+lda*2];
	b_0 = B[0+ldb*2];
	b_1 = B[1+ldb*2];
	b_2 = B[2+ldb*2];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;

	c_22 += a_2 * b_2;
	
	// k = 3
	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
	a_2 = A[2+lda*3];
	a_3 = A[3+lda*3];
	b_0 = B[0+ldb*3];
	b_1 = B[1+ldb*3];
	b_2 = B[2+ldb*3];
	b_3 = B[3+ldb*3];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_33 += a_3 * b_3;

	k = 4;
	A += 16;
	B += 16;

	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];
		b_3 = B[3+ldb*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		b_0 = B[0+ldb*1];
		b_1 = B[1+ldb*1];
		b_2 = B[2+ldb*1];
		b_3 = B[3+ldb*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		b_0 = B[0+ldb*2];
		b_1 = B[1+ldb*2];
		b_2 = B[2+ldb*2];
		b_3 = B[3+ldb*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		b_0 = B[0+ldb*3];
		b_1 = B[1+ldb*3];
		b_2 = B[2+ldb*3];
		b_3 = B[3+ldb*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];
		b_3 = B[3+ldb*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;

		A += 4;
		B += 4;

		}
	
	double 
		d_00,
		d_10, d_11,
		d_20, d_21, d_22,
		d_30, d_31, d_32, d_33;

	// store
	if(alg==0)
		{
		D[0+ldc*0] = c_00;
		D[1+ldc*0] = c_10;
		D[2+ldc*0] = c_20;
		D[3+ldc*0] = c_30;

		D[1+ldc*1] = c_11;
		D[2+ldc*1] = c_21;
		D[3+ldc*1] = c_31;

		D[2+ldc*2] = c_22;
		D[3+ldc*2] = c_32;

		D[3+ldc*3] = c_33;
		}
	else
		{
		d_00 = C[0+ldc*0];
		d_10 = C[1+ldc*0];
		d_20 = C[2+ldc*0];
		d_30 = C[3+ldc*0];
	
		d_11 = C[1+ldc*1];
		d_21 = C[2+ldc*1];
		d_31 = C[3+ldc*1];
	
		d_22 = C[2+ldc*2];
		d_32 = C[3+ldc*2];
	
		d_33 = C[3+ldc*3];

		if(alg==1)
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;
	
			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;
	
			d_22 += c_22;
			d_32 += c_32;
	
			d_33 += c_33;
			}
		else
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;
	
			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;
	
			d_22 -= c_22;
			d_32 -= c_32;
	
			d_33 -= c_33;
			}
	
		D[0+ldc*0] = d_00;
		D[1+ldc*0] = d_10;
		D[2+ldc*0] = d_20;
		D[3+ldc*0] = d_30;

		D[1+ldc*1] = d_11;
		D[2+ldc*1] = d_21;
		D[3+ldc*1] = d_31;

		D[2+ldc*2] = d_22;
		D[3+ldc*2] = d_32;

		D[3+ldc*3] = d_33;

		}
	}

#if ! defined(BLASFEO)


// computes the (lower triangular) diagonal blocks of the symmetric matrix U*U'
void kernel_dsyttmm_ul_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{

	const int lda = 4;
	const int ldb = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;

	// initial triangles

	// k = 0
	a_0 = A[0+lda*0];
	b_0 = B[0+ldb*0];
		
	c_00 += a_0 * b_0;
	
	// k = 1
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	b_0 = B[0+ldb*1];
	b_1 = B[1+ldb*1];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	c_11 += a_1 * b_1;
	
	// k = 2
	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
	a_2 = A[2+lda*2];
	b_0 = B[0+ldb*2];
	b_1 = B[1+ldb*2];
	b_2 = B[2+ldb*2];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;

	c_22 += a_2 * b_2;
	
	// k = 3
	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
	a_2 = A[2+lda*3];
	a_3 = A[3+lda*3];
	b_0 = B[0+ldb*3];
	b_1 = B[1+ldb*3];
	b_2 = B[2+ldb*3];
	b_3 = B[3+ldb*3];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_33 += a_3 * b_3;

	k = 4;
	A += 16;
	B += 16;

	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];
		b_3 = B[3+ldb*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		b_0 = B[0+ldb*1];
		b_1 = B[1+ldb*1];
		b_2 = B[2+ldb*1];
		b_3 = B[3+ldb*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		b_0 = B[0+ldb*2];
		b_1 = B[1+ldb*2];
		b_2 = B[2+ldb*2];
		b_3 = B[3+ldb*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		b_0 = B[0+ldb*3];
		b_1 = B[1+ldb*3];
		b_2 = B[2+ldb*3];
		b_3 = B[3+ldb*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		b_0 = B[0+ldb*0];
		b_1 = B[1+ldb*0];
		b_2 = B[2+ldb*0];
		b_3 = B[3+ldb*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;

		A += 4;
		B += 4;

		}
	
	double 
		d_00,
		d_10, d_11,
		d_20, d_21, d_22,
		d_30, d_31, d_32, d_33;

	// store
	if(alg==0)
		{
		D[0+ldc*0] = c_00;
		D[1+ldc*0] = c_10;
		D[2+ldc*0] = c_20;
		D[3+ldc*0] = c_30;

		D[1+ldc*1] = c_11;
		D[2+ldc*1] = c_21;
		D[3+ldc*1] = c_31;

		D[2+ldc*2] = c_22;
		D[3+ldc*2] = c_32;

		D[3+ldc*3] = c_33;
		}
	else
		{
		d_00 = C[0+ldc*0];
		d_10 = C[1+ldc*0];
		d_20 = C[2+ldc*0];
		d_30 = C[3+ldc*0];
	
		d_11 = C[1+ldc*1];
		d_21 = C[2+ldc*1];
		d_31 = C[3+ldc*1];
	
		d_22 = C[2+ldc*2];
		d_32 = C[3+ldc*2];
	
		d_33 = C[3+ldc*3];

		if(alg==1)
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;
	
			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;
	
			d_22 += c_22;
			d_32 += c_32;
	
			d_33 += c_33;
			}
		else
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;
	
			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;
	
			d_22 -= c_22;
			d_32 -= c_32;
	
			d_33 -= c_33;
			}
	
		D[0+ldc*0] = d_00;
		D[1+ldc*0] = d_10;
		D[2+ldc*0] = d_20;
		D[3+ldc*0] = d_30;

		D[1+ldc*1] = d_11;
		D[2+ldc*1] = d_21;
		D[3+ldc*1] = d_31;

		D[2+ldc*2] = d_22;
		D[3+ldc*2] = d_32;

		D[3+ldc*3] = d_33;

		}
	}



// computes the (lower triangular) diagonal blocks of the symmetric matrix L*L'
void kernel_dsyttmm_lu_nt_4x4_lib4(int kmax, double *A, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		c_22 += a_2 * a_2;
		c_32 += a_3 * a_2;

		c_33 += a_3 * a_3;
		
		
		A += 16;

		}
			
	// clean up at the end
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
		
	c_00 += a_0 * a_0;
	c_10 += a_1 * a_0;
	c_20 += a_2 * a_0;
	c_30 += a_3 * a_0;

	c_11 += a_1 * a_1;
	c_21 += a_2 * a_1;
	c_31 += a_3 * a_1;

	c_22 += a_2 * a_2;
	c_32 += a_3 * a_2;

	c_33 += a_3 * a_3;


	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];

	c_11 += a_1 * a_1;
	c_21 += a_2 * a_1;
	c_31 += a_3 * a_1;

	c_22 += a_2 * a_2;
	c_32 += a_3 * a_2;

	c_33 += a_3 * a_3;


	a_2 = A[2+lda*2];
	a_3 = A[3+lda*2];

	c_22 += a_2 * a_2;
	c_32 += a_3 * a_2;

	c_33 += a_3 * a_3;


	a_3 = A[3+lda*3];

	c_33 += a_3 * a_3;
		
		
	// store

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;

	C[2+ldc*2] = c_22;
	C[3+ldc*2] = c_32;

	C[3+ldc*3] = c_33;
	
	}



// normal-transposed, 2x2 with data packed in 4
void kernel_dsyttmm_lu_nt_2x2_lib4(int kmax, double *A, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1,
		c_00=0, 
		c_10=0, c_11=0; 
		
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;
		
		
		A += 16;

		}
			
	for(; k<kmax-2; k+=2)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		A += 8;

		}
			
	// clean up at the end
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
		
	c_00 += a_0 * a_0;
	c_10 += a_1 * a_0;

	c_11 += a_1 * a_1;


	a_1 = A[1+lda*1];

	c_11 += a_1 * a_1;

		
	// store

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;

	C[1+ldc*1] = c_11;
	
	}



// normal-transposed, 4x4 with data packed in 4
void corner_dttmm_ll_nt_4x4_lib4(double *A, double *B, double *C)
	{
	
	const int bs = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];

	b_0 = B[0+bs*0];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;
	
	// k=1
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];

	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	// k=2
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];

	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	// k=3
	a_3 = A[3+bs*3];

	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];

	c_30 += a_3 * b_0;

	c_31 += a_3 * b_1;

	c_32 += a_3 * b_2;

	c_33 += a_3 * b_3;

	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	C[3+bs*3] = c_33;
	
	}



// normal-transposed, 4x4 with data packed in 4
void kernel_dttmm_ll_nt_4x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int bs = 4;

	int k = 0;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];
	a_1 = A[1+bs*0];
	a_2 = A[2+bs*0];
	a_3 = A[3+bs*0];

	b_0 = B[0+bs*0];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;
	
	// k=1
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

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];

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

	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];

	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
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

	A += 4*bs;
	B += 4*bs;

	k = 4;
	for( ; k<kmax-4; k+=4)
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
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
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
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
		
		A += 4*bs;
		B += 4*bs;

		}

	// k = kmax-4
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
		
	// k = kmax-3
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];
		
	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];
		
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k = kmax-2
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];
		
	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];
		
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k = kmax-1
	a_3 = A[3+bs*3];
		
	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
	b_3 = B[3+bs*3];
		
	c_30 += a_3 * b_0;

	c_31 += a_3 * b_1;

	c_32 += a_3 * b_2;

	c_33 += a_3 * b_3;
		
	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;
	
	}	



// normal-transposed, 4x4 with data packed in 4
void corner_dttmm_uu_nt_4x4_lib4(double *A, double *B, double *C)
	{
	
	const int bs = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		        c_11=0, c_12=0, c_13=0,
		                c_22=0, c_23=0, 
		                        c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];

	c_00 += a_0 * b_0;

	c_01 += a_0 * b_1;

	c_02 += a_0 * b_2;

	c_03 += a_0 * b_3;

	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];

	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];

	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;

	// k=3
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];

	b_3 = B[3+bs*3];

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;
	
	// store result
	C[0+bs*0] = c_00;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;

	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;

	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;
	
	}



// normal-transposed, 4x4 with data packed in 4
void corner_dttmm_uu_nt_2x2_lib4(double *A, double *B, double *C)
	{
	
	const int bs = 4;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		        c_11=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];

	c_00 += a_0 * b_0;

	c_01 += a_0 * b_1;

	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];

	b_1 = B[1+bs*1];

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	// store result
	C[0+bs*0] = c_00;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	
	}



// normal-transposed, 4x4 with data packed in 4
void kernel_dttmm_uu_nt_4x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int bs = 4;

	int k = 0;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];
	b_2 = B[2+bs*0];
	b_3 = B[3+bs*0];

	c_00 += a_0 * b_0;
	
	c_01 += a_0 * b_1;
	
	c_02 += a_0 * b_2;
	
	c_03 += a_0 * b_3;
	
	// k=1
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

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];

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
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];

	b_0 = B[0+bs*3];
	b_1 = B[1+bs*3];
	b_2 = B[2+bs*3];
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

	A += 4*bs;
	B += 4*bs;

	k = 4;
	for( ; k<kmax-4; k+=4)
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


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
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
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		b_2 = B[2+bs*3];
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
		
		A += 4*bs;
		B += 4*bs;

		}

	// k = kmax-4
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
		
	// k = kmax-3
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];
		
	b_1 = B[1+bs*1];
	b_2 = B[2+bs*1];
	b_3 = B[3+bs*1];
		
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

	// k = kmax-2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];
	a_3 = A[3+bs*2];
		
	b_2 = B[2+bs*2];
	b_3 = B[3+bs*2];
		
	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k = kmax-1
	a_0 = A[0+bs*3];
	a_1 = A[1+bs*3];
	a_2 = A[2+bs*3];
	a_3 = A[3+bs*3];
		
	b_3 = B[3+bs*3];
		
	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;
		
	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;

	C[0+bs*2] = c_02;
	C[1+bs*2] = c_12;
	C[2+bs*2] = c_22;
	C[3+bs*2] = c_32;

	C[0+bs*3] = c_03;
	C[1+bs*3] = c_13;
	C[2+bs*3] = c_23;
	C[3+bs*3] = c_33;
	
	}	



// normal-transposed, 4x4 with data packed in 4
void kernel_dttmm_uu_nt_4x2_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int bs = 4;

	int k = 0;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
	
	// k=0
	a_0 = A[0+bs*0];

	b_0 = B[0+bs*0];
	b_1 = B[1+bs*0];

	c_00 += a_0 * b_0;
	
	c_01 += a_0 * b_1;
	
	// k=1
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];

	b_0 = B[0+bs*1];
	b_1 = B[1+bs*1];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	// k=2
	a_0 = A[0+bs*2];
	a_1 = A[1+bs*2];
	a_2 = A[2+bs*2];

	b_0 = B[0+bs*2];
	b_1 = B[1+bs*2];

	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;

	// k=3
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

	A += 4*bs;
	B += 4*bs;

	k = 4;
	for( ; k<kmax-4; k+=4)
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
		
		A += 4*bs;
		B += 4*bs;

		}

	// k = kmax-4
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
		
	// k = kmax-3
	a_0 = A[0+bs*1];
	a_1 = A[1+bs*1];
	a_2 = A[2+bs*1];
	a_3 = A[3+bs*1];
		
	b_1 = B[1+bs*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;
	
	// store result
	C[0+bs*0] = c_00;
	C[1+bs*0] = c_10;
	C[2+bs*0] = c_20;
	C[3+bs*0] = c_30;

	C[0+bs*1] = c_01;
	C[1+bs*1] = c_11;
	C[2+bs*1] = c_21;
	C[3+bs*1] = c_31;
	
	}	

#endif



