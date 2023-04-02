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

#include "../../include/block_size.h"


#if ! defined(BLASFEO)

void kernel_dsyrk_nt_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	for(k=0; k<kmax-3; k+=4)
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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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
	
	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		d_22 = C[2+bs*2];
		d_32 = C[3+bs*2];
		
		d_33 = C[3+bs*3];
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;

			c_22 = d_22 + c_22;
			c_32 = d_32 + c_32;

			c_33 = d_33 + c_33;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;

			c_22 = d_22 - c_22;
			c_32 = d_32 - c_32;

			c_33 = d_33 - c_33;
			}

		goto store;
		}
	
	store:
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		if(kn>=4)
			D[3+bs*3] = c_33;
		}
	else
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;

		D[2+bs*2] = c_22;
		}

	}



void kernel_dsyrk_nt_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 4;

		}
	double
		d_00,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;
			}

		goto store;
		}

	store:
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		if(kn>=2)
			{
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;
			}
		}
	else
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		if(kn>=2)
			{
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			}
		}

	}



void kernel_dsyrk_nt_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

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

		c_11 += a_1 * b_1;


		A += 4;
		B += 4;

		}
	
	double
		d_00,
		d_10, d_11;
	
	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		
		d_11 = C[1+bs*1];
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_11 = d_11 + c_11;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_11 = d_11 - c_11;
			}

		goto store;
		}

	store:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		if(kn>=2)
			D[1+bs*1] = c_11;
		}
	else
		{
		D[0+bs*0] = c_00;
		}

	}



void kernel_dsyrk_nn_4x4_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		A += 4;
		B += 1;

		}

	double
		d_00,
		d_10, d_11,
		d_20, d_21, d_22,
		d_30, d_31, d_32, d_33;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		D[3+bs*3] = c_33;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		d_22 = C[2+bs*2];
		d_32 = C[3+bs*2];
		
		d_33 = C[3+bs*3];
		
		if(alg==1) // C += A * B'
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
		else // C -= A * B'
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

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;

		D[2+bs*2] = d_22;
		D[3+bs*2] = d_32;

		D[3+bs*3] = d_33;
		}
	
	}



void kernel_dsyrk_nn_4x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 1;

		}
	double
		d_00,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;

			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;
		}

	}



void kernel_dsyrk_nn_2x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

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

		c_11 += a_1 * b_1;


		A += 4;
		B += 1;

		}
	
	double
		d_00,
		d_10, d_11;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		D[1+bs*1] = c_11;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		
		d_11 = C[1+bs*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;

			d_11 += c_11;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;

			d_11 -= c_11;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;

		D[1+bs*1] = d_11;
		}

	}

#endif


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_4_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		a_2 = - Al[2];
		a_3 = - Al[3];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		a_2 = Al[2];
		a_3 = Al[3];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-4; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;


		a_r = Ar[3];
		
		b_3 = B[3+bs*3];
		
		c_3 = a_3 * b_3 * a_r;

		D[3+bs*3] = c_3;

		}
	else
		{

		for(k=0; k<kmax-4; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*0] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*0] + a_3 * b_3 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*1] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*1] + a_3 * b_3 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*2] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*2] + a_3 * b_3 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*3] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*3] + a_3 * b_3 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];

		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*0] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*0] + a_3 * b_3 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*1] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*1] + a_3 * b_3 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_2 = C[2+bs*2] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*2] + a_3 * b_3 * a_r;

		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;


		a_r = Ar[3];
		
		b_3 = B[3+bs*3];
		
		c_3 = C[3+bs*3] + a_3 * b_3 * a_r;

		D[3+bs*3] = c_3;

		}
	
	}



// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_3_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0, a_1, a_2,
		b_0, b_1, b_2,
		c_0, c_1, c_2;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		a_2 = - Al[2];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		a_2 = Al[2];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		
		c_2 = a_2 * b_2 * a_r;

		D[2+bs*2] = c_2;

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*0] + a_2 * b_2 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*1] + a_2 * b_2 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*2] + a_2 * b_2 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*3] + a_2 * b_2 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*0] + a_2 * b_2 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*1] + a_2 * b_2 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		
		c_2 = C[2+bs*2] + a_2 * b_2 * a_r;

		D[2+bs*2] = c_2;

		}
	
	}


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_2_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0, a_1,
		b_0, b_1,
		c_0, c_1;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		
		c_1 = a_1 * b_1 * a_r;

		D[1+bs*1] = c_1;

		}
	else
		{

		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;

		D[1+bs*1] = c_1;


		}
	
	}


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_1_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0,
		b_0,
		c_0;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		}
	else
		{
		a_0 = Al[0];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-1; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*0] = c_0;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*1] = c_0;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*2] = c_0;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*3] = c_0;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		
		c_0 = a_0 * b_0 * a_r;

		D[0+bs*0] = c_0;

		}
	else
		{

		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;

			D[0+bs*0] = c_0;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;

			D[0+bs*1] = c_0;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;

			D[0+bs*2] = c_0;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;

			D[0+bs*3] = c_0;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;

		D[0+bs*0] = c_0;

		}
	
	}



