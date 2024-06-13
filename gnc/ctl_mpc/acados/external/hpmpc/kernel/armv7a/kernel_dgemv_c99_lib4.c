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



void kernel_dgemv_t_8_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k, ka=kmax;
	
	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0,
		y_4=0, y_5=0, y_6=0, y_7=0;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
#endif		
	__builtin_prefetch( A+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+12 );
#endif		
	__builtin_prefetch( A+16 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+20 );
#endif		
	__builtin_prefetch( A+24 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+28 );
#endif		

	k=0;
	for(; k<ka-bs+1; k+=bs)
		{
		
		__builtin_prefetch( A+sda*bs );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+4 );
#endif		
		__builtin_prefetch( A+sda*bs+8 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+12 );
#endif		
		__builtin_prefetch( A+sda*bs+16 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+20 );
#endif		
		__builtin_prefetch( A+sda*bs+24 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+28 );
#endif		

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];
		
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;
		y_3 += A[0+lda*3] * x_0;
		y_4 += A[0+lda*4] * x_0;
		y_5 += A[0+lda*5] * x_0;
		y_6 += A[0+lda*6] * x_0;
		y_7 += A[0+lda*7] * x_0;

		y_0 += A[1+lda*0] * x_1;
		y_1 += A[1+lda*1] * x_1;
		y_2 += A[1+lda*2] * x_1;
		y_3 += A[1+lda*3] * x_1;
		y_4 += A[1+lda*4] * x_1;
		y_5 += A[1+lda*5] * x_1;
		y_6 += A[1+lda*6] * x_1;
		y_7 += A[1+lda*7] * x_1;
		
		y_0 += A[2+lda*0] * x_2;
		y_1 += A[2+lda*1] * x_2;
		y_2 += A[2+lda*2] * x_2;
		y_3 += A[2+lda*3] * x_2;
		y_4 += A[2+lda*4] * x_2;
		y_5 += A[2+lda*5] * x_2;
		y_6 += A[2+lda*6] * x_2;
		y_7 += A[2+lda*7] * x_2;

		y_0 += A[3+lda*0] * x_3;
		y_1 += A[3+lda*1] * x_3;
		y_2 += A[3+lda*2] * x_3;
		y_3 += A[3+lda*3] * x_3;
		y_4 += A[3+lda*4] * x_3;
		y_5 += A[3+lda*5] * x_3;
		y_6 += A[3+lda*6] * x_3;
		y_7 += A[3+lda*7] * x_3;
		
		A += sda*bs;
		x += 4;

		}
	
	for(; k<ka; k++)
		{
		
		x_0 = x[0];
	
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;
		y_3 += A[0+lda*3] * x_0;
		y_4 += A[0+lda*4] * x_0;
		y_5 += A[0+lda*5] * x_0;
		y_6 += A[0+lda*6] * x_0;
		y_7 += A[0+lda*7] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(alg==0)
		{
		z[0] = y_0;
		z[1] = y_1;
		z[2] = y_2;
		z[3] = y_3;
		z[4] = y_4;
		z[5] = y_5;
		z[6] = y_6;
		z[7] = y_7;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		z[1] = y[1] + y_1;
		z[2] = y[2] + y_2;
		z[3] = y[3] + y_3;
		z[4] = y[4] + y_4;
		z[5] = y[5] + y_5;
		z[6] = y[6] + y_6;
		z[7] = y[7] + y_7;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		z[1] = y[1] - y_1;
		z[2] = y[2] - y_2;
		z[3] = y[3] - y_3;
		z[4] = y[4] - y_4;
		z[5] = y[5] - y_5;
		z[6] = y[6] - y_6;
		z[7] = y[7] - y_7;
		}

	}
	
	
	
void kernel_dgemv_t_4_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k, ka=kmax;
	
	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
#endif		
	__builtin_prefetch( A+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+12 );
#endif		

	k=0;
	for(; k<ka-bs+1; k+=bs)
		{
		
		__builtin_prefetch( A+sda*bs );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+4 );
#endif		
		__builtin_prefetch( A+sda*bs+8 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+12 );
#endif		

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
	
	for(; k<ka; k++)
		{
		
		x_0 = x[0];
	
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;
		y_3 += A[0+lda*3] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(alg==0)
		{
		z[0] = y_0;
		z[1] = y_1;
		z[2] = y_2;
		z[3] = y_3;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		z[1] = y[1] + y_1;
		z[2] = y[2] + y_2;
		z[3] = y[3] + y_3;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		z[1] = y[1] - y_1;
		z[2] = y[2] - y_2;
		z[3] = y[3] - y_3;
		}

	}
	
	
	
void kernel_dgemv_t_3_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k, ka=kmax;
	
	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
#endif		
	__builtin_prefetch( A+8 );

	k=0;
	for(; k<ka-bs+1; k+=bs)
		{
		
		__builtin_prefetch( A+sda*bs );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+4 );
#endif		
		__builtin_prefetch( A+sda*bs+8 );

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
	
	for(; k<ka; k++)
		{
		
		x_0 = x[0];
	
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
		y_2 += A[0+lda*2] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(alg==0)
		{
		y[0] = y_0;
		y[1] = y_1;
		y[2] = y_2;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		z[1] = y[1] + y_1;
		z[2] = y[2] + y_2;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		z[1] = y[1] - y_1;
		z[2] = y[2] - y_2;
		}

	}
	
	
	
void kernel_dgemv_t_2_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k, ka=kmax;
	
	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
#endif		

	k=0;
	for(; k<ka-bs+1; k+=bs)
		{
		
		__builtin_prefetch( A+sda*bs );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+sda*bs+4 );
#endif		

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
	
	for(; k<ka; k++)
		{
		
		x_0 = x[0];
	
		y_0 += A[0+lda*0] * x_0;
		y_1 += A[0+lda*1] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(alg==0)
		{
		y[0] = y_0;
		y[1] = y_1;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		z[1] = y[1] + y_1;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		z[1] = y[1] - y_1;
		}

	}
	
	
	
void kernel_dgemv_t_1_lib4(int kmax, double *A, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	const int bs  = 4;
	
	int
		k, ka=kmax;
	
	double
		x_0, x_1, x_2, x_3,
		y_0=0;
	
	__builtin_prefetch( A );

	k=0;
	for(; k<ka-bs+1; k+=bs)
		{
		
		__builtin_prefetch( A+sda*bs );

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
	
	for(; k<ka; k++)
		{
		
		x_0 = x[0];
	
		y_0 += A[0+lda*0] * x_0;
	
		A += 1;
		x += 1;
		
		}

	if(alg==0)
		{
		y[0] = y_0;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		}

	}
	
	
	
void kernel_dgemv_n_8_vs_lib4(int km, int kmax, double *A0, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	double *A1 = A0 + 4*sda;
	
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+4 );
	__builtin_prefetch( A1+4 );
#endif		
	__builtin_prefetch( A0+8 );
	__builtin_prefetch( A1+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+12 );
	__builtin_prefetch( A1+12 );
#endif		

	const int lda = 4;
	
	int k;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0,
		y_4=0, y_5=0, y_6=0, y_7=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		__builtin_prefetch( A0+4*lda );
		__builtin_prefetch( A1+4*lda );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A0+4*lda+4 );
		__builtin_prefetch( A1+4*lda+4 );
#endif		
		__builtin_prefetch( A0+4*lda+8 );
		__builtin_prefetch( A1+4*lda+8 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A0+4*lda+12 );
		__builtin_prefetch( A1+4*lda+12 );
#endif		

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

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A0[0+lda*0] * x_0;
		y_1 += A0[1+lda*0] * x_0;
		y_2 += A0[2+lda*0] * x_0;
		y_3 += A0[3+lda*0] * x_0;
		y_4 += A1[0+lda*0] * x_0;
		y_5 += A1[1+lda*0] * x_0;
		y_6 += A1[2+lda*0] * x_0;
		y_7 += A1[3+lda*0] * x_0;
		
		A0 += 1*lda;
		A1 += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		goto store;
		}
	else if(alg==1)
		{
		y_0 += y[0];
		y_1 += y[1];
		y_2 += y[2];
		y_3 += y[3];
		y_4 += y[4];
		y_5 += y[5];
		y_6 += y[6];
		y_7 += y[7];

		goto store;
		}
	else // alg==-1
		{
		y_0 = y[0] - y_0;
		y_1 = y[1] - y_1;
		y_2 = y[2] - y_2;
		y_3 = y[3] - y_3;
		y_4 = y[4] - y_4;
		y_5 = y[5] - y_5;
		y_6 = y[6] - y_6;
		y_7 = y[7] - y_7;

		goto store;
		}
	
	store:
	z[0] = y_0;
	z[1] = y_1;
	z[2] = y_2;
	z[3] = y_3;
	if(km>=8)
		{
		z[4] = y_4;
		z[5] = y_5;
		z[6] = y_6;
		z[7] = y_7;
		}
	else
		{
		z[4] = y_4;
		if(km>=6)
			{
			z[5] = y_5;
			if(km>6)
				{
				z[6] = y_6;
				}
			}
		}

	}
	
	
void kernel_dgemv_n_8_lib4(int kmax, double *A0, int sda, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	double *A1 = A0 + 4*sda;
	
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+4 );
	__builtin_prefetch( A1+4 );
#endif		
	__builtin_prefetch( A0+8 );
	__builtin_prefetch( A1+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+12 );
	__builtin_prefetch( A1+12 );
#endif		

	const int lda = 4;
	
	int k;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0,
		y_4=0, y_5=0, y_6=0, y_7=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		__builtin_prefetch( A0+4*lda );
		__builtin_prefetch( A1+4*lda );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A0+4*lda+4 );
		__builtin_prefetch( A1+4*lda+4 );
#endif		
		__builtin_prefetch( A0+4*lda+8 );
		__builtin_prefetch( A1+4*lda+8 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A0+4*lda+12 );
		__builtin_prefetch( A1+4*lda+12 );
#endif		

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

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A0[0+lda*0] * x_0;
		y_1 += A0[1+lda*0] * x_0;
		y_2 += A0[2+lda*0] * x_0;
		y_3 += A0[3+lda*0] * x_0;
		y_4 += A1[0+lda*0] * x_0;
		y_5 += A1[1+lda*0] * x_0;
		y_6 += A1[2+lda*0] * x_0;
		y_7 += A1[3+lda*0] * x_0;
		
		A0 += 1*lda;
		A1 += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		z[0] = y_0;
		z[1] = y_1;
		z[2] = y_2;
		z[3] = y_3;
		z[4] = y_4;
		z[5] = y_5;
		z[6] = y_6;
		z[7] = y_7;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		z[1] = y[1] + y_1;
		z[2] = y[2] + y_2;
		z[3] = y[3] + y_3;
		z[4] = y[4] + y_4;
		z[5] = y[5] + y_5;
		z[6] = y[6] + y_6;
		z[7] = y[7] + y_7;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		z[1] = y[1] - y_1;
		z[2] = y[2] - y_2;
		z[3] = y[3] - y_3;
		z[4] = y[4] - y_4;
		z[5] = y[5] - y_5;
		z[6] = y[6] - y_6;
		z[7] = y[7] - y_7;
		}

	}
	
	
	
void kernel_dgemv_n_4_vs_lib4(int km, int kmax, double *A, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
#endif		
	__builtin_prefetch( A+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+12 );
#endif		

	const int lda = 4;
	
	int k;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0, y_2=0, y_3=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		__builtin_prefetch( A+4*lda );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+4*lda+4 );
#endif		
		__builtin_prefetch( A+4*lda+8 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+4*lda+12 );
#endif		

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

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;
		y_2 += A[2+lda*0] * x_0;
		y_3 += A[3+lda*0] * x_0;
		
		A += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		goto store;
		}
	else if(alg==1)
		{
		y_0 += y[0];
		y_1 += y[1];
		y_2 += y[2];
		y_3 += y[3];

		goto store;
		}
	else // alg==-1
		{
		y_0 = y[0] - y_0;
		y_1 = y[1] - y_1;
		y_2 = y[2] - y_2;
		y_3 = y[3] - y_3;

		goto store;
		}

	store:
	if(km>=4)
		{
		z[0] = y_0;
		z[1] = y_1;
		z[2] = y_2;
		z[3] = y_3;
		}
	else
		{
		z[0] = y_0;
		if(km>=2)
			{
			z[1] = y_1;
			if(km>2)
				{
				z[2] = y_2;
				}
			}
		}

	}
	
	
	
void kernel_dgemv_n_4_lib4(int kmax, double *A, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	kernel_dgemv_n_4_vs_lib4(4, kmax, A, x, y, z, alg);

	}



void kernel_dgemv_n_2_lib4(int kmax, double *A, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
#endif		
	__builtin_prefetch( A+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+12 );
#endif		

	const int lda = 4;
	
	int k;

	double
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		__builtin_prefetch( A+4*lda );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+4*lda+4 );
#endif		
		__builtin_prefetch( A+4*lda+8 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+4*lda+12 );
#endif		

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;

		y_0 += A[0+lda*1] * x_1;
		y_1 += A[1+lda*1] * x_1;

		y_0 += A[0+lda*2] * x_2;
		y_1 += A[1+lda*2] * x_2;

		y_0 += A[0+lda*3] * x_3;
		y_1 += A[1+lda*3] * x_3;
		
		A += 4*lda;
		x += 4;

		}

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;
		
		A += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		y[0] = y_0;
		y[1] = y_1;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		z[1] = y[1] + y_1;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		z[1] = y[1] - y_1;
		}

	}
	
	
	
void kernel_dgemv_n_1_lib4(int kmax, double *A, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0) 
		return;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
#endif		
	__builtin_prefetch( A+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+12 );
#endif		

	const int lda = 4;
	
	int k;

	double
		x_0, x_1, x_2, x_3,
		y_0=0;
	
	k=0;
	for(; k<kmax-3; k+=4)
		{

		__builtin_prefetch( A+4*lda );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+4*lda+4 );
#endif		
		__builtin_prefetch( A+4*lda+8 );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( A+4*lda+12 );
#endif		

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A[0+lda*0] * x_0;
		y_0 += A[0+lda*1] * x_1;
		y_0 += A[0+lda*2] * x_2;
		y_0 += A[0+lda*3] * x_3;
		
		A += 4*lda;
		x += 4;

		}

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A[0+lda*0] * x_0;
		
		A += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		y[0] = y_0;
		}
	else if(alg==1)
		{
		z[0] = y[0] + y_0;
		}
	else // alg==-1
		{
		z[0] = y[0] - y_0;
		}

	}
	
	
	
void kernel_dgemv_diag_lib4(int kmax, double *dA, double *x, double *y, double *z, int alg)
	{

	if(kmax<=0)
		return;
	
	int k;

	if(alg==0)
		{
		k = 0;
		for( ; k<kmax-3; k+=4)
			{

			z[0] = dA[0] * x[0];
			z[1] = dA[1] * x[1];
			z[2] = dA[2] * x[2];
			z[3] = dA[3] * x[3];

			dA += 4;
			x  += 4;
			z  += 4;

			}
		for( ; k<kmax; k++)
			{

			z[0] = dA[0] * x[0];

			dA += 1;
			x  += 1;
			z  += 1;

			}
		}
	else if(alg==1)
		{
		k = 0;
		for( ; k<kmax-3; k+=4)
			{

			z[0] = y[0] + dA[0] * x[0];
			z[1] = y[1] + dA[1] * x[1];
			z[2] = y[2] + dA[2] * x[2];
			z[3] = y[3] + dA[3] * x[3];

			dA += 4;
			x  += 4;
			y  += 4;
			z  += 4;

			}
		for( ; k<kmax; k++)
			{

			z[0] = y[0] + dA[0] * x[0];

			dA += 1;
			x  += 1;
			y  += 1;
			z  += 1;

			}
		}
	else //if(alg==-1)
		{
		k = 0;
		for( ; k<kmax-3; k+=4)
			{

			z[0] = y[0] - dA[0] * x[0];
			z[1] = y[1] - dA[1] * x[1];
			z[2] = y[2] - dA[2] * x[2];
			z[3] = y[3] - dA[3] * x[3];

			dA += 4;
			x  += 4;
			y  += 4;
			z  += 4;

			}
		for( ; k<kmax; k++)
			{

			z[0] = y[0] - dA[0] * x[0];

			dA += 1;
			x  += 1;
			y  += 1;
			z  += 1;

			}

		}
	
	}
			

