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



void kernel_ssymv_4_lib4(int kmax, int kna, float *A, int sda, float *x_n, float *y_n, float *x_t, float *y_t, int tri, int alg)
	{
	
	if(kmax<=0) 
		return;
	
	const int bs = 4;
	
	int k;
	
	float
		a_00, a_01, a_02, a_03,
		x_n_0, x_n_1, x_n_2, x_n_3, y_n_0,
		x_t_0, y_t_0, y_t_1, y_t_2, y_t_3;
	
	if(alg==1)
		{
		x_n_0 = x_n[0];
		x_n_1 = x_n[1];
		x_n_2 = x_n[2];
		x_n_3 = x_n[3];
		}
	else // alg==-1
		{
		x_n_0 = - x_n[0];
		x_n_1 = - x_n[1];
		x_n_2 = - x_n[2];
		x_n_3 = - x_n[3];
		}

	y_t_0 = 0;
	y_t_1 = 0;
	y_t_2 = 0;
	y_t_3 = 0;
	
	k=0;

	// corner
	if(tri==1)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
/*		a_01 = A[0+bs*1];*/
/*		a_02 = A[0+bs*2];*/
/*		a_03 = A[0+bs*3];*/
		
/*		y_n_0 += a_00 * x_n_0;*/
		y_t_0 += a_00 * x_t_0;
/*		y_n_0 += a_01 * x_n_1;*/
/*		y_t_1 += a_01 * x_t_0;*/
/*		y_n_0 += a_02 * x_n_2;*/
/*		y_t_2 += a_02 * x_t_0;*/
/*		y_n_0 += a_03 * x_n_3;*/
/*		y_t_3 += a_03 * x_t_0;*/
		
		y_n[0] = y_n_0;


		y_n_0 = y_n[1];
		x_t_0 = x_t[1];
		
		a_00 = A[1+bs*0];
		a_01 = A[1+bs*1];
/*		a_02 = A[1+bs*2];*/
/*		a_03 = A[1+bs*3];*/
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
/*		y_n_0 += a_01 * x_n_1;*/
		y_t_1 += a_01 * x_t_0;
/*		y_n_0 += a_02 * x_n_2;*/
/*		y_t_2 += a_02 * x_t_0;*/
/*		y_n_0 += a_03 * x_n_3;*/
/*		y_t_3 += a_03 * x_t_0;*/
		
		y_n[1] = y_n_0;

		
		y_n_0 = y_n[2];
		x_t_0 = x_t[2];
		
		a_00 = A[2+bs*0];
		a_01 = A[2+bs*1];
		a_02 = A[2+bs*2];
/*		a_03 = A[2+bs*3];*/
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
/*		y_n_0 += a_02 * x_n_2;*/
		y_t_2 += a_02 * x_t_0;
/*		y_n_0 += a_03 * x_n_3;*/
/*		y_t_3 += a_03 * x_t_0;*/
		
		y_n[2] = y_n_0;


		y_n_0 = y_n[3];
		x_t_0 = x_t[3];
		
		a_00 = A[3+bs*0];
		a_01 = A[3+bs*1];
		a_02 = A[3+bs*2];
		a_03 = A[3+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
/*		y_n_0 += a_03 * x_n_3;*/
		y_t_3 += a_03 * x_t_0;
		
		y_n[3] = y_n_0;


		A += 4;
		y_n += 4;
		x_t += 4;

		k += 4;

		}
	for(; k<kna; k++)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;
		
		y_n[0] = y_n_0;

	
		A += 1;
		y_n += 1;
		x_t += 1;
		
		}
	if(kna>0 || tri==1)
		{
		A += (sda-1)*bs;
		}
	for(; k<kmax-3; k+=bs)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;
		
		y_n[0] = y_n_0;


		y_n_0 = y_n[1];
		x_t_0 = x_t[1];
		
		a_00 = A[1+bs*0];
		a_01 = A[1+bs*1];
		a_02 = A[1+bs*2];
		a_03 = A[1+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;
		
		y_n[1] = y_n_0;

		
		y_n_0 = y_n[2];
		x_t_0 = x_t[2];
		
		a_00 = A[2+bs*0];
		a_01 = A[2+bs*1];
		a_02 = A[2+bs*2];
		a_03 = A[2+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;
		
		y_n[2] = y_n_0;


		y_n_0 = y_n[3];
		x_t_0 = x_t[3];
		
		a_00 = A[3+bs*0];
		a_01 = A[3+bs*1];
		a_02 = A[3+bs*2];
		a_03 = A[3+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;
		
		y_n[3] = y_n_0;

		
		A += sda*bs;
		y_n += 4;
		x_t += 4;

		}
	
	for(; k<kmax; k++)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		a_02 = A[0+bs*2];
		a_03 = A[0+bs*3];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		y_n_0 += a_02 * x_n_2;
		y_t_2 += a_02 * x_t_0;
		y_n_0 += a_03 * x_n_3;
		y_t_3 += a_03 * x_t_0;
		
		y_n[0] = y_n_0;

	
		A += 1;
		y_n += 1;
		x_t += 1;
		
		}

	if(alg==1)
		{
		y_t[0] += y_t_0;
		y_t[1] += y_t_1;
		y_t[2] += y_t_2;
		y_t[3] += y_t_3;
		}
	else // alg==-1
		{
		y_t[0] -= y_t_0;
		y_t[1] -= y_t_1;
		y_t[2] -= y_t_2;
		y_t[3] -= y_t_3;
		}
	
	}
	
	
	
void kernel_ssymv_2_lib4(int kmax, int kna, float *A, int sda, float *x_n, float *y_n, float *x_t, float *y_t, int tri, int alg)
	{
	
	if(kmax<=0) 
		return;
	
	const int bs = 4;
	
	int k;
	
	float
		a_00, a_01,
		x_n_0, x_n_1, y_n_0,
		x_t_0, y_t_0, y_t_1;
	
	if(alg==1)
		{
		x_n_0 = x_n[0];
		x_n_1 = x_n[1];
		}
	else // alg==-1
		{
		x_n_0 = - x_n[0];
		x_n_1 = - x_n[1];
		}

	y_t_0 = 0;
	y_t_1 = 0;
	
	k=0;

	// corner
	if(tri==1)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
/*		a_01 = A[0+bs*1];*/
		
/*		y_n_0 += a_00 * x_n_0;*/
		y_t_0 += a_00 * x_t_0;
/*		y_n_0 += a_01 * x_n_1;*/
/*		y_t_1 += a_01 * x_t_0;*/
		
		y_n[0] = y_n_0;


		y_n_0 = y_n[1];
		x_t_0 = x_t[1];
		
		a_00 = A[1+bs*0];
		a_01 = A[1+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
/*		y_n_0 += a_01 * x_n_1;*/
		y_t_1 += a_01 * x_t_0;
		
		y_n[1] = y_n_0;

		
		A += 2;
		y_n += 2;
		x_t += 2;

		k += 2;

		}
	for(; k<kna; k++)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		
		y_n[0] = y_n_0;

	
		A += 1;
		y_n += 1;
		x_t += 1;
		
		}
	if(kna>0 || tri==1)
		{
		A += (sda-1)*bs;
		}
	for(; k<kmax-3; k+=bs)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		
		y_n[0] = y_n_0;


		y_n_0 = y_n[1];
		x_t_0 = x_t[1];
		
		a_00 = A[1+bs*0];
		a_01 = A[1+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		
		y_n[1] = y_n_0;

		
		y_n_0 = y_n[2];
		x_t_0 = x_t[2];
		
		a_00 = A[2+bs*0];
		a_01 = A[2+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		
		y_n[2] = y_n_0;


		y_n_0 = y_n[3];
		x_t_0 = x_t[3];
		
		a_00 = A[3+bs*0];
		a_01 = A[3+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		
		y_n[3] = y_n_0;

		
		A += sda*bs;
		y_n += 4;
		x_t += 4;

		}
	
	for(; k<kmax; k++)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		a_01 = A[0+bs*1];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		y_n_0 += a_01 * x_n_1;
		y_t_1 += a_01 * x_t_0;
		
		y_n[0] = y_n_0;

	
		A += 1;
		y_n += 1;
		x_t += 1;
		
		}

	if(alg==1)
		{
		y_t[0] += y_t_0;
		y_t[1] += y_t_1;
		}
	else // alg==-1
		{
		y_t[0] -= y_t_0;
		y_t[1] -= y_t_1;
		}
	
	}
	
	
	
void kernel_ssymv_1_lib4(int kmax, int kna, float *A, int sda, float *x_n, float *y_n, float *x_t, float *y_t, int tri, int alg)
	{
	
	if(kmax<=0) 
		return;
	
	const int bs = 4;
	
	int k;
	
	float
		a_00,
		x_n_0, y_n_0,
		x_t_0, y_t_0;
	
	if(alg==1)
		{
		x_n_0 = x_n[0];
		}
	else // alg==-1
		{
		x_n_0 = - x_n[0];
		}

	y_t_0 = 0;
	
	k=0;

	// corner
	if(tri==1)
		{
		
/*		y_n_0 = y_n[0];*/
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		
/*		y_n_0 += a_00 * x_n_0;*/
		y_t_0 += a_00 * x_t_0;
		
/*		y_n[0] = y_n_0;*/

		A += 1;
		y_n += 1;
		x_t += 1;

		k += 1;

		}
	for(; k<kna; k++)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		
		y_n[0] = y_n_0;

	
		A += 1;
		y_n += 1;
		x_t += 1;
		
		}
	if(kna>0 || tri==1)
		{
		A += (sda-1)*bs;
		}
	for(; k<kmax-3; k+=bs)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		
		y_n[0] = y_n_0;


		y_n_0 = y_n[1];
		x_t_0 = x_t[1];
		
		a_00 = A[1+bs*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		
		y_n[1] = y_n_0;

		
		y_n_0 = y_n[2];
		x_t_0 = x_t[2];
		
		a_00 = A[2+bs*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		
		y_n[2] = y_n_0;


		y_n_0 = y_n[3];
		x_t_0 = x_t[3];
		
		a_00 = A[3+bs*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		
		y_n[3] = y_n_0;

		
		A += sda*bs;
		y_n += 4;
		x_t += 4;

		}
	
	for(; k<kmax; k++)
		{
		
		y_n_0 = y_n[0];
		x_t_0 = x_t[0];
		
		a_00 = A[0+bs*0];
		
		y_n_0 += a_00 * x_n_0;
		y_t_0 += a_00 * x_t_0;
		
		y_n[0] = y_n_0;

	
		A += 1;
		y_n += 1;
		x_t += 1;
		
		}

	if(alg==1)
		{
		y_t[0] += y_t_0;
		}
	else // alg==-1
		{
		y_t[0] -= y_t_0;
		}
	
	}
	
	
	

