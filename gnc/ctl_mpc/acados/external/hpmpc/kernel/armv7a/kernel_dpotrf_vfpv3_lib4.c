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

//#include "../../include/block_size.h"




// new kernels
void kernel_dpotrf_nt_4x4_lib4_new(int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];

		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_33 += C[3+bs*3];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		inv_diag_D[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		inv_diag_D[0] = c_00;
		c_10 = 0.0;
		c_20 = 0.0;
		c_30 = 0.0;
		}
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	D[3+bs*0] = c_30;
		
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		inv_diag_D[1] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		inv_diag_D[1] = c_11;
		c_21 = 0.0;
		c_31 = 0.0;
		}
	D[2+bs*1] = c_21;
	D[3+bs*1] = c_31;

	// third column
	c_22 -= c_20*c_20;
	c_22 -= c_21*c_21;
	c_32 -= c_30*c_20;
	c_32 -= c_31*c_21;
	if(c_22 > 1e-15)
		{
		c_22 = sqrt(c_22);
		D[2+bs*2] = c_22;
		c_22 = 1.0/c_22;
		inv_diag_D[2] = c_22;
		c_32 *= c_22;
		}
	else
		{
		c_22 = 0.0;
		D[2+bs*2] = c_22;
		inv_diag_D[2] = c_22;
		c_32 = 0.0;
		}
	D[3+bs*2] = c_32;
	
	// fourth column
	c_33 -= c_30*c_30;
	c_33 -= c_31*c_31;
	c_33 -= c_32*c_32;
	if(c_33 > 1e-15)
		{
		c_33 = sqrt(c_33);
		D[3+bs*3] = c_33;
		c_33 = 1.0/c_33;
		inv_diag_D[3] = c_33;
		}
	else
		{
		c_33 = 0.0;
		D[3+bs*3] = c_33;
		inv_diag_D[3] = c_33;
		}

	}



void kernel_dsyrk_dpotrf_nt_4x4_lib4_new(int kadd, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];

		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_33 += C[3+bs*3];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		inv_diag_D[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		inv_diag_D[0] = c_00;
		c_10 = 0.0;
		c_20 = 0.0;
		c_30 = 0.0;
		}
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	D[3+bs*0] = c_30;
		
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		inv_diag_D[1] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		inv_diag_D[1] = c_11;
		c_21 = 0.0;
		c_31 = 0.0;
		}
	D[2+bs*1] = c_21;
	D[3+bs*1] = c_31;

	// third column
	c_22 -= c_20*c_20;
	c_22 -= c_21*c_21;
	c_32 -= c_30*c_20;
	c_32 -= c_31*c_21;
	if(c_22 > 1e-15)
		{
		c_22 = sqrt(c_22);
		D[2+bs*2] = c_22;
		c_22 = 1.0/c_22;
		inv_diag_D[2] = c_22;
		c_32 *= c_22;
		}
	else
		{
		c_22 = 0.0;
		D[2+bs*2] = c_22;
		inv_diag_D[2] = c_22;
		c_32 = 0.0;
		}
	D[3+bs*2] = c_32;
	
	// fourth column
	c_33 -= c_30*c_30;
	c_33 -= c_31*c_31;
	c_33 -= c_32*c_32;
	if(c_33 > 1e-15)
		{
		c_33 = sqrt(c_33);
		D[3+bs*3] = c_33;
		c_33 = 1.0/c_33;
		inv_diag_D[3] = c_33;
		}
	else
		{
		c_33 = 0.0;
		D[3+bs*3] = c_33;
		inv_diag_D[3] = c_33;
		}

	}



void kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	k = 0;

	if(kadd>0)
		{

		if(tri_A==1)
			{

			// initial triangle

			if(kadd>=4)
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
				
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
				
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
				b_2 = Bp[2+bs*2];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;

				c_22 += a_2 * b_2;


				// k = 3
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

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;

				c_22 += a_2 * b_2;
				c_32 += a_3 * b_2;

				c_33 += a_3 * b_3;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
					
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
					
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
						
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
						b_2 = Bp[2+bs*2];
						
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						c_22 += a_2 * b_2;

						k  += 1;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];

		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_33 += C[3+bs*3];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		inv_diag_D[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		inv_diag_D[0] = c_00;
		c_10 = 0.0;
		c_20 = 0.0;
		c_30 = 0.0;
		}
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	if(km>=4)
		{
		D[3+bs*0] = c_30;
		}
		
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		inv_diag_D[1] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		inv_diag_D[1] = c_11;
		c_21 = 0.0;
		c_31 = 0.0;
		}
	D[2+bs*1] = c_21;
	if(km>=4)
		{
		D[3+bs*1] = c_31;
		}

	// third column
	c_22 -= c_20*c_20;
	c_22 -= c_21*c_21;
	c_32 -= c_30*c_20;
	c_32 -= c_31*c_21;
	if(c_22 > 1e-15)
		{
		c_22 = sqrt(c_22);
		D[2+bs*2] = c_22;
		c_22 = 1.0/c_22;
		inv_diag_D[2] = c_22;
		c_32 *= c_22;
		}
	else
		{
		c_22 = 0.0;
		D[2+bs*2] = c_22;
		inv_diag_D[2] = c_22;
		c_32 = 0.0;
		}
	if(km>=4)
		{
		D[3+bs*2] = c_32;
		}
	
	if(kn==3 || km==3)
		return;

	// fourth column
	c_33 -= c_30*c_30;
	c_33 -= c_31*c_31;
	c_33 -= c_32*c_32;
	if(c_33 > 1e-15)
		{
		c_33 = sqrt(c_33);
		D[3+bs*3] = c_33;
		c_33 = 1.0/c_33;
		inv_diag_D[3] = c_33;
		}
	else
		{
		c_33 = 0.0;
		D[3+bs*3] = c_33;
		inv_diag_D[3] = c_33;
		}

	}



void kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
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

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
				
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
				
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;


				// k = 3
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

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
					
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
					
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
						
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
						
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						k  += 1;

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

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		inv_diag_D[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		inv_diag_D[0] = c_00;
		c_10 = 0.0;
		c_20 = 0.0;
		c_30 = 0.0;
		}
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	if(km>=4)
		{
		D[3+bs*0] = c_30;
		}
		
	if(kn==1)
		return;

	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		inv_diag_D[1] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		inv_diag_D[1] = c_11;
		c_21 = 0.0;
		c_31 = 0.0;
		}
	D[2+bs*1] = c_21;
	if(km>=4)
		{
		D[3+bs*1] = c_31;
		}

	}


void kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0; 
		
	k = 0;

	if(kadd>0)
		{

		if(tri_A==1)
			{

			// initial triangle

			if(kadd>=4)
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
				
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 3
				a_0 = Ap[0+bs*3];
				a_1 = Ap[1+bs*3];
					
				b_0 = Bp[0+bs*3];
				b_1 = Bp[1+bs*3];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
					
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
					
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
						
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;

						c_11 += a_1 * b_1;

						k  += 1;

						}

					}

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

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

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

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_11 += C[1+bs*1];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		inv_diag_D[0] = c_00;
		c_10 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		inv_diag_D[0] = c_00;
		c_10 = 0.0;
		}
	if(km>=2)
		{
		D[1+bs*0] = c_10;
		}
		
	if(kn==1 || km==1)
		return;

	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		inv_diag_D[1] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		inv_diag_D[1] = c_11;
		}

	}


// old kernels

void kernel_dsyrk_dpotrf_nt_4x4_lib4(int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{
	
	if(kadd>0)
		{
		__builtin_prefetch( Ap );
		__builtin_prefetch( Bp );
#if defined(TARGET_CORTEX_A9)
		__builtin_prefetch( Ap+4 );
		__builtin_prefetch( Bp+4 );
#endif
		}


	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


//	printf("\n%d %d %d\n", ki_add, kl_add, ki_sub);
//	d_print_mat(4, kadd, Ap, 4);
//	d_print_mat(4, kadd, Bp, 4);
//	d_print_mat(4, 4, C, 4);
//	d_print_mat(4, 4, D, 4);


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r3, %0, %1               \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
		"fldd   d0, .DOUBLEZERO          \n\t" // load zero double
		"fcpyd  d1, d0                   \n\t"
		"fcpyd  d2, d0                   \n\t"
		"fcpyd  d3, d0                   \n\t"
		"fcpyd  d4, d0                   \n\t"
		"fcpyd  d5, d0                   \n\t"
		"fcpyd  d6, d0                   \n\t"
		"fcpyd  d7, d0                   \n\t"
		"fcpyd  d8, d0                   \n\t"
		"fcpyd  d9, d0                   \n\t"
		"fcpyd  d10, d0                  \n\t"
		"fcpyd  d11, d0                  \n\t"
		"fcpyd  d12, d0                  \n\t"
		"fcpyd  d13, d0                  \n\t"
		"fcpyd  d14, d0                  \n\t"
		"fcpyd  d15, d0                  \n\t"
		"                                \n\t"
		"b      .DENDZERO                \n\t"
		".align 3                        \n\t"
		".DOUBLEZERO:                    \n\t" // zero double word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".DENDZERO:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    r3, #0                   \n\t"
		"ble    .DCONSIDERSUB            \n\t" // kadd = 0
		"                                \n\t"
		"                                \n\t"
		"pld    [%3, #64]                 \n\t"
		"pld    [%4, #64]                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #96]                \n\t"
		"pld    [%4, #96]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"mov    r3, %0                   \n\t" // ki_add
		"                                \n\t"
		"                                \n\t"
		"fldd   d16, [%3, #0]            \n\t" // prefetch A_even
		"fldd   d17, [%3, #8]            \n\t"
		"fldd   d18, [%3, #16]           \n\t"
		"fldd   d19, [%3, #24]           \n\t"
		"                                \n\t"
		"fldd   d20, [%4, #0]            \n\t" // prefetch B_even
		"fldd   d21, [%4, #8]            \n\t"
		"fldd   d22, [%4, #16]           \n\t"
		"fldd   d23, [%4, #24]           \n\t"
		"                                \n\t"
		"fldd   d24, [%3, #32]           \n\t" // prefetch A_odd
		"fldd   d25, [%3, #40]           \n\t"
		"fldd   d26, [%3, #48]           \n\t"
		"fldd   d27, [%3, #56]           \n\t"
		"                                \n\t"
		"fldd   d28, [%4, #32]           \n\t" // prefetch B_odd
		"fldd   d29, [%4, #40]           \n\t"
		"fldd   d30, [%4, #48]           \n\t"
		"fldd   d31, [%4, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %11, #1                  \n\t"
		"bne    .DCONSIDERLOOPADD        \n\t" // tri != 1
		"                                \n\t"
		"                                \n\t"
		"cmp    %0, #0                   \n\t"
		"ble    .DTRIADD                 \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		"pld    [%3, #128]               \n\t"
		"pld    [%4, #128]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
//		"fmacd  d1, d17, d20             \n\t"
//		"fmacd  d2, d18, d20             \n\t"
//		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%4, #64]           \n\t" // prefetch B_even
		"                                \n\t"
//		"fmacd  d4, d16, d21             \n\t"
//		"fmacd  d5, d17, d21             \n\t"
//		"fmacd  d6, d18, d21             \n\t"
//		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%4, #72]           \n\t"
		"                                \n\t"
//		"fmacd  d8, d16, d22             \n\t"
//		"fmacd  d9, d17, d22             \n\t"
//		"fmacd  d10, d18, d22            \n\t"
//		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%4, #80]           \n\t"
		"                                \n\t"
//		"fmacd  d12, d16, d23            \n\t"
		"fldd   d16, [%3, #64]           \n\t" // prefetch A_even
//		"fmacd  d13, d17, d23            \n\t"
		"fldd   d17, [%3, #72]           \n\t"
//		"fmacd  d14, d18, d23            \n\t"
		"fldd   d18, [%3, #80]           \n\t"
//		"fmacd  d15, d19, d23            \n\t"
		"fldd   d23, [%4, #88]           \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #160]               \n\t"
		"pld    [%4, #160]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"fldd   d19, [%3, #88]           \n\t"
		"fmacd  d1, d25, d28             \n\t"
		"sub    r3, r3, #1               \n\t" // iter++
//		"fmacd  d2, d26, d28             \n\t"
//		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%4, #96]           \n\t" // prefetch B_odd
		"                                \n\t"
//		"fmacd  d4, d24, d29             \n\t"
		"fmacd  d5, d25, d29             \n\t"
//		"fmacd  d6, d26, d29             \n\t"
//		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%4, #104]          \n\t"
		"                                \n\t"
//		"fmacd  d8, d24, d30             \n\t"
//		"fmacd  d9, d25, d30             \n\t"
//		"fmacd  d10, d26, d30            \n\t"
//		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%4, #112]          \n\t"
		"                                \n\t"
//		"fmacd  d12, d24, d31            \n\t"
		"fldd   d24, [%3, #96]           \n\t" // prefetch A_odd
//		"fmacd  d13, d25, d31            \n\t"
		"fldd   d25, [%3, #104]          \n\t"
//		"fmacd  d14, d26, d31            \n\t"
		"fldd   d26, [%3, #112]          \n\t"
//		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%4, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%3, #192]               \n\t"
		"pld    [%4, #192]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d27, [%3, #120]          \n\t"
		"fmacd  d1, d17, d20             \n\t"
		"cmp    r3, #0                   \n\t" // next iter?
		"fmacd  d2, d18, d20             \n\t"
//		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%4, #128]          \n\t" // prefetch B_even
		"                                \n\t"
//		"fmacd  d4, d16, d21             \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fmacd  d6, d18, d21             \n\t"
//		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%4, #136]          \n\t"
		"                                \n\t"
//		"fmacd  d8, d16, d22             \n\t"
//		"fmacd  d9, d17, d22             \n\t"
		"fmacd  d10, d18, d22            \n\t"
//		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%4, #144]          \n\t"
		"                                \n\t"
//		"fmacd  d12, d16, d23            \n\t"
		"fldd   d16, [%3, #128]          \n\t" // prefetch A_even
//		"fmacd  d13, d17, d23            \n\t"
		"fldd   d17, [%3, #136]          \n\t"
//		"fmacd  d14, d18, d23            \n\t"
		"fldd   d18, [%3, #144]          \n\t"
//		"fmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%3, #152]          \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #224]               \n\t"
		"pld    [%4, #224]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"add    %3, %3, #128             \n\t" // increase A
		"fmacd  d1, d25, d28             \n\t"
		"fldd   d23, [%4, #152]          \n\t"
		"fmacd  d2, d26, d28             \n\t"
		"add    %4, %4, #128             \n\t" // increase B
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%4, #32]           \n\t" // prefetch B_odd
		"                                \n\t"
//		"fmacd  d4, d24, d29             \n\t"
		"fmacd  d5, d25, d29             \n\t"
		"fmacd  d6, d26, d29             \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%4, #40]           \n\t"
		"                                \n\t"
//		"fmacd  d8, d24, d30             \n\t"
//		"fmacd  d9, d25, d30             \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%4, #48]           \n\t"
		"                                \n\t"
//		"fmacd  d12, d24, d31            \n\t"
		"fldd   d24, [%3, #32]           \n\t" // prefetch A_odd
//		"fmacd  d13, d25, d31            \n\t"
		"fldd   d25, [%3, #40]           \n\t"
//		"fmacd  d14, d26, d31            \n\t"
		"fldd   d26, [%3, #48]           \n\t"
		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%4, #56]           \n\t"
		"fldd   d27, [%3, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"b       .DCONSIDERLOOPADD       \n\t"
		"                                \n\t"
		"                                \n\t"
		".DTRIADD:                       \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
//		"fmacd  d1, d17, d20             \n\t"
//		"fmacd  d2, d18, d20             \n\t"
//		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%4, #64]           \n\t" // prefetch B_even
		"                                \n\t"
//		"fmacd  d4, d16, d21             \n\t"
//		"fmacd  d5, d17, d21             \n\t"
//		"fmacd  d6, d18, d21             \n\t"
//		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%4, #72]           \n\t"
		"                                \n\t"
//		"fmacd  d8, d16, d22             \n\t"
//		"fmacd  d9, d17, d22             \n\t"
//		"fmacd  d10, d18, d22            \n\t"
//		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%4, #80]           \n\t"
		"                                \n\t"
//		"fmacd  d12, d16, d23            \n\t"
		"fldd   d16, [%3, #64]           \n\t" // prefetch A_even
//		"fmacd  d13, d17, d23            \n\t"
		"fldd   d17, [%3, #72]           \n\t"
//		"fmacd  d14, d18, d23            \n\t"
		"fldd   d18, [%3, #80]           \n\t"
//		"fmacd  d15, d19, d23            \n\t"
		"fldd   d23, [%4, #88]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %1, #2                   \n\t"
		"blt    .DCONSIDERSUB            \n\t" //
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"fldd   d19, [%3, #88]           \n\t"
		"fmacd  d1, d25, d28             \n\t"
//		"fmacd  d2, d26, d28             \n\t"
//		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%4, #96]           \n\t" // prefetch B_odd
		"                                \n\t"
//		"fmacd  d4, d24, d29             \n\t"
		"fmacd  d5, d25, d29             \n\t"
//		"fmacd  d6, d26, d29             \n\t"
//		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%4, #104]          \n\t"
		"                                \n\t"
//		"fmacd  d8, d24, d30             \n\t"
//		"fmacd  d9, d25, d30             \n\t"
//		"fmacd  d10, d26, d30            \n\t"
//		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%4, #112]          \n\t"
		"                                \n\t"
//		"fmacd  d12, d24, d31            \n\t"
		"fldd   d24, [%3, #96]           \n\t" // prefetch A_odd
//		"fmacd  d13, d25, d31            \n\t"
		"fldd   d25, [%3, #104]          \n\t"
//		"fmacd  d14, d26, d31            \n\t"
		"fldd   d26, [%3, #112]          \n\t"
//		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%4, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"beq    .DCONSIDERSUB            \n\t" //
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d27, [%3, #120]          \n\t"
		"fmacd  d1, d17, d20             \n\t"
		"fmacd  d2, d18, d20             \n\t"
//		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%4, #128]          \n\t" // prefetch B_even
		"                                \n\t"
//		"fmacd  d4, d16, d21             \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fmacd  d6, d18, d21             \n\t"
//		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%4, #136]          \n\t"
		"                                \n\t"
//		"fmacd  d8, d16, d22             \n\t"
//		"fmacd  d9, d17, d22             \n\t"
		"fmacd  d10, d18, d22            \n\t"
//		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%4, #144]          \n\t"
		"                                \n\t"
//		"fmacd  d12, d16, d23            \n\t"
		"fldd   d16, [%3, #128]          \n\t" // prefetch A_even
//		"fmacd  d13, d17, d23            \n\t"
		"fldd   d17, [%3, #136]          \n\t"
//		"fmacd  d14, d18, d23            \n\t"
		"fldd   d18, [%3, #144]          \n\t"
//		"fmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%3, #152]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"b      .DCONSIDERSUB            \n\t" //
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLOOPADD:              \n\t" // MAIN LOOP add
		"                                \n\t"
		"                                \n\t"
		"cmp    r3, #0                   \n\t"
		"ble    .DCONSIDERADD            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD:                      \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
		"pld    [%3, #128]               \n\t"
		"pld    [%4, #128]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d16, [%3, #64]           \n\t" // prefetch A_even
		"fmacd  d1, d17, d20             \n\t"
		"fmacd  d2, d18, d20             \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%4, #64]           \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%3, #72]           \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%4, #72]           \n\t"
		"                                \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%3, #80]           \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%4, #80]           \n\t"
		"                                \n\t"
		"fmacd  d15, d19, d23            \n\t"
		"fldd   d23, [%4, #88]           \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #160]               \n\t"
		"pld    [%4, #160]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"fldd   d19, [%3, #88]           \n\t"
		"fmacd  d1, d25, d28             \n\t"
		"sub    r3, r3, #1               \n\t" // iter++
		"fmacd  d2, d26, d28             \n\t"
		"fldd   d24, [%3, #96]           \n\t" // prefetch A_odd
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%4, #96]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fmacd  d5, d25, d29             \n\t"
		"fldd   d25, [%3, #104]          \n\t"
		"fmacd  d6, d26, d29             \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%4, #104]          \n\t"
		"                                \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fldd   d26, [%3, #112]          \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%4, #112]          \n\t"
		"                                \n\t"
		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%4, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%3, #192]               \n\t"
		"pld    [%4, #192]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d27, [%3, #120]          \n\t"
		"fmacd  d1, d17, d20             \n\t"
		"cmp    r3, #0                   \n\t" // next iter?
		"fmacd  d2, d18, d20             \n\t"
		"fldd   d16, [%3, #128]          \n\t" // prefetch A_even
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%4, #128]          \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%3, #136]          \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%4, #136]          \n\t"
		"                                \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%3, #144]          \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%4, #144]          \n\t"
		"                                \n\t"
		"fmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%3, #152]          \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #224]               \n\t"
		"pld    [%4, #224]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"add    %3, %3, #128             \n\t" // increase A
		"fmacd  d1, d25, d28             \n\t"
		"fldd   d23, [%4, #152]          \n\t"
		"fmacd  d2, d26, d28             \n\t"
		"add    %4, %4, #128             \n\t" // increase B
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%4, #32]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fmacd  d5, d25, d29             \n\t"
		"fldd   d24, [%3, #32]           \n\t" // prefetch A_odd
		"fmacd  d6, d26, d29             \n\t"
		"fldd   d25, [%3, #40]           \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%4, #40]           \n\t"
		"                                \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fldd   d26, [%3, #48]           \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%4, #48]           \n\t"
		"                                \n\t"
		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%4, #56]           \n\t"
		"fldd   d27, [%3, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPADD                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD:                  \n\t" // consider left
		"                                \n\t"
		"                                \n\t"
		"mov    r3, %1                   \n\t" // k_left
		"cmp    r3, #0                   \n\t"
		"ble    .DCONSIDERSUB            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT:                     \n\t" // clean up loop
		"                                \n\t"
		"sub    r3, r3, #1               \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d16, [%3, #32]           \n\t" // prefetch A_even
		"fmacd  d1, d17, d20             \n\t"
		"fmacd  d2, d18, d20             \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%4, #32]           \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%3, #40]           \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%4, #40]           \n\t"
		"                                \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%3, #48]           \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%4, #48]           \n\t"
		"                                \n\t"
		"cmp    r3, #0                   \n\t"
		"                                \n\t"
		"fmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%3, #56]           \n\t"
		"add    %3, %3, #32              \n\t"
		"fldd   d23, [%4, #56]           \n\t"
		"add    %4, %4, #32              \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPLEFT               \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"cmp    %2, #0                   \n\t"
		"ble    .DPOSTACC                \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%9, #0]                 \n\t"
		"pld    [%10, #0]                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%9, #32]                \n\t"
		"pld    [%10, #32]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"mov    r3, %2                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
		"fldd   d16, [%9, #0]            \n\t" // prefetch A_even
		"fldd   d17, [%9, #8]            \n\t"
		"fldd   d18, [%9, #16]           \n\t"
		"fldd   d19, [%9, #24]           \n\t"
		"                                \n\t"
		"fldd   d20, [%10, #0]            \n\t" // prefetch B_even
		"fldd   d21, [%10, #8]            \n\t"
		"fldd   d22, [%10, #16]           \n\t"
		"fldd   d23, [%10, #24]           \n\t"
		"                                \n\t"
		"fldd   d24, [%9, #32]           \n\t" // prefetch A_odd
		"fldd   d25, [%9, #40]           \n\t"
		"fldd   d26, [%9, #48]           \n\t"
		"fldd   d27, [%9, #56]           \n\t"
		"                                \n\t"
		"fldd   d28, [%10, #32]           \n\t" // prefetch B_odd
		"fldd   d29, [%10, #40]           \n\t"
		"fldd   d30, [%10, #48]           \n\t"
		"fldd   d31, [%10, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%9, #64]                \n\t"
		"pld    [%10, #64]               \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%9, #96]                \n\t"
		"pld    [%10, #96]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB:                      \n\t" // main loop 2
		"                                \n\t"
		"                                \n\t"
		"pld    [%9, #128]               \n\t"
		"pld    [%10, #128]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fnmacd  d0, d16, d20             \n\t"
		"fldd   d16, [%9, #64]           \n\t" // prefetch A_even
		"fnmacd  d1, d17, d20             \n\t"
		"fnmacd  d2, d18, d20             \n\t"
		"fnmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%10, #64]           \n\t" // prefetch B_even
		"                                \n\t"
		"fnmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%9, #72]           \n\t"
		"fnmacd  d6, d18, d21             \n\t"
		"fnmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%10, #72]           \n\t"
		"                                \n\t"
		"fnmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%9, #80]           \n\t"
		"fnmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%10, #80]           \n\t"
		"                                \n\t"
		"fnmacd  d15, d19, d23            \n\t"
		"fldd   d23, [%10, #88]           \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%9, #160]               \n\t"
		"pld    [%10, #160]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fnmacd  d0, d24, d28             \n\t"
		"fldd   d19, [%9, #88]           \n\t"
		"fnmacd  d1, d25, d28             \n\t"
		"sub    r3, r3, #1               \n\t" // iter++
		"fnmacd  d2, d26, d28             \n\t"
		"fldd   d24, [%9, #96]           \n\t" // prefetch A_odd
		"fnmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%10, #96]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fnmacd  d5, d25, d29             \n\t"
		"fldd   d25, [%9, #104]          \n\t"
		"fnmacd  d6, d26, d29             \n\t"
		"fnmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%10, #104]          \n\t"
		"                                \n\t"
		"fnmacd  d10, d26, d30            \n\t"
		"fldd   d26, [%9, #112]          \n\t"
		"fnmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%10, #112]          \n\t"
		"                                \n\t"
		"fnmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%10, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%9, #192]               \n\t"
		"pld    [%10, #192]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fnmacd  d0, d16, d20             \n\t"
		"fldd   d27, [%9, #120]          \n\t"
		"fnmacd  d1, d17, d20             \n\t"
		"cmp    r3, #0                   \n\t" // next iter?
		"fnmacd  d2, d18, d20             \n\t"
		"fldd   d16, [%9, #128]          \n\t" // prefetch A_even
		"fnmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%10, #128]          \n\t" // prefetch B_even
		"                                \n\t"
		"fnmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%9, #136]          \n\t"
		"fnmacd  d6, d18, d21             \n\t"
		"fnmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%10, #136]          \n\t"
		"                                \n\t"
		"fnmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%9, #144]          \n\t"
		"fnmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%10, #144]          \n\t"
		"                                \n\t"
		"fnmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%9, #152]          \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%9, #224]               \n\t"
		"pld    [%10, #224]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fnmacd  d0, d24, d28             \n\t"
		"add    %9, %9, #128             \n\t" // increase A
		"fnmacd  d1, d25, d28             \n\t"
		"fldd   d23, [%10, #152]          \n\t"
		"fnmacd  d2, d26, d28             \n\t"
		"add    %10, %10, #128             \n\t" // increase B
		"fnmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%10, #32]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fnmacd  d5, d25, d29             \n\t"
		"fldd   d24, [%9, #32]           \n\t" // prefetch A_odd
		"fnmacd  d6, d26, d29             \n\t"
		"fldd   d25, [%9, #40]           \n\t"
		"fnmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%10, #40]           \n\t"
		"                                \n\t"
		"fnmacd  d10, d26, d30            \n\t"
		"fldd   d26, [%9, #48]           \n\t"
		"fnmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%10, #48]           \n\t"
		"                                \n\t"
		"fnmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%10, #56]           \n\t"
		"fldd   d27, [%9, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPSUB                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %8, #1                   \n\t" // alg
		"bne    .DSOLVE                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"fldd   d16, [%5, #0]            \n\t" // load C elements
		"fldd   d17, [%5, #8]            \n\t"
		"fldd   d18, [%5, #16]           \n\t"
		"fldd   d19, [%5, #24]           \n\t"
		"                                \n\t"
		"fldd   d21, [%5, #40]           \n\t"
		"fldd   d22, [%5, #48]           \n\t"
		"fldd   d23, [%5, #56]           \n\t"
		"                                \n\t"
		"fldd   d26, [%5, #80]           \n\t"
		"fldd   d27, [%5, #88]           \n\t"
		"                                \n\t"
		"fldd   d31, [%5, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"faddd  d0, d0, d16              \n\t"
		"faddd  d1, d1, d17              \n\t"
		"faddd  d2, d2, d18              \n\t"
		"faddd  d3, d3, d19              \n\t"
		"                                \n\t"
		"faddd  d5, d5, d21              \n\t"
		"faddd  d6, d6, d22              \n\t"
		"faddd  d7, d7, d23              \n\t"
		"                                \n\t"
		"faddd  d10, d10, d26            \n\t"
		"faddd  d11, d11, d27            \n\t"
		"                                \n\t"
		"faddd  d15, d15, d31            \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE:                        \n\t"
		"                                \n\t"
		"                                \n\t"
		"fconstd d8, #112                \n\t" // 1.0
		"                                \n\t"
		"fldd	d4, .DTHRESHOLD          \n\t" // 1e-15
		"                                \n\t"
		"                                \n\t"
	// first column
		"fcmped	d0, d4                   \n\t"
		"fmstat                          \n\t"
		"ble    .DELSE1                  \n\t"
		"                                \n\t"
		"fsqrtd d0, d0                   \n\t"
		"fstd   d0, [%6, #0]             \n\t"
		"fdivd	d0, d8, d0               \n\t"
		"fmuld	d1, d1, d0               \n\t"
		"fmuld	d2, d2, d0               \n\t"
		"fmuld	d3, d3, d0               \n\t"
		"                                \n\t"
		"b      .DELSE1END               \n\t"
		"                                \n\t"
		".DELSE1:                        \n\t"
		"                                \n\t"
		"fldd	d0, .DTHRESHOLD+8        \n\t"
		"fstd   d0, [%6, #0]             \n\t"
//		"fcpyd	d1, d0                   \n\t"
//		"fcpyd	d2, d0                   \n\t"
//		"fcpyd	d3, d0                   \n\t"
		"                                \n\t"
		".DELSE1END:                     \n\t"
		"                                \n\t"
		"fstd   d1, [%6, #8]             \n\t"
		"fstd   d2, [%6, #16]            \n\t"
		"fstd   d3, [%6, #24]            \n\t"
		"                                \n\t"
		"                                \n\t"
	// second column
		"fnmacd d5, d1, d1               \n\t"
		"fcmped	d5, d4                   \n\t"
		"fmstat                          \n\t"
		"ble    .DELSE2                  \n\t"
		"                                \n\t"
		"fsqrtd d5, d5                   \n\t"
		"fnmacd d6, d1, d2               \n\t"
		"fnmacd d7, d1, d3               \n\t"
		"fstd   d5, [%6, #40]            \n\t"
		"fdivd	d5, d8, d5               \n\t"
		"fmuld	d6, d6, d5               \n\t"
		"fmuld	d7, d7, d5               \n\t"
		"                                \n\t"
		"b      .DELSE2END               \n\t"
		"                                \n\t"
		".DELSE2:                        \n\t"
		"                                \n\t"
		"fldd	d5, .DTHRESHOLD+8        \n\t"
		"fstd   d5, [%6, #40]            \n\t"
//		"fcpyd	d6, d5                   \n\t"
//		"fcpyd	d7, d5                   \n\t"
		"                                \n\t"
		".DELSE2END:                     \n\t"
		"                                \n\t"
		"fstd   d6, [%6, #48]            \n\t"
		"fstd   d7, [%6, #56]            \n\t"
		"                                \n\t"
		"                                \n\t"
	// third column
		"fnmacd d10, d2, d2              \n\t"
		"fnmacd d10, d6, d6              \n\t"
		"fcmped	d10, d4                  \n\t"
		"fmstat                          \n\t"
		"ble    .DELSE3                  \n\t"
		"                                \n\t"
		"fsqrtd d10, d10                 \n\t"
		"fnmacd d11, d6, d7              \n\t"
		"fnmacd d11, d2, d3              \n\t"
		"fstd   d10, [%6, #80]           \n\t"
		"fdivd	d10, d8, d10             \n\t"
		"fmuld	d11, d11, d10            \n\t"
		"                                \n\t"
		"b      .DELSE3END               \n\t"
		"                                \n\t"
		".DELSE3:                        \n\t"
		"                                \n\t"
		"fldd	d10, .DTHRESHOLD+8       \n\t"
		"fstd   d10, [%6, #80]           \n\t"
//		"fcpyd	d11, d10                 \n\t"
		"                                \n\t"
		".DELSE3END:                     \n\t"
		"                                \n\t"
		"fstd   d11, [%6, #88]           \n\t"
		"                                \n\t"
		"                                \n\t"
	// fourth column
		"fnmacd d15, d3, d3              \n\t"
		"fnmacd d15, d7, d7              \n\t"
		"fnmacd d15, d11, d11            \n\t"
		"fcmped	d15, d4                  \n\t"
		"fmstat                          \n\t"
		"ble    .DELSE4                  \n\t"
		"                                \n\t"
		"fsqrtd d15, d15                 \n\t"
		"fstd   d15, [%6, #120]          \n\t"
		"fdivd	d15, d8, d15             \n\t"
		"                                \n\t"
		"b      .DELSE4END               \n\t"
		"                                \n\t"
		".DELSE4:                        \n\t"
		"                                \n\t"
		"fldd	d15, .DTHRESHOLD+8       \n\t"
		"fstd   d15, [%6, #120]          \n\t"
		"                                \n\t"
		".DELSE4END:                     \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"fstd   d0, [%7, #0]             \n\t" // 0
		"fstd   d1, [%7, #8]             \n\t" // 1
		"fstd   d2, [%7, #24]            \n\t" // 3
		"fstd   d3, [%7, #48]            \n\t" // 6
		"fstd   d5, [%7, #16]            \n\t" // 2
		"fstd   d6, [%7, #32]            \n\t" // 4
		"fstd   d7, [%7, #56]            \n\t" // 7
		"fstd   d10, [%7, #40]           \n\t" // 5
		"fstd   d11, [%7, #64]           \n\t" // 8
		"fstd   d15, [%7, #72]           \n\t" // 9
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"b      .DTHRESHOLDEND           \n\t"
		".align 3                        \n\t"
		".DTHRESHOLD:                    \n\t" // 1e-15 double word
		".word  -1629006314              \n\t"
		".word  1020396463               \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".DTHRESHOLDEND:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DEND:                          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (ki_add),		// %0
		  "r" (kl_add),		// %1
		  "r" (ki_sub),		// %2
		  "r" (Ap),			// %3
		  "r" (Bp),			// %4
		  "r" (C),			// %5
		  "r" (D),			// %6
		  "r" (fact),		// %7
		  "r" (alg),			// %8
		  "r" (Am),			// %9
		  "r" (Bm),			// %10
		  "r" (tri)			// %11
		: // register clobber list
		  "r3",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}




void kernel_dsyrk_dpotrf_nt_4x4_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	if(km>=4 && kn>=4)
		kernel_dsyrk_dpotrf_nt_4x4_lib4(tri, kadd, ksub, Ap, Bp, Am, Bm, C, D, fact, alg, fast_rsqrt);

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	k = 0;

	if(kadd>0)
		{

		if(tri==1)
			{

			// initial triangle

			if(kadd>=4)
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
				
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
				
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
				b_2 = Bp[2+bs*2];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;

				c_22 += a_2 * b_2;


				// k = 3
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

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;

				c_22 += a_2 * b_2;
				c_32 += a_3 * b_2;

				c_33 += a_3 * b_3;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
					
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
					
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
						
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
						b_2 = Bp[2+bs*2];
						
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						c_22 += a_2 * b_2;

						k  += 1;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;

			c_22 += a_2 * b_2;
			c_32 += a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;

		c_22 -= a_2 * b_2;
		c_32 -= a_3 * b_2;

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

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];

		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_33 += C[3+bs*3];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00;
		}
	D[1+bs*0] = c_10;
	fact[1] = c_10;
	D[2+bs*0] = c_20;
	fact[3] = c_20;
	if(km>=4)
		{
		D[3+bs*0] = c_30;
		fact[6] = c_30;
		}
		
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		}
	D[2+bs*1] = c_21;
	fact[4] = c_21;
	if(km>=4)
		{
		D[3+bs*1] = c_31;
		fact[7] = c_31;
		}

	// third column
	c_22 -= c_20*c_20;
	c_22 -= c_21*c_21;
	c_32 -= c_30*c_20;
	c_32 -= c_31*c_21;
	if(c_22 > 1e-15)
		{
		c_22 = sqrt(c_22);
		D[2+bs*2] = c_22;
		c_22 = 1.0/c_22;
		fact[5] = c_22;
		c_32 *= c_22;
		}
	else
		{
		c_22 = 0.0;
		D[2+bs*2] = c_22;
		fact[5] = c_22;
		}
	if(km>=4)
		{
		D[3+bs*2] = c_32;
		fact[8] = c_32;
		}
	
	if(kn==3)
		return;

	// fourth column
	c_33 -= c_30*c_30;
	c_33 -= c_31*c_31;
	c_33 -= c_32*c_32;
	if(c_33 > 1e-15)
		{
		c_33 = sqrt(c_33);
		D[3+bs*3] = c_33;
		c_33 = 1.0/c_33;
		fact[9] = c_33;
		}
	else
		{
		c_33 = 0.0;
		D[3+bs*3] = c_33;
		fact[9] = c_33;
		}

	}



void kernel_dsyrk_dpotrf_nt_4x2_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0,  
		c_30=0, c_31=0;

	k = 0;

	if(kadd>0)
		{

		if(tri==1)
			{

			// initial triangle

			if(kadd>=4)
				{
			
				// k = 0
				a_0 = Ap[0+bs*0];
				
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
					
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;


				// k = 3
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

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];
				
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
						
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
						
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
							
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
							
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						k  += 1;

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

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];
		}
	
	// dpotrf
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00;
		}
	D[1+bs*0] = c_10;
	fact[1] = c_10;
	D[2+bs*0] = c_20;
	fact[3] = c_20;
	if(km>=4)
		{
		D[3+bs*0] = c_30;
		fact[6] = c_30;
		}
	
	if(kn==1)
		return;
	
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		}
	D[2+bs*1] = c_21;
	fact[4] = c_21;
	if(km>=4)
		{
		D[3+bs*1] = c_31;
		fact[7] = c_31;
		}

	}



void kernel_dsyrk_dpotrf_nt_2x2_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;

	k = 0;

	if(kadd>0)
		{
		
		if(tri==1)
			{
		
			// initial triangle

			if(kadd>=2)
				{

				// k = 0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				Ap += 8;
				Bp += 8;
				k  += 2;

				}
			else
				{
				// k = 0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;

				k  += 1;

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

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

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

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_11 += C[1+bs*1];
		}
	
	// dpotrf
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		c_10 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00;
		}
	if(km>=2)
		{
		D[1+bs*0] = c_10;
		fact[1] = c_10;
		}

	if(kn==1)
		return;
	
	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		}

	}




#if 0
// A is upper triangular and it coincides with B
void kernel_dtsyrk_dpotrf_nt_4x4_lib4(int km, int kn, int kadd, int ksub, double *A, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	const int bs = 4;
	const int d_ncl = D_NCL;

	int k;

	double
		a_0, a_1, a_2, a_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;

	// initialize loop counter
	k = 0;

	if(kadd>=4)
		{

		// initial triangle

		// k = 0
		a_0 = A[0+bs*0];
		
		c_00 += a_0 * a_0;


		// k = 1
		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		// k = 2
		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;

		c_22 += a_2 * a_2;


		// k = 3
		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
			
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
		k = 4;

				
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
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


			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
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


			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
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


			a_0 = A[0+bs*3];
			a_1 = A[1+bs*3];
			a_2 = A[2+bs*3];
			a_3 = A[3+bs*3];
			
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
		for(; k<kadd; k++)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
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


			A += 4;

			}

		}
	else if(kadd>0)
		{

		// k = 0
		a_0 = A[0+bs*0];
		
		c_00 += a_0 * a_0;

		A += 4;
		k += 1;

		if(kadd>1)
			{

			// k = 1
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;

			c_11 += a_1 * a_1;

			A += 4;
			k += 1;

			if(kadd>2)
				{

				// k = 2
				a_0 = A[0+bs*0];
				a_1 = A[1+bs*0];
				a_2 = A[2+bs*0];
				
				c_00 += a_0 * a_0;
				c_10 += a_1 * a_0;
				c_20 += a_2 * a_0;

				c_11 += a_1 * a_1;
				c_21 += a_2 * a_1;

				c_22 += a_2 * a_2;

				A += 4;
				k += 1;

				}

			}

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			}
		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;

		c_22 -= a_2 * a_2;
		c_32 -= a_3 * a_2;

		c_33 -= a_3 * a_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;

		c_22 -= a_2 * a_2;
		c_32 -= a_3 * a_2;

		c_33 -= a_3 * a_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;

		c_22 -= a_2 * a_2;
		c_32 -= a_3 * a_2;

		c_33 -= a_3 * a_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;

		c_22 -= a_2 * a_2;
		c_32 -= a_3 * a_2;

		c_33 -= a_3 * a_3;
		
		
		A += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];
		c_20 += C[2+bs*0];
		c_30 += C[3+bs*0];

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];

		c_22 += C[2+bs*2];
		c_32 += C[3+bs*2];

		c_33 += C[3+bs*3];
		}
	
	// dpotrf
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00; // store 0.0
		c_00 = 1.0; // continue factorization with 1.0
		}
	c_10 *= c_00;
	c_20 *= c_00;
	c_30 *= c_00;
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	D[3+bs*0] = c_30;
	fact[1] = c_10;
	fact[3] = c_20;
	fact[6] = c_30;
		
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		c_11 = 1.0;
		}
	c_21 *= c_11;
	c_31 *= c_11;
	D[2+bs*1] = c_21;
	D[3+bs*1] = c_31;
	fact[4] = c_21;
	fact[7] = c_31;

	// third column
	c_22 -= c_20*c_20;
	c_22 -= c_21*c_21;
	c_32 -= c_30*c_20;
	c_32 -= c_31*c_21;
	if(c_22 > 1e-15)
		{
		c_22 = sqrt(c_22);
		D[2+bs*2] = c_22;
		c_22 = 1.0/c_22;
		fact[5] = c_22;
		}
	else
		{
		c_22 = 0.0;
		D[2+bs*2] = c_22;
		fact[5] = c_22;
		c_22 = 1.0;
		}
	c_32 *= c_22;
	D[3+bs*2] = c_32;
	fact[8] = c_32;

	// fourth column
	c_33 -= c_30*c_30;
	c_33 -= c_31*c_31;
	c_33 -= c_32*c_32;
	if(c_33 > 1e-15)
		{
		c_33 = sqrt(c_33);
		D[3+bs*3] = c_33;
		c_33 = 1.0/c_33;
		fact[9] = c_33;
		}
	else
		{
		c_33 = 0.0;
		D[3+bs*3] = c_33;
		fact[9] = c_33;
		c_33 = 1.0;
		}
	
	}



// A is upper triangular and it coincides with B
void kernel_dtsyrk_dpotrf_nt_4x2_lib4(int km, int kn, int kadd, int ksub, double *A, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	const int bs = 4;
	const int d_ncl = D_NCL;

	int k;

	double
		a_0, a_1, a_2, a_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0,
		c_30=0, c_31=0;

	// initialize loop counter
	k = 0;

	if(kadd>=4)
		{

		// initial triangle

		// k = 0
		a_0 = A[0+bs*0];
			
		c_00 += a_0 * a_0;


		// k = 1
		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
			
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		// k = 2
		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
			
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;


		// k = 3
		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
			
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;
		c_20 += a_2 * a_0;
		c_30 += a_3 * a_0;

		c_11 += a_1 * a_1;
		c_21 += a_2 * a_1;
		c_31 += a_3 * a_1;

		A += 16;
		k = 4;
			
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;
			c_20 += a_2 * a_0;
			c_30 += a_3 * a_0;

			c_11 += a_1 * a_1;
			c_21 += a_2 * a_1;
			c_31 += a_3 * a_1;


			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			a_2 = A[2+bs*1];
			a_3 = A[3+bs*1];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;
			c_20 += a_2 * a_0;
			c_30 += a_3 * a_0;

			c_11 += a_1 * a_1;
			c_21 += a_2 * a_1;
			c_31 += a_3 * a_1;


			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			a_2 = A[2+bs*2];
			a_3 = A[3+bs*2];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;
			c_20 += a_2 * a_0;
			c_30 += a_3 * a_0;

			c_11 += a_1 * a_1;
			c_21 += a_2 * a_1;
			c_31 += a_3 * a_1;


			a_0 = A[0+bs*3];
			a_1 = A[1+bs*3];
			a_2 = A[2+bs*3];
			a_3 = A[3+bs*3];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;
			c_20 += a_2 * a_0;
			c_30 += a_3 * a_0;

			c_11 += a_1 * a_1;
			c_21 += a_2 * a_1;
			c_31 += a_3 * a_1;
			
			
			A += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			a_2 = A[2+bs*0];
			a_3 = A[3+bs*0];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;
			c_20 += a_2 * a_0;
			c_30 += a_3 * a_0;

			c_11 += a_1 * a_1;
			c_21 += a_2 * a_1;
			c_31 += a_3 * a_1;


			A += 4;

			}

		}
	else if(kadd>0)
		{

		// k = 0
		a_0 = A[0+bs*0];
		
		c_00 += a_0 * a_0;

		A += 4;
		k += 1;

		if(kadd>1)
			{

			// k = 1
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;

			c_11 += a_1 * a_1;

			A += 4;
			k += 1;

			if(kadd>2)
				{

				// k = 2
				a_0 = A[0+bs*0];
				a_1 = A[1+bs*0];
				a_2 = A[2+bs*0];
				
				c_00 += a_0 * a_0;
				c_10 += a_1 * a_0;
				c_20 += a_2 * a_0;

				c_11 += a_1 * a_1;
				c_21 += a_2 * a_1;

				A += 4;
				k += 1;

				}

			}

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			}
		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;
		c_20 -= a_2 * a_0;
		c_30 -= a_3 * a_0;

		c_11 -= a_1 * a_1;
		c_21 -= a_2 * a_1;
		c_31 -= a_3 * a_1;
		
		
		A += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];
		c_20 += C[2+bs*0];
		c_30 += C[3+bs*0];

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];
		}
	
	// dpotrf
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00;
		c_00 = 1.0;
		}
	c_10 *= c_00;
	c_20 *= c_00;
	c_30 *= c_00;
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	D[3+bs*0] = c_30;
	fact[1] = c_10;
	fact[3] = c_20;
	fact[6] = c_30;
	
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		c_11 = 1.0;
		}
	c_21 *= c_11;
	c_31 *= c_11;
	D[2+bs*1] = c_21;
	D[3+bs*1] = c_31;
	fact[4] = c_21;
	fact[7] = c_31;

	}



// A is upper triangular and it coincides with B
void kernel_dtsyrk_dpotrf_nt_2x2_lib4(int km, int kn, int kadd, int ksub, double *A, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	const int bs = 4;
	const int d_ncl = D_NCL;

	int k;

	double
		a_0, a_1, a_2, a_3,
		c_00=0, 
		c_10=0, c_11=0; 

	// initialize loop counter
	k = 0;

	if(kadd>=2)
		{

		// initial triangle

		// k = 0
		a_0 = A[0+bs*0];
			
		c_00 += a_0 * a_0;


		// k = 1
		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
			
		c_00 += a_0 * a_0;
		c_10 += a_1 * a_0;

		c_11 += a_1 * a_1;


		A += 8;
		k = 2;
			
		for(; k<kadd-3; k+=4)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;

			c_11 += a_1 * a_1;


			a_0 = A[0+bs*1];
			a_1 = A[1+bs*1];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;

			c_11 += a_1 * a_1;


			a_0 = A[0+bs*2];
			a_1 = A[1+bs*2];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;

			c_11 += a_1 * a_1;


			a_0 = A[0+bs*3];
			a_1 = A[1+bs*3];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;

			c_11 += a_1 * a_1;
			
			
			A += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = A[0+bs*0];
			a_1 = A[1+bs*0];
			
			c_00 += a_0 * a_0;
			c_10 += a_1 * a_0;

			c_11 += a_1 * a_1;


			A += 4;

			}

		}
	else if(kadd>0)
		{

		// k = 0
		a_0 = A[0+bs*0];
		
		c_00 += a_0 * a_0;

		A += 4;
		k += 1;

		}

	if(ksub>0)
		{
		if(kadd>0)
			{
			A += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			}
		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;

		c_11 -= a_1 * a_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;

		c_11 -= a_1 * a_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;

		c_11 -= a_1 * a_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		c_00 -= a_0 * a_0;
		c_10 -= a_1 * a_0;

		c_11 -= a_1 * a_1;
		
		
		A += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_11 += C[1+bs*1];
		}
	
	// dpotrf
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00;
		c_00 = 1.0;
		}
	c_10 *= c_00;
	D[1+bs*0] = c_10;
	fact[1] = c_10;
	
	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		c_11 = 1.0;
		}

	}
#endif




