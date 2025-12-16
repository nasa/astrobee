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

#define N 4+0*40


int dpotrf_ref(int n, double *A, int lda)
	{

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



void dsyrk_ref(int n, double *A, int lda, double *C)
	{

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dcopy_ref(int n, double *A, int lda, double *C)
	{

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



int dpotrf_codegen_0(double *A)
	{

	const int n = N+0;
	const int lda = N+0;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_1(double *A)
	{

	const int n = N+1*4;
	const int lda = N+1*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_2(double *A)
	{

	const int n = N+2*4;
	const int lda = N+2*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_3(double *A)
	{

	const int n = N+3*4;
	const int lda = N+3*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_4(double *A)
	{

	const int n = N+4*4;
	const int lda = N+4*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_5(double *A)
	{

	const int n = N+5*4;
	const int lda = N+5*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_6(double *A)
	{

	const int n = N+6*4;
	const int lda = N+6*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_7(double *A)
	{

	const int n = N+7*4;
	const int lda = N+7*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_8(double *A)
	{

	const int n = N+8*4;
	const int lda = N+8*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}



int dpotrf_codegen_9(double *A)
	{

	const int n = N+9*4;
	const int lda = N+9*4;

	double a_jj, temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{

		a_jj = A[jj+lda*jj];

		for(kk=0; kk<jj; kk++)
			{
			a_jj -= A[jj+lda*kk] * A[jj+lda*kk];
			}

		if(a_jj<=0)
			{
			A[jj+lda*jj] = a_jj;
			return jj;
			}
		
		a_jj = sqrt(a_jj);
		A[jj+lda*jj] = a_jj;

		for(kk=0; kk<jj; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+lda*jj] -= temp * A[ii+lda*kk];
				}
			}

		a_jj = 1.0 / a_jj;

		for(ii=jj+1; ii<n; ii++)
			{
			A[ii+lda*jj] *= a_jj;
			}

		}
	
	return 0;
	
	}


void dsyrk_codegen_0(double *A, double *C)
	{

	const int n = N+0*4;
	const int lda = N+0*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_1(double *A, double *C)
	{

	const int n = N+1*4;
	const int lda = N+1*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_2(double *A, double *C)
	{

	const int n = N+2*4;
	const int lda = N+2*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_3(double *A, double *C)
	{

	const int n = N+3*4;
	const int lda = N+3*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_4(double *A, double *C)
	{

	const int n = N+4*4;
	const int lda = N+4*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_5(double *A, double *C)
	{

	const int n = N+5*4;
	const int lda = N+5*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_6(double *A, double *C)
	{

	const int n = N+6*4;
	const int lda = N+6*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_7(double *A, double *C)
	{

	const int n = N+7*4;
	const int lda = N+7*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_8(double *A, double *C)
	{

	const int n = N+8*4;
	const int lda = N+8*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dsyrk_codegen_9(double *A, double *C)
	{

	const int n = N+9*4;
	const int lda = N+9*4;

	double temp;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(kk=0; kk<n; kk++)
			{
			temp = A[jj+lda*kk];
			for(ii=jj; ii<n; ii++)
				{
				C[ii+lda*jj] += temp * A[ii+lda*kk];
				}
			}
		}
	
	}



void dcopy_codegen_0(double *A, double *C)
	{

	const int n = N+0*4;
	const int lda = N+0*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_1(double *A, double *C)
	{

	const int n = N+1*4;
	const int lda = N+1*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_2(double *A, double *C)
	{

	const int n = N+2*4;
	const int lda = N+2*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_3(double *A, double *C)
	{

	const int n = N+3*4;
	const int lda = N+3*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_4(double *A, double *C)
	{

	const int n = N+4*4;
	const int lda = N+4*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_5(double *A, double *C)
	{

	const int n = N+5*4;
	const int lda = N+5*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_6(double *A, double *C)
	{

	const int n = N+6*4;
	const int lda = N+6*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_7(double *A, double *C)
	{

	const int n = N+7*4;
	const int lda = N+7*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_8(double *A, double *C)
	{

	const int n = N+8*4;
	const int lda = N+8*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



void dcopy_codegen_9(double *A, double *C)
	{

	const int n = N+9*4;
	const int lda = N+9*4;

	int ii, jj;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+lda*jj] = A[ii+lda*jj];
			}
		}
	
	}



#if defined(TARGET_X64_AVX2)

// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_0(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+0*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

#if (N==4)
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[0], &pD[0], 1, &pC[0], &pD[0], &inv_diag_D[0]);
		return;
#else

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

#endif

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_1(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+1*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

#if (N==4)
		kernel_dpotrf_nt_8x8_lib4_new(0, &pD[0], sdd, &pD[0], sdd, 1, &pC[0], sdc, &pD[0], sdd, &inv_diag_D[0]);
		return;
#else

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

#endif

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_2(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+2*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

#if (N==4)
		kernel_dpotrf_nt_12x4_lib4_new(0, &pD[0], sdd, &pD[0], 1, &pC[0], sdc, &pD[0], sdd, &inv_diag_D[0]);
		kernel_dpotrf_nt_8x8_lib4_new(4, &pD[4*sdd], sdd, &pD[4*sdd], sdd, 1, &pC[4*bs+4*sdc], sdc, &pD[4*bs+4*sdd], sdd, &inv_diag_D[4]);
		return;
#else

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

#endif

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_3(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+3*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_4(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+4*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_5(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+5*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_6(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+6*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_7(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+7*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_8(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+8*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_9(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+9*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;

	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}

	if(m%12==8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x8_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		return;
		}

	if(m%12==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		return;
		}

	}



#endif



#if defined(TARGET_X64_AVX)

// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_0(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+0*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

#if (N==4)
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[0], &pD[0], 1, &pC[0], &pD[0], &inv_diag_D[0]);
		return;
#else

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

#endif

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_1(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+1*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

#if (N==4)
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[0], sdd, &pD[0], 1, &pC[0], sdc, &pD[0], sdd, &inv_diag_D[0]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[4*sdd], &pD[j*sdd], 1, &pC[4*bs+4*sdc], &pD[4*bs+4*sdd], &inv_diag_D[4]);
		return;
#else

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

#endif

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_2(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+2*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_3(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+3*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_4(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+4*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_5(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+5*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_6(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+6*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_7(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+7*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_8(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+8*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



// XXX assume m==n, and multiple of 4, therefore also m==n==sdc==sdd
void dpotrf_lib_codegen_9(double *pC, double *pD, double *inv_diag_D)
	{

	const int n = N+9*4;
	const int m = n;
	const int sdc = n;
	const int sdd = n;

	const int bs = 4;

	double *dummy;
	
	int i, j;

	i = 0;
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dpotrf_nt_4x4_lib4_new(j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
		}

	if(m%8==4)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		}

	}



#endif
