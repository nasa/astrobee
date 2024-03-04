/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include <math.h>
#include <stdio.h>

#include "../../include/blasfeo_common.h"
#include "../../include/blasfeo_d_aux.h"



// swap two rows
void kernel_drowsw_lib(int kmax, double *pA, int lda, double *pC, int ldc)
	{

	int ii;
	double tmp;

	for(ii=0; ii<kmax-3; ii+=4)
		{
		tmp = pA[0+lda*0];
		pA[0+lda*0] = pC[0+ldc*0];
		pC[0+ldc*0] = tmp;
		tmp = pA[0+lda*1];
		pA[0+lda*1] = pC[0+ldc*1];
		pC[0+ldc*1] = tmp;
		tmp = pA[0+lda*2];
		pA[0+lda*2] = pC[0+ldc*2];
		pC[0+ldc*2] = tmp;
		tmp = pA[0+lda*3];
		pA[0+lda*3] = pC[0+ldc*3];
		pC[0+ldc*3] = tmp;
		pA += 4*lda;
		pC += 4*ldc;
		}
	for( ; ii<kmax; ii++)
		{
		tmp = pA[0+lda*0];
		pA[0+lda*0] = pC[0+ldc*0];
		pC[0+ldc*0] = tmp;
		pA += 1*lda;
		pC += 1*ldc;
		}

	}



// C numbering, starting from 0
void didamax_lib(int n, double *pA, int *p_idamax, double *p_amax)
	{

	int ii;
	double tmp, amax;
		
	int idamax = -1;

	p_idamax[0] = idamax;
	if(n<1)
		return;

	amax = -1.0;
	ii = 0;
	for( ; ii<n-3; ii+=4)
		{
		tmp = fabs(pA[0]);
		if(tmp>amax)
			{
			idamax = ii+0;
			amax = tmp;
			}
		tmp = fabs(pA[1]);
		if(tmp>amax)
			{
			idamax = ii+1;
			amax = tmp;
			}
		tmp = fabs(pA[2]);
		if(tmp>amax)
			{
			idamax = ii+2;
			amax = tmp;
			}
		tmp = fabs(pA[3]);
		if(tmp>amax)
			{
			idamax = ii+3;
			amax = tmp;
			}
		pA += 4;
		}
	for( ; ii<n; ii++)
		{
		tmp = fabs(pA[0]);
		if(tmp>amax)
			{
			idamax = ii+0;
			amax = tmp;
			}
		pA += 1;
		}
	
	p_amax[0] = amax;
	p_idamax[0] = idamax;

	return;

	}



// C numering (starting from zero) in the ipiv
// it process m>=4 rows and 4 cols
void kernel_dgetrf_pivot_4_lib(int m, double *pA, int lda, double *inv_diag_A, int* ipiv)
	{

	// assume m>=4
	int ma = m-4;

	double
		tmp0, tmp1, tmp2, tmp3,
		u_00, u_01, u_02, u_03,
		      u_11, u_12, u_13,
		            u_22, u_23,
		                  u_33;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	int
		ia0, ia1, ia2;

	// first column
	didamax_lib(m-0, &pA[0+lda*0], &idamax, &tmp0);
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			{
			kernel_drowsw_lib(4, pA+0, lda, pA+ipiv[0], lda);
			}

		tmp0 = 1.0 / pA[0+lda*0];
		inv_diag_A[0] = tmp0;
		pA[1+lda*0] *= tmp0;
		pA[2+lda*0] *= tmp0;
		pA[3+lda*0] *= tmp0;
		pB = pA + 4;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+lda*0] *= tmp0;
			pB[1+lda*0] *= tmp0;
			pB[2+lda*0] *= tmp0;
			pB[3+lda*0] *= tmp0;
			pB += 4;
			}
		for( ; k<ma; k++)
			{
			pB[0+lda*0] *= tmp0;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[0] = 0.0;
		}

	// second column
	u_01  = pA[0+lda*1];
	tmp1  = pA[1+lda*1];
	tmp2  = pA[2+lda*1];
	tmp3  = pA[3+lda*1];
	tmp1 -= pA[1+lda*0] * u_01;
	tmp2 -= pA[2+lda*0] * u_01;
	tmp3 -= pA[3+lda*0] * u_01;
	pA[1+lda*1] = tmp1;
	pA[2+lda*1] = tmp2;
	pA[3+lda*1] = tmp3;
	pB = pA + 4;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+lda*1];
		tmp1  = pB[1+lda*1];
		tmp2  = pB[2+lda*1];
		tmp3  = pB[3+lda*1];
		tmp0 -= pB[0+lda*0] * u_01;
		tmp1 -= pB[1+lda*0] * u_01;
		tmp2 -= pB[2+lda*0] * u_01;
		tmp3 -= pB[3+lda*0] * u_01;
		pB[0+lda*1] = tmp0;
		pB[1+lda*1] = tmp1;
		pB[2+lda*1] = tmp2;
		pB[3+lda*1] = tmp3;
		pB += 4;
		}
	for( ; k<ma; k++)
		{
		tmp0 = pB[0+lda*1];
		tmp0 -= pB[0+lda*0] * u_01;
		pB[0+lda*1] = tmp0;
		pB += 1;
		}

	didamax_lib(m-1, &pA[1+lda*1], &idamax, &tmp1);
	ipiv[1] = idamax+1;
	if(tmp1!=0)
		{
		if(ipiv[1]!=1)
			{
			kernel_drowsw_lib(4, pA+1, lda, pA+ipiv[1], lda);
			}

		tmp1 = 1.0 / pA[1+lda*1];
		inv_diag_A[1] = tmp1;
		pA[2+lda*1] *= tmp1;
		pA[3+lda*1] *= tmp1;
		pB = pA + 4;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+lda*1] *= tmp1;
			pB[1+lda*1] *= tmp1;
			pB[2+lda*1] *= tmp1;
			pB[3+lda*1] *= tmp1;
			pB += 4;
			}
		for( ; k<ma; k++)
			{
			pB[0+lda*1] *= tmp1;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[1] = 0.0;
		}

	// third column
	u_02  = pA[0+lda*2];
	u_12  = pA[1+lda*2];
	u_12 -= pA[1+lda*0] * u_02;
	pA[1+lda*2] = u_12;
	tmp2  = pA[2+lda*2];
	tmp3  = pA[3+lda*2];
	tmp2 -= pA[2+lda*0] * u_02;
	tmp3 -= pA[3+lda*0] * u_02;
	tmp2 -= pA[2+lda*1] * u_12;
	tmp3 -= pA[3+lda*1] * u_12;
	pA[2+lda*2] = tmp2;
	pA[3+lda*2] = tmp3;
	pB = pA + 4;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+lda*2];
		tmp1  = pB[1+lda*2];
		tmp2  = pB[2+lda*2];
		tmp3  = pB[3+lda*2];
		tmp0 -= pB[0+lda*0] * u_02;
		tmp1 -= pB[1+lda*0] * u_02;
		tmp2 -= pB[2+lda*0] * u_02;
		tmp3 -= pB[3+lda*0] * u_02;
		tmp0 -= pB[0+lda*1] * u_12;
		tmp1 -= pB[1+lda*1] * u_12;
		tmp2 -= pB[2+lda*1] * u_12;
		tmp3 -= pB[3+lda*1] * u_12;
		pB[0+lda*2] = tmp0;
		pB[1+lda*2] = tmp1;
		pB[2+lda*2] = tmp2;
		pB[3+lda*2] = tmp3;
		pB += 4;
		}
	for( ; k<ma; k++)
		{
		tmp0  = pB[0+lda*2];
		tmp0 -= pB[0+lda*0] * u_02;
		tmp0 -= pB[0+lda*1] * u_12;
		pB[0+lda*2] = tmp0;
		pB += 1;
		}

	didamax_lib(m-2, &pA[2+lda*2], &idamax, &tmp2);
	ipiv[2] = idamax+2;
	if(tmp2!=0)
		{
		if(ipiv[2]!=2)
			{
			kernel_drowsw_lib(4, pA+2, lda, pA+ipiv[2], lda);
			}

		tmp2 = 1.0 / pA[2+lda*2];
		inv_diag_A[2] = tmp2;
		pA[3+lda*2] *= tmp2;
		pB = pA + 4;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+lda*2] *= tmp2;
			pB[1+lda*2] *= tmp2;
			pB[2+lda*2] *= tmp2;
			pB[3+lda*2] *= tmp2;
			pB += 4;
			}
		for( ; k<ma; k++)
			{
			pB[0+lda*2] *= tmp2;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[2] = 0.0;
		}

	// fourth column
	u_03  = pA[0+lda*3];
	u_13  = pA[1+lda*3];
	u_13 -= pA[1+lda*0] * u_03;
	pA[1+lda*3] = u_13;
	u_23  = pA[2+lda*3];
	u_23 -= pA[2+lda*0] * u_03;
	u_23 -= pA[2+lda*1] * u_13;
	pA[2+lda*3] = u_23;
	tmp3  = pA[3+lda*3];
	tmp3 -= pA[3+lda*0] * u_03;
	tmp3 -= pA[3+lda*1] * u_13;
	tmp3 -= pA[3+lda*2] * u_23;
	pA[3+lda*3] = tmp3;
	pB = pA + 4;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+lda*3];
		tmp1  = pB[1+lda*3];
		tmp2  = pB[2+lda*3];
		tmp3  = pB[3+lda*3];
		tmp0 -= pB[0+lda*0] * u_03;
		tmp1 -= pB[1+lda*0] * u_03;
		tmp2 -= pB[2+lda*0] * u_03;
		tmp3 -= pB[3+lda*0] * u_03;
		tmp0 -= pB[0+lda*1] * u_13;
		tmp1 -= pB[1+lda*1] * u_13;
		tmp2 -= pB[2+lda*1] * u_13;
		tmp3 -= pB[3+lda*1] * u_13;
		tmp0 -= pB[0+lda*2] * u_23;
		tmp1 -= pB[1+lda*2] * u_23;
		tmp2 -= pB[2+lda*2] * u_23;
		tmp3 -= pB[3+lda*2] * u_23;
		pB[0+lda*3] = tmp0;
		pB[1+lda*3] = tmp1;
		pB[2+lda*3] = tmp2;
		pB[3+lda*3] = tmp3;
		pB += 4;
		}
	for( ; k<ma; k++)
		{
		tmp0  = pB[0+lda*3];
		tmp0 -= pB[0+lda*0] * u_03;
		tmp0 -= pB[0+lda*1] * u_13;
		tmp0 -= pB[0+lda*2] * u_23;
		pB[0+lda*3] = tmp0;
		pB += 1;
		}

	didamax_lib(m-3, &pA[3+lda*3], &idamax, &tmp3);
	ipiv[3] = idamax+3;
	if(tmp3!=0)
		{
		if(ipiv[3]!=3)
			{
			kernel_drowsw_lib(4, pA+3, lda, pA+ipiv[3], lda);
			}

		tmp3 = 1.0 / pA[3+lda*3];
		inv_diag_A[3] = tmp3;
		pB = pA + 4;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+lda*3] *= tmp3;
			pB[1+lda*3] *= tmp3;
			pB[2+lda*3] *= tmp3;
			pB[3+lda*3] *= tmp3;
			pB += 4;
			}
		for( ; k<ma; k++)
			{
			pB[0+lda*3] *= tmp3;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[3] = 0.0;
		}
	
	return;

	}




// it process m>0 rows and 0<n<=4 cols
void kernel_dgetrf_pivot_4_vs_lib(int m, double *pA, int lda, double *inv_diag_A, int* ipiv, int n)
	{

	if(m<=0 || n<=0)
		return;

	// assume m>=4
	int ma = m-4;

	double
		tmp0, tmp1, tmp2, tmp3,
		u_00, u_01, u_02, u_03,
		      u_11, u_12, u_13,
		            u_22, u_23,
		                  u_33;
	
	double
		*pB;
	
	int 
		k, idamax;
	
	int n4 = n<4 ? n : 4;
	
	// first column

	// find pivot & scale
	didamax_lib(m-0, &pA[0+lda*0], &idamax, &tmp0);
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			kernel_drowsw_lib(n4, pA+0, lda, pA+ipiv[0], lda);

		tmp0 = 1.0 / pA[0+lda*0];
		inv_diag_A[0] = tmp0;
		if(m>=4)
			{
			pA[1+lda*0] *= tmp0;
			pA[2+lda*0] *= tmp0;
			pA[3+lda*0] *= tmp0;
			pB = pA + 4;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+lda*0] *= tmp0;
				pB[1+lda*0] *= tmp0;
				pB[2+lda*0] *= tmp0;
				pB[3+lda*0] *= tmp0;
				pB += 4;
				}
			for( ; k<ma; k++)
				{
				pB[0+lda*0] *= tmp0;
				pB += 1;
				}
			}
		else // m = {1,2,3}
			{
			if(m>1)
				{
				pA[1+lda*0] *= tmp0;
				if(m>2)
					pA[2+lda*0] *= tmp0;
				}
			}
		}
	else
		{
		inv_diag_A[0] = 0.0;
		}

	if(n==1 || m==1) // XXX for the first row there is nothing to do, so we can return here
		{
		return;
		}

	// second column

	// correct
	if(m>=4)
		{
		u_01  = pA[0+lda*1];
		tmp1  = pA[1+lda*1];
		tmp2  = pA[2+lda*1];
		tmp3  = pA[3+lda*1];
		tmp1 -= pA[1+lda*0] * u_01;
		tmp2 -= pA[2+lda*0] * u_01;
		tmp3 -= pA[3+lda*0] * u_01;
		pA[1+lda*1] = tmp1;
		pA[2+lda*1] = tmp2;
		pA[3+lda*1] = tmp3;
		pB = pA + 4;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+lda*1];
			tmp1  = pB[1+lda*1];
			tmp2  = pB[2+lda*1];
			tmp3  = pB[3+lda*1];
			tmp0 -= pB[0+lda*0] * u_01;
			tmp1 -= pB[1+lda*0] * u_01;
			tmp2 -= pB[2+lda*0] * u_01;
			tmp3 -= pB[3+lda*0] * u_01;
			pB[0+lda*1] = tmp0;
			pB[1+lda*1] = tmp1;
			pB[2+lda*1] = tmp2;
			pB[3+lda*1] = tmp3;
			pB += 4;
			}
		for( ; k<ma; k++)
			{
			tmp0 = pB[0+lda*1];
			tmp0 -= pB[0+lda*0] * u_01;
			pB[0+lda*1] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_01  = pA[0+lda*1];
		tmp1  = pA[1+lda*1];
		tmp1 -= pA[1+lda*0] * u_01;
		pA[1+lda*1] = tmp1;
		if(m>2)
			{
			tmp2  = pA[2+lda*1];
			tmp2 -= pA[2+lda*0] * u_01;
			pA[2+lda*1] = tmp2;
			}
		}

	// find pivot & scale
	didamax_lib(m-1, &pA[1+lda*1], &idamax, &tmp1);
	ipiv[1] = idamax+1;
	if(tmp1!=0)
		{
		if(ipiv[1]!=1)
			{
			kernel_drowsw_lib(n4, pA+1, lda, pA+ipiv[1], lda);
			}

		tmp1 = 1.0 / pA[1+lda*1];
		inv_diag_A[1] = tmp1;
		if(m>=4)
			{
			pA[2+lda*1] *= tmp1;
			pA[3+lda*1] *= tmp1;
			pB = pA + 4;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+lda*1] *= tmp1;
				pB[1+lda*1] *= tmp1;
				pB[2+lda*1] *= tmp1;
				pB[3+lda*1] *= tmp1;
				pB += 4;
				}
			for( ; k<ma; k++)
				{
				pB[0+lda*1] *= tmp1;
				pB += 1;
				}
			}
		else // m = {2,3}
			{
			if(m>2)
				pA[2+lda*1] *= tmp1;
			}
		}
	else
		{
		inv_diag_A[1] = 0.0;
		}

	if(n==2)
		{
		return;
		}

	// third column

	// correct
	if(m>=4)
		{
		u_02  = pA[0+lda*2];
		u_12  = pA[1+lda*2];
		u_12 -= pA[1+lda*0] * u_02;
		pA[1+lda*2] = u_12;
		tmp2  = pA[2+lda*2];
		tmp3  = pA[3+lda*2];
		tmp2 -= pA[2+lda*0] * u_02;
		tmp3 -= pA[3+lda*0] * u_02;
		tmp2 -= pA[2+lda*1] * u_12;
		tmp3 -= pA[3+lda*1] * u_12;
		pA[2+lda*2] = tmp2;
		pA[3+lda*2] = tmp3;
		pB = pA + 4;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+lda*2];
			tmp1  = pB[1+lda*2];
			tmp2  = pB[2+lda*2];
			tmp3  = pB[3+lda*2];
			tmp0 -= pB[0+lda*0] * u_02;
			tmp1 -= pB[1+lda*0] * u_02;
			tmp2 -= pB[2+lda*0] * u_02;
			tmp3 -= pB[3+lda*0] * u_02;
			tmp0 -= pB[0+lda*1] * u_12;
			tmp1 -= pB[1+lda*1] * u_12;
			tmp2 -= pB[2+lda*1] * u_12;
			tmp3 -= pB[3+lda*1] * u_12;
			pB[0+lda*2] = tmp0;
			pB[1+lda*2] = tmp1;
			pB[2+lda*2] = tmp2;
			pB[3+lda*2] = tmp3;
			pB += 4;
			}
		for( ; k<ma; k++)
			{
			tmp0  = pB[0+lda*2];
			tmp0 -= pB[0+lda*0] * u_02;
			tmp0 -= pB[0+lda*1] * u_12;
			pB[0+lda*2] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_02  = pA[0+lda*2];
		u_12  = pA[1+lda*2];
		u_12 -= pA[1+lda*0] * u_02;
		pA[1+lda*2] = u_12;
		if(m>2)
			{
			tmp2  = pA[2+lda*2];
			tmp2 -= pA[2+lda*0] * u_02;
			tmp2 -= pA[2+lda*1] * u_12;
			pA[2+lda*2] = tmp2;
			}
		}

	// find pivot & scale
	if(m>2)
		{
		didamax_lib(m-2, &pA[2+lda*2], &idamax, &tmp2);
		ipiv[2] = idamax+2;
		if(tmp2!=0)
			{
			if(ipiv[2]!=2)
				kernel_drowsw_lib(n4, pA+2, lda, pA+ipiv[2], lda);

			tmp2 = 1.0 / pA[2+lda*2];
			inv_diag_A[2] = tmp2;
			if(m>=4)
				{
				pA[3+lda*2] *= tmp2;
				pB = pA + 4;
				for(k=0; k<ma-3; k+=4)
					{
					pB[0+lda*2] *= tmp2;
					pB[1+lda*2] *= tmp2;
					pB[2+lda*2] *= tmp2;
					pB[3+lda*2] *= tmp2;
					pB += 4;
					}
				for( ; k<ma; k++)
					{
					pB[0+lda*2] *= tmp2;
					pB += 1;
					}
				}
			}
		else
			{
			inv_diag_A[2] = 0.0;
			}
		}

	if(n<4)
		{
		return;
		}

	// fourth column

	// correct
	if(m>=4)
		{
		u_03  = pA[0+lda*3];
		u_13  = pA[1+lda*3];
		u_13 -= pA[1+lda*0] * u_03;
		pA[1+lda*3] = u_13;
		u_23  = pA[2+lda*3];
		u_23 -= pA[2+lda*0] * u_03;
		u_23 -= pA[2+lda*1] * u_13;
		pA[2+lda*3] = u_23;
		tmp3  = pA[3+lda*3];
		tmp3 -= pA[3+lda*0] * u_03;
		tmp3 -= pA[3+lda*1] * u_13;
		tmp3 -= pA[3+lda*2] * u_23;
		pA[3+lda*3] = tmp3;
		pB = pA + 4;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+lda*3];
			tmp1  = pB[1+lda*3];
			tmp2  = pB[2+lda*3];
			tmp3  = pB[3+lda*3];
			tmp0 -= pB[0+lda*0] * u_03;
			tmp1 -= pB[1+lda*0] * u_03;
			tmp2 -= pB[2+lda*0] * u_03;
			tmp3 -= pB[3+lda*0] * u_03;
			tmp0 -= pB[0+lda*1] * u_13;
			tmp1 -= pB[1+lda*1] * u_13;
			tmp2 -= pB[2+lda*1] * u_13;
			tmp3 -= pB[3+lda*1] * u_13;
			tmp0 -= pB[0+lda*2] * u_23;
			tmp1 -= pB[1+lda*2] * u_23;
			tmp2 -= pB[2+lda*2] * u_23;
			tmp3 -= pB[3+lda*2] * u_23;
			pB[0+lda*3] = tmp0;
			pB[1+lda*3] = tmp1;
			pB[2+lda*3] = tmp2;
			pB[3+lda*3] = tmp3;
			pB += 4;
			}
		for( ; k<ma; k++)
			{
			tmp0  = pB[0+lda*3];
			tmp0 -= pB[0+lda*0] * u_03;
			tmp0 -= pB[0+lda*1] * u_13;
			tmp0 -= pB[0+lda*2] * u_23;
			pB[0+lda*3] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_03  = pA[0+lda*3];
		u_13  = pA[1+lda*3];
		u_13 -= pA[1+lda*0] * u_03;
		pA[1+lda*3] = u_13;
		if(m>2)
			{
			u_23  = pA[2+lda*3];
			u_23 -= pA[2+lda*0] * u_03;
			u_23 -= pA[2+lda*1] * u_13;
			pA[2+lda*3] = u_23;
			}
		}

	if(m>3)
		{
		// find pivot & scale
		didamax_lib(m-3, &pA[3+lda*3], &idamax, &tmp3);
		ipiv[3] = idamax+3;
		if(tmp3!=0)
			{
			if(ipiv[3]!=3)
				kernel_drowsw_lib(n4, pA+3, lda, pA+ipiv[3], lda);

			tmp3 = 1.0 / pA[3+lda*3];
			inv_diag_A[3] = tmp3;
			pB = pA + 4;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+lda*3] *= tmp3;
				pB[1+lda*3] *= tmp3;
				pB[2+lda*3] *= tmp3;
				pB[3+lda*3] *= tmp3;
				pB += 4;
				}
			for( ; k<ma; k++)
				{
				pB[0+lda*3] *= tmp3;
				pB += 1;
				}
			}
		else
			{
			inv_diag_A[3] = 0.0;
			}
		}
	
	return;

	}




