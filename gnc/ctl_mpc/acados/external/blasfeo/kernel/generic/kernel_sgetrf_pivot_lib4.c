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
#include "../../include/blasfeo_s_aux.h"



// C numbering, starting from 0
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void sidamax_lib4(int n, int offset, float *pA, int sda, int *p_idamax, float *p_amax)
	{

	int ii;
	float tmp, amax;
		
	int idamax = -1;

	p_idamax[0] = idamax;
	if(n<1)
		return;

	const int bs = 4;

	int na = (bs - offset%bs)%bs;
	na = n<na ? n : na;

	amax = -1.0;
	ii = 0;
	if(na>0)
		{
		for( ; ii<na; ii++)
			{
			tmp = fabs(pA[0]);
			if(tmp>amax)
				{
				idamax = ii+0;
				amax = tmp;
				}
			pA += 1;
			}
		pA += bs*(sda-1);
		}
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
		pA += bs*sda;
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
#endif



// C numering (starting from zero) in the ipiv
// it process m>=4 rows and 4 cols
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgetrf_pivot_4_lib4(int m, float *pA, int sda, float *inv_diag_A, int* ipiv)
	{

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	float
		tmp0, tmp1, tmp2, tmp3,
		u_00, u_01, u_02, u_03,
		      u_11, u_12, u_13,
		            u_22, u_23,
		                  u_33;
	
	float
		*pB;
	
	int 
		k, idamax;
	
	// first column
	sidamax_lib4(m-0, 0, &pA[0+bs*0], sda, &idamax, &tmp0);
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			srowsw_lib(4, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		tmp0 = 1.0 / pA[0+bs*0];
		inv_diag_A[0] = tmp0;
		pA[1+bs*0] *= tmp0;
		pA[2+bs*0] *= tmp0;
		pA[3+bs*0] *= tmp0;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*0] *= tmp0;
			pB[1+bs*0] *= tmp0;
			pB[2+bs*0] *= tmp0;
			pB[3+bs*0] *= tmp0;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*0] *= tmp0;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[0] = 0.0;
		}

	// second column
	u_01  = pA[0+bs*1];
	tmp1  = pA[1+bs*1];
	tmp2  = pA[2+bs*1];
	tmp3  = pA[3+bs*1];
	tmp1 -= pA[1+bs*0] * u_01;
	tmp2 -= pA[2+bs*0] * u_01;
	tmp3 -= pA[3+bs*0] * u_01;
	pA[1+bs*1] = tmp1;
	pA[2+bs*1] = tmp2;
	pA[3+bs*1] = tmp3;
	pB = pA + bs*sda;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+bs*1];
		tmp1  = pB[1+bs*1];
		tmp2  = pB[2+bs*1];
		tmp3  = pB[3+bs*1];
		tmp0 -= pB[0+bs*0] * u_01;
		tmp1 -= pB[1+bs*0] * u_01;
		tmp2 -= pB[2+bs*0] * u_01;
		tmp3 -= pB[3+bs*0] * u_01;
		pB[0+bs*1] = tmp0;
		pB[1+bs*1] = tmp1;
		pB[2+bs*1] = tmp2;
		pB[3+bs*1] = tmp3;
		pB += bs*sda;
		}
	for( ; k<ma; k++)
		{
		tmp0 = pB[0+bs*1];
		tmp0 -= pB[0+bs*0] * u_01;
		pB[0+bs*1] = tmp0;
		pB += 1;
		}

	sidamax_lib4(m-1, 1, &pA[1+bs*1], sda, &idamax, &tmp1);
	ipiv[1] = idamax+1;
	if(tmp1!=0)
		{
		if(ipiv[1]!=1)
			srowsw_lib(4, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

		tmp1 = 1.0 / pA[1+bs*1];
		inv_diag_A[1] = tmp1;
		pA[2+bs*1] *= tmp1;
		pA[3+bs*1] *= tmp1;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*1] *= tmp1;
			pB[1+bs*1] *= tmp1;
			pB[2+bs*1] *= tmp1;
			pB[3+bs*1] *= tmp1;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*1] *= tmp1;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[1] = 0.0;
		}

	// third column
	u_02  = pA[0+bs*2];
	u_12  = pA[1+bs*2];
	u_12 -= pA[1+bs*0] * u_02;
	pA[1+bs*2] = u_12;
	tmp2  = pA[2+bs*2];
	tmp3  = pA[3+bs*2];
	tmp2 -= pA[2+bs*0] * u_02;
	tmp3 -= pA[3+bs*0] * u_02;
	tmp2 -= pA[2+bs*1] * u_12;
	tmp3 -= pA[3+bs*1] * u_12;
	pA[2+bs*2] = tmp2;
	pA[3+bs*2] = tmp3;
	pB = pA + bs*sda;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+bs*2];
		tmp1  = pB[1+bs*2];
		tmp2  = pB[2+bs*2];
		tmp3  = pB[3+bs*2];
		tmp0 -= pB[0+bs*0] * u_02;
		tmp1 -= pB[1+bs*0] * u_02;
		tmp2 -= pB[2+bs*0] * u_02;
		tmp3 -= pB[3+bs*0] * u_02;
		tmp0 -= pB[0+bs*1] * u_12;
		tmp1 -= pB[1+bs*1] * u_12;
		tmp2 -= pB[2+bs*1] * u_12;
		tmp3 -= pB[3+bs*1] * u_12;
		pB[0+bs*2] = tmp0;
		pB[1+bs*2] = tmp1;
		pB[2+bs*2] = tmp2;
		pB[3+bs*2] = tmp3;
		pB += bs*sda;
		}
	for( ; k<ma; k++)
		{
		tmp0  = pB[0+bs*2];
		tmp0 -= pB[0+bs*0] * u_02;
		tmp0 -= pB[0+bs*1] * u_12;
		pB[0+bs*2] = tmp0;
		pB += 1;
		}

	sidamax_lib4(m-2, 2, &pA[2+bs*2], sda, &idamax, &tmp2);
	ipiv[2] = idamax+2;
	if(tmp2!=0)
		{
		if(ipiv[2]!=2)
			srowsw_lib(4, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

		tmp2 = 1.0 / pA[2+bs*2];
		inv_diag_A[2] = tmp2;
		pA[3+bs*2] *= tmp2;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*2] *= tmp2;
			pB[1+bs*2] *= tmp2;
			pB[2+bs*2] *= tmp2;
			pB[3+bs*2] *= tmp2;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*2] *= tmp2;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[2] = 0.0;
		}

	// fourth column
	u_03  = pA[0+bs*3];
	u_13  = pA[1+bs*3];
	u_13 -= pA[1+bs*0] * u_03;
	pA[1+bs*3] = u_13;
	u_23  = pA[2+bs*3];
	u_23 -= pA[2+bs*0] * u_03;
	u_23 -= pA[2+bs*1] * u_13;
	pA[2+bs*3] = u_23;
	tmp3  = pA[3+bs*3];
	tmp3 -= pA[3+bs*0] * u_03;
	tmp3 -= pA[3+bs*1] * u_13;
	tmp3 -= pA[3+bs*2] * u_23;
	pA[3+bs*3] = tmp3;
	pB = pA + bs*sda;
	for(k=0; k<ma-3; k+=4)
		{
		tmp0  = pB[0+bs*3];
		tmp1  = pB[1+bs*3];
		tmp2  = pB[2+bs*3];
		tmp3  = pB[3+bs*3];
		tmp0 -= pB[0+bs*0] * u_03;
		tmp1 -= pB[1+bs*0] * u_03;
		tmp2 -= pB[2+bs*0] * u_03;
		tmp3 -= pB[3+bs*0] * u_03;
		tmp0 -= pB[0+bs*1] * u_13;
		tmp1 -= pB[1+bs*1] * u_13;
		tmp2 -= pB[2+bs*1] * u_13;
		tmp3 -= pB[3+bs*1] * u_13;
		tmp0 -= pB[0+bs*2] * u_23;
		tmp1 -= pB[1+bs*2] * u_23;
		tmp2 -= pB[2+bs*2] * u_23;
		tmp3 -= pB[3+bs*2] * u_23;
		pB[0+bs*3] = tmp0;
		pB[1+bs*3] = tmp1;
		pB[2+bs*3] = tmp2;
		pB[3+bs*3] = tmp3;
		pB += bs*sda;
		}
	for( ; k<ma; k++)
		{
		tmp0  = pB[0+bs*3];
		tmp0 -= pB[0+bs*0] * u_03;
		tmp0 -= pB[0+bs*1] * u_13;
		tmp0 -= pB[0+bs*2] * u_23;
		pB[0+bs*3] = tmp0;
		pB += 1;
		}

	sidamax_lib4(m-3, 3, &pA[3+bs*3], sda, &idamax, &tmp3);
	ipiv[3] = idamax+3;
	if(tmp3!=0)
		{
		if(ipiv[3]!=3)
			srowsw_lib(4, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

		tmp3 = 1.0 / pA[3+bs*3];
		inv_diag_A[3] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			pB[0+bs*3] *= tmp3;
			pB[1+bs*3] *= tmp3;
			pB[2+bs*3] *= tmp3;
			pB[3+bs*3] *= tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			pB[0+bs*3] *= tmp3;
			pB += 1;
			}
		}
	else
		{
		inv_diag_A[3] = 0.0;
		}
	
	return;

	}
#endif



// it process m>0 rows and 0<n<=4 cols
#if defined(TARGET_GENERIC) || defined(TARGET_X86_AMD_BARCELONA) || defined(TARGET_X86_AMD_JAGUAR) || defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE) || defined(TARGET_X64_INTEL_CORE) || defined(TARGET_X64_AMD_BULLDOZER) || defined(TARGET_ARMV7A_ARM_CORTEX_A15) || defined(TARGET_ARMV7A_ARM_CORTEX_A7) || defined(TARGET_ARMV7A_ARM_CORTEX_A9) || defined(TARGET_ARMV8A_ARM_CORTEX_A57) || defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_sgetrf_pivot_4_vs_lib4(int m, int n, float *pA, int sda, float *inv_diag_A, int* ipiv)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	// assume m>=4
	int ma = m-4;

	float
		tmp0, tmp1, tmp2, tmp3,
		u_00, u_01, u_02, u_03,
		      u_11, u_12, u_13,
		            u_22, u_23,
		                  u_33;
	
	float
		*pB;
	
	int 
		k, idamax;
	
	// first column

	// find pivot & scale
	sidamax_lib4(m-0, 0, &pA[0+bs*0], sda, &idamax, &tmp0);
	ipiv[0] = idamax;
	if(tmp0!=0.0)
		{
		if(ipiv[0]!=0)
			srowsw_lib(4, pA+0, pA+ipiv[0]/bs*bs*sda+ipiv[0]%bs);

		tmp0 = 1.0 / pA[0+bs*0];
		inv_diag_A[0] = tmp0;
		if(m>=4)
			{
			pA[1+bs*0] *= tmp0;
			pA[2+bs*0] *= tmp0;
			pA[3+bs*0] *= tmp0;
			pB = pA + bs*sda;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+bs*0] *= tmp0;
				pB[1+bs*0] *= tmp0;
				pB[2+bs*0] *= tmp0;
				pB[3+bs*0] *= tmp0;
				pB += bs*sda;
				}
			for( ; k<ma; k++)
				{
				pB[0+bs*0] *= tmp0;
				pB += 1;
				}
			}
		else // m = {1,2,3}
			{
			if(m>1)
				{
				pA[1+bs*0] *= tmp0;
				if(m>2)
					pA[2+bs*0] *= tmp0;
				}
			}
		}
	else
		{
		inv_diag_A[0] = 0.0;
		}
	
	if(n==1 || m==1) // XXX for the first row there is nothing to do, so we can return here
		return;

	// second column

	// correct
	if(m>=4)
		{
		u_01  = pA[0+bs*1];
		tmp1  = pA[1+bs*1];
		tmp2  = pA[2+bs*1];
		tmp3  = pA[3+bs*1];
		tmp1 -= pA[1+bs*0] * u_01;
		tmp2 -= pA[2+bs*0] * u_01;
		tmp3 -= pA[3+bs*0] * u_01;
		pA[1+bs*1] = tmp1;
		pA[2+bs*1] = tmp2;
		pA[3+bs*1] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+bs*1];
			tmp1  = pB[1+bs*1];
			tmp2  = pB[2+bs*1];
			tmp3  = pB[3+bs*1];
			tmp0 -= pB[0+bs*0] * u_01;
			tmp1 -= pB[1+bs*0] * u_01;
			tmp2 -= pB[2+bs*0] * u_01;
			tmp3 -= pB[3+bs*0] * u_01;
			pB[0+bs*1] = tmp0;
			pB[1+bs*1] = tmp1;
			pB[2+bs*1] = tmp2;
			pB[3+bs*1] = tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			tmp0 = pB[0+bs*1];
			tmp0 -= pB[0+bs*0] * u_01;
			pB[0+bs*1] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_01  = pA[0+bs*1];
		tmp1  = pA[1+bs*1];
		tmp1 -= pA[1+bs*0] * u_01;
		pA[1+bs*1] = tmp1;
		if(m>2)
			{
			tmp2  = pA[2+bs*1];
			tmp2 -= pA[2+bs*0] * u_01;
			pA[2+bs*1] = tmp2;
			}
		}

	// find pivot & scale
	sidamax_lib4(m-1, 1, &pA[1+bs*1], sda, &idamax, &tmp1);
	ipiv[1] = idamax+1;
	if(tmp1!=0)
		{
		if(ipiv[1]!=1)
			srowsw_lib(4, pA+1, pA+ipiv[1]/bs*bs*sda+ipiv[1]%bs);

		tmp1 = 1.0 / pA[1+bs*1];
		inv_diag_A[1] = tmp1;
		if(m>=4)
			{
			pA[2+bs*1] *= tmp1;
			pA[3+bs*1] *= tmp1;
			pB = pA + bs*sda;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+bs*1] *= tmp1;
				pB[1+bs*1] *= tmp1;
				pB[2+bs*1] *= tmp1;
				pB[3+bs*1] *= tmp1;
				pB += bs*sda;
				}
			for( ; k<ma; k++)
				{
				pB[0+bs*1] *= tmp1;
				pB += 1;
				}
			}
		else // m = {2,3}
			{
			if(m>2)
				pA[2+bs*1] *= tmp1;
			}
		}
	else
		{
		inv_diag_A[1] = 0.0;
		}

	if(n==2)
		return;

	// third column

	// correct
	if(m>=4)
		{
		u_02  = pA[0+bs*2];
		u_12  = pA[1+bs*2];
		u_12 -= pA[1+bs*0] * u_02;
		pA[1+bs*2] = u_12;
		tmp2  = pA[2+bs*2];
		tmp3  = pA[3+bs*2];
		tmp2 -= pA[2+bs*0] * u_02;
		tmp3 -= pA[3+bs*0] * u_02;
		tmp2 -= pA[2+bs*1] * u_12;
		tmp3 -= pA[3+bs*1] * u_12;
		pA[2+bs*2] = tmp2;
		pA[3+bs*2] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+bs*2];
			tmp1  = pB[1+bs*2];
			tmp2  = pB[2+bs*2];
			tmp3  = pB[3+bs*2];
			tmp0 -= pB[0+bs*0] * u_02;
			tmp1 -= pB[1+bs*0] * u_02;
			tmp2 -= pB[2+bs*0] * u_02;
			tmp3 -= pB[3+bs*0] * u_02;
			tmp0 -= pB[0+bs*1] * u_12;
			tmp1 -= pB[1+bs*1] * u_12;
			tmp2 -= pB[2+bs*1] * u_12;
			tmp3 -= pB[3+bs*1] * u_12;
			pB[0+bs*2] = tmp0;
			pB[1+bs*2] = tmp1;
			pB[2+bs*2] = tmp2;
			pB[3+bs*2] = tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			tmp0  = pB[0+bs*2];
			tmp0 -= pB[0+bs*0] * u_02;
			tmp0 -= pB[0+bs*1] * u_12;
			pB[0+bs*2] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_02  = pA[0+bs*2];
		u_12  = pA[1+bs*2];
		u_12 -= pA[1+bs*0] * u_02;
		pA[1+bs*2] = u_12;
		if(m>2)
			{
			tmp2  = pA[2+bs*2];
			tmp2 -= pA[2+bs*0] * u_02;
			tmp2 -= pA[2+bs*1] * u_12;
			pA[2+bs*2] = tmp2;
			}
		}

	// find pivot & scale
	if(m>2)
		{
		sidamax_lib4(m-2, 2, &pA[2+bs*2], sda, &idamax, &tmp2);
		ipiv[2] = idamax+2;
		if(tmp2!=0)
			{
			if(ipiv[2]!=2)
				srowsw_lib(4, pA+2, pA+ipiv[2]/bs*bs*sda+ipiv[2]%bs);

			tmp2 = 1.0 / pA[2+bs*2];
			inv_diag_A[2] = tmp2;
			if(m>=4)
				{
				pA[3+bs*2] *= tmp2;
				pB = pA + bs*sda;
				for(k=0; k<ma-3; k+=4)
					{
					pB[0+bs*2] *= tmp2;
					pB[1+bs*2] *= tmp2;
					pB[2+bs*2] *= tmp2;
					pB[3+bs*2] *= tmp2;
					pB += bs*sda;
					}
				for( ; k<ma; k++)
					{
					pB[0+bs*2] *= tmp2;
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
		return;

	// fourth column

	// correct
	if(m>=4)
		{
		u_03  = pA[0+bs*3];
		u_13  = pA[1+bs*3];
		u_13 -= pA[1+bs*0] * u_03;
		pA[1+bs*3] = u_13;
		u_23  = pA[2+bs*3];
		u_23 -= pA[2+bs*0] * u_03;
		u_23 -= pA[2+bs*1] * u_13;
		pA[2+bs*3] = u_23;
		tmp3  = pA[3+bs*3];
		tmp3 -= pA[3+bs*0] * u_03;
		tmp3 -= pA[3+bs*1] * u_13;
		tmp3 -= pA[3+bs*2] * u_23;
		pA[3+bs*3] = tmp3;
		pB = pA + bs*sda;
		for(k=0; k<ma-3; k+=4)
			{
			tmp0  = pB[0+bs*3];
			tmp1  = pB[1+bs*3];
			tmp2  = pB[2+bs*3];
			tmp3  = pB[3+bs*3];
			tmp0 -= pB[0+bs*0] * u_03;
			tmp1 -= pB[1+bs*0] * u_03;
			tmp2 -= pB[2+bs*0] * u_03;
			tmp3 -= pB[3+bs*0] * u_03;
			tmp0 -= pB[0+bs*1] * u_13;
			tmp1 -= pB[1+bs*1] * u_13;
			tmp2 -= pB[2+bs*1] * u_13;
			tmp3 -= pB[3+bs*1] * u_13;
			tmp0 -= pB[0+bs*2] * u_23;
			tmp1 -= pB[1+bs*2] * u_23;
			tmp2 -= pB[2+bs*2] * u_23;
			tmp3 -= pB[3+bs*2] * u_23;
			pB[0+bs*3] = tmp0;
			pB[1+bs*3] = tmp1;
			pB[2+bs*3] = tmp2;
			pB[3+bs*3] = tmp3;
			pB += bs*sda;
			}
		for( ; k<ma; k++)
			{
			tmp0  = pB[0+bs*3];
			tmp0 -= pB[0+bs*0] * u_03;
			tmp0 -= pB[0+bs*1] * u_13;
			tmp0 -= pB[0+bs*2] * u_23;
			pB[0+bs*3] = tmp0;
			pB += 1;
			}
		}
	else // m = {2,3}
		{
		u_03  = pA[0+bs*3];
		u_13  = pA[1+bs*3];
		u_13 -= pA[1+bs*0] * u_03;
		pA[1+bs*3] = u_13;
		if(m>2)
			{
			u_23  = pA[2+bs*3];
			u_23 -= pA[2+bs*0] * u_03;
			u_23 -= pA[2+bs*1] * u_13;
			pA[2+bs*3] = u_23;
			}
		}

	if(m>3)
		{
		// find pivot & scale
		sidamax_lib4(m-3, 3, &pA[3+bs*3], sda, &idamax, &tmp3);
		ipiv[3] = idamax+3;
		if(tmp3!=0)
			{
			if(ipiv[3]!=3)
				srowsw_lib(4, pA+3, pA+ipiv[3]/bs*bs*sda+ipiv[3]%bs);

			tmp3 = 1.0 / pA[3+bs*3];
			inv_diag_A[3] = tmp3;
			pB = pA + bs*sda;
			for(k=0; k<ma-3; k+=4)
				{
				pB[0+bs*3] *= tmp3;
				pB[1+bs*3] *= tmp3;
				pB[2+bs*3] *= tmp3;
				pB[3+bs*3] *= tmp3;
				pB += bs*sda;
				}
			for( ; k<ma; k++)
				{
				pB[0+bs*3] *= tmp3;
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
#endif


	



