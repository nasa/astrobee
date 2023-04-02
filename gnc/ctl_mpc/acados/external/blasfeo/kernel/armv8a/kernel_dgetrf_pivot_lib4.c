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

#include <stdlib.h>
#include <stdio.h>


#include "../../include/blasfeo_target.h"
#include "../../include/blasfeo_common.h"
#include "../../include/blasfeo_d_aux.h"
#include "../../include/blasfeo_d_kernel.h"



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)
void kernel_dgetrf_pivot_12_lib4(int m, double *pC, int sdc, double *pd, int *ipiv)
	{

	const int ps = 4;

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// fact left column
	kernel_dgetrf_pivot_8_lib4(m, pC, sdc, pd, ipiv);

	// apply pivot to right column
	for(ii=0; ii<8; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(4, pC+ii/ps*ps*sdc+ii%ps+8*ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps+8*ps);
			}
		}

	// solve top right block
	kernel_dtrsm_nn_ll_one_8x4_lib4(0, dummy, 0, dummy, 0, &d1, pC+8*ps, sdc, pC+8*ps, sdc, pC, sdc);

	// correct rigth block
	ii = 8;
	for(; ii<m-11; ii+=12)
		{
		kernel_dgemm_nn_12x4_lib4(8, &dm1, pC+ii*sdc, sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, sdc, pC+ii*sdc+8*ps, sdc);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			kernel_dgemm_nn_4x4_vs_lib4(8, &dm1, pC+ii*sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, pC+ii*sdc+8*ps, m-ii, 4);
			}
		else if(m-ii<=8)
			{
			kernel_dgemm_nn_8x4_vs_lib4(8, &dm1, pC+ii*sdc, sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, sdc, pC+ii*sdc+8*ps, sdc, m-ii, 4);
			}
		else //if(m-ii<=12)
			{
			kernel_dgemm_nn_12x4_vs_lib4(8, &dm1, pC+ii*sdc, sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, sdc, pC+ii*sdc+8*ps, sdc, m-ii, 4);
			}
		}

	// fact right column
	kernel_dgetrf_pivot_4_lib4(m-8, pC+8*sdc+8*ps, sdc, pd+8, ipiv+8);

	for(ii=8; ii<12; ii++)
		ipiv[ii] += 8;

	// apply pivot to left column
	for(ii=8; ii<12; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(8, pC+ii/ps*ps*sdc+ii%ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps);
			}
		}

	return;

	}
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)
// m>=1 and n={9,10,11,12}
void kernel_dgetrf_pivot_12_vs_lib4(int m, double *pC, int sdc, double *pd, int *ipiv, int n)
	{

	const int ps = 4;

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// saturate n to 12
	n = 12<n ? 12 : n;

	int p = m<n ? m : n;

	int n_max;

	// fact left column
	kernel_dgetrf_pivot_8_vs_lib4(m, pC, sdc, pd, ipiv, n);

	n_max = p<8 ? p : 8;

	// apply pivot to right column
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(n-8, pC+ii/ps*ps*sdc+ii%ps+8*ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps+8*ps);
			}
		}

	// solve top right block
	kernel_dtrsm_nn_ll_one_8x4_vs_lib4(0, dummy, 0, dummy, 0, &d1, pC+8*ps, sdc, pC+8*ps, sdc, pC, sdc, m, n-8);

	if(m>8)
		{

		// correct rigth block
		ii = 8;
		for(; ii<m-8; ii+=12)
			{
			kernel_dgemm_nn_12x4_vs_lib4(8, &dm1, pC+ii*sdc, sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, sdc, pC+ii*sdc+8*ps, sdc, m-ii, n-8);
			}
		if(ii<m)
			{
			if(m-ii<=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(8, &dm1, pC+ii*sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, pC+ii*sdc+8*ps, m-ii, n-8);
				}
			else //if(m-ii<=8)
				{
				kernel_dgemm_nn_8x4_vs_lib4(8, &dm1, pC+ii*sdc, sdc, 0, pC+8*ps, sdc, &d1, pC+ii*sdc+8*ps, sdc, pC+ii*sdc+8*ps, sdc, m-ii, n-8);
				}
			}

		// fact right column
		kernel_dgetrf_pivot_4_vs_lib4(m-8, pC+8*sdc+8*ps, sdc, pd+8, ipiv+8, n-8);

		n_max = p;

		for(ii=8; ii<n_max; ii++)
			ipiv[ii] += 8;

		// apply pivot to left column
		for(ii=8; ii<n_max; ii++)
			{
			if(ipiv[ii]!=ii)
				{
				kernel_drowsw_lib4(8, pC+ii/ps*ps*sdc+ii%ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps);
				}
			}

		}

	return;

	}
#endif



void kernel_dgetrf_pivot_8_lib4(int m, double *pC, int sdc, double *pd, int *ipiv)
	{

	const int ps = 4;

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// fact left column
	kernel_dgetrf_pivot_4_lib4(m, pC, sdc, pd, ipiv);

	// apply pivot to right column
	for(ii=0; ii<4; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(4, pC+ii/ps*ps*sdc+ii%ps+4*ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps+4*ps);
			}
		}

	// solve top right block
	kernel_dtrsm_nn_ll_one_4x4_lib4(0, dummy, dummy, 0, &d1, pC+4*ps, pC+4*ps, pC);

	// correct rigth block
	ii = 4;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-11; ii+=12)
		{
		kernel_dgemm_nn_12x4_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			kernel_dgemm_nn_4x4_vs_lib4(4, &dm1, pC+ii*sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, pC+ii*sdc+4*ps, m-ii, 4);
			}
		else if(m-ii<=8)
			{
			kernel_dgemm_nn_8x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, 4);
			}
		else //if(m-ii<=12)
			{
			kernel_dgemm_nn_12x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, 4);
			}
		}
#else
	for(; ii<m-7; ii+=8)
		{
		kernel_dgemm_nn_8x4_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			kernel_dgemm_nn_4x4_vs_lib4(4, &dm1, pC+ii*sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, pC+ii*sdc+4*ps, m-ii, 4);
			}
		else //if(m-ii<=8)
			{
			kernel_dgemm_nn_8x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, 4);
			}
		}
#endif

	// fact right column
	kernel_dgetrf_pivot_4_lib4(m-4, pC+4*sdc+4*ps, sdc, pd+4, ipiv+4);

	for(ii=4; ii<8; ii++)
		ipiv[ii] += 4;

	// apply pivot to left column
	for(ii=4; ii<8; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(4, pC+ii/ps*ps*sdc+ii%ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps);
			}
		}

	return;

	}



// m>=1 and n={5,6,7,8}
void kernel_dgetrf_pivot_8_vs_lib4(int m, double *pC, int sdc, double *pd, int *ipiv, int n)
	{

	const int ps = 4;

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// saturate n to 8
	n = 8<n ? 8 : n;

	int p = m<n ? m : n;

	int n_max;

	// fact left column
	kernel_dgetrf_pivot_4_vs_lib4(m, pC, sdc, pd, ipiv, 4);

	n_max = p<4 ? p : 4;

	// apply pivot to right column
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib4(n-4, pC+ii/ps*ps*sdc+ii%ps+4*ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps+4*ps);
			}
		}

	// solve top right block
	kernel_dtrsm_nn_ll_one_4x4_vs_lib4(0, dummy, dummy, 0, &d1, pC+4*ps, pC+4*ps, pC, m, n-4);

	if(m>4)
		{

		// correct rigth block
		ii = 4;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)
		for(; ii<m-8; ii+=12)
			{
			kernel_dgemm_nn_12x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, n-4);
			}
		if(ii<m)
			{
			if(m-ii<=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(4, &dm1, pC+ii*sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, pC+ii*sdc+4*ps, m-ii, n-4);
				}
			else //if(m-ii<=8)
				{
				kernel_dgemm_nn_8x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, n-4);
				}
			}
#else
		for(; ii<m-4; ii+=8)
			{
			kernel_dgemm_nn_8x4_vs_lib4(4, &dm1, pC+ii*sdc, sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, sdc, pC+ii*sdc+4*ps, sdc, m-ii, n-4);
			}
		if(ii<m)
			{
			kernel_dgemm_nn_4x4_vs_lib4(4, &dm1, pC+ii*sdc, 0, pC+4*ps, sdc, &d1, pC+ii*sdc+4*ps, pC+ii*sdc+4*ps, m-ii, n-4);
			}
#endif

		// fact right column
		kernel_dgetrf_pivot_4_vs_lib4(m-4, pC+4*sdc+4*ps, sdc, pd+4, ipiv+4, n-4);

		n_max = p;

		for(ii=4; ii<n_max; ii++)
			ipiv[ii] += 4;

		// apply pivot to left column
		for(ii=4; ii<n_max; ii++)
			{
			if(ipiv[ii]!=ii)
				{
				kernel_drowsw_lib4(4, pC+ii/ps*ps*sdc+ii%ps, pC+ipiv[ii]/ps*ps*sdc+ipiv[ii]%ps);
				}
			}

		}

	return;

	}





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgetrf_pivot_lib.c"

#endif
