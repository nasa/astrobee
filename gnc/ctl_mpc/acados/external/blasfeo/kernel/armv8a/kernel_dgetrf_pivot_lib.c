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

#include "../../include/blasfeo_common.h"
#include "../../include/blasfeo_d_aux.h"
#include "../../include/blasfeo_d_kernel.h"



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)
// m>=1 and n={9,10,11,12}
void kernel_dgetrf_pivot_12_vs_lib(int m, double *C, int ldc, double *pd, int* ipiv, int n)
	{

	const int ps = 4;

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
#endif
	int sdu0 = (m+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	double *pU;
	int sdu;

	double *tmp_pU;
	int m4;

	if(m>K_MAX_STACK)
		{
		m4 = (m+3)/4*4;
		tmp_pU = malloc(3*4*m4*sizeof(double)+64);
		blasfeo_align_64_byte(tmp_pU, (void **) &pU);
		sdu = m4;
		}
	else
		{
		pU = pU0;
		sdu = sdu0;
		}

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// saturate n to 12
	n = 12<n ? 12 : n;

	int p = m<n ? m : n;

	int n_max;

	// fact left column
	kernel_dgetrf_pivot_8_vs_lib(m, C, ldc, pd, ipiv, n);

	n_max = p<8 ? p : 8;

	// apply pivot to right column
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib(n-8, C+ii+8*ldc, ldc, C+ipiv[ii]+8*ldc, ldc);
			}
		}

	// pack
	kernel_dpack_tn_4_vs_lib4(8, C+8*ldc, ldc, pU+8*sdu, n-8);

	// solve top right block
	kernel_dtrsm_nt_rl_one_4x4_vs_lib4c44c(0, pU+8*sdu, C, ldc, &d1, pU+8*sdu, pU+8*sdu, C, ldc, n-8, m);
	kernel_dtrsm_nt_rl_one_4x4_vs_lib4c44c(4, pU+8*sdu, C+4, ldc, &d1, pU+8*sdu+4*ps, pU+8*sdu+4*ps, C+4+4*ldc, ldc, n-8, m-4);

	// unpack
	kernel_dunpack_nt_4_vs_lib4(8, pU+8*sdu, C+8*ldc, ldc, n-8);

	if(m>8)
		{

		// correct rigth block
		ii = 8;
		// TODO larger kernels ???
		for(; ii<m; ii+=4)
			{
			kernel_dgemm_nt_4x4_vs_libc4cc(8, &dm1, C+ii, ldc, pU+8*sdu, &d1, C+ii+8*ldc, ldc, C+ii+8*ldc, ldc, m-ii, n-8);
			}

		// fact right column
		kernel_dgetrf_pivot_4_vs_lib(m-8, C+8+8*ldc, ldc, pd+8, ipiv+8, n-8);

		n_max = p;

		for(ii=8; ii<n_max; ii++)
			ipiv[ii] += 8;

		// apply pivot to left column
		for(ii=8; ii<n_max; ii++)
			{
			if(ipiv[ii]!=ii)
				{
				kernel_drowsw_lib(8, C+ii, ldc, C+ipiv[ii], ldc);
				}
			}

		}

	end:
	if(m>K_MAX_STACK)
		{
		free(tmp_pU);
		}

	return;

	}
#endif



// m>=1 and n={5,6,7,8}
void kernel_dgetrf_pivot_8_vs_lib(int m, double *C, int ldc, double *pd, int* ipiv, int n)
	{

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
#endif
	int sdu0 = (m+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	double *pU = pU0;
	int sdu = sdu0;

	double *tmp_pU;
	int m4;

	if(m>K_MAX_STACK)
		{
		m4 = (m+3)/4*4;
		tmp_pU = malloc(3*4*m4*sizeof(double)+64);
		blasfeo_align_64_byte(tmp_pU, (void **) &pU);
		sdu = m4;
		}
	else
		{
		pU = pU0;
		sdu = sdu0;
		}

	int ii;

	double *dummy = NULL;

	double d1 = 1.0;
	double dm1 = -1.0;

	// saturate n to 8
	n = 8<n ? 8 : n;

	int p = m<n ? m : n;

	int n_max;

	// fact left column
	kernel_dgetrf_pivot_4_vs_lib(m, C, ldc, pd, ipiv, n);

	n_max = p<4 ? p : 4;

	// apply pivot to right column
	for(ii=0; ii<n_max; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			kernel_drowsw_lib(n-4, C+ii+4*ldc, ldc, C+ipiv[ii]+4*ldc, ldc);
			}
		}

	// pack
	kernel_dpack_tn_4_vs_lib4(4, C+4*ldc, ldc, pU+4*sdu, n-4);

	// solve top right block
	kernel_dtrsm_nt_rl_one_4x4_vs_lib4c44c(0, dummy, dummy, 0, &d1, pU+4*sdu, pU+4*sdu, C, ldc, n-4, m);

	// unpack
	kernel_dunpack_nt_4_vs_lib4(4, pU+4*sdu, C+4*ldc, ldc, n-4);

	if(m>4)
		{

		// correct rigth block
		ii = 4;
		// TODO larger kernels ???
		for(; ii<m; ii+=4)
			{
			kernel_dgemm_nt_4x4_vs_libc4cc(4, &dm1, C+ii, ldc, pU+4*sdu, &d1, C+ii+4*ldc, ldc, C+ii+4*ldc, ldc, m-ii, n-4);
			}

		// fact right column
		kernel_dgetrf_pivot_4_vs_lib(m-4, C+4+4*ldc, ldc, pd+4, ipiv+4, n-4);

		n_max = p;

		for(ii=4; ii<n_max; ii++)
			ipiv[ii] += 4;

		// apply pivot to left column
		for(ii=4; ii<n_max; ii++)
			{
			if(ipiv[ii]!=ii)
				{
				kernel_drowsw_lib(4, C+ii, ldc, C+ipiv[ii], ldc);
				}
			}

		}

	end:
	if(m>K_MAX_STACK)
		{
		free(tmp_pU);
		}

	return;

	}




