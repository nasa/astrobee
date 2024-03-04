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

#include <blasfeo_common.h>
#include <blasfeo_d_kernel.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_hp_dgetr blasfeo_hp_cm_dgetr
#define blasfeo_dgetr blasfeo_cm_dgetr
#endif



static void blasfeo_dgetrf_m0_p0(int m, int n, double *pA, int lda, double *pB, int ldb, double *pAp, double *pBp)
	{
	int ii;

	ii=0;
#if defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<n-8; ii+=8)
		{
		kernel_dgetr_tn_8_p0_lib(m, pA+ii*lda, lda, pB+ii, ldb, pA+(ii+8)*lda, pB+ii+8);
		}
	if(ii<n-7)
		{
		kernel_dgetr_tn_8_p0_lib(m, pA+ii*lda, lda, pB+ii, ldb, pAp, pBp);
		ii+=8;
		}
	if(ii<n-3)
		{
		kernel_dgetr_tn_4_lib(m, pA+ii*lda, lda, pB+ii, ldb);
		ii+=4;
		}
#elif defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dgetr_tn_8_lib(m, pA+ii*lda, lda, pB+ii, ldb);
		}
	if(ii<n-3)
		{
		kernel_dgetr_tn_4_lib(m, pA+ii*lda, lda, pB+ii, ldb);
		ii+=4;
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dgetr_tn_4_lib(m, pA+ii*lda, lda, pB+ii, ldb);
		}
#endif
	if(ii<n)
		{
		kernel_dgetr_tn_4_vs_lib(m, pA+ii*lda, lda, pB+ii, ldb, n-ii);
		}
	
	return;

	}



static void blasfeo_dgetrf_m0(int m, int n, double *pA, int lda, double *pB, int ldb)
	{
	int ii;

	ii=0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<n-7; ii+=8)
		{
		kernel_dgetr_tn_8_lib(m, pA+ii*lda, lda, pB+ii, ldb);
		}
	if(ii<n-3)
		{
		kernel_dgetr_tn_4_lib(m, pA+ii*lda, lda, pB+ii, ldb);
		ii+=4;
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dgetr_tn_4_lib(m, pA+ii*lda, lda, pB+ii, ldb);
		}
#endif
	if(ii<n)
		{
		kernel_dgetr_tn_4_vs_lib(m, pA+ii*lda, lda, pB+ii, ldb, n-ii);
		}
	
	return;

	}



#if 0
static void blasfeo_dgetrf_n0_p0(int m, int n, double *pA, int lda, double *pB, int ldb, double *pAp, double *pBp)
	{
	int ii;

	ii=0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-8; ii+=8)
		{
		kernel_dgetr_nt_8_p0_lib(n, pA+ii, lda, pB+ii*ldb, ldb, pA+ii+8, pB+(ii+8)*ldb);
		}
	if(ii<m-7)
		{
		kernel_dgetr_nt_8_p0_lib(n, pA+ii, lda, pB+ii*ldb, ldb, pAp, pBp);
		ii+=8;
		}
	if(ii<m-3)
		{
		kernel_dgetr_nt_4_lib(n, pA+ii, lda, pB+ii*ldb, ldb);
		ii+=4;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dgetr_nt_4_lib(n, pA+ii, lda, pB+ii*ldb, ldb);
		}
#endif
	if(ii<m)
		{
//		kernel_dgetr_nt_4_vs_lib(n, pA+ii, lda, pB+ii*ldb, ldb, m-ii);
		}
	
	return;

	}



static void blasfeo_dgetrf_n0(int m, int n, double *pA, int lda, double *pB, int ldb)
	{
	int ii;

	ii=0;
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE) //| defined(TARGET_ARMV8A_ARM_CORTEX_A57) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	for(; ii<m-7; ii+=8)
		{
		kernel_dgetr_nt_8_lib(n, pA+ii, lda, pB+ii*ldb, ldb);
		}
	if(ii<m-3)
		{
		kernel_dgetr_nt_4_lib(n, pA+ii, lda, pB+ii*ldb, ldb);
		ii+=4;
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dgetr_nt_4_lib(n, pA+ii, lda, pB+ii*ldb, ldb);
		}
#endif
	if(ii<m)
		{
//		kernel_dgetr_nt_4_vs_lib(n, pA+ii, lda, pB+ii*ldb, ldb, m-ii);
		}
	
	return;

	}
#endif



// copy and transpose a generic strmat into a generic strmat
void blasfeo_hp_dgetr(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{
	// invalidate stored inverse diagonal
	sB->use_dA = 0;
	int lda = sA->m;
	int ldb = sB->m;
	double *pA = sA->pA + ai + aj*lda;
	double *pB = sB->pA + bi + bj*ldb;

	double *pAp, *pBp;

	int ii, jj;

	int mc = 16;
	int nc = 16;

	int mleft, nleft;

#if 1

	if(m<=56 & n<=56)
		{

		blasfeo_dgetrf_m0(m, n, pA, lda, pB, ldb);

		}
	else
		{

		for(ii=0; ii<m; ii+=mleft)
			{

			mleft = m-ii<=mc ? m-ii : mc;
			pAp = m-ii<=mc ? pA+ii : pA+ii+mc;
			pBp = m-ii<=mc ? pB+ii*ldb : pB+(ii+mc)*ldb;

			blasfeo_dgetrf_m0_p0(mleft, n, pA+ii, lda, pB+ii*ldb, ldb, pAp, pBp);

			}

		}

#else

	if(m<=56 & n<=56)
		{

		blasfeo_dgetrf_n0(m, n, pA, lda, pB, ldb);

		}
	else
		{

		for(ii=0; ii<n; ii+=nleft)
			{

			nleft = n-ii<=nc ? n-ii : nc;
			pAp = n-ii<=nc ? pA+ii*lda : pA+(ii+nc)*lda;
			pBp = n-ii<=nc ? pB+ii : pB+ii+nc;

			blasfeo_dgetrf_n0(m, nleft, pA+ii*lda, lda, pB+ii, ldb);
//			blasfeo_dgetrf_n0_p0(m, nleft, pA+ii*lda, lda, pB+ii, ldb, pAp, pBp);

			}

		}

#endif

	return;
	}



#if defined(LA_HIGH_PERFORMANCE)
//#ifndef HP_BLAS



void blasfeo_dgetr(int m, int n, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dmat *sB, int bi, int bj)
	{
	blasfeo_hp_dgetr(m, n, sA, ai, aj, sB, bi, bj);
	}



//#endif
#endif

