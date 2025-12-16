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

#include "../include/blasfeo_d_kernel.h"

#include "../../../../include/blasfeo_target.h"
#include "../../../../include/blasfeo_common.h"
#include "../../../../include/blasfeo_d_aux.h"
#include "../../../../include/blasfeo_d_kernel.h"
#include "../../../../include/blasfeo_d_blas.h"



void blasfeo_dpotrf(char *uplo, int *pm, double *C, int *pldc) // TODO int *info
	{

	int m = *pm;
	int ldc = *pldc;

	int ii, jj;

	int bs = 4;

	ALIGNED( double pd[256], 64 );

	ALIGNED( double pU[12*256], 64 );
	int sdu = 256;

	struct blasfeo_dmat sC;
	int sdc;
	double *pc;
	int sC_size, stot_size;
	void *smat_mem, *smat_mem_align;


	if(*uplo=='l')
		{
		if(m>=128)
			{
			goto l_1;
			}
		else
			{
			goto l_0;
			}
		}
	else
		{
		goto u;
		}


l_0:

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib4ccc(jj, pU, sdu, C+jj, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_12_lib4(4, C+ii+jj*ldc, ldc, pU+jj*bs, sdu);
			}
		kernel_dpotrf_nt_l_12x4_lib44cc(jj, pU, sdu, pU, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
		kernel_dpack_nn_8_lib4(4, C+ii+4+jj*ldc, ldc, pU+4*sdu+jj*bs, sdu);
		kernel_dpotrf_nt_l_8x4_lib44cc(jj+4, pU+4*sdu, sdu, pU+4*sdu, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pd+jj+4);
		kernel_dpack_nn_4_lib4(4, C+ii+8+(jj+4)*ldc, ldc, pU+8*sdu+(jj+4)*bs);
		kernel_dpotrf_nt_l_4x4_lib44cc(jj+8, pU+8*sdu, pU+8*sdu, C+ii+8+(jj+8)*ldc, ldc, C+ii+8+(jj+8)*ldc, ldc, pd+jj+8);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto l_0_left_4;
			}
		if(m-ii<=8)
			{
			goto l_0_left_8;
			}
		else
			{
			goto l_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib4ccc(jj, pU, sdu, C+jj, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_8_lib4(4, C+ii+jj*ldc, ldc, pU+jj*bs, sdu);
			}
		kernel_dpotrf_nt_l_8x4_lib44cc(jj, pU, sdu, pU, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
		kernel_dpack_nn_4_lib4(4, C+ii+4+jj*ldc, ldc, pU+4*sdu+jj*bs);
		kernel_dpotrf_nt_l_4x4_lib44cc(jj+4, pU+4*sdu, pU+4*sdu, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pd+jj+4);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto l_0_left_4;
			}
		else
			{
			goto l_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib4ccc(jj, pU, C+jj, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj);
			kernel_dpack_nn_4_lib4(4, C+ii+jj*ldc, ldc, pU+jj*bs);
			}
		kernel_dpotrf_nt_l_4x4_lib44cc(jj, pU, pU, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj);
		}
	if(ii<m)
		{
		goto l_0_left_4;
		}
#endif
	goto l_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
l_0_left_12:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib4ccc(jj, pU, sdu, C+jj, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, ii-jj);
		kernel_dpack_nn_12_vs_lib4(4, C+ii+jj*ldc, ldc, pU+jj*bs, sdu, m-ii);
		}
	kernel_dpotrf_nt_l_12x4_vs_lib44cc(jj, pU, sdu, pU, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, m-jj);
	kernel_dpack_nn_8_vs_lib4(4, C+ii+4+jj*ldc, ldc, pU+4*sdu+jj*bs, sdu, m-ii-4);
	kernel_dpotrf_nt_l_8x4_vs_lib44cc(jj+4, pU+4*sdu, sdu, pU+4*sdu, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pd+jj+4, m-ii-4, m-jj-4);
	kernel_dpack_nn_4_vs_lib4(4, C+ii+8+(jj+4)*ldc, ldc, pU+8*sdu+(jj+4)*bs, m-ii-8);
	kernel_dpotrf_nt_l_4x4_vs_lib44cc(jj+8, pU+8*sdu, pU+8*sdu, C+ii+8+(jj+8)*ldc, ldc, C+ii+8+(jj+8)*ldc, ldc, pd+jj+8, m-ii-8, m-jj-8);
	goto l_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
l_0_left_8:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib4ccc(jj, pU, sdu, C+jj, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, ii-jj);
		kernel_dpack_nn_8_vs_lib4(4, C+ii+jj*ldc, ldc, pU+jj*bs, sdu, m-ii);
		}
	kernel_dpotrf_nt_l_8x4_vs_lib44cc(jj, pU, sdu, pU, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, m-jj);
	kernel_dpack_nn_4_vs_lib4(4, C+ii+4+jj*ldc, ldc, pU+4*sdu+jj*bs, m-ii-4);
	kernel_dpotrf_nt_l_4x4_vs_lib44cc(jj+4, pU+4*sdu, pU+4*sdu, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pd+jj+4, m-ii-4, m-jj-4);
	goto l_0_return;
#endif

l_0_left_4:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib4ccc(jj, pU, C+jj, ldc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pd+jj, m-ii, ii-jj);
		kernel_dpack_nn_4_vs_lib4(4, C+ii+jj*ldc, ldc, pU+jj*bs, m-ii);
		}
	kernel_dpotrf_nt_l_4x4_vs_lib44cc(jj, pU, pU, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pd+jj, m-ii, m-jj);
	goto l_0_return;

l_0_return:
	return;



l_1:
	
	sC_size = blasfeo_memsize_dmat(m, m);
	stot_size = sC_size + m*sizeof(double);
	smat_mem = malloc(stot_size+63);
	smat_mem_align = (void *) ( ( ( (unsigned long long) smat_mem ) + 63) / 64 * 64 );
	blasfeo_create_dmat(m, m, &sC, smat_mem_align);
	sdc = sC.cn;
	pc = smat_mem_align + sC_size; // TODO use the memory in sC for that !!!

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		jj = 0;
		for(; jj<ii; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_12x4_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pc+jj);
			kernel_dpack_nn_12_lib4(4, C+ii+jj*ldc, ldc, sC.pA+ii*sdc+jj*bs, sdc);
			}
		kernel_dpotrf_nt_l_12x4_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pc+jj);
		kernel_dpack_nn_8_lib4(4, C+ii+4+jj*ldc, ldc, sC.pA+(ii+4)*sdc+jj*bs, sdc);
		kernel_dpotrf_nt_l_8x4_lib44cc(jj+4, sC.pA+(ii+4)*sdc, sdc, sC.pA+(jj+4)*sdc, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pc+jj+4);
		kernel_dpack_nn_4_lib4(4, C+ii+8+(jj+4)*ldc, ldc, sC.pA+(ii+8)*sdc+(jj+4)*bs);
		kernel_dpotrf_nt_l_4x4_lib44cc(jj+8, sC.pA+(ii+8)*sdc, sC.pA+(jj+8)*sdc, C+ii+8+(jj+8)*ldc, ldc, C+ii+8+(jj+8)*ldc, ldc, pc+jj+8);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto l_1_left_4;
			}
		if(m-ii<=8)
			{
			goto l_1_left_8;
			}
		else
			{
			goto l_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		jj = 0;
		for(; jj<ii; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_8x4_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pc+jj);
			kernel_dpack_nn_8_lib4(4, C+ii+jj*ldc, ldc, sC.pA+ii*sdc+jj*bs, sdc);
			}
		kernel_dpotrf_nt_l_8x4_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pc+jj);
		kernel_dpack_nn_4_lib4(4, C+ii+4+jj*ldc, ldc, sC.pA+(ii+4)*sdc+jj*bs);
		kernel_dpotrf_nt_l_4x4_lib44cc(jj+4, sC.pA+(ii+4)*sdc, sC.pA+(jj+4)*sdc, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pc+jj+4);
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto l_1_left_4;
			}
		else
			{
			goto l_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		jj = 0;
		for(; jj<ii; jj+=4)
			{
			kernel_dtrsm_nt_rl_inv_4x4_lib44cc(jj, sC.pA+ii*sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pc+jj);
			kernel_dpack_nn_4_lib4(4, C+ii+jj*ldc, ldc, sC.pA+ii*sdc+jj*bs);
			}
		kernel_dpotrf_nt_l_4x4_lib44cc(jj, sC.pA+ii*sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pc+jj);
		}
	if(ii<m)
		{
		goto l_1_left_4;
		}
#endif
	goto l_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
l_1_left_12:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pc+jj, m-ii, ii-jj);
		kernel_dpack_nn_12_vs_lib4(4, C+ii+jj*ldc, ldc, sC.pA+ii*sdc+jj*bs, sdc, m-ii);
		}
	kernel_dpotrf_nt_l_12x4_vs_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pc+jj, m-ii, m-jj);
	kernel_dpack_nn_8_vs_lib4(4, C+ii+4+jj*ldc, ldc, sC.pA+(ii+4)*sdc+jj*bs, sdc, m-ii-4);
	kernel_dpotrf_nt_l_8x4_vs_lib44cc(jj+4, sC.pA+(ii+4)*sdc, sdc, sC.pA+(jj+4)*sdc, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pc+jj+4, m-ii-4, m-jj-4);
	kernel_dpack_nn_4_vs_lib4(4, C+ii+8+(jj+4)*ldc, ldc, sC.pA+(ii+8)*sdc+(jj+4)*bs, m-ii-8);
	kernel_dpotrf_nt_l_4x4_vs_lib44cc(jj+8, sC.pA+(ii+8)*sdc, sC.pA+(jj+8)*sdc, C+ii+8+(jj+8)*ldc, ldc, C+ii+8+(jj+8)*ldc, ldc, pc+jj+8, m-ii-8, m-jj-8);


	goto l_1_return;
#endif


#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
l_1_left_8:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pc+jj, m-ii, ii-jj);
		kernel_dpack_nn_8_vs_lib4(4, C+ii+jj*ldc, ldc, sC.pA+ii*sdc+jj*bs, sdc, m-ii);
		}
	kernel_dpotrf_nt_l_8x4_vs_lib44cc(jj, sC.pA+ii*sdc, sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pc+jj, m-ii, m-jj);
	kernel_dpack_nn_4_vs_lib4(4, C+ii+4+jj*ldc, ldc, sC.pA+(ii+4)*sdc+jj*bs, m-ii-4);
	kernel_dpotrf_nt_l_4x4_vs_lib44cc(jj+4, sC.pA+(ii+4)*sdc, sC.pA+(jj+4)*sdc, C+ii+4+(jj+4)*ldc, ldc, C+ii+4+(jj+4)*ldc, ldc, pc+jj+4, m-ii-4, m-jj-4);
	goto l_1_return;
#endif

l_1_left_4:
	for(jj=0; jj<ii; jj+=4)
		{
		kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc(jj, sC.pA+ii*sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, C+jj+jj*ldc, ldc, pc+jj, m-ii, ii-jj);
		kernel_dpack_nn_4_vs_lib4(4, C+ii+jj*ldc, ldc, sC.pA+ii*sdc+jj*bs, m-ii);
		}
	kernel_dpotrf_nt_l_4x4_vs_lib44cc(jj, sC.pA+ii*sdc, sC.pA+jj*sdc, C+ii+jj*ldc, ldc, C+ii+jj*ldc, ldc, pc+jj, m-ii, m-jj);
	goto l_1_return;

l_1_return:
	free(smat_mem);
	return;



u:
	
	sC_size = blasfeo_memsize_dmat(m, m);
	stot_size = sC_size;
	smat_mem = malloc(stot_size+63);
	smat_mem_align = (void *) ( ( ( (unsigned long long) smat_mem ) + 63) / 64 * 64 );
	blasfeo_create_dmat(m, m, &sC, smat_mem_align);
	sdc = sC.cn;

	blasfeo_pack_tran_dmat(m, m, C, ldc, &sC, 0, 0); // TODO triangular

	blasfeo_dpotrf_l(m, &sC, 0, 0, &sC, 0, 0);

	blasfeo_unpack_tran_dmat(m, m, &sC, 0, 0, C, ldc); // TODO triangular

	// TODO check diagonal to compute info

	free(smat_mem);

	}
