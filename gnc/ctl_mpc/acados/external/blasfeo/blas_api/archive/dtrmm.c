/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS for embedded optimization.                                                      *
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

#include "../include/blasfeo_target.h"
#include "../include/blasfeo_common.h"
#include "../include/blasfeo_d_aux.h"
#include "../include/blasfeo_d_kernel.h"



#if defined(FORTRAN_BLAS_API)
#define blasfeo_dtrmm dtrmm_
#endif



#if defined(FALLBACK_TO_EXTERNAL_BLAS)
void dtrmv_(char *uplo, char *transa, char *diag, int *m, double *A, int *lda, double *x, int *incx);
void dscal_(int *m, double *alpha, double *x, int *incx);
#endif



void blas_dtrmm(char *side, char *uplo, char *transa, char *diag, int *pm, int *pn, double *alpha, double *A, int *plda, double *B, int *pldb)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_dtrmm %c %c %c %c %d %d %f %p %d %p %d\n", *side, *uplo, *transa, *diag, *pm, *pn, *alpha, A, *plda, B, *pldb);
#endif

	int m = *pm;
	int n = *pn;
	int lda = *plda;
	int ldb = *pldb;

#if defined(DIM_CHECK)
	if( !(*side=='l' | *side=='L' | *side=='r' | *side=='R') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for side\n");
		return;
		}
	if( !(*uplo=='l' | *uplo=='L' | *uplo=='u' | *uplo=='U') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for uplo\n");
		return;
		}
	if( !(*transa=='c' | *transa=='C' | *transa=='n' | *transa=='N' | *transa=='t' | *transa=='T') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for transa\n");
		return;
		}
	if( !(*diag=='n' | *diag=='N' | *diag=='u' | *diag=='U') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for diag\n");
		return;
		}
#endif

	char c_n = 'n';
	char c_t = 't';
	int i_1 = 1;

#if defined(FALLBACK_TO_EXTERNAL_BLAS)
	// fallback to dtrmv if B is a vector
	if(n==1 & (*side=='l' | *side=='L'))
		{
		dtrmv_(uplo, transa, diag, pm, A, plda, B, &i_1);
		if(*alpha!=1.0)
			{
			dscal_(pm, alpha, B, &i_1);
			}
		return;
		}
	else if(m==1 & (*side=='r' | *side=='r'))
		{
		if(*transa=='n' | *transa=='N')
			{
			dtrmv_(uplo, &c_t, diag, pn, A, plda, B, pldb);
			}
		else
			{
			dtrmv_(uplo, &c_n, diag, pn, A, plda, B, pldb);
			}
		if(*alpha!=1.0)
			{
			dscal_(pn, alpha, B, pldb);
			}
		return;
		}
#endif

	double d_0 = 0.0;

	int ii, jj;

	int ps = 4;

	if(m<=0 | n<=0)
		return;

// TODO visual studio alignment
#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	ALIGNED( double pU0[3*4*K_MAX_STACK], 64 );
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE) | defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	ALIGNED( double pU0[2*4*K_MAX_STACK], 64 );
#elif defined(TARGET_GENERIC)
	double pU0[1*4*K_MAX_STACK];
#else
	ALIGNED( double pU0[1*4*K_MAX_STACK], 64 );
#endif

	int k0;
	// TODO update if necessary !!!!!
	if(*side=='l' | *side=='L')
		k0 = m;
	else
		k0 = n;

	int sdu0 = (k0+3)/4*4;
	sdu0 = sdu0<K_MAX_STACK ? sdu0 : K_MAX_STACK;

	struct blasfeo_dmat sA, sB;
	double *pU, *pB, *dA, *dB;
	int sda, sdb, sdu;
	int sA_size, sB_size;
	void *mem;
	char *mem_align;
	int m1, n1;
	int pack_tran = 0;

	if(*side=='l' | *side=='L') // _l
		{
		if(*uplo=='l' | *uplo=='L') // _ll
			{
			if(*transa=='n' | *transa=='N') // _lln
				{
				if(*diag=='n' | *diag=='N') // _llnn
					{
					goto llnn;
					}
				else //if(*diag=='u' | *diag=='U') // _llnu
					{
					goto llnu;
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _llt
				{
				if(*diag=='n' | *diag=='N') // _lltn
					{
					goto lltn;
					}
				else //if(*diag=='u' | *diag=='U') // _lltu
					{
					goto lltu;
					}
				}
			}
		else if(*uplo=='u' | *uplo=='U') // _lu
			{
			if(*transa=='n' | *transa=='N') // _lun
				{
				if(*diag=='n' | *diag=='N') // _lunn
					{
					goto lunn;
					}
				else //if(*diag=='u' | *diag=='U') // _lunu
					{
					goto lunu;
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _lut
				{
				if(*diag=='n' | *diag=='N') // _lutn
					{
					goto lutn;
					}
				else //if(*diag=='u' | *diag=='U') // _lutu
					{
					goto lutu;
					}
				}
			}
		}
	else if(*side=='r' | *side=='R') // _r
		{
		if(*uplo=='l' | *uplo=='L') // _rl
			{
			if(*transa=='n' | *transa=='N') // _rln
				{
				if(*diag=='n' | *diag=='N') // _rlnn
					{
					goto rlnn;
					}
				else //if(*diag=='u' | *diag=='U') // _rlnu
					{
					goto rlnu;
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _rlt
				{
				if(*diag=='n' | *diag=='N') // _rltn
					{
					goto rltn;
					}
				else //if(*diag=='u' | *diag=='U') // _rltu
					{
					goto rltu;
					}
				}
			}
		else if(*uplo=='u' | *uplo=='U') // _ru
			{
			if(*transa=='n' | *transa=='N') // _run
				{
				if(*diag=='n' | *diag=='N') // _runn
					{
					goto runn;
					}
				else //if(*diag=='u' | *diag=='U') // _runu
					{
					goto runu;
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _rut
				{
				if(*diag=='n' | *diag=='N') // _rutn
					{
					goto rutn;
					}
				else //if(*diag=='u' | *diag=='U') // _rutu
					{
					goto rutu;
					}
				}
			}
		}



/************************************************
* llnn
************************************************/
llnn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto llnn_1;
		}
	else
		{
		goto llnn_0;
		}

llnn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x12_tran_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_0_left_4;
			}
		else if(n-ii<=8)
			{
			goto llnn_0_left_8;
			}
		else
			{
			goto llnn_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x8_tran_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_0_left_4;
			}
		else
			{
			goto llnn_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x4_tran_lib4c4c(jj, alpha, pU, A+jj, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto llnn_0_left_4;
		}
#endif
goto llnn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
llnn_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
llnn_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnn_0_return;
#endif

llnn_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnn_0_return;

llnn_0_return:
	return;



llnn_1:
	m1 = (m+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, m1);
	sB_size = blasfeo_memsize_dmat(m1, m1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, m, &sA, (void *) mem_align);
	blasfeo_create_dmat(m, m, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // upper to lower
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tn_4_vs_lib4(m, A+ii*lda, lda, pB+ii*sdb, m-ii);
			}
		}
	else // lower to lower
		{
#if 1
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb);
			}
		if(ii<m)
			{
			kernel_dpack_tt_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb, m-ii);
			}
#else
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(ii+4, A+ii, lda, pB+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_nn_4_vs_lib4(m, A+ii, lda, pB+ii*sdb, m-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x12_tran_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_4x12_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_1_left_4;
			}
		else if(n-ii<=8)
			{
			goto llnn_1_left_8;
			}
		else
			{
			goto llnn_1_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x8_tran_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_4x8_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnn_1_left_4;
			}
		else
			{
			goto llnn_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x4_tran_lib444c(jj, alpha, pU, pB+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_4x4_tran_vs_lib444c(jj, alpha, pU, pB+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto llnn_1_left_4;
		}
#endif
goto llnn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
llnn_1_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x12_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
llnn_1_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x8_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnn_1_return;
#endif

llnn_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x4_tran_vs_lib444c(jj, alpha, pU, pB+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnn_1_return;

llnn_1_return:
	free(mem);
	return;



/************************************************
* llnu
************************************************/
llnu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto llnu_1;
		}
	else
		{
		goto llnu_0;
		}

llnu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x12_tran_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_0_left_4;
			}
		if(n-ii<=8)
			{
			goto llnu_0_left_8;
			}
		else
			{
			goto llnu_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x8_tran_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_one_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_0_left_4;
			}
		else
			{
			goto llnu_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x4_tran_lib4c4c(jj, alpha, pU, A+jj, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_one_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto llnu_0_left_4;
		}
#endif
goto llnu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
llnu_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
llnu_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnu_0_return;
#endif

llnu_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnu_0_return;

llnu_0_return:
	return;



llnu_1:
	m1 = (m+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, m1);
	sB_size = blasfeo_memsize_dmat(m1, m1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, m, &sA, (void *) mem_align);
	blasfeo_create_dmat(m, m, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // upper to lower
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tn_4_vs_lib4(m, A+ii*lda, lda, pB+ii*sdb, m-ii);
			}
		}
	else // lower to lower
		{
#if 1
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb);
			}
		if(ii<m)
			{
			kernel_dpack_tt_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb, m-ii);
			}
#else
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(ii+4, A+ii, lda, pB+ii*sdb);
			}
		if(ii<m)
			{
			kernel_dpack_nn_4_vs_lib4(m, A+ii, lda, pB+ii*sdb, m-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x12_tran_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_1_left_4;
			}
		else if(n-ii<=8)
			{
			goto llnu_1_left_8;
			}
		else
			{
			goto llnu_1_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x8_tran_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_one_4x8_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto llnu_1_left_4;
			}
		else
			{
			goto llnu_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x4_tran_lib444c(jj, alpha, pU, pB+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_rl_one_4x4_tran_vs_lib444c(jj, alpha, pU, pB+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto llnu_1_left_4;
		}
#endif
goto llnu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
llnu_1_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
llnu_1_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x8_tran_vs_lib444c(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnu_1_return;
#endif

llnu_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x4_tran_vs_lib444c(jj, alpha, pU, pB+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto llnu_1_return;

llnu_1_return:
	free(mem);
	return;



/************************************************
* lltn
************************************************/
lltn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto lunn_1;
		}
	else
		{
		goto lltn_0;
		}

lltn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x12_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_rl_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltn_0_left_4;
			}
		if(n-ii<=8)
			{
			goto lltn_0_left_8;
			}
		else
			{
			goto lltn_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x8_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_rl_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltn_0_left_4;
			}
		else
			{
			goto lltn_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x4_tran_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_rl_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lltn_0_left_4;
		}
#endif
	goto lltn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lltn_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lltn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lltn_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lltn_0_return;
#endif

lltn_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lltn_0_return;

lltn_0_return:
	return;



/************************************************
* lltu
************************************************/
lltu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto lunu_1;
		}
	else
		{
		goto lltu_0;
		}

lltu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_one_4x12_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_rl_one_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltu_0_left_4;
			}
		if(n-ii<=8)
			{
			goto lltu_0_left_8;
			}
		else
			{
			goto lltu_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_one_4x8_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_rl_one_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lltu_0_left_4;
			}
		else
			{
			goto lltu_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_one_4x4_tran_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_rl_one_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lltu_0_left_4;
		}
#endif
	goto lltu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lltu_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_rl_one_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lltu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lltu_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_rl_one_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lltu_0_return;
#endif

lltu_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_rl_one_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lltu_0_return;

lltu_0_return:
	return;



/************************************************
* lunn
************************************************/
lunn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto lunn_1;
		}
	else
		{
		goto lunn_0;
		}

lunn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x12_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_0_left_4;
			}
		if(n-ii<=8)
			{
			goto lunn_0_left_8;
			}
		else
			{
			goto lunn_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x8_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_0_left_4;
			}
		else
			{
			goto lunn_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x4_tran_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lunn_0_left_4;
		}
#endif
goto lunn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunn_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunn_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunn_0_return;
#endif

lunn_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunn_0_return;

lunn_0_return:
	return;



lunn_1:
	m1 = (m+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, m1);
	sB_size = blasfeo_memsize_dmat(m1, m1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, m, &sA, (void *) mem_align);
	blasfeo_create_dmat(m, m, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // lower to upper
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<m)
			{
			kernel_dpack_tn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, m-ii);
			}
		}
	else // upper to upper
		{
#if 1
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
			}
		if(ii<m)
			{
			kernel_dpack_tt_4_vs_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb, m-ii);
			}
#else
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<m)
			{
			kernel_dpack_nn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, m-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x12_tran_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_4x12_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_1_left_4;
			}
		if(n-ii<=8)
			{
			goto lunn_1_left_8;
			}
		else
			{
			goto lunn_1_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x8_tran_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_4x8_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunn_1_left_4;
			}
		else
			{
			goto lunn_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x4_tran_lib444c(m-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_4x4_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lunn_1_left_4;
		}
#endif
goto lunn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunn_1_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x12_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunn_1_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x8_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunn_1_return;
#endif

lunn_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x4_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunn_1_return;

lunn_1_return:
	free(mem);
	return;



/************************************************
* lunu
************************************************/
lunu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto lunu_1;
		}
	else
		{
		goto lunu_0;
		}

lunu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x12_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_0_left_4;
			}
		if(n-ii<=8)
			{
			goto lunu_0_left_8;
			}
		else
			{
			goto lunu_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x8_tran_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_one_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_0_left_4;
			}
		else
			{
			goto lunu_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x4_tran_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_one_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lunu_0_left_4;
		}
#endif
goto lunu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunu_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunu_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x8_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunu_0_return;
#endif

lunu_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x4_tran_vs_lib4c4c(m-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunu_0_return;

lunu_0_return:
	return;



lunu_1:
	m1 = (m+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, m1);
	sB_size = blasfeo_memsize_dmat(m1, m1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, m, &sA, (void *) mem_align);
	blasfeo_create_dmat(m, m, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // lower to upper
		{
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<m)
			{
			kernel_dpack_tn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, m-ii);
			}
		}
	else // upper to upper
		{
#if 1
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
			}
		if(ii<m)
			{
			kernel_dpack_tt_4_vs_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb, m-ii);
			}
#else
		for(ii=0; ii<m-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<m)
			{
			kernel_dpack_nn_4_vs_lib4(m-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, m-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x12_tran_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_1_left_4;
			}
		if(n-ii<=8)
			{
			goto lunu_1_left_8;
			}
		else
			{
			goto lunu_1_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x8_tran_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_one_4x8_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lunu_1_left_4;
			}
		else
			{
			goto lunu_1_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x4_tran_lib444c(m-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nt_ru_one_4x4_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lunu_1_left_4;
		}
#endif
goto lunu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lunu_1_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lunu_1_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x8_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunu_1_return;
#endif

lunu_1_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x4_tran_vs_lib444c(m-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lunu_1_return;

lunu_1_return:
	free(mem);
	return;



/************************************************
* lutn
************************************************/
lutn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto llnn_1;
		}
	else
		{
		goto lutn_0;
		}

lutn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_4x12_tran_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_ru_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutn_0_left_4;
			}
		if(n-ii<=8)
			{
			goto lutn_0_left_8;
			}
		else
			{
			goto lutn_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_4x8_tran_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_ru_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutn_0_left_4;
			}
		else
			{
			goto lutn_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_4x4_tran_lib4c4c(jj, alpha, pU, A+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_ru_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lutn_0_left_4;
		}
#endif
goto lutn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lutn_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_ru_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lutn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lutn_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_ru_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lutn_0_return;
#endif

lutn_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_ru_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lutn_0_return;

lutn_0_return:
	return;



/************************************************
* lutu
************************************************/
lutu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto llnu_1;
		}
	else
		{
		goto lutu_0;
		}

lutu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<n-11; ii+=12)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu);
		kernel_dpack_tn_4_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_one_4x12_tran_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_ru_one_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutu_0_left_4;
			}
		if(n-ii<=8)
			{
			goto lutu_0_left_8;
			}
		else
			{
			goto lutu_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<n-7; ii+=8)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		kernel_dpack_tn_4_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_one_4x8_tran_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_ru_one_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		if(n-ii<=4)
			{
			goto lutu_0_left_4;
			}
		else
			{
			goto lutu_0_left_8;
			}
		}
#else
	for(; ii<n-3; ii+=4)
		{
		kernel_dpack_tn_4_lib4(m, B+ii*ldb, ldb, pU);
		for(jj=0; jj<m-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_one_4x4_tran_lib4c4c(jj, alpha, pU, A+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb);
			}
		if(jj<m)
			{
			kernel_dtrmm_nn_ru_one_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
			}
		}
	if(ii<n)
		{
		goto lutu_0_left_4;
		}
#endif
goto lutu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
lutu_0_left_12:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+4*sdu, n-(ii+4));
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+8)*ldb, ldb, pU+8*sdu, n-(ii+8));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_ru_one_4x12_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lutu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) //| defined(TARGET_X64_INTEL_SANDY_BRIDGE)
lutu_0_left_8:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	kernel_dpack_tn_4_vs_lib4(m, B+(ii+4)*ldb, ldb, pU+ps*sdu, n-(ii+4));
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_ru_one_4x8_tran_vs_lib4c4c(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, pU+jj*ps, sdu, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lutu_0_return;
#endif

lutu_0_left_4:
	kernel_dpack_tn_4_vs_lib4(m, B+ii*ldb, ldb, pU, n-ii);
	for(jj=0; jj<m; jj+=4)
		{
		kernel_dtrmm_nn_ru_one_4x4_tran_vs_lib4c4c(jj, alpha, pU, A+jj*lda, lda, &d_0, pU+jj*ps, B+jj+ii*ldb, ldb, m-jj, n-ii);
		}
goto lutu_0_return;

lutu_0_return:
	return;



/************************************************
* rlnn
************************************************/
rlnn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>300 | n>300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto rutn_1;
		}
	else
		{
		goto rlnn_0;
		}

rlnn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_12x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_rl_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnn_0_left_4;
			}
		if(m-ii<=8)
			{
			goto rlnn_0_left_8;
			}
		else
			{
			goto rlnn_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_8x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_rl_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnn_0_left_4;
			}
		else
			{
			goto rlnn_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_4x4_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_rl_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rlnn_0_left_4;
		}
#endif
	goto rlnn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rlnn_0_left_12:
	kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rlnn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rlnn_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rlnn_0_return;
#endif

rlnn_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rlnn_0_return;

rlnn_0_return:
	return;



/************************************************
* rlnu
************************************************/
rlnu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>300 | n>300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto rutu_1;
		}
	else
		{
		goto rlnu_0;
		}

rlnu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_one_12x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_rl_one_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnu_0_left_4;
			}
		if(m-ii<=8)
			{
			goto rlnu_0_left_8;
			}
		else
			{
			goto rlnu_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_one_8x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_rl_one_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rlnu_0_left_4;
			}
		else
			{
			goto rlnu_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_rl_one_4x4_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_rl_one_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rlnu_0_left_4;
		}
#endif
	goto rlnu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rlnu_0_left_12:
	kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_one_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rlnu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rlnu_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_one_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rlnu_0_return;
#endif

rlnu_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_rl_one_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rlnu_0_return;

rlnu_0_return:
	return;



/************************************************
* rltn
************************************************/
rltn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto rltn_1;
		}
	else
		{
		goto rltn_0;
		}

rltn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_12x4_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_0_left_4;
			}
		if(m-ii<=8)
			{
			goto rltn_0_left_8;
			}
		else
			{
			goto rltn_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_8x4_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_0_left_4;
			}
		else
			{
			goto rltn_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x4_lib4ccc(jj, alpha, pU, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_4x4_vs_lib4ccc(jj, alpha, pU, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rltn_0_left_4;
		}
#endif
goto rltn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rltn_0_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rltn_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltn_0_return;
#endif

rltn_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x4_vs_lib4ccc(jj, alpha, pU, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltn_0_return;

rltn_0_return:
	return;



rltn_1:
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, n1);
	sB_size = blasfeo_memsize_dmat(n1, n1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, n, &sA, (void *) mem_align);
	blasfeo_create_dmat(n, n, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // upper to lower
		{
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tn_4_vs_lib4(n, A+ii*lda, lda, pB+ii*sdb, n-ii);
			}
		}
	else // lower to lower
		{
#if 1
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tt_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb, n-ii);
			}
#else
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(ii+4, A+ii, lda, pB+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_nn_4_vs_lib4(n, A+ii, lda, pB+ii*sdb, n-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_12x4_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_12x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_1_left_4;
			}
		if(m-ii<=8)
			{
			goto rltn_1_left_8;
			}
		else
			{
			goto rltn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_8x4_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_8x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltn_1_left_4;
			}
		else
			{
			goto rltn_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_4x4_lib44cc(jj, alpha, pU, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_4x4_vs_lib44cc(jj, alpha, pU, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rltn_1_left_4;
		}
#endif
goto rltn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rltn_1_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_12x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rltn_1_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_8x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltn_1_return;
#endif

rltn_1_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_4x4_vs_lib44cc(jj, alpha, pU, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltn_1_return;

rltn_1_return:
	free(mem);
	return;



/************************************************
* rltu
************************************************/
rltu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto rltu_1;
		}
	else
		{
		goto rltu_0;
		}

rltu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_12x4_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_one_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_0_left_4;
			}
		if(m-ii<=8)
			{
			goto rltu_0_left_8;
			}
		else
			{
			goto rltu_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_8x4_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_one_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_0_left_4;
			}
		else
			{
			goto rltu_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x4_lib4ccc(jj, alpha, pU, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_one_4x4_vs_lib4ccc(jj, alpha, pU, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rltu_0_left_4;
		}
#endif
goto rltu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rltu_0_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rltu_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltu_0_return;
#endif

rltu_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x4_vs_lib4ccc(jj, alpha, pU, A+jj, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltu_0_return;

rltu_0_return:
	return;



rltu_1:
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, n1);
	sB_size = blasfeo_memsize_dmat(n1, n1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, n, &sA, (void *) mem_align);
	blasfeo_create_dmat(n, n, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // upper to lower
		{
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(ii+4, A+ii*lda, lda, pB+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tn_4_vs_lib4(n, A+ii*lda, lda, pB+ii*sdb, n-ii);
			}
		}
	else // lower to lower
		{
#if 1
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tt_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*sdb+ii*ps, sdb, n-ii);
			}
#else
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(ii+4, A+ii, lda, pB+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_nn_4_vs_lib4(n, A+ii, lda, pB+ii*sdb, n-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_12x4_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_one_12x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_1_left_4;
			}
		if(m-ii<=8)
			{
			goto rltu_1_left_8;
			}
		else
			{
			goto rltu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_8x4_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_one_8x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rltu_1_left_4;
			}
		else
			{
			goto rltu_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_rl_one_4x4_lib44cc(jj, alpha, pU, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_rl_one_4x4_vs_lib44cc(jj, alpha, pU, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rltu_1_left_4;
		}
#endif
goto rltu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rltu_1_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_12x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rltu_1_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_8x4_vs_lib44cc(jj, alpha, pU, sdu, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltu_1_return;
#endif

rltu_1_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_rl_one_4x4_vs_lib44cc(jj, alpha, pU, pB+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rltu_1_return;

rltu_1_return:
	free(mem);
	return;



/************************************************
* runn
************************************************/
runn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto rltn_1;
		}
	else
		{
		goto runn_0;
		}

runn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_12x4_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_ru_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runn_0_left_4;
			}
		if(m-ii<=8)
			{
			goto runn_0_left_8;
			}
		else
			{
			goto runn_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_8x4_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_ru_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runn_0_left_4;
			}
		else
			{
			goto runn_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_4x4_lib4ccc(jj, alpha, pU, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_ru_4x4_vs_lib4ccc(jj, alpha, pU, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto runn_0_left_4;
		}
#endif
goto runn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
runn_0_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_ru_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto runn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
runn_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_ru_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto runn_0_return;
#endif

runn_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_ru_4x4_vs_lib4ccc(jj, alpha, pU, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto runn_0_return;

runn_0_return:
	return;



/************************************************
* runu
************************************************/
runu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=300 | n>=300 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 1;
		goto rltu_1;
		}
	else
		{
		goto runu_0;
		}

runu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_one_12x4_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_ru_one_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runu_0_left_4;
			}
		if(m-ii<=8)
			{
			goto runu_0_left_8;
			}
		else
			{
			goto runu_0_left_12;
			}
		}
#elif 0//defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_one_8x4_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_ru_one_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto runu_0_left_4;
			}
		else
			{
			goto runu_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nn_ru_one_4x4_lib4ccc(jj, alpha, pU, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nn_ru_one_4x4_vs_lib4ccc(jj, alpha, pU, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto runu_0_left_4;
		}
#endif
goto runu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
runu_0_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_ru_one_12x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto runu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
runu_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_ru_one_8x4_vs_lib4ccc(jj, alpha, pU, sdu, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto runu_0_return;
#endif

runu_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nn_ru_one_4x4_vs_lib4ccc(jj, alpha, pU, A+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto runu_0_return;

runu_0_return:
	return;



/************************************************
* rutn
************************************************/
rutn:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto rutn_1;
		}
	else
		{
		goto rutn_0;
		}

rutn_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_12x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_0_left_4;
			}
		if(m-ii<=8)
			{
			goto rutn_0_left_8;
			}
		else
			{
			goto rutn_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_8x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_0_left_4;
			}
		else
			{
			goto rutn_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x4_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rutn_0_left_4;
		}
#endif
goto rutn_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutn_0_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutn_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutn_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutn_0_return;
#endif

rutn_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutn_0_return;

rutn_0_return:
	return;



rutn_1:
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, n1);
	sB_size = blasfeo_memsize_dmat(n1, n1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, n, &sA, (void *) mem_align);
	blasfeo_create_dmat(n, n, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // lower to upper
		{
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, n-ii);
			}
		}
	else // upper to upper
		{
#if 1
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tt_4_vs_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb, n-ii);
			}
#else
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_nn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, n-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_12x4_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_12x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_1_left_4;
			}
		if(m-ii<=8)
			{
			goto rutn_1_left_8;
			}
		else
			{
			goto rutn_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_8x4_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_8x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutn_1_left_4;
			}
		else
			{
			goto rutn_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_4x4_lib44cc(n-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_4x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rutn_1_left_4;
		}
#endif
goto rutn_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutn_1_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_12x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutn_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutn_1_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_8x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutn_1_return;
#endif

rutn_1_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_4x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutn_1_return;

rutn_1_return:
	free(mem);
	return;



/************************************************
* rutu
************************************************/
rutu:
#if defined(TARGET_X64_INTEL_HASWELL)
	if(m>=200 | n>=200 | n>K_MAX_STACK)
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	if(m>=64 | n>=64 | n>K_MAX_STACK)
#else
	if(m>=12 | n>=12 | n>K_MAX_STACK)
#endif
		{
		pack_tran = 0;
		goto rutu_1;
		}
	else
		{
		goto rutu_0;
		}

rutu_0:
	pU = pU0;
	sdu = sdu0;

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_12x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_one_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_0_left_4;
			}
		if(m-ii<=8)
			{
			goto rutu_0_left_8;
			}
		else
			{
			goto rutu_0_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_8x4_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_one_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_0_left_4;
			}
		else
			{
			goto rutu_0_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x4_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_one_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rutu_0_left_4;
		}
#endif
goto rutu_0_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutu_0_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_12x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutu_0_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutu_0_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_8x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, sdu, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutu_0_return;
#endif

rutu_0_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x4_vs_lib4ccc(n-jj, alpha, pU+jj*ps, A+jj+jj*lda, lda, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutu_0_return;

rutu_0_return:
	return;



rutu_1:
	n1 = (n+128-1)/128*128;
	sA_size = blasfeo_memsize_dmat(12, n1);
	sB_size = blasfeo_memsize_dmat(n1, n1);
	mem = malloc(sA_size+sB_size+64);
	blasfeo_align_64_byte(mem, (void **) &mem_align);
	blasfeo_create_dmat(12, n, &sA, (void *) mem_align);
	blasfeo_create_dmat(n, n, &sB, (void *) (mem_align+sA_size));

	pU = sA.pA;
	sdu = sA.cn;
	pB = sB.pA;
	sdb = sB.cn;

	if(pack_tran) // lower to upper
		{
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, n-ii);
			}
		}
	else // upper to upper
		{
#if 1
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_tt_4_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb);
			}
		if(ii<n)
			{
			kernel_dpack_tt_4_vs_lib4(ii+4, A+ii*lda, lda, pB+ii*ps, sdb, n-ii);
			}
#else
		for(ii=0; ii<n-3; ii+=4)
			{
			kernel_dpack_nn_4_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb);
			}
		if(ii<n)
			{
			kernel_dpack_nn_4_vs_lib4(n-ii, A+ii+ii*lda, lda, pB+ii*ps+ii*sdb, n-ii);
			}
#endif
		}

	ii = 0;
#if defined(TARGET_X64_INTEL_HASWELL)
	for(; ii<m-11; ii+=12)
		{
		kernel_dpack_nn_12_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_12x4_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_one_12x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_1_left_4;
			}
		if(m-ii<=8)
			{
			goto rutu_1_left_8;
			}
		else
			{
			goto rutu_1_left_12;
			}
		}
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		kernel_dpack_nn_8_lib4(n, B+ii, ldb, pU, sdu);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_8x4_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_one_8x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		if(m-ii<=4)
			{
			goto rutu_1_left_4;
			}
		else
			{
			goto rutu_1_left_8;
			}
		}
#else
	for(; ii<m-3; ii+=4)
		{
		kernel_dpack_nn_4_lib4(n, B+ii, ldb, pU);
		for(jj=0; jj<n-3; jj+=4)
			{
			kernel_dtrmm_nt_ru_one_4x4_lib44cc(n-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb);
			}
		if(jj<n)
			{
			kernel_dtrmm_nt_ru_one_4x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
			}
		}
	if(ii<m)
		{
		goto rutu_1_left_4;
		}
#endif
goto rutu_1_return;

#if defined(TARGET_X64_INTEL_HASWELL)
rutu_1_left_12:
	kernel_dpack_nn_12_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_12x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutu_1_return;
#endif

#if defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE)
rutu_1_left_8:
	kernel_dpack_nn_8_vs_lib4(n, B+ii, ldb, pU, sdu, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_8x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, sdu, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutu_1_return;
#endif

rutu_1_left_4:
	kernel_dpack_nn_4_vs_lib4(n, B+ii, ldb, pU, m-ii);
	for(jj=0; jj<n; jj+=4)
		{
		kernel_dtrmm_nt_ru_one_4x4_vs_lib44cc(n-jj, alpha, pU+jj*ps, pB+jj*ps+jj*sdb, &d_0, B+ii+jj*ldb, ldb, B+ii+jj*ldb, ldb, m-ii, n-jj);
		}
goto rutu_1_return;

rutu_1_return:
	free(mem);
	return;

	}



