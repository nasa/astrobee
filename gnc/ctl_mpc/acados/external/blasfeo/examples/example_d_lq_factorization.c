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

#include <blasfeo.h>


int main()
	{

	printf("\nExample of LQ factorization\n\n");

#if defined(LA_HIGH_PERFORMANCE)

	printf("\nLA provided by BLASFEO\n\n");

#elif defined(LA_REFERENCE)

	printf("\nLA provided by REFERENCE\n\n");

#elif defined(LA_EXTERNAL_BLAS_WRAPPER)

	printf("\nLA provided by EXTERNAL_BLAS_WRAPPER\n\n");

#else

	printf("\nLA provided by ???\n\n");
	exit(2);

#endif

	printf( "Testing processor\n" );

	char supportString[50];
	blasfeo_processor_library_string( supportString );
	printf( "Library requires processor features:%s\n", supportString );

	int features = 0;
	int procCheckSucceed = blasfeo_processor_cpu_features( &features );
	blasfeo_processor_feature_string( features, supportString );
	printf( "Processor supports features:%s\n", supportString );

	if( !procCheckSucceed )
	{
		printf("Current processor does not support the current compiled BLASFEO library.\n");
		printf("Please get a BLASFEO library compatible with this processor.\n");
		exit(3);
	}


	int ii;

#if 1
	int m = 13;
	int n = 12;

	/* matrices in column-major format */

	double *A; d_zeros(&A, n, m);
	for(ii=0; ii<n; ii++)
		{
		A[ii+n*ii] = 1.0;
		A[ii+n*(ii/2)] = 1.0;
		A[(ii/3)+n*ii] = 1.0;
		A[ii+n*(m-1)] = 1.0;
		}
#else
	int n = 3;
	int m = 3;
	double *A; d_zeros(&A, n, m);
	A[0+0*n] = 1;
	A[1+0*n] = 2;
	A[2+0*n] = 1;
	A[0+1*n] = 1;
	A[1+1*n] = 2;
	A[2+1*n] = 2;
	A[0+2*n] = 2;
	A[1+2*n] = 4;
	A[2+2*n] = 3;
#endif

#if 0
	A[0+n*0] = 1.0;
	A[0+n*2] = 1.0;
	A[0+n*5] = 1.0;
	A[1+n*1] = 1.0;
	A[1+n*3] = 1.0;
	A[1+n*5] = 1.0;
	A[2+n*2] = 1.0;
	A[2+n*5] = 1.0;
	A[3+n*3] = 1.0;
	A[3+n*5] = 1.0;
	A[4+n*4] = 1.0;
	A[4+n*5] = 1.0;
#endif

	printf("\nA = \n");
	d_print_mat(n, m, A, n);


	/* matrices in blasfeo matrix struct format */

	struct blasfeo_dmat sA;
	int sA_size = blasfeo_memsize_dmat(n, m);
	void *sA_mem; v_zeros_align(&sA_mem, sA_size);
	blasfeo_create_dmat(n, m, &sA, sA_mem);
	blasfeo_pack_dmat(n, m, A, n, &sA, 0, 0);

	printf("\nsA = \n");
	blasfeo_print_dmat(n, m, &sA, 0, 0);


	/* LQ factorization */

	struct blasfeo_dmat sA_fact;
	void *sA_fact_mem; v_zeros_align(&sA_fact_mem, sA_size);
	blasfeo_create_dmat(n, m, &sA_fact, sA_fact_mem);

	int lq_size = blasfeo_dgelqf_worksize(n, m);
	void *lq_work = malloc(lq_size);

	blasfeo_dgelqf(n, m, &sA, 0, 0, &sA_fact, 0, 0, lq_work);

	printf("\nLQ fact of sA = \n");
	blasfeo_print_dmat(n, m, &sA_fact, 0, 0);
	d_print_mat(1, n, sA_fact.dA, 1);

	/* extract L */

	struct blasfeo_dmat sL;
	int sL_size = blasfeo_memsize_dmat(n, n);
	void *sL_mem; v_zeros_align(&sL_mem, sL_size);
	blasfeo_create_dmat(n, n, &sL, sL_mem);

	blasfeo_dtrcp_l(n, &sA_fact, 0, 0, &sL, 0, 0);

	printf("\nL = \n");
	blasfeo_print_dmat(n, n, &sL, 0, 0);

	/* compute Q */

	struct blasfeo_dmat sQ;
	int sQ_size = blasfeo_memsize_dmat(m, m);
	void *sQ_mem; v_zeros_align(&sQ_mem, sQ_size);
	blasfeo_create_dmat(m, m, &sQ, sQ_mem);

#if 1
	int orglq_size = blasfeo_dorglq_worksize(m, m, n);
	void *orglq_work = malloc(orglq_size);

	blasfeo_dorglq(m, m, n, &sA_fact, 0, 0, &sQ, 0, 0, orglq_work);

	free(orglq_work);
#else
	for(ii=0; ii<m; ii++)
		BLASFEO_DMATEL(&sQ, ii, ii) = 1.0;
	blasfeo_print_dmat(m, m, &sQ, 0, 0);

	double *pT; d_zeros_align(&pT, 4, 4);

	// last 1
	kernel_dlarft_1_lib4(m-4, sA.pA+1*sA.cn*4+1*4*4, sA.dA+4, pT);
	d_print_mat(4, 4, pT, 4);
	kernel_dlarfb1_rt_1_lib4(m-4, sA.pA+1*sA.cn*4+1*4*4, pT, sQ.pA+1*sQ.cn*4+1*4*4+0);
	kernel_dlarfb1_rt_1_lib4(m-4, sA.pA+1*sA.cn*4+1*4*4, pT, sQ.pA+1*sQ.cn*4+1*4*4+1);

	// first 4
	kernel_dlarft_4_lib4(m, sA.pA, sA.dA, pT);
	d_print_mat(4, 4, pT, 4);
	kernel_dlarfb4_rt_4_lib4(m, sA.pA+0, pT, sQ.pA+0*sQ.cn*4+0);
	kernel_dlarfb4_rt_1_lib4(m, sA.pA+0, pT, sQ.pA+1*sQ.cn*4+0);
	kernel_dlarfb4_rt_1_lib4(m, sA.pA+0, pT, sQ.pA+1*sQ.cn*4+1);

	d_free_align(pT);

//	kernel_dlarfb3_rt_1_lib4(m, sA.pA, pT, Q+0+4*0);
//	kernel_dlarfb3_rt_1_lib4(m, sA.pA, pT, Q+1+4*0);
//	kernel_dlarfb3_rt_1_lib4(m, sA.pA, pT, Q+2+4*0);
//	kernel_dlarfb3_r_4_lib4(m, sA.pA, pT, Q+0+4*0);
//	kernel_dlarfb4_r_4_lib4(m, sA.pA, pT, Q+0+4*0);
//	kernel_dlarfb4_rt_4_lib4(m, sA.pA, pT, Q+0+4*0);
//	kernel_dlarfb4_rt_4_lib4(m, sA.pA, pT, Q+0+4*0);
//	d_print_mat(4, 4, Q, 4);
#endif

	printf("\nQ = \n");
	blasfeo_print_dmat(m, m, &sQ, 0, 0);

	/* compute A-LQ */

	struct blasfeo_dmat sE;
	int sE_size = blasfeo_memsize_dmat(n, m);
	void *sE_mem; v_zeros_align(&sE_mem, sE_size);
	blasfeo_create_dmat(n, m, &sE, sE_mem);

	blasfeo_dgemm_nn(n, m, n, -1.0, &sL, 0, 0, &sQ, 0, 0, 1.0, &sA, 0, 0, &sE, 0, 0);

	printf("\nA - L * Q = \n");
	blasfeo_print_dmat(n, m, &sE, 0, 0);

	/* free memory */

	d_free(A);
	v_free_align(sA_mem);
	v_free_align(sA_fact_mem);
	v_free_align(sL_mem);
	v_free_align(sQ_mem);
	v_free_align(sE_mem);
	free(lq_work);

	return 0;

	}

