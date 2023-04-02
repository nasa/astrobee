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

#include "../include/blasfeo_target.h"
#include "../include/blasfeo_common.h"
#include "../include/blasfeo_timing.h"
#include "../include/blasfeo_d_aux.h"
#include "../include/blasfeo_d_aux_ext_dep.h"
#include "../include/blasfeo_d_kernel.h"
#include "../include/blasfeo_d_blas.h"

#include "../include/d_blas.h"



int main()
	{

#if !defined(BLAS_API)
	printf("\nRecompile with BLAS_API=1 to run this benchmark!\n\n");
	exit(1);
#endif

	int n = 32;

	int ii;


	/* colunm-major matrices */

	double *A = malloc(n*n*sizeof(double));
	for(ii=0; ii<n*n; ii++)
		A[ii] = ii;
	int lda = n;
//	d_print_mat(n, n, A, n);

	double *B = malloc(n*n*sizeof(double));
	for(ii=0; ii<n*n; ii++)
		B[ii] = -0.0;
	for(ii=0; ii<n; ii++)
		B[ii*(n+1)] = 1.0;
	int ldb = n;
//	d_print_mat(n, n, B, ldb);

	double *C = malloc(n*n*sizeof(double));
	for(ii=0; ii<n*n; ii++)
		C[ii] = -1;
	int ldc = n;
//	d_print_mat(n, n, C, ldc);

	double *D = malloc(n*n*sizeof(double));
	for(ii=0; ii<n*n; ii++)
		D[ii] = -1;
	int ldd = n;
//	d_print_mat(n, n, C, ldc);

	int *ipiv = malloc(n*sizeof(int));


	/* panel-major matrices */

	struct blasfeo_dmat sA;
	blasfeo_allocate_dmat(n, n, &sA);
	blasfeo_pack_dmat(n, n, A, n, &sA, 0, 0);

	struct blasfeo_dmat sB;
	blasfeo_allocate_dmat(n, n, &sB);
	blasfeo_pack_dmat(n, n, B, n, &sB, 0, 0);

	struct blasfeo_dmat sC;
	blasfeo_allocate_dmat(n, n, &sC);
	blasfeo_pack_dmat(n, n, C, n, &sC, 0, 0);

	struct blasfeo_dmat sD;
	blasfeo_allocate_dmat(n, n, &sD);
	blasfeo_pack_dmat(n, n, D, n, &sD, 0, 0);



//	int bs = 4;

	double d_0 = 0.0;
	double d_1 = 1.0;

	char c_l = 'l';
	char c_n = 'n';
	char c_r = 'r';
	char c_t = 't';
	char c_u = 'u';

	double alpha = 2.0;
	double beta = 1.0;

	char ta = 'n';
	char tb = 't';
	char uplo = 'u';
	int info = 0;

	int m0, n0, k0;



	/* BLAS API */

	// dgemm
	m0 = 3; n0 = 3; k0 = 3;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	m0 = 6; n0 = 6; k0 = 6;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	m0 = 9; n0 = 9; k0 = 9;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	m0 = 12; n0 = 12; k0 = 12;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	m0 = 15; n0 = 15; k0 = 15;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	m0 = 18; n0 = 18; k0 = 18;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	m0 = 21; n0 = 21; k0 = 21;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	m0 = 24; n0 = 24; k0 = 24;
	blasfeo_dgemm(&c_n, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_n, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_n, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);
	blasfeo_dgemm(&c_t, &c_t, &m0, &n0, &k0, &alpha, A, &n, B, &n, &beta, C, &n);

	// dsyrk
	m0 = 3; n0 = 3; k0 = 3;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	m0 = 6; n0 = 6; k0 = 6;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	m0 = 9; n0 = 9; k0 = 9;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	m0 = 12; n0 = 12; k0 = 12;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	m0 = 15; n0 = 15; k0 = 15;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	m0 = 18; n0 = 18; k0 = 18;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	m0 = 21; n0 = 21; k0 = 21;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	m0 = 24; n0 = 24; k0 = 24;
	blasfeo_dsyrk(&c_l, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_l, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_n, &m0, &k0, &alpha, A, &n, &beta, C, &n);
	blasfeo_dsyrk(&c_u, &c_t, &m0, &k0, &alpha, A, &n, &beta, C, &n);

	// dtrmm
	m0 = 3; n0 = 3;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 6; n0 = 6;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 9; n0 = 9;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 12; n0 = 12;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 15; n0 = 15;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 18; n0 = 18;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 21; n0 = 21;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 24; n0 = 24;
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrmm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);

	// dtrsm
	m0 = 3; n0 = 3;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 6; n0 = 6;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 9; n0 = 9;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 12; n0 = 12;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 15; n0 = 15;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 18; n0 = 18;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 21; n0 = 21;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	m0 = 24; n0 = 24;
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_l, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_l, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_n, &c_u, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_n, &m0, &n0, &alpha, A, &n, C, &n);
	blasfeo_dtrsm(&c_r, &c_u, &c_t, &c_u, &m0, &n0, &alpha, A, &n, C, &n);

	// dgetrf
	m0 = 3; n0 = 3;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);
	m0 = 6; n0 = 6;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);
	m0 = 9; n0 = 9;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);
	m0 = 12; n0 = 12;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);
	m0 = 15; n0 = 15;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);
	m0 = 18; n0 = 18;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);
	m0 = 21; n0 = 21;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);
	m0 = 24; n0 = 24;
	blasfeo_dgetrf(&m0, &n0, B, &n, ipiv, &info);

	// dgetrs
	m0 = 3; n0 = 3;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);
	m0 = 6; n0 = 6;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);
	m0 = 9; n0 = 9;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);
	m0 = 12; n0 = 12;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);
	m0 = 15; n0 = 15;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);
	m0 = 18; n0 = 18;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);
	m0 = 21; n0 = 21;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);
	m0 = 24; n0 = 24;
	blasfeo_dgetrs(&c_n, &m0, &n0, B, &n, ipiv, D, &n, &info);
	blasfeo_dgetrs(&c_t, &m0, &n0, B, &n, ipiv, D, &n, &info);

	// dpotrf
	m0 = 3;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);
	m0 = 6;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);
	m0 = 9;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);
	m0 = 12;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);
	m0 = 15;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);
	m0 = 18;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);
	m0 = 21;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);
	m0 = 24;
	blasfeo_dpotrf(&c_l, &m0, B, &n, &info);
	blasfeo_dpotrf(&c_u, &m0, B, &n, &info);

	// dgetrs
	m0 = 3; n0 = 3;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);
	m0 = 6; n0 = 6;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);
	m0 = 9; n0 = 9;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);
	m0 = 12; n0 = 12;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);
	m0 = 15; n0 = 15;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);
	m0 = 18; n0 = 18;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);
	m0 = 21; n0 = 21;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);
	m0 = 24; n0 = 24;
	blasfeo_dpotrs(&c_l, &m0, &n0, B, &n, D, &n, &info);
	blasfeo_dpotrs(&c_u, &m0, &n0, B, &n, D, &n, &info);



	/* BLAS API */

	// dgemm
	m0 = 3; n0 = 3; k0 = 3;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 6; n0 = 6; k0 = 6;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 9; n0 = 9; k0 = 9;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 12; n0 = 12; k0 = 12;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 15; n0 = 15; k0 = 15;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 18; n0 = 18; k0 = 18;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 21; n0 = 21; k0 = 21;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 24; n0 = 24; k0 = 24;
	blasfeo_dgemm_nn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_nt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dgemm_tt(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);

	// dsyrk
	m0 = 3; n0 = 3; k0 = 3;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 6; n0 = 6; k0 = 6;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 9; n0 = 9; k0 = 9;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 12; n0 = 12; k0 = 12;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 15; n0 = 15; k0 = 15;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 18; n0 = 18; k0 = 18;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 21; n0 = 21; k0 = 21;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	m0 = 24; n0 = 24; k0 = 24;
	blasfeo_dsyrk_ln(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln_mn(m0, n0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_lt(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_un(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ut(m0, k0, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sD, 0, 0);

	// dgetrf
	m0 = 3; n0 = 3;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);
	m0 = 6; n0 = 6;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);
	m0 = 9; n0 = 9;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);
	m0 = 12; n0 = 12;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);
	m0 = 15; n0 = 15;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);
	m0 = 18; n0 = 18;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);
	m0 = 21; n0 = 21;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);
	m0 = 24; n0 = 24;
	blasfeo_dgetrf_np(m0, n0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(m0, n0, &sB, 0, 0, &sD, 0, 0, ipiv);

	// dpotrf
	m0 = 3; n0 = 3;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);
	m0 = 6; n0 = 6;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);
	m0 = 9; n0 = 9;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);
	m0 = 12; n0 = 12;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);
	m0 = 15; n0 = 15;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);
	m0 = 18; n0 = 18;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);
	m0 = 21; n0 = 21;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);
	m0 = 24; n0 = 24;
	blasfeo_dpotrf_l(m0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dpotrf_l_mn(m0, n0, &sB, 0, 0, &sD, 0, 0);



	/* free memory */

	blasfeo_free_dmat(&sA);
	blasfeo_free_dmat(&sB);
	blasfeo_free_dmat(&sC);
	blasfeo_free_dmat(&sD);

	free(A);
	free(B);
	free(C);
	free(D);
	free(ipiv);


	// return
	printf("\nsuccess!\n\n");

	return 0;

	}

