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

#include "tools.h"

#include <blasfeo.h>



#ifdef EXTERNAL_BLAS_MKL
#include <mkl.h>
#endif
#ifdef EXTERNAL_BLAS_OPENBLAS
#include <d_blas.h>
#endif



#if ( defined(EXTERNAL_BLAS_SYSTEM) | defined(EXTERNAL_BLAS_MKL) | defined(EXTERNAL_BLAS_OPENBLAS) | defined(EXTERNAL_BLAS_NETLIB) | defined(EXTERNAL_BLAS_BLIS) | defined(EXTERNAL_BLAS_ATLAS) )
#define EXTERNAL_BLAS 1
#else
#define EXTERNAL_BLAS 0
#endif



//#define PRINT_DATA



static void d_back_ric_sv_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsL, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hspi, struct blasfeo_dmat *hswork_mat, struct blasfeo_dvec *hswork_vec)
	{

	int nn;

	// factorization and backward substitution

	// last stage
	blasfeo_dpotrf_l_mn(nx[N]+1, nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsBAbt[N-nn-1], 0, 0, &hswork_mat[0], 0, 0);
		blasfeo_dgead(1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn]+nx[N-nn], nu[N-nn], &hswork_mat[0], nu[N-nn-1]+nx[N-nn-1], 0);
#if 1
		blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#else
		blasfeo_dsyrk_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
		blasfeo_dpotrf_l_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#endif
		}

	// forward substitution

	// first stage
	nn = 0;
	blasfeo_drowex(nu[nn]+nx[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
	blasfeo_dtrsv_ltn(nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_drowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
	blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
	blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
	blasfeo_drowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec[0], 0);
	blasfeo_dtrmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);
	blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);
	blasfeo_dtrmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		blasfeo_drowex(nu[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
		blasfeo_dtrsv_ltn_mn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_drowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
		blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
		blasfeo_drowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec[0], 0);
		blasfeo_dtrmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);
		blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);
		blasfeo_dtrmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);
		}

	return;

	}



static void d_back_ric_trf_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsL, struct blasfeo_dmat *hswork_mat)
	{

	int nn;

	// factorization

	// last stage
	blasfeo_dpotrf_l(nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsBAbt[N-nn-1], 0, 0, &hswork_mat[0], 0, 0);
#if 1
		blasfeo_dsyrk_dpotrf_ln(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#else
		blasfeo_dsyrk_ln(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
		blasfeo_dpotrf_l(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#endif
//		blasfeo_print_dmat(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0);
		}

	return;

	}



#if ( EXTERNAL_BLAS!=0 | (defined(BLAS_API) & defined(FORTRAN_BLAS_API)) )
static void d_back_ric_trf(int N, int *nx, int *nu, double **hBAbt, double **hRSQrq, double **hL, double **hwork_mat)
	{

	char c_l = 'l';
	char c_n = 'n';
	char c_r = 'r';

	double d_1 = 1.0;

	int m, n;
	int inc = 1;
	int lda, ldb;
	int info;

	int ii, nn;

	// factorization

	// last stage
	m = nx[N];
	lda = nu[N]+nx[N]+1;
	ldb = nu[N]+nx[N];
	for(ii=0; ii<nx[N]; ii++)
		dcopy_(&m, hRSQrq[N]+ii*lda, &inc, hL[N]+ii*ldb, &inc);
	dpotrf_(&c_l, &m, hL[N], &ldb, &info);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		m = nu[N-nn-1]+nx[N-nn-1];
		lda = nu[N-nn-1]+nx[N-nn-1]+1;
		ldb = nu[N-nn-1]+nx[N-nn-1];
		for(ii=0; ii<nx[N-nn]; ii++)
			dcopy_(&m, hBAbt[N-nn-1]+ii*lda, &inc, hwork_mat[0]+ii*ldb, &inc);
		lda = nu[N-nn]+nx[N-nn];
		ldb = nu[N-nn-1]+nx[N-nn-1];
		m = nu[N-nn-1]+nx[N-nn-1];
		n = nx[N-nn];
		dtrmm_(&c_r, &c_l, &c_n, &c_n, &m, &n, &d_1, hL[N-nn]+nu[N-nn]*(lda+1), &lda, hwork_mat[0], &ldb);
		m = nu[N-nn-1]+nx[N-nn-1];
		lda = nu[N-nn-1]+nx[N-nn-1]+1;
		ldb = nu[N-nn-1]+nx[N-nn-1];
		for(ii=0; ii<nx[N-nn-1]+nu[N-nn-1]; ii++)
			dcopy_(&m, hRSQrq[N-nn-1]+ii*lda, &inc, hL[N-nn-1]+ii*ldb, &inc);
		m = nu[N-nn-1]+nx[N-nn-1];
		n = nx[N-nn];
		lda = nu[N-nn-1]+nx[N-nn-1];
		ldb = nu[N-nn-1]+nx[N-nn-1];
		dsyrk_(&c_l, &c_n, &m, &n, &d_1, hwork_mat[0], &lda, &d_1, hL[N-nn-1], &ldb);
		dpotrf_(&c_l, &m, hL[N-nn-1], &ldb, &info);
		}

	return;
	}
#endif



static void d_back_ric_trs_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsL, struct blasfeo_dvec *hsPb, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hswork_vec)
	{
//	printf("\nblasfeo api\n");

	int nn;

	// backward substitution

	// last stage
	blasfeo_dveccp(nu[N]+nx[N], &hsrq[N], 0, &hsux[N], 0);

	// middle stages
	for(nn=0; nn<N-1; nn++)
		{
		// compute Pb
		blasfeo_dtrmv_ltn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, &hsPb[N-nn-1], 0);
		blasfeo_dtrmv_lnn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsPb[N-nn-1], 0, &hsPb[N-nn-1], 0);
		blasfeo_dveccp(nu[N-nn-1]+nx[N-nn-1], &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
		blasfeo_dveccp(nx[N-nn], &hsPb[N-nn-1], 0, &hswork_vec[0], 0);
		blasfeo_daxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec[0], 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		blasfeo_dtrsv_lnn_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		}

	// first stage
	nn = N-1;
	blasfeo_dtrmv_ltn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, &hsPb[N-nn-1], 0);
	blasfeo_dtrmv_lnn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsPb[N-nn-1], 0, &hsPb[N-nn-1], 0);
	blasfeo_dveccp(nu[N-nn-1]+nx[N-nn-1], &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
	blasfeo_dveccp(nx[N-nn], &hsPb[N-nn-1], 0, &hswork_vec[0], 0);
	blasfeo_daxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec[0], 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
	blasfeo_dtrsv_lnn(nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);


	// forward substitution

	// first stage
	nn = 0;
	blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
	blasfeo_dvecsc(nu[nn]+nx[nn], -1.0, &hsux[nn], 0);
	blasfeo_dtrsv_ltn(nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
	blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hswork_vec[0], 0);
	blasfeo_dtrmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_dtrmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
		blasfeo_dvecsc(nu[nn], -1.0, &hsux[nn], 0);
		blasfeo_dtrsv_ltn_mn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hswork_vec[0], 0);
		blasfeo_dtrmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_dtrmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);

		}

	return;

	}



#if ( EXTERNAL_BLAS!=0 | (defined(BLAS_API) & defined(FORTRAN_BLAS_API)) )
static void d_back_ric_trs(int N, int *nx, int *nu, double **hBAbt, double **hb, double **hrq, double **hL, double **hPb, double **hux, double **hpi, double **hwork_vec)
	{
//	printf("\nblas api\n");

	int m, n, lda;
	int inc = 1;

	char c_l = 'l';
	char c_n = 'n';
	char c_t = 't';

	double d_1 = 1.0;
	double d_m1 = -1.0;

	int nn;

	// backward substitution

	// last stage
	m = nu[N]+nx[N];
	dcopy_(&m, hrq[N], &inc, hux[N], &inc);

	// middle stages
	for(nn=0; nn<N-1; nn++)
		{
		m = nx[N-nn];
		dcopy_(&m, hb[N-nn-1], &inc, hPb[N-nn-1], &inc);
		lda = nu[N-nn]+nx[N-nn];
		dtrmv_(&c_l, &c_t, &c_n, &m, hL[N-nn]+nu[N-nn]*(lda+1), &lda, hPb[N-nn-1], &inc);
		dtrmv_(&c_l, &c_n, &c_n, &m, hL[N-nn]+nu[N-nn]*(lda+1), &lda, hPb[N-nn-1], &inc);
		m = nu[N-nn-1]+nx[N-nn-1];
		dcopy_(&m, hrq[N-nn-1], &inc, hux[N-nn-1], &inc);
		m = nx[N-nn];
		dcopy_(&m, hPb[N-nn-1], &inc, hwork_vec[0], &inc);
		daxpy_(&m, &d_1, hux[N-nn]+nu[N-nn], &inc, hwork_vec[0], &inc);
		m = nu[N-nn-1]+nx[N-nn-1];
		n = nx[N-nn];
		lda = nu[N-nn-1]+nx[N-nn-1]+1;
		dgemv_(&c_n, &m, &n, &d_1, hBAbt[N-nn-1], &lda, hwork_vec[0], &inc, &d_1, hux[N-nn-1], &inc);
		m = nu[N-nn-1];
		lda = nu[N-nn-1]+nx[N-nn-1];
		dtrsv_(&c_l, &c_n, &c_n, &m, hL[N-nn-1], &lda, hux[N-nn-1], &inc);
		m = nx[N-nn-1];
		n = nu[N-nn-1];
		dgemv_(&c_n, &m, &n, &d_m1, hL[N-nn-1]+nu[N-nn-1], &lda, hux[N-nn-1], &inc, &d_1, hux[N-nn-1]+nu[N-nn-1], &inc);
		}

	// first stage
	nn = N-1;
	m = nx[N-nn];
	dcopy_(&m, hb[N-nn-1], &inc, hPb[N-nn-1], &inc);
	lda = nu[N-nn]+nx[N-nn];
	dtrmv_(&c_l, &c_t, &c_n, &m, hL[N-nn]+nu[N-nn]*(lda+1), &lda, hPb[N-nn-1], &inc);
	dtrmv_(&c_l, &c_n, &c_n, &m, hL[N-nn]+nu[N-nn]*(lda+1), &lda, hPb[N-nn-1], &inc);
	m = nu[N-nn-1]+nx[N-nn-1];
	dcopy_(&m, hrq[N-nn-1], &inc, hux[N-nn-1], &inc);
	m = nx[N-nn];
	dcopy_(&m, hPb[N-nn-1], &inc, hwork_vec[0], &inc);
	daxpy_(&m, &d_1, hux[N-nn]+nu[N-nn], &inc, hwork_vec[0], &inc);
	m = nu[N-nn-1]+nx[N-nn-1];
	n = nx[N-nn];
	lda = nu[N-nn-1]+nx[N-nn-1]+1;
	dgemv_(&c_n, &m, &n, &d_1, hBAbt[N-nn-1], &lda, hwork_vec[0], &inc, &d_1, hux[N-nn-1], &inc);
	m = nu[N-nn-1];
	lda = nu[N-nn-1]+nx[N-nn-1];
	dtrsv_(&c_l, &c_n, &c_n, &m, hL[N-nn-1], &lda, hux[N-nn-1], &inc);


	// forward substitution

	// first stage
	nn = 0;
	m = nx[nn+1];
	dcopy_(&m, hux[nn+1]+nu[nn+1], &inc, hpi[nn], &inc);
	m = nu[nn]+nx[nn];
	dscal_(&m, &d_m1, hux[nn], &inc);
	lda = nu[nn]+nx[nn];
	dtrsv_(&c_l, &c_t, &c_n, &m, hL[nn], &lda, hux[nn], &inc);
	m = nu[nn]+nx[nn];
	n = nx[nn+1];
	lda = nu[nn]+nx[nn]+1;
	dcopy_(&n, hb[nn], &inc, hux[nn+1]+nu[nn+1], &inc);
	dgemv_(&c_t, &m, &n, &d_1, hBAbt[nn], &lda, hux[nn], &inc, &d_1, hux[nn+1]+nu[nn+1], &inc);
	dcopy_(&n, hux[nn+1]+nu[nn+1], &inc, hwork_vec[0], &inc);
	lda = nu[nn+1]+nx[nn+1];
	dtrmv_(&c_l, &c_t, &c_n, &n, hL[nn+1]+nu[nn+1]*(lda+1), &lda, hwork_vec[0], &inc);
	dtrmv_(&c_l, &c_n, &c_n, &n, hL[nn+1]+nu[nn+1]*(lda+1), &lda, hwork_vec[0], &inc);
	daxpy_(&n, &d_1, hwork_vec[0], &inc, hpi[nn], &inc);

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		m = nx[nn+1];
		dcopy_(&m, hux[nn+1]+nu[nn+1], &inc, hpi[nn], &inc);
		m = nu[nn];
		dscal_(&m, &d_m1, hux[nn], &inc);
		lda = nu[nn]+nx[nn];
		m = nx[nn];
		n = nu[nn];
		dgemv_(&c_t, &m, &n, &d_m1, hL[nn]+nu[nn], &lda, hux[nn]+nu[nn], &inc, &d_1, hux[nn], &inc);
		dtrsv_(&c_l, &c_t, &c_n, &n, hL[nn], &lda, hux[nn], &inc);
		m = nu[nn]+nx[nn];
		n = nx[nn+1];
		lda = nu[nn]+nx[nn]+1;
		dcopy_(&n, hb[nn], &inc, hux[nn+1]+nu[nn+1], &inc);
		dgemv_(&c_t, &m, &n, &d_1, hBAbt[nn], &lda, hux[nn], &inc, &d_1, hux[nn+1]+nu[nn+1], &inc);
		dcopy_(&n, hux[nn+1]+nu[nn+1], &inc, hwork_vec[0], &inc);
		lda = nu[nn+1]+nx[nn+1];
		dtrmv_(&c_l, &c_t, &c_n, &n, hL[nn+1]+nu[nn+1]*(lda+1), &lda, hwork_vec[0], &inc);
		dtrmv_(&c_l, &c_n, &c_n, &n, hL[nn+1]+nu[nn+1]*(lda+1), &lda, hwork_vec[0], &inc);
		daxpy_(&n, &d_1, hwork_vec[0], &inc, hpi[nn], &inc);
		}

	return;

	}
#endif



/************************************************
Mass-spring system: nx/2 masses connected each other with springs (in a row), and the first and the last one to walls. nu (<=nx) controls act on the first nu masses. The system is sampled with sampling time Ts.
************************************************/
static void d_mass_spring_system(double Ts, int nx, int nu, int N, double *A, double *B, double *b, double *x0)
	{

	int nx2 = nx*nx;

	int info = 0;

	int pp = nx/2; // number of masses

/************************************************
* build the continuous time system
************************************************/

	double *T; d_zeros(&T, pp, pp);
	int ii;
	for(ii=0; ii<pp; ii++) T[ii*(pp+1)] = -2;
	for(ii=0; ii<pp-1; ii++) T[ii*(pp+1)+1] = 1;
	for(ii=1; ii<pp; ii++) T[ii*(pp+1)-1] = 1;

	double *Z; d_zeros(&Z, pp, pp);
	double *I; d_zeros(&I, pp, pp); for(ii=0; ii<pp; ii++) I[ii*(pp+1)]=1.0; // = eye(pp);
	double *Ac; d_zeros(&Ac, nx, nx);
	dmcopy(pp, pp, Z, pp, Ac, nx);
	dmcopy(pp, pp, T, pp, Ac+pp, nx);
	dmcopy(pp, pp, I, pp, Ac+pp*nx, nx);
	dmcopy(pp, pp, Z, pp, Ac+pp*(nx+1), nx);
	free(T);
	free(Z);
	free(I);

	d_zeros(&I, nu, nu); for(ii=0; ii<nu; ii++) I[ii*(nu+1)]=1.0; //I = eye(nu);
	double *Bc; d_zeros(&Bc, nx, nu);
	dmcopy(nu, nu, I, nu, Bc+pp, nx);
	free(I);

/************************************************
* compute the discrete time system
************************************************/

	double *bb; d_zeros(&bb, nx, 1);
	dmcopy(nx, 1, bb, nx, b, nx);

	dmcopy(nx, nx, Ac, nx, A, nx);
	dscal_3l(nx2, Ts, A);
	expm(nx, A);

	d_zeros(&T, nx, nx);
	d_zeros(&I, nx, nx); for(ii=0; ii<nx; ii++) I[ii*(nx+1)]=1.0; //I = eye(nx);
	dmcopy(nx, nx, A, nx, T, nx);
	daxpy_3l(nx2, -1.0, I, T);
	dgemm_nn_3l(nx, nu, nx, T, nx, Bc, nx, B, nx);
	free(T);
	free(I);

	int *ipiv = (int *) malloc(nx*sizeof(int));
	dgesv_3l(nx, nu, Ac, nx, ipiv, B, nx, &info);
	free(ipiv);

	free(Ac);
	free(Bc);
	free(bb);


/************************************************
* initial state
************************************************/

	if(nx==4)
		{
		x0[0] = 5;
		x0[1] = 10;
		x0[2] = 15;
		x0[3] = 20;
		}
	else
		{
		int jj;
		for(jj=0; jj<nx; jj++)
			x0[jj] = 1;
		}

	}



int main()
	{

	printf("\nExample of Riccati recursion factorization and backsolve\n\n");

	// check Linear Algebra backend
#if defined(LA_HIGH_PERFORMANCE)

	printf("\nLA provided by HIGH_PERFORMANCE\n");

#elif defined(LA_REFERENCE)

	printf("\nLA provided by REFERENCE\n");

#elif defined(LA_EXTERNAL_BLAS_WRAPPER)

	printf("\nLA provided by EXTERNAL_BLAS_WRAPPER\n");

#else

	printf("\nLA provided by ???\n");
	exit(2);

#endif

	// check Matrix Format
#if defined(MF_PANELMAJ)

	printf("\nMF provided by PANELMAJ\n");

#elif defined(MF_COLMAJ)

	printf("\nMF provided by COLMAJ\n");

#else

	printf("\nMF provided by ???\n");
	exit(2);

#endif

	// check processor
	printf( "\nTesting processor\n" );

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

	// loop index
	int ii, jj;

/************************************************
* problem size
************************************************/

	// problem size
	int N = 10;
	int nx_ = 8;
	int nu_ = 3;

	// stage-wise variant size
	int *nx = malloc((N+1)*sizeof(int));
	nx[0] = 0;
	for(ii=1; ii<=N; ii++)
		nx[ii] = nx_;
	nx[N] = nx_;

	int *nu = malloc((N+1)*sizeof(int));
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_;
	nu[N] = 0;

/************************************************
* dynamical system
************************************************/

	double *A; d_zeros(&A, nx_, nx_); // states update matrix

	double *B; d_zeros(&B, nx_, nu_); // inputs matrix

	double *b; d_zeros(&b, nx_, 1); // states offset
	double *x0; d_zeros(&x0, nx_, 1); // initial state

	double Ts = 0.5; // sampling time
	d_mass_spring_system(Ts, nx_, nu_, N, A, B, b, x0);

	for(ii=0; ii<nx_; ii++)
		b[ii] = 0.1;

	for(ii=0; ii<nx_; ii++)
		x0[ii] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

#ifdef PRINT_DATA
	printf("A:\n");
	d_print_exp_mat(nx_, nx_, A, nx_);
	printf("B:\n");
	d_print_exp_mat(nx_, nu_, B, nx_);
	printf("b:\n");
	d_print_exp_mat(1, nx_, b, 1);
	printf("x0:\n");
	d_print_exp_mat(1, nx_, x0, 1);
#endif

/************************************************
* cost function
************************************************/

	double *R; d_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 2.0;

	double *S; d_zeros(&S, nu_, nx_);

	double *Q; d_zeros(&Q, nx_, nx_);
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 1.0;

	double *r; d_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.2;

	double *q; d_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.1;

#ifdef PRINT_DATA
	printf("R:\n");
	d_print_exp_mat(nu_, nu_, R, nu_);
	printf("S:\n");
	d_print_exp_mat(nu_, nx_, S, nu_);
	printf("Q:\n");
	d_print_exp_mat(nx_, nx_, Q, nx_);
	printf("r:\n");
	d_print_exp_mat(1, nu_, r, 1);
	printf("q:\n");
	d_print_exp_mat(1, nx_, q, 1);
#endif

/************************************************
* BLASFEO API
************************************************/

/************************************************
* matrices as strmat
************************************************/

	struct blasfeo_dmat sA;
	blasfeo_allocate_dmat(nx_, nx_, &sA);
	blasfeo_pack_dmat(nx_, nx_, A, nx_, &sA, 0, 0);
	struct blasfeo_dvec sb;
	blasfeo_allocate_dvec(nx_, &sb);
	blasfeo_pack_dvec(nx_, b, 1, &sb, 0);
	struct blasfeo_dvec sx0;
	blasfeo_allocate_dvec(nx_, &sx0);
	blasfeo_pack_dvec(nx_, x0, 1, &sx0, 0);
	struct blasfeo_dvec sb0;
	blasfeo_allocate_dvec(nx_, &sb0);
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb, 0, &sb0, 0);
#ifdef PRINT_DATA
	printf("b0:\n");
	blasfeo_print_tran_dvec(nx_, &sb0, 0);
#endif

	struct blasfeo_dmat sBbt0;
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &sBbt0);
	blasfeo_pack_tran_dmat(nx_, nu_, B, nx_, &sBbt0, 0, 0);
	blasfeo_drowin(nx_, 1.0, &sb0, 0, &sBbt0, nu_, 0);
#ifdef PRINT_DATA
	printf("Bbt0:\n");
	blasfeo_print_exp_dmat(nu_+1, nx_, &sBbt0, 0, 0);
#endif

	struct blasfeo_dmat sBAbt1;
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &sBAbt1);
	blasfeo_pack_tran_dmat(nx_, nu_, B, nx_, &sBAbt1, 0, 0);
	blasfeo_pack_tran_dmat(nx_, nx_, A, nx_, &sBAbt1, nu_, 0);
	blasfeo_pack_tran_dmat(nx_, 1, b, nx_, &sBAbt1, nu_+nx_, 0);
#ifdef PRINT_DATA
	printf("BAbt1:\n");
	blasfeo_print_exp_dmat(nu_+nx_+1, nx_, &sBAbt1, 0, 0);
#endif

	struct blasfeo_dvec sr0; // XXX no need to update r0 since S=0
	blasfeo_allocate_dvec(nu_, &sr0);
	blasfeo_pack_dvec(nu_, r, 1, &sr0, 0);

	struct blasfeo_dmat sRr0;
	blasfeo_allocate_dmat(nu_+1, nu_, &sRr0);
	blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRr0, 0, 0);
	blasfeo_drowin(nu_, 1.0, &sr0, 0, &sRr0, nu_, 0);
#ifdef PRINT_DATA
	printf("sRr0:\n");
	blasfeo_print_exp_dmat(nu_+1, nu_, &sRr0, 0, 0);
#endif

	struct blasfeo_dvec srq1;
	blasfeo_allocate_dvec(nu_+nx_, &srq1);
	blasfeo_pack_dvec(nu_, r, 1, &srq1, 0);
	blasfeo_pack_dvec(nx_, q, 1, &srq1, nu_);

	struct blasfeo_dmat sRSQrq1;
	blasfeo_allocate_dmat(nu_+nx_+1, nu_+nx_, &sRSQrq1);
	blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRSQrq1, 0, 0);
	blasfeo_pack_tran_dmat(nu_, nx_, S, nu_, &sRSQrq1, nu_, 0);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sRSQrq1, nu_, nu_);
	blasfeo_drowin(nu_+nx_, 1.0, &srq1, 0, &sRSQrq1, nu_+nx_, 0);
#ifdef PRINT_DATA
	printf("RSQrq1:\n");
	blasfeo_print_exp_dmat(nu_+nx_+1, nu_+nx_, &sRSQrq1, 0, 0);
#endif

	struct blasfeo_dvec sqN;
	blasfeo_allocate_dvec(nx_, &sqN);
	blasfeo_pack_dvec(nx_, q, 1, &sqN, 0);

	struct blasfeo_dmat sQqN;
	blasfeo_allocate_dmat(nx_+1, nx_, &sQqN);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sQqN, 0, 0);
	blasfeo_drowin(nx_, 1.0, &sqN, 0, &sQqN, nx_, 0);
#ifdef PRINT_DATA
	printf("QqN:\n");
	blasfeo_print_exp_dmat(nx_+1, nx_, &sQqN, 0, 0);
#endif

/************************************************
* array of matrices
************************************************/

	struct blasfeo_dmat *hsBAbt = malloc(N*sizeof(struct blasfeo_dmat));
	struct blasfeo_dvec *hsb = malloc(N*sizeof(struct blasfeo_dvec));
	struct blasfeo_dmat *hsRSQrq = malloc((N+1)*sizeof(struct blasfeo_dmat));
	struct blasfeo_dvec *hsrq = malloc((N+1)*sizeof(struct blasfeo_dvec));
	struct blasfeo_dmat *hsL = malloc((N+1)*sizeof(struct blasfeo_dmat));
	struct blasfeo_dvec *hsPb = malloc(N*sizeof(struct blasfeo_dvec));
	struct blasfeo_dvec *hsux = malloc((N+1)*sizeof(struct blasfeo_dvec));
	struct blasfeo_dvec *hspi = malloc(N*sizeof(struct blasfeo_dvec));
	struct blasfeo_dmat *hswork_mat = malloc(1*sizeof(struct blasfeo_dmat));
	struct blasfeo_dvec *hswork_vec = malloc(1*sizeof(struct blasfeo_dvec));

	hsBAbt[0] = sBbt0;
	hsb[0] = sb0;
	hsRSQrq[0] = sRr0;
	hsrq[0] = sr0;
	blasfeo_allocate_dmat(nu_+1, nu_, &hsL[0]);
	blasfeo_allocate_dvec(nx_, &hsPb[0]);
	blasfeo_allocate_dvec(nx_+nu_+1, &hsux[0]);
	blasfeo_allocate_dvec(nx_, &hspi[0]);
	for(ii=1; ii<N; ii++)
		{
		hsBAbt[ii] = sBAbt1;
		hsb[ii] = sb;
		hsRSQrq[ii] = sRSQrq1;
		hsrq[ii] = srq1;
		blasfeo_allocate_dmat(nu_+nx_+1, nu_+nx_, &hsL[ii]);
		blasfeo_allocate_dvec(nx_, &hsPb[ii]);
		blasfeo_allocate_dvec(nx_+nu_+1, &hsux[ii]);
		blasfeo_allocate_dvec(nx_, &hspi[ii]);
		}
	hsRSQrq[N] = sQqN;
	hsrq[N] = sqN;
	blasfeo_allocate_dmat(nx_+1, nx_, &hsL[N]);
	blasfeo_allocate_dvec(nx_+nu_+1, &hsux[N]);
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &hswork_mat[0]);
	blasfeo_allocate_dvec(nx_, &hswork_vec[0]);

//	for(ii=0; ii<N; ii++)
//		blasfeo_print_exp_dmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
//	return 0;

/************************************************
* BLAS API
************************************************/

#if ( EXTERNAL_BLAS!=0 | (defined(BLAS_API) & defined(FORTRAN_BLAS_API)) )

#ifdef PRINT_DATA
	printf("\n*** BLAS_API ***\n\n");
#endif

//	int incx, incy;

	double *Bbt0 = malloc((nu_+1)*nx_*sizeof(double));
	blasfeo_unpack_dmat(nu_+1, nx_, &sBbt0, 0, 0, Bbt0, nu_+1);
#ifdef PRINT_DATA
	printf("Bbt0:\n");
	d_print_exp_mat(nu_+1, nx_, Bbt0, nu_+1);
#endif

	double *b0 = malloc(nx_*sizeof(double));
	blasfeo_unpack_dvec(nx_, &sb0, 0, b0, 1);
#ifdef PRINT_DATA
	printf("b0:\n");
	d_print_exp_mat(1, nx_, b0, 1);
#endif

	double *BAbt1 = malloc((nu_+nx_+1)*nx_*sizeof(double));
	blasfeo_unpack_dmat(nu_+nx_+1, nx_, &sBAbt1, 0, 0, BAbt1, nu_+nx_+1);
#ifdef PRINT_DATA
	printf("BAbt1:\n");
	d_print_exp_mat(nu_+nx_+1, nx_, BAbt1, nu_+nx_+1);
#endif

	double *b1 = malloc(nx_*sizeof(double));
	blasfeo_unpack_dvec(nx_, &sb, 0, b1, 1);
#ifdef PRINT_DATA
	printf("b1:\n");
	d_print_exp_mat(1, nx_, b1, 1);
#endif

	double *Rr0 = malloc((nu_+1)*nu_*sizeof(double));
	blasfeo_unpack_dmat(nu_+1, nu_, &sRr0, 0, 0, Rr0, nu_+1);
#ifdef PRINT_DATA
	printf("Rr0:\n");
	d_print_exp_mat(nu_+1, nu_, Rr0, nu_+1);
#endif

	double *r0 = malloc(nu_*sizeof(double));
	blasfeo_unpack_dvec(nu_, &sr0, 0, r0, 1);
#ifdef PRINT_DATA
	printf("r0:\n");
	d_print_exp_mat(1, nu_, r0, 1);
#endif

	double *RSQrq1 = malloc((nu_+nx_+1)*(nu_+nx_)*sizeof(double));
	blasfeo_unpack_dmat(nu_+nx_+1, nu_+nx_, &sRSQrq1, 0, 0, RSQrq1, nu_+nx_+1);
#ifdef PRINT_DATA
	printf("RSQrq1:\n");
	d_print_exp_mat(nu_+nx_+1, nu_+nx_, RSQrq1, nu_+nx_+1);
#endif

	double *rq1 = malloc((nu_+nx_)*sizeof(double));
	blasfeo_unpack_dvec(nu_+nx_, &srq1, 0, rq1, 1);
#ifdef PRINT_DATA
	printf("rq1:\n");
	d_print_exp_mat(1, nu_+nx_, rq1, 1);
#endif

	double *QqN = malloc((nx_+1)*nx_*sizeof(double));
	blasfeo_unpack_dmat(nx_+1, nx_, &sQqN, 0, 0, QqN, nx_+1);
#ifdef PRINT_DATA
	printf("RSQrq1:\n");
	d_print_exp_mat(nx_+1, nx_, QqN, nx_+1);
#endif

	double *qN = malloc((nx_)*sizeof(double));
	blasfeo_unpack_dvec(nx_, &sqN, 0, qN, 1);
#ifdef PRINT_DATA
	printf("qN:\n");
	d_print_exp_mat(1, nx_, qN, 1);
#endif



	double **hBAbt = malloc(N*sizeof(double *));
	double **hb = malloc(N*sizeof(double *));
	double **hRSQrq = malloc((N+1)*sizeof(double *));
	double **hrq = malloc((N+1)*sizeof(double *));
	double **hL = malloc((N+1)*sizeof(double *));
	double **hPb = malloc(N*sizeof(double *));
	double **hux = malloc((N+1)*sizeof(double *));
	double **hpi = malloc(N*sizeof(double *));
	double **hwork_mat = malloc(1*sizeof(double *));
	double **hwork_vec = malloc(1*sizeof(double *));

	hBAbt[0] = Bbt0;
	hb[0] = b0;
	hRSQrq[0] = Rr0;
	hrq[0] = r0;
	hL[0] = malloc((nu_+1)*(nu_)*sizeof(double));
	hPb[0] = malloc(nx_*sizeof(double));
	hux[0] = malloc((nu_)*sizeof(double));
	hpi[0] = malloc(nx_*sizeof(double));
	hwork_mat[0] = malloc((nu_+nx_+1)*(nx_)*sizeof(double));
	hwork_vec[0] = malloc(nx_*sizeof(double));
	for(ii=1; ii<N; ii++)
		{
		hBAbt[ii] = BAbt1;
		hb[ii] = b1;
		hRSQrq[ii] = RSQrq1;
		hrq[ii] = rq1;
		hL[ii] = malloc((nu_+nx_+1)*(nu_+nx_)*sizeof(double));
		hPb[ii] = malloc(nx_*sizeof(double));
		hux[ii] = malloc((nu_+nx_)*sizeof(double));
		hpi[ii] = malloc(nx_*sizeof(double));
		}
	ii = N;
	hRSQrq[ii] = QqN;
	hrq[ii] = qN;
	hL[ii] = malloc((nx_+1)*(nx_)*sizeof(double));
	hux[ii] = malloc((nx_)*sizeof(double));

//	return 0;

#endif

/************************************************
* call Riccati solver
************************************************/

	// timing
	blasfeo_timer timer;
	int nrep = 1000;
	int rep;

	double
		time_sv=0.0, time_trf=0.0, time_trs=0.0,
		time_trf_blas_api=0.0, time_trs_blas_api=0.0;

	/* BLASFEO API */

	// factorization + solution
	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_sv_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hsux, hspi, hswork_mat, hswork_vec);
		}

	time_sv = blasfeo_toc(&timer) / nrep;

	// factorization
	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_trf_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hswork_mat);
		}

	time_trf = blasfeo_toc(&timer) / nrep;

	// solution
	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_trs_libstr(N, nx, nu, hsBAbt, hsb, hsrq, hsL, hsPb, hsux, hspi, hswork_vec);
		}

	time_trs = blasfeo_toc(&timer) / nrep;

	/* BLAS API */
#if ( EXTERNAL_BLAS!=0 | (defined(BLAS_API) & defined(FORTRAN_BLAS_API)) )

	// factorization
	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_trf(N, nx, nu, hBAbt, hRSQrq, hL, hwork_mat);
		}

	time_trf_blas_api = blasfeo_toc(&timer) / nrep;

	// solution
	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_trs(N, nx, nu, hBAbt, hb, hrq, hL, hPb, hux, hpi, hwork_vec);
		}

	time_trs_blas_api = blasfeo_toc(&timer) / nrep;

#endif

	// print sol
	printf("\nBLASFEO API\n\n");
	printf("\nux = \n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);

	printf("\npi = \n\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_tran_dvec(nx[ii+1], &hspi[ii], 0);

//	printf("\nL = \n\n");
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_exp_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], 0, 0);

#if ( EXTERNAL_BLAS!=0 | (defined(BLAS_API) & defined(FORTRAN_BLAS_API)) )
	printf("\nBLAS API\n\n");
	printf("\nux = \n\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu[ii]+nx[ii], hux[ii], 1);

	printf("\npi = \n\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx[ii+1], hpi[ii], 1);
#endif

	printf("\n           \ttime sv\t\ttime trf\ttime trs\n");
	printf("\nBLASFEO API\t%e\t%e\t%e\n", time_sv, time_trf, time_trs);
	printf("\nBLAS API   \t%e\t%e\t%e\n", 0.0, time_trf_blas_api, time_trs_blas_api);
	printf("\n");

/************************************************
* free memory
************************************************/

	d_free(A);
	d_free(B);
	d_free(b);
	d_free(x0);
	d_free(R);
	d_free(S);
	d_free(Q);
	d_free(r);
	d_free(q);
	blasfeo_free_dmat(&sA);
	blasfeo_free_dvec(&sb);
	blasfeo_free_dvec(&sx0);
	blasfeo_free_dmat(&sBbt0);
	blasfeo_free_dvec(&sb0);
	blasfeo_free_dmat(&sBAbt1);
	blasfeo_free_dmat(&sRr0);
	blasfeo_free_dvec(&sr0);
	blasfeo_free_dmat(&sRSQrq1);
	blasfeo_free_dvec(&srq1);
	blasfeo_free_dmat(&sQqN);
	blasfeo_free_dvec(&sqN);
	blasfeo_free_dmat(&hsL[0]);
	blasfeo_free_dvec(&hsPb[0]);
	blasfeo_free_dvec(&hsux[0]);
	blasfeo_free_dvec(&hspi[0]);
	for(ii=1; ii<N; ii++)
		{
		blasfeo_free_dmat(&hsL[ii]);
		blasfeo_free_dvec(&hsPb[ii]);
		blasfeo_free_dvec(&hsux[ii]);
		blasfeo_free_dvec(&hspi[ii]);
		}
	blasfeo_free_dmat(&hsL[N]);
	blasfeo_free_dvec(&hsux[N]);
	blasfeo_free_dmat(&hswork_mat[0]);
	blasfeo_free_dvec(&hswork_vec[0]);

	free(nx);
	free(nu);
	free(hsBAbt);
	free(hsb);
	free(hsRSQrq);
	free(hsrq);
	free(hsL);
	free(hsPb);
	free(hsux);
	free(hspi);
	free(hswork_mat);
	free(hswork_vec);

#if (defined(BLAS_API) & defined(FORTRAN_BLAS_API))
	free(Bbt0);
	free(b0);
	free(BAbt1);
	free(b1);
	free(Rr0);
	free(r0);
	free(RSQrq1);
	free(rq1);
	free(QqN);
	free(qN);

	free(hwork_mat[0]);
	free(hwork_vec[0]);
	for(ii=0; ii<N; ii++)
		{
		free(hL[ii]);
		free(hPb[ii]);
		free(hux[ii]);
		free(hpi[ii]);
		}
	ii = N;
	free(hL[ii]);
	free(hux[ii]);

	free(hBAbt);
	free(hb);
	free(hRSQrq);
	free(hrq);
	free(hL);
	free(hPb);
	free(hux);
	free(hpi);
	free(hwork_mat);
	free(hwork_vec);
#endif


/************************************************
* return
************************************************/

	return 0;

	}
