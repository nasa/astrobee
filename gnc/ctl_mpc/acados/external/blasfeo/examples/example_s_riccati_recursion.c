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



static void s_back_ric_sv_libstr(int N, int *nx, int *nu, struct blasfeo_smat *hsBAbt, struct blasfeo_smat *hsRSQrq, struct blasfeo_smat *hsL, struct blasfeo_svec *hsux, struct blasfeo_svec *hspi, struct blasfeo_smat *hswork_mat, struct blasfeo_svec *hswork_vec)
	{

	int nn;

	// factorization and backward substitution

	// last stage
	blasfeo_spotrf_l_mn(nx[N]+1, nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		blasfeo_strmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsBAbt[N-nn-1], 0, 0, &hswork_mat[0], 0, 0);
		blasfeo_sgead(1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn]+nx[N-nn], nu[N-nn], &hswork_mat[0], nu[N-nn-1]+nx[N-nn-1], 0);
#if 1
		blasfeo_ssyrk_spotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#else
		blasfeo_ssyrk_ln(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
		blasfeo_spotrf_l(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#endif
		}

	// forward substitution

	// first stage
	nn = 0;
	blasfeo_srowex(nu[nn]+nx[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
	blasfeo_strsv_ltn(nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_srowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
	blasfeo_sgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
	blasfeo_sveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
	blasfeo_srowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec[0], 0);
	blasfeo_strmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);
	blasfeo_saxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);
	blasfeo_strmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		blasfeo_srowex(nu[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
		blasfeo_strsv_ltn_mn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_srowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_sgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
		blasfeo_sveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
		blasfeo_srowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec[0], 0);
		blasfeo_strmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);
		blasfeo_saxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);
		blasfeo_strmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn], 0, &hspi[nn], 0);
		}

	return;

	}



static void s_back_ric_trf_libstr(int N, int *nx, int *nu, struct blasfeo_smat *hsBAbt, struct blasfeo_smat *hsRSQrq, struct blasfeo_smat *hsL, struct blasfeo_smat *hswork_mat)
	{

	int nn;

	// factorization

	// last stage
	blasfeo_spotrf_l(nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		blasfeo_strmm_rlnn(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsBAbt[N-nn-1], 0, 0, &hswork_mat[0], 0, 0);
#if 1
		blasfeo_ssyrk_spotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#else
		blasfeo_ssyrk_ln(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
		blasfeo_spotrf_l(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#endif
		}
		return;

	}



static void s_back_ric_trs_libstr(int N, int *nx, int *nu, struct blasfeo_smat *hsBAbt, struct blasfeo_svec *hsb, struct blasfeo_svec *hsrq, struct blasfeo_smat *hsL, struct blasfeo_svec *hsPb, struct blasfeo_svec *hsux, struct blasfeo_svec *hspi, struct blasfeo_svec *hswork_vec)
	{

	int nn;

	// backward substitution

	// last stage
	blasfeo_sveccp(nu[N]+nx[N], &hsrq[N], 0, &hsux[N], 0);

	// middle stages
	for(nn=0; nn<N-1; nn++)
		{
		// compute Pb
		blasfeo_strmv_ltn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, &hsPb[N-nn-1], 0);
		blasfeo_strmv_lnn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsPb[N-nn-1], 0, &hsPb[N-nn-1], 0);
		blasfeo_sveccp(nu[N-nn-1]+nx[N-nn-1], &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
		blasfeo_sveccp(nx[N-nn], &hsPb[N-nn-1], 0, &hswork_vec[0], 0);
		blasfeo_saxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_sgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec[0], 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		blasfeo_strsv_lnn_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		}

	// first stage
	nn = N-1;
	blasfeo_strmv_ltn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, &hsPb[N-nn-1], 0);
	blasfeo_strmv_lnn(nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsPb[N-nn-1], 0, &hsPb[N-nn-1], 0);
	blasfeo_sveccp(nu[N-nn-1]+nx[N-nn-1], &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
	blasfeo_sveccp(nx[N-nn], &hsPb[N-nn-1], 0, &hswork_vec[0], 0);
	blasfeo_saxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_sgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec[0], 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
	blasfeo_strsv_lnn(nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);

	// forward substitution

	// first stage
	nn = 0;
	blasfeo_sveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
	blasfeo_svecsc(nu[nn]+nx[nn], -1.0, &hsux[nn], 0);
	blasfeo_strsv_ltn(nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_sgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
	blasfeo_sveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hswork_vec[0], 0);
	blasfeo_strmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_strmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_saxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		blasfeo_sveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
		blasfeo_svecsc(nu[nn], -1.0, &hsux[nn], 0);
		blasfeo_strsv_ltn_mn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_sgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_sveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hswork_vec[0], 0);
		blasfeo_strmv_ltn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_strmv_lnn(nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_saxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0, &hspi[nn], 0);
		}

	return;

	}



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

	// loop index
	int ii;

/************************************************
* problem size
************************************************/

	// problem size
	int N = 4;
	int nx_ = 4;
	int nu_ = 1;

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

	double *Ad; d_zeros(&Ad, nx_, nx_); // states update matrix

	double *Bd; d_zeros(&Bd, nx_, nu_); // inputs matrix

	double *bd; d_zeros(&bd, nx_, 1); // states offset
	double *x0d; d_zeros(&x0d, nx_, 1); // initial state

	double Ts = 0.5; // sampling time
	d_mass_spring_system(Ts, nx_, nu_, N, Ad, Bd, bd, x0d);

	float *A; s_zeros(&A, nx_, nx_); for(ii=0; ii<nx_*nx_; ii++) A[ii] = (float) Ad[ii];
	float *B; s_zeros(&B, nx_, nu_); for(ii=0; ii<nx_*nu_; ii++) B[ii] = (float) Bd[ii];
	float *b; s_zeros(&b, nx_, 1); for(ii=0; ii<nx_; ii++) b[ii] = (float) bd[ii];
	float *x0; s_zeros(&x0, nx_, 1); for(ii=0; ii<nx_; ii++) x0[ii] = (float) x0d[ii];

	for(ii=0; ii<nx_; ii++)
		b[ii] = 0.1;

	for(ii=0; ii<nx_; ii++)
		x0[ii] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

	s_print_mat(nx_, nx_, A, nx_);
	s_print_mat(nx_, nu_, B, nx_);
	s_print_mat(1, nx_, b, 1);
	s_print_mat(1, nx_, x0, 1);

/************************************************
* cost function
************************************************/

	float *R; s_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 2.0;

	float *S; s_zeros(&S, nu_, nx_);

	float *Q; s_zeros(&Q, nx_, nx_);
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 1.0;

	float *r; s_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.2;

	float *q; s_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.1;

	s_print_mat(nu_, nu_, R, nu_);
	s_print_mat(nu_, nx_, S, nu_);
	s_print_mat(nx_, nx_, Q, nx_);
	s_print_mat(1, nu_, r, 1);
	s_print_mat(1, nx_, q, 1);

/************************************************
* matrices as strmat
************************************************/

	struct blasfeo_smat sA;
	blasfeo_allocate_smat(nx_, nx_, &sA);
	blasfeo_pack_smat(nx_, nx_, A, nx_, &sA, 0, 0);
	struct blasfeo_svec sb;
	blasfeo_allocate_svec(nx_, &sb);
	blasfeo_pack_svec(nx_, b, 1, &sb, 0);
	struct blasfeo_svec sx0;
	blasfeo_allocate_svec(nx_, &sx0);
	blasfeo_pack_svec(nx_, x0, 1, &sx0, 0);
	struct blasfeo_svec sb0;
	blasfeo_allocate_svec(nx_, &sb0);
	float *b0; s_zeros(&b0, nx_, 1); // states offset
	blasfeo_sgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb, 0, &sb0, 0);
	blasfeo_print_tran_svec(nx_, &sb0, 0);

	struct blasfeo_smat sBbt0;
	blasfeo_allocate_smat(nu_+nx_+1, nx_, &sBbt0);
	blasfeo_pack_tran_smat(nx_, nx_, B, nx_, &sBbt0, 0, 0);
	blasfeo_srowin(nx_, 1.0, &sb0, 0, &sBbt0, nu_, 0);
	blasfeo_print_smat(nu_+1, nx_, &sBbt0, 0, 0);

	struct blasfeo_smat sBAbt1;
	blasfeo_allocate_smat(nu_+nx_+1, nx_, &sBAbt1);
	blasfeo_pack_tran_smat(nx_, nu_, B, nx_, &sBAbt1, 0, 0);
	blasfeo_pack_tran_smat(nx_, nx_, A, nx_, &sBAbt1, nu_, 0);
	blasfeo_pack_tran_smat(nx_, 1, b, nx_, &sBAbt1, nu_+nx_, 0);
	blasfeo_print_smat(nu_+nx_+1, nx_, &sBAbt1, 0, 0);

	struct blasfeo_svec sr0; // XXX no need to update r0 since S=0
	blasfeo_allocate_svec(nu_, &sr0);
	blasfeo_pack_svec(nu_, r, 1, &sr0, 0);

	struct blasfeo_smat sRr0;
	blasfeo_allocate_smat(nu_+1, nu_, &sRr0);
	blasfeo_pack_smat(nu_, nu_, R, nu_, &sRr0, 0, 0);
	blasfeo_srowin(nu_, 1.0, &sr0, 0, &sRr0, nu_, 0);
	blasfeo_print_smat(nu_+1, nu_, &sRr0, 0, 0);

	struct blasfeo_svec srq1;
	blasfeo_allocate_svec(nu_+nx_, &srq1);
	blasfeo_pack_svec(nu_, r, 1, &srq1, 0);
	blasfeo_pack_svec(nx_, q, 1, &srq1, nu_);

	struct blasfeo_smat sRSQrq1;
	blasfeo_allocate_smat(nu_+nx_+1, nu_+nx_, &sRSQrq1);
	blasfeo_pack_smat(nu_, nu_, R, nu_, &sRSQrq1, 0, 0);
	blasfeo_pack_tran_smat(nu_, nx_, S, nu_, &sRSQrq1, nu_, 0);
	blasfeo_pack_smat(nx_, nx_, Q, nx_, &sRSQrq1, nu_, nu_);
	blasfeo_srowin(nu_+nx_, 1.0, &srq1, 0, &sRSQrq1, nu_+nx_, 0);
	blasfeo_print_smat(nu_+nx_+1, nu_+nx_, &sRSQrq1, 0, 0);

	struct blasfeo_svec sqN;
	blasfeo_allocate_svec(nx_, &sqN);
	blasfeo_pack_svec(nx_, q, 1, &sqN, 0);

	struct blasfeo_smat sQqN;
	blasfeo_allocate_smat(nx_+1, nx_, &sQqN);
	blasfeo_pack_smat(nx_, nx_, Q, nx_, &sQqN, 0, 0);
	blasfeo_srowin(nx_, 1.0, &sqN, 0, &sQqN, nx_, 0);
	blasfeo_print_smat(nx_+1, nx_, &sQqN, 0, 0);

/************************************************
* array of matrices
************************************************/

	struct blasfeo_smat *hsBAbt = malloc(N*sizeof(struct blasfeo_smat));
	struct blasfeo_svec *hsb = malloc(N*sizeof(struct blasfeo_svec));
	struct blasfeo_smat *hsRSQrq = malloc((N+1)*sizeof(struct blasfeo_smat));
	struct blasfeo_svec *hsrq = malloc((N+1)*sizeof(struct blasfeo_svec));
	struct blasfeo_smat *hsL = malloc((N+1)*sizeof(struct blasfeo_smat));
	struct blasfeo_svec *hsPb = malloc(N*sizeof(struct blasfeo_svec));
	struct blasfeo_svec *hsux = malloc((N+1)*sizeof(struct blasfeo_svec));
	struct blasfeo_svec *hspi = malloc(N*sizeof(struct blasfeo_svec));
	struct blasfeo_smat *hswork_mat = malloc(1*sizeof(struct blasfeo_smat));
	struct blasfeo_svec *hswork_vec = malloc(1*sizeof(struct blasfeo_svec));

	hsBAbt[0] = sBbt0;
	hsb[0] = sb0;
	hsRSQrq[0] = sRr0;
	hsrq[0] = sr0;
	blasfeo_allocate_smat(nu_+1, nu_, &hsL[0]);
	blasfeo_allocate_svec(nx_, &hsPb[0]);
	blasfeo_allocate_svec(nx_+nu_+1, &hsux[0]);
	blasfeo_allocate_svec(nx_, &hspi[0]);
	for(ii=1; ii<N; ii++)
		{
		hsBAbt[ii] = sBAbt1;
		hsb[ii] = sb;
		hsRSQrq[ii] = sRSQrq1;
		hsrq[ii] = srq1;
		blasfeo_allocate_smat(nu_+nx_+1, nu_+nx_, &hsL[ii]);
		blasfeo_allocate_svec(nx_, &hsPb[ii]);
		blasfeo_allocate_svec(nx_+nu_+1, &hsux[ii]);
		blasfeo_allocate_svec(nx_, &hspi[ii]);
		}
	hsRSQrq[N] = sQqN;
	hsrq[N] = sqN;
	blasfeo_allocate_smat(nx_+1, nx_, &hsL[N]);
	blasfeo_allocate_svec(nx_+nu_+1, &hsux[N]);
	blasfeo_allocate_smat(nu_+nx_+1, nx_, &hswork_mat[0]);
	blasfeo_allocate_svec(nx_, &hswork_vec[0]);

//	for(ii=0; ii<N; ii++)
//		blasfeo_print_dmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
//	return 0;

/************************************************
* call Riccati solver
************************************************/

	// timing
	blasfeo_timer timer;
	int nrep = 1000;
	int rep;

	double time_sv, time_trf, time_trs;

	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		s_back_ric_sv_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hsux, hspi, hswork_mat, hswork_vec);
		}

	time_sv = blasfeo_toc(&timer) / nrep;
	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		s_back_ric_trf_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hswork_mat);
		}

	time_trf = blasfeo_toc(&timer) / nrep;
	blasfeo_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		s_back_ric_trs_libstr(N, nx, nu, hsBAbt, hsb, hsrq, hsL, hsPb, hsux, hspi, hswork_vec);
		}

	time_trs = blasfeo_toc(&timer) / nrep;

	// print sol
	printf("\nux = \n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(nu[ii]+nx[ii], &hsux[ii], 0);

	printf("\npi = \n\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_tran_svec(nx[ii+1], &hspi[ii], 0);

//	printf("\nL = \n\n");
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_smat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], 0, 0);

	printf("\ntime sv\t\ttime trf\t\ttime trs\n");
	printf("\n%e\t%e\t%e\n", time_sv, time_trf, time_trs);
	printf("\n");

/************************************************
* free memory
************************************************/

	d_free(Ad);
	d_free(Bd);
	d_free(bd);
	d_free(x0d);
	s_free(A);
	s_free(B);
	s_free(b);
	s_free(x0);
	s_free(R);
	s_free(S);
	s_free(Q);
	s_free(r);
	s_free(q);
	s_free(b0);
	blasfeo_free_smat(&sA);
	blasfeo_free_svec(&sb);
	blasfeo_free_smat(&sBbt0);
	blasfeo_free_svec(&sb0);
	blasfeo_free_smat(&sBAbt1);
	blasfeo_free_smat(&sRr0);
	blasfeo_free_svec(&sr0);
	blasfeo_free_smat(&sRSQrq1);
	blasfeo_free_svec(&srq1);
	blasfeo_free_smat(&sQqN);
	blasfeo_free_svec(&sqN);
	blasfeo_free_smat(&hsL[0]);
	blasfeo_free_svec(&hsPb[0]);
	blasfeo_free_svec(&hsux[0]);
	blasfeo_free_svec(&hspi[0]);
	for(ii=1; ii<N; ii++)
		{
		blasfeo_free_smat(&hsL[ii]);
		blasfeo_free_svec(&hsPb[ii]);
		blasfeo_free_svec(&hsux[ii]);
		blasfeo_free_svec(&hspi[ii]);
		}
	blasfeo_free_smat(&hsL[N]);
	blasfeo_free_svec(&hsux[N]);
	blasfeo_free_smat(&hswork_mat[0]);
	blasfeo_free_svec(&hswork_vec[0]);

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


/************************************************
* return
************************************************/

	return 0;

	}




