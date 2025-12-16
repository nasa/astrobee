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
#include <sys/time.h>

#include "tools.h"

#include <blasfeo.h>



void d_back_ric_sv_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsL, struct blasfeo_dmat *hsLxt, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hspi, struct blasfeo_dmat *hswork_mat, struct blasfeo_dvec *hswork_vec)
	{

	int nn;

	// factorization and backward substitution

	// last stage
	blasfeo_dpotrf_l(nx[N]+1, nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);
	blasfeo_dtrtr_l(nx[N], &hsL[N], 0, 0, &hsLxt[N], 0, 0);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		blasfeo_dtrmm_rutn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hsLxt[N-nn], 0, 0, 0.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0);
		blasfeo_dgead(1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn]+nx[N-nn], nu[N-nn], &hswork_mat[0], nu[N-nn-1]+nx[N-nn-1], 0);
#if 1
		blasfeo_dsyrk_dpotrf_ln(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#else
		blasfeo_dsyrk_ln(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
		blasfeo_dpotrf_l(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
#endif
		blasfeo_dtrtr_l(nx[N-nn-1], &hsL[N-nn-1], nu[N-nn-1], nu[N-nn-1], &hsLxt[N-nn-1], 0, 0);
		}

	// forward substitution

	// first stage
	nn = 0;
	blasfeo_drowex(nu[nn]+nx[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
	blasfeo_dtrsv_ltn(nu[nn]+nx[nn], nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_drowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
	blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
	blasfeo_dveccp(nx[nn+1], 1.0, &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
	blasfeo_drowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec[0], 0);
	blasfeo_dtrmv_unn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hspi[nn], 0, &hspi[nn], 0);
	blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0);
	blasfeo_dtrmv_utn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hspi[nn], 0, &hspi[nn], 0);

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		blasfeo_drowex(nu[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
		blasfeo_dtrsv_ltn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_drowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
		blasfeo_dveccp(nx[nn+1], 1.0, &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
		blasfeo_drowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec[0], 0);
		blasfeo_dtrmv_unn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hspi[nn], 0, &hspi[nn], 0);
		blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0);
		blasfeo_dtrmv_utn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hspi[nn], 0, &hspi[nn], 0);
		}

	return;

	}



void d_back_ric_trf_funnel1_libstr(int md, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsL, struct blasfeo_dmat *hsLxt_old, struct blasfeo_dmat *hsLxt_new, struct blasfeo_dmat *hswork_mat)
	{

	int ii;

	ii = 0;
	blasfeo_dtrmm_rutn(nu[0]+nx[0], nx[1], 1.0, &hsBAbt[ii], 0, 0, &hsLxt_old[ii], 0, 0, 0.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0);
	blasfeo_dsyrk_ln(nu[0]+nx[0], nu[0]+nx[0], nx[1], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsRSQrq[0], 0, 0, &hsL[0], 0, 0);
	for(ii=1; ii<md; ii++)
		{
		blasfeo_dtrmm_rutn(nu[0]+nx[0], nx[1], 1.0, &hsBAbt[ii], 0, 0, &hsLxt_old[ii], 0, 0, 0.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0);
		blasfeo_dsyrk_ln(nu[0]+nx[0], nu[0]+nx[0], nx[1], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsL[0], 0, 0, &hsL[0], 0, 0);
		}

	blasfeo_dpotrf_l(nu[0]+nx[0], nu[0]+nx[0], &hsL[0], 0, 0, &hsL[0], 0, 0);
	blasfeo_dtrtr_l(nx[0], &hsL[0], nu[0], nu[0], &hsLxt_new[0], 0, 0);

	return;

	}



void d_back_ric_trf_step1_libstr(int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsL, struct blasfeo_dmat *hsLxt, struct blasfeo_dmat *hswork_mat)
	{

	blasfeo_dtrmm_rutn(nu[0]+nx[0], nx[1], 1.0, &hsBAbt[0], 0, 0, &hsLxt[1], 0, 0, 0.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0);
	blasfeo_dsyrk_ln(nu[0]+nx[0], nu[0]+nx[0], nx[1], 1.0, &hswork_mat[0], 0, 0, &hswork_mat[0], 0, 0, 1.0, &hsRSQrq[0], 0, 0, &hsL[0], 0, 0);
	blasfeo_dpotrf_l(nu[0]+nx[0], nu[0]+nx[0], &hsL[0], 0, 0, &hsL[0], 0, 0);
	blasfeo_dtrtr_l(nx[0], &hsL[0], nu[0], nu[0], &hsLxt[0], 0, 0);

	return;

	}



void d_back_ric_trf_stepN_libstr(int *nx, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsL, struct blasfeo_dmat *hsLxt)
	{

	blasfeo_dpotrf_l(nx[0], nx[0], &hsRSQrq[0], 0, 0, &hsL[0], 0, 0);
	blasfeo_dtrtr_l(nx[0], &hsL[0], 0, 0, &hsLxt[0], 0, 0);

	return;

	}



void d_back_ric_trf_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsL, struct blasfeo_dmat *hsLxt, struct blasfeo_dmat *hswork_mat)
	{

	int nn;

	// factorization

	// last stage
	d_back_ric_trf_stepN_libstr(&nx[N], &hsRSQrq[N], &hsL[N], &hsLxt[N]);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		d_back_ric_trf_step1_libstr(&nx[N-nn-1], &nu[N-nn-1], &hsBAbt[N-nn-1], &hsRSQrq[N-nn-1], &hsL[N-nn-1], &hsLxt[N-nn-1], hswork_mat);
		}

	return;

	}



void d_back_ric_trs_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsL, struct blasfeo_dmat *hsLxt, struct blasfeo_dvec *hsPb, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hswork_vec)
	{

	int nn;

	// backward substitution

	// last stage
	blasfeo_dveccp(nu[N]+nx[N], 1.0, &hsrq[N], 0, &hsux[N], 0);

	// middle stages
	for(nn=0; nn<N-1; nn++)
		{
		// compute Pb
		blasfeo_dtrmv_unn(nx[N-nn], &hsLxt[N-nn], 0, 0, &hsb[N-nn-1], 0, &hsPb[N-nn-1], 0);
		blasfeo_dtrmv_utn(nx[N-nn], &hsLxt[N-nn], 0, 0, &hsPb[N-nn-1], 0, &hsPb[N-nn-1], 0);
		blasfeo_dveccp(nu[N-nn-1]+nx[N-nn-1], 1.0, &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
		blasfeo_dveccp(nx[N-nn], 1.0, &hsPb[N-nn-1], 0, &hswork_vec[0], 0);
		blasfeo_daxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hswork_vec[0], 0);
		blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec[0], 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		blasfeo_dtrsv_lnn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		}

	// first stage
	nn = N-1;
	blasfeo_dtrmv_unn(nx[N-nn], &hsLxt[N-nn], 0, 0, &hsb[N-nn-1], 0, &hsPb[N-nn-1], 0);
	blasfeo_dtrmv_utn(nx[N-nn], &hsLxt[N-nn], 0, 0, &hsPb[N-nn-1], 0, &hsPb[N-nn-1], 0);
	blasfeo_dveccp(nu[N-nn-1]+nx[N-nn-1], 1.0, &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
	blasfeo_dveccp(nx[N-nn], 1.0, &hsPb[N-nn-1], 0, &hswork_vec[0], 0);
	blasfeo_daxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hswork_vec[0], 0);
	blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec[0], 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
	blasfeo_dtrsv_lnn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);

	// forward substitution

	// first stage
	nn = 0;
	blasfeo_dveccp(nx[nn+1], 1.0, &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
	blasfeo_dveccp(nu[nn]+nx[nn], -1.0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_dtrsv_ltn(nu[nn]+nx[nn], nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
	blasfeo_dveccp(nx[nn+1], 1.0, &hsux[nn+1], nu[nn+1], &hswork_vec[0], 0);
	blasfeo_dtrmv_unn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_dtrmv_utn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hswork_vec[0], 0, &hswork_vec[0], 0);
	blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0);

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		blasfeo_dveccp(nx[nn+1], 1.0, &hsux[nn+1], nu[nn+1], &hspi[nn], 0);
		blasfeo_dveccp(nu[nn], -1.0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_dtrsv_ltn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_dveccp(nx[nn+1], 1.0, &hsux[nn+1], nu[nn+1], &hswork_vec[0], 0);
		blasfeo_dtrmv_unn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_dtrmv_utn(nx[nn+1], &hsLxt[nn+1], 0, 0, &hswork_vec[0], 0, &hswork_vec[0], 0);
		blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec[0], 0, &hspi[nn], 0);
		}

	return;

	}



/************************************************
Mass-spring system: nx/2 masses connected each other with springs (in a row), and the first and the last one to walls. nu (<=nx) controls act on the first nu masses. The system is sampled with sampling time Ts.
************************************************/
void mass_spring_system(double Ts, int nx, int nu, int N, double *A, double *B, double *b, double *x0)
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

	printf("\nExample of LU factorization and backsolve\n\n");

#if defined(LA_HIGH_PERFORMANCE)

	printf("\nLA provided by BLASFEO\n\n");

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
	int nx_ = 8;
	int nu_ = 3;

	// stage-wise variant size
	int nx[N+1];
	nx[0] = 0;
	for(ii=1; ii<=N; ii++)
		nx[ii] = nx_;
	nx[N] = nx_;

	int nu[N+1];
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_;
	nu[N] = 0;

/************************************************
* dynamical system
************************************************/

	double *A; d_zeros(&A, nx_, nx_); // states update matrix

	double *B; d_zeros(&B, nx_, nu_); // inputs matrix

	double *b; d_zeros(&b, nx_, 1); // states offset
	double *x0; d_zeros_align(&x0, nx_, 1); // initial state

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx_, nu_, N, A, B, b, x0);

	for(ii=0; ii<nx_; ii++)
		b[ii] = 0.1;

	for(ii=0; ii<nx_; ii++)
		x0[ii] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

	d_print_mat(nx_, nx_, A, nx_);
	d_print_mat(nx_, nu_, B, nx_);
	d_print_mat(1, nx_, b, 1);
	d_print_mat(1, nx_, x0, 1);

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

	d_print_mat(nu_, nu_, R, nu_);
	d_print_mat(nu_, nx_, S, nu_);
	d_print_mat(nx_, nx_, Q, nx_);
	d_print_mat(1, nu_, r, 1);
	d_print_mat(1, nx_, q, 1);

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
	double *b0; d_zeros(&b0, nx_, 1); // states offset
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb, 0, &sb0, 0);
	blasfeo_print_tran_dvec(nx_, &sb0, 0);

	struct blasfeo_dmat sBbt0;
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &sBbt0);
	blasfeo_pack_tran_dmat(nx_, nx_, B, nx_, &sBbt0, 0, 0);
	blasfeo_drowin(nx_, 1.0, &sb0, 0, &sBbt0, nu_, 0);
	blasfeo_print_dmat(nu_+1, nx_, &sBbt0, 0, 0);

	struct blasfeo_dmat sBAbt1;
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &sBAbt1);
	blasfeo_pack_tran_dmat(nx_, nu_, B, nx_, &sBAbt1, 0, 0);
	blasfeo_pack_tran_dmat(nx_, nx_, A, nx_, &sBAbt1, nu_, 0);
	blasfeo_pack_tran_dmat(nx_, 1, b, nx_, &sBAbt1, nu_+nx_, 0);
	blasfeo_print_dmat(nu_+nx_+1, nx_, &sBAbt1, 0, 0);

	struct blasfeo_dvec sr0; // XXX no need to update r0 since S=0
	blasfeo_allocate_dvec(nu_, &sr0);
	blasfeo_pack_dvec(nu_, r, 1, &sr0, 0);

	struct blasfeo_dmat sRr0;
	blasfeo_allocate_dmat(nu_+1, nu_, &sRr0);
	blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRr0, 0, 0);
	blasfeo_drowin(nu_, 1.0, &sr0, 0, &sRr0, nu_, 0);
	blasfeo_print_dmat(nu_+1, nu_, &sRr0, 0, 0);

	struct blasfeo_dvec srq1;
	blasfeo_allocate_dvec(nu_+nx_, &srq1);
	blasfeo_pack_dvec(nu_, r, 1, &srq1, 0);
	blasfeo_pack_dvec(nx_, q, &srq1, nu_);

	struct blasfeo_dmat sRSQrq1;
	blasfeo_allocate_dmat(nu_+nx_+1, nu_+nx_, &sRSQrq1);
	blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRSQrq1, 0, 0);
	blasfeo_pack_tran_dmat(nu_, nx_, S, nu_, &sRSQrq1, nu_, 0);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sRSQrq1, nu_, nu_);
	blasfeo_drowin(nu_+nx_, 1.0, &srq1, 0, &sRSQrq1, nu_+nx_, 0);
	blasfeo_print_dmat(nu_+nx_+1, nu_+nx_, &sRSQrq1, 0, 0);

	struct blasfeo_dvec sqN;
	blasfeo_allocate_dvec(nx_, &sqN);
	blasfeo_pack_dvec(nx_, q, 1, &sqN, 0);

	struct blasfeo_dmat sQqN;
	blasfeo_allocate_dmat(nx_+1, nx_, &sQqN);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sQqN, 0, 0);
	blasfeo_drowin(nx_, 1.0, &sqN, 0, &sQqN, nx_, 0);
	blasfeo_print_dmat(nx_+1, nx_, &sQqN, 0, 0);

/************************************************
* array of matrices
************************************************/

	struct blasfeo_dmat hsBAbt[N];
	struct blasfeo_dvec hsb[N];
	struct blasfeo_dmat hsRSQrq[N+1];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dmat hsL[N+1];
	struct blasfeo_dmat hsLxt[N+1];
	struct blasfeo_dvec hsPb[N];
	struct blasfeo_dvec hsux[N+1];
	struct blasfeo_dvec hspi[N];
	struct blasfeo_dmat hswork_mat[1];
	struct blasfeo_dvec hswork_vec[1];

	hsBAbt[0] = sBbt0;
	hsb[0] = sb0;
	hsRSQrq[0] = sRr0;
	hsrq[0] = sr0;
	blasfeo_allocate_dmat(nu_+1, nu_, &hsL[0]);
//	blasfeo_allocate_dmat(nu_+1, nu_, &hsLxt[0]);
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
		blasfeo_allocate_dmat(nx_, nu_+nx_, &hsLxt[ii]);
		blasfeo_allocate_dvec(nx_, &hsPb[ii]);
		blasfeo_allocate_dvec(nx_+nu_+1, &hsux[ii]);
		blasfeo_allocate_dvec(nx_, &hspi[ii]);
		}
	hsRSQrq[N] = sQqN;
	hsrq[N] = sqN;
	blasfeo_allocate_dmat(nx_+1, nx_, &hsL[N]);
	blasfeo_allocate_dmat(nx_, nx_, &hsLxt[N]);
	blasfeo_allocate_dvec(nx_+nu_+1, &hsux[N]);
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &hswork_mat[0]);
	blasfeo_allocate_dvec(nx_, &hswork_vec[0]);

//	for(ii=0; ii<N; ii++)
//		blasfeo_print_dmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
//	return 0;

/************************************************
* call Riccati solver
************************************************/

	// timing
	struct timeval tv0, tv1, tv2, tv3;
	int nrep = 1000;
	int rep;

	gettimeofday(&tv0, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
//		d_back_ric_sv_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hsLxt, hsux, hspi, hswork_mat, hswork_vec);
		}

	gettimeofday(&tv1, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_trf_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hsLxt, hswork_mat);
		}

	gettimeofday(&tv2, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_trs_libstr(N, nx, nu, hsBAbt, hsb, hsrq, hsL, hsLxt, hsPb, hsux, hspi, hswork_vec);
		}

	gettimeofday(&tv3, NULL); // time

	float time_sv  = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	float time_trf = (float) (tv2.tv_sec-tv1.tv_sec)/(nrep+0.0)+(tv2.tv_usec-tv1.tv_usec)/(nrep*1e6);
	float time_trs = (float) (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);

	// print sol
	printf("\nux = \n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);

	printf("\npi = \n\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_tran_dvec(nx[ii+1], &hspi[ii], 0);

	printf("\ntime sv\t\ttime trf\t\ttime trs\n");
	printf("\n%e\t%e\t%e\n", time_sv, time_trf, time_trs);
	printf("\n");

/************************************************
* free memory
************************************************/

	d_free(A);
	d_free(B);
	d_free(b);
	d_free_align(x0);
	d_free(R);
	d_free(S);
	d_free(Q);
	d_free(r);
	d_free(q);
	d_free(b0);
	blasfeo_free_dmat(&sA);
	blasfeo_free_dvec(&sb);
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
//	blasfeo_free_dmat(&hsLxt[0]);
	blasfeo_free_dvec(&hsPb[0]);
	blasfeo_free_dvec(&hsux[0]);
	blasfeo_free_dvec(&hspi[0]);
	for(ii=1; ii<N; ii++)
		{
		blasfeo_free_dmat(&hsL[ii]);
		blasfeo_free_dmat(&hsLxt[ii]);
		blasfeo_free_dvec(&hsPb[ii]);
		blasfeo_free_dvec(&hsux[ii]);
		blasfeo_free_dvec(&hspi[ii]);
		}
	blasfeo_free_dmat(&hsL[N]);
	blasfeo_free_dmat(&hsLxt[N]);
	blasfeo_free_dvec(&hsux[N]);
	blasfeo_free_dmat(&hswork_mat[0]);
	blasfeo_free_dvec(&hswork_vec[0]);


/************************************************
* return
************************************************/

	return 0;

	}



