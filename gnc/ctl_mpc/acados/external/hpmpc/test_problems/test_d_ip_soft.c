/**************************************************************************************************
*                                                                                                 *
* This file is part of HPMPC.                                                                     *
*                                                                                                 *
* HPMPC -- Library for High-Performance implementation of solvers for MPC.                        *
* Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                *
*                                                                                                 *
* HPMPC is free software; you can redistribute it and/or                                          *
* modify it under the terms of the GNU Lesser General Public                                      *
* License as published by the Free Software Foundation; either                                    *
* version 2.1 of the License, or (at your option) any later version.                              *
*                                                                                                 *
* HPMPC is distributed in the hope that it will be useful,                                        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  *
*                                                                                                 *
* Author: Gianluca Frison, giaf (at) dtu.dk                                                       *
*                                                                                                 *
**************************************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
#include <xmmintrin.h> // needed to flush to zero sub-normals with _MM_SET_FLUSH_ZERO_MODE (_MM_FLUSH_ZERO_ON); in the main()
#endif

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>
#endif

#include "../include/aux_d.h"
#include "../include/aux_s.h"
#include "../include/blas_d.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../problem_size.h"
#include "../include/block_size.h"
#include "tools.h"
#include "test_param.h"



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
	
	printf("\n");
	printf("\n");
	printf("\n");
	printf(" HPMPC -- Library for High-Performance implementation of solvers for MPC.\n");
	printf(" Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.\n");
	printf("\n");
	printf(" HPMPC is distributed in the hope that it will be useful,\n");
	printf(" but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
	printf(" MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n");
	printf(" See the GNU Lesser General Public License for more details.\n");
	printf("\n");
	printf("\n");
	printf("\n");

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
/*	printf("\nflush subnormals to zero\n");*/
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); // flush to zero subnormals !!! works only with one thread !!!
#endif

	int ii, jj, idx;
	
	int rep, nrep=1;//NREP;

	int nx_ = NX; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = NN; // horizon lenght
	int nb_ = nu_;//nu+nx/2; // number of hard box constraints
	int ns_ = nx_;//nx/2;//nx; // number of soft box constraints

	printf(" Test problem: mass-spring system with %d masses and %d controls.\n", nx_/2, nu_);
	printf("\n");
	printf(" MPC problem size: %d horizon length, %d states, %d inputs, %d two-sided box constraints on inputs and states, %d two-sided soft constraints on states.\n", N, nx_, nu_, nb_, ns_);
	printf("\n");
	printf(" IP method parameters: predictor-corrector IP, double precision, %d maximum iterations, %5.1e exit tolerance in duality measure (edit file test_d_ip_box.c to change them).\n", K_MAX, MU_TOL);

	// size of help matrices in panel-major format
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	
	const int nz_ = nx_+nu_+1;
	const int pnx_ = bs*((nx_+bs-1)/bs);
	const int pnu_ = bs*((nu_+bs-1)/bs);
	const int cnx_ = ncl*((nx_+ncl-1)/ncl);
	

/************************************************
* dynamical system
************************************************/	

	double *A; d_zeros(&A, nx_, nx_); // states update matrix

	double *B; d_zeros(&B, nx_, nu_); // inputs matrix

	double *b; d_zeros(&b, nx_, 1); // states offset
	double *x0; d_zeros(&x0, nx_, 1); // initial state

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx_, nu_, N, A, B, b, x0);
	
	for(jj=0; jj<nx_; jj++)
		b[jj] = 0.0;
	
	for(jj=0; jj<nx_; jj++)
		x0[jj] = 0;
	x0[0] = 3.5;
	x0[1] = 3.5;
	
	// compute b0
	double *pA; d_zeros_align(&pA, pnx_, cnx_);
	d_cvt_mat2pmat(nx_, nx_, A, nx_, 0, pA, cnx_);
	double *b0; d_zeros_align(&b0, pnx_, 1);
#if defined(BLASFEO)
	dgemv_n_lib(nx_, nx_, 1.0, pA, cnx_, x0, 1.0, b, b0);
#else
	dgemv_n_lib(nx_, nx_, pA, cnx_, x0, 1, b, b0);
#endif
	//d_print_pmat(nx_, nx_, bs, pA, cnx_);
	//d_print_mat(nx_, 1, b0, nx_);

/************************************************
* box constraints
************************************************/	

	double u_min_hard = - 0.5;
	double u_max_hard =   0.5;
	double x_min_hard = - 4.0;
	double x_max_hard =   4.0;
	double x_min_soft = - 1.0;
	double x_max_soft =   1.0;

/************************************************
* cost function
************************************************/	

	// cost function wrt states and inputs
	double *Q; d_zeros(&Q, nx_, nx_);
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 0.0;

	double *S; d_zeros(&S, nu_, nx_);

	double *R; d_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 2.0;

	double *q; d_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.1;
	
	double *r; d_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.2;
	
	// cost function wrt slack variables of the soft constraints
	double *Z; d_zeros(&Z, 2*ns_, 1); // quadratic penalty
	for(ii=0; ii<2*ns_; ii++) Z[ii] = 0.0;
	double *z; d_zeros(&z, 2*ns_, 1); // linear penalty
	for(ii=0; ii<2*ns_; ii++) z[ii] = 100.0;

	// maximum element in cost functions
	double mu0 = 100.0;

	// compute r0
	double *pS; d_zeros_align(&pS, pnu_, cnx_);
	d_cvt_mat2pmat(nu_, nx_, S, nu_, 0, pS, cnx_);
	//d_print_pmat(nu, nx, bs, pS, cnx_);
	double *r0; d_zeros_align(&r0, pnu_, 1);
#if defined(BLASFEO)
	dgemv_n_lib(nu_, nx_, 1.0, pS, cnx_, x0, 1.0, r, r0);
#else
	dgemv_n_lib(nu_, nx_, pS, cnx_, x0, 1, r, r0);
#endif
	//d_print_mat(nu, 1, q0, nu);

/************************************************
* IPM settings
************************************************/

	// problem size
	int nx[N+1];
	int nu[N+1];
	int nb[N+1];
	int ng[N+1];
	int ns[N+1];

	// first stage
	nx[0] = 0;
	nu[0] = nu_;
	nb[0] = nu_;
	ng[0] = 0;
	ns[0] = 0;

	// middle stages
	for(ii=1; ii<N; ii++)
		{
		nx[ii] = nx_;
		nu[ii] = nu_;
		nb[ii] = nu_;
		ng[ii] = 0;
		ns[ii] = nx_;
		}
	
	// last stage
	nx[N] = nx_;
	nu[N] = 0;
	nb[N] = 0;
	ng[N] = 0;
	ns[N] = nx_;

	// constraints
	int *idxb0 = (int *) malloc((nb[0]+ns[0])*sizeof(int));
	int nbu0;
	nbu0 = nu[0]<nb[0] ? nu[0] : nb[0];
	idx = 0;
	for(jj=0; jj<nbu0; jj++)
		{
		idxb0[idx] = idx;
		idx++;
		}

	int *idxb1 = (int *) malloc((nb[1]+ns[1])*sizeof(int));
	nbu0 = nu[1]<nb[1] ? nu[1] : nb[1];
	idx = 0;
	for(jj=0; jj<nbu0; jj++)
		{
		idxb1[idx] = idx;
		idx++;
		}
	for(jj=nu[1]; jj<nb[1]; jj++)
		{
		idxb1[idx] = idx;
		idx++;
		}
	for(jj=0; jj<ns[1]; jj++)
		{
		idxb1[idx] = idx;
		idx++;
		}

	int *idxbN = (int *) malloc((nb[N]+ns[N])*sizeof(int));
	idx = 0;
	for(jj=nu[N]; jj<nb[N]; jj++)
		{
		idxbN[idx] = idx;
		idx++;
		}
	for(jj=0; jj<ns[N]; jj++)
		{
		idxbN[idx] = idx;
		idx++;
		}
	
	int *hidxb[N+1];
	hidxb[0] = idxb0;
	for(ii=1; ii<N; ii++)
		{
		hidxb[ii] = idxb1;
		}
	hidxb[N] = idxbN;


	// IPM arguments
	int kk = 0; // acutal number of iterations
	int k_max = K_MAX; // maximum number of iterations in the IP method
	double mu_tol = MU_TOL; // tolerance in the duality measure
	double alpha_min = ALPHA_MIN; // minimum accepted step length
	double sigma[] = {0.4, 0.3, 0.01}; // control primal-dual IP behaviour
	double *stat; d_zeros(&stat, 5, k_max); // stats from the IP routine
	int compute_mult = COMPUTE_MULT;
	int warm_start = WARM_START;
	double mu = -1.0;
	int hpmpc_status;
	
	// initial states
	double xx0[] = {3.5, 3.5, 3.66465, 2.15833, 1.81327, -0.94207, 1.86531, -2.35760, 2.91534, 1.79890, -1.49600, -0.76600, -2.60268, 1.92456, 1.66630, -2.28522, 3.12038, 1.83830, 1.93519, -1.87113};

	// timing
	struct timeval tv0, tv1;

/**************************************************************************************************
*
*	high-level interace
*
**************************************************************************************************/

	double *hA[N];
	double *hB[N];
	double *hb[N];
	double *hR[N];
	double *hS[N];
	double *hQ[N+1];
	double *hr[N];
	double *hq[N+1];

	ii = 0;
	//hA[ii] = A;
	hB[ii] = B;
	hb[ii] = b0;
	hR[ii] = R;
	hr[ii] = r0;
	for(ii=1; ii<N; ii++)
		{
		hA[ii] = A;
		hB[ii] = B;
		hb[ii] = b;
		hR[ii] = R;
		hS[ii] = S;
		hQ[ii] = Q;
		hr[ii] = r;
		hq[ii] = q;
		}
	ii = N;
	hQ[ii] = Q;
	hq[ii] = q;

	double inf_norm_res[4];

	void *work0;
	v_zeros(&work0, hpmpc_d_ip_ocp_soft_tv_work_space_size_bytes(N, nx, nu, nb, hidxb, ng, ns));
	printf("\nhigh level interface work space size in bytes = %d\n", hpmpc_d_ip_ocp_soft_tv_work_space_size_bytes(N, nx, nu, nb, hidxb, ng, ns));

//	TODO
//	hpmpc_status = fortran_order_d_ip_ocp_soft_tv(&kk, k_max, mu0, mu_tol, N, nx, nu, nb, hidxb, ng, ns, warm_start, hA, hB, hb, hQ, hS, hR, hrq, hr, hZ, hz, hlb, hub, hC, hD, hlg, hug, hx, hu, hpi, hlam, inf_norm_res, work0, stat);

/**************************************************************************************************
*
*	(OLD) low-level interface
*
**************************************************************************************************/

	// matrix sizes
	int pnz[N+1];
	int pnx[N+1];
	int pnb[N+1];
	int png[N+1];
	int pns[N+1];
	int cnz[N+1];
	int cnux[N+1];
	int cnx[N+1];
	int cnl[N+1];

	for(ii=0; ii<=N; ii++)
		{
		pnz[ii] = (nu[ii]+nx[ii]+1+bs-1)/bs*bs;
		pnx[ii] = (nx[ii]+bs-1)/bs*bs;
		pnb[ii] = (nb[ii]+bs-1)/bs*bs;
		png[ii] = (ng[ii]+bs-1)/bs*bs;
		pns[ii] = (ns[ii]+bs-1)/bs*bs;
		cnz[ii] = (nu[ii]+nx[ii]+1+ncl-1)/ncl*ncl;
		cnux[ii] = (nu[ii]+nx[ii]+ncl-1)/ncl*ncl;
		cnx[ii] = (nx[ii]+ncl-1)/ncl*ncl;
		cnl[ii] = cnz[ii]<cnx[ii]+ncl ? cnx[ii]+ncl : cnz[ii];
		}
	
//	for(ii=0; ii<=N; ii++)
//		printf("\n%d\t%d\t%d\t%d\t%d\t%d\t%d\n", pnz[ii], pnx[ii], pnb[ii], pns[ii], cnz[ii], cnx[ii], cnl[ii]);



	// state-space matrices
	//d_print_mat(nx, nx, A, nx);
	//d_print_mat(nx, nu, B, nx);
	//for(ii=0; ii<nx; ii++) b[ii] = 1.0;
	//d_print_mat(nx, 1, b, nx);
	//d_print_mat(nx, 1, x0, nx);

	double *pBAbt0; d_zeros_align(&pBAbt0, pnz[0], cnx[1]);
	d_cvt_tran_mat2pmat(nx_, nu_, B, nx_, 0, pBAbt0, cnx[1]);
	d_cvt_tran_mat2pmat(nx_, 1, b0, nx_, nu_, pBAbt0+nu_/bs*bs*cnx[1]+nu_%bs, cnx[1]);
	//d_print_pmat(nu[0]+nx[0]+1, nx[1], bs, pBAbt0, cnx[1]);

	double *pBAbt1; d_zeros_align(&pBAbt1, pnz[1], cnx[2]);
	d_cvt_tran_mat2pmat(nx_, nu_, B, nx_, 0, pBAbt1, cnx[2]);
	d_cvt_tran_mat2pmat(nx_, nx_, A, nx_, nu_, pBAbt1+nu_/bs*bs*cnx[2]+nu_%bs, cnx[2]);
	d_cvt_tran_mat2pmat(nx_, 1, b, nx_, nu_+nx_, pBAbt1+(nu_+nx_)/bs*bs*cnx[2]+(nu_+nx_)%bs, cnx[2]);
//	d_print_pmat(nu[1]+nx[1]+1, nx[2], bs, pBAbt1, cnx[2]);
	
	double *(hpBAbt[N]);
	hpBAbt[0] = pBAbt0;
	for(ii=1; ii<N; ii++)
		hpBAbt[ii] = pBAbt1;
	

	// cost function matrices
	//for(ii=nu; ii<nu+nx; ii++) Q[ii*(nz+1)] = 1.0; // TODO remove !!!!
	//d_print_mat(nz, nz, Q, nz);

	double *pRSQrq0; d_zeros_align(&pRSQrq0, pnz[0], cnux[0]);
	d_cvt_mat2pmat(nu_, nu_, R, nu_, 0, pRSQrq0, cnux[0]);
	d_cvt_tran_mat2pmat(nu_, 1, r0, nu_, nu_, pRSQrq0+nu_/bs*bs*cnux[0]+nu_%bs, cnux[0]);
	//d_print_pmat(nu[0]+nx[0]+1, nu[0]+nx[0]+1, bs, pQ0, pnz[0]);
	
	double *pRSQrq1; d_zeros_align(&pRSQrq1, pnz[1], cnux[1]);
	d_cvt_mat2pmat(nu_, nu_, R, nu_, 0, pRSQrq1, cnux[1]);
	d_cvt_tran_mat2pmat(nu_, nx_, S, nu_, nu_, pRSQrq1+nu_/bs*bs*cnux[1]+nu_%bs, cnux[1]);
	d_cvt_tran_mat2pmat(nx_, nx_, Q, nx_, nu_, pRSQrq1+nu_/bs*bs*cnux[1]+nu_%bs+nu_*bs, cnux[1]);
	d_cvt_tran_mat2pmat(nu_, 1, r, nu_, nu_+nx_, pRSQrq1+(nu_+nx_)/bs*bs*cnux[1]+(nu_+nx_)%bs, cnux[1]);
	d_cvt_tran_mat2pmat(nx_, 1, q, nx_, nu_+nx_, pRSQrq1+(nu_+nx_)/bs*bs*cnux[1]+(nu_+nx_)%bs+nu_*bs, cnux[1]);
	//d_print_pmat(nu[1]+nx[1]+1, nu[1]+nx[1]+1, bs, pQ1, pnz[1]);

	double *pRSQrqN; d_zeros_align(&pRSQrqN, pnz[N], cnux[N]);
	d_cvt_mat2pmat(nx_, nx_, Q, nx_, 0, pRSQrqN, cnux[N]);
	d_cvt_tran_mat2pmat(nx_, 1, q, nx_, nx_, pRSQrqN+nx_/bs*bs*cnux[N]+nx_%bs, cnux[N]);
	//d_print_pmat(nu[N]+nx[N]+1, nu[N]+nx[N], pQN, cnux[N]);

	double *(hpQ[N+1]);
	hpQ[0] = pRSQrq0;
	for(ii=1; ii<N; ii++)
		hpQ[ii] = pRSQrq1;
	hpQ[N] = pRSQrqN;
	


	double *(hpL[N+1]);
	for(ii=0; ii<=N; ii++)
		d_zeros_align(&hpL[ii], pnz[ii], cnl[ii]);

	double *(hdL[N+1]);
	for(ii=0; ii<=N; ii++)
		d_zeros_align(&hdL[ii], pnz[ii], 1);



	double *hux[N+1];
	for(ii=0; ii<=N; ii++)
		d_zeros_align(&hux[ii], (nu[ii]+nx[ii]+bs-1)/bs*bs, 1);
	
	double *hpi[N];
	for(ii=0; ii<N; ii++)
		d_zeros_align(&hpi[ii], pnx[ii+1], 1);
	

	// dummy variables
	int **pdummyi;
	double **pdummyd;
	

	// constraints
	double *db0; d_zeros_align(&db0, 2*pnb[0]+2*pns[0], 1);
	nbu0 = nu[0]<nb[0] ? nu[0] : nb[0];
	for(jj=0; jj<nbu0; jj++)
		{
		db0[0*pnb[0]+jj] = u_min_hard;
		db0[1*pnb[0]+jj] = u_max_hard;
		}

	double *db1; d_zeros_align(&db1, 2*pnb[1]+2*pns[1], 1);
	nbu0 = nu[1]<nb[1] ? nu[1] : nb[1];
	for(jj=0; jj<nbu0; jj++)
		{
		db1[0*pnb[1]+jj] = u_min_hard;
		db1[1*pnb[1]+jj] = u_max_hard;
		}
	for(jj=nu[1]; jj<nb[1]; jj++)
		{
		db1[0*pnb[1]+jj] = x_min_hard;
		db1[1*pnb[1]+jj] = x_max_hard;
		}
	for(jj=0; jj<ns[1]; jj++)
		{
		db1[2*pnb[1]+0*pns[1]+jj] = x_min_soft;
		db1[2*pnb[1]+1*pns[1]+jj] = x_max_soft;
		}

	double *dbN; d_zeros_align(&dbN, 2*pnb[N]+2*pns[N], 1);
	for(jj=nu[N]; jj<nb[N]; jj++)
		{
		dbN[0*pnb[N]+jj] = x_min_hard;
		dbN[1*pnb[N]+jj] = x_max_hard;
		}
	for(jj=0; jj<ns[N]; jj++)
		{
		dbN[2*pnb[N]+0*pns[N]+jj] = x_min_soft;
		dbN[2*pnb[N]+1*pns[N]+jj] = x_max_soft;
		}
	
	double *hdb[N+1];
	hdb[0] = db0;
	for(ii=1; ii<N; ii++)
		{
		hdb[ii] = db1;
		}
	hdb[N] = dbN;

#if 0
	for(ii=0; ii<=N; ii++)
		{
		for(jj=0; jj<nb[ii]+ns[ii]; jj++)
			printf("\t%d", idxb[ii][jj]);
		printf("\n");
		}
#endif
	

	// cost function of the soft contraint slack variables
	double *Z1; d_zeros_align(&Z1, 2*pns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		{
		Z1[0*pns[1]+ii] = Z[0*ns_+ii];
		Z1[1*pns[1]+ii] = Z[1*ns_+ii];
		}
	double *z1; d_zeros_align(&z1, 2*pns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		{
		z1[0*pns[1]+ii] = z[0*ns_+ii];
		z1[1*pns[1]+ii] = z[1*ns_+ii];
		}
	
	double *hZ[N+1];
	double *hz[N+1];
	for(ii=0; ii<=N; ii++)
		{
		hZ[ii] = Z1;
		hz[ii] = z1;
		}

	// lagrangian multipliers and slack variables
	double *hlam[N+1];
	double *ht[N+1];
	for(ii=0; ii<=N; ii++)
		{
		d_zeros_align(&hlam[ii], 2*pnb[ii]+2*png[ii]+4*pns[ii], 1);
		d_zeros_align(&ht[ii], 2*pnb[ii]+2*png[ii]+4*pns[ii], 1);
		}



	// ip soft work space
	double *ip_soft_work; d_zeros_align(&ip_soft_work, d_ip2_mpc_soft_tv_work_space_size_bytes(N, nx, nu, nb, ng, ns)/sizeof(double), 1);

	// call the ip soft solver
	d_ip2_mpc_soft_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx, nu, nb, hidxb, ng, ns, hpBAbt, hpQ, hZ, hz, pdummyd, hdb, hux, 1, hpi, hlam, ht, ip_soft_work);



	// start timer
	gettimeofday(&tv0, NULL); // start



	for(rep=0; rep<nrep; rep++)
		{

		idx = rep%10;
//		x0[0] = xx0[2*idx];
//		x0[1] = xx0[2*idx+1];

		// initialize states and inputs
//		for(ii=0; ii<=N; ii++)
//			for(jj=0; jj<nx+nu; jj++)
//				hux[ii][jj] = 0;

		x0[0] = xx0[2*idx];
		x0[1] = xx0[2*idx+1];

		// update initial state embedded in b and r
#if defined(BLASFEO)
		dgemv_n_lib(nx_, nx_, 1.0, pA, cnx_, x0, 1.0, b, b0);
#else
		dgemv_n_lib(nx_, nx_, pA, cnx_, x0, 1, b, b0);
#endif
		d_cvt_tran_mat2pmat(nx_, 1, b0, nx_, nu_, pBAbt0+nu_/bs*bs*cnx[1]+nu_%bs, cnx[1]);
#if defined(BLASFEO)
		dgemv_n_lib(nu_, nx_, 1.0, pS, cnx_, x0, 1.0, r, r0);
#else
		dgemv_n_lib(nu_, nx_, pS, cnx_, x0, 1, r, r0);
#endif
		d_cvt_tran_mat2pmat(nu_, 1, r0, nu_, nu_, pRSQrq0+nu_/bs*bs*cnux[0]+nu_%bs, cnux[0]);

		// call the IP solver
		d_ip2_mpc_soft_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx, nu, nb, hidxb, ng, ns, hpBAbt, hpQ, hZ, hz, pdummyd, hdb, hux, 1, hpi, hlam, ht, ip_soft_work);

		}
	
	gettimeofday(&tv1, NULL); // stop
	

	
	double *hrrq[N+1];
	double *hrb[N];
	double *hrd[N+1];
	double *hrz[N+1];
	double *hrq[N+1];

	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hrrq[ii], pnz[ii], 1);
		d_zeros_align(&hrb[ii], pnx[ii+1], 1);
		d_zeros_align(&hrd[ii], 2*pnb[ii]+2*png[ii]+2*pns[ii], 1);
		d_zeros_align(&hrz[ii], 2*pns[ii], 1);
		d_zeros_align(&hrq[ii], pnz[ii], 1);
		}
	d_zeros_align(&hrrq[N], pnz[N], 1);
	d_zeros_align(&hrd[N], 2*pnb[N]+2*png[N]+2*pns[N], 1);
	d_zeros_align(&hrz[N], 2*pns[N], 1);
	d_zeros_align(&hrq[N], pnz[N], 1);


	// restore linear part of cost function 
	for(ii=0; ii<=N; ii++)
		{
		drowex_lib(nu[ii]+nx[ii], 1.0, hpQ[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs, hrq[ii]);
		}
	
	for(ii=0; ii<=N; ii++)
		d_print_pmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], hpQ[ii], cnux[ii]);



// TODO
#if 1
	// residuals computation
//	d_res_ip_soft_mpc(nx, nu, N, nh, ns, hpBAbt, hpQ, hrq, hZ, hz, hux, hdb, hpi, hlam, ht, hrrq, hrb, hrd, hrz, &mu);
	d_res_mpc_soft_tv(N, nx, nu, nb, hidxb, ng, ns, hpBAbt, hpQ, hrq, hZ, hz, hux, pdummyd, hdb, hpi, hlam, ht, hrrq, hrb, hrd, hrz, &mu);
#endif




	if(PRINTSTAT==1)
		{

		printf("\n");
		printf("\n");
		printf(" Print IP statistics of the last run (soft-constraints time-variant solver)\n");
		printf("\n");

		for(jj=0; jj<kk; jj++)
			printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
		printf("\n");
		
		}

	if(PRINTRES==1)
		{

		printf("\n");
		printf("\n");
		printf(" Print solution\n");
		printf("\n");

		// print solution
		printf("\nhux = \n\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, nu[ii]+nx[ii], hux[ii], 1);
		
		printf("\nhpi = \n\n");
		for(ii=0; ii<N; ii++)
			d_print_mat(1, nx[ii+1], hpi[ii], 1);
		
		}

	if(PRINTRES==1 && COMPUTE_MULT==1)
		{
		// print result 
		// print result 
		printf("\n");
		printf("\n");
		printf(" Print residuals\n\n");
		printf("\n");
		printf("\n");
		printf("rq = \n\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, nu[ii]+nx[ii], hrrq[ii], 1);
		printf("\n");
		printf("\n");
		printf("rz = \n\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, 2*pns[ii], hrz[ii], 1);
		printf("\n");
		printf("\n");
		printf("rb = \n\n");
		for(ii=0; ii<N; ii++)
			d_print_mat(1, nx[ii], hrb[ii], 1);
		printf("\n");
		printf("\n");
		printf("rd = \n\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, 2*pnb[ii]+2*png[ii]+2*pns[ii], hrd[ii], 1);
		printf("\n");
		printf("\n");
		printf("mu = %e\n\n", mu);
		
		}



	// free memory
	free(pA);
	free(b0);
	free(pBAbt0);
	free(pBAbt1);
	free(pRSQrq0);
	free(pRSQrq1);
	free(pRSQrqN);
	free(idxb0);
	free(idxb1);
	free(idxbN);
	free(db0);
	free(db1);
	free(dbN);
	free(Z1);
	free(z1);
	for(ii=0; ii<=N; ii++) free(hpL[ii]);
	for(ii=0; ii<=N; ii++) free(hdL[ii]);
	for(ii=0; ii<=N; ii++) free(hux[ii]);
	for(ii=0; ii<=N; ii++) free(hpi[ii]);
	return 0;
	for(ii=0; ii<=N; ii++) free(hlam[ii]);
	for(ii=0; ii<=N; ii++) free(ht[ii]);
	for(ii=0; ii<=N; ii++) free(hrrq[ii]);
	for(ii=0; ii<N; ii++) free(hrb[ii]);
	for(ii=0; ii<=N; ii++) free(hrd[ii]);
	for(ii=0; ii<=N; ii++) free(hrz[ii]);
	for(ii=0; ii<=N; ii++) free(hrq[ii]);



/**************************************************************************************************
*	printing timings
**************************************************************************************************/

	double time_soft_low_level = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	
/*	printf("\nnx\tnu\tN\tkernel\n\n");*/
/*	printf("\n%d\t%d\t%d\t%e\n\n", nx, nu, N, time);*/
	
	printf("\n");
	printf(" Average solution time over %d runs: %5.2e seconds (soft-constraints solver)\n", nrep, time_soft_low_level);
	printf("\n");



/************************************************
* free memory and return
************************************************/

	free(A);
	free(B);
	free(b);
	free(x0);



	return 0;

	}



