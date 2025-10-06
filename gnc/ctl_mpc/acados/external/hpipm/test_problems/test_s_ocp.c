/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
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
#include <math.h>
#include <sys/time.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_s_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_s_blas.h>

#include <hpipm_s_ocp_qp.h>
#include <hpipm_s_ocp_qp_sol.h>
#include <hpipm_s_ocp_qp_ipm_hard.h>

#include "s_tools.h"



#define KEEP_X0 0

// printing
#define PRINT 1

/************************************************ 
Mass-spring system: nx/2 masses connected each other with springs (in a row), and the first and the last one to walls. nu (<=nx) controls act on the first nu masses. The system is sampled with sampling time Ts. 
************************************************/
void mass_spring_system(float Ts, int nx, int nu, int N, float *A, float *B, float *b, float *x0)
	{

	int nx2 = nx*nx;

	int info = 0;

	int pp = nx/2; // number of masses
	
/************************************************
* build the continuous time system 
************************************************/
	
	float *T; s_zeros(&T, pp, pp);
	int ii;
	for(ii=0; ii<pp; ii++) T[ii*(pp+1)] = -2;
	for(ii=0; ii<pp-1; ii++) T[ii*(pp+1)+1] = 1;
	for(ii=1; ii<pp; ii++) T[ii*(pp+1)-1] = 1;

	float *Z; s_zeros(&Z, pp, pp);
	float *I; s_zeros(&I, pp, pp); for(ii=0; ii<pp; ii++) I[ii*(pp+1)]=1.0; // = eye(pp);
	float *Ac; s_zeros(&Ac, nx, nx);
	smcopy(pp, pp, Z, pp, Ac, nx);
	smcopy(pp, pp, T, pp, Ac+pp, nx);
	smcopy(pp, pp, I, pp, Ac+pp*nx, nx);
	smcopy(pp, pp, Z, pp, Ac+pp*(nx+1), nx); 
	free(T);
	free(Z);
	free(I);
	
	s_zeros(&I, nu, nu); for(ii=0; ii<nu; ii++) I[ii*(nu+1)]=1.0; //I = eye(nu);
	float *Bc; s_zeros(&Bc, nx, nu);
	smcopy(nu, nu, I, nu, Bc+pp, nx);
	free(I);
	
/************************************************
* compute the discrete time system 
************************************************/

	float *bb; s_zeros(&bb, nx, 1);
	smcopy(nx, 1, bb, nx, b, nx);
		
	smcopy(nx, nx, Ac, nx, A, nx);
	sscal_3l(nx2, Ts, A);
	expm(nx, A);
	
	s_zeros(&T, nx, nx);
	s_zeros(&I, nx, nx); for(ii=0; ii<nx; ii++) I[ii*(nx+1)]=1.0; //I = eye(nx);
	smcopy(nx, nx, A, nx, T, nx);
	saxpy_3l(nx2, -1.0, I, T);
	sgemm_nn_3l(nx, nu, nx, T, nx, Bc, nx, B, nx);
	free(T);
	free(I);
	
	int *ipiv = (int *) malloc(nx*sizeof(int));
	sgesv_3l(nx, nu, Ac, nx, ipiv, B, nx, &info);
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


	// local variables

	int ii, jj;
	
	int rep, nrep=1000;

	struct timeval tv0, tv1;



	// problem size

	int nx_ = 16; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = 7; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 5; // horizon lenght



	// stage-wise variant size

	int nx[N+1];
#if KEEP_X0
	nx[0] = nx_;
#else
	nx[0] = 0;
#endif
	for(ii=1; ii<=N; ii++)
		nx[ii] = nx_;
//	nx[N] = 0;

	int nu[N+1];
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_;
	nu[N] = 0;

#if 1
	int nb[N+1];
#if KEEP_X0
	nb[0] = nu[0]+nx[0]/2;
#else
	nb[0] = nu[0];
#endif
	for(ii=1; ii<N; ii++)
		nb[ii] = nu[1]+nx[1]/2;
	nb[N] = nx[N]/2;

	int ng[N+1];
	ng[0] = 0;
	for(ii=1; ii<N; ii++)
		ng[ii] = 0;
	ng[N] = 0;
#elif 0
	int nb[N+1];
	nb[0] = 0;
	for(ii=1; ii<N; ii++)
		nb[ii] = 0;
	nb[N] = 0;

	int ng[N+1];
#if KEEP_X0
	ng[0] = nu[0]+nx[0]/2;
#else
	ng[0] = nu[0];
#endif
	for(ii=1; ii<N; ii++)
		ng[ii] = nu[1]+nx[1]/2;
	ng[N] = nx[N]/2;
#else
	int nb[N+1];
	nb[0] = nu[0] + nx[0]/2;
	for(ii=1; ii<N; ii++)
		nb[ii] = nu[ii] + nx[ii]/2;
	nb[N] = nu[N] + nx[N]/2;

	int ng[N+1];
#if KEEP_X0
	ng[0] = nx[0]/2;
#else
	ng[0] = 0;
#endif
	for(ii=1; ii<N; ii++)
		ng[ii] = nx[1]/2;
	ng[N] = nx[N]/2;
#endif

/************************************************
* dynamical system
************************************************/	

	float *A; s_zeros(&A, nx_, nx_); // states update matrix

	float *B; s_zeros(&B, nx_, nu_); // inputs matrix

	float *b; s_zeros(&b, nx_, 1); // states offset
	float *x0; s_zeros(&x0, nx_, 1); // initial state

	float Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx_, nu_, N, A, B, b, x0);
	
	for(jj=0; jj<nx_; jj++)
		b[jj] = 0.1;
	
	for(jj=0; jj<nx_; jj++)
		x0[jj] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

	float *b0; s_zeros(&b0, nx_, 1);
	sgemv_n_3l(nx_, nx_, A, nx_, x0, b0);
	saxpy_3l(nx_, 1.0, b, b0);

#if PRINT
	s_print_mat(nx_, nx_, A, nx_);
	s_print_mat(nx_, nu_, B, nu_);
	s_print_mat(1, nx_, b, 1);
	s_print_mat(1, nx_, x0, 1);
	s_print_mat(1, nx_, b0, 1);
#endif

/************************************************
* cost function
************************************************/	
	
	float *Q; s_zeros(&Q, nx_, nx_);
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 1.0;

	float *R; s_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 2.0;

	float *S; s_zeros(&S, nu_, nx_);

	float *q; s_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.1;

	float *r; s_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.2;

	float *r0; s_zeros(&r0, nu_, 1);
	sgemv_n_3l(nu_, nx_, S, nu_, x0, r0);
	saxpy_3l(nu_, 1.0, r, r0);

#if PRINT
	s_print_mat(nx_, nx_, Q, nx_);
	s_print_mat(nu_, nu_, R, nu_);
	s_print_mat(nu_, nx_, S, nu_);
	s_print_mat(1, nx_, q, 1);
	s_print_mat(1, nu_, r, 1);
	s_print_mat(1, nu_, r0, 1);
#endif

	// maximum element in cost functions
	float mu0 = 2.0;

/************************************************
* box & general constraints
************************************************/	

	int *idxb0; int_zeros(&idxb0, nb[0], 1);
	float *d_lb0; s_zeros(&d_lb0, nb[0], 1);
	float *d_ub0; s_zeros(&d_ub0, nb[0], 1);
	float *d_lg0; s_zeros(&d_lg0, ng[0], 1);
	float *d_ug0; s_zeros(&d_ug0, ng[0], 1);
	for(ii=0; ii<nb[0]; ii++)
		{
		if(ii<nu[0]) // input
			{
			d_lb0[ii] = - 0.5; // umin
			d_ub0[ii] =   0.5; // umax
			}
		else // state
			{
			d_lb0[ii] = - 4.0; // xmin
			d_ub0[ii] =   4.0; // xmax
			}
		idxb0[ii] = ii;
		}
	for(ii=0; ii<ng[0]; ii++)
		{
		if(ii<nu[0]-nb[0]) // input
			{
			d_lg0[ii] = - 0.5; // umin
			d_ug0[ii] =   0.5; // umax
			}
		else // state
			{
			d_lg0[ii] = - 4.0; // xmin
			d_ug0[ii] =   4.0; // xmax
			}
		}

	int *idxb1; int_zeros(&idxb1, nb[1], 1);
	float *d_lb1; s_zeros(&d_lb1, nb[1], 1);
	float *d_ub1; s_zeros(&d_ub1, nb[1], 1);
	float *d_lg1; s_zeros(&d_lg1, ng[1], 1);
	float *d_ug1; s_zeros(&d_ug1, ng[1], 1);
	for(ii=0; ii<nb[1]; ii++)
		{
		if(ii<nu[1]) // input
			{
			d_lb1[ii] = - 0.5; // umin
			d_ub1[ii] =   0.5; // umax
			}
		else // state
			{
			d_lb1[ii] = - 4.0; // xmin
			d_ub1[ii] =   4.0; // xmax
			}
		idxb1[ii] = ii;
		}
	for(ii=0; ii<ng[1]; ii++)
		{
		if(ii<nu[1]-nb[1]) // input
			{
			d_lg1[ii] = - 0.5; // umin
			d_ug1[ii] =   0.5; // umax
			}
		else // state
			{
			d_lg1[ii] = - 4.0; // xmin
			d_ug1[ii] =   4.0; // xmax
			}
		}


	int *idxbN; int_zeros(&idxbN, nb[N], 1);
	float *d_lbN; s_zeros(&d_lbN, nb[N], 1);
	float *d_ubN; s_zeros(&d_ubN, nb[N], 1);
	float *d_lgN; s_zeros(&d_lgN, ng[N], 1);
	float *d_ugN; s_zeros(&d_ugN, ng[N], 1);
	for(ii=0; ii<nb[N]; ii++)
		{
		d_lbN[ii] = - 4.0; // xmin
		d_ubN[ii] =   4.0; // xmax
		idxbN[ii] = ii;
		}
	for(ii=0; ii<ng[N]; ii++)
		{
		d_lgN[ii] = - 4.0; // dmin
		d_ugN[ii] =   4.0; // dmax
		}

	float *C0; s_zeros(&C0, ng[0], nx[0]);
	float *D0; s_zeros(&D0, ng[0], nu[0]);
	for(ii=0; ii<nu[0]-nb[0] & ii<ng[0]; ii++)
		D0[ii+(nb[0]+ii)*ng[0]] = 1.0;
	for(; ii<ng[0]; ii++)
		C0[ii+(nb[0]+ii-nu[0])*ng[0]] = 1.0;

	float *C1; s_zeros(&C1, ng[1], nx[1]);
	float *D1; s_zeros(&D1, ng[1], nu[1]);
	for(ii=0; ii<nu[1]-nb[1] & ii<ng[1]; ii++)
		D1[ii+(nb[1]+ii)*ng[1]] = 1.0;
	for(; ii<ng[1]; ii++)
		C1[ii+(nb[1]+ii-nu[1])*ng[1]] = 1.0;

	float *CN; s_zeros(&CN, ng[N], nx[N]);
	float *DN; s_zeros(&DN, ng[N], nu[N]);
	for(ii=0; ii<nu[N]-nb[N] & ii<ng[N]; ii++)
		DN[ii+(nb[N]+ii)*ng[N]] = 1.0;
	for(; ii<ng[N]; ii++)
		CN[ii+(nb[N]+ii-nu[N])*ng[N]] = 1.0;

#if PRINT
	// box constraints
	int_print_mat(1, nb[0], idxb0, 1);
	s_print_mat(1, nb[0], d_lb0, 1);
	s_print_mat(1, nb[0], d_ub0, 1);
	int_print_mat(1, nb[1], idxb1, 1);
	s_print_mat(1, nb[1], d_lb1, 1);
	s_print_mat(1, nb[1], d_ub1, 1);
	int_print_mat(1, nb[N], idxbN, 1);
	s_print_mat(1, nb[N], d_lbN, 1);
	s_print_mat(1, nb[N], d_ubN, 1);
	// general constraints
	s_print_mat(1, ng[0], d_lg0, 1);
	s_print_mat(1, ng[0], d_ug0, 1);
	s_print_mat(ng[0], nu[0], D0, ng[0]);
	s_print_mat(ng[0], nx[0], C0, ng[0]);
	s_print_mat(1, ng[1], d_lg1, 1);
	s_print_mat(1, ng[1], d_ug1, 1);
	s_print_mat(ng[1], nu[1], D1, ng[1]);
	s_print_mat(ng[1], nx[1], C1, ng[1]);
	s_print_mat(1, ng[N], d_lgN, 1);
	s_print_mat(1, ng[N], d_ugN, 1);
	s_print_mat(ng[N], nu[N], DN, ng[N]);
	s_print_mat(ng[N], nx[N], CN, ng[N]);
#endif

/************************************************
* array of matrices
************************************************/	

	float *hA[N];
	float *hB[N];
	float *hb[N];
	float *hQ[N+1];
	float *hS[N+1];
	float *hR[N+1];
	float *hq[N+1];
	float *hr[N+1];
	float *hd_lb[N+1];
	float *hd_ub[N+1];
	float *hd_lg[N+1];
	float *hd_ug[N+1];
	float *hC[N+1];
	float *hD[N+1];
	int *hidxb[N+1];

	hA[0] = A;
	hB[0] = B;
	hb[0] = b0;
	hQ[0] = Q;
	hS[0] = S;
	hR[0] = R;
	hq[0] = q;
	hr[0] = r0;
	hidxb[0] = idxb0;
	hd_lb[0] = d_lb0;
	hd_ub[0] = d_ub0;
	hd_lg[0] = d_lg0;
	hd_ug[0] = d_ug0;
	hC[0] = C0;
	hD[0] = D0;
	for(ii=1; ii<N; ii++)
		{
		hA[ii] = A;
		hB[ii] = B;
		hb[ii] = b;
		hQ[ii] = Q;
		hS[ii] = S;
		hR[ii] = R;
		hq[ii] = q;
		hr[ii] = r;
		hidxb[ii] = idxb1;
		hd_lb[ii] = d_lb1;
		hd_ub[ii] = d_ub1;
		hd_lg[ii] = d_lg1;
		hd_ug[ii] = d_ug1;
		hC[ii] = C1;
		hD[ii] = D1;
		}
	hQ[N] = Q;
	hS[N] = S;
	hR[N] = R;
	hq[N] = q;
	hr[N] = r;
	hidxb[N] = idxbN;
	hd_lb[N] = d_lbN;
	hd_ub[N] = d_ubN;
	hd_lg[N] = d_lgN;
	hd_ug[N] = d_ugN;
	hC[N] = CN;
	hD[N] = DN;
	
/************************************************
* ocp qp
************************************************/	
	
	hpipm_size_t qp_size = s_memsize_ocp_qp(N, nx, nu, nb, ng);
	printf("\nqp size = %d\n", qp_size);
	void *qp_mem = malloc(qp_size);

	struct s_ocp_qp qp;
	s_create_ocp_qp(N, nx, nu, nb, ng, &qp, qp_mem);
	s_cvt_colmaj_to_ocp_qp(hA, hB, hb, hQ, hS, hR, hq, hr, hidxb, hd_lb, hd_ub, hC, hD, hd_lg, hd_ug, &qp);
#if 0
	printf("\nN = %d\n", qp.N);
	for(ii=0; ii<N; ii++)
		blasfeo_print_smat(qp.nu[ii]+qp.nx[ii]+1, qp.nx[ii+1], qp.BAbt+ii, 0, 0);
	for(ii=0; ii<N; ii++)
		blasfeo_print_tran_svec(qp.nx[ii+1], qp.b+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_smat(qp.nu[ii]+qp.nx[ii]+1, qp.nu[ii]+qp.nx[ii], qp.RSQrq+ii, 0, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(qp.nu[ii]+qp.nx[ii], qp.rq+ii, 0);
	for(ii=0; ii<=N; ii++)
		int_print_mat(1, nb[ii], qp.idxb[ii], 1);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(qp.nb[ii], qp.d_lb+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(qp.nb[ii], qp.d_ub+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_smat(qp.nu[ii]+qp.nx[ii], qp.ng[ii], qp.DCt+ii, 0, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(qp.ng[ii], qp.d_lg+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(qp.ng[ii], qp.d_ug+ii, 0);
	return;
#endif

/************************************************
* ocp qp
************************************************/	
	
	hpipm_size_t qp_sol_size = s_memsize_ocp_qp_sol(N, nx, nu, nb, ng);
	printf("\nqp sol size = %d\n", qp_sol_size);
	void *qp_sol_mem = malloc(qp_sol_size);

	struct s_ocp_qp_sol qp_sol;
	s_create_ocp_qp_sol(N, nx, nu, nb, ng, &qp_sol, qp_sol_mem);

/************************************************
* ipm
************************************************/	

	struct s_ipm_hard_ocp_qp_arg arg;
	arg.alpha_min = 1e-8;
	arg.mu_max = 1e-12;
	arg.iter_max = 20;
	arg.mu0 = 2.0;

	hpipm_size_t ipm_size = s_memsize_ipm_hard_ocp_qp(&qp, &arg);
	printf("\nipm size = %d\n", ipm_size);
	void *ipm_mem = malloc(ipm_size);

	struct s_ipm_hard_ocp_qp_workspace workspace;
	s_create_ipm_hard_ocp_qp(&qp, &arg, &workspace, ipm_mem);

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
//		s_solve_ipm_hard_ocp_qp(&qp, &qp_sol, &workspace);
		s_solve_ipm2_hard_ocp_qp(&qp, &qp_sol, &workspace);
		}

	gettimeofday(&tv1, NULL); // stop

	float time_ocp_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 1
	printf("\nsolution\n\n");
	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(nu[ii]+nx[ii], qp_sol.ux+ii, 0);
	printf("\npi\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_tran_svec(nx[ii+1], qp_sol.pi+ii, 0);
	printf("\nlam_lb\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(nb[ii], qp_sol.lam_lb+ii, 0);
	printf("\nlam_ub\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(nb[ii], qp_sol.lam_ub+ii, 0);
	printf("\nlam_lg\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(ng[ii], qp_sol.lam_lg+ii, 0);
	printf("\nlam_ug\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(ng[ii], qp_sol.lam_ug+ii, 0);
	printf("\nt_lb\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(nb[ii], qp_sol.t_lb+ii, 0);
	printf("\nt_ub\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(nb[ii], qp_sol.t_ub+ii, 0);
	printf("\nt_lg\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(ng[ii], qp_sol.t_lg+ii, 0);
	printf("\nt_ug\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_svec(ng[ii], qp_sol.t_ug+ii, 0);

	printf("\nresiduals\n\n");
	printf("\nres_g\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(nu[ii]+nx[ii], workspace.res_g+ii, 0);
	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_exp_tran_svec(nx[ii+1], workspace.res_b+ii, 0);
	printf("\nres_m_lb\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(nb[ii], workspace.res_m_lb+ii, 0);
	printf("\nres_m_ub\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(nb[ii], workspace.res_m_ub+ii, 0);
	printf("\nres_m_lg\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(ng[ii], workspace.res_m_lg+ii, 0);
	printf("\nres_m_ug\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(ng[ii], workspace.res_m_ug+ii, 0);
	printf("\nres_d_lb\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(nb[ii], workspace.res_d_lb+ii, 0);
	printf("\nres_d_ub\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(nb[ii], workspace.res_d_ub+ii, 0);
	printf("\nres_d_lg\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(ng[ii], workspace.res_d_lg+ii, 0);
	printf("\nres_d_ug\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(ng[ii], workspace.res_d_ug+ii, 0);
	printf("\nres_mu\n");
	printf("\n%e\n\n", workspace.res_mu);
#endif

	printf("\nipm iter = %d\n", workspace.iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
	s_print_exp_tran_mat(5, workspace.iter, workspace.stat, 5);

	printf("\nocp ipm time = %e [s]\n\n", time_ocp_ipm);

/************************************************
* free memory
************************************************/	

	s_free(A);
	s_free(B);
	s_free(b);
	s_free(x0);
	s_free(Q);
	s_free(R);
	s_free(S);
	s_free(q);
	s_free(r);
	s_free(r0);
	int_free(idxb0);
	s_free(d_lb0);
	s_free(d_ub0);
	int_free(idxb1);
	s_free(d_lb1);
	s_free(d_ub1);
	int_free(idxbN);
	s_free(d_lbN);
	s_free(d_ubN);
	s_free(C0);
	s_free(D0);
	s_free(d_lg0);
	s_free(d_ug0);
	s_free(C1);
	s_free(D1);
	s_free(d_lg1);
	s_free(d_ug1);
	s_free(CN);
	s_free(DN);
	s_free(d_lgN);
	s_free(d_ugN);

	free(qp_mem);
	free(qp_sol_mem);
	free(ipm_mem);

/************************************************
* return
************************************************/	

	return 0;

	}
