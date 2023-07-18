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
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_utils.h>
#include <hpipm_d_part_cond.h>

#include "d_tools.h"



#define KEEP_X0 0

// printing
#ifndef PRINT
#define PRINT 1
#endif



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


	// local variables

	int ii, jj;

	int rep, nrep=1000;

	struct timeval tv0, tv1;



	// problem size

	int nx_ = 8; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = 3; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 8; // horizon lenght



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

	int nbu[N+1];
	for (ii=0; ii<=N; ii++)
		nbu[ii] = nu[ii];

#if 1
	int nbx[N+1];
#if KEEP_X0
	nbx[0] = nx[0]/2;
#else
	nbx[0] = 0;
#endif
	for(ii=1; ii<=N; ii++)
		nbx[ii] = nx[ii]/2;

	int nb[N+1];
	for (ii=0; ii<=N; ii++)
		nb[ii] = nbu[ii]+nbx[ii];

	int ng[N+1];
	ng[0] = 0;
	for(ii=1; ii<N; ii++)
		ng[ii] = 0;
	ng[N] = 0;

	int nsbx[N+1];
	nsbx[0] = 0;
	for(ii=1; ii<N; ii++)
		nsbx[ii] = nx[ii]/2;
	nsbx[N] = nx[N]/2;

	int nsbu[N+1];
	for(ii=0; ii<=N; ii++)
		nsbu[ii] = 0;

	int nsg[N+1];
	for(ii=0; ii<=N; ii++)
		nsg[ii] = 0;

	int ns[N+1];
	for(ii=0; ii<=N; ii++)
		ns[ii] = nsbx[ii] + nsbu[ii] + nsg[ii];

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
	x0[0] = 2.5;
	x0[1] = 2.5;

	double *b0; d_zeros(&b0, nx_, 1);
	dgemv_n_3l(nx_, nx_, A, nx_, x0, b0);
	daxpy_3l(nx_, 1.0, b, b0);

#if PRINT
	d_print_mat(nx_, nx_, A, nx_);
	d_print_mat(nx_, nu_, B, nu_);
	d_print_mat(1, nx_, b, 1);
	d_print_mat(1, nx_, x0, 1);
	d_print_mat(1, nx_, b0, 1);
#endif

/************************************************
* cost function
************************************************/

	double *Q; d_zeros(&Q, nx_, nx_);
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 0.0;

	double *R; d_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 2.0;

	double *S; d_zeros(&S, nu_, nx_);

	double *q; d_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.0;

	double *r; d_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.0;

	double *r0; d_zeros(&r0, nu_, 1);
	dgemv_n_3l(nu_, nx_, S, nu_, x0, r0);
	daxpy_3l(nu_, 1.0, r, r0);

#if PRINT
	d_print_mat(nx_, nx_, Q, nx_);
	d_print_mat(nu_, nu_, R, nu_);
	d_print_mat(nu_, nx_, S, nu_);
	d_print_mat(1, nx_, q, 1);
	d_print_mat(1, nu_, r, 1);
	d_print_mat(1, nu_, r0, 1);
#endif

	// maximum element in cost functions
	double mu0;
	if(ns[1]>0 | ns[N]>0)
		mu0 = 1000.0;
	else
		mu0 = 2.0;


/************************************************
* box & general constraints
************************************************/

	int *idxbx0; int_zeros(&idxbx0, nbx[0], 1);
	double *d_lbx0; d_zeros(&d_lbx0, nbx[0], 1);
	double *d_ubx0; d_zeros(&d_ubx0, nbx[0], 1);
	int *idxbu0; int_zeros(&idxbu0, nbu[0], 1);
	double *d_lbu0; d_zeros(&d_lbu0, nbu[0], 1);
	double *d_ubu0; d_zeros(&d_ubu0, nbu[0], 1);
	double *d_lg0; d_zeros(&d_lg0, ng[0], 1);
	double *d_ug0; d_zeros(&d_ug0, ng[0], 1);
	for(ii=0; ii<nbu[0]; ii++)
		{
		d_lbu0[ii] = - 0.5; // umin
		d_ubu0[ii] =   0.5; // umax
		idxbu0[ii] = ii;
		}
	for(ii=0; ii<nbx[0]; ii++)
		{
		d_lbx0[ii] = - 4.0; // xmin
		d_ubx0[ii] =   4.0; // xmax
		idxbx0[ii] = ii;
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

	int *idxbx1; int_zeros(&idxbx1, nbx[1], 1);
	double *d_lbx1; d_zeros(&d_lbx1, nbx[1], 1);
	double *d_ubx1; d_zeros(&d_ubx1, nbx[1], 1);
	int *idxbu1; int_zeros(&idxbu1, nbu[1], 1);
	double *d_lbu1; d_zeros(&d_lbu1, nbu[1], 1);
	double *d_ubu1; d_zeros(&d_ubu1, nbu[1], 1);
	double *d_lg1; d_zeros(&d_lg1, ng[1], 1);
	double *d_ug1; d_zeros(&d_ug1, ng[1], 1);
	for(ii=0; ii<nbu[1]; ii++)
		{
		d_lbu1[ii] = - 0.5; // umin
		d_ubu1[ii] =   0.5; // umax
		idxbu1[ii] = ii;
		}
	for(ii=0; ii<nbx[1]; ii++)
		{
		d_lbx1[ii] = - 1.0; // xmin
		d_ubx1[ii] =   1.0; // xmax
		idxbx1[ii] = ii;
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


	int *idxbxN; int_zeros(&idxbxN, nbx[N], 1);
	double *d_lbxN; d_zeros(&d_lbxN, nbx[N], 1);
	double *d_ubxN; d_zeros(&d_ubxN, nbx[N], 1);
	double *d_lgN; d_zeros(&d_lgN, ng[N], 1);
	double *d_ugN; d_zeros(&d_ugN, ng[N], 1);
	for(ii=0; ii<nbx[N]; ii++)
		{
		d_lbxN[ii] = - 1.0; // xmin
		d_ubxN[ii] =   1.0; // xmax
		idxbxN[ii] = ii;
		}
	for(ii=0; ii<ng[N]; ii++)
		{
		d_lgN[ii] = - 4.0; // dmin
		d_ugN[ii] =   4.0; // dmax
		}

	double *C0; d_zeros(&C0, ng[0], nx[0]);
	double *D0; d_zeros(&D0, ng[0], nu[0]);
	for(ii=0; ii<nu[0]-nb[0] & ii<ng[0]; ii++)
		D0[ii+(nb[0]+ii)*ng[0]] = 1.0;
	for(; ii<ng[0]; ii++)
		C0[ii+(nb[0]+ii-nu[0])*ng[0]] = 1.0;

	double *C1; d_zeros(&C1, ng[1], nx[1]);
	double *D1; d_zeros(&D1, ng[1], nu[1]);
	for(ii=0; ii<nu[1]-nb[1] & ii<ng[1]; ii++)
		D1[ii+(nb[1]+ii)*ng[1]] = 1.0;
	for(; ii<ng[1]; ii++)
		C1[ii+(nb[1]+ii-nu[1])*ng[1]] = 1.0;

	double *CN; d_zeros(&CN, ng[N], nx[N]);
	double *DN; d_zeros(&DN, ng[N], nu[N]);
	for(ii=0; ii<nu[N]-nb[N] & ii<ng[N]; ii++)
		DN[ii+(nb[N]+ii)*ng[N]] = 1.0;
	for(; ii<ng[N]; ii++)
		CN[ii+(nb[N]+ii-nu[N])*ng[N]] = 1.0;

#if PRINT
	// box constraints
	int_print_mat(1, nbx[0], idxbx0, 1);
	d_print_mat(1, nbx[0], d_lbx0, 1);
	d_print_mat(1, nbx[0], d_ubx0, 1);
	int_print_mat(1, nbu[0], idxbu0, 1);
	d_print_mat(1, nbu[0], d_lbu0, 1);
	d_print_mat(1, nbu[0], d_ubu0, 1);
	int_print_mat(1, nbx[1], idxbx1, 1);
	d_print_mat(1, nbx[1], d_lbx1, 1);
	d_print_mat(1, nbx[1], d_ubx1, 1);
	int_print_mat(1, nbu[1], idxbu1, 1);
	d_print_mat(1, nbu[1], d_lbu1, 1);
	d_print_mat(1, nbu[1], d_ubu1, 1);
	int_print_mat(1, nbx[N], idxbxN, 1);
	d_print_mat(1, nbx[N], d_lbxN, 1);
	d_print_mat(1, nbx[N], d_ubxN, 1);
	// general constraints
	d_print_mat(1, ng[0], d_lg0, 1);
	d_print_mat(1, ng[0], d_ug0, 1);
	d_print_mat(ng[0], nu[0], D0, ng[0]);
	d_print_mat(ng[0], nx[0], C0, ng[0]);
	d_print_mat(1, ng[1], d_lg1, 1);
	d_print_mat(1, ng[1], d_ug1, 1);
	d_print_mat(ng[1], nu[1], D1, ng[1]);
	d_print_mat(ng[1], nx[1], C1, ng[1]);
	d_print_mat(1, ng[N], d_lgN, 1);
	d_print_mat(1, ng[N], d_ugN, 1);
	d_print_mat(ng[N], nu[N], DN, ng[N]);
	d_print_mat(ng[N], nx[N], CN, ng[N]);
#endif

/************************************************
* soft constraints
************************************************/

	double *Zl0; d_zeros(&Zl0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zl0[ii] = 1e3;
	double *Zu0; d_zeros(&Zu0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zu0[ii] = 1e3;
	double *zl0; d_zeros(&zl0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		zl0[ii] = 0e2;
	double *zu0; d_zeros(&zu0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		zu0[ii] = 1e2;
	int *idxs0; int_zeros(&idxs0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		idxs0[ii] = nu[0]+ii;
	double *d_ls0; d_zeros(&d_ls0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		d_ls0[ii] = -1.0; //0.0; //-1.0;
	double *d_us0; d_zeros(&d_us0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		d_us0[ii] = -1.0; //0.0;

	double *Zl1; d_zeros(&Zl1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zl1[ii] = 1e3;
	double *Zu1; d_zeros(&Zu1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zu1[ii] = 1e3;
	double *zl1; d_zeros(&zl1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		zl1[ii] = 0e2;
	double *zu1; d_zeros(&zu1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		zu1[ii] = 1e2;
	int *idxs1; int_zeros(&idxs1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		idxs1[ii] = nu[1]+ii;
	double *d_ls1; d_zeros(&d_ls1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		d_ls1[ii] = -1.0; //0.0; //-1.0;
	double *d_us1; d_zeros(&d_us1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		d_us1[ii] = -1.0; //0.0;

	double *ZlN; d_zeros(&ZlN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		ZlN[ii] = 1e3;
	double *ZuN; d_zeros(&ZuN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		ZuN[ii] = 1e3;
	double *zlN; d_zeros(&zlN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		zlN[ii] = 0e2;
	double *zuN; d_zeros(&zuN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		zuN[ii] = 1e2;
	int *idxsN; int_zeros(&idxsN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		idxsN[ii] = nu[N]+ii;
	double *d_lsN; d_zeros(&d_lsN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		d_lsN[ii] = -1.0; //0.0; //-1.0;
	double *d_usN; d_zeros(&d_usN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		d_usN[ii] = -1.0; //0.0;

#if PRINT
	// soft constraints
	int_print_mat(1, ns[0], idxs0, 1);
	d_print_mat(1, ns[0], Zl0, 1);
	d_print_mat(1, ns[0], Zu0, 1);
	d_print_mat(1, ns[0], zl0, 1);
	d_print_mat(1, ns[0], zu0, 1);
	d_print_mat(1, ns[0], d_ls0, 1);
	d_print_mat(1, ns[0], d_us0, 1);
	int_print_mat(1, ns[1], idxs1, 1);
	d_print_mat(1, ns[1], Zl1, 1);
	d_print_mat(1, ns[1], Zu1, 1);
	d_print_mat(1, ns[1], zl1, 1);
	d_print_mat(1, ns[1], zu1, 1);
	d_print_mat(1, ns[1], d_ls1, 1);
	d_print_mat(1, ns[1], d_us1, 1);
	int_print_mat(1, ns[N], idxsN, 1);
	d_print_mat(1, ns[N], ZlN, 1);
	d_print_mat(1, ns[N], ZuN, 1);
	d_print_mat(1, ns[N], zlN, 1);
	d_print_mat(1, ns[N], zuN, 1);
	d_print_mat(1, ns[N], d_lsN, 1);
	d_print_mat(1, ns[N], d_usN, 1);
#endif

/************************************************
* array of matrices
************************************************/

	double *hA[N];
	double *hB[N];
	double *hb[N];
	double *hQ[N+1];
	double *hS[N+1];
	double *hR[N+1];
	double *hq[N+1];
	double *hr[N+1];
	int *hidxbx[N+1];
	double *hd_lbx[N+1];
	double *hd_ubx[N+1];
	int *hidxbu[N+1];
	double *hd_lbu[N+1];
	double *hd_ubu[N+1];
	double *hC[N+1];
	double *hD[N+1];
	double *hd_lg[N+1];
	double *hd_ug[N+1];
	double *hZl[N+1];
	double *hZu[N+1];
	double *hzl[N+1];
	double *hzu[N+1];
	int *hidxs[N+1]; // XXX
	double *hd_ls[N+1];
	double *hd_us[N+1];

	hA[0] = A;
	hB[0] = B;
	hb[0] = b0;
	hQ[0] = Q;
	hS[0] = S;
	hR[0] = R;
	hq[0] = q;
	hr[0] = r0;
	hidxbx[0] = idxbx0;
	hd_lbx[0] = d_lbx0;
	hd_ubx[0] = d_ubx0;
	hidxbu[0] = idxbu0;
	hd_lbu[0] = d_lbu0;
	hd_ubu[0] = d_ubu0;
	hC[0] = C0;
	hD[0] = D0;
	hd_lg[0] = d_lg0;
	hd_ug[0] = d_ug0;
	hZl[0] = Zl0;
	hZu[0] = Zu0;
	hzl[0] = zl0;
	hzu[0] = zu0;
	hidxs[0] = idxs0;
	hd_ls[0] = d_ls0;
	hd_us[0] = d_us0;
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
		hidxbx[ii] = idxbx1;
		hd_lbx[ii] = d_lbx1;
		hd_ubx[ii] = d_ubx1;
		hidxbu[ii] = idxbu1;
		hd_lbu[ii] = d_lbu1;
		hd_ubu[ii] = d_ubu1;
		hd_lg[ii] = d_lg1;
		hd_ug[ii] = d_ug1;
		hC[ii] = C1;
		hD[ii] = D1;
		hZl[ii] = Zl1;
		hZu[ii] = Zu1;
		hzl[ii] = zl1;
		hzu[ii] = zu1;
		hidxs[ii] = idxs1;
		hd_ls[ii] = d_ls1;
		hd_us[ii] = d_us1;
		}
	hQ[N] = Q;
	hS[N] = S;
	hR[N] = R;
	hq[N] = q;
	hr[N] = r;
	hidxbx[N] = idxbxN;
	hd_lbx[N] = d_lbxN;
	hd_ubx[N] = d_ubxN;
	hd_lg[N] = d_lgN;
	hd_ug[N] = d_ugN;
	hC[N] = CN;
	hD[N] = DN;
	hZl[N] = ZlN;
	hZu[N] = ZuN;
	hzl[N] = zlN;
	hzu[N] = zuN;
	hidxs[N] = idxsN;
	hd_ls[N] = d_lsN;
	hd_us[N] = d_usN;

/************************************************
* ocp qp dim
************************************************/

	hpipm_size_t dim_size = d_ocp_qp_dim_memsize(N);
#if PRINT
	printf("\ndim size = %d\n", dim_size);
#endif
	void *dim_mem = malloc(dim_size);

	struct d_ocp_qp_dim dim;
	d_ocp_qp_dim_create(N, &dim, dim_mem);
	d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dim);

/************************************************
* ocp qp
************************************************/

	hpipm_size_t ocp_qp_size = d_ocp_qp_memsize(&dim);
#if PRINT
	printf("\nocp qp size = %d\n", ocp_qp_size);
#endif
	void *ocp_qp_mem = malloc(ocp_qp_size);

	struct d_ocp_qp ocp_qp;
	d_ocp_qp_create(&dim, &ocp_qp, ocp_qp_mem);
	d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr, hidxbx, hd_lbx, hd_ubx, hidxbu, hd_lbu, hd_ubu, hC, hD, hd_lg, hd_ug, hZl, hZu, hzl, hzu, hidxs, hd_ls, hd_us, &ocp_qp);

#if 0
	printf("\nN = %d\n", ocp_qp.N);
	for(ii=0; ii<N; ii++)
		blasfeo_print_dmat(ocp_qp.nu[ii]+ocp_qp.nx[ii]+1, ocp_qp.nx[ii+1], ocp_qp.BAbt+ii, 0, 0);
	for(ii=0; ii<N; ii++)
		blasfeo_print_tran_dvec(ocp_qp.nx[ii+1], ocp_qp.b+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_dmat(ocp_qp.nu[ii]+ocp_qp.nx[ii]+1, ocp_qp.nu[ii]+ocp_qp.nx[ii], ocp_qp.RSQrq+ii, 0, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(ocp_qp.nu[ii]+ocp_qp.nx[ii], ocp_qp.rq+ii, 0);
	for(ii=0; ii<=N; ii++)
		int_print_mat(1, nb[ii], ocp_qp.idxb[ii], 1);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(ocp_qp.nb[ii], ocp_qp.d_lb+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(ocp_qp.nb[ii], ocp_qp.d_ub+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_dmat(ocp_qp.nu[ii]+ocp_qp.nx[ii], ocp_qp.ng[ii], ocp_qp.DCt+ii, 0, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(ocp_qp.ng[ii], ocp_qp.d_lg+ii, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(ocp_qp.ng[ii], ocp_qp.d_ug+ii, 0);
#endif

/************************************************
* part dense qp dim
************************************************/

	int N2 = 2; // horizon of partially condensed problem

	int nx2[N2+1];
	int nu2[N2+1];
	int nb2[N2+1];
	int nbx2[N2+1];
	int nbu2[N2+1];
	int ng2[N2+1];
	int ns2[N2+1];
	int nsbx2[N2+1];
	int nsbu2[N2+1];
	int nsg2[N2+1];

	hpipm_size_t dim_size2 = d_ocp_qp_dim_memsize(N2);
#if PRINT
	printf("\ndim size2 = %d\n", dim_size2);
#endif
	void *dim_mem2 = malloc(dim_size2);

	struct d_ocp_qp_dim dim2;
	d_ocp_qp_dim_create(N2, &dim2, dim_mem2);
//	d_ocp_qp_dim_set_all(nx2, nu2, nbx2, nbu2, ng2, nsbx2, nsbu2, nsg2, &dim2);

/************************************************
* part dense qp
************************************************/

	hpipm_size_t block_size[N2+1];
#if 1
	d_part_cond_qp_compute_block_size(N, N2, block_size);
#else
	block_size[0] = 1;
	block_size[1] = 1;
	block_size[2] = 3;
#endif
#if PRINT
	printf("\nblock_size\n");
	int_print_mat(1, N2+1, block_size, 1);
#endif

	d_part_cond_qp_compute_dim(&dim, block_size, &dim2);
	for(ii=0; ii<=N2; ii++)
		nx2[ii] = dim2.nx[ii];
	for(ii=0; ii<=N2; ii++)
		nu2[ii] = dim2.nu[ii];
	for(ii=0; ii<=N2; ii++)
		nb2[ii] = dim2.nb[ii];
	for(ii=0; ii<=N2; ii++)
		nbx2[ii] = dim2.nbx[ii];
	for(ii=0; ii<=N2; ii++)
		nbu2[ii] = dim2.nbu[ii];
	for(ii=0; ii<=N2; ii++)
		ng2[ii] = dim2.ng[ii];
	for(ii=0; ii<=N2; ii++)
		ns2[ii] = dim2.ns[ii];
	for(ii=0; ii<=N2; ii++)
		nsbx2[ii] = dim2.nsbx[ii];
	for(ii=0; ii<=N2; ii++)
		nsbu2[ii] = dim2.nsbu[ii];
	for(ii=0; ii<=N2; ii++)
		nsg2[ii] = dim2.nsg[ii];
#if PRINT
	for(ii=0; ii<=N2; ii++)
		printf("\n%d %d %d %d\n", nx2[ii], nu2[ii], nb2[ii], ng2[ii]);
#endif

	// qp
	hpipm_size_t part_dense_qp_size = d_ocp_qp_memsize(&dim2);
#if PRINT
	printf("\npart dense qp size = %d\n", part_dense_qp_size);
#endif
	void *part_dense_qp_mem = malloc(part_dense_qp_size);

	struct d_ocp_qp part_dense_qp;
	d_ocp_qp_create(&dim2, &part_dense_qp, part_dense_qp_mem);

	// arg
	hpipm_size_t part_cond_arg_size = d_part_cond_qp_arg_memsize(dim2.N);
#if PRINT
	printf("\npart cond_arg size = %d\n", part_cond_arg_size);
#endif
	void *part_cond_arg_mem = malloc(part_cond_arg_size);

	struct d_part_cond_qp_arg part_cond_arg;
	d_part_cond_qp_arg_create(dim2.N, &part_cond_arg, part_cond_arg_mem);
	d_part_cond_qp_arg_set_default(&part_cond_arg);

//	for(ii=0; ii<=N2; ii++)
//		part_cond_arg.cond_arg[ii].square_root_alg = 0;

	// ws
	hpipm_size_t part_cond_size = d_part_cond_qp_ws_memsize(&dim, block_size, &dim2, &part_cond_arg);
#if PRINT
	printf("\npart cond size = %d\n", part_cond_size);
#endif
	void *part_cond_mem = malloc(part_cond_size);

	struct d_part_cond_qp_ws part_cond_ws;
	d_part_cond_qp_ws_create(&dim, block_size, &dim2, &part_cond_arg, &part_cond_ws, part_cond_mem);

	/* part cond */

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_part_cond_qp_cond(&ocp_qp, &part_dense_qp, &part_cond_arg, &part_cond_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	printf("\npart cond data\n\n");
	d_ocp_qp_print(&dim2, &part_dense_qp);
#endif

	/* update part cond */

	// index of updated dynamics
	int idxc[N];
	for(ii=0; ii<N; ii++)
		idxc[ii] = 0;
	idxc[0] = 1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_part_cond_qp_update(idxc, &ocp_qp, &part_dense_qp, &part_cond_arg, &part_cond_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_update_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	printf("\nupdate part cond data\n\n");
	d_ocp_qp_print(&dim2, &part_dense_qp);
#endif

	/* cond lhs */

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_part_cond_qp_cond_lhs(&ocp_qp, &part_dense_qp, &part_cond_arg, &part_cond_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_cond_lhs = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	printf("\npart cond lhs data\n\n");
	d_ocp_qp_print(&dim2, &part_dense_qp);
#endif

	/* cond rhs */

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_part_cond_qp_cond_rhs(&ocp_qp, &part_dense_qp, &part_cond_arg, &part_cond_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_cond_rhs = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	printf("\npart cond rhs data\n\n");
	d_ocp_qp_print(&dim2, &part_dense_qp);
#endif

#if 0
	for(ii=0; ii<N; ii++)
		{
		for(jj=0; jj<
		struct blasfeo_dmat *tmp_mat += (part_cond_ws.cond_workspace+ii)->Gamma;
		blasfeo_print_dmat(tmp_mat->m, tmp_mat->n, tmp_mat, 0, 0);
		}
#endif

/************************************************
* part dense qp sol
************************************************/

	hpipm_size_t part_dense_qp_sol_size = d_ocp_qp_sol_memsize(&dim2);
#if PRINT
	printf("\npart dense qp sol size = %d\n", part_dense_qp_sol_size);
#endif
	void *part_dense_qp_sol_mem = malloc(part_dense_qp_sol_size);

	struct d_ocp_qp_sol part_dense_qp_sol;
	d_ocp_qp_sol_create(&dim2, &part_dense_qp_sol, part_dense_qp_sol_mem);

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim2);
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_ocp_qp_ipm_arg arg;
	d_ocp_qp_ipm_arg_create(&dim2, &arg, ipm_arg_mem);

//	enum hpipm_mode mode = SPEED_ABS;
	enum hpipm_mode mode = SPEED;
//	enum hpipm_mode mode = BALANCE;
//	enum hpipm_mode mode = ROBUST;
	d_ocp_qp_ipm_arg_set_default(mode, &arg);

	int iter_max = 20;
	double alpha_min = 1e-8;
	double tol_stat = 1e-8;
	double tol_eq = 1e-12;
	double tol_ineq = 1e-12;
	double tol_comp = 1e-12;
	double reg_prim = 1e-12;
	int warm_start = 0;
	int pred_corr = 1;
	int ric_alg = 1;

	d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
	d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
	d_ocp_qp_ipm_arg_set_alpha_min(&alpha_min, &arg);
	d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
	d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
	d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
	d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
	d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
	d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
	d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
	d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);

/************************************************
* ipm
************************************************/

	hpipm_size_t ipm_size = d_ocp_qp_ipm_ws_memsize(&dim2, &arg);
#if PRINT
	printf("\nipm size = %d\n", ipm_size);
#endif
	void *ipm_mem = malloc(ipm_size);

	struct d_ocp_qp_ipm_ws workspace;
	d_ocp_qp_ipm_ws_create(&dim2, &arg, &workspace, ipm_mem);

	int hpipm_status; // 0 normal; 1 max iter

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_ocp_qp_ipm_solve(&part_dense_qp, &part_dense_qp_sol, &arg, &workspace);
		d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_ocp_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* extract and print part cond solution
************************************************/

	double *u2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(u2+ii, nu2[ii], 1);
	double *x2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(x2+ii, nx2[ii], 1);
	double *ls2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(ls2+ii, ns2[ii], 1);
	double *us2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(us2+ii, ns2[ii], 1);
	double *pi2[N2]; for(ii=0; ii<N2; ii++) d_zeros(pi2+ii, nx2[ii+1], 1);
	double *lam_lb2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(lam_lb2+ii, nb2[ii], 1);
	double *lam_ub2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(lam_ub2+ii, nb2[ii], 1);
	double *lam_lg2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(lam_lg2+ii, ng2[ii], 1);
	double *lam_ug2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(lam_ug2+ii, ng2[ii], 1);
	double *lam_ls2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(lam_ls2+ii, ns2[ii], 1);
	double *lam_us2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(lam_us2+ii, ns2[ii], 1);

	d_ocp_qp_sol_get_all(&part_dense_qp_sol, u2, x2, ls2, us2, pi2, lam_lb2, lam_ub2, lam_lg2, lam_ug2, lam_ls2, lam_us2);

#if PRINT
	printf("\nsolution\n\n");
	printf("\nu2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, nu2[ii], u2[ii], 1);
	printf("\nx2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, nx2[ii], x2[ii], 1);
	printf("\npi2\n");
	for(ii=0; ii<N2; ii++)
		d_print_mat(1, nx2[ii+1], pi2[ii], 1);
	printf("\nls2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ns2[ii], ls2[ii], 1);
	printf("\nus2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ns2[ii], us2[ii], 1);
	printf("\nlam_lb2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, nb2[ii], lam_lb2[ii], 1);
	printf("\nlam_ub2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, nb2[ii], lam_ub2[ii], 1);
	printf("\nlam_lg2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ng2[ii], lam_lg2[ii], 1);
	printf("\nlam_ug2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ng2[ii], lam_ug2[ii], 1);
	printf("\nlam_ls2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ns2[ii], lam_ls2[ii], 1);
	printf("\nlam_us2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ns2[ii], lam_us2[ii], 1);

	printf("\nt_lb2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, nb2[ii], (part_dense_qp_sol.t+ii)->pa+0, 1);
	printf("\nt_ub2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, nb2[ii], (part_dense_qp_sol.t+ii)->pa+nb2[ii]+ng2[ii], 1);
	printf("\nt_lg2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ng2[ii], (part_dense_qp_sol.t+ii)->pa+nb2[ii], 1);
	printf("\nt_ug2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ng2[ii], (part_dense_qp_sol.t+ii)->pa+2*nb2[ii]+ng2[ii], 1);
	printf("\nt_ls2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ns2[ii], (part_dense_qp_sol.t+ii)->pa+2*nb2[ii]+2*ng2[ii], 1);
	printf("\nt_us2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, ns2[ii], (part_dense_qp_sol.t+ii)->pa+2*nb2[ii]+2*ng2[ii]+ns2[ii], 1);
#endif

/************************************************
* extract and print residuals
************************************************/

	double *res_r2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_r2+ii, nu2[ii], 1);
	double *res_q2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_q2+ii, nx2[ii], 1);
	double *res_ls2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_ls2+ii, ns2[ii], 1);
	double *res_us2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_us2+ii, ns2[ii], 1);
	double *res_b2[N2]; for(ii=0; ii<N2; ii++) d_zeros(res_b2+ii, nx2[ii+1], 1);
	double *res_d_lb2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_d_lb2+ii, nb2[ii], 1);
	double *res_d_ub2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_d_ub2+ii, nb2[ii], 1);
	double *res_d_lg2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_d_lg2+ii, ng2[ii], 1);
	double *res_d_ug2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_d_ug2+ii, ng2[ii], 1);
	double *res_d_ls2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_d_ls2+ii, ns2[ii], 1);
	double *res_d_us2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_d_us2+ii, ns2[ii], 1);
	double *res_m_lb2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_m_lb2+ii, nb2[ii], 1);
	double *res_m_ub2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_m_ub2+ii, nb2[ii], 1);
	double *res_m_lg2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_m_lg2+ii, ng2[ii], 1);
	double *res_m_ug2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_m_ug2+ii, ng2[ii], 1);
	double *res_m_ls2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_m_ls2+ii, ns2[ii], 1);
	double *res_m_us2[N2+1]; for(ii=0; ii<=N2; ii++) d_zeros(res_m_us2+ii, ns2[ii], 1);

	d_ocp_qp_res_get_all(workspace.res, res_r2, res_q2, res_ls2, res_us2, res_b2, res_d_lb2, res_d_ub2, res_d_lg2, res_d_ug2, res_d_ls2, res_d_us2, res_m_lb2, res_m_ub2, res_m_lg2, res_m_ug2, res_m_ls2, res_m_us2);

#if PRINT
	printf("\npart cond residuals\n\n");
	printf("\nres_r\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, nu2[ii], res_r2[ii], 1);
	printf("\nres_q\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, nx2[ii], res_q2[ii], 1);
	printf("\nres_ls\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ns2[ii], res_ls2[ii], 1);
	printf("\nres_us\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ns2[ii], res_us2[ii], 1);
	printf("\nres_b\n");
	for(ii=0; ii<N2; ii++)
		d_print_exp_mat(1, nx2[ii+1], res_b2[ii], 1);
	printf("\nres_d_lb\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, nb2[ii], res_d_lb2[ii], 1);
	printf("\nres_d_ub\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, nb2[ii], res_d_ub2[ii], 1);
	printf("\nres_d_lg\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ng2[ii], res_d_lg2[ii], 1);
	printf("\nres_d_ug\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ng2[ii], res_d_ug2[ii], 1);
	printf("\nres_d_ls\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ns2[ii], res_d_ls2[ii], 1);
	printf("\nres_d_us\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ns2[ii], res_d_us2[ii], 1);
	printf("\nres_m_lb\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, nb2[ii], res_m_lb2[ii], 1);
	printf("\nres_m_ub\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, nb2[ii], res_m_ub2[ii], 1);
	printf("\nres_m_lg\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ng2[ii], res_m_lg2[ii], 1);
	printf("\nres_m_ug\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ng2[ii], res_m_ug2[ii], 1);
	printf("\nres_m_ls\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ns2[ii], res_m_ls2[ii], 1);
	printf("\nres_m_us\n");
	for(ii=0; ii<=N2; ii++)
		d_print_exp_mat(1, ns2[ii], res_m_us2[ii], 1);
#endif

/************************************************
* print ipm statistics
************************************************/

	int iter; d_ocp_qp_ipm_get_iter(&workspace, &iter);
	double res_stat; d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
	double res_eq; d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
	double res_ineq; d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
	double res_comp; d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
	double *stat; d_ocp_qp_ipm_get_stat(&workspace, &stat);
	int stat_m; d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

#if PRINT
	printf("\nipm return = %d\n", hpipm_status);
	printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

	printf("\nipm iter = %d\n", iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
	d_print_exp_tran_mat(stat_m, iter+1, stat, stat_m);

	printf("\npart cond time         = %e [s]\n", time_cond);
	printf("\nupdate part cond time  = %e [s]\n", time_update_cond);
	printf("\npart cond lhs time     = %e [s]\n", time_cond_lhs);
	printf("\npart cond rhs time     = %e [s]\n", time_cond_rhs);
	printf("\npart cond ocp ipm time = %e [s]\n\n", time_ocp_ipm);
#endif

/************************************************
* full space ocp qp sol
************************************************/

	hpipm_size_t ocp_qp_sol_size = d_ocp_qp_sol_memsize(&dim);
#if PRINT
	printf("\nocp qp sol size = %d\n", ocp_qp_sol_size);
#endif
	void *ocp_qp_sol_mem = malloc(ocp_qp_sol_size);

	struct d_ocp_qp_sol ocp_qp_sol;
	d_ocp_qp_sol_create(&dim, &ocp_qp_sol, ocp_qp_sol_mem);

/************************************************
* expand solution
************************************************/

	d_part_cond_qp_expand_sol(&ocp_qp, &part_dense_qp, &part_dense_qp_sol, &ocp_qp_sol, &part_cond_arg, &part_cond_ws);

	double *u[N+1]; for(ii=0; ii<=N; ii++) d_zeros(u+ii, nu[ii], 1);
	double *x[N+1]; for(ii=0; ii<=N; ii++) d_zeros(x+ii, nx[ii], 1);
	double *ls[N+1]; for(ii=0; ii<=N; ii++) d_zeros(ls+ii, ns[ii], 1);
	double *us[N+1]; for(ii=0; ii<=N; ii++) d_zeros(us+ii, ns[ii], 1);
	double *pi[N]; for(ii=0; ii<N; ii++) d_zeros(pi+ii, nx[ii+1], 1);
	double *lam_lb[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_lb+ii, nb[ii], 1);
	double *lam_ub[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_ub+ii, nb[ii], 1);
	double *lam_lg[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_lg+ii, ng[ii], 1);
	double *lam_ug[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_ug+ii, ng[ii], 1);
	double *lam_ls[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_ls+ii, ns[ii], 1);
	double *lam_us[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_us+ii, ns[ii], 1);

	d_ocp_qp_sol_get_all(&ocp_qp_sol, u, x, ls, us, pi, lam_lb, lam_ub, lam_lg, lam_ug, lam_ls, lam_us);


#if PRINT
	printf("\nfull space solution\n\n");
	printf("\nu\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu[ii], u[ii], 1);
	printf("\nx\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx[ii], x[ii], 1);
	printf("\nls\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ns[ii], ls[ii], 1);
	printf("\nus\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ns[ii], us[ii], 1);
	printf("\npi\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx[ii+1], pi[ii], 1);
	printf("\nlam_lb\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nb[ii], lam_lb[ii], 1);
	printf("\nlam_ub\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nb[ii], lam_ub[ii], 1);
	printf("\nlam_lg\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ng[ii], lam_lg[ii], 1);
	printf("\nlam_ug\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ng[ii], lam_ug[ii], 1);
	printf("\nlam_ls\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ns[ii], lam_ls[ii], 1);
	printf("\nlam_us\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ns[ii], lam_us[ii], 1);

	printf("\nt_lb\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nb[ii], (ocp_qp_sol.t+ii)->pa, 1);
	printf("\nt_ub\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nb[ii], (ocp_qp_sol.t+ii)->pa+nb[ii]+ng[ii], 1);
	printf("\nt_lg\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ng[ii], (ocp_qp_sol.t+ii)->pa+nb[ii], 1);
	printf("\nt_ug\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ng[ii], (ocp_qp_sol.t+ii)->pa+2*nb[ii]+ng[ii], 1);
	printf("\nt_ls\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ns[ii], (ocp_qp_sol.t+ii)->pa+2*nb[ii]+2*ng[ii], 1);
	printf("\nt_us\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, ns[ii], (ocp_qp_sol.t+ii)->pa+2*nb[ii]+2*ng[ii]+ns[ii], 1);
#endif

/************************************************
* free memory
************************************************/

	d_free(A);
	d_free(B);
	d_free(b);
	d_free(x0);
	d_free(Q);
	d_free(R);
	d_free(S);
	d_free(q);
	d_free(r);
	d_free(r0);
	int_free(idxbx0);
	d_free(d_lbx0);
	d_free(d_ubx0);
	int_free(idxbu0);
	d_free(d_lbu0);
	d_free(d_ubu0);
	int_free(idxbx1);
	d_free(d_lbx1);
	d_free(d_ubx1);
	int_free(idxbu1);
	d_free(d_lbu1);
	d_free(d_ubu1);
	int_free(idxbxN);
	d_free(d_lbxN);
	d_free(d_ubxN);
	d_free(C0);
	d_free(D0);
	d_free(d_lg0);
	d_free(d_ug0);
	d_free(C1);
	d_free(D1);
	d_free(d_lg1);
	d_free(d_ug1);
	d_free(CN);
	d_free(DN);
	d_free(d_lgN);
	d_free(d_ugN);

	d_free(Zl0);
	d_free(Zu0);
	d_free(zl0);
	d_free(zu0);
	int_free(idxs0);
	d_free(d_ls0);
	d_free(d_us0);
	d_free(Zl1);
	d_free(Zu1);
	d_free(zl1);
	d_free(zu1);
	int_free(idxs1);
	d_free(d_ls1);
	d_free(d_us1);
	d_free(ZlN);
	d_free(ZuN);
	d_free(zlN);
	d_free(zuN);
	int_free(idxsN);
	d_free(d_lsN);
	d_free(d_usN);

	for(ii=0; ii<N; ii++)
		{
		d_free(u[ii]);
		d_free(x[ii]);
		d_free(ls[ii]);
		d_free(us[ii]);
		d_free(pi[ii]);
		d_free(lam_lb[ii]);
		d_free(lam_ub[ii]);
		d_free(lam_lg[ii]);
		d_free(lam_ug[ii]);
		d_free(lam_ls[ii]);
		d_free(lam_us[ii]);
		}
	d_free(u[ii]);
	d_free(x[ii]);
	d_free(ls[ii]);
	d_free(us[ii]);
	d_free(lam_lb[ii]);
	d_free(lam_ub[ii]);
	d_free(lam_lg[ii]);
	d_free(lam_ug[ii]);
	d_free(lam_ls[ii]);
	d_free(lam_us[ii]);

	for(ii=0; ii<N2; ii++)
		{
		d_free(u2[ii]);
		d_free(x2[ii]);
		d_free(ls2[ii]);
		d_free(us2[ii]);
		d_free(pi2[ii]);
		d_free(lam_lb2[ii]);
		d_free(lam_ub2[ii]);
		d_free(lam_lg2[ii]);
		d_free(lam_ug2[ii]);
		d_free(lam_ls2[ii]);
		d_free(lam_us2[ii]);

		d_free(res_r2[ii]);
		d_free(res_q2[ii]);
		d_free(res_ls2[ii]);
		d_free(res_us2[ii]);
		d_free(res_b2[ii]);
		d_free(res_d_lb2[ii]);
		d_free(res_d_ub2[ii]);
		d_free(res_d_lg2[ii]);
		d_free(res_d_ug2[ii]);
		d_free(res_d_ls2[ii]);
		d_free(res_d_us2[ii]);
		d_free(res_m_lb2[ii]);
		d_free(res_m_ub2[ii]);
		d_free(res_m_lg2[ii]);
		d_free(res_m_ug2[ii]);
		d_free(res_m_ls2[ii]);
		d_free(res_m_us2[ii]);
		}
	d_free(u2[ii]);
	d_free(x2[ii]);
	d_free(ls2[ii]);
	d_free(us2[ii]);
	d_free(lam_lb2[ii]);
	d_free(lam_ub2[ii]);
	d_free(lam_lg2[ii]);
	d_free(lam_ug2[ii]);
	d_free(lam_ls2[ii]);
	d_free(lam_us2[ii]);

	d_free(res_r2[ii]);
	d_free(res_q2[ii]);
	d_free(res_ls2[ii]);
	d_free(res_us2[ii]);
	d_free(res_d_lb2[ii]);
	d_free(res_d_ub2[ii]);
	d_free(res_d_lg2[ii]);
	d_free(res_d_ug2[ii]);
	d_free(res_d_ls2[ii]);
	d_free(res_d_us2[ii]);
	d_free(res_m_lb2[ii]);
	d_free(res_m_ub2[ii]);
	d_free(res_m_lg2[ii]);
	d_free(res_m_ug2[ii]);
	d_free(res_m_ls2[ii]);
	d_free(res_m_us2[ii]);

	free(ocp_qp_mem);
	free(ocp_qp_sol_mem);
	free(part_dense_qp_mem);
	free(part_dense_qp_sol_mem);
	free(part_cond_mem);
	free(part_cond_arg_mem);
	free(ipm_mem);

/************************************************
* return
************************************************/

	return hpipm_status;

	}
