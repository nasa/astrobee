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

#include <hpipm_d_ocp_qcqp_dim.h>
#include <hpipm_d_ocp_qcqp.h>
#include <hpipm_d_ocp_qcqp_sol.h>
#include <hpipm_d_ocp_qcqp_utils.h>
#include <hpipm_d_dense_qcqp.h>
#include <hpipm_d_dense_qcqp_sol.h>
#include <hpipm_d_dense_qcqp_res.h>
#include <hpipm_d_dense_qcqp_ipm.h>
#include <hpipm_d_dense_qcqp_utils.h>
#include <hpipm_d_cond_qcqp.h>
#include <hpipm_d_part_cond_qcqp.h>

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

	int nx_ = 2; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = 1; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 15; // horizon lenght



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
	for (ii=0; ii<N; ii++)
		nbu[ii] = 0;//nu[ii];
	nbu[N] = 0;

	int nbx[N+1];
#if KEEP_X0
	nbx[0] = nx[0];///2;
#else
	nbx[0] = 0;
#endif
	for(ii=1; ii<=N; ii++)
		nbx[ii] = 0;//nx[ii]/2;

	int nb[N+1];
	for(ii=0; ii<=N; ii++)
		nb[ii] = nbu[ii]+nbx[ii];

	int ng[N+1];
	ng[0] = 0;
	for(ii=1; ii<N; ii++)
		ng[ii] = 0;
	ng[N] = 0;

	int nq[N+1];
	nq[0] = 1;//0;
	for(ii=1; ii<N; ii++)
		nq[ii] = 1;//2;
//	nq[N-1] = 1;
	nq[N] = 1;

	int nsbx[N+1];
	nsbx[0] = 0;
	for(ii=1; ii<N; ii++)
		nsbx[ii] = 0;//nx[ii]/2;
	nsbx[N] = 0;//nx[N]/2;

	int nsbu[N+1];
	for(ii=0; ii<=N; ii++)
		nsbu[ii] = 0;

	int nsg[N+1];
	for(ii=0; ii<=N; ii++)
		nsg[ii] = 0;

	int nsq[N+1];
	nsq[0] = 0;
	for(ii=1; ii<=N; ii++)
		nsq[ii] = 0;

	int ns[N+1];
	for(ii=0; ii<=N; ii++)
		ns[ii] = nsbx[ii] + nsbu[ii] + nsg[ii] + nsq[ii];

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
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 1.0;

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
		mu0 = 10.0;


/************************************************
* box & general constraints
************************************************/

	int *idxbx0; int_zeros(&idxbx0, nbx[0], 1);
	double *lbx0; d_zeros(&lbx0, nbx[0], 1);
	double *ubx0; d_zeros(&ubx0, nbx[0], 1);
	int *idxbu0; int_zeros(&idxbu0, nbu[0], 1);
	double *lbu0; d_zeros(&lbu0, nbu[0], 1);
	double *ubu0; d_zeros(&ubu0, nbu[0], 1);
	double *lg0; d_zeros(&lg0, ng[0], 1);
	double *ug0; d_zeros(&ug0, ng[0], 1);
	for(ii=0; ii<nbu[0]; ii++)
		{
		lbu0[ii] = - 0.5; // umin
		ubu0[ii] =   0.5; // umax
		idxbu0[ii] = ii;
		}
	for(ii=0; ii<nbx[0]; ii++)
		{
		lbx0[ii] = - 4.0; // xmin
		ubx0[ii] =   4.0; // xmax
		idxbx0[ii] = ii;
		}
	for(ii=0; ii<ng[0]; ii++)
		{
		if(ii<nu[0]-nb[0]) // input
			{
			lg0[ii] = - 0.5; // umin
			ug0[ii] =   0.5; // umax
			}
		else // state
			{
			lg0[ii] = - 4.0; // xmin
			ug0[ii] =   4.0; // xmax
			}
		}

	int *idxbx1; int_zeros(&idxbx1, nbx[1], 1);
	double *lbx1; d_zeros(&lbx1, nbx[1], 1);
	double *ubx1; d_zeros(&ubx1, nbx[1], 1);
	int *idxbu1; int_zeros(&idxbu1, nbu[1], 1);
	double *lbu1; d_zeros(&lbu1, nbu[1], 1);
	double *ubu1; d_zeros(&ubu1, nbu[1], 1);
	double *lg1; d_zeros(&lg1, ng[1], 1);
	double *ug1; d_zeros(&ug1, ng[1], 1);
	for(ii=0; ii<nbu[1]; ii++)
		{
		lbu1[ii] = - 0.5; // umin
		ubu1[ii] =   0.5; // umax
		idxbu1[ii] = ii;
		}
	for(ii=0; ii<nbx[1]; ii++)
		{
		lbx1[ii] = - 4.0; // xmin
		ubx1[ii] =   4.0; // xmax
		idxbx1[ii] = ii;
		}
	for(ii=0; ii<ng[1]; ii++)
		{
		if(ii<nu[1]-nb[1]) // input
			{
			lg1[ii] = - 0.5; // umin
			ug1[ii] =   0.5; // umax
			}
		else // state
			{
			lg1[ii] = - 4.0; // xmin
			ug1[ii] =   4.0; // xmax
			}
		}


	int *idxbxN; int_zeros(&idxbxN, nbx[N], 1);
	double *lbxN; d_zeros(&lbxN, nbx[N], 1);
	double *ubxN; d_zeros(&ubxN, nbx[N], 1);
	double *lgN; d_zeros(&lgN, ng[N], 1);
	double *ugN; d_zeros(&ugN, ng[N], 1);
	for(ii=0; ii<nbx[N]; ii++)
		{
		lbxN[ii] = - 4.0; // xmin
		ubxN[ii] =   4.0; // xmax
		idxbxN[ii] = ii;
		}
	for(ii=0; ii<ng[N]; ii++)
		{
		lgN[ii] = - 4.0; // dmin
		ugN[ii] =   4.0; // dmax
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
	d_print_mat(1, nbx[0], lbx0, 1);
	d_print_mat(1, nbx[0], ubx0, 1);
	int_print_mat(1, nbu[0], idxbu0, 1);
	d_print_mat(1, nbu[0], lbu0, 1);
	d_print_mat(1, nbu[0], ubu0, 1);
	int_print_mat(1, nbx[1], idxbx1, 1);
	d_print_mat(1, nbx[1], lbx1, 1);
	d_print_mat(1, nbx[1], ubx1, 1);
	int_print_mat(1, nbu[1], idxbu1, 1);
	d_print_mat(1, nbu[1], lbu1, 1);
	d_print_mat(1, nbu[1], ubu1, 1);
	int_print_mat(1, nbx[N], idxbxN, 1);
	d_print_mat(1, nbx[N], lbxN, 1);
	d_print_mat(1, nbx[N], ubxN, 1);
	// general constraints
	d_print_mat(1, ng[0], lg0, 1);
	d_print_mat(1, ng[0], ug0, 1);
	d_print_mat(ng[0], nu[0], D0, ng[0]);
	d_print_mat(ng[0], nx[0], C0, ng[0]);
	d_print_mat(1, ng[1], lg1, 1);
	d_print_mat(1, ng[1], ug1, 1);
	d_print_mat(ng[1], nu[1], D1, ng[1]);
	d_print_mat(ng[1], nx[1], C1, ng[1]);
	d_print_mat(1, ng[N], lgN, 1);
	d_print_mat(1, ng[N], ugN, 1);
	d_print_mat(ng[N], nu[N], DN, ng[N]);
	d_print_mat(ng[N], nx[N], CN, ng[N]);
#endif

/************************************************
* quadratic constraints
************************************************/
	
	double *Rq1; d_zeros(&Rq1, nu[1], nu[1]*nq[1]);
	if(nq[1]>0)
		for(ii=0; ii<nu[1]; ii++)
			Rq1[ii*(nu[1]+1)] = 2*1.0;

	double *Rq2; d_zeros(&Rq2, nu[2], nu[2]*nq[2]);
	if(nq[2]>0)
		for(ii=0; ii<nu[2]; ii++)
			Rq2[ii*(nu[2]+1)] = 2*1.0/2;

	double *Rq3; d_zeros(&Rq3, nu[3], nu[3]*nq[3]);
	if(nq[3]>0)
		for(ii=0; ii<nu[3]; ii++)
			Rq3[ii*(nu[3]+1)] = 2*1.0/3;

	double *Qq1; d_zeros(&Qq1, nx[1], nx[1]*nq[1]);
//	for(ii=0; ii<nx[1]/2; ii++)
//		Qq1[(nx[1]/2+ii)*(nx[1]+1)] = 0.0;
//	d_print_mat(nx[1], nx[1], Qq1, nx[1]);

	double *qq1; d_zeros(&qq1, nx[1], nq[1]);
//	qq1[0*nx[1]+0] = -1;
//	qq1[1*nx[1]+0] =  1;

	double *uq1; d_zeros(&uq1, nq[1], 1);
	if(nq[1]>0)
		uq1[0] =  0.5*0.5;
//	uq1[0] =  4.0;
//	uq1[1] =  4.0;

	double *uq1_mask; d_zeros(&uq1_mask, nq[1], 1);
//	uq1_mask[0] = 1.0;
//	uq1_mask[1] = 1.0;


//	double *QqNm1; d_zeros(&QqNm1, nx[N-1], nx[N-1]*nq[N-1]);
//	for(ii=0; ii<nx[N-1]; ii++)
//		QqNm1[ii*(nx[N-1]+1)] = 1.0;

//	double *qqNm1; d_zeros(&qqNm1, nx[N-1], nq[N-1]);
//	qqN[0*nx[N]+0] = 0.0;

//	double *uqNm1; d_zeros(&uqNm1, nq[N-1], 1);
//	uqNm1[0] = 1.5;

//	double *uqNm1_mask; d_zeros(&uqNm1_mask, nq[N-1], 1);
//	uqNm1_mask[0] = 1.0;


	double *QqN; d_zeros(&QqN, nx[N], nx[N]*nq[N]);
	if(nq[N]>0)
		for(ii=0; ii<nx[N]; ii++)
			QqN[ii*(nx[N]+1)] = 1.0;

	double *qqN; d_zeros(&qqN, nx[N], nq[N]);
//	qqN[0*nx[N]+0] = 0.0;

	double *uqN; d_zeros(&uqN, nq[N], 1);
	if(nq[N]>0)
		uqN[0] = 1.5;

	double *uqN_mask; d_zeros(&uqN_mask, nq[N], 1);
	if(nq[N]>0)
		uqN_mask[0] = 1.0;

/************************************************
* soft constraints
************************************************/

	double *Zl0; d_zeros(&Zl0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zl0[ii] = 0e3;
	double *Zu0; d_zeros(&Zu0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zu0[ii] = 0e3;
	double *zl0; d_zeros(&zl0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		zl0[ii] = 1e2;
	double *zu0; d_zeros(&zu0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		zu0[ii] = 1e2;
	int *idxs0; int_zeros(&idxs0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		idxs0[ii] = nu[0]+ii;
	double *d_ls0; d_zeros(&d_ls0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		d_ls0[ii] = 0.0; //-1.0;
	double *d_us0; d_zeros(&d_us0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		d_us0[ii] = 0.0;

	double *Zl1; d_zeros(&Zl1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zl1[ii] = 0e3;
	double *Zu1; d_zeros(&Zu1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zu1[ii] = 0e3;
	double *zl1; d_zeros(&zl1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		zl1[ii] = 1e2;
	double *zu1; d_zeros(&zu1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		zu1[ii] = 1e2;
	int *idxs1; int_zeros(&idxs1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		idxs1[ii] = nu[1]+ii;
	double *d_ls1; d_zeros(&d_ls1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		d_ls1[ii] = 0.0; //-1.0;
	double *d_us1; d_zeros(&d_us1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		d_us1[ii] = 0.0;

	double *ZlN; d_zeros(&ZlN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		ZlN[ii] = 0e3;
	double *ZuN; d_zeros(&ZuN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		ZuN[ii] = 0e3;
	double *zlN; d_zeros(&zlN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		zlN[ii] = 1e2;
	double *zuN; d_zeros(&zuN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		zuN[ii] = 1e2;
	int *idxsN; int_zeros(&idxsN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		idxsN[ii] = nu[N]+ii;
	double *d_lsN; d_zeros(&d_lsN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		d_lsN[ii] = 0.0; //-1.0;
	double *d_usN; d_zeros(&d_usN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		d_usN[ii] = 0.0;

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
* ocp qp dim
************************************************/

	hpipm_size_t ocp_dim_size = d_ocp_qcqp_dim_memsize(N);
#if PRINT
	printf("\ndim size = %d\n", ocp_dim_size);
#endif
	void *ocp_dim_mem = malloc(ocp_dim_size);

	struct d_ocp_qcqp_dim ocp_dim;
	d_ocp_qcqp_dim_create(N, &ocp_dim, ocp_dim_mem);
	
	for(ii=0; ii<=N; ii++)
		{
		d_ocp_qcqp_dim_set_nx(ii, nx[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nu(ii, nu[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nbx(ii, nbx[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nbu(ii, nbu[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_ng(ii, ng[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nq(ii, nq[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_ns(ii, ns[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nsbx(ii, nsbx[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nsbu(ii, nsbu[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nsg(ii, nsg[ii], &ocp_dim);
		d_ocp_qcqp_dim_set_nsq(ii, nsq[ii], &ocp_dim);
		}

#if PRINT
	d_ocp_qcqp_dim_print(&ocp_dim);
#endif

/************************************************
* ocp qp
************************************************/

	hpipm_size_t ocp_qp_size = d_ocp_qcqp_memsize(&ocp_dim);
#if PRINT
	printf("\nqp size = %d\n", ocp_qp_size);
#endif
	void *ocp_qp_mem = malloc(ocp_qp_size);

	struct d_ocp_qcqp ocp_qp;
	d_ocp_qcqp_create(&ocp_dim, &ocp_qp, ocp_qp_mem);

	// dynamics
	ii = 0;
#if KEEP_X0
	d_ocp_qcqp_set_A(ii, A, &ocp_qp);
	d_ocp_qcqp_set_B(ii, B, &ocp_qp);
	d_ocp_qcqp_set_b(ii, b, &ocp_qp);
#else
	d_ocp_qcqp_set_B(ii, B, &ocp_qp);
	d_ocp_qcqp_set_b(ii, b0, &ocp_qp);
#endif
	for(ii=1; ii<N; ii++)
		{
		d_ocp_qcqp_set_A(ii, A, &ocp_qp);
		d_ocp_qcqp_set_B(ii, B, &ocp_qp);
		d_ocp_qcqp_set_b(ii, b, &ocp_qp);
		}

	// cost
	ii = 0;
#if KEEP_X0
	d_ocp_qcqp_set_Q(ii, Q, &ocp_qp);
	d_ocp_qcqp_set_S(ii, S, &ocp_qp);
	d_ocp_qcqp_set_R(ii, R, &ocp_qp);
	d_ocp_qcqp_set_q(ii, q, &ocp_qp);
	d_ocp_qcqp_set_r(ii, r, &ocp_qp);
#else
	d_ocp_qcqp_set_R(ii, R, &ocp_qp);
	d_ocp_qcqp_set_r(ii, r0, &ocp_qp);
#endif
	for(ii=1; ii<N; ii++)
		{
		d_ocp_qcqp_set_Q(ii, Q, &ocp_qp);
		d_ocp_qcqp_set_S(ii, S, &ocp_qp);
		d_ocp_qcqp_set_R(ii, R, &ocp_qp);
		d_ocp_qcqp_set_q(ii, q, &ocp_qp);
		d_ocp_qcqp_set_r(ii, r, &ocp_qp);
		}
	ii = N;
	d_ocp_qcqp_set_Q(ii, Q, &ocp_qp);
	d_ocp_qcqp_set_q(ii, q, &ocp_qp);
	
	// constraints
	ii = 0;
	d_ocp_qcqp_set_idxbx(ii, idxbx0, &ocp_qp);
#if KEEP_X0
	d_ocp_qcqp_set_lbx(ii, x0, &ocp_qp);
	d_ocp_qcqp_set_ubx(ii, x0, &ocp_qp);
#else
	d_ocp_qcqp_set_lbx(ii, lbx0, &ocp_qp);
	d_ocp_qcqp_set_ubx(ii, ubx0, &ocp_qp);
#endif
	d_ocp_qcqp_set_idxbu(ii, idxbu0, &ocp_qp);
	d_ocp_qcqp_set_lbu(ii, lbu0, &ocp_qp);
	d_ocp_qcqp_set_ubu(ii, ubu0, &ocp_qp);
	d_ocp_qcqp_set_C(ii, C0, &ocp_qp);
	d_ocp_qcqp_set_D(ii, D0, &ocp_qp);
	d_ocp_qcqp_set_lg(ii, lg0, &ocp_qp);
	d_ocp_qcqp_set_ug(ii, ug0, &ocp_qp);
	d_ocp_qcqp_set_Rq(ii, Rq1, &ocp_qp);
	d_ocp_qcqp_set_uq(ii, uq1, &ocp_qp);
	for(ii=1; ii<N; ii++)
		{
		d_ocp_qcqp_set_idxbx(ii, idxbx1, &ocp_qp);
		d_ocp_qcqp_set_lbx(ii, lbx1, &ocp_qp);
		d_ocp_qcqp_set_ubx(ii, ubx1, &ocp_qp);
		d_ocp_qcqp_set_idxbu(ii, idxbu1, &ocp_qp);
		d_ocp_qcqp_set_lbu(ii, lbu1, &ocp_qp);
		d_ocp_qcqp_set_ubu(ii, ubu1, &ocp_qp);
		d_ocp_qcqp_set_C(ii, C1, &ocp_qp);
		d_ocp_qcqp_set_D(ii, D1, &ocp_qp);
		d_ocp_qcqp_set_lg(ii, lg1, &ocp_qp);
		d_ocp_qcqp_set_ug(ii, ug1, &ocp_qp);
		d_ocp_qcqp_set_Rq(ii, Rq1, &ocp_qp);
		d_ocp_qcqp_set_Qq(ii, Qq1, &ocp_qp);
		d_ocp_qcqp_set_qq(ii, qq1, &ocp_qp);
		d_ocp_qcqp_set_uq(ii, uq1, &ocp_qp);
//		d_ocp_qcqp_set_uq_mask(ii, uq1_mask, &ocp_qp);
		}
//if(2<N)
//	d_ocp_qcqp_set_Rq(2, Rq2, &ocp_qp);
//if(3<N)
//	d_ocp_qcqp_set_Rq(3, Rq3, &ocp_qp);
//d_ocp_qcqp_set_Qq(N-1, QqNm1, &ocp_qp);
//d_ocp_qcqp_set_uq(N-1, uqNm1, &ocp_qp);
	ii = N;
	d_ocp_qcqp_set_idxbx(ii, idxbxN, &ocp_qp);
	d_ocp_qcqp_set_lbx(ii, lbxN, &ocp_qp);
	d_ocp_qcqp_set_ubx(ii, ubxN, &ocp_qp);
	d_ocp_qcqp_set_C(ii, CN, &ocp_qp);
	d_ocp_qcqp_set_lg(ii, lgN, &ocp_qp);
	d_ocp_qcqp_set_ug(ii, ugN, &ocp_qp);
	d_ocp_qcqp_set_Qq(ii, QqN, &ocp_qp);
	d_ocp_qcqp_set_qq(ii, qqN, &ocp_qp);
	d_ocp_qcqp_set_uq(ii, uqN, &ocp_qp);
	d_ocp_qcqp_set_uq_mask(ii, uqN_mask, &ocp_qp);

	// dynamic constraints removal
	double *lbu_mask; d_zeros(&lbu_mask, nbu[0], 1);
	double *ubu_mask; d_zeros(&ubu_mask, nbu[0], 1);
	double *lbx_mask; d_zeros(&lbx_mask, nbx[0], 1);
	double *ubx_mask; d_zeros(&ubx_mask, nbx[0], 1);
//	d_ocp_qp_set("lbu_mask", 0, lbu_mask, &qp);
//	d_ocp_qp_set("ubu_mask", 0, ubu_mask, &qp);
//	d_ocp_qp_set("lbx_mask", N, lbx_mask, &qp);
//	d_ocp_qp_set("ubx_mask", N, ubx_mask, &qp);

#if PRINT
	d_ocp_qcqp_print(&ocp_dim, &ocp_qp);
#endif

/************************************************
* part dense qp dim
************************************************/

	int N2 = 2; // horizon of partially condensed problem

	hpipm_size_t ocp_dim_size2 = d_ocp_qcqp_dim_memsize(N2);
#if PRINT
	printf("\ndim size2 = %d\n", ocp_dim_size2);
#endif
	void *ocp_dim_mem2 = malloc(ocp_dim_size2);

	struct d_ocp_qcqp_dim ocp_dim2;
	d_ocp_qcqp_dim_create(N2, &ocp_dim2, ocp_dim_mem2);

	hpipm_size_t block_size[N2+1];
#if 1
	d_part_cond_qcqp_compute_block_size(N, N2, block_size);
#else
	block_size[0] = 1;
	block_size[1] = 1;
	block_size[2] = 3;
#endif
#if PRINT
	printf("\nblock_size\n");
	int_print_mat(1, N2+1, block_size, 1);
#endif

	d_part_cond_qcqp_compute_dim(&ocp_dim, block_size, &ocp_dim2);

#if PRINT
	d_ocp_qcqp_dim_print(&ocp_dim2);
#endif

/************************************************
* part dense qp
************************************************/

	// qp
	hpipm_size_t ocp_qp_size2 = d_ocp_qcqp_memsize(&ocp_dim2);
#if PRINT
	printf("\npart dense qp size = %d\n", ocp_qp_size2);
#endif
	void *ocp_qp_mem2 = malloc(ocp_qp_size2);

	struct d_ocp_qcqp ocp_qp2;
	d_ocp_qcqp_create(&ocp_dim2, &ocp_qp2, ocp_qp_mem2);

	// arg
	hpipm_size_t part_cond_arg_size = d_part_cond_qcqp_arg_memsize(ocp_dim2.N);
#if PRINT
	printf("\npart cond_arg size = %d\n", part_cond_arg_size);
#endif
	void *part_cond_arg_mem = malloc(part_cond_arg_size);

	struct d_part_cond_qcqp_arg part_cond_arg;
	d_part_cond_qcqp_arg_create(ocp_dim2.N, &part_cond_arg, part_cond_arg_mem);
	d_part_cond_qcqp_arg_set_default(&part_cond_arg);

//	for(ii=0; ii<=N2; ii++)
//		part_cond_arg.cond_arg[ii].square_root_alg = 0;

	// ws
	hpipm_size_t part_cond_size = d_part_cond_qcqp_ws_memsize(&ocp_dim, block_size, &ocp_dim2, &part_cond_arg);
#if PRINT
	printf("\npart cond size = %d\n", part_cond_size);
#endif
	void *part_cond_mem = malloc(part_cond_size);

	struct d_part_cond_qcqp_ws part_cond_ws;
	d_part_cond_qcqp_ws_create(&ocp_dim, block_size, &ocp_dim2, &part_cond_arg, &part_cond_ws, part_cond_mem);

	/* part cond */

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_part_cond_qcqp_cond(&ocp_qp, &ocp_qp2, &part_cond_arg, &part_cond_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	d_ocp_qcqp_print(&ocp_dim2, &ocp_qp2);
#endif

	/* part cond rhs */

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_part_cond_qcqp_cond_rhs(&ocp_qp, &ocp_qp2, &part_cond_arg, &part_cond_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_cond_rhs = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	/* part cond lhs */

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_part_cond_qcqp_cond_lhs(&ocp_qp, &ocp_qp2, &part_cond_arg, &part_cond_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_cond_lhs = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* part dense qp sol
************************************************/

	hpipm_size_t ocp_qp_sol_size2 = d_ocp_qcqp_sol_memsize(&ocp_dim2);
#if PRINT
	printf("\npart dense qp sol size = %d\n", ocp_qp_sol_size2);
#endif
	void *ocp_qp_sol_mem2 = malloc(ocp_qp_sol_size2);

	struct d_ocp_qcqp_sol ocp_qp_sol2;
	d_ocp_qcqp_sol_create(&ocp_dim2, &ocp_qp_sol2, ocp_qp_sol_mem2);

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_ocp_qcqp_ipm_arg_memsize(&ocp_dim2);
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_ocp_qcqp_ipm_arg arg;
	d_ocp_qcqp_ipm_arg_create(&ocp_dim2, &arg, ipm_arg_mem);

//	enum hpipm_mode mode = SPEED_ABS;
	enum hpipm_mode mode = SPEED;
//	enum hpipm_mode mode = BALANCE;
//	enum hpipm_mode mode = ROBUST;
	d_ocp_qcqp_ipm_arg_set_default(mode, &arg);

	mu0 = 10;
	int iter_max = 30;
	double alpha_min = 1e-8;
	double tol_stat = 1e-8;
	double tol_eq = 1e-12;
	double tol_ineq = 1e-12;
	double tol_comp = 1e-12;
	double reg_prim = 1e-12;
	int warm_start = 0;
	int pred_corr = 1;
	int ric_alg = 1;

	d_ocp_qcqp_ipm_arg_set_mu0(&mu0, &arg);
	d_ocp_qcqp_ipm_arg_set_iter_max(&iter_max, &arg);
	d_ocp_qcqp_ipm_arg_set_alpha_min(&alpha_min, &arg);
	d_ocp_qcqp_ipm_arg_set_tol_stat(&tol_stat, &arg);
	d_ocp_qcqp_ipm_arg_set_tol_eq(&tol_eq, &arg);
	d_ocp_qcqp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
	d_ocp_qcqp_ipm_arg_set_tol_comp(&tol_comp, &arg);
	d_ocp_qcqp_ipm_arg_set_reg_prim(&reg_prim, &arg);
	d_ocp_qcqp_ipm_arg_set_warm_start(&warm_start, &arg);
	d_ocp_qcqp_ipm_arg_set_pred_corr(&pred_corr, &arg);
	d_ocp_qcqp_ipm_arg_set_ric_alg(&ric_alg, &arg);

/************************************************
* ipm
************************************************/

	hpipm_size_t ipm_size = d_ocp_qcqp_ipm_ws_memsize(&ocp_dim2, &arg);
#if PRINT
	printf("\nipm size = %d\n", ipm_size);
#endif
	void *ipm_mem = malloc(ipm_size);

	struct d_ocp_qcqp_ipm_ws workspace;
	d_ocp_qcqp_ipm_ws_create(&ocp_dim2, &arg, &workspace, ipm_mem);

	int hpipm_status; // 0 normal; 1 max iter

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_ocp_qcqp_ipm_solve(&ocp_qp2, &ocp_qp_sol2, &arg, &workspace);
		d_ocp_qcqp_ipm_get_status(&workspace, &hpipm_status);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_ocp_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	d_ocp_qcqp_sol_print(&ocp_dim2, &ocp_qp_sol2);
#endif

/************************************************
* print ipm statistics
************************************************/

	int iter; d_ocp_qcqp_ipm_get_iter(&workspace, &iter);
	double res_stat; d_ocp_qcqp_ipm_get_max_res_stat(&workspace, &res_stat);
	double res_eq; d_ocp_qcqp_ipm_get_max_res_eq(&workspace, &res_eq);
	double res_ineq; d_ocp_qcqp_ipm_get_max_res_ineq(&workspace, &res_ineq);
	double res_comp; d_ocp_qcqp_ipm_get_max_res_comp(&workspace, &res_comp);
	double *stat; d_ocp_qcqp_ipm_get_stat(&workspace, &stat);
	int stat_m; d_ocp_qcqp_ipm_get_stat_m(&workspace, &stat_m);

#if PRINT
	printf("\nipm return = %d\n", hpipm_status);
	printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

	printf("\nipm iter = %d\n", iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
	d_print_exp_tran_mat(stat_m, iter+1, stat, stat_m);

	printf("\npart cond time         = %e [s]\n", time_cond);
//	printf("\nupdate part cond time  = %e [s]\n", time_update_cond);
	printf("\npart cond lhs time     = %e [s]\n", time_cond_lhs);
	printf("\npart cond rhs time     = %e [s]\n", time_cond_rhs);
	printf("\npart cond ocp ipm time = %e [s]\n\n", time_ocp_ipm);
#endif

/************************************************
* full space ocp qp sol
************************************************/

	hpipm_size_t ocp_qp_sol_size = d_ocp_qcqp_sol_memsize(&ocp_dim);
#if PRINT
	printf("\nocp qp sol size = %d\n", ocp_qp_sol_size);
#endif
	void *ocp_qp_sol_mem = malloc(ocp_qp_sol_size);

	struct d_ocp_qcqp_sol ocp_qp_sol;
	d_ocp_qcqp_sol_create(&ocp_dim, &ocp_qp_sol, ocp_qp_sol_mem);

/************************************************
* expand solution
************************************************/

	d_part_cond_qcqp_expand_sol(&ocp_qp, &ocp_qp2, &ocp_qp_sol2, &ocp_qp_sol, &part_cond_arg, &part_cond_ws);

#if PRINT
	d_ocp_qcqp_sol_print(&ocp_dim, &ocp_qp_sol);
#endif

/************************************************
* free memory
************************************************/

	d_free(A);
	d_free(B);
	d_free(b);
	d_free(x0);
	d_free(Q);
//	d_free(QN);
	d_free(R);
	d_free(S);
	d_free(q);
//	d_free(qN);
	d_free(r);
	d_free(r0);
	int_free(idxbx0);
	d_free(lbx0);
	d_free(ubx0);
	int_free(idxbu0);
	d_free(lbu0);
	d_free(ubu0);
	int_free(idxbx1);
	d_free(lbx1);
	d_free(ubx1);
	int_free(idxbu1);
	d_free(lbu1);
	d_free(ubu1);
	int_free(idxbxN);
	d_free(lbxN);
	d_free(ubxN);
	d_free(C0);
	d_free(D0);
	d_free(lg0);
	d_free(ug0);
	d_free(C1);
	d_free(D1);
	d_free(lg1);
	d_free(ug1);
	d_free(CN);
	d_free(DN);
	d_free(lgN);
	d_free(ugN);
	d_free(Rq1);
	d_free(Rq2);
	d_free(Rq3);
	d_free(Qq1);
	d_free(QqN);
	d_free(qqN);
	d_free(uq1);
	d_free(uqN);
	d_free(uq1_mask);
	d_free(uqN_mask);
	d_free(lbu_mask);
	d_free(ubu_mask);
	d_free(lbx_mask);
	d_free(ubx_mask);

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

	free(ocp_dim_mem);
	free(ocp_qp_mem);
	free(ocp_dim_mem2);
//	free(ocp_qp_sol_mem);
//	free(dense_dim_mem);
//	free(dense_qp_mem);
//	free(dense_qp_sol_mem);
//	free(cond_mem);
//	free(cond_arg_mem);
//	free(dense_ipm_mem);

/************************************************
* return
************************************************/

	return hpipm_status;

	}
