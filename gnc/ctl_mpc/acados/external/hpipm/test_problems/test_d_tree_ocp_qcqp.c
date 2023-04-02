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

#include <hpipm_tree.h>
#include <hpipm_scenario_tree.h>
#include <hpipm_d_tree_ocp_qcqp_dim.h>
#include <hpipm_d_tree_ocp_qcqp.h>
#include <hpipm_d_tree_ocp_qcqp_sol.h>
#include <hpipm_d_tree_ocp_qcqp_res.h>
#include <hpipm_d_tree_ocp_qcqp_ipm.h>
#include <hpipm_d_tree_ocp_qcqp_utils.h>

#include "d_tools.h"



#define KEEP_X0 0

// printing
#ifndef PRINT
#define PRINT 1
#endif



#if ! defined(EXT_DEP)
/* creates a zero matrix */
void d_zeros(double **pA, int row, int col)
	{
	*pA = malloc((row*col)*sizeof(double));
	double *A = *pA;
	int i;
	for(i=0; i<row*col; i++) A[i] = 0.0;
	}
/* frees matrix */
void d_free(double *pA)
	{
	free( pA );
	}
/* prints a matrix in column-major format */
void d_print_mat(int m, int n, double *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
/* prints the transposed of a matrix in column-major format */
void d_print_tran_mat(int row, int col, double *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
/* prints a matrix in column-major format (exponential notation) */
void d_print_exp_mat(int m, int n, double *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
/* prints the transposed of a matrix in column-major format (exponential notation) */
void d_print_exp_tran_mat(int row, int col, double *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
/* creates a zero matrix aligned */
void int_zeros(int **pA, int row, int col)
	{
	void *temp = malloc((row*col)*sizeof(int));
	*pA = temp;
	int *A = *pA;
	int i;
	for(i=0; i<row*col; i++) A[i] = 0;
	}
/* frees matrix */
void int_free(int *pA)
	{
	free( pA );
	}
/* prints a matrix in column-major format */
void int_print_mat(int row, int col, int *A, int lda)
	{
	int i, j;
	for(i=0; i<row; i++)
		{
		for(j=0; j<col; j++)
			{
			printf("%d ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
#endif



/************************************************
Mass-spring system: nx/2 masses connected each other with springs (in a row), and the first and the last one to walls. nu (<=nx) controls act on the first nu masses. The system is sampled with sampling time Ts.
************************************************/
void mass_spring_system(double Ts, int nx, int nu, double *A, double *B, double *b, double *x0)
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

	int ii, jj;
	int stage;

	int nx_ = 2;
	int nu_ = 1;

	int md = 1;
	int Nr = 1;
	int Nh = 15;

	// stage-wise size
	int nx[Nh+1];
	int nu[Nh+1];
	int nbx[Nh+1];
	int nbu[Nh+1];
	int nb[Nh+1];
	int ng[Nh+1];
	int nq[Nh+1];
	int ns[Nh+1];
	int nsbx[Nh+1];
	int nsbu[Nh+1];
	int nsg[Nh+1];
	int nsq[Nh+1];

	nx[0] = 0;
	nu[0] = nu_;
	nbx[0] = 0;
	nbu[0] = 0;
	nb[0] = nbx[0] + nbu[0];
	ng[0] = 0;
	nq[0] = 1;
	ns[0] = 0;
	nsbx[0] = 0;
	nsbu[0] = 0;
	nsg[0] = 0;
	nsq[0] = 0;
	for(ii=1; ii<Nh; ii++)
		{
		nx[ii] = nx_;
		nu[ii] = nu_;
		nbx[ii] = 0;
		nbu[ii] = 0;
		nb[ii] = nbx[ii] + nbu[ii];
		ng[ii] = 0;
		nq[ii] = 1;
		nsbx[ii] = 0;
		nsbu[ii] = 0;
		nsg[ii] = 0;
		nsq[ii] = 0;
		ns[ii] = nsbx[ii]+nsbu[ii]+nsg[ii]+nsq[ii];
		}
	nx[Nh] = nx_;
	nu[Nh] = 0;
	nbx[Nh] = 0;
	nbu[Nh] = 0;
	nb[Nh] = nbx[Nh] + nbu[Nh];
	ng[Nh] = 0;
	nq[Nh] = 1;
	nsbx[Nh] = 0;
	nsbu[Nh] = 0;
	nsg[Nh] = 0;
	nsq[Nh] = 0;
	ns[Nh] = nsbx[Nh]+nsbu[Nh]+nsg[Nh]+nsq[Nh];

/************************************************
* dynamical system
************************************************/

	double *A; d_zeros(&A, nx_, nx_); // states update matrix

	double *B; d_zeros(&B, nx_, nu_); // inputs matrix

	double *b; d_zeros(&b, nx_, 1); // states offset
	double *x0; d_zeros(&x0, nx_, 1); // initial state

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx_, nu_, A, B, b, x0);

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


	int *idxbxN; int_zeros(&idxbxN, nbx[Nh], 1);
	double *lbxN; d_zeros(&lbxN, nbx[Nh], 1);
	double *ubxN; d_zeros(&ubxN, nbx[Nh], 1);
	double *lgN; d_zeros(&lgN, ng[Nh], 1);
	double *ugN; d_zeros(&ugN, ng[Nh], 1);
	for(ii=0; ii<nbx[Nh]; ii++)
		{
		lbxN[ii] = - 4.0; // xmin
		ubxN[ii] =   4.0; // xmax
		idxbxN[ii] = ii;
		}
	for(ii=0; ii<ng[Nh]; ii++)
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

	double *CN; d_zeros(&CN, ng[Nh], nx[Nh]);
	double *DN; d_zeros(&DN, ng[Nh], nu[Nh]);
	for(ii=0; ii<nu[Nh]-nb[Nh] & ii<ng[Nh]; ii++)
		DN[ii+(nb[Nh]+ii)*ng[Nh]] = 1.0;
	for(; ii<ng[Nh]; ii++)
		CN[ii+(nb[Nh]+ii-nu[Nh])*ng[Nh]] = 1.0;

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
	int_print_mat(1, nbx[Nh], idxbxN, 1);
	d_print_mat(1, nbx[Nh], lbxN, 1);
	d_print_mat(1, nbx[Nh], ubxN, 1);
	// general constraints
	d_print_mat(1, ng[0], lg0, 1);
	d_print_mat(1, ng[0], ug0, 1);
	d_print_mat(ng[0], nu[0], D0, ng[0]);
	d_print_mat(ng[0], nx[0], C0, ng[0]);
	d_print_mat(1, ng[1], lg1, 1);
	d_print_mat(1, ng[1], ug1, 1);
	d_print_mat(ng[1], nu[1], D1, ng[1]);
	d_print_mat(ng[1], nx[1], C1, ng[1]);
	d_print_mat(1, ng[Nh], lgN, 1);
	d_print_mat(1, ng[Nh], ugN, 1);
	d_print_mat(ng[Nh], nu[Nh], DN, ng[Nh]);
	d_print_mat(ng[Nh], nx[Nh], CN, ng[Nh]);
#endif

/************************************************
* quadratic constraints
************************************************/

	double *Qq0; d_zeros(&Qq0, nx[0], nx[0]*nq[0]);

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
	if(nq[1]>0)
		uq1_mask[0] = 1.0;
//	uq1_mask[1] = 1.0;


//	double *QqNm1; d_zeros(&QqNm1, nx[Nh-1], nx[Nh-1]*nq[Nh-1]);
//	for(ii=0; ii<nx[Nh-1]; ii++)
//		QqNm1[ii*(nx[Nh-1]+1)] = 1.0;

//	double *qqNm1; d_zeros(&qqNm1, nx[Nh-1], nq[Nh-1]);
//	qqN[0*nx[Nh]+0] = 0.0;

//	double *uqNm1; d_zeros(&uqNm1, nq[Nh-1], 1);
//	uqNm1[0] = 1.5;

//	double *uqNm1_mask; d_zeros(&uqNm1_mask, nq[Nh-1], 1);
//	uqNm1_mask[0] = 1.0;


	double *RqN; d_zeros(&RqN, nu[Nh], nu[Nh]*nq[Nh]);

	double *QqN; d_zeros(&QqN, nx[Nh], nx[Nh]*nq[Nh]);
	if(nq[Nh]>0)
		for(ii=0; ii<nx[Nh]; ii++)
			QqN[ii*(nx[Nh]+1)] = 1.0;

	double *qqN; d_zeros(&qqN, nx[Nh], nq[Nh]);
//	qqN[0*nx[Nh]+0] = 0.0;

	double *uqN; d_zeros(&uqN, nq[Nh], 1);
	if(nq[Nh]>0)
		uqN[0] = 1.5;

	double *uqN_mask; d_zeros(&uqN_mask, nq[Nh], 1);
	if(nq[Nh]>0)
		uqN_mask[0] = 1.0;

/************************************************
* soft constraints
************************************************/

#if 0
	double *Zl0; d_zeros(&Zl0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zl0[ii] = 1e3;
	double *Zu0; d_zeros(&Zu0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zu0[ii] = 1e3;
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
		d_ls0[ii] = 0.0;
	double *d_us0; d_zeros(&d_us0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		d_us0[ii] = 0.0;

	double *Zl1; d_zeros(&Zl1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zl1[ii] = 1e3;
	double *Zu1; d_zeros(&Zu1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zu1[ii] = 1e3;
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
		d_ls1[ii] = 0.0;
	double *d_us1; d_zeros(&d_us1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		d_us1[ii] = 0.0;

	double *ZlN; d_zeros(&ZlN, ns[Nh], 1);
	for(ii=0; ii<ns[Nh]; ii++)
		ZlN[ii] = 1e3;
	double *ZuN; d_zeros(&ZuN, ns[Nh], 1);
	for(ii=0; ii<ns[Nh]; ii++)
		ZuN[ii] = 1e3;
	double *zlN; d_zeros(&zlN, ns[Nh], 1);
	for(ii=0; ii<ns[Nh]; ii++)
		zlN[ii] = 1e2;
	double *zuN; d_zeros(&zuN, ns[Nh], 1);
	for(ii=0; ii<ns[Nh]; ii++)
		zuN[ii] = 1e2;
	int *idxsN; int_zeros(&idxsN, ns[Nh], 1);
	for(ii=0; ii<ns[Nh]; ii++)
		idxsN[ii] = nu[Nh]+ii;
	double *d_lsN; d_zeros(&d_lsN, ns[Nh], 1);
	for(ii=0; ii<ns[Nh]; ii++)
		d_lsN[ii] = 0.0;
	double *d_usN; d_zeros(&d_usN, ns[Nh], 1);
	for(ii=0; ii<ns[Nh]; ii++)
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
	int_print_mat(1, ns[Nh], idxsN, 1);
	d_print_mat(1, ns[Nh], ZlN, 1);
	d_print_mat(1, ns[Nh], ZuN, 1);
	d_print_mat(1, ns[Nh], zlN, 1);
	d_print_mat(1, ns[Nh], zuN, 1);
	d_print_mat(1, ns[Nh], d_lsN, 1);
	d_print_mat(1, ns[Nh], d_usN, 1);
#endif
#endif

/************************************************
* create scenario tree
************************************************/

	hpipm_size_t tree_memsize = sctree_memsize(md, Nr, Nh);
#if PRINT
	printf("\ntree memsize = %d\n", tree_memsize);
#endif
	void *tree_memory = malloc(tree_memsize);

	struct sctree st;
	sctree_create(md, Nr, Nh, &st, tree_memory);

	int Nn = st.Nn;

#if 0
	int Nn = st.Nn;
	printf("\nscenario tree\n");
	for(ii=0; ii<Nn; ii++)
		{
		printf("\n");
		printf("idx = %d\n", (st.root+ii)->idx);
		printf("stage = %d\n", (st.root+ii)->stage);
		printf("real = %d\n", (st.root+ii)->real);
		printf("idxkid = %d\n", (st.root+ii)->idxkid);
		printf("dad = %d\n", (st.root+ii)->dad);
		printf("nkids = %d\n", (st.root+ii)->nkids);
		printf("kids =");
		for(jj=0; jj<(st.root+ii)->nkids; jj++)
			printf(" %d", (st.root+ii)->kids[jj]);
		printf("\n\n");
		}
#endif

/************************************************
* cast scenario tree into tree
************************************************/

	struct tree ttree;
	sctree_cast_to_tree(&st, &ttree);

#if 0
	Nn = ttree.Nn;
	printf("\ntree\n");
	for(ii=0; ii<Nn; ii++)
		{
		printf("\n");
		printf("idx = %d\n", (ttree.root+ii)->idx);
		printf("stage = %d\n", (ttree.root+ii)->stage);
		printf("real = %d\n", (ttree.root+ii)->real);
		printf("idxkid = %d\n", (ttree.root+ii)->idxkid);
		printf("dad = %d\n", (ttree.root+ii)->dad);
		printf("nkids = %d\n", (ttree.root+ii)->nkids);
		printf("kids =");
		for(jj=0; jj<(ttree.root+ii)->nkids; jj++)
			printf(" %d", (ttree.root+ii)->kids[jj]);
		printf("\n\n");
		}
#endif

/************************************************
* tree ocp problem size
************************************************/

	// node-wise size
	int nxt[Nn];
	int nut[Nn];
	int nbxt[Nn];
	int nbut[Nn];
	int nbt[Nn];
	int ngt[Nn];
	int nqt[Nn];
	int nst[Nn];
	int nsbxt[Nn];
	int nsbut[Nn];
	int nsgt[Nn];
	int nsqt[Nn];

	for(ii=0; ii<Nn; ii++)
		{
		stage = (ttree.root+ii)->stage;
		nxt[ii] = nx[stage];
		nut[ii] = nu[stage];
		nbxt[ii] = nbx[stage];
		nbut[ii] = nbu[stage];
		nbt[ii] = nb[stage];
		ngt[ii] = ng[stage];
		nqt[ii] = nq[stage];
		nst[ii] = ns[stage];
		nsbxt[ii] = nsbx[stage];
		nsbut[ii] = nsbu[stage];
		nsgt[ii] = nsg[stage];
		nsqt[ii] = nsg[stage];
		}

#if 0
	for(ii=0; ii<Nn; ii++)
		{
		printf("\n%d %d %d %d\n", nxt[ii], nut[ii], nbt[ii], ngt[ii]);
		}
#endif

/************************************************
* tree ocp data
************************************************/

	// stage-wise data

	double *hA[Nh];
	double *hB[Nh];
	double *hb[Nh];
	double *hQ[Nh+1];
	double *hS[Nh+1];
	double *hR[Nh+1];
	double *hq[Nh+1];
	double *hr[Nh+1];
	int *hidxbx[Nh+1];
	double *hlbx[Nh+1];
	double *hubx[Nh+1];
	int *hidxbu[Nh+1];
	double *hlbu[Nh+1];
	double *hubu[Nh+1];
	double *hC[Nh+1];
	double *hD[Nh+1];
	double *hlg[Nh+1];
	double *hug[Nh+1];
	double *hQq[Nh+1];
//	double *hSq[Nh+1];
	double *hRq[Nh+1];
//	double *hqq[Nh+1];
//	double *hrq[Nh+1];
	double *huq[Nh+1];
//	double *hZl[Nh+1];
//	double *hZu[Nh+1];
//	double *hzl[Nh+1];
//	double *hzu[Nh+1];
//	int *hidxs[Nh+1];
//	double *hls[Nh+1];
//	double *hus[Nh+1];

	hA[0] = A;
	hB[0] = B;
	hb[0] = b0;
	hQ[0] = Q;
	hS[0] = S;
	hR[0] = R;
	hq[0] = q;
	hr[0] = r0;
	hidxbx[0] = idxbx0;
	hlbx[0] = lbx0;
	hubx[0] = ubx0;
	hidxbu[0] = idxbu0;
	hlbu[0] = lbu0;
	hubu[0] = ubu0;
	hC[0] = C0;
	hD[0] = D0;
	hlg[0] = lg0;
	hug[0] = ug0;
	hQq[0] = Qq0;
//	hSq[0] = S;
	hRq[0] = Rq1;
//	hqq[0] = q;
//	hrq[0] = r0;
	huq[0] = uq1;
//	hZl[0] = Zl0;
//	hZu[0] = Zu0;
//	hzl[0] = zl0;
//	hzu[0] = zu0;
//	hidxs[0] = idxs0;
//	hls[0] = ls0;
//	hus[0] = us0;
	for(ii=1; ii<Nh; ii++)
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
		hlbx[ii] = lbx1;
		hubx[ii] = ubx1;
		hidxbu[ii] = idxbu1;
		hlbu[ii] = lbu1;
		hubu[ii] = ubu1;
		hC[ii] = C1;
		hD[ii] = D1;
		hlg[ii] = lg1;
		hug[ii] = ug1;
		hQq[ii] = Qq1;
//		hSq[ii] = S;
		hRq[ii] = Rq1;
//		hqq[ii] = q;
//		hrq[ii] = r0;
		huq[ii] = uq1;
//		hZl[ii] = Zl1;
//		hZu[ii] = Zu1;
//		hzl[ii] = zl1;
//		hzu[ii] = zu1;
//		hidxs[ii] = idxs1;
//		hls[ii] = ls1;
//		hus[ii] = us1;
		}
	hQ[Nh] = Q;
	hS[Nh] = S;
	hR[Nh] = R;
	hq[Nh] = q;
	hr[Nh] = r;
	hidxbx[Nh] = idxbxN;
	hlbx[Nh] = lbxN;
	hubx[Nh] = ubxN;
	hC[Nh] = CN;
	hD[Nh] = DN;
	hlg[Nh] = lgN;
	hug[Nh] = ugN;
	hQq[Nh] = QqN;
//	hSq[Nh] = S;
	hRq[Nh] = RqN;
//	hqq[Nh] = q;
//	hrq[Nh] = r0;
	huq[Nh] = uqN;
//	hZl[Nh] = ZlN;
//	hZu[Nh] = ZuN;
//	hzl[Nh] = zlN;
//	hzu[Nh] = zuN;
//	hidxs[Nh] = idxsN;
//	hls[Nh] = lsN;
//	hus[Nh] = usN;

/************************************************
* create tree ocp qp dim
************************************************/

	hpipm_size_t dim_size = d_tree_ocp_qcqp_dim_memsize(Nn);
#if PRINT
	printf("\ndim size = %d\n", dim_size);
#endif
	void *dim_mem = malloc(dim_size);

	struct d_tree_ocp_qcqp_dim dim;
	d_tree_ocp_qcqp_dim_create(Nn, &dim, dim_mem);

//	d_tree_ocp_qcqp_dim_set_all(&ttree, nxt, nut, nbxt, nbut, ngt, nsbxt, nsbut, nsgt, &dim);
	
	d_tree_ocp_qcqp_dim_set_tree(&ttree, &dim);
	for(ii=0; ii<Nn; ii++)
		{
		d_tree_ocp_qcqp_dim_set_nx(ii, nxt[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nu(ii, nut[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nbx(ii, nbxt[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nbu(ii, nbut[ii], &dim);
		d_tree_ocp_qcqp_dim_set_ng(ii, ngt[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nq(ii, nqt[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nsbx(ii, nsbxt[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nsbu(ii, nsbut[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nsg(ii, nsgt[ii], &dim);
		d_tree_ocp_qcqp_dim_set_nsq(ii, nsqt[ii], &dim);
		}
	
#if PRINT
	d_tree_ocp_qcqp_dim_print(&dim);
//	exit(1);
#endif

/************************************************
* create tree ocp qp
************************************************/

	hpipm_size_t tree_ocp_qp_memsize = d_tree_ocp_qcqp_memsize(&dim);
#if PRINT
	printf("\ntree ocp qp memsize = %d\n", tree_ocp_qp_memsize);
#endif
	void *tree_ocp_qp_memory = malloc(tree_ocp_qp_memsize);

	struct d_tree_ocp_qcqp qp;
	d_tree_ocp_qcqp_create(&dim, &qp, tree_ocp_qp_memory);

//	d_tree_ocp_qcqp_set_all(hAt, hBt, hbt, hQt, hSt, hRt, hqt, hrt, hidxbt, hd_lbt, hd_ubt, hCt, hDt, hd_lgt, hd_ugt, hZlt, hZut, hzlt, hzut, hidxst, hd_lst, hd_ust, &qp);
	
	// node-wise data

	for(ii=0; ii<Nn-1; ii++)
		{
		stage = (ttree.root+ii+1)->stage-1;
		// dynamics
		d_tree_ocp_qcqp_set_A(ii, hA[stage], &qp);
		d_tree_ocp_qcqp_set_B(ii, hB[stage], &qp);
		d_tree_ocp_qcqp_set_b(ii, hb[stage], &qp);
		}

	for(ii=0; ii<Nn; ii++)
		{
		stage = (ttree.root+ii)->stage;
		// cost
		d_tree_ocp_qcqp_set_Q(ii, hQ[stage], &qp);
		d_tree_ocp_qcqp_set_S(ii, hS[stage], &qp);
		d_tree_ocp_qcqp_set_R(ii, hR[stage], &qp);
		d_tree_ocp_qcqp_set_q(ii, hq[stage], &qp);
		d_tree_ocp_qcqp_set_r(ii, hr[stage], &qp);
		// constraints
		d_tree_ocp_qcqp_set_idxbx(ii, hidxbx[stage], &qp);
		d_tree_ocp_qcqp_set_lbx(ii, hlbx[stage], &qp);
		d_tree_ocp_qcqp_set_ubx(ii, hubx[stage], &qp);
		d_tree_ocp_qcqp_set_idxbu(ii, hidxbu[stage], &qp);
		d_tree_ocp_qcqp_set_lbu(ii, hlbu[stage], &qp);
		d_tree_ocp_qcqp_set_ubu(ii, hubu[stage], &qp);
		d_tree_ocp_qcqp_set_C(ii, hC[stage], &qp);
		d_tree_ocp_qcqp_set_D(ii, hD[stage], &qp);
		d_tree_ocp_qcqp_set_lg(ii, hlg[stage], &qp);
		d_tree_ocp_qcqp_set_ug(ii, hug[stage], &qp);
		d_tree_ocp_qcqp_set_Qq(ii, hQq[stage], &qp);
//		d_tree_ocp_qcqp_set_Sq(ii, hSq[stage], &qp);
		d_tree_ocp_qcqp_set_Rq(ii, hRq[stage], &qp);
//		d_tree_ocp_qcqp_set_qq(ii, hqq[stage], &qp);
//		d_tree_ocp_qcqp_set_rq(ii, hrq[stage], &qp);
		d_tree_ocp_qcqp_set_uq(ii, huq[stage], &qp);
//		d_tree_ocp_qcqp_set_Zl(ii, hZlt[stage], &qp);
//		d_tree_ocp_qcqp_set_Zu(ii, hZut[stage], &qp);
//		d_tree_ocp_qcqp_set_zl(ii, hzlt[stage], &qp);
//		d_tree_ocp_qcqp_set_zu(ii, hzut[stage], &qp);
//		d_tree_ocp_qcqp_set_idxs(ii, hidxst[stage], &qp);
//		d_tree_ocp_qcqp_set_lls(ii, hlst[stage], &qp);
//		d_tree_ocp_qcqp_set_lus(ii, hust[stage], &qp);
		}

#if PRINT
	d_tree_ocp_qcqp_print(&dim, &qp);
//	exit(1);
#endif

/************************************************
* ocp qp sol
************************************************/

	hpipm_size_t tree_ocp_qp_sol_size = d_tree_ocp_qcqp_sol_memsize(&dim);
#if PRINT
	printf("\ntree ocp qp sol memsize = %d\n", tree_ocp_qp_sol_size);
#endif
	void *tree_ocp_qp_sol_memory = malloc(tree_ocp_qp_sol_size);

	struct d_tree_ocp_qcqp_sol qp_sol;
	d_tree_ocp_qcqp_sol_create(&dim, &qp_sol, tree_ocp_qp_sol_memory);

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_tree_ocp_qcqp_ipm_arg_memsize(&dim);
#if PRINT
	printf("\nipm arg size = %d\n", ipm_arg_size);
#endif
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_tree_ocp_qcqp_ipm_arg arg;
	d_tree_ocp_qcqp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
//	enum hpipm_mode mode = SPEED_ABS;
	enum hpipm_mode mode = SPEED;
//	enum hpipm_mode mode = BALANCE;
//	enum hpipm_mode mode = ROBUST;
	d_tree_ocp_qcqp_ipm_arg_set_default(mode, &arg);

//	arg.alpha_min = 1e-8;
//	arg.res_g_max = 1e-8;
//	arg.res_b_max = 1e-8;
//	arg.res_d_max = 1e-12;
//	arg.res_m_max = 1e-12;
//	arg.mu0 = 10.0;
//	arg.iter_max = 20;
//	arg.stat_max = 100;
//	arg.pred_corr = 1;

/************************************************
* ipm
************************************************/

	hpipm_size_t ipm_size = d_tree_ocp_qcqp_ipm_ws_memsize(&dim, &arg);
#if PRINT
	printf("\nipm size = %d\n", ipm_size);
#endif
	void *ipm_memory = malloc(ipm_size);

	struct d_tree_ocp_qcqp_ipm_ws workspace;
	d_tree_ocp_qcqp_ipm_ws_create(&dim, &arg, &workspace, ipm_memory);

	int hpipm_status; // 0 normal; 1 max iter

	int rep, nrep=100;

	struct timeval tv0, tv1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_tree_ocp_qcqp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
		d_tree_ocp_qcqp_ipm_get_status(&workspace, &hpipm_status);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_ocp_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* extract and print solution
************************************************/

	double *u[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(u+ii, nut[ii], 1);
	double *x[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(x+ii, nxt[ii], 1);
	double *ls[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(ls+ii, nst[ii], 1);
	double *us[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(us+ii, nst[ii], 1);
	double *pi[Nn-1]; for(ii=0; ii<Nn-1; ii++) d_zeros(pi+ii, nxt[ii+1], 1);
	double *lam_lb[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(lam_lb+ii, nbt[ii], 1);
	double *lam_ub[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(lam_ub+ii, nbt[ii], 1);
	double *lam_lg[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(lam_lg+ii, ngt[ii], 1);
	double *lam_ug[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(lam_ug+ii, ngt[ii], 1);
	double *lam_ls[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(lam_ls+ii, nst[ii], 1);
	double *lam_us[Nn]; for(ii=0; ii<Nn; ii++) d_zeros(lam_us+ii, nst[ii], 1);

//	d_tree_ocp_qcqp_sol_get_all(&qp, &qp_sol, u, x, ls, us, pi, lam_lb, lam_ub, lam_lg, lam_ug, lam_ls, lam_us);
	
	for(ii=0; ii<Nn; ii++)
		{
		d_tree_ocp_qcqp_sol_get_u(ii, &qp_sol, u[ii]);
		d_tree_ocp_qcqp_sol_get_x(ii, &qp_sol, x[ii]);
		d_tree_ocp_qcqp_sol_get_sl(ii, &qp_sol, ls[ii]);
		d_tree_ocp_qcqp_sol_get_su(ii, &qp_sol, us[ii]);
		d_tree_ocp_qcqp_sol_get_lam_lb(ii, &qp_sol, lam_lb[ii]);
		d_tree_ocp_qcqp_sol_get_lam_ub(ii, &qp_sol, lam_ub[ii]);
		d_tree_ocp_qcqp_sol_get_lam_lg(ii, &qp_sol, lam_lg[ii]);
		d_tree_ocp_qcqp_sol_get_lam_ug(ii, &qp_sol, lam_ug[ii]);
		}
	for(ii=0; ii<Nn-1; ii++)
		{
		d_tree_ocp_qcqp_sol_get_pi(ii, &qp_sol, pi[ii]);
		}

#if PRINT
#if 1
	d_tree_ocp_qcqp_sol_print(&dim, &qp_sol);
	d_tree_ocp_qcqp_res_print(&dim, workspace.qcqp_res);
#else
	printf("\nsolution\n\n");
	printf("\nu\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, nut[ii], u[ii], 1);
	printf("\nx\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, nxt[ii], x[ii], 1);
	printf("\npi\n");
	for(ii=0; ii<Nn-1; ii++)
		d_print_mat(1, nxt[ii+1], pi[ii], 1);
	printf("\nlam_lb\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, nbt[ii], lam_lb[ii], 1);
	printf("\nlam_ub\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, nbt[ii], lam_ub[ii], 1);
	printf("\nlam_lg\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, ngt[ii], lam_lg[ii], 1);
	printf("\nlam_ug\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, ngt[ii], lam_ug[ii], 1);

	printf("\nt_lb\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_mat(1, nbt[ii], (qp_sol.t+ii)->pa+0, 1);
	printf("\nt_ub\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_mat(1, nbt[ii], (qp_sol.t+ii)->pa+nbt[ii]+ngt[ii], 1);
	printf("\nt_lg\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_mat(1, ngt[ii], (qp_sol.t+ii)->pa+nbt[ii], 1);
	printf("\nt_ug\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_mat(1, ngt[ii], (qp_sol.t+ii)->pa+2*nbt[ii]+ngt[ii], 1);

	printf("\nresiduals\n\n");
	printf("\nres_g\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, nut[ii]+nxt[ii], (workspace.res->res_g+ii)->pa, 1);
	printf("\nres_b\n");
//	for(ii=0; ii<Nn-1; ii++)
//		d_print_exp_mat(1, nxt[ii+1], (workspace.res->res_b+ii)->pa, 1);
	printf("\nres_m_lb\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, nbt[ii], (workspace.res->res_m+ii)->pa+0, 1);
	printf("\nres_m_ub\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, nbt[ii], (workspace.res->res_m+ii)->pa+nbt[ii]+ngt[ii], 1);
	printf("\nres_m_lg\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, ngt[ii], (workspace.res->res_m+ii)->pa+nbt[ii], 1);
	printf("\nres_m_ug\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, ngt[ii], (workspace.res->res_m+ii)->pa+2*nbt[ii]+ngt[ii], 1);
	printf("\nres_d_lb\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, nbt[ii], (workspace.res->res_d+ii)->pa+0, 1);
	printf("\nres_d_ub\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, nbt[ii], (workspace.res->res_d+ii)->pa+nbt[ii]+ngt[ii], 1);
	printf("\nres_d_lg\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, ngt[ii], (workspace.res->res_d+ii)->pa+nbt[ii], 1);
	printf("\nres_d_ug\n");
//	for(ii=0; ii<Nn; ii++)
//		d_print_exp_mat(1, ngt[ii], (workspace.res->res_d+ii)->pa+2*nbt[ii]+ngt[ii], 1);
	printf("\nres_mu\n");
//	printf("\n%e\n\n", workspace.res->res_mu);
#endif
#endif

#if PRINT
//	d_print_mat(1, nx_, x[Nn-1], 1);
	double tmp_qc = 0.0;
	for(ii=0; ii<nx_; ii++)
		tmp_qc += QqN[ii+nx_*ii]*x[Nn-1][ii]*x[Nn-1][ii];
	tmp_qc *= 0.5;
	printf("\nquadr constr at node Nn: %f\n\n", tmp_qc);
#endif

/************************************************
* print ipm statistics
************************************************/

	int iter; d_tree_ocp_qcqp_ipm_get_iter(&workspace, &iter);
	double res_stat; d_tree_ocp_qcqp_ipm_get_max_res_stat(&workspace, &res_stat);
	double res_eq; d_tree_ocp_qcqp_ipm_get_max_res_eq(&workspace, &res_eq);
	double res_ineq; d_tree_ocp_qcqp_ipm_get_max_res_ineq(&workspace, &res_ineq);
	double res_comp; d_tree_ocp_qcqp_ipm_get_max_res_comp(&workspace, &res_comp);
	double *stat; d_tree_ocp_qcqp_ipm_get_stat(&workspace, &stat);
	int stat_m; d_tree_ocp_qcqp_ipm_get_stat_m(&workspace, &stat_m);

#if PRINT
	printf("\nipm return = %d\n", hpipm_status);
	printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

	printf("\nipm iter = %d\n", iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
	d_print_exp_tran_mat(stat_m, iter+1, stat, stat_m);

	printf("\nocp ipm time = %e [s]\n\n", time_ocp_ipm);
#endif

/************************************************
* free memory
************************************************/

#if 0
	free(A);
	free(B);
	free(b);
	free(x0);
	free(b0);
	free(Q);
	free(R);
	free(S);
	free(q);
	free(r);
	free(r0);
	int_free(idxb0);
	d_free(d_lb0);
	d_free(d_ub0);
	int_free(idxb1);
	d_free(d_lb1);
	d_free(d_ub1);
	int_free(idxbN);
	d_free(d_lbN);
	d_free(d_ubN);
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
	d_free(Zl1);
	d_free(Zu1);
	d_free(zl1);
	d_free(zu1);
	int_free(idxs1);
	d_free(ZlN);
	d_free(ZuN);
	d_free(zlN);
	d_free(zuN);
	int_free(idxsN);
	d_free(uqN);
	d_free(QqN);


	for(ii=0; ii<Nn-1; ii++)
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

	free(tree_memory);
	free(tree_ocp_qp_memory);
	free(tree_ocp_qp_sol_memory);
	free(ipm_memory);
#endif

	return hpipm_status;

	}

