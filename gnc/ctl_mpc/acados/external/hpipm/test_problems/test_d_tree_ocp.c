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
#include <hpipm_d_tree_ocp_qp_dim.h>
#include <hpipm_d_tree_ocp_qp.h>
#include <hpipm_d_tree_ocp_qp_sol.h>
#include <hpipm_d_tree_ocp_qp_res.h>
#include <hpipm_d_tree_ocp_qp_ipm.h>

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

	int nx_ = 8;
	int nu_ = 3;

	int md = 2;
	int Nr = 2;
	int Nh = 5;

	// stage-wise size
	int nx[Nh+1];
	int nu[Nh+1];
	int nbx[Nh+1];
	int nbu[Nh+1];
	int nb[Nh+1];
	int ng[Nh+1];
	int ns[Nh+1];
	int nsbx[Nh+1];
	int nsbu[Nh+1];
	int nsg[Nh+1];

	nx[0] = 0;
	nu[0] = nu_;
	nbx[0] = nx[0]/2;
	nbu[0] = nu[0];
	nb[0] = nbx[0] + nbu[0];
	ng[0] = 0;
	ns[0] = 0;
	nsbx[0] = 0;
	nsbu[0] = 0;
	nsg[0] = 0;
	for(ii=1; ii<Nh; ii++)
		{
		nx[ii] = nx_;
		nu[ii] = nu_;
		nbx[ii] = nx[ii]/2;
		nbu[ii] = nu[ii];
		nb[ii] = nbx[ii] + nbu[ii];
		ng[ii] = 0;
		nsbx[ii] = nx[ii]/2;
		nsbu[ii] = 0;
		nsg[ii] = 0;
		ns[ii] = nsbx[ii]+nsbu[ii]+nsg[ii];
		}
	nx[Nh] = nx_;
	nu[Nh] = 0;
	nbx[Nh] = nx[Nh]/2;
	nbu[Nh] = nu[Nh];
	nb[Nh] = nbx[Nh] + nbu[Nh];
	ng[Nh] = 0;
	nsbx[Nh] = nx[Nh]/2;
	nsbu[Nh] = 0;
	nsg[Nh] = 0;
	ns[ii] = nsbx[ii]+nsbu[ii]+nsg[ii];

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
	double mu0 = 2.0;

/************************************************
* box & general constraints
************************************************/

	int *idxb0; int_zeros(&idxb0, nb[0], 1);
	double *d_lb0; d_zeros(&d_lb0, nb[0], 1);
	double *d_ub0; d_zeros(&d_ub0, nb[0], 1);
	double *d_lg0; d_zeros(&d_lg0, ng[0], 1);
	double *d_ug0; d_zeros(&d_ug0, ng[0], 1);
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
	double *d_lb1; d_zeros(&d_lb1, nb[1], 1);
	double *d_ub1; d_zeros(&d_ub1, nb[1], 1);
	double *d_lg1; d_zeros(&d_lg1, ng[1], 1);
	double *d_ug1; d_zeros(&d_ug1, ng[1], 1);
	for(ii=0; ii<nb[1]; ii++)
		{
		if(ii<nu[1]) // input
			{
			d_lb1[ii] = - 0.5; // umin
			d_ub1[ii] =   0.5; // umax
			}
		else // state
			{
			d_lb1[ii] = - 1.0; // xmin
			d_ub1[ii] =   1.0; // xmax
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


	int *idxbN; int_zeros(&idxbN, nb[Nh], 1);
	double *d_lbN; d_zeros(&d_lbN, nb[Nh], 1);
	double *d_ubN; d_zeros(&d_ubN, nb[Nh], 1);
	double *d_lgN; d_zeros(&d_lgN, ng[Nh], 1);
	double *d_ugN; d_zeros(&d_ugN, ng[Nh], 1);
	for(ii=0; ii<nb[Nh]; ii++)
		{
		d_lbN[ii] = - 1.0; // xmin
		d_ubN[ii] =   1.0; // xmax
		idxbN[ii] = ii;
		}
	for(ii=0; ii<ng[Nh]; ii++)
		{
		d_lgN[ii] =   0.1; // dmin
		d_ugN[ii] =   0.1; // dmax
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
	int_print_mat(1, nb[0], idxb0, 1);
	d_print_mat(1, nb[0], d_lb0, 1);
	d_print_mat(1, nb[0], d_ub0, 1);
	int_print_mat(1, nb[1], idxb1, 1);
	d_print_mat(1, nb[1], d_lb1, 1);
	d_print_mat(1, nb[1], d_ub1, 1);
	int_print_mat(1, nb[Nh], idxbN, 1);
	d_print_mat(1, nb[Nh], d_lbN, 1);
	d_print_mat(1, nb[Nh], d_ubN, 1);
	// general constraints
	d_print_mat(1, ng[0], d_lg0, 1);
	d_print_mat(1, ng[0], d_ug0, 1);
	d_print_mat(ng[0], nu[0], D0, ng[0]);
	d_print_mat(ng[0], nx[0], C0, ng[0]);
	d_print_mat(1, ng[1], d_lg1, 1);
	d_print_mat(1, ng[1], d_ug1, 1);
	d_print_mat(ng[1], nu[1], D1, ng[1]);
	d_print_mat(ng[1], nx[1], C1, ng[1]);
	d_print_mat(1, ng[Nh], d_lgN, 1);
	d_print_mat(1, ng[Nh], d_ugN, 1);
	d_print_mat(ng[Nh], nu[Nh], DN, ng[Nh]);
	d_print_mat(ng[Nh], nx[Nh], CN, ng[Nh]);
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
	int nst[Nn];
	int nsbxt[Nn];
	int nsbut[Nn];
	int nsgt[Nn];

	for(ii=0; ii<Nn; ii++)
		{
		stage = (ttree.root+ii)->stage;
		nxt[ii] = nx[stage];
		nut[ii] = nu[stage];
		nbxt[ii] = nbx[stage];
		nbut[ii] = nbu[stage];
		nbt[ii] = nb[stage];
		ngt[ii] = ng[stage];
		nst[ii] = ns[stage];
		nsbxt[ii] = nsbx[stage];
		nsbut[ii] = nsbu[stage];
		nsgt[ii] = nsg[stage];
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
	int *hidxb[Nh+1];
	double *hd_lb[Nh+1];
	double *hd_ub[Nh+1];
	double *hC[Nh+1];
	double *hD[Nh+1];
	double *hd_lg[Nh+1];
	double *hd_ug[Nh+1];
	double *hZl[Nh+1];
	double *hZu[Nh+1];
	double *hzl[Nh+1];
	double *hzu[Nh+1];
	int *hidxs[Nh+1];
	double *hd_ls[Nh+1];
	double *hd_us[Nh+1];

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
		hidxb[ii] = idxb1;
		hd_lb[ii] = d_lb1;
		hd_ub[ii] = d_ub1;
		hC[ii] = C1;
		hD[ii] = D1;
		hd_lg[ii] = d_lg1;
		hd_ug[ii] = d_ug1;
		hZl[ii] = Zl1;
		hZu[ii] = Zu1;
		hzl[ii] = zl1;
		hzu[ii] = zu1;
		hidxs[ii] = idxs1;
		hd_ls[ii] = d_ls1;
		hd_us[ii] = d_us1;
		}
	hQ[Nh] = Q;
	hS[Nh] = S;
	hR[Nh] = R;
	hq[Nh] = q;
	hr[Nh] = r;
	hidxb[Nh] = idxbN;
	hd_lb[Nh] = d_lbN;
	hd_ub[Nh] = d_ubN;
	hC[Nh] = CN;
	hD[Nh] = DN;
	hd_lg[Nh] = d_lgN;
	hd_ug[Nh] = d_ugN;
	hZl[Nh] = ZlN;
	hZu[Nh] = ZuN;
	hzl[Nh] = zlN;
	hzu[Nh] = zuN;
	hidxs[Nh] = idxsN;
	hd_ls[Nh] = d_lsN;
	hd_us[Nh] = d_usN;

	// node-wise data

	double *hAt[Nn-1];
	double *hBt[Nn-1];
	double *hbt[Nn-1];
	double *hQt[Nn];
	double *hSt[Nn];
	double *hRt[Nn];
	double *hqt[Nn];
	double *hrt[Nn];
	int *hidxbt[Nn];
	double *hd_lbt[Nn];
	double *hd_ubt[Nn];
	double *hCt[Nn];
	double *hDt[Nn];
	double *hd_lgt[Nn];
	double *hd_ugt[Nn];
	double *hZlt[Nn];
	double *hZut[Nn];
	double *hzlt[Nn];
	double *hzut[Nn];
	int *hidxst[Nn];
	double *hd_lst[Nn];
	double *hd_ust[Nn];

	for(ii=0; ii<Nn-1; ii++)
		{
		stage = (ttree.root+ii+1)->stage-1;
		hAt[ii] = hA[stage];
		hBt[ii] = hB[stage];
		hbt[ii] = hb[stage];
		}

	for(ii=0; ii<Nn; ii++)
		{
		stage = (ttree.root+ii)->stage;
		hQt[ii] = hQ[stage];
		hRt[ii] = hR[stage];
		hSt[ii] = hS[stage];
		hqt[ii] = hq[stage];
		hrt[ii] = hr[stage];
		hidxbt[ii] = hidxb[stage];
		hd_lbt[ii] = hd_lb[stage];
		hd_ubt[ii] = hd_ub[stage];
		hCt[ii] = hC[stage];
		hDt[ii] = hD[stage];
		hd_lgt[ii] = hd_lg[stage];
		hd_ugt[ii] = hd_ug[stage];
		hZlt[ii] = hZl[stage];
		hZut[ii] = hZu[stage];
		hzlt[ii] = hzl[stage];
		hzut[ii] = hzu[stage];
		hidxst[ii] = hidxs[stage];
		hd_lst[ii] = hd_ls[stage];
		hd_ust[ii] = hd_us[stage];
		}

/************************************************
* create tree ocp qp dim
************************************************/

	hpipm_size_t dim_size = d_tree_ocp_qp_dim_memsize(Nn);
#if PRINT
	printf("\ndim size = %d\n", dim_size);
#endif
	void *dim_mem = malloc(dim_size);

	struct d_tree_ocp_qp_dim dim;
	d_tree_ocp_qp_dim_create(Nn, &dim, dim_mem);
	d_tree_ocp_qp_dim_set_all(&ttree, nxt, nut, nbxt, nbut, ngt, nsbxt, nsbut, nsgt, &dim);

/************************************************
* create tree ocp qp
************************************************/

	hpipm_size_t tree_ocp_qp_memsize = d_tree_ocp_qp_memsize(&dim);
#if PRINT
	printf("\ntree ocp qp memsize = %d\n", tree_ocp_qp_memsize);
#endif
	void *tree_ocp_qp_memory = malloc(tree_ocp_qp_memsize);

	struct d_tree_ocp_qp qp;
	d_tree_ocp_qp_create(&dim, &qp, tree_ocp_qp_memory);
	d_tree_ocp_qp_set_all(hAt, hBt, hbt, hQt, hSt, hRt, hqt, hrt, hidxbt, hd_lbt, hd_ubt, hCt, hDt, hd_lgt, hd_ugt, hZlt, hZut, hzlt, hzut, hidxst, hd_lst, hd_ust, &qp);

#if 0
	struct blasfeo_dmat *tmat;
	struct blasfeo_dvec *tvec;
	for(ii=0; ii<Nn-1; ii++)
		{
		tmat = qp.BAbt+ii;
		d_print_strmat(tmat->m, tmat->n, tmat, 0, 0);
		}
	for(ii=0; ii<Nn-1; ii++)
		{
		tvec = qp.b+ii;
		blasfeo_print_tran_dvec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tmat = qp.RSQrq+ii;
		d_print_strmat(tmat->m, tmat->n, tmat, 0, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tvec = qp.rq+ii;
		blasfeo_print_tran_dvec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tvec = qp.d+ii;
		blasfeo_print_tran_dvec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		int_print_mat(1, qp.nb[ii], qp.idxb[ii], 1);
		}
exit(1);
#endif

/************************************************
* ocp qp sol
************************************************/

	hpipm_size_t tree_ocp_qp_sol_size = d_tree_ocp_qp_sol_memsize(&dim);
#if PRINT
	printf("\ntree ocp qp sol memsize = %d\n", tree_ocp_qp_sol_size);
#endif
	void *tree_ocp_qp_sol_memory = malloc(tree_ocp_qp_sol_size);

	struct d_tree_ocp_qp_sol qp_sol;
	d_tree_ocp_qp_sol_create(&dim, &qp_sol, tree_ocp_qp_sol_memory);

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_tree_ocp_qp_ipm_arg_memsize(&dim);
#if PRINT
	printf("\nipm arg size = %d\n", ipm_arg_size);
#endif
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_tree_ocp_qp_ipm_arg arg;
	d_tree_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
//	enum hpipm_mode mode = SPEED_ABS;
	enum hpipm_mode mode = SPEED;
//	enum hpipm_mode mode = BALANCE;
//	enum hpipm_mode mode = ROBUST;
	d_tree_ocp_qp_ipm_arg_set_default(mode, &arg);

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

	hpipm_size_t ipm_size = d_tree_ocp_qp_ipm_ws_memsize(&dim, &arg);
#if PRINT
	printf("\nipm size = %d\n", ipm_size);
#endif
	void *ipm_memory = malloc(ipm_size);

	struct d_tree_ocp_qp_ipm_ws workspace;
	d_tree_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_memory);

	int hpipm_status; // 0 normal; 1 max iter

	int rep, nrep=100;

	struct timeval tv0, tv1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_tree_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
		d_tree_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
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

	d_tree_ocp_qp_sol_get_all(&qp, &qp_sol, u, x, ls, us, pi, lam_lb, lam_ub, lam_lg, lam_ug, lam_ls, lam_us);

#if PRINT
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
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, nbt[ii], (qp_sol.t+ii)->pa+0, 1);
	printf("\nt_ub\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, nbt[ii], (qp_sol.t+ii)->pa+nbt[ii]+ngt[ii], 1);
	printf("\nt_lg\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, ngt[ii], (qp_sol.t+ii)->pa+nbt[ii], 1);
	printf("\nt_ug\n");
	for(ii=0; ii<Nn; ii++)
		d_print_mat(1, ngt[ii], (qp_sol.t+ii)->pa+2*nbt[ii]+ngt[ii], 1);

	printf("\nresiduals\n\n");
	printf("\nres_g\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, nut[ii]+nxt[ii], (workspace.res->res_g+ii)->pa, 1);
	printf("\nres_b\n");
	for(ii=0; ii<Nn-1; ii++)
		d_print_exp_mat(1, nxt[ii+1], (workspace.res->res_b+ii)->pa, 1);
	printf("\nres_m_lb\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, nbt[ii], (workspace.res->res_m+ii)->pa+0, 1);
	printf("\nres_m_ub\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, nbt[ii], (workspace.res->res_m+ii)->pa+nbt[ii]+ngt[ii], 1);
	printf("\nres_m_lg\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, ngt[ii], (workspace.res->res_m+ii)->pa+nbt[ii], 1);
	printf("\nres_m_ug\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, ngt[ii], (workspace.res->res_m+ii)->pa+2*nbt[ii]+ngt[ii], 1);
	printf("\nres_d_lb\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, nbt[ii], (workspace.res->res_d+ii)->pa+0, 1);
	printf("\nres_d_ub\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, nbt[ii], (workspace.res->res_d+ii)->pa+nbt[ii]+ngt[ii], 1);
	printf("\nres_d_lg\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, ngt[ii], (workspace.res->res_d+ii)->pa+nbt[ii], 1);
	printf("\nres_d_ug\n");
	for(ii=0; ii<Nn; ii++)
		d_print_exp_mat(1, ngt[ii], (workspace.res->res_d+ii)->pa+2*nbt[ii]+ngt[ii], 1);
	printf("\nres_mu\n");
	printf("\n%e\n\n", workspace.res->res_mu);
#endif

/************************************************
* print ipm statistics
************************************************/

	int iter; d_tree_ocp_qp_ipm_get_iter(&workspace, &iter);
	double res_stat; d_tree_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
	double res_eq; d_tree_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
	double res_ineq; d_tree_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
	double res_comp; d_tree_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
	double *stat; d_tree_ocp_qp_ipm_get_stat(&workspace, &stat);
	int stat_m; d_tree_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

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

	return hpipm_status;

	}
