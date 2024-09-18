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
#include <sys/time.h>

#include "tools.h"

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include "../include/tree.h"
#include "../include/lqcp_solvers.h"



int ipow(int base, int exp)
	{
	int result = 1;
	while(exp)
		{
		if(exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
		}
	return result;
	}



int get_number_of_nodes(int md, int Nr, int Nh)
	{
	int n_nodes;
	if(md==1) // i.e. standard block-banded structure
		n_nodes = Nh+1;
	else
		n_nodes = (Nh-Nr)*ipow(md,Nr) + (ipow(md,Nr+1)-1)/(md-1);
	return n_nodes;
	}



void print_node(struct node *tree)
	{
	int ii;
	printf("\n");
	printf("idx = \t\t%d\n", tree[0].idx);
	printf("dad = \t\t%d\n", tree[0].dad);
	printf("nkids = \t%d\n", tree[0].nkids);
	printf("kids = \t\t");
	for(ii=0; ii<tree[0].nkids; ii++)
		printf("%d\t", tree[0].kids[ii]);
	printf("\n");
	printf("stage = \t%d\n", tree[0].stage);
	printf("realization = \t%d\n", tree[0].real);
	printf("index as a kid = \t%d\n", tree[0].idxkid);
	printf("\n");
	return;
	}



void setup_tree(int md, int Nr, int Nh, int Nn, struct node *tree)
	{
	int ii;
	int idx, dad, stage, real, nkids, idxkid;
	// root
	idx = 0;
	dad = -1;
	stage = 0;
	real = -1;
	if(stage<Nr)
		nkids = md;
	else if(stage<Nh)
		nkids = 1;
	else
		nkids = 0;
	tree[idx].idx = idx;
	tree[idx].dad = dad;
	tree[idx].stage = stage;
	tree[idx].real = real;
	tree[idx].nkids = nkids;
	tree[idx].idxkid = 0;
	if(nkids>0)
		{
		tree[idx].kids = (int *) malloc(nkids*sizeof(int));
		if(nkids>1)
			{
			for(ii=0; ii<nkids; ii++)
				{
				idxkid = ii+1;
				tree[idx].kids[ii] = idxkid;
				tree[idxkid].dad = idx;
				tree[idxkid].real = ii;
				tree[idxkid].idxkid = ii;
				}
			}
		else // nkids==1
			{
			idxkid = 1;
			tree[idx].kids[0] = idxkid;
			tree[idxkid].dad = idx;
			tree[idxkid].real = 0;
			tree[idxkid].idxkid = 0;
			}
		}
	// kids
	for(idx=1; idx<Nn; idx++)
		{
		stage = tree[tree[idx].dad].stage+1;
		if(stage<Nr)
			nkids = md;
		else if(stage<Nh)
			nkids = 1;
		else
			nkids = 0;
		tree[idx].idx = idx;
		tree[idx].stage = stage;
		tree[idx].nkids = nkids;
		if(nkids>0)
			{
			tree[idx].kids = (int *) malloc(nkids*sizeof(int));
			if(nkids>1)
				{
				for(ii=0; ii<nkids; ii++)
					{
					idxkid = tree[idx-1].kids[tree[idx-1].nkids-1]+ii+1;
					tree[idx].kids[ii] = idxkid;
					tree[idxkid].dad = idx;
					tree[idxkid].real = ii;
					tree[idxkid].idxkid = ii;
					}
				}
			else // nkids==1
				{
				idxkid = tree[idx-1].kids[tree[idx-1].nkids-1]+1;
				tree[idx].kids[0] = idxkid;
				tree[idxkid].dad = idx;
				tree[idxkid].real = tree[idx].real;
				tree[idxkid].idxkid = 0;
				}
			}
		}
	// return
	return;
	}



void free_tree(int md, int Nr, int Nh, int Nn, struct node *tree)
	{
	int ii;
	int idx, dad, stage, real, nkids, idxkid;
	// root
	idx = 0;
	dad = -1;
	stage = 0;
	real = -1;
	if(stage<Nr)
		nkids = md;
	else if(stage<Nh)
		nkids = 1;
	else
		nkids = 0;
	if(nkids>0)
		{
		free(tree[idx].kids);
		}
	// kids
	for(idx=1; idx<Nn; idx++)
		{
		stage = tree[tree[idx].dad].stage+1;
		if(stage<Nr)
			nkids = md;
		else if(stage<Nh)
			nkids = 1;
		else
			nkids = 0;
		if(nkids>0)
			{
			free(tree[idx].kids);
			}
		}
	// return
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

	printf("\nLA provided by HIGH_PERFORMANCE\n\n");

#elif defined(LA_BLAS)

	printf("\nLA provided by BLAS\n\n");

#elif defined(LA_REFERENCE)

	printf("\nLA provided by REFERENCE\n\n");

#else

	printf("\nLA provided by ???\n\n");
	exit(2);

#endif

	// loop index
	int ii, jj;

/************************************************
* problem size
************************************************/

	// problem size
	int N = 5;
	int nx_ = 4;
	int nu_ = 1;

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

	int nb[N+1];
	for(ii=0; ii<=N; ii++)
		nb[ii] = 0;

	int ng[N+1];
	for(ii=0; ii<=N; ii++)
		ng[ii] = 0;

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

//	d_print_mat(nx_, nx_, A, nx_);
//	d_print_mat(nx_, nu_, B, nx_);
//	d_print_mat(1, nx_, b, 1);
//	d_print_mat(1, nx_, x0, 1);

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

//	d_print_mat(nu_, nu_, R, nu_);
//	d_print_mat(nu_, nx_, S, nu_);
//	d_print_mat(nx_, nx_, Q, nx_);
//	d_print_mat(1, nu_, r, 1);
//	d_print_mat(1, nx_, q, 1);

/************************************************
* matrices as strmat
************************************************/

	struct blasfeo_dmat sA;
	blasfeo_allocate_dmat(nx_, nx_, &sA);
	blasfeo_pack_dmat(nx_, nx_, A, nx_, &sA, 0, 0);
	struct blasfeo_dvec sb;
	blasfeo_allocate_dvec(nx_, &sb);
	blasfeo_pack_dvec(nx_, b, &sb, 0);
	struct blasfeo_dvec sx0;
	blasfeo_allocate_dvec(nx_, &sx0);
	blasfeo_pack_dvec(nx_, x0, &sx0, 0);
	struct blasfeo_dvec sb0;
	blasfeo_allocate_dvec(nx_, &sb0);
	double *b0; d_zeros(&b0, nx_, 1); // states offset
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb, 0, &sb0, 0);
//	blasfeo_print_tran_dvec(nx_, &sb0, 0);

	struct blasfeo_dmat sBbt0;
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &sBbt0);
	blasfeo_pack_tran_dmat(nx_, nx_, B, nx_, &sBbt0, 0, 0);
	blasfeo_drowin(nx_, 1.0, &sb0, 0, &sBbt0, nu_, 0);
//	d_print_strmat(nu_+1, nx_, &sBbt0, 0, 0);

	struct blasfeo_dmat sBAbt1;
	blasfeo_allocate_dmat(nu_+nx_+1, nx_, &sBAbt1);
	blasfeo_pack_tran_dmat(nx_, nu_, B, nx_, &sBAbt1, 0, 0);
	blasfeo_pack_tran_dmat(nx_, nx_, A, nx_, &sBAbt1, nu_, 0);
	blasfeo_pack_tran_dmat(nx_, 1, b, nx_, &sBAbt1, nu_+nx_, 0);
//	d_print_strmat(nu_+nx_+1, nx_, &sBAbt1, 0, 0);

	struct blasfeo_dvec sr0; // XXX no need to update r0 since S=0
	blasfeo_allocate_dvec(nu_, &sr0);
	blasfeo_pack_dvec(nu_, r, &sr0, 0);

	struct blasfeo_dmat sRr0;
	blasfeo_allocate_dmat(nu_+1, nu_, &sRr0);
	blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRr0, 0, 0);
	blasfeo_drowin(nu_, 1.0, &sr0, 0, &sRr0, nu_, 0);
//	d_print_strmat(nu_+1, nu_, &sRr0, 0, 0);

	struct blasfeo_dvec srq1;
	blasfeo_allocate_dvec(nu_+nx_, &srq1);
	blasfeo_pack_dvec(nu_, r, &srq1, 0);
	blasfeo_pack_dvec(nx_, q, &srq1, nu_);

	struct blasfeo_dmat sRSQrq1;
	blasfeo_allocate_dmat(nu_+nx_+1, nu_+nx_, &sRSQrq1);
	blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRSQrq1, 0, 0);
	blasfeo_pack_tran_dmat(nu_, nx_, S, nu_, &sRSQrq1, nu_, 0);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sRSQrq1, nu_, nu_);
	blasfeo_drowin(nu_+nx_, 1.0, &srq1, 0, &sRSQrq1, nu_+nx_, 0);
//	d_print_strmat(nu_+nx_+1, nu_+nx_, &sRSQrq1, 0, 0);

	struct blasfeo_dvec sqN;
	blasfeo_allocate_dvec(nx_, &sqN);
	blasfeo_pack_dvec(nx_, q, &sqN, 0);

	struct blasfeo_dmat sQqN;
	blasfeo_allocate_dmat(nx_+1, nx_, &sQqN);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sQqN, 0, 0);
	blasfeo_drowin(nx_, 1.0, &sqN, 0, &sQqN, nx_, 0);
//	d_print_strmat(nx_+1, nx_, &sQqN, 0, 0);

/************************************************
* array of matrices
************************************************/

	struct blasfeo_dmat hsBAbt[N];
	struct blasfeo_dvec hsb[N];
	struct blasfeo_dmat hsRSQrq[N+1];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dmat hsDCt[N+1];
	struct blasfeo_dvec hsQx[N+1];
	struct blasfeo_dvec hsqx[N+1];
	struct blasfeo_dmat hsL[N+1];
	struct blasfeo_dvec hsPb[N+1];
	struct blasfeo_dvec hsux[N+1];
	struct blasfeo_dvec hspi[N+1];
	struct blasfeo_dmat hswork_mat[1];
	struct blasfeo_dvec hswork_vec[1];
	int *hidxb[N+1];

	// a different matrix per stage !!!!!
#if 1
	blasfeo_allocate_dmat(nu_+1, nx_, &hsBAbt[0]);
	blasfeo_dgecp(nu_+1, nx_, 1.0, &sBbt0, 0, 0, &hsBAbt[0], 0, 0);
	blasfeo_allocate_dvec(nx_, &hsb[0]);
	blasfeo_dveccp(nx_, 1.0, &sb0, 0, &hsb[0], 0);
	blasfeo_allocate_dmat(nu_+nx_+1, nu_+nx_, &hsRSQrq[0]);
	blasfeo_dgecp(nu_+1, nu_, 1.0, &sRr0, 0, 0, &hsRSQrq[0], 0, 0);
	blasfeo_allocate_dvec(nu_, &hsrq[0]);
	blasfeo_dveccp(nu_, 1.0, &sr0, 0, &hsrq[0], 0);
#else
	hsBAbt[0] = sBbt0;
	hsb[0] = sb0;
	hsRSQrq[0] = sRr0;
	hsrq[0] = sr0;
#endif
	blasfeo_allocate_dmat(nu_+1, nu_, &hsL[0]);
	blasfeo_allocate_dvec(nx_, &hsPb[1]);
	blasfeo_allocate_dvec(nx_+nu_+1, &hsux[0]);
	blasfeo_allocate_dvec(nx_, &hspi[1]);
	for(ii=1; ii<N; ii++)
		{
#if 1
		blasfeo_allocate_dmat(nu_+nx_+1, nx_, &hsBAbt[ii]);
		blasfeo_dgecp(nu_+nx_+1, nx_, 1.0, &sBAbt1, 0, 0, &hsBAbt[ii], 0, 0);
		blasfeo_allocate_dvec(nx_, &hsb[ii]);
		blasfeo_dveccp(nx_, 1.0, &sb, 0, &hsb[ii], 0);
		blasfeo_allocate_dmat(nu_+nx_+1, nu_+nx_, &hsRSQrq[ii]);
		blasfeo_dgecp(nu_+nx_+1, nu_+nx_, 1.0, &sRSQrq1, 0, 0, &hsRSQrq[ii], 0, 0);
		blasfeo_allocate_dvec(nu_+nx_, &hsrq[ii]);
		blasfeo_dveccp(nu_+nx_, 1.0, &srq1, 0, &hsrq[ii], 0);
#else
		hsBAbt[ii] = sBAbt1;
		hsb[ii] = sb;
		hsRSQrq[ii] = sRSQrq1;
		hsrq[ii] = srq1;
#endif
		blasfeo_allocate_dmat(nu_+nx_+1, nu_+nx_, &hsL[ii]);
		blasfeo_allocate_dvec(nx_, &hsPb[ii+1]);
		blasfeo_allocate_dvec(nx_+nu_+1, &hsux[ii]);
		blasfeo_allocate_dvec(nx_, &hspi[ii+1]);
		}
#if 1
	blasfeo_allocate_dmat(nx_+1, nx_, &hsRSQrq[N]);
	blasfeo_dgecp(nx_+1, nx_, 1.0, &sQqN, 0, 0, &hsRSQrq[N], 0, 0);
	blasfeo_allocate_dvec(nx_, &hsrq[N]);
	blasfeo_dveccp(nx_, 1.0, &sqN, 0, &hsrq[N], 0);
#else
	hsRSQrq[N] = sQqN;
	hsrq[N] = sqN;
#endif
	blasfeo_allocate_dmat(nx_+1, nx_, &hsL[N]);
	blasfeo_allocate_dvec(nx_+nu_+1, &hsux[N]);

	// riccati work space
	void *work_ric;
	v_zeros_align(&work_ric, d_back_ric_rec_work_space_size_bytes_libstr(N, nx, nu, nb, ng));

//	for(ii=0; ii<N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
//	return 0;

/************************************************
* call Riccati solver
************************************************/

	// timing
	struct timeval tv0, tv1, tv2, tv3;
	int nrep = 1000;
	int rep;

//	for(ii=0; ii<=N; ii++)
//		printf("\n%d %d\n", nu[ii], nx[ii]);

	// warm up ???
	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_rec_trf_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsQx, hsL, work_ric);
		}

	gettimeofday(&tv0, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
//		d_back_ric_sv_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hsux, hspi, hswork_mat, hswork_vec);
		}

	gettimeofday(&tv1, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_rec_trf_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsQx, hsL, work_ric);
		}

	gettimeofday(&tv2, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_rec_trs_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsrq, hsDCt, hsqx, hsux, 1, hspi, 1, hsPb, hsL, work_ric);
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
	for(ii=1; ii<=N; ii++)
		blasfeo_print_tran_dvec(nx[ii], &hspi[ii], 0);

//	printf("\nL = \n\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], 0, 0);

	printf("\ntime sv\t\ttime trf\t\ttime trs\n");
	printf("\n%e\t%e\t%e\n", time_sv, time_trf, time_trs);
	printf("\n");

/************************************************
* scenario-tree MPC
************************************************/

	int Nh = N; // control horizion
	int Nr = 1; // robust horizion
	int md = 1; // number of realizations

	int Nn = get_number_of_nodes(md, Nr, Nh);
	printf("\nnumber of nodes = %d\n", Nn);

	struct node tree[Nn];

	// setup the tree
//printf("\n%d %d %d %d\n", md, Nr, Nh, Nn);
	setup_tree(md, Nr, Nh, Nn, tree);
//printf("\nciao\n");

	// print the tree
//	for(ii=0; ii<Nn; ii++)
//		print_node(&tree[ii]);

	// data structure

	int stage;

	// stage-wise variant size (tmp)
	int t_nx[Nn];
	int t_nu[Nn];
	int t_nb[Nn];
	int t_ng[Nn];
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		t_nx[ii] = nx[stage];
		t_nu[ii] = nu[stage];
		t_nb[ii] = nb[stage];
		t_ng[ii] = ng[stage];
		}

	// dynamics indexed by node (ecluding the root) // no real atm
	struct blasfeo_dmat t_hsBAbt[Nn-1]; // XXX defined on the edges !!!!!
	struct blasfeo_dvec t_hsb[Nn-1]; // XXX defined on the edges !!!!!
	for(ii=0; ii<Nn-1; ii++)
		{
		stage = tree[ii+1].stage-1;
		if(stage<N)
			{
			t_hsBAbt[ii] = hsBAbt[stage];
			t_hsb[ii] = hsb[stage];
			}
		}
#if 0
	for(ii=0; ii<Nn-1; ii++)
		{
		stage = tree[ii+1].stage-1;
		printf("\nstage = %d\n", stage);
		d_print_strmat(t_nu[stage]+t_nx[stage]+1, t_nx[stage+1], &t_hsBAbt[ii], 0, 0);
		}
	return;
#endif

	// temporary cost function indexed by stage
	struct blasfeo_dmat tmp_hsRSQrq[Nh+1];
	struct blasfeo_dvec tmp_hsrq[Nh+1];
	// first stages: scale cost function
	for(ii=0; ii<Nr; ii++)
		{
		blasfeo_allocate_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &tmp_hsRSQrq[ii]);
		blasfeo_allocate_dvec(nu[ii]+nx[ii], &tmp_hsrq[ii]);
		}
	// last stages: original cost function
	for(ii=Nr; ii<Nh; ii++)
		{
		tmp_hsRSQrq[ii] = sRSQrq1;
		tmp_hsrq[ii] = srq1;
		}
	// last stage
	tmp_hsRSQrq[Nh] = sQqN;
	tmp_hsrq[Nh] = sqN;
	// scale at first stages
	for(ii=Nr-1; ii>0; ii--)
		{
		blasfeo_dgecp(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], md, &tmp_hsRSQrq[ii+1], 0, 0, &tmp_hsRSQrq[ii], 0, 0);
		blasfeo_dveccp(nu[ii]+nx[ii], md, &tmp_hsrq[ii+1], 0, &tmp_hsrq[ii], 0);
		}
	// scale first stage
	ii = 0;
	blasfeo_dgecp(nu[ii]+nx[ii], nu[ii]+nx[ii], md, &tmp_hsRSQrq[ii+1], 0, 0, &tmp_hsRSQrq[ii], 0, 0);
	blasfeo_dgecp(1, nu[ii]+nx[ii], md, &tmp_hsRSQrq[ii+1], nu[ii+1]+nx[ii+1], 0, &tmp_hsRSQrq[ii], nu[ii]+nx[ii], 0);
	blasfeo_dveccp(nu[ii]+nx[ii], md, &tmp_hsrq[ii+1], 0, &tmp_hsrq[ii], 0);
//	for(ii=0; ii<=Nh; ii++)
//		{
//		d_print_strmat(t_nu[ii]+t_nx[ii]+1, t_nu[ii]+t_nx[ii], &tmp_hsRSQrq[ii], 0, 0);
//		}
//	return;

	// cost function indexed by node
	struct blasfeo_dmat t_hsRSQrq[Nn];
	struct blasfeo_dvec t_hsrq[Nn];
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		t_hsRSQrq[ii] = tmp_hsRSQrq[stage];
		t_hsrq[ii] = tmp_hsrq[stage];
		}

	// store factorization indexed by node
	struct blasfeo_dmat t_hsL[Nn];
	for(ii=0; ii<Nn; ii++)
		{
		blasfeo_allocate_dmat(t_nu[ii]+t_nx[ii]+1, t_nu[ii]+t_nx[ii], &t_hsL[ii]);
		}
	struct blasfeo_dvec t_hsPb[Nn];
	for(ii=0; ii<Nn; ii++)
		{
		blasfeo_allocate_dvec(t_nx[ii], &t_hsPb[ii]);
		}

	// solution indexed by node
	struct blasfeo_dvec t_hsux[Nn];
	for(ii=0; ii<Nn; ii++)
		{
		blasfeo_allocate_dvec(t_nu[ii]+t_nx[ii], &t_hsux[ii]);
		}
	struct blasfeo_dvec t_hspi[Nn];
	for(ii=0; ii<Nn; ii++)
		{
		blasfeo_allocate_dvec(t_nx[ii], &t_hspi[ii]);
		}

	// dummy
	int *t_hidxb[Nn];
	struct blasfeo_dmat hsmatdummy[Nn];
	struct blasfeo_dvec hsvecdummy[Nn];


	// call riccati
	gettimeofday(&tv1, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_tree_back_ric_rec_trf_libstr(Nn, tree, t_nx, t_nu, t_nb, t_hidxb, t_ng, t_hsBAbt, t_hsRSQrq, hsmatdummy, hsvecdummy, t_hsL, work_ric);
		}

	gettimeofday(&tv2, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_tree_back_ric_rec_trs_libstr(Nn, tree, t_nx, t_nu, t_nb, t_hidxb, t_ng, t_hsBAbt, t_hsb, t_hsrq, hsmatdummy, hsvecdummy, t_hsux, 1, t_hspi, 1, t_hsPb, t_hsL, work_ric);
		}

	gettimeofday(&tv3, NULL); // time

	// print factorization
//	for(ii=0; ii<Nn; ii++)
//		{
//		d_print_strmat(t_nu[ii]+t_nx[ii]+1, t_nu[ii]+t_nx[ii], &t_hsL[ii], 0, 0);
//		}
	for(ii=0; ii<Nn; ii++)
		{
		blasfeo_print_tran_dvec(t_nu[ii]+t_nx[ii], &t_hsux[ii], 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		blasfeo_print_tran_dvec(t_nx[ii], &t_hspi[ii], 0);
		}

	float time_tree_trf = (float) (tv2.tv_sec-tv1.tv_sec)/(nrep+0.0)+(tv2.tv_usec-tv1.tv_usec)/(nrep*1e6);
	float time_tree_trs = (float) (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);

	printf("\ntime sv\t\ttime trf\t\ttime trs\n");
	printf("\n%e\t%e\t%e\n", 0.0, time_tree_trf, time_tree_trs);
	printf("\n");

/************************************************
* scenario-tree MPC with standard solver
************************************************/

	int nx2[N+1];
	int nu2[N+1];
	for(ii=0; ii<=N; ii++)
		{
		nx2[ii] = 0;
		nu2[ii] = 0;
		}
	for(ii=Nn-1; ii>=0; ii--)
		{
		stage = tree[ii].stage;
		nx2[stage] += t_nx[ii];
		nu2[stage] += t_nu[ii];
		}
//	for(ii=0; ii<=N; ii++)
//		printf("\n%d\n", nx2[ii]);
//	for(ii=0; ii<=N; ii++)
//		printf("\n%d\n", nu2[ii]);

	// cost function
	struct blasfeo_dmat hsRSQrq2[N+1];
	struct blasfeo_dvec hsrq2[N+1];
	int tmp0[N+1];
	int tmp1[N+1];
	// allocate
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_allocate_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii]);
		blasfeo_allocate_dvec(nu2[ii]+nx2[ii], &hsrq2[ii]);
		}
	// zero index
	for(ii=0; ii<=N; ii++)
		{
		tmp0[ii] = 0;
		tmp1[ii] = 0;
		}
	// R
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		blasfeo_dgecp(t_nu[ii], t_nu[ii], 1.0, &t_hsRSQrq[ii], 0, 0, &hsRSQrq2[stage], tmp0[stage], tmp0[stage]);
		tmp0[stage] += t_nu[ii];
		}
	// Q
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		blasfeo_dgecp(t_nx[ii], t_nx[ii], 1.0, &t_hsRSQrq[ii], t_nu[ii], t_nu[ii], &hsRSQrq2[stage], tmp0[stage], tmp0[stage]);
		tmp0[stage] += t_nx[ii];
		}
	// r
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		blasfeo_dgecp(1, t_nu[ii], 1.0, &t_hsRSQrq[ii], t_nu[ii]+t_nx[ii], 0, &hsRSQrq2[stage], tmp0[stage], tmp1[stage]);
		blasfeo_dveccp(t_nu[ii], 1.0, &t_hsrq[ii], 0, &hsrq2[stage], tmp1[stage]);
		tmp1[stage] += t_nu[ii];
		}
	// q
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		blasfeo_dgecp(1, t_nx[ii], 1.0, &t_hsRSQrq[ii], t_nu[ii]+t_nx[ii], t_nu[ii], &hsRSQrq2[stage], tmp0[stage], tmp1[stage]);
		blasfeo_dveccp(t_nx[ii], 1.0, &t_hsrq[ii], t_nu[ii], &hsrq2[stage], tmp1[stage]);
		tmp1[stage] += t_nx[ii];
		}
	// print
//	for(ii=0; ii<=N; ii++)
//		{
//		d_print_strmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii], 0, 0);
//		blasfeo_print_tran_dvec(nu2[ii]+nx2[ii], &hsrq2[ii], 0);
//		}

	// dynamical system
	struct blasfeo_dmat hsBAbt2[N];
	struct blasfeo_dvec hsb2[N];
	int idxkid, kid_stage;
//	int tmp0;
	for(ii=0; ii<N; ii++)
		{
		blasfeo_allocate_dmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii]);
		blasfeo_allocate_dvec(nx2[ii+1], &hsb2[ii]);
		}
	// zero index
	for(ii=0; ii<=N; ii++)
		{
		tmp0[ii] = 0;
		tmp1[ii] = 0;
		}
	// B
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		for(jj=0; jj<tree[ii].nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			kid_stage = tree[idxkid].stage;
			blasfeo_dgecp(t_nu[ii], t_nx[idxkid], 1.0, &t_hsBAbt[idxkid-1], 0, 0, &hsBAbt2[stage], tmp1[stage], tmp0[kid_stage]);
			tmp0[kid_stage] += t_nx[idxkid];
			}
		tmp1[stage] += t_nu[ii];
		}
	// zero index
	for(ii=0; ii<=N; ii++)
		{
		tmp0[ii] = 0;
		}
	// A
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		for(jj=0; jj<tree[ii].nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			kid_stage = tree[idxkid].stage;
			blasfeo_dgecp(t_nx[ii], t_nx[idxkid], 1.0, &t_hsBAbt[idxkid-1], t_nu[ii], 0, &hsBAbt2[stage], tmp1[stage], tmp0[kid_stage]);
			tmp0[kid_stage] += t_nx[idxkid];
			}
		tmp1[stage] += t_nx[ii];
		}
	// zero index
	for(ii=0; ii<=N; ii++)
		{
		tmp0[ii] = 0;
		}
	// b
	for(ii=0; ii<Nn; ii++)
		{
		stage = tree[ii].stage;
		for(jj=0; jj<tree[ii].nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			kid_stage = tree[idxkid].stage;
			blasfeo_dgecp(1, t_nx[idxkid], 1.0, &t_hsBAbt[idxkid-1], t_nu[ii]+t_nx[ii], 0, &hsBAbt2[stage], tmp1[stage], tmp0[kid_stage]);
			blasfeo_dveccp(t_nx[idxkid], 1.0, &t_hsb[idxkid-1], 0, &hsb2[stage], tmp0[kid_stage]);
			tmp0[kid_stage] += t_nx[idxkid];
			}
//		tmp1[stage] += 1;
		}
	// print
//	for(ii=0; ii<N; ii++)
//		{
//		d_print_strmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii], 0, 0);
//		blasfeo_print_tran_dvec(nx2[ii+1], &hsb2[ii], 0);
//		}

	// work space
	struct blasfeo_dvec hsux2[N+1];
	struct blasfeo_dvec hspi2[N+1];
	struct blasfeo_dmat hsL2[N+1];
	struct blasfeo_dvec hsPb2[N+1];
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_allocate_dvec(nu2[ii]+nx2[ii], &hsux2[ii]);
		blasfeo_allocate_dvec(nx2[ii], &hspi2[ii]);
		blasfeo_allocate_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsL2[ii]);
		blasfeo_allocate_dvec(nx2[ii], &hsPb2[ii]);
		}

	// riccati work space
	void *work_ric2;
	v_zeros_align(&work_ric2, d_back_ric_rec_work_space_size_bytes_libstr(N, nx2, nu2, nb, ng));

	// call Riccati solver


//	for(ii=0; ii<=N; ii++)
//		printf("\n%d %d\n", nu2[ii], nx2[ii]);

	gettimeofday(&tv0, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
//		d_back_ric_sv_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsL, hsux, hspi, hswork_mat, hswork_vec);
		}

	gettimeofday(&tv1, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_rec_trf_libstr(N, nx2, nu2, nb, hidxb, ng, hsBAbt2, hsRSQrq2, hsDCt, hsQx, hsL2, work_ric2);
		}

	gettimeofday(&tv2, NULL); // time

	for(rep=0; rep<nrep; rep++)
		{
		d_back_ric_rec_trs_libstr(N, nx2, nu2, nb, hidxb, ng, hsBAbt2, hsb2, hsrq2, hsDCt, hsqx, hsux2, 1, hspi2, 1, hsPb2, hsL2, work_ric2);
		}

	gettimeofday(&tv3, NULL); // time

	float time_sv2  = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	float time_trf2 = (float) (tv2.tv_sec-tv1.tv_sec)/(nrep+0.0)+(tv2.tv_usec-tv1.tv_usec)/(nrep*1e6);
	float time_trs2 = (float) (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);

	// print
//	printf("\nL\n");
//	for(ii=0; ii<=N; ii++)
//		{
//		d_print_strmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsL2[ii], 0, 0);
//		}
	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_print_tran_dvec(nu2[ii]+nx2[ii], &hsux2[ii], 0);
		}
	printf("\npi\n");
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_print_tran_dvec(nx2[ii], &hspi2[ii], 0);
		}

	printf("\ntime sv2\ttime trf2\ttime trs2\n");
	printf("\n%e\t%e\t%e\n", time_sv2, time_trf2, time_trs2);
	printf("\n");

	return 0;

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
	blasfeo_free_dvec(&hsPb[1]);
	blasfeo_free_dvec(&hsux[0]);
	blasfeo_free_dvec(&hspi[1]);
	for(ii=1; ii<N; ii++)
		{
		blasfeo_free_dmat(&hsL[ii]);
		blasfeo_free_dvec(&hsPb[ii+1]);
		blasfeo_free_dvec(&hsux[ii]);
		blasfeo_free_dvec(&hspi[ii+1]);
		}
	blasfeo_free_dmat(&hsL[N]);
	blasfeo_free_dvec(&hsux[N]);
	blasfeo_free_dmat(&hswork_mat[0]);
	blasfeo_free_dvec(&hswork_vec[0]);

	// free memory allocated in the tree
	free_tree(md, Nr, Nh, Nn, tree);


/************************************************
* return
************************************************/

	return 0;

	}
