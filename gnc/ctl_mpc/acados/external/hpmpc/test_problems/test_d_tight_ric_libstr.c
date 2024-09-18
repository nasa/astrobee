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
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>
#endif

#include "../include/aux_d.h"
#include "../include/aux_s.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_aux.h"
#include "../include/mpc_solvers.h"
#include "tools.h"


#define KEEP_X0 0


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
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); // flush to zero subnormals !!! works only with one thread !!!
#endif

	int ii, jj;
	
	int rep, nrep=1;//NREP;

	int nx_ = 8; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = 3; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 10; // horizon lenght

	int M = 3; // not-tightened horizon (the last N-M stages are tightened)
	M = N<M ? N : M;


	// stage-wise variant size
	int nx[N+1];
#if KEEP_X0
	nx[0] = nx_;
#else
	nx[0] = 0;
#endif
	for(ii=1; ii<=N; ii++)
		nx[ii] = nx_;

	int nu[N+1];
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_;
	nu[N] = 0;

	int nb[N+1];
	for(ii=0; ii<M; ii++) // XXX not M !!!
		nb[ii] = nu[ii] + nx[ii]/2;
//		nb[ii] = 0;
	for(; ii<=N; ii++)
		nb[ii] = 0;

	int ng[N+1];
	for(ii=0; ii<=N; ii++)
		ng[ii] = 0;
	

	// max sizes
	int ngM = 0;
	for(ii=0; ii<=N; ii++)
		{
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}

	int nzM  = 0;
	for(ii=0; ii<=N; ii++)
		{
		nzM = nu[ii]+nx[ii]+1>nzM ? nu[ii]+nx[ii]+1 : nzM;
		}

	int nxgM = ng[N];
	for(ii=0; ii<N; ii++)
		{
		nxgM = nx[ii+1]+ng[ii]>nxgM ? nx[ii+1]+ng[ii] : nxgM;
		}
	


/************************************************
* dynamical system
************************************************/	

	double *A; d_zeros(&A, nx_, nx_); // states update matrix

	double *B; d_zeros(&B, nx_, nu_); // inputs matrix

	double *b; d_zeros_align(&b, nx_, 1); // states offset
	double *x0; d_zeros_align(&x0, nx_, 1); // initial state

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx_, nu_, N, A, B, b, x0);
	
	for(jj=0; jj<nx_; jj++)
		b[jj] = 0.1;
	
	for(jj=0; jj<nx_; jj++)
		x0[jj] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

	d_print_mat(nx_, nx_, A, nx_);
	d_print_mat(nx_, nu_, B, nu_);
	d_print_mat(1, nx_, b, 1);
	d_print_mat(1, nx_, x0, 1);

	struct blasfeo_dmat sA;
	blasfeo_allocate_dmat(nx_, nx_, &sA);
	blasfeo_pack_dmat(nx_, nx_, A, nx_, &sA, 0, 0);
	d_print_strmat(nx_, nx_, &sA, 0, 0);

	struct blasfeo_dvec sx0;
	blasfeo_allocate_dvec(nx_, &sx0);
	blasfeo_pack_dvec(nx_, x0, &sx0, 0);
	blasfeo_print_tran_dvec(nx_, &sx0, 0);

	struct blasfeo_dvec sb0;
	blasfeo_allocate_dvec(nx_, &sb0);
	blasfeo_pack_dvec(nx_, b, &sb0, 0);
	blasfeo_print_tran_dvec(nx_, &sb0, 0);
#if ! KEEP_X0
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb0, 0, &sb0, 0);
#endif
	blasfeo_print_tran_dvec(nx_, &sb0, 0);

	struct blasfeo_dmat sBAbt0;
	blasfeo_allocate_dmat(nu[0]+nx[0]+1, nx[1], &sBAbt0);
	blasfeo_pack_tran_dmat(nx[1], nu[0], B, nx_, &sBAbt0, 0, 0);
	blasfeo_pack_tran_dmat(nx[1], nx[0], A, nx_, &sBAbt0, nu[0], 0);
	blasfeo_drowin(nx[1], 1.0, &sb0, 0, &sBAbt0, nu[0]+nx[0], 0);
	d_print_strmat(nu[0]+nx[0]+1, nx[1], &sBAbt0, 0, 0);

	struct blasfeo_dmat sBAbt1;
	struct blasfeo_dvec sb1;
	if(N>1)
		{
		blasfeo_allocate_dmat(nu[1]+nx[1]+1, nx[2], &sBAbt1);
		blasfeo_pack_tran_dmat(nx[2], nu[1], B, nx_, &sBAbt1, 0, 0);
		blasfeo_pack_tran_dmat(nx[2], nx[1], A, nx_, &sBAbt1, nu[1], 0);
		blasfeo_pack_tran_dmat(nx[2], 1, b, nx_, &sBAbt1, nu[1]+nx[1], 0);
		d_print_strmat(nu[1]+nx[1]+1, nx[2], &sBAbt1, 0, 0);
		blasfeo_allocate_dvec(nx_, &sb1);
		blasfeo_pack_dvec(nx_, b, &sb1, 0);
		}

/************************************************
* cost function
************************************************/	
	
	double *Q; d_zeros(&Q, nx_, nx_);
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 1.0;

	double *R; d_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 2.0;

	double *S; d_zeros(&S, nu_, nx_); // S=0, so no need to update r0

	double *q; d_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.1;

	double *r; d_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.2;

	double mu0 = 2.0;

	struct blasfeo_dmat sRSQrq0;
	struct blasfeo_dvec srq0;
	blasfeo_allocate_dmat(nu[0]+nx[0]+1, nu[0]+nx[0], &sRSQrq0);
	blasfeo_pack_dmat(nu[0], nu[0], R, nu_, &sRSQrq0, 0, 0);
	blasfeo_pack_tran_dmat(nu[0], nx[0], S, nu_, &sRSQrq0, nu[0], 0);
	blasfeo_pack_dmat(nx[0], nx[0], Q, nx_, &sRSQrq0, nu[0], nu[0]);
	blasfeo_pack_tran_dmat(nu[0], 1, r, nu_, &sRSQrq0, nu[0]+nx[0], 0);
	blasfeo_pack_tran_dmat(nx[0], 1, q, nx_, &sRSQrq0, nu[0]+nx[0], nu[0]);
	d_print_strmat(nu[0]+nx[0]+1, nu[0]+nx[0], &sRSQrq0, 0, 0);
	blasfeo_allocate_dvec(nu[0]+nx[0], &srq0);
	blasfeo_pack_dvec(nu[0], r, &srq0, 0);
	blasfeo_pack_dvec(nx[0], q, &srq0, nu[0]);
	blasfeo_print_tran_dvec(nu[0]+nx[0], &srq0, 0);

	struct blasfeo_dmat sRSQrq1;
	struct blasfeo_dvec srq1;
	if(N>1)
		{
		blasfeo_allocate_dmat(nu[1]+nx[1]+1, nu[1]+nx[1], &sRSQrq1);
		blasfeo_pack_dmat(nu[1], nu[1], R, nu_, &sRSQrq1, 0, 0);
		blasfeo_pack_tran_dmat(nu[1], nx[1], S, nu_, &sRSQrq1, nu[1], 0);
		blasfeo_pack_dmat(nx[1], nx[1], Q, nx_, &sRSQrq1, nu[1], nu[1]);
		blasfeo_pack_tran_dmat(nu[1], 1, r, nu_, &sRSQrq1, nu[1]+nx[1], 0);
		blasfeo_pack_tran_dmat(nx[1], 1, q, nx_, &sRSQrq1, nu[1]+nx[1], nu[1]);
		d_print_strmat(nu[1]+nx[1]+1, nu[1]+nx[1], &sRSQrq1, 0, 0);
		blasfeo_allocate_dvec(nu[1]+nx[1], &srq1);
		blasfeo_pack_dvec(nu[1], r, &srq1, 0);
		blasfeo_pack_dvec(nx[1], q, &srq1, nu[1]);
		blasfeo_print_tran_dvec(nu[1]+nx[1], &srq1, 0);
		}

	struct blasfeo_dmat sRSQrqN;
	struct blasfeo_dvec srqN;
	blasfeo_allocate_dmat(nu[N]+nx[N]+1, nu[N]+nx[N], &sRSQrqN);
	blasfeo_pack_dmat(nu[N], nu[N], R, nu_, &sRSQrqN, 0, 0);
	blasfeo_pack_tran_dmat(nu[N], nx[N], S, nu_, &sRSQrqN, nu[N], 0);
	blasfeo_pack_dmat(nx[N], nx[N], Q, nx_, &sRSQrqN, nu[N], nu[N]);
	blasfeo_pack_tran_dmat(nu[N], 1, r, nu_, &sRSQrqN, nu[N]+nx[N], 0);
	blasfeo_pack_tran_dmat(nx[N], 1, q, nx_, &sRSQrqN, nu[N]+nx[N], nu[N]);
	d_print_strmat(nu[N]+nx[N]+1, nu[N]+nx[N], &sRSQrqN, 0, 0);
	blasfeo_allocate_dvec(nu[N]+nx[N], &srqN);
	blasfeo_pack_dvec(nu[N], r, &srqN, 0);
	blasfeo_pack_dvec(nx[N], q, &srqN, nu[N]);
	blasfeo_print_tran_dvec(nu[N]+nx[N], &srqN, 0);

/************************************************
* box & general constraints
************************************************/	

	int *idxb0; int_zeros(&idxb0, nb[0], 1);
	double *d0; d_zeros(&d0, 2*nb[0]+2*ng[0], 1);
	for(ii=0; ii<nb[0]; ii++)
		{
		if(ii<nu[0]) // input
			{
			d0[ii]             = - 0.5; // umin
			d0[nb[0]+ng[0]+ii] =   0.5; // umax
			}
		else // state
			{
			d0[ii]             = - 4.0; // xmin
			d0[nb[0]+ng[0]+ii] =   4.0; // xmax
			}
		idxb0[ii] = ii;
		}
	for(ii=0; ii<ng[0]; ii++)
		{
		d0[nb[0]+ii]             = - 100.0; // dmin
		d0[nb[0]+ng[0]+nb[0]+ii] =   100.0; // dmax
		}
	int_print_mat(1, nb[0], idxb0, 1);
	d_print_mat(1, 2*nb[0]+2*ng[0], d0, 1);

	int *idxb1; int_zeros(&idxb1, nb[1], 1);
	double *d1; d_zeros(&d1, 2*nb[1]+2*ng[1], 1);
	for(ii=0; ii<nb[1]; ii++)
		{
		if(ii<nu[1]) // input
			{
			d1[ii]             = - 0.5; // umin
			d1[nb[1]+ng[1]+ii] =   0.5; // umax
			}
		else // state
			{
			d1[ii]             = - 4.0; // xmin
			d1[nb[1]+ng[1]+ii] =   4.0; // xmax
			}
		idxb1[ii] = ii;
		}
	for(ii=0; ii<ng[1]; ii++)
		{
		d1[nb[1]+ii]             = - 100.0; // dmin
		d1[nb[1]+ng[1]+nb[1]+ii] =   100.0; // dmax
		}
	int_print_mat(1, nb[1], idxb1, 1);
	d_print_mat(1, 2*nb[1]+2*ng[1], d1, 1);

	int *idxbN; int_zeros(&idxbN, nb[N], 1);
	double *dN; d_zeros(&dN, 2*nb[N]+2*ng[N], 1);
	for(ii=0; ii<nb[N]; ii++)
		{
		if(ii<nu[N]) // input
			{
			dN[ii]             = - 0.5; // umin
			dN[nb[N]+ng[N]+ii] =   0.5; // umax
			}
		else // state
			{
			dN[ii]             = - 4.0; // xmin
			dN[nb[N]+ng[N]+ii] =   4.0; // xmax
			}
		idxbN[ii] = ii;
		}
	for(ii=0; ii<ng[N]; ii++)
		{
		dN[nb[N]+ii]             = - 100.0; // dmin
		dN[nb[N]+ng[N]+nb[N]+ii] =   100.0; // dmax
		}
	int_print_mat(1, nb[N], idxbN, 1);
	d_print_mat(1, 2*nb[N]+2*ng[N], dN, 1);

//	double *C; d_zeros(&C, ng_, nx_);
//	for(ii=0; ii<ng_; ii++)
//		C[ii*(ng_+1)] = 1.0;
//	double *D; d_zeros(&D, ng_, nu_);

	struct blasfeo_dmat sDCt0;
//	blasfeo_allocate_dmat(nu[0]+nx[0], ng[0], &sDCt0);
//	blasfeo_pack_tran_dmat(ng[0], nu[0], D, ng_, &sDCt0, 0, 0);
//	blasfeo_pack_tran_dmat(ng[0], nx[0], C, ng_, &sDCt0, nu[0], 0);
//	d_print_strmat(nu[0]+nx[0], ng[0], &sDCt0, 0, 0);
	struct blasfeo_dvec sd0;
	blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &sd0);
	blasfeo_pack_dvec(2*nb[0]+2*ng[0], d0, &sd0, 0);
	blasfeo_print_tran_dvec(2*nb[0]+2*ng[0], &sd0, 0);

	struct blasfeo_dmat sDCt1;
//	blasfeo_allocate_dmat(nu[1]+nx[1], ng[1], &sDCt1);
//	blasfeo_pack_tran_dmat(ng[1], nu[1], D, ng_, &sDCt1, 0, 0);
//	blasfeo_pack_tran_dmat(ng[1], nx[1], C, ng_, &sDCt1, nu[1], 0);
//	d_print_strmat(nu[1]+nx[1], ng[1], &sDCt1, 0, 0);
	struct blasfeo_dvec sd1;
	blasfeo_allocate_dvec(2*nb[1]+2*ng[1], &sd1);
	blasfeo_pack_dvec(2*nb[1]+2*ng[1], d1, &sd1, 0);
	blasfeo_print_tran_dvec(2*nb[1]+2*ng[1], &sd1, 0);

	struct blasfeo_dmat sDCtN;
//	blasfeo_allocate_dmat(nu[N]+nx[N], ng[N], &sDCtN);
//	blasfeo_pack_tran_dmat(ng[N], nu[N], D, ng_, &sDCtN, 0, 0);
//	blasfeo_pack_tran_dmat(ng[N], nx[N], C, ng_, &sDCtN, nu[N], 0);
//	d_print_strmat(nu[N]+nx[N], ng[N], &sDCtN, 0, 0);
	struct blasfeo_dvec sdN;
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &sdN);
	blasfeo_pack_dvec(2*nb[N]+2*ng[N], dN, &sdN, 0);
	blasfeo_print_tran_dvec(2*nb[N]+2*ng[N], &sdN, 0);

/************************************************
* libstr riccati solver
************************************************/	

	struct blasfeo_dmat *hsmatdummy;
	struct blasfeo_dvec *hsvecdummy;

	struct blasfeo_dmat hsBAbt[N];
	struct blasfeo_dvec hsb[N];
	struct blasfeo_dmat hsRSQrq[N+1];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dmat hsDCt[N+1];
	struct blasfeo_dvec hsd[N+1];
	int *hidxb[N+1];
	struct blasfeo_dvec hsux[N+1];
	struct blasfeo_dvec hspi[N+1];
	struct blasfeo_dvec hslam[N+1];
	struct blasfeo_dvec hst[N+1];
	struct blasfeo_dvec hsPb[N+1];
	struct blasfeo_dmat hsL[N+1];

	int nuM;
	int nbM;
	struct blasfeo_dmat sRSQrqM;
	struct blasfeo_dvec srqM;
	struct blasfeo_dvec srqM_tmp;
	struct blasfeo_dmat sLxM;
	struct blasfeo_dmat sPpM;
	struct blasfeo_dvec suxM;

	void *work_memory;


	hsBAbt[0] = sBAbt0;
	hsb[0] = sb0;
	hsRSQrq[0] = sRSQrq0;
	hsrq[0] = srq0;
	hsDCt[0] = sDCt0;
	hsd[0] = sd0;
	hidxb[0] = idxb0;
	blasfeo_allocate_dvec(nu[0]+nx[0], &hsux[0]);
	blasfeo_allocate_dvec(nx[1], &hspi[1]);
	blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &hslam[0]);
	blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &hst[0]);
	blasfeo_allocate_dvec(nx[1], &hsPb[1]);
	blasfeo_allocate_dmat(nu[0]+nx[0]+1, nu[0]+nx[0], &hsL[0]);
	for(ii=1; ii<N; ii++)
		{
		hsBAbt[ii] = sBAbt1;
		hsb[ii] = sb1;
		hsRSQrq[ii] = sRSQrq1;
		hsrq[ii] = srq1;
		hsDCt[ii] = sDCt1;
		hsd[ii] = sd1;
		hidxb[ii] = idxb1;
		blasfeo_allocate_dvec(nu[ii]+nx[ii], &hsux[ii]);
		blasfeo_allocate_dvec(nx[ii+1], &hspi[ii+1]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hslam[ii]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hst[ii]);
		blasfeo_allocate_dvec(nx[ii+1], &hsPb[ii+1]);
		blasfeo_allocate_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii]);
		}
	hsRSQrq[N] = sRSQrqN;
	hsrq[N] = srqN;
	hsDCt[N] = sDCtN;
	hsd[N] = sdN;
	hidxb[N] = idxbN;
	blasfeo_allocate_dvec(nu[N]+nx[N], &hsux[N]);
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hslam[N]);
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hst[N]);
	blasfeo_allocate_dmat(nu[N]+nx[N]+1, nu[N]+nx[N], &hsL[N]);
	
	blasfeo_allocate_dmat(nu[M]+nx[M]+1, nu[M]+nx[M], &sRSQrqM);
	blasfeo_allocate_dvec(nu[M]+nx[M], &srqM);
	blasfeo_allocate_dvec(nu[M]+nx[M], &srqM_tmp);
	blasfeo_allocate_dmat(nx[M]+1, nx[M], &sLxM);
	blasfeo_allocate_dmat(nx[M]+1, nx[M], &sPpM);
	blasfeo_allocate_dvec(nu[M]+nx[M], &suxM);

	// riccati work space
	void *work_ric;
	v_zeros_align(&work_ric, d_back_ric_rec_work_space_size_bytes_libstr(N, nx, nu, nb, ng));

	v_zeros_align(&work_memory, d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng));

	struct blasfeo_dmat hstmpmat0;
	struct blasfeo_dvec hstmpvec0;

	// IPM constants
	int hpmpc_status;
	int kk, kk_avg;
	int k_max = 10;
	double mu_tol = 1e-12;
	double alpha_min = 1e-8;
	int warm_start = 0; // read initial guess from x and u
	double *stat; d_zeros(&stat, k_max, 5);
	int compute_res = 1;
	int compute_mult = 1;


//	for(ii=0; ii<=N; ii++)
//		printf("\n%d\n", nb[ii]);
//	exit(1);



	struct timeval tv0, tv1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{

#if 1
		// backward riccati factorization and solution at the end
		d_back_ric_rec_sv_back_libstr(N-M, &nx[M], &nu[M], &nb[M], &hidxb[M], &ng[M], 0, &hsBAbt[M], hsvecdummy, 0, &hsRSQrq[M], hsvecdummy, hsmatdummy, hsvecdummy, hsvecdummy, &hsux[M], 1, &hspi[M], 1, &hsPb[M], &hsL[M], work_ric);
		// extract chol factor of [P p; p' *]
		d_print_strmat(nu[M]+nx[M]+1, nu[M]+nx[M], &hsL[M], 0, 0);
		blasfeo_dtrcp_l(nx[M], 1.0, &hsL[M], nu[M], nu[M], &sLxM, 0, 0); // TODO have m and n !!!!!
		blasfeo_dgecp(1, nx[M], 1.0, &hsL[M], nu[M]+nx[M], nu[M], &sLxM, nx[M], 0);
		d_print_strmat(nx[M]+1, nx[M], &sLxM, 0, 0);
		// recover [P p; p' *]
		blasfeo_dsyrk_ln(nx[M]+1, nx[M], nx[M], 1.0, &sLxM, 0, 0, &sLxM, 0, 0, 0.0, &sPpM, 0, 0, &sPpM, 0, 0);
		d_print_strmat(nx[M]+1, nx[M], &sPpM, 0, 0);
		// backup stage M
		nuM = nu[M];
		nbM = nb[M];
		hstmpmat0 = hsRSQrq[M];
		hstmpvec0 = hsux[M];
		// update new terminal cost
		nu[M] = 0;
		nb[M] = 0;
		hsRSQrq[M] = sPpM;
//		hsux[M].pa += nuM;
		hsux[M] = suxM;
		// IPM at the beginning
		hpmpc_status = d_ip2_res_mpc_hard_libstr(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, M, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, hsux, compute_mult, hspi, hslam, hst, work_memory);
		// recover original stage M
		nu[M] = nuM;
		nb[M] = nbM;
		hsRSQrq[M] = hstmpmat0;
//		hsux[M].pa -= nuM;
		hsux[M] = hstmpvec0;
		blasfeo_dveccp(nx[M], 1.0, &suxM, 0, &hsux[M], nu[M]);
		// forward riccati solution at the beginning
		d_back_ric_rec_sv_forw_libstr(N-M, &nx[M], &nu[M], &nb[M], &hidxb[M], &ng[M], 0, &hsBAbt[M], hsvecdummy, 0, &hsRSQrq[M], hsvecdummy, hsmatdummy, hsvecdummy, hsvecdummy, &hsux[M], 1, &hspi[M], 1, &hsPb[M], &hsL[M], work_ric);
//		exit(1);

#if 0
		blasfeo_drowex(nu[M]+nx[M], 1.0, &hsL[M], nu[M]+nx[M], 0, &srqM, 0);
		blasfeo_dsyrk_ln(nu[M]+nx[M]+1, nu[M]+nx[M], nu[M]+nx[M], 1.0, &hsL[M], 0, 0, &hsL[M], 0, 0, 0.0, &sRSQrqM, 0, 0, &sRSQrqM, 0, 0);
//		d_print_strmat(nu[M]+nx[M]+1, nu[M]+nx[M], &sRSQrqM, 0, 0);
//		blasfeo_print_tran_dvec(nu[M]+nx[M], &srqM, 0);
		dsetmat_libstr(nx[M], nx[M], 0.0, &sLxM, 0, 0);
		blasfeo_dtrcp_l(nx[M], 1.0, &hsL[M], nu[M], nu[M], &sLxM, 0, 0);
		blasfeo_dveccp(nx[M], 1.0, &srqM, nu[M], &srqM_tmp, 0);
//		d_print_strmat(nx[M], nx[M], &sLxM, 0, 0);
		blasfeo_dgemv_n(nx[M], nx[M], 1.0, &sLxM, 0, 0, &srqM_tmp, 0, 0.0, &srqM, nu[M], &srqM, nu[M]); // TODO implement dtrmv !!!!!
		blasfeo_print_tran_dvec(nu[M]+nx[M], &srqM, 0);
		exit(1);
		hstmpmat0 = hsRSQrq[M];
		hstmpvec0 = hsrq[M];
		hsRSQrq[M] = sRSQrqM;
		hsrq[M] = srqM;
		d_back_ric_rec_sv_libstr(M, &nx[0], &nu[0], nb, hidxb, ng, 0, &hsBAbt[0], hsvecdummy, 0, &hsRSQrq[0], hsvecdummy, hsmatdummy, hsvecdummy, hsvecdummy, &hsux[0], 1, &hspi[0], 1, &hsPb[0], &hsL[0], work_ric);
		d_back_ric_rec_trs_libstr(M, &nx[0], &nu[0], nb, hidxb, ng, &hsBAbt[0], &hsb[0], &hsrq[0], hsmatdummy, hsvecdummy, &hsux[0], 1, &hspi[0], 1, &hsPb[0], &hsL[0], work_ric);
//		hpmpc_status = d_ip2_res_mpc_hard_libstr(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, M, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, hsux, compute_mult, hspi, hslam, hst, work_memory);
		hsRSQrq[M] = hstmpmat0;
		hsrq[M] = hstmpvec0;
		d_back_ric_rec_sv_forw_libstr(N-M, &nx[M], &nu[M], &nb[M], &hidxb[M], &ng[M], 0, &hsBAbt[M], hsvecdummy, 0, &hsRSQrq[M], hsvecdummy, hsmatdummy, hsvecdummy, hsvecdummy, &hsux[M], 1, &hspi[M], 1, &hsPb[M], &hsL[M], work_ric);
#endif
#else
//		d_back_ric_rec_sv_libstr(N, nx, nu, nb, hidxb, ng, 0, hsBAbt, hsvecdummy, 0, hsRSQrq, hsvecdummy, hsmatdummy, hsvecdummy, hsvecdummy, hsux, 1, hspi, 1, hsPb, hsL, work_ric);
//		d_back_ric_rec_trs_libstr(N, &nx[0], &nu[0], nb, hidxb, ng, &hsBAbt[0], &hsb[0], &hsrq[0], hsmatdummy, hsvecdummy, &hsux[0], 1, &hspi[0], 1, &hsPb[0], &hsL[0], work_ric);
		hpmpc_status = d_ip2_res_mpc_hard_libstr(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, hsux, compute_mult, hspi, hslam, hst, work_memory);
#endif

		}

	gettimeofday(&tv1, NULL); // stop

	printf("\nux =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);

	printf("\npi =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nx[ii], &hspi[ii], 0);

//	printf("\nL =\n\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], 0, 0);

	printf("\nstat =\n\n");
	d_print_e_tran_mat(5, kk, stat, 5);

	double time_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* libstr ip2 residuals
************************************************/	

	struct blasfeo_dvec hsrrq[N+1];
	struct blasfeo_dvec hsrb[N];
	struct blasfeo_dvec hsrd[N+1];
	struct blasfeo_dvec hsrm[N+1];
	double mu;

	blasfeo_allocate_dvec(nu[0]+nx[0], &hsrrq[0]);
	blasfeo_allocate_dvec(nx[1], &hsrb[0]);
	blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &hsrd[0]);
	blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &hsrm[0]);
	for(ii=1; ii<N; ii++)
		{
		blasfeo_allocate_dvec(nu[ii]+nx[ii], &hsrrq[ii]);
		blasfeo_allocate_dvec(nx[ii+1], &hsrb[ii]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii]);
		}
	blasfeo_allocate_dvec(nu[N]+nx[N], &hsrrq[N]);
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hsrd[N]);
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hsrm[N]);

	void *work_res;
	v_zeros_align(&work_res, d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng));

//	printf("\nRSQrq =\n\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0);

	d_res_res_mpc_hard_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hslam, hst, hsrrq, hsrb, hsrd, hsrm, &mu, work_res);

	printf("\nres_rq\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nu[ii]+nx[ii], &hsrrq[ii], 0);

	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_exp_tran_dvec(nx[ii], &hsrb[ii], 0);

	printf("\nres_d\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], 0);

	printf("\nres_m\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], 0);

	printf(" Average solution time over %d runs: %5.2e seconds (IPM)\n", nrep, time_ipm);

/************************************************
* free memory
************************************************/	

	d_free(A);
	d_free(B);
	d_free(b);
	d_free(x0);

	blasfeo_free_dmat(&sA);
	blasfeo_free_dvec(&sx0);
	blasfeo_free_dmat(&sBAbt0);
	blasfeo_free_dvec(&sb0);
	blasfeo_free_dmat(&sRSQrq0);
	blasfeo_free_dvec(&srq0);
	blasfeo_free_dmat(&sRSQrqN);
	blasfeo_free_dvec(&srqN);
	if(N>1)
		{
		blasfeo_free_dmat(&sBAbt1);
		blasfeo_free_dvec(&sb1);
		blasfeo_free_dmat(&sRSQrq1);
		blasfeo_free_dvec(&srq1);
		}
	blasfeo_free_dvec(&hsux[0]);
	blasfeo_free_dvec(&hspi[1]);
	blasfeo_free_dvec(&hsrrq[0]);
	blasfeo_free_dvec(&hsrb[0]);
	for(ii=1; ii<N; ii++)
		{
		blasfeo_free_dvec(&hsux[ii]);
		blasfeo_free_dvec(&hspi[ii+1]);
		blasfeo_free_dvec(&hsrrq[ii]);
		blasfeo_free_dvec(&hsrb[ii]);
		}
	blasfeo_free_dvec(&hsux[N]);
	blasfeo_free_dvec(&hsrrq[N]);

/************************************************
* return
************************************************/	

	return 0;
	}
