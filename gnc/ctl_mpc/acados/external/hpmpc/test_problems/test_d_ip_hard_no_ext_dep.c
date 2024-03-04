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
#if defined(EXT_DEP)
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#endif
#endif

#if ! defined(EXT_DEP)
void d_zeros(double **pA, int row, int col)
	{
	*pA = malloc(row*col*sizeof(double));
	double *A = *pA;
	int ii;
	for(ii=0; ii<row*col; ii++)
		A[ii] = 0.0;
	return;
	}

void d_free(double *pA)
	{
	free(pA);
	return;
	}

void v_free(void *pA)
	{
	free( pA );
	}

void v_zeros(void **ptrA, int size)
	{
	*ptrA = (void *) malloc(size);
	char *A = *ptrA;
	int i;
	for(i=0; i<size; i++) A[i] = 0;
	}

void int_zeros(int **pA, int row, int col)
	{
	void *temp = malloc((row*col)*sizeof(int));
	*pA = temp;
	int *A = *pA;
	int i;
	for(i=0; i<row*col; i++) A[i] = 0;
	}

void int_free(int *pA)
	{
	free( pA );
	}

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

void d_print_e_tran_mat(int row, int col, double *A, int lda)
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

#include "../include/lqcp_solvers.h"
//#include "../include/mpc_aux.h"
#include "../include/mpc_solvers.h"
#include "../include/c_interface.h"
#include "tools.h"


#define KEEP_X0 0

// printing
#define PRINT 1

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
	
	int rep;

	int nx_ = 8; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = 3; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 10; // horizon lenght

	// partial condensing horizon
	int N2 = 1; //N/2;

	// maximum number of IPM iterations
	int k_max = 10;

	// exit tolerance in duality measure
	double mu_tol = 1e-10;

	// minimum step size lenght 
	double alpha_min = 1e-8;

	// number of calls to solver (for more accurate timings)
	int nrep=1000;



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

	printf(" Test problem: mass-spring system with %d masses and %d controls.\n", nx_/2, nu_);
	printf("\n");
	printf(" MPC problem size: %d states, %d inputs, %d horizon length, %d two-sided box constraints, %d two-sided general constraints.\n", nx[1], nu[1], N, nb[1], ng[1]);
	printf("\n");
	printf(" IP method parameters: predictor-corrector IP, double precision, %d maximum iterations, %5.1e exit tolerance in duality measure (edit file test_param.c to change them).\n", k_max, mu_tol);



/************************************************
* dynamical system
************************************************/	

	double *A; // states update matrix
	d_zeros(&A, nx_, nx_);

	double *B; // inputs matrix
	d_zeros(&B, nx_, nu_);

	double *b; // states offset
	d_zeros(&b, nx_, 1);

	double *x0; // initial state
	d_zeros(&x0, nx_, 1);

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx_, nu_, N, A, B, b, x0);
	
	for(jj=0; jj<nx_; jj++)
		b[jj] = 0.1;
	
	for(jj=0; jj<nx_; jj++)
		x0[jj] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

#if PRINT
	d_print_mat(nx_, nx_, A, nx_);
	d_print_mat(nx_, nu_, B, nu_);
	d_print_mat(1, nx_, b, 1);
	d_print_mat(1, nx_, x0, 1);
#endif

	double *b0;
	d_zeros(&b0, nx_, 1);
#if ! KEEP_X0
	for(ii=0; ii<nx_; ii++)
		b0[ii] = b[ii];
	for(ii=0; ii<nx_; ii++)
		for(jj=0; jj<nx_; jj++)
			b0[ii] += A[ii+nx_*jj] * x0[jj];
#endif

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

	// maximum element in cost functions
	double mu0 = 2.0;

/************************************************
* box & general constraints
************************************************/	

	double *lb0;
	d_zeros(&lb0, nb[0], 1);
	double *ub0;
	d_zeros(&ub0, nb[0], 1);
	double *lg0;
	d_zeros(&lg0, ng[0], 1);
	double *ug0;
	d_zeros(&ug0, ng[0], 1);
	int *idxb0; 
	int_zeros(&idxb0, nb[0], 1);
	for(ii=0; ii<nb[0]; ii++)
		{
		if(ii<nu[0]) // input
			{
			lb0[ii] = - 0.5; // umin
			ub0[ii] =   0.5; // umax
			}
		else // state
			{
			lb0[ii] = - 4.0; // xmin
			ub0[ii] =   4.0; // xmax
			}
		idxb0[ii] = ii;
		}
	for(ii=0; ii<ng[0]; ii++)
		{
		if(ii<nu[0]) // input
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
#if PRINT
	int_print_mat(1, nb[0], idxb0, 1);
	d_print_mat(1, nb[0], lb0, 1);
	d_print_mat(1, nb[0], ub0, 1);
	d_print_mat(1, ng[0], lg0, 1);
	d_print_mat(1, ng[0], ug0, 1);
#endif

	double *lb1;
	d_zeros(&lb1, nb[1], 1);
	double *ub1;
	d_zeros(&ub1, nb[1], 1);
	double *lg1;
	d_zeros(&lg1, ng[1], 1);
	double *ug1;
	d_zeros(&ug1, ng[1], 1);
	int *idxb1; int_zeros(&idxb1, nb[1], 1);
	for(ii=0; ii<nb[1]; ii++)
		{
		if(ii<nu[1]) // input
			{
			lb1[ii] = - 0.5; // umin
			ub1[ii] =   0.5; // umax
			}
		else // state
			{
			lb1[ii] = - 4.0; // xmin
			ub1[ii] =   4.0; // xmax
			}
		idxb1[ii] = ii;
		}
	for(ii=0; ii<ng[1]; ii++)
		{
		if(ii<nu[1]) // input
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
#if PRINT
	int_print_mat(1, nb[1], idxb1, 1);
	d_print_mat(1, nb[1], lb1, 1);
	d_print_mat(1, nb[1], ub1, 1);
	d_print_mat(1, ng[1], lg1, 1);
	d_print_mat(1, ng[1], ug1, 1);
#endif

	double *lbN;
	d_zeros(&lbN, nb[N], 1);
	double *ubN;
	d_zeros(&ubN, nb[N], 1);
	double *lgN;
	d_zeros(&lgN, ng[N], 1);
	double *ugN;
	d_zeros(&ugN, ng[N], 1);
	int *idxbN; int_zeros(&idxbN, nb[N], 1);
	for(ii=0; ii<nb[N]; ii++)
		{
		lbN[ii] = - 4.0; // xmin
		ubN[ii] =   4.0; // xmax
		idxbN[ii] = ii;
		}
	for(ii=0; ii<ng[N]; ii++)
		{
		lgN[ii] = - 4.0; // dmin
		ugN[ii] =   4.0; // dmax
		}
#if PRINT
	int_print_mat(1, nb[N], idxbN, 1);
	d_print_mat(1, nb[N], lbN, 1);
	d_print_mat(1, nb[N], ubN, 1);
	d_print_mat(1, ng[N], lgN, 1);
	d_print_mat(1, ng[N], ugN, 1);
#endif

	double *DC0; d_zeros(&DC0, ng[0], nu[0]+nx[0]);
	for(ii=0; ii<ng[0]; ii++)
		DC0[ii*(ng[0]+1)] = 1.0;
	double *DC1; d_zeros(&DC1, ng[1], nu[1]+nx[1]);
	for(ii=0; ii<ng[1]; ii++)
		DC1[ii*(ng[1]+1)] = 1.0;
	double *DCN; d_zeros(&DCN, ng[N], nx[N]);
	for(ii=0; ii<ng[N]; ii++)
		DCN[ii*(ng[N]+1)] = 1.0;

	double *D0;
	d_zeros(&D0, ng[0], nu[0]);
	for(jj=0; jj<nu[0]; jj++)
		for(ii=0; ii<ng[0]; ii++)
			D0[ii+ng[0]*jj] = DC0[ii+ng[0]*jj];

	double *C0;
	d_zeros(&C0, ng[0], nx[0]);
	for(jj=0; jj<nx[0]; jj++)
		for(ii=0; ii<ng[0]; ii++)
			C0[ii+ng[0]*jj] = DC0[ii+ng[0]*(nu[0]+jj)];

	double *D1;
	d_zeros(&D1, ng[1], nu[1]);
	for(jj=0; jj<nu[1]; jj++)
		for(ii=0; ii<ng[1]; ii++)
			D1[ii+ng[1]*jj] = DC1[ii+ng[1]*jj];

	double *C1;
	d_zeros(&C1, ng[1], nx[1]);
	for(jj=0; jj<nx[1]; jj++)
		for(ii=0; ii<ng[1]; ii++)
			C1[ii+ng[1]*jj] = DC1[ii+ng[1]*(nu[1]+jj)];

	double *CN;
	d_zeros(&CN, ng[N], nx[N]);
	for(jj=0; jj<nx[N]; jj++)
		for(ii=0; ii<ng[N]; ii++)
			CN[ii+ng[N]*jj] = DCN[ii+ng[N]*(nu[N]+jj)];

/************************************************
* high-level interface
************************************************/	
	
	double *hA[N];
	double *hB[N];
	double *hb[N];
	double *hR[N];
	double *hS[N];
	double *hQ[N+1];
	double *hr[N];
	double *hq[N+1];
	double *hlb[N+1];
	double *hub[N+1];
	int *hidxb[N+1];
	double *hC[N+1];
	double *hD[N];
	double *hlg[N+1];
	double *hug[N+1];
	double *hx[N+1];
	double *hu[N];
	double *hpi[N];
	double *hlam[N+1];
	double inf_norm_res[5];

	ii = 0;
	hB[ii] = B;
	hb[ii] = b0;
	hC[ii] = C0;
	hD[ii] = D0;
	hR[ii] = R;
	hr[ii] = r; // XXX r0
	hlb[ii] = lb0;
	hub[ii] = ub0;
	hidxb[0] = idxb0;
	hlg[ii] = lg0;
	hug[ii] = ug0;
	d_zeros(&hu[ii], nu[ii], 1);
	d_zeros(&hx[ii], nx[ii], 1);
	d_zeros(&hpi[ii], nx[ii+1], 1);
	d_zeros(&hlam[ii], 2*nb[ii]+2*ng[ii], 1);
	for(ii=1; ii<N; ii++)
		{
		hA[ii] = A;
		hB[ii] = B;
		hb[ii] = b;
		hC[ii] = C1;
		hD[ii] = D1;
		hR[ii] = R;
		hS[ii] = S;
		hQ[ii] = Q;
		hr[ii] = r;
		hq[ii] = q;
		hlb[ii] = lb1;
		hub[ii] = ub1;
		hidxb[ii] = idxb1;
		hlg[ii] = lg1;
		hug[ii] = ug1;
		d_zeros(&hu[ii], nu[ii], 1);
		d_zeros(&hx[ii], nx[ii], 1);
		d_zeros(&hpi[ii], nx[ii+1], 1);
		d_zeros(&hlam[ii], 2*nb[ii]+2*ng[ii], 1);
		}
	ii = N;
	hC[ii] = CN;
	hQ[ii] = Q;
	hq[ii] = q;
	hlb[ii] = lbN;
	hub[ii] = ubN;
	hidxb[N] = idxbN;
	hlg[ii] = lgN;
	hug[ii] = ugN;
	d_zeros(&hx[ii], nx[ii], 1);
	d_zeros(&hlam[ii], 2*nb[ii]+2*ng[ii], 1);
	
	// IP options
	int kk = -1;
	int warm_start = 0;
	double stat[5*k_max];

	int hpmpc_exit;

	struct timeval tv0, tv1;

	void *work_ipm_high;
	v_zeros(&work_ipm_high, hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(N, nx, nu, nb, hidxb, ng, N2));
	printf("\nwork space size 2 (in bytes): %d\n", hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(N, nx, nu, nb, hidxb, ng, N2));

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{

		hpmpc_exit = fortran_order_d_ip_ocp_hard_tv(&kk, k_max, mu0, mu_tol, N, nx, nu, nb, hidxb, ng, N2, warm_start, hA, hB, hb, hQ, hS, hR, hq, hr, hlb, hub, hC, hD, hlg, hug, hx, hu, hpi, hlam, inf_norm_res, work_ipm_high, stat);
		}

	gettimeofday(&tv1, NULL); // stop

	printf("\nstat = (%d iter)\n\nsigma\t\talpha1\t\tmu1\t\talpha2\t\tmu2\n\n", kk);
	d_print_e_tran_mat(5, kk, stat, 5);
#if PRINT

	printf("\nu = \n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nu[ii], hu[ii], 1);

	printf("\nx = \n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx[ii], hx[ii], 1);
#endif
	
	printf("\ninf norm res = %e, %e, %e, %e, %e\n\n", inf_norm_res[0], inf_norm_res[1], inf_norm_res[2], inf_norm_res[3], inf_norm_res[4]); 

	double time_ipm_high = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf(" Average solution time over %d runs: %5.2e seconds (IPM high)\n\n", nrep, time_ipm_high);

/************************************************
* high-level interface (resolve last kkt new rhs)
************************************************/	
	
	// change constraints
//	for(ii=0; ii<nb[0]; ii++) lb0[ii] = -0.51;
//	for(ii=0; ii<nb[0]; ii++) ub0[ii] =  0.51;

	// zero solution
	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nu[ii]; jj++)
			hu[ii][jj] = 0.0;
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<nx[ii]; jj++)
			hx[ii][jj] = 0.0;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{

		fortran_order_d_ip_last_kkt_new_rhs_ocp_hard_libstr(N, nx, nu, nb, hidxb, ng, N2, hb, hq, hr, hlb, hub, hlg, hug, hx, hu, hpi, hlam, inf_norm_res, work_ipm_high);
		}

	gettimeofday(&tv1, NULL); // stop

#if PRINT
	printf("\nu = \n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nu[ii], hu[ii], 1);

	printf("\nx = \n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx[ii], hx[ii], 1);
#endif
	
	printf("\ninf norm res = %e, %e, %e, %e, %e\n\n", inf_norm_res[0], inf_norm_res[1], inf_norm_res[2], inf_norm_res[3], inf_norm_res[4]); 

	double time_ipm_high_kkt = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf(" Average solution time over %d runs: %5.2e seconds (IPM high kkt last rhs)\n\n", nrep, time_ipm_high_kkt);

/************************************************
* free memory
************************************************/	

	d_free(A);
	d_free(B);
	d_free(b);
	d_free(x0);
	d_free(b0);
	d_free(Q);
	d_free(S);
	d_free(R);
	d_free(q);
	d_free(r);
	int_free(idxb0);
	d_free(lb0);
	d_free(ub0);
	d_free(lg0);
	d_free(ug0);
	int_free(idxb1);
	d_free(lb1);
	d_free(ub1);
	d_free(lg1);
	d_free(ug1);
	int_free(idxbN);
	d_free(lbN);
	d_free(ubN);
	d_free(lgN);
	d_free(ugN);
	d_free(DC0);
	d_free(DC1);
	d_free(DCN);
	d_free(C0);
	d_free(D0);
	d_free(C1);
	d_free(D1);
	d_free(CN);
	v_free(work_ipm_high);

/************************************************
* return
************************************************/	

	return 0;
	}

