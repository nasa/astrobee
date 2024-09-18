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
#include "../include/blas_d.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_aux.h"
#include "../include/mpc_solvers.h"
#include "../include/block_size.h"
#include "tools.h"
#include "../include/c_interface.h"



// XXX
//#include "../lqcp_solvers/d_part_cond.c"



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
	
	int rep, nrep=1000; //000;//NREP;

	int nx_ = 8;//NX; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = 3;//NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 10;//NN; // horizon lenght
//	int nb  = nu+nx; // number of box constrained inputs and states
//	int ng  = nx; //4;  // number of general constraints
//	int ngN = nx; // number of general constraints at the last stage
	printf("\nN = %d, nx = %d, nu = %d\n\n", N, nx_, nu_);

#define MHE 0


//	int nbu = nu<nb ? nu : nb ;
//	int nbx = nb-nu>0 ? nb-nu : 0;


	// stage-wise variant size
	int nx[N+1];
#if MHE==1
	nx[0] = nx_;
#else
	nx[0] = 0;
#endif
	for(ii=1; ii<=N; ii++)
		nx[ii] = nx_;

	int nu[N+1];
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_;
	nu[N] = 0; // XXX

	int nb[N+1];
	nb[0] = nu[0] + nx[0]/2;
	for(ii=1; ii<N; ii++)
		nb[ii] = nu[1] + nx[ii]/2;
	nb[N] = nu[N] + nx[N]/2;

	int ng[N+1];
	for(ii=0; ii<N; ii++)
		ng[ii] = 0; //ng;
	ng[N] = 0; //ngN;
//	ng[M] = nx_; // XXX
	

/************************************************
* IPM common arguments
************************************************/	

	int hpmpc_status;
	int kk = -1;
	int k_max = 10;
	double mu0 = 2.0;
	double mu_tol = 1e-20;
	double alpha_min = 1e-8;
	int warm_start = 0; // read initial guess from x and u
	double *stat; d_zeros(&stat, k_max, 5);
	int compute_res = 1;
	int compute_mult = 1;

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

#if MHE!=1
	struct blasfeo_dvec sx0;
	blasfeo_allocate_dvec(nx_, &sx0);
	blasfeo_pack_dvec(nx_, x0, &sx0, 0);
	struct blasfeo_dvec sb;
	blasfeo_allocate_dvec(nx_, &sb);
	blasfeo_pack_dvec(nx_, b, &sb, 0);
	struct blasfeo_dmat sA;
	blasfeo_allocate_dmat(nx_, nx_, &sA);
	blasfeo_pack_dmat(nx_, nx_, A, nx_, &sA, 0, 0);
	struct blasfeo_dvec sb0;
	blasfeo_allocate_dvec(nx_, &sb0);
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb, 0, &sb0, 0);

	struct blasfeo_dmat sBAbt0;
	blasfeo_allocate_dmat(nu[0]+1, nx[1], &sBAbt0);
	blasfeo_pack_tran_dmat(nx_, nu_, B, nx_, &sBAbt0, 0, 0);
	blasfeo_drowin(nx[1], 1.0, &sb0, 0, &sBAbt0, nu[0], 0);
//	d_print_strmat(nu[0]+1, nx[1], &sBAbt0, 0, 0);
#endif

	struct blasfeo_dmat sBAbt1;
	if(N>1)
		{
		blasfeo_allocate_dmat(nu[1]+nx[1]+1, nx[2], &sBAbt1);
		blasfeo_pack_tran_dmat(nx_, nu_, B, nx_, &sBAbt1, 0, 0);
		blasfeo_pack_tran_dmat(nx_, nx_, A, nx_, &sBAbt1, nu[1], 0);
		blasfeo_pack_tran_dmat(nx_, 1, b, nx_, &sBAbt1, nu[1]+nx[1], 0);
//		d_print_strmat(nu[1]+nx[1]+1, nx[2], &sBAbt1, 0, 0);
		}
	
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

#if MHE!=1
	struct blasfeo_dvec sr;
	blasfeo_allocate_dvec(nu_, &sr);
	blasfeo_pack_dvec(nu_, r, &sr, 0);
	struct blasfeo_dmat sS;
	blasfeo_allocate_dmat(nu_, nx_, &sS);
	blasfeo_pack_dmat(nu_, nx_, S, nu_, &sS, 0, 0);
	struct blasfeo_dvec sr0;
	blasfeo_allocate_dvec(nu_, &sr0);
	blasfeo_dgemv_n(nu_, nx_, 1.0, &sS, 0, 0, &sx0, 0, 1.0, &sr, 0, &sr0, 0);

	struct blasfeo_dmat sRSQrq0;
	blasfeo_allocate_dmat(nu[0]+nx[0]+1, nu[0]+nx[0], &sRSQrq0);
	blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRSQrq0, 0, 0);
	blasfeo_drowin(nu[0], 1.0, &sr0, 0, &sRSQrq0, nu[0], 0);
//	d_print_strmat(nu[0]+nx[0]+1, nu[0]+nx[0], &sRSQrq0, 0, 0);

	struct blasfeo_dvec srq0;
	blasfeo_allocate_dvec(nu[0]+nx[0], &srq0);
	blasfeo_dveccp(nu[0], 1.0, &sr0, 0, &srq0, 0);
#endif

	struct blasfeo_dmat sRSQrq1;
	struct blasfeo_dvec srq1;
	if(N>1)
		{
		blasfeo_allocate_dmat(nu[1]+nx[1]+1, nu[1]+nx[1], &sRSQrq1);
		blasfeo_pack_dmat(nu_, nu_, R, nu_, &sRSQrq1, 0, 0);
		blasfeo_pack_tran_dmat(nu_, nx_, S, nu_, &sRSQrq1, nu[1], 0);
		blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sRSQrq1, nu[1], nu[1]);
		blasfeo_pack_tran_dmat(nu_, 1, r, nu_, &sRSQrq1, nu[1]+nx[1], 0);
		blasfeo_pack_tran_dmat(nx_, 1, q, nx_, &sRSQrq1, nu[1]+nx[1], nu[1]);
//		d_print_strmat(nu[1]+nx[1]+1, nu[1]+nx[1], &sRSQrq1, 0, 0);

		blasfeo_allocate_dvec(nu[1]+nx[1], &srq1);
		blasfeo_pack_dvec(nu_, r, &srq1, 0);
		blasfeo_pack_dvec(nx_, q, &srq1, nu[1]);
		}

	struct blasfeo_dmat sRSQrqN;
	blasfeo_allocate_dmat(nx[N]+1, nx[N], &sRSQrqN);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sRSQrqN, 0, 0);
	blasfeo_pack_tran_dmat(nx_, 1, q, nx_, &sRSQrqN, nx[1], 0);
//	d_print_strmat(nu[N]+nx[N]+1, nu[N]+nx[N], &sRSQrqN, 0, 0);

	struct blasfeo_dvec srqN;
	blasfeo_allocate_dvec(nx[N], &srqN);
	blasfeo_pack_dvec(nx_, q, &srqN, 0);

/************************************************
* constraints
************************************************/	

#if MHE!=1
	double *d0; d_zeros(&d0, 2*nb[0]+2*ng[0], 1);
	int *idxb0; int_zeros(&idxb0, nb[0], 1);
	// inputs
	for(ii=0; ii<nu[0]; ii++)
		{
		d0[ii]             = - 0.5; // u_min
		d0[nb[0]+ng[0]+ii] = + 0.5; // u_max
		idxb0[ii] = ii;
		}
	// states
	for( ; ii<nb[0]; ii++)
		{
		d0[ii]             = - 4.0; // x_min
		d0[nb[0]+ng[0]+ii] = + 4.0; // x_max
		idxb0[ii] = ii;
		}
#endif

	double *d1; 
	int *idxb1; 
	if(N>1)
		{
		d_zeros(&d1, 2*nb[1]+2*ng[1], 1);
		int_zeros(&idxb1, nb[1], 1);
		// inputs
		for(ii=0; ii<nu[1]; ii++)
			{
			d1[ii]             = - 0.5; // u_min
			d1[nb[1]+ng[1]+ii] = + 0.5; // u_max
			idxb1[ii] = ii;
			}
		// states
		for( ; ii<nb[1]; ii++)
			{
			d1[ii]             = - 4.0; // x_min
			d1[nb[1]+ng[1]+ii] = + 4.0; // x_max
			idxb1[ii] = ii;
			}
		}

	double *dN; d_zeros(&dN, 2*nb[N]+2*ng[N], 1);
	int *idxbN; int_zeros(&idxbN, nb[N], 1);
	// no inputs
	// states
	for(ii=0 ; ii<nb[N]; ii++)
		{
		dN[ii]             = - 4.0; // x_min
		dN[nb[N]+ng[N]+ii] = + 4.0; // x_max
		idxbN[ii] = ii;
		}

	struct blasfeo_dvec sd0;
	blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &sd0);
	blasfeo_pack_dvec(2*nb[0]+2*ng[0], d0, &sd0, 0);
//	blasfeo_print_tran_dvec(2*nb[0], &sd0, 0);

	struct blasfeo_dvec sd1;
	blasfeo_allocate_dvec(2*nb[1]+2*ng[1], &sd1);
	blasfeo_pack_dvec(2*nb[1]+2*ng[1], d1, &sd1, 0);
//	blasfeo_print_tran_dvec(2*nb[1], &sd1, 0);

	struct blasfeo_dvec sdN;
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &sdN);
	blasfeo_pack_dvec(2*nb[N]+2*ng[N], dN, &sdN, 0);
//	blasfeo_print_tran_dvec(2*nb[N], &sdN, 0);

/************************************************
* array of data matrices
************************************************/	

	// original MPC
	struct blasfeo_dmat hsBAbt[N];
	struct blasfeo_dvec hsb[N];
	struct blasfeo_dmat hsRSQrq[N+1];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dmat hsDCt[N+1]; // XXX
	struct blasfeo_dvec hsd[N+1];
	int *hidxb[N+1];

	ii = 0;
#if MHE!=1
	hsBAbt[ii] = sBAbt0;
	hsb[ii] = sb0;
	hsRSQrq[ii] = sRSQrq0;
	hsrq[ii] = srq0;
	hsd[ii] = sd0;
	hidxb[0] = idxb0;
#else
	hsBAbt[ii] = sBAbt1;
	hsb[ii] = sb;
	hsRSQrq[ii] = sRSQrq1;
	hsrq[ii] = srq1;
	hsd[ii] = sd1;
	hidxb[0] = idxb1;
#endif

	for(ii=1; ii<N; ii++)
		{
		hsBAbt[ii] = sBAbt1;
		hsb[ii] = sb;
		hsRSQrq[ii] = sRSQrq1;
		hsrq[ii] = srq1;
		hsd[ii] = sd1;
		hidxb[ii] = idxb1;
		}
	hsRSQrq[ii] = sRSQrqN;
	hsrq[ii] = srqN;
	hsd[ii] = sdN;
	hidxb[N] = idxbN;

/************************************************
* solve full spase system using Riccati / IPM
************************************************/	

	// result vectors
	struct blasfeo_dvec hsux[N+1];
	struct blasfeo_dvec hspi[N+1];
	struct blasfeo_dvec hslam[N+1];
	struct blasfeo_dvec hst[N+1];
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_allocate_dvec(nu[ii]+nx[ii], &hsux[ii]);
		blasfeo_allocate_dvec(nx[ii], &hspi[ii]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hslam[ii]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hst[ii]);
		}

	// work space
	void *work_space_ipm;
	v_zeros_align(&work_space_ipm, d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng));

	struct timeval tv0, tv1;

	printf("\nsolving... (full space system)\n");

	gettimeofday(&tv0, NULL); // stop

	for(rep=0; rep<nrep; rep++)
		{
		hpmpc_status = d_ip2_res_mpc_hard_libstr(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, hsux, 1, hspi, hslam, hst, work_space_ipm);
		}

	gettimeofday(&tv1, NULL); // stop

	printf("\n... done\n");

	float time_ipm_full = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
	printf("\nux =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);

	printf("\npi =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nx[ii], &hspi[ii], 0);

	printf("\nlam =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0);

	printf("\nt =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0);

	// residuals vectors
	struct blasfeo_dvec hsrrq[N+1];
	struct blasfeo_dvec hsrb[N+1];
	struct blasfeo_dvec hsrd[N+1];
	struct blasfeo_dvec hsrm[N+1];
	double mu;

	for(ii=0; ii<N; ii++)
		{
		blasfeo_allocate_dvec(nu[ii]+nx[ii], &hsrrq[ii]);
		blasfeo_allocate_dvec(nx[ii+1], &hsrb[ii]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii]);
		blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii]);
		}
	blasfeo_allocate_dvec(nu[N]+nx[N], &hsrrq[N]);
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hsrd[N]);
	blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hsrm[N]);

	int ngM = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}

	void *work_space_res;
	v_zeros_align(&work_space_res, d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng));

	d_res_res_mpc_hard_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hslam, hst, hsrrq, hsrb, hsrd, hsrm, &mu, work_space_res);

	printf("\nres_rq\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nu[ii]+nx[ii], &hsrrq[ii], 0);

	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_exp_tran_dvec(nx[ii+1], &hsrb[ii], 0);

	printf("\nres_d\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], 0);

	printf("\nres_m\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], 0);

/************************************************
* full condensing
************************************************/	

	// condensed problem size
	int N2 = 1;

	int nx2[N2+1];
	int nu2[N2+1];
	int nb2[N2+1];
	int ng2[N2+1];

	d_cond_compute_problem_size_libstr(N, nx, nu, nb, hidxb, ng, nx2, nu2, nb2, ng2);
	
#if 0
	for(ii=0; ii<=N2; ii++)
		printf("\n%d %d %d %d\n", nx2[ii], nu2[ii], nb2[ii], ng2[ii]);
#endif

	int work_sizes_cond[5];
	int work_size_cond = d_cond_work_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, nx2, nu2, nb2, ng2, work_sizes_cond);
	int memo_size_cond = d_cond_memory_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, nx, nu2, nb2, ng2);
	int work_size_ipm_cond = d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N2, nx2, nu2, nb2, ng2);
	int work_sizes_expa[2];
	int work_size_expa = d_expand_work_space_size_bytes_libstr(N, nx, nu, nb, ng, work_sizes_expa);

	// work space
	void *work_cond;
	void *memo_cond;
	void *work_ipm_cond;
	void *work_expa;

	v_zeros_align(&work_cond, work_size_cond);
	v_zeros_align(&memo_cond, memo_size_cond);
	v_zeros_align(&work_ipm_cond, work_size_ipm_cond);
	v_zeros_align(&work_expa, work_size_expa);

	// data matrices
	struct blasfeo_dmat hsBAbt2[N2];
	struct blasfeo_dvec hsb2[N2];
	struct blasfeo_dmat hsRSQrq2[N2+1];
	struct blasfeo_dvec hsrq2[N2+1];
	struct blasfeo_dmat hsDCt2[N2+1];
	struct blasfeo_dvec hsd2[N2+1];
	int *hidxb2[N2+1];

	for(ii=0; ii<N2; ii++)
		blasfeo_allocate_dmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii]);
	
	for(ii=0; ii<N2; ii++)
		blasfeo_allocate_dvec(nx2[ii+1], &hsb2[ii]);
	
	for(ii=0; ii<=N2; ii++)
		blasfeo_allocate_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii]);
	
	for(ii=0; ii<=N2; ii++)
		blasfeo_allocate_dvec(nu2[ii]+nx2[ii], &hsrq2[ii]);
	
	for(ii=0; ii<=N2; ii++)
		blasfeo_allocate_dmat(nu2[ii]+nx2[ii]+1, ng2[ii], &hsDCt2[ii]);
	
	for(ii=0; ii<=N2; ii++)
		blasfeo_allocate_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii]);
	
	for(ii=0; ii<=N2; ii++)
		int_zeros(&hidxb2[ii], nb2[ii], 1);
	
	// result vectors
	struct blasfeo_dvec hsux2[N2+1];
	struct blasfeo_dvec hspi2[N2+1];
	struct blasfeo_dvec hslam2[N2+1];
	struct blasfeo_dvec hst2[N2+1];
	for(ii=0; ii<=N2; ii++)
		{
		blasfeo_allocate_dvec(nu2[ii]+nx2[ii], &hsux2[ii]);
		blasfeo_allocate_dvec(nx2[ii], &hspi2[ii]);
		blasfeo_allocate_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii]);
		blasfeo_allocate_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii]);
		}

	d_cond_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsRSQrq2, hsDCt2, hsd2, memo_cond, work_cond, work_sizes_cond);

#if 0
	printf("\nBAbt2\n");
	for(ii=0; ii<N2; ii++)
		d_print_strmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii], 0, 0);
	printf("\nRSQrq2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_strmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii], 0, 0);
	printf("\nDCt2\n");
	for(ii=0; ii<=N2; ii++)
		d_print_strmat(nu2[ii]+nx2[ii], ng2[ii], &hsDCt2[ii], 0, 0);
	printf("\nd2\n");
	for(ii=0; ii<=N2; ii++)
		blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii], 0);
#endif


/************************************************
* solve condensed system using IPM
************************************************/	
	
	// zero solution
	for(ii=0; ii<=N; ii++)
		blasfeo_dvecse(nu[ii]+nx[ii], 0.0, &hsux[ii], 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_dvecse(nx[ii], 0.0, &hspi[ii], 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_dvecse(2*nb[ii]+2*ng[ii], 0.0, &hslam[ii], 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_dvecse(2*nb[ii]+2*ng[ii], 0.0, &hst[ii], 0);

	printf("\nsolving... (condensed system)\n");

	gettimeofday(&tv0, NULL); // stop

	for(rep=0; rep<nrep; rep++)
		{

		d_cond_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsRSQrq2, hsDCt2, hsd2, memo_cond, work_cond, work_sizes_cond);

		hpmpc_status = d_ip2_res_mpc_hard_libstr(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N2, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsRSQrq2, hsDCt2, hsd2, hsux2, 1, hspi2, hslam2, hst2, work_ipm_cond);

		d_expand_solution_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsDCt, hsux, hspi, hslam, hst, nx2, nu2, nb2, hidxb2, ng2, hsux2, hspi2, hslam2, hst2, work_expa, work_sizes_expa);

		}

	gettimeofday(&tv1, NULL); // stop

	printf("\n... done\n");

	float time_ipm_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
#if 0
	printf("\nux2 =\n\n");
	for(ii=0; ii<=N2; ii++)
		blasfeo_print_tran_dvec(nu2[ii]+nx2[ii], &hsux2[ii], 0);

	printf("\npi2 =\n\n");
	for(ii=0; ii<=N2; ii++)
		blasfeo_print_tran_dvec(nx2[ii], &hspi2[ii], 0);

	printf("\nlam2 =\n\n");
	for(ii=0; ii<=N2; ii++)
		blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii], 0);

	printf("\nt2 =\n\n");
	for(ii=0; ii<=N2; ii++)
		blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii], 0);
#endif

	printf("\nux =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);

	printf("\npi =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nx[ii], &hspi[ii], 0);

	printf("\nlam =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0);

	printf("\nt =\n\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0);

/************************************************
* free memory full space
************************************************/	

	// TODO
	d_free(A);
	d_free(B);
	d_free(b);
	d_free(x0);
	d_free(R);
	d_free(S);
	d_free(Q);
	d_free(r);
	d_free(q);
	d_free(d0);
	int_free(idxb0);
	d_free(d1);
	int_free(idxb1);
	d_free(dN);
	int_free(idxbN);

	v_free_align(work_space_ipm);

	blasfeo_free_dvec(&sx0);
	blasfeo_free_dvec(&sb);
	blasfeo_free_dmat(&sA);
	blasfeo_free_dvec(&sb0);
	blasfeo_free_dmat(&sBAbt0);
	if(N>1)
		blasfeo_free_dmat(&sBAbt1);
	blasfeo_free_dvec(&sr);
	blasfeo_free_dmat(&sS);
	blasfeo_free_dvec(&sr0);
	blasfeo_free_dmat(&sRSQrq0);
	blasfeo_free_dvec(&srq0);
	if(N>1)
		blasfeo_free_dmat(&sRSQrq1);
	if(N>1)
		blasfeo_free_dvec(&srq1);
	blasfeo_free_dmat(&sRSQrqN);
	blasfeo_free_dvec(&srqN);
	blasfeo_free_dvec(&sd0);
	blasfeo_free_dvec(&sd1);
	blasfeo_free_dvec(&sdN);
	for(ii=0; ii<N; ii++)
		{
		blasfeo_free_dvec(&hsux[ii]);
		blasfeo_free_dvec(&hspi[ii]);
		blasfeo_free_dvec(&hslam[ii]);
		blasfeo_free_dvec(&hst[ii]);
		blasfeo_free_dvec(&hsrrq[ii]);
		blasfeo_free_dvec(&hsrb[ii]);
		blasfeo_free_dvec(&hsrd[ii]);
		blasfeo_free_dvec(&hsrm[ii]);
		}
	ii = N;
	blasfeo_free_dvec(&hsux[ii]);
	blasfeo_free_dvec(&hspi[ii]);
	blasfeo_free_dvec(&hslam[ii]);
	blasfeo_free_dvec(&hst[ii]);
	blasfeo_free_dvec(&hsrrq[ii]);
	blasfeo_free_dvec(&hsrd[ii]);
	blasfeo_free_dvec(&hsrm[ii]);

	v_free_align(work_space_res);

/************************************************
* print timings
************************************************/	

	printf("\ntime ipm full (in sec): %e", time_ipm_full);
	printf("\ntime ipm cond (in sec): %e\n\n", time_ipm_cond);

/************************************************
* return
************************************************/	

	return 0;

	}
