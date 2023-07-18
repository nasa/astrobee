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

#define PERF_TEST 1


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

	int ii, jj, ll;
	
	printf(" Performance test for the backward Riccati recursion.\n");
	printf("\n");


#if PERF_TEST
	printf("N\tnx\tnu\tsv time\t\tGflops\t\ttrf time\tGflops\t\tmax_res_sv\n");
#endif

	int nn[] = {2, 4, 6, 8, 10, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 10000, 4000, 4000, 2000, 2000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};

#if PERF_TEST
	int ll_max = 78;
#else
	int ll_max = 1;
#endif

	for(ll=0; ll<ll_max; ll++)
		{

#if PERF_TEST
		int nx_ = nn[ll];
		int nu_ = nx_/2;
		int N = 10;

		int rep;
		int nrep = nnrep[ll]/2;
#else
		int nx_ = 8; // number of states (it has to be even for the mass-spring system test problem)
		int nu_ = 3; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
		int N  = 10; // horizon lenght

		int rep;
		int nrep = 1000;
#endif


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
		for(ii=0; ii<=N; ii++)
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

#if ! PERF_TEST
		d_print_mat(nx_, nx_, A, nx_);
		d_print_mat(nx_, nu_, B, nu_);
		d_print_mat(1, nx_, b, 1);
		d_print_mat(1, nx_, x0, 1);
#endif

		struct blasfeo_dmat sA;
		blasfeo_allocate_dmat(nx_, nx_, &sA);
		blasfeo_pack_dmat(nx_, nx_, A, nx_, &sA, 0, 0);
#if ! PERF_TEST
		d_print_strmat(nx_, nx_, &sA, 0, 0);
#endif

		struct blasfeo_dvec sx0;
		blasfeo_allocate_dvec(nx_, &sx0);
		blasfeo_pack_dvec(nx_, x0, &sx0, 0);
#if ! PERF_TEST
		blasfeo_print_tran_dvec(nx_, &sx0, 0);
#endif

		struct blasfeo_dvec sb0;
		blasfeo_allocate_dvec(nx_, &sb0);
		blasfeo_pack_dvec(nx_, b, &sb0, 0);
#if ! PERF_TEST
		blasfeo_print_tran_dvec(nx_, &sb0, 0);
#endif
#if ! KEEP_X0
		blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb0, 0, &sb0, 0);
#endif
#if ! PERF_TEST
		blasfeo_print_tran_dvec(nx_, &sb0, 0);
#endif

		struct blasfeo_dmat sBAbt0;
		blasfeo_allocate_dmat(nu[0]+nx[0]+1, nx[1], &sBAbt0);
		blasfeo_pack_tran_dmat(nx[1], nu[0], B, nx_, &sBAbt0, 0, 0);
		blasfeo_pack_tran_dmat(nx[1], nx[0], A, nx_, &sBAbt0, nu[0], 0);
		blasfeo_drowin(nx[1], 1.0, &sb0, 0, &sBAbt0, nu[0]+nx[0], 0);
#if ! PERF_TEST
		d_print_strmat(nu[0]+nx[0]+1, nx[1], &sBAbt0, 0, 0);
#endif

		struct blasfeo_dmat sBAbt1;
		struct blasfeo_dvec sb1;
		if(N>1)
			{
			blasfeo_allocate_dmat(nu[1]+nx[1]+1, nx[2], &sBAbt1);
			blasfeo_pack_tran_dmat(nx[2], nu[1], B, nx_, &sBAbt1, 0, 0);
			blasfeo_pack_tran_dmat(nx[2], nx[1], A, nx_, &sBAbt1, nu[1], 0);
			blasfeo_pack_tran_dmat(nx[2], 1, b, nx_, &sBAbt1, nu[1]+nx[1], 0);
#if ! PERF_TEST
			d_print_strmat(nu[1]+nx[1]+1, nx[2], &sBAbt1, 0, 0);
#endif
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

		struct blasfeo_dmat sRSQrq0;
		struct blasfeo_dvec srq0;
		blasfeo_allocate_dmat(nu[0]+nx[0]+1, nu[0]+nx[0], &sRSQrq0);
		blasfeo_pack_dmat(nu[0], nu[0], R, nu_, &sRSQrq0, 0, 0);
		blasfeo_pack_tran_dmat(nu[0], nx[0], S, nu_, &sRSQrq0, nu[0], 0);
		blasfeo_pack_dmat(nx[0], nx[0], Q, nx_, &sRSQrq0, nu[0], nu[0]);
		blasfeo_pack_tran_dmat(nu[0], 1, r, nu_, &sRSQrq0, nu[0]+nx[0], 0);
		blasfeo_pack_tran_dmat(nx[0], 1, q, nx_, &sRSQrq0, nu[0]+nx[0], nu[0]);
#if ! PERF_TEST
		d_print_strmat(nu[0]+nx[0]+1, nu[0]+nx[0], &sRSQrq0, 0, 0);
#endif
		blasfeo_allocate_dvec(nu[0]+nx[0], &srq0);
		blasfeo_pack_dvec(nu[0], r, &srq0, 0);
		blasfeo_pack_dvec(nx[0], q, &srq0, nu[0]);
#if ! PERF_TEST
		blasfeo_print_tran_dvec(nu[0]+nx[0], &srq0, 0);
#endif

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
#if ! PERF_TEST
			d_print_strmat(nu[1]+nx[1]+1, nu[1]+nx[1], &sRSQrq1, 0, 0);
#endif
			blasfeo_allocate_dvec(nu[1]+nx[1], &srq1);
			blasfeo_pack_dvec(nu[1], r, &srq1, 0);
			blasfeo_pack_dvec(nx[1], q, &srq1, nu[1]);
#if ! PERF_TEST
			blasfeo_print_tran_dvec(nu[1]+nx[1], &srq1, 0);
#endif
			}

		struct blasfeo_dmat sRSQrqN;
		struct blasfeo_dvec srqN;
		blasfeo_allocate_dmat(nu[N]+nx[N]+1, nu[N]+nx[N], &sRSQrqN);
		blasfeo_pack_dmat(nu[N], nu[N], R, nu_, &sRSQrqN, 0, 0);
		blasfeo_pack_tran_dmat(nu[N], nx[N], S, nu_, &sRSQrqN, nu[N], 0);
		blasfeo_pack_dmat(nx[N], nx[N], Q, nx_, &sRSQrqN, nu[N], nu[N]);
		blasfeo_pack_tran_dmat(nu[N], 1, r, nu_, &sRSQrqN, nu[N]+nx[N], 0);
		blasfeo_pack_tran_dmat(nx[N], 1, q, nx_, &sRSQrqN, nu[N]+nx[N], nu[N]);
#if ! PERF_TEST
		d_print_strmat(nu[N]+nx[N]+1, nu[N]+nx[N], &sRSQrqN, 0, 0);
#endif
		blasfeo_allocate_dvec(nu[N]+nx[N], &srqN);
		blasfeo_pack_dvec(nu[N], r, &srqN, 0);
		blasfeo_pack_dvec(nx[N], q, &srqN, nu[N]);
#if ! PERF_TEST
		blasfeo_print_tran_dvec(nu[N]+nx[N], &srqN, 0);
#endif

/************************************************
* libstr riccati solver
************************************************/	

		struct blasfeo_dmat *hsmatdummy;
		struct blasfeo_dvec *hsvecdummy;

		struct blasfeo_dmat hsBAbt[N];
		struct blasfeo_dmat hsRSQrq[N+1];
		struct blasfeo_dmat hsDCt[N+1];
		struct blasfeo_dvec hsd[N+1];
		int *hidxb[N+1];
		struct blasfeo_dvec hsux[N+1];
		struct blasfeo_dvec hspi[N+1];
		struct blasfeo_dvec hsPb[N+1];
		struct blasfeo_dmat hsL[N+1];


		hsBAbt[0] = sBAbt0;
		hsRSQrq[0] = sRSQrq0;
		blasfeo_allocate_dvec(nu[0]+nx[0], &hsux[0]);
		blasfeo_allocate_dvec(nx[1], &hspi[1]);
		blasfeo_allocate_dvec(nx[1], &hsPb[1]);
		blasfeo_allocate_dmat(nu[0]+nx[0]+1, nu[0]+nx[0], &hsL[0]);
		for(ii=1; ii<N; ii++)
			{
			hsBAbt[ii] = sBAbt1;
			hsRSQrq[ii] = sRSQrq1;
			blasfeo_allocate_dvec(nu[ii]+nx[ii], &hsux[ii]);
			blasfeo_allocate_dvec(nx[ii+1], &hspi[ii+1]);
			blasfeo_allocate_dvec(nx[ii+1], &hsPb[ii+1]);
			blasfeo_allocate_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii]);
			}
		hsRSQrq[N] = sRSQrqN;
		blasfeo_allocate_dvec(nu[N]+nx[N], &hsux[N]);
		blasfeo_allocate_dmat(nu[N]+nx[N]+1, nu[N]+nx[N], &hsL[N]);
		
		// riccati work space
		void *work_ric;
		v_zeros_align(&work_ric, d_back_ric_rec_work_space_size_bytes_libstr(N, nx, nu, nb, ng));



		int M = 3;

		int nxM;

		struct blasfeo_dmat hstmpmat0;



		struct timeval tv0, tv1, tv2;

		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			d_back_ric_rec_sv_libstr(N, nx, nu, nb, hidxb, ng, 0, hsBAbt, hsvecdummy, 0, hsRSQrq, hsvecdummy, hsmatdummy, hsvecdummy, hsvecdummy, hsux, 1, hspi, 1, hsPb, hsL, work_ric);
			}

		gettimeofday(&tv1, NULL); // stop

		for(rep=0; rep<nrep; rep++)
			{
			d_back_ric_rec_trf_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsmatdummy, hsvecdummy, hsL, work_ric);
			}

		gettimeofday(&tv2, NULL); // stop

#if ! PERF_TEST
		printf("\nux =\n\n");
		for(ii=0; ii<=N; ii++)
			blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);

		printf("\npi =\n\n");
		for(ii=0; ii<=N; ii++)
			blasfeo_print_tran_dvec(nx[ii], &hspi[ii], 0);

		printf("\nL =\n\n");
		for(ii=0; ii<=N; ii++)
			d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], 0, 0);
#endif

		double time_sv  = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		double flop_sv = (1.0/3.0*nx_*nx_*nx_+3.0/2.0*nx_*nx_) + N*(7.0/3.0*nx_*nx_*nx_+4.0*nx_*nx_*nu_+2.0*nx_*nu_*nu_+1.0/3.0*nu_*nu_*nu_+13.0/2.0*nx_*nx_+9.0*nx_*nu_+5.0/2.0*nu_*nu_) - (nx_*(nx_+nu_)+1.0/3.0*nx_*nx_*nx_+3.0/2.0*nx_*nx_); // TODO
		double Gflops_sv = 1e-9*flop_sv/time_sv;

		double time_trf = (tv2.tv_sec-tv1.tv_sec)/(nrep+0.0)+(tv2.tv_usec-tv1.tv_usec)/(nrep*1e6);
//		double flop_trf = (1.0/3.0*nx*nx*nx) + (N-1)*(7.0/3.0*nx*nx*nx+4.0*nx*nx*nu+2.0*nx*nu*nu+1.0/3.0*nu*nu*nu) + (1.0*nx*nx*nu+1.0*nx*nu*nu+1.0/3.0*nu*nu*nu);
		double flop_trf = 0.0;
		flop_trf += 1.0/3.0*(nu[N]+nx[N])*(nu[N]+nx[N])*(nu[N]+nx[N]); // potrf
		for(ii=0; ii<N; ii++)
			{
			flop_trf += 1.0*(nu[N-ii-1]+nx[N-ii-1])*nx[N-ii]*nx[N-ii]; // trmm
			flop_trf += 1.0*(nu[N-ii-1]+nx[N-ii-1])*(nu[N-ii-1]+nx[N-ii-1])*(nx[N-ii]); // syrk
			flop_trf += 1.0/3.0*(nu[N-ii-1]+nx[N-ii-1])*(nu[N-ii-1]+nx[N-ii-1])*(nu[N-ii-1]+nx[N-ii-1]); // potrf
			}
		double Gflops_trf = 1e-9*flop_trf/time_trf;

		double flop_trs = N*(6*nx_*nx_+8.0*nx_*nu_+2.0*nu_*nu_);

/************************************************
* libstr ip2 residuals
************************************************/	

		struct blasfeo_dvec hsb[N];
		struct blasfeo_dvec hsrq[N+1];
		struct blasfeo_dvec hsrrq[N+1];
		struct blasfeo_dvec hsrb[N];
		struct blasfeo_dvec hsrd[N+1];
		struct blasfeo_dvec hsrm[N+1];
		double mu = 0.0;

		hsb[0] = sb0;
		hsrq[0] = srq0;
		blasfeo_allocate_dvec(nu[0]+nx[0], &hsrrq[0]);
		blasfeo_allocate_dvec(nx[1], &hsrb[0]);
		blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &hsrd[0]);
		blasfeo_allocate_dvec(2*nb[0]+2*ng[0], &hsrm[0]);
		for(ii=1; ii<N; ii++)
			{
			hsb[ii] = sb1;
			hsrq[ii] = srq1;
			blasfeo_allocate_dvec(nu[ii]+nx[ii], &hsrrq[ii]);
			blasfeo_allocate_dvec(nx[ii+1], &hsrb[ii]);
			blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii]);
			blasfeo_allocate_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii]);
			}
		hsrq[N] = srqN;
		blasfeo_allocate_dvec(nu[N]+nx[N], &hsrrq[N]);
		blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hsrd[N]);
		blasfeo_allocate_dvec(2*nb[N]+2*ng[N], &hsrm[N]);

		void *work_res;
		v_zeros_align(&work_res, d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng));

		d_res_res_mpc_hard_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hsvecdummy, hsvecdummy, hsrrq, hsrb, hsrd, hsrm, &mu, work_res);

		double res_max = 0.0;
		double tmp;
		for(ii=0; ii<=N; ii++)
			{
			for(jj=0; jj<nu[ii]+nx[ii]; jj++)
				{
				tmp = blasfeo_dvecex1(&hsrrq[ii], jj);
				res_max = tmp>res_max ? tmp : res_max;
				}
			}
		for(ii=0; ii<N; ii++)
			{
			for(jj=0; jj<nx[ii+1]; jj++)
				{
				tmp = blasfeo_dvecex1(&hsrb[ii], jj);
				res_max = tmp>res_max ? tmp : res_max;
				}
			}
		for(ii=0; ii<=N; ii++)
			{
			for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++)
				{
				tmp = blasfeo_dvecex1(&hsrd[ii], jj);
				res_max = tmp>res_max ? tmp : res_max;
				}
			}
		for(ii=0; ii<=N; ii++)
			{
			for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++)
				{
				tmp = blasfeo_dvecex1(&hsrm[ii], jj);
				res_max = tmp>res_max ? tmp : res_max;
				}
			}
		res_max = mu>res_max ? mu : res_max;

#if ! PERF_TEST
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

		printf(" Average solution time over %d runs: %5.2e seconds (IPM)\n", nrep, time_sv);
#else
		printf("%d\t%d\t%d\t%e\t%f\t%e\t%f\t%e\n", N, nx_, nu_, time_sv, Gflops_sv, time_trf, Gflops_trf, res_max);
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
		
		v_free_align(work_ric);
		v_free_align(work_res);

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
		blasfeo_free_dmat(&hsL[0]);
		blasfeo_free_dvec(&hsux[0]);
		blasfeo_free_dvec(&hspi[1]);
		blasfeo_free_dvec(&hsPb[1]);
		blasfeo_free_dvec(&hsrrq[0]);
		blasfeo_free_dvec(&hsrb[0]);
		blasfeo_free_dvec(&hsrd[0]);
		blasfeo_free_dvec(&hsrm[0]);
		for(ii=1; ii<N; ii++)
			{
			blasfeo_free_dmat(&hsL[ii]);
			blasfeo_free_dvec(&hsux[ii]);
			blasfeo_free_dvec(&hspi[ii+1]);
			blasfeo_free_dvec(&hsPb[ii+1]);
			blasfeo_free_dvec(&hsrrq[ii]);
			blasfeo_free_dvec(&hsrb[ii]);
			blasfeo_free_dvec(&hsrd[ii]);
			blasfeo_free_dvec(&hsrm[ii]);
			}
		blasfeo_free_dmat(&hsL[N]);
		blasfeo_free_dvec(&hsux[N]);
		blasfeo_free_dvec(&hsrrq[N]);
		blasfeo_free_dvec(&hsrd[N]);
		blasfeo_free_dvec(&hsrm[N]);
	
	}

/************************************************
* return
************************************************/	

	return 0;
	}
