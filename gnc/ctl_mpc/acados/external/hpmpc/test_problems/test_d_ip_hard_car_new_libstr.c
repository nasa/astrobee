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

#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../include/c_interface.h"
#include "tools.h"



#define PRINT 1



void double_integrator(double Ts, int nx, int nu, int N, double *A, double *B, double *b, double *x0)
	{

	int ii;
	double *T;

	// build the continuous time system 

	
	double *Ac; d_zeros(&Ac, nx, nx);
	Ac[0+nx*1] = 1.0;
	
	double *Bc; d_zeros(&Bc, nx, nu);
	Bc[1] = 1.0;

	// compute the discrete time system 

	double *bb; d_zeros(&bb, nx, 1);
	dmcopy(nx, 1, bb, nx, b, nx);

	d_zeros(&T, nx+nu, nx+nu);
	dmcopy(nx, nx, Ac, nx, T, nx+nu);
	dmcopy(nx, nu, Bc, nx, T+nx*(nx+nu), nx+nu);
	dscal_3l((nx+nu)*(nx+nu), Ts, T);
	expm(nx+nu, T);
	dmcopy(nx, nx, T, nx+nu, A, nx);
	dmcopy(nx, nu, T+nx*(nx+nu), nx+nu, B, nx);
	
	d_free(T);
	
	d_free(Ac);
	d_free(Bc);
	d_free(bb);
	
			
	// initial state 
	
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
	
	int rep, nrep=100;

	int nx_ = 2; // number of states (it has to be even for the mass-spring system test problem)
	int nu_ = 1; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 100; // horizon lenght
//	int nb  = nx+nu; // number of box constrained inputs and states
//	int ng  = nx; // number of general constraints
//	int ngN = 0; // number of general constraints at the last stage
	
	int N2 = N; // partial condensing
	


	// in and out times
	double ti = 2.24866092;
	double to = 2.54591719;

	double Ts = 0.1; // sampling time
	int ki = (int) floor(ti/Ts); // index of in time
	int ko = (int) floor(to/Ts); // index of out time



	// stage-wise variant size
	int nx[N+1];
	nx[0] = 0;
	for(ii=1; ii<=N; ii++)
		nx[ii] = nx_;

	int nu[N+1];
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_;
	nu[N] = 0;

	int nb[N+1];
	nb[0] = nu_;
	for(ii=1; ii<N; ii++)
		nb[ii] = nu_+1;
	nb[N] = 1;

	int ng[N+1];
	for(ii=0; ii<N; ii++)
		ng[ii] = 0;
	ng[N] = 0;
	ng[ki] = 1; // general constraints at ki
	ng[ko] = 1; // general constraints at ko

//	for(ii=0; ii<=N; ii++)
//		printf("%d %d\n", ii, ng_v[ii]);
//	exit(1);
	
/************************************************
* dynamical system
************************************************/	

	double *A; d_zeros(&A, nx_, nx_); // states update matrix

	double *B; d_zeros(&B, nx_, nu_); // inputs matrix

	double *b; d_zeros(&b, nx_, 1); // states offset
	double *x0; d_zeros(&x0, nx_, 1); // initial state

	double_integrator(Ts, nx_, nu_, N, A, B, b, x0);
	
	for(jj=0; jj<nx_; jj++)
		b[jj] = 0.0;
	
	for(jj=0; jj<nx_; jj++)
		x0[jj] = 0;
	x0[0] = -55;
	x0[1] = 80/3.6;

//	d_print_mat(2, 2, A, 2);
//	d_print_mat(2, 1, B, 2);
//	d_print_mat(1, 2, b, 1);
//	d_print_mat(1, 2, x0, 1);

#if 1
	// initial guess used in the affine state trasformation
	double *x_bar; d_zeros(&x_bar, nx_, 1);
	x_bar[0] = 0.0;
	x_bar[1] = 10.0;
	int warm_start = 0;
#else
	// or initial guess used for warm start // TODO
	double *x_bar; d_zeros(&x_bar, nx_, 1);
	x_bar[0] = 0.0;
	x_bar[1] = 0.0;
	int warm_start = 1;
#endif
	struct blasfeo_dvec sx_bar; blasfeo_create_dvec(nx_, &sx_bar, x_bar);
	blasfeo_print_tran_dvec(nx_, &sx_bar, 0);
	d_print_mat(1, nx_, x_bar, 1);

	struct blasfeo_dvec sx0; blasfeo_create_dvec(nx_, &sx0, x0);
	blasfeo_print_tran_dvec(nx_, &sx0, 0);
	blasfeo_daxpy(nx_, -1.0, &sx_bar, 0, &sx0, 0, &sx0, 0);
	blasfeo_print_tran_dvec(nx_, &sx0, 0);
	d_print_mat(1, nx_, x0, 1);

	// matrix in panel-wise format to use matrix-vector multiplication routine
	struct blasfeo_dmat sA; blasfeo_allocate_dmat(nx_, nx_, &sA);
	blasfeo_pack_dmat(nx_, nx_, A, nx_, &sA, 0, 0);
	d_print_strmat(nx_, nx_, &sA, 0, 0);

	// affine state transformation
	struct blasfeo_dvec sb; blasfeo_create_dvec(nx_, &sb, b);
	blasfeo_pack_dvec(nx_, b, &sb, 0);
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx_bar, 0, 1.0, &sb, 0, &sb, 0);
	blasfeo_print_tran_dvec(nx_, &sb, 0);
	blasfeo_daxpy(nx_, -1.0, &sx_bar, 0, &sb, 0, &sb, 0);
	blasfeo_print_tran_dvec(nx_, &sb, 0);
	d_print_mat(1, nx_, b, 1);

	// initial guess
	double *b0; d_zeros(&b0, nx_, 1);
	struct blasfeo_dvec sb0; blasfeo_create_dvec(nx_, &sb0, b0);
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sA, 0, 0, &sx0, 0, 1.0, &sb, 0, &sb0, 0);
	blasfeo_print_tran_dvec(nx_, &sb0, 0);
	d_print_mat(1, nx_, b0, 1);

/************************************************
* box & general constraints
************************************************/	

	double dti = ti - 2.2;
	double dto = to - 2.5;

#if 0
	double *C; d_zeros(&C, nx_, nx_);
	C[0+nx_*0] = 1.0;
	C[0+nx_*1] = dti;
	C[1+nx_*0] = 1.0;
	C[1+nx_*1] = dto;

	double *D; d_zeros(&D, nx_, nu_);
	D[0] = 0.5*dti*dti;
	D[1] = 0.5*dto*dto;

	struct blasfeo_dmat sC; blasfeo_allocate_dmat(nx_, nx_, &sC);
	blasfeo_pack_dmat(nx_, nx_, C, nx_, &sC, 0, 0);
	d_print_strmat(nx_, nx_, &sC, 0, 0);

#else

	double *Ci; d_zeros(&Ci, 1, nx_);
	Ci[0+1*0] = 1.0;
	Ci[0+1*1] = dti;

	double *Di; d_zeros(&Di, 1, nu_);
	Di[0] = 0.5*dti*dti;

	double *Co; d_zeros(&Co, 1, nx_);
	Co[0+1*0] = 1.0;
	Co[0+1*1] = dto;

	double *Do; d_zeros(&Do, 1, nu_);
	Do[0] = 0.5*dto*dto;

//	d_print_mat(2, 2, C, 2);
//	d_print_mat(2, 1, D, 2);

	struct blasfeo_dmat sCi; blasfeo_allocate_dmat(1, nx_, &sCi);
	blasfeo_pack_dmat(1, nx_, Ci, 1, &sCi, 0, 0);
	d_print_strmat(1, nx_, &sCi, 0, 0);

	struct blasfeo_dmat sCo; blasfeo_allocate_dmat(1, nx_, &sCo);
	blasfeo_pack_dmat(1, nx_, Co, 1, &sCo, 0, 0);
	d_print_strmat(1, nx_, &sCo, 0, 0);

#endif


	double *lb0; d_zeros(&lb0, nb[0], 1);
	double *ub0; d_zeros(&ub0, nb[0], 1);
	int *idxb0; int_zeros(&idxb0, nb[0], 1);
	// inputs
	lb0[0] = - 2.0;   //   umin
	ub0[0] =   2.0;   //   umax
	idxb0[0] = 0;

	double *lb1; d_zeros(&lb1, nb[1], 1);
	double *ub1; d_zeros(&ub1, nb[1], 1);
	int *idxb1; int_zeros(&idxb1, nb[1], 1);
	// inputs
	lb1[0] = - 2.0;   //   umin
	ub1[0] =   2.0;   //   umax
	idxb1[0] = 0;
	// states
//	lb1[1] = - 1000.0;   //   xmin
//	ub1[1] =   1000.0;   //   xmax
//	idxb1[1] = 1;
//	lb1[2] = - 0.0;      //   xmin
//	ub1[2] =   1000.0;   //   xmax
//	idxb1[2] = 2;
	lb1[1] = - 0.0;      //   xmin
	ub1[1] =   1000.0;   //   xmax
	idxb1[1] = 2;
//	for(jj=0; jj<nx_; jj++) lb1[nu[1]+jj] -= x_bar[jj];
//	for(jj=0; jj<nx_; jj++) ub1[nu[1]+jj] -= x_bar[jj];
	lb1[1] -= x_bar[1];
	ub1[1] -= x_bar[1];

	double *lbN; d_zeros(&lbN, nb[N], 1);
	double *ubN; d_zeros(&ubN, nb[N], 1);
	int *idxbN; int_zeros(&idxbN, nb[N], 1);
	// states
//	lbN[0] = - 1000.0;   //   xmin
//	ubN[0] =   1000.0;   //   xmax
//	idxbN[0] = 0;
//	lbN[1] = - 0.0;      //   xmin
//	ubN[1] =   1000.0;   //   xmax
//	idxbN[1] = 1;
	lbN[0] = - 0.0;      //   xmin
	ubN[0] =   1000.0;   //   xmax
	idxbN[0] = 1;
//	for(jj=0; jj<nx_; jj++) lbN[nu[N]+jj] -= x_bar[jj];
//	for(jj=0; jj<nx_; jj++) ubN[nu[N]+jj] -= x_bar[jj];
	lbN[0] -= x_bar[1];
	ubN[0] -= x_bar[1];

	double *lgi; d_zeros(&lgi, nb[ki], 1);
	double *ugi; d_zeros(&ugi, nb[ki], 1);
	lgi[0] =      0.0-1e-6; //   dmin
	ugi[0] =      0.0+1e-6; //   dmax
	struct blasfeo_dvec slgi; blasfeo_create_dvec(nb[ki], &slgi, lgi);
	struct blasfeo_dvec sugi; blasfeo_create_dvec(nb[ki], &sugi, ugi);
	blasfeo_dgemv_n(ng[ki], nx_, -1.0, &sCi, 0, 0, &sx_bar, 0, 1.0, &slgi, 0, &slgi, 0);
	blasfeo_dgemv_n(ng[ki], nx_, -1.0, &sCi, 0, 0, &sx_bar, 0, 1.0, &sugi, 0, &sugi, 0);

	double *lgo; d_zeros(&lgo, ng[ko], 1);
	double *ugo; d_zeros(&ugo, ng[ko], 1);
	lgo[0] =      8.0-1e-6; //   dmin
	ugo[0] =      8.0+1e-6; //   dmax
	struct blasfeo_dvec slgo; blasfeo_create_dvec(nb[ko], &slgo, lgo);
	struct blasfeo_dvec sugo; blasfeo_create_dvec(nb[ko], &sugo, ugo);
	blasfeo_dgemv_n(ng[ko], nx_, -1.0, &sCo, 0, 0, &sx_bar, 0, 1.0, &slgo, 0, &slgo, 0);
	blasfeo_dgemv_n(ng[ko], nx_, -1.0, &sCo, 0, 0, &sx_bar, 0, 1.0, &sugo, 0, &sugo, 0);

#if 0
	d_print_mat(1, nb[0], lb0, 1);
	d_print_mat(1, nb[0], ub0, 1);
	d_print_mat(1, nb[1], lb1, 1);
	d_print_mat(1, nb[1], ub1, 1);
	d_print_mat(1, nb[N], lbN, 1);
	d_print_mat(1, nb[N], ubN, 1);
	d_print_mat(1, ng[ki], lgi, 1);
	d_print_mat(1, ng[ki], ugi, 1);
	d_print_mat(1, ng[ko], lgo, 1);
	d_print_mat(1, ng[ko], ugo, 1);
	exit(2);
#endif


/************************************************
* cost function
************************************************/	
	
	double *Q; d_zeros(&Q, nx_, nx_);
	Q[0+nx_*0] = 0.0;
	Q[1+nx_*1] = 1.0;

	double *R; d_zeros(&R, nu_, nu_);
	R[0+nu_*0] = 1.0;

	double *S; d_zeros(&S, nu_, nx_);
	// XXX S=0, then no need to compute r+S*x0
	// XXX S=0, then no need to compute r+S*x_bar

	double *q; d_zeros(&q, nx_, 1);
	q[0] = 0.0;
	q[1] = -80/3.6;

	double *r; d_zeros(&r, nu_, 1);

	// matrix in panel-wise format to use matrix-vector multiplication routine
	struct blasfeo_dmat sQ; blasfeo_allocate_dmat(nx_, nx_, &sQ);
	blasfeo_pack_dmat(nx_, nx_, Q, nx_, &sQ, 0, 0);
	struct blasfeo_dvec sq; blasfeo_create_dvec(nx_, &sq, q);
	blasfeo_dgemv_n(nx_, nx_, 1.0, &sQ, 0, 0, &sx_bar, 0, 1.0, &sq, 0, &sq, 0);

#if 0
	d_print_mat(nx_, nx_, Q, nx_);
	d_print_mat(nu_, nx_, S, nu_);
	d_print_mat(nu_, nu_, R, nu_);
	d_print_mat(1, nx_, q, 1);
	d_print_mat(1, nu_, r, 1);
	exit(2);
#endif

	// maximum element in cost functions
	double mu0 = 1.0;

/************************************************
* interface work space
************************************************/	

	double *hA[N];
	double *hB[N];
	double *hb[N];
	double *hQ[N+1];
	double *hS[N];
	double *hR[N];
	double *hq[N+1];
	double *hr[N];
	double *hC[N+1];
	double *hD[N+1];
	double *hlb[N+1];
	double *hub[N+1];
	int *hidxb[N+1];
	double *hlg[N+1];
	double *hug[N+1];
	double *hx[N+1];
	double *hu[N];
	double *hpi[N];
	double *hlam[N+1];

	// first stage
//	hA[0] = NULL;
	hB[0] = B;
	hb[0] = b0;
//	hQ[0] = NULL;
	hS[0] = S;
	hR[0] = R;
//	hq[0] = NULL;
	hr[0] = r;
//	hC[0] = NULL;
//	hD[0] = NULL;
	hlb[0] = lb0;
	hub[0] = ub0;
	hidxb[0] = idxb0;
//	hlg[0] = NULL;
//	hug[0] = NULL;
	d_zeros(&hx[0], nx[0], 1);
	d_zeros(&hu[0], nu[0], 1);
	d_zeros(&hpi[0], nx[1], 1);
	d_zeros(&hlam[0], 2*nb[0]+2*ng[0], 1);
	// general stage
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
//		hC[ii] = NULL;
//		hD[ii] = NULL;
		hlb[ii] = lb1;
		hub[ii] = ub1;
		hidxb[ii] = idxb1;
//		hlg[ii] = NULL;
//		hug[ii] = NULL;
		d_zeros(&hx[ii], nx[ii], 1);
		d_zeros(&hu[ii], nu[ii], 1);
		d_zeros(&hpi[ii], nx[ii+1], 1);
		d_zeros(&hlam[ii], 2*nb[ii]+2*ng[ii], 1);
		}
	// last stage
	hQ[N] = Q;
	hq[N] = q;
//	hC[N] = NULL;
//	hD[N] = NULL;
	hlb[N] = lbN;
	hub[N] = ubN;
	hidxb[N] = idxbN;
//	hlg[N] = NULL;
//	hug[N] = NULL;
	d_zeros(&hx[N], nx[N], 1);
	d_zeros(&hlam[N], 2*nb[N]+2*ng[N], 1);


	// stage ki
	hC[ki] = Ci;
	hD[ki] = Di;
	hlg[ki] = lgi;
	hug[ki] = ugi;

	// stage ki
	hC[ko] = Co;
	hD[ko] = Do;
	hlg[ko] = lgo;
	hug[ko] = ugo;


	double mu = 0.0;

	void *work_ipm; v_zeros(&work_ipm, hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(N, nx, nu, nb, hidxb, ng, N2));

	// warm start
	if(warm_start)
		{
		for(ii=1; ii<=N; ii++)
			{
			hx[ii][1] = 10;
			}
		}

/************************************************
* solvers common stuff
************************************************/	

	int hpmpc_status;
	int kk, kk_avg;
	int k_max = 30;
	double mu_tol = 1e-12;
	double *stat; d_zeros(&stat, k_max, 5);
	double inf_norm_res[5];

	struct timeval tv0, tv1, tv2, tv3;
	double time;

	double **dummy;

/************************************************
* call the solver (high-level interface)
************************************************/	

	gettimeofday(&tv0, NULL); // stop

	kk_avg = 0;

	for(rep=0; rep<nrep; rep++)
		{

		hpmpc_status = fortran_order_d_ip_ocp_hard_tv(&kk, k_max, mu0, mu_tol, N, nx, nu, nb, hidxb, ng, N2, warm_start, hA, hB, hb, hQ, hS, hR, hq, hr, hlb, hub, hC, hD, hlg, hug, hx, hu, hpi, hlam, inf_norm_res, work_ipm, stat);

		kk_avg += kk;

		}
	
	gettimeofday(&tv1, NULL); // stop

#if PRINT
	printf("\nsolution from high-level interface\n\n");
	printf("\nx = \n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx[ii], hx[ii], 1);
	printf("\nu = \n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nu[ii], hu[ii], 1);
	printf("\nlam = \n");
	for(ii=0; ii<=N; ii++)
		d_print_e_mat(1, 2*nb[ii]+2*ng[ii], hlam[ii], 1);
#endif

	printf("\ninfinity norm of residuals\n\n");
	d_print_e_mat(1, 5, inf_norm_res, 1);

	time = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
	printf("\n");
	printf(" Average number of iterations over %d runs: %5.1f\n", nrep, kk_avg / (double) nrep);
	printf(" Average solution time over %d runs: %5.2e seconds\n", nrep, time);
	printf("\n\n");

	// zero solution
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<nx[ii]; jj++)
			hx[ii][jj] = 0.0;
	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nu[ii]; jj++)
			hu[ii][jj] = 0.0;

	gettimeofday(&tv0, NULL); // stop

	kk_avg = 0;

	for(rep=0; rep<nrep; rep++)
		{

		fortran_order_d_ip_last_kkt_new_rhs_ocp_hard_libstr(N, nx, nu, nb, hidxb, ng, N2, hb, hq, hr, hlb, hub, hlg, hug, hx, hu, hpi, hlam, inf_norm_res, work_ipm);

		kk_avg += kk;

		}
	
	gettimeofday(&tv1, NULL); // stop

#if PRINT
	printf("\nsolution from high-level interface (resolve final kkt)\n\n");
	printf("\nx = \n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx[ii], hx[ii], 1);
	printf("\nu = \n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nu[ii], hu[ii], 1);
	printf("\nlam = \n");
	for(ii=0; ii<=N; ii++)
		d_print_e_mat(1, 2*nb[ii]+2*ng[ii], hlam[ii], 1);
#endif


	printf("\ninfinity norm of residuals\n\n");
	d_print_e_mat(1, 5, inf_norm_res, 1);

	time = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf(" Average solution time over %d runs: %5.2e seconds\n", nrep, time);

/************************************************
* free memory
************************************************/	

	d_free(A);
	d_free(B);
	d_free(b);
	d_free(b0);
	d_free(x0);
	d_free(x_bar);
	d_free(Ci);
	d_free(Co);
	d_free(Di);
	d_free(Do);
	d_free(Q);
	d_free(S);
	d_free(R);
	d_free(q);
	d_free(r);
	d_free(lb0);
	d_free(ub0);
	int_free(idxb0);
	d_free(lb1);
	d_free(ub1);
	int_free(idxb1);
	d_free(lbN);
	d_free(ubN);
	int_free(idxbN);
	d_free(lgi);
	d_free(ugi);
	d_free(lgo);
	d_free(ugo);
	for(ii=0; ii<N; ii++)
		{
		d_free(hx[ii]);
		d_free(hu[ii]);
		d_free(hpi[ii]);
		d_free(hlam[ii]);
		}
	d_free(hx[N]);
	d_free(hlam[N]);

	free(work_ipm);
	free(stat);

	blasfeo_free_dmat(&sA);
	blasfeo_free_dmat(&sCi);
	blasfeo_free_dmat(&sCo);
	blasfeo_free_dmat(&sQ);

	return 0;
	
	}



