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
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>
#endif

#include "../include/aux_d.h"
#include "../include/aux_s.h"
#include "../include/blas_d.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../include/block_size.h"
#include "tools.h"
#include "../include/c_interface.h"



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

	int nx = 8; // number of states (it has to be even for the mass-spring system test problem)
	int nu = 3; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 10; // horizon lenght
	int nb  = nu+nx/2; // number of box constrained inputs and states
	int ng  = 0; //nx; //4;  // number of general constraints
	int ngN = 0;//nx/2; //nx; // number of general constraints at the last stage

	// partial condensing horizon
	int N2 = N; //N/2;

	// maximum number of IPM iterations
	int k_max = 10;

	// exit tolerance in duality measure
	double mu_tol = 1e-12;

	// minimum step size length
	double alpha_min = 1e-8;

	// number of calls to solver (for more accurate timings)
	int nrep = 1000;



# define USE_IPM_RES 1
	
//	int M = 32; // where the equality constraint hold

	int nbu = nu<nb ? nu : nb ;
	int nbx = nb-nu>0 ? nb-nu : 0;

#define KEEP_X0 0

	// stage-wise variant size
	int nx_v[N+1];
#if KEEP_X0
	nx_v[0] = nx;
#else
	nx_v[0] = 0;
#endif
	for(ii=1; ii<=N; ii++)
		nx_v[ii] = nx;

	int nu_v[N+1];
	for(ii=0; ii<N; ii++)
		nu_v[ii] = nu;
	nu_v[N] = 0;

	int nb_v[N+1];
#if KEEP_X0
	nb_v[0] = nb;
#else
	nb_v[0] = nbu;
#endif
	for(ii=1; ii<N; ii++)
		nb_v[ii] = nb;
	nb_v[N] = nbx;

	int ng_v[N+1];
	for(ii=0; ii<N; ii++)
		ng_v[ii] = ng;
	ng_v[N] = ngN;
//	ng_v[M] = nx; // XXX
	



	printf(" Test problem: mass-spring system with %d masses and %d controls.\n", nx/2, nu);
	printf("\n");
	printf(" MPC problem size: %d states, %d inputs, %d horizon length, %d two-sided box constraints, %d two-sided general constraints.\n", nx, nu, N, nb, ng);
	printf("\n");
	printf(" IP method parameters: predictor-corrector IP, double precision, %d maximum iterations, %5.1e exit tolerance in duality measure (edit file test_param.c to change them).\n", k_max, mu_tol);

	int info = 0;
		
	const int bs  = D_MR; //d_get_mr();
	const int ncl = D_NCL;

	int pnz = (nu+nx+1+bs-1)/bs*bs;
	int pnu = (nu+bs-1)/bs*bs;
	int pnu1 = (nu+1+bs-1)/bs*bs;
	int pnx = (nx+bs-1)/bs*bs;
	int pnx1 = (nx+1+bs-1)/bs*bs;
	int pnux = (nu+nx+bs-1)/bs*bs;
	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cnux = (nu+nx+ncl-1)/ncl*ncl;

	int pnb_v[N+1]; 
	int png_v[N+1]; 
	int pnx_v[N+1]; 
	int pnz_v[N+1]; 
	int pnux_v[N+1]; 
	int cnx_v[N+1]; 
	int cnux_v[N+1]; 
	int cng_v[N+1]; 

	for(ii=0; ii<N; ii++) 
		{
		pnb_v[ii] = (nb_v[ii]+bs-1)/bs*bs;
		png_v[ii] = (ng_v[ii]+bs-1)/bs*bs;
		pnx_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
		pnz_v[ii] = (nu_v[ii]+nx_v[ii]+1+bs-1)/bs*bs;
		pnux_v[ii] = (nu_v[ii]+nx_v[ii]+bs-1)/bs*bs;
		cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
		cnux_v[ii] = (nu_v[ii]+nx_v[ii]+ncl-1)/ncl*ncl;
		cng_v[ii] = (ng_v[ii]+ncl-1)/ncl*ncl;
		}
	ii = N;
	pnb_v[ii] = (nb_v[ii]+bs-1)/bs*bs;
	png_v[ii] = (ng_v[ii]+bs-1)/bs*bs;
	pnx_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
	pnz_v[ii] = (nx_v[ii]+1+bs-1)/bs*bs;
	pnux_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
	cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
	cnux_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
	cng_v[ii] = (ng_v[ii]+ncl-1)/ncl*ncl;


/************************************************
* dynamical system
************************************************/	

	double *A; d_zeros(&A, nx, nx); // states update matrix

	double *B; d_zeros(&B, nx, nu); // inputs matrix

	double *b; d_zeros_align(&b, nx, 1); // states offset
	double *x0; d_zeros_align(&x0, nx, 1); // initial state

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx, nu, N, A, B, b, x0);
	
	for(jj=0; jj<nx; jj++)
		b[jj] = 0.1;
	
	for(jj=0; jj<nx; jj++)
		x0[jj] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

	double *pA; d_zeros_align(&pA, pnx, cnx);
	d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);
	double *b0; d_zeros_align(&b0, pnx, 1);
	for(ii=0; ii<nx; ii++) b0[ii] = b[ii];
#if ! KEEP_X0
#if defined(BLASFEO)
	dgemv_n_lib(nx, nx, 1.0, pA, cnx, x0, 1.0, b0, b0);
#else
	dgemv_n_lib(nx, nx, pA, cnx, x0, 1, b0, b0);
#endif
#endif

	double *pBAbt0; 
	d_zeros_align(&pBAbt0, pnz_v[0], cnx_v[1]);
	d_cvt_tran_mat2pmat(nx_v[1], nu_v[0], B, nx_v[1], 0, pBAbt0, cnx_v[1]);
	d_cvt_tran_mat2pmat(nx_v[1], nx_v[0], A, nx_v[1], nu_v[0], pBAbt0+nu_v[0]/bs*bs*cnx_v[1]+nu_v[0]%bs, cnx_v[1]);
	d_cvt_tran_mat2pmat(nx_v[1], 1, b0, nx_v[1], nu_v[0]+nx_v[0], pBAbt0+(nu_v[0]+nx_v[0])/bs*bs*cnx_v[1]+(nu_v[0]+nx_v[0])%bs, cnx_v[1]);

	double *pBAbt1; 
	if(N>1)
		{
		d_zeros_align(&pBAbt1, pnz_v[1], cnx_v[2]);
		d_cvt_tran_mat2pmat(nx_v[2], nu_v[1], B, nx_v[2], 0, pBAbt1, cnx_v[2]);
		d_cvt_tran_mat2pmat(nx_v[2], nx_v[1], A, nx_v[2], nu_v[1], pBAbt1+nu_v[1]/bs*bs*cnx_v[2]+nu_v[1]%bs, cnx_v[2]);
		d_cvt_tran_mat2pmat(nx_v[2], 1, b, nx_v[2], nu_v[1]+nx_v[1], pBAbt1+(nu_v[1]+nx_v[1])/bs*bs*cnx_v[2]+(nu_v[1]+nx_v[1])%bs, cnx_v[2]);
		}

#if 0
d_print_pmat(nu_v[0]+nx_v[0]+1, nx_v[1], bs, pBAbt0, cnx_v[1]);
d_print_pmat(nu_v[1]+nx_v[1]+1, nx_v[2], bs, pBAbt1, cnx_v[2]);
exit(2);
#endif

/************************************************
* box & general constraints
************************************************/	

	int *idxb0; int_zeros(&idxb0, nb_v[0], 1);
	double *d0; d_zeros_align(&d0, 2*pnb_v[0]+2*png_v[0], 1);
#if KEEP_X0
	for(jj=0; jj<nbu; jj++)
		{
		d0[jj]          = - 0.5;   //   umin
		d0[pnb_v[0]+jj] =   0.5;   //   umax
		idxb0[jj] = jj;
		}
	for(; jj<nb; jj++)
		{
		d0[jj]          =   x0[jj-nu];   //   xmin
		d0[pnb_v[0]+jj] =   x0[jj-nu];   //   xmax
		idxb0[jj] = jj;
		}
#else
	for(jj=0; jj<nbu; jj++)
		{
		d0[jj]          = - 0.5;   //   umin
		d0[pnb_v[0]+jj] =   0.5;   //   umax
		idxb0[jj] = jj;
		}
#endif
	for(jj=0; jj<ng_v[0]; jj++)
		{
		d0[2*pnb_v[0]+jj]          = - 100.0;   //   xmin
		d0[2*pnb_v[0]+png_v[0]+jj] =   100.0;   //   xmax
		}
#if 0
	int_print_mat(1, nb_v[0], idxb0, 1);
	d_print_mat(1, 2*pnb_v[0]+2*png_v[0], d0, 1);
	exit(2);
#endif

	int *idxb1; int_zeros(&idxb1, nb_v[1], 1);
	double *d1; d_zeros_align(&d1, 2*pnb_v[1]+2*png_v[1], 1);
	for(jj=0; jj<nbu; jj++)
		{
		d1[jj]          = - 0.5;   //   umin
		d1[pnb_v[1]+jj] =   0.5;   //   umax
		idxb1[jj] = jj;
		}
	for(; jj<nb; jj++)
		{
		d1[jj]          = - 4.0;   //   xmin
		d1[pnb_v[1]+jj] =   4.0;   //   xmax
		idxb1[jj] = jj;
		}
	for(jj=0; jj<ng_v[1]; jj++)
		{
		d1[2*pnb_v[1]+jj]          = - 100.0;   //   xmin
		d1[2*pnb_v[1]+png_v[1]+jj] =   100.0;   //   xmax
		}
//	int_print_mat(nb, 1, idxb1, nb);

	int *idxbN; int_zeros(&idxbN, nb_v[N], 1);
	double *dN; d_zeros_align(&dN, 2*pnb_v[N]+2*png_v[N], 1);
	for(jj=0; jj<nbx; jj++)
		{
		dN[jj]          = - 4.0;   //   xmin
		dN[pnb_v[N]+jj] =   4.0;   //   xmax
		idxbN[jj] = jj;
		}
	for(jj=0; jj<ng_v[N]; jj++)
		{
		dN[2*pnb_v[N]+jj]          = - 0.0;   //   xmin
		dN[2*pnb_v[N]+png_v[N]+jj] =   0.0;   //   xmax
		}
//	d_print_mat(1, 2*pnb+2*png, d, 1);
//	d_print_mat(1, 2*pnb_v[N]+2*png_v[N], dN, 1);
//	exit(1);
	
//	double *dM; d_zeros_align(&dM, 2*pnb_v[M]+2*png_v[M], 1);
//	for(jj=0; jj<nbu; jj++)
//		{
//		dM[jj]          = - 0.5;   //   umin
//		dM[pnb_v[1]+jj] =   0.5;   //   umax
//		}
//	for(; jj<nb; jj++)
//		{
//		dM[jj]          = - 4.0;   //   xmin
//		dM[pnb_v[1]+jj] =   4.0;   //   xmax
//		}
//	for(jj=0; jj<ng_v[M]; jj++)
//		{
//		dM[2*pnb_v[M]+jj]          = - 0.5;   //   xmin
//		dM[2*pnb_v[M]+png_v[M]+jj] = - 0.5;   //   xmax
//		}

	double *C; d_zeros(&C, ng, nx);
	for(ii=0; ii<ng; ii++)
		C[ii*(ng+1)] = 1.0;
	double *D; d_zeros(&D, ng, nu);

	// first stage
	double *pDCt0; d_zeros_align(&pDCt0, pnux_v[0], cng_v[0]);
	// middle stage
	double *DC1; d_zeros(&DC1, ng_v[1], nu_v[1]+nx_v[1]);
	for(jj=0; jj<ng_v[1]; jj++) DC1[jj+(nu_v[1]+jj)*ng_v[1]] = 1.0;
//	d_print_mat(ng_v[1], nu_v[1]+nx_v[1], DC1, ng_v[1]);
	double *pDCt1; d_zeros_align(&pDCt1, pnux_v[1], cng_v[1]);
	d_cvt_tran_mat2pmat(ng_v[1], nu_v[1]+nx_v[1], DC1, ng_v[1], 0, pDCt1, cng_v[1]);
//	d_print_pmat(nu_v[1]+nx_v[1], ng_v[1], bs, pDCt1, cng_v[1]);
//	exit(2);
	// last stage
	double *DCN; d_zeros(&DCN, ng_v[N], nx_v[N]);
	for(jj=0; jj<ng_v[N]; jj++) DCN[jj*(ng_v[N]+1)] = 1.0;
//	d_print_mat(ng_v[N], nx_v[N], DCN, ng_v[N]);
	double *pDCtN; d_zeros_align(&pDCtN, pnx_v[N], cng_v[N]);
	d_cvt_tran_mat2pmat(ng_v[N], nx_v[N], DCN, ng_v[N], 0, pDCtN, cng_v[N]);
//	d_print_pmat(nx_v[N], ng_v[N], bs, pDCtN, cng_v[N]);
	// constrained stage
//	double *DCM; d_zeros(&DCM, ng_v[M], nu_v[M]+nx_v[M]);
//	for(jj=0; jj<ng_v[M]; jj++) DCM[jj+(jj+nu_v[M])*ng_v[M]] = 1.0;
//	d_print_mat(ng_v[M], nu_v[M]+nx_v[M], DCM, ng_v[M]);
//	double *pDCtM; d_zeros_align(&pDCtM, pnux_v[M], cng_v[M]);
//	d_cvt_tran_mat2pmat(ng_v[M], nu_v[M]+nx_v[M], DCM, ng_v[M], 0, pDCtM, cng_v[M]);
//	d_print_pmat(nu_v[M]+nx_v[M], ng_v[M], bs, pDCtM, cng_v[M]);
//	exit(2);

/************************************************
* cost function
************************************************/	
	
	double *Q; d_zeros(&Q, nx, nx);
	for(ii=0; ii<nx; ii++) Q[ii*(nx+1)] = 1.0;

	double *R; d_zeros(&R, nu, nu);
	for(ii=0; ii<nu; ii++) R[ii*(nu+1)] = 2.0;

	double *S; d_zeros(&S, nu, nx); // S=0, so no need to update r0

	double *q; d_zeros(&q, nx, 1);
	for(ii=0; ii<nx; ii++) q[ii] = 0.1;

	double *r; d_zeros(&r, nu, 1);
	for(ii=0; ii<nu; ii++) r[ii] = 0.2;

#if KEEP_X0
	double  *pRSQ0; d_zeros_align(&pRSQ0, pnz, cnux);
	d_cvt_mat2pmat(nu, nu, R, nu, 0, pRSQ0, cnux);
	d_cvt_tran_mat2pmat(nu, nx, S, nu, nu, pRSQ0+nu/bs*bs*cnux+nu%bs, cnux);
	d_cvt_tran_mat2pmat(nu, 1, r, nu, nu+nx, pRSQ0+(nu+nx)/bs*bs*cnux+(nu+nx)%bs, cnux);
	d_cvt_mat2pmat(nx, nx, Q, nx, nu, pRSQ0+nu/bs*bs*cnux+nu%bs+nu*bs, cnux);
	d_cvt_tran_mat2pmat(nx, 1, q, nx, nu+nx, pRSQ0+(nu+nx)/bs*bs*cnux+(nu+nx)%bs+nu*bs, cnux);
//	d_print_pmat(nu+nx+1, nu+nx, bs, pRSQ0, cnux);
	double *rq0; d_zeros_align(&rq0, pnux, 1);
	d_copy_mat(nu, 1, r, nu, rq0, pnux);
	d_copy_mat(nx, 1, q, nx, rq0+nu, pnux);
#else
	double  *pRSQ0; d_zeros_align(&pRSQ0, pnu1, cnu);
	d_cvt_mat2pmat(nu, nu, R, nu, 0, pRSQ0, cnu);
	d_cvt_tran_mat2pmat(nu, 1, r, nu, nu, pRSQ0+nu/bs*bs*cnu+nu%bs, cnu);
//	d_print_pmat(nu+1, nu, bs, pRSQ0, cnu);
	double *rq0; d_zeros_align(&rq0, pnu, 1);
	d_copy_mat(nu, 1, r, nu, rq0, pnu);
#endif

	double  *pRSQ1; d_zeros_align(&pRSQ1, pnz, cnux);
	d_cvt_mat2pmat(nu, nu, R, nu, 0, pRSQ1, cnux);
	d_cvt_tran_mat2pmat(nu, nx, S, nu, nu, pRSQ1+nu/bs*bs*cnux+nu%bs, cnux);
	d_cvt_tran_mat2pmat(nu, 1, r, nu, nu+nx, pRSQ1+(nu+nx)/bs*bs*cnux+(nu+nx)%bs, cnux);
	d_cvt_mat2pmat(nx, nx, Q, nx, nu, pRSQ1+nu/bs*bs*cnux+nu%bs+nu*bs, cnux);
	d_cvt_tran_mat2pmat(nx, 1, q, nx, nu+nx, pRSQ1+(nu+nx)/bs*bs*cnux+(nu+nx)%bs+nu*bs, cnux);
//	d_print_pmat(nu+nx+1, nu+nx, bs, pRSQ1, cnux);
	double *rq1; d_zeros_align(&rq1, pnux, 1);
	d_copy_mat(nu, 1, r, nu, rq1, pnux);
	d_copy_mat(nx, 1, q, nx, rq1+nu, pnux);

	double  *pRSQN; d_zeros_align(&pRSQN, pnx1, cnx);
	d_cvt_mat2pmat(nx, nx, Q, nx, 0, pRSQN, cnx);
	d_cvt_tran_mat2pmat(nx, 1, q, nx, nx, pRSQN+(nx)/bs*bs*cnx+(nx)%bs, cnx);
//	d_print_pmat(nx+1, nx, bs, pRSQN, cnx);
	double *rqN; d_zeros_align(&rqN, pnx, 1);
	d_copy_mat(nx, 1, q, nx, rqN, pnx);


	// maximum element in cost functions
	double mu0 = 2.0;

/************************************************
* high level interface work space
************************************************/	

#if 0
	double *rA; d_zeros(&rA, nx, N*nx);
	d_rep_mat(N, nx, nx, A, nx, rA, nx);

	double *rB; d_zeros(&rB, nx, N*nu);
	d_rep_mat(N, nx, nu, B, nx, rB, nx);

	double *rC; d_zeros(&rC, ng, (N+1)*nx);
	d_rep_mat(N, ng, nx, C, ng, rC+nx*ng, ng);

	double *CN = DCN;

	double *rD; d_zeros(&rD, ng, N*nu);
	d_rep_mat(N, ng, nu, D, ng, rD, ng);

	double *rb; d_zeros(&rb, nx, N*1);
	d_rep_mat(N, nx, 1, b, nx, rb, nx);

	double *rQ; d_zeros(&rQ, nx, N*nx);
	d_rep_mat(N, nx, nx, Q, nx, rQ, nx);

	double *rQf; d_zeros(&rQf, nx, nx);
	d_copy_mat(nx, nx, Q, nx, rQf, nx);

	double *rS; d_zeros(&rS, nu, N*nx);
	d_rep_mat(N, nu, nx, S, nu, rS, nu);

	double *rR; d_zeros(&rR, nu, N*nu);
	d_rep_mat(N, nu, nu, R, nu, rR, nu);

	double *rq; d_zeros(&rq, nx, N);
	d_rep_mat(N, nx, 1, q, nx, rq, nx);

	double *rqf; d_zeros(&rqf, nx, 1);
	d_copy_mat(nx, 1, q, nx, rqf, nx);

	double *rr; d_zeros(&rr, nu, N);
	d_rep_mat(N, nu, 1, r, nu, rr, nu);

	double *lb; d_zeros(&lb, nb, 1);
	for(ii=0; ii<nb; ii++)
		lb[ii] = d1[ii];
	double *rlb; d_zeros(&rlb, nb, N+1);
	d_rep_mat(N+1, nb, 1, lb, nb, rlb, nb);
//	d_print_mat(nb, N+1, rlb, nb);

	double *lg; d_zeros(&lg, ng, 1);
	for(ii=0; ii<ng; ii++)
		lg[ii] = d1[2*pnb_v[1]+ii];
	double *rlg; d_zeros(&rlg, ng, N);
	d_rep_mat(N, ng, 1, lg, ng, rlg, ng);
//	d_print_mat(ng, N, rlg, ng);

	double *lgN; d_zeros(&lgN, ngN, 1);
	for(ii=0; ii<ngN; ii++)
		lgN[ii] = dN[2*pnb_v[N]+ii];
//	d_print_mat(ngN, 1, lgN, ngN);

	double *ub; d_zeros(&ub, nb, 1);
	for(ii=0; ii<nb; ii++)
		ub[ii] = d1[pnb_v[1]+ii];
	double *rub; d_zeros(&rub, nb, N+1);
	d_rep_mat(N+1, nb, 1, ub, nb, rub, nb);
//	d_print_mat(nb, N+1, rub, nb);

	double *ug; d_zeros(&ug, ng, 1);
	for(ii=0; ii<ng; ii++)
		ug[ii] = d1[2*pnb_v[1]+png_v[1]+ii];
	double *rug; d_zeros(&rug, ng, N);
	d_rep_mat(N, ng, 1, ug, ng, rug, ng);
//	d_print_mat(ng, N, rug, ng);

	double *ugN; d_zeros(&ugN, ngN, 1);
	for(ii=0; ii<ngN; ii++)
		ugN[ii] = dN[2*pnb_v[N]+png_v[N]+ii];
//	d_print_mat(ngN, 1, ugN, ngN);

	double *rx; d_zeros(&rx, nx, N+1);
	d_copy_mat(nx, 1, x0, nx, rx, nx);

	double *ru; d_zeros(&ru, nu, N);

	double *rpi; d_zeros(&rpi, nx, N);

	double *rlam; d_zeros(&rlam, N*2*(nb+ng)+2*(nb+ngN), 1);

	double *rt; d_zeros(&rt, N*2*(nb+ng)+2*(nb+ngN), 1);

	double *rwork = (double *) malloc(hpmpc_d_ip_mpc_hard_tv_work_space_size_bytes(N, nx, nu, nb, ng, ngN));

	double inf_norm_res[4] = {}; // infinity norm of residuals: rq, rb, rd, mu
#endif

/************************************************
* low level interface work space
************************************************/	

	double *hpBAbt[N];
	double *hpDCt[N+1];
	double *hb[N];
	double *hpRSQ[N+1];
	double *hrq[N+1];
	double *hd[N+1];
	int *hidxb[N+1];
	double *hux[N+1];
	double *hpi[N];
	double *hlam[N+1];
	double *ht[N+1];
	double *hrb[N];
	double *hrrq[N+1];
	double *hrd[N+1];
	hpBAbt[0] = pBAbt0;
	hpDCt[0] = pDCt0;
	hb[0] = b0;
	hpRSQ[0] = pRSQ0;
	hrq[0] = rq0;
	hd[0] = d0;
	hidxb[0] = idxb0;
	d_zeros_align(&hux[0], pnux_v[0], 1);
	d_zeros_align(&hpi[0], pnx_v[1], 1);
	d_zeros_align(&hlam[0], 2*pnb_v[0]+2*png_v[0], 1);
	d_zeros_align(&ht[0], 2*pnb_v[0]+2*png_v[0], 1);
	d_zeros_align(&hrb[0], pnx_v[1], 1);
	d_zeros_align(&hrrq[0], pnz_v[0], 1);
	d_zeros_align(&hrd[0], 2*pnb_v[0]+2*png_v[0], 1);
	for(ii=1; ii<N; ii++)
		{
		hpBAbt[ii] = pBAbt1;
//		d_zeros_align(&hpBAbt[ii], pnz_v[ii], cnx_v[ii+1]); for(jj=0; jj<pnz_v[ii]*cnx_v[ii+1]; jj++) hpBAbt[ii][jj] = pBAbt1[jj];
		hpDCt[ii] = pDCt1;
		hb[ii] = b;
		hpRSQ[ii] = pRSQ1;
//		d_zeros_align(&hpRSQ[ii], pnz_v[ii], cnux_v[ii]); for(jj=0; jj<pnz_v[ii]*cnux_v[ii]; jj++) hpRSQ[ii][jj] = pRSQ1[jj];
		hrq[ii] = rq1;
		hd[ii] = d1;
		hidxb[ii] = idxb1;
		d_zeros_align(&hux[ii], pnux_v[ii], 1);
		d_zeros_align(&hpi[ii], pnx_v[ii+1], 1);
		d_zeros_align(&hlam[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		d_zeros_align(&ht[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		d_zeros_align(&hrb[ii], pnx_v[ii+1], 1);
		d_zeros_align(&hrrq[ii], pnz_v[ii], 1);
		d_zeros_align(&hrd[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		}
	hpDCt[N] = pDCtN;
	hpRSQ[N] = pRSQN;
	hrq[N] = rqN;
	hd[N] = dN;
	hidxb[N] = idxbN;
	d_zeros_align(&hux[N], pnx, 1);
	d_zeros_align(&hlam[N], 2*pnb_v[N]+2*png_v[N], 1);
	d_zeros_align(&ht[N], 2*pnb_v[N]+2*png_v[N], 1);
	d_zeros_align(&hrrq[N], pnz_v[N], 1);
	d_zeros_align(&hrd[N], 2*pnb_v[N]+2*png_v[N], 1);

//	hpDCt[M] = pDCtM;
//	hd[M] = dM;

	double mu = 0.0;

#if USE_IPM_RES
	double *work; d_zeros_align(&work, d_ip2_res_mpc_hard_tv_work_space_size_bytes(N, nx_v, nu_v, nb_v, ng_v)/sizeof(double), 1);
#else
	double *work; d_zeros_align(&work, d_ip2_mpc_hard_tv_work_space_size_bytes(N, nx_v, nu_v, nb_v, ng_v)/sizeof(double), 1);
#endif

/************************************************
* (new) high level interface work space
************************************************/	

	// box constraints
	double *lb0; d_zeros(&lb0, nb_v[0], 1);
	for(ii=0; ii<nb_v[0]; ii++)
		lb0[ii] = d0[ii];
	double *ub0; d_zeros(&ub0, nb_v[0], 1);
	for(ii=0; ii<nb_v[0]; ii++)
		ub0[ii] = d0[pnb_v[0]+ii];
	double *lb1; d_zeros(&lb1, nb_v[1], 1);
	for(ii=0; ii<nb_v[1]; ii++)
		lb1[ii] = d1[ii];
	double *ub1; d_zeros(&ub1, nb_v[1], 1);
	for(ii=0; ii<nb_v[1]; ii++)
		ub1[ii] = d1[pnb_v[1]+ii];
	double *lbN; d_zeros(&lbN, nb_v[N], 1);
	for(ii=0; ii<nb_v[N]; ii++)
		lbN[ii] = dN[ii];
	double *ubN; d_zeros(&ubN, nb_v[N], 1);
	for(ii=0; ii<nb_v[N]; ii++)
		ubN[ii] = dN[pnb_v[N]+ii];

	// general constraints
	double *lg0; d_zeros(&lg0, ng_v[0], 1);
	for(ii=0; ii<ng_v[0]; ii++)
		lg0[ii] = d0[2*pnb_v[0]+ii];
	double *ug0; d_zeros(&ug0, ng_v[0], 1);
	for(ii=0; ii<ng_v[0]; ii++)
		ug0[ii] = d0[2*pnb_v[0]+png_v[0]+ii];
	double *lg1; d_zeros(&lg1, ng_v[1], 1);
	for(ii=0; ii<ng_v[1]; ii++)
		lg1[ii] = d1[2*pnb_v[1]+ii];
	double *ug1; d_zeros(&ug1, ng_v[1], 1);
	for(ii=0; ii<ng_v[1]; ii++)
		ug1[ii] = d1[2*pnb_v[1]+png_v[1]+ii];
	double *lgN; d_zeros(&lgN, ng_v[N], 1);
	for(ii=0; ii<ng_v[N]; ii++)
		lgN[ii] = dN[2*pnb_v[N]+ii];
	double *ugN; d_zeros(&ugN, ng_v[N], 1);
	for(ii=0; ii<ng_v[N]; ii++)
		ugN[ii] = dN[2*pnb_v[N]+png_v[N]+ii];

	// data matrices
	double *hA[N];
	double *hB[N];
	double *hC[N+1];
	double *hD[N];
	double *hQ[N+1];
	double *hS[N];
	double *hR[N];
	double *hq[N+1];
	double *hr[N];
	double *hlb[N+1];
	double *hub[N+1];
	double *hlg[N+1];
	double *hug[N+1];
	double *hx[N+1];
	double *hu[N];
	double *hpi1[N];
	double *hlam1[N+1];
	double *ht1[N+1];
	double inf_norm_res[4] = {}; // infinity norm of residuals: rq, rb, rd, mu

	ii = 0;
	hA[0] = A;
	hB[0] = B;
	hC[0] = C;
	hD[0] = D;
	hQ[0] = Q;
	hS[0] = S;
	hR[0] = R;
	hq[0] = q;
	hr[0] = r;
	hlb[0] = lb0;
	hub[0] = ub0;
	hlg[0] = lg0;
	hug[0] = ug0;
	d_zeros(&hx[0], nx_v[0], 1);
	d_zeros(&hu[0], nu_v[0], 1);
	d_zeros(&hpi1[0], nx_v[1], 1);
	d_zeros(&hlam1[0], 2*nb_v[0]+2*ng_v[0], 1);
	d_zeros(&ht1[0], 2*nb_v[0]+2*ng_v[0], 1);
	for(ii=1; ii<N; ii++)
		{
		hA[ii] = A;
		hB[ii] = B;
		hC[ii] = C;
		hD[ii] = D;
		hQ[ii] = Q;
		hS[ii] = S;
		hR[ii] = R;
		hq[ii] = q;
		hr[ii] = r;
		hlb[ii] = lb1;
		hub[ii] = ub1;
		hlg[ii] = lg1;
		hug[ii] = ug1;
		d_zeros(&hx[ii], nx_v[ii], 1);
		d_zeros(&hu[ii], nu_v[ii], 1);
		d_zeros(&hpi1[ii], nx_v[ii+1], 1);
		d_zeros(&hlam1[ii], 2*nb_v[ii]+2*ng_v[ii], 1);
		d_zeros(&ht1[ii], 2*nb_v[ii]+2*ng_v[ii], 1);
		}
	ii = N;
	hC[N] = C;
	hQ[N] = Q;
	hq[N] = q;
	hlb[N] = lbN;
	hub[N] = ubN;
	hlg[N] = lgN;
	hug[N] = ugN;
	d_zeros(&hx[N], nx_v[N], 1);
	d_zeros(&hlam1[N], 2*nb_v[N]+2*ng_v[N], 1);
	d_zeros(&ht1[N], 2*nb_v[N]+2*ng_v[N], 1);

	// work space
#if 0
	printf("work space in bytes: %d\n", hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(N, nx_v, nu_v, nb_v, ng_v));
	exit(3);
#endif
	void *work1 = malloc(hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(N, nx_v, nu_v, nb_v, hidxb, ng_v, N2));

/************************************************
* solvers common stuff
************************************************/	

	int hpmpc_status;
	int kk, kk_avg;
	int warm_start = 0; // read initial guess from x and u
	double *stat; d_zeros(&stat, k_max, 5);
	int compute_res = 1;
	int compute_mult = 1;

	struct timeval tv0, tv1, tv2, tv3;
	double time;

	double **dummy;

/************************************************
* call the solver (high-level interface)
************************************************/	

#if 1
	int time_invariant = 0; // assume the problem to be time invariant
	int free_x0 = 0; // assume x0 as optimization variable

	gettimeofday(&tv0, NULL); // stop

	kk_avg = 0;

	for(rep=0; rep<nrep; rep++)
		{

////		hpmpc_status = fortran_order_d_ip_mpc_hard_tv(&kk, k_max, mu0, mu_tol, N, nx, nu, nb, ng, ngN, time_invariant, free_x0, warm_start, rA, rB, rb, rQ, rQf, rS, rR, rq, rqf, rr, rlb, rub, rC, rD, rlg, rug, CN, lgN, ugN, rx, ru, rpi, rlam, rt, inf_norm_res, rwork, stat);
//		hpmpc_status = fortran_order_d_ip_ocp_hard_tv(&kk, k_max, mu0, mu_tol, N, nx_v, nu_v, nb_v, hidxb, ng_v, N2, warm_start, hA, hB, hb, hQ, hS, hR, hq, hr, hlb, hub, hC, hD, hlg, hug, hx, hu, hpi1, hlam1, /*ht1,*/ inf_norm_res, work1, stat);

		kk_avg += kk;

		}
	
	gettimeofday(&tv1, NULL); // stop

	printf("\nsolution from high-level interface\n\n");
//	d_print_mat(nx, N+1, rx, nx);
//	d_print_mat(nu, N, ru, nu);
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx_v[ii], hx[ii], 1);
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nu_v[ii], hu[ii], 1);

	printf("\ninfinity norm of residuals\n\n");
#ifdef BLASFEO
	d_print_e_mat(1, 4, inf_norm_res, 1);
#else
	d_print_mat_e(1, 4, inf_norm_res, 1);
#endif

	time = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
	printf("\n");
	printf(" Average number of iterations over %d runs: %5.1f\n", nrep, kk_avg / (double) nrep);
	printf(" Average solution time over %d runs: %5.2e seconds\n", nrep, time);
	printf("\n\n");

	gettimeofday(&tv0, NULL); // stop

	kk_avg = 0;

	for(rep=0; rep<nrep; rep++)
		{

//		fortran_order_d_solve_kkt_new_rhs_mpc_hard_tv(N, nx, nu, nb, ng, ngN, time_invariant, free_x0, rA, rB, rb, rQ, rQf, rS, rR, rq, rqf, rr, rlb, rub, rC, rD, rlg, rug, CN, lgN, ugN, rx, ru, rpi, rlam, rt, inf_norm_res, rwork);
		fortran_order_d_solve_kkt_new_rhs_ocp_hard_tv(N, nx_v, nu_v, nb_v, hidxb, ng_v, hA, hB, hb, hQ, hS, hR, hq, hr, hlb, hub, hC, hD, hlg, hug, hx, hu, hpi1, hlam1, /*ht1,*/ inf_norm_res, work1);

		kk_avg += kk;

		}
	
	gettimeofday(&tv1, NULL); // stop

	printf("\nsolution from high-level interface (resolve final kkt)\n\n");
//	d_print_mat(nx, N+1, rx, nx);
//	d_print_mat(nu, N, ru, nu);
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx_v[ii], hx[ii], 1);
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nu_v[ii], hu[ii], 1);

	printf("\ninfinity norm of residuals\n\n");
#ifdef BLASFEO
	d_print_e_mat(1, 4, inf_norm_res, 1);
#else
	d_print_mat_e(1, 4, inf_norm_res, 1);
#endif

	time = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf(" Average solution time over %d runs: %5.2e seconds\n", nrep, time);
#endif

/************************************************
* call the solver (low-level interface)
************************************************/	

//	for(ii=0; ii<N; ii++)
//		d_print_pmat(nu_v[ii]+nx_v[ii]+1, nx_v[ii+1], bs, hpBAbt[ii], cnx_v[ii+1]);
//	exit(3);

	gettimeofday(&tv0, NULL); // stop

	kk_avg = 0;

	printf("\nsolution...\n");
	for(rep=0; rep<nrep; rep++)
		{

#if USE_IPM_RES
		hpmpc_status = d_ip2_res_mpc_hard_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hpRSQ, hpDCt, hd, hux, compute_mult, hpi, hlam, ht, work);
#else
		hpmpc_status = d_ip2_mpc_hard_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hpRSQ, hpDCt, hd, hux, compute_mult, hpi, hlam, ht, work);
#endif
		
		kk_avg += kk;

		}
	printf("\ndone\n");

	gettimeofday(&tv1, NULL); // stop

	printf("\nsolution from low-level interface (original problem)\n\n");
	printf("\nux\n\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu_v[ii]+nx_v[ii], hux[ii], 1);
	printf("\npi\n\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx_v[ii+1], hpi[ii], 1);
//	printf("\nux\n\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], hlam[ii], 1);
//	printf("\nux\n\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], ht[ii], 1);
	
	// residuals
	if(compute_res)
		{
		// compute residuals
		d_res_mpc_hard_tv(N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hb, hpRSQ, hrq, hux, hpDCt, hd, hpi, hlam, ht, hrrq, hrb, hrd, &mu);

		// print residuals
		printf("\nhrrq\n\n");
		for(ii=0; ii<=N; ii++)
#ifdef BLASFEO
			d_print_e_mat(1, nu_v[ii]+nx_v[ii], hrrq[ii], 1);
#else
			d_print_mat_e(1, nu_v[ii]+nx_v[ii], hrrq[ii], 1);
#endif

		printf("\nhrb\n\n");
		for(ii=0; ii<N; ii++)
#ifdef BLASFEO
			d_print_e_mat(1, nx_v[ii+1], hrb[ii], 1);
#else
			d_print_mat_e(1, nx_v[ii+1], hrb[ii], 1);
#endif

		printf("\nhrd low\n\n");
		for(ii=0; ii<=N; ii++)
#ifdef BLASFEO
			d_print_e_mat(1, nb_v[ii], hrd[ii], 1);
#else
			d_print_mat_e(1, nb_v[ii], hrd[ii], 1);
#endif

		printf("\nhrd up\n\n");
		for(ii=0; ii<=N; ii++)
#ifdef BLASFEO
			d_print_e_mat(1, nb_v[ii], hrd[ii]+pnb_v[ii], 1);
#else
			d_print_mat_e(1, nb_v[ii], hrd[ii]+pnb_v[ii], 1);
#endif

		}



	// zero the solution again
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<nu_v[ii]+nx_v[ii]; jj++) hux[ii][jj] = 0.0;

	// modify constraints
#if 0
	for(jj=0; jj<nbx; jj++)
		{
		dN[jj]          = - 4.0;   //   xmin
		dN[pnb_v[N]+jj] =   4.0;   //   xmax
		idxbN[jj] = jj;
		}
	for(jj=0; jj<ng_v[N]; jj++)
		{
		dN[2*pnb_v[N]+jj]          =   0.1;   //   xmin
		dN[2*pnb_v[N]+png_v[N]+jj] =   0.1;   //   xmax
		}
#endif

#if 0
for(ii=0; ii<=N; ii++)
	d_print_pmat(nu_v[ii]+nx_v[ii]+1, nu_v[ii]+nx_v[ii], bs, hpRSQ[ii], cnux_v[ii]);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu_v[ii]+nx_v[ii], hrq[ii], 1);
exit(1);
#endif

	gettimeofday(&tv2, NULL); // stop

	printf("\nsolution...\n");
	for(rep=0; rep<nrep; rep++)
		{

#if USE_IPM_RES
		d_kkt_solve_new_rhs_res_mpc_hard_tv(N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hb, hpRSQ, hrq, hpDCt, hd, hux, compute_mult, hpi, hlam, ht, work);
#else
		d_kkt_solve_new_rhs_mpc_hard_tv(N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hb, hpRSQ, hrq, hpDCt, hd, hux, compute_mult, hpi, hlam, ht, work);
#endif

		}
	printf("\ndone\n");

	gettimeofday(&tv3, NULL); // stop

#if 0
	printf("\nsolution from low-level interface (resolve final kkt)\n\n");
	printf("\nux\n\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu_v[ii]+nx_v[ii], hux[ii], 1);
	printf("\npi\n\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx_v[ii+1], hpi[ii], 1);
//	printf("\nux\n\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], hlam[ii], 1);
//	printf("\nux\n\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], ht[ii], 1);
#endif

	// residuals
	if(compute_res)
		{
		// compute residuals
		d_res_mpc_hard_tv(N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hb, hpRSQ, hrq, hux, hpDCt, hd, hpi, hlam, ht, hrrq, hrb, hrd, &mu);

#if 0
		// print residuals
		printf("\nhrrq\n\n");
		for(ii=0; ii<=N; ii++)
			d_print_e_mat(1, nu_v[ii]+nx_v[ii], hrrq[ii], 1);

		printf("\nhrb\n\n");
		for(ii=0; ii<N; ii++)
			d_print_e_mat(1, nx_v[ii+1], hrb[ii], 1);

		printf("\nhrd low\n\n");
		for(ii=0; ii<=N; ii++)
			d_print_e_mat(1, nb_v[ii], hrd[ii], 1);

		printf("\nhrd up\n\n");
		for(ii=0; ii<=N; ii++)
			d_print_e_mat(1, nb_v[ii], hrd[ii]+pnb_v[ii], 1);
#endif

		}

	double time_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	double time_final = (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
	printf("\n");
	printf(" Average number of iterations over %d runs: %5.1f\n", nrep, kk_avg / (double) nrep);
	printf(" Average solution time over %d runs: %5.2e seconds (IPM)\n", nrep, time_ipm);
	printf(" Average solution time over %d runs: %5.2e seconds (resolve final kkt)\n", nrep, time_final);
	printf("\n\n");

/************************************************
* compute residuals
************************************************/	

/************************************************
* free memory
************************************************/	

	// problem data
	free(A);
	free(B);
	d_free_align(b);
	d_free_align(x0);
	free(C);
	free(D);
	free(Q);
	free(S);
	free(R);
	free(q);
	free(r);

	// low level interface
	d_free_align(pA);
	d_free_align(b0);
	d_free_align(pBAbt0);
	d_free_align(pBAbt1);
	d_free_align(d0);
	d_free_align(d1);
	d_free_align(dN);
	d_free_align(pDCt0);
	d_free_align(pDCt1);
	free(DCN);
	d_free_align(pDCtN);
	free(idxb0);
	free(idxb1);
	free(idxbN);
	d_free_align(pRSQ0);
	d_free_align(pRSQ1);
	d_free_align(pRSQN);
	d_free_align(rq0);
	d_free_align(rq1);
	d_free_align(rqN);
	d_free_align(work);
	free(stat);
	for(ii=0; ii<N; ii++)
		{
		d_free_align(hux[ii]);
		d_free_align(hpi[ii]);
		d_free_align(hlam[ii]);
		d_free_align(ht[ii]);
		d_free_align(hrb[ii]);
		d_free_align(hrrq[ii]);
		d_free_align(hrd[ii]);
		}
	d_free_align(hux[N]);
	d_free_align(hlam[N]);
	d_free_align(ht[N]);
	d_free_align(hrrq[N]);
	d_free_align(hrd[N]);
	
#if 0
	// high level interface
	free(rA);
	free(rB);
	free(rC);
	free(rD);
	free(rb);
	free(rQ);
	free(rQf);
	free(rS);
	free(rR);
	free(rq);
	free(rqf);
	free(rr);
	free(lb);
	free(rlb);
	free(lg);
	free(rlg);
	free(lgN);
	free(ub);
	free(rub);
	free(ug);
	free(rug);
	free(ugN);
	free(rx);
	free(ru);
	free(rpi);
	free(rlam);
	free(rt);
	free(rwork);
#endif
	
	// new high level interface
	free(lb0);
	free(ub0);
	free(lb1);
	free(ub1);
	free(lbN);
	free(ubN);
	free(lg0);
	free(ug0);
	free(lg1);
	free(ug1);
	free(work1);
	for(ii=0; ii<N; ii++)
		{
		free(hx[ii]);
		free(hu[ii]);
		free(hpi1[ii]);
		free(hlam1[ii]);
		free(ht1[ii]);
		}
	free(hx[N]);
	free(hlam1[N]);
	free(ht1[N]);

	return 0;
	
	}


