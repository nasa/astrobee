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
	
	int rep, nrep=1000;

	int nx = 8; // number of states (it has to be even for the mass-spring system test problem)
	int nu = 3; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = 5; // horizon lenght
//	int nb  = nu+nx; // number of box constrained inputs and states
//	int ng  = nx; //4;  // number of general constraints
//	int ngN = nx; // number of general constraints at the last stage
	printf("\nN = %d, nx = %d, nu = %d\n\n", N, nx, nu);

#define MHE 0


//	int nbu = nu<nb ? nu : nb ;
//	int nbx = nb-nu>0 ? nb-nu : 0;


	// stage-wise variant size
	int nx_v[N+1];
#if MHE==1
	nx_v[0] = nx;
#else
	nx_v[0] = 0;
#endif
	for(ii=1; ii<=N; ii++)
		nx_v[ii] = nx;

	int nu_v[N+1];
	for(ii=0; ii<N; ii++)
		nu_v[ii] = nu;
	nu_v[N] = 0; // XXX

	int nb_v[N+1];
	nb_v[0] = nu_v[0] + nx_v[0]/2;
	for(ii=1; ii<N; ii++)
		nb_v[ii] = nu_v[1] + nx_v[ii]/2;
	nb_v[N] = nu_v[N] + nx_v[N]/2;

	int ng_v[N+1];
	for(ii=0; ii<N; ii++)
		ng_v[ii] = 0; //ng;
	ng_v[N] = 0; //ngN;
//	ng_v[M] = nx; // XXX
	


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

	int pnx_v[N+1]; 
	int pnz_v[N+1]; 
	int pnux_v[N+1]; 
	int pnb_v[N+1]; 
	int png_v[N+1]; 
	int cnx_v[N+1]; 
	int cnux_v[N+1]; 
	int cng_v[N+1]; 
	int nuM = 0;
	int nxM = 0;
	int nuxM = 0;
	int nu2 = 0;

	for(ii=0; ii<N; ii++) 
		{
		pnx_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
		pnz_v[ii] = (nu_v[ii]+nx_v[ii]+1+bs-1)/bs*bs;
		pnux_v[ii] = (nu_v[ii]+nx_v[ii]+bs-1)/bs*bs;
		pnb_v[ii] = (nb_v[ii]+bs-1)/bs*bs;
		png_v[ii] = (ng_v[ii]+bs-1)/bs*bs;
		cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
		cnux_v[ii] = (nu_v[ii]+nx_v[ii]+ncl-1)/ncl*ncl;
		cng_v[ii] = (ng_v[ii]+ncl-1)/ncl*ncl;
		nuM = nu_v[ii]>nuM ? nu_v[ii] : nuM;
		nxM = nx_v[ii]>nxM ? nx_v[ii] : nxM;
		nuxM = nu_v[ii]+nx_v[ii]>nuxM ? nu_v[ii]+nx_v[ii] : nuxM;
		nu2 += nu_v[ii];
		}
	ii = N;
	pnx_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
	pnz_v[ii] = (nx_v[ii]+1+bs-1)/bs*bs;
	pnux_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
	pnb_v[ii] = (nb_v[ii]+bs-1)/bs*bs;
	png_v[ii] = (ng_v[ii]+bs-1)/bs*bs;
	cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
	cnux_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
	cng_v[ii] = (ng_v[ii]+ncl-1)/ncl*ncl;
	nxM = nx_v[ii]>nxM ? nx_v[ii] : nxM;
	nuxM = nx_v[ii]>nuxM ? nx_v[ii] : nuxM;

	int pnuM = (nuM+bs-1)/bs*bs;
	int pnxM = (nxM+bs-1)/bs*bs;
	int pnx1M = (nxM+1+bs-1)/bs*bs;
	int pnzM = (nuxM+1+bs-1)/bs*bs;
	int cnxM = (nxM+ncl-1)/ncl*ncl;
	int cnuM = (nuM+ncl-1)/ncl*ncl;
	int cnuxM = (nuxM+ncl-1)/ncl*ncl;
	int pnz2 = (nu2+nx_v[0]+1+bs-1)/bs*bs;
	int cnux2 = (nu2+nx_v[0]+ncl-1)/ncl*ncl;

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

#if MHE!=1
	double *pA; d_zeros_align(&pA, pnx, cnx);
	d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);
	double *b0; d_zeros_align(&b0, pnx, 1);
#ifdef BLASFEO
	dgemv_n_lib(nx, nx, 1.0, pA, cnx, x0, 1.0, b, b0);
#else
	dgemv_n_lib(nx, nx, pA, cnx, x0, 1, b, b0);
#endif

	double *pBAbt0; 
	d_zeros_align(&pBAbt0, pnz_v[0], cnx_v[1]);
	d_cvt_tran_mat2pmat(nx_v[1], nu_v[0], B, nx_v[1], 0, pBAbt0, cnx_v[1]);
	d_cvt_tran_mat2pmat(nx_v[1], nx_v[0], A, nx_v[1], nu_v[0], pBAbt0+nu_v[0]/bs*bs*cnx_v[1]+nu_v[0]%bs, cnx_v[1]);
	d_cvt_tran_mat2pmat(nx_v[1], 1, b0, nx_v[1], nu_v[0]+nx_v[0], pBAbt0+(nu_v[0]+nx_v[0])/bs*bs*cnx_v[1]+(nu_v[0]+nx_v[0])%bs, cnx_v[1]);
//	printf("\npBAbt0 = \n");
//	d_print_pmat(nu_v[0]+nx_v[0]+1, nx_v[1], bs, pBAbt0, cnx_v[1]);
#endif

	double *pBAbt1; 
	if(N>1)
		{
		d_zeros_align(&pBAbt1, pnz_v[1], cnx_v[2]);
		d_cvt_tran_mat2pmat(nx_v[2], nu_v[1], B, nx_v[2], 0, pBAbt1, cnx_v[2]);
		d_cvt_tran_mat2pmat(nx_v[2], nx_v[1], A, nx_v[2], nu_v[1], pBAbt1+nu_v[1]/bs*bs*cnx_v[2]+nu_v[1]%bs, cnx_v[2]);
		d_cvt_tran_mat2pmat(nx_v[2], 1, b, nx_v[2], nu_v[1]+nx_v[1], pBAbt1+(nu_v[1]+nx_v[1])/bs*bs*cnx_v[2]+(nu_v[1]+nx_v[1])%bs, cnx_v[2]);
//		printf("\npBAbt1 = \n");
//		d_print_pmat(nu_v[1]+nx_v[1]+1, nx_v[2], bs, pBAbt1, cnx_v[2]);
		}
	
/************************************************
* cost function
************************************************/	

	double *R; d_zeros(&R, nu, nu);
	for(ii=0; ii<nu; ii++) R[ii*(nu+1)] = 2.0;

	double *S; d_zeros(&S, nu, nx);

	double *Q; d_zeros(&Q, nx, nx);
	for(ii=0; ii<nx; ii++) Q[ii*(nx+1)] = 1.0;

	double *r; d_zeros(&r, nu, 1);
	for(ii=0; ii<nu; ii++) r[ii] = 0.2;

	double *q; d_zeros(&q, nx, 1);
	for(ii=0; ii<nx; ii++) q[ii] = 0.1;

#if MHE!=1
	double *pS; d_zeros_align(&pS, pnu, cnx);
	d_cvt_mat2pmat(nu, nx, S, nu, 0, pS, cnx);
	double *r0; d_zeros_align(&r0, pnu, 1);
#ifdef BLASFEO
	dgemv_n_lib(nu, nx, 1.0, pS, cnx, x0, 1.0, r, r0);
#else
	dgemv_n_lib(nu, nx, pS, cnx, x0, 1, r, r0);
#endif

	double *pRSQrq0;
	d_zeros_align(&pRSQrq0, pnz_v[0], cnux_v[0]);
	d_cvt_mat2pmat(nu_v[0], nu_v[0], R, nu, 0, pRSQrq0, cnux_v[0]);
	d_cvt_tran_mat2pmat(nu_v[0], 1, r0, nu, nu_v[0], pRSQrq0+nu_v[0]/bs*bs*cnux_v[0]+nu_v[0]%bs, cnux_v[0]);
//	printf("\npRSQrq0 = \n");
//	d_print_pmat(nu_v[0]+1, nu_v[0], bs, pRSQrq0, cnux_v[0]);

	double *rq0;
	d_zeros_align(&rq0, pnz_v[0], 1);
	d_copy_mat(nu_v[0], 1, r0, nu, rq0, nu_v[0]+nx_v[0]);
//	printf("\nrq0 = \n");
//	d_print_mat(1, nu_v[0]+1, rq0, 1);
#endif

	double *pRSQrq1;
	double *rq1;
	if(N>1)
		{
		d_zeros_align(&pRSQrq1, pnz_v[1], cnux_v[1]);
		d_cvt_mat2pmat(nu_v[1], nu_v[1], R, nu, 0, pRSQrq1, cnux_v[1]);
		d_cvt_tran_mat2pmat(nu_v[1], nx_v[1], S, nu, nu_v[1], pRSQrq1+nu_v[1]/bs*bs*cnux_v[1]+nu_v[1]%bs, cnux_v[1]);
		d_cvt_mat2pmat(nx_v[1], nx_v[1], Q, nx, nu_v[1], pRSQrq1+nu_v[1]/bs*bs*cnux_v[1]+nu_v[1]%bs+nu_v[1]*bs, cnux_v[1]);
		d_cvt_tran_mat2pmat(nu_v[1], 1, r, nu, nu_v[1]+nx_v[1], pRSQrq1+(nu_v[1]+nx_v[1])/bs*bs*cnux_v[1]+(nu_v[1]+nx_v[1])%bs, cnux_v[1]);
		d_cvt_tran_mat2pmat(nx_v[1], 1, q, nx, nu_v[1]+nx_v[1], pRSQrq1+(nu_v[1]+nx_v[1])/bs*bs*cnux_v[1]+(nu_v[1]+nx_v[1])%bs+nu_v[1]*bs, cnux_v[1]);
//		printf("\npRSQrq1 = \n");
//		d_print_pmat(nu_v[1]+nx_v[1]+1, nu_v[1]+nx_v[1], bs, pRSQrq1, cnux_v[1]);

		d_zeros_align(&rq1, pnz_v[1], 1);
		d_copy_mat(nu_v[1], 1, r, nu, rq1, nu_v[1]+nx_v[1]);
		d_copy_mat(nx_v[1], 1, q, nx, rq1+nu_v[1], nu_v[1]+nx_v[1]);
//		printf("\nrq1 = \n");
//		d_print_mat(1, nu_v[1]+nx_v[1]+1, rq1, 1);

		}

	double *pRSQrqN;
	d_zeros_align(&pRSQrqN, pnz_v[N], cnux_v[N]);
	d_cvt_mat2pmat(nx_v[N], nx_v[N], Q, nx, 0, pRSQrqN, cnux_v[N]);
	d_cvt_tran_mat2pmat(nx_v[N], 1, q, nx, 0, pRSQrqN+nx_v[N]/bs*bs*cnux_v[N]+nx_v[N]%bs, cnux_v[N]);
//	printf("\npRSQrqN = \n");
//	d_print_pmat(nx_v[N]+1, nx_v[N], bs, pRSQrqN, cnux_v[N]);

	double *rqN;
	d_zeros_align(&rqN, pnz_v[N], 1);
	d_copy_mat(nx_v[N], 1, q, nx, rqN, nx_v[N]);
//	printf("\nrqN = \n");
//	d_print_mat(1, nx_v[N]+1, rqN, 1);

/************************************************
* constraints
************************************************/	

#if MHE!=1
	double *d0; d_zeros_align(&d0, 2*pnb_v[0], 1);
	int *idxb0; i_zeros(&idxb0, nb_v[0], 1);
	// inputs
	for(ii=0; ii<nu_v[0]; ii++)
		{
		d0[0*pnb_v[0]+ii] = - 0.5; // u_min
		d0[1*pnb_v[0]+ii] = + 0.5; // u_max
		idxb0[ii] = ii;
		}
	// states
	for( ; ii<nb_v[0]; ii++)
		{
		d0[0*pnb_v[0]+ii] = - 800.0; // x_min
		d0[1*pnb_v[0]+ii] = + 800.0; // x_max
		idxb0[ii] = ii;
		}
#endif

	double *d1; 
	int *idxb1; 
	if(N>1)
		{
		d_zeros_align(&d1, 2*pnb_v[1], 1);
		i_zeros(&idxb1, nb_v[1], 1);
		// inputs
		for(ii=0; ii<nu_v[1]; ii++)
			{
			d1[0*pnb_v[1]+ii] = - 0.5; // u_min
			d1[1*pnb_v[1]+ii] = + 0.5; // u_max
			idxb1[ii] = ii;
			}
		// states
		for( ; ii<nb_v[1]; ii++)
			{
			d1[0*pnb_v[1]+ii] = - 800.0; // x_min
			d1[1*pnb_v[1]+ii] = + 800.0; // x_max
			idxb1[ii] = ii;
			}
		}

	double *dN; d_zeros_align(&dN, 2*pnb_v[N], 1);
	int *idxbN; i_zeros(&idxbN, nb_v[N], 1);
	// no inputs
	// states
	for(ii=0 ; ii<nb_v[N]; ii++)
		{
		dN[0*pnb_v[N]+ii] = - 800.0; // x_min
		dN[1*pnb_v[N]+ii] = + 800.0; // x_max
		idxbN[ii] = ii;
		}

/************************************************
* array of matrices & work space
************************************************/	

	double *hpBAbt[N];
	double *hpRSQrq[N+1];
	double *hrq[N+1];
	double *hpDCt[N+1];
	double *hd[N+1];
	int *hidxb[N+1];
	double *hpGamma[N];
	double *pBAbt2;
	double *pRSQrq2;
	double *pDCt2;
	double *d2;
	int *idxb2;
	double *work0;
	double *work1;

	int nu_tmp = 0;

	nu_tmp += nu_v[0];
#if MHE!=1
	hpBAbt[0] = pBAbt0;
	hpRSQrq[0] = pRSQrq0;
	hrq[0] = rq0;
	hd[0] = d0;
	hidxb[0] = idxb0;
#else
	hpBAbt[0] = pBAbt1;
	hpRSQrq[0] = pRSQrq1;
	hrq[0] = pq1;
	hd[0] = d1;
	hidxb[0] = idxb1;
#endif
	d_zeros_align(&hpGamma[0], (nx_v[0]+1+nu_tmp+bs-1)/bs*bs, cnx_v[1]);
	for(ii=1; ii<N; ii++)
		{
		nu_tmp += nu_v[ii];
		hpBAbt[ii] = pBAbt1;
		hpRSQrq[ii] = pRSQrq1;
		hrq[ii] = rq1;
		hd[ii] = d1;
		hidxb[ii] = idxb1;
		d_zeros_align(&hpGamma[ii], (nx_v[0]+1+nu_tmp+bs-1)/bs*bs, cnx_v[ii+1]);
		}
	hpRSQrq[N] = pRSQrqN;
	hrq[N] = rqN;
	hd[N] = dN;
	hidxb[N] = idxbN;

	// data matrices for the condensed problem
	d_zeros_align(&pBAbt2, pnz2, cnx_v[N]);
	d_zeros_align(&pRSQrq2, pnz2, cnux2);

	int nbb = nb_v[0]; // box that remain box constraints
	int nbg = 0; // box that become general constraints
	for(ii=1; ii<N; ii++)
		for(jj=0; jj<nb_v[ii]; jj++)
			if(hidxb[ii][jj]<nu_v[ii])
				nbb++;
			else
				nbg++;

	int cnbg = (nbg+ncl-1)/ncl*ncl;
	int pnbb = (nbb+bs-1)/bs*bs;
	int pnbg = (nbg+bs-1)/bs*bs;

	d_zeros_align(&pDCt2, pnz2, cnbg);
	d_zeros_align(&d2, 2*pnbb+2*pnbg, 1);
	i_zeros(&idxb2, nbb, 1);


	d_zeros_align(&work0, pnxM*cnxM+pnz2*cnxM, 1);
	d_zeros_align(&work1, pnx1M*cnxM+pnzM*cnuxM+pnzM*cnxM+pnuM*cnxM+pnz2*cnuM+pnzM, 1);
	
//	for(ii=0; ii<N; ii++)
//		{
//		d_print_pmat(nu_v[ii]+nx_v[ii]+1, nx_v[ii+1], bs, hpBAbt[ii], cnx_v[ii]);
//		}
	
/************************************************
* solve full spase system using Riccati / IPM
************************************************/	

	// result vectors
	double *hux[N+1];
	double *hpi[N];
	double *hlam[N+1];
	double *ht[N+1];
	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hux[ii], pnux_v[ii], 1);
		d_zeros_align(&hpi[ii], pnx_v[ii+1], 1);
		d_zeros_align(&hlam[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		d_zeros_align(&ht[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		}
	ii = N;
	d_zeros_align(&hux[ii], pnux_v[ii], 1);
	d_zeros_align(&hlam[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
	d_zeros_align(&ht[ii], 2*pnb_v[ii]+2*png_v[ii], 1);

		// IPM stuff
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

	double **dummy;

	// work space
	double *work_ipm_full; d_zeros_align(&work_ipm_full, d_ip2_res_mpc_hard_tv_work_space_size_bytes(N, nx_v, nu_v, nb_v, ng_v)/sizeof(double), 1);

	struct timeval tv0, tv1;

	printf("\nsolving... (full space system)\n");

	gettimeofday(&tv0, NULL); // stop

	for(rep=0; rep<nrep; rep++)
		{

//		d_back_ric_rec_sv_tv_res(N2, nx2_v, nu2_v, nb2_v, idxb2, ng2_v, 0, hpBAbt2, dummy, 0, hpRSQrq2, dummy, dummy, dummy, dummy, dummy, hux2, 1, hpi2, 0, dummy, memory_ric_cond, work_ric_cond);
//		hpmpc_status = d_ip2_res_mpc_hard_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hpRSQrq, hpDCt, hd, hux, compute_mult, hpi, hlam, ht, work_ipm_full);

		}

	gettimeofday(&tv1, NULL); // stop

	float time_ipm_full = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
	printf("\nux =\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu_v[ii]+nx_v[ii], hux[ii], 1);

	printf("\npi =\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx_v[ii+1], hpi[ii], 1);

	printf("\nlam =\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], hlam[ii], 1);

	printf("\nt =\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], ht[ii], 1);

/************************************************
* full condensing
************************************************/	

//	d_cond_B(N, nx_v, nu_v, hpBAbt, work, hpGamma_u, pBAbt2);
//	d_cond_A(N, nx_v, nu_v, hpBAbt, work, hpGamma_x0, pBAbt2);
//	d_cond_b(N, nx_v, nu_v, hpBAbt, work, hpGamma_b, pBAbt2);
//	d_cond_BAb(N, nx_v, nu_v, hpBAbt, work, hpGamma_u, hpGamma_x0, hpGamma_b, pBAbt2);
	
//	printf("\nGamma_x0\n\n");
//	for(ii=0; ii<N; ii++)
//		d_print_pmat(nx_v[0], nx_v[ii+1], bs, hpGamma_x0[ii], cnx_v[ii+1]);
	
//	printf("\nGamma_u\n\n");
//	nu_tmp = 0;
//	for(ii=0; ii<N; ii++)
//		{
//		nu_tmp += nu_v[ii];
//		d_print_pmat(nu_tmp, nx_v[ii+1], bs, hpGamma_u[ii], cnx_v[ii+1]);
//		}
	
//	printf("\nGamma_b\n\n");
//	for(ii=0; ii<N; ii++)
//		d_print_mat(1, nx_v[ii+1], hpGamma_b[ii], 1);
	
//	printf("\nBAbt2\n\n");
//	d_print_pmat(nu2+nx_v[0]+1, nx_v[N], bs, pBAbt2, cnx_v[N]);

	// routine packing the Gammas
	// zero the solution
	for(ii=0; ii<pnz2*cnx_v[N]; ii++) pBAbt2[ii] = 0;

	d_cond_BAbt(N, nx_v, nu_v, hpBAbt, work0, hpGamma, pBAbt2);
	
	printf("\nGamma\n\n");
	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nu_tmp += nu_v[ii];
		d_print_pmat(nx_v[0]+1+nu_tmp, nx_v[ii+1], hpGamma[ii], cnx_v[ii+1]);
		}
	
	printf("\nBAbt2\n\n");
	d_print_pmat(nu2+nx_v[0]+1, nx_v[N], pBAbt2, cnx_v[N]);


	d_cond_RSQrq(N, nx_v, nu_v, hpBAbt, hpRSQrq, hpGamma, work1, pRSQrq2);

	printf("\nRSQrq2\n\n");
	d_print_pmat(nu2+nx_v[0]+1, nu2+nx_v[0], pRSQrq2, cnux2);


	d_cond_DCtd(N, nx_v, nu_v, nb_v, hidxb, hd, hpGamma, pDCt2, d2, idxb2);

	printf("\nDCt2\n\n");
	d_print_pmat(nu2+nx_v[0], nbg, pDCt2, cnbg);
	d_print_mat(1, nbb, d2, 1);
	d_print_mat(1, nbb, d2+pnbb, 1);
	d_print_mat(1, nbg, d2+2*pnbb, 1);
	d_print_mat(1, nbg, d2+2*pnbb+pnbg, 1);
	i_print_mat(1, nbb, idxb2, 1);

/************************************************
* solve condensed system using Riccati / IPM
************************************************/	
	
	// problem size
	int N2 = 1;

	int nx2_v[N2+1];
	nx2_v[0] = nx_v[0];
	nx2_v[1] = nx_v[N];

	int nu2_v[N2];
	nu2_v[0] = nu2;
	nu2_v[1] = 0; // XXX

	int nb2_v[N2+1];
	nb2_v[0] = nbb;
	nb2_v[1] = nb_v[N];

	int ng2_v[N2+1];
	ng2_v[0] = nbg; // + ng...
	ng2_v[1] = 0; // ng[N];

	int *hidxb2[N2+1];
	hidxb2[0] = idxb2;
	hidxb2[1] = hidxb[N];


	int pnux2_v[N2+1];
	int pnx2_v[N2+1];
	int pnb2_v[N2+1];
	int png2_v[N2+1];
	for(ii=0; ii<N2; ii++)
		{
		pnux2_v[ii] = (nu2_v[ii]+nx2_v[ii]+bs-1)/bs*bs;
		pnx2_v[ii] = (nx2_v[ii]+bs-1)/bs*bs;
		pnb2_v[ii] = (nb2_v[ii]+bs-1)/bs*bs;
		png2_v[ii] = (ng2_v[ii]+bs-1)/bs*bs;
		}
	ii = N2;
	pnux2_v[ii] = (nx2_v[ii]+bs-1)/bs*bs;
	pnx2_v[ii] = (nx2_v[ii]+bs-1)/bs*bs;
	pnb2_v[ii] = (nb2_v[ii]+bs-1)/bs*bs;
	png2_v[ii] = (ng2_v[ii]+bs-1)/bs*bs;

	
	// data matrices

	double *hpBAbt2[N2];
	hpBAbt2[0] = pBAbt2;

	double *hpRSQrq2[N2+1];
	hpRSQrq2[0] = pRSQrq2;
	hpRSQrq2[1] = hpRSQrq[N];

	double *hpDCt2[N2+1];
	hpDCt2[0] = pDCt2;
	hpDCt2[1] = hpDCt[N];

	double *hd2[N+1];
	hd2[0] = d2;
	hd2[1] = hd[N];

	double *hux2[N2+1];
	double *hpi2[N2];
	double *hlam2[N2+1];
	double *ht2[N2+1];
	for(ii=0; ii<N2; ii++)
		{
		d_zeros_align(&hux2[ii], pnux2_v[ii], 1);
		d_zeros_align(&hpi2[ii], pnx2_v[ii+1], 1);
		d_zeros_align(&hlam2[ii], 2*pnb2_v[ii]+2*png2_v[ii], 1);
		d_zeros_align(&ht2[ii], 2*pnb2_v[ii]+2*png2_v[ii], 1);
		}
	ii = N2;
	d_zeros_align(&hux2[ii], pnux2_v[ii], 1);
	d_zeros_align(&hlam2[ii], 2*pnb2_v[ii]+2*png2_v[ii], 1);
	d_zeros_align(&ht2[ii], 2*pnb2_v[ii]+2*png2_v[ii], 1);


	// work space

	double *work_ric_cond; d_zeros_align(&work_ric_cond, d_back_ric_rec_sv_tv_work_space_size_bytes(N2, nx2_v, nu2_v, nb2_v, ng2_v)/sizeof(double), 1);
	double *memory_ric_cond; d_zeros_align(&memory_ric_cond, d_back_ric_rec_sv_tv_memory_space_size_bytes(N2, nx2_v, nu2_v, nb2_v, ng2_v)/sizeof(double), 1);
	double *work_ipm_cond; d_zeros_align(&work_ipm_cond, d_ip2_res_mpc_hard_tv_work_space_size_bytes(N2, nx2_v, nu2_v, nb2_v, ng2_v)/sizeof(double), 1);



	// call solver
	printf("\nsolving... (condensed system)\n");

	gettimeofday(&tv0, NULL); // stop

	for(rep=0; rep<nrep; rep++)
		{

//		d_back_ric_rec_sv_tv_res(N2, nx2_v, nu2_v, nb2_v, idxb2, ng2_v, 0, hpBAbt2, dummy, 0, hpRSQrq2, dummy, dummy, dummy, dummy, dummy, hux2, 1, hpi2, 0, dummy, memory_ric_cond, work_ric_cond);
		hpmpc_status = d_ip2_res_mpc_hard_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N2, nx2_v, nu2_v, nb2_v, hidxb2, ng2_v, hpBAbt2, hpRSQrq2, hpDCt2, hd2, hux2, compute_mult, hpi2, hlam2, ht2, work_ipm_cond);

		}

	gettimeofday(&tv1, NULL); // stop

	float time_ipm_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\ndone! (status = %d)\n", hpmpc_status);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
	printf("\nux2 =\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, nu2_v[ii]+nx2_v[ii], hux2[ii], 1);

	printf("\npi2 =\n");
	for(ii=0; ii<N2; ii++)
		d_print_mat(1, nx2_v[ii+1], hpi2[ii], 1);

	printf("\nlam2 =\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, 2*pnb2_v[ii]+2*png2_v[ii], hlam2[ii], 1);

	printf("\nt2 =\n");
	for(ii=0; ii<=N2; ii++)
		d_print_mat(1, 2*pnb2_v[ii]+2*png2_v[ii], ht2[ii], 1);

	// convert result vectors to full space formulation
	for(ii=0; ii<N; ii++)
		d_copy_mat(nu, 1, hux2[0]+(N-ii-1)*nu, nu, hux[ii], nu);
	
	// copy x0
	for(jj=0; jj<nx_v[0]; jj++) hux[0][nu_v[0]+jj] = hux2[0][nu2_v[0]+jj];

	// simulate the system
	for(ii=0; ii<N; ii++)
		{
		for(jj=0; jj<nx_v[ii+1]; jj++)
			hux[ii+1][nu_v[ii+1]+jj] = hpBAbt[ii][(nu_v[ii]+nx_v[ii])/bs*bs*cnx_v[ii+1]+(nu_v[ii]+nx_v[ii])%bs+jj*bs];
#ifdef BLASFEO
		dgemv_t_lib(nu_v[ii]+nx_v[ii], nx_v[ii+1], 1.0, hpBAbt[ii], cnx_v[ii+1], hux[ii], 1.0, hux[ii+1]+nu_v[ii+1], hux[ii+1]+nu_v[ii+1]);
#else
		dgemv_t_lib(nu_v[ii]+nx_v[ii], nx_v[ii+1], hpBAbt[ii], cnx_v[ii+1], hux[ii], 1, hux[ii+1]+nu_v[ii+1], hux[ii+1]+nu_v[ii+1]);
#endif
		}
	// compute lagrangian multipliers TODO
	
	printf("\nux =\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu_v[ii]+nx_v[ii], hux[ii], 1);

/************************************************
* partial condensing
************************************************/	
	
	int N3 = 2;
	int nx3_v[N3+1];
	int nu3_v[N3+1];
	int nb3_v[N3+1];
	int ng3_v[N3+1];

	int *hidxb3[N3+1];

	double *hpBAbt3[N3];
	double *hpRSQrq3[N3+1];
	double *hpDCt3[N3+1];
	double *hd3[N3+1];

	void *memory_part_cond;
	v_zeros_align(&memory_part_cond, d_part_cond_memory_space_size_bytes(N, nx_v, nu_v, nb_v, hidxb, ng_v, N3, nx3_v, nu3_v, nb3_v, ng3_v));

	void *work_part_cond;
	v_zeros_align(&work_part_cond, d_part_cond_work_space_size_bytes(N, nx_v, nu_v, nb_v, hidxb, ng_v, N3, nx3_v, nu3_v, nb3_v, ng3_v));

	d_part_cond(N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hpRSQrq, hpDCt, hd, N3, nx3_v, nu3_v, nb3_v, hidxb3, ng3_v, hpBAbt3, hpRSQrq3, hpDCt3, hd3, memory_part_cond, work_part_cond);

//	for(ii=0; ii<=N3; ii++)
//		printf("\n%d %d %d %d\n", nx3_v[ii], nu3_v[ii], nb3_v[ii], ng3_v[ii]);
	
	int pnx3_v[N3+1];
	int pnb3_v[N3+1];
	int png3_v[N3+1];
	int pnux3_v[N3+1];
	int cnx3_v[N3+1];
	int cnux3_v[N3+1];
	int cng3_v[N3+1];
	for(ii=0; ii<=N3; ii++)
		{
		pnx3_v[ii] = (nx3_v[ii]+bs-1)/bs*bs;
		pnb3_v[ii] = (nb3_v[ii]+bs-1)/bs*bs;
		png3_v[ii] = (ng3_v[ii]+bs-1)/bs*bs;
		pnux3_v[ii] = (nu3_v[ii]+nx3_v[ii]+bs-1)/bs*bs;
		cnx3_v[ii] = (nx3_v[ii]+ncl-1)/ncl*ncl;
		cnux3_v[ii] = (nu3_v[ii]+nx3_v[ii]+ncl-1)/ncl*ncl;
		cng3_v[ii] = (ng3_v[ii]+ncl-1)/ncl*ncl;
		}
	
//	printf("\nBAbt3 =\n");
//	for(ii=0; ii<N3; ii++)
//		d_print_pmat(nu3_v[ii]+nx3_v[ii]+1, nx3_v[ii+1], hpBAbt3[ii], cnx3_v[ii+1]);
	
//	printf("\nRSQrq3 =\n");
//	for(ii=0; ii<=N3; ii++)
//		d_print_pmat(nu3_v[ii]+nx3_v[ii]+1, nu3_v[ii]+nx3_v[ii], bs, hpRSQrq3[ii], cnux3_v[ii]);
	
//	printf("\nDCt3 =\n");
//	for(ii=0; ii<=N3; ii++)
//		d_print_pmat(nu3_v[ii]+nx3_v[ii], ng3_v[ii], bs, hpDCt3[ii], cng3_v[ii]);
	
//	printf("\nd3 =\n");
//	for(ii=0; ii<=N3; ii++)
//		d_print_mat(1, 2*pnb3_v[ii]+2*png3_v[ii], hd3[ii], 1);
	
//	printf("\nidxb3 = \n");
//	for(ii=0; ii<=N3; ii++)
//		i_print_mat(1, nb3_v[ii], hidxb3[ii], 1);
	
/************************************************
* solve partially condensed system using IPM
************************************************/	
	
	double *hux3[N3+1];
	double *hpi3[N3];
	double *hlam3[N3+1];
	double *ht3[N3+1];
	for(ii=0; ii<N3; ii++)
		{
		d_zeros_align(&hux3[ii], pnux3_v[ii], 1);
		d_zeros_align(&hpi3[ii], pnx3_v[ii+1], 1);
		d_zeros_align(&hlam3[ii], 2*pnb3_v[ii]+2*png3_v[ii], 1);
		d_zeros_align(&ht3[ii], 2*pnb3_v[ii]+2*png3_v[ii], 1);
		}
	ii = N3;
	d_zeros_align(&hux3[ii], pnux3_v[ii], 1);
	d_zeros_align(&hlam3[ii], 2*pnb3_v[ii]+2*png3_v[ii], 1);
	d_zeros_align(&ht3[ii], 2*pnb3_v[ii]+2*png3_v[ii], 1);


	// work space
	double *work_ipm3; d_zeros_align(&work_ipm3, d_ip2_res_mpc_hard_tv_work_space_size_bytes(N3, nx3_v, nu3_v, nb3_v, ng3_v)/sizeof(double), 1);



	// same IPM stuff



	// call solver
	printf("\nsolving (partial condensed system)...\n");

	gettimeofday(&tv0, NULL); // stop

	for(rep=0; rep<nrep; rep++)
		{

		d_part_cond(N, nx_v, nu_v, nb_v, hidxb, ng_v, hpBAbt, hpRSQrq, hpDCt, hd, N3, nx3_v, nu3_v, nb3_v, hidxb3, ng3_v, hpBAbt3, hpRSQrq3, hpDCt3, hd3, memory_part_cond, work_part_cond);

//		d_back_ric_rec_sv_tv_res(N2, nx2_v, nu2_v, nb2_v, idxb2, ng2_v, 0, hpBAbt2, dummy, 0, hpRSQrq2, dummy, dummy, dummy, dummy, dummy, hux2, 1, hpi2, 0, dummy, memory_ric_cond, work_ric_cond);
		hpmpc_status = d_ip2_res_mpc_hard_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N3, nx3_v, nu3_v, nb3_v, hidxb3, ng3_v, hpBAbt3, hpRSQrq3, hpDCt3, hd3, hux3, compute_mult, hpi3, hlam3, ht3, work_ipm3);

		}

	gettimeofday(&tv1, NULL); // stop

	printf("\ndone! (status = %d)\n", hpmpc_status);

	float time_ipm_part_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\nstatistics from last run\n\n");
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");
	
	printf("\nux3 =\n");
	for(ii=0; ii<=N3; ii++)
		d_print_mat(1, nu3_v[ii]+nx3_v[ii], hux3[ii], 1);
	
	printf("\npi3 =\n");
	for(ii=0; ii<N3; ii++)
		d_print_mat(1, nx3_v[ii+1], hpi3[ii], 1);

	printf("\nlam3 =\n");
	for(ii=0; ii<=N3; ii++)
		d_print_mat(1, 2*png3_v[ii]+2*png3_v[ii], hlam3[ii], 1);
	
	printf("\nt3 =\n");
	for(ii=0; ii<=N3; ii++)
		d_print_mat(1, 2*png3_v[ii]+2*png3_v[ii], ht3[ii], 1);

	// convert result vectors to full space formulation
//	double *hux[N];
//	for(ii=0; ii<=N; ii++)
//		d_zeros_align(&hux[ii], pnux_v[ii], 1);
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<pnux_v[ii]; jj++)
			hux[ii][jj] = 0.0;
	
	for(ii=0; ii<N; ii++)
		for(jj=0; jj<pnx_v[ii+1]; jj++)
			hpi[ii][jj] = 0.0;
	
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<2*pnb_v[ii]+2*png_v[ii]; jj++)
			hlam[ii][jj] = 0.0;
	
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<2*pnb_v[ii]+2*png_v[ii]; jj++)
			ht[ii][jj] = 0.0;
	
//	for(ii=0; ii<N; ii++)
//		d_copy_mat(nu, 1, hux2[0]+(N-ii-1)*nu, nu, hux[ii], nu);
	int N1 = N/N3; // (floor) horizon of small blocks
	int R1 = N - N3*N1; // the first R1 blocks have horizion N1+1
	int M1 = R1>0 ? N1+1 : N1; // (ceil) horizon of large blocks
	int T1; // horizon of current block
	int N_tmp = 0;
	for(ii=0; ii<N3; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		nu_tmp = 0;
		// final stages: copy only input
		for(jj=0; jj<T1-1; jj++)
			{
			for(kk=0; kk<nu_v[N_tmp+T1-1-jj]; kk++)
				hux[N_tmp+T1-1-jj][kk] = hux3[ii][nu_tmp+kk];
			nu_tmp += nu_v[N_tmp+T1-1-jj];
			}
		// first stage: copy input and state
		for(kk=0; kk<nu_v[N_tmp+0]+nx_v[N_tmp+0]; kk++)
			hux[N_tmp+0][kk] = hux3[ii][nu_tmp+kk];
		//
		N_tmp += T1;
		}
	// last stage: just copy state
	for(kk=0; kk<nx_v[N]; kk++)
		hux[N][kk] = hux3[N3][kk];
	// compute missing states by simulation within each block
	N_tmp = 0;
	for(ii=0; ii<N3; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		for(jj=0; jj<T1-1; jj++) // last stage is already there !!!
			{
			for(kk=0; kk<nx_v[N_tmp+jj+1]; kk++)
				hux[N_tmp+jj+1][nu_v[N_tmp+jj+1]+kk] = hpBAbt[N_tmp+jj][(nu_v[N_tmp+jj]+nx_v[N_tmp+jj])/bs*bs*cnx_v[N_tmp+jj+1]+(nu_v[N_tmp+jj]+nx_v[N_tmp+jj])%bs+kk*bs];
#ifdef BLASFEO
			dgemv_t_lib(nu_v[N_tmp+jj]+nx_v[N_tmp+jj], nx_v[N_tmp+jj+1], 1.0, hpBAbt[N_tmp+jj], cnx_v[N_tmp+jj+1], hux[N_tmp+jj], 1.0, hux[N_tmp+jj+1]+nu_v[N_tmp+jj+1], hux[N_tmp+jj+1]+nu_v[N_tmp+jj+1]);
#else
			dgemv_t_lib(nu_v[N_tmp+jj]+nx_v[N_tmp+jj], nx_v[N_tmp+jj+1], hpBAbt[N_tmp+jj], cnx_v[N_tmp+jj+1], hux[N_tmp+jj], 1, hux[N_tmp+jj+1]+nu_v[N_tmp+jj+1], hux[N_tmp+jj+1]+nu_v[N_tmp+jj+1]);
#endif
			}
		//
		N_tmp += T1;
		}
	
	// copy x0
//	for(jj=0; jj<nx_v[0]; jj++) hux[0][nu_v[0]+jj] = hux3[0][nu3_v[0]+jj];

	// simulate the system
//	for(ii=0; ii<N; ii++)
//		{
//		for(jj=0; jj<nx_v[ii+1]; jj++)
//			hux[ii+1][nu_v[ii+1]+jj] = hpBAbt[ii][(nu_v[ii]+nx_v[ii])/bs*bs*cnx_v[ii+1]+(nu_v[ii]+nx_v[ii])%bs+jj*bs];
//		dgemv_t_lib(nu_v[ii]+nx_v[ii], nx_v[ii+1], hpBAbt[ii], cnx_v[ii+1], hux[ii], 1, hux[ii+1]+nu_v[ii+1], hux[ii+1]+nu_v[ii+1]);
//		}
	
	// slack variables and ineq lagrange multipliers
	int nbb3, nbg3, nbb3_tmp, nbg3_tmp;
	N_tmp = 0;
	for(ii=0; ii<N3; ii++)
		{
		nbb3_tmp = 0;
		nbg3_tmp = 0;
		T1 = ii<R1 ? M1 : N1;
		// final stages
		for(jj=0; jj<T1-1; jj++)
			{
			nbb3 = 0;
			nbg3 = 0;
			for(kk=0; kk<nb_v[N_tmp+T1-1-jj]; kk++)
				if(hidxb[N_tmp+T1-1-jj][kk]<nu_v[N_tmp+T1-1-jj])
					nbb3++;
				else
					nbg3++;
			for(kk=0; kk<nbb3; kk++) // box as box
				{
				hlam[N_tmp+T1-1-jj][0*pnb_v[N_tmp+T1-1-jj]+kk] = hlam3[ii][0*pnb3_v[ii]+nbb3_tmp+kk];
				hlam[N_tmp+T1-1-jj][1*pnb_v[N_tmp+T1-1-jj]+kk] = hlam3[ii][1*pnb3_v[ii]+nbb3_tmp+kk];
				ht[N_tmp+T1-1-jj][0*pnb_v[N_tmp+T1-1-jj]+kk] = ht3[ii][0*pnb3_v[ii]+nbb3_tmp+kk];
				ht[N_tmp+T1-1-jj][1*pnb_v[N_tmp+T1-1-jj]+kk] = ht3[ii][1*pnb3_v[ii]+nbb3_tmp+kk];
				}
			for(kk=0; kk<nbg3; kk++) // box as general XXX change when decide where nbg are placed wrt ng
				{
				hlam[N_tmp+T1-1-jj][0*pnb_v[N_tmp+T1-1-jj]+nbb3+kk] = hlam3[ii][2*pnb3_v[ii]+0*png3_v[ii]+nbg3_tmp+kk];
				hlam[N_tmp+T1-1-jj][1*pnb_v[N_tmp+T1-1-jj]+nbb3+kk] = hlam3[ii][2*pnb3_v[ii]+1*png3_v[ii]+nbg3_tmp+kk];
				ht[N_tmp+T1-1-jj][0*pnb_v[N_tmp+T1-1-jj]+nbb3+kk] = ht3[ii][2*pnb3_v[ii]+0*png3_v[ii]+nbg3_tmp+kk];
				ht[N_tmp+T1-1-jj][1*pnb_v[N_tmp+T1-1-jj]+nbb3+kk] = ht3[ii][2*pnb3_v[ii]+1*png3_v[ii]+nbg3_tmp+kk];
				}
			nbb3_tmp += nbb3;
			nbg3_tmp += nbg3;
			}
		// first stage
		for(kk=0; kk<nb_v[N_tmp+0]; kk++) // all remain box
			{
			hlam[N_tmp+T1-1-jj][0*pnb_v[N_tmp+T1-1-jj]+kk] = hlam3[ii][0*pnb3_v[ii]+nbb3_tmp+kk];
			hlam[N_tmp+T1-1-jj][1*pnb_v[N_tmp+T1-1-jj]+kk] = hlam3[ii][1*pnb3_v[ii]+nbb3_tmp+kk];
			ht[N_tmp+T1-1-jj][0*pnb_v[N_tmp+T1-1-jj]+kk] = ht3[ii][0*pnb3_v[ii]+nbb3_tmp+kk];
			ht[N_tmp+T1-1-jj][1*pnb_v[N_tmp+T1-1-jj]+kk] = ht3[ii][1*pnb3_v[ii]+nbb3_tmp+kk];
			}
//		nbb3_tmp += nbb3;
//		nbg3_tmp += nbg3;
		//
		N_tmp += T1;
		}
	// last stage: just copy
	for(jj=0; jj<nb_v[N]; jj++)
		{
		hlam[N][0*pnb_v[N]+jj] = hlam3[N3][0*pnb3_v[N3]+jj];
		hlam[N][1*pnb_v[N]+jj] = hlam3[N3][1*pnb3_v[N3]+jj];
		ht[N][0*pnb_v[N]+jj] = ht3[N3][0*pnb3_v[N3]+jj];
		ht[N][1*pnb_v[N]+jj] = ht3[N3][1*pnb3_v[N3]+jj];
		}
	for(jj=0; jj<ng_v[N]; jj++)
		{
		hlam[N][2*pnb_v[N]+0*png_v[N]+jj] = hlam3[N3][2*pnb3_v[N3]+0*png3_v[N3]+jj];
		hlam[N][2*pnb_v[N]+1*png_v[N]+jj] = hlam3[N3][2*pnb3_v[N3]+1*png3_v[N3]+jj];
		ht[N][2*pnb_v[N]+0*png_v[N]+jj] = ht3[N3][2*pnb3_v[N3]+0*png3_v[N3]+jj];
		ht[N][2*pnb_v[N]+1*png_v[N]+jj] = ht3[N3][2*pnb3_v[N3]+1*png3_v[N3]+jj];
		}
	
	// lagrange multipliers of equality constraints
	double *pi_work; d_zeros_align(&pi_work, pnz_v[1], 1);
	double *work_tmp; d_zeros_align(&work_tmp, png_v[1], 1);
	N_tmp = 0;
	for(ii=0; ii<N3; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		// last stage: just copy
		for(kk=0; kk<nx_v[N_tmp+T1]; kk++)
			hpi[N_tmp+T1-1][kk] = hpi3[ii][kk];
		// middle stages: backward simulation
		for(jj=0; jj<T1-1; jj++)
			{
			for(kk=0; kk<nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj]; kk++)
				pi_work[kk] = hrq[N_tmp+T1-1-jj][kk];
			for(kk=0; kk<nb_v[N_tmp+T1-1-jj]; kk++)
				pi_work[hidxb[N_tmp+T1-1-jj][kk]] += - hlam[N_tmp+T1-1-jj][0*pnb_v[N_tmp+T1-1-jj]+kk] + hlam[N_tmp+T1-1-jj][1*pnb_v[N_tmp+T1-1-jj]+kk];
#ifdef BLASFEO
			dsymv_l_lib(nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], 1.0, hpRSQrq[N_tmp+T1-1-jj], cnux_v[N_tmp+T1-1-jj], hux[N_tmp+T1-1-jj], 1.0, pi_work, pi_work);
			dgemv_n_lib(nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], 1.0, nx_v[N_tmp+T1-jj], hpBAbt[N_tmp+T1-1-jj], cnx_v[N_tmp+T1-jj], hpi[N_tmp+T1-1-jj], 1.0, pi_work, pi_work);
#else
			dsymv_lib(nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], hpRSQrq[N_tmp+T1-1-jj], cnux_v[N_tmp+T1-1-jj], hux[N_tmp+T1-1-jj], 1, pi_work, pi_work);
			dgemv_n_lib(nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], nx_v[N_tmp+T1-jj], hpBAbt[N_tmp+T1-1-jj], cnx_v[N_tmp+T1-jj], hpi[N_tmp+T1-1-jj], 1, pi_work, pi_work);
#endif
			for(kk=0; kk<ng_v[N_tmp+T1-1-jj]; kk++)
				work_tmp[kk] = hlam[N_tmp+T1-1-jj][2*pnb_v[N_tmp+T1-1-jj]+1*png_v[N_tmp+T1-1-jj]+kk] - hlam[N_tmp+T1-1-jj][2*pnb_v[N_tmp+T1-1-jj]+0*png_v[N_tmp+T1-1-jj]+kk];
#ifdef BLASFEO
			dgemv_n_lib(nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], ng_v[N_tmp+T1-1-jj], 1.0, hpDCt[N_tmp+T1-1-jj], cng_v[N_tmp+T1-1-jj], work_tmp, 1.0, pi_work, pi_work);
#else
			dgemv_n_lib(nu_v[N_tmp+T1-1-jj]+nx_v[N_tmp+T1-1-jj], ng_v[N_tmp+T1-1-jj], hpDCt[N_tmp+T1-1-jj], cng_v[N_tmp+T1-1-jj], work_tmp, 1, pi_work, pi_work);
#endif
			//
			for(kk=0; kk<nx_v[N_tmp+T1-1-jj]; kk++)
				hpi[N_tmp+T1-2-jj][kk] = + pi_work[nu_v[N_tmp+T1-1-jj]+kk];
			}
		//
		N_tmp += T1;
		}

	printf("\nux =\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu_v[ii]+nx_v[ii], hux[ii], 1);

	printf("\npi =\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx_v[ii+1], hpi[ii], 1);

	printf("\nlam =\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], hlam[ii], 1);

	printf("\nt =\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, 2*pnb_v[ii]+2*png_v[ii], ht[ii], 1);

	// print timings
	printf("time ipm full      (N = %3d) = %5.2e\n", N, time_ipm_full);
	printf("time ipm cond      (N = %3d) = %5.2e\n", 1, time_ipm_cond);
	printf("time ipm part cond (N = %3d) = %5.2e\n\n", N3, time_ipm_part_cond);

/************************************************
* free memory
************************************************/	
	
#if MHE!=1
	free(pBAbt0);
	free(b0);
	free(pA);
	free(pRSQrq0);
	free(d0);
	free(idxb0);
#endif
	if(N>1) free(pBAbt1);
	if(N>1) free(pRSQrq1);
	if(N>1) free(d1);
	if(N>1) free(idxb1);
	free(pRSQrqN);
	free(dN);
	free(idxbN);
	//
	free(pBAbt2);
	free(pRSQrq2);
	free(pDCt2);
	free(d2);
	free(work0);
	free(work1);
	free(work_ipm_full);
	free(memory_ric_cond);
	free(work_ric_cond);
	free(work_ipm_cond);
	free(work_ipm3);
	free(memory_part_cond);
	free(work_part_cond);
	for(ii=0; ii<N; ii++)
		{
	//	free(hpGamma_x0[ii]);
	//	free(hpGamma_u[ii]);
	//	free(hpGamma_b[ii]);
		free(hpGamma[ii]);
		}

/************************************************
* return
************************************************/	

	return 0;

	}
