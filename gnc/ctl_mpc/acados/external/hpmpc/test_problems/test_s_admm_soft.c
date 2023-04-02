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

#include "../include/aux_d.h"
#include "../include/aux_s.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../problem_size.h"
#include "../include/block_size.h"
#include "tools.h"
#include "test_param.h"



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
	printf(" Copyright (C) 2014 by Technical University of Denmark. All rights reserved.\n");
	printf("\n");
	printf(" HPMPC is distributed in the hope that it will be useful,\n");
	printf(" but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
	printf(" MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n");
	printf(" See the GNU Lesser General Public License for more details.\n");
	printf("\n");
	printf("\n");
	printf("\n");

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
/*	printf("\nflush subnormals to zero\n");*/
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); // flush to zero subnormals !!! works only with one thread !!!
#endif

	int ii, jj, idx;
	
	int rep, nrep=NREP;

	int nx = NX; // number of states (it has to be even for the mass-spring system test problem)
	int nu = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = NN; // horizon lenght
	int nb = NB; // number of box constrained inputs and states

	printf(" Test problem: mass-spring system with %d masses and %d controls.\n", nx/2, nu);
	printf("\n");
	printf(" MPC problem size: %d states, %d inputs, %d horizon length, %d two-sided box constraints.\n", nx, nu, N, nb);
	printf("\n");
	printf(" ADMM method parameters: single precision, %d maximum iterations, %5.1e exit tolerance in primal and duality measure (edit file test_admm_ip_box.c to change them).\n", K_MAX_ADMM, TOL);

	int info = 0;
		
	const int bs = S_MR; //d_get_mr();
	const int ncl = S_NCL;
	const int nal = bs*ncl; // number of doubles per cache line
	
	const int nz = nx+nu+1;
	const int pnz = bs*((nz+bs-1)/bs);
	const int pnx = bs*((nx+bs-1)/bs);
	const int cnz = ncl*((nx+nu+1+ncl-1)/ncl);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int pnb = bs*((2*nb+bs-1)/bs); // packed number of box constraints
	const int anz = nal*((nz+nal-1)/nal);
	const int anx = nal*((nx+nal-1)/nal);
	const int anb = nal*((2*nb+nal-1)/nal); // cache aligned number of box constraints

	const int pad = (ncl-nx%ncl)%ncl; // packing between BAbtL & P
	const int cnl = cnz<cnx+ncl ? nx+pad+cnx+ncl : nx+pad+cnz;
	
/************************************************
* dynamical system
************************************************/	

	double *A; d_zeros(&A, nx, nx); // states update matrix

	double *B; d_zeros(&B, nx, nu); // inputs matrix

	double *b; d_zeros(&b, nx, 1); // states offset
	double *x0; d_zeros(&x0, nx, 1); // initial state

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx, nu, N, A, B, b, x0);
	
	for(jj=0; jj<nx; jj++)
		b[jj] = 0.0;
	
	for(jj=0; jj<nx; jj++)
		x0[jj] = 0;
	x0[0] = 3.5;
	x0[1] = 3.5;
	
//	d_print_mat(nx, nx, A, nx);
//	d_print_mat(nx, nu, B, nx);
//	d_print_mat(nx, 1, b, nx);
//	d_print_mat(nx, 1, x0, nx);
	
	/* packed */
/*	double *BAb; d_zeros(&BAb, nx, nz);*/

/*	dmcopy(nx, nu, B, nx, BAb, nx);*/
/*	dmcopy(nx, nx, A, nx, BAb+nu*nx, nx);*/
/*	dmcopy(nx, 1 , b, nx, BAb+(nu+nx)*nx, nx);*/
	
	/* transposed */
/*	double *BAbt; d_zeros_align(&BAbt, pnz, pnz);*/
/*	for(ii=0; ii<nx; ii++)*/
/*		for(jj=0; jj<nz; jj++)*/
/*			{*/
/*			BAbt[jj+pnz*ii] = BAb[ii+nx*jj];*/
/*			}*/

	/* packed into contiguous memory */
	float *pBAbt; s_zeros_align(&pBAbt, pnz, cnx);
/*	d_cvt_mat2pmat(nz, nx, 0, bs, BAbt, pnz, pBAbt, cnx);*/
/*	d_cvt_tran_mat2pmat(nx, nz, 0, bs, BAb, nx, pBAbt, cnx);*/

	cvt_tran_d2s_mat2pmat(nx, nu, 0, bs, B, nx, pBAbt, cnx);
	cvt_tran_d2s_mat2pmat(nx, nx, nu, bs, A, nx, pBAbt+nu/bs*cnx*bs+nu%bs, cnx);
	for (jj = 0; jj<nx; jj++)
		pBAbt[(nx+nu)/bs*cnx*bs+(nx+nu)%bs+jj*bs] = (float) b[jj];

/*	s_print_pmat (nz, nx, bs, pBAbt, cnx);*/
/*	exit(1);*/

/************************************************
* box constraints
************************************************/	

/*	double *db; d_zeros_align(&db, 2*nb, 1);*/
/*	for(jj=0; jj<2*nu; jj++)*/
/*		db[jj] = - 0.5;   // umin*/
/*	for(; jj<2*nb; jj++)*/
/*		db[jj] = - 4.0;   // xmin*/

	float *lb; s_zeros_align(&lb, nx+nu, 1);
	for(jj=0; jj<nu; jj++)
		lb[jj] = - 0.5;   // umin
	for(; jj<nu+nx; jj++)
		lb[jj] = - 1.0;   // xmin

	float *ub; s_zeros_align(&ub, nx+nu, 1);
	for(jj=0; jj<nu; jj++)
		ub[jj] = 0.5;   // uman
	for(; jj<nu+nx; jj++)
		ub[jj] = 1.0;   // xman

/************************************************
* cost function
************************************************/	

	float *Q; s_zeros_align(&Q, pnz, pnz);
	for(ii=0; ii<nu; ii++) Q[ii*(pnz+1)] = 2.0;
	for(; ii<pnz; ii++) Q[ii*(pnz+1)] = 0.0;
	for(ii=0; ii<nz; ii++) Q[nx+nu+ii*pnz] = 0.0;
/*	Q[(nx+nu)*(pnz+1)] = 1e35; // large enough (not needed any longer) */
	
	/* packed into contiguous memory */
	float *pQ; s_zeros_align(&pQ, pnz, cnz);
	s_cvt_mat2pmat(nz, nz, Q, pnz, 0, pQ, cnz);

/************************************************
* cost function of soft constraints
************************************************/	

	float *Z; s_zeros_align(&Z, 2*anx, 1);
	for(ii=0; ii<nx; ii++) Z[ii] = 0.0; // upper
	for(ii=0; ii<nx; ii++) Z[anx+ii] = 0.0; // lower

	float *z; s_zeros_align(&z, 2*anx, 1);
	for(ii=0; ii<nx; ii++) z[ii] = 10.0; // upper
	for(ii=0; ii<nx; ii++) z[anx+ii] = 10.0; // lower

/************************************************
* matrices series
************************************************/	

	float *hpQ[N+1];
	float *hq[N+1];
	float *hux[N+1];
	float *hpi[N+1];
	float *hlam[N+1];
	float *ht[N+1];
	float *hpBAbt[N];
	float *hlb[N+1];
	float *hub[N+1];
	float *hrb[N];
	float *hrq[N+1];
	float *hrd[N+1];
	float *hZ[N+1];
	float *hz[N+1];
	float *hux_v[N+1];
	float *hux_w[N+1];
	float *hs_u[N+1];
	float *hs_v[N+1];
	float *hs_w[N+1];

	for(jj=0; jj<N; jj++)
		{
		s_zeros_align(&hpQ[jj], pnz, cnz);
		//hpQ[jj] = pQ;
		s_zeros_align(&hq[jj], anz, 1);
		s_zeros_align(&hux[jj], anz, 1);
		s_zeros_align(&hpi[jj], anx, 1);
		s_zeros_align(&hlam[jj],anb, 1); // TODO pnb
		s_zeros_align(&ht[jj], anb, 1); // TODO pnb
		hpBAbt[jj] = pBAbt;
		hlb[jj] = lb;
		hub[jj] = ub;
		s_zeros_align(&hrb[jj], anx, 1);
		s_zeros_align(&hrq[jj], anz, 1);
		s_zeros_align(&hrd[jj], anb, 1); // TODO pnb
		hZ[jj] = Z;
		hz[jj] = z;
		s_zeros_align(&hux_v[jj], anz, 1);
		s_zeros_align(&hux_w[jj], anz, 1);
		s_zeros_align(&hs_u[jj], 2*anx, 1);
		s_zeros_align(&hs_v[jj], 2*anx, 1);
		s_zeros_align(&hs_w[jj], 2*anx, 1);
		}
	s_zeros_align(&hpQ[N], pnz, pnz);
	//hpQ[N] = pQ;
	s_zeros_align(&hq[N], anz, 1);
	s_zeros_align(&hux[N], anz, 1);
	s_zeros_align(&hpi[N], anx, 1);
	s_zeros_align(&hlam[N], anb, 1); // TODO pnb
	s_zeros_align(&ht[N], anb, 1); // TODO pnb
	hlb[N] = lb;
	hub[N] = ub;
	s_zeros_align(&hrq[N], anz, 1);
	s_zeros_align(&hrd[N], anb, 1); // TODO pnb
	hZ[N] = Z;
	hz[N] = z;
	s_zeros_align(&hux_v[N], anz, 1);
	s_zeros_align(&hux_w[N], anz, 1);
	s_zeros_align(&hs_u[N], 2*anx, 1);
	s_zeros_align(&hs_v[N], 2*anx, 1);
	s_zeros_align(&hs_w[N], 2*anx, 1);
	
	// starting guess
//	for(jj=0; jj<nx; jj++) hux[0][nu+jj]=x0[jj];

/************************************************
* riccati-like iteration
************************************************/

	float *work; s_zeros_align(&work, (N+1)*(pnz*cnl + 5*anz + 6*anx) + 3*anz, 1); // work space
	int kk = 0; // acutal number of iterations
/*	char prec = PREC; // double/single precision*/
/*	float sp_thr = SP_THR; // threshold to switch between double and single precision*/
	int k_max = K_MAX_ADMM; // maximum number of iterations in the ADMM method
	float tol = TOL*sqrt(N*(nx+nu));//TOL; // tolerance in the duality measure
/*	float sigma[] = {0.4, 0.3, 0.01}; // control primal-dual IP behaviour*/
	float rho = 10.0; // penalty parameter
	float alpha = 1.9; // relaxation parameter
	float *stat; s_zeros(&stat, 5, k_max); // stats from the ADMM routine
	int compute_mult = COMPUTE_MULT_ADMM;
	int warm_start = WARM_START;
/*	float mu = -1.0;*/
	


	/* initizile the cost function */
	for(ii=0; ii<N; ii++)
		{
		for(jj=0; jj<pnz*cnz; jj++) hpQ[ii][jj]=pQ[jj];
		}
	for(jj=0; jj<pnz*cnz; jj++) hpQ[N][jj]=pQ[jj];



	// initial states
	float xx0[] = {3.5, 3.5, 3.66465, 2.15833, 1.81327, -0.94207, 1.86531, -2.35760, 2.91534, 1.79890, -1.49600, -0.76600, -2.60268, 1.92456, 1.66630, -2.28522, 3.12038, 1.83830, 1.93519, -1.87113};



	/* warm up */

	// initialize states and inputs
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<nx+nu; jj++)
			hux[ii][jj] = 0;

	hux[0][nu+0] = xx0[0];
	hux[0][nu+1] = xx0[1];

	// call the ADMM solver
//	if(FREE_X0==0)
//		{
		s_admm_soft_mpc(&kk, k_max, tol, tol, warm_start, 1, rho, alpha, stat, nx, nu, N, hpBAbt, hpQ, hZ, hz, hlb, hub, hux, hux_v, hux_w, hs_u, hs_v, hs_w, compute_mult, hpi, work);
//		}
//	else
//		{
///*		d_ip_box_mhe(&kk, k_max, tol, warm_start, sigma, stat, nx, nu, N, nb, hpBAbt, hpQ, hdb, hux, compute_mult, hpi, hlam, ht, work);*/
//		}


	int kk_avg = 0;

	/* timing */
	struct timeval tv0, tv1;
	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{

		idx = rep%10;
		x0[0] = xx0[2*idx];
		x0[1] = xx0[2*idx+1];

		// initialize states and inputs
		for(ii=0; ii<=N; ii++)
			for(jj=0; jj<nx+nu; jj++)
				hux[ii][jj] = 0;

		hux[0][nu+0] = xx0[2*idx];
		hux[0][nu+1] = xx0[2*idx+1];

		// call the ADMM solver
//		if(FREE_X0==0)
//			{
			s_admm_soft_mpc(&kk, k_max, tol, tol, warm_start, 0, rho, alpha, stat, nx, nu, N, hpBAbt, hpQ, hZ, hz, hlb, hub, hux, hux_v, hux_w, hs_u, hs_v, hs_w, compute_mult, hpi, work);
//			}
//		else
//			{
///*			d_ip_box_mhe(&kk, k_max, tol, warm_start, sigma, stat, nx, nu, N, nb, hpBAbt, hpQ, hdb, hux, compute_mult, hpi, hlam, ht, work);*/
//			}

		kk_avg += kk;

		}
	
	gettimeofday(&tv1, NULL); // stop
	


	float time = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	
/*	printf("\nnx\tnu\tN\tkernel\n\n");*/
/*	printf("\n%d\t%d\t%d\t%e\n\n", nx, nu, N, time);*/
	
	printf("\n");
	printf(" Average number of iterations over %d runs: %5.1f\n", nrep, kk_avg / (float) nrep);
/*	printf(" Average number of iterations over %d runs: %d\n", nrep, kk);*/
	printf("\n");
	printf(" Average solution time over %d runs: %5.2e seconds\n", nrep, time);
	printf("\n");



	// restore linear part of cost function 
	for(ii=0; ii<N; ii++)
		{
		for(jj=0; jj<nx+nu; jj++) hq[ii][jj] = Q[nx+nu+pnz*jj];
		}
	for(jj=0; jj<nx+nu; jj++) hq[N][jj] = Q[nx+nu+pnz*jj];

	// residuals computation
/*	if(FREE_X0==0)*/
/*		d_res_ip_box_mpc(nx, nu, N, nb, hpBAbt, hpQ, hq, hux, hdb, hpi, hlam, ht, hrq, hrb, hrd, &mu);*/
/*	else*/
/*		d_res_ip_box_mhe(nx, nu, N, nb, hpBAbt, hpQ, hq, hux, hdb, hpi, hlam, ht, hrq, hrb, hrd, &mu);*/


	if(PRINTSTAT==1)
		{

		printf("\n");
		printf("\n");
		printf(" Print ADMM statistics of the last run\n");
		printf("\n");

		for(jj=0; jj<kk; jj++)
			printf("k = %d\t\tp_res = %f\td_res = %f\n", jj, stat[5*jj], stat[5*jj+1]);
		printf("\n");
		
		}

	if(PRINTRES==1)
		{

		printf("\n");
		printf("\n");
		printf(" Print solution\n");
		printf("\n");

		printf("\nu = \n\n");
		for(ii=0; ii<N; ii++)
			s_print_mat(1, nu, hux[ii], 1);
		
		printf("\nx = \n\n");
		for(ii=0; ii<=N; ii++)
			s_print_mat(1, nx, hux[ii]+nu, 1);

		}

	if(0 && PRINTRES==1 && COMPUTE_MULT_ADMM==1)
		{
		// print result 
		// print result 
		printf("\n");
		printf("\n");
		printf(" Print residuals\n\n");
		printf("\n");
		printf("\n");
		printf("rq = \n\n");
//		if(FREE_X0==0)
//			{
			s_print_mat(1, nu, hrq[0], 1);
			for(ii=1; ii<=N; ii++)
/*				s_print_mat_e(1, nx+nu, hrq[ii], 1);*/
				s_print_mat(1, nx+nu, hrq[ii], 1);
//			}
//		else
//			{
//			for(ii=0; ii<=N; ii++)
///*				s_print_mat_e(1, nx+nu, hrq[ii], 1);*/
//				s_print_mat(1, nx+nu, hrq[ii], 1);
//			}
		printf("\n");
		printf("\n");
		printf("rb = \n\n");
		for(ii=0; ii<N; ii++)
/*			s_print_mat_e(1, nx, hrb[ii], 1);*/
			s_print_mat(1, nx, hrb[ii], 1);
		printf("\n");
		printf("\n");
		printf("rd = \n\n");
		for(ii=0; ii<=N; ii++)
/*			s_print_mat_e(1, 2*nb, hrd[ii], 1);*/
			s_print_mat(1, 2*nb, hrd[ii], 1);
		printf("\n");
		printf("\n");
/*		printf("mu = %e\n\n", mu);*/
		
		}

/*	printf("\nnx\tnu\tN\tkernel\n\n");*/
/*	printf("\n%d\t%d\t%d\t%e\n\n", nx, nu, N, time);*/
	
/************************************************
* free memory and return
************************************************/

	free(A);
	free(B);
	free(b);
	free(x0);
/*	free(BAb);*/
/*	free(BAbt);*/
	free(pBAbt);
	free(lb);
	free(ub);
	free(Q);
	free(pQ);
	free(Z);
	free(z);
	free(work);
	free(stat);
	for(jj=0; jj<N; jj++)
		{
		free(hpQ[jj]);
		free(hq[jj]);
		free(hux[jj]);
		free(hpi[jj]);
		free(hlam[jj]);
		free(ht[jj]);
		free(hrb[jj]);
		free(hrq[jj]);
		free(hrd[jj]);
		free(hux_v[jj]);
		free(hux_w[jj]);
		free(hs_u[jj]);
		free(hs_v[jj]);
		free(hs_w[jj]);
		}
	free(hpQ[N]);
	free(hq[N]);
	free(hux[N]);
	free(hpi[N]);
	free(hlam[N]);
	free(ht[N]);
	free(hrq[N]);
	free(hrd[N]);
	free(hux_v[N]);
	free(hux_w[N]);
	free(hs_u[N]);
	free(hs_v[N]);
	free(hs_w[N]);



	return 0;

	}



