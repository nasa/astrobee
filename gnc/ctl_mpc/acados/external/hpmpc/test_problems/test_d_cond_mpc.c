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
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
#include <xmmintrin.h> // needed to flush to zero sub-normals with _MM_SET_FLUSH_ZERO_MODE (_MM_FLUSH_ZERO_ON); in the main()
#endif

// to throw floating-point exception
/*#define _GNU_SOURCE*/
/*#include <fenv.h>*/

#include "test_param.h"
#include "../problem_size.h"
#include "../include/aux_d.h"
#include "../include/blas_d.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../include/block_size.h"
#include "tools.h"



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


#if 1 // build & debug

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

	printf("Riccati solver performance test - double precision\n");
	printf("\n");

	// maximum frequency of the processor
	const float GHz_max = GHZ_MAX;
	printf("Frequency used to compute theoretical peak: %5.1f GHz (edit test_param.h to modify this value).\n", GHz_max);
	printf("\n");

	// maximum flops per cycle, double precision
#if defined(TARGET_X64_AVX2)
	const float flops_max = 16;
	printf("Testing solvers for AVX & FMA3 instruction sets, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_AVX)
	const float flops_max = 8;
	printf("Testing solvers for AVX instruction set, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_SSE3) || defined(TARGET_AMD_SSE3)
	const float flops_max = 4;
	printf("Testing solvers for SSE3 instruction set, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_CORTEX_A15)
	const float flops_max = 2;
	printf("Testing solvers for ARMv7a VFPv3 instruction set, oprimized for Cortex A15: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_CORTEX_A9)
	const float flops_max = 1;
	printf("Testing solvers for ARMv7a VFPv3 instruction set, oprimized for Cortex A9: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_CORTEX_A7)
	const float flops_max = 0.5;
	printf("Testing solvers for ARMv7a VFPv3 instruction set, oprimized for Cortex A7: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_ATOM)
	const float flops_max = 1;
	printf("Testing solvers for SSE3 instruction set, 32 bit, optimized for Intel Atom: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_POWERPC_G2)
	const float flops_max = 1;
	printf("Testing solvers for POWERPC instruction set, 32 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_4X4)
	const float flops_max = 2;
	printf("Testing reference solvers, 4x4 kernel: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_4X4_PREFETCH)
	const float flops_max = 2;
	printf("Testing reference solvers, 4x4 kernel with register prefetch: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_2X2)
	const float flops_max = 2;
	printf("Testing reference solvers, 2x2 kernel: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#endif
	
	FILE *f;
	f = fopen("./test_problems/results/test_blas.m", "w"); // a

#if defined(TARGET_X64_AVX2)
	fprintf(f, "C = 'd_x64_avx2';\n");
	fprintf(f, "\n");
#elif defined(TARGET_X64_AVX)
	fprintf(f, "C = 'd_x64_avx';\n");
	fprintf(f, "\n");
#elif defined(TARGET_X64_SSE3) || defined(TARGET_AMD_SSE3)
	fprintf(f, "C = 'd_x64_sse3';\n");
	fprintf(f, "\n");
#elif defined(TARGET_CORTEX_A9)
	fprintf(f, "C = 'd_ARM_cortex_A9';\n");
	fprintf(f, "\n");
#elif defined(TARGET_CORTEX_A7)
	fprintf(f, "C = 'd_ARM_cortex_A7';\n");
	fprintf(f, "\n");
#elif defined(TARGET_CORTEX_A15)
	fprintf(f, "C = 'd_ARM_cortex_A15';\n");
	fprintf(f, "\n");
#elif defined(TARGET_X86_ATOM)
	fprintf(f, "C = 'd_x86_atom';\n");
	fprintf(f, "\n");
#elif defined(TARGET_POWERPC_G2)
	fprintf(f, "C = 'd_PowerPC_G2';\n");
	fprintf(f, "\n");
#elif defined(TARGET_C99_4X4)
	fprintf(f, "C = 'd_c99_4x4';\n");
	fprintf(f, "\n");
#elif defined(TARGET_C99_4X4_PREFETCH)
	fprintf(f, "C = 'd_c99_4x4';\n");
	fprintf(f, "\n");
#elif defined(TARGET_C99_2X2)
	fprintf(f, "C = 'd_c99_2x2';\n");
	fprintf(f, "\n");
#endif

	fprintf(f, "A = [%f %f];\n", GHz_max, flops_max);
	fprintf(f, "\n");

	fprintf(f, "B = [\n");
	

	const int LTI = 1;

	printf("\n");
	printf("Tested solvers:\n");
	printf("-sv : Riccati factorization and system solution (prediction step in IP methods)\n");
	printf("-trs: system solution after a previous call to Riccati factorization (correction step in IP methods)\n");
	printf("\n");
	if(LTI==1)
		printf("\nTest for linear time-invariant systems\n");
	else
		printf("\nTest for linear time-variant systems\n");
	printf("\n");
	
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
/*	printf("\nflush to zero on\n");*/
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); // flush to zero subnormals !!! works only with one thread !!!
#endif

	// to throw floating-point exception
/*#ifndef __APPLE__*/
/*    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);*/
/*#endif*/
	
	int ii, jj;

	double **dummy;

	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line
	
	int nn[] = {4, 6, 8, 10, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 4000, 4000, 2000, 2000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	
	int vnx[] = {8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 512, 1024};
	int vnrep[] = {100, 100, 100, 100, 100, 100, 50, 50, 50, 20, 10, 10};
	int vN[] = {4, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256};

	int nx, nu, N, nrep;

	int *nx_v, *nu_v, *nb_v, *ng_v;

	int ll;
//	int ll_max = 77;
	int ll_max = 1;
	for(ll=0; ll<ll_max; ll++)
		{
		

		if(ll_max==1)
			{
			nx = NX; // number of states (it has to be even for the mass-spring system test problem)
			nu = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = NN; // horizon lenght
			nrep = NREP;
			//nx = 25;
			//nu = 1;
			//N = 11;
			}
		else
			{
			nx = nn[ll]; // number of states (it has to be even for the mass-spring system test problem)
			nu = 2; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = 10; // horizon lenght
			nrep = 2*nnrep[ll];
			}

		int rep;
	
		int nz = nx+nu+1;
		int anz = (nz+nal-1)/nal*nal;
		int anx = (nx+nal-1)/nal*nal;
		int pnz = (nz+bs-1)/bs*bs;
		int pnx = (nx+bs-1)/bs*bs;
		int pnu = (nu+bs-1)/bs*bs;
		int pNnu = (N*nu+bs-1)/bs*bs;
		int cnz = (nx+nu+1+ncl-1)/ncl*ncl;
		int cnx = (nx+ncl-1)/ncl*ncl;
		int cnu = (nu+ncl-1)/ncl*ncl;
		//int cNnu = ((N-1)*nu+cnu+ncl-1)/ncl*ncl;
		int cNnu = (N*nu+ncl-1)/ncl*ncl;
		int cNnx = (N*nx+ncl-1)/ncl*ncl;
		int cnl = cnz>cnx+ncl ? pnz : cnx+ncl;

		int ny = nu+nx;
		int pny = (ny+bs-1)/bs*bs;
		int cny = (ny+ncl-1)/ncl*ncl;
		int cnk = cnx+ncl>cny ? cnx+ncl : cny;


/************************************************
* dynamical system
************************************************/	

		double *A; d_zeros(&A, nx, nx); // states update matrix

		double *B; d_zeros(&B, nx, nu); // inputs matrix

		double *b; d_zeros_align(&b, pnx, 1); // states offset
		double *x0; d_zeros_align(&x0, pnx, 1); // initial state

		double Ts = 0.5; // sampling time
		mass_spring_system(Ts, nx, nu, N, A, B, b, x0);
	
		for(jj=0; jj<nx; jj++)
			b[jj] = 0.1;
	
		for(jj=0; jj<nx; jj++)
			x0[jj] = 0;
		x0[0] = 3.5;
		x0[1] = 3.5;

		d_print_mat(nx, nx, A, nx);
		d_print_mat(nx, nu, B, nx);
		d_print_mat(nx, 1, b, nx);
//		d_print_mat(nx, 1, x0, nx);
//		exit(1);

		double *pA; d_zeros_align(&pA, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);
		//d_print_pmat(nx, nx, bs, pA, cnx);

		double *pAt; d_zeros_align(&pAt, pnx, cnx);
		d_cvt_tran_mat2pmat(nx, nx, A, nx, 0, pAt, cnx);
		//d_print_pmat(nx, nx, bs, pA, cnx);

		double *pB; d_zeros_align(&pB, pnx, cnu);
		d_cvt_mat2pmat(nx, nu, B, nx, 0, pB, cnu);
		//d_print_pmat(nx, nu, bs, pB, cnu);

		double *pBt; d_zeros_align(&pBt, pnu, cnx);
		d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBt, cnx);
		//d_print_pmat(nu, nx, bs, pBt, cnx);

		double *b0; d_zeros_align(&b0, pnx, 1);
		dgemv_n_lib(nx, nx, pA, cnx, x0, b, b0, 1);

		double *pBAbt; d_zeros_align(&pBAbt, pnz, cnx);
		d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBAbt, cnx);
		d_cvt_tran_mat2pmat(nx, nx, A, nx, nu, pBAbt+nu/bs*bs*cnx+nu%bs, cnx);
		d_cvt_tran_mat2pmat(nx, 1, b, nx, nu+nx, pBAbt+(nu+nx)/bs*bs*cnx+(nu+nx)%bs, cnx);
		//d_print_pmat(nx+nu+1, nx, bs, pBAbt, cnx);
		//exit(1);
	
		double *pBAt; d_zeros_align(&pBAt, pnz, cnx);
		dgecp_lib(nu, nx, 0, pBt, cnx, 0, pBAt, cnx);
		dgecp_lib(nx, nx, 0, pAt, cnx, nu, pBAt+nu/bs*bs*cnx+nu%bs, cnx);
		//d_print_pmat(nz, nx, bs, pBAt, cnx);

/************************************************
* cost function
************************************************/

		double *Q; d_zeros(&Q, nx, nx);
		for(ii=0; ii<nx; ii++)
			Q[ii*(nx+1)] = 3.0;
		//d_print_mat(nx, nx, Q, nx);

		double *R; d_zeros(&R, nu, nu);
		for(ii=0; ii<nu; ii++)
			R[ii*(nu+1)] = 2.0;
		//d_print_mat(nu, nu, R, nu);

		double *S; d_zeros(&S, nu, nx);
		for(ii=0; ii<nu; ii++)
			S[ii*(nu+1)] = 0.0;
		//d_print_mat(nu, nx, S, nu);

		double *q; d_zeros_align(&q, pnx, 1);
		for(ii=0; ii<nx; ii++)
			q[ii] = 0.1;

		double *r; d_zeros_align(&r, pnu, 1);
		for(ii=0; ii<nu; ii++)
			r[ii] = 0.1;

		double *pQ; d_zeros_align(&pQ, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, Q, nx, 0, pQ, cnx);
		//d_print_pmat(nx, nx, bs, pQ, cnx);

		double *dQ; d_zeros_align(&dQ, pnx, 1);
		for(ii=0; ii<nx; ii++) dQ[ii] = Q[ii*(nx+1)];
		//d_print_mat(1, nx, dQ, 1);

		double *pR; d_zeros_align(&pR, pnu, cnu);
		d_cvt_mat2pmat(nu, nu, R, nu, 0, pR, cnu);
		//d_print_pmat(nu, nu, bs, pR, cnu);

		double *dR; d_zeros_align(&dR, pnu, 1);
		for(ii=0; ii<nu; ii++) dR[ii] = R[ii*(nu+1)];
		//d_print_mat(1, nu, dR, 1);

		double *dRSQ; d_zeros_align(&dRSQ, pnu+pnx, 1);
		for(ii=0; ii<nu; ii++) dRSQ[ii] = R[ii*(nu+1)];
		for(ii=0; ii<nx; ii++) dRSQ[nu+ii] = Q[ii*(nx+1)];
		//d_print_mat(1, nu, dR, 1);

		double *pS; d_zeros_align(&pS, pnu, cnx); // TODO change definition to transposed !!!!!!!!!!
		d_cvt_mat2pmat(nu, nx, S, nu, 0, pS, cnx);
		//d_print_pmat(nu, nx, bs, pS, cnx);

		double *pRSQrq; d_zeros_align(&pRSQrq, pnz, cnz);
		d_cvt_mat2pmat(nu, nu, R, nu, 0, pRSQrq, cnz);
		d_cvt_tran_mat2pmat(nu, nx, S, nu, nu, pRSQrq+nu/bs*bs*cnz+nu%bs, cnz);
		d_cvt_mat2pmat(nx, nx, Q, nx, nu, pRSQrq+nu/bs*bs*cnz+nu%bs+nu*bs, cnz);
		d_cvt_tran_mat2pmat(nu, 1, r, nu, nu+nx, pRSQrq+(nu+nx)/bs*bs*cnz+(nu+nx)%bs, cnz);
		d_cvt_tran_mat2pmat(nx, 1, q, nx, nu+nx, pRSQrq+(nu+nx)/bs*bs*cnz+(nu+nx)%bs+nu*bs, cnz);
		//d_print_pmat(nz, nz, bs, pRSQrq, cnz);
		//exit(1);

		double *pRSQ; d_zeros_align(&pRSQ, pny, cny);
		dgecp_lib(nu, nu, 0, pR, cnu, 0, pRSQ, cny);
		dgetr_lib(nu, nx, 0, pS, cnx, nu, pRSQ+nu/bs*bs*cny+nu%bs, cny);
		dgecp_lib(nx, nx, 0, pQ, cnx, nu, pRSQ+nu/bs*bs*cny+nu%bs+nu*bs, cny);
//		d_print_pmat(nz, nz, bs, pRSQ, cny);
//		exit(1);

		double *rq; d_zeros_align(&rq, pny, 1);
		for(ii=0; ii<nu; ii++) rq[ii] = r[ii];
		for(ii=0; ii<nx; ii++) rq[nu+ii] = q[ii];

/************************************************
* matrix series
************************************************/

		int N2 = 5;
		int N1 = (N+N2-1)/N2;
		int cN1nu = (N1*nu+ncl-1)/ncl*ncl;
		int cN1nx = (N1*nx+ncl-1)/ncl*ncl;

		double *hpA[N];
		double *hpAt[N];
		double *hpBt[N];
		double *hb[N];
		double *hpBAbt[N];
		double *hpGamma_u[N];
		double *hpGamma_u_Q[N];
		double *hpGamma_u_Q_A[N];
		double *hpGamma_0[N];
		double *hpGamma_0_Q[N];
		double *hGamma_b[N];
		double *hGamma_b_q[N];
		double *hpQ[N+1];
		double *hdQ[N+1];
		double *hpR[N];
		double *hdR[N];
		double *hpS[N];
		double *hq[N+1];
		double *hr[N];
		double *hpRSQrq[N+1];
		double *hrq[N+1];
		double *hpL[N+1];
		double *hx[N+1];
		double *hu[N];
		double *hpL2[N+1];
		double *hux[N+1];
		double *pH_A[N2];
		double *pH_B[N2];
		double *H_b[N2];
		double *pH_R[N2];
		double *pH_Q[N2+1];
		double *pH_St[N2];
		double *H_q[N2+1];
		double *H_r[N2];
		double *pL_R;
		double *H_u;
		double *work;
		//double *pGamma_u;
		//double *pGamma_u_Q;
		//double *pGamma_u_Q_A;
		//double *pGamma_0;
		//double *pGamma_0_Q;
		double *diag; d_zeros_align(&diag, cNnu, 1);
		double *diag_ric; d_zeros_align(&diag_ric, nz, 1);
		double *diag2; d_zeros_align(&diag2, pnz, 1);
		double *work2; d_zeros_align(&work2, pnz, cnx);
		double *pD; d_zeros_align(&pD, pnu+pnx, cnu);
		double *pM; d_zeros_align(&pM, pnu, cnx);
		double *pQs; d_zeros_align(&pQs, pnx, cnx);
		double *pLam; d_zeros_align(&pLam, pny, cny);
		double *hpBAt[N];
		double *hpRSQ[N+1];
		double *hdRSQ[N+1];
		double *pL; d_zeros_align(&pL, pnz, cnl);
		double *pBAtL; d_zeros_align(&pBAtL, pnz, cnx);
		double *pBAbtL; d_zeros_align(&pBAbtL, pnz, cnx);

		for(ii=0; ii<N2; ii++)
			{
			d_zeros_align(&pH_A[ii], pnx, cnx);
			d_zeros_align(&pH_B[ii], pnx, cNnu);
			d_zeros_align(&H_b[ii], pnx, 1);
			d_zeros_align(&pH_R[ii], pNnu, cNnu);
			d_zeros_align(&pH_Q[ii], pnx, cnx);
			d_zeros_align(&pH_St[ii], pnx, cNnu);
			d_zeros_align(&H_q[ii], pnx, 1);
			d_zeros_align(&H_r[ii], pNnu, 1);
			}
		pH_Q[N2] = pQ;
		H_q[N2] = q;

		d_zeros_align(&pL_R, pNnu, cNnu);
		d_zeros_align(&H_u, pNnu, 1);
		d_zeros_align(&work, pnx, 1);
		//d_zeros_align(&pGamma_u, pNnu, cNnx);
		//d_zeros_align(&pGamma_u_Q, pNnu, cNnx);
		//d_zeros_align(&pGamma_u_Q_A, pNnu, cNnx);
		//d_zeros_align(&pGamma_0, pnx, cNnx);
		//d_zeros_align(&pGamma_0_Q, pnx, cNnx);
		for(ii=0; ii<N; ii++)
			{
			hpA[ii] = pA;
			hpAt[ii] = pAt;
			hpBt[ii] = pBt;
			hb[ii] = b;
			hpBAbt[ii] = pBAbt;
			hpBAt[ii] = pBAt;
			d_zeros_align(&hpGamma_u[ii], ((ii+1)*nu+bs-1)/bs*bs, cnx);
			d_zeros_align(&hpGamma_u_Q[ii], ((ii+2)*nu+bs-1)/bs*bs, cnx); //((ii+1)*nu+bs-1)/bs*bs, cnx);
			d_zeros_align(&hpGamma_u_Q_A[ii], ((ii+2)*nu+bs-1)/bs*bs, cnx); //((ii+1)*nu+bs-1)/bs*bs, cnx);
			//hpGamma_u[ii] = pGamma_u+ii*nx*bs;
			//hpGamma_u_Q[ii] = pGamma_u_Q+ii*nx*bs;
			//hpGamma_u_Q_A[ii] = pGamma_u_Q_A+ii*nx*bs;
			d_zeros_align(&hpGamma_0[ii], pnx, cnx);
			d_zeros_align(&hpGamma_0_Q[ii], pnx, cnx);
			//hpGamma_0[ii] = pGamma_0+ii*nx*bs;
			//hpGamma_0_Q[ii] = pGamma_0_Q+ii*nx*bs;
			d_zeros_align(&hGamma_b[ii], pnx, 1);
			d_zeros_align(&hGamma_b_q[ii], pnx, 1);
			hpQ[ii] = pQ;
			hdQ[ii] = dQ;
			hpR[ii] = pR;
			hdR[ii] = dR;
			hpS[ii] = pS;
			hq[ii] = q;
			hr[ii] = r;
			hpRSQrq[ii] = pRSQrq;
			hrq[ii] = rq;
			hpRSQ[ii] = pRSQ;
			hdRSQ[ii] = dRSQ;
			d_zeros_align(&hpL[ii], pnx, cnx);
			d_zeros_align(&hx[ii], pnx, 1);
			d_zeros_align(&hu[ii], pnu, 1);
			d_zeros_align(&hpL2[ii], pnz, cnl);
			d_zeros_align(&hux[ii], pnz, 1);
			}
		hpQ[N] = pQ;
		hdQ[N] = dQ;
		hq[N] = q;
		hpRSQrq[N] = pRSQrq;
		hrq[N] = rq;
		hpRSQ[N] = pRSQ;
		hdRSQ[N] = dRSQ;
		d_zeros_align(&hpL[N], pnx, cnx);;
		d_zeros_align(&hx[N], pnx, 1);
		d_zeros_align(&hpL2[N], pnz, cnl);
		d_zeros_align(&hux[N], pnz, 1);
		hb[0] = b0; // embed x0 !!!!!

/************************************************
* condensing
************************************************/
		
		struct timeval tv0, tv1;

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_Q(N, nx, nu, hpA, 1, 0, hdQ, hpL, 1, hpGamma_0, hpGamma_0_Q, pH_Q[0], work);
			
			d_cond_R(N, nx, nu, 0, hpA, hpAt, hpBt, hpBAt, 1, 1, hdQ, 0, hpL, hpS, hdR, hpRSQ, pD, pM, pQs, pLam, diag_ric, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			//d_cond_St(N, nx, nu, 0, hpS, 1, hpGamma_0, 0, hpGamma_0_Q, hpGamma_u_Q, pH_St[0]);

			//d_cond_q(N, nx, nu, hpA, hb, 1, 1, hdQ, hq, hpGamma_0, 1, hGamma_b, 1, hGamma_b_q, H_q[0]);

			d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]);

			//d_cond_A(N, nx, nu, hpA, 0, hpGamma_0, pH_A[0]);

			//d_cond_B(N, nx, nu, hpA, hpBt, 0, hpGamma_u, pH_B[0]);

			//d_cond_b(N, nx, nu, hpA, hb, 0, hGamma_b, H_b[0]);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 0
//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nx, nx, bs, hpGamma_0[ii], cNnx);

//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nx, nx, bs, hpGamma_0_Q[ii], cNnx);

		printf("\nH_Q\n");
		d_print_pmat(nx, nx, bs, pH_Q[0], cnx);
#endif

#if 0
//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u[ii], cNnx);

//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u_Q[ii], cNnx);

		printf("\nH_R, N^2 n_x^3 algorithm\n");
		d_print_pmat(N*nu, N*nu, bs, pH_R[0], cNnu);
//		exit(1);
#endif

#if 0
		printf("\nH_S\n");
		d_print_pmat(nx, N*nu, bs, pH_St[0], cNnu);
#endif

#if 0
		printf("\nH_q\n");
		d_print_mat(1, nx, H_q[0], 1);
#endif

#if 0
//		printf("\nGamma_b\n");
//		for(ii=0; ii<N; ii++)	
//			d_print_mat(1, nx, hGamma_b[ii], 1);

//		printf("\nGamma_b_q\n");
//		for(ii=0; ii<N; ii++)	
//			d_print_mat(1, nx, hGamma_b_q[ii], 1);

		printf("\nH_r\n");
		d_print_mat(1, N*nu, H_r[0], 1);
//		exit(1);
#endif

#if 0
		printf("\nH_A\n");
		d_print_pmat(nx, nx, bs, pH_A[0], cnx);
#endif

#if 0
		printf("\nH_B\n");
		d_print_pmat(nx, N*nu, bs, pH_B[0], cNnu);
#endif

#if 0
		printf("\nH_b\n");
		d_print_mat(1, nx, H_b[0], 1);
#endif

//exit(1);



		// clear matrix
		for(ii=0; ii<pNnu*cNnu; ii++)
			pH_R[0][ii] = 0;

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_Q(N, nx, nu, hpA, 1, 0, hdQ, hpL, 1, hpGamma_0, hpGamma_0_Q, pH_Q[0], work);
			
			d_cond_R(N, nx, nu, 1, hpA, hpAt, hpBt, hpBAt, 1, 1, hdQ, 0, hpL, hpS, hdR, hpRSQ, pD, pM, pQs, pLam, diag, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			//d_cond_St(N, nx, nu, 0, hpS, 1, hpGamma_0, 0, hpGamma_0_Q, hpGamma_u_Q, pH_St[0]);

			//d_cond_q(N, nx, nu, hpA, hb, 1, 1, hdQ, hq, hpGamma_0, 1, hGamma_b, 1, hGamma_b_q, H_q[0]);

			d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]);

			//d_cond_A(N, nx, nu, hpA, 0, hpGamma_0, pH_A[0]);

			//d_cond_B(N, nx, nu, hpA, hpBt, 0, hpGamma_u, pH_B[0]);

			//d_cond_b(N, nx, nu, hpA, hb, 0, hGamma_b, H_b[0]);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond_N2_nx2 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 0
//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u[ii], cNnx);

//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u_Q[ii], cNnx);

		printf("\nH_R, N^2 n_x^2 algorithm\n");
		d_print_pmat(N*nu, N*nu, bs, pH_R[0], cNnu);
//		exit(1);
#endif



		// clear matrix
		for(ii=0; ii<pNnu*cNnu; ii++)
			pH_R[0][ii] = 0;

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_Q(N, nx, nu, hpA, 1, 0, hdQ, hpL, 1, hpGamma_0, hpGamma_0_Q, pH_Q[0], work);
			
			d_cond_R(N, nx, nu, 2, hpA, hpAt, hpBt, hpBAt, 1, 1, hdQ, 0, hpL, hpS, hdR, hdRSQ, pD, pM, pQs, pLam, diag, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			//d_cond_St(N, nx, nu, 0, hpS, 1, hpGamma_0, 0, hpGamma_0_Q, hpGamma_u_Q, pH_St[0]);

			//d_cond_q(N, nx, nu, hpA, hb, 1, 1, hdQ, hq, hpGamma_0, 1, hGamma_b, 1, hGamma_b_q, H_q[0]);

			d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]);

			//d_cond_A(N, nx, nu, hpA, 0, hpGamma_0, pH_A[0]);

			//d_cond_B(N, nx, nu, hpA, hpBt, 0, hpGamma_u, pH_B[0]);

			//d_cond_b(N, nx, nu, hpA, hb, 0, hGamma_b, H_b[0]);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond_N2_nx3 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 0
//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u[ii], cNnx);

//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u_Q[ii], cNnx);

		printf("\nH_R, N^2 n_x^3 algorithm\n");
		d_print_pmat(N*nu, N*nu, bs, pH_R[0], cNnu);
//		exit(1);
#endif



		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			// Cholesky factorization and solution
			dpotrf_lib(N*nu, N*nu, pH_R[0], cNnu, pL_R, cNnu, diag);

//			dax_mat(N*nu, 1, -1.0, H_r[0], 1, H_u, 1);

#if 0
			for(ii=0; ii<N*nu; ii++)
				pL_R[ii/bs*bs*cNnu+ii%bs+ii*bs] = diag[ii];

			dtrsv_n_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
			dtrsv_t_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
#else
//			dtrsv_n_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
//			dtrsv_t_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
#endif
			
//			for(jj=0; jj<N; jj++)
//				{
//				dgemv_n_lib(nx, nx, pA, cnx, hx[jj], hb[jj], hx[jj+1], 1);
//				d_copy_mat(nu, 1, H_u+jj*nu, 1, hu[jj], 1);
//				dgemv_n_lib(nx, nu, pB, cnu, hu[jj], hx[jj+1], hx[jj+1], 1);
//				}

			}

		gettimeofday(&tv1, NULL); // start

		double time_fact_sol = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 0
		printf("\nN^3 cholesky factorization of Hessiam\n");
		d_print_pmat(N*nu, N*nu, bs, pL_R, cNnu);
#endif


		// clear matrix
		for(ii=0; ii<pNnu*cNnu; ii++)
			pL_R[ii] = 0;

		// permute Hessian
		for(ii=0; ii<N; ii++)
			{
			for(jj=ii; jj<N; jj++)
				{
				dgecp_lib(nu, nu, ii*nu, pH_R[0]+ii*nu/bs*bs*cNnu+ii*nu%bs+jj*nu*bs, cNnu, (N-1-ii)*nu, pL_R+(N-1-ii)*nu/bs*bs*cNnu+(N-1-ii)*nu%bs+(N-1-jj)*nu*bs, cNnu);
				}
			}

		dpotrf_lib(N*nu, N*nu, pL_R, cNnu, pL_R, cNnu, diag);

#if 0
		printf("\nN^3 cholesky factorization of reversed Hessiam\n");
		d_print_pmat(N*nu, N*nu, bs, pL_R, cNnu);
		//exit(1);
#endif


		// clear matrix
		for(ii=0; ii<pNnu*cNnu; ii++)
			pL_R[ii] = 0;

		for(ii=0; ii<(N*nu+bs-1)/bs*bs*cnx; ii++)
			hpGamma_u[N-1][ii] = 0.0;

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			// Cholesky factorization and solution
			d_cond_fact_R(N, nx, nu, 1, hpA, hpAt, hpBt, 1, hdQ, hpS, hdR, pQs, pM, pD, 1, hpGamma_u, hpGamma_u_Q, diag_ric, hpBAt, hpRSQ, pL, pBAtL, pL_R);

//			dax_mat(N*nu, 1, -1.0, H_r[0], 1, H_u, 1);

#if 0
			for(ii=0; ii<N*nu; ii++)
				pL_R[ii/bs*bs*cNnu+ii%bs+ii*bs] = diag[ii];

			dtrsv_n_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
			dtrsv_t_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
#else
//			dtrsv_n_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
//			dtrsv_t_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
#endif
			
//			for(jj=0; jj<N; jj++)
//				{
//				dgemv_n_lib(nx, nx, pA, cnx, hx[jj], hb[jj], hx[jj+1], 1);
//				d_copy_mat(nu, 1, H_u+jj*nu, 1, hu[jj], 1);
//				dgemv_n_lib(nx, nu, pB, cnu, hu[jj], hx[jj+1], hx[jj+1], 1);
//				}

			}

		gettimeofday(&tv1, NULL); // start

		double time_fact_sol_N2 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


#if 0
		printf("\nN^2 n_x^2 condensing and cholesky factorization of reversed Hessiam\n");
		d_print_pmat(N*nu, N*nu, bs, pL_R, cNnu);
//		exit(1);
#endif

#if 0
		d_print_mat(1, N*nu, H_u, 1);
#endif

#if 0
		for(jj=0; jj<N; jj++)
			d_print_mat(1, nu, hu[jj], 1);

		for(jj=0; jj<=N; jj++)
			d_print_mat(1, nx, hx[jj], 1);
#endif



		// clear matrix
		for(ii=0; ii<pNnu*cNnu; ii++)
			pL_R[ii] = 0;

		for(ii=0; ii<(N*nu+bs-1)/bs*bs*cnx; ii++)
			hpGamma_u[N-1][ii] = 0.0;

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			// Cholesky factorization and solution
			d_cond_fact_R(N, nx, nu, 0, hpA, hpAt, hpBt, 1, hdQ, hpS, hdR, pQs, pM, pD, 1, hpGamma_u, hpGamma_u_Q, diag_ric, hpBAt, hdRSQ, pL, pBAtL, pL_R);

//			dax_mat(N*nu, 1, -1.0, H_r[0], 1, H_u, 1);

#if 0
			for(ii=0; ii<N*nu; ii++)
				pL_R[ii/bs*bs*cNnu+ii%bs+ii*bs] = diag[ii];

			dtrsv_n_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
			dtrsv_t_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
#else
//			dtrsv_n_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
//			dtrsv_t_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
#endif
			
//			for(jj=0; jj<N; jj++)
//				{
//				dgemv_n_lib(nx, nx, pA, cnx, hx[jj], hb[jj], hx[jj+1], 1);
//				d_copy_mat(nu, 1, H_u+jj*nu, 1, hu[jj], 1);
//				dgemv_n_lib(nx, nu, pB, cnu, hu[jj], hx[jj+1], hx[jj+1], 1);
//				}

			}

		gettimeofday(&tv1, NULL); // start

		double time_fact_sol_N2_nx3 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


#if 0
		printf("\nN^2 n_x^3 condensing and cholesky factorization of reversed Hessiam\n");
		d_print_pmat(N*nu, N*nu, bs, pL_R, cNnu);
//		exit(1);
#endif

#if 0
		d_print_mat(1, N*nu, H_u, 1);
#endif

#if 0
		for(jj=0; jj<N; jj++)
			d_print_mat(1, nu, hu[jj], 1);

		for(jj=0; jj<=N; jj++)
			d_print_mat(1, nx, hx[jj], 1);
#endif



		int alg;
		int diag_hessian = 0;

		double *pZ; d_zeros_align(&pZ, pnx, cnx);
		double *ptr_temp;

		//printf("\n%d\n", d_cond_lqcp_work_space(N, nx, nu, N2) );
		//exit(1);

		nx_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) nx_v[ii] = nx;

		nu_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) nu_v[ii] = N1*nu;
#if 1
		nu_v[N2] = 0;
#endif
		
		d_set_pmat(pNnu, cNnu, 0.0, bs, pH_R[0], cNnu); // TODO remove !!!!!


		alg = 0;
		double *work_space_part_cond; d_zeros_align(&work_space_part_cond, d_cond_lqcp_work_space(N, nx, nu, N2, alg), 1);

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			d_cond_lqcp(N, nx, nu, alg, hpA, hpAt, hpBt, hb, hpBAt, diag_hessian, hpQ, hpS, hpR, hr, hq, hpRSQ, hrq, N2, nx_v, nu_v, pH_A, pH_B, H_b, pH_R, pH_St, pH_Q, H_r, H_q, work_space_part_cond);

			}

		gettimeofday(&tv1, NULL); // start

		double time_part_cond_0 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);



		alg = 1;
		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			d_cond_lqcp(N, nx, nu, alg, hpA, hpAt, hpBt, hb, hpBAt, diag_hessian, hpQ, hpS, hpR, hr, hq, hpRSQ, hrq, N2, nx_v, nu_v, pH_A, pH_B, H_b, pH_R, pH_St, pH_Q, H_r, H_q, work_space_part_cond);

			}

		gettimeofday(&tv1, NULL); // start

		double time_part_cond_1 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		


		alg = 2;
		free(work_space_part_cond);
		work_space_part_cond; d_zeros_align(&work_space_part_cond, d_cond_lqcp_work_space(N, nx, nu, N2, alg), 1);

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			d_cond_lqcp(N, nx, nu, alg, hpA, hpAt, hpBt, hb, hpBAt, diag_hessian, hpQ, hpS, hpR, hr, hq, hpRSQ, hrq, N2, nx_v, nu_v, pH_A, pH_B, H_b, pH_R, pH_St, pH_Q, H_r, H_q, work_space_part_cond);

			}

		gettimeofday(&tv1, NULL); // start

		double time_part_cond_2 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 0
		for(jj=0; jj<N2+1; jj++)
			d_print_pmat(nx, nx, bs, pH_Q[jj], cnx);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(nx, N1*nu, bs, pH_St[jj], cN1nu);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(N1*nu, N1*nu, bs, pH_R[jj], cN1nu);

		for(jj=0; jj<N2+1; jj++)
			d_print_mat(1, nx, H_q[jj], 1);

		for(jj=0; jj<N2; jj++)
			d_print_mat(1, N1*nu, H_r[jj], 1);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(nx, nx, bs, pH_A[jj], cnx);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(nx, N1*nu, bs, pH_B[jj], cN1nu);

		for(jj=0; jj<N2; jj++)
			d_print_mat(1, nx, H_b[jj], 1);
#endif


		int nu0, nx0, nx1, cnu0, pnx0, pnx1, cnx0, cnx1, pnz0, cnz0, cnl0;

		double *pH_BAbt[N2];
		double *pH_RSQrq[N2+1];
		double *pH_L[N2+1];
		double *H_ux[N2+1];
		double *H_pi[N2+1];
		double *H_rq[N2+1];
		double *H_res_rq[N2+1];
		double *H_res_b[N2];

		for(ii=0; ii<N2; ii++)
			{
			pnx0 = (nx_v[ii]+bs-1)/bs*bs;
			pnx1 = (nx_v[ii+1]+bs-1)/bs*bs;
			cnx0 = (nx_v[ii]+ncl-1)/ncl*ncl;
			cnx1 = (nx_v[ii+1]+ncl-1)/ncl*ncl;
			pnz0 = (nx_v[ii]+nu_v[ii]+1+bs-1)/bs*bs;
			cnz0 = (nx_v[ii]+nu_v[ii]+1+ncl-1)/ncl*ncl;
			cnl0 = cnz0<cnx0+ncl ? cnx0+ncl : cnz0;
			d_zeros_align(&pH_BAbt[ii], pnz0, cnx1);
			d_zeros_align(&pH_RSQrq[ii], pnz0, cnz0);
			d_zeros_align(&pH_L[ii], pnz0, cnl0);
			d_zeros_align(&H_ux[ii], pnz0, 1);
			d_zeros_align(&H_pi[ii], pnx0, 1);
			d_zeros_align(&H_rq[ii], pnz0, 1);
			d_zeros_align(&H_res_rq[ii], pnz0, 1);
			d_zeros_align(&H_res_b[ii], pnx1, 1);
			}
		ii = N2;
		pnx0 = (nx_v[ii]+bs-1)/bs*bs;
		pnz0 = (nx_v[ii]+nu_v[ii]+1+bs-1)/bs*bs;
		cnz0 = (nx_v[ii]+nu_v[ii]+1+ncl-1)/ncl*ncl;
		cnl0 = cnz0<cnx0+ncl ? cnx0+ncl : cnz0;
		d_zeros_align(&pH_RSQrq[N2], pnz0, cnz0);
		d_zeros_align(&pH_L[N2], pnz0, cnl0);
		d_zeros_align(&H_ux[N2], pnz0, 1);
		d_zeros_align(&H_pi[N2], pnx0, 1);
		d_zeros_align(&H_rq[N2], pnz0, 1);
		d_zeros_align(&H_res_rq[N2], pnz0, 1);

		
		for(ii=0; ii<N2; ii++)
			{
			nx0 = nx_v[ii];
			nx1 = nx_v[ii+1];
			nu0 = nu_v[ii];
			cnx0 = (nx0+ncl-1)/ncl*ncl;
			cnx1 = (nx1+ncl-1)/ncl*ncl;
			cnu0 = (nu0+ncl-1)/ncl*ncl;
			dgetr_lib(nx1, nu0, 0, pH_B[ii], cnu0, 0, pH_BAbt[ii], cnx1);
			dgetr_lib(nx1, nx0, 0, pH_A[ii], cnx0, nu0, pH_BAbt[ii]+nu0/bs*bs*cnx1+nu0%bs, cnx1);
			dgetr_lib(nx1, 1, 0, H_b[ii], 1, nu0+nx0, pH_BAbt[ii]+(nu0+nx0)/bs*bs*cnx1+(nu0+nx0)%bs, cnx1);
			d_print_pmat(nx0+nu0+1, nx1, bs, pH_BAbt[ii], cnx1);
			}

		for(ii=0; ii<=N2; ii++)
			{
			nx0 = nx_v[ii];
			nu0 = nu_v[ii];
			cnu0 = (nu0+ncl-1)/ncl*ncl;
			cnx0 = (nx0+ncl-1)/ncl*ncl;
			cnz0 = (nu0+nx0+1+ncl-1)/ncl*ncl;
			dgecp_lib(nu0, nu0, 0, pH_R[ii], cnu0, 0, pH_RSQrq[ii], cnz0);
			dgecp_lib(nx0, nu0, 0, pH_St[ii], cnu0, nu0, pH_RSQrq[ii]+(nu0)/bs*bs*cnz0+(nu0)%bs, cnz0);
			dgecp_lib(nx0, nx0, 0, pH_Q[ii], cnx0, nu0, pH_RSQrq[ii]+(nu0)/bs*bs*cnz0+(nu0)%bs+(nu0)*bs, cnz0);
			dgetr_lib(nu0, 1, 0, H_r[ii], 1, nu0+nx0, pH_RSQrq[ii]+(nu0+nx0)/bs*bs*cnz0+(nu0+nx0)%bs, cnz0);
			dgetr_lib(nx0, 1, 0, H_q[ii], 1, nu0+nx0, pH_RSQrq[ii]+(nu0+nx0)/bs*bs*cnz0+(nu0+nx0)%bs+(nu0)*bs, cnz0);
			d_print_pmat(nx0+nu0+1, nx0+nu0+1, bs, pH_RSQrq[ii], cnz0);
			}
		ii = N2;
		nx0 = nx_v[ii];
		cnx0 = (nx0+ncl-1)/ncl*ncl;
		cnz0 = (nx0+1+ncl-1)/ncl*ncl;
		dgecp_lib(nx0, nx0, 0, pH_Q[N2], cnx0, 0, pH_RSQrq[N2], cnz0);
		dgetr_lib(nx0, 1, 0, H_q[N2], 1, nx0, pH_RSQrq[N2]+nx0/bs*bs*cnz0+nx0%bs, cnz0);
		d_print_pmat(nx0+1, nx0+1, bs, pH_RSQrq[N2], cnz0);


#if 0
		// partial condensing algorithm
		printf("\npatrial condensing algorithm\n");
		d_cond_A((N+N2-1)/N2, nx, nu, hpA, 1, hpGamma_0, pH_A[0]);
//		for(ii=0; ii<N; ii++) d_print_pmat(nx, nx, bs, hpGamma_0[ii], cnx);
//		d_print_pmat(nx, nx, bs, pH_A[0], cnx);
		d_cond_B((N+N2-1)/N2, nx, nu, hpA, hpBt, 1, hpGamma_u, pH_B[0]);
//		for(ii=0; ii<N; ii++) d_print_pmat((ii+1)*nu, nx, bs, hpGamma_u[ii], cnx);
//		d_print_pmat(nx, N*nu, bs, pH_B[0], cNnu);
//		for(ii=0; ii<N; ii++) d_set_pmat(nx, 1, 0.0, 0, hGamma_b[ii], 1);
		d_cond_b((N+N2-1)/N2, nx, nu, hpA, hb, 1, hGamma_b, H_b[0]);
//		for(ii=0; ii<N; ii++) d_print_mat(1, nx, hGamma_b[ii], 1);
//		d_print_mat(1, nx, H_b[0], 1);

		d_set_pmat(N*nu, N*nu, 0.0, 0, pH_R[0], cNnu);
		d_set_pmat(nx, N*nu, 0.0, 0, pH_St[0], cNnu);
		d_set_pmat(nx, nx, 0.0, 0, pH_Q[0], cnx);
		d_set_pmat(N*nu, 1, 0.0, 0, H_r[0], 1);
		d_set_pmat(nx, 1, 0.0, 0, H_q[0], 1);
//		d_print_mat(1, nx, H_q[0], 1);
		d_part_cond_RSQrq((N+N2-1)/N2, nx, nu, hpBAt, hb, 1, hdRSQ, hrq, hpGamma_0, hpGamma_u, hGamma_b, pM, pLam, diag_ric, pBAtL, pH_R[0], pH_St[0], pH_Q[0], H_r[0], H_q[0]);
		printf("\nhola\n");
		d_print_pmat((N+N2-1)/N2, (N+N2-1)/N2, bs, pH_R[0], 4);
		d_print_pmat(nx, (N+N2-1)/N2, bs, pH_St[0], 4);
		d_print_pmat(nx, nx, bs, pH_Q[0], cnx);
		d_print_mat(1, (N+N2-1)/N2, H_r[0], 1);
		d_print_mat(1, nx, H_q[0], 1);
		exit(1);
#endif



		nb_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) nb_v[ii] = 0;

		ng_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) ng_v[ii] = 0;

		int nz_m = 0;
		for(ii=0; ii<=N2; ii++)
			nz_m = nu_v[ii]+nx_v[ii]+1>nz_m ? nu_v[ii]+nx_v[ii]+1 : nz_m ;

		double *work_tv; d_zeros_align(&work_tv, d_ric_sv_mpc_tv_work_space_size_double(N2, nx_v, nu_v, nb_v, ng_v), 1);
		double *diag_tv; d_zeros_align(&diag_tv, (nz_m+bs-1)/bs*bs, 1);

#if 0
		for(ii=0; ii<=N2; ii++)
			printf("\n%d\t%d\n", nu_v[ii], nx_v[ii]);
		//exit(1);
#endif




		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

#if 0
			d_ric_sv_mpc(nx, N1*nu, N2, pH_BAbt, pH_RSQrq, 0, dummy, dummy, H_ux, pH_L, work1, diag_tv, 0, dummy, 0, 0, 0, dummy, dummy, dummy, 0);
#else
			d_ric_sv_mpc_tv(N2, nx_v, nu_v, pH_BAbt, pH_RSQrq, H_ux, pH_L, work_tv, diag_tv, 0, dummy, 1, H_pi, nb_v, 0, dummy, dummy, ng_v, dummy, dummy, dummy, 0);
#endif

			}

		gettimeofday(&tv1, NULL); // start

		double time_part_cond_ric = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		
#if 0
		printf("\nux\n");
		for(jj=0; jj<=N2; jj++)
			d_print_mat(1, nu_v[jj]+nx_v[jj], H_ux[jj], 1);

		printf("\npi\n");
		for(jj=1; jj<=N2; jj++)
			d_print_mat(1, nx_v[jj], H_pi[jj], 1);
#endif

#if 1
		printf("\nu\n");
		for(jj=0; jj<N2; jj++)
			for(ii=0; ii<nu_v[jj]/nu; ii++)
				d_print_mat(1, nu, H_ux[jj]+ii*nu, 1);
#endif

#if 0
		printf("\nrq\n");
		for(jj=0; jj<=N2; jj++)
			{
			nu0 = nu_v[jj];
			nx0 = nx_v[jj];
			d_copy_mat(nu0, 1, H_r[jj], 1, H_rq[jj], 1);
			d_copy_mat(nx0, 1, H_q[jj], 1, H_rq[jj]+nu0, 1);
			d_print_mat(1, nx0+nu0, H_rq[jj], 1);
			}

		d_res_mpc_tv(N2, nx_v, nu_v, pH_BAbt, pH_RSQrq, H_rq, H_ux, H_pi, H_res_rq, H_res_b);

		printf("\nresiduals\n");
		for(jj=0; jj<=N2; jj++)
			d_print_mat(1, nu_v[jj]+nx_v[jj], H_res_rq[jj], 1);

		for(jj=0; jj<N2; jj++)
			d_print_mat(1, nx_v[jj+1], H_res_b[jj], 1);
		printf("\n");

	
#endif


		d_copy_mat(nx, 1, x0, nx, hux[0]+nu, nx);

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			d_ric_sv_mpc(nx, nu, N, hpBAbt, hpRSQrq, 0, dummy, dummy, hux, hpL2, work2, diag2, 0, dummy, 0, dummy, 0, 0, 0, dummy, dummy, dummy, 0);

			}

		gettimeofday(&tv1, NULL); // start

		double time_ric = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	
#if 1
		printf("\nux\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, nu+nx, hux[ii], 1);
#endif

		printf("\ntime condensing = %e seconds\n", time_cond);
		printf("\ntime factorization  = %e seconds\n", time_fact_sol);
		printf("\ntime factorization N2 = %e seconds\n", time_fact_sol_N2);
		printf("\ntime factorization N2 nx3 = %e seconds\n", time_fact_sol_N2_nx3);
		printf("\ntime partial condensing 0 = %e seconds\n", time_part_cond_0);
		printf("\ntime partial condensing 1 = %e seconds\n", time_part_cond_1);
		printf("\ntime partial condensing 2 = %e seconds\n", time_part_cond_2);
		printf("\ntime partial condensing riccati = %e seconds\n", time_part_cond_ric);
		printf("\ntime full riccati = %e seconds\n", time_ric);
		printf("\n\n");

/************************************************
* return
************************************************/

		free(nx_v);
		free(nu_v);
		free(nb_v);
		free(ng_v);

		free(A);
		free(B);
		free(b);
		free(x0);
		free(Q);
		free(pA);
		free(pAt);
		free(pB);
		free(pBt);
		free(pBAbt);
		free(pBAt);
		free(pQ);
		free(dQ);
		free(pR);
		free(dR);
		free(pS);
		free(q);
		free(r);
		free(pRSQrq);
		free(rq);
		free(pRSQ);
		free(dRSQ);
		free(pD);
		free(pM);
		free(pQs);
		free(pLam);
		free(pL);
		free(pBAtL);
		free(pBAbtL);
		for(ii=0; ii<N2; ii++)
			{
			free(pH_A[ii]);
			free(pH_B[ii]);
			free(H_b[ii]);
			free(pH_R[ii]);
			free(pH_Q[ii]);
			free(pH_St[ii]);
			free(H_q[ii]);
			free(H_r[ii]);

			free(pH_BAbt[ii]);
			free(pH_RSQrq[ii]);
			}
		free(pH_RSQrq[N2]);

		free(pL_R);
		free(H_u);
		free(work);
		//free(pGamma_u);
		//free(pGamma_u_Q);
		//free(pGamma_u_Q_A);
		//free(pGamma_0);
		//free(pGamma_0_Q);
		free(diag);
		free(diag_ric);
		free(diag_tv);
		free(work_tv);
		free(diag2);
		free(work2);
		free(work_space_part_cond);

		for(ii=0; ii<N; ii++)
			{
			free(hpGamma_u[ii]);
			free(hpGamma_u_Q[ii]);
			free(hpGamma_u_Q_A[ii]);
			free(hpGamma_0[ii]);
			free(hpGamma_0_Q[ii]);
			free(hGamma_b[ii]);
			free(hGamma_b_q[ii]);
			free(hpL[ii]);
			free(hx[ii]);
			free(hu[ii]);
			free(hpL2[ii]);
			free(hux[ii]);
			}
		free(hpL[N]);
		free(hx[N]);
		free(hpL2[N]);
		free(hux[N]);

		} // increase size

	fprintf(f, "];\n");
	fclose(f);


	return 0;

	}



#else // timing


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

	printf("Riccati solver performance test - double precision\n");
	printf("\n");

	// maximum frequency of the processor
	const float GHz_max = GHZ_MAX;
	printf("Frequency used to compute theoretical peak: %5.1f GHz (edit test_param.h to modify this value).\n", GHz_max);
	printf("\n");

	// maximum flops per cycle, double precision
#if defined(TARGET_X64_AVX2)
	const float flops_max = 16;
	printf("Testing solvers for AVX & FMA3 instruction sets, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_AVX)
	const float flops_max = 8;
	printf("Testing solvers for AVX instruction set, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_SSE3) || defined(TARGET_AMD_SSE3)
	const float flops_max = 4;
	printf("Testing solvers for SSE3 instruction set, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_CORTEX_A15)
	const float flops_max = 2;
	printf("Testing solvers for ARMv7a VFPv3 instruction set, oprimized for Cortex A15: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_CORTEX_A9)
	const float flops_max = 1;
	printf("Testing solvers for ARMv7a VFPv3 instruction set, oprimized for Cortex A9: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_CORTEX_A7)
	const float flops_max = 0.5;
	printf("Testing solvers for ARMv7a VFPv3 instruction set, oprimized for Cortex A7: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_ATOM)
	const float flops_max = 1;
	printf("Testing solvers for SSE3 instruction set, 32 bit, optimized for Intel Atom: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_POWERPC_G2)
	const float flops_max = 1;
	printf("Testing solvers for POWERPC instruction set, 32 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_4X4)
	const float flops_max = 2;
	printf("Testing reference solvers, 4x4 kernel: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_4X4_PREFETCH)
	const float flops_max = 2;
	printf("Testing reference solvers, 4x4 kernel with register prefetch: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_2X2)
	const float flops_max = 2;
	printf("Testing reference solvers, 2x2 kernel: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#endif
	
	FILE *f;
	f = fopen("./test_problems/results/test_blas.m", "w"); // a

#if defined(TARGET_X64_AVX2)
	fprintf(f, "C = 'd_x64_avx2';\n");
	fprintf(f, "\n");
#elif defined(TARGET_X64_AVX)
	fprintf(f, "C = 'd_x64_avx';\n");
	fprintf(f, "\n");
#elif defined(TARGET_X64_SSE3) || defined(TARGET_AMD_SSE3)
	fprintf(f, "C = 'd_x64_sse3';\n");
	fprintf(f, "\n");
#elif defined(TARGET_CORTEX_A9)
	fprintf(f, "C = 'd_ARM_cortex_A9';\n");
	fprintf(f, "\n");
#elif defined(TARGET_CORTEX_A7)
	fprintf(f, "C = 'd_ARM_cortex_A7';\n");
	fprintf(f, "\n");
#elif defined(TARGET_CORTEX_A15)
	fprintf(f, "C = 'd_ARM_cortex_A15';\n");
	fprintf(f, "\n");
#elif defined(TARGET_X86_ATOM)
	fprintf(f, "C = 'd_x86_atom';\n");
	fprintf(f, "\n");
#elif defined(TARGET_POWERPC_G2)
	fprintf(f, "C = 'd_PowerPC_G2';\n");
	fprintf(f, "\n");
#elif defined(TARGET_C99_4X4)
	fprintf(f, "C = 'd_c99_4x4';\n");
	fprintf(f, "\n");
#elif defined(TARGET_C99_4X4_PREFETCH)
	fprintf(f, "C = 'd_c99_4x4';\n");
	fprintf(f, "\n");
#elif defined(TARGET_C99_2X2)
	fprintf(f, "C = 'd_c99_2x2';\n");
	fprintf(f, "\n");
#endif

	fprintf(f, "A = [%f %f];\n", GHz_max, flops_max);
	fprintf(f, "\n");

	fprintf(f, "B = [\n");
	

	const int LTI = 1;

	printf("\n");
	printf("Tested solvers:\n");
	printf("-sv : Riccati factorization and system solution (prediction step in IP methods)\n");
	printf("-trs: system solution after a previous call to Riccati factorization (correction step in IP methods)\n");
	printf("\n");
	if(LTI==1)
		printf("\nTest for linear time-invariant systems\n");
	else
		printf("\nTest for linear time-variant systems\n");
	printf("\n");
	
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
/*	printf("\nflush to zero on\n");*/
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); // flush to zero subnormals !!! works only with one thread !!!
#endif

	// to throw floating-point exception
/*#ifndef __APPLE__*/
/*    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);*/
/*#endif*/
	
	int ii, jj;

	double **dummy;

	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line
	
	int nn[] = {4, 6, 8, 10, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300};
	int nnrep[] = {4000, 4000, 4000, 4000, 4000, 2000, 2000, 2000, 1000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};

	int NNN[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100};
	int NNrep[] = {2000, 2000, 2000, 2000, 1000, 1000, 1000, 1000, 1000, 400, 400, 400, 400, 400, 100, 100, 100, 100, 100, 100, 20, 20, 10, 10, 10};
	
	int vnx[] = {8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 512, 1024};
	int vnrep[] = {100, 100, 100, 100, 100, 100, 50, 50, 50, 20, 10, 10};
	int vN[] = {4, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256};

	int nx, nu, N, nrep;

	int *nx_v, *nu_v, *nb_v, *ng_v;

	int ll;
//	int ll_max = 77;
	int ll_max = 25;//27;//25;
//	int ll_max = 1;
	for(ll=0; ll<ll_max; ll++)
		{
		

		if(ll_max==1)
			{
			nx = NX; // number of states (it has to be even for the mass-spring system test problem)
			nu = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = NN; // horizon lenght
			nrep = NREP;
			//nx = 25;
			//nu = 1;
			//N = 11;
			}
		else
			{
#if 1
			nx = NX; //nn[ll]; // number of states (it has to be even for the mass-spring system test problem)
			nu = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = NNN[ll]; //10; // horizon lenght
			nrep = 10*NNrep[ll];
#else
			nx = nn[ll]; // number of states (it has to be even for the mass-spring system test problem)
			nu = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = NN; //10; // horizon lenght
			nrep = nnrep[ll];
#endif
			}


		int rep;
	
		int nz = nx+nu+1;
		int anz = (nz+nal-1)/nal*nal;
		int anx = (nx+nal-1)/nal*nal;
		int pnz = (nz+bs-1)/bs*bs;
		int pnx = (nx+bs-1)/bs*bs;
		int pnu = (nu+bs-1)/bs*bs;
		int pNnu = (N*nu+bs-1)/bs*bs;
		int cnz = (nx+nu+1+ncl-1)/ncl*ncl;
		int cnx = (nx+ncl-1)/ncl*ncl;
		int cnu = (nu+ncl-1)/ncl*ncl;
		//int cNnu = ((N-1)*nu+cnu+ncl-1)/ncl*ncl;
		int cNnu = (N*nu+ncl-1)/ncl*ncl;
		int cNnx = (N*nx+ncl-1)/ncl*ncl;
		int cnl = cnz>cnx+ncl ? pnz : cnx+ncl;

		int ny = nu+nx;
		int pny = (ny+bs-1)/bs*bs;
		int cny = (ny+ncl-1)/ncl*ncl;
		int cnk = cnx+ncl>cny ? cnx+ncl : cny;


/************************************************
* dynamical system
************************************************/	

		double *A; d_zeros(&A, nx, nx); // states update matrix

		double *B; d_zeros(&B, nx, nu); // inputs matrix

		double *b; d_zeros_align(&b, pnx, 1); // states offset
		double *x0; d_zeros_align(&x0, pnx, 1); // initial state

		double Ts = 0.5; // sampling time
		mass_spring_system(Ts, nx, nu, N, A, B, b, x0);
	
		for(jj=0; jj<nx; jj++)
			b[jj] = 0.1;
	
		for(jj=0; jj<nx; jj++)
			x0[jj] = 0;
		x0[0] = 3.5;
		x0[1] = 3.5;

//		d_print_mat(nx, nx, A, nx);
//		d_print_mat(nx, nu, B, nx);
//		d_print_mat(nx, 1, b, nx);
//		d_print_mat(nx, 1, x0, nx);
//		exit(1);

		double *pA; d_zeros_align(&pA, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);
		//d_print_pmat(nx, nx, bs, pA, cnx);

		double *pAt; d_zeros_align(&pAt, pnx, cnx);
		d_cvt_tran_mat2pmat(nx, nx, A, nx, 0, pAt, cnx);
		//d_print_pmat(nx, nx, bs, pA, cnx);

		double *pB; d_zeros_align(&pB, pnx, cnu);
		d_cvt_mat2pmat(nx, nu, B, nx, 0, pB, cnu);
		//d_print_pmat(nx, nu, bs, pB, cnu);

		double *pBt; d_zeros_align(&pBt, pnu, cnx);
		d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBt, cnx);
		//d_print_pmat(nu, nx, bs, pBt, cnx);

		double *b0; d_zeros_align(&b0, pnx, 1);
		dgemv_n_lib(nx, nx, pA, cnx, x0, b, b0, 1);

		double *pBAbt; d_zeros_align(&pBAbt, pnz, cnx);
		d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBAbt, cnx);
		d_cvt_tran_mat2pmat(nx, nx, A, nx, nu, pBAbt+nu/bs*bs*cnx+nu%bs, cnx);
		d_cvt_tran_mat2pmat(nx, 1, b, nx, nu+nx, pBAbt+(nu+nx)/bs*bs*cnx+(nu+nx)%bs, cnx);
		//d_print_pmat(nx+nu+1, nx, bs, pBAbt, cnx);
		//exit(1);
	
		double *pBAt; d_zeros_align(&pBAt, pnz, cnx);
		dgecp_lib(nu, nx, 0, pBt, cnx, 0, pBAt, cnx);
		dgecp_lib(nx, nx, 0, pAt, cnx, nu, pBAt+nu/bs*bs*cnx+nu%bs, cnx);
		//d_print_pmat(nz, nx, bs, pBAt, cnx);

/************************************************
* cost function
************************************************/

		double *Q; d_zeros(&Q, nx, nx);
		for(ii=0; ii<nx; ii++)
			Q[ii*(nx+1)] = 3.0;
		//d_print_mat(nx, nx, Q, nx);

		double *R; d_zeros(&R, nu, nu);
		for(ii=0; ii<nu; ii++)
			R[ii*(nu+1)] = 2.0;
		//d_print_mat(nu, nu, R, nu);

		double *S; d_zeros(&S, nu, nx);
		for(ii=0; ii<nu; ii++)
			S[ii*(nu+1)] = 0.0;
		//d_print_mat(nu, nx, S, nu);

		double *q; d_zeros_align(&q, pnx, 1);
		for(ii=0; ii<nx; ii++)
			q[ii] = 0.1;

		double *r; d_zeros_align(&r, pnu, 1);
		for(ii=0; ii<nu; ii++)
			r[ii] = 0.1;

		double *pQ; d_zeros_align(&pQ, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, Q, nx, 0, pQ, cnx);
		//d_print_pmat(nx, nx, bs, pQ, cnx);

		double *dQ; d_zeros_align(&dQ, pnx, 1);
		for(ii=0; ii<nx; ii++) dQ[ii] = Q[ii*(nx+1)];
		//d_print_mat(1, nx, dQ, 1);

		double *pR; d_zeros_align(&pR, pnu, cnu);
		d_cvt_mat2pmat(nu, nu, R, nu, 0, pR, cnu);
		//d_print_pmat(nu, nu, bs, pR, cnu);

		double *dR; d_zeros_align(&dR, pnu, 1);
		for(ii=0; ii<nu; ii++) dR[ii] = R[ii*(nu+1)];
		//d_print_mat(1, nu, dR, 1);

		double *dRSQ; d_zeros_align(&dRSQ, pnu+pnx, 1);
		for(ii=0; ii<nu; ii++) dRSQ[ii] = R[ii*(nu+1)];
		for(ii=0; ii<nx; ii++) dRSQ[nu+ii] = Q[ii*(nx+1)];
		//d_print_mat(1, nu, dR, 1);

		double *pS; d_zeros_align(&pS, pnu, cnx); // TODO change definition to transposed !!!!!!!!!!
		d_cvt_mat2pmat(nu, nx, S, nu, 0, pS, cnx);
		//d_print_pmat(nu, nx, bs, pS, cnx);

		double *pRSQrq; d_zeros_align(&pRSQrq, pnz, cnz);
		d_cvt_mat2pmat(nu, nu, R, nu, 0, pRSQrq, cnz);
		d_cvt_tran_mat2pmat(nu, nx, S, nu, nu, pRSQrq+nu/bs*bs*cnz+nu%bs, cnz);
		d_cvt_mat2pmat(nx, nx, Q, nx, nu, pRSQrq+nu/bs*bs*cnz+nu%bs+nu*bs, cnz);
		d_cvt_tran_mat2pmat(nu, 1, r, nu, nu+nx, pRSQrq+(nu+nx)/bs*bs*cnz+(nu+nx)%bs, cnz);
		d_cvt_tran_mat2pmat(nx, 1, q, nx, nu+nx, pRSQrq+(nu+nx)/bs*bs*cnz+(nu+nx)%bs+nu*bs, cnz);
		//d_print_pmat(nz, nz, bs, pRSQrq, cnz);
		//exit(1);

		double *pRSQ; d_zeros_align(&pRSQ, pny, cny);
		dgecp_lib(nu, nu, 0, pR, cnu, 0, pRSQ, cny);
		dgetr_lib(nu, nx, 0, pS, cnx, nu, pRSQ+nu/bs*bs*cny+nu%bs, cny);
		dgecp_lib(nx, nx, 0, pQ, cnx, nu, pRSQ+nu/bs*bs*cny+nu%bs+nu*bs, cny);
//		d_print_pmat(nz, nz, bs, pRSQ, cny);
//		exit(1);

/************************************************
* matrix series
************************************************/

		int N2 = 4;
		int N1 = (N+N2-1)/N2;
		int cN1nu = (N1*nu+ncl-1)/ncl*ncl;
		int cN1nx = (N1*nx+ncl-1)/ncl*ncl;

		double *hpA[N];
		double *hpAt[N];
		double *hpBt[N];
		double *hb[N];
		double *hpBAbt[N];
		double *hpGamma_u[N];
		double *hpGamma_u_Q[N];
		double *hpGamma_u_Q_A[N];
		double *hpGamma_0[N];
		double *hpGamma_0_Q[N];
		double *hGamma_b[N];
		double *hGamma_b_q[N];
		double *hpQ[N+1];
		double *hdQ[N+1];
		double *hpR[N];
		double *hdR[N];
		double *hpS[N];
		double *hq[N+1];
		double *hr[N];
		double *hpRSQrq[N+1];
		double *hpL[N+1];
		double *hx[N+1];
		double *hu[N];
		double *hpL2[N+1];
		double *hux[N+1];
		double *pH_A[N2];
		double *pH_B[N2];
		double *H_b[N2];
		double *pH_R[N2];
		double *pH_Q[N2+1];
		double *pH_St[N2];
		double *H_q[N2+1];
		double *H_r[N2];
		double *pL_R;
		double *H_u;
		double *work;
		//double *pGamma_u;
		//double *pGamma_u_Q;
		//double *pGamma_u_Q_A;
		//double *pGamma_0;
		//double *pGamma_0_Q;
		double *diag;
		double *diag_ric; d_zeros_align(&diag_ric, nz, 1);
		double *diag2; d_zeros_align(&diag2, pnz, 1);
		double *work2; d_zeros_align(&work2, pnz, cnx);
		double *pD; d_zeros_align(&pD, pnu+pnx, cnu);
		double *pM; d_zeros_align(&pM, pnu, cnx);
		double *pQs; d_zeros_align(&pQs, pnx, cnx);
		double *pLam; d_zeros_align(&pLam, pny, cny);
		double *hpBAt[N];
		double *hpRSQ[N+1];
		double *hdRSQ[N+1];
		double *pL; d_zeros_align(&pL, pnz, cnl);
		double *pBAtL; d_zeros_align(&pBAtL, pnz, cnx);

		for(ii=0; ii<N2; ii++)
			{
			d_zeros_align(&pH_A[ii], pnx, cnx);
			d_zeros_align(&pH_B[ii], pnx, cNnu);
			d_zeros_align(&H_b[ii], pnx, 1);
			d_zeros_align(&pH_R[ii], pNnu, cNnu);
			d_zeros_align(&pH_Q[ii], pnx, cnx);
			d_zeros_align(&pH_St[ii], pnx, cNnu);
			d_zeros_align(&H_q[ii], pnx, 1);
			d_zeros_align(&H_r[ii], pNnu, 1);
			}
		pH_Q[N2] = pQ;
		H_q[N2] = q;

		d_zeros_align(&pL_R, pNnu, cNnu);
		d_zeros_align(&H_u, pNnu, 1);
		d_zeros_align(&work, pnx, 1);
		//d_zeros_align(&pGamma_u, pNnu, cNnx);
		//d_zeros_align(&pGamma_u_Q, pNnu, cNnx);
		//d_zeros_align(&pGamma_u_Q_A, pNnu, cNnx);
		//d_zeros_align(&pGamma_0, pnx, cNnx);
		//d_zeros_align(&pGamma_0_Q, pnx, cNnx);
		d_zeros_align(&diag, cNnu, 1);
		for(ii=0; ii<N; ii++)
			{
			hpA[ii] = pA;
			hpAt[ii] = pAt;
			hpBt[ii] = pBt;
			hb[ii] = b;
			hpBAbt[ii] = pBAbt;
			hpBAt[ii] = pBAt;
			d_zeros_align(&hpGamma_u[ii], ((ii+1)*nu+bs-1)/bs*bs, cnx);
			d_zeros_align(&hpGamma_u_Q[ii], ((ii+2)*nu+bs-1)/bs*bs, cnx);
			d_zeros_align(&hpGamma_u_Q_A[ii], ((ii+2)*nu+bs-1)/bs*bs, cnx);
			//hpGamma_u[ii] = pGamma_u+ii*nx*bs;
			//hpGamma_u_Q[ii] = pGamma_u_Q+ii*nx*bs;
			//hpGamma_u_Q_A[ii] = pGamma_u_Q_A+ii*nx*bs;
			d_zeros_align(&hpGamma_0[ii], pnx, cnx);
			d_zeros_align(&hpGamma_0_Q[ii], pnx, cnx);
			//hpGamma_0[ii] = pGamma_0+ii*nx*bs;
			//hpGamma_0_Q[ii] = pGamma_0_Q+ii*nx*bs;
			d_zeros_align(&hGamma_b[ii], pnx, 1);
			d_zeros_align(&hGamma_b_q[ii], pnx, 1);
			hpQ[ii] = pQ;
			hdQ[ii] = dQ;
			hpR[ii] = pR;
			hdR[ii] = dR;
			hpS[ii] = pS;
			hq[ii] = q;
			hr[ii] = r;
			hpRSQrq[ii] = pRSQrq;
			hpRSQ[ii] = pRSQ;
			hdRSQ[ii] = dRSQ;
			d_zeros_align(&hpL[ii], pnx, cnx);
			d_zeros_align(&hx[ii], pnx, 1);
			d_zeros_align(&hu[ii], pnu, 1);
			d_zeros_align(&hpL2[ii], pnz, cnl);
			d_zeros_align(&hux[ii], pnz, 1);
			}
		hpQ[N] = pQ;
		hdQ[N] = dQ;
		hq[N] = q;
		hpRSQrq[N] = pRSQrq;
		hpRSQ[N] = pRSQ;
		hdRSQ[N] = dRSQ;
		d_zeros_align(&hpL[N], pnx, cnx);;
		d_zeros_align(&hx[N], pnx, 1);
		d_zeros_align(&hpL2[N], pnz, cnl);
		d_zeros_align(&hux[N], pnz, 1);
		hb[0] = b0; // embed x0 !!!!!

/************************************************
* condensing
************************************************/
		
		struct timeval tv0, tv1;

		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_Q(N, nx, nu, hpA, 1, 0, hdQ, hpL, 1, hpGamma_0, hpGamma_0_Q, pH_Q[0], work);
			
			d_cond_R(N, nx, nu, 1, hpA, hpAt, hpBt, hpBAt, 0, 1, hpQ, 0, hpL, hpS, hpR, hpRSQ, pD, pM, pQs, pLam, diag_ric, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			//d_cond_St(N, nx, nu, 0, hpS, 1, hpGamma_0, 0, hpGamma_0_Q, hpGamma_u_Q, pH_St[0]);

			//d_cond_q(N, nx, nu, hpA, hb, 1, 1, hdQ, hq, hpGamma_0, 1, hGamma_b, 1, hGamma_b_q, H_q[0]);

			d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]);

			//d_cond_A(N, nx, nu, hpA, 0, hpGamma_0, pH_A[0]);

			//d_cond_B(N, nx, nu, hpA, hpBt, 0, hpGamma_u, pH_B[0]);

			//d_cond_b(N, nx, nu, hpA, hb, 0, hGamma_b, H_b[0]);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);



		// timing

		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			d_cond_R(N, nx, nu, 0, hpA, hpAt, hpBt, hpBAt, 0, 1, hpQ, 0, hpL, hpS, hpR, hpRSQ, pD, pM, pQs, pLam, diag_ric, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N3_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			d_cond_R(N, nx, nu, 0, hpA, hpAt, hpBt, hpBAt, 0, 1, hpQ, 1, hpL, hpS, hpR, hpRSQ, pD, pM, pQs, pLam, diag_ric, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N3_cond_L = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			d_cond_R(N, nx, nu, 1, hpA, hpAt, hpBt, hpBAt, 0, 1, hpQ, 0, hpL, hpS, hpR, hpRSQ, pD, pM, pQs, pLam, diag_ric, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N2_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			d_cond_R(N, nx, nu, 2, hpA, hpAt, hpBt, hpBAt, 0, 1, hpQ, 0, hpL, hpS, hpR, hpRSQ, pD, pM, pQs, pLam, diag_ric, pBAtL, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N2_nx3_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_R(N, nx, nu, 1, hpA, hpAt, hpBt, 0, 1, hpQ, 0, hpL, 1, hpS, hpR, pD, pM, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);
			dpotrf_lib(N*nu, N*nu, pH_R[0], cNnu, pL_R, cNnu, diag);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N3_fact = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


		// initilize pD with identity
		d_set_pmat(pnu+pnx, cnu, 0.0, 0, pD, cnu);
		for(ii=0; ii<nu; ii++) pLam[ii/bs*bs*cnu+ii%bs+ii*bs] = 1.0;

		// initialize pQs with zero
		d_set_pmat(pnx, cnx, 0.0, 0, pQs, cnx);

		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_R(N, nx, nu, 1, hpA, hpAt, hpBt, 0, 1, hpQ, 0, hpL, 1, hpS, hpR, pD, pM, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);
			//dpotrf_lib(N*nu, N*nu, pH_R[0], cNnu, pL_R, cNnu, diag);
			for(ii=0; ii<N; ii++)
				{
				dpotrf_lib(nu+nx, nu, pD, cnu, pD, cnu, diag);
				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, pQs, cnx, pQs, cnx, 0);
				}
			dpotrf_lib(nu, nu, pD, cnu, pD, cnu, diag);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N_fact = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_R(N, nx, nu, 1, hpA, hpAt, hpBt, 0, 1, hpQ, 0, hpL, 1, hpS, hpR, pD, pM, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);
			d_cond_fact_R(N, nx, nu, 1, hpA, hpAt, hpBt, 0, hpQ, hpS, hpR, pQs, pM, pD, 1, hpGamma_u, hpGamma_u_Q, diag_ric, hpBAt, hpRSQ, pL, pBAtL, pL_R);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N2_cond_fact = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);


		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			//d_cond_R(N, nx, nu, 1, hpA, hpAt, hpBt, 0, 1, hpQ, 0, hpL, 1, hpS, hpR, pD, pM, 1, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[0]);
			d_cond_fact_R(N, nx, nu, 0, hpA, hpAt, hpBt, 0, hpQ, hpS, hpR, pQs, pM, pD, 1, hpGamma_u, hpGamma_u_Q, diag_ric, hpBAt, hpRSQ, pL, pBAtL, pL_R);

			// d_cond_r(N, nx, nu, hpA, hb, 1, 1, hdQ, 0, hpS, hq, hr, hpGamma_u, 1, hGamma_b, 1, hGamma_b_q, H_r[0]); TODO

			}

		gettimeofday(&tv1, NULL); // start

		double time_N2_nx3_cond_fact = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 0
//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nx, nx, bs, hpGamma_0[ii], cNnx);

//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nx, nx, bs, hpGamma_0_Q[ii], cNnx);

		printf("\nH_Q\n");
		d_print_pmat(nx, nx, bs, pH_Q[0], cnx);
#endif

#if 0
//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u[ii], cNnx);

//		for(ii=0; ii<N; ii++)	
//			d_print_pmat(nu*(ii+1), nx, bs, hpGamma_u_Q[ii], cNnx);

//		printf("\nH_R\n");
		d_print_pmat(N*nu, N*nu, bs, pH_R[0], cNnu);
#endif

#if 0
		printf("\nH_S\n");
		d_print_pmat(nx, N*nu, bs, pH_St[0], cNnu);
#endif

#if 0
		printf("\nH_q\n");
		d_print_mat(1, nx, H_q[0], 1);
#endif

#if 0
//		printf("\nGamma_b\n");
//		for(ii=0; ii<N; ii++)	
//			d_print_mat(1, nx, hGamma_b[ii], 1);

//		printf("\nGamma_b_q\n");
//		for(ii=0; ii<N; ii++)	
//			d_print_mat(1, nx, hGamma_b_q[ii], 1);

		printf("\nH_r\n");
		d_print_mat(1, N*nu, H_r[0], 1);
//		exit(1);
#endif

#if 0
		printf("\nH_A\n");
		d_print_pmat(nx, nx, bs, pH_A[0], cnx);
#endif

#if 0
		printf("\nH_B\n");
		d_print_pmat(nx, N*nu, bs, pH_B[0], cNnu);
#endif

#if 0
		printf("\nH_b\n");
		d_print_mat(1, nx, H_b[0], 1);
#endif

//exit(1);



		gettimeofday(&tv0, NULL); // start

//		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			// Cholesky factorization and solution
			dpotrf_lib(N*nu, N*nu, pH_R[0], cNnu, pL_R, cNnu, diag);

			dax_mat(N*nu, 1, -1.0, H_r[0], 1, H_u, 1);

#if 0
			for(ii=0; ii<N*nu; ii++)
				pL_R[ii/bs*bs*cNnu+ii%bs+ii*bs] = diag[ii];

			dtrsv_n_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
			dtrsv_t_lib(N*nu, N*nu, 1, pL_R, cNnu, H_u);
#else
			dtrsv_n_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
			dtrsv_t_lib(N*nu, N*nu, 0, pL_R, cNnu, H_u);
#endif
			
			for(jj=0; jj<N; jj++)
				{
				dgemv_n_lib(nx, nx, pA, cnx, hx[jj], hb[jj], hx[jj+1], 1);
				d_copy_mat(nu, 1, H_u+jj*nu, 1, hu[jj], 1);
				dgemv_n_lib(nx, nu, pB, cnu, hu[jj], hx[jj+1], hx[jj+1], 1);
				}

			}

		gettimeofday(&tv1, NULL); // start

		double time_fact_sol = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if 0
		d_print_pmat(N*nu, N*nu, bs, pL_R, cNnu);
#endif

#if 0
		d_print_mat(1, N*nu, H_u, 1);
#endif

#if 0
		for(jj=0; jj<N; jj++)
			d_print_mat(1, nu, hu[jj], 1);

		for(jj=0; jj<=N; jj++)
			d_print_mat(1, nx, hx[jj], 1);
#endif


#if 0

		double *pZ; d_zeros_align(&pZ, pnx, cnx);
		double *ptr_temp;

		//printf("\n%d\n", d_cond_lqcp_work_space(N, nx, nu, N2) );
		//exit(1);
		double *work_space_part_cond; d_zeros_align(&work_space_part_cond, d_cond_lqcp_work_space(N, nx, nu, N2), 1);

		nx_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) nx_v[ii] = nx;

		nu_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) nu_v[ii] = N1*nu;
#if 1
		nu_v[N2] = 0;
#endif
		
		int nzero_S = 0;
		int diag_Q = 1;
		int N2_cond = 0;

		d_set_pmat(pNnu, cNnu, 0.0, bs, pH_R[0], cNnu); // TODO remove !!!!!


		gettimeofday(&tv0, NULL); // start

		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

#if 1
			d_cond_lqcp(N, nx, nu, hpA, hpAt, hpBt, hb, hpR, nzero_S, hpS, diag_Q, hdQ, hr, hq, N2, nx_v, nu_v, pH_A, pH_B, H_b, pH_R, pH_St, pH_Q, H_r, H_q, work_space_part_cond, N2_cond);
#else
			for(jj=0; jj<N2; jj++)
				{

				//ptr_temp = hdQ[jj+N1];
				//hdQ[jj+N1] = pZ;

				d_cond_A(N1, nx, nu, hpA+jj, 1, hpGamma_0, pH_A[jj]);

				d_cond_B(N1, nx, nu, hpA+jj, hpBt+jj, 1, hpGamma_u, pH_B[jj]);

				d_cond_b(N1, nx, nu, hpA+jj, hb+jj, 1, hGamma_b, H_b[jj]);

				//d_cond_Q(N1, nx, nu, hpA+jj, 0, 0, hpQ+jj, hpL+jj, 0, hpGamma_0, hpGamma_0_Q, pH_Q[jj], work);
				d_cond_Q(N1, nx, nu, hpA+jj, 1, 0, hdQ+jj, hpL+jj, 0, hpGamma_0, hpGamma_0_Q, pH_Q[jj], work);
				
				//d_cond_R(N1, nx, nu, 0, hpA+jj, hpAt+jj, hpBt+jj, 0, 0, hpQ+jj, hpL+jj, 0, hpS+jj, hpR+jj, pD, pM, 0, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[jj]);
				d_cond_R(N1, nx, nu, 0, hpA+jj, hpAt+jj, hpBt+jj, 1, 0, hdQ+jj, hpL+jj, 0, hpS+jj, hpR+jj, pD, pM, 0, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, pH_R[jj]);

				//d_cond_St(N1, nx, nu, 0, hpS+jj, 0, hpGamma_0, 1, hpGamma_0_Q, hpGamma_u_Q, pH_St[jj]);
				d_cond_St(N1, nx, nu, 0, hpS+jj, 0, hpGamma_0, 0, hpGamma_0_Q, hpGamma_u_Q, pH_St[jj]);

				d_cond_q(N1, nx, nu, hpA+jj, hb+jj, 1, 0, hdQ+jj, hq+jj, hpGamma_0, 0, hGamma_b, 1, hGamma_b_q, H_q[jj]);

				d_cond_r(N1, nx, nu, hpA+jj, hb+jj, 1, 0, hdQ+jj, 1, hpS+jj, hq+jj, hr+jj, hpGamma_u, 0, hGamma_b, 0, hGamma_b_q, H_r[jj]);

				//hdQ[jj+N1] = ptr_temp;

				}
#endif

			}

		gettimeofday(&tv1, NULL); // start

		double time_part_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		
#if 1
		for(jj=0; jj<N2+1; jj++)
			d_print_pmat(nx, nx, bs, pH_Q[jj], cnx);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(nx, N1*nu, bs, pH_St[jj], cN1nu);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(N1*nu, N1*nu, bs, pH_R[jj], cN1nu);

		for(jj=0; jj<N2+1; jj++)
			d_print_mat(1, nx, H_q[jj], 1);

		for(jj=0; jj<N2; jj++)
			d_print_mat(1, N1*nu, H_r[jj], 1);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(nx, nx, bs, pH_A[jj], cnx);

		for(jj=0; jj<N2; jj++)
			d_print_pmat(nx, N1*nu, bs, pH_B[jj], cN1nu);

		for(jj=0; jj<N2; jj++)
			d_print_mat(1, nx, H_b[jj], 1);
#endif


		int nu0, nx0, nx1, cnu0, pnx0, pnx1, cnx0, cnx1, pnz0, cnz0, cnl0;

		double *(pH_BAbt[N2]);
		double *(pH_RSQrq[N2+1]);
		double *(pH_L[N2+1]);
		double *(H_ux[N2+1]);
		double *(H_pi[N2+1]);
		double *(H_rq[N2+1]);
		double *(H_res_rq[N2+1]);
		double *(H_res_b[N2]);

		for(ii=0; ii<N2; ii++)
			{
			pnx0 = (nx_v[ii]+bs-1)/bs*bs;
			pnx1 = (nx_v[ii+1]+bs-1)/bs*bs;
			cnx0 = (nx_v[ii]+ncl-1)/ncl*ncl;
			cnx1 = (nx_v[ii+1]+ncl-1)/ncl*ncl;
			pnz0 = (nx_v[ii]+nu_v[ii]+1+bs-1)/bs*bs;
			cnz0 = (nx_v[ii]+nu_v[ii]+1+ncl-1)/ncl*ncl;
			cnl0 = cnz0<cnx0+ncl ? cnx0+ncl : cnz0;
			d_zeros_align(&pH_BAbt[ii], pnz0, cnx1);
			d_zeros_align(&pH_RSQrq[ii], pnz0, cnz0);
			d_zeros_align(&pH_L[ii], pnz0, cnl0);
			d_zeros_align(&H_ux[ii], pnz0, 1);
			d_zeros_align(&H_pi[ii], pnx0, 1);
			d_zeros_align(&H_rq[ii], pnz0, 1);
			d_zeros_align(&H_res_rq[ii], pnz0, 1);
			d_zeros_align(&H_res_b[ii], pnx1, 1);
			}
		ii = N2;
		pnx0 = (nx_v[ii]+bs-1)/bs*bs;
		pnz0 = (nx_v[ii]+nu_v[ii]+1+bs-1)/bs*bs;
		cnz0 = (nx_v[ii]+nu_v[ii]+1+ncl-1)/ncl*ncl;
		cnl0 = cnz0<cnx0+ncl ? cnx0+ncl : cnz0;
		d_zeros_align(&pH_RSQrq[N2], pnz0, cnz0);
		d_zeros_align(&pH_L[N2], pnz0, cnl0);
		d_zeros_align(&H_ux[N2], pnz0, 1);
		d_zeros_align(&H_pi[N2], pnx0, 1);
		d_zeros_align(&H_rq[N2], pnz0, 1);
		d_zeros_align(&H_res_rq[N2], pnz0, 1);

		
		for(ii=0; ii<N2; ii++)
			{
			nx0 = nx_v[ii];
			nx1 = nx_v[ii+1];
			nu0 = nu_v[ii];
			cnx0 = (nx0+ncl-1)/ncl*ncl;
			cnx1 = (nx1+ncl-1)/ncl*ncl;
			cnu0 = (nu0+ncl-1)/ncl*ncl;
			dgetr_lib(nx1, nu0, 0, pH_B[ii], cnu0, 0, pH_BAbt[ii], cnx1);
			dgetr_lib(nx1, nx0, 0, pH_A[ii], cnx0, nu0, pH_BAbt[ii]+nu0/bs*bs*cnx1+nu0%bs, cnx1);
			dgetr_lib(nx1, 1, 0, H_b[ii], 1, nu0+nx0, pH_BAbt[ii]+(nu0+nx0)/bs*bs*cnx1+(nu0+nx0)%bs, cnx1);
//			d_print_pmat(nx0+nu0+1, nx1, bs, pH_BAbt[ii], cnx1);
			}

		for(ii=0; ii<=N2; ii++)
			{
			nx0 = nx_v[ii];
			nu0 = nu_v[ii];
			cnu0 = (nu0+ncl-1)/ncl*ncl;
			cnx0 = (nx0+ncl-1)/ncl*ncl;
			cnz0 = (nu0+nx0+1+ncl-1)/ncl*ncl;
			dgecp_lib(nu0, nu0, 0, pH_R[ii], cnu0, 0, pH_RSQrq[ii], cnz0);
			dgecp_lib(nx0, nu0, 0, pH_St[ii], cnu0, nu0, pH_RSQrq[ii]+(nu0)/bs*bs*cnz0+(nu0)%bs, cnz0);
			dgecp_lib(nx0, nx0, 0, pH_Q[ii], cnx0, nu0, pH_RSQrq[ii]+(nu0)/bs*bs*cnz0+(nu0)%bs+(nu0)*bs, cnz0);
			dgetr_lib(nu0, 1, 0, H_r[ii], 1, nu0+nx0, pH_RSQrq[ii]+(nu0+nx0)/bs*bs*cnz0+(nu0+nx0)%bs, cnz0);
			dgetr_lib(nx0, 1, 0, H_q[ii], 1, nu0+nx0, pH_RSQrq[ii]+(nu0+nx0)/bs*bs*cnz0+(nu0+nx0)%bs+(nu0)*bs, cnz0);
//			d_print_pmat(nx0+nu0+1, nx0+nu0+1, bs, pH_RSQrq[ii], cnz0);
			}
#if 0
		ii = N2;
		nx0 = nx_v[ii];
		cnx0 = (nx0+ncl-1)/ncl*ncl;
		cnz0 = (nx0+1+ncl-1)/ncl*ncl;
		dgecp_lib(nx0, nx0, 0, pH_Q[N2], cnx0, 0, pH_RSQrq[N2], cnz0);
		dgetr_lib(nx0, 1, 0, H_q[N2], 1, nx0, pH_RSQrq[N2]+nx0/bs*bs*cnz0+nx0%bs, cnz0);
		d_print_pmat(nx0+1, nx0+1, bs, pH_RSQrq[N2], cnz0);
#endif



		nb_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) nb_v[ii] = 0;

		ng_v = (int *) malloc((N2+1)*sizeof(int));
		for(ii=0; ii<=N2; ii++) ng_v[ii] = 0;

		int nz_m = 0;
		for(ii=0; ii<=N2; ii++)
			nz_m = nu_v[ii]+nx_v[ii]+1>nz_m ? nu_v[ii]+nx_v[ii]+1 : nz_m ;

		double *work_tv; d_zeros_align(&work_tv, d_ric_sv_mpc_tv_work_space_size_double(N2, nx_v, nu_v, nb_v, ng_v), 1);
		double *diag_tv; d_zeros_align(&diag_tv, (nz_m+bs-1)/bs*bs, 1);

#if 1
		for(ii=0; ii<=N2; ii++)
			printf("\n%d\t%d\n", nu_v[ii], nx_v[ii]);
		//exit(1);
#endif




		gettimeofday(&tv0, NULL); // start

		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

#if 0
			d_ric_sv_mpc(nx, N1*nu, N2, pH_BAbt, pH_RSQrq, 0, dummy, dummy, H_ux, pH_L, work1, diag_tv, 0, dummy, 0, 0, 0, dummy, dummy, dummy, 0);
#else
			d_ric_sv_mpc_tv(N2, nx_v, nu_v, pH_BAbt, pH_RSQrq, H_ux, pH_L, work_tv, diag_tv, 0, dummy, 0, H_pi, nb_v, 0, dummy, dummy, ng_v, dummy, dummy, dummy, 0);
#endif

			}

		gettimeofday(&tv1, NULL); // start

		double time_part_cond_ric = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		
#if 0
		printf("\nux\n");
		for(jj=0; jj<=N2; jj++)
			d_print_mat(1, nu_v[jj]+nx_v[jj], H_ux[jj], 1);

		printf("\npi\n");
		for(jj=1; jj<=N2; jj++)
			d_print_mat(1, nx_v[jj], H_pi[jj], 1);

		printf("\nu\n");
		for(jj=0; jj<N2; jj++)
			for(ii=0; ii<nu_v[jj]/nu; ii++)
				d_print_mat(1, nu, H_ux[jj]+ii*nu, 1);
#endif

#if 0
		printf("\nrq\n");
		for(jj=0; jj<=N2; jj++)
			{
			nu0 = nu_v[jj];
			nx0 = nx_v[jj];
			d_copy_mat(nu0, 1, H_r[jj], 1, H_rq[jj], 1);
			d_copy_mat(nx0, 1, H_q[jj], 1, H_rq[jj]+nu0, 1);
			d_print_mat(1, nx0+nu0, H_rq[jj], 1);
			}

		d_res_mpc_tv(N2, nx_v, nu_v, pH_BAbt, pH_RSQrq, H_rq, H_ux, H_pi, H_res_rq, H_res_b);

		printf("\nresiduals\n");
		for(jj=0; jj<=N2; jj++)
			d_print_mat(1, nu_v[jj]+nx_v[jj], H_res_rq[jj], 1);

		for(jj=0; jj<N2; jj++)
			d_print_mat(1, nx_v[jj+1], H_res_b[jj], 1);
		printf("\n");

	
#endif


		d_copy_mat(nx, 1, x0, nx, hux[0]+nu, nx);

		gettimeofday(&tv0, NULL); // start

		nrep = 100000;
		for(rep=0; rep<nrep; rep++)
			{

			d_ric_sv_mpc(nx, nu, N, hpBAbt, hpRSQrq, 0, dummy, dummy, hux, hpL2, work2, diag2, 0, dummy, 0, dummy, 0, 0, 0, dummy, dummy, dummy, 0);

			}

#if 0
		printf("\nux\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, nu+nx, hux[ii], 1);
#endif

		gettimeofday(&tv1, NULL); // start

		double time_ric = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	
#endif

//		printf("\ntime condensing = %e seconds\n", time_cond);
//		printf("\ntime factorization & solution = %e seconds\n", time_fact_sol);
//		printf("\ntime partial condensing = %e seconds\n", time_part_cond);
//		printf("\ntime partial condensing riccati = %e seconds\n", time_part_cond_ric);
//		printf("\ntime full riccati = %e seconds\n", time_ric);
//		printf("\n\n");

		printf("\n%d %d %d %d %e %e %e %e %e %e %e %e %e %e\n", nx, nu, N, nrep, time_N3_cond, time_N3_cond_L, time_N2_cond, time_N2_nx3_cond, time_N3_fact, time_N_fact, time_N3_cond+time_N3_fact, time_N2_cond+time_N3_fact, time_N2_cond_fact, time_N2_nx3_cond_fact);
		fprintf(f, "\n%d %d %d %d %e %e %e %e %e %e %e %e %e %e\n", nx, nu, N, nrep, time_N3_cond, time_N3_cond_L, time_N2_cond, time_N2_nx3_cond, time_N3_fact, time_N_fact, time_N3_cond+time_N3_fact, time_N2_cond+time_N3_fact, time_N2_cond_fact, time_N2_nx3_cond_fact);

/************************************************
* return
************************************************/

		free(nx_v);
		free(nu_v);
		free(nb_v);
		free(ng_v);

		free(A);
		free(B);
		free(b);
		free(x0);
		free(Q);
		free(pA);
		free(pAt);
		free(pB);
		free(pBt);
		free(pBAbt);
		free(pBAt);
		free(pQ);
		free(dQ);
		free(pR);
		free(dR);
		free(pS);
		free(q);
		free(r);
		free(pRSQrq);
		free(pRSQ);
		free(dRSQ);
		free(pD);
		free(pM);
		free(pQs);
		free(pLam);
		free(pL);
		free(pBAtL);
		for(ii=0; ii<N2; ii++)
			{
			free(pH_A[ii]);
			free(pH_B[ii]);
			free(H_b[ii]);
			free(pH_R[ii]);
			free(pH_Q[ii]);
			free(pH_St[ii]);
			free(H_q[ii]);
			free(H_r[ii]);

#if 0
			free(pH_BAbt[ii]);
			free(pH_RSQrq[ii]);
#endif
			}
#if 0
		free(pH_RSQrq[N2]);
#endif

		free(pL_R);
		free(H_u);
		free(work);
		//free(pGamma_u);
		//free(pGamma_u_Q);
		//free(pGamma_u_Q_A);
		//free(pGamma_0);
		//free(pGamma_0_Q);
		free(diag);
		free(diag_ric);
#if 0
		free(diag_tv);
		free(work_tv);
#endif
		free(diag2);
		free(work2);
#if 0
		free(work_space_part_cond);
#endif

		for(ii=0; ii<N; ii++)
			{
			free(hpGamma_u[ii]);
			free(hpGamma_u_Q[ii]);
			free(hpGamma_u_Q_A[ii]);
			free(hpGamma_0[ii]);
			free(hpGamma_0_Q[ii]);
			free(hGamma_b[ii]);
			free(hGamma_b_q[ii]);
			free(hpL[ii]);
			free(hx[ii]);
			free(hu[ii]);
			free(hpL2[ii]);
			free(hux[ii]);
			}
		free(hpL[N]);
		free(hx[N]);
		free(hpL2[N]);
		free(hux[N]);

		} // increase size

	fprintf(f, "];\n");
	fclose(f);


	return 0;

	}


#endif
