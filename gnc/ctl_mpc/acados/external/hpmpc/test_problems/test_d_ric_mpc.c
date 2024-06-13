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

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>
#endif

#include "../include/aux_d.h"
#include "../include/blas_d.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../include/block_size.h"
#include "../include/reference_code.h"
#include "tools.h"


#define PRINTRES 1
#define COMPUTE_MULT 1



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

	printf("Riccati solver performance test - double precision\n");
	printf("\n");

	// maximum frequency of the processor
	const float GHz_max = 3.6;
	printf("Frequency used to compute theoretical peak: %5.1f GHz (edit test problem to modify this value).\n", GHz_max);
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
#if 0
	int ll_max = 77;
#else
	int ll_max = 1;
#endif
	for(ll=0; ll<ll_max; ll++)
		{
		

		if(ll_max==1)
			{
			nx = 8; // number of states (it has to be even for the mass-spring system test problem)
			nu = 3; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = 10; // horizon lenght
			nrep = 1000;
			//nx = 25;
			//nu = 1;
			//N = 11;
			}
		else
			{
			nx = nn[ll]; // number of states (it has to be even for the mass-spring system test problem)
			nu = nx/2; //2; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = 10; // horizon lenght
			nrep = 2*nnrep[ll];
//			nrep = nnrep[ll]/4;
			}

		

		int pnu = (nu+bs-1)/bs*bs;
		int pnx = (nx+bs-1)/bs*bs;
		int cnx = (nx+ncl-1)/ncl*ncl;


#define MHE 0


		// define time-varian problem size
		int nx_v[N+1];
#if MHE==1
		nx_v[0] = nx;
#else
		nx_v[0] = 0;
#endif
		for(ii=1; ii<N; ii++) nx_v[ii] = nx;
		nx_v[N] = nx;

		int nu_v[N+1];
		nu_v[0] = nu;
		for(ii=1; ii<N; ii++) nu_v[ii] = nu;
		nu_v[N] = 0; // XXX

		int nb_v[N+1];
		for(ii=0; ii<=N; ii++) nb_v[ii] = 0;

		int ng_v[N+1];
		for(ii=0; ii<=N; ii++) ng_v[ii] = 0;


		int pnx_v[N+1]; for(ii=0; ii<=N; ii++) pnx_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
		int pnux_v[N+1]; for(ii=0; ii<=N; ii++) pnux_v[ii] = (nu_v[ii]+nx_v[ii]+bs-1)/bs*bs;
		int pnz_v[N+1]; for(ii=0; ii<=N; ii++) pnz_v[ii] = (nu_v[ii]+nx_v[ii]+1+bs-1)/bs*bs;
		int cnx_v[N+1]; for(ii=0; ii<=N; ii++) cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
		int cnux_v[N+1]; for(ii=0; ii<=N; ii++) cnux_v[ii] = (nu_v[ii]+nx_v[ii]+ncl-1)/ncl*ncl;


/************************************************
* dynamical system
************************************************/	

		double *A; d_zeros(&A, nx, nx); // states update matrix

		double *B; d_zeros(&B, nx, nu); // inputs matrix

		double *b; d_zeros_align(&b, nx, 1); // states offset
		double *x0; d_zeros(&x0, nx, 1); // initial state

		double Ts = 0.5; // sampling time
		mass_spring_system(Ts, nx, nu, N, A, B, b, x0);
	
		for(jj=0; jj<nx; jj++)
			b[jj] = 0.1;
	
		for(jj=0; jj<nx; jj++)
			x0[jj] = 0;
		x0[0] = 2.5;
		x0[1] = 2.5;
	
#if MHE!=1
		double *pA; d_zeros_align(&pA, pnx, cnx); // XXX pnx_v[1] !!!!! (pnx_v[0]=0)
		d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);
		double *b0; d_zeros_align(&b0, pnx, 1);
#if defined(BLASFEO)
		dgemv_n_lib(nx, nx, 1.0, pA, cnx, x0, 1.0, b, b0);
#else
		dgemv_n_lib(nx, nx, pA, cnx, x0, 1, b, b0);
#endif
#ifdef BLASFEO
		d_print_pmat(nx, nx, pA, pnx);
#else
		d_print_pmat(nx, nx, bs, pA, pnx);
#endif
		d_print_mat(1, nx, x0, 1);
		d_print_mat(1, nx, b, 1);
		d_print_mat(1, nx, b0, 1);

		double *pBAbt0; d_zeros_align(&pBAbt0, pnz_v[0], cnx_v[1]);
		d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBAbt0, cnx_v[1]);
		d_cvt_tran_mat2pmat(nx, 1, b0, nx, nu_v[0], pBAbt0+nu_v[0]/bs*bs*cnx_v[1]+nu_v[0]%bs, cnx_v[1]);
#endif

		double *pBAbt1; 
		if(N>1)
			{
			d_zeros_align(&pBAbt1, pnz_v[1], cnx_v[2]);
			d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBAbt1, cnx_v[2]);
			d_cvt_tran_mat2pmat(nx, nx, A, nx, nu_v[1], pBAbt1+nu_v[1]/bs*bs*cnx_v[2]+nu_v[1]%bs, cnx_v[2]);
			d_cvt_tran_mat2pmat(nx, 1, b, nx, nu_v[1]+nx_v[1], pBAbt1+(nu_v[1]+nx_v[1])/bs*bs*cnx_v[2]+(nu_v[1]+nx_v[1])%bs, cnx_v[2]);
			}

/************************************************
* cost function
************************************************/	

		double *Q; d_zeros(&Q, nx, nx);
		for(ii=0; ii<nx; ii++) Q[ii*(nx+1)] = 1.0;

		double *R; d_zeros(&R, nu, nu);
		for(ii=0; ii<nu; ii++) R[ii*(nu+1)] = 2.0;

		double *S; d_zeros(&S, nu, nx);

		double *q; d_zeros(&q, nx, 1);
		for(ii=0; ii<nx; ii++) q[ii] = 0.1;

		double *r; d_zeros(&r, nu, 1);
		for(ii=0; ii<nu; ii++) r[ii] = 0.2;

#if MHE!=1
		double *pS; d_zeros_align(&pS, pnu, cnx);
		d_cvt_mat2pmat(nu, nx, S, nu, 0, pS, cnx);
		double *r0; d_zeros_align(&r0, pnu, 1);
#if defined(BLASFEO)
		dgemv_n_lib(nu, nx, 1.0, pS, cnx, x0, 1.0, r, r0);
#else
		dgemv_n_lib(nu, nx, pS, cnx, x0, 1, r, r0);
#endif

		double *pQ0; d_zeros_align(&pQ0, pnz_v[0], cnux_v[0]);
		d_cvt_mat2pmat(nu, nu, R, nu, 0, pQ0, cnux_v[0]);
		d_cvt_tran_mat2pmat(nu, 1, r0, nu, nu_v[0], pQ0+nu_v[0]/bs*bs*cnux_v[0]+nu_v[0]%bs, cnux_v[0]);
		double *q0; d_zeros_align(&q0, pnux_v[0], 1);
		d_copy_mat(nu, 1, r0, nu, q0, pnux_v[0]);
#endif

		double *pQ1; 
		double *q1; 
		if(N>1)
			{
			d_zeros_align(&pQ1, pnz_v[1], cnux_v[1]);
			d_cvt_mat2pmat(nu, nu, R, nu, 0, pQ1, cnux_v[1]);
			d_cvt_tran_mat2pmat(nu, nx, S, nu, nu_v[1], pQ1+nu_v[1]/bs*bs*cnux_v[1]+nu_v[1]%bs, cnux_v[1]);
			d_cvt_tran_mat2pmat(nu, 1, r, nu, nu_v[1]+nx_v[1], pQ1+(nu_v[1]+nx_v[1])/bs*bs*cnux_v[1]+(nu_v[1]+nx_v[1])%bs, cnux_v[1]);
			d_cvt_mat2pmat(nx, nx, Q, nx, nu_v[1], pQ1+nu_v[1]/bs*bs*cnux_v[1]+nu_v[1]%bs+nu*bs, cnux_v[1]);
			d_cvt_tran_mat2pmat(nx, 1, q, nx, nu_v[1]+nx_v[1], pQ1+(nu_v[1]+nx_v[1])/bs*bs*cnux_v[1]+(nu_v[1]+nx_v[1])%bs+nu_v[1]*bs, cnux_v[1]);
			d_zeros_align(&q1, pnux_v[1], 1);
			d_copy_mat(nu, 1, r, nu, q1, pnux_v[1]);
			d_copy_mat(nx, 1, q, nx, q1+nu_v[1], pnux_v[1]);
			}

		double *pQN; d_zeros_align(&pQN, pnz_v[N], cnux_v[N]);
		d_cvt_mat2pmat(nx, nx, Q, nx, 0, pQN, cnux_v[N]);
		d_cvt_tran_mat2pmat(nx, 1, q, nx, nx_v[N], pQN+(nx_v[N])/bs*bs*cnux_v[N]+(nx_v[N])%bs, cnux_v[N]);
		double *qN; d_zeros_align(&qN, pnx_v[N], 1);
		d_copy_mat(nx, 1, q, nx, qN, pnx_v[N]);


/************************************************
* work space
************************************************/	

		double *hpBAbt[N];
		double *hb[N];
		double *hpQ[N+1];
		double *hq[N+1];
		double *hux[N+1];
		double *hpi[N+1];
		double *hPb[N];
		double *hrb[N];
		double *hrq[N+1];
#if MHE!=1
		hpBAbt[0] = pBAbt0;
		hb[0] = b0;
		hpQ[0] = pQ0;
		hq[0] = q0;
#else
		hpBAbt[0] = pBAbt1;
		hb[0] = b;
		hpQ[0] = pQ1;
		hq[0] = q1;
#endif
		d_zeros_align(&hux[0], pnux_v[0], 1);
		d_zeros_align(&hpi[0], pnx_v[0], 1);
		d_zeros_align(&hPb[0], pnx_v[1], 1);
		d_zeros_align(&hrb[0], pnx_v[1], 1);
		d_zeros_align(&hrq[0], pnz_v[0], 1);
		for(ii=1; ii<N; ii++)
			{
			hpBAbt[ii] = pBAbt1;
			hb[ii] = b;
			hpQ[ii] = pQ1;
			hq[ii] = q1;
			d_zeros_align(&hux[ii], pnux_v[ii], 1);
			d_zeros_align(&hpi[ii], pnx_v[ii], 1);
			d_zeros_align(&hrb[ii], pnx_v[ii+1], 1);
			d_zeros_align(&hPb[ii], pnx_v[ii+1], 1);
			d_zeros_align(&hrq[ii], pnz_v[ii], 1);
			}
		hpQ[N] = pQN;
		hq[N] = qN;
		d_zeros_align(&hux[N], pnx_v[N], 1);
		d_zeros_align(&hpi[N], pnx_v[N], 1);
		d_zeros_align(&hrq[N], pnz_v[N], 1);

		double *work; d_zeros_align(&work, d_back_ric_rec_sv_tv_work_space_size_bytes(N, nx_v, nu_v, nb_v, ng_v)/sizeof(double), 1);
		double *memory; d_zeros_align(&memory, d_back_ric_rec_sv_tv_memory_space_size_bytes(N, nx_v, nu_v, nb_v, ng_v)/sizeof(double), 1);


/************************************************
* backward riccati recursion solver
************************************************/
		
		// timing 
		struct timeval tv0, tv1, tv2, tv3;
		int rep;

		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			d_back_ric_rec_sv_tv_res(N, nx_v, nu_v, nb_v, 0, ng_v, 0, hpBAbt, dummy, 0, hpQ, dummy, dummy, dummy, dummy, dummy, hux, COMPUTE_MULT, hpi, 0, dummy, memory, work);
			}

		gettimeofday(&tv1, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			d_back_ric_rec_trf_tv_res(N, nx_v, nu_v, nb_v, 0, ng_v, hpBAbt, hpQ, dummy, dummy, dummy, memory, work);
			}

		gettimeofday(&tv2, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			d_back_ric_rec_trs_tv_res(N, nx_v, nu_v, nb_v, 0, ng_v, hpBAbt, hb, hq, dummy, dummy, hux, 1, hpi, 1, hPb, memory, work);
			}

		gettimeofday(&tv3, NULL); // start




//		for(ii=0; ii<=N; ii++)
//			printf("\n%d %d\n", nu_v[ii], nx_v[ii]);

		if(PRINTRES==1 && ll_max==1)
			{
			/* print result */
			printf("\n\nux\n\n");
			for(ii=0; ii<=N; ii++)
				d_print_mat(1, nu_v[ii]+nx_v[ii], hux[ii], 1);
			printf("\npi\n\n");
			for(ii=0; ii<N; ii++)
				d_print_mat(1, nx, hpi[ii], 1);
//			exit(1);
			}

		float Gflops_max = flops_max * GHz_max;

		// TODO check flop counts !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		float time_sv = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		float flop_sv = (1.0/3.0*nx*nx*nx+3.0/2.0*nx*nx) + N*(7.0/3.0*nx*nx*nx+4.0*nx*nx*nu+2.0*nx*nu*nu+1.0/3.0*nu*nu*nu+13.0/2.0*nx*nx+9.0*nx*nu+5.0/2.0*nu*nu) - (nx*(nx+nu)+1.0/3.0*nx*nx*nx+3.0/2.0*nx*nx);
		if(COMPUTE_MULT==1)
			flop_sv += N*2*nx*nx;
		float Gflops_sv = 1e-9*flop_sv/time_sv;
	
		float time_trf = (float) (tv2.tv_sec-tv1.tv_sec)/(nrep+0.0)+(tv2.tv_usec-tv1.tv_usec)/(nrep*1e6);
		float flop_trf = (1.0/3.0*nx*nx*nx) + (N-1)*(7.0/3.0*nx*nx*nx+4.0*nx*nx*nu+2.0*nx*nu*nu+1.0/3.0*nu*nu*nu) + (1.0*nx*nx*nu+1.0*nx*nu*nu+1.0/3.0*nu*nu*nu);
		float Gflops_trf = 1e-9*flop_trf/time_trf;
	
		float time_trs = (float) (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);
		float flop_trs = N*(6*nx*nx+8.0*nx*nu+2.0*nu*nu);
		if(COMPUTE_MULT==1)
			flop_trs += N*2*nx*nx;
		float Gflops_trs = 1e-9*flop_trs/time_trs;
	
		if(ll==0)
			{
			printf("\nnx\tnu\tN\tsv time\t\tsv Gflops\tsv %%\t\ttrf time\ttrf Gflops\ttrf %%\t\ttrs time\ttrs Gflops\ttrs %%\n\n");
			}
		printf("%d\t%d\t%d\t%e\t%f\t%f\t%e\t%f\t%f\t%e\t%f\t%f\n", nx, nu, N, time_sv, Gflops_sv, 100.0*Gflops_sv/Gflops_max, time_trf, Gflops_trf, 100.0*Gflops_trf/Gflops_max, time_trs, Gflops_trs, 100.0*Gflops_trs/Gflops_max);

/************************************************
* free memory TODO
************************************************/

		} // increase size

	fprintf(f, "];\n");
	fclose(f);


	return 0;

	}



