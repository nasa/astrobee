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
#include "../include/reference_code.h"
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
	

	printf("\n");
	printf("Tested solvers:\n");
	printf("-sv : Riccati factorization and system solution (prediction step in IP methods)\n");
	printf("-trs: system solution after a previous call to Riccati factorization (correction step in IP methods)\n");
	printf("\n");
	printf("\nTest for linear time-invariant systems\n");
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
	
	
	// problem size
	int nx = NX; // number of states (it has to be even for the mass-spring system test problem)
	int nu = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
	int N  = NN; // horizon lenght

	// number of problem instances solution for better timing
	int nrep = NREP; 

#if 0

	int N_v[N+1];
	int nx_v[N+1];
	int nu_v[N+1];
	int nb_v[N+1]; for(ii=0; ii<=N; ii++) nb_v[ii] = 0;
	int ng_v[N+1]; for(ii=0; ii<=N; ii++) ng_v[ii] = 0;


	int ll;
	for(ll=0; ll<N; ll++)
		{


		int Np = ll+1; // horizon length in the partially condensed problem

		int Nc = N/Np; // minimum horizon length within each stage group

		int rN = N - Np*Nc; // number of stage groups with horizon Nc+1 


		ii = 0;
		for(; ii<rN; ii++)
			N_v[ii] = Nc+1;
		for(; ii<Np; ii++)
			N_v[ii] = Nc;

//		printf("\n\t%d:\t", Np);
//		for(ii=0; ii<Np; ii++)
//			printf("%d\t", N_v[ii]);
//		printf("\n\n");

		nx_v[0] = 0;
		for(ii=1; ii<=Np; ii++)
			nx_v[ii] = nx;

//		printf("\n\t%d:\t", Np);
//		for(ii=0; ii<=Np; ii++)
//			printf("%d\t", nx_v[ii]);
//		printf("\n\n");

		for(ii=0; ii<Np; ii++)
			nu_v[ii] = nu*N_v[ii];
		nu_v[Np] = 0;

//		printf("\n\t%d:\t", Np);
//		for(ii=0; ii<=Np; ii++)
//			printf("%d\t", nu_v[ii]);
//		printf("\n\n");
		


		// matrix size
		int pnux_v[Np+1];
		int cnx_v[Np+1];
		int cnux_v[Np+1];
		for(ii=0; ii<=Np; ii++) 
			{
			pnux_v[ii] = (nu_v[ii]+nx_v[ii]+bs-1)/bs*bs;
			cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
			cnux_v[ii] = (nu_v[ii]+nx_v[ii]+ncl-1)/ncl*ncl;
			}
		


		// dynamic system matrices
		double *pBA0;
		double *pBA1;
		double *pBANm1;

		d_zeros_align(&pBA0, pnux_v[0], cnx_v[1]);
		if(Np>1) 
			d_zeros_align(&pBA1, pnux_v[1], cnx_v[2]);
		if(Np>1)
			d_zeros_align(&pBANm1, pnux_v[Np-1], cnx_v[Np]);

		double *(hpBA[Np]);
		hpBA[0] = pBA0;
		for(ii=1; ii<rN; ii++) hpBA[ii] = pBA1;
		for(; ii<Np; ii++) hpBA[ii] = pBANm1;
//		for(ii=0; ii<Np; ii++)
//			d_print_pmat(nu_v[ii]+nx_v[ii], nx_v[ii+1], bs, hpBA[ii], cnx_v[ii]);


		// cost function matrices
		double *pQ0;
		double *pQ1;
		double *pQNm1;
		double *pQN;

		d_zeros_align(&pQ0, pnux_v[0], cnux_v[0]);
		for(ii=0; ii<nu_v[0]+nx_v[0]; ii++)
			pQ0[ii/bs*bs*cnux_v[0]+ii%bs+ii*bs] = 1.0;
//		d_print_pmat(nu_v[0]+nx_v[0], nu_v[0]+nx_v[0], bs, pQ0, cnux_v[0]);
		if(Np>1)
			{
			d_zeros_align(&pQ1, pnux_v[1], cnux_v[1]);
			for(ii=0; ii<nu_v[1]+nx_v[1]; ii++)
				pQ1[ii/bs*bs*cnux_v[1]+ii%bs+ii*bs] = 1.0;
//			d_print_pmat(nu_v[1]+nx_v[1], nu_v[1]+nx_v[1], bs, pQ1, cnux_v[1]);
			}
		if(Np>1)
			{
			d_zeros_align(&pQNm1, pnux_v[Np-1], cnux_v[Np-1]);
			for(ii=0; ii<nu_v[Np-1]+nx_v[Np-1]; ii++)
				pQNm1[ii/bs*bs*cnux_v[Np-1]+ii%bs+ii*bs] = 1.0;
//			d_print_pmat(nu_v[Np-1]+nx_v[Np-1], nu_v[Np-1]+nx_v[Np-1], bs, pQNm1, cnux_v[Np-1]);
			}
		d_zeros_align(&pQN, pnux_v[Np], cnux_v[Np]);
		for(ii=0; ii<nu_v[Np]+nx_v[Np]; ii++)
			pQN[ii/bs*bs*cnux_v[Np]+ii%bs+ii*bs] = 1.0;
//		d_print_pmat(nu_v[Np]+nx_v[Np], nu_v[Np]+nx_v[Np], bs, pQN, cnux_v[Np]);

		double *(hpQ[Np+1]);
		hpQ[0] = pQ0;
		for(ii=1; ii<rN; ii++) hpQ[ii] = pQ1;
		for(; ii<Np; ii++) hpQ[ii] = pQNm1;
		hpQ[Np] = pQN;
//		for(ii=0; ii<=Np; ii++)
//			d_print_pmat(nu_v[ii]+nx_v[ii], nu_v[ii]+nx_v[ii], bs, hpQ[ii], cnux_v[ii]);



		// work space
		int cnl_v;
		double *(hpL[Np+1]);
		for(ii=0; ii<=Np; ii++)
			{
			cnl_v = cnux_v[ii]<cnx_v[ii]+ncl ? cnx_v[ii]+ncl : cnux_v[ii];
			d_zeros_align(&hpL[ii], pnux_v[ii], cnl_v);
			}
		double *(hdL[Np+1]);
		for(ii=0; ii<=Np; ii++)
			d_zeros_align(&hdL[ii], pnux_v[ii], 1);
		double *pBAL;
		int pnuxM = pnux_v[0];
		for(ii=1; ii<Np; ii++)
			pnuxM = pnux_v[ii]>pnuxM ? pnux_v[ii] : pnuxM;
		int cnxM = cnx_v[1];
		for(ii=2; ii<=Np; ii++)
			cnxM = cnx_v[ii]>cnxM ? cnx_v[ii] : cnxM;
		d_zeros_align(&pBAL, pnuxM, cnxM);



		// time the KKT matrix factorization
		struct timeval tv0, tv1;
		double **dummy;
		int rep;

		nrep = 400;

		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			d_back_ric_trf_tv(Np, nx_v, nu_v, hpBA, hpQ, hpL, hdL, pBAL, nb_v, 0, dummy, ng_v, dummy, dummy);
			}

		gettimeofday(&tv1, NULL); // start

		float time_trf_tv = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		printf("%d\t%e\n", Np, time_trf_tv);


		// free data
		free(pBA0);
		if(Np>1)
			free(pBA1);
		if(Np>1)
			free(pBANm1);
		free(pQ0);
		if(Np>1)
			free(pQ1);
		if(Np>1)
			free(pQNm1);
		free(pQN);
		for(ii=0; ii<=Np; ii++)
			free(hpL[ii]);
		for(ii=0; ii<=Np; ii++)
			free(hdL[ii]);
		free(pBAL);


		}

#else

	int offset;
	int free_x0;
	int ll;
		
	int nux  = nu+nx;
	int nz   = nu+nx+1;
	int pnx  = (nx+bs-1)/bs*bs;
	int pnx1 = (nx+1+bs-1)/bs*bs;
	int pnu  = (nu+bs-1)/bs*bs;
	int pnux = (nu+nx+bs-1)/bs*bs;
	int pnu1 = (nu+1+bs-1)/bs*bs;
	int pnz  = (nu+nx+1+bs-1)/bs*bs;
	int cnx  = (nx+ncl-1)/ncl*ncl;
	int cnu  = (nu+ncl-1)/ncl*ncl;
	int cnux = (nu+nx+ncl-1)/ncl*ncl;

/************************************************
* dynamical system
************************************************/	

	double *A; d_zeros(&A, nx, nx); // states update matrix

	double *B; d_zeros(&B, nx, nu); // inputs matrix

	double *b; d_zeros(&b, nx, 1); // states offset
	double *x0; d_zeros_align(&x0, nx, 1); // initial state

	double Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx, nu, N, A, B, b, x0);

	for(jj=0; jj<nx; jj++)
		b[jj] = 0.1;

	for(jj=0; jj<nx; jj++)
		x0[jj] = 0;
	x0[0] = 3.5;
	x0[1] = 3.5;

//	d_print_mat(nx, nx, A, nx);
//	d_print_mat(nx, nu, B, nx);
//	d_print_mat(nx, 1, b, nx);
//	d_print_mat(nx, 1, x0, nx);
	
	double *pA; d_zeros_align(&pA, pnx, cnx);
	d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);

	double *pAt; d_zeros_align(&pAt, pnx, cnx);
	d_cvt_tran_mat2pmat(nx, nx, A, nx, 0, pAt, cnx);

	double *pBt; d_zeros_align(&pBt, pnu, cnx);
	d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBt, cnx);

	// matrices for size-variant solver
	double *pBAbt0; d_zeros_align(&pBAbt0, pnu1, cnx);
	d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBAbt0, cnx);
	d_cvt_tran_mat2pmat(nx, 1, b, nx, nu, pBAbt0+nu/bs*bs*cnx+nu%bs, cnx);
	double *b0; d_zeros_align(&b0, pnx, 1);
	dgemv_n_lib(nx, nx, pA, cnx, x0, 1, b, b0);

	d_copy_mat(1, nx, b0, 1, pBAbt0+nu/bs*bs*cnx+nu%bs, bs);
	double *pBAbt1; d_zeros_align(&pBAbt1, pnz, cnx);
	d_cvt_tran_mat2pmat(nx, nu, B, nx, 0, pBAbt1, cnx);
	d_cvt_tran_mat2pmat(nx, nx, A, nx, nu, pBAbt1+nu/bs*bs*cnx+nu%bs, cnx);
	d_cvt_tran_mat2pmat(nx, 1, b, nx, nu+nx, pBAbt1+(nu+nx)/bs*bs*cnx+(nu+nx)%bs, cnx);

//	d_print_pmat(nu+1, nx, bs, pBAbt0, cnx);
//	d_print_pmat(nu+nx+1, nx, bs, pBAbt1, cnx);
//	d_print_pmat(nx, nx, bs, pA, cnx);
//	d_print_mat(1, nx, b0, 1);

/************************************************
* cost function
************************************************/	
	
	double *dRSQrq1; d_zeros_align(&dRSQrq1, 2*(nx+nu), 1);
	for(ii=0; ii<nu; ii++) dRSQrq1[ii] = 2.0;
	for( ; ii<nu+nx; ii++) dRSQrq1[ii] = 1.0;
	for( ; ii<2*nu+nx; ii++) dRSQrq1[ii] = 0.2;
	for( ; ii<2*nu+2*nx; ii++) dRSQrq1[ii] = 0.1;
//	d_print_mat(nx+nu, 2, dRSQrq1, nx+nu);

	double *dRSQrq0; d_zeros_align(&dRSQrq0, 2*nu, 1);
	d_copy_mat(nu, 1, dRSQrq1, nu, dRSQrq0, nu);
	d_copy_mat(nu, 1, dRSQrq1+nu+nx, nu, dRSQrq0+nu, nu);

	double *dRSQrqN; d_zeros_align(&dRSQrqN, 2*nx, 1);
	d_copy_mat(nx, 1, dRSQrq1+nu, nx, dRSQrqN, nx);
	d_copy_mat(nx, 1, dRSQrq1+2*nu+nx, nx, dRSQrqN+nx, nx);

	double *pRSQrq0; d_zeros_align(&pRSQrq0, pnu1, cnu);
	ddiain_lib(nu, dRSQrq0, 0, pRSQrq0, cnu);
	drowin_lib(nu, dRSQrq0+nu, pRSQrq0+nu/bs*bs*cnu+nu%bs);
//	d_print_pmat(nu+1, nu, bs, pRSQrq0, cnu);

	double *pRSQrq1; d_zeros_align(&pRSQrq1, pnz, cnux);
	ddiain_lib(nu+nx, dRSQrq1, 0, pRSQrq1, cnux);
	drowin_lib(nu+nx, dRSQrq1+nu+nx, pRSQrq1+nux/bs*bs*cnux+nux%bs);
//	d_print_pmat(nu+nx+1, nu+nx, bs, pRSQrq1, cnux);

	double *pRSQrqN; d_zeros_align(&pRSQrqN, pnx1, cnx);
	ddiain_lib(nx, dRSQrqN, 0, pRSQrqN, cnx);
	drowin_lib(nx, dRSQrqN+nx, pRSQrqN+nx/bs*bs*cnx+nx%bs);
//	d_print_pmat(nx+1, nx, bs, pRSQrqN, cnx);

//	return 0;

/************************************************
* constraints
************************************************/

	int nb[N+1];
#if 0
	int nbx = nx;
#else
	int nbx = 0;
#endif
	nb[0] = nu;
	for(ii=1; ii<N; ii++)
		nb[ii] = nu+nbx;
	nb[N] = nbx;
//	for(ii=0; ii<=N; ii++)
//		printf("%d\n", nb[ii]);

	int pnb[N+1]; for(ii=0; ii<=N; ii++) pnb[ii] = (nb[ii]+bs-1)/bs*bs;


	double *d0; d_zeros_align(&d0, 2*pnb[0], 1);
	int *idx0; i_zeros(&idx0, nb[0], 1);
	for(ii=0; ii<nb[0]; ii++)
		{
		d0[ii]        = -0.5;
		d0[pnb[0]+ii] = -0.5;
		idx0[ii]      = ii;
		}

	double *d1; d_zeros_align(&d1, 2*pnb[1], 1);
	int *idx1; i_zeros(&idx1, nb[1], 1);
	for(ii=0; ii<nu; ii++)
		{
		d1[ii]        = -0.5;
		d1[pnb[1]+ii] = -0.5;
		idx1[ii]      = ii;
		}
	for(; ii<nb[1]; ii++)
		{
		d1[ii]        = -4.0;
		d1[pnb[1]+ii] = -4.0;
		idx1[ii]      = ii;
		}

	double *dN; d_zeros_align(&dN, 2*pnb[N], 1);
	int *idxN; i_zeros(&idxN, nb[N], 1);
	for(ii=0; ii<nb[N]; ii++)
		{
		dN[ii]        = -4.0;
		dN[pnb[N]+ii] = -4.0;
		idxN[ii]      = ii;
		}
	
/************************************************
* original problem data
************************************************/

	double *hpA[N];
	double *hpAt[N];
	double *hpBt[N];
	double *hb[N];
	double *hpBAbt[N];
	double *hpRSQrq[N+1];
	double *hdRSQrq[N+1];
	double *hd[N+1];
	int *hidx[N+1];
	hpA[0] = pA;
	hpAt[0] = pAt;
	hpBt[0] = pBt;
	hb[0] = b0;
	hpBAbt[0] = pBAbt0;
	hpRSQrq[0] = pRSQrq0;
	hdRSQrq[0] = dRSQrq0;
	hd[0] = d0;
	hidx[0] = idx0;
	for(ii=1; ii<N; ii++)
		{
		hpA[ii] = pA;
		hpAt[ii] = pAt;
		hpBt[ii] = pBt;
		hb[ii] = b;
		hpBAbt[ii] = pBAbt1;
		hpRSQrq[ii] = pRSQrq1;
		hdRSQrq[ii] = dRSQrq1;
		hd[ii] = d1;
		hidx[ii] = idx1;
		}
	hpRSQrq[N] = pRSQrqN;
	hdRSQrq[N] = dRSQrqN;
	hd[N] = dN;
	hidx[N] = idxN;
//	for(ii=0; ii<N; ii++)
//		d_print_pmat(nx, nx, bs, hpA[ii], cnx);
	
#if 0
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, 2*pnb[ii], hd[ii], 1);
	exit(1);
#endif

/************************************************
* partial condensing
************************************************/

	int Np = 19;

	int Nc = N/Np; // minimum horizon length within each stage group

	int rN = N - Np*Nc; // number of stage groups with horizon Nc+1 

	int N_tmp;

	int N_v[Np+1];
	for(ii=0; ii<rN; ii++)
		N_v[ii] = Nc+1;
	for(; ii<Np; ii++)
		N_v[ii] = Nc;

	int nx_v[Np+1];
	nx_v[0] = 0;
	for(ii=1; ii<=Np; ii++)
		nx_v[ii] = nx;

	int nu_v[Np+1];
	for(ii=0; ii<Np; ii++)
		nu_v[ii] = nu*N_v[ii];
	nu_v[Np] = 0;

	// inputs and states at initial stages are kept as bounds
	int nb_v[Np+1]; 
	int nb_tmp;
	N_tmp = 0;
	for(ll=0; ll<Np; ll++) 
		{
		nb_tmp = 0;
		for(ii=0; ii<N_v[ll]; ii++)
			for(jj=0; jj<nb[N_tmp+ii]; jj++)
				if(hidx[N_tmp+ii][jj]<nu)
					nb_tmp++;
		ii = 0; // initial stage
		for(jj=0; jj<nb[N_tmp+ii]; jj++)
			if(hidx[N_tmp+ii][jj]>=nu)
				nb_tmp++;
		nb_v[ll] = nb_tmp;

		N_tmp += N_v[ll];
		}
	nb_v[Np] = nb[N];

#if 0
	for(ii=0; ii<=Np; ii++) 
		printf("\n%d\n", nb_v[ii]);
	exit(2);
#endif

	// states after the initial stage are processed as general constraints on the inputs and states at the initial stage
	int ng_v[Np+1]; 
#if 0
	for(ii=0; ii<=Np; ii++) 
		ng_v[ii] = 0;
#else
	N_tmp = 0;
	for(ll=0; ll<Np; ll++)
		{
		nb_tmp = 0;
		for(ii=1; ii<N_v[ll]; ii++)
			for(jj=0; jj<nb[N_tmp+ii]; jj++)
				if(hidx[N_tmp+ii][jj]>=nu)
					nb_tmp++;
		ng_v[ll] = nb_tmp;

		N_tmp += N_v[ll];
		}
	ng_v[Np] = 0;
#endif
#if 0
	for(ii=0; ii<=Np; ii++) 
		printf("\n%d\n", ng_v[ii]);
	exit(2);
#endif

	
	int nux_v[Np+1]; for(ii=0; ii<=Np; ii++) nux_v[ii] = nu_v[ii]+nx_v[ii];
	int pnx_v[Np+1]; for(ii=0; ii<=Np; ii++) pnx_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
	int pnz_v[Np+1]; for(ii=0; ii<=Np; ii++) pnz_v[ii] = (nu_v[ii]+nx_v[ii]+1+bs-1)/bs*bs;
	int pnux_v[Np+1]; for(ii=0; ii<=Np; ii++) pnux_v[ii] = (nu_v[ii]+nx_v[ii]+bs-1)/bs*bs;
	int pnb_v[Np+1]; for(ii=0; ii<=Np; ii++) pnb_v[ii] = (nb_v[ii]+bs-1)/bs*bs;
	int png_v[Np+1]; for(ii=0; ii<=Np; ii++) png_v[ii] = (ng_v[ii]+bs-1)/bs*bs;
	int cng_v[Np+1]; for(ii=0; ii<=Np; ii++) cng_v[ii] = (ng_v[ii]+ncl-1)/ncl*ncl;
	int cnx_v[Np+1]; for(ii=0; ii<=Np; ii++) cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
	int cnux_v[Np+1]; for(ii=0; ii<=Np; ii++) cnux_v[ii] = (nu_v[ii]+nx_v[ii]+ncl-1)/ncl*ncl;
	
	double *hpBAbt2[Np];
	double *hpRSQrq2[Np+1];
	double *hpDCt2[Np+1];
	double *hd2[Np+1];
	int *hidx2[Np+1];
	for(ii=0; ii<Np; ii++)
		{
		d_zeros_align(&hpBAbt2[ii], pnz_v[ii], cnx_v[ii+1]);
		d_zeros_align(&hpRSQrq2[ii], pnz_v[ii], cnux_v[ii]);
		d_zeros_align(&hpDCt2[ii], pnux_v[ii], cng_v[ii]);
		d_zeros_align(&hd2[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		i_zeros(&hidx2[ii], 2*pnb_v[ii], 1);
		}
	d_zeros_align(&hpRSQrq2[Np], pnz_v[Np], cnux_v[Np]);
	d_zeros_align(&hpDCt2[Np], pnux_v[Np], cng_v[Np]);
	d_zeros_align(&hd2[Np], 2*pnb_v[Np]+2*png_v[Np], 1);
	i_zeros(&hidx2[Np], 2*pnb_v[Np], 1);
	
	double *pD; d_zeros_align(&pD, pnu+pnx, cnu);
	double *pM; d_zeros_align(&pM, pnu, cnx);
	double *pQs; d_zeros_align(&pQs, pnx1, cnx);
	double *pLam; d_zeros_align(&pLam, pnz, cnux);
	double *diag_ric; d_zeros_align(&diag_ric, nz, 1);
	double *pBAbtL; d_zeros_align(&pBAbtL, pnz, cnx);

	int pnx1Nnu = (nx+1+(Nc+1)*nu+bs-1)/bs*bs;
	int cnxNnu = (nx+(Nc+1)*nu+ncl-1)/ncl*ncl;
	double *pGamma_Lu; d_zeros_align(&pGamma_Lu, pnx1Nnu, cnu);

	double *hpGamma_u_b[Nc+1];
	for(ii=0; ii<Nc+1; ii++)
		{
		offset = ((ii+1)*nu+nx+1+bs-1)/bs*bs;
		d_zeros_align(&hpGamma_u_b[ii], offset, cnx);
		}

	double *pGamma_tmp;

#define DIAG_HESSIAN 0 // diagonal Hessian of the cost function
#define Q_N_NOT_ZERO 0 // Q_N is not zero



	int rep;
	struct timeval tv0, tv1;

	printf("\nbegin...\n");

	gettimeofday(&tv0, NULL); // stop

	for(rep=0; rep<nrep; rep++)
		{
		N_tmp = 0;
		for(ll=0; ll<Np; ll++)
			{
	//		printf("\n%d %d\n", ll, N_v[ll]);

			if(ll==0)
				free_x0 = 0;
			else
				free_x0 = 1;

			pGamma_tmp = hpGamma_u_b[N_v[ll]-1];
			hpGamma_u_b[N_v[ll]-1] = hpBAbt2[ll];

			d_cond_Gamma_u_b_T(N_v[ll], nx, nu, free_x0, hpA+N_tmp, hpBt+N_tmp, hb+N_tmp, pGamma_Lu, hpGamma_u_b);
//			d_print_pmat(nu_v[ll]+nx_v[ll]+1, nx_v[ll+1], bs, hpBAbt2[ll], cnx_v[ll+1]);

			d_cond_Rr_N2_nx3(N_v[ll], nx, nu, free_x0, hpBAbt+N_tmp, DIAG_HESSIAN, Q_N_NOT_ZERO, hpRSQrq+N_tmp, pD, pM, pQs, pLam, diag_ric, pBAbtL, pGamma_Lu, hpGamma_u_b, hpRSQrq2[ll]); //pH_Rx);
//			d_print_pmat(nu_v[ll]+nx_v[ll]+1, nu_v[ll]+nx_v[ll], bs, hpRSQrq2[ll], cnux_v[ll]);


//printf("\nGamma_u_b = \n");
//for(ii=0; ii<Nc+1; ii++)
//	{
//	offset = ((ii+1)*nu+nx+1+bs-1)/bs*bs;
//	d_print_pmat(offset, nx, bs, hpGamma_u_b[ii], cnx);
//	}

			d_cond_d(N_v[ll], nx, nu, nb+N_tmp, free_x0, hd+N_tmp, hidx+N_tmp, hpGamma_u_b, hd2[ll], hidx2[ll], hpDCt2[ll]);
//			d_print_pmat(nu_v[ll]+nx_v[ll], ng_v[ll], bs, hpDCt2[ll], cng_v[ll]);
//			d_print_mat(1, 2*pnb_v[ll]+2*png_v[ll], hd2[ll], 1);
//			i_print_mat(1, pnb_v[ll], hidx2[ll], 1);

			hpGamma_u_b[N_v[ll]-1] = pGamma_tmp;

			N_tmp += N_v[ll];

			}
		// last stage 
		hpRSQrq2[Np] = pRSQrqN;
		hd2[Np] = dN;
		hidx2[Np] = idxN;

//		exit(3);

		}
	
	gettimeofday(&tv1, NULL); // stop

	printf("\nend...\n");

	double time_part_cond = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);



//	printf("\n%d %d\n", N, N_tmp);

/************************************************
* partially condensed riccati solver
************************************************/

	double *hux2[Np+1];
	double *hpi2[Np+1];
	double *hlam2[Np+1];
	double *ht2[Np+1];
	for(ii=0; ii<=Np; ii++)
		{
		d_zeros_align(&hux2[ii], pnux_v[ii], 1);
		d_zeros_align(&hpi2[ii], pnx_v[ii], 1);
		d_zeros_align(&hlam2[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		d_zeros_align(&ht2[ii], 2*pnb_v[ii]+2*png_v[ii], 1);
		}
	double *work2; d_zeros_align(&work2, d_ip2_hard_mpc_tv_work_space_size_double(Np, nx_v, nu_v, nb_v, ng_v), 1);

	printf("\nsize theory = %d\n", d_ip2_hard_mpc_tv_work_space_size_double(Np, nx_v, nu_v, nb_v, ng_v));

//	for(ii=0; ii<=Np; ii++)
//		printf("\n%d %d %d %d\n", nx_v[ii], nu_v[ii], nb_v[ii], ng_v[ii]);

	int hpmpc_status;
	int kk;
	int k_max = 10;
	double mu0 = 2; // max of cost function
	double mu_tol = 1e-20;
	double alpha_min = 1e-8;
	int warm_start = 0;
	double sigma_par[] = {0.4, 0.1, 0.001}; // control primal-dual IP behaviour
	double *stat; d_zeros(&stat, k_max, 5);


#if 0 // TODO set to 0 !!!!!
	for(ii=0; ii<=Np; ii++) 
		ng_v[ii] = 0;
#endif

//	for(ii=0; ii<N; ii++)
//		d_print_pmat(nu_v[ii]+nx_v[ii]+1, nx_v[ii+1], bs, hpBAbt2[ii], cnx_v[ii+1]);
//	exit(1);

	printf("\nbegin...\n");

	gettimeofday(&tv0, NULL); // stop

	for(rep=0; rep<nrep; rep++)
		{

		hpmpc_status = d_ip2_hard_mpc_tv(&kk, k_max, mu0, mu_tol, alpha_min, warm_start, sigma_par, stat, Np, nx_v, nu_v, nb_v, hidx2, ng_v, hpBAbt2, hpRSQrq2, hpDCt2, hd2, hux2, 0, hpi2, hlam2, ht2, work2);

		}
	gettimeofday(&tv1, NULL); // stop

	printf("\nend...\n");

	double time_solve = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);



	for(ii=0; ii<=Np; ii++)
		d_print_mat(1, nu_v[ii]+nx_v[ii], hux2[ii], 1);
	
	double *hu[N];
	double *hx[N+1];
	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hu[ii], pnu, 1);
		d_zeros_align(&hx[ii], pnx, 1);
		}
	d_zeros_align(&hx[N], pnx, 1);

	N_tmp = 0;
	for(ii=0; ii<Np; ii++)
		{
		for(jj=0; jj<N_v[ii]; jj++)
			{
			d_copy_mat(nu, 1, hux2[ii]+jj*nu, nu, hu[N_tmp], nu);
			N_tmp++;
			}
		}
	printf("\nu = \n");
	for(ii=0; ii<N; ii++)	
		d_print_mat(1, nu, hu[ii], 1);

	dgemv_t_lib(nx, nx, pAt, cnx, hx[0], 1, b0, hx[1]);
	dgemv_t_lib(nu, nx, pBt, cnx, hu[0], 1, hx[1], hx[1]);
	for(ii=1; ii<N; ii++)
		{
		dgemv_t_lib(nx, nx, pAt, cnx, hx[ii], 1, b, hx[ii+1]);
		dgemv_t_lib(nu, nx, pBt, cnx, hu[ii], 1, hx[ii+1], hx[ii+1]);
		}
	printf("\nx = \n");
	for(ii=0; ii<=N; ii++)	
		d_print_mat(1, nx, hx[ii], 1);
		
	for(jj=0; jj<kk; jj++)
		printf("k = %d\tsigma = %f\talpha = %f\tmu = %f\t\tmu = %e\talpha = %f\tmu = %f\tmu = %e\n", jj, stat[5*jj], stat[5*jj+1], stat[5*jj+2], stat[5*jj+2], stat[5*jj+3], stat[5*jj+4], stat[5*jj+4]);
	printf("\n");

	printf("\ntime_part_cond = %e\n", time_part_cond);
	printf("\ntime_solve     = %e\n", time_solve);
	printf("\ntotal_time     = %e\n", time_part_cond+time_solve);

/************************************************
* free memory
************************************************/

	free(A);
	free(B);
	free(b);
	free(b0);
	free(x0);
	free(pA);
	free(pAt);
	free(pBt);
	free(pBAbt0);
	free(pBAbt1);
	free(dRSQrq0);
	free(dRSQrq1);
	free(dRSQrqN);
	free(pRSQrq0);
	free(pRSQrq1);
	free(pRSQrqN);
	free(d0);
	free(d1);
	free(dN);
	free(idx0);
	free(idx1);
	free(idxN);
	free(pD);
	free(pM);
	free(pQs);
	free(pLam);
	free(diag_ric);
	free(pBAbtL);
	free(pGamma_Lu);
	for(ii=0; ii<Nc+1; ii++)
		{
		free(hpGamma_u_b[ii]);
		}

	for(ii=0; ii<Np; ii++)
		{
		free(hpBAbt2[ii]);
		free(hpRSQrq2[ii]);
		free(hpDCt2[ii]);
		free(hd2[ii]);
		free(hidx2[ii]);
		free(hux2[ii]);
		free(hpi2[ii]);
		free(hlam2[ii]);
		free(ht2[ii]);
		}
//	free(hpRSQrq2[Np]);
//	free(hd2[Np]);
	free(hpDCt2[Np]);
//	free(hidx2[Np]);
	free(hux2[Np]);
	free(hpi2[Np]);
	free(hlam2[Np]);
	free(ht2[Np]);
	free(work2);
	free(stat);

	for(ii=0; ii<N; ii++)
		{
		free(hu[ii]);
		free(hx[ii]);
		}
	free(hx[N]);

#endif

	return 0;

	}

