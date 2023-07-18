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

#include "test_param.h"
#include "../problem_size.h"
#include "../include/aux_d.h"
#include "../include/blas_d.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../include/block_size.h"
#include "tools.h"



//#define PRINT_ON



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

	printf("Condensing solver performance test - double precision\n");
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
#elif defined(TARGET_CORTEX_A57)
	const float flops_max = 4;
	printf("Testing solvers for ARMv8a NEON instruction set, oprimized for Cortex A57: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
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
#elif defined(TARGET_CORTEX_A57)
	fprintf(f, "C = 'd_ARM_cortex_A57';\n");
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
//	printf("-sv : Riccati factorization and system solution (prediction step in IP methods)\n");
//	printf("-trs: system solution after a previous call to Riccati factorization (correction step in IP methods)\n");
	printf("\n");
//	if(LTI==1)
//		printf("\nTest for linear time-invariant systems\n");
//	else
//		printf("\nTest for linear time-variant systems\n");
	printf("\n");
	
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
/*	printf("\nflush to zero on\n");*/
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); // flush to zero subnormals !!! works only with one thread !!!
#endif

	int ii, jj;

	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	
#if 1
	int nn[] = {4, 6, 8, 10, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 4000, 4000, 2000, 2000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	int ll_max = 77;
#else
	int nn[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 16, 20, 24, 28, 32, 36, 40, 45, 50, 55, 60, 70, 80, 90, 100};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 4000, 4000, 2000, 2000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	int ll_max = 25;
#endif
	
	int nx, nu, N, nrep;

	int *nx_v, *nu_v, *nb_v, *ng_v;

	int ll;
//	ll_max = 1;
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
			printf("\nProblem size:\n");
			printf("\tN  = \t%d\n", N);
			printf("\tnx = \t%d\n", nx);
			printf("\tnu = \t%d\n\n", nu);
			}
		else
			{
#if 1
			nx = nn[ll]; // number of states (it has to be even for the mass-spring system test problem)
			nu = 4; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = 30; // horizon lenght
			nrep = nnrep[ll]/10;
#else
			nx = 16; // number of states (it has to be even for the mass-spring system test problem)
			nu = 8; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = nn[ll]; // horizon lenght
			nrep = nnrep[ll]/4;
#endif
			}

		int rep;
	
		int nz = nx+nu+1;
		int Nnu = N*nu;
		int Nnx = N*nx;
		int nxNnu = nx + N*nu;
		int pnz = (nz+bs-1)/bs*bs;
		int pnx = (nx+bs-1)/bs*bs;
		int pnu = (nu+bs-1)/bs*bs;
		int pNnu = (N*nu+bs-1)/bs*bs;
		int pnxNnu = (nx+N*nu+bs-1)/bs*bs;
		int cnz = (nx+nu+1+ncl-1)/ncl*ncl;
		int cnx = (nx+ncl-1)/ncl*ncl;
		int cnu = (nu+ncl-1)/ncl*ncl;
		//int cNnu = ((N-1)*nu+cnu+ncl-1)/ncl*ncl;
		int cNnu = (N*nu+ncl-1)/ncl*ncl;
		int cNnx = (N*nx+ncl-1)/ncl*ncl;
		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;
//		int cnl = cnz>cnx+ncl ? pnz : cnx+ncl;

		int ny = nu+nx;
		int pny = (ny+bs-1)/bs*bs;
		int cny = (ny+ncl-1)/ncl*ncl;
		int cnk = cnx+ncl>cny ? cnx+ncl : cny;

/************************************************
* matrix series
************************************************/

		double *pD; d_zeros_align(&pD, pnu+pnx, cnu);
		double *pQs; d_zeros_align(&pQs, pnx, cnx);
		double *diag; d_zeros_align(&diag, pnxNnu, 1);

		const int N2 = 1;
		double *pH_R[N2];
		double *pH_Rx[N2];
		for(ii=0; ii<N2; ii++)
			{
			d_zeros_align(&pH_R[ii], pNnu, cNnu);
			d_zeros_align(&pH_Rx[ii], pnxNnu, cnxNnu);
			}

		for(ii=0; ii<nu; ii++)
			pD[ii/bs*bs*cnu+ii%bs+ii*bs] = 1.0;
//		d_print_pmat(nu, nu, bs, pD, cnu);

		for(ii=0; ii<nx; ii++)
			pQs[ii/bs*bs*cnx+ii%bs+ii*bs] = 1.0;
//		d_print_pmat(nx, nx, bs, pQs, cnx);

		for(ii=0; ii<N*nu; ii++)
			pH_R[0][ii/bs*bs*cNnu+ii%bs+ii*bs] = 1.0;
//		d_print_pmat(N*nu, N*nu, bs, pH_R[0], cNnu);

		for(ii=0; ii<nx+N*nu; ii++)
			pH_Rx[0][ii/bs*bs*cnxNnu+ii%bs+ii*bs] = 1.0;
//		d_print_pmat(nx+N*nu, nx+N*nu, bs, pH_Rx[0], cnxNnu);

/************************************************
* condensing
************************************************/
		
		struct timeval tv0, tv1;





// N^3 factorization algorithm for MPC
		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			dpotrf_lib(N*nu, N*nu, pH_R[0], cNnu, pH_R[0], cNnu, diag);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond_0 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);





// N^3 factorization algorithm for MHE
		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			dpotrf_lib(nx+N*nu, nx+N*nu, pH_Rx[0], cnxNnu, pH_Rx[0], cnxNnu, diag);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond_2 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);





// N factorization algorithm for MPC
		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			for(ii=0; ii<N; ii++)
				{
				dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag);
				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx);
				}
			dpotrf_lib(nu, nu, pD, cnu, pD, cnu, diag);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond_1 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);





// N factorization algorithm for MHE
		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			for(ii=0; ii<N; ii++)
				{
				dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag);
				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx);
				}
			dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag);
			dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx);
			dpotrf_lib(nx, nx, pQs, cnx, pQs, cnx, diag);

			}

		gettimeofday(&tv1, NULL); // start

		double time_cond_3 = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* print restults
************************************************/

		if(ll_max==1)
			{
			printf("\ntime factorization N^3 MPC = %e seconds\n", time_cond_0);
			printf("\ntime factorization N   MPC = %e seconds\n", time_cond_1);
			printf("\ntime factorization N^3 MHE = %e seconds\n", time_cond_2);
			printf("\ntime factorization N   MHE = %e seconds\n", time_cond_3);
			}
		else
			{
			if(ll==0)
				printf("\nN\tnx\tnu\tN3 MPC\t\tN MPC\t\tN3 MHE\t\tN MHE\n");
			printf("%d\t%d\t%d\t%e\t%e\t%e\t%e\n", N, nx, nu, time_cond_0, time_cond_1, time_cond_2, time_cond_3);
			}

/************************************************
* free memory
************************************************/

		free(pD);
		free(pQs);
		free(diag);

		for(ii=0; ii<N2; ii++)
			{
			free(pH_R[ii]);
			free(pH_Rx[ii]);
			}
		
		}

	return 0;

	}

