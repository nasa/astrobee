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


	int nn[] = {4, 6, 8, 10, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300};

	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 4000, 4000, 2000, 2000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	int NNrep[] = {10000, 10000, 10000, 10000, 10000, 4000, 4000, 2000, 2000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};


	int nx, nu, N, nrep;


#if 1
	int ll_max = 77;
#else
	int ll_max = 1;
#endif

	int ll;
	for(ll=0; ll<ll_max; ll++)
		{

		if(ll_max==1)
			{
	
			// problem size
			nx = NX; // number of states (it has to be even for the mass-spring system test problem)
			nu = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = NN; // horizon lenght

			// number of problem instances solution for better timing
			nrep = NREP; 

			}
		else
			{

#if 1
			nx = nn[ll];
			nu = nx/2;
			N  = 30;

			nrep = nnrep[ll];
#else
			nx = 40; //nn[ll];
			nu = 20;
			N  = nn[ll]; //10;

			nrep = NNrep[ll];
#endif

			}


		int nx_v[N+1]; nx_v[0] = 0; for(ii=1; ii<=N; ii++) nx_v[ii] = nx;
		int nu_v[N+1]; for(ii=0; ii<N; ii++) nu_v[ii] = nu; nu_v[N] = 0;
		int nb_v[N+1]; for(ii=0; ii<=N; ii++) nb_v[ii] = 0;
		int ng_v[N+1]; for(ii=0; ii<=N; ii++) ng_v[ii] = 0;


		// matrix size
		int pnx = (nx+bs-1)/bs*bs;
		int pnu = (nu+bs-1)/bs*bs;
		int cnx = (nx+ncl-1)/ncl*ncl;
		int cnu = (nu+ncl-1)/ncl*ncl;

		int pnx_v[N+1];
		int pnux_v[N+1];
		int pnz_v[N+1];
		int cnx_v[N+1];
		int cnux_v[N+1];
		for(ii=0; ii<=N; ii++) 
			{
			pnx_v[ii] = (nx_v[ii]+bs-1)/bs*bs;
			pnux_v[ii] = (nu_v[ii]+nx_v[ii]+bs-1)/bs*bs;
			pnz_v[ii] = (nu_v[ii]+nx_v[ii]+1+bs-1)/bs*bs;
			cnx_v[ii] = (nx_v[ii]+ncl-1)/ncl*ncl;
			cnux_v[ii] = (nu_v[ii]+nx_v[ii]+ncl-1)/ncl*ncl;
			}
			


		// dynamic system matrices
		double *pBAb0;
		d_zeros_align(&pBAb0, pnz_v[0], cnx_v[1]);
		double *pBAb1;
		d_zeros_align(&pBAb1, pnz_v[1], cnx_v[2]);

		double *hpBAb[N];
		hpBAb[0] = pBAb0;
		for(ii=1; ii<N; ii++) 
			hpBAb[ii] = pBAb1;
	//	for(ii=0; ii<Np; ii++)
	//		d_print_pmat(nu_v[ii]+nx_v[ii], nx_v[ii+1], bs, hpBA[ii], cnx_v[ii]);

		double *hb[N];
		for(ii=0; ii<N; ii++)
			d_zeros_align(&hb[ii], pnx_v[ii+1], 1);

		// cost function
		double *hq[N+1];
		for(ii=0; ii<=N; ii++)
			d_zeros_align(&hq[ii], pnux_v[ii], 1);
		double *hl[N+1];
		for(ii=0; ii<=N; ii++)
			d_zeros_align(&hl[ii], pnux_v[ii], 1);

		// L matrices
		int cnl_v;
		double *hpL[N+1];
		for(ii=0; ii<=N; ii++)
			{
			cnl_v = cnux_v[ii]<cnx_v[ii]+ncl ? cnx_v[ii]+ncl : cnux_v[ii];
			d_zeros_align(&hpL[ii], pnux_v[ii], cnl_v);
			}
		double *hdL[N+1];
		for(ii=0; ii<=N; ii++)
			{
			d_zeros_align(&hdL[ii], pnux_v[ii], 1);
			for(jj=0; jj<pnux_v[ii]; jj++) hdL[ii][jj] = 1.0;
			}

		// state & co
		double *hux[N+1];
		for(ii=0; ii<=N; ii++)
			d_zeros_align(&hux[ii], pnux_v[ii], 1);
		
		// hPb
		double *hPb[N];
		for(ii=0; ii<N; ii++)
			d_zeros_align(&hPb[ii], pnx_v[ii+1], 1);

		// work space
		double *work1; d_zeros_align(&work1, pnz_v[1], 2);



		// time the KKT matrix factorization
		struct timeval tv0, tv1;
	//	double **dummy;
		int rep;

		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			d_back_ric_trs_tv(N, nx_v, nu_v, hpBAb, hb, hpL, hdL, hq, hl, hux, work1, 1, hPb, 0, dummy, nb_v, 0, dummy, ng_v, dummy, dummy);
			}

		gettimeofday(&tv1, NULL); // start

		float Gflops_max = flops_max * GHz_max;

		float time_trs_tv = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		float flop_trs_tv = N*(6*nx*nx + 8*nx*nu + 2*nu);
		float Gflops_trs_tv = 1e-9*flop_trs_tv/time_trs_tv;


		double *A; d_zeros_align(&A, pnx, cnx);
		double *B; d_zeros_align(&B, pnx, cnu);
		double *D_inv; d_zeros_align(&D_inv, pnu, cnu);
		double *K; d_zeros_align(&K, pnu, cnx);
		double *P; d_zeros_align(&P, pnx, cnx);

		double *p; d_zeros_align(&p, pnx, 1);
		double *t; d_zeros_align(&t, pnu, 1);
		double *pt; d_zeros_align(&pt, pnx, 1);

		double *hk[N];
		for(ii=0; ii<N; ii++)
			d_zeros_align(&hk[ii], pnu, 1);
		double *hu[N];
		for(ii=0; ii<N; ii++)
			d_zeros_align(&hu[ii], pnu, 1);
		double *hx[N+1];
		for(ii=0; ii<=N; ii++)
			d_zeros_align(&hx[ii], pnx, 1);

		int kk = 0;

		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{

			d_copy_mat(nx, 1, hq[N], nx, p, nx);
			for(kk=0; kk<N; kk++)
				{
				dsymv_lib(nx, nx, P, cnx, hb[N-kk-1], 1, p, p);
//				dtrmv_u_n_lib(nx, P, cnx, hb[N-kk-1], 0, pt);
//				dtrmv_u_t_lib(nx, P, cnx, pt, 1, p);
				dgemv_t_lib(nx, nu, B, cnu, p, 1, hq[N-kk-1], t);
				dgemv_t_lib(nx, nx, A, cnx, p, 1, hq[N-kk-1]+nu, pt);
				dgemv_t_lib(nu, nx, K, cnx, t, 1, pt+nu, p);
				d_set_mat(nu, 1, 0.0, hk[N-kk-1], nu);
				dsymv_lib(nu, nu, D_inv, cnu, t, -1, hk[N-kk-1], hk[N-kk-1]);
				}
			for(kk=0; kk<N; kk++)
				{
				dgemv_n_lib(nu, nx, K, cnx, hx[kk], 1, hk[kk], hu[kk]);
				dgemv_n_lib(nx, nx, A, cnx, hx[kk], 1, hb[kk], hx[kk+1]);
				dgemv_n_lib(nx, nu, B, cnu, hu[kk], 1, hx[kk+1], hx[kk+1]);
				}

			}

		gettimeofday(&tv1, NULL); // start

		float time_trs_tv_tim = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		float flop_trs_tv_tim = N*(6*nx*nx + 8*nx*nu + 2*nu);
		float Gflops_trs_tv_tim = 1e-9*flop_trs_tv_tim/time_trs_tv_tim;



		printf("%d\t%d\t%d\t%e\t%f\t%f\t%e\t%f\t%f\n", nx, nu, N, time_trs_tv, Gflops_trs_tv, 100.0*Gflops_trs_tv/Gflops_max, time_trs_tv_tim, Gflops_trs_tv_tim, 100.0*Gflops_trs_tv_tim/Gflops_max);


		// free data
		free(pBAb0);
		free(pBAb1);
		free(work1);
		for(ii=0; ii<N; ii++)
			{
			free(hb[ii]);
			free(hPb[ii]);
			free(hpL[ii]);
			free(hdL[ii]);
			free(hq[ii]);
			free(hl[ii]);
			free(hux[ii]);
			}
		free(hpL[N]);
		free(hdL[N]);
		free(hq[N]);
		free(hl[N]);
		free(hux[N]);

		free(A);
		free(B);
		free(D_inv);
		free(K);
		free(P);
		free(p);
		free(t);
		free(pt);
		for(ii=0; ii<N; ii++)
			{
			free(hk[ii]);
			free(hu[ii]);
			free(hx[ii]);
			}
		free(hx[N]);
		
		}

	return 0;

	}

