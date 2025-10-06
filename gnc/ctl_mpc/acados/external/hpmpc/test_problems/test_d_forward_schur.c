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

	printf("Forward Schur-complement solver performance test - double precision\n");
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
	printf("-trf: factorization routine\n");
	printf("-trs: solution routine\n");
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

	int nx, nu, N, rep, nrep;

	int info;

	int ll;
#if 1
	int ll_max = 77;
#else
	int ll_max = 1;
#endif
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
			nu = nx/2; //2; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			N  = 10; // horizon lenght
			nrep = 2*nnrep[ll];
//			nrep = nnrep[ll]/4;
			}

		int nd = 0;

		int nxu = nx+nu;

		int pnx  = (nx+bs-1)/bs*bs;
		int pnxu = (nxu+bs-1)/bs*bs;
		int cnx  = (nx+ncl-1)/ncl*ncl;
		int cnxu = (nxu+ncl-1)/ncl*ncl;


		// time-variant
		int nv_tv[N+1];
		int ne_tv[N+1];

		nv_tv[0] = nu;
		ne_tv[0] = nx;
		for(ii=1; ii<N; ii++)
			{
			nv_tv[ii] = nu+nx;
			ne_tv[ii] = nx;
			}
		nv_tv[N] = nx;
		ne_tv[N] = nd;

		// diagonal hessian
		int diag_hessian[N+1];
		for(ii=0; ii<=N; ii++)
			diag_hessian[ii] = 0;
//		diag_hessian[2] = 1;


		int pnv_tv[N+1];
		int pne_tv[N+1];
		int cnv_tv[N+1];
		int cne_tv[N+1];
		for(ii=0; ii<=N; ii++)
			{
			pnv_tv[ii] = (nv_tv[ii]+bs-1)/bs*bs;
			pne_tv[ii] = (ne_tv[ii]+bs-1)/bs*bs;
			cnv_tv[ii] = (nv_tv[ii]+ncl-1)/ncl*ncl;
			cne_tv[ii] = (ne_tv[ii]+ncl-1)/ncl*ncl;
			}




/************************************************
* dynamical system
************************************************/	

		double *A; d_zeros(&A, nx, nx); // states update matrix

		double *B; d_zeros(&B, nx, nu); // inputs matrix

		double *b; d_zeros(&b, nx, 1); // states offset
		double *x0; d_zeros_align(&x0, pnx, 1); // initial state

		double Ts = 0.5; // sampling time
		mass_spring_system(Ts, nx, nu, N, A, B, b, x0);
	
		for(jj=0; jj<nx; jj++)
			b[jj] = 0.0;
	
		for(jj=0; jj<nx; jj++)
			x0[jj] = 0;
		x0[0] = 3.5;
		x0[1] = 3.5;

		double *AB; d_zeros(&AB, nx, nx+nu);
		for(ii=0; ii<nx*nx; ii++) AB[ii] = A[ii];
		for(ii=0; ii<nx*nu; ii++) AB[nx*nx+ii] = B[ii];

		double *C; d_zeros(&C, nd, nx);
		for(ii=0; ii<nd; ii++) C[ii*(nd+1)] = 1.0;
	
/************************************************
* cost function
************************************************/	

		const int ncx = nx;

		double *dQ; d_zeros_align(&dQ, nxu, 1);
		for(ii=0; ii<nx; ii++) dQ[ii] = 1.0;
		for(; ii<nxu; ii++) dQ[ii] = 2.0;

		double *Q; d_zeros_align(&Q, nxu, nxu);
		for(ii=0; ii<nx; ii++) Q[ii*(nxu+1)] = dQ[ii];
		for(; ii<nxu; ii++) Q[ii*(nxu+1)] = dQ[ii];


/************************************************
* pack matrices
************************************************/	

#if 0
		double *pQA; d_zeros_align(&pQA, pnxu+pnx, cnxu);
		d_cvt_mat2pmat(nxu, nxu, Q, nxu, 0, pQA, cnxu);
		d_cvt_mat2pmat(nx, nx, A, nx, 0, pQA+pnxu*cnxu, cnxu);
		d_cvt_mat2pmat(nx, nu, B, nx, 0, pQA+pnxu*cnxu+nx*bs, cnxu);
		// copy the bottom of AB in the empty space !!!!!
		if(pnxu-nxu<nx)
			dgecp_lib(pnxu-nxu, nxu, nxu+nx, pQA+(nxu+nx)/bs*bs*cnxu+(nxu+nx)%bs, cnxu, nxu, pQA+nxu/bs*bs*cnxu+nxu%bs, cnxu);
		else
			dgecp_lib(nx, nxu, 0, pQA+pnxu*cnxu, cnxu, nxu, pQA+nxu/bs*bs*cnxu+nxu%bs, cnxu);
//		d_print_pmat(pnxu+pnx, cnxu, bs, pQA, cnxu);

		double *(hpQA[N+1]);
		double *(hpLA[N+1]);
		double *(hdLA[N+1]);
		double *(hpLe[N+1]);
		
		for(ii=0; ii<N; ii++)
			{
			hpQA[ii] = pQA;
			d_zeros_align(&hpLA[ii], pnxu+pnx, cnxu);
			d_zeros_align(&hdLA[ii], pnxu, 1);
			d_zeros_align(&hpLe[ii], pnx, cnx);
			}
		hpQA[N] = pQA;
		d_zeros_align(&hpLA[N], pnxu+pnx, cnxu);
		d_zeros_align(&hdLA[N], pnxu, 1);
		d_zeros_align(&hpLe[N], pnx, cnx);

#endif



		// dense Hessian of cost function

		double *pQA0; d_zeros_align(&pQA0, pnv_tv[0]+pne_tv[0], cnv_tv[0]);
		d_cvt_mat2pmat(nv_tv[0], nv_tv[0], Q+nx*(nxu+1), nxu, 0, pQA0, cnv_tv[0]);
		d_cvt_mat2pmat(ne_tv[0], nv_tv[0], B, nx, 0, pQA0+pnv_tv[0]*cnv_tv[0], cnv_tv[0]);
//		d_print_pmat(pnv_tv[0]+pne_tv[0], cnv_tv[0], bs, pQA0, cnv_tv[0]);

		double *pQA1; d_zeros_align(&pQA1, pnv_tv[1]+pne_tv[1], cnv_tv[1]);
		d_cvt_mat2pmat(nv_tv[1], nv_tv[1], Q, nxu, 0, pQA1, cnv_tv[1]);
		d_cvt_mat2pmat(ne_tv[1], nv_tv[1], AB, nx, 0, pQA1+pnv_tv[1]*cnv_tv[1], cnv_tv[1]);
//		d_print_pmat(pnv_tv[1]+pne_tv[1], cnv_tv[1], bs, pQA1, cnv_tv[1]);

		double *pQAN; d_zeros_align(&pQAN, pnv_tv[N]+pne_tv[N], cnv_tv[N]);
		d_cvt_mat2pmat(nv_tv[N], nv_tv[N], Q, nxu, 0, pQAN, cnv_tv[N]);
		if(ne_tv[N]>0)
			d_cvt_mat2pmat(ne_tv[N], nv_tv[N], C, nd, 0, pQAN+pnv_tv[N]*cnv_tv[N], cnv_tv[N]);
//		d_print_pmat(pnv_tv[N]+pne_tv[N], cnv_tv[N], bs, pQAN, cnv_tv[N]);

#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_MKL) || defined(REF_BLAS_NETLIB)
		double *QA0; d_zeros(&QA0, nv_tv[0]+ne_tv[0], nv_tv[0]);
		d_copy_mat(nv_tv[0], nv_tv[0], Q+nx*(nxu+1), nxu, QA0, nv_tv[0]+ne_tv[0]);
		d_copy_mat(ne_tv[0], nv_tv[0], B, nx, QA0+nv_tv[0], nv_tv[0]+ne_tv[0]);

		double *QA1; d_zeros(&QA1, nv_tv[1]+ne_tv[1], nv_tv[1]);
		d_copy_mat(nv_tv[1], nv_tv[1], Q, nxu, QA1, nv_tv[1]+ne_tv[1]);
		d_copy_mat(ne_tv[1], nv_tv[1], AB, nx, QA1+nv_tv[1], nv_tv[1]+ne_tv[1]);

		double *QAN; d_zeros(&QAN, nv_tv[N]+ne_tv[N], nv_tv[N]);
		d_copy_mat(nv_tv[N], nv_tv[N], Q, nxu, QAN, nv_tv[N]+ne_tv[N]);
		if(ne_tv[N]>0)
			d_copy_mat(ne_tv[N], nv_tv[N], C, nx, QAN+nv_tv[N], nv_tv[N]+ne_tv[N]);

//		d_print_mat(nv_tv[0]+ne_tv[0], nv_tv[0], QA0, nv_tv[0]+ne_tv[0]);
//		d_print_mat(nv_tv[1]+ne_tv[1], nv_tv[1], QA1, nv_tv[1]+ne_tv[1]);
//		d_print_mat(nv_tv[N]+ne_tv[N], nv_tv[N], QAN, nv_tv[N]+ne_tv[N]);
//		exit(2);
#endif


		// dense Hessian of cost function

		double *dQpA0; d_zeros_align(&dQpA0, pnv_tv[0]+pne_tv[0]*cnv_tv[0], 1);
		d_copy_mat(nv_tv[0], 1, dQ+nx, 1, dQpA0, 1);
		d_cvt_mat2pmat(ne_tv[0], nv_tv[0], B, nx, 0, dQpA0+pnv_tv[0], cnv_tv[0]);
//		d_print_mat(1, pnv_tv[0], dQpA0, 1);
//		d_print_pmat(pne_tv[0], cnv_tv[0], bs, dQpA0+pnv_tv[0], cnv_tv[0]);

		double *dQpA1; d_zeros_align(&dQpA1, pnv_tv[1]+pne_tv[1]*cnv_tv[1], 1);
//		d_cvt_mat2pmat(nv_tv[1], nv_tv[1], Q, nxu, 0, pQA1, cnv_tv[1]);
//		d_cvt_mat2pmat(ne_tv[1], nv_tv[1], AB, nx, 0, pQA1+pnv_tv[1]*cnv_tv[1], cnv_tv[1]);
//		d_print_pmat(pnv_tv[1]+pne_tv[1], cnv_tv[1], bs, pQA1, cnv_tv[1]);
		d_copy_mat(nv_tv[1], 1, dQ, 1, dQpA1, 1);
		d_cvt_mat2pmat(ne_tv[1], nv_tv[1], AB, nx, 0, dQpA1+pnv_tv[1], cnv_tv[1]);
//		d_print_mat(1, pnv_tv[1], dQpA1, 1);
//		d_print_pmat(pne_tv[1], cnv_tv[1], bs, dQpA1+pnv_tv[1], cnv_tv[1]);

		double *dQpAN; d_zeros_align(&dQpAN, pnv_tv[N]+pne_tv[N]*cnv_tv[N], 1);
//		d_cvt_mat2pmat(nv_tv[N], nv_tv[N], Q, nxu, 0, pQAN, cnv_tv[N]);
//		if(ne_tv[N]>0)
//			d_cvt_mat2pmat(ne_tv[N], nv_tv[N], C, nd, 0, pQAN+pnv_tv[N]*cnv_tv[N], cnv_tv[N]);
//		d_print_pmat(pnv_tv[N]+pne_tv[N], cnv_tv[N], bs, pQAN, cnv_tv[N]);
		d_copy_mat(nv_tv[N], 1, dQ, 1, dQpAN, 1);
		if(ne_tv[N]>0)
			d_cvt_mat2pmat(ne_tv[N], nv_tv[N], C, nd, 0, dQpAN+pnv_tv[N], cnv_tv[N]);
//		d_print_mat(1, pnv_tv[N], dQpAN, 1);
//		d_print_pmat(pne_tv[N], cnv_tv[N], bs, dQpAN+pnv_tv[N], cnv_tv[N]);
//		exit(2);



		// jacobian of cost function

		double *qb0; d_zeros_align(&qb0, pnv_tv[0]+pne_tv[0], 1);
		double *pA; d_zeros_align(&pA, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);
//		d_print_pmat(nx, nx, bs, pA, cnx);
		dgemv_n_lib(nx, nx, pA, cnx, x0, 0, qb0+pnv_tv[0], qb0+pnv_tv[0]);
//		d_print_mat(1, pnv_tv[0]+pne_tv[0], qb0, 1);

		double *qb1; d_zeros_align(&qb1, pnv_tv[1]+pne_tv[1], 1);
		for(ii=0; ii<nv_tv[1]; ii++) qb1[ii] = 0.0;
		for(ii=0; ii<ne_tv[1]; ii++) qb1[pnv_tv[1]+ii] = 0.0;

		double *qbN; d_zeros_align(&qbN, pnv_tv[N]+pne_tv[N], 1);
		if(ne_tv[N]>0)
			{
			qbN[pnv_tv[N]+0] =  1.0;
			qbN[pnv_tv[N]+1] = -2.0;
			}

#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_MKL) || defined(REF_BLAS_NETLIB)
		double *qb0b; d_zeros_align(&qb0b, nv_tv[0]+ne_tv[0], 1);
		d_copy_mat(nv_tv[0], 1, qb0, 1, qb0b, 1);
		d_copy_mat(ne_tv[0], 1, qb0+pnv_tv[0], 1, qb0b+nv_tv[0], 1);
		double *qb1b; d_zeros_align(&qb1b, nv_tv[1]+ne_tv[1], 1);
		d_copy_mat(nv_tv[1], 1, qb1, 1, qb1b, 1);
		d_copy_mat(ne_tv[1], 1, qb1+pnv_tv[1], 1, qb1b+nv_tv[1], 1);
		double *qbNb; d_zeros_align(&qbNb, nv_tv[N]+ne_tv[N], 1);
		d_copy_mat(nv_tv[N], 1, qbN, 1, qbNb, 1);
		d_copy_mat(ne_tv[N], 1, qbN+pnv_tv[N], 1, qbNb+nv_tv[N], 1);

//		d_print_mat(1, nv_tv[0]+ne_tv[0], qb0b, 1);
//		d_print_mat(1, nv_tv[1]+ne_tv[1], qb1b, 1);
//		d_print_mat(1, nv_tv[N]+ne_tv[N], qbNb, 1);
//		exit(2);
#endif


		double *hpQA_tv[N+1];
//		double *hdQpA_tv[N+1];
		double *hqb_tv[N+1];
		double *hpLA_tv[N+1];
		double *hdLA_tv[N+1];
		double *hpLe_tv[N+1];
		double *hxupi_tv[N+1];
		
		if(diag_hessian[0])
			hpQA_tv[0] = dQpA0;
		else
			hpQA_tv[0] = pQA0;
		hqb_tv[0] = qb0;
		d_zeros_align(&hpLA_tv[0], pnv_tv[0]+pne_tv[0], cnv_tv[0]);
		d_zeros_align(&hdLA_tv[0], pnv_tv[0], 1);
		d_zeros_align(&hpLe_tv[0], pne_tv[0], cne_tv[0]);
		d_zeros_align(&hxupi_tv[0], pnv_tv[0]+pne_tv[0], 1);
		for(ii=1; ii<N; ii++)
			{
			if(diag_hessian[ii])
				hpQA_tv[ii] = dQpA1;
			else
				hpQA_tv[ii] = pQA1;
			hqb_tv[ii] = qb1;
			d_zeros_align(&hpLA_tv[ii], pnv_tv[ii]+pne_tv[ii], cnv_tv[ii]);
			d_zeros_align(&hdLA_tv[ii], pnv_tv[ii], 1);
			d_zeros_align(&hpLe_tv[ii], pne_tv[ii], cne_tv[ii]);
			d_zeros_align(&hxupi_tv[ii], pnv_tv[ii]+pne_tv[ii], 1);
			}
		if(diag_hessian[N])
			hpQA_tv[N] = dQpAN;
		else
			hpQA_tv[N] = pQAN;
		hqb_tv[N] = qbN;
		d_zeros_align(&hpLA_tv[N], pnv_tv[N]+pne_tv[N], cnv_tv[N]);
		d_zeros_align(&hdLA_tv[N], pnv_tv[N], 1);
		d_zeros_align(&hpLe_tv[N], pne_tv[N], cne_tv[N]);
		d_zeros_align(&hxupi_tv[N], pnv_tv[N]+pne_tv[N], 1);


		int neM = 0;
		for(ii=0; ii<=N; ii++) if(ne_tv[ii]>neM) neM = ne_tv[ii];
		int pneM = (neM+bs-1)/bs*bs;
		int cneM = (neM+ncl-1)/ncl*ncl;


		double *work_tv; d_zeros_align(&work_tv, pneM*cneM+pneM, 1);
		double *tmp_tv; d_zeros_align(&tmp_tv, pneM, 1);


		double *hres_tv[N+1];
		for(ii=0; ii<=N; ii++)
			{
			d_zeros_align(&hres_tv[ii], pnv_tv[ii]+pne_tv[ii], 1);
			}

#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_MKL) || defined(REF_BLAS_NETLIB)
		double *hQA_tv[N+1];
		double *hLA_tv[N+1];
		double *hLe_tv[N+1];
		double *hqbb_tv[N+1];
		double *Le_tmp;
		hQA_tv[0] = QA0;
		d_zeros(&hLA_tv[0], nv_tv[0]+ne_tv[0], nv_tv[0]);
		d_zeros(&hLe_tv[0], ne_tv[0], ne_tv[0]);
		hqbb_tv[0] = qb0b;
		for(ii=1; ii<N; ii++)
			{
			hQA_tv[ii] = QA1;
			d_zeros(&hLA_tv[ii], nv_tv[ii]+ne_tv[ii], nv_tv[ii]);
			d_zeros(&hLe_tv[ii], ne_tv[ii], ne_tv[ii]);
			hqbb_tv[ii] = qb1b;
			}
		hQA_tv[N] = QAN;
		d_zeros(&hLA_tv[N], nv_tv[N]+ne_tv[N], nv_tv[N]);
		d_zeros(&hLe_tv[N], ne_tv[N], ne_tv[N]);
		hqbb_tv[N] = qbNb;
		d_zeros(&Le_tmp, neM, neM);
#endif



/************************************************
* call time-invariant solver
************************************************/	

#if 0
		double *work; d_zeros_align(&work, cnx*pnx+pnx, 1);

		info = d_forward_schur_trf(N, nx, nu, 0, 0, hpQA, hpLA, hdLA, hpLe, work);	

		printf("\nreturn value = %d\n\n", info);

		for(ii=0; ii<=N; ii++)
			{
			d_print_pmat(pnxu+pnx, cnxu, bs, hpLA[ii], cnxu);
			d_print_pmat(pnx, cnx, bs, hpLe[ii], cnx);
			}
#endif


/************************************************
* call time-variant solver
************************************************/	

		struct timeval tv0, tv1, tv2, tv3, tv4, tv5, tv6;


		gettimeofday(&tv0, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
//			if(diag_hessian)
//				info = d_forward_schur_trf_tv(N, nv_tv, ne_tv, 1e-8, 1, hdQpA_tv, hpLA_tv, hdLA_tv, hpLe_tv, work_tv);	
//			else // dense hessian
				info = d_forward_schur_trf_tv(N, nv_tv, ne_tv, 1e-8, diag_hessian, hpQA_tv, hpLA_tv, hdLA_tv, hpLe_tv, work_tv);	
			}

		gettimeofday(&tv1, NULL); // start

#if 0
		printf("\nreturn value = %d\n\n", info);

		for(ii=0; ii<=N; ii++)
			{
			d_print_pmat(pnv_tv[ii]+pne_tv[ii], cnv_tv[ii], bs, hpLA_tv[ii], cnv_tv[ii]);
			d_print_pmat(pne_tv[ii], cne_tv[ii], bs, hpLe_tv[ii], cne_tv[ii]);
			}
#endif


		gettimeofday(&tv2, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
//			if(diag_hessian)
//				d_forward_schur_trs_tv(N, nv_tv, ne_tv, 1, hqb_tv, hpLA_tv, hdLA_tv, hpLe_tv, hxupi_tv, tmp_tv);	
//			else // dense hessian
				d_forward_schur_trs_tv(N, nv_tv, ne_tv, diag_hessian, hqb_tv, hpLA_tv, hdLA_tv, hpLe_tv, hxupi_tv, tmp_tv);	
			}

		gettimeofday(&tv3, NULL); // start
			

		if(ll_max==1)
			{
#if 0
			for(ii=0; ii<=N; ii++)
				{
				d_print_mat(1, pnv_tv[ii]+pne_tv[ii], hxupi_tv[ii], 1);
				}
#else
			printf("\nxu\n");
			for(ii=0; ii<=N; ii++)
				{
				d_print_mat(1, nv_tv[ii], hxupi_tv[ii], 1);
				}
			printf("\npi\n");
			for(ii=0; ii<=N; ii++)
				{
				d_print_mat(1, ne_tv[ii], hxupi_tv[ii]+pnv_tv[ii], 1);
				}
#endif
			}

		// compute residuals
#if 0

		d_forward_schur_res_tv(N, nv_tv, ne_tv, diag_hessian, hpQA_tv, hqb_tv, hxupi_tv, hres_tv);

		printf("\nresiduals\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, nv_tv[ii], hres_tv[ii], 1);
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, ne_tv[ii], hres_tv[ii]+pnv_tv[ii], 1);

#endif 

#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_MKL) || defined(REF_BLAS_NETLIB)
		gettimeofday(&tv4, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			info = d_forward_schur_trf_tv_blas(N, nv_tv, ne_tv, 1e-8, diag_hessian, hQA_tv, hLA_tv, hLe_tv, Le_tmp);	
			}

		gettimeofday(&tv5, NULL); // start

		for(rep=0; rep<nrep; rep++)
			{
			d_forward_schur_trs_tv_blas(N, nv_tv, ne_tv, diag_hessian, hqbb_tv, hLA_tv, hLe_tv, hxupi_tv, tmp_tv);	
			}

		gettimeofday(&tv6, NULL); // start
			

		if(ll_max==1)
			{
#if 0
			for(ii=0; ii<=N; ii++)
				{
				d_print_mat(1, pnv_tv[ii]+pne_tv[ii], hxupi_tv[ii], 1);
				}
#else
			printf("\nxu\n");
			for(ii=0; ii<=N; ii++)
				{
				d_print_mat(1, nv_tv[ii], hxupi_tv[ii], 1);
				}
			printf("\npi\n");
			for(ii=0; ii<=N; ii++)
				{
				d_print_mat(1, ne_tv[ii], hxupi_tv[ii]+nv_tv[ii], 1);
				}
#endif
			}

#endif


		// compute residuals
#if 0

		d_forward_schur_res_tv(N, nv_tv, ne_tv, diag_hessian, hpQA_tv, hqb_tv, hxupi_tv, hres_tv);

		printf("\nresiduals\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, nv_tv[ii], hres_tv[ii], 1);
		for(ii=0; ii<=N; ii++)
			d_print_mat(1, ne_tv[ii], hres_tv[ii]+pnv_tv[ii], 1);

#endif 

		int ne0, ne1, nv0, nv1;

		float time_trf = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		float flops_trf = 0;
#if 1
		// first stage
		ne0 = ne_tv[0];
		nv0 = nv_tv[0];
		if(diag_hessian[0])
			{
			flops_trf += 2.0*ne0*nv0;
			}
		else
			{
			flops_trf += 1.0/3.0*nv0*nv0*nv0 + nv0*nv0*ne0;
			}
		flops_trf += nv0*ne0*ne0 + 2.0/3.0*ne0*ne0*ne0;
		// middle stages
		for(ii=1; ii<N; ii++)
			{
			ne1 = ne0;
			nv1 = nv0;
			ne0 = ne_tv[ii];
			nv0 = nv_tv[ii];
			if(diag_hessian[ii])
				{
				flops_trf += 5.0/3.0*ne1*ne1*ne1 + 2.0*ne0*(nv0-ne1);
				}
			else
				{
				flops_trf += 1.0/3.0*ne1*ne1*ne1 + 1.0/3.0*nv0*nv0*nv0 + nv0*nv0*ne0;
				}
			flops_trf += nv0*ne0*ne0 + 2.0/3.0*ne0*ne0*ne0;
			}
		// last stage
		ne1 = ne0;
		nv1 = nv0;
		ne0 = ne_tv[N];
		nv0 = nv_tv[N];
		flops_trf += 1.0/3.0*ne1*ne1*ne1 + 1.0/3.0*nv0*nv0*nv0 + nv0*nv0*ne0 + ne0*ne0*nv0 + 2.0/3.0*ne0*ne0*ne0;
#else
		if(1) // dense hessian
			{
			flops_trf = N*(10.0/3.0*nx*nx*nx + 4.0*nx*nx*nu + 2.0*nx*nu*nu + 1.0/3.0*nu*nu*nu) + 1.0/3.0*nx*nx*nx + 1.0*nx*nx*nd + 1.0*nx*nd*nd + 1.0/3.0*nd*nd*nd;
			if(nv_tv[0]==nu)
				flops_trf -= 4.0/3.0*nx*nx*nx + 3.0*nx*nx*nu + 1.0*nx*nu*nu;
			}
		else
			{
			flops_trf = N*(10.0/3.0*nx*nx*nx + 1.0*nx*nx*nu) - 1.0*nx*nx*nx + 1.0*nx*nx*nd + 1.0*nx*nd*nd + 1.0/3.0*nd*nd*nd;
			if(nv_tv[0]==nu)
				flops_trf -= 1.0*nx*nx*nx;
			}
#endif
		float Gflops_trf = 1e-9*flops_trf/time_trf;

		float time_trs = (float) (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);
		float flops_trs = 0;
#if 1
		// first stage
		ne0 = ne_tv[0];
		nv0 = nv_tv[0];
		if(diag_hessian[0])
			{
			flops_trs += 2.0*nv0  + 2.0*ne0*nv0;
			}
		else
			{
			flops_trs += nv0*nv0 + 2.0*nv0*ne0;
			}
		// middle stages
		for(ii=1; ii<=N; ii++)
			{
			ne1 = ne0;
			nv1 = nv0;
			ne0 = ne_tv[ii];
			nv0 = nv_tv[ii];
			if(diag_hessian[ii])
				{
				flops_trs += ne1*ne1 + 2.0*(nv0-ne1) + 2.0*nv0*ne0;
				}
			else
				{
				flops_trs += nv0*nv0 + 2.0*nv0*ne0;
				}
			flops_trs += 2.0*ne1*ne1;
			}
		// last stage
		flops_trs += 2.0*ne0*ne0 + nv0*nv0 + 2.0*nv0*ne0;
		// middle stages
		for(ii=1; ii<N; ii++)
			{
			ne0  = ne1;
			ne1  = ne_tv[N-ii-1];//ne0;
			nv0  = nv_tv[N-ii];
			if(diag_hessian[ii])
				{
				flops_trs += ne1*ne1 + 2.0*(nv0-ne1) + 2.0*nv0*ne0;
				}
			else
				{
				flops_trs += nv0*nv0 + 2.0*nv0*ne0;
				}
			flops_trs += 2.0*ne0*ne0;
			}
		// first stage
		ii = N;
		ne0  = ne1;
		ne1  = ne_tv[N-ii-1];//ne0;
		nv0  = nv_tv[N-ii];
		if(diag_hessian[ii])
			{
			flops_trs += 2.0*nv0 + 2.0*nv0*ne0;
			}
		else
			{
			flops_trs += nv0*nv0 + 2.0*nv0*ne0;
			}
		flops_trs += 2.0*ne0*ne0;
#else
		if(1) // dense hessian
			{
			flops_trs = N*(10.0*nx*nx + 8.0*nx*nu + 2.0*nu*nu) + 2.0*nx*nx + 4.0*nx*nd + 2.0*nd*nd;
			if(nv_tv[0]==nu)
				flops_trs -= 6.0*nx*nx + 4.0*nx*nu;
			}
		else
			{
			flops_trs = N*(10.0*nx*nx + 4.0*nx*nu) + 4.0*nx*nd + 2.0*nd*nd;
			if(nv_tv[0]==nu)
				flops_trs -= 4.0*nx*nx;
			}
#endif
		float Gflops_trs = 1e-9*flops_trs/time_trs;

		float Gflops_max = flops_max * GHz_max;

		float time_trf_blas = (float) (tv5.tv_sec-tv4.tv_sec)/(nrep+0.0)+(tv5.tv_usec-tv4.tv_usec)/(nrep*1e6);
		float Gflops_trf_blas = 1e-9*flops_trf/time_trf_blas;

		float time_trs_blas = (float) (tv6.tv_sec-tv5.tv_sec)/(nrep+0.0)+(tv6.tv_usec-tv5.tv_usec)/(nrep*1e6);
		float Gflops_trs_blas = 1e-9*flops_trs/time_trs_blas;

		if(ll==0)
			printf("\nnx\tnu\tnd\tN\ttrf_time\tGlops\t\t%%\t\ttrs_time\tGlops\t\t%%\n");
		printf("%d\t%d\t%d\t%d\t%e\t%f\t%f\t%e\t%f\t%f\t%e\t%f\t%f\t%e\t%f\t%f\n", nx, nu, nd, N, time_trf, Gflops_trf, 100.0*Gflops_trf/Gflops_max, time_trs, Gflops_trs, 100.0*Gflops_trs/Gflops_max, time_trf_blas, Gflops_trf_blas, 100.0*Gflops_trf_blas/Gflops_max, time_trs_blas, Gflops_trs_blas, 100.0*Gflops_trs_blas/Gflops_max);

/************************************************
* free memory
************************************************/	

		free(A);
		free(B);
		free(b);
		free(x0);
		free(AB);
		free(C);
		free(dQ);
		free(Q);
		free(pQA0);
		free(pQA1);
		free(pQAN);
		free(dQpA0);
		free(dQpA1);
		free(dQpAN);
		free(pA);
		free(qb0);
		free(qb1);
		free(qbN);
		free(work_tv);
		free(tmp_tv);
		for(ii=0; ii<=N; ii++)
			{
			free(hpLA_tv[ii]);
			free(hdLA_tv[ii]);
			free(hpLe_tv[ii]);
			free(hxupi_tv[ii]);
			free(hres_tv[ii]);
			}

#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_MKL) || defined(REF_BLAS_NETLIB)
		free(QA0);
		free(QA1);
		free(QAN);
		free(qb0b);
		free(qb1b);
		free(qbNb);
		free(Le_tmp);
		for(ii=0; ii<=N; ii++)
			{
			free(hLA_tv[ii]);
			free(hLe_tv[ii]);
			}
#endif

		} // increase size

	fprintf(f, "];\n");
	fclose(f);


	return 0;

	}



