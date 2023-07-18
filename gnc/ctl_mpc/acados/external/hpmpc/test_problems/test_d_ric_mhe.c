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
#include "../include/block_size.h"
#include "../include/c_interface.h"
#include "../include/reference_code.h"
#include "tools.h"



#if defined(REF_BLAS_OPENBLAS)
void openblas_set_num_threads(int n_thread);
#endif



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
	dmcopy(nu, nu, I, nu, Bc+pp, nx); // TODO uncomment !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

#if defined(REF_BLAS_OPENBLAS)
	openblas_set_num_threads(1);
#endif
#if defined(REF_BLAS_BLIS)
	omp_set_num_threads(1);
#endif

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
	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line
	
	int nn[] = {4, 6, 8, 10, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 4000, 4000, 2000, 2000, 1000, 1000, 400, 400, 400, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	
	int vnx[] = {8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 512, 1024};
	int vnrep[] = {100, 100, 100, 100, 100, 100, 50, 50, 50, 20, 10, 10};
	int vN[] = {4, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256};

	int nx, nw, ny, ndN, N, nrep, Ns;
	int diag_R;

	int ll;
//	int ll_max = 77;
	int ll_max = 1;
	for(ll=0; ll<ll_max; ll++)
		{
		

		FILE* fid;
		double* yy;
		float* yy_temp;

		if(1)
			{
			fid = fopen("./test_problems/mhe_measure.dat", "r");
			if(fid==NULL)
				exit(-1);
			//printf("\nhola\n");
			int dummy_int = fscanf(fid, "%d %d %d %d", &nx, &nw, &ny, &Ns);
			//printf("\n%d %d %d %d\n", nx, nw, ny, Ns);
			yy_temp = (float*) malloc(ny*Ns*sizeof(float));
			yy = (double*) malloc(ny*Ns*sizeof(double));
			for(jj=0; jj<ny*Ns; jj++)
				{
				dummy_int = fscanf(fid, "%e", &yy_temp[jj]);
				yy[jj] = (double) yy_temp[jj];
				//printf("\n%f", yy[jj]);
				}
			//printf("\n");
			fclose(fid);
			#if 1
			N = 15; //Ns-1; // NN;
			nrep = NREP;//nnrep[ll];
			nx = 12;//nn[ll];
			nw = 5;//nn[ll];
			ny = 3;
			ndN = 0; //2;
			diag_R = 0;
			#else
			N = 10; //Ns-1; // NN;
			nrep = nnrep[ll];
			nx = nn[ll];
			nw = nn[ll];
			ny = 3;
			ndN = 0;
			diag_R = 0;
			#endif
			//printf("\nnx = %d; nw =  %d; ny =  %d; ndN = %d; N = %d\n\n", nx, nw, ny, ndN, N);
			}
		else if(ll_max==1)
			{
			nx = NX; // number of states (it has to be even for the mass-spring system test problem)
			nw = NU; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			ny = nx/2; // size of measurements vector
			N  = NN; // horizon lenght
			nrep = NREP;
			}
		else
			{
			nx = nn[ll]; // number of states (it has to be even for the mass-spring system test problem)
			nw = 2; // number of inputs (controllers) (it has to be at least 1 and at most nx/2 for the mass-spring system test problem)
			ny = nx/2; // size of measurements vector
			N  = 10; // horizon lenght
			nrep = nnrep[ll];
			}

		int rep;
		
	
		const int nz = nx+ny; // TODO delete
		const int nwx = nw+nx;
		const int anz = nal*((nz+nal-1)/nal);
		const int anx = nal*((nx+nal-1)/nal);
		const int anw = nal*((nw+nal-1)/nal);
		const int any = nal*((ny+nal-1)/nal);
		const int pnz = bs*((nz+bs-1)/bs);
		const int pnx = bs*((nx+bs-1)/bs);
		const int pnw = bs*((nw+bs-1)/bs);
		const int pny = bs*((ny+bs-1)/bs);
		const int pnx2 = bs*((2*nx+bs-1)/bs);
		const int pnwx = bs*((nw+nx+bs-1)/bs);
		const int cnz = ncl*((nz+ncl-1)/ncl);
		const int cnx = ncl*((nx+ncl-1)/ncl);
		const int cnw = ncl*((nw+ncl-1)/ncl);
		const int cny = ncl*((ny+ncl-1)/ncl);
		const int cnx2 = 2*(ncl*((nx+ncl-1)/ncl));
		const int cnwx = ncl*((nw+nx+ncl-1)/ncl);
		const int cnwx1 = ncl*((nw+nx+1+ncl-1)/ncl);
		const int cnf = cnz<cnx+ncl ? cnx+ncl : cnz;

		const int pad = (ncl-(nx+nw)%ncl)%ncl; // packing between AGL & P
		const int cnl = nx+nw+pad+cnx;
		const int pad2 = (ncl-(nx)%ncl)%ncl; // packing between AGL & P
		const int cnl2 = cnz<cnx+ncl ? nx+pad2+cnx+ncl : nx+pad2+cnz;
	
/************************************************
* dynamical system
************************************************/	

		double *A; d_zeros(&A, nx, nx); // states update matrix

		double *B; d_zeros(&B, nx, nw); // inputs matrix

		double *b; d_zeros(&b, nx, 1); // states offset
		double *x0; d_zeros(&x0, nx, 1); // initial state

		double Ts = 0.5; // sampling time
		mass_spring_system(Ts, nx, nw, N, A, B, b, x0);
	
		for(jj=0; jj<nx; jj++)
			b[jj] = 0.0;
	
		for(jj=0; jj<nx; jj++)
			x0[jj] = 0.0;
		x0[0] = 3.5;
		x0[1] = 3.5;
	
		double *C; d_zeros(&C, ny, nx); // inputs matrix
		for(jj=0; jj<ny; jj++)
			C[jj*(ny+1)] = 1.0;

//		d_print_mat(nx, nx, A, nx);
//		d_print_mat(nx, nw, B, nx);
//		d_print_mat(ny, nx, C, ny);
//		d_print_mat(nx, 1, b, nx);
//		d_print_mat(nx, 1, x0, nx);
	
		/* packed into contiguous memory */
		double *pA; d_zeros_align(&pA, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, A, nx, 0, pA, cnx);

		double *pG; d_zeros_align(&pG, pnx, cnw);
		d_cvt_mat2pmat(nx, nw, B, nx, 0, pG, cnw);
		
		double *pC; d_zeros_align(&pC, pny, cnx);
		d_cvt_mat2pmat(ny, nx, C, ny, 0, pC, cnx);
		
		double *pCA; d_zeros_align(&pCA, pnz, cnx);
		d_cvt_mat2pmat(ny, nx, C, ny, 0, pCA, cnx);
		d_cvt_mat2pmat(nx, nx, A, nx, ny, pCA+(ny/bs)*bs+ny%bs, cnx);

//		d_print_pmat(nx, nx, bs, pA, cnx);
//		d_print_pmat(nx, nw, bs, pG, cnw);
//		d_print_pmat(ny, nx, bs, pC, cnx);

/************************************************
* cost function
************************************************/	

		double *R; d_zeros(&R, nw, nw);
		for(jj=0; jj<nw; jj++)
			R[jj*(nw+1)] = 1.0;

		double *Q; d_zeros(&Q, ny, ny);
		for(jj=0; jj<ny; jj++)
			Q[jj*(ny+1)] = 1.0;

		double *Qx; d_zeros(&Qx, nx, nx);
		for(jj=0; jj<ny; jj++)
			for(ii=0; ii<ny; ii++)
				Qx[ii+nx*jj] = Q[ii+ny*jj];

		double *L0; d_zeros(&L0, nx, nx);
		for(jj=0; jj<nx; jj++)
			L0[jj*(nx+1)] = 1.0;

		double *q; d_zeros_align(&q, any, 1);
		for(jj=0; jj<ny; jj++)
			q[jj] = 0.0;

		double *r; d_zeros_align(&r, anw, 1);
		for(jj=0; jj<nw; jj++)
			r[jj] = 1.0;

		double *f; d_zeros_align(&f, anx, 1);
		for(jj=0; jj<nx; jj++)
			f[jj] = jj;//1.0; //b[jj]; //1.0;

		/* packed into contiguous memory */
		double *pR; d_zeros_align(&pR, pnw, cnw);
		d_cvt_mat2pmat(nw, nw, R, nw, 0, pR, cnw);

		double *pQ; d_zeros_align(&pQ, pny, cny);
		d_cvt_mat2pmat(ny, ny, Q, ny, 0, pQ, cny);

//		d_print_pmat(nw, nw, bs, pQ, cnw);
//		d_print_pmat(ny, ny, bs, pR, cny);

/************************************************
* compound quantities
************************************************/	
		
		double *pRG; d_zeros_align(&pRG, pnwx, cnw);
		d_cvt_mat2pmat(nw, nw, R, nw, 0, pRG, cnw);
		d_cvt_mat2pmat(nx, nw, B, nx, nw, pRG+(nw/bs)*bs*cnw+nw%bs, cnw);
		//d_print_pmat(nw+nx, nw, bs, pRG, cnw);

		double *pQA; d_zeros_align(&pQA, pnx2, cnx);
		d_cvt_mat2pmat(ny, ny, Q, ny, 0, pQA, cnx);
		d_cvt_mat2pmat(nx, nx, A, nx, nx, pQA+(nx/bs)*bs*cnx+nx%bs, cnx);
		//d_print_pmat(2*nx, cnx, bs, pQA, cnx);
		//exit(1);

/************************************************
* series of matrices
************************************************/	

		double *hpA[N];
		double *hpCA[N];
		double *hpG[N];
		double *hpC[N+1];
		double *hpR[N];
		double *hpQ[N+1];
		double *hpLp[N+1];
		double *hdLp[N+1];
		double *hpLp2[N+1];
		double *hpLe[N+1];
		double *hq[N];
		double *hr[N+1];
		double *hf[N];
		double *hxe[N+1];
		double *hxp[N+1];
		double *hw[N];
		double *hy[N+1];
		double *hlam[N];

		double *hpRG[N];
		double *hpQA[N+1];
		double *hpGLr[N];
		double *hpALe[N+1];
		double *hrr[N];
		double *hqq[N+1];
		double *hff[N+1];
		double *p_hrr; d_zeros_align(&p_hrr, anw, N);
		double *p_hqq; d_zeros_align(&p_hqq, anx, N+1);
		double *p_hff; d_zeros_align(&p_hff, anx, N+1);

		double *p_hxe; d_zeros_align(&p_hxe, anx, N+1);
		double *p_hxp; d_zeros_align(&p_hxp, anx, N+1);
		double *p_hw; d_zeros_align(&p_hw, anw, N);
		double *p_hy; d_zeros_align(&p_hy, any, N+1);
		double *p_hlam; d_zeros_align(&p_hlam, anx, N+1);

		double *hq_res[N+1];
		double *hr_res[N];
		double *hf_res[N+1];
		double *p_hq_res; d_zeros_align(&p_hq_res, anx, N+1);
		double *p_hr_res; d_zeros_align(&p_hr_res, anw, N);
		double *p_hf_res; d_zeros_align(&p_hf_res, anx, N+1);

		for(jj=0; jj<N; jj++)
			{
			hpA[jj] = pA;
			hpCA[jj] = pCA;
			hpG[jj] = pG;
			hpC[jj] = pC;
			hpR[jj] = pR;
			hpQ[jj] = pQ;
			d_zeros_align(&hpLp[jj], pnx, cnl);
			d_zeros_align(&hdLp[jj], anx, 1);
			d_zeros_align(&hpLp2[jj], pnz, cnl2);
			d_zeros_align(&hpLe[jj], pnz, cnf);
			hr[jj] = r;
			hq[jj] = q;
			hf[jj] = f;

			hpRG[jj] = pRG;
			hpQA[jj] = pQA;
			d_zeros_align(&hpGLr[jj], pnwx, cnw);
			d_zeros_align(&hpALe[jj], pnx2, cnx2);
			hrr[jj] = p_hrr+jj*anw;
			hqq[jj] = p_hqq+jj*anx;
			hff[jj] = p_hff+jj*anx;

			hxe[jj] = p_hxe+jj*anx; //d_zeros_align(&hxe[jj], anx, 1);
			hxp[jj] = p_hxp+jj*anx; //d_zeros_align(&hxp[jj], anx, 1);
			hw[jj] = p_hw+jj*anw; //d_zeros_align(&hw[jj], anw, 1);
			hy[jj] = p_hy+jj*any; //d_zeros_align(&hy[jj], any, 1);
			hlam[jj] = p_hlam+jj*anx; //d_zeros_align(&hlambda[jj], anx, 1);

			hq_res[jj] = p_hq_res+jj*anx;
			hr_res[jj] = p_hr_res+jj*anw;
			hf_res[jj] = p_hf_res+jj*anx;
			}

		hpC[N] = pC;
		hpQ[N] = pQ;
		d_zeros_align(&hpLp[N], pnx, cnl);
		d_zeros_align(&hdLp[N], anx, 1);
		d_zeros_align(&hpLp2[N], pnz, cnl2);
		d_zeros_align(&hpLe[N], pnz, cnf);
		hq[N] = q;

		// equality constraints on the states at the last stage
		double *D; d_zeros(&D, ndN, nx);
		for(ii=0; ii<ndN; ii++) D[ii*(ndN+1)] = 1;
		//D[0+ndN*0] = 1;
		//D[1+ndN*(nx-1)] = 1;
		double *d; d_zeros_align(&d, ndN, 1);
		for(ii=0; ii<ndN; ii++) d[ii] = ii;
		//d[0] = 1;
		//d[1] = 0;
		const int pnxdN = bs*((nx+ndN+bs-1)/bs);
		double *pCtQC; d_zeros_align(&pCtQC, pnxdN, cnx);
		d_cvt_mat2pmat(ny, ny, Q, ny, 0, pCtQC, cnx);
		d_cvt_mat2pmat(ndN, nx, D, ndN, nx, pCtQC+nx/bs*bs*cnx+nx%bs, cnx);
		//d_print_pmat(nx+ndN, nx, bs, pCtRC, cnx);
		hpQA[N] = pCtQC; // there is not A_N
		d_zeros_align(&hpALe[N], pnxdN, cnx2); // there is not A_N: pnx not pnx2
		hqq[N] = p_hqq+N*anx;
		hff[N] = p_hff+N*anx;
		const int pndN = bs*((ndN+bs-1)/bs);
		const int cndN = ncl*((ndN+ncl-1)/ncl);
		double *Ld; d_zeros_align(&Ld, pndN, cndN);
		double *d_res; d_zeros_align(&d_res, pndN, 1);



		hxe[N] = p_hxe+N*anx; //d_zeros_align(&hxe[N], anx, 1);
		hxp[N] = p_hxp+N*anx; //d_zeros_align(&hxp[N], anx, 1);
		hy[N] = p_hy+N*any; //d_zeros_align(&hy[N], any, 1);
		hlam[N] = p_hlam+N*anx; //d_zeros_align(&hlambda[jj], anx, 1);

		hf_res[N] = p_hf_res+N*anx;
		hq_res[N] = p_hq_res+N*anx;

		// initialize hpLp[0] with the cholesky factorization of /Pi_p
		d_cvt_mat2pmat(nx, nx, L0, nx, 0, hpLp[0]+(nx+nw+pad)*bs, cnl);
		for(ii=0; ii<nx; ii++) hdLp[0][ii] = 1.0/L0[ii*(nx+1)];
		d_cvt_mat2pmat(nx, nx, L0, nx, ny, hpLp2[0]+(ny/bs)*bs+ny%bs+(nx+pad2+ny)*bs, cnl2);
		dtrtr_l_lib(nx, ny, hpLp2[0]+(ny/bs)*bs*cnl2+ny%bs+(nx+pad2+ny)*bs, cnl2, 0, hpLp2[0]+(nx+pad2+ncl)*bs, cnl2);	
		//d_print_pmat(nx, cnl, bs, hpLp[0], cnl);
		//d_print_pmat(nz, cnl2, bs, hpLp2[0], cnl2);

		// buffer for L0
		double *pL0; d_zeros_align(&pL0, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, L0, nx, 0, pL0, cnx);
		// invert L0 in hpALe[0]
		dtrinv_lib(nx, pL0, cnx, hpALe[0], cnx2);
		double *pL0_inv; d_zeros_align(&pL0_inv, pnx, cnx);
		dtrinv_lib(nx, pL0, cnx, pL0_inv, cnx);
		//d_print_pmat(nx, nx, bs, pL0, cnx);
		//d_print_pmat(nx, nx, bs, pL0_inv, cnx);
		//d_print_pmat(pnx2, cnx2, bs, hpALe[0], cnx2);
		//exit(1);

		//double *work; d_zeros_align(&work, pny*cnx+pnz*cnz+anz+pnz*cnf+pnw*cnw, 1);
		double *work; d_zeros_align(&work, 2*pny*cnx+anz+pnw*cnw+pnx*cnx, 1);
		//printf("\nciao %d %d %d %d %d %d\n", pny, cnx, anz, pnw, cnw, pnx);

		double *work2; d_zeros_align(&work2, 2*pny*cnx+pnw*cnw+pnx*cnw+2*pnx*cnx+anz, 1);

		double *work3; d_zeros_align(&work3, pnx*cnl+anx, 1);
		double *work4; d_zeros_align(&work4, 4*anx+2*(anx+anw), 1);
//		for(jj=0; jj<2*pny*cnx+anz+pnw*cnw+pnx*cnx; jj++)
//			work[jj] = -100.0;

		// measurements
		for(jj=0; jj<=N; jj++)
			for(ii=0; ii<ny; ii++)
				hy[jj][ii] = yy[jj*ny+ii];

		//d_print_mat(ny, N+1, hy[0], any);

		// initial guess
		for(ii=0; ii<nx; ii++)
			x0[ii] = 0.0;
		for(ii=0; ii<nx; ii++)
			hxp[0][ii] = x0[ii];



		// information filter - solution
		double *y_temp; d_zeros_align(&y_temp, any, 1);
		for(ii=0; ii<N; ii++) for(jj=0; jj<nw; jj++) hrr[ii][jj] = r[jj];
		for(ii=0; ii<N; ii++) for(jj=0; jj<nx; jj++) hff[ii][jj] = f[jj];
		for(jj=0; jj<ndN; jj++) hff[N][jj] = d[jj];
		for(ii=0; ii<=N; ii++) 
			{
			for(jj=0; jj<ny; jj++) y_temp[jj] = - q[jj];
			//d_print_mat(1, ny, y_temp, 1);
			dsymv_lib(ny, ny, hpQ[ii], cny, hy[ii], -1, y_temp, y_temp);
			//d_print_mat(1, ny, y_temp, 1);
			dgemv_t_lib(ny, nx, hpC[ii], cnx, y_temp, 0, hqq[ii], hqq[ii]);
			//d_print_mat(1, nx, hqq[ii], 1);
			//if(ii==9)
			//exit(1);
			}
		//exit(1);




/************************************************
* new low-level mhe_if interface
************************************************/	

		int nrows = pnx>pnw ? 2*pnx : pnx+pnw;
		int ncols = cnwx1;

		double *pQRAG; d_zeros_align(&pQRAG, nrows, ncols);

		if(nx>=nw)
			{
			d_cvt_mat2pmat(ny, ny, Q, ny, 0, pQRAG, cnwx1);
			d_cvt_mat2pmat(nx, nx, A, nx, 0, pQRAG+pnx*cnwx1, cnwx1);
			d_cvt_mat2pmat(nw, nw, R, nw, 0, pQRAG+(pnx-pnw)*cnwx1+nx*bs, cnwx1);
			d_cvt_mat2pmat(nx, nw, B, nx, 0, pQRAG+pnx*cnwx1+nx*bs, cnwx1);
			//d_print_pmat(nrows, ncols, bs, pQRAG, ncols);
			if(nx>pnx-nx)
				d_cvt_mat2pmat(pnx-nx, nx, A+(nx-pnx+nx), nx, nx, pQRAG+nx/bs*bs*cnwx1+nx%bs, cnwx1);
			else
				d_cvt_mat2pmat(nx, nx, A, nx, nx, pQRAG+nx/bs*bs*cnwx1+nx%bs, cnwx1);
			if(nx>pnw-nw)
				d_cvt_mat2pmat(pnw-nw, nw, B+(nx-pnw+nw), nx, nw, pQRAG+(pnx-pnw+nw/bs*bs)*cnwx1+nw%bs+nx*bs, cnwx1);
			else
				d_cvt_mat2pmat(nx, nw, B, nx, nw, pQRAG+(pnx-pnw+nw/bs*bs)*cnwx1+nw%bs+nx*bs, cnwx1);
			//d_print_pmat(nrows, ncols, bs, pQRAG, ncols);
			}
		else
			{
			d_cvt_mat2pmat(ny, ny, Q, ny, 0, pQRAG+(pnw-pnx)*cnwx1, cnwx1);
			d_cvt_mat2pmat(nx, nx, A, nx, 0, pQRAG+pnw*cnwx1, cnwx1);
			d_cvt_mat2pmat(nw, nw, R, nw, 0, pQRAG+nx*bs, cnwx1);
			d_cvt_mat2pmat(nx, nw, B, nx, 0, pQRAG+pnw*cnwx1+nx*bs, cnwx1);
			//d_print_pmat(nrows, ncols, bs, pQRAG, ncols);
			if(nx>pnx-nx)
				d_cvt_mat2pmat(pnx-nx, nx, A+(nx-pnx+nx), nx, nx, pQRAG+(pnw-pnx+nx/bs*bs)*cnwx1+nx%bs, cnwx1);
			else
				d_cvt_mat2pmat(nx, nx, A, nx, nx, pQRAG+(pnw-pnx+nx/bs*bs)*cnwx1+nx%bs, cnwx1);
			if(nx>pnw-nw)
				d_cvt_mat2pmat(pnw-nw, nw, B+(nx-pnw+nw), nx, nw, pQRAG+nw/bs*bs*cnwx1+nw%bs+nx*bs, cnwx1);
			else
				d_cvt_mat2pmat(nx, nw, B, nx, nw, pQRAG+nw/bs*bs*cnwx1+nw%bs+nx*bs, cnwx1);
			//d_print_pmat(nrows, ncols, bs, pQRAG, ncols);
			}

		double *pQD; d_zeros_align(&pQD, pnx+pndN, cnx);
		d_cvt_mat2pmat(ny, ny, Q, ny, 0, pQD, cnx);
		d_cvt_mat2pmat(ndN, nx, D, ndN, 0, pQD+pnx*cnx, cnx);
		//d_print_pmat(pnx+pndN, cnx, bs, pQD, cnx);
		if(ndN>pnx-nx)
			d_cvt_mat2pmat(pnx-nx, nx, D+(ndN-pnx+nx), ndN, nx, pQD+nx/bs*bs*cnx+nx%bs, cnx);
		else
			d_cvt_mat2pmat(ndN, nx, D, ndN, nx, pQD+nx/bs*bs*cnx+nx%bs, cnx);
		//d_print_pmat(pnx+pndN, cnx, bs, pQD, cnx);
		//exit(1);




		double *hpQRAG[N+1];
		double *hpLAG[N+1];
		double *hpLe2[N+1];

		for(ii=0; ii<N; ii++)	
			{
			hpQRAG[ii] = pQRAG;
			d_zeros_align(&hpLAG[ii], nrows, ncols);
			d_zeros_align(&hpLe2[ii], pnx, cnx);
			}
		hpQRAG[N] = pQD;
		d_zeros_align(&hpLAG[N], pnx+pndN, cnx);
		d_zeros_align(&hpLe2[N], pnx, cnx);
		d_cvt_mat2pmat(nx, nx, L0, nx, 0, hpLe2[0], cnx);
		//d_print_pmat(nx, nx, bs, hpLe2[0], cnx);



		double **dummy;
#if 0

		struct timeval tv10, tv11, tv12;

		// double precision
		gettimeofday(&tv10, NULL); // start

		for(ii=0; ii<1; ii++)
		//for(ii=0; ii<nrep; ii++)
			{

			d_ric_trf_mhe_if(nx, nw, ndN, N, hpQRAG, diag_R, hpLe2, hpLAG, Ld, work3);
			//d_ric_trf_mhe_if(nx, nw, ndN, N, hpQA, hpRG, diag_R, hpALe, hpGLr, Ld, work3);

			}

		gettimeofday(&tv11, NULL); // stop

		for(ii=0; ii<1; ii++)
		//for(ii=0; ii<nrep; ii++)
			{

			d_ric_trs_mhe_if(nx, nw, ndN, N, hpLe2, hpLAG, Ld, hqq, hrr, hff, hxp, hxe, hw, hlam, work3);

			}

		gettimeofday(&tv12, NULL); // stop

		float time_trf_mhe_if_new = (float) (tv11.tv_sec-tv10.tv_sec)/(nrep+0.0)+(tv11.tv_usec-tv10.tv_usec)/(nrep*1e6);
		float time_trs_mhe_if_new = (float) (tv12.tv_sec-tv11.tv_sec)/(nrep+0.0)+(tv12.tv_usec-tv11.tv_usec)/(nrep*1e6);

		printf("\ntime = %e\t%e\n\n", time_trf_mhe_if_new, time_trs_mhe_if_new);




		//exit(1);
#endif


/************************************************
* reference code
************************************************/	

		double *hA[N];
		double *hG[N];
		double *hQ[N+1];
		double *hR[N];
		double *hAGU[N];
		double *hUp[N+1];
		double *hUe[N+1];
		double *hUr[N];
		double *Ud;
		double *work_ref;

		for(ii=0; ii<N; ii++)
			{
			hA[ii] = A;
			hG[ii] = B;
			hQ[ii] = Qx;
			hR[ii] = R;
			d_zeros(&hAGU[ii], nx, nx+nw);
			d_zeros(&hUp[ii], nx, nx);
			d_zeros(&hUe[ii], nx, nx);
			d_zeros(&hUr[ii], nw, nw);
			}
		hA[N] = D;
		hQ[N] = Qx;
		d_zeros(&hAGU[N], ndN, nx);
		d_zeros(&hUp[N], nx, nx);
		d_zeros(&hUe[N], nx, nx);
		d_zeros(&Ud, ndN, ndN);
		d_zeros(&work_ref, nx+nw, 1);

		for(ii=0; ii<nx*nx; ii++)
			hUp[0][ii] = L0[ii];



		#if 0

		printf("\nfactorization\n");
		d_ric_trf_mhe_if_blas( nx, nw, ndN, N, hA, hG, hQ, hR, hAGU, hUp, hUe, hUr, Ud);

		printf("\nsolution\n");
		d_ric_trs_mhe_if_blas( nx, nw, ndN, N, hAGU, hUp, hUe, hUr, Ud, hqq, hrr, hff, hxp, hxe, hw, hlam, work_ref);

		//d_print_mat(nx, nx, hUe[N], nx);
		//exit(1);

		#endif




/************************************************
* high-level interface
************************************************/	

#if 0
		int kk;

		double *AA; d_zeros(&AA, nx, nx*N);
		//for(ii=0; ii<N; ii++) for(jj=0; jj<nx; jj++) for(ll=0; ll<nx; ll++) AA[ll+nx*jj+nx*nx*ii] = A[ll+nx*jj];
		for(ii=0; ii<N; ii++) for(jj=0; jj<nx; jj++) for(kk=0; kk<nx; kk++) AA[jj+nx*kk+nx*nx*ii] = A[kk+nx*jj];

		double *GG; d_zeros(&GG, nx, nw*N);
		//for(ii=0; ii<N; ii++) for(jj=0; jj<nw; jj++) for(ll=0; ll<nx; ll++) GG[ll+nx*jj+nx*nw*ii] = B[ll+nx*jj];
		for(ii=0; ii<N; ii++) for(jj=0; jj<nw; jj++) for(kk=0; kk<nx; kk++) GG[jj+nw*kk+nx*nw*ii] = B[kk+nx*jj];

		double *ff; d_zeros(&ff, nx, N);
		for(ii=0; ii<N; ii++) for(jj=0; jj<nx; jj++) ff[jj+nx*ii] = f[jj];

		double *DD; d_zeros(&DD, ndN, nx);
		//for(jj=0; jj<nx; jj++) for(ll=0; ll<ndN; ll++) DD[ll+ndN*jj] = D[ll+ndN*jj];
		for(jj=0; jj<nx; jj++) for(kk=0; kk<ndN; kk++) DD[jj+nx*kk] = D[kk+ndN*jj];

		double *dd; d_zeros(&dd, ndN, 1);
		for(kk=0; kk<ndN; kk++) dd[kk] = d[kk];

		double *RR; d_zeros(&RR, nw, nw*N);
		for(ii=0; ii<N; ii++) for(jj=0; jj<nw*nw; jj++) RR[jj+nw*nw*ii] = R[jj];

		double *QQ; d_zeros(&QQ, nx, nx*N);
		for(ii=0; ii<N; ii++) 
			{
			for(jj=0; jj<ny; jj++) for(kk=0; kk<ny; kk++) QQ[kk+nx*jj+nx*nx*ii] = Q[kk+ny*jj];
			//for(jj=ny; jj<nx; jj++) QQ[jj+nx*jj+nx*nx*ii] = 1e-8;
			}

		double *Qf; d_zeros(&Qf, nx, nx);
		for(jj=0; jj<ny; jj++) for(kk=0; kk<ny; kk++) Qf[kk+nx*jj] = Q[kk+ny*jj];

		double *rr; d_zeros(&rr, nw, N);
		for(ii=0; ii<N; ii++) for(jj=0; jj<nw; jj++) rr[jj+nw*ii] = r[jj];

		double *qq; d_zeros(&qq, nx, N);
		for(ii=0; ii<N; ii++) for(jj=0; jj<ny; jj++) qq[jj+nx*ii] = q[jj];
		double *yy_tmp; d_zeros_align(&yy_tmp, any, 1);
		for(ii=0; ii<N; ii++) 
			{
			for(jj=0; jj<ny; jj++) yy_tmp[jj] = - q[jj];
			dsymv_lib(ny, ny, hpQ[ii], cny, hy[ii], yy_tmp, -1);
			dgemv_t_lib(ny, nx, hpC[ii], cnx, yy_tmp, &qq[ii*nx], 0);
			}

		double *qf; d_zeros(&qf, nx, 1);
//		for(jj=0; jj<ny; jj++) qf[jj] = q[jj];
//		if(ndN>0) 
//			{
			for(jj=0; jj<ny; jj++) yy_tmp[jj] = - q[jj];
			dsymv_lib(ny, ny, hpQ[N], cny, hy[N], yy_tmp, -1);
			dgemv_t_lib(ny, nx, hpC[N], cnx, yy_tmp, qf, 0);
//			}

		double *xx0; d_zeros(&xx0, nx, 1);

		double *LL0; d_zeros(&LL0, nx, nx);

		double *xxe; d_zeros(&xxe, nx, N+1);

		double *LLe; d_zeros(&LLe, nx, nx);

		double *ww; d_zeros(&ww, nw, N);

		double *llam; d_zeros(&llam, nx, N+1);

		double *work_high_level; d_zeros(&work_high_level, hpmpc_ric_mhe_if_dp_work_space(nx, nw, ny, ndN, N), 1);

		double *dummy0;

		struct timeval tv00, tv01;

		int error_code;

		printf("\nhigh-level\n");

		// double precision
		gettimeofday(&tv00, NULL); // start

		for(ii=0; ii<nrep; ii++)
			{

			for(jj=0; jj<nx; jj++) xx0[jj] = x0[jj];
			for(jj=0; jj<nx*nx; jj++) LL0[jj] = L0[jj];

			//error_code = fortran_order_riccati_mhe_if( 'd', 2, nx, nw, 0, ndN, N, AA, GG, dummy, ff, DD, dd, RR, QQ, Qf, rr, qq, qf, dummy, xx0, LL0, xxe, LLe, ww, llam, work_high_level);
			error_code = c_order_riccati_mhe_if( 'd', 2, nx, nw, 0, ndN, N, AA, GG, dummy0, ff, DD, dd, RR, QQ, Qf, rr, qq, qf, dummy0, xx0, LL0, xxe, LLe, ww, llam, work_high_level);

			//if(error_code)
			//	break;

			}

		gettimeofday(&tv01, NULL); // stop

		float time_mhe_if_high_level = (float) (tv01.tv_sec-tv00.tv_sec)/(nrep+0.0)+(tv01.tv_usec-tv00.tv_usec)/(nrep*1e6);

		printf("\nhigh-level interface for MHE_if\n\nerror_code: %d, time = %e\n\n", error_code, time_mhe_if_high_level);

		//d_print_mat(nx, N+1, xxe, nx);
		//d_print_mat(nw, N, ww, nw);

		free(AA);
		free(GG);
		free(ff);
		free(DD);
		free(dd);
		free(RR);
		free(QQ);
		free(Qf);
		free(rr);
		free(qq);
		free(qf);
		free(xx0);
		free(LL0);
		free(xxe);
		free(LLe);
		free(ww);
		free(llam);
		free(work_high_level);
		free(yy_tmp);

		//exit(1);
#endif


/************************************************
* call the solver
************************************************/	

		//d_print_mat(nx, nx, A, nx);
		//d_print_mat(nx, nw, B, nx);

		//d_ric_trf_mhe_test(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hpQ, hpR, hpLe, work);
		d_ric_trf_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpR, hpQ, hpLe, work);

		// estimation
		d_ric_trs_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpR, hpQ, hpLe, hr, hq, hf, hxp, hxe, hw, hy, 0, hlam, work);

#if 0
		// print solution
		printf("\nx_e\n");
		d_print_mat(nx, N+1, hxe[0], anx);
#endif
	
		// smooth estimation
		d_ric_trs_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpR, hpQ, hpLe, hr, hq, hf, hxp, hxe, hw, hy, 1, hlam, work);

		//d_print_pmat(nx, nx, bs, hpLp[N-1]+(nx+nw+pad)*bs, cnl);
		//d_print_pmat(nx, nx, bs, hpLp[N]+(nx+nw+pad)*bs, cnl);
		//d_print_pmat(nx, nx, bs, hpLe[N-1]+ncl*bs, cnf);
		//d_print_pmat(nx, nx, bs, hpLe[N]+ncl*bs, cnf);

#if 1
		printf("\nx_s\n");
		//d_print_mat(nx, N+1, hxp[0], anx);
		d_print_mat(nw, N, hw[0], anw);
		d_print_mat(nx, N+1, hxe[0], anx);
		//d_print_mat(nx, N, hlam[0], anx);
#endif

		// information filter - factorization
		//d_ric_trf_mhe_if(nx, nw, ndN, N, hpQA, hpRG, diag_R, hpALe, hpGLr, Ld, work3);
		d_ric_trf_mhe_if(nx, nw, ndN, N, hpQRAG, diag_R, hpLe2, hpLAG, Ld, work3);

		// information filter - solution
		//d_ric_trs_mhe_if(nx, nw, ndN, N, hpALe, hpGLr, Ld, hqq, hrr, hff, hxp, hxe, hw, hlam, work3);
		d_ric_trs_mhe_if(nx, nw, ndN, N, hpLe2, hpLAG, Ld, hqq, hrr, hff, hxp, hxe, hw, hlam, work3);
		//d_ric_trs_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpQ, hpR, hpLe, hq, hr, hf, hxp, hxe, hw, hy, 1, hlam, work);

		//d_print_pmat(nx, nx, bs, hpALe[N-1], cnx2);
		//d_print_pmat(nx, nx, bs, hpALe[N], cnx2);
		//d_print_pmat(nx, nx, bs, hpALe[N-2]+cnx*bs, cnx2);
		//d_print_pmat(nx, nx, bs, hpALe[N-1]+cnx*bs, cnx2);
		//d_print_pmat(nx, nx, bs, hpALe[N]+cnx*bs, cnx2);
		//d_print_pmat(nx, nx, bs, hpRA[N], cnx);

#if 1
		printf("\nx_s_if\n");
		//d_print_mat(nx, N+1, hxp[0], anx);
		d_print_mat(nw, N, hw[0], anw);
		d_print_mat(nx, N+1, hxe[0], anx);
		//d_print_mat(nx, N, hlam[0], anx);
		//exit(1);
#endif

		//d_print_pmat(nw, nw, bs, hpQ[0], cnw);
		//d_print_pmat(nx, nw, bs, hpG[0], cnw);
		//d_print_mat(nw, 1, hq[0], nw);
		//d_print_mat(nw, 1, hw[0], nw);
		//d_print_mat(nx, 1, hlam[0], nx);
		//exit(3);

#if 0
		int nZ = nw+nx+1;
		int pnZ = (nw+nx+1+bs-1)/bs*bs;
		int cnZ = (nw+nx+1+ncl-1)/ncl*ncl;

		int cnL = cnZ>cnx+ncl ? cnZ : cnx+ncl;

		double *(hpRSQrq[N+1]); 
		for(ii=0; ii<=N; ii++)
			{
			d_zeros_align(&hpRSQrq[ii], pnZ, cnZ);
			d_cvt_mat2pmat(nw, nw, R, nw, 0, hpRSQrq[ii], cnZ);
			d_cvt_mat2pmat(ny, ny, Q, ny, nw, hpRSQrq[ii]+nw/bs*bs*cnZ+nw%bs+nw*bs, cnZ);
			d_cvt_mat2pmat(1, nw, r, 1, nw+nx, hpRSQrq[ii]+(nw+nx)/bs*bs*cnZ+(nw+nx)%bs, cnZ);
			d_cvt_mat2pmat(1, nx, hqq[ii], 1, nw+nx, hpRSQrq[ii]+(nw+nx)/bs*bs*cnZ+(nw+nx)%bs+nw*bs, cnZ);
			//d_print_pmat(nZ, nZ, bs, hpRSQrq[ii], cnZ);
			}

		double *pP0; d_zeros_align(&pP0, pnx, cnx);
		d_cvt_mat2pmat(nx, nx, L0, nx, 0, pP0, cnx);
		//d_print_pmat(nx, nx, bs, pP0, cnx);
		dgead_lib(nx, nx, 1.0, 0, pP0, cnx, nw, hpRSQrq[0]+nw/bs*bs*cnZ+nw%bs+nw*bs, cnZ); 
		//d_print_pmat(nZ, nZ, bs, hpRSQrq[0], cnZ);

		double *pBAbt; d_zeros_align(&pBAbt, pnZ, cnx);
		d_cvt_tran_mat2pmat(nx, nw, B, nx, 0, pBAbt, cnx);
		d_cvt_tran_mat2pmat(nx, nx, A, nx, nw, pBAbt+nw/bs*bs*cnx+nw%bs, cnx);
		d_cvt_mat2pmat(1, nx, f, 1, nw+nx, pBAbt+(nw+nx)/bs*bs*cnx+(nw+nx)%bs, cnx);
		//d_print_pmat(nZ, nx, bs, pBAbt, cnx);

		double *(hpBAbt[N]);
		for(ii=0; ii<N; ii++)
			{
			hpBAbt[ii] = pBAbt;
			}

		double *(hpLam[N+1]);
		for(ii=0; ii<=N; ii++)
			{
			d_zeros_align(&hpLam[ii], pnZ, cnL);
			}

		double *work_ric; d_zeros_align(&work_ric, pnZ, cnx);
		double *diag_ric; d_zeros_align(&diag_ric, pnZ, 1);

		double *hux_mat; d_zeros_align(&hux_mat, pnZ, N+1);
		double *(hux[N+1]);
		for(ii=0; ii<=N; ii++)
			{
			hux[ii] = hux_mat+ii*pnZ;
			}

		double **pdummy;

		d_back_ric_sv(N, nx, nw, hpBAbt, hpRSQrq, 0, pdummy, pdummy, 0, hux, hpLam, work_ric, diag_ric, 0, pdummy, 0, pdummy, 0, 0, 0, pdummy, pdummy, pdummy);

		d_print_mat(nw, N+1, hux_mat, pnZ);
		d_print_mat(nx, N+1, hux_mat+nw, pnZ);

		exit(1);

#endif

		// compute residuals
		double *p0; d_zeros_align(&p0, anx, 1);
		double *x_temp; d_zeros_align(&x_temp, anx, 1);
		dtrmv_u_t_lib(nx, pL0_inv, cnx, x0, 0, x_temp);
		dtrmv_u_n_lib(nx, pL0_inv, cnx, x_temp, 0, p0);
		d_res_mhe_if(nx, nw, ndN, N, hpQA, hpRG, pL0_inv, hqq, hrr, hff, p0, hxe, hw, hlam, hq_res, hr_res, hf_res, work4);

//		printf("\nprint residuals\n\n");
//		d_print_mat(nx, N+1, hq_res[0], anx);
//		d_print_mat(nw, N, hr_res[0], anw);
//		d_print_mat(nx, N, hf_res[0], anx);
//		d_print_mat(ndN, 1, hf_res[0]+N*anx, anx);

		//return 0;
		//exit(1);

		if(0 && PRINTRES)
			{
			// print solution
			printf("\nx_p\n");
			d_print_mat(nx, N+1, hxp[0], anx);
			printf("\nx_s\n");
			d_print_mat(nx, N+1, hxe[0], anx);
			printf("\nw\n");
			d_print_mat(nw, N+1, hw[0], anw);
			//printf("\nL_p\n");
			//d_print_pmat(nx, nx, bs, hpLp[0]+(nx+nw+pad)*bs, cnl);
			//d_print_mat(1, nx, hdLp[0], 1);
			//d_print_pmat(nx, nx, bs, hpLp[1]+(nx+nw+pad)*bs, cnl);
			//d_print_mat(1, nx, hdLp[1], 1);
			//d_print_pmat(nx, nx, bs, hpLp[2]+(nx+nw+pad)*bs, cnl);
			//d_print_mat(1, nx, hdLp[2], 1);
			//d_print_pmat(nx, nx, bs, hpLp[N]+(nx+nw+pad)*bs, cnl);
			//d_print_mat(1, nx, hdLp[N], 1);
			//printf("\nL_p\n");
			//d_print_pmat(nz, nz, bs, hpLp2[0]+(nx+pad2)*bs, cnl2);
			//d_print_pmat(nz, nz, bs, hpLp2[1]+(nx+pad2)*bs, cnl2);
			//d_print_pmat(nz, nz, bs, hpLp2[2]+(nx+pad2)*bs, cnl2);
			//printf("\nL_e\n");
			//d_print_pmat(nz, nz, bs, hpLe[0], cnf);
			//d_print_pmat(nz, nz, bs, hpLe[1], cnf);
			//d_print_pmat(nz, nz, bs, hpLe[2], cnf);
			//d_print_pmat(nx, nx, bs, hpA[0], cnx);
			}


		// timing 
		struct timeval tv0, tv1, tv2, tv3, tv4, tv5, tv6, tv7, tv8;

		// double precision
		gettimeofday(&tv0, NULL); // start

		// factorize
		for(rep=0; rep<nrep; rep++)
			{
			//d_ric_trf_mhe_test(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hpQ, hpR, hpLe, work);
			d_ric_trf_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpR, hpQ, hpLe, work);
			}

		gettimeofday(&tv1, NULL); // start

		// solve
		for(rep=0; rep<nrep; rep++)
			{
			d_ric_trs_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpR, hpQ, hpLe, hr, hq, hf, hxp, hxe, hw, hy, 1, hlam, work);
			}

		gettimeofday(&tv2, NULL); // start

		// factorize
		for(rep=0; rep<nrep; rep++)
			{
			//d_print_pmat(nx, nx, bs, hpLe[N]+(ncl)*bs, cnf);
			//d_print_pmat(nx, nx, bs, hpLp[N]+(nx+nw+pad)*bs, cnl);
			//d_ric_trf_mhe_test(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hpQ, hpR, hpLe, work);
			d_ric_trf_mhe_end(nx, nw, ny, N, hpCA, hpG, hpC, hpLp2, hpR, hpQ, hpLe, work2);
			}

		gettimeofday(&tv3, NULL); // start

		// solve
		for(rep=0; rep<nrep; rep++)
			{
			d_ric_trs_mhe_end(nx, nw, ny, N, hpA, hpG, hpC, hpLp2, hpR, hpQ, hpLe, hr, hq, hf, hxp, hxe, hy, work2);
			}

		gettimeofday(&tv4, NULL); // start

		// factorize information filter
		for(rep=0; rep<nrep; rep++)
			{
			//d_ric_trf_mhe_if(nx, nw, ndN, N, hpQA, hpRG, diag_R, hpALe, hpGLr, Ld, work3);
			d_ric_trf_mhe_if(nx, nw, ndN, N, hpQRAG, diag_R, hpLe2, hpLAG, Ld, work3);
			}

		gettimeofday(&tv5, NULL); // start

		// factorize information filter
		for(rep=0; rep<nrep; rep++)
			{
			//d_ric_trs_mhe_if(nx, nw, ndN, N, hpALe, hpGLr, Ld, hqq, hrr, hff, hxp, hxe, hw, hlam, work3);
			d_ric_trs_mhe_if(nx, nw, ndN, N, hpLe2, hpLAG, Ld, hqq, hrr, hff, hxp, hxe, hw, hlam, work3);
			}

		gettimeofday(&tv6, NULL); // start

		// factorize information filter
		for(rep=0; rep<nrep; rep++)
			{
#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_BLIS) || defined(REF_BLAS_NETLIB)
			//d_ric_trf_mhe_if_blas( nx, nw, ndN, N, hA, hG, hQ, hR, hAGU, hUp, hUe, hUr);
			d_ric_trf_mhe_if_blas( nx, nw, ndN, N, hA, hG, hQ, hR, hAGU, hUp, hUe, hUr, Ud);
#endif
			}

		gettimeofday(&tv7, NULL); // start

		// solution information filter
		for(rep=0; rep<nrep; rep++)
			{
#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_BLIS) || defined(REF_BLAS_NETLIB)
			d_ric_trs_mhe_if_blas( nx, nw, ndN, N, hAGU, hUp, hUe, hUr, Ud, hqq, hrr, hff, hxp, hxe, hw, hlam, work_ref);
#endif
			}

		gettimeofday(&tv8, NULL); // start

		float Gflops_max = flops_max * GHz_max;

		float time_trf = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		float time_trs = (float) (tv2.tv_sec-tv1.tv_sec)/(nrep+0.0)+(tv2.tv_usec-tv1.tv_usec)/(nrep*1e6);
		float time_trf_end = (float) (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);
		float time_trs_end = (float) (tv4.tv_sec-tv3.tv_sec)/(nrep+0.0)+(tv4.tv_usec-tv3.tv_usec)/(nrep*1e6);
		float time_trf_if = (float) (tv5.tv_sec-tv4.tv_sec)/(nrep+0.0)+(tv5.tv_usec-tv4.tv_usec)/(nrep*1e6);
		float time_trs_if = (float) (tv6.tv_sec-tv5.tv_sec)/(nrep+0.0)+(tv6.tv_usec-tv5.tv_usec)/(nrep*1e6);
		float time_trf_if_blas = (float) (tv7.tv_sec-tv6.tv_sec)/(nrep+0.0)+(tv7.tv_usec-tv6.tv_usec)/(nrep*1e6);
		float time_trs_if_blas = (float) (tv8.tv_sec-tv7.tv_sec)/(nrep+0.0)+(tv8.tv_usec-tv7.tv_usec)/(nrep*1e6);

		float flop_trf_if = N*(10.0/3.0*nx*nx*nx+nx*nx*nw)+2.0/3.0*nx*nx*nx+ndN*nx*nx+ndN*ndN*nx+1.0/3.0*ndN*ndN*ndN;
		if(diag_R==0)
			flop_trf_if += N*(nx*nw*nw+1.0/3.0*nw*nw*nw);
		else
			flop_trf_if += N*(nx*nw+1.0/2.0*nw*nw);

		float Gflops_trf_if = flop_trf_if*1e-9/time_trf_if;
		float Gflops_trf_if_blas = flop_trf_if*1e-9/time_trf_if_blas;

		if(ll==0)
			{
			printf("\nnx\tnw\tny\tN\ttrf time\ttrs time\ttrf_e time\ttrs_e time\ttrf_if time\ttrf_if Gflops\ttrf_if percent\ttrs_if time\ttrf_if BLAS\tGflops\t\tpercent\t\ttrs_if BLAS\n\n");
//			fprintf(f, "\nnx\tnu\tN\tsv time\t\tsv Gflops\tsv %%\t\ttrs time\ttrs Gflops\ttrs %%\n\n");
			}
		printf("%d\t%d\t%d\t%d\t%e\t%e\t%e\t%e\t%e\t%f\t%f\t%e\t%e\t%f\t%f\t%e\n", nx, nw, ny, N, time_trf, time_trs, time_trf_end, time_trs_end, time_trf_if, Gflops_trf_if, 100*Gflops_trf_if/Gflops_max, time_trs_if, time_trf_if_blas, Gflops_trf_if_blas, 100*Gflops_trf_if_blas/Gflops_max, time_trs_if_blas);


#if 0
		return 0;


		// moving horizon test

		// window size
		N = 20;

		double *hhxe[N+1];
		double *hhxp[N+1];
		double *hhw[N];
		double *hhy[N+1];
		double *hhlam[N];

		double *p_hhxe; d_zeros_align(&p_hhxe, anx, N+1);
		double *p_hhxp; d_zeros_align(&p_hhxp, anx, N+1);
		double *p_hhw; d_zeros_align(&p_hhw, anw, N);
		double *p_hhlam; d_zeros_align(&p_hhlam, anx, N);

		// shift measurements and initial prediction
		for(ii=0; ii<N; ii++)
			{
			hhxe[ii] = p_hhxe+ii*anx; //d_zeros_align(&hxe[jj], anx, 1);
			hhxp[ii] = p_hhxp+ii*anx; //d_zeros_align(&hxp[jj], anx, 1);
			hhw[ii] = p_hhw+ii*anw; //d_zeros_align(&hw[jj], anw, 1);
			hhy[ii] = hy[ii]; //d_zeros_align(&hy[jj], any, 1);
			hhlam[ii] = p_hhlam+ii*anx; //d_zeros_align(&hlam[jj], anx, 1);
			}
		hhxe[N] = p_hhxe+N*anx; //d_zeros_align(&hxe[jj], anx, 1);
		hhxp[N] = p_hhxp+N*anx; //d_zeros_align(&hxp[jj], anx, 1);
		hhy[N] = hy[N]; //d_zeros_align(&hy[jj], any, 1);

		// shift initial prediction covariance
		//for(ii=0; ii<pnx*cnl; ii++)
		//	hpLp[0][ii] = hpLp[1][ii];

		d_ric_trf_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpQ, hpR, hpLe, work);
		d_ric_trs_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpQ, hpR, hpLe, hq, hr, hf, hhxp, hhxe, hhw, hhy, 1, hhlam, work);

		// zero data
		for(ii=0; ii<Ns*anx; ii++)
			hxe[0][ii] = 0.0;

		for(ii=anx; ii<Ns*anx; ii++)
			hxp[0][ii] = 0.0;

		for(ii=0; ii<(Ns-1)*anw; ii++)
			hw[0][ii] = 0.0;

		for(ii=0; ii<(Ns-1)*anx; ii++)
			hlam[0][ii] = 0.0;

		// save data
		for(ii=0; ii<(N+1); ii++)
			for(jj=0; jj<nx; jj++)
				hxe[ii][jj] = hhxe[ii][jj];

		for(ii=0; ii<(N+1); ii++)
			for(jj=0; jj<nx; jj++)
				hxp[ii][jj] = hhxp[ii][jj];

		for(ii=0; ii<N; ii++)
			for(jj=0; jj<nw; jj++)
				hw[ii][jj] = hhw[ii][jj];
		//d_print_mat(nw, N, hw[0], anw);

		for(ii=0; ii<N; ii++)
			for(jj=0; jj<nx; jj++)
				hlam[ii][jj] = hhlam[ii][jj];



		for(jj=1; jj<Ns-N; jj++)
			{

			//break;
			
			// shift measurements and initial prediction
			for(ii=0; ii<=N; ii++)
				{
				hhy[ii] = hy[ii+jj];
				}

			// shift initial prediction and relative covariance
			for(ii=0; ii<nx; ii++)
				hhxp[0][ii] = hhxp[1][ii];
			for(ii=0; ii<pnx*cnl; ii++)
				hpLp[0][ii] = hpLp[1][ii];

			//d_print_mat(nx, N+1, hhxp[0], anx);

			//d_print_pmat(nx, nx, bs, hpLp[1]+(nx+nw+pad)*bs, cnl);
			//d_print_pmat(nz, nz, bs, hpLe[1], cnf);
			//d_print_pmat(nx, nx, bs, hpLp[2]+(nx+nw+pad)*bs, cnl);
			//d_print_pmat(nz, nz, bs, hpLe[2], cnf);

			d_ric_trf_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpQ, hpR, hpLe, work);
			d_ric_trs_mhe(nx, nw, ny, N, hpA, hpG, hpC, hpLp, hdLp, hpQ, hpR, hpLe, hq, hr, hf, hhxp, hhxe, hhw, hhy, 1, hhlam, work);

			//d_print_mat(nx, N+1, hhxp[0], anx);

			//d_print_pmat(nx, nx, bs, hpLp[0]+(nx+nw+pad)*bs, cnl);
			//d_print_pmat(nz, nz, bs, hpLe[0], cnf);
			//d_print_pmat(nx, nx, bs, hpLp[1]+(nx+nw+pad)*bs, cnl);
			//d_print_pmat(nz, nz, bs, hpLe[1], cnf);

			// save data
			for(ii=0; ii<nx; ii++)
				hxe[N+jj][ii] = hhxe[N][ii];

			for(ii=0; ii<nx; ii++)
				hxp[N+jj][ii] = hhxp[N][ii];

			if(jj<Ns-N-1)
				for(ii=0; ii<nw; ii++)
					hw[N+jj][ii] = hhw[N-1][ii];

			if(jj<Ns-N-1)
				for(ii=0; ii<nx; ii++)
					hlam[N+jj][ii] = hhlam[N-1][ii];

			//break;

			}

		// print solution
		if(PRINTRES)
			{
			printf("\nx_p\n");
			d_print_mat(nx, Ns, hxp[0], anx);
			printf("\nx_e\n");
			d_print_mat(nx, Ns, hxe[0], anx);
			//printf("\nL_e\n");
			//d_print_pmat(nx, nx, bs, hpLp[Ns-1]+(nx+nw+pad)*bs, cnl);
			}

#endif

/************************************************
* return
************************************************/

		free(A);
		free(B);
		free(C);
		free(b);
		free(D);
		free(d);
		free(x0);
		free(Q);
		free(Qx);
		free(R);
		free(q);
		free(r);
		free(f);
		free(L0);
		free(pA);
		free(pG);
		free(pC);
		free(pQ);
		free(pR);
		free(pQA);
		free(pRG);
		free(work);
		free(work2);
		free(work3);
		free(work4);
		free(p_hxe);
		free(p_hxp);
		free(p_hy);
		free(p_hw);
		free(p_hlam);
		//free(p_hhxe);
		//free(p_hhxp);
		//free(p_hhw);
		//free(p_hhlam);
		free(x_temp);
		free(y_temp);
		free(p0);
		free(p_hr_res);
		free(p_hq_res);
		free(p_hf_res);
		free(pL0_inv);
		free(hpLp[0]);
		free(hdLp[0]);
		free(hpLe[0]);
		for(jj=0; jj<N; jj++)
			{
			free(hpLp[jj+1]);
			free(hdLp[jj+1]);
			free(hpLe[jj+1]);
			free(hpGLr[jj]);
			free(hpALe[jj]);
			free(hpLp2[jj]);
			}
		free(hpALe[N]);


		free(pQRAG);
		free(pQD);
		for(ii=0; ii<N; ii++)
			{
			free(hpLAG[ii]);
			free(hpLe2[ii]);
			}
		free(hpLAG[N]);
		free(hpLe2[N]);

		for(ii=0; ii<N; ii++)
			{
			free(hAGU[ii]);
			free(hUp[ii]);
			free(hUe[ii]);
			free(hUr[ii]);
			}
		free(hUp[N]);
		free(hUe[N]);
		free(Ud);
		free(work_ref);


		} // increase size

	fprintf(f, "];\n");
	fclose(f);


	return 0;

	}



