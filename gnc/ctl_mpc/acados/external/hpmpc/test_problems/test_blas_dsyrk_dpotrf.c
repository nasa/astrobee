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

#include "../include/aux_d.h"
/*#include "../include/aux_s.h"*/
#include "../include/blas_d.h"
/*#include "../include/blas_lib_s.h"*/
#include "../include/block_size.h"
#include "test_param.h"



//void dgemm_(char *ta, char *tb, int *m, int *n, int *k, double *alpha, double *A, int *lda, double *B, int *ldb, double *beta, double *C, int *ldc);
//void dgemv_(char *ta, int *m, int *n, double *alpha, double *A, int *lda, double *x, int *incx, double *beta, double *y, int *incy);

#if defined(REF_BLAS_OPENBLAS)
#include "../reference_code/blas.h"
void openblas_set_num_threads(int n_thread);
#endif
#if defined(REF_BLAS_BLIS)
#include <blis/blis.h>
#endif
#if defined(REF_BLAS_NETLIB)
#include "../reference_code/blas.h"
#endif



// matrix size (1 - 24)
//#define N 8

// codegen routines
#ifdef N
void dsyrk_codegen(double * restrict A, double * restrict B, double * restrict C)
	{

	const int n = N;

	int ii, jj, kk;

	for(jj=0; jj<n; jj++)
		{
		for(ii=jj; ii<n; ii++)
			{
			C[ii+n*jj] = B[ii+n*jj];
			}
		for(kk=0; kk<n; kk++)
			{
			for(ii=jj; ii<n; ii++)
				{
				C[ii+n*jj] = C[ii+n*jj] + A[ii+n*kk] * A[jj+n*kk];
				}
			}
		}
	
	}

void dpotrf_codegen(double * restrict A)
	{

	const int n = N;

	int ii, jj, kk;

	double ajj;

	for(jj=0; jj<n; jj++)
		{
		ajj = A[jj+n*jj];
		for(kk=0; kk<jj; kk++)
			{
			ajj = ajj - A[jj+n*kk] * A[jj+n*kk];
			}
		ajj = sqrt(ajj);
		A[jj+n*jj] = ajj;
		if(jj+1<n)
			{
			for(kk=0; kk<jj; kk++)
				{
				for(ii=jj+1; ii<n; ii++)
					{
					A[ii+n*jj] -= A[ii+n*kk] * A[jj+n*kk];
					}
				}
			ajj = 1/ajj;
			for(ii=jj+1; ii<n; ii++)
				{
				A[ii+n*jj] *= ajj;
				}
			}
		}
	
	}
#endif



// vectorization of column-major matrices
void dsyrk_vec(int m, int n, int k, double *A, int lda, double *B, int ldb, double *C, int ldc, double *D, int ldd, int alg)
	{

	int ii, jj;

#if 1
	
	for(ii=0; ii<m-4; ii+=8)
		{
		for(jj=0; jj<ii+4; jj+=4)
			{
			kernel_dgemm_nt_8x4_unpack_lib(k, &A[ii+lda*0], lda, &B[jj+ldb*0], ldb, &C[ii+ldc*jj], ldc, &D[ii+ldd*jj], ldd, alg, 0, 0);
			}
		kernel_dgemm_nt_4x4_unpack_lib(k, &A[ii+4+lda*0], lda, &B[jj+ldb*0], ldb, &C[ii+4+ldc*jj], ldc, &D[ii+4+ldd*jj], ldd, alg, 0, 0);
		}
	if(ii<m)
		{
		for(jj=0; jj<ii+4; jj+=4)
			{
			kernel_dgemm_nt_4x4_unpack_lib(k, &A[ii+lda*0], lda, &B[jj+ldb*0], ldb, &C[ii+ldc*jj], ldc, &D[ii+ldd*jj], ldd, alg, 0, 0);
			}
		}

#else

	for(ii=0; ii<m; ii+=4)
		{
		for(jj=0; jj<ii+4; jj+=4)
			{
			kernel_dgemm_nt_4x2_unpack_lib(k, &A[ii+lda*0], lda, &B[jj+ldb*0], ldb, &C[ii+ldc*jj], ldc, &D[ii+ldd*jj], ldd, alg, 0, 0);
			kernel_dgemm_nt_4x2_unpack_lib(k, &A[ii+lda*0], lda, &B[jj+2+ldb*0], ldb, &C[ii+ldc*(jj+2)], ldc, &D[ii+ldd*(jj+2)], ldd, alg, 0, 0);
			}
		if(jj<ii+2)
			{
			kernel_dgemm_nt_4x2_unpack_lib(k, &A[ii+lda*0], lda, &B[jj+ldb*0], ldb, &C[ii+ldc*jj], ldc, &D[ii+ldd*jj], ldd, alg, 0, 0);
			}
		}

#endif
	
	}

void dpotrf_vec(int m, int n, double *A, int lda)
	{

	int ii, jj;

#if 1
	
	for(jj=0; jj<n; jj+=4)
		{
		for(ii=jj; ii<m-4; ii+=8)
			{
			kernel_dgemm_nt_8x4_unpack_lib(jj, &A[ii+lda*0], lda, &A[jj+lda*0], lda, &A[ii+lda*jj], lda, &A[ii+lda*jj], lda, -1, 0, 0);
			}
		if(ii<m)
			{
			kernel_dgemm_nt_4x4_unpack_lib(jj, &A[ii+lda*0], lda, &A[jj+lda*0], lda, &A[ii+lda*jj], lda, &A[ii+lda*jj], lda, -1, 0, 0);
			}
		kernel_dpotrf_dtrsm_4_unpack_lib(m-jj, &A[jj+lda*jj], lda);
		}
	}

#else

	for(jj=0; jj<n; jj+=4)
		{
		for(ii=jj; ii<m; ii+=4)
			{
			kernel_dgemm_nt_4x2_unpack_lib(jj, &A[ii+lda*0], lda, &A[jj+lda*0], lda, &A[ii+lda*jj], lda, &A[ii+lda*jj], lda, -1, 0, 0);
			kernel_dgemm_nt_4x2_unpack_lib(jj, &A[ii+lda*0], lda, &A[jj+2+lda*0], lda, &A[ii+lda*(jj+2)], lda, &A[ii+lda*(jj+2)], lda, -1, 0, 0);
			}
		kernel_dpotrf_dtrsm_4_unpack_lib(m-jj, &A[jj+lda*jj], lda);
		}
	}

#endif

	



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
	printf(" Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.\n");
	printf("\n");
	printf(" HPMPC is distributed in the hope that it will be useful,\n");
	printf(" but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
	printf(" MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n");
	printf(" See the GNU Lesser General Public License for more details.\n");
	printf("\n");
	printf("\n");
	printf("\n");

	printf("BLAS performance test - double precision\n");
	printf("\n");

	// maximum frequency of the processor
	const float GHz_max = GHZ_MAX;
	printf("Frequency used to compute theoretical peak: %5.1f GHz (edit test_param.h to modify this value).\n", GHz_max);
	printf("\n");

	// maximum flops per cycle, double precision
#if defined(TARGET_X64_AVX2)
	const float flops_max = 16;
	printf("Testing BLAS version for AVX2 & FMA3 instruction sets, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_AVX)
	const float flops_max = 8;
	printf("Testing BLAS version for AVX instruction set, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_SSE3) || defined(TARGET_AMD_SSE3)
	const float flops_max = 4;
	printf("Testing BLAS version for SSE3 instruction set, 64 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
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
	printf("Testing BLAS version for SSE3 instruction set, 32 bit, optimized for Intel Atom: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_POWERPC_G2)
	const float flops_max = 1;
	printf("Testing BLAS version for POWERPC instruction set, 32 bit: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_4X4)
	const float flops_max = 2;
	printf("Testing reference BLAS version, 4x4 kernel: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_4X4_PREFETCH)
	const float flops_max = 2;
	printf("Testing reference BLAS version, 4x4 kernel with register prefetch: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_C99_2X2)
	const float flops_max = 2;
	printf("Testing reference BLAS version, 2x2 kernel: theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
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
	


	int i, j, rep, ll;
	
	const int bsd = D_MR; //d_get_mr();

	int info = 0;
	
	printf("\nn\t  BLAS\t  HPMPC\n");
	printf("\nn\t Gflops\t    %%\t Gflops\t    %%\n\n");
	
#ifdef N

	for(ll=0; ll<5; ll++)
		
		{

		const int n = N;
		int nrep = 40000; //nnrep[ll];

#else
#if 1
	int nn[] = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396, 400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444, 448, 452, 456, 460, 500, 550, 600, 650, 700};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 400, 400, 400, 400, 400, 200, 200, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 4, 4, 4, 4, 4};
	
	for(ll=0; ll<75; ll++)
//	for(ll=0; ll<115; ll++)
//	for(ll=0; ll<120; ll++)

		{

		int n = nn[ll];
		int nrep = nnrep[ll];

#else
	int nn[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
	
	for(ll=0; ll<24; ll++)

		{

		int n = nn[ll];
		int nrep = 40000; //nnrep[ll];
#endif

#endif

		int n2 = n*n;

#if defined(REF_BLAS_BLIS)
		f77_int n77 = n;
#endif
	
		double *A; d_zeros(&A, n, n);
		double *B; d_zeros(&B, n, n);
		double *C; d_zeros(&C, n, n);
		double *M; d_zeros(&M, n, n);

		char c_l = 'l';
		char c_n = 'n';
		char c_t = 't';
		int i_1 = 1;
#if defined(REF_BLAS_BLIS)
		f77_int i77_1 = i_1;
#endif
		double d_1 = 1;
		double d_0 = 0;
	
		for(i=0; i<n*n; i++)
			A[i] = i;
	
		for(i=0; i<n; i++)
			B[i*(n+1)] = 1;
	
		for(i=0; i<n*n; i++)
			M[i] = 1;
	
		int pnd = ((n+bsd-1)/bsd)*bsd;	
		int cnd = ((n+D_NCL-1)/D_NCL)*D_NCL;	
		int cnd2 = 2*((n+D_NCL-1)/D_NCL)*D_NCL;	
		int pad = (D_NCL-n%D_NCL)%D_NCL;

		double *aA; d_zeros_align(&aA, pnd, pnd);
		double *aB; d_zeros_align(&aB, pnd, pnd);
		double *aC; d_zeros_align(&aC, pnd, pnd);

		d_copy_mat(n, n, A, n, aA, pnd);
		d_copy_mat(n, n, B, n, aB, pnd);
//		d_print_mat(n, n, aA, pnd);
//		d_print_mat(n, n, aB, pnd);
//		exit(1);

		double *pA; d_zeros_align(&pA, pnd, cnd);
		double *pB; d_zeros_align(&pB, pnd, cnd);
		double *pC; d_zeros_align(&pC, pnd, cnd);
		double *pD; d_zeros_align(&pD, pnd, cnd);
		double *pE; d_zeros_align(&pE, pnd, cnd2);
		double *pF; d_zeros_align(&pF, 2*pnd, cnd);
		double *pL; d_zeros_align(&pL, pnd, cnd);
		double *pM; d_zeros_align(&pM, pnd, cnd);
		double *x; d_zeros_align(&x, pnd, 1);
		double *y; d_zeros_align(&y, pnd, 1);
		double *x2; d_zeros_align(&x2, pnd, 1);
		double *y2; d_zeros_align(&y2, pnd, 1);
		double *diag; d_zeros_align(&diag, pnd, 1);
	
		d_cvt_mat2pmat(n, n, A, n, 0, pA, cnd);
		d_cvt_mat2pmat(n, n, B, n, 0, pB, cnd);
		d_cvt_mat2pmat(n, n, B, n, 0, pD, cnd);
		d_cvt_mat2pmat(n, n, A, n, 0, pE, cnd2);
		d_cvt_mat2pmat(n, n, M, n, 0, pM, cnd);
/*		d_cvt_mat2pmat(n, n, B, n, 0, pE+n*bsd, pnd);*/
		
/*		d_print_pmat(n, 2*n, bsd, pE, 2*pnd);*/
/*		exit(2);*/
	
		for(i=0; i<pnd*cnd; i++) pC[i] = -1;
		
		for(i=0; i<pnd; i++) x[i] = 1;
		for(i=0; i<pnd; i++) x2[i] = 1;

		/* timing */
		struct timeval tvm1, tv0, tv1, tv2, tv3, tv4, tv5, tv6, tv7, tv8, tv9, tv10, tv11, tv12, tv13, tv14, tv15, tv16;

		gettimeofday(&tv0, NULL); // stop

		for(rep=0; rep<nrep; rep++)
			{

			dsyrk_dpotrf_lib(n, n, n, pA, cnd, pB, cnd, pC, cnd, diag, 1, 0);

			}
	
		gettimeofday(&tv1, NULL); // stop

		for(rep=0; rep<nrep; rep++)
			{

			dsyrk_nt_lib(n, n, n, pA, cnd, pA, cnd, pB, cnd, pC, cnd, 1);
			dpotrf_lib(n, n, pC, cnd, pC, cnd, diag);

			}
	
		gettimeofday(&tv2, NULL); // stop
	
		for(rep=0; rep<nrep; rep++)
			{
#if defined(REF_BLAS_OPENBLAS) || defined(REF_BLAS_NETLIB)
#ifdef N
			dsyrk_codegen(A, B, C);
			d_print_mat(n, n, C, n);
			//dsyrk_vec(n, n, n, aA, pnd, aA, pnd, aB, pnd, aC, pnd, 1);
			//dpotrf_vec(n, n, aA, pnd);
			//d_print_mat(n, n, aC, pnd);
			//exit(1);
			//dpotrf_codegen(aC);
#else
#if 1
			dsyrk_vec(n, n, n, aA, pnd, aA, pnd, aB, pnd, aC, pnd, 1);
			dpotrf_vec(n, n, aC, pnd);
			//dpotrf_(&c_l, &n, aC, &pnd, &info);
#else
			//dgemm_(&c_n, &c_n, &n, &n, &n, &d_1, A, &n, M, &n, &d_0, C, &n);
			dcopy_(&n2, B, &i_1, C, &i_1);
			dsyrk_(&c_l, &c_n, &n, &n, &d_1, A, &n, &d_1, C, &n);
			dpotrf_(&c_l, &n, C, &n, &info);
#endif
#endif
#endif
#if defined(REF_BLAS_BLIS)
			dgemm_(&c_n, &c_n, &n77, &n77, &n77, &d_1, A, &n77, B, &n77, &d_0, C, &n77);
#endif
			}

		gettimeofday(&tv3, NULL); // stop


		// print matricees
#if 0
		d_print_pmat(n, n, bsd, pC, cnd);

		d_print_mat(n, n, aC, pnd);

		d_print_mat(n, n, C, n);
#endif



		float Gflops_max = flops_max * GHz_max;
		float flop_dsyrk_dpotrf = 1.0*n*n*n + 1.0/3.0*n*n*n;

		float time_HPMPC_merged = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
		float Gflops_HPMPC_merged = 1e-9*flop_dsyrk_dpotrf/time_HPMPC_merged;

		float time_HPMPC_unmerged = (float) (tv2.tv_sec-tv1.tv_sec)/(nrep+0.0)+(tv2.tv_usec-tv1.tv_usec)/(nrep*1e6);
		float Gflops_HPMPC_unmerged = 1e-9*flop_dsyrk_dpotrf/time_HPMPC_unmerged;

		float time_BLAS = (float) (tv3.tv_sec-tv2.tv_sec)/(nrep+0.0)+(tv3.tv_usec-tv2.tv_usec)/(nrep*1e6);
		float Gflops_BLAS = 1e-9*flop_dsyrk_dpotrf/time_BLAS;

		printf("%d\t%7.2f\t%7.2f\t%7.2f\t%7.2f\t%7.2f\t%7.2f\n", n, Gflops_HPMPC_merged, 100.0*Gflops_HPMPC_merged/Gflops_max, Gflops_HPMPC_unmerged, 100.0*Gflops_HPMPC_unmerged/Gflops_max, Gflops_BLAS, 100.0*Gflops_BLAS/Gflops_max);

		fprintf(f, "%d\t%7.2f\t%7.2f\t%7.2f\t%7.2f\t%7.2f\t%7.2f\n", n, Gflops_HPMPC_merged, 100.0*Gflops_HPMPC_merged/Gflops_max, Gflops_HPMPC_unmerged, 100.0*Gflops_HPMPC_unmerged/Gflops_max, Gflops_BLAS, 100.0*Gflops_BLAS/Gflops_max);

		free(A);
		free(B);
		free(M);
		free(aA);
		free(aB);
		free(aC);
		free(pA);
		free(pB);
		free(pC);
		free(pD);
		free(pE);
		free(pF);
		free(pL);
		free(pM);
		free(x);
		free(y);
		free(x2);
		free(y2);
		
		}

	printf("\n");

	fprintf(f, "];\n");
	fclose(f);

	return 0;
	
	}
