/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/


#include <stdlib.h>
#include <stdio.h>



#include "../include/blasfeo.h"
#include "benchmark_x_common.h"



#ifndef S_PS
#define S_PS 1
#endif



#if defined(EXTERNAL_BLAS_NETLIB)
//#include "cblas.h"
//#include "lapacke.h"
#include "../include/d_blas.h"
#endif

#if defined(EXTERNAL_BLAS_OPENBLAS)
void openblas_set_num_threads(int num_threads);
//#include "cblas.h"
//#include "lapacke.h"
#include "../include/d_blas.h"
#endif

#if defined(EXTERNAL_BLAS_BLIS)
void omp_set_num_threads(int num_threads);
#include "blis.h"
#endif

#if defined(EXTERNAL_BLAS_MKL)
#include "mkl.h"
#endif



//#define PRINT_TO_FILE



int main()
	{

#if defined(EXTERNAL_BLAS_OPENBLAS)
	openblas_set_num_threads(1);
#endif
#if defined(EXTERNAL_BLAS_BLIS)
//	omp_set_num_threads(1);
#endif
#if defined(EXTERNAL_BLAS_MKL)
	mkl_set_num_threads(1);
#endif

	printf("\n");
	printf("\n");
	printf("\n");

	printf("BLAS performance test - single precision\n");
	printf("\n");

	// maximum frequency of the processor
	const float GHz_max = GHZ_MAX;
	printf("Frequency used to compute theoretical peak: %5.1f GHz (edit test_param.h to modify this value).\n", GHz_max);
	printf("\n");

	// maximum flops per cycle, single precision
	// maxumum memops (sustained load->store of floats) per cycle, single precision
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const float flops_max = 64; // 4x256 bit fma
	const float memops_max = 16; // 2x256 bit load + 1x256 bit store
	printf("Testing BLASFEO version for AVX512F instruction set, 64 bit (optimized for Intel Skylake-X): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_HASWELL)
	const float flops_max = 32; // 2x256 bit fma
	const float memops_max = 8; // 2x256 bit load + 1x256 bit store
	printf("Testing BLAS version for AVX2 and FMA instruction sets, 64 bit (optimized for Intel Haswell): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	const float flops_max = 16; // 1x256 bit mul + 1x256 bit add
	const float memops_max = 4; // 1x256 bit load + 1x128 bit store
	printf("Testing BLAS version for AVX instruction set, 64 bit (optimized for Intel Sandy Bridge): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_CORE)
	const float flops_max = 8; // 1x128 bit mul + 1x128 bit add
	const float memops_max = 4; // 1x128 bit load + 1x128 bit store;
	printf("Testing BLAS version for SSE3 instruction set, 64 bit (optimized for Intel Core): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_AMD_BULLDOZER)
	const float flops_max = 16; // 2x128 bit fma
	const float memops_max = 4; // 1x256 bit load + 1x128 bit store
	printf("Testing BLAS version for AVX and FMA instruction set, 64 bit (optimized for AMD Bulldozer): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_AMD_JAGUAR)
	const float flops_max = 8; // 1x128 bit mul + 1x128 bit add
	const float memops_max = 4; // 1x128 bit load + 1x128 bit store
	printf("Testing BLAS version for AVX instruction set, 32 bit (optimized for AMD Jaguar): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_AMD_BARCELONA)
	const float flops_max = 8; // 1x128 bit mul + 1x128 bit add
	const float memops_max = 4; // 1x128 bit load + 1x128 bit store
	printf("Testing BLAS version for SSE3 instruction set, 32 bit (optimized for AMD Barcelona): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_APPLE_M1)
	const float flops_max = 32; // 4x128 bit fma
	const float memops_max = 8; // ???
	printf("Testing BLAS version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A76): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A76)
	const float flops_max = 16; // 2x128 bit fma
	const float memops_max = 8; // ???
	printf("Testing BLAS version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A76): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const float flops_max = 8; // 1x128 bit fma
	const float memops_max = 4; // ???
	printf("Testing BLAS version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A57): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	const float flops_max = 8; // 1x128 bit fma
	const float memops_max = 4; // ???
	printf("Testing BLAS version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A53): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15)
	const float flops_max = 8; // 1x128 bit fma
	const float memops_max = 4; // ???
	printf("Testing BLAS version for VFPv4 instruction set, 32 bit (optimized for ARM Cortex A15): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	const float flops_max = 4; // 1x64 bit fma
	const float memops_max = 2; // ???
	printf("Testing BLAS version for VFPv3 instruction set, 32 bit (optimized for ARM Cortex A9): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	const float flops_max = 2; // 1x32 bit fma
	const float memops_max = 2; // ???
	printf("Testing BLAS version for VFPv4 instruction set, 32 bit (optimized for ARM Cortex A7): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_GENERIC)
	const float flops_max = 2; // 1x32 bit mul + 1x32 bit add ???
	const float memops_max = 1; // ???
	printf("Testing BLAS version for generic scalar instruction set: theoretical peak %5.1f Gflops ???\n", flops_max*GHz_max);
#endif

#ifdef PRINT_TO_FILE
	FILE *f;
	f = fopen("./test_problems/results/test_blas.m", "w"); // a

#if defined(TARGET_X64_INTEL_HASWELL)
//	fprintf(f, "C = 's_x64_intel_haswell';\n");
//	fprintf(f, "\n");
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	fprintf(f, "C = 's_x64_intel_sandybridge';\n");
//	fprintf(f, "\n");
#elif defined(TARGET_X64_INTEL_CORE)
//	fprintf(f, "C = 's_x64_intel_core';\n");
//	fprintf(f, "\n");
#elif defined(TARGET_X64_AMD_BULLDOZER)
//	fprintf(f, "C = 's_x64_amd_bulldozer';\n");
//	fprintf(f, "\n");
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
//	fprintf(f, "C = 's_armv7a_arm_cortex_a15';\n");
//	fprintf(f, "\n");
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15)
//	fprintf(f, "C = 's_armv7a_arm_cortex_a15';\n");
//	fprintf(f, "\n");
#elif defined(TARGET_GENERIC)
//	fprintf(f, "C = 's_generic';\n");
//	fprintf(f, "\n");
#endif

	fprintf(f, "A = [%f %f];\n", GHz_max, flops_max);
	fprintf(f, "\n");
	fprintf(f, "B = [\n");
#endif



	int i, j, rep, ll;

	const int bss = S_PS;

/*	int info = 0;*/

	printf("\nn\t  sgemm_blasfeo\t  sgemm_blas\n");
	printf("\nn\t Gflops\t    %%\t Gflops\t    %%\n\n");

#if 1
	int nn[] = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396, 400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444, 448, 452, 456, 460, 500, 550, 600, 650, 700};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 400, 400, 400, 400, 400, 200, 200, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 4, 4, 4, 4, 4};

//	for(ll=0; ll<24; ll++)
	for(ll=0; ll<75; ll++)
//	for(ll=0; ll<115; ll++)
//	for(ll=0; ll<120; ll++)

		{

		int n = nn[ll];
		int nrep = nnrep[ll];
		nrep = nrep>1 ? nrep : 1;
//		int n = ll+1;
//		int nrep = nnrep[0];
//		n = n<16 ? 16 : n;

		int n2 = n*n;

#else
	int nn[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};

	for(ll=0; ll<24; ll++)

		{

		int n = nn[ll];
		int nrep = 40000; //nnrep[ll];
#endif

		int rep_in;
		int nrep_in = 10;

		float *A; s_zeros_align(&A, n, n);
		float *B; s_zeros_align(&B, n, n);
		float *C; s_zeros_align(&C, n, n);
		float *M; s_zeros_align(&M, n, n);

		char c_n = 'n';
		char c_l = 'l';
		char c_r = 'r';
		char c_t = 't';
		char c_u = 'u';
		int i_1 = 1;
		int i_t;
		float d_1 = 1;
		float d_0 = 0;

		for(i=0; i<n*n; i++)
			A[i] = i;

		for(i=0; i<n; i++)
			B[i*(n+1)] = 1;

		for(i=0; i<n*n; i++)
			M[i] = 1;

		float *B2; s_zeros(&B2, n, n);
		for(i=0; i<n*n; i++)
			B2[i] = 1e-15;
		for(i=0; i<n; i++)
			B2[i*(n+1)] = 1;

		float *x; s_zeros(&x, n, 1);
		float *y; s_zeros(&y, n, 1);
		float *x2; s_zeros(&x2, n, 1);
		float *y2; s_zeros(&y2, n, 1);
		float *diag; s_zeros(&diag, n, 1);
		int *ipiv; int_zeros(&ipiv, n, 1);

//		for(i=0; i<n; i++) x[i] = 1;
//		for(i=0; i<n; i++) x2[i] = 1;

		// matrix struct
#if 0
		struct blasfeo_smat sA; blasfeo_allocate_smat(n+4, n+4, &sA);
		struct blasfeo_smat sB; blasfeo_allocate_smat(n+4, n+4, &sB);
		struct blasfeo_smat sC; blasfeo_allocate_smat(n+4, n+4, &sC);
		struct blasfeo_smat sD; blasfeo_allocate_smat(n+4, n+4, &sD);
		struct blasfeo_smat sE; blasfeo_allocate_smat(n+4, n+4, &sE);
#else
		struct blasfeo_smat sA; blasfeo_allocate_smat(n, n, &sA);
		struct blasfeo_smat sB; blasfeo_allocate_smat(n, n, &sB);
		struct blasfeo_smat sC; blasfeo_allocate_smat(n, n, &sC);
		struct blasfeo_smat sD; blasfeo_allocate_smat(n, n, &sD);
		struct blasfeo_smat sE; blasfeo_allocate_smat(n, n, &sE);
#endif
		struct blasfeo_svec sx; blasfeo_allocate_svec(n, &sx);
		struct blasfeo_svec sy; blasfeo_allocate_svec(n, &sy);
		struct blasfeo_svec sz; blasfeo_allocate_svec(n, &sz);

		blasfeo_pack_smat(n, n, A, n, &sA, 0, 0);
		blasfeo_pack_smat(n, n, B, n, &sB, 0, 0);
		blasfeo_pack_svec(n, x, 1, &sx, 0);

		// create matrix to pivot all the time
		// blasfeo_sgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);

		float *dummy;

		int info;

		float alpha = 1.0;
		float beta = 0.0;

		/* timing */
		blasfeo_timer timer;

		double time_blasfeo  = 1e15;
		double time_blas     = 1e15;
		double tmp_time_blasfeo;
		double tmp_time_blas;

		/* warm up */
		for(rep=0; rep<nrep; rep++)
			{
			blasfeo_sgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sC, 0, 0, &sD, 0, 0);
			}

		/* benchmarks */

		// batches repetion, find minimum averaged time
		// discard batch interrupted by the scheduler
		for(rep_in=0; rep_in<nrep_in; rep_in++)
			{

			// BENCHMARK_BLASFEO
			blasfeo_tic(&timer);

			// averaged repetions
			for(rep=0; rep<nrep; rep++)
				{

				// blasfeo kernels
//				kernel_sgemm_nt_24x4_lib8(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_sgemm_nt_16x4_lib8(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_sgemm_nt_8x8_lib8(n, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//				kernel_sgemm_nt_8x4_lib8(n, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//				kernel_sgemm_nt_4x8_gen_lib8(n, &alpha, sA.pA, sB.pA, &beta, 0, sD.pA, sD.cn, 0, sD.pA, sD.cn, 0, 4, 0, 8);
//				kernel_sgemm_nt_4x8_vs_lib8(n, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA, 4, 8);
//				kernel_sgemm_nt_4x8_lib8(n, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//				kernel_sgemm_nt_12x4_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_sgemm_nt_8x4_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_sgemm_nt_4x4_lib4(n, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//				kernel_sgemm_nn_16x4_lib8(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_sgemm_nn_8x8_lib8(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_sgemm_nn_8x4_lib8(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);

				// blasfeo routines
				blasfeo_sgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_sgemm_nn(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_sgemm_tn(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_sgemm_tt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_ssyrk_ln(n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_ssyrk_ln_mn(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_spotrf_l(n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_spotrf_l_mn(n, n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_spotrf_u(n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_sgetr(n, n, &sA, 0, 0, &sB, 0, 0);
//				blasfeo_sgetrf_nopivot(n, n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_sgetrf_rowpivot(n, n, &sB, 0, 0, &sB, 0, 0, ipiv);
//				blasfeo_strmm_rlnn(n, n, 1.0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//				blasfeo_strmm_rutn(n, n, 1.0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//				blasfeo_strsm_llnu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_strsm_lunn(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_strsm_rltn(n, n, 1.0, &sB, 0, 0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_strsm_rltu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_strsm_rutn(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_sgemv_n(n, n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sy, 0, &sz, 0);
//				blasfeo_sgemv_t(n, n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sy, 0, &sz, 0);
//				blasfeo_ssymv_l(n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sy, 0, &sz, 0);
//				blasfeo_sgemv_nt(n, n, 1.0, 1.0, &sA, 0, 0, &sx, 0, &sx, 0, 0.0, 0.0, &sy, 0, &sy, 0, &sz, 0, &sz, 0);
				}

			tmp_time_blasfeo = blasfeo_toc(&timer) / nrep;
			time_blasfeo = tmp_time_blasfeo<time_blasfeo ? tmp_time_blasfeo : time_blasfeo;
			// BENCHMARK_BLASFEO_END

			// BENCHMARK_BLAS_REF
			blasfeo_tic(&timer);

			for(rep=0; rep<nrep; rep++)
				{
				#if defined(EXTERNAL_BLAS_OPENBLAS) || defined(EXTERNAL_BLAS_NETLIB) || defined(EXTERNAL_BLAS_MKL)

//				sgemm_(&c_n, &c_t, &n, &n, &n, &d_1, A, &n, M, &n, &d_0, C, &n);
//				sgemm_(&c_n, &c_n, &n, &n, &n, &d_1, A, &n, M, &n, &d_0, C, &n);
//				scopy_(&n2, A, &i_1, B, &i_1);
//				ssyrk_(&c_l, &c_n, &n, &n, &d_1, A, &n, &d_0, C, &n);
//				strmm_(&c_r, &c_u, &c_t, &c_n, &n, &n, &d_1, A, &n, C, &n);
//				spotrf_(&c_l, &n, B2, &n, &info);
//				sgetrf_(&n, &n, B2, &n, ipiv, &info);
//				strsm_(&c_l, &c_l, &c_n, &c_u, &n, &n, &d_1, B2, &n, B, &n);
//				strsm_(&c_l, &c_u, &c_n, &c_n, &n, &n, &d_1, B2, &n, B, &n);
//				strtri_(&c_l, &c_n, &n, B2, &n, &info);
//				slauum_(&c_l, &n, B, &n, &info);
//				sgemv_(&c_n, &n, &n, &d_1, A, &n, x, &i_1, &d_0, y, &i_1);
//				sgemv_(&c_t, &n, &n, &d_1, A, &n, x2, &i_1, &d_0, y2, &i_1);
//				strmv_(&c_l, &c_n, &c_n, &n, B, &n, x, &i_1);
//				strsv_(&c_l, &c_n, &c_n, &n, B, &n, x, &i_1);
//				ssymv_(&c_l, &n, &d_1, A, &n, x, &i_1, &d_0, y, &i_1);
//				for(i=0; i<n; i++)
//					{
//					i_t = n-i;
//					scopy_(&i_t, &B[i*(n+1)], &i_1, &C[i*(n+1)], &i_1);
//					}
//				ssyrk_(&c_l, &c_n, &n, &n, &d_1, A, &n, &d_1, C, &n);
//				spotrf_(&c_l, &n, C, &n, &info);
				#endif

				#if defined(EXTERNAL_BLAS_BLIS)
				// sgemm_(&c_n, &c_t, &n77, &n77, &n77, &d_1, A, &n77, B, &n77, &d_0, C, &n77);
				// sgemm_(&c_n, &c_n, &n77, &n77, &n77, &d_1, A, &n77, B, &n77, &d_0, C, &n77);
				// ssyrk_(&c_l, &c_n, &n77, &n77, &d_1, A, &n77, &d_0, C, &n77);
				// strmm_(&c_r, &c_u, &c_t, &c_n, &n77, &n77, &d_1, A, &n77, C, &n77);
				// spotrf_(&c_l, &n77, B, &n77, &info);
				// strtri_(&c_l, &c_n, &n77, B, &n77, &info);
				// slauum_(&c_l, &n77, B, &n77, &info);
				#endif
				}

			tmp_time_blas = blasfeo_toc(&timer) / nrep;
			time_blas = tmp_time_blas<time_blas ? tmp_time_blas : time_blas;
			// BENCHMARK_BLAS_REF_END

			}

		// flops

		// blasfeo kernels
//		float flop_operation = 4*16.0*2*n; // kernel 16x4
//		float flop_operation = 3*16.0*2*n; // kernel 12x4
//		float flop_operation = 2*16.0*2*n; // kernel 8x4
//		float flop_operation = 1*16.0*2*n; // kernel 4x4
//		float flop_operation = 0.5*16.0*2*n; // kernel 2x4

		// blasfeo routines
		float flop_operation = 2.0*n*n*n; // gemm
//		float flop_operation = 1.0*n*n*n; // syrk trmm trsm
//		float flop_operation = 1.0/3.0*n*n*n; // potrf trtri
//		float flop_operation = 2.0/3.0*n*n*n; // getrf
//		float flop_operation = 4.0/3.0*n*n*n; // geqrf
//		float flop_operation = 2.0*n*n; // gemv symv
//		float flop_operation = 1.0*n*n; // trmv trsv
//		float flop_operation = 4.0*n*n; // gemv_nt
//		float flop_operation = 4.0/3.0*n*n*n; // syrk+potrf
//		float flop_operation = 1.0/3.0*n*n*n; // potrf trtri

		float Gflops_max = flops_max * GHz_max;

		float Gflops_blasfeo  = 1e-9*flop_operation/time_blasfeo;

		#ifndef EXTERNAL_BLAS_NONE
		float Gflops_blas     = 1e-9*flop_operation/time_blas;
		#else
		float Gflops_blas     = 0;
		#endif


		printf("%d\t%7.3f\t%7.3f\t%7.3f\t%7.3f\n",
			n,
			Gflops_blasfeo, 100.0*Gflops_blasfeo/Gflops_max,
			Gflops_blas, 100.0*Gflops_blas/Gflops_max);
#ifdef PRINT_TO_FILE
		fprintf(f, "%d\t%7.3f\t%7.3f\t%7.3f\t%7.3f\n",
			n,
			Gflops_blasfeo, 100.0*Gflops_blasfeo/Gflops_max,
			Gflops_blas, 100.0*Gflops_blas/Gflops_max);
#endif

		#if 0
		// memops

		float Gmemops_max = memops_max * GHz_max;

		float memop_operation = 1.0*n*n; // dgecp

		float Gmemops_blasfeo  = 1e-9*memop_operation/time_blasfeo;
		float Gmemops_blas     = 1e-9*memop_operation/time_blas;

		printf("%d\t%7.2f\t%7.2f\t%7.2f\t%7.2f\n",
			n,
			Gmemops_blasfeo, 100.0*Gmemops_blasfeo/Gmemops_max,
			Gmemops_blas, 100.0*Gmemops_blas/Gmemops_max);
		#endif

		free(A);
		free(B);
		free(B2);
		free(M);
		free(x);
		free(y);
		free(x2);
		free(y2);
		free(ipiv);

		blasfeo_free_smat(&sA);
		blasfeo_free_smat(&sB);
		blasfeo_free_smat(&sC);
		blasfeo_free_smat(&sD);
		blasfeo_free_smat(&sE);
		blasfeo_free_svec(&sx);
		blasfeo_free_svec(&sy);
		blasfeo_free_svec(&sz);

		}

	printf("\n");

#ifdef PRINT_TO_FILE
	fprintf(f, "];\n");
	fclose(f);
#endif

	return 0;

	}

