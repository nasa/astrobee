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

//#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//#include <xmmintrin.h> // needed to flush to zero sub-normals with _MM_SET_FLUSH_ZERO_MODE (_MM_FLUSH_ZERO_ON); in the main()
//#endif



#include <blasfeo.h>
#include "benchmark_x_common.h"



#ifndef D_PS
#define D_PS 1
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
//#include "../include/d_blas_64.h"
#endif

#if defined(EXTERNAL_BLAS_MKL)
#include "mkl.h"
#endif




//#define PRINT_TO_FILE



#if 0//defined(LA_HIGH_PERFORMANCE) & (defined(TARGET_X64_INTEL_HASWELL) | defined(TARGET_X64_INTEL_SANDY_BRIDGE))
void dgemm_nn_1_1_1(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_4x2_vs_lib4(1, &alpha, A, 0, B, sdb, &beta, C, D, 1, 1);
	return;
	}

void dgemm_nn_2_2_2(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_2x2_lib4(2, &alpha, A, 0, B, sdb, &beta, C, D);
	return;
	}

void dgemm_nn_3_3_3(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_4x4_vs_lib4(3, &alpha, A, 0, B, sdb, &beta, C, D, 3, 3);
	return;
	}

void dgemm_nn_4_4_4(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_4x4_lib4(4, &alpha, A, 0, B, sdb, &beta, C, D);
	return;
	}

void dgemm_nn_5_5_5(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_6x6_vs_lib4(5, &alpha, A, sda, 0, B, sdb, &beta, C, sdc, D, sdd, 5, 5);
	return;
	}

void dgemm_nn_6_6_6(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_6x6_lib4(6, &alpha, A, sda, 0, B, sdb, &beta, C, sdc, D, sdd);
	return;
	}

void dgemm_nn_7_7_7(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_8x4_vs_lib4(7, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(7, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd, 7, 3);
	return;
	}

void dgemm_nn_8_8_8(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_8x4_lib4(8, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(8, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	return;
	}

void dgemm_nn_9_9_9(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_10x4_vs_lib4(9, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(9, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd, 9, 4);
	kernel_dgemm_nn_10x2_vs_lib4(9, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd, 9, 1);
	return;
	}

void dgemm_nn_10_10_10(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_10x4_lib4(10, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_10x4_lib4(10, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_10x2_lib4(10, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	return;
	}

void dgemm_nn_11_11_11(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_12x4_vs_lib4(11, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(11, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(11, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd, 11, 3);
	return;
	}

void dgemm_nn_12_12_12(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_12x4_lib4(12, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_12x4_lib4(12, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(12, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	return;
	}

void dgemm_nn_13_13_13(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_8x4_lib4(13, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(13, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x6_vs_lib4(13, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd, 8, 5);

	kernel_dgemm_nn_6x8_vs_lib4(13, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdc, sdd, D+8*sdd, sdd, 5, 8);
	kernel_dgemm_nn_6x6_vs_lib4(13, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdc+8*4, sdd, D+8*sdd+8*4, sdd, 5, 5);
	return;
	}

void dgemm_nn_14_14_14(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_8x4_lib4(14, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(14, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x6_lib4(14, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);

	kernel_dgemm_nn_6x8_lib4(14, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdc, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_6x6_lib4(14, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdc+8*4, sdd, D+8*sdd+8*4, sdd);
	return;
	}

void dgemm_nn_15_15_15(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_8x6_lib4(15, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x6_lib4(15, &alpha, A, sda, 0, B+6*4, sdb, &beta, C+6*4, sdd, D+6*4, sdd);
	kernel_dgemm_nn_8x4_vs_lib4(15, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd, 8, 3);

	kernel_dgemm_nn_8x6_vs_lib4(15, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd, 7, 6);
	kernel_dgemm_nn_8x6_vs_lib4(15, &alpha, A+8*sda, sda, 0, B+6*4, sdb, &beta, C+8*sdd+6*4, sdd, D+8*sdd+6*4, sdd, 7, 6);
	kernel_dgemm_nn_8x4_vs_lib4(15, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd, 7, 3);
#else
	kernel_dgemm_nn_8x4_lib4(15, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(15, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(15, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_vs_lib4(15, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd, 8, 3);

	kernel_dgemm_nn_8x4_vs_lib4(15, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(15, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(15, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(15, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd, 7, 3);
#endif
	return;
	}

void dgemm_nn_16_16_16(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_8x6_lib4(16, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x6_lib4(16, &alpha, A, sda, 0, B+6*4, sdb, &beta, C+6*4, sdd, D+6*4, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);

	kernel_dgemm_nn_8x6_lib4(16, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_8x6_lib4(16, &alpha, A+8*sda, sda, 0, B+6*4, sdb, &beta, C+8*sdd+6*4, sdd, D+8*sdd+6*4, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
#else
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);

	kernel_dgemm_nn_8x4_lib4(16, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(16, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
#endif
	return;
	}

void dgemm_nn_17_17_17(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_12x4_lib4(17, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_12x4_lib4(17, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(17, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(17, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_12x4_vs_lib4(17, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd, 12, 1);

	kernel_dgemm_nn_6x8_vs_lib4(17, &alpha, A+12*sda, sda, 0, B, sdb, &beta, C+12*sdd, sdd, D+12*sdd, sdd, 5, 8);
	kernel_dgemm_nn_6x8_vs_lib4(17, &alpha, A+12*sda, sda, 0, B+8*4, sdb, &beta, C+12*sdd+8*4, sdd, D+12*sdd+8*4, sdd, 5, 8);
	kernel_dgemm_nn_6x2_vs_lib4(17, &alpha, A+12*sda, sda, 0, B+16*4, sdb, &beta, C+12*sdd+16*4, sdd, D+12*sdd+16*4, sdd, 5, 1);
#else
	kernel_dgemm_nn_8x4_lib4(17, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(17, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(17, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x6_vs_lib4(17, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd, 8, 5);

	kernel_dgemm_nn_10x4_vs_lib4(17, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(17, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(17, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(17, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd, 9, 4);
	kernel_dgemm_nn_10x2_vs_lib4(17, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd, 9, 1);
#endif
	return;
	}

void dgemm_nn_18_18_18(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_12x4_lib4(18, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_12x4_lib4(18, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(18, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(18, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_12x4_vs_lib4(18, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd, 12, 2);

	kernel_dgemm_nn_6x8_lib4(18, &alpha, A+12*sda, sda, 0, B, sdb, &beta, C+12*sdd, sdd, D+12*sdd, sdd);
	kernel_dgemm_nn_6x8_lib4(18, &alpha, A+12*sda, sda, 0, B+8*4, sdb, &beta, C+12*sdd+8*4, sdd, D+12*sdd+8*4, sdd);
	kernel_dgemm_nn_6x2_lib4(18, &alpha, A+12*sda, sda, 0, B+16*4, sdb, &beta, C+12*sdd+16*4, sdd, D+12*sdd+16*4, sdd);
#else
	kernel_dgemm_nn_8x4_lib4(18, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(18, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(18, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x6_lib4(18, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);

	kernel_dgemm_nn_10x4_lib4(18, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_10x4_lib4(18, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd);
	kernel_dgemm_nn_10x4_lib4(18, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd);
	kernel_dgemm_nn_10x4_lib4(18, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
	kernel_dgemm_nn_10x2_lib4(18, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd);
#endif
	return;
	}

void dgemm_nn_19_19_19(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_8x4_lib4(19, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(19, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(19, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(19, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_8x4_vs_lib4(19, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd, 8, 3);

	kernel_dgemm_nn_12x4_vs_lib4(19, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(19, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(19, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(19, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(19, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd, 11, 3);
	return;
	}

void dgemm_nn_20_20_20(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
	kernel_dgemm_nn_8x4_lib4(20, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(20, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(20, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(20, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_8x4_lib4(20, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);

	kernel_dgemm_nn_12x4_lib4(20, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_12x4_lib4(20, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(20, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(20, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
	kernel_dgemm_nn_12x4_lib4(20, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd);
	return;
	}

void dgemm_nn_21_21_21(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_12x4_lib4(21, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_12x4_lib4(21, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(21, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(21, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_12x4_lib4(21, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);
	kernel_dgemm_nn_12x4_vs_lib4(21, &alpha, A, sda, 0, B+20*4, sdb, &beta, C+20*4, sdd, D+20*4, sdd, 12, 1);

	kernel_dgemm_nn_10x4_vs_lib4(21, &alpha, A+12*sda, sda, 0, B, sdb, &beta, C+12*sdd, sdd, D+12*sdd, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(21, &alpha, A+12*sda, sda, 0, B+4*4, sdb, &beta, C+12*sdd+4*4, sdd, D+12*sdd+4*4, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(21, &alpha, A+12*sda, sda, 0, B+8*4, sdb, &beta, C+12*sdd+8*4, sdd, D+12*sdd+8*4, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(21, &alpha, A+12*sda, sda, 0, B+12*4, sdb, &beta, C+12*sdd+12*4, sdd, D+12*sdd+12*4, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(21, &alpha, A+12*sda, sda, 0, B+16*4, sdb, &beta, C+12*sdd+16*4, sdd, D+12*sdd+16*4, sdd, 9, 4);
	kernel_dgemm_nn_10x4_vs_lib4(21, &alpha, A+12*sda, sda, 0, B+20*4, sdb, &beta, C+12*sdd+20*4, sdd, D+12*sdd+20*4, sdd, 9, 1);
#else
	kernel_dgemm_nn_8x4_lib4(21, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(21, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(21, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(21, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_8x6_vs_lib4(21, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd, 8, 5);

	kernel_dgemm_nn_8x4_lib4(21, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_8x4_lib4(21, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(21, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(21, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
	kernel_dgemm_nn_8x6_vs_lib4(21, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd, 8, 5);

	kernel_dgemm_nn_6x8_vs_lib4(21, &alpha, A+16*sda, sda, 0, B, sdb, &beta, C+16*sdd, sdd, D+16*sdd, sdd, 5, 8);
	kernel_dgemm_nn_6x8_vs_lib4(21, &alpha, A+16*sda, sda, 0, B+8*4, sdb, &beta, C+16*sdd+8*4, sdd, D+16*sdd+8*4, sdd, 5, 8);
	kernel_dgemm_nn_6x6_vs_lib4(21, &alpha, A+16*sda, sda, 0, B+16*4, sdb, &beta, C+16*sdd+16*4, sdd, D+16*sdd+16*4, sdd, 5, 5);
#endif
	return;
	}

void dgemm_nn_22_22_22(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_12x4_lib4(22, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_12x4_lib4(22, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(22, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(22, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_12x4_lib4(22, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);
	kernel_dgemm_nn_12x4_vs_lib4(22, &alpha, A, sda, 0, B+20*4, sdb, &beta, C+20*4, sdd, D+20*4, sdd, 12, 2);

	kernel_dgemm_nn_10x4_lib4(22, &alpha, A+12*sda, sda, 0, B, sdb, &beta, C+12*sdd, sdd, D+12*sdd, sdd);
	kernel_dgemm_nn_10x4_lib4(22, &alpha, A+12*sda, sda, 0, B+4*4, sdb, &beta, C+12*sdd+4*4, sdd, D+12*sdd+4*4, sdd);
	kernel_dgemm_nn_10x4_lib4(22, &alpha, A+12*sda, sda, 0, B+8*4, sdb, &beta, C+12*sdd+8*4, sdd, D+12*sdd+8*4, sdd);
	kernel_dgemm_nn_10x4_lib4(22, &alpha, A+12*sda, sda, 0, B+12*4, sdb, &beta, C+12*sdd+12*4, sdd, D+12*sdd+12*4, sdd);
	kernel_dgemm_nn_10x4_lib4(22, &alpha, A+12*sda, sda, 0, B+16*4, sdb, &beta, C+12*sdd+16*4, sdd, D+12*sdd+16*4, sdd);
	kernel_dgemm_nn_10x4_vs_lib4(22, &alpha, A+12*sda, sda, 0, B+20*4, sdb, &beta, C+12*sdd+20*4, sdd, D+12*sdd+20*4, sdd, 10, 2);
#else
	kernel_dgemm_nn_8x4_lib4(22, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(22, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(22, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(22, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_8x6_lib4(22, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);

	kernel_dgemm_nn_8x4_lib4(22, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_8x4_lib4(22, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(22, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(22, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
	kernel_dgemm_nn_8x6_lib4(22, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd);

	kernel_dgemm_nn_6x8_lib4(22, &alpha, A+16*sda, sda, 0, B, sdb, &beta, C+16*sdd, sdd, D+16*sdd, sdd);
	kernel_dgemm_nn_6x8_lib4(22, &alpha, A+16*sda, sda, 0, B+8*4, sdb, &beta, C+16*sdd+8*4, sdd, D+16*sdd+8*4, sdd);
	kernel_dgemm_nn_6x6_lib4(22, &alpha, A+16*sda, sda, 0, B+16*4, sdb, &beta, C+16*sdd+16*4, sdd, D+16*sdd+16*4, sdd);
#endif
	return;
	}

void dgemm_nn_23_23_23(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_12x4_lib4(23, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_12x4_lib4(23, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(23, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(23, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_12x4_lib4(23, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);
	kernel_dgemm_nn_12x4_vs_lib4(23, &alpha, A, sda, 0, B+20*4, sdb, &beta, C+20*4, sdd, D+20*4, sdd, 12, 3);

	kernel_dgemm_nn_12x4_vs_lib4(23, &alpha, A+12*sda, sda, 0, B, sdb, &beta, C+12*sdd, sdd, D+12*sdd, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(23, &alpha, A+12*sda, sda, 0, B+4*4, sdb, &beta, C+12*sdd+4*4, sdd, D+12*sdd+4*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(23, &alpha, A+12*sda, sda, 0, B+8*4, sdb, &beta, C+12*sdd+8*4, sdd, D+12*sdd+8*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(23, &alpha, A+12*sda, sda, 0, B+12*4, sdb, &beta, C+12*sdd+12*4, sdd, D+12*sdd+12*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(23, &alpha, A+12*sda, sda, 0, B+16*4, sdb, &beta, C+12*sdd+16*4, sdd, D+12*sdd+16*4, sdd, 11, 4);
	kernel_dgemm_nn_12x4_vs_lib4(23, &alpha, A+12*sda, sda, 0, B+20*4, sdb, &beta, C+12*sdd+20*4, sdd, D+12*sdd+20*4, sdd, 11, 3);
#else
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);
	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A, sda, 0, B+20*4, sdb, &beta, C+20*4, sdd, D+20*4, sdd, 8, 3);

	kernel_dgemm_nn_8x4_lib4(23, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
	kernel_dgemm_nn_8x4_lib4(23, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd);
	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A+8*sda, sda, 0, B+20*4, sdb, &beta, C+8*sdd+20*4, sdd, D+8*sdd+20*4, sdd, 8, 3);

	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A+16*sda, sda, 0, B, sdb, &beta, C+16*sdd, sdd, D+16*sdd, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A+16*sda, sda, 0, B+4*4, sdb, &beta, C+16*sdd+4*4, sdd, D+16*sdd+4*4, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A+16*sda, sda, 0, B+8*4, sdb, &beta, C+16*sdd+8*4, sdd, D+16*sdd+8*4, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A+16*sda, sda, 0, B+12*4, sdb, &beta, C+16*sdd+12*4, sdd, D+16*sdd+12*4, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A+16*sda, sda, 0, B+16*4, sdb, &beta, C+16*sdd+16*4, sdd, D+16*sdd+16*4, sdd, 7, 4);
	kernel_dgemm_nn_8x4_vs_lib4(23, &alpha, A+16*sda, sda, 0, B+20*4, sdb, &beta, C+16*sdd+20*4, sdd, D+16*sdd+20*4, sdd, 7, 3);
#endif
	return;
	}

void dgemm_nn_24_24_24(double alpha, double *A, int sda, double *B, int sdb, double beta, double *C, int sdc, double *D, int sdd)
	{
#if defined(TARGET_X64_INTEL_HASWELL)
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A, sda, 0, B+20*4, sdb, &beta, C+20*4, sdd, D+20*4, sdd);

	kernel_dgemm_nn_12x4_lib4(24, &alpha, A+12*sda, sda, 0, B, sdb, &beta, C+12*sdd, sdd, D+12*sdd, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A+12*sda, sda, 0, B+4*4, sdb, &beta, C+12*sdd+4*4, sdd, D+12*sdd+4*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A+12*sda, sda, 0, B+8*4, sdb, &beta, C+12*sdd+8*4, sdd, D+12*sdd+8*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A+12*sda, sda, 0, B+12*4, sdb, &beta, C+12*sdd+12*4, sdd, D+12*sdd+12*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A+12*sda, sda, 0, B+16*4, sdb, &beta, C+12*sdd+16*4, sdd, D+12*sdd+16*4, sdd);
	kernel_dgemm_nn_12x4_lib4(24, &alpha, A+12*sda, sda, 0, B+20*4, sdb, &beta, C+12*sdd+20*4, sdd, D+12*sdd+20*4, sdd);
#else
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A, sda, 0, B, sdb, &beta, C, sdd, D, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A, sda, 0, B+4*4, sdb, &beta, C+4*4, sdd, D+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A, sda, 0, B+8*4, sdb, &beta, C+8*4, sdd, D+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A, sda, 0, B+12*4, sdb, &beta, C+12*4, sdd, D+12*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A, sda, 0, B+16*4, sdb, &beta, C+16*4, sdd, D+16*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A, sda, 0, B+20*4, sdb, &beta, C+20*4, sdd, D+20*4, sdd);

	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+8*sda, sda, 0, B, sdb, &beta, C+8*sdd, sdd, D+8*sdd, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+8*sda, sda, 0, B+4*4, sdb, &beta, C+8*sdd+4*4, sdd, D+8*sdd+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+8*sda, sda, 0, B+8*4, sdb, &beta, C+8*sdd+8*4, sdd, D+8*sdd+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+8*sda, sda, 0, B+12*4, sdb, &beta, C+8*sdd+12*4, sdd, D+8*sdd+12*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+8*sda, sda, 0, B+16*4, sdb, &beta, C+8*sdd+16*4, sdd, D+8*sdd+16*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+8*sda, sda, 0, B+20*4, sdb, &beta, C+8*sdd+20*4, sdd, D+8*sdd+20*4, sdd);

	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+16*sda, sda, 0, B, sdb, &beta, C+16*sdd, sdd, D+16*sdd, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+16*sda, sda, 0, B+4*4, sdb, &beta, C+16*sdd+4*4, sdd, D+16*sdd+4*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+16*sda, sda, 0, B+8*4, sdb, &beta, C+16*sdd+8*4, sdd, D+16*sdd+8*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+16*sda, sda, 0, B+12*4, sdb, &beta, C+16*sdd+12*4, sdd, D+16*sdd+12*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+16*sda, sda, 0, B+16*4, sdb, &beta, C+16*sdd+16*4, sdd, D+16*sdd+16*4, sdd);
	kernel_dgemm_nn_8x4_lib4(24, &alpha, A+16*sda, sda, 0, B+20*4, sdb, &beta, C+16*sdd+20*4, sdd, D+16*sdd+20*4, sdd);
#endif
	return;
	}
#endif


int main()
	{


// initialize blasfeo (e.g. pre-allocate memory buffers) (optional, not thread safe)
//blasfeo_init();


#if defined(EXTERNAL_BLAS_OPENBLAS)
	openblas_set_num_threads(1);
#endif
#if defined(EXTERNAL_BLAS_BLIS)
//	omp_set_num_threads(1);
#endif
#if defined(EXTERNAL_BLAS_MKL)
	mkl_set_num_threads(1);
#endif

//#if defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); // flush to zero subnormals !!! works only with one thread !!!
//#endif

	printf("\n");
	printf("\n");
	printf("\n");

	printf("BLASFEO performance test - BLASFEO API - double precision\n");
	printf("\n");

	// maximum frequency of the processor
	const float GHz_max = GHZ_MAX;
	printf("Frequency used to compute theoretical peak: %5.1f GHz (edit benchmarks/cpu_freq.h to modify this value).\n", GHz_max);
	printf("\n");

	// maximum flops per cycle, double precision
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const float flops_max = 32;
	printf("Testing BLASFEO version for AVX512F instruction set, 64 bit (optimized for Intel Skylake-X): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_HASWELL)
	const float flops_max = 16;
	printf("Testing BLASFEO version for AVX2 and FMA instruction sets, 64 bit (optimized for Intel Haswell): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	const float flops_max = 8;
	printf("Testing BLASFEO version for AVX instruction set, 64 bit (optimized for Intel Sandy Bridge): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_CORE)
	const float flops_max = 4;
	printf("Testing BLASFEO version for SSE3 instruction set, 64 bit (optimized for Intel Core): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_AMD_BULLDOZER)
	const float flops_max = 8;
	printf("Testing BLASFEO version for SSE3 and FMA instruction set, 64 bit (optimized for AMD Bulldozer): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_AMD_JAGUAR)
	const float flops_max = 2;
	printf("Testing BLASFEO version for AVX instruction set, 32 bit (optimized for AMD Jaguar): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_AMD_BARCELONA)
	const float flops_max = 4; // 2 on jaguar
	printf("Testing BLASFEO version for SSE3 instruction set, 32 bit (optimized for AMD Barcelona): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_APPLE_M1)
	const float flops_max = 16;
	printf("Testing BLASFEO version for NEONv2 instruction set, 64 bit (optimized for Apple M1): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A76)
	const float flops_max = 8;
	printf("Testing BLASFEO version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A76): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const float flops_max = 4;
	printf("Testing BLASFEO version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A57): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	const float flops_max = 4;
	printf("Testing BLASFEO version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A53): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	const float flops_max = 0.5;
	printf("Testing BLASFEO version for VFPv4 instruction set, 32 bit (optimized for ARM Cortex A7): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15)
	const float flops_max = 2;
	printf("Testing BLASFEO version for VFPv4 instruction set, 32 bit (optimized for ARM Cortex A15): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	const float flops_max = 1;
	printf("Testing BLAS version for VFPv3 instruction set, 32 bit (optimized for ARM Cortex A9): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_GENERIC)
	const float flops_max = 2;
	printf("Testing BLASFEO version for generic scalar instruction set: theoretical peak %5.1f Gflops ???\n", flops_max*GHz_max);
#endif



#ifdef PRINT_TO_FILE
	FILE *f;
	f = fopen("./build/benchmark_one.m", "w"); // a

	fprintf(f, "A = [%f %f];\n", GHz_max, flops_max);
	fprintf(f, "\n");
	fprintf(f, "B = [\n");
#endif

	printf("\nn\t Gflops\t    %%\t  time\t\t Gflops\t    %%\t  time\n\n");



	int i, j, rep, ll;

	const int bsd = D_PS;



#if 1
	int nn[] = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396, 400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444, 448, 452, 456, 460, 464, 468, 472, 476, 480, 484, 488, 492, 496, 500, 520, 540, 560, 580, 600, 620, 640, 680, 700, 720, 740, 760, 780, 800, 820, 840, 860, 880, 900, 920, 940, 960, 980, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 400, 400, 400, 400, 400, 200, 200, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

//	for(ll=0; ll<24; ll++)
//	for(ll=0; ll<75; ll++) // up to 300
//	for(ll=0; ll<115; ll++) // up to 460
//	for(ll=0; ll<120; ll++) // up to 700
//	for(ll=0; ll<149; ll++) // up to 1000
//	for(ll=0; ll<159; ll++) // up to 2000
//	for(ll=0; ll<169; ll++) // up to 3000
	for(ll=0; ll<169; ll++) // up to 3000

		{

		int n = nn[ll];
		int nrep = nnrep[ll]/10;
		nrep = nrep>1 ? nrep : 1;
//		int n = ll+1;
//		int nrep = nnrep[0];
//		n = n<12 ? 12 : n;
//		n = n<8 ? 8 : n;

#elif 1
	int nn[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};

	for(ll=0; ll<24; ll++)

		{

		int n = nn[ll];
		int nrep = 40000; //nnrep[ll];
#else
// TODO  ll<1 !!!!!

	for(ll=0; ll<1; ll++)

		{

		int n = 24;
		int nrep = 40000; //nnrep[ll];
#endif

		int rep_in;
		int nrep_in = 10;

		double *A; d_zeros_align(&A, n, n);
		double *B; d_zeros_align(&B, n, n);
		double *C; d_zeros_align(&C, n, n);
		double *M; d_zeros_align(&M, n, n);

		char c_n = 'n';
		char c_l = 'l';
		char c_r = 'r';
		char c_t = 't';
		char c_u = 'u';
		int i_1 = 1;
		int i_t;
		double d_1 = 1;
		double d_0 = 0;

		for(i=0; i<n*n; i++)
			A[i] = i;

		for(i=0; i<n; i++)
			B[i*(n+1)] = 1;

		for(i=0; i<n*n; i++)
			M[i] = 1;

		int n2 = n*n;
		double *B2; d_zeros(&B2, n, n);
		for(i=0; i<n*n; i++)
			B2[i] = 1e-15;
		for(i=0; i<n; i++)
			B2[i*(n+1)] = 1;

		int pnd = ((n+bsd-1)/bsd)*bsd;

		double *x; d_zeros_align(&x, pnd, 1);
		double *y; d_zeros_align(&y, pnd, 1);
		double *x2; d_zeros_align(&x2, pnd, 1);
		double *y2; d_zeros_align(&y2, pnd, 1);
		double *diag; d_zeros_align(&diag, pnd, 1);
		int *ipiv; int_zeros(&ipiv, n, 1);

		for(i=0; i<pnd; i++) x[i] = 1;
		for(i=0; i<pnd; i++) x2[i] = 1;

		// matrix struct
#if 0
		struct blasfeo_dmat sA; blasfeo_allocate_dmat(n+4, n+4, &sA);
		struct blasfeo_dmat sB; blasfeo_allocate_dmat(n+4, n+4, &sB);
		struct blasfeo_dmat sB2; blasfeo_allocate_dmat(n+4, n+4, &sB2);
		struct blasfeo_dmat sB3; blasfeo_allocate_dmat(n+4, n+4, &sB3);
		struct blasfeo_dmat sC; blasfeo_allocate_dmat(n+4, n+4, &sC);
		struct blasfeo_dmat sD; blasfeo_allocate_dmat(n+4, n+4, &sD);
		struct blasfeo_dmat sE; blasfeo_allocate_dmat(n+4, n+4, &sE);
#elif 1
		struct blasfeo_dmat sA; blasfeo_allocate_dmat(n, n, &sA);
		struct blasfeo_dmat sB; blasfeo_allocate_dmat(n, n, &sB);
		struct blasfeo_dmat sB2; blasfeo_allocate_dmat(n, n, &sB2);
		struct blasfeo_dmat sB3; blasfeo_allocate_dmat(n, n, &sB3);
		struct blasfeo_dmat sB4; blasfeo_allocate_dmat(n, 2*n, &sB4);
		struct blasfeo_dmat sB5; blasfeo_allocate_dmat(n, 3*n, &sB5);
		struct blasfeo_dmat sC; blasfeo_allocate_dmat(n, n, &sC);
		struct blasfeo_dmat sD; blasfeo_allocate_dmat(n, n, &sD);
		struct blasfeo_dmat sE; blasfeo_allocate_dmat(n, n, &sE);
#else
		int memsize = blasfeo_memsize_dmat(n, n);
//		void *mem = malloc(7*memsize);
		void *mem;
		blasfeo_malloc_align(&mem, 7*memsize);
		struct blasfeo_dmat sA; blasfeo_create_dmat(n, n, &sA, mem);
		struct blasfeo_dmat sB; blasfeo_create_dmat(n, n, &sB, mem+1*memsize);
		struct blasfeo_dmat sB2; blasfeo_create_dmat(n, n, &sB2, mem+2*memsize);
		struct blasfeo_dmat sB3; blasfeo_create_dmat(n, n, &sB3, mem+3*memsize);
		struct blasfeo_dmat sC; blasfeo_create_dmat(n, n, &sC, mem+4*memsize);
		struct blasfeo_dmat sD; blasfeo_create_dmat(n, n, &sD, mem+5*memsize);
		struct blasfeo_dmat sE; blasfeo_create_dmat(n, n, &sE, mem+6*memsize);
#endif
		struct blasfeo_dvec sx; blasfeo_allocate_dvec(n, &sx);
		struct blasfeo_dvec sy; blasfeo_allocate_dvec(n, &sy);
		struct blasfeo_dvec sz; blasfeo_allocate_dvec(n, &sz);

		blasfeo_pack_dmat(n, n, A, n, &sA, 0, 0);
		blasfeo_pack_dmat(n, n, B, n, &sB, 0, 0);
		blasfeo_pack_dmat(n, n, B2, n, &sB2, 0, 0);
		blasfeo_pack_dvec(n, x, 1, &sx, 0);
		blasfeo_pack_dvec(n, y, 1, &sy, 0);
		int ii;

		blasfeo_dgese(n, n, 1e-3, &sB3, 0, 0);
		for(ii=0; ii<n; ii++)
			{
			BLASFEO_DMATEL(&sB3, ii, ii) = 1.0;
			BLASFEO_DMATEL(&sB3, n-1, ii) = 1.0;
			BLASFEO_DMATEL(&sB3, ii, n-1) = 1.0;
			BLASFEO_DVECEL(&sx, ii) = 1.0;
			}
		// B4
		blasfeo_dgecp(n, n, &sB3, 0, 0, &sB4, 0, 0);
		blasfeo_dgecp(n, n, &sA, 0, 0, &sB4, 0, n);
		// B5
		blasfeo_dgecp(n, n, &sB3, 0, 0, &sB5, 0, 0);
		blasfeo_dgecp(n, n, &sB, 0, 0, &sB5, 0, n);
		blasfeo_dgecp(n, n, &sA, 0, 0, &sB5, 0, 2*n);

		// D
		blasfeo_dgese(n, n, 0.0, &sD, 0, 0);

		int qr_work_size = blasfeo_dgeqrf_worksize(n, n);
		void *qr_work;
		v_zeros_align(&qr_work, qr_work_size);

		int lq_work_size = blasfeo_dgelqf_worksize(n, n);
		void *lq_work;
		v_zeros_align(&lq_work, lq_work_size);

		// create matrix to pivot all the time
		// blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);

		double *dummy;

		int info;

		double alpha = 1.0;
		double beta = 0.0;

		/* timing */
		blasfeo_timer timer;

		double time_blasfeo  = 1e15;
		double time_blas     = 1e15;
		double tmp_time_blasfeo;
		double tmp_time_blas;

		/* warm up */
		for(rep=0; rep<nrep; rep++)
			{
			blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sC, 0, 0);
			}

		/* benchmarks */

		int m0 = n;
		int n0 = n;
		int k0 = n;

		// batches repetion, find minimum averaged time
		// discard batch interrupted by the scheduler
		for(rep_in=0; rep_in<nrep_in; rep_in++)
			{

			// BENCHMARK_BLASFEO
			blasfeo_tic(&timer);

			// averaged repetions
			for(rep=0; rep<nrep; rep++)
				{

//				kernel_dgemm_nt_12x4_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_dgemm_nt_8x8_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_dsyrk_nt_l_8x8_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_dgemm_nt_8x4_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_dgemm_nt_4x8_lib4(n, &alpha, sA.pA, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_dgemm_nt_4x4_lib4(n, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//				kernel_dgemm_nn_4x4_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_dger4_12_sub_lib4(n, sA.pA, sA.cn, sB.pA, sD.pA, sD.cn);
//				kernel_dger4_sub_12r_lib4(n, sA.pA, sA.cn, sB.pA, sD.pA, sD.cn);
//				kernel_dger4_sub_8r_lib4(n, sA.pA, sA.cn, sB.pA, sD.pA, sD.cn);
//				kernel_dger12_add_4r_lib4(n, sA.pA, sB.pA, sB.cn, sD.pA);
//				kernel_dger8_add_4r_lib4(n, sA.pA, sB.pA, sB.cn, sD.pA);
//				kernel_dger4_sub_4r_lib4(n, sA.pA, sB.pA, sD.pA);
//				kernel_dger2_sub_4r_lib4(n, sA.pA, sB.pA, sD.pA);
//				kernel_dger4_sub_8c_lib4(n, sA.pA, sA.cn, sB.pA, sD.pA, sD.cn);
//				kernel_dger4_sub_4c_lib4(n, sA.pA, sA.cn, sB.pA, sD.pA, sD.cn);
//				kernel_dgemm_nn_4x12_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_dgemm_nn_4x8_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_dgemm_nn_2x8_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_dgemm_nn_4x4_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_dgemm_nn_12x4_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_dgemm_nn_8x4_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_dgemm_nn_4x4_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//				kernel_dgemm_nn_8x6_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//				kernel_dgemm_nn_8x4_gen_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, 0, sD.pA, sD.cn, 0, sD.pA, sD.cn, 0, 8, 0, 4);
//				kernel_dgemm_nn_4x4_gen_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, 0, sD.pA, sD.cn, 0, sD.pA, sD.cn, 0, 8, 0, 4);



//				blasfeo_dgemm_nn(m0, n0, k0, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dgemm_nt(m0, n0, k0, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dgemm_tn(m0, n0, k0, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dgemm_tt(m0, n0, k0, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);

				blasfeo_dgemm_nn(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dgemm_tn(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dgemm_tt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk_ln(n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk3_ln(n, n, 1.0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk_ln_mn(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 0.0, &sC, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk_lt(n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk3_lt(n, n, 1.0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk_un(n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk3_un(n, n, 1.0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk_ut(n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyrk3_ut(n, n, 1.0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dsyr2k_ln(n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dpotrf_l(n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dpotrf_l_mn(n, n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dpotrf_u(n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dsyrk_dpotrf_ln(n, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//				blasfeo_dgetrf_np(n, n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dgetrf_np_test(n, n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dgetrf_rp(n, n, &sB, 0, 0, &sB, 0, 0, ipiv);
//				blasfeo_dgetrf_rp_test(n, n, &sB, 0, 0, &sB, 0, 0, ipiv);
//				blasfeo_dgeqrf(n, n, &sC, 0, 0, &sD, 0, 0, qr_work);
//				blasfeo_dcolin(n, &sx, 0, &sB3, 0, n-1);
//				blasfeo_dgelqf(n, n, &sB3, 0, 0, &sB3, 0, 0, lq_work);
//				blasfeo_dgelqf_pd(n, n, &sB3, 0, 0, &sB3, 0, 0, lq_work);
//				blasfeo_dcolin(n, &sx, 0, &sB4, 0, 2*n-1);
//				blasfeo_dgelqf_pd_la(n, n, &sB4, 0, 0, &sB4, 0, n, lq_work);
//				blasfeo_dgelqf_pd(n, 2*n, &sB4, 0, 0, &sB4, 0, 0, lq_work);
//				blasfeo_dcolin(n, &sx, 0, &sB5, 0, 3*n-1);
//				blasfeo_dgelqf_pd_lla(n, n, &sB5, 0, 0, &sB5, 0, n, &sB5, 0, 2*n, lq_work);
//				blasfeo_dgelqf_pd(n, 3*n, &sB5, 0, 0, &sB5, 0, 0, lq_work);
//				blasfeo_dtrmm_llnn(n, n, 1.0, &sB, 0, 0, &sD, 0, 0, &sD, 0, 0); //
//				blasfeo_dtrmm_lltn(n, n, 1.0, &sB, 0, 0, &sD, 0, 0, &sD, 0, 0); //
//				blasfeo_dtrmm_rlnn(n, n, 1.0, &sA, 0, 0, &sD, 0, 0, &sD, 0, 0); //
//				blasfeo_dtrmm_rutn(n, n, 1.0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//				blasfeo_dtrsm_llnn(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm2_llnn(n, n, 1.0, &sD, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm_llnu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm2_llnu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm2_rutu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm_lunn(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm_lutn(n, n, 1.0, &sB, 0, 0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dtrsm_rlnn(n, n, 1.0, &sB, 0, 0, &sD, 0, 0, &sD, 0, 0); // B2?
//				blasfeo_dtrsm_rltn(n, n, 1.0, &sB, 0, 0, &sD, 0, 0, &sD, 0, 0); // B2?
//				blasfeo_dtrsm_rltu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm_rutn(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dgemv_n(n, n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sy, 0, &sz, 0);
//				blasfeo_dgemv_t(n, n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sy, 0, &sz, 0);
//				blasfeo_dgemv_nt(n, n, 1.0, 1.0, &sA, 0, 0, &sx, 0, &sx, 0, 0.0, 0.0, &sy, 0, &sy, 0, &sz, 0, &sz, 0);
//				blasfeo_dsymv_l(n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sz, 0, &sz, 0);
//				blasfeo_dsymv_u(n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sz, 0, &sz, 0);
//				blasfeo_dtrmv_lnn(n, &sB, 0, 0, &sx, 0, &sz, 0);
//				blasfeo_dtrmv_ltn(n, &sB, 0, 0, &sx, 0, &sz, 0);
//				blasfeo_dtrsv_lnn(n, &sB, 0, 0, &sx, 0, &sz, 0);
//				blasfeo_dtrsv_ltn(n, &sB, 0, 0, &sx, 0, &sz, 0);
//				blasfeo_dger(n, n, 1.0, &sx, 0, &sy, 0, &sD, 0, 0, &sD, 0, 0);
//				blasfeo_dgetr(n, n, &sA, 0, 0, &sD, 0, 0);
				}

			tmp_time_blasfeo = blasfeo_toc(&timer) / nrep;
			time_blasfeo = tmp_time_blasfeo<time_blasfeo ? tmp_time_blasfeo : time_blasfeo;
			// BENCHMARK_BLASFEO_END

			// BENCHMARK_BLAS_REF
			blasfeo_tic(&timer);

			for(rep=0; rep<nrep; rep++)
				{
				#if defined(EXTERNAL_BLAS_OPENBLAS) || defined(EXTERNAL_BLAS_NETLIB) || defined(EXTERNAL_BLAS_MKL)
//				dpotrf_(&c_l, &n, B2, &n, &info);
				// dgemm_(&c_n, &c_n, &n, &n, &n, &d_1, A, &n, B, &n, &d_0, C, &n);
				// dgemm_(&c_n, &c_n, &n, &n, &n, &d_1, A, &n, M, &n, &d_0, C, &n);
				// dsyrk_(&c_l, &c_n, &n, &n, &d_1, A, &n, &d_0, C, &n);
				// dtrmm_(&c_r, &c_u, &c_t, &c_n, &n, &n, &d_1, A, &n, C, &n);
				// dgetrf_(&n, &n, B2, &n, ipiv, &info);
				// dtrsm_(&c_l, &c_l, &c_n, &c_u, &n, &n, &d_1, B2, &n, B, &n);
				// dtrsm_(&c_l, &c_u, &c_n, &c_n, &n, &n, &d_1, B2, &n, B, &n);
				// dtrtri_(&c_l, &c_n, &n, B2, &n, &info);
				// dlauum_(&c_l, &n, B, &n, &info);
				// dgemv_(&c_n, &n, &n, &d_1, A, &n, x, &i_1, &d_0, y, &i_1);
				// dgemv_(&c_t, &n, &n, &d_1, A, &n, x2, &i_1, &d_0, y2, &i_1);
				// dtrmv_(&c_l, &c_n, &c_n, &n, B, &n, x, &i_1);
				// dtrsv_(&c_l, &c_n, &c_n, &n, B, &n, x, &i_1);
				// dsymv_(&c_l, &n, &d_1, A, &n, x, &i_1, &d_0, y, &i_1);
				// for(i=0; i<n; i++)
				// 	{
				// 	i_t = n-i;
				// 	dcopy_(&i_t, &B[i*(n+1)], &i_1, &C[i*(n+1)], &i_1);
				// 	}
				// dsyrk_(&c_l, &c_n, &n, &n, &d_1, A, &n, &d_1, C, &n);
				// dpotrf_(&c_l, &n, C, &n, &info);
				#endif

				#if defined(EXTERNAL_BLAS_BLIS)
				// dgemm_(&c_n, &c_t, &n77, &n77, &n77, &d_1, A, &n77, B, &n77, &d_0, C, &n77);
				// dgemm_(&c_n, &c_n, &n77, &n77, &n77, &d_1, A, &n77, B, &n77, &d_0, C, &n77);
				// dsyrk_(&c_l, &c_n, &n77, &n77, &d_1, A, &n77, &d_0, C, &n77);
				// dtrmm_(&c_r, &c_u, &c_t, &c_n, &n77, &n77, &d_1, A, &n77, C, &n77);
				// dpotrf_(&c_l, &n77, B, &n77, &info);
				// dtrtri_(&c_l, &c_n, &n77, B, &n77, &info);
				// dlauum_(&c_l, &n77, B, &n77, &info);
				#endif
				}

			tmp_time_blas = blasfeo_toc(&timer) / nrep;
			time_blas = tmp_time_blas<time_blas ? tmp_time_blas : time_blas;

			// BENCHMARK_BLAS_REF_END

			}

		float Gflops_max = flops_max * GHz_max;

//		float flop_operation = 4*16.0*2*n; // kernel 16x4
//		float flop_operation = 3*16.0*2*n; // kernel 12x4
//		float flop_operation = 2*16.0*2*n; // kernel 8x4
//		float flop_operation = 1*16.0*2*n; // kernel 4x4
//		float flop_operation = 0.5*16.0*2*n; // kernel 2x4

//		float flop_operation = 2.0*m0*n0*k0; // gemm
//		float flop_operation = 1.0*m0*m0*k0; // syrk

		float flop_operation = 2.0*n*n*n; // gemm syr2k
//		float flop_operation = 1.0*n*n*n; // syrk trmm trsm
//		float flop_operation = 1.0/3.0*n*n*n; // potrf trtri
//		float flop_operation = 2.0/3.0*n*n*n; // getrf
//		float flop_operation = 4.0/3.0*n*n*n; // geqrf gelqf
//		float flop_operation = 2.0*n*n*n; // geqrf_la
//		float flop_operation = 8.0/3.0*n*n*n; // geqrf_lla
//		float flop_operation = 2.0*n*n; // gemv symv ger
//		float flop_operation = 1.0*n*n; // trmv trsv
//		float flop_operation = 4.0*n*n; // gemv_nt
//		float flop_operation = 4.0/3.0*n*n*n; // syrk+potrf

		float Gflops_blasfeo  = 1e-9*flop_operation/time_blasfeo;

		#ifndef EXTERNAL_BLAS_NONE
		float Gflops_blas     = 1e-9*flop_operation/time_blas;
		#else
		float Gflops_blas     = 0;
		#endif

		printf("%d\t%7.3f\t%7.3f\t%5.3e\t%7.3f\t%7.3f\t%5.3e\n",
			n,
			Gflops_blasfeo, 100.0*Gflops_blasfeo/Gflops_max, time_blasfeo,
			Gflops_blas, 100.0*Gflops_blas/Gflops_max, time_blas);
#ifdef PRINT_TO_FILE
		fprintf(f, "%d\t%7.3f\t%7.3f\t%5.3e\t%7.3f\t%7.3f\t%5.3e\n",
			n,
			Gflops_blasfeo, 100.0*Gflops_blasfeo/Gflops_max, time_blasfeo,
			Gflops_blas, 100.0*Gflops_blas/Gflops_max, time_blas);
#endif

		d_free(A);
		d_free(B);
		d_free(B2);
		d_free(M);
		d_free_align(x);
		d_free_align(y);
		d_free_align(x2);
		d_free_align(y2);
		int_free(ipiv);
		free(qr_work);
		free(lq_work);

#if 1
		blasfeo_free_dmat(&sA);
		blasfeo_free_dmat(&sB);
		blasfeo_free_dmat(&sB2);
		blasfeo_free_dmat(&sB3);
		blasfeo_free_dmat(&sB4);
		blasfeo_free_dmat(&sB5);
		blasfeo_free_dmat(&sC);
		blasfeo_free_dmat(&sD);
		blasfeo_free_dmat(&sE);
#else
		blasfeo_free_align(mem);
#endif
		blasfeo_free_dvec(&sx);
		blasfeo_free_dvec(&sy);
		blasfeo_free_dvec(&sz);

		}

	printf("\n");
#ifdef PRINT_TO_FILE
	fprintf(f, "];\n");

	fclose(f);
#endif


// quit blasfeo (e.g. free pre-allocated memory buffers)
//blasfeo_quit();


	return 0;

	}

