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
#include <math.h>

#include "../include/blasfeo_common.h"
#include "../include/blasfeo_i_aux_ext_dep.h"
#include "../include/blasfeo_d_aux_ext_dep.h"
#include "../include/blasfeo_v_aux_ext_dep.h"
#include "../include/blasfeo_d_aux.h"
#include "../include/blasfeo_d_kernel.h"
#include "../include/blasfeo_d_blas.h"

//#include "test_d_common.h"
//#include "test_x_common.c"

int main()
	{
//	print_compilation_flags();

	int ii;

	int n = 24;

	//
	// matrices in column-major format
	//
	double *A; d_zeros(&A, n, n);
	for(ii=0; ii<n*n; ii++) A[ii] = ii;
//	d_print_mat(n, n, A, n);

	double *B; d_zeros(&B, n, n);
	for(ii=0; ii<n; ii++) B[ii*(n+1)] = 1.0;
//	d_print_mat(n, n, B, n);

	double *C; d_zeros(&C, n, n);

	double *D; d_zeros(&D, n, n);
	for(ii=0; ii<n*n; ii++) D[ii] = -1;

	double *x_n; d_zeros(&x_n, n, 1);
//	for(ii=0; ii<n; ii++) x_n[ii] = 1.0;
	x_n[6] = 1000.0;
//	x_n[1] = 1.0;
//	x_n[2] = 2.0;
//	x_n[3] = 3.0;
	double *x_t; d_zeros(&x_t, n, 1);
//	for(ii=0; ii<n; ii++) x_n[ii] = 1.0;
	x_t[0] = 1.0;
	double *y_n; d_zeros(&y_n, n, 1);
	double *y_t; d_zeros(&y_t, n, 1);
	double *z_n; d_zeros(&z_n, n, 1);
	double *z_t; d_zeros(&z_t, n, 1);

	double *x0; d_zeros(&x0, n, 1); x0[0] = 1.0;
	double *x1; d_zeros(&x1, n, 1); x1[1] = 1.0;
	double *x2; d_zeros(&x2, n, 1); x2[2] = 1.0;
	double *x3; d_zeros(&x3, n, 1); x3[3] = 1.0;
	double *x4; d_zeros(&x4, n, 1); x4[4] = 1.0;
	double *x5; d_zeros(&x5, n, 1); x5[5] = 1.0;
	double *x6; d_zeros(&x6, n, 1); x6[6] = 1.0;
	double *x7; d_zeros(&x7, n, 1); x7[7] = 1.0;
//	double *x8; d_zeros(&x8, n, 1); x8[8] = 1.0;
//	double *x9; d_zeros(&x9, n, 1); x9[9] = 1.0;

	int *ipiv; int_zeros(&ipiv, n, 1);
	int *ipiv_tmp; int_zeros(&ipiv_tmp, n, 1);
	int *ipiv_inv; int_zeros(&ipiv_inv, n, 1);

	//
	// matrices in matrix struct format
	//
	int size_strmat = 5*blasfeo_memsize_dmat(n, n);
	void *memory_strmat; v_zeros_align(&memory_strmat, size_strmat);
	char *ptr_memory_strmat = (char *) memory_strmat;

	struct blasfeo_dmat sA;
//	blasfeo_allocate_dmat(n, n, &sA);
	blasfeo_create_dmat(n, n, &sA, ptr_memory_strmat);
	ptr_memory_strmat += sA.memsize;
	blasfeo_pack_dmat(n, n, A, n, &sA, 0, 0);
//	d_cast_mat2strmat(A, &sA);
	blasfeo_print_dmat(n, n, &sA, 0, 0);

	struct blasfeo_dmat sB;
//	blasfeo_allocate_dmat(n, n, &sB);
	blasfeo_create_dmat(n, n, &sB, ptr_memory_strmat);
	ptr_memory_strmat += sB.memsize;
	blasfeo_pack_dmat(n, n, B, n, &sB, 0, 0);
	blasfeo_print_dmat(n, n, &sB, 0, 0);

	struct blasfeo_dmat sC;
//	blasfeo_allocate_dmat(n, n, &sC);
	blasfeo_create_dmat(n, n, &sC, ptr_memory_strmat);
	ptr_memory_strmat += sC.memsize;

	struct blasfeo_dmat sD;
//	blasfeo_allocate_dmat(n, n, &sD);
	blasfeo_create_dmat(n, n, &sD, ptr_memory_strmat);
	ptr_memory_strmat += sD.memsize;
	blasfeo_pack_dmat(n, n, D, n, &sD, 0, 0);

	struct blasfeo_dmat sE;
//	blasfeo_allocate_dmat(n, n, &sE);
	blasfeo_create_dmat(n, n, &sE, ptr_memory_strmat);
	ptr_memory_strmat += sE.memsize;

	struct blasfeo_dvec sx_n;
	blasfeo_allocate_dvec(n, &sx_n);
	blasfeo_pack_dvec(n, x_n, 1, &sx_n, 0);

	struct blasfeo_dvec sx_t;
	blasfeo_allocate_dvec(n, &sx_t);
	blasfeo_pack_dvec(n, x_t, 1, &sx_t, 0);

	struct blasfeo_dvec sy_n;
	blasfeo_allocate_dvec(n, &sy_n);
	blasfeo_pack_dvec(n, y_n, 1, &sy_n, 0);

	struct blasfeo_dvec sy_t;
	blasfeo_allocate_dvec(n, &sy_t);
	blasfeo_pack_dvec(n, y_t, 1, &sy_t, 0);

	struct blasfeo_dvec sz_n;
	blasfeo_allocate_dvec(n, &sz_n);
	blasfeo_pack_dvec(n, z_n, 1, &sz_n, 0);

	struct blasfeo_dvec sz_t;
	blasfeo_allocate_dvec(n, &sz_t);
	blasfeo_pack_dvec(n, z_t, 1, &sz_t, 0);

	struct blasfeo_dvec sx0; blasfeo_create_dvec(n, &sx0, x0);
	struct blasfeo_dvec sx1; blasfeo_create_dvec(n, &sx1, x1);
	struct blasfeo_dvec sx2; blasfeo_create_dvec(n, &sx2, x2);
	struct blasfeo_dvec sx3; blasfeo_create_dvec(n, &sx3, x3);
	struct blasfeo_dvec sx4; blasfeo_create_dvec(n, &sx4, x4);
	struct blasfeo_dvec sx5; blasfeo_create_dvec(n, &sx5, x5);
	struct blasfeo_dvec sx6; blasfeo_create_dvec(n, &sx6, x6);
	struct blasfeo_dvec sx7; blasfeo_create_dvec(n, &sx7, x7);
//	struct blasfeo_dvec sx8; blasfeo_create_dvec(n, &sx8, x8);
//	struct blasfeo_dvec sx9; blasfeo_create_dvec(n, &sx9, x9);

	struct blasfeo_dvec sz0; blasfeo_allocate_dvec(n, &sz0);
	struct blasfeo_dvec sz1; blasfeo_allocate_dvec(n, &sz1);
	struct blasfeo_dvec sz2; blasfeo_allocate_dvec(n, &sz2);
	struct blasfeo_dvec sz3; blasfeo_allocate_dvec(n, &sz3);
	struct blasfeo_dvec sz4; blasfeo_allocate_dvec(n, &sz4);
	struct blasfeo_dvec sz5; blasfeo_allocate_dvec(n, &sz5);
	struct blasfeo_dvec sz6; blasfeo_allocate_dvec(n, &sz6);
	struct blasfeo_dvec sz7; blasfeo_allocate_dvec(n, &sz7);
//	struct blasfeo_dvec sz8; blasfeo_allocate_dvec(n, &sz8);
//	struct blasfeo_dvec sz9; blasfeo_allocate_dvec(n, &sz9);

	// tests
	double *v; d_zeros(&v, n, 1);
	double *vp; d_zeros(&vp, n, 1);
	double *vm; d_zeros(&vm, n, 1);
	double *m; d_zeros(&m, n, 1);
	double *r; d_zeros(&r, n, 1);

	for(ii=0; ii<n; ii++) v[ii] = ii; // x
	for(ii=0; ii<n; ii++) vp[ii] = 8.0; // upper
	for(ii=0; ii<n; ii++) vm[ii] = 3.0; // lower
	for(ii=0; ii<n; ii++) r[ii] = 2*ii+1; // x

	d_print_mat(1, n, v, 1);
	d_print_mat(1, n, vp, 1);
	d_print_mat(1, n, vm, 1);
	d_print_mat(1, n, r, 1);

	struct blasfeo_dvec sv; blasfeo_create_dvec(n, &sv, v);
	struct blasfeo_dvec svp; blasfeo_create_dvec(n, &svp, vp);
	struct blasfeo_dvec svm; blasfeo_create_dvec(n, &svm, vm);
	struct blasfeo_dvec sm; blasfeo_create_dvec(n, &sm, m);
	struct blasfeo_dvec sr; blasfeo_create_dvec(n, &sr, r);

	double alpha, beta;



#if 0
	double *ptr = sA.pA+1;
	printf("\n%f %p\n", *ptr, ptr);
	blasfeo_align_64_byte(ptr, (void **) &ptr);
	printf("\n%f %p\n", *ptr, ptr);
	return 0;
#endif

#if 0
	// panel copy
//	kernel_dpacp_nn_4_lib4(6, 3, sA.pA, sA.cn, sD.pA);
//	kernel_dpacp_nn_4_vs_lib4(6, 3, sA.pA, sA.cn, sD.pA, 1);
//	kernel_dpacp_nn_8_lib4(6, 3, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_dpacp_nn_8_vs_lib4(6, 3, sA.pA, sA.cn, sD.pA, sD.cn, 7);
//	kernel_dpacp_nn_12_lib4(6, 3, sA.pA, sA.cn, sD.pA, sD.cn);
	kernel_dpacp_nn_12_vs_lib4(6, 3, sA.pA, sA.cn, sD.pA, sD.cn, 11);
	blasfeo_print_dmat(12, n, &sD, 0, 0);
	return 0;
	
#endif

#if 1
	// gemm_nt
	alpha = 1.0;
	beta = 1.0;

	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	d_print_mat(n, n, D, n);

//	kernel_dgemm_nt_4x2_lib4(4, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_dgemm_nt_4x4_lib4(4, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_dgemm_nt_4x4_lib4(4, &alpha, sA.pA, sB.pA+1*4*sB.cn, &beta, sD.pA+1*4*4, sD.pA+1*4*4);
//	kernel_dgemm_nt_4x4_vs_lib4(4, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA, 4, 4);

//	kernel_dgemm_nt_8x4_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_dgemm_nt_8x4_libc4cc(n, &alpha, A, n, sB.pA, &beta, D, n, D, n);
//	kernel_dgemm_nt_8x4_vs_libc4cc(n, &alpha, A, n, sB.pA, &beta, D, n, D, n, 7, 3);

//	kernel_dgemm_nt_12x4_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_dgemm_nt_12x4_gen_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, 0, sA.pA, sD.cn, 3, sD.pA, sD.cn, 0, 12, 0, 4);
//	kernel_dgemm_nt_4x12_lib4ccc(n, &alpha, sA.pA, B, n, &beta, D, n, D, n);

//	kernel_dgemm_nt_8x8_lib8(8, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_dgemm_nt_8x8_vs_lib8(8, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA, 8, 8);
//	kernel_dgemm_nt_8x8_gen_lib8(8, &alpha, sA.pA, sB.pA, &beta, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 8, 0, 8);
//	kernel_dgemm_nn_8x8_lib8(8, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sA.pA, sD.pA);
//	kernel_dgemm_nn_8x8_vs_lib8(8, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sA.pA, sD.pA, 8, 8);
//	kernel_dgemm_nn_8x8_gen_lib8(8, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 8, 0, 8);
//	kernel_dgemm_tt_8x8_lib8(8, &alpha, 0, sA.pA, sA.cn, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_dgemm_tt_8x8_vs_lib8(8, &alpha, 0, sA.pA, sA.cn, sB.pA, &beta, sA.pA, sD.pA, 8, 8);
//	kernel_dgemm_tt_8x8_gen_lib8(8, &alpha, 0, sA.pA, sA.cn, sB.pA, &beta, 0, sA.pA, sA.cn, 1, sD.pA, sD.cn, 0, 8, 0, 8);
//	kernel_dsyrk_nt_l_8x8_lib8(8, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_dsyrk_nt_l_8x8_vs_lib8(8, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA, 8, 8);
//	kernel_dsyrk_nt_l_8x8_gen_lib8(8, &alpha, sA.pA, sB.pA, &beta, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 8, 0, 8);
//	kernel_dtrmm_nn_rl_8x8_lib8(n, &alpha, sB.pA, 0, sA.pA, sA.cn, sD.pA);
//	kernel_dtrmm_nn_rl_8x8_vs_lib8(8, &alpha, sB.pA, 0, sA.pA, sA.cn, sD.pA, 8, 8);
//	kernel_dtrmm_nn_rl_8x8_gen_lib8(8, &alpha, sB.pA, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 8, 0, 8);
	
//	kernel_dgemm_nt_16x8_lib8(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_dgemm_nt_16x8_vs_lib8(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 16, 8);
//	kernel_dgemm_nt_16x8_gen_lib8(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 16, 0, 8);
//	kernel_dgemm_nn_16x8_lib8(8, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_dgemm_nn_16x8_vs_lib8(8, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 16, 8);
//	kernel_dgemm_tt_8x16_lib8(16, &alpha, 0, sA.pA, sA.cn, sB.pA, sB.cn, &beta, sA.pA, sD.pA);
//	kernel_dgemm_tt_8x16_vs_lib8(16, &alpha, 0, sA.pA, sA.cn, sB.pA, sB.cn, &beta, sA.pA, sD.pA, 8, 16);
//	kernel_dgemm_tt_8x16_gen_lib8(16, &alpha, 0, sA.pA, sA.cn, sB.pA, sB.cn, &beta, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 8, 0, 16);
//	kernel_dsyrk_nt_l_16x8_gen_lib8(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 16, 0, 8);
//	kernel_dgemm_nt_8x16_lib8(16, &alpha, sA.pA, sB.pA, sB.cn, &beta, sA.pA, sD.pA);
//	kernel_dgemm_nt_8x16_vs_lib8(16, &alpha, sA.pA, sB.pA, sB.cn, &beta, sA.pA, sD.pA, 8, 16);
//	kernel_dsyrk_nt_l_16x8_lib8(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_dsyrk_nt_l_16x8_vs_lib8(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 16, 8);
//	kernel_dtrmm_nn_rl_16x8_lib8(n, &alpha, sB.pA, sB.cn, 0, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_dtrmm_nn_rl_16x8_vs_lib8(n, &alpha, sB.pA, sB.cn, 0, sA.pA, sA.cn, sD.pA, sD.cn, 16, 8);
//	kernel_dtrmm_nn_rl_16x8_gen_lib8(n, &alpha, sB.pA, sB.cn, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 0, 16, 0, 8);

//	kernel_dpacp_nn_16_lib8(n, 0, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_dpacp_nn_16_vs_lib8(n, 0, sA.pA, sA.cn, sD.pA, sD.cn, 16);
//	kernel_dpacp_nn_8_lib8(n, 0, sA.pA, sA.cn, sD.pA);
//	kernel_dpacp_nn_8_vs_lib8(n, 0, sA.pA, sA.cn, sD.pA, 8);
//	kernel_dpacp_tn_8_lib8(n, 0, sA.pA, sA.cn, sD.pA);
//	kernel_dpacp_tn_8_vs_lib8(n, 0, sA.pA, sA.cn, sD.pA, 8);
//	kernel_dpacp_l_nn_8_lib8(4, 0, sA.pA, sA.cn, sD.pA);
//	kernel_dpacp_l_nn_8_vs_lib8(4, 0, sA.pA, sA.cn, sD.pA, 8);
//	kernel_dpacp_l_tn_8_lib8(4, 0, sA.pA, sA.cn, sD.pA);
//	kernel_dpacp_l_tn_8_vs_lib8(4, 0, sA.pA, sA.cn, sD.pA, 8);
//	kernel_dpaad_nn_8_lib8(n, &alpha, 0, sA.pA, sA.cn, sD.pA);
//	kernel_dpaad_nn_8_vs_lib8(n, &alpha, 0, sA.pA, sA.cn, sD.pA, 8);

//	kernel_dpack_nn_16_lib8(n, A, n, sD.pA, sD.cn);
//	kernel_dpack_nn_16_vs_lib8(n, A, n, sD.pA, sD.cn, 16);
//	kernel_dpack_nn_8_lib8(n, A, n, sD.pA);
//	kernel_dpack_nn_8_vs_lib8(n, A, n, sD.pA, 8);
//	kernel_dpack_tn_8_lib8(n, A, n, sD.pA);
//	kernel_dpack_tn_8_vs_lib8(n, A, n, sD.pA, 8);
//	kernel_dpack_tt_4_lib8(n, A, n, sD.pA, sD.cn);
//	kernel_dpack_tt_4_vs_lib8(n, A, n, sD.pA, sD.cn, 4);

//	kernel_dgemm_nt_8x8_lib88cc(8, &alpha, sA.pA, sB.pA, &beta, D, n, D, n);
//	kernel_dgemm_nt_8x8_vs_lib88cc(8, &alpha, sA.pA, sB.pA, &beta, D, n, D, n, 8, 8);
//	kernel_dgemm_nt_8x8_lib8ccc(8, &alpha, sA.pA, B, n, &beta, D, n, D, n);
//	kernel_dgemm_nt_8x8_vs_lib8ccc(8, &alpha, sA.pA, B, n, &beta, D, n, D, n, 8, 8);
//	kernel_dgemm_nt_8x8_libc8cc(8, &alpha, A, n, sB.pA, &beta, D, n, D, n);
//	kernel_dgemm_nt_8x8_vs_libc8cc(8, &alpha, A, n, sB.pA, &beta, D, n, D, n, 8, 8);
//	kernel_dgemm_nn_8x8_lib8ccc(8, &alpha, sA.pA, B, n, &beta, D, n, D, n);
//	kernel_dgemm_nn_8x8_vs_lib8ccc(8, &alpha, sA.pA, B, n, &beta, D, n, D, n, 8, 8);
//	kernel_dgemm_tt_8x8_libc8cc(8, &alpha, A, n, sB.pA, &beta, D, n, D, n);
//	kernel_dgemm_tt_8x8_vs_libc8cc(8, &alpha, A, n, sB.pA, &beta, D, n, D, n, 8, 8);

//	kernel_dgemm_nt_16x8_lib88cc(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, D, n, D, n);
//	kernel_dgemm_nt_16x8_vs_lib88cc(8, &alpha, sA.pA, sA.cn, sB.pA, &beta, D, n, D, n, 16, 8);
//	kernel_dgemm_nt_16x8_lib8ccc(8, &alpha, sA.pA, sA.cn, B, n, &beta, D, n, D, n);
//	kernel_dgemm_nt_16x8_vs_lib8ccc(8, &alpha, sA.pA, sA.cn, B, n, &beta, D, n, D, n, 16, 8);
//	kernel_dgemm_nn_16x8_lib8ccc(8, &alpha, sA.pA, sA.cn, B, n, &beta, D, n, D, n);
//	kernel_dgemm_nn_16x8_vs_lib8ccc(8, &alpha, sA.pA, sA.cn, B, n, &beta, D, n, D, n, 16, 8);
//	kernel_dgemm_nt_8x16_libc8cc(16, &alpha, A, n, sB.pA, sB.cn, &beta, D, n, D, n);
//	kernel_dgemm_nt_8x16_vs_libc8cc(16, &alpha, A, n, sB.pA, sB.cn, &beta, D, n, D, n, 8, 16);
//	kernel_dgemm_tt_8x16_libc8cc(16, &alpha, A, n, sB.pA, sB.cn, &beta, D, n, D, n);
//	kernel_dgemm_tt_8x16_vs_libc8cc(16, &alpha, A, n, sB.pA, sB.cn, &beta, D, n, D, n, 8, 16);

//	blasfeo_dgemm_nn(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sA, 0, 0, &sD, 0, 0);
//	blasfeo_dgemm_nt(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dgemm_tn(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dgemm_tt(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_ln(n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_ln_mn(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dpotrf_l(n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dpotrf_l_mn(n, n, &sD, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_dpotrf_ln(n, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_dpotrf_ln_mn(n, n, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dtrmm_rlnn(n, n, alpha, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dtrsm_rltn(n, n, alpha, &sD, 0, 0, &sB, 0, 0, &sE, 0, 0);

//	blasfeo_dgecp(n, n, &sA, 0, 0, &sD, 0, 0);
//	blasfeo_dtrcp_l(n, &sA, 0, 0, &sD, 0, 0);
//	blasfeo_dgetr(n, n, &sA, 0, 0, &sD, 0, 0);
//	blasfeo_dtrtr_l(n, &sA, 0, 0, &sD, 0, 0);
//	blasfeo_dgead(n, n, alpha, &sA, 0, 0, &sD, 0, 0);
	
//	kernel_dpotrf_nt_l_8x8_lib8(0, sA.pA, sA.pA, sD.pA, sD.pA, sD.dA);
//	kernel_dpotrf_nt_l_8x8_vs_lib8(0, sA.pA, sA.pA, sD.pA, sD.pA, sD.dA, 8, 8);
//	kernel_dpotrf_nt_l_16x8_lib8(0, sA.pA, sA.cn, sA.pA, sD.pA, sD.cn, sD.pA, sD.cn, sD.dA);
//	kernel_dpotrf_nt_l_16x8_vs_lib8(0, sA.pA, sA.cn, sA.pA, sD.pA, sD.cn, sD.pA, sD.cn, sD.dA, 16, 8);
//	kernel_dsyrk_dpotrf_nt_l_8x8_lib8(n, sA.pA, sA.pA, 0, sA.pA, sA.pA, sB.pA, sD.pA, sD.dA);
//	kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(n, sA.pA, sA.pA, 0, sA.pA, sA.pA, sB.pA, sD.pA, sD.dA, 8, 8);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_print_dmat(n, n, &sE, 0, 0);
//	d_print_mat(n, n, D, n);
	return 0;
#endif

#if 0
	// gemm_nn
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);

//	kernel_dgemm_nn_4x4_lib4(4, &alpha, sB.pA, 0, sA.pA, sA.cn, &beta, sA.pA, sD.pA);

	blasfeo_dgemm_nn(8, 4, n, alpha, &sA, 1, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// gemm_tn
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	blasfeo_dgemm_tn(8, 8, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 1, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// gemm_tt
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	blasfeo_dgemm_tt(8, 8, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 1, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// syrk_ln
	alpha = 1.0;
	beta = 0.0;
//	blasfeo_print_dmat(n, n, &sD, 0, 0);

//	kernel_dsyrk_nt_l_4x4_lib4(4, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_dsyrk_nt_l_4x4_vs_lib4(4, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA, 3, 4);
//	kernel_dsyrk_nt_l_4x4_gen_lib4(4, &alpha, sA.pA, sB.pA, &beta, 0, sA.pA, sA.cn, 3, sD.pA, sD.cn, 3, 4, 0, 4);
//	kernel_dsyrk_nt_l_8x4_gen_lib4(4, &alpha, sA.pA, sA.cn, sB.pA, &beta, 0, sA.pA, sA.cn, 1, sD.pA, sD.cn, 0, 5, 0, 4);

	blasfeo_dsyrk_ln(5, 12, alpha, &sB, 0, 0, &sA, 1, 0, beta, &sD, 1, 0, &sD, 1, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// syrk_lt
	alpha = 1.0;
	beta = 0.0;
//	blasfeo_print_dmat(n, n, &sD, 0, 0);

	blasfeo_dsyrk_lt(12, 12, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// syrk_un
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);

//	kernel_dsyrk_nt_u_4x4_lib4(4, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_dsyrk_nt_u_4x4_vs_lib4(4, &alpha, sA.pA, sB.pA, &beta, sA.pA, sD.pA, 3, 4);
//	kernel_dsyrk_nt_u_4x4_gen_lib4(4, &alpha, sA.pA, sB.pA, &beta, 0, sA.pA, sA.cn, 0, sD.pA, sD.cn, 1, 4, 1, 3);

	blasfeo_dsyrk_un(11, 11, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// syrk_ut
	alpha = 1.0;
	beta = 0.0;
//	blasfeo_print_dmat(n, n, &sD, 0, 0);

	blasfeo_dsyrk_ut(11, 11, alpha, &sB, 0, 0, &sA, 1, 0, beta, &sD, 0, 0, &sD, 0, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// trmm_rutn
	alpha = -1.0;
	beta = 0.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);

//	kernel_dgemm_nn_4x4_lib4(8, &alpha, sA.pA+4*sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sD.pA);

	blasfeo_dtrmm_rutn(n, n, alpha, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// trmm_rlnn
	alpha = 1.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	blasfeo_dtrmm_rlnn(9, n, alpha, &sA, 3, 0, &sB, 0, 0, &sD, 0, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// trsm_llnn
	alpha = 2.0;
	beta = 1.0;

	blasfeo_dsyrk_ln(n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sC, 0, 0);
	blasfeo_dpotrf_l(n, &sC, 0, 0, &sC, 0, 0);

	blasfeo_print_dmat(n, n, &sC, 0, 0);

	blasfeo_dtrsm_llnn(15, 15, alpha, &sC, 0, 0, &sB, 0, 0, &sD, 0, 0);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// potrf
	alpha = 1.0;
	beta = 1.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	blasfeo_dgemm_nt(n, n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dsyrk_ln(n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_ln_mn(n, n-1, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_dpotrf_l(n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dtrsm_rltn(7, 4, 1.0, &sD, 0, 0, &sD, 4, 0, &sD, 4, 0);
//	blasfeo_dpotrf_l_mn(n, 7, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_dpotrf_ln(n, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_dpotrf_ln_mn(n-1, n-3, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// gemv_n
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_tran_dvec(n, &sz_n, 0);
//	kernel_dgemv_n_8_lib8(n, &alpha, sA.pA, sx_n.pa, &beta, sy_n.pa, sz_n.pa);
//	kernel_dgemv_n_8_vs_lib8(n, &alpha, sA.pA, sx_n.pa, &beta, sy_n.pa, sz_n.pa, 5);
//	kernel_dgemv_n_8_gen_lib8(n, &alpha, sA.pA, sx_n.pa, &beta, sy_n.pa, sz_n.pa, 3, 5);
//	kernel_dgemv_n_16_lib8(n, &alpha, sA.pA, sA.cn, sx_n.pa, &beta, sy_n.pa, sz_n.pa);
//	blasfeo_dgemv_n(n, n, 1.0, &sA, 0, 0, &sx_n, 0, 0.0, &sy_n, 0, &sz_n, 0);
	blasfeo_dtrmv_lnn(n, &sA, 0, 0, &sx_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;
#endif

#if 0
	// gemv_t
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_tran_dvec(n, &sz_n, 0);
//	kernel_dgemv_t_8_lib8(12, &alpha, 1, sA.pA+1, sA.cn, sx_n.pa+0, &beta, sy_n.pa, sz_n.pa);
//	kernel_dgemv_t_8_vs_lib8(12, &alpha, 1, sA.pA+1, sA.cn, sx_n.pa+0, &beta, sy_n.pa, sz_n.pa, 3);
//	blasfeo_dgemv_t(n, n, 1.0, &sA, 0, 0, &sx_n, 0, 0.0, &sy_n, 0, &sz_n, 0);
	blasfeo_dtrmv_ltn(n, &sA, 0, 0, &sx_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;
#endif

#if 0
	// trsv_lnn
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	blasfeo_dtrsv_lnn(13, &sA, 0, 1, &sx_n, 0, &sz_n, 0);
//	blasfeo_dtrsv_lnn_mn(13, 13, &sA, 0, 1, &sx_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;
#endif

#if 0
	// trsv_ltn
	blasfeo_print_tran_dvec(n, &sx_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
//	blasfeo_dtrsv_ltn(13, &sA, 0, 1, &sx_n, 0, &sz_n, 0);
	blasfeo_dtrsv_ltn_mn(7, 5, &sA, 0, 1, &sx_n, 0, &sz_n, 0);
//	blasfeo_ref_dtrsv_ltn_mn(7, 5, &sA, 0, 1, &sx_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;
#endif

#if 0
	// symv_l
	alpha = 1.0;
	blasfeo_print_tran_dvec(n, &sx_n, 0);
//	kernel_dsymv_l_8_lib8(3, &alpha, sA.pA, sA.cn, sx_n.pa, sz_n.pa);
//	kernel_dsymv_l_8_vs_lib8(3, &alpha, sA.pA, sA.cn, sx_n.pa, sz_n.pa, 8);
//	blasfeo_dsymv_l(n, 1.0, &sA, 0, 0, &sx_n, 0, 0.0, &sy_n, 0, &sz_n, 0);
	blasfeo_dsymv_l_mn(n, n, 1.0, &sA, 0, 0, &sx_n, 0, 0.0, &sy_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;
#endif

#if 0
	// gemv_nt
	blasfeo_print_tran_dvec(n, &sx_n, 0);
	blasfeo_dgemv_nt(n, n, 1.0, 1.0, &sA, 0, 0, &sx_n, 0, &sx_t, 0, 0.0, 0.0, &sy_n, 0, &sy_t, 0, &sz_n, 0, &sz_t, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_t, 0);
	return 0;
#endif

#if 0
	// getrf
	blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);

	blasfeo_dgetrf_np(n, n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dgetrf_rp(n, n, &sD, 0, 0, &sD, 0, 0, ipiv);

	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// array lq
	struct blasfeo_dmat lq0; blasfeo_allocate_dmat(n, 2*n, &lq0);
	struct blasfeo_dmat lq1; blasfeo_allocate_dmat(n, 2*n, &lq1);

	void *lq0_work = malloc(blasfeo_dgelqf_worksize(n, 2*n));

	blasfeo_pack_dmat(n, n, A, n, &lq0, 0, n);
	blasfeo_pack_dmat(n, n, B, n, &lq0, 0, 0);

	blasfeo_print_dmat(n, 2*n, &lq0, 0, 0);

//	blasfeo_dgelqf(n, 2*n, &lq0, 0, 0, &lq0, 0, 0, lq0_work);
//	blasfeo_dgelqf_pd(n, 2*n, &lq0, 0, 0, &lq0, 0, 0, lq0_work);
	blasfeo_dgelqf_pd_la(n, n, &lq0, 0, 0, &lq0, 0, n, lq0_work);

	blasfeo_print_dmat(n, 2*n, &lq0, 0, 0);
//	blasfeo_print_dmat(n, n, &lq0, 0, 0);
//	return 0;

	blasfeo_dtrcp_l(n, &lq0, 0, 0, &lq1, 0, 0);
	blasfeo_pack_dmat(n, n, B, n, &lq1, 0, n);

	blasfeo_print_dmat(n, 2*n, &lq1, 0, 0);

//	blasfeo_dgelqf_pd(n, 2*n, &lq1, 0, 0, &lq1, 0, 0, lq0_work);
//	blasfeo_dgelqf_pd_la(n, n, &lq1, 0, 0, &lq1, 0, n, lq0_work);
	blasfeo_dgelqf_pd_lla(n, 0, &lq1, 0, 0, &lq1, 0, n, &lq1, 0, 2*n, lq0_work);

	blasfeo_print_dmat(n, 2*n, &lq1, 0, 0);

	return 0;
#endif


//	blasfeo_print_tran_dvec(n, &sv, 0);
//	blasfeo_print_tran_dvec(n, &svp, 0);
//	blasfeo_print_tran_dvec(n, &svm, 0);
//	blasfeo_print_tran_dvec(n, &sm, 0);
//	blasfeo_print_tran_dvec(n, &sr, 0);

//	blasfeo_print_tran_dvec(n, &sm, 0);
//	BLASFEO_DVECEL(&sm, 0) = 0.0;
//	BLASFEO_DVECEL(&sm, 1) = 1.0;
//	BLASFEO_DVECEL(&sm, 2) = 2.0;
//	blasfeo_print_tran_dvec(n, &sm, 0);
//	return 0;

	// copy scale
#if 0
	blasfeo_print_dmat(n, n, &sA, 0, 0);
	blasfeo_dgecpsc(5, 5, 0.1, &sA, 3, 0, &sD, 3, 0);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

	// givens rotations
#if 0
	BLASFEO_DMATEL(&sD, 0, 0) = 6.0;
	BLASFEO_DMATEL(&sD, 0, 1) = 5.0;
	BLASFEO_DMATEL(&sD, 0, 2) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 0) = 5.0;
	BLASFEO_DMATEL(&sD, 1, 1) = 1.0;
	BLASFEO_DMATEL(&sD, 1, 2) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 0) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 1) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 2) = 3.0;
	//
	BLASFEO_DMATEL(&sD, 0, 5) = 1.0;
	BLASFEO_DMATEL(&sD, 0, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 0, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 6) = 1.0;
	BLASFEO_DMATEL(&sD, 1, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 7) = 1.0;
	//
	BLASFEO_DMATEL(&sD, 5, 5) = 1.0;
	BLASFEO_DMATEL(&sD, 5, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 5, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 6, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 6, 6) = 1.0;
	BLASFEO_DMATEL(&sD, 6, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 7, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 7, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 7, 7) = 1.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	double aa, bb, c, s;
	//
	aa = BLASFEO_DMATEL(&sD, 0, 0);
	bb = BLASFEO_DMATEL(&sD, 1, 0);
//	c = aa/sqrt(aa*aa+bb*bb);
//	s = bb/sqrt(aa*aa+bb*bb);
	blasfeo_drotg(aa, bb, &c, &s);
	blasfeo_drowrot(3, &sD, 0, 1, 0, c, s);
	blasfeo_drowrot(3, &sD, 0, 1, 5, c, s);
	blasfeo_dcolrot(3, &sD, 5, 5, 6, c, s);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	//
	aa = BLASFEO_DMATEL(&sD, 1, 1);
	bb = BLASFEO_DMATEL(&sD, 2, 1);
//	c = aa/sqrt(aa*aa+bb*bb);
//	s = bb/sqrt(aa*aa+bb*bb);
	blasfeo_drotg(aa, bb, &c, &s);
	blasfeo_drowrot(2, &sD, 1, 2, 1, c, s);
	blasfeo_drowrot(3, &sD, 1, 2, 5, c, s);
	blasfeo_dcolrot(3, &sD, 5, 6, 7, c, s);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif
#if 0
	BLASFEO_DMATEL(&sD, 0, 0) = 6.0;
	BLASFEO_DMATEL(&sD, 0, 1) = 5.0;
	BLASFEO_DMATEL(&sD, 0, 2) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 0) = 5.0;
	BLASFEO_DMATEL(&sD, 1, 1) = 1.0;
	BLASFEO_DMATEL(&sD, 1, 2) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 0) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 1) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 2) = 3.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	double aa, bb, c, s;
	//
	aa = BLASFEO_DMATEL(&sD, 0, 0);
	bb = BLASFEO_DMATEL(&sD, 1, 0);
	c =  bb/sqrt(aa*aa+bb*bb);
	s = -aa/sqrt(aa*aa+bb*bb);
	blasfeo_drowrot(3, &sD, 0, 1, 0, c, s);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif
#if 0
	BLASFEO_DMATEL(&sD, 0, 0) = 6.0;
	BLASFEO_DMATEL(&sD, 0, 1) = 5.0;
	BLASFEO_DMATEL(&sD, 0, 2) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 0) = 5.0;
	BLASFEO_DMATEL(&sD, 1, 1) = 1.0;
	BLASFEO_DMATEL(&sD, 1, 2) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 0) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 1) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 2) = 3.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	double aa, bb, c, s;
	//
	aa = BLASFEO_DMATEL(&sD, 0, 0);
	bb = BLASFEO_DMATEL(&sD, 0, 1);
	c =  bb/sqrt(aa*aa+bb*bb);
	s = -aa/sqrt(aa*aa+bb*bb);
	blasfeo_dcolrot(3, &sD, 0, 0, 1, c, s);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif
#if 0
	BLASFEO_DMATEL(&sD, 0, 0) = 6.0;
	BLASFEO_DMATEL(&sD, 0, 1) = 5.0;
	BLASFEO_DMATEL(&sD, 0, 2) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 0) = 5.0;
	BLASFEO_DMATEL(&sD, 1, 1) = 1.0;
	BLASFEO_DMATEL(&sD, 1, 2) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 0) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 1) = 4.0;
	BLASFEO_DMATEL(&sD, 2, 2) = 3.0;
	//
	BLASFEO_DMATEL(&sD, 0, 5) = 1.0;
	BLASFEO_DMATEL(&sD, 0, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 0, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 1, 6) = 1.0;
	BLASFEO_DMATEL(&sD, 1, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 2, 7) = 1.0;
	//
	BLASFEO_DMATEL(&sD, 5, 5) = 1.0;
	BLASFEO_DMATEL(&sD, 5, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 5, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 6, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 6, 6) = 1.0;
	BLASFEO_DMATEL(&sD, 6, 7) = 0.0;
	BLASFEO_DMATEL(&sD, 7, 5) = 0.0;
	BLASFEO_DMATEL(&sD, 7, 6) = 0.0;
	BLASFEO_DMATEL(&sD, 7, 7) = 1.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	double aa, bb, c, s;
	//
	aa = BLASFEO_DMATEL(&sD, 0, 0);
	bb = BLASFEO_DMATEL(&sD, 0, 1);
//	c = aa/sqrt(aa*aa+bb*bb);
//	s = bb/sqrt(aa*aa+bb*bb);
	blasfeo_drotg(aa, bb, &c, &s);
	blasfeo_dcolrot(3, &sD, 0, 0, 1, c, s);
	blasfeo_dcolrot(3, &sD, 0, 5, 6, c, s);
	blasfeo_drowrot(3, &sD, 5, 6, 5, c, s);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	//
	aa = BLASFEO_DMATEL(&sD, 1, 1);
	bb = BLASFEO_DMATEL(&sD, 1, 2);
//	c = aa/sqrt(aa*aa+bb*bb);
//	s = bb/sqrt(aa*aa+bb*bb);
	blasfeo_drotg(aa, bb, &c, &s);
	blasfeo_dcolrot(2, &sD, 1, 1, 2, c, s);
	blasfeo_dcolrot(3, &sD, 0, 6, 7, c, s);
	blasfeo_drowrot(3, &sD, 6, 7, 5, c, s);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
#endif

	alpha = 1.0;
	beta = 1.0;
	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	kernel_dgemm_nn_4x8_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//	kernel_dgemm_nn_4x8_vs_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA, 4, 8);
//	kernel_dgemm_nn_8x2_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_dgemm_nn_8x2_vs_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn, 8, 2);
//	kernel_dgemm_nn_2x8_lib4(3, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//	kernel_dgemm_nn_6x8_vs_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 6, 8);
//	kernel_dgemm_nn_10x4_vs_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 10, 4);
//	kernel_dgemm_nn_10x2_vs_lib4(n, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 8, 1);
//	kernel_dgemm_nn_12x4_lib4(2, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn);
//	blasfeo_dgemm_nn(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dgemm_nt(n, n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	return 0;
//	blasfeo_dgetrf_nopivot(n, n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dgetrf_rowpivot(n, n, &sD, 0, 0, &sD, 0, 0, ipiv);
//	blasfeo_dpotrf_l(n, &sD, 0, 0, &sD, 0, 0);
//	dpstrf_l_libstr(n, &sD, 0, 0, &sD, 0, 0, ipiv);
//	int_print_mat(1, n, ipiv, 1);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
	//
	blasfeo_dgemm_nt(n, n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
	blasfeo_dcolpe(n, ipiv, &sD);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_drowpe(n, ipiv, &sD);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_dpotrf_l(n, &sD, 0, 0, &sD, 0, 0);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
	//
#if 0
	// N scheme
#if 1
	blasfeo_dvecpe(n, ipiv, &sx0, 0);
	blasfeo_dvecpe(n, ipiv, &sx1, 0);
	blasfeo_dvecpe(n, ipiv, &sx2, 0);
	blasfeo_dvecpe(n, ipiv, &sx3, 0);
	blasfeo_dvecpe(n, ipiv, &sx4, 0);
	blasfeo_dvecpe(n, ipiv, &sx5, 0);
	blasfeo_dvecpe(n, ipiv, &sx6, 0);
	blasfeo_dvecpe(n, ipiv, &sx7, 0);
#endif
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx0, 0, &sz0, 0);
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx1, 0, &sz1, 0);
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx2, 0, &sz2, 0);
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx3, 0, &sz3, 0);
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx4, 0, &sz4, 0);
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx5, 0, &sz5, 0);
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx6, 0, &sz6, 0);
	blasfeo_dtrsv_lnu(n, &sD, 0, 0, &sx7, 0, &sz7, 0);
	//
	blasfeo_print_tran_dvec(n, &sz0, 0);
	blasfeo_print_tran_dvec(n, &sz1, 0);
	blasfeo_print_tran_dvec(n, &sz2, 0);
	blasfeo_print_tran_dvec(n, &sz3, 0);
	blasfeo_print_tran_dvec(n, &sz4, 0);
	blasfeo_print_tran_dvec(n, &sz5, 0);
	blasfeo_print_tran_dvec(n, &sz6, 0);
	blasfeo_print_tran_dvec(n, &sz7, 0);
	//
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz0, 0, &sz0, 0);
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz1, 0, &sz1, 0);
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz2, 0, &sz2, 0);
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz3, 0, &sz3, 0);
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz4, 0, &sz4, 0);
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz5, 0, &sz5, 0);
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz6, 0, &sz6, 0);
	blasfeo_dtrsv_unn(n, &sD, 0, 0, &sz7, 0, &sz7, 0);
	//
	blasfeo_print_tran_dvec(n, &sz0, 0);
	blasfeo_print_tran_dvec(n, &sz1, 0);
	blasfeo_print_tran_dvec(n, &sz2, 0);
	blasfeo_print_tran_dvec(n, &sz3, 0);
	blasfeo_print_tran_dvec(n, &sz4, 0);
	blasfeo_print_tran_dvec(n, &sz5, 0);
	blasfeo_print_tran_dvec(n, &sz6, 0);
	blasfeo_print_tran_dvec(n, &sz7, 0);
#else
	// T scheme
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx0, 0, &sz0, 0);
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx1, 0, &sz1, 0);
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx2, 0, &sz2, 0);
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx3, 0, &sz3, 0);
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx4, 0, &sz4, 0);
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx5, 0, &sz5, 0);
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx6, 0, &sz6, 0);
	blasfeo_dtrsv_utn(n, &sD, 0, 0, &sx7, 0, &sz7, 0);
	//
	blasfeo_print_tran_dvec(n, &sz0, 0);
	blasfeo_print_tran_dvec(n, &sz1, 0);
	blasfeo_print_tran_dvec(n, &sz2, 0);
	blasfeo_print_tran_dvec(n, &sz3, 0);
	blasfeo_print_tran_dvec(n, &sz4, 0);
	blasfeo_print_tran_dvec(n, &sz5, 0);
	blasfeo_print_tran_dvec(n, &sz6, 0);
	blasfeo_print_tran_dvec(n, &sz7, 0);
	//
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz0, 0, &sz0, 0);
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz1, 0, &sz1, 0);
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz2, 0, &sz2, 0);
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz3, 0, &sz3, 0);
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz4, 0, &sz4, 0);
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz5, 0, &sz5, 0);
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz6, 0, &sz6, 0);
	blasfeo_dtrsv_ltu(n, &sD, 0, 0, &sz7, 0, &sz7, 0);
	//
#if 1
	blasfeo_dvecpei(n, ipiv, &sz0, 0);
	blasfeo_dvecpei(n, ipiv, &sz1, 0);
	blasfeo_dvecpei(n, ipiv, &sz2, 0);
	blasfeo_dvecpei(n, ipiv, &sz3, 0);
	blasfeo_dvecpei(n, ipiv, &sz4, 0);
	blasfeo_dvecpei(n, ipiv, &sz5, 0);
	blasfeo_dvecpei(n, ipiv, &sz6, 0);
	blasfeo_dvecpei(n, ipiv, &sz7, 0);
#endif
	//
	blasfeo_print_tran_dvec(n, &sz0, 0);
	blasfeo_print_tran_dvec(n, &sz1, 0);
	blasfeo_print_tran_dvec(n, &sz2, 0);
	blasfeo_print_tran_dvec(n, &sz3, 0);
	blasfeo_print_tran_dvec(n, &sz4, 0);
	blasfeo_print_tran_dvec(n, &sz5, 0);
	blasfeo_print_tran_dvec(n, &sz6, 0);
	blasfeo_print_tran_dvec(n, &sz7, 0);
#endif
	return 0;
//	kernel_dgemm_nt_4x4_gen_lib4(4, &alpha, sA.pA, sB.pA, &beta, 0, sD.pA, sD.cn, 0, sD.pA, sD.cn, 0, 4, 0, 4);
//	kernel_dsyrk_nt_l_4x4_gen_lib4(4, &alpha, sA.pA, sB.pA, &beta, 0, sD.pA, sD.cn, 3, sD.pA, sD.cn, 0, 4, 0, 4);
//	kernel_dtrmm_nn_rl_4x4_gen_lib4(4, &alpha, sB.pA, 3, sA.pA, sB.cn, 0, sD.pA, sD.cn, 0, 4, 0, 4);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	kernel_dgemv_n_4_lib4(4, &alpha, sA.pA, sx0.pa, &beta, sx0.pa, sz0.pa);
//	kernel_dgemv_n_4_vs_lib4(4, &alpha, sA.pA, sx1.pa, &beta, sx0.pa, sz0.pa, 5);
//	kernel_dgemv_t_4_lib4(3, &alpha, sA.pA, sA.cn, sx2.pa, &beta, sx0.pa, sz0.pa);
//	kernel_dgemv_t_4_vs_lib4(3, &alpha, sA.pA, sA.cn, sx2.pa, &beta, sx0.pa, sz0.pa, 3);
//	kernel_dgemv_nt_4_lib4(4, &alpha, &alpha, sA.pA+4, sA.cn, sx0.pa, sx0.pa, &beta, sz0.pa, sz0.pa, sz1.pa);
//	kernel_dsymv_l_4_lib4(4, &alpha, sA.pA+0, sA.cn, sx3.pa, sz0.pa);
//	blasfeo_print_tran_dvec(n, &sz0, 0);
//	blasfeo_print_tran_dvec(n, &sz1, 0);
	return 0;
	blasfeo_dtrmm_rlnn(8, 8, alpha, &sA, 3, 0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dgemm_nn(8, 8, 8, alpha, &sB, 0, 0, &sA, 1, 0, beta, &sA, 0, 0, &sD, 0, 0);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;
//	blasfeo_dsyrk_ln(n, 15, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dpotrf_l_mn(n, 15, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_dpotrf_ln(n, 15, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dtrmm_rlnn(n, n, alpha, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dgese(n, n, 0.0/0.0, &sD, 0, 0);
//	kernel_dgemm_nt_4x8_lib4(n, &alpha, sA.pA, sB.pA, sB.cn, &beta, sC.pA, sD.pA);
//	kernel_dgemm_nn_4x8_lib4(n, &alpha, sA.pA, 0, sB.pA, sB.cn, &beta, sC.pA, sD.pA);
//	kernel_dsyrk_nt_l_4x4_gen_lib4(n, &alpha, sA.pA, sB.pA, &beta, 0, sC.pA, sC.cn, 3, sD.pA, sD.cn, 0, 4, 0, 4);
//	kernel_dsyrk_nt_l_8x4_gen_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, &beta, 0, sC.pA, sC.cn, 3, sD.pA, sD.cn, 0, 8, 0, 8);
//	blasfeo_dsyrk_ln(10, 10, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sC, 0, 0, &sD, 1, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx0, 0, beta, &sz0, 0, &sz0, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx1, 0, beta, &sz1, 0, &sz1, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx2, 0, beta, &sz2, 0, &sz2, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx3, 0, beta, &sz3, 0, &sz3, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx4, 0, beta, &sz4, 0, &sz4, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx5, 0, beta, &sz5, 0, &sz5, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx6, 0, beta, &sz6, 0, &sz6, 0);
	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx7, 0, beta, &sz7, 0, &sz7, 0);
//	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx8, 0, beta, &sz8, 0, &sz8, 0);
//	blasfeo_dsymv_l(10, alpha, &sA, 0, 0, &sx9, 0, beta, &sz9, 0, &sz9, 0);
	blasfeo_print_tran_dvec(n, &sz0, 0);
	blasfeo_print_tran_dvec(n, &sz1, 0);
	blasfeo_print_tran_dvec(n, &sz2, 0);
	blasfeo_print_tran_dvec(n, &sz3, 0);
	blasfeo_print_tran_dvec(n, &sz4, 0);
	blasfeo_print_tran_dvec(n, &sz5, 0);
	blasfeo_print_tran_dvec(n, &sz6, 0);
	blasfeo_print_tran_dvec(n, &sz7, 0);
//	blasfeo_print_tran_dvec(n, &sz8, 0);
//	blasfeo_print_tran_dvec(n, &sz9, 0);
	return 0;

//	blasfeo_print_dmat(n, n, &sC, 0, 0);
//	blasfeo_dgese(n, n, 1.0, &sB, 0, 0);
//	kernel_dger4_sub_4_lib4(6, sB.pA, sA.pA, sC.pA);
//	kernel_dger4_sub_4_vs_lib4(6, sB.pA, sA.pA, sC.pA, 1);
	return 0;

//	blasfeo_print_dmat(n, n, &sC, 0, 0);
//	blasfeo_dgese(n, n, 1.0, &sB, 0, 0);
//	kernel_dger4_sub_4_lib4(6, sB.pA, sA.pA, sC.pA);
//	kernel_dger4_sub_4_vs_lib4(6, sB.pA, sA.pA, sC.pA, 1);
//	kernel_dger4_sub_8_lib4(5, sB.pA, sB.cn, sA.pA, sC.pA, sC.cn);
//	kernel_dger4_sub_8_vs_lib4(5, sB.pA, sB.cn, sA.pA, sC.pA, sC.cn, 5);
//	kernel_dger4_sub_12_lib4(5, sB.pA, sB.cn, sA.pA, sC.pA, sC.cn);
//	kernel_dger4_sub_12_vs_lib4(5, sB.pA, sB.cn, sA.pA, sC.pA, sC.cn, 9);
//	kernel_dger4_sub_8c_lib4(9, sB.pA, sA.cn, sA.pA, sC.pA, sC.cn);
//	kernel_dger4_sub_4c_lib4(9, sB.pA, sA.cn, sA.pA, sC.pA, sC.cn);
//	blasfeo_print_dmat(n, n, &sC, 0, 0);
//	return 0;

#if 1
	blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sC, 0, 0);
#else
	blasfeo_dgese(n, n, 0.1, &sC, 0, 0);
	BLASFEO_DMATEL(&sC, 0, 0) = 1.0;
//	BLASFEO_DMATEL(&sC, 0, 1) = 1.0;
	for(ii=1; ii<n-1; ii++)
		{
//		BLASFEO_DMATEL(&sC, ii, ii-1) = 1.0;
		BLASFEO_DMATEL(&sC, ii, ii) = 1.0;
//		BLASFEO_DMATEL(&sC, ii, ii+1) = 1.0;
		}
//	BLASFEO_DMATEL(&sC, n-1, n-2) = 1.0;
	BLASFEO_DMATEL(&sC, n-1, n-1) = 1.0;
#endif
	blasfeo_print_dmat(n, n, &sC, 0, 0);
	blasfeo_dgese(n, n, 0.0/0.0, &sD, 0, 0);
//	blasfeo_print_dmat(n, n, &sA, 0, 0);
//	blasfeo_dgein1(12.0, &sA, 0, 0);
//	BLASFEO_DMATEL(&sA, 0, 0) =   12.0;
//	BLASFEO_DMATEL(&sA, 1, 0) =    6.0;
//	BLASFEO_DMATEL(&sA, 2, 0) = -  4.0;
//	BLASFEO_DMATEL(&sA, 0, 1) = - 51.0;
//	BLASFEO_DMATEL(&sA, 1, 1) =  167.0;
//	BLASFEO_DMATEL(&sA, 2, 1) =   24.0;
//	BLASFEO_DMATEL(&sA, 0, 2) =    4.0;
//	BLASFEO_DMATEL(&sA, 1, 2) = - 68.0;
//	BLASFEO_DMATEL(&sA, 2, 2) = - 41.0;
//	blasfeo_print_dmat(n, n, &sA, 0, 0);
	blasfeo_print_dmat(n, n, &sC, 0, 0);
//	printf("\n%f\n", DGEEL_LIBSTR(&sA, 0, 0));
//	int qr_work_size = blasfeo_dgeqrf_worksize(n, n);
	int qr_work_size = blasfeo_dgelqf_worksize(n, n);
	void *qr_work;
	v_zeros_align(&qr_work, qr_work_size);
//	blasfeo_dgeqrf(10, 10, &sC, 0, 0, &sD, 0, 0, qr_work);
	blasfeo_dgelqf(17, 17, &sC, 0, 0, &sD, 0, 0, qr_work);
//	blasfeo_dgecp(10, 10, &sC, 0, 0, &sD, 0, 0);
//	kernel_dgeqrf_4_lib4(16, 12, sD.pA, sD.cn, sD.dA, qr_work);
//	blasfeo_print_dmat(n, n, &sA, 0, 0);
//	kernel_dgeqrf_vs_lib4(10, 16, 0, sD.pA+0, sD.cn, sD.dA);
//	kernel_dgelqf_vs_lib4(10, 10, 10, 0, sD.pA+0, sD.cn, sD.dA);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	free(qr_work);
	return 0;

//	blasfeo_dveccl_mask(n, &svm, 0, &sv, 0, &svp, 0, &sv, 0, &sm, 0);
//	veccl_libstr(n, &svm, 0, &sv, 0, &svp, 0, &sv, 0);
//	blasfeo_print_tran_dvec(12, &sv, 0);
//	blasfeo_print_tran_dvec(12, &sm, 0);
//	blasfeo_dvecze(n, &sm, 0, &sr, 0, &sr, 0);
//	blasfeo_print_tran_dvec(12, &sr, 0);
//	return 0;

//	blasfeo_print_dmat(n, n, &sA, 0, 0);
//	blasfeo_dtrsv_unn(n, &sA, 1, 0, &sx0, 0, &sz0, 0);
//	blasfeo_print_tran_dvec(n, &sz0, 0);
//	blasfeo_dtrsv_unn(n, &sA, 1, 0, &sx1, 0, &sz1, 0);
//	blasfeo_print_tran_dvec(n, &sz1, 0);
//	blasfeo_dtrsv_unn(n, &sA, 1, 0, &sx2, 0, &sz2, 0);
//	blasfeo_print_tran_dvec(n, &sz2, 0);
//	blasfeo_dtrsv_unn(n, &sA, 1, 0, &sx3, 0, &sz3, 0);
//	blasfeo_print_tran_dvec(n, &sz3, 0);
//	return 0;

//	double alpha = 1.0;
//	double beta = 1.0;
//	kernel_dgemm_nt_4x12_vs_lib4(n, &alpha, sA.pA, sB.pA, sB.cn, &beta, sD.pA, sD.pA, 3, 10);
//	kernel_dgemm_nt_8x8u_vs_lib4(n, &alpha, sA.pA, sA.cn, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn, 7, 6);
	blasfeo_dgemm_nn(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_dpotrf_l(16, &sD, 0, 0, &sD, 0, 0);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;;

//	dmatse_libstr(n, n, 100.0, &sD, 0, 0);

//	for(ii=0; ii<n; ii++)
//		blasfeo_dvecin1(ii+1, &sx_n, ii);
//	blasfeo_print_tran_dvec(n, &sx_n, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	// blasfeo_ddiain(4, -1.0, &sx_n, 1, &sD, 3, 2);
//	blasfeo_ddiaad(4, -1.0, &sx_n, 1, &sD, 3, 2);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	return 0;

//	blasfeo_print_tran_dvec(n, &sx_n, 0);
//	blasfeo_dgemm_dn(n, n, 1.0, &sx_n, 0, &sA, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dgemm_nd(n, n, 1.0, &sA, 0, 0, &sx_n, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	exit(1);

//	dsetmat_libstr(n, n, 0.0, &sD, 0, 0);
//	dmatin1_libstr(2.0, &sD, 0, 0);
//	dmatin1_libstr(2.0, &sD, 1, 1);
//	dmatin1_libstr(2.0, &sD, 2, 2);
//	dmatin1_libstr(1.0, &sD, 1, 0);
//	dmatin1_libstr(1.0, &sD, 2, 1);
//	dmatin1_libstr(0.5, &sD, 2, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	blasfeo_print_tran_dvec(n, &sx_n, 0);
//	blasfeo_dtrsv_lnn(n, n, &sD, 0, 0, &sx_n, 0, &sz_n, 0);
//	blasfeo_print_tran_dvec(n, &sz_n, 0);
//	exit(1);

//	blasfeo_dgemm_nt(8, 8, 8, 1.0, &sB, 0, 0, &sA, 1, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	return 0;

//	double alpha = 1.0;
//	kernel_dtrmm_nn_rl_4x4_gen_lib4(7, &alpha, sB.pA, 2, sA.pA, sA.cn, 1, sD.pA, sD.cn, 0, 4, 1, 4);
//	kernel_dtrmm_nn_rl_4x4_gen_lib4(7, &alpha, sB.pA+sB.cn*4, 2, sA.pA, sA.cn, 1, sD.pA+sD.cn*4, sD.cn, 0, 4, 1, 4);
//	kernel_dtrmm_nn_rl_4x4_lib4(4, &alpha, sB.pA, sA.pA, sA.cn+4*4, sD.pA+4*4);
//	kernel_dtrmm_nn_rl_4x4_gen_lib4(3, &alpha, sB.pA+sB.cn*4+4*4, 2, sA.pA+sB.cn*4+4*4, sA.cn, 1, sD.pA+sD.cn*4+4*4, sD.cn, 0, 4, 0, 4);
	blasfeo_dtrmm_rlnn(8, 8, 1.0, &sB, 0, 0, &sA, 3, 0, &sD, 2, 1);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	return 0;

	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx0, 0, &sx0, 0);
	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx1, 0, &sx1, 0);
	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx2, 0, &sx2, 0);
	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx3, 0, &sx3, 0);
	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx4, 0, &sx4, 0);
	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx5, 0, &sx5, 0);
	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx6, 0, &sx6, 0);
	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx7, 0, &sx7, 0);
//	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx8, 0, &sx8, 0);
//	blasfeo_dtrmv_lnn(8, &sA, 0, 0, &sx9, 0, &sx9, 0);
	blasfeo_print_tran_dvec(n, &sx0, 0);
	blasfeo_print_tran_dvec(n, &sx1, 0);
	blasfeo_print_tran_dvec(n, &sx2, 0);
	blasfeo_print_tran_dvec(n, &sx3, 0);
	blasfeo_print_tran_dvec(n, &sx4, 0);
	blasfeo_print_tran_dvec(n, &sx5, 0);
	blasfeo_print_tran_dvec(n, &sx6, 0);
	blasfeo_print_tran_dvec(n, &sx7, 0);
//	blasfeo_print_tran_dvec(n, &sx8, 0);
//	blasfeo_print_tran_dvec(n, &sx9, 0);
	return 0;

	blasfeo_dgemv_t(2, 8, 1.0, &sA, 2, 0, &sx_n, 0, 0.0, &sy_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;

	blasfeo_dgemm_nt(4, 8, 8, 1.0, &sB, 0, 0, &sA, 0, 0, 0.0, &sB, 0, 0, &sD, 3, 0);
//	blasfeo_print_dmat(n, n, &sB, 0, 0);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	exit(1);

	blasfeo_dpotrf_l(n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dgetrf_nopivot(n, n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dgetrf_rowpivot(n, n, &sD, 0, 0, &sD, 0, 0, ipiv);
	blasfeo_print_dmat(n, n, &sD, 0, 0);
#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
	d_print_mat(1, n, sD.dA, 1);
#endif
	int_print_mat(1, n, ipiv, 1);
	blasfeo_dtrsm_rltn(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sE, 0, 0);
	blasfeo_print_dmat(n, n, &sE, 0, 0);
	exit(1);

#if 1 // solve P L U X = P B
	blasfeo_print_dmat(n, n, &sB, 0, 0);
	blasfeo_drowpe(n, ipiv, &sB);
	blasfeo_print_dmat(n, n, &sB, 0, 0);

	blasfeo_dtrsm_llnu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sE, 0, 0);
	blasfeo_print_dmat(n, n, &sE, 0, 0);
	blasfeo_dtrsm_lunn(n, n, 1.0, &sD, 0, 0, &sE, 0, 0, &sE, 0, 0);
	blasfeo_print_dmat(n, n, &sE, 0, 0);
#else // solve X^T (P L U)^T = B^T P^T
	blasfeo_print_dmat(n, n, &sB, 0, 0);
	blasfeo_dcolpe(n, ipiv, &sB);
	blasfeo_print_dmat(n, n, &sB, 0, 0);

	blasfeo_dtrsm_rltu(n, n, 1.0, &sD, 0, 0, &sB, 0, 0, &sE, 0, 0);
	blasfeo_print_dmat(n, n, &sE, 0, 0);
	blasfeo_dtrsm_rutn(n, n, 1.0, &sD, 0, 0, &sE, 0, 0, &sE, 0, 0);
	blasfeo_print_dmat(n, n, &sE, 0, 0);
#endif

//	blasfeo_print_dmat(n, n, &sA, 0, 0);
//	blasfeo_print_dmat(n, n, &sB, 0, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	blasfeo_print_dmat(n, n, &sE, 0, 0);

//	blasfeo_unpack_dmat(n, n, &sE, 0, 0, C, n);
//	d_print_mat(n, n, C, n);

	blasfeo_dtrtr_u(6, &sE, 2, 0, &sB, 1, 0);
	blasfeo_print_dmat(n, n, &sB, 0, 0);

	blasfeo_print_dmat(n, n, &sA, 0, 0);
	blasfeo_dgemv_nt(6, n, 1.0, 1.0, &sA, 0, 0, &sx_n, 0, &sx_t, 0, 0.0, 0.0, &sy_n, 0, &sy_t, 0, &sz_n, 0, &sz_t, 0);
//	blasfeo_dsymv_l(5, 1.0, &sA, 0, 0, x_n, 0.0, y_n, z_n);
	d_print_mat(1, n, z_n, 1);
	d_print_mat(1, n, z_t, 1);




//	for(ii=0; ii<sE.pm*sE.cn; ii++) sE.pA[ii] = 0.0;
//	double alpha = 0.0;
//	double beta = 1.0;
//	kernel_dgemm_nt_4x4_gen_lib4(4, &alpha, sA.pA, sB.pA, &beta, 3, sA.pA, sA.cn, 0, sE.pA, sE.cn, 0, 4, 2, 2);
//	blasfeo_print_dmat(n, n, &sE, 0, 0);

	// free memory
	free(A);
	free(B);
	free(C);
	free(D);
	free(ipiv);
//	blasfeo_free_dmat(&sA);
//	blasfeo_free_dmat(&sB);
//	blasfeo_free_dmat(&sD);
	v_free_align(memory_strmat);
//	blasfeo_free_dmat(&lq0);
//	free(lq0_work);

	return 0;

//	print_compilation_flags();
	}
