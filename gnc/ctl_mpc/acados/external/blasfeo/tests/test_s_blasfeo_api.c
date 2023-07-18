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

#include "../include/blasfeo_common.h"
#include "../include/blasfeo_i_aux_ext_dep.h"
#include "../include/blasfeo_s_aux_ext_dep.h"
#include "../include/blasfeo_s_aux.h"
#include "../include/blasfeo_s_kernel.h"
#include "../include/blasfeo_s_blas.h"

//#include "test_s_common.h"
//#include "test_x_common.c"

int main()
	{
//	print_compilation_flags();

	int ii, jj;

	int n = 16;

	//
	// matrices in column-major format
	//
	float *A; s_zeros(&A, n, n);
	for(ii=0; ii<n*n; ii++) A[ii] = ii;

//	for(jj=0; jj<n; jj++)
//		for(ii=0; ii<jj; ii++)
//			A[ii+n*jj] = 0.0/0.0;
//	s_print_mat(n, n, A, n);

	float *B; s_zeros(&B, n, n);
	for(ii=0; ii<n; ii++) B[ii*(n+1)] = 1.0;
//	s_print_mat(n, n, B, n);

	float *D; s_zeros(&D, n, n);
	for(ii=0; ii<n*n; ii++) D[ii] = -1.0;
//	s_print_mat(n, n, B, n);


	//
	// matrices in matrix struct format
	//

	struct blasfeo_smat sA;
	blasfeo_allocate_smat(n, n, &sA);
	blasfeo_pack_smat(n, n, A, n, &sA, 0, 0);
	blasfeo_print_smat(n, n, &sA, 0, 0);

	struct blasfeo_smat sB;
	blasfeo_allocate_smat(n, n, &sB);
	blasfeo_pack_smat(n, n, B, n, &sB, 0, 0);
	blasfeo_print_smat(n, n, &sB, 0, 0);

	struct blasfeo_smat sD;
	blasfeo_allocate_smat(n, n, &sD);
	blasfeo_pack_smat(n, n, D, n, &sD, 0, 0);

	struct blasfeo_svec sx;
	blasfeo_allocate_svec(n, &sx);
	sx.pa[2] = 1.0;
	blasfeo_print_tran_svec(n, &sx, 0);

	struct blasfeo_svec sz0;
	blasfeo_allocate_svec(n, &sz0);

	struct blasfeo_svec sz1;
	blasfeo_allocate_svec(n, &sz1);

	float alpha, beta;

	//
	// tests
	//

#if 0
	// gemm_nt
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_smat(n, n, &sD, 0, 0);

//	kernel_sgemm_nt_4x4_lib4(4, &alpha, sA.pA+4*sA.cn, sB.pA, &beta, sA.pA, sD.pA);
//	kernel_sgemm_nt_8x4_lib4(4, &alpha, sA.pA+0*sA.cn, sA.cn, sB.pA, &beta, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_sgemm_nt_8x4_vs_lib4(4, &alpha, sA.pA+0*sA.cn, sA.cn, sB.pA, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 7, 3);
//	kernel_sgemm_nt_8x8_lib4(8, &alpha, sA.pA, sA.cn, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn);

	blasfeo_sgemm_nt(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sD, 0, 0, &sD, 0, 0);

	blasfeo_print_smat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 1
	// gemm_nn
	alpha = 1.0;
	beta = 0.0;
	blasfeo_print_smat(n, n, &sD, 0, 0);

//	kernel_sgemm_nn_4x4_lib4(8, &alpha, sA.pA+4*sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sD.pA);
//	kernel_sgemm_nn_8x4_lib4(8, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_sgemm_nn_8x8_lib4(8, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn);
//	kernel_sgemm_nn_8x8_vs_lib4(8, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sA.cn, sD.pA, sD.cn, 7, 5);

	blasfeo_sgemm_nn(1, 5, 2, alpha, &sB, 0, 0, &sA, 1, 0, beta, &sD, 0, 0, &sD, 0, 0);

	blasfeo_print_smat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// trmm_rutn
	alpha = -1.0;
	beta = 0.0;
	blasfeo_print_smat(n, n, &sD, 0, 0);

//	kernel_sgemm_nn_4x4_lib4(8, &alpha, sA.pA+4*sA.cn, 0, sB.pA, sB.cn, &beta, sA.pA, sD.pA);

	blasfeo_strmm_rutn(n, n, alpha, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);

	blasfeo_print_smat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
	// potrf
	alpha = 1.0;
	beta = 1.0;
	blasfeo_print_smat(n, n, &sD, 0, 0);
//	blasfeo_dgemm_nt(n, n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
	blasfeo_ssyrk_ln(n, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_ln_mn(n, n-1, n, alpha, &sA, 0, 0, &sA, 0, 0, beta, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_print_dmat(n, n, &sD, 0, 0);
//	blasfeo_spotrf_l(n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dtrsm_rltn(7, 4, 1.0, &sD, 0, 0, &sD, 4, 0, &sD, 4, 0);
	blasfeo_spotrf_l_mn(n, n, &sD, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_dpotrf_ln(n, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_dsyrk_dpotrf_ln_mn(n-1, n-3, n, &sA, 0, 0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_print_smat(n, n, &sD, 0, 0);
	return 0;
#endif

#if 0
//	blasfeo_sgemv_n(n, n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sz0, 0, &sz0, 0);
	blasfeo_sgemv_n(3, n, 1.0, &sA, 1, 0, &sx, 0, 0.0, &sz0, 0, &sz0, 0);
	blasfeo_print_tran_svec(n, &sz0, 0);
	return 0;
#endif

#if 0
//	blasfeo_sgemv_n(n, n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sz0, 0, &sz0, 0);
	blasfeo_sgemv_t(n, n, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sz0, 0, &sz0, 0);
	blasfeo_print_tran_svec(n, &sz0, 0);
	return 0;
#endif

#if 1
	// trsv_lnn
	blasfeo_print_tran_svec(n, &sz0, 0);
	sA.pA[0+4*0] = 4;
	sA.pA[1+4*0] = 3;
	sA.pA[2+4*0] = 2;
	sA.pA[3+4*0] = 1;
	sA.pA[1+4*1] = 4;
	sA.pA[2+4*1] = 3;
	sA.pA[3+4*1] = 2;
	sA.pA[2+4*2] = 4;
	sA.pA[3+4*2] = 3;
	sA.pA[3+4*3] = 4;
	blasfeo_print_smat(n, n, &sA, 0, 0);
	blasfeo_strsv_lnn(4, &sA, 0, 0, &sx, 0, &sz0, 0);
//	blasfeo_strsv_lnn_mn(4, 2, &sA, 0, 0, &sx0, 0, &sz0, 0);
	blasfeo_print_tran_svec(n, &sz0, 0);
	return 0;
#endif

#if 0
	// trsv_ltn
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	sA.pA[0+4*0] = 4;
	sA.pA[1+4*0] = 3;
	sA.pA[2+4*0] = 2;
	sA.pA[3+4*0] = 1;
	sA.pA[1+4*1] = 4;
	sA.pA[2+4*1] = 3;
	sA.pA[3+4*1] = 2;
	sA.pA[2+4*2] = 4;
	sA.pA[3+4*2] = 3;
	sA.pA[3+4*3] = 4;
	blasfeo_print_dmat(n, n, &sA, 0, 0);
	blasfeo_dtrsv_ltn(4, &sA, 0, 0, &sx_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;
#endif

#if 0
	// symv_l
	blasfeo_print_tran_dvec(n, &sx_n, 0);
	blasfeo_dsymv_l(2, 2, 1.0, &sA, 1, 1, &sx_n, 0, 0.0, &sy_n, 0, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	return 0;
#endif

#if 0
	// gemv_nt
	blasfeo_print_tran_dvec(n, &sx_n, 0);
	blasfeo_dgemv_nt(6, 6, 1.0, 1.0, &sA, 1, 0, &sx_n, 0, &sx_t, 0, 0.0, 0.0, &sy_n, 0, &sy_t, 0, &sz_n, 0, &sz_t, 0);
	blasfeo_print_tran_dvec(n, &sz_n, 0);
	blasfeo_print_tran_dvec(n, &sz_t, 0);
	return 0;
#endif




	// copy scale
#if 0
	blasfeo_print_smat(n, n, &sA, 0, 0);
	blasfeo_sgecpsc(10, 10, 0.1, &sA, 0, 0, &sD, 0, 0);
	blasfeo_print_smat(n, n, &sD, 0, 0);
	return 0;
#endif

	alpha = 1.0;
	beta = 0.0;

//	kernel_sgemm_nt_4x4_lib4(n, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//	kernel_sgemm_nn_4x4_lib4(n, &alpha, sA.pA, sB.pA, sB.cn, &beta, sD.pA, sD.pA);
//	blasfeo_sgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
	blasfeo_sgemm_nn(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sD, 0, 0, &sD, 0, 0);
	blasfeo_print_smat(n, n, &sD, 0, 0);
	return 0;

//	kernel_sgemm_nt_24x4_lib8(4, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_sgemm_nt_16x4_lib8(4, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_sgemm_nt_8x8_lib8(5, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//	kernel_sgemm_nt_8x4_lib8(5, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA);
//	kernel_sgemm_nt_4x8_gen_lib8(8, &alpha, sA.pA, sB.pA, &beta, 0, sD.pA, sD.cn, 0, sD.pA, sD.cn, 0, 4, 0, 8);
//	kernel_sgemm_nt_8x4_vs_lib8(8, &alpha, sA.pA, sB.pA, &beta, sD.pA, sD.pA, 7, 4);
//	kernel_sgemm_nt_8x4_lib8(8, &alpha, sB.pA, sA.pA+4, &beta, sA.pA+4*8, sD.pA+4*8);
//	kernel_sgemm_nn_16x4_lib8(4, &alpha, sA.pA, sA.cn, 0, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_sgemm_nt_12x4_lib4(4, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_sgemm_nt_8x8_lib4(8, &alpha, sA.pA, sA.cn, sB.pA, sB.cn, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	kernel_sgemm_nt_8x4_lib4(2, &alpha, sA.pA, sA.cn, sB.pA, &beta, sD.pA, sD.cn, sD.pA, sD.cn);
//	blasfeo_print_smat(n, n, &sD, 0, 0);
//	return 0;
	blasfeo_sgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_ssyrk_ln(n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sB, 0, 0, &sD, 0, 0);
//	blasfeo_ssyrk_ln_mn(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sB, 0, 0, &sD, 0, 0);
//	kernel_ssyrk_nt_l_8x8_lib8(n, &alpha, sA.pA, sA.pA, &beta, sB.pA, sD.pA);
//	blasfeo_sgecp(16, 16, &sA, 2, 0, &sD, 1, 0);
//	blasfeo_sgetr(16, 16, &sA, 2, 0, &sD, 2, 0);
//	blasfeo_print_smat(n, n, &sD, 0, 0);
//	blasfeo_sgemv_n(6, 6, 1.0, &sA, 1, 0, &sx, 0, 0.0, &sz0, 0, &sz0, 0);
//	blasfeo_sgemv_t(11, 8, 1.0, &sA, 0, 0, &sx, 0, 0.0, &sz0, 0, &sz0, 0);
//	blasfeo_strmv_lnn(6, &sA, 1, 0, &sx, 0, &sz0, 0);
//	blasfeo_strmv_ltn(10, &sA, 1, 0, &sx, 0, &sz0, 0);
//	sA.pA[0] = 1.0;
//	blasfeo_strsv_lnn(10, &sA, 0, 0, &sx, 0, &sz0, 0);
//	for(ii=0; ii<8; ii++) sA.dA[ii] = 1.0/blasfeo_sgeex1(&sA, ii, ii);
//	kernel_strsv_lt_inv_8_lib8(0, sA.pA, sA.cn, sA.dA, sx.pa, sx.pa, sz0.pa);
//	kernel_strsv_lt_inv_8_vs_lib8(0, sA.pA, sA.cn, sA.dA, sx.pa, sx.pa, sz0.pa, 3);
//	blasfeo_print_smat(n, n, &sA, 0, 0);
//	blasfeo_strsv_ltn(12, &sA, 0, 0, &sx, 0, &sz0, 0);
//	blasfeo_strsv_ltn_mn(11, 3, &sA, 0, 0, &sx, 0, &sz0, 0);
//	blasfeo_print_smat(n, n, &sA, 0, 0);
//	kernel_sgemv_nt_4_lib8(n, &alpha, &alpha, sA.pA, sA.cn, sx.pa, sx.pa, &beta, sz1.pa, sz0.pa, sz1.pa);
//	kernel_sgemv_nt_4_vs_lib8(n, &alpha, &alpha, sA.pA, sA.cn, sx.pa, sx.pa, &beta, sz1.pa, sz0.pa, sz1.pa, 3);
//	blasfeo_sgemv_nt(5, 2, alpha, alpha, &sA, 0, 0, &sx, 0, &sx, 0, beta, beta, &sz0, 0, &sz1, 0, &sz0, 0, &sz1, 0);
//	blasfeo_ssymv_l(10, 10, alpha, &sA, 1, 0, &sx, 0, beta, &sz0, 0, &sz1, 0);
//	blasfeo_print_tran_svec(n, &sz0, 0);
//	blasfeo_print_tran_svec(n, &sz1, 0);
//	return 0;
//	blasfeo_sgesc(16, 9, 2.0, &sD, 0, 0);
//	blasfeo_print_smat(n, n, &sD, 0, 0);
//	kernel_spotrf_nt_l_8x8_lib8(0, sD.pA, sD.pA, sD.pA, sD.pA, sx.pa);
//	blasfeo_print_smat(n, n, &sD, 0, 0);
//	blasfeo_print_tran_svec(n, &sx, 0);
//	kernel_strsm_nt_rl_inv_8x8_lib8(0, sD.pA, sD.pA, sD.pA+8*sD.cn, sD.pA+8*sD.cn, sD.pA, sx.pa);
//	blasfeo_print_smat(n, n, &sD, 0, 0);
//	kernel_spotrf_nt_l_8x8_lib8(8, sD.pA+8*sD.cn, sD.pA+8*sD.cn, sD.pA+8*sD.cn+8*8, sD.pA+8*sD.cn+8*8, sx.pa+8);
//	blasfeo_spotrf_l_mn(23, 17, &sD, 0, 0, &sD, 0, 0);
	blasfeo_spotrf_l(n, &sD, 0, 0, &sD, 0, 0);
//	kernel_strmm_nn_rl_8x4_lib8(3, &alpha, sB.pA, 7, sA.pA, sA.cn, sD.pA);
//	blasfeo_strmm_rlnn(16, 12, 1.0, &sA, 0, 0, &sB, 0, 0, &sD, 0, 0);
	blasfeo_print_smat(n, n, &sD, 0, 0);
	return 0;



	//
	// free memory
	//

	free(A);
	free(B);
	free(D);
	blasfeo_free_smat(&sA);
	blasfeo_free_smat(&sB);
	blasfeo_free_smat(&sD);

	return 0;

//	print_compilation_flags();
	}
