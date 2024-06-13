/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
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
#include <sys/time.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_s_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_s_blas.h>

#include "../include/hpipm_s_dense_qp.h"
#include "../include/hpipm_s_dense_qp_sol.h"
#include "../include/hpipm_s_dense_qp_ipm_hard.h"



#define PRINT 1



int main()
	{

	int ii;

/************************************************
* qp dimension and data
************************************************/	

	int nv = 2;
	int ne = 1;
	int nb = 2;
	int ng = 0;

	float H[] = {4.0, 1.0, 1.0, 2.0};
	float g[] = {1.0, 1.0};
	float A[] = {1.0, 1.0};
	float b[] = {1.0};
//	float d_lb[] = {0.0, 0.0};
//	float d_ub[] = {INFINITY, INFINITY};
	float d_lb[] = {-1.0, -1.0};
	float d_ub[] = {1.5, 0.5};
	int idxb[] = {0, 1};
	float C[] = {1.0, 0.0, 0.0, 1.0};
	float d_lg[] = {-1.0, -1.0};
	float d_ug[] = {1.5, 0.5};

/************************************************
* dense qp
************************************************/	

	hpipm_size_t qp_size = s_memsize_dense_qp(nv, ne, nb, ng);
	printf("\nqp size = %d\n", qp_size);
	void *qp_mem = malloc(qp_size);

	struct s_dense_qp qp;
	s_create_dense_qp(nv, ne, nb, ng, &qp, qp_mem);
	s_cvt_colmaj_to_dense_qp(H, g, A, b, idxb, d_lb, d_ub, C, d_lg, d_ug, &qp);

	blasfeo_print_smat(nv+1, nv, qp.Hg, 0, 0);
	blasfeo_print_smat(ne, nv, qp.A, 0, 0);
	blasfeo_print_smat(nv, ng, qp.Ct, 0, 0);
	blasfeo_print_svec(nv, qp.g, 0);
	blasfeo_print_svec(ne, qp.b, 0);
	blasfeo_print_svec(2*nb+2*ng, qp.d, 0);
	blasfeo_print_svec(nb, qp.d_lb, 0);
	blasfeo_print_svec(nb, qp.d_ub, 0);
	blasfeo_print_svec(ng, qp.d_lg, 0);
	blasfeo_print_svec(ng, qp.d_ug, 0);

/************************************************
* dense qp sol
************************************************/	

	hpipm_size_t qp_sol_size = s_memsize_dense_qp_sol(nv, ne, nb, ng);
	printf("\nqp sol size = %d\n", qp_sol_size);
	void *qp_sol_mem = malloc(qp_sol_size);

	struct s_dense_qp_sol qp_sol;
	s_create_dense_qp_sol(nv, ne, nb, ng, &qp_sol, qp_sol_mem);

/************************************************
* ipm
************************************************/	

	struct s_ipm_hard_dense_qp_arg arg;
	arg.alpha_min = 1e-8;
	arg.mu_max = 1e-12;
	arg.iter_max = 20;
	arg.mu0 = 1.0;

	hpipm_size_t ipm_size = s_memsize_ipm_hard_dense_qp(&qp, &arg);
	printf("\nipm size = %d\n", ipm_size);
	void *ipm_mem = malloc(ipm_size);

	struct s_ipm_hard_dense_qp_workspace workspace;
	s_create_ipm_hard_dense_qp(&qp, &arg, &workspace, ipm_mem);

	int rep, nrep=1000;

	struct timeval tv0, tv1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
//		s_solve_ipm_hard_dense_qp(&qp, &qp_sol, &workspace);
		s_solve_ipm2_hard_dense_qp(&qp, &qp_sol, &workspace);
		}

	gettimeofday(&tv1, NULL); // stop

	float time_dense_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

	printf("\nsolution\n\n");
	printf("\nv\n");
	blasfeo_print_tran_svec(nv, qp_sol.v, 0);
	printf("\npi\n");
	blasfeo_print_tran_svec(ne, qp_sol.pi, 0);
	printf("\nlam_lb\n");
	blasfeo_print_tran_svec(nb, qp_sol.lam_lb, 0);
	printf("\nlam_ub\n");
	blasfeo_print_tran_svec(nb, qp_sol.lam_ub, 0);
	printf("\nlam_lg\n");
	blasfeo_print_tran_svec(ng, qp_sol.lam_lg, 0);
	printf("\nlam_ug\n");
	blasfeo_print_tran_svec(ng, qp_sol.lam_ug, 0);
	printf("\nt_lb\n");
	blasfeo_print_tran_svec(nb, qp_sol.t_lb, 0);
	printf("\nt_ub\n");
	blasfeo_print_tran_svec(nb, qp_sol.t_ub, 0);
	printf("\nt_lg\n");
	blasfeo_print_tran_svec(ng, qp_sol.t_lg, 0);
	printf("\nt_ug\n");
	blasfeo_print_tran_svec(ng, qp_sol.t_ug, 0);

	printf("\nresiduals\n\n");
	printf("\nres_g\n");
	blasfeo_print_exp_tran_svec(nv, workspace.res_g, 0);
	printf("\nres_b\n");
	blasfeo_print_exp_tran_svec(ne, workspace.res_b, 0);
	printf("\nres_d\n");
	blasfeo_print_exp_tran_svec(2*nb+2*ng, workspace.res_d, 0);
	printf("\nres_m\n");
	blasfeo_print_exp_tran_svec(2*nb+2*ng, workspace.res_m, 0);
	printf("\nres_mu\n");
	printf("\n%e\n\n", workspace.res_mu);

	printf("\nipm iter = %d\n", workspace.iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
	s_print_exp_tran_mat(5, workspace.iter, workspace.stat, 5);

	printf("\ndense ipm time = %e [s]\n\n", time_dense_ipm);

/************************************************
* free memory
************************************************/	

	free(qp_mem);
	free(qp_sol_mem);
	free(ipm_mem);

/************************************************
* return
************************************************/	

	return 0;

	}

	

