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
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include <hpipm_d_dense_qp_utils.h>
#include <hpipm_d_dense_qcqp.h>
#include <hpipm_d_dense_qcqp_sol.h>
#include <hpipm_d_dense_qcqp_res.h>
#include <hpipm_d_dense_qcqp_ipm.h>
#include <hpipm_d_dense_qcqp_utils.h>



// printing
#ifndef PRINT
#define PRINT 1
#endif



int main()
	{

	int ii;

/************************************************
* qp dimension and data
************************************************/

	int nv = 2;
	int ne = 0;
	int nb = 1;
	int ng = 0;
	int nq = 1;
//	int ns = 1;
	int nsb = 0;
	int nsg = 0;
	int nsq = 1;

	double H[] = {1.0, 0.0, 0.0, 1.0};
	double g[] = {2.0, 2.0};
	double Hq[] = {2.0, 0.0, 0.0, 2.0};
	double gq[] = {0.0, 0.0};
	double uq[] = {1.0};
	int idxb[] = {1};
	double lb[] = {-0.5};
	double ub[] = {0.5};
	double C[] = {0.0, 1.0};
	double lg[] = {-0.5};
	double ug[] = {0.5};
	double Zl[] = {0e0};
	double Zu[] = {0e0};
	double zl[] = {0e0};
	double zu[] = {1e0};
	int idxs[] = {1};

/************************************************
* dense qp dim
************************************************/

	hpipm_size_t dense_qcqp_dim_size = d_dense_qcqp_dim_memsize();
#if PRINT
	printf("\nqp dim size = %d\n", dense_qcqp_dim_size);
#endif
	void *qp_dim_mem = malloc(dense_qcqp_dim_size);

	struct d_dense_qcqp_dim qcqp_dim;
	d_dense_qcqp_dim_create(&qcqp_dim, qp_dim_mem);

	d_dense_qcqp_dim_set_nv(nv, &qcqp_dim);
	d_dense_qcqp_dim_set_nb(nb, &qcqp_dim);
	d_dense_qcqp_dim_set_ng(ng, &qcqp_dim);
	d_dense_qcqp_dim_set_nq(nq, &qcqp_dim);
	d_dense_qcqp_dim_set_nsq(nsq, &qcqp_dim);

#if PRINT
	printf("\nqcqp dim\n");
	d_dense_qcqp_dim_print(&qcqp_dim);
	printf("\nqp dim\n");
	d_dense_qp_dim_print(qcqp_dim.qp_dim);
#endif

/************************************************
* dense qp
************************************************/

	hpipm_size_t qp_size = d_dense_qcqp_memsize(&qcqp_dim);
#if PRINT
	printf("\nqp size = %d\n", qp_size);
#endif
	void *qp_mem = malloc(qp_size);

	struct d_dense_qcqp qcqp;
	d_dense_qcqp_create(&qcqp_dim, &qcqp, qp_mem);

	// test setters

	d_dense_qcqp_set_H(H, &qcqp);
	d_dense_qcqp_set_g(g, &qcqp);
	d_dense_qcqp_set_Hq(Hq, &qcqp);
	d_dense_qcqp_set_gq(gq, &qcqp);
	d_dense_qcqp_set_uq(uq, &qcqp);
	d_dense_qcqp_set_idxb(idxb, &qcqp);
	d_dense_qcqp_set_lb(lb, &qcqp);
	d_dense_qcqp_set_ub(ub, &qcqp);
	d_dense_qcqp_set_C(C, &qcqp);
	d_dense_qcqp_set_lg(lg, &qcqp);
	d_dense_qcqp_set_ug(ug, &qcqp);
	d_dense_qcqp_set_Zl(Zl, &qcqp);
	d_dense_qcqp_set_Zu(Zu, &qcqp);
	d_dense_qcqp_set_zl(zl, &qcqp);
	d_dense_qcqp_set_zu(zu, &qcqp);
	d_dense_qcqp_set_idxs(idxs, &qcqp);

#if PRINT
	printf("\nqcqp\n");
	d_dense_qcqp_print(&qcqp_dim, &qcqp);
//	exit(1);
#endif

/************************************************
* dense qp sol
************************************************/

	hpipm_size_t qp_sol_size = d_dense_qcqp_sol_memsize(&qcqp_dim);
#if PRINT
	printf("\nqp sol size = %d\n", qp_sol_size);
#endif
	void *qp_sol_mem = malloc(qp_sol_size);

	struct d_dense_qcqp_sol qcqp_sol;
	d_dense_qcqp_sol_create(&qcqp_dim, &qcqp_sol, qp_sol_mem);

#if PRINT
	printf("\nqcqp_sol\n");
	d_dense_qcqp_sol_print(&qcqp_dim, &qcqp_sol);
#endif

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_dense_qcqp_ipm_arg_memsize(&qcqp_dim);
#if PRINT
	printf("\nipm arg size = %d\n", ipm_arg_size);
#endif
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_dense_qcqp_ipm_arg arg;
	d_dense_qcqp_ipm_arg_create(&qcqp_dim, &arg, ipm_arg_mem);
//	enum hpipm_mode mode = SPEED_ABS;
	enum hpipm_mode mode = SPEED;
//	enum hpipm_mode mode = BALANCE;
//	enum hpipm_mode mode = ROBUST;
	d_dense_qcqp_ipm_arg_set_default(mode, &arg);

	int iter_max = 20; //25;
	double mu0 = 1e1;
	int comp_res_exit = 1;
	double tol_stat = 1e-12;
	double tol_eq = 1e-12;
	double tol_ineq = 1e-12;
	double tol_comp = 1e-12;

	d_dense_qcqp_ipm_arg_set_iter_max(&iter_max, &arg);
	d_dense_qcqp_ipm_arg_set_mu0(&mu0, &arg);
	d_dense_qcqp_ipm_arg_set_comp_res_exit(&comp_res_exit, &arg);
	d_dense_qcqp_ipm_arg_set_tol_stat(&tol_stat, &arg);
	d_dense_qcqp_ipm_arg_set_tol_eq(&tol_eq, &arg);
	d_dense_qcqp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
	d_dense_qcqp_ipm_arg_set_tol_comp(&tol_comp, &arg);

//	int t_lam_min = 2;
//	d_dense_qcqp_ipm_arg_set_t_lam_min(&t_lam_min, &arg);

//	arg.alpha_min = 1e-8;
//	arg.res_g_max = 1e-8;
//	arg.res_b_max = 1e-8;
//	arg.res_d_max = 1e-12;
//	arg.res_m_max = 1e-12;
//	arg.mu0 = 10.0;
//	arg.iter_max = 10;
//	arg.stat_max = 10;
//	arg.pred_corr = 1;
//	arg.scale = 1;

/************************************************
* ipm
************************************************/

	hpipm_size_t ipm_size = d_dense_qcqp_ipm_ws_memsize(&qcqp_dim, &arg);
#if PRINT
	printf("\nipm size = %d\n", ipm_size);
#endif
	void *ipm_mem = malloc(ipm_size);

	struct d_dense_qcqp_ipm_ws workspace;
	d_dense_qcqp_ipm_ws_create(&qcqp_dim, &arg, &workspace, ipm_mem);

	int hpipm_return; // 0 normal; 1 max iter

	int rep, nrep=1000;

#if PRINT
	printf("\nsolving ...\n");
#endif
	struct timeval tv0, tv1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_dense_qcqp_ipm_solve(&qcqp, &qcqp_sol, &arg, &workspace);
		d_dense_qcqp_ipm_get_status(&workspace, &hpipm_return);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_dense_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	printf("\ndone!\n");


//	printf("\nqp\n");
//	d_dense_qp_print(qcqp_dim.qp_dim, workspace.qp);

	printf("\nqcqp sol\n");
	d_dense_qcqp_sol_print(&qcqp_dim, &qcqp_sol);
#endif

/************************************************
* print ipm statistics
************************************************/

	int iter; d_dense_qcqp_ipm_get_iter(&workspace, &iter);
	double max_res_stat; d_dense_qcqp_ipm_get_max_res_stat(&workspace, &max_res_stat);
	double max_res_eq  ; d_dense_qcqp_ipm_get_max_res_eq(&workspace, &max_res_eq);
	double max_res_ineq; d_dense_qcqp_ipm_get_max_res_ineq(&workspace, &max_res_ineq);
	double max_res_comp; d_dense_qcqp_ipm_get_max_res_comp(&workspace, &max_res_comp);
	double *stat; d_dense_qcqp_ipm_get_stat(&workspace, &stat);
	int stat_m;  d_dense_qcqp_ipm_get_stat_m(&workspace, &stat_m);

#if PRINT
	printf("\nipm return = %d\n", hpipm_return);
	printf("\nipm iter = %d\n", iter);
	printf("\nipm max res: stat = %e, eq =  %e, ineq =  %e, comp = %e\n", max_res_stat, max_res_eq, max_res_ineq, max_res_comp);

	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_p\t\talpha_d\t\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
	d_print_exp_tran_mat(stat_m, iter+1, stat, stat_m);

	printf("\ndense ipm time = %e [s]\n\n", time_dense_ipm);
#endif

/************************************************
* free memory
************************************************/

	free(qp_dim_mem);
	free(qp_mem);
	free(qp_sol_mem);
	free(ipm_arg_mem);
	free(ipm_mem);

/************************************************
* return
************************************************/

	return hpipm_return;

	}
