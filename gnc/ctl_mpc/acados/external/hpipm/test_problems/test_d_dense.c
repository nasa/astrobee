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

#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_d_dense_qp_res.h>
#include <hpipm_d_dense_qp_ipm.h>



// printing
#ifndef PRINT
#define PRINT 1
#endif



#if ! defined(EXT_DEP)
/* prints a matrix in column-major format */
void d_print_mat(int m, int n, double *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
/* prints the transposed of a matrix in column-major format */
void d_print_tran_mat(int row, int col, double *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
/* prints a matrix in column-major format (exponential notation) */
void d_print_exp_mat(int m, int n, double *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
/* prints the transposed of a matrix in column-major format (exponential notation) */
void d_print_exp_tran_mat(int row, int col, double *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
#endif



int main()
	{

	int ii;

/************************************************
* qp dimension and data
************************************************/

#if 1
	int nv = 2;
	int ne = 1;
	int nb = 2; //2;
	int ng = 0;
	int ns = 0; //2;
	int nsb = 0; //2;
	int nsg = 0;

	double H[] = {4.0, 1.0, 1.0, 2.0};
	double g[] = {1.0, 1.0};
	double A[] = {1.0, 1.0};
	double b[] = {1.0};
//	double d_lb[] = {0.0, 0.0};
//	double d_ub[] = {INFINITY, INFINITY};
	double d_lb[] = {-1.0, -1.0};
	double d_ub[] = {1.5, 0.5};
	int idxb[] = {0, 1};
	double C[] = {1.0, 0.0, 0.0, 1.0};
	double d_lg[] = {-1.0, -1.0};
	double d_ug[] = {1.5, 0.5};
	double Zl[] = {1e3, 1e3};
	double Zu[] = {1e3, 1e3};
	double zl[] = {1e2, 1e2};
	double zu[] = {1e2, 1e2};
	int idxs[] = {0, 1};
	double d_ls[] = {0, 0};
	double d_us[] = {0, 0};
#elif 0
	int nv = 3;
	int ne = 2;
	int nb = 0; //2;
	int ng = 0;
	int ns = 0; //2;
	int nsb = 0; //2;
	int nsg = 0;

	double H[] = {6.0, 2.0, 1.0, 2.0, 5.0, 2.0, 1.0, 2.0, 4.0};
	double g[] = {-8.0, -3.0, -3.0};
	double A[] = {1.0, 0.0, 0.0, 1.0, 1.0, 1.0};
	double b[] = {3.0, 0.0};
//	double d_lb[] = {0.0, 0.0};
//	double d_ub[] = {INFINITY, INFINITY};
	double d_lb[] = {};
	double d_ub[] = {};
	int idxb[] = {};
	double C[] = {};
	double d_lg[] = {};
/e	double d_ug[] = {};
	double Zl[] = {};
	double Zu[] = {};
	double zl[] = {};
	double zu[] = {};
	int idxs[] = {};
	double d_ls[] = {};
	double d_us[] = {};
#elif 1
	int nv = 5;
	int ne = 5;
	int nb = 0; //2;
	int ng = 0;
	int ns = 0; //2;
	int nsb = 0; //2;
	int nsg = 0;

	double H[25] = {}; for(ii=0; ii<5; ii++) H[ii*(5+1)] = 1.0;
	double g[5] = {};
	double A[25] = {};
	double b[5] = {};
#if 0
	double x[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	double y[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
	double z[] = {2.0, 0.0, 3.0, 1.0, 3.0, 2.0};
	for(ii=0; ii<5; ii++) A[0+ii*5] = 2*x[ii];
	for(ii=0; ii<5; ii++) A[1+ii*5] = x[ii]-y[ii];
	for(ii=0; ii<5; ii++) A[2+ii*5] = 3*x[ii]-2*y[ii];
	for(ii=0; ii<5; ii++) A[3+ii*5] = x[ii]+z[ii];
	for(ii=0; ii<5; ii++) A[4+ii*5] = 3*x[ii]+0.5*y[ii]+2*z[ii];
	b[0] = 2*x[5];
	b[1] = x[5]-y[5];
	b[2] = 3*x[5]-2*y[5];
	b[3] = x[5]+z[5];
	b[4] = 3*x[5]+0.5*y[5]+2*z[5];
#else
	double w[] = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
	double x[] = {0.0, 0.0, 0.0, 1.0, 0.0, 1.0};
	double y[] = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0};
	double z[] = {0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	double zz[] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
	for(ii=0; ii<5; ii++) A[0+ii*5] = 1*zz[ii];
	for(ii=0; ii<5; ii++) A[1+ii*5] = 2*y[ii];
	for(ii=0; ii<5; ii++) A[2+ii*5] = 3*y[ii];
	for(ii=0; ii<5; ii++) A[3+ii*5] = 4*x[ii];
	for(ii=0; ii<5; ii++) A[4+ii*5] = 5*w[ii];
	b[0] = 1*zz[5];
	b[1] = 2*y[5];
	b[2] = 3*y[5];
	b[3] = 4*x[5];
	b[4] = 5*w[5];
#endif
//	double d_lb[] = {0.0, 0.0};
//	double d_ub[] = {INFINITY, INFINITY};
	double d_lb[] = {};
	double d_ub[] = {};
	int idxb[] = {};
	double C[] = {};
	double d_lg[] = {};
	double d_ug[] = {};
	double Zl[] = {};
	double Zu[] = {};
	double zl[] = {};
	double zu[] = {};
	int idxs[] = {};
	double d_ls[] = {};
	double d_us[] = {};
#else
	int nv = 10;
	int ne = 0;
	int nb = nv;
	int ng = 0;
	int ns = 0;

	double H[nv*nv]; for(ii=0; ii<nv*nv; ii++) H[ii] = 0.0; for(ii=0; ii<nv; ii++) H[ii*(nv+1)] = 1.0;
	double g[nv]; for(ii=0; ii<nv; ii++) g[ii] = 10.0;
	double A[ne*nv]; for(ii=0; ii<ne*nv; ii++) A[ii] = 0.0;
	double b[ne]; for(ii=0; ii<ne; ii++) b[ii] = 0.0;
	double d_lb[nb]; for(ii=0; ii<nb; ii++) d_lb[ii] = -1.0;
	double d_ub[nb]; for(ii=0; ii<nb; ii++) d_ub[ii] =  1.0;
	int idxb[nb]; for(ii=0; ii<nb; ii++) idxb[ii] = ii;
	double C[0];
	double d_lg[0];
	double d_ug[0];
	double Zl[0];
	double Zu[0];
	double zl[0];
	double zu[0];
	int idxs[0];
	double d_ls[] = {0, 0};
	double d_us[] = {0, 0};
#endif

/************************************************
* dense qp dim
************************************************/

	hpipm_size_t dense_qp_dim_size = d_dense_qp_dim_memsize();
#if PRINT
	printf("\nqp dim size = %d\n", dense_qp_dim_size);
#endif
	void *qp_dim_mem = malloc(dense_qp_dim_size);

	struct d_dense_qp_dim qp_dim;
	d_dense_qp_dim_create(&qp_dim, qp_dim_mem);

	d_dense_qp_dim_set_all(nv, ne, nb, ng, nsb, nsg, &qp_dim);

/************************************************
* dense qp
************************************************/

	hpipm_size_t qp_size = d_dense_qp_memsize(&qp_dim);
#if PRINT
	printf("\nqp size = %d\n", qp_size);
#endif
	void *qp_mem = malloc(qp_size);

	struct d_dense_qp qp;
	d_dense_qp_create(&qp_dim, &qp, qp_mem);

#if 1
	// test setters

	d_dense_qp_set_H(H, &qp);
	d_dense_qp_set_g(g, &qp);
	d_dense_qp_set_A(A, &qp);
	d_dense_qp_set_b(b, &qp);
	d_dense_qp_set_idxb(idxb, &qp);
	d_dense_qp_set_lb(d_lb, &qp);
	d_dense_qp_set_ub(d_ub, &qp);
	d_dense_qp_set_C(C, &qp);
	d_dense_qp_set_lg(d_lg, &qp);
	d_dense_qp_set_ug(d_ug, &qp);
	d_dense_qp_set_Zl(Zl, &qp);
	d_dense_qp_set_Zu(Zu, &qp);
	d_dense_qp_set_zl(zl, &qp);
	d_dense_qp_set_zu(zu, &qp);
	d_dense_qp_set_idxs(idxs, &qp);
	d_dense_qp_set_ls(d_ls, &qp);
	d_dense_qp_set_us(d_us, &qp);
#else
	d_dense_qp_set_all(H, g, A, b, idxb, d_lb, d_ub, C, d_lg, d_ug, Zl, Zu, zl, zu, idxs, d_ls, d_us, &qp);
#endif

#if PRINT
	printf("\nH = \n");
	blasfeo_print_dmat(nv, nv, qp.Hv, 0, 0);
	printf("\nA = \n");
	blasfeo_print_dmat(ne, nv, qp.A, 0, 0);
	printf("\nCt = \n");
	blasfeo_print_dmat(nv, ng, qp.Ct, 0, 0);
	printf("\ng = \n");
	blasfeo_print_dvec(nv, qp.gz, 0);
	printf("\nb = \n");
	blasfeo_print_dvec(ne, qp.b, 0);
	printf("\nd = \n");
	blasfeo_print_dvec(2*nb+2*ng, qp.d, 0);
#endif

//qp.d_mask->pa[0] = 0;
//qp.d_mask->pa[1] = 0;
//qp.d_mask->pa[2] = 0;
//qp.d_mask->pa[3] = 0;
//qp.d_mask->pa[4] = 0;
//qp.d_mask->pa[5] = 0;
//qp.d_mask->pa[6] = 0;
//qp.d_mask->pa[7] = 0;

/************************************************
* dense qp sol
************************************************/

	hpipm_size_t qp_sol_size = d_dense_qp_sol_memsize(&qp_dim);
#if PRINT
	printf("\nqp sol size = %d\n", qp_sol_size);
#endif
	void *qp_sol_mem = malloc(qp_sol_size);

	struct d_dense_qp_sol qp_sol;
	d_dense_qp_sol_create(&qp_dim, &qp_sol, qp_sol_mem);

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_dense_qp_ipm_arg_memsize(&qp_dim);
#if PRINT
	printf("\nipm arg size = %d\n", ipm_arg_size);
#endif
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_dense_qp_ipm_arg arg;
	d_dense_qp_ipm_arg_create(&qp_dim, &arg, ipm_arg_mem);
//	enum hpipm_mode mode = SPEED_ABS;
	enum hpipm_mode mode = SPEED;
//	enum hpipm_mode mode = BALANCE;
//	enum hpipm_mode mode = ROBUST;
	d_dense_qp_ipm_arg_set_default(mode, &arg);

	int iter_max = 20; //25;
	double mu0 = 1e1;
	int comp_res_exit = 1;
	double tol_stat = 1e-12;
	double tol_eq = 1e-12;
	double tol_ineq = 1e-12;
	double tol_comp = 1e-12;
	int kkt_fact_alg = 0;
	int remove_lin_dep_eq = 1;
	double tau_min = 1e-3;

	d_dense_qp_ipm_arg_set_iter_max(&iter_max, &arg);
	d_dense_qp_ipm_arg_set_mu0(&mu0, &arg);
	d_dense_qp_ipm_arg_set_comp_res_exit(&comp_res_exit, &arg);
	d_dense_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
	d_dense_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
	d_dense_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
	d_dense_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
	d_dense_qp_ipm_arg_set_kkt_fact_alg(&kkt_fact_alg, &arg);
	d_dense_qp_ipm_arg_set_remove_lin_dep_eq(&remove_lin_dep_eq, &arg);
//	d_dense_qp_ipm_arg_set_tau_min(&tau_min, &arg);

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

	hpipm_size_t ipm_size = d_dense_qp_ipm_ws_memsize(&qp_dim, &arg);
#if PRINT
	printf("\nipm size = %d\n", ipm_size);
#endif
	void *ipm_mem = malloc(ipm_size);

	struct d_dense_qp_ipm_ws workspace;
	d_dense_qp_ipm_ws_create(&qp_dim, &arg, &workspace, ipm_mem);

	// check for linearly dependent equality constraints
	d_dense_qp_remove_lin_dep_eq(&qp, &arg, &workspace);
#if PRINT
	blasfeo_print_dmat(qp_dim.ne, qp_dim.nv, qp.A, 0, 0);
	blasfeo_print_tran_dvec(qp_dim.ne, qp.b, 0);
#endif
	d_dense_qp_restore_lin_dep_eq(&qp, &arg, &workspace);
//	exit(1);

	int rep, nrep=1; //000;

	int hpipm_return; // 0 normal; 1 max iter

	struct timeval tv0, tv1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		d_dense_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
		d_dense_qp_ipm_get_status(&workspace, &hpipm_return);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_dense_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

#if PRINT
	printf("\nsolution\n\n");
	printf("\nv\n");
	d_print_mat(1, nv, qp_sol.v->pa, 1);
	printf("\nls\n");
	d_print_mat(1, ns, qp_sol.v->pa+nv, 1);
	printf("\nus\n");
	d_print_mat(1, ns, qp_sol.v->pa+nv+ns, 1);
	printf("\npi\n");
	d_print_mat(1, ne, qp_sol.pi->pa, 1);
	printf("\nlam_lb\n");
	d_print_mat(1, nb, qp_sol.lam->pa+0, 1);
	printf("\nlam_ub\n");
	d_print_mat(1, nb, qp_sol.lam->pa+nb+ng, 1);
	printf("\nlam_lg\n");
	d_print_mat(1, ng, qp_sol.lam->pa+nb, 1);
	printf("\nlam_ug\n");
	d_print_mat(1, ng, qp_sol.lam->pa+2*nb+ng, 1);
	printf("\nlam_ls\n");
	d_print_mat(1, ns, qp_sol.lam->pa+2*nb+2*ng, 1);
	printf("\nlam_us\n");
	d_print_mat(1, ns, qp_sol.lam->pa+2*nb+2*ng+ns, 1);
	printf("\nt_lb\n");
	d_print_mat(1, nb, qp_sol.t->pa+0, 1);
	printf("\nt_ub\n");
	d_print_mat(1, nb, qp_sol.t->pa+nb+ng, 1);
	printf("\nt_lg\n");
	d_print_mat(1, ng, qp_sol.t->pa+nb, 1);
	printf("\nt_ug\n");
	d_print_mat(1, ng, qp_sol.t->pa+2*nb+ng, 1);
	printf("\nt_ls\n");
	d_print_mat(1, ns, qp_sol.t->pa+2*nb+2*ng, 1);
	printf("\nt_us\n");
	d_print_mat(1, ns, qp_sol.t->pa+2*nb+2*ng+ns, 1);

	printf("\nresiduals\n\n");
	printf("\nres_g\n");
	d_print_exp_mat(1, nv+2*ns, workspace.res->res_g->pa, 1);
	printf("\nres_b\n");
	d_print_exp_mat(1, ne, workspace.res->res_b->pa, 1);
	printf("\nres_d\n");
	d_print_exp_mat(1, 2*nb+2*ng+2*ns, workspace.res->res_d->pa, 1);
	printf("\nres_m\n");
	d_print_exp_mat(1, 2*nb+2*ng+2*ns, workspace.res->res_m->pa, 1);
	printf("\nres_mu\n");
	printf("\n%e\n\n", workspace.res->res_mu);
#endif

/************************************************
* print ipm statistics
************************************************/

	int iter; d_dense_qp_ipm_get_iter(&workspace, &iter);
	double max_res_stat; d_dense_qp_ipm_get_max_res_stat(&workspace, &max_res_stat);
	double max_res_eq  ; d_dense_qp_ipm_get_max_res_eq(&workspace, &max_res_eq);
	double max_res_ineq; d_dense_qp_ipm_get_max_res_ineq(&workspace, &max_res_ineq);
	double max_res_comp; d_dense_qp_ipm_get_max_res_comp(&workspace, &max_res_comp);
	double *stat; d_dense_qp_ipm_get_stat(&workspace, &stat);
	int stat_m;  d_dense_qp_ipm_get_stat_m(&workspace, &stat_m);

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


