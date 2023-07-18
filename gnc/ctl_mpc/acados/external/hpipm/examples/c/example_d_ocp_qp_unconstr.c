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
#include <sys/time.h>

#include <blasfeo_d_aux_ext_dep.h>

#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>



// qp data as global data
extern int N;
extern int *nx;
extern int *nu;
extern int *nbu;
extern int *nbx;
extern int *ng;
extern int *nsbx;
extern int *nsbu;
extern int *nsg;
extern double **hA;
extern double **hB;
extern double **hb;
extern double **hQ;
extern double **hR;
extern double **hS;
extern double **hq;
extern double **hr;
extern int **hidxbx;
extern double **hlbx;
extern double **hubx;
extern int **hidxbu;
extern double **hlbu;
extern double **hubu;
extern double **hC;
extern double **hD;
extern double **hlg;
extern double **hug;
extern double **hZl;
extern double **hZu;
extern double **hzl;
extern double **hzu;
extern int **hidxs;
extern double **hlls;
extern double **hlus;
//extern double **hu_guess;
//extern double **hx_guess;
//extern double **hsl_guess;
//extern double **hsu_guess;
// arg
extern int mode;
extern int iter_max;
extern double alpha_min;
extern double mu0;
extern double tol_stat;
extern double tol_eq;
extern double tol_ineq;
extern double tol_comp;
extern double reg_prim;
extern int warm_start;
extern int pred_corr;
extern int ric_alg;



// main
int main()
	{

	int ii, jj;

	int hpipm_status;

	int rep, nrep=10;

	struct timeval tv0, tv1;

/************************************************
* new first stage data
************************************************/

	double *x0 = malloc(nx[0]*sizeof(double));

	for(ii=0; ii<nx[0]; ii++)
		x0[ii] = hlbx[0][ii];

	d_print_mat(1, nx[0], x0, 1);

	int *idxbx0 = malloc(nx[0]*sizeof(int));

	for(ii=0; ii<nx[0]; ii++)
		idxbx0[ii] = ii;

	double *b0 = malloc(nx[1]*sizeof(double));

	for(ii=0; ii<nx[1]; ii++)
		b0[ii] = hb[0][ii];
	
	for(ii=0; ii<nx[1]; ii++)
		for(jj=0; jj<nx[0]; jj++)
			b0[ii] += hA[0][ii+nx[1]*jj]*x0[jj];
	
	double *r0 = malloc(nu[0]*sizeof(double));

	for(ii=0; ii<nu[0]; ii++)
		r0[ii] = hr[0][ii];
	
	for(ii=0; ii<nu[0]; ii++)
		for(jj=0; jj<nx[0]; jj++)
			r0[ii] += hS[0][ii+nu[0]*jj]*x0[jj];

/************************************************
* remove constraints
************************************************/

#if 1
	// keep x0 as an optimization variable, set its value using equal upper-lower bounds
//	for(ii=1; ii<=N; ii++)
//		nbx[ii] = 0;

//	hlbx[0] = x0;
//	hubx[0] = x0;
//	hidxbx[0] = idxbx0;
#else
	// remove x0 from the optimization variables
	nx[0] = 0;

	for(ii=0; ii<=N; ii++)
		nbx[ii] = 0;

	hb[0] = b0;
	hr[0] = r0;
#endif

	for(ii=0; ii<=N; ii++)
		nbu[ii] = 0;

	for(ii=0; ii<=N; ii++)
		ng[ii] = 0;

	for(ii=0; ii<=N; ii++)
		nsbx[ii] = 0;

	for(ii=0; ii<=N; ii++)
		nsbu[ii] = 0;

	for(ii=0; ii<=N; ii++)
		nsg[ii] = 0;

/************************************************
* ocp qp dim
************************************************/

	hpipm_size_t dim_size = d_ocp_qp_dim_memsize(N);
	void *dim_mem = malloc(dim_size);

	struct d_ocp_qp_dim dim;
	d_ocp_qp_dim_create(N, &dim, dim_mem);

	d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dim);

/************************************************
* ocp qp
************************************************/

	hpipm_size_t qp_size = d_ocp_qp_memsize(&dim);
	void *qp_mem = malloc(qp_size);

	struct d_ocp_qp qp;
	d_ocp_qp_create(&dim, &qp, qp_mem);

	d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr, hidxbx, hlbx, hubx, hidxbu, hlbu, hubu, hC, hD, hlg, hug, hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp);

/************************************************
* ocp qp sol
************************************************/

	hpipm_size_t qp_sol_size = d_ocp_qp_sol_memsize(&dim);
	void *qp_sol_mem = malloc(qp_sol_size);

	struct d_ocp_qp_sol qp_sol;
	d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_ocp_qp_ipm_arg arg;
	d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

	d_ocp_qp_ipm_arg_set_default(mode, &arg);

	d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
	d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
	d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
	d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
	d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
	d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
	d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
	d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
	d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);

/************************************************
* ipm workspace
************************************************/

	hpipm_size_t ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
	void *ipm_mem = malloc(ipm_size);

	struct d_ocp_qp_ipm_ws workspace;
	d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

/************************************************
* ipm solver
************************************************/

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		// solution guess
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_u(ii, hu_guess[ii], &qp_sol);
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_x(ii, hx_guess[ii], &qp_sol);
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_sl(ii, hsl_guess[ii], &qp_sol);
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_su(ii, hsu_guess[ii], &qp_sol);

		// call solver
		d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
		d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* print solution info
************************************************/

    printf("\nHPIPM returned with flag %i.\n", hpipm_status);
    if(hpipm_status == 0)
		{
        printf("\n -> QP solved!\n");
		}
	else if(hpipm_status==1)
		{
        printf("\n -> Solver failed! Maximum number of iterations reached\n");
		}
	else if(hpipm_status==2)
		{
        printf("\n -> Solver failed! Minimum step lenght reached\n");
		}
	else if(hpipm_status==2)
		{
        printf("\n -> Solver failed! NaN in computations\n");
		}
	else
		{
        printf("\n -> Solver failed! Unknown return flag\n");
		}
    printf("\nAverage solution time over %i runs: %e [s]\n", nrep, time_ipm);
	printf("\n\n");

/************************************************
* extract and print solution
************************************************/

	// u

	int nu_max = nu[0];
	for(ii=1; ii<=N; ii++)
		if(nu[ii]>nu_max)
			nu_max = nu[ii];

	double *u = malloc(nu_max*sizeof(double));

	printf("\nu = \n");
	for(ii=0; ii<=N; ii++)
		{
		d_ocp_qp_sol_get_u(ii, &qp_sol, u);
		d_print_mat(1, nu[ii], u, 1);
		}

	// x

	int nx_max = nx[0];
	for(ii=1; ii<=N; ii++)
		if(nx[ii]>nx_max)
			nx_max = nx[ii];

	double *x = malloc(nx_max*sizeof(double));

	printf("\nx = \n");
	for(ii=0; ii<=N; ii++)
		{
		d_ocp_qp_sol_get_x(ii, &qp_sol, x);
		d_print_mat(1, nx[ii], x, 1);
		}

	// pi
	double *pi = malloc(nx_max*sizeof(double));

	printf("\npi = \n");
	for(ii=0; ii<N; ii++)
		{
		d_ocp_qp_sol_get_pi(ii, &qp_sol, pi);
		d_print_mat(1, nx[ii+1], pi, 1);
		}

/************************************************
* print ipm statistics
************************************************/

	int iter; d_ocp_qp_ipm_get_iter(&workspace, &iter);
	double res_stat; d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
	double res_eq; d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
	double res_ineq; d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
	double res_comp; d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
	double *stat; d_ocp_qp_ipm_get_stat(&workspace, &stat);
	int stat_m; d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

	printf("\nipm return = %d\n", hpipm_status);
	printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

	printf("\nipm iter = %d\n", iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
	d_print_exp_tran_mat(stat_m, iter+1, stat, stat_m);

	printf("\nocp ipm time = %e [s]\n\n", time_ipm);

/************************************************
* get riccati matrices and vectors
************************************************/

#if 1
	printf("\nget Riccati recursion matrices and vectors\n");

	double *Lr0 = malloc(nu[0]*nu[0]*sizeof(double));
	double *Ls0 = malloc(nx[0]*nu[0]*sizeof(double));
	double *P0 = malloc(nx[0]*nx[0]*sizeof(double));
	double *lr0 = malloc(nu[0]*sizeof(double));
	double *p0 = malloc(nx[0]*sizeof(double));
	double *K0 = malloc(nu[0]*nx[0]*sizeof(double));
	double *k0 = malloc(nu[0]*sizeof(double));

	double *Lr1 = malloc(nu[1]*nu[1]*sizeof(double));
	double *Ls1 = malloc(nx[1]*nu[1]*sizeof(double));
	double *P1 = malloc(nx[1]*nx[1]*sizeof(double));
	double *lr1 = malloc(nu[1]*sizeof(double));
	double *p1 = malloc(nx[1]*sizeof(double));
	double *K1 = malloc(nu[1]*nx[1]*sizeof(double));
	double *k1 = malloc(nu[1]*sizeof(double));

	//
	d_ocp_qp_ipm_get_ric_Lr(&qp, &arg, &workspace, 0, Lr0);
	printf("\nLr0\n");
	d_print_exp_mat(nu[0], nu[0], Lr0, nu[0]);
	//
	d_ocp_qp_ipm_get_ric_Ls(&qp, &arg, &workspace, 0, Ls0);
	printf("\nLs0\n");
	d_print_exp_mat(nx[0], nu[0], Ls0, nx[0]);
	//
	d_ocp_qp_ipm_get_ric_P(&qp, &arg, &workspace, 0, P0);
	printf("\nP0\n");
	d_print_exp_mat(nx[0], nx[0], P0, nx[0]);
	//
	d_ocp_qp_ipm_get_ric_lr(&qp, &arg, &workspace, 0, lr0);
	printf("\nlr0\n");
	d_print_exp_mat(1, nu[0], lr0, 1);
	//
	d_ocp_qp_ipm_get_ric_p(&qp, &arg, &workspace, 0, p0);
	printf("\np0\n");
	d_print_exp_mat(1, nx[0], p0, 1);
	//
	d_ocp_qp_ipm_get_ric_K(&qp, &arg, &workspace, 0, K0);
	printf("\nK0\n");
	d_print_exp_mat(nu[0], nx[0], K0, nu[0]);
	//
	d_ocp_qp_ipm_get_ric_k(&qp, &arg, &workspace, 0, k0);
	printf("\nk0\n");
	d_print_exp_mat(1, nu[0], k0, 1);

	//
	d_ocp_qp_ipm_get_ric_Lr(&qp, &arg, &workspace, 1, Lr1);
	printf("\nLr1\n");
	d_print_exp_mat(nu[1], nu[1], Lr1, nu[1]);
	//
	d_ocp_qp_ipm_get_ric_Ls(&qp, &arg, &workspace, 1, Ls1);
	printf("\nLs1\n");
	d_print_exp_mat(nx[1], nu[1], Ls1, nx[1]);
	//
	d_ocp_qp_ipm_get_ric_P(&qp, &arg, &workspace, 1, P1);
	printf("\nP1\n");
	d_print_exp_mat(nx[1], nx[1], P1, nx[1]);
	//
	d_ocp_qp_ipm_get_ric_lr(&qp, &arg, &workspace, 1, lr1);
	printf("\nlr1\n");
	d_print_exp_mat(1, nu[1], lr1, 1);
	//
	d_ocp_qp_ipm_get_ric_p(&qp, &arg, &workspace, 1, p1);
	printf("\np1\n");
	d_print_exp_mat(1, nx[1], p1, 1);
	//
	d_ocp_qp_ipm_get_ric_K(&qp, &arg, &workspace, 1, K1);
	printf("\nK1\n");
	d_print_exp_mat(nu[1], nx[1], K1, nu[1]);
	//
	d_ocp_qp_ipm_get_ric_k(&qp, &arg, &workspace, 1, k1);
	printf("\nk1\n");
	d_print_exp_mat(1, nu[1], k1, 1);

	free(Lr0);
	free(Ls0);
	free(P0);
	free(lr0);
	free(p0);
	free(K0);
	free(k0);

	free(Lr1);
	free(Ls1);
	free(P1);
	free(lr1);
	free(p1);
	free(K1);
	free(k1);
#endif

/************************************************
* free memory and return
************************************************/

	free(x0);
	free(idxbx0);
	free(b0);
	free(r0);

	free(dim_mem);
	free(qp_mem);
	free(qp_sol_mem);
	free(ipm_arg_mem);
	free(ipm_mem);

	free(u);
	free(x);
	free(pi);

	return 0;

	}



