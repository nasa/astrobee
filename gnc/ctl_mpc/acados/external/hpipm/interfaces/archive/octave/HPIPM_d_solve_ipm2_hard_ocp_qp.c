/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2017-2018 by Gianluca Frison.                                                     *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* This program is free software: you can redistribute it and/or modify                            *
* it under the terms of the GNU General Public License as published by                            *
* the Free Software Foundation, either version 3 of the License, or                               *
* (at your option) any later version                                                              *.
*                                                                                                 *
* This program is distributed in the hope that it will be useful,                                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                   *
* GNU General Public License for more details.                                                    *
*                                                                                                 *
* You should have received a copy of the GNU General Public License                               *
* along with this program.  If not, see <https://www.gnu.org/licenses/>.                          *
*                                                                                                 *
* The authors designate this particular file as subject to the "Classpath" exception              *
* as provided by the authors in the LICENSE file that accompained this code.                      *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
/*#include <math.h>*/

#include "hpipm_common.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_d_ocp_qp_ipm.h"




// z = beta*y + alpha*A*x
void dgemv_n_3l(int m, int n, double alpha, double *A, int lda, double *x, double beta, double *y, double *z)
	{

	int ii, jj;

	double tmp;

	for(ii=0; ii<m; ii++)
		z[ii] = beta * y[ii];

	for(jj=0; jj<n; jj++)
		{
		tmp = alpha * x[jj];
		for(ii=0; ii<m; ii++)
			{
			z[ii] += A[ii+lda*jj] * tmp;
			}
		}

	}



// the gateway function
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
	{

	// get data
	int k_max;
	double mu0, tol, *A, *B, *b, *Q, *Qf, *R, *S, *q, *qf, *r, *x, *u, *lb, *ub, *C, *D, *lg, *ug, *CN, *lgN, *ugN, *stat, *kkk, *inf_norm_res, *pi, *lam/*, *t*/;

	kkk   = mxGetPr(prhs[0]);
	k_max = (int) mxGetScalar(prhs[1]);
	mu0   = mxGetScalar(prhs[2]);
	tol   = mxGetScalar(prhs[3]);
	const int N   = (int) mxGetScalar(prhs[4]);
	const int nx  = (int) mxGetScalar(prhs[5]);
	const int nu  = (int) mxGetScalar(prhs[6]);
	const int nb  = (int) mxGetScalar(prhs[7]);
	const int ng  = (int) mxGetScalar(prhs[8]);
	const int ngN = (int) mxGetScalar(prhs[9]);
	const int time_invariant = (int) mxGetScalar(prhs[10]);
	const int free_x0 = (int) mxGetScalar(prhs[11]);
	const int warm_start = (int) mxGetScalar(prhs[12]);

	A    = mxGetPr(prhs[13]);
	B    = mxGetPr(prhs[14]);
	b    = mxGetPr(prhs[15]);
	Q    = mxGetPr(prhs[16]);
	Qf   = mxGetPr(prhs[17]);
	R    = mxGetPr(prhs[18]);
	S    = mxGetPr(prhs[19]);
	q    = mxGetPr(prhs[20]);
	qf   = mxGetPr(prhs[21]);
	r    = mxGetPr(prhs[22]);
	lb   = mxGetPr(prhs[23]);
	ub   = mxGetPr(prhs[24]);
	C    = mxGetPr(prhs[25]);
	D    = mxGetPr(prhs[26]);
	lg   = mxGetPr(prhs[27]);
	ug   = mxGetPr(prhs[28]);
	CN   = mxGetPr(prhs[29]);
	lgN  = mxGetPr(prhs[30]);
	ugN  = mxGetPr(prhs[31]);
	x    = mxGetPr(prhs[32]);
	u    = mxGetPr(prhs[33]);
	pi  = mxGetPr(prhs[34]);
	lam = mxGetPr(prhs[35]);
//	t   = mxGetPr(prhs[36]);
//	inf_norm_res = mxGetPr(prhs[37]);
//	stat = mxGetPr(prhs[38]);
	inf_norm_res = mxGetPr(prhs[36]);
	stat = mxGetPr(prhs[37]);

	int kk = -1;



	//
	int ii, jj;
	int i_tmp;



	int nb0 = nb<nu ? nb : nu;
	int nbN = nb-nu>0 ? nb-nu : 0;



	int nx_v[N+1];
	nx_v[0] = 0;
	for(ii=1; ii<=N; ii++)
		nx_v[ii] = nx;

	int nu_v[N+1];
	for(ii=0; ii<N; ii++)
		nu_v[ii] = nu;
	nu_v[N] = 0;

	int nb_v[N+1];
	nb_v[0] = nb<nu ? nb : nu;
	for(ii=1; ii<N; ii++)
		nb_v[ii] = nb;
	i_tmp = nb-nu;
	nb_v[N] = i_tmp<0 ? 0 : i_tmp;

	int nbu_v[N+1];
	for(ii=0; ii<N; ii++)
		nbu_v[ii] = nb<nu ? nb : nu;
	nbu_v[N] = 0;

	int nbx_v[N+1];
	for(ii=0; ii<=N; ii++)
		{
		i_tmp = nb_v[ii]-nbu_v[ii];
		nbx_v[ii] = i_tmp>=0 ? i_tmp : 0;
		}

	int ng_v[N+1];
	for(ii=0; ii<N; ii++)
		ng_v[ii] = ng;
	ng_v[N] = ngN;

	int ns_v[N+1];
	for(ii=0; ii<=N; ii++)
		ns_v[ii] = 0;

	int nsbx_v[N+1];
	for(ii=0; ii<=N; ii++)
		nsbx_v[ii] = 0;

	int nsbu_v[N+1];
	for(ii=0; ii<=N; ii++)
		nsbu_v[ii] = 0;

	int nsg_v[N+1];
	for(ii=0; ii<=N; ii++)
		nsg_v[ii] = 0;

	int *hidxb[N+1];
	int *ptr_idx = (int *) malloc((N+1)*nb*sizeof(int));
	for(ii=0; ii<=N; ii++)
		{
		hidxb[ii] = ptr_idx+ii*nb;
		for(jj=0; jj<nb_v[ii]; jj++)
			hidxb[ii][jj] = jj;
		}


	double b0[nx];
	dgemv_n_3l(nx, nx, 1.0, A, nx, x, 1.0, b, b0);

	double r0[nu];
	dgemv_n_3l(nu, nx, 1.0, S, nu, x, 1.0, r, r0);

	double lb0[nb0];
	for(ii=0; ii<nb0; ii++)
		lb0[ii] = lb[ii];

	double ub0[nb0];
	for(ii=0; ii<nb0; ii++)
		ub0[ii] = ub[ii];

	double lbN[nbN];
	double ubN[nbN];

	double lg0[ng];
	dgemv_n_3l(ng, nx, -1.0, C, ng, x, 1.0, lg, lg0);

	double ug0[ng];
	dgemv_n_3l(ng, nx, -1.0, C, ng, x, 1.0, ug, ug0);



	double *hA[N];
	double *hB[N];
	double *hb[N];
	double *hQ[N+1];
	double *hS[N];
	double *hR[N];
	double *hq[N+1];
	double *hr[N];
	double *hlb[N+1];
	double *hub[N+1];
	double *hC[N+1];
	double *hD[N];
	double *hlg[N+1];
	double *hug[N+1];
	double *hx[N+1];
	double *hu[N+1];
	double *hpi[N];
//	double *hlam[N+1];
//	double *ht[N+1];
	double *hlam_lb[N+1];
	double *hlam_ub[N+1];
	double *hlam_lg[N+1];
	double *hlam_ug[N+1];



	if(time_invariant)
		{

		for(ii=1; ii<N; ii++)
			hA[ii] = A;

		for(ii=0; ii<N; ii++)
			hB[ii] = B;

		hb[0] = b0;
		for(ii=1; ii<N; ii++)
			hb[ii] = b;

		for(ii=1; ii<N; ii++)
			hQ[ii] = Q;
		hQ[N] = Qf;

		for(ii=1; ii<N; ii++)
			hS[ii] = S;

		for(ii=0; ii<N; ii++)
			hR[ii] = R;

		for(ii=1; ii<N; ii++)
			hq[ii] = q;
		hq[N] = qf;

		hr[0] = r0;
		for(ii=1; ii<N; ii++)
			hr[ii] = r;

		for(ii=0; ii<nbN; ii++)
			lbN[ii] = lb[nu+ii];

		hlb[0] = lb0;
		for(ii=1; ii<N; ii++)
			hlb[ii] = lb;
		hlb[N] = lbN;

		for(ii=0; ii<nbN; ii++)
			ubN[ii] = ub[nu+ii];

		hub[0] = ub0;
		for(ii=1; ii<N; ii++)
			hub[ii] = ub;
		hub[N] = ubN;

		for(ii=1; ii<N; ii++)
			hC[ii] = C;
		hC[N] = CN;

		for(ii=0; ii<N; ii++)
			hD[ii] = D;

		hlg[0] = lg0;
		for(ii=1; ii<N; ii++)
			hlg[ii] = lg;
		hlg[N] = lgN;

		hug[0] = ug0;
		for(ii=1; ii<N; ii++)
			hug[ii] = ug;
		hug[N] = ugN;

		}
	else
		{

		for(ii=1; ii<N; ii++)
			hA[ii] = A+ii*nx*nx;

		for(ii=0; ii<N; ii++)
			hB[ii] = B+ii*nx*nu;

		hb[0] = b0;
		for(ii=1; ii<N; ii++)
			hb[ii] = b+ii*nx;

		for(ii=1; ii<N; ii++)
			hQ[ii] = Q+ii*nx*nx;
		hQ[N] = Qf;

		for(ii=1; ii<N; ii++)
			hS[ii] = S+ii*nu*nx;

		for(ii=0; ii<N; ii++)
			hR[ii] = R+ii*nu*nu;

		for(ii=1; ii<N; ii++)
			hq[ii] = q+ii*nx;
		hq[N] = qf;

		hr[0] = r0;
		for(ii=1; ii<N; ii++)
			hr[ii] = r+ii*nu;

		for(ii=0; ii<nbN; ii++)
			lbN[ii] = lb[nu+ii];
		hlb[0] = lb0;
		for(ii=1; ii<N; ii++)
			hlb[ii] = lb+ii*nb;
		hlb[N] = lbN;

		for(ii=0; ii<nbN; ii++)
			ubN[ii] = ub[nu+ii];
		hub[0] = ub0;
		for(ii=1; ii<N; ii++)
			hub[ii] = ub+ii*nb;
		hub[N] = ubN;

		for(ii=1; ii<N; ii++)
			hC[ii] = C+ii*ng*nx;
		hC[N] = CN;

		for(ii=0; ii<N; ii++)
			hD[ii] = D+ii*ng*nu;

		hlg[0] = lg0;
		for(ii=1; ii<N; ii++)
			hlg[ii] = lg+ii*ng;
		hlg[N] = lgN;

		hug[0] = ug0;
		for(ii=1; ii<N; ii++)
			hug[ii] = ug+ii*ng;
		hug[N] = ugN;

		}

	for(ii=0; ii<=N; ii++)
		hx[ii] = x+ii*nx;

	for(ii=0; ii<N; ii++)
		hu[ii] = u+ii*nu;

	for(ii=0; ii<N; ii++)
		hpi[ii] = pi+ii*nx;

//	for(ii=0; ii<=N; ii++)
//		hlam[ii] = lam+ii*(2*nb+2*ng);

//	for(ii=0; ii<=N; ii++)
//		ht[ii] = t+ii*(2*nb+2*ng);

	for(ii=0; ii<=N; ii++)
		{
		hlam_lb[ii] = lam+ii*nb;
		hlam_ub[ii] = lam+(N+1)*nb+ii*nb;
		hlam_lg[ii] = lam+2*(N+1)*nb+ii*ng;
//		hlam_ug[ii] = lam+2*(N+1)*nb+(N+1)*ng+ii*ng;
		hlam_ug[ii] = lam+2*(N+1)*nb+N*ng+ngN+ii*ng;
		}

	// Partial condensing horizon
//	int N2 = N;

//	hpipm_size_t work_space_size = hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(N, nx_v, nu_v, nb_v, hidxb, ng_v, N2);
//	void *work = malloc( work_space_size );



	// call solver
//	fortran_order_d_ip_ocp_hard_tv(&kk, k_max, mu0, tol, N, nx_v, nu_v, nb_v, hidxb, ng_v, N2, warm_start, hA, hB, hb, hQ, hS, hR, hq, hr, hlb, hub, hC, hD, hlg, hug, hx, hu, hpi, hlam, /*ht,*/ inf_norm_res, work, stat);




	// qp dim
	hpipm_size_t dim_size = d_memsize_ocp_qp_dim(N);
	void *dim_mem = malloc(dim_size);

	struct d_ocp_qp_dim dim;
	d_create_ocp_qp_dim(N, &dim, dim_mem);
	d_cvt_int_to_ocp_qp_dim(N, nx_v, nu_v, nbx_v, nbu_v, ng_v, nsbx_v, nsbu_v, nsg_v, &dim);

//	for(ii=0; ii<=N; ii++)
//		printf("\n%d %d %d %d %d %d %d\n", nx_v[ii], nu_v[ii], nb_v[ii], nbu_v[ii], nbx_v[ii], ng_v[ii], ns_v[ii]);


	// qp
	hpipm_size_t qp_size = d_memsize_ocp_qp(&dim);
	void *qp_mem = malloc(qp_size);
	struct d_ocp_qp qp;
	d_create_ocp_qp(&dim, &qp, qp_mem);
	d_cvt_colmaj_to_ocp_qp(hA, hB, hb, hQ, hS, hR, hq, hr, hidxb, hlb, hub, hC, hD, hlg, hug, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &qp);


	// qp sol
	hpipm_size_t qp_sol_size = d_memsize_ocp_qp_sol(&dim);
	void *qp_sol_mem = malloc(qp_sol_size);
	struct d_ocp_qp_sol qp_sol;
	d_create_ocp_qp_sol(&dim, &qp_sol, qp_sol_mem);


	// ipm arg
	hpipm_size_t ipm_arg_size = d_memsize_ocp_qp_ipm_arg(&dim);
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_ocp_qp_ipm_arg arg;
	d_create_ocp_qp_ipm_arg(&dim, &arg, ipm_arg_mem);
	enum hpipm_mode mode = SPEED;
//	enum hpipm_mode mode = BALANCE;
//	enum hpipm_mode mode = ROBUST;
	d_set_default_ocp_qp_ipm_arg(mode, &arg);

//	arg.alpha_min = 1e-12;
	arg.res_g_max = tol;
	arg.res_b_max = tol;
	arg.res_d_max = tol;
	arg.res_m_max = tol;
	arg.mu0 = mu0;
	arg.iter_max = k_max;
	arg.stat_max = k_max;
//	arg.pred_corr = 1;
//	arg.cond_pred_corr = 1;


	// ipm
	hpipm_size_t ipm_size = d_memsize_ocp_qp_ipm(&dim, &arg);
	void *ipm_mem = malloc(ipm_size);

	struct d_ocp_qp_ipm_workspace workspace;
	d_create_ocp_qp_ipm(&dim, &arg, &workspace, ipm_mem);

	// call solver
	int hpipm_return = d_solve_ocp_qp_ipm(&qp, &qp_sol, &arg, &workspace);

	// convert back solution
	d_cvt_ocp_qp_sol_to_colmaj(&qp_sol, hu, hx, NULL, NULL, hpi, hlam_lb, hlam_ub, hlam_lg, hlam_ug, NULL, NULL);

	// inf norm residual
	inf_norm_res[0] = workspace.qp_res[0];
	inf_norm_res[1] = workspace.qp_res[1];
	inf_norm_res[2] = workspace.qp_res[2];
	inf_norm_res[3] = workspace.qp_res[3];

	// stat
	for(jj=0; jj<workspace.iter; jj++)
		for(ii=0; ii<5; ii++)
			stat[ii+5*jj] = workspace.stat[ii+5*jj];



	*kkk = (double) workspace.iter;


	free(ptr_idx);
	free(dim_mem);
	free(qp_mem);
	free(qp_sol_mem);
	free(ipm_arg_mem);
	free(ipm_mem);


	return;

	}

