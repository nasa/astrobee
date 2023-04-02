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



#if defined(RUNTIME_CHECKS)
#include <stdlib.h>
#include <stdio.h>
#endif

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include "../include/hpipm_d_core_qp_ipm.h"
#include "../include/hpipm_d_core_qp_ipm_aux.h"
#include "../include/hpipm_d_rk_int.h"
#include "../include/hpipm_d_erk_int.h"
#include "../include/hpipm_d_ocp_qp.h"
#include "../include/hpipm_d_ocp_qp_sol.h"
#include "../include/hpipm_d_ocp_qp_ipm.h"
#include "../include/hpipm_d_ocp_qp_kkt.h"
#include "../include/hpipm_d_ocp_nlp.h"
#include "../include/hpipm_d_ocp_nlp_sol.h"
#include "../include/hpipm_d_ocp_nlp_aux.h"
#include "../include/hpipm_d_ocp_nlp_hyb.h"
#include "../include/hpipm_d_part_cond.h"



#define AXPY_LIBSTR blasfeo_daxpy
#define GEMV_T_LIBSTR blasfeo_dgemv_t
#define ROWIN_LIBSTR blasfeo_drowin
#define SYMV_L_LIBSTR blasfeo_dsymv_l
#define VECEX_SP_LIBSTR blasfeo_dvecex_sp

#define COMPUTE_ALPHA_QP d_compute_alpha_qp
#define COMPUTE_CENTERING_CORRECTION_QP d_compute_centering_correction_qp
#define COMPUTE_MU_AFF_QP d_compute_mu_aff_qp
#define COMPUTE_RES_OCP_QP d_compute_res_ocp_qp
#define CORE_QP_IPM_WORKSPACE d_core_qp_ipm_workspace
#define CREATE_ERK_INT d_create_erk_int
#define CREATE_OCP_QP d_create_ocp_qp
#define CREATE_OCP_QP_IPM d_create_ocp_qp_ipm
#define CREATE_OCP_QP_SOL d_create_ocp_qp_sol
#define CREATE_STRVEC blasfeo_create_dvec
#define ERK_ARG d_erk_arg
#define ERK_WORKSPACE d_erk_workspace
#define FACT_SOLVE_KKT_STEP_OCP_QP d_fact_solve_kkt_step_ocp_qp
#define INIT_VAR_OCP_QP d_init_var_ocp_qp
#define MEMSIZE_ERK_INT d_memsize_erk_int
#define MEMSIZE_OCP_QP d_memsize_ocp_qp
#define MEMSIZE_OCP_QP_IPM d_memsize_ocp_qp_ipm
#define MEMSIZE_OCP_QP_SOL d_memsize_ocp_qp_sol
#define OCP_NLP d_ocp_nlp
#define OCP_NLP_HYB_ARG d_ocp_nlp_hyb_arg
#define OCP_NLP_SOL d_ocp_nlp_sol
#define OCP_NLP_HYB_WORKSPACE d_ocp_nlp_hyb_workspace
#define OCP_QP d_ocp_qp
#define OCP_QP_IPM_ARG d_ocp_qp_ipm_arg
#define OCP_QP_IPM_WORKSPACE d_ocp_qp_ipm_workspace
#define OCP_QP_SOL d_ocp_qp_sol
#define REAL double
#define SIZE_STRVEC blasfeo_memsize_dvec
#define SOLVE_KKT_STEP_OCP_QP d_solve_kkt_step_ocp_qp
#define STRVEC blasfeo_dvec
#define UPDATE_VAR_QP d_update_var_qp

#define MEMSIZE_OCP_NLP_HYB d_memsize_ocp_nlp_hyb
#define CREATE_OCP_NLP_HYB d_create_ocp_nlp_hyb
#define SOLVE_OCP_NLP_HYB d_solve_ocp_nlp_hyb



hpipm_size_t d_memsize_ocp_nlp_hyb_arg(struct OCP_NLP *nlp)
	{

	int N = nlp->N;

	hpipm_size_t size;

	size = 0;

	size += 5*(N+1)*sizeof(int);

	size += 1*sizeof(struct d_ocp_qp_ipm_arg);

	struct d_ocp_qp qp;
	qp.N  = nlp->N;
	qp.nx = nlp->nx;
	qp.nu = nlp->nu;
	qp.nb = nlp->nb;
	qp.ng = nlp->ng;
	qp.ns = nlp->ns;

	size += d_memsize_ocp_qp_ipm_arg(&qp);

	return size;

	}



void d_create_ocp_nlp_hyb_arg(struct OCP_NLP *nlp, struct OCP_NLP_HYB_ARG *arg, void *mem)
	{

	int N = nlp->N;

	struct d_ocp_qp_ipm_arg *ipm_ptr = (struct d_ocp_qp_ipm_arg *) mem;

	//
	arg->ipm_arg = ipm_ptr;
	ipm_ptr += 1;

	int *i_ptr = (int *) ipm_ptr;

	//
	arg->nx2 = i_ptr;
	i_ptr += N+1;
	//
	arg->nu2 = i_ptr;
	i_ptr += N+1;
	//
	arg->nb2 = i_ptr;
	i_ptr += N+1;
	//
	arg->ng2 = i_ptr;
	i_ptr += N+1;
	//
	arg->ns2 = i_ptr;
	i_ptr += N+1;

	char *c_ptr = (char *) i_ptr;

	struct d_ocp_qp qp;
	qp.N  = nlp->N;
	qp.nx = nlp->nx;
	qp.nu = nlp->nu;
	qp.nb = nlp->nb;
	qp.ng = nlp->ng;
	qp.ns = nlp->ns;

	//
	d_create_ocp_qp_ipm_arg(&qp, arg->ipm_arg, c_ptr);
	c_ptr += arg->ipm_arg->memsize;

	// XXX default value for N2 !!!
	arg->N2 = N; 

	return;

	}



void d_set_default_ocp_nlp_hyb_arg(struct OCP_NLP_HYB_ARG *arg)
	{

	arg->alpha_min = 1e-8;
	arg->nlp_res_g_max = 1e-8;
	arg->nlp_res_b_max = 1e-8;
	arg->nlp_res_d_max = 1e-8;
	arg->nlp_res_m_max = 1e-8;
	arg->nlp_iter_max = 20;
	arg->stat_max = 20;
//	arg->N2 = 1;
	arg->pred_corr = 1;

	d_set_default_ocp_qp_ipm_arg(arg->ipm_arg);
	arg->ipm_arg->res_g_max = 1e-1;
	arg->ipm_arg->res_b_max = 1e-1;
	arg->ipm_arg->res_d_max = 1e-1;
	arg->ipm_arg->res_m_max = 1e-0;
	arg->ipm_arg->mu0 = 1000.0;

	return;

	}



// TODO eliminate x0 in QP !!!
hpipm_size_t MEMSIZE_OCP_NLP_HYB(struct OCP_NLP *nlp, struct OCP_NLP_HYB_ARG *arg)
	{

	int ii;

	int N = nlp->N;
	int *nx = nlp->nx;
	int *nu = nlp->nu;
	int *nb = nlp->nb;
	int *ng = nlp->ng;
	int *ns = nlp->ns;
	int **idxb = nlp->idxb;

	int N2 = arg->N2;
	int *nx2 = arg->nx2;
	int *nu2 = arg->nu2;
	int *nb2 = arg->nb2;
	int *ng2 = arg->ng2;
	int *ns2 = arg->ns2;
	if(N2<N)
		d_compute_qp_size_ocp2ocp(N, nx, nu, nb, idxb, ng, ns, N2, nx2, nu2, nb2, ng2, ns2);

	int nuxM = 0;
	int nbgM = 0;
	for(ii=0; ii<=N; ii++)
		{
		nuxM = nu[ii]+nx[ii]>nuxM ? nu[ii]+nx[ii] : nuxM;
		nbgM = nb[ii]+ng[ii]>nuxM ? nb[ii]+ng[ii] : nbgM;
		}
	
//	int tmp_nx0 = 0;

	int *i_ptr;

	hpipm_size_t size = 0;

	size += 1*sizeof(struct OCP_QP);
	size += 1*sizeof(struct OCP_QP_SOL);
	size += 1*sizeof(struct OCP_QP_IPM_WORKSPACE);
	size += N*sizeof(struct ERK_WORKSPACE);

	size += MEMSIZE_OCP_QP(N, nx, nu, nb, ng, ns);
	size += MEMSIZE_OCP_QP_SOL(N, nx, nu, nb, ng, ns);

	struct OCP_QP qp;
	qp.N = nlp->N;
	qp.nx = nlp->nx;
	qp.nu = nlp->nu;
	qp.nb = nlp->nb;
	qp.ng = nlp->ng;
	qp.ns = nlp->ns;

	size += MEMSIZE_OCP_QP_IPM(&qp, arg->ipm_arg);

	if(N2<N)
		{

		size += 1*sizeof(struct OCP_QP);
		size += 1*sizeof(struct OCP_QP_SOL);
		size += 1*sizeof(struct OCP_QP_IPM_WORKSPACE);
		size += 1*sizeof(struct d_cond_qp_ocp2ocp_workspace);

		size += MEMSIZE_OCP_QP(N2, nx2, nu2, nb2, ng2, ns2);
		size += MEMSIZE_OCP_QP_SOL(N2, nx2, nu2, nb2, ng2, ns2);

		struct OCP_QP qp2;
		qp2.N = N2;
		qp2.nx = nx2;
		qp2.nu = nu2;
		qp2.nb = nb2;
		qp2.ng = ng2;
		qp2.ns = ns2;

		size += MEMSIZE_OCP_QP_IPM(&qp2, arg->ipm_arg);
		size += d_memsize_cond_qp_ocp2ocp(&qp, &qp2);
		}

	for(ii=0; ii<N; ii++)
		{
		size += MEMSIZE_ERK_INT(arg->erk_arg+ii, nx[ii], nu[ii], nx[ii]+nu[ii], 1);
		}

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



// TODO eliminate x0 in QP !!!
void CREATE_OCP_NLP_HYB(struct OCP_NLP *nlp, struct OCP_NLP_HYB_ARG *arg, struct OCP_NLP_HYB_WORKSPACE *ws, void *mem)
	{

	int ii, jj;

	int N = nlp->N;
	int *nx = nlp->nx;
	int *nu = nlp->nu;
	int *nb = nlp->nb;
	int *ng = nlp->ng;
	int *ns = nlp->ns;
	int **idxb = nlp->idxb;

	int N2 = arg->N2;
	int *nx2 = arg->nx2;
	int *nu2 = arg->nu2;
	int *nb2 = arg->nb2;
	int *ng2 = arg->ng2;
	int *ns2 = arg->ns2;
	if(N2<N)
		d_compute_qp_size_ocp2ocp(N, nx, nu, nb, idxb, ng, ns, N2, nx2, nu2, nb2, ng2, ns2);

	int nuxM = 0;
	int nbgM = 0;
	for(ii=0; ii<=N; ii++)
		{
		nuxM = nu[ii]+nx[ii]>nuxM ? nu[ii]+nx[ii] : nuxM;
		nbgM = nb[ii]+ng[ii]>nuxM ? nb[ii]+ng[ii] : nbgM;
		}

	// ocp qp
	struct OCP_QP *qp_ptr = mem;
	//
	ws->qp = qp_ptr;
	qp_ptr += 1;
	if(N2<N)
		{
		//
		ws->qp2 = qp_ptr;
		qp_ptr += 1;
		}

	// ocp qp sol
	struct OCP_QP_SOL *qp_sol_ptr = (struct OCP_QP_SOL *) qp_ptr;
	//
	ws->qp_sol = qp_sol_ptr;
	qp_sol_ptr += 1;
	if(N2<N)
		{
		//
		ws->qp_sol2 = qp_sol_ptr;
		qp_sol_ptr += 1;
		}
	
	// cond qp ocp2ocp ws
	struct d_cond_qp_ocp2ocp_workspace *cond_ptr = (struct d_cond_qp_ocp2ocp_workspace *) qp_sol_ptr;
	if(N2<N)
		{
		//
		ws->part_cond_workspace = cond_ptr;
		cond_ptr += 1;
		}

	// ocp qp ipm ws
	struct OCP_QP_IPM_WORKSPACE *ipm_ws_ptr = (struct OCP_QP_IPM_WORKSPACE *) cond_ptr;
	//
	ws->ipm_workspace = ipm_ws_ptr;
	ipm_ws_ptr += 1;
	if(N2<N)
		{
		//
		ws->ipm_workspace2 = ipm_ws_ptr;
		ipm_ws_ptr += 1;
		}

	// erk ws
	struct ERK_WORKSPACE *erk_ws_ptr = (struct ERK_WORKSPACE *) ipm_ws_ptr;
	//
	ws->erk_workspace = erk_ws_ptr;
	erk_ws_ptr += N;

	// void stuf
	char *c_ptr = (char *) erk_ws_ptr;

	//
	CREATE_OCP_QP(N, nx, nu, nb, ng, ns, ws->qp, c_ptr);
	c_ptr += ws->qp->memsize;
	//
	CREATE_OCP_QP_SOL(N, nx, nu, nb, ng, ns, ws->qp_sol, c_ptr);
	c_ptr += ws->qp_sol->memsize;
	//
	CREATE_OCP_QP_IPM(ws->qp, arg->ipm_arg, ws->ipm_workspace, c_ptr);
	c_ptr += ws->ipm_workspace->memsize;
	if(N2<N)
		{
		//
		CREATE_OCP_QP(N2, nx2, nu2, nb2, ng2, ns2, ws->qp2, c_ptr);
		c_ptr += ws->qp2->memsize;
		//
		CREATE_OCP_QP_SOL(N2, nx2, nu2, nb2, ng2, ns2, ws->qp_sol2, c_ptr);
		c_ptr += ws->qp_sol2->memsize;
		//
		d_create_cond_qp_ocp2ocp(ws->qp, ws->qp2, ws->part_cond_workspace, c_ptr);
		c_ptr += ws->part_cond_workspace->memsize;
		//
		CREATE_OCP_QP_IPM(ws->qp2, arg->ipm_arg, ws->ipm_workspace2, c_ptr);
		c_ptr += ws->ipm_workspace2->memsize;
		}
	for(ii=0; ii<N; ii++)
		{
		//
		CREATE_ERK_INT(arg->erk_arg+ii, nx[ii], nu[ii], nx[ii]+nu[ii], 1, ws->erk_workspace+ii, c_ptr);
		c_ptr += (ws->erk_workspace+ii)->memsize;
		}
	
	//
	REAL **dp_ptr = (REAL **) c_ptr;


	ws->memsize = MEMSIZE_OCP_NLP_HYB(nlp, arg);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + ws->memsize)
		{
		printf("\nCreate_ocp_nlp_sqp: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



int SOLVE_OCP_NLP_HYB(struct OCP_NLP *nlp, struct OCP_NLP_SOL *nlp_sol, struct OCP_NLP_HYB_ARG *arg, struct OCP_NLP_HYB_WORKSPACE *ws)
	{

	struct OCP_QP *qp = ws->qp;
	struct OCP_QP *qp2 = ws->qp2;
	struct OCP_QP_SOL *qp_sol = ws->qp_sol;
	struct OCP_QP_SOL *qp_sol2 = ws->qp_sol2;
	struct d_cond_qp_ocp2ocp_workspace *part_cond_ws = ws->part_cond_workspace;
	struct OCP_QP_IPM_WORKSPACE *ipm_ws = ws->ipm_workspace;
	struct OCP_QP_IPM_WORKSPACE *ipm_ws2 = ws->ipm_workspace2;
	struct ERK_WORKSPACE *erk_ws = ws->erk_workspace;

	struct CORE_QP_IPM_WORKSPACE *cws = ipm_ws->core_workspace;

	struct OCP_QP_IPM_ARG *ipm_arg = arg->ipm_arg;
	struct ERK_ARG *erk_arg = arg->erk_arg;

	// alias qp vectors into qp_sol
	cws->v = qp_sol->ux->pa;
	cws->pi = qp_sol->pi->pa;
	cws->lam = qp_sol->lam->pa;
	cws->t = qp_sol->t->pa;

	ipm_ws->mu0 = arg->ipm_arg->mu0;

	int ss, nn, ii;

	// qp size
	int N = qp->N;
	int *nx = qp->nx;
	int *nu = qp->nu;
	int *nb = qp->nb;
	int *ng = qp->ng;
	int *ns = qp->ns;

	int N2 = arg->N2;
#if 1
	int *nx2 = qp2->nx;
	int *nu2 = qp2->nu;
	int *nb2 = qp2->nb;
	int *ng2 = qp2->ng;
	int *ns2 = qp2->ns;
#endif


#if 0
for(ii=0; ii<=N2; ii++)
	printf("\n%d %d %d %d %d\n", nx2[ii], nu2[ii], nb2[ii], ng2[ii], ns2[ii]);
exit(1);
#endif

	double *x, *u, *pi;

	struct STRVEC str_res_g;
	struct STRVEC str_res_b;
	struct STRVEC str_res_d;
	struct STRVEC str_res_m;
	str_res_g.m = cws->nv;
	str_res_b.m = cws->ne;
	str_res_d.m = cws->nc;
	str_res_m.m = cws->nc;
	str_res_g.pa = cws->res_g;
	str_res_b.pa = cws->res_b;
	str_res_d.pa = cws->res_d;
	str_res_m.pa = cws->res_m;

	struct CORE_QP_IPM_WORKSPACE *cws2;
	struct STRVEC str_res_g2;
	struct STRVEC str_res_b2;
	struct STRVEC str_res_d2;
	struct STRVEC str_res_m2;
	if(N2<N)
		{
		cws2 = ipm_ws2->core_workspace;
		str_res_g2.m = cws2->nv;
		str_res_b2.m = cws2->ne;
		str_res_d2.m = cws2->nc;
		str_res_m2.m = cws2->nc;
		str_res_g2.pa = cws2->res_g;
		str_res_b2.pa = cws2->res_b;
		str_res_d2.pa = cws2->res_d;
		str_res_m2.pa = cws2->res_m;
		}

	double nlp_res[4];

	double tmp;


	// initialize solution (to zero atm)
	for(nn=0; nn<=N; nn++)
		blasfeo_dvecse(nu[nn]+nx[nn], 0.0, nlp_sol->ux+nn, 0);
	for(nn=0; nn<N; nn++)
		blasfeo_dvecse(nx[nn+1], 0.0, nlp_sol->pi+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dvecse(2*nb[nn]+2*ng[nn], 0.0, nlp_sol->lam+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dvecse(2*nb[nn]+2*ng[nn], 0.0, nlp_sol->t+nn, 0);

	// 1 sqp iteration

	// copy nlp into qp
	nn = 0;
	for(; nn<=N; nn++)
		{
		blasfeo_dgecp(nu[nn]+nx[nn], nu[nn]+nx[nn], nlp->RSQ+nn, 0, 0, qp->RSQrq+nn, 0, 0);
		blasfeo_dgecp(nu[nn]+nx[nn], ng[nn], nlp->DCt+nn, 0, 0, qp->DCt+nn, 0, 0);
		for(ii=0; ii<nb[nn]; ii++) qp->idxb[nn][ii] = nlp->idxb[nn][ii];
		for(ii=0; ii<ns[nn]; ii++) qp->idxs[nn][ii] = nlp->idxs[nn][ii];
		}


	// simulation & sensitivity propagation
	for(nn=0; nn<N; nn++)
		{
		x  = (nlp_sol->ux+nn)->pa+nu[nn];
		u  = (nlp_sol->ux+nn)->pa;
		pi = (nlp_sol->pi+nn)->pa;
		d_init_erk_int(nx[nn]+nu[nn], 0, x, u, (nlp->model+nn)->forward_seed, NULL, (nlp->model+nn)->expl_vde_for, NULL, (nlp->model+nn)->arg, erk_ws+nn);
		d_erk_int(erk_ws+nn);
		// setup BAbt & b
		d_cvt_erk_int_to_ocp_qp(nn, erk_ws+nn, qp, nlp_sol);
		}


	// setup gradient
	for(nn=0; nn<=N; nn++)
		blasfeo_dveccp(nu[nn]+nx[nn], nlp->rq+nn, 0, qp->rq+nn, 0);
	// setup constraints
	for(nn=0; nn<=N; nn++)
		blasfeo_dveccp(2*nb[nn]+2*ng[nn], nlp->d+nn, 0, qp->d+nn, 0);


#if 0
for(nn=0; nn<=N; nn++)
	d_print_strmat(nu[nn]+nx[nn]+1, nu[nn]+nx[nn], qp->RSQrq+nn, 0, 0);
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(nu[nn]+nx[nn], qp->rq+nn, 0);
for(nn=0; nn<N; nn++)
	d_print_strmat(nu[nn]+nx[nn]+1, nx[nn+1], qp->BAbt+nn, 0, 0);
for(nn=0; nn<N; nn++)
	blasfeo_print_tran_dvec(nx[nn+1], qp->b+nn, 0);
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(2*nb[nn]+2*ng[nn]+2*ns[nn], qp->d+nn, 0);
//exit(1);
#endif


#if 0
printf("\n%d %d\n", nu[0], nx[0]);
exit(1);
#endif

	// copy nlp_sol into qp_sol
	for(nn=0; nn<=N; nn++)
		blasfeo_dveccp(nu[nn]+nx[nn]+2*ns[ii], nlp_sol->ux+nn, 0, qp_sol->ux+nn, 0);
	for(nn=0; nn<N; nn++)
		blasfeo_dveccp(nx[nn+1], nlp_sol->pi+nn, 0, qp_sol->pi+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[ii], nlp_sol->lam+nn, 0, qp_sol->lam+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[ii], nlp_sol->t+nn, 0, qp_sol->t+nn, 0);


//	// compute residuals
//	COMPUTE_RES_OCP_QP(qp, qp_sol, ipm_ws); // gests own workspace ???
//	cws->mu = ipm_ws->res_mu;

	// compute infinity norm of residuals
//	blasfeo_dvecnrm_inf(cws->nv, &str_res_g, 0, &nlp_res[0]);
//	blasfeo_dvecnrm_inf(cws->ne, &str_res_b, 0, &nlp_res[1]);
//	blasfeo_dvecnrm_inf(cws->nc, &str_res_d, 0, &nlp_res[2]);
//	blasfeo_dvecnrm_inf(cws->nc, &str_res_m, 0, &nlp_res[3]);

#if 0
printf("\nresiduals\n");
d_print_exp_mat(1, cws->nv, cws->res_g, 1);
d_print_exp_mat(1, cws->ne, cws->res_b, 1);
d_print_exp_mat(1, cws->nc, cws->res_d, 1);
d_print_exp_mat(1, cws->nc, cws->res_m, 1);
#endif

#if 0
printf("\n\niter %d nlp inf norm res %e %e %e %e\n", ss, nlp_res[0], nlp_res[1], nlp_res[2], nlp_res[3]);
#endif

//	// exit condition on residuals
//	if(!(nlp_res[0]>arg->nlp_res_g_max | nlp_res[1]>arg->nlp_res_b_max | nlp_res[2]>arg->nlp_res_d_max | nlp_res[3]>arg->nlp_res_m_max))
//		{
//		ws->iter = ss;
//		ws->nlp_res_g = nlp_res[0];
//		ws->nlp_res_b = nlp_res[1];
//		ws->nlp_res_d = nlp_res[2];
//		ws->nlp_res_m = nlp_res[3];
//		return 0;
//		}


	// update RHS
	for(nn=0; nn<=N; nn++)
		{

		// gradient
		SYMV_L_LIBSTR(nu[nn]+nx[nn], nu[nn]+nx[nn], 1.0, qp->RSQrq+nn, 0, 0, qp_sol->ux+nn, 0, 1.0, qp->rq+nn, 0, qp->rq+nn, 0);
		ROWIN_LIBSTR(nu[nn]+nx[nn], 1.0, qp->rq+nn, 0, qp->RSQrq+nn, nu[nn]+nx[nn], 0);

		// constraints
		if(nb[nn]+ng[nn]>0)
			{
			// box
			if(nb[nn]>0)
				{
				VECEX_SP_LIBSTR(nb[nn], 1.0, qp->idxb[nn], qp_sol->ux+nn, 0, ipm_ws->tmp_nbgM, 0);
				}
			// general
			if(ng[nn]>0)
				{
				GEMV_T_LIBSTR(nu[nn]+nx[nn], ng[nn], 1.0, qp->DCt+nn, 0, 0, qp_sol->ux+nn, 0, 0.0, ipm_ws->tmp_nbgM, nb[nn], ipm_ws->tmp_nbgM, nb[nn]);
				}

			AXPY_LIBSTR(nb[nn]+ng[nn], -1.0, ipm_ws->tmp_nbgM, 0, qp->d+nn, 0, qp->d+nn, 0);
			AXPY_LIBSTR(nb[nn]+ng[nn], -1.0, ipm_ws->tmp_nbgM, 0, qp->d+nn, nb[nn]+ng[nn], qp->d+nn, nb[nn]+ng[nn]);
			}
		if(ns[nn]>0)
			{
			// TODO soft constraints
			}

		// dynamics
		if(nn<N)
			{

			AXPY_LIBSTR(nx[nn+1], -1.0, qp_sol->ux+(nn+1), nu[nn+1], qp->b+nn, 0, qp->b+nn, 0);

			GEMV_T_LIBSTR(nu[nn]+nx[nn], nx[nn+1], 1.0, qp->BAbt+nn, 0, 0, qp_sol->ux+nn, 0, 1.0, qp->b+nn, 0, qp->b+nn, 0);
			ROWIN_LIBSTR(nx[nn+1], 1.0, qp->b+nn, 0, qp->BAbt+nn, nu[nn]+nx[nn], 0);

			}

		}


#if 0
printf("\nqp data\n");
for(nn=0; nn<=N; nn++)
	d_print_strmat(nu[nn]+nx[nn]+1, nu[nn]+nx[nn], qp->RSQrq+nn, 0, 0);
for(nn=0; nn<N; nn++)
	d_print_strmat(nu[nn]+nx[nn]+1, nx[nn+1], qp->BAbt+nn, 0, 0);
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(2*nb[nn]+2*ng[nn]+2*ns[nn], qp->d+nn, 0);
exit(1);
#endif



	if(N2<N)
		{

		// part cond
		d_cond_qp_ocp2ocp(qp, qp2, part_cond_ws);

#if 0
printf("\nqp data\n");
for(nn=0; nn<=N2; nn++)
	d_print_strmat(nu2[nn]+nx2[nn]+1, nu2[nn]+nx2[nn], qp2->RSQrq+nn, 0, 0);
for(nn=0; nn<N2; nn++)
	d_print_strmat(nu2[nn]+nx2[nn]+1, nx2[nn+1], qp2->BAbt+nn, 0, 0);
for(nn=0; nn<=N2; nn++)
	blasfeo_print_tran_dvec(2*nb2[nn]+2*ng2[nn]+2*ns2[nn], qp2->d+nn, 0);
exit(1);
#endif

		// solve qp
		d_solve_ocp_qp_ipm(qp2, qp_sol2, ipm_arg, ipm_ws2);
		ws->iter_qp = ipm_ws2->iter;
		for(ii=0; ii<5*ws->iter_qp; ii++)
			ipm_ws->stat[ii] = ipm_ws2->stat[ii];

#if 0
printf("\nqp sol\n");
for(nn=0; nn<=N2; nn++)
	blasfeo_print_tran_dvec(nu2[nn]+nx2[nn]+2*ns2[nn], qp_sol2->ux+nn, 0);
for(nn=0; nn<N2; nn++)
	blasfeo_print_tran_dvec(nx2[nn+1], qp_sol2->pi+nn, 0);
for(nn=0; nn<=N2; nn++)
	blasfeo_print_tran_dvec(2*nb2[nn]+2*ng2[nn]+2*ns2[nn], qp_sol2->lam+nn, 0);
for(nn=0; nn<=N2; nn++)
	blasfeo_print_tran_dvec(2*nb2[nn]+2*ng2[nn]+2*ns2[nn], qp_sol2->t+nn, 0);
d_print_exp_tran_mat(5, ipm_ws->iter, ipm_ws->stat, 5);
exit(1);
#endif

		// part expand
		d_expand_sol_ocp2ocp(qp, qp2, qp_sol2, qp_sol, part_cond_ws);

		}
	else
		{

		// solve qp
		d_solve_ocp_qp_ipm(qp, qp_sol, ipm_arg, ipm_ws);
		ws->iter_qp = ipm_ws->iter;

		}


#if 0
printf("\nqp sol\n");
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(nu[nn]+nx[nn]+2*ns[nn], qp_sol->ux+nn, 0);
for(nn=0; nn<N; nn++)
	blasfeo_print_tran_dvec(nx[nn+1], qp_sol->pi+nn, 0);
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(2*nb[nn]+2*ng[nn]+2*ns[nn], qp_sol->lam+nn, 0);
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(2*nb[nn]+2*ng[nn]+2*ns[nn], qp_sol->t+nn, 0);
d_print_exp_tran_mat(5, ipm_ws->iter, ipm_ws->stat, 5);
exit(1);
#endif

	// update primal variables (full step)
	for(nn=0; nn<=N; nn++)
		blasfeo_daxpy(nu[nn]+nx[nn], 1.0, qp_sol->ux+nn, 0, nlp_sol->ux+nn, 0, nlp_sol->ux+nn, 0);
	// copy dual multipliers
	for(nn=0; nn<N; nn++)
		blasfeo_dveccp(nx[nn+1], qp_sol->pi+nn, 0, nlp_sol->pi+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dveccp(2*nb[nn]+2*ng[nn], qp_sol->lam+nn, 0, nlp_sol->lam+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dveccp(2*nb[nn]+2*ng[nn], qp_sol->t+nn, 0, nlp_sol->t+nn, 0);

#if 0
printf("\nnlp sol\n");
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(nlp->nu[nn]+nlp->nx[nn]+2*nlp->ns[nn], nlp_sol->ux+nn, 0);
for(nn=0; nn<N; nn++)
	blasfeo_print_tran_dvec(nlp->nx[nn+1], nlp_sol->pi+nn, 0);
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(2*nlp->nb[nn]+2*nlp->ng[nn]+2*nlp->ns[nn], nlp_sol->lam+nn, 0);
for(nn=0; nn<=N; nn++)
	blasfeo_print_tran_dvec(2*nlp->nb[nn]+2*nlp->ng[nn]+2*nlp->ns[nn], nlp_sol->t+nn, 0);
//d_print_exp_tran_mat(5, ipm_ws->iter, ipm_ws->stat, 5);
//		exit(1);
#endif

	// warm-started IPM
	
	// copy nlp into qp rhs
	nn = 0;
	for(; nn<=N; nn++)
		{
//		blasfeo_dgecp(nu[nn]+nx[nn], nu[nn]+nx[nn], nlp->RSQ+nn, 0, 0, qp->RSQrq+nn, 0, 0);
//		blasfeo_dgecp(nu[nn]+nx[nn], ng[nn], nlp->DCt+nn, 0, 0, qp->DCt+nn, 0, 0);
		blasfeo_dveccp(nu[nn]+nx[nn], nlp->rq+nn, 0, qp->rq+nn, 0);
		blasfeo_drowin(nu[nn]+nx[nn], 1.0, qp->rq+nn, 0, qp->RSQrq+nn, nu[nn]+nx[nn], 0);
		blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[nn], nlp->d+nn, 0, qp->d+nn, 0);
//		for(ii=0; ii<nb[nn]; ii++) qp->idxb[nn][ii] = nlp->idxb[nn][ii];
//		for(ii=0; ii<ns[nn]; ii++) qp->idxs[nn][ii] = nlp->idxs[nn][ii];
		}


	// initialize solution
//	INIT_VAR_OCP_QP(qp, qp_sol, ipm_ws);


	// nlp loop
	for(ss=0; ss<arg->nlp_iter_max; ss++)	
		{

		for(nn=0; nn<=N; nn++)
			{
			blasfeo_dveccp(nu[nn]+nx[nn], nlp->rq+nn, 0, qp->rq+nn, 0);
//			blasfeo_drowin(nu[nn]+nx[nn], 1.0, qp->rq+nn, 0, qp->RSQrq+nn, nu[nn]+nx[nn], 0);
			}

		// simulation & sensitivity propagation
		if(ss<=0)
//		if(1)
			{
			for(nn=0; nn<N; nn++)
				{
				x  = (nlp_sol->ux+nn)->pa+nu[nn];
				u  = (nlp_sol->ux+nn)->pa;
				pi = (nlp_sol->pi+nn)->pa;
				d_init_erk_int(nx[nn]+nu[nn], 0, x, u, (nlp->model+nn)->forward_seed, NULL, (nlp->model+nn)->expl_vde_for, NULL, (nlp->model+nn)->arg, erk_ws+nn);
				d_erk_int(erk_ws+nn);
				d_cvt_erk_int_to_ocp_qp(nn, erk_ws+nn, qp, nlp_sol);
				}
			// TODO partial condensing
			if(N2<N)
				{
				d_cond_qp_ocp2ocp(qp, qp2, part_cond_ws);
				}
			}
		else
			{
			for(nn=0; nn<N; nn++)
				{
				x  = (nlp_sol->ux+nn)->pa+nu[nn];
				u  = (nlp_sol->ux+nn)->pa;
				pi = (nlp_sol->pi+nn)->pa;
//				d_init_erk_int(0, 1, x, u, NULL, pi, (nlp->model+nn)->expl_vde_for, (nlp->model+nn)->expl_vde_adj, (nlp->model+nn)->arg, erk_ws+nn);
				d_init_erk_int(0, 1, x, u, NULL, pi, (nlp->model+nn)->expl_ode, (nlp->model+nn)->expl_vde_adj, (nlp->model+nn)->arg, erk_ws+nn);
				d_erk_int(erk_ws+nn);
				d_cvt_erk_int_to_ocp_qp_rhs(nn, erk_ws+nn, qp, nlp_sol);
				}
			// TODO partial condensing rhs
			if(N2<N)
				{
				d_cond_qp_ocp2ocp(qp, qp2, part_cond_ws);
//				d_cond_rhs_qp_ocp2ocp(qp, qp2, part_cond_ws);
				}
			}

// TODO compare qp_sol with qp_sol2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//for(ii=0; ii<N; ii++)
//	blasfeo_print_exp_dmat(nlp->nu[ii]+nlp->nx[ii]+1, nlp->nx[ii+1], qp->BAbt+ii, 0, 0);
	
		if(N2<N) // partially condensed space solution
//		if(0) // partially condensed space solution
			{

#if 0
printf("\nsol\n");
d_print_exp_mat(1, cws2->nv, cws2->v, 1);
d_print_exp_mat(1, cws2->ne, cws2->pi, 1);
d_print_exp_mat(1, cws2->nc, cws2->lam, 1);
d_print_exp_mat(1, cws2->nc, cws2->t, 1);
exit(1);
#endif

			// compute residuals
			COMPUTE_RES_OCP_QP(qp2, qp_sol2, ipm_ws2->res_workspace);
			cws2->mu = ipm_ws2->res_workspace->res_mu;
			if(ss>0 & ws->iter_qp+ss<ipm_ws->stat_max)
				ipm_ws->stat[5*(ws->iter_qp+ss-1)+4] = ipm_ws2->res_workspace->res_mu;

			// compute infinity norm of residuals
			blasfeo_dvecnrm_inf(cws2->nv, &str_res_g2, 0, &nlp_res[0]); // XXX
			blasfeo_dvecnrm_inf(cws2->ne, &str_res_b2, 0, &nlp_res[1]); // XXX
			blasfeo_dvecnrm_inf(cws2->nc, &str_res_d2, 0, &nlp_res[2]); // XXX
			blasfeo_dvecnrm_inf(cws2->nc, &str_res_m2, 0, &nlp_res[3]); // XXX

//printf("\n%e %e %e %e\n", nlp_res[0], nlp_res[1], nlp_res[2], nlp_res[3]);

#if 0
printf("\nresiduals\n");
d_print_exp_mat(1, cws2->nv, cws2->res_g, 1);
d_print_exp_mat(1, cws2->ne, cws2->res_b, 1);
d_print_exp_mat(1, cws2->nc, cws2->res_d, 1);
d_print_exp_mat(1, cws2->nc, cws2->res_m, 1);
exit(1);
#endif

			// exit condition on residuals
			if(!(nlp_res[0]>arg->nlp_res_g_max | nlp_res[1]>arg->nlp_res_b_max | nlp_res[2]>arg->nlp_res_d_max | nlp_res[3]>arg->nlp_res_m_max))
				{

				// part expand
				d_expand_sol_ocp2ocp(qp, qp2, qp_sol2, qp_sol, part_cond_ws);

				// compute residuals (full space)
				COMPUTE_RES_OCP_QP(qp, qp_sol, ipm_ws->res_workspace);

				// compute infinity norm of residuals
				blasfeo_dvecnrm_inf(cws->nv, &str_res_g, 0, &nlp_res[0]);
				blasfeo_dvecnrm_inf(cws->ne, &str_res_b, 0, &nlp_res[1]);
				blasfeo_dvecnrm_inf(cws->nc, &str_res_d, 0, &nlp_res[2]);
				blasfeo_dvecnrm_inf(cws->nc, &str_res_m, 0, &nlp_res[3]);

				ws->iter_nlp = ss;
				ws->nlp_res_g = nlp_res[0];
				ws->nlp_res_b = nlp_res[1];
				ws->nlp_res_d = nlp_res[2];
				ws->nlp_res_m = nlp_res[3];

				return 0;
				}


			// fact and solve kkt
			FACT_SOLVE_KKT_STEP_OCP_QP(qp2, ipm_ws2);

			// alpha
			COMPUTE_ALPHA_QP(cws2);
			if(ws->iter_qp+ss<ipm_ws->stat_max)
				ipm_ws->stat[5*(ws->iter_qp+ss)+0] = cws2->alpha;

			// Mehrotra's corrector
			if(arg->pred_corr==1)
				{
				// mu_aff
				COMPUTE_MU_AFF_QP(cws2);
				if(ws->iter_qp+ss<ipm_ws->stat_max)
					ipm_ws->stat[5*(ws->iter_qp+ss)+1] = cws2->mu_aff;

				tmp = cws2->mu_aff/cws2->mu;
				cws2->sigma = tmp*tmp*tmp;
				if(ws->iter_qp+ss<ipm_ws->stat_max)
					ipm_ws->stat[5*(ws->iter_qp+ss)+2] = cws2->sigma;

				COMPUTE_CENTERING_CORRECTION_QP(cws2);

				// fact and solve kkt
				SOLVE_KKT_STEP_OCP_QP(qp2, ipm_ws2);

				// alpha
				COMPUTE_ALPHA_QP(cws2);
				if(ws->iter_qp+ss<ipm_ws->stat_max)
					ipm_ws->stat[5*(ws->iter_qp+ss)+3] = cws2->alpha;
				}

			// update QP variables
			UPDATE_VAR_QP(cws2);

			// part expand
			d_expand_sol_ocp2ocp(qp, qp2, qp_sol2, qp_sol, part_cond_ws);

#if 0
printf("\nqp sol\n");
d_print_exp_mat(1, cws->nv, cws->v, 1);
d_print_exp_mat(1, cws->ne, cws->pi, 1);
d_print_exp_mat(1, cws->nc, cws->lam, 1);
d_print_exp_mat(1, cws->nc, cws->t, 1);
#endif

			// update NLP variables
			for(nn=0; nn<=N; nn++)
				blasfeo_dveccp(nu[nn]+nx[nn]+2*ns[ii], qp_sol->ux+nn, 0, nlp_sol->ux+nn, 0);
			for(nn=0; nn<N; nn++)
				blasfeo_dveccp(nx[nn+1], qp_sol->pi+nn, 0, nlp_sol->pi+nn, 0);
			for(nn=0; nn<=N; nn++)
				blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[ii], qp_sol->lam+nn, 0, nlp_sol->lam+nn, 0);
			for(nn=0; nn<=N; nn++)
				blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[ii], qp_sol->t+nn, 0, nlp_sol->t+nn, 0);

			}
		else // full space solution
			{


#if 0
printf("\n%d %d\n", nu[0], nx[0]);
blasfeo_print_tran_dvec(nu[0]+nx[0], qp->rq+0, 0);
blasfeo_print_tran_dvec(nx[1], qp->b+0, 0);
exit(1);
#endif

#if 0
printf("\nsol\n");
d_print_exp_mat(1, cws->nv, cws->v, 1);
d_print_exp_mat(1, cws->ne, cws->pi, 1);
d_print_exp_mat(1, cws->nc, cws->lam, 1);
d_print_exp_mat(1, cws->nc, cws->t, 1);
exit(1);
#endif

			// compute residuals
			COMPUTE_RES_OCP_QP(qp, qp_sol, ipm_ws->res_workspace);
			cws->mu = ipm_ws->res_workspace->res_mu;
			if(ss>0 & ws->iter_qp+ss<ipm_ws->stat_max)
				ipm_ws->stat[5*(ws->iter_qp+ss-1)+4] = ipm_ws->res_workspace->res_mu;

			// compute infinity norm of residuals
			blasfeo_dvecnrm_inf(cws->nv, &str_res_g, 0, &nlp_res[0]);
			blasfeo_dvecnrm_inf(cws->ne, &str_res_b, 0, &nlp_res[1]);
			blasfeo_dvecnrm_inf(cws->nc, &str_res_d, 0, &nlp_res[2]);
			blasfeo_dvecnrm_inf(cws->nc, &str_res_m, 0, &nlp_res[3]);

//printf("\n%e %e %e %e\n", nlp_res[0], nlp_res[1], nlp_res[2], nlp_res[3]);

#if 0
printf("\nresiduals\n");
d_print_exp_mat(1, cws->nv, cws->res_g, 1);
d_print_exp_mat(1, cws->ne, cws->res_b, 1);
d_print_exp_mat(1, cws->nc, cws->res_d, 1);
d_print_exp_mat(1, cws->nc, cws->res_m, 1);
exit(1);
#endif

#if 0
printf("\n\niter %d nlp inf norm res %e %e %e %e\n", ss, nlp_res[0], nlp_res[1], nlp_res[2], nlp_res[3]);
#endif

			// exit condition on residuals
			if(!(nlp_res[0]>arg->nlp_res_g_max | nlp_res[1]>arg->nlp_res_b_max | nlp_res[2]>arg->nlp_res_d_max | nlp_res[3]>arg->nlp_res_m_max))
				{
				ws->iter_nlp = ss;
				ws->nlp_res_g = nlp_res[0];
				ws->nlp_res_b = nlp_res[1];
				ws->nlp_res_d = nlp_res[2];
				ws->nlp_res_m = nlp_res[3];
				return 0;
				}


			// fact and solve kkt
			FACT_SOLVE_KKT_STEP_OCP_QP(qp, ipm_ws);

			// alpha
			COMPUTE_ALPHA_QP(cws);
			if(ws->iter_qp+ss<ipm_ws->stat_max)
				ipm_ws->stat[5*(ws->iter_qp+ss)+0] = cws->alpha;

			// Mehrotra's corrector
			if(arg->pred_corr==1)
				{
				// mu_aff
				COMPUTE_MU_AFF_QP(cws);
				if(ws->iter_qp+ss<ipm_ws->stat_max)
					ipm_ws->stat[5*(ws->iter_qp+ss)+1] = cws->mu_aff;

				tmp = cws->mu_aff/cws->mu;
				cws->sigma = tmp*tmp*tmp;
				if(ws->iter_qp+ss<ipm_ws->stat_max)
					ipm_ws->stat[5*(ws->iter_qp+ss)+2] = cws->sigma;

				COMPUTE_CENTERING_CORRECTION_QP(cws);

				// fact and solve kkt
				SOLVE_KKT_STEP_OCP_QP(qp, ipm_ws);

				// alpha
				COMPUTE_ALPHA_QP(cws);
				if(ws->iter_qp+ss<ipm_ws->stat_max)
					ipm_ws->stat[5*(ws->iter_qp+ss)+3] = cws->alpha;
				}

#if 0
printf("\nstep\n");
d_print_exp_mat(1, cws->nv, cws->dv, 1);
d_print_exp_mat(1, cws->ne, cws->dpi, 1);
d_print_exp_mat(1, cws->nc, cws->dlam, 1);
d_print_exp_mat(1, cws->nc, cws->dt, 1);
#endif
			// update QP variables
			UPDATE_VAR_QP(cws);

#if 0
printf("\nqp sol\n");
d_print_exp_mat(1, cws->nv, cws->v, 1);
d_print_exp_mat(1, cws->ne, cws->pi, 1);
d_print_exp_mat(1, cws->nc, cws->lam, 1);
d_print_exp_mat(1, cws->nc, cws->t, 1);
#endif

#if 0
d_print_exp_tran_mat(5, kk, ipm_ws->stat, 5);
#endif

			// update NLP variables
			for(nn=0; nn<=N; nn++)
				blasfeo_dveccp(nu[nn]+nx[nn]+2*ns[ii], qp_sol->ux+nn, 0, nlp_sol->ux+nn, 0);
			for(nn=0; nn<N; nn++)
				blasfeo_dveccp(nx[nn+1], qp_sol->pi+nn, 0, nlp_sol->pi+nn, 0);
			for(nn=0; nn<=N; nn++)
				blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[ii], qp_sol->lam+nn, 0, nlp_sol->lam+nn, 0);
			for(nn=0; nn<=N; nn++)
				blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[ii], qp_sol->t+nn, 0, nlp_sol->t+nn, 0);

			}

		}
	
	// maximum iteration number reached
	ws->iter_nlp = ss;
	ws->nlp_res_g = nlp_res[0];
	ws->nlp_res_b = nlp_res[1];
	ws->nlp_res_d = nlp_res[2];
	ws->nlp_res_m = nlp_res[3];

	return 1;

	}
