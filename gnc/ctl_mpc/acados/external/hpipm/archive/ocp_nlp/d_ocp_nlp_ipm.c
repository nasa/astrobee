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
#include "../include/hpipm_d_ocp_nlp_ipm.h"



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
//#define FACT_SOLVE_KKT_UNCONSTR_OCP_QP d_fact_solve_kkt_unconstr_ocp_qp
#define INIT_VAR_OCP_QP d_init_var_ocp_qp
#define MEMSIZE_ERK_INT d_memsize_erk_int
#define MEMSIZE_OCP_NLP_IPM d_memsize_ocp_nlp_ipm
#define MEMSIZE_OCP_QP d_memsize_ocp_qp
#define MEMSIZE_OCP_QP_IPM d_memsize_ocp_qp_ipm
#define MEMSIZE_OCP_QP_SOL d_memsize_ocp_qp_sol
#define OCP_NLP d_ocp_nlp
#define OCP_NLP_IPM_ARG d_ocp_nlp_ipm_arg
#define OCP_NLP_SOL d_ocp_nlp_sol
#define OCP_NLP_IPM_WORKSPACE d_ocp_nlp_ipm_workspace
#define OCP_QP d_ocp_qp
#define OCP_QP_IPM_ARG d_ocp_qp_ipm_arg
#define OCP_QP_IPM_WORKSPACE d_ocp_qp_ipm_workspace
#define OCP_QP_SOL d_ocp_qp_sol
#define REAL double
#define SIZE_STRVEC blasfeo_memsize_dvec
#define SOLVE_KKT_STEP_OCP_QP d_solve_kkt_step_ocp_qp
#define STRVEC blasfeo_dvec
#define UPDATE_VAR_QP d_update_var_qp

#define MEMSIZE_OCP_NLP_IPM d_memsize_ocp_nlp_ipm
#define CREATE_OCP_NLP_IPM d_create_ocp_nlp_ipm
#define SOLVE_OCP_NLP_IPM d_solve_ocp_nlp_ipm



hpipm_size_t d_memsize_ocp_nlp_ipm_arg(struct OCP_NLP *nlp)
	{

	int N = nlp->N;

	hpipm_size_t size;

	size = 0;

	return size;

	}



void d_create_ocp_nlp_ipm_arg(struct OCP_NLP *nlp, struct OCP_NLP_IPM_ARG *arg, void *mem)
	{

	arg->memsize = 0;

	return;

	}



void d_set_default_ocp_nlp_ipm_arg(struct OCP_NLP_IPM_ARG *arg)
	{

	arg->alpha_min = 1e-8;
	arg->nlp_res_g_max = 1e-8;
	arg->nlp_res_b_max = 1e-8;
	arg->nlp_res_d_max = 1e-8;
	arg->nlp_res_m_max = 1e-8;
	arg->nlp_iter_max = 20;
	arg->stat_max = 20;
	arg->mu0 = 100.0;
	arg->pred_corr = 1;

	return;

	}



// TODO eliminate x0 in QP !!!
hpipm_size_t MEMSIZE_OCP_NLP_IPM(struct OCP_NLP *nlp, struct OCP_NLP_IPM_ARG *arg)
	{

	int ii;

	int N = nlp->N;
	int *nx = nlp->nx;
	int *nu = nlp->nu;
	int *nb = nlp->nb;
	int *ng = nlp->ng;
	int *ns = nlp->ns;

	int nuxM = 0;
	int nbgM = 0;
	for(ii=0; ii<=N; ii++)
		{
		nuxM = nu[ii]+nx[ii]>nuxM ? nu[ii]+nx[ii] : nuxM;
		nbgM = nb[ii]+ng[ii]>nuxM ? nb[ii]+ng[ii] : nbgM;
		}
	
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
//	qp.idxb = nlp->idxb;
//	qp.idxs = nlp->idxs;

	struct OCP_QP_IPM_ARG ipm_arg;
	ipm_arg.stat_max = arg->stat_max;

	size += MEMSIZE_OCP_QP_IPM(&qp, &ipm_arg);

	for(ii=0; ii<N; ii++)
		{
		size += MEMSIZE_ERK_INT(arg->erk_arg+ii, nx[ii], nu[ii], nx[ii]+nu[ii], 0);
		}

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



// TODO eliminate x0 in QP !!!
void CREATE_OCP_NLP_IPM(struct OCP_NLP *nlp, struct OCP_NLP_IPM_ARG *arg, struct OCP_NLP_IPM_WORKSPACE *ws, void *mem)
	{

	int ii, jj;

	int N = nlp->N;
	int *nx = nlp->nx;
	int *nu = nlp->nu;
	int *nb = nlp->nb;
	int *ng = nlp->ng;
	int *ns = nlp->ns;

	int nuxM = 0;
	int nbgM = 0;
	for(ii=0; ii<=N; ii++)
		{
		nuxM = nu[ii]+nx[ii]>nuxM ? nu[ii]+nx[ii] : nuxM;
		nbgM = nb[ii]+ng[ii]>nuxM ? nb[ii]+ng[ii] : nbgM;
		}

	// tmp ipm arg
	struct OCP_QP_IPM_ARG ipm_arg;
	ipm_arg.stat_max = arg->stat_max;

	// ocp qp
	struct OCP_QP *qp_ptr = mem;
	//
	ws->qp = qp_ptr;
	qp_ptr += 1;

	// ocp qp sol
	struct OCP_QP_SOL *qp_sol_ptr = (struct OCP_QP_SOL *) qp_ptr;
	//
	ws->qp_sol = qp_sol_ptr;
	qp_sol_ptr += 1;

	// ocp qp ipm ws
	struct OCP_QP_IPM_WORKSPACE *ipm_ws_ptr = (struct OCP_QP_IPM_WORKSPACE *) qp_sol_ptr;
	//
	ws->ipm_workspace = ipm_ws_ptr;
	ipm_ws_ptr += 1;

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
	CREATE_OCP_QP_IPM(ws->qp, &ipm_arg, ws->ipm_workspace, c_ptr);
	c_ptr += ws->ipm_workspace->memsize;

	//
	for(ii=0; ii<N; ii++)
		{
		CREATE_ERK_INT(arg->erk_arg+ii, nx[ii], nu[ii], nx[ii]+nu[ii], 0,  ws->erk_workspace+ii, c_ptr);
		c_ptr += (ws->erk_workspace+ii)->memsize;
		}
	
	//
	REAL **dp_ptr = (REAL **) c_ptr;


	ws->memsize = MEMSIZE_OCP_NLP_IPM(nlp, arg);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + ws->memsize)
		{
		printf("\nCreate_ocp_nlp_sqp: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



int SOLVE_OCP_NLP_IPM(struct OCP_NLP *nlp, struct OCP_NLP_SOL *nlp_sol, struct OCP_NLP_IPM_ARG *arg, struct OCP_NLP_IPM_WORKSPACE *ws)
	{

	struct OCP_QP *qp = ws->qp;
	struct OCP_QP_SOL *qp_sol = ws->qp_sol;
	struct OCP_QP_IPM_WORKSPACE *ipm_ws = ws->ipm_workspace;
	struct ERK_WORKSPACE *erk_ws = ws->erk_workspace;

	struct CORE_QP_IPM_WORKSPACE *cws = ipm_ws->core_workspace;

	struct ERK_ARG *erk_arg = arg->erk_arg;

	// alias qp vectors into qp_sol
	cws->v = qp_sol->ux->pa;
	cws->pi = qp_sol->pi->pa;
	cws->lam = qp_sol->lam->pa;
	cws->t = qp_sol->t->pa;

	ipm_ws->mu0 = arg->mu0;

	int ss, nn, ii;

	// qp size
	int N = qp->N;
	int *nx = qp->nx;
	int *nu = qp->nu;
	int *nb = qp->nb;
	int *ng = qp->ng;
	int *ns = qp->ns;

	double *x, *u, *pi;

	double tmp;

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

	double nlp_res[4];


	// initialize nlp sol (to zero atm)
	for(nn=0; nn<=N; nn++)
		blasfeo_dvecse(nlp->nu[nn]+nlp->nx[nn], 0.0, nlp_sol->ux+nn, 0);
	for(nn=0; nn<N; nn++)
		blasfeo_dvecse(nlp->nx[nn+1], 0.0, nlp_sol->pi+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dvecse(2*nlp->nb[nn]+2*nlp->ng[nn], 0.0, nlp_sol->lam+nn, 0);
	for(nn=0; nn<=N; nn++)
		blasfeo_dvecse(2*nlp->nb[nn]+2*nlp->ng[nn], 0.0, nlp_sol->t+nn, 0);


	// copy nlp into qp
	nn = 0;
	for(; nn<=N; nn++)
		{
		blasfeo_dgecp(nu[nn]+nx[nn], nu[nn]+nx[nn], nlp->RSQ+nn, 0, 0, qp->RSQrq+nn, 0, 0);
		blasfeo_dgecp(nu[nn]+nx[nn], ng[nn], nlp->DCt+nn, 0, 0, qp->DCt+nn, 0, 0);
		blasfeo_dveccp(nu[nn]+nx[nn], nlp->rq+nn, 0, qp->rq+nn, 0);
		blasfeo_drowin(nu[nn]+nx[nn], 1.0, qp->rq+nn, 0, qp->RSQrq+nn, nu[nn]+nx[nn], 0);
		blasfeo_dveccp(2*nb[nn]+2*ng[nn]+2*ns[nn], nlp->d+nn, 0, qp->d+nn, 0);
		for(ii=0; ii<nb[nn]; ii++) qp->idxb[nn][ii] = nlp->idxb[nn][ii];
		for(ii=0; ii<ns[nn]; ii++) qp->idxs[nn][ii] = nlp->idxs[nn][ii];
		}


	// initialize solution
	INIT_VAR_OCP_QP(qp, qp_sol, ipm_ws);


	// nlp loop
	for(ss=0; ss<arg->nlp_iter_max; ss++)	
		{

		// simulation & sensitivity propagation
		for(nn=0; nn<N; nn++)
			{
			x  = (nlp_sol->ux+nn)->pa+nu[nn];
			u  = (nlp_sol->ux+nn)->pa;
			d_init_erk_int(nx[nn]+nu[nn], 0, x, u, (nlp->model+nn)->forward_seed, NULL, (nlp->model+nn)->expl_vde_for, NULL, (nlp->model+nn)->arg, erk_ws+nn);
			d_erk_int(erk_ws+nn);
			d_cvt_erk_int_to_ocp_qp(nn, erk_ws+nn, qp, nlp_sol);
			}

//for(ii=0; ii<N; ii++)
//	blasfeo_print_exp_dmat(nlp->nu[ii]+nlp->nx[ii]+1, nlp->nx[ii+1], qp->BAbt+ii, 0, 0);
	

#if 0
printf("\n%d %d\n", nu[0], nx[0]);
blasfeo_print_tran_dvec(nu[0]+nx[0], qp->rq+0, 0);
blasfeo_print_tran_dvec(nx[1], qp->b+0, 0);
exit(1);
#endif

		// compute residuals
		COMPUTE_RES_OCP_QP(qp, qp_sol, ipm_ws->res_workspace);
		cws->mu = ipm_ws->res_workspace->res_mu;
		if(ss>0 & ss<ipm_ws->stat_max)
			ipm_ws->stat[5*(ss-1)+4] = ipm_ws->res_workspace->res_mu;

		// compute infinity norm of residuals
		blasfeo_dvecnrm_inf(cws->nv, &str_res_g, 0, &nlp_res[0]);
		blasfeo_dvecnrm_inf(cws->ne, &str_res_b, 0, &nlp_res[1]);
		blasfeo_dvecnrm_inf(cws->nc, &str_res_d, 0, &nlp_res[2]);
		blasfeo_dvecnrm_inf(cws->nc, &str_res_m, 0, &nlp_res[3]);

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

		// exit condition on residuals
		if(!(nlp_res[0]>arg->nlp_res_g_max | nlp_res[1]>arg->nlp_res_b_max | nlp_res[2]>arg->nlp_res_d_max | nlp_res[3]>arg->nlp_res_m_max))
			{
			ws->iter = ss;
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
		if(ss<ipm_ws->stat_max)
			ipm_ws->stat[5*ss+0] = cws->alpha;

		// Mehrotra's corrector
		if(arg->pred_corr==1)
			{
			// mu_aff
			COMPUTE_MU_AFF_QP(cws);
			if(ss<ipm_ws->stat_max)
				ipm_ws->stat[5*ss+1] = cws->mu_aff;

			tmp = cws->mu_aff/cws->mu;
			cws->sigma = tmp*tmp*tmp;
			if(ss<ipm_ws->stat_max)
				ipm_ws->stat[5*ss+2] = cws->sigma;

			COMPUTE_CENTERING_CORRECTION_QP(cws);

			// fact and solve kkt
			SOLVE_KKT_STEP_OCP_QP(qp, ipm_ws);

			// alpha
			COMPUTE_ALPHA_QP(cws);
			if(ss<ipm_ws->stat_max)
				ipm_ws->stat[5*ss+3] = cws->alpha;
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
	
	// maximum iteration number reached
	ws->iter = ss;
	ws->nlp_res_g = nlp_res[0];
	ws->nlp_res_b = nlp_res[1];
	ws->nlp_res_d = nlp_res[2];
	ws->nlp_res_m = nlp_res[3];

	return 1;

	}

