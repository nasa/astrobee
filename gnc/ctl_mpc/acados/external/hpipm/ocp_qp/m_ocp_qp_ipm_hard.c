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



#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_s_aux.h>

#include "../include/hpipm_d_ocp_qp.h"
#include "../include/hpipm_s_ocp_qp.h"
#include "../include/hpipm_d_ocp_qp_sol.h"
#include "../include/hpipm_d_ocp_qp_ipm_hard.h"
#include "../include/hpipm_m_ocp_qp_ipm_hard.h"
#include "../include/hpipm_d_core_qp_ipm_hard.h"
#include "../include/hpipm_d_core_qp_ipm_hard_aux.h"
#include "../include/hpipm_d_ocp_qp_kkt.h"
#include "../include/hpipm_m_ocp_qp_kkt.h"



hpipm_size_t m_memsize_ipm_hard_ocp_qp(struct d_ocp_qp *qp, struct s_ocp_qp *s_qp, struct m_ipm_hard_ocp_qp_arg *arg)
	{

	// loop index
	int ii;

	// extract ocp qp size
	int N = qp->N;
	int *nx = qp->nx;
	int *nu = qp->nu;
	int *nb = qp->nb;
	int *ng = qp->ng;

	// compute core qp size and max size
	int nvt = 0;
	int net = 0;
	int nct = 0;
	int nxM = 0;
	int nuM = 0;
	int nbM = 0;
	int ngM = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nx[ii]+nu[ii];
		net += nx[ii+1];
		nct += nb[ii]+ng[ii];
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
		nbM = nb[ii]>nbM ? nb[ii] : nbM;
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}
	nvt += nx[ii]+nu[ii];
	nct += nb[ii]+ng[ii];
	nxM = nx[ii]>nxM ? nx[ii] : nxM;
	nuM = nu[ii]>nuM ? nu[ii] : nuM;
	nbM = nb[ii]>nbM ? nb[ii] : nbM;
	ngM = ng[ii]>ngM ? ng[ii] : ngM;

	hpipm_size_t size = 0;

	size += (4+(N+1)*18)*sizeof(struct blasfeo_dvec); // dux dpi dt_lb dt_lg res_g res_b res_d res_d_lb res_d_ub res_d_lg res_d_ug res_m res_m_lb res_m_ub res_m_lg res_m_ug Qx_lb Qx_lg qx_lb qx_lg tmp_nbM tmp_ngM
	size += (1+(N+1)*11)*sizeof(struct blasfeo_svec); // sdux sdpi sres_g sres_b sQx_lb sQx_lg, sqx_lb, sqx_lg tmp_nxM Pb sSx sSi
	size += (1+(N+1)*1)*sizeof(struct blasfeo_smat); // L AL

	size += 1*blasfeo_memsize_dvec(nbM); // tmp_nbM
	size += 1*blasfeo_memsize_svec(nxM); // tmp_nxM
	size += 2*blasfeo_memsize_dvec(nxM); // tmp_ngM
	for(ii=0; ii<=N; ii++) size += 2*blasfeo_memsize_svec(nu[ii]+nx[ii]); // sdux sres_g
	for(ii=0; ii<N; ii++) size += 3*blasfeo_memsize_svec(nx[ii+1]); // sdpi sres_b Pb
	for(ii=0; ii<=N; ii++) size += 2*blasfeo_memsize_svec(nb[ii]); // sQx_lb sqx_lb
	for(ii=0; ii<=N; ii++) size += 2*blasfeo_memsize_svec(ng[ii]); // sQx_lg sqx_lg
	for(ii=0; ii<=N; ii++) size += 2*blasfeo_memsize_svec(nu[ii]+nx[ii]); // sSx sSi
	for(ii=0; ii<=N; ii++) size += 1*blasfeo_memsize_smat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // L
	size += 2*blasfeo_memsize_smat(nuM+nxM+1, nxM+ngM); // AL

	size += 1*sizeof(struct d_ipm_hard_core_qp_workspace);
	size += 1*d_memsize_ipm_hard_core_qp(nvt, net, nct, arg->iter_max);

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void m_create_ipm_hard_ocp_qp(struct d_ocp_qp *qp, struct s_ocp_qp *s_qp, struct m_ipm_hard_ocp_qp_arg *arg, struct m_ipm_hard_ocp_qp_workspace *workspace, void *mem)
	{

	// loop index
	int ii;

	// extract ocp qp size
	int N = qp->N;
	int *nx = qp->nx;
	int *nu = qp->nu;
	int *nb = qp->nb;
	int *ng = qp->ng;

	// compute core qp size and max size
	int nvt = 0;
	int net = 0;
	int nct = 0;
	int ngt = 0;
	int nxM = 0;
	int nuM = 0;
	int nbM = 0;
	int ngM = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nx[ii]+nu[ii];
		net += nx[ii+1];
		nct += nb[ii]+ng[ii];
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
		nbM = nb[ii]>nbM ? nb[ii] : nbM;
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}
	nvt += nx[ii]+nu[ii];
	nct += nb[ii]+ng[ii];
	nxM = nx[ii]>nxM ? nx[ii] : nxM;
	nuM = nu[ii]>nuM ? nu[ii] : nuM;
	nbM = nb[ii]>nbM ? nb[ii] : nbM;
	ngM = ng[ii]>ngM ? ng[ii] : ngM;


	// core struct
	struct d_ipm_hard_core_qp_workspace *sr_ptr = mem;

	// core workspace
	workspace->core_workspace = sr_ptr;
	sr_ptr += 1;
	struct d_ipm_hard_core_qp_workspace *cws = workspace->core_workspace;


	// s matrix struct
	struct blasfeo_smat *sm_ptr = (struct blasfeo_smat *) sr_ptr;

	workspace->L = sm_ptr;
	sm_ptr += N+1;
	workspace->AL = sm_ptr;
	sm_ptr += 2;


	// s vector struct
	struct blasfeo_svec *ssv_ptr = (struct blasfeo_svec *) sm_ptr;

	workspace->sdux = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sdpi = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sres_g = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sres_b = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sQx_lb = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sQx_lg = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sqx_lb = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sqx_lg = ssv_ptr;
	ssv_ptr += N+1;
	workspace->Pb = ssv_ptr;
	ssv_ptr += N+1;
	workspace->tmp_nxM = ssv_ptr;
	ssv_ptr += 1;
	workspace->sSx = ssv_ptr;
	ssv_ptr += N+1;
	workspace->sSi = ssv_ptr;
	ssv_ptr += N+1;


	// d vector struct
	struct blasfeo_dvec *sv_ptr = (struct blasfeo_dvec *) ssv_ptr;

	workspace->dux = sv_ptr;
	sv_ptr += N+1;
	workspace->dpi = sv_ptr;
	sv_ptr += N+1;
	workspace->dt_lb = sv_ptr;
	sv_ptr += N+1;
	workspace->dt_lg = sv_ptr;
	sv_ptr += N+1;
	workspace->res_g = sv_ptr;
	sv_ptr += N+1;
	workspace->res_b = sv_ptr;
	sv_ptr += N+1;
	workspace->res_d = sv_ptr;
	sv_ptr += 1;
	workspace->res_d_lb = sv_ptr;
	sv_ptr += N+1;
	workspace->res_d_ub = sv_ptr;
	sv_ptr += N+1;
	workspace->res_d_lg = sv_ptr;
	sv_ptr += N+1;
	workspace->res_d_ug = sv_ptr;
	sv_ptr += N+1;
	workspace->res_m = sv_ptr;
	sv_ptr += 1;
	workspace->res_m_lb = sv_ptr;
	sv_ptr += N+1;
	workspace->res_m_ub = sv_ptr;
	sv_ptr += N+1;
	workspace->res_m_lg = sv_ptr;
	sv_ptr += N+1;
	workspace->res_m_ug = sv_ptr;
	sv_ptr += N+1;
	workspace->Qx_lb = sv_ptr;
	sv_ptr += N+1;
	workspace->Qx_lg = sv_ptr;
	sv_ptr += N+1;
	workspace->qx_lb = sv_ptr;
	sv_ptr += N+1;
	workspace->qx_lg = sv_ptr;
	sv_ptr += N+1;
//	workspace->Pb = sv_ptr;
//	sv_ptr += N+1;
	workspace->tmp_nbM = sv_ptr;
	sv_ptr += 1;
//	workspace->tmp_nxM = sv_ptr;
//	sv_ptr += 1;
	workspace->tmp_ngM = sv_ptr;
	sv_ptr += 2;


	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) sv_ptr;
	s_ptr = (s_ptr+63)/64*64;


	// char stuf
	char *c_ptr = (char *) s_ptr;

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_smat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], workspace->L+ii, c_ptr);
		c_ptr += (workspace->L+ii)->memsize;
		}

	blasfeo_create_smat(nuM+nxM+1, nxM+ngM, workspace->AL+0, c_ptr);
	c_ptr += (workspace->AL+0)->memsize;

	blasfeo_create_smat(nuM+nxM+1, nxM+ngM, workspace->AL+1, c_ptr);
	c_ptr += (workspace->AL+1)->memsize;

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(nu[ii]+nx[ii], workspace->sdux+ii, c_ptr);
		c_ptr += (workspace->sdux+ii)->memsize;
		}

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_svec(nx[ii+1], workspace->sdpi+ii, c_ptr);
		c_ptr += (workspace->sdpi+ii)->memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(nu[ii]+nx[ii], workspace->sres_g+ii, c_ptr);
		c_ptr += (workspace->sdux+ii)->memsize;
		}

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_svec(nx[ii+1], workspace->sres_b+ii, c_ptr);
		c_ptr += (workspace->sdpi+ii)->memsize;
		}

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_svec(nx[ii+1], workspace->Pb+ii, c_ptr);
		c_ptr += (workspace->Pb+ii)->memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(nb[ii], workspace->sQx_lb+ii, c_ptr);
		c_ptr += (workspace->sQx_lb+ii)->memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(nb[ii], workspace->sqx_lb+ii, c_ptr);
		c_ptr += (workspace->sqx_lb+ii)->memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(ng[ii], workspace->sQx_lg+ii, c_ptr);
		c_ptr += (workspace->sQx_lg+ii)->memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(ng[ii], workspace->sqx_lg+ii, c_ptr);
		c_ptr += (workspace->sqx_lg+ii)->memsize;
		}

	blasfeo_create_dvec(nbM, workspace->tmp_nbM, c_ptr);
	c_ptr += workspace->tmp_nbM->memsize;

	blasfeo_create_svec(nxM, workspace->tmp_nxM, c_ptr);
	c_ptr += workspace->tmp_nxM->memsize;

	blasfeo_create_dvec(ngM, workspace->tmp_ngM+0, c_ptr);
	c_ptr += (workspace->tmp_ngM+0)->memsize;

	blasfeo_create_dvec(ngM, workspace->tmp_ngM+1, c_ptr);
	c_ptr += (workspace->tmp_ngM+1)->memsize;

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(nu[ii]+nx[ii], workspace->sSx+ii, c_ptr);
		c_ptr += (workspace->sSx+ii)->memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_svec(nu[ii]+nx[ii], workspace->sSi+ii, c_ptr);
		c_ptr += (workspace->sSi+ii)->memsize;
		}



	cws->nv = nvt;
	cws->ne = net;
	cws->nc = nct;
	cws->iter_max = arg->iter_max;
	d_create_ipm_hard_core_qp(cws, c_ptr);
	c_ptr += workspace->core_workspace->memsize;

	cws->alpha_min = arg->alpha_min;
	cws->mu_max = arg->mu_max;
	cws->mu0 = arg->mu0;
	cws->nt_inv = 1.0/(2*nct); // XXX


	// alias members of workspace and core_workspace
	//
	c_ptr = (char *) cws->dv;
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], workspace->dux+ii, c_ptr);
		c_ptr += (nu[ii]+nx[ii])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->dpi;
	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], workspace->dpi+ii, c_ptr);
		c_ptr += (nx[ii+1])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->dt;
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii], workspace->dt_lb+ii, c_ptr);
		c_ptr += (nb[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(ng[ii], workspace->dt_lg+ii, c_ptr);
		c_ptr += (ng[ii])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->res_g;
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], workspace->res_g+ii, c_ptr);
		c_ptr += (nu[ii]+nx[ii])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->res_b;
	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], workspace->res_b+ii, c_ptr);
		c_ptr += (nx[ii+1])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->res_d;
	blasfeo_create_dvec(2*nct, workspace->res_d, c_ptr);
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii], workspace->res_d_lb+ii, c_ptr);
		c_ptr += (nb[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(ng[ii], workspace->res_d_lg+ii, c_ptr);
		c_ptr += (ng[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii], workspace->res_d_ub+ii, c_ptr);
		c_ptr += (nb[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(ng[ii], workspace->res_d_ug+ii, c_ptr);
		c_ptr += (ng[ii])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->res_m;
	blasfeo_create_dvec(2*nct, workspace->res_m, c_ptr);
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii], workspace->res_m_lb+ii, c_ptr);
		c_ptr += (nb[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(ng[ii], workspace->res_m_lg+ii, c_ptr);
		c_ptr += (ng[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii], workspace->res_m_ub+ii, c_ptr);
		c_ptr += (nb[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(ng[ii], workspace->res_m_ug+ii, c_ptr);
		c_ptr += (ng[ii])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->Qx;
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii], workspace->Qx_lb+ii, c_ptr);
		c_ptr += (nb[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(ng[ii], workspace->Qx_lg+ii, c_ptr);
		c_ptr += (ng[ii])*sizeof(double);
		}
	//
	c_ptr = (char *) cws->qx;
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii], workspace->qx_lb+ii, c_ptr);
		c_ptr += (nb[ii])*sizeof(double);
		}
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(ng[ii], workspace->qx_lg+ii, c_ptr);
		c_ptr += (ng[ii])*sizeof(double);
		}
	workspace->stat = cws->stat;

	// default flag values
	workspace->compute_Pb = 0;
	workspace->scale = 0;

	return;

	}



void m_solve_ipm_hard_ocp_qp(struct d_ocp_qp *qp, struct s_ocp_qp *s_qp, struct d_ocp_qp_sol *qp_sol, struct m_ipm_hard_ocp_qp_workspace *ws)
	{

	struct d_ipm_hard_core_qp_workspace *cws = ws->core_workspace;

	// alias d_ocp_workspace to m_ocp_workspace
	struct d_ipm_hard_ocp_qp_workspace dws;
	dws.core_workspace = ws->core_workspace;
	dws.dux = ws->dux;
	dws.dpi = ws->dpi;
	dws.dt_lb = ws->dt_lb;
	dws.dt_lg = ws->dt_lg;
	dws.res_g = ws->res_g;
	dws.res_b = ws->res_b;
	dws.res_d = ws->res_d;
	dws.res_d_lb = ws->res_d_lb;
	dws.res_d_ub = ws->res_d_ub;
	dws.res_d_lg = ws->res_d_lg;
	dws.res_d_ug = ws->res_d_ug;
	dws.res_m = ws->res_m;
	dws.res_m_lb = ws->res_m_lb;
	dws.res_m_ub = ws->res_m_ub;
	dws.res_m_lg = ws->res_m_lg;
	dws.res_m_ug = ws->res_m_ug;
	dws.tmp_nbM = ws->tmp_nbM;
	dws.tmp_ngM = ws->tmp_ngM;

	// alias qp vectors into qp_sol
	cws->v = qp_sol->ux->pa;
	cws->pi = qp_sol->pi->pa;
	cws->lam = qp_sol->lam_lb->pa;
	cws->t = qp_sol->t_lb->pa;

	if(cws->nc==0)
		{
		//
		d_init_var_hard_ocp_qp(qp, qp_sol, &dws);
		//
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		//
		m_fact_solve_kkt_step_hard_ocp_qp(qp, s_qp, ws);
		//
		cws->alpha = 1.0;
		d_update_var_hard_qp(cws);
		//
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		//
		ws->compute_Pb = 1;
		m_solve_kkt_step_hard_ocp_qp(qp, s_qp, ws);
		//
		cws->alpha = 1.0;
		d_update_var_hard_qp(cws);
		//
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		//
		ws->iter = 0;
		return;
		}

	// init solver
	d_init_var_hard_ocp_qp(qp, qp_sol, &dws);

	// compute residuals
	d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
	cws->mu = dws.res_mu;
	ws->res_mu = dws.res_mu;

	ws->scale = 1;

	int kk;
	for(kk=0; kk<cws->iter_max & cws->mu>cws->mu_max; kk++)
		{

		// fact and solve kkt
		m_fact_solve_kkt_step_hard_ocp_qp(qp, s_qp, ws);

		// alpha
		d_compute_alpha_hard_qp(cws);
		cws->stat[5*kk+0] = cws->alpha;

		//
		d_update_var_hard_qp(cws);
#if 0
int ii;
for(ii=0; ii<=qp->N; ii++)
	blasfeo_print_exp_tran_dvec(qp->nu[ii]+qp->nx[ii], qp_sol->ux+ii, 0);
for(ii=0; ii<qp->N; ii++)
	blasfeo_print_exp_tran_dvec(qp->nx[ii+1], qp_sol->pi+ii, 0);
for(ii=0; ii<=qp->N; ii++)
	blasfeo_print_exp_tran_dvec(qp->nb[ii], qp_sol->lam_lb+ii, 0);
for(ii=0; ii<=qp->N; ii++)
	blasfeo_print_exp_tran_dvec(qp->nb[ii], qp_sol->lam_ub+ii, 0);
for(ii=0; ii<=qp->N; ii++)
	blasfeo_print_exp_tran_dvec(qp->nb[ii], ws->dt_lb+ii, 0);
//exit(1);
#endif

		// compute residuals
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		cws->stat[5*kk+1] = ws->res_mu;

		}
	
	ws->iter = kk;
	
	return;

	}



void m_solve_ipm2_hard_ocp_qp(struct d_ocp_qp *qp, struct s_ocp_qp *s_qp, struct d_ocp_qp_sol *qp_sol, struct m_ipm_hard_ocp_qp_workspace *ws)
	{

	struct d_ipm_hard_core_qp_workspace *cws = ws->core_workspace;

	// alias d_ocp_workspace to m_ocp_workspace
	struct d_ipm_hard_ocp_qp_workspace dws;
	dws.core_workspace = ws->core_workspace;
	dws.dux = ws->dux;
	dws.dpi = ws->dpi;
	dws.dt_lb = ws->dt_lb;
	dws.dt_lg = ws->dt_lg;
	dws.res_g = ws->res_g;
	dws.res_b = ws->res_b;
	dws.res_d = ws->res_d;
	dws.res_d_lb = ws->res_d_lb;
	dws.res_d_ub = ws->res_d_ub;
	dws.res_d_lg = ws->res_d_lg;
	dws.res_d_ug = ws->res_d_ug;
	dws.res_m = ws->res_m;
	dws.res_m_lb = ws->res_m_lb;
	dws.res_m_ub = ws->res_m_ub;
	dws.res_m_lg = ws->res_m_lg;
	dws.res_m_ug = ws->res_m_ug;
	dws.tmp_nbM = ws->tmp_nbM;
	dws.tmp_ngM = ws->tmp_ngM;

	// alias qp vectors into qp_sol
	cws->v = qp_sol->ux->pa;
	cws->pi = qp_sol->pi->pa;
	cws->lam = qp_sol->lam_lb->pa;
	cws->t = qp_sol->t_lb->pa;

	double tmp;

	if(cws->nc==0)
		{
		//
		d_init_var_hard_ocp_qp(qp, qp_sol, &dws);
		//
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		//
		m_fact_solve_kkt_step_hard_ocp_qp(qp, s_qp, ws);
		//
		cws->alpha = 1.0;
		d_update_var_hard_qp(cws);
		//
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		//
		ws->compute_Pb = 1;
		m_solve_kkt_step_hard_ocp_qp(qp, s_qp, ws);
		//
		cws->alpha = 1.0;
		d_update_var_hard_qp(cws);
		//
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		//
		ws->iter = 0;
		return;
		}

	// init solver
	d_init_var_hard_ocp_qp(qp, qp_sol, &dws);

	// compute residuals
	d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
	cws->mu = dws.res_mu;
	ws->res_mu = dws.res_mu;

#if 0
	printf("\nres_g\n");
	for(int ii=0; ii<=qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nx[ii]+qp->nu[ii], ws->res_g+ii, 0);
		}
	printf("\nres_b\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nx[ii+1], ws->res_b+ii, 0);
		}
	printf("\nres_d_lb\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nb[ii], ws->res_d_lb+ii, 0);
		}
	printf("\nres_d_ub\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nb[ii], ws->res_d_ub+ii, 0);
		}
	printf("\nres_d_lg\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->ng[ii], ws->res_d_lg+ii, 0);
		}
	printf("\nres_d_ug\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->ng[ii], ws->res_d_ug+ii, 0);
		}
	printf("\nres_m_lb\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nb[ii], ws->res_m_lb+ii, 0);
		}
	printf("\nres_m_ub\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nb[ii], ws->res_m_ub+ii, 0);
		}
	printf("\nres_m_lg\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->ng[ii], ws->res_m_lg+ii, 0);
		}
	printf("\nres_m_ug\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->ng[ii], ws->res_m_ug+ii, 0);
		}
	exit(1);
#endif

#if 0
	int ii;
	for(ii=0; ii<1; ii++)
		{
		cws->sigma = 1.0;
		cws->stat[5*kk+2] = cws->sigma;
		COMPUTE_CENTERING_CORRECTION_HARD_QP(cws);
		FACT_SOLVE_KKT_STEP_HARD_OCP_QP(qp, ws);
		COMPUTE_ALPHA_HARD_QP(cws);
		cws->stat[5*kk+3] = cws->alpha;
		UPDATE_VAR_HARD_QP(cws);
		COMPUTE_RES_HARD_OCP_QP(qp, qp_sol, ws);
		cws->mu = ws->res_mu;
		cws->stat[5*kk+4] = ws->res_mu;
		kk++;
		}
//	ws->iter = kk;
//		return;
#endif

	ws->scale = 0;

	int kk = 0;
	for(; kk<cws->iter_max & cws->mu>cws->mu_max; kk++)
		{

		// fact and solve kkt
		m_fact_solve_kkt_step_hard_ocp_qp(qp, s_qp, ws);

#if 0
	printf("\ndux\n");
	for(int ii=0; ii<=qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nx[ii]+qp->nu[ii], ws->dux+ii, 0);
		}
	printf("\ndpi\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(qp->nx[ii+1], ws->dpi+ii, 0);
		}
	printf("\ndt\n");
	for(int ii=0; ii<qp->N; ii++)
		{
		PRINT_E_TRAN_STRVEC(2*qp->nb[ii]+2*qp->ng[ii], ws->dt_lb+ii, 0);
		}
	exit(1);
#endif
		// alpha
		d_compute_alpha_hard_qp(cws);
		cws->stat[5*kk+0] = cws->alpha;

		// mu_aff
		d_compute_mu_aff_hard_qp(cws);
		cws->stat[5*kk+1] = cws->mu_aff;

		tmp = cws->mu_aff/cws->mu;
		cws->sigma = tmp*tmp*tmp;
		cws->stat[5*kk+2] = cws->sigma;

		d_compute_centering_correction_hard_qp(cws);

		// fact and solve kkt
		ws->compute_Pb = 0;
		m_solve_kkt_step_hard_ocp_qp(qp, s_qp, ws);

#if 0
int ii;
for(ii=0; ii<=qp->N; ii++)
	blasfeo_print_tran_dvec(qp->nu[ii]+qp->nx[ii], ws->dux+ii, 0);
for(ii=0; ii<qp->N; ii++)
	blasfeo_print_tran_dvec(qp->nx[ii+1], ws->dpi+ii, 0);
exit(1);
#endif
		// alpha
		d_compute_alpha_hard_qp(cws);
		cws->stat[5*kk+3] = cws->alpha;

		//
		d_update_var_hard_qp(cws);

		// compute residuals
		d_compute_res_hard_ocp_qp(qp, qp_sol, &dws);
		cws->mu = dws.res_mu;
		ws->res_mu = dws.res_mu;
		cws->stat[5*kk+4] = ws->res_mu;

		}
	
	ws->iter = kk;
	
	return;

	}




