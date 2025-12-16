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



#include <math.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_m_aux.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_s_blas.h>

#include "../include/hpipm_d_ocp_qp.h"
#include "../include/hpipm_s_ocp_qp.h"
#include "../include/hpipm_d_ocp_qp_sol.h"
#include "../include/hpipm_d_ocp_qp_ipm_hard.h"
#include "../include/hpipm_m_ocp_qp_ipm_hard.h"
#include "../include/hpipm_d_core_qp_ipm_hard.h"
#include "../include/hpipm_d_core_qp_ipm_hard_aux.h"



// backward Riccati recursion
void m_fact_solve_kkt_step_hard_ocp_qp(struct d_ocp_qp *d_qp, struct s_ocp_qp *s_qp, struct m_ipm_hard_ocp_qp_workspace *ws)
	{

	int N = s_qp->N;
	int *nx = s_qp->nx;
	int *nu = s_qp->nu;
	int *nb = s_qp->nb;
	int *ng = s_qp->ng;

	struct blasfeo_smat *BAbt = s_qp->BAbt;
	struct blasfeo_smat *RSQrq = s_qp->RSQrq;
	struct blasfeo_smat *DCt = s_qp->DCt;
	struct blasfeo_dmat *d_DCt = d_qp->DCt;
	int **idxb = s_qp->idxb;
	int **d_idxb = d_qp->idxb;

	struct blasfeo_smat *L = ws->L;
	struct blasfeo_smat *AL = ws->AL;
	struct blasfeo_dvec *d_res_b = ws->res_b;
	struct blasfeo_dvec *d_res_g = ws->res_g;
	struct blasfeo_svec *res_b = ws->sres_b;
	struct blasfeo_svec *res_g = ws->sres_g;
	struct blasfeo_dvec *d_dux = ws->dux;
	struct blasfeo_dvec *d_dpi = ws->dpi;
	struct blasfeo_svec *dux = ws->sdux;
	struct blasfeo_svec *dpi = ws->sdpi;
	struct blasfeo_dvec *d_dt_lb = ws->dt_lb;
	struct blasfeo_dvec *d_dt_lg = ws->dt_lg;
	struct blasfeo_dvec *d_Qx_lg = ws->Qx_lg;
	struct blasfeo_dvec *d_Qx_lb = ws->Qx_lb;
	struct blasfeo_dvec *d_qx_lg = ws->qx_lg;
	struct blasfeo_dvec *d_qx_lb = ws->qx_lb;
	struct blasfeo_svec *Qx_lg = ws->sQx_lg;
	struct blasfeo_svec *Qx_lb = ws->sQx_lb;
	struct blasfeo_svec *qx_lg = ws->sqx_lg;
	struct blasfeo_svec *qx_lb = ws->sqx_lb;
	struct blasfeo_svec *Pb = ws->Pb;
	struct blasfeo_svec *tmp_nxM = ws->tmp_nxM;
	struct blasfeo_svec *Sx = ws->sSx;
	struct blasfeo_svec *Si = ws->sSi;

	//
	int ii, jj;
	float d_tmp0, d_tmp1;
	float *d_ptr0, *d_ptr1;

	//
	struct d_ipm_hard_core_qp_workspace *cws = ws->core_workspace;

//	if(nb>0 | ng>0)
//		{
		d_compute_Qx_qx_hard_qp(cws);
//		}



	// cvt double => single
	for(ii=0; ii<N; ii++)
		{
		m_cvt_d2blasfeo_svec(nu[ii]+nx[ii], d_res_g+ii, 0, res_g+ii, 0);
		m_cvt_d2blasfeo_svec(nx[ii+1], d_res_b+ii, 0, res_b+ii, 0);
		m_cvt_d2blasfeo_svec(nb[ii], d_Qx_lb+ii, 0, Qx_lb+ii, 0);
		m_cvt_d2blasfeo_svec(nb[ii], d_qx_lb+ii, 0, qx_lb+ii, 0);
		m_cvt_d2blasfeo_svec(ng[ii], d_Qx_lg+ii, 0, Qx_lg+ii, 0);
		m_cvt_d2blasfeo_svec(ng[ii], d_qx_lg+ii, 0, qx_lg+ii, 0);
		}
	ii = N;
	m_cvt_d2blasfeo_svec(nu[ii]+nx[ii], d_res_g+ii, 0, res_g+ii, 0);
	m_cvt_d2blasfeo_svec(nb[ii], d_Qx_lb+ii, 0, Qx_lb+ii, 0);
	m_cvt_d2blasfeo_svec(nb[ii], d_qx_lb+ii, 0, qx_lb+ii, 0);
	m_cvt_d2blasfeo_svec(ng[ii], d_Qx_lg+ii, 0, Qx_lg+ii, 0);
	m_cvt_d2blasfeo_svec(ng[ii], d_qx_lg+ii, 0, qx_lg+ii, 0);


#if 0
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(nu[ii]+nx[ii], res_g+ii, 0);
	for(ii=0; ii<N; ii++)
		blasfeo_print_exp_tran_svec(nx[ii+1], res_b+ii, 0);
//	exit(1);
#endif


	// factorization and backward substitution

	// last stage
#if defined(DOUBLE_PRECISION)
	blasfeo_strcp_l(nu[N]+nx[N], RSQrq+N, 0, 0, L+N, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#else
	blasfeo_sgecp(nu[N]+nx[N], nu[N]+nx[N], RSQrq+N, 0, 0, L+N, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#endif
	blasfeo_srowin(nu[N]+nx[N], 1.0, res_g+N, 0, L+N, nu[N]+nx[N], 0);
	if(ws->scale)
		{
		if(nb[N]>0)
			{
			blasfeo_sdiaad_sp(nb[N], 1.0, Qx_lb+N, 0, idxb[N], L+N, 0, 0);
			blasfeo_srowad_sp(nb[N], 1.0, qx_lb+N, 0, idxb[N], L+N, nu[N]+nx[N], 0);
			}
		if(ng[N]>0)
			{
			blasfeo_sgemm_nd(nu[N]+nx[N], ng[N], 1.0, DCt+N, 0, 0, Qx_lg+N, 0, 0.0, AL+0, 0, 0, AL+0, 0, 0);
			blasfeo_srowin(ng[N], 1.0, qx_lg+N, 0, AL+0, nu[N]+nx[N], 0);
			blasfeo_ssyrk_ln_mn(nu[N]+nx[N]+1, nu[N]+nx[N], ng[N], 1.0, AL+0, 0, 0, DCt+N, 0, 0, 1.0, L+N, 0, 0, L+N, 0, 0);
			}
		blasfeo_sdiaex(nu[N]+nx[N], 1.0, L+N, 0, 0, Sx+N, 0);
		d_ptr0 = (Sx+N)->pa;
		d_ptr1 = (Si+N)->pa;
		for(jj=0; jj<nu[N]+nx[N]; jj++)
			{
			d_tmp0 = sqrt(d_ptr0[jj]);
//			d_tmp0 = sqrt(d_tmp0);
			d_tmp1 = sqrt(d_tmp0);
			d_tmp0 = sqrt(d_tmp0*d_tmp1);
			d_tmp0 = 1.0;
			d_ptr0[jj] = d_tmp0;
			d_ptr1[jj] = 1.0/d_tmp0;
			}
//		blasfeo_print_smat(nu[N]+nx[N]+1, nu[N]+nx[N], L+N, 0, 0);
		blasfeo_sgemm_nd(nu[N]+nx[N]+1, nu[N]+nx[N], 1.0, L+N, 0, 0, Si+N, 0, 0.0, L+N, 0, 0, L+N, 0, 0);
//		blasfeo_print_smat(nu[N]+nx[N]+1, nu[N]+nx[N], L+N, 0, 0);
		blasfeo_sgemm_dn(nu[N]+nx[N], nu[N]+nx[N], 1.0, Si+N, 0, L+N, 0, 0, 0.0, L+N, 0, 0, L+N, 0, 0);
//		blasfeo_print_smat(nu[N]+nx[N]+1, nu[N]+nx[N], L+N, 0, 0);

		blasfeo_spotrf_l_mn(nu[N]+nx[N]+1, nu[N]+nx[N], L+N, 0, 0, L+N, 0, 0);
		}
	else
		{
		if(nb[N]>0)
			{
			blasfeo_sdiaad_sp(nb[N], 1.0, Qx_lb+N, 0, idxb[N], L+N, 0, 0);
			blasfeo_srowad_sp(nb[N], 1.0, qx_lb+N, 0, idxb[N], L+N, nu[N]+nx[N], 0);
			}
		if(ng[N]>0)
			{
			blasfeo_sgemm_nd(nu[N]+nx[N], ng[N], 1.0, DCt+N, 0, 0, Qx_lg+N, 0, 0.0, AL+0, 0, 0, AL+0, 0, 0);
			blasfeo_srowin(ng[N], 1.0, qx_lg+N, 0, AL+0, nu[N]+nx[N], 0);
			blasfeo_ssyrk_spotrf_ln(nu[N]+nx[N]+1, nu[N]+nx[N], ng[N], AL+0, 0, 0, DCt+N, 0, 0, L+N, 0, 0, L+N, 0, 0);
			}
		else
			{
			blasfeo_spotrf_l_mn(nu[N]+nx[N]+1, nu[N]+nx[N], L+N, 0, 0, L+N, 0, 0);
			}
		}

	// middle stages
	for(ii=0; ii<N; ii++)
		{
//		printf("\n%d\n", N-ii-1);
#if defined(DOUBLE_PRECISION)
		blasfeo_strcp_l(nu[N-ii-1]+nx[N-ii-1], RSQrq+(N-ii-1), 0, 0, L+(N-ii-1), 0, 0);
#else
		blasfeo_sgecp(nu[N-ii-1]+nx[N-ii-1], nu[N-ii-1]+nx[N-ii-1], RSQrq+(N-ii-1), 0, 0, L+(N-ii-1), 0, 0);
#endif
		blasfeo_srowin(nu[N-ii-1]+nx[N-ii-1], 1.0, res_g+(N-ii-1), 0, L+(N-ii-1), nu[N-ii-1]+nx[N-ii-1], 0);

		if(ws->scale)
			{
			if(nb[N-ii-1]>0)
				{
				blasfeo_sdiaad_sp(nb[N-ii-1], 1.0, Qx_lb+N-ii-1, 0, idxb[N-ii-1], L+N-ii-1, 0, 0);
				blasfeo_srowad_sp(nb[N-ii-1], 1.0, qx_lb+N-ii-1, 0, idxb[N-ii-1], L+N-ii-1, nu[N-ii-1]+nx[N-ii-1], 0);
				}
			if(ng[N-ii-1]>0)
				{
				blasfeo_sgemm_nd(nu[N-ii-1]+nx[N-ii-1], ng[N-ii-1], 1.0, DCt+N-ii-1, 0, 0, Qx_lg+N-ii-1, 0, 0.0, AL+1, 0, 0, AL+1, 0, 0);
				blasfeo_srowin(ng[N-ii-1], 1.0, qx_lg+N-ii-1, 0, AL+1, nu[N-ii-1]+nx[N-ii-1], 0);
				blasfeo_ssyrk_ln_mn(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], ng[N-ii-1], 1.0, AL+1, 0, 0, DCt+N-ii-1, 0, 0, 1.0, L+N-ii-1, 0, 0, L+N-ii-1, 0, 0);
				}
			blasfeo_sdiaex(nu[N-ii-1]+nx[N-ii-1], 1.0, L+N-ii-1, 0, 0, Sx+N-ii-1, 0);
			d_ptr0 = (Sx+N-ii-1)->pa;
			d_ptr1 = (Si+N-ii-1)->pa;
			for(jj=0; jj<nu[N-ii-1]+nx[N-ii-1]; jj++)
				{
				d_tmp0 = sqrt(d_ptr0[jj]);
//				d_tmp0 = sqrt(d_tmp0);
				d_tmp1 = sqrt(d_tmp0);
				d_tmp0 = sqrt(d_tmp0*d_tmp1);
				d_tmp0 = 1.0;
				d_ptr0[jj] = d_tmp0;
				d_ptr1[jj] = 1.0/d_tmp0;
				}

			blasfeo_sgecp(nu[N-ii-1]+nx[N-ii-1], nx[N-ii], BAbt+(N-ii-1), 0, 0, AL, 0, 0);
			blasfeo_srowin(nx[N-ii], 1.0, res_b+(N-ii-1), 0, AL, nu[N-ii-1]+nx[N-ii-1], 0);
//			blasfeo_print_smat(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], AL, 0, 0);
			blasfeo_sgemm_nd(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], 1.0, AL, 0, 0, Sx+N-ii, nu[N-ii], 0.0, AL, 0, 0, AL, 0, 0);
//			blasfeo_print_smat(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], AL, 0, 0);
			blasfeo_sgemm_dn(nu[N-ii-1]+nx[N-ii-1], nx[N-ii], 1.0, Si+N-ii-1, 0, AL, 0, 0, 0.0, AL, 0, 0, AL, 0, 0);
//			blasfeo_print_smat(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], AL, 0, 0);

			blasfeo_strmm_rlnn(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], 1.0, L+(N-ii), nu[N-ii], nu[N-ii], AL, 0, 0, AL, 0, 0);
			blasfeo_srowex(nx[N-ii], 1.0, AL, nu[N-ii-1]+nx[N-ii-1], 0, tmp_nxM, 0);
			blasfeo_strmv_lnn(nx[N-ii], nx[N-ii], L+(N-ii), nu[N-ii], nu[N-ii], tmp_nxM, 0, Pb+(N-ii-1), 0);
			blasfeo_sgead(1, nx[N-ii], 1.0, L+(N-ii), nu[N-ii]+nx[N-ii], nu[N-ii], AL, nu[N-ii-1]+nx[N-ii-1], 0);

			blasfeo_sgemm_nd(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], 1.0, L+N-ii-1, 0, 0, Si+N-ii-1, 0, 0.0, L+N-ii-1, 0, 0, L+N-ii-1, 0, 0);
			blasfeo_sgemm_dn(nu[N-ii-1]+nx[N-ii-1], nu[N-ii-1]+nx[N-ii-1], 1.0, Si+N-ii-1, 0, L+N-ii-1, 0, 0, 0.0, L+N-ii-1, 0, 0, L+N-ii-1, 0, 0);

			blasfeo_ssyrk_spotrf_ln(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], nx[N-ii], AL, 0, 0, AL, 0, 0, L+(N-ii-1), 0, 0, L+(N-ii-1), 0, 0);
//			blasfeo_ssyrk_ln_mn(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], nx[N-ii], 1.0, AL, 0, 0, AL, 0, 0, 1.0, L+(N-ii-1), 0, 0, L+(N-ii-1), 0, 0);
//			blasfeo_spotrf_l_mn(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], L+(N-ii-1), 0, 0, L+(N-ii-1), 0, 0);
//			blasfeo_print_exp_smat(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], L+N-ii-1, 0, 0);
			}
		else
			{
			blasfeo_sgecp(nu[N-ii-1]+nx[N-ii-1], nx[N-ii], BAbt+(N-ii-1), 0, 0, AL, 0, 0);
			blasfeo_srowin(nx[N-ii], 1.0, res_b+(N-ii-1), 0, AL, nu[N-ii-1]+nx[N-ii-1], 0);
			blasfeo_strmm_rlnn(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], 1.0, L+(N-ii), nu[N-ii], nu[N-ii], AL, 0, 0, AL, 0, 0);
			blasfeo_srowex(nx[N-ii], 1.0, AL, nu[N-ii-1]+nx[N-ii-1], 0, tmp_nxM, 0);
			blasfeo_strmv_lnn(nx[N-ii], nx[N-ii], L+(N-ii), nu[N-ii], nu[N-ii], tmp_nxM, 0, Pb+(N-ii-1), 0);
			blasfeo_sgead(1, nx[N-ii], 1.0, L+(N-ii), nu[N-ii]+nx[N-ii], nu[N-ii], AL, nu[N-ii-1]+nx[N-ii-1], 0);

			if(nb[N-ii-1]>0)
				{
				blasfeo_sdiaad_sp(nb[N-ii-1], 1.0, Qx_lb+(N-ii-1), 0, idxb[N-ii-1], L+(N-ii-1), 0, 0);
				blasfeo_srowad_sp(nb[N-ii-1], 1.0, qx_lb+(N-ii-1), 0, idxb[N-ii-1], L+(N-ii-1), nu[N-ii-1]+nx[N-ii-1], 0);
				}

			if(ng[N-ii-1]>0)
				{
				blasfeo_sgemm_nd(nu[N-ii-1]+nx[N-ii-1], ng[N-ii-1], 1.0, DCt+N-ii-1, 0, 0, Qx_lg+N-ii-1, 0, 0.0, AL+0, 0, nx[N-ii], AL+0, 0, nx[N-ii]);
				blasfeo_srowin(ng[N-ii-1], 1.0, qx_lg+N-ii-1, 0, AL+0, nu[N-ii-1]+nx[N-ii-1], nx[N-ii]);
				blasfeo_sgecp(nu[N-ii-1]+nx[N-ii-1], nx[N-ii], AL+0, 0, 0, AL+1, 0, 0);
				blasfeo_sgecp(nu[N-ii-1]+nx[N-ii-1], ng[N-ii-1], DCt+N-ii-1, 0, 0, AL+1, 0, nx[N-ii]);
				blasfeo_ssyrk_spotrf_ln(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], nx[N-ii]+ng[N-ii-1], AL+0, 0, 0, AL+1, 0, 0, L+N-ii-1, 0, 0, L+N-ii-1, 0, 0);
				}
			else
				{
				blasfeo_ssyrk_spotrf_ln(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], nx[N-ii], AL, 0, 0, AL, 0, 0, L+(N-ii-1), 0, 0, L+(N-ii-1), 0, 0);
				}
			}

//		d_print_strmat(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], L+(N-ii-1), 0, 0);
		}

	// forward substitution

	// first stage
	ii = 0;
	if(ws->scale)
		{
		blasfeo_srowex(nu[ii]+nx[ii], -1.0, L+(ii), nu[ii]+nx[ii], 0, dux+ii, 0);
		blasfeo_strsv_ltn(nu[ii]+nx[ii], L+ii, 0, 0, dux+ii, 0, dux+ii, 0);

		blasfeo_sgecp(nu[ii]+nx[ii], nx[ii+1], BAbt+(ii), 0, 0, AL, 0, 0);
		blasfeo_sgemm_nd(nu[ii]+nx[ii], nx[ii+1], 1.0, AL, 0, 0, Sx+ii+1, nu[ii+1], 0.0, AL, 0, 0, AL, 0, 0);
		blasfeo_sgemm_dn(nu[ii]+nx[ii], nx[ii+1], 1.0, Si+ii, 0, AL, 0, 0, 0.0, AL, 0, 0, AL, 0, 0);
		blasfeo_sveccp(nx[ii+1], res_b+ii, 0, dux+(ii+1), nu[ii+1]);
		d_ptr0 = (dux+ii+1)->pa+nu[ii+1];
		d_ptr1 = (Sx+ii+1)->pa+nu[ii+1];
		for(jj=0; jj<nx[ii+1]; jj++)
			d_ptr0[jj] *= d_ptr1[jj];
		blasfeo_sgemv_t(nu[ii]+nx[ii], nx[ii+1], 1.0, AL, 0, 0, dux+ii, 0, 1.0, dux+(ii+1), nu[ii+1], dux+(ii+1), nu[ii+1]);

		blasfeo_srowex(nx[ii+1], 1.0, L+(ii+1), nu[ii+1]+nx[ii+1], nu[ii+1], tmp_nxM, 0);
		blasfeo_strmv_ltn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dux+(ii+1), nu[ii+1], dpi+ii, 0);
		blasfeo_saxpy(nx[ii+1], 1.0, tmp_nxM, 0, dpi+ii, 0, dpi+ii, 0);
		blasfeo_strmv_lnn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dpi+ii, 0, dpi+ii, 0);
		}
	else
		{
		blasfeo_srowex(nu[ii]+nx[ii], -1.0, L+(ii), nu[ii]+nx[ii], 0, dux+ii, 0);
		blasfeo_strsv_ltn(nu[ii]+nx[ii], L+ii, 0, 0, dux+ii, 0, dux+ii, 0);
		blasfeo_sgemv_t(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, dux+ii, 0, 1.0, res_b+ii, 0, dux+(ii+1), nu[ii+1]);
		blasfeo_srowex(nx[ii+1], 1.0, L+(ii+1), nu[ii+1]+nx[ii+1], nu[ii+1], tmp_nxM, 0);
		blasfeo_strmv_ltn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dux+(ii+1), nu[ii+1], dpi+ii, 0);
		blasfeo_saxpy(nx[ii+1], 1.0, tmp_nxM, 0, dpi+ii, 0, dpi+ii, 0);
		blasfeo_strmv_lnn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dpi+ii, 0, dpi+ii, 0);
		}

//	blasfeo_print_tran_dvec(nu[ii]+nx[ii], dux+ii, 0);

	// middle stages
	for(ii=1; ii<N; ii++)
		{
		if(ws->scale)
			{
			blasfeo_srowex(nu[ii], -1.0, L+(ii), nu[ii]+nx[ii], 0, dux+ii, 0);
			blasfeo_strsv_ltn_mn(nu[ii]+nx[ii], nu[ii], L+ii, 0, 0, dux+ii, 0, dux+ii, 0);

			blasfeo_sgecp(nu[ii]+nx[ii], nx[ii+1], BAbt+(ii), 0, 0, AL, 0, 0);
			blasfeo_sgemm_nd(nu[ii]+nx[ii], nx[ii+1], 1.0, AL, 0, 0, Sx+ii+1, nu[ii+1], 0.0, AL, 0, 0, AL, 0, 0);
			blasfeo_sgemm_dn(nu[ii]+nx[ii], nx[ii+1], 1.0, Si+ii, 0, AL, 0, 0, 0.0, AL, 0, 0, AL, 0, 0);
			blasfeo_sveccp(nx[ii+1], res_b+ii, 0, dux+(ii+1), nu[ii+1]);
			d_ptr0 = (dux+ii+1)->pa+nu[ii+1];
			d_ptr1 = (Sx+ii+1)->pa+nu[ii+1];
			for(jj=0; jj<nx[ii+1]; jj++)
				d_ptr0[jj] *= d_ptr1[jj];
			blasfeo_sgemv_t(nu[ii]+nx[ii], nx[ii+1], 1.0, AL, 0, 0, dux+ii, 0, 1.0, dux+(ii+1), nu[ii+1], dux+(ii+1), nu[ii+1]);

			blasfeo_srowex(nx[ii+1], 1.0, L+(ii+1), nu[ii+1]+nx[ii+1], nu[ii+1], tmp_nxM, 0);
			blasfeo_strmv_ltn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dux+(ii+1), nu[ii+1], dpi+ii, 0);
			blasfeo_saxpy(nx[ii+1], 1.0, tmp_nxM, 0, dpi+ii, 0, dpi+ii, 0);
			blasfeo_strmv_lnn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dpi+ii, 0, dpi+ii, 0);
			}
		else
			{
			blasfeo_srowex(nu[ii], -1.0, L+(ii), nu[ii]+nx[ii], 0, dux+ii, 0);
			blasfeo_strsv_ltn_mn(nu[ii]+nx[ii], nu[ii], L+ii, 0, 0, dux+ii, 0, dux+ii, 0);
			blasfeo_sgemv_t(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, dux+ii, 0, 1.0, res_b+ii, 0, dux+(ii+1), nu[ii+1]);
			blasfeo_srowex(nx[ii+1], 1.0, L+(ii+1), nu[ii+1]+nx[ii+1], nu[ii+1], tmp_nxM, 0);
			blasfeo_strmv_ltn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dux+(ii+1), nu[ii+1], dpi+ii, 0);
			blasfeo_saxpy(nx[ii+1], 1.0, tmp_nxM, 0, dpi+ii, 0, dpi+ii, 0);
			blasfeo_strmv_lnn(nx[ii+1], nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], dpi+ii, 0, dpi+ii, 0);
			}

//		blasfeo_print_tran_dvec(nu[ii]+nx[ii], dux+ii, 0);
		}
	
#if 0
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_svec(nu[ii]+nx[ii], dux+ii, 0);
	for(ii=0; ii<N; ii++)
		blasfeo_print_exp_tran_svec(nx[ii+1], dpi+ii, 0);
//	exit(1);
#endif
#if 0
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_smat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], L+ii, 0, 0);
#endif

	// scale solution back
	if(ws->scale)
		{
		for(ii=0; ii<=N; ii++)
			{
			d_ptr0 = (dux+ii)->pa;
			d_ptr1 = (Si+ii)->pa;
			for(jj=0; jj<nu[ii]+nx[ii]; jj++)
				d_ptr0[jj] *= d_ptr1[jj];
			}
		for(ii=0; ii<N; ii++)
			{
			d_ptr0 = (dpi+ii)->pa;
			d_ptr1 = (Sx+ii+1)->pa+nu[ii+1];
			for(jj=0; jj<nx[ii+1]; jj++)
				d_ptr0[jj] *= d_ptr1[jj];
			}
		}


	// cvt single => double
	for(ii=0; ii<N; ii++)
		{
		m_cvt_s2blasfeo_dvec(nu[ii]+nx[ii], dux+ii, 0, d_dux+ii, 0);
		m_cvt_s2blasfeo_dvec(nx[ii+1], dpi+ii, 0, d_dpi+ii, 0);
		}
	ii = N;
	m_cvt_s2blasfeo_dvec(nu[ii]+nx[ii], dux+ii, 0, d_dux+ii, 0);



//	if(nb>0)
//		{
		for(ii=0; ii<=N; ii++)
			blasfeo_dvecex_sp(nb[ii], 1.0, d_idxb[ii], d_dux+ii, 0, d_dt_lb+ii, 0);
//		}

//	if(ng>0)
//		{
		for(ii=0; ii<=N; ii++)
			blasfeo_dgemv_t(nu[ii]+nx[ii], ng[ii], 1.0, d_DCt+ii, 0, 0, d_dux+ii, 0, 0.0, d_dt_lg+ii, 0, d_dt_lg+ii, 0);
//		}

//	if(nb>0 | ng>0)
//		{
		d_compute_lam_t_hard_qp(cws);
//		}

	return;

	}



// backward Riccati recursion
void m_solve_kkt_step_hard_ocp_qp(struct d_ocp_qp *d_qp, struct s_ocp_qp *s_qp, struct m_ipm_hard_ocp_qp_workspace *ws)
	{

	int N = s_qp->N;
	int *nx = s_qp->nx;
	int *nu = s_qp->nu;
	int *nb = s_qp->nb;
	int *ng = s_qp->ng;

	struct blasfeo_smat *BAbt = s_qp->BAbt;
	struct blasfeo_smat *RSQrq = s_qp->RSQrq;
	struct blasfeo_smat *DCt = s_qp->DCt;
	struct blasfeo_dmat *d_DCt = d_qp->DCt;
	int **idxb = s_qp->idxb;
	int **d_idxb = d_qp->idxb;

	struct blasfeo_smat *L = ws->L;
	struct blasfeo_smat *AL = ws->AL;
	struct blasfeo_dvec *d_res_b = ws->res_b;
	struct blasfeo_dvec *d_res_g = ws->res_g;
	struct blasfeo_svec *res_b = ws->sres_b;
	struct blasfeo_svec *res_g = ws->sres_g;
	struct blasfeo_dvec *d_dux = ws->dux;
	struct blasfeo_dvec *d_dpi = ws->dpi;
	struct blasfeo_svec *dux = ws->sdux;
	struct blasfeo_svec *dpi = ws->sdpi;
	struct blasfeo_dvec *d_dt_lb = ws->dt_lb;
	struct blasfeo_dvec *d_dt_lg = ws->dt_lg;
	struct blasfeo_dvec *d_qx_lg = ws->qx_lg;
	struct blasfeo_dvec *d_qx_lb = ws->qx_lb;
	struct blasfeo_svec *qx_lg = ws->sqx_lg;
	struct blasfeo_svec *qx_lb = ws->sqx_lb;
	struct blasfeo_svec *Pb = ws->Pb;
	struct blasfeo_svec *tmp_nxM = ws->tmp_nxM;

	//
	int ii;

	struct d_ipm_hard_core_qp_workspace *cws = ws->core_workspace;

//	if(nb>0 | ng>0)
//		{
		d_compute_qx_hard_qp(cws);
//		}



	// cvt double => single
	for(ii=0; ii<N; ii++)
		{
		m_cvt_d2blasfeo_svec(nu[ii]+nx[ii], d_res_g+ii, 0, res_g+ii, 0);
		m_cvt_d2blasfeo_svec(nx[ii+1], d_res_b+ii, 0, res_b+ii, 0);
		m_cvt_d2blasfeo_svec(nb[ii], d_qx_lb+ii, 0, qx_lb+ii, 0);
		m_cvt_d2blasfeo_svec(ng[ii], d_qx_lg+ii, 0, qx_lg+ii, 0);
		}
	ii = N;
	m_cvt_d2blasfeo_svec(nu[ii]+nx[ii], d_res_g+ii, 0, res_g+ii, 0);
	m_cvt_d2blasfeo_svec(nb[ii], d_qx_lb+ii, 0, qx_lb+ii, 0);
	m_cvt_d2blasfeo_svec(ng[ii], d_qx_lg+ii, 0, qx_lg+ii, 0);



	// backward substitution

	// last stage
	blasfeo_sveccp(nu[N]+nx[N], res_g+N, 0, dux+N, 0);
	if(nb[N]>0)
		{
		blasfeo_svecad_sp(nb[N], 1.0, qx_lb+N, 0, idxb[N], dux+N, 0);
		}
	// general constraints
	if(ng[N]>0)
		{
		blasfeo_sgemv_n(nu[N]+nx[N], ng[N], 1.0, DCt+N, 0, 0, qx_lg+N, 0, 1.0, dux+N, 0, dux+N, 0);
		}

	// middle stages
	for(ii=0; ii<N-1; ii++)
		{
		blasfeo_sveccp(nu[N-ii-1]+nx[N-ii-1], res_g+N-ii-1, 0, dux+N-ii-1, 0);
		if(nb[N-ii-1]>0)
			{
			blasfeo_svecad_sp(nb[N-ii-1], 1.0, qx_lb+N-ii-1, 0, idxb[N-ii-1], dux+N-ii-1, 0);
			}
		if(ng[N-ii-1]>0)
			{
			blasfeo_sgemv_n(nu[N-ii-1]+nx[N-ii-1], ng[N-ii-1], 1.0, DCt+N-ii-1, 0, 0, qx_lg+N-ii-1, 0, 1.0, dux+N-ii-1, 0, dux+N-ii-1, 0);
			}
		if(ws->compute_Pb)
			{
			blasfeo_strmv_ltn(nx[N-ii], nx[N-ii], L+(N-ii), nu[N-ii], nu[N-ii], res_b+N-ii-1, 0, Pb+(N-ii-1), 0);
			blasfeo_strmv_lnn(nx[N-ii], nx[N-ii], L+(N-ii), nu[N-ii], nu[N-ii], Pb+(N-ii-1), 0, Pb+(N-ii-1), 0);
			}
		blasfeo_saxpy(nx[N-ii], 1.0, dux+N-ii, nu[N-ii], Pb+N-ii-1, 0, tmp_nxM, 0);
		blasfeo_sgemv_n(nu[N-ii-1]+nx[N-ii-1], nx[N-ii], 1.0, BAbt+N-ii-1, 0, 0, tmp_nxM, 0, 1.0, dux+N-ii-1, 0, dux+N-ii-1, 0);
		blasfeo_strsv_lnn_mn(nu[N-ii-1]+nx[N-ii-1], nu[N-ii-1], L+N-ii-1, 0, 0, dux+N-ii-1, 0, dux+N-ii-1, 0);
		}

	// first stage
	ii = N-1;
	blasfeo_sveccp(nu[N-ii-1]+nx[N-ii-1], res_g+N-ii-1, 0, dux+N-ii-1, 0);
	if(nb[N-ii-1]>0)
		{
		blasfeo_svecad_sp(nb[N-ii-1], 1.0, qx_lb+N-ii-1, 0, idxb[N-ii-1], dux+N-ii-1, 0);
		}
	if(ng[N-ii-1]>0)
		{
		blasfeo_sgemv_n(nu[N-ii-1]+nx[N-ii-1], ng[N-ii-1], 1.0, DCt+N-ii-1, 0, 0, qx_lg+N-ii-1, 0, 1.0, dux+N-ii-1, 0, dux+N-ii-1, 0);
		}
	if(ws->compute_Pb)
		{
		blasfeo_strmv_ltn(nx[N-ii], nx[N-ii], L+(N-ii), nu[N-ii], nu[N-ii], res_b+N-ii-1, 0, Pb+(N-ii-1), 0);
		blasfeo_strmv_lnn(nx[N-ii], nx[N-ii], L+(N-ii), nu[N-ii], nu[N-ii], Pb+(N-ii-1), 0, Pb+(N-ii-1), 0);
		}
	blasfeo_saxpy(nx[N-ii], 1.0, dux+N-ii, nu[N-ii], Pb+N-ii-1, 0, tmp_nxM, 0);
	blasfeo_sgemv_n(nu[N-ii-1]+nx[N-ii-1], nx[N-ii], 1.0, BAbt+N-ii-1, 0, 0, tmp_nxM, 0, 1.0, dux+N-ii-1, 0, dux+N-ii-1, 0);
	blasfeo_strsv_lnn(nu[N-ii-1]+nx[N-ii-1], L+N-ii-1, 0, 0, dux+N-ii-1, 0, dux+N-ii-1, 0);

	// first stage
	ii = 0;
	blasfeo_sveccp(nx[ii+1], dux+ii+1, nu[ii+1], dpi+ii, 0);
	blasfeo_svecsc(nu[ii]+nx[ii], -1.0, dux+ii, 0);
	blasfeo_strsv_ltn(nu[ii]+nx[ii], L+ii, 0, 0, dux+ii, 0, dux+ii, 0);
	blasfeo_sgemv_t(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, dux+ii, 0, 1.0, res_b+ii, 0, dux+ii+1, nu[ii+1]);
	blasfeo_sveccp(nx[ii+1], dux+ii+1, nu[ii+1], tmp_nxM, 0);
	blasfeo_strmv_ltn(nx[ii+1], nx[ii+1], L+ii+1, nu[ii+1], nu[ii+1], tmp_nxM, 0, tmp_nxM, 0);
	blasfeo_strmv_lnn(nx[ii+1], nx[ii+1], L+ii+1, nu[ii+1], nu[ii+1], tmp_nxM, 0, tmp_nxM, 0);
	blasfeo_saxpy(nx[ii+1], 1.0, tmp_nxM, 0, dpi+ii, 0, dpi+ii, 0);

	// middle stages
	for(ii=1; ii<N; ii++)
		{
		blasfeo_sveccp(nx[ii+1], dux+ii+1, nu[ii+1], dpi+ii, 0);
		blasfeo_svecsc(nu[ii], -1.0, dux+ii, 0);
		blasfeo_strsv_ltn_mn(nu[ii]+nx[ii], nu[ii], L+ii, 0, 0, dux+ii, 0, dux+ii, 0);
		blasfeo_sgemv_t(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, dux+ii, 0, 1.0, res_b+ii, 0, dux+ii+1, nu[ii+1]);
		blasfeo_sveccp(nx[ii+1], dux+ii+1, nu[ii+1], tmp_nxM, 0);
		blasfeo_strmv_ltn(nx[ii+1], nx[ii+1], L+ii+1, nu[ii+1], nu[ii+1], tmp_nxM, 0, tmp_nxM, 0);
		blasfeo_strmv_lnn(nx[ii+1], nx[ii+1], L+ii+1, nu[ii+1], nu[ii+1], tmp_nxM, 0, tmp_nxM, 0);
		blasfeo_saxpy(nx[ii+1], 1.0, tmp_nxM, 0, dpi+ii, 0, dpi+ii, 0);
		}



	// cvt single => double
	for(ii=0; ii<N; ii++)
		{
		m_cvt_s2blasfeo_dvec(nu[ii]+nx[ii], dux+ii, 0, d_dux+ii, 0);
		m_cvt_s2blasfeo_dvec(nx[ii+1], dpi+ii, 0, d_dpi+ii, 0);
		}
	ii = N;
	m_cvt_s2blasfeo_dvec(nu[ii]+nx[ii], dux+ii, 0, d_dux+ii, 0);



//	if(nb>0)
//		{
		for(ii=0; ii<=N; ii++)
			blasfeo_dvecex_sp(nb[ii], 1.0, d_idxb[ii], d_dux+ii, 0, d_dt_lb+ii, 0);
//		}

//	if(ng>0)
//		{
		for(ii=0; ii<=N; ii++)
			blasfeo_dgemv_t(nu[ii]+nx[ii], ng[ii], 1.0, d_DCt+ii, 0, 0, d_dux+ii, 0, 0.0, d_dt_lg+ii, 0, d_dt_lg+ii, 0);
//		}

//	if(nb>0 | ng>0)
//		{
		d_compute_lam_t_hard_qp(cws);
//		}

	return;

	}


