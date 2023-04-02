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



// backward Riccati recursion
void OCP_QP_FACT_SOLVE_KKT_UNCONSTR(struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct OCP_QP_IPM_ARG *arg, struct OCP_QP_IPM_WS *ws)
	{

	int ii;

	int N = qp->dim->N;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	struct STRMAT *BAbt = qp->BAbt;
	struct STRMAT *RSQrq = qp->RSQrq;
	struct STRVEC *b = qp->b;
	struct STRVEC *rqz = qp->rqz;

	struct STRVEC *ux = qp_sol->ux;
	struct STRVEC *pi = qp_sol->pi;

	struct STRMAT *L = ws->L;
	struct STRVEC *l = ws->l;
	struct STRMAT *AL = ws->AL;

	struct STRVEC *tmp_nuxM = ws->tmp_nuxM;

	if(ws->square_root_alg)
		{
		ws->valid_ric_p = 0;

		// factorization and backward substitution

		// last stage
		ROWIN(nu[N]+nx[N], 1.0, rqz+N, 0, RSQrq+N, nu[N]+nx[N], 0);
		DIARE(nu[N]+nx[N], arg->reg_prim, RSQrq+N, 0, 0);
		POTRF_L_MN(nu[N]+nx[N]+1, nu[N]+nx[N], RSQrq+N, 0, 0, L+N, 0, 0);
		DIARE(nu[N]+nx[N], -arg->reg_prim, RSQrq+N, 0, 0);

		// middle stages
		for(ii=0; ii<N; ii++)
			{
			ROWIN(nx[N-ii], 1.0, b+N-ii-1, 0, BAbt+N-ii-1, nu[N-ii-1]+nx[N-ii-1], 0);
			TRMM_RLNN(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], 1.0, L+(N-ii), nu[N-ii], nu[N-ii], BAbt+N-ii-1, 0, 0, AL, 0, 0);
			GEAD(1, nx[N-ii], 1.0, L+N-ii, nu[N-ii]+nx[N-ii], nu[N-ii], AL, nu[N-ii-1]+nx[N-ii-1], 0);

			ROWIN(nu[N-ii-1]+nx[N-ii-1], 1.0, rqz+N-ii-1, 0, RSQrq+N-ii-1, nu[N-ii-1]+nx[N-ii-1], 0);
			DIARE(nu[N-ii-1]+nx[N-ii-1], arg->reg_prim, RSQrq+N-ii-1, 0, 0);
			SYRK_POTRF_LN_MN(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], nx[N-ii], AL, 0, 0, AL, 0, 0, RSQrq+(N-ii-1), 0, 0, L+(N-ii-1), 0, 0);
			DIARE(nu[N-ii-1]+nx[N-ii-1], -arg->reg_prim, RSQrq+N-ii-1, 0, 0);
			}

		// forward substitution

		// first stage
		ii = 0;
		ROWEX(nu[ii]+nx[ii], -1.0, L+ii, nu[ii]+nx[ii], 0, ux+ii, 0);
		VECCPSC(nu[ii]+nx[ii], -1.0, ux+ii, 0, l+ii, 0);
		ROWEX(nx[ii+1], 1.0, L+ii+1, nu[ii+1]+nx[ii+1], nu[ii+1], tmp_nuxM, 0);
		VECCP(nx[ii+1], tmp_nuxM, 0, l+ii+1, nu[ii+1]); // TODO remove tmp_nuxM and use l instead !!!!!
		TRSV_LTN(nu[ii]+nx[ii], L+ii, 0, 0, ux+ii, 0, ux+ii, 0);
		GEMV_T(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, ux+ii, 0, 1.0, b+ii, 0, ux+ii+1, nu[ii+1]);
		if(arg->comp_dual_sol_eq)
			{
			TRMV_LTN(nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], ux+(ii+1), nu[ii+1], pi+ii, 0);
			AXPY(nx[ii+1], 1.0, tmp_nuxM, 0, pi+ii, 0, pi+ii, 0);
			TRMV_LNN(nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], pi+ii, 0, pi+ii, 0);
			}

		// middle stages
		for(ii=1; ii<N; ii++)
			{
			ROWEX(nu[ii], -1.0, L+ii, nu[ii]+nx[ii], 0, ux+ii, 0);
			VECCPSC(nu[ii], -1.0, ux+ii, 0, l+ii, 0);
			ROWEX(nx[ii+1], 1.0, L+(ii+1), nu[ii+1]+nx[ii+1], nu[ii+1], tmp_nuxM, 0);
			VECCP(nx[ii+1], tmp_nuxM, 0, l+ii+1, nu[ii+1]); // TODO remove tmp_nuxM and use l instead !!!!!
			TRSV_LTN_MN(nu[ii]+nx[ii], nu[ii], L+ii, 0, 0, ux+ii, 0, ux+ii, 0);
			GEMV_T(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, ux+ii, 0, 1.0, b+ii, 0, ux+(ii+1), nu[ii+1]);
			if(arg->comp_dual_sol_eq)
				{
				TRMV_LTN(nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], ux+(ii+1), nu[ii+1], pi+ii, 0);
				AXPY(nx[ii+1], 1.0, tmp_nuxM, 0, pi+ii, 0, pi+ii, 0);
				TRMV_LNN(nx[ii+1], L+(ii+1), nu[ii+1], nu[ii+1], pi+ii, 0, pi+ii, 0);
				}
			}
		
		// last stage
		ii = N;
		ROWEX(nu[ii], -1.0, L+ii, nu[ii]+nx[ii], 0, ux+ii, 0);
		VECCPSC(nu[ii], -1.0, ux+ii, 0, l+ii, 0);
		TRSV_LTN_MN(nu[ii]+nx[ii], nu[ii], L+ii, 0, 0, ux+ii, 0, ux+ii, 0);

		}
	else
		{
		ws->valid_ric_p = 1;

		struct STRMAT *P = ws->P;
		struct STRMAT *Ls = ws->Ls;

		// factorization and backward substitution

		// last stage
		ROWIN(nu[N]+nx[N], 1.0, rqz+N, 0, RSQrq+N, nu[N]+nx[N], 0);
		DIARE(nu[N], arg->reg_prim, RSQrq+N, 0, 0);
		POTRF_L_MN(nu[N]+nx[N]+1, nu[N], RSQrq+N, 0, 0, L+N, 0, 0);
		DIARE(nu[N], -arg->reg_prim, RSQrq+N, 0, 0);
		GECP(nx[N]+1, nu[N], L+N, nu[N], 0, Ls, 0, 0);
		SYRK_LN_MN(nx[N]+1, nx[N], nu[N], -1.0, Ls, 0, 0, Ls, 0, 0, 1.0, RSQrq+N, nu[N], nu[N], P+N, 0, 0);
		TRTR_L(nx[N], P+N, 0, 0, P+N, 0, 0);

		// middle statges
		for(ii=0; ii<N-1; ii++)
			{
			ROWIN(nx[N-ii], 1.0, b+N-ii-1, 0, BAbt+N-ii-1, nu[N-ii-1]+nx[N-ii-1], 0);
			GEMM_NT(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], nx[N-ii], 1.0, BAbt+N-ii-1, 0, 0, P+N-ii, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm
			GEAD(1, nx[N-ii], 1.0, P+N-ii, nx[N-ii], 0, AL, nu[N-ii-1]+nx[N-ii-1], 0);
			ROWIN(nu[N-ii-1]+nx[N-ii-1], 1.0, rqz+N-ii-1, 0, RSQrq+N-ii-1, nu[N-ii-1]+nx[N-ii-1], 0);
			DIARE(nu[N-ii-1], arg->reg_prim, RSQrq+N-ii-1, 0, 0);
			SYRK_LN_MN(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], nx[N-ii], 1.0, AL, 0, 0, BAbt+N-ii-1, 0, 0, 1.0, RSQrq+N-ii-1, 0, 0, L+N-ii-1, 0, 0);
			DIARE(nu[N-ii-1], -arg->reg_prim, RSQrq+N-ii-1, 0, 0);
			POTRF_L_MN(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1], L+N-ii-1, 0, 0, L+N-ii-1, 0, 0);
			GECP(nx[N-ii-1]+1, nu[N-ii-1], L+N-ii-1, nu[N-ii-1], 0, Ls, 0, 0);
			SYRK_LN_MN(nx[N-ii-1]+1, nx[N-ii-1], nu[N-ii-1], -1.0, Ls, 0, 0, Ls, 0, 0, 1.0, L+N-ii-1, nu[N-ii-1], nu[N-ii-1], P+N-ii-1, 0, 0);
			TRTR_L(nx[N-ii-1], P+N-ii-1, 0, 0, P+N-ii-1, 0, 0);
			}

		// first stage: factorize P in L too
		if(N>0)
			{
			ROWIN(nx[N-ii], 1.0, b+N-ii-1, 0, BAbt+N-ii-1, nu[N-ii-1]+nx[N-ii-1], 0);
			GEMM_NT(nu[N-ii-1]+nx[N-ii-1]+1, nx[N-ii], nx[N-ii], 1.0, BAbt+N-ii-1, 0, 0, P+N-ii, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm
			GEAD(1, nx[N-ii], 1.0, P+N-ii, nx[N-ii], 0, AL, nu[N-ii-1]+nx[N-ii-1], 0);
			ROWIN(nu[N-ii-1]+nx[N-ii-1], 1.0, rqz+N-ii-1, 0, RSQrq+N-ii-1, nu[N-ii-1]+nx[N-ii-1], 0);
			DIARE(nu[N-ii-1], arg->reg_prim, RSQrq+N-ii-1, 0, 0);
			SYRK_POTRF_LN_MN(nu[N-ii-1]+nx[N-ii-1]+1, nu[N-ii-1]+nx[N-ii-1], nx[N-ii], AL, 0, 0, BAbt+N-ii-1, 0, 0, RSQrq+N-ii-1, 0, 0, L+N-ii-1, 0, 0);
			DIARE(nu[N-ii-1], -arg->reg_prim, RSQrq+N-ii-1, 0, 0);
			}

		// forward substitution

		// first stage
		ii = 0;
		ROWEX(nu[ii]+nx[ii], -1.0, L+ii, nu[ii]+nx[ii], 0, ux+ii, 0);
		VECCPSC(nu[ii]+nx[ii], -1.0, ux+ii, 0, l+ii, 0);
		ROWEX(nx[ii+1], 1.0, P+ii+1, nx[ii+1], 0, tmp_nuxM, 0);
		VECCP(nx[ii+1], tmp_nuxM, 0, l+ii+1, nu[ii+1]); // TODO remove tmp_nuxM and use l instead !!!!!
		TRSV_LTN(nu[ii]+nx[ii], L+ii, 0, 0, ux+ii, 0, ux+ii, 0);
		GEMV_T(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, ux+ii, 0, 1.0, b+ii, 0, ux+ii+1, nu[ii+1]);
		if(arg->comp_dual_sol_eq)
			{
			GEMV_N(nx[ii+1], nx[ii+1], 1.0, P+ii+1, 0, 0, ux+ii+1, nu[ii+1], 1.0, tmp_nuxM, 0, pi+ii, 0);
			}

		// middle stages
		for(ii=1; ii<N; ii++)
			{
			ROWEX(nu[ii], -1.0, L+ii, nu[ii]+nx[ii], 0, ux+ii, 0);
			VECCPSC(nu[ii], -1.0, ux+ii, 0, l+ii, 0);
			ROWEX(nx[ii+1], 1.0, P+ii+1, nx[ii+1], 0, tmp_nuxM, 0);
			VECCP(nx[ii+1], tmp_nuxM, 0, l+ii+1, nu[ii+1]); // TODO remove tmp_nuxM and use l instead !!!!!
			TRSV_LTN_MN(nu[ii]+nx[ii], nu[ii], L+ii, 0, 0, ux+ii, 0, ux+ii, 0);
			GEMV_T(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, ux+ii, 0, 1.0, b+ii, 0, ux+(ii+1), nu[ii+1]);
			if(arg->comp_dual_sol_eq)
				{
				GEMV_N(nx[ii+1], nx[ii+1], 1.0, P+ii+1, 0, 0, ux+ii+1, nu[ii+1], 1.0, tmp_nuxM, 0, pi+ii, 0);
				}
			}

		// last stage
		ii = N;
		ROWEX(nu[ii], -1.0, L+ii, nu[ii]+nx[ii], 0, ux+ii, 0);
		VECCPSC(nu[ii], -1.0, ux+ii, 0, l+ii, 0);
		TRSV_LTN_MN(nu[ii]+nx[ii], nu[ii], L+ii, 0, 0, ux+ii, 0, ux+ii, 0);

		}

	return;

	}



static void COND_SLACKS_FACT_SOLVE(int ss, struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct OCP_QP_IPM_ARG *arg, struct OCP_QP_IPM_WS *ws)
	{

	int ii, idx;

	int nx0 = qp->dim->nx[ss];
	int nu0 = qp->dim->nu[ss];
	int nb0 = qp->dim->nb[ss];
	int ng0 = qp->dim->ng[ss];
	int ns0 = qp->dim->ns[ss];

	struct STRVEC *Z = qp->Z+ss;
	int *idxs_rev0 = qp->idxs_rev[ss];

//	struct STRVEC *res_g = ws->res->res_g+ss; // TODO !!!
	struct STRVEC *res_g = qp->rqz+ss;

//	struct STRVEC *dux = ws->sol_step->ux+ss; // TODO !!!
	struct STRVEC *dux = qp_sol->ux+ss;

	struct STRVEC *Gamma = ws->Gamma+ss;
	struct STRVEC *gamma = ws->gamma+ss;
	struct STRVEC *Zs_inv = ws->Zs_inv+ss;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;

	REAL *ptr_Gamma = Gamma->pa;
	REAL *ptr_gamma = gamma->pa;
	REAL *ptr_Z = Z->pa;
	REAL *ptr_Zs_inv = Zs_inv->pa;
	REAL *ptr_dux = dux->pa;
	REAL *ptr_res_g = res_g->pa;
	REAL *ptr_tmp0 = (tmp_nbgM+0)->pa;
	REAL *ptr_tmp1 = (tmp_nbgM+1)->pa;
	REAL *ptr_tmp2 = (tmp_nbgM+2)->pa;
	REAL *ptr_tmp3 = (tmp_nbgM+3)->pa;

	REAL tmp0, tmp1;

	VECCP(nb0+ng0, Gamma, 0, tmp_nbgM+0, 0);
	VECCP(nb0+ng0, Gamma, nb0+ng0, tmp_nbgM+1, 0);
	VECCP(nb0+ng0, gamma, 0, tmp_nbgM+2, 0);
	VECCP(nb0+ng0, gamma, nb0+ng0, tmp_nbgM+3, 0);

	// idxs_rev
	for(ii=0; ii<nb0+ng0; ii++)
		{
		idx = idxs_rev0[ii];
		if(idx!=-1)
			{
			// ii   constr index
			// idx <= slack index
			ptr_Zs_inv[0+idx]   = ptr_Z[0+idx]   + arg->reg_prim + ptr_Gamma[0+ii]       + ptr_Gamma[2*nb0+2*ng0+idx];
			ptr_Zs_inv[ns0+idx] = ptr_Z[ns0+idx] + arg->reg_prim + ptr_Gamma[nb0+ng0+ii] + ptr_Gamma[2*nb0+2*ng0+ns0+idx];
			ptr_dux[nu0+nx0+idx]      = ptr_res_g[nu0+nx0+idx]     + ptr_gamma[0+ii]       + ptr_gamma[2*nb0+2*ng0+idx];
			ptr_dux[nu0+nx0+ns0+idx]  = ptr_res_g[nu0+nx0+ns0+idx] + ptr_gamma[nb0+ng0+ii] + ptr_gamma[2*nb0+2*ng0+ns0+idx];
			ptr_Zs_inv[0+idx]   = 1.0/ptr_Zs_inv[0+idx];
			ptr_Zs_inv[ns0+idx] = 1.0/ptr_Zs_inv[ns0+idx];
			tmp0 = ptr_dux[nu0+nx0+idx]*ptr_Zs_inv[0+idx];
			tmp1 = ptr_dux[nu0+nx0+ns0+idx]*ptr_Zs_inv[ns0+idx];
			ptr_tmp0[ii] = ptr_tmp0[ii] - ptr_tmp0[ii]*ptr_Zs_inv[0+idx]*ptr_tmp0[ii];
			ptr_tmp1[ii] = ptr_tmp1[ii] - ptr_tmp1[ii]*ptr_Zs_inv[ns0+idx]*ptr_tmp1[ii];
			ptr_tmp2[ii] = ptr_tmp2[ii] - ptr_Gamma[0+ii]*tmp0;
			ptr_tmp3[ii] = ptr_tmp3[ii] - ptr_Gamma[nb0+ng0+ii]*tmp1;
			}
		}

	AXPY(nb0+ng0,  1.0, tmp_nbgM+1, 0, tmp_nbgM+0, 0, tmp_nbgM+0, 0);
	AXPY(nb0+ng0, -1.0, tmp_nbgM+3, 0, tmp_nbgM+2, 0, tmp_nbgM+1, 0);

	return;

	}



static void COND_SLACKS_SOLVE(int ss, struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct OCP_QP_IPM_WS *ws)
	{

	int ii, idx;

	int nx0 = qp->dim->nx[ss];
	int nu0 = qp->dim->nu[ss];
	int nb0 = qp->dim->nb[ss];
	int ng0 = qp->dim->ng[ss];
	int ns0 = qp->dim->ns[ss];

	int *idxs_rev0 = qp->idxs_rev[ss];

//	struct STRVEC *res_g = ws->res->res_g+ss; // TODO !!!
	struct STRVEC *res_g = qp->rqz+ss;

//	struct STRVEC *dux = ws->sol_step->ux+ss; // TODO !!!
	struct STRVEC *dux = qp_sol->ux+ss;

	struct STRVEC *Gamma = ws->Gamma+ss;
	struct STRVEC *gamma = ws->gamma+ss;
	struct STRVEC *Zs_inv = ws->Zs_inv+ss;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;

	REAL *ptr_Gamma = Gamma->pa;
	REAL *ptr_gamma = gamma->pa;
	REAL *ptr_Zs_inv = Zs_inv->pa;
	REAL *ptr_dux = dux->pa;
	REAL *ptr_res_g = res_g->pa;
	REAL *ptr_tmp2 = (tmp_nbgM+2)->pa;
	REAL *ptr_tmp3 = (tmp_nbgM+3)->pa;

	REAL tmp0, tmp1;

	VECCP(nb0+ng0, gamma, 0, tmp_nbgM+2, 0);
	VECCP(nb0+ng0, gamma, nb0+ng0, tmp_nbgM+3, 0);

	// idxs_rev
	for(ii=0; ii<nb0+ng0; ii++)
		{
		idx = idxs_rev0[ii];
		if(idx!=-1)
			{
			// ii  <= constr index
			// idx <= slack index
			ptr_dux[nu0+nx0+idx]      = ptr_res_g[nu0+nx0+idx]     + ptr_gamma[0+ii]       + ptr_gamma[2*nb0+2*ng0+idx];
			ptr_dux[nu0+nx0+ns0+idx]  = ptr_res_g[nu0+nx0+ns0+idx] + ptr_gamma[nb0+ng0+ii] + ptr_gamma[2*nb0+2*ng0+ns0+idx];
			tmp0 = ptr_dux[nu0+nx0+idx]*ptr_Zs_inv[0+idx];
			tmp1 = ptr_dux[nu0+nx0+ns0+idx]*ptr_Zs_inv[ns0+idx];
			ptr_tmp2[ii] = ptr_tmp2[ii] - ptr_Gamma[0+ii]*tmp0;
			ptr_tmp3[ii] = ptr_tmp3[ii] - ptr_Gamma[nb0+ng0+ii]*tmp1;
			}
		}

	AXPY(nb0+ng0, -1.0, tmp_nbgM+3, 0, tmp_nbgM+2, 0, tmp_nbgM+1, 0);

	return;

	}



static void EXPAND_SLACKS(int ss, struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct OCP_QP_IPM_WS *ws)
	{

	int ii, idx;

	int nx0 = qp->dim->nx[ss];
	int nu0 = qp->dim->nu[ss];
	int nb0 = qp->dim->nb[ss];
	int ng0 = qp->dim->ng[ss];
	int ns0 = qp->dim->ns[ss];

	int *idxs_rev0 = qp->idxs_rev[ss];

	struct STRVEC *dux = qp_sol->ux+ss;
	struct STRVEC *dt = qp_sol->t+ss;

	struct STRVEC *Gamma = ws->Gamma+ss;
	struct STRVEC *Zs_inv = ws->Zs_inv+ss;

	REAL *ptr_Gamma = Gamma->pa;
	REAL *ptr_dux = dux->pa;
	REAL *ptr_dt = dt->pa;
	REAL *ptr_Zs_inv = Zs_inv->pa;

	// idxs_rev
	for(ii=0; ii<nb0+ng0; ii++)
		{
		idx = idxs_rev0[ii];
		if(idx!=-1)
			{
			// ii  <= constr index
			// idx <= slack index
			ptr_dux[nu0+nx0+idx]     = - ptr_Zs_inv[0+idx]   * (ptr_dux[nu0+nx0+idx]     + ptr_dt[ii]*ptr_Gamma[ii]);
			ptr_dux[nu0+nx0+ns0+idx] = - ptr_Zs_inv[ns0+idx] * (ptr_dux[nu0+nx0+ns0+idx] + ptr_dt[nb0+ng0+ii]*ptr_Gamma[nb0+ng0+ii]);
			ptr_dt[2*nb0+2*ng0+idx]     = ptr_dux[nu0+nx0+idx];
			ptr_dt[2*nb0+2*ng0+ns0+idx] = ptr_dux[nu0+nx0+ns0+idx];
			ptr_dt[0+ii]       = ptr_dt[0+ii]   + ptr_dux[nu0+nx0+idx];
			ptr_dt[nb0+ng0+ii] = ptr_dt[nb0+ng0+ii] + ptr_dux[nu0+nx0+ns0+idx];
			}
		}

	return;

	}



// backward Riccati recursion
void OCP_QP_FACT_SOLVE_KKT_STEP(struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct OCP_QP_IPM_ARG *arg, struct OCP_QP_IPM_WS *ws)
	{

	int N = qp->dim->N;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	struct STRMAT *BAbt = qp->BAbt;
	struct STRMAT *RSQrq = qp->RSQrq;
	struct STRMAT *DCt = qp->DCt;
	struct STRVEC *Z = qp->Z;
	struct STRVEC *res_g = qp->rqz;
	struct STRVEC *res_b = qp->b;
	struct STRVEC *res_d = qp->d;
	struct STRVEC *res_m = qp->m;
	int **idxb = qp->idxb;

	struct STRVEC *dux = qp_sol->ux;
	struct STRVEC *dpi = qp_sol->pi;
	struct STRVEC *dlam = qp_sol->lam;
	struct STRVEC *dt = qp_sol->t;

	struct STRMAT *L = ws->L;
	struct STRVEC *l = ws->l;
	struct STRMAT *AL = ws->AL;
	struct STRVEC *Gamma = ws->Gamma;
	struct STRVEC *gamma = ws->gamma;
	struct STRVEC *Pb = ws->Pb;
	struct STRVEC *Zs_inv = ws->Zs_inv;
	struct STRVEC *tmp_nuxM = ws->tmp_nuxM;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;

	REAL *ptr0, *ptr1, *ptr2, *ptr3;

	//
	int ii, nn, ss, idx;

	struct CORE_QP_IPM_WORKSPACE *cws = ws->core_workspace;

	COMPUTE_GAMMA_GAMMA_QP(res_d[0].pa, res_m[0].pa, cws);

	if(ws->square_root_alg)
		{
		ws->valid_ric_p = 0;

		// factorization and backward substitution

		// last stage
		ss = N;
#if defined(DOUBLE_PRECISION)
		TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#else
		GECP(nu[ss]+nx[ss], nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#endif
		DIARE(nu[ss]+nx[ss], arg->reg_prim, L+ss, 0, 0);
		ROWIN(nu[ss]+nx[ss], 1.0, res_g+ss, 0, L+ss, nu[ss]+nx[ss], 0);

		if(ns[ss]>0)
			{
			COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
			}
		else if(nb[ss]+ng[ss]>0)
			{
			AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
			AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
			}
		if(nb[ss]>0)
			{
			DIAAD_SP(nb[ss], 1.0, tmp_nbgM+0, 0, idxb[ss], L+ss, 0, 0);
			ROWAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], L+ss, nu[ss]+nx[ss], 0);
			}
		if(ng[ss]>0)
			{
			GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, AL+0, 0, 0, AL+0, 0, 0);
			ROWIN(ng[ss], 1.0, tmp_nbgM+1, nb[ss], AL+0, nu[ss]+nx[ss], 0);
			SYRK_POTRF_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], ng[ss], AL+0, 0, 0, DCt+ss, 0, 0, L+ss, 0, 0, L+ss, 0, 0);
			}
		else
			{
			POTRF_L_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], L+ss, 0, 0, L+ss, 0, 0);
			}

		// middle stages
		for(nn=0; nn<N; nn++)
			{
			ss = N-nn-1;
			ROWIN(nx[ss+1], 1.0, res_b+ss, 0, BAbt+ss, nu[ss]+nx[ss], 0);
			TRMM_RLNN(nu[ss]+nx[ss]+1, nx[ss+1], 1.0, L+ss+1, nu[ss+1], nu[ss+1], BAbt+ss, 0, 0, AL, 0, 0);
			ROWEX(nx[ss+1], 1.0, AL, nu[ss]+nx[ss], 0, tmp_nuxM, 0);
			TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, Pb+ss, 0);
			GEAD(1, nx[ss+1], 1.0, L+ss+1, nu[ss+1]+nx[ss+1], nu[ss+1], AL, nu[ss]+nx[ss], 0);

#if defined(DOUBLE_PRECISION)
			TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0);
#else
			GECP(nu[ss]+nx[ss], nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0);
#endif
			DIARE(nu[ss]+nx[ss], arg->reg_prim, L+ss, 0, 0);
			ROWIN(nu[ss]+nx[ss], 1.0, res_g+ss, 0, L+ss, nu[ss]+nx[ss], 0);

			if(ns[ss]>0)
				{
				COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
				}
			else if(nb[ss]+ng[ss]>0)
				{
				AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
				AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
				}
			if(nb[ss]>0)
				{
				DIAAD_SP(nb[ss], 1.0, tmp_nbgM+0, 0, idxb[ss], L+ss, 0, 0);
				ROWAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], L+ss, nu[ss]+nx[ss], 0);
				}
			if(ng[ss]>0)
				{
				GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, AL+0, 0, nx[ss+1], AL+0, 0, nx[ss+1]);
				ROWIN(ng[ss], 1.0, tmp_nbgM+1, nb[ss], AL+0, nu[ss]+nx[ss], nx[ss+1]);
				GECP(nu[ss]+nx[ss], nx[ss+1], AL+0, 0, 0, AL+1, 0, 0);
				GECP(nu[ss]+nx[ss], ng[ss], DCt+ss, 0, 0, AL+1, 0, nx[ss+1]);
				SYRK_POTRF_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], nx[ss+1]+ng[ss], AL+0, 0, 0, AL+1, 0, 0, L+ss, 0, 0, L+ss, 0, 0);
				}
			else
				{
				SYRK_POTRF_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], nx[ss+1], AL, 0, 0, AL, 0, 0, L+ss, 0, 0, L+ss, 0, 0);
				}

			}

		// forward substitution

		// first stage
		ss = 0;
		ROWEX(nu[ss]+nx[ss], -1.0, L+ss, nu[ss]+nx[ss], 0, dux+ss, 0);
		VECCPSC(nu[ss]+nx[ss], -1.0, dux+ss, 0, l+ss, 0);
		ROWEX(nx[ss+1], 1.0, L+ss+1, nu[ss+1]+nx[ss+1], nu[ss+1], tmp_nuxM, 0);
		VECCP(nx[ss+1], tmp_nuxM, 0, l+ss+1, nu[ss+1]); // TODO remove tmp_nuxM and use l instead !!!!!
		TRSV_LTN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
		GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
		if(arg->comp_dual_sol_eq)
			{
			TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
			AXPY(nx[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0, dpi+ss, 0);
			TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], dpi+ss, 0, dpi+ss, 0);
			}

		// middle stages
		for(ss=1; ss<N; ss++)
			{
			ROWEX(nu[ss], -1.0, L+ss, nu[ss]+nx[ss], 0, dux+ss, 0);
			VECCPSC(nu[ss], -1.0, dux+ss, 0, l+ss, 0);
			ROWEX(nx[ss+1], 1.0, L+ss+1, nu[ss+1]+nx[ss+1], nu[ss+1], tmp_nuxM, 0);
			VECCP(nx[ss+1], tmp_nuxM, 0, l+ss+1, nu[ss+1]); // TODO remove tmp_nuxM and use l instead !!!!!
			TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
			GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+(ss+1), nu[ss+1]);
			if(arg->comp_dual_sol_eq)
				{
				TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
				AXPY(nx[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0, dpi+ss, 0);
				TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], dpi+ss, 0, dpi+ss, 0);
				}
			}

		ss = N;
		ROWEX(nu[ss], -1.0, L+ss, nu[ss]+nx[ss], 0, dux+ss, 0);
		VECCPSC(nu[ss], -1.0, dux+ss, 0, l+ss, 0);
		TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

		}
	else // classical algorithm
		{
		ws->valid_ric_p = 1;

		struct STRMAT *P = ws->P;
		struct STRMAT *Ls = ws->Ls;

		// factorization and backward substitution

		// last stage
		ss = N;
#if defined(DOUBLE_PRECISION)
		TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#else
		GECP(nu[ss]+nx[ss], nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#endif
		DIARE(nu[ss]+nx[ss], arg->reg_prim, L+ss, 0, 0);
		ROWIN(nu[ss]+nx[ss], 1.0, res_g+ss, 0, L+ss, nu[ss]+nx[ss], 0);

		if(ns[ss]>0)
			{
			COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
			}
		else if(nb[ss]+ng[ss]>0)
			{
			AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
			AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
			}
		if(nb[ss]>0)
			{
			DIAAD_SP(nb[ss], 1.0, tmp_nbgM+0, 0, idxb[ss], L+ss, 0, 0);
			ROWAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], L+ss, nu[ss]+nx[ss], 0);
			}
		if(ng[ss]>0)
			{
			GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, AL+0, 0, 0, AL+0, 0, 0);
			ROWIN(ng[ss], 1.0, tmp_nbgM+1, nb[ss], AL+0, nu[ss]+nx[ss], 0);
			SYRK_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], ng[ss], 1.0, AL+0, 0, 0, DCt+ss, 0, 0, 1.0, L+ss, 0, 0, L+ss, 0, 0);
			}
		POTRF_L_MN(nu[ss]+nx[ss]+1, nu[ss], L+ss, 0, 0, L+ss, 0, 0);
		GECP(nx[ss]+1, nu[ss], L+ss, nu[ss], 0, Ls, 0, 0);
		SYRK_LN_MN(nx[ss]+1, nx[ss], nu[ss], -1.0, Ls, 0, 0, Ls, 0, 0, 1.0, L+ss, nu[ss], nu[ss], P+ss, 0, 0);
		TRTR_L(nx[ss], P+ss, 0, 0, P+ss, 0, 0);

		// middle stages
		for(nn=0; nn<N-1; nn++)
			{
			ss = N-nn-1;
			ROWIN(nx[ss+1], 1.0, res_b+ss, 0, BAbt+ss, nu[ss]+nx[ss], 0);
			GEMM_NT(nu[ss]+nx[ss]+1, nx[ss+1], nx[ss+1], 1.0, BAbt+ss, 0, 0, P+ss+1, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm
			ROWEX(nx[ss+1], 1.0, AL, nu[ss]+nx[ss], 0, Pb+ss, 0);
			GEAD(1, nx[ss+1], 1.0, P+ss+1, nx[ss+1], 0, AL, nu[ss]+nx[ss], 0);

#if defined(DOUBLE_PRECISION)
			TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0);
#else
			GECP(nu[ss]+nx[ss], nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0);
#endif
			DIARE(nu[ss]+nx[ss], arg->reg_prim, L+ss, 0, 0);
			ROWIN(nu[ss]+nx[ss], 1.0, res_g+ss, 0, L+ss, nu[ss]+nx[ss], 0);

			if(ns[ss]>0)
				{
				COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
				}
			else if(nb[ss]+ng[ss]>0)
				{
				AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
				AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
				}
			if(nb[ss]>0)
				{
				DIAAD_SP(nb[ss], 1.0, tmp_nbgM+0, 0, idxb[ss], L+ss, 0, 0);
				ROWAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], L+ss, nu[ss]+nx[ss], 0);
				}
			if(ng[ss]>0)
				{
				GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, AL+0, 0, nx[ss+1], AL+0, 0, nx[ss+1]);
				ROWIN(ng[ss], 1.0, tmp_nbgM+1, nb[ss], AL+0, nu[ss]+nx[ss], nx[ss+1]);
				SYRK_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], ng[ss], 1.0, AL+0, 0, nx[ss+1], DCt+ss, 0, 0, 1.0, L+ss, 0, 0, L+ss, 0, 0);
				}
			SYRK_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], nx[ss+1], 1.0, AL, 0, 0, BAbt+ss, 0, 0, 1.0, L+ss, 0, 0, L+ss, 0, 0);
			POTRF_L_MN(nu[ss]+nx[ss]+1, nu[ss], L+ss, 0, 0, L+ss, 0, 0);
			GECP(nx[ss]+1, nu[ss], L+ss, nu[ss], 0, Ls, 0, 0);
			SYRK_LN_MN(nx[ss]+1, nx[ss], nu[ss], -1.0, Ls, 0, 0, Ls, 0, 0, 1.0, L+ss, nu[ss], nu[ss], P+ss, 0, 0);
			TRTR_L(nx[ss], P+ss, 0, 0, P+ss, 0, 0);
			}

		// first stage: factorize P in L too
		if(N>0)
			{
			ss = N-nn-1;
			ROWIN(nx[ss+1], 1.0, res_b+ss, 0, BAbt+ss, nu[ss]+nx[ss], 0);
			GEMM_NT(nu[ss]+nx[ss]+1, nx[ss+1], nx[ss+1], 1.0, BAbt+ss, 0, 0, P+ss+1, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm
			ROWEX(nx[ss+1], 1.0, AL, nu[ss]+nx[ss], 0, Pb+ss, 0);
			GEAD(1, nx[ss+1], 1.0, P+ss+1, nx[ss+1], 0, AL, nu[ss]+nx[ss], 0);

#if defined(DOUBLE_PRECISION)
			TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0);
#else
			GECP(nu[ss]+nx[ss], nu[ss]+nx[ss], RSQrq+ss, 0, 0, L+ss, 0, 0);
#endif
			DIARE(nu[ss]+nx[ss], arg->reg_prim, L+ss, 0, 0);
			ROWIN(nu[ss]+nx[ss], 1.0, res_g+ss, 0, L+ss, nu[ss]+nx[ss], 0);

			if(ns[ss]>0)
				{
				COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
				}
			else if(nb[ss]+ng[ss]>0)
				{
				AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
				AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
				}
			if(nb[ss]>0)
				{
				DIAAD_SP(nb[ss], 1.0, tmp_nbgM+0, 0, idxb[ss], L+ss, 0, 0);
				ROWAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], L+ss, nu[ss]+nx[ss], 0);
				}
			if(ng[ss]>0)
				{
				GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, AL+0, 0, nx[ss+1], AL+0, 0, nx[ss+1]);
				ROWIN(ng[ss], 1.0, tmp_nbgM+1, nb[ss], AL+0, nu[ss]+nx[ss], nx[ss+1]);
				SYRK_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], ng[ss], 1.0, AL+0, 0, nx[ss+1], DCt+ss, 0, 0, 1.0, L+ss, 0, 0, L+ss, 0, 0);
				}
			SYRK_POTRF_LN_MN(nu[ss]+nx[ss]+1, nu[ss]+nx[ss], nx[ss+1], AL, 0, 0, BAbt+ss, 0, 0, L+ss, 0, 0, L+ss, 0, 0);
			}

		// forward substitution

		// first stage
		ss = 0;
		ROWEX(nu[ss]+nx[ss], -1.0, L+ss, nu[ss]+nx[ss], 0, dux+ss, 0);
		// XXX P+0 is empty !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//		VECCPSC(nu[ss], -1.0, dux+ss, 0, l+ss, 0);
//		ROWEX(nx[ss], 1.0, P+ss, nx[ss], 0, l+ss, nu[ss]);
		VECCPSC(nu[ss]+nx[ss], -1.0, dux+ss, 0, l+ss, 0);
		ROWEX(nx[ss+1], 1.0, P+ss+1, nx[ss+1], 0, tmp_nuxM, 0);
		VECCP(nx[ss+1], tmp_nuxM, 0, l+ss+1, nu[ss+1]); // TODO remove tmp_nuxM and use l instead !!!!!
		TRSV_LTN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
		GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
		if(arg->comp_dual_sol_eq)
			{
			GEMV_N(nx[ss+1], nx[ss+1], 1.0, P+ss+1, 0, 0, dux+ss+1, nu[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0);
			}

		// middle stages
		for(ss=1; ss<N; ss++)
			{
			ROWEX(nu[ss], -1.0, L+ss, nu[ss]+nx[ss], 0, dux+ss, 0);
			VECCPSC(nu[ss], -1.0, dux+ss, 0, l+ss, 0);
			ROWEX(nx[ss+1], 1.0, P+ss+1, nx[ss+1], 0, tmp_nuxM, 0);
			VECCP(nx[ss+1], tmp_nuxM, 0, l+ss+1, nu[ss+1]); // TODO remove tmp_nuxM and use l instead !!!!!
			TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
			GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+(ss+1), nu[ss+1]);
			if(arg->comp_dual_sol_eq)
				{
				GEMV_N(nx[ss+1], nx[ss+1], 1.0, P+ss+1, 0, 0, dux+ss+1, nu[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0);
				}
			}

		ss = N;
		ROWEX(nu[ss], -1.0, L+ss, nu[ss]+nx[ss], 0, dux+ss, 0);
		VECCPSC(nu[ss], -1.0, dux+ss, 0, l+ss, 0);
		TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

		}

	for(ss=0; ss<=N; ss++)
		VECEX_SP(nb[ss], 1.0, idxb[ss], dux+ss, 0, dt+ss, 0);
	for(ss=0; ss<=N; ss++)
		GEMV_T(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, dux+ss, 0, 0.0, dt+ss, nb[ss], dt+ss, nb[ss]);
	
	for(ss=0; ss<=N; ss++)
		{
		VECCP(nb[ss]+ng[ss], dt+ss, 0, dt+ss, nb[ss]+ng[ss]);
		VECSC(nb[ss]+ng[ss], -1.0, dt+ss, nb[ss]+ng[ss]);
		}

	for(ss=0; ss<=N; ss++)
		{
		if(ns[ss]>0)
			EXPAND_SLACKS(ss, qp, qp_sol, ws);
		}

	COMPUTE_LAM_T_QP(res_d[0].pa, res_m[0].pa, dlam[0].pa, dt[0].pa, cws);

	return;

	}



void OCP_QP_FACT_LQ_SOLVE_KKT_STEP(struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct OCP_QP_IPM_ARG *arg, struct OCP_QP_IPM_WS *ws)
	{

	// TODO find something better ???
	if(!ws->square_root_alg)
		{
		OCP_QP_FACT_SOLVE_KKT_STEP(qp, qp_sol, arg, ws);
		return;
		}
	
	int N = qp->dim->N;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	struct STRMAT *BAbt = qp->BAbt;
	struct STRMAT *RSQrq = qp->RSQrq;
	struct STRMAT *DCt = qp->DCt;
	struct STRVEC *Z = qp->Z;
	struct STRVEC *res_g = qp->rqz;
	struct STRVEC *res_b = qp->b;
	struct STRVEC *res_d = qp->d;
	struct STRVEC *res_m = qp->m;
	int **idxb = qp->idxb;

	struct STRVEC *dux = qp_sol->ux;
	struct STRVEC *dpi = qp_sol->pi;
	struct STRVEC *dlam = qp_sol->lam;
	struct STRVEC *dt = qp_sol->t;

	struct STRMAT *L = ws->L;
	struct STRVEC *l = ws->l;
	struct STRMAT *Lh = ws->Lh;
	struct STRMAT *AL = ws->AL;
	struct STRVEC *Gamma = ws->Gamma;
	struct STRVEC *gamma = ws->gamma;
	struct STRVEC *Pb = ws->Pb;
	struct STRVEC *Zs_inv = ws->Zs_inv;
	struct STRVEC *tmp_nuxM = ws->tmp_nuxM;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;
	struct STRMAT *lq0 = ws->lq0;
	void *lq_work0 = ws->lq_work0;

	REAL *ptr0, *ptr1, *ptr2, *ptr3;

	REAL tmp;

	//
	int ii, nn, ss, idx;

	struct CORE_QP_IPM_WORKSPACE *cws = ws->core_workspace;

	ws->valid_ric_p = 1;

	COMPUTE_GAMMA_GAMMA_QP(res_d[0].pa, res_m[0].pa, cws);

	// factorization and backward substitution

	// last stage
	ss = N;

	VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
	GESE(nu[ss]+nx[ss], nu[ss]+nx[ss]+ng[ss], 0.0, lq0, 0, nu[ss]+nx[ss]);
#else
	GESE(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+ng[ss], 0.0, lq0, 0, 0);
#endif

	if(ns[ss]>0)
		{
		COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
		}
	else if(nb[ss]+ng[ss]>0)
		{
		AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
		AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
		}
	if(nb[ss]>0)
		{
		for(ii=0; ii<nb[ss]; ii++)
			{
			tmp = BLASFEO_VECEL(tmp_nbgM+0, ii);
			tmp = tmp>=0.0 ? tmp : 0.0;
			tmp = sqrt( tmp );
			BLASFEO_MATEL(lq0, idxb[ss][ii], nu[ss]+nx[ss]+idxb[ss][ii]) = tmp>0.0 ? tmp : 0.0;
			}
		VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
		}
	if(ng[ss]>0)
		{
		for(ii=0; ii<ng[ss]; ii++)
			{
			tmp = BLASFEO_VECEL(tmp_nbgM+0, nb[ss]+ii);
			tmp = tmp>=0.0 ? tmp : 0.0;
			tmp = sqrt( tmp );
			BLASFEO_VECEL(tmp_nbgM+0, nb[ss]+ii) = tmp;
			}
		GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, lq0, 0, 2*nu[ss]+2*nx[ss], lq0, 0, 2*nu[ss]+2*nx[ss]);
		GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
		}

	if(ws->use_hess_fact[ss]==0)
		{
		TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, Lh+ss, 0, 0);
		DIARE(nu[ss]+nx[ss], arg->reg_prim, Lh+ss, 0, 0);
		POTRF_L(nu[ss]+nx[ss], Lh+ss, 0, 0, Lh+ss, 0, 0);
		ws->use_hess_fact[ss]=1;
		}

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
	TRCP_L(nu[ss]+nx[ss], Lh+ss, 0, 0, L+ss, 0, 0);
//	GELQF_PD(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+ng[ss], lq0, 0, 0, lq0, 0, 0, lq_work0);
	GELQF_PD_LLA(nu[ss]+nx[ss], ng[ss], L+ss, 0, 0, lq0, 0, nu[ss]+nx[ss], lq0, 0, 2*nu[ss]+2*nx[ss], lq_work0); // TODO reduce lq1 size !!!
#else
	TRCP_L(nu[ss]+nx[ss], Lh+ss, 0, 0, lq0, 0, 0);
	GELQF(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+ng[ss], lq0, 0, 0, lq0, 0, 0, lq_work0);
	TRCP_L(nu[ss]+nx[ss], lq0, 0, 0, L+ss, 0, 0);
	for(ii=0; ii<nu[ss]+nx[ss]; ii++)
		if(BLASFEO_MATEL(L+ss, ii, ii) < 0)
			COLSC(nu[ss]+nx[ss]-ii, -1.0, L+ss, ii, ii);
#endif

	TRSV_LNN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);


	// middle stages
	for(nn=0; nn<N-1; nn++)
		{
		ss = N-nn-1;

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
		GESE(nu[ss]+nx[ss], nu[ss]+nx[ss]+ng[ss], 0.0, lq0, 0, nu[ss]+nx[ss]);
#else
		GESE(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+nx[ss+1]+ng[ss], 0.0, lq0, 0, 0);
#endif

		TRMM_RLNN(nu[ss]+nx[ss], nx[ss+1], 1.0, L+ss+1, nu[ss+1], nu[ss+1], BAbt+ss, 0, 0, lq0, 0, 2*nu[ss]+2*nx[ss]+ng[ss]);
		TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], res_b+ss, 0, Pb+ss, 0);
		TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], Pb+ss, 0, Pb+ss, 0);

		VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
		AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], Pb+ss, 0, tmp_nuxM, 0);
		GEMV_N(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, tmp_nuxM, 0, 1.0, dux+ss, 0, dux+ss, 0);

		if(ns[ss]>0)
			{
			COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
			}
		else if(nb[ss]+ng[ss]>0)
			{
			AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
			AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
			}
		if(nb[ss]>0)
			{
			for(ii=0; ii<nb[ss]; ii++)
				{
				tmp = BLASFEO_VECEL(tmp_nbgM+0, ii);
				tmp = tmp>=0.0 ? tmp : 0.0;
				tmp = sqrt( tmp );
				BLASFEO_MATEL(lq0, idxb[ss][ii], nu[ss]+nx[ss]+idxb[ss][ii]) = tmp>0.0 ? tmp : 0.0;
				}
			VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
			}
		if(ng[ss]>0)
			{
			for(ii=0; ii<ng[ss]; ii++)
				{
				tmp = BLASFEO_VECEL(tmp_nbgM+0, nb[ss]+ii);
				tmp = tmp>=0.0 ? tmp : 0.0;
				tmp = sqrt( tmp );
				BLASFEO_VECEL(tmp_nbgM+0, nb[ss]+ii) = tmp;
				}
			GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, lq0, 0, 2*nu[ss]+2*nx[ss], lq0, 0, 2*nu[ss]+2*nx[ss]);
			GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
			}

		if(ws->use_hess_fact[ss]==0)
			{
			TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, Lh+ss, 0, 0);
			DIARE(nu[ss]+nx[ss], arg->reg_prim, Lh+ss, 0, 0);
			POTRF_L(nu[ss]+nx[ss], Lh+ss, 0, 0, Lh+ss, 0, 0);
			ws->use_hess_fact[ss]=1;
			}

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
		TRCP_L(nu[ss]+nx[ss], Lh+ss, 0, 0, L+ss, 0, 0);
//		GELQF_PD(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+nx[ss+1]+ng[ss], lq0, 0, 0, lq0, 0, 0, lq_work0);
//		GELQF_PD_LA(nu[ss]+nx[ss], nu[ss]+nx[ss]+nx[ss+1]+ng[ss], L+ss, 0, 0, lq0, 0, nu[ss]+nx[ss], lq_work0); // TODO reduce lq1 size !!!
		GELQF_PD_LLA(nu[ss]+nx[ss], nx[ss+1]+ng[ss], L+ss, 0, 0, lq0, 0, nu[ss]+nx[ss], lq0, 0, 2*nu[ss]+2*nx[ss], lq_work0); // TODO reduce lq1 size !!!
#else
		TRCP_L(nu[ss]+nx[ss], Lh+ss, 0, 0, lq0, 0, 0);
		GELQF(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+nx[ss+1]+ng[ss], lq0, 0, 0, lq0, 0, 0, lq_work0);
		TRCP_L(nu[ss]+nx[ss], lq0, 0, 0, L+ss, 0, 0);
		for(ii=0; ii<nu[ss]+nx[ss]; ii++)
			if(BLASFEO_MATEL(L+ss, ii, ii) < 0)
				COLSC(nu[ss]+nx[ss]-ii, -1.0, L+ss, ii, ii);
#endif

		TRSV_LNN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

		}

	// first stage
	nn = N-1;
	ss = N-nn-1;

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
	GESE(nu[ss]+nx[ss], nu[ss]+nx[ss]+ng[ss], 0.0, lq0, 0, nu[ss]+nx[ss]);
#else
	GESE(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+nx[ss+1]+ng[ss], 0.0, lq0, 0, 0);
#endif

	TRMM_RLNN(nu[ss]+nx[ss], nx[ss+1], 1.0, L+ss+1, nu[ss+1], nu[ss+1], BAbt+ss, 0, 0, lq0, 0, 2*nu[ss]+2*nx[ss]+ng[ss]);
	TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], res_b+ss, 0, Pb+ss, 0);
	TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], Pb+ss, 0, Pb+ss, 0);

	VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
	AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], Pb+ss, 0, tmp_nuxM, 0);
	GEMV_N(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, tmp_nuxM, 0, 1.0, dux+ss, 0, dux+ss, 0);

	if(ns[ss]>0)
		{
		COND_SLACKS_FACT_SOLVE(ss, qp, qp_sol, arg, ws);
		}
	else if(nb[ss]+ng[ss]>0)
		{
		AXPY(nb[ss]+ng[ss],  1.0, Gamma+ss, nb[ss]+ng[ss], Gamma+ss, 0, tmp_nbgM+0, 0);
		AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
		}
	if(nb[ss]>0)
		{
		for(ii=0; ii<nb[ss]; ii++)
			{
			tmp = BLASFEO_VECEL(tmp_nbgM+0, ii);
			tmp = tmp>=0.0 ? tmp : 0.0;
			tmp = sqrt( tmp );
			BLASFEO_MATEL(lq0, idxb[ss][ii], nu[ss]+nx[ss]+idxb[ss][ii]) = tmp>0.0 ? tmp : 0.0;
			}
		VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
		}
	if(ng[ss]>0)
		{
		for(ii=0; ii<ng[ss]; ii++)
			{
			tmp = BLASFEO_VECEL(tmp_nbgM+0, nb[ss]+ii);
			tmp = tmp>=0.0 ? tmp : 0.0;
			tmp = sqrt( tmp );
			BLASFEO_VECEL(tmp_nbgM+0, nb[ss]+ii) = tmp;
			}
		GEMM_R_DIAG(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+0, nb[ss], 0.0, lq0, 0, 2*nu[ss]+2*nx[ss], lq0, 0, 2*nu[ss]+2*nx[ss]);
		GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
		}

	if(ws->use_hess_fact[ss]==0)
		{
		TRCP_L(nu[ss]+nx[ss], RSQrq+ss, 0, 0, Lh+ss, 0, 0);
		DIARE(nu[ss]+nx[ss], arg->reg_prim, Lh+ss, 0, 0);
		POTRF_L(nu[ss]+nx[ss], Lh+ss, 0, 0, Lh+ss, 0, 0);
		ws->use_hess_fact[ss]=1;
		}

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
	TRCP_L(nu[ss]+nx[ss], Lh+ss, 0, 0, L+ss, 0, 0);
//	GELQF_PD(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+nx[ss+1]+ng[ss], lq0, 0, 0, lq0, 0, 0, lq_work0);
	GELQF_PD_LLA(nu[ss]+nx[ss], nx[ss+1]+ng[ss], L+ss, 0, 0, lq0, 0, nu[ss]+nx[ss], lq0, 0, 2*nu[ss]+2*nx[ss], lq_work0); // TODO reduce lq1 size !!!
#else
	TRCP_L(nu[ss]+nx[ss], Lh+ss, 0, 0, lq0, 0, 0);
	GELQF(nu[ss]+nx[ss], 2*nu[ss]+2*nx[ss]+nx[ss+1]+ng[ss], lq0, 0, 0, lq0, 0, 0, lq_work0);
	TRCP_L(nu[ss]+nx[ss], lq0, 0, 0, L+ss, 0, 0);
	for(ii=0; ii<nu[ss]+nx[ss]; ii++)
		if(BLASFEO_MATEL(L+ss, ii, ii) < 0)
			COLSC(nu[ss]+nx[ss]-ii, -1.0, L+ss, ii, ii);
#endif

	TRSV_LNN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

	// forward substitution

	// first stage
	ss = 0;
	VECCP(nu[ss]+nx[ss], dux+ss, 0, l+ss, 0);
	VECCP(nx[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
	VECCP(nx[ss+1], dux+ss+1, nu[ss+1], l+ss+1, nu[ss+1]);
	VECSC(nu[ss]+nx[ss], -1.0, dux+ss, 0);
	TRSV_LTN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
	GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
	VECCP(nx[ss+1], dux+ss+1, nu[ss+1], tmp_nuxM, 0);
	TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
	TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
	AXPY(nx[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0, dpi+ss, 0);

	// middle stages
	for(ss=1; ss<N; ss++)
		{
		VECCP(nu[ss], dux+ss, 0, l+ss, 0);
		VECCP(nx[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
		VECCP(nx[ss+1], dux+ss+1, nu[ss+1], l+ss+1, nu[ss+1]);
		VECSC(nu[ss], -1.0, dux+ss, 0);
		TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
		GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
		VECCP(nx[ss+1], dux+ss+1, nu[ss+1], tmp_nuxM, 0);
		TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
		TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
		AXPY(nx[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0, dpi+ss, 0);
		}

	ss = N;
	VECSC(nu[ss], -1.0, dux+ss, 0);
	VECCP(nu[ss], dux+ss, 0, l+ss, 0);
	TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);


	for(ss=0; ss<=N; ss++)
		VECEX_SP(nb[ss], 1.0, idxb[ss], dux+ss, 0, dt+ss, 0);
	for(ss=0; ss<=N; ss++)
		GEMV_T(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, dux+ss, 0, 0.0, dt+ss, nb[ss], dt+ss, nb[ss]);
	
	for(ss=0; ss<=N; ss++)
		{
		VECCP(nb[ss]+ng[ss], dt+ss, 0, dt+ss, nb[ss]+ng[ss]);
		VECSC(nb[ss]+ng[ss], -1.0, dt+ss, nb[ss]+ng[ss]);
		}

	for(ss=0; ss<=N; ss++)
		{
		if(ns[ss]>0)
			EXPAND_SLACKS(ss, qp, qp_sol, ws);
		}

	COMPUTE_LAM_T_QP(res_d[0].pa, res_m[0].pa, dlam[0].pa, dt[0].pa, cws);

	return;

	}



// backward Riccati recursion
void OCP_QP_SOLVE_KKT_STEP(struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct OCP_QP_IPM_ARG *arg, struct OCP_QP_IPM_WS *ws)
	{

	int N = qp->dim->N;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	struct STRMAT *BAbt = qp->BAbt;
//	struct STRMAT *RSQrq = qp->RSQrq;
	struct STRMAT *DCt = qp->DCt;
	struct STRVEC *res_g = qp->rqz;
	struct STRVEC *res_b = qp->b;
	struct STRVEC *res_d = qp->d;
	struct STRVEC *res_m = qp->m;
	int **idxb = qp->idxb;

	struct STRVEC *dux = qp_sol->ux;
	struct STRVEC *dpi = qp_sol->pi;
	struct STRVEC *dlam = qp_sol->lam;
	struct STRVEC *dt = qp_sol->t;

	struct STRMAT *L = ws->L;
	struct STRVEC *l = ws->l;
	struct STRVEC *gamma = ws->gamma;
	struct STRVEC *Pb = ws->Pb;
	struct STRVEC *tmp_nuxM = ws->tmp_nuxM;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;

	//
	int ss, nn, ii;

	struct CORE_QP_IPM_WORKSPACE *cws = ws->core_workspace;

//printf("\nin solve\n");
	COMPUTE_GAMMA_QP(res_d[0].pa, res_m[0].pa, cws);

	if(ws->square_root_alg)
		{
		ws->valid_ric_p = 1;

		// backward substitution

		// last stage
		ss = N;
//blasfeo_print_exp_tran_dvec(2*nb[ss]+2*ng[ss], gamma+ss, 0);
		VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
		if(ns[ss]>0)
			{
			COND_SLACKS_SOLVE(ss, qp, qp_sol, ws);
			}
		else if(nb[ss]+ng[ss]>0)
			{
			AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
			}
		if(nb[ss]>0)
			{
			VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
			}
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
		if(ng[ss]>0)
			{
			GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
			}
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
		TRSV_LNN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);

		// middle stages
		for(nn=0; nn<N-1; nn++)
			{
			ss = N-nn-1;
			VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
			if(ns[ss]>0)
				{
				COND_SLACKS_SOLVE(ss, qp, qp_sol, ws);
				}
			else if(nb[ss]+ng[ss]>0)
				{
				AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
				}
			if(nb[ss]>0)
				{
				VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
				}
			if(ng[ss]>0)
				{
				GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
				}
			if(ws->use_Pb)
				{
				AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], Pb+ss, 0, tmp_nuxM, 0);
				}
			else
				{
				TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], res_b+ss, 0, tmp_nuxM, 0);
				TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
				AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
				}
			GEMV_N(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, tmp_nuxM, 0, 1.0, dux+ss, 0, dux+ss, 0);
			TRSV_LNN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
			}

		// first stage
		nn = N-1;
		ss = N-nn-1;
		VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
		if(ns[ss]>0)
			{
			COND_SLACKS_SOLVE(ss, qp, qp_sol, ws);
			}
		else if(nb[ss]+ng[ss]>0)
			{
			AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
			}
		if(nb[ss]>0)
			{
			VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
			}
		if(ng[ss]>0)
			{
			GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
			}
		if(ws->use_Pb)
			{
			AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], Pb+ss, 0, tmp_nuxM, 0);
			}
		else
			{
			TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], res_b+ss, 0, tmp_nuxM, 0);
			TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
			AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
			}
		GEMV_N(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, tmp_nuxM, 0, 1.0, dux+ss, 0, dux+ss, 0);
		TRSV_LNN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

		// forward substitution

		// first stage
		ss = 0;
		if(arg->comp_dual_sol_eq)
			{
			VECCP(nx[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
			}
		VECCP(nu[ss]+nx[ss], dux+ss, 0, l+ss, 0);
		VECCP(nx[ss+1], dux+ss+1, nu[ss+1], l+ss+1, nu[ss+1]);
		VECSC(nu[ss]+nx[ss], -1.0, dux+ss, 0);
		TRSV_LTN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
		GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
		if(arg->comp_dual_sol_eq)
			{
			TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], dux+ss+1, nu[ss+1], tmp_nuxM, 0);
			TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
			AXPY(nx[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0, dpi+ss, 0);
			}

		// middle stages
		for(ss=1; ss<N; ss++)
			{
			if(arg->comp_dual_sol_eq)
				{
				VECCP(nx[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
				}
			VECCP(nu[ss], dux+ss, 0, l+ss, 0);
			VECCP(nx[ss+1], dux+ss+1, nu[ss+1], l+ss+1, nu[ss+1]);
			VECSC(nu[ss], -1.0, dux+ss, 0);
			TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
			GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
			if(arg->comp_dual_sol_eq)
				{
				TRMV_LTN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], dux+ss+1, nu[ss+1], tmp_nuxM, 0);
				TRMV_LNN(nx[ss+1], L+ss+1, nu[ss+1], nu[ss+1], tmp_nuxM, 0, tmp_nuxM, 0);
				AXPY(nx[ss+1], 1.0, tmp_nuxM, 0, dpi+ss, 0, dpi+ss, 0);
				}
			}

		ss = N;
		VECCP(nu[ss], dux+ss, 0, l+ss, 0);
		VECSC(nu[ss], -1.0, dux+ss, 0);
		TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

		}
	else // classical algirthm
		{
		ws->valid_ric_p = 1;

		struct STRMAT *P = ws->P;

		// backward substitution

		// last stage
		ss = N;
//blasfeo_print_exp_tran_dvec(2*nb[ss]+2*ng[ss], gamma+ss, 0);
		VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
		if(ns[ss]>0)
			{
			COND_SLACKS_SOLVE(ss, qp, qp_sol, ws);
			}
		else if(nb[ss]+ng[ss]>0)
			{
			AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
			}
		if(nb[ss]>0)
			{
			VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
			}
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
		if(ng[ss]>0)
			{
			GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
			}
//blasfeo_print_dmat(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0);
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
		TRSV_LNN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);

		// middle stages
		for(nn=0; nn<N-1; nn++)
			{
			ss = N-nn-1;
			VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
			if(ns[ss]>0)
				{
				COND_SLACKS_SOLVE(ss, qp, qp_sol, ws);
				}
			else if(nb[ss]+ng[ss]>0)
				{
				AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
				}
			if(nb[ss]>0)
				{
				VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
				}
			if(ng[ss]>0)
				{
				GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
				}
			if(ws->use_Pb)
				{
				AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], Pb+ss, 0, tmp_nuxM, 0);
				}
			else
				{
				GEMV_N(nx[ss+1], nx[ss+1], 1.0, P+ss+1, 0, 0, res_b+ss, 0, 1.0, dux+ss+1, nu[ss+1], tmp_nuxM, 0);
				}
			GEMV_N(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, tmp_nuxM, 0, 1.0, dux+ss, 0, dux+ss, 0);
//blasfeo_print_dmat(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0);
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
			TRSV_LNN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
//blasfeo_print_exp_tran_dvec(nu[ss]+nx[ss], dux+ss, 0);
//exit(1);
			}

		// first stage
		nn = N-1;
		ss = N-nn-1;
		VECCP(nu[ss]+nx[ss], res_g+ss, 0, dux+ss, 0);
		if(ns[ss]>0)
			{
			COND_SLACKS_SOLVE(ss, qp, qp_sol, ws);
			}
		else if(nb[ss]+ng[ss]>0)
			{
			AXPY(nb[ss]+ng[ss], -1.0, gamma+ss, nb[ss]+ng[ss], gamma+ss, 0, tmp_nbgM+1, 0);
			}
		if(nb[ss]>0)
			{
			VECAD_SP(nb[ss], 1.0, tmp_nbgM+1, 0, idxb[ss], dux+ss, 0);
			}
		if(ng[ss]>0)
			{
			GEMV_N(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, tmp_nbgM+1, nb[ss], 1.0, dux+ss, 0, dux+ss, 0);
			}
		if(ws->use_Pb)
			{
			AXPY(nx[ss+1], 1.0, dux+ss+1, nu[ss+1], Pb+ss, 0, tmp_nuxM, 0);
			}
		else
			{
			GEMV_N(nx[ss+1], nx[ss+1], 1.0, P+ss+1, 0, 0, res_b+ss, 0, 1.0, dux+ss+1, nu[ss+1], tmp_nuxM, 0);
			}
		GEMV_N(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, tmp_nuxM, 0, 1.0, dux+ss, 0, dux+ss, 0);
		TRSV_LNN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

		// forward substitution

		// first stage
		ss = 0;
		if(arg->comp_dual_sol_eq)
			{
			VECCP(nx[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
			}
		VECCP(nu[ss]+nx[ss], dux+ss, 0, l+ss, 0);
		VECCP(nx[ss+1], dux+ss+1, nu[ss+1], l+ss+1, nu[ss+1]);
		VECSC(nu[ss]+nx[ss], -1.0, dux+ss, 0);
		TRSV_LTN(nu[ss]+nx[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
		GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
		if(arg->comp_dual_sol_eq)
			{
			GEMV_N(nx[ss+1], nx[ss+1], 1.0, P+ss+1, 0, 0, dux+ss+1, nu[ss+1], 1.0, dpi+ss, 0, dpi+ss, 0);
			}

		// middle stages
		for(ss=1; ss<N; ss++)
			{
			if(arg->comp_dual_sol_eq)
				{
				VECCP(nx[ss+1], dux+ss+1, nu[ss+1], dpi+ss, 0);
				}
			VECCP(nu[ss], dux+ss, 0, l+ss, 0);
			VECCP(nx[ss+1], dux+ss+1, nu[ss+1], l+ss+1, nu[ss+1]);
			VECSC(nu[ss], -1.0, dux+ss, 0);
			TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);
			GEMV_T(nu[ss]+nx[ss], nx[ss+1], 1.0, BAbt+ss, 0, 0, dux+ss, 0, 1.0, res_b+ss, 0, dux+ss+1, nu[ss+1]);
			if(arg->comp_dual_sol_eq)
				{
				GEMV_N(nx[ss+1], nx[ss+1], 1.0, P+ss+1, 0, 0, dux+ss+1, nu[ss+1], 1.0, dpi+ss, 0, dpi+ss, 0);
				}
			}

		ss = N;
		VECCP(nu[ss], dux+ss, 0, l+ss, 0);
		VECSC(nu[ss], -1.0, dux+ss, 0);
		TRSV_LTN_MN(nu[ss]+nx[ss], nu[ss], L+ss, 0, 0, dux+ss, 0, dux+ss, 0);

		}

	for(ss=0; ss<=N; ss++)
		VECEX_SP(nb[ss], 1.0, idxb[ss], dux+ss, 0, dt+ss, 0);
	for(ss=0; ss<=N; ss++)
		GEMV_T(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, dux+ss, 0, 0.0, dt+ss, nb[ss], dt+ss, nb[ss]);

	for(ss=0; ss<=N; ss++)
		{
		VECCP(nb[ss]+ng[ss], dt+ss, 0, dt+ss, nb[ss]+ng[ss]);
		VECSC(nb[ss]+ng[ss], -1.0, dt+ss, nb[ss]+ng[ss]);
		}

	for(ss=0; ss<=N; ss++)
		{
		if(ns[ss]>0)
			EXPAND_SLACKS(ss, qp, qp_sol, ws);
		}

	COMPUTE_LAM_T_QP(res_d[0].pa, res_m[0].pa, dlam[0].pa, dt[0].pa, cws);

	return;

	}


