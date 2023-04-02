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
void TREE_OCP_QP_FACT_SOLVE_KKT_UNCONSTR(struct TREE_OCP_QP *qp, struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QP_IPM_ARG *arg, struct TREE_OCP_QP_IPM_WS *ws)
	{

	int Nn = qp->dim->Nn;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	struct tree *ttree = qp->dim->ttree;
	
	struct STRMAT *BAbt = qp->BAbt;
	struct STRMAT *RSQrq = qp->RSQrq;
	struct STRVEC *b = qp->b;
	struct STRVEC *rqz = qp->rqz;

	struct STRVEC *ux = qp_sol->ux;
	struct STRVEC *pi = qp_sol->pi;

	struct STRMAT *L = ws->L;
	struct STRMAT *AL = ws->AL;
	struct STRVEC *tmp_nxM = ws->tmp_nxM;

	//
	int ii, jj;

	int idx, nkids, idxkid;

	struct CORE_QP_IPM_WORKSPACE *cws = ws->core_workspace;

	// backward factorization and substitution

	// loop over nodes, starting from the end

	for(ii=0; ii<Nn; ii++)
		{

		idx = Nn-ii-1;

		nkids = (ttree->root+idx)->nkids;

#if defined(DOUBLE_PRECISION)
		TRCP_L(nu[idx]+nx[idx], RSQrq+idx, 0, 0, L+idx, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#else
		GECP(nu[idx]+nx[idx], nu[idx]+nx[idx], RSQrq+idx, 0, 0, L+idx, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#endif
		ROWIN(nu[idx]+nx[idx], 1.0, rqz+idx, 0, L+idx, nu[idx]+nx[idx], 0);

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			ROWIN(nx[idxkid], 1.0, b+idxkid-1, 0, BAbt+idxkid-1, nu[idx]+nx[idx], 0);
			TRMM_RLNN(nu[idx]+nx[idx]+1, nx[idxkid], 1.0, L+idxkid, nu[idxkid], nu[idxkid], BAbt+idxkid-1, 0, 0, AL, 0, 0);
			GEAD(1, nx[idxkid], 1.0, L+idxkid, nu[idxkid]+nx[idxkid], nu[idxkid], AL, nu[idx]+nx[idx], 0);

			SYRK_LN_MN(nu[idx]+nx[idx]+1, nu[idx]+nx[idx], nx[idxkid], 1.0, AL, 0, 0, AL, 0, 0, 1.0, L+idx, 0, 0, L+idx, 0, 0);

			}

		POTRF_L_MN(nu[idx]+nx[idx]+1, nu[idx]+nx[idx], L+idx, 0, 0, L+idx, 0, 0);

		}


	// forward substitution

	// loop over nodes, starting from the root

	// root
	ii = 0;

	idx = ii;
	nkids = (ttree->root+idx)->nkids;

	ROWEX(nu[idx]+nx[idx], -1.0, L+idx, nu[idx]+nx[idx], 0, ux+idx, 0);
	TRSV_LTN(nu[idx]+nx[idx], L+idx, 0, 0, ux+idx, 0, ux+idx, 0);

	for(jj=0; jj<nkids; jj++)
		{

		idxkid = (ttree->root+idx)->kids[jj];

		GEMV_T(nu[idx]+nx[idx], nx[idxkid], 1.0, BAbt+idxkid-1, 0, 0, ux+idx, 0, 1.0, b+idxkid-1, 0, ux+idxkid, nu[idxkid]);
		ROWEX(nx[idxkid], 1.0, L+idxkid, nu[idxkid]+nx[idxkid], nu[idxkid], tmp_nxM, 0);
		TRMV_LTN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], ux+idxkid, nu[idxkid], pi+idxkid-1, 0);
		AXPY(nx[idxkid], 1.0, tmp_nxM, 0, pi+idxkid-1, 0, pi+idxkid-1, 0);
		TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], pi+idxkid-1, 0, pi+idxkid-1, 0);

		}

	// other nodes
	for(ii=1; ii<Nn; ii++)
		{

		idx = ii;
		nkids = (ttree->root+idx)->nkids;

		ROWEX(nu[idx], -1.0, L+idx, nu[idx]+nx[idx], 0, ux+idx, 0);
		TRSV_LTN_MN(nu[idx]+nx[idx], nu[idx], L+idx, 0, 0, ux+idx, 0, ux+idx, 0);

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			GEMV_T(nu[idx]+nx[idx], nx[idxkid], 1.0, BAbt+idxkid-1, 0, 0, ux+idx, 0, 1.0, b+idxkid-1, 0, ux+idxkid, nu[idxkid]);
			ROWEX(nx[idxkid], 1.0, L+idxkid, nu[idxkid]+nx[idxkid], nu[idxkid], tmp_nxM, 0);
			TRMV_LTN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], ux+idxkid, nu[idxkid], pi+idxkid-1, 0);
			AXPY(nx[idxkid], 1.0, tmp_nxM, 0, pi+idxkid-1, 0, pi+idxkid-1, 0);
			TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], pi+idxkid-1, 0, pi+idxkid-1, 0);

			}

		}

	return;

	}



static void COND_SLACKS_FACT_SOLVE(int ss, struct TREE_OCP_QP *qp, struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QP_IPM_ARG *arg, struct TREE_OCP_QP_IPM_WS *ws)
	{

	int ii, idx;

	int nx0 = qp->dim->nx[ss];
	int nu0 = qp->dim->nu[ss];
	int nb0 = qp->dim->nb[ss];
	int ng0 = qp->dim->ng[ss];
	int ns0 = qp->dim->ns[ss];

	struct STRVEC *Z = qp->Z+ss;
//	int *idxs0 = qp->idxs[ss];
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

#if 1
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
#else
	for(ii=0; ii<ns0; ii++)
		{
		idx = idxs0[ii];
		ptr_Zs_inv[0+ii]   = ptr_Z[0+ii]   + arg->reg_prim + ptr_Gamma[0+idx]       + ptr_Gamma[2*nb0+2*ng0+ii];
		ptr_Zs_inv[ns0+ii] = ptr_Z[ns0+ii] + arg->reg_prim + ptr_Gamma[nb0+ng0+idx] + ptr_Gamma[2*nb0+2*ng0+ns0+ii];
		ptr_dux[nu0+nx0+ii]      = ptr_res_g[nu0+nx0+ii]     + ptr_gamma[0+idx]   + ptr_gamma[2*nb0+2*ng0+ii];
		ptr_dux[nu0+nx0+ns0+ii]  = ptr_res_g[nu0+nx0+ns0+ii] + ptr_gamma[nb0+ng0+idx] + ptr_gamma[2*nb0+2*ng0+ns0+ii];
		ptr_Zs_inv[0+ii]   = 1.0/ptr_Zs_inv[0+ii];
		ptr_Zs_inv[ns0+ii] = 1.0/ptr_Zs_inv[ns0+ii];
		tmp0 = ptr_dux[nu0+nx0+ii]*ptr_Zs_inv[0+ii];
		tmp1 = ptr_dux[nu0+nx0+ns0+ii]*ptr_Zs_inv[ns0+ii];
		ptr_tmp0[idx] = ptr_tmp0[idx] - ptr_tmp0[idx]*ptr_Zs_inv[0+ii]*ptr_tmp0[idx];
		ptr_tmp1[idx] = ptr_tmp1[idx] - ptr_tmp1[idx]*ptr_Zs_inv[ns0+ii]*ptr_tmp1[idx];
		ptr_tmp2[idx] = ptr_tmp2[idx] - ptr_Gamma[0+idx]*tmp0;
		ptr_tmp3[idx] = ptr_tmp3[idx] - ptr_Gamma[nb0+ng0+idx]*tmp1;
		}
#endif

	AXPY(nb0+ng0,  1.0, tmp_nbgM+1, 0, tmp_nbgM+0, 0, tmp_nbgM+0, 0);
	AXPY(nb0+ng0, -1.0, tmp_nbgM+3, 0, tmp_nbgM+2, 0, tmp_nbgM+1, 0);

	return;

	}



static void COND_SLACKS_SOLVE(int ss, struct TREE_OCP_QP *qp, struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QP_IPM_WS *ws)
	{

	int ii, idx;

	int nx0 = qp->dim->nx[ss];
	int nu0 = qp->dim->nu[ss];
	int nb0 = qp->dim->nb[ss];
	int ng0 = qp->dim->ng[ss];
	int ns0 = qp->dim->ns[ss];

//	int *idxs0 = qp->idxs[ss];
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

#if 1
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
#else
	for(ii=0; ii<ns0; ii++)
		{
		idx = idxs0[ii];
		ptr_dux[nu0+nx0+ii]      = ptr_res_g[nu0+nx0+ii]     + ptr_gamma[0+idx]       + ptr_gamma[2*nb0+2*ng0+ii];
		ptr_dux[nu0+nx0+ns0+ii]  = ptr_res_g[nu0+nx0+ns0+ii] + ptr_gamma[nb0+ng0+idx] + ptr_gamma[2*nb0+2*ng0+ns0+ii];
		tmp0 = ptr_dux[nu0+nx0+ii]*ptr_Zs_inv[0+ii];
		tmp1 = ptr_dux[nu0+nx0+ns0+ii]*ptr_Zs_inv[ns0+ii];
		ptr_tmp2[idx] = ptr_tmp2[idx] - ptr_Gamma[0+idx]*tmp0;
		ptr_tmp3[idx] = ptr_tmp3[idx] - ptr_Gamma[nb0+ng0+idx]*tmp1;
		}
#endif

	AXPY(nb0+ng0, -1.0, tmp_nbgM+3, 0, tmp_nbgM+2, 0, tmp_nbgM+1, 0);

	return;

	}



static void EXPAND_SLACKS(int ss, struct TREE_OCP_QP *qp, struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QP_IPM_WS *ws)
	{

	int ii, idx;

	int nx0 = qp->dim->nx[ss];
	int nu0 = qp->dim->nu[ss];
	int nb0 = qp->dim->nb[ss];
	int ng0 = qp->dim->ng[ss];
	int ns0 = qp->dim->ns[ss];

//	int *idxs0 = qp->idxs[ss];
	int *idxs_rev0 = qp->idxs_rev[ss];

	struct STRVEC *dux = qp_sol->ux+ss;
	struct STRVEC *dt = qp_sol->t+ss;

	struct STRVEC *Gamma = ws->Gamma+ss;
	struct STRVEC *Zs_inv = ws->Zs_inv+ss;

	REAL *ptr_Gamma = Gamma->pa;
	REAL *ptr_dux = dux->pa;
	REAL *ptr_dt = dt->pa;
	REAL *ptr_Zs_inv = Zs_inv->pa;

#if 1
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
#else
	for(ii=0; ii<ns0; ii++)
		{
		idx = idxs0[ii];
		ptr_dux[nu0+nx0+ii]     = - ptr_Zs_inv[0+ii]   * (ptr_dux[nu0+nx0+ii]     + ptr_dt[idx]*ptr_Gamma[idx]);
		ptr_dux[nu0+nx0+ns0+ii] = - ptr_Zs_inv[ns0+ii] * (ptr_dux[nu0+nx0+ns0+ii] + ptr_dt[nb0+ng0+idx]*ptr_Gamma[nb0+ng0+idx]);
		ptr_dt[2*nb0+2*ng0+ii]     = ptr_dux[nu0+nx0+ii];
		ptr_dt[2*nb0+2*ng0+ns0+ii] = ptr_dux[nu0+nx0+ns0+ii];
		ptr_dt[0+idx]       = ptr_dt[0+idx]   + ptr_dux[nu0+nx0+ii];
		ptr_dt[nb0+ng0+idx] = ptr_dt[nb0+ng0+idx] + ptr_dux[nu0+nx0+ns0+ii];
		}
#endif

	return;

	}



// backward Riccati recursion
void TREE_OCP_QP_FACT_SOLVE_KKT_STEP(struct TREE_OCP_QP *qp, struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QP_IPM_ARG *arg, struct TREE_OCP_QP_IPM_WS *ws)
	{

	int Nn = qp->dim->Nn;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	struct tree *ttree = qp->dim->ttree;
	
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
	struct STRMAT *AL = ws->AL;
	struct STRVEC *Gamma = ws->Gamma;
	struct STRVEC *gamma = ws->gamma;
	struct STRVEC *Pb = ws->Pb;
	struct STRVEC *Zs_inv = ws->Zs_inv;
	struct STRVEC *tmp_nxM = ws->tmp_nxM;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;

	REAL *ptr0, *ptr1, *ptr2, *ptr3;

	//
	int ss, jj;

	int idx, nkids, idxkid;

	struct CORE_QP_IPM_WORKSPACE *cws = ws->core_workspace;


	COMPUTE_GAMMA_GAMMA_QP(res_d[0].pa, res_m[0].pa, cws);

	// backward factorization and substitution

	// loop over nodes, starting from the end
	for(ss=0; ss<Nn; ss++)
		{

		idx = Nn-ss-1;

		nkids = (ttree->root+idx)->nkids;

#if defined(DOUBLE_PRECISION)
		TRCP_L(nu[idx]+nx[idx], RSQrq+idx, 0, 0, L+idx, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#else
		GECP(nu[idx]+nx[idx], nu[idx]+nx[idx], RSQrq+idx, 0, 0, L+idx, 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
#endif
		DIARE(nu[idx]+nx[idx], arg->reg_prim, L+idx, 0, 0);
		ROWIN(nu[idx]+nx[idx], 1.0, res_g+idx, 0, L+idx, nu[idx]+nx[idx], 0);

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			GECP(nu[idx]+nx[idx], nx[idxkid], BAbt+idxkid-1, 0, 0, AL, 0, 0);
			ROWIN(nx[idxkid], 1.0, res_b+idxkid-1, 0, AL, nu[idx]+nx[idx], 0);
			TRMM_RLNN(nu[idx]+nx[idx]+1, nx[idxkid], 1.0, L+idxkid, nu[idxkid], nu[idxkid], AL, 0, 0, AL, 0, 0);
			ROWEX(nx[idxkid], 1.0, AL, nu[idx]+nx[idx], 0, tmp_nxM, 0);
			TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], tmp_nxM, 0, Pb+idxkid-1, 0);
			GEAD(1, nx[idxkid], 1.0, L+idxkid, nu[idxkid]+nx[idxkid], nu[idxkid], AL, nu[idx]+nx[idx], 0);

			SYRK_LN_MN(nu[idx]+nx[idx]+1, nu[idx]+nx[idx], nx[idxkid], 1.0, AL, 0, 0, AL, 0, 0, 1.0, L+idx, 0, 0, L+idx, 0, 0);

			}

		if(ns[idx]>0)
			{
			COND_SLACKS_FACT_SOLVE(idx, qp, qp_sol, arg, ws);
			}
		else if(nb[idx]+ng[idx]>0)
			{
			AXPY(nb[idx]+ng[idx],  1.0, Gamma+idx, nb[idx]+ng[idx], Gamma+idx, 0, tmp_nbgM+0, 0);
			AXPY(nb[idx]+ng[idx], -1.0, gamma+idx, nb[idx]+ng[idx], gamma+idx, 0, tmp_nbgM+1, 0);
			}
		if(nb[idx]>0)
			{
			DIAAD_SP(nb[idx], 1.0, tmp_nbgM+0, 0, idxb[idx], L+idx, 0, 0);
			ROWAD_SP(nb[idx], 1.0, tmp_nbgM+1, 0, idxb[idx], L+idx, nu[idx]+nx[idx], 0);
			}
		if(ng[idx]>0)
			{
			GEMM_R_DIAG(nu[idx]+nx[idx], ng[idx], 1.0, DCt+idx, 0, 0, tmp_nbgM+0, nb[idx], 0.0, AL+0, 0, 0, AL+0, 0, 0);
			ROWIN(ng[idx], 1.0, tmp_nbgM+1, nb[idx], AL+0, nu[idx]+nx[idx], 0);
			SYRK_POTRF_LN_MN(nu[idx]+nx[idx]+1, nu[idx]+nx[idx], ng[idx], AL+0, 0, 0, DCt+idx, 0, 0, L+idx, 0, 0, L+idx, 0, 0);
			}
		else
			{
			POTRF_L_MN(nu[idx]+nx[idx]+1, nu[idx]+nx[idx], L+idx, 0, 0, L+idx, 0, 0);
			}

		}

	// forward substitution

	// loop over nodes, starting from the root
	for(ss=0; ss<Nn; ss++)
		{

		idx = ss;
		nkids = (ttree->root+idx)->nkids;

		if(idx>0)
			{
			ROWEX(nu[idx], -1.0, L+idx, nu[idx]+nx[idx], 0, dux+idx, 0);
			TRSV_LTN_MN(nu[idx]+nx[idx], nu[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}
		else
			{
			ROWEX(nu[idx]+nx[idx], -1.0, L+idx, nu[idx]+nx[idx], 0, dux+idx, 0);
			TRSV_LTN(nu[idx]+nx[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			GEMV_T(nu[idx]+nx[idx], nx[idxkid], 1.0, BAbt+idxkid-1, 0, 0, dux+idx, 0, 1.0, res_b+idxkid-1, 0, dux+idxkid, nu[idxkid]);
			if(arg->comp_dual_sol_eq)
				{
				ROWEX(nx[idxkid], 1.0, L+idxkid, nu[idxkid]+nx[idxkid], nu[idxkid], tmp_nxM, 0);
				TRMV_LTN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], dux+idxkid, nu[idxkid], dpi+idxkid-1, 0);
				AXPY(nx[idxkid], 1.0, tmp_nxM, 0, dpi+idxkid-1, 0, dpi+idxkid-1, 0);
				TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], dpi+idxkid-1, 0, dpi+idxkid-1, 0);
				}

			}

		}



	for(ss=0; ss<Nn; ss++)
		VECEX_SP(nb[ss], 1.0, idxb[ss], dux+ss, 0, dt+ss, 0);
	for(ss=0; ss<Nn; ss++)
		GEMV_T(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, dux+ss, 0, 0.0, dt+ss, nb[ss], dt+ss, nb[ss]);

	for(ss=0; ss<Nn; ss++)
		{
		VECCP(nb[ss]+ng[ss], dt+ss, 0, dt+ss, nb[ss]+ng[ss]);
		VECSC(nb[ss]+ng[ss], -1.0, dt+ss, nb[ss]+ng[ss]);
		}

	for(ss=0; ss<Nn; ss++)
		{
		if(ns[ss]>0)
			EXPAND_SLACKS(ss, qp, qp_sol, ws);
		}

	COMPUTE_LAM_T_QP(res_d[0].pa, res_m[0].pa, dlam[0].pa, dt[0].pa, cws);

	return;

	}



// backward Riccati recursion
void TREE_OCP_QP_FACT_LQ_SOLVE_KKT_STEP(struct TREE_OCP_QP *qp, struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QP_IPM_ARG *arg, struct TREE_OCP_QP_IPM_WS *ws)
	{

	int Nn = qp->dim->Nn;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	struct tree *ttree = qp->dim->ttree;
	
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
	struct STRMAT *Lh = ws->Lh;
	struct STRMAT *AL = ws->AL;
	struct STRVEC *Gamma = ws->Gamma;
	struct STRVEC *gamma = ws->gamma;
	struct STRVEC *Pb = ws->Pb;
	struct STRVEC *Zs_inv = ws->Zs_inv;
	struct STRVEC *tmp_nxM = ws->tmp_nxM;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;
	struct STRMAT *lq0 = ws->lq0;
	void *lq_work0 = ws->lq_work0;

	REAL *ptr0, *ptr1, *ptr2, *ptr3;

	REAL tmp;

	//
	int ss, ii, jj;

	int idx, nkids, idxkid;

	struct CORE_QP_IPM_WORKSPACE *cws = ws->core_workspace;


	COMPUTE_GAMMA_GAMMA_QP(res_d[0].pa, res_m[0].pa, cws);

	// backward factorization and substitution

	// loop over nodes, starting from the end
	for(ss=0; ss<Nn; ss++)
		{

		idx = Nn-ss-1;

		nkids = (ttree->root+idx)->nkids;

//		GESE(nu[idx]+nx[idx], 2*nu[idx]+2*nx[idx]+ng[idx], 0.0, lq0, 0, 0);
		GESE(nu[idx]+nx[idx], nu[idx]+nx[idx]+ng[idx], 0.0, lq0, 0, nu[idx]+nx[idx]);
		//
		if(ws->use_hess_fact[idx]==0)
			{
			TRCP_L(nu[idx]+nx[idx], RSQrq+idx, 0, 0, Lh+idx, 0, 0);
			DIARE(nu[idx]+nx[idx], arg->reg_prim, Lh+idx, 0, 0);
			POTRF_L(nu[idx]+nx[idx], Lh+idx, 0, 0, Lh+idx, 0, 0);
			ws->use_hess_fact[idx]=1;
			}
#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
		TRCP_L(nu[idx]+nx[idx], Lh+idx, 0, 0, L+idx, 0, 0);
#else
		GECP(nu[idx]+nx[idx], nu[idx]+nx[idx], Lh+idx, 0, 0, L+idx, 0, 0);
#endif

		VECCP(nu[idx]+nx[idx], res_g+idx, 0, dux+idx, 0);

		if(ns[idx]>0)
			{
			COND_SLACKS_FACT_SOLVE(idx, qp, qp_sol, arg, ws);
			}
		else if(nb[idx]+ng[idx]>0)
			{
			AXPY(nb[idx]+ng[idx],  1.0, Gamma+idx, nb[idx]+ng[idx], Gamma+idx, 0, tmp_nbgM+0, 0);
			AXPY(nb[idx]+ng[idx], -1.0, gamma+idx, nb[idx]+ng[idx], gamma+idx, 0, tmp_nbgM+1, 0);
			}
		if(nb[idx]>0)
			{
			for(ii=0; ii<nb[idx]; ii++)
				{
				tmp = BLASFEO_VECEL(tmp_nbgM+0, ii);
				tmp = tmp>=0.0 ? tmp : 0.0;
				tmp = sqrt( tmp );
				BLASFEO_MATEL(lq0, idxb[idx][ii], nu[idx]+nx[idx]+idxb[idx][ii]) = tmp>0.0 ? tmp : 0.0;
				}
			VECAD_SP(nb[idx], 1.0, tmp_nbgM+1, 0, idxb[idx], dux+idx, 0);
			}
		if(ng[idx]>0)
			{
			for(ii=0; ii<ng[idx]; ii++)
				{
				tmp = BLASFEO_VECEL(tmp_nbgM+0, nb[idx]+ii);
				tmp = tmp>=0.0 ? tmp : 0.0;
				tmp = sqrt( tmp );
				BLASFEO_VECEL(tmp_nbgM+0, nb[idx]+ii) = tmp;
				}
			GEMM_R_DIAG(nu[idx]+nx[idx], ng[idx], 1.0, DCt+idx, 0, 0, tmp_nbgM+0, nb[idx], 0.0, lq0, 0, 2*nu[idx]+2*nx[idx], lq0, 0, 2*nu[idx]+2*nx[idx]);
			GEMV_N(nu[idx]+nx[idx], ng[idx], 1.0, DCt+idx, 0, 0, tmp_nbgM+1, nb[idx], 1.0, dux+idx, 0, dux+idx, 0);
			}

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
		GELQF_PD_LLA(nu[idx]+nx[idx], ng[idx], L+idx, 0, 0, lq0, 0, nu[idx]+nx[idx], lq0, 0, 2*nu[idx]+2*nx[idx], lq_work0); // TODO reduce lq1 size !!!
#else
		TRCP_L(nu[idx]+nx[idx], L+idx, 0, 0, lq0, 0, 0);
		GELQF(nu[idx]+nx[idx], 2*nu[idx]+2*nx[idx]+ng[idx], lq0, 0, 0, lq0, 0, 0, lq_work0);
		TRCP_L(nu[idx]+nx[idx], lq0, 0, 0, L+idx, 0, 0);
		for(ii=0; ii<nu[idx]+nx[idx]; ii++)
			if(BLASFEO_MATEL(L+idx, ii, ii) < 0)
				COLSC(nu[idx]+nx[idx]-ii, -1.0, L+idx, ii, ii);
#endif

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			TRMM_RLNN(nu[idx]+nx[idx], nx[idxkid], 1.0, L+idxkid, nu[idxkid], nu[idxkid], BAbt+idxkid-1, 0, 0, lq0, 0, nu[idx]+nx[idx]);

#if defined(LA_HIGH_PERFORMANCE) | defined(LA_REFERENCE)
			GELQF_PD_LA(nu[idx]+nx[idx], nx[idxkid], L+idx, 0, 0, lq0, 0, nu[idx]+nx[idx], lq_work0);
#else
			TRCP_L(nu[idx]+nx[idx], L+idx, 0, 0, lq0, 0, 0);
			GELQF(nu[idx]+nx[idx], nu[idx]+nx[idx]+nx[idxkid], lq0, 0, 0, lq0, 0, 0, lq_work0);
			TRCP_L(nu[idx]+nx[idx], lq0, 0, 0, L+idx, 0, 0);
			for(ii=0; ii<nu[idx]+nx[idx]; ii++)
				if(BLASFEO_MATEL(L+idx, ii, ii) < 0)
					COLSC(nu[idx]+nx[idx]-ii, -1.0, L+idx, ii, ii);
#endif

			TRMV_LTN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], res_b+idxkid-1, 0, Pb+idxkid-1, 0);
			TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], Pb+idxkid-1, 0, Pb+idxkid-1, 0);
			AXPY(nx[idxkid], 1.0, dux+idxkid, nu[idxkid], Pb+idxkid-1, 0, tmp_nxM, 0);
			GEMV_N(nu[idx]+nx[idx], nx[idxkid], 1.0, BAbt+idxkid-1, 0, 0, tmp_nxM, 0, 1.0, dux+idx, 0, dux+idx, 0);

			}

		if(idx>0)
			{
			TRSV_LNN_MN(nu[idx]+nx[idx], nu[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}
		else // root
			{
			TRSV_LNN(nu[idx]+nx[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}

		}



	// forward substitution

	// loop over nodes, starting from the root
	for(ss=0; ss<Nn; ss++)
		{

		idx = ss;
		nkids = (ttree->root+idx)->nkids;

		if(idx>0)
			{
			VECSC(nu[idx], -1.0, dux+idx, 0);
			TRSV_LTN_MN(nu[idx]+nx[idx], nu[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}
		else // root
			{
			VECSC(nu[idx]+nx[idx], -1.0, dux+idx, 0);
			TRSV_LTN(nu[idx]+nx[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			VECCP(nx[idxkid], dux+idxkid, nu[idxkid], dpi+idxkid-1, 0);
			GEMV_T(nu[idx]+nx[idx], nx[idxkid], 1.0, BAbt+idxkid-1, 0, 0, dux+idx, 0, 1.0, res_b+idxkid-1, 0, dux+idxkid, nu[idxkid]);
			VECCP(nx[idxkid], dux+idxkid, nu[idxkid], tmp_nxM, 0);
			TRMV_LTN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], tmp_nxM, 0, tmp_nxM, 0);
			TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], tmp_nxM, 0, tmp_nxM, 0);
			AXPY(nx[idxkid], 1.0, tmp_nxM, 0, dpi+idxkid-1, 0, dpi+idxkid-1, 0);

			}

		}



	for(ss=0; ss<Nn; ss++)
		VECEX_SP(nb[ss], 1.0, idxb[ss], dux+ss, 0, dt+ss, 0);
	for(ss=0; ss<Nn; ss++)
		GEMV_T(nu[ss]+nx[ss], ng[ss], 1.0, DCt+ss, 0, 0, dux+ss, 0, 0.0, dt+ss, nb[ss], dt+ss, nb[ss]);

	for(ss=0; ss<Nn; ss++)
		{
		VECCP(nb[ss]+ng[ss], dt+ss, 0, dt+ss, nb[ss]+ng[ss]);
		VECSC(nb[ss]+ng[ss], -1.0, dt+ss, nb[ss]+ng[ss]);
		}

	for(ss=0; ss<Nn; ss++)
		{
		if(ns[ss]>0)
			EXPAND_SLACKS(ss, qp, qp_sol, ws);
		}

	COMPUTE_LAM_T_QP(res_d[0].pa, res_m[0].pa, dlam[0].pa, dt[0].pa, cws);

	return;

	}


// backward Riccati recursion
void TREE_OCP_QP_SOLVE_KKT_STEP(struct TREE_OCP_QP *qp, struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QP_IPM_ARG *arg, struct TREE_OCP_QP_IPM_WS *ws)
	{

	int Nn = qp->dim->Nn;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	struct tree *ttree = qp->dim->ttree;
	
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
	struct STRVEC *gamma = ws->gamma;
	struct STRVEC *Pb = ws->Pb;
	struct STRVEC *tmp_nxM = ws->tmp_nxM;
	struct STRVEC *tmp_nbgM = ws->tmp_nbgM;

	//
	int ii, jj;

	int idx, nkids, idxkid;

	struct CORE_QP_IPM_WORKSPACE *cws = ws->core_workspace;

	COMPUTE_GAMMA_QP(res_d[0].pa, res_m[0].pa, cws);


	// backward substitution

	// loop over nodes, starting from the end
	for(ii=0; ii<Nn; ii++)
		{

		idx = Nn-ii-1;

		nkids = (ttree->root+idx)->nkids;

		VECCP(nu[idx]+nx[idx], res_g+idx, 0, dux+idx, 0);

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			if(ws->use_Pb)
				{
				AXPY(nx[idxkid], 1.0, dux+idxkid, nu[idxkid], Pb+idxkid-1, 0, tmp_nxM, 0);
				}
			else
				{
				TRMV_LTN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], res_b+idxkid-1, 0, tmp_nxM, 0);
				TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], tmp_nxM, 0, tmp_nxM, 0);
				AXPY(nx[idxkid], 1.0, dux+idxkid, nu[idxkid], tmp_nxM, 0, tmp_nxM, 0);
				}
			GEMV_N(nu[idx]+nx[idx], nx[idxkid], 1.0, BAbt+idxkid-1, 0, 0, tmp_nxM, 0, 1.0, dux+idx, 0, dux+idx, 0);

			}

		if(ns[idx]>0)
			{
			COND_SLACKS_SOLVE(idx, qp, qp_sol, ws);
			}
		else if(nb[idx]+ng[idx]>0)
			{
			AXPY(nb[idx]+ng[idx], -1.0, gamma+idx, nb[idx]+ng[idx], gamma+idx, 0, tmp_nbgM+1, 0);
			}
		if(nb[idx]>0)
			{
			VECAD_SP(nb[idx], 1.0, tmp_nbgM+1, 0, idxb[idx], dux+idx, 0);
			}
		if(ng[idx]>0)
			{
			GEMV_N(nu[idx]+nx[idx], ng[idx], 1.0, DCt+idx, 0, 0, tmp_nbgM+1, nb[idx], 1.0, dux+idx, 0, dux+idx, 0);
			}

		if(idx>0)
			{
			TRSV_LNN_MN(nu[idx]+nx[idx], nu[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}
		else // root
			{
			TRSV_LNN(nu[idx]+nx[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}

		}


	// forward substitution

	// loop over nodes, starting from the root
	for(ii=0; ii<Nn; ii++)
		{

		idx = ii;
		nkids = (ttree->root+idx)->nkids;

		if(idx>0)
			{
			VECSC(nu[idx], -1.0, dux+idx, 0);
			TRSV_LTN_MN(nu[idx]+nx[idx], nu[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}
		else // root
			{
			VECSC(nu[idx]+nx[idx], -1.0, dux+idx, 0);
			TRSV_LTN(nu[idx]+nx[idx], L+idx, 0, 0, dux+idx, 0, dux+idx, 0);
			}

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = (ttree->root+idx)->kids[jj];

			if(arg->comp_dual_sol_eq)
				{
				VECCP(nx[idxkid], dux+idxkid, nu[idxkid], dpi+idxkid-1, 0);
				}
			GEMV_T(nu[idx]+nx[idx], nx[idxkid], 1.0, BAbt+idxkid-1, 0, 0, dux+idx, 0, 1.0, res_b+idxkid-1, 0, dux+idxkid, nu[idxkid]);
			if(arg->comp_dual_sol_eq)
				{
				VECCP(nx[idxkid], dux+idxkid, nu[idxkid], tmp_nxM, 0);
				TRMV_LTN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], tmp_nxM, 0, tmp_nxM, 0);
				TRMV_LNN(nx[idxkid], L+idxkid, nu[idxkid], nu[idxkid], tmp_nxM, 0, tmp_nxM, 0);
				AXPY(nx[idxkid], 1.0, tmp_nxM, 0, dpi+idxkid-1, 0, dpi+idxkid-1, 0);
				}

			}

		}



	for(ii=0; ii<Nn; ii++)
		VECEX_SP(nb[ii], 1.0, idxb[ii], dux+ii, 0, dt+ii, 0);
	for(ii=0; ii<Nn; ii++)
		GEMV_T(nu[ii]+nx[ii], ng[ii], 1.0, DCt+ii, 0, 0, dux+ii, 0, 0.0, dt+ii, nb[ii], dt+ii, nb[ii]);

	for(ii=0; ii<Nn; ii++)
		{
		VECCP(nb[ii]+ng[ii], dt+ii, 0, dt+ii, nb[ii]+ng[ii]);
		VECSC(nb[ii]+ng[ii], -1.0, dt+ii, nb[ii]+ng[ii]);
		}

	for(ii=0; ii<Nn; ii++)
		{
		if(ns[ii]>0)
			EXPAND_SLACKS(ii, qp, qp_sol, ws);
		}

	COMPUTE_LAM_T_QP(res_d[0].pa, res_m[0].pa, dlam[0].pa, dt[0].pa, cws);

	return;

	}



