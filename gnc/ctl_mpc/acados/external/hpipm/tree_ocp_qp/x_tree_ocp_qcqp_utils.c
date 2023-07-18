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



void TREE_OCP_QCQP_DIM_PRINT(struct TREE_OCP_QCQP_DIM *qp_dim)
	{
	int ii;

	int Nn  = qp_dim->Nn;
	int *nx = qp_dim->nx;
	int *nu = qp_dim->nu;
	int *nbx = qp_dim->nbx;
	int *nbu = qp_dim->nbu;
	int *ng = qp_dim->ng;
	int *nq = qp_dim->nq;
	int *nsbx = qp_dim->nsbx;
	int *nsbu = qp_dim->nsbu;
	int *nsg = qp_dim->nsg;
	int *nsq = qp_dim->nsq;
//	int *nbxe = qp_dim->nbxe;
//	int *nbue = qp_dim->nbue;
//	int *nge = qp_dim->nge;

	printf("Nn = %d\n\n", Nn);

	printf("nx =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nx[ii]);
	printf("\n\n");

	printf("nu =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nu[ii]);
	printf("\n\n");

	printf("nbx =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nbx[ii]);
	printf("\n\n");

	printf("nbu =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nbu[ii]);
	printf("\n\n");

	printf("ng =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", ng[ii]);
	printf("\n\n");

	printf("nq =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nq[ii]);
	printf("\n\n");

	printf("nsbx =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nsbx[ii]);
	printf("\n\n");

	printf("nsbu =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nsbu[ii]);
	printf("\n\n");

	printf("nsg =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nsg[ii]);
	printf("\n\n");

	printf("nsq =\n");
	for (ii = 0; ii<Nn; ii++)
		printf("\t%d", nsq[ii]);
	printf("\n\n");

//	printf("nbxe =\n");
//	for (ii = 0; ii<Nn; ii++)
//		printf("\t%d", nbxe[ii]);
//	printf("\n\n");

//	printf("nbue =\n");
//	for (ii = 0; ii<Nn; ii++)
//		printf("\t%d", nbue[ii]);
//	printf("\n\n");

//	printf("nge =\n");
//	for (ii = 0; ii<Nn; ii++)
//		printf("\t%d", nge[ii]);
//	printf("\n\n");

	return;
	}



void TREE_OCP_QCQP_PRINT(struct TREE_OCP_QCQP_DIM *dim, struct TREE_OCP_QCQP *qp)
	{
	int ii, jj;

	struct tree *ttree = dim->ttree;
	int Nn  = dim->Nn;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;
//	int *nbxe = dim->nbxe;
//	int *nbue = dim->nbue;
//	int *nge = dim->nge;

	int idx, idxdad;

	printf("BAt =\n");
	for (ii = 0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		idxdad = (ttree->root+idx)->dad;
		BLASFEO_PRINT_MAT(nu[idxdad]+nx[idxdad], nx[idx], qp->BAbt+ii, 0, 0);
		}

	printf("b =\n");
	for (ii = 0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		BLASFEO_PRINT_TRAN_VEC(nx[idx], qp->b+ii, 0);
		}

	printf("RSQ =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_MAT(nu[ii]+nx[ii], nu[ii]+nx[ii], qp->RSQrq+ii, 0, 0);

	printf("Z =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*ns[ii], qp->Z+ii, 0);

	printf("rqz =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(nu[ii]+nx[ii]+2*ns[ii], qp->rqz+ii, 0);

	printf("idxb = \n");
	for (ii = 0; ii<Nn; ii++)
		int_print_mat(1, nb[ii], qp->idxb[ii], 1);

	printf("d =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->d+ii, 0);

	printf("d_mask =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->d_mask+ii, 0);

	printf("DCt =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_MAT(nu[ii]+nx[ii], ng[ii], qp->DCt+ii, 0, 0);

	printf("Hq =\n");
	for (ii = 0; ii<Nn; ii++)
		if(nq[ii]==0)
			printf("\n\n");
		else
			for(jj=0; jj<nq[ii]; jj++)
				BLASFEO_PRINT_MAT(nu[ii]+nx[ii], nu[ii]+nx[ii], &qp->Hq[ii][jj], 0, 0);

	printf("gq =\n");
	for (ii = 0; ii<Nn; ii++)
		if(nq[ii]==0)
			printf("\n\n");
		else
			for(jj=0; jj<nq[ii]; jj++)
				BLASFEO_PRINT_TRAN_MAT(nu[ii]+nx[ii], 1, qp->DCt+ii, 0, ng[ii]+jj);

	printf("idxs_rev = \n");
	for (ii = 0; ii<Nn; ii++)
		int_print_mat(1, nb[ii]+ng[ii], qp->idxs_rev[ii], 1);

	printf("m =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->m+ii, 0);

//	printf("idxe = \n");
//	for (ii = 0; ii<Nn; ii++)
//		int_print_mat(1, nbue[ii]+nbxe[ii]+nge[ii], qp->idxe[ii], 1);

	return;
	}



void TREE_OCP_QCQP_SOL_PRINT(struct TREE_OCP_QCQP_DIM *qp_dim, struct TREE_OCP_QCQP_SOL *qp_sol)
	{
	int ii;

	int Nn  = qp_dim->Nn;
	int *nx = qp_dim->nx;
	int *nu = qp_dim->nu;
	int *nb = qp_dim->nb;
	int *ng = qp_dim->ng;
	int *nq = qp_dim->nq;
	int *ns = qp_dim->ns;

	int idx;

	printf("uxs =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(nu[ii]+nx[ii]+2*ns[ii], &qp_sol->ux[ii], 0);

	printf("pi =\n");
	for (ii = 0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		BLASFEO_PRINT_TRAN_VEC(nx[idx], &qp_sol->pi[ii], 0);
		}

	printf("lam =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_sol->lam[ii], 0);

	printf("t =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_sol->t[ii], 0);

	return;
	}



void TREE_OCP_QCQP_IPM_ARG_PRINT(struct TREE_OCP_QCQP_DIM *qp_dim, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	int ii;

	// iter_max
	printf("/* mode */\n");
	printf("int mode = %d;\n", arg->mode);
	// iter_max
	printf("/* iter_max */\n");
	printf("int iter_max = %d;\n", arg->iter_max);
	// alpha_min
	printf("/* alpha_min */\n");
	printf("double alpha_min = %18.15e;\n", arg->alpha_min);
	// mu0
	printf("/* mu0 */\n");
	printf("double mu0 = %18.15e;\n", arg->mu0);
	// tol_stat
	printf("/* tol_stat */\n");
	printf("double tol_stat = %18.15e;\n", arg->res_g_max);
	// tol_eq
	printf("/* tol_eq */\n");
	printf("double tol_eq = %18.15e;\n", arg->res_b_max);
	// tol_ineq
	printf("/* tol_ineq */\n");
	printf("double tol_ineq = %18.15e;\n", arg->res_d_max);
	// tol_comp
	printf("/* tol_comp */\n");
	printf("double tol_comp = %18.15e;\n", arg->res_m_max);
	// reg_prim
	printf("/* reg_prim */\n");
	printf("double reg_prim = %18.15e;\n", arg->reg_prim);
	// warm_start
	printf("/* warm_start */\n");
	printf("int warm_start = %d;\n", arg->warm_start);
	// pred_corr
	printf("/* pred_corr */\n");
	printf("int pred_corr = %d;\n", arg->pred_corr);
	// ric_alg
//	printf("/* ric_alg */\n");
//	printf("int ric_alg = %d;\n", arg->square_root_alg);
	// split_step
	printf("/* split_step */\n");
	printf("int split_step = %d;\n", arg->split_step);

	return;
	}



void TREE_OCP_QCQP_RES_PRINT(struct TREE_OCP_QCQP_DIM *qp_dim, struct TREE_OCP_QCQP_RES *qp_res)
	{
	int ii;

	int Nn  = qp_dim->Nn;
	int *nx = qp_dim->nx;
	int *nu = qp_dim->nu;
	int *nb = qp_dim->nb;
	int *ng = qp_dim->ng;
	int *nq = qp_dim->nq;
	int *ns = qp_dim->ns;

	int idx;

	printf("res_g =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(nu[ii]+nx[ii]+2*ns[ii], &qp_res->res_g[ii], 0);

	printf("res_b =\n");
	for (ii = 0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		BLASFEO_PRINT_TRAN_VEC(nx[ii+1], &qp_res->res_b[ii], 0);
		}

	printf("res_d =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_res->res_d[ii], 0);

	printf("res_m =\n");
	for (ii = 0; ii<Nn; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_res->res_m[ii], 0);

	return;
	}





