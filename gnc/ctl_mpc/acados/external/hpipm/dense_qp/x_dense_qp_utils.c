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



void DENSE_QP_DIM_PRINT(struct DENSE_QP_DIM *qp_dim)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int nsb = qp_dim->nsb;
	int nsg = qp_dim->nsg;
	int ns = qp_dim->ns;

	printf("nv = %d\n\n", nv);
	printf("ne = %d\n\n", ne);
	printf("nb = %d\n\n", nb);
	printf("ng = %d\n\n", ng);
	printf("nsb = %d\n\n", nsb);
	printf("nsg = %d\n\n", nsg);
	printf("ns = %d\n\n", ns);

	return;
	}



void DENSE_QP_PRINT(struct DENSE_QP_DIM *qp_dim, struct DENSE_QP *qp)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int ns = qp_dim->ns;

	printf("H = \n");
	BLASFEO_PRINT_MAT(nv, nv, qp->Hv, 0, 0);

	printf("A = \n");
	BLASFEO_PRINT_MAT(ne, nv, qp->A, 0, 0);

	printf("Ct = \n");
	BLASFEO_PRINT_MAT(nv, ng, qp->Ct, 0, 0);

	printf("idxb = \n");
	int_print_mat(1, nb, qp->idxb, 1);

	printf("gz = \n");
	BLASFEO_PRINT_TRAN_VEC(nv+2*ns, qp->gz, 0);

	printf("b = \n");
	BLASFEO_PRINT_TRAN_VEC(ne, qp->b, 0);

	printf("d = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*ns, qp->d, 0);

	printf("d_mask = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*ns, qp->d_mask, 0);

	printf("m = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*ns, qp->m, 0);

	printf("Z = \n");
	BLASFEO_PRINT_TRAN_VEC(2*ns, qp->Z, 0);

	printf("idxs_rev = \n");
	int_print_mat(1, nb+ng, qp->idxs_rev, 1);

	return;
	}



void DENSE_QP_SOL_PRINT(struct DENSE_QP_DIM *qp_dim, struct DENSE_QP_SOL *qp_sol)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int ns = qp_dim->ns;

	printf("v = \n");
	BLASFEO_PRINT_TRAN_VEC(nv+2*ns, qp_sol->v, 0);

	printf("pi = \n");
	BLASFEO_PRINT_TRAN_VEC(ne, qp_sol->pi, 0);

	printf("lam = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*ns, qp_sol->lam, 0);

	printf("t = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*ns, qp_sol->t, 0);

	return;
	}



void DENSE_QP_RES_PRINT(struct DENSE_QP_DIM *qp_dim, struct DENSE_QP_RES *qp_res)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int ns = qp_dim->ns;

	printf("res_g = \n");
	BLASFEO_PRINT_TRAN_VEC(nv+2*ns, qp_res->res_g, 0);

	printf("res_b = \n");
	BLASFEO_PRINT_TRAN_VEC(ne, qp_res->res_b, 0);

	printf("res_d = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*ns, qp_res->res_d, 0);

	printf("res_m = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*ns, qp_res->res_m, 0);

	return;
	}



void DENSE_QP_IPM_ARG_PRINT(struct DENSE_QP_DIM *qp_dim, struct DENSE_QP_IPM_ARG *arg)
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
	// split_step
	printf("/* split_step */\n");
	printf("int split_step = %d;\n", arg->split_step);

	return;
	}




