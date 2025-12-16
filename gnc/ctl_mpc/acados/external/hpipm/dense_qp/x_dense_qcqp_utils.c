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



void DENSE_QCQP_DIM_PRINT(struct DENSE_QCQP_DIM *qp_dim)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int nq = qp_dim->nq;
	int nsb = qp_dim->nsb;
	int nsg = qp_dim->nsg;
	int nsq = qp_dim->nsq;
	int ns = qp_dim->ns;

	printf("nv = %d\n\n", nv);
	printf("ne = %d\n\n", ne);
	printf("nb = %d\n\n", nb);
	printf("ng = %d\n\n", ng);
	printf("nq = %d\n\n", nq);
	printf("nsb = %d\n\n", nsb);
	printf("nsg = %d\n\n", nsg);
	printf("nsq = %d\n\n", nsq);
	printf("ns = %d\n\n", ns);

	return;
	}



void DENSE_QCQP_PRINT(struct DENSE_QCQP_DIM *qp_dim, struct DENSE_QCQP *qp)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int nq = qp_dim->nq;
	int ns = qp_dim->ns;

	printf("H = \n");
	BLASFEO_PRINT_MAT(nv, nv, qp->Hv, 0, 0);

	printf("A = \n");
	BLASFEO_PRINT_MAT(ne, nv, qp->A, 0, 0);

	printf("Ct = \n");
	BLASFEO_PRINT_MAT(nv, ng, qp->Ct, 0, 0);

	printf("Hq = \n");
	for(ii=0; ii<nq; ii++)
		BLASFEO_PRINT_MAT(nv, nv, qp->Hq+ii, 0, 0);

	printf("Hq_nzero = \n");
	int_print_mat(1, nq, qp->Hq_nzero, 1);

	printf("idxb = \n");
	int_print_mat(1, nb, qp->idxb, 1);

	printf("gz = \n");
	BLASFEO_PRINT_TRAN_VEC(nv+2*ns, qp->gz, 0);

	printf("b = \n");
	BLASFEO_PRINT_TRAN_VEC(ne, qp->b, 0);

	printf("d = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*nq+2*ns, qp->d, 0);

	printf("d_mask = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*nq+2*ns, qp->d_mask, 0);

	printf("m = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*nq+2*ns, qp->m, 0);

	printf("Z = \n");
	BLASFEO_PRINT_TRAN_VEC(2*ns, qp->Z, 0);

	printf("gq = \n");
	for(ii=0; ii<nq; ii++)
		BLASFEO_PRINT_TRAN_MAT(nv, 1, qp->Ct, 0, ng+ii);

	printf("idxs_rev = \n");
	int_print_mat(1, nb+ng+nq, qp->idxs_rev, 1);

	return;
	}



void DENSE_QCQP_SOL_PRINT(struct DENSE_QCQP_DIM *qp_dim, struct DENSE_QCQP_SOL *qp_sol)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int nq = qp_dim->nq;
	int ns = qp_dim->ns;

	printf("v = \n");
	BLASFEO_PRINT_TRAN_VEC(nv+2*ns, qp_sol->v, 0);

	printf("pi = \n");
	BLASFEO_PRINT_TRAN_VEC(ne, qp_sol->pi, 0);

	printf("lam = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*nq+2*ns, qp_sol->lam, 0);

	printf("t = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*nq+2*ns, qp_sol->t, 0);

	return;
	}



void DENSE_QCQP_RES_PRINT(struct DENSE_QCQP_DIM *qp_dim, struct DENSE_QCQP_RES *qp_res)
	{
	int ii;

	int nv = qp_dim->nv;
	int ne = qp_dim->ne;
	int nb = qp_dim->nb;
	int ng = qp_dim->ng;
	int nq = qp_dim->nq;
	int ns = qp_dim->ns;

	printf("res_g = \n");
	BLASFEO_PRINT_TRAN_VEC(nv+2*ns, qp_res->res_g, 0);

	printf("res_b = \n");
	BLASFEO_PRINT_TRAN_VEC(ne, qp_res->res_b, 0);

	printf("res_d = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*nq+2*ns, qp_res->res_d, 0);

	printf("res_m = \n");
	BLASFEO_PRINT_TRAN_VEC(2*nb+2*ng+2*nq+2*ns, qp_res->res_m, 0);

	return;
	}



