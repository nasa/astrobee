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



hpipm_size_t TREE_OCP_QCQP_SOL_MEMSIZE(struct TREE_OCP_QCQP_DIM *dim)
	{

	// extract dim
	struct tree *ttree = dim->ttree;
	int Nn = dim->Nn;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;

	int ii, idx;

	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<Nn; ii++)
		{
		nvt += nu[ii]+nx[ii]+2*ns[ii];
		nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];
		}
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		net += nx[idx];
		}

	hpipm_size_t size = 0;

	size += 3*Nn*sizeof(struct STRVEC); // ux lam t
	size += 1*(Nn-1)*sizeof(struct STRVEC); // pi

	size += 1*SIZE_STRVEC(nvt); // ux
	size += 1*SIZE_STRVEC(net); // pi
	size += 2*SIZE_STRVEC(nct); // lam t

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 64; // align to typical cache line size
	
	return size;

	}



void TREE_OCP_QCQP_SOL_CREATE(struct TREE_OCP_QCQP_DIM *dim, struct TREE_OCP_QCQP_SOL *qp_sol, void *mem)
	{

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = TREE_OCP_QCQP_SOL_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// extract dim
	struct tree *ttree = dim->ttree;
	int Nn = dim->Nn;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;

	int ii, idx;


	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<Nn; ii++)
		{
		nvt += nu[ii]+nx[ii]+2*ns[ii];
		nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];
		}
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		net += nx[idx];
		}


	// vector struct stuff
	struct STRVEC *sv_ptr = (struct STRVEC *) mem;

	qp_sol->ux = sv_ptr;
	sv_ptr += Nn;
	qp_sol->pi = sv_ptr;
	sv_ptr += Nn-1;
	qp_sol->lam = sv_ptr;
	sv_ptr += Nn;
	qp_sol->t = sv_ptr;
	sv_ptr += Nn;


	// align to typical cache line size
	hpipm_size_t l_ptr = (hpipm_size_t) sv_ptr;
	l_ptr = (l_ptr+63)/64*64;


	// double stuff
	char *c_ptr;
	c_ptr = (char *) l_ptr;

	char *tmp_ptr;

	// ux
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nvt);
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(nu[ii]+nx[ii]+2*ns[ii], qp_sol->ux+ii, tmp_ptr);
		tmp_ptr += nu[ii]*sizeof(REAL); // u
		tmp_ptr += nx[ii]*sizeof(REAL); // x
		tmp_ptr += ns[ii]*sizeof(REAL); // s_ls
		tmp_ptr += ns[ii]*sizeof(REAL); // s_us
		}
	// pi
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(net);
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		CREATE_STRVEC(nx[idx], qp_sol->pi+ii, tmp_ptr);
		tmp_ptr += (nx[idx])*sizeof(REAL); // pi
		}
	// lam
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol->lam+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL); // lb
		tmp_ptr += ng[ii]*sizeof(REAL); // lg
		tmp_ptr += nq[ii]*sizeof(REAL); // lq
		tmp_ptr += nb[ii]*sizeof(REAL); // ub
		tmp_ptr += ng[ii]*sizeof(REAL); // ug
		tmp_ptr += nq[ii]*sizeof(REAL); // uq
		tmp_ptr += ns[ii]*sizeof(REAL); // ls
		tmp_ptr += ns[ii]*sizeof(REAL); // us
		}
	// t
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol->t+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL); // lb
		tmp_ptr += ng[ii]*sizeof(REAL); // lg
		tmp_ptr += nq[ii]*sizeof(REAL); // lq
		tmp_ptr += nb[ii]*sizeof(REAL); // ub
		tmp_ptr += ng[ii]*sizeof(REAL); // ug
		tmp_ptr += nq[ii]*sizeof(REAL); // uq
		tmp_ptr += ns[ii]*sizeof(REAL); // ls
		tmp_ptr += ns[ii]*sizeof(REAL); // us
		}
	
	qp_sol->dim = dim;

	qp_sol->memsize = memsize; //MEMSIZE_TREE_OCP_QCQP_SOL(dim);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + qp_sol->memsize)
		{
		printf("\nCreate_tree_ocp_qp_sol: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void TREE_OCP_QCQP_SOL_GET(char *field, int node_edge, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	if(hpipm_strcmp(field, "u"))
		{ 
		TREE_OCP_QCQP_SOL_GET_U(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "x"))
		{ 
		TREE_OCP_QCQP_SOL_GET_X(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "sl"))
		{ 
		TREE_OCP_QCQP_SOL_GET_SL(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "su"))
		{ 
		TREE_OCP_QCQP_SOL_GET_SU(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "pi"))
		{ 
		TREE_OCP_QCQP_SOL_GET_PI(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_lb"))
		{ 
		TREE_OCP_QCQP_SOL_GET_LAM_LB(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_ub"))
		{ 
		TREE_OCP_QCQP_SOL_GET_LAM_UB(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_lg"))
		{ 
		TREE_OCP_QCQP_SOL_GET_LAM_LG(node_edge, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_ug"))
		{ 
		TREE_OCP_QCQP_SOL_GET_LAM_UG(node_edge, qp_sol, vec);
		}
	else 
		{
		printf("error [TREE_OCP_QCQP_DIM_GET]: unknown field name '%s'. Exiting.\n", field);
		exit(1);
		}
	return;
	}



void TREE_OCP_QCQP_SOL_GET_U(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nu = qp_sol->dim->nu;
	UNPACK_VEC(nu[node], qp_sol->ux+node, 0, vec, 1);
	}



void TREE_OCP_QCQP_SOL_GET_X(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nx = qp_sol->dim->nx;
	int *nu = qp_sol->dim->nu;
	UNPACK_VEC(nx[node], qp_sol->ux+node, nu[node], vec, 1);
	}



void TREE_OCP_QCQP_SOL_GET_SL(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nu = qp_sol->dim->nu;
	int *nx = qp_sol->dim->nx;
	int *ns = qp_sol->dim->ns;
	UNPACK_VEC(ns[node], qp_sol->ux+node, nu[node]+nx[node], vec, 1);
	return;
	}



void TREE_OCP_QCQP_SOL_GET_SU(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nu = qp_sol->dim->nu;
	int *nx = qp_sol->dim->nx;
	int *ns = qp_sol->dim->ns;
	UNPACK_VEC(ns[node], qp_sol->ux+node, nu[node]+nx[node]+ns[node], vec, 1);
	return;
	}


void TREE_OCP_QCQP_SOL_GET_PI(int edge, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
//	struct *tree *ttree = qp->dim->ttree;
	int *nx = qp_sol->dim->nx;
	int idx = edge+1;
//	int idxdad = (ttree->root+idx)->dad;
	UNPACK_VEC(nx[idx], qp_sol->pi+edge, 0, vec, 1);
	}



void TREE_OCP_QCQP_SOL_GET_LAM_LB(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	UNPACK_VEC(nb[node], qp_sol->lam+node, 0, vec, 1);
	}



void TREE_OCP_QCQP_SOL_GET_LAM_UB(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	int *ng = qp_sol->dim->ng;
	int *nq = qp_sol->dim->nq;
	UNPACK_VEC(nb[node], qp_sol->lam+node, nb[node]+ng[node]+nq[node], vec, 1);
	}



void TREE_OCP_QCQP_SOL_GET_LAM_LG(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	int *ng = qp_sol->dim->ng;
	UNPACK_VEC(ng[node], qp_sol->lam+node, nb[node], vec, 1);
	}



void TREE_OCP_QCQP_SOL_GET_LAM_UG(int node, struct TREE_OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	int *ng = qp_sol->dim->ng;
	int *nq = qp_sol->dim->nq;
	UNPACK_VEC(ng[node], qp_sol->lam+node, 2*nb[node]+ng[node]+nq[node], vec, 1);
	}




