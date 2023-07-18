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



hpipm_size_t OCP_QCQP_SOL_STRSIZE()
	{
	return sizeof(struct OCP_QCQP_SOL);
	}



hpipm_size_t OCP_QCQP_SOL_MEMSIZE(struct OCP_QCQP_DIM *dim)
	{

	// extract dim
	int N = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;

	// loop index
	int ii;

	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nu[ii]+nx[ii]+2*ns[ii];
		net += nx[ii+1];
		nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];
		}
	nvt += nu[ii]+nx[ii]+2*ns[ii];
	nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];

	hpipm_size_t size = 0;

	size += 3*(N+1)*sizeof(struct STRVEC); // ux lam t
	size += 1*N*sizeof(struct STRVEC); // pi

	size += 1*SIZE_STRVEC(nvt); // ux
	size += 1*SIZE_STRVEC(net); // pi
	size += 2*SIZE_STRVEC(nct); // lam t

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 64; // align to typical cache line size

	return size;

	}



void OCP_QCQP_SOL_CREATE(struct OCP_QCQP_DIM *dim, struct OCP_QCQP_SOL *qp_sol, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = OCP_QCQP_SOL_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// extract dim
	int N = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;

	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nu[ii]+nx[ii]+2*ns[ii];
		net += nx[ii+1];
		nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];
		}
	nvt += nu[ii]+nx[ii]+2*ns[ii];
	nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];


	// vector struct stuff
	struct STRVEC *sv_ptr = (struct STRVEC *) mem;

	qp_sol->ux = sv_ptr;
	sv_ptr += N+1;
	qp_sol->pi = sv_ptr;
	sv_ptr += N;
	qp_sol->lam = sv_ptr;
	sv_ptr += N+1;
	qp_sol->t = sv_ptr;
	sv_ptr += N+1;


	// align to typical cache line size
	hpipm_size_t l_ptr = (hpipm_size_t) sv_ptr;
	l_ptr = (l_ptr+63)/64*64;


	// REAL stuff
	char *c_ptr;
	c_ptr = (char *) l_ptr;

	char *tmp_ptr;

	// ux
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nvt);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(nu[ii]+nx[ii]+2*ns[ii], qp_sol->ux+ii, tmp_ptr);
		tmp_ptr += nu[ii]*sizeof(REAL); // u
		tmp_ptr += nx[ii]*sizeof(REAL); // x
		tmp_ptr += ns[ii]*sizeof(REAL); // s_ls
		tmp_ptr += ns[ii]*sizeof(REAL); // s_us
//		VECSE(nu[ii]+nx[ii]+2*ns[ii], 0.0, qp_sol->ux+ii, 0);
		}
	// pi
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(net);
	for(ii=0; ii<N; ii++)
		{
		CREATE_STRVEC(nx[ii+1], qp_sol->pi+ii, tmp_ptr);
		tmp_ptr += nx[ii+1]*sizeof(REAL); // pi
//		VECSE(nx[ii+1], 0.0, qp_sol->pi+ii, 0);
		}
	// lam
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<=N; ii++)
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
//		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp_sol->lam+ii, 0);
		}
	// t
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<=N; ii++)
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
//		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp_sol->t+ii, 0);
		}

	qp_sol->dim = dim;

	qp_sol->memsize = memsize; //OCP_QCQP_SOL_MEMSIZE(dim);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + qp_sol->memsize)
		{
		printf("\nCreate_ocp_qcqp_sol: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void OCP_QCQP_SOL_COPY_ALL(struct OCP_QCQP_SOL *qp_sol_orig, struct OCP_QCQP_SOL *qp_sol_dest)
	{

	int N = qp_sol_orig->dim->N;
	int *nx = qp_sol_orig->dim->nx;
	int *nu = qp_sol_orig->dim->nu;
	int *nb = qp_sol_orig->dim->nb;
	int *ng = qp_sol_orig->dim->ng;
	int *nq = qp_sol_orig->dim->nq;
	int *ns = qp_sol_orig->dim->ns;

	int ii;

	// copy dim pointer
//	qp_sol_dest->dim = qp_sol_orig->dim;

	for(ii=0; ii<N; ii++)
		{
		VECCP(nu[ii]+nx[ii]+2*ns[ii], qp_sol_orig->ux+ii, 0, qp_sol_dest->ux+ii, 0);
		VECCP(nx[ii+1], qp_sol_orig->pi+ii, 0, qp_sol_dest->pi+ii, 0);
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol_orig->lam+ii, 0, qp_sol_dest->lam+ii, 0);
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol_orig->t+ii, 0, qp_sol_dest->t+ii, 0);
		}
	ii = N;
	VECCP(nu[ii]+nx[ii]+2*ns[ii], qp_sol_orig->ux+ii, 0, qp_sol_dest->ux+ii, 0);
	VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol_orig->lam+ii, 0, qp_sol_dest->lam+ii, 0);
	VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol_orig->t+ii, 0, qp_sol_dest->t+ii, 0);

	return;

	}



void OCP_QCQP_SOL_GET(char *field, int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	if(hpipm_strcmp(field, "u"))
		{ 
		OCP_QCQP_SOL_GET_U(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "x"))
		{ 
		OCP_QCQP_SOL_GET_X(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "sl"))
		{ 
		OCP_QCQP_SOL_GET_SL(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "su"))
		{ 
		OCP_QCQP_SOL_GET_SU(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "pi"))
		{ 
		OCP_QCQP_SOL_GET_PI(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_lb"))
		{ 
		OCP_QCQP_SOL_GET_LAM_LB(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_ub"))
		{ 
		OCP_QCQP_SOL_GET_LAM_UB(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_lg"))
		{ 
		OCP_QCQP_SOL_GET_LAM_LG(stage, qp_sol, vec);
		}
	else if(hpipm_strcmp(field, "lam_ug"))
		{ 
		OCP_QCQP_SOL_GET_LAM_UG(stage, qp_sol, vec);
		}
	else 
		{
		printf("error [OCP_QCQP_DIM_GET]: unknown field name '%s'. Exiting.\n", field);
		exit(1);
		}
	return;
	}



void OCP_QCQP_SOL_GET_U(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nu = qp_sol->dim->nu;
	UNPACK_VEC(nu[stage], qp_sol->ux+stage, 0, vec, 1);
	}



void OCP_QCQP_SOL_GET_X(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nx = qp_sol->dim->nx;
	int *nu = qp_sol->dim->nu;
	UNPACK_VEC(nx[stage], qp_sol->ux+stage, nu[stage], vec, 1);
	}



void OCP_QCQP_SOL_GET_SL(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nu = qp_sol->dim->nu;
	int *nx = qp_sol->dim->nx;
	int *ns = qp_sol->dim->ns;
	UNPACK_VEC(ns[stage], qp_sol->ux+stage, nu[stage]+nx[stage], vec, 1);
	return;
	}



void OCP_QCQP_SOL_GET_SU(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nu = qp_sol->dim->nu;
	int *nx = qp_sol->dim->nx;
	int *ns = qp_sol->dim->ns;
	UNPACK_VEC(ns[stage], qp_sol->ux+stage, nu[stage]+nx[stage]+ns[stage], vec, 1);
	return;
	}


void OCP_QCQP_SOL_GET_PI(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nx = qp_sol->dim->nx;
	UNPACK_VEC(nx[stage+1], qp_sol->pi+stage, 0, vec, 1);
	}



void OCP_QCQP_SOL_GET_LAM_LB(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	UNPACK_VEC(nb[stage], qp_sol->lam+stage, 0, vec, 1);
	}



void OCP_QCQP_SOL_GET_LAM_UB(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	int *ng = qp_sol->dim->ng;
	UNPACK_VEC(nb[stage], qp_sol->lam+stage, nb[stage]+ng[stage], vec, 1);
	}



void OCP_QCQP_SOL_GET_LAM_LG(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	int *ng = qp_sol->dim->ng;
	UNPACK_VEC(ng[stage], qp_sol->lam+stage, nb[stage], vec, 1);
	}



void OCP_QCQP_SOL_GET_LAM_UG(int stage, struct OCP_QCQP_SOL *qp_sol, REAL *vec)
	{
	int *nb = qp_sol->dim->nb;
	int *ng = qp_sol->dim->ng;
	UNPACK_VEC(ng[stage], qp_sol->lam+stage, 2*nb[stage]+ng[stage], vec, 1);
	}



void OCP_QCQP_SOL_SET(char *field, int stage, REAL *vec, struct OCP_QCQP_SOL *qp_sol)
	{
	if(hpipm_strcmp(field, "u"))
		{ 
		OCP_QCQP_SOL_SET_U(stage, vec, qp_sol);
		}
	else if(hpipm_strcmp(field, "x"))
		{ 
		OCP_QCQP_SOL_SET_X(stage, vec, qp_sol);
		}
	else if(hpipm_strcmp(field, "sl"))
		{ 
		OCP_QCQP_SOL_SET_SL(stage, vec, qp_sol);
		}
	else if(hpipm_strcmp(field, "su"))
		{ 
		OCP_QCQP_SOL_SET_SU(stage, vec, qp_sol);
		}
	else 
		{
		printf("error [OCP_QCQP_DIM_SET]: unknown field name '%s'. Exiting.\n", field);
		exit(1);
		}
	return;
	}



void OCP_QCQP_SOL_SET_U(int stage, REAL *vec, struct OCP_QCQP_SOL *qp_sol)
	{
	int *nu = qp_sol->dim->nu;
	PACK_VEC(nu[stage], vec, 1, qp_sol->ux+stage, 0);
	return;
	}



void OCP_QCQP_SOL_SET_X(int stage, REAL *vec, struct OCP_QCQP_SOL *qp_sol)
	{
	int *nu = qp_sol->dim->nu;
	int *nx = qp_sol->dim->nx;
	PACK_VEC(nx[stage], vec, 1, qp_sol->ux+stage, nu[stage]);
	return;
	}



void OCP_QCQP_SOL_SET_SL(int stage, REAL *vec, struct OCP_QCQP_SOL *qp_sol)
	{
	int *nu = qp_sol->dim->nu;
	int *nx = qp_sol->dim->nx;
	int *ns = qp_sol->dim->ns;
	PACK_VEC(ns[stage], vec, 1, qp_sol->ux+stage, nu[stage]+nx[stage]);
	return;
	}



void OCP_QCQP_SOL_SET_SU(int stage, REAL *vec, struct OCP_QCQP_SOL *qp_sol)
	{
	int *nu = qp_sol->dim->nu;
	int *nx = qp_sol->dim->nx;
	int *ns = qp_sol->dim->ns;
	PACK_VEC(ns[stage], vec, 1, qp_sol->ux+stage, nu[stage]+nx[stage]+ns[stage]);
	return;
	}

