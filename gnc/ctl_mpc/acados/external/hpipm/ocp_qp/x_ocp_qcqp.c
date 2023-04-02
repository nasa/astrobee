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



hpipm_size_t OCP_QCQP_STRSIZE()
	{
	return sizeof(struct OCP_QCQP);
	}



hpipm_size_t OCP_QCQP_MEMSIZE(struct OCP_QCQP_DIM *dim)
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

	// compute core qp size
	int nvt = 0;
	int net = 0;
	int nct = 0;
	int nqt = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nx[ii]+nu[ii]+2*ns[ii];
		net += nx[ii+1];
		nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];
		nqt += nq[ii];
		}
	nvt += nx[ii]+nu[ii]+2*ns[ii];
	nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];
	nqt += nq[ii];

	hpipm_size_t size = 0;

	size += (N+1)*sizeof(struct STRMAT *); // Hq

//	size += 5*(N+1)*sizeof(int); // nx nu nb ng ns
	size += 3*(N+1)*sizeof(int *); // idxb idxs_rev Hq_nzero
	size += (2*(N+1)+nqt)*sizeof(struct STRMAT); // RSqrq DCt Hq
	size += 1*N*sizeof(struct STRMAT); // BAbt
	size += 5*(N+1)*sizeof(struct STRVEC); // rqz d m Z d_mask
	size += 1*N*sizeof(struct STRVEC); // b

	for(ii=0; ii<N; ii++)
		{
		size += nb[ii]*sizeof(int); // idxb
		size += (nb[ii]+ng[ii]+nq[ii])*sizeof(int); // idxs_rev
		size += nq[ii]*sizeof(int); // Hq_nzero
		size += SIZE_STRMAT(nu[ii]+nx[ii]+1, nx[ii+1]); // BAbt
		size += SIZE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // RSQrq
		size += SIZE_STRMAT(nu[ii]+nx[ii], ng[ii]+nq[ii]); // DCt
		size += SIZE_STRVEC(2*ns[ii]); // Z
		size += nq[ii]*SIZE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // Hq
		}
	ii = N;
	size += nb[ii]*sizeof(int); // idxb
	size += nq[ii]*sizeof(int); // Hq_nzero
	size += (nb[ii]+ng[ii]+nq[ii])*sizeof(int); // idxs_rev
	size += SIZE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // RSQrq
	size += SIZE_STRMAT(nu[ii]+nx[ii], ng[ii]+nq[ii]); // DCt
	size += SIZE_STRVEC(2*ns[ii]); // Z
	size += nq[ii]*SIZE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // Hq

	size += 1*SIZE_STRVEC(nvt); // rqz
	size += 1*SIZE_STRVEC(net); // b
	size += 3*SIZE_STRVEC(nct); // d m d_mask

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 64; // align to typical cache line size

	return size;

	}



void OCP_QCQP_CREATE(struct OCP_QCQP_DIM *dim, struct OCP_QCQP *qp, void *mem)
	{

	// loop index
	int ii, jj;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = OCP_QCQP_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// extract dim
	int N = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;

	// compute core qp size
	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nx[ii]+nu[ii]+2*ns[ii];
		net += nx[ii+1];
		nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];
		}
	nvt += nx[ii]+nu[ii]+2*ns[ii];
	nct += 2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii];



	// int pointer stuff
	int **ip_ptr;
	ip_ptr = (int **) mem;

	// idxb
	qp->idxb = ip_ptr;
	ip_ptr += N+1;
	// idxs_rev
	qp->idxs_rev = ip_ptr;
	ip_ptr += N+1;
	// Hq_nzero
	qp->Hq_nzero = ip_ptr;
	ip_ptr += N+1;


	// matrix struct pointer stuff
	struct STRMAT **smp_ptr = (struct STRMAT **) ip_ptr;

	// Hq
	qp->Hq = smp_ptr;
	smp_ptr += N+1;


	// matrix struct stuff
	struct STRMAT *sm_ptr = (struct STRMAT *) smp_ptr;

	// BAbt
	qp->BAbt = sm_ptr;
	sm_ptr += N;
	// RSQrq
	qp->RSQrq = sm_ptr;
	sm_ptr += N+1;
	// DCt
	qp->DCt = sm_ptr;
	sm_ptr += N+1;
	// Hq
	for(ii=0; ii<=N; ii++)
		{
		qp->Hq[ii] = sm_ptr;
		sm_ptr += nq[ii];
		}


	// vector struct stuff
	struct STRVEC *sv_ptr = (struct STRVEC *) sm_ptr;

	// b
	qp->b = sv_ptr;
	sv_ptr += N;
	// rqz
	qp->rqz = sv_ptr;
	sv_ptr += N+1;
	// d
	qp->d = sv_ptr;
	sv_ptr += N+1;
	// d_mask
	qp->d_mask = sv_ptr;
	sv_ptr += N+1;
	// m
	qp->m = sv_ptr;
	sv_ptr += N+1;
	// Z
	qp->Z = sv_ptr;
	sv_ptr += N+1;


	// integer stuff
	int *i_ptr;
	i_ptr = (int *) sv_ptr;

	// idxb
	for(ii=0; ii<=N; ii++)
		{
		(qp->idxb)[ii] = i_ptr;
		i_ptr += nb[ii];
		for(jj=0; jj<nb[ii]; jj++)
			qp->idxb[ii][jj] = 0;
		}
	// idxs_rev
	for(ii=0; ii<=N; ii++)
		{
		(qp->idxs_rev)[ii] = i_ptr;
		i_ptr += nb[ii]+ng[ii]+nq[ii];
		for(jj=0; jj<nb[ii]+ng[ii]+nq[ii]; jj++)
			qp->idxs_rev[ii][jj] = -1;
		}
	// Hq_nzero
	for(ii=0; ii<=N; ii++)
		{
		(qp->Hq_nzero)[ii] = i_ptr;
		i_ptr += nq[ii];
		for(jj=0; jj<nq[ii]; jj++)
			qp->Hq_nzero[ii][jj] = 0;
		}


	// align to typical cache line size
	hpipm_size_t l_ptr = (hpipm_size_t) i_ptr;
	l_ptr = (l_ptr+63)/64*64;


	// floating point stuff
	char *c_ptr;
	c_ptr = (char *) l_ptr;

	char *tmp_ptr;

	// BAbt
	for(ii=0; ii<N; ii++)
		{
		CREATE_STRMAT(nu[ii]+nx[ii]+1, nx[ii+1], qp->BAbt+ii, c_ptr);
		c_ptr += (qp->BAbt+ii)->memsize;
//		GESE(nu[ii]+nx[ii]+1, nx[ii+1], 0.0, qp->BAbt+ii, 0, 0);
		}
	// RSQrq
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], qp->RSQrq+ii, c_ptr);
		c_ptr += (qp->RSQrq+ii)->memsize;
//		GESE(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], 0.0, qp->RSQrq+ii, 0, 0);
		}
	// DCt
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRMAT(nu[ii]+nx[ii], ng[ii]+nq[ii], qp->DCt+ii, c_ptr);
		c_ptr += (qp->DCt+ii)->memsize;
//		GESE(nu[ii]+nx[ii], ng[ii], 0.0, qp->DCt+ii, 0, 0);
		}
	// Hq
	for(ii=0; ii<=N; ii++)
		{
		for(jj=0; jj<nq[ii]; jj++)
			{
			CREATE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &qp->Hq[ii][jj], c_ptr);
			c_ptr += qp->Hq[ii][jj].memsize;
//			GESE(nu[ii]+nx[ii], ng[ii], 0.0, qp->DCt+ii, 0, 0);
			}
		}
	// Z
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(2*ns[ii], qp->Z+ii, c_ptr);
		c_ptr += (qp->Z+ii)->memsize;
//		VECSE(2*ns[ii], 0.0, qp->Z+ii, 0);
		}
	// g
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nvt);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(nu[ii]+nx[ii]+2*ns[ii], qp->rqz+ii, tmp_ptr);
		tmp_ptr += nu[ii]*sizeof(REAL);
		tmp_ptr += nx[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
//		VECSE(nu[ii]+nx[ii]+2*ns[ii], 0.0, qp->rqz+ii, 0);
		}
	// b
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(net);
	for(ii=0; ii<N; ii++)
		{
		CREATE_STRVEC(nx[ii+1], qp->b+ii, tmp_ptr);
		tmp_ptr += nx[ii+1]*sizeof(REAL);
//		VECSE(nx[ii+1], 0.0, qp->b+ii, 0);
		}
	// d
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->d+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL); // lb
		tmp_ptr += ng[ii]*sizeof(REAL); // lg
		tmp_ptr += nq[ii]*sizeof(REAL); // lq
		tmp_ptr += nb[ii]*sizeof(REAL); // ub
		tmp_ptr += ng[ii]*sizeof(REAL); // ug
		tmp_ptr += nq[ii]*sizeof(REAL); // uq
		tmp_ptr += ns[ii]*sizeof(REAL); // ls
		tmp_ptr += ns[ii]*sizeof(REAL); // us
//		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp->d+ii, 0);
		}
	// d_mask
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->d_mask+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL); // lb
		tmp_ptr += ng[ii]*sizeof(REAL); // lg
		tmp_ptr += nq[ii]*sizeof(REAL); // lq
		tmp_ptr += nb[ii]*sizeof(REAL); // ub
		tmp_ptr += ng[ii]*sizeof(REAL); // ug
		tmp_ptr += nq[ii]*sizeof(REAL); // uq
		tmp_ptr += ns[ii]*sizeof(REAL); // ls
		tmp_ptr += ns[ii]*sizeof(REAL); // us
		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 1.0, qp->d_mask+ii, 0);
		}
	// m
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->m+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL); // lb
		tmp_ptr += ng[ii]*sizeof(REAL); // lg
		tmp_ptr += nq[ii]*sizeof(REAL); // lq
		tmp_ptr += nb[ii]*sizeof(REAL); // ub
		tmp_ptr += ng[ii]*sizeof(REAL); // ug
		tmp_ptr += nq[ii]*sizeof(REAL); // uq
		tmp_ptr += ns[ii]*sizeof(REAL); // ls
		tmp_ptr += ns[ii]*sizeof(REAL); // us
//		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp->m+ii, 0);
		}

	qp->dim = dim;

	qp->memsize = memsize; //OCP_QCQP_MEMSIZE(dim);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + qp->memsize)
		{
		printf("\nerror: OCP_QCQP_CREATE: outside memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void OCP_QCQP_COPY_ALL(struct OCP_QCQP *qp_orig, struct OCP_QCQP *qp_dest)
	{

	// extract dim
	int N = qp_orig->dim->N;
	int *nx = qp_orig->dim->nx;
	int *nu = qp_orig->dim->nu;
	int *nb = qp_orig->dim->nb;
	int *nbx = qp_orig->dim->nbx;
	int *nbu = qp_orig->dim->nbu;
	int *ng = qp_orig->dim->ng;
	int *nq = qp_orig->dim->nq;
	int *ns = qp_orig->dim->ns;

	int ii, jj;

	// copy dim pointer
//	qp_dest->dim = qp_orig->dim;

	for(ii=0; ii<N; ii++)
		{
		GECP(nu[ii]+nx[ii]+1, nx[ii+1], qp_orig->BAbt+ii, 0, 0, qp_dest->BAbt+ii, 0, 0);
		VECCP(nx[ii+1], qp_orig->b+ii, 0, qp_dest->b+ii, 0);
		}

	for(ii=0; ii<=N; ii++)
		{
		GECP(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], qp_orig->RSQrq+ii, 0, 0, qp_dest->RSQrq+ii, 0, 0);
		VECCP(2*ns[ii], qp_orig->Z+ii, 0, qp_dest->Z+ii, 0);
		VECCP(nu[ii]+nx[ii]+2*ns[ii], qp_orig->rqz+ii, 0, qp_dest->rqz+ii, 0);
		for(jj=0; jj<nb[ii]; jj++)
			qp_dest->idxb[ii][jj] = qp_orig->idxb[ii][jj];
		GECP(nu[ii]+nx[ii], ng[ii]+nq[ii], qp_orig->DCt+ii, 0, 0, qp_dest->DCt+ii, 0, 0);
		for(jj=0; jj<nq[ii]; jj++)
			GECP(nu[ii]+nx[ii], nu[ii]+nx[ii], &qp_orig->Hq[ii][jj], 0, 0, &qp_dest->Hq[ii][jj], 0, 0);
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_orig->d+ii, 0, qp_dest->d+ii, 0);
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_orig->d_mask+ii, 0, qp_dest->d_mask+ii, 0);
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_orig->m+ii, 0, qp_dest->m+ii, 0);
		for(jj=0; jj<nb[ii]+ng[ii]+nq[ii]; jj++)
			qp_dest->idxs_rev[ii][jj] = qp_orig->idxs_rev[ii][jj];
		for(jj=0; jj<nq[ii]; jj++)
			qp_dest->Hq_nzero[ii][jj] = qp_orig->Hq_nzero[ii][jj];
		}

	return;

	}



void OCP_QCQP_SET_ALL_ZERO(struct OCP_QCQP *qp)
	{

	// extract dim
	int N = qp->dim->N;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	int ii, jj;

	for(ii=0; ii<N; ii++)
		{
		GESE(nu[ii]+nx[ii]+1, nx[ii+1], 0.0, qp->BAbt+ii, 0, 0);
		VECSE(nx[ii+1], 0.0, qp->b+ii, 0);
		}

	for(ii=0; ii<=N; ii++)
		{
		GESE(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], 0.0, qp->RSQrq+ii, 0, 0);
		VECSE(2*ns[ii], 0.0, qp->Z+ii, 0);
		VECSE(nu[ii]+nx[ii]+2*ns[ii], 0.0, qp->rqz+ii, 0);
		for(jj=0; jj<nb[ii]; jj++)
			qp->idxb[ii][jj] = 0;
		GESE(nu[ii]+nx[ii], ng[ii]+nq[ii], 0.0, qp->DCt+ii, 0, 0);
		for(jj=0; jj<nq[ii]; jj++)
			GESE(nu[ii]+nx[ii], nu[ii]+nx[ii], 0.0, &qp->Hq[ii][jj], 0, 0);
		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp->d+ii, 0);
		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 1.0, qp->d_mask+ii, 0);
		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp->m+ii, 0);
		for(jj=0; jj<nb[ii]+ng[ii]+nq[ii]; jj++)
			qp->idxs_rev[ii][jj] = -1;
		for(jj=0; jj<nq[ii]; jj++)
			qp->Hq_nzero[ii][jj] = 0;
		}

	return;

	}



void OCP_QCQP_SET_RHS_ZERO(struct OCP_QCQP *qp)
	{

	// extract dim
	int N = qp->dim->N;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	int ii, jj;

	for(ii=0; ii<N; ii++)
		{
		VECSE(nx[ii+1], 0.0, qp->b+ii, 0);
		}

	for(ii=0; ii<=N; ii++)
		{
		VECSE(2*ns[ii], 0.0, qp->Z+ii, 0);
		VECSE(nu[ii]+nx[ii]+2*ns[ii], 0.0, qp->rqz+ii, 0);
		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp->d+ii, 0);
		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 1.0, qp->d_mask+ii, 0);
		VECSE(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], 0.0, qp->m+ii, 0);
		}

	return;

	}



void OCP_QCQP_SET(char *field, int stage, void *value, struct OCP_QCQP *qp)
	{
	REAL *r_ptr;
	int *i_ptr;
    
	if(hpipm_strcmp(field, "A")) 
		{
		OCP_QCQP_SET_A(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "B")) 
		{
		OCP_QCQP_SET_B(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Q")) 
		{
		OCP_QCQP_SET_Q(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "S")) 
		{
		OCP_QCQP_SET_S(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "R")) 
		{
		OCP_QCQP_SET_R(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "C")) 
		{
		OCP_QCQP_SET_C(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "D")) 
		{
		OCP_QCQP_SET_D(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "b"))
		{ 
		OCP_QCQP_SET_BVEC(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "q"))
		{ 
		OCP_QCQP_SET_QVEC(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "r"))
		{ 
		OCP_QCQP_SET_RVEC(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lb"))
		{ 
		OCP_QCQP_SET_LB(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lb_mask"))
		{ 
		OCP_QCQP_SET_LB_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lbu") | hpipm_strcmp(field, "lu"))
		{ 
		OCP_QCQP_SET_LBU(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lbu_mask"))
		{ 
		OCP_QCQP_SET_LBU_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lbx") | hpipm_strcmp(field, "lx"))
		{ 
		OCP_QCQP_SET_LBX(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lbx_mask"))
		{ 
		OCP_QCQP_SET_LBX_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ub"))
		{ 
		OCP_QCQP_SET_UB(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ub_mask"))
		{ 
		OCP_QCQP_SET_UB_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ubu") | hpipm_strcmp(field, "uu"))
		{ 
		OCP_QCQP_SET_UBU(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ubu_mask"))
		{ 
		OCP_QCQP_SET_UBU_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ubx") | hpipm_strcmp(field, "ux"))
		{ 
		OCP_QCQP_SET_UBX(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ubx_mask"))
		{ 
		OCP_QCQP_SET_UBX_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lg"))
		{ 
		OCP_QCQP_SET_LG(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lg_mask"))
		{ 
		OCP_QCQP_SET_LG_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ug"))
		{ 
		OCP_QCQP_SET_UG(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "ug_mask"))
		{ 
		OCP_QCQP_SET_UG_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Qq"))
		{ 
		OCP_QCQP_SET_QQ(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Sq"))
		{ 
		OCP_QCQP_SET_SQ(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Rq"))
		{ 
		OCP_QCQP_SET_RQ(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "qq"))
		{ 
		OCP_QCQP_SET_QQVEC(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "rq"))
		{ 
		OCP_QCQP_SET_RQVEC(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "uq"))
		{ 
		OCP_QCQP_SET_UQ(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "uq_mask"))
		{ 
		OCP_QCQP_SET_UQ_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Zl"))
		{ 
		OCP_QCQP_SET_ZL(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Zu"))
		{ 
		OCP_QCQP_SET_ZU(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "zl"))
		{ 
		OCP_QCQP_SET_ZLVEC(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "zu"))
		{ 
		OCP_QCQP_SET_ZUVEC(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lls"))
		{ 
		OCP_QCQP_SET_LLS(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lls_mask"))
		{ 
		OCP_QCQP_SET_LLS_MASK(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lus"))
		{ 
		OCP_QCQP_SET_LUS(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "lus_mask"))
		{ 
		OCP_QCQP_SET_LUS_MASK(stage, value, qp);
		}
	// int
	else if(hpipm_strcmp(field, "idxb"))
		{
		OCP_QCQP_SET_IDXB(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "idxbx"))
		{
		OCP_QCQP_SET_IDXBX(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Jbx") | hpipm_strcmp(field, "Jx"))
		{
		OCP_QCQP_SET_JBX(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "idxbu"))
		{
		OCP_QCQP_SET_IDXBU(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Jbu") | hpipm_strcmp(field, "Ju"))
		{
		OCP_QCQP_SET_JBU(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "idxs"))
		{
		OCP_QCQP_SET_IDXS(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "idxs_rev"))
		{
		OCP_QCQP_SET_IDXS_REV(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Jsbu") | hpipm_strcmp(field, "Jsu"))
		{
		OCP_QCQP_SET_JSBU(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Jsbx") | hpipm_strcmp(field, "Jsx"))
		{
		OCP_QCQP_SET_JSBX(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Jsg"))
		{
		OCP_QCQP_SET_JSG(stage, value, qp);
		}
	else if(hpipm_strcmp(field, "Jsq"))
		{
		OCP_QCQP_SET_JSQ(stage, value, qp);
		}
	else
		{
		printf("error: OCP_QCQP_SET: wrong field name '%s'. Exiting.\n", field);
		exit(1);	
		}
	return;
	}



void OCP_QCQP_SET_EL(char *field, int stage, int index, void *elem, struct OCP_QCQP *qp)
	{
	REAL *r_ptr;
	int *i_ptr;
    
	// matrices
	if(hpipm_strcmp(field, "lbx") | hpipm_strcmp(field, "lx"))
		{ 
		OCP_QCQP_SET_EL_LBX(stage, index, elem, qp);
		}
	else if(hpipm_strcmp(field, "ubx") | hpipm_strcmp(field, "ux"))
		{ 
		OCP_QCQP_SET_EL_UBX(stage, index, elem, qp);
		}
	else
		{
		printf("error: OCP_QCQP_SET_EL: wrong field%s\n", field);
		exit(1);	
		}
	return;
	}



void OCP_QCQP_SET_A(int stage, REAL *A, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_MAT2STRMAT(nx[stage+1], nx[stage], A, nx[stage+1], qp->BAbt+stage, nu[stage], 0);

	return;
	}



void OCP_QCQP_SET_B(int stage, REAL *B, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_MAT2STRMAT(nx[stage+1], nu[stage], B, nx[stage+1], qp->BAbt+stage, 0, 0);

	return;
	}



void OCP_QCQP_SET_BVEC(int stage, REAL *b, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_MAT2STRMAT(nx[stage+1], 1, b, nx[stage+1], &(qp->BAbt[stage]), nu[stage]+nx[stage], 0); // TODO remove ???
	PACK_VEC(nx[stage+1], b, 1, qp->b+stage, 0);

	return;
	}



void OCP_QCQP_SET_Q(int stage, REAL *Q, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_MAT2STRMAT(nx[stage], nx[stage], Q, nx[stage], qp->RSQrq+stage, nu[stage], nu[stage]);

	return;
	}



void OCP_QCQP_SET_S(int stage, REAL *S, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_MAT2STRMAT(nu[stage], nx[stage], S, nu[stage], qp->RSQrq+stage, nu[stage], 0);

	return;
	}



void OCP_QCQP_SET_R(int stage, REAL *R, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_MAT2STRMAT(nu[stage], nu[stage], R, nu[stage], qp->RSQrq+stage, 0, 0);

	return;
	}



void OCP_QCQP_SET_QVEC(int stage, REAL *q, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

 	CVT_TRAN_MAT2STRMAT(nx[stage], 1, q, nx[stage], &(qp->RSQrq[stage]), nu[stage]+nx[stage], nu[stage]); // TODO remove ???
	PACK_VEC(nx[stage], q, 1, qp->rqz+stage, nu[stage]);

	return;
	}



void OCP_QCQP_SET_RVEC(int stage, REAL *r, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_MAT2STRMAT(nu[stage], 1, r, nu[stage], &(qp->RSQrq[stage]), nu[stage]+nx[stage], 0); // TODO remove ???
	PACK_VEC(nu[stage], r, 1, qp->rqz+stage, 0);

	return;
	}



void OCP_QCQP_SET_LB(int stage, REAL *lb, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;

	PACK_VEC(nb[stage], lb, 1, qp->d+stage, 0);

	return;
	}



void OCP_QCQP_SET_LB_MASK(int stage, REAL *lb_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;

	PACK_VEC(nb[stage], lb_mask, 1, qp->d_mask+stage, 0);

	return;
	}



void OCP_QCQP_SET_LBX(int stage, REAL *lbx, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;

	PACK_VEC(nbx[stage], lbx, 1, qp->d+stage, nbu[stage]);

	return;
	}



void OCP_QCQP_SET_LBX_MASK(int stage, REAL *lbx_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;

	PACK_VEC(nbx[stage], lbx_mask, 1, qp->d_mask+stage, nbu[stage]);

	return;
	}



void OCP_QCQP_SET_EL_LBX(int stage, int index, REAL *elem, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	BLASFEO_VECEL(qp->d+stage, nbu[stage]+index) = *elem;

	return;
	}



void OCP_QCQP_SET_LBU(int stage, REAL *lbu, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	PACK_VEC(nbu[stage], lbu, 1, qp->d+stage, 0);

	return;
	}



void OCP_QCQP_SET_LBU_MASK(int stage, REAL *lbu_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	PACK_VEC(nbu[stage], lbu_mask, 1, qp->d_mask+stage, 0);

	return;
	}



void OCP_QCQP_SET_UB(int stage, REAL *ub, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nb[stage], ub, 1, qp->d+stage, nb[stage]+ng[stage]+nq[stage]);
	VECSC(nb[stage], -1.0, qp->d+stage, nb[stage]+ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_UB_MASK(int stage, REAL *ub_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nb[stage], ub_mask, 1, qp->d_mask+stage, nb[stage]+ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_UBX(int stage, REAL *ubx, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nbx[stage], ubx, 1, qp->d+stage, nb[stage]+ng[stage]+nq[stage]+nbu[stage]);
	VECSC(nbx[stage], -1.0, qp->d+stage, nb[stage]+ng[stage]+nq[stage]+nbu[stage]);

	return;
	}



void OCP_QCQP_SET_UBX_MASK(int stage, REAL *ubx_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nbx[stage], ubx_mask, 1, qp->d_mask+stage, nb[stage]+ng[stage]+nq[stage]+nbu[stage]);

	return;
	}



void OCP_QCQP_SET_EL_UBX(int stage, int index, REAL *elem, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	BLASFEO_VECEL(qp->d+stage, nb[stage]+ng[stage]+nq[stage]+nbu[stage]+index) = - *elem;

	return;
	}



void OCP_QCQP_SET_UBU(int stage, REAL *ubu, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nbu[stage], ubu, 1, qp->d+stage, nb[stage]+ng[stage]+nq[stage]);
	VECSC(nbu[stage], -1.0, qp->d+stage, nb[stage]+ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_UBU_MASK(int stage, REAL *ubu_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nbu[stage], ubu_mask, 1, qp->d_mask+stage, nb[stage]+ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_IDXB(int stage, int *idxb, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;

	int ii;
	for(ii=0; ii<nb[stage]; ii++)
		qp->idxb[stage][ii] = idxb[ii];

	return;
	}



void OCP_QCQP_SET_IDXBX(int stage, int *idxbx, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;

	int ii;
	for(ii=0; ii<nbx[stage]; ii++)
		{
		qp->idxb[stage][nbu[stage]+ii] = nu[stage] + idxbx[ii];
		}

	return;
	}



void OCP_QCQP_SET_JBX(int stage, REAL *Jbx, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;

	int ii, jj, jj0;
	for(ii=0; ii<nbx[stage]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<nx[stage]; jj++)
			{
			if(jj0==-1 & Jbx[ii+jj*nbx[stage]]!=0.0)
				{
				jj0 = jj;
				qp->idxb[stage][nbu[stage]+ii] = nu[stage]+jj;
				}
			}
		}
	return;
	}



void OCP_QCQP_SET_IDXBU(int stage, int *idxbx, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	int ii;
	for(ii=0; ii<nbu[stage]; ii++)
		{
		qp->idxb[stage][ii] = idxbx[ii];
		}

	return;
	}



void OCP_QCQP_SET_JBU(int stage, REAL *Jbu, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nbu = qp->dim->nbu;

	int ii, jj, jj0;
	for(ii=0; ii<nbu[stage]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<nu[stage]; jj++)
			{
			if(jj0==-1 & Jbu[ii+jj*nbu[stage]]!=0.0)
				{
				jj0 = jj;
				qp->idxb[stage][ii] = jj;
				}
			}
		}
	return;
	}



void OCP_QCQP_SET_C(int stage, REAL *C, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;

	CVT_TRAN_MAT2STRMAT(ng[stage], nx[stage], C, ng[stage], qp->DCt+stage, nu[stage], 0);

	return;
	}



void OCP_QCQP_SET_D(int stage, REAL *D, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;

	CVT_TRAN_MAT2STRMAT(ng[stage], nu[stage], D, ng[stage], qp->DCt+stage, 0, 0);

	return;
	}



void OCP_QCQP_SET_LG(int stage, REAL *lg, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(ng[stage], lg, 1, qp->d+stage, nb[stage]);

	return;
	}



void OCP_QCQP_SET_LG_MASK(int stage, REAL *lg_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(ng[stage], lg_mask, 1, qp->d_mask+stage, nb[stage]);

	return;
	}



void OCP_QCQP_SET_UG(int stage, REAL *ug, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(ng[stage], ug, 1, qp->d+stage, 2*nb[stage]+ng[stage]+nq[stage]);
	VECSC(ng[stage], -1.0, qp->d+stage, 2*nb[stage]+ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_UG_MASK(int stage, REAL *ug_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(ng[stage], ug_mask, 1, qp->d_mask+stage, 2*nb[stage]+ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_QQ(int stage, REAL *Qq, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nq = qp->dim->nq;

	int ii, jj;

	int nzero = 0;

	for(ii=0; ii<nq[stage]; ii++)
		{
		for(jj=0; jj<nx[stage]*nx[stage]; jj++)
			{
			if(Qq[ii*nx[stage]*nx[stage]+jj]!=0.0)
				{
				nzero = 1;
				break;
				}
			}
		CVT_MAT2STRMAT(nx[stage], nx[stage], Qq+ii*nx[stage]*nx[stage], nx[stage], &qp->Hq[stage][ii], nu[stage], nu[stage]);
		if(nzero)
			{
			qp->Hq_nzero[stage][ii] |= 1;
			}
		}

	return;
	}



void OCP_QCQP_SET_SQ(int stage, REAL *Sq, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nq = qp->dim->nq;

	int ii, jj;

	int nzero = 0;

	for(ii=0; ii<nq[stage]; ii++)
		{
		for(jj=0; jj<nu[stage]*nx[stage]; jj++)
			{
			if(Sq[ii*nu[stage]*nx[stage]+jj]!=0.0)
				{
				nzero = 1;
				break;
				}
			}
		CVT_TRAN_MAT2STRMAT(nu[stage], nx[stage], Sq+ii*nu[stage]*nx[stage], nu[stage], &qp->Hq[stage][ii], nu[stage], 0);
		if(nzero)
			{
			qp->Hq_nzero[stage][ii] |= 2;
			}
		}

	return;
	}



void OCP_QCQP_SET_RQ(int stage, REAL *Rq, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nq = qp->dim->nq;

	int ii, jj;

	int nzero = 0;

	for(ii=0; ii<nq[stage]; ii++)
		{
		for(jj=0; jj<nu[stage]*nu[stage]; jj++)
			{
			if(Rq[ii*nu[stage]*nu[stage]+jj]!=0.0)
				{
				nzero = 1;
				break;
				}
			}
		CVT_MAT2STRMAT(nu[stage], nu[stage], Rq+ii*nu[stage]*nu[stage], nu[stage], &qp->Hq[stage][ii], 0, 0);
		if(nzero)
			{
			qp->Hq_nzero[stage][ii] |= 4;
			}
		}

	return;
	}



void OCP_QCQP_SET_QQVEC(int stage, REAL *qq, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int ii;

	CVT_MAT2STRMAT(nx[stage], nq[stage], qq, nx[stage], qp->DCt+stage, nu[stage], ng[stage]);

	return;
	}



void OCP_QCQP_SET_RQVEC(int stage, REAL *rq, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int ii;

	CVT_MAT2STRMAT(nu[stage], nq[stage], rq, nu[stage], qp->DCt+stage, 0, ng[stage]);

	return;
	}



void OCP_QCQP_SET_UQ(int stage, REAL *value, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nq[stage], value, 1, qp->d+stage, 2*nb[stage]+2*ng[stage]+nq[stage]);
	VECSC(nq[stage], -1.0, qp->d+stage, 2*nb[stage]+2*ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_UQ_MASK(int stage, REAL *value, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	PACK_VEC(nq[stage], value, 1, qp->d_mask+stage, 2*nb[stage]+2*ng[stage]+nq[stage]);

	return;
	}



void OCP_QCQP_SET_ZL(int stage, REAL *Zl, struct OCP_QCQP *qp)
	{
	// extract dim
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], Zl, 1, qp->Z+stage, 0);

	return;
	}



void OCP_QCQP_SET_ZU(int stage, REAL *Zu, struct OCP_QCQP *qp)
	{
	// extract dim
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], Zu, 1, qp->Z+stage, ns[stage]);

	return;
	}



void OCP_QCQP_SET_ZLVEC(int stage, REAL *zl, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nx = qp->dim->nx;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], zl, 1, qp->rqz+stage, nu[stage]+nx[stage]);

	return;
	}



void OCP_QCQP_SET_ZUVEC(int stage, REAL *zu, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nx = qp->dim->nx;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], zu, 1, qp->rqz+stage, nu[stage]+nx[stage]+ns[stage]);

	return;
	}



void OCP_QCQP_SET_IDXS(int stage, int *idxs, struct OCP_QCQP *qp)
	{
	// extract dim
	int *ns = qp->dim->ns;

	int ii;
	for(ii=0; ii<ns[stage]; ii++)
		{
		qp->idxs_rev[stage][idxs[ii]] = ii;
		}

	return;
	}



void OCP_QCQP_SET_IDXS_REV(int stage, int *idxs_rev, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int ii;
	for(ii=0; ii<nb[stage]+ng[stage]+nq[stage]; ii++)
		{
		qp->idxs_rev[stage][ii] = idxs_rev[ii];
		}

	return;
	}



void OCP_QCQP_SET_JSBU(int stage, REAL *Jsbu, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	int ii, jj, jj0, idx_tmp;
	// compute nbu part of idxs_rev
	for(ii=0; ii<nbu[stage]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<ns[stage]; jj++)
			{
			if(jj0==-1 & Jsbu[ii+jj*nbu[stage]]!=0.0)
				{
				jj0 = jj;
				qp->idxs_rev[stage][0+ii] = jj;
				}
			}
		}
	return;
	}



void OCP_QCQP_SET_JSBX(int stage, REAL *Jsbx, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	int ii, jj, jj0, idx_tmp;
	// compute nbx part of idxs_rev
	for(ii=0; ii<nbx[stage]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<ns[stage]; jj++)
			{
			if(jj0==-1 & Jsbx[ii+jj*nbx[stage]]!=0.0)
				{
				jj0 = jj;
				qp->idxs_rev[stage][nbu[stage]+ii] = jj;
				}
			}
		}
	return;
	}



void OCP_QCQP_SET_JSG(int stage, REAL *Jsg, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	int ii, jj, jj0, idx_tmp;
	// compute nbx part of idxs_rev
	for(ii=0; ii<ng[stage]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<ns[stage]; jj++)
			{
			if(jj0==-1 & Jsg[ii+jj*ng[stage]]!=0.0)
				{
				jj0 = jj;
				qp->idxs_rev[stage][nb[stage]+ii] = jj;
				}
			}
		}
	return;
	}



void OCP_QCQP_SET_JSQ(int stage, REAL *Jsq, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	int ii, jj, jj0, idx_tmp;
	// compute nbx part of idxs_rev
	for(ii=0; ii<nq[stage]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<ns[stage]; jj++)
			{
			if(jj0==-1 & Jsq[ii+jj*nq[stage]]!=0.0)
				{
				jj0 = jj;
				qp->idxs_rev[stage][nb[stage]+ng[stage]+ii] = jj;
				}
			}
		}
	return;
	}



void OCP_QCQP_SET_LLS(int stage, REAL *ls, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], ls, 1, qp->d+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage]);

	return;
	}



void OCP_QCQP_SET_LLS_MASK(int stage, REAL *ls_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], ls_mask, 1, qp->d_mask+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage]);

	return;
	}



void OCP_QCQP_SET_LUS(int stage, REAL *us, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], us, 1, qp->d+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage]+ns[stage]);

	return;
	}



void OCP_QCQP_SET_LUS_MASK(int stage, REAL *lus_mask, struct OCP_QCQP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[stage], lus_mask, 1, qp->d_mask+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage]+ns[stage]);

	return;
	}



void OCP_QCQP_GET(char *field, int stage, struct OCP_QCQP *qp, void *value)
	{
	// matrices
	if(hpipm_strcmp(field, "A")) 
		{
		OCP_QCQP_GET_A(stage, qp, value);
		}
	// vectors
	else if(hpipm_strcmp(field, "lbx") | hpipm_strcmp(field, "lx"))
		{ 
		OCP_QCQP_GET_LBX(stage, qp, value);
		}
	else if(hpipm_strcmp(field, "ubx") | hpipm_strcmp(field, "ux"))
		{ 
		OCP_QCQP_GET_UBX(stage, qp, value);
		}
	// int
	else
		{
		printf("error: OCP_QCQP_GET: wrong field %s\n", field);
		exit(1);	
		}
	return;
	}



void OCP_QCQP_GET_A(int stage, struct OCP_QCQP *qp, REAL *A)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_STRMAT2MAT(nx[stage], nx[stage+1], qp->BAbt+stage, nu[stage], 0, A, nx[stage+1]);

	return;
	}



void OCP_QCQP_GET_B(int stage, struct OCP_QCQP *qp, REAL *B)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_STRMAT2MAT(nu[stage], nx[stage+1], qp->BAbt+stage, 0, 0, B, nx[stage+1]);

	return;
	}



void OCP_QCQP_GET_BVEC(int stage, struct OCP_QCQP *qp, REAL *b)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	UNPACK_VEC(nx[stage+1], qp->b+stage, 0, b, 1);

	return;
	}



void OCP_QCQP_GET_Q(int stage, struct OCP_QCQP *qp, REAL *Q)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_STRMAT2MAT(nx[stage], nx[stage], qp->RSQrq+stage, nu[stage], nu[stage], Q, nx[stage]);

	return;
	}



void OCP_QCQP_GET_S(int stage, struct OCP_QCQP *qp, REAL *S)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	CVT_TRAN_STRMAT2MAT(nx[stage], nu[stage], qp->RSQrq+stage, nu[stage], 0, S, nu[stage]);

	return;
	}



void OCP_QCQP_GET_R(int stage, struct OCP_QCQP *qp, REAL *R)
	{
	// extract dim
	int *nu = qp->dim->nu;

	CVT_STRMAT2MAT(nu[stage], nu[stage], qp->RSQrq+stage, 0, 0, R, nu[stage]);

	return;
	}



void OCP_QCQP_GET_QVEC(int stage, struct OCP_QCQP *qp, REAL *q)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	UNPACK_VEC(nx[stage], qp->rqz+stage, nu[stage], q, 1);

	return;
	}



void OCP_QCQP_GET_RVEC(int stage, struct OCP_QCQP *qp, REAL *r)
	{
	// extract dim
	int *nu = qp->dim->nu;

	UNPACK_VEC(nu[stage], qp->rqz+stage, 0, r, 1);

	return;
	}



void OCP_QCQP_GET_LB(int stage, struct OCP_QCQP *qp, REAL *lb)
	{
	// extract dim
	int *nb = qp->dim->nb;

	int i;

	UNPACK_VEC(nb[stage], qp->d+stage, 0, lb, 1);

	return;
	}



void OCP_QCQP_GET_LB_MASK(int stage, struct OCP_QCQP *qp, REAL *lb_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;

	int i;

	UNPACK_VEC(nb[stage], qp->d_mask+stage, 0, lb_mask, 1);

	return;
	}



void OCP_QCQP_GET_UB(int stage, struct OCP_QCQP *qp, REAL *ub)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(nb[stage], qp->d+stage, nb[stage]+ng[stage]+nq[stage], ub, 1);
	for(i=0; i<nb[stage]; i++)
		{
		ub[i] = -ub[i];
		}

	return;
	}



void OCP_QCQP_GET_UB_MASK(int stage, struct OCP_QCQP *qp, REAL *ub_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(nb[stage], qp->d_mask+stage, nb[stage]+ng[stage]+nq[stage], ub_mask, 1);

	return;
	}



void OCP_QCQP_GET_LBX(int stage, struct OCP_QCQP *qp, REAL *lbx)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;

	UNPACK_VEC(nbx[stage], qp->d+stage, nbu[stage], lbx, 1);

	return;
	}



void OCP_QCQP_GET_LBX_MASK(int stage, struct OCP_QCQP *qp, REAL *lbx_mask)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;

	UNPACK_VEC(nbx[stage], qp->d_mask+stage, nbu[stage], lbx_mask, 1);

	return;
	}



void OCP_QCQP_GET_UBX(int stage, struct OCP_QCQP *qp, REAL *ubx)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(nbx[stage], qp->d+stage, nb[stage]+ng[stage]+nq[stage]+nbu[stage], ubx, 1);
	for(i=0; i<nbx[stage]; i++)
		{
		ubx[i] = -ubx[i];
		}

	return;
	}



void OCP_QCQP_GET_UBX_MASK(int stage, struct OCP_QCQP *qp, REAL *ubx_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(nbx[stage], qp->d_mask+stage, nb[stage]+ng[stage]+nq[stage]+nbu[stage], ubx_mask, 1);

	return;
	}



void OCP_QCQP_GET_LBU(int stage, struct OCP_QCQP *qp, REAL *lbu)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	UNPACK_VEC(nbu[stage], qp->d+stage, 0, lbu, 1);

	return;
	}



void OCP_QCQP_GET_LBU_MASK(int stage, struct OCP_QCQP *qp, REAL *lbu_mask)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	UNPACK_VEC(nbu[stage], qp->d_mask+stage, 0, lbu_mask, 1);

	return;
	}



void OCP_QCQP_GET_UBU(int stage, struct OCP_QCQP *qp, REAL *ubu)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(nbu[stage], qp->d+stage, nb[stage]+ng[stage]+nq[stage], ubu, 1);
	for(i=0; i<nbu[stage]; i++)
		{
		ubu[i] = -ubu[i];
		}

	return;
	}



void OCP_QCQP_GET_UBU_MASK(int stage, struct OCP_QCQP *qp, REAL *ubu_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(nbu[stage], qp->d_mask+stage, nb[stage]+ng[stage]+nq[stage], ubu_mask, 1);

	return;
	}



void OCP_QCQP_GET_IDXB(int stage, struct OCP_QCQP *qp, int *idxb)
	{
	// extract dim
	int *nb = qp->dim->nb;

	int ii;
	for(ii=0; ii<nb[stage]; ii++)
		idxb[ii] = qp->idxb[stage][ii];

	return;
	}



//void OCP_QCQP_GET_IDXBX(int stage, struct OCP_QCQP *qp, int *idxb)
//	{
//	TODO
//	return;
//	}



//void OCP_QCQP_GET_JBX(int stage, struct OCP_QCQP *qp, int *Jbx)
//	{
//	TODO
//	return;
//	}



//void OCP_QCQP_GET_IDXBU(int stage, struct OCP_QCQP *qp, int *idxbu)
//	{
//	TODO
//	return;
//	}



//void OCP_QCQP_GET_JBU(int stage, struct OCP_QCQP *qp, int *Jbu)
//	{
//	TODO
//	return;
//	}



void OCP_QCQP_GET_C(int stage, struct OCP_QCQP *qp, REAL *C)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;

	CVT_TRAN_STRMAT2MAT(nx[stage], ng[stage], qp->DCt+stage, nu[stage], 0, C, ng[stage]);

	return;
	}



void OCP_QCQP_GET_D(int stage, struct OCP_QCQP *qp, REAL *D)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;

	CVT_TRAN_STRMAT2MAT(nu[stage], ng[stage], qp->DCt+stage, 0, 0, D, ng[stage]);

	return;
	}



void OCP_QCQP_GET_LG(int stage, struct OCP_QCQP *qp, REAL *lg)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	UNPACK_VEC(ng[stage], qp->d+stage, nb[stage], lg, 1);

	return;
	}



void OCP_QCQP_GET_LG_MASK(int stage, struct OCP_QCQP *qp, REAL *lg_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	UNPACK_VEC(ng[stage], qp->d_mask+stage, nb[stage], lg_mask, 1);

	return;
	}



void OCP_QCQP_GET_UG(int stage, struct OCP_QCQP *qp, REAL *ug)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(ng[stage], qp->d+stage, 2*nb[stage]+ng[stage]+nq[stage], ug, 1);
	for(i=0; i<ng[stage]; i++)
		{
		ug[i] = -ug[i];
		}

	return;
	}



void OCP_QCQP_GET_UG_MASK(int stage, struct OCP_QCQP *qp, REAL *ug_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int i;

	UNPACK_VEC(ng[stage], qp->d_mask+stage, 2*nb[stage]+ng[stage]+nq[stage], ug_mask, 1);

	return;
	}



void OCP_QCQP_GET_ZL(int stage, struct OCP_QCQP *qp, REAL *Zl)
	{
	// extract dim
	int *ns = qp->dim->ns;

	UNPACK_VEC(ns[stage], qp->Z+stage, 0, Zl, 1);

	return;
	}



void OCP_QCQP_GET_ZU(int stage, struct OCP_QCQP *qp, REAL *Zu)
	{
	// extract dim
	int *ns = qp->dim->ns;

	UNPACK_VEC(ns[stage], qp->Z+stage, ns[stage], Zu, 1);

	return;
	}



void OCP_QCQP_GET_ZLVEC(int stage, struct OCP_QCQP *qp, REAL *zl)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nx = qp->dim->nx;
	int *ns = qp->dim->ns;

	UNPACK_VEC(ns[stage], qp->rqz+stage, nu[stage]+nx[stage], zl, 1);

	return;
	}



void OCP_QCQP_GET_ZUVEC(int stage, struct OCP_QCQP *qp, REAL *zu)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nx = qp->dim->nx;
	int *ns = qp->dim->ns;

	UNPACK_VEC(ns[stage], qp->rqz+stage, nu[stage]+nx[stage]+ns[stage], zu, 1);

	return;
	}



// XXX only valid if there is one slack per softed constraint !!!
void OCP_QCQP_GET_IDXS(int stage, struct OCP_QCQP *qp, int *idxs)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	int ii, idx_tmp;

	for(ii=0; ii<nb[stage]+ng[stage]+nq[stage]; ii++)
		{
		idx_tmp = qp->idxs_rev[stage][ii];
		if(idx_tmp!=-1)
			{
			idxs[idx_tmp] = ii;
			}
		}

	return;
	}



void OCP_QCQP_GET_IDXS_REV(int stage, struct OCP_QCQP *qp, int *idxs_rev)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;

	int ii;
	for(ii=0; ii<nb[stage]+ng[stage]+nq[stage]; ii++)
		idxs_rev[ii] = qp->idxs_rev[stage][ii];

	return;
	}



//void OCP_QCQP_GET_JSBX(int stage, struct OCP_QCQP *qp, int *Jsbx)
//	{
//	TODO
//	return;
//	}



//void OCP_QCQP_GET_JSBX(int stage, struct OCP_QCQP *qp, int *Jsbx)
//	{
//	TODO
//	return;
//	}



//void OCP_QCQP_GET_JSG(int stage, struct OCP_QCQP *qp, int *Jsg)
//	{
//	TODO
//	return;
//	}



void OCP_QCQP_GET_LLS(int stage, struct OCP_QCQP *qp, REAL *ls)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	UNPACK_VEC(ns[stage], qp->d+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage], ls, 1);

	return;
	}



void OCP_QCQP_GET_LLS_MASK(int stage, struct OCP_QCQP *qp, REAL *ls_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	UNPACK_VEC(ns[stage], qp->d_mask+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage], ls_mask, 1);

	return;
	}



void OCP_QCQP_GET_LUS(int stage, struct OCP_QCQP *qp, REAL *us)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	int i;

	UNPACK_VEC(ns[stage], qp->d+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage]+ns[stage], us, 1);

	return;
	}



void OCP_QCQP_GET_LUS_MASK(int stage, struct OCP_QCQP *qp, REAL *us_mask)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	int i;

	UNPACK_VEC(ns[stage], qp->d_mask+stage, 2*nb[stage]+2*ng[stage]+2*nq[stage]+ns[stage], us_mask, 1);

	return;
	}





