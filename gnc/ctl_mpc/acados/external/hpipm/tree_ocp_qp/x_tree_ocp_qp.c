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



hpipm_size_t TREE_OCP_QP_STRSIZE()
	{
	return sizeof(struct TREE_OCP_QP);
	}



hpipm_size_t TREE_OCP_QP_MEMSIZE(struct TREE_OCP_QP_DIM *dim)
	{

	// extract dim
	struct tree *ttree = dim->ttree;
	int Nn = dim->Nn;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *ns = dim->ns;

	int ii, idx, idxdad;

	// compute core qp size
	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<Nn; ii++)
		{
		nvt += nx[ii]+nu[ii]+2*ns[ii];
		nct += 2*nb[ii]+2*ng[ii]+2*ns[ii];
		}
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		net += nx[idx];
		}

	hpipm_size_t size = 0;

	size += 2*Nn*sizeof(int *); // idxb idxs_rev
//	size += 1*Nn*sizeof(int *); // idxs
	size += 2*Nn*sizeof(struct STRMAT); // RSQrq DCt
	size += 1*(Nn-1)*sizeof(struct STRMAT); // BAbt
	size += 5*Nn*sizeof(struct STRVEC); // rqz d m Z d_mask
	size += 1*(Nn-1)*sizeof(struct STRVEC); // b

	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		idxdad = (ttree->root+idx)->dad;
		size += SIZE_STRMAT(nu[idxdad]+nx[idxdad]+1, nx[idx]); // BAbt
		}

	for(ii=0; ii<Nn; ii++)
		{
		size += nb[ii]*sizeof(int); // idxb
		size += (nb[ii]+ng[ii])*sizeof(int); // idxs_rev
//		size += ns[ii]*sizeof(int); // idxs
		size += SIZE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // RSQrq
		size += SIZE_STRMAT(nu[ii]+nx[ii], ng[ii]); // DCt
		size += SIZE_STRVEC(2*ns[ii]); // Z
		}

	size += 1*SIZE_STRVEC(nvt); // rqz
	size += 1*SIZE_STRVEC(net); // b
	size += 3*SIZE_STRVEC(nct); // d m d_mask

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 64; // align to typical cache line size

	return size;

	}



void TREE_OCP_QP_CREATE(struct TREE_OCP_QP_DIM *dim, struct TREE_OCP_QP *qp, void *mem)
	{

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = TREE_OCP_QP_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// extract dim
	struct tree *ttree = dim->ttree;
	int Nn = dim->Nn;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *ns = dim->ns;

	int ii, jj, idx, idxdad;



	// compute core qp size
	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<Nn; ii++)
		{
		nvt += nx[ii]+nu[ii]+2*ns[ii];
		nct += 2*nb[ii]+2*ng[ii]+2*ns[ii];
		}
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		net += nx[idx];
		}



	// int pointer stuff
	int **ip_ptr;
	ip_ptr = (int **) mem;

	// idxb
	qp->idxb = ip_ptr;
	ip_ptr += Nn;

	// idxs_rev
	qp->idxs_rev = ip_ptr;
	ip_ptr += Nn;

//	// idxs
//	qp->idxs = ip_ptr;
//	ip_ptr += Nn;


	// matrix struct stuff
	struct STRMAT *sm_ptr = (struct STRMAT *) ip_ptr;

	// BAbt
	qp->BAbt = sm_ptr;
	sm_ptr += Nn-1;

	// RSQrq
	qp->RSQrq = sm_ptr;
	sm_ptr += Nn;

	// DCt
	qp->DCt = sm_ptr;
	sm_ptr += Nn;


	// vector struct stuff
	struct STRVEC *sv_ptr = (struct STRVEC *) sm_ptr;

	// b
	qp->b = sv_ptr;
	sv_ptr += Nn-1;

	// rqz
	qp->rqz = sv_ptr;
	sv_ptr += Nn;

	// d
	qp->d = sv_ptr;
	sv_ptr += Nn;

	// d_mask
	qp->d_mask = sv_ptr;
	sv_ptr += Nn;

	// m
	qp->m = sv_ptr;
	sv_ptr += Nn;

	// Z
	qp->Z = sv_ptr;
	sv_ptr += Nn;


	// integer stuff
	int *i_ptr;
	i_ptr = (int *) sv_ptr;

	// idxb
	for(ii=0; ii<Nn; ii++)
		{
		(qp->idxb)[ii] = i_ptr;
		i_ptr += nb[ii];
		}

	// idxs_rev
	for(ii=0; ii<Nn; ii++)
		{
		(qp->idxs_rev)[ii] = i_ptr;
		i_ptr += nb[ii]+ng[ii];
		// default value: -1
		for(jj=0; jj<nb[ii]+ng[ii]; jj++)
			qp->idxs_rev[ii][jj] = -1;
		}

//	// idxs
//	for(ii=0; ii<Nn; ii++)
//		{
//		(qp->idxs)[ii] = i_ptr;
//		i_ptr += ns[ii];
//		}


	// align to typical cache line size
	hpipm_size_t l_ptr = (hpipm_size_t) i_ptr;
	l_ptr = (l_ptr+63)/64*64;


	// double stuff
	char *c_ptr, *tmp_ptr;
	c_ptr = (char *) l_ptr;

	// BAbt
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		idxdad = (ttree->root+idx)->dad;
		CREATE_STRMAT(nu[idxdad]+nx[idxdad]+1, nx[idx], qp->BAbt+ii, c_ptr);
		c_ptr += (qp->BAbt+ii)->memsize;
		}

	// RSQrq
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], qp->RSQrq+ii, c_ptr);
		c_ptr += (qp->RSQrq+ii)->memsize;
		}

	// DCt
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRMAT(nu[ii]+nx[ii], ng[ii], qp->DCt+ii, c_ptr);
		c_ptr += (qp->DCt+ii)->memsize;
		}

	// Z
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(2*ns[ii], qp->Z+ii, c_ptr);
		c_ptr += (qp->Z+ii)->memsize;
		}

	// g
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nvt);
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(nu[ii]+nx[ii]+2*ns[ii], qp->rqz+ii, tmp_ptr);
		tmp_ptr += nu[ii]*sizeof(REAL);
		tmp_ptr += nx[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		}

	// b
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(net);
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		CREATE_STRVEC(nx[idx], qp->b+ii, tmp_ptr);
		tmp_ptr += nx[idx]*sizeof(REAL);
		}

	// d
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*ns[ii], qp->d+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL);
		tmp_ptr += ng[ii]*sizeof(REAL);
		tmp_ptr += nb[ii]*sizeof(REAL);
		tmp_ptr += ng[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		}

	// d_mask
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*ns[ii], qp->d_mask+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL);
		tmp_ptr += ng[ii]*sizeof(REAL);
		tmp_ptr += nb[ii]*sizeof(REAL);
		tmp_ptr += ng[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		VECSE(2*nb[ii]+2*ng[ii]+2*ns[ii], 1.0, qp->d_mask+ii, 0);
		}

	// m
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nct);
	for(ii=0; ii<Nn; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*ns[ii], qp->m+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL);
		tmp_ptr += ng[ii]*sizeof(REAL);
		tmp_ptr += nb[ii]*sizeof(REAL);
		tmp_ptr += ng[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		tmp_ptr += ns[ii]*sizeof(REAL);
		}

	qp->dim = dim;

	qp->memsize = memsize;

#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + qp->memsize)
		{
		printf("\nCreate_tree_ocp_qp: outside memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void TREE_OCP_QP_SET_ALL(REAL **A, REAL **B, REAL **b, REAL **Q, REAL **S, REAL **R, REAL **q, REAL **r, int **idxb, REAL **d_lb, REAL **d_ub, REAL **C, REAL **D, REAL **d_lg, REAL **d_ug, REAL **Zl, REAL **Zu, REAL **zl, REAL **zu, int **idxs, REAL **d_ls, REAL **d_us, struct TREE_OCP_QP *qp)
	{

	int Nn = qp->dim->Nn;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	struct tree *ttree = qp->dim->ttree;

	int ii, jj, idx, idxdad;

	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		idxdad = (ttree->root+idx)->dad;
		PACK_TRAN_MAT(nx[idx], nu[idxdad], B[ii], nx[idx], qp->BAbt+ii, 0, 0);
		PACK_TRAN_MAT(nx[idx], nx[idxdad], A[ii], nx[idx], qp->BAbt+ii, nu[idxdad], 0);
		PACK_TRAN_MAT(nx[idx], 1, b[ii], nx[idx], qp->BAbt+ii, nu[idxdad]+nx[idxdad], 0);
		PACK_VEC(nx[idx], b[ii], 1, qp->b+ii, 0);
		}

	for(ii=0; ii<Nn; ii++)
		{
		PACK_MAT(nu[ii], nu[ii], R[ii], nu[ii], qp->RSQrq+ii, 0, 0);
		PACK_TRAN_MAT(nu[ii], nx[ii], S[ii], nu[ii], qp->RSQrq+ii, nu[ii], 0);
		PACK_MAT(nx[ii], nx[ii], Q[ii], nx[ii], qp->RSQrq+ii, nu[ii], nu[ii]);
		PACK_TRAN_MAT(nu[ii], 1, r[ii], nu[ii], qp->RSQrq+ii, nu[ii]+nx[ii], 0);
		PACK_TRAN_MAT(nx[ii], 1, q[ii], nx[ii], qp->RSQrq+ii, nu[ii]+nx[ii], nu[ii]);
		PACK_VEC(nu[ii], r[ii], 1, qp->rqz+ii, 0);
		PACK_VEC(nx[ii], q[ii], 1, qp->rqz+ii, nu[ii]);
		}

	for(ii=0; ii<Nn; ii++)
		{
		if(nb[ii]>0)
			{
			for(jj=0; jj<nb[ii]; jj++)
				qp->idxb[ii][jj] = idxb[ii][jj];
			PACK_VEC(nb[ii], d_lb[ii], 1, qp->d+ii, 0);
			PACK_VEC(nb[ii], d_ub[ii], 1, qp->d+ii, nb[ii]+ng[ii]);
			VECSC(nb[ii], -1.0, qp->d+ii, nb[ii]+ng[ii]);
			VECSE(nb[ii], 0.0, qp->m+ii, 0);
			VECSE(nb[ii], 0.0, qp->m+ii, nb[ii]+ng[ii]);
			}
		}

	for(ii=0; ii<Nn; ii++)
		{
		if(ng[ii]>0)
			{
			PACK_TRAN_MAT(ng[ii], nu[ii], D[ii], ng[ii], qp->DCt+ii, 0, 0);
			PACK_TRAN_MAT(ng[ii], nx[ii], C[ii], ng[ii], qp->DCt+ii, nu[ii], 0);
			PACK_VEC(ng[ii], d_lg[ii], 1, qp->d+ii, nb[ii]);
			PACK_VEC(ng[ii], d_ug[ii], 1, qp->d+ii, 2*nb[ii]+ng[ii]);
			VECSC(ng[ii], -1.0, qp->d+ii, 2*nb[ii]+ng[ii]);
			VECSE(ng[ii], 0.0, qp->m+ii, nb[ii]);
			VECSE(ng[ii], 0.0, qp->m+ii, 2*nb[ii]+ng[ii]);
			}
		}

	for(ii=0; ii<Nn; ii++)
		{
		if(ns[ii]>0)
			{
			// TODO idxs_rev
			for(jj=0; jj<ns[ii]; jj++)
				{
//				qp->idxs[ii][jj] = idxs[ii][jj];
				qp->idxs_rev[ii][idxs[ii][jj]] = jj;
				}
			PACK_VEC(ns[ii], Zl[ii], 1, qp->Z+ii, 0);
			PACK_VEC(ns[ii], Zu[ii], 1, qp->Z+ii, ns[ii]);
			PACK_VEC(ns[ii], zl[ii], 1, qp->rqz+ii, nu[ii]+nx[ii]);
			PACK_VEC(ns[ii], zu[ii], 1, qp->rqz+ii, nu[ii]+nx[ii]+ns[ii]);
			PACK_VEC(ns[ii], d_ls[ii], 1, qp->d+ii, 2*nb[ii]+2*ng[ii]);
			PACK_VEC(ns[ii], d_us[ii], 1, qp->d+ii, 2*nb[ii]+2*ng[ii]+ns[ii]);
			VECSE(ns[ii], 0.0, qp->m+ii, 2*nb[ii]+2*ng[ii]);
			VECSE(ns[ii], 0.0, qp->m+ii, 2*nb[ii]+2*ng[ii]+ns[ii]);
			}
		}

	return;

	}



void TREE_OCP_QP_SET(char *field, int node_edge, void *value, struct TREE_OCP_QP *qp)
	{
	REAL *r_ptr;
	int *i_ptr;
    
	// matrices
	if(hpipm_strcmp(field, "A")) 
		{
		TREE_OCP_QP_SET_A(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "B")) 
		{
		TREE_OCP_QP_SET_B(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Q")) 
		{
		TREE_OCP_QP_SET_Q(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "S")) 
		{
		TREE_OCP_QP_SET_S(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "R")) 
		{
		TREE_OCP_QP_SET_R(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "C")) 
		{
		TREE_OCP_QP_SET_C(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "D")) 
		{
		TREE_OCP_QP_SET_D(node_edge, value, qp);
		}
	// vectors
	else if(hpipm_strcmp(field, "b"))
		{ 
		TREE_OCP_QP_SET_BVEC(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "q"))
		{ 
		TREE_OCP_QP_SET_QVEC(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "r"))
		{ 
		TREE_OCP_QP_SET_RVEC(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lb"))
		{ 
		TREE_OCP_QP_SET_LB(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lb_mask"))
		{ 
		TREE_OCP_QP_SET_LB_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lbu") | hpipm_strcmp(field, "lu"))
		{ 
		TREE_OCP_QP_SET_LBU(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lbu_mask"))
		{ 
		TREE_OCP_QP_SET_LBU_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lbx") | hpipm_strcmp(field, "lx"))
		{ 
		TREE_OCP_QP_SET_LBX(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lbx_mask"))
		{ 
		TREE_OCP_QP_SET_LBX_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ub"))
		{ 
		TREE_OCP_QP_SET_UB(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ub_mask"))
		{ 
		TREE_OCP_QP_SET_UB_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ubu") | hpipm_strcmp(field, "uu"))
		{ 
		TREE_OCP_QP_SET_UBU(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ubu_mask"))
		{ 
		TREE_OCP_QP_SET_UBU_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ubx") | hpipm_strcmp(field, "ux"))
		{ 
		TREE_OCP_QP_SET_UBX(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ubx_mask"))
		{ 
		TREE_OCP_QP_SET_UBX_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lg"))
		{ 
		TREE_OCP_QP_SET_LG(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lg_mask"))
		{ 
		TREE_OCP_QP_SET_LG_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ug"))
		{ 
		TREE_OCP_QP_SET_UG(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "ug_mask"))
		{ 
		TREE_OCP_QP_SET_UG_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Zl"))
		{ 
		TREE_OCP_QP_SET_ZL(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Zu"))
		{ 
		TREE_OCP_QP_SET_ZU(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "zl"))
		{ 
		TREE_OCP_QP_SET_ZLVEC(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "zu"))
		{ 
		TREE_OCP_QP_SET_ZUVEC(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lls"))
		{ 
		TREE_OCP_QP_SET_LLS(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lls_mask"))
		{ 
		TREE_OCP_QP_SET_LLS_MASK(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lus"))
		{ 
		TREE_OCP_QP_SET_LUS(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "lus_mask"))
		{ 
		TREE_OCP_QP_SET_LUS_MASK(node_edge, value, qp);
		}
	// int
	else if(hpipm_strcmp(field, "idxb"))
		{
		TREE_OCP_QP_SET_IDXB(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "idxbx"))
		{
		TREE_OCP_QP_SET_IDXBX(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Jbx") | hpipm_strcmp(field, "Jx"))
		{
		TREE_OCP_QP_SET_JBX(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "idxbu"))
		{
		TREE_OCP_QP_SET_IDXBU(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Jbu") | hpipm_strcmp(field, "Ju"))
		{
		TREE_OCP_QP_SET_JBU(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "idxs"))
		{
		TREE_OCP_QP_SET_IDXS(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "idxs_rev"))
		{
		TREE_OCP_QP_SET_IDXS_REV(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Jsbu") | hpipm_strcmp(field, "Jsu"))
		{
		TREE_OCP_QP_SET_JSBU(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Jsbx") | hpipm_strcmp(field, "Jsx"))
		{
		TREE_OCP_QP_SET_JSBX(node_edge, value, qp);
		}
	else if(hpipm_strcmp(field, "Jsg"))
		{
		TREE_OCP_QP_SET_JSG(node_edge, value, qp);
		}
//	else if(hpipm_strcmp(field, "idxe"))
//		{
//		TREE_OCP_QP_SET_IDXE(node_edge, value, qp);
//		}
//	else if(hpipm_strcmp(field, "idxbue"))
//		{
//		TREE_OCP_QP_SET_IDXBUE(node_edge, value, qp);
//		}
//	else if(hpipm_strcmp(field, "idxbxe"))
//		{
//		TREE_OCP_QP_SET_IDXBXE(node_edge, value, qp);
//		}
//	else if(hpipm_strcmp(field, "idxge"))
//		{
//		TREE_OCP_QP_SET_IDXGE(node_edge, value, qp);
//		}
//	else if(hpipm_strcmp(field, "Jbue"))
//		{
//		TREE_OCP_QP_SET_JBUE(node_edge, value, qp);
//		}
//	else if(hpipm_strcmp(field, "Jbxe"))
//		{
//		TREE_OCP_QP_SET_JBXE(node_edge, value, qp);
//		}
//	else if(hpipm_strcmp(field, "Jge"))
//		{
//		TREE_OCP_QP_SET_JGE(node_edge, value, qp);
//		}
//	else if(hpipm_strcmp(field, "diag_H_flag"))
//		{
//		TREE_OCP_QP_SET_DIAG_H_FLAG(node_edge, value, qp);
//		}
	else
		{
		printf("error: TREE_OCP_QP_SET: wrong field name '%s'. Exiting.\n", field);
		exit(1);	
		}
	return;
	}



void TREE_OCP_QP_SET_A(int edge, REAL *A, struct TREE_OCP_QP *qp)
	{
	// extract dim
	struct tree *ttree = qp->dim->ttree;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	int idx = edge+1;
	int idxdad = (ttree->root+idx)->dad;

	PACK_TRAN_MAT(nx[idx], nx[idxdad], A, nx[idx], qp->BAbt+edge, nu[idxdad], 0);

	return;
	}



void TREE_OCP_QP_SET_B(int edge, REAL *B, struct TREE_OCP_QP *qp)
	{
	// extract dim
	struct tree *ttree = qp->dim->ttree;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	int idx = edge+1;
	int idxdad = (ttree->root+idx)->dad;

	PACK_TRAN_MAT(nx[idx], nu[idxdad], B, nx[idx], qp->BAbt+edge, 0, 0);

	return;
	}



void TREE_OCP_QP_SET_BVEC(int edge, REAL *b, struct TREE_OCP_QP *qp)
	{
	// extract dim
	struct tree *ttree = qp->dim->ttree;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	int idx = edge+1;
	int idxdad = (ttree->root+idx)->dad;

	PACK_TRAN_MAT(nx[idx], 1, b, nx[idx], qp->BAbt+edge, nu[idxdad]+nx[idxdad], 0); // TODO remove ???
	PACK_VEC(nx[idx], b, 1, qp->b+edge, 0);

	return;
	}



void TREE_OCP_QP_SET_Q(int node, REAL *Q, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	PACK_MAT(nx[node], nx[node], Q, nx[node], qp->RSQrq+node, nu[node], nu[node]);

	return;
	}



void TREE_OCP_QP_SET_S(int node, REAL *S, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	PACK_TRAN_MAT(nu[node], nx[node], S, nu[node], qp->RSQrq+node, nu[node], 0);

	return;
	}



void TREE_OCP_QP_SET_R(int node, REAL *R, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	PACK_MAT(nu[node], nu[node], R, nu[node], qp->RSQrq+node, 0, 0);

	return;
	}



void TREE_OCP_QP_SET_QVEC(int node, REAL *q, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

 	PACK_TRAN_MAT(nx[node], 1, q, nx[node], &(qp->RSQrq[node]), nu[node]+nx[node], nu[node]); // TODO remove ???
	PACK_VEC(nx[node], q, 1, qp->rqz+node, nu[node]);

	return;
	}



void TREE_OCP_QP_SET_RVEC(int node, REAL *r, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;

	PACK_TRAN_MAT(nu[node], 1, r, nu[node], &(qp->RSQrq[node]), nu[node]+nx[node], 0); // TODO remove ???
	PACK_VEC(nu[node], r, 1, qp->rqz+node, 0);

	return;
	}



void TREE_OCP_QP_SET_LB(int node, REAL *lb, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;

	PACK_VEC(nb[node], lb, 1, qp->d+node, 0);

	return;
	}



void TREE_OCP_QP_SET_LB_MASK(int node, REAL *lb_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;

	PACK_VEC(nb[node], lb_mask, 1, qp->d_mask+node, 0);

	return;
	}



void TREE_OCP_QP_SET_LBX(int node, REAL *lbx, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;

	PACK_VEC(nbx[node], lbx, 1, qp->d+node, nbu[node]);

	return;
	}



void TREE_OCP_QP_SET_LBX_MASK(int node, REAL *lbx_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;

	PACK_VEC(nbx[node], lbx_mask, 1, qp->d_mask+node, nbu[node]);

	return;
	}



void TREE_OCP_QP_SET_LBU(int node, REAL *lbu, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	PACK_VEC(nbu[node], lbu, 1, qp->d+node, 0);

	return;
	}



void TREE_OCP_QP_SET_LBU_MASK(int node, REAL *lbu_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	PACK_VEC(nbu[node], lbu_mask, 1, qp->d_mask+node, 0);

	return;
	}



void TREE_OCP_QP_SET_UB(int node, REAL *ub, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(nb[node], ub, 1, qp->d+node, nb[node]+ng[node]);
	VECSC(nb[node], -1.0, qp->d+node, nb[node]+ng[node]);

	return;
	}



void TREE_OCP_QP_SET_UB_MASK(int node, REAL *ub_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(nb[node], ub_mask, 1, qp->d_mask+node, nb[node]+ng[node]);

	return;
	}



void TREE_OCP_QP_SET_UBX(int node, REAL *ubx, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;

	PACK_VEC(nbx[node], ubx, 1, qp->d+node, nb[node]+ng[node]+nbu[node]);
	VECSC(nbx[node], -1.0, qp->d+node, nb[node]+ng[node]+nbu[node]);

	return;
	}



void TREE_OCP_QP_SET_UBX_MASK(int node, REAL *ubx_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;

	PACK_VEC(nbx[node], ubx_mask, 1, qp->d_mask+node, nb[node]+ng[node]+nbu[node]);

	return;
	}



void TREE_OCP_QP_SET_UBU(int node, REAL *ubu, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;

	PACK_VEC(nbu[node], ubu, 1, qp->d+node, nb[node]+ng[node]);
	VECSC(nbu[node], -1.0, qp->d+node, nb[node]+ng[node]);

	return;
	}



void TREE_OCP_QP_SET_UBU_MASK(int node, REAL *ubu_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *nbu = qp->dim->nbu;
	int *ng = qp->dim->ng;

	PACK_VEC(nbu[node], ubu_mask, 1, qp->d_mask+node, nb[node]+ng[node]);

	return;
	}



void TREE_OCP_QP_SET_IDXB(int node, int *idxb, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;

	int ii;
	for(ii=0; ii<nb[node]; ii++)
		qp->idxb[node][ii] = idxb[ii];

	return;
	}



void TREE_OCP_QP_SET_IDXBX(int node, int *idxbx, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;

	int ii;
	for(ii=0; ii<nbx[node]; ii++)
		{
		qp->idxb[node][nbu[node]+ii] = nu[node] + idxbx[ii];
		}

	return;
	}



void TREE_OCP_QP_SET_JBX(int node, REAL *Jbx, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nbx = qp->dim->nbx;
	int *nbu = qp->dim->nbu;

	int ii, jj, jj0;
	for(ii=0; ii<nbx[node]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<nx[node]; jj++)
			{
			if(jj0==-1 & Jbx[ii+jj*nbx[node]]!=0.0)
				{
				jj0 = jj;
				qp->idxb[node][nbu[node]+ii] = nu[node]+jj;
				}
			}
		}
	return;
	}



void TREE_OCP_QP_SET_IDXBU(int node, int *idxbx, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;

	int ii;
	for(ii=0; ii<nbu[node]; ii++)
		{
		qp->idxb[node][ii] = idxbx[ii];
		}

	return;
	}



void TREE_OCP_QP_SET_JBU(int node, REAL *Jbu, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nbu = qp->dim->nbu;

	int ii, jj, jj0;
	for(ii=0; ii<nbu[node]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<nu[node]; jj++)
			{
			if(jj0==-1 & Jbu[ii+jj*nbu[node]]!=0.0)
				{
				jj0 = jj;
				qp->idxb[node][ii] = jj;
				}
			}
		}
	return;
	}



void TREE_OCP_QP_SET_C(int node, REAL *C, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;

	PACK_TRAN_MAT(ng[node], nx[node], C, ng[node], qp->DCt+node, nu[node], 0);

	return;
	}



void TREE_OCP_QP_SET_D(int node, REAL *D, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *ng = qp->dim->ng;

	PACK_TRAN_MAT(ng[node], nu[node], D, ng[node], qp->DCt+node, 0, 0);

	return;
	}



void TREE_OCP_QP_SET_LG(int node, REAL *lg, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(ng[node], lg, 1, qp->d+node, nb[node]);

	return;
	}



void TREE_OCP_QP_SET_LG_MASK(int node, REAL *lg_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(ng[node], lg_mask, 1, qp->d_mask+node, nb[node]);

	return;
	}



void TREE_OCP_QP_SET_UG(int node, REAL *ug, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(ng[node], ug, 1, qp->d+node, 2*nb[node]+ng[node]);
	VECSC(ng[node], -1.0, qp->d+node, 2*nb[node]+ng[node]);

	return;
	}



void TREE_OCP_QP_SET_UG_MASK(int node, REAL *ug_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	PACK_VEC(ng[node], ug_mask, 1, qp->d_mask+node, 2*nb[node]+ng[node]);

	return;
	}



void TREE_OCP_QP_SET_ZL(int node, REAL *Zl, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], Zl, 1, qp->Z+node, 0);

	return;
	}



void TREE_OCP_QP_SET_ZU(int node, REAL *Zu, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], Zu, 1, qp->Z+node, ns[node]);

	return;
	}



void TREE_OCP_QP_SET_ZLVEC(int node, REAL *zl, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nx = qp->dim->nx;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], zl, 1, qp->rqz+node, nu[node]+nx[node]);

	return;
	}



void TREE_OCP_QP_SET_ZUVEC(int node, REAL *zu, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nu = qp->dim->nu;
	int *nx = qp->dim->nx;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], zu, 1, qp->rqz+node, nu[node]+nx[node]+ns[node]);

	return;
	}



void TREE_OCP_QP_SET_IDXS(int node, int *idxs, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *ns = qp->dim->ns;

	int ii;
	for(ii=0; ii<ns[node]; ii++)
		{
		// TODO idxs_rev
		qp->idxs_rev[node][idxs[ii]] = ii;
//		qp->idxs[node][ii] = idxs[ii];
		}

	return;
	}



void TREE_OCP_QP_SET_IDXS_REV(int node, int *idxs_rev, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;

	int ii;
	for(ii=0; ii<nb[node]+ng[node]; ii++)
		{
		qp->idxs_rev[node][ii] = idxs_rev[ii];
		}

	return;
	}



void TREE_OCP_QP_SET_JSBU(int node, REAL *Jsbu, struct TREE_OCP_QP *qp)
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
	for(ii=0; ii<nbu[node]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<ns[node]; jj++)
			{
			if(jj0==-1 & Jsbu[ii+jj*nbu[node]]!=0.0)
				{
				jj0 = jj;
				qp->idxs_rev[node][0+ii] = jj;
				}
			}
		}
	return;
	}



void TREE_OCP_QP_SET_JSBX(int node, REAL *Jsbx, struct TREE_OCP_QP *qp)
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
	for(ii=0; ii<nbx[node]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<ns[node]; jj++)
			{
			if(jj0==-1 & Jsbx[ii+jj*nbx[node]]!=0.0)
				{
				jj0 = jj;
				qp->idxs_rev[node][nbu[node]+ii] = jj;
				}
			}
		}
	return;
	}



void TREE_OCP_QP_SET_JSG(int node, REAL *Jsg, struct TREE_OCP_QP *qp)
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
	// compute ng part of idxs_rev
	for(ii=0; ii<ng[node]; ii++)
		{
		jj0 = -1;
		for(jj=0; jj<ns[node]; jj++)
			{
			if(jj0==-1 & Jsg[ii+jj*ng[node]]!=0.0)
				{
				jj0 = jj;
				qp->idxs_rev[node][nb[node]+ii] = jj;
				}
			}
		}
	return;
	}



void TREE_OCP_QP_SET_LLS(int node, REAL *ls, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], ls, 1, qp->d+node, 2*nb[node]+2*ng[node]);

	return;
	}



void TREE_OCP_QP_SET_LLS_MASK(int node, REAL *ls_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], ls_mask, 1, qp->d_mask+node, 2*nb[node]+2*ng[node]);

	return;
	}



void TREE_OCP_QP_SET_LUS(int node, REAL *us, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], us, 1, qp->d+node, 2*nb[node]+2*ng[node]+ns[node]);

	return;
	}



void TREE_OCP_QP_SET_LUS_MASK(int node, REAL *lus_mask, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *ns = qp->dim->ns;

	PACK_VEC(ns[node], lus_mask, 1, qp->d_mask+node, 2*nb[node]+2*ng[node]+ns[node]);

	return;
	}



//void TREE_OCP_QP_SET_IDXE(int node, int *idxe, struct TREE_OCP_QP *qp)
//	{
//	// extract dim
//	int *nbxe = qp->dim->nbxe;
//	int *nbue = qp->dim->nbue;
//	int *nge = qp->dim->nge;
//
//	int ii;
//	for(ii=0; ii<nbxe[node]+nbue[node]+nge[node]; ii++)
//		qp->idxe[node][ii] = idxe[ii];
//
//	return;
//	}



//void TREE_OCP_QP_SET_IDXBUE(int node, int *idxbue, struct TREE_OCP_QP *qp)
//	{
//	// extract dim
//	int *nbxe = qp->dim->nbxe;
//	int *nbue = qp->dim->nbue;
//	int *nge = qp->dim->nge;
//
//	int ii;
//	for(ii=0; ii<nbue[node]; ii++)
//		qp->idxe[node][ii] = idxbue[ii];
//
//	return;
//	}



#if 0
void TREE_OCP_QP_SET_IDXBXE(int node, int *idxbxe, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbxe = qp->dim->nbxe;
	int *nbue = qp->dim->nbue;
	int *nge = qp->dim->nge;

	int ii;
	for(ii=0; ii<nbxe[node]; ii++)
		qp->idxe[node][nbue[node]+ii] = nbu[node] + idxbxe[ii];

	return;
	}
#endif



#if 0
void TREE_OCP_QP_SET_IDXGE(int node, int *idxge, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;
	int *nbxe = qp->dim->nbxe;
	int *nbue = qp->dim->nbue;
	int *nge = qp->dim->nge;

	int ii;
	for(ii=0; ii<nge[node]; ii++)
		qp->idxe[node][nbue[node]+nbxe[node]+ii] = nbu[node] + nbx[node] + idxge[ii];

	return;
	}
#endif



#if 0
void TREE_OCP_QP_SET_JBUE(int node, REAL *Jbue, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbue = qp->dim->nbue;

	int ii, idx;
	idx = 0;
	for(ii=0; ii<nbu[node]; ii++)
		{
		if(idx<nbue[node] | Jbue[ii]!=0.0)
			{
			qp->idxe[node][idx] = ii;
			idx++;
			}
		}
	return;
	}
#endif



#if 0
void TREE_OCP_QP_SET_JBXE(int node, REAL *Jbxe, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;
	int *nbue = qp->dim->nbue;
	int *nbxe = qp->dim->nbxe;

	int ii, idx;
	idx = 0;
	for(ii=0; ii<nbx[node]; ii++)
		{
		if(idx<nbxe[node] | Jbxe[ii]!=0.0)
			{
			qp->idxe[node][nbue[node]+idx] = nbu[node] + ii;
			idx++;
			}
		}
	return;
	}
#endif



#if 0
void TREE_OCP_QP_SET_JGE(int node, REAL *Jge, struct TREE_OCP_QP *qp)
	{
	// extract dim
	int *nbu = qp->dim->nbu;
	int *nbx = qp->dim->nbx;
	int *ng = qp->dim->ng;
	int *nbue = qp->dim->nbue;
	int *nbxe = qp->dim->nbxe;
	int *nge = qp->dim->nge;

	int ii, idx;
	idx = 0;
	for(ii=0; ii<ng[node]; ii++)
		{
		if(idx<nge[node] | Jge[ii]!=0.0)
			{
			qp->idxe[node][nbue[node]+nbxe[node]+idx] = nbu[node] + nbx[node] + ii;
			idx++;
			}
		}
	return;
	}
#endif



#if 0
void TREE_OCP_QP_SET_DIAG_H_FLAG(int node, int *value, struct TREE_OCP_QP *qp)
	{
	qp->diag_H_flag[node] = *value;

	return;
	}
#endif




