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



hpipm_size_t DENSE_QCQP_MEMSIZE(struct DENSE_QCQP_DIM *dim)
	{

	int nv = dim->nv;
	int ne = dim->ne;
	int nb = dim->nb;
	int ng = dim->ng;
	int nq = dim->nq;
	int ns = dim->ns;

	hpipm_size_t size = 0;

	size += 6*sizeof(struct STRVEC); // gz b d m Z d_mask
	size += (nq+3)*sizeof(struct STRMAT); // Hv A Ct Hq

	size += 1*SIZE_STRVEC(nv+2*ns); // g
	size += 1*SIZE_STRVEC(ne); // b
	size += 3*SIZE_STRVEC(2*nb+2*ng+2*nq+2*ns); // d m d_mask
	size += 1*SIZE_STRVEC(2*ns); // Z
	size += 1*nb*sizeof(int); // idxb
	size += 1*(nb+ng+nq)*sizeof(int); // idxs_rev
	size += 1*nq*sizeof(int); // Hq_nzero

	size += 1*SIZE_STRMAT(nv+1, nv); // Hv
	size += nq*SIZE_STRMAT(nv+1, nv); // Hq
	size += 1*SIZE_STRMAT(ne, nv); // A
	size += 1*SIZE_STRMAT(nv, ng+nq); // Ct

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void DENSE_QCQP_CREATE(struct DENSE_QCQP_DIM *dim, struct DENSE_QCQP *qp, void *mem)
	{

	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = DENSE_QCQP_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// extract dim
	int nv = dim->nv;
	int ne = dim->ne;
	int nb = dim->nb;
	int ng = dim->ng;
	int nq = dim->nq;
	int ns = dim->ns;


	// matrix struct stuff
	struct STRMAT *sm_ptr = (struct STRMAT *) mem;

	qp->Hv = sm_ptr;
	sm_ptr += 1;

	qp->A = sm_ptr;
	sm_ptr += 1;

	qp->Ct = sm_ptr;
	sm_ptr += 1;

	qp->Hq = sm_ptr;
	sm_ptr += nq;


	// vector struct stuff
	struct STRVEC *sv_ptr = (struct STRVEC *) sm_ptr;

	qp->gz = sv_ptr;
	sv_ptr += 1;

	qp->b = sv_ptr;
	sv_ptr += 1;

	qp->d = sv_ptr;
	sv_ptr += 1;

	qp->d_mask = sv_ptr;
	sv_ptr += 1;

	qp->m = sv_ptr;
	sv_ptr += 1;

	qp->Z = sv_ptr;
	sv_ptr += 1;


	// int stuff
	int *i_ptr;
	i_ptr = (int *) sv_ptr;

	// idxb
	qp->idxb = i_ptr;
	i_ptr += nb;

	// idxs_rev
	qp->idxs_rev = i_ptr;
	i_ptr += nb+ng+nq;
	for(ii=0; ii<nb+ng+nq; ii++)
		qp->idxs_rev[ii] = -1;

	// Hq_nzero
	qp->Hq_nzero = i_ptr;
	i_ptr += nq;


	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) i_ptr;
	s_ptr = (s_ptr+63)/64*64;


	//  stuff
	char *c_ptr;
	c_ptr = (char *) s_ptr;

	CREATE_STRMAT(nv+1, nv, qp->Hv, c_ptr);
	c_ptr += qp->Hv->memsize;

	CREATE_STRMAT(ne, nv, qp->A, c_ptr);
	c_ptr += qp->A->memsize;

	CREATE_STRMAT(nv, ng+nq, qp->Ct, c_ptr);
	c_ptr += qp->Ct->memsize;

	for(ii=0; ii<nq; ii++)
		{
		CREATE_STRMAT(nv+1, nv, qp->Hq+ii, c_ptr);
		c_ptr += (qp->Hq+ii)->memsize;
		}

	CREATE_STRVEC(nv+2*ns, qp->gz, c_ptr);
	c_ptr += qp->gz->memsize;

	CREATE_STRVEC(ne, qp->b, c_ptr);
	c_ptr += qp->b->memsize;

	CREATE_STRVEC(2*nb+2*ng+2*nq+2*ns, qp->d, c_ptr);
	c_ptr += qp->d->memsize;

	CREATE_STRVEC(2*nb+2*ng+2*nq+2*ns, qp->d_mask, c_ptr);
	c_ptr += qp->d_mask->memsize;

	CREATE_STRVEC(2*nb+2*ng+2*nq+2*ns, qp->m, c_ptr);
	c_ptr += qp->m->memsize;

	CREATE_STRVEC(2*ns, qp->Z, c_ptr);
	c_ptr += qp->Z->memsize;


	// default initialization
	VECSE(2*nb+2*ng+2*nq+2*ns, 1.0, qp->d_mask, 0);
	// disregard lower quadr constr
	VECSE(nq, 0.0, qp->d_mask, nb+ng);
	//
	for(ii=0; ii<nq; ii++)
		qp->Hq_nzero[ii] = 0;


	qp->dim = dim;

	qp->memsize = DENSE_QCQP_MEMSIZE(dim);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + qp->memsize)
		{
		printf("\ndense_qcqp_create: outside memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void DENSE_QCQP_SET(char *field, void *value, struct DENSE_QCQP *qp)
	{
	REAL *r_ptr;
	int *i_ptr;
    
	// matrices
	if(hpipm_strcmp(field, "H")) 
		{
		DENSE_QCQP_SET_H(value, qp);
		}
	else if(hpipm_strcmp(field, "A")) 
		{
		DENSE_QCQP_SET_A(value, qp);
		}
	else if(hpipm_strcmp(field, "C")) 
		{
		DENSE_QCQP_SET_C(value, qp);
		}
	else if(hpipm_strcmp(field, "HQ")) 
		{
		DENSE_QCQP_SET_HQ(value, qp);
		}
	// vectors
	else if(hpipm_strcmp(field, "g")) 
		{
		DENSE_QCQP_SET_G(value, qp);
		}
	else if(hpipm_strcmp(field, "b")) 
		{
		DENSE_QCQP_SET_B(value, qp);
		}
	else if(hpipm_strcmp(field, "lb")) 
		{
		DENSE_QCQP_SET_LB(value, qp);
		}
	else if(hpipm_strcmp(field, "lb_mask")) 
		{
		DENSE_QCQP_SET_LB_MASK(value, qp);
		}
	else if(hpipm_strcmp(field, "ub")) 
		{
		DENSE_QCQP_SET_UB(value, qp);
		}
	else if(hpipm_strcmp(field, "ub_mask")) 
		{
		DENSE_QCQP_SET_UB_MASK(value, qp);
		}
	else if(hpipm_strcmp(field, "lg")) 
		{
		DENSE_QCQP_SET_LG(value, qp);
		}
	else if(hpipm_strcmp(field, "lg_mask")) 
		{
		DENSE_QCQP_SET_LG_MASK(value, qp);
		}
	else if(hpipm_strcmp(field, "ug")) 
		{
		DENSE_QCQP_SET_UG(value, qp);
		}
	else if(hpipm_strcmp(field, "ug_mask")) 
		{
		DENSE_QCQP_SET_UG_MASK(value, qp);
		}
	else if(hpipm_strcmp(field, "gq")) 
		{
		DENSE_QCQP_SET_GQ(value, qp);
		}
	else if(hpipm_strcmp(field, "uq")) 
		{
		DENSE_QCQP_SET_UQ(value, qp);
		}
	else if(hpipm_strcmp(field, "uq_mask")) 
		{
		DENSE_QCQP_SET_UQ_MASK(value, qp);
		}
	else if(hpipm_strcmp(field, "Zl")) 
		{
		DENSE_QCQP_SET_ZZL(value, qp);
		}
	else if(hpipm_strcmp(field, "Zu")) 
		{
		DENSE_QCQP_SET_ZZU(value, qp);
		}
	else if(hpipm_strcmp(field, "zl")) 
		{
		DENSE_QCQP_SET_ZL(value, qp);
		}
	else if(hpipm_strcmp(field, "zu")) 
		{
		DENSE_QCQP_SET_ZU(value, qp);
		}
	else if(hpipm_strcmp(field, "lls")) 
		{
		DENSE_QCQP_SET_LS(value, qp);
		}
	else if(hpipm_strcmp(field, "lls_mask")) 
		{
		DENSE_QCQP_SET_LS_MASK(value, qp);
		}
	else if(hpipm_strcmp(field, "lus")) 
		{
		DENSE_QCQP_SET_US(value, qp);
		}
	else if(hpipm_strcmp(field, "lus_mask")) 
		{
		DENSE_QCQP_SET_US_MASK(value, qp);
		}
	// int
	else if(hpipm_strcmp(field, "idxb"))
		{
		DENSE_QCQP_SET_IDXB(value, qp);
		}
	else if(hpipm_strcmp(field, "idxs"))
		{
		DENSE_QCQP_SET_IDXS(value, qp);
		}
	else if(hpipm_strcmp(field, "idxs_rev"))
		{
		DENSE_QCQP_SET_IDXS_REV(value, qp);
		}
	else
		{
		printf("error: DENSE_QCQP_SET: wrong field name '%s'. Exiting.\n", field);
		exit(1);	
		}
	return;
	}



void DENSE_QCQP_SET_H(REAL *H, struct DENSE_QCQP *qp)
	{

	int nv = qp->dim->nv;

	CVT_MAT2STRMAT(nv, nv, H, nv, qp->Hv, 0, 0);

	return;

	}



void DENSE_QCQP_SET_G(REAL *g, struct DENSE_QCQP *qp)
	{

	int nv = qp->dim->nv;

	PACK_VEC(nv, g, 1, qp->gz, 0);

	return;

	}



void DENSE_QCQP_SET_A(REAL *A, struct DENSE_QCQP *qp)
	{

	int nv = qp->dim->nv;
	int ne = qp->dim->ne;

	CVT_MAT2STRMAT(ne, nv, A, ne, qp->A, 0, 0);

	return;

	}



void DENSE_QCQP_SET_B(REAL *b, struct DENSE_QCQP *qp)
	{

	int ne = qp->dim->ne;

	PACK_VEC(ne, b, 1, qp->b, 0);

	return;

	}



void DENSE_QCQP_SET_IDXB(int *idxb, struct DENSE_QCQP *qp)
	{

	int ii;
	int nb = qp->dim->nb;

	for(ii=0; ii<nb; ii++) qp->idxb[ii] = idxb[ii];

	return;

	}



void DENSE_QCQP_SET_LB(REAL *lb, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;

	PACK_VEC(nb, lb, 1, qp->d, 0);
	VECSE(nb, 0.0, qp->m, 0);

	return;

	}



void DENSE_QCQP_SET_LB_MASK(REAL *lb_mask, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;

	int ii;

	for(ii=0; ii<nb; ii++)
		if(lb_mask[ii]==0.0)
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qp->d_mask, ii) = 0;
		else
			BLASFEO_DVECEL(qp->d_mask, ii) = 1;
#else
			BLASFEO_SVECEL(qp->d_mask, ii) = 0;
		else
			BLASFEO_SVECEL(qp->d_mask, ii) = 1;
#endif

	return;

	}



void DENSE_QCQP_SET_UB(REAL *ub, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	PACK_VEC(nb, ub, 1, qp->d, nb+ng+nq);
	VECSC(nb, -1.0, qp->d, nb+ng+nq);
	VECSE(nb, 0.0, qp->m, nb+ng+nq);

	return;

	}



void DENSE_QCQP_SET_UB_MASK(REAL *ub_mask, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	int ii;

	for(ii=0; ii<nb; ii++)
		if(ub_mask[ii]==0.0)
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qp->d_mask, nb+ng+nq+ii) = 0;
		else
			BLASFEO_DVECEL(qp->d_mask, nb+ng+nq+ii) = 1;
#else
			BLASFEO_SVECEL(qp->d_mask, nb+ng+nq+ii) = 0;
		else
			BLASFEO_SVECEL(qp->d_mask, nb+ng+nq+ii) = 1;
#endif

	return;

	}



void DENSE_QCQP_SET_C(REAL *C, struct DENSE_QCQP *qp)
	{

	int nv = qp->dim->nv;
	int ng = qp->dim->ng;

	CVT_TRAN_MAT2STRMAT(ng, nv, C, ng, qp->Ct, 0, 0);

	return;

	}



void DENSE_QCQP_SET_LG(REAL *lg, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;

	PACK_VEC(ng, lg, 1, qp->d, nb);
	VECSE(ng, 0.0, qp->m, nb);

	return;

	}



void DENSE_QCQP_SET_LG_MASK(REAL *lg_mask, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;

	int ii;

	for(ii=0; ii<ng; ii++)
		if(lg_mask[ii]==0.0)
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qp->d_mask, nb+ii) = 0;
		else
			BLASFEO_DVECEL(qp->d_mask, nb+ii) = 1;
#else
			BLASFEO_SVECEL(qp->d_mask, nb+ii) = 0;
		else
			BLASFEO_SVECEL(qp->d_mask, nb+ii) = 1;
#endif

	return;

	}



void DENSE_QCQP_SET_UG(REAL *ug, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	PACK_VEC(ng, ug, 1, qp->d, 2*nb+ng+nq);
	VECSC(ng, -1.0, qp->d, 2*nb+ng+nq);
	VECSE(ng, 0.0, qp->m, 2*nb+ng+nq);

	return;

	}



void DENSE_QCQP_SET_UG_MASK(REAL *ug_mask, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	int ii;

	for(ii=0; ii<ng; ii++)
		if(ug_mask[ii]==0.0)
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qp->d_mask, 2*nb+ng+nq+ii) = 0;
		else
			BLASFEO_DVECEL(qp->d_mask, 2*nb+ng+nq+ii) = 1;
#else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+ng+nq+ii) = 0;
		else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+ng+nq+ii) = 1;
#endif

	return;

	}



void DENSE_QCQP_SET_HQ(REAL *HQ, struct DENSE_QCQP *qp)
	{

	int ii, jj;

	int nv = qp->dim->nv;
	int nq = qp->dim->nq;

	int nzero = 0;

	for(ii=0; ii<nq; ii++)
		{
		for(jj=0; jj<nv*nv; jj++)
			{
			if(HQ[ii*nv*nv+jj]!=0.0)
				{
				nzero = 1;
				break;
				}
			}
		CVT_MAT2STRMAT(nv, nv, HQ+ii*nv*nv, nv, qp->Hq+ii, 0, 0);
		if(nzero)
			{
			qp->Hq_nzero[ii] = 1;
			}
		}

	return;

	}



void DENSE_QCQP_SET_GQ(REAL *gq, struct DENSE_QCQP *qp)
	{

	int nv = qp->dim->nv;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	CVT_MAT2STRMAT(nv, nq, gq, nv, qp->Ct, 0, ng);

	return;

	}



void DENSE_QCQP_SET_UQ(REAL *uq, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	PACK_VEC(nq, uq, 1, qp->d, 2*nb+2*ng+nq);
	VECSC(nq, -1.0, qp->d, 2*nb+2*ng+nq);
	VECSE(nq, 0.0, qp->m, 2*nb+2*ng+nq);

	// XXX lower quard constr is disregarded
	REAL inf = 1e3;
	VECSE(nq, -inf, qp->d, nb+ng);
	VECSE(nq, 0.0, qp->m, nb+ng);

	return;

	}



void DENSE_QCQP_SET_UQ_MASK(REAL *uq_mask, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	int ii;

	for(ii=0; ii<nq; ii++)
		if(uq_mask[ii]==0.0)
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qp->d_mask, 2*nb+2*ng+nq+ii) = 0;
		else
			BLASFEO_DVECEL(qp->d_mask, 2*nb+2*ng+nq+ii) = 1;
#else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+2*ng+nq+ii) = 0;
		else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+2*ng+nq+ii) = 1;
#endif

	return;

	}



void DENSE_QCQP_SET_IDXS(int *idxs, struct DENSE_QCQP *qp)
	{

	int ii;
	int ns = qp->dim->ns;

	for(ii=0; ii<ns; ii++)
		{
		qp->idxs_rev[idxs[ii]] = ii;
		}

	return;

	}



void DENSE_QCQP_SET_IDXS_REV(int *idxs_rev, struct DENSE_QCQP *qp)
	{

	int ii;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	for(ii=0; ii<nb+ng+nq; ii++)
		{
		qp->idxs_rev[ii] = idxs_rev[ii];
		}

	return;

	}



void DENSE_QCQP_SET_ZZL(REAL *Zl, struct DENSE_QCQP *qp)
	{

	int ns = qp->dim->ns;

	PACK_VEC(ns, Zl, 1, qp->Z, 0);

	return;

	}



void DENSE_QCQP_SET_ZZU(REAL *Zu, struct DENSE_QCQP *qp)
	{

	int ns = qp->dim->ns;

	PACK_VEC(ns, Zu, 1, qp->Z, ns);

	return;

	}




void DENSE_QCQP_SET_ZL(REAL *zl, struct DENSE_QCQP *qp)
	{

	int ns = qp->dim->ns;
	int nv = qp->dim->nv;

	PACK_VEC(ns, zl, 1, qp->gz, nv);

	return;

	}



void DENSE_QCQP_SET_ZU(REAL *zu, struct DENSE_QCQP *qp)
	{

	int ns = qp->dim->ns;
	int nv = qp->dim->nv;

	PACK_VEC(ns, zu, 1, qp->gz, nv+ns);

	return;

	}


void DENSE_QCQP_SET_LS(REAL *ls, struct DENSE_QCQP *qp)
	{

	int ns = qp->dim->ns;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	PACK_VEC(ns, ls, 1, qp->d, 2*nb+2*ng+2*nq);
	VECSE(ns, 0.0, qp->m, 2*nb+2*ng+2*nq);

	return;

	}



void DENSE_QCQP_SET_LS_MASK(REAL *ls_mask, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;
	int ns = qp->dim->ns;

	int ii;

	for(ii=0; ii<ns; ii++)
		if(ls_mask[ii]==0.0)
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ii) = 0;
		else
			BLASFEO_DVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ii) = 1;
#else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ii) = 0;
		else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ii) = 1;
#endif

	return;

	}



void DENSE_QCQP_SET_US(REAL *us, struct DENSE_QCQP *qp)
	{

	int ns = qp->dim->ns;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	PACK_VEC(ns, us, 1, qp->d, 2*nb+2*ng+2*nq+ns);
	VECSE(ns, 0.0, qp->m, 2*nb+2*ng+2*nq+ns);

	return;

	}



void DENSE_QCQP_SET_US_MASK(REAL *us_mask, struct DENSE_QCQP *qp)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;
	int ns = qp->dim->ns;

	int ii;

	for(ii=0; ii<ns; ii++)
		if(us_mask[ii]==0.0)
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ns+ii) = 0;
		else
			BLASFEO_DVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ns+ii) = 1;
#else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ns+ii) = 0;
		else
			BLASFEO_SVECEL(qp->d_mask, 2*nb+2*ng+2*nq+ns+ii) = 1;
#endif

	return;

	}



void DENSE_QCQP_GET_H(struct DENSE_QCQP *qp, REAL *H)
	{

	int nv = qp->dim->nv;

	CVT_STRMAT2MAT(nv, nv, qp->Hv, 0, 0, H, nv);

	return;

	}



void DENSE_QCQP_GET_G(struct DENSE_QCQP *qp, REAL *g)
	{

	int nv = qp->dim->nv;

	UNPACK_VEC(nv, qp->gz, 0, g, 1);

	return;

	}



void DENSE_QCQP_GET_A(struct DENSE_QCQP *qp, REAL *A)
	{

	int nv = qp->dim->nv;
	int ne = qp->dim->ne;

	CVT_STRMAT2MAT(ne, nv, qp->A, 0, 0, A, ne);

	return;

	}



void DENSE_QCQP_GET_B(struct DENSE_QCQP *qp, REAL *b)
	{

	int ne = qp->dim->ne;

	UNPACK_VEC(ne, qp->b, 0, b, 1);

	return;

	}



void DENSE_QCQP_GET_IDXB(struct DENSE_QCQP *qp, int *idxb)
	{

	int ii;
	int nb = qp->dim->nb;

	for(ii=0; ii<nb; ii++) idxb[ii] = qp->idxb[ii];

	return;

	}



void DENSE_QCQP_GET_LB(struct DENSE_QCQP *qp, REAL *lb)
	{

	int nb = qp->dim->nb;

	UNPACK_VEC(nb, qp->d, 0, lb, 1);

	return;

	}



void DENSE_QCQP_GET_LB_MASK(struct DENSE_QCQP *qp, REAL *lb_mask)
	{

	int nb = qp->dim->nb;

	UNPACK_VEC(nb, qp->d_mask, 0, lb_mask, 1);

	return;

	}



void DENSE_QCQP_GET_UB(struct DENSE_QCQP *qp, REAL *ub)
	{

	int ii;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(nb, qp->d, nb+ng+nq, ub, 1);
	for(ii=0; ii<nb; ii++) ub[ii] = - ub[ii];

	return;

	}



void DENSE_QCQP_GET_UB_MASK(struct DENSE_QCQP *qp, REAL *ub_mask)
	{

	int ii;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(nb, qp->d_mask, nb+ng+nq, ub_mask, 1);

	return;

	}



void DENSE_QCQP_GET_C(struct DENSE_QCQP *qp, REAL *C)
	{

	int nv = qp->dim->nv;
	int ng = qp->dim->ng;

	CVT_TRAN_STRMAT2MAT(nv, ng, qp->Ct, 0, 0, C, ng);

	return;

	}



void DENSE_QCQP_GET_LG(struct DENSE_QCQP *qp, REAL *lg)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;

	UNPACK_VEC(ng, qp->d, nb, lg, 1);

	return;

	}



void DENSE_QCQP_GET_LG_MASK(struct DENSE_QCQP *qp, REAL *lg_mask)
	{

	int nb = qp->dim->nb;
	int ng = qp->dim->ng;

	UNPACK_VEC(ng, qp->d_mask, nb, lg_mask, 1);

	return;

	}



void DENSE_QCQP_GET_UG(struct DENSE_QCQP *qp, REAL *ug)
	{

	int ii;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(ng, qp->d, 2*nb+ng+nq, ug, 1);
	for(ii=0; ii<ng; ii++) ug[ii] = - ug[ii];

	return;

	}



void DENSE_QCQP_GET_UG_MASK(struct DENSE_QCQP *qp, REAL *ug_mask)
	{

	int ii;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(ng, qp->d_mask, 2*nb+ng+nq, ug_mask, 1);

	return;

	}



void DENSE_QCQP_GET_IDXS(struct DENSE_QCQP *qp, int *idxs)
	{

	int ii, idx_tmp;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;
	int ns = qp->dim->ns;

	for(ii=0; ii<nb+ng+nq; ii++)
		{
		idx_tmp = qp->idxs_rev[ii];
		if(idx_tmp!=-1)
			{
			idxs[idx_tmp] = ii;
			}
		}

	return;

	}



void DENSE_QCQP_GET_IDXS_REV(struct DENSE_QCQP *qp, int *idxs_rev)
	{

	int ii;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	for(ii=0; ii<nb+ng+nq; ii++)
		{
		idxs_rev[ii] = qp->idxs_rev[ii];
		}

	return;

	}



void DENSE_QCQP_GET_ZZL(struct DENSE_QCQP *qp, REAL *Zl)
	{

	int ns = qp->dim->ns;

	UNPACK_VEC(ns, qp->Z, 0, Zl, 1);

	return;

	}



void DENSE_QCQP_GET_ZZU(struct DENSE_QCQP *qp, REAL *Zu)
	{

	int ns = qp->dim->ns;

	UNPACK_VEC(ns, qp->Z, ns, Zu, 1);

	return;

	}




void DENSE_QCQP_GET_ZL(struct DENSE_QCQP *qp, REAL *zl)
	{

	int ns = qp->dim->ns;
	int nv = qp->dim->nv;

	UNPACK_VEC(ns, qp->gz, nv, zl, 1);

	return;

	}



void DENSE_QCQP_GET_ZU(struct DENSE_QCQP *qp, REAL *zu)
	{

	int ns = qp->dim->ns;
	int nv = qp->dim->nv;

	UNPACK_VEC(ns, qp->gz, nv+ns, zu, 1);

	return;

	}


void DENSE_QCQP_GET_LS(struct DENSE_QCQP *qp, REAL *ls)
	{

	int ns = qp->dim->ns;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(ns, qp->d, 2*nb+2*ng+2*nq, ls, 1);

	return;

	}



void DENSE_QCQP_GET_LS_MASK(struct DENSE_QCQP *qp, REAL *ls_mask)
	{

	int ns = qp->dim->ns;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(ns, qp->d_mask, 2*nb+2*ng+2*nq, ls_mask, 1);

	return;

	}



void DENSE_QCQP_GET_US(struct DENSE_QCQP *qp, REAL *us)
	{

	int ns = qp->dim->ns;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(ns, qp->d, 2*nb+2*ng+2*nq+ns, us, 1);

	return;

	}



void DENSE_QCQP_GET_US_MASK(struct DENSE_QCQP *qp, REAL *us_mask)
	{

	int ns = qp->dim->ns;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;

	UNPACK_VEC(ns, qp->d_mask, 2*nb+2*ng+2*nq+ns, us_mask, 1);

	return;

	}




