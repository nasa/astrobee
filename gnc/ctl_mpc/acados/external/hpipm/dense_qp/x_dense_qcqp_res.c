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



hpipm_size_t DENSE_QCQP_RES_MEMSIZE(struct DENSE_QCQP_DIM *dim)
	{

	// loop index
	int ii;

	// extract ocp qp size
	int nv = dim->nv;
	int ne = dim->ne;
	int nb = dim->nb;
	int ng = dim->ng;
	int nq = dim->nq;
	int ns = dim->ns;

	hpipm_size_t size = 0;

	size += 4*sizeof(struct STRVEC); // res_g res_b res_d res_m

	size += 1*SIZE_STRVEC(nv+2*ns); // res_g
	size += 1*SIZE_STRVEC(ne); // res_b
	size += 2*SIZE_STRVEC(2*nb+2*ng+2*nq+2*ns); // res_d res_m

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void DENSE_QCQP_RES_CREATE(struct DENSE_QCQP_DIM *dim, struct DENSE_QCQP_RES *res, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = DENSE_QCQP_RES_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// extract ocp qp size
	int nv = dim->nv;
	int ne = dim->ne;
	int nb = dim->nb;
	int ng = dim->ng;
	int nq = dim->nq;
	int ns = dim->ns;


	// vector struct
	struct STRVEC *sv_ptr = (struct STRVEC *) mem;

	res->res_g = sv_ptr;
	sv_ptr += 1;
	res->res_b = sv_ptr;
	sv_ptr += 1;
	res->res_d = sv_ptr;
	sv_ptr += 1;
	res->res_m = sv_ptr;
	sv_ptr += 1;


	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) sv_ptr;
	s_ptr = (s_ptr+63)/64*64;


	// void stuf
	char *c_ptr = (char *) s_ptr;

	CREATE_STRVEC(nv+2*ns, res->res_g, c_ptr);
	c_ptr += (res->res_g)->memsize;

	CREATE_STRVEC(ne, res->res_b, c_ptr);
	c_ptr += (res->res_b)->memsize;

	CREATE_STRVEC(2*nb+2*ng+2*nq+2*ns, res->res_d, c_ptr);
	c_ptr += (res->res_d)->memsize;

	CREATE_STRVEC(2*nb+2*ng+2*nq+2*ns, res->res_m, c_ptr);
	c_ptr += (res->res_m)->memsize;

	res->dim = dim;

	res->memsize = DENSE_QCQP_RES_MEMSIZE(dim);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + res->memsize)
		{
		printf("\ncreate_dense_qcpp_res: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



hpipm_size_t DENSE_QCQP_RES_WS_MEMSIZE(struct DENSE_QCQP_DIM *dim)
	{

	// loop index
	int ii;

	// extract ocp qp size
	int nv = dim->nv;
	int ne = dim->ne;
	int nb = dim->nb;
	int ng = dim->ng;
	int nq = dim->nq;
	int ns = dim->ns;

	hpipm_size_t size = 0;

	size += 7*sizeof(struct STRVEC); // 2*tmp_nv 2*tmp_nbgq tmp_ns q_fun q_adj

	size += 3*SIZE_STRVEC(nv); // 2*tmp_nv q_adj
	size += 2*SIZE_STRVEC(nb+ng+nq); // 2*tmp_nbgq
	size += 1*SIZE_STRVEC(ns); // tmp_ns
	size += 1*SIZE_STRVEC(nq); // q_fun

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void DENSE_QCQP_RES_WS_CREATE(struct DENSE_QCQP_DIM *dim, struct DENSE_QCQP_RES_WS *ws, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = DENSE_QCQP_RES_WS_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// extract ocp qp size
	int nv = dim->nv;
	int ne = dim->ne;
	int nb = dim->nb;
	int ng = dim->ng;
	int nq = dim->nq;
	int ns = dim->ns;


	// vector struct
	struct STRVEC *sv_ptr = (struct STRVEC *) mem;

	ws->tmp_nv = sv_ptr;
	sv_ptr += 2;
	ws->tmp_nbgq = sv_ptr;
	sv_ptr += 2;
	ws->tmp_ns = sv_ptr;
	sv_ptr += 1;
	ws->q_fun = sv_ptr;
	sv_ptr += 1;
	ws->q_adj = sv_ptr;
	sv_ptr += 1;


	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) sv_ptr;
	s_ptr = (s_ptr+63)/64*64;


	// void stuf
	char *c_ptr = (char *) s_ptr;


	CREATE_STRVEC(nv, ws->tmp_nv+0, c_ptr);
	c_ptr += (ws->tmp_nv+0)->memsize;

	CREATE_STRVEC(nv, ws->tmp_nv+1, c_ptr);
	c_ptr += (ws->tmp_nv+1)->memsize;

	CREATE_STRVEC(nb+ng+nq, ws->tmp_nbgq+0, c_ptr);
	c_ptr += (ws->tmp_nbgq+0)->memsize;

	CREATE_STRVEC(nb+ng+nq, ws->tmp_nbgq+1, c_ptr);
	c_ptr += (ws->tmp_nbgq+1)->memsize;

	CREATE_STRVEC(ns, ws->tmp_ns+0, c_ptr);
	c_ptr += (ws->tmp_ns+0)->memsize;

	CREATE_STRVEC(nq, ws->q_fun, c_ptr);
	c_ptr += (ws->q_fun)->memsize;

	CREATE_STRVEC(nv, ws->q_adj, c_ptr);
	c_ptr += (ws->q_adj)->memsize;

	ws->use_q_fun = 0;
	ws->use_q_adj = 0;

	ws->memsize = DENSE_QCQP_RES_WS_MEMSIZE(dim);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + ws->memsize)
		{
		printf("\ncreate_dense_qp_res_workspace: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void DENSE_QCQP_RES_COMPUTE(struct DENSE_QCQP *qp, struct DENSE_QCQP_SOL *qp_sol, struct DENSE_QCQP_RES *res, struct DENSE_QCQP_RES_WS *ws)
	{

	int ii, idx;

	int nv = qp->dim->nv;
	int ne = qp->dim->ne;
	int nb = qp->dim->nb;
	int ng = qp->dim->ng;
	int nq = qp->dim->nq;
	int ns = qp->dim->ns;

	int nvt = nv+2*ns;
	int net = ne;
	int nct = 2*nb+2*ng+2*nq+2*ns;

	REAL nct_inv = 1.0/nct;

	struct STRMAT *Hg = qp->Hv;
	struct STRMAT *A = qp->A;
	struct STRMAT *Ct = qp->Ct;
	struct STRMAT *Hq = qp->Hq;
	struct STRVEC *gz = qp->gz;
	struct STRVEC *b = qp->b;
	struct STRVEC *d = qp->d;
//	struct STRVEC *gq = qp->gq;
	struct STRVEC *m = qp->m;
	int *idxb = qp->idxb;
	struct STRVEC *Z = qp->Z;
	int *idxs_rev = qp->idxs_rev;

	struct STRVEC *v = qp_sol->v;
	struct STRVEC *pi = qp_sol->pi;
	struct STRVEC *lam = qp_sol->lam;
	struct STRVEC *t = qp_sol->t;

	struct STRVEC *res_g = res->res_g;
	struct STRVEC *res_b = res->res_b;
	struct STRVEC *res_d = res->res_d;
	struct STRVEC *res_m = res->res_m;

	struct STRVEC *tmp_nv = ws->tmp_nv;
	struct STRVEC *tmp_nbgq = ws->tmp_nbgq;
	struct STRVEC *tmp_ns = ws->tmp_ns;

	REAL mu, tmp;

	// res g
	SYMV_L(nv, 1.0, Hg, 0, 0, v, 0, 1.0, gz, 0, res_g, 0);

	if(nb+ng+nq>0)
		{
		AXPY(nb+ng+nq, -1.0, lam, 0, lam, nb+ng+nq, tmp_nbgq+0, 0);
//		AXPY(nb+ng,  1.0, d, 0, t, 0, res_d, 0);
//		AXPY(nb+ng,  1.0, d, nb+ng, t, nb+ng, res_d, nb+ng);
		AXPY(2*nb+2*ng+2*nq,  1.0, d, 0, t, 0, res_d, 0);
		// box
		if(nb>0)
			{
			VECAD_SP(nb, 1.0, tmp_nbgq+0, 0, idxb, res_g, 0);
			VECEX_SP(nb, 1.0, idxb, v, 0, tmp_nbgq+1, 0);
			}
		// general
		if(ng>0)
			{
			GEMV_NT(nv, ng, 1.0, 1.0, Ct, 0, 0, tmp_nbgq+0, nb, v, 0, 1.0, 0.0, res_g, 0, tmp_nbgq+1, nb, res_g, 0, tmp_nbgq+1, nb);
			}
		// quadratic
		if(nq>0)
			{
//			AXPY(nq,  1.0, d, 2*nb+2*ng+2*ns, t, 2*nb+2*ng+2*ns, res_d, 2*nb+2*ng+2*ns);
			if(ws->use_q_fun & ws->use_q_adj)
				{
				VECCP(nq, ws->q_fun, 0, tmp_nbgq+1, nb+ng);
				AXPY(nv, 1.0, ws->q_adj, 0, res_g, 0, res_g, 0);
				}
			else
				{
				for(ii=0; ii<nq; ii++)
					{
					SYMV_L(nv, 1.0, Hq+ii, 0, 0, v, 0, 0.0, tmp_nv+0, 0, tmp_nv+0, 0);
#ifdef DOUBLE_PRECISION
					tmp = BLASFEO_DVECEL(tmp_nbgq+0, nb+ng+ii);
#else
					tmp = BLASFEO_SVECEL(tmp_nbgq+0, nb+ng+ii);
#endif
					AXPY(nv, tmp, tmp_nv+0, 0, res_g, 0, res_g, 0);
					COLEX(nv, Ct, 0, ng+ii, tmp_nv+1, 0);
					AXPY(nv, tmp, tmp_nv+1, 0, res_g, 0, res_g, 0);
					AXPY(nv, 0.5, tmp_nv+0, 0, tmp_nv+1, 0, tmp_nv+0, 0);
					tmp = DOT(nv, tmp_nv+0, 0, v, 0);
#ifdef DOUBLE_PRECISION
					BLASFEO_DVECEL(tmp_nbgq+1, nb+ng+ii) = tmp;
#else
					BLASFEO_SVECEL(tmp_nbgq+1, nb+ng+ii) = tmp;
#endif
					}
				}
			}
		AXPY(nb+ng+nq, -1.0, tmp_nbgq+1, 0, res_d, 0, res_d, 0);
		AXPY(nb+ng+nq,  1.0, tmp_nbgq+1, 0, res_d, nb+ng+nq, res_d, nb+ng+nq);
		}
	if(ns>0)
		{
		// res_g
		GEMV_DIAG(2*ns, 1.0, Z, 0, v, nv, 1.0, gz, nv, res_g, nv);
		AXPY(2*ns, -1.0, lam, 2*nb+2*ng+2*nq, res_g, nv, res_g, nv);
		for(ii=0; ii<nb+ng+nq; ii++)
			{
			idx = idxs_rev[ii];
			if(idx!=-1)
				{
#ifdef DOUBLE_PRECISION
				BLASFEO_DVECEL(res_g, nv+idx) -= BLASFEO_DVECEL(lam, ii);
				BLASFEO_DVECEL(res_g, nv+ns+idx) -= BLASFEO_DVECEL(lam, nb+ng+nq+ii);
				// res_d
				BLASFEO_DVECEL(res_d, ii) -= BLASFEO_DVECEL(v+ii, nv+idx);
				BLASFEO_DVECEL(res_d, nb+ng+nq+ii) -= BLASFEO_DVECEL(v, nv+ns+idx);
#else
				BLASFEO_SVECEL(res_g, nv+idx) -= BLASFEO_SVECEL(lam, ii);
				BLASFEO_SVECEL(res_g, nv+ns+idx) -= BLASFEO_SVECEL(lam, nb+ng+nq+ii);
				// res_d
				BLASFEO_SVECEL(res_d, ii) -= BLASFEO_SVECEL(v+ii, nv+idx);
				BLASFEO_SVECEL(res_d, nb+ng+nq+ii) -= BLASFEO_SVECEL(v, nv+ns+idx);
#endif
				}
			}
		// res_d
		AXPY(2*ns, -1.0, v, nv, t, 2*nb+2*ng+2*nq, res_d, 2*nb+2*ng+2*nq);
		AXPY(2*ns, 1.0, d, 2*nb+2*ng+2*nq, res_d, 2*nb+2*ng+2*nq, res_d, 2*nb+2*ng+2*nq);
		}
	
	// res b, res g
	if(ne>0)
		GEMV_NT(ne, nv, -1.0, -1.0, A, 0, 0, v, 0, pi, 0, 1.0, 1.0, b, 0, res_g, 0, res_b, 0, res_g, 0);

	// res_m res_mu
	mu = VECMULDOT(nct, lam, 0, t, 0, res_m, 0);
	AXPY(nct, -1.0, m, 0, res_m, 0, res_m, 0);
	res->res_mu = mu*nct_inv;

	return;

	}



void DENSE_QCQP_RES_COMPUTE_INF_NORM(struct DENSE_QCQP_RES *res)
	{

	int nv = res->dim->nv;
	int ne = res->dim->ne;
	int nb = res->dim->nb;
	int ng = res->dim->ng;
	int nq = res->dim->nq;
	int ns = res->dim->ns;

	int nvt = nv+2*ns;
	int net = ne;
	int nct = 2*nb+2*ng+2*nq+2*ns;

	struct STRVEC *res_g = res->res_g;
	struct STRVEC *res_b = res->res_b;
	struct STRVEC *res_d = res->res_d;
	struct STRVEC *res_m = res->res_m;

	// compute infinity norm
	VECNRM_INF(nvt, res_g, 0, res->res_max+0);
	VECNRM_INF(net, res_b, 0, res->res_max+1);
	VECNRM_INF(nct, res_d, 0, res->res_max+2);
	VECNRM_INF(nct, res_m, 0, res->res_max+3);

	return;

	}




