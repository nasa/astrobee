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

void COND_QCQP_COMPUTE_DIM(struct OCP_QCQP_DIM *ocp_dim, struct DENSE_QCQP_DIM *dense_dim)
	{

	int N = ocp_dim->N;
	int *nx = ocp_dim->nx;
	int *nu = ocp_dim->nu;
	int *nbx = ocp_dim->nbx;
	int *nbu = ocp_dim->nbu;
	int *ng = ocp_dim->ng;
	int *nq = ocp_dim->nq;
	int *ns = ocp_dim->ns;
	int *nsbx = ocp_dim->nsbx;
	int *nsbu = ocp_dim->nsbu;
	int *nsg = ocp_dim->nsg;
	int *nsq = ocp_dim->nsq;

	int ii;

	int nvc = 0;
	int nec = 0;
	int nbc = 0;
	int ngc = 0;
	int nqc = 0;
	int nsc = 0;
	int nsbc = 0;
	int nsgc = 0;
	int nsqc = 0;

	// first stage
	nvc += nx[0]+nu[0];
	nbc += nbx[0]+nbu[0];
	ngc += ng[0];
	nqc += nq[0];
	nsc += ns[0];
	nsbc += nsbx[0]+nsbu[0];
	nsgc += nsg[0];
	nsqc += nsq[0];
	// remaining stages
	for(ii=1; ii<=N; ii++)
		{
		nvc += nu[ii];
		nbc += nbu[ii];
		ngc += nbx[ii]+ng[ii];
		nqc += nq[ii];
		nsc += ns[ii];
		nsbc += nsbu[ii];
		nsgc += nsbx[ii]+nsg[ii];
		nsqc += nsq[ii];
		}

	// XXX must use setters to correctly set qp ones too !
	DENSE_QCQP_DIM_SET_NV(nvc, dense_dim);
	DENSE_QCQP_DIM_SET_NE(nec, dense_dim);
	DENSE_QCQP_DIM_SET_NB(nbc, dense_dim);
	DENSE_QCQP_DIM_SET_NG(ngc, dense_dim);
	DENSE_QCQP_DIM_SET_NQ(nqc, dense_dim);
	DENSE_QCQP_DIM_SET_NS(nsc, dense_dim);
	DENSE_QCQP_DIM_SET_NSB(nsbc, dense_dim);
	DENSE_QCQP_DIM_SET_NSG(nsgc, dense_dim);
	DENSE_QCQP_DIM_SET_NSQ(nsqc, dense_dim);

	return;

	}



hpipm_size_t COND_QCQP_ARG_MEMSIZE()
	{

	hpipm_size_t size = 0;

	size += 1*sizeof(struct COND_QP_ARG);
	size += 1*COND_QP_ARG_MEMSIZE();

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void COND_QCQP_ARG_CREATE(struct COND_QCQP_ARG *cond_arg, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = COND_QCQP_ARG_MEMSIZE();
	hpipm_zero_memset(memsize, mem);

	struct COND_QP_ARG *arg_ptr = mem;

	cond_arg->qp_arg = arg_ptr;
	arg_ptr += 1;

	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) arg_ptr;
	s_ptr = (s_ptr+63)/64*64;

	// void
	char *c_ptr = (char *) s_ptr;

	COND_QP_ARG_CREATE(cond_arg->qp_arg, c_ptr);
	c_ptr += cond_arg->qp_arg->memsize;


	cond_arg->memsize = COND_QCQP_ARG_MEMSIZE();

#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + cond_arg->memsize)
		{
		printf("\nerror: COND_QCQP_ARG_CREATE: outside memory bounds!\n\n");
		exit(1);
		}
#endif

	return;

	}



void COND_QCQP_ARG_SET_DEFAULT(struct COND_QCQP_ARG *cond_arg)
	{

	cond_arg->cond_last_stage = 1; // condense last stage
	cond_arg->comp_prim_sol = 1; // compute primal solution (v)
	cond_arg->comp_dual_sol_eq = 1; // compute dual solution equality constr (pi)
	cond_arg->comp_dual_sol_ineq = 1; // compute dual solution inequality constr (lam t)
	cond_arg->square_root_alg = 1; // square root algorithm (faster but requires RSQ>0)

	// set arg of qp struct
	cond_arg->qp_arg->cond_last_stage = cond_arg->cond_last_stage;
	cond_arg->qp_arg->comp_prim_sol = cond_arg->comp_prim_sol;
	cond_arg->qp_arg->comp_dual_sol_eq = cond_arg->comp_dual_sol_eq;
	cond_arg->qp_arg->comp_dual_sol_ineq = cond_arg->comp_dual_sol_ineq;
	cond_arg->qp_arg->square_root_alg = cond_arg->square_root_alg;

	return;

	}



void COND_QCQP_ARG_SET_RIC_ALG(int ric_alg, struct COND_QCQP_ARG *cond_arg)
	{

	cond_arg->square_root_alg = ric_alg;

	// set arg of qp struct
	cond_arg->qp_arg->square_root_alg = cond_arg->square_root_alg;

	return;

	}



void COND_QCQP_ARG_SET_COND_LAST_STAGE(int cond_last_stage, struct COND_QCQP_ARG *cond_arg)
	{

	cond_arg->cond_last_stage = cond_last_stage;

	// set arg of qp struct
	cond_arg->qp_arg->cond_last_stage = cond_arg->cond_last_stage;

	return;

	}



hpipm_size_t COND_QCQP_WS_MEMSIZE(struct OCP_QCQP_DIM *ocp_dim, struct COND_QCQP_ARG *cond_arg)
	{

	hpipm_size_t size = 0;

	size += 1*sizeof(struct COND_QP_WS);
	size += 1*COND_QP_WS_MEMSIZE(ocp_dim->qp_dim, cond_arg->qp_arg);

	int ii;

	int N = ocp_dim->N;
	int *nx = ocp_dim->nx;
	int *nu = ocp_dim->nu;
	int *nb = ocp_dim->nb;
	int *ng = ocp_dim->ng;
	int *nq = ocp_dim->nq;

	// compute core qp size and max size
//	int nvt = 0;
//	int net = 0;
//	int nbt = 0;
//	int ngt = 0;
	int nxM = 0;
	int nuM = 0;
//	int nbM = 0;
//	int ngM = 0;

	for(ii=0; ii<N; ii++)
		{
//		nvt += nx[ii]+nu[ii];
//		net += nx[ii+1];
//		nbt += nb[ii];
//		ngt += ng[ii];
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
//		nbM = nb[ii]>nbM ? nb[ii] : nbM;
//		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}
	ii = N;
//	nvt += nx[ii]+nu[ii];
//	nbt += nb[ii];
//	ngt += ng[ii];
	nxM = nx[ii]>nxM ? nx[ii] : nxM;
	nuM = nu[ii]>nuM ? nu[ii] : nuM;
//	nbM = nb[ii]>nbM ? nb[ii] : nbM;
//	ngM = ng[ii]>ngM ? ng[ii] : ngM;

	int nvc = 0;
	nvc += nx[0]+nu[0];
	for(ii=1; ii<=N; ii++)
		nvc += nu[ii];

	size += 3*(N+1)*sizeof(struct STRMAT); // hess_array GammaQ tmp_DCt
	size += 1*sizeof(struct STRMAT); // zero_hess
	size += 1*(N+1)*sizeof(struct STRVEC); // grad_array
	size += 3*sizeof(struct STRVEC); // zero_grad tmp_nvc tmp_nuxM

	size += 1*SIZE_STRMAT(nuM+nxM+1, nuM+nxM); // zero_hess
	size += 1*SIZE_STRVEC(nuM+nxM); // zero_grad
	size += 1*SIZE_STRVEC(nvc); // tmp_nvc
	size += 1*SIZE_STRVEC(nuM+nxM); // tmp_nuxM
	int nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nu_tmp += nu[ii];
		size += SIZE_STRMAT(nu_tmp+nx[0]+1, nx[ii+1]); // GammaQ
//		size += SIZE_STRMAT(nu_tmp+nx[0], nx[ii+1]); // GammaQ TODO without the +1 ???
		}
	for(ii=0; ii<=N; ii++)
		{
		size += SIZE_STRMAT(nu[ii]+nx[ii], ng[ii]+nq[ii]);
		}

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void COND_QCQP_WS_CREATE(struct OCP_QCQP_DIM *ocp_dim, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = COND_QCQP_WS_MEMSIZE(ocp_dim, cond_arg);
	hpipm_zero_memset(memsize, mem);

	int N = ocp_dim->N;
	int *nx = ocp_dim->nx;
	int *nu = ocp_dim->nu;
	int *nb = ocp_dim->nb;
	int *ng = ocp_dim->ng;
	int *nq = ocp_dim->nq;

	// compute core qp dim and max dim
//	int nvt = 0;
//	int net = 0;
//	int nbt = 0;
//	int ngt = 0;
	int nxM = 0;
	int nuM = 0;
//	int nbM = 0;
//	int ngM = 0;

	for(ii=0; ii<N; ii++)
		{
//		nvt += nx[ii]+nu[ii];
//		net += nx[ii+1];
//		nbt += nb[ii];
//		ngt += ng[ii];
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
//		nbM = nb[ii]>nbM ? nb[ii] : nbM;
//		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}
	ii = N;
//	nvt += nx[ii]+nu[ii];
//	nbt += nb[ii];
//	ngt += ng[ii];
	nxM = nx[ii]>nxM ? nx[ii] : nxM;
	nuM = nu[ii]>nuM ? nu[ii] : nuM;
//	nbM = nb[ii]>nbM ? nb[ii] : nbM;
//	ngM = ng[ii]>ngM ? ng[ii] : ngM;

	int nvc = 0;
	nvc += nx[0]+nu[0];
	for(ii=1; ii<=N; ii++)
		nvc += nu[ii];


	// cond qp ws struct
	struct COND_QP_WS *ws_ptr = mem;

	cond_ws->qp_ws = ws_ptr;
	ws_ptr += 1;

	// matrix struct
	struct STRMAT *sm_ptr = (struct STRMAT *) ws_ptr;

	cond_ws->hess_array = sm_ptr;
	sm_ptr += N+1;
	cond_ws->zero_hess = sm_ptr;
	sm_ptr += 1;
	cond_ws->GammaQ = sm_ptr;
	sm_ptr += N+1;
	cond_ws->tmp_DCt = sm_ptr;
	sm_ptr += N+1;

	// vector struct
	struct STRVEC *sv_ptr = (struct STRVEC *) sm_ptr;

	cond_ws->grad_array = sv_ptr;
	sv_ptr += N+1;
	cond_ws->zero_grad = sv_ptr;
	sv_ptr += 1;
	cond_ws->tmp_nvc = sv_ptr;
	sv_ptr += 1;
	cond_ws->tmp_nuxM = sv_ptr;
	sv_ptr += 1;

	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) sv_ptr;
	s_ptr = (s_ptr+63)/64*64;

	// void stuf
	char *c_ptr = (char *) s_ptr;
	char *c_tmp;

	//
	int nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nu_tmp += nu[ii];
		CREATE_STRMAT(nu_tmp+nx[0]+1, nx[ii+1], cond_ws->GammaQ+ii, c_ptr);
		c_ptr += (cond_ws->GammaQ+ii)->memsize;
		}
	//
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRMAT(nu[ii]+nx[ii], ng[ii]+nq[ii], cond_ws->tmp_DCt+ii, c_ptr);
		c_ptr += (cond_ws->tmp_DCt+ii)->memsize;
		}
	//
	COND_QP_WS_CREATE(ocp_dim->qp_dim, cond_arg->qp_arg, cond_ws->qp_ws, c_ptr);
	c_ptr += cond_ws->qp_ws->memsize;
	//
	CREATE_STRMAT(nuM+nxM+1, nuM+nxM, cond_ws->zero_hess, c_ptr);
	c_ptr += cond_ws->zero_hess->memsize;
	//
	CREATE_STRVEC(nuM+nxM, cond_ws->zero_grad, c_ptr);
	c_ptr += cond_ws->zero_grad->memsize;
	//
	CREATE_STRVEC(nvc, cond_ws->tmp_nvc, c_ptr);
	c_ptr += cond_ws->tmp_nvc->memsize;
	//
	CREATE_STRVEC(nuM+nxM, cond_ws->tmp_nuxM, c_ptr);
	c_ptr += cond_ws->tmp_nuxM->memsize;

	cond_ws->memsize = memsize; //COND_QCQP_WS_MEMSIZE(ocp_dim, cond_arg);

#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + cond_ws->memsize)
		{
		printf("\nerror: COND_QCQP_WS_CREATE: outside memory bounds!\n\n");
		exit(1);
		}
#endif

	return;

	}



void COND_QCQP_QC(struct OCP_QCQP *ocp_qp, struct STRMAT *Hq2, int *Hq_nzero2, struct STRMAT *Ct2, struct STRVEC *d2, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws)
	{

	// cond quadr constr
	int ii, jj, kk;

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;

	int *nu = ocp_qp->dim->nu;
	int *nx = ocp_qp->dim->nx;
	int *nbu = ocp_qp->dim->nbu;
	int *nbx = ocp_qp->dim->nbx;
	int *ng = ocp_qp->dim->ng;
	int *nq = ocp_qp->dim->nq;

	// early return
	if(N==0)
		{
		for(jj=0; jj<nq[0]; jj++)
			{
			GECP(nu[0]+nx[0], nu[0]+nx[0], ocp_qp->Hq[0]+jj, 0, 0, Hq2+jj, 0, 0);
			}
		return;
		}

	int nvc = nu[0]+nx[0];
	int nbc = nbu[0]+nbx[0];
	int ngc = ng[0];
	int nqc = nq[0];
	for(ii=1; ii<=N; ii++)
		{
		nvc += nu[ii];
		nbc += nbu[ii];
		ngc += nbx[ii]+ng[ii];
		nqc += nq[ii];
		}

	int nu_tmp, nq_tmp, nu_tot_tmp, nq_tot_tmp;

	REAL rho;

	GESE(nvc, nqc, 0.0, Ct2, 0, ngc);

	nu_tmp = 0;
	nq_tmp = 0;
	nu_tot_tmp = nvc - nx[0];
	nq_tot_tmp = nqc;
	for(kk=0; kk<=N; kk++)
		{

		nu_tot_tmp -= nu[kk];
		nq_tot_tmp -= nq[kk];

		nq_tmp = nq_tot_tmp;

		for(jj=0; jj<nq[kk]; jj++)
			{

			GESE(nvc+1, nvc, 0.0, Hq2+nq_tmp, 0, 0);
			Hq_nzero2[nq_tmp] = 0;
			rho = 0.0;

			if(kk==0)
				{
				TRCP_L(nu[0]+nx[0], ocp_qp->Hq[kk]+nq[jj], 0, 0, Hq2+nq_tmp, nu_tot_tmp, nu_tot_tmp);
				if(nx[0]>0)
					{
					if(ocp_qp->Hq_nzero[kk][jj]%2==1) // Q nzero
						Hq_nzero2[nq_tmp] |= 1;
					if((ocp_qp->Hq_nzero[kk][jj]>>1)%2==1) // S nzero
						Hq_nzero2[nq_tmp] |= 2;
					}
				if((ocp_qp->Hq_nzero[kk][jj]>>2)%2==1) // R nzero
					Hq_nzero2[nq_tmp] |= 4;
				}
			else
				{
				// XXX make Q full or use SYMM
				// hessian
				if(ocp_qp->Hq_nzero[kk][jj]%2==1)
					{
//					TRTR_L(nx[kk], ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], ocp_qp->Hq[kk]+jj, nu[kk], nu[kk]); // buggy ???
					TRTR_L(nu[kk]+nx[kk], ocp_qp->Hq[kk]+jj, 0, 0, ocp_qp->Hq[kk]+jj, 0, 0);
					GEMM_NN(nu_tmp+nx[0]+1, nx[kk], nx[kk], 1.0, cond_ws->qp_ws->Gamma+kk-1, 0, 0, ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], 0.0, cond_ws->GammaQ+kk-1, 0, 0, cond_ws->GammaQ+kk-1, 0, 0);
					ROWEX(nx[kk], 1.0, cond_ws->GammaQ+kk-1, nu_tmp+nx[0], 0, cond_ws->tmp_nuxM, 0);
//					SYMV_L(nx[kk], 1.0, ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], cond_ws->qp_ws->Gammab+kk-1, 0, 0.0, cond_ws->tmp_nuxM, 0, cond_ws->tmp_nuxM, 0);
					rho = 0.5*DOT(nx[kk], cond_ws->tmp_nuxM, 0, cond_ws->qp_ws->Gammab+kk-1, 0);
					SYRK_LN_MN(nu_tmp+nx[0]+1, nu_tmp+nx[0], nx[kk], 1.0, cond_ws->qp_ws->Gamma+kk-1, 0, 0, cond_ws->GammaQ+kk-1, 0, 0, 0.0, Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp+nu[kk], Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp+nu[kk]);
					if(nx[0]>0)
						Hq_nzero2[nq_tmp] |= 7;
					else
						Hq_nzero2[nq_tmp] |= 4;
					}
				if((ocp_qp->Hq_nzero[kk][jj]>>2)%2==1)
					{
					GEAD(nu[kk], nu[kk], 1.0, ocp_qp->Hq[kk]+jj, 0, 0, Hq2+nq_tmp, nu_tot_tmp, nu_tot_tmp);
					Hq_nzero2[nq_tmp] |= 4;
					}
				if((ocp_qp->Hq_nzero[kk][jj]>>1)%2==1)
					{
					GEMM_NN(nu_tmp+nx[0]+1, nu[kk], nx[kk], 1.0, cond_ws->qp_ws->Gamma+kk-1, 0, 0, ocp_qp->Hq[kk]+jj, nu[kk], 0, 1.0, Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp, Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp);
					if(nx[0]>0)
						Hq_nzero2[nq_tmp] |= 6;
					else
						Hq_nzero2[nq_tmp] |= 4;
					}
				// gradient
				ROWEX(nu_tmp+nx[0], 1.0, Hq2+nq_tmp, nvc, nu_tot_tmp+nu[kk], cond_ws->tmp_nvc, 0);
				COLAD(nu_tmp+nx[0], 1.0, cond_ws->tmp_nvc, 0, Ct2, nu_tot_tmp+nu[kk], ngc+nq_tmp);
				}

//			printf("\nrho %d %f\n", kk, rho);
#if defined(DOUBLE_PRECISION)
			BLASFEO_DVECEL(d2, nbc+ngc+nq_tmp) -= rho;
			BLASFEO_DVECEL(d2, 2*nbc+2*ngc+nqc+nq_tmp) += rho;
#else
			BLASFEO_SVECEL(d2, nbc+ngc+nq_tmp) -= rho;
			BLASFEO_SVECEL(d2, 2*nbc+2*ngc+nqc+nq_tmp) += rho;
#endif

			nq_tmp++;

			}

		nu_tmp += nu[kk];

		}

	return;

	}



void COND_QCQP_QC_LHS(struct OCP_QCQP *ocp_qp, struct STRMAT *Hq2, int *Hq_nzero2, struct STRMAT *Ct2, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws)
	{

	// cond quadr constr
	int ii, jj, kk;

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;

	int *nu = ocp_qp->dim->nu;
	int *nx = ocp_qp->dim->nx;
	int *nbu = ocp_qp->dim->nbu;
	int *nbx = ocp_qp->dim->nbx;
	int *ng = ocp_qp->dim->ng;
	int *nq = ocp_qp->dim->nq;

	// early return
	if(N==0)
		{
		for(jj=0; jj<nq[0]; jj++)
			{
			GECP(nu[0]+nx[0], nu[0]+nx[0], ocp_qp->Hq[0]+jj, 0, 0, Hq2+jj, 0, 0);
			}
		return;
		}

	int nvc = nu[0]+nx[0];
	int nbc = nbu[0]+nbx[0];
	int ngc = ng[0];
	int nqc = nq[0];
	for(ii=1; ii<=N; ii++)
		{
		nvc += nu[ii];
		nbc += nbu[ii];
		ngc += nbx[ii]+ng[ii];
		nqc += nq[ii];
		}

	int nu_tmp, nq_tmp, nu_tot_tmp, nq_tot_tmp;

	REAL rho;

	GESE(nvc, nqc, 0.0, Ct2, 0, ngc);

	nu_tmp = 0;
	nq_tmp = 0;
	nu_tot_tmp = nvc - nx[0];
	nq_tot_tmp = nqc;
	for(kk=0; kk<=N; kk++)
		{

		nu_tot_tmp -= nu[kk];
		nq_tot_tmp -= nq[kk];

		nq_tmp = nq_tot_tmp;

		for(jj=0; jj<nq[kk]; jj++)
			{

			GESE(nvc+1, nvc, 0.0, Hq2+nq_tmp, 0, 0);
			Hq_nzero2[nq_tmp] = 0;
			rho = 0.0;

			if(kk==0)
				{
				TRCP_L(nu[0]+nx[0], ocp_qp->Hq[kk]+nq[jj], 0, 0, Hq2+nq_tmp, nu_tot_tmp, nu_tot_tmp);
				if(nx[0]>0)
					{
					if(ocp_qp->Hq_nzero[kk][jj]%2==1) // Q nzero
						Hq_nzero2[nq_tmp] |= 1;
					if((ocp_qp->Hq_nzero[kk][jj]>>1)%2==1) // S nzero
						Hq_nzero2[nq_tmp] |= 2;
					}
				if((ocp_qp->Hq_nzero[kk][jj]>>2)%2==1) // R nzero
					Hq_nzero2[nq_tmp] |= 4;
				}
			else
				{
				// XXX make Q full or use SYMM
				// hessian
				if(ocp_qp->Hq_nzero[kk][jj]%2==1)
					{
//					TRTR_L(nx[kk], ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], ocp_qp->Hq[kk]+jj, nu[kk], nu[kk]); // buggy ???
					TRTR_L(nu[kk]+nx[kk], ocp_qp->Hq[kk]+jj, 0, 0, ocp_qp->Hq[kk]+jj, 0, 0);
					GEMM_NN(nu_tmp+nx[0]+1, nx[kk], nx[kk], 1.0, cond_ws->qp_ws->Gamma+kk-1, 0, 0, ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], 0.0, cond_ws->GammaQ+kk-1, 0, 0, cond_ws->GammaQ+kk-1, 0, 0);
//					ROWEX(nx[kk], 1.0, cond_ws->GammaQ+kk-1, nu_tmp+nx[0], 0, cond_ws->tmp_nuxM, 0);
//					SYMV_L(nx[kk], 1.0, ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], cond_ws->qp_ws->Gammab+kk-1, 0, 0.0, cond_ws->tmp_nuxM, 0, cond_ws->tmp_nuxM, 0);
//					rho = 0.5*DOT(nx[kk], cond_ws->tmp_nuxM, 0, cond_ws->qp_ws->Gammab+kk-1, 0);
					SYRK_LN_MN(nu_tmp+nx[0]+1, nu_tmp+nx[0], nx[kk], 1.0, cond_ws->qp_ws->Gamma+kk-1, 0, 0, cond_ws->GammaQ+kk-1, 0, 0, 0.0, Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp+nu[kk], Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp+nu[kk]);
					if(nx[0]>0)
						Hq_nzero2[nq_tmp] |= 7;
					else
						Hq_nzero2[nq_tmp] |= 4;
					}
				if((ocp_qp->Hq_nzero[kk][jj]>>2)%2==1)
					{
					GEAD(nu[kk], nu[kk], 1.0, ocp_qp->Hq[kk]+jj, 0, 0, Hq2+nq_tmp, nu_tot_tmp, nu_tot_tmp);
					Hq_nzero2[nq_tmp] |= 4;
					}
				if((ocp_qp->Hq_nzero[kk][jj]>>1)%2==1)
					{
					GEMM_NN(nu_tmp+nx[0]+1, nu[kk], nx[kk], 1.0, cond_ws->qp_ws->Gamma+kk-1, 0, 0, ocp_qp->Hq[kk]+jj, nu[kk], 0, 1.0, Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp, Hq2+nq_tmp, nu_tot_tmp+nu[kk], nu_tot_tmp);
					if(nx[0]>0)
						Hq_nzero2[nq_tmp] |= 6;
					else
						Hq_nzero2[nq_tmp] |= 4;
					}
				// gradient
				ROWEX(nu_tmp+nx[0], 1.0, Hq2+nq_tmp, nvc, nu_tot_tmp+nu[kk], cond_ws->tmp_nvc, 0);
				COLAD(nu_tmp+nx[0], 1.0, cond_ws->tmp_nvc, 0, Ct2, nu_tot_tmp+nu[kk], ngc+nq_tmp);
				}

			nq_tmp++;

			}

		nu_tmp += nu[kk];

		}

	return;

	}



void COND_QCQP_QC_RHS(struct OCP_QCQP *ocp_qp, struct STRVEC *d2, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws)
	{

	// cond quadr constr
	int ii, jj, kk;

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<=0)
		return;

	int *nu = ocp_qp->dim->nu;
	int *nx = ocp_qp->dim->nx;
	int *nbu = ocp_qp->dim->nbu;
	int *nbx = ocp_qp->dim->nbx;
	int *ng = ocp_qp->dim->ng;
	int *nq = ocp_qp->dim->nq;

	int nvc = nu[0]+nx[0];
	int nbc = nbu[0]+nbx[0];
	int ngc = ng[0];
	int nqc = nq[0];
	for(ii=1; ii<=N; ii++)
		{
		nvc += nu[ii];
		nbc += nbu[ii];
		ngc += nbx[ii]+ng[ii];
		nqc += nq[ii];
		}

	int nu_tmp, nq_tmp, nu_tot_tmp, nq_tot_tmp;

	REAL rho;

	nu_tmp = 0;
	nq_tmp = 0;
	nu_tot_tmp = nvc - nx[0];
	nq_tot_tmp = nqc;
	for(kk=0; kk<=N; kk++)
		{

		nu_tot_tmp -= nu[kk];
		nq_tot_tmp -= nq[kk];

		nq_tmp = nq_tot_tmp;

		for(jj=0; jj<nq[kk]; jj++)
			{

			rho = 0.0;

			if(kk==0)
				{
				// nothing to do
				}
			else
				{
				if(ocp_qp->Hq_nzero[kk][jj]%2==1)
					{
//					SYMV_L(nx[kk], 1.0, ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], cond_ws->qp_ws->Gammab+kk-1, 0, 0.0, cond_ws->tmp_nuxM, 0, cond_ws->tmp_nuxM, 0); // XXX buggy !!!
					TRTR_L(nu[kk]+nx[kk], ocp_qp->Hq[kk]+jj, 0, 0, ocp_qp->Hq[kk]+jj, 0, 0);
					GEMV_N(nx[kk], nx[kk], 1.0, ocp_qp->Hq[kk]+jj, nu[kk], nu[kk], cond_ws->qp_ws->Gammab+kk-1, 0, 0.0, cond_ws->tmp_nuxM, 0, cond_ws->tmp_nuxM, 0);
					rho = 0.5*DOT(nx[kk], cond_ws->tmp_nuxM, 0, cond_ws->qp_ws->Gammab+kk-1, 0);
//					GEMV_N(nu_tmp+nx[0], nx[kk], 1.0, cond_ws->qp_ws->Gamma+kk-1, 0, 0, cond_ws->tmp_nuxM, 0, 0.0, cond_ws->tmp_nvc, 0, cond_ws->tmp_nvc, 0);
					}

				// XXX this does not affect rho (i.e. the RHS)
//				if((ocp_qp->Hq_nzero[kk][jj]>>1)%2==1)
//					{
//					// TODO S
//					}

//				COLAD(nu_tmp+nx[0], 1.0, cond_ws->tmp_nvc, 0, Ct2, nu_tot_tmp+nu[kk], ngc+nq_tmp);
				}

#if defined(DOUBLE_PRECISION)
			BLASFEO_DVECEL(d2, nbc+ngc+nq_tmp) -= rho;
			BLASFEO_DVECEL(d2, 2*nbc+2*ngc+nqc+nq_tmp) += rho;
#else
			BLASFEO_SVECEL(d2, nbc+ngc+nq_tmp) -= rho;
			BLASFEO_SVECEL(d2, 2*nbc+2*ngc+nqc+nq_tmp) += rho;
#endif

			nq_tmp++;

			}

		nu_tmp += nu[kk];

		}

	return;

	}



void COND_QCQP_COND(struct OCP_QCQP *ocp_qp, struct DENSE_QCQP *dense_qp, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws)
	{

	// create tmp QP
	struct OCP_QP tmp_ocp_qp;

	// alias
	tmp_ocp_qp.dim = ocp_qp->dim->qp_dim;
	tmp_ocp_qp.idxb = ocp_qp->idxb;
	tmp_ocp_qp.BAbt = ocp_qp->BAbt;
	tmp_ocp_qp.b = ocp_qp->b;
	tmp_ocp_qp.RSQrq = ocp_qp->RSQrq;
	tmp_ocp_qp.rqz = ocp_qp->rqz;
	tmp_ocp_qp.DCt = ocp_qp->DCt;
	tmp_ocp_qp.d = ocp_qp->d;
	tmp_ocp_qp.d_mask = ocp_qp->d_mask;
	tmp_ocp_qp.Z = ocp_qp->Z;
	tmp_ocp_qp.idxs_rev = ocp_qp->idxs_rev;

	COND_BABT(&tmp_ocp_qp, NULL, NULL, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_RSQRQ(&tmp_ocp_qp, dense_qp->Hv, dense_qp->gz, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_DCTD(&tmp_ocp_qp, dense_qp->idxb, dense_qp->Ct, dense_qp->d, dense_qp->d_mask, dense_qp->idxs_rev, dense_qp->Z, dense_qp->gz, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_QCQP_QC(ocp_qp, dense_qp->Hq, dense_qp->Hq_nzero, dense_qp->Ct, dense_qp->d, cond_arg, cond_ws);

	return;

	}



void COND_QCQP_COND_LHS(struct OCP_QCQP *ocp_qp, struct DENSE_QCQP *dense_qp, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws)
	{

	// create tmp QP
	struct OCP_QP tmp_ocp_qp;

	// alias
	tmp_ocp_qp.dim = ocp_qp->dim->qp_dim;
	tmp_ocp_qp.idxb = ocp_qp->idxb;
	tmp_ocp_qp.BAbt = ocp_qp->BAbt;
	tmp_ocp_qp.b = ocp_qp->b;
	tmp_ocp_qp.RSQrq = ocp_qp->RSQrq;
	tmp_ocp_qp.rqz = ocp_qp->rqz;
	tmp_ocp_qp.DCt = ocp_qp->DCt;
	tmp_ocp_qp.d = ocp_qp->d;
	tmp_ocp_qp.d_mask = ocp_qp->d_mask;
	tmp_ocp_qp.Z = ocp_qp->Z;
	tmp_ocp_qp.idxs_rev = ocp_qp->idxs_rev;

	COND_BAT(&tmp_ocp_qp, NULL, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_RSQ(&tmp_ocp_qp, dense_qp->Hv, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_DCT(&tmp_ocp_qp, dense_qp->idxb, dense_qp->Ct, dense_qp->idxs_rev, dense_qp->Z, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_QCQP_QC_LHS(ocp_qp, dense_qp->Hq, dense_qp->Hq_nzero, dense_qp->Ct, cond_arg, cond_ws);

	return;

	}



void COND_QCQP_COND_RHS(struct OCP_QCQP *ocp_qp, struct DENSE_QCQP *dense_qp, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws)
	{

	// create tmp QP
	struct OCP_QP tmp_ocp_qp;

	// alias
	tmp_ocp_qp.dim = ocp_qp->dim->qp_dim;
	tmp_ocp_qp.idxb = ocp_qp->idxb;
	tmp_ocp_qp.BAbt = ocp_qp->BAbt;
	tmp_ocp_qp.b = ocp_qp->b;
	tmp_ocp_qp.RSQrq = ocp_qp->RSQrq;
	tmp_ocp_qp.rqz = ocp_qp->rqz;
	tmp_ocp_qp.DCt = ocp_qp->DCt;
	tmp_ocp_qp.d = ocp_qp->d;
	tmp_ocp_qp.d_mask = ocp_qp->d_mask;
	tmp_ocp_qp.Z = ocp_qp->Z;
	tmp_ocp_qp.idxs_rev = ocp_qp->idxs_rev;

	COND_B(&tmp_ocp_qp, NULL, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_RQ(&tmp_ocp_qp, dense_qp->gz, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_D(&tmp_ocp_qp, dense_qp->d, dense_qp->d_mask, dense_qp->gz, cond_arg->qp_arg, cond_ws->qp_ws);

	COND_QCQP_QC_RHS(ocp_qp, dense_qp->d, cond_arg, cond_ws);

	return;

	}



void COND_QCQP_EXPAND_SOL(struct OCP_QCQP *ocp_qp, struct DENSE_QCQP_SOL *dense_qp_sol, struct OCP_QCQP_SOL *ocp_qp_sol, struct COND_QCQP_ARG *cond_arg, struct COND_QCQP_WS *cond_ws)
	{

	int ii, jj;

	// extract dim
	int N = ocp_qp->dim->N;
	int *nu = ocp_qp->dim->nu;
	int *nx = ocp_qp->dim->nx;
	int *ng = ocp_qp->dim->ng;
	int *nq = ocp_qp->dim->nq;

	// create tmp QP
	struct OCP_QP tmp_ocp_qp;

	// alias
	tmp_ocp_qp.dim = ocp_qp->dim->qp_dim;
	tmp_ocp_qp.idxb = ocp_qp->idxb;
	tmp_ocp_qp.BAbt = ocp_qp->BAbt;
	tmp_ocp_qp.b = ocp_qp->b;
	tmp_ocp_qp.RSQrq = ocp_qp->RSQrq;
	tmp_ocp_qp.rqz = ocp_qp->rqz;
	tmp_ocp_qp.DCt = ocp_qp->DCt;
	tmp_ocp_qp.d = ocp_qp->d;
	tmp_ocp_qp.d_mask = ocp_qp->d_mask;
	tmp_ocp_qp.Z = ocp_qp->Z;
	tmp_ocp_qp.idxs_rev = ocp_qp->idxs_rev;
	// TODO d_mask ????????


	// create tmp QP
	struct OCP_QP_SOL tmp_ocp_qp_sol;

	// alias
	tmp_ocp_qp_sol.dim = ocp_qp_sol->dim->qp_dim;
	tmp_ocp_qp_sol.ux = ocp_qp_sol->ux;
	tmp_ocp_qp_sol.pi = ocp_qp_sol->pi;
	tmp_ocp_qp_sol.lam = ocp_qp_sol->lam;
	tmp_ocp_qp_sol.t = ocp_qp_sol->t;


	// create tmp QP
	struct DENSE_QP_SOL tmp_dense_qp_sol;

	// alias
	tmp_dense_qp_sol.dim = dense_qp_sol->dim->qp_dim;
	tmp_dense_qp_sol.v = dense_qp_sol->v;
	tmp_dense_qp_sol.pi = dense_qp_sol->pi;
	tmp_dense_qp_sol.lam = dense_qp_sol->lam;
	tmp_dense_qp_sol.t = dense_qp_sol->t;

	int bkp_comp_prim_sol = cond_arg->qp_arg->comp_prim_sol;
	int bkp_comp_dual_sol_eq = cond_arg->qp_arg->comp_dual_sol_eq;
	int bkp_comp_dual_sol_ineq = cond_arg->qp_arg->comp_dual_sol_ineq;

	cond_arg->qp_arg->comp_prim_sol = 1 & bkp_comp_prim_sol;
	cond_arg->qp_arg->comp_dual_sol_eq = 0 & bkp_comp_dual_sol_eq;
	cond_arg->qp_arg->comp_dual_sol_ineq = 1 & bkp_comp_dual_sol_ineq;

//	cond_arg->comp_dual_sol = 0; // compute dual solution
//	cond_arg->qp_arg->comp_dual_sol = 0; // compute dual solution

	EXPAND_SOL(&tmp_ocp_qp, &tmp_dense_qp_sol, &tmp_ocp_qp_sol, cond_arg->qp_arg, cond_ws->qp_ws);

	// linearize quadr constr
	for(ii=0; ii<=N; ii++)
		{
		GECP(nu[ii]+nx[ii], ng[ii]+nq[ii], ocp_qp->DCt+ii, 0, 0, cond_ws->tmp_DCt+ii, 0, 0);
		for(jj=0; jj<nq[ii]; jj++)
			{
			SYMV_L(nu[ii]+nx[ii], 1.0, ocp_qp->Hq[ii]+jj, 0, 0, ocp_qp_sol->ux+ii, 0, 0.0, cond_ws->tmp_nuxM, 0, cond_ws->tmp_nuxM, 0);
			COLAD(nu[ii]+nx[ii], 1.0, cond_ws->tmp_nuxM, 0, cond_ws->tmp_DCt+ii, 0, ng[ii]+jj);
			}
		}

	tmp_ocp_qp.DCt = cond_ws->tmp_DCt;

	cond_arg->qp_arg->comp_prim_sol = 0 & bkp_comp_prim_sol;;
	cond_arg->qp_arg->comp_dual_sol_eq = 1 & bkp_comp_dual_sol_eq;
	cond_arg->qp_arg->comp_dual_sol_ineq = 0 & bkp_comp_dual_sol_ineq;

	EXPAND_SOL(&tmp_ocp_qp, &tmp_dense_qp_sol, &tmp_ocp_qp_sol, cond_arg->qp_arg, cond_ws->qp_ws);

	cond_arg->qp_arg->comp_prim_sol = bkp_comp_prim_sol;
	cond_arg->qp_arg->comp_dual_sol_eq = bkp_comp_dual_sol_eq;
	cond_arg->qp_arg->comp_dual_sol_ineq = bkp_comp_dual_sol_ineq;

	return;

	}



