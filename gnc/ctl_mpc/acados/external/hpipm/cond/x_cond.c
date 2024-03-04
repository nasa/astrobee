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

void COND_QP_COMPUTE_DIM(struct OCP_QP_DIM *ocp_dim, struct DENSE_QP_DIM *dense_dim)
	{

	int N = ocp_dim->N;
	int *nx = ocp_dim->nx;
	int *nu = ocp_dim->nu;
	int *nbx = ocp_dim->nbx;
	int *nbu = ocp_dim->nbu;
	int *ng = ocp_dim->ng;
	int *ns = ocp_dim->ns;
	int *nsbx = ocp_dim->nsbx;
	int *nsbu = ocp_dim->nsbu;
	int *nsg = ocp_dim->nsg;

	int ii;

	int nvc = 0;
	int nec = 0;
	int nbc = 0;
	int ngc = 0;
	int nsc = 0;
	int nsbc = 0;
	int nsgc = 0;

	// first stage
	nvc += nx[0]+nu[0];
	nbc += nbx[0]+nbu[0];
	ngc += ng[0];
	nsc += ns[0];
	nsbc += nsbx[0]+nsbu[0];
	nsgc += nsg[0];
	// remaining stages
	for(ii=1; ii<=N; ii++)
		{
		nvc += nu[ii];
		nbc += nbu[ii];
		ngc += nbx[ii]+ng[ii];
		nsc += ns[ii];
		nsbc += nsbu[ii];
		nsgc += nsbx[ii]+nsg[ii];
		}

	dense_dim->nv = nvc;
	dense_dim->ne = nec;
	dense_dim->nb = nbc;
	dense_dim->ng = ngc;
	dense_dim->ns = nsc;
	dense_dim->nsb = nsbc;
	dense_dim->nsg = nsgc;

	return;

	}



hpipm_size_t COND_QP_ARG_MEMSIZE()
	{

	hpipm_size_t size = 0;

	return size;

	}



void COND_QP_ARG_CREATE(struct COND_QP_ARG *cond_arg, void *mem)
	{

	cond_arg->memsize = COND_QP_ARG_MEMSIZE();

	return;

	}



void COND_QP_ARG_SET_DEFAULT(struct COND_QP_ARG *cond_arg)
	{

	cond_arg->cond_alg = 0; // condensing algorithm
	cond_arg->cond_last_stage = 1; // condense last stage
	cond_arg->comp_prim_sol = 1; // compute primal solution (v)
	cond_arg->comp_dual_sol_eq = 1; // compute dual solution equality constr (pi)
	cond_arg->comp_dual_sol_ineq = 1; // compute dual solution inequality constr (lam t)
	cond_arg->square_root_alg = 1; // square root algorithm (faster but requires RSQ>0)

	return;

	}



void COND_QP_ARG_SET_COND_ALG(int cond_alg, struct COND_QP_ARG *cond_arg)
	{

	cond_arg->cond_alg = cond_alg;

	return;

	}



void COND_QP_ARG_SET_RIC_ALG(int ric_alg, struct COND_QP_ARG *cond_arg)
	{

	cond_arg->square_root_alg = ric_alg;

	return;

	}



void COND_QP_ARG_SET_COND_LAST_STAGE(int cond_last_stage, struct COND_QP_ARG *cond_arg)
	{

	cond_arg->cond_last_stage = cond_last_stage;

	return;

	}



void COND_QP_ARG_SET_COMP_PRIM_SOL(int value, struct COND_QP_ARG *cond_arg)
	{

	cond_arg->comp_prim_sol = value;

	return;

	}



void COND_QP_ARG_SET_COMP_DUAL_SOL_EQ(int value, struct COND_QP_ARG *cond_arg)
	{

	cond_arg->comp_dual_sol_eq = value;

	return;

	}



void COND_QP_ARG_SET_COMP_DUAL_SOL_INEQ(int value, struct COND_QP_ARG *cond_arg)
	{

	cond_arg->comp_dual_sol_ineq = value;

	return;

	}



hpipm_size_t COND_QP_WS_MEMSIZE(struct OCP_QP_DIM *ocp_dim, struct COND_QP_ARG *cond_arg)
	{

	int ii;
	int nu_tmp;

	int N = ocp_dim->N;
	int *nx = ocp_dim->nx;
	int *nu = ocp_dim->nu;
	int *nb = ocp_dim->nb;
	int *ng = ocp_dim->ng;

	// compute core qp size and max size
	int nvt = 0;
	int net = 0;
	int nbt = 0;
	int ngt = 0;
	int nxM = 0;
	int nuM = 0;
	int nbM = 0;
	int ngM = 0;

	for(ii=0; ii<N; ii++)
		{
		nvt += nx[ii]+nu[ii];
		net += nx[ii+1];
		nbt += nb[ii];
		ngt += ng[ii];
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
		nbM = nb[ii]>nbM ? nb[ii] : nbM;
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}
	ii = N;
	nvt += nx[ii]+nu[ii];
	nbt += nb[ii];
	ngt += ng[ii];
	nxM = nx[ii]>nxM ? nx[ii] : nxM;
	nuM = nu[ii]>nuM ? nu[ii] : nuM;
	nbM = nb[ii]>nbM ? nb[ii] : nbM;
	ngM = ng[ii]>ngM ? ng[ii] : ngM;

	hpipm_size_t size = 0;

	size += 1*N*sizeof(struct STRMAT); // Gamma
	size += 1*N*sizeof(struct STRVEC); // Gammab
	size += 2*sizeof(struct STRVEC); // tmp_nbgM tmp_nuxM
	if(cond_arg->cond_alg==0)
		{
		size += 1*(N+1)*sizeof(struct STRMAT); // L
		size += 2*sizeof(struct STRMAT); // Lx AL
		size += 1*(N+1)*sizeof(struct STRVEC); // l
		}
	else
		{
		size += 1*sizeof(struct STRMAT); // GammaQ
		}

	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nu_tmp += nu[ii];
		size += SIZE_STRMAT(nu_tmp+nx[0]+1, nx[ii+1]); // Gamma
		}
	if(cond_arg->cond_alg==0)
		{
		for(ii=0; ii<=N; ii++)
			size += SIZE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // L
		size += SIZE_STRMAT(nxM+1, nxM); // Lx
		size += SIZE_STRMAT(nuM+nxM+1, nxM); // AL
		for(ii=0; ii<=N; ii++)
			size += SIZE_STRVEC(nu[ii]+nx[ii]); // l
		}
	else
		{
		nu_tmp = 0;
		for(ii=0; ii<N; ii++)
			nu_tmp += nu[ii];
		size += SIZE_STRMAT(nu_tmp+nxM+1, nxM); // GammaQ
		}

	for(ii=0; ii<N; ii++)
		size += 1*SIZE_STRVEC(nx[ii+1]); // Gammab
	size += 1*SIZE_STRVEC(nbM+ngM); // tmp_nbgM
	size += 1*SIZE_STRVEC(nuM+nxM); // tmp_nuxM

	size += 1*64; // align to typical cache line size
	size += 1*8; // align to 64 bits

	size = (size+63)/64*64; // make multiple of typical cache line size

	return size;

	}



void COND_QP_WS_CREATE(struct OCP_QP_DIM *ocp_dim, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws, void *mem)
	{

	int ii;
	int nu_tmp;

	size_t s_ptr;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = COND_QP_WS_MEMSIZE(ocp_dim, cond_arg);
	hpipm_zero_memset(memsize, mem);

	int N = ocp_dim->N;
	int *nx = ocp_dim->nx;
	int *nu = ocp_dim->nu;
	int *nb = ocp_dim->nb;
	int *ng = ocp_dim->ng;

	// compute core qp dim and max dim
	int nvt = 0;
	int net = 0;
	int nbt = 0;
	int ngt = 0;
	int nxM = 0;
	int nuM = 0;
	int nbM = 0;
	int ngM = 0;

	for(ii=0; ii<N; ii++)
		{
		nvt += nx[ii]+nu[ii];
		net += nx[ii+1];
		nbt += nb[ii];
		ngt += ng[ii];
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
		nbM = nb[ii]>nbM ? nb[ii] : nbM;
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		}
	ii = N;
	nvt += nx[ii]+nu[ii];
	nbt += nb[ii];
	ngt += ng[ii];
	nxM = nx[ii]>nxM ? nx[ii] : nxM;
	nuM = nu[ii]>nuM ? nu[ii] : nuM;
	nbM = nb[ii]>nbM ? nb[ii] : nbM;
	ngM = ng[ii]>ngM ? ng[ii] : ngM;


	// align to typical cache line size
	s_ptr = (size_t) mem;
	s_ptr = (s_ptr+7)/8*8;


	// matrix struct
	struct STRMAT *sm_ptr = (struct STRMAT *) s_ptr;

	cond_ws->Gamma = sm_ptr;
	sm_ptr += N;
	if(cond_arg->cond_alg==0)
		{
		cond_ws->L = sm_ptr;
		sm_ptr += N+1;
		cond_ws->Lx = sm_ptr;
		sm_ptr += 1;
		cond_ws->AL = sm_ptr;
		sm_ptr += 1;
		}
	else
		{
		cond_ws->GammaQ = sm_ptr;
		sm_ptr += 1;
		}


	// vector struct
	struct STRVEC *sv_ptr = (struct STRVEC *) sm_ptr;

	cond_ws->Gammab = sv_ptr;
	sv_ptr += N;
	cond_ws->tmp_nbgM = sv_ptr;
	sv_ptr += 1;
	cond_ws->tmp_nuxM = sv_ptr;
	sv_ptr += 1;
	if(cond_arg->cond_alg==0)
		{
		cond_ws->l = sv_ptr;
		sv_ptr += N+1;
		}


	// align to typical cache line size
	s_ptr = (size_t) sv_ptr;
	s_ptr = (s_ptr+63)/64*64;


	// void stuf
	char *c_ptr = (char *) s_ptr;
	char *c_tmp;

	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nu_tmp += nu[ii];
		CREATE_STRMAT(nu_tmp+nx[0]+1, nx[ii+1], cond_ws->Gamma+ii, c_ptr);
		c_ptr += (cond_ws->Gamma+ii)->memsize;
		}
	if(cond_arg->cond_alg==0)
		{
		for(ii=0; ii<=N; ii++)
			{
			CREATE_STRMAT(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], cond_ws->L+ii, c_ptr);
			c_ptr += (cond_ws->L+ii)->memsize;
			}
		CREATE_STRMAT(nxM+1, nxM, cond_ws->Lx, c_ptr);
		c_ptr += cond_ws->Lx->memsize;
		CREATE_STRMAT(nuM+nxM+1, nxM, cond_ws->AL, c_ptr);
		c_ptr += cond_ws->AL->memsize;
		for(ii=0; ii<=N; ii++)
			{
			CREATE_STRVEC(nu[ii]+nx[ii], cond_ws->l+ii, c_ptr);
			c_ptr += (cond_ws->l+ii)->memsize;
			}
		}
	else
		{
		nu_tmp = 0;
		for(ii=0; ii<N; ii++)
			nu_tmp += nu[ii];
		CREATE_STRMAT(nu_tmp+nxM+1, nxM, cond_ws->GammaQ, c_ptr);
		c_ptr += cond_ws->GammaQ->memsize;
		}
	for(ii=0; ii<N; ii++)
		{
		CREATE_STRVEC(nx[ii+1], cond_ws->Gammab+ii, c_ptr);
		c_ptr += (cond_ws->Gammab+ii)->memsize;
		}
	CREATE_STRVEC(nbM+ngM, cond_ws->tmp_nbgM, c_ptr);
	c_ptr += cond_ws->tmp_nbgM->memsize;
	c_tmp = c_ptr;
	CREATE_STRVEC(nuM+nxM, cond_ws->tmp_nuxM, c_ptr);
	c_ptr += cond_ws->tmp_nuxM->memsize;

	cond_ws->bs = N;

	cond_ws->memsize = COND_QP_WS_MEMSIZE(ocp_dim, cond_arg);

#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + cond_ws->memsize)
		{
		printf("\nCreate_cond_qp_ocp2dense: outsize memory bounds!\n\n");
		exit(1);
		}
#endif

	return;

	}



void COND_QP_COND(struct OCP_QP *ocp_qp, struct DENSE_QP *dense_qp, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	COND_BABT(ocp_qp, NULL, NULL, cond_arg, cond_ws);

	COND_RSQRQ(ocp_qp, dense_qp->Hv, dense_qp->gz, cond_arg, cond_ws);

	COND_DCTD(ocp_qp, dense_qp->idxb, dense_qp->Ct, dense_qp->d, dense_qp->d_mask, dense_qp->idxs_rev, dense_qp->Z, dense_qp->gz, cond_arg, cond_ws);

	return;

	}



void COND_QP_COND_LHS(struct OCP_QP *ocp_qp, struct DENSE_QP *dense_qp, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	COND_BAT(ocp_qp, NULL, cond_arg, cond_ws);

	COND_RSQ(ocp_qp, dense_qp->Hv, cond_arg, cond_ws);

	COND_DCT(ocp_qp, dense_qp->idxb, dense_qp->Ct, dense_qp->idxs_rev, dense_qp->Z, cond_arg, cond_ws);

	return;

	}



void COND_QP_COND_RHS(struct OCP_QP *ocp_qp, struct DENSE_QP *dense_qp, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	COND_B(ocp_qp, NULL, cond_arg, cond_ws);

	COND_RQ(ocp_qp, dense_qp->gz, cond_arg, cond_ws);

	COND_D(ocp_qp, dense_qp->d, dense_qp->d_mask, dense_qp->gz, cond_arg, cond_ws);

	return;

	}



void COND_QP_EXPAND_SOL(struct OCP_QP *ocp_qp, struct DENSE_QP_SOL *dense_qp_sol, struct OCP_QP_SOL *ocp_qp_sol, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	EXPAND_SOL(ocp_qp, dense_qp_sol, ocp_qp_sol, cond_arg, cond_ws);

	return;

	}



// TODO remove
void COND_QP_EXPAND_PRIMAL_SOL(struct OCP_QP *ocp_qp, struct DENSE_QP_SOL *dense_qp_sol, struct OCP_QP_SOL *ocp_qp_sol, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	EXPAND_PRIMAL_SOL(ocp_qp, dense_qp_sol, ocp_qp_sol, cond_arg, cond_ws);

	return;

	}



/************************************************
* update cond
************************************************/

void COND_QP_UPDATE(int *idxc, struct OCP_QP *ocp_qp, struct DENSE_QP *dense_qp, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	UPDATE_COND_BABT(idxc, ocp_qp, NULL, NULL, cond_arg, cond_ws);

	UPDATE_COND_RSQRQ_N2NX3(idxc, ocp_qp, dense_qp->Hv, dense_qp->gz, cond_arg, cond_ws);

	UPDATE_COND_DCTD(idxc, ocp_qp, dense_qp->idxb, dense_qp->Ct, dense_qp->d, dense_qp->idxs_rev, dense_qp->Z, dense_qp->gz, cond_arg, cond_ws);

	return;

	}




