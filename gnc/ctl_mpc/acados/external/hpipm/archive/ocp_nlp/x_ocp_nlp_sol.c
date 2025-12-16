/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2017-2018 by Gianluca Frison.                                                     *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* This program is free software: you can redistribute it and/or modify                            *
* it under the terms of the GNU General Public License as published by                            *
* the Free Software Foundation, either version 3 of the License, or                               *
* (at your option) any later version                                                              *.
*                                                                                                 *
* This program is distributed in the hope that it will be useful,                                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                   *
* GNU General Public License for more details.                                                    *
*                                                                                                 *
* You should have received a copy of the GNU General Public License                               *
* along with this program.  If not, see <https://www.gnu.org/licenses/>.                          *
*                                                                                                 *
* The authors designate this particular file as subject to the "Classpath" exception              *
* as provided by the authors in the LICENSE file that accompained this code.                      *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/



hpipm_size_t MEMSIZE_OCP_NLP_SOL(int N, int *nx, int *nu, int *nb, int *ng, int *ns)
	{

	int ii;

	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nu[ii]+nx[ii]+2*ns[ii];
		net += nx[ii+1];
		nct += nb[ii]+ng[ii]+ns[ii];
		}
	nvt += nu[ii]+nx[ii]+2*ns[ii];
	nct += nb[ii]+ng[ii]+ns[ii];

	hpipm_size_t size = 0;

	size += 3*(N+1)*sizeof(struct STRVEC); // ux lam t
	size += 1*N*sizeof(struct STRVEC); // pi

	size += 1*SIZE_STRVEC(nvt); // ux
	size += 1*SIZE_STRVEC(net); // pi
	size += 2*SIZE_STRVEC(2*nct); // lam t

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 64; // align to typical cache line size
	
	return size;

	}



void CREATE_OCP_NLP_SOL(int N, int *nx, int *nu, int *nb, int *ng, int *ns, struct OCP_NLP_SOL *nlp_sol, void *mem)
	{

	int ii;


	int nvt = 0;
	int net = 0;
	int nct = 0;
	for(ii=0; ii<N; ii++)
		{
		nvt += nu[ii]+nx[ii]+2*ns[ii];
		net += nx[ii+1];
		nct += nb[ii]+ng[ii]+ns[ii];
		}
	nvt += nu[ii]+nx[ii]+2*ns[ii];
	nct += nb[ii]+ng[ii]+ns[ii];


	// vector struct stuff
	struct STRVEC *sv_ptr = (struct STRVEC *) mem;

	//
	nlp_sol->ux = sv_ptr;
	sv_ptr += N+1;
	//
	nlp_sol->pi = sv_ptr;
	sv_ptr += N;
	//
	nlp_sol->lam = sv_ptr;
	sv_ptr += N+1;
	//
	nlp_sol->t = sv_ptr;
	sv_ptr += N+1;


	// align to typical cache line size
	long long l_ptr = (long long) sv_ptr;
	l_ptr = (l_ptr+63)/64*64;


	// double stuff
	char *c_ptr;
	c_ptr = (char *) l_ptr;

	char *tmp_ptr;

	// ux
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(nvt);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(nu[ii]+nx[ii]+2*ns[ii], nlp_sol->ux+ii, tmp_ptr);
		tmp_ptr += nu[ii]*sizeof(REAL); // u
		tmp_ptr += nx[ii]*sizeof(REAL); // x
		tmp_ptr += ns[ii]*sizeof(REAL); // s_ls
		tmp_ptr += ns[ii]*sizeof(REAL); // s_us
		}
	// pi
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(net);
	for(ii=0; ii<N; ii++)
		{
		CREATE_STRVEC(nx[ii+1], nlp_sol->pi+ii, tmp_ptr);
		tmp_ptr += (nx[ii+1])*sizeof(REAL); // pi
		}
	// lam
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(2*nct);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*ns[ii], nlp_sol->lam+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL); // lb
		tmp_ptr += ng[ii]*sizeof(REAL); // lg
		tmp_ptr += nb[ii]*sizeof(REAL); // ub
		tmp_ptr += ng[ii]*sizeof(REAL); // ug
		tmp_ptr += ns[ii]*sizeof(REAL); // ls
		tmp_ptr += ns[ii]*sizeof(REAL); // us
		}
	// t
	tmp_ptr = c_ptr;
	c_ptr += SIZE_STRVEC(2*nct);
	for(ii=0; ii<=N; ii++)
		{
		CREATE_STRVEC(2*nb[ii]+2*ng[ii]+2*ns[ii], nlp_sol->t+ii, tmp_ptr);
		tmp_ptr += nb[ii]*sizeof(REAL); // lb
		tmp_ptr += ng[ii]*sizeof(REAL); // lg
		tmp_ptr += nb[ii]*sizeof(REAL); // ub
		tmp_ptr += ng[ii]*sizeof(REAL); // ug
		tmp_ptr += ns[ii]*sizeof(REAL); // ls
		tmp_ptr += ns[ii]*sizeof(REAL); // us
		}
	

	nlp_sol->memsize = MEMSIZE_OCP_NLP_SOL(N, nx, nu, nb, ng, ns);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + nlp_sol->memsize)
		{
		printf("\nCreate_ocp_nlp_sol: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void CVT_OCP_NLP_SOL_TO_COLMAJ(struct OCP_NLP *qp, struct OCP_NLP_SOL *qp_sol, REAL **u, REAL **x, REAL **ls, REAL **us, REAL **pi, REAL **lam_lb, REAL **lam_ub, REAL **lam_lg, REAL **lam_ug, REAL **lam_ls, REAL **lam_us)
	{

	int N = qp->N;
	int *nx = qp->nx;
	int *nu = qp->nu;
	int *nb = qp->nb;
	int *ng = qp->ng;
	int *ns = qp->ns;
	
	int ii;

	for(ii=0; ii<N; ii++)
		{
		CVT_STRVEC2VEC(nx[ii+1], qp_sol->pi+ii, 0, pi[ii]);
		}
	
	for(ii=0; ii<=N; ii++)
		{
		CVT_STRVEC2VEC(nu[ii], qp_sol->ux+ii, 0, u[ii]);
		CVT_STRVEC2VEC(nx[ii], qp_sol->ux+ii, nu[ii], x[ii]);
		if(nb[ii]>0)
			{
			CVT_STRVEC2VEC(nb[ii], qp_sol->lam+ii, 0, lam_lb[ii]);
			CVT_STRVEC2VEC(nb[ii], qp_sol->lam+ii, nb[ii]+ng[ii], lam_ub[ii]);
			}
		if(ng[ii]>0)
			{
			CVT_STRVEC2VEC(ng[ii], qp_sol->lam+ii, nb[ii], lam_lg[ii]);
			CVT_STRVEC2VEC(ng[ii], qp_sol->lam+ii, 2*nb[ii]+ng[ii], lam_ug[ii]);
			}
		if(ns[ii]>0)
			{
			CVT_STRVEC2VEC(ns[ii], qp_sol->ux+ii, nu[ii]+nx[ii], ls[ii]);
			CVT_STRVEC2VEC(ns[ii], qp_sol->ux+ii, nu[ii]+nx[ii]+ns[ii], us[ii]);
			CVT_STRVEC2VEC(ns[ii], qp_sol->lam+ii, 2*nb[ii]+2*ng[ii], lam_ls[ii]);
			CVT_STRVEC2VEC(ns[ii], qp_sol->lam+ii, 2*nb[ii]+2*ng[ii]+ns[ii], lam_us[ii]);
			}
		}

	return;

	}



void CVT_OCP_NLP_SOL_TO_ROWMAJ(struct OCP_NLP *qp, struct OCP_NLP_SOL *qp_sol, REAL **u, REAL **x, REAL **ls, REAL **us, REAL **pi, REAL **lam_lb, REAL **lam_ub, REAL **lam_lg, REAL **lam_ug, REAL **lam_ls, REAL **lam_us)
	{

	int N = qp->N;
	int *nx = qp->nx;
	int *nu = qp->nu;
	int *nb = qp->nb;
	int *ng = qp->ng;
	int *ns = qp->ns;
	
	int ii;

	for(ii=0; ii<N; ii++)
		{
		CVT_STRVEC2VEC(nx[ii+1], qp_sol->pi+ii, 0, pi[ii]);
		}

	for(ii=0; ii<=N; ii++)
		{
		CVT_STRVEC2VEC(nu[ii], qp_sol->ux+ii, 0, u[ii]);
		CVT_STRVEC2VEC(nx[ii], qp_sol->ux+ii, nu[ii], x[ii]);
		if(nb[ii]>0)
			{
			CVT_STRVEC2VEC(nb[ii], qp_sol->lam+ii, 0, lam_lb[ii]);
			CVT_STRVEC2VEC(nb[ii], qp_sol->lam+ii, nb[ii]+ng[ii], lam_ub[ii]);
			}
		if(ng[ii]>0)
			{
			CVT_STRVEC2VEC(ng[ii], qp_sol->lam+ii, nb[ii], lam_lg[ii]);
			CVT_STRVEC2VEC(ng[ii], qp_sol->lam+ii, 2*nb[ii]+ng[ii], lam_ug[ii]);
			}
		if(ns[ii]>0)
			{
			CVT_STRVEC2VEC(ns[ii], qp_sol->ux+ii, nu[ii]+nx[ii], ls[ii]);
			CVT_STRVEC2VEC(ns[ii], qp_sol->ux+ii, nu[ii]+nx[ii]+ns[ii], us[ii]);
			CVT_STRVEC2VEC(ns[ii], qp_sol->lam+ii, 2*nb[ii]+2*ng[ii], lam_ls[ii]);
			CVT_STRVEC2VEC(ns[ii], qp_sol->lam+ii, 2*nb[ii]+2*ng[ii]+ns[ii], lam_us[ii]);
			}
		}

	return;

	}



void CVT_OCP_NLP_SOL_TO_LIBSTR(struct OCP_NLP *qp, struct OCP_NLP_SOL *qp_sol, struct STRVEC *u, struct STRVEC *ls, struct STRVEC *us, struct STRVEC *x, struct STRVEC *pi, struct STRVEC *lam_lb, struct STRVEC *lam_ub, struct STRVEC *lam_lg, struct STRVEC *lam_ug, struct STRVEC *lam_ls, struct STRVEC *lam_us)
	{

	int N = qp->N;
	int *nx = qp->nx;
	int *nu = qp->nu;
	int *nb = qp->nb;
	int *ng = qp->ng;
	int *ns = qp->ns;
	
	int ii;

	for(ii=0; ii<N; ii++)
		{
		VECCP_LIBSTR(nx[ii+1], qp_sol->pi+ii, 0, pi+ii, 0);
		}

	for(ii=0; ii<=N; ii++)
		{
		VECCP_LIBSTR(nu[ii], qp_sol->ux+ii, 0, u+ii, 0);
		VECCP_LIBSTR(nx[ii], qp_sol->ux+ii, nu[ii], x+ii, 0);
		if(nb[ii]>0)
			{
			VECCP_LIBSTR(nb[ii], qp_sol->lam+ii, 0, lam_lb+ii, 0);
			VECCP_LIBSTR(nb[ii], qp_sol->lam+ii, nb[ii]+ng[ii], lam_ub+ii, 0);
			}
		if(ng[ii]>0)
			{
			VECCP_LIBSTR(ng[ii], qp_sol->lam+ii, nb[ii], lam_lg+ii, 0);
			VECCP_LIBSTR(ng[ii], qp_sol->lam+ii, 2*nb[ii]+ng[ii], lam_ug+ii, 0);
			}
		if(ns[ii]>0)
			{
			VECCP_LIBSTR(ns[ii], qp_sol->ux+ii, nu[ii]+nx[ii], ls+ii, 0);
			VECCP_LIBSTR(ns[ii], qp_sol->ux+ii, nu[ii]+nx[ii]+ns[ii], us+ii, 0);
			VECCP_LIBSTR(ns[ii], qp_sol->lam+ii, 2*nb[ii]+2*ng[ii], lam_ls+ii, 0);
			VECCP_LIBSTR(ns[ii], qp_sol->lam+ii, 2*nb[ii]+2*ng[ii]+ns[ii], lam_us+ii, 0);
			}
		}

	return;

	}
