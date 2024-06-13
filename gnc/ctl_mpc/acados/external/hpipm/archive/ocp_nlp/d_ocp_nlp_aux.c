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



#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include "../include/hpipm_d_erk_int.h"
#include "../include/hpipm_d_ocp_qp.h"
#include "../include/hpipm_d_ocp_nlp.h"
#include "../include/hpipm_d_ocp_nlp_sol.h"



void d_cvt_erk_int_to_ocp_qp(int n, struct d_erk_workspace *erk_ws, struct d_ocp_qp *qp, struct d_ocp_nlp_sol *nlp_sol)
	{

	int ii;

	int nx = qp->nx[n];
	int nu = qp->nu[n];
	struct blasfeo_dmat *BAbt = qp->BAbt+n;
	struct blasfeo_dvec *b = qp->b+n;

	struct blasfeo_dvec *ux = nlp_sol->ux+n;

//	int nx = erk_ws->nx;
	int nf = erk_ws->nf;
	int nX = nx*(1+nf);

	double *x = erk_ws->x_for;
//	if(adj_sens!=0 & erk_ws->erk_arg->adj_sens!=0)
//		x = erk_ws->x + nX*erk_ws->erk_arg->steps;

//	blasfeo_pack_tran_dmat(nx[1], nu[0]+nx[0], x, nx[1], BAbt, 0, 0);
	blasfeo_pack_tran_dmat(nx, nu+nx, x+nx, nx, BAbt, 0, 0);

	blasfeo_pack_dvec(nx, x, b, 0);
	// XXX not compute this again in residuals !!!
	blasfeo_dgemv_t(nu+nx, nx, -1.0, BAbt, 0, 0, ux, 0, 1.0, b, 0, b, 0);
	blasfeo_drowin(nx, 1.0, b, 0, BAbt, nu+nx, 0);

	return;

	}



void d_cvt_erk_int_to_ocp_qp_rhs(int n, struct d_erk_workspace *erk_ws, struct d_ocp_qp *qp, struct d_ocp_nlp_sol *nlp_sol)
	{

	int ii;

	int nx = qp->nx[n];
	int nu = qp->nu[n];
	struct blasfeo_dmat *BAbt = qp->BAbt+n;
	struct blasfeo_dvec *b = qp->b+n;

	struct blasfeo_dvec *ux = nlp_sol->ux+n;

//	int nx = erk_ws->nx;
//	int np = erk_ws->np;
	int nf = erk_ws->nf;

	int nX = nx*(1+nf);

//	double *x = erk_ws->x_for;
//	if(erk_ws->erk_arg->adj_sens!=0)
//	double *x = erk_ws->x_for + nX*erk_ws->erk_arg->steps;
	double *x = erk_ws->x_for;

	struct blasfeo_dvec sl;
	blasfeo_create_dvec(nu+nx, &sl, erk_ws->l);

	blasfeo_daxpy(nu+nx, 1.0, &sl, 0, qp->rq+n, 0, qp->rq+n, 0);
	blasfeo_pack_dvec(nx, x, b, 0);
	// XXX not compute this again in residuals !!!
	blasfeo_dgemv_nt(nu+nx, nx, -1.0, -1.0, BAbt, 0, 0, nlp_sol->pi+n, 0, ux, 0, 1.0, 1.0, qp->rq+n, 0, b, 0, qp->rq+n, 0, b, 0);
	blasfeo_drowin(nu+nx, 1.0, qp->rq+n, 0, qp->RSQrq+n, nu+nx, 0);
	blasfeo_drowin(nx, 1.0, b, 0, BAbt, nu+nx, 0);

	return;

	}
