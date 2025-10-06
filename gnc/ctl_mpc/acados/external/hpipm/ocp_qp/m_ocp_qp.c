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



#if defined(RUNTIME_CHECKS)
#include <stdlib.h>
#endif

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_m_aux.h>

#include <hpipm_d_ocp_qp.h>
#include <hpipm_s_ocp_qp.h>



void m_cvt_d_ocp_qp_to_s_ocp_qp(struct d_ocp_qp *d_qp, struct s_ocp_qp *s_qp)
	{

	// TODO check that they have equal sizes !!!!!

	int N = d_qp->N;
	int *nx = d_qp->nx;
	int *nu = d_qp->nu;
	int *nb = d_qp->nb;
	int *ng = d_qp->ng;

	int ii, jj;

	for(ii=0; ii<N; ii++)
		{
		m_cvt_d2blasfeo_smat(nu[ii]+nx[ii]+1, nx[ii+1], d_qp->BAbt+ii, 0, 0, s_qp->BAbt+ii, 0, 0);
		m_cvt_d2blasfeo_smat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], d_qp->RSQrq+ii, 0, 0, s_qp->RSQrq+ii, 0, 0);
		m_cvt_d2blasfeo_smat(nu[ii]+nx[ii], ng[ii], d_qp->DCt+ii, 0, 0, s_qp->DCt+ii, 0, 0);
		m_cvt_d2blasfeo_svec(nx[ii+1], d_qp->b+ii, 0, s_qp->b+ii, 0);
		m_cvt_d2blasfeo_svec(nu[ii]+nx[ii], d_qp->rq+ii, 0, s_qp->rq+ii, 0);
		m_cvt_d2blasfeo_svec(nb[ii], d_qp->d_lb+ii, 0, s_qp->d_lb+ii, 0);
		m_cvt_d2blasfeo_svec(nb[ii], d_qp->d_ub+ii, 0, s_qp->d_ub+ii, 0);
		m_cvt_d2blasfeo_svec(ng[ii], d_qp->d_lg+ii, 0, s_qp->d_lg+ii, 0);
		m_cvt_d2blasfeo_svec(ng[ii], d_qp->d_ug+ii, 0, s_qp->d_ug+ii, 0);
		for(jj=0; jj<nb[ii]; jj++) s_qp->idxb[ii][jj] = d_qp->idxb[ii][jj];
		}
	ii = N;
	m_cvt_d2blasfeo_smat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], d_qp->RSQrq+ii, 0, 0, s_qp->RSQrq+ii, 0, 0);
	m_cvt_d2blasfeo_smat(nu[ii]+nx[ii], ng[ii], d_qp->DCt+ii, 0, 0, s_qp->DCt+ii, 0, 0);
	m_cvt_d2blasfeo_svec(nu[ii]+nx[ii], d_qp->rq+ii, 0, s_qp->rq+ii, 0);
	m_cvt_d2blasfeo_svec(nb[ii], d_qp->d_lb+ii, 0, s_qp->d_lb+ii, 0);
	m_cvt_d2blasfeo_svec(nb[ii], d_qp->d_ub+ii, 0, s_qp->d_ub+ii, 0);
	m_cvt_d2blasfeo_svec(ng[ii], d_qp->d_lg+ii, 0, s_qp->d_lg+ii, 0);
	m_cvt_d2blasfeo_svec(ng[ii], d_qp->d_ug+ii, 0, s_qp->d_ug+ii, 0);
	for(jj=0; jj<nb[ii]; jj++) s_qp->idxb[ii][jj] = d_qp->idxb[ii][jj];

	return;

	}
