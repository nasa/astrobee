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



#include <stdlib.h>
#include <stdio.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_m_aux.h>

#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_s_dense_qp_dim.h>
#include <hpipm_s_dense_qp.h>



void cvt_d2s_dense_qp(struct d_dense_qp *qpd, struct s_dense_qp *qps)
	{

	int ii;

	int nv = qpd->dim->nv;
	int ne = qpd->dim->ne;
	int nb = qpd->dim->nb;
	int ng = qpd->dim->ng;
	int ns = qpd->dim->ns;

	blasfeo_cvt_d2s_mat(nv, nv, qpd->Hv, 0, 0, qps->Hv, 0, 0);
	blasfeo_cvt_d2s_vec(2*ns, qpd->Z, 0, qps->Z, 0);
	blasfeo_cvt_d2s_vec(nv+2*ns, qpd->gz, 0, qps->gz, 0);
	blasfeo_cvt_d2s_mat(ne, nv, qpd->A, 0, 0, qps->A, 0, 0);
	blasfeo_cvt_d2s_vec(ne, qpd->b, 0, qps->b, 0);
	blasfeo_cvt_d2s_mat(nv, ng, qpd->Ct, 0, 0, qps->Ct, 0, 0);
	blasfeo_cvt_d2s_vec(2*nb+2*ng+2*ns, qpd->d, 0, qps->d, 0);
	blasfeo_cvt_d2s_vec(2*nb+2*ng+2*ns, qpd->m, 0, qps->m, 0);
	for(ii=0; ii<nb; ii++) qps->idxb[ii] = qpd->idxb[ii];
	for(ii=0; ii<nb+ng; ii++) qps->idxs_rev[ii] = qpd->idxs_rev[ii];

	return;

	}
