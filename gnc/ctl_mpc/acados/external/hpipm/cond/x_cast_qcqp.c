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

void CAST_QCQP_COMPUTE_DIM(struct OCP_QCQP_DIM *ocp_dim, struct DENSE_QCQP_DIM *dense_dim)
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
	nec = 0;
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
		nvc += nx[ii]+nu[ii];
		nec += nx[ii];
		nbc += nbx[ii]+nbu[ii];
		ngc += ng[ii];
		nqc += nq[ii];
		nsc += ns[ii];
		nsbc += nsbx[ii]+nsbu[ii];
		nsgc += nsg[ii];
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



void CAST_QCQP_COND(struct OCP_QCQP *ocp_qp, struct DENSE_QCQP *dense_qp)
	{

	int ii, jj;

	int N = ocp_qp->dim->N;
	int *nu = ocp_qp->dim->nu;
	int *nx = ocp_qp->dim->nx;
	int *nb = ocp_qp->dim->nb;
	int *ng = ocp_qp->dim->ng;
	int *nq = ocp_qp->dim->nq;

//	int nvc = dense_qp->dim->nv;
//	int nec = dense_qp->dim->ne;
	int nbc = dense_qp->dim->nb;
	int ngc = dense_qp->dim->ng;
	int nqc = dense_qp->dim->nq;

	int idxc, idxr, idxq;

	// cost
	idxr = 0;
	idxc = 0;
	for(ii=0; ii<=N; ii++)
		{
		GECP(nu[ii]+nx[ii], nu[ii]+nx[ii], ocp_qp->RSQrq+ii, 0, 0, dense_qp->Hv, idxr, idxc);
		VECCP(nu[ii]+nx[ii], ocp_qp->rqz+ii, 0, dense_qp->gz, idxr);
		idxr += nu[ii]+nx[ii];
		idxc += nu[ii]+nx[ii];
		}

	// dynamics
	idxr = 0;
	idxc = 0;
	for(ii=0; ii<N; ii++)
		{
		GETR(nu[ii]+nx[ii], nx[ii+1], ocp_qp->BAbt+ii, 0, 0, dense_qp->A, idxr, idxc);
		DIARE(nx[ii+1], -1.0, dense_qp->A, idxr, idxc+nu[ii]+nx[ii]+nu[ii+1]);
		VECCP(nx[ii+1], ocp_qp->b+ii, 0, dense_qp->b, idxr);
		idxr += nx[ii+1];
		idxc += nu[ii]+nx[ii];
		}

	// box constraints
	idxr = 0;
	idxc = 0;
	for(ii=0; ii<=N; ii++)
		{
		VECCP(nb[ii], ocp_qp->d+ii, 0, dense_qp->d, idxr);
		VECCP(nb[ii], ocp_qp->d+ii, nb[ii]+ng[ii]+nq[ii], dense_qp->d, idxr+nbc+ngc+nqc);
		for(jj=0; jj<nb[ii]; jj++)
			dense_qp->idxb[idxr+jj] = ocp_qp->idxb[ii][jj] + idxc;
		idxr += nb[ii];
		idxc += nu[ii]+nx[ii];
		}

	// general constraints
	idxr = 0;
	idxc = 0;
	for(ii=0; ii<=N; ii++)
		{
		VECCP(ng[ii], ocp_qp->d+ii, nb[ii], dense_qp->d, idxr+nbc);
		VECCP(ng[ii], ocp_qp->d+ii, 2*nb[ii]+ng[ii]+nq[ii], dense_qp->d, idxr+2*nbc+ngc+nqc);
		GECP(nu[ii]+nx[ii], ng[ii], ocp_qp->DCt+ii, 0, 0, dense_qp->Ct, idxc, idxr);
		idxr += ng[ii];
		idxc += nu[ii]+nx[ii];
		}

	// quadratic constraints
	idxr = 0;
	idxc = 0;
	idxq = 0;
	for(ii=0; ii<=N; ii++)
		{
		for(jj=0; jj<nq[ii]; jj++)
			{
			GECP(nu[ii]+nx[ii], nu[ii]+nx[ii], ocp_qp->Hq[ii]+jj, 0, 0, dense_qp->Hq+idxq, idxr, idxc);
			idxq++;
			}
		idxr += nu[ii]+nx[ii];
		idxc += nu[ii]+nx[ii];
		}
	idxr = 0;
	idxc = 0;
	for(ii=0; ii<=N; ii++)
		{
		GECP(nu[ii]+nx[ii], nq[ii], ocp_qp->DCt+ii, 0, ng[ii], dense_qp->Ct, idxr, ngc+idxc);
		idxr += nu[ii]+nx[ii];
		idxc += nq[ii];
		}
	idxr = 0;
	idxc = 0;
	for(ii=0; ii<=N; ii++)
		{
//		VECCP(nb[ii], ocp_qp->d+ii, 0, dense_qp->d, idxr);
		VECCP(nq[ii], ocp_qp->d+ii, 2*nb[ii]+2*ng[ii]+nq[ii], dense_qp->d, idxr+2*nbc+2*ngc+nqc);
		idxr += nq[ii];
		}

	// TODO soft constraints

	return;

	}




