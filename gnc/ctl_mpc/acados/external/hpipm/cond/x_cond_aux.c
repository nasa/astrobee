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

void COND_BABT(struct OCP_QP *ocp_qp, struct STRMAT *BAbt2, struct STRVEC *b2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;

	// early return
	if(N<0)
		return;
	
	if(N==0 & cond_arg->cond_last_stage==1)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRVEC *Gammab = cond_ws->Gammab;

	int ii, jj;

	int nu_tmp;

	nu_tmp = 0;
	ii = 0;
	// B & A & b
	GECP(nu[0]+nx[0], nx[1], &BAbt[0], 0, 0, &Gamma[0], 0, 0);
	ROWIN(nx[1], 1.0, &b[0], 0, &Gamma[0], nu[0]+nx[0], 0);
	// b
	VECCP(nx[1], &b[0], 0, &Gammab[0], 0);

	nu_tmp += nu[0];
	ii++;

	for(ii=1; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// Gamma * A^T
		GEMM_NN(nu_tmp+nx[0]+1, nx[ii+1], nx[ii], 1.0, &Gamma[ii-1], 0, 0, &BAbt[ii], nu[ii], 0, 0.0, &Gamma[ii], nu[ii], 0, &Gamma[ii], nu[ii], 0); // Gamma * A^T

		GECP(nu[ii], nx[ii+1], &BAbt[ii], 0, 0, &Gamma[ii], 0, 0);

		nu_tmp += nu[ii];

		ROWAD(nx[ii+1], 1.0, &b[ii], 0, &Gamma[ii], nu_tmp+nx[0], 0);

		ROWEX(nx[ii+1], 1.0, &Gamma[ii], nu_tmp+nx[0], 0, &Gammab[ii], 0);
		}
	
	if(cond_arg->cond_last_stage==0)
		{
		// B & A
		GECP(nu_tmp+nx[0], nx[N], &Gamma[N-1], 0, 0, &BAbt2[0], 0, 0);
		// b
		ROWEX(nx[N], 1.0, &Gamma[N-1], nu_tmp+nx[0], 0, &b2[0], 0);
		}

	return;

	}



void COND_BAT(struct OCP_QP *ocp_qp, struct STRMAT *BAbt2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;

	// early return
	if(N<0)
		return;
	
	if(N==0 & cond_arg->cond_last_stage==1)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	struct STRMAT *BAbt = ocp_qp->BAbt;

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;

	int ii, jj;

	int nu_tmp;

	nu_tmp = 0;
	ii = 0;
	// B & A
	GECP(nu[0]+nx[0], nx[1], &BAbt[0], 0, 0, &Gamma[0], 0, 0);

	nu_tmp += nu[0];
	ii++;

	for(ii=1; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// Gamma * A^T
		GEMM_NN(nu_tmp+nx[0], nx[ii+1], nx[ii], 1.0, &Gamma[ii-1], 0, 0, &BAbt[ii], nu[ii], 0, 0.0, &Gamma[ii], nu[ii], 0, &Gamma[ii], nu[ii], 0); // Gamma * A^T

		GECP(nu[ii], nx[ii+1], &BAbt[ii], 0, 0, &Gamma[ii], 0, 0);

		nu_tmp += nu[ii];
		}
	
	if(cond_arg->cond_last_stage==0)
		{
		// B & A
		GECP(nu_tmp+nx[0], nx[N], &Gamma[N-1], 0, 0, &BAbt2[0], 0, 0);
		}

	return;

	}



void COND_B(struct OCP_QP *ocp_qp, struct STRVEC *b2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;

	// early return
	if(N<0)
		return;

	if(N==0 & cond_arg->cond_last_stage==1)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;

	// extract memory members
	struct STRVEC *Gammab = cond_ws->Gammab;

	int ii, jj;

	ii = 0;
	// b
	VECCP(nx[1], b+0, 0, Gammab+0, 0);

	ii++;

	for(ii=1; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// A * Gammab
		GEMV_T(nx[ii], nx[ii+1], 1.0, BAbt+ii, nu[ii], 0, Gammab+(ii-1), 0, 1.0, b+ii, 0, Gammab+ii, 0);
		}
	
	if(cond_arg->cond_last_stage==0)
		{
		// b
		VECCP(nx[N], Gammab+(N-1), 0, b2+0, 0);
		}

	return;

	}



void COND_RSQRQ(struct OCP_QP *ocp_qp, struct STRMAT *RSQrq2, struct STRVEC *rqz2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;

	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;
	struct STRMAT *RSQrq = ocp_qp->RSQrq;
	struct STRVEC *rqz = ocp_qp->rqz;
	int *diag_H_flag = ocp_qp->diag_H_flag;

	// early return
	if(N==0)
		{
		GECP(nu[0]+nx[0], nu[0]+nx[0], &RSQrq[0], 0, 0, &RSQrq2[0], 0, 0);
		VECCP(nu[0]+nx[0], &rqz[0], 0, &rqz2[0], 0);
		return;
		}

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRMAT *L = cond_ws->L;
	struct STRMAT *Lx = cond_ws->Lx;
	struct STRMAT *AL = cond_ws->AL;
	struct STRMAT *GammaQ = cond_ws->GammaQ;
	struct STRVEC *tmp_nuxM = cond_ws->tmp_nuxM;

	int nn;

	int nu2 = 0; // sum of all nu
	for(nn=0; nn<=N; nn++)
		nu2 += nu[nn];
	
	int nub = nu2; // backward partial sum
	int nuf = 0; // forward partial sum

	if(cond_arg->cond_alg==0) // N2 nx3
		{
		if(cond_arg->square_root_alg)
			{

			// final stage
			nub -= nu[N];

			GECP(nu[N]+nx[N], nu[N]+nx[N], &RSQrq[N], 0, 0, &L[N], 0, 0);
			ROWIN(nu[N]+nx[N], 1.0, &rqz[N], 0, &L[N], nu[N]+nx[N], 0);

			// D
			TRCP_L(nu[N], &L[N], 0, 0, &RSQrq2[0], nuf, nuf);

			GEMM_NN(nub+nx[0]+1, nu[N], nx[N], 1.0, &Gamma[N-1], 0, 0, &L[N], nu[N], 0, 0.0, &RSQrq2[0], nuf+nu[N], nuf, &RSQrq2[0], nuf+nu[N], nuf);

			// m
			GEAD(1, nu[N], 1.0, &L[N], nu[N]+nx[N], 0, &RSQrq2[0], nu2+nx[0], nuf);

			nuf += nu[N];


			// middle stages
			for(nn=0; nn<N-1; nn++)
				{
				nub -= nu[N-nn-1];

#if defined(LA_HIGH_PERFORMANCE)
				GECP(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);

				POTRF_L_MN(nx[N-nn]+1, nx[N-nn], Lx, 0, 0, Lx, 0, 0);
#else
				POTRF_L_MN(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
#endif
				ROWIN(nx[N-nn], 1.0, &b[N-nn-1], 0, &BAbt[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				TRMM_RLNN(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, Lx, 0, 0, &BAbt[N-nn-1], 0, 0, AL, 0, 0);
				GEAD(1, nx[N-nn], 1.0, Lx, nx[N-nn], 0, AL, nu[N-nn-1]+nx[N-nn-1], 0);

				ROWIN(nu[N-nn-1]+nx[N-nn-1], 1.0, &rqz[N-nn-1], 0, &RSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, AL, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

				// D
				TRCP_L(nu[N-nn-1], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf);

				GEMM_NN(nub+nx[0]+1, nu[N-nn-1], nx[N-nn-1], 1.0, &Gamma[N-nn-2], 0, 0, &L[N-nn-1], nu[N-nn-1], 0, 0.0, &RSQrq2[0], nuf+nu[N-nn-1], nuf, &RSQrq2[0], nuf+nu[N-nn-1], nuf);

				// m
				GEAD(1, nu[N-nn-1], 1.0, &L[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0, &RSQrq2[0], nu2+nx[0], nuf);

				nuf += nu[N-nn-1];

				}

			// first stage
			nn = N-1;

#if defined(LA_HIGH_PERFORMANCE)
			GECP(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);

			POTRF_L_MN(nx[N-nn]+1, nx[N-nn], Lx, 0, 0, Lx, 0, 0);
#else
			POTRF_L_MN(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
#endif
			ROWIN(nx[N-nn], 1.0, &b[N-nn-1], 0, &BAbt[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
			TRMM_RLNN(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, Lx, 0, 0, &BAbt[N-nn-1], 0, 0, AL, 0, 0);
			GEAD(1, nx[N-nn], 1.0, Lx, nx[N-nn], 0, AL, nu[N-nn-1]+nx[N-nn-1], 0);

			ROWIN(nu[N-nn-1]+nx[N-nn-1], 1.0, &rqz[N-nn-1], 0, &RSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
			SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, AL, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

			// D, M, m, P, p
		//	GECP(nu[0]+nx[0]+1, nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf); // TODO dtrcp for 'rectangular' matrices
			TRCP_L(nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf); // TODO dtrcp for 'rectangular' matrices
			GECP(1, nu[0]+nx[0], &L[N-nn-1], nu[0]+nx[0], 0, &RSQrq2[0], nuf+nu[0]+nx[0], nuf); // TODO dtrcp for 'rectangular' matrices
			// m p
			ROWEX(nu2+nx[0], 1.0, &RSQrq2[0], nu2+nx[0], 0, &rqz2[0], 0);

			}
		else
			{

			// final stage
			nub -= nu[N];

			GECP(nu[N]+nx[N], nu[N]+nx[N], &RSQrq[N], 0, 0, &L[N], 0, 0);
			ROWIN(nu[N]+nx[N], 1.0, &rqz[N], 0, &L[N], nu[N]+nx[N], 0);

			// D
			TRCP_L(nu[N], &L[N], 0, 0, &RSQrq2[0], nuf, nuf);

			GEMM_NN(nub+nx[0]+1, nu[N], nx[N], 1.0, &Gamma[N-1], 0, 0, &L[N], nu[N], 0, 0.0, &RSQrq2[0], nuf+nu[N], nuf, &RSQrq2[0], nuf+nu[N], nuf);

			// m
			GEAD(1, nu[N], 1.0, &L[N], nu[N]+nx[N], 0, &RSQrq2[0], nu2+nx[0], nuf);

			nuf += nu[N];


			// middle stages
			for(nn=0; nn<N-1; nn++)
				{
				nub -= nu[N-nn-1];

				TRCP_L(nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
				TRTR_L(nx[N-nn], Lx, 0, 0, Lx, 0, 0);
				ROWIN(nx[N-nn], 1.0, &b[N-nn-1], 0, &BAbt[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				GEMM_NT(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], nx[N-nn], 1.0, &BAbt[N-nn-1], 0, 0, Lx, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm
				GEAD(1, nx[N-nn], 1.0, L+N-nn, nu[N-nn]+nx[N-nn], nu[N-nn], AL, nu[N-nn-1]+nx[N-nn-1], 0);

				ROWIN(nu[N-nn-1]+nx[N-nn-1], 1.0, &rqz[N-nn-1], 0, &RSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, BAbt+N-nn-1, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

				// D
				TRCP_L(nu[N-nn-1], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf);

				GEMM_NN(nub+nx[0]+1, nu[N-nn-1], nx[N-nn-1], 1.0, &Gamma[N-nn-2], 0, 0, &L[N-nn-1], nu[N-nn-1], 0, 0.0, &RSQrq2[0], nuf+nu[N-nn-1], nuf, &RSQrq2[0], nuf+nu[N-nn-1], nuf);

				// m
				GEAD(1, nu[N-nn-1], 1.0, &L[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0, &RSQrq2[0], nu2+nx[0], nuf);

				nuf += nu[N-nn-1];

				}

			// first stage
			nn = N-1;

			TRCP_L(nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
			TRTR_L(nx[N-nn], Lx, 0, 0, Lx, 0, 0);
			ROWIN(nx[N-nn], 1.0, &b[N-nn-1], 0, &BAbt[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
			GEMM_NT(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], nx[N-nn], 1.0, &BAbt[N-nn-1], 0, 0, Lx, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm
			GEAD(1, nx[N-nn], 1.0, L+N-nn, nu[N-nn]+nx[N-nn], nu[N-nn], AL, nu[N-nn-1]+nx[N-nn-1], 0);

			ROWIN(nu[N-nn-1]+nx[N-nn-1], 1.0, &rqz[N-nn-1], 0, &RSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
			SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, BAbt+N-nn-1, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

			// D, M, m, P, p
		//	GECP(nu[0]+nx[0]+1, nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf); // TODO dtrcp for 'rectangular' matrices
			TRCP_L(nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf); // TODO dtrcp for 'rectangular' matrices
			GECP(1, nu[0]+nx[0], &L[N-nn-1], nu[0]+nx[0], 0, &RSQrq2[0], nuf+nu[0]+nx[0], nuf); // TODO dtrcp for 'rectangular' matrices
			// m p
			ROWEX(nu2+nx[0], 1.0, &RSQrq2[0], nu2+nx[0], 0, &rqz2[0], 0);

			}
		}
	else // cond_alg==1, N3 nx2
		{

//		nuf = 0;
//		nub = nu2;

		// TODO merge with first stage
		GESE(nu2+nx[0]+1, nu2+nx[0], 0.0, RSQrq2, 0, 0);

		for(nn=0; nn<=N; nn++)
			{

			nub -= nu[nn];

			if(nn==0)
				{
				TRCP_L(nu[0]+nx[0], RSQrq+0, 0, 0, RSQrq2, nub, nub);
				ROWAD(nu[0]+nx[0], 1.0, rqz+0, 0, RSQrq2, nu2+nx[0], nub);
				}
			else
				{

				// if Q is not zero
				if(1)
					{
					// if Q is diagonal
					if(diag_H_flag[nn]!=0)
						{
						DIAEX(nx[nn], 1.0, RSQrq+nn, nu[nn], nu[nn], tmp_nuxM, 0);
						GEMM_ND(nuf+nx[0]+1, nx[nn], 1.0, Gamma+nn-1, 0, 0, tmp_nuxM, 0, 0.0, GammaQ, 0, 0, GammaQ, 0, 0);
						}
					else
						{
						// XXX make Q full or use SYMM
						TRTR_L(nu[nn]+nx[nn], RSQrq+nn, 0, 0, RSQrq+nn, 0, 0);
						GEMM_NN(nuf+nx[0]+1, nx[nn], nx[nn], 1.0, Gamma+nn-1, 0, 0, RSQrq+nn, nu[nn], nu[nn], 0.0, GammaQ, 0, 0, GammaQ, 0, 0);
						}
					ROWAD(nx[nn], 1.0, rqz+nn, nu[nn], GammaQ, nuf+nx[0], 0);
					SYRK_LN_MN(nuf+nx[0]+1, nuf+nx[0], nx[nn], 1.0, GammaQ, 0, 0, Gamma+nn-1, 0, 0, 1.0, RSQrq2, nub+nu[nn], nub+nu[nn], RSQrq2, nub+nu[nn], nub+nu[nn]);
					}
				else
					{
					GEMV_N(nuf+nx[0], nx[nn], 1.0, Gamma+nn-1, 0, 0, rqz+nn, nu[nn], 1.0, rqz2, nub+nu[nn], rqz2, nub+nu[nn]);
					}

				// if R is not zero
				if(1)
					{
					GEAD(nu[nn], nu[nn], 1.0, RSQrq+nn, 0, 0, RSQrq2, nub, nub);
					}

				// if S is not zero
				if(diag_H_flag[nn]==0)
					{
					GEMM_NN(nuf+nx[0]+1, nu[nn], nx[nn], 1.0, Gamma+nn-1, 0, 0, RSQrq+nn, nu[nn], 0, 1.0, RSQrq2, nub+nu[nn], nub, RSQrq2, nub+nu[nn], nub);
					}

				ROWAD(nu[nn], 1.0, rqz+nn, 0, RSQrq2, nu2+nx[0], nub);
				ROWEX(nuf+nx[0], 1.0, RSQrq2, nu2+nx[0], nub+nu[nn], rqz2, 0);

				}

			nuf += nu[nn];

			}

		}

	return;

	}



void COND_RSQ(struct OCP_QP *ocp_qp, struct STRMAT *RSQrq2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;

	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRMAT *RSQrq = ocp_qp->RSQrq;
	int *diag_H_flag = ocp_qp->diag_H_flag;

	// early return
	if(N==0)
		{
		GECP(nu[0]+nx[0], nu[0]+nx[0], &RSQrq[0], 0, 0, &RSQrq2[0], 0, 0);
		return;
		}

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRMAT *L = cond_ws->L;
	struct STRMAT *Lx = cond_ws->Lx;
	struct STRMAT *AL = cond_ws->AL;
	struct STRMAT *GammaQ = cond_ws->GammaQ;
	struct STRVEC *tmp_nuxM = cond_ws->tmp_nuxM;

	int nn;

	int nu2 = 0; // sum of all nu
	for(nn=0; nn<=N; nn++)
		nu2 += nu[nn];
	
	int nub = nu2; // backward partial sum
	int nuf = 0; // forward partial sum

	if(cond_arg->cond_alg==0) // N2 nx3
		{
		if(cond_arg->square_root_alg)
			{

			// final stage
			nub -= nu[N];

			GECP(nu[N]+nx[N], nu[N]+nx[N], &RSQrq[N], 0, 0, &L[N], 0, 0);

			// D
			TRCP_L(nu[N], &L[N], 0, 0, &RSQrq2[0], nuf, nuf);

			GEMM_NN(nub+nx[0], nu[N], nx[N], 1.0, &Gamma[N-1], 0, 0, &L[N], nu[N], 0, 0.0, &RSQrq2[0], nuf+nu[N], nuf, &RSQrq2[0], nuf+nu[N], nuf);

			nuf += nu[N];


			// middle stages
			for(nn=0; nn<N-1; nn++)
				{
				nub -= nu[N-nn-1];

#if defined(LA_HIGH_PERFORMANCE)
				GECP(nx[N-nn], nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);

				POTRF_L_MN(nx[N-nn], nx[N-nn], Lx, 0, 0, Lx, 0, 0);
#else
				POTRF_L_MN(nx[N-nn], nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
#endif
				TRMM_RLNN(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, Lx, 0, 0, &BAbt[N-nn-1], 0, 0, AL, 0, 0);

				SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, AL, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

				// D
				TRCP_L(nu[N-nn-1], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf);

				GEMM_NN(nub+nx[0]+1, nu[N-nn-1], nx[N-nn-1], 1.0, &Gamma[N-nn-2], 0, 0, &L[N-nn-1], nu[N-nn-1], 0, 0.0, &RSQrq2[0], nuf+nu[N-nn-1], nuf, &RSQrq2[0], nuf+nu[N-nn-1], nuf);

				nuf += nu[N-nn-1];

				}

			// first stage
			nn = N-1;

#if defined(LA_HIGH_PERFORMANCE)
			GECP(nx[N-nn], nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);

			POTRF_L_MN(nx[N-nn], nx[N-nn], Lx, 0, 0, Lx, 0, 0);
#else
			POTRF_L_MN(nx[N-nn], nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
#endif
			TRMM_RLNN(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, Lx, 0, 0, &BAbt[N-nn-1], 0, 0, AL, 0, 0);

			SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, AL, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

			// D, M, m, P, p
			TRCP_L(nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf);

			}
		else
			{

			// final stage
			nub -= nu[N];

			GECP(nu[N]+nx[N], nu[N]+nx[N], &RSQrq[N], 0, 0, &L[N], 0, 0);

			// D
			TRCP_L(nu[N], &L[N], 0, 0, &RSQrq2[0], nuf, nuf);

			GEMM_NN(nub+nx[0], nu[N], nx[N], 1.0, &Gamma[N-1], 0, 0, &L[N], nu[N], 0, 0.0, &RSQrq2[0], nuf+nu[N], nuf, &RSQrq2[0], nuf+nu[N], nuf);

			nuf += nu[N];


			// middle stages
			for(nn=0; nn<N-1; nn++)
				{
				nub -= nu[N-nn-1];

				TRCP_L(nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
				TRTR_L(nx[N-nn], Lx, 0, 0, Lx, 0, 0);
				GEMM_NT(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], nx[N-nn], 1.0, &BAbt[N-nn-1], 0, 0, Lx, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm and reference

				SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, BAbt+N-nn-1, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

				// D
				TRCP_L(nu[N-nn-1], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf);

				GEMM_NN(nub+nx[0], nu[N-nn-1], nx[N-nn-1], 1.0, &Gamma[N-nn-2], 0, 0, &L[N-nn-1], nu[N-nn-1], 0, 0.0, &RSQrq2[0], nuf+nu[N-nn-1], nuf, &RSQrq2[0], nuf+nu[N-nn-1], nuf);

				nuf += nu[N-nn-1];

				}

			// first stage
			nn = N-1;

			TRCP_L(nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
			TRTR_L(nx[N-nn], Lx, 0, 0, Lx, 0, 0);
			GEMM_NT(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], nx[N-nn], 1.0, &BAbt[N-nn-1], 0, 0, Lx, 0, 0, 0.0, AL, 0, 0, AL, 0, 0); // TODO symm and reference

			SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, BAbt+N-nn-1, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

			// D, M, m, P, p
			TRCP_L(nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf);

			}
		}
	else // cond_alg==1, N3 nx2
		{

//		nuf = 0;
//		nub = nu2;

		// TODO merge with first stage
		GESE(nu2+nx[0], nu2+nx[0], 0.0, RSQrq2, 0, 0);

		for(nn=0; nn<=N; nn++)
			{

			nub -= nu[nn];

			if(nn==0)
				{
				TRCP_L(nu[0]+nx[0], RSQrq+0, 0, 0, RSQrq2, nub, nub);
				}
			else
				{

				// if Q is not zero
				if(1)
					{
					// if Q is diagonal
					if(diag_H_flag[nn]!=0)
						{
						DIAEX(nx[nn], 1.0, RSQrq+nn, nu[nn], nu[nn], tmp_nuxM, 0);
						GEMM_ND(nuf+nx[0], nx[nn], 1.0, Gamma+nn-1, 0, 0, tmp_nuxM, 0, 0.0, GammaQ, 0, 0, GammaQ, 0, 0);
						}
					else
						{
						// XXX make Q full or use SYMM and reference
						TRTR_L(nu[nn]+nx[nn], RSQrq+nn, 0, 0, RSQrq+nn, 0, 0);
						GEMM_NN(nuf+nx[0], nx[nn], nx[nn], 1.0, Gamma+nn-1, 0, 0, RSQrq+nn, nu[nn], nu[nn], 0.0, GammaQ, 0, 0, GammaQ, 0, 0);
						}
					SYRK_LN_MN(nuf+nx[0], nuf+nx[0], nx[nn], 1.0, GammaQ, 0, 0, Gamma+nn-1, 0, 0, 1.0, RSQrq2, nub+nu[nn], nub+nu[nn], RSQrq2, nub+nu[nn], nub+nu[nn]);
					}

				// if R is not zero
				if(1)
					{
					GEAD(nu[nn], nu[nn], 1.0, RSQrq+nn, 0, 0, RSQrq2, nub, nub);
					}

				// if S is not zero
				if(diag_H_flag[nn]==0)
					{
					GEMM_NN(nuf+nx[0], nu[nn], nx[nn], 1.0, Gamma+nn-1, 0, 0, RSQrq+nn, nu[nn], 0, 1.0, RSQrq2, nub+nu[nn], nub, RSQrq2, nub+nu[nn], nub);
					}

				}

			nuf += nu[nn];

			}

		}

	return;

	}



void COND_RQ(struct OCP_QP *ocp_qp, struct STRVEC *rqz2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;

	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;
	struct STRMAT *RSQrq = ocp_qp->RSQrq;
	struct STRVEC *rqz = ocp_qp->rqz;
	int *diag_H_flag = ocp_qp->diag_H_flag;

	// early return
	if(N==0)
		{
		VECCP(nu[0]+nx[0], rqz+0, 0, rqz2+0, 0);
		return;
		}

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRMAT *L = cond_ws->L;
	struct STRVEC *Gammab = cond_ws->Gammab;
	struct STRVEC *l = cond_ws->l;
	struct STRVEC *tmp_nuxM = cond_ws->tmp_nuxM;

	int nn;

	int nu2 = 0; // sum of all nu
	for(nn=0; nn<=N; nn++)
		nu2 += nu[nn];
	
	int nub = nu2; // backward partial sum
	int nuf = 0; // forward partial sum

	if(cond_arg->cond_alg==0) // N2 nx3
		{
		// final stage 
		nub -= nu[N];

		VECCP(nu[N]+nx[N], rqz+N, 0, l+N, 0);

		GEMV_T(nx[N], nu[N], 1.0, L+N, nu[N], 0, Gammab+(N-1), 0, 1.0, l+N, 0, rqz2+0, nuf);

		nuf += nu[N];


		// middle stages 
		for(nn=0; nn<N-1; nn++)
			{	
			nub -= nu[N-nn-1];

	//		SYMV_L(nx[N-nn], 1.0, L+(N-nn), nu[N-nn], nu[N-nn], b+(N-nn-1), 0, 1.0, l+(N-nn), nu[N-nn], tmp_nuxM, 0); // XXX buggy !!!
			TRTR_L(nx[N-nn]+nu[N-nn], L+N-nn, 0, 0, L+N-nn, 0, 0);
			GEMV_N(nx[N-nn], nx[N-nn], 1.0, L+(N-nn), nu[N-nn], nu[N-nn], b+(N-nn-1), 0, 1.0, l+(N-nn), nu[N-nn], tmp_nuxM, 0);

			GEMV_N(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, BAbt+(N-nn-1), 0, 0, tmp_nuxM, 0, 1.0, rqz+(N-nn-1), 0, l+(N-nn-1), 0);

			GEMV_T(nx[N-nn-1], nu[N-nn-1], 1.0, L+(N-nn-1), nu[N-nn-1], 0, Gammab+(N-nn-2), 0, 1.0, l+(N-nn-1), 0, rqz2+0, nuf);

			nuf += nu[N-nn-1];

			}

		// first stage
		nn = N-1;

	//	SYMV_L(nx[N-nn], 1.0, L+(N-nn), nu[N-nn], nu[N-nn], b+(N-nn-1), 0, 1.0, l+(N-nn), nu[N-nn], tmp_nuxM, 0);
		TRTR_L(nx[N-nn]+nu[N-nn], L+N-nn, 0, 0, L+N-nn, 0, 0);
		GEMV_N(nx[N-nn], nx[N-nn], 1.0, L+(N-nn), nu[N-nn], nu[N-nn], b+(N-nn-1), 0, 1.0, l+(N-nn), nu[N-nn], tmp_nuxM, 0);

		GEMV_N(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, BAbt+(N-nn-1), 0, 0, tmp_nuxM, 0, 1.0, rqz+(N-nn-1), 0, l+(N-nn-1), 0);

		// m p
		VECCP(nu[0]+nx[0], l+(N-nn-1), 0, rqz2+0, nuf);
		}
	else // cond_alg==1, N3 nx2
		{

//		nuf = 0;
//		nub = nu2;

		VECSE(nu2+nx[0], 0.0, rqz2, 0);

		for(nn=0; nn<=N; nn++)
			{

			nub -= nu[nn];

			if(nn==0)
				{
				VECCP(nu[0]+nx[0], rqz+0, 0, rqz2, nub);
				}
			else
				{

				// if Q is not zero
				if(1)
					{
					if(diag_H_flag[nn]!=0)
						{
						DIAEX(nx[nn], 1.0, RSQrq+nn, nu[nn], nu[nn], tmp_nuxM, 0);
						GEMV_D(nx[nn], 1.0, tmp_nuxM, 0, Gammab+nn-1, 0, 1.0, rqz+nn, nu[nn], tmp_nuxM, 0);
						}
					else
						{
						SYMV_L(nx[nn], 1.0, RSQrq+nn, nu[nn], nu[nn], Gammab+nn-1, 0, 1.0, rqz+nn, nu[nn], tmp_nuxM, 0);
						}
					GEMV_N(nuf+nx[0], nx[nn], 1.0, Gamma+nn-1, 0, 0, tmp_nuxM, 0, 1.0, rqz2, nub+nu[nn], rqz2, nub+nu[nn]);
					}
				else
					{
					GEMV_N(nuf+nx[0], nx[nn], 1.0, Gamma+nn-1, 0, 0, rqz+nn, nu[nn], 1.0, rqz2, nub+nu[nn], rqz2, nub+nu[nn]);
					}

				// if S is not zero
				if(diag_H_flag[nn]==0)
					{
					GEMV_T(nx[nn], nu[nn], 1.0, RSQrq+nn, nu[nn], 0, Gammab+nn-1, 0, 1.0, rqz+nn, 0, rqz2, nub);
					}
				else
					{
					AXPY(nu[nn], 1.0, rqz+nn, 0, rqz2, nub, rqz2, nub);
					}

				}

			nuf += nu[nn];

			}

		}

	return;

	}



void COND_DCTD(struct OCP_QP *ocp_qp, int *idxb2, struct STRMAT *DCt2, struct STRVEC *d2, struct STRVEC *d_mask2, int *idxs_rev2, struct STRVEC *Z2, struct STRVEC *rqz2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;
	
	int ii, jj, idx;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	int *nb = ocp_qp->dim->nb;
	int *ng = ocp_qp->dim->ng;
	int *ns = ocp_qp->dim->ns;

	int **idxb = ocp_qp->idxb;
	struct STRVEC *d = ocp_qp->d;
	struct STRVEC *d_mask = ocp_qp->d_mask;
	struct STRMAT *DCt = ocp_qp->DCt;
	int **idxs_rev = ocp_qp->idxs_rev;
	struct STRVEC *Z = ocp_qp->Z;
	struct STRVEC *rqz = ocp_qp->rqz;

	// early return
	if(N==0 & cond_arg->cond_last_stage==1)
		{
		VECCP(2*nb[0]+2*ng[0]+2*ns[0], ocp_qp->d, 0, d2, 0);
		VECCP(2*nb[0]+2*ng[0]+2*ns[0], ocp_qp->d_mask, 0, d_mask2, 0);
		GECP(nu[0]+nx[0], ng[0], ocp_qp->DCt, 0, 0, DCt2, 0, 0);
		for(ii=0; ii<nb[0]; ii++) idxb2[ii] = ocp_qp->idxb[0][ii];
		VECCP(2*ns[0], ocp_qp->Z, 0, Z2, 0);
		VECCP(2*ns[0], ocp_qp->rqz, nu[0]+nx[0], rqz2, nu[0]+nx[0]); // XXX rqz2 offset
		for(ii=0; ii<nb[0]+ng[0]; ii++) idxs_rev2[ii] = ocp_qp->idxs_rev[0][ii];
		return;
		}

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRVEC *Gammab = cond_ws->Gammab;
	struct STRVEC *tmp_nbgM = cond_ws->tmp_nbgM;


	REAL *ptr_d_lb;
	REAL *ptr_d_ub;
	REAL *ptr_d_ls;
	REAL *ptr_d_us;
	REAL *ptr_d_mask_lb;
	REAL *ptr_d_mask_ub;
	REAL *ptr_d_mask_ls;
	REAL *ptr_d_mask_us;

	int nu_tmp, ng_tmp;

	int nu0, nx0, nb0, ng0, ns0;

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(idxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ns2 = ns[0];
	int ngg = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ns2 += ns[ii];
		ngg += ng[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	REAL *d_lb3 = d2->pa+0;
	REAL *d_ub3 = d2->pa+nb2+ng2;
	REAL *d_lg3 = d2->pa+nb2;
	REAL *d_ug3 = d2->pa+2*nb2+ng2;
	REAL *d_ls3 = d2->pa+2*nb2+2*ng2;
	REAL *d_us3 = d2->pa+2*nb2+2*ng2+ns2;
	REAL *d_mask_lb3 = d_mask2->pa+0;
	REAL *d_mask_ub3 = d_mask2->pa+nb2+ng2;
	REAL *d_mask_lg3 = d_mask2->pa+nb2;
	REAL *d_mask_ug3 = d_mask2->pa+2*nb2+ng2;
	REAL *d_mask_ls3 = d_mask2->pa+2*nb2+2*ng2;
	REAL *d_mask_us3 = d_mask2->pa+2*nb2+2*ng2+ns2;

	// set constraint matrix to zero (it's 2 lower triangular matrices atm)
	GESE(nu2+nx2, ng2, 0.0, &DCt2[0], 0, 0);

	// box constraints

	int idx_gammab = nx[0];
	for(ii=0; ii<N; ii++)
		idx_gammab += nu[ii];

	int ib = 0;
	int ig = 0;
	int is = 0; // XXX

	int idxb0, idxg0;

	REAL *ptr_Z;
	REAL *ptr_z;
	REAL *ptr_Z2 = Z2->pa;
	REAL *ptr_z2 = rqz2->pa+nu2+nx2;

	REAL tmp;
	int idx_g;

	// middle stages
	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_Z = Z[N-ii].pa;
			ptr_z = rqz[N-ii].pa+nu0+nx0;
			}
		nu_tmp += nu0;
		ptr_d_lb = d[N-ii].pa+0;
		ptr_d_ub = d[N-ii].pa+nb0+ng0;
		ptr_d_ls = d[N-ii].pa+2*nb0+2*ng0;
		ptr_d_us = d[N-ii].pa+2*nb0+2*ng0+ns0;
		ptr_d_mask_lb = d_mask[N-ii].pa+0;
		ptr_d_mask_ub = d_mask[N-ii].pa+nb0+ng0;
		ptr_d_mask_ls = d_mask[N-ii].pa+2*nb0+2*ng0;
		ptr_d_mask_us = d_mask[N-ii].pa+2*nb0+2*ng0+ns0;
		for(jj=0; jj<nb0; jj++)
			{
			idxb0 = idxb[N-ii][jj];
			if(idxb0<nu0) // input: box constraint
				{
				d_lb3[ib] = ptr_d_lb[jj];
				d_ub3[ib] = ptr_d_ub[jj];
				d_mask_lb3[ib] = ptr_d_mask_lb[jj];
				d_mask_ub3[ib] = ptr_d_mask_ub[jj];
				idxb2[ib] = nu_tmp - nu0 + idxb[N-ii][jj];
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					idxs_rev2[ib] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					d_mask_ls3[0+is]    = ptr_d_mask_ls[0+idx];
					d_mask_us3[0+is]    = ptr_d_mask_us[0+idx];
					is++;
					}
				ib++;
				}
			else // state: general constraint
				{
				idx_g = idxb0-nu0;
				tmp = BLASFEO_DVECEL(&Gammab[N-1-ii], idx_g);
				d_lg3[ig] = ptr_d_lb[jj] - tmp;
//				d_ug3[ig] = ptr_d_ub[jj] - tmp;
				d_ug3[ig] = ptr_d_ub[jj] + tmp; // XXX
				d_mask_lg3[ig] = ptr_d_mask_lb[jj];
				d_mask_ug3[ig] = ptr_d_mask_ub[jj];
				GECP(idx_gammab, 1, &Gamma[N-ii-1], 0, idx_g, &DCt2[0], nu_tmp, ig);
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					idxs_rev2[nb2+ig] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					d_mask_ls3[0+is]    = ptr_d_mask_ls[0+idx];
					d_mask_us3[0+is]    = ptr_d_mask_us[0+idx];
					is++;
					}
				ig++;
				}
			}
		idx_gammab -= nu[N-1-ii];
		}

	// initial stage: both inputs and states as box constraints
	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_Z = Z[0].pa;
		ptr_z = rqz[0].pa+nu0+nx0;
		}
	nu_tmp += nu0;
	ptr_d_lb = d[0].pa+0;
	ptr_d_ub = d[0].pa+nb0+ng0;
	ptr_d_ls = d[0].pa+2*nb0+2*ng0;
	ptr_d_us = d[0].pa+2*nb0+2*ng0+ns0;
	ptr_d_mask_lb = d_mask[0].pa+0;
	ptr_d_mask_ub = d_mask[0].pa+nb0+ng0;
	ptr_d_mask_ls = d_mask[0].pa+2*nb0+2*ng0;
	ptr_d_mask_us = d_mask[0].pa+2*nb0+2*ng0+ns0;
	for(jj=0; jj<nb0; jj++)
		{
		idxb0 = idxb[0][jj];
		d_lb3[ib] = ptr_d_lb[jj];
		d_ub3[ib] = ptr_d_ub[jj];
		d_mask_lb3[ib] = ptr_d_mask_lb[jj];
		d_mask_ub3[ib] = ptr_d_mask_ub[jj];
		idxb2[ib] = nu_tmp - nu0 + idxb0;
		idx = idxs_rev[0][jj];
		if(idx>=0)
			{
			idxs_rev2[ib] = is;
			ptr_Z2[0+is]   = ptr_Z[0+idx];
			ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
			ptr_z2[0+is]   = ptr_z[0+idx];
			ptr_z2[ns2+is] = ptr_z[ns0+idx];
			d_ls3[0+is]    = ptr_d_ls[0+idx];
			d_us3[0+is]    = ptr_d_us[0+idx];
			d_mask_ls3[0+is] = ptr_d_mask_ls[0+idx];
			d_mask_us3[0+is] = ptr_d_mask_us[0+idx];
			is++;
			}
		ib++;
		}

	// XXX for now, just shift after box-to-general constraints
	// better interleave them, to keep the block lower trianlgular structure !!!

	// general constraints

	char *c_ptr;

	nu_tmp = 0;
	ng_tmp = 0;
	for(ii=0; ii<N; ii++)
		{

		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_Z = Z[N-ii].pa;
			ptr_z = rqz[N-ii].pa+nu0+nx0;
			}
		ptr_d_ls = d[N-ii].pa+2*nb0+2*ng0;
		ptr_d_us = d[N-ii].pa+2*nb0+2*ng0+ns0;
		ptr_d_mask_ls = d_mask[N-ii].pa+2*nb0+2*ng0;
		ptr_d_mask_us = d_mask[N-ii].pa+2*nb0+2*ng0+ns0;

		if(ng0>0)
			{
			for(ig=0; ig<ng0; ig++)
				{
				idx = idxs_rev[N-ii][nb0+ig];
				if(idx>=0)
					{
					idxs_rev2[nb2+nbg+ng_tmp+ig] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					d_mask_ls3[0+is] = ptr_d_mask_ls[0+idx];
					d_mask_us3[0+is] = ptr_d_mask_us[0+idx];
					is++;
					}
				}

			GECP(nu0, ng0, &DCt[N-ii], 0, 0, DCt2, nu_tmp, nbg+ng_tmp);

			nu_tmp += nu0;

			GEMM_NN(nu2+nx[0]-nu_tmp, ng0, nx0, 1.0, &Gamma[N-1-ii], 0, 0, &DCt[N-ii], nu0, 0, 0.0, DCt2, nu_tmp, nbg+ng_tmp, DCt2, nu_tmp, nbg+ng_tmp);

			VECCP(ng0, &d[N-ii], nb0, d2, nb2+nbg+ng_tmp);
			VECCP(ng0, &d[N-ii], 2*nb0+ng0, d2, 2*nb2+ng2+nbg+ng_tmp);
			VECCP(ng0, &d_mask[N-ii], nb0, d_mask2, nb2+nbg+ng_tmp);
			VECCP(ng0, &d_mask[N-ii], 2*nb0+ng0, d_mask2, 2*nb2+ng2+nbg+ng_tmp);

			GEMV_T(nx0, ng0, 1.0, &DCt[N-ii], nu0, 0, &Gammab[N-ii-1], 0, 0.0, tmp_nbgM, 0, tmp_nbgM, 0);

			AXPY(ng0, -1.0, tmp_nbgM, 0, d2, nb2+nbg+ng_tmp, d2, nb2+nbg+ng_tmp);
//			AXPY(ng0, -1.0, tmp_nbgM, 0, d2, 2*nb2+ng2+nbg+ng_tmp, d2, 2*nb2+ng2+nbg+ng_tmp);
			AXPY(ng0,  1.0, tmp_nbgM, 0, d2, 2*nb2+ng2+nbg+ng_tmp, d2, 2*nb2+ng2+nbg+ng_tmp); // XXX

			ng_tmp += ng0;
			
			}
		else
			{

			nu_tmp += nu0;

			}

		}

	ii = N;

	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_Z = Z[0].pa;
		ptr_z = rqz[0].pa+nu0+nx0;
		}
	ptr_d_ls = d[0].pa+2*nb0+2*ng0;
	ptr_d_us = d[0].pa+2*nb0+2*ng0+ns0;
	ptr_d_mask_ls = d_mask[0].pa+2*nb0+2*ng0;
	ptr_d_mask_us = d_mask[0].pa+2*nb0+2*ng0+ns0;

	if(ng0>0)
		{
		for(ig=0; ig<ng0; ig++)
			{
			idx = idxs_rev[0][nb0+ig];
			if(idx>=0)
				{
				idxs_rev2[nb2+nbg+ng_tmp+ig] = is;
				ptr_Z2[0+is]   = ptr_Z[0+idx];
				ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
				ptr_z2[0+is]   = ptr_z[0+idx];
				ptr_z2[ns2+is] = ptr_z[ns0+idx];
				d_ls3[0+is]    = ptr_d_ls[0+idx];
				d_us3[0+is]    = ptr_d_us[0+idx];
				d_mask_ls3[0+is] = ptr_d_mask_ls[0+idx];
				d_mask_us3[0+is] = ptr_d_mask_us[0+idx];
				is++;
				}

			}

		GECP(nu0+nx0, ng0, &DCt[0], 0, 0, DCt2, nu_tmp, nbg+ng_tmp);

		VECCP(ng0, &d[0], nb0, d2, nb2+nbg+ng_tmp);
		VECCP(ng0, &d[0], 2*nb0+ng0, d2, 2*nb2+ng2+nbg+ng_tmp);
		VECCP(ng0, &d_mask[0], nb0, d_mask2, nb2+nbg+ng_tmp);
		VECCP(ng0, &d_mask[0], 2*nb0+ng0, d_mask2, 2*nb2+ng2+nbg+ng_tmp);

//		ng_tmp += ng[N-ii];

		}

	return;

	}



void COND_DCT(struct OCP_QP *ocp_qp, int *idxb2, struct STRMAT *DCt2, int *idxs_rev2, struct STRVEC *Z2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;
	
	int ii, jj, idx;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	int *nb = ocp_qp->dim->nb;
	int *ng = ocp_qp->dim->ng;
	int *ns = ocp_qp->dim->ns;

	int **idxb = ocp_qp->idxb;
	struct STRMAT *DCt = ocp_qp->DCt;
	int **idxs_rev = ocp_qp->idxs_rev;
	struct STRVEC *Z = ocp_qp->Z;

	// early return
	if(N==0 & cond_arg->cond_last_stage==1)
		{
		GECP(nu[0]+nx[0], ng[0], ocp_qp->DCt, 0, 0, DCt2, 0, 0);
		for(ii=0; ii<nb[0]; ii++) idxb2[ii] = ocp_qp->idxb[0][ii];
		VECCP(2*ns[0], ocp_qp->Z, 0, Z2, 0);
		for(ii=0; ii<nb[0]+ng[0]; ii++) idxs_rev2[ii] = ocp_qp->idxs_rev[0][ii];
		return;
		}

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRVEC *tmp_nbgM = cond_ws->tmp_nbgM;


	REAL *ptr_d_lb;
	REAL *ptr_d_ub;
	REAL *ptr_d_ls;
	REAL *ptr_d_us;
	REAL *ptr_d_mask_lb;
	REAL *ptr_d_mask_ub;
	REAL *ptr_d_mask_ls;
	REAL *ptr_d_mask_us;

	int nu_tmp, ng_tmp;

	int nu0, nx0, nb0, ng0, ns0;

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(idxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ns2 = ns[0];
	int ngg = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ns2 += ns[ii];
		ngg += ng[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	// set constraint matrix to zero (it's 2 lower triangular matrices atm)
	GESE(nu2+nx2, ng2, 0.0, &DCt2[0], 0, 0);

	// box constraints

	int idx_gammab = nx[0];
	for(ii=0; ii<N; ii++)
		idx_gammab += nu[ii];

	int ib = 0;
	int ig = 0;
	int is = 0; // XXX

	int idxb0, idxg0;

	REAL *ptr_Z;
	REAL *ptr_Z2 = Z2->pa;

	REAL tmp;
	int idx_g;

	// middle stages
	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_Z = Z[N-ii].pa;
			}
		nu_tmp += nu0;
		for(jj=0; jj<nb0; jj++)
			{
			idxb0 = idxb[N-ii][jj];
			if(idxb0<nu0) // input: box constraint
				{
				idxb2[ib] = nu_tmp - nu0 + idxb[N-ii][jj];
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					idxs_rev2[ib] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					is++;
					}
				ib++;
				}
			else // state: general constraint
				{
				idx_g = idxb0-nu0;
				GECP(idx_gammab, 1, &Gamma[N-ii-1], 0, idx_g, &DCt2[0], nu_tmp, ig);
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					idxs_rev2[nb2+ig] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					is++;
					}
				ig++;
				}
			}
		idx_gammab -= nu[N-1-ii];
		}

	// initial stage: both inputs and states as box constraints
	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_Z = Z[0].pa;
		}
	nu_tmp += nu0;
	for(jj=0; jj<nb0; jj++)
		{
		idxb0 = idxb[0][jj];
		idxb2[ib] = nu_tmp - nu0 + idxb0;
		idx = idxs_rev[0][jj];
		if(idx>=0)
			{
			idxs_rev2[ib] = is;
			ptr_Z2[0+is]   = ptr_Z[0+idx];
			ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
			is++;
			}
		ib++;
		}

	// XXX for now, just shift after box-to-general constraints
	// better interleave them, to keep the block lower trianlgular structure !!!

	// general constraints

	char *c_ptr;

	nu_tmp = 0;
	ng_tmp = 0;
	for(ii=0; ii<N; ii++)
		{

		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_Z = Z[N-ii].pa;
			}

		if(ng0>0)
			{
			for(ig=0; ig<ng0; ig++)
				{
				idx = idxs_rev[N-ii][nb0+ig];
				if(idx>=0)
					{
					idxs_rev2[nb2+nbg+ng_tmp+ig] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					is++;
					}
				}

			GECP(nu0, ng0, &DCt[N-ii], 0, 0, DCt2, nu_tmp, nbg+ng_tmp);

			nu_tmp += nu0;

			GEMM_NN(nu2+nx[0]-nu_tmp, ng0, nx0, 1.0, &Gamma[N-1-ii], 0, 0, &DCt[N-ii], nu0, 0, 0.0, DCt2, nu_tmp, nbg+ng_tmp, DCt2, nu_tmp, nbg+ng_tmp);

			ng_tmp += ng0;
			
			}
		else
			{

			nu_tmp += nu0;

			}

		}

	ii = N;

	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_Z = Z[0].pa;
		}

	if(ng0>0)
		{
		for(ig=0; ig<ng0; ig++)
			{
			idx = idxs_rev[0][nb0+ig];
			if(idx>=0)
				{
				idxs_rev2[nb2+nbg+ng_tmp+ig] = is;
				ptr_Z2[0+is]   = ptr_Z[0+idx];
				ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
				is++;
				}

			}

		GECP(nu0+nx0, ng0, &DCt[0], 0, 0, DCt2, nu_tmp, nbg+ng_tmp);

//		ng_tmp += ng[N-ii];

		}

	return;

	}



void COND_D(struct OCP_QP *ocp_qp, struct STRVEC *d2, struct STRVEC *d_mask2, struct STRVEC *rqz2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;

	int ii, jj, idx;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	int *nb = ocp_qp->dim->nb;
	int *ng = ocp_qp->dim->ng;
	int *ns = ocp_qp->dim->ns;

	int **idxb = ocp_qp->idxb;
	struct STRVEC *d = ocp_qp->d;
	struct STRVEC *d_mask = ocp_qp->d_mask;
	struct STRMAT *DCt = ocp_qp->DCt;
	int **idxs_rev = ocp_qp->idxs_rev;
	struct STRVEC *Z = ocp_qp->Z;
	struct STRVEC *rqz = ocp_qp->rqz;

	// early return
	if(N==0 & cond_arg->cond_last_stage==1)
		{
		VECCP(2*nb[0]+2*ng[0]+2*ns[0], ocp_qp->d, 0, d2, 0);
		VECCP(2*nb[0]+2*ng[0]+2*ns[0], ocp_qp->d_mask, 0, d_mask2, 0);
		VECCP(2*ns[0], ocp_qp->rqz, nu[0]+nx[0], rqz2, nu[0]+nx[0]); // XXX rqz2 offset
		return;
		}

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRVEC *Gammab = cond_ws->Gammab;
	struct STRVEC *tmp_nbgM = cond_ws->tmp_nbgM;


	REAL *ptr_d_lb;
	REAL *ptr_d_ub;
	REAL *ptr_d_ls;
	REAL *ptr_d_us;
	REAL *ptr_d_mask_lb;
	REAL *ptr_d_mask_ub;
	REAL *ptr_d_mask_ls;
	REAL *ptr_d_mask_us;
	
	int nu_tmp, ng_tmp;

	int nu0, nx0, nb0, ng0, ns0;

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(idxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ns2 = ns[0];
	int ngg = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ns2 += ns[ii];
		ngg += ng[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	REAL *d_lb3 = d2->pa+0;
	REAL *d_ub3 = d2->pa+nb2+ng2;
	REAL *d_lg3 = d2->pa+nb2;
	REAL *d_ug3 = d2->pa+2*nb2+ng2;
	REAL *d_ls3 = d2->pa+2*nb2+2*ng2;
	REAL *d_us3 = d2->pa+2*nb2+2*ng2+ns2;
	REAL *d_mask_lb3 = d_mask2->pa+0;
	REAL *d_mask_ub3 = d_mask2->pa+nb2+ng2;
	REAL *d_mask_lg3 = d_mask2->pa+nb2;
	REAL *d_mask_ug3 = d_mask2->pa+2*nb2+ng2;
	REAL *d_mask_ls3 = d_mask2->pa+2*nb2+2*ng2;
	REAL *d_mask_us3 = d_mask2->pa+2*nb2+2*ng2+ns2;

	// box constraints

	int idx_gammab = nx[0];
	for(ii=0; ii<N; ii++)
		idx_gammab += nu[ii];

	int ib = 0;
	int ig = 0;
	int is = 0; // XXX

	int idxb0, idxg0;

	REAL *ptr_z;
	REAL *ptr_z2 = rqz2->pa+nu2+nx2;

	REAL tmp;
	int idx_g;

	// middle stages
	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_z = rqz[N-ii].pa+nu0+nx0;
			}
		nu_tmp += nu0;
		ptr_d_lb = d[N-ii].pa+0;
		ptr_d_ub = d[N-ii].pa+nb0+ng0;
		ptr_d_ls = d[N-ii].pa+2*nb0+2*ng0;
		ptr_d_us = d[N-ii].pa+2*nb0+2*ng0+ns0;
		ptr_d_mask_lb = d_mask[N-ii].pa+0;
		ptr_d_mask_ub = d_mask[N-ii].pa+nb0+ng0;
		ptr_d_mask_ls = d_mask[N-ii].pa+2*nb0+2*ng0;
		ptr_d_mask_us = d_mask[N-ii].pa+2*nb0+2*ng0+ns0;
		for(jj=0; jj<nb0; jj++)
			{
			idxb0 = idxb[N-ii][jj];
			if(idxb0<nu0) // input: box constraint
				{
				d_lb3[ib] = ptr_d_lb[jj];
				d_ub3[ib] = ptr_d_ub[jj];
				d_mask_lb3[ib] = ptr_d_mask_lb[jj];
				d_mask_ub3[ib] = ptr_d_mask_ub[jj];
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns[N-ii]+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					d_mask_ls3[0+is]    = ptr_d_mask_ls[0+idx];
					d_mask_us3[0+is]    = ptr_d_mask_us[0+idx];
					is++;
					}
				ib++;
				}
			else // state: general constraint
				{
				idx_g = idxb0-nu0;
				tmp = BLASFEO_DVECEL(&Gammab[N-1-ii], idx_g);
				d_lg3[ig] = ptr_d_lb[jj] - tmp;
//				d_ug3[ig] = ptr_d_ub[jj] - tmp;
				d_ug3[ig] = ptr_d_ub[jj] + tmp; // XXX
				d_mask_lg3[ig] = ptr_d_mask_lb[jj];
				d_mask_ug3[ig] = ptr_d_mask_ub[jj];
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					d_mask_ls3[0+is]    = ptr_d_mask_ls[0+idx];
					d_mask_us3[0+is]    = ptr_d_mask_us[0+idx];
					is++;
					}
				ig++;
				}
			}
		idx_gammab -= nu[N-1-ii];
		}

	// initial stage: both inputs and states as box constraints
	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_z = rqz[0].pa+nu0+nx0;
		}
	nu_tmp += nu0;
	ptr_d_lb = d[0].pa+0;
	ptr_d_ub = d[0].pa+nb0+ng0;
	ptr_d_ls = d[0].pa+2*nb0+2*ng0;
	ptr_d_us = d[0].pa+2*nb0+2*ng0+ns0;
	ptr_d_mask_lb = d_mask[0].pa+0;
	ptr_d_mask_ub = d_mask[0].pa+nb0+ng0;
	ptr_d_mask_ls = d_mask[0].pa+2*nb0+2*ng0;
	ptr_d_mask_us = d_mask[0].pa+2*nb0+2*ng0+ns0;
	for(jj=0; jj<nb0; jj++)
		{
		idxb0 = idxb[0][jj];
		d_lb3[ib] = ptr_d_lb[jj];
		d_ub3[ib] = ptr_d_ub[jj];
		d_mask_lb3[ib] = ptr_d_mask_lb[jj];
		d_mask_ub3[ib] = ptr_d_mask_ub[jj];
		idx = idxs_rev[0][jj];
		if(idx>=0)
			{
			ptr_z2[0+is]   = ptr_z[0+idx];
			ptr_z2[ns2+is] = ptr_z[ns0+idx];
			d_ls3[0+is]    = ptr_d_ls[0+idx];
			d_us3[0+is]    = ptr_d_us[0+idx];
			d_mask_ls3[0+is] = ptr_d_mask_ls[0+idx];
			d_mask_us3[0+is] = ptr_d_mask_us[0+idx];
			is++;
			}
		ib++;
		}

	// XXX for now, just shift after box-to-general constraints
	// better interleave them, to keep the block lower trianlgular structure !!!

	// general constraints

	char *c_ptr;

	nu_tmp = 0;
	ng_tmp = 0;
	for(ii=0; ii<N; ii++)
		{

		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_z = rqz[N-ii].pa+nu0+nx0;
			}
		ptr_d_ls = d[N-ii].pa+2*nb0+2*ng0;
		ptr_d_us = d[N-ii].pa+2*nb0+2*ng0+ns0;
		ptr_d_mask_ls = d_mask[N-ii].pa+2*nb0+2*ng0;
		ptr_d_mask_us = d_mask[N-ii].pa+2*nb0+2*ng0+ns0;

		if(ng0>0)
			{
			for(ig=0; ig<ng0; ig++)
				{
				idx = idxs_rev[N-ii][nb0+ig];
				if(idx>=0)
					{
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					d_mask_ls3[0+is] = ptr_d_mask_ls[0+idx];
					d_mask_us3[0+is] = ptr_d_mask_us[0+idx];
					is++;
					}
				}

			nu_tmp += nu0;

			VECCP(ng0, &d[N-ii], nb0, d2, nb2+nbg+ng_tmp);
			VECCP(ng0, &d[N-ii], 2*nb0+ng0, d2, 2*nb2+ng2+nbg+ng_tmp);
			VECCP(ng0, &d_mask[N-ii], nb0, d_mask2, nb2+nbg+ng_tmp);
			VECCP(ng0, &d_mask[N-ii], 2*nb0+ng0, d_mask2, 2*nb2+ng2+nbg+ng_tmp);

			GEMV_T(nx0, ng0, 1.0, &DCt[N-ii], nu0, 0, &Gammab[N-ii-1], 0, 0.0, tmp_nbgM, 0, tmp_nbgM, 0);

			AXPY(ng0, -1.0, tmp_nbgM, 0, d2, nb2+nbg+ng_tmp, d2, nb2+nbg+ng_tmp);
			AXPY(ng0, -1.0, tmp_nbgM, 0, d2, 2*nb2+ng2+nbg+ng_tmp, d2, 2*nb2+ng2+nbg+ng_tmp);

			ng_tmp += ng0;
			
			}
		else
			{

			nu_tmp += nu0;

			}

		}

	ii = N;

	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_z = rqz[0].pa+nu0+nx0;
		}
	ptr_d_ls = d[0].pa+2*nb0+2*ng0;
	ptr_d_us = d[0].pa+2*nb0+2*ng0+ns0;
	ptr_d_mask_ls = d_mask[0].pa+2*nb0+2*ng0;
	ptr_d_mask_us = d_mask[0].pa+2*nb0+2*ng0+ns0;

	if(ng0>0)
		{
		for(ig=0; ig<ng0; ig++)
			{
			idx = idxs_rev[0][nb0+ig];
			if(idx>=0)
				{
				ptr_z2[0+is]   = ptr_z[0+idx];
				ptr_z2[ns2+is] = ptr_z[ns0+idx];
				d_ls3[0+is]    = ptr_d_ls[0+idx];
				d_us3[0+is]    = ptr_d_us[0+idx];
				d_mask_ls3[0+is] = ptr_d_mask_ls[0+idx];
				d_mask_us3[0+is] = ptr_d_mask_us[0+idx];
				is++;
				}

			}

		VECCP(ng0, &d[0], nb0, d2, nb2+nbg+ng_tmp);
		VECCP(ng0, &d[0], 2*nb0+ng0, d2, 2*nb2+ng2+nbg+ng_tmp);
		VECCP(ng0, &d_mask[0], nb0, d_mask2, nb2+nbg+ng_tmp);
		VECCP(ng0, &d_mask[0], 2*nb0+ng0, d_mask2, 2*nb2+ng2+nbg+ng_tmp);

//		ng_tmp += ng[N-ii];

		}

	return;

	}



void EXPAND_SOL(struct OCP_QP *ocp_qp, struct DENSE_QP_SOL *dense_qp_sol, struct OCP_QP_SOL *ocp_qp_sol, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	int Np = N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	int ii, jj, idx;

	int *nu = ocp_qp->dim->nu;
	int *nx = ocp_qp->dim->nx;
	int *nb = ocp_qp->dim->nb;
	int *ng = ocp_qp->dim->ng;
	int *ns = ocp_qp->dim->ns;

	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;
	int **idxb = ocp_qp->idxb;
	struct STRMAT *RSQrq = ocp_qp->RSQrq;
	struct STRVEC *rqz = ocp_qp->rqz;
	struct STRMAT *DCt = ocp_qp->DCt;
	int **idxs_rev = ocp_qp->idxs_rev;

	struct STRVEC *vc = dense_qp_sol->v;
	struct STRVEC *pic = dense_qp_sol->pi;
	struct STRVEC *lamc = dense_qp_sol->lam;
	struct STRVEC *tc = dense_qp_sol->t;

	struct STRVEC *ux = ocp_qp_sol->ux;
	struct STRVEC *pi = ocp_qp_sol->pi;
	struct STRVEC *lam = ocp_qp_sol->lam;
	struct STRVEC *t = ocp_qp_sol->t;

	struct STRVEC *tmp_nuxM = cond_ws->tmp_nuxM;
	struct STRVEC *tmp_nbgM = cond_ws->tmp_nbgM;

	// early return
	if(N==0 & cond_arg->cond_last_stage==1)
		{
		// primal solution
		if(cond_arg->comp_prim_sol!=0)
			{
			VECCP(nu[0]+nx[0]+2*ns[0], dense_qp_sol->v, 0, ocp_qp_sol->ux, 0);
			}
		// dual solution
		if(cond_arg->comp_dual_sol_ineq!=0)
			{
			VECCP(2*nb[N]+2*ng[N]+2*ns[N], dense_qp_sol->lam, 0, ocp_qp_sol->lam, 0);
			VECCP(2*nb[N]+2*ng[N]+2*ns[N], dense_qp_sol->t, 0, ocp_qp_sol->t, 0);
			}
		return;
		}

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(idxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ngg = ng[0];
	int ns2 = ns[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ngg += ng[ii];
		ns2 += ns[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	REAL *ptr_ux;
	REAL *ptr_vc = vc->pa;
	int is = 0;
	int nx0, nu0, nb0, ng0, ns0;
	int idxb0;

	// primal solution
prim_sol:

	if(cond_arg->comp_prim_sol==0)
		{
		goto dual_sol_ineq;
		}

	// inputs & initial states
	int nu_tmp = 0;
	// final stages: copy only input
	for(ii=0; ii<N; ii++)
		{
		VECCP(nu[N-ii], vc, nu_tmp, ux+(N-ii), 0);
		nu_tmp += nu[N-ii];
		}
	// first stage: copy input and state
	VECCP(nu[0]+nx[0], vc, nu_tmp, ux+0, 0);

	// compute missing states by simulation within each block
	for(ii=0; ii<N; ii++)
		{
		GEMV_T(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, ux+ii, 0, 1.0, b+ii, 0, ux+(ii+1), nu[ii+1]);
		}
	
//	if(cond_arg->comp_dual_sol_ineq==0)
//		{
	// slack variables
	is = 0;
	// all box first XXX this keeps the same order as in cond !!!
	for(ii=0; ii<=N; ii++)
		{
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			nx0 = nx[N-ii];
			nu0 = nu[N-ii];
			nb0 = nb[N-ii];
			ng0 = ng[N-ii];
			ptr_ux = (ux+N-ii)->pa;
			for(jj=0; jj<nb0; jj++)
				{
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					ptr_ux[nu0+nx0+idx] = ptr_vc[nu2+nx2+is];
					ptr_ux[nu0+nx0+ns0+idx] = ptr_vc[nu2+nx2+ns2+is];
					is++;
					}
				}
			}
		}
	// all general after XXX this keeps the same order as in cond !!!
	for(ii=0; ii<=N; ii++)
		{
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			nx0 = nx[N-ii];
			nu0 = nu[N-ii];
			nb0 = nb[N-ii];
			ng0 = ng[N-ii];
			ptr_ux = (ux+N-ii)->pa;
			for(jj=nb0; jj<nb0+ng0; jj++)
				{
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					ptr_ux[nu0+nx0+idx] = ptr_vc[nu2+nx2+is];
					ptr_ux[nu0+nx0+ns0+idx] = ptr_vc[nu2+nx2+ns2+is];
					is++;
					}
				}
			}
		}

//		goto dual_sol_eq;
//		}


	// dual variables + slacks
dual_sol_ineq:

	if(cond_arg->comp_dual_sol_ineq==0)
		{
		goto dual_sol_eq;
		}

	// slack variables and ineq lagrange multipliers
	nbb = 0;
	nbg = 0;
	ngg = 0;
	REAL *ptr_lam;
	REAL *ptr_lam_lb;
	REAL *ptr_lam_ub;
	REAL *ptr_lam_lg;
	REAL *ptr_lam_ug;
	REAL *ptr_t;
	REAL *ptr_t_lb;
	REAL *ptr_t_ub;
	REAL *ptr_t_lg;
	REAL *ptr_t_ug;
	REAL *ptr_lamc = lamc->pa;
	REAL *ptr_lam_lbc = lamc->pa+0;
	REAL *ptr_lam_ubc = lamc->pa+nb2+ng2;
	REAL *ptr_lam_lgc = lamc->pa+nb2;
	REAL *ptr_lam_ugc = lamc->pa+2*nb2+ng2;
	REAL *ptr_tc = tc->pa;
	REAL *ptr_t_lbc = tc->pa+0;
	REAL *ptr_t_ubc = tc->pa+nb2+ng2;
	REAL *ptr_t_lgc = tc->pa+nb2;
	REAL *ptr_t_ugc = tc->pa+2*nb2+ng2;

	// box constraints
	// final stages
	for(ii=0; ii<N; ii++)
		{
		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ptr_lam_lb = (lam+N-ii)->pa+0;
		ptr_lam_ub = (lam+N-ii)->pa+nb0+ng0;
		ptr_t_lb = (t+N-ii)->pa+0;
		ptr_t_ub = (t+N-ii)->pa+nb0+ng0;
		for(jj=0; jj<nb0; jj++)
			{
			idxb0 = idxb[N-ii][jj];
			if(idxb0<nu0)
				{
				// box as box
				ptr_lam_lb[jj] = ptr_lam_lbc[nbb];
				ptr_lam_ub[jj] = ptr_lam_ubc[nbb];
				ptr_t_lb[jj] = ptr_t_lbc[nbb];
				ptr_t_ub[jj] = ptr_t_ubc[nbb];
				nbb++;
				}
			else
				{
				// box as general XXX change when decide where nbg are placed wrt ng
				ptr_lam_lb[jj] = ptr_lam_lgc[nbg];
				ptr_lam_ub[jj] = ptr_lam_ugc[nbg];
				ptr_t_lb[jj] = ptr_t_lgc[nbg];
				ptr_t_ub[jj] = ptr_t_ugc[nbg];
				nbg++;
				}
			}
		}
	// first stage
	// all box as box
	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	VECCP(nb0, lamc, 0+nbb, lam+0, 0);
	VECCP(nb0, lamc, nb2+ng2+nbb, lam+0, nb0+ng0);
	VECCP(nb0, tc, 0+nbb, t+0, 0);
	VECCP(nb0, tc, nb2+ng2+nbb, t+0, nb0+ng0);
	nbb += nb0;

	// general constraints
	// TODO process as vectors ???
	// XXX change when decide when nbg are placed wrt ng
	for(ii=0; ii<N; ii++)
		{
		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ptr_lam_lg = (lam+(N-ii))->pa+nb0;
		ptr_lam_ug = (lam+(N-ii))->pa+2*nb0+ng0;
		ptr_t_lg = (t+(N-ii))->pa+nb0;
		ptr_t_ug = (t+(N-ii))->pa+2*nb0+ng0;
		for(jj=0; jj<ng0; jj++)
			{
			// genenral as general
			ptr_lam_lg[jj] = ptr_lam_lgc[nbg+ngg];
			ptr_lam_ug[jj] = ptr_lam_ugc[nbg+ngg];
			ptr_t_lg[jj] = ptr_t_lgc[nbg+ngg];
			ptr_t_ug[jj] = ptr_t_ugc[nbg+ngg];
			ngg++;
			}
		}
	// first stage
	// all general as general
	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	VECCP(ng[0], lamc, nb2+nbg+ngg, lam+0, nb0);
	VECCP(ng[0], lamc, 2*nb2+ng2+nbg+ngg, lam+0, 2*nb0+ng0);
	VECCP(ng[0], tc, nb2+nbg+ngg, t+0, nb0);
	VECCP(ng[0], tc, 2*nb2+ng2+nbg+ngg, t+0, 2*nb0+ng0);
	ngg += ng0;
	
	// soft constraints
	is = 0;
	// all box first XXX this keeps the same order as in cond !!!
	for(ii=0; ii<=N; ii++)
		{
		ns0 = ns[N-ii];
		if(ns0>0)
			{
	//		nx0 = nx[N-ii];
	//		nu0 = nu[N-ii];
			nb0 = nb[N-ii];
			ng0 = ng[N-ii];
			ptr_ux = (ux+N-ii)->pa;
			ptr_lam = (lam+N-ii)->pa;
			ptr_t = (t+N-ii)->pa;
			for(jj=0; jj<nb0; jj++)
				{
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					ptr_lam[2*nb0+2*ng0+idx]     = ptr_lamc[2*nb2+2*ng2+is];
					ptr_lam[2*nb0+2*ng0+ns0+idx] = ptr_lamc[2*nb2+2*ng2+ns2+is];
					ptr_t[2*nb0+2*ng0+idx]     = ptr_tc[2*nb2+2*ng2+is];
					ptr_t[2*nb0+2*ng0+ns0+idx] = ptr_tc[2*nb2+2*ng2+ns2+is];
	//				ptr_ux[nu0+nx0+idx] = ptr_vc[nu2+nx2+is];
	//				ptr_ux[nu0+nx0+ns0+idx] = ptr_vc[nu2+nx2+ns2+is];
					is++;
					}
				}
			}
		}
	// all general after XXX this keeps the same order as in cond !!!
	for(ii=0; ii<=N; ii++)
		{
		ns0 = ns[N-ii];
		if(ns0>0)
			{
	//		nx0 = nx[N-ii];
	//		nu0 = nu[N-ii];
			nb0 = nb[N-ii];
			ng0 = ng[N-ii];
			ptr_ux = (ux+N-ii)->pa;
			ptr_lam = (lam+N-ii)->pa;
			ptr_t = (t+N-ii)->pa;
			for(jj=nb0; jj<nb0+ng0; jj++)
				{
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					ptr_lam[2*nb0+2*ng0+idx]     = ptr_lamc[2*nb2+2*ng2+is];
					ptr_lam[2*nb0+2*ng0+ns0+idx] = ptr_lamc[2*nb2+2*ng2+ns2+is];
					ptr_t[2*nb0+2*ng0+idx]     = ptr_tc[2*nb2+2*ng2+is];
					ptr_t[2*nb0+2*ng0+ns0+idx] = ptr_tc[2*nb2+2*ng2+ns2+is];
	//				ptr_ux[nu0+nx0+idx] = ptr_vc[nu2+nx2+is];
	//				ptr_ux[nu0+nx0+ns0+idx] = ptr_vc[nu2+nx2+ns2+is];
					is++;
					}
				}
			}
		}

	// lagrange multipliers of equality constraints
dual_sol_eq:

	if(cond_arg->comp_dual_sol_eq==0)
		{
		goto end;
		}

	// last stage
	if(cond_arg->cond_last_stage==0)
		{
		VECCP(nx[Np], pic, 0, pi+Np-1, 0); // XXX is the size nx[Np] correct ? is pic of that size ???
		}
	else
		{
//		SYMV_L(nx[Np], 1.0, RSQrq+Np, nu[Np], nu[Np], ux+Np, nu[Np], 1.0, rqz+Np, nu[Np], pi+Np-1, 0);
		VECCP(nx[Np], rqz+(Np), nu[Np], tmp_nuxM, nu[Np]);
		AXPY(nb[Np]+ng[Np], -1.0, lam+Np, 0, lam+Np, nb[Np]+ng[Np], tmp_nbgM, 0);
		VECAD_SP(nb[Np], 1.0, tmp_nbgM, 0, idxb[Np], tmp_nuxM, 0);
		// TODO avoid to multiply by R ???
		SYMV_L(nu[Np]+nx[Np], 1.0, RSQrq+(Np), 0, 0, ux+(Np), 0, 1.0, tmp_nuxM, 0, tmp_nuxM, 0);
		GEMV_N(nx[Np], ng[Np], 1.0, DCt+(Np), nu[Np], 0, tmp_nbgM, nb[Np], 1.0, tmp_nuxM, nu[Np], tmp_nuxM, nu[Np]);

		VECCP(nx[Np], tmp_nuxM, nu[Np], pi+(Np-1), 0);
		}

	for(ii=0; ii<Np-1; ii++)
		{
		VECCP(nx[Np-1-ii], rqz+(Np-1-ii), nu[Np-1-ii], tmp_nuxM, nu[Np-1-ii]);
		AXPY(nb[Np-1-ii]+ng[Np-1-ii], -1.0, lam+Np-1-ii, 0, lam+Np-1-ii, nb[Np-1-ii]+ng[Np-1-ii], tmp_nbgM, 0);
		VECAD_SP(nb[Np-1-ii], 1.0, tmp_nbgM, 0, idxb[Np-1-ii], tmp_nuxM, 0);
		// TODO avoid to multiply by R ???
		SYMV_L(nu[Np-1-ii]+nx[Np-1-ii], 1.0, RSQrq+(Np-1-ii), 0, 0, ux+(Np-1-ii), 0, 1.0, tmp_nuxM, 0, tmp_nuxM, 0);
		GEMV_N(nx[Np-1-ii], nx[Np-ii], 1.0, BAbt+(Np-1-ii), nu[Np-1-ii], 0, pi+(Np-1-ii), 0, 1.0, tmp_nuxM, nu[Np-1-ii], tmp_nuxM, nu[Np-1-ii]);
		GEMV_N(nx[Np-1-ii], ng[Np-1-ii], 1.0, DCt+(Np-1-ii), nu[Np-1-ii], 0, tmp_nbgM, nb[Np-1-ii], 1.0, tmp_nuxM, nu[Np-1-ii], tmp_nuxM, nu[Np-1-ii]);

		VECCP(nx[Np-1-ii], tmp_nuxM, nu[Np-1-ii], pi+(Np-2-ii), 0);
		}

end:

	return;

	}



void EXPAND_PRIMAL_SOL(struct OCP_QP *ocp_qp, struct DENSE_QP_SOL *dense_qp_sol, struct OCP_QP_SOL *ocp_qp_sol, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	int Np = N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	int ii, jj;

	int *nu = ocp_qp->dim->nu;
	int *nx = ocp_qp->dim->nx;
	int *nb = ocp_qp->dim->nb;
	int *ng = ocp_qp->dim->ng;
	int *ns = ocp_qp->dim->ns;

	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;
	int **idxb = ocp_qp->idxb;

	struct STRVEC *vc = dense_qp_sol->v;

	struct STRVEC *ux = ocp_qp_sol->ux;

	struct STRVEC *tmp_nuxM = cond_ws->tmp_nuxM;

	// early return
	if(N==0 & cond_arg->cond_last_stage==1)
		{
		VECCP(nu[0]+nx[0]+2*ns[0], dense_qp_sol->v, 0, ocp_qp_sol->ux, 0);
		return;
		}

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(idxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ngg = ng[0];
	int ns2 = ns[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ngg += ng[ii];
		ns2 += ns[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	// inputs & initial states
	int nu_tmp = 0;
	// final stages: copy only input
	for(ii=0; ii<N; ii++)
		{
		VECCP(nu[N-ii], vc, nu_tmp, ux+(N-ii), 0);
		nu_tmp += nu[N-ii];
		}
	// first stage: copy input and state
	VECCP(nu[0]+nx[0], vc, nu_tmp, ux+0, 0);

	// compute missing states by simulation within each block
	for(ii=0; ii<N; ii++)
		{
		GEMV_T(nu[ii]+nx[ii], nx[ii+1], 1.0, BAbt+ii, 0, 0, ux+ii, 0, 1.0, b+ii, 0, ux+(ii+1), nu[ii+1]);
		}

	return;

	}



/************************************************
* update cond
************************************************/

// update cond assuming that dynamics change in [0,idx-1], and to remain the same in [idx,N-1]
void UPDATE_COND_BABT(int *idxc, struct OCP_QP *ocp_qp, struct STRMAT *BAbt2, struct STRVEC *b2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;

	// early return
	if(N<0)
		return;
	
	int ii, jj;

	// index after first changed dynamic
	int idx = 0;
	for(ii=N-1; ii>=0; ii--)
		{
		if(idxc[ii]!=0)
			{
			idx = ii+1;
			break;
			}
		}
	
	// no changes
	if(idx==0)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRVEC *Gammab = cond_ws->Gammab;

	int nu_tmp, nu_tmp0, nu_tmp1;

	nu_tmp = 0;
	ii = 0;
	// B & A & b
	GECP(nu[0]+nx[0], nx[1], &BAbt[0], 0, 0, &Gamma[0], 0, 0);
	ROWIN(nx[1], 1.0, &b[0], 0, &Gamma[0], nu[0]+nx[0], 0);
	// b
	VECCP(nx[1], &b[0], 0, &Gammab[0], 0);

	nu_tmp += nu[0];
	ii++;

	for(ii=1; ii<idx; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// Gamma * A^T
		GEMM_NN(nu_tmp+nx[0]+1, nx[ii+1], nx[ii], 1.0, &Gamma[ii-1], 0, 0, &BAbt[ii], nu[ii], 0, 0.0, &Gamma[ii], nu[ii], 0, &Gamma[ii], nu[ii], 0); // Gamma * A^T

		GECP(nu[ii], nx[ii+1], &BAbt[ii], 0, 0, &Gamma[ii], 0, 0);

		nu_tmp += nu[ii];

		ROWAD(nx[ii+1], 1.0, &b[ii], 0, &Gamma[ii], nu_tmp+nx[0], 0);

		ROWEX(nx[ii+1], 1.0, &Gamma[ii], nu_tmp+nx[0], 0, &Gammab[ii], 0);
		}
	
	nu_tmp0 = nu_tmp;
	nu_tmp1 = 0;

	for(; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// Gamma * A^T
		GEMM_NN(nu_tmp0+nx[0]+1, nx[ii+1], nx[ii], 1.0, &Gamma[ii-1], nu_tmp1, 0, &BAbt[ii], nu[ii], 0, 0.0, &Gamma[ii], nu_tmp1+nu[ii], 0, &Gamma[ii], nu_tmp1+nu[ii], 0); // Gamma * A^T

		nu_tmp1 += nu[ii];
		nu_tmp += nu[ii];

		ROWAD(nx[ii+1], 1.0, &b[ii], 0, &Gamma[ii], nu_tmp+nx[0], 0);

		ROWEX(nx[ii+1], 1.0, &Gamma[ii], nu_tmp+nx[0], 0, &Gammab[ii], 0);
		}

	if(cond_arg->cond_last_stage==0)
		{
		// B & A
		GECP(nu_tmp+nx[0], nx[N], &Gamma[N-1], 0, 0, &BAbt2[0], 0, 0);
		// b
		ROWEX(nx[N], 1.0, &Gamma[N-1], nu_tmp+nx[0], 0, &b2[0], 0);
		}

	return;

	}



// update cond assuming that dynamics change in [0,idx-1], and to remain the same in [idx,N-1]
void UPDATE_COND_RSQRQ_N2NX3(int *idxc, struct OCP_QP *ocp_qp, struct STRMAT *RSQrq2, struct STRVEC *rqz2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;
	
	int nn;

	// index after first changed dynamic
	int idx = 0;
	for(nn=N-1; nn>=0; nn--)
		{
		if(idxc[nn]!=0)
			{
			idx = nn+1;
			break;
			}
		}

	// no changes
	if(idx==0 & idxc[N]==0)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;

	struct STRMAT *BAbt = ocp_qp->BAbt;
	struct STRVEC *b = ocp_qp->b;
	struct STRMAT *RSQrq = ocp_qp->RSQrq;
	struct STRVEC *rqz = ocp_qp->rqz;

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRMAT *L = cond_ws->L;
	struct STRMAT *Lx = cond_ws->Lx;
	struct STRMAT *AL = cond_ws->AL;

	// early return
	if(N==0)
		{
		GECP(nu[0]+nx[0], nu[0]+nx[0], &RSQrq[0], 0, 0, &RSQrq2[0], 0, 0);
		VECCP(nu[0]+nx[0], &rqz[0], 0, &rqz2[0], 0);
		return;
		}
	
	int nu2 = 0; // sum of all nu
	for(nn=0; nn<=N; nn++)
		nu2 += nu[nn];
	
	int nub = nu2; // backward partial sum
	int nuf = 0; // forward partial sum

	// sum of nu of changed dynamics
	int nuc = 0;
	for(nn=0; nn<idx; nn++)
		nuc += nu[nn];
//	printf("\nnuc %d\n", nuc);

	// final stage 
	nub -= nu[N];

	GECP(nu[N]+nx[N], nu[N]+nx[N], &RSQrq[N], 0, 0, &L[N], 0, 0);
	ROWIN(nu[N]+nx[N], 1.0, &rqz[N], 0, &L[N], nu[N]+nx[N], 0);

	// D
	TRCP_L(nu[N], &L[N], 0, 0, &RSQrq2[0], nuf, nuf);

	GEMM_NN(nub+nx[0]+1, nu[N], nx[N], 1.0, &Gamma[N-1], 0, 0, &L[N], nu[N], 0, 0.0, &RSQrq2[0], nuf+nu[N], nuf, &RSQrq2[0], nuf+nu[N], nuf);

	// m
	GEAD(1, nu[N], 1.0, &L[N], nu[N]+nx[N], 0, &RSQrq2[0], nu2+nx[0], nuf);

	nuf += nu[N];



	// middle stages 
	nn = 0;

	// unchanged dynamics
	for(; N-nn-1>=idx & nn<N-1; nn++)
		{	
		nub -= nu[N-nn-1];

		GEMM_NN(nuc+nx[0]+1, nu[N-nn-1], nx[N-nn-1], 1.0, &Gamma[N-nn-2], nub-nuc, 0, &L[N-nn-1], nu[N-nn-1], 0, 0.0, &RSQrq2[0], nuf+nu[N-nn-1]+nub-nuc, nuf, &RSQrq2[0], nuf+nu[N-nn-1]+nub-nuc, nuf);

		// m
		GEAD(1, nu[N-nn-1], 1.0, &L[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0, &RSQrq2[0], nu2+nx[0], nuf);

		nuf += nu[N-nn-1];

		}

	// changed dynamics
	for(; nn<N-1; nn++)
		{	
		nub -= nu[N-nn-1];

#if defined(LA_HIGH_PERFORMANCE)
		GECP(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);

		POTRF_L_MN(nx[N-nn]+1, nx[N-nn], Lx, 0, 0, Lx, 0, 0);
#else
		POTRF_L_MN(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
#endif
		ROWIN(nx[N-nn], 1.0, &b[N-nn-1], 0, &BAbt[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
		TRMM_RLNN(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, Lx, 0, 0, &BAbt[N-nn-1], 0, 0, AL, 0, 0);
		GEAD(1, nx[N-nn], 1.0, Lx, nx[N-nn], 0, AL, nu[N-nn-1]+nx[N-nn-1], 0);

		ROWIN(nu[N-nn-1]+nx[N-nn-1], 1.0, &rqz[N-nn-1], 0, &RSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
		SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, AL, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

		// D
		TRCP_L(nu[N-nn-1], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf);

		GEMM_NN(nub+nx[0]+1, nu[N-nn-1], nx[N-nn-1], 1.0, &Gamma[N-nn-2], 0, 0, &L[N-nn-1], nu[N-nn-1], 0, 0.0, &RSQrq2[0], nuf+nu[N-nn-1], nuf, &RSQrq2[0], nuf+nu[N-nn-1], nuf);

		// m
		GEAD(1, nu[N-nn-1], 1.0, &L[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0, &RSQrq2[0], nu2+nx[0], nuf);

		nuf += nu[N-nn-1];

		}

	// first stage
	nn = N-1;

#if defined(LA_HIGH_PERFORMANCE)
	GECP(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);

	POTRF_L_MN(nx[N-nn]+1, nx[N-nn], Lx, 0, 0, Lx, 0, 0);
#else
	POTRF_L_MN(nx[N-nn]+1, nx[N-nn], &L[N-nn], nu[N-nn], nu[N-nn], Lx, 0, 0);
#endif
	ROWIN(nx[N-nn], 1.0, &b[N-nn-1], 0, &BAbt[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
	TRMM_RLNN(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, Lx, 0, 0, &BAbt[N-nn-1], 0, 0, AL, 0, 0);
	GEAD(1, nx[N-nn], 1.0, Lx, nx[N-nn], 0, AL, nu[N-nn-1]+nx[N-nn-1], 0);

	ROWIN(nu[N-nn-1]+nx[N-nn-1], 1.0, &rqz[N-nn-1], 0, &RSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
	SYRK_LN_MN(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, AL, 0, 0, AL, 0, 0, 1.0, &RSQrq[N-nn-1], 0, 0, &L[N-nn-1], 0, 0);

	// D, M, m, P, p
//	GECP(nu[0]+nx[0]+1, nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf); // TODO dtrcp for 'rectangular' matrices
	TRCP_L(nu[0]+nx[0], &L[N-nn-1], 0, 0, &RSQrq2[0], nuf, nuf); // TODO dtrcp for 'rectangular' matrices
	GECP(1, nu[0]+nx[0], &L[N-nn-1], nu[0]+nx[0], 0, &RSQrq2[0], nuf+nu[0]+nx[0], nuf); // TODO dtrcp for 'rectangular' matrices
	// m p
	ROWEX(nu2+nx[0], 1.0, &RSQrq2[0], nu2+nx[0], 0, &rqz2[0], 0);

	return;

	}



// TODO
void UPDATE_COND_DCTD(int *idxc, struct OCP_QP *ocp_qp, int *idxb2, struct STRMAT *DCt2, struct STRVEC *d2, int *idxs_rev2, struct STRVEC *Z2, struct STRVEC *rqz2, struct COND_QP_ARG *cond_arg, struct COND_QP_WS *cond_ws)
	{

	int N = ocp_qp->dim->N;
	if(cond_arg->cond_last_stage==0)
		N -= 1;

	// early return
	if(N<0)
		return;

	int ii, jj;

	// index after first changed dynamic
	int idx = 0;
	for(ii=N-1; ii>=0; ii--)
		{
		if(idxc[ii]!=0)
			{
			idx = ii+1;
			break;
			}
		}
	
	// no changes
	if(idx==0 & idxc[N]==0)
		return;

	// extract input members
	int *nx = ocp_qp->dim->nx;
	int *nu = ocp_qp->dim->nu;
	int *nb = ocp_qp->dim->nb;
	int *ng = ocp_qp->dim->ng;
	int *ns = ocp_qp->dim->ns;

	int **idxb = ocp_qp->idxb;
	struct STRVEC *d = ocp_qp->d;
	struct STRMAT *DCt = ocp_qp->DCt;
	int **idxs_rev = ocp_qp->idxs_rev;
	struct STRVEC *Z = ocp_qp->Z;
	struct STRVEC *rqz = ocp_qp->rqz;

	// early return
	if(N==0 & cond_arg->cond_last_stage==1)
		{
		VECCP(2*nb[0]+2*ng[0]+2*ns[0], ocp_qp->d, 0, d2, 0);
		GECP(nu[0]+nx[0], ng[0], ocp_qp->DCt, 0, 0, DCt2, 0, 0);
		for(ii=0; ii<nb[0]; ii++) idxb2[ii] = ocp_qp->idxb[0][ii];
		VECCP(2*ns[0], ocp_qp->Z, 0, Z2, 0);
		VECCP(2*ns[0], ocp_qp->rqz, nu[0]+nx[0], rqz2, nu[0]+nx[0]); // XXX rqz2 offset
		for(ii=0; ii<nb[0]+ng[0]; ii++) idxs_rev2[ii] = ocp_qp->idxs_rev[0][ii];
		return;
		}

	// extract memory members
	struct STRMAT *Gamma = cond_ws->Gamma;
	struct STRVEC *Gammab = cond_ws->Gammab;
	struct STRVEC *tmp_nbgM = cond_ws->tmp_nbgM;


	REAL *ptr_d_lb;
	REAL *ptr_d_ub;
	REAL *ptr_d_ls;
	REAL *ptr_d_us;
	
	int nu_tmp, ng_tmp;

	int nu0, nx0, nb0, ng0, ns0;

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(idxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ns2 = ns[0];
	int ngg = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ns2 += ns[ii];
		ngg += ng[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	REAL *d_lb3 = d2->pa+0;
	REAL *d_ub3 = d2->pa+nb2+ng2;
	REAL *d_lg3 = d2->pa+nb2;
	REAL *d_ug3 = d2->pa+2*nb2+ng2;
	REAL *d_ls3 = d2->pa+2*nb2+2*ng2;
	REAL *d_us3 = d2->pa+2*nb2+2*ng2+ns2;

	// set constraint matrix to zero (it's 2 lower triangular matrices atm)
	GESE(nu2+nx2, ng2, 0.0, &DCt2[0], 0, 0);

	// box constraints

	int idx_gammab = nx[0];
	for(ii=0; ii<N; ii++)
		idx_gammab += nu[ii];

	int ib = 0;
	int ig = 0;
	int is = 0; // XXX

	int idxb0, idxg0;

	REAL *ptr_Z;
	REAL *ptr_z;
	REAL *ptr_Z2 = Z2->pa;
	REAL *ptr_z2 = rqz2->pa+nu2+nx2;

	REAL tmp;
	int idx_g;

	// middle stages
	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_Z = Z[N-ii].pa;
			ptr_z = rqz[N-ii].pa+nu0+nx0;
			}
		nu_tmp += nu0;
		ptr_d_lb = d[N-ii].pa+0;
		ptr_d_ub = d[N-ii].pa+nb0+ng0;
		ptr_d_ls = d[N-ii].pa+2*nb0+2*ng0;
		ptr_d_us = d[N-ii].pa+2*nb0+2*ng0+ns0;
		for(jj=0; jj<nb0; jj++)
			{
			idxb0 = idxb[N-ii][jj];
			if(idxb0<nu0) // input: box constraint
				{
				d_lb3[ib] = ptr_d_lb[jj];
				d_ub3[ib] = ptr_d_ub[jj];
				idxb2[ib] = nu_tmp - nu0 + idxb[N-ii][jj];
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					idxs_rev2[ib] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					is++;
					}
				ib++;
				}
			else // state: general constraint
				{
				idx_g = idxb0-nu0;
				tmp = BLASFEO_DVECEL(&Gammab[N-1-ii], idx_g);
				d_lg3[ig] = ptr_d_lb[jj] - tmp;
//				d_ug3[ig] = ptr_d_ub[jj] - tmp;
				d_ug3[ig] = ptr_d_ub[jj] + tmp; // XXX
				GECP(idx_gammab, 1, &Gamma[N-ii-1], 0, idx_g, &DCt2[0], nu_tmp, ig);
				idx = idxs_rev[N-ii][jj];
				if(idx>=0)
					{
					idxs_rev2[nb2+ig] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					is++;
					}
				ig++;
				}
			}
		idx_gammab -= nu[N-1-ii];
		}

	// initial stage: both inputs and states as box constraints
	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_Z = Z[0].pa;
		ptr_z = rqz[0].pa+nu0+nx0;
		}
	nu_tmp += nu0;
	ptr_d_lb = d[0].pa+0;
	ptr_d_ub = d[0].pa+nb0+ng0;
	ptr_d_ls = d[0].pa+2*nb0+2*ng0;
	ptr_d_us = d[0].pa+2*nb0+2*ng0+ns0;
	for(jj=0; jj<nb0; jj++)
		{
		idxb0 = idxb[0][jj];
		d_lb3[ib] = ptr_d_lb[jj];
		d_ub3[ib] = ptr_d_ub[jj];
		idxb2[ib] = nu_tmp - nu0 + idxb0;
		idx = idxs_rev[0][jj];
		if(idx>=0)
			{
			idxs_rev2[ib] = is;
			ptr_Z2[0+is]   = ptr_Z[0+idx];
			ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
			ptr_z2[0+is]   = ptr_z[0+idx];
			ptr_z2[ns2+is] = ptr_z[ns0+idx];
			d_ls3[0+is]    = ptr_d_ls[0+idx];
			d_us3[0+is]    = ptr_d_us[0+idx];
			is++;
			}
		ib++;
		}

	// XXX for now, just shift after box-to-general constraints
	// better interleave them, to keep the block lower trianlgular structure !!!

	// general constraints

	char *c_ptr;

	nu_tmp = 0;
	ng_tmp = 0;
	for(ii=0; ii<N; ii++)
		{

		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		ns0 = ns[N-ii];
		if(ns0>0)
			{
			ptr_Z = Z[N-ii].pa;
			ptr_z = rqz[N-ii].pa+nu0+nx0;
			}
		ptr_d_ls = d[N-ii].pa+2*nb0+2*ng0;
		ptr_d_us = d[N-ii].pa+2*nb0+2*ng0+ns0;

		if(ng0>0)
			{
			for(ig=0; ig<ng0; ig++)
				{
				idx = idxs_rev[N-ii][nb0+ig];
				if(idx>=0)
					{
					idxs_rev2[nb2+nbg+ng_tmp+ig] = is;
					ptr_Z2[0+is]   = ptr_Z[0+idx];
					ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
					ptr_z2[0+is]   = ptr_z[0+idx];
					ptr_z2[ns2+is] = ptr_z[ns0+idx];
					d_ls3[0+is]    = ptr_d_ls[0+idx];
					d_us3[0+is]    = ptr_d_us[0+idx];
					is++;
					}
				}

			GECP(nu0, ng0, &DCt[N-ii], 0, 0, DCt2, nu_tmp, nbg+ng_tmp);

			nu_tmp += nu0;

			GEMM_NN(nu2+nx[0]-nu_tmp, ng0, nx0, 1.0, &Gamma[N-1-ii], 0, 0, &DCt[N-ii], nu0, 0, 0.0, DCt2, nu_tmp, nbg+ng_tmp, DCt2, nu_tmp, nbg+ng_tmp);

			VECCP(ng0, &d[N-ii], nb0, d2, nb2+nbg+ng_tmp);
			VECCP(ng0, &d[N-ii], 2*nb0+ng0, d2, 2*nb2+ng2+nbg+ng_tmp);

			GEMV_T(nx0, ng0, 1.0, &DCt[N-ii], nu0, 0, &Gammab[N-ii-1], 0, 0.0, tmp_nbgM, 0, tmp_nbgM, 0);

			AXPY(ng0, -1.0, tmp_nbgM, 0, d2, nb2+nbg+ng_tmp, d2, nb2+nbg+ng_tmp);
//			AXPY(ng0, -1.0, tmp_nbgM, 0, d2, 2*nb2+ng2+nbg+ng_tmp, d2, 2*nb2+ng2+nbg+ng_tmp);
			AXPY(ng0,  1.0, tmp_nbgM, 0, d2, 2*nb2+ng2+nbg+ng_tmp, d2, 2*nb2+ng2+nbg+ng_tmp); // XXX

			ng_tmp += ng0;
			
			}
		else
			{

			nu_tmp += nu0;

			}

		}

	ii = N;

	nx0 = nx[0];
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	ns0 = ns[0];
	if(ns0>0)
		{
		ptr_Z = Z[0].pa;
		ptr_z = rqz[0].pa+nu0+nx0;
		}
	ptr_d_ls = d[0].pa+2*nb0+2*ng0;
	ptr_d_us = d[0].pa+2*nb0+2*ng0+ns0;

	if(ng0>0)
		{
		for(ig=0; ig<ng0; ig++)
			{
			idx = idxs_rev[0][nb0+ig];
			if(idx>=0)
				{
				idxs_rev2[nb2+nbg+ng_tmp+ig] = is;
				ptr_Z2[0+is]   = ptr_Z[0+idx];
				ptr_Z2[ns2+is] = ptr_Z[ns0+idx];
				ptr_z2[0+is]   = ptr_z[0+idx];
				ptr_z2[ns2+is] = ptr_z[ns0+idx];
				d_ls3[0+is]    = ptr_d_ls[0+idx];
				d_us3[0+is]    = ptr_d_us[0+idx];
				is++;
				}
			}

		GECP(nu0+nx0, ng0, &DCt[0], 0, 0, DCt2, nu_tmp, nbg+ng_tmp);

		VECCP(ng0, &d[0], nb0, d2, nb2+nbg+ng_tmp);
		VECCP(ng0, &d[0], 2*nb0+ng0, d2, 2*nb2+ng2+nbg+ng_tmp);

//		ng_tmp += ng[N-ii];

		}

	return;

	}




