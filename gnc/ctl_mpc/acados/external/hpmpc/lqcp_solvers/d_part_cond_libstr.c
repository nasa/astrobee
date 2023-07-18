/**************************************************************************************************
*                                                                                                 *
* This file is part of HPMPC.                                                                     *
*                                                                                                 *
* HPMPC -- Library for High-Performance implementation of solvers for MPC.                        *
* Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                *
*                                                                                                 *
* HPMPC is free software; you can redistribute it and/or                                          *
* modify it under the terms of the GNU Lesser General Public                                      *
* License as published by the Free Software Foundation; either                                    *
* version 2.1 of the License, or (at your option) any later version.                              *
*                                                                                                 *
* HPMPC is distributed in the hope that it will be useful,                                        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  *
*                                                                                                 *
* Author: Gianluca Frison, giaf (at) dtu.dk                                                       *
*                                                                                                 *
**************************************************************************************************/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "../include/aux_d.h"
#include "../include/blas_d.h"
#include "../include/block_size.h"
#include "../include/lqcp_aux.h"

#ifdef BLASFEO

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>



static void d_comp_Gamma_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsGamma)
	{

	int ii, jj;

	struct blasfeo_dmat sA;

	int nu_tmp;

	nu_tmp = 0;
	ii = 0;
	// B & A & b
	blasfeo_dgecp(nu[0]+nx[0]+1, nx[1], &hsBAbt[0], 0, 0, &hsGamma[0], 0, 0);
	//
	nu_tmp += nu[0];
	ii++;

	for(ii=1; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// Gamma * A^T
		blasfeo_dgemm_nn(nu_tmp+nx[0]+1, nx[ii+1], nx[ii], 1.0, &hsGamma[ii-1], 0, 0, &hsBAbt[ii], nu[ii], 0, 0.0, &hsGamma[ii], nu[ii], 0, &hsGamma[ii], nu[ii], 0); // Gamma * A^T

		blasfeo_dgecp(nu[ii], nx[ii+1], &hsBAbt[ii], 0, 0, &hsGamma[ii], 0, 0);

		nu_tmp += nu[ii];

		blasfeo_dgead(1, nx[ii+1], 1.0, &hsBAbt[ii], nu[ii]+nx[ii], 0, &hsGamma[ii], nu_tmp+nx[0], 0);
		}
	
	return;

	}



static void d_comp_Gammab_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dvec *hsGammab)
	{

	int ii, jj;

	ii = 0;
	// B & A & b
	blasfeo_dveccp(nx[1], &hsb[0], 0, &hsGammab[0], 0);
	//
	ii++;

	for(ii=1; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// A * Gammab
		blasfeo_dgemv_t(nx[ii], nx[ii+1], 1.0, &hsBAbt[ii], nu[ii], 0, &hsGammab[ii-1], 0, 0.0, &hsGammab[ii], 0, &hsGammab[ii], 0);

		blasfeo_dvecad(nx[ii+1], 1.0, &hsb[ii], 0, &hsGammab[ii], 0);
		}
	
	return;

	}



static void d_cond_BAbt_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsGamma, struct blasfeo_dmat *sBAbt2)
	{

	int ii, jj;

	struct blasfeo_dmat sA;

	int nu_tmp;

	nu_tmp = 0;
	ii = 0;
	// B & A & b
	blasfeo_dgecp(nu[0]+nx[0]+1, nx[1], &hsBAbt[0], 0, 0, &hsGamma[0], 0, 0);
	//
	nu_tmp += nu[0];
	ii++;

	for(ii=1; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// Gamma * A^T
		blasfeo_dgemm_nn(nu_tmp+nx[0]+1, nx[ii+1], nx[ii], 1.0, &hsGamma[ii-1], 0, 0, &hsBAbt[ii], nu[ii], 0, 0.0, &hsGamma[ii], nu[ii], 0, &hsGamma[ii], nu[ii], 0); // Gamma * A^T

		blasfeo_dgecp(nu[ii], nx[ii+1], &hsBAbt[ii], 0, 0, &hsGamma[ii], 0, 0);

		nu_tmp += nu[ii];

		blasfeo_dgead(1, nx[ii+1], 1.0, &hsBAbt[ii], nu[ii]+nx[ii], 0, &hsGamma[ii], nu_tmp+nx[0], 0);
		}
	
	// B & A & b
	blasfeo_dgecp(nu_tmp+nx[0]+1, nx[N], &hsGamma[N-1], 0, 0, sBAbt2, 0, 0);
	// TODO pass sBAbt2 as Gamma[N-1] !!!!!

	return;

	}



static void d_cond_b_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dvec *hsGammab, struct blasfeo_dvec *sb2)
	{

	int ii, jj;

	ii = 0;
	// B & A & b
	blasfeo_dveccp(nx[1], &hsb[0], 0, &hsGammab[0], 0);
	//
	ii++;

	for(ii=1; ii<N; ii++)
		{
		// TODO check for equal pointers and avoid copy

		// A * Gammab
		blasfeo_dgemv_t(nx[ii], nx[ii+1], 1.0, &hsBAbt[ii], nu[ii], 0, &hsGammab[ii-1], 0, 0.0, &hsGammab[ii], 0, &hsGammab[ii], 0);

		blasfeo_dvecad(nx[ii+1], 1.0, &hsb[ii], 0, &hsGammab[ii], 0);
		}
	
	// B & A & b
	blasfeo_dveccp(nx[N], &hsGammab[N-1], 0, sb2, 0);

	return;

	}



static void d_cond_RSQrq_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsGamma, struct blasfeo_dmat *sRSQrq2, struct blasfeo_dmat *hsL, void *work_space, int *work_space_sizes)
	{

	// early return
	if(N<0)
		return;

	// early return
	if(N==0)
		{
		blasfeo_dgecp(nu[0]+nx[0]+1, nu[0]+nx[0], &hsRSQrq[0], 0, 0, sRSQrq2, 0, 0);
		return;
		}

	int nn;

	struct blasfeo_dmat sL;
	struct blasfeo_dmat sM;
	struct blasfeo_dmat sLx;
	struct blasfeo_dmat sBAbtL;

	int nu2 = 0; // sum of all nu
	for(nn=0; nn<=N; nn++)
		nu2 += nu[nn];
	
	int nub = nu2; // backward partial sum
	int nuf = 0; // forward partial sum

	char *c_ptr[3];
	c_ptr[0] = (char *) work_space;
	c_ptr[1] = c_ptr[0] + work_space_sizes[0];


	// final stage 
	nub -= nu[N];

	blasfeo_dgecp(nu[N]+nx[N]+1, nu[N]+nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);

	// D
	blasfeo_dtrcp_l(nu[N], &hsL[N], 0, 0, sRSQrq2, nuf, nuf);

	blasfeo_dgemm_nn(nub+nx[0]+1, nu[N], nx[N], 1.0, &hsGamma[N-1], 0, 0, &hsL[N], nu[N], 0, 0.0, sRSQrq2, nuf+nu[N], nuf, sRSQrq2, nuf+nu[N], nuf);

	// m
	blasfeo_dgead(1, nu[N], 1.0, &hsL[N], nu[N]+nx[N], 0, sRSQrq2, nu2+nx[0], nuf);

	nuf += nu[N];



	// middle stages 
	for(nn=0; nn<N-1; nn++)
		{	
		nub -= nu[N-nn-1];

		blasfeo_create_dmat(nx[N-nn]+1, nx[N-nn], &sLx, (void *) c_ptr[0]);
		blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], &sBAbtL, (void *) c_ptr[1]);

#if defined(LA_HIGH_PERFORMANCE)
		blasfeo_dgecp(nx[N-nn]+1, nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &sLx, 0, 0);

		blasfeo_dpotrf_l_mn(nx[N-nn]+1, nx[N-nn], &sLx, 0, 0, &sLx, 0, 0);

		blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &sLx, 0, 0, &hsBAbt[N-nn-1], 0, 0, &sBAbtL, 0, 0);
#else
		blasfeo_dpotrf_l_mn(nx[N-nn]+1, nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &sLx, 0, 0);

		blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &sLx, 0, 0, &hsBAbt[N-nn-1], 0, 0, &sBAbtL, 0, 0);
#endif
		blasfeo_dgead(1, nx[N-nn], 1.0, &sLx, nx[N-nn], 0, &sBAbtL, nu[N-nn-1]+nx[N-nn-1], 0);

		blasfeo_dsyrk_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &sBAbtL, 0, 0, &sBAbtL, 0, 0, 1.0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);

		// D
		blasfeo_dtrcp_l(nu[N-nn-1], &hsL[N-nn-1], 0, 0, sRSQrq2, nuf, nuf);

		blasfeo_dgemm_nn(nub+nx[0]+1, nu[N-nn-1], nx[N-nn-1], 1.0, &hsGamma[N-nn-2], 0, 0, &hsL[N-nn-1], nu[N-nn-1], 0, 0.0, sRSQrq2, nuf+nu[N-nn-1], nuf, sRSQrq2, nuf+nu[N-nn-1], nuf);

		// m
		blasfeo_dgead(1, nu[N-nn-1], 1.0, &hsL[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0, sRSQrq2, nu2+nx[0], nuf);

		nuf += nu[N-nn-1];

		}

	// first stage
	nn = N-1;

	blasfeo_create_dmat(nx[N-nn]+1, nx[N-nn], &sLx, (void *) c_ptr[0]);
	blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], &sBAbtL, (void *) c_ptr[1]);
	
#if defined(LA_HIGH_PERFORMANCE)
	blasfeo_dgecp(nx[N-nn]+1, nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &sLx, 0, 0);

	blasfeo_dpotrf_l_mn(nx[N-nn]+1, nx[N-nn], &sLx, 0, 0, &sLx, 0, 0);

	blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &sLx, 0, 0, &hsBAbt[N-nn-1], 0, 0, &sBAbtL, 0, 0);
#else
	blasfeo_dpotrf_l_mn(nx[N-nn]+1, nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &sLx, 0, 0);

	blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &sLx, 0, 0, &hsBAbt[N-nn-1], 0, 0, &sBAbtL, 0, 0);
#endif
	blasfeo_dgead(1, nx[N-nn], 1.0, &sLx, nx[N-nn], 0, &sBAbtL, nu[N-nn-1]+nx[N-nn-1], 0);

	blasfeo_dsyrk_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &sBAbtL, 0, 0, &sBAbtL, 0, 0, 1.0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);

	// D, M, m, P, p
//	blasfeo_dgecp(nu[0]+nx[0]+1, nu[0]+nx[0], &hsL[N-nn-1], 0, 0, sRSQrq2, nuf, nuf); // TODO dtrcp for 'rectangular' matrices
	blasfeo_dtrcp_l(nu[0]+nx[0], &hsL[N-nn-1], 0, 0, sRSQrq2, nuf, nuf); // TODO dtrcp for 'rectangular' matrices
	blasfeo_dgecp(1, nu[0]+nx[0], &hsL[N-nn-1], nu[0]+nx[0], 0, sRSQrq2, nuf+nu[0]+nx[0], nuf); // TODO dtrcp for 'rectangular' matrices

	return;

	}



static void d_cond_rq_libstr(int N, int *nx, int *nu, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsL, struct blasfeo_dvec *hsGammab, struct blasfeo_dvec *srq2, void *work_space, int *work_space_sizes)
	{

	// early return
	if(N<0)
		return;

	// early return
	if(N==0)
		{
		blasfeo_dveccp(nu[0]+nx[0], &hsrq[0], 0, srq2, 0);
		return;
		}

	int nn;

	struct blasfeo_dvec sl;
	struct blasfeo_dvec sPbp;

	int nuf = 0;

	char *c_ptr[2];
	c_ptr[0] = (char *) work_space;
	c_ptr[1] = c_ptr[0] + work_space_sizes[0];


	// final stage

	blasfeo_create_dvec(nu[N]+nx[N], &sl, (void *) c_ptr[0]);

	blasfeo_dveccp(nu[N]+nx[N], &hsrq[N], 0, &sl, 0);

	blasfeo_dgemv_t(nx[N], nu[N], 1.0, &hsL[N], nu[N], 0, &hsGammab[N-1], 0, 1.0, &sl, 0, srq2, nuf);

	nuf += nu[N];


	// middle stages 
	for(nn=0; nn<N-1; nn++)
		{

		blasfeo_create_dvec(nx[N-nn], &sPbp, (void *) c_ptr[1]);

		blasfeo_dsymv_l(nx[N-nn], nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, 1.0, &sl, nu[N-nn], &sPbp, 0);

		blasfeo_create_dvec(nu[N-nn-1]+nx[N-nn-1], &sl, (void *) c_ptr[0]);

		blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &sPbp, 0, 1.0, &hsrq[N-nn-1], 0, &sl, 0);

		blasfeo_dgemv_t(nx[N-nn-1], nu[N-nn-1], 1.0, &hsL[N-nn-1], nu[N-nn-1], 0, &hsGammab[N-nn-2], 0, 1.0, &sl, 0, srq2, nuf);

		nuf += nu[N-nn-1];

		}

	// first stage
	nn = N-1;

	blasfeo_create_dvec(nx[N-nn], &sPbp, (void *) c_ptr[1]);

	blasfeo_dsymv_l(nx[N-nn], nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, 1.0, &sl, nu[N-nn], &sPbp, 0);

	blasfeo_create_dvec(nu[N-nn-1]+nx[N-nn-1], &sl, (void *) c_ptr[0]);

	blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &sPbp, 0, 1.0, &hsrq[N-nn-1], 0, &sl, 0);

	blasfeo_dveccp(nu[N-nn-1]+nx[N-nn-1], &sl, 0, srq2, nuf);

	return;

	}



static void d_cond_DCtd_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, struct blasfeo_dmat *hsGamma, struct blasfeo_dmat *sDCt2, struct blasfeo_dvec *sd2, int *idxb2, void *work_space)
	{

	// early return
	if(N<0)
		return;

	double *d2 = sd2->pa;
	double *ptr_d;
	
	int nu_tmp, ng_tmp;

	int ii, jj;

	int nu0, nx0, nb0, ng0, nt0;

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(hidxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ngg = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ngg += ng[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	// set constraint matrix to zero (it's 2 lower triangular matrices atm)
	blasfeo_dgese(nu2+nx2, ng2, 0.0, sDCt2, 0, 0);

	// box constraints

	int idx_gammab = nx[0];
	for(ii=0; ii<N; ii++)
		idx_gammab += nu[ii];

	int ib = 0;
	int ig = 0;

	double tmp;
	int idx_g;

	// middle stages
	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		nt0 = nb0 + ng0;
		nu_tmp += nu0;
		ptr_d = hsd[N-ii].pa;
		for(jj=0; jj<nb0; jj++)
			{
			if(hidxb[N-ii][jj]<nu0) // input: box constraint
				{
				d2[0*nt2+ib] = ptr_d[0*nt0+jj];
				d2[1*nt2+ib] = ptr_d[1*nt0+jj];
				idxb2[ib] = nu_tmp - nu0 + hidxb[N-ii][jj];
				ib++;
				}
			else // state: general constraint
				{
				idx_g = hidxb[N-ii][jj]-nu0;
				tmp = blasfeo_dgeex1(&hsGamma[N-1-ii], idx_gammab, idx_g);
				d2[nb2+0*nt2+ig] = ptr_d[0*nt0+jj] - tmp;
				d2[nb2+1*nt2+ig] = ptr_d[1*nt0+jj] - tmp;
				blasfeo_dgecp(idx_gammab, 1, &hsGamma[N-ii-1], 0, idx_g, sDCt2, nu_tmp, ig);
				ig++;
				}
			}
		idx_gammab -= nu[N-1-ii];
		}

	// initial stage: both inputs and states as box constraints
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	nt0 = nb0 + ng0;
	nu_tmp += nu0;
	ptr_d = hsd[0].pa;
	for(jj=0; jj<nb0; jj++)
		{
		d2[0*nt2+ib] = ptr_d[0*nt0+jj];
		d2[1*nt2+ib] = ptr_d[1*nt0+jj];
		idxb2[ib] = nu_tmp - nu0 + hidxb[0][jj];
		ib++;
		}

	// XXX for now, just shift after box-to-general constraints
	// better interleave them, to keep the block lower trianlgular structure !!!

	// general constraints

	struct blasfeo_dvec sGammab;
	struct blasfeo_dvec sCGammab;

	char *c_ptr;

	nu_tmp = 0;
	ng_tmp = 0;
	for(ii=0; ii<N; ii++)
		{

		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		ng0 = ng[N-ii];
		nt0 = nb0 + ng0;

		if(ng0>0)
			{

			c_ptr = (char *) work_space;

			blasfeo_dgecp(nu0, ng0, &hsDCt[N-ii], 0, 0, sDCt2, nu_tmp, nbg+ng_tmp);

			nu_tmp += nu0;

			blasfeo_dgemm_nn(nu2+nx[0]-nu_tmp, ng0, nx0, 1.0, &hsGamma[N-1-ii], 0, 0, &hsDCt[N-ii], nu0, 0, 0.0, sDCt2, nu_tmp, nbg+ng_tmp, sDCt2, nu_tmp, nbg+ng_tmp);

			blasfeo_dveccp(ng0, &hsd[N-ii], nb0+0*nt0, sd2, nb2+0*nt2+nbg+ng_tmp);
			blasfeo_dveccp(ng0, &hsd[N-ii], nb0+1*nt0, sd2, nb2+1*nt2+nbg+ng_tmp);

			blasfeo_create_dvec(nx0, &sGammab, (void *) c_ptr);
			c_ptr += sGammab.memsize;
			blasfeo_create_dvec(ng0, &sCGammab, (void *) c_ptr);
			c_ptr += sCGammab.memsize;

			blasfeo_drowex(nx0, 1.0, &hsGamma[N-1-ii], nu2+nx[0]-nu_tmp, 0, &sGammab, 0);

			blasfeo_dgemv_t(nx0, ng0, 1.0, &hsDCt[N-ii], nu0, 0, &sGammab, 0, 0.0, &sCGammab, 0, &sCGammab, 0);

			blasfeo_daxpy(ng0, -1.0, &sCGammab, 0, sd2, nb2+0*nt2+nbg+ng_tmp, sd2, nb2+0*nt2+nbg+ng_tmp);
			blasfeo_daxpy(ng0, -1.0, &sCGammab, 0, sd2, nb2+1*nt2+nbg+ng_tmp, sd2, nb2+1*nt2+nbg+ng_tmp);

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
	ng0 = ng[0];
	nt0 = nb0 + ng0;

	if(ng0>0)
		{

		blasfeo_dgecp(nu0+nx0, ng0, &hsDCt[0], 0, 0, sDCt2, nu_tmp, nbg+ng_tmp);

		blasfeo_dveccp(ng0, &hsd[0], nb0+0*nt0, sd2, nb2+0*nt2+nbg+ng_tmp);
		blasfeo_dveccp(ng0, &hsd[0], nb0+1*nt0, sd2, nb2+1*nt2+nbg+ng_tmp);

//		ng_tmp += ng[N-ii];

		}

	return;

	}



static void d_cond_d_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, struct blasfeo_dvec *hsGammab, struct blasfeo_dvec *sd2, void *work_space)
	{

	// early return
	if(N<0)
		return;

	double *d2 = sd2->pa;
	double *ptr_d;
	
	int nu_tmp, ng_tmp;

	int ii, jj;

	int nu0, nx0, nb0, ng0, nt0;

	// problem size

	int nbb = nb[0]; // box that remain box constraints
	int nbg = 0; // box that becomes general constraints
	for(ii=1; ii<=N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(hidxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
	
	int nx2 = nx[0];
	int nu2 = nu[0];
	int ngg = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		nu2 += nu[ii];
		ngg += ng[ii];
		}
	int ng2 = nbg + ngg;
	int nb2 = nbb;
	int nt2 = nb2 + ng2;

	// box constraints

	int idx_gammab = nx[0];
	for(ii=0; ii<N; ii++)
		idx_gammab += nu[ii];

	int ib = 0;
	int ig = 0;

	double tmp;
	int idx_g;

	// middle stages
	nu_tmp = 0;
	for(ii=0; ii<N; ii++)
		{
		nu0 = nu[N-ii];
		nb0 = nb[N-ii];
		ng0 = ng[N-ii];
		nt0 = nb0 + ng0;
		nu_tmp += nu0;
		ptr_d = hsd[N-ii].pa;
		for(jj=0; jj<nb0; jj++)
			{
			if(hidxb[N-ii][jj]<nu[N-ii]) // input: box constraint
				{
				d2[0*nt2+ib] = ptr_d[0*nt0+jj];
				d2[1*nt2+ib] = ptr_d[1*nt0+jj];
				ib++;
				}
			else // state: general constraint
				{
				idx_g = hidxb[N-ii][jj]-nu0;
				tmp = blasfeo_dvecex1(&hsGammab[N-1-ii], idx_g);
				d2[nb2+0*nt2+ig] = ptr_d[0*nt0+jj] - tmp;
				d2[nb2+1*nt2+ig] = ptr_d[1*nt0+jj] - tmp;
				ig++;
				}
			}
		}

	// initial stage: both inputs and states as box constraints
	nu0 = nu[0];
	nb0 = nb[0];
	ng0 = ng[0];
	nt0 = nb0 + ng0;
	nu_tmp += nu0;
	ptr_d = hsd[0].pa;
	for(jj=0; jj<nb0; jj++)
		{
		d2[0*nt2+ib] = ptr_d[0*nt0+jj];
		d2[1*nt2+ib] = ptr_d[1*nt0+jj];
		ib++;
		}

	// XXX for now, just shift after box-to-general constraints
	// better interleave them, to keep the block lower trianlgular structure !!!

	// general constraints

	struct blasfeo_dvec sCGammab;

	char *c_ptr = (char *) work_space;

	nu_tmp = 0;
	ng_tmp = 0;
	for(ii=0; ii<N; ii++)
		{

		nx0 = nx[N-ii];
		nu0 = nu[N-ii];
		ng0 = ng[N-ii];
		nt0 = nb0 + ng0;

		nu_tmp += nu0;

		if(ng0>0)
			{

			blasfeo_dveccp(ng0, &hsd[N-ii], nb0+0*nt0, sd2, nb2+0*nt2+nbg+ng_tmp);
			blasfeo_dveccp(ng0, &hsd[N-ii], nb0+1*nt0, sd2, nb2+1*nt2+nbg+ng_tmp);

			blasfeo_create_dvec(ng0, &sCGammab, (void *) c_ptr);

			blasfeo_dgemv_t(nx0, ng0, 1.0, &hsDCt[N-ii], nu0, 0, &hsGammab[N-1-ii], 0, 0.0, &sCGammab, 0, &sCGammab, 0);

			blasfeo_daxpy(ng0, -1.0, &sCGammab, 0, sd2, nb2+0*nt2+nbg+ng_tmp, sd2, nb2+0*nt2+nbg+ng_tmp);
			blasfeo_daxpy(ng0, -1.0, &sCGammab, 0, sd2, nb2+1*nt2+nbg+ng_tmp, sd2, nb2+1*nt2+nbg+ng_tmp);

			ng_tmp += ng0;

			}

		}

	ii = N;

	nx0 = nx[0];
	nu0 = nu[0];
	ng0 = ng[0];
	nt0 = nb0 + ng0;

	if(ng0>0)
		{

		blasfeo_dveccp(ng0, &hsd[0], nb0+0*nt0, sd2, nb2+0*nt2+nbg+ng_tmp);
		blasfeo_dveccp(ng0, &hsd[0], nb0+1*nt0, sd2, nb2+1*nt2+nbg+ng_tmp);

//		ng_tmp += ng[N-ii];

		}

	return;

	}



// TODO XXX update problem size when adding flag for condensing the last stage !!!!!!!!!!!!!!!
// XXX does not compute hidxb2, since nb2 has to be known to allocate the right space for hidxb2 !!!
void d_part_cond_compute_problem_size_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int N2, int *nx2, int *nu2, int *nb2, int *ng2)
	{

	int ii, jj, kk;

	int N1 = N/N2; // (floor) horizon of small blocks
	int R1 = N - N2*N1; // the first R1 blocks have horizon N1+1
	int M1 = R1>0 ? N1+1 : N1; // (ceil) horizon of large blocks
	int T1; // horizon of current block

	int N_tmp = 0; // temporary sum of horizons
	int nbb; // box constr that remain box constr
	int nbg; // box constr that becomes general constr
	for(ii=0; ii<N2; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		nx2[ii] = nx[N_tmp+0];
		nu2[ii] = nu[N_tmp+0];
		nb2[ii] = nb[N_tmp+0];
		ng2[ii] = ng[N_tmp+0];
		for(jj=1; jj<T1; jj++)
			{
			nbb = 0;
			nbg = 0;
			for(kk=0; kk<nb[N_tmp+jj]; kk++)
				if(hidxb[N_tmp+jj][kk]<nu[N_tmp+jj])
					nbb++;
				else
					nbg++;
			nx2[ii] += 0;
			nu2[ii] += nu[N_tmp+jj];
			nb2[ii] += nbb;
			ng2[ii] += ng[N_tmp+jj] + nbg;
			}
		N_tmp += T1;
		}
	nx2[N2] = nx[N];
	nu2[N2] = nu[N];
	nb2[N2] = nb[N];
	ng2[N2] = ng[N];

	}



void d_part_cond_compute_problem_size_libstr_noidxb(int N, int *nx, int *nu, int *nb, int *nbx, int *nbu, int *ng, int N2, int *nx2, int *nu2, int *nb2, int *ng2)
	{

	int ii, jj, kk;

	int N1 = N/N2; // (floor) horizon of small blocks
	int R1 = N - N2*N1; // the first R1 blocks have horizon N1+1
	int M1 = R1>0 ? N1+1 : N1; // (ceil) horizon of large blocks
	int T1; // horizon of current block

	int N_tmp = 0; // temporary sum of horizons
	for(ii=0; ii<N2; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		nx2[ii] = nx[N_tmp+0];
		nu2[ii] = nu[N_tmp+0];
		nb2[ii] = nb[N_tmp+0];
		ng2[ii] = ng[N_tmp+0];
		for(jj=1; jj<T1; jj++)
			{
			nx2[ii] += 0;
			nu2[ii] += nu[N_tmp+jj];
			nb2[ii] += nbu[N_tmp+jj];
			ng2[ii] += ng[N_tmp+jj] + nbx[N_tmp+jj];
			}
		N_tmp += T1;
		}
	nx2[N2] = nx[N];
	nu2[N2] = nu[N];
	nb2[N2] = nb[N];
	ng2[N2] = ng[N];

	}



int d_part_cond_work_space_size_bytes_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int N2, int *nx2, int *nu2, int *nb2, int *ng2, int *work_space_sizes)
	{

	// early return
	if(N2==N)
		{
		return 0;
		}

	int ii, jj;
	int nu_tmp;

	int N1 = N/N2; // (floor) horizon of small blocks
	int R1 = N - N2*N1; // the first R1 blocks have horizon N1+1
	int M1 = R1>0 ? N1+1 : N1; // (ceil) horizon of large blocks
	int T1; // horizon of current block
	int N_tmp; // temporary sum of horizons

	int Gamma_size = 0;
	work_space_sizes[0] = 0;
	work_space_sizes[1] = 0;
	work_space_sizes[2] = 0;
	work_space_sizes[3] = 0;

	int tmp_size;

	N_tmp = 0;
	for(ii=0; ii<N2; ii++)
		{

		T1 = ii<R1 ? M1 : N1;

		// hsGamma // XXX size(hsGamma) > size(hsGammab) !!!!!
		nu_tmp = 0;
		tmp_size = 0;
		for(jj=0; jj<T1; jj++)
			{
			nu_tmp += nu[N_tmp+jj];
			tmp_size += blasfeo_memsize_dmat(nx[N_tmp]+nu_tmp+1, nx[N_tmp+jj+1]);
			}
		Gamma_size = tmp_size>Gamma_size ? tmp_size : Gamma_size;

		// sLx : 1 => N-1
		for(jj=1; jj<T1; jj++)
			{
			tmp_size = blasfeo_memsize_dmat(nx[N_tmp+jj]+1, nx[N_tmp+jj]);
			work_space_sizes[0] = tmp_size>work_space_sizes[0] ? tmp_size : work_space_sizes[0];
			}

		// sBAbtL : 0 => N-2
		for(jj=0; jj<T1-1; jj++)
			{
			tmp_size = blasfeo_memsize_dmat(nu[N_tmp+jj]+nx[N_tmp+jj]+1, nx[N_tmp+1+jj]);
			work_space_sizes[1] = tmp_size>work_space_sizes[1] ? tmp_size : work_space_sizes[1];
			}

		// sl : 0 => N-1
		for(jj=0; jj<T1; jj++)
			{
			tmp_size = blasfeo_memsize_dvec(nu[N_tmp+jj]+nx[N_tmp+jj]);
			work_space_sizes[2] = tmp_size>work_space_sizes[2] ? tmp_size : work_space_sizes[2];
			}

		// sPbp : 0 => N-2
		for(jj=0; jj<T1-1; jj++)
			{
			tmp_size = blasfeo_memsize_dvec(nx[N_tmp+1+jj]);
			work_space_sizes[3] = tmp_size>work_space_sizes[3] ? tmp_size : work_space_sizes[3];
			}

		N_tmp += T1;

		}
	
	tmp_size = work_space_sizes[0] + work_space_sizes[1];
	tmp_size = work_space_sizes[2]+work_space_sizes[3]>tmp_size ? work_space_sizes[2]+work_space_sizes[3] : tmp_size;
	int size = Gamma_size+tmp_size;
	
	size = (size + 63) / 64 * 64; // make work space multiple of (typical) cache line size

	return size;

	}



int d_part_cond_memory_space_size_bytes_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int N2, int *nx2, int *nu2, int *nb2, int *ng2)
	{

	// early return
	if(N2==N)
		{
		return 0;
		}

	int ii;

	// data matrices
	int size = 0;
	for(ii=0; ii<N; ii++) // XXX not last stage !!!
		{
		// hsL
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]);
		}

	// make memory space multiple of (typical) cache line size
	size = (size + 63) / 64 * 64;

	return size;

	}



void d_part_cond_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, int N2, int *nx2, int *nu2, int *nb2, int **hidxb2, int *ng2, struct blasfeo_dmat *hsBAbt2, struct blasfeo_dmat *hsRSQrq2, struct blasfeo_dmat *hsDCt2, struct blasfeo_dvec *hsd2, void *memory, void *work, int *work_space_sizes)
	{

	int ii, jj, kk;
	int nu_tmp;

	// early return
	if(N2==N)
		{
		for(ii=0; ii<N; ii++)
			{
			for(jj=0; jj<nb[ii]; jj++) hidxb2[ii][jj] = hidxb[ii][jj];
			blasfeo_dgecp(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0, &hsBAbt2[ii], 0, 0);
			blasfeo_dgecp(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0, &hsRSQrq2[ii], 0, 0);
			blasfeo_dgecp(nu[ii]+nx[ii]+1, ng[ii], &hsDCt[ii], 0, 0, &hsDCt2[ii], 0, 0);
			blasfeo_dveccp(2*nb[ii]+2*ng[ii], &hsd[ii], 0, &hsd2[ii], 0);
			}
		return;
		}
	
	// sequential update not implemented
	if(N2>N)
		{
		printf("\nError: it must be N2<=N, sequential update not implemented\n\n");
		exit(1);
		}
	
	int N1 = N/N2; // (floor) horizon of small blocks
	int R1 = N - N2*N1; // the first R1 blocks have horizon N1+1
	int M1 = R1>0 ? N1+1 : N1; // (ceil) horizon of large blocks
	int T1; // horizon of current block
	int N_tmp = 0; // temporary sum of horizons

	// memory space
	char *c_ptr = (char *) memory;
	// recursion matrices (memory space)
	struct blasfeo_dmat hsL[N]; // XXX not last stage !!!
	for(ii=0; ii<N; ii++) // XXX not last stage !!!
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], (void *) c_ptr);
		c_ptr += hsL[ii].memsize;
		}

	// work space
	struct blasfeo_dmat hsGamma[M1];
	int tmp_i, tmp_size;

	// other stages
	N_tmp = 0;
	for(ii=0; ii<N2; ii++)
		{

		T1 = ii<R1 ? M1 : N1;

		c_ptr = (char *) work;

		// sGamma
		nu_tmp = nu[N_tmp+0];
		for(jj=0; jj<T1; jj++)
			{
			blasfeo_create_dmat(nx[N_tmp+0]+nu_tmp+1, nx[N_tmp+jj+1], &hsGamma[jj], (void *) c_ptr);
			c_ptr += hsGamma[jj].memsize;
			nu_tmp += nu[N_tmp+jj+1];
			}
		// sA
		// no c_ptr += ... : overwrite sA
		d_cond_BAbt_libstr(T1, &nx[N_tmp], &nu[N_tmp], &hsBAbt[N_tmp], hsGamma, &hsBAbt2[ii]);

		d_cond_RSQrq_libstr(T1-1, &nx[N_tmp], &nu[N_tmp], &hsBAbt[N_tmp], &hsRSQrq[N_tmp], hsGamma, &hsRSQrq2[ii], &hsL[N_tmp], (void *) c_ptr, work_space_sizes);

		d_cond_DCtd_libstr(T1-1, &nx[N_tmp], &nu[N_tmp], &nb[N_tmp], &hidxb[N_tmp], &ng[N_tmp], &hsDCt[N_tmp], &hsd[N_tmp], hsGamma, &hsDCt2[ii], &hsd2[ii], hidxb2[ii], (void *) c_ptr);
		N_tmp += T1;
//exit(1);
		}
	
	// no last stage

	return;

	}



void d_part_cond_rhs_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, int N2, int *nx2, int *nu2, int *nb2, int **hidxb2, int *ng2, struct blasfeo_dmat *hsBAbt2, struct blasfeo_dvec *hsb2, struct blasfeo_dmat *hsRSQrq2, struct blasfeo_dvec *hsrq2, struct blasfeo_dmat *hsDCt2, struct blasfeo_dvec *hsd2, void *memory, void *work, int *work_space_sizes)
	{

	int ii, jj, kk;

	// early return
	if(N2==N)
		{
		for(ii=0; ii<N; ii++)
			{
			for(jj=0; jj<nb[ii]; jj++) hidxb2[ii][jj] = hidxb[ii][jj];
			blasfeo_dgecp(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0, &hsBAbt2[ii], 0, 0);
			blasfeo_dveccp(nx[ii+1], &hsb[ii], 0, &hsb2[ii], 0);
			blasfeo_dgecp(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0, &hsRSQrq2[ii], 0, 0);
			blasfeo_dveccp(nu[ii]+nx[ii], &hsrq[ii], 0, &hsrq2[ii], 0);
			blasfeo_dgecp(nu[ii]+nx[ii]+1, ng[ii], &hsDCt[ii], 0, 0, &hsDCt2[ii], 0, 0);
			blasfeo_dveccp(2*nb[ii]+2*ng[ii], &hsd[ii], 0, &hsd2[ii], 0);
			}
		return;
		}
	
	// sequential update not implemented
	if(N2>N)
		{
		printf("\nError: it must be N2<=N, sequential update not implemented\n\n");
		exit(1);
		}
	
	int N1 = N/N2; // (floor) horizon of small blocks
	int R1 = N - N2*N1; // the first R1 blocks have horizon N1+1
	int M1 = R1>0 ? N1+1 : N1; // (ceil) horizon of large blocks
	int T1; // horizon of current block
	int N_tmp = 0; // temporary sum of horizons

	// memory space
	char *c_ptr = (char *) memory;
	// recursion matrices (memory space)
	struct blasfeo_dmat hsL[N]; // XXX not last stage !!!
	for(ii=0; ii<N; ii++) // XXX not last stage !!!
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], (void *) c_ptr);
		c_ptr += hsL[ii].memsize;
		}

	// work space
	struct blasfeo_dvec hsGammab[M1];
	int tmp_i, tmp_size;

	// other stages
	N_tmp = 0;
	for(ii=0; ii<N2; ii++)
		{

		T1 = ii<R1 ? M1 : N1;

		c_ptr = (char *) work;

		// sGamma
		for(jj=0; jj<T1; jj++)
			{
			blasfeo_create_dvec(nx[N_tmp+jj+1], &hsGammab[jj], (void *) c_ptr);
			c_ptr += hsGammab[jj].memsize;
			}
		d_cond_b_libstr(T1, &nx[N_tmp], &nu[N_tmp], &hsBAbt[N_tmp], &hsb[N_tmp], hsGammab, &hsb2[ii]);

		d_cond_rq_libstr(T1-1, &nx[N_tmp], &nu[N_tmp], &hsBAbt[N_tmp], &hsb[N_tmp], &hsrq[N_tmp], &hsL[N_tmp], hsGammab, &hsrq2[ii], (void *) c_ptr, work_space_sizes);

		d_cond_d_libstr(T1-1, &nx[N_tmp], &nu[N_tmp], &nb[N_tmp], &hidxb[N_tmp], &ng[N_tmp], &hsDCt[N_tmp], &hsd[N_tmp], hsGammab, &hsd2[ii], (void *) c_ptr);
		N_tmp += T1;
//exit(1);
		}
	
	// no last stage

	return;

	}



int d_part_expand_work_space_size_bytes_libstr(int N, int *nx, int *nu, int *nb, int *ng, int *work_space_sizes)
	{

	int ii, i_tmp;

	int nzM = blasfeo_memsize_dvec(nu[0]+nx[0]);
	int ngM = blasfeo_memsize_dvec(ng[0]);

	for(ii=1; ii<=N; ii++)
		{
		i_tmp = blasfeo_memsize_dvec(nu[ii]+nx[ii]);
		nzM = i_tmp>nzM ? i_tmp : nzM;
		i_tmp = blasfeo_memsize_dvec(ng[ii]);
		ngM = i_tmp>ngM ? i_tmp : ngM;
		}
	
	work_space_sizes[0] = nzM;
	work_space_sizes[1] = ngM;
	
	int size = work_space_sizes[0] + work_space_sizes[1];

	size = (size + 63) / 64 * 64; // make multiple of (typical) cache line size

	return size;

	}



void d_part_expand_solution_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hst, int N2, int *nx2, int *nu2, int *nb2, int **hidxb2, int *ng2, struct blasfeo_dvec *hsux2, struct blasfeo_dvec *hspi2, struct blasfeo_dvec *hslam2, struct blasfeo_dvec *hst2, void *work_space, int *work_space_sizes)
	{

	int ii, jj, ll;

	int nu0, nx0, nx1, nb0, ng0, nt0, nt2;

	struct blasfeo_dvec workvec[2];

	double *ptr_work0, *ptr_work1, *ptr_lam, *ptr_t, *ptr_lam2, *ptr_t2;

	char *c_ptr[2];
	c_ptr[0] = (char *) work_space;
	c_ptr[1] = c_ptr[0] + work_space_sizes[0];

	int N1 = N/N2; // (floor) horizon of small blocks
	int R1 = N - N2*N1; // the first R1 blocks have horizion N1+1
	int M1 = R1>0 ? N1+1 : N1; // (ceil) horizon of large blocks
	int T1; // horizon of current block
	int N_tmp, nu_tmp;
	int nbb2, nbg2, ngg2;
	int stg;

	// inputs & initial states
	N_tmp = 0;
	for(ii=0; ii<N2; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		nu_tmp = 0;
		// final stages: copy only input
		for(jj=0; jj<T1-1; jj++)
			{
			blasfeo_dveccp(nu[N_tmp+T1-1-jj], &hsux2[ii], nu_tmp, &hsux[N_tmp+T1-1-jj], 0);
			nu_tmp += nu[N_tmp+T1-1-jj];
			}
		// first stage: copy input and state
		blasfeo_dveccp(nu[N_tmp+0]+nx[N_tmp+0], &hsux2[ii], nu_tmp, &hsux[N_tmp+0], 0);
		//
		N_tmp += T1;
		}
	// no last stage

	// compute missing states by simulation within each block
	N_tmp = 0;
	for(ii=0; ii<N2; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		for(jj=0; jj<T1-1; jj++) // last stage is already there !!!
			{
			blasfeo_dgemv_t(nu[N_tmp+jj]+nx[N_tmp+jj], nx[N_tmp+jj+1], 1.0, &hsBAbt[N_tmp+jj], 0, 0, &hsux[N_tmp+jj], 0, 1.0, &hsb[N_tmp+jj], 0, &hsux[N_tmp+jj+1], nu[N_tmp+jj+1]);
			}
		//
		N_tmp += T1;
		}

	// slack variables and ineq lagrange multipliers
	N_tmp = 0;
	for(ii=0; ii<N2; ii++)
		{
		ptr_lam2 = hslam2[ii].pa;
		ptr_t2 = hst2[ii].pa;
		nbb2 = 0;
		nbg2 = 0;
		ngg2 = 0;
		nt2 = nb2[ii]+ng2[ii];
		T1 = ii<R1 ? M1 : N1;
		// final stages
		for(jj=0; jj<T1-1; jj++)
			{
			stg = N_tmp+T1-1-jj;
			nb0 = nb[stg];
			ng0 = ng[stg];
			nt0 = nb0 + ng0;
			ptr_lam = hslam[stg].pa;
			ptr_t = hst[stg].pa;
			for(ll=0; ll<nb[stg]; ll++)
				{
				if(hidxb[stg][ll]<nu[stg])
					{
					// box as box
					ptr_lam[0*nt0+ll] = ptr_lam2[0*nt2+nbb2];
					ptr_lam[1*nt0+ll] = ptr_lam2[1*nt2+nbb2];
					ptr_t[0*nt0+ll] = ptr_t2[0*nt2+nbb2];
					ptr_t[1*nt0+ll] = ptr_t2[1*nt2+nbb2];
					nbb2++;
					}
				else
					{
					// box as general XXX change when decide where nbg are placed wrt ng
					ptr_lam[0*nt0+ll] = ptr_lam2[nb2[ii]+0*nt2+nbg2];
					ptr_lam[1*nt0+ll] = ptr_lam2[nb2[ii]+1*nt2+nbg2];
					ptr_t[0*nt0+ll] = ptr_t2[nb2[ii]+0*nt2+nbg2];
					ptr_t[1*nt0+ll] = ptr_t2[nb2[ii]+1*nt2+nbg2];
					nbg2++;
					}
				}
			}
		for(jj=0; jj<T1-1; jj++)
			{
			stg = N_tmp+T1-1-jj;
			nb0 = nb[stg];
			ng0 = ng[stg];
			nt0 = nb0 + ng0;
			ptr_lam = hslam[stg].pa;
			ptr_t = hst[stg].pa;
			for(ll=0; ll<ng[stg]; ll++)
				{
				// general as general
				ptr_lam[nb0+0*nt0+ll] = ptr_lam2[nb2[ii]+0*nt2+nbg2+ngg2];
				ptr_lam[nb0+1*nt0+ll] = ptr_lam2[nb2[ii]+1*nt2+nbg2+ngg2];
				ptr_t[nb0+0*nt0+ll] = ptr_t2[nb2[ii]+0*nt2+nbg2+ngg2];
				ptr_t[nb0+1*nt0+ll] = ptr_t2[nb2[ii]+1*nt2+nbg2+ngg2];
				ngg2++;
				}
			}
		// first stage
		stg = N_tmp;
		nb0 = nb[stg];
		ng0 = ng[stg];
		nt0 = nb0 + ng0;
		// all box as box
		blasfeo_dveccp(nb0, &hslam2[ii], 0*nt2+nbb2, &hslam[stg], 0*nt0);
		blasfeo_dveccp(nb0, &hslam2[ii], 1*nt2+nbb2, &hslam[stg], 1*nt0);
		blasfeo_dveccp(nb0, &hst2[ii], 0*nt2+nbb2, &hst[stg], 0*nt0);
		blasfeo_dveccp(nb0, &hst2[ii], 1*nt2+nbb2, &hst[stg], 1*nt0);
		// first stage: general
		blasfeo_dveccp(ng0, &hslam2[ii], nb2[ii]+0*nt2+nbg2+ngg2, &hslam[stg], nb0+0*nt0);
		blasfeo_dveccp(ng0, &hslam2[ii], nb2[ii]+1*nt2+nbg2+ngg2, &hslam[stg], nb0+1*nt0);
		blasfeo_dveccp(ng0, &hst2[ii], nb2[ii]+0*nt2+nbg2+ngg2, &hst[stg], nb0+0*nt0);
		blasfeo_dveccp(ng0, &hst2[ii], nb2[ii]+1*nt2+nbg2+ngg2, &hst[stg], nb0+1*nt0);
		//
		N_tmp += T1;
		}
	// no last stage

	// lagrange multipliers of equality constraints
	// TODO avoid to multiply by R and B (i.e. the u part)
	N_tmp = 0;
	for(ii=0; ii<N2; ii++)
		{
		T1 = ii<R1 ? M1 : N1;
		// last stage: just copy
		blasfeo_dveccp(nx[N_tmp+T1], &hspi2[ii+1], 0, &hspi[N_tmp+T1], 0);
		// middle stages: backward simulation
		for(jj=0; jj<T1-1; jj++)
			{
			stg = N_tmp+T1-1-jj;
			nu0 = nu[stg];
			nx0 = nx[stg];
			nx1 = nx[stg+1];
			nb0 = nb[stg];
			ng0 = ng[stg];
			nt0 = nb0 + ng0;
			blasfeo_create_dvec(nu0+nx0, &workvec[0], (void *) c_ptr[0]);
			blasfeo_dveccp(nu0+nx0, &hsrq[stg], 0, &workvec[0], 0);
			ptr_work0 = workvec[0].pa;
			ptr_lam = hslam[stg].pa;
			for(ll=0; ll<nb0; ll++)
				ptr_work0[hidxb[stg][ll]] += - ptr_lam[0*nt0+ll] + ptr_lam[1*nt0+ll];
			blasfeo_dsymv_l(nu0+nx0, nu0+nx0, 1.0, &hsRSQrq[stg], 0, 0, &hsux[stg], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
			blasfeo_dgemv_n(nu0+nx0, nx1, 1.0, &hsBAbt[stg], 0, 0, &hspi[stg+1], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
			blasfeo_create_dvec(ng0, &workvec[1], (void *) c_ptr[1]);
			ptr_work1 = workvec[1].pa;
			for(ll=0; ll<ng0; ll++)
				ptr_work1[ll] = ptr_lam[nb0+1*nt0+ll] - ptr_lam[nb0+0*nt0+ll];
			blasfeo_dgemv_n(nu0+nx0, ng0, 1.0, &hsDCt[stg], 0, 0, &workvec[1], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
			blasfeo_dveccp(nx0, &workvec[0], nu0,  &hspi[stg], 0);
			}
		N_tmp += T1;
		}

	return;

	}



void d_cond_compute_problem_size_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int *nx2, int *nu2, int *nb2, int *ng2)
	{

	const int N2 = 1;

	int ii, jj;

	int nbb; // box constr that remain box constr
	int nbg; // box constr that becomes general constr
	nx2[0] = nx[0];
	nu2[0] = nu[0];
	nb2[0] = nb[0];
	ng2[0] = ng[0];
	for(ii=1; ii<=N; ii++)
		{
		nbb = 0;
		nbg = 0;
		for(jj=0; jj<nb[ii]; jj++)
			if(hidxb[ii][jj]<nu[ii])
				nbb++;
			else
				nbg++;
		nx2[0] += 0;
		nu2[0] += nu[ii];
		nb2[0] += nbb;
		ng2[0] += ng[ii] + nbg;
		}
	// last stage
	nx2[N2] = 0;
	nu2[N2] = 0;
	nb2[N2] = 0;
	ng2[N2] = 0;

	}



int d_cond_work_space_size_bytes_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int *nx2, int *nu2, int *nb2, int *ng2, int *work_space_sizes)
	{

	const int N2 = 1;

	int ii;
	int nu_tmp;

	int Gamma_size = 0;
	work_space_sizes[0] = 0;
	work_space_sizes[1] = 0;
	work_space_sizes[2] = 0;
	work_space_sizes[3] = 0;

	int tmp_size;

	// hsGamma // XXX size(hsGamma) > size(hsGammab) !!!!!
	nu_tmp = 0;
	tmp_size = 0;
	for(ii=0; ii<N; ii++)
		{
		nu_tmp += nu[ii];
		tmp_size += blasfeo_memsize_dmat(nx[0]+nu_tmp+1, nx[ii+1]);
		}
	Gamma_size = tmp_size>Gamma_size ? tmp_size : Gamma_size;

	// sLx : 1 => N
	for(ii=1; ii<=N; ii++)
		{
		tmp_size = blasfeo_memsize_dmat(nx[ii]+1, nx[ii]);
		work_space_sizes[0] = tmp_size>work_space_sizes[0] ? tmp_size : work_space_sizes[0];
		}

	// sBAbtL : 0 => N-1
	for(ii=0; ii<N; ii++)
		{
		tmp_size = blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nx[1+ii]);
		work_space_sizes[1] = tmp_size>work_space_sizes[1] ? tmp_size : work_space_sizes[1];
		}

	// sl : 0 => N
	for(ii=0; ii<=N; ii++)
		{
		tmp_size = blasfeo_memsize_dvec(nu[ii]+nx[ii]);
		work_space_sizes[2] = tmp_size>work_space_sizes[2] ? tmp_size : work_space_sizes[2];
		}

	// sPbp : 0 => N-1
	for(ii=0; ii<N; ii++)
		{
		tmp_size = blasfeo_memsize_dvec(nx[1+ii]);
		work_space_sizes[3] = tmp_size>work_space_sizes[3] ? tmp_size : work_space_sizes[3];
		}
	
	tmp_size = work_space_sizes[0] + work_space_sizes[1];
	tmp_size = work_space_sizes[2]+work_space_sizes[3]>tmp_size ? work_space_sizes[2]+work_space_sizes[3] : tmp_size;
	int size = Gamma_size+tmp_size;
	
	size = (size + 63) / 64 * 64; // make work space multiple of (typical) cache line size

	return size;

	}



int d_cond_memory_space_size_bytes_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int *nx2, int *nu2, int *nb2, int *ng2)
	{

	const int N2 = 1;

	int ii;

	// data matrices
	int size = 0;
	for(ii=0; ii<=N; ii++)
		{
		// hsL
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]);
		}

	// make memory space multiple of (typical) cache line size
	size = (size + 63) / 64 * 64;

	return size;

	}



void d_cond_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, int *nx2, int *nu2, int *nb2, int **hidxb2, int *ng2, struct blasfeo_dmat *hsBAbt2, struct blasfeo_dmat *hsRSQrq2, struct blasfeo_dmat *hsDCt2, struct blasfeo_dvec *hsd2, void *memory, void *work, int *work_space_sizes)
	{

	const int N2 = 1;

	int ii, jj, kk;
	int nu_tmp;

	// memory space
	char *c_ptr = (char *) memory;
	// recursion matrices (memory space)
	struct blasfeo_dmat hsL[N+1]; // XXX not last stage !!!
	for(ii=0; ii<=N; ii++) // XXX not last stage !!!
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], (void *) c_ptr);
		c_ptr += hsL[ii].memsize;
		}

	// work space
	struct blasfeo_dmat hsGamma[N];
	int tmp_i, tmp_size;

	c_ptr = (char *) work;

	// sGamma
	nu_tmp = nu[0];
	for(jj=0; jj<N; jj++)
		{
		blasfeo_create_dmat(nx[0]+nu_tmp+1, nx[jj+1], &hsGamma[jj], (void *) c_ptr);
		c_ptr += hsGamma[jj].memsize;
		nu_tmp += nu[jj+1];
		}
	// sA
	// no c_ptr += ... : overwrite sA
	// TODO avoid copying back BAbt2 !!!!!
	d_comp_Gamma_libstr(N, nx, nu, hsBAbt, hsGamma);

	d_cond_RSQrq_libstr(N, nx, nu, hsBAbt, hsRSQrq, hsGamma, &hsRSQrq2[0], hsL, (void *) c_ptr, work_space_sizes);

	d_cond_DCtd_libstr(N, nx, nu, nb, hidxb, ng, hsDCt, hsd, hsGamma, &hsDCt2[0], &hsd2[0], hidxb2[0], (void *) c_ptr);

	return;

	}



void d_cond_rhs_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, int *nx2, int *nu2, int *nb2, int **hidxb2, int *ng2, struct blasfeo_dmat *hsBAbt2, struct blasfeo_dvec *hsb2, struct blasfeo_dmat *hsRSQrq2, struct blasfeo_dvec *hsrq2, struct blasfeo_dmat *hsDCt2, struct blasfeo_dvec *hsd2, void *memory, void *work, int *work_space_sizes)
	{

	const int N2 = 1;

	int ii, jj, kk;

	// memory space
	char *c_ptr = (char *) memory;
	// recursion matrices (memory space)
	struct blasfeo_dmat hsL[N+1];
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], (void *) c_ptr);
		c_ptr += hsL[ii].memsize;
		}

	// work space
	struct blasfeo_dvec hsGammab[N];
	int tmp_i, tmp_size;

	c_ptr = (char *) work;

	// sGamma
	for(jj=0; jj<N; jj++)
		{
		blasfeo_create_dvec(nx[jj+1], &hsGammab[jj], (void *) c_ptr);
		c_ptr += hsGammab[jj].memsize;
		}
	d_comp_Gammab_libstr(N, nx, nu, hsBAbt, hsb, hsGammab);

	d_cond_rq_libstr(N, nx, nu, hsBAbt, hsb, hsrq, hsL, hsGammab, &hsrq2[0], (void *) c_ptr, work_space_sizes);

	d_cond_d_libstr(N, nx, nu, nb, hidxb, ng, hsDCt, hsd, hsGammab, &hsd2[0], (void *) c_ptr);
	
	return;

	}



int d_expand_work_space_size_bytes_libstr(int N, int *nx, int *nu, int *nb, int *ng, int *work_space_sizes)
	{

	int ii, i_tmp;

	int nzM = blasfeo_memsize_dvec(nu[0]+nx[0]);
	int ngM = blasfeo_memsize_dvec(ng[0]);

	for(ii=1; ii<=N; ii++)
		{
		i_tmp = blasfeo_memsize_dvec(nu[ii]+nx[ii]);
		nzM = i_tmp>nzM ? i_tmp : nzM;
		i_tmp = blasfeo_memsize_dvec(ng[ii]);
		ngM = i_tmp>ngM ? i_tmp : ngM;
		}
	
	work_space_sizes[0] = nzM;
	work_space_sizes[1] = ngM;
	
	int size = work_space_sizes[0] + work_space_sizes[1];

	size = (size + 63) / 64 * 64; // make multiple of (typical) cache line size

	return size;

	}



void d_expand_solution_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hst, int *nx2, int *nu2, int *nb2, int **hidxb2, int *ng2, struct blasfeo_dvec *hsux2, struct blasfeo_dvec *hspi2, struct blasfeo_dvec *hslam2, struct blasfeo_dvec *hst2, void *work_space, int *work_space_sizes)
	{

	const int N2 = 1;

	int ii, jj, ll;

	int nu0, nx0, nx1, nb0, ng0, nt0, nt2;

	int nu_tmp, stg, nbb2, nbg2, ngg2;

	struct blasfeo_dvec workvec[2];

	double *ptr_work0, *ptr_work1, *ptr_lam, *ptr_t, *ptr_lam2, *ptr_t2;

	char *c_ptr[2];
	c_ptr[0] = (char *) work_space;
	c_ptr[1] = c_ptr[0] + work_space_sizes[0];

	// inputs & initial states
	nu_tmp = 0;
	// final stages: copy only input
	for(jj=0; jj<N; jj++)
		{
		stg = N-jj;
		blasfeo_dveccp(nu[stg], &hsux2[0], nu_tmp, &hsux[stg], 0);
		nu_tmp += nu[stg];
		}
	// first stage: copy input and state
	blasfeo_dveccp(nu[0]+nx[0], &hsux2[0], nu_tmp, &hsux[0], 0);

	// compute missing states by simulation within each block
	for(jj=0; jj<N; jj++)
		{
		blasfeo_dgemv_t(nu[jj]+nx[jj], nx[jj+1], 1.0, &hsBAbt[jj], 0, 0, &hsux[jj], 0, 1.0, &hsb[jj], 0, &hsux[jj+1], nu[jj+1]);
		}

	// slack variables and ineq lagrange multipliers
	ptr_lam2 = hslam2[0].pa;
	ptr_t2 = hst2[0].pa;
	nbb2 = 0;
	nbg2 = 0;
	ngg2 = 0;
	nt2 = nb2[0]+ng2[0];
	// final stages: box
	for(jj=0; jj<N; jj++)
		{
		stg = N-jj;
		nb0 = nb[stg];
		ng0 = ng[stg];
		nt0 = nb0 + ng0;
		ptr_lam = hslam[stg].pa;
		ptr_t = hst[stg].pa;
		for(ll=0; ll<nb[stg]; ll++)
			{
			if(hidxb[stg][ll]<nu[stg])
				{
				// box as box
				ptr_lam[0*nt0+ll] = ptr_lam2[0*nt2+nbb2];
				ptr_lam[1*nt0+ll] = ptr_lam2[1*nt2+nbb2];
				ptr_t[0*nt0+ll] = ptr_t2[0*nt2+nbb2];
				ptr_t[1*nt0+ll] = ptr_t2[1*nt2+nbb2];
				nbb2++;
				}
			else
				{
				// box as general XXX change when decide where nbg are placed wrt ng
				ptr_lam[0*nt0+ll] = ptr_lam2[nb2[ii]+0*nt2+nbg2];
				ptr_lam[1*nt0+ll] = ptr_lam2[nb2[ii]+1*nt2+nbg2];
				ptr_t[0*nt0+ll] = ptr_t2[nb2[ii]+0*nt2+nbg2];
				ptr_t[1*nt0+ll] = ptr_t2[nb2[ii]+1*nt2+nbg2];
				nbg2++;
				}
			}
		}
	// final stages: general
	for(jj=0; jj<N; jj++)
		{
		stg = N-jj;
		nb0 = nb[stg];
		ng0 = ng[stg];
		nt0 = nb0 + ng0;
		ptr_lam = hslam[stg].pa;
		ptr_t = hst[stg].pa;
		for(ll=0; ll<ng[stg]; ll++)
			{
			// general as general
			ptr_lam[nb0+0*nt0+ll] = ptr_lam2[nb2[ii]+0*nt2+nbg2+ngg2];
			ptr_lam[nb0+1*nt0+ll] = ptr_lam2[nb2[ii]+1*nt2+nbg2+ngg2];
			ptr_t[nb0+0*nt0+ll] = ptr_t2[nb2[ii]+0*nt2+nbg2+ngg2];
			ptr_t[nb0+1*nt0+ll] = ptr_t2[nb2[ii]+1*nt2+nbg2+ngg2];
			ngg2++;
			}
		}
	// first stage
	nb0 = nb[0];
	ng0 = ng[0];
	nt0 = nb0 + ng0;
	// all box as box
	blasfeo_dveccp(nb0, &hslam2[0], 0*nt2+nbb2, &hslam[0], 0*nt0);
	blasfeo_dveccp(nb0, &hslam2[0], 1*nt2+nbb2, &hslam[0], 1*nt0);
	blasfeo_dveccp(nb0, &hst2[0], 0*nt2+nbb2, &hst[0], 0*nt0);
	blasfeo_dveccp(nb0, &hst2[0], 1*nt2+nbb2, &hst[0], 1*nt0);
	// first stage: general
	blasfeo_dveccp(ng0, &hslam2[0], nb2[0]+0*nt2+nbg2+ngg2, &hslam[0], nb0+0*nt0);
	blasfeo_dveccp(ng0, &hslam2[0], nb2[0]+1*nt2+nbg2+ngg2, &hslam[0], nb0+1*nt0);
	blasfeo_dveccp(ng0, &hst2[0], nb2[0]+0*nt2+nbg2+ngg2, &hst[0], nb0+0*nt0);
	blasfeo_dveccp(ng0, &hst2[0], nb2[0]+1*nt2+nbg2+ngg2, &hst[0], nb0+1*nt0);

	// lagrange multipliers of equality constraints
	// TODO avoid to multiply by R and B (i.e. the u part)
	// last stage
	stg = N;
	nu0 = nu[stg];
	nx0 = nx[stg];
	nb0 = nb[stg];
	ng0 = ng[stg];
	nt0 = nb0 + ng0;
	blasfeo_create_dvec(nu0+nx0, &workvec[0], (void *) c_ptr[0]);
	blasfeo_dveccp(nu0+nx0, &hsrq[stg], 0, &workvec[0], 0);
	ptr_work0 = workvec[0].pa;
	ptr_lam = hslam[stg].pa;
	for(ll=0; ll<nb0; ll++)
		ptr_work0[hidxb[stg][ll]] += - ptr_lam[0*nt0+ll] + ptr_lam[1*nt0+ll];
	blasfeo_dsymv_l(nu0+nx0, nu0+nx0, 1.0, &hsRSQrq[stg], 0, 0, &hsux[stg], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
	blasfeo_create_dvec(ng0, &workvec[1], (void *) c_ptr[1]);
	ptr_work1 = workvec[1].pa;
	for(ll=0; ll<ng0; ll++)
		ptr_work1[ll] = ptr_lam[nb0+1*nt0+ll] - ptr_lam[nb0+0*nt0+ll];
	blasfeo_dgemv_n(nu0+nx0, ng0, 1.0, &hsDCt[stg], 0, 0, &workvec[1], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
	blasfeo_dveccp(nx0, &workvec[0], nu0,  &hspi[stg], 0);
	// middle stages: backward simulation
	for(jj=0; jj<N-1; jj++)
		{
		stg = N-1-jj;
		nu0 = nu[stg];
		nx0 = nx[stg];
		nx1 = nx[stg+1];
		nb0 = nb[stg];
		ng0 = ng[stg];
		nt0 = nb0 + ng0;
		blasfeo_create_dvec(nu0+nx0, &workvec[0], (void *) c_ptr[0]);
		blasfeo_dveccp(nu0+nx0, &hsrq[stg], 0, &workvec[0], 0);
		ptr_work0 = workvec[0].pa;
		ptr_lam = hslam[stg].pa;
		for(ll=0; ll<nb0; ll++)
			ptr_work0[hidxb[stg][ll]] += - ptr_lam[0*nt0+ll] + ptr_lam[1*nt0+ll];
		blasfeo_dsymv_l(nu0+nx0, nu0+nx0, 1.0, &hsRSQrq[stg], 0, 0, &hsux[stg], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
		blasfeo_dgemv_n(nu0+nx0, nx1, 1.0, &hsBAbt[stg], 0, 0, &hspi[stg+1], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
		blasfeo_create_dvec(ng0, &workvec[1], (void *) c_ptr[1]);
		ptr_work1 = workvec[1].pa;
		for(ll=0; ll<ng0; ll++)
			ptr_work1[ll] = ptr_lam[nb0+1*nt0+ll] - ptr_lam[nb0+0*nt0+ll];
		blasfeo_dgemv_n(nu0+nx0, ng0, 1.0, &hsDCt[stg], 0, 0, &workvec[1], 0, 1.0, &workvec[0], 0, &workvec[0], 0);
		blasfeo_dveccp(nx0, &workvec[0], nu0,  &hspi[stg], 0);
		}

	return;

	}



#endif

