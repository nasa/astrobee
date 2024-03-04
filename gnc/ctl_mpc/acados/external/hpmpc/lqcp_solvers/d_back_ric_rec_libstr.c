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

#include <stdlib.h>
#include <stdio.h>

#ifdef BLASFEO

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#include <blasfeo_d_blas.h>



int d_back_ric_rec_work_space_size_bytes_libstr(int N, int *nx, int *nu, int *nb, int *ng)
	{

	int ii;

	// max sizes
	int nxM  = 0;
	int ngM = 0;
	int nuxM  = 0;
	int nxgM = ng[N];
	for(ii=0; ii<N; ii++)
		{
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		nuxM = nu[ii]+nx[ii]>nuxM ? nu[ii]+nx[ii]+1 : nuxM;
		nxgM = nx[ii+1]+ng[ii]>nxgM ? nx[ii+1]+ng[ii] : nxgM;
		}
	ii = N;
	nxM = nx[ii]>nxM ? nx[ii] : nxM;
	ngM = ng[ii]>ngM ? ng[ii] : ngM;
	nuxM = nu[ii]+nx[ii]>nuxM ? nu[ii]+nx[ii]+1 : nuxM;

	int size = 0;

	size += blasfeo_memsize_dmat(nuxM+1, nxgM); // ric_work_mat[0]
	if(ngM>0)
		size += blasfeo_memsize_dmat(nuxM, nxgM); // ric_work_mat[1]
	size += blasfeo_memsize_dvec(nxM); // ric_work_vec[0]

	// make multiple of (typical) cache line size
	size = (size+63)/64*64;

	return size;
	}



void d_back_ric_rec_sv_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int update_b, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, int update_q, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsQx, struct blasfeo_dvec *hsqx, struct blasfeo_dvec *hsux, int compute_pi, struct blasfeo_dvec *hspi, int compute_Pb, struct blasfeo_dvec *hsPb, struct blasfeo_dmat *hsL, void *work)
	{

	char *c_ptr;

	struct blasfeo_dmat hswork_mat_0, hswork_mat_1;
	struct blasfeo_dvec hswork_vec_0;

	int nn;

	// factorization and backward substitution

	// last stage
	if(nb[N]>0 | ng[N]>0 | update_q)
		{
		blasfeo_dtrcp_l(nu[N]+nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
		if(update_q)
			{
			blasfeo_drowin(nu[N]+nx[N], 1.0, &hsrq[N], 0, &hsL[N], nu[N]+nx[N], 0);
			}
		else
			{
			blasfeo_dgecp(1, nu[N]+nx[N], &hsRSQrq[N], nu[N]+nx[N], 0, &hsL[N], nu[N]+nx[N], 0);
			}
		if(nb[N]>0)
			{
			blasfeo_ddiaad_sp(nb[N], 1.0, &hsQx[N], 0, hidxb[N], &hsL[N], 0, 0);
			blasfeo_drowad_sp(nb[N], 1.0, &hsqx[N], 0, hidxb[N], &hsL[N], nu[N]+nx[N], 0);
			}
		if(ng[N]>0)
			{
			c_ptr = (char *) work;
			blasfeo_create_dmat(nu[N]+nx[N]+1, ng[N], &hswork_mat_0, (void *) c_ptr);
			c_ptr += hswork_mat_0.memsize;
			blasfeo_dgemm_nd(nu[N]+nx[N], ng[N], 1.0, &hsDCt[N], 0, 0, &hsQx[N], nb[N], 0.0, &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0);
			blasfeo_drowin(ng[N], 1.0, &hsqx[N], nb[N], &hswork_mat_0, nu[N]+nx[N], 0);
			blasfeo_dsyrk_dpotrf_ln_mn(nu[N]+nx[N]+1, nu[N]+nx[N], ng[N], &hswork_mat_0, 0, 0, &hsDCt[N], 0, 0, &hsL[N], 0, 0, &hsL[N], 0, 0);
			}
		else
			{
			blasfeo_dpotrf_l_mn(nu[N]+nx[N]+1, nu[N]+nx[N], &hsL[N], 0, 0, &hsL[N], 0, 0);
			}
		}
	else
		{
		blasfeo_dpotrf_l_mn(nu[N]+nx[N]+1, nu[N]+nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);
		}

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		c_ptr = (char *) work;
		blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn]+ng[N-nn-1], &hswork_mat_0, (void *) c_ptr);
		c_ptr += hswork_mat_0.memsize;
		if(ng[N-nn-1]>0)
			{
			blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1], nx[N-nn]+ng[N-nn-1], &hswork_mat_1, (void *) c_ptr);
			c_ptr += hswork_mat_1.memsize;
			}
		blasfeo_create_dvec(nx[N-nn], &hswork_vec_0, (void *) c_ptr);
		c_ptr += hswork_vec_0.memsize;
		if(update_b)
			{
			blasfeo_drowin(nx[N-nn], 1.0, &hsb[N-nn-1], 0, &hsBAbt[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
			}
		blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsBAbt[N-nn-1], 0, 0, &hswork_mat_0, 0, 0);
		if(compute_Pb)
			{
			blasfeo_drowex(nx[N-nn], 1.0, &hswork_mat_0, nu[N-nn-1]+nx[N-nn-1], 0, &hswork_vec_0, 0);
			blasfeo_dtrmv_lnn(nx[N-nn], nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hswork_vec_0, 0, &hsPb[N-nn], 0);
			}
		blasfeo_dgead(1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn]+nx[N-nn], nu[N-nn], &hswork_mat_0, nu[N-nn-1]+nx[N-nn-1], 0);
		if(nb[N-nn-1]>0 | ng[N-nn-1]>0 | update_q)
			{
			blasfeo_dtrcp_l(nu[N-nn-1]+nx[N-nn-1], &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
			if(update_q)
				{
				blasfeo_drowin(nu[N-nn-1]+nx[N-nn-1], 1.0, &hsrq[N-nn-1], 0, &hsL[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				}
			else
				{
				blasfeo_dgecp(1, nu[N-nn-1]+nx[N-nn-1], &hsRSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0, &hsL[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				}
			if(nb[N-nn-1]>0)
				{
				blasfeo_ddiaad_sp(nb[N-nn-1], 1.0, &hsQx[N-nn-1], 0, hidxb[N-nn-1], &hsL[N-nn-1], 0, 0);
				blasfeo_drowad_sp(nb[N-nn-1], 1.0, &hsqx[N-nn-1], 0, hidxb[N-nn-1], &hsL[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				}
			if(ng[N-nn-1]>0)
				{
				blasfeo_dgemm_nd(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], 1.0, &hsDCt[N-nn-1], 0, 0, &hsQx[N-nn-1], nb[N-nn-1], 0.0, &hswork_mat_0, 0, nx[N-nn], &hswork_mat_0, 0, nx[N-nn]);
				blasfeo_drowin(ng[N-nn-1], 1.0, &hsqx[N-nn-1], nb[N-nn-1], &hswork_mat_0, nu[N-nn-1]+nx[N-nn-1], nx[N-nn]);
				blasfeo_dgecp(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_1, 0, 0);
				blasfeo_dgecp(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], &hsDCt[N-nn-1], 0, 0, &hswork_mat_1, 0, nx[N-nn]);
				blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn]+ng[N-nn-1], &hswork_mat_0, 0, 0, &hswork_mat_1, 0, 0, &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
				}
			else
				{
				blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0, &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
				}
			}
		else
			{
			blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
			}
		}

	// forward substitution

	// first stage
	nn = 0;
	blasfeo_drowex(nu[nn]+nx[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
	blasfeo_dtrsv_ltn_mn(nu[nn]+nx[nn], nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_drowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
	blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
	if(compute_pi)
		{
		c_ptr = (char *) work;
		blasfeo_create_dvec(nx[nn+1], &hswork_vec_0, (void *) c_ptr);
		c_ptr += hswork_vec_0.memsize;
		blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn+1], 0);
		blasfeo_drowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec_0, 0);
		blasfeo_dtrmv_ltn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn+1], 0, &hspi[nn+1], 0);
		blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec_0, 0, &hspi[nn+1], 0, &hspi[nn+1], 0);
		blasfeo_dtrmv_lnn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn+1], 0, &hspi[nn+1], 0);
		}

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		blasfeo_drowex(nu[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
		blasfeo_dtrsv_ltn_mn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_drowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
		if(compute_pi)
			{
			c_ptr = (char *) work;
			blasfeo_create_dvec(nx[nn+1], &hswork_vec_0, (void *) c_ptr);
			c_ptr += hswork_vec_0.memsize;
			blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn+1], 0);
			blasfeo_drowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec_0, 0);
			blasfeo_dtrmv_ltn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn+1], 0, &hspi[nn+1], 0);
			blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec_0, 0, &hspi[nn+1], 0, &hspi[nn+1], 0);
			blasfeo_dtrmv_lnn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn+1], 0, &hspi[nn+1], 0);
			}
		}

	return;

	}



void d_back_ric_rec_trf_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsQx, struct blasfeo_dmat *hsL, void *work)
	{

	char *c_ptr;

	struct blasfeo_dmat hswork_mat_0, hswork_mat_1;

	int nn;

	// factorization

	// last stage
	if(nb[N]>0 | ng[N]>0)
		{
		blasfeo_dtrcp_l(nu[N]+nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);
		if(nb[N]>0)
			{
			blasfeo_ddiaad_sp(nb[N], 1.0, &hsQx[N], 0, hidxb[N], &hsL[N], 0, 0);
			}
		if(ng[N]>0)
			{
			c_ptr = (char *) work;
			blasfeo_create_dmat(nu[N]+nx[N], ng[N], &hswork_mat_0, (void *) c_ptr);
			c_ptr += hswork_mat_0.memsize;
			blasfeo_dgemm_nd(nu[N]+nx[N], ng[N], 1.0, &hsDCt[N], 0, 0, &hsQx[N], nb[N], 0.0, &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0);
			blasfeo_dsyrk_dpotrf_ln_mn(nu[N]+nx[N], nu[N]+nx[N], ng[N], &hswork_mat_0, 0, 0, &hsDCt[N], 0, 0, &hsL[N], 0, 0, &hsL[N], 0, 0);
			}
		else
			{
			blasfeo_dpotrf_l(nu[N]+nx[N], &hsL[N], 0, 0, &hsL[N], 0, 0);
			}
		}
	else
		{
		blasfeo_dpotrf_l(nu[N]+nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);
		}

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		c_ptr = (char *) work;
		blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1], nx[N-nn]+ng[N-nn-1], &hswork_mat_0, (void *) c_ptr);
		c_ptr += hswork_mat_0.memsize;
		if(ng[N-nn-1]>0)
			{
			blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1], nx[N-nn]+ng[N-nn-1], &hswork_mat_1, (void *) c_ptr);
			c_ptr += hswork_mat_1.memsize;
			}
		blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsBAbt[N-nn-1], 0, 0, &hswork_mat_0, 0, 0);
		if(nb[N-nn-1]>0 | ng[N-nn-1]>0)
			{
			blasfeo_dtrcp_l(nu[N-nn-1]+nx[N-nn-1], &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
			if(nb[N-nn-1]>0)
				{
				blasfeo_ddiaad_sp(nb[N-nn-1], 1.0, &hsQx[N-nn-1], 0, hidxb[N-nn-1], &hsL[N-nn-1], 0, 0);
				}
			if(ng[N-nn-1]>0)
				{
				blasfeo_dgemm_nd(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], 1.0, &hsDCt[N-nn-1], 0, 0, &hsQx[N-nn-1], nb[N], 0.0, &hswork_mat_0, 0, nx[N-nn], &hswork_mat_0, 0, nx[N-nn]);
				blasfeo_dgecp(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_1, 0, 0);
				blasfeo_dgecp(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], &hsDCt[N-nn-1], 0, 0, &hswork_mat_1, 0, nx[N-nn]);
				blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn]+ng[N-nn-1], &hswork_mat_0, 0, 0, &hswork_mat_1, 0, 0, &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
				}
			else
				{
				blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0, &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
				}
			}
		else
			{
			blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
			}
		}

	return;

	}



void d_back_ric_rec_trs_libstr(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsqx, struct blasfeo_dvec *hsux, int compute_pi, struct blasfeo_dvec *hspi, int compute_Pb, struct blasfeo_dvec *hsPb, struct blasfeo_dmat *hsL, void *work)
	{

	char *c_ptr;

	struct blasfeo_dvec hswork_vec_0;

	int nn;

	// backward substitution

	// last stage
	blasfeo_dveccp(nu[N]+nx[N], &hsrq[N], 0, &hsux[N], 0);
	if(nb[N]>0)
		{
		blasfeo_dvecad_sp(nb[N], 1.0, &hsqx[N], 0, idxb[N], &hsux[N], 0);
		}
	// general constraints
	if(ng[N]>0)
		{
		blasfeo_dgemv_n(nu[N]+nx[N], ng[N], 1.0, &hsDCt[N], 0, 0, &hsqx[N], nb[N], 1.0, &hsux[N], 0, &hsux[N], 0);
		}

	// middle stages
	for(nn=0; nn<N-1; nn++)
		{
		c_ptr = (char *) work;
		blasfeo_create_dvec(nx[N-nn], &hswork_vec_0, (void *) c_ptr);
		c_ptr += hswork_vec_0.memsize;
		if(compute_Pb)
			{
			blasfeo_dtrmv_ltn(nx[N-nn], nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, &hsPb[N-nn], 0);
			blasfeo_dtrmv_lnn(nx[N-nn], nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsPb[N-nn], 0, &hsPb[N-nn], 0);
			}
		blasfeo_dveccp(nu[N-nn-1]+nx[N-nn-1], &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
		if(nb[N-nn-1]>0)
			{
			blasfeo_dvecad_sp(nb[N-nn-1], 1.0, &hsqx[N-nn-1], 0, idxb[N-nn-1], &hsux[N-nn-1], 0);
			}
		if(ng[N-nn-1]>0)
			{
			blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], 1.0, &hsDCt[N-nn-1], 0, 0, &hsqx[N-nn-1], nb[N-nn-1], 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
			}
		blasfeo_daxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hsPb[N-nn], 0, &hswork_vec_0, 0);
		blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec_0, 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		blasfeo_dtrsv_lnn_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		}

	// first stage
	nn = N-1;
	c_ptr = (char *) work;
	blasfeo_create_dvec(nx[N-nn], &hswork_vec_0, (void *) c_ptr);
	c_ptr += hswork_vec_0.memsize;
	if(compute_Pb)
		{
		blasfeo_dtrmv_ltn(nx[N-nn], nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsb[N-nn-1], 0, &hsPb[N-nn], 0);
		blasfeo_dtrmv_lnn(nx[N-nn], nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hsPb[N-nn], 0, &hsPb[N-nn], 0);
		}
	blasfeo_dveccp(nu[N-nn-1]+nx[N-nn-1], &hsrq[N-nn-1], 0, &hsux[N-nn-1], 0);
	if(nb[N-nn-1]>0)
		{
		blasfeo_dvecad_sp(nb[N-nn-1], 1.0, &hsqx[N-nn-1], 0, idxb[N-nn-1], &hsux[N-nn-1], 0);
		}
	if(ng[N-nn-1]>0)
		{
		blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], 1.0, &hsDCt[N-nn-1], 0, 0, &hsqx[N-nn-1], nb[N-nn-1], 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
		}
	blasfeo_daxpy(nx[N-nn], 1.0, &hsux[N-nn], nu[N-nn], &hsPb[N-nn], 0, &hswork_vec_0, 0);
	blasfeo_dgemv_n(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], 1.0, &hsBAbt[N-nn-1], 0, 0, &hswork_vec_0, 0, 1.0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);
	blasfeo_dtrsv_lnn_mn(nu[N-nn-1]+nx[N-nn-1], nu[N-nn-1]+nx[N-nn-1], &hsL[N-nn-1], 0, 0, &hsux[N-nn-1], 0, &hsux[N-nn-1], 0);

	// first stage
	nn = 0;
	if(compute_pi)
		{
		blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn+1], 0);
		}
	blasfeo_dvecsc(nu[nn]+nx[nn], -1.0, &hsux[nn], 0);
	blasfeo_dtrsv_ltn_mn(nu[nn]+nx[nn], nu[nn]+nx[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
	blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
	if(compute_pi)
		{
		c_ptr = (char *) work;
		blasfeo_create_dvec(nx[nn+1], &hswork_vec_0, (void *) c_ptr);
		c_ptr += hswork_vec_0.memsize;
		blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hswork_vec_0, 0);
		blasfeo_dtrmv_ltn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec_0, 0, &hswork_vec_0, 0);
		blasfeo_dtrmv_lnn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec_0, 0, &hswork_vec_0, 0);
		blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec_0, 0, &hspi[nn+1], 0, &hspi[nn+1], 0);
		}

	// middle stages
	for(nn=1; nn<N; nn++)
		{
		if(compute_pi)
			{
			blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn+1], 0);
			}
		blasfeo_dvecsc(nu[nn], -1.0, &hsux[nn], 0);
		blasfeo_dtrsv_ltn_mn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsb[nn], 0, &hsux[nn+1], nu[nn+1]);
		if(compute_pi)
			{
			c_ptr = (char *) work;
			blasfeo_create_dvec(nx[nn+1], &hswork_vec_0, (void *) c_ptr);
			c_ptr += hswork_vec_0.memsize;
			blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hswork_vec_0, 0);
			blasfeo_dtrmv_ltn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec_0, 0, &hswork_vec_0, 0);
			blasfeo_dtrmv_lnn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hswork_vec_0, 0, &hswork_vec_0, 0);
			blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec_0, 0, &hspi[nn+1], 0, &hspi[nn+1], 0);
			}
		}

	return;

	}



void d_back_ric_rec_sv_back_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int update_b, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, int update_q, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsQx, struct blasfeo_dvec *hsqx, struct blasfeo_dvec *hsux, int compute_pi, struct blasfeo_dvec *hspi, int compute_Pb, struct blasfeo_dvec *hsPb, struct blasfeo_dmat *hsL, void *work)
	{

	char *c_ptr;

	struct blasfeo_dmat hswork_mat_0, hswork_mat_1;
	struct blasfeo_dvec hswork_vec_0;

	int nn;

	// factorization and backward substitution

	// last stage
	if(nb[N]>0 | ng[N]>0 | update_q)
		{
		blasfeo_dtrcp_l(nu[N]+nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0); // TODO blasfeo_dtrcp_l with m and n, for m>=n
		if(update_q)
			{
			blasfeo_drowin(nu[N]+nx[N], 1.0, &hsrq[N], 0, &hsL[N], nu[N]+nx[N], 0);
			}
		else
			{
			blasfeo_dgecp(1, nu[N]+nx[N], &hsRSQrq[N], nu[N]+nx[N], 0, &hsL[N], nu[N]+nx[N], 0);
			}
		if(nb[N]>0)
			{
			blasfeo_ddiaad_sp(nb[N], 1.0, &hsQx[N], 0, hidxb[N], &hsL[N], 0, 0);
			blasfeo_drowad_sp(nb[N], 1.0, &hsqx[N], 0, hidxb[N], &hsL[N], nu[N]+nx[N], 0);
			}
		if(ng[N]>0)
			{
			c_ptr = (char *) work;
			blasfeo_create_dmat(nu[N]+nx[N]+1, ng[N], &hswork_mat_0, (void *) c_ptr);
			c_ptr += hswork_mat_0.memsize;
			blasfeo_dgemm_nd(nu[N]+nx[N], ng[N], 1.0, &hsDCt[N], 0, 0, &hsQx[N], nb[N], 0.0, &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0);
			blasfeo_drowin(ng[N], 1.0, &hsqx[N], nb[N], &hswork_mat_0, nu[N]+nx[N], 0);
			blasfeo_dsyrk_dpotrf_ln_mn(nu[N]+nx[N]+1, nu[N]+nx[N], ng[N], &hswork_mat_0, 0, 0, &hsDCt[N], 0, 0, &hsL[N], 0, 0, &hsL[N], 0, 0);
			}
		else
			{
			blasfeo_dpotrf_l_mn(nu[N]+nx[N]+1, nu[N]+nx[N], &hsL[N], 0, 0, &hsL[N], 0, 0);
			}
		}
	else
		{
		blasfeo_dpotrf_l_mn(nu[N]+nx[N]+1, nu[N]+nx[N], &hsRSQrq[N], 0, 0, &hsL[N], 0, 0);
		}

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		c_ptr = (char *) work;
		blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn]+ng[N-nn-1], &hswork_mat_0, (void *) c_ptr);
		c_ptr += hswork_mat_0.memsize;
		if(ng[N-nn-1]>0)
			{
			blasfeo_create_dmat(nu[N-nn-1]+nx[N-nn-1], nx[N-nn]+ng[N-nn-1], &hswork_mat_1, (void *) c_ptr);
			c_ptr += hswork_mat_1.memsize;
			}
		blasfeo_create_dvec(nx[N-nn], &hswork_vec_0, (void *) c_ptr);
		c_ptr += hswork_vec_0.memsize;
		if(update_b)
			{
			blasfeo_drowin(nx[N-nn], 1.0, &hsb[N-nn-1], 0, &hsBAbt[N-nn-1], nu[N-nn-1]+nu[N-nn-1], 0);
			}
		blasfeo_dtrmm_rlnn(nu[N-nn-1]+nx[N-nn-1]+1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn], nu[N-nn], &hsBAbt[N-nn-1], 0, 0, &hswork_mat_0, 0, 0);
		if(compute_Pb)
			{
			blasfeo_drowex(nx[N-nn], 1.0, &hswork_mat_0, nu[N-nn-1]+nx[N-nn-1], 0, &hswork_vec_0, 0);
			blasfeo_dtrmv_lnn(nx[N-nn], nx[N-nn], &hsL[N-nn], nu[N-nn], nu[N-nn], &hswork_vec_0, 0, &hsPb[N-nn], 0);
			}
		blasfeo_dgead(1, nx[N-nn], 1.0, &hsL[N-nn], nu[N-nn]+nx[N-nn], nu[N-nn], &hswork_mat_0, nu[N-nn-1]+nx[N-nn-1], 0);
		if(nb[N-nn-1]>0 | ng[N-nn-1]>0 | update_q)
			{
			blasfeo_dtrcp_l(nu[N-nn-1]+nx[N-nn-1], &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
			if(update_q)
				{
				blasfeo_drowin(nu[N-nn-1]+nx[N-nn-1], 1.0, &hsrq[N-nn-1], 0, &hsL[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				}
			else
				{
				blasfeo_dgecp(1, nu[N-nn-1]+nx[N-nn-1], &hsRSQrq[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0, &hsL[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				}
			if(nb[N-nn-1]>0)
				{
				blasfeo_ddiaad_sp(nb[N-nn-1], 1.0, &hsQx[N-nn-1], 0, hidxb[N-nn-1], &hsL[N-nn-1], 0, 0);
				blasfeo_drowad_sp(nb[N-nn-1], 1.0, &hsqx[N-nn-1], 0, hidxb[N-nn-1], &hsL[N-nn-1], nu[N-nn-1]+nx[N-nn-1], 0);
				}
			if(ng[N-nn-1]>0)
				{
				blasfeo_dgemm_nd(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], 1.0, &hsDCt[N-nn-1], 0, 0, &hsQx[N-nn-1], nb[N], 0.0, &hswork_mat_0, 0, nx[N-nn], &hswork_mat_0, 0, nx[N-nn]);
				blasfeo_drowin(ng[N-nn-1], 1.0, &hsqx[N-nn-1], nb[N-nn-1], &hswork_mat_0, nu[N-nn-1]+nx[N-nn-1], nx[N-nn]);
				blasfeo_dgecp(nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_1, 0, 0);
				blasfeo_dgecp(nu[N-nn-1]+nx[N-nn-1], ng[N-nn-1], &hsDCt[N-nn-1], 0, 0, &hswork_mat_1, 0, nx[N-nn]);
				blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn]+ng[N-nn-1], &hswork_mat_0, 0, 0, &hswork_mat_1, 0, 0, &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
				}
			else
				{
				blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0, &hsL[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
				}
			}
		else
			{
			blasfeo_dsyrk_dpotrf_ln_mn(nu[N-nn-1]+nx[N-nn-1]+1, nu[N-nn-1]+nx[N-nn-1], nx[N-nn], &hswork_mat_0, 0, 0, &hswork_mat_0, 0, 0, &hsRSQrq[N-nn-1], 0, 0, &hsL[N-nn-1], 0, 0);
			}
		}


	return;

	}



void d_back_ric_rec_sv_forw_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int update_b, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, int update_q, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsQx, struct blasfeo_dvec *hsqx, struct blasfeo_dvec *hsux, int compute_pi, struct blasfeo_dvec *hspi, int compute_Pb, struct blasfeo_dvec *hsPb, struct blasfeo_dmat *hsL, void *work)
	{

	char *c_ptr;

	struct blasfeo_dvec hswork_vec_0;

	int nn;

	// forward substitution (without first stage)

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		blasfeo_drowex(nu[nn], -1.0, &hsL[nn], nu[nn]+nx[nn], 0, &hsux[nn], 0);
		blasfeo_dtrsv_ltn_mn(nu[nn]+nx[nn], nu[nn], &hsL[nn], 0, 0, &hsux[nn], 0, &hsux[nn], 0);
		blasfeo_drowex(nx[nn+1], 1.0, &hsBAbt[nn], nu[nn]+nx[nn], 0, &hsux[nn+1], nu[nn+1]);
		blasfeo_dgemv_t(nu[nn]+nx[nn], nx[nn+1], 1.0, &hsBAbt[nn], 0, 0, &hsux[nn], 0, 1.0, &hsux[nn+1], nu[nn+1], &hsux[nn+1], nu[nn+1]);
		if(compute_pi)
			{
			c_ptr = (char *) work;
			blasfeo_create_dvec(nx[nn+1], &hswork_vec_0, (void *) c_ptr);
			c_ptr += hswork_vec_0.memsize;
			blasfeo_dveccp(nx[nn+1], &hsux[nn+1], nu[nn+1], &hspi[nn+1], 0);
			blasfeo_drowex(nx[nn+1], 1.0, &hsL[nn+1], nu[nn+1]+nx[nn+1], nu[nn+1], &hswork_vec_0, 0);
			blasfeo_dtrmv_ltn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn+1], 0, &hspi[nn+1], 0);
			blasfeo_daxpy(nx[nn+1], 1.0, &hswork_vec_0, 0, &hspi[nn+1], 0, &hspi[nn+1], 0);
			blasfeo_dtrmv_lnn(nx[nn+1], nx[nn+1], &hsL[nn+1], nu[nn+1], nu[nn+1], &hspi[nn+1], 0, &hspi[nn+1], 0);
			}
		}

	return;

	}



#endif
