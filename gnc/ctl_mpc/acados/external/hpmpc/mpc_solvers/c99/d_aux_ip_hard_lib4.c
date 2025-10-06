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
#include <math.h>

#include "../../include/d_blas_aux.h"
#include "../../include/block_size.h"

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#endif
//#else
#include "../../include/blas_d.h"
//#endif

// initialize variables
void d_init_var_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, double **ux, double **pi, double **pDCt, double **db, double **t, double **lam, double mu0, int warm_start)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int jj, ll, ii;

	int nb0, pnb, ng0, png, cng;

	double
		*ptr_t, *ptr_lam, *ptr_db;

	double thr0 = 0.1; // minimum vale of t (minimum distance from a constraint)


	// cold start
	if(warm_start==0)
		{
		for(jj=0; jj<=N; jj++)
			{
			for(ll=0; ll<nu[jj]+nx[jj]; ll++)
				{
				ux[jj][ll] = 0.0;
				}
			}
		}


	// check bounds & initialize multipliers
	for(jj=0; jj<=N; jj++)
		{
		nb0 = nb[jj];
		pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints
		for(ll=0; ll<nb0; ll++)
			{
#if 1
			t[jj][ll]     = - db[jj][ll]     + ux[jj][idxb[jj][ll]];
			t[jj][pnb+ll] =   db[jj][pnb+ll] - ux[jj][idxb[jj][ll]];
			if(t[jj][ll] < thr0)
				{
				if(t[jj][pnb+ll] < thr0)
					{
					ux[jj][idxb[jj][ll]] = ( - db[jj][pnb+ll] + db[jj][ll])*0.5;
					t[jj][ll]     = thr0; //- db[jj][ll]     + ux[jj][idxb[jj][ll]];
					t[jj][pnb+ll] = thr0; //  db[jj][pnb+ll] - ux[jj][idxb[jj][ll]];
					}
				else
					{
					t[jj][ll] = thr0;
					ux[jj][idxb[jj][ll]] = db[jj][ll] + thr0;
					}
				}
			else if(t[jj][pnb+ll] < thr0)
				{
				t[jj][pnb+ll] = thr0;
				ux[jj][idxb[jj][ll]] = db[jj][pnb+ll] - thr0;
				}
#else
			t[jj][ll]     = thr0;
			t[jj][pnb+ll] = thr0;
#endif
			lam[jj][ll]     = mu0/t[jj][ll];
			lam[jj][pnb+ll] = mu0/t[jj][pnb+ll];
			}
		}


	// initialize pi
	for(jj=0; jj<N; jj++)
		for(ll=0; ll<nx[jj+1]; ll++)
			pi[jj][ll] = 0.0; // initialize multipliers to zero


	// TODO find a better way to initialize general constraints
	for(jj=0; jj<=N; jj++)
		{
		nb0 = nb[jj];
		pnb = (nb0+bs-1)/bs*bs;
		ng0 = ng[jj];
		png = (ng0+bs-1)/bs*bs;
		cng = (ng0+ncl-1)/ncl*ncl;
		if(ng0>0)
			{
			ptr_t   = t[jj];
			ptr_lam = lam[jj];
			ptr_db  = db[jj];
#ifdef BLASFEO
			dgemv_t_lib(nu[jj]+nx[jj], ng0, 1.0, pDCt[jj], cng, ux[jj], 0.0, ptr_t+2*pnb, ptr_t+2*pnb);
#else
			dgemv_t_lib(nu[jj]+nx[jj], ng0, pDCt[jj], cng, ux[jj], 0, ptr_t+2*pnb, ptr_t+2*pnb);
#endif
			for(ll=2*pnb; ll<2*pnb+ng0; ll++)
				{
				ptr_t[ll+png] = - ptr_t[ll];
				ptr_t[ll]      += - ptr_db[ll];
				ptr_t[ll+png] += ptr_db[ll+png];
				ptr_t[ll]      = fmax( thr0, ptr_t[ll] );
				ptr_t[png+ll] = fmax( thr0, ptr_t[png+ll] );
				ptr_lam[ll]      = mu0/ptr_t[ll];
				ptr_lam[png+ll] = mu0/ptr_t[png+ll];
				}
			}
		}

	}



// initialize variables (single newton step)
void d_init_var_mpc_hard_tv_single_newton(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, double **ux, double **pi, double **pDCt, double **db, double **t, double **lam, double **ux0, double **pi0, double **lam0, double **t0)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int jj, ll, ii;

	int nb0, pnb, ng0, png, cng;

	double
		*ptr_t, *ptr_lam, *ptr_db;

	// initialize states and controls
	for(jj=0; jj<=N; jj++)
		{
		for(ll=0; ll<nu[jj]+nx[jj]; ll++)
			{
			ux[jj][ll] = ux0[jj][ll];
			}
		}

	// check bounds & initialize multipliers
	for(jj=0; jj<=N; jj++)
		{
		nb0 = nb[jj];
		pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints
		for(ll=0; ll<nb0; ll++)
			{
			lam[jj][ll]     = lam0[jj][ll];
			lam[jj][pnb+ll] = lam0[jj][nb0+ll];
			t[jj][ll]     = t0[jj][ll];
			t[jj][pnb+ll] = t0[jj][nb0+ll];
			}
		}

	// initialize equality multipliers
	for(jj=0; jj<N; jj++)
		for(ll=0; ll<nx[jj+1]; ll++)
			pi[jj][ll] = pi0[jj][ll];


	// TODO find a better way to initialize general constraints
	for(jj=0; jj<=N; jj++)
		{
		nb0 = nb[jj];
		pnb = (nb0+bs-1)/bs*bs;
		ng0 = ng[jj];
		png = (ng0+bs-1)/bs*bs;
		cng = (ng0+ncl-1)/ncl*ncl;
		if(ng0>0)
			{
			printf("General constraints not supported yet!!");
			exit(1);
			}
		}
	}



// IPM with no residuals

void d_update_hessian_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double **db, double sigma_mu, double **t, double **t_inv, double **lam, double **lamt, double **dlam, double **Qx, double **qx)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png;

	double temp0, temp1;

	double
		*ptr_db, *ptr_Qx, *ptr_qx,
		*ptr_t, *ptr_lam, *ptr_lamt, *ptr_dlam, *ptr_tinv;

	int ii, jj, bs0;

	for(jj=0; jj<=N; jj++)
		{

		ptr_t     = t[jj];
		ptr_lam   = lam[jj];
		ptr_lamt  = lamt[jj];
		ptr_dlam  = dlam[jj];
		ptr_tinv  = t_inv[jj];
		ptr_db    = db[jj];
		ptr_Qx    = Qx[jj];
		ptr_qx    = qx[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<nb0-3; ii+=4)
				{

				ptr_tinv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_tinv[ii+pnb+0] = 1.0/ptr_t[ii+pnb+0];
				ptr_lamt[ii+0] = ptr_lam[ii+0]*ptr_tinv[ii+0];
				ptr_lamt[ii+pnb+0] = ptr_lam[ii+pnb+0]*ptr_tinv[ii+pnb+0];
				ptr_dlam[ii+0] = ptr_tinv[ii+0]*sigma_mu; // !!!!!
				ptr_dlam[ii+pnb+0] = ptr_tinv[ii+pnb+0]*sigma_mu; // !!!!!
				ptr_Qx[ii+0] = ptr_lamt[ii+0] + ptr_lamt[ii+pnb+0];
				ptr_qx[ii+0] = ptr_lam[ii+pnb+0] - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] + ptr_dlam[ii+pnb+0] - ptr_lam[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0] - ptr_dlam[ii+0];

				ptr_tinv[ii+1] = 1.0/ptr_t[ii+1];
				ptr_tinv[ii+pnb+1] = 1.0/ptr_t[ii+pnb+1];
				ptr_lamt[ii+1] = ptr_lam[ii+1]*ptr_tinv[ii+1];
				ptr_lamt[ii+pnb+1] = ptr_lam[ii+pnb+1]*ptr_tinv[ii+pnb+1];
				ptr_dlam[ii+1] = ptr_tinv[ii+1]*sigma_mu; // !!!!!
				ptr_dlam[ii+pnb+1] = ptr_tinv[ii+pnb+1]*sigma_mu; // !!!!!
				ptr_Qx[ii+1] = ptr_lamt[ii+1] + ptr_lamt[ii+pnb+1];
				ptr_qx[ii+1] = ptr_lam[ii+pnb+1] - ptr_lamt[ii+pnb+1]*ptr_db[ii+pnb+1] + ptr_dlam[ii+pnb+1] - ptr_lam[ii+1] - ptr_lamt[ii+1]*ptr_db[ii+1] - ptr_dlam[ii+1];

				ptr_tinv[ii+2] = 1.0/ptr_t[ii+2];
				ptr_tinv[ii+pnb+2] = 1.0/ptr_t[ii+pnb+2];
				ptr_lamt[ii+2] = ptr_lam[ii+2]*ptr_tinv[ii+2];
				ptr_lamt[ii+pnb+2] = ptr_lam[ii+pnb+2]*ptr_tinv[ii+pnb+2];
				ptr_dlam[ii+2] = ptr_tinv[ii+2]*sigma_mu; // !!!!!
				ptr_dlam[ii+pnb+2] = ptr_tinv[ii+pnb+2]*sigma_mu; // !!!!!
				ptr_Qx[ii+2] = ptr_lamt[ii+2] + ptr_lamt[ii+pnb+2];
				ptr_qx[ii+2] = ptr_lam[ii+pnb+2] - ptr_lamt[ii+pnb+2]*ptr_db[ii+pnb+2] + ptr_dlam[ii+pnb+2] - ptr_lam[ii+2] - ptr_lamt[ii+2]*ptr_db[ii+2] - ptr_dlam[ii+2];

				ptr_tinv[ii+3] = 1.0/ptr_t[ii+3];
				ptr_tinv[ii+pnb+3] = 1.0/ptr_t[ii+pnb+3];
				ptr_lamt[ii+3] = ptr_lam[ii+3]*ptr_tinv[ii+3];
				ptr_lamt[ii+pnb+3] = ptr_lam[ii+pnb+3]*ptr_tinv[ii+pnb+3];
				ptr_dlam[ii+3] = ptr_tinv[ii+3]*sigma_mu; // !!!!!
				ptr_dlam[ii+pnb+3] = ptr_tinv[ii+pnb+3]*sigma_mu; // !!!!!
				ptr_Qx[ii+3] = ptr_lamt[ii+3] + ptr_lamt[ii+pnb+3];
				ptr_qx[ii+3] = ptr_lam[ii+pnb+3] - ptr_lamt[ii+pnb+3]*ptr_db[ii+pnb+3] + ptr_dlam[ii+pnb+3] - ptr_lam[ii+3] - ptr_lamt[ii+3]*ptr_db[ii+3] - ptr_dlam[ii+3];

				}
			for(; ii<nb0; ii++)
				{

				ptr_tinv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_tinv[ii+pnb+0] = 1.0/ptr_t[ii+pnb+0];
				ptr_lamt[ii+0] = ptr_lam[ii+0]*ptr_tinv[ii+0];
				ptr_lamt[ii+pnb+0] = ptr_lam[ii+pnb+0]*ptr_tinv[ii+pnb+0];
				ptr_dlam[ii+0] = ptr_tinv[ii+0]*sigma_mu; // !!!!!
				ptr_dlam[ii+pnb+0] = ptr_tinv[ii+pnb+0]*sigma_mu; // !!!!!
				ptr_Qx[ii] = ptr_lamt[ii+0] + ptr_lamt[ii+pnb+0];
				ptr_qx[ii] = ptr_lam[ii+pnb+0] - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] + ptr_dlam[ii+pnb+0] - ptr_lam[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0] - ptr_dlam[ii+0];

				}

			ptr_t     += 2*pnb;
			ptr_lam   += 2*pnb;
			ptr_lamt  += 2*pnb;
			ptr_dlam  += 2*pnb;
			ptr_tinv  += 2*pnb;
			ptr_db    += 2*pnb;
			ptr_Qx    += pnb;
			ptr_qx    += pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			png = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

			for(ii=0; ii<ng0-3; ii+=4)
				{

				ptr_tinv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_tinv[ii+png+0] = 1.0/ptr_t[ii+png+0];
				ptr_lamt[ii+0] = ptr_lam[ii+0]*ptr_tinv[ii+0];
				ptr_lamt[ii+png+0] = ptr_lam[ii+png+0]*ptr_tinv[ii+png+0];
				ptr_dlam[ii+0] = ptr_tinv[ii+0]*sigma_mu; // !!!!!
				ptr_dlam[ii+png+0] = ptr_tinv[ii+png+0]*sigma_mu; // !!!!!
				ptr_Qx[ii+0] = ptr_lamt[ii+0] + ptr_lamt[ii+png+0];
				ptr_qx[ii+0] =  ptr_lam[ii+png+0] - ptr_lamt[ii+png+0]*ptr_db[ii+png+0] + ptr_dlam[ii+png+0] - ptr_lam[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0] - ptr_dlam[ii+0];

				ptr_tinv[ii+1] = 1.0/ptr_t[ii+1];
				ptr_tinv[ii+png+1] = 1.0/ptr_t[ii+png+1];
				ptr_lamt[ii+1] = ptr_lam[ii+1]*ptr_tinv[ii+1];
				ptr_lamt[ii+png+1] = ptr_lam[ii+png+1]*ptr_tinv[ii+png+1];
				ptr_dlam[ii+1] = ptr_tinv[ii+1]*sigma_mu; // !!!!!
				ptr_dlam[ii+png+1] = ptr_tinv[ii+png+1]*sigma_mu; // !!!!!
				ptr_Qx[ii+1] = ptr_lamt[ii+1] + ptr_lamt[ii+png+1];
				ptr_qx[ii+1] =  ptr_lam[ii+png+1] - ptr_lamt[ii+png+1]*ptr_db[ii+png+1] + ptr_dlam[ii+png+1] - ptr_lam[ii+1] - ptr_lamt[ii+1]*ptr_db[ii+1] - ptr_dlam[ii+1];

				ptr_tinv[ii+2] = 1.0/ptr_t[ii+2];
				ptr_tinv[ii+png+2] = 1.0/ptr_t[ii+png+2];
				ptr_lamt[ii+2] = ptr_lam[ii+2]*ptr_tinv[ii+2];
				ptr_lamt[ii+png+2] = ptr_lam[ii+png+2]*ptr_tinv[ii+png+2];
				ptr_dlam[ii+2] = ptr_tinv[ii+2]*sigma_mu; // !!!!!
				ptr_dlam[ii+png+2] = ptr_tinv[ii+png+2]*sigma_mu; // !!!!!
				ptr_Qx[ii+2] = ptr_lamt[ii+2] + ptr_lamt[ii+png+2];
				ptr_qx[ii+2] =  ptr_lam[ii+png+2] - ptr_lamt[ii+png+2]*ptr_db[ii+png+2] + ptr_dlam[ii+png+2] - ptr_lam[ii+2] - ptr_lamt[ii+2]*ptr_db[ii+2] - ptr_dlam[ii+2];

				ptr_tinv[ii+3] = 1.0/ptr_t[ii+3];
				ptr_tinv[ii+png+3] = 1.0/ptr_t[ii+png+3];
				ptr_lamt[ii+3] = ptr_lam[ii+3]*ptr_tinv[ii+3];
				ptr_lamt[ii+png+3] = ptr_lam[ii+png+3]*ptr_tinv[ii+png+3];
				ptr_dlam[ii+3] = ptr_tinv[ii+3]*sigma_mu; // !!!!!
				ptr_dlam[ii+png+3] = ptr_tinv[ii+png+3]*sigma_mu; // !!!!!
				ptr_Qx[ii+3] = ptr_lamt[ii+3] + ptr_lamt[ii+png+3];
				ptr_qx[ii+3] =  ptr_lam[ii+png+3] - ptr_lamt[ii+png+3]*ptr_db[ii+png+3] + ptr_dlam[ii+png+3] - ptr_lam[ii+3] - ptr_lamt[ii+3]*ptr_db[ii+3] - ptr_dlam[ii+3];

				}
			for(; ii<ng0; ii++)
				{

				ptr_tinv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_tinv[ii+png+0] = 1.0/ptr_t[ii+png+0];
				ptr_lamt[ii+0] = ptr_lam[ii+0]*ptr_tinv[ii+0];
				ptr_lamt[ii+png+0] = ptr_lam[ii+png+0]*ptr_tinv[ii+png+0];
				ptr_dlam[ii+0] = ptr_tinv[ii+0]*sigma_mu; // !!!!!
				ptr_dlam[ii+png+0] = ptr_tinv[ii+png+0]*sigma_mu; // !!!!!
				ptr_Qx[ii+0] = ptr_lamt[ii+0] + ptr_lamt[ii+png+0];
				ptr_qx[ii+0] =  ptr_lam[ii+png+0] - ptr_lamt[ii+png+0]*ptr_db[ii+png+0] + ptr_dlam[ii+png+0] - ptr_lam[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0] - ptr_dlam[ii+0];

				}

			}

		}

	}



void d_update_gradient_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double sigma_mu, double **dt, double **dlam, double **t_inv, double **qx)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii, jj;

	int nb0, pnb, ng0, png;

	double
		*ptr_dlam, *ptr_t_inv, *ptr_dt, *ptr_pl2, *ptr_qx;

	for(jj=0; jj<=N; jj++)
		{

		ptr_dlam  = dlam[jj];
		ptr_dt    = dt[jj];
		ptr_t_inv = t_inv[jj];
		ptr_qx    = qx[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<nb0-3; ii+=4)
				{
				ptr_dlam[0*pnb+ii+0] = ptr_t_inv[0*pnb+ii+0]*(sigma_mu - ptr_dlam[0*pnb+ii+0]*ptr_dt[0*pnb+ii+0]);
				ptr_dlam[1*pnb+ii+0] = ptr_t_inv[1*pnb+ii+0]*(sigma_mu - ptr_dlam[1*pnb+ii+0]*ptr_dt[1*pnb+ii+0]);
				ptr_qx[ii+0] += ptr_dlam[1*pnb+ii+0] - ptr_dlam[0*pnb+ii+0];

				ptr_dlam[0*pnb+ii+1] = ptr_t_inv[0*pnb+ii+1]*(sigma_mu - ptr_dlam[0*pnb+ii+1]*ptr_dt[0*pnb+ii+1]);
				ptr_dlam[1*pnb+ii+1] = ptr_t_inv[1*pnb+ii+1]*(sigma_mu - ptr_dlam[1*pnb+ii+1]*ptr_dt[1*pnb+ii+1]);
				ptr_qx[ii+1] += ptr_dlam[1*pnb+ii+1] - ptr_dlam[0*pnb+ii+1];

				ptr_dlam[0*pnb+ii+2] = ptr_t_inv[0*pnb+ii+2]*(sigma_mu - ptr_dlam[0*pnb+ii+2]*ptr_dt[0*pnb+ii+2]);
				ptr_dlam[1*pnb+ii+2] = ptr_t_inv[1*pnb+ii+2]*(sigma_mu - ptr_dlam[1*pnb+ii+2]*ptr_dt[1*pnb+ii+2]);
				ptr_qx[ii+2] += ptr_dlam[1*pnb+ii+2] - ptr_dlam[0*pnb+ii+2];

				ptr_dlam[0*pnb+ii+3] = ptr_t_inv[0*pnb+ii+3]*(sigma_mu - ptr_dlam[0*pnb+ii+3]*ptr_dt[0*pnb+ii+3]);
				ptr_dlam[1*pnb+ii+3] = ptr_t_inv[1*pnb+ii+3]*(sigma_mu - ptr_dlam[1*pnb+ii+3]*ptr_dt[1*pnb+ii+3]);
				ptr_qx[ii+3] += ptr_dlam[1*pnb+ii+3] - ptr_dlam[0*pnb+ii+3];
				}
			for(; ii<nb0; ii++)
				{
				ptr_dlam[0*pnb+ii+0] = ptr_t_inv[0*pnb+ii+0]*(sigma_mu - ptr_dlam[0*pnb+ii+0]*ptr_dt[0*pnb+ii+0]);
				ptr_dlam[1*pnb+ii+0] = ptr_t_inv[1*pnb+ii+0]*(sigma_mu - ptr_dlam[1*pnb+ii+0]*ptr_dt[1*pnb+ii+0]);
				ptr_qx[ii+0] += ptr_dlam[1*pnb+ii+0] - ptr_dlam[0*pnb+ii+0];
				}

			ptr_dlam  += 2*pnb;
			ptr_dt    += 2*pnb;
			ptr_t_inv += 2*pnb;
			ptr_qx    += pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			png  = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

			for(ii=0; ii<ng0-3; ii+=4)
				{
				ptr_dlam[0*png+ii+0] = ptr_t_inv[0*png+ii+0]*(sigma_mu - ptr_dlam[0*png+ii+0]*ptr_dt[0*png+ii+0]);
				ptr_dlam[1*png+ii+0] = ptr_t_inv[1*png+ii+0]*(sigma_mu - ptr_dlam[1*png+ii+0]*ptr_dt[1*png+ii+0]);
				ptr_qx[ii+0] += ptr_dlam[1*png+ii+0] - ptr_dlam[0*png+ii+0];

				ptr_dlam[0*png+ii+1] = ptr_t_inv[0*png+ii+1]*(sigma_mu - ptr_dlam[0*png+ii+1]*ptr_dt[0*png+ii+1]);
				ptr_dlam[1*png+ii+1] = ptr_t_inv[1*png+ii+1]*(sigma_mu - ptr_dlam[1*png+ii+1]*ptr_dt[1*png+ii+1]);
				ptr_qx[ii+1] += ptr_dlam[1*png+ii+1] - ptr_dlam[0*png+ii+1];

				ptr_dlam[0*png+ii+2] = ptr_t_inv[0*png+ii+2]*(sigma_mu - ptr_dlam[0*png+ii+2]*ptr_dt[0*png+ii+2]);
				ptr_dlam[1*png+ii+2] = ptr_t_inv[1*png+ii+2]*(sigma_mu - ptr_dlam[1*png+ii+2]*ptr_dt[1*png+ii+2]);
				ptr_qx[ii+2] += ptr_dlam[1*png+ii+2] - ptr_dlam[0*png+ii+2];

				ptr_dlam[0*png+ii+3] = ptr_t_inv[0*png+ii+3]*(sigma_mu - ptr_dlam[0*png+ii+3]*ptr_dt[0*png+ii+3]);
				ptr_dlam[1*png+ii+3] = ptr_t_inv[1*png+ii+3]*(sigma_mu - ptr_dlam[1*png+ii+3]*ptr_dt[1*png+ii+3]);
				ptr_qx[ii+3] += ptr_dlam[1*png+ii+3] - ptr_dlam[0*png+ii+3];

				}
			for(; ii<ng0; ii++)
				{
				ptr_dlam[0*png+ii+0] = ptr_t_inv[0*png+ii+0]*(sigma_mu - ptr_dlam[0*png+ii+0]*ptr_dt[0*png+ii+0]);
				ptr_dlam[1*png+ii+0] = ptr_t_inv[1*png+ii+0]*(sigma_mu - ptr_dlam[1*png+ii+0]*ptr_dt[1*png+ii+0]);
				ptr_qx[ii+0] += ptr_dlam[1*png+ii+0] - ptr_dlam[0*png+ii+0];
				}

			}

		}

	}



void d_compute_alpha_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, double *ptr_alpha, double **t, double **dt, double **lam, double **dlam, double **lamt, double **dux, double **pDCt, double **db)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nb0, pnb, ng0, png, cng;

	double alpha = ptr_alpha[0];

	double
		*ptr_db, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lamt, *ptr_lam, *ptr_dlam;

	int
		*ptr_idxb;

	int jj, ll;

	for(jj=0; jj<=N; jj++)
		{

		ptr_db   = db[jj];
		ptr_dux  = dux[jj];
		ptr_t    = t[jj];
		ptr_dt   = dt[jj];
		ptr_lamt = lamt[jj];
		ptr_lam  = lam[jj];
		ptr_dlam = dlam[jj];
		ptr_idxb = idxb[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb = (nb0+bs-1)/bs*bs;

			// box constraints
			for(ll=0; ll<nb0; ll++)
				{

				ptr_dt[ll+0]   =   ptr_dux[ptr_idxb[ll]] - ptr_db[ll+0]   - ptr_t[ll+0];
				ptr_dt[ll+pnb] = - ptr_dux[ptr_idxb[ll]] + ptr_db[ll+pnb] - ptr_t[ll+pnb];
				ptr_dlam[ll+0]   -= ptr_lamt[ll+0]   * ptr_dt[ll+0]   + ptr_lam[ll+0];
				ptr_dlam[ll+pnb] -= ptr_lamt[ll+pnb] * ptr_dt[ll+pnb] + ptr_lam[ll+pnb];
				if( -alpha*ptr_dlam[ll+0]>ptr_lam[ll+0] )
					{
					alpha = - ptr_lam[ll+0] / ptr_dlam[ll+0];
					}
				if( -alpha*ptr_dlam[ll+pnb]>ptr_lam[ll+pnb] )
					{
					alpha = - ptr_lam[ll+pnb] / ptr_dlam[ll+pnb];
					}
				if( -alpha*ptr_dt[ll+0]>ptr_t[ll+0] )
					{
					alpha = - ptr_t[ll+0] / ptr_dt[ll+0];
					}
				if( -alpha*ptr_dt[ll+pnb]>ptr_t[ll+pnb] )
					{
					alpha = - ptr_t[ll+pnb] / ptr_dt[ll+pnb];
					}

				}

			ptr_db   += 2*pnb;
			ptr_t    += 2*pnb;
			ptr_dt   += 2*pnb;
			ptr_lamt += 2*pnb;
			ptr_lam  += 2*pnb;
			ptr_dlam += 2*pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			nu0 = nu[jj];
			nx0 = nx[jj];
			png = (ng0+bs-1)/bs*bs;
			cng = (ng0+ncl-1)/ncl*ncl;

#ifdef BLASFEO
			dgemv_t_lib(nx0+nu0, ng0, 1.0, pDCt[jj], cng, ptr_dux, 0.0, ptr_dt, ptr_dt);
#else
			dgemv_t_lib(nx0+nu0, ng0, pDCt[jj], cng, ptr_dux, 0, ptr_dt, ptr_dt);
#endif

			for(ll=0; ll<ng0; ll++)
				{
				ptr_dt[ll+png] = - ptr_dt[ll];
				ptr_dt[ll+0]   += - ptr_db[ll+0]   - ptr_t[ll+0];
				ptr_dt[ll+png] +=   ptr_db[ll+png] - ptr_t[ll+png];
				ptr_dlam[ll+0]   -= ptr_lamt[ll+0]   * ptr_dt[ll+0]   + ptr_lam[ll+0];
				ptr_dlam[ll+png] -= ptr_lamt[ll+png] * ptr_dt[ll+png] + ptr_lam[ll+png];
				if( -alpha*ptr_dlam[ll+0]>ptr_lam[ll+0] )
					{
					alpha = - ptr_lam[ll+0] / ptr_dlam[ll+0];
					}
				if( -alpha*ptr_dlam[ll+png]>ptr_lam[ll+png] )
					{
					alpha = - ptr_lam[ll+png] / ptr_dlam[ll+png];
					}
				if( -alpha*ptr_dt[ll+0]>ptr_t[ll+0] )
					{
					alpha = - ptr_t[ll+0] / ptr_dt[ll+0];
					}
				if( -alpha*ptr_dt[ll+png]>ptr_t[ll+png] )
					{
					alpha = - ptr_t[ll+png] / ptr_dt[ll+png];
					}

				}

			}

		}

	// store alpha
	ptr_alpha[0] = alpha;

	return;

	}



void d_update_var_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, double **ux, double **dux, double **t, double **dt, double **lam, double **dlam, double **pi, double **dpi)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nx1, nb0, pnb, ng0, png;

	int jj, ll;

	double
		*ptr_pi, *ptr_dpi, *ptr_ux, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lam, *ptr_dlam;

	double mu = 0;

	for(jj=0; jj<=N; jj++)
		{

		nx0 = nx[jj];
		nu0 = nu[jj];
		nb0 = nb[jj];
		pnb = bs*((nb0+bs-1)/bs); // cache aligned number of box constraints
		ng0 = ng[jj];
		png = bs*((ng0+bs-1)/bs); // cache aligned number of box constraints
		if(jj<N)
			nx1 = nx[jj+1];
		else
			nx1 = 0;

		ptr_pi   = pi[jj];
		ptr_dpi  = dpi[jj];
		ptr_ux   = ux[jj];
		ptr_dux  = dux[jj];
		ptr_t    = t[jj];
		ptr_dt   = dt[jj];
		ptr_lam  = lam[jj];
		ptr_dlam = dlam[jj];

		// update inputs and states
		for(ll=0; ll<nu0+nx0-3; ll+=4)
			{
			ptr_ux[ll+0] += alpha*(ptr_dux[ll+0] - ptr_ux[ll+0]);
			ptr_ux[ll+1] += alpha*(ptr_dux[ll+1] - ptr_ux[ll+1]);
			ptr_ux[ll+2] += alpha*(ptr_dux[ll+2] - ptr_ux[ll+2]);
			ptr_ux[ll+3] += alpha*(ptr_dux[ll+3] - ptr_ux[ll+3]);
			}
		for(; ll<nu0+nx0; ll++)
			ptr_ux[ll] += alpha*(ptr_dux[ll] - ptr_ux[ll]);
		// update equality constrained multipliers
		for(ll=0; ll<nx1-3; ll+=4)
			{
			ptr_pi[ll+0] += alpha*(ptr_dpi[ll+0] - ptr_pi[ll+0]);
			ptr_pi[ll+1] += alpha*(ptr_dpi[ll+1] - ptr_pi[ll+1]);
			ptr_pi[ll+2] += alpha*(ptr_dpi[ll+2] - ptr_pi[ll+2]);
			ptr_pi[ll+3] += alpha*(ptr_dpi[ll+3] - ptr_pi[ll+3]);
			}
		for(; ll<nx1; ll++)
			ptr_pi[ll] += alpha*(ptr_dpi[ll] - ptr_pi[ll]);
		// box constraints
		for(ll=0; ll<nb0; ll++)
			{
			ptr_lam[ll+0] += alpha*ptr_dlam[ll+0];
			ptr_lam[ll+pnb] += alpha*ptr_dlam[ll+pnb];
			ptr_t[ll+0] += alpha*ptr_dt[ll+0];
			ptr_t[ll+pnb] += alpha*ptr_dt[ll+pnb];
			mu += ptr_lam[ll+0] * ptr_t[ll+0] + ptr_lam[ll+pnb] * ptr_t[ll+pnb];
			}

		ptr_t    += 2*pnb;
		ptr_dt   += 2*pnb;
		ptr_lam  += 2*pnb;
		ptr_dlam += 2*pnb;

		// genreal constraints
		for(ll=0; ll<ng0; ll++)
			{
			ptr_lam[ll+0] += alpha*ptr_dlam[ll+0];
			ptr_lam[ll+png] += alpha*ptr_dlam[ll+png];
			ptr_t[ll+0] += alpha*ptr_dt[ll+0];
			ptr_t[ll+png] += alpha*ptr_dt[ll+png];
			mu += ptr_lam[ll+0] * ptr_t[ll+0] + ptr_lam[ll+png] * ptr_t[ll+png];
			}

		}

	// scale mu
	mu *= mu_scal;

	ptr_mu[0] = mu;

	return;

	}



void d_compute_mu_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, double **lam, double **dlam, double **t, double **dt)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png;

	int jj, ll;

	double
		*ptr_t, *ptr_lam, *ptr_dt, *ptr_dlam;

	double mu = 0;

	for(jj=0; jj<=N; jj++)
		{

		nb0 = nb[jj];
		pnb = (nb0+bs-1)/bs*bs;
		ng0 = ng[jj];
		png = (ng0+bs-1)/bs*bs;

		ptr_t    = t[jj];
		ptr_lam  = lam[jj];
		ptr_dt   = dt[jj];
		ptr_dlam = dlam[jj];

		// box constraints
		for(ll=0 ; ll<nb0; ll++)
			{
			mu += (ptr_lam[ll+0] + alpha*ptr_dlam[ll+0]) * (ptr_t[ll+0] + alpha*ptr_dt[ll+0]) + (ptr_lam[ll+pnb] + alpha*ptr_dlam[ll+pnb]) * (ptr_t[ll+pnb] + alpha*ptr_dt[ll+pnb]);
			}

		ptr_t    += 2*pnb;
		ptr_dt   += 2*pnb;
		ptr_lam  += 2*pnb;
		ptr_dlam += 2*pnb;

		// general constraints
		for(ll=0; ll<ng0; ll++)
			{
			mu += (ptr_lam[ll+0] + alpha*ptr_dlam[ll+0]) * (ptr_t[ll+0] + alpha*ptr_dt[ll+0]) + (ptr_lam[ll+png] + alpha*ptr_dlam[ll+png]) * (ptr_t[ll+png] + alpha*ptr_dt[ll+png]);
			}

		}

	// scale mu
	mu *= mu_scal;

	ptr_mu[0] = mu;

	return;

	}



void d_update_gradient_new_rhs_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double **db, double **lamt, double **qx)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png;

	double temp0, temp1;

	double
		*ptr_db, *ptr_Qx, *ptr_qx,
		*ptr_lamt;

	int ii, jj, bs0;

	for(jj=0; jj<=N; jj++)
		{

		ptr_lamt  = lamt[jj];
		ptr_db    = db[jj];
		ptr_qx    = qx[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<nb0-3; ii+=4)
				{

				ptr_qx[ii+0] = - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] - ptr_lamt[ii+0]*ptr_db[ii+0];

				ptr_qx[ii+1] = - ptr_lamt[ii+pnb+1]*ptr_db[ii+pnb+1] - ptr_lamt[ii+1]*ptr_db[ii+1];

				ptr_qx[ii+2] = - ptr_lamt[ii+pnb+2]*ptr_db[ii+pnb+2] - ptr_lamt[ii+2]*ptr_db[ii+2];

				ptr_qx[ii+3] = - ptr_lamt[ii+pnb+3]*ptr_db[ii+pnb+3] - ptr_lamt[ii+3]*ptr_db[ii+3];

				}
			for(; ii<nb0; ii++)
				{

				ptr_qx[ii+0] = - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] - ptr_lamt[ii+0]*ptr_db[ii+0];

				}

			ptr_lamt  += 2*pnb;
			ptr_db    += 2*pnb;
			ptr_qx    += pnb;

			} // end of if nb0>0

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			png = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

			for(ii=0; ii<ng0-3; ii+=4)
				{

				ptr_qx[ii+0] = - ptr_lamt[ii+png+0]*ptr_db[ii+png+0] - ptr_lamt[ii+0]*ptr_db[ii+0];

				ptr_qx[ii+1] = - ptr_lamt[ii+png+1]*ptr_db[ii+png+1] - ptr_lamt[ii+1]*ptr_db[ii+1];

				ptr_qx[ii+2] = - ptr_lamt[ii+png+2]*ptr_db[ii+png+2] - ptr_lamt[ii+2]*ptr_db[ii+2];

				ptr_qx[ii+3] = - ptr_lamt[ii+png+3]*ptr_db[ii+png+3] - ptr_lamt[ii+3]*ptr_db[ii+3];

				}
			for(; ii<ng0; ii++)
				{

				ptr_qx[ii+0] = - ptr_lamt[ii+png+0]*ptr_db[ii+png+0] - ptr_lamt[ii+0]*ptr_db[ii+0];

				}

			} // end of if ng0>0

		} // end of jj loop over N

	}



void d_compute_t_lam_new_rhs_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, double **t_aff, double **lam_aff, double **lamt, double **tinv, double **dux, double **pDCt, double **db)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nb0, pnb, ng0, png, cng;

	double
		*ptr_db, *ptr_dux, *ptr_t_aff, *ptr_lam_aff, *ptr_lamt, *ptr_tinv;

	int
		*ptr_idxb;

	int jj, ll;

	for(jj=0; jj<=N; jj++)
		{

		ptr_db      = db[jj];
		ptr_dux     = dux[jj];
		ptr_t_aff   = t_aff[jj];
		ptr_lam_aff = lam_aff[jj];
		ptr_lamt    = lamt[jj];
		ptr_tinv    = tinv[jj];
		ptr_idxb    = idxb[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb = (nb0+bs-1)/bs*bs;

			// box constraints
			for(ll=0; ll<nb0; ll++)
				{

				ptr_t_aff[ll+0]   =   ptr_dux[ptr_idxb[ll]] - ptr_db[ll+0];
				ptr_t_aff[ll+pnb] = - ptr_dux[ptr_idxb[ll]] + ptr_db[ll+pnb];
				ptr_lam_aff[ll+0]   = - ptr_lamt[ll+0]   * ptr_t_aff[ll+0];
				ptr_lam_aff[ll+pnb] = - ptr_lamt[ll+pnb] * ptr_t_aff[ll+pnb];
				}

			ptr_db      += 2*pnb;
			ptr_t_aff   += 2*pnb;
			ptr_lam_aff += 2*pnb;
			ptr_lamt    += 2*pnb;
			ptr_tinv    += 2*pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			nu0 = nu[jj];
			nx0 = nx[jj];
			png = (ng0+bs-1)/bs*bs;
			cng = (ng0+ncl-1)/ncl*ncl;

#ifdef BLASFEO
			dgemv_t_lib(nx0+nu0, ng0, 1.0, pDCt[jj], cng, ptr_dux, 0.0, ptr_t_aff, ptr_t_aff);
#else
			dgemv_t_lib(nx0+nu0, ng0, pDCt[jj], cng, ptr_dux, 0, ptr_t_aff, ptr_t_aff);
#endif

			for(ll=0; ll<ng0; ll++)
				{
				ptr_t_aff[ll+png] = - ptr_t_aff[ll+0];
				ptr_t_aff[ll+0]   -= ptr_db[ll+0];
				ptr_t_aff[ll+png] += ptr_db[ll+png];
				ptr_lam_aff[ll+0]   = - ptr_lamt[ll+0]   * ptr_t_aff[ll+0];
				ptr_lam_aff[ll+png] = - ptr_lamt[ll+png] * ptr_t_aff[ll+png];
				}

			}

		}

	return;

	}



// IPM with residuals

void d_update_hessian_gradient_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double **res_d, double **res_m, double **t, double **lam, double **t_inv, double **Qx, double **qx)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png;

	double temp0, temp1;

	double
		*ptr_res_d, *ptr_Qx, *ptr_qx, *ptr_t, *ptr_lam, *ptr_res_m, *ptr_t_inv;

	int ii, jj, bs0;

	for(jj=0; jj<=N; jj++)
		{

		ptr_t     = t[jj];
		ptr_lam   = lam[jj];
		ptr_t_inv = t_inv[jj];
		ptr_res_d = res_d[jj];
		ptr_res_m = res_m[jj];
		ptr_Qx    = Qx[jj];
		ptr_qx    = qx[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<nb0-3; ii+=4)
				{

				ptr_t_inv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_t_inv[ii+pnb+0] = 1.0/ptr_t[ii+pnb+0];
				ptr_Qx[ii+0] = ptr_t_inv[ii+0]*ptr_lam[ii+0] + ptr_t_inv[ii+pnb+0]*ptr_lam[ii+pnb+0];
				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+pnb+0]*(ptr_res_m[ii+pnb+0]+ptr_lam[ii+pnb+0]*ptr_res_d[ii+pnb+0]);

				ptr_t_inv[ii+1] = 1.0/ptr_t[ii+1];
				ptr_t_inv[ii+pnb+1] = 1.0/ptr_t[ii+pnb+1];
				ptr_Qx[ii+1] = ptr_t_inv[ii+1]*ptr_lam[ii+1] + ptr_t_inv[ii+pnb+1]*ptr_lam[ii+pnb+1];
				ptr_qx[ii+1] = ptr_t_inv[ii+1]*(ptr_res_m[ii+1]-ptr_lam[ii+1]*ptr_res_d[ii+1]) - ptr_t_inv[ii+pnb+1]*(ptr_res_m[ii+pnb+1]+ptr_lam[ii+pnb+1]*ptr_res_d[ii+pnb+1]);

				ptr_t_inv[ii+2] = 1.0/ptr_t[ii+2];
				ptr_t_inv[ii+pnb+2] = 1.0/ptr_t[ii+pnb+2];
				ptr_Qx[ii+2] = ptr_t_inv[ii+2]*ptr_lam[ii+2] + ptr_t_inv[ii+pnb+2]*ptr_lam[ii+pnb+2];
				ptr_qx[ii+2] = ptr_t_inv[ii+2]*(ptr_res_m[ii+2]-ptr_lam[ii+2]*ptr_res_d[ii+2]) - ptr_t_inv[ii+pnb+2]*(ptr_res_m[ii+pnb+2]+ptr_lam[ii+pnb+2]*ptr_res_d[ii+pnb+2]);

				ptr_t_inv[ii+3] = 1.0/ptr_t[ii+3];
				ptr_t_inv[ii+pnb+3] = 1.0/ptr_t[ii+pnb+3];
				ptr_Qx[ii+3] = ptr_t_inv[ii+3]*ptr_lam[ii+3] + ptr_t_inv[ii+pnb+3]*ptr_lam[ii+pnb+3];
				ptr_qx[ii+3] = ptr_t_inv[ii+3]*(ptr_res_m[ii+3]-ptr_lam[ii+3]*ptr_res_d[ii+3]) - ptr_t_inv[ii+pnb+3]*(ptr_res_m[ii+pnb+3]+ptr_lam[ii+pnb+3]*ptr_res_d[ii+pnb+3]);

				}
			for(; ii<nb0; ii++)
				{

				ptr_t_inv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_t_inv[ii+pnb+0] = 1.0/ptr_t[ii+pnb+0];
				ptr_Qx[ii+0] = ptr_t_inv[ii+0]*ptr_lam[ii+0] + ptr_t_inv[ii+pnb+0]*ptr_lam[ii+pnb+0];
				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+pnb+0]*(ptr_res_m[ii+pnb+0]+ptr_lam[ii+pnb+0]*ptr_res_d[ii+pnb+0]);

				}

			ptr_t     += 2*pnb;
			ptr_lam   += 2*pnb;
			ptr_t_inv += 2*pnb;
			ptr_res_d += 2*pnb;
			ptr_res_m += 2*pnb;
			ptr_Qx    += pnb;
			ptr_qx    += pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{


			png = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

			for(ii=0; ii<ng0-3; ii+=4)
				{

				ptr_t_inv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_t_inv[ii+png+0] = 1.0/ptr_t[ii+png+0];
				ptr_Qx[ii+0] = ptr_t_inv[ii+0]*ptr_lam[ii+0] + ptr_t_inv[ii+png+0]*ptr_lam[ii+png+0];
				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+png+0]*(ptr_res_m[ii+png+0]+ptr_lam[ii+png+0]*ptr_res_d[ii+png+0]);

				ptr_t_inv[ii+1] = 1.0/ptr_t[ii+1];
				ptr_t_inv[ii+png+1] = 1.0/ptr_t[ii+png+1];
				ptr_Qx[ii+1] = ptr_t_inv[ii+1]*ptr_lam[ii+1] + ptr_t_inv[ii+png+1]*ptr_lam[ii+png+1];
				ptr_qx[ii+1] = ptr_t_inv[ii+1]*(ptr_res_m[ii+1]-ptr_lam[ii+1]*ptr_res_d[ii+1]) - ptr_t_inv[ii+png+1]*(ptr_res_m[ii+png+1]+ptr_lam[ii+png+1]*ptr_res_d[ii+png+1]);

				ptr_t_inv[ii+2] = 1.0/ptr_t[ii+2];
				ptr_t_inv[ii+png+2] = 1.0/ptr_t[ii+png+2];
				ptr_Qx[ii+2] = ptr_t_inv[ii+2]*ptr_lam[ii+2] + ptr_t_inv[ii+png+2]*ptr_lam[ii+png+2];
				ptr_qx[ii+2] = ptr_t_inv[ii+2]*(ptr_res_m[ii+2]-ptr_lam[ii+2]*ptr_res_d[ii+2]) - ptr_t_inv[ii+png+2]*(ptr_res_m[ii+png+2]+ptr_lam[ii+png+2]*ptr_res_d[ii+png+2]);

				ptr_t_inv[ii+3] = 1.0/ptr_t[ii+3];
				ptr_t_inv[ii+png+3] = 1.0/ptr_t[ii+png+3];
				ptr_Qx[ii+3] = ptr_t_inv[ii+3]*ptr_lam[ii+3] + ptr_t_inv[ii+png+3]*ptr_lam[ii+png+3];
				ptr_qx[ii+3] = ptr_t_inv[ii+3]*(ptr_res_m[ii+3]-ptr_lam[ii+3]*ptr_res_d[ii+3]) - ptr_t_inv[ii+png+3]*(ptr_res_m[ii+png+3]+ptr_lam[ii+png+3]*ptr_res_d[ii+png+3]);

				}
			for(; ii<ng0; ii++)
				{

				ptr_t_inv[ii+0] = 1.0/ptr_t[ii+0];
				ptr_t_inv[ii+png+0] = 1.0/ptr_t[ii+png+0];
				ptr_Qx[ii+0] = ptr_t_inv[ii+0]*ptr_lam[ii+0] + ptr_t_inv[ii+png+0]*ptr_lam[ii+png+0];
				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+png+0]*(ptr_res_m[ii+png+0]+ptr_lam[ii+png+0]*ptr_res_d[ii+png+0]);

				}

			}

		}

	}



void d_compute_dt_dlam_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, double **dux, double **t, double **t_inv, double **lam, double **pDCt, double **res_d, double **res_m, double **dt, double **dlam)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nb0, pnb, ng0, png, cng;

	double
		*ptr_res_d, *ptr_res_m, *ptr_dux, *ptr_t, *ptr_t_inv, *ptr_dt, *ptr_lam, *ptr_dlam;

	int
		*ptr_idxb;

	int jj, ll;

	for(jj=0; jj<=N; jj++)
		{

		ptr_res_d = res_d[jj];
		ptr_res_m = res_m[jj];
		ptr_dux   = dux[jj];
		ptr_t     = t[jj];
		ptr_t_inv = t_inv[jj];
		ptr_dt    = dt[jj];
		ptr_lam   = lam[jj];
		ptr_dlam  = dlam[jj];
		ptr_idxb  = idxb[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb = (nb0+bs-1)/bs*bs;

			// box constraints
			for(ll=0; ll<nb0; ll++)
				{

				ptr_dt[ll+0]   =   ptr_dux[ptr_idxb[ll]] - ptr_res_d[ll+0];
				ptr_dt[ll+pnb] = - ptr_dux[ptr_idxb[ll]] + ptr_res_d[ll+pnb];

				ptr_dlam[ll+0]   = - ptr_t_inv[ll+0]   * ( ptr_lam[ll+0]*ptr_dt[ll+0]     + ptr_res_m[ll+0] );
				ptr_dlam[ll+pnb] = - ptr_t_inv[ll+pnb] * ( ptr_lam[ll+pnb]*ptr_dt[ll+pnb] + ptr_res_m[ll+pnb] );

				}

			ptr_res_d += 2*pnb;
			ptr_res_m += 2*pnb;
			ptr_t     += 2*pnb;
			ptr_t_inv += 2*pnb;
			ptr_dt    += 2*pnb;
			ptr_lam   += 2*pnb;
			ptr_dlam  += 2*pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			nu0 = nu[jj];
			nx0 = nx[jj];
			png = (ng0+bs-1)/bs*bs;
			cng = (ng0+ncl-1)/ncl*ncl;

#ifdef BLASFEO
			dgemv_t_lib(nx0+nu0, ng0, 1.0, pDCt[jj], cng, ptr_dux, 0.0, ptr_dt, ptr_dt);
#else
			dgemv_t_lib(nx0+nu0, ng0, pDCt[jj], cng, ptr_dux, 0, ptr_dt, ptr_dt);
#endif

			for(ll=0; ll<ng0; ll++)
				{

				ptr_dt[ll+png] = - ptr_dt[ll];

				ptr_dt[ll+0]   -= ptr_res_d[ll+0];
				ptr_dt[ll+png] += ptr_res_d[ll+png];

				ptr_dlam[ll+0]   = - ptr_t_inv[ll+0]   * ( ptr_lam[ll+0]*ptr_dt[ll+0]     + ptr_res_m[ll+0] );
				ptr_dlam[ll+png] = - ptr_t_inv[ll+png] * ( ptr_lam[ll+png]*ptr_dt[ll+png] + ptr_res_m[ll+png] );

				}

			}

		}

	return;

	}



void d_compute_alpha_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, double **dux, double **t, double **t_inv, double **lam, double **pDCt, double **res_d, double **res_m, double **dt, double **dlam, double *ptr_alpha)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nb0, pnb, ng0, png, cng;

	double alpha = ptr_alpha[0];

	double
		*ptr_res_d, *ptr_res_m, *ptr_dux, *ptr_t, *ptr_t_inv, *ptr_dt, *ptr_lam, *ptr_dlam;

	int
		*ptr_idxb;

	int jj, ll;

	for(jj=0; jj<=N; jj++)
		{

		ptr_res_d = res_d[jj];
		ptr_res_m = res_m[jj];
		ptr_dux   = dux[jj];
		ptr_t     = t[jj];
		ptr_t_inv = t_inv[jj];
		ptr_dt    = dt[jj];
		ptr_lam   = lam[jj];
		ptr_dlam  = dlam[jj];
		ptr_idxb  = idxb[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb = (nb0+bs-1)/bs*bs;

			// box constraints
			for(ll=0; ll<nb0; ll++)
				{

				ptr_dt[ll+0]   =   ptr_dux[ptr_idxb[ll]] - ptr_res_d[ll+0];
				ptr_dt[ll+pnb] = - ptr_dux[ptr_idxb[ll]] + ptr_res_d[ll+pnb];

				ptr_dlam[ll+0]   = - ptr_t_inv[ll+0]   * ( ptr_lam[ll+0]*ptr_dt[ll+0]     + ptr_res_m[ll+0] );
				ptr_dlam[ll+pnb] = - ptr_t_inv[ll+pnb] * ( ptr_lam[ll+pnb]*ptr_dt[ll+pnb] + ptr_res_m[ll+pnb] );

				if( -alpha*ptr_dlam[ll+0]>ptr_lam[ll+0] )
					{
					alpha = - ptr_lam[ll+0] / ptr_dlam[ll+0];
					}
				if( -alpha*ptr_dlam[ll+pnb]>ptr_lam[ll+pnb] )
					{
					alpha = - ptr_lam[ll+pnb] / ptr_dlam[ll+pnb];
					}
				if( -alpha*ptr_dt[ll+0]>ptr_t[ll+0] )
					{
					alpha = - ptr_t[ll+0] / ptr_dt[ll+0];
					}
				if( -alpha*ptr_dt[ll+pnb]>ptr_t[ll+pnb] )
					{
					alpha = - ptr_t[ll+pnb] / ptr_dt[ll+pnb];
					}

				}

			ptr_res_d += 2*pnb;
			ptr_res_m += 2*pnb;
			ptr_t     += 2*pnb;
			ptr_t_inv += 2*pnb;
			ptr_dt    += 2*pnb;
			ptr_lam   += 2*pnb;
			ptr_dlam  += 2*pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			nu0 = nu[jj];
			nx0 = nx[jj];
			png = (ng0+bs-1)/bs*bs;
			cng = (ng0+ncl-1)/ncl*ncl;

#ifdef BLASFEO
			dgemv_t_lib(nx0+nu0, ng0, 1.0, pDCt[jj], cng, ptr_dux, 0.0, ptr_dt, ptr_dt);
#else
			dgemv_t_lib(nx0+nu0, ng0, pDCt[jj], cng, ptr_dux, 0, ptr_dt, ptr_dt);
#endif

			for(ll=0; ll<ng0; ll++)
				{

				ptr_dt[ll+png] = - ptr_dt[ll];

				ptr_dt[ll+0]   -= ptr_res_d[ll+0];
				ptr_dt[ll+png] += ptr_res_d[ll+png];

				ptr_dlam[ll+0]   = - ptr_t_inv[ll+0]   * ( ptr_lam[ll+0]*ptr_dt[ll+0]     + ptr_res_m[ll+0] );
				ptr_dlam[ll+png] = - ptr_t_inv[ll+png] * ( ptr_lam[ll+png]*ptr_dt[ll+png] + ptr_res_m[ll+png] );

				if( -alpha*ptr_dlam[ll+0]>ptr_lam[ll+0] )
					{
					alpha = - ptr_lam[ll+0] / ptr_dlam[ll+0];
					}
				if( -alpha*ptr_dlam[ll+png]>ptr_lam[ll+png] )
					{
					alpha = - ptr_lam[ll+png] / ptr_dlam[ll+png];
					}
				if( -alpha*ptr_dt[ll+0]>ptr_t[ll+0] )
					{
					alpha = - ptr_t[ll+0] / ptr_dt[ll+0];
					}
				if( -alpha*ptr_dt[ll+png]>ptr_t[ll+png] )
					{
					alpha = - ptr_t[ll+png] / ptr_dt[ll+png];
					}

				}

			}

		}

	// store alpha
	ptr_alpha[0] = alpha;

	return;

	}



void d_update_var_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double alpha, double **ux, double **dux, double **pi, double **dpi, double **t, double **dt, double **lam, double **dlam)
	{

	// constants
	const int bs = D_MR;

	int nu0, nx0, nx1, nb0, pnb, ng0, png;

	int jj, ll;

	double
		*ptr_ux, *ptr_dux, *ptr_pi, *ptr_dpi, *ptr_t, *ptr_dt, *ptr_lam, *ptr_dlam;

	for(jj=0; jj<=N; jj++)
		{

		nx0 = nx[jj];
		nu0 = nu[jj];
		nb0 = nb[jj];
		pnb = bs*((nb0+bs-1)/bs); // cache aligned number of box constraints
		ng0 = ng[jj];
		png = bs*((ng0+bs-1)/bs); // cache aligned number of box constraints
		if(jj<N)
			nx1 = nx[jj+1];
		else
			nx1 = 0;

		// update inputs and states
		ptr_ux     = ux[jj];
		ptr_dux    = dux[jj];
		daxpy_lib(nu0+nx0, alpha, ptr_dux, ptr_ux);

		// update equality constrained multipliers
		ptr_pi     = pi[jj];
		ptr_dpi    = dpi[jj];
		daxpy_lib(nx1, alpha, ptr_dpi, ptr_pi);

		// box constraints
		ptr_t       = t[jj];
		ptr_dt      = dt[jj];
		ptr_lam     = lam[jj];
		ptr_dlam    = dlam[jj];
		daxpy_lib(nb0, alpha, &ptr_dlam[0], &ptr_lam[0]);
		daxpy_lib(nb0, alpha, &ptr_dlam[pnb], &ptr_lam[pnb]);
		daxpy_lib(nb0, alpha, &ptr_dt[0], &ptr_t[0]);
		daxpy_lib(nb0, alpha, &ptr_dt[pnb], &ptr_t[pnb]);

		// general constraints
		ptr_t       += 2*pnb;
		ptr_dt      += 2*pnb;
		ptr_lam     += 2*pnb;
		ptr_dlam    += 2*pnb;
		daxpy_lib(ng0, alpha, &ptr_dlam[0], &ptr_lam[0]);
		daxpy_lib(ng0, alpha, &ptr_dlam[png], &ptr_lam[png]);
		daxpy_lib(ng0, alpha, &ptr_dt[0], &ptr_t[0]);
		daxpy_lib(ng0, alpha, &ptr_dt[png], &ptr_t[png]);

		}

	return;

	}



void d_backup_update_var_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double alpha, double **ux_bkp, double **ux, double **dux, double **pi_bkp, double **pi, double **dpi, double **t_bkp, double **t, double **dt, double **lam_bkp, double **lam, double **dlam)
	{

	// constants
	const int bs = D_MR;

	int nu0, nx0, nx1, nb0, pnb, ng0, png;

	int jj, ll;

	double
		*ptr_ux_bkp, *ptr_ux, *ptr_dux, *ptr_pi_bkp, *ptr_pi, *ptr_dpi, *ptr_t_bkp, *ptr_t, *ptr_dt, *ptr_lam_bkp, *ptr_lam, *ptr_dlam;

	for(jj=0; jj<=N; jj++)
		{

		nx0 = nx[jj];
		nu0 = nu[jj];
		nb0 = nb[jj];
		pnb = bs*((nb0+bs-1)/bs); // cache aligned number of box constraints
		ng0 = ng[jj];
		png = bs*((ng0+bs-1)/bs); // cache aligned number of box constraints
		if(jj<N)
			nx1 = nx[jj+1];
		else
			nx1 = 0;

		// update inputs and states
		ptr_ux_bkp = ux_bkp[jj];
		ptr_ux     = ux[jj];
		ptr_dux    = dux[jj];
		daxpy_bkp_lib(nu0+nx0, alpha, ptr_dux, ptr_ux, ptr_ux_bkp);

		// update equality constrained multipliers
		ptr_pi_bkp = pi_bkp[jj];
		ptr_pi     = pi[jj];
		ptr_dpi    = dpi[jj];
		daxpy_bkp_lib(nx1, alpha, ptr_dpi, ptr_pi, ptr_pi_bkp);

		// box constraints
		ptr_t_bkp   = t_bkp[jj];
		ptr_t       = t[jj];
		ptr_dt      = dt[jj];
		ptr_lam_bkp = lam_bkp[jj];
		ptr_lam     = lam[jj];
		ptr_dlam    = dlam[jj];
		daxpy_bkp_lib(nb0, alpha, &ptr_dlam[0], &ptr_lam[0], &ptr_lam_bkp[0]);
		daxpy_bkp_lib(nb0, alpha, &ptr_dlam[pnb], &ptr_lam[pnb], &ptr_lam_bkp[pnb]);
		daxpy_bkp_lib(nb0, alpha, &ptr_dt[0], &ptr_t[0], &ptr_t_bkp[0]);
		daxpy_bkp_lib(nb0, alpha, &ptr_dt[pnb], &ptr_t[pnb], &ptr_t_bkp[pnb]);

		// general constraints
		ptr_t_bkp   += 2*pnb;
		ptr_t       += 2*pnb;
		ptr_dt      += 2*pnb;
		ptr_lam_bkp += 2*pnb;
		ptr_lam     += 2*pnb;
		ptr_dlam    += 2*pnb;
		daxpy_bkp_lib(ng0, alpha, &ptr_dlam[0], &ptr_lam[0], &ptr_lam_bkp[0]);
		daxpy_bkp_lib(ng0, alpha, &ptr_dlam[png], &ptr_lam[png], &ptr_lam_bkp[png]);
		daxpy_bkp_lib(ng0, alpha, &ptr_dt[0], &ptr_t[0], &ptr_t_bkp[0]);
		daxpy_bkp_lib(ng0, alpha, &ptr_dt[png], &ptr_t[png], &ptr_t_bkp[png]);

		}

	return;

	}



void d_compute_mu_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double alpha, double **lam, double **dlam, double **t, double **dt, double *ptr_mu, double mu_scal)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png;

	int jj, ll;

	double
		*ptr_t, *ptr_lam, *ptr_dt, *ptr_dlam;

	double mu = 0;

	for(jj=0; jj<=N; jj++)
		{

		nb0 = nb[jj];
		pnb = (nb0+bs-1)/bs*bs;
		ng0 = ng[jj];
		png = (ng0+bs-1)/bs*bs;

		ptr_t    = t[jj];
		ptr_lam  = lam[jj];
		ptr_dt   = dt[jj];
		ptr_dlam = dlam[jj];

		// box constraints
		for(ll=0 ; ll<nb0; ll++)
			{
			mu += (ptr_lam[ll+0] + alpha*ptr_dlam[ll+0]) * (ptr_t[ll+0] + alpha*ptr_dt[ll+0]) + (ptr_lam[ll+pnb] + alpha*ptr_dlam[ll+pnb]) * (ptr_t[ll+pnb] + alpha*ptr_dt[ll+pnb]);
			}

		ptr_t    += 2*pnb;
		ptr_dt   += 2*pnb;
		ptr_lam  += 2*pnb;
		ptr_dlam += 2*pnb;

		// general constraints
		for(ll=0; ll<ng0; ll++)
			{
			mu += (ptr_lam[ll+0] + alpha*ptr_dlam[ll+0]) * (ptr_t[ll+0] + alpha*ptr_dt[ll+0]) + (ptr_lam[ll+png] + alpha*ptr_dlam[ll+png]) * (ptr_t[ll+png] + alpha*ptr_dt[ll+png]);
			}

		}

	// scale mu
	mu *= mu_scal;

	ptr_mu[0] = mu;

	return;

	}



void d_compute_centering_correction_res_mpc_hard_tv(int N, int *nb, int *ng, double sigma_mu, double **dt, double **dlam, double **res_m)
	{

	const int bs = D_MR;

	int pnb, png;

	int ii, jj;

	double
		*ptr_res_m, *ptr_dt, *ptr_dlam;

	for(ii=0; ii<=N; ii++)
		{

		pnb = (nb[ii]+bs-1)/bs*bs;
		png = (ng[ii]+bs-1)/bs*bs;

		ptr_res_m = res_m[ii];
		ptr_dt    = dt[ii];
		ptr_dlam  = dlam[ii];

		for(jj=0; jj<nb[ii]; jj++)
			{
			ptr_res_m[jj]     += ptr_dt[jj]     * ptr_dlam[jj]     - sigma_mu;
			ptr_res_m[pnb+jj] += ptr_dt[pnb+jj] * ptr_dlam[pnb+jj] - sigma_mu;
			}
		for(jj=0; jj<ng[ii]; jj++)
			{
			ptr_res_m[2*pnb+jj]     += ptr_dt[2*pnb+jj]     * ptr_dlam[2*pnb+jj]     - sigma_mu;
			ptr_res_m[2*pnb+png+jj] += ptr_dt[2*pnb+png+jj] * ptr_dlam[2*pnb+png+jj] - sigma_mu;
			}
		}

	}



void d_update_gradient_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double **res_d, double **res_m, double **lam, double **t_inv, double **qx)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png;

	double temp0, temp1;

	double
		*ptr_res_d, *ptr_Qx, *ptr_qx, *ptr_lam, *ptr_res_m, *ptr_t_inv;

	int ii, jj, bs0;

	for(jj=0; jj<=N; jj++)
		{

		ptr_lam   = lam[jj];
		ptr_t_inv = t_inv[jj];
		ptr_res_d = res_d[jj];
		ptr_res_m = res_m[jj];
		ptr_qx    = qx[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<nb0-3; ii+=4)
				{

				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+pnb+0]*(ptr_res_m[ii+pnb+0]+ptr_lam[ii+pnb+0]*ptr_res_d[ii+pnb+0]);

				ptr_qx[ii+1] = ptr_t_inv[ii+1]*(ptr_res_m[ii+1]-ptr_lam[ii+1]*ptr_res_d[ii+1]) - ptr_t_inv[ii+pnb+1]*(ptr_res_m[ii+pnb+1]+ptr_lam[ii+pnb+1]*ptr_res_d[ii+pnb+1]);

				ptr_qx[ii+2] = ptr_t_inv[ii+2]*(ptr_res_m[ii+2]-ptr_lam[ii+2]*ptr_res_d[ii+2]) - ptr_t_inv[ii+pnb+2]*(ptr_res_m[ii+pnb+2]+ptr_lam[ii+pnb+2]*ptr_res_d[ii+pnb+2]);

				ptr_qx[ii+3] = ptr_t_inv[ii+3]*(ptr_res_m[ii+3]-ptr_lam[ii+3]*ptr_res_d[ii+3]) - ptr_t_inv[ii+pnb+3]*(ptr_res_m[ii+pnb+3]+ptr_lam[ii+pnb+3]*ptr_res_d[ii+pnb+3]);

				}
			for(; ii<nb0; ii++)
				{

				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+pnb+0]*(ptr_res_m[ii+pnb+0]+ptr_lam[ii+pnb+0]*ptr_res_d[ii+pnb+0]);

				}

			ptr_lam   += 2*pnb;
			ptr_t_inv += 2*pnb;
			ptr_res_d += 2*pnb;
			ptr_res_m += 2*pnb;
			ptr_qx    += pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			png = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

			for(ii=0; ii<ng0-3; ii+=4)
				{

				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+png+0]*(ptr_res_m[ii+png+0]+ptr_lam[ii+png+0]*ptr_res_d[ii+png+0]);

				ptr_qx[ii+1] = ptr_t_inv[ii+1]*(ptr_res_m[ii+1]-ptr_lam[ii+1]*ptr_res_d[ii+1]) - ptr_t_inv[ii+png+1]*(ptr_res_m[ii+png+1]+ptr_lam[ii+png+1]*ptr_res_d[ii+png+1]);

				ptr_qx[ii+2] = ptr_t_inv[ii+2]*(ptr_res_m[ii+2]-ptr_lam[ii+2]*ptr_res_d[ii+2]) - ptr_t_inv[ii+png+2]*(ptr_res_m[ii+png+2]+ptr_lam[ii+png+2]*ptr_res_d[ii+png+2]);

				ptr_qx[ii+3] = ptr_t_inv[ii+3]*(ptr_res_m[ii+3]-ptr_lam[ii+3]*ptr_res_d[ii+3]) - ptr_t_inv[ii+png+3]*(ptr_res_m[ii+png+3]+ptr_lam[ii+png+3]*ptr_res_d[ii+png+3]);

				}
			for(; ii<ng0; ii++)
				{

				ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+png+0]*(ptr_res_m[ii+png+0]+ptr_lam[ii+png+0]*ptr_res_d[ii+png+0]);

				}

			}

		}

	}
