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
#include "../../include/blas_d.h"
#include "../../include/block_size.h"

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#endif



void d_init_var_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, int *ns, double **ux, double **pi, double **pDCt, double **db, double **t, double **lam, double mu0, int warm_start)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int jj, ll, ii;

	int nb0, pnb, ng0, png, cng, ns0, pns;
	
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
			t[jj][ll]     = - db[jj][ll]     + ux[jj][idxb[jj][ll]];
			t[jj][pnb+ll] =   db[jj][pnb+ll] - ux[jj][idxb[jj][ll]];
			if(t[jj][ll] < thr0)
				{
				if(t[jj][pnb+ll] < thr0)
					{
					ux[jj][idxb[jj][ll]] = ( - db[jj][pnb+ll] + db[jj][ll])*0.5;
					t[jj][ll]     = thr0; //- db[jj][ll]     + ux[jj][idxb[jj][ll]];
					t[jj][pnb+ll] = thr0; //- db[jj][pnb+ll] - ux[jj][idxb[jj][ll]];
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
			lam[jj][ll]     = mu0/t[jj][ll];
			lam[jj][pnb+ll] = mu0/t[jj][pnb+ll];
			}
		}


	// inizialize t_theta and lam_theta (cold start only for the moment)
	for(jj=0; jj<=N; jj++)
		{
		nb0 = nb[jj];
		pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints
		ng0 = ng[jj];
		png  = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints
		ns0 = ns[jj];
		pns  = (ns0+bs-1)/bs*bs; // simd aligned number of box soft constraints
		for(ll=0; ll<ns[jj]; ll++)
			{
			t[jj][2*pnb+2*png+0*pns+ll] = 1.0;
			t[jj][2*pnb+2*png+1*pns+ll] = 1.0;
			t[jj][2*pnb+2*png+2*pns+ll] = 1.0;
			t[jj][2*pnb+2*png+3*pns+ll] = 1.0;
			lam[jj][2*pnb+2*png+0*pns+ll] = mu0; // /t[jj][pnb+ll]; // TODO restore division if needed
			lam[jj][2*pnb+2*png+1*pns+ll] = mu0; // /t[jj][pnb+ll]; // TODO restore division if needed
			lam[jj][2*pnb+2*png+2*pns+ll] = mu0; // /t[jj][pnb+ll]; // TODO restore division if needed
			lam[jj][2*pnb+2*png+3*pns+ll] = mu0; // /t[jj][pnb+ll]; // TODO restore division if needed
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



void d_update_hessian_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double **db, double sigma_mu, double **t, double **tinv, double **lam, double **lamt, double **dlam, double **Qx, double **qx, double **Z, double **z, double **Zl, double **zl)
	{
	
	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png, ns0, pns, pnbs;
	
	double temp0, temp1;
	
	double 
		*ptr_db, *ptr_Qx, *ptr_qx,
		*ptr_t, *ptr_lam, *ptr_lamt, *ptr_dlam, *ptr_tinv,
		*ptr_Z, *ptr_z, *ptr_Zl, *ptr_zl;
	
	double rQx0, rQx1, rqx0, rqx1;

	int ii, jj, bs0;
	
	for(jj=0; jj<=N; jj++)
		{
		
		ptr_t     = t[jj];
		ptr_lam   = lam[jj];
		ptr_lamt  = lamt[jj];
		ptr_dlam  = dlam[jj];
		ptr_tinv  = tinv[jj];
		ptr_db    = db[jj];
		ptr_Qx    = Qx[jj];
		ptr_qx    = qx[jj];

		nb0 = nb[jj];
		ng0 = ng[jj];
		ns0 = ns[jj];
		pnb = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints
		png = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints
		pns = (ns0+bs-1)/bs*bs; // simd aligned number of box constraints
		pnbs = (nb0+ns0+bs-1)/bs*bs; // simd aligned number of box and soft constraints

		// box constraints
		if(nb0>0)
			{

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
				ptr_Qx[ii+0] = ptr_lamt[ii+0] + ptr_lamt[ii+pnb+0];
				ptr_qx[ii+0] = ptr_lam[ii+pnb+0] - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] + ptr_dlam[ii+pnb+0] - ptr_lam[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0] - ptr_dlam[ii+0];

				}

			ptr_t     += 2*pnb;
			ptr_lam   += 2*pnb;
			ptr_lamt  += 2*pnb;
			ptr_dlam  += 2*pnb;
			ptr_tinv  += 2*pnb;
			ptr_db    += 2*pnb;

			}

		// general constraints
		if(ng0>0)
			{

			ptr_Qx    = Qx[jj]+pnbs;
			ptr_qx    = qx[jj]+pnbs;

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

			ptr_t     += 2*png;
			ptr_lam   += 2*png;
			ptr_lamt  += 2*png;
			ptr_dlam  += 2*png;
			ptr_tinv  += 2*png;
			ptr_db    += 2*png;

			}

		// box soft constraints
		if(ns0>0)
			{

			ptr_Qx    = Qx[jj]+nb0;
			ptr_qx    = qx[jj]+nb0;

			ptr_Z     = Z[jj];
			ptr_z     = z[jj];
			ptr_Zl    = Zl[jj];
			ptr_zl    = zl[jj];

			for(ii=0; ii<ns0-3; ii+=4)
				{

				ptr_tinv[ii+0*pns+0] = 1.0/ptr_t[ii+0*pns+0];
				ptr_tinv[ii+1*pns+0] = 1.0/ptr_t[ii+1*pns+0];
				ptr_tinv[ii+2*pns+0] = 1.0/ptr_t[ii+2*pns+0];
				ptr_tinv[ii+3*pns+0] = 1.0/ptr_t[ii+3*pns+0];
				ptr_lamt[ii+0*pns+0] = ptr_lam[ii+0*pns+0]*ptr_tinv[ii+0*pns+0];
				ptr_lamt[ii+1*pns+0] = ptr_lam[ii+1*pns+0]*ptr_tinv[ii+1*pns+0];
				ptr_lamt[ii+2*pns+0] = ptr_lam[ii+2*pns+0]*ptr_tinv[ii+2*pns+0];
				ptr_lamt[ii+3*pns+0] = ptr_lam[ii+3*pns+0]*ptr_tinv[ii+3*pns+0];
				ptr_dlam[ii+0*pns+0] = ptr_tinv[ii+0*pns+0]*sigma_mu;
				ptr_dlam[ii+1*pns+0] = ptr_tinv[ii+1*pns+0]*sigma_mu;
				ptr_dlam[ii+2*pns+0] = ptr_tinv[ii+2*pns+0]*sigma_mu;
				ptr_dlam[ii+3*pns+0] = ptr_tinv[ii+3*pns+0]*sigma_mu;
				rQx0 = ptr_lamt[ii+0*pns+0];
				rQx1 = ptr_lamt[ii+1*pns+0];
				rqx0 = ptr_lam[ii+0*pns+0] + ptr_dlam[ii+0*pns+0] + ptr_lamt[ii+0*pns+0]*ptr_db[ii+0*pns+0];
				rqx1 = ptr_lam[ii+1*pns+0] + ptr_dlam[ii+1*pns+0] - ptr_lamt[ii+1*pns+0]*ptr_db[ii+1*pns+0];
				ptr_Zl[ii+0*pns+0] = 1.0 / (ptr_Z[ii+0*pns+0] + rQx0 + ptr_lamt[ii+2*pns+0]);
				ptr_Zl[ii+1*pns+0] = 1.0 / (ptr_Z[ii+1*pns+0] + rQx1 + ptr_lamt[ii+3*pns+0]);
				ptr_zl[ii+0*pns+0] = - ptr_z[ii+0*pns+0] + rqx0 + ptr_lam[ii+2*pns+0] + ptr_dlam[ii+2*pns+0];
				ptr_zl[ii+1*pns+0] = - ptr_z[ii+1*pns+0] + rqx1 + ptr_lam[ii+3*pns+0] + ptr_dlam[ii+3*pns+0];
				rqx0 = rqx0 - rQx0*ptr_zl[ii+0*pns+0]*ptr_Zl[ii+0*pns+0]; // update this before Qx !!!!!!!!!!!
				rqx1 = rqx1 - rQx1*ptr_zl[ii+1*pns+0]*ptr_Zl[ii+1*pns+0]; // update this before Qx !!!!!!!!!!!
				rQx0 = rQx0 - rQx0*rQx0*ptr_Zl[ii+0*pns+0];
				rQx1 = rQx1 - rQx1*rQx1*ptr_Zl[ii+1*pns+0];
				ptr_Qx[ii+0] = rQx1 + rQx0;
				ptr_qx[ii+0] = rqx1 - rqx0;

				ptr_tinv[ii+0*pns+1] = 1.0/ptr_t[ii+0*pns+1];
				ptr_tinv[ii+1*pns+1] = 1.0/ptr_t[ii+1*pns+1];
				ptr_tinv[ii+2*pns+1] = 1.0/ptr_t[ii+2*pns+1];
				ptr_tinv[ii+3*pns+1] = 1.0/ptr_t[ii+3*pns+1];
				ptr_lamt[ii+0*pns+1] = ptr_lam[ii+0*pns+1]*ptr_tinv[ii+0*pns+1];
				ptr_lamt[ii+1*pns+1] = ptr_lam[ii+1*pns+1]*ptr_tinv[ii+1*pns+1];
				ptr_lamt[ii+2*pns+1] = ptr_lam[ii+2*pns+1]*ptr_tinv[ii+2*pns+1];
				ptr_lamt[ii+3*pns+1] = ptr_lam[ii+3*pns+1]*ptr_tinv[ii+3*pns+1];
				ptr_dlam[ii+0*pns+1] = ptr_tinv[ii+0*pns+1]*sigma_mu;
				ptr_dlam[ii+1*pns+1] = ptr_tinv[ii+1*pns+1]*sigma_mu;
				ptr_dlam[ii+2*pns+1] = ptr_tinv[ii+2*pns+1]*sigma_mu;
				ptr_dlam[ii+3*pns+1] = ptr_tinv[ii+3*pns+1]*sigma_mu;
				rQx0 = ptr_lamt[ii+0*pns+1];
				rQx1 = ptr_lamt[ii+1*pns+1];
				rqx0 = ptr_lam[ii+0*pns+1] + ptr_dlam[ii+0*pns+1] + ptr_lamt[ii+0*pns+1]*ptr_db[ii+0*pns+1];
				rqx1 = ptr_lam[ii+1*pns+1] + ptr_dlam[ii+1*pns+1] - ptr_lamt[ii+1*pns+1]*ptr_db[ii+1*pns+1];
				ptr_Zl[ii+0*pns+1] = 1.0 / (ptr_Z[ii+0*pns+1] + rQx0 + ptr_lamt[ii+2*pns+1]);
				ptr_Zl[ii+1*pns+1] = 1.0 / (ptr_Z[ii+1*pns+1] + rQx1 + ptr_lamt[ii+3*pns+1]);
				ptr_zl[ii+0*pns+1] = - ptr_z[ii+0*pns+1] + rqx0 + ptr_lam[ii+2*pns+1] + ptr_dlam[ii+2*pns+1];
				ptr_zl[ii+1*pns+1] = - ptr_z[ii+1*pns+1] + rqx1 + ptr_lam[ii+3*pns+1] + ptr_dlam[ii+3*pns+1];
				rqx0 = rqx0 - rQx0*ptr_zl[ii+0*pns+1]*ptr_Zl[ii+0*pns+1]; // update this before Qx !!!!!!!!!!!
				rqx1 = rqx1 - rQx1*ptr_zl[ii+1*pns+1]*ptr_Zl[ii+1*pns+1]; // update this before Qx !!!!!!!!!!!
				rQx0 = rQx0 - rQx0*rQx0*ptr_Zl[ii+0*pns+1];
				rQx1 = rQx1 - rQx1*rQx1*ptr_Zl[ii+1*pns+1];
				ptr_Qx[ii+1] = rQx1 + rQx0;
				ptr_qx[ii+1] = rqx1 - rqx0;

				ptr_tinv[ii+0*pns+2] = 1.0/ptr_t[ii+0*pns+2];
				ptr_tinv[ii+1*pns+2] = 1.0/ptr_t[ii+1*pns+2];
				ptr_tinv[ii+2*pns+2] = 1.0/ptr_t[ii+2*pns+2];
				ptr_tinv[ii+3*pns+2] = 1.0/ptr_t[ii+3*pns+2];
				ptr_lamt[ii+0*pns+2] = ptr_lam[ii+0*pns+2]*ptr_tinv[ii+0*pns+2];
				ptr_lamt[ii+1*pns+2] = ptr_lam[ii+1*pns+2]*ptr_tinv[ii+1*pns+2];
				ptr_lamt[ii+2*pns+2] = ptr_lam[ii+2*pns+2]*ptr_tinv[ii+2*pns+2];
				ptr_lamt[ii+3*pns+2] = ptr_lam[ii+3*pns+2]*ptr_tinv[ii+3*pns+2];
				ptr_dlam[ii+0*pns+2] = ptr_tinv[ii+0*pns+2]*sigma_mu;
				ptr_dlam[ii+1*pns+2] = ptr_tinv[ii+1*pns+2]*sigma_mu;
				ptr_dlam[ii+2*pns+2] = ptr_tinv[ii+2*pns+2]*sigma_mu;
				ptr_dlam[ii+3*pns+2] = ptr_tinv[ii+3*pns+2]*sigma_mu;
				rQx0 = ptr_lamt[ii+0*pns+2];
				rQx1 = ptr_lamt[ii+1*pns+2];
				rqx0 = ptr_lam[ii+0*pns+2] + ptr_dlam[ii+0*pns+2] + ptr_lamt[ii+0*pns+2]*ptr_db[ii+0*pns+2];
				rqx1 = ptr_lam[ii+1*pns+2] + ptr_dlam[ii+1*pns+2] - ptr_lamt[ii+1*pns+2]*ptr_db[ii+1*pns+2];
				ptr_Zl[ii+0*pns+2] = 1.0 / (ptr_Z[ii+0*pns+2] + rQx0 + ptr_lamt[ii+2*pns+2]);
				ptr_Zl[ii+1*pns+2] = 1.0 / (ptr_Z[ii+1*pns+2] + rQx1 + ptr_lamt[ii+3*pns+2]);
				ptr_zl[ii+0*pns+2] = - ptr_z[ii+0*pns+2] + rqx0 + ptr_lam[ii+2*pns+2] + ptr_dlam[ii+2*pns+2];
				ptr_zl[ii+1*pns+2] = - ptr_z[ii+1*pns+2] + rqx1 + ptr_lam[ii+3*pns+2] + ptr_dlam[ii+3*pns+2];
				rqx0 = rqx0 - rQx0*ptr_zl[ii+0*pns+2]*ptr_Zl[ii+0*pns+2]; // update this before Qx !!!!!!!!!!!
				rqx1 = rqx1 - rQx1*ptr_zl[ii+1*pns+2]*ptr_Zl[ii+1*pns+2]; // update this before Qx !!!!!!!!!!!
				rQx0 = rQx0 - rQx0*rQx0*ptr_Zl[ii+0*pns+2];
				rQx1 = rQx1 - rQx1*rQx1*ptr_Zl[ii+1*pns+2];
				ptr_Qx[ii+2] = rQx1 + rQx0;
				ptr_qx[ii+2] = rqx1 - rqx0;

				ptr_tinv[ii+0*pns+3] = 1.0/ptr_t[ii+0*pns+3];
				ptr_tinv[ii+1*pns+3] = 1.0/ptr_t[ii+1*pns+3];
				ptr_tinv[ii+2*pns+3] = 1.0/ptr_t[ii+2*pns+3];
				ptr_tinv[ii+3*pns+3] = 1.0/ptr_t[ii+3*pns+3];
				ptr_lamt[ii+0*pns+3] = ptr_lam[ii+0*pns+3]*ptr_tinv[ii+0*pns+3];
				ptr_lamt[ii+1*pns+3] = ptr_lam[ii+1*pns+3]*ptr_tinv[ii+1*pns+3];
				ptr_lamt[ii+2*pns+3] = ptr_lam[ii+2*pns+3]*ptr_tinv[ii+2*pns+3];
				ptr_lamt[ii+3*pns+3] = ptr_lam[ii+3*pns+3]*ptr_tinv[ii+3*pns+3];
				ptr_dlam[ii+0*pns+3] = ptr_tinv[ii+0*pns+3]*sigma_mu;
				ptr_dlam[ii+1*pns+3] = ptr_tinv[ii+1*pns+3]*sigma_mu;
				ptr_dlam[ii+2*pns+3] = ptr_tinv[ii+2*pns+3]*sigma_mu;
				ptr_dlam[ii+3*pns+3] = ptr_tinv[ii+3*pns+3]*sigma_mu;
				rQx0 = ptr_lamt[ii+0*pns+3];
				rQx1 = ptr_lamt[ii+1*pns+3];
				rqx0 = ptr_lam[ii+0*pns+3] + ptr_dlam[ii+0*pns+3] + ptr_lamt[ii+0*pns+3]*ptr_db[ii+0*pns+3];
				rqx1 = ptr_lam[ii+1*pns+3] + ptr_dlam[ii+1*pns+3] - ptr_lamt[ii+1*pns+3]*ptr_db[ii+1*pns+3];
				ptr_Zl[ii+0*pns+3] = 1.0 / (ptr_Z[ii+0*pns+3] + rQx0 + ptr_lamt[ii+2*pns+3]);
				ptr_Zl[ii+1*pns+3] = 1.0 / (ptr_Z[ii+1*pns+3] + rQx1 + ptr_lamt[ii+3*pns+3]);
				ptr_zl[ii+0*pns+3] = - ptr_z[ii+0*pns+3] + rqx0 + ptr_lam[ii+2*pns+3] + ptr_dlam[ii+2*pns+3];
				ptr_zl[ii+1*pns+3] = - ptr_z[ii+1*pns+3] + rqx1 + ptr_lam[ii+3*pns+3] + ptr_dlam[ii+3*pns+3];
				rqx0 = rqx0 - rQx0*ptr_zl[ii+0*pns+3]*ptr_Zl[ii+0*pns+3]; // update this before Qx !!!!!!!!!!!
				rqx1 = rqx1 - rQx1*ptr_zl[ii+1*pns+3]*ptr_Zl[ii+1*pns+3]; // update this before Qx !!!!!!!!!!!
				rQx0 = rQx0 - rQx0*rQx0*ptr_Zl[ii+0*pns+3];
				rQx1 = rQx1 - rQx1*rQx1*ptr_Zl[ii+1*pns+3];
				ptr_Qx[ii+3] = rQx1 + rQx0;
				ptr_qx[ii+3] = rqx1 - rqx0;

				}
			for(; ii<ns0; ii++)
				{

				ptr_tinv[ii+0*pns+0] = 1.0/ptr_t[ii+0*pns+0];
				ptr_tinv[ii+1*pns+0] = 1.0/ptr_t[ii+1*pns+0];
				ptr_tinv[ii+2*pns+0] = 1.0/ptr_t[ii+2*pns+0];
				ptr_tinv[ii+3*pns+0] = 1.0/ptr_t[ii+3*pns+0];
				ptr_lamt[ii+0*pns+0] = ptr_lam[ii+0*pns+0]*ptr_tinv[ii+0*pns+0];
				ptr_lamt[ii+1*pns+0] = ptr_lam[ii+1*pns+0]*ptr_tinv[ii+1*pns+0];
				ptr_lamt[ii+2*pns+0] = ptr_lam[ii+2*pns+0]*ptr_tinv[ii+2*pns+0];
				ptr_lamt[ii+3*pns+0] = ptr_lam[ii+3*pns+0]*ptr_tinv[ii+3*pns+0];
				ptr_dlam[ii+0*pns+0] = ptr_tinv[ii+0*pns+0]*sigma_mu;
				ptr_dlam[ii+1*pns+0] = ptr_tinv[ii+1*pns+0]*sigma_mu;
				ptr_dlam[ii+2*pns+0] = ptr_tinv[ii+2*pns+0]*sigma_mu;
				ptr_dlam[ii+3*pns+0] = ptr_tinv[ii+3*pns+0]*sigma_mu;
				rQx0 = ptr_lamt[ii+0*pns+0];
				rQx1 = ptr_lamt[ii+1*pns+0];
				rqx0 = ptr_lam[ii+0*pns+0] + ptr_dlam[ii+0*pns+0] + ptr_lamt[ii+0*pns+0]*ptr_db[ii+0*pns+0];
				rqx1 = ptr_lam[ii+1*pns+0] + ptr_dlam[ii+1*pns+0] - ptr_lamt[ii+1*pns+0]*ptr_db[ii+1*pns+0];
				ptr_Zl[ii+0*pns+0] = 1.0 / (ptr_Z[ii+0*pns+0] + rQx0 + ptr_lamt[ii+2*pns+0]);
				ptr_Zl[ii+1*pns+0] = 1.0 / (ptr_Z[ii+1*pns+0] + rQx1 + ptr_lamt[ii+3*pns+0]);
				ptr_zl[ii+0*pns+0] = - ptr_z[ii+0*pns+0] + rqx0 + ptr_lam[ii+2*pns+0] + ptr_dlam[ii+2*pns+0];
				ptr_zl[ii+1*pns+0] = - ptr_z[ii+1*pns+0] + rqx1 + ptr_lam[ii+3*pns+0] + ptr_dlam[ii+3*pns+0];
				rqx0 = rqx0 - rQx0*ptr_zl[ii+0*pns+0]*ptr_Zl[ii+0*pns+0]; // update this before Qx !!!!!!!!!!!
				rqx1 = rqx1 - rQx1*ptr_zl[ii+1*pns+0]*ptr_Zl[ii+1*pns+0]; // update this before Qx !!!!!!!!!!!
				rQx0 = rQx0 - rQx0*rQx0*ptr_Zl[ii+0*pns+0];
				rQx1 = rQx1 - rQx1*rQx1*ptr_Zl[ii+1*pns+0];
				ptr_Qx[ii+0] = rQx1 + rQx0;
				ptr_qx[ii+0] = rqx1 - rqx0;

				}

			}

		}

	}



void d_update_gradient_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double sigma_mu, double **dt, double **dlam, double **t_inv, double **lamt, double **qx, double **Zl, double **zl)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii, jj;

	int nb0, pnb, ng0, png, ns0, pns, pnbs;

	double
		*ptr_dlam, *ptr_t_inv, *ptr_dt, *ptr_lamt, *ptr_pl2, *ptr_qx, *ptr_Zl, *ptr_zl;

	double rQx0, rQx1, rqx0, rqx1;

	for(jj=0; jj<=N; jj++)
		{

		ptr_dlam  = dlam[jj];
		ptr_dt    = dt[jj];
		ptr_lamt  = lamt[jj];
		ptr_t_inv = t_inv[jj];
		ptr_qx    = qx[jj];

		nb0 = nb[jj];
		ng0 = ng[jj];
		ns0 = ns[jj];
		pnb = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints
		png = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints
		pns = (ns0+bs-1)/bs*bs; // simd aligned number of box constraints
		pnbs = (nb0+ns0+bs-1)/bs*bs; // simd aligned number of box and soft constraints

		// box constraints
		if(nb0>0)
			{


			for(ii=0; ii<nb0; ii++)
				{
				ptr_dlam[0*pnb+ii] = ptr_t_inv[0*pnb+ii]*(sigma_mu - ptr_dlam[0*pnb+ii]*ptr_dt[0*pnb+ii]);
				ptr_dlam[1*pnb+ii] = ptr_t_inv[1*pnb+ii]*(sigma_mu - ptr_dlam[1*pnb+ii]*ptr_dt[1*pnb+ii]);
				ptr_qx[ii] += ptr_dlam[1*pnb+ii] - ptr_dlam[0*pnb+ii];
				}

			ptr_dlam  += 2*pnb;
			ptr_dt    += 2*pnb;
			ptr_lamt  += 2*pnb;
			ptr_t_inv += 2*pnb;
			ptr_qx    = qx[jj]+pnbs;

			}

		// general constraints
		if(ng0>0)
			{

			for(ii=2*pnb; ii<2*pnb+ng0; ii++)
				{
				ptr_dlam[0*png+ii] = ptr_t_inv[0*png+ii]*(sigma_mu - ptr_dlam[0*png+ii]*ptr_dt[0*png+ii]);
				ptr_dlam[1*png+ii] = ptr_t_inv[1*png+ii]*(sigma_mu - ptr_dlam[1*png+ii]*ptr_dt[1*png+ii]);
				ptr_qx[ii] += ptr_dlam[1*png+ii] - ptr_dlam[0*png+ii];
				}

			ptr_dlam  += 2*png;
			ptr_dt    += 2*png;
			ptr_lamt  += 2*png;
			ptr_t_inv += 2*png;
			ptr_qx    = qx[jj]+nb0;

			}

		// box soft constraitns
		if(ns0>0)
			{

			ptr_Zl    = Zl[jj];
			ptr_zl    = zl[jj];

			for(ii=0; ii<ns0; ii++)
				{
				ptr_dlam[0*pns+ii] = ptr_t_inv[0*pns+ii]*(sigma_mu - ptr_dlam[0*pns+ii]*ptr_dt[0*pns+ii]);
				ptr_dlam[1*pns+ii] = ptr_t_inv[1*pns+ii]*(sigma_mu - ptr_dlam[1*pns+ii]*ptr_dt[1*pns+ii]);
				ptr_dlam[2*pns+ii] = ptr_t_inv[2*pns+ii]*(sigma_mu - ptr_dlam[2*pns+ii]*ptr_dt[2*pns+ii]);
				ptr_dlam[3*pns+ii] = ptr_t_inv[3*pns+ii]*(sigma_mu - ptr_dlam[3*pns+ii]*ptr_dt[3*pns+ii]);
				rQx0 = ptr_lamt[0*pns+ii];
				rQx1 = ptr_lamt[1*pns+ii];
				rqx0 = ptr_dlam[0*pns+ii];
				rqx1 = ptr_dlam[1*pns+ii];
				ptr_zl[0*pns+ii] += rqx0 + ptr_dlam[2*pns+ii];
				ptr_zl[1*pns+ii] += rqx1 + ptr_dlam[3*pns+ii];
				rqx0 = rqx0 - rQx0*(rqx0 + ptr_dlam[2*pns+ii])*ptr_Zl[0*pns+ii];
				rqx1 = rqx1 - rQx1*(rqx1 + ptr_dlam[3*pns+ii])*ptr_Zl[1*pns+ii];
				ptr_qx[nb0+ii] += rqx1 - rqx0;
				}

			}

		}

	}


void d_compute_alpha_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, int *ns, double *ptr_alpha, double **t, double **dt, double **lam, double **dlam, double **lamt, double **dux, double **pDCt, double **db, double **Zl, double **zl)
	{
	
	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nb0, pnb, ng0, png, cng, ns0, pns;

	double alpha = ptr_alpha[0];
	
	double
		*ptr_db, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lamt, *ptr_lam, *ptr_dlam, *ptr_zl, *ptr_Zl;
	
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

			ptr_db   += 2*png;
			ptr_t    += 2*png;
			ptr_dt   += 2*png;
			ptr_lamt += 2*png;
			ptr_lam  += 2*png;
			ptr_dlam += 2*png;

			}

		// box soft constraints
		ns0 = ns[jj];
		if(ns0>0)
			{

			ptr_Zl   = Zl[jj];
			ptr_zl   = zl[jj];

			pns = (ns0+bs-1)/bs*bs;

			// box constraints
			for(ll=0; ll<ns0; ll++)
				{
				ptr_dt[2*pns+ll] = ( ptr_zl[0*pns+ll] - ptr_lamt[0*pns+ll]*ptr_dux[ptr_idxb[nb0+ll]] ) * ptr_Zl[0*pns+ll];
				ptr_dt[3*pns+ll] = ( ptr_zl[1*pns+ll] + ptr_lamt[1*pns+ll]*ptr_dux[ptr_idxb[nb0+ll]] ) * ptr_Zl[1*pns+ll];
				ptr_dt[0*pns+ll] = ptr_dt[2*pns+ll] + ptr_dux[ptr_idxb[nb0+ll]] - ptr_db[0*pns+ll] - ptr_t[0*pns+ll];
				ptr_dt[1*pns+ll] = ptr_dt[3*pns+ll] - ptr_dux[ptr_idxb[nb0+ll]] + ptr_db[1*pns+ll] - ptr_t[1*pns+ll];
				ptr_dt[2*pns+ll] -= ptr_t[2*pns+ll];
				ptr_dt[3*pns+ll] -= ptr_t[3*pns+ll];
				ptr_dlam[0*pns+ll] -= ptr_lamt[0*pns+ll] * ptr_dt[0*pns+ll] + ptr_lam[0*pns+ll];
				ptr_dlam[1*pns+ll] -= ptr_lamt[1*pns+ll] * ptr_dt[1*pns+ll] + ptr_lam[1*pns+ll];
				ptr_dlam[2*pns+ll] -= ptr_lamt[2*pns+ll] * ptr_dt[2*pns+ll] + ptr_lam[2*pns+ll];
				ptr_dlam[3*pns+ll] -= ptr_lamt[3*pns+ll] * ptr_dt[3*pns+ll] + ptr_lam[3*pns+ll];
				if( -alpha*ptr_dlam[0*pns+ll]>ptr_lam[0*pns+ll] )
					{
					alpha = - ptr_lam[0*pns+ll] / ptr_dlam[0*pns+ll];
					}
				if( -alpha*ptr_dlam[1*pns+ll]>ptr_lam[1*pns+ll] )
					{
					alpha = - ptr_lam[1*pns+ll] / ptr_dlam[1*pns+ll];
					}
				if( -alpha*ptr_dlam[2*pns+ll]>ptr_lam[2*pns+ll] )
					{
					alpha = - ptr_lam[2*pns+ll] / ptr_dlam[2*pns+ll];
					}
				if( -alpha*ptr_dlam[3*pns+ll]>ptr_lam[3*pns+ll] )
					{
					alpha = - ptr_lam[3*pns+ll] / ptr_dlam[3*pns+ll];
					}
				if( -alpha*ptr_dt[0*pns+ll]>ptr_t[0*pns+ll] )
					{
					alpha = - ptr_t[0*pns+ll] / ptr_dt[0*pns+ll];
					}
				if( -alpha*ptr_dt[1*pns+ll]>ptr_t[1*pns+ll] )
					{
					alpha = - ptr_t[1*pns+ll] / ptr_dt[1*pns+ll];
					}
				if( -alpha*ptr_dt[2*pns+ll]>ptr_t[2*pns+ll] )
					{
					alpha = - ptr_t[2*pns+ll] / ptr_dt[2*pns+ll];
					}
				if( -alpha*ptr_dt[3*pns+ll]>ptr_t[3*pns+ll] )
					{
					alpha = - ptr_t[3*pns+ll] / ptr_dt[3*pns+ll];
					}

				}

			}

		}		
	
	// store alpha
	ptr_alpha[0] = alpha;

	return;
	
	}


void d_update_var_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double *ptr_mu, double mu_scal, double alpha, double **ux, double **dux, double **t, double **dt, double **lam, double **dlam, double **pi, double **dpi)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nx1, nb0, pnb, ng0, png, ns0, pns;

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

		ptr_t    += 2*png;
		ptr_dt   += 2*png;
		ptr_lam  += 2*png;
		ptr_dlam += 2*png;

		// box soft constraints
		ns0 = ns[jj];
		pns  = bs*((ns0+bs-1)/bs); // cache aligned number of box soft constraints

		for(ll=0; ll<ns0; ll++)
			{
			ptr_lam[0*pns+ll] += alpha*ptr_dlam[0*pns+ll];
			ptr_lam[1*pns+ll] += alpha*ptr_dlam[1*pns+ll];
			ptr_lam[2*pns+ll] += alpha*ptr_dlam[2*pns+ll];
			ptr_lam[3*pns+ll] += alpha*ptr_dlam[3*pns+ll];
			ptr_t[0*pns+ll] += alpha*ptr_dt[0*pns+ll];
			ptr_t[1*pns+ll] += alpha*ptr_dt[1*pns+ll];
			ptr_t[2*pns+ll] += alpha*ptr_dt[2*pns+ll];
			ptr_t[3*pns+ll] += alpha*ptr_dt[3*pns+ll];
			mu += ptr_lam[0*pns+ll] * ptr_t[0*pns+ll] + ptr_lam[1*pns+ll] * ptr_t[1*pns+ll] + ptr_lam[2*pns+ll] * ptr_t[2*pns+ll] + ptr_lam[3*pns+ll] * ptr_t[3*pns+ll];
			}

		}

	// scale mu
	mu *= mu_scal;

	ptr_mu[0] = mu;

	return;
	
	}


void d_compute_mu_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double *ptr_mu, double mu_scal, double alpha, double **lam, double **dlam, double **t, double **dt)
	{
	
	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png, ns0, pns;

	int jj, ll;
	
	double
		*ptr_t, *ptr_lam, *ptr_dt, *ptr_dlam;
		
	double mu = 0;
	
	for(jj=0; jj<=N; jj++)
		{
		
		nb0 = nb[jj];
		pnb = (nb0+bs-1)/bs*bs;
		
		ptr_t    = t[jj];
		ptr_dt   = dt[jj];
		ptr_lam  = lam[jj];
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
		ng0 = ng[jj];
		png = (ng0+bs-1)/bs*bs;
		for(ll=0; ll<ng0; ll++)
			{
			mu += (ptr_lam[ll+0] + alpha*ptr_dlam[ll+0]) * (ptr_t[ll+0] + alpha*ptr_dt[ll+0]) + (ptr_lam[ll+png] + alpha*ptr_dlam[ll+png]) * (ptr_t[ll+png] + alpha*ptr_dt[ll+png]);
			}

		ptr_t    += 2*png;
		ptr_dt   += 2*png;
		ptr_lam  += 2*png;
		ptr_dlam += 2*png;

		// box soft constraints
		ns0 = ns[jj];
		pns  = bs*((ns0+bs-1)/bs); // cache aligned number of box soft constraints

		for(ll=0; ll<ns0; ll++)
			{
			mu += (ptr_lam[0*pns+ll] + alpha*ptr_dlam[0*pns+ll]) * (ptr_t[0*pns+ll] + alpha*ptr_dt[0*pns+ll]) + (ptr_lam[1*pns+ll] + alpha*ptr_dlam[1*pns+ll]) * (ptr_t[1*pns+ll] + alpha*ptr_dt[1*pns+ll]) + (ptr_lam[2*pns+ll] + alpha*ptr_dlam[2*pns+ll]) * (ptr_t[2*pns+ll] + alpha*ptr_dt[2*pns+ll]) + (ptr_lam[3*pns+ll] + alpha*ptr_dlam[3*pns+ll]) * (ptr_t[3*pns+ll] + alpha*ptr_dt[3*pns+ll]);
			}

		}

	// scale mu
	mu *= mu_scal;
		
	ptr_mu[0] = mu;

	return;

	}




