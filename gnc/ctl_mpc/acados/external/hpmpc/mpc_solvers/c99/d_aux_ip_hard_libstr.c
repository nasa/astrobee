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

#ifdef BLASFEO

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include "../../include/block_size.h" // TODO remove !!!!!



// initialize variables

void d_init_var_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hspi, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsdb, struct blasfeo_dvec *hst, struct blasfeo_dvec *hslam, double mu0, int warm_start)
	{

	int jj, ll, ii;

	double *ptr_ux, *ptr_pi, *ptr_db, *ptr_t, *ptr_lam;

	int nb0, ng0, nt0;
	
	double thr0 = 0.1; // minimum vale of t (minimum distance from a constraint)


	// cold start
	if(warm_start==0)
		{
		for(jj=0; jj<=N; jj++)
			{
			ptr_ux = hsux[jj].pa;
			for(ll=0; ll<nu[jj]+nx[jj]; ll++)
				{
				ptr_ux[ll] = 0.0;
				}
			}
		}


	// check bounds & initialize multipliers
	for(jj=0; jj<=N; jj++)
		{
		nb0 = nb[jj];
		nt0 = nb[jj]+ng[jj];
		ptr_ux = hsux[jj].pa;
		ptr_db = hsdb[jj].pa;
		ptr_lam = hslam[jj].pa;
		ptr_t = hst[jj].pa;
		for(ll=0; ll<nb0; ll++)
			{
			ptr_t[ll]     = - ptr_db[ll]     + ptr_ux[hidxb[jj][ll]];
			ptr_t[nt0+ll] =   ptr_db[nt0+ll] - ptr_ux[hidxb[jj][ll]];
			if(ptr_t[ll] < thr0)
				{
				if(ptr_t[nt0+ll] < thr0)
					{
					ptr_ux[hidxb[jj][ll]] = ( - ptr_db[nt0+ll] + ptr_db[ll])*0.5;
					ptr_t[ll]     = thr0; //- hdb[jj][ll]     + hux[jj][hidxb[jj][ll]];
					ptr_t[nt0+ll] = thr0; //  hdb[jj][nt0+ll] - hux[jj][hidxb[jj][ll]];
					}
				else
					{
					ptr_t[ll] = thr0;
					ptr_ux[hidxb[jj][ll]] = ptr_db[ll] + thr0;
					}
				}
			else if(ptr_t[nt0+ll] < thr0)
				{
				ptr_t[nt0+ll] = thr0;
				ptr_ux[hidxb[jj][ll]] = ptr_db[nt0+ll] - thr0;
				}
			ptr_lam[ll]     = mu0/ptr_t[ll];
			ptr_lam[nt0+ll] = mu0/ptr_t[nt0+ll];
			}
		}


	// initialize pi
	for(jj=1; jj<=N; jj++)
		{
		ptr_pi = hspi[jj].pa;
		for(ll=0; ll<nx[jj]; ll++)
			ptr_pi[ll] = 0.0; // initialize multipliers to zero
		}


	// TODO find a better way to initialize general constraints
	for(jj=0; jj<=N; jj++)
		{
		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;
		if(ng0>0)
			{
			ptr_t   = hst[jj].pa;
			ptr_lam = hslam[jj].pa;
			ptr_db  = hsdb[jj].pa;
			blasfeo_dgemv_t(nu[jj]+nx[jj], ng0, 1.0, &hsDCt[jj], 0, 0, &hsux[jj], 0, 0.0, &hst[jj], nb0, &hst[jj], nb0);
			for(ll=nb0; ll<nb0+ng0; ll++)
				{
				ptr_t[ll+nt0] = - ptr_t[ll];
				ptr_t[ll]     -= ptr_db[ll];
				ptr_t[ll+nt0] += ptr_db[ll+nt0];
				ptr_t[ll]     = fmax( thr0, ptr_t[ll] );
				ptr_t[nt0+ll] = fmax( thr0, ptr_t[nt0+ll] );
				ptr_lam[ll]     = mu0/ptr_t[ll];
				ptr_lam[nt0+ll] = mu0/ptr_t[nt0+ll];
				}
			}
		}

	}



// IPM with no residuals

void d_update_hessian_gradient_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, struct blasfeo_dvec *hsdb, double sigma_mu, struct blasfeo_dvec *hst, struct blasfeo_dvec *hstinv, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hslamt, struct blasfeo_dvec *hsdlam, struct blasfeo_dvec *hsQx, struct blasfeo_dvec *hsqx)
	{
	
	int ii, jj;
	
	int nb0, ng0, nt0;
	
	double temp0, temp1;
	
	double 
		*ptr_db, *ptr_Qx, *ptr_qx,
		*ptr_t, *ptr_lam, *ptr_lamt, *ptr_dlam, *ptr_tinv;
	
	for(jj=0; jj<=N; jj++)
		{
		
		ptr_t     = hst[jj].pa;
		ptr_lam   = hslam[jj].pa;
		ptr_lamt  = hslamt[jj].pa;
		ptr_dlam  = hsdlam[jj].pa;
		ptr_tinv  = hstinv[jj].pa;
		ptr_db    = hsdb[jj].pa;
		ptr_Qx    = hsQx[jj].pa;
		ptr_qx    = hsqx[jj].pa;

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;

		for(ii=0; ii<nt0-3; ii+=4)
			{

			ptr_tinv[ii+0] = 1.0/ptr_t[ii+0];
			ptr_tinv[ii+nt0+0] = 1.0/ptr_t[ii+nt0+0];
			ptr_lamt[ii+0] = ptr_lam[ii+0]*ptr_tinv[ii+0];
			ptr_lamt[ii+nt0+0] = ptr_lam[ii+nt0+0]*ptr_tinv[ii+nt0+0];
			ptr_dlam[ii+0] = ptr_tinv[ii+0]*sigma_mu; // !!!!!
			ptr_dlam[ii+nt0+0] = ptr_tinv[ii+nt0+0]*sigma_mu; // !!!!!
			ptr_Qx[ii+0] = ptr_lamt[ii+0] + ptr_lamt[ii+nt0+0];
			ptr_qx[ii+0] = ptr_lam[ii+nt0+0] - ptr_lamt[ii+nt0+0]*ptr_db[ii+nt0+0] + ptr_dlam[ii+nt0+0] - ptr_lam[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0] - ptr_dlam[ii+0];

			ptr_tinv[ii+1] = 1.0/ptr_t[ii+1];
			ptr_tinv[ii+nt0+1] = 1.0/ptr_t[ii+nt0+1];
			ptr_lamt[ii+1] = ptr_lam[ii+1]*ptr_tinv[ii+1];
			ptr_lamt[ii+nt0+1] = ptr_lam[ii+nt0+1]*ptr_tinv[ii+nt0+1];
			ptr_dlam[ii+1] = ptr_tinv[ii+1]*sigma_mu; // !!!!!
			ptr_dlam[ii+nt0+1] = ptr_tinv[ii+nt0+1]*sigma_mu; // !!!!!
			ptr_Qx[ii+1] = ptr_lamt[ii+1] + ptr_lamt[ii+nt0+1];
			ptr_qx[ii+1] = ptr_lam[ii+nt0+1] - ptr_lamt[ii+nt0+1]*ptr_db[ii+nt0+1] + ptr_dlam[ii+nt0+1] - ptr_lam[ii+1] - ptr_lamt[ii+1]*ptr_db[ii+1] - ptr_dlam[ii+1];

			ptr_tinv[ii+2] = 1.0/ptr_t[ii+2];
			ptr_tinv[ii+nt0+2] = 1.0/ptr_t[ii+nt0+2];
			ptr_lamt[ii+2] = ptr_lam[ii+2]*ptr_tinv[ii+2];
			ptr_lamt[ii+nt0+2] = ptr_lam[ii+nt0+2]*ptr_tinv[ii+nt0+2];
			ptr_dlam[ii+2] = ptr_tinv[ii+2]*sigma_mu; // !!!!!
			ptr_dlam[ii+nt0+2] = ptr_tinv[ii+nt0+2]*sigma_mu; // !!!!!
			ptr_Qx[ii+2] = ptr_lamt[ii+2] + ptr_lamt[ii+nt0+2];
			ptr_qx[ii+2] = ptr_lam[ii+nt0+2] - ptr_lamt[ii+nt0+2]*ptr_db[ii+nt0+2] + ptr_dlam[ii+nt0+2] - ptr_lam[ii+2] - ptr_lamt[ii+2]*ptr_db[ii+2] - ptr_dlam[ii+2];

			ptr_tinv[ii+3] = 1.0/ptr_t[ii+3];
			ptr_tinv[ii+nt0+3] = 1.0/ptr_t[ii+nt0+3];
			ptr_lamt[ii+3] = ptr_lam[ii+3]*ptr_tinv[ii+3];
			ptr_lamt[ii+nt0+3] = ptr_lam[ii+nt0+3]*ptr_tinv[ii+nt0+3];
			ptr_dlam[ii+3] = ptr_tinv[ii+3]*sigma_mu; // !!!!!
			ptr_dlam[ii+nt0+3] = ptr_tinv[ii+nt0+3]*sigma_mu; // !!!!!
			ptr_Qx[ii+3] = ptr_lamt[ii+3] + ptr_lamt[ii+nt0+3];
			ptr_qx[ii+3] = ptr_lam[ii+nt0+3] - ptr_lamt[ii+nt0+3]*ptr_db[ii+nt0+3] + ptr_dlam[ii+nt0+3] - ptr_lam[ii+3] - ptr_lamt[ii+3]*ptr_db[ii+3] - ptr_dlam[ii+3];

			}
		for(; ii<nt0; ii++)
			{

			ptr_tinv[ii+0] = 1.0/ptr_t[ii+0];
			ptr_tinv[ii+nt0+0] = 1.0/ptr_t[ii+nt0+0];
			ptr_lamt[ii+0] = ptr_lam[ii+0]*ptr_tinv[ii+0];
			ptr_lamt[ii+nt0+0] = ptr_lam[ii+nt0+0]*ptr_tinv[ii+nt0+0];
			ptr_dlam[ii+0] = ptr_tinv[ii+0]*sigma_mu; // !!!!!
			ptr_dlam[ii+nt0+0] = ptr_tinv[ii+nt0+0]*sigma_mu; // !!!!!
			ptr_Qx[ii] = ptr_lamt[ii+0] + ptr_lamt[ii+nt0+0];
			ptr_qx[ii] = ptr_lam[ii+nt0+0] - ptr_lamt[ii+nt0+0]*ptr_db[ii+nt0+0] + ptr_dlam[ii+nt0+0] - ptr_lam[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0] - ptr_dlam[ii+0];

			}

		}
	
	return;

	}



void d_update_gradient_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double sigma_mu, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hsdlam, struct blasfeo_dvec *hstinv, struct blasfeo_dvec *hsqx)
	{

	int ii, jj;

	int nb0, ng0, nt0;

	double
		*ptr_dlam, *ptr_t_inv, *ptr_dt, *ptr_pl2, *ptr_qx;

	for(jj=0; jj<=N; jj++)
		{

		ptr_dlam  = hsdlam[jj].pa;
		ptr_dt    = hsdt[jj].pa;
		ptr_t_inv = hstinv[jj].pa;
		ptr_qx    = hsqx[jj].pa;

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;

		for(ii=0; ii<nt0-3; ii+=4)
			{
			ptr_dlam[0*nt0+ii+0] = ptr_t_inv[0*nt0+ii+0]*(sigma_mu - ptr_dlam[0*nt0+ii+0]*ptr_dt[0*nt0+ii+0]);
			ptr_dlam[1*nt0+ii+0] = ptr_t_inv[1*nt0+ii+0]*(sigma_mu - ptr_dlam[1*nt0+ii+0]*ptr_dt[1*nt0+ii+0]);
			ptr_qx[ii+0] += ptr_dlam[1*nt0+ii+0] - ptr_dlam[0*nt0+ii+0];

			ptr_dlam[0*nt0+ii+1] = ptr_t_inv[0*nt0+ii+1]*(sigma_mu - ptr_dlam[0*nt0+ii+1]*ptr_dt[0*nt0+ii+1]);
			ptr_dlam[1*nt0+ii+1] = ptr_t_inv[1*nt0+ii+1]*(sigma_mu - ptr_dlam[1*nt0+ii+1]*ptr_dt[1*nt0+ii+1]);
			ptr_qx[ii+1] += ptr_dlam[1*nt0+ii+1] - ptr_dlam[0*nt0+ii+1];

			ptr_dlam[0*nt0+ii+2] = ptr_t_inv[0*nt0+ii+2]*(sigma_mu - ptr_dlam[0*nt0+ii+2]*ptr_dt[0*nt0+ii+2]);
			ptr_dlam[1*nt0+ii+2] = ptr_t_inv[1*nt0+ii+2]*(sigma_mu - ptr_dlam[1*nt0+ii+2]*ptr_dt[1*nt0+ii+2]);
			ptr_qx[ii+2] += ptr_dlam[1*nt0+ii+2] - ptr_dlam[0*nt0+ii+2];

			ptr_dlam[0*nt0+ii+3] = ptr_t_inv[0*nt0+ii+3]*(sigma_mu - ptr_dlam[0*nt0+ii+3]*ptr_dt[0*nt0+ii+3]);
			ptr_dlam[1*nt0+ii+3] = ptr_t_inv[1*nt0+ii+3]*(sigma_mu - ptr_dlam[1*nt0+ii+3]*ptr_dt[1*nt0+ii+3]);
			ptr_qx[ii+3] += ptr_dlam[1*nt0+ii+3] - ptr_dlam[0*nt0+ii+3];
			}
		for(; ii<nt0; ii++)
			{
			ptr_dlam[0*nt0+ii+0] = ptr_t_inv[0*nt0+ii+0]*(sigma_mu - ptr_dlam[0*nt0+ii+0]*ptr_dt[0*nt0+ii+0]);
			ptr_dlam[1*nt0+ii+0] = ptr_t_inv[1*nt0+ii+0]*(sigma_mu - ptr_dlam[1*nt0+ii+0]*ptr_dt[1*nt0+ii+0]);
			ptr_qx[ii+0] += ptr_dlam[1*nt0+ii+0] - ptr_dlam[0*nt0+ii+0];
			}

		}

	return;

	}



void d_compute_alpha_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, double *ptr_alpha, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam, struct blasfeo_dvec *hslamt, struct blasfeo_dvec *hsdux, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsdb)
	{
	
	int nu0, nx0, nb0, ng0, nt0;

	double alpha = ptr_alpha[0];
	
	double
		*ptr_db, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lamt, *ptr_lam, *ptr_dlam;
	
	int
		*ptr_idxb;
	
	int jj, ll;

	for(jj=0; jj<=N; jj++)
		{

		ptr_db   = hsdb[jj].pa;
		ptr_dux  = hsdux[jj].pa;
		ptr_t    = hst[jj].pa;
		ptr_dt   = hsdt[jj].pa;
		ptr_lamt = hslamt[jj].pa;
		ptr_lam  = hslam[jj].pa;
		ptr_dlam = hsdlam[jj].pa;
		ptr_idxb = idxb[jj];

		nu0 = nu[jj];
		nx0 = nx[jj];
		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;

		// box constraints // TODO dvecex_libstr
		for(ll=0; ll<nb0; ll++)
			ptr_dt[ll] = ptr_dux[ptr_idxb[ll]];

		// general constraints
		blasfeo_dgemv_t(nx0+nu0, ng0, 1.0, &hsDCt[jj], 0, 0, &hsdux[jj], 0, 0.0, &hsdt[jj], nb0, &hsdt[jj], nb0);

		// all constraints
		for(ll=0; ll<nt0; ll++)
			{
			ptr_dt[ll+nt0] = - ptr_dt[ll];
			ptr_dt[ll+0]   += - ptr_db[ll+0]   - ptr_t[ll+0];
			ptr_dt[ll+nt0] +=   ptr_db[ll+nt0] - ptr_t[ll+nt0];
			ptr_dlam[ll+0]   -= ptr_lamt[ll+0]   * ptr_dt[ll+0]   + ptr_lam[ll+0];
			ptr_dlam[ll+nt0] -= ptr_lamt[ll+nt0] * ptr_dt[ll+nt0] + ptr_lam[ll+nt0];
			if( -alpha*ptr_dlam[ll+0]>ptr_lam[ll+0] )
				{
				alpha = - ptr_lam[ll+0] / ptr_dlam[ll+0];
				}
			if( -alpha*ptr_dlam[ll+nt0]>ptr_lam[ll+nt0] )
				{
				alpha = - ptr_lam[ll+nt0] / ptr_dlam[ll+nt0];
				}
			if( -alpha*ptr_dt[ll+0]>ptr_t[ll+0] )
				{
				alpha = - ptr_t[ll+0] / ptr_dt[ll+0];
				}
			if( -alpha*ptr_dt[ll+nt0]>ptr_t[ll+nt0] )
				{
				alpha = - ptr_t[ll+nt0] / ptr_dt[ll+nt0];
				}

			}

		}		

	// store alpha
	ptr_alpha[0] = alpha;

	return;
	
	}



void d_update_var_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hsdpi, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam)
	{

	int ii;

	ptr_mu[0] = 0.0;
	
	// backup and update equality constrains multipliers
	for(ii=1; ii<=N; ii++)
		{
		blasfeo_daxpy(nx[ii], -1.0, &hspi[ii], 0, &hsdpi[ii], 0, &hsdpi[ii], 0);
		blasfeo_daxpy(nx[ii], alpha, &hsdpi[ii], 0, &hspi[ii], 0, &hspi[ii], 0);
		}

	// backup and update inputs and states
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_daxpy(nu[ii]+nx[ii], -1.0, &hsux[ii], 0, &hsdux[ii], 0, &hsdux[ii], 0);
		blasfeo_daxpy(nu[ii]+nx[ii], alpha, &hsdux[ii], 0, &hsux[ii], 0, &hsux[ii], 0);
		}

	// backup and update inequality constraints multipliers and slack variables
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdlam[ii], 0, &hslam[ii], 0, &hslam[ii], 0);
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdt[ii], 0, &hst[ii], 0, &hst[ii], 0);
		ptr_mu[0] += blasfeo_ddot(2*nb[ii]+2*ng[ii], &hst[ii], 0, &hslam[ii], 0);
		}
	
	ptr_mu[0] *= mu_scal;

	return;
	
	}



void d_backup_update_var_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, struct blasfeo_dvec *hsux_bkp, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hspi_bkp, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hsdpi, struct blasfeo_dvec *hst_bkp, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hslam_bkp, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam)
	{

	int ii;

	ptr_mu[0] = 0.0;
	
	// backup and update equality constrains multipliers
	for(ii=1; ii<=N; ii++)
		{
		blasfeo_dveccp(nx[ii], &hspi[ii], 0, &hspi_bkp[ii], 0);
		blasfeo_daxpy(nx[ii], -1.0, &hspi[ii], 0, &hsdpi[ii], 0, &hsdpi[ii], 0);
		blasfeo_daxpy(nx[ii], alpha, &hsdpi[ii], 0, &hspi[ii], 0, &hspi[ii], 0);
		}

	// backup and update inputs and states
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_dveccp(nu[ii]+nx[ii], &hsux[ii], 0, &hsux_bkp[ii], 0);
		blasfeo_daxpy(nu[ii]+nx[ii], -1.0, &hsux[ii], 0, &hsdux[ii], 0, &hsdux[ii], 0);
		blasfeo_daxpy(nu[ii]+nx[ii], alpha, &hsdux[ii], 0, &hsux[ii], 0, &hsux[ii], 0);
		}

	// backup and update inequality constraints multipliers and slack variables
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_dveccp(2*nb[ii]+2*ng[ii], &hslam[ii], 0, &hslam_bkp[ii], 0);
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdlam[ii], 0, &hslam[ii], 0, &hslam[ii], 0);
		blasfeo_dveccp(2*nb[ii]+2*ng[ii], &hst[ii], 0, &hst_bkp[ii], 0);
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdt[ii], 0, &hst[ii], 0, &hst[ii], 0);
		ptr_mu[0] += blasfeo_ddot(2*nb[ii]+2*ng[ii], &hst[ii], 0, &hslam[ii], 0);
		}
	
	ptr_mu[0] *= mu_scal;

	return;
	
	}



void d_compute_mu_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt)
	{
	
	int nb0, ng0;

	int jj, ll;
	
	double
		*ptr_t, *ptr_lam, *ptr_dt, *ptr_dlam;
		
	double mu = 0;
	
	for(jj=0; jj<=N; jj++)
		{
		
		nb0 = nb[jj];
		ng0 = ng[jj];
		
		ptr_t    = hst[jj].pa;
		ptr_lam  = hslam[jj].pa;
		ptr_dt   = hsdt[jj].pa;
		ptr_dlam = hsdlam[jj].pa;

		ll = 0;
		for( ; ll<2*nb0+2*ng0-3; ll+=4)
			{
			mu += (ptr_lam[ll+0] + alpha*ptr_dlam[ll+0]) * (ptr_t[ll+0] + alpha*ptr_dt[ll+0]);
			mu += (ptr_lam[ll+1] + alpha*ptr_dlam[ll+1]) * (ptr_t[ll+1] + alpha*ptr_dt[ll+1]);
			mu += (ptr_lam[ll+2] + alpha*ptr_dlam[ll+2]) * (ptr_t[ll+2] + alpha*ptr_dt[ll+2]);
			mu += (ptr_lam[ll+3] + alpha*ptr_dlam[ll+3]) * (ptr_t[ll+3] + alpha*ptr_dt[ll+3]);
			}
		for( ; ll<2*nb0+2*ng0; ll++)
			{
			mu += (ptr_lam[ll+0] + alpha*ptr_dlam[ll+0]) * (ptr_t[ll+0] + alpha*ptr_dt[ll+0]);
			}

		}

	// scale mu
	mu *= mu_scal;
		
	ptr_mu[0] = mu;

	return;

	}




// IPM with residuals

void d_update_hessian_gradient_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, struct blasfeo_dvec *hsres_d, struct blasfeo_dvec *hsres_m, struct blasfeo_dvec *hst, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hst_inv, struct blasfeo_dvec *hsQx, struct blasfeo_dvec *hsqx)
	{
	
	int nb0, ng0, nt0;
	
	double temp0, temp1;
	
	double 
		*ptr_res_d, *ptr_Qx, *ptr_qx, *ptr_t, *ptr_lam, *ptr_res_m, *ptr_t_inv;
	
	int ii, jj;
	
	for(jj=0; jj<=N; jj++)
		{
		
		ptr_t     = hst[jj].pa;
		ptr_lam   = hslam[jj].pa;
		ptr_t_inv = hst_inv[jj].pa;
		ptr_res_d = hsres_d[jj].pa;
		ptr_res_m = hsres_m[jj].pa;
		ptr_Qx    = hsQx[jj].pa;
		ptr_qx    = hsqx[jj].pa;

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;

		for(ii=0; ii<nt0-3; ii+=4)
			{

			ptr_t_inv[ii+0] = 1.0/ptr_t[ii+0];
			ptr_t_inv[ii+nt0+0] = 1.0/ptr_t[ii+nt0+0];
			ptr_Qx[ii+0] = ptr_t_inv[ii+0]*ptr_lam[ii+0] + ptr_t_inv[ii+nt0+0]*ptr_lam[ii+nt0+0];
			ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+nt0+0]*(ptr_res_m[ii+nt0+0]+ptr_lam[ii+nt0+0]*ptr_res_d[ii+nt0+0]);

			ptr_t_inv[ii+1] = 1.0/ptr_t[ii+1];
			ptr_t_inv[ii+nt0+1] = 1.0/ptr_t[ii+nt0+1];
			ptr_Qx[ii+1] = ptr_t_inv[ii+1]*ptr_lam[ii+1] + ptr_t_inv[ii+nt0+1]*ptr_lam[ii+nt0+1];
			ptr_qx[ii+1] = ptr_t_inv[ii+1]*(ptr_res_m[ii+1]-ptr_lam[ii+1]*ptr_res_d[ii+1]) - ptr_t_inv[ii+nt0+1]*(ptr_res_m[ii+nt0+1]+ptr_lam[ii+nt0+1]*ptr_res_d[ii+nt0+1]);

			ptr_t_inv[ii+2] = 1.0/ptr_t[ii+2];
			ptr_t_inv[ii+nt0+2] = 1.0/ptr_t[ii+nt0+2];
			ptr_Qx[ii+2] = ptr_t_inv[ii+2]*ptr_lam[ii+2] + ptr_t_inv[ii+nt0+2]*ptr_lam[ii+nt0+2];
			ptr_qx[ii+2] = ptr_t_inv[ii+2]*(ptr_res_m[ii+2]-ptr_lam[ii+2]*ptr_res_d[ii+2]) - ptr_t_inv[ii+nt0+2]*(ptr_res_m[ii+nt0+2]+ptr_lam[ii+nt0+2]*ptr_res_d[ii+nt0+2]);

			ptr_t_inv[ii+3] = 1.0/ptr_t[ii+3];
			ptr_t_inv[ii+nt0+3] = 1.0/ptr_t[ii+nt0+3];
			ptr_Qx[ii+3] = ptr_t_inv[ii+3]*ptr_lam[ii+3] + ptr_t_inv[ii+nt0+3]*ptr_lam[ii+nt0+3];
			ptr_qx[ii+3] = ptr_t_inv[ii+3]*(ptr_res_m[ii+3]-ptr_lam[ii+3]*ptr_res_d[ii+3]) - ptr_t_inv[ii+nt0+3]*(ptr_res_m[ii+nt0+3]+ptr_lam[ii+nt0+3]*ptr_res_d[ii+nt0+3]);

			}
		for(; ii<nt0; ii++)
			{

			ptr_t_inv[ii+0] = 1.0/ptr_t[ii+0];
			ptr_t_inv[ii+nt0+0] = 1.0/ptr_t[ii+nt0+0];
			ptr_Qx[ii+0] = ptr_t_inv[ii+0]*ptr_lam[ii+0] + ptr_t_inv[ii+nt0+0]*ptr_lam[ii+nt0+0];
			ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+nt0+0]*(ptr_res_m[ii+nt0+0]+ptr_lam[ii+nt0+0]*ptr_res_d[ii+nt0+0]);

			}

		}

	return;

	}



void d_compute_alpha_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hst, struct blasfeo_dvec *hst_inv, struct blasfeo_dvec *hslam, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsres_d, struct blasfeo_dvec *hsres_m, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hsdlam, double *ptr_alpha)
	{
	
	int nu0, nx0, nb0, ng0, nt0;

	double alpha = ptr_alpha[0];
	
	double
		*ptr_res_d, *ptr_res_m, *ptr_dux, *ptr_t, *ptr_t_inv, *ptr_dt, *ptr_lam, *ptr_dlam;
	
	int
		*ptr_idxb;
	
	int jj, ll;

	for(jj=0; jj<=N; jj++)
		{

		ptr_res_d = hsres_d[jj].pa;
		ptr_res_m = hsres_m[jj].pa;
		ptr_dux   = hsdux[jj].pa;
		ptr_t     = hst[jj].pa;
		ptr_t_inv = hst_inv[jj].pa;
		ptr_dt    = hsdt[jj].pa;
		ptr_lam   = hslam[jj].pa;
		ptr_dlam  = hsdlam[jj].pa;
		ptr_idxb  = idxb[jj];

		nu0 = nu[jj];
		nx0 = nx[jj];
		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;

		// box constraints // TODO blasfeo_dvecex_sp
		for(ll=0; ll<nb0; ll++)
			ptr_dt[ll] = ptr_dux[ptr_idxb[ll]];

		// general constraints
		blasfeo_dgemv_t(nx0+nu0, ng0, 1.0, &hsDCt[jj], 0, 0, &hsdux[jj], 0, 0.0, &hsdt[jj], nb0, &hsdt[jj], nb0);

		for(ll=0; ll<nt0; ll++)
			{

			ptr_dt[ll+nt0] = - ptr_dt[ll];

			ptr_dt[ll+0]   -= ptr_res_d[ll+0];
			ptr_dt[ll+nt0] += ptr_res_d[ll+nt0];

			ptr_dlam[ll+0]   = - ptr_t_inv[ll+0]   * ( ptr_lam[ll+0]*ptr_dt[ll+0]     + ptr_res_m[ll+0] );
			ptr_dlam[ll+nt0] = - ptr_t_inv[ll+nt0] * ( ptr_lam[ll+nt0]*ptr_dt[ll+nt0] + ptr_res_m[ll+nt0] );

			if( -alpha*ptr_dlam[ll+0]>ptr_lam[ll+0] )
				{
				alpha = - ptr_lam[ll+0] / ptr_dlam[ll+0];
				}
			if( -alpha*ptr_dlam[ll+nt0]>ptr_lam[ll+nt0] )
				{
				alpha = - ptr_lam[ll+nt0] / ptr_dlam[ll+nt0];
				}
			if( -alpha*ptr_dt[ll+0]>ptr_t[ll+0] )
				{
				alpha = - ptr_t[ll+0] / ptr_dt[ll+0];
				}
			if( -alpha*ptr_dt[ll+nt0]>ptr_t[ll+nt0] )
				{
				alpha = - ptr_t[ll+nt0] / ptr_dt[ll+nt0];
				}

			}

		}		

	// store alpha
	ptr_alpha[0] = alpha;

	return;
	
	}



void d_update_var_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double alpha, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hsdpi, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam)
	{

	int ii;
	
	// update equality constrains multipliers
	for(ii=1; ii<=N; ii++)
		blasfeo_daxpy(nx[ii], alpha, &hsdpi[ii], 0, &hspi[ii], 0, &hspi[ii], 0);

	// update inputs and states
	for(ii=0; ii<=N; ii++)
		blasfeo_daxpy(nu[ii]+nx[ii], alpha, &hsdux[ii], 0, &hsux[ii], 0, &hsux[ii], 0);

	// update inequality constraints multipliers
	for(ii=0; ii<=N; ii++)
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdlam[ii], 0, &hslam[ii], 0, &hslam[ii], 0);

	// update slack variables
	for(ii=0; ii<=N; ii++)
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdt[ii], 0, &hst[ii], 0, &hst[ii], 0);

	return;
	
	}



void d_backup_update_var_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double alpha, struct blasfeo_dvec *hsux_bkp, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hspi_bkp, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hsdpi, struct blasfeo_dvec *hst_bkp, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hslam_bkp, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam)
	{

	int ii;
	
	// backup and update equality constrains multipliers
	for(ii=1; ii<=N; ii++)
		{
		blasfeo_dveccp(nx[ii], &hspi[ii], 0, &hspi_bkp[ii], 0);
		blasfeo_daxpy(nx[ii], alpha, &hsdpi[ii], 0, &hspi[ii], 0, &hspi[ii], 0);
		}

	// backup and update inputs and states
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_dveccp(nu[ii]+nx[ii], &hsux[ii], 0, &hsux_bkp[ii], 0);
		blasfeo_daxpy(nu[ii]+nx[ii], alpha, &hsdux[ii], 0, &hsux[ii], 0, &hsux[ii], 0);
		}

	// backup and update inequality constraints multipliers
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_dveccp(2*nb[ii]+2*ng[ii], &hslam[ii], 0, &hslam_bkp[ii], 0);
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdlam[ii], 0, &hslam[ii], 0, &hslam[ii], 0);
		}

	// backup and update slack variables
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_dveccp(2*nb[ii]+2*ng[ii], &hst[ii], 0, &hst_bkp[ii], 0);
		blasfeo_daxpy(2*nb[ii]+2*ng[ii], alpha, &hsdt[ii], 0, &hst[ii], 0, &hst[ii], 0);
		}

	return;
	
	}



void d_compute_centering_correction_res_mpc_hard_libstr(int N, int *nb, int *ng, double sigma_mu, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hsdlam, struct blasfeo_dvec *hsres_m)
	{

	int ii, jj, jjmax;

	double
		*ptr_res_m, *ptr_dt, *ptr_dlam;
	
	for(ii=0; ii<=N; ii++)
		{

		ptr_res_m = hsres_m[ii].pa;
		ptr_dt    = hsdt[ii].pa;
		ptr_dlam  = hsdlam[ii].pa;

		jjmax = 2*nb[ii]+2*ng[ii];

		jj = 0;
		for(; jj<jjmax-3; jj+=4)
			{
			ptr_res_m[jj+0] += ptr_dt[jj+0] * ptr_dlam[jj+0] - sigma_mu;
			ptr_res_m[jj+1] += ptr_dt[jj+1] * ptr_dlam[jj+1] - sigma_mu;
			ptr_res_m[jj+2] += ptr_dt[jj+2] * ptr_dlam[jj+2] - sigma_mu;
			ptr_res_m[jj+3] += ptr_dt[jj+3] * ptr_dlam[jj+3] - sigma_mu;
			}
		for(; jj<jjmax; jj++)
			{
			ptr_res_m[jj+0] += ptr_dt[jj+0] * ptr_dlam[jj+0] - sigma_mu;
			}
		}

	}



void d_update_gradient_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, struct blasfeo_dvec *hsres_d, struct blasfeo_dvec *hsres_m, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hst_inv, struct blasfeo_dvec *hsqx)
	{
	
	int nt0;
	
	double temp0, temp1;
	
	double 
		*ptr_res_d, *ptr_Qx, *ptr_qx, *ptr_lam, *ptr_res_m, *ptr_t_inv;
	
	int ii, jj;
	
	for(jj=0; jj<=N; jj++)
		{
		
		ptr_lam   = hslam[jj].pa;
		ptr_t_inv = hst_inv[jj].pa;
		ptr_res_d = hsres_d[jj].pa;
		ptr_res_m = hsres_m[jj].pa;
		ptr_qx    = hsqx[jj].pa;

		nt0 = nb[jj] + ng[jj];

		for(ii=0; ii<nt0-3; ii+=4)
			{

			ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+nt0+0]*(ptr_res_m[ii+nt0+0]+ptr_lam[ii+nt0+0]*ptr_res_d[ii+nt0+0]);

			ptr_qx[ii+1] = ptr_t_inv[ii+1]*(ptr_res_m[ii+1]-ptr_lam[ii+1]*ptr_res_d[ii+1]) - ptr_t_inv[ii+nt0+1]*(ptr_res_m[ii+nt0+1]+ptr_lam[ii+nt0+1]*ptr_res_d[ii+nt0+1]);

			ptr_qx[ii+2] = ptr_t_inv[ii+2]*(ptr_res_m[ii+2]-ptr_lam[ii+2]*ptr_res_d[ii+2]) - ptr_t_inv[ii+nt0+2]*(ptr_res_m[ii+nt0+2]+ptr_lam[ii+nt0+2]*ptr_res_d[ii+nt0+2]);

			ptr_qx[ii+3] = ptr_t_inv[ii+3]*(ptr_res_m[ii+3]-ptr_lam[ii+3]*ptr_res_d[ii+3]) - ptr_t_inv[ii+nt0+3]*(ptr_res_m[ii+nt0+3]+ptr_lam[ii+nt0+3]*ptr_res_d[ii+nt0+3]);

			}
		for(; ii<nt0; ii++)
			{

			ptr_qx[ii+0] = ptr_t_inv[ii+0]*(ptr_res_m[ii+0]-ptr_lam[ii+0]*ptr_res_d[ii+0]) - ptr_t_inv[ii+nt0+0]*(ptr_res_m[ii+nt0+0]+ptr_lam[ii+nt0+0]*ptr_res_d[ii+nt0+0]);

			}

		}

	return;

	}



#endif

