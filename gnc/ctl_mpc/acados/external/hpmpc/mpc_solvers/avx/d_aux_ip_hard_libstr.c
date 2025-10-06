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

#include <math.h> // TODO remove if not needed

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX

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
	
	int ii, jj, bs0;

	__m256d
		v_ones, v_sigma_mu, v_mask, v_left,
		v_tmp, v_lam, v_lamt, v_dlam, v_db,
		v_tmp0, v_tmp1,
		v_lam0, v_lam1,
		v_lamt0, v_lamt1,
		v_dlam0, v_dlam1,
		v_Qx0, v_Qx1,
		v_qx0, v_qx1,
		v_bd0, v_bd2,
		v_db0, v_db2;
	
	__m256i
		i_mask;

	double 
		*ptr_db, *ptr_Qx, *ptr_qx,
		*ptr_t, *ptr_lam, *ptr_lamt, *ptr_dlam, *ptr_tinv;
	
	v_ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	v_sigma_mu = _mm256_set_pd( sigma_mu, sigma_mu, sigma_mu, sigma_mu );


	double ii_left;

	int nb0, ng0, nt0;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

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

		ii = 0;
		for(; ii<nt0-3; ii+=4)
			{

			v_tmp0  = _mm256_loadu_pd( &ptr_t[0*nt0+ii] );
			v_tmp1  = _mm256_loadu_pd( &ptr_t[1*nt0+ii] );
			v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
			v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[0*nt0+ii] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[1*nt0+ii] );
			v_qx0   = _mm256_loadu_pd( &ptr_db[0*nt0+ii] );
			v_qx1   = _mm256_loadu_pd( &ptr_db[1*nt0+ii] );
			_mm256_storeu_pd( &ptr_tinv[0*nt0+ii], v_tmp0 );
			_mm256_storeu_pd( &ptr_tinv[1*nt0+ii], v_tmp1 );
			v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
			v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
			v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
			v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
			_mm256_storeu_pd( &ptr_lamt[0*nt0+ii], v_lamt0 );
			_mm256_storeu_pd( &ptr_lamt[1*nt0+ii], v_lamt1 );
			v_qx0   = _mm256_mul_pd( v_qx0, v_lamt0 );
			v_qx1   = _mm256_mul_pd( v_qx1, v_lamt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			_mm256_storeu_pd( &ptr_dlam[0*nt0+ii], v_dlam0 );
			_mm256_storeu_pd( &ptr_dlam[1*nt0+ii], v_dlam1 );
			v_qx0   = _mm256_add_pd( v_qx0, v_lam0 );
			v_qx1   = _mm256_sub_pd( v_lam1, v_qx1 );
			v_Qx0   = _mm256_add_pd( v_lamt0, v_lamt1 );
			v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
			_mm256_storeu_pd( &ptr_Qx[ii], v_Qx0 );
			_mm256_storeu_pd( &ptr_qx[ii], v_qx0 );

			}
		if(ii<nt0)
			{

			ii_left = nt0-ii;
			v_left= _mm256_broadcast_sd( &ii_left );
			v_mask= _mm256_loadu_pd( d_mask );
			i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_tmp0  = _mm256_loadu_pd( &ptr_t[0*nt0+ii] );
			v_tmp1  = _mm256_loadu_pd( &ptr_t[1*nt0+ii] );
			v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
			v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
			_mm256_maskstore_pd( &ptr_tinv[0*nt0+ii], i_mask, v_tmp0 );
			_mm256_maskstore_pd( &ptr_tinv[1*nt0+ii], i_mask, v_tmp1 );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[0*nt0+ii] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[1*nt0+ii] );
			v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
			v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
			_mm256_maskstore_pd( &ptr_lamt[0*nt0+ii], i_mask, v_lamt0 );
			_mm256_maskstore_pd( &ptr_lamt[1*nt0+ii], i_mask, v_lamt1 );
			v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
			v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
			_mm256_maskstore_pd( &ptr_dlam[0*nt0+ii], i_mask, v_dlam0 );
			_mm256_maskstore_pd( &ptr_dlam[1*nt0+ii], i_mask, v_dlam1 );
			v_qx0   = _mm256_loadu_pd( &ptr_db[0*nt0+ii] );
			v_qx1   = _mm256_loadu_pd( &ptr_db[1*nt0+ii] );
			v_qx0   = _mm256_mul_pd( v_qx0, v_lamt0 );
			v_qx1   = _mm256_mul_pd( v_qx1, v_lamt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_qx0   = _mm256_add_pd( v_qx0, v_lam0 );
			v_qx1   = _mm256_sub_pd( v_lam1, v_qx1 );
			v_Qx0   = _mm256_add_pd( v_lamt0, v_lamt1 );
			v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
			_mm256_maskstore_pd( &ptr_Qx[ii], i_mask, v_Qx0 );
			_mm256_maskstore_pd( &ptr_qx[ii], i_mask, v_qx0 );

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
	
	int ii, jj, ll;

	__m256
		t_sign, t_ones, t_zeros,
		t_mask0, t_mask1,
		t_lam, t_dlam, t_t, t_dt,
		t_tmp0, t_tmp1,
		t_alpha0, t_alpha1;

	__m128
		s_sign, s_ones, s_mask, s_mask0, s_mask1, s_zeros,
		s_lam, s_dlam, s_t, s_dt, s_tmp0, s_tmp1, s_alpha0, s_alpha1;
	
	__m256d
		v_sign, v_alpha, v_mask, v_left,
		v_temp0, v_dt0, v_dux, v_db0, v_dlam0, v_lamt0, v_t0, v_lam0,
		v_temp1, v_dt1, v_db1, v_dlam1, v_lamt1, v_t1, v_lam1;
	
	__m128d
		u_sign, u_dux, u_alpha,
		u_dt0, u_temp0, u_db0, u_dlam0, u_lamt0, u_t0, u_lam0,
		u_dt1, u_temp1, u_db1, u_dlam1, u_lamt1, u_t1, u_lam1;
	
	__m256i
		i_mask;
	
	int nu0, nx0, nb0, ng0, nt0;

	long long long_sign = 0x8000000000000000;
	v_sign = _mm256_broadcast_sd( (double *) &long_sign );
	u_sign = _mm_loaddup_pd( (double *) &long_sign );

	int int_sign = 0x80000000;
	s_sign = _mm_broadcast_ss( (float *) &int_sign );
	t_sign = _mm256_broadcast_ss( (float *) &int_sign );
	
	s_ones  = _mm_set_ps( 1.0, 1.0, 1.0, 1.0 );
	s_zeros = _mm_setzero_ps( );

	s_alpha0 = _mm_set_ps( 1.0, 1.0, 1.0, 1.0 );

	t_ones  = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );
	t_zeros = _mm256_setzero_ps( );

	t_alpha0 = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );
	t_alpha1 = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );
	
	double ll_left;

	static double d_mask[4]  = {0.5, 1.5, 2.5, 3.5};

	static double dux_tmp[4] = {};

	double alpha = ptr_alpha[0];

	double
		*ptr_db, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lamt, *ptr_lam, *ptr_dlam;
	
	int
		*ptr_idxb;
	
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
		for(ll=0; ll<nt0-3; ll+=4)
			{
			v_dt0   = _mm256_loadu_pd( &ptr_dt[0*nt0+ll] );
			v_dt1   = _mm256_xor_pd( v_dt0, v_sign );
			v_db0   = _mm256_loadu_pd( &ptr_db[0*nt0+ll] );
			v_db1   = _mm256_loadu_pd( &ptr_db[1*nt0+ll] );
			v_dt0   = _mm256_sub_pd ( v_dt0, v_db0 );
			v_dt1   = _mm256_add_pd ( v_dt1, v_db1 );
			v_t0    = _mm256_loadu_pd( &ptr_t[0*nt0+ll] );
			v_t1    = _mm256_loadu_pd( &ptr_t[1*nt0+ll] );
			v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
			v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
			_mm256_storeu_pd( &ptr_dt[0*nt0+ll], v_dt0 );
			_mm256_storeu_pd( &ptr_dt[1*nt0+ll], v_dt1 );

			v_lamt0 = _mm256_loadu_pd( &ptr_lamt[0*nt0+ll] );
			v_lamt1 = _mm256_loadu_pd( &ptr_lamt[1*nt0+ll] );
			v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
			v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[0*nt0+ll] );
			v_dlam1 = _mm256_loadu_pd( &ptr_dlam[1*nt0+ll] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[0*nt0+ll] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[1*nt0+ll] );
			v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
			v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
			v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
			v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
			_mm256_storeu_pd( &ptr_dlam[0*nt0+ll], v_dlam0 );
			_mm256_storeu_pd( &ptr_dlam[1*nt0+ll], v_dlam1 );

			t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
			t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
			t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
			t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
			t_lam    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam1 ) ), 0x20 );
			t_t      = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t1 ) ), 0x20 );
			t_lam    = _mm256_xor_ps( t_lam, t_sign );
			t_t      = _mm256_xor_ps( t_t, t_sign );
			t_tmp0   = _mm256_div_ps( t_lam, t_dlam );
			t_tmp1   = _mm256_div_ps( t_t, t_dt );
			t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
			t_tmp1   = _mm256_blendv_ps( t_ones, t_tmp1, t_mask1 );
			t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );
			t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp1 );

			}
		if(ll<nt0)
			{

			ll_left = nt0 - ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_dt0   = _mm256_loadu_pd( &ptr_dt[0*nt0+ll] );
			v_dt1   = _mm256_xor_pd( v_dt0, v_sign );
			v_db0   = _mm256_loadu_pd( &ptr_db[0*nt0+ll] );
			v_db1   = _mm256_loadu_pd( &ptr_db[1*nt0+ll] );
			v_dt0   = _mm256_sub_pd ( v_dt0, v_db0 );
			v_dt1   = _mm256_add_pd ( v_dt1, v_db1 );
			v_t0    = _mm256_loadu_pd( &ptr_t[0*nt0+ll] );
			v_t1    = _mm256_loadu_pd( &ptr_t[1*nt0+ll] );
			v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
			v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
			_mm256_maskstore_pd( &ptr_dt[0*nt0+ll], i_mask, v_dt0 );
			_mm256_maskstore_pd( &ptr_dt[1*nt0+ll], i_mask, v_dt1 );

			v_lamt0 = _mm256_loadu_pd( &ptr_lamt[0*nt0+ll] );
			v_lamt1 = _mm256_loadu_pd( &ptr_lamt[1*nt0+ll] );
			v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
			v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[0*nt0+ll] );
			v_dlam1 = _mm256_loadu_pd( &ptr_dlam[1*nt0+ll] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[0*nt0+ll] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[1*nt0+ll] );
			v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
			v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
			v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
			v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
			_mm256_maskstore_pd( &ptr_dlam[0*nt0+ll], i_mask, v_dlam0 );
			_mm256_maskstore_pd( &ptr_dlam[1*nt0+ll], i_mask, v_dlam1 );

			if(ll<nt0-2) // 3 left
				{

				t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
				t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
				t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
				t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
				t_lam    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam1 ) ), 0x20 );
				t_t      = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t1 ) ), 0x20 );
				t_lam    = _mm256_xor_ps( t_lam, t_sign );
				t_t      = _mm256_xor_ps( t_t, t_sign );
				t_tmp0   = _mm256_div_ps( t_lam, t_dlam );
				t_tmp1   = _mm256_div_ps( t_t, t_dt );
				t_mask0  = _mm256_blend_ps( t_zeros, t_mask0, 0x77 );
				t_mask1  = _mm256_blend_ps( t_zeros, t_mask1, 0x77 );
				t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
				t_tmp1   = _mm256_blendv_ps( t_ones, t_tmp1, t_mask1 );
				t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );
				t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp1 );

				}
			else // 1 or 2 left
				{

				s_mask   = _mm256_cvtpd_ps( v_mask );
				s_mask   = _mm_shuffle_ps( s_mask, s_mask, 0x44 );
				t_mask1  = _mm256_permute2f128_ps( _mm256_castps128_ps256( s_mask ), _mm256_castps128_ps256( s_mask ), 0x20 );
				t_mask1  = _mm256_cmp_ps( t_mask1, t_zeros, 0x01 );

				v_dt0    = _mm256_permute2f128_pd( v_dt0, v_dt1, 0x20 );
				v_t0     = _mm256_permute2f128_pd( v_t0, v_t1, 0x20 );
				v_dlam0  = _mm256_permute2f128_pd( v_dlam0, v_dlam1, 0x20 );
				v_lam0   = _mm256_permute2f128_pd( v_lam0, v_lam1, 0x20 );

				t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), 0x20 );
				t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
				t_lam    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), 0x20 );
				t_mask0  = _mm256_and_ps( t_mask0, t_mask1 );
				t_lam    = _mm256_xor_ps( t_lam, t_sign );
				t_tmp0   = _mm256_div_ps( t_lam, t_dlam );
				t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
				t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );

				}
			}
		}		

	// reduce alpha
	t_alpha0 = _mm256_min_ps( t_alpha0, t_alpha1 );
	s_alpha0 = _mm256_extractf128_ps( t_alpha0, 0x1 );
//	s_alpha1 = _mm256_extractf128_ps( t_alpha0, 0x1 );
//	s_alpha0  = _mm_min_ps( s_alpha0 , s_alpha1 );
	s_alpha1 = _mm256_castps256_ps128( t_alpha0 );
	s_alpha0 = _mm_min_ps( s_alpha0, s_alpha1 );
	
	v_alpha = _mm256_cvtps_pd( s_alpha0 );
	u_alpha = _mm256_extractf128_pd( v_alpha, 0x1 );
	u_alpha = _mm_min_pd( u_alpha, _mm256_castpd256_pd128( v_alpha ) );
	u_alpha = _mm_min_sd( u_alpha, _mm_permute_pd( u_alpha, 0x1 ) );
/*	u_alpha = _mm_min_sd( u_alpha, _mm_load_sd( &alpha ) );*/
	_mm_store_sd( &alpha, u_alpha );

	
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

	int nu0, nx0, nx1, nb0, ng0, nt0;

	int jj, ll, ll_bkp, ll_end;
	double ll_left;
	
	double d_mask[4] = {0.5, 1.5, 2.5, 3.5};
	
	__m128d
		u_mu0, u_tmp;

	__m256d
		v_mask, v_left, v_zeros,
		v_alpha, v_ux, v_dux, v_pi, v_dpi, 
		v_t0, v_dt0, v_lam0, v_dlam0, v_mu0,
		v_t1, v_dt1, v_lam1, v_dlam1, v_mu1;
		
	__m256i
		i_mask;
		
	v_alpha = _mm256_broadcast_sd( &alpha );
	
	v_zeros = _mm256_setzero_pd();
	v_mu0 = _mm256_setzero_pd();
	v_mu1 = _mm256_setzero_pd();

	double
		*ptr_pi_bkp, *ptr_pi, *ptr_dpi, *ptr_ux_bkp, *ptr_ux, *ptr_dux, *ptr_t_bkp, *ptr_t, *ptr_dt, *ptr_lam_bkp, *ptr_lam, *ptr_dlam;

	// multipliers of equality constraints: backup, compute step, update
	for(jj=1; jj<=N; jj++)
		{

		nx0 = nx[jj];

		ptr_pi_bkp = hspi_bkp[jj].pa;
		ptr_pi     = hspi[jj].pa;
		ptr_dpi    = hsdpi[jj].pa;

		ll = 0;
		for(; ll<nx0-3; ll+=4)
			{
			v_pi  = _mm256_loadu_pd( &ptr_pi[ll] );
			v_dpi = _mm256_loadu_pd( &ptr_dpi[ll] );
			_mm256_storeu_pd( &ptr_pi_bkp[ll], v_pi );
			v_dpi = _mm256_sub_pd( v_dpi, v_pi );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_storeu_pd( &ptr_pi[ll], v_pi );
			}
		if(ll<nx0)
			{
			ll_left = nx0-ll;
			v_left= _mm256_broadcast_sd( &ll_left );
			v_mask= _mm256_loadu_pd( d_mask );
			i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_pi  = _mm256_loadu_pd( &ptr_pi[ll] );
			v_dpi = _mm256_loadu_pd( &ptr_dpi[ll] );
			_mm256_maskstore_pd( &ptr_pi_bkp[ll], i_mask, v_pi );
			v_dpi = _mm256_sub_pd( v_dpi, v_pi );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_maskstore_pd( &ptr_pi[ll], i_mask, v_pi );
			}

		}

	// inputs and states: backup, compute step, update
	for(jj=0; jj<=N; jj++)
		{

		nx0 = nx[jj];
		nu0 = nu[jj];
		
		ptr_ux_bkp  = hsux_bkp[jj].pa;
		ptr_ux      = hsux[jj].pa;
		ptr_dux     = hsdux[jj].pa;

		for(ll=0; ll<nu0+nx0-3; ll+=4)
			{
			v_ux  = _mm256_loadu_pd( &ptr_ux[ll] );
			v_dux = _mm256_loadu_pd( &ptr_dux[ll] );
			_mm256_storeu_pd( &ptr_ux_bkp[ll], v_ux );
			v_dux = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_storeu_pd( &ptr_ux[ll], v_ux );
			}
		if(ll<nu0+nx0)
			{
			ll_left = nu0+nx0-ll;
			v_left = _mm256_broadcast_sd( &ll_left );
			v_mask = _mm256_loadu_pd( d_mask );
			i_mask = _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_ux  = _mm256_loadu_pd( &ptr_ux[ll] );
			v_dux = _mm256_loadu_pd( &ptr_dux[ll] );
			_mm256_maskstore_pd( &ptr_ux_bkp[ll], i_mask, v_ux );
			v_dux = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_maskstore_pd( &ptr_ux[ll], i_mask, v_ux );
			}

		}

	// multipliers and slack of inequalities: backup, update, compute mu
	for(jj=0; jj<=N; jj++)
		{

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;
		
		ptr_t_bkp   = hst_bkp[jj].pa;
		ptr_t       = hst[jj].pa;
		ptr_dt      = hsdt[jj].pa;
		ptr_lam_bkp = hslam_bkp[jj].pa;
		ptr_lam     = hslam[jj].pa;
		ptr_dlam    = hsdlam[jj].pa;

		// box constraints
		ll = 0;
		for(; ll<2*nt0-7; ll+=8)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_t1    = _mm256_loadu_pd( &ptr_t[ll+4] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ll+4] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dt1   = _mm256_loadu_pd( &ptr_dt[ll+4] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			v_dlam1 = _mm256_loadu_pd( &ptr_dlam[ll+4] );
			_mm256_storeu_pd( &ptr_t_bkp[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_t_bkp[ll+4], v_t1 );
			_mm256_storeu_pd( &ptr_lam_bkp[ll+0], v_lam0 );
			_mm256_storeu_pd( &ptr_lam_bkp[ll+4], v_lam1 );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
#endif
			_mm256_storeu_pd( &ptr_t[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_t[ll+4], v_t1 );
			_mm256_storeu_pd( &ptr_lam[ll+0], v_lam0 );
			_mm256_storeu_pd( &ptr_lam[ll+4], v_lam1 );
#if defined(TARGET_X64_AVX2)
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
#endif
			}
		for(; ll<2*nt0-3; ll+=4)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			_mm256_storeu_pd( &ptr_t_bkp[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_lam_bkp[ll+0], v_lam0 );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
#endif
			_mm256_storeu_pd( &ptr_t[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_lam[ll+0], v_lam0 );
#if defined(TARGET_X64_AVX2)
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
#endif
			}
		if(ll<2*nt0)
			{
			ll_left = 2*nt0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			_mm256_maskstore_pd( &ptr_t_bkp[ll+0], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_lam_bkp[ll+0], i_mask, v_lam0 );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
#endif
			_mm256_maskstore_pd( &ptr_t[ll+0], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_lam[ll+0], i_mask, v_lam0 );
#if defined(TARGET_X64_AVX2)
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
#endif
			}

		}

	// reduce mu
	v_mu0 = _mm256_add_pd( v_mu0, v_mu1 );
	u_mu0 = _mm_add_pd( _mm256_castpd256_pd128( v_mu0 ), _mm256_extractf128_pd( v_mu0, 0x1 ) );
	u_mu0 = _mm_hadd_pd( u_mu0, u_mu0 );
	u_tmp = _mm_load_sd( &mu_scal );
	u_mu0 = _mm_mul_sd( u_mu0, u_tmp );
	_mm_store_sd( ptr_mu, u_mu0 );

	return;
	
	}



void d_compute_mu_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt)
	{

	int ii;
	
	int jj, ll, ll_bkp, ll_end;
	double ll_left;
	
	double d_mask[4] = {0.5, 1.5, 2.5, 3.5};
	
	__m128d
		u_mu0, u_tmp;

	__m256d
		v_alpha, v_mask, v_left, v_zeros,
		v_t0, v_dt0, v_lam0, v_dlam0, v_mu0, 
		v_t1, v_dt1, v_lam1, v_dlam1, v_mu1;
		
	double
		*ptr_t, *ptr_lam, *ptr_dt, *ptr_dlam;

	int nb0, ng0, nt0;
		
	v_alpha = _mm256_set_pd( alpha, alpha, alpha, alpha );
	
	v_zeros = _mm256_setzero_pd();
	v_mu0 = _mm256_setzero_pd();
	v_mu1 = _mm256_setzero_pd();

	for(jj=0; jj<=N; jj++)
		{
		
		ptr_t    = hst[jj].pa;
		ptr_lam  = hslam[jj].pa;
		ptr_dt   = hsdt[jj].pa;
		ptr_dlam = hsdlam[jj].pa;

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;

		ll = 0;
		for(; ll<2*nt0-7; ll+=8)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_t1    = _mm256_loadu_pd( &ptr_t[ll+4] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ll+4] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dt1   = _mm256_loadu_pd( &ptr_dt[ll+4] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			v_dlam1 = _mm256_loadu_pd( &ptr_dlam[ll+4] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
#endif
			}
		for(; ll<2*nt0-3; ll+=4)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[0*nb0+ll] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[0*nb0+ll] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[0*nb0+ll] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[0*nb0+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
#endif
			}
		if(ll<2*nt0)
			{
			ll_left = 2*nt0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );

			v_t0    = _mm256_loadu_pd( &ptr_t[0*nb0+ll] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[0*nb0+ll] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[0*nb0+ll] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[0*nb0+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
#endif
			}

		}

	v_mu0 = _mm256_add_pd( v_mu0, v_mu1 );
	u_mu0 = _mm_add_pd( _mm256_castpd256_pd128( v_mu0 ), _mm256_extractf128_pd( v_mu0, 0x1 ) );
	u_mu0 = _mm_hadd_pd( u_mu0, u_mu0 );
	u_tmp = _mm_load_sd( &mu_scal );
	u_mu0 = _mm_mul_sd( u_mu0, u_tmp );
	_mm_store_sd( ptr_mu, u_mu0 );

	return;

	}



// IPM with residuals

void d_update_hessian_gradient_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, struct blasfeo_dvec *hsres_d, struct blasfeo_dvec *hsres_m, struct blasfeo_dvec *hst, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hstinv, struct blasfeo_dvec *hsQx, struct blasfeo_dvec *hsqx)
	{
	
	int ii, jj, bs0;
	
	int nb0, ng0, nt0;
	
	double 
		*ptr_res_d, *ptr_Qx, *ptr_qx, *ptr_t, *ptr_lam, *ptr_res_m, *ptr_t_inv;
	
	__m256d
		v_ones,
		v_tmp0, v_tinv0, v_lam0, v_resm0, v_resd0,
		v_tmp1, v_tinv1, v_lam1, v_resm1, v_resd1;
	
	__m256i
		i_mask;
	
	v_ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	double ii_left;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	for(jj=0; jj<=N; jj++)
		{
		
		ptr_t     = hst[jj].pa;
		ptr_lam   = hslam[jj].pa;
		ptr_t_inv = hstinv[jj].pa;
		ptr_res_d = hsres_d[jj].pa;
		ptr_res_m = hsres_m[jj].pa;
		ptr_Qx    = hsQx[jj].pa;
		ptr_qx    = hsqx[jj].pa;

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;

		for(ii=0; ii<nt0-3; ii+=4)
			{

			v_tinv0 = _mm256_loadu_pd( &ptr_t[ii+0] );
			v_tinv1 = _mm256_loadu_pd( &ptr_t[ii+nt0] );
			v_tinv0 = _mm256_div_pd( v_ones, v_tinv0 );
			v_tinv1 = _mm256_div_pd( v_ones, v_tinv1 );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ii+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ii+nt0] );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[ii+0] );
			v_resm1 = _mm256_loadu_pd( &ptr_res_m[ii+nt0] );
			v_resd0 = _mm256_loadu_pd( &ptr_res_d[ii+0] );
			v_resd1 = _mm256_loadu_pd( &ptr_res_d[ii+nt0] );
			v_tmp0  = _mm256_mul_pd( v_tinv0, v_lam0 );
			v_tmp1  = _mm256_mul_pd( v_tinv1, v_lam1 );
			_mm256_storeu_pd( &ptr_t_inv[ii+0], v_tinv0 );
			_mm256_storeu_pd( &ptr_t_inv[ii+nt0], v_tinv1 );
			v_tmp0  = _mm256_add_pd( v_tmp0, v_tmp1 );
			_mm256_storeu_pd( &ptr_Qx[ii+0], v_tmp0 );
			v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
			v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
			v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
			v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
			v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
			v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
			_mm256_storeu_pd( &ptr_qx[ii+0], v_tmp0 );

			}
		if(ii<nt0)
			{

			ii_left = nt0-ii;
			i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

			v_tinv0 = _mm256_loadu_pd( &ptr_t[ii+0] );
			v_tinv1 = _mm256_loadu_pd( &ptr_t[ii+nt0] );
			v_tinv0 = _mm256_div_pd( v_ones, v_tinv0 );
			v_tinv1 = _mm256_div_pd( v_ones, v_tinv1 );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ii+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ii+nt0] );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[ii+0] );
			v_resm1 = _mm256_loadu_pd( &ptr_res_m[ii+nt0] );
			v_resd0 = _mm256_loadu_pd( &ptr_res_d[ii+0] );
			v_resd1 = _mm256_loadu_pd( &ptr_res_d[ii+nt0] );
			v_tmp0  = _mm256_mul_pd( v_tinv0, v_lam0 );
			v_tmp1  = _mm256_mul_pd( v_tinv1, v_lam1 );
			_mm256_maskstore_pd( &ptr_t_inv[ii+0], i_mask, v_tinv0 );
			_mm256_maskstore_pd( &ptr_t_inv[ii+nt0], i_mask, v_tinv1 );
			v_tmp0  = _mm256_add_pd( v_tmp0, v_tmp1 );
			_mm256_maskstore_pd( &ptr_Qx[ii+0], i_mask, v_tmp0 );
			v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
			v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
			v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
			v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
			v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
			v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
			_mm256_maskstore_pd( &ptr_qx[ii+0], i_mask, v_tmp0 );

			}

		}

	return;

	}



void d_compute_alpha_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hst, struct blasfeo_dvec *hstinv, struct blasfeo_dvec *hslam, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsres_d, struct blasfeo_dvec *hsres_m, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hsdlam, double *ptr_alpha)
	{
	
	int ii, jj, ll;

	int nu0, nx0, nb0, ng0, nt0;

	double alpha = ptr_alpha[0];
	
	double
		*ptr_res_d, *ptr_res_m, *ptr_dux, *ptr_t, *ptr_t_inv, *ptr_dt, *ptr_lam, *ptr_dlam;
	
	int
		*ptr_idxb;
	
	__m128d
		u_dux, u_alpha,
		u_resm0, u_resd0, u_dt0, u_dlam0, u_tmp0, u_tinv0, u_lam0, u_t0,
		u_resm1, u_resd1, u_dt1, u_dlam1, u_tmp1, u_tinv1, u_lam1, u_t1;
	
	__m256d
		v_dux, v_sign, v_alpha,
		v_resm0, v_resd0, v_dt0, v_dlam0, v_tmp0, v_tinv0, v_lam0, v_t0,
		v_resm1, v_resd1, v_dt1, v_dlam1, v_tmp1, v_tinv1, v_lam1, v_t1;
	
	__m128
		s_dlam, s_lam, s_mask0, s_tmp0,
		s_alpha0,
		s_alpha1;

	__m256
		t_dlam, t_dt, t_lam, t_t, t_sign, t_ones, t_zeros,
		t_mask0, t_tmp0, t_alpha0,
		t_mask1, t_tmp1, t_alpha1;
	
	__m256i
		i_mask;

	long long long_sign = 0x8000000000000000;
	v_sign = _mm256_broadcast_sd( (double *) &long_sign );

	int int_sign = 0x80000000;
	t_sign = _mm256_broadcast_ss( (float *) &int_sign );

	t_ones  = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );

	t_zeros = _mm256_setzero_ps( );

	// initialize alpha with 1.0
	t_alpha0 = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );
	t_alpha1 = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );

	for(jj=0; jj<=N; jj++)
		{

		ptr_res_d = hsres_d[jj].pa;
		ptr_res_m = hsres_m[jj].pa;
		ptr_dux   = hsdux[jj].pa;
		ptr_t     = hst[jj].pa;
		ptr_t_inv = hstinv[jj].pa;
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

		for(ll=0; ll<nt0-3; ll+=4)
			{

			v_tmp0  = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_resd0 = _mm256_loadu_pd( &ptr_res_d[ll+0] );
			v_resd1 = _mm256_loadu_pd( &ptr_res_d[ll+nt0] );
			v_dt0   = _mm256_sub_pd( v_tmp0, v_resd0 );
			v_dt1   = _mm256_sub_pd( v_resd1, v_tmp0 );
			_mm256_storeu_pd( &ptr_dt[ll+0], v_dt0 );
			_mm256_storeu_pd( &ptr_dt[ll+nt0], v_dt1 );

			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ll+nt0] );
			v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
			v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[ll+0] );
			v_resm1 = _mm256_loadu_pd( &ptr_res_m[ll+nt0] );
			v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
			v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
			v_tinv0 = _mm256_loadu_pd( &ptr_t_inv[ll+0] );
			v_tinv1 = _mm256_loadu_pd( &ptr_t_inv[ll+nt0] );
			v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
			v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
			v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
			v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
			_mm256_storeu_pd( &ptr_dlam[ll+0], v_dlam0 );
			_mm256_storeu_pd( &ptr_dlam[ll+nt0], v_dlam1 );

			t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
			t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
			t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
			t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
			v_t0  = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_t1  = _mm256_loadu_pd( &ptr_t[ll+nt0] );
			t_lam    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam1 ) ), 0x20 );
			t_t      = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t1 ) ), 0x20 );
			t_lam    = _mm256_xor_ps( t_lam, t_sign );
			t_t      = _mm256_xor_ps( t_t, t_sign );
			t_tmp0   = _mm256_div_ps( t_lam, t_dlam );
			t_tmp1   = _mm256_div_ps( t_t, t_dt );
			t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
			t_tmp1   = _mm256_blendv_ps( t_ones, t_tmp1, t_mask1 );
			t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );
			t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp1 );

			}
		if(ll<nt0)
			{
			
			if(nt0-ll==1)
				{

				u_tmp0  = _mm_loadu_pd( &ptr_dt[ll+0] );
				u_resd0 = _mm_loadu_pd( &ptr_res_d[ll+0] );
				u_resd1 = _mm_loadu_pd( &ptr_res_d[ll+nt0] );
				u_dt0   = _mm_sub_pd( u_tmp0, u_resd0 );
				u_dt1   = _mm_sub_pd( u_resd1, u_tmp0 );
				_mm_store_sd( &ptr_dt[ll+0], u_dt0 );
				_mm_store_sd( &ptr_dt[ll+nt0], u_dt1 );

				u_lam0   = _mm_load_sd( &ptr_lam[ll+0] );
				u_lam1   = _mm_load_sd( &ptr_lam[ll+nt0] );
				u_tmp0   = _mm_mul_pd( u_lam0, u_dt0 );
				u_tmp1   = _mm_mul_pd( u_lam1, u_dt1 );
				u_resm0  = _mm_load_sd( &ptr_res_m[ll+0] );
				u_resm1  = _mm_load_sd( &ptr_res_m[ll+nt0] );
				u_tmp0   = _mm_add_pd( u_tmp0, u_resm0 );
				u_tmp1   = _mm_add_pd( u_tmp1, u_resm1 );
				u_tinv0  = _mm_load_sd( &ptr_t_inv[ll+0] );
				u_tinv1  = _mm_load_sd( &ptr_t_inv[ll+nt0] );
				u_tinv0  = _mm_xor_pd( u_tinv0, _mm256_castpd256_pd128( v_sign ) );
				u_tinv1  = _mm_xor_pd( u_tinv1, _mm256_castpd256_pd128( v_sign ) );
				u_dlam0  = _mm_mul_pd( u_tinv0, u_tmp0 );
				u_dlam1  = _mm_mul_pd( u_tinv1, u_tmp1 );
				_mm_store_sd( &ptr_dlam[ll+0], u_dlam0 );
				_mm_store_sd( &ptr_dlam[ll+nt0], u_dlam1 );

				u_dt1    = _mm_movedup_pd( u_dt1 );
				u_dt0    = _mm_move_sd( u_dt1, u_dt0 );
				u_t1     = _mm_loaddup_pd( &ptr_t[ll+nt0] );
				u_t0     = _mm_load_sd( &ptr_t[ll+0] );
				u_t0     = _mm_move_sd( u_t1, u_t0 );
				u_dlam1  = _mm_movedup_pd( u_dlam1 );
				u_dlam0  = _mm_move_sd( u_dlam1, u_dlam0 );
				u_lam1   = _mm_movedup_pd( u_lam1 );
				u_lam0   = _mm_move_sd( u_lam1, u_lam0 );

				v_dlam0  = _mm256_castpd128_pd256( u_dlam0 );
				v_dlam0  = _mm256_insertf128_pd( v_dlam0, u_dt0, 0x1 );
				v_lam0   = _mm256_castpd128_pd256( u_lam0 );
				v_lam0   = _mm256_insertf128_pd( v_lam0, u_t0, 0x1 );

				s_dlam   = _mm256_cvtpd_ps( v_dlam0 );
				s_lam    = _mm256_cvtpd_ps( v_lam0 );
				s_mask0  = _mm_cmp_ps( s_dlam, _mm256_castps256_ps128( t_zeros ), 0x01 );
				s_lam    = _mm_xor_ps( s_lam, _mm256_castps256_ps128( t_sign ) );
				s_tmp0   = _mm_div_ps( s_lam, s_dlam );
				s_tmp0   = _mm_blendv_ps( _mm256_castps256_ps128( t_ones ), s_tmp0, s_mask0 );
				t_tmp0   = _mm256_blend_ps( t_ones, _mm256_castps128_ps256( s_tmp0 ), 0xf );
				t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );

				}
			else if(nt0-ll==2)
				{

				u_tmp0  = _mm_loadu_pd( &ptr_dt[ll+0] );
				u_resd0 = _mm_loadu_pd( &ptr_res_d[ll+0] );
				u_resd1 = _mm_loadu_pd( &ptr_res_d[ll+nt0] );
				u_dt0   = _mm_sub_pd( u_tmp0, u_resd0 );
				u_dt1   = _mm_sub_pd( u_resd1, u_tmp0 );
				_mm_storeu_pd( &ptr_dt[ll+0], u_dt0 );
				_mm_storeu_pd( &ptr_dt[ll+nt0], u_dt1 );

				u_lam0   = _mm_loadu_pd( &ptr_lam[ll+0] );
				u_lam1   = _mm_loadu_pd( &ptr_lam[ll+nt0] );
				u_tmp0   = _mm_mul_pd( u_lam0, u_dt0 );
				u_tmp1   = _mm_mul_pd( u_lam1, u_dt1 );
				u_resm0  = _mm_loadu_pd( &ptr_res_m[ll+0] );
				u_resm1  = _mm_loadu_pd( &ptr_res_m[ll+nt0] );
				u_tmp0   = _mm_add_pd( u_tmp0, u_resm0 );
				u_tmp1   = _mm_add_pd( u_tmp1, u_resm1 );
				u_tinv0  = _mm_loadu_pd( &ptr_t_inv[ll+0] );
				u_tinv1  = _mm_loadu_pd( &ptr_t_inv[ll+nt0] );
				u_tinv0  = _mm_xor_pd( u_tinv0, _mm256_castpd256_pd128( v_sign ) );
				u_tinv1  = _mm_xor_pd( u_tinv1, _mm256_castpd256_pd128( v_sign ) );
				u_dlam0  = _mm_mul_pd( u_tinv0, u_tmp0 );
				u_dlam1  = _mm_mul_pd( u_tinv1, u_tmp1 );
				_mm_storeu_pd( &ptr_dlam[ll+0], u_dlam0 );
				_mm_storeu_pd( &ptr_dlam[ll+nt0], u_dlam1 );

				v_dt0    = _mm256_castpd128_pd256( u_dt0 );
				v_dt0    = _mm256_insertf128_pd( v_dt0, u_dt1, 0x1 );
				v_t0     = _mm256_castpd128_pd256( _mm_loadu_pd( &ptr_t[ll+0] ) );
				v_t0     = _mm256_insertf128_pd( v_t0, _mm_loadu_pd( &ptr_t[ll+nt0]), 0x1 );
				v_dlam0  = _mm256_castpd128_pd256( u_dlam0 );
				v_dlam0  = _mm256_insertf128_pd( v_dlam0, u_dlam1, 0x1 );
				v_lam0   = _mm256_castpd128_pd256( u_lam0 );
				v_lam0   = _mm256_insertf128_pd( v_lam0, u_lam1, 0x1 );

				t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), 0x20 );
				t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
				t_lam    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), 0x20 );
				t_lam    = _mm256_xor_ps( t_lam, t_sign );
				t_tmp0   = _mm256_div_ps( t_lam, t_dlam );
				t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
				t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );

				}
			else // if(ng-ll==3)
				{

				i_mask = _mm256_castpd_si256( _mm256_set_pd( 1.0, -1.0, -1.0, -1.0 ) );

				v_tmp0  = _mm256_loadu_pd( &ptr_dt[ll+0] );
				v_resd0 = _mm256_loadu_pd( &ptr_res_d[ll+0] );
				v_resd1 = _mm256_loadu_pd( &ptr_res_d[ll+nt0] );
				v_dt0   = _mm256_sub_pd( v_tmp0, v_resd0 );
				v_dt1   = _mm256_sub_pd( v_resd1, v_tmp0 );
				_mm256_maskstore_pd( &ptr_dt[ll+0], i_mask, v_dt0 );
				_mm256_maskstore_pd( &ptr_dt[ll+nt0], i_mask, v_dt1 );

				v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
				v_lam1  = _mm256_loadu_pd( &ptr_lam[ll+nt0] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
				v_resm0 = _mm256_loadu_pd( &ptr_res_m[ll+0] );
				v_resm1 = _mm256_loadu_pd( &ptr_res_m[ll+nt0] );
				v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
				v_tinv0 = _mm256_loadu_pd( &ptr_t_inv[ll+0] );
				v_tinv1 = _mm256_loadu_pd( &ptr_t_inv[ll+nt0] );
				v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
				v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
				v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
				v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
				_mm256_maskstore_pd( &ptr_dlam[ll+0], i_mask, v_dlam0 );
				_mm256_maskstore_pd( &ptr_dlam[ll+nt0], i_mask, v_dlam1 );

				t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
				t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
				t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
				t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
				v_t0  = _mm256_loadu_pd( &ptr_t[ll+0] );
				v_t1  = _mm256_loadu_pd( &ptr_t[ll+nt0] );
				t_lam    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam1 ) ), 0x20 );
				t_t      = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t1 ) ), 0x20 );
				t_lam    = _mm256_xor_ps( t_lam, t_sign );
				t_t      = _mm256_xor_ps( t_t, t_sign );
				t_tmp0   = _mm256_div_ps( t_lam, t_dlam );
				t_tmp1   = _mm256_div_ps( t_t, t_dt );
				t_mask0  = _mm256_blend_ps( t_zeros, t_mask0, 0x77 );
				t_mask1  = _mm256_blend_ps( t_zeros, t_mask1, 0x77 );
				t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
				t_tmp1   = _mm256_blendv_ps( t_ones, t_tmp1, t_mask1 );
				t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );
				t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp1 );

				}

			}

		}		

	// reduce alpha
	t_alpha0 = _mm256_min_ps( t_alpha0, t_alpha1 );
	s_alpha0 = _mm256_extractf128_ps( t_alpha0, 0x1 );
	s_alpha1 = _mm256_castps256_ps128( t_alpha0 );
	s_alpha0 = _mm_min_ps( s_alpha0, s_alpha1 );
	
	v_alpha = _mm256_cvtps_pd( s_alpha0 );
	u_alpha = _mm256_extractf128_pd( v_alpha, 0x1 );
	u_alpha = _mm_min_pd( u_alpha, _mm256_castpd256_pd128( v_alpha ) );
	u_alpha = _mm_min_sd( u_alpha, _mm_permute_pd( u_alpha, 0x1 ) );
	u_alpha = _mm_min_sd( u_alpha, _mm_load_sd( &alpha ) );
//	u_alpha = _mm_min_sd( u_alpha, _mm_set_sd( 1.0 ) );
	_mm_store_sd( ptr_alpha, u_alpha );

	// store alpha
//	ptr_alpha[0] = alpha;

	return;
	
	}



void d_update_var_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double alpha, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hsdpi, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam)
	{
	
	int ii;

	int nu0, nx0, nx1, nb0, ng0, nt0;

	int jj, ll, ll_bkp, ll_end;
	double ll_left;
	
	double d_mask[4] = {0.5, 1.5, 2.5, 3.5};
	
	__m128d
		u_mu0, u_tmp;

	__m256d
		v_mask, v_left, v_zeros,
		v_alpha, v_ux, v_dux, v_pi, v_dpi, 
		v_t0, v_dt0, v_lam0, v_dlam0, v_mu0,
		v_t1, v_dt1, v_lam1, v_dlam1, v_mu1;
		
	__m256i
		i_mask;
		
	v_alpha = _mm256_broadcast_sd( &alpha );
	
	v_zeros = _mm256_setzero_pd();
	v_mu0 = _mm256_setzero_pd();
	v_mu1 = _mm256_setzero_pd();

	double
		*ptr_pi, *ptr_dpi, *ptr_ux, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lam, *ptr_dlam;

	// multipliers of equality constraints: update
	for(jj=1; jj<=N; jj++)
		{

		nx0 = nx[jj];

		ptr_pi     = hspi[jj].pa;
		ptr_dpi    = hsdpi[jj].pa;

		ll = 0;
		for(; ll<nx0-3; ll+=4)
			{
			v_pi  = _mm256_loadu_pd( &ptr_pi[ll] );
			v_dpi = _mm256_loadu_pd( &ptr_dpi[ll] );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_storeu_pd( &ptr_pi[ll], v_pi );
			}
		if(ll<nx0)
			{
			ll_left = nx0-ll;
			v_left= _mm256_broadcast_sd( &ll_left );
			v_mask= _mm256_loadu_pd( d_mask );
			i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_pi  = _mm256_loadu_pd( &ptr_pi[ll] );
			v_dpi = _mm256_loadu_pd( &ptr_dpi[ll] );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_maskstore_pd( &ptr_pi[ll], i_mask, v_pi );
			}

		}

	// inputs and states: update
	for(jj=0; jj<=N; jj++)
		{

		nx0 = nx[jj];
		nu0 = nu[jj];
		
		ptr_ux      = hsux[jj].pa;
		ptr_dux     = hsdux[jj].pa;

		for(ll=0; ll<nu0+nx0-3; ll+=4)
			{
			v_ux  = _mm256_loadu_pd( &ptr_ux[ll] );
			v_dux = _mm256_loadu_pd( &ptr_dux[ll] );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_storeu_pd( &ptr_ux[ll], v_ux );
			}
		if(ll<nu0+nx0)
			{
			ll_left = nu0+nx0-ll;
			v_left = _mm256_broadcast_sd( &ll_left );
			v_mask = _mm256_loadu_pd( d_mask );
			i_mask = _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_ux  = _mm256_loadu_pd( &ptr_ux[ll] );
			v_dux = _mm256_loadu_pd( &ptr_dux[ll] );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_maskstore_pd( &ptr_ux[ll], i_mask, v_ux );
			}

		}

	// multipliers and slack of inequalities: update
	for(jj=0; jj<=N; jj++)
		{

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;
		
		ptr_t       = hst[jj].pa;
		ptr_dt      = hsdt[jj].pa;
		ptr_lam     = hslam[jj].pa;
		ptr_dlam    = hsdlam[jj].pa;

		ll = 0;
		for(; ll<2*nt0-7; ll+=8)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_t1    = _mm256_loadu_pd( &ptr_t[ll+4] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ll+4] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dt1   = _mm256_loadu_pd( &ptr_dt[ll+4] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			v_dlam1 = _mm256_loadu_pd( &ptr_dlam[ll+4] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
#endif
			_mm256_storeu_pd( &ptr_t[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_t[ll+4], v_t1 );
			_mm256_storeu_pd( &ptr_lam[ll+0], v_lam0 );
			_mm256_storeu_pd( &ptr_lam[ll+4], v_lam1 );
			}
		for(; ll<2*nt0-3; ll+=4)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
#endif
			_mm256_storeu_pd( &ptr_t[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_lam[ll+0], v_lam0 );
			}
		if(ll<2*nt0)
			{
			ll_left = 2*nt0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
#endif
			_mm256_maskstore_pd( &ptr_t[ll+0], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_lam[ll+0], i_mask, v_lam0 );
			}

		}

	return;
	
	}



void d_backup_update_var_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, double alpha, struct blasfeo_dvec *hsux_bkp, struct blasfeo_dvec *hsux, struct blasfeo_dvec *hsdux, struct blasfeo_dvec *hspi_bkp, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hsdpi, struct blasfeo_dvec *hst_bkp, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hslam_bkp, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hsdlam)
	{
	
	int ii;

	int nu0, nx0, nx1, nb0, ng0, nt0;

	int jj, ll, ll_bkp, ll_end;
	double ll_left;
	
	double d_mask[4] = {0.5, 1.5, 2.5, 3.5};
	
	__m128d
		u_mu0, u_tmp;

	__m256d
		v_mask, v_left, v_zeros,
		v_alpha, v_ux, v_dux, v_pi, v_dpi, 
		v_t0, v_dt0, v_lam0, v_dlam0, v_mu0,
		v_t1, v_dt1, v_lam1, v_dlam1, v_mu1;
		
	__m256i
		i_mask;
		
	v_alpha = _mm256_broadcast_sd( &alpha );
	
	v_zeros = _mm256_setzero_pd();
	v_mu0 = _mm256_setzero_pd();
	v_mu1 = _mm256_setzero_pd();

	double
		*ptr_pi_bkp, *ptr_pi, *ptr_dpi, *ptr_ux_bkp, *ptr_ux, *ptr_dux, *ptr_t_bkp, *ptr_t, *ptr_dt, *ptr_lam_bkp, *ptr_lam, *ptr_dlam;

	// multipliers of equality constraints: backup, update
	for(jj=1; jj<=N; jj++)
		{

		nx0 = nx[jj];

		ptr_pi_bkp = hspi_bkp[jj].pa;
		ptr_pi     = hspi[jj].pa;
		ptr_dpi    = hsdpi[jj].pa;

		ll = 0;
		for(; ll<nx0-3; ll+=4)
			{
			v_pi  = _mm256_loadu_pd( &ptr_pi[ll] );
			v_dpi = _mm256_loadu_pd( &ptr_dpi[ll] );
			_mm256_storeu_pd( &ptr_pi_bkp[ll], v_pi );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_storeu_pd( &ptr_pi[ll], v_pi );
			}
		if(ll<nx0)
			{
			ll_left = nx0-ll;
			v_left= _mm256_broadcast_sd( &ll_left );
			v_mask= _mm256_loadu_pd( d_mask );
			i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_pi  = _mm256_loadu_pd( &ptr_pi[ll] );
			v_dpi = _mm256_loadu_pd( &ptr_dpi[ll] );
			_mm256_maskstore_pd( &ptr_pi_bkp[ll], i_mask, v_pi );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_maskstore_pd( &ptr_pi[ll], i_mask, v_pi );
			}

		}

	// inputs and states: backup, update
	for(jj=0; jj<=N; jj++)
		{

		nx0 = nx[jj];
		nu0 = nu[jj];
		
		ptr_ux_bkp  = hsux_bkp[jj].pa;
		ptr_ux      = hsux[jj].pa;
		ptr_dux     = hsdux[jj].pa;

		for(ll=0; ll<nu0+nx0-3; ll+=4)
			{
			v_ux  = _mm256_loadu_pd( &ptr_ux[ll] );
			v_dux = _mm256_loadu_pd( &ptr_dux[ll] );
			_mm256_storeu_pd( &ptr_ux_bkp[ll], v_ux );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_storeu_pd( &ptr_ux[ll], v_ux );
			}
		if(ll<nu0+nx0)
			{
			ll_left = nu0+nx0-ll;
			v_left = _mm256_broadcast_sd( &ll_left );
			v_mask = _mm256_loadu_pd( d_mask );
			i_mask = _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_ux  = _mm256_loadu_pd( &ptr_ux[ll] );
			v_dux = _mm256_loadu_pd( &ptr_dux[ll] );
			_mm256_maskstore_pd( &ptr_ux_bkp[ll], i_mask, v_ux );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_maskstore_pd( &ptr_ux[ll], i_mask, v_ux );
			}

		}

	// multipliers and slack of inequalities: backup, update
	for(jj=0; jj<=N; jj++)
		{

		nb0 = nb[jj];
		ng0 = ng[jj];
		nt0 = nb0 + ng0;
		
		ptr_t_bkp   = hst_bkp[jj].pa;
		ptr_t       = hst[jj].pa;
		ptr_dt      = hsdt[jj].pa;
		ptr_lam_bkp = hslam_bkp[jj].pa;
		ptr_lam     = hslam[jj].pa;
		ptr_dlam    = hsdlam[jj].pa;

		// box constraints
		ll = 0;
		for(; ll<2*nt0-7; ll+=8)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_t1    = _mm256_loadu_pd( &ptr_t[ll+4] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ll+4] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dt1   = _mm256_loadu_pd( &ptr_dt[ll+4] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			v_dlam1 = _mm256_loadu_pd( &ptr_dlam[ll+4] );
			_mm256_storeu_pd( &ptr_t_bkp[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_t_bkp[ll+4], v_t1 );
			_mm256_storeu_pd( &ptr_lam_bkp[ll+0], v_lam0 );
			_mm256_storeu_pd( &ptr_lam_bkp[ll+4], v_lam1 );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
#endif
			_mm256_storeu_pd( &ptr_t[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_t[ll+4], v_t1 );
			_mm256_storeu_pd( &ptr_lam[ll+0], v_lam0 );
			_mm256_storeu_pd( &ptr_lam[ll+4], v_lam1 );
			}
		for(; ll<2*nt0-3; ll+=4)
			{
			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			_mm256_storeu_pd( &ptr_t_bkp[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_lam_bkp[ll+0], v_lam0 );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
#endif
			_mm256_storeu_pd( &ptr_t[ll+0], v_t0 );
			_mm256_storeu_pd( &ptr_lam[ll+0], v_lam0 );
			}
		if(ll<2*nt0)
			{
			ll_left = 2*nt0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_t0    = _mm256_loadu_pd( &ptr_t[ll+0] );
			v_lam0  = _mm256_loadu_pd( &ptr_lam[ll+0] );
			v_dt0   = _mm256_loadu_pd( &ptr_dt[ll+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[ll+0] );
			_mm256_maskstore_pd( &ptr_t_bkp[ll+0], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_lam_bkp[ll+0], i_mask, v_lam0 );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
#endif
			_mm256_maskstore_pd( &ptr_t[ll+0], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_lam[ll+0], i_mask, v_lam0 );
			}

		}

	return;
	
	}



void d_compute_centering_correction_res_mpc_hard_libstr(int N, int *nb, int *ng, double sigma_mu, struct blasfeo_dvec *hsdt, struct blasfeo_dvec *hsdlam, struct blasfeo_dvec *hsres_m)
	{

	int ii, jj;

	int nt0;

	double
		*ptr_res_m, *ptr_dt, *ptr_dlam;
	
	__m256d
		v_sigma_mu,
		v_dt0, v_dlam0, v_tmp0, v_resm0,
		v_dt1, v_dlam1, v_tmp1, v_resm1;
	
	__m256i
		i_mask;
	
	double ii_left;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	v_sigma_mu = _mm256_broadcast_sd( &sigma_mu );

	for(ii=0; ii<=N; ii++)
		{

		ptr_res_m = hsres_m[ii].pa;
		ptr_dt    = hsdt[ii].pa;
		ptr_dlam  = hsdlam[ii].pa;

		nt0 = nb[ii]+ng[ii];

		jj = 0;
		for(; jj<2*nt0-7; jj+=8)
			{
			v_dt0   = _mm256_loadu_pd( &ptr_dt[jj+0] );
			v_dt1   = _mm256_loadu_pd( &ptr_dt[jj+4] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[jj+0] );
			v_dlam1 = _mm256_loadu_pd( &ptr_dlam[jj+4] );
			v_tmp0  = _mm256_mul_pd( v_dt0, v_dlam0 );
			v_tmp1  = _mm256_mul_pd( v_dt1, v_dlam1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_sigma_mu );
			v_tmp1  = _mm256_sub_pd( v_tmp1, v_sigma_mu );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[jj+0] );
			v_resm1 = _mm256_loadu_pd( &ptr_res_m[jj+4] );
			v_resm0 = _mm256_add_pd( v_resm0, v_tmp0 );
			v_resm1 = _mm256_add_pd( v_resm1, v_tmp1 );
			_mm256_storeu_pd( &ptr_res_m[jj+0], v_resm0 );
			_mm256_storeu_pd( &ptr_res_m[jj+4], v_resm1 );
			}
		for(; jj<2*nt0-3; jj+=4)
			{
			v_dt0   = _mm256_loadu_pd( &ptr_dt[jj+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[jj+0] );
			v_tmp0  = _mm256_mul_pd( v_dt0, v_dlam0 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_sigma_mu );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[jj+0] );
			v_resm0 = _mm256_add_pd( v_resm0, v_tmp0 );
			_mm256_storeu_pd( &ptr_res_m[jj+0], v_resm0 );
			}
		if(jj<2*nt0)
			{
			ii_left = 2*nt0-jj;
			i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

			v_dt0   = _mm256_loadu_pd( &ptr_dt[jj+0] );
			v_dlam0 = _mm256_loadu_pd( &ptr_dlam[jj+0] );
			v_tmp0  = _mm256_mul_pd( v_dt0, v_dlam0 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_sigma_mu );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[jj+0] );
			v_resm0 = _mm256_add_pd( v_resm0, v_tmp0 );
			_mm256_maskstore_pd( &ptr_res_m[jj+0], i_mask, v_resm0 );
			}

		}

	}



void d_update_gradient_res_mpc_hard_libstr(int N, int *nx, int *nu, int *nb, int *ng, struct blasfeo_dvec *hsres_d, struct blasfeo_dvec *hsres_m, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hstinv, struct blasfeo_dvec *hsqx)
	{
	
	int ii, jj;

	int nt0;
	
	double temp0, temp1;
	
	double 
		*ptr_res_d, *ptr_Qx, *ptr_qx, *ptr_lam, *ptr_res_m, *ptr_t_inv;
	
	__m256d
		v_ones,
		v_tmp0, v_tinv0, v_lam0, v_resm0, v_resd0,
		v_tmp1, v_tinv1, v_lam1, v_resm1, v_resd1;
	
	__m256i
		i_mask;
	
	v_ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	double ii_left;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	for(jj=0; jj<=N; jj++)
		{
		
		ptr_lam   = hslam[jj].pa;
		ptr_t_inv = hstinv[jj].pa;
		ptr_res_d = hsres_d[jj].pa;
		ptr_res_m = hsres_m[jj].pa;
		ptr_qx    = hsqx[jj].pa;

		nt0 = nb[jj] + ng[jj];

		for(ii=0; ii<nt0-3; ii+=4)
			{

			v_lam0  = _mm256_loadu_pd( &ptr_lam[ii+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ii+nt0] );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[ii+0] );
			v_resm1 = _mm256_loadu_pd( &ptr_res_m[ii+nt0] );
			v_resd0 = _mm256_loadu_pd( &ptr_res_d[ii+0] );
			v_resd1 = _mm256_loadu_pd( &ptr_res_d[ii+nt0] );
			v_tinv0 = _mm256_loadu_pd( &ptr_t_inv[ii+0] );
			v_tinv1 = _mm256_loadu_pd( &ptr_t_inv[ii+nt0] );
			v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
			v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
			v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
			v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
			v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
			v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
			_mm256_storeu_pd( &ptr_qx[ii+0], v_tmp0 );

			}
		if(ii<nt0)
			{

			ii_left = nt0-ii;
			i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

			v_lam0  = _mm256_loadu_pd( &ptr_lam[ii+0] );
			v_lam1  = _mm256_loadu_pd( &ptr_lam[ii+nt0] );
			v_resm0 = _mm256_loadu_pd( &ptr_res_m[ii+0] );
			v_resm1 = _mm256_loadu_pd( &ptr_res_m[ii+nt0] );
			v_resd0 = _mm256_loadu_pd( &ptr_res_d[ii+0] );
			v_resd1 = _mm256_loadu_pd( &ptr_res_d[ii+nt0] );
			v_tinv0 = _mm256_loadu_pd( &ptr_t_inv[ii+0] );
			v_tinv1 = _mm256_loadu_pd( &ptr_t_inv[ii+nt0] );
			v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
			v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
			v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
			v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
			v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
			v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
			_mm256_maskstore_pd( &ptr_qx[ii+0], i_mask, v_tmp0 );

			}

		}

	return;

	}



#endif

