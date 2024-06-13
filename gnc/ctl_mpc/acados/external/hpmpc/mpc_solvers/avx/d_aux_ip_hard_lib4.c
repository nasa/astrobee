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
#include <math.h> // TODO remove if not needed

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX

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

void d_update_hessian_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double **db, double sigma_mu, double **t, double **tinv, double **lam, double **lamt, double **dlam, double **Qx, double **qx)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;
//	const int nal = bs*ncl; // number of doubles per cache line

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

	int ii, jj, bs0;

	double ii_left;

	int nb0, pnb, ng0, png;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

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

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<nb0-3; ii+=4)
				{

				v_tmp0  = _mm256_load_pd( &ptr_t[0*pnb+ii] );
				v_tmp1  = _mm256_load_pd( &ptr_t[1*pnb+ii] );
				v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
				v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ii] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ii] );
				v_qx0   = _mm256_load_pd( &ptr_db[0*pnb+ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[1*pnb+ii] );
				_mm256_store_pd( &ptr_tinv[0*pnb+ii], v_tmp0 );
				_mm256_store_pd( &ptr_tinv[1*pnb+ii], v_tmp1 );
				v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
				v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
				v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
				v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
				_mm256_store_pd( &ptr_lamt[0*pnb+ii], v_lamt0 );
				_mm256_store_pd( &ptr_lamt[1*pnb+ii], v_lamt1 );
				v_qx0   = _mm256_mul_pd( v_qx0, v_lamt0 );
				v_qx1   = _mm256_mul_pd( v_qx1, v_lamt1 );
				v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
				v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
				_mm256_store_pd( &ptr_dlam[0*pnb+ii], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[1*pnb+ii], v_dlam1 );
				v_qx0   = _mm256_add_pd( v_qx0, v_lam0 );
				v_qx1   = _mm256_sub_pd( v_lam1, v_qx1 );
				v_Qx0   = _mm256_add_pd( v_lamt0, v_lamt1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				_mm256_store_pd( &ptr_Qx[ii], v_Qx0 );
				_mm256_store_pd( &ptr_qx[ii], v_qx0 );

				}
			if(ii<nb0)
				{

				ii_left = nb0-ii;
				v_left= _mm256_broadcast_sd( &ii_left );
				v_mask= _mm256_loadu_pd( d_mask );
				i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

				v_tmp0  = _mm256_load_pd( &ptr_t[0*pnb+ii] );
				v_tmp1  = _mm256_load_pd( &ptr_t[1*pnb+ii] );
				v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
				v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
				_mm256_maskstore_pd( &ptr_tinv[0*pnb+ii], i_mask, v_tmp0 );
				_mm256_maskstore_pd( &ptr_tinv[1*pnb+ii], i_mask, v_tmp1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ii] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ii] );
				v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
				v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
				_mm256_maskstore_pd( &ptr_lamt[0*pnb+ii], i_mask, v_lamt0 );
				_mm256_maskstore_pd( &ptr_lamt[1*pnb+ii], i_mask, v_lamt1 );
				v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
				v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
				_mm256_maskstore_pd( &ptr_dlam[0*pnb+ii], i_mask, v_dlam0 );
				_mm256_maskstore_pd( &ptr_dlam[1*pnb+ii], i_mask, v_dlam1 );
				v_qx0   = _mm256_load_pd( &ptr_db[0*pnb+ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[1*pnb+ii] );
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

			png  = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

			for(ii=0; ii<ng0-3; ii+=4)
				{

				v_tmp0  = _mm256_load_pd( &ptr_t[0*png+ii] );
				v_tmp1  = _mm256_load_pd( &ptr_t[1*png+ii] );
				v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
				v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*png+ii] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*png+ii] );
				v_qx0   = _mm256_load_pd( &ptr_db[0*png+ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[1*png+ii] );
				_mm256_store_pd( &ptr_tinv[0*png+ii], v_tmp0 );
				_mm256_store_pd( &ptr_tinv[1*png+ii], v_tmp1 );
				v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
				v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
				v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
				v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
				_mm256_store_pd( &ptr_lamt[0*png+ii], v_lamt0 );
				_mm256_store_pd( &ptr_lamt[1*png+ii], v_lamt1 );
				v_qx0   = _mm256_mul_pd( v_qx0, v_lamt0 );
				v_qx1   = _mm256_mul_pd( v_qx1, v_lamt1 );
				v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
				v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
				_mm256_store_pd( &ptr_dlam[0*png+ii], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[1*png+ii], v_dlam1 );
				v_qx0   = _mm256_add_pd( v_qx0, v_lam0 );
				v_qx1   = _mm256_sub_pd( v_lam1, v_qx1 );
				v_Qx0   = _mm256_add_pd( v_lamt0, v_lamt1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				_mm256_store_pd( &ptr_Qx[ii], v_Qx0 );
				_mm256_store_pd( &ptr_qx[ii], v_qx0 );

				}
			if(ii<ng0)
				{

				ii_left = ng0 - ii;
				v_left  = _mm256_broadcast_sd( &ii_left );
				v_mask  = _mm256_loadu_pd( d_mask );
				i_mask  = _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

				v_tmp0  = _mm256_load_pd( &ptr_t[0*png+ii] );
				v_tmp1  = _mm256_load_pd( &ptr_t[1*png+ii] );
				v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
				v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
				_mm256_maskstore_pd( &ptr_tinv[0*png+ii], i_mask, v_tmp0 );
				_mm256_maskstore_pd( &ptr_tinv[1*png+ii], i_mask, v_tmp1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*png+ii] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*png+ii] );
				v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
				v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
				_mm256_maskstore_pd( &ptr_lamt[0*png+ii], i_mask, v_lamt0 );
				_mm256_maskstore_pd( &ptr_lamt[1*png+ii], i_mask, v_lamt1 );
				v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
				v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
				_mm256_maskstore_pd( &ptr_dlam[0*png+ii], i_mask, v_dlam0 );
				_mm256_maskstore_pd( &ptr_dlam[1*png+ii], i_mask, v_dlam1 );
				v_qx0   = _mm256_load_pd( &ptr_db[0*png+ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[1*png+ii] );
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

	int nu0, nx0, nb0, pnb, ng0, png, cng;

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

	int ii, jj, ll;

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

			for(ll=0; ll<nb0-3; ll+=4)
				{
				//v_dux   = _mm256_load_pd( &ptr_dux[ll] );
				u_temp0 = _mm_load_sd( &ptr_dux[ptr_idxb[ll+0]] );
				u_temp1 = _mm_load_sd( &ptr_dux[ptr_idxb[ll+2]] );
				u_temp0 = _mm_loadh_pd( u_temp0, &ptr_dux[ptr_idxb[ll+1]] );
				u_temp1 = _mm_loadh_pd( u_temp1, &ptr_dux[ptr_idxb[ll+3]] );
				v_dux   = _mm256_castpd128_pd256( u_temp0 );
				v_dux   = _mm256_insertf128_pd( v_dux, u_temp1, 0x1 );
				v_db0   = _mm256_load_pd( &ptr_db[0*pnb+ll] );
				v_db1   = _mm256_load_pd( &ptr_db[1*pnb+ll] );
				v_dt0   = _mm256_sub_pd ( v_dux, v_db0 );
				v_dt1   = _mm256_sub_pd ( v_db1, v_dux );
				v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
				v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
				v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
				v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
				_mm256_store_pd( &ptr_dt[0*pnb+ll], v_dt0 );
				_mm256_store_pd( &ptr_dt[1*pnb+ll], v_dt1 );

				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*pnb+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*pnb+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
				_mm256_store_pd( &ptr_dlam[0*pnb+ll], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[1*pnb+ll], v_dlam1 );

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
			if(ll<nb0)
				{

				ll_left = nb0 - ll;
				v_left  = _mm256_broadcast_sd( &ll_left );
				v_mask  = _mm256_loadu_pd( d_mask );
				v_mask  = _mm256_sub_pd( v_mask, v_left );
				i_mask  = _mm256_castpd_si256( v_mask );

				u_temp0 = _mm_load_sd( &ptr_dux[ptr_idxb[ll+0]] );
				if(ll_left>1) u_temp0 = _mm_loadh_pd( u_temp0, &ptr_dux[ptr_idxb[ll+1]] );
				if(ll_left>2) u_temp1 = _mm_load_sd( &ptr_dux[ptr_idxb[ll+2]] );
				//u_temp1 = _mm_loadh_pd( u_temp1, &ptr_dux[ptr_idxb[ll+3]] );
				v_dux   = _mm256_castpd128_pd256( u_temp0 );
				v_dux   = _mm256_insertf128_pd( v_dux, u_temp1, 0x1 );
				v_db0   = _mm256_load_pd( &ptr_db[0*pnb+ll] );
				v_db1   = _mm256_load_pd( &ptr_db[1*pnb+ll] );
				v_dt0   = _mm256_sub_pd ( v_dux, v_db0 );
				v_dt1   = _mm256_sub_pd ( v_db1, v_dux );
				v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
				v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
				v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
				v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
				_mm256_maskstore_pd( &ptr_dt[0*pnb+ll], i_mask, v_dt0 );
				_mm256_maskstore_pd( &ptr_dt[1*pnb+ll], i_mask, v_dt1 );

				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*pnb+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*pnb+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
				_mm256_maskstore_pd( &ptr_dlam[0*pnb+ll], i_mask, v_dlam0 );
				_mm256_maskstore_pd( &ptr_dlam[1*pnb+ll], i_mask, v_dlam1 );

				if(ll<nb0-2) // 3 left
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

			for(ll=0; ll<ng0-3; ll+=4)
				{
				v_dt0   = _mm256_load_pd( &ptr_dt[0*png+ll] );
				v_dt1   = _mm256_xor_pd( v_dt0, v_sign );
				v_db0   = _mm256_load_pd( &ptr_db[0*png+ll] );
				v_db1   = _mm256_load_pd( &ptr_db[1*png+ll] );
				v_dt0   = _mm256_sub_pd ( v_dt0, v_db0 );
				v_dt1   = _mm256_add_pd ( v_dt1, v_db1 );
				v_t0    = _mm256_load_pd( &ptr_t[0*png+ll] );
				v_t1    = _mm256_load_pd( &ptr_t[1*png+ll] );
				v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
				v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
				_mm256_store_pd( &ptr_dt[0*png+ll], v_dt0 );
				_mm256_store_pd( &ptr_dt[1*png+ll], v_dt1 );

				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*png+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*png+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[0*png+ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[1*png+ll] );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*png+ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*png+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
				_mm256_store_pd( &ptr_dlam[0*png+ll], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[1*png+ll], v_dlam1 );

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
			if(ll<ng0)
				{

				ll_left = ng0 - ll;
				v_left  = _mm256_broadcast_sd( &ll_left );
				v_mask  = _mm256_loadu_pd( d_mask );
				v_mask  = _mm256_sub_pd( v_mask, v_left );
				i_mask  = _mm256_castpd_si256( v_mask );

				v_dt0   = _mm256_load_pd( &ptr_dt[0*png+ll] );
				v_dt1   = _mm256_xor_pd( v_dt0, v_sign );
				v_db0   = _mm256_load_pd( &ptr_db[0*png+ll] );
				v_db1   = _mm256_load_pd( &ptr_db[1*png+ll] );
				v_dt0   = _mm256_sub_pd ( v_dt0, v_db0 );
				v_dt1   = _mm256_add_pd ( v_dt1, v_db1 );
				v_t0    = _mm256_load_pd( &ptr_t[0*png+ll] );
				v_t1    = _mm256_load_pd( &ptr_t[1*png+ll] );
				v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
				v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
				_mm256_maskstore_pd( &ptr_dt[0*png+ll], i_mask, v_dt0 );
				_mm256_maskstore_pd( &ptr_dt[1*png+ll], i_mask, v_dt1 );

				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*png+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*png+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[0*png+ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[1*png+ll] );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*png+ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*png+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
				_mm256_maskstore_pd( &ptr_dlam[0*png+ll], i_mask, v_dlam0 );
				_mm256_maskstore_pd( &ptr_dlam[1*png+ll], i_mask, v_dlam1 );

				if(ll<ng0-2) // 3 left
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



void d_update_var_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, double **ux, double **dux, double **t, double **dt, double **lam, double **dlam, double **pi, double **dpi)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line

	int nu0, nx0, nx1, nb0, pnb, ng0, png;

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

	v_alpha = _mm256_set_pd( alpha, alpha, alpha, alpha );

	v_zeros = _mm256_setzero_pd();
	v_mu0 = _mm256_setzero_pd();
	v_mu1 = _mm256_setzero_pd();

	double
		*ptr_pi, *ptr_dpi, *ptr_ux, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lam, *ptr_dlam;

	for(jj=0; jj<=N; jj++)
		{

		nx0 = nx[jj];
		nu0 = nu[jj];
		nb0 = nb[jj];
		pnb  = bs*((nb0+bs-1)/bs); // cache aligned number of box constraints
		ng0 = ng[jj];
		png  = bs*((ng0+bs-1)/bs); // cache aligned number of box constraints
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

		ll = 0;
		for(; ll<nx1-3; ll+=4)
			{
			v_pi  = _mm256_load_pd( &ptr_pi[ll] );
			v_dpi = _mm256_load_pd( &ptr_dpi[ll] );
			v_dpi = _mm256_sub_pd( v_dpi, v_pi );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_store_pd( &ptr_pi[ll], v_pi );
			}
		if(ll<nx1)
			{
			ll_left = nx1-ll;
			v_left= _mm256_broadcast_sd( &ll_left );
			v_mask= _mm256_loadu_pd( d_mask );
			i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_pi  = _mm256_load_pd( &ptr_pi[ll] );
			v_dpi = _mm256_load_pd( &ptr_dpi[ll] );
			v_dpi = _mm256_sub_pd( v_dpi, v_pi );
#if defined(TARGET_X64_AVX2)
			v_pi  = _mm256_fmadd_pd( v_alpha, v_dpi, v_pi );
#else
			v_dpi = _mm256_mul_pd( v_alpha, v_dpi );
			v_pi  = _mm256_add_pd( v_pi, v_dpi );
#endif
			_mm256_maskstore_pd( &ptr_pi[ll], i_mask, v_pi );
			}

		// update inputs & states
#if 0
		// box constraints
		ll = 0;
		for(; ll<nb0-3; ll+=4)
			{
			v_ux    = _mm256_load_pd( &ptr_ux[ll] );
			v_dux   = _mm256_load_pd( &ptr_dux[ll] );
			v_t0    = _mm256_load_pd( &ptr_t[ll] );
			v_t1    = _mm256_load_pd( &ptr_t[pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[pnb+ll] );
			v_dux   = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_dux   = _mm256_mul_pd( v_alpha, v_dux );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_ux    = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_store_pd( &ptr_t[ll], v_t0 );
			_mm256_store_pd( &ptr_t[pnb+ll], v_t1 );
			_mm256_store_pd( &ptr_lam[ll], v_lam0 );
			_mm256_store_pd( &ptr_lam[pnb+ll], v_lam1 );
			_mm256_store_pd( &ptr_ux[ll], v_ux );
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
		if(ll<nb0 && nb0==nx0+nu0)
			{
			ll_left = nb0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_ux    = _mm256_load_pd( &ptr_ux[ll] );
			v_dux   = _mm256_load_pd( &ptr_dux[ll] );
			v_t0    = _mm256_load_pd( &ptr_t[ll] );
			v_t1    = _mm256_load_pd( &ptr_t[pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[pnb+ll] );
			v_dux   = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_dux   = _mm256_mul_pd( v_alpha, v_dux );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_ux    = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_maskstore_pd( &ptr_t[ll], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_t[pnb+ll], i_mask, v_t1 );
			_mm256_maskstore_pd( &ptr_lam[ll], i_mask, v_lam0 );
			_mm256_maskstore_pd( &ptr_lam[pnb+ll], i_mask, v_lam1 );
			_mm256_maskstore_pd( &ptr_ux[ll], i_mask, v_ux );
#if defined(TARGET_X64_AVX2)
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
#endif
			}
		else
			{
			// backup ll
			ll_bkp = ll;
			// clean up inputs & states
			for(; ll<nu0+nx0-3; ll+=4)
				{
				v_ux  = _mm256_load_pd( &ptr_ux[ll] );
				v_dux = _mm256_load_pd( &ptr_dux[ll] );
				v_dux = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
				v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
				v_dux = _mm256_mul_pd( v_alpha, v_dux );
				v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
				_mm256_store_pd( &ptr_ux[ll], v_ux );
				}
			if(ll<nu0+nx0)
				{
				ll_left = nu0+nx0-ll;
				v_left= _mm256_broadcast_sd( &ll_left );
				v_mask= _mm256_loadu_pd( d_mask );
				i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

				v_ux  = _mm256_load_pd( &ptr_ux[ll] );
				v_dux = _mm256_load_pd( &ptr_dux[ll] );
				v_dux = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
				v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
				v_dux = _mm256_mul_pd( v_alpha, v_dux );
				v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
				_mm256_maskstore_pd( &ptr_ux[ll], i_mask, v_ux );
				}
			// cleanup box constraints
			ll = ll_bkp;
			for(; ll<nb0-3; ll+=4)
				{
				v_t0    = _mm256_load_pd( &ptr_t[ll] );
				v_t1    = _mm256_load_pd( &ptr_t[pnb+ll] );
				v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[pnb+ll] );
				v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
				v_dt1   = _mm256_load_pd( &ptr_dt[pnb+ll] );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[pnb+ll] );
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
				_mm256_store_pd( &ptr_t[ll], v_t0 );
				_mm256_store_pd( &ptr_t[pnb+ll], v_t1 );
				_mm256_store_pd( &ptr_lam[ll], v_lam0 );
				_mm256_store_pd( &ptr_lam[pnb+ll], v_lam1 );
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
			if(ll<nb0)
				{
				ll_left = nb0-ll;
				v_left  = _mm256_broadcast_sd( &ll_left );
				v_mask  = _mm256_loadu_pd( d_mask );
				v_mask  = _mm256_sub_pd( v_mask, v_left );
				i_mask  = _mm256_castpd_si256( v_mask );

				v_t0    = _mm256_load_pd( &ptr_t[ll] );
				v_t1    = _mm256_load_pd( &ptr_t[pnb+ll] );
				v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[pnb+ll] );
				v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
				v_dt1   = _mm256_load_pd( &ptr_dt[pnb+ll] );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[pnb+ll] );
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
				_mm256_maskstore_pd( &ptr_t[ll], i_mask, v_t0 );
				_mm256_maskstore_pd( &ptr_t[pnb+ll], i_mask, v_t1 );
				_mm256_maskstore_pd( &ptr_lam[ll], i_mask, v_lam0 );
				_mm256_maskstore_pd( &ptr_lam[pnb+ll], i_mask, v_lam1 );
#if defined(TARGET_X64_AVX2)
				v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
				v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
				v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
				v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#else
				v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
				v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
				v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
				v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
				v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
				v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
#endif
				}
			}

#else

		// inputs and states
		for(ll=0; ll<nu0+nx0-3; ll+=4)
			{
			v_ux  = _mm256_load_pd( &ptr_ux[ll] );
			v_dux = _mm256_load_pd( &ptr_dux[ll] );
			v_dux = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_store_pd( &ptr_ux[ll], v_ux );
			}
		if(ll<nu0+nx0)
			{
			ll_left = nu0+nx0-ll;
			v_left= _mm256_broadcast_sd( &ll_left );
			v_mask= _mm256_loadu_pd( d_mask );
			i_mask= _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

			v_ux  = _mm256_load_pd( &ptr_ux[ll] );
			v_dux = _mm256_load_pd( &ptr_dux[ll] );
			v_dux = _mm256_sub_pd( v_dux, v_ux );
#if defined(TARGET_X64_AVX2)
			v_ux    = _mm256_fmadd_pd( v_alpha, v_dux, v_ux );
#else
			v_dux = _mm256_mul_pd( v_alpha, v_dux );
			v_ux  = _mm256_add_pd( v_ux, v_dux );
#endif
			_mm256_maskstore_pd( &ptr_ux[ll], i_mask, v_ux );
			}

		// box constraints
		for(ll=0; ll<nb0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pnb+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
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
			_mm256_store_pd( &ptr_t[0*pnb+ll], v_t0 );
			_mm256_store_pd( &ptr_t[1*pnb+ll], v_t1 );
			_mm256_store_pd( &ptr_lam[0*pnb+ll], v_lam0 );
			_mm256_store_pd( &ptr_lam[1*pnb+ll], v_lam1 );
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
		if(ll<nb0)
			{
			ll_left = nb0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pnb+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
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
			_mm256_maskstore_pd( &ptr_t[0*pnb+ll], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_t[1*pnb+ll], i_mask, v_t1 );
			_mm256_maskstore_pd( &ptr_lam[0*pnb+ll], i_mask, v_lam0 );
			_mm256_maskstore_pd( &ptr_lam[1*pnb+ll], i_mask, v_lam1 );
#if defined(TARGET_X64_AVX2)
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
#endif
			}

#endif

		ptr_t    += 2*pnb;
		ptr_dt   += 2*pnb;
		ptr_lam  += 2*pnb;
		ptr_dlam += 2*pnb;

		// genreal constraints
		for(ll=0; ll<ng0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[0*png+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*png+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*png+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*png+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*png+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*png+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*png+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*png+ll] );
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
			_mm256_store_pd( &ptr_t[0*png+ll], v_t0 );
			_mm256_store_pd( &ptr_t[1*png+ll], v_t1 );
			_mm256_store_pd( &ptr_lam[0*png+ll], v_lam0 );
			_mm256_store_pd( &ptr_lam[1*png+ll], v_lam1 );
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
		if(ll<ng0)
			{

			ll_left = ng0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_t0    = _mm256_load_pd( &ptr_t[0*png+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*png+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*png+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*png+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*png+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*png+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*png+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*png+ll] );
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
			_mm256_maskstore_pd( &ptr_t[0*png+ll], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_t[1*png+ll], i_mask, v_t1 );
			_mm256_maskstore_pd( &ptr_lam[0*png+ll], i_mask, v_lam0 );
			_mm256_maskstore_pd( &ptr_lam[1*png+ll], i_mask, v_lam1 );
#if defined(TARGET_X64_AVX2)
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
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



void d_compute_mu_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double *ptr_mu, double mu_scal, double alpha, double **lam, double **dlam, double **t, double **dt)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line

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

	int nb0, pnb, ng0, png;

	v_alpha = _mm256_set_pd( alpha, alpha, alpha, alpha );

	v_zeros = _mm256_setzero_pd();
	v_mu0 = _mm256_setzero_pd();
	v_mu1 = _mm256_setzero_pd();

	for(jj=0; jj<=N; jj++)
		{

		ptr_t    = t[jj];
		ptr_lam  = lam[jj];
		ptr_dt   = dt[jj];
		ptr_dlam = dlam[jj];

		// box constraints
		nb0 = nb[jj];
		pnb = (nb0+bs-1)/bs*bs;
		for(ll=0; ll<nb0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pnb+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
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
		if(ll<nb0)
			{
			ll_left = nb0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );

			v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pnb+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
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
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
#endif
			}

		ptr_t    += 2*pnb;
		ptr_lam  += 2*pnb;
		ptr_dt   += 2*pnb;
		ptr_dlam += 2*pnb;

		// general constraints
		ng0 = ng[jj];
		png = (ng0+bs-1)/bs*bs;
		for(ll=0; ll<ng0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[ll] );
			v_t1    = _mm256_load_pd( &ptr_t[png+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[png+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[png+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[png+ll] );
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
		if(ll<ng0)
			{
			ll_left = ng0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );

			v_t0    = _mm256_load_pd( &ptr_t[ll] );
			v_t1    = _mm256_load_pd( &ptr_t[png+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[png+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[png+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[png+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
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
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
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



void d_update_gradient_new_rhs_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double **db, double **t_inv, double **lamt, double **qx)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nb0, pnb, ng0, png;

	double temp0, temp1;

	double
		*ptr_db, *ptr_qx,
		*ptr_t_inv, *ptr_lamt;

	int ii, jj, bs0;

	for(jj=0; jj<=N; jj++)
		{

		ptr_t_inv = t_inv[jj];
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

#if 0
				ptr_qx[ii+0] = ptr_lamt[ii+pnb+0]/ptr_t_inv[ii+pnb+0] - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] - ptr_lamt[ii+0]/ptr_t_inv[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0];

				ptr_qx[ii+1] = ptr_lamt[ii+pnb+1]/ptr_t_inv[ii+pnb+1] - ptr_lamt[ii+pnb+1]*ptr_db[ii+pnb+1] - ptr_lamt[ii+1]/ptr_t_inv[ii+1] - ptr_lamt[ii+1]*ptr_db[ii+1];

				ptr_qx[ii+2] = ptr_lamt[ii+pnb+2]/ptr_t_inv[ii+pnb+2] - ptr_lamt[ii+pnb+2]*ptr_db[ii+pnb+2] - ptr_lamt[ii+2]/ptr_t_inv[ii+2] - ptr_lamt[ii+2]*ptr_db[ii+2];

				ptr_qx[ii+3] = ptr_lamt[ii+pnb+3]/ptr_t_inv[ii+pnb+3] - ptr_lamt[ii+pnb+3]*ptr_db[ii+pnb+3] - ptr_lamt[ii+3]/ptr_t_inv[ii+3] - ptr_lamt[ii+3]*ptr_db[ii+3];
#else
				ptr_qx[ii+0] = - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] - ptr_lamt[ii+0]*ptr_db[ii+0];

				ptr_qx[ii+1] = - ptr_lamt[ii+pnb+1]*ptr_db[ii+pnb+1] - ptr_lamt[ii+1]*ptr_db[ii+1];

				ptr_qx[ii+2] = - ptr_lamt[ii+pnb+2]*ptr_db[ii+pnb+2] - ptr_lamt[ii+2]*ptr_db[ii+2];

				ptr_qx[ii+3] = - ptr_lamt[ii+pnb+3]*ptr_db[ii+pnb+3] - ptr_lamt[ii+3]*ptr_db[ii+3];
#endif

				}
			for(; ii<nb0; ii++)
				{

#if 0
				ptr_qx[ii+0] = ptr_lamt[ii+pnb+0]/ptr_t_inv[ii+pnb+0] - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] - ptr_lamt[ii+0]/ptr_t_inv[ii+0] - ptr_lamt[ii+0]*ptr_db[ii+0];
#else
				ptr_qx[ii+0] = - ptr_lamt[ii+pnb+0]*ptr_db[ii+pnb+0] - ptr_lamt[ii+0]*ptr_db[ii+0];
#endif

				}

			ptr_t_inv += 2*pnb;
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

	int nb0, pnb, ng0, png;

	double
		*ptr_res_d, *ptr_Qx, *ptr_qx, *ptr_t, *ptr_lam, *ptr_res_m, *ptr_t_inv;

	__m256d
		v_ones,
		v_tmp0, v_tinv0, v_lam0, v_resm0, v_resd0,
		v_tmp1, v_tinv1, v_lam1, v_resm1, v_resd1;

	__m256i
		i_mask;

	v_ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	int ii, jj, bs0;

	double ii_left;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

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

				v_tinv0 = _mm256_load_pd( &ptr_t[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t[ii+pnb] );
				v_tinv0 = _mm256_div_pd( v_ones, v_tinv0 );
				v_tinv1 = _mm256_div_pd( v_ones, v_tinv1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+pnb] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+pnb] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+pnb] );
				v_tmp0  = _mm256_mul_pd( v_tinv0, v_lam0 );
				v_tmp1  = _mm256_mul_pd( v_tinv1, v_lam1 );
				_mm256_store_pd( &ptr_t_inv[ii+0], v_tinv0 );
				_mm256_store_pd( &ptr_t_inv[ii+pnb], v_tinv1 );
				v_tmp0  = _mm256_add_pd( v_tmp0, v_tmp1 );
				_mm256_store_pd( &ptr_Qx[ii+0], v_tmp0 );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
				v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
				v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
				v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
				v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
				_mm256_store_pd( &ptr_qx[ii+0], v_tmp0 );

				}
			if(ii<nb0)
				{

				ii_left = nb0-ii;
				i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

				v_tinv0 = _mm256_load_pd( &ptr_t[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t[ii+pnb] );
				v_tinv0 = _mm256_div_pd( v_ones, v_tinv0 );
				v_tinv1 = _mm256_div_pd( v_ones, v_tinv1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+pnb] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+pnb] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+pnb] );
				v_tmp0  = _mm256_mul_pd( v_tinv0, v_lam0 );
				v_tmp1  = _mm256_mul_pd( v_tinv1, v_lam1 );
				_mm256_maskstore_pd( &ptr_t_inv[ii+0], i_mask, v_tinv0 );
				_mm256_maskstore_pd( &ptr_t_inv[ii+pnb], i_mask, v_tinv1 );
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

				v_tinv0 = _mm256_load_pd( &ptr_t[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t[ii+png] );
				v_tinv0 = _mm256_div_pd( v_ones, v_tinv0 );
				v_tinv1 = _mm256_div_pd( v_ones, v_tinv1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+png] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+png] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+png] );
				v_tmp0  = _mm256_mul_pd( v_tinv0, v_lam0 );
				v_tmp1  = _mm256_mul_pd( v_tinv1, v_lam1 );
				_mm256_store_pd( &ptr_t_inv[ii+0], v_tinv0 );
				_mm256_store_pd( &ptr_t_inv[ii+png], v_tinv1 );
				v_tmp0  = _mm256_add_pd( v_tmp0, v_tmp1 );
				_mm256_store_pd( &ptr_Qx[ii+0], v_tmp0 );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
				v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
				v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
				v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
				v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
				_mm256_store_pd( &ptr_qx[ii+0], v_tmp0 );

				}
			if(ii<ng0)
				{

				ii_left = ng0-ii;
				i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

				v_tinv0 = _mm256_load_pd( &ptr_t[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t[ii+png] );
				v_tinv0 = _mm256_div_pd( v_ones, v_tinv0 );
				v_tinv1 = _mm256_div_pd( v_ones, v_tinv1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+png] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+png] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+png] );
				v_tmp0  = _mm256_mul_pd( v_tinv0, v_lam0 );
				v_tmp1  = _mm256_mul_pd( v_tinv1, v_lam1 );
				_mm256_maskstore_pd( &ptr_t_inv[ii+0], i_mask, v_tinv0 );
				_mm256_maskstore_pd( &ptr_t_inv[ii+png], i_mask, v_tinv1 );
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

		}

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
			ll = 0;
#if 1
			for(; ll<nb0-3; ll+=4)
				{

				u_tmp0  = _mm_load_sd( &ptr_dux[ptr_idxb[ll+0]] );
				u_tmp1  = _mm_load_sd( &ptr_dux[ptr_idxb[ll+2]] );
				u_tmp0  = _mm_loadh_pd( u_tmp0, &ptr_dux[ptr_idxb[ll+1]] );
				u_tmp1  = _mm_loadh_pd( u_tmp1, &ptr_dux[ptr_idxb[ll+3]] );
				v_dux   = _mm256_castpd128_pd256( u_tmp0 );
				v_dux   = _mm256_insertf128_pd( v_dux, u_tmp1, 0x1 );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ll+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ll+pnb] );
				v_dt0   = _mm256_sub_pd( v_dux, v_resd0 );
				v_dt1   = _mm256_sub_pd( v_resd1, v_dux );
				_mm256_store_pd( &ptr_dt[ll+0], v_dt0 );
				_mm256_store_pd( &ptr_dt[ll+pnb], v_dt1 );

				v_lam0  = _mm256_load_pd( &ptr_lam[ll+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ll+pnb] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ll+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ll+pnb] );
				v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ll+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ll+pnb] );
				v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
				v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
				v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
				v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
				_mm256_store_pd( &ptr_dlam[ll+0], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[ll+pnb], v_dlam1 );

				t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
				t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
				t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
				t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
				v_t0  = _mm256_load_pd( &ptr_t[ll+0] );
				v_t1  = _mm256_load_pd( &ptr_t[ll+pnb] );
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
			if(ll<nb0)
				{

				if(nb0-ll==1)
					{

					u_dux    = _mm_load_sd( &ptr_dux[ptr_idxb[ll+0]] );
					u_resd0  = _mm_load_sd( &ptr_res_d[ll+0] );
					u_resd1  = _mm_load_sd( &ptr_res_d[ll+pnb] );
					u_dt0    = _mm_sub_pd( u_dux, u_resd0 );
					u_dt1    = _mm_sub_pd( u_resd1, u_dux );
					_mm_store_sd( &ptr_dt[ll+0], u_dt0 );
					_mm_store_sd( &ptr_dt[ll+pnb], u_dt1 );

					u_lam0   = _mm_load_sd( &ptr_lam[ll+0] );
					u_lam1   = _mm_load_sd( &ptr_lam[ll+pnb] );
					u_tmp0   = _mm_mul_pd( u_lam0, u_dt0 );
					u_tmp1   = _mm_mul_pd( u_lam1, u_dt1 );
					u_resm0  = _mm_load_sd( &ptr_res_m[ll+0] );
					u_resm1  = _mm_load_sd( &ptr_res_m[ll+pnb] );
					u_tmp0   = _mm_add_pd( u_tmp0, u_resm0 );
					u_tmp1   = _mm_add_pd( u_tmp1, u_resm1 );
					u_tinv0  = _mm_load_sd( &ptr_t_inv[ll+0] );
					u_tinv1  = _mm_load_sd( &ptr_t_inv[ll+pnb] );
					u_tinv0  = _mm_xor_pd( u_tinv0, _mm256_castpd256_pd128( v_sign ) );
					u_tinv1  = _mm_xor_pd( u_tinv1, _mm256_castpd256_pd128( v_sign ) );
					u_dlam0  = _mm_mul_pd( u_tinv0, u_tmp0 );
					u_dlam1  = _mm_mul_pd( u_tinv1, u_tmp1 );
					_mm_store_sd( &ptr_dlam[ll+0], u_dlam0 );
					_mm_store_sd( &ptr_dlam[ll+pnb], u_dlam1 );

					u_dt1    = _mm_movedup_pd( u_dt1 );
					u_dt0    = _mm_move_sd( u_dt1, u_dt0 );
					u_t1     = _mm_loaddup_pd( &ptr_t[ll+pnb] );
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
				else if(nb0-ll==2)
					{

					u_dux    = _mm_load_sd( &ptr_dux[ptr_idxb[ll+0]] );
					u_dux    = _mm_loadh_pd( u_dux, &ptr_dux[ptr_idxb[ll+1]] );
					u_resd0  = _mm_load_pd( &ptr_res_d[ll+0] );
					u_resd1  = _mm_load_pd( &ptr_res_d[ll+pnb] );
					u_dt0    = _mm_sub_pd( u_dux, u_resd0 );
					u_dt1    = _mm_sub_pd( u_resd1, u_dux );
					_mm_store_pd( &ptr_dt[ll+0], u_dt0 );
					_mm_store_pd( &ptr_dt[ll+pnb], u_dt1 );

					u_lam0   = _mm_load_pd( &ptr_lam[ll+0] );
					u_lam1   = _mm_load_pd( &ptr_lam[ll+pnb] );
					u_tmp0   = _mm_mul_pd( u_lam0, u_dt0 );
					u_tmp1   = _mm_mul_pd( u_lam1, u_dt1 );
					u_resm0  = _mm_load_pd( &ptr_res_m[ll+0] );
					u_resm1  = _mm_load_pd( &ptr_res_m[ll+pnb] );
					u_tmp0   = _mm_add_pd( u_tmp0, u_resm0 );
					u_tmp1   = _mm_add_pd( u_tmp1, u_resm1 );
					u_tinv0  = _mm_load_pd( &ptr_t_inv[ll+0] );
					u_tinv1  = _mm_load_pd( &ptr_t_inv[ll+pnb] );
					u_tinv0  = _mm_xor_pd( u_tinv0, _mm256_castpd256_pd128( v_sign ) );
					u_tinv1  = _mm_xor_pd( u_tinv1, _mm256_castpd256_pd128( v_sign ) );
					u_dlam0  = _mm_mul_pd( u_tinv0, u_tmp0 );
					u_dlam1  = _mm_mul_pd( u_tinv1, u_tmp1 );
					_mm_store_pd( &ptr_dlam[ll+0], u_dlam0 );
					_mm_store_pd( &ptr_dlam[ll+pnb], u_dlam1 );

					v_dt0    = _mm256_castpd128_pd256( u_dt0 );
					v_dt0    = _mm256_insertf128_pd( v_dt0, u_dt1, 0x1 );
					v_t0     = _mm256_castpd128_pd256( _mm_load_pd( &ptr_t[ll+0] ) );
					v_t0     = _mm256_insertf128_pd( v_t0, _mm_load_pd( &ptr_t[ll+pnb]), 0x1 );
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
				else // if(nb-ll==3)
					{

					i_mask = _mm256_castpd_si256( _mm256_set_pd( 1.0, -1.0, -1.0, -1.0 ) );

					u_tmp0  = _mm_load_sd( &ptr_dux[ptr_idxb[ll+0]] );
					u_tmp1  = _mm_load_sd( &ptr_dux[ptr_idxb[ll+2]] );
					u_tmp0  = _mm_loadh_pd( u_tmp0, &ptr_dux[ptr_idxb[ll+1]] );
					v_dux   = _mm256_castpd128_pd256( u_tmp0 );
					v_dux   = _mm256_insertf128_pd( v_dux, u_tmp1, 0x1 );
					v_resd0 = _mm256_load_pd( &ptr_res_d[ll+0] );
					v_resd1 = _mm256_load_pd( &ptr_res_d[ll+pnb] );
					v_dt0   = _mm256_sub_pd( v_dux, v_resd0 );
					v_dt1   = _mm256_sub_pd( v_resd1, v_dux );
					_mm256_maskstore_pd( &ptr_dt[ll+0], i_mask, v_dt0 );
					_mm256_maskstore_pd( &ptr_dt[ll+pnb], i_mask, v_dt1 );

					v_lam0  = _mm256_load_pd( &ptr_lam[ll+0] );
					v_lam1  = _mm256_load_pd( &ptr_lam[ll+pnb] );
					v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
					v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
					v_resm0 = _mm256_load_pd( &ptr_res_m[ll+0] );
					v_resm1 = _mm256_load_pd( &ptr_res_m[ll+pnb] );
					v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
					v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
					v_tinv0 = _mm256_load_pd( &ptr_t_inv[ll+0] );
					v_tinv1 = _mm256_load_pd( &ptr_t_inv[ll+pnb] );
					v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
					v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
					v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
					v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
					_mm256_maskstore_pd( &ptr_dlam[ll+0], i_mask, v_dlam0 );
					_mm256_maskstore_pd( &ptr_dlam[ll+pnb], i_mask, v_dlam1 );

					t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
					t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
					t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
					t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
					v_t0  = _mm256_load_pd( &ptr_t[ll+0] );
					v_t1  = _mm256_load_pd( &ptr_t[ll+pnb] );
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

#else
			for(; ll<nb0; ll++)
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
#endif

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

			ll = 0;
#if 1
			for(; ll<ng0-3; ll+=4)
				{

				v_tmp0  = _mm256_load_pd( &ptr_dt[ll+0] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ll+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ll+png] );
				v_dt0   = _mm256_sub_pd( v_tmp0, v_resd0 );
				v_dt1   = _mm256_sub_pd( v_resd1, v_tmp0 );
				_mm256_store_pd( &ptr_dt[ll+0], v_dt0 );
				_mm256_store_pd( &ptr_dt[ll+png], v_dt1 );

				v_lam0  = _mm256_load_pd( &ptr_lam[ll+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ll+png] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ll+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ll+png] );
				v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ll+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ll+png] );
				v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
				v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
				v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
				v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
				_mm256_store_pd( &ptr_dlam[ll+0], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[ll+png], v_dlam1 );

				t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
				t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
				t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
				t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
				v_t0  = _mm256_load_pd( &ptr_t[ll+0] );
				v_t1  = _mm256_load_pd( &ptr_t[ll+png] );
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
			if(ll<ng0)
				{

				if(ng0-ll==1)
					{

					u_tmp0  = _mm_load_pd( &ptr_dt[ll+0] );
					u_resd0 = _mm_load_pd( &ptr_res_d[ll+0] );
					u_resd1 = _mm_load_pd( &ptr_res_d[ll+png] );
					u_dt0   = _mm_sub_pd( u_tmp0, u_resd0 );
					u_dt1   = _mm_sub_pd( u_resd1, u_tmp0 );
					_mm_store_sd( &ptr_dt[ll+0], u_dt0 );
					_mm_store_sd( &ptr_dt[ll+png], u_dt1 );

					u_lam0   = _mm_load_sd( &ptr_lam[ll+0] );
					u_lam1   = _mm_load_sd( &ptr_lam[ll+png] );
					u_tmp0   = _mm_mul_pd( u_lam0, u_dt0 );
					u_tmp1   = _mm_mul_pd( u_lam1, u_dt1 );
					u_resm0  = _mm_load_sd( &ptr_res_m[ll+0] );
					u_resm1  = _mm_load_sd( &ptr_res_m[ll+png] );
					u_tmp0   = _mm_add_pd( u_tmp0, u_resm0 );
					u_tmp1   = _mm_add_pd( u_tmp1, u_resm1 );
					u_tinv0  = _mm_load_sd( &ptr_t_inv[ll+0] );
					u_tinv1  = _mm_load_sd( &ptr_t_inv[ll+png] );
					u_tinv0  = _mm_xor_pd( u_tinv0, _mm256_castpd256_pd128( v_sign ) );
					u_tinv1  = _mm_xor_pd( u_tinv1, _mm256_castpd256_pd128( v_sign ) );
					u_dlam0  = _mm_mul_pd( u_tinv0, u_tmp0 );
					u_dlam1  = _mm_mul_pd( u_tinv1, u_tmp1 );
					_mm_store_sd( &ptr_dlam[ll+0], u_dlam0 );
					_mm_store_sd( &ptr_dlam[ll+png], u_dlam1 );

					u_dt1    = _mm_movedup_pd( u_dt1 );
					u_dt0    = _mm_move_sd( u_dt1, u_dt0 );
					u_t1     = _mm_loaddup_pd( &ptr_t[ll+png] );
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
				else if(ng0-ll==2)
					{

					u_tmp0  = _mm_load_pd( &ptr_dt[ll+0] );
					u_resd0 = _mm_load_pd( &ptr_res_d[ll+0] );
					u_resd1 = _mm_load_pd( &ptr_res_d[ll+png] );
					u_dt0   = _mm_sub_pd( u_tmp0, u_resd0 );
					u_dt1   = _mm_sub_pd( u_resd1, u_tmp0 );
					_mm_store_pd( &ptr_dt[ll+0], u_dt0 );
					_mm_store_pd( &ptr_dt[ll+png], u_dt1 );

					u_lam0   = _mm_load_pd( &ptr_lam[ll+0] );
					u_lam1   = _mm_load_pd( &ptr_lam[ll+png] );
					u_tmp0   = _mm_mul_pd( u_lam0, u_dt0 );
					u_tmp1   = _mm_mul_pd( u_lam1, u_dt1 );
					u_resm0  = _mm_load_pd( &ptr_res_m[ll+0] );
					u_resm1  = _mm_load_pd( &ptr_res_m[ll+png] );
					u_tmp0   = _mm_add_pd( u_tmp0, u_resm0 );
					u_tmp1   = _mm_add_pd( u_tmp1, u_resm1 );
					u_tinv0  = _mm_load_pd( &ptr_t_inv[ll+0] );
					u_tinv1  = _mm_load_pd( &ptr_t_inv[ll+png] );
					u_tinv0  = _mm_xor_pd( u_tinv0, _mm256_castpd256_pd128( v_sign ) );
					u_tinv1  = _mm_xor_pd( u_tinv1, _mm256_castpd256_pd128( v_sign ) );
					u_dlam0  = _mm_mul_pd( u_tinv0, u_tmp0 );
					u_dlam1  = _mm_mul_pd( u_tinv1, u_tmp1 );
					_mm_store_pd( &ptr_dlam[ll+0], u_dlam0 );
					_mm_store_pd( &ptr_dlam[ll+png], u_dlam1 );

					v_dt0    = _mm256_castpd128_pd256( u_dt0 );
					v_dt0    = _mm256_insertf128_pd( v_dt0, u_dt1, 0x1 );
					v_t0     = _mm256_castpd128_pd256( _mm_load_pd( &ptr_t[ll+0] ) );
					v_t0     = _mm256_insertf128_pd( v_t0, _mm_load_pd( &ptr_t[ll+png]), 0x1 );
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

					v_tmp0  = _mm256_load_pd( &ptr_dt[ll+0] );
					v_resd0 = _mm256_load_pd( &ptr_res_d[ll+0] );
					v_resd1 = _mm256_load_pd( &ptr_res_d[ll+png] );
					v_dt0   = _mm256_sub_pd( v_tmp0, v_resd0 );
					v_dt1   = _mm256_sub_pd( v_resd1, v_tmp0 );
					_mm256_maskstore_pd( &ptr_dt[ll+0], i_mask, v_dt0 );
					_mm256_maskstore_pd( &ptr_dt[ll+png], i_mask, v_dt1 );

					v_lam0  = _mm256_load_pd( &ptr_lam[ll+0] );
					v_lam1  = _mm256_load_pd( &ptr_lam[ll+png] );
					v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
					v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
					v_resm0 = _mm256_load_pd( &ptr_res_m[ll+0] );
					v_resm1 = _mm256_load_pd( &ptr_res_m[ll+png] );
					v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
					v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
					v_tinv0 = _mm256_load_pd( &ptr_t_inv[ll+0] );
					v_tinv1 = _mm256_load_pd( &ptr_t_inv[ll+png] );
					v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
					v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
					v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
					v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
					_mm256_maskstore_pd( &ptr_dlam[ll+0], i_mask, v_dlam0 );
					_mm256_maskstore_pd( &ptr_dlam[ll+png], i_mask, v_dlam1 );

					t_dlam   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
					t_dt     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
					t_mask0  = _mm256_cmp_ps( t_dlam, t_zeros, 0x01 );
					t_mask1  = _mm256_cmp_ps( t_dt, t_zeros, 0x01 );
					v_t0  = _mm256_load_pd( &ptr_t[ll+0] );
					v_t1  = _mm256_load_pd( &ptr_t[ll+png] );
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

#else
			for(; ll<ng0; ll++)
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
#endif

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

	__m128d
		u_tmp0,
		u_tmp1;

	__m256d
		v_dux, v_sign,
		v_resm0, v_resd0, v_dt0, v_dlam0, v_tmp0, v_tinv0, v_lam0, v_t0,
		v_resm1, v_resd1, v_dt1, v_dlam1, v_tmp1, v_tinv1, v_lam1, v_t1;

	long long long_sign = 0x8000000000000000;
	v_sign = _mm256_broadcast_sd( (double *) &long_sign );

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
			ll = 0;
			for(; ll<nb0-3; ll+=4)
				{

				u_tmp0  = _mm_load_sd( &ptr_dux[ptr_idxb[ll+0]] );
				u_tmp1  = _mm_load_sd( &ptr_dux[ptr_idxb[ll+2]] );
				u_tmp0  = _mm_loadh_pd( u_tmp0, &ptr_dux[ptr_idxb[ll+1]] );
				u_tmp1  = _mm_loadh_pd( u_tmp1, &ptr_dux[ptr_idxb[ll+3]] );
				v_dux   = _mm256_castpd128_pd256( u_tmp0 );
				v_dux   = _mm256_insertf128_pd( v_dux, u_tmp1, 0x1 );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ll+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ll+pnb] );
				v_dt0   = _mm256_sub_pd( v_dux, v_resd0 );
				v_dt1   = _mm256_sub_pd( v_resd1, v_dux );
				_mm256_store_pd( &ptr_dt[ll+0], v_dt0 );
				_mm256_store_pd( &ptr_dt[ll+pnb], v_dt1 );

				v_lam0  = _mm256_load_pd( &ptr_lam[ll+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ll+pnb] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ll+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ll+pnb] );
				v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ll+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ll+pnb] );
				v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
				v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
				v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
				v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
				_mm256_store_pd( &ptr_dlam[ll+0], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[ll+pnb], v_dlam1 );

				}
			for(; ll<nb0; ll++)
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

			ll = 0;
			for(; ll<ng0-3; ll+=4)
				{

				v_tmp0  = _mm256_load_pd( &ptr_dt[ll+0] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ll+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ll+png] );
				v_dt0   = _mm256_sub_pd( v_tmp0, v_resd0 );
				v_dt1   = _mm256_sub_pd( v_resd1, v_tmp0 );
				_mm256_store_pd( &ptr_dt[ll+0], v_dt0 );
				_mm256_store_pd( &ptr_dt[ll+png], v_dt1 );

				v_lam0  = _mm256_load_pd( &ptr_lam[ll+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ll+png] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_dt0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_dt1 );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ll+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ll+png] );
				v_tmp0  = _mm256_add_pd( v_tmp0, v_resm0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_resm1 );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ll+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ll+png] );
				v_tinv0 = _mm256_xor_pd( v_tinv0, v_sign );
				v_tinv1 = _mm256_xor_pd( v_tinv1, v_sign );
				v_dlam0  = _mm256_mul_pd( v_tinv0, v_tmp0 );
				v_dlam1  = _mm256_mul_pd( v_tinv1, v_tmp1 );
				_mm256_store_pd( &ptr_dlam[ll+0], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[ll+png], v_dlam1 );

				}
			for(; ll<ng0; ll++)
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

	int nb0, pnb, ng0, png;

	v_alpha = _mm256_set_pd( alpha, alpha, alpha, alpha );

	v_zeros = _mm256_setzero_pd();
	v_mu0 = _mm256_setzero_pd();
	v_mu1 = _mm256_setzero_pd();

	for(jj=0; jj<=N; jj++)
		{

		ptr_t    = t[jj];
		ptr_lam  = lam[jj];
		ptr_dt   = dt[jj];
		ptr_dlam = dlam[jj];

		// box constraints
		nb0 = nb[jj];
		pnb = (nb0+bs-1)/bs*bs;
		for(ll=0; ll<nb0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pnb+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#endif
#if defined(TARGET_X64_AVX)
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
		if(ll<nb0)
			{
			ll_left = nb0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );

			v_t0    = _mm256_load_pd( &ptr_t[0*pnb+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pnb+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pnb+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pnb+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pnb+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pnb+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pnb+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pnb+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#endif
#if defined(TARGET_X64_AVX)
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
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
#endif
			}

		ptr_t    += 2*pnb;
		ptr_lam  += 2*pnb;
		ptr_dt   += 2*pnb;
		ptr_dlam += 2*pnb;

		// general constraints
		ng0 = ng[jj];
		png = (ng0+bs-1)/bs*bs;
		for(ll=0; ll<ng0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[ll] );
			v_t1    = _mm256_load_pd( &ptr_t[png+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[png+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[png+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[png+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#endif
#if defined(TARGET_X64_AVX)
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
		if(ll<ng0)
			{
			ll_left = ng0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );

			v_t0    = _mm256_load_pd( &ptr_t[ll] );
			v_t1    = _mm256_load_pd( &ptr_t[png+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[png+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[png+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[png+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
#endif
#if defined(TARGET_X64_AVX)
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
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
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



void d_compute_centering_correction_res_mpc_hard_tv(int N, int *nb, int *ng, double sigma_mu, double **dt, double **dlam, double **res_m)
	{

	const int bs = D_MR;

	int pnb, png;

	int ii, jj;

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

		pnb = (nb[ii]+bs-1)/bs*bs;
		png = (ng[ii]+bs-1)/bs*bs;

		ptr_res_m = res_m[ii];
		ptr_dt    = dt[ii];
		ptr_dlam  = dlam[ii];

		for(jj=0; jj<nb[ii]-3; jj+=4)
			{
			v_dt0   = _mm256_load_pd( &ptr_dt[jj+0] );
			v_dt1   = _mm256_load_pd( &ptr_dt[jj+pnb] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[jj+0] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[jj+pnb] );
			v_tmp0  = _mm256_mul_pd( v_dt0, v_dlam0 );
			v_tmp1  = _mm256_mul_pd( v_dt1, v_dlam1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_sigma_mu );
			v_tmp1  = _mm256_sub_pd( v_tmp1, v_sigma_mu );
			v_resm0 = _mm256_load_pd( &ptr_res_m[jj+0] );
			v_resm1 = _mm256_load_pd( &ptr_res_m[jj+pnb] );
			v_resm0 = _mm256_add_pd( v_resm0, v_tmp0 );
			v_resm1 = _mm256_add_pd( v_resm1, v_tmp1 );
			_mm256_store_pd( &ptr_res_m[jj+0], v_resm0 );
			_mm256_store_pd( &ptr_res_m[jj+pnb], v_resm1 );
			}
		if(jj<nb[ii])
			{
			ii_left = nb[ii]-jj;
			i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

			v_dt0   = _mm256_load_pd( &ptr_dt[jj+0] );
			v_dt1   = _mm256_load_pd( &ptr_dt[jj+pnb] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[jj+0] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[jj+pnb] );
			v_tmp0  = _mm256_mul_pd( v_dt0, v_dlam0 );
			v_tmp1  = _mm256_mul_pd( v_dt1, v_dlam1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_sigma_mu );
			v_tmp1  = _mm256_sub_pd( v_tmp1, v_sigma_mu );
			v_resm0 = _mm256_load_pd( &ptr_res_m[jj+0] );
			v_resm1 = _mm256_load_pd( &ptr_res_m[jj+pnb] );
			v_resm0 = _mm256_add_pd( v_resm0, v_tmp0 );
			v_resm1 = _mm256_add_pd( v_resm1, v_tmp1 );
			_mm256_maskstore_pd( &ptr_res_m[jj+0], i_mask, v_resm0 );
			_mm256_maskstore_pd( &ptr_res_m[jj+pnb], i_mask, v_resm1 );
			}

		ptr_res_m += 2*pnb;
		ptr_dt    += 2*pnb;
		ptr_dlam  += 2*pnb;

		for(jj=0; jj<ng[ii]-3; jj+=4)
			{
			v_dt0   = _mm256_load_pd( &ptr_dt[jj+0] );
			v_dt1   = _mm256_load_pd( &ptr_dt[jj+png] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[jj+0] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[jj+png] );
			v_tmp0  = _mm256_mul_pd( v_dt0, v_dlam0 );
			v_tmp1  = _mm256_mul_pd( v_dt1, v_dlam1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_sigma_mu );
			v_tmp1  = _mm256_sub_pd( v_tmp1, v_sigma_mu );
			v_resm0 = _mm256_load_pd( &ptr_res_m[jj+0] );
			v_resm1 = _mm256_load_pd( &ptr_res_m[jj+png] );
			v_resm0 = _mm256_add_pd( v_resm0, v_tmp0 );
			v_resm1 = _mm256_add_pd( v_resm1, v_tmp1 );
			_mm256_store_pd( &ptr_res_m[jj+0], v_resm0 );
			_mm256_store_pd( &ptr_res_m[jj+png], v_resm1 );
			}
		if(jj<ng[ii])
			{
			ii_left = ng[ii]-jj;
			i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

			v_dt0   = _mm256_load_pd( &ptr_dt[jj+0] );
			v_dt1   = _mm256_load_pd( &ptr_dt[jj+png] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[jj+0] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[jj+png] );
			v_tmp0  = _mm256_mul_pd( v_dt0, v_dlam0 );
			v_tmp1  = _mm256_mul_pd( v_dt1, v_dlam1 );
			v_tmp0  = _mm256_sub_pd( v_tmp0, v_sigma_mu );
			v_tmp1  = _mm256_sub_pd( v_tmp1, v_sigma_mu );
			v_resm0 = _mm256_load_pd( &ptr_res_m[jj+0] );
			v_resm1 = _mm256_load_pd( &ptr_res_m[jj+png] );
			v_resm0 = _mm256_add_pd( v_resm0, v_tmp0 );
			v_resm1 = _mm256_add_pd( v_resm1, v_tmp1 );
			_mm256_maskstore_pd( &ptr_res_m[jj+0], i_mask, v_resm0 );
			_mm256_maskstore_pd( &ptr_res_m[jj+png], i_mask, v_resm1 );
			}

		}

	}



void d_update_gradient_res_mpc_hard_tv(int N, int *nx, int *nu, int *nb, int *ng, double **res_d, double **res_m, double **lam, double **t_inv, double **qx)
	{

	// constants
	const int bs = D_MR;

	int nb0, pnb, ng0, png;

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

	int ii, jj, bs0;

	double ii_left;

	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

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

				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+pnb] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+pnb] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+pnb] );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ii+pnb] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
				v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
				v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
				v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
				v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
				_mm256_store_pd( &ptr_qx[ii+0], v_tmp0 );

				}
			if(ii<nb0)
				{

				ii_left = nb0-ii;
				i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+pnb] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+pnb] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+pnb] );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ii+pnb] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
				v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
				v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
				v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
				v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
				_mm256_maskstore_pd( &ptr_qx[ii+0], i_mask, v_tmp0 );

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

				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+png] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+png] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+png] );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ii+png] );
				v_tmp0  = _mm256_mul_pd( v_lam0, v_resd0 );
				v_tmp1  = _mm256_mul_pd( v_lam1, v_resd1 );
				v_tmp0  = _mm256_sub_pd( v_resm0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_resm1, v_tmp1 );
				v_tmp0  = _mm256_mul_pd( v_tmp0, v_tinv0 );
				v_tmp1  = _mm256_mul_pd( v_tmp1, v_tinv1 );
				v_tmp0  = _mm256_sub_pd( v_tmp0, v_tmp1 );
				_mm256_store_pd( &ptr_qx[ii+0], v_tmp0 );

				}
			if(ii<ng0)
				{

				ii_left = ng0-ii;
				i_mask  = _mm256_castpd_si256( _mm256_sub_pd( _mm256_loadu_pd( d_mask ), _mm256_broadcast_sd( &ii_left ) ) );

				v_lam0  = _mm256_load_pd( &ptr_lam[ii+0] );
				v_lam1  = _mm256_load_pd( &ptr_lam[ii+png] );
				v_resm0 = _mm256_load_pd( &ptr_res_m[ii+0] );
				v_resm1 = _mm256_load_pd( &ptr_res_m[ii+png] );
				v_resd0 = _mm256_load_pd( &ptr_res_d[ii+0] );
				v_resd1 = _mm256_load_pd( &ptr_res_d[ii+png] );
				v_tinv0 = _mm256_load_pd( &ptr_t_inv[ii+0] );
				v_tinv1 = _mm256_load_pd( &ptr_t_inv[ii+png] );
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

		}

	}
