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
		jj=0;
		for(ll=0; ll<nu[jj]; ll++)
			{
			ux[jj][ll] = 0.0;
			}
		for(jj=1; jj<=N; jj++)
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
			t[jj][pnb+ll] = - db[jj][pnb+ll] - ux[jj][idxb[jj][ll]];
			if(t[jj][ll] < thr0)
				{
				if(t[jj][pnb+ll] < thr0)
					{
					ux[jj][idxb[jj][ll]] = ( - db[jj][pnb+ll] + db[jj][ll])*0.5;
					t[jj][ll]     = - db[jj][ll]     + ux[jj][idxb[jj][ll]];
					t[jj][pnb+ll] = - db[jj][pnb+ll] - ux[jj][idxb[jj][ll]];
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
				ux[jj][idxb[jj][ll]] = - db[jj][pnb+ll] - thr0;
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
				ptr_t[ll+png] += - ptr_db[ll+png];
				ptr_t[ll]      = fmax( thr0, ptr_t[ll] );
				ptr_t[png+ll] = fmax( thr0, ptr_t[png+ll] );
				ptr_lam[ll]      = mu0/ptr_t[ll];
				ptr_lam[png+ll] = mu0/ptr_t[png+ll];
				}
			}
		}

	}



void d_update_hessian_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double sigma_mu, double **t, double **tinv, double **lam, double **lamt, double **dlam, double **Qx, double **qx, double **qx2, double **bd, double **bl, double **pd, double **pl, double **db, double **Z, double **z, double **Zl, double **zl)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	__m256d
		v_ones, v_sigma_mu, v_mask, v_left,
		v_tmp, v_lam, v_lamt, v_dlam, v_db,
		v_tmp0, v_tmp1, v_tmp2, v_tmp3,
		v_lam0, v_lam1, v_lam2, v_lam3,
		v_lamt0, v_lamt1, v_lamt2, v_lamt3,
		v_dlam0, v_dlam1, v_dlam2, v_dlam3,
		v_Qx0, v_Qx1,
		v_qx0, v_qx1,
		v_bd0, v_bd2,
		v_db0, v_db2,
		v_Zl0, v_Zl1,
		v_zl0, v_zl1;
	
	__m256i
		i_mask;

	double 
		*ptr_pd, *ptr_pl, *ptr_bd, *ptr_bl, *ptr_db, *ptr_Qx, *ptr_qx, *ptr_qx2,
		*ptr_t, *ptr_lam, *ptr_lamt, *ptr_dlam, *ptr_tinv, 
		*ptr_Z, *ptr_z, *ptr_Zl, *ptr_zl;
	
	v_ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );
	v_sigma_mu = _mm256_set_pd( sigma_mu, sigma_mu, sigma_mu, sigma_mu );

	int ii, jj, bs0;

	double ii_left;

	int nb0, pnb, ng0, png, ns0, pns;
	
	static double d_mask[4] = {0.5, 1.5, 2.5, 3.5};

	for(jj=0; jj<=N; jj++)
		{
		
		ptr_t     = t[jj];
		ptr_lam   = lam[jj];
		ptr_lamt  = lamt[jj];
		ptr_dlam  = dlam[jj];
		ptr_tinv  = tinv[jj];
		ptr_db    = db[jj];
		ptr_bd    = bd[jj];
		ptr_bl    = bl[jj];
		ptr_pd    = pd[jj];
		ptr_pl    = pl[jj];

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
				v_qx0   = _mm256_load_pd( &ptr_db[ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[pnb+ii] );
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
				v_qx1   = _mm256_add_pd( v_qx1, v_lam1 );

				v_lamt0 = _mm256_add_pd( v_lamt0, v_lamt1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				v_tmp0  = _mm256_load_pd( &ptr_bd[ii] );
				v_tmp1  = _mm256_load_pd( &ptr_bl[ii] );
				v_tmp0  = _mm256_add_pd( v_lamt0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_qx0 );
				_mm256_store_pd( &ptr_pd[ii], v_tmp0 );
				_mm256_store_pd( &ptr_pl[ii], v_tmp1 );

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

				v_Qx0   = v_lamt0;
				v_Qx1   = v_lamt1;
				v_qx0   = _mm256_load_pd( &ptr_db[0*pnb+ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[1*pnb+ii] );
				v_qx0   = _mm256_mul_pd( v_qx0, v_lamt0 );
				v_qx1   = _mm256_mul_pd( v_qx1, v_lamt1 );
				v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
				v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
				v_qx0   = _mm256_add_pd( v_qx0, v_lam0 );
				v_qx1   = _mm256_add_pd( v_qx1, v_lam1 );

				v_Qx0   = _mm256_add_pd( v_Qx0, v_Qx1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				v_tmp0  = _mm256_load_pd( &bd[jj][ii] );
				v_tmp1  = _mm256_load_pd( &bl[jj][ii] );
				v_tmp0  = _mm256_add_pd( v_Qx0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_qx0 );
				_mm256_maskstore_pd( &pd[jj][ii], i_mask, v_tmp0 );
				_mm256_maskstore_pd( &pl[jj][ii], i_mask, v_tmp1 );

				}
		
			ptr_t     += 2*pnb;
			ptr_lam   += 2*pnb;
			ptr_lamt  += 2*pnb;
			ptr_dlam  += 2*pnb;
			ptr_tinv  += 2*pnb;
			ptr_db    += 2*pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			ptr_Qx    = Qx[jj];
			ptr_qx    = qx[jj];
			ptr_qx2   = qx2[jj];

			png  = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

			for(ii=0; ii<ng0-3; ii+=4)
				{
				
				v_tmp0  = _mm256_load_pd( &ptr_t[0*png+ii] );
				v_tmp1  = _mm256_load_pd( &ptr_t[1*png+ii] );
				v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
				v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
				_mm256_store_pd( &ptr_tinv[0*png+ii], v_tmp0 );
				_mm256_store_pd( &ptr_tinv[1*png+ii], v_tmp1 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*png+ii] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*png+ii] );
				v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
				v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
				_mm256_store_pd( &ptr_lamt[0*png+ii], v_lamt0 );
				_mm256_store_pd( &ptr_lamt[1*png+ii], v_lamt1 );
				v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
				v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
				_mm256_store_pd( &ptr_dlam[0*png+ii], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[1*png+ii], v_dlam1 );

				v_Qx0   = _mm256_add_pd( v_lamt0, v_lamt1 );
				v_Qx0   = _mm256_sqrt_pd( v_Qx0 );
				_mm256_store_pd( &ptr_Qx[ii], v_Qx0 );
				v_qx0   = _mm256_load_pd( &ptr_db[0*png+ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[1*png+ii] );
				v_qx0   = _mm256_mul_pd( v_qx0, v_lamt0 );
				v_qx1   = _mm256_mul_pd( v_qx1, v_lamt1 );
				v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
				v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
				v_qx0   = _mm256_add_pd( v_qx0, v_lam0 );
				v_qx1   = _mm256_add_pd( v_qx1, v_lam1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				_mm256_store_pd( &ptr_qx[ii], v_qx0 );
				v_qx0   = _mm256_div_pd( v_qx0, v_Qx0 );
				_mm256_store_pd( &ptr_qx2[ii], v_qx0 );

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

				v_Qx0   = _mm256_add_pd( v_lamt0, v_lamt1 );
				v_Qx0   = _mm256_sqrt_pd( v_Qx0 );
				_mm256_maskstore_pd( &ptr_Qx[ii], i_mask, v_Qx0 );
				v_qx0   = _mm256_load_pd( &ptr_db[0*png+ii] );
				v_qx1   = _mm256_load_pd( &ptr_db[1*png+ii] );
				v_qx0   = _mm256_mul_pd( v_qx0, v_lamt0 );
				v_qx1   = _mm256_mul_pd( v_qx1, v_lamt1 );
				v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
				v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
				v_qx0   = _mm256_add_pd( v_qx0, v_lam0 );
				v_qx1   = _mm256_add_pd( v_qx1, v_lam1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				_mm256_maskstore_pd( &ptr_qx[ii], i_mask, v_qx0 );
				v_qx0   = _mm256_div_pd( v_qx0, v_Qx0 );
				_mm256_maskstore_pd( &ptr_qx2[ii], i_mask, v_qx0 );

				}

			ptr_t     += 2*png;
			ptr_lam   += 2*png;
			ptr_lamt  += 2*png;
			ptr_dlam  += 2*png;
			ptr_tinv  += 2*png;
			ptr_db    += 2*png;

			}

		// box soft constraints
		ns0 = ns[jj];
		if(ns0>0)
			{

			ptr_Z     = Z[jj];
			ptr_z     = z[jj];
			ptr_Zl    = Zl[jj];
			ptr_zl    = zl[jj];

			pns  = (ns0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<ns0-3; ii+=4)
				{

				v_tmp0  = _mm256_load_pd( &ptr_t[0*pns+ii] ); // lower bound
				v_tmp1  = _mm256_load_pd( &ptr_t[1*pns+ii] ); // upper bound
				v_tmp2  = _mm256_load_pd( &ptr_t[2*pns+ii] ); // lower slack
				v_tmp3  = _mm256_load_pd( &ptr_t[3*pns+ii] ); // upper slack
				v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
				v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
				v_tmp2  = _mm256_div_pd( v_ones, v_tmp2 );
				v_tmp3  = _mm256_div_pd( v_ones, v_tmp3 );
				_mm256_store_pd( &ptr_tinv[0*pns+ii], v_tmp0 );
				_mm256_store_pd( &ptr_tinv[1*pns+ii], v_tmp1 );
				_mm256_store_pd( &ptr_tinv[2*pns+ii], v_tmp2 );
				_mm256_store_pd( &ptr_tinv[3*pns+ii], v_tmp3 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ii] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ii] );
				v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ii] );
				v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ii] );
				v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
				v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
				v_lamt2 = _mm256_mul_pd( v_tmp2, v_lam2 );
				v_lamt3 = _mm256_mul_pd( v_tmp3, v_lam3 );
				_mm256_store_pd( &ptr_lamt[0*pns+ii], v_lamt0 );
				_mm256_store_pd( &ptr_lamt[1*pns+ii], v_lamt1 );
				_mm256_store_pd( &ptr_lamt[2*pns+ii], v_lamt2 );
				_mm256_store_pd( &ptr_lamt[3*pns+ii], v_lamt3 );
				v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
				v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
				v_dlam2 = _mm256_mul_pd( v_tmp2, v_sigma_mu );
				v_dlam3 = _mm256_mul_pd( v_tmp3, v_sigma_mu );
				_mm256_store_pd( &ptr_dlam[0*pns+ii], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[1*pns+ii], v_dlam1 );
				_mm256_store_pd( &ptr_dlam[2*pns+ii], v_dlam2 );
				_mm256_store_pd( &ptr_dlam[3*pns+ii], v_dlam3 );

				v_Qx0 = v_lamt0;
				v_Qx1 = v_lamt1;
				v_qx0  = _mm256_load_pd( &ptr_db[0*pns+ii] );
				v_qx1  = _mm256_load_pd( &ptr_db[1*pns+ii] );
				v_qx0  = _mm256_mul_pd( v_qx0, v_lamt0 );
				v_qx1  = _mm256_mul_pd( v_qx1, v_lamt1 );
				v_lam0 = _mm256_add_pd( v_lam0, v_dlam0 );
				v_lam1 = _mm256_add_pd( v_lam1, v_dlam1 );
				v_qx0  = _mm256_add_pd( v_qx0, v_lam0 );
				v_qx1  = _mm256_add_pd( v_qx1, v_lam1 );

				v_Zl0  = _mm256_load_pd( &ptr_Z[0*pns+ii] );
				v_Zl1  = _mm256_load_pd( &ptr_Z[1*pns+ii] );
				v_zl0  = _mm256_load_pd( &ptr_z[0*pns+ii] );
				v_zl1  = _mm256_load_pd( &ptr_z[1*pns+ii] );
				v_Zl0  = _mm256_add_pd( v_Zl0, v_Qx0 );
				v_Zl1  = _mm256_add_pd( v_Zl1, v_Qx1 );
				v_Zl0  = _mm256_add_pd( v_Zl0, v_lamt2 );
				v_Zl1  = _mm256_add_pd( v_Zl1, v_lamt3 );
				v_Zl0  = _mm256_div_pd( v_ones, v_Zl0 );
				v_Zl1  = _mm256_div_pd( v_ones, v_Zl1 );
				v_zl0  = _mm256_sub_pd( v_qx0, v_zl0 );
				v_zl1  = _mm256_sub_pd( v_qx1, v_zl1 );
				v_zl0  = _mm256_add_pd( v_zl0, v_lam2 );
				v_zl1  = _mm256_add_pd( v_zl1, v_lam3 );
				v_zl0  = _mm256_add_pd( v_zl0, v_dlam2 );
				v_zl1  = _mm256_add_pd( v_zl1, v_dlam3 );
				_mm256_store_pd( &ptr_Zl[0*pns+ii], v_Zl0 );
				_mm256_store_pd( &ptr_Zl[1*pns+ii], v_Zl1 );
				_mm256_store_pd( &ptr_zl[0*pns+ii], v_zl0 );
				_mm256_store_pd( &ptr_zl[1*pns+ii], v_zl1 );
				v_tmp0 = _mm256_mul_pd( v_Qx0, v_Zl0 );
				v_tmp1 = _mm256_mul_pd( v_Qx1, v_Zl1 );
				v_tmp2 = _mm256_mul_pd( v_tmp0, v_zl0 );
				v_tmp3 = _mm256_mul_pd( v_tmp1, v_zl1 );
				v_qx0  = _mm256_sub_pd( v_qx0, v_tmp2 );
				v_qx1  = _mm256_sub_pd( v_qx1, v_tmp3 );
				v_tmp0 = _mm256_mul_pd( v_Qx0, v_tmp0 );
				v_tmp1 = _mm256_mul_pd( v_Qx1, v_tmp1 );
				v_Qx0  = _mm256_sub_pd( v_Qx0, v_tmp0 );
				v_Qx1  = _mm256_sub_pd( v_Qx1, v_tmp1 );

				v_Qx0   = _mm256_add_pd( v_Qx0, v_Qx1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				v_tmp0  = _mm256_loadu_pd( &ptr_bd[nb0+ii] );
				v_tmp1  = _mm256_loadu_pd( &ptr_bl[nb0+ii] );
				v_tmp0  = _mm256_add_pd( v_Qx0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_qx0 );
				_mm256_storeu_pd( &ptr_pd[nb0+ii], v_tmp0 );
				_mm256_storeu_pd( &ptr_pl[nb0+ii], v_tmp1 );

				}
			if(ii<ns0)
				{

				ii_left = ns0 - ii;
				v_left  = _mm256_broadcast_sd( &ii_left );
				v_mask  = _mm256_loadu_pd( d_mask );
				i_mask  = _mm256_castpd_si256( _mm256_sub_pd( v_mask, v_left ) );

				v_tmp0  = _mm256_load_pd( &ptr_t[0*pns+ii] ); // lower bound
				v_tmp1  = _mm256_load_pd( &ptr_t[1*pns+ii] ); // upper bound
				v_tmp2  = _mm256_load_pd( &ptr_t[2*pns+ii] ); // lower slack
				v_tmp3  = _mm256_load_pd( &ptr_t[3*pns+ii] ); // upper slack
				v_tmp0  = _mm256_div_pd( v_ones, v_tmp0 );
				v_tmp1  = _mm256_div_pd( v_ones, v_tmp1 );
				v_tmp2  = _mm256_div_pd( v_ones, v_tmp2 );
				v_tmp3  = _mm256_div_pd( v_ones, v_tmp3 );
				_mm256_maskstore_pd( &ptr_tinv[0*pns+ii], i_mask, v_tmp0 );
				_mm256_maskstore_pd( &ptr_tinv[1*pns+ii], i_mask, v_tmp1 );
				_mm256_maskstore_pd( &ptr_tinv[2*pns+ii], i_mask, v_tmp2 );
				_mm256_maskstore_pd( &ptr_tinv[3*pns+ii], i_mask, v_tmp3 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ii] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ii] );
				v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ii] );
				v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ii] );
				v_lamt0 = _mm256_mul_pd( v_tmp0, v_lam0 );
				v_lamt1 = _mm256_mul_pd( v_tmp1, v_lam1 );
				v_lamt2 = _mm256_mul_pd( v_tmp2, v_lam2 );
				v_lamt3 = _mm256_mul_pd( v_tmp3, v_lam3 );
				_mm256_maskstore_pd( &ptr_lamt[0*pns+ii], i_mask, v_lamt0 );
				_mm256_maskstore_pd( &ptr_lamt[1*pns+ii], i_mask, v_lamt1 );
				_mm256_maskstore_pd( &ptr_lamt[2*pns+ii], i_mask, v_lamt2 );
				_mm256_maskstore_pd( &ptr_lamt[3*pns+ii], i_mask, v_lamt3 );
				v_dlam0 = _mm256_mul_pd( v_tmp0, v_sigma_mu );
				v_dlam1 = _mm256_mul_pd( v_tmp1, v_sigma_mu );
				v_dlam2 = _mm256_mul_pd( v_tmp2, v_sigma_mu );
				v_dlam3 = _mm256_mul_pd( v_tmp3, v_sigma_mu );
				_mm256_maskstore_pd( &ptr_dlam[0*pns+ii], i_mask, v_dlam0 );
				_mm256_maskstore_pd( &ptr_dlam[1*pns+ii], i_mask, v_dlam1 );
				_mm256_maskstore_pd( &ptr_dlam[2*pns+ii], i_mask, v_dlam2 );
				_mm256_maskstore_pd( &ptr_dlam[3*pns+ii], i_mask, v_dlam3 );

				v_Qx0 = v_lamt0;
				v_Qx1 = v_lamt1;
				v_qx0  = _mm256_load_pd( &ptr_db[0*pns+ii] );
				v_qx1  = _mm256_load_pd( &ptr_db[1*pns+ii] );
				v_qx0  = _mm256_mul_pd( v_qx0, v_lamt0 );
				v_qx1  = _mm256_mul_pd( v_qx1, v_lamt1 );
				v_lam0 = _mm256_add_pd( v_lam0, v_dlam0 );
				v_lam1 = _mm256_add_pd( v_lam1, v_dlam1 );
				v_qx0  = _mm256_add_pd( v_qx0, v_lam0 );
				v_qx1  = _mm256_add_pd( v_qx1, v_lam1 );

				v_Zl0  = _mm256_load_pd( &ptr_Z[0*pns+ii] );
				v_Zl1  = _mm256_load_pd( &ptr_Z[1*pns+ii] );
				v_zl0  = _mm256_load_pd( &ptr_z[0*pns+ii] );
				v_zl1  = _mm256_load_pd( &ptr_z[1*pns+ii] );
				v_Zl0  = _mm256_add_pd( v_Zl0, v_Qx0 );
				v_Zl1  = _mm256_add_pd( v_Zl1, v_Qx1 );
				v_Zl0  = _mm256_add_pd( v_Zl0, v_lamt2 );
				v_Zl1  = _mm256_add_pd( v_Zl1, v_lamt3 );
				v_Zl0  = _mm256_div_pd( v_ones, v_Zl0 );
				v_Zl1  = _mm256_div_pd( v_ones, v_Zl1 );
				v_zl0  = _mm256_sub_pd( v_qx0, v_zl0 );
				v_zl1  = _mm256_sub_pd( v_qx1, v_zl1 );
				v_zl0  = _mm256_add_pd( v_zl0, v_lam2 );
				v_zl1  = _mm256_add_pd( v_zl1, v_lam3 );
				v_zl0  = _mm256_add_pd( v_zl0, v_dlam2 );
				v_zl1  = _mm256_add_pd( v_zl1, v_dlam3 );
				_mm256_maskstore_pd( &ptr_Zl[0*pns+ii], i_mask, v_Zl0 );
				_mm256_maskstore_pd( &ptr_Zl[1*pns+ii], i_mask, v_Zl1 );
				_mm256_maskstore_pd( &ptr_zl[0*pns+ii], i_mask, v_zl0 );
				_mm256_maskstore_pd( &ptr_zl[1*pns+ii], i_mask, v_zl1 );
				v_tmp0 = _mm256_mul_pd( v_Qx0, v_Zl0 );
				v_tmp1 = _mm256_mul_pd( v_Qx1, v_Zl1 );
				v_tmp2 = _mm256_mul_pd( v_tmp0, v_zl0 );
				v_tmp3 = _mm256_mul_pd( v_tmp1, v_zl1 );
				v_qx0  = _mm256_sub_pd( v_qx0, v_tmp2 );
				v_qx1  = _mm256_sub_pd( v_qx1, v_tmp3 );
				v_tmp0 = _mm256_mul_pd( v_Qx0, v_tmp0 );
				v_tmp1 = _mm256_mul_pd( v_Qx1, v_tmp1 );
				v_Qx0  = _mm256_sub_pd( v_Qx0, v_tmp0 );
				v_Qx1  = _mm256_sub_pd( v_Qx1, v_tmp1 );

				v_Qx0   = _mm256_add_pd( v_Qx0, v_Qx1 );
				v_qx0   = _mm256_sub_pd( v_qx1, v_qx0 );
				v_tmp0  = _mm256_loadu_pd( &ptr_bd[nb0+ii] );
				v_tmp1  = _mm256_loadu_pd( &ptr_bl[nb0+ii] );
				v_tmp0  = _mm256_add_pd( v_Qx0, v_tmp0 );
				v_tmp1  = _mm256_add_pd( v_tmp1, v_qx0 );
				_mm256_maskstore_pd( &ptr_pd[nb0+ii], i_mask, v_tmp0 );
				_mm256_maskstore_pd( &ptr_pl[nb0+ii], i_mask, v_tmp1 );

				}
			}

		} // end of loop

	}



void d_update_gradient_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double sigma_mu, double **dt, double **dlam, double **t_inv, double **lamt, double **pl2, double **qxr, double **Zl, double **zl)
	{

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii, jj;

	int nb0, pnb, ng0, png, ns0, pns;

	double
		*ptr_dlam, *ptr_t_inv, *ptr_dt, *ptr_lamt, *ptr_pl2, *ptr_qx, *ptr_Zl, *ptr_zl;

	static double Qx[2] = {};
	static double qx[2] = {};

	for(jj=0; jj<=N; jj++)
		{

		ptr_dlam  = dlam[jj];
		ptr_dt    = dt[jj];
		ptr_lamt  = lamt[jj];
		ptr_t_inv = t_inv[jj];
		ptr_pl2   = pl2[jj];

		// box constraints
		nb0 = nb[jj];
		if(nb0>0)
			{

			pnb  = (nb0+bs-1)/bs*bs; // simd aligned number of box constraints

			for(ii=0; ii<nb0; ii++)
				{
				ptr_dlam[0*pnb+ii] = ptr_t_inv[0*pnb+ii]*(sigma_mu - ptr_dlam[0*pnb+ii]*ptr_dt[0*pnb+ii]);
				ptr_dlam[1*pnb+ii] = ptr_t_inv[1*pnb+ii]*(sigma_mu - ptr_dlam[1*pnb+ii]*ptr_dt[1*pnb+ii]);
				ptr_pl2[ii] += ptr_dlam[1*pnb+ii] - ptr_dlam[0*pnb+ii];
				}

			ptr_dlam  += 2*pnb;
			ptr_dt    += 2*pnb;
			ptr_lamt  += 2*pnb;
			ptr_t_inv += 2*pnb;

			}

		// general constraints
		ng0 = ng[jj];
		if(ng0>0)
			{

			ptr_qx    = qxr[jj];

			png  = (ng0+bs-1)/bs*bs; // simd aligned number of general constraints

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

			}

		// box soft constraitns
		ns0 = ns[jj];
		if(ns0>0)
			{

			ptr_Zl    = Zl[jj];
			ptr_zl    = zl[jj];

			pns  = (ns0+bs-1)/bs*bs; // simd aligned number of box soft constraints

			for(ii=0; ii<ns0; ii++)
				{
				ptr_dlam[0*pns+ii] = ptr_t_inv[0*pns+ii]*(sigma_mu - ptr_dlam[0*pns+ii]*ptr_dt[0*pns+ii]);
				ptr_dlam[1*pns+ii] = ptr_t_inv[1*pns+ii]*(sigma_mu - ptr_dlam[1*pns+ii]*ptr_dt[1*pns+ii]);
				ptr_dlam[2*pns+ii] = ptr_t_inv[2*pns+ii]*(sigma_mu - ptr_dlam[2*pns+ii]*ptr_dt[2*pns+ii]);
				ptr_dlam[3*pns+ii] = ptr_t_inv[3*pns+ii]*(sigma_mu - ptr_dlam[3*pns+ii]*ptr_dt[3*pns+ii]);
				Qx[0] = ptr_lamt[0*pns+ii];
				Qx[1] = ptr_lamt[1*pns+ii];
				qx[0] = ptr_dlam[0*pns+ii];
				qx[1] = ptr_dlam[1*pns+ii];
				ptr_zl[0*pns+ii] += qx[0] + ptr_dlam[2*pns+ii];
				ptr_zl[1*pns+ii] += qx[1] + ptr_dlam[3*pns+ii];
				qx[0] = qx[0] - Qx[0]*(qx[0] + ptr_dlam[2*pns+ii])*ptr_Zl[0*pns+ii];
				qx[1] = qx[1] - Qx[1]*(qx[1] + ptr_dlam[3*pns+ii])*ptr_Zl[1*pns+ii];
				ptr_pl2[nb0+ii] += qx[1] - qx[0];
				}

			}

		}

	}




void d_compute_alpha_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int **idxb, int *ng, int *ns, double *ptr_alpha, double **t, double **dt, double **lam, double **dlam, double **lamt, double **dux, double **pDCt, double **db, double **Zl, double **zl)
	{
	
	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	__m256
		t_sign, t_ones, t_zeros,
		t_mask0, t_mask1, t_mask2, t_mask3,
		t_lam, t_dlam, t_t, t_dt,
		t_lam0, t_lam1, t_dlam0, t_dlam1,
		t_t0, t_t1, t_dt0, t_dt1,
		t_tmp0, t_tmp1, t_tmp2, t_tmp3,
		t_alpha0, t_alpha1;

	__m128
		s_sign, s_ones, s_mask, s_mask0, s_mask1, s_zeros,
		s_lam, s_dlam, s_t, s_dt, s_tmp0, s_tmp1, s_alpha0, s_alpha1;
	
	__m256d
		v_sign, v_alpha, v_mask, v_dux, v_left,
		v_temp0, v_dt0, v_db0, v_dlam0, v_lamt0, v_t0, v_lam0,
		v_temp1, v_dt1, v_db1, v_dlam1, v_lamt1, v_t1, v_lam1,
		v_temp2, v_dt2, v_db2, v_dlam2, v_lamt2, v_t2, v_lam2,
		v_temp3, v_dt3, v_db3, v_dlam3, v_lamt3, v_t3, v_lam3;
	
	__m128d
		u_sign, u_dux, u_alpha,
		u_dt0, u_temp0, u_db0, u_dlam0, u_lamt0, u_t0, u_lam0,
		u_dt1, u_temp1, u_db1, u_dlam1, u_lamt1, u_t1, u_lam1;
	
	__m256i
		i_mask;
	
	int nu0, nx0, nb0, pnb, ng0, png, cng, ns0, pns;

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
		*ptr_db, *ptr_dux, *ptr_t, *ptr_dt, *ptr_lamt, *ptr_lam, *ptr_dlam, *ptr_zl, *ptr_Zl;
	
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
				v_db1   = _mm256_xor_pd( v_db1, v_sign );
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
				v_db1   = _mm256_xor_pd( v_db1, v_sign );
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
				v_dt1   = _mm256_sub_pd ( v_dt1, v_db1 );
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
				v_dt1   = _mm256_sub_pd ( v_dt1, v_db1 );
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

			for(ll=0; ll<ns0-3; ll+=4)
				{
				//v_dux   = _mm256_load_pd( &ptr_dux[ll] );
				u_temp0 = _mm_load_sd( &ptr_dux[ptr_idxb[nb0+ll+0]] );
				u_temp1 = _mm_load_sd( &ptr_dux[ptr_idxb[nb0+ll+2]] );
				u_temp0 = _mm_loadh_pd( u_temp0, &ptr_dux[ptr_idxb[nb0+ll+1]] );
				u_temp1 = _mm_loadh_pd( u_temp1, &ptr_dux[ptr_idxb[nb0+ll+3]] );
				v_dux   = _mm256_castpd128_pd256( u_temp0 );
				v_dux   = _mm256_insertf128_pd( v_dux, u_temp1, 0x1 );
				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*pns+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*pns+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dux );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dux );
				v_dt2   = _mm256_load_pd( &ptr_zl[0*pns+ll] );
				v_dt3   = _mm256_load_pd( &ptr_zl[1*pns+ll] );
				v_dt2   = _mm256_sub_pd ( v_dt2, v_temp0 );
				v_dt3   = _mm256_add_pd ( v_dt3, v_temp1 );
				v_temp0 = _mm256_load_pd( &ptr_Zl[0*pns+ll] );
				v_temp1 = _mm256_load_pd( &ptr_Zl[1*pns+ll] );
				v_dt2   = _mm256_mul_pd ( v_dt2, v_temp0 );
				v_dt3   = _mm256_mul_pd ( v_dt3, v_temp1 );
				v_dt0   = _mm256_add_pd ( v_dt2, v_dux );
				v_dt1   = _mm256_sub_pd ( v_dt3, v_dux );
				v_db0   = _mm256_load_pd( &ptr_db[0*pns+ll] );
				v_db1   = _mm256_load_pd( &ptr_db[1*pns+ll] );
				v_dt0   = _mm256_sub_pd ( v_dt0, v_db0 );
				v_dt1   = _mm256_sub_pd ( v_dt1, v_db1 );
				v_t0    = _mm256_load_pd( &ptr_t[0*pns+ll] );
				v_t1    = _mm256_load_pd( &ptr_t[1*pns+ll] );
				v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
				v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
				_mm256_store_pd( &ptr_dt[0*pns+ll], v_dt0 );
				_mm256_store_pd( &ptr_dt[1*pns+ll], v_dt1 );
				v_t2    = _mm256_load_pd( &ptr_t[2*pns+ll] );
				v_t3    = _mm256_load_pd( &ptr_t[3*pns+ll] );
				v_dt2   = _mm256_sub_pd( v_dt2, v_t2 );
				v_dt3   = _mm256_sub_pd( v_dt3, v_t3 );
				_mm256_store_pd( &ptr_dt[2*pns+ll], v_dt2 );
				_mm256_store_pd( &ptr_dt[3*pns+ll], v_dt3 );
	//			printf("\n%f %f %f %f\n", ptr_dt[0*pns+ll+0], ptr_dt[1*pns+ll+0], ptr_dt[2*pns+ll+0], ptr_dt[3*pns+ll+0]);
	//			printf("\n%f %f %f %f\n", ptr_dt[0*pns+ll+1], ptr_dt[1*pns+ll+1], ptr_dt[2*pns+ll+1], ptr_dt[3*pns+ll+1]);
	//			printf("\n%f %f %f %f\n", ptr_dt[0*pns+ll+2], ptr_dt[1*pns+ll+2], ptr_dt[2*pns+ll+2], ptr_dt[3*pns+ll+2]);
	//			printf("\n%f %f %f %f\n", ptr_dt[0*pns+ll+3], ptr_dt[1*pns+ll+3], ptr_dt[2*pns+ll+3], ptr_dt[3*pns+ll+3]);

				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*pns+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*pns+ll] );
				v_lamt2 = _mm256_load_pd( &ptr_lamt[2*pns+ll] );
				v_lamt3 = _mm256_load_pd( &ptr_lamt[3*pns+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
				v_temp2 = _mm256_mul_pd( v_lamt2, v_dt2 );
				v_temp3 = _mm256_mul_pd( v_lamt3, v_dt3 );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pns+ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pns+ll] );
				v_dlam2 = _mm256_load_pd( &ptr_dlam[2*pns+ll] );
				v_dlam3 = _mm256_load_pd( &ptr_dlam[3*pns+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
				v_dlam2 = _mm256_sub_pd( v_dlam2, v_temp2 );
				v_dlam3 = _mm256_sub_pd( v_dlam3, v_temp3 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ll] );
				v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ll] );
				v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
				v_dlam2 = _mm256_sub_pd( v_dlam2, v_lam2 );
				v_dlam3 = _mm256_sub_pd( v_dlam3, v_lam3 );
				_mm256_store_pd( &ptr_dlam[0*pns+ll], v_dlam0 );
				_mm256_store_pd( &ptr_dlam[1*pns+ll], v_dlam1 );
				_mm256_store_pd( &ptr_dlam[2*pns+ll], v_dlam2 );
				_mm256_store_pd( &ptr_dlam[3*pns+ll], v_dlam3 );

				t_dlam0  = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
				t_dt0    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
				t_dlam1  = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam3 ) ), 0x20 );
				t_dt1    = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt3 ) ), 0x20 );
				t_mask0  = _mm256_cmp_ps( t_dlam0, t_zeros, 0x01 );
				t_mask1  = _mm256_cmp_ps( t_dt0, t_zeros, 0x01 );
				t_mask2  = _mm256_cmp_ps( t_dlam1, t_zeros, 0x01 );
				t_mask3  = _mm256_cmp_ps( t_dt1, t_zeros, 0x01 );
				t_lam0   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam1 ) ), 0x20 );
				t_t0     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t1 ) ), 0x20 );
				t_lam1   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam3 ) ), 0x20 );
				t_t1     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t3 ) ), 0x20 );
				t_lam0   = _mm256_xor_ps( t_lam0, t_sign );
				t_t0     = _mm256_xor_ps( t_t0, t_sign );
				t_lam1   = _mm256_xor_ps( t_lam1, t_sign );
				t_t1     = _mm256_xor_ps( t_t1, t_sign );
				t_tmp0   = _mm256_div_ps( t_lam0, t_dlam0 );
				t_tmp1   = _mm256_div_ps( t_t0, t_dt0 );
				t_tmp2   = _mm256_div_ps( t_lam1, t_dlam1 );
				t_tmp3   = _mm256_div_ps( t_t1, t_dt1 );
				t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
				t_tmp1   = _mm256_blendv_ps( t_ones, t_tmp1, t_mask1 );
				t_tmp2   = _mm256_blendv_ps( t_ones, t_tmp2, t_mask2 );
				t_tmp3   = _mm256_blendv_ps( t_ones, t_tmp3, t_mask3 );
				t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );
				t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp1 );
				t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp2 );
				t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp3 );

				}
			if(ll<ns0)
				{

				ll_left = ns0 - ll;
				v_left  = _mm256_broadcast_sd( &ll_left );
				v_mask  = _mm256_loadu_pd( d_mask );
				v_mask  = _mm256_sub_pd( v_mask, v_left );
				i_mask  = _mm256_castpd_si256( v_mask );

				u_temp0 = _mm_load_sd( &ptr_dux[ptr_idxb[nb0+ll+0]] );
				if(ll_left>1) u_temp0 = _mm_loadh_pd( u_temp0, &ptr_dux[ptr_idxb[nb0+ll+1]] );
				if(ll_left>2) u_temp1 = _mm_load_sd( &ptr_dux[ptr_idxb[nb0+ll+2]] );
				//u_temp1 = _mm_loadh_pd( u_temp1, &ptr_dux[ptr_idxb[nb0+ll+3]] );
				v_dux   = _mm256_castpd128_pd256( u_temp0 );
				v_dux   = _mm256_insertf128_pd( v_dux, u_temp1, 0x1 );
				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*pns+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*pns+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dux );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dux );
				v_dt2   = _mm256_load_pd( &ptr_zl[0*pns+ll] );
				v_dt3   = _mm256_load_pd( &ptr_zl[1*pns+ll] );
				v_dt2   = _mm256_sub_pd ( v_dt2, v_temp0 );
				v_dt3   = _mm256_add_pd ( v_dt3, v_temp1 );
				v_temp0 = _mm256_load_pd( &ptr_Zl[0*pns+ll] );
				v_temp1 = _mm256_load_pd( &ptr_Zl[1*pns+ll] );
				v_dt2   = _mm256_mul_pd ( v_dt2, v_temp0 );
				v_dt3   = _mm256_mul_pd ( v_dt3, v_temp1 );
				v_dt0   = _mm256_add_pd ( v_dt2, v_dux );
				v_dt1   = _mm256_sub_pd ( v_dt3, v_dux );
				v_db0   = _mm256_load_pd( &ptr_db[0*pns+ll] );
				v_db1   = _mm256_load_pd( &ptr_db[1*pns+ll] );
				v_dt0   = _mm256_sub_pd ( v_dt0, v_db0 );
				v_dt1   = _mm256_sub_pd ( v_dt1, v_db1 );
				v_t0    = _mm256_load_pd( &ptr_t[0*pns+ll] );
				v_t1    = _mm256_load_pd( &ptr_t[1*pns+ll] );
				v_dt0   = _mm256_sub_pd( v_dt0, v_t0 );
				v_dt1   = _mm256_sub_pd( v_dt1, v_t1 );
				_mm256_maskstore_pd( &ptr_dt[0*pns+ll], i_mask, v_dt0 );
				_mm256_maskstore_pd( &ptr_dt[1*pns+ll], i_mask, v_dt1 );
				v_t2    = _mm256_load_pd( &ptr_t[2*pns+ll] );
				v_t3    = _mm256_load_pd( &ptr_t[3*pns+ll] );
				v_dt2   = _mm256_sub_pd( v_dt2, v_t2 );
				v_dt3   = _mm256_sub_pd( v_dt3, v_t3 );
				_mm256_maskstore_pd( &ptr_dt[2*pns+ll], i_mask, v_dt2 );
				_mm256_maskstore_pd( &ptr_dt[3*pns+ll], i_mask, v_dt3 );

				v_lamt0 = _mm256_load_pd( &ptr_lamt[0*pns+ll] );
				v_lamt1 = _mm256_load_pd( &ptr_lamt[1*pns+ll] );
				v_lamt2 = _mm256_load_pd( &ptr_lamt[2*pns+ll] );
				v_lamt3 = _mm256_load_pd( &ptr_lamt[3*pns+ll] );
				v_temp0 = _mm256_mul_pd( v_lamt0, v_dt0 );
				v_temp1 = _mm256_mul_pd( v_lamt1, v_dt1 );
				v_temp2 = _mm256_mul_pd( v_lamt2, v_dt2 );
				v_temp3 = _mm256_mul_pd( v_lamt3, v_dt3 );
				v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pns+ll] );
				v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pns+ll] );
				v_dlam2 = _mm256_load_pd( &ptr_dlam[2*pns+ll] );
				v_dlam3 = _mm256_load_pd( &ptr_dlam[3*pns+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_temp0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_temp1 );
				v_dlam2 = _mm256_sub_pd( v_dlam2, v_temp2 );
				v_dlam3 = _mm256_sub_pd( v_dlam3, v_temp3 );
				v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ll] );
				v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ll] );
				v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ll] );
				v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ll] );
				v_dlam0 = _mm256_sub_pd( v_dlam0, v_lam0 );
				v_dlam1 = _mm256_sub_pd( v_dlam1, v_lam1 );
				v_dlam2 = _mm256_sub_pd( v_dlam2, v_lam2 );
				v_dlam3 = _mm256_sub_pd( v_dlam3, v_lam3 );
				_mm256_maskstore_pd( &ptr_dlam[0*pns+ll], i_mask, v_dlam0 );
				_mm256_maskstore_pd( &ptr_dlam[1*pns+ll], i_mask, v_dlam1 );
				_mm256_maskstore_pd( &ptr_dlam[2*pns+ll], i_mask, v_dlam2 );
				_mm256_maskstore_pd( &ptr_dlam[3*pns+ll], i_mask, v_dlam3 );

				if(ll<nb0-2) // 3 left
					{

					t_dlam0   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), 0x20 );
					t_dt0     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
					t_dlam1   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam3 ) ), 0x20 );
					t_dt1     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt3 ) ), 0x20 );
					t_mask0  = _mm256_cmp_ps( t_dlam0, t_zeros, 0x01 );
					t_mask1  = _mm256_cmp_ps( t_dt0, t_zeros, 0x01 );
					t_mask2  = _mm256_cmp_ps( t_dlam1, t_zeros, 0x01 );
					t_mask3  = _mm256_cmp_ps( t_dt1, t_zeros, 0x01 );
					t_lam0   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam1 ) ), 0x20 );
					t_t0     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t1 ) ), 0x20 );
					t_lam1   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam3 ) ), 0x20 );
					t_t1     = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t2 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t3 ) ), 0x20 );
					t_lam0    = _mm256_xor_ps( t_lam0, t_sign );
					t_t0      = _mm256_xor_ps( t_t0, t_sign );
					t_lam1    = _mm256_xor_ps( t_lam1, t_sign );
					t_t1      = _mm256_xor_ps( t_t1, t_sign );
					t_tmp0   = _mm256_div_ps( t_lam0, t_dlam0 );
					t_tmp1   = _mm256_div_ps( t_t0, t_dt0 );
					t_tmp2   = _mm256_div_ps( t_lam1, t_dlam1 );
					t_tmp3   = _mm256_div_ps( t_t1, t_dt1 );
					t_mask0  = _mm256_blend_ps( t_zeros, t_mask0, 0x77 );
					t_mask1  = _mm256_blend_ps( t_zeros, t_mask1, 0x77 );
					t_mask2  = _mm256_blend_ps( t_zeros, t_mask2, 0x77 );
					t_mask3  = _mm256_blend_ps( t_zeros, t_mask3, 0x77 );
					t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
					t_tmp1   = _mm256_blendv_ps( t_ones, t_tmp1, t_mask1 );
					t_tmp2   = _mm256_blendv_ps( t_ones, t_tmp2, t_mask2 );
					t_tmp3   = _mm256_blendv_ps( t_ones, t_tmp3, t_mask3 );
					t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );
					t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp1 );
					t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp2 );
					t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp3 );

					}
				else // 1 or 2 left
					{

					s_mask   = _mm256_cvtpd_ps( v_mask );
					s_mask   = _mm_shuffle_ps( s_mask, s_mask, 0x44 );
					t_mask3  = _mm256_permute2f128_ps( _mm256_castps128_ps256( s_mask ), _mm256_castps128_ps256( s_mask ), 0x20 );
					t_mask3  = _mm256_cmp_ps( t_mask3, t_zeros, 0x01 );

					v_dt0    = _mm256_permute2f128_pd( v_dt0, v_dt1, 0x20 );
					v_t0     = _mm256_permute2f128_pd( v_t0, v_t1, 0x20 );
					v_dlam0  = _mm256_permute2f128_pd( v_dlam0, v_dlam1, 0x20 );
					v_lam0   = _mm256_permute2f128_pd( v_lam0, v_lam1, 0x20 );
					v_dt1    = _mm256_permute2f128_pd( v_dt2, v_dt3, 0x20 );
					v_t1     = _mm256_permute2f128_pd( v_t2, v_t3, 0x20 );
					v_dlam1  = _mm256_permute2f128_pd( v_dlam2, v_dlam3, 0x20 );
					v_lam1   = _mm256_permute2f128_pd( v_lam2, v_lam3, 0x20 );

					t_dlam0  = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt0 ) ), 0x20 );
					t_mask0  = _mm256_cmp_ps( t_dlam0, t_zeros, 0x01 );
					t_lam0   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam0 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t0 ) ), 0x20 );
					t_dlam1  = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dlam1 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_dt1 ) ), 0x20 );
					t_mask1  = _mm256_cmp_ps( t_dlam1, t_zeros, 0x01 );
					t_lam1   = _mm256_permute2f128_ps( _mm256_castps128_ps256( _mm256_cvtpd_ps( v_lam1 ) ), _mm256_castps128_ps256( _mm256_cvtpd_ps( v_t1 ) ), 0x20 );
					t_mask0  = _mm256_and_ps( t_mask0, t_mask3 );
					t_mask1  = _mm256_and_ps( t_mask1, t_mask3 );
					t_lam0   = _mm256_xor_ps( t_lam0, t_sign );
					t_lam1   = _mm256_xor_ps( t_lam1, t_sign );
					t_tmp0   = _mm256_div_ps( t_lam0, t_dlam0 );
					t_tmp1   = _mm256_div_ps( t_lam1, t_dlam1 );
					t_tmp0   = _mm256_blendv_ps( t_ones, t_tmp0, t_mask0 );
					t_tmp1   = _mm256_blendv_ps( t_ones, t_tmp1, t_mask1 );
					t_alpha0 = _mm256_min_ps( t_alpha0, t_tmp0 );
					t_alpha1 = _mm256_min_ps( t_alpha1, t_tmp1 );

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



void d_update_var_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double *ptr_mu, double mu_scal, double alpha, double **ux, double **dux, double **t, double **dt, double **lam, double **dlam, double **pi, double **dpi)
	{
	
	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int nu0, nx0, nx1, nb0, pnb, ng0, png, ns0, pns;

	int jj, ll, ll_bkp, ll_end;
	double ll_left;
	
	double d_mask[4] = {0.5, 1.5, 2.5, 3.5};
	
	__m128d
		u_mu0, u_tmp;

	__m256d
		v_mask, v_left, v_zeros,
		v_alpha, v_ux, v_dux, v_pi, v_dpi, 
		v_t0, v_dt0, v_lam0, v_dlam0, v_mu0,
		v_t1, v_dt1, v_lam1, v_dlam1, v_mu1,
		v_t2, v_dt2, v_lam2, v_dlam2, v_mu2,
		v_t3, v_dt3, v_lam3, v_dlam3, v_mu3;
		
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

		ptr_pi   = pi[jj];
		ptr_dpi  = dpi[jj];
		ptr_ux   = ux[jj];
		ptr_dux  = dux[jj];
		ptr_t    = t[jj];
		ptr_dt   = dt[jj];
		ptr_lam  = lam[jj];
		ptr_dlam = dlam[jj];

		nu0 = nu[jj];
		nx0 = nx[jj];
		if(jj<N)
			nx1 = nx[jj+1];
		else
			nx1 = 0;

		// equality constraints
		for(ll=0; ll<nx1-3; ll+=4)
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
			ll_left = nx1 - ll;
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
		nb0 = nb[jj];
		pnb  = bs*((nb0+bs-1)/bs); // cache aligned number of box constraints
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

		ptr_t    += 2*pnb;
		ptr_dt   += 2*pnb;
		ptr_lam  += 2*pnb;
		ptr_dlam += 2*pnb;

		// general constraints
		ng0 = ng[jj];
		png  = bs*((ng0+bs-1)/bs); // cache aligned number of general constraints
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

		ptr_t    += 2*png;
		ptr_dt   += 2*png;
		ptr_lam  += 2*png;
		ptr_dlam += 2*png;

		// box soft constraints
		ns0 = ns[jj];
		pns  = bs*((ns0+bs-1)/bs); // cache aligned number of box soft constraints
		for(ll=0; ll<ns0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[0*pns+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pns+ll] );
			v_t2    = _mm256_load_pd( &ptr_t[2*pns+ll] );
			v_t3    = _mm256_load_pd( &ptr_t[3*pns+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ll] );
			v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ll] );
			v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pns+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pns+ll] );
			v_dt2   = _mm256_load_pd( &ptr_dt[2*pns+ll] );
			v_dt3   = _mm256_load_pd( &ptr_dt[3*pns+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pns+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pns+ll] );
			v_dlam2 = _mm256_load_pd( &ptr_dlam[2*pns+ll] );
			v_dlam3 = _mm256_load_pd( &ptr_dlam[3*pns+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_t2    = _mm256_fmadd_pd( v_alpha, v_dt2, v_t2 );
			v_t3    = _mm256_fmadd_pd( v_alpha, v_dt3, v_t3 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam2  = _mm256_fmadd_pd( v_alpha, v_dlam2, v_lam2 );
			v_lam3  = _mm256_fmadd_pd( v_alpha, v_dlam3, v_lam3 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_dt2   = _mm256_mul_pd( v_alpha, v_dt2 );
			v_t2    = _mm256_add_pd( v_t2, v_dt2 );
			v_dt3   = _mm256_mul_pd( v_alpha, v_dt3 );
			v_t3    = _mm256_add_pd( v_t3, v_dt3 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_dlam2 = _mm256_mul_pd( v_alpha, v_dlam2 );
			v_lam2  = _mm256_add_pd( v_lam2, v_dlam2 );
			v_dlam3 = _mm256_mul_pd( v_alpha, v_dlam3 );
			v_lam3  = _mm256_add_pd( v_lam3, v_dlam3 );
#endif
			_mm256_store_pd( &ptr_t[0*pns+ll], v_t0 );
			_mm256_store_pd( &ptr_t[1*pns+ll], v_t1 );
			_mm256_store_pd( &ptr_t[2*pns+ll], v_t2 );
			_mm256_store_pd( &ptr_t[3*pns+ll], v_t3 );
			_mm256_store_pd( &ptr_lam[0*pns+ll], v_lam0 );
			_mm256_store_pd( &ptr_lam[1*pns+ll], v_lam1 );
			_mm256_store_pd( &ptr_lam[2*pns+ll], v_lam2 );
			_mm256_store_pd( &ptr_lam[3*pns+ll], v_lam3 );
#if defined(TARGET_X64_AVX2)
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
			v_mu0   = _mm256_fmadd_pd( v_lam2, v_t2, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam3, v_t3, v_mu1 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
			v_lam2  = _mm256_mul_pd( v_lam2, v_t2 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam2 );
			v_lam3  = _mm256_mul_pd( v_lam3, v_t3 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam3 );
#endif
			}
		if(ll<ns0)
			{
			ll_left = ns0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );
			i_mask  = _mm256_castpd_si256( v_mask );

			v_t0    = _mm256_load_pd( &ptr_t[0*pns+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pns+ll] );
			v_t2    = _mm256_load_pd( &ptr_t[2*pns+ll] );
			v_t3    = _mm256_load_pd( &ptr_t[3*pns+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ll] );
			v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ll] );
			v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pns+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pns+ll] );
			v_dt2   = _mm256_load_pd( &ptr_dt[2*pns+ll] );
			v_dt3   = _mm256_load_pd( &ptr_dt[3*pns+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pns+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pns+ll] );
			v_dlam2 = _mm256_load_pd( &ptr_dlam[2*pns+ll] );
			v_dlam3 = _mm256_load_pd( &ptr_dlam[3*pns+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_t2    = _mm256_fmadd_pd( v_alpha, v_dt2, v_t2 );
			v_t3    = _mm256_fmadd_pd( v_alpha, v_dt3, v_t3 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam2  = _mm256_fmadd_pd( v_alpha, v_dlam2, v_lam2 );
			v_lam3  = _mm256_fmadd_pd( v_alpha, v_dlam3, v_lam3 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_dt2   = _mm256_mul_pd( v_alpha, v_dt2 );
			v_t2    = _mm256_add_pd( v_t2, v_dt2 );
			v_dt3   = _mm256_mul_pd( v_alpha, v_dt3 );
			v_t3    = _mm256_add_pd( v_t3, v_dt3 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_dlam2 = _mm256_mul_pd( v_alpha, v_dlam2 );
			v_lam2  = _mm256_add_pd( v_lam2, v_dlam2 );
			v_dlam3 = _mm256_mul_pd( v_alpha, v_dlam3 );
			v_lam3  = _mm256_add_pd( v_lam3, v_dlam3 );
#endif
			_mm256_maskstore_pd( &ptr_t[0*pns+ll], i_mask, v_t0 );
			_mm256_maskstore_pd( &ptr_t[1*pns+ll], i_mask, v_t1 );
			_mm256_maskstore_pd( &ptr_t[2*pns+ll], i_mask, v_t2 );
			_mm256_maskstore_pd( &ptr_t[3*pns+ll], i_mask, v_t3 );
			_mm256_maskstore_pd( &ptr_lam[0*pns+ll], i_mask, v_lam0 );
			_mm256_maskstore_pd( &ptr_lam[1*pns+ll], i_mask, v_lam1 );
			_mm256_maskstore_pd( &ptr_lam[2*pns+ll], i_mask, v_lam2 );
			_mm256_maskstore_pd( &ptr_lam[3*pns+ll], i_mask, v_lam3 );
#if defined(TARGET_X64_AVX2)
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
			v_lam2  = _mm256_blendv_pd( v_zeros, v_lam2, v_mask );
			v_lam3  = _mm256_blendv_pd( v_zeros, v_lam3, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam2, v_t2, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam3, v_t3, v_mu1 );
#else
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
			v_lam2  = _mm256_mul_pd( v_lam2, v_t2 );
			v_lam3  = _mm256_mul_pd( v_lam3, v_t3 );
			v_lam2  = _mm256_blendv_pd( v_zeros, v_lam2, v_mask );
			v_lam3  = _mm256_blendv_pd( v_zeros, v_lam3, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam2 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam3 );
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



void d_compute_mu_mpc_soft_tv(int N, int *nx, int *nu, int *nb, int *ng, int *ns, double *ptr_mu, double mu_scal, double alpha, double **lam, double **dlam, double **t, double **dt)
	{
	
	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;

	int jj, ll, ll_bkp, ll_end;
	double ll_left;
	
	double d_mask[4] = {0.5, 1.5, 2.5, 3.5};
	
	__m128d
		u_mu0, u_tmp;

	__m256d
		v_alpha, v_mask, v_left, v_zeros,
		v_t0, v_dt0, v_lam0, v_dlam0, v_mu0, 
		v_t1, v_dt1, v_lam1, v_dlam1, v_mu1,
		v_t2, v_dt2, v_lam2, v_dlam2, v_mu2, 
		v_t3, v_dt3, v_lam3, v_dlam3, v_mu3;
		
	double
		*ptr_t, *ptr_lam, *ptr_dt, *ptr_dlam;

	int nb0, pnb, ng0, png, ns0, pns;
		
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

		ptr_t    += 2*png;
		ptr_lam  += 2*png;
		ptr_dt   += 2*png;
		ptr_dlam += 2*png;

		// box soft constraints
		ns0 = ns[jj];
		pns  = bs*((ns0+bs-1)/bs); // cache aligned number of box soft constraints
		for(ll=0; ll<ns0-3; ll+=4)
			{
			v_t0    = _mm256_load_pd( &ptr_t[0*pns+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pns+ll] );
			v_t2    = _mm256_load_pd( &ptr_t[2*pns+ll] );
			v_t3    = _mm256_load_pd( &ptr_t[3*pns+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ll] );
			v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ll] );
			v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pns+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pns+ll] );
			v_dt2   = _mm256_load_pd( &ptr_dt[2*pns+ll] );
			v_dt3   = _mm256_load_pd( &ptr_dt[3*pns+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pns+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pns+ll] );
			v_dlam2 = _mm256_load_pd( &ptr_dlam[2*pns+ll] );
			v_dlam3 = _mm256_load_pd( &ptr_dlam[3*pns+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_t2    = _mm256_fmadd_pd( v_alpha, v_dt2, v_t2 );
			v_t3    = _mm256_fmadd_pd( v_alpha, v_dt3, v_t3 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam2  = _mm256_fmadd_pd( v_alpha, v_dlam2, v_lam2 );
			v_lam3  = _mm256_fmadd_pd( v_alpha, v_dlam3, v_lam3 );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
			v_mu0   = _mm256_fmadd_pd( v_lam2, v_t2, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam3, v_t3, v_mu1 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_dt2   = _mm256_mul_pd( v_alpha, v_dt2 );
			v_t2    = _mm256_add_pd( v_t2, v_dt2 );
			v_dt3   = _mm256_mul_pd( v_alpha, v_dt3 );
			v_t3    = _mm256_add_pd( v_t3, v_dt3 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_dlam2 = _mm256_mul_pd( v_alpha, v_dlam2 );
			v_lam2  = _mm256_add_pd( v_lam2, v_dlam2 );
			v_dlam3 = _mm256_mul_pd( v_alpha, v_dlam3 );
			v_lam3  = _mm256_add_pd( v_lam3, v_dlam3 );
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
			v_lam2  = _mm256_mul_pd( v_lam2, v_t2 );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam2 );
			v_lam3  = _mm256_mul_pd( v_lam3, v_t3 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam3 );
#endif
			}
		if(ll<ns0)
			{
			ll_left = ns0-ll;
			v_left  = _mm256_broadcast_sd( &ll_left );
			v_mask  = _mm256_loadu_pd( d_mask );
			v_mask  = _mm256_sub_pd( v_mask, v_left );

			v_t0    = _mm256_load_pd( &ptr_t[0*pns+ll] );
			v_t1    = _mm256_load_pd( &ptr_t[1*pns+ll] );
			v_t2    = _mm256_load_pd( &ptr_t[2*pns+ll] );
			v_t3    = _mm256_load_pd( &ptr_t[3*pns+ll] );
			v_lam0  = _mm256_load_pd( &ptr_lam[0*pns+ll] );
			v_lam1  = _mm256_load_pd( &ptr_lam[1*pns+ll] );
			v_lam2  = _mm256_load_pd( &ptr_lam[2*pns+ll] );
			v_lam3  = _mm256_load_pd( &ptr_lam[3*pns+ll] );
			v_dt0   = _mm256_load_pd( &ptr_dt[0*pns+ll] );
			v_dt1   = _mm256_load_pd( &ptr_dt[1*pns+ll] );
			v_dt2   = _mm256_load_pd( &ptr_dt[2*pns+ll] );
			v_dt3   = _mm256_load_pd( &ptr_dt[3*pns+ll] );
			v_dlam0 = _mm256_load_pd( &ptr_dlam[0*pns+ll] );
			v_dlam1 = _mm256_load_pd( &ptr_dlam[1*pns+ll] );
			v_dlam2 = _mm256_load_pd( &ptr_dlam[2*pns+ll] );
			v_dlam3 = _mm256_load_pd( &ptr_dlam[3*pns+ll] );
#if defined(TARGET_X64_AVX2)
			v_t0    = _mm256_fmadd_pd( v_alpha, v_dt0, v_t0 );
			v_t1    = _mm256_fmadd_pd( v_alpha, v_dt1, v_t1 );
			v_t2    = _mm256_fmadd_pd( v_alpha, v_dt2, v_t2 );
			v_t3    = _mm256_fmadd_pd( v_alpha, v_dt3, v_t3 );
			v_lam0  = _mm256_fmadd_pd( v_alpha, v_dlam0, v_lam0 );
			v_lam1  = _mm256_fmadd_pd( v_alpha, v_dlam1, v_lam1 );
			v_lam2  = _mm256_fmadd_pd( v_alpha, v_dlam2, v_lam2 );
			v_lam3  = _mm256_fmadd_pd( v_alpha, v_dlam3, v_lam3 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam0, v_t0, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam1, v_t1, v_mu1 );
			v_lam2  = _mm256_blendv_pd( v_zeros, v_lam2, v_mask );
			v_lam3  = _mm256_blendv_pd( v_zeros, v_lam3, v_mask );
			v_mu0   = _mm256_fmadd_pd( v_lam2, v_t2, v_mu0 );
			v_mu1   = _mm256_fmadd_pd( v_lam3, v_t3, v_mu1 );
#else
			v_dt0   = _mm256_mul_pd( v_alpha, v_dt0 );
			v_t0    = _mm256_add_pd( v_t0, v_dt0 );
			v_dt1   = _mm256_mul_pd( v_alpha, v_dt1 );
			v_t1    = _mm256_add_pd( v_t1, v_dt1 );
			v_dt2   = _mm256_mul_pd( v_alpha, v_dt2 );
			v_t2    = _mm256_add_pd( v_t2, v_dt2 );
			v_dt3   = _mm256_mul_pd( v_alpha, v_dt3 );
			v_t3    = _mm256_add_pd( v_t3, v_dt3 );
			v_dlam0 = _mm256_mul_pd( v_alpha, v_dlam0 );
			v_lam0  = _mm256_add_pd( v_lam0, v_dlam0 );
			v_dlam1 = _mm256_mul_pd( v_alpha, v_dlam1 );
			v_lam1  = _mm256_add_pd( v_lam1, v_dlam1 );
			v_dlam2 = _mm256_mul_pd( v_alpha, v_dlam2 );
			v_lam2  = _mm256_add_pd( v_lam2, v_dlam2 );
			v_dlam3 = _mm256_mul_pd( v_alpha, v_dlam3 );
			v_lam3  = _mm256_add_pd( v_lam3, v_dlam3 );
			v_lam0  = _mm256_mul_pd( v_lam0, v_t0 );
			v_lam1  = _mm256_mul_pd( v_lam1, v_t1 );
			v_lam0  = _mm256_blendv_pd( v_zeros, v_lam0, v_mask );
			v_lam1  = _mm256_blendv_pd( v_zeros, v_lam1, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam0 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam1 );
			v_lam2  = _mm256_mul_pd( v_lam2, v_t2 );
			v_lam3  = _mm256_mul_pd( v_lam3, v_t3 );
			v_lam2  = _mm256_blendv_pd( v_zeros, v_lam2, v_mask );
			v_lam3  = _mm256_blendv_pd( v_zeros, v_lam3, v_mask );
			v_mu0   = _mm256_add_pd( v_mu0, v_lam2 );
			v_mu1   = _mm256_add_pd( v_mu1, v_lam3 );
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




