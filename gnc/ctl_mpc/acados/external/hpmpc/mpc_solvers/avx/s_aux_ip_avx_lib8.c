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

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX

/*#include "../include/block_size.h"*/



void s_init_ux_pi_t_box_mpc(int N, int nx, int nu, int nbu, int nb, float **ux, float **pi, float **db, float **t, int warm_start)
	{
	
	int jj, ll, ii;
	
	float thr0 = 1e-3; // minimum distance from a constraint

	if(warm_start==1)
		{
		for(ll=0; ll<2*nbu; ll+=2)
			{
			t[0][ll+0] =   ux[0][ll/2] - db[0][ll+0];
			t[0][ll+1] = - db[0][ll+1] - ux[0][ll/2];
			if(t[0][ll+0] < thr0)
				{
				if(t[0][ll+1] < thr0)
					{
					ux[0][ll/2] = ( - db[0][ll+1] + db[0][ll+0])*0.5;
					t[0][ll+0] =   ux[0][ll/2] - db[0][ll+0];
					t[0][ll+1] = - db[0][ll+1] - ux[0][ll/2];
					}
				else
					{
					t[0][ll+0] = thr0;
					ux[0][ll/2] = db[0][ll+0] + thr0;
					}
				}
			else if(t[0][ll+1] < thr0)
				{
				t[0][ll+1] = thr0;
				ux[0][ll/2] = - db[0][ll+1] - thr0;
				}
			}
		for(; ll<2*nb; ll++)
			t[0][ll] = 1.0; // this has to be strictly positive !!!
		for(jj=1; jj<N; jj++)
			{
			for(ll=0; ll<2*nb; ll+=2)
				{
				t[jj][ll+0] = ux[jj][ll/2] - db[jj][ll+0];
				t[jj][ll+1] = - db[jj][ll+1] - ux[jj][ll/2];
				if(t[jj][ll+0] < thr0)
					{
					if(t[jj][ll+1] < thr0)
						{
						ux[jj][ll/2] = ( - db[jj][ll+1] + db[jj][ll+0])*0.5;
						t[jj][ll+0] =   ux[jj][ll/2] - db[jj][ll+0];
						t[jj][ll+1] = - db[jj][ll+1] - ux[jj][ll/2];
						}
					else
						{
						t[jj][ll+0] = thr0;
						ux[jj][ll/2] = db[jj][ll+0] + thr0;
						}
					}
				else if(t[jj][ll+1] < thr0)
					{
					t[jj][ll+1] = thr0;
					ux[jj][ll/2] = - db[jj][ll+1] - thr0;
					}
				}
			}
		for(ll=0; ll<2*nbu; ll++) // this has to be strictly positive !!!
			t[N][ll] = 1;
		for(ll=2*nu; ll<2*nb; ll+=2)
			{
			t[N][ll+0] =   ux[N][ll/2] - db[N][ll+0];
			t[N][ll+1] = - db[N][ll+1] - ux[N][ll/2];
			if(t[N][ll+0] < thr0)
				{
				if(t[N][ll+1] < thr0)
					{
					ux[N][ll/2] = ( - db[N][ll+1] + db[N][ll+0])*0.5;
					t[N][ll+0] =   ux[N][ll/2] - db[N][ll+0];
					t[N][ll+1] = - db[N][ll+1] - ux[N][ll/2];
					}
				else
					{
					t[N][ll+0] = thr0;
					ux[N][ll/2] = db[N][ll+0] + thr0;
					}
				}
			else if(t[N][ll+1] < thr0)
				{
				t[N][ll+1] = thr0;
				ux[N][ll/2] = - db[N][ll+1] - thr0;
				}
			}

		}
	else // cold start
		{
		for(ll=0; ll<2*nbu; ll+=2)
			{
			ux[0][ll/2] = 0.0;
/*			t[0][ll+0] = 1.0;*/
/*			t[0][ll+1] = 1.0;*/
			t[0][ll+0] =   ux[0][ll/2] - db[0][ll+0];
			t[0][ll+1] = - db[0][ll+1] - ux[0][ll/2];
			if(t[0][ll+0] < thr0)
				{
				if(t[0][ll+1] < thr0)
					{
					ux[0][ll/2] = ( - db[0][ll+1] + db[0][ll+0])*0.5;
					t[0][ll+0] =   ux[0][ll/2] - db[0][ll+0];
					t[0][ll+1] = - db[0][ll+1] - ux[0][ll/2];
					}
				else
					{
					t[0][ll+0] = thr0;
					ux[0][ll/2] = db[0][ll+0] + thr0;
					}
				}
			else if(t[0][ll+1] < thr0)
				{
				t[0][ll+1] = thr0;
				ux[0][ll/2] = - db[0][ll+1] - thr0;
				}
			}
		for(ii=ll/2; ii<nu; ii++)
			ux[0][ii] = 0.0; // initialize remaining components of u to zero
		for(; ll<2*nb; ll++)
			t[0][ll] = 1.0; // this has to be strictly positive !!!
		for(jj=1; jj<N; jj++)
			{
			for(ll=0; ll<2*nb; ll+=2)
				{
				ux[jj][ll/2] = 0.0;
/*				t[jj][ll+0] = 1.0;*/
/*				t[jj][ll+1] = 1.0;*/
				t[jj][ll+0] =   ux[jj][ll/2] - db[jj][ll+0];
				t[jj][ll+1] = - db[jj][ll+1] - ux[jj][ll/2];
				if(t[jj][ll+0] < thr0)
					{
					if(t[jj][ll+1] < thr0)
						{
						ux[jj][ll/2] = ( - db[jj][ll+1] + db[jj][ll+0])*0.5;
						t[jj][ll+0] =   ux[jj][ll/2] - db[jj][ll+0];
						t[jj][ll+1] = - db[jj][ll+1] - ux[jj][ll/2];
						}
					else
						{
						t[jj][ll+0] = thr0;
						ux[jj][ll/2] = db[jj][ll+0] + thr0;
						}
					}
				else if(t[jj][ll+1] < thr0)
					{
					t[jj][ll+1] = thr0;
					ux[jj][ll/2] = - db[jj][ll+1] - thr0;
					}
				}
			for(ii=ll/2; ii<nx+nu; ii++)
				ux[jj][ii] = 0.0; // initialize remaining components of u and x to zero
			}
		for(ll=0; ll<2*nbu; ll++)
			t[N][ll] = 1.0; // this has to be strictly positive !!!
		for(ll=2*nu; ll<2*nb; ll+=2)
			{
			ux[N][ll/2] = 0.0;
/*			t[N][ll+0] = 1.0;*/
/*			t[N][ll+1] = 1.0;*/
			t[N][ll+0] =   ux[N][ll/2] - db[N][ll+0];
			t[N][ll+1] = - db[N][ll+1] - ux[N][ll/2];
			if(t[N][ll+0] < thr0)
				{
				if(t[N][ll+1] < thr0)
					{
					ux[N][ll/2] = ( - db[N][ll+1] + db[N][ll+0])*0.5;
					t[N][ll+0] =   ux[N][ll/2] - db[N][ll+0];
					t[N][ll+1] = - db[N][ll+1] - ux[N][ll/2];
					}
				else
					{
					t[N][ll+0] = thr0;
					ux[N][ll/2] = db[N][ll+0] + thr0;
					}
				}
			else if(t[N][ll+1] < thr0)
				{
				t[N][ll+1] = thr0;
				ux[N][ll/2] = - db[N][ll+1] - thr0;
				}
			}
		for(ii=ll/2; ii<nx+nu; ii++)
			ux[N][ii] = 0.0; // initialize remaining components of x to zero

		for(jj=0; jj<=N; jj++)
			for(ll=0; ll<nx; ll++)
				pi[jj][ll] = 0.0; // initialize multipliers to zero

		}
	
	}



void s_init_lam_mpc(int N, int nu, int nbu, int nb, float **t, float **lam)	// TODO approximate reciprocal
	{
	
	int jj, ll;
	
	for(ll=0; ll<2*nbu; ll++)
		lam[0][ll] = 1/t[0][ll];
	for(; ll<2*nb; ll++)
		lam[0][ll] = 1.0; // this has to be strictly positive !!!
	for(jj=1; jj<N; jj++)
		{
		for(ll=0; ll<2*nb; ll++)
			lam[jj][ll] = 1/t[jj][ll];
/*			lam[jj][ll] = thr0/t[jj][ll];*/
		}
	for(ll=0; ll<2*nu; ll++)
		lam[N][ll] = 1.0; // this has to be strictly positive !!!
	for(ll=2*nu; ll<2*nb; ll++)
		lam[N][ll] = 1/t[jj][ll];
/*		lam[N][ll] = thr0/t[jj][ll];*/
	
	}



void s_update_hessian_box_mpc(int N, int k0, int k1, int kmax, int cnz, float sigma_mu, float **t, float **t_inv, float **lam, float **lamt, float **dlam, float **bd, float **bl, float **pd, float **pl, float **pl2, float **db)

/*void d_update_hessian_box(int k0, int kmax, int nb, int cnz, float sigma_mu, float *t, float *lam, float *lamt, float *dlam, float *bd, float *bl, float *pd, float *pl, float *lb, float *ub)*/
	{
	
	__m256
		v_ones, v_sigma_mu,
		v_tmp, v_lam, v_lamt, v_dlam, v_db;
		
	__m128
		u_tmp, u_lamt, u_bd, u_bl, u_lam, u_dlam, u_db;
	
	v_ones = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );
	v_sigma_mu = _mm256_set_ps( sigma_mu, sigma_mu, sigma_mu, sigma_mu, sigma_mu, sigma_mu, sigma_mu, sigma_mu );
	
	const int bs = 8; //s_get_mr();
	
	float temp0, temp1;
	
	float *ptr_t, *ptr_lam, *ptr_lamt, *ptr_dlam, *ptr_t_inv, *ptr_pd, *ptr_pl, *ptr_pl2, *ptr_bd, *ptr_bl, *ptr_db;
	
	int ii, jj, ll, bs0;



	// first stage

	ptr_t     = t[0];
	ptr_lam   = lam[0];
	ptr_lamt  = lamt[0];
	ptr_dlam  = dlam[0];
	ptr_t_inv  = t_inv[0];
	ptr_pd    = pd[0];
	ptr_pl    = pl[0];
	ptr_pl2   = pl2[0];
	ptr_bd    = bd[0];
	ptr_bl    = bl[0];
	ptr_db    = db[0];
	
	ii = 0;
	for(; ii<k0-7; ii+=8)
		{

		v_tmp  = _mm256_load_ps( &ptr_t[0] );
		v_tmp  = _mm256_div_ps( v_ones, v_tmp );
		_mm256_store_ps( &ptr_t_inv[0], v_tmp ); // store t_inv
		v_lam  = _mm256_load_ps( &ptr_lam[0] );
		v_lamt = _mm256_mul_ps( v_tmp, v_lam );
		_mm256_store_ps( &ptr_lamt[0], v_lamt );
		v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
		_mm256_store_ps( &ptr_dlam[0], v_dlam );
		u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
		u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] , lamt[4]+lamt[5] , lamt[6]+lamt[7] ]
		u_bd   = _mm_load_ps( &bd[0][ii] );
		u_bd   = _mm_add_ps( u_bd, u_lamt );
		_mm_store_ss( &pd[0][0+(ii+0)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
		_mm_store_ss( &pd[0][1+(ii+1)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
		_mm_store_ss( &pd[0][2+(ii+2)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
		_mm_store_ss( &pd[0][3+(ii+3)*bs+ii*cnz], u_bd );
		v_db   = _mm256_load_ps( &db[0][2*ii+0] );
		v_db   = _mm256_mul_ps( v_db, v_lamt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		v_lam  = _mm256_add_ps( v_lam, v_db );
		u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
		u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] , ... ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] , ... ]
		u_bl   = _mm_load_ps( &bl[0][ii] );
		u_bl   = _mm_sub_ps( u_bl, u_lam );
		_mm_store_ps( &pl2[0][ii+0], u_bl );
		_mm_store_ss( &pl[0][(ii+0)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
		_mm_store_ss( &pl[0][(ii+1)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
		_mm_store_ss( &pl[0][(ii+2)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
		_mm_store_ss( &pl[0][(ii+3)*bs], u_bl );

		v_tmp  = _mm256_load_ps( &ptr_t[8] );
		v_tmp  = _mm256_div_ps( v_ones, v_tmp );
		_mm256_store_ps( &ptr_t_inv[8], v_tmp ); // store t_inv
		v_lam  = _mm256_load_ps( &ptr_lam[8] );
		v_lamt = _mm256_mul_ps( v_tmp, v_lam );
		_mm256_store_ps( &ptr_lamt[8], v_lamt );
		v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
		_mm256_store_ps( &ptr_dlam[8], v_dlam );
		u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
		u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] ]
		u_bd   = _mm_load_ps( &bd[0][ii+4] );
		u_bd   = _mm_add_ps( u_bd, u_lamt );
		_mm_store_ss( &pd[0][4+(ii+4)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
		_mm_store_ss( &pd[0][5+(ii+5)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
		_mm_store_ss( &pd[0][6+(ii+6)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
		_mm_store_ss( &pd[0][7+(ii+7)*bs+ii*cnz], u_bd );
		v_db   = _mm256_load_ps( &db[0][2*ii+8] );
		v_db   = _mm256_mul_ps( v_db, v_lamt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		v_lam  = _mm256_add_ps( v_lam, v_db );
		u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
		u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] ]
		u_bl   = _mm_load_ps( &bl[0][ii+4] );
		u_bl   = _mm_sub_ps( u_bl, u_lam );
		_mm_store_ps( &pl2[0][ii+4], u_bl );
		_mm_store_ss( &pl[0][(ii+4)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
		_mm_store_ss( &pl[0][(ii+5)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
		_mm_store_ss( &pl[0][(ii+6)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
		_mm_store_ss( &pl[0][(ii+7)*bs], u_bl );

		ptr_t     += 16;
		ptr_lam   += 16;
		ptr_lamt  += 16;
		ptr_dlam  += 16;
		ptr_t_inv  += 16;

		}
	if(ii<k0)
		{
		bs0 = k0-ii;
		ll=0;
		for(; ll<bs0-3; ll+=4)
			{

			v_tmp  = _mm256_load_ps( &ptr_t[0] );
			v_tmp  = _mm256_div_ps( v_ones, v_tmp );
			_mm256_store_ps( &ptr_t_inv[0], v_tmp ); // store t_inv
			v_lam  = _mm256_load_ps( &ptr_lam[0] );
			v_lamt = _mm256_mul_ps( v_tmp, v_lam );
			_mm256_store_ps( &ptr_lamt[0], v_lamt );
			v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
			_mm256_store_ps( &ptr_dlam[0], v_dlam );
			u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
			u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] , lamt[4]+lamt[5] , lamt[6]+lamt[7] ]
			u_bd   = _mm_load_ps( &bd[0][ii] );
			u_bd   = _mm_add_ps( u_bd, u_lamt );
			_mm_store_ss( &pd[0][0+(ii+0)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
			_mm_store_ss( &pd[0][1+(ii+1)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
			_mm_store_ss( &pd[0][2+(ii+2)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
			_mm_store_ss( &pd[0][3+(ii+3)*bs+ii*cnz], u_bd );
			v_db   = _mm256_load_ps( &db[0][2*ii+0] );
			v_db   = _mm256_mul_ps( v_db, v_lamt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
			v_lam  = _mm256_add_ps( v_lam, v_db );
			u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
			u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] , ... ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] , ... ]
			u_bl   = _mm_load_ps( &bl[0][ii] );
			u_bl   = _mm_sub_ps( u_bl, u_lam );
			_mm_store_ps( &pl2[0][ii+0], u_bl );
			_mm_store_ss( &pl[0][(ii+0)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
			_mm_store_ss( &pl[0][(ii+1)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
			_mm_store_ss( &pl[0][(ii+2)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
			_mm_store_ss( &pl[0][(ii+3)*bs], u_bl );

			ptr_t     += 8;
			ptr_lam   += 8;
			ptr_lamt  += 8;
			ptr_dlam  += 8;
			ptr_t_inv  += 8;

			}
		for(; ll<bs0; ll++)
			{

			ptr_t_inv[0] = 1.0/ptr_t[0];
			ptr_t_inv[1] = 1.0/ptr_t[1];
			ptr_lamt[0] = ptr_lam[0]*ptr_t_inv[0];
			ptr_lamt[1] = ptr_lam[1]*ptr_t_inv[1];
			ptr_dlam[0] = ptr_t_inv[0]*sigma_mu; // !!!!!
			ptr_dlam[1] = ptr_t_inv[1]*sigma_mu; // !!!!!
			ptr_pd[ll+(ii+ll)*bs+ii*cnz] = ptr_bd[ii+ll] + ptr_lamt[0] + ptr_lamt[1];
			ptr_pl[(ii+ll)*bs] = ptr_bl[ii+ll] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+2*ll+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+2*ll+0] - ptr_dlam[0];
			ptr_pl2[ii+ll+0] = ptr_pl[(ii+ll)*bs];

			ptr_t     += 2;
			ptr_lam   += 2;
			ptr_lamt  += 2;
			ptr_dlam  += 2;
			ptr_t_inv  += 2;

			}
		}

	// middle stages

	for(jj=1; jj<N; jj++)
		{
		
		ptr_t     = t[jj];
		ptr_lam   = lam[jj];
		ptr_lamt  = lamt[jj];
		ptr_dlam  = dlam[jj];
		ptr_t_inv  = t_inv[jj];
		ptr_pd    = pd[jj];
		ptr_pl    = pl[jj];
		ptr_pl2   = pl2[jj];
		ptr_bd    = bd[jj];
		ptr_bl    = bl[jj];
		ptr_db    = db[jj];

		ii = 0;
		for(; ii<kmax-7; ii+=8)
			{

			v_tmp  = _mm256_load_ps( &ptr_t[0] );
			v_tmp  = _mm256_div_ps( v_ones, v_tmp );
			_mm256_store_ps( &ptr_t_inv[0], v_tmp ); // store t_inv
			v_lam  = _mm256_load_ps( &ptr_lam[0] );
			v_lamt = _mm256_mul_ps( v_tmp, v_lam );
			_mm256_store_ps( &ptr_lamt[0], v_lamt );
			v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
			_mm256_store_ps( &ptr_dlam[0], v_dlam );
			u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
			u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] , lamt[4]+lamt[5] , lamt[6]+lamt[7] ]
			u_bd   = _mm_load_ps( &bd[jj][ii] );
			u_bd   = _mm_add_ps( u_bd, u_lamt );
			_mm_store_ss( &pd[jj][0+(ii+0)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
			_mm_store_ss( &pd[jj][1+(ii+1)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
			_mm_store_ss( &pd[jj][2+(ii+2)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
			_mm_store_ss( &pd[jj][3+(ii+3)*bs+ii*cnz], u_bd );
			v_db   = _mm256_load_ps( &db[jj][2*ii+0] );
			v_db   = _mm256_mul_ps( v_db, v_lamt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
			v_lam  = _mm256_add_ps( v_lam, v_db );
			u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
			u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] , ... ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] , ... ]
			u_bl   = _mm_load_ps( &bl[jj][ii] );
			u_bl   = _mm_sub_ps( u_bl, u_lam );
			_mm_store_ps( &pl2[jj][ii+0], u_bl );
			_mm_store_ss( &pl[jj][(ii+0)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
			_mm_store_ss( &pl[jj][(ii+1)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
			_mm_store_ss( &pl[jj][(ii+2)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
			_mm_store_ss( &pl[jj][(ii+3)*bs], u_bl );

			v_tmp  = _mm256_load_ps( &ptr_t[8] );
			v_tmp  = _mm256_div_ps( v_ones, v_tmp );
			_mm256_store_ps( &ptr_t_inv[8], v_tmp ); // store t_inv
			v_lam  = _mm256_load_ps( &ptr_lam[8] );
			v_lamt = _mm256_mul_ps( v_tmp, v_lam );
			_mm256_store_ps( &ptr_lamt[8], v_lamt );
			v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
			_mm256_store_ps( &ptr_dlam[8], v_dlam );
			u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
			u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] ]
			u_bd   = _mm_load_ps( &bd[jj][ii+4] );
			u_bd   = _mm_add_ps( u_bd, u_lamt );
			_mm_store_ss( &pd[jj][4+(ii+4)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
			_mm_store_ss( &pd[jj][5+(ii+5)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
			_mm_store_ss( &pd[jj][6+(ii+6)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
			_mm_store_ss( &pd[jj][7+(ii+7)*bs+ii*cnz], u_bd );
			v_db   = _mm256_load_ps( &db[jj][2*ii+8] );
			v_db   = _mm256_mul_ps( v_db, v_lamt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
			v_lam  = _mm256_add_ps( v_lam, v_db );
			u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
			u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] ]
			u_bl   = _mm_load_ps( &bl[jj][ii+4] );
			u_bl   = _mm_sub_ps( u_bl, u_lam );
			_mm_store_ps( &pl2[jj][ii+4], u_bl );
			_mm_store_ss( &pl[jj][(ii+4)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
			_mm_store_ss( &pl[jj][(ii+5)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
			_mm_store_ss( &pl[jj][(ii+6)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
			_mm_store_ss( &pl[jj][(ii+7)*bs], u_bl );

			ptr_t     += 16;
			ptr_lam   += 16;
			ptr_lamt  += 16;
			ptr_dlam  += 16;
			ptr_t_inv  += 16;

			}
		if(ii<kmax)
			{
			bs0 = kmax-ii;
			ll = 0;
			for(; ll<bs0-3; ll+=4)
				{

				v_tmp  = _mm256_load_ps( &ptr_t[0] );
				v_tmp  = _mm256_div_ps( v_ones, v_tmp );
				_mm256_store_ps( &ptr_t_inv[0], v_tmp ); // store t_inv
				v_lam  = _mm256_load_ps( &ptr_lam[0] );
				v_lamt = _mm256_mul_ps( v_tmp, v_lam );
				_mm256_store_ps( &ptr_lamt[0], v_lamt );
				v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
				_mm256_store_ps( &ptr_dlam[0], v_dlam );
				u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
				u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] , lamt[4]+lamt[5] , lamt[6]+lamt[7] ]
				u_bd   = _mm_load_ps( &bd[jj][ii] );
				u_bd   = _mm_add_ps( u_bd, u_lamt );
				_mm_store_ss( &pd[jj][0+(ii+0)*bs+ii*cnz], u_bd );
				u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
				_mm_store_ss( &pd[jj][1+(ii+1)*bs+ii*cnz], u_bd );
				u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
				_mm_store_ss( &pd[jj][2+(ii+2)*bs+ii*cnz], u_bd );
				u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
				_mm_store_ss( &pd[jj][3+(ii+3)*bs+ii*cnz], u_bd );
				v_db   = _mm256_load_ps( &db[jj][2*ii+0] );
				v_db   = _mm256_mul_ps( v_db, v_lamt );
				v_lam  = _mm256_add_ps( v_lam, v_dlam );
				v_lam  = _mm256_add_ps( v_lam, v_db );
				u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
				u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] , ... ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] , ... ]
				u_bl   = _mm_load_ps( &bl[jj][ii] );
				u_bl   = _mm_sub_ps( u_bl, u_lam );
				_mm_store_ps( &pl2[jj][ii+0], u_bl );
				_mm_store_ss( &pl[jj][(ii+0)*bs], u_bl );
				u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
				_mm_store_ss( &pl[jj][(ii+1)*bs], u_bl );
				u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
				_mm_store_ss( &pl[jj][(ii+2)*bs], u_bl );
				u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
				_mm_store_ss( &pl[jj][(ii+3)*bs], u_bl );

				ptr_t     += 8;
				ptr_lam   += 8;
				ptr_lamt  += 8;
				ptr_dlam  += 8;
				ptr_t_inv  += 8;

				}
			for(; ll<bs0; ll++)
				{

				ptr_t_inv[0] = 1.0/ptr_t[0];
				ptr_t_inv[1] = 1.0/ptr_t[1];
				ptr_lamt[0] = ptr_lam[0]*ptr_t_inv[0];
				ptr_lamt[1] = ptr_lam[1]*ptr_t_inv[1];
				ptr_dlam[0] = ptr_t_inv[0]*sigma_mu; // !!!!!
				ptr_dlam[1] = ptr_t_inv[1]*sigma_mu; // !!!!!
				ptr_pd[ll+(ii+ll)*bs+ii*cnz] = ptr_bd[ii+ll] + ptr_lamt[0] + ptr_lamt[1];
				ptr_pl[(ii+ll)*bs] = ptr_bl[ii+ll] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+2*ll+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+2*ll+0] - ptr_dlam[0];
				ptr_pl2[ii+ll+0] = ptr_pl[(ii+ll)*bs];

				ptr_t     += 2;
				ptr_lam   += 2;
				ptr_lamt  += 2;
				ptr_dlam  += 2;
				ptr_t_inv  += 2;

				}
			}
	
		}

	// last stage

	ptr_t     = t[N]     + 2*k1;
	ptr_lam   = lam[N]   + 2*k1;
	ptr_lamt  = lamt[N]  + 2*k1;
	ptr_dlam  = dlam[N]  + 2*k1;
	ptr_t_inv  = t_inv[N] + 2*k1;
	ptr_pd    = pd[N];
	ptr_pl    = pl[N];
	ptr_pl2   = pl2[N];
	ptr_bd    = bd[N];
	ptr_bl    = bl[N];
	ptr_db    = db[N];

	ii=k1; // k1 supposed to be multiple of bs !!!!!!!!!!

	for(; ii<kmax-7; ii+=8)
		{

		v_tmp  = _mm256_load_ps( &ptr_t[0] );
		v_tmp  = _mm256_div_ps( v_ones, v_tmp );
		_mm256_store_ps( &ptr_t_inv[0], v_tmp ); // store t_inv
		v_lam  = _mm256_load_ps( &ptr_lam[0] );
		v_lamt = _mm256_mul_ps( v_tmp, v_lam );
		_mm256_store_ps( &ptr_lamt[0], v_lamt );
		v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
		_mm256_store_ps( &ptr_dlam[0], v_dlam );
		u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
		u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] , lamt[4]+lamt[5] , lamt[6]+lamt[7] ]
		u_bd   = _mm_load_ps( &bd[N][ii] );
		u_bd   = _mm_add_ps( u_bd, u_lamt );
		_mm_store_ss( &pd[N][0+(ii+0)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
		_mm_store_ss( &pd[N][1+(ii+1)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
		_mm_store_ss( &pd[N][2+(ii+2)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
		_mm_store_ss( &pd[N][3+(ii+3)*bs+ii*cnz], u_bd );
		v_db   = _mm256_load_ps( &db[N][2*ii+0] );
		v_db   = _mm256_mul_ps( v_db, v_lamt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		v_lam  = _mm256_add_ps( v_lam, v_db );
		u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
		u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] , ... ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] , ... ]
		u_bl   = _mm_load_ps( &bl[N][ii] );
		u_bl   = _mm_sub_ps( u_bl, u_lam );
		_mm_store_ps( &pl2[N][ii+0], u_bl );
		_mm_store_ss( &pl[N][(ii+0)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
		_mm_store_ss( &pl[N][(ii+1)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
		_mm_store_ss( &pl[N][(ii+2)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
		_mm_store_ss( &pl[N][(ii+3)*bs], u_bl );

		v_tmp  = _mm256_load_ps( &ptr_t[8] );
		v_tmp  = _mm256_div_ps( v_ones, v_tmp );
		_mm256_store_ps( &ptr_t_inv[8], v_tmp ); // store t_inv
		v_lam  = _mm256_load_ps( &ptr_lam[8] );
		v_lamt = _mm256_mul_ps( v_tmp, v_lam );
		_mm256_store_ps( &ptr_lamt[8], v_lamt );
		v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
		_mm256_store_ps( &ptr_dlam[8], v_dlam );
		u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
		u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] ]
		u_bd   = _mm_load_ps( &bd[N][ii+4] );
		u_bd   = _mm_add_ps( u_bd, u_lamt );
		_mm_store_ss( &pd[N][4+(ii+4)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
		_mm_store_ss( &pd[N][5+(ii+5)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
		_mm_store_ss( &pd[N][6+(ii+6)*bs+ii*cnz], u_bd );
		u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
		_mm_store_ss( &pd[N][7+(ii+7)*bs+ii*cnz], u_bd );
		v_db   = _mm256_load_ps( &db[N][2*ii+8] );
		v_db   = _mm256_mul_ps( v_db, v_lamt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		v_lam  = _mm256_add_ps( v_lam, v_db );
		u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
		u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] ]
		u_bl   = _mm_load_ps( &bl[N][ii+4] );
		u_bl   = _mm_sub_ps( u_bl, u_lam );
		_mm_store_ps( &pl2[N][ii+4], u_bl );
		_mm_store_ss( &pl[N][(ii+4)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
		_mm_store_ss( &pl[N][(ii+5)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
		_mm_store_ss( &pl[N][(ii+6)*bs], u_bl );
		u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
		_mm_store_ss( &pl[N][(ii+7)*bs], u_bl );

		ptr_t     += 16;
		ptr_lam   += 16;
		ptr_lamt  += 16;
		ptr_dlam  += 16;
		ptr_t_inv  += 16;

		}
	if(ii<kmax)
		{
		bs0 = kmax-ii;
		ll=0;
		for(; ll<bs0-3; ll+=4)
			{

			v_tmp  = _mm256_load_ps( &ptr_t[0] );
			v_tmp  = _mm256_div_ps( v_ones, v_tmp );
			_mm256_store_ps( &ptr_t_inv[0], v_tmp ); // store t_inv
			v_lam  = _mm256_load_ps( &ptr_lam[0] );
			v_lamt = _mm256_mul_ps( v_tmp, v_lam );
			_mm256_store_ps( &ptr_lamt[0], v_lamt );
			v_dlam = _mm256_mul_ps( v_tmp, v_sigma_mu );
			_mm256_store_ps( &ptr_dlam[0], v_dlam );
			u_lamt = _mm256_extractf128_ps( v_lamt, 0x1 );
			u_lamt = _mm_hadd_ps( _mm256_castps256_ps128( v_lamt ), u_lamt ); // [ lamt[0]+lamt[1] , lamt[2]+lamt[3] , lamt[4]+lamt[5] , lamt[6]+lamt[7] ]
			u_bd   = _mm_load_ps( &bd[N][ii] );
			u_bd   = _mm_add_ps( u_bd, u_lamt );
			_mm_store_ss( &pd[N][0+(ii+0)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe5 );
			_mm_store_ss( &pd[N][1+(ii+1)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe6 );
			_mm_store_ss( &pd[N][2+(ii+2)*bs+ii*cnz], u_bd );
			u_bd = _mm_shuffle_ps( u_bd, u_bd, 0xe7 );
			_mm_store_ss( &pd[N][3+(ii+3)*bs+ii*cnz], u_bd );
			v_db   = _mm256_load_ps( &db[N][2*ii+0] );
			v_db   = _mm256_mul_ps( v_db, v_lamt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
			v_lam  = _mm256_add_ps( v_lam, v_db );
			u_lam  = _mm256_extractf128_ps( v_lam, 0x1 );
			u_lam  = _mm_hsub_ps( _mm256_castps256_ps128( v_lam ), u_lam ); // [ lam[1]-lam[0] , lam[3]-lam[2] , ... ] + [ dlam[1]-dlam[0] , dlam[3]-dlam[2] , ... ]
			u_bl   = _mm_load_ps( &bl[N][ii] );
			u_bl   = _mm_sub_ps( u_bl, u_lam );
			_mm_store_ps( &pl2[N][ii+0], u_bl );
			_mm_store_ss( &pl[N][(ii+0)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe5 );
			_mm_store_ss( &pl[N][(ii+1)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe6 );
			_mm_store_ss( &pl[N][(ii+2)*bs], u_bl );
			u_bl = _mm_shuffle_ps( u_bl, u_bl, 0xe7 );
			_mm_store_ss( &pl[N][(ii+3)*bs], u_bl );

			ptr_t     += 8;
			ptr_lam   += 8;
			ptr_lamt  += 8;
			ptr_dlam  += 8;
			ptr_t_inv  += 8;

			}
		for(; ll<bs0; ll++)
			{

			ptr_t_inv[0] = 1.0/ptr_t[0];
			ptr_t_inv[1] = 1.0/ptr_t[1];
			ptr_lamt[0] = ptr_lam[0]*ptr_t_inv[0];
			ptr_lamt[1] = ptr_lam[1]*ptr_t_inv[1];
			ptr_dlam[0] = ptr_t_inv[0]*sigma_mu; // !!!!!
			ptr_dlam[1] = ptr_t_inv[1]*sigma_mu; // !!!!!
			ptr_pd[ll+(ii+ll)*bs+ii*cnz] = ptr_bd[ii+ll] + ptr_lamt[0] + ptr_lamt[1];
			ptr_pl[(ii+ll)*bs] = ptr_bl[ii+ll] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+2*ll+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+2*ll+0] - ptr_dlam[0];
			ptr_pl2[ii+ll+0] = ptr_pl[(ii+ll)*bs];

			ptr_t     += 2;
			ptr_lam   += 2;
			ptr_lamt  += 2;
			ptr_dlam  += 2;
			ptr_t_inv  += 2;

			}
		}


	}



void s_compute_alpha_box_mpc(int N, int k0, int k1, int kmax, float *ptr_alpha, float **t, float **dt, float **lam, float **dlam, float **lamt, float **dux, float **db)
	{
	
	const int bs = 8; //d_get_mr();
	
	float 
		k_left_d, alpha = ptr_alpha[0];
	
	int kna = ((k1+bs-1)/bs)*bs;

	int jj, ll;

	__m128
		u_alpha, u_temp;

	__m256
		mask, tmp_mask,
		v_sign, v_tmp0, v_tmp1, v_ones, v_zeros, v_mask0, v_mask1,
		v_dt, v_dux, v_db, v_dlam, v_lamt, v_t, v_alpha, v_lam;

	int int_sign = 0x80000000;
	v_sign = _mm256_broadcast_ss( (float *) &int_sign );
	
	v_ones  = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );
	v_zeros = _mm256_setzero_ps( );

	v_alpha  = _mm256_set_ps( 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 );

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};
	mask = _mm256_loadu_ps( mask_f ); 



	// first stage

	ll = 0;
	for(; ll<k0-7; ll+=8)
		{
/*		break;*/

		v_db    = _mm256_load_ps( &db[0][ll] );
		v_dux    = _mm256_broadcast_ps( (__m128 *) &dux[0][ll/2+0] );
		v_tmp0  = _mm256_shuffle_ps( v_dux, v_dux, 0xfa );
		v_dux    = _mm256_shuffle_ps( v_dux, v_dux, 0x50 );
		v_dux    = _mm256_blend_ps( v_dux, v_tmp0, 0xf0 );
		v_dt    = _mm256_addsub_ps( v_db, v_dux );
		v_dt    = _mm256_xor_ps( v_dt, v_sign );
		v_t     = _mm256_load_ps( &t[0][ll] );
		v_dt    = _mm256_sub_ps( v_dt, v_t );
		_mm256_store_ps( &dt[0][ll], v_dt );

		v_lamt  = _mm256_load_ps( &lamt[0][ll] );
		v_tmp0  = _mm256_mul_ps( v_lamt, v_dt );
		v_lam   = _mm256_load_ps( &lam[0][ll] );
		v_dlam  = _mm256_load_ps( &dlam[0][ll] );
		v_dlam  = _mm256_sub_ps( v_dlam, v_lam );
		v_dlam  = _mm256_sub_ps( v_dlam, v_tmp0 );
		_mm256_store_ps( &dlam[0][ll], v_dlam );

		v_mask0 = _mm256_cmp_ps( v_dlam, v_zeros, 1 );
		v_mask1 = _mm256_cmp_ps( v_dt, v_zeros, 1 );
		v_lam   = _mm256_xor_ps( v_lam, v_sign );
		v_t     = _mm256_xor_ps( v_t, v_sign );
		v_tmp0  = _mm256_div_ps( v_lam, v_dlam );
		v_tmp1  = _mm256_div_ps( v_t, v_dt );
		v_tmp0  = _mm256_blendv_ps( v_ones, v_tmp0, v_mask0 );
		v_tmp1  = _mm256_blendv_ps( v_ones, v_tmp1, v_mask1 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp0 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp1 );

		}
	if(ll<k0)
		{

		k_left_d = 8.0 - (k0 - ll);
		tmp_mask = _mm256_sub_ps( _mm256_broadcast_ss( &k_left_d), mask );
		
		v_db    = _mm256_load_ps( &db[0][ll] );
		v_dux    = _mm256_broadcast_ps( (__m128 *) &dux[0][ll/2+0] );
		v_tmp0  = _mm256_shuffle_ps( v_dux, v_dux, 0xfa );
		v_dux    = _mm256_shuffle_ps( v_dux, v_dux, 0x50 );
		v_dux    = _mm256_blend_ps( v_dux, v_tmp0, 0xf0 );
		v_dt    = _mm256_addsub_ps( v_db, v_dux );
		v_dt    = _mm256_xor_ps( v_dt, v_sign );
		v_t     = _mm256_load_ps( &t[0][ll] );
		v_dt    = _mm256_sub_ps( v_dt, v_t );
		_mm256_maskstore_ps( &dt[0][ll], _mm256_castps_si256( tmp_mask ), v_dt );

		v_lamt  = _mm256_load_ps( &lamt[0][ll] );
		v_tmp0  = _mm256_mul_ps( v_lamt, v_dt );
		v_dlam  = _mm256_load_ps( &dlam[0][ll] );
		v_lam   = _mm256_load_ps( &lam[0][ll] );
		v_dlam  = _mm256_sub_ps( v_dlam, v_lam );
		v_dlam  = _mm256_sub_ps( v_dlam, v_tmp0 );
		_mm256_maskstore_ps( &dlam[0][ll], _mm256_castps_si256( tmp_mask ), v_dlam );

		v_mask0 = _mm256_cmp_ps( v_dlam, v_zeros, 1 );
		v_mask1 = _mm256_cmp_ps( v_dt, v_zeros, 1 );
		v_mask0 = _mm256_and_ps( v_mask0, tmp_mask );
		v_mask1 = _mm256_and_ps( v_mask1, tmp_mask );
		v_lam   = _mm256_xor_ps( v_lam, v_sign );
		v_t     = _mm256_xor_ps( v_t, v_sign );
		v_tmp0  = _mm256_div_ps( v_lam, v_dlam );
		v_tmp1  = _mm256_div_ps( v_t, v_dt );
		v_tmp0  = _mm256_blendv_ps( v_ones, v_tmp0, v_mask0 );
		v_tmp1  = _mm256_blendv_ps( v_ones, v_tmp1, v_mask1 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp0 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp1 );
		
		}

	// middle stages
	for(jj=1; jj<N; jj++)
		{

		ll = 0;
		for(; ll<kmax-7; ll+=8)
			{

			v_db    = _mm256_load_ps( &db[jj][ll] );
			v_dux    = _mm256_broadcast_ps( (__m128 *) &dux[jj][ll/2+0] );
			v_tmp0  = _mm256_shuffle_ps( v_dux, v_dux, 0xfa );
			v_dux    = _mm256_shuffle_ps( v_dux, v_dux, 0x50 );
			v_dux    = _mm256_blend_ps( v_dux, v_tmp0, 0xf0 );
			v_dt    = _mm256_addsub_ps( v_db, v_dux );
			v_dt    = _mm256_xor_ps( v_dt, v_sign );
			v_t     = _mm256_load_ps( &t[jj][ll] );
			v_dt    = _mm256_sub_ps( v_dt, v_t );
			_mm256_store_ps( &dt[jj][ll], v_dt );
			
			v_lamt  = _mm256_load_ps( &lamt[jj][ll] );
			v_tmp0  = _mm256_mul_ps( v_lamt, v_dt );
			v_dlam  = _mm256_load_ps( &dlam[jj][ll] );
			v_lam   = _mm256_load_ps( &lam[jj][ll] );
			v_dlam  = _mm256_sub_ps( v_dlam, v_lam );
			v_dlam  = _mm256_sub_ps( v_dlam, v_tmp0 );
			_mm256_store_ps( &dlam[jj][ll], v_dlam );
			
			v_mask0 = _mm256_cmp_ps( v_dlam, v_zeros, 1 );
			v_mask1 = _mm256_cmp_ps( v_dt, v_zeros, 1 );
			v_lam   = _mm256_xor_ps( v_lam, v_sign );
			v_t     = _mm256_xor_ps( v_t, v_sign );
			v_tmp0  = _mm256_div_ps( v_lam, v_dlam );
			v_tmp1  = _mm256_div_ps( v_t, v_dt );
			v_tmp0  = _mm256_blendv_ps( v_ones, v_tmp0, v_mask0 );
			v_tmp1  = _mm256_blendv_ps( v_ones, v_tmp1, v_mask1 );
			v_alpha = _mm256_min_ps( v_alpha, v_tmp0 );
			v_alpha = _mm256_min_ps( v_alpha, v_tmp1 );

			}
		if(ll<kmax)
			{

			k_left_d = 8.0 - (kmax - ll);
			tmp_mask = _mm256_sub_ps( _mm256_broadcast_ss( &k_left_d), mask );
			
			v_db    = _mm256_load_ps( &db[jj][ll] );
			v_dux    = _mm256_broadcast_ps( (__m128 *) &dux[jj][ll/2+0] );
			v_tmp0  = _mm256_shuffle_ps( v_dux, v_dux, 0xfa );
			v_dux    = _mm256_shuffle_ps( v_dux, v_dux, 0x50 );
			v_dux    = _mm256_blend_ps( v_dux, v_tmp0, 0xf0 );
			v_dt    = _mm256_addsub_ps( v_db, v_dux );
			v_dt    = _mm256_xor_ps( v_dt, v_sign );
			v_t     = _mm256_load_ps( &t[jj][ll] );
			v_dt    = _mm256_sub_ps( v_dt, v_t );
			_mm256_maskstore_ps( &dt[jj][ll], _mm256_castps_si256( tmp_mask ), v_dt );

			v_lamt  = _mm256_load_ps( &lamt[jj][ll] );
			v_tmp0  = _mm256_mul_ps( v_lamt, v_dt );
			v_dlam  = _mm256_load_ps( &dlam[jj][ll] );
			v_lam   = _mm256_load_ps( &lam[jj][ll] );
			v_dlam  = _mm256_sub_ps( v_dlam, v_lam );
			v_dlam  = _mm256_sub_ps( v_dlam, v_tmp0 );
			_mm256_maskstore_ps( &dlam[jj][ll], _mm256_castps_si256( tmp_mask ), v_dlam );

			v_mask0 = _mm256_cmp_ps( v_dlam, v_zeros, 1 );
			v_mask1 = _mm256_cmp_ps( v_dt, v_zeros, 1 );
			v_mask0 = _mm256_and_ps( v_mask0, tmp_mask );
			v_mask1 = _mm256_and_ps( v_mask1, tmp_mask );
			v_lam   = _mm256_xor_ps( v_lam, v_sign );
			v_t     = _mm256_xor_ps( v_t, v_sign );
			v_tmp0  = _mm256_div_ps( v_lam, v_dlam );
			v_tmp1  = _mm256_div_ps( v_t, v_dt );
			v_tmp0  = _mm256_blendv_ps( v_ones, v_tmp0, v_mask0 );
			v_tmp1  = _mm256_blendv_ps( v_ones, v_tmp1, v_mask1 );
			v_alpha = _mm256_min_ps( v_alpha, v_tmp0 );
			v_alpha = _mm256_min_ps( v_alpha, v_tmp1 );
			
			}

		}		

	// last stage
	ll = k1;
	if(ll<kna)
		{

		k_left_d = (float) (kna - ll);
		tmp_mask = _mm256_sub_ps( mask, _mm256_broadcast_ss( &k_left_d) );
		
		v_db    = _mm256_load_ps( &db[N][ll-(8-(kna-k1))] );
		v_dux   = _mm256_broadcast_ps( (__m128 *) &dux[N][(ll-(8-(kna-k1)))/2+0] );
		v_tmp0  = _mm256_shuffle_ps( v_dux, v_dux, 0xfa );
		v_dux   = _mm256_shuffle_ps( v_dux, v_dux, 0x50 );
		v_dux   = _mm256_blend_ps( v_dux, v_tmp0, 0xf0 );
		v_dt    = _mm256_addsub_ps( v_db, v_dux );
		v_dt    = _mm256_xor_ps( v_dt, v_sign );
		v_t     = _mm256_load_ps( &t[N][ll-(8-(kna-k1))] );
		v_dt    = _mm256_sub_ps( v_dt, v_t );
		_mm256_maskstore_ps( &dt[N][ll-(8-(kna-k1))], _mm256_castps_si256( tmp_mask ), v_dt );

		v_lamt  = _mm256_load_ps( &lamt[N][ll-(8-(kna-k1))] );
		v_tmp0  = _mm256_mul_ps( v_lamt, v_dt );
		v_dlam  = _mm256_load_ps( &dlam[N][ll-(8-(kna-k1))] );
		v_lam   = _mm256_load_ps( &lam[N][ll-(8-(kna-k1))] );
		v_dlam  = _mm256_sub_ps( v_dlam, v_lam );
		v_dlam  = _mm256_sub_ps( v_dlam, v_tmp0 );
		_mm256_maskstore_ps( &dlam[N][ll-(8-(kna-k1))], _mm256_castps_si256( tmp_mask ), v_dlam );

		v_mask0 = _mm256_cmp_ps( v_dlam, v_zeros, 1 );
		v_mask1 = _mm256_cmp_ps( v_dt, v_zeros, 1 );
		v_mask0 = _mm256_and_ps( v_mask0, tmp_mask );
		v_mask1 = _mm256_and_ps( v_mask1, tmp_mask );
		v_lam   = _mm256_xor_ps( v_lam, v_sign );
		v_t     = _mm256_xor_ps( v_t, v_sign );
		v_tmp0  = _mm256_div_ps( v_lam, v_dlam );
		v_tmp1  = _mm256_div_ps( v_t, v_dt );
		v_tmp0  = _mm256_blendv_ps( v_ones, v_tmp0, v_mask0 );
		v_tmp1  = _mm256_blendv_ps( v_ones, v_tmp1, v_mask1 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp0 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp1 );
		
		ll = kna;
		}
	for(; ll<kmax-7; ll+=8)
		{

		v_db    = _mm256_load_ps( &db[N][ll] );
		v_dux    = _mm256_broadcast_ps( (__m128 *) &dux[N][ll/2+0] );
		v_tmp0  = _mm256_shuffle_ps( v_dux, v_dux, 0xfa );
		v_dux    = _mm256_shuffle_ps( v_dux, v_dux, 0x50 );
		v_dux    = _mm256_blend_ps( v_dux, v_tmp0, 0xf0 );
		v_dt    = _mm256_addsub_ps( v_db, v_dux );
		v_dt    = _mm256_xor_ps( v_dt, v_sign );
		v_t     = _mm256_load_ps( &t[N][ll] );
		v_dt    = _mm256_sub_ps( v_dt, v_t );
		_mm256_store_ps( &dt[N][ll], v_dt );

		v_lamt  = _mm256_load_ps( &lamt[N][ll] );
		v_tmp0  = _mm256_mul_ps( v_lamt, v_dt );
		v_dlam  = _mm256_load_ps( &dlam[N][ll] );
		v_lam   = _mm256_load_ps( &lam[N][ll] );
		v_dlam  = _mm256_sub_ps( v_dlam, v_lam );
		v_dlam  = _mm256_sub_ps( v_dlam, v_tmp0 );
		_mm256_store_ps( &dlam[N][ll], v_dlam );

		v_mask0 = _mm256_cmp_ps( v_dlam, v_zeros, 1 );
		v_mask1 = _mm256_cmp_ps( v_dt, v_zeros, 1 );
		v_lam   = _mm256_xor_ps( v_lam, v_sign );
		v_t     = _mm256_xor_ps( v_t, v_sign );
		v_tmp0  = _mm256_div_ps( v_lam, v_dlam );
		v_tmp1  = _mm256_div_ps( v_t, v_dt );
		v_tmp0  = _mm256_blendv_ps( v_ones, v_tmp0, v_mask0 );
		v_tmp1  = _mm256_blendv_ps( v_ones, v_tmp1, v_mask1 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp0 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp1 );

		}
	if(ll<kmax)
		{

		k_left_d = 8.0 - (kmax - ll);
		tmp_mask = _mm256_sub_ps( _mm256_broadcast_ss( &k_left_d), mask );
		
		v_db    = _mm256_load_ps( &db[N][ll] );
		v_dux    = _mm256_broadcast_ps( (__m128 *) &dux[N][ll/2+0] );
		v_tmp0  = _mm256_shuffle_ps( v_dux, v_dux, 0xfa );
		v_dux    = _mm256_shuffle_ps( v_dux, v_dux, 0x50 );
		v_dux    = _mm256_blend_ps( v_dux, v_tmp0, 0xf0 );
		v_dt    = _mm256_addsub_ps( v_db, v_dux );
		v_dt    = _mm256_xor_ps( v_dt, v_sign );
		v_t     = _mm256_load_ps( &t[N][ll] );
		v_dt    = _mm256_sub_ps( v_dt, v_t );
		_mm256_maskstore_ps( &dt[N][ll], _mm256_castps_si256( tmp_mask ), v_dt );

		v_lamt  = _mm256_load_ps( &lamt[N][ll] );
		v_tmp0  = _mm256_mul_ps( v_lamt, v_dt );
		v_dlam  = _mm256_load_ps( &dlam[N][ll] );
		v_lam   = _mm256_load_ps( &lam[N][ll] );
		v_dlam  = _mm256_sub_ps( v_dlam, v_lam );
		v_dlam  = _mm256_sub_ps( v_dlam, v_tmp0 );
		_mm256_maskstore_ps( &dlam[N][ll], _mm256_castps_si256( tmp_mask ), v_dlam );

		v_mask0 = _mm256_cmp_ps( v_dlam, v_zeros, 1 );
		v_mask1 = _mm256_cmp_ps( v_dt, v_zeros, 1 );
		v_mask0 = _mm256_and_ps( v_mask0, tmp_mask );
		v_mask1 = _mm256_and_ps( v_mask1, tmp_mask );
		v_lam   = _mm256_xor_ps( v_lam, v_sign );
		v_t     = _mm256_xor_ps( v_t, v_sign );
		v_tmp0  = _mm256_div_ps( v_lam, v_dlam );
		v_tmp1  = _mm256_div_ps( v_t, v_dt );
		v_tmp0  = _mm256_blendv_ps( v_ones, v_tmp0, v_mask0 );
		v_tmp1  = _mm256_blendv_ps( v_ones, v_tmp1, v_mask1 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp0 );
		v_alpha = _mm256_min_ps( v_alpha, v_tmp1 );
		
		}
	
	u_alpha = _mm256_extractf128_ps( v_alpha, 0x1 );
	u_alpha = _mm_min_ps( u_alpha, _mm256_castps256_ps128( v_alpha ) );
	u_temp  = _mm_permute_ps( u_alpha, 0x4e );
	u_alpha = _mm_min_ps( u_alpha, u_temp );
	u_temp  = _mm_permute_ps( u_alpha, 0x01 );
	u_alpha = _mm_min_ss( u_alpha, u_temp );

/*	u_alpha = _mm_min_ss( u_alpha, _mm_load_ps( &alpha ) );*/

	_mm_store_ss( &alpha, u_alpha );

	ptr_alpha[0] = alpha;

	return;
	
	}



void s_update_var_mpc(int nx, int nu, int N, int nb, int nbu, float *ptr_mu, float mu_scal, float alpha, float **ux, float **dux, float **t, float **dt, float **lam, float **dlam, float **pi, float **dpi)
	{
	
	int 
		jj, ll, ll_left;
	
	float 
		ll_left_f;

	__m128
		u_mu, u_tmp;

	__m256
		mask, zeros, alpha_mask,
		v_alpha, v_ux, v_dux, v_pi, v_dpi, v_t, v_dt, v_lam, v_dlam, v_mu;
		
	v_alpha = _mm256_set_ps( alpha, alpha, alpha, alpha, alpha, alpha, alpha, alpha );
	
	v_mu = _mm256_setzero_ps();

	zeros = _mm256_setzero_ps();

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};
	mask = _mm256_loadu_ps( mask_f );



	// update inputs
	ll = 0;
	for(; ll<nu-7; ll+=8)
		{
		v_ux  = _mm256_load_ps( &ux[jj][ll] );
		v_dux = _mm256_load_ps( &dux[jj][ll] );
		v_dux = _mm256_sub_ps( v_dux, v_ux );
		v_dux = _mm256_mul_ps( v_alpha, v_dux );
		v_ux  = _mm256_add_ps( v_ux, v_dux );
		_mm256_store_ps( &ux[jj][ll], v_ux );
		}
	ll_left = nu - ll;
	if( ll_left>0 )
		{
		ll_left_f = 8.0 - ll_left;
		alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_ux  = _mm256_load_ps( &ux[jj][ll] );
		v_dux = _mm256_load_ps( &dux[jj][ll] );
		v_dux = _mm256_sub_ps( v_dux, v_ux );
		v_dux = _mm256_mul_ps( alpha_mask, v_dux );
		v_ux  = _mm256_add_ps( v_ux, v_dux );
		_mm256_store_ps( &ux[jj][ll], v_ux );
		}

	// box constraints
	ll = 0;
	for(; ll<2*nbu-7; ll+=8)
		{
		v_t    = _mm256_load_ps( &t[0][ll] );
		v_lam  = _mm256_load_ps( &lam[0][ll] );
		v_dt   = _mm256_load_ps( &dt[0][ll] );
		v_dlam = _mm256_load_ps( &dlam[0][ll] );
		v_dt   = _mm256_mul_ps( v_alpha, v_dt );
		v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		_mm256_store_ps( &t[0][ll], v_t );
		_mm256_store_ps( &lam[0][ll], v_lam );
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}
	ll_left = 2*nbu - ll;
	if( ll_left>0 )
		{
		ll_left_f = 8.0 - ll_left;
		alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_t    = _mm256_load_ps( &t[0][ll] );
		v_lam  = _mm256_load_ps( &lam[0][ll] );
		v_dt   = _mm256_load_ps( &dt[0][ll] );
		v_dlam = _mm256_load_ps( &dlam[0][ll] );
		v_dt   = _mm256_mul_ps( alpha_mask, v_dt );
		v_dlam = _mm256_mul_ps( alpha_mask, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		_mm256_store_ps( &t[0][ll], v_t );
		_mm256_store_ps( &lam[0][ll], v_lam );
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_lam = _mm256_blendv_ps( v_lam, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}


	for(jj=1; jj<N; jj++)
		{
		ll = 0;
		for(; ll<nu+nx-7; ll+=8)
			{
			v_ux  = _mm256_load_ps( &ux[jj][ll] );
			v_dux = _mm256_load_ps( &dux[jj][ll] );
			v_dux = _mm256_sub_ps( v_dux, v_ux );
			v_dux = _mm256_mul_ps( v_alpha, v_dux );
			v_ux  = _mm256_add_ps( v_ux, v_dux );
			_mm256_store_ps( &ux[jj][ll], v_ux );
			}
		ll_left = nu + nx - ll;
		if( ll_left>0 )
			{
			ll_left_f = 8.0 - ll_left;
			alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
			v_ux  = _mm256_load_ps( &ux[jj][ll] );
			v_dux = _mm256_load_ps( &dux[jj][ll] );
			v_dux = _mm256_sub_ps( v_dux, v_ux );
			v_dux = _mm256_mul_ps( alpha_mask, v_dux );
			v_ux  = _mm256_add_ps( v_ux, v_dux );
			_mm256_store_ps( &ux[jj][ll], v_ux );
			}

		// update equality constrained multipliers
		ll = 0;
		for(; ll<nx-7; ll+=8)
			{
			v_pi  = _mm256_load_ps( &pi[jj][ll] );
			v_dpi = _mm256_load_ps( &dpi[jj][ll] );
			v_dpi = _mm256_sub_ps( v_dpi, v_pi );
			v_dpi = _mm256_mul_ps( v_alpha, v_dpi );
			v_pi  = _mm256_add_ps( v_pi, v_dpi );
			_mm256_store_ps( &pi[jj][ll], v_pi );
			}
		ll_left = nx - ll;
		if( ll_left>0 )
			{
			ll_left_f = 8.0 - ll_left;
			alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
			v_pi  = _mm256_load_ps( &pi[jj][ll] );
			v_dpi = _mm256_load_ps( &dpi[jj][ll] );
			v_dpi = _mm256_sub_ps( v_dpi, v_pi );
			v_dpi = _mm256_mul_ps( alpha_mask, v_dpi );
			v_pi  = _mm256_add_ps( v_pi, v_dpi );
			_mm256_store_ps( &pi[jj][ll], v_pi );
			}

		// box constraints
		ll = 0;
		for(; ll<2*nb-7; ll+=8)
			{
			v_t    = _mm256_load_ps( &t[jj][ll] );
			v_lam  = _mm256_load_ps( &lam[jj][ll] );
			v_dt   = _mm256_load_ps( &dt[jj][ll] );
			v_dlam = _mm256_load_ps( &dlam[jj][ll] );
			v_dt   = _mm256_mul_ps( v_alpha, v_dt );
			v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
			v_t    = _mm256_add_ps( v_t, v_dt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
			_mm256_store_ps( &t[jj][ll], v_t );
			_mm256_store_ps( &lam[jj][ll], v_lam );
			v_lam  = _mm256_mul_ps( v_lam, v_t );
			v_mu   = _mm256_add_ps( v_mu, v_lam );
			}
		ll_left = 2*nb - ll;
		if( ll_left>0 )
			{
			ll_left_f = 8.0 - ll_left;
			alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
			v_t    = _mm256_load_ps( &t[jj][ll] );
			v_lam  = _mm256_load_ps( &lam[jj][ll] );
			v_dt   = _mm256_load_ps( &dt[jj][ll] );
			v_dlam = _mm256_load_ps( &dlam[jj][ll] );
			v_dt   = _mm256_mul_ps( alpha_mask, v_dt );
			v_dlam = _mm256_mul_ps( alpha_mask, v_dlam );
			v_t    = _mm256_add_ps( v_t, v_dt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
			_mm256_store_ps( &t[jj][ll], v_t );
			_mm256_store_ps( &lam[jj][ll], v_lam );
			v_lam  = _mm256_mul_ps( v_lam, v_t );
			v_lam = _mm256_blendv_ps( v_lam, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
			v_mu   = _mm256_add_ps( v_mu, v_lam );
			}

		}

	// update states
	ll = 0;
	for(; ll<nx-7; ll+=8)
		{
		v_ux  = _mm256_loadu_ps( &ux[N][nu+ll] );
		v_dux = _mm256_loadu_ps( &dux[N][nu+ll] );
		v_dux = _mm256_sub_ps( v_dux, v_ux );
		v_dux = _mm256_mul_ps( v_alpha, v_dux );
		v_ux  = _mm256_add_ps( v_ux, v_dux );
		_mm256_storeu_ps( &ux[N][nu+ll], v_ux );
		}
	ll_left = nx - ll;
	if( ll_left>0 )
		{
		ll_left_f = 8.0 - ll_left;
		alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_ux  = _mm256_loadu_ps( &ux[N][nu+ll] );
		v_dux = _mm256_loadu_ps( &dux[N][nu+ll] );
		v_dux = _mm256_sub_ps( v_dux, v_ux );
		v_dux = _mm256_mul_ps( alpha_mask, v_dux );
		v_ux  = _mm256_add_ps( v_ux, v_dux );
		_mm256_storeu_ps( &ux[N][nu+ll], v_ux );
		}

	// update equality constrained multipliers
	ll = 0;
	for(; ll<nx-7; ll+=8)
		{
		v_pi  = _mm256_load_ps( &pi[N][ll] );
		v_dpi = _mm256_load_ps( &dpi[N][ll] );
		v_dpi = _mm256_sub_ps( v_dpi, v_pi );
		v_dpi = _mm256_mul_ps( v_alpha, v_dpi );
		v_pi  = _mm256_add_ps( v_pi, v_dpi );
		_mm256_store_ps( &pi[N][ll], v_pi );
		}
	ll_left = nx - ll;
	if( ll_left>0 )
		{
		ll_left_f = 8.0 - ll_left;
		alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_pi  = _mm256_load_ps( &pi[N][ll] );
		v_dpi = _mm256_load_ps( &dpi[N][ll] );
		v_dpi = _mm256_sub_ps( v_dpi, v_pi );
		v_dpi = _mm256_mul_ps( alpha_mask, v_dpi );
		v_dpi  = _mm256_add_ps( v_pi, v_dpi );
		v_pi = _mm256_blendv_ps( v_dpi, v_pi, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		_mm256_store_ps( &pi[N][ll], v_pi );
		}

	// box constraints
	ll = 2*nu;
	for(; ll<2*nb-7; ll+=8)
		{
		v_t    = _mm256_loadu_ps( &t[N][ll] );
		v_lam  = _mm256_loadu_ps( &lam[N][ll] );
		v_dt   = _mm256_loadu_ps( &dt[N][ll] );
		v_dlam = _mm256_loadu_ps( &dlam[N][ll] );
		v_dt   = _mm256_mul_ps( v_alpha, v_dt );
		v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		_mm256_storeu_ps( &t[N][ll], v_t );
		_mm256_storeu_ps( &lam[N][ll], v_lam );
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}
	ll_left = 2*nb - ll;
	if( ll_left>0 )
		{
		ll_left_f = 8.0 - ll_left;
		alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_t    = _mm256_loadu_ps( &t[N][ll] );
		v_lam  = _mm256_loadu_ps( &lam[N][ll] );
		v_dt   = _mm256_loadu_ps( &dt[N][ll] );
		v_dlam = _mm256_loadu_ps( &dlam[N][ll] );
		v_dt   = _mm256_mul_ps( alpha_mask, v_dt );
		v_dlam = _mm256_mul_ps( alpha_mask, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
		_mm256_storeu_ps( &t[N][ll], v_t );
		_mm256_storeu_ps( &lam[N][ll], v_lam );
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_lam = _mm256_blendv_ps( v_lam, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}

	u_tmp = _mm256_extractf128_ps( v_mu, 0x1 );
	u_mu  = _mm_add_ps( u_tmp, _mm256_castps256_ps128( v_mu ) );

	u_mu  = _mm_hadd_ps( u_mu, u_mu );
	u_mu  = _mm_hadd_ps( u_mu, u_mu );

	u_tmp = _mm_load_ss( &mu_scal );
	u_mu  = _mm_mul_ss( u_mu, u_tmp );
	_mm_store_ss( ptr_mu, u_mu );

	return;
	
	}



void s_compute_mu_mpc(int N, int nbu, int nu, int nb, float *ptr_mu, float mu_scal, float alpha, float **lam, float **dlam, float **t, float **dt)
	{

	int 
		jj, ll, ll_left;
	
	float 
		ll_left_f;

	__m128
		u_mu, u_tmp;

	__m256
		mask, zeros, alpha_mask,
		v_alpha, v_t, v_dt, v_lam, v_dlam, v_mu;
		
	v_alpha = _mm256_set_ps( alpha, alpha, alpha, alpha, alpha, alpha, alpha, alpha );
	
	v_mu = _mm256_setzero_ps();

	zeros = _mm256_setzero_ps();

	const float mask_f[] = {7.5, 6.5, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5};
	mask = _mm256_loadu_ps( mask_f );

	// box constraints
	ll = 0;
	for(; ll<2*nbu-7; ll+=8)
		{
		v_t    = _mm256_load_ps( &t[0][ll] );
		v_lam  = _mm256_load_ps( &lam[0][ll] );
		v_dt   = _mm256_load_ps( &dt[0][ll] );
		v_dlam = _mm256_load_ps( &dlam[0][ll] );
		v_dt   = _mm256_mul_ps( v_alpha, v_dt );
		v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
/*		_mm256_store_ps( &t[0][ll], v_t );*/
/*		_mm256_store_ps( &lam[0][ll], v_lam );*/
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}
	ll_left = 2*nbu - ll;
	if( ll_left>0 )
		{
		ll_left_f = 8.0 - ll_left;
/*		alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );*/
		v_t    = _mm256_load_ps( &t[0][ll] );
		v_lam  = _mm256_load_ps( &lam[0][ll] );
		v_dt   = _mm256_load_ps( &dt[0][ll] );
		v_dlam = _mm256_load_ps( &dlam[0][ll] );
		v_dt   = _mm256_mul_ps( v_alpha, v_dt );
		v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
/*		_mm256_store_ps( &t[0][ll], v_t );*/
/*		_mm256_store_ps( &lam[0][ll], v_lam );*/
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_lam = _mm256_blendv_ps( v_lam, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}


	for(jj=1; jj<N; jj++)
		{

		// box constraints
		ll = 0;
		for(; ll<2*nb-7; ll+=8)
			{
			v_t    = _mm256_load_ps( &t[jj][ll] );
			v_lam  = _mm256_load_ps( &lam[jj][ll] );
			v_dt   = _mm256_load_ps( &dt[jj][ll] );
			v_dlam = _mm256_load_ps( &dlam[jj][ll] );
			v_dt   = _mm256_mul_ps( v_alpha, v_dt );
			v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
			v_t    = _mm256_add_ps( v_t, v_dt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
/*			_mm256_store_ps( &t[jj][ll], v_t );*/
/*			_mm256_store_ps( &lam[jj][ll], v_lam );*/
			v_lam  = _mm256_mul_ps( v_lam, v_t );
			v_mu   = _mm256_add_ps( v_mu, v_lam );
			}
		ll_left = 2*nb - ll;
		if( ll_left>0 )
			{
			ll_left_f = 8.0 - ll_left;
			alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
			v_t    = _mm256_load_ps( &t[jj][ll] );
			v_lam  = _mm256_load_ps( &lam[jj][ll] );
			v_dt   = _mm256_load_ps( &dt[jj][ll] );
			v_dlam = _mm256_load_ps( &dlam[jj][ll] );
			v_dt   = _mm256_mul_ps( v_alpha, v_dt );
			v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
			v_t    = _mm256_add_ps( v_t, v_dt );
			v_lam  = _mm256_add_ps( v_lam, v_dlam );
/*			_mm256_store_ps( &t[jj][ll], v_t );*/
/*			_mm256_store_ps( &lam[jj][ll], v_lam );*/
			v_lam  = _mm256_mul_ps( v_lam, v_t );
			v_lam = _mm256_blendv_ps( v_lam, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
			v_mu   = _mm256_add_ps( v_mu, v_lam );
			}

		}

	// box constraints
	ll = 2*nu;
	for(; ll<2*nb-7; ll+=8)
		{
		v_t    = _mm256_loadu_ps( &t[N][ll] );
		v_lam  = _mm256_loadu_ps( &lam[N][ll] );
		v_dt   = _mm256_loadu_ps( &dt[N][ll] );
		v_dlam = _mm256_loadu_ps( &dlam[N][ll] );
		v_dt   = _mm256_mul_ps( v_alpha, v_dt );
		v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
/*		_mm256_storeu_ps( &t[N][ll], v_t );*/
/*		_mm256_storeu_ps( &lam[N][ll], v_lam );*/
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}
	ll_left = 2*nb - ll;
	if( ll_left>0 )
		{
		ll_left_f = 8.0 - ll_left;
		alpha_mask = _mm256_blendv_ps( v_alpha, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_t    = _mm256_loadu_ps( &t[N][ll] );
		v_lam  = _mm256_loadu_ps( &lam[N][ll] );
		v_dt   = _mm256_loadu_ps( &dt[N][ll] );
		v_dlam = _mm256_loadu_ps( &dlam[N][ll] );
		v_dt   = _mm256_mul_ps( v_alpha, v_dt );
		v_dlam = _mm256_mul_ps( v_alpha, v_dlam );
		v_t    = _mm256_add_ps( v_t, v_dt );
		v_lam  = _mm256_add_ps( v_lam, v_dlam );
/*		_mm256_storeu_ps( &t[N][ll], v_t );*/
/*		_mm256_storeu_ps( &lam[N][ll], v_lam );*/
		v_lam  = _mm256_mul_ps( v_lam, v_t );
		v_lam = _mm256_blendv_ps( v_lam, zeros, _mm256_sub_ps( mask, _mm256_broadcast_ss( &ll_left_f) ) );
		v_mu   = _mm256_add_ps( v_mu, v_lam );
		}

	u_tmp = _mm256_extractf128_ps( v_mu, 0x1 );
	u_mu  = _mm_add_ps( u_tmp, _mm256_castps256_ps128( v_mu ) );

	u_mu  = _mm_hadd_ps( u_mu, u_mu );
	u_mu  = _mm_hadd_ps( u_mu, u_mu );

	u_tmp = _mm_load_ss( &mu_scal );
	u_mu  = _mm_mul_ss( u_mu, u_tmp );
	_mm_store_ss( ptr_mu, u_mu );

	return;

	}



