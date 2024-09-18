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

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX

#include "../include/hpipm_d_core_qp_ipm.h"



void d_compute_Gamma_gamma_qp(double *res_d, double *res_m, struct d_core_qp_ipm_workspace *cws)
	{

	int nc = cws->nc;

	double *lam = cws->lam;
	double *t = cws->t;
	double *t_inv = cws->t_inv;
	double *Gamma = cws->Gamma;
	double *gamma = cws->gamma;
	double lam_min = cws->lam_min;
	double t_min = cws->t_min;
	double t_min_inv = cws->t_min_inv;

	double t_inv_tmp, lam_tmp;

	__m256d
		y_ones, y_lam_min, y_t_min, y_t_min_inv,
		y_tmp0, y_t0, y_t_inv0, y_lam0, y_mask0,
		y_tmp1, y_t1, y_t_inv1, y_lam1, y_mask1;

	y_ones = _mm256_set_pd( 1.0, 1.0, 1.0, 1.0 );

	// local variables
	int ii;

	y_lam_min = _mm256_broadcast_sd( &lam_min );
	y_t_min = _mm256_broadcast_sd( &t_min );
	y_t_min_inv = _mm256_broadcast_sd( &t_min_inv );

	if(cws->t_lam_min==1)
		{
		ii = 0;
		for(; ii<nc-3; ii+=4)
			{
			y_t0 = _mm256_loadu_pd( &t[ii] );
			y_t_inv0 = _mm256_div_pd( y_ones, y_t0 );
			_mm256_storeu_pd( &t_inv[ii], y_t_inv0 );
			y_lam0 = _mm256_loadu_pd( &lam[ii] );

			y_mask0 = _mm256_cmp_pd( y_lam0, y_lam_min, 2 );
			y_mask1 = _mm256_cmp_pd( y_t0, y_t_min, 2 );
			y_lam0 = _mm256_blendv_pd( y_lam0, y_lam_min, y_mask0 );
			y_t_inv0 = _mm256_blendv_pd( y_t_inv0, y_t_min_inv, y_mask1 );

			y_tmp0 = _mm256_mul_pd( y_t_inv0, y_lam0 );
			_mm256_storeu_pd( &Gamma[ii], y_tmp0 );
			y_tmp0 = _mm256_mul_pd( y_lam0, _mm256_loadu_pd( &res_d[ii] ) );
			y_tmp0 = _mm256_sub_pd( _mm256_loadu_pd( &res_m[ii] ), y_tmp0 );
			y_tmp0 = _mm256_mul_pd( y_t_inv0, y_tmp0 );
			_mm256_storeu_pd( &gamma[ii], y_tmp0 );
			}
		for(; ii<nc; ii++)
			{
			t_inv[ii] = 1.0/t[ii];
			t_inv_tmp = t[ii]<t_min ? t_min_inv : t_inv[ii];
			lam_tmp = lam[ii]<lam_min ? lam_min : lam[ii];
			Gamma[ii] = t_inv_tmp*lam_tmp;
			gamma[ii] = t_inv[ii]*(res_m[ii]-lam[ii]*res_d[ii]);
			}
		}
	else
		{
		ii = 0;
		for(; ii<nc-3; ii+=4)
			{
			y_t0 = _mm256_loadu_pd( &t[ii] );
			y_t_inv0 = _mm256_div_pd( y_ones, y_t0 );
			_mm256_storeu_pd( &t_inv[ii], y_t_inv0 );
			y_lam0 = _mm256_loadu_pd( &lam[ii] );

			y_tmp0 = _mm256_mul_pd( y_t_inv0, y_lam0 );
			_mm256_storeu_pd( &Gamma[ii], y_tmp0 );
			y_tmp0 = _mm256_mul_pd( y_lam0, _mm256_loadu_pd( &res_d[ii] ) );
			y_tmp0 = _mm256_sub_pd( _mm256_loadu_pd( &res_m[ii] ), y_tmp0 );
			y_tmp0 = _mm256_mul_pd( y_t_inv0, y_tmp0 );
			_mm256_storeu_pd( &gamma[ii], y_tmp0 );
			}
		for(; ii<nc; ii++)
			{
			t_inv[ii] = 1.0/t[ii];
			Gamma[ii] = t_inv[ii]*lam[ii];
			gamma[ii] = t_inv[ii]*(res_m[ii]-lam[ii]*res_d[ii]);
			}
		}

	return;

	}



void d_compute_gamma_qp(double *res_d, double *res_m, struct d_core_qp_ipm_workspace *cws)
	{

	int nc = cws->nc;

	double *lam = cws->lam;
	double *t_inv = cws->t_inv;
	double *gamma = cws->gamma;

	__m256d
		y_tmp0, y_lam0,
		y_tmp1, y_lam1;

	// local variables
	int ii;

	ii = 0;
#if 0
	for(; ii<nc-7; ii+=8)
		{
		y_lam0 = _mm256_loadu_pd( &lam[ii+0] );
		y_lam1 = _mm256_loadu_pd( &lam[ii+4] );
		y_tmp0 = _mm256_mul_pd( y_lam0, _mm256_loadu_pd( &res_d[ii+0] ) );
		y_tmp1 = _mm256_mul_pd( y_lam1, _mm256_loadu_pd( &res_d[ii+4] ) );
		y_tmp0 = _mm256_sub_pd( _mm256_loadu_pd( &res_m[ii+0] ), y_tmp0 );
		y_tmp1 = _mm256_sub_pd( _mm256_loadu_pd( &res_m[ii+4] ), y_tmp1 );
		y_tmp0 = _mm256_mul_pd( _mm256_loadu_pd( &t_inv[ii+0] ), y_tmp0 );
		y_tmp1 = _mm256_mul_pd( _mm256_loadu_pd( &t_inv[ii+4] ), y_tmp1 );
		_mm256_storeu_pd( &gamma[ii+0], y_tmp0 );
		_mm256_storeu_pd( &gamma[ii+4], y_tmp1 );
		}
#endif
	for(; ii<nc-3; ii+=4)
		{
		y_lam0 = _mm256_loadu_pd( &lam[ii] );
		y_tmp0 = _mm256_mul_pd( y_lam0, _mm256_loadu_pd( &res_d[ii] ) );
		y_tmp0 = _mm256_sub_pd( _mm256_loadu_pd( &res_m[ii] ), y_tmp0 );
		y_tmp0 = _mm256_mul_pd( _mm256_loadu_pd( &t_inv[ii] ), y_tmp0 );
		_mm256_storeu_pd( &gamma[ii], y_tmp0 );
		}
	for(; ii<nc; ii++)
		{
		gamma[ii] = t_inv[ii]*(res_m[ii]-lam[ii]*res_d[ii]);
		}

	return;

	}



void d_compute_lam_t_qp(double *res_d, double *res_m, double *dlam, double *dt, struct d_core_qp_ipm_workspace *cws)
	{

	int nc = cws->nc;

	double *lam = cws->lam;
	double *t_inv = cws->t_inv;

	__m256d
		y_sign,
		y_tmp0, y_tmp2, y_dt0,
		y_tmp1, y_tmp3, y_dt1;

	long long long_sign = 0x8000000000000000;
	y_sign = _mm256_broadcast_sd( (double *) &long_sign );

	// local variables
	int ii;

	ii = 0;
#if 0
	for(; ii<nc-7; ii+=8)
		{
		y_dt0 = _mm256_loadu_pd( &dt[ii+0] );
		y_dt1 = _mm256_loadu_pd( &dt[ii+4] );
		y_dt0 = _mm256_sub_pd( y_dt0, _mm256_loadu_pd( &res_d[ii+0] ) );
		y_dt1 = _mm256_sub_pd( y_dt1, _mm256_loadu_pd( &res_d[ii+4] ) );
		_mm256_storeu_pd( &dt[ii+0], y_dt0 );
		_mm256_storeu_pd( &dt[ii+4], y_dt1 );
		y_tmp0 = _mm256_mul_pd( y_dt0, _mm256_loadu_pd( &lam[ii+0] ) );
		y_tmp1 = _mm256_mul_pd( y_dt1, _mm256_loadu_pd( &lam[ii+4] ) );
		y_tmp2 = _mm256_loadu_pd( &t_inv[ii+0] );
		y_tmp3 = _mm256_loadu_pd( &t_inv[ii+4] );
		y_tmp0 = _mm256_add_pd( y_tmp0, _mm256_loadu_pd( &res_m[ii+0] ) );
		y_tmp1 = _mm256_add_pd( y_tmp1, _mm256_loadu_pd( &res_m[ii+4] ) );
		y_tmp2 = _mm256_xor_pd( y_tmp2, y_sign );
		y_tmp3 = _mm256_xor_pd( y_tmp3, y_sign );
		y_tmp0 = _mm256_mul_pd( y_tmp0, y_tmp2 );
		y_tmp1 = _mm256_mul_pd( y_tmp1, y_tmp3 );
		_mm256_storeu_pd( &dlam[ii+0], y_tmp0 );
		_mm256_storeu_pd( &dlam[ii+4], y_tmp1 );
		}
#endif
	for(; ii<nc-3; ii+=4)
		{
		y_dt0 = _mm256_loadu_pd( &dt[ii+0] );
		y_dt0 = _mm256_sub_pd( y_dt0, _mm256_loadu_pd( &res_d[ii+0] ) );
		_mm256_storeu_pd( &dt[ii+0], y_dt0 );
		y_tmp0 = _mm256_mul_pd( y_dt0, _mm256_loadu_pd( &lam[ii+0] ) );
		y_tmp2 = _mm256_loadu_pd( &t_inv[ii+0] );
		y_tmp0 = _mm256_add_pd( y_tmp0, _mm256_loadu_pd( &res_m[ii+0] ) );
		y_tmp2 = _mm256_xor_pd( y_tmp2, y_sign );
		y_tmp0 = _mm256_mul_pd( y_tmp0, y_tmp2 );
		_mm256_storeu_pd( &dlam[ii+0], y_tmp0 );
		}
	for(; ii<nc; ii++)
		{
		dt[ii] -= res_d[ii];
		// TODO compute lamda alone ???
		dlam[ii] = - t_inv[ii] * (lam[ii]*dt[ii] + res_m[ii]);
		}
	
	return;

	}



void d_compute_alpha_qp(struct d_core_qp_ipm_workspace *cws)
	{
	
	// extract workspace members
	int nc = cws->nc;

	double *lam = cws->lam;
	double *t = cws->t;
	double *dlam = cws->dlam;
	double *dt = cws->dt;

	double alpha_prim = - 1.0;
	double alpha_dual = - 1.0;
	double alpha = - 1.0;

	__m256d
		y_alpha0, y_alpha1;

	__m128d
		x_alpha0, x_alpha1;

	__m256d
		d_zeros, d_mones,
		d_tmp0, d_tmp2, d_mask0, d_mask2, d_alpha0, d_alpha2;

	d_mones  = _mm256_set_pd( -1.0, -1.0, -1.0, -1.0 );
	d_zeros = _mm256_setzero_pd( );

	d_alpha0 = _mm256_set_pd( -1.0, -1.0, -1.0, -1.0 );
	d_alpha2 = _mm256_set_pd( -1.0, -1.0, -1.0, -1.0 );

#if 0
	__m128
		s_zeros, s_mones,
		s_tmp0, s_tmp2, s_mask0, s_mask2, s_alpha0, s_alpha2,
		s_tmp1, s_tmp3, s_mask1, s_mask3, s_alpha1, s_alpha3;

	s_mones  = _mm_set_ps( -1.0, -1.0, -1.0, -1.0 );
	s_zeros = _mm_setzero_ps( );

	s_alpha0 = _mm_set_ps( -1.0, -1.0, -1.0, -1.0 );
	s_alpha2 = _mm_set_ps( -1.0, -1.0, -1.0, -1.0 );
#endif

	// local variables
	int ii;

	ii = 0;
#if 0
	s_alpha1 = _mm_set_ps( -1.0, -1.0, -1.0, -1.0 );
	s_alpha3 = _mm_set_ps( -1.0, -1.0, -1.0, -1.0 );
	for(; ii<nc-7; ii+=8)
		{
		s_tmp0 = _mm256_cvtpd_ps( _mm256_loadu_pd( &dlam[ii+0] ) );
		s_tmp1 = _mm256_cvtpd_ps( _mm256_loadu_pd( &dlam[ii+4] ) );
		s_tmp2 = _mm256_cvtpd_ps( _mm256_loadu_pd( &dt[ii+0] ) );
		s_tmp3 = _mm256_cvtpd_ps( _mm256_loadu_pd( &dt[ii+4] ) );
		s_mask0 = _mm_cmp_ps( s_tmp0, s_zeros, 0x01 );
		s_mask1 = _mm_cmp_ps( s_tmp1, s_zeros, 0x01 );
		s_mask2 = _mm_cmp_ps( s_tmp2, s_zeros, 0x01 );
		s_mask3 = _mm_cmp_ps( s_tmp3, s_zeros, 0x01 );
		s_tmp0 = _mm_div_ps( _mm256_cvtpd_ps( _mm256_loadu_pd( &lam[ii+0] ) ), s_tmp0 );
		s_tmp1 = _mm_div_ps( _mm256_cvtpd_ps( _mm256_loadu_pd( &lam[ii+4] ) ), s_tmp1 );
		s_tmp2 = _mm_div_ps( _mm256_cvtpd_ps( _mm256_loadu_pd( &t[ii+0] ) ), s_tmp2 );
		s_tmp3 = _mm_div_ps( _mm256_cvtpd_ps( _mm256_loadu_pd( &t[ii+4] ) ), s_tmp3 );
		s_tmp0 = _mm_blendv_ps( s_mones, s_tmp0, s_mask0 );
		s_tmp1 = _mm_blendv_ps( s_mones, s_tmp1, s_mask1 );
		s_tmp2 = _mm_blendv_ps( s_mones, s_tmp2, s_mask2 );
		s_tmp3 = _mm_blendv_ps( s_mones, s_tmp3, s_mask3 );
		s_alpha0 = _mm_max_ps( s_alpha0, s_tmp0 );
		s_alpha1 = _mm_max_ps( s_alpha1, s_tmp1 );
		s_alpha2 = _mm_max_ps( s_alpha2, s_tmp2 );
		s_alpha3 = _mm_max_ps( s_alpha3, s_tmp3 );
		}
	s_alpha0 = _mm_max_ps( s_alpha0, s_alpha1 );
	s_alpha2 = _mm_max_ps( s_alpha2, s_alpha3 );
#endif
#if 0
	for(; ii<nc-3; ii+=4)
		{
		s_tmp0 = _mm256_cvtpd_ps( _mm256_loadu_pd( &dlam[ii+0] ) );
		s_tmp2 = _mm256_cvtpd_ps( _mm256_loadu_pd( &dt[ii+0] ) );
		s_mask0 = _mm_cmp_ps( s_tmp0, s_zeros, 0x01 );
		s_mask2 = _mm_cmp_ps( s_tmp2, s_zeros, 0x01 );
		s_tmp0 = _mm_div_ps( _mm256_cvtpd_ps( _mm256_loadu_pd( &lam[ii+0] ) ), s_tmp0 );
		s_tmp2 = _mm_div_ps( _mm256_cvtpd_ps( _mm256_loadu_pd( &t[ii+0] ) ), s_tmp2 );
		s_tmp0 = _mm_blendv_ps( s_mones, s_tmp0, s_mask0 );
		s_tmp2 = _mm_blendv_ps( s_mones, s_tmp2, s_mask2 );
		s_alpha0 = _mm_max_ps( s_alpha0, s_tmp0 );
		s_alpha2 = _mm_max_ps( s_alpha2, s_tmp2 );
		}
#endif
	for(; ii<nc-3; ii+=4)
		{
		d_tmp0 = _mm256_loadu_pd( &dlam[ii+0] );
		d_tmp2 = _mm256_loadu_pd( &dt[ii+0] );
		d_mask0 = _mm256_cmp_pd( d_tmp0, d_zeros, 0x01 );
		d_mask2 = _mm256_cmp_pd( d_tmp2, d_zeros, 0x01 );
		d_tmp0 = _mm256_div_pd( _mm256_loadu_pd( &lam[ii+0] ), d_tmp0 );
		d_tmp2 = _mm256_div_pd( _mm256_loadu_pd( &t[ii+0] ), d_tmp2 );
		d_tmp0 = _mm256_blendv_pd( d_mones, d_tmp0, d_mask0 );
		d_tmp2 = _mm256_blendv_pd( d_mones, d_tmp2, d_mask2 );
		d_alpha0 = _mm256_max_pd( d_alpha0, d_tmp0 );
		d_alpha2 = _mm256_max_pd( d_alpha2, d_tmp2 );
		}
	for(; ii<nc; ii++)
		{

		if( alpha_dual*dlam[ii]>lam[ii] )
			{
			alpha_dual = lam[ii] / dlam[ii];
			}
		if( alpha_prim*dt[ii]>t[ii] )
			{
			alpha_prim = t[ii] / dt[ii];
			}

		}

#if 0
	y_alpha0 = _mm256_cvtps_pd( s_alpha0 );
	y_alpha1 = _mm256_cvtps_pd( s_alpha2 );
#endif
	y_alpha0 = d_alpha0;
	y_alpha1 = d_alpha2;
	x_alpha0 = _mm_max_pd( _mm256_extractf128_pd( y_alpha0, 0x1 ), _mm256_castpd256_pd128( y_alpha0 ) );
	x_alpha1 = _mm_max_pd( _mm256_extractf128_pd( y_alpha1, 0x1 ), _mm256_castpd256_pd128( y_alpha1 ) );
	x_alpha0 = _mm_max_sd( x_alpha0, _mm_permute_pd( x_alpha0, 0x1 ) );
	x_alpha1 = _mm_max_sd( x_alpha1, _mm_permute_pd( x_alpha1, 0x1 ) );
	x_alpha0 = _mm_max_sd( x_alpha0, _mm_load_sd( &alpha_dual ) );
	x_alpha1 = _mm_max_sd( x_alpha1, _mm_load_sd( &alpha_prim ) );
	_mm_store_sd( &alpha_dual, x_alpha0 );
	_mm_store_sd( &alpha_prim, x_alpha1 );

	alpha = alpha_prim>alpha_dual ? alpha_prim : alpha_dual;

	// store alpha
	cws->alpha_prim = - alpha_prim;
	cws->alpha_dual = - alpha_dual;
	cws->alpha = - alpha;

	return;

	}
	


void d_update_var_qp(struct d_core_qp_ipm_workspace *cws)
	{
	
	// extract workspace members
	int nv = cws->nv;
	int ne = cws->ne;
	int nc = cws->nc;

	double *v = cws->v;
	double *pi = cws->pi;
	double *lam = cws->lam;
	double *t = cws->t;
	double *v_bkp = cws->v_bkp;
	double *pi_bkp = cws->pi_bkp;
	double *lam_bkp = cws->lam_bkp;
	double *t_bkp = cws->t_bkp;
	double *dv = cws->dv;
	double *dpi = cws->dpi;
	double *dlam = cws->dlam;
	double *dt = cws->dt;
	double alpha = cws->alpha;
	double alpha_prim = cws->alpha_prim;
	double alpha_dual = cws->alpha_dual;

	double tmp_alpha_prim, tmp_alpha_dual;

	__m256d
		x_tmp0, x_tmp1,
		y_tmp0, y_tmp1,
		y_alpha, y_alpha_prim, y_alpha_dual,
		y_lam_min, y_t_min, y_mask0, y_mask1;
	
#if 0
	if(alpha<1.0)
		alpha *= 0.995;
#else
	if(alpha<1.0)
		{
		alpha_prim = alpha_prim * ((1.0-alpha_prim)*0.99 + alpha_prim*0.9999999);
		alpha_dual = alpha_dual * ((1.0-alpha_dual)*0.99 + alpha_dual*0.9999999);
		alpha = alpha * ((1.0-alpha)*0.99 + alpha*0.9999999);
		}
#endif

	// local variables
	int ii;

	if(cws->split_step==0)
		{
		tmp_alpha_prim = alpha;
		tmp_alpha_dual = alpha;
		}
	else
		{
		tmp_alpha_prim = alpha_prim;
		tmp_alpha_dual = alpha_dual;
		}

	y_alpha_prim = _mm256_broadcast_sd( &tmp_alpha_prim );
	y_alpha_dual = _mm256_broadcast_sd( &tmp_alpha_dual );

	// update v
	ii = 0;
	for(; ii<nv-3; ii+=4)
		{
		x_tmp0 = _mm256_loadu_pd( &v[ii] );
		_mm256_storeu_pd( &v_bkp[ii], x_tmp0 );
		y_tmp0 = _mm256_mul_pd( y_alpha_prim, _mm256_loadu_pd( &dv[ii] ) );
		y_tmp0 = _mm256_add_pd( y_tmp0, x_tmp0 );
		_mm256_storeu_pd( &v[ii], y_tmp0 );
		}
	for(; ii<nv; ii++)
		{
		v_bkp[ii] = v[ii];
		v[ii] += tmp_alpha_prim * dv[ii];
		}

	// update pi
	ii = 0;
	for(; ii<ne-3; ii+=4)
		{
		x_tmp0 = _mm256_loadu_pd( &pi[ii] );
		_mm256_storeu_pd( &pi_bkp[ii], x_tmp0 );
		y_tmp0 = _mm256_mul_pd( y_alpha_dual, _mm256_loadu_pd( &dpi[ii] ) );
		y_tmp0 = _mm256_add_pd( y_tmp0, x_tmp0 );
		_mm256_storeu_pd( &pi[ii], y_tmp0 );
		}
	for(; ii<ne; ii++)
		{
		pi_bkp[ii] = pi[ii];
		pi[ii] += tmp_alpha_dual * dpi[ii];
		}

	if(cws->t_lam_min==2)
		{

		// update lam and t
		y_lam_min = _mm256_broadcast_sd( &cws->lam_min );
		y_t_min = _mm256_broadcast_sd( &cws->t_min );
		ii = 0;
		for(; ii<nc-3; ii+=4)
			{
			x_tmp0 = _mm256_loadu_pd( &lam[ii] );
			x_tmp1 = _mm256_loadu_pd( &t[ii] );
			_mm256_storeu_pd( &lam_bkp[ii], x_tmp0 );
			_mm256_storeu_pd( &t_bkp[ii], x_tmp1 );
			y_tmp0 = _mm256_mul_pd( y_alpha_dual, _mm256_loadu_pd( &dlam[ii] ) );
			y_tmp1 = _mm256_mul_pd( y_alpha_prim, _mm256_loadu_pd( &dt[ii] ) );
			y_tmp0 = _mm256_add_pd( y_tmp0, x_tmp0 );
			y_tmp1 = _mm256_add_pd( y_tmp1, x_tmp1 );
			// max does not preserve NaN !!!
	//		y_tmp0 = _mm256_max_pd( y_tmp0, y_lam_min );
	//		y_tmp1 = _mm256_max_pd( y_tmp1, y_t_min );
			y_mask0 = _mm256_cmp_pd( y_tmp0, y_lam_min, 2 );
			y_mask1 = _mm256_cmp_pd( y_tmp1, y_t_min, 2 );
			y_tmp0 = _mm256_blendv_pd( y_tmp0, y_lam_min, y_mask0 );
			y_tmp1 = _mm256_blendv_pd( y_tmp1, y_t_min, y_mask1 );
			_mm256_storeu_pd( &lam[ii], y_tmp0 );
			_mm256_storeu_pd( &t[ii], y_tmp1 );
			}
		for(; ii<nc; ii++)
			{
			lam_bkp[ii] = lam[ii];
			t_bkp[ii] = t[ii];
			lam[ii] += tmp_alpha_dual * dlam[ii];
			t[ii] += tmp_alpha_prim * dt[ii];
			lam[ii] = lam[ii]<=cws->lam_min ? cws->lam_min : lam[ii];
			t[ii] = t[ii]<=cws->t_min ? cws->t_min : t[ii];
			}

		}
	else // split step
		{

		// update lam and t
//		y_lam_min = _mm256_broadcast_sd( &cws->lam_min );
//		y_t_min = _mm256_broadcast_sd( &cws->t_min );
		ii = 0;
		for(; ii<nc-3; ii+=4)
			{
			x_tmp0 = _mm256_loadu_pd( &lam[ii] );
			x_tmp1 = _mm256_loadu_pd( &t[ii] );
			_mm256_storeu_pd( &lam_bkp[ii], x_tmp0 );
			_mm256_storeu_pd( &t_bkp[ii], x_tmp1 );
			y_tmp0 = _mm256_mul_pd( y_alpha_dual, _mm256_loadu_pd( &dlam[ii] ) );
			y_tmp1 = _mm256_mul_pd( y_alpha_prim, _mm256_loadu_pd( &dt[ii] ) );
			y_tmp0 = _mm256_add_pd( y_tmp0, x_tmp0 );
			y_tmp1 = _mm256_add_pd( y_tmp1, x_tmp1 );
			// max does not preserve NaN !!!
	//		y_tmp0 = _mm256_max_pd( y_tmp0, y_lam_min );
	//		y_tmp1 = _mm256_max_pd( y_tmp1, y_t_min );
//			y_mask0 = _mm256_cmp_pd( y_tmp0, y_lam_min, 2 );
//			y_mask1 = _mm256_cmp_pd( y_tmp1, y_t_min, 2 );
//			y_tmp0 = _mm256_blendv_pd( y_tmp0, y_lam_min, y_mask0 );
//			y_tmp1 = _mm256_blendv_pd( y_tmp1, y_t_min, y_mask1 );
			_mm256_storeu_pd( &lam[ii], y_tmp0 );
			_mm256_storeu_pd( &t[ii], y_tmp1 );
			}
		for(; ii<nc; ii++)
			{
			lam_bkp[ii] = lam[ii];
			t_bkp[ii] = t[ii];
			lam[ii] += tmp_alpha_dual * dlam[ii];
			t[ii] += tmp_alpha_prim * dt[ii];
//			lam[ii] = lam[ii]<=cws->lam_min ? cws->lam_min : lam[ii];
//			t[ii] = t[ii]<=cws->t_min ? cws->t_min : t[ii];
			}

		}

	return;

	}



void d_compute_mu_aff_qp(struct d_core_qp_ipm_workspace *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	double *lam = cws->lam;
	double *t = cws->t;
	double *dlam = cws->dlam;
	double *dt = cws->dt;
	double alpha = cws->alpha;
	// this affects the minimum value of signa !!!
//		alpha *= 0.99;

	__m256d
		y_tmp0, y_tmp1,
		y_alpha, y_mu;
	
	__m128d
		x_mu;

	double mu = 0;

	y_mu = _mm256_setzero_pd( );

	y_alpha = _mm256_broadcast_sd( &alpha );

	ii = 0;
	for(; ii<nc-3; ii+=4)
		{
		y_tmp0 = _mm256_mul_pd( y_alpha, _mm256_loadu_pd( &dlam[ii] ) );
		y_tmp1 = _mm256_mul_pd( y_alpha, _mm256_loadu_pd( &dt[ii] ) );
		y_tmp0 = _mm256_add_pd( y_tmp0, _mm256_loadu_pd( &lam[ii] ) );
		y_tmp1 = _mm256_add_pd( y_tmp1, _mm256_loadu_pd( &t[ii] ) );
		y_tmp0 = _mm256_mul_pd( y_tmp0, y_tmp1 );
		y_mu = _mm256_add_pd( y_mu, y_tmp0 );
		}
	for(; ii<nc; ii++)
		{
		mu += (lam[ii] + alpha*dlam[ii]) * (t[ii] + alpha*dt[ii]);
		}
	
	x_mu = _mm_add_pd( _mm256_castpd256_pd128( y_mu ), _mm256_extractf128_pd( y_mu, 0x1 ) );
	x_mu = _mm_hadd_pd( x_mu, x_mu );
	x_mu = _mm_add_sd( x_mu, _mm_load_sd( &mu ) );
	_mm_store_sd( &mu, x_mu );

	cws->mu_aff = mu*cws->nc_mask_inv;

	return;

	}



void d_backup_res_m(struct d_core_qp_ipm_workspace *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	double *res_m = cws->res_m;
	double *res_m_bkp = cws->res_m_bkp;

	__m256d
		y_tmp0;

	ii = 0;
	for(; ii<nc-3; ii+=4)
		{
		y_tmp0 = _mm256_loadu_pd( &res_m[ii] );
		_mm256_storeu_pd( &res_m_bkp[ii], y_tmp0 );
		}
	for(; ii<nc; ii++)
		{
		res_m_bkp[ii] = res_m[ii];
		}

	return;

	}



void d_compute_centering_correction_qp(struct d_core_qp_ipm_workspace *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	double *dlam = cws->dlam;
	double *dt = cws->dt;
	double *res_m = cws->res_m;
	double *res_m_bkp = cws->res_m_bkp;

	__m256d
		y_tmp0,
		y_sigma_mu;

	double sigma_mu = cws->sigma*cws->mu;
	sigma_mu = sigma_mu>cws->tau_min ? sigma_mu : cws->tau_min;

	y_sigma_mu = _mm256_broadcast_sd( &sigma_mu );

	ii = 0;
	for(; ii<nc-3; ii+=4)
		{
		y_tmp0 = _mm256_mul_pd( _mm256_loadu_pd( &dt[ii] ), _mm256_loadu_pd( &dlam[ii] ) );
		y_tmp0 = _mm256_add_pd( y_tmp0, _mm256_loadu_pd( &res_m_bkp[ii] ) );
		y_tmp0 = _mm256_sub_pd( y_tmp0, y_sigma_mu );
		_mm256_storeu_pd( &res_m[ii], y_tmp0 );
		}
	for(; ii<nc; ii++)
		{
		res_m[ii] = res_m_bkp[ii] + dt[ii] * dlam[ii] - sigma_mu;
		}

	return;

	}



void d_compute_centering_qp(struct d_core_qp_ipm_workspace *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	double *res_m = cws->res_m;
	double *res_m_bkp = cws->res_m_bkp;

	__m256d
		y_tmp0,
		y_sigma_mu;

	double sigma_mu = cws->sigma*cws->mu;
	sigma_mu = sigma_mu>cws->tau_min ? sigma_mu : cws->tau_min;

	y_sigma_mu = _mm256_broadcast_sd( &sigma_mu );

	ii = 0;
	for(; ii<nc-3; ii+=4)
		{
		y_tmp0 = _mm256_sub_pd( _mm256_loadu_pd( &res_m_bkp[ii] ), y_sigma_mu );
		_mm256_storeu_pd( &res_m[ii], y_tmp0 );
		}
	for(; ii<nc; ii++)
		{
		res_m[ii] = res_m_bkp[ii] - sigma_mu;
		}

	return;

	}



void d_compute_tau_min_qp(struct d_core_qp_ipm_workspace *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	double *res_m = cws->res_m;
	double *res_m_bkp = cws->res_m_bkp;

	__m256d
		y_tmp0,
		y_tau_min;

	double tau_min = cws->tau_min;

	y_tau_min = _mm256_broadcast_sd( &tau_min );

	ii = 0;
	for(; ii<nc-3; ii+=4)
		{
		y_tmp0 = _mm256_sub_pd( _mm256_loadu_pd( &res_m_bkp[ii] ), y_tau_min );
		_mm256_storeu_pd( &res_m[ii], y_tmp0 );
		}
	for(; ii<nc; ii++)
		{
		res_m[ii] = res_m_bkp[ii] - tau_min;
		}

	return;

	}



