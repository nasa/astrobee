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



void COMPUTE_GAMMA_GAMMA_QP(REAL *res_d, REAL *res_m, struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int nc = cws->nc;

	REAL *lam = cws->lam;
	REAL *t = cws->t;
//	REAL *res_d = cws->res_d; // TODO rename d ???
//	REAL *res_m = cws->res_m; // TODO rename m ???
	REAL *t_inv = cws->t_inv;
	REAL *Gamma = cws->Gamma;
	REAL *gamma = cws->gamma;
	REAL lam_min = cws->lam_min;
	REAL t_min = cws->t_min;
	REAL t_min_inv = cws->t_min_inv;
	REAL lam0, t0, t_inv_tmp, lam_tmp;

	// local variables
	int ii;

//printf("\ngamma gamma\n");
	if(cws->t_lam_min==1)
		{
		for(ii=0; ii<nc; ii++)
			{
			lam0 = lam[ii];
			t0 = t[ii];
			t_inv[ii] = 1.0/t0;
			t_inv_tmp = t0<t_min ? t_min_inv : t_inv[ii];
			lam_tmp = lam0<lam_min ? lam_min : lam0;
			Gamma[ii] = t_inv_tmp*lam_tmp;
			gamma[ii] = t_inv[ii]*(res_m[ii]-lam0*res_d[ii]);
			}
		}
	else
		{
		for(ii=0; ii<nc; ii++)
			{
			lam0 = lam[ii];
			t0 = t[ii];
			t_inv[ii] = 1.0/t0;
			Gamma[ii] = t_inv[ii]*lam0;
			gamma[ii] = t_inv[ii]*(res_m[ii]-lam0*res_d[ii]);
			}
		}

	return;

	}



void COMPUTE_GAMMA_QP(REAL *res_d, REAL *res_m, struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int nc = cws->nc;

	REAL *lam = cws->lam;
//	REAL *res_m = cws->res_m;
//	REAL *res_d = cws->res_d;
	REAL *t_inv = cws->t_inv;
	REAL *gamma = cws->gamma;
	REAL lam_min = cws->lam_min;
	REAL lam0;

	// local variables
	int ii;

	for(ii=0; ii<nc; ii++)
		{
		lam0 = lam[ii];
//		lam0 = lam0<lam_min ? lam_min : lam0;
		gamma[ii] = t_inv[ii]*(res_m[ii]-lam0*res_d[ii]);
		}

	return;

	}

void COMPUTE_LAM_T_QP(REAL *res_d, REAL *res_m, REAL *dlam, REAL *dt, struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int nc = cws->nc;

	REAL *lam = cws->lam;
	REAL *t_inv = cws->t_inv;
	REAL lam_min = cws->lam_min;
	REAL lam0;

	// local variables
	int ii;

	for(ii=0; ii<nc; ii++)
		{
		lam0 = lam[ii];
//		lam0 = lam0<lam_min ? lam_min : lam0;
		dlam[ii] = - t_inv[ii] * (res_m[ii] + (lam0*dt[ii]) - (lam0*res_d[ii]));
		dt[ii] -= res_d[ii];
		// TODO compute lamda alone ???
//		dlam[ii] = - t_inv[ii] * (lam0*dt[ii] + res_m[ii]);
		}

	return;

	}



void COMPUTE_ALPHA_QP(struct CORE_QP_IPM_WORKSPACE *cws)
	{
	
	// extract workspace members
	int nc = cws->nc;

	REAL *lam = cws->lam;
	REAL *t = cws->t;
	REAL *dlam = cws->dlam;
	REAL *dt = cws->dt;

	REAL alpha_prim = - 1.0;
	REAL alpha_dual = - 1.0;
	REAL alpha = - 1.0;

#if 1


	// local variables
	int ii;

	for(ii=0; ii<nc; ii++)
		{

		if( alpha_dual*dlam[ii+0]>lam[ii+0] )
			{
			alpha_dual = lam[ii+0] / dlam[ii+0];
			}
		if( alpha_prim*dt[ii+0]>t[ii+0] )
			{
			alpha_prim = t[ii+0] / dt[ii+0];
			}

		}

#else // fraction to the boundary

	REAL mu = cws->mu;
	REAL tau = 1.0;
//	REAL tau = 0.995;

	tau = tau>(1-mu) ? tau : 1-mu;
	
	// local variables
	int ii;

	for(ii=0; ii<nc; ii++)
		{

		if( alpha_dual*dlam[ii+0]>tau*lam[ii+0] )
			{
			alpha_dual = tau*lam[ii+0] / dlam[ii+0];
			}
		if( alpha_prim*dt[ii+0]>tau*t[ii+0] )
			{
			alpha_prim = tau*t[ii+0] / dt[ii+0];
			}

		}

#endif
	alpha = alpha_prim>alpha_dual ? alpha_prim : alpha_dual;

	// store alpha
	cws->alpha_prim = - alpha_prim;
	cws->alpha_dual = - alpha_dual;
	cws->alpha = - alpha;

	return;

	}
	


void UPDATE_VAR_QP(struct CORE_QP_IPM_WORKSPACE *cws)
	{
	
	// extract workspace members
	int nv = cws->nv;
	int ne = cws->ne;
	int nc = cws->nc;

	REAL *v = cws->v;
	REAL *pi = cws->pi;
	REAL *lam = cws->lam;
	REAL *t = cws->t;
	REAL *v_bkp = cws->v_bkp;
	REAL *pi_bkp = cws->pi_bkp;
	REAL *lam_bkp = cws->lam_bkp;
	REAL *t_bkp = cws->t_bkp;
	REAL *dv = cws->dv;
	REAL *dpi = cws->dpi;
	REAL *dlam = cws->dlam;
	REAL *dt = cws->dt;
	REAL alpha = cws->alpha;
	REAL alpha_prim = cws->alpha_prim;
	REAL alpha_dual = cws->alpha_dual;
	REAL lam_min = cws->lam_min;
	REAL t_min = cws->t_min;

	REAL tmp_alpha_prim, tmp_alpha_dual;
	
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

	// update v
	for(ii=0; ii<nv; ii++)
		{
		v_bkp[ii] = v[ii];
		v[ii] += tmp_alpha_prim * dv[ii];
		}

	// update pi
	for(ii=0; ii<ne; ii++)
		{
		pi_bkp[ii] = pi[ii];
		pi[ii] += tmp_alpha_dual * dpi[ii];
		}

	if(cws->t_lam_min==2)
		{
		// update lam
		for(ii=0; ii<nc; ii++)
			{
			lam_bkp[ii] = lam[ii];
			lam[ii] += tmp_alpha_dual * dlam[ii];
			lam[ii] = lam[ii]<=lam_min ? lam_min : lam[ii];
			}

		// update t
		for(ii=0; ii<nc; ii++)
			{
			t_bkp[ii] = t[ii];
			t[ii] += tmp_alpha_prim * dt[ii];
			t[ii] = t[ii]<=t_min ? t_min : t[ii];
			}
		}
	else
		{
		// update lam
		for(ii=0; ii<nc; ii++)
			{
			lam_bkp[ii] = lam[ii];
			lam[ii] += tmp_alpha_dual * dlam[ii];
			}

		// update t
		for(ii=0; ii<nc; ii++)
			{
			t_bkp[ii] = t[ii];
			t[ii] += tmp_alpha_prim * dt[ii];
			}
		}

	return;

	}



void COMPUTE_MU_AFF_QP(struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	REAL *ptr_lam = cws->lam;
	REAL *ptr_t = cws->t;
	REAL *ptr_dlam = cws->dlam;
	REAL *ptr_dt = cws->dt;
	REAL alpha = cws->alpha;
	// this affects the minimum value of signa !!!
//		alpha *= 0.99;

	REAL mu = 0;

	for(ii=0; ii<nc; ii++)
		{
		mu += (ptr_lam[ii+0] + alpha*ptr_dlam[ii+0]) * (ptr_t[ii+0] + alpha*ptr_dt[ii+0]);
		}
	
	cws->mu_aff = mu*cws->nc_mask_inv;

	return;

	}



void BACKUP_RES_M(struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	REAL *ptr_res_m = cws->res_m;
	REAL *ptr_res_m_bkp = cws->res_m_bkp;

	for(ii=0; ii<nc; ii++)
		{
		ptr_res_m_bkp[ii+0] = ptr_res_m[ii+0];
		}

	return;

	}



void COMPUTE_CENTERING_CORRECTION_QP(struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	REAL *ptr_dlam = cws->dlam;
	REAL *ptr_dt = cws->dt;
	REAL *ptr_res_m = cws->res_m;
	REAL *ptr_res_m_bkp = cws->res_m_bkp;

	REAL sigma_mu = cws->sigma*cws->mu;
	sigma_mu = sigma_mu>cws->tau_min ? sigma_mu : cws->tau_min;

	for(ii=0; ii<nc; ii++)
		{
		ptr_res_m[ii+0] = ptr_res_m_bkp[ii+0] + ptr_dt[ii+0] * ptr_dlam[ii+0] - sigma_mu;
		}

	return;

	}



void COMPUTE_CENTERING_QP(struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	REAL *ptr_res_m = cws->res_m;
	REAL *ptr_res_m_bkp = cws->res_m_bkp;

	REAL sigma_mu = cws->sigma*cws->mu;
	sigma_mu = sigma_mu>cws->tau_min ? sigma_mu : cws->tau_min;

	for(ii=0; ii<nc; ii++)
		{
		ptr_res_m[ii+0] = ptr_res_m_bkp[ii+0] - sigma_mu;
		}

	return;

	}



void COMPUTE_TAU_MIN_QP(struct CORE_QP_IPM_WORKSPACE *cws)
	{

	int ii;

	// extract workspace members
	int nc = cws->nc;

	REAL *ptr_res_m = cws->res_m;
	REAL *ptr_res_m_bkp = cws->res_m_bkp;

	REAL tau_min = cws->tau_min;

	for(ii=0; ii<nc; ii++)
		{
		ptr_res_m[ii+0] = ptr_res_m_bkp[ii+0] - tau_min;
		}

	return;

	}



