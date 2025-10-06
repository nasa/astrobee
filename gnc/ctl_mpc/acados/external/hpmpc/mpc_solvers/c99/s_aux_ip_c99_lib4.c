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
	
//	sigma_mu = 0;

	const int bs = 4; //d_get_mr();
	
	float temp0, temp1;
	
	float *ptr_t, *ptr_lam, *ptr_lamt, *ptr_dlam, *ptr_tinv, *ptr_pd, *ptr_pl, *ptr_pl2, *ptr_bd, *ptr_bl, *ptr_db;
	
	int ii, jj, ll, bs0;
	
	// first stage
	
	ptr_t     = t[0];
	ptr_lam   = lam[0];
	ptr_lamt  = lamt[0];
	ptr_dlam  = dlam[0];
	ptr_tinv  = t_inv[0];
	ptr_pd    = pd[0];
	ptr_pl    = pl[0];
	ptr_pl2   = pl2[0];
	ptr_bd    = bd[0];
	ptr_bl    = bl[0];
	ptr_db    = db[0];
	
	ii = 0;
	for(; ii<k0-3; ii+=4)
		{

		ptr_tinv[0] = 1.0/ptr_t[0];
		ptr_tinv[1] = 1.0/ptr_t[1];
		ptr_lamt[0] = ptr_lam[0]*ptr_tinv[0];
		ptr_lamt[1] = ptr_lam[1]*ptr_tinv[1];
		ptr_dlam[0] = ptr_tinv[0]*sigma_mu; // !!!!!
		ptr_dlam[1] = ptr_tinv[1]*sigma_mu; // !!!!!
		ptr_pd[0+(ii+0)*bs+ii*cnz] = ptr_bd[ii+0] + ptr_lamt[0] + ptr_lamt[1];
		ptr_pl[(ii+0)*bs] = ptr_bl[ii+0] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+0] - ptr_dlam[0];
		ptr_pl2[ii+0] = ptr_pl[(ii+0)*bs];

		ptr_tinv[2] = 1.0/ptr_t[2];
		ptr_tinv[3] = 1.0/ptr_t[3];
		ptr_lamt[2] = ptr_lam[2]*ptr_tinv[2];
		ptr_lamt[3] = ptr_lam[3]*ptr_tinv[3];
		ptr_dlam[2] = ptr_tinv[2]*sigma_mu; // !!!!!
		ptr_dlam[3] = ptr_tinv[3]*sigma_mu; // !!!!!
		ptr_pd[1+(ii+1)*bs+ii*cnz] = ptr_bd[ii+1] + ptr_lamt[2] + ptr_lamt[3];
		ptr_pl[(ii+1)*bs] = ptr_bl[ii+1] + ptr_lam[3] + ptr_lamt[3]*ptr_db[2*ii+3] + ptr_dlam[3] - ptr_lam[2] - ptr_lamt[2]*ptr_db[2*ii+2] - ptr_dlam[2];
		ptr_pl2[ii+1] = ptr_pl[(ii+1)*bs];

		ptr_tinv[4] = 1.0/ptr_t[4];
		ptr_tinv[5] = 1.0/ptr_t[5];
		ptr_lamt[4] = ptr_lam[4]*ptr_tinv[4];
		ptr_lamt[5] = ptr_lam[5]*ptr_tinv[5];
		ptr_dlam[4] = ptr_tinv[4]*sigma_mu; // !!!!!
		ptr_dlam[5] = ptr_tinv[5]*sigma_mu; // !!!!!
		ptr_pd[2+(ii+2)*bs+ii*cnz] = ptr_bd[ii+2] + ptr_lamt[4] + ptr_lamt[5];
		ptr_pl[(ii+2)*bs] = ptr_bl[ii+2] + ptr_lam[5] + ptr_lamt[5]*ptr_db[2*ii+5] + ptr_dlam[5] - ptr_lam[4] - ptr_lamt[4]*ptr_db[2*ii+4] - ptr_dlam[4];
		ptr_pl2[ii+2] = ptr_pl[(ii+2)*bs];

		ptr_tinv[6] = 1.0/ptr_t[6];
		ptr_tinv[7] = 1.0/ptr_t[7];
		ptr_lamt[6] = ptr_lam[6]*ptr_tinv[6];
		ptr_lamt[7] = ptr_lam[7]*ptr_tinv[7];
		ptr_dlam[6] = ptr_tinv[6]*sigma_mu; // !!!!!
		ptr_dlam[7] = ptr_tinv[7]*sigma_mu; // !!!!!
		ptr_pd[3+(ii+3)*bs+ii*cnz] = ptr_bd[ii+3] + ptr_lamt[6] + ptr_lamt[7];
		ptr_pl[(ii+3)*bs] = ptr_bl[ii+3] + ptr_lam[7] + ptr_lamt[7]*ptr_db[2*ii+7] + ptr_dlam[7] - ptr_lam[6] - ptr_lamt[6]*ptr_db[2*ii+6] - ptr_dlam[6];
		ptr_pl2[ii+3] = ptr_pl[(ii+3)*bs];

		ptr_t     += 8;
		ptr_lam   += 8;
		ptr_lamt  += 8;
		ptr_dlam  += 8;
		ptr_tinv  += 8;

		}
	if(ii<k0)
		{
		bs0 = k0-ii;
		for(ll=0; ll<bs0; ll++)
			{
			ptr_tinv[0] = 1.0/ptr_t[0];
			ptr_tinv[1] = 1.0/ptr_t[1];
			ptr_lamt[0] = ptr_lam[0]*ptr_tinv[0];
			ptr_lamt[1] = ptr_lam[1]*ptr_tinv[1];
			ptr_dlam[0] = ptr_tinv[0]*sigma_mu; // !!!!!
			ptr_dlam[1] = ptr_tinv[1]*sigma_mu; // !!!!!
			ptr_pd[ll+(ii+ll)*bs+ii*cnz] = ptr_bd[ii+ll] + ptr_lamt[0] + ptr_lamt[1];
			ptr_pl[(ii+ll)*bs] = ptr_bl[ii+ll] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+2*ll+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+2*ll+0] - ptr_dlam[0];
			ptr_pl2[ii+ll+0] = ptr_pl[(ii+ll)*bs];

			ptr_t     += 2;
			ptr_lam   += 2;
			ptr_lamt  += 2;
			ptr_dlam  += 2;
			ptr_tinv  += 2;
			}
		}

	// middle stages

	for(jj=1; jj<N; jj++)
		{
		
		ptr_t     = t[jj];
		ptr_lam   = lam[jj];
		ptr_lamt  = lamt[jj];
		ptr_dlam  = dlam[jj];
		ptr_tinv  = t_inv[jj];
		ptr_pd    = pd[jj];
		ptr_pl    = pl[jj];
		ptr_pl2   = pl2[jj];
		ptr_bd    = bd[jj];
		ptr_bl    = bl[jj];
		ptr_db    = db[jj];

		ii = 0;
		for(; ii<kmax-3; ii+=4)
			{
			ptr_tinv[0] = 1.0/ptr_t[0];
			ptr_tinv[1] = 1.0/ptr_t[1];
			ptr_lamt[0] = ptr_lam[0]*ptr_tinv[0];
			ptr_lamt[1] = ptr_lam[1]*ptr_tinv[1];
			ptr_dlam[0] = ptr_tinv[0]*sigma_mu; // !!!!!
			ptr_dlam[1] = ptr_tinv[1]*sigma_mu; // !!!!!
			ptr_pd[0+(ii+0)*bs+ii*cnz] = ptr_bd[ii+0] + ptr_lamt[0] + ptr_lamt[1];
			ptr_pl[(ii+0)*bs] = ptr_bl[ii+0] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+0] - ptr_dlam[0];
			ptr_pl2[ii+0] = ptr_pl[(ii+0)*bs];

			ptr_tinv[2] = 1.0/ptr_t[2];
			ptr_tinv[3] = 1.0/ptr_t[3];
			ptr_lamt[2] = ptr_lam[2]*ptr_tinv[2];
			ptr_lamt[3] = ptr_lam[3]*ptr_tinv[3];
			ptr_dlam[2] = ptr_tinv[2]*sigma_mu; // !!!!!
			ptr_dlam[3] = ptr_tinv[3]*sigma_mu; // !!!!!
			ptr_pd[1+(ii+1)*bs+ii*cnz] = ptr_bd[ii+1] + ptr_lamt[2] + ptr_lamt[3];
			ptr_pl[(ii+1)*bs] = ptr_bl[ii+1] + ptr_lam[3] + ptr_lamt[3]*ptr_db[2*ii+3] + ptr_dlam[3] - ptr_lam[2] - ptr_lamt[2]*ptr_db[2*ii+2] - ptr_dlam[2];
			ptr_pl2[ii+1] = ptr_pl[(ii+1)*bs];

			ptr_tinv[4] = 1.0/ptr_t[4];
			ptr_tinv[5] = 1.0/ptr_t[5];
			ptr_lamt[4] = ptr_lam[4]*ptr_tinv[4];
			ptr_lamt[5] = ptr_lam[5]*ptr_tinv[5];
			ptr_dlam[4] = ptr_tinv[4]*sigma_mu; // !!!!!
			ptr_dlam[5] = ptr_tinv[5]*sigma_mu; // !!!!!
			ptr_pd[2+(ii+2)*bs+ii*cnz] = ptr_bd[ii+2] + ptr_lamt[4] + ptr_lamt[5];
			ptr_pl[(ii+2)*bs] = ptr_bl[ii+2] + ptr_lam[5] + ptr_lamt[5]*ptr_db[2*ii+5] + ptr_dlam[5] - ptr_lam[4] - ptr_lamt[4]*ptr_db[2*ii+4] - ptr_dlam[4];
			ptr_pl2[ii+2] = ptr_pl[(ii+2)*bs];

			ptr_tinv[6] = 1.0/ptr_t[6];
			ptr_tinv[7] = 1.0/ptr_t[7];
			ptr_lamt[6] = ptr_lam[6]*ptr_tinv[6];
			ptr_lamt[7] = ptr_lam[7]*ptr_tinv[7];
			ptr_dlam[6] = ptr_tinv[6]*sigma_mu; // !!!!!
			ptr_dlam[7] = ptr_tinv[7]*sigma_mu; // !!!!!
			ptr_pd[3+(ii+3)*bs+ii*cnz] = ptr_bd[ii+3] + ptr_lamt[6] + ptr_lamt[7];
			ptr_pl[(ii+3)*bs] = ptr_bl[ii+3] + ptr_lam[7] + ptr_lamt[7]*ptr_db[2*ii+7] + ptr_dlam[7] - ptr_lam[6] - ptr_lamt[6]*ptr_db[2*ii+6] - ptr_dlam[6];
			ptr_pl2[ii+3] = ptr_pl[(ii+3)*bs];

			ptr_t     += 8;
			ptr_lam   += 8;
			ptr_lamt  += 8;
			ptr_dlam  += 8;
			ptr_tinv  += 8;

			}
		if(ii<kmax)
			{
			bs0 = kmax-ii;
			for(ll=0; ll<bs0; ll++)
				{
				ptr_tinv[0] = 1.0/ptr_t[0];
				ptr_tinv[1] = 1.0/ptr_t[1];
				ptr_lamt[0] = ptr_lam[0]*ptr_tinv[0];
				ptr_lamt[1] = ptr_lam[1]*ptr_tinv[1];
				ptr_dlam[0] = ptr_tinv[0]*sigma_mu; // !!!!!
				ptr_dlam[1] = ptr_tinv[1]*sigma_mu; // !!!!!
				ptr_pd[ll+(ii+ll)*bs+ii*cnz] = ptr_bd[ii+ll] + ptr_lamt[0] + ptr_lamt[1];
				ptr_pl[(ii+ll)*bs] = ptr_bl[ii+ll] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+2*ll+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+2*ll+0] - ptr_dlam[0];
				ptr_pl2[ii+ll+0] = ptr_pl[(ii+ll)*bs];

				ptr_t     += 2;
				ptr_lam   += 2;
				ptr_lamt  += 2;
				ptr_dlam  += 2;
				ptr_tinv  += 2;
				}
			}
	
		}

	// last stage

	ptr_t     = t[N]     + 2*k1;
	ptr_lam   = lam[N]   + 2*k1;
	ptr_lamt  = lamt[N]  + 2*k1;
	ptr_dlam  = dlam[N]  + 2*k1;
	ptr_tinv  = t_inv[N] + 2*k1;
	ptr_pd    = pd[N];
	ptr_pl    = pl[N];
	ptr_pl2   = pl2[N];
	ptr_bd    = bd[N];
	ptr_bl    = bl[N];
	ptr_db    = db[N];

	ii=k1; // k1 supposed to be multiple of bs !!!!!!!!!!

	for(; ii<kmax-3; ii+=4)
		{
		ptr_tinv[0] = 1.0/ptr_t[0];
		ptr_tinv[1] = 1.0/ptr_t[1];
		ptr_lamt[0] = ptr_lam[0]*ptr_tinv[0];
		ptr_lamt[1] = ptr_lam[1]*ptr_tinv[1];
		ptr_dlam[0] = ptr_tinv[0]*sigma_mu; // !!!!!
		ptr_dlam[1] = ptr_tinv[1]*sigma_mu; // !!!!!
		ptr_pd[0+(ii+0)*bs+ii*cnz] = ptr_bd[ii+0] + ptr_lamt[0] + ptr_lamt[1];
		ptr_pl[(ii+0)*bs] = ptr_bl[ii+0] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+0] - ptr_dlam[0];
		ptr_pl2[ii+0] = ptr_pl[(ii+0)*bs];

		ptr_tinv[2] = 1.0/ptr_t[2];
		ptr_tinv[3] = 1.0/ptr_t[3];
		ptr_lamt[2] = ptr_lam[2]*ptr_tinv[2];
		ptr_lamt[3] = ptr_lam[3]*ptr_tinv[3];
		ptr_dlam[2] = ptr_tinv[2]*sigma_mu; // !!!!!
		ptr_dlam[3] = ptr_tinv[3]*sigma_mu; // !!!!!
		ptr_pd[1+(ii+1)*bs+ii*cnz] = ptr_bd[ii+1] + ptr_lamt[2] + ptr_lamt[3];
		ptr_pl[(ii+1)*bs] = ptr_bl[ii+1] + ptr_lam[3] + ptr_lamt[3]*ptr_db[2*ii+3] + ptr_dlam[3] - ptr_lam[2] - ptr_lamt[2]*ptr_db[2*ii+2] - ptr_dlam[2];
		ptr_pl2[ii+1] = ptr_pl[(ii+1)*bs];

		ptr_tinv[4] = 1.0/ptr_t[4];
		ptr_tinv[5] = 1.0/ptr_t[5];
		ptr_lamt[4] = ptr_lam[4]*ptr_tinv[4];
		ptr_lamt[5] = ptr_lam[5]*ptr_tinv[5];
		ptr_dlam[4] = ptr_tinv[4]*sigma_mu; // !!!!!
		ptr_dlam[5] = ptr_tinv[5]*sigma_mu; // !!!!!
		ptr_pd[2+(ii+2)*bs+ii*cnz] = ptr_bd[ii+2] + ptr_lamt[4] + ptr_lamt[5];
		ptr_pl[(ii+2)*bs] = ptr_bl[ii+2] + ptr_lam[5] + ptr_lamt[5]*ptr_db[2*ii+5] + ptr_dlam[5] - ptr_lam[4] - ptr_lamt[4]*ptr_db[2*ii+4] - ptr_dlam[4];
		ptr_pl2[ii+2] = ptr_pl[(ii+2)*bs];

		ptr_tinv[6] = 1.0/ptr_t[6];
		ptr_tinv[7] = 1.0/ptr_t[7];
		ptr_lamt[6] = ptr_lam[6]*ptr_tinv[6];
		ptr_lamt[7] = ptr_lam[7]*ptr_tinv[7];
		ptr_dlam[6] = ptr_tinv[6]*sigma_mu; // !!!!!
		ptr_dlam[7] = ptr_tinv[7]*sigma_mu; // !!!!!
		ptr_pd[3+(ii+3)*bs+ii*cnz] = ptr_bd[ii+3] + ptr_lamt[6] + ptr_lamt[7];
		ptr_pl[(ii+3)*bs] = ptr_bl[ii+3] + ptr_lam[7] + ptr_lamt[7]*ptr_db[2*ii+7] + ptr_dlam[7] - ptr_lam[6] - ptr_lamt[6]*ptr_db[2*ii+6] - ptr_dlam[6];
		ptr_pl2[ii+3] = ptr_pl[(ii+3)*bs];

		ptr_t     += 8;
		ptr_lam   += 8;
		ptr_lamt  += 8;
		ptr_dlam  += 8;
		ptr_tinv  += 8;

		}
	if(ii<kmax)
		{
		bs0 = kmax-ii;
		for(ll=0; ll<bs0; ll++)
			{
			ptr_tinv[0] = 1.0/ptr_t[0];
			ptr_tinv[1] = 1.0/ptr_t[1];
			ptr_lamt[0] = ptr_lam[0]*ptr_tinv[0];
			ptr_lamt[1] = ptr_lam[1]*ptr_tinv[1];
			ptr_dlam[0] = ptr_tinv[0]*sigma_mu; // !!!!!
			ptr_dlam[1] = ptr_tinv[1]*sigma_mu; // !!!!!
			ptr_pd[ll+(ii+ll)*bs+ii*cnz] = ptr_bd[ii+ll] + ptr_lamt[0] + ptr_lamt[1];
			ptr_pl[(ii+ll)*bs] = ptr_bl[ii+ll] + ptr_lam[1] + ptr_lamt[1]*ptr_db[2*ii+2*ll+1] + ptr_dlam[1] - ptr_lam[0] - ptr_lamt[0]*ptr_db[2*ii+2*ll+0] - ptr_dlam[0];
			ptr_pl2[ii+ll+0] = ptr_pl[(ii+ll)*bs];

			ptr_t     += 2;
			ptr_lam   += 2;
			ptr_lamt  += 2;
			ptr_dlam  += 2;
			ptr_tinv  += 2;
			}
		}


	}



void s_compute_alpha_box_mpc(int N, int k0, int k1, int kmax, float *ptr_alpha, float **t, float **dt, float **lam, float **dlam, float **lamt, float **dux, float **db)
	{
	
	const int bs = 4; //d_get_mr();
	
	float alpha = ptr_alpha[0];
	
	int kna = ((k1+bs-1)/bs)*bs;

	int jj, ll;


	// first stage

	ll = 0;
	for(; ll<k0; ll+=2)
		{

		dt[0][ll+0] =   dux[0][ll/2] - db[0][ll+0] - t[0][ll+0];
		dt[0][ll+1] = - dux[0][ll/2] - db[0][ll+1] - t[0][ll+1];
		dlam[0][ll+0] -= lamt[0][ll+0] * dt[0][ll+0] + lam[0][ll+0];
		dlam[0][ll+1] -= lamt[0][ll+1] * dt[0][ll+1] + lam[0][ll+1];
		if( -alpha*dlam[0][ll+0]>lam[0][ll+0] )
			{
			alpha = - lam[0][ll+0] / dlam[0][ll+0];
			}
		if( -alpha*dlam[0][ll+1]>lam[0][ll+1] )
			{
			alpha = - lam[0][ll+1] / dlam[0][ll+1];
			}
		if( -alpha*dt[0][ll+0]>t[0][ll+0] )
			{
			alpha = - t[0][ll+0] / dt[0][ll+0];
			}
		if( -alpha*dt[0][ll+1]>t[0][ll+1] )
			{
			alpha = - t[0][ll+1] / dt[0][ll+1];
			}

		}

	// middle stages
	for(jj=1; jj<N; jj++)
		{

		ll = 0;
		for(; ll<kmax; ll+=2)
			{

			dt[jj][ll+0] =   dux[jj][ll/2] - db[jj][ll+0] - t[jj][ll+0];
			dt[jj][ll+1] = - dux[jj][ll/2] - db[jj][ll+1] - t[jj][ll+1];
			dlam[jj][ll+0] -= lamt[jj][ll+0] * dt[jj][ll+0] + lam[jj][ll+0];
			dlam[jj][ll+1] -= lamt[jj][ll+1] * dt[jj][ll+1] + lam[jj][ll+1];
			if( -alpha*dlam[jj][ll+0]>lam[jj][ll+0] )
				{
				alpha = - lam[jj][ll+0] / dlam[jj][ll+0];
				}
			if( -alpha*dlam[jj][ll+1]>lam[jj][ll+1] )
				{
				alpha = - lam[jj][ll+1] / dlam[jj][ll+1];
				}
			if( -alpha*dt[jj][ll+0]>t[jj][ll+0] )
				{
				alpha = - t[jj][ll+0] / dt[jj][ll+0];
				}
			if( -alpha*dt[jj][ll+1]>t[jj][ll+1] )
				{
				alpha = - t[jj][ll+1] / dt[jj][ll+1];
				}

			}

		}		

	// last stage
	ll = k1;
	for(; ll<kmax; ll+=2)
		{

		dt[N][ll+0] =   dux[N][ll/2] - db[N][ll+0] - t[N][ll+0];
		dt[N][ll+1] = - dux[N][ll/2] - db[N][ll+1] - t[N][ll+1];
		dlam[N][ll+0] -= lamt[N][ll+0] * dt[N][ll+0] + lam[N][ll+0];
		dlam[N][ll+1] -= lamt[N][ll+1] * dt[N][ll+1] + lam[N][ll+1];
		if( -alpha*dlam[N][ll+0]>lam[N][ll+0] )
			{
			alpha = - lam[N][ll+0] / dlam[N][ll+0];
			}
		if( -alpha*dlam[N][ll+1]>lam[N][ll+1] )
			{
			alpha = - lam[N][ll+1] / dlam[N][ll+1];
			}
		if( -alpha*dt[N][ll+0]>t[N][ll+0] )
			{
			alpha = - t[N][ll+0] / dt[N][ll+0];
			}
		if( -alpha*dt[N][ll+1]>t[N][ll+1] )
			{
			alpha = - t[N][ll+1] / dt[N][ll+1];
			}

		}
	
	ptr_alpha[0] = alpha;

	return;
	
	}



void s_update_var_mpc(int nx, int nu, int N, int nb, int nbu, float *ptr_mu, float mu_scal, float alpha, float **ux, float **dux, float **t, float **dt, float **lam, float **dlam, float **pi, float **dpi)
	{
	
	int jj, ll;
	
	float mu = 0;

	// update inputs
	for(ll=0; ll<nu; ll++)
		ux[0][ll] += alpha*(dux[0][ll] - ux[0][ll]);
	// box constraints
	for(ll=0; ll<2*nbu; ll+=2)
		{
		lam[0][ll+0] += alpha*dlam[0][ll+0];
		lam[0][ll+1] += alpha*dlam[0][ll+1];
		t[0][ll+0] += alpha*dt[0][ll+0];
		t[0][ll+1] += alpha*dt[0][ll+1];
		mu += lam[0][ll+0] * t[0][ll+0] + lam[0][ll+1] * t[0][ll+1];
		}

	for(jj=1; jj<N; jj++)
		{
		// update inputs
		for(ll=0; ll<nu; ll++)
			ux[jj][ll] += alpha*(dux[jj][ll] - ux[jj][ll]);
		// update states
		for(ll=0; ll<nx; ll++)
			ux[jj][nu+ll] += alpha*(dux[jj][nu+ll] - ux[jj][nu+ll]);
		// update equality constrained multipliers
		for(ll=0; ll<nx; ll++)
			pi[jj][ll] += alpha*(dpi[jj][ll] - pi[jj][ll]);
		// box constraints
		for(ll=0; ll<2*nb; ll+=2)
			{
			lam[jj][ll+0] += alpha*dlam[jj][ll+0];
			lam[jj][ll+1] += alpha*dlam[jj][ll+1];
			t[jj][ll+0] += alpha*dt[jj][ll+0];
			t[jj][ll+1] += alpha*dt[jj][ll+1];
			mu += lam[jj][ll+0] * t[jj][ll+0] + lam[jj][ll+1] * t[jj][ll+1];
			}
		}

	// update states
	for(ll=0; ll<nx; ll++)
		ux[N][nu+ll] += alpha*(dux[N][nu+ll] - ux[N][nu+ll]);
	// update equality constrained multipliers
	for(ll=0; ll<nx; ll++)
		pi[N][ll] += alpha*(dpi[N][ll] - pi[N][ll]);
	// box constraints
	for(ll=2*nu; ll<2*nb; ll+=2)
		{
		lam[N][ll+0] += alpha*dlam[N][ll+0];
		lam[N][ll+1] += alpha*dlam[N][ll+1];
		t[N][ll+0] += alpha*dt[N][ll+0];
		t[N][ll+1] += alpha*dt[N][ll+1];
		mu += lam[N][ll+0] * t[N][ll+0] + lam[N][ll+1] * t[N][ll+1];
		}
	mu *= mu_scal;

	ptr_mu[0] = mu;

	return;
	
	}



void s_compute_mu_mpc(int N, int nbu, int nu, int nb, float *ptr_mu, float mu_scal, float alpha, float **lam, float **dlam, float **t, float **dt)
	{
	
	int jj, ll;
	
	float mu = 0;
	
	for(ll=0 ; ll<2*nbu; ll+=2)
		mu += (lam[0][ll+0] + alpha*dlam[0][ll+0]) * (t[0][ll+0] + alpha*dt[0][ll+0]) + (lam[0][ll+1] + alpha*dlam[0][ll+1]) * (t[0][ll+1] + alpha*dt[0][ll+1]);

	for(jj=1; jj<N; jj++)
		for(ll=0 ; ll<2*nb; ll+=2)
			mu += (lam[jj][ll+0] + alpha*dlam[jj][ll+0]) * (t[jj][ll+0] + alpha*dt[jj][ll+0]) + (lam[jj][ll+1] + alpha*dlam[jj][ll+1]) * (t[jj][ll+1] + alpha*dt[jj][ll+1]);

	for(ll=2*nu ; ll<2*nb; ll+=2)
		mu += (lam[N][ll+0] + alpha*dlam[N][ll+0]) * (t[N][ll+0] + alpha*dt[N][ll+0]) + (lam[N][ll+1] + alpha*dlam[N][ll+1]) * (t[N][ll+1] + alpha*dt[N][ll+1]);

	mu *= mu_scal;
		
	ptr_mu[0] = mu;

	return;

	}



