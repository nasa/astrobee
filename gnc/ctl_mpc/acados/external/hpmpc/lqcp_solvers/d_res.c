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


#include "../include/d_blas_aux.h"
#include "../include/blas_d.h"
#include "../include/block_size.h"

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#endif


void d_back_ric_res_tv(int N, int *nx, int *nu, double **hpBAbt, double **hb, double **hpQ, double **hq, double **hux, double **hpi, double **hrq, double **hrb)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line

	static double temp[D_MR] = {};

	int ii, jj;
	
	int nu0, nu1, cnux0, nx0, nx1, nxm, cnx0, cnx1;


	// first block
	ii = 0;
	nu0 = nu[ii];
	nu1 = nu[ii+1];
	nx0 = nx[ii];
	nx1 = nx[ii+1];
	cnx1  = (nx1+ncl-1)/ncl*ncl;
//	cnz0  = (nu0+nx0+1+ncl-1)/ncl*ncl;
	cnux0 = (nu0+nx0+ncl-1)/ncl*ncl;
	
	for(jj=0; jj<nu0; jj++) 
		hrq[ii][jj] = - hq[ii][jj];
	if(nx0>0)
		{
		for(jj=0; jj<nu0%bs; jj++) 
			{ 
			temp[jj] = hux[ii][nu0/bs*bs+jj]; 
			hux[ii][nu0/bs*bs+jj] = 0.0; 
			}
#if defined(BLASFEO)
		dgemv_t_lib(nx0+nu0%bs, nu0, -1.0, hpQ[ii]+nu0/bs*bs*cnux0, cnux0, hux[ii]+nu0/bs*bs, 1.0, hrq[ii], hrq[ii]);
#else
		dgemv_t_lib(nx0+nu0%bs, nu0, hpQ[ii]+nu0/bs*bs*cnux0, cnux0, hux[ii]+nu0/bs*bs, -1, hrq[ii], hrq[ii]);
#endif
		for(jj=0; jj<nu0%bs; jj++) 
			hux[ii][nu0/bs*bs+jj] = temp[jj];
		}
#if defined(BLASFEO)
	dsymv_l_lib(nu0, nu0, -1.0, hpQ[ii], cnux0, hux[ii], 1.0, hrq[ii], hrq[ii]);
#else
	dsymv_lib(nu0, nu0, hpQ[ii], cnux0, hux[ii], -1, hrq[ii], hrq[ii]);
#endif
#if defined(BLASFEO)
	dgemv_n_lib(nu0, nx1, -1.0, hpBAbt[ii], cnx1, hpi[ii], 1.0, hrq[ii], hrq[ii]);
#else
	dgemv_n_lib(nu0, nx1, hpBAbt[ii], cnx1, hpi[ii], -1, hrq[ii], hrq[ii]);
#endif
	
	for(jj=0; jj<nx1; jj++) 
		hrb[ii][jj] = hux[ii+1][nu1+jj] - hb[ii][jj];
#if defined(BLASFEO)
	dgemv_t_lib(nu0+nx0, nx1, -1.0, hpBAbt[ii], cnx1, hux[ii], 1.0, hrb[ii], hrb[ii]);
#else
	dgemv_t_lib(nu0+nx0, nx1, hpBAbt[ii], cnx1, hux[ii], -1, hrb[ii], hrb[ii]);
#endif



	// middle blocks
	for(ii=1; ii<N; ii++)
		{
		nu0 = nu1;
		nu1 = nu[ii+1];
		nx0 = nx1;
		nx1 = nx[ii+1];
		cnx0 = cnx1;
		cnx1  = (nx1+ncl-1)/ncl*ncl;
//		cnz0  = (nu0+nx0+1+ncl-1)/ncl*ncl;
		cnux0 = (nu0+nx0+ncl-1)/ncl*ncl;

		for(jj=0; jj<nu0; jj++) 
			hrq[ii][jj] = - hq[ii][jj];
		for(jj=0; jj<nx0; jj++) 
			hrq[ii][nu0+jj] = - hq[ii][nu0+jj] + hpi[ii-1][jj];
#if defined(BLASFEO)
		dsymv_l_lib(nu0+nx0, nu0+nx0, -1.0, hpQ[ii], cnux0, hux[ii], 1.0, hrq[ii], hrq[ii]);
#else
		dsymv_lib(nu0+nx0, nu0+nx0, hpQ[ii], cnux0, hux[ii], -1, hrq[ii], hrq[ii]);
#endif

		for(jj=0; jj<nx1; jj++) 
			hrb[ii][jj] = hux[ii+1][nu1+jj] - hb[ii][jj];
#if defined(BLASFEO)
		dgemv_nt_lib(nu0+nx0, nx1, -1.0, -1.0, hpBAbt[ii], cnx1, hpi[ii], hux[ii], 1.0, 1.0, hrq[ii], hrb[ii], hrq[ii], hrb[ii]);
#else
		dgemv_nt_lib(nu0+nx0, nx1, hpBAbt[ii], cnx1, hpi[ii], hux[ii], -1, -1, hrq[ii], hrb[ii], hrq[ii], hrb[ii]);
#endif

		}
	


	// last block
	ii = N;
	nu0 = nu1;
	nx0 = nx1;
//	cnz0  = (nu0+nx0+1+ncl-1)/ncl*ncl;
	cnux0 = (nu0+nx0+ncl-1)/ncl*ncl;

	for(jj=0; jj<nx0; jj++) 
		hrq[ii][nu0+jj] = hpi[ii-1][jj] - hq[ii][nu0+jj];
#if defined(BLASFEO)
	dsymv_l_lib(nx0+nu0%bs, nx0+nu0%bs, -1.0, hpQ[ii]+nu0/bs*bs*cnux0+nu0/bs*bs*bs, cnux0, hux[ii]+nu0/bs*bs, 1.0, hrq[ii]+nu0/bs*bs, hrq[ii]+nu0/bs*bs);
#else
	dsymv_lib(nx0+nu0%bs, nx0+nu0%bs, hpQ[ii]+nu0/bs*bs*cnux0+nu0/bs*bs*bs, cnux0, hux[ii]+nu0/bs*bs, -1, hrq[ii]+nu0/bs*bs, hrq[ii]+nu0/bs*bs);
#endif

	}



void d_forward_schur_res_tv(int N, int *nv, int *ne, int *diag_hessian, double **hpQA, double **hqb, double **hxupi, double **hr)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii, jj, pnv0, cnv0, pnv1;

	// first stage
	ii = 0;
	pnv0 = (nv[ii]+bs-1)/bs*bs;
	cnv0 = (nv[ii]+ncl-1)/ncl*ncl;
	if(diag_hessian[ii])
		{
#if defined(BLASFEO)
		dgemv_nt_lib(ne[ii], nv[ii], 1.0, 1.0, hpQA[ii]+pnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1.0, 1.0, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
#else
		dgemv_nt_lib(ne[ii], nv[ii], hpQA[ii]+pnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1, 1, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
#endif
		dgemv_diag_lib(nv[ii], hpQA[ii], hxupi[ii], 1, hr[ii], hr[ii]);
		}
	else
		{
#if defined(BLASFEO)
		dgemv_nt_lib(ne[ii], nv[ii], 1.0, 1.0, hpQA[ii]+pnv0*cnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1.0, 1.0, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
		dsymv_l_lib(nv[ii], nv[ii], 1.0, hpQA[ii], cnv0, hxupi[ii], 1.0, hr[ii], hr[ii]);
#else
		dgemv_nt_lib(ne[ii], nv[ii], hpQA[ii]+pnv0*cnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1, 1, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
		dsymv_lib(nv[ii], nv[ii], hpQA[ii], cnv0, hxupi[ii], 1, hr[ii], hr[ii]);
#endif
		}
	for(jj=0; jj<ne[ii]; jj++) hr[ii][pnv0+jj] -= hxupi[ii+1][jj];

	// middle stages
	for(ii=1; ii<N; ii++)
		{
		pnv1 = pnv0;
		pnv0 = (nv[ii]+bs-1)/bs*bs;
		cnv0 = (nv[ii]+ncl-1)/ncl*ncl;
		if(diag_hessian[ii])
			{
#if defined(BLASFEO)
			dgemv_nt_lib(ne[ii], nv[ii], 1.0, 1.0, hpQA[ii]+pnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1.0, 1.0, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
#else
			dgemv_nt_lib(ne[ii], nv[ii], hpQA[ii]+pnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1, 1, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
#endif
			dgemv_diag_lib(nv[ii], hpQA[ii], hxupi[ii], 1, hr[ii], hr[ii]);
			}
		else
			{
#if defined(BLASFEO)
			dgemv_nt_lib(ne[ii], nv[ii], 1.0, 1.0, hpQA[ii]+pnv0*cnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1.0, 1.0, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
			dsymv_l_lib(nv[ii], nv[ii], 1.0, hpQA[ii], cnv0, hxupi[ii], 1.0, hr[ii], hr[ii]);
#else
			dgemv_nt_lib(ne[ii], nv[ii], hpQA[ii]+pnv0*cnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1, 1, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
			dsymv_lib(nv[ii], nv[ii], hpQA[ii], cnv0, hxupi[ii], 1, hr[ii], hr[ii]);
#endif
			}
		for(jj=0; jj<ne[ii-1]; jj++) hr[ii][jj] -= hxupi[ii-1][pnv1+jj];
		for(jj=0; jj<ne[ii]; jj++) hr[ii][pnv0+jj] -= hxupi[ii+1][jj];
		}

	// middle stages
	pnv1 = pnv0;
	pnv0 = (nv[ii]+bs-1)/bs*bs;
	cnv0 = (nv[ii]+ncl-1)/ncl*ncl;
	if(diag_hessian[ii])
		{
#if defined(BLASFEO)
		dgemv_nt_lib(ne[ii], nv[ii], 1.0, 1.0, hpQA[ii]+pnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1.0, 1.0, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
#else
		dgemv_nt_lib(ne[ii], nv[ii], hpQA[ii]+pnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1, 1, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
#endif
		dgemv_diag_lib(nv[ii], hpQA[ii], hxupi[ii], 1, hr[ii], hr[ii]);
		}
	else
		{
#if defined(BLASFEO)
		dgemv_nt_lib(ne[ii], nv[ii], 1.0, 1.0, hpQA[ii]+pnv0*cnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1.0, 1.0, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
		dsymv_l_lib(nv[ii], nv[ii], 1.0, hpQA[ii], cnv0, hxupi[ii], 1.0, hr[ii], hr[ii]);
#else
		dgemv_nt_lib(ne[ii], nv[ii], hpQA[ii]+pnv0*cnv0, cnv0, hxupi[ii], hxupi[ii]+pnv0, 1, 1, hqb[ii]+pnv0, hqb[ii], hr[ii]+pnv0, hr[ii]);
		dsymv_lib(nv[ii], nv[ii], hpQA[ii], cnv0, hxupi[ii], 1, hr[ii], hr[ii]);
#endif
		}
	for(jj=0; jj<ne[ii-1]; jj++) hr[ii][jj] -= hxupi[ii-1][pnv1+jj];

	}


