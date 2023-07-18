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

/*#include "../include/aux_d.h"*/
#include "../include/aux_d.h"
#include "../include/blas_d.h"
#include "../include/block_size.h"
#include "../include/lqcp_aux.h"



#if 1 // not allow for singular P

void d_back_ric_sv_new(int N, int nx, int nu, double **hpBAbt, double **hpQ, int update_hessian, double **hQd, double **hQl, int fixed_x0, double **hux, double **hpL, double **hdL, double *pLBAbtDCt, double *work, int compute_Pb, double **hPb, int compute_pi, double **hpi, int nb, int ng, int ngN, double **hpDCt, double **Qx, double **qx)
	{
	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	
	const int nz   = nx+nu+1;
	const int pnz  = bs*((nz+bs-1)/bs);
	const int pnx  = bs*((nx+bs-1)/bs);
	const int pnb  = bs*((nb+bs-1)/bs);
	const int png  = bs*((ng+bs-1)/bs);
	const int pngN = bs*((ngN+bs-1)/bs);
	const int cnz  = ncl*((nz+ncl-1)/ncl);
	const int cnx  = ncl*((nx+ncl-1)/ncl);
	const int cng  = ncl*((ng+ncl-1)/ncl);
	const int cngN = ncl*((ngN+ncl-1)/ncl);
	const int cnxg = ncl*((ng+nx+ncl-1)/ncl);

	const int cnl = cnz<cnx+ncl ? cnx+ncl : cnz;

	int nu0 = (nu/bs)*bs;

	int ii, jj, ll, nn;

	double temp;

	double diag_min = 1.0;

	// factorization and backward substitution 

//dgeset_lib(nz, nz, 0.0, 0, hpL[N], cnl);

	// final stage 
	if(ngN>0)
		{
		dgemv_n_lib(nu+nx, ngN, hpDCt[N], cngN, qx[N]+2*pnb, 1, hQl[N], hQl[N]);
		dgemm_diag_right_lib(nu+nx, ngN, hpDCt[N], cngN, Qx[N]+2*pnb, 0, pLBAbtDCt, cngN, pLBAbtDCt, cngN);
		for(jj=0; jj<ngN; jj++) pLBAbtDCt[(nu+nx)/bs*bs*cngN+(nu+nx)%bs+jj*bs] = 0.0;
		}
	if(update_hessian)
		{
		ddiain_lib(nu%bs+nx, hQd[N]+nu0, 0, hpQ[N]+nu0*cnz+nu0*bs, cnz);
		drowin_lib(nu%bs+nx, hQl[N]+nu0, hpQ[N]+nu0*bs+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
		}

	dsyrk_dpotrf_lib(nx+nu%bs+1, nx+nu%bs, ngN, pLBAbtDCt+(nu/bs)*bs*cngN, cngN, pLBAbtDCt+(nu/bs)*bs*cngN, cngN, 1, hpQ[N]+(nu/bs)*bs*cnz+(nu/bs)*bs*bs, cnz, hpL[N]+(nu/bs)*bs*cnl+(nu/bs)*bs*bs, cnl, hdL[N]+nu);

	dtrtr_l_lib(nx, nu, hpL[N]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[N]+(ncl)*bs, cnl);	



	// middle stages 
	for(nn=0; nn<N-1; nn++)
		{	

		dtrmm_nt_u_lib(nz, nx, hpBAbt[N-nn-1], cnx, hpL[N-nn]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);

		if(compute_Pb==1)
			{
			for(jj=0; jj<nx; jj++) work[jj] = pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs]; // backup in work !!!
			dtrmv_u_t_lib(nx, hpL[N-nn]+(ncl)*bs, cnl, work, 0, hPb[N-nn-1]); // L*(L'*b)
			}
		for(jj=0; jj<nx; jj++) pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs] += hpL[N-nn][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];

		if(ng>0)
			{
			dgemv_n_lib(nx+nu, ng, hpDCt[N-nn-1], cng, qx[N-nn-1]+2*pnb, 1, hQl[N-nn-1], hQl[N-nn-1]);
			dgemm_diag_right_lib(nx+nu, ng, hpDCt[N-nn-1], cng, Qx[N-nn-1]+2*pnb, 0, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg);
			for(jj=0; jj<ng; jj++)
				pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
			}
		if(update_hessian)
			{
			ddiain_lib(nx+nu, hQd[N-nn-1], 0, hpQ[N-nn-1], cnz);
			drowin_lib(nx+nu, hQl[N-nn-1], hpQ[N-nn-1]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
			}

		dsyrk_dpotrf_lib(nz, nu+nx, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[N-nn-1], cnz, hpL[N-nn-1], cnl, hdL[N-nn-1]);

		dtrtr_l_lib(nx, nu, hpL[N-nn-1]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[N-nn-1]+(ncl)*bs, cnl);	

		}



	if(fixed_x0==1) // mpc
		{

		// first stage 
		dtrmm_nt_u_lib(nz, nx, hpBAbt[0], cnx, hpL[1]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);
		if(compute_Pb)
			{
			for(jj=0; jj<nx; jj++) work[jj] = pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs];
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, work, 0, hPb[0]); // L*(L'*b)
			}
		for(jj=0; jj<nx; jj++) pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs] += hpL[1][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];
		if(ng>0)
			{
			dgemv_n_lib(nx+nu, ng, hpDCt[0], cng, qx[0]+2*pnb, 1, hQl[0], hQl[0]);
			dgemm_diag_right_lib(nx+nu, ng, hpDCt[0], cng, Qx[0]+2*pnb, 0, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg);
			for(jj=0; jj<ng; jj++)
				pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
			}
		if(update_hessian)
			{
			ddiain_lib(nu, hQd[0], 0, hpQ[0], cnz);
			drowin_lib(nu, hQl[0], hpQ[0]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
			}
		dsyrk_dpotrf_lib(nz, ((nu+2-1)/2)*2, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[0], cnz, hpL[0], cnl, hdL[0]);


		// forward substitution 
		// first stage
		nn = 0;
		for(jj=0; jj<nu; jj++) hux[0][jj] = - hpL[0][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*jj];
		dtrsv_t_lib(nx+nu, nu, hpL[0], cnl, 1, hdL[0], &hux[0][0], &hux[0][0]);
		for(jj=0; jj<nx; jj++) hux[1][nu+jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[0], cnx, &hux[0][0], 1, &hux[1][nu], &hux[1][nu]);
		if(compute_pi)
			{
			for(jj=0; jj<nx; jj++) hpi[1][jj] = hux[1][nu+jj]; // copy x into aligned memory
			for(jj=0; jj<nx; jj++) work[jj] = hpL[1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, &hpi[1][0], 1, &work[0]);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, &work[0], 0, &hpi[1][0]); // L*(L'*b) + p
			}

		}
	else // mhe
		{

		// first stage 
		dtrmm_nt_u_lib(nz, nx, hpBAbt[0], cnx, hpL[1]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);
		if(compute_Pb)
			{
			for(jj=0; jj<nx; jj++) work[jj] = pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs];
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, work, 0, hPb[0]); // L*(L'*b)
			}
		for(jj=0; jj<nx; jj++) pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs] += hpL[1][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];
		if(ng>0)
			{
			dgemv_n_lib(nx+nu, ng, hpDCt[0], cng, qx[0]+2*pnb, 1, hQl[0], hQl[0]);
			dgemm_diag_right_lib(nx+nu, ng, hpDCt[0], cng, Qx[0]+2*pnb, 0, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg);
			for(jj=0; jj<ng; jj++)
				pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
			}
		if(update_hessian)
			{
			ddiain_lib(nx+nu, hQd[0], 0, hpQ[0], cnz);
			drowin_lib(nx+nu, hQl[0], hpQ[0]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
			}
		dsyrk_dpotrf_lib(nz, nu+nx, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[0], cnz, hpL[0], cnl, hdL[0]);

		dtrtr_l_lib(nx, nu, hpL[0]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[0]+(ncl)*bs, cnl);	



		// forward substitution 
		// first stage
		nn = 0;
		drowex_lib(nu+nx, hpL[0]+(nx+nu)/bs*bs*cnl+(nu+nx)%bs, hux[0]);
		d_scale_mat(nu+nx, 1, -1.0, hux[0], 1);
		dtrsv_t_lib(nx+nu, nu+nx, hpL[0], cnl, 1, hdL[0], &hux[0][0], &hux[0][0]);
		for(jj=0; jj<nx; jj++) hux[1][nu+jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[0], cnx, &hux[0][0], 1, &hux[1][nu], &hux[1][nu]);
		if(compute_pi)
			{
			for(jj=0; jj<nx; jj++) hpi[1][jj] = hux[1][nu+jj]; // copy x into aligned memory
			for(jj=0; jj<nx; jj++) work[jj] = hpL[1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, &hpi[1][0], 1, &work[0]);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, &work[0], 0, &hpi[1][0]); // L*(L'*b) + p
			}

		}



	// final stages
	for(nn=1; nn<N; nn++)
		{
		for(jj=0; jj<nu; jj++) hux[nn][jj] = - hpL[nn][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*jj];
		dtrsv_t_lib(nx+nu, nu, hpL[nn], cnl, 1, hdL[nn], &hux[nn][0], &hux[nn][0]);
		for(jj=0; jj<nx; jj++) hux[nn+1][nu+jj] = hpBAbt[nn][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[nn], cnx, &hux[nn][0], 1, &hux[nn+1][nu], &hux[nn+1][nu]);
		if(compute_pi)
			{
			for(jj=0; jj<nx; jj++) hpi[nn+1][jj] = hux[nn+1][nu+jj]; // copy x into aligned memory
			for(jj=0; jj<nx; jj++) work[jj] = hpL[nn+1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
			dtrmv_u_n_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &hpi[nn+1][0], 1, &work[0]);
			dtrmv_u_t_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &work[0], 0, &hpi[nn+1][0]); // L*(L'*b) + p
			}
		}
	
	}

#else // allowing for singular P

void d_back_ric_sv_new(int N, int nx, int nu, double **hpBAbt, double **hpQ, int update_hessian, double **hQd, double **hQl, int fixed_x0, double **hux, double **hpL, double **hdL, double *pLBAbtDCt, double *work, int compute_Pb, double **hPb, int compute_pi, double **hpi, int nb, int ng, int ngN, double **hpDCt, double **Qx, double **qx)
	{
	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line
	
	const int nz   = nx+nu+1;
	const int pnz  = bs*((nz+bs-1)/bs);
	const int pnx  = bs*((nx+bs-1)/bs);
	const int pnb  = bs*((nb+bs-1)/bs);
	const int png  = bs*((ng+bs-1)/bs);
	const int pngN = bs*((ngN+bs-1)/bs);
	const int cnz  = ncl*((nz+ncl-1)/ncl);
	const int cnx  = ncl*((nx+ncl-1)/ncl);
	const int cng  = ncl*((ng+ncl-1)/ncl);
	const int cngN = ncl*((ngN+ncl-1)/ncl);
	const int cnxg = ncl*((ng+nx+ncl-1)/ncl);

	const int cnl = cnz<cnx+ncl ? cnx+ncl : cnz;

	int nu0 = (nu/bs)*bs;

	int ii, jj, ll, nn;

	double temp;

	double diag_min = 1.0;

	// factorization and backward substitution 

//dgeset_lib(nz, nz, 0.0, 0, hpL[N], cnl);

	// final stage 
	if(ngN>0)
		{
		dgemv_n_lib(nx+nu, ngN, hpDCt[N], cngN, qx[N]+2*pnb, hQl[N], hQl[N], 1);
		dgemm_diag_right_lib(nx+nu, ngN, hpDCt[N], cngN, Qx[N]+2*pnb, pLBAbtDCt, cngN, pLBAbtDCt, cngN, 0);
		for(jj=0; jj<ngN; jj++) pLBAbtDCt[nx/bs*cngN*bs+nx%bs+jj*bs] = 0.0;
		}
	if(update_hessian)
		{
		ddiain_lib(nu%bs+nx, hQd[N]+nu0, 0, hpQ[N]+nu0*cnz+nu0*bs, cnz);
		drowin_lib(nu%bs+nx, hQl[N]+nu0, hpQ[N]+nu0*bs+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
		}

	dsyrk_dpotrf_lib(nx+nu%bs+1, nx+nu%bs ngN, pLBAbtDCt+(nu/bs)*bs*cngN, cngN, pLBAbtDCt+(nu/bs)*bs*cngN, cngN, 1, hpQ[N]+(nu/bs)*bs*cnz+(nu/bs)*bs*bs, cnz, hpL[N]+(nu/bs)*bs*cnl+(nu/bs)*bs*bs, cnl, hdL[N]+nu);

	dtrtr_l_lib(nx, nu, hpL[N]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[N]+(ncl)*bs, cnl);	

	diag_min = d_min_mat(nx, 1, hdL[N], 1);

	if(diag_min==0.0) // compute again linear part !!!!!
		{
		for(jj=0; jj<nx; jj++) hpL[N][(nu+nx)/bs*bs*cnl+(nu+nx)%bs+(nu+jj)*bs] = hpQ[N][nx/bs*bs*cnz+nx%bs+jj*bs];
		}
	else // copy last part properly
		{
		for(jj=0; jj<nx; jj++) hpL[N][(nu+nx)/bs*bs*cnl+(nu+nx)%bs+(nu+jj)*bs] = hpL[N][(nx)/bs*bs*cnl+(nx)%bs+(jj)*bs];
		}

#if 0
	d_print_pmat(nz, nx+ng, bs, pLBAbtDCt, cnxg);
	d_print_pmat(nz, nz, bs, hpL[N], cnl);
	d_print_mat(1, ng, Qx[N]+2*pnb, 1);
	d_print_mat(1, ng, Qx[N]+2*pnb+png, 1);
	d_print_mat(1, ng, qx[N]+2*pnb, 1);
	d_print_mat(1, ng, qx[N]+2*pnb+png, 1);
	d_print_mat(1, ng, Qx[N-1]+2*pnb, 1);
	d_print_mat(1, ng, Qx[N-1]+2*pnb+png, 1);
	d_print_mat(1, ng, qx[N-1]+2*pnb, 1);
	d_print_mat(1, ng, qx[N-1]+2*pnb+png, 1);
	exit(1);
#endif


	// middle stages 
	for(nn=0; nn<N-1; nn++)
		{	

		if(diag_min!=0.0)
			{

			dtrmm_nt_u_lib(nz, nx, hpBAbt[N-nn-1], cnx, hpL[N-nn]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);
			for(jj=0; jj<nx; jj++) work[jj] = pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs]; // backup in work !!!
			if(compute_Pb==1)
				{
				dtrmv_u_t_lib(nx, hpL[N-nn]+(ncl)*bs, cnl, work, hPb[N-nn-1], 0); // L*(L'*b)
				}
			for(jj=0; jj<nx; jj++) pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs] += hpL[N-nn][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];

			if(ng>0)
				{
				dgemv_n_lib(nx+nu, ng, hpDCt[N-nn-1], cng, qx[N-nn-1]+2*pnb, hQl[N-nn-1], hQl[N-nn-1], 1);
				dgemm_diag_right_lib(nx+nu, ng, hpDCt[N-nn-1], cng, Qx[N-nn-1]+2*pnb, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg, 0);
				for(jj=0; jj<ng; jj++)
					pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
				}
			if(update_hessian)
				{
				ddiain_lib(nx+nu, hQd[N-nn-1], 0, hpQ[N-nn-1], cnz);
				drowin_lib(nx+nu, hQl[N-nn-1], hpQ[N-nn-1]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
				}

			dsyrk_dpotrf_lib(nz, nu+nx, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[N-nn-1], cnz, hpL[N-nn-1], cnl, hdL[N-nn-1]);

			dtrtr_l_lib(nx, nu, hpL[N-nn-1]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[N-nn-1]+(ncl)*bs, cnl);	

			diag_min = d_min_mat(nx+nu, 1, hdL[N-nn-1], 1);

			if(diag_min==0.0) // compute again linear part !!!!!
				{
				if(compute_Pb!=1)
					{
					dtrmv_u_t_lib(nx, hpL[N-nn]+(ncl)*bs, cnl, work, hPb[N-nn-1], 0); // L*(L'*b)
					}
				for(jj=0; jj<nx; jj++) work[jj] = hPb[N-nn-1][jj] + hpL[N-nn][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];
				drowex_lib(nu+nx, hpQ[N-nn-1]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs, work+pnz);
				dgemv_n_lib(nx+nu, nx, hpBAbt[N-nn-1], cnx, work, work+pnz, work+pnz, 1);
				dtrsv_n_lib(nu+nx, nu, hpL[N-nn-1], cnl, 1, hdL[N-nn-1], work+pnz, work+pnz);
				drowin_lib(nx, work+pnz+nu, hpL[N-nn-1]+(nu+nx)/bs*bs*cnl+(nu+nx)%bs+nu*bs);
				}

			}
		else
			{

			dtrmm_nt_u_lib(nu+nx, nx, hpBAbt[N-nn-1], cnx, hpL[N-nn]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);

			for(jj=0; jj<nx; jj++) work[jj] = hpBAbt[N-nn-1][((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs]; // copy b in aligned memory
			dtrmv_u_n_lib(nx, hpL[N-nn]+(ncl)*bs, cnl, work, work+anz, 0);
			dtrmv_u_t_lib(nx, hpL[N-nn]+(ncl)*bs, cnl, work+anz, hPb[N-nn-1], 0); // L*(L'*b)

			if(ng>0)
				{
				dgemv_n_lib(nx+nu, ng, hpDCt[N-nn-1], cng, qx[N-nn-1]+2*pnb, hQl[N-nn-1], hQl[N-nn-1], 1);
				dgemm_diag_right_lib(nx+nu, ng, hpDCt[N-nn-1], cng, Qx[N-nn-1]+2*pnb, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg, 0);
				for(jj=0; jj<ng; jj++)
					pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
				}
			if(update_hessian)
				{
				ddiain_lib(nx+nu, hQd[N-nn-1], 0, hpQ[N-nn-1], cnz);
				drowin_lib(nx+nu, hQl[N-nn-1], hpQ[N-nn-1]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
				}

			dsyrk_nt_lib(nu+nx, nu+nx, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, hpQ[N-nn-1], cnz, hpL[N-nn-1], cnl, 1);

			for(jj=0; jj<nx; jj++) work[jj] = hPb[N-nn-1][jj] + hpL[N-nn][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];
			drowex_lib(nu+nx, hpQ[N-nn-1]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs, work+pnz);
			dgemv_n_lib(nx+nu, nx, hpBAbt[N-nn-1], cnx, work, work+pnz, work+pnz, 1);
			drowin_lib(nu+nx, work+pnz, hpL[N-nn-1]+(nu+nx)/bs*bs*cnl+(nu+nx)%bs);

			dpotrf_lib(nz, nu+nx, hpL[N-nn-1], cnl, hpL[N-nn-1], cnl, hdL[N-nn-1]);

			dtrtr_l_lib(nx, nu, hpL[N-nn-1]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[N-nn-1]+(ncl)*bs, cnl);	

			diag_min = d_min_mat(nx+nu, 1, hdL[N-nn-1], 1);

			if(diag_min==0.0) // compute again linear part !!!!!
				{
				for(jj=0; jj<nx; jj++) work[jj] = hPb[N-nn-1][jj] + hpL[N-nn][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];
				drowex_lib(nu+nx, hpQ[N-nn-1]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs, work+pnz);
				dgemv_n_lib(nx+nu, nx, hpBAbt[N-nn-1], cnx, work, work+pnz, work+pnz, 1);
				dtrsv_n_lib(nu+nx, nu, hpL[N-nn-1], cnl, 1, hdL[N-nn-1], work+pnz, work+pnz);
				drowin_lib(nx, work+pnz+nu, hpL[N-nn-1]+(nu+nx)/bs*bs*cnl+(nu+nx)%bs+nu*bs);
				}

			}


#if 0
	d_print_pmat(nz, nx+ng, bs, pLBAbtDCt, cnxg);
	d_print_pmat(nz, nz, bs, hpL[N-nn-1], cnl);
	d_print_mat(1, ng, Qx[N]+2*pnb, 1);
	d_print_mat(1, ng, Qx[N]+2*pnb+png, 1);
	d_print_mat(1, ng, qx[N]+2*pnb, 1);
	d_print_mat(1, ng, qx[N]+2*pnb+png, 1);
	d_print_mat(1, ng, Qx[N-1]+2*pnb, 1);
	d_print_mat(1, ng, Qx[N-1]+2*pnb+png, 1);
	d_print_mat(1, ng, qx[N-1]+2*pnb, 1);
	d_print_mat(1, ng, qx[N-1]+2*pnb+png, 1);
	exit(1);
#endif


		}

	if(fixed_x0==1) // mpc
		{

		// first stage 
		if(diag_min!=0.0)
			{

			dtrmm_nt_u_lib(nz, nx, hpBAbt[0], cnx, hpL[1]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);
			for(jj=0; jj<nx; jj++) work[jj] = pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs]; // backup in work !!!
			if(compute_Pb==1)
				{
				dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, work, hPb[0], 0); // L*(L'*b)
				}
			for(jj=0; jj<nx; jj++) pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs] += hpL[1][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];

			if(ng>0)
				{
				dgemv_n_lib(nx+nu, ng, hpDCt[0], cng, qx[0]+2*pnb, hQl[0], hQl[0], 1);
				dgemm_diag_right_lib(nx+nu, ng, hpDCt[0], cng, Qx[0]+2*pnb, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg, 0);
				for(jj=0; jj<ng; jj++)
					pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
				}
			if(update_hessian)
				{
				ddiain_lib(nu, hQd[0], 0, hpQ[0], cnz);
				drowin_lib(nu, hQl[0], hpQ[0]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
				}

			dsyrk_dpotrf_lib(nz, nu, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[0], cnz, hpL[0], cnl, hdL[0]);

			}
		else
			{

			dtrmm_nt_u_lib(nu+nx, nx, hpBAbt[0], cnx, hpL[1]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);

			for(jj=0; jj<nx; jj++) work[jj] = hpBAbt[0][((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs]; // copy b in aligned memory
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, work, work+anz, 0);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, work+anz, hPb[N-nn-1], 0); // L*(L'*b)

			if(ng>0)
				{
				dgemv_n_lib(nx+nu, ng, hpDCt[0], cng, qx[0]+2*pnb, hQl[0], hQl[0], 1);
				dgemm_diag_right_lib(nx+nu, ng, hpDCt[0], cng, Qx[0]+2*pnb, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg, 0);
				for(jj=0; jj<ng; jj++)
					pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
				}
			if(update_hessian)
				{
				ddiain_lib(nu, hQd[0], 0, hpQ[0], cnz);
				drowin_lib(nu, hQl[0], hpQ[0]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
				}

			dsyrk_nt_lib(nu+nx, nu, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, hpQ[0], cnz, hpL[0], cnl, 1);

			for(jj=0; jj<nx; jj++) work[jj] = hPb[0][jj] + hpL[1][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];
			drowex_lib(nu, hpQ[0]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs, work+pnz);
			dgemv_n_lib(nx, nx, hpBAbt[0], cnx, work, work+pnz, work+pnz, 1);
			drowin_lib(nu, work+pnz, hpL[0]+(nu+nx)/bs*bs*cnl+(nu+nx)%bs);

			dpotrf_lib(nz, nu, hpL[0], cnl, hpL[0], cnl, hdL[0]);

			}


		// forward substitution 
		// first stage
		nn = 0;
		for(jj=0; jj<nu; jj++) hux[0][jj] = - hpL[0][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*jj];
		dtrsv_t_lib(nx+nu, nu, hpL[0], cnl, 1, hdL[0], &hux[0][0], &hux[0][0]);
		for(jj=0; jj<nx; jj++) hux[1][nu+jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[0], cnx, &hux[0][0], &hux[1][nu], &hux[1][nu], 1);
		if(compute_pi)
			{
			diag_min = d_min_mat(nx, 1, hdL[1], 1);
			if(diag_min!=0)
				{
				for(jj=0; jj<nx; jj++) hpi[1][jj] = hux[1][nu+jj]; // copy x into aligned memory
				for(jj=0; jj<nx; jj++) work[jj] = hpL[1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
				dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, &hpi[1][0], &work[0], 1);
				dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, &work[0], &hpi[1][0], 0); // L*(L'*b) + p
				}
			else
				{
				for(jj=0; jj<nx; jj++) hpi[1][jj] = hux[1][nu+jj]; // copy x into aligned memory
				dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, &hpi[1][0], &work[0], 0);
				dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, &work[0], &hpi[1][0], 0); // L*(L'*b) + p
				for(jj=0; jj<nx; jj++) hpi[1][jj] += hpL[1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
				}
			}

		}
	else // mhe
		{

		// TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		// first stage 
		dtrmm_nt_u_lib(nz, nx, hpBAbt[0], cnx, hpL[1]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);
		if(compute_Pb)
			{
			for(jj=0; jj<nx; jj++) work[jj] = pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs];
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, work, hPb[0], 0); // L*(L'*b)
			}
		for(jj=0; jj<nx; jj++) pLBAbtDCt[((nx+nu)/bs)*bs*cnxg+(nx+nu)%bs+(jj)*bs] += hpL[1][((nx+nu)/bs)*bs*cnl+(nx+nu)%bs+(nu+jj)*bs];
		if(ng>0)
			{
			dgemv_n_lib(nx+nu, ng, hpDCt[0], cng, qx[0]+2*pnb, hQl[0], hQl[0], 1);
			dgemm_diag_right_lib(nx+nu, ng, hpDCt[0], cng, Qx[0]+2*pnb, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg, 0);
			for(jj=0; jj<ng; jj++)
				pLBAbtDCt[(nu+nx)/bs*cnxg*bs+(nu+nx)%bs+(nx+jj)*bs] = 0.0;
			}
		if(update_hessian)
			{
			ddiain_lib(nx+nu, hQd[0], 0, hpQ[0], cnz);
			drowin_lib(nx+nu, hQl[0], hpQ[0]+((nx+nu)/bs)*bs*cnz+(nx+nu)%bs);
			}
		dsyrk_dpotrf_lib(nz, nu+nx, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[0], cnz, hpL[0], cnl, hdL[0]);

		dtrtr_l_lib(nx, nu, hpL[0]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[0]+(ncl)*bs, cnl);	



		// forward substitution 
		// first stage
		nn = 0;
		drowex_lib(nu+nx, hpL[0]+(nx+nu)/bs*bs*cnl+(nu+nx)%bs, hux[0]);
		d_scale_mat(nu+nx, 1, -1.0, hux[0], 1);
		dtrsv_t_lib(nx+nu, nu+nx, hpL[0], cnl, 1, hdL[0], &hux[0][0], &hux[0][0]);
		for(jj=0; jj<nx; jj++) hux[1][nu+jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[0], cnx, &hux[0][0], &hux[1][nu], &hux[1][nu], 1);
		if(compute_pi)
			{
			for(jj=0; jj<nx; jj++) hpi[1][jj] = hux[1][nu+jj]; // copy x into aligned memory
			for(jj=0; jj<nx; jj++) work[jj] = hpL[1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, &hpi[1][0], &work[0], 1);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, &work[0], &hpi[1][0], 0); // L*(L'*b) + p
			}

		}



	// final stages
	for(nn=1; nn<N; nn++)
		{
		for(jj=0; jj<nu; jj++) hux[nn][jj] = - hpL[nn][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*jj];
		dtrsv_t_lib(nx+nu, nu, hpL[nn], cnl, 1, hdL[nn], &hux[nn][0], &hux[nn][0]);
		for(jj=0; jj<nx; jj++) hux[nn+1][nu+jj] = hpBAbt[nn][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[nn], cnx, &hux[nn][0], &hux[nn+1][nu], &hux[nn+1][nu], 1);
		if(compute_pi)
			{
			diag_min = d_min_mat(nx, 1, hdL[nn+1], 1);
			if(diag_min!=0)
				{
				for(jj=0; jj<nx; jj++) hpi[nn+1][jj] = hux[nn+1][nu+jj]; // copy x into aligned memory
				for(jj=0; jj<nx; jj++) work[jj] = hpL[nn+1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
				dtrmv_u_n_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &hpi[nn+1][0], &work[0], 1);
				dtrmv_u_t_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &work[0], &hpi[nn+1][0], 0); // L*(L'*b) + p
				}
			else
				{
				for(jj=0; jj<nx; jj++) hpi[nn+1][jj] = hux[nn+1][nu+jj]; // copy x into aligned memory
				dtrmv_u_n_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &hpi[nn+1][0], &work[0], 0);
				dtrmv_u_t_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &work[0], &hpi[nn+1][0], 0); // L*(L'*b) + p
				for(jj=0; jj<nx; jj++) hpi[nn+1][jj] += hpL[nn+1][((nu+nx)/bs)*bs*cnl+(nu+nx)%bs+bs*(nu+jj)]; // work space
				}
			}
		}
	
#if 0
	d_print_pmat(nz, nz, bs, hpL[0], cnl);
	d_print_pmat(nz, nz, bs, hpL[1], cnl);
	d_print_pmat(nz, nz, bs, hpL[2], cnl);
	d_print_pmat(nz, nz, bs, hpL[3], cnl);
	d_print_pmat(nz, nz, bs, hpL[N-1], cnl);
	d_print_pmat(nz, nz, bs, hpL[N], cnl);
	//exit(1);
#endif

	}

#endif
	


// pQ is of size pnux x cnux !!!!! XXX
// pL is of size pnz x cnl
void d_back_ric_trf_new(int N, int nx, int nu, double **hpBAbt, double **hpQ, int update_hessian, double **hQd, int fixed_x0, double **hpL, double **hdL, double *pLBAbtDCt, double *work, int nb, int ng, int ngN, double **hpDCt, double **Qx)
	{
	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	
	const int nz   = nx+nu+1; // keep to compatibility with trs routine !!!!
	const int nux  = nu+nx;
//	const int pnz  = bs*((nz+bs-1)/bs);
	const int pnux = bs*((nux+bs-1)/bs);
	const int pnx  = bs*((nx+bs-1)/bs);
	const int pnb  = bs*((nb+bs-1)/bs);
	const int png  = bs*((ng+bs-1)/bs);
	const int pngN = bs*((ngN+bs-1)/bs);
	const int cnz  = ncl*((nz+ncl-1)/ncl);
	const int cnx  = ncl*((nx+ncl-1)/ncl);
	const int cnux = ncl*((nux+ncl-1)/ncl);
	const int cng  = ncl*((ng+ncl-1)/ncl);
	const int cngN = ncl*((ngN+ncl-1)/ncl);
	const int cnxg = ncl*((ng+nx+ncl-1)/ncl);

	const int cnl = cnz<cnx+ncl ? cnx+ncl : cnz; // keep to compatibility with trs routine !!!!

	int nu0 = (nu/bs)*bs;

	int ii, jj, ll, nn;

	double temp;

	double diag_min = 1.0;

	// factorization and backward substitution 

//dgeset_lib(nz, nz, 0.0, 0, hpL[N], cnl);

	// final stage 
	if(ngN>0)
		{
		dgemm_diag_right_lib(nux, ngN, hpDCt[N], cngN, Qx[N]+2*pnb, 0, pLBAbtDCt, cngN, pLBAbtDCt, cngN);
		}
	if(update_hessian)
		{
		ddiain_lib(nu%bs+nx, hQd[N]+nu0, 0, hpQ[N]+nu0*cnux+nu0*bs, cnux);
		}

	dsyrk_dpotrf_lib(nx+nu%bs, nx+nu%bs, ngN, pLBAbtDCt+(nu/bs)*bs*cngN, cngN, pLBAbtDCt+(nu/bs)*bs*cngN, cngN, 1, hpQ[N]+(nu/bs)*bs*cnux+(nu/bs)*bs*bs, cnux, hpL[N]+(nu/bs)*bs*cnl+(nu/bs)*bs*bs, cnl, hdL[N]+nu);

	dtrtr_l_lib(nx, nu, hpL[N]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[N]+(ncl)*bs, cnl);	


	// middle stages 
	for(nn=0; nn<N-1; nn++)
		{	

		dtrmm_nt_u_lib(nux, nx, hpBAbt[N-nn-1], cnx, hpL[N-nn]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);

		if(ng>0)
			{
			dgemm_diag_right_lib(nux, ng, hpDCt[N-nn-1], cng, Qx[N-nn-1]+2*pnb, 0, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg);
			}
		if(update_hessian)
			{
			ddiain_lib(nux, hQd[N-nn-1], 0, hpQ[N-nn-1], cnux);
			}

		dsyrk_dpotrf_lib(nux, nux, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[N-nn-1], cnux, hpL[N-nn-1], cnl, hdL[N-nn-1]);

		dtrtr_l_lib(nx, nu, hpL[N-nn-1]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[N-nn-1]+(ncl)*bs, cnl);	

		}


	if(fixed_x0==1) // mpc
		{

		// first stage 
		dtrmm_nt_u_lib(nux, nx, hpBAbt[0], cnx, hpL[1]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);

		if(ng>0)
			{
			dgemm_diag_right_lib(nux, ng, hpDCt[0], cng, Qx[0]+2*pnb, 0, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg);
			}
		if(update_hessian)
			{
			ddiain_lib(nu, hQd[0], 0, hpQ[0], cnux);
			}

		dsyrk_dpotrf_lib(nux, ((nu+2-1)/2)*2, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[0], cnux, hpL[0], cnl, hdL[0]);

		}
	else // mhe
		{

		// first stage 
		dtrmm_nt_u_lib(nux, nx, hpBAbt[0], cnx, hpL[1]+(ncl)*bs, cnl, pLBAbtDCt, cnxg);

		if(ng>0)
			{
			dgemm_diag_right_lib(nx+nu, ng, hpDCt[0], cng, Qx[0]+2*pnb, 0, pLBAbtDCt+nx*bs, cnxg, pLBAbtDCt+nx*bs, cnxg);
			}
		if(update_hessian)
			{
			ddiain_lib(nx+nu, hQd[0], 0, hpQ[0], cnux);
			}

		dsyrk_dpotrf_lib(nux, nux, nx+ng, pLBAbtDCt, cnxg, pLBAbtDCt, cnxg, 1, hpQ[0], cnux, hpL[0], cnl, hdL[0]);

		dtrtr_l_lib(nx, nu, hpL[0]+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, hpL[0]+(ncl)*bs, cnl);	

		}

	}



void d_back_ric_trs_new(int N, int nx, int nu, double **hpBAbt, double **hq, int fixed_x0, double **hux, double **hpL, double **hdL, double *work, int compute_Pb, double **hPb, int compute_pi, double **hpi, int nb, int ng, int ngN, double **hpDCt, double **qx)
	{
	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	
	const int nz   = nx+nu+1;
	const int pnz  = bs*((nz+bs-1)/bs);
	const int pnx  = bs*((nx+bs-1)/bs);
	const int pnb  = bs*((nb+bs-1)/bs);
	const int png  = bs*((ng+bs-1)/bs);
	const int pngN = bs*((ngN+bs-1)/bs);
	const int cnz  = ncl*((nz+ncl-1)/ncl);
	const int cnx  = ncl*((nx+ncl-1)/ncl);
	const int cng  = ncl*((ng+ncl-1)/ncl);
	const int cngN = ncl*((ngN+ncl-1)/ncl);
	const int cnxg = ncl*((ng+nx+ncl-1)/ncl);

	const int cnl = cnz<cnx+ncl ? cnx+ncl : cnz;

	int nu0 = (nu/bs)*bs;

	int ii, jj, ll, nn;

	double temp;

	double diag_min = 1.0;

	// backward substitution 
	// last stages
	if(ngN>0)
		{
		dgemv_n_lib(nx+nu, ngN, hpDCt[N], cngN, qx[N]+2*pnb, 1, hq[N], hq[N]);
		}
	for(nn=0; nn<N-1; nn++)
		{
		if(compute_Pb)
			{
			for(jj=0; jj<nx; jj++) hPb[N-nn-1][jj] = hpBAbt[N-nn-1][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj]; // copy b
			dtrmv_u_n_lib(nx, hpL[N-nn]+(ncl)*bs, cnl, hPb[N-nn-1], 0, work);
			dtrmv_u_t_lib(nx, hpL[N-nn]+(ncl)*bs, cnl, work, 0, hPb[N-nn-1]); // L*(L'*b)
			}
		// general constraints
		if(ng>0)
			{
			dgemv_n_lib(nx+nu, ng, hpDCt[N-nn-1], cng, qx[N-nn-1]+2*pnb, 1, hq[N-nn-1], hq[N-nn-1]);
			}
		for(jj=0; jj<nx; jj++) work[jj] = hPb[N-nn-1][jj] + hq[N-nn][nu+jj]; // add p
		dgemv_n_lib(nx+nu, nx, hpBAbt[N-nn-1], cnx, work, 1, hq[N-nn-1], hq[N-nn-1]);
		dtrsv_n_lib(nu+nx, nu, hpL[N-nn-1], cnl, 1, hdL[N-nn-1], hq[N-nn-1], hq[N-nn-1]);
		}




	if(fixed_x0==1) // mpc
		{

		// first stage 
		if(compute_Pb)
			{
			for(jj=0; jj<nx; jj++) hPb[0][jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj]; // copy b
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, hPb[0], 0, work);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, work, 0, hPb[0]); // L*(L'*b)
			}
		// general constraints
		if(ng>0)
			{
			dgemv_n_lib(nx+nu, ng, hpDCt[0], cng, qx[0]+2*pnb, 1, hq[0], hq[0]);
			}
		for(jj=0; jj<nx; jj++) work[jj] = hPb[0][jj] + hq[1][nu+jj]; // add p
		dgemv_n_lib(nx+nu, nx, hpBAbt[0], cnx, work, 1, hq[0], hq[0]);
		dtrsv_n_lib(nu+nx, nu, hpL[0], cnl, 1, hdL[0], hq[0], hq[0]);


		// forward substitution 
		// first stage
		nn = 0;
		for(jj=0; jj<nu; jj++) hux[0][jj] = - hq[0][jj];
		dtrsv_t_lib(nx+nu, nu, hpL[0], cnl, 1, hdL[0], &hux[0][0], &hux[0][0]);
		for(jj=0; jj<nx; jj++) hux[1][nu+jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[0], cnx, &hux[0][0], 1, &hux[1][nu], &hux[1][nu]);
		if(compute_pi)
			{
			for(jj=0; jj<nx; jj++) hpi[1][jj] = hux[1][nu+jj]; // copy x into aligned memory
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, &hpi[1][0], 0, &work[0]);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, &work[0], 0, &hpi[1][0]); // L*(L'*b) + p
			for(jj=0; jj<nx; jj++) hpi[1][jj] += hq[1][nu+jj];
			}

		}
	else // mhe
		{

		// first stage 
		if(compute_Pb)
			{
			for(jj=0; jj<nx; jj++) hPb[0][jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj]; // copy b
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, hPb[0], 0, work);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, work, 0, hPb[0]); // L*(L'*b)
			}
		// general constraints
		if(ng>0)
			{
			dgemv_n_lib(nx+nu, ng, hpDCt[0], cng, qx[0]+2*pnb, 1, hq[0], hq[0]);
			}
		for(jj=0; jj<nx; jj++) work[jj] = hPb[0][jj] + hq[1][nu+jj]; // add p
		dgemv_n_lib(nx+nu, nx, hpBAbt[0], cnx, work, 1, hq[0], hq[0]);
		dtrsv_n_lib(nu+nx, nu+nx, hpL[0], cnl, 1, hdL[0], hq[0], hq[0]);




		// forward substitution 
		// first stage
		nn = 0;
		for(jj=0; jj<nu+nx; jj++) hux[0][jj] = - hq[0][jj];
		dtrsv_t_lib(nx+nu, nu+nx, hpL[0], cnl, 1, hdL[0], &hux[0][0], &hux[0][0]);
		for(jj=0; jj<nx; jj++) hux[1][nu+jj] = hpBAbt[0][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[0], cnx, &hux[0][0], 1, &hux[1][nu], &hux[1][nu]);
		if(compute_pi)
			{
			for(jj=0; jj<nx; jj++) hpi[1][jj] = hux[1][nu+jj]; // copy x into aligned memory
			dtrmv_u_n_lib(nx, hpL[1]+(ncl)*bs, cnl, &hpi[1][0], 0, &work[0]);
			dtrmv_u_t_lib(nx, hpL[1]+(ncl)*bs, cnl, &work[0], 0, &hpi[1][0]); // L*(L'*b) + p
			for(jj=0; jj<nx; jj++) hpi[1][jj] += hq[1][nu];
			}

		}



	// final stages
	for(nn=1; nn<N; nn++)
		{
		for(jj=0; jj<nu; jj++) hux[nn][jj] = - hq[nn][jj];
		dtrsv_t_lib(nx+nu, nu, hpL[nn], cnl, 1, hdL[nn], &hux[nn][0], &hux[nn][0]);
		for(jj=0; jj<nx; jj++) hux[nn+1][nu+jj] = hpBAbt[nn][((nu+nx)/bs)*bs*cnx+(nu+nx)%bs+bs*jj];
		dgemv_t_lib(nx+nu, nx, hpBAbt[nn], cnx, &hux[nn][0], 1, &hux[nn+1][nu], &hux[nn+1][nu]);
		if(compute_pi)
			{
			for(jj=0; jj<nx; jj++) hpi[nn+1][jj] = hux[nn+1][nu+jj]; // copy x into aligned memory
			dtrmv_u_n_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &hpi[nn+1][0], 0, &work[0]);
			dtrmv_u_t_lib(nx, hpL[nn+1]+(ncl)*bs, cnl, &work[0], 0, &hpi[nn+1][0]); // L*(L'*b) + p
			for(jj=0; jj<nx; jj++) hpi[nn+1][jj] += hq[nn+1][nu+jj];
			}
		}
	
	}



// version exploiting the fact that A is the diagonal of a matrix
// L = chol(R + B'*P*B)
// K = (diag(A)'*P*B)\L
// P = Q + diag(A')*P*diag(A) + K*K'
void d_ric_diag_trf_mpc(int N, int *nx, int *nu, double **hdA, double **hpBt, double **hpR, double **hpSt, double **hpQ, double **hpL, double *pK, double **hpP, double *work) //, int update_hessian, double **hpd)
	{

	// K 0f size (pnu[ii+1]) x (cnu[ii])
	// L of size (pnu[ii]+pnx[ii]) x (cnu[ii])

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;

	int nu0, nx0, nx1, nxm, pnu0, cnu0, cnx0, cnx1;
	
	int ii, jj, ll, nn;

	// last stage: inintialize P with Q_N
	nu0 = nu[N];
	nx0 = nx[N];
	cnx0  = ncl*((nx[N]+ncl-1)/ncl);

	d_copy_pmat(nx0, nx0, bs, hpQ[N], cnx0, hpP[N], cnx0);
	//if(update_hessian)
	//	d_update_diag_pmat(nx0, hpP[N], cnx0, hpd[N]+nu0);
	//d_print_pmat(nx0, nx0, bs, hpP[N], cnx0);

	// factorization and backward substitution 
	for(nn=0; nn<N; nn++)
		{

		nu0 = nu[N-nn-1];
		nx1 = nx0;
		nx0 = nx[N-nn-1];
		pnu0  = bs*((nu0+bs-1)/bs);
		cnu0  = ncl*((nu0+ncl-1)/ncl);
		cnx1 = cnx0;
		cnx0  = ncl*((nx0+ncl-1)/ncl);
		nxm = (nx0<nx1) ? nx0 : nx1;

		// PB = P*(B')'
		//d_print_pmat(nu0, nx1, bs, hpBt[N-nn-1], cnx1);
		dgemm_nt_lib(nx1, nu0, nx1, hpP[N-nn], cnx1, hpBt[N-nn-1], cnx1, 0, pK, cnu0, pK, cnu0, 0, 0);
		//d_print_pmat(nx1, nu0, bs, pK, cnu0);

		//if(update_hessian)
		//	d_update_diag_pmat(nu0, hpR[N-nn-1], cnu0, hpd[N-nn-1]);

		//d_print_pmat(nu0, nu0, bs, hpR[N-nn-1], cnu0);
		dsyrk_nn_lib(nu0, nu0, nx1, hpBt[N-nn-1], cnx1, pK, cnu0, 1, hpR[N-nn-1], cnu0, hpL[N-nn-1], cnu0);
		//d_print_pmat(nu0, nu0, bs, hpL[N-nn-1], cnu0);

		d_copy_pmat(nx0, nu0, bs, hpSt[N-nn-1], cnu0, hpL[N-nn-1]+pnu0*cnu0, cnu0);
		//d_print_pmat(nx0+nu0, nu0, bs, hpL[N-nn-1], cnu0);

		dgemm_diag_left_lib(nxm, nu0, hdA[N-nn-1], pK, cnu0, 1, hpL[N-nn-1]+pnu0*cnu0, cnu0, hpL[N-nn-1]+pnu0*cnu0, cnu0);
		//d_print_pmat(pnu0+nx0, nu0, bs, hpL[N-nn-1], cnu0);

		//d_copy_pmat_panel(pnu0-nu0, nu0, 
		if(nx0>pnu0-nu0)
			d_align_pmat_panel(pnu0-nu0, nu0, nu0+nx0, hpL[N-nn-1]+(nu0+nx0)/bs*bs*cnu0+(nu0+nx0)%bs, cnu0, hpL[N-nn-1]+nu0/bs*bs*cnu0+nu0%bs);
		else
			d_align_pmat_panel(nx0, nu0, pnu0, hpL[N-nn-1]+pnu0*cnu0, cnu0, hpL[N-nn-1]+nu0/bs*bs*cnu0+nu0%bs);
		//d_print_pmat(pnu0+nx0, nu0, bs, hpL[N-nn-1], cnu0);

		dpotrf_lib_old(nu0+nx0, nu0, hpL[N-nn-1], cnu0, hpL[N-nn-1], cnu0, work);
		//d_print_pmat(pnu0+nx0, nu0, bs, hpL[N-nn-1], cnu0);
		for(jj=0; jj<nu0; jj++) hpL[N-nn-1][(jj/bs)*bs*cnu0+jj%bs+jj*bs] = work[jj]; // copy reciprocal of diagonal
		//d_print_pmat(pnu0+nx0, nu0, bs, hpL[N-nn-1], cnu0);

		if(nx0>pnu0-nu0)
			d_copy_pmat_panel(pnu0-nu0, nu0, nu0+nx0, hpL[N-nn-1]+nu0/bs*bs*cnu0+nu0%bs, hpL[N-nn-1]+(nu0+nx0)/bs*bs*cnu0+(nu0+nx0)%bs, cnu0);
		else
			d_copy_pmat_panel(nx0, nu0, pnu0, hpL[N-nn-1]+nu0/bs*bs*cnu0+nu0%bs, hpL[N-nn-1]+pnu0*cnu0, cnu0);
		//d_print_pmat(pnu0+nx0, nu0, bs, hpL[N-nn-1], cnu0);
		//for(jj=0; jj<nu0; jj++) for(ii=nu0; ii<pnu0; ii++) hpL[N-nn-1][nu0/bs*bs*cnu0+nu0%bs+jj*bs] = 0.0;

		d_copy_pmat(nx0, nx0, bs, hpQ[N-nn-1], cnx0, hpP[N-nn-1], cnx0);

		//if(update_hessian)
		//	d_update_diag_pmat(nx0, hpP[N-nn-1], cnx0, hpd[N-nn-1]+nu0);
		//d_print_pmat(nx0, nx0, bs, hpP[N-nn-1], cnx0);

		dsyrk_diag_left_right_lib(nxm, hdA[N-nn-1], hdA[N-nn-1], hpP[N-nn], cnx1, 1, hpP[N-nn-1], cnx0, hpP[N-nn-1], cnx0);
		//d_print_mat(1, nxm, hdA[N-nn-1], 1);
		//d_print_pmat(nx0, nx0, bs, hpP[N-nn-1], cnx0);

		dsyrk_nt_lib(nx0, nx0, nu0, hpL[N-nn-1]+pnu0*cnu0, cnu0, hpL[N-nn-1]+pnu0*cnu0, cnu0, -1, hpP[N-nn-1], cnx0, hpP[N-nn-1], cnx0);
		//d_print_pmat(nx0, nx0, bs, hpP[N-nn-1], cnx0);

		dtrtr_l_lib(nx0, 0, hpP[N-nn-1], cnx0, 0, hpP[N-nn-1], cnx0);	
		//d_print_pmat(nx0, nx0, bs, hpP[N-nn-1], cnx0);

		//exit(1);
		}
		
	}



// version exploiting the fact that A is the diagonal of a matrix
void d_ric_diag_trs_mpc(int N, int *nx, int *nu, double **hdA, double **hpBt, double **hpL, double **hpP, double **hb, double **hrq, double **hux, int compute_Pb, double **hPb, int compute_pi, double **hpi, double *work)
	{

	const int bs  = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line

	int nu0, nu1, nx0, nx1, nxm, pnu0, cnu0, cnx0, cnx1;

	int ii, jj;
	
	static double x0_bkp[D_MR];
	int cp_max = (nu[0]+bs-1)/bs*bs - nu0;
	cp_max = nx[0]<cp_max ? nx[0] : cp_max;
	for(jj=0; jj<cp_max; jj++)
		x0_bkp[jj] =  hux[0][nu[0]+jj];

	/* backward substitution */

	nu0 = nu[N];
	nx0 = nx[N];
	cnx0  = ncl*((nx[N]+ncl-1)/ncl);

	for(jj=0; jj<nx0; jj++) hpi[N][jj] = hrq[N][nu0+jj];

	for(ii=0; ii<N; ii++)
		{

		nu0 = nu[N-ii-1];
		nx1 = nx0;
		nx0 = nx[N-ii-1];
		pnu0  = bs*((nu0+bs-1)/bs);
		cnu0  = ncl*((nu0+ncl-1)/ncl);
		cnx1 = cnx0;
		cnx0  = ncl*((nx0+ncl-1)/ncl);
		nxm = (nx0<nx1) ? nx0 : nx1;

		if(compute_Pb)
			{
			dgemv_n_lib(nx1, nx1, hpP[N-ii], cnx1, hb[N-ii-1], 0, hPb[N-ii-1], hPb[N-ii-1]);
			}
		for(jj=0; jj<nx1; jj++) work[jj] = hPb[N-ii-1][jj] + hpi[N-ii][jj];
		for(jj=0; jj<nu0; jj++) hux[N-ii-1][jj] = hrq[N-ii-1][jj];
		dgemv_n_lib(nu0, nx1, hpBt[N-ii-1], cnx1, work, 1, hux[N-ii-1], hux[N-ii-1]);
		for(jj=0; jj<nxm; jj++) hpi[N-ii-1][jj] = hrq[N-ii-1][nu0+jj] + hdA[N-ii-1][jj] * work[jj];
		for(; jj<nx0; jj++) hpi[N-ii-1][jj] = hrq[N-ii-1][nu0+jj];
		dtrsv_n_lib_old(nu0, nu0, 1, hpL[N-ii-1], cnu0, hux[N-ii-1]);
		dgemv_n_lib(nx0, nu0, hpL[N-ii-1]+pnu0*cnu0, cnu0, hux[N-ii-1], -1, hpi[N-ii-1], hpi[N-ii-1]);
		}


	/* forward substitution */

	for(jj=0; jj<cp_max; jj++)
		hux[0][nu[0]+jj] = x0_bkp[jj];

	nu0 = nu[0];
	nu1 = nu[1];
	nx0 = nx[0];
	nx1 = nx[1];
	cnx1  = ncl*((nx[1]+ncl-1)/ncl);
	for(jj=0; jj<nx0; jj++) work[jj] = hux[0][nu0+jj];

	for(ii=0; ii<N; ii++)
		{

		nu0 = nu1;
		nu1 = nu[ii+1];
		nx0 = nx1;
		nx1 = nx[ii+1];
		pnu0  = bs*((nu0+bs-1)/bs);
		cnu0  = ncl*((nu0+ncl-1)/ncl);
		cnx0 = cnx1;
		cnx1  = ncl*((nx1+ncl-1)/ncl);
		nxm = (nx0<nx1) ? nx0 : nx1;

		for(jj=0; jj<nu0; jj++) hux[ii][jj] = - hux[ii][jj];
		dgemv_t_lib(nx0, nu0, hpL[ii]+pnu0*cnu0, cnu0, work, -1, hux[ii], hux[ii]); // no overwrite of x
		dtrsv_t_lib_old(nu0, nu0, 1, hpL[ii], cnu0, hux[ii]); // no overwrite of x
		for(jj=0; jj<nx1; jj++) hux[ii+1][nu1+jj] = hb[ii][jj];
		dgemv_t_lib(nu0, nx1, hpBt[ii], cnx1, hux[ii], 1, hux[ii+1]+nu1, hux[ii+1]+nu1);
		for(jj=0; jj<nxm; jj++) 
			{
			hux[ii+1][nu1+jj] += hdA[ii][jj] * work[jj];
			work[jj] = hux[ii+1][nu1+jj];
			}
		for( ; jj<nx1; jj++)
			{
			work[jj] = hux[ii+1][nu1+jj];
			}
		if(compute_pi)
			{
			dgemv_n_lib(nx1, nx1, hpP[ii+1], cnx1, work, 1, hpi[ii+1], hpi[ii+1]); // P*b + p
			}
		}

	}



// version exploiting A=I
// L = chol(R + B'*P*B)
// K = (P*B)\L
// P = Q + P + K*K'
void d_ric_eye_sv_mpc(int nx, int nu, int N, double **hpBt, double **hpR, double **hpSt, double **hpQ, double **hpL, double **hpP, double *work, double *diag)
	{

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
//	const int nal = bs*ncl; // number of doubles per cache line
	
//	const int nz   = nx+nu+1;
//	const int anz  = nal*((nz+nal-1)/nal);
//	const int pnz  = bs*((nz+bs-1)/bs);
	const int pnx  = bs*((nx+bs-1)/bs);
	const int pnu  = bs*((nu+bs-1)/bs);
//	const int pnb  = bs*((nb+bs-1)/bs);
//	const int png  = bs*((ng+bs-1)/bs);
//	const int pngN = bs*((ngN+bs-1)/bs);
//	const int cnz  = ncl*((nz+ncl-1)/ncl);
	const int cnu  = ncl*((nu+ncl-1)/ncl);
	const int cnx  = ncl*((nx+ncl-1)/ncl);
//	const int cng  = ncl*((ng+ncl-1)/ncl);
//	const int cngN = ncl*((ngN+ncl-1)/ncl);
//	const int cnxg = ncl*((ng+nx+ncl-1)/ncl);

//	const int cnl = cnz<cnx+ncl ? cnx+ncl : cnz;

	// number of general constraints TODO
	//const int ng = 0;

	int nu_m = (nu/bs)*bs;
	int nu_r = nu%bs;

	int ii, jj, ll, nn;

	double temp;

	double *pPB, *pPBt, *pK;

	pPB = work;
	work += pnx*cnu;

	pPBt = work;
	work += pnu*cnx;

	pK = work;
	work += pnx*cnu;

	// last stage: inintialize P with Q_N
	d_copy_pmat(nx, nx, bs, hpQ[N], cnx, hpP[N], cnx);

	// factorization and backward substitution 
	for(nn=0; nn<N; nn++)
		{
		// PB = P*(B')'
#if defined(TARGET_C99_4X4) || defined(TARGET_C99_4X4_PREFETCH) || defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		dgemm_nt_lib(nx, nu, nx, hpP[N-nn], cnx, hpBt[N-nn-1], cnx, 0, pPB, cnu, pPBt, cnx, 0, 1); // TODO embed transpose of result in dgemm_nt
#else
		dgemm_nt_lib(nx, nu, nx, hpP[N-nn], cnx, hpBt[N-nn-1], cnx, 0, pPB, cnu, pPB, cnu, 0, 0);
		//d_print_pmat(nx, nx, bs, hpP[N-nn], cnx);
		//d_print_pmat(nu, nx, bs, hpBt[N-nn-1], cnx);
		//d_print_pmat(nx, nu, bs, pPB, cnu);
		//exit(1);

		// PBt = (PB)'
		dgetr_lib(nx, nu, 0, pPB, cnu, 0, pPBt, cnx);
		//d_print_pmat(nu, nx, bs, pPBt, cnx);
		//exit(1);
#endif

		// R + PBt*B'
		//dgemm_nt_lib(nu, nu, nx, pPBt, cnx, hpBt[N-nn-1], cnx, hpL[N-nn-1], cnu, 0);
		dsyrk_nt_lib(nu, nu, nx, pPBt, cnx, hpBt[N-nn-1], cnx, 1, hpR[N-nn-1], cnu, hpL[N-nn-1], cnu);
		//d_print_pmat(nu, nu, bs, hpL[N-nn-1], cnu);
		//exit(1);

		// S + PBt
		for(ii=0; ii<pnu*cnx; ii++) pPBt[ii] = pPBt[ii] + hpSt[N-nn-1][ii]; // TODO routine for this
		//d_print_pmat(nu, nx, bs, pPBt, cnx);
		//exit(1);

		// PB on bottom of L
		dgetr_lib(nu, nx, 0, pPBt, cnx, nu, hpL[N-nn-1]+nu_m*cnu+nu_r, cnu);
		//d_print_pmat(nu+nx, nu, bs, hpL[N-nn-1], cnu);
		//exit(1);
		
		// [L; K] = chol([R + B'*P*B; P*B])
		dpotrf_lib_old(nx+nu, nu, hpL[N-nn-1], cnu, hpL[N-nn-1], cnu, diag);
		//d_print_pmat(nu+nx, nu, bs, hpL[N-nn-1], cnu);
		//exit(1);

		// copy K to alinged memory
		d_align_pmat(nx, nu, nu, bs, hpL[N-nn-1]+nu/bs*bs*cnu, cnu, pK, cnu); // TODO make kernel for this
		//d_print_pmat(nx, nu, bs, pK, cnu);
		//exit(1);

		// P_n = Q_n + P_{n+1}
		for(ii=0; ii<pnx*cnx; ii++) hpP[N-nn-1][ii] = hpP[N-nn][ii] + hpQ[N-nn-1][ii]; // TODO routine for this
		//d_print_pmat(nx, nx, bs, hpP[N-nn-1], cnx);

		// TODO if nu small, low-rank update
		dsyrk_nt_lib(nx, nx, nu, pK, cnu, pK, cnu, -1, hpP[N-nn-1], cnx, hpP[N-nn-1], cnx);
		//d_print_pmat(nx, nx, bs, hpP[N-nn-1], cnx);

		// copy lower triangular to upper triangular
		dtrtr_l_lib(nx, 0, hpP[N-nn-1], cnx, 0, hpP[N-nn-1], cnx);	
		//d_print_pmat(nx, nx, bs, hpP[N-nn-1], cnx);
		//exit(1);

		}
		
	}



int d_forward_schur_trf(int N, int nx, int nu, int nd, int diag_hessian, double **hpQA, double **hpLA, double **hdLA, double **hpLe, double *work)
	{

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;

	const int nxu  = nx+nu;
	const int nxxu = nx+nx+nu;
	const int pnx  = (nx+bs-1)/bs*bs;
	const int pnxu = (nxu+bs-1)/bs*bs;
	const int cnx  = (nx+ncl-1)/ncl*ncl;
	const int cnxu = (nxu+ncl-1)/ncl*ncl;

	double *hpLe_tmp = work; 
	work += pnx*cnx;
	double *hdLe_tmp = work; 
	work += pnx;



	float diag_min;
	diag_min = 1.0;

	int ii, jj, ll;	


	dgecp_lib(nxxu, nxu, 0, hpQA[0], cnxu, 0, hpLA[0], cnxu);

	for(ii=0; ii<N; ii++)
		{

		// assume that A is aligned to a panel boundary, and that the lower part of A is copied between Q and A
		//dlauum_dpotrf_lib(nxxu, nxu, nx, hpLe[ii], cnx, hpLe[ii], cnx, hpQA[ii], cnxu, 0, hpLA[ii], cnxu, hdLA[ii]);
		//dpotrf_lib(nxxu, nxu, hpQA[ii], cnxu, hpLA[ii], cnxu, hdLA[ii]);
		dpotrf_lib(nxxu, nxu, hpLA[ii], cnxu, hpLA[ii], cnxu, hdLA[ii]);

		for(jj=0; jj<nxu; jj++) 
			diag_min = fmin(diag_min, hdLA[ii][jj]);

		// copy back the lower part of A
		if(nx>pnxu-nxu)
			dgecp_lib(pnxu-nxu, nxu, nxu, hpLA[ii]+nxu/bs*bs*cnxu+nxu%bs, cnxu, 0, hpLA[ii]+nxxu/bs*bs*cnxu+nxxu%bs, cnxu);
		else
			dgecp_lib(nx, nxu, nxu, hpLA[ii]+nxu/bs*bs*cnxu+nxu%bs, cnxu, 0, hpLA[ii]+pnxu*cnxu, cnxu);

#if 1
		dsyrk_dpotrf_lib(nx, nx, nxu, hpLA[ii]+pnxu*cnxu, cnxu, hpLA[ii]+pnxu*cnxu, cnxu, 0, hpLe_tmp, cnx, hpLe_tmp, cnx, hdLe_tmp);
#else
		dsyrk_nt_lib(nx, nx, nxu, hpLA[ii]+pnxu*cnxu, cnxu, hpLA[ii]+pnxu*cnxu, cnxu, 0, hpLe_tmp, cnx, hpLe_tmp, cnx);
		dpotrf_lib(nx, nx, hpLe_tmp, cnx, hpLe_tmp, cnx, hdLe_tmp);
#endif

		for(jj=0; jj<nx; jj++) 
			diag_min = fmin(diag_min, hdLe_tmp[jj]);

		dtrtri_lib(nx, hpLe_tmp, cnx, 1, hdLe_tmp, hpLe[ii+1], cnx);

		dgecp_lib(nxxu, nxu, 0, hpQA[ii+1], cnxu, 0, hpLA[ii+1], cnxu);

		dlauum_lib(nx, hpLe[ii+1], cnx, hpLe[ii+1], cnx, 1, hpLA[ii+1], cnxu, hpLA[ii+1], cnxu);


		if(diag_min==0.0)
			return ii+1;

		}

	// equality constraints at the last stage
	if(nd>0)
		{
		}
	else
		{
		dpotrf_lib(nxxu, nxu, hpLA[N], cnxu, hpLA[N], cnxu, hdLA[N]);

		// copy back the lower part of A
		if(nx>pnxu-nxu)
			dgecp_lib(pnxu-nxu, nxu, nxu, hpLA[N]+nxu/bs*bs*cnxu+nxu%bs, cnxu, 0, hpLA[N]+nxxu/bs*bs*cnxu+nxxu%bs, cnxu);
		else
			dgecp_lib(nx, nxu, nxu, hpLA[N]+nxu/bs*bs*cnxu+nxu%bs, cnxu, 0, hpLA[N]+pnxu*cnxu, cnxu);

		for(jj=0; jj<nxu; jj++) 
			diag_min = fmin(diag_min, hdLA[N][jj]);

		}

	if(diag_min==0.0)
		return ii+1;

	return 0;

	}



int d_forward_schur_trs(int N, int nx, int nu, int nd, int diag_hessian, double **hpLA, double **hdLA, double **hpLe, double *work)
	{


	}



// information filter version
int d_ric_trf_mhe_if(int nx, int nw, int ndN, int N, double **hpQRAG, int diag_R, double **hpLe, double **hpLAG, double *Ld, double *work)
	{

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl;

	const int nwx = nw+nx;
	const int anx = nal*((nx+nal-1)/nal);
	const int pnx = bs*((nx+bs-1)/bs);
	const int pnw = bs*((nw+bs-1)/bs);
	const int pnx2 = bs*((2*nx+bs-1)/bs);
	const int pnwx = bs*((nw+nx+bs-1)/bs);
	const int pndN = bs*((ndN+bs-1)/bs);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int cnw = ncl*((nw+ncl-1)/ncl);
	const int cnwx = ncl*((nw+nx+ncl-1)/ncl);
	const int cnwx1 = ncl*((nw+nx+1+ncl-1)/ncl);
	const int cnx2 = 2*(ncl*((nx+ncl-1)/ncl));
	//const int cnwx = ncl*((nw+nx+ncl-1)/ncl);

	const int pad = (ncl-(nx+nw)%ncl)%ncl; // packing
	const int cnl = nx+nw+pad+cnx;

	float diag_min;
	diag_min = 1.0;

	double *ptr;
	ptr = work;

	double *pLe_temp; pLe_temp = ptr;
	ptr += pnx*cnx;

	double *diag; diag = ptr;
	ptr += anx;

	int ii, jj, ll;	

	double alpha;


#if 1

	// if(pnx>-pnw)
	for(ii=0; ii<N; ii++)
		{

		//d_print_pmat(nx, nx, bs, hpLe[ii], cnx);
		//d_print_pmat(2*pnx, cnwx1, bs, hpQRAG[ii], cnwx1);
		if(1) // merged
			{
			dtsyrk_dpotrf_lib(2*nx, nx, nx, hpLe[ii], cnx, 1, hpQRAG[ii], cnwx1, hpLAG[ii], cnwx1, diag);
			}
		else // un-merged
			{
			d_copy_pmat(nx+nx%bs, nx, bs, hpQRAG[ii]+nx/bs*bs*cnwx1, cnwx1, hpLAG[ii]+nx/bs*bs*cnwx1, cnwx1);
			//d_print_pmat(2*pnx, cnwx1, bs, hpQRAG[ii], cnwx1);
			//d_print_pmat(2*pnx, cnwx1, bs, hpLAG[ii], cnwx1);
			//exit(2);
			//d_print_pmat(nx, nx, bs, hpLe[ii], cnx);
			dsyttmm_ul_lib(nx, hpLe[ii], cnx, 1, hpQRAG[ii], cnwx1, hpLAG[ii], cnwx1);
			//d_print_pmat(2*pnx, cnwx1, bs, hpLAG[ii], cnwx1);
			dpotrf_lib_old(2*nx, nx, hpLAG[ii], cnwx1, hpLAG[ii], cnwx1, diag);
			}
		//d_print_pmat(2*pnx, cnwx1, bs, hpLAG[ii], cnwx1);
		//if(ii==1)
		//exit(1);
		if(nx>pnx-nx)
			d_copy_pmat_panel(pnx-nx, nx, 2*nx, hpLAG[ii]+nx/bs*bs*cnwx1+nx%bs, hpLAG[ii]+(2*nx)/bs*bs*cnwx1+(2*nx)%bs, cnwx1);
		else
			d_copy_pmat_panel(nx, nx, pnx, hpLAG[ii]+nx/bs*bs*cnwx1+nx%bs, hpLAG[ii]+pnx*cnwx1, cnwx1);
		//d_print_pmat(2*pnx, cnwx, bs, hpLAG[ii], cnwx1);

		// copy reciprocal of diagonal
		for(jj=0; jj<nx-3; jj+=4) 
			{
			diag_min = fmin(diag_min, diag[jj+0]);
			hpLAG[ii][jj*cnwx1+0+(jj+0)*bs] = diag[jj+0]; 
			diag_min = fmin(diag_min, diag[jj+1]);
			hpLAG[ii][jj*cnwx1+1+(jj+1)*bs] = diag[jj+1]; 
			diag_min = fmin(diag_min, diag[jj+2]);
			hpLAG[ii][jj*cnwx1+2+(jj+2)*bs] = diag[jj+2]; 
			diag_min = fmin(diag_min, diag[jj+3]);
			hpLAG[ii][jj*cnwx1+3+(jj+3)*bs] = diag[jj+3]; 
			}
		for(; jj<nx; jj++) 
			{
			diag_min = fmin(diag_min, diag[jj]);
			hpLAG[ii][jj/bs*bs*cnwx1+jj%bs+jj*bs] = diag[jj]; 
			}
		//d_print_pmat(2*pnx, cnwx, bs, hpLAG[ii], cnwx1);

		if(diag_R)
			{
			dpotrf_diag_lib(nwx, nw, hpQRAG[ii]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, hpLAG[ii]+(pnx-pnw)*cnwx1+nx*bs, cnwx1);
			//d_print_pmat(2*pnx, cnwx, bs, hpLAG[ii], cnwx1);
			}
		else
			{
			dpotrf_lib_old(nwx, nw, hpQRAG[ii]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, hpLAG[ii]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, diag);
			//d_print_pmat(2*pnx, cnwx, bs, hpLAG[ii], cnwx1);

			// copy reciprocal of diagonal
			for(jj=0; jj<nw-3; jj+=4) 
				{
				hpLAG[ii][(jj+pnx-pnw)*cnwx1+0+(nx+jj+0)*bs] = diag[jj+0];
				hpLAG[ii][(jj+pnx-pnw)*cnwx1+1+(nx+jj+1)*bs] = diag[jj+1];
				hpLAG[ii][(jj+pnx-pnw)*cnwx1+2+(nx+jj+2)*bs] = diag[jj+2];
				hpLAG[ii][(jj+pnx-pnw)*cnwx1+3+(nx+jj+3)*bs] = diag[jj+3];
				}
			for(; jj<nw; jj++) 
				{
				hpLAG[ii][(jj/bs*bs+pnx-pnw)*cnwx1+jj%bs+(nx+jj)*bs] = diag[jj];
				}
			//d_print_pmat(2*pnx, cnwx, bs, hpLAG[ii], cnwx1);
			}
		if(nx>pnw-nw)
			d_copy_pmat_panel(pnw-nw, nw, nx+nw, hpLAG[ii]+(pnx-pnw+nw)/bs*bs*cnwx1+nw%bs+nx*bs, hpLAG[ii]+(pnx-pnw+nw+nx)/bs*bs*cnwx1+(pnx-pnw+nw+nx)%bs+nx*bs, cnwx1);
		else
			d_copy_pmat_panel(nx, nw, pnx, hpLAG[ii]+(pnx-pnw+nw)/bs*bs*cnwx1+nw%bs+nx*bs, hpLAG[ii]+pnx*cnwx1+nx*bs, cnwx1);
		//d_print_pmat(2*pnx, cnwx1, bs, hpLAG[ii], cnwx1);

		if(1) // merged
			{
			dsyrk_dpotrf_dtrinv_lib(nx, nx, nwx, hpLAG[ii]+pnx*cnwx1, cnwx1, 0, ptr, 0, pLe_temp, cnx, hpLe[ii+1], cnx, diag);
			}
		else
			{
			if(1) // un-merged
				{
				dsyrk_nt_lib(nx, nx, nwx, hpLAG[ii]+pnx*cnwx1, cnwx1, hpLAG[ii]+pnx*cnwx1, cnwx1, 0, ptr, 0, pLe_temp, cnx);
				dsyrk_dpotrf_dtrinv_lib(nx, nx, 0, ptr, 0, 1, pLe_temp, cnx, pLe_temp, cnx, hpLe[ii+1], cnx, diag);
				}
			else
				{
				dsyrk_nt_lib(nx, nx, nwx, hpLAG[ii]+pnx*cnwx1, cnwx1, hpLAG[ii]+pnx*cnwx1, cnwx1, 0, ptr, 0, pLe_temp, cnx);
				dpotrf_lib_old(nx, nx, pLe_temp, cnx, pLe_temp, cnx, diag);
				dtrinv_lib(nx, pLe_temp, cnx, hpLe[ii+1], cnx);
				}
			}
		//d_print_pmat(nx, nx, bs, pLe_temp, cnx);
		//d_print_pmat(nx, nx, bs, hpLe[ii+1], cnx);
		//exit(1);

		for(jj=0; jj<nx; jj++) 
			{
			diag_min = fmin(diag_min, diag[jj]);
			}
		//if(diag_min==0.0)
		//	{
		//	d_print_pmat(nx, cnl, bs, GLrALeLp, cnl);
		//	d_print_pmat(2*nx, cnx2, bs, hpALe[ii+1], cnx2);
		//	}

		if(diag_min==0.0)
			return ii+1;


		//exit(1);

		}

	//dtsyrk_dpotrf_lib(nx+ndN, nx, nx, hpALe[N], cnx2, hpQA[N], cnx, hpALe[N]+(cnx)*bs, cnx2, diag, 1);
	dtsyrk_dpotrf_lib(nx+ndN, nx, nx, hpLe[N], cnx, 1, hpQRAG[N], cnx, hpLAG[N], cnx, diag);
	//d_print_pmat(pnx+pndN, cnx, bs, hpLAG[N], cnx);
	if(ndN>0)
		{
		if(ndN>pnx-nx)
			d_copy_pmat_panel(pnx-nx, nx, nx+ndN, hpLAG[N]+nx/bs*bs*cnx+nx%bs, hpLAG[N]+(nx+ndN)/bs*bs*cnx+(nx+ndN)%bs, cnx);
		else
			d_copy_pmat_panel(ndN, nx, pnx, hpLAG[N]+nx/bs*bs*cnx+nx%bs, hpLAG[N]+pnx*cnx, cnx);
		}
	//d_print_pmat(pnx+pndN, cnx, bs, hpLAG[N], cnx);

	// copy reciprocal of diagonal
	for(jj=0; jj<nx-3; jj+=4) 
		{
		diag_min = fmin(diag_min, diag[jj+0]);
		hpLAG[N][jj*cnx+0+(jj+0)*bs] = diag[jj+0]; 
		diag_min = fmin(diag_min, diag[jj+1]);
		hpLAG[N][jj*cnx+1+(jj+1)*bs] = diag[jj+1]; 
		diag_min = fmin(diag_min, diag[jj+2]);
		hpLAG[N][jj*cnx+2+(jj+2)*bs] = diag[jj+2]; 
		diag_min = fmin(diag_min, diag[jj+3]);
		hpLAG[N][jj*cnx+3+(jj+3)*bs] = diag[jj+3]; 
		}
	for(; jj<nx; jj++) 
		{
		diag_min = fmin(diag_min, diag[jj]);
		hpLAG[N][jj/bs*bs*cnx+jj%bs+jj*bs] = diag[jj]; 
		}
	//d_print_pmat(pnx+pndN, cnx, bs, hpLAG[N], cnx);

	// equality constraints at the last stage
	if(ndN>0)
		{
		int cndN = ncl*((ndN+ncl-1)/ncl);
		dsyrk_dpotrf_lib_old(ndN, ndN, nx, hpLAG[N]+pnx*cnx, cnx, 0, Ld, cndN, Ld, cndN, diag, 0);
		//d_print_pmat(ndN, ndN, bs, Ld, cndN);
		// copy reciprocal of diagonal
		for(jj=0; jj<ndN-3; jj+=4) 
			{
			Ld[jj*cndN+0+(jj+0)*bs] = diag[jj+0];
			Ld[jj*cndN+1+(jj+1)*bs] = diag[jj+1];
			Ld[jj*cndN+2+(jj+2)*bs] = diag[jj+2];
			Ld[jj*cndN+3+(jj+3)*bs] = diag[jj+3];
			}
		for(; jj<ndN; jj++) 
			{
			Ld[jj/bs*bs*cndN+jj%bs+jj*bs] = diag[jj];
			}
		//d_print_pmat(ndN, ndN, bs, Ld, cndN);
		}

	//exit(1);

#else

	for(ii=0; ii<N; ii++)
		{
		//d_print_pmat(2*nx, cnx, bs, hpQA[ii], cnx);
		//dtsyrk_dpotrf_lib(2*nx, nx, nx, hpALe[ii], cnx2, hpQA[ii], cnx, diag, 1);
		//d_print_pmat(2*nx, cnx2, bs, hpALe[ii], cnx2);
		dtsyrk_dpotrf_lib(2*nx, nx, nx, hpALe[ii], cnx2, hpQA[ii], cnx, hpALe[ii]+(cnx)*bs, cnx2, diag, 1);
		//d_print_pmat(2*nx, cnx2, bs, hpALe[ii], cnx2);
		//exit(1);
		// copy reciprocal of diagonal
		//d_print_pmat(2*nx, cnx2, bs, hpALe[ii], cnx2);
		for(jj=0; jj<nx; jj++) 
			{
			diag_min = fmin(diag_min, diag[jj]);
			hpALe[ii][cnx*bs+(jj/bs)*bs*cnx2+jj%bs+jj*bs] = diag[jj]; 
			}
		//if(diag_min==0.0)
		//	{
		//	d_print_pmat(2*nx, cnx, bs, hpQA[ii], cnx);
		//	d_print_pmat(2*nx, cnx2, bs, hpALe[ii], cnx2);
		//	}
		//d_print_pmat(2*nx, cnx2, bs, hpALe[ii], cnx2);

		if(diag_R)
			{
			for(jj=0; jj<nw; jj++)
				{
				alpha = 1.0/sqrt(hpRG[ii][jj/bs*bs*cnw+jj%bs+jj*bs]);
				for(ll=0; ll<pnwx; ll+=4)
					{
					hpGLr[ii][0+ll*cnw+jj*bs] = alpha * hpRG[ii][0+ll*cnw+jj*bs];
					hpGLr[ii][1+ll*cnw+jj*bs] = alpha * hpRG[ii][1+ll*cnw+jj*bs];
					hpGLr[ii][2+ll*cnw+jj*bs] = alpha * hpRG[ii][2+ll*cnw+jj*bs];
					hpGLr[ii][3+ll*cnw+jj*bs] = alpha * hpRG[ii][3+ll*cnw+jj*bs];
					}
				hpGLr[ii][jj/bs*bs*cnw+jj%bs+jj*bs] = alpha;
				}
			//d_print_pmat(nwx, nw, bs, hpGLr[ii], cnw);
			}
		else
			{
			dpotrf_lib_old(nwx, nw, hpRG[ii], cnw, hpGLr[ii], cnw, diag);
			//d_print_pmat(nwx, nw, bs, hpGLr[ii], cnw);

			// copy reciprocal of diagonal
			for(jj=0; jj<nw; jj++) hpGLr[ii][(jj/bs)*bs*cnw+jj%bs+jj*bs] = diag[jj]; 
			//d_print_pmat(nwx, nw, bs, hpGLr[ii], cnw);
			}
		//exit(1);

		d_align_pmat(nx, nw, nw, bs, hpGLr[ii], cnw, GLrALeLp, cnl);
		d_align_pmat(nx, nx, nx, bs, hpALe[ii]+cnx*bs, cnx2, GLrALeLp+nw*bs, cnl);
		//d_print_pmat(nx, cnl, bs, GLrALeLp, cnl);

		//dsyrk_dpotrf_dtrinv_lib(nx, nx, nwx, GLrALeLp, cnl, ptr, 0, hpALe[ii+1], cnx2, diag, 0);
		dsyrk_dpotrf_dtrinv_lib(nx, nx, nwx, GLrALeLp, cnl, ptr, 0, GLrALeLp+(nw+nx+pad)*bs, cnl, hpALe[ii+1], cnx2, diag, 0);
		//d_print_pmat(nx, cnl, bs, GLrALeLp, cnl);
		//d_print_pmat(2*nx, cnx2, bs, hpALe[ii+1], cnx2);
		for(jj=0; jj<nx; jj++) 
			{
			diag_min = fmin(diag_min, diag[jj]);
			}
		//if(diag_min==0.0)
		//	{
		//	d_print_pmat(nx, cnl, bs, GLrALeLp, cnl);
		//	d_print_pmat(2*nx, cnx2, bs, hpALe[ii+1], cnx2);
		//	}

		if(diag_min==0.0)
			return ii+1;

		//if(ii==2)
		//exit(1);
		}


	//d_print_pmat(nx, nx, bs, GLrALeLp+(nx+nw+pad)*bs, cnl);
	//dtsyrk_dpotrf_lib(nx, nx, nx, hpALe[N], cnx2, hpQA[N], cnx, diag, 1);
	dtsyrk_dpotrf_lib(nx+ndN, nx, nx, hpALe[N], cnx2, hpQA[N], cnx, hpALe[N]+(cnx)*bs, cnx2, diag, 1);
	//d_print_pmat(nx+ndN, cnx2, bs, hpALe[N], cnx2);

	// copy reciprocal of diagonal
	for(jj=0; jj<nx; jj++) 
		{
		diag_min = fmin(diag_min, diag[jj]);
		hpALe[N][cnx*bs+(jj/bs)*bs*cnx2+jj%bs+jj*bs] = diag[jj]; 
		}

	// equality constraints at the last stage
	if(ndN>0)
		{
		d_align_pmat(ndN, nx, nx, bs, hpALe[N]+cnx*bs, cnx2, GLrALeLp, cnl);

		int cndN = ncl*((ndN+ncl-1)/ncl);
		dsyrk_dpotrf_lib_old(ndN, ndN, nx, GLrALeLp, cnl, Ld, cndN, Ld, cndN, diag, 0, 0);
		for(jj=0; jj<ndN; jj++) Ld[jj/bs*bs*cndN+jj%bs+jj*bs] = diag[jj];
		//d_print_pmat(ndN, ndN, bs, Ld, cndN);
		//exit(1);
		}

#endif

	if(diag_min==0.0)
		return ii+1;

	//exit(1);

	return 0;

	}



// information filter version
void d_ric_trs_mhe_if(int nx, int nw, int ndN, int N, double **hpLe, double **hpLAG, double *Ld, double **hq, double **hr, double **hf, double **hxp, double **hx, double **hw, double **hlam, double *work)
	{

	//printf("\nin solver\n");

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl;

	const int anx = nal*((nx+nal-1)/nal);
	const int anw = nal*((nw+nal-1)/nal);
	const int pnx = bs*((nx+bs-1)/bs);
	const int pnw = bs*((nw+bs-1)/bs);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int cnw = ncl*((nw+ncl-1)/ncl);
	const int cnx2 = 2*(ncl*((nx+ncl-1)/ncl));
	const int cnwx1 = ncl*((nw+nx+1+ncl-1)/ncl);

	const int pnm = pnx>pnw ? pnx : pnw;

	int ii, jj;

	double *ptr = work;

	double *x_temp; x_temp = ptr; //d_zeros_align(&x_temp, 2*anx, 1);
	ptr += pnm+pnx; // assume ndN <= nx !!!!!


#if 1

	// if nx>=nw
	// forward substitution
	for(ii=0; ii<N; ii++)
		{

		//printf("\nii = %d\n", ii);

		// compute sigma
		for(jj=0; jj<nx; jj++) hx[ii][jj] = - hq[ii][jj];
		//d_print_mat(1, nx, hx[ii], 1);
		//d_print_mat(1, nx, hxp[ii], 1);
		//d_print_pmat(nx, nx, bs, hpALe[ii], cnx2); 
		dtrmv_u_t_lib(nx, hpLe[ii], cnx, hxp[ii], 0, x_temp); // L*(L'*b) + p
		//d_print_mat(1, nx, x_temp, 1);
		dtrmv_u_n_lib(nx, hpLe[ii], cnx, x_temp, 1, hx[ii]);
		//d_print_mat(1, nx, hx[ii], 1);

		// compute hxp
		if(nx>pnx-nx)
			{
			for(jj=0; jj<pnx-nx; jj++) hx[ii][nx+jj] = - hf[ii][nx-pnx+nx+jj];
			dtrsv_n_lib_old(pnx, nx, 1, hpLAG[ii], cnwx1, hx[ii]);
			//d_print_mat(1, pnx, hx[ii], 1);
			//for(jj=0; jj<nx; jj++) hx[ii][jj] = - hx[ii][jj];
			//d_print_mat(1, nx, hx[ii], 1);
			for(jj=0; jj<nx-pnx+nx; jj++) hxp[ii+1][jj] = hf[ii][jj];
			dgemv_n_lib(nx-pnx+nx, nx, hpLAG[ii]+pnx*cnwx1, cnwx1, hx[ii], 1, hxp[ii+1], hxp[ii+1]);
			for(jj=0; jj<pnx-nx; jj++) hxp[ii+1][nx-pnx+nx+jj] = - hx[ii][nx+jj];
			//d_print_mat(1, pnx, hxp[ii+1], 1);
			}
		else
			{
			for(jj=0; jj<nx; jj++) hx[ii][nx+jj] = - hf[ii][jj];
			dtrsv_n_lib_old(nx+nx, nx, 1, hpLAG[ii], cnwx1, hx[ii]);
			for(jj=0; jj<nx; jj++) hxp[ii+1][jj] = - hx[ii][nx+jj];
			}

		for(jj=0; jj<nw; jj++) hw[ii][jj] = hr[ii][jj];
		if(nx>pnw-nw)
			{
			for(jj=0; jj<pnw-nw; jj++) hw[ii][nw+jj] = hxp[ii+1][nx-pnw+nw+jj];
			dtrsv_n_lib_old(pnw, nw, 1, hpLAG[ii]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, hw[ii]);
			//d_print_mat(1, pnw, hw[ii], 1);
			dgemv_n_lib(nx-pnw+nw, nw, hpLAG[ii]+pnx*cnwx1+nx*bs, cnwx1, hw[ii], -1, hxp[ii+1], hxp[ii+1]);
			for(jj=0; jj<pnw-nw; jj++) hxp[ii+1][nx-pnw+nw+jj] = hw[ii][nw+jj];
			//d_print_mat(1, pnx, hxp[ii+1], 1);
			}
		else
			{
			for(jj=0; jj<nx; jj++) hw[ii][nw+jj] = hxp[ii+1][jj];
			dtrsv_n_lib_old(nw+nx, nw, 1, hpLAG[ii]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, hw[ii]);
			for(jj=0; jj<nx; jj++) hxp[ii+1][jj] = hw[ii][nw+jj];
			}


		//if(ii==1)
		//return;
		//exit(1);
		}

	// compute - sigma !!! - !!!
	//d_print_mat(1, nx, hxp[N], 1);
	for(jj=0; jj<nx; jj++) hx[N][jj] = - hq[N][jj];
	//d_print_pmat(nx, nx, bs, hpALe[N], cnx2); 
	//d_print_mat(1, nx, hx[N], 1);
	//d_print_mat(1, nx, hxp[N], 1);
	dtrmv_u_t_lib(nx, hpLe[N], cnx, hxp[N], 0, x_temp); // L*(L'*b) + p
	//d_print_mat(1, nx, x_temp, 1);
	dtrmv_u_n_lib(nx, hpLe[N], cnx, x_temp, 1, hx[N]);
	//d_print_mat(1, nx, hx[N], 1);


	// backwars substitution
	if(ndN<=0)
		{
		//d_print_pmat(nx, nx, bs, hpALe[N]+cnx*bs, cnx2); 
		//d_print_mat(1, nx, hx[N], 1);
		dtrsv_n_lib_old(nx, nx, 1, hpLAG[N], cnx, hx[N]);
		//d_print_mat(1, nx, hx[N], 1);
		dtrsv_t_lib_old(nx, nx, 1, hpLAG[N], cnx, hx[N]);
		//d_print_mat(1, nx, hx[N], 1);
		}
	else
		{
		if(ndN>pnx-nx)
			{
			for(jj=0; jj<pnx-nx; jj++) hx[N][nx+jj] = hf[N][ndN-pnx+nx+jj];
			//d_print_mat(1, pnx, hx[N], 1);
			//d_print_pmat(pnx+ndN, nx, bs, hpLAG[N], cnx);
			dtrsv_n_lib_old(pnx, nx, 1, hpLAG[N], cnx, hx[N]);
			//d_print_mat(1, pnx, hx[N], 1);
			for(jj=0; jj<ndN-pnx+nx; jj++) hlam[N][jj] = - hf[N][jj];
			//d_print_mat(1, ndN, hlam[N], 1);
			dgemv_n_lib(ndN-pnx+nx, nx, hpLAG[N]+pnx*cnx, cnx, hx[N], 1, hlam[N], hlam[N]);
			for(jj=0; jj<pnx-nx; jj++) hlam[N][ndN-pnx+nx+jj] = - hx[N][nx+jj];
			}
		else
			{
			for(jj=0; jj<ndN; jj++) hx[N][nx+jj] = hf[N][jj];
			dtrsv_n_lib_old(nx+ndN, nx, 1, hpLAG[N], cnx, hx[N]);
			for(jj=0; jj<ndN; jj++) hlam[N][jj] = - hx[N][nx+jj];
			}

		//d_print_mat(1, ndN, hlam[N], 1);
		int cndN = ncl*((ndN+ncl-1)/ncl);
		//d_print_mat(1, ndN, d, 1);
		//d_print_pmat(ndN, ndN, bs, Ld, cndN);
		dtrsv_n_lib_old(ndN, ndN, 1, Ld, cndN, hlam[N]);
		//d_print_mat(1, ndN, hlam[N], 1);
		dtrsv_t_lib_old(ndN, ndN, 1, Ld, cndN, hlam[N]);
		//d_print_mat(1, ndN, hlam[N], 1);
		//exit(1);

		if(ndN>pnx-nx)
			{
			//d_print_mat(1, pnx, hx[N], 1);
			dgemv_t_lib(ndN-pnx+nx, nx, hpLAG[N]+pnx*cnx, cnx, hlam[N], -1, hx[N], hx[N]);
			//d_print_mat(1, pnx, hx[N], 1);
			for(jj=0; jj<pnx-nx; jj++) hx[N][nx+jj] = hlam[N][ndN-pnx+nx+jj];
			//d_print_mat(1, pnx, hx[N], 1);
			dtrsv_t_lib_old(pnx, nx, 1, hpLAG[N], cnx, hx[N]);
			}
		else
			{
			for(jj=0; jj<ndN; jj++) hx[N][nx+jj] = hlam[N][jj];
			dtrsv_t_lib_old(nx+ndN, nx, 1, hpLAG[N], cnx, hx[N]);
			}
			

		//d_print_mat(1, nx, hx[N], 1);
		//exit(1);

		}

	for(ii=0; ii<N; ii++)
		{

		// compute lambda
//		for(jj=0; jj<nx; jj++) x_temp[jj] = hxp[N-ii][jj] - hx[N-ii][jj];
		//d_print_mat(1, nx, x_temp, 1);
//		dtrmv_u_t_lib(nx, hpLe[N-ii], cnx, x_temp, x_temp+anx, 0); // L*(L'*b) + p
		//d_print_mat(1, nx, x_temp+anx, 1);
//		dtrmv_u_n_lib(nx, hpLe[N-ii], cnx, x_temp+anx, hlam[N-ii-1], 0);
		//d_print_mat(1, nx, hlam[N-ii-1], 1);

		for(jj=0; jj<nx; jj++) x_temp[pnx+jj] = hxp[N-ii][jj] - hx[N-ii][jj];
		dtrmv_u_t_lib(nx, hpLe[N-ii], cnx, x_temp+pnx, 0, x_temp); // L*(L'*b) + p
		dtrmv_u_n_lib(nx, hpLe[N-ii], cnx, x_temp, 0, x_temp+pnx);
		//d_print_mat(1, pnx+nx, x_temp, 1);

		// compute x
		if(nx>pnx-nx)
			{
			//for(jj=0; jj<nx; jj++) x_temp[jj] = hx[N-ii-1][jj];
			//for(jj=0; jj<nx; jj++) x_temp[nx+jj] = hlam[N-ii-1][jj];
//			dgemv_t_lib(nx-pnx+nx, nx, hpLAG[N-ii-1]+pnx*cnwx1, cnwx1, hlam[N-ii-1], hx[N-ii-1], -1);
//			for(jj=0; jj<pnx-nx; jj++) hx[N-ii-1][nx+jj] = hlam[N-ii-1][nx-pnx+nx+jj];
//			dtrsv_t_lib_old(pnx, nx, 1, hpLAG[N-ii-1], cnwx1, hx[N-ii-1]);
			//for(jj=0; jj<nx; jj++) hx[N-ii-1][jj] = x_temp[jj];
			//d_print_mat(1, nx, hx[N-ii-1], 1);

			for(jj=0; jj<nx; jj++) x_temp[jj] = hx[N-ii-1][jj];
			for(jj=0; jj<pnx-nx; jj++) x_temp[nx+jj] = x_temp[pnx+nx-pnx+nx+jj];
			dtrsv_t_lib_old(2*nx, nx, 1, hpLAG[N-ii-1], cnwx1, x_temp);
			for(jj=0; jj<nx; jj++) hx[N-ii-1][jj] = x_temp[jj];
			//d_print_mat(1, nx, hx[N-ii-1], 1);
			}
		else
			{
//			for(jj=0; jj<nx; jj++) hx[N-ii-1][nx+jj] = hlam[N-ii-1][jj];
//			dtrsv_t_lib_old(nx+nx, nx, 1, hpLAG[N-ii-1], cnwx1, hx[N-ii-1]);

			for(jj=0; jj<nx; jj++) hx[N-ii-1][nx+jj] = x_temp[pnx+jj];
			dtrsv_t_lib_old(nx+nx, nx, 1, hpLAG[N-ii-1], cnwx1, hx[N-ii-1]);
			}

		// compute w

		//d_print_mat(1, pnx+nx, x_temp, 1);
		if(nx>pnw-nw)
			{
//			for(jj=0; jj<nw; jj++) hw[N-ii-1][jj] = - hw[N-ii-1][jj];
//			dgemv_t_lib(nx-pnw+nw, nw, hpLAG[N-ii-1]+pnx*cnwx1+nx*bs, cnwx1, hlam[N-ii-1], hw[N-ii-1], -1);
//			for(jj=0; jj<pnw-nw; jj++) hw[N-ii-1][nw+jj] = hlam[N-ii-1][nx-pnw+nw+jj];
//			dtrsv_t_lib_old(pnw, nw, 1, hpLAG[N-ii-1]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, hw[N-ii-1]);
			//d_print_mat(1, nw, hw[N-ii-1], 1);

			for(jj=0; jj<nw; jj++) x_temp[pnx-pnw+jj] = - hw[N-ii-1][jj];
			for(jj=0; jj<pnw-nw; jj++) x_temp[pnx-pnw+nw+jj] = x_temp[pnx+nx-pnw+nw+jj];
			//d_print_mat(1, pnx+nx, x_temp, 1);
			dtrsv_t_lib_old(nw+nx, nw, 1, hpLAG[N-ii-1]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, x_temp+pnx-pnw);
			for(jj=0; jj<nw; jj++) hw[N-ii-1][jj] = x_temp[pnx-pnw+jj];
			//d_print_mat(1, nw, hw[N-ii-1], 1);
			}
		else
			{
			for(jj=0; jj<nw; jj++) hw[N-ii-1][jj] = - hw[N-ii-1][jj];
			for(jj=0; jj<nx; jj++) hw[N-ii-1][nw+jj] = x_temp[pnx+jj];
			dtrsv_t_lib_old(nw+nx, nw, 1, hpLAG[N-ii-1]+(pnx-pnw)*cnwx1+nx*bs, cnwx1, hw[N-ii-1]);
			}

		for(jj=0; jj<nx; jj++) hlam[N-ii-1][jj] = x_temp[pnx+jj];

		}

	//d_print_mat(1, nw, hw[0], 1);
	//exit(1);
	//return;

#else

	// forward substitution
	for(ii=0; ii<N; ii++)
		{

		//printf("\nii = %d\n", ii);

		// compute sigma
		for(jj=0; jj<nx; jj++) hx[ii][jj] = hq[ii][jj];
		//d_print_mat(1, nx, hx[ii], 1);
		//d_print_mat(1, nx, hxp[ii], 1);
		//d_print_pmat(nx, nx, bs, hpALe[ii], cnx2); 
		dtrmv_u_t_lib(nx, hpALe[ii], cnx2, hxp[ii], x_temp, 0); // L*(L'*b) + p
		//d_print_mat(1, nx, x_temp, 1);
		dtrmv_u_n_lib(nx, hpALe[ii], cnx2, x_temp, hx[ii], -1);
		//d_print_mat(1, nx, hx[ii], 1);

		// compute hxp
		for(jj=0; jj<nx; jj++) x_temp[jj] = hx[ii][jj];
		for(jj=0; jj<nx; jj++) x_temp[nx+jj] = hf[ii][jj];
		//d_print_pmat(2*nx, nx, bs, hpALe[ii]+cnx*bs, cnx2);
		dtrsv_n_lib_old(2*nx, nx, 1, hpALe[ii]+cnx*bs, cnx2, x_temp);
		for(jj=0; jj<nx; jj++) hx[ii][jj] = - x_temp[jj]; // restore sign
		//d_print_mat(1, 2*nx, x_temp, 1);
		for(jj=0; jj<nw; jj++) x_temp[jj] = hr[ii][jj];
		for(jj=0; jj<nx; jj++) x_temp[nw+jj] = x_temp[nx+jj];
		dtrsv_n_lib_old(nw+nx, nw, 1, hpGLr[ii], cnw, x_temp);
		//d_print_mat(1, nw+nx, x_temp, 1);
		for(jj=0; jj<nw; jj++) hw[ii][jj] = x_temp[jj];
		for(jj=0; jj<nx; jj++) hxp[ii+1][jj] = x_temp[nw+jj];
		//d_print_mat(1, nw, hw[ii], 1);
		//d_print_mat(1, nx, hxp[ii+1], 1);
	
		//if(ii==1)
		//return 0;
		//exit(1);
		}

	// compute - sigma !!! - !!!
	for(jj=0; jj<nx; jj++) hx[N][jj] = - hq[N][jj];
	//d_print_pmat(nx, nx, bs, hpALe[N], cnx2); 
	//d_print_mat(1, nx, hx[N], 1);
	//d_print_mat(1, nx, hxp[N], 1);
	dtrmv_u_t_lib(nx, hpALe[N], cnx2, hxp[N], x_temp, 0); // L*(L'*b) + p
	//d_print_mat(1, nx, x_temp, 1);
	dtrmv_u_n_lib(nx, hpALe[N], cnx2, x_temp, hx[N], 1);
	//d_print_mat(1, nx, hx[N], 1);


	// backwars substitution
	if(ndN<=0)
		{
		//d_print_pmat(nx, nx, bs, hpALe[N]+cnx*bs, cnx2); 
		//d_print_mat(1, nx, hx[N], 1);
		dtrsv_n_lib_old(nx, nx, 1, hpALe[N]+cnx*bs, cnx2, hx[N]);
		//d_print_mat(1, nx, hx[N], 1);
		dtrsv_t_lib_old(nx, nx, 1, hpALe[N]+cnx*bs, cnx2, hx[N]);
		//d_print_mat(1, nx, hx[N], 1);
		}
	else
		{
		for(jj=0; jj<nx; jj++) x_temp[jj] = hx[N][jj];
		for(jj=0; jj<ndN; jj++) x_temp[nx+jj] = hf[N][jj];
		//d_print_mat(1, nx+ndN, x_temp, 1);
		dtrsv_n_lib_old(nx+ndN, nx, 1, hpALe[N]+cnx*bs, cnx2, x_temp);
		//d_print_mat(1, nx+ndN, x_temp, 1);
		//exit(1);

		for(jj=0; jj<ndN; jj++) hlam[N][jj] = - x_temp[nx+jj];
		int cndN = ncl*((ndN+ncl-1)/ncl);
		//d_print_mat(1, ndN, d, 1);
		//d_print_pmat(ndN, ndN, bs, Ld, cndN);
		dtrsv_n_lib_old(ndN, ndN, 1, Ld, cndN, hlam[N]);
		//d_print_mat(1, ndN, hlam[N], 1);
		dtrsv_t_lib_old(ndN, ndN, 1, Ld, cndN, hlam[N]);
		//d_print_mat(1, ndN, hlam[N], 1);
		//exit(1);

		for(jj=0; jj<ndN; jj++) x_temp[nx+jj] = hlam[N][jj];
		//d_print_mat(1, nx+ndN, x_temp, 1);
		dtrsv_t_lib_old(nx+ndN, nx, 1, hpALe[N]+cnx*bs, cnx2, x_temp);

		for(jj=0; jj<nx; jj++) hx[N][jj] = x_temp[jj];
		//d_print_mat(1, nx+ndN, x_temp, 1);
		//exit(1);

		}

	for(ii=0; ii<N; ii++)
		{

		// compute lambda
		for(jj=0; jj<nx; jj++) x_temp[jj] = hxp[N-ii][jj] - hx[N-ii][jj];
		//d_print_mat(1, nx, x_temp, 1);
		dtrmv_u_t_lib(nx, hpALe[N-ii], cnx2, x_temp, x_temp+anx, 0); // L*(L'*b) + p
		//d_print_mat(1, nx, x_temp+anx, 1);
		dtrmv_u_n_lib(nx, hpALe[N-ii], cnx2, x_temp+anx, hlam[N-ii-1], 0);
		//d_print_mat(1, nx, hlam[N-ii-1], 1);

		// compute x
		for(jj=0; jj<nx; jj++) x_temp[jj] = hx[N-ii-1][jj];
		for(jj=0; jj<nx; jj++) x_temp[nx+jj] = hlam[N-ii-1][jj];
		dtrsv_t_lib_old(nx+nx, nx, 1, hpALe[N-ii-1]+cnx*bs, cnx2, x_temp);
		for(jj=0; jj<nx; jj++) hx[N-ii-1][jj] = x_temp[jj];
		//d_print_mat(1, nx, hx[N-ii-1], 1);

		// compute w
		for(jj=0; jj<nw; jj++) x_temp[jj] = -hw[N-ii-1][jj];
		for(jj=0; jj<nx; jj++) x_temp[nw+jj] = hlam[N-ii-1][jj];
		dtrsv_t_lib_old(nw+nx, nw, 1, hpGLr[N-ii-1], cnw, x_temp);
		for(jj=0; jj<nw; jj++) hw[N-ii-1][jj] = x_temp[jj];
		//d_print_mat(1, nw, hw[N-ii-1], 1);

		}

#endif

	//d_print_mat(1, nw, hw[0], 1);

	// free memory TODO remove !!!
	//free(*sigma);
	//free(x_temp);
	//free(x2_temp);
	//free(wx_temp);

	//exit(1);

	return;

	}



// xp is the vector of predictions, xe is the vector of estimates
//#if 0
int d_ric_trs_mhe(int nx, int nw, int ny, int N, double **hpA, double **hpG, double **hpC, double **hpLp, double **hdLp, double **hpQ, double **hpR, double **hpLe, double **hq, double **hr, double **hf, double **hxp, double **hxe, double **hw, double **hy, int smooth, double **hlam, double *work)
	{

	//printf("\nhola\n");

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl;

	const int nz = nx+ny;
	const int anx = nal*((nx+nal-1)/nal);
	const int anw = nal*((nw+nal-1)/nal);
	const int any = nal*((ny+nal-1)/nal);
	const int anz = nal*((nz+nal-1)/nal);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int cnw = ncl*((nw+ncl-1)/ncl);
	const int cny = ncl*((ny+ncl-1)/ncl);
	const int cnz = ncl*((nz+ncl-1)/ncl);
	const int cnf = cnz<cnx+ncl ? cnx+ncl : cnz;

	const int pad = (ncl-(nx+nw)%ncl)%ncl; // packing between AGL & P
	const int cnl = nx+nw+pad+cnx;

	int ii, jj, ll;
	int return_value = 0;
	double *ptr;
	double Lmin;

	ptr = work;

	double *y_temp = ptr; //; d_zeros_align(&y_temp, anz, 1);
	ptr += anz;

	double *w_temp = ptr; //; d_zeros_align(&w_temp, anw, 1);
	ptr += anw;

	double *x_temp = ptr; //; d_zeros_align(&w_temp, anw, 1);
	ptr += 2*anx;

	// loop over horizon
	for(ii=0; ii<N; ii++)
		{

		//printf("\nii = %d\n", ii);

		// copy y
		for(jj=0; jj<ny; jj++) y_temp[jj] = - hy[ii][jj];
		//d_print_mat(1, nz, y_temp, 1);
	
		// compute y + R*r
		dsymv_lib(ny, ny, hpR[ii], cny, hr[ii], -1, y_temp, y_temp);
		//d_print_mat(1, nz, y_temp, 1);

		// compute y + R*r - C*xp
		//int pny = bs*((ny+bs-1)/bs);
		//d_print_pmat(pny, cnx, bs, hpC[ii], cnx);
		//d_print_mat(1, anx, hxp[ii], 1);
		dgemv_n_lib(ny, nx, hpC[ii], cnx, hxp[ii], 1, y_temp, y_temp);
		//d_print_mat(1, nz, y_temp, 1);

		//d_print_pmat(nz, ny, bs, hpLe[ii], cnf);

		// copy xp
		for(jj=0; jj<nx; jj++) y_temp[ny+jj] = hxp[ii][jj];
		//d_print_mat(1, nz, y_temp, 1);
	
		// compute xe
		dtrsv_n_lib_old(ny+nx, ny, 1, hpLe[ii], cnf, y_temp);
		//d_print_mat(1, nz, y_temp, 1);

		// copy xe
		for(jj=0; jj<nx; jj++) hxe[ii][jj] = y_temp[ny+jj];
		//d_print_mat(1, nx, hxe[ii], 1);
		//exit(1);

		// copy f in xp
		for(jj=0; jj<nx; jj++) hxp[ii+1][jj] = hf[ii][jj];
		//d_print_mat(1, nx, hxp[ii+1], 1);
	
		// xp += A*xe
		dgemv_n_lib(nx, nx, hpA[ii], cnx, hxe[ii], 1, hxp[ii+1], hxp[ii+1]);
		//d_print_mat(1, nx, hxp[ii+1], 1);

		// initialize w with 0
		for(jj=0; jj<nw; jj++) hw[ii][jj] = 0.0;
		//d_print_mat(1, nw, w_temp, 1);
	
		// compute Q*q
		dsymv_lib(nw, nw, hpQ[ii], cnw, hq[ii], -1, hw[ii], hw[ii]);
		//d_print_mat(1, nw, w_temp, 1);

		// xp += G*w
		dgemv_n_lib(nx, nw, hpG[ii], cnw, hw[ii], 1, hxp[ii+1], hxp[ii+1]);
		//d_print_mat(1, nx, hxp[ii+1], 1);
	
		//if(ii==1)
		//return 0;
		//exit(1);

		}
	
	// stage N

	// copy y
	for(jj=0; jj<ny; jj++) y_temp[jj] = - hy[N][jj];
	//d_print_mat(1, nz, y_temp, 1);
	
	// compute y + R*r
	dsymv_lib(ny, ny, hpR[N], cny, hr[N], -1, y_temp, y_temp);
	//d_print_mat(1, nz, y_temp, 1);

	// compute y + R*r - C*xp
	dgemv_n_lib(ny, nx, hpC[N], cnx, hxp[N], 1, y_temp, y_temp);
	//d_print_mat(1, nz, y_temp, 1);

	//d_print_pmat(nz, ny, bs, hpLe[N], cnf);

	// copy xp
	for(jj=0; jj<nx; jj++) y_temp[ny+jj] = hxp[N][jj];
	//d_print_mat(1, nz, y_temp, 1);
	
	// compute xe
	dtrsv_n_lib_old(ny+nx, ny, 1, hpLe[N], cnf, y_temp);
	//d_print_mat(1, nz, y_temp, 1);

	// copy xe
	for(jj=0; jj<nx; jj++) hxe[N][jj] = y_temp[ny+jj];
	//d_print_mat(1, nx, hxe[N], 1);

	//return 0;
	//exit(1);

	if(smooth==0)
		return return_value;
	
	// backward recursion to compute smoothed values

	for(ii=0; ii<N; ii++)
		{

		//printf("\nii = %d\n", ii);

		// check for singular covariance
		Lmin = 1;
		for(jj=0; jj<nx; jj++) Lmin = fmin(Lmin, hdLp[N-ii][jj]);
		//printf("\nL_min = %f\n", Lmin);

		// if singular, keep the current estimate as smooth value and go to the next iteration
		if(Lmin==0.0)
			{

			// the N-ii th prediction covariance matrix is singular
			return_value = N-ii;

			}
		// else compute smooth values
		else
			{

			// backup diagonal and overwrite with inverted diagonal
			//d_print_pmat(nx, nx, bs, hpLp[N-ii]+(nx+nw+pad)*bs, cnl);
			for(jj=0; jj<nx; jj++)
				{
				x_temp[jj] = hpLp[N-ii][(jj/bs)*bs*cnl+jj%bs+(nx+nw+pad+jj)*bs];
				hpLp[N-ii][(jj/bs)*bs*cnl+jj%bs+(nx+nw+pad+jj)*bs] = hdLp[N-ii][jj];
				}
			//d_print_pmat(nx, nx, bs, hpLp[N-ii]+(nx+nw+pad)*bs, cnl);

			// lam = xp - xe
			for(jj=0; jj<nx; jj++) hlam[N-ii-1][jj] = hxp[N-ii][jj] - hxe[N-ii][jj];
			//d_print_mat(1, nx, hlam[N-ii-1], 1);

			// lam = \Pi^{-1}*lam
			dtrsv_n_lib_old(nx, nx, 1, hpLp[N-ii]+(nx+nw+pad)*bs, cnl, hlam[N-ii-1]);
			//d_print_mat(1, nx, hlam[N-ii-1], 1);
			dtrsv_t_lib_old(nx, nx, 1, hpLp[N-ii]+(nx+nw+pad)*bs, cnl, hlam[N-ii-1]);
			//d_print_mat(1, nx, hlam[N-ii-1], 1);

			// restore diagonal
			for(jj=0; jj<nx; jj++)
				hpLp[N-ii][(jj/bs)*bs*cnl+jj%bs+(nx+nw+pad+jj)*bs] = x_temp[jj];
			//d_print_pmat(nx, nx, bs, hpLp[N-ii]+(nx+nw+pad)*bs, cnl);

			// G'*lam
			//d_print_pmat(nx, nw, bs, hpG[N-ii-1], cnw);
			dgemv_t_lib(nx, nw, hpG[N-ii-1], cnw, hlam[N-ii-1], 0, w_temp, w_temp);
			//d_print_mat(nw, 1, w_temp, 1);

			// w = w - Q*G'*lam
			//d_print_pmat(nw, nw, bs, hpQ[N-ii-1], cnw);
			dsymv_lib(nw, nw, hpQ[N-ii-1], cnw, w_temp, -1, hw[N-ii-1], hw[N-ii-1]);
			//d_print_mat(nw, 1, hw[N-ii-1], 1);

			// A'*lam
			//d_print_pmat(nx, nx, bs, hpA[N-ii-1], cnx);
			dgemv_t_lib(nx, nx, hpA[N-ii-1], cnx, hlam[N-ii-1], 0, x_temp, x_temp);
			//d_print_mat(nx, 1, x_temp, 1);

			// xe = xe - Pi_e*A'*lam
			//d_print_pmat(nx, nx, bs, hpLe[N-ii-1]+ncl*bs, cnf);
			dtrmv_u_n_lib(nx, hpLe[N-ii-1]+ncl*bs, cnf, x_temp, 0, x_temp+anx);
			//d_print_mat(nx, 1, x_temp+anx, 1);
			//d_print_mat(nx, 1, hxe[N-ii-1], 1);
			dtrmv_u_t_lib(nx, hpLe[N-ii-1]+ncl*bs, cnf, x_temp+anx, -1, hxe[N-ii-1]); // L*(L'*b) + p
			//d_print_mat(nx, 1, hxe[N-ii-1], 1);

			//exit(1);

			}

		//if(ii==40)
		//	exit(1);

		}

	return return_value;

	}
//#endif



// xp is the vector of predictions, xe is the vector of estimates; explicitly computes estimates only at the last stage
//#if 0
void d_ric_trs_mhe_end(int nx, int nw, int ny, int N, double **hpA, double **hpG, double **hpC, double **hpLp, double **hpQ, double **hpR, double **hpLe, double **hq, double **hr, double **hf, double **hxp, double **hxe, double **hy, double *work)
	{

	//printf("\nhola\n");

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl;

	const int nz = nx+ny;
	const int anw = nal*((nw+nal-1)/nal);
	const int any = nal*((ny+nal-1)/nal);
	const int anz = nal*((nz+nal-1)/nal);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int cnw = ncl*((nw+ncl-1)/ncl);
	const int cny = ncl*((ny+ncl-1)/ncl);
	const int cnz = ncl*((nz+ncl-1)/ncl);
	const int cnf = cnz<cnx+ncl ? cnx+ncl : cnz;

	const int pad = (ncl-(nx)%ncl)%ncl; // packing between AGL & P
	const int cnl = cnz<cnx+ncl ? nx+pad+cnx+ncl : nx+pad+cnz;

	int ii, jj, ll;
	double *ptr;

	ptr = work;

	double *y_temp = ptr; //; d_zeros_align(&y_temp, anz, 1);
	ptr += anz;

	double *w_temp = ptr; //; d_zeros_align(&w_temp, anw, 1);
	ptr += anw;

	// loop over horizon
	for(ii=0; ii<N; ii++)
		{

		//printf("\nii = %d\n", ii);

		// copy y
		for(jj=0; jj<ny; jj++) y_temp[jj] = - hy[ii][jj];
		//d_print_mat(1, nz, y_temp, 1);
	
		// copy A*xp
		//for(jj=0; jj<nx; jj++) y_temp[ny+jj] = hxp[ii][jj];
		dgemv_n_lib(nx, nx, hpA[ii], cnx, hxp[ii], 0, y_temp+ny, y_temp+ny);
		//d_print_mat(1, nz, y_temp, 1);
	
		// compute y + R*r
		dsymv_lib(ny, ny, hpR[ii], cny, hr[ii], -1, y_temp, y_temp);
		//d_print_mat(1, nz, y_temp, 1);

		// compute y + R*r - C*xp
		dgemv_n_lib(ny, nx, hpC[ii], cnx, hxp[ii], 1, y_temp, y_temp);
		//d_print_mat(1, nz, y_temp, 1);

		//d_print_pmat(nz, ny, bs, hpLp[ii+1]+(nx+pad)*bs, cnl);
		//d_print_pmat(nz, cnl, bs, hpLp[ii+1], cnl);

		// compute A*xe
		dtrsv_n_lib_old(ny+nx, ny, 1, hpLp[ii+1]+(nx+pad)*bs, cnl, y_temp);
		//d_print_mat(1, nz, y_temp, 1);

		// copy A*xe in xp
		for(jj=0; jj<nx; jj++) hxp[ii+1][jj] = y_temp[ny+jj];
		//d_print_mat(1, nx, hxp[ii+1], 1);

		// add f to xp
		for(jj=0; jj<nx; jj++) hxp[ii+1][jj] += hf[ii][jj];
		//d_print_mat(1, nx, hxp[ii+1], 1);
	
		// initialize w_temp with 0
		for(jj=0; jj<nw; jj++) w_temp[jj] = 0.0;
		//d_print_mat(1, nw, w_temp, 1);
	
		// compute Q*q
		dsymv_lib(nw, nw, hpQ[ii], cnw, hq[ii], -1, w_temp, w_temp);
		//d_print_mat(1, nw, w_temp, 1);

		// xp += G*w_temp
		dgemv_n_lib(nx, nw, hpG[ii], cnw, w_temp, 1, hxp[ii+1], hxp[ii+1]);
		//d_print_mat(1, nx, hxp[ii+1], 1);
	
		// xp += A*xe
		//dgemv_n_lib(nx, nx, hpA[ii], cnx, hxe[ii], hxp[ii+1], 1);
		//d_print_mat(1, nx, hxp[ii+1], 1);

		//if(ii==1)
		//exit(1);

		}
	
	// stage N

	// copy y
	for(jj=0; jj<ny; jj++) y_temp[jj] = - hy[N][jj];
	//d_print_mat(1, nz, y_temp, 1);
	
	// copy xp
	for(jj=0; jj<nx; jj++) y_temp[ny+jj] = hxp[N][jj];
	//d_print_mat(1, nz, y_temp, 1);
	
	// compute y + R*r
	dsymv_lib(ny, ny, hpR[N], cny, hr[N], -1, y_temp, y_temp);
	//d_print_mat(1, nz, y_temp, 1);

	// compute y + R*r - C*xp
	dgemv_n_lib(ny, nx, hpC[N], cnx, hxp[N], 1, y_temp, y_temp);
	//d_print_mat(1, nz, y_temp, 1);

	//d_print_pmat(nz, ny, bs, hpLe[N], cnf);

	// compute xe
	dtrsv_n_lib_old(ny+nx, ny, 1, hpLe[N], cnf, y_temp);
	//d_print_mat(1, nz, y_temp, 1);

	// copy xe
	for(jj=0; jj<nx; jj++) hxe[N][jj] = y_temp[ny+jj];
	//d_print_mat(1, nx, hxe[N], 1);

	//exit(1);

	}
//#endif



//#if defined(TARGET_C99_4X4)
// version tailored for MHE
void d_ric_trf_mhe(int nx, int nw, int ny, int N, double **hpA, double **hpG, double **hpC, double **hpLp, double **hdLp, double **hpQ, double **hpR, double **hpLe, double *work)
	{

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl;

	const int nz = nx+ny;
	const int anz = nal*((nz+nal-1)/nal);
	const int pnx = bs*((nx+bs-1)/bs);
	const int pnw = bs*((nw+bs-1)/bs);
	const int pny = bs*((ny+bs-1)/bs);
	const int pnz = bs*((nz+bs-1)/bs);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int cnw = ncl*((nw+ncl-1)/ncl);
	const int cny = ncl*((ny+ncl-1)/ncl);
	const int cnz = ncl*((nz+ncl-1)/ncl);
	const int cnf = cnz<cnx+ncl ? cnx+ncl : cnz;

	const int pad = (ncl-(nx+nw)%ncl)%ncl; // packing between AGL & P
	const int cnl = nx+nw+pad+cnx;

	int ii, jj, ll;
	double *ptr;

	ptr = work;

	double *CL = ptr; //d_zeros_align(&CL, pny, cnx);
	ptr += pny*cnx;

	double *CLLt = ptr; //d_zeros_align(&CLLt, pny, cnx);
	ptr += pny*cnx;

	double *diag = ptr; // d_zeros_align(&diag, anz, 1);
	ptr += anz;

	double *Lam_w = ptr; // d_zeros_align(&Lam_w, pnw, cnw);
	ptr += pnw*cnw;
	
	double *Pi_p = ptr; //  d_zeros_align(&Pi_p, pnx, cnx);
	ptr += pnx*cnx;

	//double *CLLt = ptr; d_zeros_align(&CLLt, pny, cnx);
	static double buffer[6] = {};

	double *ptr1;

	// compute /Pi_p from its cholesky factor
	//d_print_pmat(nx, nx, bs, hpLp[0]+(nx+nw+pad)*bs, cnl);
	dsyttmm_lu_lib(nx, hpLp[0]+(nx+nw+pad)*bs, cnl, Pi_p, cnx);
	//d_print_pmat(nx, nx, bs, Pi_p, cnx);

	// copy /Pi_p on the bottom right block of Lam
	dtrcp_l_lib(nx, 0, Pi_p, cnx, ny, hpLe[0]+(ny/bs)*bs*cnf+ny%bs+ny*bs, cnf);
	//d_print_pmat(nz, nz, bs, Lam, cnz);

	// loop over horizon
	for(ii=0; ii<N; ii++)
		{

		// backup the top-left part of the bottom right block of Lam
		ptr1 = buffer;
		for(jj=ny; jj<((ny+bs-1)/bs)*bs; jj+=1)
			{
			ptr = &hpLe[ii][(jj/bs)*bs*cnf+jj%bs+jj*bs];
			ptr1[0] = ptr[0];
			ptr += 1;
			ptr1 += 1;
			for(ll=jj+1; ll<((ny+bs-1)/bs)*bs; ll+=1)
				{
				ptr1[0] = ptr[0];
				ptr += 1;
				ptr1 += 1;
				}
			}
		//d_print_mat(6, 1, buffer, 1);

		// compute C*U', with U upper cholesky factor of /Pi_p
		dtrmm_nt_u_lib(ny, nx, hpC[ii], cnx, hpLp[ii]+(nx+nw+pad)*bs, cnl, CL, cnx);
		//d_print_pmat(ny, nx, bs, CL, cnx);

		// compute R + (C*U')*(C*U')' on the top left of Lam
		dsyrk_nt_lib(ny, ny, nx, CL, cnx, CL, cnx, 1, hpR[ii], cny, hpLe[ii], cnf);
		//d_print_pmat(nz, nz, bs, Lam, cnz);

		// recover overwritten part of /Pi_p in bottom right part of Lam
		ptr1 = buffer;
		for(jj=ny; jj<((ny+bs-1)/bs)*bs; jj+=1)
			{
			ptr = &hpLe[ii][(jj/bs)*bs*cnf+jj%bs+jj*bs];
			ptr[0] = ptr1[0];
			ptr += 1;
			ptr1 += 1;
			for(ll=jj+1; ll<((ny+bs-1)/bs)*bs; ll+=1)
				{
				ptr[0] = ptr1[0];
				ptr += 1;
				ptr1 += 1;
				}
			}
		//d_print_pmat(nz, nz, bs, Lam, cnz);

		// compute C*U'*L'
		dtrmm_nt_l_lib(ny, nx, CL, cnx, hpLp[ii]+(nx+nw+pad)*bs, cnl, CLLt, cnx);
		//d_print_pmat(ny, nx, bs, CLLt, cnx);

		// copy C*U'*L' on the bottom left of Lam
		dgetr_lib(ny, nx, 0, CLLt, cnx, ny, hpLe[ii]+(ny/bs)*bs*cnf+ny%bs, cnf);
		//d_print_pmat(nz, nz, bs, Lam, cnz);

		// cholesky factorization of Lam
		dpotrf_lib_old(nz, nz, hpLe[ii], cnf, hpLe[ii], cnf, diag);
		//d_print_pmat(nz, nz, bs, hpLe[ii], cnf);
		//d_print_pmat(nz, nz, bs, Lam, cnz);
		//d_print_mat(nz, 1, diag, 1);

		// inverted diagonal of top-left part of hpLe
		for(jj=0; jj<ny; jj++) hpLe[ii][(jj/bs)*bs*cnf+jj%bs+jj*bs] = diag[jj];

		// transpose and align /Pi_e
		dtrtr_l_lib(nx, ny, hpLe[ii]+(ny/bs)*bs*cnf+ny%bs+ny*bs, cnf, 0, hpLe[ii]+ncl*bs, cnf);	
		//d_print_pmat(nz, nz, bs, hpLe[ii], cnf);

		// compute A*U', with U' upper cholesky factor of /Pi_e
		// d_print_pmat(nx, nx, bs, hpA[ii], cnx);
		dtrmm_nt_u_lib(nx, nx, hpA[ii], cnx, hpLe[ii]+ncl*bs, cnf, hpLp[ii+1], cnl);
		//d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii+1], cnl);

		// compute lower cholesky factor of Q
		dpotrf_lib_old(nw, nw, hpQ[ii], cnw, Lam_w, cnw, diag);
		//d_print_pmat(nw, nw, bs, Lam_w, cnw);

		// transpose in place the lower cholesky factor of Q
		dtrtr_l_lib(nw, 0, Lam_w, cnw, 0, Lam_w, cnw);	
		//d_print_pmat(nw, nw, bs, Lam_w, cnw);

		// compute G*U', with U' upper cholesky factor of Q
		// d_print_pmat(nx, nw, bs, hpG[ii], cnw);
		dtrmm_nt_u_lib(nx, nw, hpG[ii], cnw, Lam_w, cnw, hpLp[ii+1]+nx*bs, cnl);
		//d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii+1], cnl);

		// compute /Pi_p
		dsyrk_nt_lib(nx, nx, nx+nw, hpLp[ii+1], cnl, hpLp[ii+1], cnl, 0, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, hpLp[ii+1]+(nx+nw+pad)*bs, cnl);
		//d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii+1], cnl);
		//d_print_pmat(nx, nx, bs, hpLp[ii+1]+(nx+nw+pad)*bs, cnl);

		// copy /Pi_p on the bottom right block of Lam
		dtrcp_l_lib(nx, 0, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, ny, hpLe[ii+1]+(ny/bs)*bs*cnf+ny%bs+ny*bs, cnf);
		//d_print_pmat(nz, nz, bs, Lam, cnz);

		// factorize Pi_p
		dpotrf_lib_old(nx, nx, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, hdLp[ii+1]); //diag);
		//d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii+1], cnl);

		// transpose in place the lower cholesky factor of /Pi_p
		dtrtr_l_lib(nx, 0, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, 0, hpLp[ii+1]+(nx+nw+pad)*bs, cnl);	
		//d_print_pmat(nx, cnl, bs, hpLp[ii+1], cnl);
		//d_print_pmat(nx, nx, bs, hpLp[ii+1]+(nx+nw+pad)*bs, cnl);


		//dsyttmm_lu_lib(nx, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, Pi_p, cnx);
		//d_print_pmat(nx, nx, bs, Pi_p, cnx);
		//exit(1);
		}

	// stage N

	// backup the top-left part of the bottom right block of Lam
	ptr1 = buffer;
	for(jj=ny; jj<((ny+bs-1)/bs)*bs; jj+=1)
		{
		ptr = &hpLe[N][(jj/bs)*bs*cnf+jj%bs+jj*bs];
		ptr1[0] = ptr[0];
		ptr += 1;
		ptr1 += 1;
		for(ll=jj+1; ll<((ny+bs-1)/bs)*bs; ll+=1)
			{
			ptr1[0] = ptr[0];
			ptr += 1;
			ptr1 += 1;
			}
		}
	//d_print_mat(6, 1, buffer, 1);

	// compute C*U', with U upper cholesky factor of /Pi_p
	dtrmm_nt_u_lib(ny, nx, hpC[N], cnx, hpLp[N]+(nx+nw+pad)*bs, cnl, CL, cnx);
	//d_print_pmat(ny, nx, bs, CL, cnx);

	// compute R + (C*U')*(C*U')' on the top left of Lam
	dsyrk_nt_lib(ny, ny, nx, CL, cnx, CL, cnx, 1, hpR[N], cny, hpLe[N], cnf);
	//d_print_pmat(nz, nz, bs, Lam, cnz);

	// recover overwritten part of I in bottom right part of Lam
	ptr1 = buffer;
	for(jj=ny; jj<((ny+bs-1)/bs)*bs; jj+=1)
		{
		ptr = &hpLe[N][(jj/bs)*bs*cnf+jj%bs+jj*bs];
		ptr[0] = ptr1[0];
		ptr += 1;
		ptr1 += 1;
		for(ll=jj+1; ll<((ny+bs-1)/bs)*bs; ll+=1)
			{
			ptr[0] = ptr1[0];
			ptr += 1;
			ptr1 += 1;
			}
		}
	//d_print_pmat(nz, nz, bs, Lam, cnz);

	// compute C*U'*L'
	dtrmm_nt_l_lib(ny, nx, CL, cnx, hpLp[N]+(nx+nw+pad)*bs, cnl, CLLt, cnx);
	//d_print_pmat(ny, nx, bs, CLLt, cnx);

	// copy C*U'*L' on the bottom left of Lam
	dgetr_lib(ny, nx, 0, CLLt, cnx, ny, hpLe[N]+(ny/bs)*bs*cnf+ny%bs, cnf);
	//d_print_pmat(nz, nz, bs, Lam, cnz);

	// cholesky factorization of Lam
	dpotrf_lib_old(nz, nz, hpLe[N], cnf, hpLe[N], cnf, diag);
	//d_print_pmat(nz, nz, bs, hpLe[N], cnf);
	//d_print_pmat(nz, nz, bs, Lam, cnz);
	//d_print_mat(nz, 1, diag, 1);

	// inverted diagonal of top-left part of hpLe
	for(jj=0; jj<ny; jj++) hpLe[N][(jj/bs)*bs*cnf+jj%bs+jj*bs] = diag[jj];

	// transpose and align /Pi_e
	dtrtr_l_lib(nx, ny, hpLe[N]+(ny/bs)*bs*cnf+ny%bs+ny*bs, cnf, 0, hpLe[N]+ncl*bs, cnf);	
	//d_print_pmat(nz, nz, bs, hpLe[N], cnf);

	//exit(1);

	}
//#endif




//#if defined(TARGET_C99_4X4)
// version tailored for MHE; explicitly computes estimates only at the last stage
void d_ric_trf_mhe_end(int nx, int nw, int ny, int N, double **hpCA, double **hpG, double **hpC, double **hpLp, double **hpQ, double **hpR, double **hpLe, double *work)
	{

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl;

	const int nz = nx+ny;
	const int anz = nal*((nz+nal-1)/nal);
	const int pnx = bs*((nx+bs-1)/bs);
	const int pnw = bs*((nw+bs-1)/bs);
	const int pny = bs*((ny+bs-1)/bs);
	const int pnz = bs*((nz+bs-1)/bs);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int cnw = ncl*((nw+ncl-1)/ncl);
	const int cny = ncl*((ny+ncl-1)/ncl);
	const int cnz = ncl*((nz+ncl-1)/ncl);
	const int cnf = cnz<cnx+ncl ? cnx+ncl : cnz;

	const int pad = (ncl-(nx)%ncl)%ncl; // packing between AGL & P
	const int cnl = cnz<cnx+ncl ? nx+pad+cnx+ncl : nx+pad+cnz;

	int ii, jj, ll;
	double *ptr;

	ptr = work;

	double *Lam_w = ptr; // d_zeros_align(&Lam_w, pnw, cnw);
	ptr += pnw*cnw;
	
	double *GLam_w = ptr; // d_zeros_align(&Lam_w, pnw, cnw);
	ptr += pnx*cnw;
	
	double *GQGt = ptr; //d_zeros_align(&GQGt, pnx, cnx);
	ptr += pnx*cnx;

	double *diag = ptr; // d_zeros_align(&diag, anz, 1);
	ptr += anz;

	double *Pi_p = ptr; //  d_zeros_align(&Pi_p, pnx, cnx);
	ptr += pnx*cnx;

	double *CL = ptr; //d_zeros_align(&CL, pny, cnx);
	ptr += pny*cnx;

	double *CLLt = ptr; //d_zeros_align(&CLLt, pny, cnx);
	ptr += pny*cnx;

	static double buffer[6] = {};

	double *ptr1;

	// loop over horizon
	for(ii=0; ii<N; ii++)
		{

		// zero cross term in bottom left block
		for(jj=(ny/bs)*bs; jj<pnz; jj+=4)
			for(ll=0; ll<bs*ny; ll++)
				hpLp[ii+1][jj*cnl+(nx+pad)*bs+ll] = 0.0;

		// copy R in top left block
		d_copy_pmat_l(ny, bs, hpR[ii], cny, hpLp[ii+1]+(nx+pad)*bs, cnl);
		//d_print_pmat(nz, cnl, bs, hpLp[ii+1], cnl);
		//exit(1);

		// compute lower cholesky factor of Q
		dpotrf_lib_old(nw, nw, hpQ[ii], cnw, Lam_w, cnw, diag);
		//d_print_pmat(nw, nw, bs, Lam_w, cnw);

		// transpose in place the lower cholesky factor of Q
		dtrtr_l_lib(nw, 0, Lam_w, cnw, 0, Lam_w, cnw);	
		//d_print_pmat(nw, nw, bs, Lam_w, cnw);

		// compute G*U', with U' upper cholesky factor of Q
		// d_print_pmat(nx, nw, bs, hpG[ii], cnw);
		dtrmm_nt_u_lib(nx, nw, hpG[ii], cnw, Lam_w, cnw, GLam_w, cnw);
		//d_print_pmat(nx, nw, bs, GLam_w, cnw);

		// compute GQGt
		dsyrk_nt_lib(nx, nx, nw, GLam_w, cnw, GLam_w, cnw, 0, GQGt, cnx, GQGt, cnx);
		//d_print_pmat(nx, nx, bs, GQGt, cnx);
		//d_print_pmat(nx, nx, bs, hpLp[ii+1]+(nx+nw+pad)*bs, cnl);

		// copy GQGt on the bottom right block of Lam
		dtrcp_l_lib(nx, 0, GQGt, cnx, ny, hpLp[ii+1]+(ny/bs)*bs*cnz+ny%bs+(nx+pad+ny)*bs, cnl);
		//d_print_pmat(nz, cnl-1, bs, hpLp[ii+1], cnl);
	
		// compute CA*U', with U upper cholesky factor of /Pi_p
		//d_print_pmat(nz, cnl-1, bs, hpLp[ii], cnl);
		//d_print_pmat(nz, nx, bs, hpCA[ii], cnx);
		dtrmm_nt_u_lib(nz, nx, hpCA[ii], cnx, hpLp[ii]+(nx+pad+ncl)*bs, cnl, hpLp[ii+1], cnl);
		//d_print_pmat(nz, cnl-1, bs, hpLp[ii+1], cnl);

		// compute Lp
		//dsyrk_dpotrf_lib_old(nz, nz, nx, hpLp[ii+1], cnl, hpLp[ii+1]+(nx+pad)*bs, cnl, diag, 1);
		dsyrk_dpotrf_lib_old(nz, nz, nx, hpLp[ii+1], cnl, 1, hpLp[ii+1]+(nx+pad)*bs, cnl, hpLp[ii+1]+(nx+pad)*bs, cnl, diag, 0);

		// inverted diagonal of top-left part of hpLe
		for(jj=0; jj<ny; jj++) hpLp[ii+1][(jj/bs)*bs*cnl+jj%bs+(nx+pad+jj)*bs] = diag[jj];

		// transpose and align Lp
		dtrtr_l_lib(nx, ny, hpLp[ii+1]+(ny/bs)*bs*cnl+ny%bs+(nx+pad+ny)*bs, cnl, 0, hpLp[ii+1]+(nx+pad+ncl)*bs, cnl);	
		//d_print_pmat(nz, cnl-1, bs, hpLp[ii+1], cnl);

		//exit(1);
		}

	//exit(1);

	// stage N

	//d_print_pmat(nx, nx, bs, hpLp[N]+(nx+pad+ncl)*bs, cnl);
	dtrtr_u_lib(nx, 0, hpLp[N]+(nx+pad+ncl)*bs, cnl, 0, GQGt, cnx);
	//d_print_pmat(nx, nx, bs, GQGt, cnx);

	dsyttmm_lu_lib(nx, GQGt, cnx, Pi_p, cnx);
	//d_print_pmat(nx, nx, bs, Pi_p, cnx);

	// copy /Pi_p on the bottom right block of Lam
	//dtrcp_l_lib(nx, 0, Pi_p, cnx, ny, Lam+(ny/bs)*bs*cnz+ny%bs+ny*bs, cnz);
	dtrcp_l_lib(nx, 0, Pi_p, cnx, ny, hpLe[N]+(ny/bs)*bs*cnz+ny%bs+ny*bs, cnf);
	//d_print_pmat(nz, nz, bs, hpLe[N], cnf);

	// backup the top-left part of the bottom right block of Lam
	ptr1 = buffer;
	for(jj=ny; jj<((ny+bs-1)/bs)*bs; jj+=1)
		{
		//ptr = &Lam[(jj/bs)*bs*cnz+jj%bs+jj*bs];
		ptr = &hpLe[N][(jj/bs)*bs*cnf+jj%bs+jj*bs];
		ptr1[0] = ptr[0];
		ptr += 1;
		ptr1 += 1;
		for(ll=jj+1; ll<((ny+bs-1)/bs)*bs; ll+=1)
			{
			ptr1[0] = ptr[0];
			ptr += 1;
			ptr1 += 1;
			}
		}
	//d_print_mat(6, 1, buffer, 1);

	// compute C*U', with U upper cholesky factor of /Pi_p
	dtrmm_nt_u_lib(ny, nx, hpC[N], cnx, hpLp[N]+(nx+pad+ncl)*bs, cnl, CL, cnx);
	//d_print_pmat(ny, nx, bs, CL, cnx);

	// compute R + (C*U')*(C*U')' on the top left of hpLe
	dsyrk_nt_lib(ny, ny, nx, CL, cnx, CL, cnx, 1, hpR[N], cny, hpLe[N], cnf);
	//dsyrk_nt_lib(ny, ny, nx, CL, cnx, CL, cnx, hpR[N], cny, Lam, cnz, 1);
	//d_print_pmat(nz, nz, bs, Lam, cnz);

	// recover overwritten part of I in bottom right part of hpLe
	ptr1 = buffer;
	for(jj=ny; jj<((ny+bs-1)/bs)*bs; jj+=1)
		{
		//ptr = &Lam[(jj/bs)*bs*cnz+jj%bs+jj*bs];
		ptr = &hpLe[N][(jj/bs)*bs*cnf+jj%bs+jj*bs];
		ptr[0] = ptr1[0];
		ptr += 1;
		ptr1 += 1;
		for(ll=jj+1; ll<((ny+bs-1)/bs)*bs; ll+=1)
			{
			ptr[0] = ptr1[0];
			ptr += 1;
			ptr1 += 1;
			}
		}
	//d_print_pmat(nz, nz, bs, Lam, cnz);
	//d_print_pmat(nz, nz, bs, hpLe[N], cnf);

	// compute C*U'*L'
	dtrmm_nt_l_lib(ny, nx, CL, cnx, GQGt, cnx, CLLt, cnx);
	//d_print_pmat(ny, nx, bs, CLLt, cnx);

	// copy C*U'*L' on the bottom left of hpLe
	dgetr_lib(ny, nx, 0, CLLt, cnx, ny, hpLe[N]+(ny/bs)*bs*cnf+ny%bs, cnf);
	//d_print_pmat(nz, nz, bs, Lam, cnz);
	//d_print_pmat(nz, nz, bs, hpLe[N], cnf);

	// cholesky factorization of hpLe
	dpotrf_lib_old(nz, nz, hpLe[N], cnf, hpLe[N], cnf, diag);
	//d_print_pmat(nz, nz, bs, hpLe[N], cnf);
	//d_print_pmat(nz, nz, bs, Lam, cnz);
	//d_print_mat(nz, 1, diag, 1);

	// inverted diagonal of top-left part of hpLe
	for(jj=0; jj<ny; jj++) hpLe[N][(jj/bs)*bs*cnf+jj%bs+jj*bs] = diag[jj];

	// transpose and align /Pi_e
	dtrtr_l_lib(nx, ny, hpLe[N]+(ny/bs)*bs*cnf+ny%bs+ny*bs, cnf, 0, hpLe[N]+ncl*bs, cnf);	
	//d_print_pmat(nz, nz, bs, hpLe[N], cnf);

	//dpotrf_lib_old(nz, nz, Lam, cnz, Lam, cnz, diag);
	//d_print_pmat(nz, nz, bs, Lam, cnz);

	//exit(1);

	}
//#endif




//#if defined(TARGET_C99_4X4)
// version tailored for MHE (test)
void d_ric_trf_mhe_test(int nx, int nw, int ny, int N, double **hpA, double **hpG, double **hpC, double **hpLp, double **hpQ, double **hpR, double **hpLe, double *work)
	{

	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl;

	const int nz = nx+ny;
	const int anz = nal*((nz+nal-1)/nal);
	const int pnx = bs*((nx+bs-1)/bs);
	const int pnw = bs*((nw+bs-1)/bs);
	const int pny = bs*((ny+bs-1)/bs);
	const int pnz = bs*((nz+bs-1)/bs);
	const int cnx = ncl*((nx+ncl-1)/ncl);
	const int cnw = ncl*((nw+ncl-1)/ncl);
	const int cny = ncl*((ny+ncl-1)/ncl);
	const int cnz = ncl*((nz+ncl-1)/ncl);
	const int cnf = cnz<cnx+ncl ? cnx+ncl : cnz;

	const int pad = (ncl-(nx+nw)%ncl)%ncl; // packing between AGL & P
	const int cnl = nx+nw+pad+cnx;

	int ii, jj, ll;
	double *ptr;

	ptr = work;

	double *CL = ptr; //d_zeros_align(&CL, pny, cnx);
	ptr += pny*cnx;

	double *Lam = ptr; // d_zeros_align(&Lam, pnz, cnz);
	ptr += pnz*cnz;

	double *diag = ptr; // d_zeros_align(&diag, anz, 1);
	ptr += anz;

	double *Fam = ptr; // d_zeros_align(&Fam, pnz, cnf);
	ptr += pnz*cnf;

	double *Lam_w = ptr; // d_zeros_align(&Lam_w, pnw, cnw);
	ptr += pnw*cnw;
	
	// initialize bottom right part of Lam with identity
	for(ii=0; ii<pnz*cnz; ii++)
		Lam[ii] = 0.0;
	for(ii=0; ii<nx; ii++)
		Lam[((ny+ii)/bs)*bs*cnz+(ny+ii)%bs+(ny+ii)*bs] = 1.0;
	// d_print_pmat(nz, nz, bs, Lam, cnz);

	// loop over horizon
	for(ii=0; ii<N; ii++)
		{

		// compute C*U', with U upper cholesky factor of /Pi_p
		dtrmm_nt_u_lib(ny, nx, hpC[ii], cnx, hpLp[ii]+(nx+nw+pad)*bs, cnl, CL, cnx);
		//d_print_pmat(ny, nx, bs, CL, cnx);

		// compute R + (C*U')*(C*U')' on the top left of Lam
		dsyrk_nt_lib(ny, ny, nx, CL, cnx, CL, cnx, 1, hpR[ii], cny, Lam, cnz);
		//d_print_pmat(nz, nz, bs, Lam, cnz);

		// copy C*U' on the bottom left of Lam
		dgetr_lib(ny, nx, 0, CL, cnx, ny, Lam+(ny/bs)*bs*cnz+ny%bs, cnz);
		//d_print_pmat(nz, nz, bs, Lam, cnz);

		// recover overwritten part of I in bottom right part of Lam
		for(jj=ny; jj<((ny+bs-1)/bs)*bs; jj+=1)
			{
			ptr = &Lam[(jj/bs)*bs*cnz+jj%bs+jj*bs];
			*ptr = 1.0;
			ptr += 1;
			for(ll=jj+1; ll<((ny+bs-1)/bs)*bs; ll+=1)
				{
				*ptr = 0.0;
				ptr += 1;
				}
			}
		// d_print_pmat(nz, nz, bs, Lam, cnz);

		// cholesky factorization of Lam
		dpotrf_lib_old(nz, nz, Lam, cnz, Fam, cnf, diag);
		//d_print_pmat(nz, nz, bs, Fam, cnf);
		//d_print_pmat(nz, nz, bs, Lam, cnz);
		//d_print_mat(nz, 1, diag, 1);

		// transpose and align the bottom right part of Lam
		dtrtr_l_lib(nx, ny, Fam+(ny/bs)*bs*cnf+ny%bs+ny*bs, cnf, 0, Fam+ncl*bs, cnf);	
		//d_print_pmat(nz, nz, bs, Fam, cnf);

		// compute upper cholesky factor of /Pi_e using triangular-triangular matrix multiplication
		// d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii], cnl);
		dttmm_uu_lib(nx, Fam+ncl*bs, cnf, hpLp[ii]+(nx+nw+pad)*bs, cnl, hpLe[ii], cnx);
		//d_print_pmat(nx, nx, bs, hpLe[ii], cnx);

		// compute A*U', with U' upper cholesky factor of /Pi_e
		// d_print_pmat(nx, nx, bs, hpA[ii], cnx);
		dtrmm_nt_u_lib(nx, nx, hpA[ii], cnx, hpLe[ii], cnx, hpLp[ii+1], cnl);
		//d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii+1], cnl);

		// compute lower cholesky factor of Q
		dpotrf_lib_old(nw, nw, hpQ[ii], cnw, Lam_w, cnw, diag);
		//d_print_pmat(nw, nw, bs, Lam_w, cnw);

		// transpose in place the lower cholesky factor of Q
		dtrtr_l_lib(nw, 0, Lam_w, cnw, 0, Lam_w, cnw);	
		//d_print_pmat(nw, nw, bs, Lam_w, cnw);

		// compute G*U', with U' upper cholesky factor of Q
		// d_print_pmat(nx, nw, bs, hpG[ii], cnw);
		dtrmm_nt_u_lib(nx, nw, hpG[ii], cnw, Lam_w, cnw, hpLp[ii+1]+nx*bs, cnl);
		//d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii+1], cnl);

		// compute /Pi_p and factorize it
		//dsyrk_dpotrf_lib_old(nx, nx, nx+nw, hpLp[ii+1], cnl, hpLp[ii+1], cnl, diag, 0);
		dsyrk_dpotrf_lib_old(nx, nx, nx+nw, hpLp[ii+1], cnl, 0, hpLp[ii+1], cnl, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, diag, 0);
		//d_print_pmat(nx, nx+nw+pad+nx, bs, hpLp[ii+1], cnl);

		// transpose in place the lower cholesky factor of /Pi_p
		dtrtr_l_lib(nx, 0, hpLp[ii+1]+(nx+nw+pad)*bs, cnl, 0, hpLp[ii+1]+(nx+nw+pad)*bs, cnl);	
		//d_print_pmat(nx, cnl, bs, hpLp[ii+1], cnl);

		//exit(1);
		}

	//exit(1);

	}
//#endif





