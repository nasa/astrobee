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

#include "../include/aux_d.h"
#include "../include/blas_d.h"
#include "../include/lqcp_aux.h"
#include "../include/block_size.h"



void d_cond_Gamma_u_T(int N, int nx, int nu, int free_x0, double **pA, double **pBt, double **pGamma_u)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;

	int ii, offset;

	if(free_x0) // MHE
		{
		dgetr_lib(nx, nx, 0, pA[0], cnx, 0, pGamma_u[0], cnx);
		offset = nx;
		dgecp_lib(nu, nx, 0, pBt[0], cnx, offset, pGamma_u[0]+offset/bs*bs*cnx+offset%bs, cnx);
		for(ii=1; ii<N; ii++)
			{
			offset = nx + ii*nu;
			dgemm_nt_lib(nx+ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
			dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset, pGamma_u[ii]+offset/bs*bs*cnx+offset%bs, cnx);
			}
		}
	else // MPC
		{
		dgecp_lib(nu, nx, 0, pBt[0], cnx, 0, pGamma_u[0], cnx);
		for(ii=1; ii<N; ii++)
			{
			offset = ii*nu;
#if 1
			dgemm_nt_lib(nx, ii*nu, nx, pA[ii], cnx, pGamma_u[ii-1], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
			dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
			dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset, pGamma_u[ii]+offset/bs*bs*cnx+offset%bs, cnx);
			}
		}

	}



void d_cond_R_N3_nx2(int N, int nx, int nu, int free_x0, double **pAt, double **pBt, int diag_hessian, int nzero_Q_N, double **pQ, double **pS, double **pR, double *pL, double *dL, double **pGamma_u, double **pGamma_u_Q, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	if(free_x0) // MHE
		{

		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// Gamma_u * Q
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib(nx+(ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}
			if(nzero_Q_N==0)
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);

			dgeset_lib(nx+N*nu, nx+N*nu, 0.0, 0, pH_R, cnxNnu);

			// dR
			ddiain_lib(nx, pQ[0], 0, pH_R, cnxNnu);
			for(ii=0; ii<N; ii++)
				{
				offset = nx+ii*nu;
				ddiain_lib(nu, pR[ii], offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				}

			for(ii=0; ii<N1; ii++)
				dsyrk_nt_lib(nx+(N1-ii)*nu, nx+(N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u[N1-1-ii], cnx, 1, pH_R, cnxNnu, pH_R, cnxNnu); 

			}
		else
			{

			// Gamma_u * Q
			for(ii=0; ii<N1; ii++)
				{
#if 1
				dpotrf_lib(nx, nx, pQ[ii+1], cnx, pL, cnx, dL);
				dtrmm_nt_u_lib(nx+(ii+1)*nu, nx, pGamma_u[ii], cnx, pL, cnx, pGamma_u_Q[ii], cnx);
#else
				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
				}
			if(nzero_Q_N==0)
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);

			// Gamma_u * bar_S
			dgetr_lib(nu, nx, 0, pS[0], cnx, 0, pH_R, cnxNnu);
			for(ii=1; ii<N; ii++)
				{
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pS[ii], cnx, 0, pH_R+ii*nu*bs, cnxNnu, pH_R+ii*nu*bs, cnxNnu, 0, 0);
				}

			// transpose H in the lower triangular
			dtrtr_u_lib(nx+N*nu, 0, pH_R, cnxNnu, 0, pH_R, cnxNnu);

			// R
			dgecp_lib(nx, nx, 0, pQ[0], cnx, 0, pH_R, cnxNnu);
			for(ii=0; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nu, 0, pR[ii], cnu, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				}

#if 1
				for(ii=0; ii<N1; ii++)
					dsyrk_nt_lib(nx+(N1-ii)*nu, nx+(N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u_Q[N1-1-ii], cnx, 1, pH_R, cnxNnu, pH_R, cnxNnu); 
#else
				for(ii=0; ii<N1; ii++)
					dsyrk_nt_lib(nx+(N1-ii)*nu, nx+(N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u[N1-1-ii], cnx, 1, pH_R, cnxNnu, pH_R, cnxNnu); 
#endif

			}

		}
	else // MPC
		{

		int cNnu = (N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// Gamma_u * Q
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib((ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}

			if(nzero_Q_N==0)
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);

			dgeset_lib(N*nu, N*nu, 0.0, 0, pH_R, cNnu);

			// dR
			for(ii=0; ii<N; ii++)
				{
				ddiain_lib(nu, pR[ii], ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			for(ii=0; ii<N1; ii++)
				dsyrk_nt_lib((N1-ii)*nu, (N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u[N1-1-ii], cnx, 1, pH_R, cNnu, pH_R, cNnu); 

			}
		else
			{

			// Gamma_u * Q
			for(ii=0; ii<N1; ii++)
				{
#if 1
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pQ[ii+1], cnx, pGamma_u[ii], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
				}

			if(nzero_Q_N==0)
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);

			// Gamma_u * bar_S
			for(ii=1; ii<N; ii++)
				{
				dgemm_nt_lib(ii*nu, nu, nx, pGamma_u[ii-1], cnx, pS[ii], cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);
				}

			// transpose H in the lower triangular
			dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nu, 0, pR[ii], cnu, ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			for(ii=0; ii<N1; ii++)
				dsyrk_nt_lib((N1-ii)*nu, (N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u[N1-1-ii], cnx, 1, pH_R, cNnu, pH_R, cNnu); 

			}

		}

	return;

	}



void d_cond_R_N2_nx2(int N, int nx, int nu, int free_x0, double **pAt, double **pBt, int diag_hessian, int nzero_Q_N, double **pQ, double **pS, double **pR, double *pD, double *pM, double *pP, double **pGamma_u, double **pGamma_u_Q, double **pGamma_u_Q_A, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	if(free_x0) // MHE
		{

		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;

		// Gamma_u * Q
		if(diag_hessian)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib(nx+(ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}
			
			// zero S
			for(ii=1; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgeset_lib(nu, nx, 0.0, offset, pGamma_u_Q[ii-1]+offset/bs*bs*cnx+offset%bs, cnx);
				}

			// Gamma_u_Q * bar_A
			dgecp_lib(nx+N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
				}

			// Gamma_u * M
			dgemm_nt_lib(nx, nu, nx, pGamma_u_Q_A[0], cnx, pBt[0], cnx, 0, pH_R+nx*bs, cnxNnu, pH_R+nx*bs, cnxNnu, 0, 0);
			for(ii=1; ii<N1; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii-1]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);
				}

			// R
			dgemm_nt_lib(nx, nx, nx, pGamma_u_Q_A[0], cnx, pAt[0], cnx, 0, pH_R, cnxNnu, pH_R, cnxNnu, 0, 0);
			ddiaad_lib(nx, 1.0, pQ[ii], 0, pH_R, cnxNnu);
			for(ii=0; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 0, pD, cnu, pD, cnu, 0, 0);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				dgecp_lib(nu, nu, 0, pD, cnu, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				}

			}
		else
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}

			// copy S
			for(ii=1; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, 0, pS[ii], cnx, offset, pGamma_u_Q[ii-1]+offset/bs*bs*cnx+offset%bs, cnx);
				}

//				if(nzero_Q_N==0)
//					dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
			
			// Gamma_u_Q * bar_A^{-1}
			dgecp_lib(nx+N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
				}

			// Gamma_u * M
			dgemm_nt_lib(nx, nu, nx, pGamma_u_Q_A[0], cnx, pBt[0], cnx, 1, pS[0], cnx, pH_R+nx*bs, cnxNnu, 1, 0);
			for(ii=1; ii<N1; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii-1]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);
				}

			// R
			dgemm_nt_lib(nx, nx, nx, pGamma_u_Q_A[0], cnx, pAt[0], cnx, 1, pQ[0], cnx, pH_R, cnxNnu, 0, 0);
			for(ii=0; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 1, pR[ii], cnu, pD, cnu, 0, 0);
				dgecp_lib(nu, nu, 0, pD, cnu, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				}

			}

		// transpose H in the lower triangular
		dtrtr_u_lib(nx+N*nu, 0, pH_R, cnxNnu, 0, pH_R, cnxNnu);

		}
	else
		{

		int cNnu = (N*nu+ncl-1)/ncl*ncl;

		// Gamma_u * Q
		if(diag_hessian)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib((ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}
			
			// zero S
			for(ii=1; ii<N; ii++)
				{
				dgeset_lib(nu, nx, 0.0, (ii)*nu, pGamma_u_Q[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				}

			// Gamma_u_Q * bar_A
			dgecp_lib(N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
#if 1
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_u_Q_A[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
#endif
				}

			// Gamma_u * M
			for(ii=1; ii<N1; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);
				}

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 0, pD, cnu, pD, cnu, 0, 0);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				dgecp_lib(nu, nu, 0, pD, cnu, ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			}
		else
			{
			for(ii=0; ii<N1; ii++)
				{
#if 1
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pQ[ii+1], cnx, pGamma_u[ii], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}

			// copy S
			for(ii=1; ii<N; ii++)
				{
				dgecp_lib(nu, nx, 0, pS[ii], cnx, (ii)*nu, pGamma_u_Q[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				}

	//			if(nzero_Q_N==0)
	//				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
			
			// Gamma_u_Q * bar_A^{-1}
			dgecp_lib(N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
#if 1
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_u_Q_A[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
#endif
				}

			// Gamma_u * M
			for(ii=1; ii<N1; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);
				}

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 1, pR[ii], cnu, pD, cnu, 0, 0);
				dgecp_lib(nu, nu, 0, pD, cnu, ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			}

		// transpose H in the lower triangular
		dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

		}

	return;

	
	}



void d_cond_R_N2_nx3(int N, int nx, int nu, int free_x0, double **pBAt, int diag_hessian, int nzero_Q_N, double **pRSQ, double *pD, double *pM, double *pP, double *pLam, double *diag, double *pBAtL, double **pGamma_u, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	int nz = nx+nu;
	int cnz = (nz+ncl-1)/ncl*ncl;

	if(free_x0) // MHE
		{

		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					offset = nx;
					dgeset_lib(nx+nu, nx+nu, 0.0, 0, pH_R, cnxNnu);
					ddiaad_lib(nu, 1.0, pRSQ[0], offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					ddiaad_lib(nx, 1.0, pRSQ[0]+nu, 0, pH_R, cnxNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset = nx + ii*nu;
					dgeset_lib(nx+(ii+1)*nu, nu, 0.0, 0, pH_R+offset*bs, cnxNnu);
					ddiaad_lib(nu, 1.0, pRSQ[ii], offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);
					ddiain_sqrt_lib(nx, pRSQ[ii]+nu, 0, pP, cnx);

					ii--;

					}

				}
			else
				{

				if(N==1)
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQ[1][ii]);

					// first stage 
					dgemm_diag_right_lib(nz, nx, pBAt[0], cnx, pP, 0, pBAtL, cnx, pBAtL, cnx);
					dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
					ddiaad_lib(nz, 1.0, pRSQ[0], 0, pLam, cnz);

					offset = nx;
					dtrtr_l_lib(nu, 0, pLam, cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_R+offset*bs, cnxNnu);
					dtrcp_l_lib(nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_R, cnxNnu);

					// transpose H in the lower triangular
					dgetr_lib(nx, N*nu, 0, pH_R+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs, cnxNnu);
					dtrtr_u_lib(N*nu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu);

					return;

					}
				else
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQ[N][ii]);

					// second final stage 
					ii = N-1;
					dgemm_diag_right_lib(nz, nx, pBAt[ii], cnx, pP, 0, pBAtL, cnx, pBAtL, cnx);
					dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
					ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pLam, cnz);

					offset = nx + ii*nu;
					dtrtr_l_lib(nu, 0, pLam, cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
					dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);

					dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
					dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}

				}

			// middle stages 
			for(; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pP, cnx, pBAtL, cnx);
				dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
				ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pLam, cnz);

				offset = nx + ii*nu;
				dtrtr_l_lib(nu, 0, pLam, cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
				dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nz, nx, pBAt[0], cnx, pP, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nz, 1.0, pRSQ[0], 0, pLam, cnz);

			offset = nx;
			dtrtr_l_lib(nu, 0, pLam, cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
			dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_R+offset*bs, cnxNnu);
			dtrcp_l_lib(nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_R, cnxNnu);

			// transpose H in the lower triangular
			dgetr_lib(nx, N*nu, 0, pH_R+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs, cnxNnu);
			dtrtr_u_lib(N*nu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu);

			return;

			}
		else // dense Hessian
			{

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					offset = nx;
					dtrcp_l_lib(nx, nu, pRSQ[0]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_R, cnxNnu);
					dtrcp_l_lib(nu, 0, pRSQ[0], cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgetr_lib(nx, nu, nu, pRSQ[0]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs, cnxNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset = nx + ii*nu;
					dtrtr_l_lib(nu, 0, pRSQ[ii], cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgetr_lib(nx, nu, nu, pRSQ[ii]+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
					dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);

					dgecp_lib(nx, nx, nu, pRSQ[ii]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
					dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}

				}
			else
				{

				// final stage 
				dpotrf_lib(nx, nx, pRSQ[N], cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				ii = N-1;

				}

			// middle stages 
			for(; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pP, cnx, pBAtL, cnx);
				dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii], cnz, pLam, cnz);

				offset = nx + ii*nu;
				dtrtr_l_lib(nu, 0, pLam, cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
				dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nz, nx, pBAt[0], cnx, pP, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[0], cnz, pLam, cnz);

			offset = nx;
			dtrtr_l_lib(nu, 0, pLam, cnz, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
			dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_R+offset*bs, cnxNnu);
			dtrcp_l_lib(nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_R, cnxNnu);

			// transpose H in the lower triangular
			dgetr_lib(nx, N*nu, 0, pH_R+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs, cnxNnu);
			dtrtr_u_lib(N*nu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu);

			return;

			}

		}
	else // fixed x0: MPC
		{

		int cNnu = (N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					dgeset_lib(nx+nu, nx+nu, 0.0, 0, pH_R, cNnu);
					ddiaad_lib(nu, 1.0, pRSQ[0], 0, pH_R, cNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset = ii*nu;
					dgeset_lib((ii+1)*nu, nu, 0.0, 0, pH_R+offset*bs, cNnu);
					ddiaad_lib(nu, 1.0, pRSQ[ii], offset, pH_R+offset/bs*bs*cNnu+offset%bs+offset*bs, cNnu);
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);
					ddiain_sqrt_lib(nx, pRSQ[ii]+nu, 0, pP, cnx);

					ii--;

					}

				}
			else
				{

				if(N==1)
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQ[1][ii]);

					// first stage 
					dgemm_diag_right_lib(nu, nx, pBAt[0], cnx, pP, 0, pBAtL, cnx, pBAtL, cnx);
					dsyrk_nt_lib(nu, nu, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
					ddiaad_lib(nu, 1.0, pRSQ[0], 0, pLam, cnz);
					dtrtr_l_lib(nu, 0, pLam, cnz, 0, pH_R, cNnu);

					// transpose H in the lower triangular
					dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

					return;

					}
				else
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQ[N][ii]);

					// second final stage 
					ii = N-1;
					dgemm_diag_right_lib(nz, nx, pBAt[ii], cnx, pP, 0, pBAtL, cnx, pBAtL, cnx);
					dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
					ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pLam, cnz);
					dtrtr_l_lib(nu, 0, pLam, cnz, (ii)*nu, pH_R+((ii)*nu)/bs*bs*cNnu+((ii)*nu)%bs+(ii)*nu*bs, cNnu);
					dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
					dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);

					dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
					dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}

				}

			// middle stages 
			for(; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pP, cnx, pBAtL, cnx);
				dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
				ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pLam, cnz);
				dtrtr_l_lib(nu, 0, pLam, cnz, (ii)*nu, pH_R+((ii)*nu)/bs*bs*cNnu+((ii)*nu)%bs+(ii)*nu*bs, cNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
				dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAt[0], cnx, pP, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nu, nu, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nu, 1.0, pRSQ[0], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pH_R, cNnu);

			// transpose H in the lower triangular
			dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

			return;

			}
		else // dense hessian
			{

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					dtrcp_l_lib(nu, 0, pRSQ[0], cnu, 0, pH_R, cNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset = ii*nu;
					dtrtr_l_lib(nu, 0, pRSQ[ii], cnz, offset, pH_R+offset/bs*bs*cNnu+offset%bs+offset*bs, cNnu);
					dgetr_lib(nx, nu, nu, pRSQ[ii]+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
					dgemm_nt_lib(ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cNnu, pH_R+offset*bs, cNnu, 0, 0);

					dgecp_lib(nx, nx, nu, pRSQ[ii]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
					dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}

				}
			else
				{

				// final stage 
				dpotrf_lib(nx, nx, pRSQ[N], cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				ii = N-1;

				}

			// middle stages 
			for(; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pP, cnx, pBAtL, cnx);
				dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii], cnz, pLam, cnz);
				dtrtr_l_lib(nu, 0, pLam, cnz, (ii)*nu, pH_R+((ii)*nu)/bs*bs*cNnu+((ii)*nu)%bs+(ii)*nu*bs, cNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
				dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAt[0], cnx, pP, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nu, nu, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[0], cnu, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pH_R, cNnu);

			// transpose H in the lower triangular
			dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

			return;

			}

		}

	return;

	}



void d_cond_R_N2_nx2_permute(int N, int nx, int nu, int free_x0, double **pAt, double **pBt, int diag_hessian, int nzero_Q_N, double **pQ, double **pS, double **pR, double *pD, double *pM, double *pP, double *pGamma_L, double **pGamma_u, double **pGamma_u_Q, double **pGamma_u_Q_A, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	if(free_x0) // MHE
		{

		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;

		// Gamma_u * Q
		if(diag_hessian)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib(nx+(ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}
			
			// zero S
			for(ii=1; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgeset_lib(nu, nx, 0.0, offset, pGamma_u_Q[ii-1]+offset/bs*bs*cnx+offset%bs, cnx);
				}

			// Gamma_u_Q * bar_A
			dgecp_lib(nx+N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
				}

			// Gamma_u * M
			dgemm_nt_lib(nx, nu, nx, pGamma_u_Q_A[0], cnx, pBt[0], cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
			dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1)*nu*bs, cnxNnu);
			for(ii=1; ii<N1; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii-1]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1-ii)*nu*bs, cnxNnu);
				for(jj=0; jj<ii; jj++)
					dgecp_lib(nu, nu, nx+jj*nu, pGamma_L+(nx+jj*nu)/bs*bs*cnu+(nx+jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+(N-1-jj)*nu/bs*bs*cnxNnu+(N-1-jj)*nu%bs+(N-1-ii)*nu*bs, cnxNnu);
				}

			// R
			dgemm_nt_lib(nx, nx, nx, pGamma_u_Q_A[0], cnx, pAt[0], cnx, 0, pP, cnx, pP, cnx, 0, 0);
			ddiaad_lib(nx, 1.0, pQ[ii], 0, pP, cnx);
			offset = N*nu;
			dtrcp_l_lib(nx, 0, pP, cnx, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
			for(ii=0; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 0, pD, cnu, pD, cnu, 0, 0);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				offset = (N-1-ii)*nu;
				dtrcp_l_lib(nu, 0, pD, cnu, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				}

			}
		else // dense hessian
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(nx+N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}

			// copy S
			for(ii=1; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, 0, pS[ii], cnx, offset, pGamma_u_Q[ii-1]+offset/bs*bs*cnx+offset%bs, cnx);
				}

//				if(nzero_Q_N==0)
//					dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
			
			// Gamma_u_Q * bar_A^{-1}
			dgecp_lib(nx+N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
				}

			// Gamma_u * M
			dgemm_nt_lib(nx, nu, nx, pGamma_u_Q_A[0], cnx, pBt[0], cnx, 1, pS[0], cnx, pGamma_L, cnu, 1, 0);
			dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1)*nu*bs, cnxNnu);
			for(ii=1; ii<N1; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii-1]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1-ii)*nu*bs, cnxNnu);
				for(jj=0; jj<ii; jj++)
					dgecp_lib(nu, nu, nx+jj*nu, pGamma_L+(nx+jj*nu)/bs*bs*cnu+(nx+jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+(N-1-jj)*nu/bs*bs*cnxNnu+(N-1-jj)*nu%bs+(N-1-ii)*nu*bs, cnxNnu);
				}

			// R
			dgemm_nt_lib(nx, nx, nx, pGamma_u_Q_A[0], cnx, pAt[0], cnx, 1, pQ[0], cnx, pP, cnx, 0, 0);
			offset = N*nu;
			dtrcp_l_lib(nx, 0, pP, cnx, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
			for(ii=0; ii<N; ii++)
				{
				offset = nx + ii*nu;
				dgecp_lib(nu, nx, offset, pGamma_u_Q_A[ii]+offset/bs*bs*cnx+offset%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 1, pR[ii], cnu, pD, cnu, 0, 0);
				offset = (N-1-ii)*nu;
				dtrcp_l_lib(nu, 0, pD, cnu, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				}

			}

		}
	else
		{

		int cNnu = (N*nu+ncl-1)/ncl*ncl;

		// Gamma_u * Q
		if(diag_hessian)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib((ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}
			
			// zero S
			for(ii=1; ii<N; ii++)
				{
				dgeset_lib(nu, nx, 0.0, (ii)*nu, pGamma_u_Q[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				}

			// Gamma_u_Q * bar_A
			dgecp_lib(N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
#if 1
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_u_Q_A[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
#endif
				}

			// Gamma_u * M
			for(ii=1; ii<N1; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					dgecp_lib(nu, nu, jj*nu, pGamma_L+jj*nu/bs*bs*cnu+jj*nu%bs, cnu, (N-1-jj)*nu, pH_R+(N-1-jj)*nu/bs*bs*cNnu+(N-1-jj)*nu%bs+(N-1-ii)*nu*bs, cNnu);
				}

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 0, pD, cnu, pD, cnu, 0, 0);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				dtrcp_l_lib(nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
				}

			}
		else // dense hessian
			{
			for(ii=0; ii<N1; ii++)
				{
#if 1
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pQ[ii+1], cnx, pGamma_u[ii], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}

			// copy S
			for(ii=1; ii<N; ii++)
				{
				dgecp_lib(nu, nx, 0, pS[ii], cnx, (ii)*nu, pGamma_u_Q[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				}

	//			if(nzero_Q_N==0)
	//				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
			
			// Gamma_u_Q * bar_A^{-1}
			dgecp_lib(N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
#if 1
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_u_Q_A[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
#endif
				}

			// Gamma_u * M
			for(ii=1; ii<N1; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					dgecp_lib(nu, nu, jj*nu, pGamma_L+jj*nu/bs*bs*cnu+jj*nu%bs, cnu, (N-1-jj)*nu, pH_R+(N-1-jj)*nu/bs*bs*cNnu+(N-1-jj)*nu%bs+(N-1-ii)*nu*bs, cNnu);
				}

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 1, pR[ii], cnu, pD, cnu, 0, 0);
				dtrcp_l_lib(nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
				}

			}

		// transpose H in the lower triangular
//		dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

		}

	return;

	
	}



void d_cond_fact_R_N2_nx2_permute(int N, int nx, int nu, int free_x0, double **pAt, double **pBt, int diag_hessian, double **pQ, double **pS, double **pR, double *pD, double *pM, double *pQs, double *diag, double *pGamma_L, double **pGamma_u, double **pGamma_w, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int pnu = (nu+bs-1)/bs*bs;
	int pnx = (nx+bs-1)/bs*bs;
	int cnu = (nu+ncl-1)/ncl*ncl;

	int ii, jj, offset, i_temp;

	if(free_x0) // MHE
		{

		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// last stage
			d_copy_mat(nx, 1, pQ[N], nx, pQs, nx);
			dgemm_diag_right_lib(nx+N*nu, nx, pGamma_u[N-1], cnx, pQs, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx);

			// middle stages
			for(ii=N-1; ii>0; ii--)
				{

				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_w[ii], cnx, pAt[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);

				dgecp_lib(nu, nx, nx+ii*nu, pGamma_w[ii]+(nx+ii*nu)/bs*bs*cnx+(nx+ii*nu)%bs, cnx, 0, pM, cnx);
				dsyrk_nt_lib(nu, nu, nx, pBt[ii], cnx, pM, cnx, 0, pD, cnu, pD, cnu);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				dgetr_lib(nu, nx, nx+ii*nu, pGamma_w[ii-1]+(nx+ii*nu)/bs*bs*cnx+(nx+ii*nu)%bs, cnx, 0, pD+pnu*cnu, cnu);

				dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag); // TODO copy last part in the hole
				dtrcp_l_lib(nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cnxNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
			
				dgemm_nn_lib(nx+(ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, nx+jj*nu, pGamma_L+(nx+jj*nu)/bs*bs*cnu+(nx+jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cnxNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
					}
				dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1-ii)*nu*bs, cnxNnu);

				dgeset_lib(nx, nx, 0.0, 0, pQs, cnx);
				ddiain_lib(nx, pQ[ii], 0, pQs, cnx);
				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx);
				dtrtr_l_lib(nx, 0, pQs, cnx, 0, pQs, cnx);	
				dgemm_nt_lib(nx+ii*nu, nx, nx, pGamma_u[ii-1], cnx, pQs, cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);

				}

			dgecp_lib(nu, nx, nx, pGamma_w[0]+nx/bs*bs*cnx+nx%bs, cnx, 0, pM, cnx);
			dsyrk_nt_lib(nu, nu, nx, pBt[0], cnx, pM, cnx, 0, pD, cnu, pD, cnu);
			ddiaad_lib(nu, 1.0, pR[0], 0, pD, cnu);
			dgemm_nt_lib(nx, nu, nx, pAt[0], cnx, pM, cnx, 0, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, 0, 0);

			dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag);
			dtrcp_l_lib(nu, 0, pD, cnu, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cnxNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cnxNnu);
			dgecp_lib(nx, nu, 0, pD+pnu*cnu, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1)*nu*bs, cnxNnu);

			dsyrk_nt_lib(nx, nx, nx, pGamma_w[0], cnx, pAt[0], cnx, 0, pQs, cnx, pQs, cnx);
			ddiaad_lib(nx, 1.0, pQ[0], 0, pQs, cnx);
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
			dsyrk_dpotrf_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx, diag);
#else
			dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx);
			dpotrf_lib(nx, nx, pQs, cnx, pQs, cnx, diag);
#endif
			dtrcp_l_lib(nx, 0, pQs, cnx, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+N*nu*bs, cnxNnu);	

			}
		else // dense hessian
			{

			// last stage
			dgecp_lib(nx, nx, 0, pQ[N], cnx, 0, pQs, cnx);
			dgemm_nt_lib(nx+N*nu, nx, nx, pGamma_u[N-1], cnx, pQs, cnx, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx, 0, 0);

			// middle stages
			for(ii=N-1; ii>0; ii--)
				{

				dgemm_nt_lib(nx+(ii+1)*nu, nx, nx, pGamma_w[ii], cnx, pAt[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
				dgead_lib(nu, nx, 1.0, 0, pS[ii], cnx, nx+ii*nu, pGamma_w[ii-1]+(nx+ii*nu)/bs*bs*cnx+(nx+ii*nu)%bs, cnx);

				dgecp_lib(nu, nx, nx+ii*nu, pGamma_w[ii]+(nx+ii*nu)/bs*bs*cnx+(nx+ii*nu)%bs, cnx, 0, pM, cnx);
				dsyrk_nt_lib(nu, nu, nx, pBt[ii], cnx, pM, cnx, 1, pR[ii], cnu, pD, cnu);
				dgetr_lib(nu, nx, nx+ii*nu, pGamma_w[ii-1]+(nx+ii*nu)/bs*bs*cnx+(nx+ii*nu)%bs, cnx, 0, pD+pnu*cnu, cnu);

				dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag); // TODO copy last part in the hole
				dtrcp_l_lib(nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cnxNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
			
				dgemm_nn_lib(nx+(ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, nx+jj*nu, pGamma_L+(nx+jj*nu)/bs*bs*cnu+(nx+jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cnxNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
					}
				dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1-ii)*nu*bs, cnxNnu);

				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQ[ii], cnx, pQs, cnx);
				dtrtr_l_lib(nx, 0, pQs, cnx, 0, pQs, cnx);	
				dgemm_nt_lib(nx+ii*nu, nx, nx, pGamma_u[ii-1], cnx, pQs, cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);

				}

			dgecp_lib(nu, nx, nx, pGamma_w[0]+nx/bs*bs*cnx+nx%bs, cnx, 0, pM, cnx);
			dsyrk_nt_lib(nu, nu, nx, pBt[0], cnx, pM, cnx, 1, pR[0], cnu, pD, cnu);
			dgemm_nt_lib(nx, nu, nx, pAt[0], cnx, pM, cnx, 0, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, 0, 0);

			dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag);
			dtrcp_l_lib(nu, 0, pD, cnu, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cnxNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cnxNnu);
			dgecp_lib(nx, nu, 0, pD+pnu*cnu, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1)*nu*bs, cnxNnu);

			dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQ[0], cnx, pQs, cnx);
			dsyrk_dpotrf_lib(nx, nx, nx, pGamma_w[0], cnx, pAt[0], cnx, 1, pQs, cnx, pQs, cnx, diag);
			dtrcp_l_lib(nx, 0, pQs, cnx, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+N*nu*bs, cnxNnu);	

			}

		}
	else // MPC
		{

		int cNnu = (N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// last stage
			d_copy_mat(nx, 1, pQ[N], nx, pQs, nx);
			dgemm_diag_right_lib(N*nu, nx, pGamma_u[N-1], cnx, pQs, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx);

			// middle stages
			for(ii=N-1; ii>0; ii--)
				{

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_w[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_w[ii], cnx, pAt[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif

				dgecp_lib(nu, nx, ii*nu, pGamma_w[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dsyrk_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 0, pD, cnu, pD, cnu);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				dgetr_lib(nu, nx, ii*nu, pGamma_w[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pD+pnu*cnu, cnu);

				dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag); // TODO copy last part in the hole
				dtrcp_l_lib(nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
			
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pGamma_L+(jj*nu)/bs*bs*cnu+(jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dgeset_lib(nx, nx, 0.0, 0, pQs, cnx);
				ddiain_lib(nx, pQ[ii], 0, pQs, cnx);
				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx);
				dtrtr_l_lib(nx, 0, pQs, cnx, 0, pQs, cnx);	
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, ii*nu, nx, pQs, cnx, pGamma_u[ii-1], cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pQs, cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif

				}

			dgecp_lib(nu, nx, 0, pGamma_w[0], cnx, 0, pM, cnx);
			dsyrk_nt_lib(nu, nu, nx, pM, cnx, pBt[0], cnx, 0, pD, cnu, pD, cnu);
			ddiaad_lib(nu, 1.0, pR[0], 0, pD, cnu);
			dpotrf_lib(nu, nu, pD, cnu, pD, cnu, diag);
			dtrcp_l_lib(nu, 0, pD, cnu, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}
		else // dense hessian
			{

			// last stage
			dgecp_lib(nx, nx, 0, pQ[N], cnx, 0, pQs, cnx);
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
			dgemm_nt_lib(nx, N*nu, nx, pQs, cnx, pGamma_u[N-1], cnx, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx, 0, 1);
#else
			dgemm_nt_lib(N*nu, nx, nx, pGamma_u[N-1], cnx, pQs, cnx, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx, 0, 0);
#endif

			// middle stages
			for(ii=N-1; ii>0; ii--)
				{

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_w[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_w[ii], cnx, pAt[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif
				//dgecp_lib(nu, nx, 0, pS[ii], cnx, (ii)*nu, pGamma_w[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				dgead_lib(nu, nx, 1.0, 0, pS[ii], cnx, (ii)*nu, pGamma_w[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);

				dgecp_lib(nu, nx, ii*nu, pGamma_w[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dsyrk_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 1, pR[ii], cnu, pD, cnu);
				dgetr_lib(nu, nx, ii*nu, pGamma_w[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pD+pnu*cnu, cnu);

				dpotrf_lib(pnu+nx, nu, pD, cnu, pD, cnu, diag); // TODO copy last part in the hole
				dtrcp_l_lib(nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
			
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pGamma_L+(jj*nu)/bs*bs*cnu+(jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQ[ii], cnx, pQs, cnx);
				dtrtr_l_lib(nx, 0, pQs, cnx, 0, pQs, cnx);	
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, ii*nu, nx, pQs, cnx, pGamma_u[ii-1], cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pQs, cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif

				}

			dgecp_lib(nu, nx, 0, pGamma_w[0], cnx, 0, pM, cnx);
			dsyrk_nt_lib(nu, nu, nx, pM, cnx, pBt[0], cnx, 1, pR[0], cnu, pD, cnu);
			dpotrf_lib(nu, nu, pD, cnu, pD, cnu, diag);
			dtrcp_l_lib(nu, 0, pD, cnu, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}

		}

	}



void d_cond_fact_R_N2_nx3_permute(int N, int nx, int nu, int free_x0, double **pBAt, int diag_hessian, double **pRSQ, double *pD, double *pM, double *pQs, double *pL, double *diag, double *pBAtL, double *pGamma_L, double **pGamma_u, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int pnu = (nu+bs-1)/bs*bs;
	int pnx = (nx+bs-1)/bs*bs;
	int cnu = (nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int nz = nx+nu;
	int cnz = (nz+ncl-1)/ncl*ncl;
	int cnl = cnz<cnx+ncl ? cnx+ncl : cnz;

	int ii, jj, offset, i_temp;

	if(free_x0) // MHE
		{

		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// final stage 
			dgeset_lib(nx, nx, 0.0, 0, pL, cnl);
			ddiain_sqrt_lib(nx, pRSQ[N], 0, pL, cnl);

			dtrtr_l_lib(nx, 0, pL, cnl, 0, pL+(ncl)*bs, cnl);	

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
				dgeset_lib(nz, nz, 0.0, 0, pL, cnl);
				ddiain_lib(nz, pRSQ[ii], 0, pL, cnl);
				dsyrk_dpotrf_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pL, cnl, pL, cnl, diag);

				dtrcp_l_lib(nu, 0, pL, cnl, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cnxNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
				dgecp_lib(nx, nu, nu, pL+nu/bs*bs*cnl+nu%bs, cnl, 0, pD+pnu*cnu, cnu);
				dgemm_nn_lib(nx+(ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, nx+jj*nu, pGamma_L+(nx+jj*nu)/bs*bs*cnu+(nx+jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cnxNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
					}
				dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1-ii)*nu*bs, cnxNnu);

				dtrtr_l_lib(nx, nu, pL+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, pL+(ncl)*bs, cnl);	
				}

			// first stage 
			dtrmm_nt_u_lib(nz, nx, pBAt[0], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
			dgeset_lib(nz, nz, 0.0, 0, pL, cnl);
			ddiain_lib(nz, pRSQ[0], 0, pL, cnl);
			dsyrk_dpotrf_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pL, cnl, pL, cnl, diag);

			dtrcp_l_lib(nz, 0, pL, cnl, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cnxNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cnxNnu);

			}
		else
			{

			// final stage 
			dpotrf_lib(nx, nx, pRSQ[N], cnx, pL, cnl, diag);

			dtrtr_l_lib(nx, 0, pL, cnl, 0, pL+(ncl)*bs, cnl);	

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
				dsyrk_dpotrf_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii], cnz, pL, cnl, diag);

				dtrcp_l_lib(nu, 0, pL, cnl, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cnxNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
				dgecp_lib(nx, nu, nu, pL+nu/bs*bs*cnl+nu%bs, cnl, 0, pD+pnu*cnu, cnu);
				dgemm_nn_lib(nx+(ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, nx+jj*nu, pGamma_L+(nx+jj*nu)/bs*bs*cnu+(nx+jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cnxNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cnxNnu);
					}
				dgecp_lib(nx, nu, 0, pGamma_L, cnu, N*nu, pH_R+N*nu/bs*bs*cnxNnu+N*nu%bs+(N-1-ii)*nu*bs, cnxNnu);

				dtrtr_l_lib(nx, nu, pL+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, pL+(ncl)*bs, cnl);	
				}

			// first stage 
			dtrmm_nt_u_lib(nz, nx, pBAt[0], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
			dsyrk_dpotrf_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[0], cnz, pL, cnl, diag);

			dtrcp_l_lib(nz, 0, pL, cnl, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cnxNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cnxNnu);

			}

		}
	else // MPC
		{

		int cNnu = (N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// final stage 
			dgeset_lib(nx, nx, 0.0, 0, pL, cnl);
			ddiain_sqrt_lib(nx, pRSQ[N], 0, pL, cnl);

			dtrtr_l_lib(nx, 0, pL, cnl, 0, pL+(ncl)*bs, cnl);	

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
				dgeset_lib(nz, nz, 0.0, 0, pL, cnl);
				ddiain_lib(nz, pRSQ[ii], 0, pL, cnl);
				dsyrk_dpotrf_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pL, cnl, pL, cnl, diag);

				dtrcp_l_lib(nu, 0, pL, cnl, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
				dgecp_lib(nx, nu, nu, pL+nu/bs*bs*cnl+nu%bs, cnl, 0, pD+pnu*cnu, cnu);
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pGamma_L+(jj*nu)/bs*bs*cnu+(jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dtrtr_l_lib(nx, nu, pL+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, pL+(ncl)*bs, cnl);	
				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAt[0], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
			dgeset_lib(nu, nu, 0.0, 0, pL, cnl);
			ddiain_lib(nu, pRSQ[0], 0, pL, cnl);
			dsyrk_dpotrf_lib(nu, nu, nx, pBAtL, cnx, pBAtL, cnx, 1, pL, cnl, pL, cnl, diag);

			dtrcp_l_lib(nu, 0, pL, cnl, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}
		else
			{

			// final stage 
			dpotrf_lib(nx, nx, pRSQ[N], cnx, pL, cnl, diag);

			dtrtr_l_lib(nx, 0, pL, cnl, 0, pL+(ncl)*bs, cnl);	

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
				dsyrk_dpotrf_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii], cnz, pL, cnl, diag);

				dtrcp_l_lib(nu, 0, pL, cnl, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
				dgecp_lib(nx, nu, nu, pL+nu/bs*bs*cnl+nu%bs, cnl, 0, pD+pnu*cnu, cnu);
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pGamma_L+(jj*nu)/bs*bs*cnu+(jj*nu)%bs, cnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dtrtr_l_lib(nx, nu, pL+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, pL+(ncl)*bs, cnl);	
				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAt[0], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
			dsyrk_dpotrf_lib(nu, nu, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[0], cnu, pL, cnl, diag);

			dtrcp_l_lib(nu, 0, pL, cnl, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}

		}

	}



void d_cond_Gamma_u_b_T(int N, int nx, int nu, int free_x0, double **pA, double **pBt, double **b, double *pGamma_L, double **pGamma_u)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;

	int ii, offset0, offset1;

	if(free_x0) // MHE
		{
		dgecp_lib(nu, nx, 0, pBt[0], cnx, 0, pGamma_u[0], cnx);
		offset0 = nu;
		dgetr_lib(nx, nx, 0, pA[0], cnx, offset0, pGamma_u[0]+offset0/bs*bs*cnx+offset0%bs, cnx);
		offset0 = nu+nx;
		drowin_lib(nx, b[0], pGamma_u[0]+offset0/bs*bs*cnx+offset0%bs);
		for(ii=1; ii<N; ii++)
			{
			dgemm_nt_lib(nx+1+ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_L, cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
			offset0 = ii*nu+nx;
			drowad_lib(nx, 1.0, b[ii], pGamma_u[ii]+offset0/bs*bs*cnx+offset0%bs);
			offset0 = ii*nu;
			offset1 = (ii+1)*nu;
			dgecp_lib(nx+1, nx, offset0, pGamma_u[ii]+offset0/bs*bs*cnx+offset0%bs, cnx, 0, pGamma_L, cnx);
			dgecp_lib(nx+1, nx, 0, pGamma_L, cnx, offset1, pGamma_u[ii]+offset1/bs*bs*cnx+offset1%bs, cnx);
			dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset0, pGamma_u[ii]+offset0/bs*bs*cnx+offset0%bs, cnx);
			}
		return;
		}
	else // MPC
		{
		dgecp_lib(nu, nx, 0, pBt[0], cnx, 0, pGamma_u[0], cnx);
		offset0 = nu;
		drowin_lib(nx, b[0], pGamma_u[0]+offset0/bs*bs*cnx+offset0%bs);
		for(ii=1; ii<N; ii++)
			{
#if 1
			dgemm_nt_lib(nx, 1+ii*nu, nx, pA[ii], cnx, pGamma_u[ii-1], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
			dgemm_nt_lib(1+ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
			offset0 = ii*nu;
			drowad_lib(nx, 1.0, b[ii], pGamma_u[ii]+offset0/bs*bs*cnx+offset0%bs);
			offset1 = (ii+1)*nu;
			dgecp_lib(1, nx, offset0, pGamma_u[ii]+offset0/bs*bs*cnx+offset0%bs, cnx, offset1, pGamma_u[ii]+offset1/bs*bs*cnx+offset1%bs, cnx);
			dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset0, pGamma_u[ii]+offset0/bs*bs*cnx+offset0%bs, cnx);
			}
		return;
		}

	}



void d_cond_Rr_N2_nx3(int N, int nx, int nu, int free_x0, double **pBAbt, int diag_hessian, int nzero_Q_N, double **pRSQrq, double *pD, double *pM, double *pP, double *pLam, double *diag, double *pBAbtL, double *pGamma_L, double **pGamma_u_b, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, offset0, offset1, i_temp;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	int nz = nx+nu+1;
	int nux = nu+nx;
	int pnz = (nu+nx+1+bs-1)/bs*bs;
	int cnux = (nu+nx+ncl-1)/ncl*ncl;

	if(free_x0) // MHE
		{

		int cnxNnu = (nx+N*nu+ncl-1)/ncl*ncl;
		int cnx1Nnu = (nx+1+N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// TODO
			exit(1);

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					offset = nx;
					dgeset_lib(nx+nu, nx+nu, 0.0, 0, pH_R, cnxNnu);
					ddiaad_lib(nu, 1.0, pRSQrq[0], offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					ddiaad_lib(nx, 1.0, pRSQrq[0]+nu, 0, pH_R, cnxNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset = nx + ii*nu;
					dgeset_lib(nx+(ii+1)*nu, nu, 0.0, 0, pH_R+offset*bs, cnxNnu);
					ddiaad_lib(nu, 1.0, pRSQrq[ii], offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);
					ddiain_sqrt_lib(nx, pRSQrq[ii]+nu, 0, pP, cnx);

					ii--;

					}

				}
			else
				{

				if(N==1)
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQrq[1][ii]);

					// first stage 
					dgemm_diag_right_lib(nux, nx, pBAbt[0], cnx, pP, 0, pBAbtL, cnx, pBAbtL, cnx);
					dsyrk_nt_lib(nux, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
					ddiaad_lib(nux, 1.0, pRSQrq[0], 0, pLam, cnux);

					offset = nx;
					dtrtr_l_lib(nu, 0, pLam, cnux, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pH_R+offset*bs, cnxNnu);
					dtrcp_l_lib(nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pH_R, cnxNnu);

					// transpose H in the lower triangular
					dgetr_lib(nx, N*nu, 0, pH_R+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs, cnxNnu);
					dtrtr_u_lib(N*nu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu);

					return;

					}
				else
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQrq[N][ii]);

					// second final stage 
					ii = N-1;
					dgemm_diag_right_lib(nux, nx, pBAbt[ii], cnx, pP, 0, pBAbtL, cnx, pBAbtL, cnx);
					dsyrk_nt_lib(nux, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
					ddiaad_lib(nux, 1.0, pRSQrq[ii], 0, pLam, cnux);

					offset = nx + ii*nu;
					dtrtr_l_lib(nu, 0, pLam, cnux, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
					dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
					dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);

					dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
					dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}

				}

			// middle stages 
			for(; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nux, nx, pBAbt[ii], cnx, pP, cnx, pBAbtL, cnx);
				dsyrk_nt_lib(nux, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
				ddiaad_lib(nux, 1.0, pRSQrq[ii], 0, pLam, cnux);

				offset = nx + ii*nu;
				dtrtr_l_lib(nu, 0, pLam, cnux, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
				dgemm_nt_lib(nx+ii*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pH_R+offset*bs, cnxNnu, pH_R+offset*bs, cnxNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
				dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nux, nx, pBAbt[0], cnx, pP, cnx, pBAbtL, cnx);
			dsyrk_nt_lib(nux, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
			ddiaad_lib(nux, 1.0, pRSQrq[0], 0, pLam, cnux);

			offset = nx;
			dtrtr_l_lib(nu, 0, pLam, cnux, offset, pH_R+offset/bs*bs*cnxNnu+offset%bs+offset*bs, cnxNnu);
			dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pH_R+offset*bs, cnxNnu);
			dtrcp_l_lib(nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pH_R, cnxNnu);

			// transpose H in the lower triangular
			dgetr_lib(nx, N*nu, 0, pH_R+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs, cnxNnu);
			dtrtr_u_lib(N*nu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu, nx, pH_R+nx/bs*bs*cnxNnu+nx%bs+nx*bs, cnxNnu);

			return;

			}
		else // dense Hessian
			{

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					offset0 = nux;
					dtrcp_l_lib(nux, 0, pRSQrq[0], cnux, 0, pH_R, cnxNnu);
					dgecp_lib(1, nux, offset0, pRSQrq[0]+offset0/bs*bs*cnux+offset0%bs, cnux, offset0, pH_R+offset0/bs*bs*cnxNnu+offset0%bs, cnxNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx+1, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset0 = ii*nu;
					dtrcp_l_lib(nu, 0, pRSQrq[ii], cnux, offset0, pH_R+offset0/bs*bs*cnxNnu+offset0%bs+offset0*bs, cnxNnu);
					dgetr_lib(nx, nu, nu, pRSQrq[ii]+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
					dgemm_nt_lib(nx+1+ii*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
					offset1 = nx+ii*nu;
					dgead_lib(1, nu, 1.0, nux, pRSQrq[ii]+nux/bs*bs*cnux+nux%bs, cnux, offset1, pGamma_L+offset1/bs*bs*cnu+offset1%bs, cnu);
					dgetr_lib(ii*nu, nu, 0, pGamma_L, cnu, offset0, pH_R+offset0/bs*bs*cnxNnu+offset0%bs, cnxNnu);
					offset1 = N*nu;
					dgecp_lib(nx+1, nu, offset0, pGamma_L+offset0/bs*bs*cnu+offset0%bs, cnu, offset1, pH_R+offset1/bs*bs*cnxNnu+offset1%bs+offset0*bs, cnxNnu);

					dtrcp_l_lib(nx+1, nu, pRSQrq[ii]+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
					dpotrf_lib(nx+1, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}
				}
			else
				{
				// final stage 
				dpotrf_lib(nx+1, nx, pRSQrq[N], cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				ii = N-1;
				}

			// middle stages 
			for(; ii>0; ii--)
				{	

				dtrmm_nt_u_lib(nz, nx, pBAbt[ii], cnx, pP, cnx, pBAbtL, cnx);
				dgead_lib(1, nx, 1.0, nux, pP+nx/bs*bs*cnx+nx%bs, cnx, nux, pBAbtL+nux/bs*bs*cnx+nux%bs, cnx);
				dsyrk_nt_lib(nz, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 1, pRSQrq[ii], cnux, pLam, cnux);

				offset0 = ii*nu;
				dtrcp_l_lib(nu, 0, pLam, cnux, offset0, pH_R+offset0/bs*bs*cnxNnu+offset0%bs+offset0*bs, cnxNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
				dgemm_nt_lib(nx+1+ii*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				offset1 = nx+ii*nu;
				dgead_lib(1, nu, 1.0, nux, pLam+nux/bs*bs*cnux+nux%bs, cnux, offset1, pGamma_L+offset1/bs*bs*cnu+offset1%bs, cnu);
				dgetr_lib(ii*nu, nu, 0, pGamma_L, cnu, offset0, pH_R+offset0/bs*bs*cnxNnu+offset0%bs, cnxNnu);
				offset1 = N*nu;
				dgecp_lib(nx+1, nu, offset0, pGamma_L+offset0/bs*bs*cnu+offset0%bs, cnu, offset1, pH_R+offset1/bs*bs*cnxNnu+offset1%bs+offset0*bs, cnxNnu);

				dgecp_lib(nx+1, nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
				dpotrf_lib(nx+1, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nz, nx, pBAbt[0], cnx, pP, cnx, pBAbtL, cnx);
			dgead_lib(1, nx, 1.0, nux, pP+nx/bs*bs*cnx+nx%bs, cnx, nux, pBAbtL+nux/bs*bs*cnx+nux%bs, cnx);
			dsyrk_nt_lib(nz, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 1, pRSQrq[0], cnux, pLam, cnux);

			dtrcp_l_lib(nu, 0, pLam, cnux, 0, pH_R, cnxNnu);
			offset0 = N*nu;
			dgecp_lib(nx+1, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, offset0, pH_R+offset0/bs*bs*cnxNnu+offset0%bs, cnxNnu);
			dtrcp_l_lib(nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, offset0, pH_R+offset0/bs*bs*cnxNnu+offset0%bs+offset0*bs, cnxNnu); // TODO routine to merge with the following
			offset1 = N*nu + nx;
			dgecp_lib(1, nx, nux, pLam+nux/bs*bs*cnux+nux%bs+nu*bs, cnux, offset1, pH_R+offset1/bs*bs*cnxNnu+offset1%bs+offset0*bs, cnxNnu);

			return;

			}

		}
	else // fixed x0: MPC
		{

		int cNnu = (N*nu+ncl-1)/ncl*ncl;
		int c1Nnu = (1+N*nu+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// TODO
			exit(1);

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					dgeset_lib(nx+nu, nx+nu, 0.0, 0, pH_R, cNnu);
					ddiaad_lib(nu, 1.0, pRSQrq[0], 0, pH_R, cNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset = ii*nu;
					dgeset_lib((ii+1)*nu, nu, 0.0, 0, pH_R+offset*bs, cNnu);
					ddiaad_lib(nu, 1.0, pRSQrq[ii], offset, pH_R+offset/bs*bs*cNnu+offset%bs+offset*bs, cNnu);
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);
					ddiain_sqrt_lib(nx, pRSQrq[ii]+nu, 0, pP, cnx);

					ii--;

					}

				}
			else
				{

				if(N==1)
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQrq[1][ii]);

					// first stage 
					dgemm_diag_right_lib(nu, nx, pBAbt[0], cnx, pP, 0, pBAbtL, cnx, pBAbtL, cnx);
					dsyrk_nt_lib(nu, nu, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
					ddiaad_lib(nu, 1.0, pRSQrq[0], 0, pLam, cnux);
					dtrtr_l_lib(nu, 0, pLam, cnux, 0, pH_R, cNnu);

					// transpose H in the lower triangular
					dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

					return;

					}
				else
					{

					// final stage 
					for(ii=0; ii<nx; ii++)
						pP[ii] = sqrt(pRSQrq[N][ii]);

					// second final stage 
					ii = N-1;
					dgemm_diag_right_lib(nux, nx, pBAbt[ii], cnx, pP, 0, pBAbtL, cnx, pBAbtL, cnx);
					dsyrk_nt_lib(nux, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
					ddiaad_lib(nux, 1.0, pRSQrq[ii], 0, pLam, cnux);
					dtrtr_l_lib(nu, 0, pLam, cnux, (ii)*nu, pH_R+((ii)*nu)/bs*bs*cNnu+((ii)*nu)%bs+(ii)*nu*bs, cNnu);
					dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
					dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);

					dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
					dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}

				}

			// middle stages 
			for(; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nux, nx, pBAbt[ii], cnx, pP, cnx, pBAbtL, cnx);
				dsyrk_nt_lib(nux, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
				ddiaad_lib(nux, 1.0, pRSQrq[ii], 0, pLam, cnux);

				dtrtr_l_lib(nu, 0, pLam, cnux, (ii)*nu, pH_R+((ii)*nu)/bs*bs*cNnu+((ii)*nu)%bs+(ii)*nu*bs, cNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
				dpotrf_lib(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAbt[0], cnx, pP, cnx, pBAbtL, cnx);
			dsyrk_nt_lib(nu, nu, nx, pBAbtL, cnx, pBAbtL, cnx, 0, pLam, cnux, pLam, cnux);
			ddiaad_lib(nu, 1.0, pRSQrq[0], 0, pLam, cnux);
			dtrtr_l_lib(nu, 0, pLam, cnux, 0, pH_R, cNnu);

			// transpose H in the lower triangular
			dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

			return;

			}
		else // dense hessian
			{

			if(nzero_Q_N==0)
				{

				if(N==1)
					{

					// first stage 
					offset0 = nu;
					dtrcp_l_lib(nu, 0, pRSQrq[0], cnu, 0, pH_R, cNnu);
					dgecp_lib(1, nu, offset0, pRSQrq[0]+offset0/bs*bs*cnu+offset0%bs, cnu, offset0, pH_R+offset0/bs*bs*cNnu+offset0%bs, cNnu);

					return;

					}
				else
					{

					// final stage 
					dgeset_lib(nx, nx, 0.0, 0, pP, cnx);

					// second final stage 
					ii = N-1;
					offset0 = ii*nu;
					dtrcp_l_lib(nu, 0, pRSQrq[ii], cnux, offset0, pH_R+offset0/bs*bs*cNnu+offset0%bs+offset0*bs, cNnu);

					dgetr_lib(nx, nu, nu, pRSQrq[ii]+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
					dgemm_nt_lib(1+ii*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
					dgead_lib(1, nu, 1.0, nux, pRSQrq[ii]+nux/bs*bs*cnux+nux%bs, cnux, offset0, pGamma_L+offset0/bs*bs*cnu+offset0%bs, cnu);
					dgetr_lib(ii*nu, nu, 0, pGamma_L, cnu, offset0, pH_R+offset0/bs*bs*cNnu+offset0%bs, cNnu);
					offset1 = N*nu;
					dgecp_lib(1, nu, offset0, pGamma_L+offset0/bs*bs*cnu+offset0%bs, cnu, offset1, pH_R+offset1/bs*bs*cNnu+offset1%bs+offset0*bs, cNnu);

					dtrcp_l_lib(nx+1, nu, pRSQrq[ii]+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
					dpotrf_lib(nx+1, nx, pP, cnx, pP, cnx, diag);
					dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

					ii--;

					}

				}
			else
				{

				// final stage 
				dpotrf_lib(nx, nx, pRSQrq[N], cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				ii = N-1;

				}

			// middle stages 
			for(; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAbt[ii], cnx, pP, cnx, pBAbtL, cnx);
				dgead_lib(1, nx, 1.0, nux, pP+nx/bs*bs*cnx+nx%bs, cnx, nux, pBAbtL+nux/bs*bs*cnx+nux%bs, cnx);
				dsyrk_nt_lib(nz, nux, nx, pBAbtL, cnx, pBAbtL, cnx, 1, pRSQrq[ii], cnux, pLam, cnux);

				offset0 = ii*nu;
				dtrcp_l_lib(nu, 0, pLam, cnux, offset0, pH_R+offset0/bs*bs*cNnu+offset0%bs+offset0*bs, cNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnux+nu%bs, cnux, 0, pM, cnx);
				dgemm_nt_lib(1+ii*nu, nu, nx, pGamma_u_b[ii-1], cnx, pM, cnx, 0, pGamma_L, cnu, pGamma_L, cnu, 0, 0);
				dgead_lib(1, nu, 1.0, nux, pLam+nux/bs*bs*cnux+nux%bs, cnux, offset0, pGamma_L+offset0/bs*bs*cnu+offset0%bs, cnu);
				dgetr_lib(ii*nu, nu, 0, pGamma_L, cnu, offset0, pH_R+offset0/bs*bs*cNnu+offset0%bs, cNnu);
				offset1 = N*nu;
				dgecp_lib(1, nu, offset0, pGamma_L+offset0/bs*bs*cnu+offset0%bs, cnu, offset1, pH_R+offset1/bs*bs*cNnu+offset1%bs+offset0*bs, cNnu);

				dgecp_lib(nx+1, nx, nu, pLam+nu/bs*bs*cnux+nu%bs+nu*bs, cnux, 0, pP, cnx);
				dpotrf_lib(nx+1, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nu+1, nx, pBAbt[0], cnx, pP, cnx, pBAbtL, cnx);
			dgead_lib(1, nx, 1.0, nux, pP+nx/bs*bs*cnx+nx%bs, cnx, nu, pBAbtL+nu/bs*bs*cnx+nu%bs, cnx);
			dsyrk_nt_lib(nu+1, nu, nx, pBAbtL, cnx, pBAbtL, cnx, 1, pRSQrq[0], cnu, pLam, cnu);

			dtrcp_l_lib(nu, 0, pLam, cnux, 0, pH_R, cNnu);
			offset0 = N*nu;
			dgecp_lib(1, nu, nu, pLam+nu/bs*bs*cnu+nu%bs, cnu, offset0, pH_R+offset0/bs*bs*cNnu+offset0%bs, cNnu);

			return;

			}

		}

	return;

	}



void d_cond_d(int N, int nx, int nu, int *nb, int free_x0, double **hd, int **hidx, double **hpGamma_u, double *d2, int *idx2, double *pDCt2)
	{

	const int bs  = D_MR;
	const int ncl = D_NCL;

	int ii, jj;

	int pnb, nb2, pnb2, nb_tmp, ng2, png2, cng2, ng_tmp, ng_tmp2;
	int offset0, offset1, idx_tmp;
	double Gamma_b_tmp;

	int cnx    = (nx+ncl-1)/ncl*ncl;


	// inputs
	nb2 = 0;
	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(hidx[ii][jj]<nu)
				nb2++;
	// initial state
	for(jj=0; jj<nb[0]; jj++)
		if(hidx[0][jj]>=nu)
			nb2++;
	pnb2 = (nb2+bs-1)/bs*bs;
	// following states
	ng2 = 0;
	for(ii=1; ii<N; ii++)
		for(jj=0; jj<nb[ii]; jj++)
			if(hidx[ii][jj]>=nu)
				ng2++;
	png2 = (ng2+bs-1)/bs*bs;
	cng2 = (ng2+ncl-1)/ncl*ncl;

	nb_tmp = 0;
	// inputs
	for(ii=0; ii<N; ii++)
		{
		pnb = (nb[ii]+bs-1)/bs*bs;

		for(jj=0; jj<nb[ii]; jj++)
			{
			if(hidx[ii][jj]<nu)
				{
				d2[nb_tmp]      = hd[ii][jj]; // lower bound
				d2[pnb2+nb_tmp] = hd[ii][pnb+jj]; // upper bound
				idx2[nb_tmp] = hidx[ii][jj]+ii*nu; // index
				nb_tmp++;
				}
			}
		}
	// states on the first stage
	ii = 0;
	pnb = (nb[ii]+bs-1)/bs*bs;

	for(jj=0; jj<nb[ii]; jj++)
		{
		if(hidx[ii][jj]>=nu)
			{
			d2[nb_tmp]      = hd[ii][jj]; // lower bound
			d2[pnb2+nb_tmp] = hd[ii][pnb+jj]; // upper bound
			idx2[nb_tmp] = hidx[ii][jj]-nu+N*nu; // index
			nb_tmp++;
			}
		}
	// other states as general constraints
	// XXX assumes that all the first components of the states are constrained, and in the proper order (i.e. C of the original problem is the identity I)
	// TODO general case !!!!!!!!!!!!!!!!!!!!!!!!!

	if(free_x0)
		{

		dgeset_lib(nx+N*nu, ng2, 0.0, 0, pDCt2, cng2);

		ng_tmp2 = 0;
		for(ii=1; ii<N; ii++)
			{

			pnb = (nb[ii]+bs-1)/bs*bs;

//			d_print_pmat(1+ii*nu, nx, bs, hpGamma_u[ii-1], cnx);
			// count the bounds on states
			ng_tmp = 0;
			for(jj=0; jj<nb[ii]; jj++)
				{
				if(hidx[ii][jj]>=nu)
					{
					idx_tmp = hidx[ii][jj]-nu;
					offset0 = nx+ii*nu;
					Gamma_b_tmp = hpGamma_u[ii-1][offset0/bs*bs*cnx+offset0%bs+idx_tmp*bs];
					d2[2*pnb2+ng_tmp2+ng_tmp]      = hd[ii][jj]     - Gamma_b_tmp; // lower bound
					d2[2*pnb2+png2+ng_tmp2+ng_tmp] = hd[ii][pnb+jj] + Gamma_b_tmp; // upper bound
//					printf("\n%d %d %f %f %f %f %f\n", hidx[ii][jj], idx_tmp, Gamma_b_tmp, hd[ii][jj], d2[2*pnb2+ng_tmp2+ng_tmp], hd[ii][pnb+jj], d2[2*pnb2+png2+ng_tmp2+ng_tmp]);
					ng_tmp++;
					}
				}

			// copy
			dgecp_lib(ii*nu, ng_tmp, 0, hpGamma_u[ii-1], cnx, 0, pDCt2+ng_tmp2*bs, cng2);
			offset0 = ii*nu;
			offset1 = N*nu;
			dgecp_lib(nx, ng_tmp, offset0, hpGamma_u[ii-1]+offset0/bs*bs*cnx+offset0%bs, cnx, offset1, pDCt2+offset1/bs*bs*cng2+offset1%bs+ng_tmp2*bs, cng2); 

			ng_tmp2 += ng_tmp;

			}

//		d_print_pmat(nx+N*nu, ng2, bs, pDCt2, cng2);

		}
	else
		{

		dgeset_lib(N*nu, ng2, 0.0, 0, pDCt2, cng2);

		ng_tmp2 = 0;
		for(ii=1; ii<N; ii++)
			{

			pnb = (nb[ii]+bs-1)/bs*bs;

//			d_print_pmat(1+ii*nu, nx, bs, hpGamma_u[ii-1], cnx);
			// count the bounds on states
			ng_tmp = 0;
			for(jj=0; jj<nb[ii]; jj++)
				{
				if(hidx[ii][jj]>=nu)
					{
					idx_tmp = hidx[ii][jj]-nu;
					offset0 = ii*nu;
					Gamma_b_tmp = hpGamma_u[ii-1][offset0/bs*bs*cnx+offset0%bs+idx_tmp*bs];
					d2[2*pnb2+ng_tmp2+ng_tmp]      = hd[ii][jj]     - Gamma_b_tmp; // lower bound
					d2[2*pnb2+png2+ng_tmp2+ng_tmp] = hd[ii][pnb+jj] + Gamma_b_tmp; // upper bound
//					printf("\n%d %d %f %f %f %f %f\n", hidx[ii][jj], idx_tmp, Gamma_b_tmp, hd[ii][jj], d2[2*pnb2+ng_tmp2+ng_tmp], hd[ii][pnb+jj], d2[2*pnb2+png2+ng_tmp2+ng_tmp]);
					ng_tmp++;
					}
				}

			// copy
			dgecp_lib(ii*nu, ng_tmp, 0, hpGamma_u[ii-1], cnx, 0, pDCt2+ng_tmp2*bs, cng2);

			ng_tmp2 += ng_tmp;

			}

//		d_print_pmat(N*nu, ng2, bs, pDCt2, cng2);

		}
	
//	for(ii=0; ii<N; ii++)
//		d_print_mat(1, 2*pnb2+2*png2, d2, 1);

	}


	









void d_cond_Q(int N, int nx, int nu, double **pA, int diag_Q, int nzero_Q_N, double **pQ, double **pL, int compute_Gamma_0, double **pGamma_0, double **pGamma_0_Q, double *pH_Q, double *work)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii, jj;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	if(compute_Gamma_0)
		{
		dgetr_lib(nx, nx, 0, pA[0], cnx, 0, pGamma_0[0], cnx);
		for(ii=1; ii<N; ii++)
			{
			dgemm_nt_lib(nx, nx, nx, pGamma_0[ii-1], cnx, pA[ii], cnx, 0, pGamma_0[ii], cnx, pGamma_0[ii], cnx, 0, 0);
			}
		}
	
	if(diag_Q)
		{

#if 0
		for(jj=0; jj<nx; jj++) pL[1][jj] = sqrt(pQ[1][jj]);
		dgemm_diag_right_lib(nx, nx, pGamma_0[0], cnx, pL[1], pGamma_0_Q[0], cnx, pGamma_0_Q[0], cnx, 0);
		dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[0], cnx, pGamma_0_Q[0], cnx, pH_Q, cnx, pH_Q, cnx, 0);
		for(ii=1; ii<N1; ii++)
			{
			for(jj=0; jj<nx; jj++) pL[ii+1][jj] = sqrt(pQ[ii+1][jj]);
			dgemm_diag_right_lib(nx, nx, pGamma_0[ii], cnx, pL[ii+1], pGamma_0_Q[ii], cnx, pGamma_0_Q[ii], cnx, 0);
			dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[ii], cnx, pGamma_0_Q[ii], cnx, pH_Q, cnx, pH_Q, cnx, 1);
			}
		d_add_diag_pmat(nx, pH_Q, cnx, pQ[0]);
#else
		if(N1>0)
			{
			dgemm_diag_right_lib(nx, nx, pGamma_0[0], cnx, pQ[1], 0, pGamma_0_Q[0], cnx, pGamma_0_Q[0], cnx);
			dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[0], cnx, pGamma_0[0], cnx, 0, pH_Q, cnx, pH_Q, cnx);
			}
		else
			{
			dgeset_lib(nx, nx, 0.0, 0, pH_Q, cnx);
			}
		for(ii=1; ii<N1; ii++)
			{
			dgemm_diag_right_lib(nx, nx, pGamma_0[ii], cnx, pQ[ii+1], 0, pGamma_0_Q[ii], cnx, pGamma_0_Q[ii], cnx);
			dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[ii], cnx, pGamma_0[ii], cnx, 1, pH_Q, cnx, pH_Q, cnx);
			}
		//d_add_diag_pmat(nx, pH_Q, cnx, pQ[0]);
		ddiaad_lib(nx, 1.0, pQ[0], 0, pH_Q, cnx);
#endif

		}
	else
		{
#if 1

		dpotrf_lib_old(nx, nx, pQ[1], cnx, pL[1], cnx, work);
		dtrtr_l_lib(nx, 0, pL[1], cnx, 0, pL[1], cnx);
		dtrmm_nt_u_lib(nx, nx, pGamma_0[0], cnx, pL[1], cnx, pGamma_0_Q[0], cnx);
		dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[0], cnx, pGamma_0_Q[0], cnx, 1, pQ[0], cnx, pH_Q, cnx);
		for(ii=1; ii<N1; ii++)
			{
			dpotrf_lib_old(nx, nx, pQ[ii+1], cnx, pL[ii+1], cnx, work);
			dtrtr_l_lib(nx, 0, pL[ii+1], cnx, 0, pL[ii+1], cnx);
			dtrmm_nt_u_lib(nx, nx, pGamma_0[ii], cnx, pL[ii+1], cnx, pGamma_0_Q[ii], cnx);
			dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[ii], cnx, pGamma_0_Q[ii], cnx, 1, pH_Q, cnx, pH_Q, cnx);
			}
#else
	
		// Gamma_0 * bar_Q * Gamma_0'
		dgemm_nt_lib(nx, nx, nx, pGamma_0[0], cnx, pQ[1], cnx, pGamma_0_Q[0], cnx, pGamma_0_Q[0], cnx, 0, 0, 0);
		//dgemm_nt_lib(nx, nx, nx, pGamma_0_Q[0], cnx, pGamma_0[0], cnx, pQ[0], cnx, pH_Q, cnx, 1, 0, 0);
		dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[0], cnx, pGamma_0[0], cnx, pQ[0], cnx, pH_Q, cnx, 1);
		for(ii=1; ii<N1; ii++)
			{
			dgemm_nt_lib(nx, nx, nx, pGamma_0[ii], cnx, pQ[ii+1], cnx, pGamma_0_Q[ii], cnx, pGamma_0_Q[ii], cnx, 0, 0, 0);
			//dgemm_nt_lib(nx, nx, nx, pGamma_0_Q[ii], cnx, pGamma_0[ii], cnx, pH_Q, cnx, pH_Q, cnx, 1, 0, 0);
			dsyrk_nt_lib(nx, nx, nx, pGamma_0_Q[ii], cnx, pGamma_0[ii], cnx, pH_Q, cnx, pH_Q, cnx, 1);
			}
#endif
		}

	}



// condensing algorithms:
// alg==0 : N^3 n_x^2 algorithm
// alg==1 : N^2 n_x^2 algorithm
// alg==2 : N^2 n_x^3 algorithm
void d_cond_R(int N, int nx, int nu, int alg, double **pA, double **pAt, double **pBt, double **pBAt, int diag_hessian, int nzero_Q_N, double **pQ, int use_L, double **pL, double **pS, double **pR, double **pRSQ, double *pD, double *pM, double *pP, double *pLam, double *diag, double *pBAtL, int compute_Gamma_u, double **pGamma_u, double **pGamma_u_Q, double **pGamma_u_Q_A, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	

	// Gamma_u^T
	if(compute_Gamma_u)
		{
		dgecp_lib(nu, nx, 0, pBt[0], cnx, 0, pGamma_u[0], cnx);
		for(ii=1; ii<N; ii++)
			{
			offset = ii*nu;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
			dgemm_nt_lib(nx, ii*nu, nx, pA[ii], cnx, pGamma_u[ii-1], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
			dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
			dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset, pGamma_u[ii]+offset/bs*bs*cnx+offset%bs, cnx);
			}
		}


	if(alg==0) // N^3 n_x^2
		{
		
		if(diag_hessian)
			{

			// Gamma_u * Q
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib((ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}

			if(nzero_Q_N==0)
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);

			dgeset_lib(N*nu, N*nu, 0.0, 0, pH_R, cNnu);

			// dR
			for(ii=0; ii<N; ii++)
				{
				ddiain_lib(nu, pR[ii], ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			for(ii=0; ii<N1; ii++)
				dsyrk_nt_lib((N1-ii)*nu, (N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u[N1-1-ii], cnx, 1, pH_R, cNnu, pH_R, cNnu); 

			}
		else
			{

			// Gamma_u * Q
			for(ii=0; ii<N1; ii++)
				{
				if(use_L)
					{
					dtrmm_nt_u_lib((ii+1)*nu, nx, pGamma_u[ii], cnx, pL[ii+1], cnx, pGamma_u_Q[ii], cnx);
					}
				else
					{
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
					dgemm_nt_lib(nx, (ii+1)*nu, nx, pQ[ii+1], cnx, pGamma_u[ii], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
					dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
					}
				}

			if(nzero_Q_N==0)
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);

			// Gamma_u * bar_S
			for(ii=1; ii<N; ii++)
				{
				dgemm_nt_lib(ii*nu, nu, nx, pGamma_u[ii-1], cnx, pS[ii], cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);
				}

			// transpose H in the lower triangular
			dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nu, 0, pR[ii], cnu, ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			if(use_L)
				{
				for(ii=0; ii<N1; ii++)
					dsyrk_nt_lib((N1-ii)*nu, (N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u_Q[N1-1-ii], cnx, 1, pH_R, cNnu, pH_R, cNnu); 
				}
			else
				{
				for(ii=0; ii<N1; ii++)
					dsyrk_nt_lib((N1-ii)*nu, (N1-ii)*nu, nx, pGamma_u_Q[N1-1-ii], cnx, pGamma_u[N1-1-ii], cnx, 1, pH_R, cNnu, pH_R, cNnu); 
				}

			}

		return;

		}
	if(alg==1) // N^2 n_x^2
		{

		// Gamma_u * Q
		if(diag_hessian)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_diag_right_lib((ii+1)*nu, nx, pGamma_u[ii], cnx, pQ[ii+1], 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx);
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}
			
			// zero S
			for(ii=1; ii<N; ii++)
				{
				//dgecp_lib(nu, nx, 0, pS[ii], cnx, (ii)*nu, pGamma_u_Q[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				dgeset_lib(nu, nx, 0.0, (ii)*nu, pGamma_u_Q[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				}

			// Gamma_u_Q * bar_A
			dgecp_lib(N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				//dgemm_nt_lib(nx, ii*nu, nx, pAt[ii], cnx, pGamma_u_Q_A[ii], cnx, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 1, 1);
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_u_Q_A[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 1);
#else
				//dgemm_nt_lib(ii*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 0, 0);
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
#endif
				}

#if 0
			dgeset_lib(N*nu, N*nu, 0.0, 0, pH_R, cNnu);
			
			// R
			for(ii=0; ii<N; ii++)
				{
				ddiain_lib(nu, pR[ii], ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			// Gamma_u_Q_A * B
			for(ii=0; ii<N1; ii++)
				{
				dgemm_nt_lib((ii+1)*nu, nu, nx, pGamma_u_Q_A[ii], cnx, pBt[ii], cnx, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 1, 0, 0);
				}

#else

			// Gamma_u * M
			for(ii=1; ii<N1; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);
				}

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 0, pD, cnu, pD, cnu, 0, 0);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				dgecp_lib(nu, nu, 0, pD, cnu, ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

#endif

			}
		else
			{
			for(ii=0; ii<N1; ii++)
				{
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pQ[ii+1], cnx, pGamma_u[ii], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u[ii], cnx, pQ[ii+1], cnx, 0, pGamma_u_Q[ii], cnx, pGamma_u_Q[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
				}

			if(nzero_Q_N==0)
				{
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q[N-1], cnx);
				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
				}

			// copy S
			for(ii=1; ii<N; ii++)
				{
				dgecp_lib(nu, nx, 0, pS[ii], cnx, (ii)*nu, pGamma_u_Q[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				}

//			if(nzero_Q_N==0)
//				dgeset_lib(N*nu, nx, 0.0, 0, pGamma_u_Q_A[N-1], cnx);
			
			// Gamma_u_Q * bar_A^{-1}
			dgecp_lib(N1*nu, nx, 0, pGamma_u_Q[N1-1], cnx, 0, pGamma_u_Q_A[N1-1], cnx);
			for(ii=N1-1; ii>0; ii--)
				{
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_u_Q_A[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_u_Q_A[ii], cnx, pAt[ii], cnx, 1, pGamma_u_Q[ii-1], cnx, pGamma_u_Q_A[ii-1], cnx, 0, 0);
#endif
				}

			// Gamma_u * M
			for(ii=1; ii<N1; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);
				}

			// R
			for(ii=0; ii<N; ii++)
				{
				dgecp_lib(nu, nx, ii*nu, pGamma_u_Q_A[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 1, pR[ii], cnu, pD, cnu, 0, 0);
				dgecp_lib(nu, nu, 0, pD, cnu, ii*nu, pH_R+(ii*nu)/bs*bs*cNnu+(ii*nu)%bs+ii*nu*bs, cNnu);
				}

			}

		// transpose H in the lower triangular
		dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

		return;

		}
	if(alg==2) // N^2 n_x^3 algorithm
		{

		// TODO nzero_Q_N !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		int nz = nx+nu;
		int cnz = (nz+ncl-1)/ncl*ncl;

		if(diag_hessian)
			{

			// final stage 
			dgeset_lib(nx, nx, 0.0, 0, pP, cnx);
			for(jj=0; jj<nx-3; jj+=4)
				{
				pP[jj*cnx+0+(jj+0)*bs] = sqrt(pRSQ[N][nu+jj+0]);
				pP[jj*cnx+1+(jj+1)*bs] = sqrt(pRSQ[N][nu+jj+1]);
				pP[jj*cnx+2+(jj+2)*bs] = sqrt(pRSQ[N][nu+jj+2]);
				pP[jj*cnx+3+(jj+3)*bs] = sqrt(pRSQ[N][nu+jj+3]);
				}
			for(kk=0; kk<nx-jj; kk++)
				{
				pP[jj*cnx+kk+(jj+kk)*bs] = sqrt(pRSQ[N][nu+jj+kk]);
				}

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pP, cnx, pBAtL, cnx);
				dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
				ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pLam, cnz);
				dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

				dgecp_lib(nu, nu, 0, pLam, cnz, (ii)*nu, pH_R+((ii)*nu)/bs*bs*cNnu+((ii)*nu)%bs+(ii)*nu*bs, cNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
				dpotrf_lib_old(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAt[0], cnx, pP, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nu, nu, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nu, 1.0, pRSQ[0], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, 0, pH_R, cNnu);

			}
		else
			{

			// final stage 
			dgecp_lib(nx, nx, nu, pRSQ[N]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
			dpotrf_lib_old(nx, nx, pP, cnx, pP, cnx, diag);
			dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pP, cnx, pBAtL, cnx);
				dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii], cnz, pLam, cnz);
				dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

				dgecp_lib(nu, nu, 0, pLam, cnz, (ii)*nu, pH_R+((ii)*nu)/bs*bs*cNnu+((ii)*nu)%bs+(ii)*nu*bs, cNnu);
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pM, cnx, 0, pH_R+ii*nu*bs, cNnu, pH_R+ii*nu*bs, cNnu, 0, 0);

				dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pP, cnx);
				dpotrf_lib_old(nx, nx, pP, cnx, pP, cnx, diag);
				dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAt[0], cnx, pP, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nu, nu, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[0], cnz, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, 0, pH_R, cNnu);

			}

		// transpose H in the lower triangular
		dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

		return;

		}


	
	}



void d_cond_St(int N, int nx, int nu, int nzero_S, double **pS, int nzero_Q_N, double **pGamma_0, int use_pGamma_0_Q, double **pGamma_0_Q, double **pGamma_u_Q, double *pH_St)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	if(nzero_S)
		{
		// Gamma_0 * bar_S
		dgetr_lib(nu, nx, 0, pS[0], cnx, 0, pH_St, cNnu);
		for(ii=1; ii<N; ii++)
			{
			dgemm_nt_lib(nx, nu, nx, pGamma_0[ii-1], cnx, pS[ii], cnx, 0, pH_St+ii*nu*bs, cNnu, pH_St+ii*nu*bs, cNnu, 0, 0);
			}

		if(use_pGamma_0_Q)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pGamma_0_Q[ii], cnx, pGamma_u_Q[ii], 1, cnx, pH_St, cNnu, pH_St, cNnu, 0, 0);
				}
			}
		else
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pGamma_0[ii], cnx, pGamma_u_Q[ii], cnx, 1, pH_St, cNnu, pH_St, cNnu, 0, 0);
				}
			}
		}
	else
		{
		if(use_pGamma_0_Q)
			{
			dgemm_nt_lib(nx, N1*nu, nx, pGamma_0_Q[N1-1], cnx, pGamma_u_Q[N1-1], cnx, 0, pH_St, cNnu, pH_St, cNnu, 0, 0);
			if(nzero_Q_N==0)
				dgeset_lib(nx, nu, 0.0, 0, pH_St+N1*nu*bs, cNnu);
			for(ii=0; ii<N1-1; ii++)
				{
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pGamma_0_Q[ii], cnx, pGamma_u_Q[ii], cnx, 1, pH_St, cNnu, pH_St, cNnu, 0, 0);
				}
			}
		else
			{
			dgemm_nt_lib(nx, N1*nu, nx, pGamma_0[N1-1], cnx, pGamma_u_Q[N1-1], cnx, 0, pH_St, cNnu, pH_St, cNnu, 0, 0);
			if(nzero_Q_N==0)
				dgeset_lib(nx, nu, 0.0, 0, pH_St+N1*nu*bs, cNnu);
			for(ii=0; ii<N1-1; ii++)
				{
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pGamma_0[ii], cnx, pGamma_u_Q[ii], cnx, 1, pH_St, cNnu, pH_St, cNnu, 0, 0);
				}
			}
		}
	
	}



void d_cond_q(int N, int nx, int nu, double **pA, double **b, int diag_hessian, int nzero_Q_N, double **pQ, double **q, double **pGamma_0, int compute_Gamma_b, double **Gamma_b, int compute_Gamma_b_q, double **Gamma_b_q, double *H_q)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	// Gamma_b
	if(compute_Gamma_b)
		{
		d_copy_mat(nx, 1, b[0], 1, Gamma_b[0], 1);
		for(ii=1; ii<N; ii++)
			{
			dgemv_n_lib(nx, nx, pA[ii], cnx, Gamma_b[ii-1], 1, b[ii], Gamma_b[ii]);
			}
		}
	
	// Gamma_b * Q + q
	if(compute_Gamma_b_q)
		{
		if(diag_hessian)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemv_diag_lib(nx, pQ[ii+1], Gamma_b[ii], 1, q[ii+1], Gamma_b_q[ii]);
				}
			}
		else
			{
			for(ii=0; ii<N1; ii++)
				{
				//dgemv_n_lib(nx, nx, pQ[ii+1], cnx, Gamma_b[ii], q[ii+1], Gamma_b_q[ii], 1);
				dsymv_lib(nx, nx, pQ[ii+1], cnx, Gamma_b[ii], 1, q[ii+1], Gamma_b_q[ii]);
				}
			}
		}
		
	// Gamma_0' * Gamma_b_q
	d_copy_mat(nx, 1, q[0], 1, H_q, 1);
	for(ii=0; ii<N1; ii++)
		{
		dgemv_n_lib(nx, nx, pGamma_0[ii], cnx, Gamma_b_q[ii], 1, H_q, H_q);
		}
	
	}



void d_cond_r(int N, int nx, int nu, double **pA, double **b, int diag_hessian, int nzero_Q_N, double **pQ, double **pS, double **q, double **r, double **pGamma_u, int compute_Gamma_b, double **Gamma_b, int compute_Gamma_b_q, double **Gamma_b_q, double *H_r)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii;

	int N1 = N;
	if(nzero_Q_N==0)
		N1 = N-1;
	
	// Gamma_b
	if(compute_Gamma_b)
		{
		d_copy_mat(nx, 1, b[0], 1, Gamma_b[0], 1);
		for(ii=1; ii<N; ii++)
			{
			dgemv_n_lib(nx, nx, pA[ii], cnx, Gamma_b[ii-1], 1, b[ii], Gamma_b[ii]);
			}
		}

	// barS * Gamma_b
	d_copy_mat(nu, 1, r[0], 1, H_r, 1);
	if(diag_hessian)
		{
		for(ii=1; ii<N; ii++)
			{
			dgemv_n_lib(nu, nx, pS[ii], cnx, Gamma_b[ii-1], 1, r[ii], H_r+ii*nu);
			}
		}
	else
		{
		for(ii=1; ii<N; ii++)
			{
			d_copy_mat(nu, 1, r[ii], 1, H_r+ii*nu, 1);
			}
		}
	
	// Gamma_b * Q + q
	if(compute_Gamma_b_q)
		{
		if(diag_hessian)
			{
			for(ii=0; ii<N1; ii++)
				{
				dgemv_diag_lib(nx, pQ[ii+1], Gamma_b[ii], 1, q[ii+1], Gamma_b_q[ii]);
				}
			}
		else
			{
			for(ii=0; ii<N1; ii++)
				{
				//dgemv_n_lib(nx, nx, pQ[ii+1], cnx, Gamma_b[ii], q[ii+1], Gamma_b_q[ii], 1);
				dsymv_lib(nx, nx, pQ[ii+1], cnx, Gamma_b[ii], 1, q[ii+1], Gamma_b_q[ii]);
				}
			}
		}

	// Gamma_u * Gamma_b_q
	for(ii=0; ii<N1; ii++)
		{
		dgemv_n_lib((ii+1)*nu, nx, pGamma_u[ii], cnx, Gamma_b_q[ii], 1, H_r, H_r);
		}
		
	}



// pGamma_x = [A_0' A_0'A_1' A_0'A_1'A_2' ... ], there is not I at the beginning !!!
void d_cond_A(int N, int nx, int nu, double **pA, int compute_Gamma_0, double **pGamma_0, double *pH_A)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii, jj;

	if(compute_Gamma_0)
		{
		dgetr_lib(nx, nx, 0, pA[0], cnx, 0, pGamma_0[0], cnx);
		for(ii=1; ii<N; ii++)
			{
			dgemm_nt_lib(nx, nx, nx, pGamma_0[ii-1], cnx, pA[ii], cnx, 0, pGamma_0[ii], cnx, pGamma_0[ii], cnx, 0, 0);
			}
		}
	
	dgetr_lib(nx, nx, 0, pGamma_0[N-1], cnx, 0, pH_A, cnx);

	}



void d_cond_B(int N, int nx, int nu, double **pA, double **pBt, int compute_Gamma_u, double **pGamma_u, double *pH_B)
	{
	
	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii, jj, offset, i_temp;

	// Gamma_u
	if(compute_Gamma_u)
		{
		dgecp_lib(nu, nx, 0, pBt[0], cnx, 0, pGamma_u[0], cnx);
		for(ii=1; ii<N; ii++)
			{
			offset = ii*nu;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
			dgemm_nt_lib(nx, ii*nu, nx, pA[ii], cnx, pGamma_u[ii-1], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
			dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
			dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset, pGamma_u[ii]+offset/bs*bs*cnx+offset%bs, cnx);
			}
		}
	
	dgetr_lib(N*nu, nx, 0, pGamma_u[N-1], cnx, 0, pH_B, cNnu);

	}



void d_cond_b(int N, int nx, int nu, double **pA, double **b, int compute_Gamma_b, double **Gamma_b, double *H_b)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii;

	// Gamma_b
	if(compute_Gamma_b)
		{
		d_copy_mat(nx, 1, b[0], 1, Gamma_b[0], 1);
		for(ii=1; ii<N; ii++)
			{
			dgemv_n_lib(nx, nx, pA[ii], cnx, Gamma_b[ii-1], 1, b[ii], Gamma_b[ii]);
			}
		}
	
	d_copy_mat(nx, 1, Gamma_b[N-1], 1, H_b, 1);
	
	}
	


void d_part_cond_RQrq(int N, int nx, int nu, double **pBAt, double **b, int diag_hessian, double **pRSQ, double **rq, double **pGamma_x, double **pGamma_u, double **pGamma_b, double *pM, double *pLam, double *diag, double *pBAtL, double *pH_R, double *pH_St, double *pH_Q, double *H_r, double *H_q)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;


	int nz = nx+nu;
	int cnz = (nz+ncl-1)/ncl*ncl;

	if(diag_hessian)
		{

		// final stage N1 = N-1
		ii = N;

		d_copy_mat(nx, 1, rq[ii-1]+nu, 1, H_q, 1);

		d_copy_mat(nu, 1, rq[ii-1], 1, H_r+(ii-1)*nu, 1);

		dgeset_lib(nu, nu, 0.0, 0, pLam, cnz);
		ddiain_lib(nu, pRSQ[ii-1], 0, pLam, cnz);

		dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);
		dgeset_lib((ii-1)*nu, nu, 0.0, 0, pH_R+(ii-1)*nu*bs, cNnu);

//		dgeset_lib(nx, nu, 0.0, 0, pH_St+(ii-1)*bs, cNnu);

		dgeset_lib(nx, nx, 0.0, 0, pH_Q, cnx);
		ddiain_lib(nx, pRSQ[ii-1]+nu, 0, pH_Q, cnx);

		// second final stage N1-1
		ii = N-1;
		if(ii>0) // at least another stage
			{

			dsymv_lib(nx, nx, pH_Q, cnx, b[ii-1], 1, H_q, H_q);
			dgemv_n_lib(nz, nx, pBAt[ii-1], cnx, H_q, 1, rq[ii-1], pLam);
			d_copy_mat(nu, 1, pLam, 1, H_r+(ii-1)*nu, 1);
			d_copy_mat(nx, 1, pLam+nu, 1, H_q, 1);

			for(jj=0; jj<nx-3; jj+=4)
				{
				pH_Q[jj*cnx+0+(jj+0)*bs] = sqrt(pH_Q[jj*cnx+0+(jj+0)*bs]);
				pH_Q[jj*cnx+1+(jj+1)*bs] = sqrt(pH_Q[jj*cnx+1+(jj+1)*bs]);
				pH_Q[jj*cnx+2+(jj+2)*bs] = sqrt(pH_Q[jj*cnx+2+(jj+2)*bs]);
				pH_Q[jj*cnx+3+(jj+3)*bs] = sqrt(pH_Q[jj*cnx+3+(jj+3)*bs]);
				}
			for(kk=0; kk<nx-jj; kk++)
				{
				pH_Q[jj*cnx+kk+(jj+kk)*bs] = sqrt(pH_Q[jj*cnx+kk+(jj+kk)*bs]);
				}

			dtrmm_nt_u_lib(nz, nx, pBAt[ii-1], cnx, pH_Q, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nz, 1.0, pRSQ[ii-1], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, H_r+(ii-1)*nu, H_r+(ii-1)*nu);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
//				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0, 0);
				}
			else
				{
//				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

			// decrease index
			ii--;

			}
		// first stages 
		for(; ii>0; ii--)
			{	

			dsymv_lib(nx, nx, pH_Q, cnx, b[ii-1], 1, H_q, H_q);
			dgemv_n_lib(nz, nx, pBAt[ii-1], cnx, H_q, 1, rq[ii-1], pLam);
			d_copy_mat(nu, 1, pLam, 1, H_r+(ii-1)*nu, 1);
			d_copy_mat(nx, 1, pLam+nu, 1, H_q, 1);

			dpotrf_lib_old(nx, nx, pH_Q, cnx, pH_Q, cnx, diag);
			dtrtr_l_lib(nx, 0, pH_Q, cnx, 0, pH_Q, cnx);	

			dtrmm_nt_u_lib(nz, nx, pBAt[ii-1], cnx, pH_Q, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nz, 1.0, pRSQ[ii-1], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, H_r+(ii-1)*nu, H_r+(ii-1)*nu);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
//				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0, 0);
				}
			else
				{
//				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

			}

		}
	else // dense hessian
		{

		// final stage N1 = N-1
		ii = N;

		d_copy_mat(nx, 1, rq[ii-1]+nu, 1, H_q, 1);

		dgecp_lib(nu, nu, 0, pRSQ[ii-1], cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

		if(ii>1)
			{
			dgetr_lib(nx, nu, nu, pRSQ[ii-1]+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
			dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, rq[ii-1], H_r+(ii-1)*nu);
			dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
//			dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0, 0);
			}
		else
			{
			d_copy_mat(nu, 1, rq[ii-1], 1, H_r+(ii-1)*nu, 1);
//			dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
			}

		dgecp_lib(nx, nx, nu, pRSQ[ii-1]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

		// first stages 
		for(ii=N-1; ii>0; ii--)
			{	

			dsymv_lib(nx, nx, pH_Q, cnx, b[ii-1], 1, H_q, H_q);
			dgemv_n_lib(nz, nx, pBAt[ii-1], cnx, H_q, 1, rq[ii-1], pLam);
			d_copy_mat(nu, 1, pLam, 1, H_r+(ii-1)*nu, 1);
			d_copy_mat(nx, 1, pLam+nu, 1, H_q, 1);

			dpotrf_lib_old(nx, nx, pH_Q, cnx, pH_Q, cnx, diag);
			dtrtr_l_lib(nx, 0, pH_Q, cnx, 0, pH_Q, cnx);	

			dtrmm_nt_u_lib(nz, nx, pBAt[ii-1], cnx, pH_Q, cnx, pBAtL, cnx);
			drowad_lib(nx, 1.0, H_q, pBAtL+(nu+nx)/bs*bs*cnx+(nu+nx)%bs);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii-1], cnz, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, H_r+(ii-1)*nu, H_r+(ii-1)*nu);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
//				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0, 0);
				}
			else
				{
//				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

			}

		}

	// transpose H in the lower triangular
	dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

	return;

	
	}



void d_part_cond_RSQrq(int N, int nx, int nu, double **pBAt, double **b, int diag_hessian, double **pRSQ, double **rq, double **pGamma_x, double **pGamma_u, double **pGamma_b, double *pM, double *pLam, double *diag, double *pBAtL, double *pH_R, double *pH_St, double *pH_Q, double *H_r, double *H_q)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;


	int nz = nx+nu;
	int cnz = (nz+ncl-1)/ncl*ncl;

	if(diag_hessian)
		{

		// final stage N1 = N-1
		ii = N;

		d_copy_mat(nx, 1, rq[ii-1]+nu, 1, H_q, 1);

		d_copy_mat(nu, 1, rq[ii-1], 1, H_r+(ii-1)*nu, 1);

		dgeset_lib(nu, nu, 0.0, 0, pLam, cnz);
		ddiain_lib(nu, pRSQ[ii-1], 0, pLam, cnz);

		dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);
		dgeset_lib((ii-1)*nu, nu, 0.0, 0, pH_R+(ii-1)*nu*bs, cNnu);

		dgeset_lib(nx, nu, 0.0, 0, pH_St+(ii-1)*bs, cNnu);

		dgeset_lib(nx, nx, 0.0, 0, pH_Q, cnx);
		ddiain_lib(nx, pRSQ[ii-1]+nu, 0, pH_Q, cnx);

		// second final stage N1-1
		ii = N-1;
		if(ii>0) // at least another stage
			{

			dsymv_lib(nx, nx, pH_Q, cnx, b[ii-1], 1, H_q, H_q);
			dgemv_n_lib(nz, nx, pBAt[ii-1], cnx, H_q, 1, rq[ii-1], pLam);
			d_copy_mat(nu, 1, pLam, 1, H_r+(ii-1)*nu, 1);
			d_copy_mat(nx, 1, pLam+nu, 1, H_q, 1);

			for(jj=0; jj<nx-3; jj+=4)
				{
				pH_Q[jj*cnx+0+(jj+0)*bs] = sqrt(pH_Q[jj*cnx+0+(jj+0)*bs]);
				pH_Q[jj*cnx+1+(jj+1)*bs] = sqrt(pH_Q[jj*cnx+1+(jj+1)*bs]);
				pH_Q[jj*cnx+2+(jj+2)*bs] = sqrt(pH_Q[jj*cnx+2+(jj+2)*bs]);
				pH_Q[jj*cnx+3+(jj+3)*bs] = sqrt(pH_Q[jj*cnx+3+(jj+3)*bs]);
				}
			for(kk=0; kk<nx-jj; kk++)
				{
				pH_Q[jj*cnx+kk+(jj+kk)*bs] = sqrt(pH_Q[jj*cnx+kk+(jj+kk)*bs]);
				}

			dtrmm_nt_u_lib(nz, nx, pBAt[ii-1], cnx, pH_Q, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nz, 1.0, pRSQ[ii-1], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, H_r+(ii-1)*nu, H_r+(ii-1)*nu);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, 0, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0);
				}
			else
				{
				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

			// decrease index
			ii--;

			}
		// first stages 
		for(; ii>0; ii--)
			{	

			dsymv_lib(nx, nx, pH_Q, cnx, b[ii-1], 1, H_q, H_q);
			dgemv_n_lib(nz, nx, pBAt[ii-1], cnx, H_q, 1, rq[ii-1], pLam);
			d_copy_mat(nu, 1, pLam, 1, H_r+(ii-1)*nu, 1);
			d_copy_mat(nx, 1, pLam+nu, 1, H_q, 1);

			dpotrf_lib_old(nx, nx, pH_Q, cnx, pH_Q, cnx, diag);
			dtrtr_l_lib(nx, 0, pH_Q, cnx, 0, pH_Q, cnx);	

			dtrmm_nt_u_lib(nz, nx, pBAt[ii-1], cnx, pH_Q, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nz, 1.0, pRSQ[ii-1], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, H_r+(ii-1)*nu, H_r+(ii-1)*nu);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, 0, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0);
				}
			else
				{
				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

			}

		}
	else // dense hessian
		{

		// final stage N1 = N-1
		ii = N;

		d_copy_mat(nx, 1, rq[ii-1]+nu, 1, H_q, 1);

		dgecp_lib(nu, nu, 0, pRSQ[ii-1], cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

		if(ii>1)
			{
			dgetr_lib(nx, nu, nu, pRSQ[ii-1]+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
			dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, rq[ii-1], H_r+(ii-1)*nu);
			dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
			dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, 0, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0);
			}
		else
			{
			d_copy_mat(nu, 1, rq[ii-1], 1, H_r+(ii-1)*nu, 1);
			dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
			}

		dgecp_lib(nx, nx, nu, pRSQ[ii-1]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

		// first stages 
		for(ii=N-1; ii>0; ii--)
			{	

			dsymv_lib(nx, nx, pH_Q, cnx, b[ii-1], 1, H_q, H_q);
			dgemv_n_lib(nz, nx, pBAt[ii-1], cnx, H_q, 1, rq[ii-1], pLam);
			d_copy_mat(nu, 1, pLam, 1, H_r+(ii-1)*nu, 1);
			d_copy_mat(nx, 1, pLam+nu, 1, H_q, 1);

			dpotrf_lib_old(nx, nx, pH_Q, cnx, pH_Q, cnx, diag);
			dtrtr_l_lib(nx, 0, pH_Q, cnx, 0, pH_Q, cnx);	

			dtrmm_nt_u_lib(nz, nx, pBAt[ii-1], cnx, pH_Q, cnx, pBAtL, cnx);
			drowad_lib(nx, 1.0, H_q, pBAtL+(nu+nx)/bs*bs*cnx+(nu+nx)%bs);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii-1], cnz, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemv_n_lib(nu, nx, pM, cnx, pGamma_b[ii-2], 1, H_r+(ii-1)*nu, H_r+(ii-1)*nu);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, 0, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0);
				}
			else
				{
				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

			}

		}

	// transpose H in the lower triangular
	dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

	return;

	
	}



void d_part_cond_RSQ(int N, int nx, int nu, double **pBAt, int diag_hessian, double **pRSQ, double **pGamma_x, double **pGamma_u, double *pM, double *pLam, double *diag, double *pBAtL, double *pH_R, double *pH_St, double *pH_Q)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int ii, jj, kk, offset, i_temp;


	int nz = nx+nu;
	int cnz = (nz+ncl-1)/ncl*ncl;

	if(diag_hessian)
		{

		// final stage N1 = N-1
		ii = N;

		dgeset_lib(nu, nu, 0.0, 0, pLam, cnz);
		ddiain_lib(nu, pRSQ[N], 0, pLam, cnz);

		dgecp_lib(nu, nu, 0, pLam, cnz, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);
		dgeset_lib((N-1)*nu, nu, 0.0, 0, pH_R+(N-1)*nu*bs, cNnu);

		dgeset_lib(nx, nu, 0.0, 0, pH_St+(N-1)*bs, cNnu);

		dgeset_lib(nx, nx, 0.0, 0, pH_Q, cnx);
		ddiain_lib(nx, pRSQ[N-1]+nu, 0, pH_Q, cnx);

		// second final stage N1-1
		ii = N-1;
		if(ii>0) // at least another stage
			{

			for(jj=0; jj<nx-3; jj+=4)
				{
				pH_Q[jj*cnx+0+(jj+0)*bs] = sqrt(pH_Q[jj*cnx+0+(jj+0)*bs]);
				pH_Q[jj*cnx+1+(jj+1)*bs] = sqrt(pH_Q[jj*cnx+1+(jj+1)*bs]);
				pH_Q[jj*cnx+2+(jj+2)*bs] = sqrt(pH_Q[jj*cnx+2+(jj+2)*bs]);
				pH_Q[jj*cnx+3+(jj+3)*bs] = sqrt(pH_Q[jj*cnx+3+(jj+3)*bs]);
				}
			for(kk=0; kk<nx-jj; kk++)
				{
				pH_Q[jj*cnx+kk+(jj+kk)*bs] = sqrt(pH_Q[jj*cnx+kk+(jj+kk)*bs]);
				}

			dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pH_Q, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, 0, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0);
				}
			else
				{
				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);
//			dpotrf_lib_old(nx, nx, pP, cnx, pP, cnx, diag);
//			dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

			// decrease index
			ii--;

			}
		// first stages 
		for(; ii>0; ii--)
			{	
			dpotrf_lib_old(nx, nx, pH_Q, cnx, pH_Q, cnx, diag);
			dtrtr_l_lib(nx, 0, pH_Q, cnx, 0, pH_Q, cnx);	

			dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pH_Q, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pLam, cnz, pLam, cnz);
			ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, 0, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0);
				}
			else
				{
				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);
//			dpotrf_lib_old(nx, nx, pP, cnx, pP, cnx, diag);
//			dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

			}

		}
	else
		{

		// final stage N1 = N-1
		ii = N;

		dgecp_lib(nu, nu, 0, pRSQ[N], cnz, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

		if(ii>1)
			{
			dgetr_lib(nx, nu, nu, pRSQ[N]+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
			dgemm_nt_lib((N-1)*nu, nu, nx, pGamma_u[N-2], cnx, pM, cnx, 0, pH_R+(N-1)*nu*bs, cNnu, pH_R+(N-1)*nu*bs, cNnu, 0, 0);
			dgemm_nt_lib(nx, nu, nx, pGamma_x[N-2], cnx, pM, cnx, 0, pH_St+(N-1)*nu*bs, cNnu, pH_St+(N-1)*nu*bs, cNnu, 0, 0);
			}
		else
			{
			dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
			}

		dgecp_lib(nx, nx, nu, pRSQ[N]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);

		// first stages 
		for(ii=N-1; ii>0; ii--)
			{	
			dpotrf_lib_old(nx, nx, pH_Q, cnx, pH_Q, cnx, diag);
			dtrtr_l_lib(nx, 0, pH_Q, cnx, 0, pH_Q, cnx);	

			dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pH_Q, cnx, pBAtL, cnx);
			dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii], cnz, pLam, cnz);
			dtrtr_l_lib(nu, 0, pLam, cnz, 0, pLam, cnz);	

			dgecp_lib(nu, nu, 0, pLam, cnz, (ii-1)*nu, pH_R+((ii-1)*nu)/bs*bs*cNnu+((ii-1)*nu)%bs+(ii-1)*nu*bs, cNnu);

			if(ii>1)
				{
				dgetr_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pM, cnx);
				dgemm_nt_lib((ii-1)*nu, nu, nx, pGamma_u[ii-2], cnx, pM, cnx, 0, pH_R+(ii-1)*nu*bs, cNnu, pH_R+(ii-1)*nu*bs, cNnu, 0, 0);
				dgemm_nt_lib(nx, nu, nx, pGamma_x[ii-2], cnx, pM, cnx, 0, pH_St+(ii-1)*nu*bs, cNnu, pH_St+(ii-1)*nu*bs, cNnu, 0, 0);
				}
			else
				{
				dgecp_lib(nx, nu, nu, pLam+nu/bs*bs*cnz+nu%bs, cnz, 0, pH_St, cNnu);
				}

			dgecp_lib(nx, nx, nu, pLam+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pH_Q, cnx);
//			dpotrf_lib_old(nx, nx, pP, cnx, pP, cnx, diag);
//			dtrtr_l_lib(nx, 0, pP, cnx, 0, pP, cnx);	

			}

		}

	// transpose H in the lower triangular
	dtrtr_u_lib(N*nu, 0, pH_R, cNnu, 0, pH_R, cNnu);

	return;

	
	}



int d_cond_lqcp_work_space(int N, int nx, int nu, int N2, int alg)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int nz = nu+nx;
	int pnx = (nx+bs-1)/bs*bs;
	int pnu = (nu+bs-1)/bs*bs;
	int pnz = (nz+bs-1)/bs*bs;
	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cnz = (nz+ncl-1)/ncl*ncl;

	// find problem size
	int N1 = N/N2; // floor
	int R1 = N - N2*N1; // the first r1 stages are of size N1+1
	int M1 = R1>0 ? N1+1 : N1;

	int work_space_size = 0;

	int jj;

	if(alg==0 || alg==1)
		{
		for(jj=0; jj<M1; jj++)
			{
			work_space_size += 3*pnx*cnx + 3*((jj+2)*nu+bs-1)/bs*bs*cnx + 2*pnx;
			}
		work_space_size += pnx*cnx + pnx + pnu*cnu + pnu*cnx;
		return work_space_size;
		}

	if(alg==2)
		{
		for(jj=0; jj<M1; jj++)
			{
			work_space_size += 1*pnx*cnx + 1*((jj+2)*nu+bs-1)/bs*bs*cnx + 2*pnx;
			}
		work_space_size += pnx + pnz*cnz + pnz*cnx + pnu*cnx;
		return work_space_size;
		}

	}



void d_cond_lqcp(int N, int nx, int nu, int alg, double **hpA, double **hpAt, double **hpBt, double **hb, double **hpBAt, int diag_hessian, double **hpQ, double **hpS, double **hpR, double **hr, double **hq, double **hpRSQ, double **hrq, int N2, int *nx2, int *nu2, double **hpA2, double **hpB2, double **hb2, double **hpR2, double **hpSt2, double **hpQ2, double **hr2, double **hq2, double *work_double)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int pnx = (nx+bs-1)/bs*bs;
	int pnu = (nu+bs-1)/bs*bs;
	int cnx = (nx+ncl-1)/ncl*ncl;
	int cnu = (nu+ncl-1)/ncl*ncl;

	// find problem size
	int N1 = N/N2; // floor
	int R1 = N - N2*N1; // the first r1 stages are of size N1+1
	int M1 = R1>0 ? N1+1 : N1;
	int T1;

	int ii, jj, nn;

	if(alg==0 || alg==1) // using N^3 n_x^2 or N^2 n_x^2 condensing algorithm
		{

		int use_Gamma_x_Q = 0;
		if(alg==0 && diag_hessian==0)
			use_Gamma_x_Q = 1;

		double *hpGamma_x[M1];
		double *hpGamma_x_Q[M1];
		double *hpGamma_u[M1];
		double *hpGamma_u_Q[M1];
		double *hpGamma_u_Q_A[M1];
		double *hGamma_b[M1];
		double *hGamma_b_q[M1];
		double *hpL[M1+1];
		double *pD;
		double *pM;
		double *work;

		double *ptr;
		ptr = work_double;

		for(jj=0; jj<M1; jj++)
			{
			hpGamma_x[jj] = ptr;
			ptr += pnx*cnx;
			}

		for(jj=0; jj<M1; jj++)
			{
			hpGamma_x_Q[jj] = ptr;
			ptr += pnx*cnx;
			}

		for(jj=0; jj<M1; jj++)
			{
			hpGamma_u[jj] = ptr;
			ptr += ((jj+1)*nu+bs-1)/bs*bs*cnx;
			}

		for(jj=0; jj<M1-1; jj++)
			{
			hpGamma_u_Q[jj] = ptr;
			ptr += ((jj+2)*nu+bs-1)/bs*bs*cnx;
			}
		jj = M1-1;
		hpGamma_u_Q[jj] = ptr;
		ptr += ((jj+1)*nu+bs-1)/bs*bs*cnx;

		for(jj=0; jj<M1-1; jj++)
			{
			hpGamma_u_Q_A[jj] = ptr;
			ptr += ((jj+2)*nu+bs-1)/bs*bs*cnx;
			}
		jj = M1-1;
		hpGamma_u_Q_A[jj] = ptr;
		ptr += ((jj+1)*nu+bs-1)/bs*bs*cnx;
		
		for(jj=0; jj<=M1; jj++)
			{
			hpL[jj] = ptr;
			if(diag_hessian)
				ptr += pnx;
			else
				ptr += pnx*cnx;
			}

		for(jj=0; jj<M1; jj++)
			{
			hGamma_b[jj] = ptr;
			ptr += pnx;
			}

		for(jj=0; jj<M1; jj++)
			{
			hGamma_b_q[jj] = ptr;
			ptr += pnx;
			}
		
		pD = ptr;
		ptr += pnu*cnu;

		pM = ptr;
		ptr += pnu*cnx;

		work = ptr;
		ptr += pnx;

		double *dummy;
		double **pdummy;


		// first stage
		nn = 0;
		jj = 0;

		T1 = jj<R1 ? M1 : N1;

		nx2[jj] = 0;
		nu2[jj] = T1*nu;


		// condense dynamic system
		//d_cond_A(T1, nx, nu, hpA+nn, 1, hpGamma_x, hpA2[jj]);

		d_cond_B(T1, nx, nu, hpA+nn, hpBt+nn, 1, hpGamma_u, hpB2[jj]);

		d_cond_b(T1, nx, nu, hpA+nn, hb+nn, 1, hGamma_b, hb2[jj]);


		// condense cost function
		//d_cond_Q(T1, nx, nu, hpA+nn, diag_Q, 0, hpQ+nn, hpL, 0, hpGamma_x, hpGamma_x_Q, hpQ2[jj], work);
		
		d_cond_R(T1, nx, nu, alg, hpA+nn, hpAt+nn, hpBt+nn, pdummy, diag_hessian, 0, hpQ+nn, 0, hpL, hpS+nn, hpR+nn, pdummy, pD, pM, dummy, dummy, dummy, dummy, 0, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, hpR2[jj]);

		//d_cond_St(T1, nx, nu, nzero_S, hpS+nn, 0, hpGamma_x, use_Gamma_x_Q, hpGamma_x_Q, hpGamma_u_Q, hpSt2[jj]);

		//d_cond_q(T1, nx, nu, hpA+nn, hb+nn, diag_Q, 0, hpQ+nn, hq+nn, hpGamma_x, 0, hGamma_b, 1, hGamma_b_q, hq2[jj]);

		d_cond_r(T1, nx, nu, hpA+nn, hb+nn, diag_hessian, 0, hpQ+nn, hpS+nn, hq+nn, hr+nn, hpGamma_u, 0, hGamma_b, 1, hGamma_b_q, hr2[jj]);


		// increment stage counter
		nn += T1;
		jj++;


		// general stages
		for(; jj<N2; jj++)
			{

			T1 = jj<R1 ? M1 : N1;

			nx2[jj] = nx;
			nu2[jj] = T1*nu;


			// condense dynamic system
			d_cond_A(T1, nx, nu, hpA+nn, 1, hpGamma_x, hpA2[jj]);

			d_cond_B(T1, nx, nu, hpA+nn, hpBt+nn, 1, hpGamma_u, hpB2[jj]);

			d_cond_b(T1, nx, nu, hpA+nn, hb+nn, 1, hGamma_b, hb2[jj]);


			// condense cost function
			d_cond_Q(T1, nx, nu, hpA+nn, diag_hessian, 0, hpQ+nn, hpL, 0, hpGamma_x, hpGamma_x_Q, hpQ2[jj], work);
			
			d_cond_R(T1, nx, nu, alg, hpA+nn, hpAt+nn, hpBt+nn, pdummy, diag_hessian, 0, hpQ+nn, 1, hpL, hpS+nn, hpR+nn, pdummy, pD, pM, dummy, dummy, dummy, dummy, 0, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, hpR2[jj]);

			d_cond_St(T1, nx, nu, diag_hessian, hpS+nn, 0, hpGamma_x, use_Gamma_x_Q, hpGamma_x_Q, hpGamma_u_Q, hpSt2[jj]);

			d_cond_q(T1, nx, nu, hpA+nn, hb+nn, diag_hessian, 0, hpQ+nn, hq+nn, hpGamma_x, 0, hGamma_b, 1, hGamma_b_q, hq2[jj]);

			d_cond_r(T1, nx, nu, hpA+nn, hb+nn, diag_hessian, 0, hpQ+nn, hpS+nn, hq+nn, hr+nn, hpGamma_u, 0, hGamma_b, 0, hGamma_b_q, hr2[jj]);


			// increment stage counter
			nn += T1;

			}

			return;

		}
	if(alg==2) // using N^2 n_x^3 condensing algorithm
		{

#if 1

		int nz = nu+nx;
		int pnz = (nz+bs-1)/bs*bs;
		int cnz = (nz+ncl-1)/ncl*ncl;

		double *hpGamma_x[M1];
		double *hpGamma_u[M1];
		double *hpGamma_u_Q[M1]; // TODO remove !!!!!
		double *hpGamma_u_Q_A[M1]; // TODO remove !!!!!
		double *hGamma_b[M1];
		double *hGamma_b_q[M1];
		double *pBAtL;
		double *pLam;
		double *pM;
		double *diag;

		double *ptr;
		ptr = work_double;

		for(jj=0; jj<M1; jj++)
			{
			hpGamma_x[jj] = ptr;
			ptr += pnx*cnx;
			}

		for(jj=0; jj<M1; jj++)
			{
			hpGamma_u[jj] = ptr;
			ptr += ((jj+1)*nu+bs-1)/bs*bs*cnx;
			}

#if 0
		for(jj=0; jj<M1-1; jj++)
			{
			hpGamma_u_Q[jj] = ptr;
			ptr += ((jj+2)*nu+bs-1)/bs*bs*cnx;
			}
		jj = M1-1;
		hpGamma_u_Q[jj] = ptr;
		ptr += ((jj+1)*nu+bs-1)/bs*bs*cnx;

		for(jj=0; jj<M1-1; jj++)
			{
			hpGamma_u_Q_A[jj] = ptr;
			ptr += ((jj+2)*nu+bs-1)/bs*bs*cnx;
			}
		jj = M1-1;
		hpGamma_u_Q_A[jj] = ptr;
		ptr += ((jj+1)*nu+bs-1)/bs*bs*cnx;
#endif

		pBAtL = ptr;
		ptr += pnz*cnx;

		pLam = ptr;
		ptr += pnz*cnz;

		pM = ptr;
		ptr += pnu*cnx;

		for(jj=0; jj<M1; jj++)
			{
			hGamma_b[jj] = ptr;
			ptr += pnx;
			}

		for(jj=0; jj<M1; jj++)
			{
			hGamma_b_q[jj] = ptr;
			ptr += pnx;
			}
		
		diag = ptr;
		ptr += pnx;

		double *dummy;
		double **pdummy;


		// first stage
		nn = 0;
		jj = 0;

#if 1
		T1 = jj<R1 ? M1 : N1;

		nx2[jj] = 0;
		nu2[jj] = T1*nu;


		// condense dynamic system
		//d_cond_A(T1, nx, nu, hpA+nn, 1, hpGamma_x, hpA2[jj]);

		d_cond_B(T1, nx, nu, hpA+nn, hpBt+nn, 1, hpGamma_u, hpB2[jj]);

		d_cond_b(T1, nx, nu, hpA+nn, hb+nn, 1, hGamma_b, hb2[jj]);


		// condense cost function
		//d_cond_Q(T1, nx, nu, hpA+nn, diag_Q, 0, hpQ+nn, hpL, 0, hpGamma_x, hpGamma_x_Q, hpQ2[jj], diag);
		
		//d_cond_R(T1, nx, nu, 0, hpA+nn, hpAt+nn, hpBt+nn, pdummy, diag_hessian, 0, hpQ+nn, 0, pdummy, hpS+nn, hpR+nn, pdummy, pLam, pM, dummy, dummy, dummy, dummy, 0, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, hpR2[jj]);

		//d_cond_St(T1, nx, nu, nzero_S, hpS+nn, 0, hpGamma_x, use_Gamma_x_Q, hpGamma_x_Q, hpGamma_u_Q, hpSt2[jj]);

		//d_cond_q(T1, nx, nu, hpA+nn, hb+nn, diag_Q, 0, hpQ+nn, hq+nn, hpGamma_x, 0, hGamma_b, 1, hGamma_b_q, hq2[jj]);

		//d_cond_r(T1, nx, nu, hpA+nn, hb+nn, diag_hessian, 0, hpQ+nn, hpS+nn, hq+nn, hr+nn, hpGamma_u, 0, hGamma_b, 1, hGamma_b_q, hr2[jj]);

		d_part_cond_RQrq(T1, nx, nu, hpBAt+nn, hb+nn, diag_hessian, hpRSQ+nn, hrq+nn, hpGamma_x, hpGamma_u, hGamma_b, pM, pLam, diag, pBAtL, hpR2[jj], hpSt2[jj], hpQ2[jj], hr2[jj], hq2[jj]); // XXX no S !!!!!

		// increment stage counter
		nn += T1;
		jj++;
#endif


		// general stages
		for(; jj<N2; jj++)
			{

			T1 = jj<R1 ? M1 : N1;

			nx2[jj] = nx;
			nu2[jj] = T1*nu;


			// condense dynamic system
			d_cond_A(T1, nx, nu, hpA+nn, 1, hpGamma_x, hpA2[jj]);

			d_cond_B(T1, nx, nu, hpA+nn, hpBt+nn, 1, hpGamma_u, hpB2[jj]);

			d_cond_b(T1, nx, nu, hpA+nn, hb+nn, 1, hGamma_b, hb2[jj]);


			// condense cost function
			//d_cond_Q(T1, nx, nu, hpA+nn, diag_hessian, 0, hpQ+nn, hpL, 0, hpGamma_x, hpGamma_x_Q, hpQ2[jj], diag);
			
			//d_cond_R(T1, nx, nu, alg, hpA+nn, hpAt+nn, hpBt+nn, pdummy, diag_hessian, 0, hpQ+nn, 1, hpL, hpS+nn, hpR+nn, pdummy, pD, pM, dummy, dummy, dummy, dummy, 0, hpGamma_u, hpGamma_u_Q, hpGamma_u_Q_A, hpR2[jj]);

			//d_cond_St(T1, nx, nu, diag_hessian, hpS+nn, 0, hpGamma_x, use_Gamma_x_Q, hpGamma_x_Q, hpGamma_u_Q, hpSt2[jj]);

			//d_cond_q(T1, nx, nu, hpA+nn, hb+nn, diag_hessian, 0, hpQ+nn, hq+nn, hpGamma_x, 0, hGamma_b, 1, hGamma_b_q, hq2[jj]);

			//d_cond_r(T1, nx, nu, hpA+nn, hb+nn, diag_hessian, 0, hpQ+nn, hpS+nn, hq+nn, hr+nn, hpGamma_u, 0, hGamma_b, 0, hGamma_b_q, hr2[jj]);

			d_part_cond_RSQrq(T1, nx, nu, hpBAt+nn, hb+nn, diag_hessian, hpRSQ+nn, hrq+nn, hpGamma_x, hpGamma_u, hGamma_b, pM, pLam, diag, pBAtL, hpR2[jj], hpSt2[jj], hpQ2[jj], hr2[jj], hq2[jj]);


			// increment stage counter
			nn += T1;

			}

			return;


#endif

		}

	}



void d_cond_fact_R(int N, int nx, int nu, int nx2_fact, double **pA, double **pAt, double **pBt, int diag_hessian, double **pQ, double **pS, double **pR, double *pQs, double *pM, double *pD, int compute_Gamma_u, double **pGamma_u, double **pGamma_w, double *diag, double **pBAt, double **pRSQ, double *pL, double *pBAtL, double *pH_R)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int cnx = (nx+ncl-1)/ncl*ncl;
	int pnu = (nu+bs-1)/bs*bs;
	int pnx = (nx+bs-1)/bs*bs;
	int cnu = (nu+ncl-1)/ncl*ncl;
	int cNnu = (N*nu+ncl-1)/ncl*ncl;
	//int cNnx = (N*nx+ncl-1)/ncl*ncl;

	int nz = nx+nu;
	int cnz = (nz+ncl-1)/ncl*ncl;
	int cnl = cnz<cnx+ncl ? cnx+ncl : cnz;

	int ii, jj, offset, i_temp;


	if(nx2_fact) // N^2 n_x^2 algorithm
		{

		// Gamma_u
		if(compute_Gamma_u)
			{
			dgecp_lib(nu, nx, 0, pBt[0], cnx, 0, pGamma_u[0], cnx);
			for(ii=1; ii<N; ii++)
				{
				offset = ii*nu;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, ii*nu, nx, pA[ii], cnx, pGamma_u[ii-1], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
				dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
				dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset, pGamma_u[ii]+offset/bs*bs*cnx+offset%bs, cnx);
				}
			}

		if(diag_hessian)
			{

			// last stage
			dgecp_lib(nx, nx, 0, pQ[N], cnx, 0, pQs, cnx);
			dgemm_diag_right_lib(N*nu, nx, pGamma_u[N-1], cnx, pQs, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx);

			// middle stages
			for(ii=N-1; ii>0; ii--)
				{

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_w[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_w[ii], cnx, pAt[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif

				dgecp_lib(nu, nx, ii*nu, pGamma_w[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 0, pD, cnu, pD, cnu, 0, 0);
				ddiaad_lib(nu, 1.0, pR[ii], 0, pD, cnu);
				dgetr_lib(nu, nx, ii*nu, pGamma_w[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pD+pnu*cnu, cnu); // tr?? cp??

				dpotrf_lib_old(pnu+nx, nu, pD, cnu, pD, cnu, diag); // TODO copy last part in the hole
				dgecp_lib(nu, nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
			
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pH_R+(N-1)*nu*bs, cNnu, pH_R+(N-1)*nu*bs, cNnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pH_R+(jj*nu)/bs*bs*cNnu+(jj*nu)%bs+(N-1)*nu*bs, cNnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dgeset_lib(nx, nx, 0.0, 0, pQs, cnx);
				ddiain_lib(nx, pQ[ii], 0, pQs, cnx);
				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQs, cnx, pQs, cnx);
				dtrtr_l_lib(nx, 0, pQs, cnx, 0, pQs, cnx);	
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, ii*nu, nx, pQs, cnx, pGamma_u[ii-1], cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pQs, cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif

				}

			dgecp_lib(nu, nx, ii*nu, pGamma_w[0], cnx, 0, pM, cnx);
			dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[0], cnx, 0, pD, cnu, pD, cnu, 0, 0);
			ddiaad_lib(nu, 1.0, pR[0], 0, pD, cnu);
			dpotrf_lib_old(nu, nu, pD, cnu, pD, cnu, diag);
			dgecp_lib(nu, nu, 0, pD, cnu, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}
		else
			{

			// last stage
			dgecp_lib(nx, nx, 0, pQ[N], cnx, 0, pQs, cnx);
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
			dgemm_nt_lib(nx, N*nu, nx, pQs, cnx, pGamma_u[N-1], cnx, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx, 0, 1);
#else
			dgemm_nt_lib(N*nu, nx, nx, pGamma_u[N-1], cnx, pQs, cnx, 0, pGamma_w[N-1], cnx, pGamma_w[N-1], cnx, 0, 0);
#endif

			// middle stages
			for(ii=N-1; ii>0; ii--)
				{

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, (ii+1)*nu, nx, pAt[ii], cnx, pGamma_w[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib((ii+1)*nu, nx, nx, pGamma_w[ii], cnx, pAt[ii], cnx, 0, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif
				//dgecp_lib(nu, nx, 0, pS[ii], cnx, (ii)*nu, pGamma_w[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);
				dgead_lib(nu, nx, 1.0, 0, pS[ii], cnx, (ii)*nu, pGamma_w[ii-1]+(ii)*nu/bs*bs*cnx+(ii)*nu%bs, cnx);

				dgecp_lib(nu, nx, ii*nu, pGamma_w[ii]+(ii*nu)/bs*bs*cnx+(ii*nu)%bs, cnx, 0, pM, cnx);
				dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[ii], cnx, 1, pR[ii], cnu, pD, cnu, 0, 0);
				dgetr_lib(nu, nx, ii*nu, pGamma_w[ii-1]+(ii*nu)/bs*bs*cnx+ii*nu%bs, cnx, 0, pD+pnu*cnu, cnu); // tr?? cp??

				dpotrf_lib_old(pnu+nx, nu, pD, cnu, pD, cnu, diag); // TODO copy last part in the hole
				dgecp_lib(nu, nu, 0, pD, cnu, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
			
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pH_R+(N-1)*nu*bs, cNnu, pH_R+(N-1)*nu*bs, cNnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pH_R+(jj*nu)/bs*bs*cNnu+(jj*nu)%bs+(N-1)*nu*bs, cNnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dsyrk_nt_lib(nx, nx, nu, pD+pnu*cnu, cnu, pD+pnu*cnu, cnu, -1, pQ[ii], cnx, pQs, cnx);
				dtrtr_l_lib(nx, 0, pQs, cnx, 0, pQs, cnx);	
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, ii*nu, nx, pQs, cnx, pGamma_u[ii-1], cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 1, 1);
#else
				dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pQs, cnx, 1, pGamma_w[ii-1], cnx, pGamma_w[ii-1], cnx, 0, 0);
#endif

				}

			dgecp_lib(nu, nx, ii*nu, pGamma_w[0], cnx, 0, pM, cnx);
			dgemm_nt_lib(nu, nu, nx, pM, cnx, pBt[0], cnx, 1, pR[0], cnu, pD, cnu, 0, 0);
			dpotrf_lib_old(nu, nu, pD, cnu, pD, cnu, diag);
			dgecp_lib(nu, nu, 0, pD, cnu, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}

		}
	else // N^2 n_x^3 algorithm
		{

		// Gamma_u
		if(compute_Gamma_u)
			{
			dgecp_lib(nu, nx, 0, pBt[0], cnx, 0, pGamma_u[0], cnx);
			for(ii=1; ii<N-1; ii++)
				{
				offset = ii*nu;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_C99_4X4)
				dgemm_nt_lib(nx, ii*nu, nx, pA[ii], cnx, pGamma_u[ii-1], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 1); // (A * Gamma_u^T)^T
#else
				dgemm_nt_lib(ii*nu, nx, nx, pGamma_u[ii-1], cnx, pA[ii], cnx, 0, pGamma_u[ii], cnx, pGamma_u[ii], cnx, 0, 0); // Gamma_u * A^T
#endif
				dgecp_lib(nu, nx, 0, pBt[ii], cnx, offset, pGamma_u[ii]+offset/bs*bs*cnx+offset%bs, cnx);
				}
			}

		if(diag_hessian)
			{

			// final stage 
			dgeset_lib(nx, nx, 0.0, 0, pL, cnl);
			ddiain_sqrt_lib(nx, pRSQ[N]+nu, 0, pL, cnl);

			dtrtr_l_lib(nx, 0, pL, cnl, 0, pL+(ncl)*bs, cnl);	

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
				if(nz<128)
					{
					dgeset_lib(nz, nz, 0.0, 0, pL, cnl);
					ddiain_lib(nz, pRSQ[ii], 0, pL, cnl);
					dsyrk_dpotrf_lib_old(nz, nz, nx, pBAtL, cnx, 1, pL, cnl, pL, cnl, diag, 0);
					}
				else
					{
					dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 0, pL, cnl, pL, cnl);
					ddiaad_lib(nz, 1.0, pRSQ[ii], 0, pL, cnl);
					dpotrf_lib_old(nz, nz, pL, cnl, pL, cnl, diag);
					}

				dgecp_lib(nu, nu, 0, pL, cnl, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
				dgecp_lib(nx, nu, nu, pL+nu/bs*bs*cnl+nu%bs, cnl, 0, pD+pnu*cnu, cnu);
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pH_R+(N-1)*nu*bs, cNnu, pH_R+(N-1)*nu*bs, cNnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pH_R+(jj*nu)/bs*bs*cNnu+(jj*nu)%bs+(N-1)*nu*bs, cNnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dtrtr_l_lib(nx, nu, pL+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, pL+(ncl)*bs, cnl);	
				}

			// first stage 
			dtrmm_nt_u_lib(nu, nx, pBAt[0], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
			dgeset_lib(nu, nu, 0.0, 0, pL, cnl);
			ddiain_lib(nu, pRSQ[0], 0, pL, cnl);
			dsyrk_dpotrf_lib_old(nu, nu, nx, pBAtL, cnx, 1, pL, cnl, pL, cnl, diag, 0);

			dgecp_lib(nu, nu, 0, pL, cnl, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}
		else
			{

			// final stage 
			dgecp_lib(nx, nx, nu, pRSQ[N]+nu/bs*bs*cnz+nu%bs+nu*bs, cnz, 0, pL, cnl);
			dpotrf_lib_old(nx, nx, pL, cnl, pL, cnl, diag);

			dtrtr_l_lib(nx, 0, pL, cnl, 0, pL+(ncl)*bs, cnl);	

			// middle stages 
			for(ii=N-1; ii>0; ii--)
				{	
				dtrmm_nt_u_lib(nz, nx, pBAt[ii], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
				if(nz<128)
					{
					dsyrk_dpotrf_lib_old(nz, nz, nx, pBAtL, cnx, 1, pRSQ[ii], cnz, pL, cnl, diag, 0);
					}
				else
					{
					dsyrk_nt_lib(nz, nz, nx, pBAtL, cnx, pBAtL, cnx, 1, pRSQ[ii], cnz, pL, cnl);
					dpotrf_lib_old(nz, nz, pL, cnl, pL, cnl, diag);
					}

				dgecp_lib(nu, nu, 0, pL, cnl, (N-1-ii)*nu, pH_R+((N-1-ii)*nu)/bs*bs*cNnu+((N-1-ii)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
				dgecp_lib(nx, nu, nu, pL+nu/bs*bs*cnl+nu%bs, cnl, 0, pD+pnu*cnu, cnu);
				dgemm_nn_lib((ii)*nu, nu, nx, pGamma_u[ii-1], cnx, pD+pnu*cnu, cnu, 0, pH_R+(N-1)*nu*bs, cNnu, pH_R+(N-1)*nu*bs, cNnu, 0, 0);
				for(jj=0; jj<ii; jj++)
					{
					dgecp_lib(nu, nu, jj*nu, pH_R+(jj*nu)/bs*bs*cNnu+(jj*nu)%bs+(N-1)*nu*bs, cNnu, (N-1-jj)*nu, pH_R+((N-1-jj)*nu)/bs*bs*cNnu+((N-1-jj)*nu)%bs+(N-1-ii)*nu*bs, cNnu);
					}

				dtrtr_l_lib(nx, nu, pL+(nu/bs)*bs*cnl+nu%bs+nu*bs, cnl, 0, pL+(ncl)*bs, cnl);	
				}

			// first stage 
			dtrmm_nt_u_lib(nz, nx, pBAt[0], cnx, pL+(ncl)*bs, cnl, pBAtL, cnx);
			dsyrk_dpotrf_lib_old(nz, nu, nx, pBAtL, cnx, 1, pRSQ[0], cnz, pL, cnl, diag, 0);

			dgecp_lib(nu, nu, 0, pL, cnl, (N-1)*nu, pH_R+((N-1)*nu)/bs*bs*cNnu+((N-1)*nu)%bs+(N-1)*nu*bs, cNnu);

			}

		}

	}



	
