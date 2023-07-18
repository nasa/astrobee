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
#include "../include/block_size.h"
#include "../include/lqcp_aux.h"
#include "../include/d_blas_aux.h"

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>
#endif
//#else
#include "../include/blas_d.h"
//#endif



int d_forward_schur_trf_tv(int N, int *nv, int *ne, double reg, int *diag_hessian, double **hpQA, double **hpLA, double **hdLA, double **hpLe, double *work)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii, jj, ll;	


	int neM = 0;
	for(ii=0; ii<=N; ii++) if(ne[ii]>neM) neM = ne[ii];
	int pneM = (neM+bs-1)/bs*bs;
	int cneM = (neM+ncl-1)/ncl*ncl;

	double *hpLe_tmp = work; 
	work += pneM*cneM;
	double *hdLe_tmp = work; 
	work += pneM;


	float diag_min;
	diag_min = 1.0;


	int nv0, ne0, ne1, nve0, pnv0, pne0, cnv0, cne0, cne1;

	int nx0, nu0, ncp;


	// first stage
	ii = 0;
	nv0 = nv[0];
	ne0 = ne[0];
	nve0 = nv0 + ne0;
	pnv0 = (nv0+bs-1)/bs*bs;
	pne0 = (ne0+bs-1)/bs*bs;
	cnv0 = (nv0+ncl-1)/ncl*ncl;
	cne0 = (ne0+ncl-1)/ncl*ncl;

	if(diag_hessian[0])
		{

		dgeset_lib(pnv0, nv0, 0.0, 0, hpLA[ii], cnv0);
		for(jj=0; jj<nv0; jj++) hdLA[ii][jj] = sqrt(hpQA[ii][jj]+reg);
#ifdef BLASFEO
		ddiain_lib(nv0, 1.0, hdLA[ii], 0, hpLA[ii], cnv0);
#else
		ddiain_lib(nv0, hdLA[ii], 0, hpLA[ii], cnv0);
#endif
		for(jj=0; jj<nv0; jj++) hdLA[ii][jj] = 1.0/hdLA[ii][jj];

#ifdef BLASFEO
		dgemm_diag_right_lib(ne0, nv0, 1.0, hpQA[ii]+pnv0, cnv0, hdLA[ii], 0.0, hpLA[ii]+pnv0*cnv0, cnv0, hpLA[ii]+pnv0*cnv0, cnv0);
#else
		dgemm_diag_right_lib(ne0, nv0, hpQA[ii]+pnv0, cnv0, hdLA[ii], 0, hpLA[ii]+pnv0*cnv0, cnv0, hpLA[ii]+pnv0*cnv0, cnv0);
#endif

		}
	else
		{

#ifdef BLASFEO
		dgecp_lib(pnv0+pne0, nv0, 1.0, 0, hpQA[ii], cnv0, 0, hpLA[ii], cnv0); // copy entire panels
#else
		dgecp_lib(pnv0+pne0, nv0, 0, hpQA[ii], cnv0, 0, hpLA[ii], cnv0); // copy entire panels
#endif

		// copy the lower part of A in the padd space
		if(ne0>pnv0-nv0)
#ifdef BLASFEO
			dgecp_lib(pnv0-nv0, nv0, 1.0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#else
			dgecp_lib(pnv0-nv0, nv0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#endif
		else
#ifdef BLASFEO
			dgecp_lib(ne0, nv0, 1.0, 0, hpLA[ii]+pnv0*cnv0, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#else
			dgecp_lib(ne0, nv0, 0, hpLA[ii]+pnv0*cnv0, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#endif

		// regularize 
		ddiareg_lib(nv0, reg, 0, hpLA[ii], cnv0);

		// assume that A is aligned to a panel boundary, and that the lower part of A is copied between Q and A
#ifdef BLASFEO
		dpotrf_nt_l_lib(nve0, nv0, hpLA[ii], cnv0, hpLA[ii], cnv0, hdLA[ii]);
#else
		dpotrf_lib(nve0, nv0, hpLA[ii], cnv0, hpLA[ii], cnv0, hdLA[ii]);
#endif

		for(jj=0; jj<nv0; jj++) 
			diag_min = fmin(diag_min, hdLA[ii][jj]);

		// copy back the lower part of A
		if(ne0>pnv0-nv0)
#ifdef BLASFEO
			dgecp_lib(pnv0-nv0, nv0, 1.0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0);
#else
			dgecp_lib(pnv0-nv0, nv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0);
#endif
		else
#ifdef BLASFEO
			dgecp_lib(ne0, nv0, 1.0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
#else
			dgecp_lib(ne0, nv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
#endif

		}
		
	dgeset_lib(ne0, ne0, 0.0, 0, hpLe_tmp, cne0);
	ddiareg_lib(ne0, reg, 0, hpLe_tmp, cne0);
#ifdef BLASFEO
	dsyrk_dpotrf_nt_l_lib(ne0, ne0, nv0, hpLA[ii]+pnv0*cnv0, cnv0, hpLA[ii]+pnv0*cnv0, cnv0, hpLe_tmp, cne0, hpLe_tmp, cne0, hdLe_tmp);
#else
	dsyrk_dpotrf_lib(ne0, ne0, nv0, hpLA[ii]+pnv0*cnv0, cnv0, hpLA[ii]+pnv0*cnv0, cnv0, 1, hpLe_tmp, cne0, hpLe_tmp, cne0, hdLe_tmp);
#endif

	for(jj=0; jj<ne0; jj++) 
		diag_min = fmin(diag_min, hdLe_tmp[jj]);

	dtrtri_lib(ne0, hpLe_tmp, cne0, 1, hdLe_tmp, hpLe[ii], cne0);



	// middle stages
	for(ii=1; ii<N; ii++)
		{

		ne1  = ne0;
		cne1 = cne0;
		nv0  = nv[ii];
		ne0  = ne[ii];
		nve0 = nv0 + ne0;
		pnv0 = (nv0+bs-1)/bs*bs;
		pne0 = (ne0+bs-1)/bs*bs;
		cnv0 = (nv0+ncl-1)/ncl*ncl;
		cne0 = (ne0+ncl-1)/ncl*ncl;

		if(diag_hessian[ii])
			{

			nx0 = ne1;
			nu0 = nv0 - nx0;

			dlauum_lib(ne1, hpLe[ii-1], cne1, hpLe[ii-1], cne1, 0, hpLA[ii], cnv0, hpLA[ii], cnv0);

			ddiaad_lib(nx0, 1.0, hpQA[ii], 0, hpLA[ii], cnv0);

			ddiareg_lib(nx0, reg, 0, hpLA[ii], cnv0);

//			d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);

			// copy the lower part of A in the padd space
//			printf("\n%d %d %d\n", ne0, pnv0, nx0);
			if(ne0>pnv0-nx0)
				{
				ncp = ne0-pnv0+nx0;
#ifdef BLASFEO
				dgecp_lib(ncp, nx0, 1.0, 0, hpQA[ii]+pnv0, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
				dgecp_lib(pnv0-nx0, 1.0, nx0, ncp, hpQA[ii]+pnv0+ncp/bs*bs*cnv0+ncp%bs, cnv0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0);
#else
				dgecp_lib(ncp, nx0, 0, hpQA[ii]+pnv0, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
				dgecp_lib(pnv0-nx0, nx0, ncp, hpQA[ii]+pnv0+ncp/bs*bs*cnv0+ncp%bs, cnv0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0);
#endif
//				d_print_pmat(pne0, cnv0, bs, hpQA[ii]+pnv0, cnv0);
//				d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);
//				exit(2);
				}
			else // copy all A in the pad space
				{
#ifdef BLASFEO
				dgecp_lib(ne0, nx0, 1.0, 0, hpQA[ii]+pnv0, cnv0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0);
#else
				dgecp_lib(ne0, nx0, 0, hpQA[ii]+pnv0, cnv0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0);
#endif
				}

//			d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);

			// assume that A is aligned to a panel boundary, and that the lower part of A is copied between Q and A
#ifdef BLASFEO
			dpotrf_nt_l_lib(2*nx0, nx0, hpLA[ii], cnv0, hpLA[ii], cnv0, hdLA[ii]);
#else
			dpotrf_lib(2*nx0, nx0, hpLA[ii], cnv0, hpLA[ii], cnv0, hdLA[ii]);
#endif

			for(jj=0; jj<nx0; jj++) 
				diag_min = fmin(diag_min, hdLA[ii][jj]);

//			d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);

			// copy back the lower part of A
			if(ne0>pnv0-nx0)
#ifdef BLASFEO
				dgecp_lib(pnv0-nx0, nx0, 1.0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0, 2*nx0, hpLA[ii]+2*nx0/bs*bs*cnv0+2*nx0%bs, cnv0);
#else
				dgecp_lib(pnv0-nx0, nx0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0, 2*nx0, hpLA[ii]+2*nx0/bs*bs*cnv0+2*nx0%bs, cnv0);
#endif
			else
#ifdef BLASFEO
				dgecp_lib(ne0, nx0, 1.0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
#else
				dgecp_lib(ne0, nx0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
#endif

//			d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);

			dgeset_lib(nu0, nu0, 0.0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs+nx0*bs, cnv0);
			for(jj=0; jj<nu0; jj++) hdLA[ii][nx0+jj] = sqrt(hpQA[ii][nx0+jj]+reg);
#ifdef BLASFEO
			ddiain_lib(nu0, 1.0, hdLA[ii]+nx0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs+nx0*bs, cnv0);
#else
			ddiain_lib(nu0, hdLA[ii]+nx0, nx0, hpLA[ii]+nx0/bs*bs*cnv0+nx0%bs+nx0*bs, cnv0);
#endif
			for(jj=0; jj<nu0; jj++) hdLA[ii][nx0+jj] = 1.0/hdLA[ii][nx0+jj];
			
//			d_print_mat(1, nv0, hdLA[ii], 1);

#ifdef BLASFEO
			dgemm_diag_right_lib(ne0, nu0, 1.0, hpQA[ii]+pnv0+nx0*bs, cnv0, hdLA[ii]+nx0, 0.0, hpLA[ii]+pnv0*cnv0+nx0*bs, cnv0, hpLA[ii]+pnv0*cnv0+nx0*bs, cnv0);
#else
			dgemm_diag_right_lib(ne0, nu0, hpQA[ii]+pnv0+nx0*bs, cnv0, hdLA[ii]+nx0, 0, hpLA[ii]+pnv0*cnv0+nx0*bs, cnv0, hpLA[ii]+pnv0*cnv0+nx0*bs, cnv0);
#endif

//			d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);

//			exit(1);

			}
		else
			{

#ifdef BLASFEO
			dgecp_lib(pnv0+ne0, nv0, 1.0, 0, hpQA[ii], cnv0, 0, hpLA[ii], cnv0); // copy entire panels
#else
			dgecp_lib(pnv0+ne0, nv0, 0, hpQA[ii], cnv0, 0, hpLA[ii], cnv0); // copy entire panels
#endif

			// copy the lower part of A in the padd space
			if(ne0>pnv0-nv0)
#ifdef BLASFEO
				dgecp_lib(pnv0-nv0, nv0, 1.0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#else
				dgecp_lib(pnv0-nv0, nv0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#endif
			else
#ifdef BLASFEO
				dgecp_lib(ne0, nv0, 1.0, 0, hpLA[ii]+pnv0*cnv0, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#else
				dgecp_lib(ne0, nv0, 0, hpLA[ii]+pnv0*cnv0, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#endif

			// regularize 
			ddiareg_lib(nv0, reg, 0, hpLA[ii], cnv0);

			// assume that A is aligned to a panel boundary, and that the lower part of A is copied between Q and A
#if defined(TARGET_X64_AVX2)
			dlauum_dpotrf_lib(nve0, nv0, ne1, hpLe[ii-1], cne1, hpLe[ii-1], cne1, 1, hpLA[ii], cnv0, hpLA[ii], cnv0, hdLA[ii]);
#else
			dlauum_lib(ne1, hpLe[ii-1], cne1, hpLe[ii-1], cne1, 1, hpLA[ii], cnv0, hpLA[ii], cnv0);
//			d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);
#ifdef BLASFEO
			dpotrf_nt_l_lib(nve0, nv0, hpLA[ii], cnv0, hpLA[ii], cnv0, hdLA[ii]);
#else
			dpotrf_lib(nve0, nv0, hpLA[ii], cnv0, hpLA[ii], cnv0, hdLA[ii]);
#endif
#endif

			for(jj=0; jj<nv0; jj++) 
				diag_min = fmin(diag_min, hdLA[ii][jj]);

			// copy back the lower part of A
			if(ne0>pnv0-nv0)
#ifdef BLASFEO
				dgecp_lib(pnv0-nv0, nv0, 1.0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0);
#else
				dgecp_lib(pnv0-nv0, nv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0);
#endif
			else
#ifdef BLASFEO
				dgecp_lib(ne0, nv0, 1.0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
#else
				dgecp_lib(ne0, nv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0);
#endif

//			d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);
//			exit(1);

			}

		dgeset_lib(ne0, ne0, 0.0, 0, hpLe_tmp, cne0);
		ddiareg_lib(ne0, reg, 0, hpLe_tmp, cne0);
#ifdef BLASFEO
		dsyrk_dpotrf_nt_l_lib(ne0, ne0, nv0, hpLA[ii]+pnv0*cnv0, cnv0, hpLA[ii]+pnv0*cnv0, cnv0, hpLe_tmp, cne0, hpLe_tmp, cne0, hdLe_tmp);
#else
		dsyrk_dpotrf_lib(ne0, ne0, nv0, hpLA[ii]+pnv0*cnv0, cnv0, hpLA[ii]+pnv0*cnv0, cnv0, 1, hpLe_tmp, cne0, hpLe_tmp, cne0, hdLe_tmp);
#endif

		for(jj=0; jj<ne0; jj++) 
			diag_min = fmin(diag_min, hdLe_tmp[jj]);

		dtrtri_lib(ne0, hpLe_tmp, cne0, 1, hdLe_tmp, hpLe[ii], cne0);



		if(diag_min==0.0)
			return ii+1;

		}


	// last stage
	ii = N;
	ne1  = ne0;
	cne1 = cne0;
	nv0  = nv[ii];
	ne0  = ne[ii];
	nve0 = nv0 + ne0;
	pnv0 = (nv0+bs-1)/bs*bs;
	pne0 = (ne0+bs-1)/bs*bs;
	cnv0 = (nv0+ncl-1)/ncl*ncl;
	cne0 = (ne0+ncl-1)/ncl*ncl;


	if(diag_hessian[N])
		{

		dlauum_lib(ne1, hpLe[ii-1], cne1, hpLe[ii-1], cne1, 0, hpLA[ii], cnv0, hpLA[ii], cnv0);

		ddiaad_lib(nv0, 1.0, hpQA[ii], 0, hpLA[ii], cnv0);

#ifdef BLASFEO
		dgecp_lib(ne0, nv0, 1.0, 0, hpQA[ii]+pnv0, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0); // copy entire panels
#else
		dgecp_lib(ne0, nv0, 0, hpQA[ii]+pnv0, cnv0, 0, hpLA[ii]+pnv0*cnv0, cnv0); // copy entire panels
#endif

		}
	else
		{

#ifdef BLASFEO
		dgecp_lib(pnv0+pne0, nv0, 1.0, 0, hpQA[ii], cnv0, 0, hpLA[ii], cnv0); // copy entire panels
#else
		dgecp_lib(pnv0+pne0, nv0, 0, hpQA[ii], cnv0, 0, hpLA[ii], cnv0); // copy entire panels
#endif

		dlauum_lib(ne1, hpLe[ii-1], cne1, hpLe[ii-1], cne1, 1, hpLA[ii], cnv0, hpLA[ii], cnv0);

		}

	// copy the lower part of A in the padd space
	if(ne0>pnv0-nv0)
#ifdef BLASFEO
		dgecp_lib(pnv0-nv0, nv0, 1.0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#else
		dgecp_lib(pnv0-nv0, nv0, nve0, hpLA[ii]+nve0/bs*bs*cnv0+nve0%bs, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#endif
	else
#ifdef BLASFEO
		dgecp_lib(ne0, nv0, 1.0, 0, hpLA[ii]+pnv0*cnv0, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#else
		dgecp_lib(ne0, nv0, 0, hpLA[ii]+pnv0*cnv0, cnv0, nv0, hpLA[ii]+nv0/bs*bs*cnv0+nv0%bs, cnv0);
#endif

	// regularize 
	ddiareg_lib(nv0, reg, 0, hpLA[N], cnv0);

#ifdef BLASFEO
	dpotrf_nt_l_lib(nve0, nv0, hpLA[N], cnv0, hpLA[N], cnv0, hdLA[N]);
#else
	dpotrf_lib(nve0, nv0, hpLA[N], cnv0, hpLA[N], cnv0, hdLA[N]);
#endif

	for(jj=0; jj<nv0; jj++) 
		diag_min = fmin(diag_min, hdLA[N][jj]);

	if(ne0>0)
		{
		// copy back the lower part of A
		if(ne0>pnv0-nv0)
#ifdef BLASFEO
			dgecp_lib(pnv0-nv0, nv0, 1.0, nv0, hpLA[N]+nv0/bs*bs*cnv0+nv0%bs, cnv0, nve0, hpLA[N]+nve0/bs*bs*cnv0+nve0%bs, cnv0);
#else
			dgecp_lib(pnv0-nv0, nv0, nv0, hpLA[N]+nv0/bs*bs*cnv0+nv0%bs, cnv0, nve0, hpLA[N]+nve0/bs*bs*cnv0+nve0%bs, cnv0);
#endif
		else
#ifdef BLASFEO
			dgecp_lib(ne0, nv0, 1.0, nv0, hpLA[N]+nv0/bs*bs*cnv0+nv0%bs, cnv0, 0, hpLA[N]+pnv0*cnv0, cnv0);
#else
			dgecp_lib(ne0, nv0, nv0, hpLA[N]+nv0/bs*bs*cnv0+nv0%bs, cnv0, 0, hpLA[N]+pnv0*cnv0, cnv0);
#endif

#if 1
		dgeset_lib(ne0, ne0, 0.0, 0, hpLe_tmp, cne0);
		ddiareg_lib(ne0, reg, 0, hpLe_tmp, cne0);
#ifdef BLASFEO
		dsyrk_dpotrf_nt_l_lib(ne0, ne0, nv0, hpLA[N]+pnv0*cnv0, cnv0, hpLA[N]+pnv0*cnv0, cnv0, hpLe_tmp, cne0, hpLe_tmp, cne0, hdLe_tmp);
#else
		dsyrk_dpotrf_lib(ne0, ne0, nv0, hpLA[N]+pnv0*cnv0, cnv0, hpLA[N]+pnv0*cnv0, cnv0, 1, hpLe_tmp, cne0, hpLe_tmp, cne0, hdLe_tmp);
#endif

		for(jj=0; jj<ne0; jj++) 
			diag_min = fmin(diag_min, hdLe_tmp[jj]);

		dtrtri_lib(ne0, hpLe_tmp, cne0, 1, hdLe_tmp, hpLe[N], cne0);
#else
		dgeset_lib(ne0, ne0, 0.0, 0, hpLe[N], cne0);
		ddiareg_lib(ne0, reg, 0, hpLe[N], cne0);
#ifdef BLASFEO
		dsyrk_dpotrf_nt_l_lib(ne0, ne0, nv0, hpLA[N]+pnv0*cnv0, cnv0, hpLA[N]+pnv0*cnv0, cnv0, hpLe[N], cne0, hpLe[N], cne0, hdLe_tmp);
#else
		dsyrk_dpotrf_lib(ne0, ne0, nv0, hpLA[N]+pnv0*cnv0, cnv0, hpLA[N]+pnv0*cnv0, cnv0, 1, hpLe[N], cne0, hpLe[N], cne0, hdLe_tmp);
#endif
#endif
		
		}

	if(diag_min==0.0)
		return ii+1;
	
//	d_print_pmat(pnv0+pne0, cnv0, bs, hpLA[ii], cnv0);
//	exit(1);

	return 0;

	}



void d_forward_schur_trs_tv(int N, int *nv, int *ne, int *diag_hessian, double **hqb, double **hpLA, double **hdLA, double **hpLe, double **hxupi, double *tmp)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii, jj, ll;	

	int nv0, ne0, ne1, nve0, pnv0, pnv1, pne0, pne1, cnv0, cne0, cne1;


	int nx0, nu0;


	// forward recursion

	// first stage
	ii = 0;
	nv0  = nv[0];
	ne0  = ne[0];
	nve0 = nv0 + ne0;
	pnv0 = (nv0+bs-1)/bs*bs;
	pne0 = (ne0+bs-1)/bs*bs;
	cnv0 = (nv0+ncl-1)/ncl*ncl;
	cne0 = (ne0+ncl-1)/ncl*ncl;

	d_copy_mat(pnv0+ne0, 1, hqb[ii], 1, hxupi[ii], 1);

	if(diag_hessian[ii])
		{

//		d_print_pmat(nve0, nv0, bs, hpLA[ii], cnv0);
//		d_print_mat(1, pnv0+pne0, hxupi[ii], 1);

		//dtrsv_n_lib(nv0, nv0, hpLA[ii], cnv0, 1, hdLA[ii], hxupi[ii], hxupi[ii]);
		dgemv_diag_lib(nv0, hdLA[ii], hxupi[ii], 0, hxupi[ii], hxupi[ii]); 
#ifdef BLASFEO
		dgemv_n_lib(ne0, nv0, -1.0, hpLA[ii]+pnv0*cnv0, cnv0, hxupi[ii], 1.0, hxupi[ii]+pnv0, hxupi[ii]+pnv0);
#else
		dgemv_n_lib(ne0, nv0, hpLA[ii]+pnv0*cnv0, cnv0, hxupi[ii], -1, hxupi[ii]+pnv0, hxupi[ii]+pnv0);
#endif

		}
	else
		{

		// copy in the pad space
		if(ne0>pnv0-nv0)
			for(jj=0; jj<pnv0-nv0; jj++) hxupi[ii][nv0+jj] = hxupi[ii][nve0+jj];
		else
			for(jj=0; jj<ne0; jj++) hxupi[ii][nv0+jj] = hxupi[ii][pnv0+jj];
		
#ifdef BLASFEO
		dtrsv_ln_inv_lib(nve0, nv0, hpLA[ii], cnv0, hdLA[ii], hxupi[ii], hxupi[ii]);
#else
		dtrsv_n_lib(nve0, nv0, hpLA[ii], cnv0, 1, hdLA[ii], hxupi[ii], hxupi[ii]);
#endif

		// copy back
		if(ne0>pnv0-nv0)
			for(jj=0; jj<pnv0-nv0; jj++) hxupi[ii][nve0+jj] = hxupi[ii][nv0+jj];
		else
			for(jj=0; jj<ne0; jj++) hxupi[ii][pnv0+jj] = hxupi[ii][nv0+jj];

		}
	


	// middle stages
	for(ii=1; ii<=N; ii++)
		{

		ne1  = ne0;
		pnv1 = pnv0;
		pne1 = pne0;
		cne1 = cne0;
		nv0  = nv[ii];
		ne0  = ne[ii];
		nve0 = nv0 + ne0;
		pnv0 = (nv0+bs-1)/bs*bs;
		pne0 = (ne0+bs-1)/bs*bs;
		cnv0 = (nv0+ncl-1)/ncl*ncl;
		cne0 = (ne0+ncl-1)/ncl*ncl;

		d_copy_mat(pnv0+ne0, 1, hqb[ii], 1, hxupi[ii], 1);

#ifdef BLASFEO
		dtrmv_ut_lib(ne1, hpLe[ii-1], cne1, hxupi[ii-1]+pnv1, 0, tmp, tmp);
		dtrmv_un_lib(ne1, hpLe[ii-1], cne1, tmp, -1, hxupi[ii], hxupi[ii]);
#else
		dtrmv_u_t_lib(ne1, hpLe[ii-1], cne1, hxupi[ii-1]+pnv1, 0, tmp);
		dtrmv_u_n_lib(ne1, hpLe[ii-1], cne1, tmp, -1, hxupi[ii]);
#endif

		if(diag_hessian[ii])
			{

			nx0 = ne1;
			nu0 = nv0 - nx0;
			
#ifdef BLASFEO
			dtrsv_ln_inv_lib(nx0, nx0, hpLA[ii], cnv0, hdLA[ii], hxupi[ii], hxupi[ii]);
#else
			dtrsv_n_lib(nx0, nx0, hpLA[ii], cnv0, 1, hdLA[ii], hxupi[ii], hxupi[ii]);
#endif
			dgemv_diag_lib(nu0, hdLA[ii]+nx0, hxupi[ii]+nx0, 0, hxupi[ii]+nx0, hxupi[ii]+nx0); 
#ifdef BLASFEO
			dgemv_n_lib(ne0, nv0, -1.0, hpLA[ii]+pnv0*cnv0, cnv0, hxupi[ii], 1.0, hxupi[ii]+pnv0, hxupi[ii]+pnv0);
#else
			dgemv_n_lib(ne0, nv0, hpLA[ii]+pnv0*cnv0, cnv0, hxupi[ii], -1, hxupi[ii]+pnv0, hxupi[ii]+pnv0);
#endif

			}
		else
			{

			// copy in the pad space
			if(ne0>pnv0-nv0)
				for(jj=0; jj<pnv0-nv0; jj++) hxupi[ii][nv0+jj] = hxupi[ii][nve0+jj];
			else
				for(jj=0; jj<ne0; jj++) hxupi[ii][nv0+jj] = hxupi[ii][pnv0+jj];

#ifdef BLASFEO
			dtrsv_ln_inv_lib(nve0, nv0, hpLA[ii], cnv0, hdLA[ii], hxupi[ii], hxupi[ii]);
#else
			dtrsv_n_lib(nve0, nv0, hpLA[ii], cnv0, 1, hdLA[ii], hxupi[ii], hxupi[ii]);
#endif

			// copy back
			if(ne0>pnv0-nv0)
				for(jj=0; jj<pnv0-nv0; jj++) hxupi[ii][nve0+jj] = hxupi[ii][nv0+jj];
			else
				for(jj=0; jj<ne0; jj++) hxupi[ii][pnv0+jj] = hxupi[ii][nv0+jj];

			}

//		d_print_mat(1, pnv0+pne0, hxupi[ii], 1);
//		exit(1);

		}
	

	// last stage
	ii = N;
#if 1
#ifdef BLASFEO
	dtrmv_ut_lib(ne0, hpLe[N], cne0, hxupi[ii]+pnv0, 0, tmp, tmp);
#else
	dtrmv_u_t_lib(ne0, hpLe[N], cne0, hxupi[ii]+pnv0, 0, tmp);
#endif
#else
	double *dummy;
#ifdef BLASFEO
	dtrsv_ln_lib(ne0, ne0, hpLe[N], cne0, dummy, hxupi[ii]+pnv0, tmp); // not inverted !!!!!
#else
	dtrsv_n_lib(ne0, ne0, hpLe[N], cne0, 0, dummy, hxupi[ii]+pnv0, tmp);
#endif
#endif

	
	// backward recursion
#if 1
#ifdef BLASFEO
	dtrmv_un_lib(ne0, hpLe[N], cne0, tmp, 0, hxupi[ii]+pnv0, hxupi[ii]+pnv0);
#else
	dtrmv_u_n_lib(ne0, hpLe[N], cne0, tmp, 0, hxupi[ii]+pnv0);
#endif
#else
#ifdef BLASFEO
	dtrsv_lt_lib(ne0, ne0, hpLe[N], cne0, dummy, tmp, hxupi[ii]+pnv0); // not inverted !!!!!
#else
	dtrsv_t_lib(ne0, ne0, hpLe[N], cne0, 0, dummy, tmp, hxupi[ii]+pnv0);
#endif
#endif

	// last stage
	for(jj=0; jj<nv0; jj++) hxupi[N][jj] = - hxupi[N][jj];

//	d_print_mat(1, pnv0+pne0, hxupi[ii], 1);

	if(diag_hessian[ii])
		{

#ifdef BLASFEO
		dgemv_t_lib(ne0, nv0, -1.0, hpLA[N]+pnv0*cnv0, cnv0, hxupi[N]+pnv0, 1.0, hxupi[N], hxupi[N]);
		dtrsv_lt_inv_lib(nv0, nv0, hpLA[N], cnv0, hdLA[N], hxupi[N], hxupi[N]);
#else
		dgemv_t_lib(ne0, nv0, hpLA[N]+pnv0*cnv0, cnv0, hxupi[N]+pnv0, -1, hxupi[N], hxupi[N]);
		dtrsv_t_lib(nv0, nv0, hpLA[N], cnv0, 1, hdLA[N], hxupi[N], hxupi[N]);
#endif

		}
	else
		{

		if(ne0>pnv0-nv0)
			for(jj=0; jj<pnv0-nv0; jj++) hxupi[N][nv0+jj] = hxupi[N][nve0+jj];
		else
			for(jj=0; jj<ne0; jj++) hxupi[N][nv0+jj] = hxupi[N][pnv0+jj];

#ifdef BLASFEO
		dtrsv_lt_inv_lib(nve0, nv0, hpLA[N], cnv0, hdLA[N], hxupi[N], hxupi[N]);
#else
		dtrsv_t_lib(nve0, nv0, hpLA[N], cnv0, 1, hdLA[N], hxupi[N], hxupi[N]);
#endif

//		if(ne0>pnv0-nv0)
//			for(jj=0; jj<pnv0-nv0; jj++) hxupi[N][nve0+jj] = hxupi[N][nv0+jj];
//		else
//			for(jj=0; jj<ne0; jj++) hxupi[N][pnv0+jj] = hxupi[N][nv0+jj];

		}
	
//	d_print_mat(1, pnv0+pne0, hxupi[ii], 1);
//	return;
//	exit(1);

//	ne1 = ne[N-1];

	// middle stages
	for(ii=1; ii<N; ii++)
		{
		
		ne0  = ne1;
		ne1  = ne[N-ii-1];//ne0;
//		pnv1 = pnv0;
//		pne1 = pne0;
//		cne1 = cne0;
		nv0  = nv[N-ii];
//		ne0  = ne[N-ii];
		nve0 = nv0 + ne0;
		pnv0 = (nv0+bs-1)/bs*bs;
		pne0 = (ne0+bs-1)/bs*bs;
		cnv0 = (nv0+ncl-1)/ncl*ncl;
		cne0 = (ne0+ncl-1)/ncl*ncl;

		for(jj=0; jj<ne0; jj++) hxupi[N-ii][pnv0+jj] = hxupi[N-ii][pnv0+jj] - hxupi[N-ii+1][jj]; 
#ifdef BLASFEO
		dtrmv_ut_lib(ne0, hpLe[N-ii], cne0, hxupi[N-ii]+pnv0, 0, tmp, tmp);
		dtrmv_un_lib(ne0, hpLe[N-ii], cne0, tmp, 0, hxupi[N-ii]+pnv0, hxupi[N-ii]+pnv0);
#else
		dtrmv_u_t_lib(ne0, hpLe[N-ii], cne0, hxupi[N-ii]+pnv0, 0, tmp);
		dtrmv_u_n_lib(ne0, hpLe[N-ii], cne0, tmp, 0, hxupi[N-ii]+pnv0);
#endif

		for(jj=0; jj<nv0; jj++) hxupi[N-ii][jj] = - hxupi[N-ii][jj];

		if(diag_hessian[N-ii])
			{

			nx0 = ne1;
			nu0 = nv0 - nx0;

#ifdef BLASFEO
			dgemv_t_lib(ne0, nv0, -1.0, hpLA[N-ii]+pnv0*cnv0, cnv0, hxupi[N-ii]+pnv0, 1.0, hxupi[N-ii], hxupi[N-ii]);
			dtrsv_lt_inv_lib(nx0, nx0, hpLA[N-ii], cnv0, hdLA[N-ii], hxupi[N-ii], hxupi[N-ii]);
#else
			dgemv_t_lib(ne0, nv0, hpLA[N-ii]+pnv0*cnv0, cnv0, hxupi[N-ii]+pnv0, -1, hxupi[N-ii], hxupi[N-ii]);
			dtrsv_t_lib(nx0, nx0, hpLA[N-ii], cnv0, 1, hdLA[N-ii], hxupi[N-ii], hxupi[N-ii]);
#endif
			dgemv_diag_lib(nu0, hdLA[N-ii]+nx0, hxupi[N-ii]+nx0, 0, hxupi[N-ii]+nx0, hxupi[N-ii]+nx0); 

			}
		else
			{

			if(ne0>pnv0-nv0)
				for(jj=0; jj<pnv0-nv0; jj++) hxupi[N-ii][nv0+jj] = hxupi[N-ii][nve0+jj];
			else
				for(jj=0; jj<ne0; jj++) hxupi[N-ii][nv0+jj] = hxupi[N-ii][pnv0+jj];

#ifdef BLASFEO
			dtrsv_lt_inv_lib(nve0, nv0, hpLA[N-ii], cnv0, hdLA[N-ii], hxupi[N-ii], hxupi[N-ii]);
#else
			dtrsv_t_lib(nve0, nv0, hpLA[N-ii], cnv0, 1, hdLA[N-ii], hxupi[N-ii], hxupi[N-ii]);
#endif

			}

		}
	
	// first stage
	ii = N;
	ne0  = ne1;
	ne1  = ne[N-ii-1];//ne0;
//	pnv1 = pnv0;
//	pne1 = pne0;
//	cne1 = cne0;
	nv0  = nv[N-ii];
//	ne0  = ne[N-ii];
	nve0 = nv0 + ne0;
	pnv0 = (nv0+bs-1)/bs*bs;
	pne0 = (ne0+bs-1)/bs*bs;
	cnv0 = (nv0+ncl-1)/ncl*ncl;
	cne0 = (ne0+ncl-1)/ncl*ncl;

	for(jj=0; jj<ne0; jj++) hxupi[N-ii][pnv0+jj] = hxupi[N-ii][pnv0+jj] - hxupi[N-ii+1][jj];

#ifdef BLASFEO
	dtrmv_ut_lib(ne0, hpLe[N-ii], cne0, hxupi[N-ii]+pnv0, 0, tmp, tmp);
	dtrmv_un_lib(ne0, hpLe[N-ii], cne0, tmp, 0, hxupi[N-ii]+pnv0, hxupi[N-ii]+pnv0);
#else
	dtrmv_u_t_lib(ne0, hpLe[N-ii], cne0, hxupi[N-ii]+pnv0, 0, tmp);
	dtrmv_u_n_lib(ne0, hpLe[N-ii], cne0, tmp, 0, hxupi[N-ii]+pnv0);
#endif

	for(jj=0; jj<nv0; jj++) hxupi[N-ii][jj] = - hxupi[N-ii][jj];

	if(diag_hessian[N-ii])
		{

#ifdef BLASFEO
		dgemv_t_lib(ne0, nv0, -1.0, hpLA[N-ii]+pnv0*cnv0, cnv0, hxupi[N-ii]+pnv0, 1.0, hxupi[N-ii], hxupi[N-ii]);
#else
		dgemv_t_lib(ne0, nv0, hpLA[N-ii]+pnv0*cnv0, cnv0, hxupi[N-ii]+pnv0, -1, hxupi[N-ii], hxupi[N-ii]);
#endif
		dgemv_diag_lib(nv0, hdLA[N-ii], hxupi[N-ii], 0, hxupi[N-ii], hxupi[N-ii]); 

		}
	else
		{

		if(ne0>pnv0-nv0)
			for(jj=0; jj<pnv0-nv0; jj++) hxupi[N-ii][nv0+jj] = hxupi[N-ii][nve0+jj];
		else
			for(jj=0; jj<ne0; jj++) hxupi[N-ii][nv0+jj] = hxupi[N-ii][pnv0+jj];

#ifdef BLASFEO
		dtrsv_lt_inv_lib(nve0, nv0, hpLA[N-ii], cnv0, hdLA[N-ii], hxupi[N-ii], hxupi[N-ii]);
#else
		dtrsv_t_lib(nve0, nv0, hpLA[N-ii], cnv0, 1, hdLA[N-ii], hxupi[N-ii], hxupi[N-ii]);
#endif

		}

	
//	d_print_mat(1, pnv0+pne0, hxupi[N-ii], 1);
//	exit(1);

	return;

	}




