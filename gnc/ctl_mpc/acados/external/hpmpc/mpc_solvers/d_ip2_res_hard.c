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
#include <math.h>

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>
#endif
//#else
#include "../include/blas_d.h"
//#endif

#include "../include/aux_d.h"
#include "../include/aux_s.h"
#include "../include/lqcp_solvers.h"
#include "../include/block_size.h"
#include "../include/mpc_aux.h"
#include "../include/mpc_solvers.h"
#include "../include/d_blas_aux.h"

// use iterative refinement to increase accuracy of the solution of the equality constrained sub-problems
#define ITER_REF 0
#define THR_ITER_REF 1e-5
//#define ITER_REF_REG 0.0
#define CORRECTOR_LOW 1
#define CORRECTOR_HIGH 1



/* computes work space size */
int d_ip2_res_mpc_hard_tv_work_space_size_bytes(int N, int *nx, int *nu, int *nb, int *ng)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii;

	int pnx, pnz, pnb, png, cnx, cnux;

	int d_size = 0;
	int pnzM = 0;
	int pngM = 0;
	for(ii=0; ii<N; ii++)
		{
		pnz = (nx[ii]+nu[ii]+1+bs-1)/bs*bs;
		if(pnz>pnzM) pnzM = pnz;
		pnb = (nb[ii]+bs-1)/bs*bs;
		png = (ng[ii]+bs-1)/bs*bs;
		if(png>pngM) pngM = png;
		cnx = (nx[ii]+ncl-1)/ncl*ncl;
		cnux = (nu[ii]+nx[ii]+ncl-1)/ncl*ncl;
		pnx = (nx[ii]+bs-1)/bs*bs;
		pnz = (nx[ii]+nu[ii]+1+bs-1)/bs*bs;
		d_size += pnz*(cnx+ncl>cnux ? cnx+ncl : cnux) + 5*pnx + 6*pnz + 19*pnb + 18*png;
#if ITER_REF>0
		d_size += pnz*cnux + 3*pnx + 3*pnz;
#endif
		}
	ii = N;
	pnz = (nx[ii]+1+bs-1)/bs*bs;
	if(pnz>pnzM) pnzM = pnz;
	pnb = (nb[ii]+bs-1)/bs*bs;
	png = (ng[ii]+bs-1)/bs*bs;
	if(png>pngM) pngM = png;
	cnx = (nx[ii]+ncl-1)/ncl*ncl;
	cnux = (nx[ii]+ncl-1)/ncl*ncl;
	pnx = (nx[ii]+bs-1)/bs*bs;
	pnz = (nx[ii]+1+bs-1)/bs*bs;
	d_size += pnz*(cnx+ncl>cnux ? cnx+ncl : cnux) + 5*pnx + 6*pnz + 19*pnb + 18*png;
#if ITER_REF>0
	d_size += pnz*cnux + 3*pnx + 3*pnz;
#endif

	d_size += 2*pngM;

	int size = d_size*sizeof(double);

	size += d_back_ric_rec_sv_tv_work_space_size_bytes(N, nx, nu, nb, ng);
	size += d_back_ric_rec_sv_tv_memory_space_size_bytes(N, nx, nu, nb, ng);

	size = (size + 63) / 64 * 64; // make work space multiple of (typical) cache line size

	return size;
	}



/* primal-dual interior-point method computing residuals at each iteration, hard constraints, time variant matrices, time variant size (mpc version) */
int d_ip2_res_mpc_hard_tv(int *kk, int k_max, double mu0, double mu_tol, double alpha_min, int warm_start, double *stat, int N, int *nx, int *nu_N, int *nb, int **idxb, int *ng, double **pBAbt, double **pQ, double **pDCt, double **d, double **ux, int compute_mult, double **pi, double **lam, double **t, double *double_work_memory)
	{

	// indeces
	int jj, ll, ii, bs0, it_ref;

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;



	// nu with nu[N]=0
	int nu[N+1];
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_N[ii];
	nu[N] = 0;



	// matrices size
	int idx;

	int pnx[N+1];
	int pnz[N+1];
	int pnb[N+1];
	int png[N+1];
	int cnx[N+1];
	int cnux[N+1];

	int pngM = 0;

	for(jj=0; jj<=N; jj++)
		{
		pnx[jj] = (nx[jj]+bs-1)/bs*bs;
		pnz[jj] = (nu[jj]+nx[jj]+1+bs-1)/bs*bs;
		pnb[jj] = (nb[jj]+bs-1)/bs*bs;
		png[jj] = (ng[jj]+bs-1)/bs*bs;
		if(png[jj]>pngM) pngM = png[jj];
		cnx[jj] = (nx[jj]+ncl-1)/ncl*ncl;
		cnux[jj] = (nu[jj]+nx[jj]+ncl-1)/ncl*ncl;
		}



#if 0
printf("\nBAbt\n");
for(ii=0; ii<N; ii++)
	d_print_pmat(nu[ii]+nx[ii]+1, nx[ii+1], bs, pBAbt[ii], cnux[ii+1]);
printf("\nRSQrq\n");
for(ii=0; ii<=N; ii++)
	d_print_pmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], bs, pQ[ii], cnux[ii]);
printf("\nd\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], d[ii], 1);
exit(2);
#endif



	// initialize work space
	double *ptr;
	ptr = double_work_memory; // supposed to be aligned to cache line boundaries

	double *work;
	double *memory;
	double *b[N];
	double *q[N+1];
	double *dux[N+1];
	double *dpi[N];
	double *bd[N+1]; // backup diagonal of Hessian
//	double *bl[N+1]; // backup gradient
	double *dlam[N+1];
	double *dt[N+1];
	double *t_inv[N+1];
	double *lamt[N+1];
	double *Qx[N+1];
	double *qx[N+1];
	double *Pb[N];
	double *res_work;
	double *res_q[N+1];
	double *res_b[N];
	double *res_d[N+1];
	double *res_m[N+1];
	double *ux_bkp[N+1];
	double *pi_bkp[N];
	double *t_bkp[N+1];
	double *lam_bkp[N+1];
#if ITER_REF>0
	double *pQ2[N+1];
	double *q2[N+1];
	double *res_q2[N+1];
	double *res_b2[N];
	double *dux2[N+1];
	double *dpi2[N];
	double *Pb2[N];

	int nb2[N+1]; for(ii=0; ii<=N; ii++) nb2[ii] = 0;
	int ng2[N+1]; for(ii=0; ii<=N; ii++) ng2[ii] = 0;
	int cng[N+1]; for(jj=0; jj<=N; jj++) cng[jj] = (ng[jj]+ncl-1)/ncl*ncl;

	double *pdummy;
	double **ppdummy;
	double *work2;
#endif

	// work space
	work = ptr;
	ptr += d_back_ric_rec_sv_tv_work_space_size_bytes(N, nx, nu, nb, ng) / sizeof(double);

	// memory space
	memory = ptr;
	ptr += d_back_ric_rec_sv_tv_memory_space_size_bytes(N, nx, nu, nb, ng) / sizeof(double);

	// b as vector
	for(jj=0; jj<N; jj++)
		{
		b[jj] = ptr;
		ptr += pnx[jj+1];
//		d_copy_mat(1, nx[jj+1], pBAbt[jj]+(nu[jj]+nx[jj])/bs*bs*cnx[jj+1]+(nu[jj]+nx[jj])%bs, bs, b[jj], 1);
		for(ii=0; ii<nx[jj+1]; ii++)
			b[jj][ii] = pBAbt[jj][(nu[jj]+nx[jj])/bs*bs*cnx[jj+1]+(nu[jj]+nx[jj])%bs+ii*bs];
		}

	// inputs and states
	for(jj=0; jj<=N; jj++)
		{
		dux[jj] = ptr;
		ptr += pnz[jj];
		}

	// equality constr multipliers
	for(jj=0; jj<N; jj++)
		{
		dpi[jj] = ptr;
		ptr += pnx[jj+1];
		}

	// backup of P*b
	for(jj=0; jj<N; jj++)
		{
		Pb[jj] = ptr;
		ptr += pnx[jj+1];
		}

	// linear part of cost function (and copy it)
	for(jj=0; jj<=N; jj++)
		{
		q[jj] = ptr;
		ptr += pnz[jj];
		for(ll=0; ll<nu[jj]+nx[jj]; ll++)
			q[jj][ll] = pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+ll*bs];
		}

	// diagonal of Hessian and gradient backup
	for(jj=0; jj<=N; jj++)
		{
		bd[jj] = ptr;
//		bl[jj] = ptr+pnb[jj];
//		ptr += 2*pnb[jj];
		ptr += pnb[jj];
		// backup
		for(ll=0; ll<nb[jj]; ll++)
			{
			idx = idxb[jj][ll];
			bd[jj][ll] = pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs];
//			bl[jj][ll] = q[jj][idx]; // XXX this has to come after q !!!
			}
		}

	// slack variables, Lagrangian multipliers for inequality constraints and work space
	for(jj=0; jj<=N; jj++)
		{
		dlam[jj] = ptr;
		dt[jj]   = ptr + 2*pnb[jj]+2*png[jj];
		ptr += 4*pnb[jj]+4*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		t_inv[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		lamt[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		Qx[jj] = ptr;
		qx[jj] = ptr+pnb[jj]+png[jj];
		ptr += 2*pnb[jj]+2*png[jj];
		}

	// residuals
	res_work = ptr;
	ptr += 2*pngM;

	for(jj=0; jj<=N; jj++)
		{
		res_q[jj] = ptr;
		ptr += pnz[jj];
		}

	for(jj=0; jj<N; jj++)
		{
		res_b[jj] = ptr;
		ptr += pnx[jj+1];
		}

	for(jj=0; jj<=N; jj++)
		{
		res_d[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		res_m[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		ux_bkp[jj] = ptr;
		ptr += pnz[jj];
		}

	for(jj=0; jj<N; jj++)
		{
		pi_bkp[jj] = ptr;
		ptr += pnx[jj+1];
		}

	for(jj=0; jj<=N; jj++)
		{
		t_bkp[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		lam_bkp[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

#if ITER_REF>0
	for(jj=0; jj<=N; jj++)
		{
		pQ2[jj] = ptr;
		ptr += pnz[jj]*cnux[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		q2[jj] = ptr;
		ptr += pnz[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		res_q2[jj] = ptr;
		ptr += pnz[jj];
		}

	for(jj=0; jj<N; jj++)
		{
		res_b2[jj] = ptr;
		ptr += pnx[jj+1];
		}

	for(jj=0; jj<=N; jj++)
		{
		dux2[jj] = ptr;
		ptr += pnz[jj];
		}

	for(jj=0; jj<N; jj++)
		{
		dpi2[jj] = ptr;
		ptr += pnx[jj+1];
		}

	for(jj=0; jj<N; jj++)
		{
		Pb2[jj] = ptr;
		ptr += pnx[jj+1];
		}

#endif

#if 0
	int cng[N+1];
	for(jj=0; jj<=N; jj++)
		{
		cng[jj] = (ng[jj]+ncl-1)/ncl*ncl;
		}
	d_print_pmat(nu[0]+nx[0]+1, nx[1], bs, pBAbt[0], cnx[1]);
	d_print_pmat(nu[1]+nx[1]+1, nx[2], bs, pBAbt[1], cnx[2]);
	d_print_pmat(nu[0]+nx[0], ng[0], bs, pDCt[0], cng[0]);
	d_print_pmat(nu[1]+nx[1], ng[1], bs, pDCt[1], cng[1]);
	d_print_pmat(nu[N]+nx[N], ng[N], bs, pDCt[N], cng[N]);
	exit(1);
#endif


	double temp0, temp1;
	double alpha, mu, mu_aff;

	// check if there are inequality constraints
	double mu_scal = 0.0;
	for(jj=0; jj<=N; jj++) mu_scal += 2*nb[jj] + 2*ng[jj];
	if(mu_scal!=0.0) // there are some constraints
		{
		mu_scal = 1.0 / mu_scal;
		}
	else // call the riccati solver and return
		{
		double **dummy;
		d_back_ric_rec_sv_tv_res(N, nx, nu, nb, idxb, ng, 0, pBAbt, b, 0, pQ, q, dummy, dummy, dummy, dummy, ux, compute_mult, pi, 1, Pb, memory, work);
		// backup solution
		for(ii=0; ii<=N; ii++)
			for(jj=0; jj<nu[ii]+nx[ii]; jj++)
				ux_bkp[ii][jj] = ux[ii][jj];
		for(ii=0; ii<N; ii++)
			for(jj=0; jj<nx[ii+1]; jj++)
				pi_bkp[ii][jj] = pi[ii][jj];
		// no IPM iterations
		*kk = 0;
		// return success
		return 0;
		}

	//printf("\nmu_scal = %f\n", mu_scal);
	double sigma = 0.0;
	//for(ii=0; ii<=N; ii++)
	//	printf("\n%d %d\n", nb[ii], ng[ii]);
	//exit(1);



	// initialize ux & pi & t>0 & lam>0
	d_init_var_mpc_hard_tv(N, nx, nu, nb, idxb, ng, ux, pi, pDCt, d, t, lam, mu0, warm_start);

#if 0
printf("\nux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], ux[ii], 1);
printf("\npi\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nx[ii+1], pi[ii], 1);
printf("\nlam\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], lam[ii], 1);
printf("\nt\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], t[ii], 1);
exit(1);
#endif



	// compute the duality gap
	mu = mu0;

	// set to zero iteration count
	*kk = 0;

	// larger than minimum accepted step size
	alpha = 1.0;





	//
	// loop without residuals compuation at early iterations
	//

	double mu_tol_low = mu_tol<THR_ITER_REF ? THR_ITER_REF : mu_tol ;

#if 0
	if(0)
#else
	while( *kk<k_max && mu>mu_tol_low && alpha>=alpha_min )
#endif
		{

//		printf("\nkk = %d (no res)\n", *kk);



		//update cost function matrices and vectors (box constraints)
		d_update_hessian_mpc_hard_tv(N, nx, nu, nb, ng, d, 0.0, t, t_inv, lam, lamt, dlam, Qx, qx);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], Qx[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], qx[ii], 1);
//if(*kk==1)
exit(1);
#endif


		// compute the search direction: factorize and solve the KKT system
//		d_back_ric_rec_sv_tv(N, nx, nu, pBAbt, pQ, dux, pL, dL, work, 1, Pb, compute_mult, dpi, nb, idxb, pd, pl, ng, pDCt, Qx, qx2);
#if 1
		d_back_ric_rec_sv_tv_res(N, nx, nu, nb, idxb, ng, 0, pBAbt, b, 1, pQ, q, bd, pDCt, Qx, qx, dux, compute_mult, dpi, 1, Pb, memory, work);
#else
		d_back_ric_rec_trf_tv_res(N, nx, nu, pBAbt, pQ, pL, dL, work, nb, idxb, ng, pDCt, Qx, bd);
		d_back_ric_rec_trs_tv_res(N, nx, nu, pBAbt, b, pL, dL, q, l, dux, work, 1, Pb, compute_mult, dpi, nb, idxb, ng, pDCt, qx);
#endif


#if 0
for(ii=0; ii<=N; ii++)
	d_print_pmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]+1, bs, pQ[ii], cnux[ii]);
exit(1);
#endif
#if 0
for(ii=0; ii<=N; ii++)
	d_print_pmat(pnz[ii], cnz[ii], bs, pL[ii], cnz[ii]);
//exit(1);
#endif
#if 0
printf("\ndux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], dux[ii], 1);
printf("\ndpi\n");
for(ii=0; ii<N; ii++)
	d_print_mat(1, nx[ii+1], dpi[ii], 1);
//if(*kk==1)
exit(1);
#endif


#if CORRECTOR_LOW==1 // IPM1

		// compute t_aff & dlam_aff & dt_aff & alpha
		alpha = 1.0;
		d_compute_alpha_mpc_hard_tv(N, nx, nu, nb, idxb, ng, &alpha, t, dt, lam, dlam, lamt, dux, pDCt, d);



		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+1] = alpha;

		alpha *= 0.995;

#if 0
printf("\nalpha = %f\n", alpha);
exit(1);
#endif


		// compute the affine duality gap
		d_compute_mu_mpc_hard_tv(N, nx, nu, nb, ng, &mu_aff, mu_scal, alpha, lam, dlam, t, dt);

		stat[5*(*kk)+2] = mu_aff;

#if 0
printf("\nmu = %f\n", mu_aff);
exit(1);
#endif



		// compute sigma
		sigma = mu_aff/mu;
		sigma = sigma*sigma*sigma;
//		if(sigma<sigma_min)
//			sigma = sigma_min;
//printf("\n%f %f %f %f\n", mu_aff, mu, sigma, mu_scal);
//exit(1);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii], dt[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii], dlam[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii], t_inv[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nb[ii], pl[ii], 1);
//exit(1);
#endif


//		d_update_gradient_mpc_hard_tv(N, nx, nu, nb, ng, sigma*mu, dt, dlam, t_inv, pl, qx);
		d_update_gradient_mpc_hard_tv(N, nx, nu, nb, ng, sigma*mu, dt, dlam, t_inv, qx);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nb[ii], qx[ii], 1);
//if(*kk==1)
exit(1);
#endif


//		// copy b into x
//		for(ii=0; ii<N; ii++)
//			for(jj=0; jj<nx[ii+1]; jj++)
//				dux[ii+1][nu[ii+1]+jj] = pBAbt[ii][(nu[ii]+nx[ii])/bs*bs*cnx[ii+1]+(nu[ii]+nx[ii])%bs+bs*jj]; // copy b



		// solve the system
//		d_back_ric_rec_trs_tv(N, nx, nu, pBAbt, b, pL, dL, q, l, dux, work, 0, Pb, compute_mult, dpi, nb, idxb, pl, ng, pDCt, qx);
		d_back_ric_rec_trs_tv_res(N, nx, nu, nb, idxb, ng, pBAbt, b, q, pDCt, qx, dux, compute_mult, dpi, 0, Pb, memory, work);

#if 0
printf("\ndux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], dux[ii], 1);
//if(*kk==1)
exit(1);
#endif



#endif // end of IPM1


		// compute t & dlam & dt & alpha
		alpha = 1.0;
		d_compute_alpha_mpc_hard_tv(N, nx, nu, nb, idxb, ng, &alpha, t, dt, lam, dlam, lamt, dux, pDCt, d);

#if 0
printf("\nalpha = %f\n", alpha);
printf("\ndt\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], dt[ii], 1);
printf("\ndlam\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], dlam[ii], 1);
exit(2);
#endif

		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+3] = alpha;

		alpha *= 0.995;



		// backup solution // TODO embedd into update_var e add 'step' to the name
		for(ii=0; ii<=N; ii++)
			for(jj=0; jj<nu[ii]+nx[ii]; jj++)
				ux_bkp[ii][jj] = ux[ii][jj];
		for(ii=0; ii<N; ii++)
			for(jj=0; jj<nx[ii+1]; jj++)
				pi_bkp[ii][jj] = pi[ii][jj];
		for(ii=0; ii<=N; ii++)
			{
			for(jj=0; jj<nb[ii]; jj++)
				{
				lam_bkp[ii][jj] = lam[ii][jj];
				lam_bkp[ii][pnb[ii]+jj] = lam[ii][pnb[ii]+jj];
				t_bkp[ii][jj] = t[ii][jj];
				t_bkp[ii][pnb[ii]+jj] = t[ii][pnb[ii]+jj];
				}
			for(jj=0; jj<ng[ii]; jj++)
				{
				lam_bkp[ii][2*pnb[ii]+jj] = lam[ii][2*pnb[ii]+jj];
				lam_bkp[ii][2*pnb[ii]+png[ii]+jj] = lam[ii][2*pnb[ii]+png[ii]+jj];
				t_bkp[ii][2*pnb[ii]+jj] = t[ii][2*pnb[ii]+jj];
				t_bkp[ii][2*pnb[ii]+png[ii]+jj] = t[ii][2*pnb[ii]+png[ii]+jj];
				}
			}

		// compute step & update x, u, lam, t & compute the duality gap mu
		d_update_var_mpc_hard_tv(N, nx, nu, nb, ng, &mu, mu_scal, alpha, ux, dux, t, dt, lam, dlam, pi, dpi);

		stat[5*(*kk)+4] = mu;


#if 0
printf("\nux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], ux[ii], 1);
printf("\npi\n");
for(ii=0; ii<N; ii++)
	d_print_mat(1, nx[ii+1], pi[ii], 1);
printf("\nlam\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], lam[ii], 1);
printf("\nt\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], t[ii], 1);
//if(*kk==1)
exit(1);
#endif


		// increment loop index
		(*kk)++;


		} // end of IP loop


	// restore Hessian
	for(jj=0; jj<=N; jj++)
		{
		for(ll=0; ll<nb[jj]; ll++)
			{
			idx = idxb[jj][ll];
			pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs] = bd[jj][ll];
//			pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+idx*bs] = bl[jj][ll];
			}
		for(ll=0; ll<nu[jj]+nx[jj]; ll++)
			pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+ll*bs] = q[jj][ll];
		}

#if 0
printf("\nux\n");
for(jj=0; jj<=N; jj++)
	d_print_mat(1, nu[jj]+nx[jj], ux[jj], 1);
printf("\npi\n");
for(jj=0; jj<N; jj++)
	d_print_mat(1, nx[jj+1], pi[jj], 1);
printf("\nlam\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], lam[ii], 1);
printf("\nt\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], t[ii], 1);
exit(2);
#endif


	//
	// loop with residuals computation and iterative refinement for high-accuracy result
	//

	// compute residuals
	d_res_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, pBAbt, b, pQ, q, ux, pDCt, d, pi, lam, t, res_work, res_q, res_b, res_d, res_m, &mu);
#if 0
	printf("kk = %d\n", *kk);
	printf("\nres_q\n");
	for(jj=0; jj<=N; jj++)
		d_print_e_mat(1, nu[jj]+nx[jj], res_q[jj], 1);
	printf("\nres_b\n");
	for(jj=0; jj<N; jj++)
		d_print_e_mat(1, nx[jj+1], res_b[jj], 1);
	printf("\nres_d\n");
	for(jj=0; jj<=N; jj++)
		d_print_e_mat(1, 2*pnb[jj]+2*png[jj], res_d[jj], 1);
	printf("\nres_m\n");
	for(jj=0; jj<=N; jj++)
		d_print_e_mat(1, 2*pnb[jj]+2*png[jj], res_m[jj], 1);
	printf("\nmu\n");
	d_print_e_mat(1, 1, &mu, 1);
	exit(2);
#endif



	// IP loop
#if 0
	int ipm_it;
	for(ipm_it=0; ipm_it<3; ipm_it++)
#else
	while( *kk<k_max && mu>mu_tol && alpha>=alpha_min ) // XXX exit conditions on residuals???
#endif
		{

//		printf("\nkk = %d (res)\n", *kk);


#if 0
printf("\nIPM it %d\n", *kk);
#endif



		// compute the update of Hessian and gradient from box and general constraints
		d_update_hessian_gradient_res_mpc_hard_tv(N, nx, nu, nb, ng, res_d, res_m, t, lam, t_inv, Qx, qx);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], Qx[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], qx[ii], 1);
//if(*kk==1)
exit(1);
#endif



		// compute the search direction: factorize and solve the KKT system
#if ITER_REF>0
		// update Hessian and gradient
		for(ii=0; ii<=N; ii++)
			{

			// box constraints
			// gradient
			for(jj=0; jj<nu[ii]+nx[ii]; jj++)
				q2[ii][jj] = res_q[ii][jj];
			dvecad_libsp(nb[ii], idxb[ii], 1.0, qx[ii], q2[ii]);
//			d_print_mat_e(1, nu[ii]+nx[ii], q2[ii], 1);
			// hessian
			for(jj=0; jj<pnz[ii]*cnux[ii]; jj++)
				pQ2[ii][jj] = pQ[ii][jj];
			ddiaad_libsp(nb[ii], idxb[ii], 1.0, Qx[ii], pQ2[ii], cnux[ii]);
#ifdef BLASFEO
			drowin_lib(cnux[ii], 1.0, q2[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs);
#else
			drowin_lib(cnux[ii], q2[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs);
#endif
//			drowin_lib(cnux[ii], res_q[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs);
//			drowad_libsp(nb[ii], idxb[ii], 1.0, qx[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs);
//			d_print_pmat_e(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], bs, pQ2[ii], cnux[ii]);

			// general constraints
			if(ng[ii]>0) // TODO unsymmetric update not requiring sqrt & div ???
				{
				work2 = work + pnz[ii]*cng[ii];
//				for(jj=0; jj<ng[ii]; jj++)
//					Qx[ii][pnb[ii]+jj] = sqrt(Qx[ii][pnb[ii]+jj]); // XXX
#ifdef BLASFEO
				dgemm_diag_right_lib(nu[ii]+nx[ii], ng[ii], 1.0, pDCt[ii], cng[ii], Qx[ii]+pnb[ii], 0.0, work, cng[ii], work, cng[ii]);
				drowin_lib(ng[ii], 1.0, qx[ii]+pnb[ii], work+(nu[ii]+nx[ii])/bs*cng[ii]*bs+(nu[ii]+nx[ii])%bs);
#else
				dgemm_diag_right_lib(nu[ii]+nx[ii], ng[ii], pDCt[ii], cng[ii], Qx[ii]+pnb[ii], 0, work, cng[ii], work, cng[ii]);
				drowin_lib(ng[ii], qx[ii]+pnb[ii], work+(nu[ii]+nx[ii])/bs*cng[ii]*bs+(nu[ii]+nx[ii])%bs);
#endif
//					work[(nu[ii]+nx[ii])/bs*cng[ii]*bs+(nu[ii]+nx[ii])%bs+jj*bs] /= Qx[ii][pnb[ii]+jj];
#ifdef BLASFEO
				dgecp_lib(nu[ii]+nx[ii], ng[ii], 1.0, 0, pDCt[ii], cng[ii], 0, work2, png[ii]);
				dsyrk_nt_l_lib(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], ng[ii], 1.0, work, cng[ii], work2, cng[ii], 1.0, pQ2[ii], cnux[ii], pQ2[ii], cnux[ii]);
				drowex_lib(cnux[ii], 1.0, pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs, q2[ii]);
#else
				dgecp_lib(nu[ii]+nx[ii], ng[ii], 0, pDCt[ii], cng[ii], 0, work2, png[ii]);
				dsyrk_nt_lib(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], ng[ii], work, cng[ii], work2, cng[ii], 1, pQ2[ii], cnux[ii], pQ2[ii], cnux[ii]);
				drowex_lib(cnux[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs, q2[ii]);
#endif
				}

//			d_print_pmat_e(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], bs, pQ2[ii], cnux[ii]);

//			// regularization
//			ddiareg_lib(nu[ii]+nx[ii], ITER_REF_REG, 0, pQ2[ii], cnux[ii]);

			}
//		exit(2);

		// factorize & solve KKT system
		d_back_ric_rec_sv_tv_res(N, nx, nu, nb2, idxb, ng2, 1, pBAbt, res_b, 0, pQ2, res_q, ppdummy, ppdummy, ppdummy, ppdummy, dux, compute_mult, dpi, 1, Pb, memory, work);

#if CORRECTOR_HIGH==1
		if(0)
#else
		for(it_ref=0; it_ref<ITER_REF; it_ref++)
#endif
			{

//			// remove regularization
//			for(ii=0; ii<=N; ii++)
//				ddiareg_lib(nu[ii]+nx[ii], -ITER_REF_REG, 0, pQ2[ii], cnux[ii]);

			// compute residuals
			d_res_res_mpc_hard_tv(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, pQ2, q2, dux, ppdummy, ppdummy, dpi, ppdummy, ppdummy, res_work, res_q2, res_b2, ppdummy, ppdummy, pdummy);

#if 0
			printf("\niterative refinemet %d\n", it_ref);
			printf("\nres_q2\n");
			for(ii=0; ii<=N; ii++)
				d_print_mat_e(1, nu[ii]+nx[ii], res_q2[ii], 1);
			printf("\nres_b2\n");
			for(ii=0; ii<N; ii++)
				d_print_mat_e(1, nx[ii+1], res_b2[ii], 1);
//			exit(2);
#endif

			// solve for residuals
			d_back_ric_rec_trs_tv_res(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b2, res_q2, ppdummy, ppdummy, dux2, compute_mult, dpi2, 1, Pb2, memory, work);

	//		printf("\nux2\n");
	//		for(ii=0; ii<=N; ii++)
	//			d_print_mat_e(1, nu[ii]+nx[ii], dux2[ii], 1);
	//		printf("\npi2\n");
	//		for(ii=0; ii<N; ii++)
	//			d_print_mat_e(1, nx[ii+1], dpi2[ii], 1);
	//		exit(2);

			// update solution
			for(ii=0; ii<=N; ii++)
				for(jj=0; jj<nu[ii]+nx[ii]; jj++)
					dux[ii][jj] += dux2[ii][jj];
			for(ii=0; ii<N; ii++)
				for(jj=0; jj<nx[ii+1]; jj++)
					dpi[ii][jj] += dpi2[ii][jj];

			}

#if 0
		// compute residuals again
		d_res_res_mpc_hard_tv(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, pQ2, q2, dux, ppdummy, ppdummy, dpi, ppdummy, ppdummy, res_work, res_q2, res_b2, ppdummy, ppdummy, pdummy);

		printf("\nres_q2\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat_e(1, nu[ii]+nx[ii], res_q2[ii], 1);
		printf("\nres_b2\n");
		for(ii=0; ii<N; ii++)
			d_print_mat_e(1, nx[ii+1], res_b2[ii], 1);
//		exit(2);
#endif


#else // no iterative refinement
#if 0
for(ii=0; ii<=N; ii++)
	d_print_e_mat(1, nu[ii]+nx[ii], res_q[ii], 1);
for(ii=0; ii<N; ii++)
	d_print_e_mat(1, nx[ii+1], res_b[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], Qx[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], qx[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_pmat(nu[ii]+nx[ii], nu[ii]+nx[ii], pQ[ii], cnux[ii]);
for(ii=0; ii<N; ii++)
	d_print_pmat(nu[ii]+nx[ii], nx[ii+1], pBAbt[ii], cnx[ii+1]);
exit(1);
#endif
#if 1
		d_back_ric_rec_sv_tv_res(N, nx, nu, nb, idxb, ng, 1, pBAbt, res_b, 1, pQ, res_q, bd, pDCt, Qx, qx, dux, compute_mult, dpi, 1, Pb, memory, work);
#else
		d_back_ric_rec_trf_tv_res(N, nx, nu, pBAbt, pQ, pL, dL, work, nb, idxb, ng, pDCt, Qx, bd);
		d_back_ric_rec_trs_tv_res(N, nx, nu, pBAbt, res_b, pL, dL, res_q, l, dux, work, 1, Pb, compute_mult, dpi, nb, idxb, ng, pDCt, qx);
#endif
#endif


#if 0
//printf("\npL\n");
//for(ii=0; ii<=N; ii++)
//	d_print_pmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]+1, bs, pL[ii], cnux[ii]);
printf("\ndL\n");
for(ii=0; ii<=N; ii++)
	d_print_mat_e(1, nu[ii]+nx[ii], dL[ii], 1);
//exit(1);
#endif
#if 0
//if(*kk==1)
//{
printf("\ndux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], dux[ii], 1);
printf("\ndpi\n");
for(ii=0; ii<N; ii++)
	d_print_mat(1, nx[ii+1], dpi[ii], 1);
//}
//if(*kk==1)
exit(1);
#endif



#if 0
for(ii=0; ii<=N; ii++)
	d_print_pmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]+1, bs, pQ[ii], cnux[ii]);
//exit(1);
#endif
#if 0
for(ii=0; ii<=N; ii++)
	d_print_pmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], bs, pL[ii], cnux[ii]);
//exit(1);
#endif
#if 0
printf("\nux_aff\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], dux[ii], 1);
printf("\npi_aff\n");
for(ii=0; ii<N; ii++)
	d_print_mat(1, nx[ii+1], dpi[ii], 1);
if(*kk==1)
exit(1);
#endif


#if CORRECTOR_HIGH==1 // IPM1

		// compute t_aff & dlam_aff & dt_aff & alpha
		alpha = 1.0;
		d_compute_alpha_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, dux, t, t_inv, lam, pDCt, res_d, res_m, dt, dlam, &alpha);



		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+1] = alpha;

		alpha *= 0.995;

#if 0
printf("\nalpha = %f\n", alpha);
exit(1);
#endif


		// compute the affine duality gap
		d_compute_mu_res_mpc_hard_tv(N, nx, nu, nb, ng, alpha, lam, dlam, t, dt, &mu_aff, mu_scal);

		stat[5*(*kk)+2] = mu_aff;

#if 0
printf("\nmu = %f\n", mu_aff);
exit(1);
#endif



		// compute sigma
		sigma = mu_aff/mu;
		sigma = sigma*sigma*sigma;
//		if(sigma<sigma_min)
//			sigma = sigma_min;
//printf("\n%f %f %f %f\n", mu_aff, mu, sigma, mu_scal);
//exit(1);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii], dt[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii], dlam[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii], t_inv[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nb[ii], pl[ii], 1);
//exit(1);
#endif


		// update res_m
		d_compute_centering_correction_res_mpc_hard_tv(N, nb, ng, sigma*mu, dt, dlam, res_m);



		// update gradient
		d_update_gradient_res_mpc_hard_tv(N, nx, nu, nb, ng, res_d, res_m, lam, t_inv, qx);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], qx[ii], 1);
if(*kk==1)
exit(1);
#endif



#if ITER_REF>0

		// update gradient
		for(ii=0; ii<=N; ii++)
			{
			// copy gradient
			for(jj=0; jj<nu[ii]+nx[ii]; jj++)
				q2[ii][jj] = res_q[ii][jj];
			// box constraints
			if(nb[ii]>0)
				dvecad_libsp(nb[ii], idxb[ii], 1.0, qx[ii], q2[ii]);
			// general constraints
			if(ng[ii]>0)
#ifdef BLASFEO
				dgemv_n_lib(nu[ii]+nx[ii], ng[ii], 1.0, pDCt[ii], cng[ii], qx[ii]+pnb[ii], 1.0, q2[ii], q2[ii]);
#else
				dgemv_n_lib(nu[ii]+nx[ii], ng[ii], pDCt[ii], cng[ii], qx[ii]+pnb[ii], 1, q2[ii], q2[ii]);
#endif
			}

		// solve the KKT system
		d_back_ric_rec_trs_tv_res(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, q2, ppdummy, ppdummy, dux, compute_mult, dpi, 0, Pb, memory, work);

#if 0
printf("\ndux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], dux[ii], 1);
if(*kk==1)
exit(1);
#endif



#if 0
		if(0)
#else
		for(it_ref=0; it_ref<ITER_REF; it_ref++)
#endif
			{

//			// remove regularization
//			for(ii=0; ii<=N; ii++)
//				ddiareg_lib(nu[ii]+nx[ii], -ITER_REF_REG, 0, pQ2[ii], cnux[ii]);

			// compute residuals
			d_res_res_mpc_hard_tv(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, pQ2, q2, dux, ppdummy, ppdummy, dpi, ppdummy, ppdummy, res_work, res_q2, res_b2, ppdummy, ppdummy, pdummy);

#if 0
			printf("\niterative refinemet %d\n", it_ref);
			printf("\nres_q2\n");
			for(ii=0; ii<=N; ii++)
				d_print_mat_e(1, nu[ii]+nx[ii], res_q2[ii], 1);
			printf("\nres_b2\n");
			for(ii=0; ii<N; ii++)
				d_print_mat_e(1, nx[ii+1], res_b2[ii], 1);
//			exit(2);
#endif

			// solve for residuals
			d_back_ric_rec_trs_tv_res(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b2, res_q2, ppdummy, ppdummy, dux2, compute_mult, dpi2, 1, Pb2, memory, work);

	//		printf("\nux2\n");
	//		for(ii=0; ii<=N; ii++)
	//			d_print_mat_e(1, nu[ii]+nx[ii], dux2[ii], 1);
	//		printf("\npi2\n");
	//		for(ii=0; ii<N; ii++)
	//			d_print_mat_e(1, nx[ii+1], dpi2[ii], 1);
	//		exit(2);

			// update solution
			for(ii=0; ii<=N; ii++)
				for(jj=0; jj<nu[ii]+nx[ii]; jj++)
					dux[ii][jj] += dux2[ii][jj];
			for(ii=0; ii<N; ii++)
				for(jj=0; jj<nx[ii+1]; jj++)
					dpi[ii][jj] += dpi2[ii][jj];

			}

#if 0
		// compute residuals again
		d_res_res_mpc_hard_tv(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, pQ2, q2, dux, ppdummy, ppdummy, dpi, ppdummy, ppdummy, res_work, res_q2, res_b2, ppdummy, ppdummy, pdummy);

		printf("\nres_q2\n");
		for(ii=0; ii<=N; ii++)
			d_print_mat_e(1, nu[ii]+nx[ii], res_q2[ii], 1);
		printf("\nres_b2\n");
		for(ii=0; ii<N; ii++)
			d_print_mat_e(1, nx[ii+1], res_b2[ii], 1);
//		exit(2);
#endif


#else // no iter res

		// solve the KKT system
		d_back_ric_rec_trs_tv_res(N, nx, nu, nb, idxb, ng, pBAbt, res_b, res_q, pDCt, qx, dux, compute_mult, dpi, 0, Pb, memory, work);


#endif



#endif // end of IPM1


		// compute t & dlam & dt & alpha
		alpha = 1.0;
		d_compute_alpha_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, dux, t, t_inv, lam, pDCt, res_d, res_m, dt, dlam, &alpha);

#if 0
printf("\nalpha = %f\n", alpha);
printf("\nd\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], d[ii], 1);
printf("\nres_d\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], res_d[ii], 1);
printf("\ndt\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], dt[ii], 1);
printf("\ndlam\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], dlam[ii], 1);
exit(2);
#endif

		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+3] = alpha;

		alpha *= 0.995;



		// backup & update x, u, pi, lam, t
		d_backup_update_var_res_mpc_hard_tv(N, nx, nu, nb, ng, alpha, ux_bkp, ux, dux, pi_bkp, pi, dpi, t_bkp, t, dt, lam_bkp, lam, dlam);
//		d_update_var_res_mpc_hard_tv(N, nx, nu, nb, ng, alpha, ux, dux, pi, dpi, t, dt, lam, dlam);


#if 0
printf("\nux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], ux[ii], 1);
printf("\npi\n");
for(ii=0; ii<N; ii++)
	d_print_mat(1, nx[ii+1], pi[ii], 1);
printf("\nlam\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], lam[ii], 1);
printf("\nt\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], t[ii], 1);
//if(*kk==1)
//exit(1);
#endif


		// compute residuals
		// restore Hessian XXX check for time-invariant problem !!!!!
		for(jj=0; jj<=N; jj++)
			{
			for(ll=0; ll<nb[jj]; ll++)
				{
				idx = idxb[jj][ll];
				pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs] = bd[jj][ll];
//				pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+idx*bs] = bl[jj][ll];
				}
			}
		for(ii=0; ii<=N; ii++)
			for(jj=0; jj<nu[ii]+nx[ii]; jj++)
				pQ[ii][(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs+jj*bs] = q[ii][jj];
		for(ii=0; ii<N; ii++)
			for(jj=0; jj<nx[ii+1]; jj++)
				pBAbt[ii][(nu[ii]+nx[ii])/bs*bs*cnx[ii+1]+(nu[ii]+nx[ii])%bs+jj*bs] = b[ii][jj];
		d_res_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, pBAbt, b, pQ, q, ux, pDCt, d, pi, lam, t, res_work, res_q, res_b, res_d, res_m, &mu);
#if 0
		printf("\nres_q\n");
		for(jj=0; jj<=N; jj++)
			d_print_mat_e(1, nu[jj]+nx[jj], res_q[jj], 1);
//		printf("\nres_b\n");
//		for(jj=0; jj<N; jj++)
//			d_print_mat_e(1, nx[jj+1], res_b[jj], 1);
//		printf("\nres_d\n");
//		for(jj=0; jj<=N; jj++)
//			d_print_mat_e(1, 2*pnb[jj]+2*png[jj], res_d[jj], 1);
//		printf("\nres_m\n");
//		for(jj=0; jj<=N; jj++)
//			d_print_mat_e(1, 2*pnb[jj]+2*png[jj], res_m[jj], 1);
//		printf("\nmu\n");
//		d_print_mat_e(1, 1, &mu, 1);
//		exit(2);
#endif

		stat[5*(*kk)+4] = mu;



		// increment loop index
		(*kk)++;


		} // end of IP loop



	// restore Hessian XXX and gradient
//	for(jj=0; jj<=N; jj++)
//		{
//		for(ll=0; ll<nb[jj]; ll++)
//			{
//			idx = idxb[jj][ll];
//			pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs] = bd[jj][ll];
//			pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+idx*bs] = bl[jj][ll];
//			}
//		}

#if 0
printf("\nABb\n");
for(jj=0; jj<N; jj++)
	d_print_pmat(nu[jj]+nx[jj]+1, nx[jj+1], bs, pBAbt[jj], cnx[jj+1]);
printf("\nQ\n");
for(jj=0; jj<=N; jj++)
	d_print_pmat(nu[jj]+nx[jj]+1, nu[jj]+nx[jj], bs, pQ[jj], cnux[jj]);
//printf("\nux\n");
//for(jj=0; jj<=N; jj++)
//	d_print_mat(1, nu[jj]+nx[jj], ux[jj], 1);
//exit(2);
#endif

#if 0
printf("\nux\n");
for(jj=0; jj<=N; jj++)
	d_print_mat(1, nu[jj]+nx[jj], ux[jj], 1);
printf("\npi\n");
for(jj=0; jj<N; jj++)
	d_print_mat(1, nx[jj+1], pi[jj], 1);
printf("\nlam\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], lam[ii], 1);
printf("\nt\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], t[ii], 1);
exit(2);
#endif



	// TODO if mu is nan, recover solution !!!
//	if(mu==1.0/0.0 || mu==-1.0/0.0)
//		{
//		printf("\nnan!!!\n");
//		exit(3);
//		}



//exit(2);

	// successful exit
	if(mu<=mu_tol)
		return 0;

	// max number of iterations reached
	if(*kk>=k_max)
		return 1;

	// no improvement
	if(alpha<alpha_min)
		return 2;

	// impossible
	return -1;

	} // end of ipsolver

	/* primal-dual interior-point method computing residuals at each iteration, hard constraints, time variant matrices, time variant size (mpc version) */
	int d_ip2_res_mpc_hard_tv_single_newton_step(int *kk, int k_max, double mu0, double mu_tol, double alpha_min, int warm_start, double *stat, int N, int *nx, int *nu_N, int *nb, int **idxb, int *ng, double **pBAbt, double **pQ, double **pDCt, double **d, double **ux, int compute_mult, double **pi, double **lam, double **t, double *double_work_memory, double **ux0,
		double **pi0, double **lam0, double **t0)
		{

		// indeces
		int jj, ll, ii, bs0, it_ref;

		// constants
		const int bs = D_MR;
		const int ncl = D_NCL;



		// nu with nu[N]=0
		int nu[N+1];
		for(ii=0; ii<N; ii++)
			nu[ii] = nu_N[ii];
		nu[N] = 0;



		// matrices size
		int idx;

		int pnx[N+1];
		int pnz[N+1];
		int pnb[N+1];
		int png[N+1];
		int cnx[N+1];
		int cnux[N+1];

		int pngM = 0;

		for(jj=0; jj<=N; jj++)
			{
			pnx[jj] = (nx[jj]+bs-1)/bs*bs;
			pnz[jj] = (nu[jj]+nx[jj]+1+bs-1)/bs*bs;
			pnb[jj] = (nb[jj]+bs-1)/bs*bs;
			png[jj] = (ng[jj]+bs-1)/bs*bs;
			if(png[jj]>pngM) pngM = png[jj];
			cnx[jj] = (nx[jj]+ncl-1)/ncl*ncl;
			cnux[jj] = (nu[jj]+nx[jj]+ncl-1)/ncl*ncl;
			}

		// initialize work space
		double *ptr;
		ptr = double_work_memory; // supposed to be aligned to cache line boundaries

		double *work;
		double *memory;
		double *b[N];
		double *q[N+1];
		double *dux[N+1];
		double *dpi[N];
		double *bd[N+1]; // backup diagonal of Hessian
	//	double *bl[N+1]; // backup gradient
		double *dlam[N+1];
		double *dt[N+1];
		double *t_inv[N+1];
		double *lamt[N+1];
		double *Qx[N+1];
		double *qx[N+1];
		double *Pb[N];
		double *res_work;
		double *res_q[N+1];
		double *res_b[N];
		double *res_d[N+1];
		double *res_m[N+1];
		double *ux_bkp[N+1];
		double *pi_bkp[N];
		double *t_bkp[N+1];
		double *lam_bkp[N+1];
	#if ITER_REF>0
		double *pQ2[N+1];
		double *q2[N+1];
		double *res_q2[N+1];
		double *res_b2[N];
		double *dux2[N+1];
		double *dpi2[N];
		double *Pb2[N];

		int nb2[N+1]; for(ii=0; ii<=N; ii++) nb2[ii] = 0;
		int ng2[N+1]; for(ii=0; ii<=N; ii++) ng2[ii] = 0;
		int cng[N+1]; for(jj=0; jj<=N; jj++) cng[jj] = (ng[jj]+ncl-1)/ncl*ncl;

		double *pdummy;
		double **ppdummy;
		double *work2;
	#endif

		// work space
		work = ptr;
		ptr += d_back_ric_rec_sv_tv_work_space_size_bytes(N, nx, nu, nb, ng) / sizeof(double);

		// memory space
		memory = ptr;
		ptr += d_back_ric_rec_sv_tv_memory_space_size_bytes(N, nx, nu, nb, ng) / sizeof(double);

		// b as vector
		for(jj=0; jj<N; jj++)
			{
			b[jj] = ptr;
			ptr += pnx[jj+1];
			for(ii=0; ii<nx[jj+1]; ii++)
				b[jj][ii] = pBAbt[jj][(nu[jj]+nx[jj])/bs*bs*cnx[jj+1]+(nu[jj]+nx[jj])%bs+ii*bs];
			}

		// inputs and states
		for(jj=0; jj<=N; jj++)
			{
			dux[jj] = ptr;
			ptr += pnz[jj];
			}

		// equality constr multipliers
		for(jj=0; jj<N; jj++)
			{
			dpi[jj] = ptr;
			ptr += pnx[jj+1];
			}

		// backup of P*b
		for(jj=0; jj<N; jj++)
			{
			Pb[jj] = ptr;
			ptr += pnx[jj+1];
			}

		// linear part of cost function (and copy it)
		for(jj=0; jj<=N; jj++)
			{
			q[jj] = ptr;
			ptr += pnz[jj];
			for(ll=0; ll<nu[jj]+nx[jj]; ll++)
				q[jj][ll] = pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+ll*bs];
			}

		// diagonal of Hessian and gradient backup
		for(jj=0; jj<=N; jj++)
			{
			bd[jj] = ptr;
	//		bl[jj] = ptr+pnb[jj];
	//		ptr += 2*pnb[jj];
			ptr += pnb[jj];
			// backup
			for(ll=0; ll<nb[jj]; ll++)
				{
				idx = idxb[jj][ll];
				bd[jj][ll] = pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs];
	//			bl[jj][ll] = q[jj][idx]; // XXX this has to come after q !!!
				}
			}

		// slack variables, Lagrangian multipliers for inequality constraints and work space
		for(jj=0; jj<=N; jj++)
			{
			dlam[jj] = ptr;
			dt[jj]   = ptr + 2*pnb[jj]+2*png[jj];
			ptr += 4*pnb[jj]+4*png[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			t_inv[jj] = ptr;
			ptr += 2*pnb[jj]+2*png[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			lamt[jj] = ptr;
			ptr += 2*pnb[jj]+2*png[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			Qx[jj] = ptr;
			qx[jj] = ptr+pnb[jj]+png[jj];
			ptr += 2*pnb[jj]+2*png[jj];
			}

		// residuals
		res_work = ptr;
		ptr += 2*pngM;

		for(jj=0; jj<=N; jj++)
			{
			res_q[jj] = ptr;
			ptr += pnz[jj];
			}

		for(jj=0; jj<N; jj++)
			{
			res_b[jj] = ptr;
			ptr += pnx[jj+1];
			}

		for(jj=0; jj<=N; jj++)
			{
			res_d[jj] = ptr;
			ptr += 2*pnb[jj]+2*png[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			res_m[jj] = ptr;
			ptr += 2*pnb[jj]+2*png[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			ux_bkp[jj] = ptr;
			ptr += pnz[jj];
			}

		for(jj=0; jj<N; jj++)
			{
			pi_bkp[jj] = ptr;
			ptr += pnx[jj+1];
			}

		for(jj=0; jj<=N; jj++)
			{
			t_bkp[jj] = ptr;
			ptr += 2*pnb[jj]+2*png[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			lam_bkp[jj] = ptr;
			ptr += 2*pnb[jj]+2*png[jj];
			}

	#if ITER_REF>0
		for(jj=0; jj<=N; jj++)
			{
			pQ2[jj] = ptr;
			ptr += pnz[jj]*cnux[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			q2[jj] = ptr;
			ptr += pnz[jj];
			}

		for(jj=0; jj<=N; jj++)
			{
			res_q2[jj] = ptr;
			ptr += pnz[jj];
			}

		for(jj=0; jj<N; jj++)
			{
			res_b2[jj] = ptr;
			ptr += pnx[jj+1];
			}

		for(jj=0; jj<=N; jj++)
			{
			dux2[jj] = ptr;
			ptr += pnz[jj];
			}

		for(jj=0; jj<N; jj++)
			{
			dpi2[jj] = ptr;
			ptr += pnx[jj+1];
			}

		for(jj=0; jj<N; jj++)
			{
			Pb2[jj] = ptr;
			ptr += pnx[jj+1];
			}

	#endif

		double temp0, temp1;
		double alpha, mu, mu_aff;

		// check if there are inequality constraints
		double mu_scal = 0.0;
		for(jj=0; jj<=N; jj++) mu_scal += 2*nb[jj] + 2*ng[jj];
		if(mu_scal!=0.0) // there are some constraints
			{
			mu_scal = 1.0 / mu_scal;
			}
		else // call the riccati solver and return
			{
			double **dummy;
			d_back_ric_rec_sv_tv_res(N, nx, nu, nb, idxb, ng, 0, pBAbt, b, 0, pQ, q, dummy, dummy, dummy, dummy, ux, compute_mult, pi, 1, Pb, memory, work);
			// backup solution
			for(ii=0; ii<=N; ii++)
				for(jj=0; jj<nu[ii]+nx[ii]; jj++)
					ux_bkp[ii][jj] = ux[ii][jj];
			for(ii=0; ii<N; ii++)
				for(jj=0; jj<nx[ii+1]; jj++)
					pi_bkp[ii][jj] = pi[ii][jj];
			// no IPM iterations
			*kk = 0;
			// return success
			return 0;
			}

		double sigma = 0.0;

		// Andrea: skip init, all variables should be initialized in acados.
		// initialize ux & pi & t>0 & lam>0
		d_init_var_mpc_hard_tv_single_newton(N, nx, nu, nb, idxb, ng, ux, pi, pDCt, d, t, lam, ux0, pi0, lam0, t0);

		// compute the duality gap
		mu = mu0;

		// set to zero iteration count
		*kk = 0;

		// larger than minimum accepted step size
		alpha = 1.0;

		// restore Hessian
		for(jj=0; jj<=N; jj++)
			{
			for(ll=0; ll<nb[jj]; ll++)
				{
				idx = idxb[jj][ll];
				pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs] = bd[jj][ll];
	//			pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+idx*bs] = bl[jj][ll];
				}
			for(ll=0; ll<nu[jj]+nx[jj]; ll++)
				pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+ll*bs] = q[jj][ll];
			}

		//
		// loop with residuals computation and iterative refinement for high-accuracy result
		//

		// compute residuals
		d_res_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, pBAbt, b, pQ, q, ux, pDCt, d, pi, lam, t, res_work, res_q, res_b, res_d, res_m, &mu);

		// IP loop
		while( *kk<k_max ) // XXX exit conditions on residuals???
			{
			// compute the update of Hessian and gradient from box and general constraints
			d_update_hessian_gradient_res_mpc_hard_tv(N, nx, nu, nb, ng, res_d, res_m, t, lam, t_inv, Qx, qx);

	// compute the search direction: factorize and solve the KKT system
	#if ITER_REF>0
			// update Hessian and gradient
			for(ii=0; ii<=N; ii++)
				{

				// box constraints
				// gradient
				for(jj=0; jj<nu[ii]+nx[ii]; jj++)
					q2[ii][jj] = res_q[ii][jj];
				dvecad_libsp(nb[ii], idxb[ii], 1.0, qx[ii], q2[ii]);
	//			d_print_mat_e(1, nu[ii]+nx[ii], q2[ii], 1);
				// hessian
				for(jj=0; jj<pnz[ii]*cnux[ii]; jj++)
					pQ2[ii][jj] = pQ[ii][jj];
				ddiaad_libsp(nb[ii], idxb[ii], 1.0, Qx[ii], pQ2[ii], cnux[ii]);
	#ifdef BLASFEO
				drowin_lib(cnux[ii], 1.0, q2[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs);
	#else
				drowin_lib(cnux[ii], q2[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs);
	#endif
				// general constraints
				if(ng[ii]>0) // TODO unsymmetric update not requiring sqrt & div ???
					{
					work2 = work + pnz[ii]*cng[ii];
					dgemm_diag_right_lib(nu[ii]+nx[ii], ng[ii], pDCt[ii], cng[ii], Qx[ii]+pnb[ii], 0, work, cng[ii], work, cng[ii]);
	#ifdef BLASFEO
					drowin_lib(ng[ii], 1.0, qx[ii]+pnb[ii], work+(nu[ii]+nx[ii])/bs*cng[ii]*bs+(nu[ii]+nx[ii])%bs);
	#else
					drowin_lib(ng[ii], qx[ii]+pnb[ii], work+(nu[ii]+nx[ii])/bs*cng[ii]*bs+(nu[ii]+nx[ii])%bs);
	#endif
	#ifdef BLASFEO
					dgecp_lib(nu[ii]+nx[ii], ng[ii], 1.0, 0, pDCt[ii], cng[ii], 0, work2, png[ii]);
					dsyrk_nt_l_lib(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], ng[ii], 1.0, work, cng[ii], work2, cng[ii], 1.0, pQ2[ii], cnux[ii], pQ2[ii], cnux[ii]);
					drowex_lib(cnux[ii], 1.0, pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs, q2[ii]);
	#else
					dgecp_lib(nu[ii]+nx[ii], ng[ii], 0, pDCt[ii], cng[ii], 0, work2, png[ii]);
					dsyrk_nt_lib(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], ng[ii], work, cng[ii], work2, cng[ii], 1, pQ2[ii], cnux[ii], pQ2[ii], cnux[ii]);
					drowex_lib(cnux[ii], pQ2[ii]+(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs, q2[ii]);
	#endif
					}
				}
			// factorize & solve KKT system
			d_back_ric_rec_sv_tv_res(N, nx, nu, nb2, idxb, ng2, 1, pBAbt, res_b, 0, pQ2, res_q, ppdummy, ppdummy, ppdummy, ppdummy, dux, compute_mult, dpi, 1, Pb, memory, work);

	#if CORRECTOR_HIGH==1
			if(0)
	#else
			for(it_ref=0; it_ref<ITER_REF; it_ref++)
	#endif
				{

				// compute residuals
				d_res_res_mpc_hard_tv(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, pQ2, q2, dux, ppdummy, ppdummy, dpi, ppdummy, ppdummy, res_work, res_q2, res_b2, ppdummy, ppdummy, pdummy);

				// solve for residuals
				d_back_ric_rec_trs_tv_res(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b2, res_q2, ppdummy, ppdummy, dux2, compute_mult, dpi2, 1, Pb2, memory, work);

				// update solution
				for(ii=0; ii<=N; ii++)
					for(jj=0; jj<nu[ii]+nx[ii]; jj++)
						dux[ii][jj] += dux2[ii][jj];
				for(ii=0; ii<N; ii++)
					for(jj=0; jj<nx[ii+1]; jj++)
						dpi[ii][jj] += dpi2[ii][jj];

				}

	#else // no iterative refinement
	#if 1
			d_back_ric_rec_sv_tv_res(N, nx, nu, nb, idxb, ng, 0, pBAbt, b, 1, pQ, q, bd, pDCt, Qx, qx, dux, compute_mult, dpi, 1, Pb, memory, work);
	#else
			d_back_ric_rec_trf_tv_res(N, nx, nu, pBAbt, pQ, pL, dL, work, nb, idxb, ng, pDCt, Qx, bd);
			d_back_ric_rec_trs_tv_res(N, nx, nu, pBAbt, res_b, pL, dL, res_q, l, dux, work, 1, Pb, compute_mult, dpi, nb, idxb, ng, pDCt, qx);
	#endif
	#endif

	#if CORRECTOR_HIGH==1 // IPM1

			// compute t_aff & dlam_aff & dt_aff & alpha
			alpha = 1.0;
			d_compute_alpha_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, dux, t, t_inv, lam, pDCt, res_d, res_m, dt, dlam, &alpha);



			stat[5*(*kk)] = sigma;
			stat[5*(*kk)+1] = alpha;

			alpha *= 0.995;

			// compute the affine duality gap
			d_compute_mu_res_mpc_hard_tv(N, nx, nu, nb, ng, alpha, lam, dlam, t, dt, &mu_aff, mu_scal);

			stat[5*(*kk)+2] = mu_aff;

			// Andrea: computation of sigma is moved one level up, in acados. Keep sigma = 1.
			// update res_m
			d_compute_centering_correction_res_mpc_hard_tv(N, nb, ng, mu0, dt, dlam, res_m);

			// update gradient
			d_update_gradient_res_mpc_hard_tv(N, nx, nu, nb, ng, res_d, res_m, lam, t_inv, qx);

	#if ITER_REF>0

			// update gradient
			for(ii=0; ii<=N; ii++)
				{
				// copy gradient
				for(jj=0; jj<nu[ii]+nx[ii]; jj++)
					q2[ii][jj] = res_q[ii][jj];
				// box constraints
				if(nb[ii]>0)
					dvecad_libsp(nb[ii], idxb[ii], 1.0, qx[ii], q2[ii]);
				// general constraints
				if(ng[ii]>0)
	#ifdef BLASFEO
					dgemv_n_lib(nu[ii]+nx[ii], ng[ii], 1.0, pDCt[ii], cng[ii], qx[ii]+pnb[ii], 1.0, q2[ii], q2[ii]);
	#else
					dgemv_n_lib(nu[ii]+nx[ii], ng[ii], pDCt[ii], cng[ii], qx[ii]+pnb[ii], 1, q2[ii], q2[ii]);
	#endif
				}

			// solve the KKT system
			d_back_ric_rec_trs_tv_res(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, q2, ppdummy, ppdummy, dux, compute_mult, dpi, 0, Pb, memory, work);



			for(it_ref=0; it_ref<ITER_REF; it_ref++)
				{
				// compute residuals
				d_res_res_mpc_hard_tv(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, pQ2, q2, dux, ppdummy, ppdummy, dpi, ppdummy, ppdummy, res_work, res_q2, res_b2, ppdummy, ppdummy, pdummy);

				// solve for residuals
				d_back_ric_rec_trs_tv_res(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b2, res_q2, ppdummy, ppdummy, dux2, compute_mult, dpi2, 1, Pb2, memory, work);
				// update solution
				for(ii=0; ii<=N; ii++)
					for(jj=0; jj<nu[ii]+nx[ii]; jj++)
						dux[ii][jj] += dux2[ii][jj];
				for(ii=0; ii<N; ii++)
					for(jj=0; jj<nx[ii+1]; jj++)
						dpi[ii][jj] += dpi2[ii][jj];

				}

	#if 0
			// compute residuals again
			d_res_res_mpc_hard_tv(N, nx, nu, nb2, idxb, ng2, pBAbt, res_b, pQ2, q2, dux, ppdummy, ppdummy, dpi, ppdummy, ppdummy, res_work, res_q2, res_b2, ppdummy, ppdummy, pdummy);

			printf("\nres_q2\n");
			for(ii=0; ii<=N; ii++)
				d_print_mat_e(1, nu[ii]+nx[ii], res_q2[ii], 1);
			printf("\nres_b2\n");
			for(ii=0; ii<N; ii++)
				d_print_mat_e(1, nx[ii+1], res_b2[ii], 1);
	//		exit(2);
	#endif


	#else // no iter res

			// solve the KKT system
			d_back_ric_rec_trs_tv_res(N, nx, nu, nb, idxb, ng, pBAbt, res_b, res_q, pDCt, qx, dux, compute_mult, dpi, 0, Pb, memory, work);


	#endif



	#endif // end of IPM1


			// compute t & dlam & dt & alpha
			alpha = 1.0;
			d_compute_alpha_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, dux, t, t_inv, lam, pDCt, res_d, res_m, dt, dlam, &alpha);

			stat[5*(*kk)] = sigma;
			stat[5*(*kk)+3] = alpha;

			alpha *= 0.995;



			// backup & update x, u, pi, lam, t
			d_backup_update_var_res_mpc_hard_tv(N, nx, nu, nb, ng, alpha, ux_bkp, ux, dux, pi_bkp, pi, dpi, t_bkp, t, dt, lam_bkp, lam, dlam);

			// compute residuals
			// restore Hessian XXX check for time-invariant problem !!!!!
			for(jj=0; jj<=N; jj++)
				{
				for(ll=0; ll<nb[jj]; ll++)
					{
					idx = idxb[jj][ll];
					pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs] = bd[jj][ll];
	//				pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+idx*bs] = bl[jj][ll];
					}
				}
			for(ii=0; ii<=N; ii++)
				for(jj=0; jj<nu[ii]+nx[ii]; jj++)
					pQ[ii][(nu[ii]+nx[ii])/bs*bs*cnux[ii]+(nu[ii]+nx[ii])%bs+jj*bs] = q[ii][jj];
			for(ii=0; ii<N; ii++)
				for(jj=0; jj<nx[ii+1]; jj++)
					pBAbt[ii][(nu[ii]+nx[ii])/bs*bs*cnx[ii+1]+(nu[ii]+nx[ii])%bs+jj*bs] = b[ii][jj];
			d_res_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, pBAbt, b, pQ, q, ux, pDCt, d, pi, lam, t, res_work, res_q, res_b, res_d, res_m, &mu);

			stat[5*(*kk)+4] = mu;



			// increment loop index
			(*kk)++;


			} // end of IP loop

	//exit(2);

		// max number of iterations reached
		if(*kk>=k_max)
			return 1;

		// no improvement
		if(alpha<alpha_min)
			return 2;

		// impossible
		return -1;

		} // end of ipsolver

void d_kkt_solve_new_rhs_res_mpc_hard_tv(int N, int *nx, int *nu_N, int *nb, int **idxb, int *ng, double **pBAbt, double **b, double **pQ, double **q, double **pDCt, double **d, double **ux, int compute_mult, double **pi, double **lam, double **t, double *double_work_memory)
	{

	// indeces
	int jj, ll, ii, bs0;

	// constants
	const int bs = D_MR;
	const int ncl = D_NCL;



	// nu with nu[N]=0
	int nu[N+1];
	for(ii=0; ii<N; ii++)
		nu[ii] = nu_N[ii];
	nu[N] = 0;



	// matrices size
	int idx;

	int pnx[N+1];
	int pnz[N+1];
	int pnb[N+1];
	int png[N+1];
	int cnx[N+1];
	int cnux[N+1];

	int pngM = 0;

	for(jj=0; jj<=N; jj++)
		{
		pnx[jj] = (nx[jj]+bs-1)/bs*bs;
		pnz[jj] = (nu[jj]+nx[jj]+1+bs-1)/bs*bs;
		pnb[jj] = (nb[jj]+bs-1)/bs*bs;
		png[jj] = (ng[jj]+bs-1)/bs*bs;
		if(png[jj]>pngM) pngM = png[jj];
		cnx[jj] = (nx[jj]+ncl-1)/ncl*ncl;
		cnux[jj] = (nu[jj]+nx[jj]+ncl-1)/ncl*ncl;
		}



	// initialize work space
	double *ptr;
	ptr = double_work_memory; // supposed to be aligned to cache line boundaries

	double *work;
	double *memory;
	double *b_old[N];
	double *q_old[N+1];
	double *dux[N+1];
	double *dpi[N];
	double *bd[N+1]; // backup diagonal of Hessian
//	double *bl[N+1]; // backup diagonal of Hessian
	double *dlam[N+1];
	double *dt[N+1];
	double *t_inv[N+1];
	double *lamt[N+1];
	double *Qx[N+1];
	double *qx[N+1];
	double *Pb[N];
	double *res_work;
	double *res_q[N+1];
	double *res_b[N];
	double *res_d[N+1];
	double *res_m[N+1];
	double *ux_bkp[N+1];
	double *pi_bkp[N];
	double *t_bkp[N+1];
	double *lam_bkp[N+1];

	// work space
	work = ptr;
	ptr += d_back_ric_rec_sv_tv_work_space_size_bytes(N, nx, nu, nb, ng) / sizeof(double);

	// work space
	memory = ptr;
	ptr += d_back_ric_rec_sv_tv_memory_space_size_bytes(N, nx, nu, nb, ng) / sizeof(double);

	// b as vector
	for(jj=0; jj<N; jj++)
		{
		b_old[jj] = ptr;
		ptr += pnx[jj+1];
		}

	// inputs and states
	for(jj=0; jj<=N; jj++)
		{
		dux[jj] = ptr;
		ptr += pnz[jj];
		}

	// equality constr multipliers
	for(jj=0; jj<N; jj++)
		{
		dpi[jj] = ptr;
		ptr += pnx[jj+1];
		}

	// backup of P*b
	for(jj=0; jj<N; jj++)
		{
		Pb[jj] = ptr;
		ptr += pnx[jj+1];
		}

	// linear part of cost function (and copy it)
	for(jj=0; jj<=N; jj++)
		{
		q_old[jj] = ptr;
		ptr += pnz[jj];
		}

	// diagonal of Hessian backup
	for(jj=0; jj<=N; jj++)
		{
		bd[jj] = ptr;
//		bl[jj] = ptr+pnb[jj];
//		ptr += 2*pnb[jj];
		ptr += pnb[jj];
//		// backup
//		for(ll=0; ll<nb[jj]; ll++)
//			{
//			idx = idxb[jj][ll];
//			bd[jj][ll] = pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs];
//			bl[jj][ll] = q[jj][idx]; // XXX this has to come after q !!!
//			}
		}

	// slack variables, Lagrangian multipliers for inequality constraints and work space
	for(jj=0; jj<=N; jj++)
		{
		dlam[jj] = ptr;
		dt[jj]   = ptr + 2*pnb[jj]+2*png[jj];
		ptr += 4*pnb[jj]+4*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		t_inv[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		lamt[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		Qx[jj] = ptr;
		qx[jj] = ptr+pnb[jj]+png[jj];
		ptr += 2*pnb[jj]+2*png[jj];
		}

	// residuals
	res_work = ptr;
	ptr += 2*pngM;

	for(jj=0; jj<=N; jj++)
		{
		res_q[jj] = ptr;
		ptr += pnz[jj];
		}

	for(jj=0; jj<N; jj++)
		{
		res_b[jj] = ptr;
		ptr += pnx[jj+1];
		}

	for(jj=0; jj<=N; jj++)
		{
		res_d[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		res_m[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		ux_bkp[jj] = ptr;
		ptr += pnz[jj];
		}

	for(jj=0; jj<N; jj++)
		{
		pi_bkp[jj] = ptr;
		ptr += pnx[jj+1];
		}

	for(jj=0; jj<=N; jj++)
		{
		t_bkp[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		lam_bkp[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

#if 1

	double mu;

	// initialize solution with backup
	// bkp ux
	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<nu[ii]+nx[ii]; jj++)
			ux[ii][jj] = ux_bkp[ii][jj];
	// bkp pi
	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nx[ii+1]; jj++)
			pi[ii][jj] = pi_bkp[ii][jj];
	// bkp t
	for(ii=0; ii<=N; ii++)
		{
		for(jj=0; jj<nb[ii]; jj++)
			{
			t[ii][jj] = t_bkp[ii][jj];
			t[ii][pnb[ii]+jj] = t_bkp[ii][pnb[ii]+jj];
			}
		for(jj=0; jj<ng[ii]; jj++)
			{
			t[ii][2*pnb[ii]+jj] = t_bkp[ii][2*pnb[ii]+jj];
			t[ii][2*pnb[ii]+png[ii]+jj] = t_bkp[ii][2*pnb[ii]+png[ii]+jj];
			}
		}
	// bkp lam
	for(ii=0; ii<=N; ii++)
		{
		for(jj=0; jj<nb[ii]; jj++)
			{
			lam[ii][jj] = lam_bkp[ii][jj];
			lam[ii][pnb[ii]+jj] = lam_bkp[ii][pnb[ii]+jj];
			}
		for(jj=0; jj<ng[ii]; jj++)
			{
			lam[ii][2*pnb[ii]+jj] = lam_bkp[ii][2*pnb[ii]+jj];
			lam[ii][2*pnb[ii]+png[ii]+jj] = lam_bkp[ii][2*pnb[ii]+png[ii]+jj];
			}
		}

#if 0
printf("\nux_bkp\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], ux[ii], 1);
printf("\npi_bkp\n");
for(ii=0; ii<N; ii++)
	d_print_mat(1, nx[ii+1], pi[ii], 1);
printf("\nlam_bkp\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], lam[ii], 1);
printf("\nt_bkp\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, 2*pnb[ii]+2*png[ii], t[ii], 1);
exit(2);
#endif

	// compute residuals
	d_res_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, pBAbt, b, pQ, q, ux, pDCt, d, pi, lam, t, res_work, res_q, res_b, res_d, res_m, &mu);

#if 0
printf("\nres_q\n");
for(jj=0; jj<=N; jj++)
	d_print_mat_e(1, nu[jj]+nx[jj], res_q[jj], 1);
printf("\nres_b\n");
for(jj=0; jj<N; jj++)
	d_print_mat_e(1, nx[jj+1], res_b[jj], 1);
printf("\nres_d\n");
for(jj=0; jj<=N; jj++)
	d_print_mat_e(1, 2*pnb[jj]+2*png[jj], res_d[jj], 1);
printf("\nres_m\n");
for(jj=0; jj<=N; jj++)
	d_print_mat_e(1, 2*pnb[jj]+2*png[jj], res_m[jj], 1);
printf("\nmu\n");
d_print_mat_e(1, 1, &mu, 1);
exit(2);
#endif

	// update gradient
	d_update_gradient_res_mpc_hard_tv(N, nx, nu, nb, ng, res_d, res_m, lam, t_inv, qx);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, pnb[ii]+png[ii], qx[ii], 1);
exit(2);
#endif

	// solve the system
	d_back_ric_rec_trs_tv_res(N, nx, nu, nb, idxb, ng, pBAbt, res_b, res_q, pDCt, qx, dux, compute_mult, dpi, 1, Pb, memory, work);

#if 0
printf("\nNEW dux\n");
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], dux[ii], 1);
//if(*kk==1)
//exit(1);
#endif

	// compute t & dlam & dt
	d_compute_dt_dlam_res_mpc_hard_tv(N, nx, nu, nb, idxb, ng, dux, t, t_inv, lam, pDCt, res_d, res_m, dt, dlam);

	// update x, u, lam, t & compute the duality gap mu
	d_update_var_res_mpc_hard_tv(N, nx, nu, nb, ng, 1.0, ux, dux, pi, dpi, t, dt, lam, dlam);

#else

	// XXX temporary allocation
	double *lamt[N+1];
	for(ii=0; ii<=N; ii++)
		{
		d_zeros_align(&lamt[ii], 2*pnb[ii]+2*png[ii], 1);
		for(jj=0; jj<nb[ii]; jj++)
			{
			lamt[ii][jj] = lam_bkp[ii][jj] * t_inv[ii][jj];
			lamt[ii][pnb[ii]+jj] = lam_bkp[ii][pnb[ii]+jj] * t_inv[ii][pnb[ii]+jj];
			}
		for(jj=0; jj<ng[ii]; jj++)
			{
			lamt[ii][jj] = lam_bkp[ii][jj] * t_inv[ii][jj];
			lamt[ii][2*pnb[ii]+png[ii]+jj] = lam_bkp[ii][2*pnb[ii]+png[ii]+jj] * t_inv[ii][2*pnb[ii]+png[ii]+jj];
			}
		}

	double *bl[N+1];
	for(ii=0; ii<=N; ii++)
		{
		d_zeros_align(&bl[ii], nb[ii], 1);
		for(jj=0; jj<nb[ii]; jj++)
			{
			bl[ii][jj] = r_H[ii][idxb[ii][jj]];
			}
		}

	double *pl[N+1];
	for(ii=0; ii<=N; ii++)
		d_zeros_align(&pl[ii], nb[ii], 1);



	//update cost function vectors for a generic RHS (not tailored to IPM)
	d_update_gradient_new_rhs_mpc_hard_tv(N, nx, nu, nb, ng, t_inv, lamt, qx, bl, pl, r_C);


	// solve the system
	d_back_ric_rec_trs_tv(N, nx, nu, pBAbt, r_A, pL, dL, r_H, l, ux, work, 1, Pb, compute_mult, pi, nb, idxb, pl, ng, pDCt, qx);


	// compute t & lam
	d_compute_t_lam_new_rhs_mpc_hard_tv(N, nx, nu, nb, idxb, ng, t, lam, lamt, t_inv, ux, pDCt, r_C);



	// free memory
	for(ii=0; ii<=N; ii++)
		{
		free(lamt[ii]);
		free(bl[ii]);
		free(pl[ii]);
		}

#endif



	} // end of final kkt solve
