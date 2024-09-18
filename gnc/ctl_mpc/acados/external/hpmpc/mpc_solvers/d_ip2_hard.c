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

#if defined(BLASFEO)
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#endif

#include "../include/aux_d.h"
#include "../include/aux_s.h"
#include "../include/lqcp_solvers.h"
#include "../include/block_size.h"
#include "../include/mpc_aux.h"



/* computes work space size */
int d_ip2_mpc_hard_tv_work_space_size_bytes(int N, int *nx, int *nu, int *nb, int *ng)
	{

	const int bs = D_MR;
	const int ncl = D_NCL;

	int ii;

	int pnx, pnz, pnb, png, cnx, cnux; 

	int size = 0;
	int pnzM = 0;
	for(ii=0; ii<N; ii++)
		{
		pnz = (nx[ii]+nu[ii]+1+bs-1)/bs*bs;
		if(pnz>pnzM) pnzM = pnz;
		pnb = (nb[ii]+bs-1)/bs*bs;
		png = (ng[ii]+bs-1)/bs*bs;
		cnx = (nx[ii]+ncl-1)/ncl*ncl;
		cnux = (nu[ii]+nx[ii]+ncl-1)/ncl*ncl;
		pnx = (nx[ii]+bs-1)/bs*bs;
		pnz = (nx[ii]+nu[ii]+1+bs-1)/bs*bs;
		size += 3*pnx + 3*pnz + 11*pnb + 10*png;
		}
	ii = N;
	pnz = (nx[ii]+1+bs-1)/bs*bs;
	if(pnz>pnzM) pnzM = pnz;
	pnb = (nb[ii]+bs-1)/bs*bs;
	png = (ng[ii]+bs-1)/bs*bs;
	cnx = (nx[ii]+ncl-1)/ncl*ncl;
	cnux = (nx[ii]+ncl-1)/ncl*ncl;
	pnx = (nx[ii]+bs-1)/bs*bs;
	pnz = (nx[ii]+1+bs-1)/bs*bs;
	size += 3*pnx + 3*pnz + 11*pnb + 10*png;

	size *= sizeof(double);

	size += d_back_ric_rec_sv_tv_work_space_size_bytes(N, nx, nu, nb, ng);
	size += d_back_ric_rec_sv_tv_memory_space_size_bytes(N, nx, nu, nb, ng);

	return size;
	}



int d_ip2_mpc_hard_tv(int *kk, int k_max, double mu0, double mu_tol, double alpha_min, int warm_start, double *stat, int N, int *nx, int *nu_N, int *nb, int **idxb, int *ng, double **pBAbt, double **pQ, double **pDCt, double **d, double **ux, int compute_mult, double **pi, double **lam, double **t, double *double_work_memory)
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
	int nzM = 0;

	int pnx[N+1];
	int pnz[N+1];
	int pnb[N+1];
	int png[N+1];
	int cnx[N+1];
	int cnux[N+1];

	for(jj=0; jj<=N; jj++)
		{
		pnx[jj] = (nx[jj]+bs-1)/bs*bs;
		pnz[jj] = (nu[jj]+nx[jj]+1+bs-1)/bs*bs;
		pnb[jj] = (nb[jj]+bs-1)/bs*bs;
		png[jj] = (ng[jj]+bs-1)/bs*bs;
		cnx[jj] = (nx[jj]+ncl-1)/ncl*ncl;
		cnux[jj] = (nu[jj]+nx[jj]+ncl-1)/ncl*ncl;
		if(nu[jj]+nx[jj]+1>nzM) nzM = nu[jj]+nx[jj]+1;
		}



	// initialize work space
	// work_space_double_size_per_stage = pnz*cnl + 2*pnz + 2*pnx + 14*pnb + 10*png
	// work_space_double_size_const_max = pnz*cnxg + pnz
	double *ptr;
	ptr = double_work_memory; // supposed to be aligned to cache line boundaries

	double *work;
	double *memory;
	double *b[N];
	double *q[N+1];
	double *dux[N+1];
	double *dpi[N];
//	double *pd[N+1]; // pointer to diagonal of Hessian
//	double *pl[N+1]; // pointer to linear part of Hessian
	double *bd[N+1]; // backup diagonal of Hessian
//	double *bl[N+1]; // backup linear part of Hessian
	double *dlam[N+1];
	double *dt[N+1];
	double *lamt[N+1];
	double *t_inv[N+1];
	double *Qx[N+1];
	double *qx[N+1];
//	double *qx2[N+1];
	double *Pb[N];

//	int size = 0;

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

	// linear part of cost function
	for(jj=0; jj<=N; jj++)
		{
		q[jj] = ptr;
		ptr += pnz[jj];
		for(ll=0; ll<nu[jj]+nx[jj]; ll++) 
			q[jj][ll] = pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+ll*bs];
		}

	// Hessian backup
	for(jj=0; jj<=N; jj++)
		{
//		pd[jj] = ptr;
//		pl[jj] = ptr + pnb[jj];
//		bd[jj] = ptr + 2*pnb[jj];
//		bl[jj] = ptr + 3*pnb[jj];
//		ptr += 4*pnb[jj];
		bd[jj] = ptr;
//		bl[jj] = ptr + pnb[jj];
//		ptr += 2*pnb[jj];
		ptr += pnb[jj];
		// backup
		for(ll=0; ll<nb[jj]; ll++)
			{
			idx = idxb[jj][ll];
			bd[jj][ll] = pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs];
//			bl[jj][ll] = q[jj][idx];
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
		lamt[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		t_inv[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		Qx[jj] = ptr;
		qx[jj] = ptr+pnb[jj]+png[jj];
//		qx2[jj] = ptr+2*pnb[jj]+2*png[jj];
//		ptr += 3*pnb[jj]+3*png[jj];
		ptr += 2*pnb[jj]+2*png[jj];
		}
	
//	printf("\nsize real = %d\n", size);

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
//	printf("\nmu_scal = %f\n", mu_scal);
//	exit(2);
	if(mu_scal!=0.0) // there are some constraints
		{
		mu_scal = 1.0 / mu_scal;
		}
	else // call the riccati solver and return
		{
		double **dummy;
		d_back_ric_rec_sv_tv_res(N, nx, nu, nb, idxb, ng, 0, pBAbt, b, 0, pQ, q, dummy, dummy, dummy, dummy, dux, compute_mult, dpi, 1, Pb, memory, work);
//		d_back_ric_rec_sv_tv(N, nx, nu, pBAbt, pQ, ux, pL, dL, work, 1, Pb, compute_mult, pi, nb, idxb, dummy, dummy, ng, dummy, dummy, dummy);
		*kk = 0;
		return 0;
		}

	//printf("\nmu_scal = %f\n", mu_scal);
	double sigma, sigma_decay, sigma_min;
	//for(ii=0; ii<=N; ii++)
	//	printf("\n%d %d\n", nb[ii], ng[ii]);
	//exit(1);



	// initialize ux & t>0 (slack variable)
	d_init_var_mpc_hard_tv(N, nx, nu, nb, idxb, ng, ux, pi, pDCt, d, t, lam, mu0, warm_start);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nu[ii]+nx[ii], ux[ii], 1);
exit(1);
#endif


	// initialize pi
	for(jj=0; jj<N; jj++)
		for(ll=0; ll<nx[jj+1]; ll++)
			dpi[jj][ll] = 0.0;



	// initialize dux
	for(ll=0; ll<nx[0]; ll++)
		dux[0][nu[0]+ll] = ux[0][nu[0]+ll];




	// compute the duality gap
	//alpha = 0.0; // needed to compute mu !!!!!
	//d_compute_mu_hard_mpc(N, nx, nu, nb, &mu, mu_scal, alpha, lam, dlam, t, dt);
	mu = mu0;

	// set to zero iteration count
	*kk = 0;	

	// larger than minimum accepted step size
	alpha = 1.0;

	// update hessian in Riccati routine
	const int update_hessian = 1;



	// IP loop		
	while( *kk<k_max && mu>mu_tol && alpha>=alpha_min )
		{
						


		//update cost function matrices and vectors (box constraints)
		d_update_hessian_mpc_hard_tv(N, nx, nu, nb, ng, d, 0.0, t, t_inv, lam, lamt, dlam, Qx, qx);

#if 0
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nb[ii], pd[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, nb[ii], pl[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, ng[ii], Qx[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, ng[ii], qx[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, ng[ii], qx2[ii], 1);
//if(*kk==1)
exit(1);
#endif

#if 0
int cnl[N+1];
for(ii=0; ii<N; ii++)
	{
	cnl[ii] = cnux[ii]<cnx[ii]+ncl ? cnx[ii]+ncl : cnux[ii];
	}
double *ptr = memory;
double *hpL[N+1];
for(ii=0; ii<=N; ii++)
	{
	hpL[ii] = ptr;
	ptr += pnz[ii]*cnl[ii];
	}
for(ii=0; ii<=N; ii++)
	d_print_pmat(pnz[ii], cnl[ii], bs, hpL[ii], cnl[ii]);
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
int cnl[N+1];
for(ii=0; ii<N; ii++)
	{
	cnl[ii] = cnux[ii]<cnx[ii]+ncl ? cnx[ii]+ncl : cnux[ii];
	}
double *ptr = memory;
double *hpL[N+1];
for(ii=0; ii<=N; ii++)
	{
	hpL[ii] = ptr;
	ptr += pnz[ii]*cnl[ii];
	}
for(ii=0; ii<=N; ii++)
	d_print_pmat(2*pnz[ii], cnl[ii], bs, hpL[ii], cnl[ii]);
exit(1);
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


#if 1 // IPM1

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
	d_print_mat(1, nb[ii], pl[ii], 1);
for(ii=0; ii<=N; ii++)
	d_print_mat(1, ng[ii], qx[ii], 1);
if(*kk==1)
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
if(*kk==1)
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



		// update x, u, lam, t & compute the duality gap mu

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
printf("\nq\n");
for(jj=0; jj<=N; jj++)
	d_print_mat(1, nb[jj], bl[jj], 1);
exit(2);
#endif
#if 0
printf("\nq\n");
for(jj=0; jj<=N; jj++)
	d_print_mat(1, nu[jj]+nx[jj], q[jj], 1);
exit(2);
#endif
#if 0
printf("\nux\n");
for(jj=0; jj<=N; jj++)
	d_print_mat(1, nu[jj]+nx[jj], ux[jj], 1);
exit(2);
#endif



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



void d_kkt_solve_new_rhs_mpc_hard_tv(int N, int *nx, int *nu_N, int *nb, int **idxb, int *ng, double **pBAbt, double **r_A, double **pQ, double **r_H, double **pDCt, double **r_C, double **ux, int compute_mult, double **pi, double **lam, double **t, double *double_work_memory)
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
//	int nxM = 0;
	int nzM = 0;
//	int ngM = 0;

	int pnx[N+1];
	int pnz[N+1];
	int pnb[N+1];
	int png[N+1];
	int cnx[N+1];
//	int cnz[N+1];
	int cnux[N+1];

	for(jj=0; jj<=N; jj++)
		{
		pnx[jj] = (nx[jj]+bs-1)/bs*bs;
		pnz[jj] = (nu[jj]+nx[jj]+1+bs-1)/bs*bs;
		pnb[jj] = (nb[jj]+bs-1)/bs*bs;
		png[jj] = (ng[jj]+bs-1)/bs*bs;
		cnx[jj] = (nx[jj]+ncl-1)/ncl*ncl;
//		cnz[jj] = (nu[jj]+nx[jj]+1+ncl-1)/ncl*ncl;
		cnux[jj] = (nu[jj]+nx[jj]+ncl-1)/ncl*ncl;
//		if(nx[jj]>nxM) nxM = nx[jj];
		if(nu[jj]+nx[jj]+1>nzM) nzM = nu[jj]+nx[jj]+1;
//		if(ng[jj]>ngM) ngM = ng[jj];
//		if(nx[jj]+ng[jj]>nxgM) nxgM = nx[jj]+ng[jj];
		}



	// initialize work space
	// work_space_double_size_per_stage = pnz*cnl + 2*pnz + 2*pnx + 14*pnb + 10*png
	// work_space_double_size_const_max = pnz*cnxg + pnz
	double *ptr;
	ptr = double_work_memory; // supposed to be aligned to cache line boundaries

	double *work;
	double *memory;
	double *b[N];
	double *q[N+1];
	double *dux[N+1];
	double *dpi[N];
//	double *pd[N+1]; // pointer to diagonal of Hessian
//	double *pl[N+1]; // pointer to linear part of Hessian
	double *bd[N+1]; // backup diagonal of Hessian
//	double *bl[N+1]; // backup linear part of Hessian
	double *diag;
	double *dlam[N+1];
	double *dt[N+1];
	double *lamt[N+1];
	double *t_inv[N+1];
	double *Qx[N+1];
	double *qx[N+1];
//	double *qx2[N+1];
	double *Pb[N];

//	int size = 0;

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
//		for(ii=0; ii<nx[jj+1]; ii++)
//			b[jj][ii] = pBAbt[jj][(nu[jj]+nx[jj])/bs*bs*cnx[jj+1]+(nu[jj]+nx[jj])%bs+ii*bs];
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

	// linear part of cost function
	for(jj=0; jj<=N; jj++)
		{
		q[jj] = ptr;
		ptr += pnz[jj];
//		for(ll=0; ll<nu[jj]+nx[jj]; ll++) q[jj][ll] = pQ[jj][(nu[jj]+nx[jj])/bs*bs*cnux[jj]+(nu[jj]+nx[jj])%bs+ll*bs];
		}

	// Hessian backup
	for(jj=0; jj<=N; jj++)
		{
//		pd[jj] = ptr;
//		pl[jj] = ptr + pnb[jj];
//		bd[jj] = ptr + 2*pnb[jj];
//		bl[jj] = ptr + 3*pnb[jj];
//		ptr += 4*pnb[jj];
		bd[jj] = ptr;
//		bl[jj] = ptr + pnb[jj];
//		ptr += 2*pnb[jj];
		ptr += pnb[jj];
		// backup
		for(ll=0; ll<nb[jj]; ll++)
			{
			idx = idxb[jj][ll];
//			bd[jj][ll] = pQ[jj][idx/bs*bs*cnux[jj]+idx%bs+idx*bs];
//			bl[jj][ll] = q[jj][idx];
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
		lamt[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		t_inv[jj] = ptr;
		ptr += 2*pnb[jj]+2*png[jj];
		}

	for(jj=0; jj<=N; jj++)
		{
		Qx[jj] = ptr;
		qx[jj] = ptr+pnb[jj]+png[jj];
//		qx2[jj] = ptr+2*pnb[jj]+2*png[jj];
//		ptr += 3*pnb[jj]+3*png[jj];
		ptr += 2*pnb[jj]+2*png[jj];
		}
	//	printf("\nsize real = %d\n", size);




	//update cost function vectors for a generic RHS (not tailored to IPM)
	d_update_gradient_new_rhs_mpc_hard_tv(N, nx, nu, nb, ng, r_C, t_inv, lamt, qx); 


	// solve the system
//	d_back_ric_rec_trs_tv(N, nx, nu, pBAbt, r_A, pL, dL, r_H, l, ux, work, 1, Pb, compute_mult, pi, nb, idxb, pl, ng, pDCt, qx);
	d_back_ric_rec_trs_tv_res(N, nx, nu, nb, idxb, ng, pBAbt, r_A, r_H, pDCt, qx, ux, compute_mult, pi, 1, Pb, memory, work);


	// compute t & lam 
	d_compute_t_lam_new_rhs_mpc_hard_tv(N, nx, nu, nb, idxb, ng, t, lam, lamt, t_inv, ux, pDCt, r_C);

	


	} // end of final kkt solve



