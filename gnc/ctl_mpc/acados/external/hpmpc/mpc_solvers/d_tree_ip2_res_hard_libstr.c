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
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include "../include/tree.h"

#include "../include/lqcp_solvers.h"
#include "../include/mpc_aux.h"
#include "../include/mpc_solvers.h"



#define THR_ITER_REF 1e-5
#define CORRECTOR_LOW 1
#define CORRECTOR_HIGH 1



// work space size 
int d_tree_ip2_res_mpc_hard_work_space_size_bytes_libstr(int Nn, struct node *tree, int *nx, int *nu, int *nb, int *ng)
	{

	int ii;

	int size = 0;

	for(ii=0; ii<Nn; ii++)
		{
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // L
		size += 5*blasfeo_memsize_dvec(nx[ii]); // b, dpi, Pb, res_b, pi_bkp
		size += 4*blasfeo_memsize_dvec(nu[ii]+nx[ii]); // dux, rq, res_rq, ux_bkp
		size += 8*blasfeo_memsize_dvec(2*nb[ii]+2*ng[ii]); // dlam, dt, tinv, lamt, res_d, res_m, t_bkp, lam_bkp
		size += 2*blasfeo_memsize_dvec(nb[ii]+ng[ii]); // Qx, qx
		}

	// residuals work space size
	size += d_tree_res_res_mpc_hard_work_space_size_bytes_libstr(Nn, tree, nx, nu, nb, ng); // TODO

	// riccati work space size
	size += d_tree_back_ric_rec_work_space_size_bytes_libstr(Nn, tree, nx, nu, nb, ng);

	return size;
	}



/* primal-dual interior-point method computing residuals at each iteration, hard constraints, time variant matrices, time variant size (mpc version) */
int d_tree_ip2_res_mpc_hard_libstr(int *kk, int k_max, double mu0, double mu_tol, double alpha_min, int warm_start, double *stat, int Nn, struct node *tree, int *nx, int *nu, int *nb, int **idxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, struct blasfeo_dvec *hsux, int compute_mult, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hst, void *work)
	{

	// adjust number of nodes
	int N = Nn-1;

	// indeces
	int jj, ll, ii, it_ref;
	int nkids, idxkid;



	struct blasfeo_dmat *hsmatdummy;
	struct blasfeo_dvec *hsvecdummy;

	struct blasfeo_dvec hsb[N];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dvec hsQx[N+1];
	struct blasfeo_dvec hsqx[N+1];
	struct blasfeo_dvec hsdux[N+1];
	struct blasfeo_dvec hsdpi[N+1];
	struct blasfeo_dvec hsdt[N+1];
	struct blasfeo_dvec hsdlam[N+1];
	struct blasfeo_dvec hstinv[N+1];
	struct blasfeo_dvec hslamt[N+1];
	struct blasfeo_dvec hsPb[N+1];
	struct blasfeo_dmat hsL[N+1];
	struct blasfeo_dvec hsres_rq[N+1];
	struct blasfeo_dvec hsres_b[N+1];
	struct blasfeo_dvec hsres_d[N+1];
	struct blasfeo_dvec hsres_m[N+1];
	struct blasfeo_dmat hsric_work_mat[2];
	struct blasfeo_dvec hsric_work_vec[1];
	struct blasfeo_dvec hsres_work[2];
	struct blasfeo_dvec hsux_bkp[N+1];
	struct blasfeo_dvec hspi_bkp[N+1];
	struct blasfeo_dvec hst_bkp[N+1];
	struct blasfeo_dvec hslam_bkp[N+1];

	void *d_tree_back_ric_rec_work_space;
	void *d_tree_res_res_mpc_hard_work_space;

	char *c_ptr = work;

	// L
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsL[ii], (void *) c_ptr);
		c_ptr += hsL[ii].memsize;
		}

	// riccati work space
	d_tree_back_ric_rec_work_space = (void *) c_ptr;
	c_ptr += d_tree_back_ric_rec_work_space_size_bytes_libstr(Nn, tree, nx, nu, nb, ng);

	// b as vector
	for(ii=0; ii<Nn; ii++)
		{
		nkids = tree[ii].nkids;
		for(jj=0; jj<nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			blasfeo_create_dvec(nx[idxkid], &hsb[idxkid-1], (void *) c_ptr);
			c_ptr += hsb[idxkid-1].memsize;
			}
		}

	// inputs and states step
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsdux[ii], (void *) c_ptr);
		c_ptr += hsdux[ii].memsize;
		}

	// equality constr multipliers
	for(ii=0; ii<Nn; ii++)
		{
		nkids = tree[ii].nkids;
		for(jj=0; jj<nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			blasfeo_create_dvec(nx[idxkid], &hsdpi[idxkid], (void *) c_ptr);
			c_ptr += hsdpi[idxkid].memsize;
			}
		}
	
	// backup of P*b
	for(ii=0; ii<Nn; ii++)
		{
		nkids = tree[ii].nkids;
		for(jj=0; jj<nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			blasfeo_create_dvec(nx[idxkid], &hsPb[idxkid], (void *) c_ptr);
			c_ptr += hsPb[idxkid].memsize;
			}
		}
	
	// linear part of cost function
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsrq[ii], (void *) c_ptr);
		c_ptr += hsrq[ii].memsize;
		}

	// slack variables, Lagrangian multipliers for inequality constraints and work space
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsdlam[ii], (void *) c_ptr);
		c_ptr += hsdlam[ii].memsize;
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsdt[ii], (void *) c_ptr);
		c_ptr += hsdt[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hstinv[ii], (void *) c_ptr);
		c_ptr += hstinv[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hslamt[ii], (void *) c_ptr);
		c_ptr += hslamt[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nb[ii]+ng[ii], &hsQx[ii], (void *) c_ptr);
		c_ptr += hsQx[ii].memsize;
		blasfeo_create_dvec(nb[ii]+ng[ii], &hsqx[ii], (void *) c_ptr);
		c_ptr += hsqx[ii].memsize;
		}


	// residuals work space
	d_tree_res_res_mpc_hard_work_space = (void *) c_ptr;
	c_ptr += d_tree_res_res_mpc_hard_work_space_size_bytes_libstr(Nn, tree, nx, nu, nb, ng);

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsres_rq[ii], (void *) c_ptr);
		c_ptr += hsres_rq[ii].memsize;
		}

	for(ii=0; ii<Nn; ii++)
		{
		nkids = tree[ii].nkids;
		for(jj=0; jj<nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			blasfeo_create_dvec(nx[idxkid], &hsres_b[idxkid-1], (void *) c_ptr);
			c_ptr += hsres_b[idxkid-1].memsize;
			}
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsres_d[ii], (void *) c_ptr);
		c_ptr += hsres_d[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsres_m[ii], (void *) c_ptr);
		c_ptr += hsres_m[ii].memsize;
		}

	// backup solution
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsux_bkp[ii], (void *) c_ptr);
		c_ptr += hsux_bkp[ii].memsize;
		blasfeo_create_dvec(nx[ii], &hspi_bkp[ii], (void *) c_ptr);
		c_ptr += hspi_bkp[ii].memsize;
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hslam_bkp[ii], (void *) c_ptr);
		c_ptr += hslam_bkp[ii].memsize;
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hst_bkp[ii], (void *) c_ptr);
		c_ptr += hst_bkp[ii].memsize;
		}

	// extract arrays
	double *hpRSQrq[N+1];
	for(jj=0; jj<=N; jj++)
		hpRSQrq[jj] = hsRSQrq[jj].pA;

	// extract b
	for(ii=0; ii<Nn; ii++)
		{
		nkids = tree[ii].nkids;
		for(jj=0; jj<nkids; jj++)
			{
			idxkid = tree[ii].kids[jj];
			blasfeo_drowex(nx[idxkid], 1.0, &hsBAbt[idxkid-1], nu[ii]+nx[ii], 0, &hsb[idxkid-1], 0);
			}
		}
	
	// extract q
	for(jj=0; jj<=N; jj++)
		{
		blasfeo_drowex(nu[jj]+nx[jj], 1.0, &hsRSQrq[jj], nu[jj]+nx[jj], 0, &hsrq[jj], 0);
		}



	double temp0, temp1;
	double alpha, mu, mu_aff, sigma;



	// compute the total number of inequality constraints
	double mu_scal = 0.0; 
	for(jj=0; jj<=N; jj++) 
		mu_scal += 2*nb[jj] + 2*ng[jj];



	// check if there are inequality constraints
	if(mu_scal!=0.0) // there are some constraints
		{
		mu_scal = 1.0 / mu_scal;
		}
	else // call the riccati solver and return
		{
#if 1
		d_tree_back_ric_rec_sv_libstr(Nn, tree, nx, nu, nb, idxb, ng, 0, hsBAbt, hsvecdummy, 0, hsRSQrq, hsvecdummy, hsmatdummy, hsvecdummy, hsvecdummy, hsux, compute_mult, hspi, 0, hsvecdummy, hsL, d_tree_back_ric_rec_work_space);
#else
		d_tree_back_ric_rec_trf_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsRSQrq, hsmatdummy, hsvecdummy, hsL, d_tree_back_ric_rec_work_space);
		d_tree_back_ric_rec_trs_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsb, hsrq, hsmatdummy, hsvecdummy, hsux, compute_mult, hspi, 1, hsPb, hsL, d_tree_back_ric_rec_work_space);
#endif
		// no IPM iterations
		*kk = 0;
		// return success
		return 0;
		}



	// initialize ux & pi & t>0 & lam>0
	d_init_var_mpc_hard_libstr(N, nx, nu, nb, idxb, ng, hsux, hspi, hsDCt, hsd, hst, hslam, mu0, warm_start);



	// compute the duality gap
	mu = mu0;

	// set to zero iteration count
	*kk = 0;	

	// set to zero initial sigma value
	sigma = 0.0;

	// larger than minimum accepted step size
	alpha = 1.0;



	//
	// loop without residuals compuation at early iterations
	//

//	double mu_tol_low = mu_tol;
	double mu_tol_low = mu_tol<THR_ITER_REF ? THR_ITER_REF : mu_tol ;

	while( *kk<k_max && mu>mu_tol_low && alpha>=alpha_min )
		{



		// compute the update of Hessian and gradient from box and general constraints (no residual IPM)
		d_update_hessian_gradient_mpc_hard_libstr(N, nx, nu, nb, ng, hsd, 0.0, hst, hstinv, hslam, hslamt, hsdlam, hsQx, hsqx);



		// compute the search direction: factorize and solve the KKT system
#if 1
		d_tree_back_ric_rec_sv_libstr(Nn, tree, nx, nu, nb, idxb, ng, 0, hsBAbt, hsb, 1, hsRSQrq, hsrq, hsDCt, hsQx, hsqx, hsdux, compute_mult, hsdpi, 1, hsPb, hsL, d_tree_back_ric_rec_work_space);
#else
		d_tree_back_ric_rec_trf_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsRSQrq, hsDCt, hsQx, hsL, d_tree_back_ric_rec_work_space);
		d_tree_back_ric_rec_trs_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsb, hsrq, hsDCt, hsqx, hsdux, compute_mult, hsdpi, 1, hsPb, hsL, d_tree_back_ric_rec_work_space);
#endif



#if CORRECTOR_LOW==1 // IPM2



		// compute t_aff & dlam_aff & dt_aff & alpha
		alpha = 1.0;
		d_compute_alpha_mpc_hard_libstr(N, nx, nu, nb, idxb, ng, &alpha, hst, hsdt, hslam, hsdlam, hslamt, hsdux, hsDCt, hsd);

		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+1] = alpha;
			
		alpha *= 0.995;



		// compute the affine duality gap
		d_compute_mu_mpc_hard_libstr(N, nx, nu, nb, ng, &mu_aff, mu_scal, alpha, hslam, hsdlam, hst, hsdt);

		stat[5*(*kk)+2] = mu_aff;



		// compute sigma
		sigma = mu_aff/mu;
		sigma = sigma*sigma*sigma;



		d_update_gradient_mpc_hard_libstr(N, nx, nu, nb, ng, sigma*mu, hsdt, hsdlam, hstinv, hsqx);



		// solve the system
		d_tree_back_ric_rec_trs_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsb, hsrq, hsDCt, hsqx, hsdux, compute_mult, hsdpi, 0, hsPb, hsL, d_tree_back_ric_rec_work_space);



#endif // end of IPM2



		// compute t & dlam & dt & alpha
		alpha = 1.0;
		d_compute_alpha_mpc_hard_libstr(N, nx, nu, nb, idxb, ng, &alpha, hst, hsdt, hslam, hsdlam, hslamt, hsdux, hsDCt, hsd);



		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+3] = alpha;
			
		alpha *= 0.995;



		// compute step dux, dpi & update ux, pi, lam, t & compute the duality gap mu
		d_backup_update_var_mpc_hard_libstr(N, nx, nu, nb, ng, &mu, mu_scal, alpha, hsux_bkp, hsux, hsdux, hspi_bkp, hspi, hsdpi, hst_bkp, hst, hsdt, hslam_bkp, hslam, hsdlam);



		stat[5*(*kk)+4] = mu;
		


		// increment loop index
		(*kk)++;



		} // end of IP loop
	


	//
	// loop with residuals computation (and iterative refinement ???) for high-accuracy result
	//

	// compute residuals
	d_tree_res_res_mpc_hard_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hslam, hst, hsres_rq, hsres_b, hsres_d, hsres_m, &mu, d_tree_res_res_mpc_hard_work_space);



	// IP loop		
	while( *kk<k_max && mu>mu_tol && alpha>=alpha_min ) // XXX exit conditions on residuals???
		{



		// compute the update of Hessian and gradient from box and general constraints (residual IPM)
		d_update_hessian_gradient_res_mpc_hard_libstr(N, nx, nu, nb, ng, hsres_d, hsres_m, hst, hslam, hstinv, hsQx, hsqx);



		// compute the search direction: factorize and solve the KKT system
#if 1
		d_tree_back_ric_rec_sv_libstr(Nn, tree, nx, nu, nb, idxb, ng, 1, hsBAbt, hsres_b, 1, hsRSQrq, hsres_rq, hsDCt, hsQx, hsqx, hsdux, compute_mult, hsdpi, 1, hsPb, hsL, d_tree_back_ric_rec_work_space);
#else
		d_tree_back_ric_rec_trf_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsRSQrq, hsDCt, hsQx, hsL, d_tree_back_ric_rec_work_space);
		d_tree_back_ric_rec_trs_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsres_b, hsres_rq, hsDCt, hsqx, hsdux, compute_mult, hsdpi, 1, hsPb, hsL, d_tree_back_ric_rec_work_space);
#endif


		
#if CORRECTOR_HIGH==1 // IPM2



		// compute t_aff & dlam_aff & dt_aff & alpha
		alpha = 1.0;
		d_compute_alpha_res_mpc_hard_libstr(N, nx, nu, nb, idxb, ng, hsdux, hst, hstinv, hslam, hsDCt, hsres_d, hsres_m, hsdt, hsdlam, &alpha);

		

		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+1] = alpha;
			
		alpha *= 0.995;



		// compute the affine duality gap
		d_compute_mu_mpc_hard_libstr(N, nx, nu, nb, ng, &mu_aff, mu_scal, alpha, hslam, hsdlam, hst, hsdt);

		stat[5*(*kk)+2] = mu_aff;



		// compute sigma
		sigma = mu_aff/mu;
		sigma = sigma*sigma*sigma;



		// update res_m
		d_compute_centering_correction_res_mpc_hard_libstr(N, nb, ng, sigma*mu, hsdt, hsdlam, hsres_m);



		// update gradient
		d_update_gradient_res_mpc_hard_libstr(N, nx, nu, nb, ng, hsres_d, hsres_m, hslam, hstinv, hsqx);



		// solve the KKT system
		d_tree_back_ric_rec_trs_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsres_b, hsres_rq, hsDCt, hsqx, hsdux, compute_mult, hsdpi, 0, hsPb, hsL, d_tree_back_ric_rec_work_space);



#endif // end of IPM2



		// compute t & dlam & dt & alpha
		alpha = 1.0;
		d_compute_alpha_res_mpc_hard_libstr(N, nx, nu, nb, idxb, ng, hsdux, hst, hstinv, hslam, hsDCt, hsres_d, hsres_m, hsdt, hsdlam, &alpha);



		stat[5*(*kk)] = sigma;
		stat[5*(*kk)+3] = alpha;
			
		alpha *= 0.995;



		// backup & update x, u, pi, lam, t 
		d_backup_update_var_res_mpc_hard_libstr(N, nx, nu, nb, ng, alpha, hsux_bkp, hsux, hsdux, hspi_bkp, hspi, hsdpi, hst_bkp, hst, hsdt, hslam_bkp, hslam, hsdlam);



		// restore dynamics
		for(ii=0; ii<Nn; ii++)
			{
			nkids = tree[ii].nkids;
			for(jj=0; jj<nkids; jj++)
				{
				idxkid = tree[ii].kids[jj];
				blasfeo_drowin(nx[idxkid], 1.0, &hsb[idxkid-1], 0, &hsBAbt[idxkid-1], nu[ii]+nx[ii], 0);
				}
			}



		// compute residuals
		d_tree_res_res_mpc_hard_libstr(Nn, tree, nx, nu, nb, idxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hslam, hst, hsres_rq, hsres_b, hsres_d, hsres_m, &mu, d_tree_res_res_mpc_hard_work_space);



		stat[5*(*kk)+4] = mu;



		// increment loop index
		(*kk)++;



		} // end of IP loop
	


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



#endif
